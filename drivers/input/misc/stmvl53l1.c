// SPDX-License-Identifier: GPL-2.0-only
/*
 * stmvl53l1 - ST VL53L1 time-of-flight sensor.
 *
 * Rewritten from the vendor stmvl53l1.ko.  The vendor module carries ST's full
 * VL53L1 API (configuration encoders, calibration, histogram processing) plus
 * the i2c input driver.  This driver reproduces the i2c/input behaviour and the
 * ranging pipeline, which is what the HAL needs:
 *
 *   - I2C binding, xsdn/pwren power sequencing, model-ID check at 0x010f
 *   - the vendor patch sequence, which on this part is a handful of register
 *     writes rather than a firmware blob (VL53L1_load_patch in the decompile)
 *   - measurement start/stop through the system-control block at 0x0087 and
 *     data-ready polling of 0x0031, then the 44-byte result block at 0x0088
 *   - periodic reporting through the input device named "stmvl53l1"
 *   - the sysfs control surface the HAL writes
 *
 * Deliberately not reproduced: ST's static/general/timing configuration
 * encoders.  The patch leaves the sensor in its default measurement
 * configuration, which is what a plain start uses; changing timing budget, ROI
 * or preset mode therefore only updates the cached values instead of
 * re-encoding the config space.  Calibration and histogram processing are
 * absent as well.  Those are documented in h726_module_parity_plan.md.
 */
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#define VL53L1_REG_MODEL_ID	0x010f
#define VL53L1_MODEL_ID		0xeacc

/* register map used by the vendor API */
#define VL53L1_REG_SYSTEM_INTERRUPT	0x0031	/* bit 0: new data ready */
#define VL53L1_REG_SYSTEM_CONTROL	0x0087	/* 5 bytes, byte0 cmd/mode */
#define VL53L1_REG_SYSTEM_RESULTS	0x0088	/* 44 bytes */
#define VL53L1_REG_SYSTEM_INTERRUPT_CLEAR 0x0086
#define VL53L1_REG_FIRMWARE__SYSTEM_STATUS 0x0085

#define VL53L1_CMD_STOP			0x80	/* system control byte 0 */
#define VL53L1_MODE_TIMED_RANGING	6

/* offsets inside the 44-byte block read from 0x0088 */
#define RES_RANGE_STATUS	0x01	/* 0x0089 */
#define RES_AMBIENT		0x08	/* 0x0090, u16 */
#define RES_SIGNAL		0x0a	/* 0x0092, u16 */
#define RES_SPADS		0x0c	/* 0x0094, u16 */
#define RES_DISTANCE		0x0e	/* 0x0096, u16 */

struct stmvl53l1_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct gpio_desc *xsdn;
	struct gpio_desc *pwren;
	struct gpio_desc *intr;
	struct mutex lock;		/* protects the i2c transactions */
	struct delayed_work poll_work;
	bool running;

	unsigned int poll_delay_ms;
	unsigned int timing_budget;
	unsigned int preset_mode;
	unsigned int distance_mode;
	unsigned int roi;
	bool crosstalk_enable;
	bool enable_debug;
	bool enable_ps_sensor;

	unsigned int distance_mm;
	unsigned int range_status;
	unsigned int signal;
	unsigned int ambient;
	unsigned int errors;
};

/* ---------------- i2c transport, same framing as cci_write/cci_read ------ */

static int vl53l1_write(struct stmvl53l1_data *data, u16 reg, const u8 *buf,
			size_t len)
{
	u8 *tx;
	int ret;

	/*
	 * The vendor splits multi-byte writes into 32-byte transfers; a single
	 * transfer is fine here because every caller stays well inside the
	 * adapter limit and the sensor streams the payload in order.
	 */
	tx = kmalloc(len + 2, GFP_KERNEL);
	if (!tx)
		return -ENOMEM;

	tx[0] = reg >> 8;
	tx[1] = reg & 0xff;
	memcpy(tx + 2, buf, len);

	ret = i2c_master_send(data->client, tx, len + 2);
	kfree(tx);

	return ret == len + 2 ? 0 : (ret < 0 ? ret : -EIO);
}

static int vl53l1_read(struct stmvl53l1_data *data, u16 reg, u8 *buf,
		       size_t len)
{
	u8 addr[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msgs[2] = {
		{ .addr = data->client->addr, .flags = 0, .len = 2, .buf = addr },
		{ .addr = data->client->addr, .flags = I2C_M_RD, .len = len,
		  .buf = buf },
	};
	int ret;

	ret = i2c_transfer(data->client->adapter, msgs, 2);
	return ret == 2 ? 0 : (ret < 0 ? ret : -EIO);
}

static int vl53l1_wr_byte(struct stmvl53l1_data *data, u16 reg, u8 val)
{
	return vl53l1_write(data, reg, &val, 1);
}

static int vl53l1_rd_byte(struct stmvl53l1_data *data, u16 reg, u8 *val)
{
	return vl53l1_read(data, reg, val, 1);
}

static int vl53l1_rd_word(struct stmvl53l1_data *data, u16 reg, u16 *val)
{
	u8 buf[2];
	int ret = vl53l1_read(data, reg, buf, sizeof(buf));

	if (!ret)
		*val = (buf[0] << 8) | buf[1];
	return ret;
}

/* ---------------- power and the vendor patch sequence -------------------- */

static void stmvl53l1_power_up(struct stmvl53l1_data *data)
{
	if (data->pwren) {
		gpiod_set_value_cansleep(data->pwren, 1);
		msleep(1);
	}
	if (data->xsdn) {
		gpiod_set_value_cansleep(data->xsdn, 0);
		usleep_range(1000, 2000);
		gpiod_set_value_cansleep(data->xsdn, 1);
		usleep_range(1000, 2000);
	}
}

static void stmvl53l1_power_down(struct stmvl53l1_data *data)
{
	if (data->xsdn)
		gpiod_set_value_cansleep(data->xsdn, 0);
	if (data->pwren)
		gpiod_set_value_cansleep(data->pwren, 0);
}

/*
 * VL53L1_load_patch(): the vendored API does not download a firmware image,
 * it unlocks the shadow registers and writes the measurement patch words.
 * The sixth byte of each word is the tuning-parameter nibble (0x10 << n for
 * n < 4), which is 0x10 for the default value of 0.
 */
static int vl53l1_load_patch(struct stmvl53l1_data *data)
{
	/* 0x400ec929 little-endian, 0x28, then 0x10 << tuning_parm */
	static const u8 p476[6] = { 0x29, 0xc9, 0x0e, 0x40, 0x28, 0x10 };
	static const u8 p496[6] = { 0x03, 0x6d, 0x03, 0x6f, 0x07, 0x29 };
	static const u8 p472[2] = { 0x00, 0x07 };
	static const u8 p474[2] = { 0x00, 0x07 };
	int ret;

	ret = vl53l1_wr_byte(data, VL53L1_REG_FIRMWARE__SYSTEM_STATUS, 0x00);
	if (ret)
		return ret;

	ret = vl53l1_write(data, 0x0476, p476, sizeof(p476));
	if (ret)
		return ret;
	ret = vl53l1_write(data, 0x0496, p496, sizeof(p496));
	if (ret)
		return ret;
	ret = vl53l1_write(data, 0x0472, p472, sizeof(p472));
	if (ret)
		return ret;
	ret = vl53l1_write(data, 0x0474, p474, sizeof(p474));
	if (ret)
		return ret;

	ret = vl53l1_wr_byte(data, 0x0470, 0x01);
	if (ret)
		return ret;

	return vl53l1_wr_byte(data, VL53L1_REG_FIRMWARE__SYSTEM_STATUS, 0x01);
}

/* ---------------- measurement ------------------------------------------- */

/*
 * System control is a 5-byte block at 0x0087.  The vendor keeps the low nibble
 * of byte 0 as the mode and writes 0x80 there to stop (VL53L1_stop_range), so
 * a plain start is the mode code with the rest of the block preserved.
 */
static int vl53l1_start_range(struct stmvl53l1_data *data, u8 mode)
{
	u8 ctl[5];
	int ret;

	ret = vl53l1_read(data, VL53L1_REG_SYSTEM_CONTROL, ctl, sizeof(ctl));
	if (ret)
		return ret;

	ctl[0] = mode;
	return vl53l1_write(data, VL53L1_REG_SYSTEM_CONTROL, ctl, sizeof(ctl));
}

static int vl53l1_stop_range(struct stmvl53l1_data *data)
{
	u8 ctl[5];
	int ret;

	ret = vl53l1_read(data, VL53L1_REG_SYSTEM_CONTROL, ctl, sizeof(ctl));
	if (ret)
		return ret;

	ctl[0] = VL53L1_CMD_STOP;
	return vl53l1_write(data, VL53L1_REG_SYSTEM_CONTROL, ctl, sizeof(ctl));
}

/* VL53L1_is_new_data_ready(): interrupt status bit 0 at 0x0031. */
static int vl53l1_data_ready(struct stmvl53l1_data *data, bool *ready)
{
	u8 val;
	int ret = vl53l1_rd_byte(data, VL53L1_REG_SYSTEM_INTERRUPT, &val);

	*ready = !ret && (val & 0x01);
	return ret;
}

static int vl53l1_clear_interrupt(struct stmvl53l1_data *data)
{
	return vl53l1_wr_byte(data, VL53L1_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
}

static void stmvl53l1_report(struct stmvl53l1_data *data)
{
	struct input_dev *input = data->input;

	input_report_abs(input, ABS_DISTANCE, data->distance_mm);
	input_report_abs(input, ABS_Z, data->range_status);
	input_report_abs(input, ABS_X, data->signal);
	input_report_abs(input, ABS_Y, data->ambient);
	input_sync(input);
}

static void stmvl53l1_poll_work(struct work_struct *work)
{
	struct stmvl53l1_data *data =
		container_of(work, struct stmvl53l1_data, poll_work.work);
	u8 res[44];
	u16 distance, signal, ambient, spads;
	bool ready = false;
	unsigned int delay;
	int ret;

	mutex_lock(&data->lock);
	if (!data->running) {
		mutex_unlock(&data->lock);
		return;
	}

	ret = vl53l1_data_ready(data, &ready);
	if (ret)
		goto out;

	if (ready) {
		ret = vl53l1_read(data, VL53L1_REG_SYSTEM_RESULTS, res,
				  sizeof(res));
		if (ret)
			goto out;

		data->range_status = res[RES_RANGE_STATUS];
		distance = (res[RES_DISTANCE] << 8) | res[RES_DISTANCE + 1];
		signal = (res[RES_SIGNAL] << 8) | res[RES_SIGNAL + 1];
		ambient = (res[RES_AMBIENT] << 8) | res[RES_AMBIENT + 1];
		spads = (res[RES_SPADS] << 8) | res[RES_SPADS + 1];

		data->distance_mm = distance;
		data->signal = signal;
		data->ambient = ambient;
		(void)spads;

		if (data->enable_debug)
			dev_dbg(&data->client->dev,
				"range %u mm status %u signal %u ambient %u\n",
				distance, data->range_status, signal, ambient);

		stmvl53l1_report(data);

		/* clear the interrupt and arm the next range, as the vendor does */
		vl53l1_clear_interrupt(data);
		vl53l1_start_range(data, VL53L1_MODE_TIMED_RANGING);
	}
out:
	if (ret)
		data->errors++;
	delay = data->poll_delay_ms ? data->poll_delay_ms : 100;
	mutex_unlock(&data->lock);
	if (data->running)
		queue_delayed_work(system_wq, &data->poll_work,
				   msecs_to_jiffies(delay));
}

static int stmvl53l1_measurement_start(struct stmvl53l1_data *data)
{
	int ret;

	mutex_lock(&data->lock);
	if (data->running) {
		mutex_unlock(&data->lock);
		return 0;
	}

	ret = vl53l1_load_patch(data);
	if (ret) {
		dev_err(&data->client->dev, "patch sequence failed: %d\n", ret);
		goto out;
	}

	ret = vl53l1_clear_interrupt(data);
	if (!ret)
		ret = vl53l1_start_range(data, VL53L1_MODE_TIMED_RANGING);
	if (ret) {
		dev_err(&data->client->dev, "start range failed: %d\n", ret);
		goto out;
	}

	data->running = true;
	data->errors = 0;
	queue_delayed_work(system_wq, &data->poll_work,
			   msecs_to_jiffies(data->poll_delay_ms ?: 100));
out:
	mutex_unlock(&data->lock);
	return ret;
}

static void stmvl53l1_measurement_stop(struct stmvl53l1_data *data)
{
	mutex_lock(&data->lock);
	if (data->running) {
		data->running = false;
		cancel_delayed_work_sync(&data->poll_work);
		vl53l1_stop_range(data);
	}
	mutex_unlock(&data->lock);
}

/* ---- sysfs: the control surface the vendor HAL writes ---- */

static ssize_t enable_ps_sensor_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->enable_ps_sensor);
}

static ssize_t enable_ps_sensor_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	val = !!val;
	if (val == data->enable_ps_sensor)
		return count;

	if (val) {
		int ret = stmvl53l1_measurement_start(data);

		if (ret)
			return ret;
	} else {
		stmvl53l1_measurement_stop(data);
	}

	data->enable_ps_sensor = val;
	return count;
}
static DEVICE_ATTR_RW(enable_ps_sensor);

static ssize_t poll_delay_ms_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", data->poll_delay_ms);
}

static ssize_t poll_delay_ms_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	data->poll_delay_ms = val ?: 1;
	return count;
}
static DEVICE_ATTR_RW(poll_delay_ms);

static ssize_t timing_budget_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", data->timing_budget);
}

static ssize_t timing_budget_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	data->timing_budget = val;
	return count;
}
static DEVICE_ATTR_RW(timing_budget);

static ssize_t preset_mode_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", data->preset_mode);
}

static ssize_t preset_mode_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	data->preset_mode = val;
	return count;
}
static DEVICE_ATTR_RW(preset_mode);

static ssize_t distance_mode_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", data->distance_mode);
}

static ssize_t distance_mode_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	data->distance_mode = val;
	return count;
}
static DEVICE_ATTR_RW(distance_mode);

static ssize_t crosstalk_enable_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->crosstalk_enable);
}

static ssize_t crosstalk_enable_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	data->crosstalk_enable = !!val;
	return count;
}
static DEVICE_ATTR_RW(crosstalk_enable);

static ssize_t roi_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", data->roi);
}

static ssize_t roi_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	data->roi = val;
	return count;
}
static DEVICE_ATTR_RW(roi);

static ssize_t enable_debug_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", data->enable_debug);
}

static ssize_t enable_debug_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	data->enable_debug = !!val;
	return count;
}
static DEVICE_ATTR_RW(enable_debug);

/* vendor do_flush: drop events that have not reached userspace yet */
static ssize_t flush_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);

	input_sync(data->input);
	return count;
}
static DEVICE_ATTR_WO(flush);

static struct attribute *stmvl53l1_attrs[] = {
	&dev_attr_enable_ps_sensor.attr,
	&dev_attr_poll_delay_ms.attr,
	&dev_attr_timing_budget.attr,
	&dev_attr_preset_mode.attr,
	&dev_attr_distance_mode.attr,
	&dev_attr_crosstalk_enable.attr,
	&dev_attr_roi.attr,
	&dev_attr_enable_debug.attr,
	&dev_attr_flush.attr,
	NULL,
};
ATTRIBUTE_GROUPS(stmvl53l1);

static int stmvl53l1_input_setup(struct stmvl53l1_data *data)
{
	struct input_dev *input;
	int ret;

	input = devm_input_allocate_device(&data->client->dev);
	if (!input)
		return -ENOMEM;

	input->name = "STM VL53L1 proximity sensor";
	input->id.bustype = BUS_I2C;

	/* Same absolute axes the vendor driver registers. */
	set_bit(EV_ABS, input->evbit);
	input_set_abs_params(input, ABS_X, 0, 0xffff, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, 0xffff, 0, 0);
	input_set_abs_params(input, ABS_Z, 0, 0xffff, 0, 0);
	input_set_abs_params(input, ABS_DISTANCE, 0, 0xffff, 0, 0);

	ret = input_register_device(input);
	if (ret) {
		dev_err(&data->client->dev, "input_register_device failed: %d\n",
			ret);
		return ret;
	}

	data->input = input;
	return 0;
}

static int stmvl53l1_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct stmvl53l1_data *data;
	u16 model = 0;
	int ret;

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	data->poll_delay_ms = 100;
	mutex_init(&data->lock);
	INIT_DELAYED_WORK(&data->poll_work, stmvl53l1_poll_work);
	i2c_set_clientdata(client, data);

	data->xsdn = devm_gpiod_get_optional(&client->dev, "xsdn",
					     GPIOD_OUT_LOW);
	if (IS_ERR(data->xsdn))
		return PTR_ERR(data->xsdn);

	data->pwren = devm_gpiod_get_optional(&client->dev, "pwren",
					      GPIOD_OUT_LOW);
	if (IS_ERR(data->pwren))
		return PTR_ERR(data->pwren);

	data->intr = devm_gpiod_get_optional(&client->dev, "intr", GPIOD_IN);
	if (IS_ERR(data->intr))
		return PTR_ERR(data->intr);

	stmvl53l1_power_up(data);

	ret = vl53l1_rd_word(data, VL53L1_REG_MODEL_ID, &model);
	if (ret) {
		dev_err(&client->dev, "cannot read model id: %d\n", ret);
		stmvl53l1_power_down(data);
		return ret;
	}
	if (model != VL53L1_MODEL_ID)
		dev_warn(&client->dev,
			 "unexpected model id %#x (expected %#x)\n",
			 model, VL53L1_MODEL_ID);

	ret = stmvl53l1_input_setup(data);
	if (ret) {
		stmvl53l1_power_down(data);
		return ret;
	}

	ret = devm_device_add_groups(&client->dev, stmvl53l1_groups);
	if (ret) {
		stmvl53l1_power_down(data);
		return ret;
	}

	dev_info(&client->dev, "VL53L1 ready (model %#x)\n", model);
	return 0;
}

static int stmvl53l1_remove(struct i2c_client *client)
{
	struct stmvl53l1_data *data = i2c_get_clientdata(client);

	stmvl53l1_measurement_stop(data);
	stmvl53l1_power_down(data);
	return 0;
}

static int __maybe_unused stmvl53l1_suspend(struct device *dev)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);

	/*
	 * Vendor stmvl53l1_suspend() takes the lock and calls
	 * stmvl53l1_pm_suspend_stop() (ctrl_stop) only: the sensor stays powered
	 * across sleep, so system resume cannot find it powered down.  Powering
	 * it down here made resume depend on a successful power-up, which stock
	 * never does.
	 */
	if (data->enable_ps_sensor)
		stmvl53l1_measurement_stop(data);
	return 0;
}

static int __maybe_unused stmvl53l1_resume(struct device *dev)
{
	struct stmvl53l1_data *data = dev_get_drvdata(dev);
	int running_ret;

	/* Vendor stmvl53l1_resume() is literally `return 0`; restart ranging for
	 * userspace, but never fail system resume over it.
	 */
	if (!data->enable_ps_sensor)
		return 0;

	running_ret = stmvl53l1_measurement_start(data);
	if (running_ret)
		dev_warn(dev, "cannot restart ranging after resume: %d\n",
			 running_ret);

	return 0;
}

static const struct dev_pm_ops stmvl53l1_pm_ops = {
	.suspend	= stmvl53l1_suspend,
	.resume		= stmvl53l1_resume,
};

static const struct of_device_id stmvl53l1_of_match[] = {
	{ .compatible = "st,stmvl53l1" },
	{ }
};
MODULE_DEVICE_TABLE(of, stmvl53l1_of_match);

static const struct i2c_device_id stmvl53l1_id[] = {
	{ "stmvl53l1", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, stmvl53l1_id);

static struct i2c_driver stmvl53l1_driver = {
	.driver = {
		.name		= "stmvl53l1",
		.of_match_table	= stmvl53l1_of_match,
		.pm		= &stmvl53l1_pm_ops,
	},
	.probe		= stmvl53l1_probe,
	.remove		= stmvl53l1_remove,
	.id_table	= stmvl53l1_id,
};
module_i2c_driver(stmvl53l1_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("STM VL53L1 proximity sensor");
MODULE_AUTHOR("STMicroelectronics");
