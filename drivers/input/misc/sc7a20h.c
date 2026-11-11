// SPDX-License-Identifier: GPL-2.0-only
/*
 * Silan SC7A20H 3-axis accelerometer.
 *
 * Rewrite of the vendor sc7a20h.ko (Allwinner gsensor framework, module alias
 * "i2c:sc7a20h", compatible "allwinner,sc7a20h"), recovered from
 * dec_mod/sc7a20h.c and the module's own strings.
 *
 * What it reproduces:
 *  - probe-time chip identification: register 0x0f must read 0x11 and 0x70
 *    must read 0x28, otherwise probe returns -ENODEV (the vendor did the same
 *    check and never bound the wrong part);
 *  - the vendor register sequence (CTRL1 0x20 = 0x37, CTRL2 0x1f = 0x01,
 *    CTRL4 0x23 = 0x98 for 4 g full scale, INT1_CFG 0x21 = 0x73,
 *    MODE 0x30 = 0x2a, 0x32 = 0x02, 0x33 = 0x00, INT_MAP1 0x25 = 0x03,
 *    CTRL3 0x22 = 0x10);
 *  - the report path: wait for status 0x27 bit 3, read 0x29/0x2b/0x2d, and
 *    report EV_ABS ABS_X/Y/Z as signed bytes, with Z assembled from the high
 *    byte and shifted right by 3.  That yields the 32 counts per g the Android
 *    sensors HAL expects for this part (12-bit left-justified data over 4 g,
 *    high byte only: 2048 / 4 / 16 = 32);
 *  - the "all axes zero" recovery: power down, re-run the init sequence and
 *    power back up, which is what the vendor did in the same spot;
 *  - sysfs attributes on the input device: enable, delay, register (the vendor
 *    dumps 0x0f and 0x10..0x59, and accepts 0xRR=0xVV writes).
 *
 * Board note: the HY300 tree has no "allwinner,sc7a20h" node.  Its two
 * accelerometer nodes are stk,stk83xx at 0x18 and 0x19, and stk83xx.c already
 * identifies id 0x11 as "stk8ba58/sc7a20h" and drives that part (with tilt,
 * swipe and shake on top).  This driver therefore stays inert on the current
 * device tree and only binds when a node with its compatible is added; it must
 * never be pointed at an address an stk,stk83xx node already owns.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

/* Register map, as used by the vendor probe and report path. */
#define SC7A20H_REG_DEVID	0x0f
#define SC7A20H_REG_CTRL2	0x1f
#define SC7A20H_REG_CTRL1	0x20
#define SC7A20H_REG_INT1_CFG	0x21
#define SC7A20H_REG_CTRL3	0x22
#define SC7A20H_REG_CTRL4	0x23
#define SC7A20H_REG_INT_MAP1	0x25
#define SC7A20H_REG_STATUS	0x27
#define SC7A20H_REG_XOUT_H	0x29
#define SC7A20H_REG_YOUT_H	0x2b
#define SC7A20H_REG_ZOUT_H	0x2d
#define SC7A20H_REG_MODE	0x30
#define SC7A20H_REG_MODE2	0x32
#define SC7A20H_REG_MODE3	0x33
#define SC7A20H_REG_ID2		0x70

/* Register dump range of the vendor's "register" attribute. */
#define SC7A20H_DUMP_FIRST	0x0f
#define SC7A20H_DUMP_LAST	0x59

#define SC7A20H_DEVID_VALUE	0x11
#define SC7A20H_ID2_VALUE	0x28

#define SC7A20H_DATA_RDY	BIT(3)

#define SC7A20H_POLL_DEFAULT_MS	50
#define SC7A20H_POLL_MIN_MS	10
#define SC7A20H_POLL_MAX_MS	1000

/* The vendor keeps only the high byte: 4 g, 12-bit data, 32 counts per g. */
#define SC7A20H_Z_SHIFT		3

struct sc7a20h {
	struct i2c_client *client;
	struct input_dev *idev;
	struct mutex lock;
	struct hrtimer timer;
	struct work_struct work;
	u32 delay_ms;
	bool enabled;
};

static int sc7a20h_read(struct sc7a20h *acc, u8 reg)
{
	return i2c_smbus_read_byte_data(acc->client, reg);
}

static int sc7a20h_write(struct sc7a20h *acc, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(acc->client, reg, val);
}

/* The exact sequence the vendor probe wrote before registering the device. */
static void sc7a20h_init_chip(struct sc7a20h *acc)
{
	sc7a20h_write(acc, SC7A20H_REG_CTRL1, 0x37);
	sc7a20h_write(acc, SC7A20H_REG_CTRL2, 0x01);
	sc7a20h_write(acc, SC7A20H_REG_CTRL4, 0x98);
	sc7a20h_write(acc, SC7A20H_REG_INT1_CFG, 0x73);
	sc7a20h_write(acc, SC7A20H_REG_MODE, 0x2a);
	sc7a20h_write(acc, SC7A20H_REG_MODE2, 0x02);
	sc7a20h_write(acc, SC7A20H_REG_MODE3, 0x00);
	sc7a20h_write(acc, SC7A20H_REG_INT_MAP1, 0x03);
	sc7a20h_write(acc, SC7A20H_REG_CTRL3, 0x10);
}

static void sc7a20h_set_active(struct sc7a20h *acc, bool on)
{
	int val;

	if (!on) {
		sc7a20h_write(acc, SC7A20H_REG_CTRL1, 0x00);
		return;
	}

	sc7a20h_write(acc, SC7A20H_REG_CTRL1, 0x47);

	/* Re-force the 4 g range, as the vendor's enable path did. */
	val = sc7a20h_read(acc, SC7A20H_REG_CTRL4);
	if (val >= 0)
		sc7a20h_write(acc, SC7A20H_REG_CTRL4, (val & 0xcf) | 0x10);
}

static void sc7a20h_report(struct sc7a20h *acc)
{
	int xh, yh, zh, sts;
	int i;
	s8 x, y;
	s16 z;

	for (i = 0; i < 40; i++) {
		sts = sc7a20h_read(acc, SC7A20H_REG_STATUS);
		if (sts < 0)
			return;
		if (sts & SC7A20H_DATA_RDY)
			break;
		usleep_range(500, 1000);
	}

	xh = sc7a20h_read(acc, SC7A20H_REG_XOUT_H);
	yh = sc7a20h_read(acc, SC7A20H_REG_YOUT_H);
	zh = sc7a20h_read(acc, SC7A20H_REG_ZOUT_H);
	if (xh < 0 || yh < 0 || zh < 0)
		return;

	x = (s8)xh;
	y = (s8)yh;
	z = ((s16)(zh << 8)) >> SC7A20H_Z_SHIFT;

	/*
	 * All-zero is the vendor's "sensor wedged" case: power down, re-init,
	 * power back up rather than report a flat sample.
	 */
	if (!x && !y && !z) {
		dev_warn_ratelimited(&acc->client->dev,
				     "all axes read zero, re-initialising\n");
		sc7a20h_write(acc, SC7A20H_REG_CTRL1, 0x00);
		msleep(50);
		sc7a20h_init_chip(acc);
		sc7a20h_set_active(acc, true);
		return;
	}

	input_report_abs(acc->idev, ABS_X, x);
	input_report_abs(acc->idev, ABS_Y, y);
	input_report_abs(acc->idev, ABS_Z, z);
	input_sync(acc->idev);
}

static void sc7a20h_work(struct work_struct *work)
{
	struct sc7a20h *acc = container_of(work, struct sc7a20h, work);

	mutex_lock(&acc->lock);
	if (acc->enabled)
		sc7a20h_report(acc);
	mutex_unlock(&acc->lock);
}

static enum hrtimer_restart sc7a20h_timer(struct hrtimer *timer)
{
	struct sc7a20h *acc = container_of(timer, struct sc7a20h, timer);

	schedule_work(&acc->work);
	hrtimer_forward_now(timer, ms_to_ktime(acc->delay_ms));

	return HRTIMER_RESTART;
}

static void sc7a20h_stop(struct sc7a20h *acc)
{
	hrtimer_cancel(&acc->timer);
	cancel_work_sync(&acc->work);
}

static void sc7a20h_start(struct sc7a20h *acc)
{
	if (!acc->enabled)
		return;

	hrtimer_start(&acc->timer, ms_to_ktime(acc->delay_ms),
		      HRTIMER_MODE_REL);
}

static void sc7a20h_set_enabled(struct sc7a20h *acc, bool enable)
{
	if (!enable) {
		sc7a20h_stop(acc);
		mutex_lock(&acc->lock);
		acc->enabled = false;
		sc7a20h_set_active(acc, false);
		mutex_unlock(&acc->lock);
		return;
	}

	mutex_lock(&acc->lock);
	sc7a20h_set_active(acc, true);
	acc->enabled = true;
	mutex_unlock(&acc->lock);

	sc7a20h_start(acc);
}

/*
 * The attributes live on the input device, matching the vendor (they appeared
 * under /sys/class/input/inputN/), so driver data is reached through it.
 */
static struct sc7a20h *dev_to_acc(struct device *dev)
{
	return input_get_drvdata(container_of(dev, struct input_dev, dev));
}

static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct sc7a20h *acc = dev_to_acc(dev);

	return sysfs_emit(buf, "%d\n", acc->enabled);
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct sc7a20h *acc = dev_to_acc(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	sc7a20h_set_enabled(acc, !!val);

	return count;
}

static ssize_t delay_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct sc7a20h *acc = dev_to_acc(dev);

	return sysfs_emit(buf, "%u\n", acc->delay_ms);
}

static ssize_t delay_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct sc7a20h *acc = dev_to_acc(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	val = clamp(val, SC7A20H_POLL_MIN_MS, SC7A20H_POLL_MAX_MS);

	mutex_lock(&acc->lock);
	acc->delay_ms = val;
	mutex_unlock(&acc->lock);

	if (acc->enabled) {
		hrtimer_cancel(&acc->timer);
		sc7a20h_start(acc);
	}

	return count;
}

static ssize_t register_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct sc7a20h *acc = dev_to_acc(dev);
	ssize_t len = 0;
	int reg, val;

	for (reg = SC7A20H_DUMP_FIRST; reg <= SC7A20H_DUMP_LAST; reg++) {
		val = sc7a20h_read(acc, reg);
		if (val < 0)
			continue;
		len += sysfs_emit_at(buf, len, "0x%x: 0x%x\n", reg, val);
	}

	return len;
}

static ssize_t register_store(struct device *dev,
			      struct device_attribute *attr, const char *buf,
			      size_t count)
{
	struct sc7a20h *acc = dev_to_acc(dev);
	unsigned int reg, val;

	if (sscanf(buf, "0x%x=0x%x", &reg, &val) != 2)
		return -EINVAL;
	if (reg > 0xff || val > 0xff)
		return -EINVAL;

	if (sc7a20h_write(acc, reg, val))
		return -EIO;

	return count;
}

static DEVICE_ATTR_RW(enable);
static DEVICE_ATTR_RW(delay);
static DEVICE_ATTR_RW(register);

static struct attribute *sc7a20h_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_register.attr,
	NULL,
};
ATTRIBUTE_GROUPS(sc7a20h);

static int sc7a20h_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct sc7a20h *acc;
	int devid, id2, ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "adapter lacks SMBus byte-data support\n");

	acc = devm_kzalloc(dev, sizeof(*acc), GFP_KERNEL);
	if (!acc)
		return -ENOMEM;

	acc->client = client;
	acc->delay_ms = SC7A20H_POLL_DEFAULT_MS;
	acc->enabled = true;

	devid = sc7a20h_read(acc, SC7A20H_REG_DEVID);
	id2 = sc7a20h_read(acc, SC7A20H_REG_ID2);
	if (devid < 0)
		return devid;
	if (id2 < 0)
		return id2;

	if (devid != SC7A20H_DEVID_VALUE || id2 != SC7A20H_ID2_VALUE)
		return dev_err_probe(dev, -ENODEV,
				     "not an SC7A20H (0x0f=0x%02x, 0x70=0x%02x)\n",
				     devid, id2);

	mutex_init(&acc->lock);

	sc7a20h_init_chip(acc);
	msleep(5);
	sc7a20h_set_active(acc, true);

	acc->idev = devm_input_allocate_device(dev);
	if (!acc->idev)
		return -ENOMEM;

	acc->idev->name = "sc7a20h";
	acc->idev->phys = "sc7a20h/input0";
	acc->idev->id.bustype = BUS_I2C;
	acc->idev->id.vendor = 0x0001;
	acc->idev->id.product = 0x0001;
	acc->idev->id.version = 0x0001;
	acc->idev->dev.parent = dev;

	input_set_capability(acc->idev, EV_ABS, ABS_X);
	input_set_capability(acc->idev, EV_ABS, ABS_Y);
	input_set_capability(acc->idev, EV_ABS, ABS_Z);
	input_set_abs_params(acc->idev, ABS_X, -2048, 2047, 0, 0);
	input_set_abs_params(acc->idev, ABS_Y, -2048, 2047, 0, 0);
	input_set_abs_params(acc->idev, ABS_Z, -2048, 2047, 0, 0);

	hrtimer_init(&acc->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	acc->timer.function = sc7a20h_timer;
	INIT_WORK(&acc->work, sc7a20h_work);

	i2c_set_clientdata(client, acc);
	input_set_drvdata(acc->idev, acc);

	ret = input_register_device(acc->idev);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register input device\n");

	ret = devm_device_add_groups(&acc->idev->dev, sc7a20h_groups);
	if (ret)
		dev_warn(dev, "cannot create sysfs attributes: %d\n", ret);

	sc7a20h_start(acc);

	dev_info(dev, "SC7A20H at 0x%02x, polling every %u ms\n",
		 client->addr, acc->delay_ms);

	return 0;
}

static int sc7a20h_remove(struct i2c_client *client)
{
	struct sc7a20h *acc = i2c_get_clientdata(client);

	sc7a20h_stop(acc);
	mutex_lock(&acc->lock);
	acc->enabled = false;
	sc7a20h_set_active(acc, false);
	mutex_unlock(&acc->lock);

	return 0;
}

static int __maybe_unused sc7a20h_suspend(struct device *dev)
{
	struct sc7a20h *acc = dev_get_drvdata(dev);

	sc7a20h_stop(acc);
	sc7a20h_set_active(acc, false);

	return 0;
}

static int __maybe_unused sc7a20h_resume(struct device *dev)
{
	struct sc7a20h *acc = dev_get_drvdata(dev);

	if (!acc->enabled)
		return 0;

	sc7a20h_set_active(acc, true);
	sc7a20h_start(acc);

	return 0;
}

static SIMPLE_DEV_PM_OPS(sc7a20h_pm_ops, sc7a20h_suspend, sc7a20h_resume);

static const struct of_device_id sc7a20h_of_match[] = {
	{ .compatible = "allwinner,sc7a20h" },
	{ }
};
MODULE_DEVICE_TABLE(of, sc7a20h_of_match);

static const struct i2c_device_id sc7a20h_i2c_id[] = {
	{ "sc7a20h", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sc7a20h_i2c_id);

static struct i2c_driver sc7a20h_driver = {
	.driver = {
		.name = "sc7a20h",
		.of_match_table = sc7a20h_of_match,
		.pm = &sc7a20h_pm_ops,
	},
	.probe_new = sc7a20h_probe,
	.remove = sc7a20h_remove,
	.id_table = sc7a20h_i2c_id,
};
module_i2c_driver(sc7a20h_driver);

MODULE_AUTHOR("H726 kernel rewrite");
MODULE_DESCRIPTION("Silan SC7A20H 3-axis accelerometer");
MODULE_LICENSE("GPL");
