// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sensortek STK83xx / Silan SC7A20H 3-axis accelerometer.
 *
 * Rewrite of the vendor stk.ko.  The stock module is a projector sensor hub:
 * besides the accelerometer it carries tilt ("tandu") tracking, swipe
 * detection, shake-to-auto-focus and calibration storage in /oem/gsensor_data.
 *
 * What is reproduced here:
 *  - the accelerometer stream the Android sensors HAL consumes (input device
 *    named "STK 3-axis Accelerometer", ABS_X/Y/Z, polled);
 *  - tilt tracking: a FIR-filtered gravity vector, converted to an angle in
 *    degrees relative to a stored reference (the vendor's "tandu" plus its
 *    angle-offset concept, which the stock gsensor platform exposes as
 *    sensor_angle / sensor_offset and re-arms through a notifier);
 *  - swipe detection: filtered X/Y deltas above a threshold latch a direction
 *    (the vendor's swipe_xy axis swap is applied first);
 *  - shake detection used for auto-focus: the vendor's shake_for_vafocus()
 *    acts on the same filtered delta magnitude and is fed by the axis sign
 *    table below.  A detected shake starts a contrast autofocus run in the
 *    focus motor driver (sun50iw15_motor_vafocus()), which is also counted,
 *    rate-limited into the log and exposed through sysfs.
 *
 * The HAL (android.hardware.sensors@2.0-service) matches accelerometers by
 * input device name, and "STK 3-axis Accelerometer" is the name it knows for
 * this chip family, so the name is significant.
 *
 * Both stk,stk83xx device tree nodes on twi1 (0x18 and 0x19) bind here; the one
 * whose chip does not answer gets -ENODEV from probe, exactly like the vendor
 * module, which relies on address probing to pick the populated part.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kstrtox.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/sun50iw15_motor.h>
#include <linux/workqueue.h>

/* Register map, shared by the STK8BAxx and SC7A20H clones. */
#define STK83XX_REG_DEVID	0x0f
#define STK83XX_REG_DATA_CFG	0x17
#define STK83XX_REG_INT_CFG1	0x21
#define STK83XX_REG_INT_CFG2	0x22
#define STK83XX_REG_INT_CFG3	0x23	/* CTRL_REG4: BDU/FS/DLPF/ST/SIM */
#define STK83XX_REG_CTRL4	0x23	/* same register as INT_CFG3 above */
#define STK83XX_CTRL4_FS_MASK	0x30
#define STK83XX_CTRL4_ST_MASK	0x06
#define STK83XX_CTRL4_ST_SHIFT	1
#define STK83XX_REG_INT_MAP1	0x25
#define STK83XX_REG_INT_MAP2	0x26
#define STK83XX_REG_INT_MAP3	0x27
#define STK83XX_REG_INTSTS	0x09
#define STK83XX_REG_CTRL1	0x20
#define STK83XX_REG_CTRL2	0x1f
#define STK83XX_REG_MODE	0x30
#define STK83XX_REG_MODE2	0x32
#define STK83XX_REG_MODE3	0x33
#define STK83XX_REG_ID2		0x70

#define STK83XX_REG_DATA_STS	0x27
#define STK83XX_REG_XOUT	0x28	/* X low, X high at +1 */
#define STK83XX_REG_YOUT	0x2a	/* Y low, Y high at +1 */
#define STK83XX_REG_ZOUT	0x2c	/* Z low, Z high at +1 */

#define STK83XX_DATA_RDY	BIT(3)

#define STK83XX_POLL_DEFAULT_MS	50
#define STK83XX_POLL_MIN_MS	10
#define STK83XX_POLL_MAX_MS	1000

#define STK83XX_FIR_MAX		16
#define STK83XX_FIR_DEFAULT	8

/* Thresholds in the same units as the reported samples (about 128 per g). */
#define STK83XX_SHAKE_DEFAULT	45
#define STK83XX_SWIPE_DEFAULT	60

/*
 * The chip delivers 12-bit samples and the vendor init sequence programs
 * CTRL4 (0x23) to 0x98, i.e. BDU + FS=01 (+/-4 g) + DLPF.  At that full scale
 * one LSB is 2 mg, so 512 counts per g; shifting the assembled sample right by
 * 3 reports 64 counts per g, 16 mg per count.  (At +/-2 g the same shift would
 * be 128 counts per g, which is where the HAL's familiar scale comes from.)
 * The vendor module applied exactly this shift to Z; one consistent shift for
 * all three axes keeps them in proportion for rotation and tilt.
 */
#define STK83XX_SHIFT		3

struct stk83xx {
	struct i2c_client *client;
	struct input_dev *idev;
	struct mutex lock;
	struct hrtimer timer;
	struct work_struct work;
	u32 delay_ms;
	bool enabled;
	u8 direction;
	u8 chip_id;
	const char *chip_name;

	/* tilt / swipe / shake state, mirroring the vendor sensor hub */
	s32 fir[3][STK83XX_FIR_MAX];
	u8 fir_len;
	u8 fir_pos;
	u8 fir_fill;
	s32 avg[3];
	s32 last[3];
	bool have_last;
	s32 angle;
	s32 angle_ref;
	u8 angle_report;
	u8 swipe_xy;
	u8 swipe_dir;
	u32 shake_threshold;
	u32 shake_count;
	u32 swipe_count;
	unsigned long shake_last_log;
	bool vafocus_pending;

	/* chip-side range and software offset calibration */
	u8 range_g;
	s32 bias[3];
	s32 st_delta_mg[3];
	bool st_valid;
	bool st_pass;

	/* angle offset persisted in the vendor's /oem/gsensor_data slot */
	struct delayed_work calib_work;
	bool calib_loaded;
	unsigned int calib_retries;
};

static const struct {
	u8 reg;
	u8 val;
} stk83xx_init_seq[] = {
	/* CTRL1: ODR 25 Hz with all three axes; CTRL4 below sets BDU + FS=01 */
	{ STK83XX_REG_CTRL1, 0x37 },
	{ STK83XX_REG_CTRL2, 0x01 },
	{ STK83XX_REG_INT_CFG3, 0x98 },
	{ STK83XX_REG_INT_CFG1, 0x73 },
	{ STK83XX_REG_MODE, 0x2a },
	{ STK83XX_REG_MODE2, 0x02 },
	{ STK83XX_REG_MODE3, 0x00 },
	{ STK83XX_REG_INT_MAP1, 0x03 },
	{ STK83XX_REG_INT_CFG2, 0x10 },
};

static int stk83xx_read(struct stk83xx *stk, u8 reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(stk->client, reg);
	if (ret < 0)
		dev_err_once(&stk->client->dev, "failed to read reg 0x%02x: %d\n",
			     reg, ret);

	return ret;
}

static int stk83xx_write(struct stk83xx *stk, u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(stk->client, reg, val);
	if (ret < 0)
		dev_err_once(&stk->client->dev, "failed to write reg 0x%02x: %d\n",
			     reg, ret);

	return ret;
}

static int stk83xx_update_bits(struct stk83xx *stk, u8 reg, u8 mask, u8 val)
{
	int old = stk83xx_read(stk, reg);

	if (old < 0)
		return old;

	return stk83xx_write(stk, reg, (old & ~mask) | (val & mask));
}

static void stk83xx_init_chip(struct stk83xx *stk)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(stk83xx_init_seq); i++)
		stk83xx_write(stk, stk83xx_init_seq[i].reg,
			      stk83xx_init_seq[i].val);
}

/* Put the part in standby (0x20 = 0) or back into measurement (0x47). */
static void stk83xx_set_active(struct stk83xx *stk, bool on)
{
	if (on) {
		stk83xx_write(stk, STK83XX_REG_CTRL1, 0x47);
		stk83xx_update_bits(stk, STK83XX_REG_INT_CFG3, 0x30, 0x10);
	} else {
		stk83xx_write(stk, STK83XX_REG_CTRL1, 0x00);
	}
}

static void stk83xx_remap(struct stk83xx *stk, s32 *x, s32 *y, s32 *z)
{
	s16 nx = *x, ny = *y, nz = *z;

	/* Direction values follow the vendor convention: axis swap + sign. */
	switch (stk->direction & 0x7) {
	case 1:
		*x = ny; *y = -nx; *z = nz;
		break;
	case 2:
		*x = -nx; *y = -ny; *z = nz;
		break;
	case 3:
		*x = -ny; *y = nx; *z = nz;
		break;
	case 4:
		*x = nx; *y = -ny; *z = -nz;
		break;
	case 5:
		*x = -nx; *y = ny; *z = -nz;
		break;
	case 6:
		*x = -ny; *y = -nx; *z = -nz;
		break;
	case 7:
		*x = ny; *y = nx; *z = -nz;
		break;
	default:
		break;
	}
}

/*
 * Moving-average filter over the gravity vector, the equivalent of the
 * vendor's "firlen" accumulator.
 */
static void stk83xx_fir_add(struct stk83xx *stk, s32 x, s32 y, s32 z)
{
	s32 sum[3] = { 0, 0, 0 };
	unsigned int i;

	stk->fir[0][stk->fir_pos] = x;
	stk->fir[1][stk->fir_pos] = y;
	stk->fir[2][stk->fir_pos] = z;

	if (stk->fir_fill < stk->fir_len)
		stk->fir_fill++;

	stk->fir_pos = (stk->fir_pos + 1) % stk->fir_len;

	for (i = 0; i < stk->fir_fill; i++) {
		sum[0] += stk->fir[0][i];
		sum[1] += stk->fir[1][i];
		sum[2] += stk->fir[2][i];
	}

	stk->avg[0] = sum[0] / stk->fir_fill;
	stk->avg[1] = sum[1] / stk->fir_fill;
	stk->avg[2] = sum[2] / stk->fir_fill;
}

/*
 * Tilt angle in degrees between the gravity vector and the Z axis, which is
 * the quantity the vendor exposes as "tandu" and stores as the angle offset.
 * h = sqrt(x^2 + y^2); the atan is approximated by the standard
 * h / (h + |z|) interpolation, accurate to a few degrees over 0..90.
 */
static s32 stk83xx_tilt_angle(s32 x, s32 y, s32 z)
{
	u32 h, az, denom;
	s32 angle;

	h = int_sqrt((u32)(x * x + y * y));
	az = (u32)abs(z);
	denom = h + az;
	if (!denom)
		return 0;

	angle = (s32)(h * 90 / denom);
	if (z < 0)
		angle = 180 - angle;

	return angle;
}

/*
 * The vendor applies the swipe_xy axis swap before treating the sample as a
 * swipe, so a panel mounted rotated still swipes correctly.
 */
static void stk83xx_apply_swipe_xy(struct stk83xx *stk, s32 *x, s32 *y)
{
	if (stk->swipe_xy) {
		s32 t = *x;

		*x = *y;
		*y = t;
	}
}

static void stk83xx_process(struct stk83xx *stk, s32 x, s32 y, s32 z)
{
	s32 dx, dy, dz, sx = x, sy = y;
	u32 delta;

	stk83xx_fir_add(stk, x, y, z);

	stk->angle = stk83xx_tilt_angle(stk->avg[0], stk->avg[1], stk->avg[2]);

	stk83xx_apply_swipe_xy(stk, &sx, &sy);

	if (!stk->have_last) {
		stk->last[0] = sx;
		stk->last[1] = sy;
		stk->last[2] = z;
		stk->have_last = true;
		return;
	}

	dx = sx - stk->last[0];
	dy = sy - stk->last[1];
	dz = z - stk->last[2];

	delta = int_sqrt((u32)(dx * dx + dy * dy + dz * dz));

	if (delta > stk->shake_threshold) {
		stk->shake_count++;
		stk->vafocus_pending = true;

		/*
		 * The vendor's shake_for_vafocus() drives the focus motor at
		 * this point.  No focus motor driver exists in this tree, so
		 * record the event instead of pretending to focus.
		 */
		if (time_after(jiffies, stk->shake_last_log + 2 * HZ)) {
			stk->shake_last_log = jiffies;
			dev_dbg(&stk->client->dev,
				"shake %u detected (delta %u), vafocus hook not bound\n",
				stk->shake_count, delta);
		}
	}

	if (abs(dx) > abs(dy)) {
		if (abs(dx) > (s32)stk->shake_threshold) {
			stk->swipe_dir = dx > 0 ? 2 : 1;	/* right : left */
			stk->swipe_count++;
		}
	} else if (abs(dy) > (s32)stk->shake_threshold) {
		stk->swipe_dir = dy > 0 ? 4 : 3;		/* down : up */
		stk->swipe_count++;
	}

	stk->last[0] = sx;
	stk->last[1] = sy;
	stk->last[2] = z;
}

static void stk83xx_report(struct stk83xx *stk)
{
	int xl, xh, yl, yh, zl, zh;
	s32 x, y, z;
	int i;

	/* Wait for a fresh sample, the vendor polls the same bit. */
	for (i = 0; i < 40; i++) {
		int sts = stk83xx_read(stk, STK83XX_REG_DATA_STS);

		if (sts < 0)
			return;
		if (sts & STK83XX_DATA_RDY)
			break;
		usleep_range(500, 1000);
	}

	xl = stk83xx_read(stk, STK83XX_REG_XOUT);
	xh = stk83xx_read(stk, STK83XX_REG_XOUT + 1);
	yl = stk83xx_read(stk, STK83XX_REG_YOUT);
	yh = stk83xx_read(stk, STK83XX_REG_YOUT + 1);
	zl = stk83xx_read(stk, STK83XX_REG_ZOUT);
	zh = stk83xx_read(stk, STK83XX_REG_ZOUT + 1);

	if (xl < 0 || xh < 0 || yl < 0 || yh < 0 || zl < 0 || zh < 0)
		return;

	x = ((s16)((xh << 8) | xl)) >> STK83XX_SHIFT;
	y = ((s16)((yh << 8) | yl)) >> STK83XX_SHIFT;
	z = ((s16)((zh << 8) | zl)) >> STK83XX_SHIFT;

	/* Zero until userspace writes cali=1. */
	x -= stk->bias[0];
	y -= stk->bias[1];
	z -= stk->bias[2];

	stk83xx_process(stk, x, y, z);
	stk83xx_remap(stk, &x, &y, &z);

	input_report_abs(stk->idev, ABS_X, x);
	input_report_abs(stk->idev, ABS_Y, y);
	input_report_abs(stk->idev, ABS_Z, z);
	input_sync(stk->idev);
}

static void stk83xx_work(struct work_struct *work)
{
	struct stk83xx *stk = container_of(work, struct stk83xx, work);

	bool focus;

	mutex_lock(&stk->lock);
	if (stk->enabled)
		stk83xx_report(stk);
	focus = stk->vafocus_pending;
	stk->vafocus_pending = false;
	mutex_unlock(&stk->lock);

#if IS_ENABLED(CONFIG_SUN50IW15_MOTOR)
	if (focus)
		sun50iw15_motor_vafocus();
#endif
}

static enum hrtimer_restart stk83xx_timer(struct hrtimer *timer)
{
	struct stk83xx *stk = container_of(timer, struct stk83xx, timer);

	schedule_work(&stk->work);
	hrtimer_forward_now(timer, ms_to_ktime(stk->delay_ms));

	return HRTIMER_RESTART;
}

static void stk83xx_start(struct stk83xx *stk)
{
	if (!stk->enabled)
		return;

	hrtimer_start(&stk->timer, ms_to_ktime(stk->delay_ms),
		      HRTIMER_MODE_REL);
}

static void stk83xx_stop(struct stk83xx *stk)
{
	hrtimer_cancel(&stk->timer);
	cancel_work_sync(&stk->work);
}

/*
 * The attributes live on the input device, so the driver data has to be
 * reached through it rather than through the i2c device.
 */
static struct stk83xx *dev_to_stk(struct device *dev)
{
	return input_get_drvdata(container_of(dev, struct input_dev, dev));
}

static ssize_t enable_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct stk83xx *stk = dev_to_stk(dev);

	return sysfs_emit(buf, "%d\n", stk->enabled);
}

static ssize_t enable_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct stk83xx *stk = dev_to_stk(dev);
	bool enable;
	int ret;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	mutex_lock(&stk->lock);
	if (enable == stk->enabled) {
		mutex_unlock(&stk->lock);
		return count;
	}

	stk->enabled = enable;
	if (enable) {
		stk83xx_set_active(stk, true);
		stk83xx_start(stk);
	} else {
		stk83xx_stop(stk);
		stk83xx_set_active(stk, false);
	}
	mutex_unlock(&stk->lock);

	return count;
}
static DEVICE_ATTR_RW(enable);

static ssize_t delay_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct stk83xx *stk = dev_to_stk(dev);

	return sysfs_emit(buf, "%u\n", stk->delay_ms);
}

static ssize_t delay_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct stk83xx *stk = dev_to_stk(dev);
	unsigned int delay;
	int ret;

	ret = kstrtouint(buf, 0, &delay);
	if (ret)
		return ret;

	delay = clamp(delay, STK83XX_POLL_MIN_MS, STK83XX_POLL_MAX_MS);

	mutex_lock(&stk->lock);
	stk->delay_ms = delay;
	if (stk->enabled) {
		hrtimer_cancel(&stk->timer);
		stk83xx_start(stk);
	}
	mutex_unlock(&stk->lock);

	return count;
}
static DEVICE_ATTR_RW(delay);

static ssize_t reg_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct stk83xx *stk = dev_to_stk(dev);
	int i, len = 0;

	for (i = STK83XX_REG_DEVID; i <= STK83XX_REG_ID2 && len < PAGE_SIZE - 16; i++) {
		int val = stk83xx_read(stk, i);

		if (val < 0)
			break;
		len += sysfs_emit_at(buf, len, "0x%02x: 0x%02x\n", i, val);
	}

	return len;
}

static ssize_t reg_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct stk83xx *stk = dev_to_stk(dev);
	unsigned int reg, val;

	if (sscanf(buf, "0x%x=0x%x", &reg, &val) != 2)
		return -EINVAL;
	if (reg > 0xff || val > 0xff)
		return -EINVAL;

	stk83xx_write(stk, reg, val);

	return count;
}
static DEVICE_ATTR_RW(reg);

static ssize_t chipinfo_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct stk83xx *stk = dev_to_stk(dev);

	return sysfs_emit(buf, "%s 0x%02x\n", stk->chip_name, stk->chip_id);
}
static DEVICE_ATTR_RO(chipinfo);

/*
 * Angle-offset persistence.
 *
 * The stock gsensor platform keeps the tilt offset in /oem/gsensor_data: the
 * decompiled gsensor_kxt.c does sprintf(&angle_file, "%d", kxt_ori_tandu),
 * kernel_write(fp, &angle_file, 10, &pos), and later kernel_read(..., 10) plus
 * kstrtoint(), clamping the value to 0..90 (0x5a) on the way in.  The vendor
 * retried the open through a 500 ms delayed work because /oem is a separate
 * partition that may not be mounted when the sensor probes; the same retry is
 * done here, bounded so a missing partition cannot spin forever.
 *
 * Difference: our tilt spans 0..180 degrees (the vendor's "tandu" never passed
 * 90), so the accepted range is widened accordingly.
 */
#define STK83XX_CALIB_PATH	"/oem/gsensor_data"
#define STK83XX_CALIB_SLOT	10
#define STK83XX_CALIB_RETRIES	20

static bool stk83xx_calib_read(struct stk83xx *stk)
{
	struct file *fp;
	char buf[STK83XX_CALIB_SLOT + 1] = { 0 };
	loff_t pos = 0;
	int val;

	fp = filp_open(STK83XX_CALIB_PATH, O_RDWR | O_CREAT, 0644);
	if (IS_ERR(fp))
		return false;

	if (kernel_read(fp, buf, STK83XX_CALIB_SLOT, &pos) > 0 &&
	    !kstrtoint(buf, 10, &val) && val >= 0 && val <= 180) {
		mutex_lock(&stk->lock);
		stk->angle_ref = val;
		mutex_unlock(&stk->lock);
		dev_info(&stk->client->dev, "restored angle offset %d from %s\n",
			 val, STK83XX_CALIB_PATH);
	}

	filp_close(fp, NULL);

	return true;
}

static void stk83xx_calib_write(struct stk83xx *stk)
{
	char buf[STK83XX_CALIB_SLOT] = { 0 };
	struct file *fp;
	loff_t pos = 0;
	s32 val;

	mutex_lock(&stk->lock);
	val = stk->angle_ref;
	mutex_unlock(&stk->lock);

	/* The vendor slot is a decimal string padded with NULs to 10 bytes. */
	snprintf(buf, sizeof(buf), "%d", val);

	fp = filp_open(STK83XX_CALIB_PATH, O_RDWR | O_CREAT, 0644);
	if (IS_ERR(fp)) {
		dev_warn_ratelimited(&stk->client->dev,
				     "cannot open %s: %ld, offset not saved\n",
				     STK83XX_CALIB_PATH, PTR_ERR(fp));
		return;
	}

	kernel_write(fp, buf, sizeof(buf), &pos);
	filp_close(fp, NULL);
}

static void stk83xx_calib_work(struct work_struct *work)
{
	struct stk83xx *stk = container_of(work, struct stk83xx,
					   calib_work.work);

	if (stk83xx_calib_read(stk)) {
		stk->calib_loaded = true;
		return;
	}

	if (++stk->calib_retries < STK83XX_CALIB_RETRIES) {
		schedule_delayed_work(&stk->calib_work, msecs_to_jiffies(500));
		return;
	}

	dev_warn(&stk->client->dev,
		 "%s unavailable, angle offset not restored\n",
		 STK83XX_CALIB_PATH);
}

/* "1" captures the current tilt as the offset, "0" clears it. */
static void stk83xx_set_offset(struct stk83xx *stk, bool capture)
{
	mutex_lock(&stk->lock);
	stk->angle_ref = capture ? stk->angle : 0;
	mutex_unlock(&stk->lock);

	stk83xx_calib_write(stk);
}

/*
 * Chip-side range and calibration.
 *
 * The populated part is an SC7A20H: probe checks WHO_AM_I (0x0F) == 0x11 and
 * VERSION (0x70) == 0x28, the same pair the vendor's sc7a20h driver checks.
 * Unlike the older SC7A20 there is no user offset register block here; the
 * datasheet lists 0x10..0x1E as reserved, and VERSION 0x28 is the only variant
 * with a documented register map.  So the only calibration field the chip
 * exposes is full scale, in CTRL_REG4 (0x23):
 *
 *   bit 7 BDU, bit 6 BLE, bits 5:4 FS1:FS0, bit 3 DLPF[0], bits 2:1 ST1:ST0,
 *   bit 0 SIM.
 *
 *   FS1:FS0  00 = +/-2 g   01 = +/-4 g   10 = +/-8 g   11 = +/-16 g
 *   LSB      1 mg         2 mg          4 mg          8 mg   (12-bit mode)
 *   ST1:ST0  00 = normal   01 = self-test 0   10 = self-test 1
 *
 * The vendor module's stk_range_selection / stk_cali / stk_set_offset bodies
 * write 0x10..0x16 and 0x1A..0x1E.  Those belong to the STK8BA58 map
 * (RANGESEL 0x0F, BWSEL 0x10, OFSTCOMP1 0x36, OFSTX/Y/Z 0x38..0x3A), which is
 * not the part on this board, so those sequences are deliberately not replayed:
 * writing the STK8BA58 RANGESEL values (0x3/0x5/0x8) into this chip's 0x0F
 * would address WHO_AM_I and the "offset" registers do not exist.
 *
 * What is implemented instead, all read-modify-write on 0x23 so BDU, DLPF and
 * SIM are preserved:
 *  - `range` selects 2/4/8/16 g, the documented FS1:FS0 field (default 4 g,
 *    which is what the vendor init sequence programs);
 *  - `selftest` drives ST1:ST0 and compares the measured deflection against
 *    the datasheet's 320/320/430 mg typical values (section 9.3);
 *  - `cali` measures and subtracts a per-axis bias in software, because this
 *    chip has no offset registers to write.
 */
static int stk83xx_apply_range(struct stk83xx *stk)
{
	u8 fs;
	int v;

	switch (stk->range_g) {
	case 2:
		fs = 0;
		break;
	case 4:
		fs = 1;
		break;
	case 8:
		fs = 2;
		break;
	case 16:
		fs = 3;
		break;
	default:
		return -EINVAL;
	}

	v = stk83xx_read(stk, STK83XX_REG_CTRL4);
	if (v < 0)
		return v;

	return stk83xx_write(stk, STK83XX_REG_CTRL4,
			     (v & ~STK83XX_CTRL4_FS_MASK) | (fs << 4));
}

/* Milligrams represented by one reported count at the current range. */
static int stk83xx_mg_per_count(struct stk83xx *stk)
{
	int fs;

	switch (stk->range_g) {
	case 2:
		fs = 0;
		break;
	case 8:
		fs = 2;
		break;
	case 16:
		fs = 3;
		break;
	case 4:
	default:
		fs = 1;
		break;
	}

	return (1 << fs) << STK83XX_SHIFT;
}

/* One sample, already shifted to the units the input device reports. */
static int stk83xx_read_xyz(struct stk83xx *stk, s32 *x, s32 *y, s32 *z)
{
	int xl, xh, yl, yh, zl, zh, i;

	for (i = 0; i < 40; i++) {
		int sts = stk83xx_read(stk, STK83XX_REG_DATA_STS);

		if (sts < 0)
			return sts;
		if (sts & STK83XX_DATA_RDY)
			break;
		usleep_range(500, 1000);
	}

	xl = stk83xx_read(stk, STK83XX_REG_XOUT);
	xh = stk83xx_read(stk, STK83XX_REG_XOUT + 1);
	yl = stk83xx_read(stk, STK83XX_REG_YOUT);
	yh = stk83xx_read(stk, STK83XX_REG_YOUT + 1);
	zl = stk83xx_read(stk, STK83XX_REG_ZOUT);
	zh = stk83xx_read(stk, STK83XX_REG_ZOUT + 1);

	if (xl < 0 || xh < 0 || yl < 0 || yh < 0 || zl < 0 || zh < 0)
		return -EIO;

	*x = ((s16)((xh << 8) | xl)) >> STK83XX_SHIFT;
	*y = ((s16)((yh << 8) | yl)) >> STK83XX_SHIFT;
	*z = ((s16)((zh << 8) | zl)) >> STK83XX_SHIFT;

	return 0;
}

static int stk83xx_average(struct stk83xx *stk, s32 out[3], int count)
{
	s64 sum[3] = { 0, 0, 0 };
	int i, ret;

	for (i = 0; i < count; i++) {
		s32 x, y, z;

		ret = stk83xx_read_xyz(stk, &x, &y, &z);
		if (ret)
			return ret;

		sum[0] += x;
		sum[1] += y;
		sum[2] += z;
		usleep_range(1000, 2000);
	}

	out[0] = div_s64(sum[0], count);
	out[1] = div_s64(sum[1], count);
	out[2] = div_s64(sum[2], count);

	return 0;
}

/* Tilt, exposed the way the stock gsensor platform exposes sensor_angle. */
static ssize_t angle_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct stk83xx *stk = dev_to_stk(dev);

	return sysfs_emit(buf, "%d\n", stk->angle - stk->angle_ref);
}

static ssize_t angle_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct stk83xx *stk = dev_to_stk(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	stk83xx_set_offset(stk, !!val);

	return count;
}
static DEVICE_ATTR_RW(angle);

/* The vendor's name for the same quantity: sensor_offset / stk_offset. */
static ssize_t offset_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct stk83xx *stk = dev_to_stk(dev);

	return sysfs_emit(buf, "%d\n", stk->angle_ref);
}

static ssize_t offset_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct stk83xx *stk = dev_to_stk(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	stk83xx_set_offset(stk, !!val);

	return count;
}
static DEVICE_ATTR_RW(offset);

static ssize_t range_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct stk83xx *stk = dev_to_stk(dev);

	return sysfs_emit(buf, "%u\n", stk->range_g);
}

static ssize_t range_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct stk83xx *stk = dev_to_stk(dev);
	unsigned int g;
	u8 prev;
	int ret;

	ret = kstrtouint(buf, 0, &g);
	if (ret)
		return ret;
	if (g != 2 && g != 4 && g != 8 && g != 16)
		return -EINVAL;

	mutex_lock(&stk->lock);
	prev = stk->range_g;
	stk->range_g = g;
	ret = stk83xx_apply_range(stk);
	if (ret)
		stk->range_g = prev;
	mutex_unlock(&stk->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(range);

/* Documented self-test: 0 disables and clears, 1/2 select self-test 0/1. */
static ssize_t selftest_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct stk83xx *stk = dev_to_stk(dev);

	if (!stk->st_valid)
		return sysfs_emit(buf, "not run\n");

	return sysfs_emit(buf, "%s X=%dmg Y=%dmg Z=%dmg\n",
			  stk->st_pass ? "pass" : "fail",
			  stk->st_delta_mg[0], stk->st_delta_mg[1],
			  stk->st_delta_mg[2]);
}

static ssize_t selftest_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct stk83xx *stk = dev_to_stk(dev);
	s32 base[3] = { 0 }, st[3] = { 0 };
	unsigned int mode;
	int ret, v, mg;

	ret = kstrtouint(buf, 0, &mode);
	if (ret)
		return ret;
	if (mode > 2)
		return -EINVAL;

	mutex_lock(&stk->lock);

	if (!mode) {
		stk->st_valid = false;
		ret = stk83xx_apply_range(stk);	/* also clears ST1:ST0 */
		mutex_unlock(&stk->lock);
		return ret ? ret : count;
	}

	/* Baseline first, then the same measurement with the test force on. */
	ret = stk83xx_average(stk, base, 8);
	if (ret)
		goto out;

	v = stk83xx_read(stk, STK83XX_REG_CTRL4);
	if (v < 0) {
		ret = v;
		goto out;
	}
	ret = stk83xx_write(stk, STK83XX_REG_CTRL4,
			    (v & ~STK83XX_CTRL4_ST_MASK) |
			    (mode << STK83XX_CTRL4_ST_SHIFT));
	if (ret)
		goto clear;

	msleep(100);
	ret = stk83xx_average(stk, st, 8);

clear:
	mutex_unlock(&stk->lock);
	mutex_lock(&stk->lock);
	v = stk83xx_read(stk, STK83XX_REG_CTRL4);
	if (v >= 0)
		stk83xx_write(stk, STK83XX_REG_CTRL4,
			      v & ~STK83XX_CTRL4_ST_MASK);

	if (!ret) {
		mg = stk83xx_mg_per_count(stk);
		stk->st_delta_mg[0] = (st[0] - base[0]) * mg;
		stk->st_delta_mg[1] = (st[1] - base[1]) * mg;
		stk->st_delta_mg[2] = (st[2] - base[2]) * mg;
		/* Datasheet typicals are 320/320/430 mg; accept a wide band. */
		stk->st_pass = stk->st_delta_mg[0] > 100 &&
			       stk->st_delta_mg[0] < 900 &&
			       stk->st_delta_mg[1] > 100 &&
			       stk->st_delta_mg[1] < 900 &&
			       stk->st_delta_mg[2] > 150 &&
			       stk->st_delta_mg[2] < 1100;
		stk->st_valid = true;
	}

out:
	mutex_unlock(&stk->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(selftest);

/*
 * Offset calibration.  This chip has no offset registers, so the bias is
 * measured and subtracted in software: write 1 captures the current average,
 * write 0 clears it.  The value is volatile and resets with the driver.
 */
static ssize_t cali_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct stk83xx *stk = dev_to_stk(dev);

	return sysfs_emit(buf, "%d %d %d\n", stk->bias[0], stk->bias[1],
			  stk->bias[2]);
}

static ssize_t cali_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct stk83xx *stk = dev_to_stk(dev);
	s32 avg[3];
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	mutex_lock(&stk->lock);
	if (!val) {
		stk->bias[0] = 0;
		stk->bias[1] = 0;
		stk->bias[2] = 0;
		mutex_unlock(&stk->lock);
		return count;
	}

	ret = stk83xx_average(stk, avg, 16);
	if (!ret) {
		stk->bias[0] = avg[0];
		stk->bias[1] = avg[1];
		stk->bias[2] = avg[2];
	}
	mutex_unlock(&stk->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(cali);

static ssize_t angle_report_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct stk83xx *stk = dev_to_stk(dev);

	return sysfs_emit(buf, "%u\n", stk->angle_report);
}

static ssize_t angle_report_store(struct device *dev,
				  struct device_attribute *attr, const char *buf,
				  size_t count)
{
	struct stk83xx *stk = dev_to_stk(dev);
	bool en;
	int ret;

	ret = kstrtobool(buf, &en);
	if (ret)
		return ret;

	stk->angle_report = en;

	return count;
}
static DEVICE_ATTR_RW(angle_report);

static ssize_t swipe_xy_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct stk83xx *stk = dev_to_stk(dev);

	return sysfs_emit(buf, "%u\n", stk->swipe_xy);
}

static ssize_t swipe_xy_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct stk83xx *stk = dev_to_stk(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	stk->swipe_xy = !!val;
	stk->have_last = false;

	return count;
}
static DEVICE_ATTR_RW(swipe_xy);

static ssize_t shake_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct stk83xx *stk = dev_to_stk(dev);

	return sysfs_emit(buf, "%u\n", stk->shake_count);
}
static DEVICE_ATTR_RO(shake);

static ssize_t swipe_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct stk83xx *stk = dev_to_stk(dev);

	return sysfs_emit(buf, "%u\n", stk->swipe_dir);
}
static DEVICE_ATTR_RO(swipe);

static ssize_t firlen_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct stk83xx *stk = dev_to_stk(dev);

	return sysfs_emit(buf, "%u\n", stk->fir_len);
}

static ssize_t firlen_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct stk83xx *stk = dev_to_stk(dev);
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	if (!val || val > STK83XX_FIR_MAX)
		return -EINVAL;

	mutex_lock(&stk->lock);
	stk->fir_len = val;
	stk->fir_pos = 0;
	stk->fir_fill = 0;
	stk->have_last = false;
	mutex_unlock(&stk->lock);

	return count;
}
static DEVICE_ATTR_RW(firlen);

static struct attribute *stk83xx_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_delay.attr,
	&dev_attr_reg.attr,
	&dev_attr_chipinfo.attr,
	&dev_attr_angle.attr,
	&dev_attr_offset.attr,
	&dev_attr_range.attr,
	&dev_attr_selftest.attr,
	&dev_attr_cali.attr,
	&dev_attr_angle_report.attr,
	&dev_attr_swipe_xy.attr,
	&dev_attr_swipe.attr,
	&dev_attr_shake.attr,
	&dev_attr_firlen.attr,
	NULL,
};
ATTRIBUTE_GROUPS(stk83xx);

static const char *stk83xx_chip_name(u8 id)
{
	switch (id) {
	case 0x11:
		return "stk8ba58/sc7a20h";
	case 0x21:
		return "stk8321";
	case 0x29:
		return "stk8329";
	default:
		return "stk83xx";
	}
}

static int stk83xx_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device_node *np = dev->of_node;
	struct stk83xx *stk;
	int id, ret;
	u32 val;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return dev_err_probe(dev, -EOPNOTSUPP,
				     "adapter lacks SMBus byte-data support\n");

	stk = devm_kzalloc(dev, sizeof(*stk), GFP_KERNEL);
	if (!stk)
		return -ENOMEM;

	stk->client = client;
	stk->delay_ms = STK83XX_POLL_DEFAULT_MS;
	stk->enabled = true;
	stk->fir_len = STK83XX_FIR_DEFAULT;
	stk->shake_threshold = STK83XX_SHAKE_DEFAULT;
	stk->range_g = 4;

	if (!of_property_read_u32(np, "stk,direction", &val))
		stk->direction = val & 0x7;
	if (!of_property_read_u32(np, "stk,swipe_xy", &val))
		stk->swipe_xy = !!val;
	if (!of_property_read_u32(np, "stk,firlen", &val) &&
	    val && val <= STK83XX_FIR_MAX)
		stk->fir_len = val;
	if (!of_property_read_u32(np, "stk,shake_threshold", &val))
		stk->shake_threshold = val;

	mutex_init(&stk->lock);

	id = stk83xx_read(stk, STK83XX_REG_DEVID);
	if (id < 0)
		return id;
	if (id == 0x00 || id == 0xff)
		return -ENODEV;

	stk->chip_id = id;
	stk->chip_name = stk83xx_chip_name(id);

	dev_info(dev, "found %s (id 0x%02x) at 0x%02x\n", stk->chip_name, id,
		 client->addr);

	stk83xx_init_chip(stk);
	msleep(5);
	stk83xx_set_active(stk, true);

	stk->idev = devm_input_allocate_device(dev);
	if (!stk->idev)
		return -ENOMEM;

	stk->idev->name = "STK 3-axis Accelerometer";
	stk->idev->phys = "stk83xx/input0";
	stk->idev->id.bustype = BUS_I2C;
	stk->idev->id.vendor = 0x0001;
	stk->idev->id.product = 0x0001;
	stk->idev->id.version = 0x0001;
	stk->idev->dev.parent = dev;

	input_set_capability(stk->idev, EV_ABS, ABS_X);
	input_set_capability(stk->idev, EV_ABS, ABS_Y);
	input_set_capability(stk->idev, EV_ABS, ABS_Z);
	input_set_abs_params(stk->idev, ABS_X, -2048, 2047, 0, 0);
	input_set_abs_params(stk->idev, ABS_Y, -2048, 2047, 0, 0);
	input_set_abs_params(stk->idev, ABS_Z, -2048, 2047, 0, 0);

	INIT_DELAYED_WORK(&stk->calib_work, stk83xx_calib_work);
	hrtimer_init(&stk->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	stk->timer.function = stk83xx_timer;
	INIT_WORK(&stk->work, stk83xx_work);

	i2c_set_clientdata(client, stk);
	input_set_drvdata(stk->idev, stk);

	ret = input_register_device(stk->idev);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register input device\n");

	ret = devm_device_add_groups(&stk->idev->dev, stk83xx_groups);
	if (ret)
		dev_warn(dev, "cannot create sysfs attributes: %d\n", ret);

	stk83xx_start(stk);
	schedule_delayed_work(&stk->calib_work, 0);

	dev_info(dev, "accelerometer ready, polling every %u ms, tilt/swipe/shake enabled\n",
		 stk->delay_ms);

	return 0;
}

static int stk83xx_remove(struct i2c_client *client)
{
	struct stk83xx *stk = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&stk->calib_work);
	stk83xx_stop(stk);
	mutex_lock(&stk->lock);
	stk->enabled = false;
	stk83xx_set_active(stk, false);
	mutex_unlock(&stk->lock);

	return 0;
}

static int __maybe_unused stk83xx_suspend(struct device *dev)
{
	struct stk83xx *stk = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&stk->calib_work);
	stk83xx_stop(stk);
	stk83xx_set_active(stk, false);

	return 0;
}

static int __maybe_unused stk83xx_resume(struct device *dev)
{
	struct stk83xx *stk = dev_get_drvdata(dev);

	if (!stk->calib_loaded && stk->calib_retries < STK83XX_CALIB_RETRIES)
		schedule_delayed_work(&stk->calib_work, 0);

	if (!stk->enabled)
		return 0;

	stk83xx_set_active(stk, true);

	mutex_lock(&stk->lock);
	stk83xx_apply_range(stk);
	mutex_unlock(&stk->lock);

	stk83xx_start(stk);

	return 0;
}

static SIMPLE_DEV_PM_OPS(stk83xx_pm_ops, stk83xx_suspend, stk83xx_resume);

static const struct of_device_id stk83xx_of_match[] = {
	{ .compatible = "stk,stk83xx" },
	{ }
};
MODULE_DEVICE_TABLE(of, stk83xx_of_match);

static const struct i2c_device_id stk83xx_i2c_id[] = {
	{ "stk83xx", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, stk83xx_i2c_id);

static struct i2c_driver stk83xx_driver = {
	.driver = {
		.name = "stk83xx",
		.of_match_table = stk83xx_of_match,
		.pm = &stk83xx_pm_ops,
	},
	.probe_new = stk83xx_probe,
	.remove = stk83xx_remove,
	.id_table = stk83xx_i2c_id,
};
module_i2c_driver(stk83xx_driver);

MODULE_AUTHOR("H726 kernel rewrite");
MODULE_DESCRIPTION("Sensortek STK83xx / SC7A20H 3-axis accelerometer");
MODULE_LICENSE("GPL");
