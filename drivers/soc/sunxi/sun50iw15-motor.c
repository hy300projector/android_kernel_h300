// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allwinner H726/HY300 focus motor: stepper phase sequencer, limit switch and
 * contrast autofocus.
 *
 * The stock firmware splits this over two pieces:
 *  - motor-control, built into the vendor kernel (`motor.c`), drives the four
 *    phase GPIOs from the `motor-cw-table` / `motor-ccw-table` byte sequences,
 *    homes against the up limit switch and exposes
 *    /sys/devices/platform/motor_ctr/motor_ctrl.  Its command word is decimal
 *    `0xCCAA`: CC = command, AA = argument; 1 = up, 2 = down, 3 = reset
 *    position, 7 = goto step, 8 = autofocus.
 *  - motor-limiter.ko, a character device /dev/motor_limiter that reports the
 *    limit switch through read() and SIGIO (fasync), version 1.0.1.
 *
 * This driver provides both halves in one built-in driver, because nothing else
 * in this kernel binds the `motor_ctrl` node (motor-control.ko does not exist
 * on the stock images and the vendor copy is built in).
 *
 * Autofocus: the vendor's 0x08 command only homes the lens, because the
 * vendor's focus logic lives in userspace.  Here the driver runs the search
 * itself - home to the up stop, coarse sweep down recording a sharpness value
 * at every position, locate the peak, fine sweep around it, parabolic
 * interpolation between the three best samples and finally a move to the
 * interpolated best position.  Sharpness comes from a registered
 * sun50iw15_motor_af_source (built into the header above); when none is
 * registered the values can be pushed from userspace through `af_metric`
 * after `af_bridge` is armed, which is how a HAL sitting on the vendor's
 * sensor can drive the search.  With neither, the driver falls back to the
 * vendor behaviour of homing alone.
 *
 * Hardware description comes from the device tree:
 *   compatible = "motor-control";
 *   motor-phase-num = <4>;  motor-gpio-phase0..3 = <&pio ...>;
 *   limiter-up-gpio = <&pio ...>;  active_level = <1>;
 *   motor-step-num, motor-cycle, motor-cw-table, motor-ccw-table,
 *   motor-phase-udelay, motor-step-mdelay.
 * Optional: af-coarse-step, af-fine-step, af-max-steps, af-settle-ms,
 * af-timeout-ms, af-bridge.
 */

#include <linux/cdev.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kstrtox.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sun50iw15_motor.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#define SUNXI_MOTOR_MAX_PHASES	8
#define SUNXI_MOTOR_MAX_STEPS	64
#define SUNXI_MOTOR_HOME_MAX	160

#define SUNXI_AF_MAX_POINTS	48
#define SUNXI_AF_HARD_MAX	400
#define SUNXI_AF_SEED_POINTS	8

enum sunxi_motor_af_state {
	AF_IDLE,
	AF_HOMING,
	AF_COARSE,
	AF_FINE,
	AF_DONE,
	AF_FLAT,
	AF_CANCELLED,
	AF_NO_SOURCE,
	AF_TIMEOUT,
	AF_ERROR,
};

static const char *const af_state_names[] = {
	[AF_IDLE]	= "idle",
	[AF_HOMING]	= "homing",
	[AF_COARSE]	= "coarse",
	[AF_FINE]	= "fine",
	[AF_DONE]	= "done",
	[AF_FLAT]	= "flat",
	[AF_CANCELLED]	= "cancelled",
	[AF_NO_SOURCE]	= "no_source",
	[AF_TIMEOUT]	= "timeout",
	[AF_ERROR]	= "error",
};

struct sunxi_motor_af_point {
	u32 pos;
	u32 fv;
};

struct sunxi_motor {
	struct device *dev;
	struct mutex lock;
	struct gpio_desc *phase[SUNXI_MOTOR_MAX_PHASES];
	unsigned int phase_num;
	struct gpio_desc *limit;
	int limit_irq;
	u8 cw[SUNXI_MOTOR_MAX_STEPS];
	u8 ccw[SUNXI_MOTOR_MAX_STEPS];
	unsigned int step_num;
	unsigned int cycle;
	unsigned int phase_udelay;
	unsigned int step_mdelay;
	unsigned int active_level;
	unsigned int back_step;
	bool stepper;

	/* lens position in half-steps below the homed reference point */
	u32 pos;

	/* contrast autofocus */
	struct work_struct af_work;
	struct mutex af_lock;
	struct mutex af_cancel_lock;
	struct completion af_done;
	struct completion af_sample;
	enum sunxi_motor_af_state af_state;
	bool af_running;
	bool af_cancel;
	bool af_bridge;
	bool af_pending;
	bool af_got;
	u32 af_pending_pos;
	u32 af_metric_val;
	u32 af_coarse;
	u32 af_fine;
	u32 af_max_steps;
	u32 af_settle_ms;
	u32 af_timeout_ms;
	u32 af_best_pos;
	u32 af_best_fv;
	u32 af_last_fv;
	u32 af_samples;

	/* set once the lens has been homed at least once */
	bool homed;

	/* focus seeding from a measured distance */
	struct work_struct seed_work;
	bool af_seed_valid;
	bool af_preposition;
	u32 af_seed_mm;
	u32 af_seed_pos;
	u32 af_seed_window;
	u32 af_seed_points;
	u32 af_seed_table[SUNXI_AF_SEED_POINTS * 2];

	dev_t devt;
	struct cdev cdev;
	struct class *cls;
	struct device *chrdev;
	struct device *motor0;
	struct fasync_struct *async;
	atomic_t open_count;
};

static struct sunxi_motor *sunxi_motor_dev;

static DEFINE_MUTEX(af_source_lock);
static const struct sun50iw15_motor_af_source *af_source;

static int sunxi_motor_fasync(int fd, struct file *file, int on);
static int sunxi_motor_af_start(struct sunxi_motor *m);
static void sunxi_motor_af_cancel(struct sunxi_motor *m);

static int sunxi_motor_limited(struct sunxi_motor *m)
{
	if (!m->limit)
		return 0;

	return gpiod_get_raw_value(m->limit) == m->active_level;
}

/**
 * sun50iw15_motor_limited - report the focus end stop
 *
 * Returns 1 when the lens has reached the limit switch, 0 otherwise, or a
 * negative errno when no motor is registered.
 */
int sun50iw15_motor_limited(void)
{
	struct sunxi_motor *m = sunxi_motor_dev;

	if (!m)
		return -ENODEV;

	return sunxi_motor_limited(m);
}
EXPORT_SYMBOL_GPL(sun50iw15_motor_limited);

static void sunxi_motor_deenergize(struct sunxi_motor *m)
{
	unsigned int i;

	for (i = 0; i < m->phase_num; i++)
		gpiod_set_raw_value(m->phase[i], 0);
}

/* One full sequence ("mstep") in the vendor's sense: cycle x step_num phases. */
static void sunxi_motor_mstep(struct sunxi_motor *m, bool up, unsigned int steps)
{
	const u8 *table = up ? m->cw : m->ccw;
	unsigned int s, c, i, p;

	for (s = 0; s < steps; s++) {
		for (c = 0; c < m->cycle; c++) {
			for (i = 0; i < m->step_num; i++) {
				u8 pattern = table[i % SUNXI_MOTOR_MAX_STEPS];

				for (p = 0; p < m->phase_num; p++)
					gpiod_set_raw_value(m->phase[p],
							    (pattern >> p) & 1);
				if (m->phase_udelay)
					usleep_range(m->phase_udelay * 800,
						     m->phase_udelay * 1000);
			}
		}
		if (m->step_mdelay)
			msleep(m->step_mdelay);
	}

	sunxi_motor_deenergize(m);
}

/*
 * Home towards the up stop, then back off to a repeatable reference.  The
 * caller must hold m->lock.  Cancellation is ignored here so that the lens
 * always ends up at a known position.
 */
static int sunxi_motor_home_locked(struct sunxi_motor *m)
{
	unsigned int i;

	for (i = 0; i < SUNXI_MOTOR_HOME_MAX; i++) {
		if (sunxi_motor_limited(m))
			break;
		sunxi_motor_mstep(m, true, 1);
		cond_resched();
	}
	if (m->back_step)
		sunxi_motor_mstep(m, false, m->back_step);

	m->pos = 0;
	m->homed = true;

	return 0;
}

static int sunxi_motor_home(struct sunxi_motor *m)
{
	int ret;

	if (!m->stepper)
		return -EOPNOTSUPP;

	mutex_lock(&m->lock);
	ret = sunxi_motor_home_locked(m);
	mutex_unlock(&m->lock);

	return ret;
}

/* ---- contrast autofocus ------------------------------------------------- */

static const struct sun50iw15_motor_af_source *sunxi_motor_af_source_get(void)
{
	const struct sun50iw15_motor_af_source *src;

	mutex_lock(&af_source_lock);
	src = af_source;
	mutex_unlock(&af_source_lock);

	return src;
}

static void sunxi_motor_af_set_state(struct sunxi_motor *m,
				     enum sunxi_motor_af_state state)
{
	mutex_lock(&m->af_lock);
	m->af_state = state;
	mutex_unlock(&m->af_lock);
}

static bool sunxi_motor_af_cancelled(struct sunxi_motor *m)
{
	bool cancel;

	mutex_lock(&m->af_lock);
	cancel = m->af_cancel;
	mutex_unlock(&m->af_lock);

	return cancel;
}

/* Move the lens to an absolute position, below the homed reference. */
static int sunxi_motor_af_move_to(struct sunxi_motor *m, u32 target)
{
	u32 cur;

	if (sunxi_motor_af_cancelled(m))
		return -ECANCELED;

	mutex_lock(&m->lock);
	cur = m->pos;
	if (target > cur)
		sunxi_motor_mstep(m, false, target - cur);
	else if (target < cur)
		sunxi_motor_mstep(m, true, cur - target);
	m->pos = target;
	mutex_unlock(&m->lock);

	return 0;
}

/*
 * Ask for one sharpness reading at the current position.  Prefers a registered
 * kernel source; otherwise hands the position to userspace through
 * `af_awaiting` and waits for `af_metric`, bounded by af_timeout_ms.
 */
static int sunxi_motor_af_take_sample(struct sunxi_motor *m, u32 *fv)
{
	const struct sun50iw15_motor_af_source *src = sunxi_motor_af_source_get();
	unsigned long timeout;
	long rc;

	if (src) {
		int ret = src->sample(src->priv, fv);

		if (ret) {
			dev_warn_ratelimited(m->dev,
					     "af source %s failed: %d\n",
					     src->name, ret);
			return ret;
		}

		return 0;
	}

	if (!m->af_bridge)
		return -ENODEV;

	mutex_lock(&m->af_lock);
	if (m->af_cancel) {
		mutex_unlock(&m->af_lock);
		return -ECANCELED;
	}
	m->af_pending = true;
	m->af_got = false;
	m->af_pending_pos = m->pos;
	reinit_completion(&m->af_sample);
	mutex_unlock(&m->af_lock);

	timeout = msecs_to_jiffies(m->af_timeout_ms);
	rc = wait_for_completion_interruptible_timeout(&m->af_sample, timeout);

	mutex_lock(&m->af_lock);
	m->af_pending = false;
	if (m->af_cancel) {
		rc = -ECANCELED;
	} else if (rc == 0) {
		rc = -ETIMEDOUT;
	} else if (rc > 0) {
		if (!m->af_got)
			rc = -EAGAIN;
		else
			*fv = m->af_metric_val;
	}
	mutex_unlock(&m->af_lock);

	if (rc < 0)
		return (int)rc;

	return 0;
}

static int sunxi_motor_af_measure(struct sunxi_motor *m, u32 target, u32 *fv)
{
	int ret;

	ret = sunxi_motor_af_move_to(m, target);
	if (ret)
		return ret;

	if (m->af_settle_ms)
		msleep(m->af_settle_ms);

	if (sunxi_motor_af_cancelled(m))
		return -ECANCELED;

	ret = sunxi_motor_af_take_sample(m, fv);
	if (ret)
		return ret;

	mutex_lock(&m->af_lock);
	m->af_last_fv = *fv;
	m->af_samples++;
	mutex_unlock(&m->af_lock);

	return 0;
}

static unsigned int sunxi_motor_af_best(const struct sunxi_motor_af_point *pts,
				       unsigned int n)
{
	unsigned int i, best = 0;

	for (i = 1; i < n; i++)
		if (pts[i].fv > pts[best].fv)
			best = i;

	return best;
}

/* True when the newest sample is the highest so far, i.e. the peak may lie
 * further along and the sweep should be extended. */
static bool sunxi_motor_af_rising(const struct sunxi_motor_af_point *pts,
				  unsigned int n)
{
	unsigned int i;

	if (n < 2)
		return false;

	for (i = 0; i + 1 < n; i++)
		if (pts[i].fv >= pts[n - 1].fv)
			return false;

	return true;
}

/*
 * Parabolic interpolation through the three samples around the best one.
 * Returns the step offset (in lens steps) to add to the best position.
 */
static int sunxi_motor_af_interpolate(const struct sunxi_motor_af_point *pts,
				      unsigned int best, u32 step)
{
	s64 y0, y1, y2, den, num, off;

	if (best == 0 || best + 1 >= SUNXI_AF_MAX_POINTS)
		return 0;

	y0 = pts[best - 1].fv;
	y1 = pts[best].fv;
	y2 = pts[best + 1].fv;

	den = y0 - 2 * y1 + y2;
	if (den >= 0)
		return 0;

	num = (y0 - y2) * (s64)step;
	off = div_s64(num, 2 * den);

	if (off > (s64)step)
		off = step;
	if (off < -(s64)step)
		off = -(s64)step;

	return (int)off;
}

static void sunxi_motor_af_work(struct work_struct *work)
{
	struct sunxi_motor *m = container_of(work, struct sunxi_motor, af_work);
	struct sunxi_motor_af_point coarse[SUNXI_AF_MAX_POINTS];
	struct sunxi_motor_af_point fine[SUNXI_AF_MAX_POINTS];
	unsigned int ncoarse = 0, nfine = 0, best;
	u32 limit, pos, fv = 0, lo, hi, target;
	u32 start, seed_pos, window;
	u32 best_pos, best_fv, samples;
	bool seeded;
	enum sunxi_motor_af_state state;
	bool flat = false;
	int ret, off;

	sunxi_motor_af_set_state(m, AF_HOMING);

	mutex_lock(&m->lock);
	ret = sunxi_motor_home_locked(m);
	mutex_unlock(&m->lock);
	if (ret)
		goto fail;

	/* Coarse sweep: walk away from the homed reference, sampling sharpness. */
	sunxi_motor_af_set_state(m, AF_COARSE);

	mutex_lock(&m->af_lock);
	seeded = m->af_seed_valid;
	seed_pos = m->af_seed_pos;
	window = m->af_seed_window;
	mutex_unlock(&m->af_lock);

	if (seeded && window) {
		start = seed_pos > window / 2 ? seed_pos - window / 2 : 0;
		limit = min_t(u32, start + window, (u32)SUNXI_AF_HARD_MAX);
	} else {
		start = 0;
		limit = m->af_max_steps;
	}

	for (pos = start; pos <= limit && ncoarse < SUNXI_AF_MAX_POINTS;
	     pos += m->af_coarse) {
		ret = sunxi_motor_af_measure(m, pos, &fv);
		if (ret)
			goto fail;

		coarse[ncoarse].pos = pos;
		coarse[ncoarse].fv = fv;
		ncoarse++;

		/* Peak at the far end: keep hunting further out. */
		if (pos == limit && limit < SUNXI_AF_HARD_MAX && fv &&
		    sunxi_motor_af_rising(coarse, ncoarse))
			limit = min(limit + m->af_max_steps,
				    (u32)SUNXI_AF_HARD_MAX);
	}

	/*
	 * A stale seed can put the peak below the seeded window; walk back to the
	 * far stop before committing to an answer.
	 */
	if (start > 0 && ncoarse > 1 &&
	    sunxi_motor_af_best(coarse, ncoarse) == 0) {
		pos = start > m->af_coarse ? start - m->af_coarse : 0;

		while (ncoarse < SUNXI_AF_MAX_POINTS) {
			ret = sunxi_motor_af_measure(m, pos, &fv);
			if (ret)
				goto fail;

			coarse[ncoarse].pos = pos;
			coarse[ncoarse].fv = fv;
			ncoarse++;

			if (!pos)
				break;
			pos = pos > m->af_coarse ? pos - m->af_coarse : 0;
		}
	}

	best = sunxi_motor_af_best(coarse, ncoarse);
	if (!coarse[best].fv) {
		/* Nothing to focus on: park at the reference position. */
		flat = true;
		ret = sunxi_motor_af_move_to(m, 0);
		if (ret)
			goto fail;
		goto done;
	}

	/* Fine sweep around the coarse peak. */
	sunxi_motor_af_set_state(m, AF_FINE);
	lo = coarse[best].pos > m->af_coarse / 2 ?
	     coarse[best].pos - m->af_coarse / 2 : 0;
	hi = min(coarse[best].pos + m->af_coarse / 2, limit);

	for (pos = lo; pos <= hi && nfine < SUNXI_AF_MAX_POINTS; pos += m->af_fine) {
		ret = sunxi_motor_af_measure(m, pos, &fv);
		if (ret)
			goto fail;

		fine[nfine].pos = pos;
		fine[nfine].fv = fv;
		nfine++;
	}

	if (fine[nfine - 1].pos != hi && nfine < SUNXI_AF_MAX_POINTS) {
		ret = sunxi_motor_af_measure(m, hi, &fv);
		if (ret)
			goto fail;

		fine[nfine].pos = hi;
		fine[nfine].fv = fv;
		nfine++;
	}

	best = sunxi_motor_af_best(fine, nfine);
	target = fine[best].pos;

	if (best > 0 && best + 1 < nfine) {
		off = sunxi_motor_af_interpolate(fine, best, m->af_fine);
		if (off > 0) {
			target = min_t(u32, target + (u32)off, hi);
		} else if (off < 0) {
			u32 back = (u32)(-off);

			target = target > back ? target - back : 0;
		}
	}

	if (target != m->pos) {
		ret = sunxi_motor_af_move_to(m, target);
		if (ret)
			goto fail;

		if (m->af_settle_ms)
			msleep(m->af_settle_ms);
	}

	mutex_lock(&m->af_lock);
	m->af_best_pos = target;
	m->af_best_fv = fine[best].fv;
	mutex_unlock(&m->af_lock);

done:
	mutex_lock(&m->af_lock);
	m->af_running = false;
	m->af_state = flat ? AF_FLAT : (m->af_cancel ? AF_CANCELLED : AF_DONE);
	state = m->af_state;
	best_pos = m->af_best_pos;
	best_fv = m->af_best_fv;
	samples = m->af_samples;
	complete(&m->af_done);
	mutex_unlock(&m->af_lock);

	dev_info(m->dev, "autofocus %s: best %u @ %u (%u samples)\n",
		 af_state_names[state], best_fv, best_pos, samples);
	return;

fail:
	mutex_lock(&m->af_lock);
	m->af_running = false;
	if (ret == -ENODEV)
		m->af_state = AF_NO_SOURCE;
	else if (ret == -ETIMEDOUT)
		m->af_state = AF_TIMEOUT;
	else if (ret == -ECANCELED)
		m->af_state = AF_CANCELLED;
	else
		m->af_state = AF_ERROR;
	complete(&m->af_done);
	mutex_unlock(&m->af_lock);

	dev_dbg(m->dev, "autofocus stopped: %d (%s)\n", ret,
		af_state_names[m->af_state]);
}

static int sunxi_motor_af_start(struct sunxi_motor *m)
{
	if (!m->stepper)
		return -EOPNOTSUPP;

	mutex_lock(&m->af_lock);
	if (m->af_running) {
		mutex_unlock(&m->af_lock);
		return -EBUSY;
	}
	if (!m->af_bridge && !sunxi_motor_af_source_get()) {
		m->af_state = AF_NO_SOURCE;
		mutex_unlock(&m->af_lock);
		return -ENODEV;
	}
	m->af_running = true;
	m->af_cancel = false;
	m->af_best_pos = 0;
	m->af_best_fv = 0;
	m->af_samples = 0;
	m->af_state = AF_IDLE;
	mutex_unlock(&m->af_lock);

	queue_work(system_wq, &m->af_work);

	return 0;
}

static void sunxi_motor_af_cancel(struct sunxi_motor *m)
{
	bool running;

	mutex_lock(&m->af_cancel_lock);
	mutex_lock(&m->af_lock);
	running = m->af_running;
	if (running)
		m->af_cancel = true;
	mutex_unlock(&m->af_lock);

	if (running) {
		/* Release a bridge waiter so it notices the cancellation. */
		complete(&m->af_sample);
		wait_for_completion(&m->af_done);
	}
	mutex_unlock(&m->af_cancel_lock);
}

int sun50iw15_motor_start_autofocus(void)
{
	struct sunxi_motor *m = sunxi_motor_dev;

	if (!m)
		return -ENODEV;

	return sunxi_motor_af_start(m);
}
EXPORT_SYMBOL_GPL(sun50iw15_motor_start_autofocus);

void sun50iw15_motor_cancel_autofocus(void)
{
	struct sunxi_motor *m = sunxi_motor_dev;

	if (m)
		sunxi_motor_af_cancel(m);
}
EXPORT_SYMBOL_GPL(sun50iw15_motor_cancel_autofocus);

int sun50iw15_motor_af_busy(void)
{
	struct sunxi_motor *m = sunxi_motor_dev;
	bool busy;

	if (!m)
		return 0;

	mutex_lock(&m->af_lock);
	busy = m->af_running;
	mutex_unlock(&m->af_lock);

	return busy;
}
EXPORT_SYMBOL_GPL(sun50iw15_motor_af_busy);

int sun50iw15_motor_register_af_source(const struct sun50iw15_motor_af_source *src)
{
	int ret = 0;

	if (!src || !src->name || !src->sample)
		return -EINVAL;

	mutex_lock(&af_source_lock);
	if (af_source && af_source != src)
		ret = -EBUSY;
	else
		af_source = src;
	mutex_unlock(&af_source_lock);

	return ret;
}
EXPORT_SYMBOL_GPL(sun50iw15_motor_register_af_source);

void sun50iw15_motor_unregister_af_source(const struct sun50iw15_motor_af_source *src)
{
	mutex_lock(&af_source_lock);
	if (af_source == src)
		af_source = NULL;
	mutex_unlock(&af_source_lock);
}
EXPORT_SYMBOL_GPL(sun50iw15_motor_unregister_af_source);

/**
 * sun50iw15_motor_focus_value - Sobel gradient focus value for a luma frame
 * @luma: 8-bit greyscale frame
 * @width: frame width in pixels
 * @height: frame height in pixels
 * @stride: row stride in bytes
 *
 * Returns the mean sum of absolute Sobel gradients, a standard contrast
 * metric: it rises as edges get sharper and is largest at best focus.
 */
u32 sun50iw15_motor_focus_value(const u8 *luma, u32 width, u32 height,
				u32 stride)
{
	u64 sum = 0;
	u32 x, y, n = 0;

	if (!luma || width < 3 || height < 3 || stride < width)
		return 0;

	for (y = 1; y + 1 < height; y++) {
		const u8 *r0 = luma + (y - 1) * stride;
		const u8 *r1 = luma + y * stride;
		const u8 *r2 = luma + (y + 1) * stride;

		for (x = 1; x + 1 < width; x++) {
			int gx = (int)r0[x + 1] + 2 * (int)r1[x + 1] + (int)r2[x + 1] -
				 (int)r0[x - 1] - 2 * (int)r1[x - 1] - (int)r2[x - 1];
			int gy = (int)r2[x - 1] + 2 * (int)r2[x] + (int)r2[x + 1] -
				 (int)r0[x - 1] - 2 * (int)r0[x] - (int)r0[x + 1];

			sum += abs(gx) + abs(gy);
			n++;
		}
	}

	if (!n)
		return 0;

	return (u32)div_u64(sum, n);
}
EXPORT_SYMBOL_GPL(sun50iw15_motor_focus_value);

/**
 * sun50iw15_motor_vafocus - autofocus the lens
 *
 * Starts a full contrast search when a sharpness source (kernel or the
 * userspace bridge) is available, and falls back to the vendor's homing sweep
 * otherwise.  Returns immediately; the search runs on system_wq.
 */
int sun50iw15_motor_vafocus(void)
{
	struct sunxi_motor *m = sunxi_motor_dev;
	int ret;

	if (!m)
		return -ENODEV;

	if (!m->stepper)
		return -EOPNOTSUPP;

	if (!m->af_bridge && !sunxi_motor_af_source_get())
		return sunxi_motor_home(m);

	ret = sunxi_motor_af_start(m);
	if (ret == -EBUSY)
		return 0;

	return ret;
}
EXPORT_SYMBOL_GPL(sun50iw15_motor_vafocus);


/* ---- focus seeding ------------------------------------------------------ */

/*
 * Distance to lens position.  The table holds (mm, steps) pairs sorted by
 * distance and is interpolated linearly; a nearer subject maps to a larger
 * step count.  Direction and scale are board facts, so the default table is
 * only a starting point and boards override it with af-seed-table.
 */
static u32 sunxi_motor_seed_lookup(struct sunxi_motor *m, u32 mm)
{
	const u32 *t = m->af_seed_table;
	u32 n = m->af_seed_points;
	u32 i;

	if (n < 2)
		return 0;

	if (mm <= t[0])
		return t[1];
	if (mm >= t[(n - 1) * 2])
		return t[(n - 1) * 2 + 1];

	for (i = 1; i < n; i++) {
		u32 lo = t[(i - 1) * 2];
		u32 hi = t[i * 2];
		s64 a = t[(i - 1) * 2 + 1];
		s64 b = t[i * 2 + 1];

		if (mm > hi)
			continue;

		if (hi <= lo)
			return (u32)b;

		return (u32)(a + div_s64((b - a) * (s64)(mm - lo),
					 (s64)(hi - lo)));
	}

	return t[(n - 1) * 2 + 1];
}

/* Park the lens at the seeded position, homing first if the reference is not
 * known yet.  Runs from system_wq so a sysfs write does not block on the
 * ~1.3 s homing sweep. */
static void sunxi_motor_seed_work(struct work_struct *work)
{
	struct sunxi_motor *m = container_of(work, struct sunxi_motor, seed_work);
	u32 target;
	bool valid;

	mutex_lock(&m->af_lock);
	valid = m->af_seed_valid;
	target = m->af_seed_pos;
	mutex_unlock(&m->af_lock);

	if (!valid || !m->stepper || sun50iw15_motor_af_busy())
		return;

	mutex_lock(&m->lock);
	if (!m->homed)
		sunxi_motor_home_locked(m);
	mutex_unlock(&m->lock);

	if (sun50iw15_motor_af_busy())
		return;

	/* move_to() refuses to move while a cancel is pending; a cancelled run
	 * leaves the flag set, and there is no run to cancel here. */
	mutex_lock(&m->af_lock);
	m->af_cancel = false;
	mutex_unlock(&m->af_lock);

	sunxi_motor_af_move_to(m, min_t(u32, target, SUNXI_AF_HARD_MAX));
}

static void sunxi_motor_seed_apply(struct sunxi_motor *m, u32 mm, u32 pos)
{
	bool preposition;

	mutex_lock(&m->af_lock);
	m->af_seed_mm = mm;
	m->af_seed_pos = min_t(u32, pos, SUNXI_AF_HARD_MAX);
	m->af_seed_valid = true;
	preposition = m->af_preposition && m->stepper;
	mutex_unlock(&m->af_lock);

	if (preposition)
		queue_work(system_wq, &m->seed_work);
}

static void sunxi_motor_seed_forget(struct sunxi_motor *m)
{
	mutex_lock(&m->af_lock);
	m->af_seed_valid = false;
	m->af_seed_mm = 0;
	m->af_seed_pos = 0;
	mutex_unlock(&m->af_lock);
}

int sun50iw15_motor_seed_distance(u32 distance_mm)
{
	struct sunxi_motor *m = sunxi_motor_dev;

	if (!m)
		return -ENODEV;

	sunxi_motor_seed_apply(m, distance_mm,
			       sunxi_motor_seed_lookup(m, distance_mm));

	return 0;
}
EXPORT_SYMBOL_GPL(sun50iw15_motor_seed_distance);

int sun50iw15_motor_seed_steps(u32 position_steps)
{
	struct sunxi_motor *m = sunxi_motor_dev;

	if (!m)
		return -ENODEV;

	sunxi_motor_seed_apply(m, 0, position_steps);

	return 0;
}
EXPORT_SYMBOL_GPL(sun50iw15_motor_seed_steps);

void sun50iw15_motor_seed_clear(void)
{
	struct sunxi_motor *m = sunxi_motor_dev;

	if (m)
		sunxi_motor_seed_forget(m);
}
EXPORT_SYMBOL_GPL(sun50iw15_motor_seed_clear);

/* ---- sysfs -------------------------------------------------------------- */

static ssize_t motor_ctrl_show(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u %u %u\n", m->step_num, m->back_step,
			  sunxi_motor_limited(m));
}

static ssize_t motor_ctrl_store(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);
	unsigned long val;
	unsigned int cmd, arg;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	/* Same encoding as the vendor: 0xCCAA, command in the high byte. */
	cmd = (val >> 8) & 0xff;
	arg = val & 0xff;

	if (!m->stepper)
		return -EOPNOTSUPP;

	switch (cmd) {
	case 1:		/* drive up */
	case 2:		/* drive down */
		sunxi_motor_af_cancel(m);
		mutex_lock(&m->lock);
		sunxi_motor_mstep(m, cmd == 1, arg ? arg : 1);
		if (cmd == 1)
			m->pos = arg > m->pos ? 0 : m->pos - arg;
		else
			m->pos += arg ? arg : 1;
		mutex_unlock(&m->lock);
		break;
	case 3:		/* reset position counter */
		sunxi_motor_af_cancel(m);
		mutex_lock(&m->lock);
		m->pos = 0;
		mutex_unlock(&m->lock);
		break;
	case 7:		/* goto absolute step */
		sunxi_motor_af_cancel(m);
		if (sunxi_motor_af_move_to(m, min_t(unsigned int,
						    arg ? arg : 0,
						    SUNXI_AF_HARD_MAX)))
			return -EIO;
		break;
	case 8:		/* start contrast autofocus */
		if (sunxi_motor_af_start(m) == -ENODEV)
			return -ENODEV;
		break;
	case 9:		/* abort autofocus */
		sunxi_motor_af_cancel(m);
		break;
	default:
		return -EINVAL;
	}

	return count;
}
static DEVICE_ATTR_RW(motor_ctrl);

static ssize_t motor_limit_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", sunxi_motor_limited(m));
}
static DEVICE_ATTR_RO(motor_limit);

static ssize_t af_start_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);
	int val, ret;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	if (!val) {
		sunxi_motor_af_cancel(m);
		return count;
	}

	ret = sunxi_motor_af_start(m);
	if (ret && ret != -EBUSY)
		return ret;

	return count;
}
static DEVICE_ATTR_WO(af_start);

static ssize_t af_bridge_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", m->af_bridge);
}

static ssize_t af_bridge_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);
	int val;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	m->af_bridge = !!val;

	return count;
}
static DEVICE_ATTR_RW(af_bridge);

static ssize_t af_metric_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);
	u32 val;

	if (kstrtou32(buf, 0, &val))
		return -EINVAL;

	mutex_lock(&m->af_lock);
	if (!m->af_pending) {
		mutex_unlock(&m->af_lock);
		return -EINVAL;
	}
	m->af_metric_val = val;
	m->af_got = true;
	m->af_pending = false;
	mutex_unlock(&m->af_lock);

	complete(&m->af_sample);

	return count;
}
static DEVICE_ATTR_WO(af_metric);

static ssize_t af_awaiting_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);
	long pos;

	mutex_lock(&m->af_lock);
	pos = m->af_pending ? m->af_pending_pos : -1;
	mutex_unlock(&m->af_lock);

	return sysfs_emit(buf, "%ld\n", pos);
}
static DEVICE_ATTR_RO(af_awaiting);

static ssize_t af_status_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);
	enum sunxi_motor_af_state state;
	u32 pos, fv, samples;

	mutex_lock(&m->af_lock);
	state = m->af_state;
	fv = m->af_last_fv;
	samples = m->af_samples;
	mutex_unlock(&m->af_lock);

	pos = READ_ONCE(m->pos);

	return sysfs_emit(buf, "%s pos=%u fv=%u samples=%u\n",
			  af_state_names[state], pos, fv, samples);
}
static DEVICE_ATTR_RO(af_status);

static ssize_t af_result_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);
	u32 pos, fv, samples;

	mutex_lock(&m->af_lock);
	pos = m->af_best_pos;
	fv = m->af_best_fv;
	samples = m->af_samples;
	mutex_unlock(&m->af_lock);

	return sysfs_emit(buf, "%u %u %u\n", pos, fv, samples);
}
static DEVICE_ATTR_RO(af_result);

static ssize_t af_source_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	const struct sun50iw15_motor_af_source *src = sunxi_motor_af_source_get();

	return sysfs_emit(buf, "%s\n", src ? src->name : "none");
}
static DEVICE_ATTR_RO(af_source);

static ssize_t af_coarse_step_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);
	u32 val;

	if (kstrtou32(buf, 0, &val) || !val)
		return -EINVAL;

	m->af_coarse = val;

	return count;
}

static ssize_t af_coarse_step_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", m->af_coarse);
}
static DEVICE_ATTR_RW(af_coarse_step);

static ssize_t af_fine_step_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);
	u32 val;

	if (kstrtou32(buf, 0, &val) || !val)
		return -EINVAL;

	m->af_fine = val;

	return count;
}

static ssize_t af_fine_step_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", m->af_fine);
}
static DEVICE_ATTR_RW(af_fine_step);

static ssize_t af_max_steps_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);
	u32 val;

	if (kstrtou32(buf, 0, &val) || !val)
		return -EINVAL;

	m->af_max_steps = min_t(u32, val, SUNXI_AF_HARD_MAX);

	return count;
}

static ssize_t af_max_steps_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", m->af_max_steps);
}
static DEVICE_ATTR_RW(af_max_steps);

static ssize_t af_settle_ms_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);
	u32 val;

	if (kstrtou32(buf, 0, &val))
		return -EINVAL;

	m->af_settle_ms = min_t(u32, val, 2000);

	return count;
}

static ssize_t af_settle_ms_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", m->af_settle_ms);
}
static DEVICE_ATTR_RW(af_settle_ms);

static ssize_t af_timeout_ms_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);
	u32 val;

	if (kstrtou32(buf, 0, &val) || !val)
		return -EINVAL;

	m->af_timeout_ms = min_t(u32, val, 30000);

	return count;
}

static ssize_t af_timeout_ms_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", m->af_timeout_ms);
}
static DEVICE_ATTR_RW(af_timeout_ms);

static ssize_t af_seed_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);
	u32 mm, pos;
	bool valid;

	mutex_lock(&m->af_lock);
	valid = m->af_seed_valid;
	mm = m->af_seed_mm;
	pos = m->af_seed_pos;
	mutex_unlock(&m->af_lock);

	if (!valid)
		return sysfs_emit(buf, "none\n");

	return sysfs_emit(buf, "%u %u\n", mm, pos);
}

static ssize_t af_seed_store(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);
	int mm;

	if (kstrtoint(buf, 0, &mm))
		return -EINVAL;

	if (mm < 0) {
		sunxi_motor_seed_forget(m);
		return count;
	}

	sunxi_motor_seed_apply(m, mm, sunxi_motor_seed_lookup(m, mm));

	return count;
}
static DEVICE_ATTR_RW(af_seed);

static ssize_t af_seed_pos_show(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);
	u32 pos;
	bool valid;

	mutex_lock(&m->af_lock);
	valid = m->af_seed_valid;
	pos = m->af_seed_pos;
	mutex_unlock(&m->af_lock);

	return sysfs_emit(buf, "%d\n", valid ? (int)pos : -1);
}
static DEVICE_ATTR_RO(af_seed_pos);

static ssize_t af_seed_window_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", m->af_seed_window);
}

static ssize_t af_seed_window_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct sunxi_motor *m = dev_get_drvdata(dev);
	u32 val;

	if (kstrtou32(buf, 0, &val) || !val)
		return -EINVAL;

	m->af_seed_window = min_t(u32, val, SUNXI_AF_HARD_MAX);

	return count;
}
static DEVICE_ATTR_RW(af_seed_window);

static struct attribute *sunxi_motor_attrs[] = {
	&dev_attr_motor_ctrl.attr,
	&dev_attr_motor_limit.attr,
	&dev_attr_af_start.attr,
	&dev_attr_af_bridge.attr,
	&dev_attr_af_metric.attr,
	&dev_attr_af_awaiting.attr,
	&dev_attr_af_status.attr,
	&dev_attr_af_result.attr,
	&dev_attr_af_source.attr,
	&dev_attr_af_coarse_step.attr,
	&dev_attr_af_fine_step.attr,
	&dev_attr_af_max_steps.attr,
	&dev_attr_af_settle_ms.attr,
	&dev_attr_af_timeout_ms.attr,
	&dev_attr_af_seed.attr,
	&dev_attr_af_seed_pos.attr,
	&dev_attr_af_seed_window.attr,
	NULL,
};
ATTRIBUTE_GROUPS(sunxi_motor);

/* ---- character device --------------------------------------------------- */

static int sunxi_motor_open(struct inode *inode, struct file *file)
{
	struct sunxi_motor *m = container_of(inode->i_cdev, struct sunxi_motor,
					     cdev);

	file->private_data = m;
	atomic_inc(&m->open_count);
	dev_dbg(m->dev, "motor-limiter open, count %d\n",
		atomic_read(&m->open_count));

	return 0;
}

static int sunxi_motor_release(struct inode *inode, struct file *file)
{
	struct sunxi_motor *m = file->private_data;

	atomic_dec(&m->open_count);
	return sunxi_motor_fasync(-1, file, 0);
}

static ssize_t sunxi_motor_read(struct file *file, char __user *buf,
				size_t len, loff_t *ppos)
{
	struct sunxi_motor *m = file->private_data;
	int state = sunxi_motor_limited(m);

	if (len < sizeof(state))
		return -EINVAL;
	if (*ppos)
		return 0;

	if (put_user(state, (int __user *)buf))
		return -EFAULT;

	*ppos = sizeof(state);

	return sizeof(state);
}

static __poll_t sunxi_motor_poll(struct file *file, poll_table *wait)
{
	struct sunxi_motor *m = file->private_data;

	return sunxi_motor_limited(m) ? EPOLLIN | EPOLLRDNORM : 0;
}

static int sunxi_motor_fasync(int fd, struct file *file, int on)
{
	struct sunxi_motor *m = file->private_data;

	return fasync_helper(fd, file, on, &m->async);
}

static const struct file_operations sunxi_motor_fops = {
	.owner		= THIS_MODULE,
	.open		= sunxi_motor_open,
	.release	= sunxi_motor_release,
	.read		= sunxi_motor_read,
	.poll		= sunxi_motor_poll,
	.fasync		= sunxi_motor_fasync,
	.llseek		= no_llseek,
};

static irqreturn_t sunxi_motor_irq(int irq, void *data)
{
	struct sunxi_motor *m = data;

	if (m->async)
		kill_fasync(&m->async, SIGIO, POLL_IN);

	return IRQ_HANDLED;
}

/* ---- probe -------------------------------------------------------------- */

static void sunxi_motor_read_table(struct device_node *np, const char *prop,
				   u8 *dst, unsigned int *dst_len)
{
	int n, i;
	u32 *cells;

	n = of_property_count_u32_elems(np, prop);
	if (n <= 0)
		return;

	cells = kcalloc(n, sizeof(*cells), GFP_KERNEL);
	if (!cells)
		return;

	if (of_property_read_u32_array(np, prop, cells, n)) {
		kfree(cells);
		return;
	}

	for (i = 0; i < n && (i * 4) < SUNXI_MOTOR_MAX_STEPS; i++) {
		dst[i * 4 + 0] = cells[i] & 0xff;
		dst[i * 4 + 1] = (cells[i] >> 8) & 0xff;
		dst[i * 4 + 2] = (cells[i] >> 16) & 0xff;
		dst[i * 4 + 3] = (cells[i] >> 24) & 0xff;
	}

	*dst_len = min_t(unsigned int, n * 4, SUNXI_MOTOR_MAX_STEPS);
	kfree(cells);
}

static int sunxi_motor_parse_gpios(struct platform_device *pdev,
				   struct sunxi_motor *m)
{
	struct device_node *np = pdev->dev.of_node;
	struct device *dev = &pdev->dev;
	unsigned int i;
	char name[32];
	u32 val;
	int gpio;

	if (!of_property_read_u32(np, "motor-phase-num", &val))
		m->phase_num = min_t(unsigned int, val, SUNXI_MOTOR_MAX_PHASES);
	if (!m->phase_num)
		m->phase_num = 4;

	for (i = 0; i < m->phase_num; i++) {
		snprintf(name, sizeof(name), "motor-gpio-phase%u", i);
		gpio = of_get_named_gpio(np, name, 0);
		if (gpio < 0)
			return dev_err_probe(dev, gpio, "missing %s\n", name);

		if (devm_gpio_request(dev, gpio, name))
			return dev_err_probe(dev, -EBUSY, "cannot request %s\n",
					     name);

		m->phase[i] = gpio_to_desc(gpio);
		gpiod_direction_output_raw(m->phase[i], 0);
	}

	gpio = of_get_named_gpio(np, "limiter-up-gpio", 0);
	if (gpio == -ENOENT)
		gpio = of_get_named_gpio(np, "limiter-gpio", 0);
	if (gpio < 0)
		return dev_err_probe(dev, gpio, "missing limiter GPIO\n");

	if (devm_gpio_request(dev, gpio, "limiter-gpio"))
		return dev_err_probe(dev, -EBUSY, "cannot request limiter GPIO\n");

	m->limit = gpio_to_desc(gpio);
	gpiod_direction_input(m->limit);

	return 0;
}

static void sunxi_motor_parse_seed_table(struct device_node *np,
					 struct sunxi_motor *m)
{
	static const u32 defaults[] = {
		0, 400, 400, 330, 700, 260, 1000, 200,
		1500, 130, 2000, 90, 3000, 45, 6000, 10,
	};
	u32 cells[SUNXI_AF_SEED_POINTS * 2];
	int n;

	n = of_property_count_u32_elems(np, "af-seed-table");
	if (n >= 4 && !(n & 1) && n <= (int)ARRAY_SIZE(cells) &&
	    !of_property_read_u32_array(np, "af-seed-table", cells, n)) {
		memcpy(m->af_seed_table, cells, n * sizeof(u32));
		m->af_seed_points = n / 2;
		return;
	}

	memcpy(m->af_seed_table, defaults, sizeof(defaults));
	m->af_seed_points = ARRAY_SIZE(defaults) / 2;
}

static int sunxi_motor_parse_params(struct platform_device *pdev,
				    struct sunxi_motor *m)
{
	struct device_node *np = pdev->dev.of_node;
	u32 val;

	if (!of_property_read_u32(np, "motor_type", &val) && val == 1) {
		dev_warn(&pdev->dev,
			 "DC motor mode (motor_type=1) is not supported\n");
		m->stepper = false;
	} else {
		m->stepper = true;
	}

	sunxi_motor_read_table(np, "motor-cw-table", m->cw, &m->step_num);

	if (!of_property_read_u32(np, "motor-step-num", &val) && val &&
	    val <= SUNXI_MOTOR_MAX_STEPS)
		m->step_num = min(m->step_num ? m->step_num : val, val);

	if (!m->step_num)
		m->step_num = 8;

	m->cycle = 1;
	if (!of_property_read_u32(np, "motor-cycle", &val) && val)
		m->cycle = min_t(unsigned int, val, 10);

	m->phase_udelay = 1;
	of_property_read_u32(np, "motor-phase-udelay", &m->phase_udelay);

	m->step_mdelay = 1;
	of_property_read_u32(np, "motor-step-mdelay", &m->step_mdelay);

	m->active_level = 1;
	of_property_read_u32(np, "active_level", &m->active_level);

	m->back_step = 5;
	of_property_read_u32(np, "motor-back-step", &m->back_step);

	/* Keep the ccw table as long as cw unless the DT gives one. */
	if (of_property_count_u32_elems(np, "motor-ccw-table") > 0) {
		unsigned int len = 0;

		sunxi_motor_read_table(np, "motor-ccw-table", m->ccw, &len);
	} else {
		memcpy(m->ccw, m->cw, sizeof(m->ccw));
	}

	/* Contrast autofocus tuning. */
	m->af_coarse = 8;
	of_property_read_u32(np, "af-coarse-step", &m->af_coarse);

	m->af_fine = 2;
	of_property_read_u32(np, "af-fine-step", &m->af_fine);

	m->af_max_steps = 120;
	of_property_read_u32(np, "af-max-steps", &m->af_max_steps);

	m->af_settle_ms = 40;
	of_property_read_u32(np, "af-settle-ms", &m->af_settle_ms);

	m->af_timeout_ms = 2000;
	of_property_read_u32(np, "af-timeout-ms", &m->af_timeout_ms);

	m->af_bridge = of_property_read_bool(np, "af-bridge");

	/* Distance seeding: af-seed-table = <mm steps ...>, nearest first. */
	sunxi_motor_parse_seed_table(np, m);

	m->af_seed_window = m->af_max_steps * 2;
	of_property_read_u32(np, "af-seed-window", &m->af_seed_window);
	m->af_seed_window = clamp_t(u32, m->af_seed_window, m->af_coarse,
				    SUNXI_AF_HARD_MAX);

	m->af_preposition = !of_property_read_bool(np, "af-seed-no-preposition");

	m->af_coarse = clamp_t(u32, m->af_coarse, 1, SUNXI_AF_HARD_MAX);
	m->af_fine = clamp_t(u32, m->af_fine, 1, m->af_coarse);
	m->af_max_steps = clamp_t(u32, m->af_max_steps, m->af_coarse,
				  SUNXI_AF_HARD_MAX);
	m->af_settle_ms = min_t(u32, m->af_settle_ms, 2000);
	m->af_timeout_ms = clamp_t(u32, m->af_timeout_ms, 100, 30000);

	return 0;
}

static int sunxi_motor_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sunxi_motor *m;
	int ret;

	m = devm_kzalloc(dev, sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;

	m->dev = dev;
	mutex_init(&m->lock);
	mutex_init(&m->af_lock);
	mutex_init(&m->af_cancel_lock);
	init_completion(&m->af_done);
	init_completion(&m->af_sample);
	INIT_WORK(&m->seed_work, sunxi_motor_seed_work);
	INIT_WORK(&m->af_work, sunxi_motor_af_work);
	m->af_state = AF_IDLE;
	atomic_set(&m->open_count, 0);

	ret = sunxi_motor_parse_gpios(pdev, m);
	if (ret)
		return ret;

	ret = sunxi_motor_parse_params(pdev, m);
	if (ret)
		return ret;

	sunxi_motor_deenergize(m);

	if (m->limit) {
		m->limit_irq = gpiod_to_irq(m->limit);
		if (m->limit_irq > 0) {
			ret = devm_request_threaded_irq(dev, m->limit_irq, NULL,
							sunxi_motor_irq,
							IRQF_TRIGGER_RISING |
							IRQF_TRIGGER_FALLING |
							IRQF_ONESHOT,
							"motor-limiter", m);
			if (ret) {
				dev_warn(dev, "cannot request limiter IRQ: %d\n",
					 ret);
				m->limit_irq = 0;
			}
		}
	}

	ret = alloc_chrdev_region(&m->devt, 0, 1, "motor_limiter");
	if (ret)
		return dev_err_probe(dev, ret, "cannot allocate chrdev\n");

	cdev_init(&m->cdev, &sunxi_motor_fops);
	m->cdev.owner = THIS_MODULE;
	ret = cdev_add(&m->cdev, m->devt, 1);
	if (ret)
		goto err_region;

	m->cls = class_create(THIS_MODULE, "motor-limiter");
	if (IS_ERR(m->cls)) {
		ret = PTR_ERR(m->cls);
		goto err_cdev;
	}

	m->chrdev = device_create(m->cls, NULL, m->devt, m, "motor_limiter");
	if (IS_ERR(m->chrdev)) {
		ret = PTR_ERR(m->chrdev);
		goto err_class;
	}

	/* init.input.rc chmods /dev/motor0 as well; keep the name available. */
	m->motor0 = device_create(m->cls, NULL, m->devt, m, "motor0");
	if (IS_ERR(m->motor0))
		m->motor0 = NULL;

	platform_set_drvdata(pdev, m);
	sunxi_motor_dev = m;

	dev_info(dev, "motor limiter ready: %u phases, %u steps, af %u/%u/%u, limit %s\n",
		 m->phase_num, m->step_num, m->af_coarse, m->af_fine,
		 m->af_max_steps, sunxi_motor_limited(m) ? "closed" : "open");

	return 0;

err_class:
	class_destroy(m->cls);
err_cdev:
	cdev_del(&m->cdev);
err_region:
	unregister_chrdev_region(m->devt, 1);
	return ret;
}

static int sunxi_motor_remove(struct platform_device *pdev)
{
	struct sunxi_motor *m = platform_get_drvdata(pdev);

	sunxi_motor_af_cancel(m);
	cancel_work_sync(&m->af_work);
	cancel_work_sync(&m->seed_work);
	sunxi_motor_dev = NULL;
	if (m->motor0)
		device_destroy(m->cls, m->devt);
	device_destroy(m->cls, m->devt);
	class_destroy(m->cls);
	cdev_del(&m->cdev);
	unregister_chrdev_region(m->devt, 1);
	sunxi_motor_deenergize(m);

	return 0;
}

static const struct of_device_id sunxi_motor_of_match[] = {
	{ .compatible = "motor-control" },
	{ .compatible = "allwinner,motor-limiter" },
	{ }
};
MODULE_DEVICE_TABLE(of, sunxi_motor_of_match);

static struct platform_driver sunxi_motor_driver = {
	.probe = sunxi_motor_probe,
	.remove = sunxi_motor_remove,
	.driver = {
		.name = "sunxi-motor",
		.of_match_table = sunxi_motor_of_match,
		.dev_groups = sunxi_motor_groups,
	},
};
module_platform_driver(sunxi_motor_driver);

MODULE_AUTHOR("H726 kernel rewrite");
MODULE_DESCRIPTION("Allwinner sun50iw15 focus motor, limiter and autofocus");
MODULE_LICENSE("GPL");
