/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Public interface of the Allwinner sun50iw15 focus motor driver.
 *
 * Consumers: the STK83xx accelerometer (shake -> autofocus), and any driver or
 * board code that knows where the subject is - a time-of-flight range sensor,
 * for example, seeded through sun50iw15_motor_seed_distance().
 */
#ifndef _LINUX_SUN50IW15_MOTOR_H
#define _LINUX_SUN50IW15_MOTOR_H

#include <linux/kconfig.h>
#include <linux/types.h>

/*
 * Pluggable sharpness source for the contrast autofocus search.  sample() is
 * called from the autofocus worker with the lens already at the position being
 * measured; it returns 0 and a metric where a larger value means sharper.
 */
struct sun50iw15_motor_af_source {
	const char *name;
	int (*sample)(void *priv, u32 *focus_value);
	void *priv;
};

#if IS_ENABLED(CONFIG_SUN50IW15_MOTOR)

int sun50iw15_motor_limited(void);
int sun50iw15_motor_vafocus(void);
int sun50iw15_motor_start_autofocus(void);
void sun50iw15_motor_cancel_autofocus(void);
int sun50iw15_motor_af_busy(void);
int sun50iw15_motor_register_af_source(const struct sun50iw15_motor_af_source *src);
void sun50iw15_motor_unregister_af_source(const struct sun50iw15_motor_af_source *src);
u32 sun50iw15_motor_focus_value(const u8 *luma, u32 width, u32 height,
				u32 stride);

/*
 * Seed the next autofocus search from a measured subject distance.  The
 * distance is mapped to a lens position through the af-seed-table device tree
 * property (or the built-in default table); the search then only sweeps the
 * window around that position instead of the whole travel.
 */
int sun50iw15_motor_seed_distance(u32 distance_mm);
/* Seed with a lens position directly, for callers with their own mapping. */
int sun50iw15_motor_seed_steps(u32 position_steps);
void sun50iw15_motor_seed_clear(void);

#else /* !CONFIG_SUN50IW15_MOTOR */

static inline int sun50iw15_motor_limited(void) { return 0; }
static inline int sun50iw15_motor_vafocus(void) { return -ENODEV; }
static inline int sun50iw15_motor_start_autofocus(void) { return -ENODEV; }
static inline void sun50iw15_motor_cancel_autofocus(void) { }
static inline int sun50iw15_motor_af_busy(void) { return 0; }
static inline int
sun50iw15_motor_register_af_source(const struct sun50iw15_motor_af_source *src)
{
	return -ENODEV;
}
static inline void
sun50iw15_motor_unregister_af_source(const struct sun50iw15_motor_af_source *src) { }
static inline u32 sun50iw15_motor_focus_value(const u8 *luma, u32 width,
					      u32 height, u32 stride)
{
	return 0;
}
static inline int sun50iw15_motor_seed_distance(u32 distance_mm) { return -ENODEV; }
static inline int sun50iw15_motor_seed_steps(u32 position_steps) { return -ENODEV; }
static inline void sun50iw15_motor_seed_clear(void) { }

#endif /* CONFIG_SUN50IW15_MOTOR */

#endif /* _LINUX_SUN50IW15_MOTOR_H */
