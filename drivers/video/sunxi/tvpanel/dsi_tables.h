/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Static tables lifted from the vendor tvpanel.ko .data section.
 * GENERATED - do not hand-edit (values are read straight out of the ELF).
 *
 *   dsi_pixel_bits[4]  .data+0x1e4  bits per pixel, indexed by lcd_dsi_format
 *   dsi_lane_den[4]    .data+0x1f4  lane-enable masks, indexed by lcd_dsi_lane-1
 *   tcon_div           .data+0x204  TCON divider used for the DSI DRQ watermark
 */
#ifndef __TVPANEL_DSI_TABLES_H
#define __TVPANEL_DSI_TABLES_H

#include <linux/types.h>

static const u32 dsi_pixel_bits[4] = {
	24, 24, 18, 16,
};

static const u32 dsi_lane_den[4] = {
	0x1, 0x3, 0x7, 0xf,
};

#define TVPANEL_TCON_DIV 4

#endif /* __TVPANEL_DSI_TABLES_H */
