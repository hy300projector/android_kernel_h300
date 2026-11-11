// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allwinner H726 tvpanel - DSI mode programming.
 *
 * Port of the vendor dsi_cfg(): given a disp_panel_para it programmes pixel
 * format, video/burst/single mode, timings, the virtual-channel packet
 * headers, the instruction-sequencer lane setup and the D-PHY timers.
 *
 * The vendor version indexes raw words ("v7[16] = 805306370"); here every
 * access is a named field of the BTF-derived block layout, so the magic
 * offsets are gone and the intent is visible.
 */
#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/module.h>

#include "tvpanel.h"
#include "tvpanel_regs.h"
#include "dsi_tables.h"

/* ------------------------------------------------------------------ */
/* packet helpers                                                      */
/* ------------------------------------------------------------------ */

/* tvpanel_dsi_ecc()/tvpanel_dsi_crc() live in tvpanel_dsi.c: the DCS packet
 * builders in the command layer need the same helpers.
 */

/*
 * One entry of the instruction table:
 *   lane_den[3:0], lane_cen[4], trans_packet[23:20], instru_mode[31:28].
 */
static void dsi_inst_setup(union dsi_basic_inst0_reg_t __iomem *reg,
			   u32 lane_den, unsigned int mode, bool cen,
			   bool trans_packet, bool escape_clear)
{
	u32 v = readl(reg);

	v &= 0x0fffffffu;		/* instru_mode = 0 */
	if (escape_clear)
		v &= 0xf0ffffffu;	/* escape_enrty = 0 */
	if (trans_packet)
		v = (v & 0xff0fffffu) | (1u << 20);
	v = (v & ~0x1fu) | (lane_den & 0xfu);
	if (cen)
		v |= BIT(4);
	v |= ((u32)mode & 0xfu) << 28;
	writel(v, reg);
}

/* short packet: dt set, d0/d1 cleared, ECC recomputed */
static void dsi_short_pkt(union dsi_short_pkg_reg_t __iomem *reg, u32 dt)
{
	u32 v = readl(reg);

	v &= 0xffffff3fu;		/* dt = 0 */
	v |= dt & 0x3fu;
	v &= 0xffff00ffu;		/* d0 = 0 */
	v &= 0xff00ffffu;		/* d1 = 0 */
	v = (v & 0xffffffu) | ((u32)tvpanel_dsi_ecc(v) << 24);
	writel(v, reg);
}

/* long packet: dt = 0x19, wc, ECC; payload CRC goes in the pf field */
static void dsi_blk_pkt(union dsi_blk_pkg0_reg_t __iomem *pkg0,
			union dsi_blk_pkg1_reg_t __iomem *pkg1, u32 wc)
{
	u32 v = readl(pkg0);

	v &= 0xffffff3fu;
	v |= 0x19u;
	v = (v & 0xff0000ffu) | ((wc << 8) & 0xffff00u);
	v = (v & 0xffffffu) | ((u32)tvpanel_dsi_ecc(v) << 24);
	writel(v, pkg0);

	v = readl(pkg1);
	v &= 0xffffu;			/* pd = 0 */
	v |= (u32)tvpanel_dsi_crc(wc, 0xffff) << 16;
	writel(v, pkg1);
}

/* ------------------------------------------------------------------ */
/* mode programming                                                    */
/* ------------------------------------------------------------------ */

int tvpanel_dsi_cfg(unsigned int id, const struct disp_panel_para *para,
		    const struct combodphy_config *cfg)
{
	struct __de_dsi_dev_t __iomem *dsi = tvpanel_dsi_get_reg_base(id);
	struct __de_dsi_dphy_dev_t __iomem *dphy = tvpanel_dsi_get_dphy_base(id);
	u32 if_type, lanes, pixbits, lane_den, v, ph, fmt_code, hdr_ht;
	u32 x, ht, hbp, hspw, dual, vtotal, hblank;
	u32 wc_hsa = 0, wc_hbp = 0, wc_hfp = 0, wc_hblk = 0, wc_vblk = 0;

	if (!dsi || !dphy || !para || !cfg)
		return -EINVAL;

	lanes = para->lcd_dsi_lane;
	if (lanes < 1 || lanes > 4)
		return -EINVAL;

	if_type = para->lcd_dsi_if;
	pixbits = dsi_pixel_bits[para->lcd_dsi_format & 3];
	lane_den = dsi_lane_den[lanes - 1] & 0xfu;

	x = para->lcd_x;
	ht = para->lcd_ht;
	hbp = para->lcd_hbp;
	hspw = para->lcd_hspw;
	dual = para->lcd_dsi_dual_dsi;
	vtotal = para->lcd_vt;

	/* dsi_res has to be programmed before the controller is switched on */
	tvpanel_dsi_res_select();

	/* basic_ctl0: ecc_en, crc_en, hs_eotp_en */
	v = readl(&dsi->dsi_basic_ctl0);
	v |= BIT(16) | BIT(17);
	if (para->lcd_dsi_eotp)
		v |= BIT(18);
	else
		v &= ~BIT(18);
	writel(v, &dsi->dsi_basic_ctl0);

	if (if_type == TVPANEL_DSI_IF_VIDEO) {
		/* video mode: the DSI video engine stays off, no DRQ */
		writel(readl(&dsi->dsi_basic_ctl1) & ~BIT(0), &dsi->dsi_basic_ctl1);
	} else {
		u32 drq;

		v = readl(&dsi->dsi_basic_ctl1);
		v = (v & 0xfffff00fu) | (1u << 4);	/* video_start_delay = 1 */
		v |= BIT(2);				/* precision align */
		writel(v, &dsi->dsi_basic_ctl1);

		/* dsi_work_mode = var_refresh_rate_en */
		v = readl(&dsi->dsi_basic_ctl);
		v = (v & ~BIT(30)) | ((para->var_refresh_rate_en & 1u) << 30);
		writel(v, &dsi->dsi_basic_ctl);

		/* video_frame_start follows !var_refresh_rate_en, then dsi_mode = 1 */
		v = readl(&dsi->dsi_basic_ctl1) & ~BIT(1);
		if (!para->var_refresh_rate_en)
			v |= BIT(1);
		v |= BIT(0);
		writel(v, &dsi->dsi_basic_ctl1);

		if (if_type == 2) {
			/* burst mode: line buffer length and DRQ watermark */
			u32 bytes = pixbits * ht / (8 * lanes);
			u32 lim = (x + hbp + 20) * pixbits / (8 * lanes) + 40;

			if (lim >= bytes)
				lim = bytes;
			drq = lim + ((TVPANEL_TCON_DIV * (x + 40)) >> 3);
			drq = (drq > bytes) ? (drq - bytes) : 1;

			v = readl(&dsi->dsi_burst_drq);
			writel((v & 0xffff0000u) | (drq & 0xffffu),
			       &dsi->dsi_burst_drq);
			v = readl(&dsi->dsi_burst_drq);
			writel(v & 0xffffu, &dsi->dsi_burst_drq);
			writel(readl(&dsi->dsi_tcon_drq) | BIT(28),
			       &dsi->dsi_tcon_drq);

			v = readl(&dsi->dsi_burst_line);
			writel((v & 0xffff0000u) | (bytes & 0xffffu),
			       &dsi->dsi_burst_line);
			v = readl(&dsi->dsi_burst_line);
			writel((v & 0xffffu) | (40u << 16), &dsi->dsi_burst_line);

			writel(readl(&dsi->dsi_basic_ctl) | BIT(0),
			       &dsi->dsi_basic_ctl);	/* video_mode_burst */

			if (lanes == 4) {
				v = readl(&dsi->dsi_basic_ctl);
				v = (v & ~0xf0u) | 0xc0u;	/* trail_inv */
				v |= BIT(3);			/* trail_fill */
				writel(v, &dsi->dsi_basic_ctl);
			}
		} else {
			/* single mode: only the TCON DRQ bit depends on hblank */
			hblank = ht - (x + hbp);
			v = readl(&dsi->dsi_tcon_drq);
			if (hblank > 0x14) {
				v = (((pixbits * (hblank + 32748)) >> 5) & 0x3ffu) |
				    ((v >> 10) << 10);
				v |= BIT(28);
			} else {
				v &= ~BIT(28);
			}
			writel(v, &dsi->dsi_tcon_drq);
		}
	}

	/* start position for the sequencer */
	v = readl(&dsi->dsi_trans_start);
	writel((v & 0xffffe000u) | 0xau, &dsi->dsi_trans_start);
	writel(readl(&dsi->dsi_trans_zero) & 0xffff0000u, &dsi->dsi_trans_zero);

	/* ---- instruction table: lane enable, clock lane, mode ---- */
	dsi_inst_setup(&dsi->dsi_inst_func[0], lane_den, 0, true,  false, false);
	dsi_inst_setup(&dsi->dsi_inst_func[1], 1,        1, false, false, false);
	dsi_inst_setup(&dsi->dsi_inst_func[2], 0,        2, true,  false, false);
	dsi_inst_setup(&dsi->dsi_inst_func[3], lane_den, 2, false, false, false);
	dsi_inst_setup(&dsi->dsi_inst_func[4], 1,        3, false, true,  true);
	dsi_inst_setup(&dsi->dsi_inst_func[5], 0,        4, true,  false, false);
	dsi_inst_setup(&dsi->dsi_inst_func[6], lane_den, 0, false, false, false);
	dsi_inst_setup(&dsi->dsi_inst_func[7], lane_den, 5, true,  false, false);
	dsi_inst_setup(&dsi->dsi_inst_func1[0], lane_den, 0, true,  false, false);
	dsi_inst_setup(&dsi->dsi_inst_func1[1], 0,        2, true,  false, false);
	dsi_inst_setup(&dsi->dsi_inst_func1[2], lane_den, 6, false, false, false);
	dsi_inst_setup(&dsi->dsi_inst_func1[3], 1,        3, false, true,  true);
	dsi_inst_setup(&dsi->dsi_inst_func1[4], 0,        4, true,  false, false);
	dsi_inst_setup(&dsi->dsi_inst_func1[5], lane_den, 0, false, false, false);
	dsi_inst_setup(&dsi->dsi_inst_func1[6], lane_den, 5, true,  false, false);

	/* loop selectors, then the loop count (0x31 = 49 iterations) */
	writel(0x30000002u, &dsi->dsi_inst_loop_sel);
	writel(0x03000002u, &dsi->dsi_inst_loop_sel1);
	v = readl(&dsi->dsi_inst_loop_num);
	writel((v & 0xfffff000u) | 0x31u, &dsi->dsi_inst_loop_num);
	v = readl(&dsi->dsi_inst_loop_num2);
	writel((v & 0xfffff000u) | 0x31u, &dsi->dsi_inst_loop_num2);

	switch (if_type) {
	case 0:
		v = readl(&dsi->dsi_inst_loop_num);
		writel((v & 0xf000ffffu) | 0x00310000u, &dsi->dsi_inst_loop_num);
		v = readl(&dsi->dsi_inst_loop_num2);
		writel((v & 0xf000ffffu) | 0x00310000u, &dsi->dsi_inst_loop_num2);
		break;
	case 1:
	case 2:
		v = readl(&dsi->dsi_inst_loop_num);
		writel(v & 0xf000ffffu, &dsi->dsi_inst_loop_num);
		v = readl(&dsi->dsi_inst_loop_num2);
		writel(v & 0xf000ffffu, &dsi->dsi_inst_loop_num2);
		break;
	}

	/* jump target: video mode jumps on vtotal, the rest on 1 */
	v = readl(&dsi->dsi_inst_jump_cfg[0]);
	if (if_type == TVPANEL_DSI_IF_VIDEO) {
		v |= BIT(28);
		hdr_ht = para->lcd_y & 0xffffu;
	} else {
		v &= ~BIT(28);
		hdr_ht = 1;
	}
	v = (v & 0xffff0000u) | hdr_ht;
	v = (v & 0xfff0ffffu) | (0x6u << 16);
	v = (v & 0xff0fffffu) | (0x5u << 20);
	writel(v, &dsi->dsi_inst_jump_cfg[0]);

	v = readl(&dsi->dsi_debug_data);
	writel((v & 0xff000000u) | 0xffu, &dsi->dsi_debug_data);
	writel(readl(&dsi->dsi_gctl) | BIT(0), &dsi->dsi_gctl);	/* dsi_en */

	/* ---- pixel stream header ---- */
	v = readl(&dsi->dsi_pixel_ctl0);
	if (if_type == TVPANEL_DSI_IF_VIDEO) {
		writel(v & ~BIT(16), &dsi->dsi_pixel_ctl0);	/* pd_plug_dis = 0 */
		ph = readl(&dsi->dsi_pixel_ph);
		ph = (ph & ~0x3fu) | 0x39u;		/* dt = 0x39 (RGB888) */
		ph = (ph & 0xff0000ffu) |
		     ((32u * (x * pixbits + 8)) & 0xffff00u);
		ph = (ph & 0xffffffu) | ((u32)tvpanel_dsi_ecc(ph) << 24);
		writel(ph, &dsi->dsi_pixel_ph);

		v = readl(&dsi->dsi_pixel_pd);
		writel((v & 0xffffff00u) | 0x2cu, &dsi->dsi_pixel_pd);
		v = readl(&dsi->dsi_pixel_pd);
		writel((v & 0xff00ffffu) | 0x003c0000u, &dsi->dsi_pixel_pd);

		writel(readl(&dsi->dsi_pixel_pf0) | 0xffffu, &dsi->dsi_pixel_pf0);
		v = readl(&dsi->dsi_pixel_pf1);
		writel((v & 0xffff0000u) | 0xe4e9u, &dsi->dsi_pixel_pf1);
		v = readl(&dsi->dsi_pixel_pf1);
		writel((v & 0xffffu) | (0xf468u << 16), &dsi->dsi_pixel_pf1);
		fmt_code = para->lcd_dsi_format;
	} else {
		writel(v | BIT(16), &dsi->dsi_pixel_ctl0);	/* pd_plug_dis = 1 */
		ph = readl(&dsi->dsi_pixel_ph);
		ph &= ~0x3fu;
		ph |= (62u - 16u * (u8)para->lcd_dsi_format) & 0x3eu;
		ph = (ph & 0xff0000ffu) | ((32u * x * pixbits) & 0xffff00u);
		ph = (ph & 0xffffffu) | ((u32)tvpanel_dsi_ecc(ph) << 24);
		writel(ph, &dsi->dsi_pixel_ph);

		writel(readl(&dsi->dsi_pixel_pf0) | 0xffffu, &dsi->dsi_pixel_pf0);
		writel(readl(&dsi->dsi_pixel_pf1) | 0xffffffffu,
		       &dsi->dsi_pixel_pf1);
		fmt_code = para->lcd_dsi_format + 8;
	}
	v = readl(&dsi->dsi_pixel_ctl0);
	writel((v & 0xfffffff0u) | (fmt_code & 0xfu), &dsi->dsi_pixel_ctl0);

	/* ---- blanking / sync packets (single and burst mode only) ---- */
	if (if_type == 0 || if_type == 2) {
		if (if_type == 2) {
			/* burst: the controller generates sync from the TCON */
			wc_hfp = (pixbits * x) >> 3;
			writel(readl(&dsi->dsi_basic_ctl) | BIT(1) | BIT(2),
			       &dsi->dsi_basic_ctl);	/* hsa_hse_dis, hbp_dis */
		} else if (dual) {
			u32 hact, tmp;

			v = readl(&dsi->dsi_pixel_ph);
			writel((v & 0xff0000ffu) |
			       ((32u * (x >> 1) * pixbits) & 0xffff00u),
			       &dsi->dsi_pixel_ph);

			hact = ht - hspw;
			writel(readl(&dsi->dsi_basic_ctl) | BIT(8),
			       &dsi->dsi_basic_ctl);	/* start_mode */

			tmp = pixbits * (hact >> 1);
			v = readl(&dsi->dsi_basic_ctl1);
			writel((v & 0xffffu) | (48u << 16), &dsi->dsi_basic_ctl1);

			wc_hfp = max(tmp >> 3, 10u) - 10;
			tmp = pixbits * (hbp >> 1);
			wc_hbp = max(tmp >> 3, 10u) - 10;
			tmp = pixbits * (hspw >> 1);
			wc_hsa = max(tmp >> 3, 10u) - 10;

			if (lanes == 4)
				wc_hblk = (3 * ((ht >> 1) - (hspw >> 1)) - 10) >> 1;

			tmp = 3 * (((hact - hbp) >> 1) - (x >> 1)) - 12;
			wc_vblk = tmp & ~(tmp >> 31);	/* clamp to >= 0 */
		} else {
			u32 hblankb = ht - (x + hbp + hspw);
			u32 t66 = pixbits * (ht - hspw);
			u32 t67 = pixbits * hbp;
			u32 t68 = pixbits * hspw;

			if (lanes == 4)
				wc_vblk = 4 - ((((u8)((pixbits * ht) >> 3) *
						 (u8)vtotal) -
						(u8)(t66 >> 3)) & 3u);
			wc_hblk = ((pixbits * hblankb) >> 3) - 12;
			wc_hfp = (t66 >> 3) - 10;
			wc_hbp = (t67 >> 3) - 10;
			wc_hsa = (t68 >> 3) - 10;

			writel(readl(&dsi->dsi_basic_ctl) | BIT(8),
			       &dsi->dsi_basic_ctl);	/* start_mode */

			v = readl(&dsi->dsi_basic_ctl1);
			writel((v & 0xffffu) | (48u << 16), &dsi->dsi_basic_ctl1);
		}

		dsi_short_pkt(&dsi->dsi_sync_hss, 0x21);
		dsi_short_pkt(&dsi->dsi_sync_hse, 0x31);
		dsi_short_pkt(&dsi->dsi_sync_vss, 0x01);
		dsi_short_pkt(&dsi->dsi_sync_vse, 0x11);

		/* vertical: vsa/vbp then vact/vt */
		v = readl(&dsi->dsi_basic_size0);
		v = (v & 0xfffff000u) | (para->lcd_vspw & 0xfffu);
		v = (v & 0xf000ffffu) | ((para->lcd_vbp << 16) & 0x0fff0000u);
		writel(v, &dsi->dsi_basic_size0);

		v = readl(&dsi->dsi_basic_size1);
		v = (v & 0xfffff000u) | (para->lcd_y & 0xfffu);
		v = (v & 0xe000ffffu) | ((vtotal << 16) & 0x1fff0000u);
		writel(v, &dsi->dsi_basic_size1);

		dsi_blk_pkt(&dsi->dsi_blk_hsa0, &dsi->dsi_blk_hsa1, wc_hsa);
		dsi_blk_pkt(&dsi->dsi_blk_hbp0, &dsi->dsi_blk_hbp1, wc_hbp);
		dsi_blk_pkt(&dsi->dsi_blk_hfp0, &dsi->dsi_blk_hfp1, wc_hfp);
		dsi_blk_pkt(&dsi->dsi_blk_hblk0, &dsi->dsi_blk_hblk1, wc_hblk);
		dsi_blk_pkt(&dsi->dsi_blk_vblk0, &dsi->dsi_blk_vblk1, wc_vblk);
	}

	/* ---- D-PHY ---- */
	v = readl(&dphy->dphy_gctl);
	v &= ~BIT(0);				/* module_en = 0 */
	v = (v & 0xffffffcfu) |
	    (16u * (((16u * lanes + 48u) >> 4) & 3u));	/* lane_num = lanes - 1 */
	writel(v, &dphy->dphy_gctl);

	/* hstx_clk_cont: clock lane stays in HS between packets */
	writel(readl(&dphy->dphy_tx_ctl) | BIT(28), &dphy->dphy_tx_ctl);

	writel(cfg->dphy_tx_time0, &dphy->dphy_tx_time0);
	/* ck_prep = 7, ck_zero = 0x32, ck_pre = 3, ck_post = 0xa */
	writel(0x0a032207u, &dphy->dphy_tx_time1);

	v = readl(&dphy->dphy_tx_time2);
	v = (v & 0xffffff00u) | 0x1eu;		/* ck_trail_set */
	v &= ~BIT(28);				/* hs_dly_mode = 0 */
	v &= 0xff0000ffu;			/* hs_dly = 0 */
	writel(v, &dphy->dphy_tx_time2);

	/* lptx_ulps_exit = 0 */
	writel(readl(&dphy->dphy_tx_time3) & 0xfff00000u, &dphy->dphy_tx_time3);

	v = readl(&dphy->dphy_tx_time4);
	v = (v & 0xffffff00u) | 0x03u;		/* hstx_ana0_set */
	v = (v & 0xffff00ffu) | 0x0300u;	/* hstx_ana1_set */
	writel(v, &dphy->dphy_tx_time4);

	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_cfg);

MODULE_LICENSE("GPL v2");
