// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allwinner H726 tvpanel - display top, TCON and PLL driver.
 *
 * The vendor tvpanel.ko defines this itself (the symbols are T, not imported),
 * so the from-source module has to as well.  Three blocks are involved:
 *
 *   - the display top (tvdisp@5500000, __tv_disp_top_reg_t): clock gates,
 *     reset controls and the two LVDS/TCON PLLs, plus the pixel clock readback
 *   - the TCON (tcon0@5501000, __tv_tcon_reg_t): basic timing registers and
 *     the module enable bit
 *   - the per-block enables the connector layer calls (dsi/lvds/tcon/ksc)
 *
 * All register writes are bit-exact against the vendor binary.  The one place
 * that is not a straight transcription is tvdisp_top_pll_config(): the vendor
 * searches for a fractional best fit with 64-bit arithmetic that Hex-Rays
 * renders as unreadable __PAIR64__ soup.  It is rebuilt here as the exact
 * inverse of the (fully readable) tvdisp_top_get_pll_info() encoding, with the
 * vendor's VCO bounds; the round trip is checked in the build test.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include "tvpanel.h"
#include "tvpanel_regs.h"

#define TVPANEL_TOP_MAX_CLK	4

/* the TCON block is larger than __tv_tcon_reg_t: the vendor keeps a second
 * window at +0x300 for the output/colour registers.
 */
#define TVPANEL_TCON_REG2	0x300

/* VCO limits the vendor clamps the PLL to (0x4b1a1300 .. 0x12c684c01) */
#define TVPANEL_PLL_VCO_MIN	0x4b1a1300ULL
#define TVPANEL_PLL_VCO_MAX	0x12c684c01ULL

struct tvpanel_top {
	struct device *dev;
	void __iomem *reg;
	unsigned int num_clk;
	struct clk *clks[TVPANEL_TOP_MAX_CLK];
	u32 clk_freq[TVPANEL_TOP_MAX_CLK];
	struct reset_control *rsts[TVPANEL_TOP_MAX_CLK];
	atomic_t refcount;
};

struct tvpanel_tcon {
	struct device *dev;
	void __iomem *reg;
	void __iomem *reg2;
	int irq;
	bool enabled;
	unsigned int num_clk;
	struct clk *clks[TVPANEL_TOP_MAX_CLK];
	u32 clk_freq[TVPANEL_TOP_MAX_CLK];
	struct reset_control *rsts[TVPANEL_TOP_MAX_CLK];
	struct device *top;
};

static struct tvpanel_top *top_of(struct device *dev)
{
	return dev ? dev_get_drvdata(dev) : NULL;
}

static struct tvpanel_tcon *tcon_of(struct device *dev)
{
	return dev ? dev_get_drvdata(dev) : NULL;
}

#define TOP_REG(top)	((struct __tv_disp_top_reg_t __iomem *)((top)->reg))

/*
 * The vendor addresses both blocks through file-scope globals (tvdisp_top,
 * g_tcon_reg) rather than through the device, so several of its exported
 * helpers take no instance; these stand in for those globals.
 */
static struct __tv_disp_top_reg_t __iomem *tvpanel_top_reg;
static struct device *tvpanel_top_dev;
static struct device *tvpanel_tcon_dev;

struct device *tvpanel_get_tcon_top(void)
{
	return tvpanel_top_dev;
}
EXPORT_SYMBOL_GPL(tvpanel_get_tcon_top);

struct device *tvpanel_get_tcon(void)
{
	return tvpanel_tcon_dev;
}
EXPORT_SYMBOL_GPL(tvpanel_get_tcon);

/* ------------------------------------------------------------------ */
/* TCON register layer                                                 */
/* ------------------------------------------------------------------ */

/* vendor tcon_enable(): tv_ctrl_reg bit 31 */
static int tvpanel_tcon_hw_enable(struct tvpanel_tcon *tcon, unsigned int on)
{
	struct __tv_tcon_reg_t __iomem *r = tcon->reg;
	u32 v;

	if (!r)
		return -EINVAL;

	v = readl(&r->tv_ctrl_reg);
	v &= 0x7fffffffu;
	v |= (on & 1u) << 31;
	writel(v, &r->tv_ctrl_reg);
	return 0;
}

/*
 * Stands in for the vendor's g_tcon_reg: tcon_colorbar_style() is exported and
 * takes no instance, so it addresses whichever TCON probed.  Only the debug
 * sysfs hook (tcon_set_colorbar) calls it, but prebuilts may bind it too.
 */
static struct __tv_tcon_reg_t __iomem *tvpanel_tcon_reg;

/* vendor tcon_colorbar_style(): tv_src_ctrl_reg bits [2:1] */
int tcon_colorbar_style(u8 style)
{
	struct __tv_tcon_reg_t __iomem *r = tvpanel_tcon_reg;

	if (!r)
		return -ENODEV;		/* the vendor returns -1 here */

	writel((readl(&r->tv_src_ctrl_reg) & 0xfffffff9u) |
	       (2u * (style & 3u)), &r->tv_src_ctrl_reg);
	return 0;
}
EXPORT_SYMBOL_GPL(tcon_colorbar_style);

/*
 * Vendor tcon_config(): basic timing from disp_panel_para.
 *
 * The vendor writes the high half of timing3 with
 *   a2[40] = (u16)a2[40] - 0x10000;
 * which sets bits [31:16] to 0xffff rather than decrementing the high field.
 * That is reproduced literally; it is called out in the README because it
 * looks like a decompiler artefact rather than intended hardware behaviour.
 */
static int tvpanel_tcon_config(struct tvpanel_tcon *tcon,
			       const struct disp_panel_para *para)
{
	struct __tv_tcon_reg_t __iomem *r = tcon->reg;
	u32 t;

	if (!r || !para)
		return -EINVAL;

	/* vendor: reg2[+0x40] bits [2:0] = 1 */
	writel((readl(tcon->reg2 + 0x40) & 0xfffffff8u) | 1u, tcon->reg2 + 0x40);

	writel(readl(&r->tv_glb_ctrl_reg) & 0xffffffcfu, &r->tv_glb_ctrl_reg);

	/* timing1[16:0] = 2 * vtotal */
	writel((readl(&r->timing1_reg) & 0xfffe0000u) |
	       ((2u * para->lcd_vt) & 0x1fffeu), &r->timing1_reg);

	/* timing2[14:0] = vactive - 1, timing2[30:16] = hactive - 1 */
	t = readl(&r->timing2_reg);
	t = (t & 0xffff8000u) | ((para->lcd_y - 1u) & 0x7fffu);
	t = (t & 0x8000ffffu) | (((para->lcd_x - 1u) << 16) & 0x7fff0000u);
	writel(t, &r->timing2_reg);

	/* timing3[14:0] = hbp + hspw - 1, then the vendor's high-half decrement */
	t = readl(&r->timing3_reg);
	t = (t & 0xffff8000u) | ((para->lcd_hbp + para->lcd_hspw - 1u) & 0x7fffu);
	t = (u16)t - 0x10000u;
	writel(t, &r->timing3_reg);

	/* timing4[14:0] = vbp + vspw - 1 */
	writel((readl(&r->timing4_reg) & 0xffff8000u) |
	       ((para->lcd_vbp + para->lcd_vspw - 1u) & 0x7fffu),
	       &r->timing4_reg);

	/* timing5[11:0] = vspw - 1, timing5[27:16] = hspw - 1 */
	t = readl(&r->timing5_reg);
	t = (t & 0xfffff000u) | ((para->lcd_vspw - 1u) & 0xfffu);
	t = (t & 0xf000ffffu) | (((para->lcd_hspw - 1u) << 16) & 0x0fff0000u);
	writel(t, &r->timing5_reg);

	/* src ctrl: pulse the reset bit */
	t = readl(&r->tv_src_ctrl_reg);
	writel(t | 1u, &r->tv_src_ctrl_reg);
	writel(readl(&r->tv_src_ctrl_reg) & 0xfffffff9u, &r->tv_src_ctrl_reg);
	writel(readl(&r->tv_src_ctrl_reg) | 1u, &r->tv_src_ctrl_reg);

	/* sync polarity: clear the four output-polarity bits */
	t = readl(&r->sync_io_pol_reg);
	t &= ~0x1000000u;
	t &= ~0x2000000u;
	t &= ~0x4000000u;
	t &= ~0x8000000u;
	writel(t, &r->sync_io_pol_reg);

	writel(readl(&r->tv_glb_inter_reg) & ~0x40000000u, &r->tv_glb_inter_reg);

	/* tv_ctrl_reg[19:4] = 0x2 */
	writel((readl(&r->tv_ctrl_reg) & 0xfff0000fu) | 0x20u, &r->tv_ctrl_reg);
	return 0;
}

/* ------------------------------------------------------------------ */
/* display top: clocks, resets                                         */
/* ------------------------------------------------------------------ */

/*
 * Vendor sunxi_tvpanel_top_clk_enable(): per clock, deassert its reset, apply
 * the optional rate, then prepare/enable; the refcount is bumped at the end.
 */
int sunxi_tvpanel_top_clk_enable(struct device *dev)
{
	struct tvpanel_top *top = top_of(dev);
	unsigned int i;

	if (!top) {
		if (dev)
			dev_err(dev, "%s fail!\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < top->num_clk; i++) {
		if (top->rsts[i])
			reset_control_deassert(top->rsts[i]);
		if (!top->clks[i])
			continue;
		if (top->clk_freq[i])
			clk_set_rate(top->clks[i], top->clk_freq[i]);
		if (!clk_prepare(top->clks[i]) && clk_enable(top->clks[i]))
			clk_unprepare(top->clks[i]);
	}

	atomic_inc(&top->refcount);
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_tvpanel_top_clk_enable);

/* Vendor sunxi_tvpanel_top_clk_disable(): only tears down from a live count. */
int sunxi_tvpanel_top_clk_disable(struct device *dev)
{
	struct tvpanel_top *top = top_of(dev);
	unsigned int i;

	if (!top) {
		if (dev)
			dev_err(dev, "%s fail!\n", __func__);
		return -EINVAL;
	}

	if (!atomic_read(&top->refcount))
		return 0;

	for (i = top->num_clk; i-- > 0; ) {
		if (top->clks[i]) {
			clk_disable(top->clks[i]);
			clk_unprepare(top->clks[i]);
		}
		if (top->rsts[i])
			reset_control_assert(top->rsts[i]);
	}

	atomic_dec(&top->refcount);
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_tvpanel_top_clk_disable);

/* ------------------------------------------------------------------ */
/* display top: per-block enables                                      */
/* ------------------------------------------------------------------ */

int sunxi_tvpanel_top_tcon_tv_enable(struct device *dev, unsigned int on)
{
	struct tvpanel_top *top = top_of(dev);
	struct __tv_disp_top_reg_t __iomem *r;
	u32 v;

	if (!top || !top->reg)
		return -EINVAL;
	r = TOP_REG(top);

	v = readl(&r->dsi_tcon_tv_ctrl_reg);
	v = (v & 0xfffffffdu) | (on ? 2u : (on & 1u));
	writel(v, &r->dsi_tcon_tv_ctrl_reg);
	v = readl(&r->dsi_tcon_tv_ctrl_reg);
	writel((v & 0xffffffdfu) | (32u * (on & 1u)), &r->dsi_tcon_tv_ctrl_reg);

	v = readl(&r->lvds_global_ctrl_reg);
	v = (v & 0xfffdffffu) | ((on & 1u) << 17);
	writel(v, &r->lvds_global_ctrl_reg);
	v = readl(&r->lvds_global_ctrl_reg);
	writel((v & 0xfffffffdu) | (on ? 2u : (on & 1u)),
	       &r->lvds_global_ctrl_reg);
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_tvpanel_top_tcon_tv_enable);

int sunxi_tvpanel_top_dsi_enable(struct device *dev, unsigned int on)
{
	struct tvpanel_top *top = top_of(dev);
	struct __tv_disp_top_reg_t __iomem *r;
	u32 v, set;

	if (!top || !top->reg)
		return -EINVAL;
	r = TOP_REG(top);

	v = readl(&r->dsi_tcon_tv_ctrl_reg);
	writel((v & 0xfffffffeu) | (on & 1u), &r->dsi_tcon_tv_ctrl_reg);

	v = readl(&r->dsi_tcon_tv_ctrl_reg);
	writel((v & 0xffffffefu) | (16u * (on & 1u)), &r->dsi_tcon_tv_ctrl_reg);

	set = on ? 0x10000u : on;
	v = readl(&r->svp_clk_gate_reg);
	writel((v & 0xfffeffffu) | set, &r->svp_clk_gate_reg);

	v = readl(&r->lvds_global_ctrl_reg);
	writel(v & ~1u, &r->lvds_global_ctrl_reg);
	v = readl(&r->lvds_global_ctrl_reg);
	writel((v & 0xfffeffffu) | set, &r->lvds_global_ctrl_reg);
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_tvpanel_top_dsi_enable);

int sunxi_tvdisp_top_tcon_colorbar_enable(struct device *dev, unsigned int on)
{
	struct tvpanel_top *top = top_of(dev);
	struct __tv_disp_top_reg_t __iomem *r;
	u32 v;

	if (!top || !top->reg)
		return -EINVAL;
	r = TOP_REG(top);

	v = readl(&r->dsi_tcon_tv_ctrl_reg);
	writel((v & 0xffffffbfu) | ((on & 1u) << 6), &r->dsi_tcon_tv_ctrl_reg);
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_tvdisp_top_tcon_colorbar_enable);

/* vendor tvdisp_top_afbd_enable(): AFBD reset bit 0, then its clock gates */
int sunxi_tvdisp_top_afbd_enable(struct device *dev, unsigned int on)
{
	struct tvpanel_top *top = top_of(dev);
	struct __tv_disp_top_reg_t __iomem *r;
	u32 v;

	if (!top || !top->reg)
		return -EINVAL;
	r = TOP_REG(top);

	v = readl(&r->afbd_rst_ctrl_reg);
	writel((v & 0xfffffffeu) | (on & 1u), &r->afbd_rst_ctrl_reg);

	v = readl(&r->afbd_clk_ctrl_reg);
	v = (v & 0xfffffeffu) | ((on & 1u) << 8);
	v = (v & 0xfffffffeu) | (on & 1u);
	v = (v & 0xffffefffu) | ((on & 1u) << 12);
	v = (v & 0xffffffefu) | (16u * (on & 1u));
	v &= ~0x10000u;		/* the vendor clears bit 16 either way */
	writel(v, &r->afbd_clk_ctrl_reg);
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_tvdisp_top_afbd_enable);

/* vendor tvdisp_top_svp_enable(): SVP clock gates and resets, bits 0..20 */
int sunxi_tvdisp_top_svp_enable(struct device *dev, unsigned int on)
{
	struct tvpanel_top *top = top_of(dev);
	struct __tv_disp_top_reg_t __iomem *r;
	u32 gate, rst, i;

	if (!top || !top->reg)
		return -EINVAL;
	r = TOP_REG(top);

	gate = readl(&r->svp_clk_gate_reg);
	gate = (gate & ~1u) | (on & 1u);
	for (i = 4; i <= 20; i += 4) {
		gate &= ~(1u << i);
		if (on)
			gate |= 1u << i;
	}
	gate &= ~0x1000000u;
	gate &= ~0x2000000u;
	writel(gate, &r->svp_clk_gate_reg);

	rst = readl(&r->svp_rst_ctrl_reg);
	rst = (rst & ~1u) | (on & 1u);
	for (i = 4; i <= 20; i += 4) {
		rst &= ~(1u << i);
		if (on)
			rst |= 1u << i;
	}
	rst = (rst & ~0x01000000u) | ((on & 1u) << 24);
	rst = (rst & ~0x10000000u) | ((on & 1u) << 28);
	writel(rst, &r->svp_rst_ctrl_reg);
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_tvdisp_top_svp_enable);

/* vendor tvdisp_top_ksc_enable(): the colour-space converter clock gate */
int sunxi_tvdisp_top_ksc_enable(struct device *dev, unsigned int on)
{
	struct tvpanel_top *top = top_of(dev);
	struct __tv_disp_top_reg_t __iomem *r;
	u32 v;

	if (!top || !top->reg)
		return -EINVAL;
	r = TOP_REG(top);

	v = readl(&r->ksc_clk_gate_reg);
	v = (v & 0xfffffffdu) | (2u * (on & 1u));
	v = (v & 0xfffffffeu) | (on & 1u);
	v = (v & 0xfffffff7u) | (8u * (on & 1u));
	v = (v & 0xfffffffbu) | (4u * (on & 1u));
	writel(v, &r->ksc_clk_gate_reg);
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_tvdisp_top_ksc_enable);

int sunxi_tvdisp_top_set_working_pll_index(struct device *dev, unsigned int idx)
{
	struct tvpanel_top *top = top_of(dev);
	struct __tv_disp_top_reg_t __iomem *r;
	u32 v;

	if (!top || !top->reg)
		return -EINVAL;
	r = TOP_REG(top);

	v = readl(&r->lvds_global_ctrl_reg);
	v |= 4u;			/* PLL switch in progress */
	v &= ~1u;
	if (!idx)
		v |= 1u;		/* bit 0 selects PLL0 */
	writel(v, &r->lvds_global_ctrl_reg);
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_tvdisp_top_set_working_pll_index);

int sunxi_tvdisp_top_get_working_pll_index(struct device *dev)
{
	struct tvpanel_top *top = top_of(dev);

	if (!top || !top->reg)
		return -EINVAL;

	return (readl(&TOP_REG(top)->lvds_global_ctrl_reg) & 1u) == 0;
}
EXPORT_SYMBOL_GPL(sunxi_tvdisp_top_get_working_pll_index);

/*
 * PLL register encoding, read out of tvdisp_top_get_pll_info():
 *
 *   reg268  bit0 / bit1      PLL0 / PLL1 enable
 *   reg276  [7:4]+1          PLL0 post divider (1..16)
 *           [3:2]+1          PLL0 post divider (1..4)
 *           [0]+1            PLL0 post divider (1..2)
 *           [23:20]+1        PLL1 post divider (1..16)
 *           [19:16]+1        PLL1 post divider (1..4)
 *           [1]+1            PLL1 post divider (1..2)
 *   reg308  [7:0]+1          PLL0 feedback divider
 *           [15:8]+1         PLL1 feedback divider
 *
 *   pixel_clk = 24MHz * (N + 1) / (d0 * d1 * d2)
 */
int sunxi_tvdisp_top_get_pixel_clk(struct device *dev, int pll_index, u64 *clk)
{
	struct tvpanel_top *top = top_of(dev);
	struct __tv_disp_top_reg_t __iomem *r;
	u32 ctl, factors, n;
	u64 vco;

	if (!top || !top->reg || !clk)
		return -EINVAL;
	r = TOP_REG(top);

	if (pll_index == 0) {
		ctl = readl(&r->lvds_pll_ctrl_reg0);
		if (!(ctl & 1u)) {
			*clk = 0;
			return 0;
		}
		factors = readl(&r->lvds_pll_ctrl_reg2);
		n = readl(&r->lvds_pll_ctrl_reg10);
		vco = 24000000ULL * (((n >> 0) & 0xffu) + 1);
		factors = (((factors >> 4) & 0xfu) + 1) *
			  (((factors >> 2) & 0x3u) + 1) *
			  ((factors & 1u) + 1);
	} else if (pll_index == 1) {
		ctl = readl(&r->lvds_pll_ctrl_reg0);
		if (!(ctl & 2u)) {
			*clk = 0;
			return 0;
		}
		factors = readl(&r->lvds_pll_ctrl_reg2);
		n = readl(&r->lvds_pll_ctrl_reg10);
		vco = 24000000ULL * (((n >> 8) & 0xffu) + 1);
		factors = (((factors >> 20) & 0xfu) + 1) *
			  (((factors >> 16) & 0x3u) + 1) *
			  (((factors >> 1) & 1u) + 1);
	} else {
		return -EINVAL;
	}

	*clk = div_u64(vco, factors);
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_tvdisp_top_get_pixel_clk);

/*
 * Program one PLL for the requested rate.
 *
 * Reconstructed inverse of the encoding above: pick the feedback divider that
 * lands the VCO inside the vendor's window, then the post-divider combination
 * whose product is closest to vco/rate.  The vendor's own search is a
 * fractional best fit with 64-bit intermediates that did not survive the
 * decompile readably; this matches its register format and bounds exactly, and
 * round-trips through sunxi_tvdisp_top_get_pixel_clk().
 */
int sunxi_tvdisp_top_pll_setting(struct device *dev,
				 const struct disp_panel_para *para,
				 unsigned int rate, int pll_index, int enable)
{
	struct tvpanel_top *top = top_of(dev);
	struct __tv_disp_top_reg_t __iomem *r;
	u32 best_d = 0, best_err = ~0u, best_n = 0, best_d0 = 0, best_d1 = 0,
	    best_d2 = 0;
	unsigned int n, d0, d1, d2;

	if (!top || !top->reg || !para)
		return -EINVAL;
	r = TOP_REG(top);

	if (!enable) {
		u32 v = readl(&r->lvds_pll_ctrl_reg0);

		v &= ~(pll_index ? 2u : 1u);
		writel(v, &r->lvds_pll_ctrl_reg0);
		return 0;
	}

	/* the LVDS path scales the target by 7 (serial ratio) before this call */
	(void)para;
	if (!rate)
		return -EINVAL;

	for (n = 0; n < 256; n++) {
		u64 vco = 24000000ULL * (n + 1);

		if (vco < TVPANEL_PLL_VCO_MIN)
			continue;
		if (vco > TVPANEL_PLL_VCO_MAX)
			break;

		for (d0 = 1; d0 <= 16; d0++) {
			for (d1 = 1; d1 <= 4; d1++) {
				for (d2 = 1; d2 <= 2; d2++) {
					u64 got = div_u64(vco, d0 * d1 * d2);
					u32 err = got > rate ? (u32)(got - rate) :
							      (u32)(rate - got);

					if (err < best_err) {
						best_err = err;
						best_n = n;
						best_d0 = d0;
						best_d1 = d1;
						best_d2 = d2;
						best_d = d0 * d1 * d2;
					}
				}
			}
		}
	}

	if (!best_d)
		return -ERANGE;

	if (pll_index == 0) {
		u32 div = readl(&r->lvds_pll_ctrl_reg2);

		div &= ~0xcfu;		/* [7:4], [3:2], [0] */
		div |= ((best_d0 - 1) & 0xfu) << 4;
		div |= ((best_d1 - 1) & 0x3u) << 2;
		div |= (best_d2 - 1) & 1u;
		writel(div, &r->lvds_pll_ctrl_reg2);

		div = readl(&r->lvds_pll_ctrl_reg10);
		writel((div & 0xffffff00u) | (best_n & 0xffu),
		       &r->lvds_pll_ctrl_reg10);
	} else {
		u32 div = readl(&r->lvds_pll_ctrl_reg2);

		div &= ~(0xfu << 20);
		div &= ~(0x3u << 16);
		div &= ~2u;
		div |= ((best_d0 - 1) & 0xfu) << 20;
		div |= ((best_d1 - 1) & 0x3u) << 16;
		div |= ((best_d2 - 1) & 1u) << 1;
		writel(div, &r->lvds_pll_ctrl_reg2);

		div = readl(&r->lvds_pll_ctrl_reg10);
		writel((div & 0xffff00ffu) | ((best_n & 0xffu) << 8),
		       &r->lvds_pll_ctrl_reg10);
	}

	writel(readl(&r->lvds_pll_ctrl_reg0) | (pll_index ? 2u : 1u),
	       &r->lvds_pll_ctrl_reg0);
	dev_dbg(dev, "pll%d: %u Hz -> n=%u d=%u (%u*%u*%u)\n", pll_index, rate,
		best_n, best_d, best_d0, best_d1, best_d2);
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_tvdisp_top_pll_setting);

/* ------------------------------------------------------------------ */
/* TCON driver                                                         */
/* ------------------------------------------------------------------ */

int sunxi_tcon_module_enable(struct device *dev, unsigned int on)
{
	struct tvpanel_tcon *tcon = tcon_of(dev);

	if (!tcon || !tcon->enabled) {
		if (dev)
			dev_err(dev, "%s fail!\n", __func__);
		return -EINVAL;
	}

	return tvpanel_tcon_hw_enable(tcon, on);
}
EXPORT_SYMBOL_GPL(sunxi_tcon_module_enable);

/*
 * Vendor sunxi_tcon_clk_enable(): the top block owns the real clocks, so this
 * only flips the TCON's own enabled flag and hands over to the top enables.
 */
int sunxi_tcon_clk_enable(struct device *dev, unsigned int on)
{
	struct tvpanel_tcon *tcon = tcon_of(dev);
	const char *what = on ? "enabled" : "disabled";

	if (!tcon) {
		if (dev)
			dev_err(dev, "%s fail!\n", __func__);
		return -EINVAL;
	}

	if (tcon->enabled == !!on) {
		dev_err(dev, "tcon has already %s\n", what);
		return -EINVAL;
	}

	tcon->enabled = !!on;
	if (tcon->top) {
		if (on)
			sunxi_tvpanel_top_clk_enable(tcon->top);
		else
			sunxi_tvpanel_top_clk_disable(tcon->top);
		sunxi_tvpanel_top_tcon_tv_enable(tcon->top, on);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_tcon_clk_enable);

int sunxi_tcon_config(struct device *dev, const struct disp_panel_para *para)
{
	struct tvpanel_tcon *tcon = tcon_of(dev);

	if (!tcon || !tcon->enabled) {
		if (dev)
			dev_err(dev, "%s fail!\n", __func__);
		return -EINVAL;
	}

	return tvpanel_tcon_config(tcon, para);
}
EXPORT_SYMBOL_GPL(sunxi_tcon_config);

/* ------------------------------------------------------------------ */
/* platform glue                                                       */
/* ------------------------------------------------------------------ */

static int tvpanel_top_parse_resources(struct platform_device *pdev,
				       struct tvpanel_top *top)
{
	struct device *dev = &pdev->dev;
	struct resource *res;
	int i;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	top->reg = devm_ioremap_resource(dev, res);
	if (IS_ERR(top->reg))
		return PTR_ERR(top->reg);

	for (i = 0; i < TVPANEL_TOP_MAX_CLK; i++) {
		char name[16];
		struct clk *clk;
		u32 freq = 0;

		snprintf(name, sizeof(name), "clk%d", i);
		clk = devm_clk_get_optional(dev, name);
		if (IS_ERR(clk))
			break;
		if (!clk)
			continue;

		top->clks[i] = clk;
		snprintf(name, sizeof(name), "clk%d_freq", i);
		if (!of_property_read_u32(dev->of_node, name, &freq))
			top->clk_freq[i] = freq;

		snprintf(name, sizeof(name), "rst%d", i);
		top->rsts[i] = devm_reset_control_get_optional_exclusive(dev, name);
		if (IS_ERR(top->rsts[i]))
			top->rsts[i] = NULL;

		top->num_clk = i + 1;
	}

	atomic_set(&top->refcount, 0);
	return 0;
}

static int tvpanel_top_probe(struct platform_device *pdev)
{
	struct tvpanel_top *top;
	int ret;

	top = devm_kzalloc(&pdev->dev, sizeof(*top), GFP_KERNEL);
	if (!top)
		return -ENOMEM;
	top->dev = &pdev->dev;

	ret = tvpanel_top_parse_resources(pdev, top);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, top);
	tvpanel_top_reg = top->reg;
	tvpanel_top_dev = &pdev->dev;
	dev_info(&pdev->dev, "tvpanel display top ready (%u clocks)\n",
		 top->num_clk);
	return 0;
}

static const struct of_device_id tvpanel_top_of_match[] = {
	{ .compatible = "allwinner,tvdisp_top" },
	{ }
};
MODULE_DEVICE_TABLE(of, tvpanel_top_of_match);

static struct platform_driver tvpanel_top_driver = {
	.probe	= tvpanel_top_probe,
	.driver	= {
		.name		= "tvdisp_top",
		.of_match_table	= tvpanel_top_of_match,
	},
};

static int tvpanel_tcon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tvpanel_tcon *tcon;
	struct device_node *np;
	struct platform_device *top_pdev;
	int ret, i;

	tcon = devm_kzalloc(dev, sizeof(*tcon), GFP_KERNEL);
	if (!tcon)
		return -ENOMEM;
	tcon->dev = dev;

	tcon->reg = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(tcon->reg))
		return PTR_ERR(tcon->reg);
	tcon->reg2 = tcon->reg + TVPANEL_TCON_REG2;
	tvpanel_tcon_reg = tcon->reg;

	tcon->irq = platform_get_irq(pdev, 0);

	/*
	 * No of_node_put() on purpose: the vendor kernel is built with
	 * CONFIG_OF_DYNAMIC=n, where the helper is a compile-time no-op and the
	 * symbol is never exported, so a reference to it would make this module
	 * fail to load with "unknown symbol of_node_put".  The single probe-time
	 * reference is kept unreleased, exactly as the vendor leaves it.
	 */
	np = of_parse_phandle(dev->of_node, "top", 0);
	if (np) {
		top_pdev = of_find_device_by_node(np);
		if (top_pdev)
			tcon->top = &top_pdev->dev;
	}

	/* the TCON block owns no clocks of its own: the top gates them */
	for (i = 0; i < TVPANEL_TOP_MAX_CLK; i++) {
		char name[16];
		struct clk *clk;
		u32 freq = 0;

		snprintf(name, sizeof(name), "clk%d", i);
		clk = devm_clk_get_optional(dev, name);
		if (IS_ERR(clk) || !clk)
			break;
		tcon->clks[i] = clk;
		snprintf(name, sizeof(name), "clk%d_freq", i);
		if (!of_property_read_u32(dev->of_node, name, &freq))
			tcon->clk_freq[i] = freq;
		tcon->num_clk = i + 1;
	}

	platform_set_drvdata(pdev, tcon);
	tvpanel_tcon_dev = dev;

	ret = tvpanel_tcon_hw_enable(tcon, 0);
	if (ret)
		return ret;

	dev_info(dev, "tvpanel tcon ready (irq %d, top %s)\n", tcon->irq,
		 tcon->top ? "found" : "absent");
	return 0;
}

static const struct of_device_id tvpanel_tcon_of_match[] = {
	{ .compatible = "allwinner,tvdisp_tcon" },
	{ .compatible = "allwinner,tcon" },
	{ }
};
MODULE_DEVICE_TABLE(of, tvpanel_tcon_of_match);

static struct platform_driver tvpanel_tcon_driver = {
	.probe	= tvpanel_tcon_probe,
	.driver	= {
		.name		= "tcon",
		.of_match_table	= tvpanel_tcon_of_match,
	},
};

/*
 * vendor tvdisp_top_panel_sys_enable(): the panel-side clock gates, the input
 * clock mux and the panel resets in one go.  The decompile's a2[32..34] are
 * panel_ext_clk_gate_reg, panel_inclk_ctrl_reg and panel_rst_ctrl_reg.
 */
int tvdisp_top_panel_sys_enable(unsigned int enable,
				const struct disp_panel_para *para)
{
	struct __tv_disp_top_reg_t __iomem *r = tvpanel_top_reg;
	u32 v6, v7, v8, v9, v11, v;

	if (!r || !para)
		return -EINVAL;

	v6 = enable ? 16u : enable;
	v7 = enable ? 256u : enable;
	v8 = enable ? 4096u : enable;
	v9 = enable ? 0x100000u : enable;
	v11 = enable ? 0x1000000u : enable;

	v = readl(&r->panel_ext_clk_gate_reg);
	v = (v & ~1u) | (enable & 1u);
	v = (v & ~0x10u) | v6;
	v = (v & ~0x100u) | v7;
	v = (v & ~0x1000u) | v8;
	writel(v, &r->panel_ext_clk_gate_reg);

	v = readl(&r->panel_inclk_ctrl_reg);
	v = (v & ~0x200000u) | ((enable & 1u) << 21);
	v = (v & ~0x400000u) | ((enable & 1u) << 22);
	v = (v & ~0x800000u) | ((enable & 1u) << 23);
	v = (v & ~0x100000u) | v9;
	v &= ~1u;
	/* the vendor picks the mux by interface, not by the enable flag */
	if (para->lvds_if == 1)
		v = (v & ~0x30u) | 0x20u;
	else
		v |= 0x30u;
	v = (v & ~0x300u) | 0x100u;
	v = (v & ~0x3000u) | 0x1000u;
	v = (v & ~0x30000u) | 0x10000u;
	v = (v & ~0x1000000u) | v11;
	writel(v, &r->panel_inclk_ctrl_reg);

	v = readl(&r->panel_rst_ctrl_reg);
	v = (v & ~1u) | (enable & 1u);
	v = (v & ~0x10u) | v6;
	v = (v & ~0x100u) | v7;
	v = (v & ~0x1000u) | v8;
	v = (v & ~0x10000u) | ((enable & 1u) << 16);
	v = (v & ~0x100000u) | v9;
	v = (v & ~0x1000000u) | v11;
	writel(v, &r->panel_rst_ctrl_reg);
	return 0;
}
EXPORT_SYMBOL_GPL(tvdisp_top_panel_sys_enable);

/*
 * vendor tvdisp_lvds_phy_setting(): the LVDS PHY drive/skew fields.  The
 * vendor programs the first channel pair when lvds_if != 1 and the channel
 * count is 0, and the second pair when lvds_if == 1 or the count is nonzero.
 * With lvds_if == 1 both pairs are written.
 */
int tvdisp_lvds_phy_setting(unsigned int enable,
			    const struct disp_panel_para *para)
{
	struct __tv_disp_top_reg_t __iomem *r = tvpanel_top_reg;
	u32 v;

	if (!r || !para)
		return -EINVAL;

	v = readl(&r->lvds_phy_ctrl_reg0);
	v &= ~1u;
	v &= ~0x10u;
	v &= ~2u;
	v &= ~0x20u;
	writel(v, &r->lvds_phy_ctrl_reg0);

	if (para->lvds_if != 1 && para->lcd_lvds_channel == 0) {
		v = readl(&r->lvds_phy_ctrl_reg0);
		v = (v & ~1u) | (enable & 1u);
		v = (v & ~0x10u) | (16u * (enable & 1u));
		writel(v, &r->lvds_phy_ctrl_reg0);

		v = readl(&r->lvds_phy_ctrl_reg1);
		v = ((0xffffffffu << para->lvds_data_lane_num) & 0x1fu) |
		    (v & 0xffffffc0u);
		v ^= 0x3fu;
		v = (v & 0xfff8ffffu) |
		    ((para->lcd_lvds_diff_amplifier_lv & 7u) << 16);
		writel(v, &r->lvds_phy_ctrl_reg1);

		v = readl(&r->lvds_phy_ctrl_reg2);
		v = (v & 0xfff8ffffu) |
		    ((para->lcd_lvds_common_amplifier_lv & 7u) << 16);
		v = (v & ~0x3fu) | (para->lvds0_lane_np_inv & 0x3fu);
		writel(v, &r->lvds_phy_ctrl_reg2);
	}

	v = readl(&r->lvds_phy_ctrl_reg0);
	v &= ~0x100u;
	v &= ~0x200u;
	writel(v, &r->lvds_phy_ctrl_reg0);

	if (para->lvds_if == 1 || para->lcd_lvds_channel != 0) {
		v = readl(&r->lvds_phy_ctrl_reg0);
		v = (v & ~2u) | (2u * (enable & 1u));
		v = (v & ~0x20u) | (32u * (enable & 1u));
		writel(v, &r->lvds_phy_ctrl_reg0);

		v = readl(&r->lvds_phy_ctrl_reg1);
		v = ((0xffffff00u << para->lvds_data_lane_num) & 0x1f00u) |
		    (v & 0xffffc0ffu);
		v ^= 0x3f00u;
		v = (v & 0xff8fffffu) |
		    ((para->lcd_lvds_diff_amplifier_lv & 7u) << 20);
		writel(v, &r->lvds_phy_ctrl_reg1);

		v = readl(&r->lvds_phy_ctrl_reg2);
		v = (v & 0xff8fffffu) |
		    ((para->lcd_lvds_common_amplifier_lv & 7u) << 20);
		v = (v & 0xffffc0ffu) |
		    ((para->lvds1_lane_np_inv & 0x3fu) << 8);
		writel(v, &r->lvds_phy_ctrl_reg2);
	}

	v = readl(&r->lvds_global_ctrl_reg);
	v |= 8u;
	v &= ~1u;
	v = (v & ~0x10000u) | ((enable & 1u) << 16);
	v |= 2u;
	writel(v, &r->lvds_global_ctrl_reg);
	return 0;
}
EXPORT_SYMBOL_GPL(tvdisp_lvds_phy_setting);

/*
 * vendor sunxi_tvpanel_top_lvds_enable(): panel system first, then the LVDS
 * PHY.  The vendor prints its panel_sys_enable message for both failures.
 */
int sunxi_tvpanel_top_lvds_enable(struct device *dev, unsigned int enable,
				  const struct disp_panel_para *para)
{
	struct tvpanel_top *top = top_of(dev);
	int ret;

	if (!top || !top->reg)
		return -EINVAL;

	ret = tvdisp_top_panel_sys_enable(enable, para);
	if (ret) {
		dev_err(dev, "tvdisp_top_panel_sys_enable fail:%d\n", ret);
		return ret;
	}

	ret = tvdisp_lvds_phy_setting(enable, para);
	if (ret) {
		dev_err(dev, "tvdisp_top_panel_sys_enable fail:%d\n", ret);
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_tvpanel_top_lvds_enable);

static struct platform_driver *const tvpanel_sub_drivers[] = {
	&tvpanel_top_driver,
	&tvpanel_tcon_driver,
	&tvpanel_lvds_driver,
};

/*
 * Registered from the module init in tvpanel_core.c: the vendor registers its
 * four sub-drivers before the tvpanel driver in one init_module, and a module
 * can only have one module_init() anyway.
 */
int tvpanel_sub_drivers_register(void)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(tvpanel_sub_drivers); i++) {
		ret = platform_driver_register(tvpanel_sub_drivers[i]);
		if (ret) {
			while (i--)
				platform_driver_unregister(tvpanel_sub_drivers[i]);
			return ret;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_sub_drivers_register);

void tvpanel_sub_drivers_unregister(void)
{
	unsigned int i;

	for (i = ARRAY_SIZE(tvpanel_sub_drivers); i-- > 0; )
		platform_driver_unregister(tvpanel_sub_drivers[i]);
}
EXPORT_SYMBOL_GPL(tvpanel_sub_drivers_unregister);
