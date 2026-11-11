// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allwinner H726 tvpanel - LVDS transmitter.
 *
 * Three blocks are involved, all mapped from the one LVDS register window
 * passed to lvds_set_regbase():
 *
 *   +0x00000  struct __tv_lvds_tx20_core_reg_t   channel/lane control
 *   +0x80000  struct __tv_lvds_buf_reg_t         buffer, timing, test pattern
 *   +0xc0000  struct __tv_lvds_wm_top_reg_t      PHY-side top control
 *
 * plus the combo D-PHY, which is the same register block the DSI controller
 * uses one page above its base (tvpanel_dsi_get_dphy_base()).
 *
 * Two deliberate differences from the vendor module, both because this port
 * does not use the component framework:
 *
 *   - the vendor probes with component_add() and does the whole panel lookup
 *     in the component bind callback; here the platform driver does that work
 *     in probe() and returns -EPROBE_DEFER until the panel module registered.
 *     sunxi_tvpanel_lvds_add() is still exported with the vendor's
 *     component_add() semantics for prebuilt callers.
 *   - the vendor caches the target rate in its instance; the value it reads
 *     back is just disp_panel_para.lcd_dclk_freq, so the PLL call site passes
 *     that field scaled by 7 (LVDS serial ratio) because
 *     sunxi_tvdisp_top_pll_setting() does not apply that factor itself.
 */
#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include "tvpanel.h"
#include "tvpanel_regs.h"

#define LVDS_CORE	((struct __tv_lvds_tx20_core_reg_t __iomem *)lvds_core_base)
#define LVDS_WM_TOP	((struct __tv_lvds_wm_top_reg_t __iomem *)lvds_wm_top_base)
#define LVDS_BUF	((struct __tv_lvds_buf_reg_t __iomem *)lvds_buf_base)

static void __iomem *lvds_core_base;
static void __iomem *lvds_buf_base;
static void __iomem *lvds_wm_top_base;

static struct tvpanel_lvds *g_lvds;

/* vendor lvds_set_regbase(): one base, three sub-blocks at fixed offsets */
int lvds_set_regbase(void __iomem *base)
{
	if (!base)
		return -EINVAL;

	lvds_core_base = base;
	lvds_buf_base = base + TVPANEL_LVDS_BUF_OFFSET;
	lvds_wm_top_base = base + TVPANEL_LVDS_WM_TOP_OFFSET;
	return 0;
}
EXPORT_SYMBOL_GPL(lvds_set_regbase);

/*
 * The vendor applies the same two sequences to channel pairs: (reg10, reg14)
 * for the first channel and (reg20, reg24) for the second.  Written as one
 * helper each instead of duplicating thirty read-modify-writes.
 */
static void lvds_core_channel_pair(void __iomem *a, void __iomem *b)
{
	u32 v = readl(a);

	v &= ~7u;
	v = (v & 0xfffff8ffu) | 0x100u;
	v = (v & 0xfff8ffffu) | 0x20000u;
	v = (v & 0xf8ffffffu) | 0x3000000u;
	v |= 8u;
	v |= 0x800u;
	v |= 0x80000u;
	v |= 0x8000000u;
	writel(v, a);

	v = readl(b);
	v &= ~7u;
	v = (v & 0xfffff8ffu) | 0x500u;
	v |= 8u;
	v &= ~0x800u;
	writel(v, b);
}

/* vendor: identical mask sequence on wm_top reg20 and reg24 */
static void lvds_wm_top_channel(void __iomem *reg)
{
	u32 v = readl(reg);

	v |= 7u;
	v |= 8u;
	v |= 0x70u;
	v = (v & 0xffc0ffffu) | 0xf0000u;
	v = (v & 0xc0ffffffu) | 0x2f000000u;
	v |= 0x40000000u;
	writel(v, reg);
}

/* vendor lvds_cfg(): full transmitter setup from disp_panel_para */
int lvds_cfg(const struct disp_panel_para *para)
{
	struct __tv_lvds_tx20_core_reg_t __iomem *core = LVDS_CORE;
	struct __tv_lvds_wm_top_reg_t __iomem *wm = LVDS_WM_TOP;
	struct __tv_lvds_buf_reg_t __iomem *buf = LVDS_BUF;
	u32 v, vfp;

	if (!core || !wm || !buf || !para)
		return -EINVAL;

	v = readl(&core->lvds_ctrl_reg);
	v |= 4u;
	v = (v & 0xffffffe7u) | (8u * (para->lvds_bit_depth & 3u));
	v &= ~0x20u;
	v = (v & 0xffffff3fu) | ((para->lvds_mode & 3u) << 6);
	v = (v & 0xffffbfffu) | ((para->lvds_even_odd_dir & 1u) << 14);
	v &= 0xfffffcffu;
	v = (v & 0xfffff3ffu) | 0x400u;
	v = (v & 0xffffcfffu) | 0x2000u;
	v |= 0x8000u;
	v = (v & 0xfffdffffu) | ((para->lcd_lvds_vs_invert & 1u) << 17);
	v = (v & 0xfffeffffu) | ((para->lcd_lvds_hs_invert & 1u) << 16);
	v = (v & 0xfffbffffu) | ((para->lcd_lvds_de_invert & 1u) << 18);
	v = (v & 0xfeffffffu) | ((para->lcd_lvds_clk_invert & 1u) << 24);
	v &= 0xffe7ffffu;
	v |= 0x600000u;
	v |= 0x800000u;
	v &= ~0x10000000u;
	v &= ~0x80000000u;
	writel(v, &core->lvds_ctrl_reg);

	writel(0, &core->reg04);
	writel(0, &core->reg10);
	writel(0, &core->reg14);
	writel(0, &core->reg20);
	writel(0, &core->reg24);

	/*
	 * Vendor control flow, kept as written: with lvds_if == 1 both channel
	 * pairs are programmed; otherwise the pair is chosen by the channel
	 * count, pair 1 for a single channel and pair 2 for dual.
	 */
	if (para->lvds_if == 1 || para->lcd_lvds_channel == 0)
		lvds_core_channel_pair(&core->reg10, &core->reg14);
	if (para->lvds_if == 1 || para->lcd_lvds_channel != 0)
		lvds_core_channel_pair(&core->reg20, &core->reg24);

	v = readl(&wm->reg10);
	writel(v | 0x40000000u, &wm->reg10);

	v = readl(&wm->reg0);
	v &= ~0x20000000u;
	v &= ~0x10000000u;
	v &= ~0x40u;
	writel(v, &wm->reg0);

	v = readl(&buf->reg0c);
	v &= ~1u;
	v |= 2u;
	v &= ~4u;
	v &= ~8u;
	writel(v, &buf->reg0c);

	v = readl(&buf->reg1c);
	v |= 2u;
	writel(v, &buf->reg1c);

	v = readl(&buf->reg0c);
	v &= ~0x10u;
	v &= ~0x20u;
	v &= ~0x40u;
	v &= ~0x80u;
	v &= ~0x100u;
	v |= 0x200u;
	v &= ~0x400u;
	writel(v, &buf->reg0c);

	v = readl(&buf->reg0c);
	v &= 0xffffcfffu;
	if (para->lvds_if == 1)
		v |= 0x1000u;
	writel(v, &buf->reg0c);

	v = readl(&buf->reg0c);
	v &= ~0x4000u;
	v &= 0xfffcffffu;
	v = (v & 0xfff3ffffu) | 0x40000u;
	v = (v & 0xffcfffffu) | 0x200000u;
	v |= 0xc00000u;
	v |= 1u;
	writel(v, &buf->reg0c);

	msleep(2);

	v = readl(&buf->reg0c);
	writel(v & ~1u, &buf->reg0c);

	if (para->lvds_if == 1 || para->lcd_lvds_channel == 0)
		lvds_wm_top_channel(&wm->reg20);
	if (para->lvds_if == 1 || para->lcd_lvds_channel != 0)
		lvds_wm_top_channel(&wm->reg24);

	/* buffer timing: total, active, then the blanking sums */
	v = readl(&buf->reg20);
	v = (v & 0xffffe000u) | (para->lcd_ht & 0x1fffu);
	v = (v & 0xe000ffffu) | ((para->lcd_vt << 16) & 0x1fff0000u);
	writel(v, &buf->reg20);

	v = readl(&buf->reg24);
	v = (v & 0xf000ffffu) | ((para->lcd_y & 0xfffu) << 16);
	v = (v & 0xfffff000u) | (para->lcd_x & 0xfffu);
	writel(v, &buf->reg24);

	v = readl(&buf->reg28);
	v = (v & 0xf000ffffu) |
	    ((((u16)para->lcd_vbp + (u16)para->lcd_vspw) & 0xfffu) << 16);
	v = (v & 0xfffff000u) |
	    (((u16)para->lcd_hbp + (u16)para->lcd_hspw) & 0xfffu);
	writel(v, &buf->reg28);

	v = readl(&buf->reg2c);
	writel((v & 0xffffff00u) | (para->lcd_hspw & 0xffu) | 0x80000000u,
	       &buf->reg2c);

	/*
	 * reg30 carries the vertical front porch: the vendor computes
	 *   vfp = (u8)(lcd_vt - (lcd_y + lcd_vspw + lcd_vbp))
	 * then stores vfp in [23:16] and vfp + lcd_vspw in [7:0].
	 */
	vfp = (u8)(para->lcd_vt -
		   (para->lcd_y + para->lcd_vspw + para->lcd_vbp));
	v = readl(&buf->reg30);
	v |= 0x80000000u;
	v = (v & 0xffffff00u) | (u8)(vfp + para->lcd_vspw);
	v = (v & 0xff00ffffu) | (vfp << 16);
	writel(v, &buf->reg30);
	return 0;
}
EXPORT_SYMBOL_GPL(lvds_cfg);

/* vendor lvds_test_pattern(): 0 disables, patterns 1..8 are the vendor set */
int lvds_test_pattern(unsigned int enable, unsigned int pattern)
{
	struct __tv_lvds_buf_reg_t __iomem *buf = LVDS_BUF;
	u32 v;

	if (!buf)
		return -EINVAL;

	v = readl(&buf->reg0c);
	writel((v & ~1u) | (enable & 1u), &buf->reg0c);

	v = readl(&buf->reg1c);
	writel((v & ~4u) | (4u * (enable & 1u)), &buf->reg1c);

	v = readl(&buf->reg0c);
	writel((v & ~8u) | (8u * (enable & 1u)), &buf->reg0c);

	v = readl(&buf->reg1c);
	v &= ~2u;
	if (!enable)
		v |= 2u;
	writel(v, &buf->reg1c);

	switch (pattern) {
	case 1:
	case 8:
		v = readl(&buf->reg38);
		v = (v & 0xfffff000u) | 0x80u;
		v = (v & 0xf000ffffu) | 0x800000u;
		writel(v, &buf->reg38);
		break;
	default:
		v = readl(&buf->reg38);
		v &= 0xfffff000u;
		v &= 0xf000ffffu;
		writel(v, &buf->reg38);
		break;
	}

	switch (pattern) {
	case 1:
		writel(0, &buf->reg3c);
		writel(0x3fffffffu, &buf->reg40);
		break;
	case 2:
		writel(0, &buf->reg3c);
		writel(0, &buf->reg40);
		break;
	case 3:
		writel(0x3fffffffu, &buf->reg3c);
		writel(0x3fffffffu, &buf->reg40);
		break;
	case 4:
		writel(0xff000000u, &buf->reg3c);
		writel(0, &buf->reg40);
		break;
	case 5:
		writel(0x00ff0000u, &buf->reg3c);
		writel(0, &buf->reg40);
		break;
	case 6:
		writel(0xffff0000u, &buf->reg3c);
		writel(0, &buf->reg40);
		break;
	case 7:
		writel(0xffu, &buf->reg3c);
		writel(0, &buf->reg40);
		break;
	case 8:
		writel(0xff000000u, &buf->reg3c);
		writel(0xffu, &buf->reg40);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(lvds_test_pattern);

/*
 * vendor lvds_combphy_open(): the combo PHY is the DSI D-PHY block, so this
 * drives combo_phy_reg1/reg0 and then the analog block, each delay being the
 * vendor's const_udelay(1073740), i.e. 250us.
 */
int lvds_combphy_open(unsigned int id)
{
	struct __de_dsi_dphy_dev_t __iomem *dphy =
		(struct __de_dsi_dphy_dev_t __iomem *)
			tvpanel_dsi_get_dphy_base(id);

	if (!dphy)
		return -EINVAL;

	writel(0x53u, &dphy->combo_phy_reg1);
	writel(1u, &dphy->combo_phy_reg0);
	usleep_range(250, 260);
	writel(5u, &dphy->combo_phy_reg0);
	usleep_range(250, 260);
	writel(7u, &dphy->combo_phy_reg0);
	usleep_range(250, 260);

	writel(0xfu, &dphy->combo_phy_reg0);
	writel(0x84000000u, &dphy->dphy_ana4);
	writel(0x01040000u, &dphy->dphy_ana3);
	writel(0, &dphy->dphy_ana2);
	writel(0, &dphy->dphy_ana1);
	return 0;
}
EXPORT_SYMBOL_GPL(lvds_combphy_open);

/* vendor lvds_combphy_close(): plain register clears, no masks */
int lvds_combphy_close(unsigned int id)
{
	struct __de_dsi_dphy_dev_t __iomem *dphy =
		(struct __de_dsi_dphy_dev_t __iomem *)
			tvpanel_dsi_get_dphy_base(id);

	if (!dphy)
		return -EINVAL;

	writel(0, &dphy->combo_phy_reg1);
	writel(0, &dphy->combo_phy_reg0);
	writel(0, &dphy->dphy_ana4);
	writel(0, &dphy->dphy_ana3);
	writel(0, &dphy->dphy_ana1);
	return 0;
}
EXPORT_SYMBOL_GPL(lvds_combphy_close);

/* ------------------------------------------------------------------ */
/* LVDS connector ops                                                  */
/* ------------------------------------------------------------------ */

static struct tvpanel_lvds *conn_to_lvds(struct tvpanel_connector *conn)
{
	return container_of(conn, struct tvpanel_lvds, conn);
}

int sunxi_lvds_get_timing(struct tvpanel_connector *conn,
			  struct display_timing *dt)
{
	struct tvpanel_lvds *lvds = conn_to_lvds(conn);
	u64 pixel_clk = 0;
	int pll, ret;

	if (!dt || !lvds)
		return -EINVAL;

	if (!lvds->bound) {
		dev_err(lvds->dev, "lvds driver has not been bound yet!\n");
		return -EINVAL;
	}

	pll = sunxi_tvdisp_top_get_working_pll_index(lvds->tcon_top);
	ret = sunxi_tvdisp_top_get_pixel_clk(lvds->tcon_top, pll, &pixel_clk);

	memcpy(dt, &conn->timing, sizeof(*dt));
	if (ret == 0)
		dt->pixelclock.typ = (u32)pixel_clk;
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_lvds_get_timing);

/* vendor sunxi_lvds_clk_enable(): first user brings the block up */
int sunxi_lvds_connector_prepare(struct tvpanel_connector *conn)
{
	struct tvpanel_lvds *lvds = conn_to_lvds(conn);
	unsigned int i, rate;
	int ret;

	if (!lvds || !lvds->dev)
		return -EINVAL;

	atomic_inc(&lvds->clk_enabled);
	if (atomic_read(&lvds->clk_enabled) != 1 || lvds->enabled)
		return 0;

	sunxi_tvpanel_top_clk_enable(lvds->tcon_top);

	for (i = 0; i < ARRAY_SIZE(lvds->clks); i++) {
		if (lvds->rsts[i])
			reset_control_deassert(lvds->rsts[i]);
		if (!lvds->clks[i])
			continue;
		if (lvds->clk_freq[i]) {
			dev_dbg(lvds->dev, "set clk%u lvds rate:%u\n", i,
				lvds->clk_freq[i]);
			clk_set_rate(lvds->clks[i], lvds->clk_freq[i]);
		}
		if (!clk_prepare(lvds->clks[i]) && clk_enable(lvds->clks[i]))
			clk_unprepare(lvds->clks[i]);
	}

	ret = sunxi_tvpanel_top_lvds_enable(lvds->tcon_top, 1, &lvds->para);
	if (ret)
		return ret;

	sunxi_tvdisp_top_set_working_pll_index(lvds->tcon_top, 1);

	/* 7x: the LVDS serial clock, which the PLL layer does not scale */
	rate = lvds->para.lcd_dclk_freq * 7u;
	sunxi_tvdisp_top_pll_setting(lvds->tcon_top, &lvds->para, rate, 0, 1);
	sunxi_tvdisp_top_pll_setting(lvds->tcon_top, &lvds->para, rate, 1, 1);
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_lvds_connector_prepare);

/* vendor sunxi_lvds_clk_disable(): only the last user tears the block down */
int sunxi_lvds_connector_unprepare(struct tvpanel_connector *conn)
{
	struct tvpanel_lvds *lvds = conn_to_lvds(conn);
	unsigned int i, rate;

	if (!lvds || !lvds->dev)
		return -EINVAL;

	if (!atomic_dec_and_test(&lvds->clk_enabled))
		return 0;

	sunxi_tvpanel_top_lvds_enable(lvds->tcon_top, 0, &lvds->para);

	for (i = 0; i < ARRAY_SIZE(lvds->clks); i++) {
		if (lvds->clks[i]) {
			clk_disable(lvds->clks[i]);
			clk_unprepare(lvds->clks[i]);
		}
		if (lvds->rsts[i])
			reset_control_assert(lvds->rsts[i]);
	}

	rate = lvds->para.lcd_dclk_freq * 7u;
	sunxi_tvdisp_top_pll_setting(lvds->tcon_top, &lvds->para, rate, 0, 0);
	sunxi_tvdisp_top_pll_setting(lvds->tcon_top, &lvds->para, rate, 1, 0);
	sunxi_tvpanel_top_clk_disable(lvds->tcon_top);
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_lvds_connector_unprepare);

int sunxi_lvds_connector_enable(struct tvpanel_connector *conn)
{
	struct tvpanel_lvds *lvds = conn_to_lvds(conn);

	if (!lvds || !lvds->dev)
		return -EINVAL;

	tvpanel_pin_set_state(lvds->dev, "active");
	tvpanel_panel_prepare(conn->panel);
	lvds_cfg(&lvds->para);
	tvpanel_panel_enable(conn->panel);

	lvds->enabled = true;
	conn->is_enabled = true;
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_lvds_connector_enable);

int sunxi_lvds_connector_disable(struct tvpanel_connector *conn)
{
	struct tvpanel_lvds *lvds = conn_to_lvds(conn);

	if (!lvds || !lvds->dev)
		return -EINVAL;

	tvpanel_panel_disable(conn->panel);
	tvpanel_panel_unprepare(conn->panel);
	tvpanel_pin_set_state(lvds->dev, "sleep");

	lvds->enabled = false;
	conn->is_enabled = false;
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_lvds_connector_disable);

/*
 * vendor sunxi_lvds_connector_vrr(): reprogram the PLL that is *not* current,
 * then switch to it, so the running clock is never disturbed.  The vendor's
 * own symbol takes only (conn, rate); the connector vtable always passes four
 * arguments, and the two totals are not used on the LVDS path.
 */
int sunxi_lvds_connector_vrr(struct tvpanel_connector *conn, unsigned int rate,
			     unsigned int htotal, unsigned int vtotal)
{
	struct tvpanel_lvds *lvds = conn_to_lvds(conn);
	int pll, next;

	if (!lvds || !lvds->dev)
		return -EINVAL;

	if (!lvds->enabled) {
		dev_err(lvds->dev, "Connector is not enabled yet!\n");
		return -EINVAL;
	}

	if (rate && conn->timing.pixelclock.typ != rate) {
		pll = sunxi_tvdisp_top_get_working_pll_index(lvds->tcon_top);
		if (pll < 0) {
			dev_err(lvds->dev, "Wrong pll index:%d!\n", pll);
			return -EINVAL;
		}
		next = (pll == 0);
		sunxi_tvdisp_top_pll_setting(lvds->tcon_top, &lvds->para, rate,
					     next, 1);
		sunxi_tvdisp_top_set_working_pll_index(lvds->tcon_top, next);
		conn->timing.pixelclock.typ = rate;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_lvds_connector_vrr);

/* vendor sunxi_lvds_connector_sw_enable(): clocks and panel, no lvds_cfg */
int sunxi_lvds_connector_sw_enable(struct tvpanel_connector *conn)
{
	struct tvpanel_lvds *lvds = conn_to_lvds(conn);
	unsigned int i;

	if (!lvds || !lvds->dev)
		return -EINVAL;

	sunxi_tvpanel_top_clk_enable(lvds->tcon_top);

	for (i = 0; i < ARRAY_SIZE(lvds->clks); i++) {
		if (lvds->rsts[i])
			reset_control_deassert(lvds->rsts[i]);
		if (!lvds->clks[i])
			continue;
		if (lvds->clk_freq[i]) {
			dev_dbg(lvds->dev, "set clk%u lvds rate:%u\n", i,
				lvds->clk_freq[i]);
			clk_set_rate(lvds->clks[i], lvds->clk_freq[i]);
		}
		if (!clk_prepare(lvds->clks[i]) && clk_enable(lvds->clks[i]))
			clk_unprepare(lvds->clks[i]);
	}

	tvpanel_pin_set_state(lvds->dev, "active");
	tvpanel_panel_sw_enable(conn->panel);

	lvds->enabled = true;
	conn->is_enabled = true;
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_lvds_connector_sw_enable);

const struct tvpanel_connector_funcs tvpanel_lvds_conn_funcs = {
	.get_timing		= sunxi_lvds_get_timing,
	.enable			= sunxi_lvds_connector_enable,
	.disable		= sunxi_lvds_connector_disable,
	.prepare		= sunxi_lvds_connector_prepare,
	.unprepare		= sunxi_lvds_connector_unprepare,
	.sw_enable		= sunxi_lvds_connector_sw_enable,
	.adjust_refresh_rate	= sunxi_lvds_connector_vrr,
};
EXPORT_SYMBOL_GPL(tvpanel_lvds_conn_funcs);

/* ------------------------------------------------------------------ */
/* component glue, as the vendor exposes it                            */
/* ------------------------------------------------------------------ */

static int sunxi_lvds_component_bind(struct device *dev,
				     struct device *master, void *data)
{
	return 0;
}

static void sunxi_lvds_component_unbind(struct device *dev,
					struct device *master, void *data)
{
}

static const struct component_ops sunxi_lvds_component_ops = {
	.bind	= sunxi_lvds_component_bind,
	.unbind	= sunxi_lvds_component_unbind,
};

/*
 * vendor sunxi_tvpanel_lvds_add(): component_add(dev, ops).  The vendor exports
 * this one plainly (its section is `__ksymtab`, unlike the panel_* set), so the
 * export class is kept as the vendor has it.
 */
int sunxi_tvpanel_lvds_add(struct device *dev)
{
	return component_add(dev, &sunxi_lvds_component_ops);
}
EXPORT_SYMBOL(sunxi_tvpanel_lvds_add);

/* ------------------------------------------------------------------ */
/* platform driver                                                     */
/* ------------------------------------------------------------------ */

static const struct tvpanel_lvds_data tvpanel_lvds_data0 = {
	.id = 0,
};

static int tvpanel_lvds_bind_panel(struct tvpanel_lvds *lvds)
{
	struct device *dev = lvds->dev;
	struct tvpanel_panel *panel;
	struct display_timing *native = NULL;
	struct tvpanel_timing timing;
	int ret;

	panel = tvpanel_get_panel_dev(dev);
	if (!panel) {
		dev_err(dev, "of_tvpanel_find_panel fail\n");
		return -EPROBE_DEFER;
	}

	ret = tvpanel_panel_get_native_timing(panel, &native);
	if (ret || !native) {
		dev_err(dev, "Fail to get native timing! return:%d\n", ret);
		return ret ? ret : -EINVAL;
	}

	if (panel->funcs && panel->funcs->get_extra_para &&
	    panel->funcs->get_extra_para(panel, &lvds->para))
		dev_info(dev, "get_extra_para fail\n");

	tvpanel_timing_from_display_timing(native, &timing);
	display_timing_to_panel_para(&timing, &lvds->para);
	lvds->para.lcd_if = 1;		/* vendor: *(u32 *)(inst + 188) = 1 */

	lvds->conn.panel = panel;
	ret = tvpanel_connector_bind(&lvds->conn, tvpanel_get_device(), dev,
				     lvds->lvds_id, &tvpanel_lvds_conn_funcs,
				     native, TVPANEL_CONN_LVDS);
	if (ret)
		return ret;

	lvds->bound = true;
	dev_info(dev, "%s\n", __func__);
	return 0;
}

static int tvpanel_lvds_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tvpanel_lvds *lvds;
	const struct tvpanel_lvds_data *data;
	void __iomem *base;
	unsigned int i;
	int ret;

	lvds = devm_kzalloc(dev, sizeof(*lvds), GFP_KERNEL);
	if (!lvds)
		return -ENOMEM;
	lvds->dev = dev;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	lvds->irq = platform_get_irq(pdev, 0);
	if (lvds->irq == -EPROBE_DEFER)
		return lvds->irq;
	if (lvds->irq < 0)
		lvds->irq = 0;

	for (i = 0; i < ARRAY_SIZE(lvds->clks); i++) {
		char name[16];
		struct clk *clk;
		u32 freq = 0;

		snprintf(name, sizeof(name), "clk%u", i);
		clk = devm_clk_get_optional(dev, name);
		if (IS_ERR(clk))
			return PTR_ERR(clk);
		if (!clk)
			continue;
		lvds->clks[i] = clk;

		snprintf(name, sizeof(name), "clk%u_freq", i);
		if (!of_property_read_u32(dev->of_node, name, &freq))
			lvds->clk_freq[i] = freq;

		snprintf(name, sizeof(name), "rst%u", i);
		lvds->rsts[i] = devm_reset_control_get_optional_exclusive(dev,
									 name);
		if (IS_ERR(lvds->rsts[i]))
			return PTR_ERR(lvds->rsts[i]);
	}

	data = of_device_get_match_data(dev);
	if (!data) {
		dev_err(dev, "fail to get match data\n");
		return -ENODEV;
	}
	lvds->lvds_data = data;
	lvds->lvds_id = data->id;

	lvds->tcon_top = tvpanel_get_tcon_top();
	lvds->tcon = tvpanel_get_tcon();
	if (!lvds->tcon_top || !lvds->tcon)
		dev_warn(dev, "display top/tcon not ready yet\n");

	lvds->conn.dev = dev;
	lvds->conn.tpdev = tvpanel_get_device();

	ret = lvds_set_regbase(base);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, lvds);
	g_lvds = lvds;

	ret = tvpanel_lvds_bind_panel(lvds);
	if (ret == -EPROBE_DEFER)
		return ret;
	if (ret)
		dev_err(dev, "panel bind failed: %d\n", ret);

	dev_info(dev, "lvds%u ready (irq %d)\n", lvds->lvds_id, lvds->irq);
	return 0;
}

static int tvpanel_lvds_remove(struct platform_device *pdev)
{
	struct tvpanel_lvds *lvds = platform_get_drvdata(pdev);

	if (g_lvds == lvds)
		g_lvds = NULL;
	return 0;
}

static const struct of_device_id tvpanel_lvds_of_match[] = {
	{ .compatible = "allwinner,lvds0", .data = &tvpanel_lvds_data0 },
	{ .compatible = "allwinner,tvdisp_lvds", .data = &tvpanel_lvds_data0 },
	{ }
};
MODULE_DEVICE_TABLE(of, tvpanel_lvds_of_match);

struct platform_driver tvpanel_lvds_driver = {
	.probe	= tvpanel_lvds_probe,
	.remove	= tvpanel_lvds_remove,
	.driver	= {
		.name		= "lvds",
		.of_match_table	= tvpanel_lvds_of_match,
	},
};
