// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allwinner H726 tvpanel - connector layer.
 *
 * A connector is the thing a display pipeline drives: panel drivers register
 * a struct tvpanel_panel, the TV backend binds a connector to it, and
 * enable/disable walk the real bring-up order (top clocks, PLL, TCON, DSI
 * mode programming, PHY, panel power).
 *
 * Field writes and call order follow the vendor binary exactly; the register
 * side is reached through tvpanel_dsi_{cfg,open,close,io_*}() instead of raw
 * offsets.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/reset.h>

#include "tvpanel.h"
#include "tvpanel_regs.h"
#include "dsi_tables.h"

/* ------------------------------------------------------------------ */
/* panel helpers used by the connector timing queries                  */
/* ------------------------------------------------------------------ */

int tvpanel_panel_get_native_timing(struct tvpanel_panel *panel,
				    struct display_timing **timing)
{
	if (!panel || !timing)
		return -EINVAL;

	*timing = NULL;
	if (panel->funcs && panel->funcs->get_native_timing)
		return panel->funcs->get_native_timing(panel, timing);

	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_panel_get_native_timing);

/*
 * Software-enable a panel without the full connector bring-up: used by the
 * sw_enable connector op when the display core has already set the hardware
 * up.  Vendor: clear the backlight state, then run its enable, and mark the
 * panel prepared + enabled in one go (the 0x0101 store at offset 24).
 */
int tvpanel_panel_sw_enable(struct tvpanel_panel *panel)
{
	struct backlight_device *bd = panel ? panel->backlight : NULL;

	if (bd) {
		bd->props.power = 0;
		bd->props.fb_blank = 0;
		bd->props.state &= ~BL_CORE_FBBLANK;
		mutex_lock(&bd->update_lock);
		if (bd->ops && bd->ops->update_status)
			bd->ops->update_status(bd);
		mutex_unlock(&bd->update_lock);
	}

	if (panel) {
		panel->prepared = true;
		panel->enabled = true;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_panel_sw_enable);

/* Vendor: devm-less pinctrl_get / lookup_state / select_state, warnings only. */
int tvpanel_pin_set_state(struct device *dev, const char *name)
{
	struct pinctrl *pinctrl;
	struct pinctrl_state *state;
	int ret;

	dev_dbg(dev, "%s start\n", __func__);

	pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(pinctrl)) {
		dev_warn(dev, "can NOT get pinctrl for %s\n", dev_name(dev));
		return 0;
	}

	state = pinctrl_lookup_state(pinctrl, name);
	if (IS_ERR(state)) {
		dev_err(dev, "pinctrl_lookup_state(%s) fail\n", name);
		return PTR_ERR(state);
	}

	ret = pinctrl_select_state(pinctrl, state);
	if (ret < 0) {
		dev_err(dev, "pinctrl_select_state(%s) for %s fail\n", name,
			dev_name(dev));
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_pin_set_state);

/* ------------------------------------------------------------------ */
/* connector registry                                                  */
/* ------------------------------------------------------------------ */

/*
 * The vendor looks the tvpanel_device up through offset 100 of its second
 * argument (device->driver_data).  Callers here pass it explicitly instead,
 * which is why the tpdev argument is separate; every field written to the
 * connector matches the vendor byte for byte.
 */
int tvpanel_connector_bind(struct tvpanel_connector *conn,
			   struct tvpanel_device *tpdev, struct device *dev,
			   int connector_id,
			   const struct tvpanel_connector_funcs *funcs,
			   const struct display_timing *timing,
			   int connector_type)
{
	if (!conn || !tpdev || !funcs || !timing)
		return -EINVAL;

	conn->tpdev = tpdev;
	conn->dev = dev;
	conn->panel = of_tvpanel_find_panel(dev->of_node);
	conn->funcs = funcs;
	conn->connector_id = connector_id;
	conn->connector_type = connector_type;
	memcpy(&conn->timing, timing, sizeof(conn->timing));
	conn->htotal = 0;
	conn->vtotal = 0;
	conn->is_enabled = false;

	list_add_tail(&conn->head, &tpdev->connector_list);
	tpdev->num_connector++;
	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_connector_bind);

/*
 * tvpanel_conn_enable() walks the connector the way the vendor sysfs/debug
 * entry point does: "1" prepares then enables, "0" disables then unprepares.
 * The connector funcs use the same offsets as struct tvpanel_connector_funcs
 * (prepare at +20, enable at +12, disable at +16, unprepare at +24).
 */
int tvpanel_conn_enable(int enable, struct device *dev, const char *buf,
			unsigned int count)
{
	struct tvpanel_connector *conn;
	struct tvpanel_device *tpdev = tvpanel_get_device();
	unsigned int val = 0;
	int ret;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		return ret;

	conn = get_first_tvpanel_connector(tpdev);
	if (!conn) {
		dev_err(dev, "no connector\n");
		return count;
	}

	if (val) {
		if (conn->funcs && conn->funcs->prepare)
			if (conn->funcs->prepare(conn))
				dev_err(conn->dev, "Connector prepare fail!\n");

		if (conn->funcs && conn->funcs->enable) {
			if (conn->is_enabled)
				dev_err(conn->dev,
					"Connector has already been enabled!\n");
			else if (conn->funcs->enable(conn))
				dev_err(conn->dev, "Connector enable fail!\n");
		}
	} else {
		if (conn->funcs && conn->funcs->disable && conn->is_enabled)
			if (conn->funcs->disable(conn))
				dev_err(conn->dev, "Connector disable fail!\n");

		if (conn->funcs && conn->funcs->unprepare)
			if (conn->funcs->unprepare(conn))
				dev_err(conn->dev, "Connector prepare fail!\n");
	}

	return count;
}
EXPORT_SYMBOL_GPL(tvpanel_conn_enable);

/* ------------------------------------------------------------------ */
/* DSI instance helpers                                                */
/* ------------------------------------------------------------------ */

static struct tvpanel_dsi *conn_to_dsi(struct tvpanel_connector *conn)
{
	if (!conn || !conn->dev)
		return NULL;
	return dev_get_drvdata(conn->dev);
}

/*
 * Refcounted clock/reset/PLL bring-up (vendor sunxi_dsi_clk_enable()): the
 * first user powers the whole block, later users only bump the count.  The
 * vendor's counter is at offset 548 and each clock has a matching reset and
 * an optional rate.
 */
static int tvpanel_dsi_clk_enable_all(struct tvpanel_dsi *dsi)
{
	bool first;
	unsigned int i;

	if (!dsi->dev)
		return -EINVAL;

	/* vendor: enable when the count is zero OR the block is marked off */
	first = !atomic_read(&dsi->clk_enabled) || !dsi->enabled;
	atomic_inc(&dsi->clk_enabled);
	if (!first)
		return 0;

	sunxi_tvpanel_top_clk_enable(dsi->tcon_top);
	sunxi_tvdisp_top_set_working_pll_index(dsi->tcon_top, 1);
	sunxi_tvdisp_top_pll_setting(dsi->tcon_top, &dsi->para,
				     dsi->para.lcd_dclk_freq, 0, 1);
	sunxi_tvdisp_top_pll_setting(dsi->tcon_top, &dsi->para,
				     dsi->para.lcd_dclk_freq, 1, 1);

	for (i = 0; i < ARRAY_SIZE(dsi->clks); i++) {
		if (dsi->rsts[i])
			reset_control_deassert(dsi->rsts[i]);
		if (!dsi->clks[i])
			continue;
		if (dsi->clk_freq[i]) {
			dev_dbg(dsi->dev, "set clk%u dsi rate:%u\n", i,
				dsi->clk_freq[i]);
			clk_set_rate(dsi->clks[i], dsi->clk_freq[i]);
		}
		if (!clk_prepare(dsi->clks[i]) && clk_enable(dsi->clks[i]))
			clk_unprepare(dsi->clks[i]);
	}
	return 0;
}

/*
 * Vendor sunxi_dsi_clk_disable(): only the last user tears the block down.
 * The vendor also bumps the counter back up inside the zero branch; that is
 * not reproduced - the "enabled" flag is what decides a re-enable, so the
 * count may safely reach zero here.
 */
static int tvpanel_dsi_clk_disable_all(struct tvpanel_dsi *dsi)
{
	unsigned int i;

	if (!dsi->dev)
		return -EINVAL;

	if (!atomic_dec_and_test(&dsi->clk_enabled))
		return 0;

	for (i = ARRAY_SIZE(dsi->clks); i-- > 0; ) {
		if (dsi->clks[i]) {
			clk_disable(dsi->clks[i]);
			clk_unprepare(dsi->clks[i]);
		}
		if (dsi->rsts[i])
			reset_control_assert(dsi->rsts[i]);
	}

	sunxi_tvdisp_top_pll_setting(dsi->tcon_top, &dsi->para,
				     dsi->para.lcd_dclk_freq, 0, 0);
	sunxi_tvdisp_top_pll_setting(dsi->tcon_top, &dsi->para,
				     dsi->para.lcd_dclk_freq, 1, 0);
	sunxi_tvpanel_top_clk_disable(dsi->tcon_top);
	return 0;
}

/* Vendor dsi_get_status(): sticky FIFO/instruction error flags. */
static int tvpanel_dsi_get_status(unsigned int id)
{
	struct __de_dsi_dev_t __iomem *dsi = tvpanel_dsi_get_reg_base(id);
	u32 v;

	if (!dsi)
		return 0;

	v = readl(&dsi->dsi_debug_inst);
	if (!(v & 0x3u))
		return 0;

	writel(v | 0x3u, &dsi->dsi_debug_inst);
	return -1;
}

static irqreturn_t tvpanel_dsi_irq_handler(int irq, void *data)
{
	struct tvpanel_dsi *dsi = data;

	tvpanel_dsi_irq_query(dsi->dsi_id, 2);
	if (tvpanel_dsi_get_status(dsi->dsi_id))
		dsi->err_count++;
	dsi->irq_count++;
	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* refresh-rate adjustment (vendor dsi_adjust_{h,v}total)              */
/* ------------------------------------------------------------------ */

/*
 * Wait for the video counter (dsi_debug_video0[12:0]) to wrap into the
 * window around the programmed total before the register is rewritten, so
 * the change lands on a frame boundary.  Vendor: 200 tries, 5ms apart.
 */
static void tvpanel_dsi_wait_vcount(struct __de_dsi_dev_t __iomem *dsi,
				    u32 lo, u32 hi)
{
	u32 v = readl(&dsi->dsi_debug_video0) & 0x1fff;
	int i;

	if (v <= hi && v >= lo)
		return;

	for (i = 0; i < 200; i++) {
		usleep_range(5000, 6000);
		v = readl(&dsi->dsi_debug_video0) & 0x1fff;
		if (v <= hi && v >= lo)
			break;
	}
}

static int tvpanel_dsi_adjust_vtotal(unsigned int id, u32 vtotal,
				     const struct disp_panel_para *para)
{
	struct __de_dsi_dev_t __iomem *dsi = tvpanel_dsi_get_reg_base(id);
	u32 base, lo, hi;

	if (!dsi)
		return -EINVAL;

	base = para->lcd_vbp + para->lcd_vspw;
	lo = base + 40;
	hi = para->lcd_y + base - 40;
	tvpanel_dsi_wait_vcount(dsi, lo, hi);

	writel((readl(&dsi->dsi_basic_size1) & 0xe000ffffu) |
	       ((vtotal & 0x1fffu) << 16), &dsi->dsi_basic_size1);
	return 0;
}

/*
 * Vendor dsi_adjust_htotal(): recompute the two horizontal blanking packet
 * headers for the new htotal and refresh their ECC/CRC.
 *
 * Note the wc formulas are attached to the opposite registers here compared
 * with tvpanel_dsi_cfg() (hblank against hfp0 rather than hblk0).  Both are
 * reproduced as the vendor has them; one of the two paths is likely a
 * vendor bug, but changing either would alter the emitted packet stream.
 */
static int tvpanel_dsi_adjust_htotal(unsigned int id, u32 htotal,
				     const struct disp_panel_para *para)
{
	struct __de_dsi_dev_t __iomem *dsi = tvpanel_dsi_get_reg_base(id);
	u32 bits = dsi_pixel_bits[para->lcd_dsi_format & 3];
	u32 hspw = para->lcd_hspw, hbp = para->lcd_hbp, hfp = para->lcd_hfp;
	u32 base, lo, hi, blank, wc_hfp, wc_hblk, v;

	if (!dsi)
		return -EINVAL;

	base = hspw + hbp + hfp;
	lo = base + 40;
	hi = para->lcd_x + base - 40;
	tvpanel_dsi_wait_vcount(dsi, lo, hi);

	blank = htotal - base;

	/* dsi_blk_hfp0: long packet, wc from the blanking length */
	wc_hfp = ((blank * bits) >> 3) - 12;
	v = readl(&dsi->dsi_blk_hfp0);
	v &= 0xffffff3fu;
	v |= 0x19u;
	v = (v & 0xff0000ffu) | ((wc_hfp & 0xffffu) << 8);
	v = (v & 0xffffffu) | ((u32)tvpanel_dsi_ecc(v) << 24);
	writel(v, &dsi->dsi_blk_hfp0);

	writel((readl(&dsi->dsi_blk_hfp1) & 0xffffu) |
	       ((u32)tvpanel_dsi_crc(wc_hfp, 0xffff) << 16),
	       &dsi->dsi_blk_hfp1);

	/* dsi_blk_hblk0: wc from htotal - hspw */
	wc_hblk = ((bits * (htotal - hspw)) >> 3) - 10;
	v = readl(&dsi->dsi_blk_hblk0);
	v &= 0xffffff3fu;
	v |= 0x19u;
	v = (v & 0xff0000ffu) | ((wc_hblk & 0xffffu) << 8);
	v = (v & 0xffffffu) | ((u32)tvpanel_dsi_ecc(v) << 24);
	writel(v, &dsi->dsi_blk_hblk0);

	writel((readl(&dsi->dsi_blk_hblk1) & 0xffffu) |
	       ((u32)tvpanel_dsi_crc(wc_hblk, 0xffff) << 16),
	       &dsi->dsi_blk_hblk1);
	return 0;
}

/* ------------------------------------------------------------------ */
/* DSI connector ops                                                   */
/* ------------------------------------------------------------------ */

static int sunxi_dsi_connector_get_timing(struct tvpanel_connector *conn,
					  struct display_timing *dt)
{
	struct tvpanel_dsi *dsi = conn_to_dsi(conn);
	struct display_timing *native = NULL;
	u64 pixel_clk = 0;
	int pll, ret;

	if (!dt || !dsi)
		return -EINVAL;

	pll = sunxi_tvdisp_top_get_working_pll_index(dsi->tcon_top);
	ret = sunxi_tvdisp_top_get_pixel_clk(dsi->tcon_top, pll, &pixel_clk);

	tvpanel_panel_get_native_timing(conn->panel, &native);
	if (!native)
		return -EINVAL;

	memcpy(dt, native, sizeof(*dt));
	if (ret == 0)
		dt->pixelclock.typ = (u32)pixel_clk;
	return 0;
}

static int sunxi_dsi_connector_prepare(struct tvpanel_connector *conn)
{
	struct tvpanel_dsi *dsi = conn_to_dsi(conn);

	if (!dsi || !dsi->dev)
		return -EINVAL;

	return tvpanel_dsi_clk_enable_all(dsi);
}

static int sunxi_dsi_connector_unprepare(struct tvpanel_connector *conn)
{
	struct tvpanel_dsi *dsi = conn_to_dsi(conn);

	if (!dsi)
		return -EINVAL;

	return tvpanel_dsi_clk_disable_all(dsi);
}

static int sunxi_dsi_connector_enable(struct tvpanel_connector *conn)
{
	struct tvpanel_dsi *dsi = conn_to_dsi(conn);

	if (!dsi || !dsi->dev)
		return -EINVAL;

	if (devm_request_threaded_irq(dsi->dev, dsi->irq,
				      tvpanel_dsi_irq_handler, NULL, 0,
				      dev_name(dsi->dev), dsi))
		dev_err(dsi->dev, "Couldn't request the IRQ for dsi\n");

	sunxi_tcon_clk_enable(dsi->tcon, 1);
	sunxi_tcon_config(dsi->tcon, &dsi->para);
	sunxi_tvdisp_top_tcon_colorbar_enable(dsi->tcon_top, 0);
	sunxi_tvpanel_top_dsi_enable(dsi->tcon_top, 1);

	tvpanel_dsi_cfg(dsi->dsi_id, &dsi->para,
			dsi->dsi_data ? &dsi->dsi_data->phy_config : NULL);

	tvpanel_pin_set_state(dsi->dev, "active");

	tvpanel_dsi_io_open(dsi->dsi_id, &dsi->para,
			    dsi->dsi_data ? &dsi->dsi_data->phy_config : NULL);

	tvpanel_panel_prepare(conn->panel);

	tvpanel_dsi_clk_enable(dsi->dsi_id, &dsi->para);

	sunxi_tcon_module_enable(dsi->tcon, 1);

	tvpanel_dsi_open(dsi->dsi_id, &dsi->para);

	tvpanel_panel_enable(conn->panel);

	dsi->enabled = true;
	conn->is_enabled = true;
	return 0;
}

static int sunxi_dsi_connector_disable(struct tvpanel_connector *conn)
{
	struct tvpanel_dsi *dsi = conn_to_dsi(conn);

	if (!dsi || !dsi->dev)
		return -EINVAL;

	tvpanel_panel_disable(conn->panel);
	tvpanel_panel_unprepare(conn->panel);

	sunxi_tvpanel_top_dsi_enable(dsi->tcon_top, 0);
	tvpanel_pin_set_state(dsi->dev, "sleep");
	tvpanel_dsi_close(dsi->dsi_id);
	sunxi_tcon_module_enable(dsi->tcon, 0);
	sunxi_tcon_clk_enable(dsi->tcon, 0);

	devm_free_irq(dsi->dev, dsi->irq, dsi);

	dsi->enabled = false;
	conn->is_enabled = false;
	return 0;
}

/*
 * Vendor sunxi_dsi_connector_vrr() as the ->adjust_refresh_rate op:
 * rate is in Hz, htotal/vtotal the new totals, each applied only when it
 * actually changes something.
 */
static int sunxi_dsi_connector_vrr(struct tvpanel_connector *conn,
				   unsigned int rate, unsigned int htotal,
				   unsigned int vtotal)
{
	struct tvpanel_dsi *dsi = conn_to_dsi(conn);
	int pll, next;

	if (!dsi || !dsi->dev)
		return -EINVAL;

	if (!dsi->enabled) {
		dev_err(dsi->dev, "Connector is not enabled yet!\n");
		return -EINVAL;
	}

	if (rate && conn->timing.pixelclock.typ != rate) {
		pll = sunxi_tvdisp_top_get_working_pll_index(dsi->tcon_top);
		if (pll < 0) {
			dev_err(dsi->dev, "Wrong pll index:%d!\n", pll);
			return -EINVAL;
		}
		next = (pll == 0);
		sunxi_tvdisp_top_pll_setting(dsi->tcon_top, &dsi->para, rate,
					     next, 1);
		sunxi_tvdisp_top_set_working_pll_index(dsi->tcon_top, next);
		conn->timing.pixelclock.typ = rate;
	}

	if (conn->htotal != htotal && conn->timing.hactive.typ < htotal) {
		conn->htotal = htotal;
		tvpanel_dsi_adjust_htotal(dsi->dsi_id, htotal, &dsi->para);
	}

	if (conn->vtotal != vtotal && conn->timing.vactive.typ < vtotal) {
		conn->vtotal = vtotal;
		tvpanel_dsi_adjust_vtotal(dsi->dsi_id, vtotal, &dsi->para);
	}

	return 0;
}

/*
 * Software enable: everything except the TCON and DSI mode setup, for when
 * the display core has already programmed those.
 */
static int sunxi_dsi_connector_sw_enable(struct tvpanel_connector *conn)
{
	struct tvpanel_dsi *dsi = conn_to_dsi(conn);
	unsigned int i;

	if (!dsi || !dsi->dev)
		return -EINVAL;

	if (devm_request_threaded_irq(dsi->dev, dsi->irq,
				      tvpanel_dsi_irq_handler, NULL, 0,
				      dev_name(dsi->dev), dsi))
		dev_err(dsi->dev, "Couldn't request the IRQ for dsi\n");

	sunxi_tvpanel_top_clk_enable(dsi->tcon_top);
	sunxi_tcon_clk_enable(dsi->tcon, 1);

	for (i = 0; i < ARRAY_SIZE(dsi->clks); i++) {
		if (dsi->rsts[i])
			reset_control_deassert(dsi->rsts[i]);
		if (!dsi->clks[i])
			continue;
		if (dsi->clk_freq[i]) {
			dev_dbg(dsi->dev, "set clk%u dsi rate:%u\n", i,
				dsi->clk_freq[i]);
			clk_set_rate(dsi->clks[i], dsi->clk_freq[i]);
		}
		if (!clk_prepare(dsi->clks[i]) && clk_enable(dsi->clks[i]))
			clk_unprepare(dsi->clks[i]);
	}

	tvpanel_pin_set_state(dsi->dev, "active");
	tvpanel_panel_sw_enable(conn->panel);

	dsi->enabled = true;
	conn->is_enabled = true;
	return 0;
}

const struct tvpanel_connector_funcs tvpanel_dsi_conn_funcs = {
	.get_timing		= sunxi_dsi_connector_get_timing,
	.enable			= sunxi_dsi_connector_enable,
	.disable		= sunxi_dsi_connector_disable,
	.prepare		= sunxi_dsi_connector_prepare,
	.unprepare		= sunxi_dsi_connector_unprepare,
	.sw_enable		= sunxi_dsi_connector_sw_enable,
	.adjust_refresh_rate	= sunxi_dsi_connector_vrr,
};
EXPORT_SYMBOL_GPL(tvpanel_dsi_conn_funcs);

MODULE_LICENSE("GPL v2");
