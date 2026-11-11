// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allwinner H726 tvpanel - panel framework, connector registry and platform
 * glue.
 *
 * Function bodies here follow the vendor binary (tvpanel.ko) where it was
 * decompiled; the panel/connector structures are the BTF-recovered ABI so the
 * vendor panel modules can bind against them unchanged.
 */
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/slab.h>

#include "tvpanel.h"

#define TVPANEL_MAX_CLK	5

static DEFINE_MUTEX(panel_lock);
static DECLARE_COMPLETION(tvpanel_init_done);
static LIST_HEAD(panel_list);
static struct tvpanel_device *g_tpdev;

struct tvpanel_device *tvpanel_get_device(void)
{
	return g_tpdev;
}

/* ------------------------------------------------------------------ */
/* panel registry                                                      */
/* ------------------------------------------------------------------ */

/*
 * vendor tvpanel_panel_panel_init(): fill in the panel a panel driver has
 * allocated, before it calls tvpanel_panel_add().
 *
 * The vendor writes exactly five words - dev, funcs, both list pointers and
 * connector_type - and leaves everything else alone, because its callers pass
 * a fresh block and set the remaining fields themselves.  Reproduced as-is
 * rather than memsetting, so a caller that keeps state in the unused fields
 * sees the same behaviour.
 */
int tvpanel_panel_panel_init(struct tvpanel_panel *panel, struct device *dev,
			     const struct tvpanel_panel_funcs *funcs,
			     int connector_type)
{
	if (!panel || !dev || !funcs)
		return -EINVAL;

	INIT_LIST_HEAD(&panel->list);
	panel->dev = dev;
	panel->funcs = funcs;
	panel->connector_type = connector_type;
	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_panel_panel_init);

/*
 * vendor tvpanel_panel_of_backlight(): attach whatever backlight the panel
 * node references.  The vendor returns the devm_of_find_backlight() error
 * unchanged and -EINVAL for a panel without a device; a node with no
 * backlight property yields NULL, which counts as success and is stored.
 */
int tvpanel_panel_of_backlight(struct tvpanel_panel *panel)
{
	struct backlight_device *bd;

	if (!panel || !panel->dev)
		return -EINVAL;

	bd = devm_of_find_backlight(panel->dev);
	if (IS_ERR(bd))
		return PTR_ERR(bd);

	panel->backlight = bd;
	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_panel_of_backlight);

/*
 * Export class: the vendor puts the whole panel-side set in __ksymtab_gpl
 * (the GPL marker is the section name, not a symbol prefix), and every vendor
 * panel module is GPL, so matching it keeps prebuilt panel modules loading.
 */
int tvpanel_panel_add(struct tvpanel_panel *panel)
{
	if (!panel)
		return -EINVAL;

	mutex_lock(&panel_lock);
	list_add_tail(&panel->list, &panel_list);
	mutex_unlock(&panel_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_panel_add);

int tvpanel_panel_remove(struct tvpanel_panel *panel)
{
	if (!panel)
		return -EINVAL;

	mutex_lock(&panel_lock);
	list_del(&panel->list);
	mutex_unlock(&panel_lock);
	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_panel_remove);

struct tvpanel_panel *of_tvpanel_find_panel(struct device_node *np)
{
	struct tvpanel_panel *panel;

	if (!np)
		return ERR_PTR(-EINVAL);

	if (!of_device_is_available(np)) {
		pr_info("tvpanel: panel node %pOF is not available\n", np);
		return ERR_PTR(-ENODEV);
	}

	mutex_lock(&panel_lock);
	list_for_each_entry(panel, &panel_list, list) {
		if (panel->dev && panel->dev->of_node == np) {
			mutex_unlock(&panel_lock);
			return panel;
		}
	}
	mutex_unlock(&panel_lock);
	return NULL;
}
EXPORT_SYMBOL_GPL(of_tvpanel_find_panel);

/* the panel node a device points at, as the LVDS/DSI probes need it */
struct tvpanel_panel *tvpanel_get_panel_dev(struct device *dev)
{
	struct tvpanel_panel *panel;

	if (!dev)
		return NULL;

	panel = of_tvpanel_find_panel(dev->of_node);
	return IS_ERR(panel) ? NULL : panel;
}
EXPORT_SYMBOL_GPL(tvpanel_get_panel_dev);

/* ------------------------------------------------------------------ */
/* panel power sequencing                                              */
/* ------------------------------------------------------------------ */

int tvpanel_panel_prepare(struct tvpanel_panel *panel)
{
	int ret;

	if (!panel)
		return -EINVAL;

	if (panel->prepared) {
		dev_warn(panel->dev, "Skipping prepare of already prepared panel\n");
		return 0;
	}

	if (panel->funcs && panel->funcs->prepare) {
		ret = panel->funcs->prepare(panel);
		if (ret < 0)
			return ret;
	}

	panel->prepared = true;
	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_panel_prepare);

int tvpanel_panel_unprepare(struct tvpanel_panel *panel)
{
	int ret;

	if (!panel)
		return -EINVAL;

	if (!panel->prepared) {
		dev_warn(panel->dev, "Skipping unprepare of unprepared panel\n");
		return 0;
	}

	if (panel->funcs && panel->funcs->unprepare) {
		ret = panel->funcs->unprepare(panel);
		if (ret < 0)
			return ret;
	}

	panel->prepared = false;
	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_panel_unprepare);

int tvpanel_panel_enable(struct tvpanel_panel *panel)
{
	int ret;

	if (!panel)
		return -EINVAL;

	if (panel->enabled) {
		dev_warn(panel->dev, "Skipping enable of already enabled panel\n");
		return 0;
	}

	if (panel->funcs && panel->funcs->enable) {
		ret = panel->funcs->enable(panel);
		if (ret < 0)
			return ret;
	}

	panel->enabled = true;
	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_panel_enable);

int tvpanel_panel_disable(struct tvpanel_panel *panel)
{
	int ret;

	if (!panel)
		return -EINVAL;

	if (!panel->enabled) {
		dev_warn(panel->dev, "Skipping disable of already disabled panel\n");
		return 0;
	}

	if (panel->funcs && panel->funcs->disable) {
		ret = panel->funcs->disable(panel);
		if (ret < 0)
			return ret;
	}

	panel->enabled = false;
	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_panel_disable);

/* ------------------------------------------------------------------ */
/* timing <-> panel parameters                                         */
/* ------------------------------------------------------------------ */

/*
 * Member-for-member port of the vendor display_timing_to_panel_para().
 * See the comment above struct tvpanel_timing for why this cannot be a cast
 * to struct display_timing.
 */
int display_timing_to_panel_para(const struct tvpanel_timing *t,
				 struct disp_panel_para *para)
{
	if (!t || !para)
		return -EINVAL;

	para->lcd_dclk_freq = t->pixelclock.typ;
	para->lcd_x = t->hactive.typ;
	para->lcd_y = t->vactive.typ;
	para->lcd_hspw = t->hsync_len.typ;
	para->lcd_hbp = t->hback_porch.typ;
	para->lcd_hfp = t->hfront_porch.typ;
	para->lcd_vbp = t->vback_porch.typ;
	para->lcd_vspw = t->vsync_len.typ;
	para->lcd_vfp = t->vfront_porch.typ;

	para->lcd_ht = t->hactive.typ + t->hfront_porch.typ +
		       t->hback_porch.typ + t->hsync_len.typ;
	para->lcd_vt = t->vactive.typ + t->vfront_porch.typ +
		       t->vback_porch.typ + t->vsync_len.typ;
	return 0;
}
EXPORT_SYMBOL_GPL(display_timing_to_panel_para);

int panel_para_to_display_timing(const struct disp_panel_para *para,
				 struct tvpanel_timing *t)
{
	if (!para || !t)
		return -EINVAL;

	t->pixelclock.typ = para->lcd_dclk_freq;
	t->hactive.typ = para->lcd_x;
	t->vactive.typ = para->lcd_y;
	t->hfront_porch.typ = para->lcd_hfp;
	t->hback_porch.typ = para->lcd_hbp;
	t->hsync_len.typ = para->lcd_hspw;
	t->vfront_porch.typ = para->lcd_vfp;
	t->vback_porch.typ = para->lcd_vbp;
	t->vsync_len.typ = para->lcd_vspw;
	return 0;
}
EXPORT_SYMBOL_GPL(panel_para_to_display_timing);

/* Boundary adapters to the kernel's own struct display_timing. */
void tvpanel_timing_from_display_timing(const struct display_timing *dt,
					struct tvpanel_timing *t)
{
	memset(t, 0, sizeof(*t));
	t->pixelclock = dt->pixelclock;
	t->hactive = dt->hactive;
	t->hfront_porch = dt->hfront_porch;
	t->hback_porch = dt->hback_porch;
	t->hsync_len = dt->hsync_len;
	t->vactive = dt->vactive;
	t->vfront_porch = dt->vfront_porch;
	t->vback_porch = dt->vback_porch;
	t->vsync_len = dt->vsync_len;
	t->flags = dt->flags;
}
EXPORT_SYMBOL_GPL(tvpanel_timing_from_display_timing);

void tvpanel_timing_to_display_timing(const struct tvpanel_timing *t,
				      struct display_timing *dt)
{
	memset(dt, 0, sizeof(*dt));
	dt->pixelclock = t->pixelclock;
	dt->hactive = t->hactive;
	dt->hfront_porch = t->hfront_porch;
	dt->hback_porch = t->hback_porch;
	dt->hsync_len = t->hsync_len;
	dt->vactive = t->vactive;
	dt->vfront_porch = t->vfront_porch;
	dt->vback_porch = t->vback_porch;
	dt->vsync_len = t->vsync_len;
	dt->flags = t->flags;
}
EXPORT_SYMBOL_GPL(tvpanel_timing_to_display_timing);

/* ------------------------------------------------------------------ */
/* connector lookup                                                    */
/* ------------------------------------------------------------------ */

struct tvpanel_connector *get_tvpanel_connector_by_type(int type, int id)
{
	struct tvpanel_connector *conn;

	if (!g_tpdev)
		return NULL;

	list_for_each_entry(conn, &g_tpdev->connector_list, head) {
		if (conn->connector_type == type && conn->connector_id == id)
			return conn;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(get_tvpanel_connector_by_type);

struct tvpanel_connector *get_first_tvpanel_connector(struct tvpanel_device *tpdev)
{
	struct tvpanel_connector *conn;

	if (!tpdev)
		return NULL;

	list_for_each_entry(conn, &tpdev->connector_list, head)
		return conn;
	return NULL;
}
EXPORT_SYMBOL_GPL(get_first_tvpanel_connector);

struct tvpanel_connector *get_tvpanel_connector_by_device(int type, int id)
{
	return get_tvpanel_connector_by_type(type, id);
}
EXPORT_SYMBOL_GPL(get_tvpanel_connector_by_device);

/* ------------------------------------------------------------------ */
/* platform glue                                                       */
/* ------------------------------------------------------------------ */

static int tvpanel_parse_clocks(struct platform_device *pdev,
				struct tvpanel_dsi *dsi)
{
	struct device *dev = &pdev->dev;
	int n = of_property_count_strings(dev->of_node, "clock-names");
	int i, ret;

	if (n < 0)
		return 0;
	if (n > TVPANEL_MAX_CLK) {
		dev_warn(dev, "device tree lists %d clocks, using %d\n",
			 n, TVPANEL_MAX_CLK);
		n = TVPANEL_MAX_CLK;
	}

	for (i = 0; i < n; i++) {
		const char *name;

		ret = of_property_read_string_index(dev->of_node, "clock-names",
						    i, &name);
		if (ret)
			return ret;

		dsi->clks[i] = devm_clk_get_optional(dev, name);
		if (IS_ERR(dsi->clks[i]))
			return PTR_ERR(dsi->clks[i]);
	}
	return 0;
}

static int tvpanel_parse_resets(struct platform_device *pdev,
				struct tvpanel_dsi *dsi)
{
	struct device *dev = &pdev->dev;
	struct reset_control *rst;
	int i = 0;
	int ret;

	while (i < TVPANEL_MAX_CLK) {
		char name[16];

		snprintf(name, sizeof(name), "rst%d", i);
		rst = devm_reset_control_get_optional_exclusive(dev, name);
		if (IS_ERR(rst))
			return PTR_ERR(rst);
		if (!rst)
			break;
		dsi->rsts[i] = rst;
		i++;
	}
	ret = 0;
	return ret;
}

static int tvpanel_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tvpanel_dsi *dsi;
	struct tvpanel_device *tpdev;
	void __iomem *base, *res;
	int irq, ret;

	dev_info(dev, "probing tvpanel\n");

	tpdev = devm_kzalloc(dev, sizeof(*tpdev), GFP_KERNEL);
	if (!tpdev)
		return -ENOMEM;
	tpdev->dev = dev;
	INIT_LIST_HEAD(&tpdev->connector_list);
	INIT_LIST_HEAD(&tpdev->display_list);

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	/*
	 * DT regs: index 0 is the DSI controller (0x1fff bytes; the D-PHY sits
	 * one page in), index 1 is the 4-byte dsi_res register.
	 */
	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	res = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(res))
		res = NULL;	/* dsi_res is optional for a DSI-less (LVDS) panel */

	ret = tvpanel_dsi_set_reg_base(0, base, res);
	if (ret)
		return ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0 && irq != -ENXIO)
		return irq;

	dsi->dev = dev;
	dsi->dsi_id = 0;
	dsi->irq = irq;
	dsi->conn.dev = dev;
	dsi->conn.tpdev = tpdev;
	dsi->conn.panel = of_tvpanel_find_panel(dev->of_node);
	dsi->tcon_top = tvpanel_get_tcon_top();
	dsi->tcon = tvpanel_get_tcon();
	if (!dsi->tcon_top || !dsi->tcon)
		dev_warn(dev, "display top/tcon not ready yet\n");

	ret = tvpanel_parse_clocks(pdev, dsi);
	if (ret)
		return ret;
	ret = tvpanel_parse_resets(pdev, dsi);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, dsi);
	g_tpdev = tpdev;

	dev_info(dev, "tvpanel ready (irq %d, res_base %s)\n", irq,
		 res ? "mapped" : "absent");
	return 0;
}

static int tvpanel_remove(struct platform_device *pdev)
{
	g_tpdev = NULL;
	return 0;
}

static const struct of_device_id tvpanel_of_match[] = {
	{ .compatible = "allwinner,tvpanel" },
	{ }
};
MODULE_DEVICE_TABLE(of, tvpanel_of_match);

static struct platform_driver tvpanel_driver = {
	.probe	= tvpanel_probe,
	.remove	= tvpanel_remove,
	.driver	= {
		.name		= "tvpanel",
		.of_match_table	= tvpanel_of_match,
	},
};
/*
 * The vendor's own display core (vs-display.ko) drives the first connector
 * through these five entry points and they are exported from tvpanel.ko, so a
 * prebuilt consumer expects them by name.  They are thin wrappers over the
 * connector ops, exactly as in the vendor binary: the first registered
 * connector is the one the TV backend drives.
 */
static struct tvpanel_connector *tvpanel_first_connector(void)
{
	struct tvpanel_device *tpdev = tvpanel_get_device();

	if (!tpdev)
		return NULL;
	return get_first_tvpanel_connector(tpdev);
}

int sunxi_tvpanel_prepare(void)
{
	struct tvpanel_connector *conn = tvpanel_first_connector();
	int ret;

	if (!conn) {
		pr_err("tvpanel connector not found\n");
		return -1;
	}
	if (!conn->funcs || !conn->funcs->prepare)
		return -1;

	ret = conn->funcs->prepare(conn);
	if (ret) {
		dev_err(conn->dev, "Connector prepare fail!\n");
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_tvpanel_prepare);

int sunxi_tvpanel_unprepare(void)
{
	struct tvpanel_connector *conn = tvpanel_first_connector();
	int ret;

	if (!conn) {
		pr_err("tvpanel connector not found\n");
		return -1;
	}
	if (!conn->funcs || !conn->funcs->unprepare)
		return -1;

	ret = conn->funcs->unprepare(conn);
	if (ret) {
		dev_err(conn->dev, "Connector prepare fail!\n");
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_tvpanel_unprepare);

int sunxi_tvpanel_enable(void)
{
	struct tvpanel_connector *conn = tvpanel_first_connector();
	int ret;

	if (!conn) {
		pr_err("tvpanel connector not found\n");
		return -1;
	}
	if (!conn->funcs || !conn->funcs->enable)
		return -1;
	if (conn->is_enabled) {
		dev_err(conn->dev, "Connector has already been enabled!\n");
		return 0;
	}

	ret = conn->funcs->enable(conn);
	if (ret) {
		dev_err(conn->dev, "Connector enable fail!\n");
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_tvpanel_enable);

int sunxi_tvpanel_disable(void)
{
	struct tvpanel_connector *conn = tvpanel_first_connector();
	int ret;

	if (!conn) {
		pr_err("tvpanel connector not found\n");
		return -1;
	}
	if (!conn->funcs || !conn->funcs->disable || !conn->is_enabled)
		return 0;

	ret = conn->funcs->disable(conn);
	if (ret) {
		dev_err(conn->dev, "Connector enable fail!\n");
		return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(sunxi_tvpanel_disable);

/*
 * vendor sunxi_tvpanel_wait_init_done(): waits on the completion its panel
 * modules signal once their init work is done.  As in the vendor binary, a
 * timeout (0 left) is reported as true.
 */
bool sunxi_tvpanel_wait_init_done(void)
{
	long left = wait_for_completion_interruptible_timeout(&tvpanel_init_done,
							     100);

	if (!left)
		pr_err("wait tvpanel init done timeout\n");
	return left == 0;
}
EXPORT_SYMBOL_GPL(sunxi_tvpanel_wait_init_done);

/*
 * One init for the whole module: the vendor registers its sub-drivers (top,
 * TCON, DSI, LVDS) first and the panel framework driver last.
 */
static int __init tvpanel_init(void)
{
	int ret;

	ret = tvpanel_sub_drivers_register();
	if (ret)
		return ret;

	ret = platform_driver_register(&tvpanel_driver);
	if (ret) {
		tvpanel_sub_drivers_unregister();
		return ret;
	}

	complete_all(&tvpanel_init_done);
	return 0;
}

static void __exit tvpanel_exit(void)
{
	platform_driver_unregister(&tvpanel_driver);
	tvpanel_sub_drivers_unregister();
}

module_init(tvpanel_init);
module_exit(tvpanel_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Allwinner H726 tvpanel driver");
MODULE_AUTHOR("H726 mainline/Android port");
