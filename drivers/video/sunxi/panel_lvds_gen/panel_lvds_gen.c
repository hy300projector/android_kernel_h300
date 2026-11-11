// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allwinner H726 (sun50iw15p1) generic LVDS panel driver.
 *
 * Reconstruction of the vendor panel_lvds_gen.ko module: it parses an LVDS
 * panel node (timings, supplies, enable GPIOs and the whole set of
 * panel_* LVDS properties) and registers itself with the tvpanel panel
 * framework as connector type 1.
 *
 * The panel-facing structures come from ../tvpanel/tvpanel.h and are the
 * BTF-recovered vendor ABI, so this module binds to a prebuilt tvpanel.ko, and
 * the vendor's own panel modules keep working against a from-source tvpanel.
 *
 * Everything below follows the vendor binary (Hex-Rays output in
 * modules_dec/panel_lvds_gen.c) function by function; deviations are called out
 * in comments.
 */
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <video/display_timing.h>
#include <video/of_display_timing.h>

#include "../tvpanel/tvpanel.h"

/* the vendor always requests exactly three panel supplies (v4[39..41]) */
#define LVDS_MAX_POWER		3
/* "gpio-num" default when the node does not say */
#define LVDS_DEFAULT_GPIO_NUM	3

struct panel_lvds {
	struct tvpanel_panel panel;		/* must stay first */
	struct device *dev;
	struct display_timing *timing;		/* v4[11], native timing */

	u32 power_delay_ms;			/* v4[12] "power-delay-ms" */
	u32 enable_delay_ms;			/* v4[13] "enable-delay-ms" */
	u32 protocol;				/* v4[14] "panel_protocol" */
	u32 channel;				/* v4[15] "reg" */
	u32 dual_port;				/* v4[16] "panel_dual_port" */
	u32 bitwidth;				/* v4[17] "panel_bitwidth" */
	u32 data_swap;				/* v4[18] "panel_data_swap" */
	u32 de_invert;				/* v4[19] "panel_de_invert" */
	u32 hs_invert;				/* v4[20] "panel_hs_invert" */
	u32 vs_invert;				/* v4[21] "panel_vs_invert" */
	u32 dclk_invert;			/* v4[22] "panel_dclk_invert" */
	u32 lane_number;			/* v4[23] default 4 */
	u32 de_current;				/* v4[24] */
	u32 odd_data_current;			/* v4[25] */
	u32 even_data_current;			/* v4[26] */
	u32 ssc_en;				/* v4[27] */
	u32 ssc_step;				/* v4[28] */
	u32 ssc_span;				/* v4[29] */
	u32 diff_amplifier_lv;			/* v4[30] default 4 */
	u32 common_amplifier_lv;		/* v4[31] default 3 */
	u32 poweron_delay0;			/* v4[32] "panel_poweron_delay0" */
	u32 poweron_delay1;			/* v4[33] "panel_poweron_delay1" */
	u32 powerdown_delay0;			/* v4[34] "panel_powerdown_delay0" */
	u32 powerdown_delay1;			/* v4[35] "panel_powerdown_delay1" */
	u32 lvds0_pol;				/* v4[36] "panel_lvds0_pol" */
	u32 lvds1_pol;				/* v4[37] "panel_lvds1_pol" */

	struct regulator *power[LVDS_MAX_POWER];	/* v4[39..41] */
	struct gpio_desc **enable_gpio;			/* v4[42] */
	u32 gpio_num;					/* v4[43] */
};

static struct panel_lvds *to_panel_lvds(struct tvpanel_panel *panel)
{
	return container_of(panel, struct panel_lvds, panel);
}

/* same helper the vendor inlines everywhere: 0 skips, <21 ms sleeps finely */
static void panel_lvds_delay(u32 ms)
{
	if (!ms)
		return;
	if (ms < 21)
		usleep_range(1000 * ms, 1000 * ms + 1000);
	else
		msleep(ms);
}

/* ------------------------------------------------------------------ */
/* device tree                                                         */
/* ------------------------------------------------------------------ */

/*
 * "panel_bitwidth" (6..10) to the LCD_LVDS_*_bit depth the DSI/LVDS layers
 * expect.  The vendor keeps this as a switch; the five-entry table lives at
 * .rodata+0x1a8 as {0, 1, 1, 1, 2} and anything outside 6..10 falls back to 1.
 */
static const u8 panel_lvds_bit_depth_map[] = { 0, 1, 1, 1, 2 };

static int panel_lvds_parse_dt(struct panel_lvds *pd)
{
	struct device *dev = pd->dev;
	struct device_node *np = dev->of_node;
	struct display_timings *timings;
	char *name;
	u32 index, i;
	int ret;

	timings = of_get_display_timings(np);
	if (!timings)
		goto bad_timing;

	index = timings->native_mode;
	if (timings->num_timings <= index) {
		pd->timing = NULL;
		goto bad_timing;
	}

	pd->timing = timings->timings[index];
	if (!pd->timing)
		goto bad_timing;

	/* exactly three supplies, every one of them optional */
	for (i = 0; i < ARRAY_SIZE(pd->power); i++) {
		name = kasprintf(GFP_KERNEL, "power%d", i);
		if (!name)
			return -ENOMEM;

		pd->power[i] = devm_regulator_get_optional(dev, name);
		ret = IS_ERR(pd->power[i]) ? PTR_ERR(pd->power[i]) : 0;
		if (ret && ret != -ENODEV && ret != -EPROBE_DEFER)
			dev_err(dev, "failed to request regulator(%s): %d\n",
				name, ret);
		kfree(name);

		if (ret && ret != -ENODEV)
			return ret;
		if (ret == -ENODEV)
			pd->power[i] = NULL;
	}

	pd->gpio_num = LVDS_DEFAULT_GPIO_NUM;
	of_property_read_u32(np, "gpio-num", &pd->gpio_num);

	if (pd->gpio_num) {
		pd->enable_gpio = devm_kcalloc(dev, pd->gpio_num,
					       sizeof(*pd->enable_gpio),
					       GFP_KERNEL);
		if (!pd->enable_gpio) {
			dev_err(dev, "Fail to malloc %d num gpio desc",
				pd->gpio_num);
			return -ENOMEM;
		}

		for (i = 0; i < pd->gpio_num; i++) {
			name = kasprintf(GFP_KERNEL, "enable%d", i);
			if (!name)
				return -ENOMEM;

			pd->enable_gpio[i] =
				devm_gpiod_get_optional(dev, name,
							GPIOD_OUT_HIGH);
			if (IS_ERR(pd->enable_gpio[i])) {
				ret = PTR_ERR(pd->enable_gpio[i]);
				dev_err(dev,
					"failed to request %s GPIO: %d\n",
					name, ret);
				/*
				 * The vendor logs and keeps the ERR_PTR, which
				 * its own prepare/unprepare then hand to
				 * gpiod_set_value_cansleep(); store NULL so a
				 * missing line stays inert instead.
				 */
				pd->enable_gpio[i] = NULL;
			}
			kfree(name);
		}
	}

	of_property_read_u32(np, "power-delay-ms", &pd->power_delay_ms);
	of_property_read_u32(np, "enable-delay-ms", &pd->enable_delay_ms);
	of_property_read_u32(np, "panel_poweron_delay0", &pd->poweron_delay0);
	of_property_read_u32(np, "panel_poweron_delay1", &pd->poweron_delay1);
	of_property_read_u32(np, "panel_powerdown_delay0",
			     &pd->powerdown_delay0);
	of_property_read_u32(np, "panel_powerdown_delay1",
			     &pd->powerdown_delay1);
	of_property_read_u32(np, "panel_protocol", &pd->protocol);
	of_property_read_u32(np, "panel_dual_port", &pd->dual_port);
	of_property_read_u32(np, "panel_bitwidth", &pd->bitwidth);
	of_property_read_u32(np, "panel_data_swap", &pd->data_swap);
	of_property_read_u32(np, "panel_de_invert", &pd->de_invert);
	of_property_read_u32(np, "panel_vs_invert", &pd->vs_invert);
	of_property_read_u32(np, "panel_hs_invert", &pd->hs_invert);
	of_property_read_u32(np, "panel_dclk_invert", &pd->dclk_invert);
	of_property_read_u32(np, "panel_lvds0_pol", &pd->lvds0_pol);
	of_property_read_u32(np, "panel_lvds1_pol", &pd->lvds1_pol);
	of_property_read_u32(np, "reg", &pd->channel);

	if (of_property_read_u32(np, "panel_lane_number", &pd->lane_number) < 0)
		pd->lane_number = 4;

	of_property_read_u32(np, "panel_de_current", &pd->de_current);
	of_property_read_u32(np, "panel_odd_data_current", &pd->odd_data_current);
	of_property_read_u32(np, "panel_even_data_current",
			     &pd->even_data_current);
	of_property_read_u32(np, "panel_ssc_en", &pd->ssc_en);
	of_property_read_u32(np, "panel_ssc_step", &pd->ssc_step);
	of_property_read_u32(np, "panel_ssc_span", &pd->ssc_span);

	if (of_property_read_u32(np, "panel_lvds_diff_amplifier_lv",
				 &pd->diff_amplifier_lv) < 0)
		pd->diff_amplifier_lv = 4;

	if (of_property_read_u32(np, "panel_lvds_common_amplifier_lv",
				 &pd->common_amplifier_lv) < 0)
		pd->common_amplifier_lv = 3;

	return 0;

bad_timing:
	dev_err(dev, "%pOF: problems parsing panel-timin\n", np);
	return -ENODEV;
}

/* ------------------------------------------------------------------ */
/* panel operations                                                    */
/* ------------------------------------------------------------------ */

/*
 * vendor panel_lvds_prepare(): drive every enable line low, wait 10 ms, bring
 * the supplies up in order, then raise the enable lines and wait
 * panel_poweron_delay0.
 *
 * A failed regulator does not abort the sequence: the vendor logs it and jumps
 * straight to raising the enable lines, which is what the break below does.
 */
static int panel_lvds_prepare(struct tvpanel_panel *panel)
{
	struct panel_lvds *pd = to_panel_lvds(panel);
	u32 i;
	int ret;

	for (i = 0; i < pd->gpio_num; i++)
		if (pd->enable_gpio[i])
			gpiod_set_value_cansleep(pd->enable_gpio[i], 0);

	usleep_range(10000, 11000);

	for (i = 0; i < ARRAY_SIZE(pd->power); i++) {
		if (!pd->power[i])
			continue;

		ret = regulator_enable(pd->power[i]);
		if (ret < 0) {
			dev_err(pd->dev, "failed to enable supply%d: %d\n", i,
				ret);
			break;
		}
		panel_lvds_delay(pd->power_delay_ms);
	}

	for (i = 0; i < pd->gpio_num; i++) {
		if (!pd->enable_gpio[i])
			continue;
		gpiod_set_value_cansleep(pd->enable_gpio[i], 1);
		panel_lvds_delay(pd->enable_delay_ms);
	}

	panel_lvds_delay(pd->poweron_delay0);
	return 0;
}

static int panel_lvds_unprepare(struct tvpanel_panel *panel)
{
	struct panel_lvds *pd = to_panel_lvds(panel);
	int i;

	panel_lvds_delay(pd->powerdown_delay1);

	/* enable lines drop in reverse order, then power2..power0 */
	for (i = pd->gpio_num - 1; i >= 0; i--) {
		if (!pd->enable_gpio[i])
			continue;
		gpiod_set_value_cansleep(pd->enable_gpio[i], 0);
		panel_lvds_delay(pd->enable_delay_ms);
	}

	for (i = ARRAY_SIZE(pd->power) - 1; i >= 0; i--) {
		if (!pd->power[i])
			continue;
		regulator_disable(pd->power[i]);
		panel_lvds_delay(pd->power_delay_ms);
	}

	return 0;
}

/* the vendor waits panel_poweron_delay1 / panel_powerdown_delay1 here */
static int panel_lvds_enable(struct tvpanel_panel *panel)
{
	struct panel_lvds *pd = to_panel_lvds(panel);

	panel_lvds_delay(pd->poweron_delay1);
	return 0;
}

static int panel_lvds_disable(struct tvpanel_panel *panel)
{
	struct panel_lvds *pd = to_panel_lvds(panel);

	panel_lvds_delay(pd->powerdown_delay1);
	return 0;
}

/* the vendor hands out the single native timing it cached in parse_dt */
static int panel_lvds_get_native_timing(struct tvpanel_panel *panel,
					struct display_timing **timing)
{
	struct panel_lvds *pd = to_panel_lvds(panel);

	*timing = pd->timing;
	return 0;
}

/*
 * vendor panel_lvds_get_extra_para(): translate the panel_* properties into
 * disp_panel_para.  Note that the vendor feeds panel_ssc_span into ssc_freq and
 * panel_ssc_step into ssc_percent, and pins ssc_mode to 3.
 */
static int panel_lvds_get_extra_para(struct tvpanel_panel *panel,
				     struct disp_panel_para *para)
{
	struct panel_lvds *pd = to_panel_lvds(panel);
	u32 index;
	u8 bit_depth = 1;

	if (!para)
		return -EINVAL;

	para->ssc_en = pd->ssc_en;
	para->ssc_freq = pd->ssc_span;
	para->ssc_mode = 3;
	para->ssc_percent = pd->ssc_step;

	para->lvds_even_odd_dir = pd->data_swap;

	index = pd->bitwidth - 6;
	if (index < ARRAY_SIZE(panel_lvds_bit_depth_map))
		bit_depth = panel_lvds_bit_depth_map[index];
	para->lvds_bit_depth = bit_depth;

	para->lvds_if = pd->dual_port;
	para->lvds_mode = pd->protocol;
	para->lcd_lvds_de_invert = pd->de_invert;
	para->lcd_lvds_hs_invert = pd->hs_invert;
	para->lcd_lvds_vs_invert = pd->vs_invert;
	para->lcd_lvds_clk_invert = pd->dclk_invert;
	para->lvds_data_lane_num = pd->lane_number;
	para->lcd_lvds_diff_amplifier_lv = pd->diff_amplifier_lv;
	para->lcd_lvds_common_amplifier_lv = pd->common_amplifier_lv;
	para->power_on_delay0 = pd->poweron_delay0;
	para->power_on_delay1 = pd->poweron_delay1;
	para->power_down_delay0 = pd->powerdown_delay0;
	para->power_down_delay1 = pd->powerdown_delay1;
	para->lcd_lvds_channel = pd->channel;
	para->lvds0_lane_np_inv = pd->lvds0_pol;
	para->lvds1_lane_np_inv = pd->lvds1_pol;

	return 0;
}

/*
 * The vendor table fills six of eight slots: 0..4 are the operations above,
 * slot 5 (get_timings) and slot 6 (get_dsc_para) are zero in .rodata, and slot
 * 7 is get_extra_para (the only relocation after the hole).
 */
static const struct tvpanel_panel_funcs panel_lvds_funcs = {
	.unprepare = panel_lvds_unprepare,
	.prepare = panel_lvds_prepare,
	.enable = panel_lvds_enable,
	.disable = panel_lvds_disable,
	.get_native_timing = panel_lvds_get_native_timing,
	.get_timings = NULL,
	.get_dsc_para = NULL,
	.get_extra_para = panel_lvds_get_extra_para,
};
static_assert(sizeof(struct tvpanel_panel_funcs) == 8 * 4);

/* ------------------------------------------------------------------ */
/* probe / remove                                                      */
/* ------------------------------------------------------------------ */

static int panel_lvds_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct panel_lvds *pd;
	int ret;

	dev_info(dev, "[LVDS-PANEL] panel_lvds_probe start\n");

	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	pd->dev = dev;

	ret = panel_lvds_parse_dt(pd);
	if (ret)
		return ret;

	tvpanel_panel_panel_init(&pd->panel, dev, &panel_lvds_funcs,
				 TVPANEL_CONN_LVDS);

	ret = tvpanel_panel_of_backlight(&pd->panel);
	if (ret)
		dev_info(dev, "[LVDS-PANEL] tvpanel_panel_of_backlight fail!:%d\n",
			 ret);

	tvpanel_panel_add(&pd->panel);
	dev_set_drvdata(dev, pd);

	/*
	 * The vendor adds the component on dev->parent, not on the panel device:
	 * it hands the panel's ops to whatever controller owns the panel node.
	 * sunxi_tvpanel_lvds_add() is the component_add() wrapper for that.
	 */
	if (dev->parent)
		sunxi_tvpanel_lvds_add(dev->parent);

	dev_info(dev, "[LVDS-PANEL] panel_lvds_probe finish\n");
	return ret;
}

static int panel_lvds_remove(struct platform_device *pdev)
{
	struct panel_lvds *pd = dev_get_drvdata(&pdev->dev);

	if (!pd)
		return 0;

	tvpanel_panel_remove(&pd->panel);
	panel_lvds_delay(pd->powerdown_delay1);
	panel_lvds_unprepare(&pd->panel);
	return 0;
}

static const struct of_device_id panel_lvds_of_table[] = {
	{ .compatible = "allwinner,panel_lvds_gen" },
	{ }
};
MODULE_DEVICE_TABLE(of, panel_lvds_of_table);

static struct platform_driver panel_lvds_driver = {
	.probe = panel_lvds_probe,
	.remove = panel_lvds_remove,
	.driver = {
		.name = "panel_lvds",
		.of_match_table = panel_lvds_of_table,
	},
};
module_platform_driver(panel_lvds_driver);

MODULE_DESCRIPTION("LVDS Panel Driver");
MODULE_AUTHOR("xiaozhineng <xiaozhineng@allwinnertech.com>");
MODULE_LICENSE("GPL v2");
