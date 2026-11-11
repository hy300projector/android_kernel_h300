// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allwinner H726 (sun50iw15p1) generic DSI panel driver.
 *
 * Reconstruction of the vendor panel_dsi_gen.ko module: it parses a panel node
 * (timings, supplies, enable/reset GPIOs, init/exit DCS sequences and an
 * optional DSC configuration) and registers itself with the tvpanel panel
 * framework.
 *
 * The panel-facing structures come from ../tvpanel/tvpanel.h and are the
 * BTF-recovered vendor ABI, so this module binds to a prebuilt tvpanel.ko, and
 * the vendor's own panel modules keep working against a from-source tvpanel.
 *
 * Everything below follows the vendor binary (Hex-Rays output in
 * modules_dec/panel_dsi_gen.c) function by function; deviations are called out
 * in comments.
 */
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <drm/drm_dsc.h>
#include <drm/drm_mipi_dsi.h>
#include <video/display_timing.h>
#include <video/of_display_timing.h>

#include "../tvpanel/tvpanel.h"

/*
 * The vendor indexes 10 supplies (v5[19..28]) and 10 enable GPIOs
 * (v5[33..42]).  It does not range check the DT counts; a panel node asking
 * for more would walk off the end of the struct, so the counts are clamped
 * here instead.
 */
#define DSI_MAX_POWER		10
#define DSI_MAX_GPIO		10

/* [type][delay-ms][payload-length] in the DT blob */
#define DSI_CMD_HDR_LEN		3

/*
 * One packet of a DT init/exit sequence, in the 8-byte form the vendor parser
 * expands the blob into (v16[..]: u16 type+delay, u8 len, pad, payload).
 */
struct panel_dsi_cmd {
	u8 type;
	u8 delay;
	u8 len;
	u8 pad;
	const u8 *payload;
};
static_assert(sizeof(struct panel_dsi_cmd) == 8);

/* 8-byte holder allocated per sequence property (vendor v10[7]/v10[8]) */
struct panel_dsi_cmd_seq {
	const struct panel_dsi_cmd *cmds;
	unsigned int count;
};
static_assert(sizeof(struct panel_dsi_cmd_seq) == 8);

/*
 * 52-byte panel descriptor, layout as in the vendor module (v10[] indices).
 * Word offsets: 0 timings, 1 width-mm, 2 height-mm, 3 reset-num,
 * 4 power-delay-ms, 5 enable-delay-ms, 6 reset-delay-ms, 7 init, 8 exit,
 * 9 dsc, 10 dsi,flags, 11 dsi,format, 12 dsi,lanes.
 */
struct panel_dsi_desc {
	struct display_timings *timings;	/* 0 */
	u32 width_mm;				/* 4 */
	u32 height_mm;				/* 8 */
	u32 reset_num;				/* 12 */
	u32 power_delay_ms;			/* 16 */
	u32 enable_delay_ms;			/* 20 */
	u32 reset_delay_ms;			/* 24 */
	struct panel_dsi_cmd_seq *init_seq;	/* 28 */
	struct panel_dsi_cmd_seq *exit_seq;	/* 32 */
	struct drm_dsc_config *dsc;		/* 36 */
	u32 flags;				/* 40, "dsi,flags" */
	u32 format;				/* 44, "dsi,format" */
	u32 lanes;				/* 48, "dsi,lanes" */
};
static_assert(sizeof(struct panel_dsi_desc) == 52);

/*
 * The vendor's DSC block is the mainline struct drm_dsc_config: every offset it
 * writes (0 line_buf_depth, 1 bits_per_component, 3 slice_count, 4 slice_width,
 * 6 slice_height, 16 bits_per_pixel, 26 block_pred_enable, 126 native_422,
 * 127 native_420) lands on that struct's members, and the 0x88-byte copy in
 * ->get_dsc_para() is its size.
 */
static_assert(sizeof(struct drm_dsc_config) == 0x88);

struct panel_dsi {
	struct tvpanel_panel panel;		/* must stay first */
	struct device *dev;
	struct mipi_dsi_device *dsi;
	const struct panel_dsi_desc *desc;

	/* extra parameters handed to the connector through ->get_extra_para() */
	u32 vrr_step;				/* "dsc,vrr-setp" */
	u32 ssc_en;				/* "dsi,ssc_en" */
	u32 ssc_percent;			/* "dsi,ssc_percent" */
	u32 ssc_mode;				/* "dsi,ssc_mode" */
	u32 ssc_freq;				/* "dsi,ssc_freq" */

	u32 power_num;				/* "power-num", default 3 */
	u32 gpio_num;				/* "gpio-num", default 3 */
	struct regulator *power[DSI_MAX_POWER];
	struct regulator *avdd;
	struct regulator *avee;
	u32 avdd_voltage;			/* "avdd-output-voltage" */
	u32 avee_voltage;			/* "avee-output-voltage" */
	struct gpio_desc *enable_gpio[DSI_MAX_GPIO];
	struct gpio_desc *reset_gpio;
	const struct drm_dsc_config *dsc;
};

static struct panel_dsi *to_panel_dsi(struct tvpanel_panel *panel)
{
	return container_of(panel, struct panel_dsi, panel);
}

/*
 * The vendor's delay helper everywhere: values below 21 ms use
 * usleep_range(ms*1000, ms*1000+1000), longer ones msleep(), and 0 skips.
 */
static void panel_dsi_delay(u32 ms)
{
	if (!ms)
		return;
	if (ms < 21)
		usleep_range(1000 * ms, 1000 * ms + 1000);
	else
		msleep(ms);
}

/* ------------------------------------------------------------------ */
/* power and GPIO sequencing                                           */
/* ------------------------------------------------------------------ */

/*
 * vendor panel_dsi_regulator_enable(): supplies in order, then avdd and avee
 * with their programmed voltage, each followed by power-delay-ms.  The vendor
 * passes the same value as min and max to regulator_set_voltage().
 *
 * Exported because the vendor exports it (plain EXPORT_SYMBOL, no in-tree
 * importer), so the symbol surface stays identical.  The argument is the panel
 * pointer, which is the start of this struct, exactly as in the vendor module.
 */
int panel_dsi_regulator_enable(struct tvpanel_panel *panel)
{
	struct panel_dsi *pd = to_panel_dsi(panel);
	int i, ret;

	for (i = 0; i < pd->power_num; i++) {
		if (!pd->power[i])
			continue;

		ret = regulator_enable(pd->power[i]);
		if (ret < 0) {
			dev_err(pd->dev, "failed to enable supply%d: %d\n", i, ret);
			return ret;
		}
		panel_dsi_delay(pd->desc->power_delay_ms);
	}

	if (pd->avdd) {
		ret = regulator_enable(pd->avdd);
		if (ret < 0) {
			/* the vendor logs its (stale) supply loop index here */
			dev_err(pd->dev, "failed to enable supply%d: %d\n", i, ret);
			return ret;
		}
		if (pd->avdd_voltage)
			regulator_set_voltage(pd->avdd, pd->avdd_voltage,
					      pd->avdd_voltage);
		panel_dsi_delay(pd->desc->power_delay_ms);
	}

	if (pd->avee) {
		ret = regulator_enable(pd->avee);
		if (ret < 0) {
			dev_err(pd->dev, "failed to enable supply%d: %d\n", i, ret);
			return ret;
		}
		if (pd->avee_voltage)
			regulator_set_voltage(pd->avee, pd->avee_voltage,
					      pd->avee_voltage);
		panel_dsi_delay(pd->desc->power_delay_ms);
	}

	return 0;
}
EXPORT_SYMBOL(panel_dsi_regulator_enable);

/* ------------------------------------------------------------------ */
/* DCS / generic sequence handling                                     */
/* ------------------------------------------------------------------ */

/*
 * Expand a "panel-init-sequence"/"panel-exit-sequence" blob into the packet
 * array.  Faithful to the vendor's arithmetic, including its two quirks: a
 * truncated packet inside the buffer returns -EINVAL without a message, while
 * a buffer that does not tile exactly (1..3 trailing bytes) logs
 * "dts panel-init data length error".
 */
static int panel_dsi_parse_cmd_seq(struct device *dev, const void *data,
				   unsigned int len,
				   struct panel_dsi_cmd_seq *seq)
{
	const u8 *walk = data, *start = data;
	unsigned int remaining = len, count = 0, i;

	if (len >= DSI_CMD_HDR_LEN + 1) {
		do {
			u8 plen = walk[2];

			if (remaining - DSI_CMD_HDR_LEN < plen)
				return -EINVAL;
			remaining -= DSI_CMD_HDR_LEN + plen;
			walk += DSI_CMD_HDR_LEN + plen;
			count++;
		} while (remaining > DSI_CMD_HDR_LEN);

		if (remaining)
			goto bad_length;
	} else if (len) {
		goto bad_length;
	}

	/* vendor: count >= 0x20000000 gives up and leaves the array NULL */
	if (count >= 0x20000000) {
		seq->cmds = NULL;
		return -ENOMEM;
	}

	seq->count = count;
	seq->cmds = devm_kzalloc(dev, count * sizeof(*seq->cmds), GFP_KERNEL);
	if (!seq->cmds)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		struct panel_dsi_cmd *cmd = (struct panel_dsi_cmd *)&seq->cmds[i];

		cmd->type = start[0];
		cmd->delay = start[1];
		cmd->len = start[2];
		cmd->payload = start + DSI_CMD_HDR_LEN;
		start += DSI_CMD_HDR_LEN + cmd->len;
	}

	return 0;

bad_length:
	dev_err(dev, "dts panel-init data length error:%d\n", remaining);
	return -EINVAL;
}

/*
 * vendor panel_dsi_cmd_seq(): short/long generic and DCS writes.  Write
 * failures are logged but do not abort the sequence; an unknown type aborts it.
 */
static int panel_dsi_cmd_seq(struct panel_dsi *pd,
			     const struct panel_dsi_cmd_seq *seq)
{
	unsigned int i;
	int ret;

	if (!seq || !seq->count || !pd->dsi)
		return 0;

	for (i = 0; i < seq->count; i++) {
		const struct panel_dsi_cmd *cmd = &seq->cmds[i];

		switch (cmd->type) {
		case 0x03:	/* generic short write, no parameter */
		case 0x13:	/* generic short write, 1 parameter */
		case 0x23:	/* generic short write, 2 parameters */
		case 0x29:	/* generic long write */
			ret = mipi_dsi_generic_write(pd->dsi, cmd->payload,
						     cmd->len);
			break;
		case 0x05:	/* DCS short write, no parameter */
		case 0x15:	/* DCS short write, 1 parameter */
		case 0x39:	/* DCS long write */
			ret = mipi_dsi_dcs_write_buffer(pd->dsi, cmd->payload,
							cmd->len);
			break;
		default:
			dev_err(pd->dev,
				"There are %d packets in total, the %d packet length is error!!!\n",
				seq->count, i);
			return -EINVAL;
		}

		if (ret < 0)
			dev_err(pd->dev, "failed to write dcs cmd: %d\n", ret);

		panel_dsi_delay(cmd->delay);
	}

	return 0;
}

/* ------------------------------------------------------------------ */
/* panel operations                                                    */
/* ------------------------------------------------------------------ */

static int panel_dsi_unprepare(struct tvpanel_panel *panel)
{
	struct panel_dsi *pd = to_panel_dsi(panel);
	const struct panel_dsi_desc *desc = pd->desc;
	int i, ret;

	if (desc->exit_seq && pd->dsi)
		panel_dsi_cmd_seq(pd, desc->exit_seq);

	/* with DSC configured the panel is shut down over DCS first */
	if (desc->dsc) {
		ret = mipi_dsi_dcs_set_display_off(pd->dsi);
		if (ret < 0) {
			dev_err(pd->dev, "Failed to set display off: %d\n", ret);
			return ret;
		}
		msleep(20);

		ret = mipi_dsi_dcs_enter_sleep_mode(pd->dsi);
		if (ret < 0) {
			dev_err(pd->dev, "Failed to enter sleep mode: %d\n", ret);
			return ret;
		}
		msleep(120);
	}

	/* enable GPIOs are released in reverse order */
	for (i = pd->gpio_num - 1; i >= 0; i--) {
		if (!pd->enable_gpio[i])
			continue;
		gpiod_set_value_cansleep(pd->enable_gpio[i], 0);
		panel_dsi_delay(desc->enable_delay_ms);
	}

	if (pd->reset_gpio)
		gpiod_set_value_cansleep(pd->reset_gpio, 0);
	panel_dsi_delay(desc->reset_delay_ms);

	if (pd->avdd) {
		regulator_disable(pd->avdd);
		panel_dsi_delay(desc->power_delay_ms);
	}

	if (pd->avee) {
		regulator_disable(pd->avee);
		panel_dsi_delay(desc->power_delay_ms);
	}

	for (i = pd->power_num - 1; i >= 0; i--) {
		if (!pd->power[i])
			continue;
		regulator_disable(pd->power[i]);
		panel_dsi_delay(desc->power_delay_ms);
	}

	return 0;
}

static int panel_dsi_prepare(struct tvpanel_panel *panel)
{
	struct panel_dsi *pd = to_panel_dsi(panel);
	const struct panel_dsi_desc *desc = pd->desc;
	unsigned int i;

	panel_dsi_regulator_enable(panel);

	for (i = 0; i < pd->gpio_num; i++) {
		if (!pd->enable_gpio[i])
			continue;
		gpiod_set_value_cansleep(pd->enable_gpio[i], 1);
		panel_dsi_delay(desc->enable_delay_ms);
	}

	/*
	 * "reset-num" pulse trains.  The vendor keeps the delays outside the
	 * gpio checks, so they happen even on a panel without a reset line.
	 */
	for (i = 0; i < desc->reset_num; i++) {
		if (pd->reset_gpio)
			gpiod_set_value_cansleep(pd->reset_gpio, 0);
		panel_dsi_delay(desc->reset_delay_ms);

		if (pd->reset_gpio)
			gpiod_set_value_cansleep(pd->reset_gpio, 1);
		panel_dsi_delay(desc->reset_delay_ms);
	}

	if (desc->init_seq && pd->dsi)
		panel_dsi_cmd_seq(pd, desc->init_seq);

	return 0;
}

/* both are no-ops in the vendor module: enable/disable only sequence power */
static int panel_dsi_enable(struct tvpanel_panel *panel)
{
	return 0;
}

static int panel_dsi_disable(struct tvpanel_panel *panel)
{
	return 0;
}

/*
 * vendor panel_dsi_get_native_timing(): picks display_timings->native_mode,
 * falling back to the first entry when it is out of range, and returns -1 when
 * the node has no timings at all.
 */
static int panel_dsi_get_native_timing(struct tvpanel_panel *panel,
				       struct display_timing **timing)
{
	struct panel_dsi *pd = to_panel_dsi(panel);
	struct display_timings *timings = pd->desc->timings;
	unsigned int index;

	if (!timings || !timings->num_timings)
		return -1;

	index = timings->native_mode;
	if (index > timings->num_timings - 1)
		index = 0;

	*timing = timings->timings[index];
	return 0;
}

/* 0x88-byte copy of the parsed DSC configuration, -EINVAL when there is none */
static int panel_dsi_get_dsc_para(struct tvpanel_panel *panel, void *dsc)
{
	struct panel_dsi *pd = to_panel_dsi(panel);
	const struct drm_dsc_config *src;

	if (!dsc)
		return -EINVAL;

	src = pd->desc->dsc;
	if (!src)
		return -EINVAL;

	memcpy(dsc, src, sizeof(*src));
	return 0;
}

/*
 * vendor panel_dsi_get_extra_para(): the DSI panel only contributes the spread
 * spectrum settings, and pins ssc_mode to 3 regardless of the DT.
 */
static int panel_dsi_get_extra_para(struct tvpanel_panel *panel,
				    struct disp_panel_para *para)
{
	struct panel_dsi *pd = to_panel_dsi(panel);

	if (!para)
		return -EINVAL;

	para->ssc_en = pd->ssc_en;
	para->ssc_mode = 3;
	para->ssc_freq = pd->ssc_freq;
	para->ssc_percent = pd->ssc_percent;

	return 0;
}

/*
 * The vendor module's table leaves slot 5 (get_timings) empty: there is no
 * relocation at that offset in the binary, unlike slots 0-4, 6 and 7.
 */
static const struct tvpanel_panel_funcs panel_dsi_funcs = {
	.unprepare = panel_dsi_unprepare,
	.prepare = panel_dsi_prepare,
	.enable = panel_dsi_enable,
	.disable = panel_dsi_disable,
	.get_native_timing = panel_dsi_get_native_timing,
	.get_timings = NULL,
	.get_dsc_para = panel_dsi_get_dsc_para,
	.get_extra_para = panel_dsi_get_extra_para,
};
static_assert(sizeof(struct tvpanel_panel_funcs) == 8 * 4);

/* ------------------------------------------------------------------ */
/* device tree                                                         */
/* ------------------------------------------------------------------ */

static int panel_dsi_parse_dt(struct panel_dsi *pd, struct device_node *np)
{
	struct device *dev = pd->dev;
	struct panel_dsi_desc *desc;
	struct drm_dsc_config *dsc;
	struct panel_dsi_cmd_seq *seq;
	const void *raw, *blob;
	const char *status;
	u32 val;
	int len, ret;

	desc = devm_kzalloc(dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	desc->timings = of_get_display_timings(np);
	if (!desc->timings) {
		dev_err(dev, "%pOF: problems parsing panel-timin\n", np);
		return -ENODEV;
	}

	of_property_read_u32(np, "width-mm", &desc->width_mm);
	of_property_read_u32(np, "height-mm", &desc->height_mm);
	of_property_read_u32(np, "power-delay-ms", &desc->power_delay_ms);
	of_property_read_u32(np, "enable-delay-ms", &desc->enable_delay_ms);
	of_property_read_u32(np, "reset-delay-ms", &desc->reset_delay_ms);
	of_property_read_u32(np, "reset-num", &desc->reset_num);

	if (!of_property_read_string(np, "dsc,status", &status) &&
	    !strncmp(status, "okay", 4)) {
		dsc = devm_kzalloc(dev, sizeof(*dsc), GFP_KERNEL);
		if (!dsc)
			return -ENOMEM;
		desc->dsc = dsc;

		if (!of_property_read_u32(np, "dsc,slice-height", &val))
			dsc->slice_height = val;
		if (!of_property_read_u32(np, "dsc,slice-width", &val))
			dsc->slice_width = val;
		if (!of_property_read_u32(np, "dsc,bits-per-component", &val))
			dsc->bits_per_component = val;
		if (!of_property_read_u32(np, "dsc,block-pred-enable", &val))
			dsc->block_pred_enable = !!val;

		/* the vendor's fixed values, in its write order */
		dsc->native_420 = true;		/* byte 127 */
		dsc->native_422 = true;		/* byte 126 */
		dsc->slice_count = 2;		/* byte 3 */
		dsc->bits_per_pixel = 128;	/* 8 bpp with 4 fractional bits */
		dsc->line_buf_depth = 9;	/* byte 0 */
	}

	/*
	 * The vendor copies the blob with devm_kmemdup() and keeps pointers into
	 * the copy (cmd->payload), which is what makes those pointers survive for
	 * the module's lifetime; the property buffer is walked twice otherwise.
	 */
	raw = of_get_property(np, "panel-init-sequence", &len);
	if (raw) {
		blob = devm_kmemdup(dev, raw, len, GFP_KERNEL);
		if (!blob)
			return -ENOMEM;

		seq = devm_kzalloc(dev, sizeof(*seq), GFP_KERNEL);
		if (!seq)
			return -ENOMEM;

		ret = panel_dsi_parse_cmd_seq(dev, blob, len, seq);
		if (ret) {
			dev_err(dev, "failed to parse init sequence\n");
			return ret;
		}
		desc->init_seq = seq;
	}

	raw = of_get_property(np, "panel-exit-sequence", &len);
	if (raw) {
		blob = devm_kmemdup(dev, raw, len, GFP_KERNEL);
		if (!blob)
			return -ENOMEM;

		seq = devm_kzalloc(dev, sizeof(*seq), GFP_KERNEL);
		if (!seq)
			return -ENOMEM;

		ret = panel_dsi_parse_cmd_seq(dev, blob, len, seq);
		if (ret) {
			dev_err(dev, "failed to parse exit sequence\n");
			return ret;
		}
		desc->exit_seq = seq;
	}

	of_property_read_u32(np, "dsi,flags", &desc->flags);
	of_property_read_u32(np, "dsi,format", &desc->format);
	of_property_read_u32(np, "dsi,lanes", &desc->lanes);

	pd->desc = desc;
	return 0;
}

/* ------------------------------------------------------------------ */
/* probe / remove                                                      */
/* ------------------------------------------------------------------ */

static const struct of_device_id panel_dsi_of_match[] = {
	{ .compatible = "allwinner,panel_dsi_gen" },
	{ }
};
MODULE_DEVICE_TABLE(of, panel_dsi_of_match);

static int panel_dsi_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *np = dev->of_node;
	const struct panel_dsi_desc *match_desc;
	const struct of_device_id *match;
	struct panel_dsi *pd;
	char *name;
	int err, ret;
	u32 i;

	dev_info(dev, "[DSI-PANEL] panel_dsi_probe start\n");

	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	pd->dev = dev;
	pd->dsi = dsi;

	match = of_match_node(panel_dsi_of_match, np);
	match_desc = match ? match->data : NULL;

	if (match_desc) {
		/* a pre-baked descriptor in the match table wins over the DT */
		pd->desc = match_desc;
	} else {
		ret = panel_dsi_parse_dt(pd, np);
		if (ret) {
			dev_err(dev, "failed to get desc data: %d\n", ret);
			return ret;
		}
	}

	dsi->mode_flags = pd->desc->flags;
	dsi->format = pd->desc->format;
	dsi->lanes = pd->desc->lanes;

	/* "power-num" supplies, default 3, each optional */
	pd->power_num = 3;
	of_property_read_u32(np, "power-num", &pd->power_num);
	if (pd->power_num > ARRAY_SIZE(pd->power))
		pd->power_num = ARRAY_SIZE(pd->power);

	for (i = 0; i < pd->power_num; i++) {
		struct regulator *reg;

		name = kasprintf(GFP_KERNEL, "power%d", i);
		if (!name)
			return -ENOMEM;

		reg = devm_regulator_get_optional(dev, name);
		err = IS_ERR(reg) ? PTR_ERR(reg) : 0;
		if (err && err != -ENODEV && err != -EPROBE_DEFER)
			dev_err(dev, "failed to request regulator(%s): %d\n",
				name, err);
		kfree(name);

		if (err && err != -ENODEV)
			return err;

		pd->power[i] = err == -ENODEV ? NULL : reg;
	}

	pd->avdd = devm_regulator_get_optional(dev, "avdd");
	if (IS_ERR(pd->avdd)) {
		err = PTR_ERR(pd->avdd);
		if (err != -ENODEV) {
			if (err != -EPROBE_DEFER)
				dev_err(dev,
					"failed to request regulator(%s): %d\n",
					"avdd", err);
			return err;
		}
		pd->avdd = NULL;
	} else {
		of_property_read_u32(np, "avdd-output-voltage",
				     &pd->avdd_voltage);
	}

	pd->avee = devm_regulator_get_optional(dev, "avee");
	if (IS_ERR(pd->avee)) {
		err = PTR_ERR(pd->avee);
		if (err != -ENODEV) {
			if (err != -EPROBE_DEFER)
				dev_err(dev,
					"failed to request regulator(%s): %d\n",
					"avee", err);
			return err;
		}
		pd->avee = NULL;
	} else {
		of_property_read_u32(np, "avee-output-voltage",
				     &pd->avee_voltage);
	}

	/* "gpio-num" enable lines, default 3, requested as outputs driven high */
	pd->gpio_num = 3;
	of_property_read_u32(np, "gpio-num", &pd->gpio_num);
	if (pd->gpio_num > ARRAY_SIZE(pd->enable_gpio))
		pd->gpio_num = ARRAY_SIZE(pd->enable_gpio);

	for (i = 0; i < pd->gpio_num; i++) {
		name = kasprintf(GFP_KERNEL, "enable%d", i);
		if (!name)
			return -ENOMEM;

		pd->enable_gpio[i] = devm_gpiod_get_optional(dev, name,
							     GPIOD_OUT_HIGH);
		if (IS_ERR(pd->enable_gpio[i])) {
			err = PTR_ERR(pd->enable_gpio[i]);
			dev_err(dev, "failed to request %s GPIO: %d\n", name,
				err);
			kfree(name);
			return err;
		}
		kfree(name);
	}

	pd->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(pd->reset_gpio)) {
		err = PTR_ERR(pd->reset_gpio);
		dev_err(dev, "failed to request %s GPIO: %d\n", "reset", err);
		return err;
	}

	of_property_read_u32(np, "dsc,vrr-setp", &pd->vrr_step);
	of_property_read_u32(np, "dsi,ssc_en", &pd->ssc_en);
	of_property_read_u32(np, "dsi,ssc_mode", &pd->ssc_mode);
	of_property_read_u32(np, "dsi,ssc_percent", &pd->ssc_percent);
	of_property_read_u32(np, "dsi,ssc_freq", &pd->ssc_freq);

	pd->dsc = pd->desc->dsc;

	tvpanel_panel_panel_init(&pd->panel, dev, &panel_dsi_funcs,
				 TVPANEL_CONN_DSI);
	tvpanel_panel_of_backlight(&pd->panel);
	tvpanel_panel_add(&pd->panel);

	dev_set_drvdata(dev, pd);
	mipi_dsi_attach(dsi);

	dev_info(dev, "[DSI-PANEL] panel_dsi_probe finish\n");
	return 0;
}

static int panel_dsi_remove(struct mipi_dsi_device *dsi)
{
	struct panel_dsi *pd = dev_get_drvdata(&dsi->dev);

	if (!pd)
		return 0;

	tvpanel_panel_remove(&pd->panel);
	panel_dsi_unprepare(&pd->panel);
	return 0;
}

static struct mipi_dsi_driver panel_dsi_driver = {
	.probe = panel_dsi_probe,
	.remove = panel_dsi_remove,
	.driver = {
		.name = "panel-dsi",
		.of_match_table = panel_dsi_of_match,
	},
};

static int __init panel_dsi_init(void)
{
	int ret;

	ret = mipi_dsi_driver_register_full(&panel_dsi_driver, THIS_MODULE);
	if (ret)
		printk("[DSI-PANEL] dsi driver regsiter fail\n");

	return ret;
}

static void __exit panel_dsi_exit(void)
{
	mipi_dsi_driver_unregister(&panel_dsi_driver);
}

module_init(panel_dsi_init);
module_exit(panel_dsi_exit);

/* metadata as the vendor module carries it */
MODULE_DESCRIPTION("dsi Panel Driver");
MODULE_VERSION("1.0.0");
MODULE_LICENSE("GPL");
