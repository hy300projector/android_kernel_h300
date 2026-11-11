// SPDX-License-Identifier: GPL-2.0
/*
 * Allwinner sun50iw15 (H726 / HY300) internal audio codec
 *
 * Open replacement for the vendor snd_soc_sunxi_internal_codec module
 * (compatible "allwinner,sunxi-snd-codec", vendor source path
 * bsp/drivers/sound/platform/snd_sun50iw15_codec.c).
 *
 * Register offsets, masks and values come from the decompiled vendor module
 * (dec_mod/snd_sunxi_internal_codec.c): sunxi_codec_component_probe(),
 * sunxi_codec_dai_hw_params(), sunxi_codec_dai_prepare(),
 * sunxi_codec_dai_trigger() and the sunxi_*_event() DAPM handlers.
 *
 * The block has three register windows on the SoC:
 *   region 0  0x2030000  len 0x34c   digital + analog control (main)
 *   region 1  0x2031000  len 0x7c    DAC FIFO / DMA interface
 *   region 2  0x2032000  len 0x7c    ADC FIFO / DMA interface
 * and four clocks: clk_bus_adda, clk_pll_audio_4x, clk_audio_dac, clk_audio_adc.
 *
 * Not implemented: the vendor's pa-pin GPIO helper, jack detection and the
 * sunxi_rx_sync framework, all of which are separate vendor subsystems.
 */

#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <sound/jack.h>
#include <sound/soc-jack.h>
#include <sound/tlv.h>

/* Main block (region 0) */
#define SUNXI_CODEC_CTRL		0x00
#define SUNXI_CODEC_CTRL_DAC_ON		BIT(31)	/* playback DAPM event */
#define SUNXI_CODEC_CTRL_MODE_MASK	GENMASK(5, 4)
#define SUNXI_CODEC_CTRL_MODE_I2S	(0x1 << 4)
#define SUNXI_CODEC_CTRL_ADC_DST_I2S	BIT(8)
#define SUNXI_CODEC_CTRL_DAC_SRC_I2S	BIT(29)
#define SUNXI_CODEC_CTRL_TX_PIN_SHIFT	12
#define SUNXI_CODEC_CTRL_TX_PIN_MASK	GENMASK(17, 12)

#define SUNXI_CODEC_FMT			0x04
#define SUNXI_CODEC_FMT_DAC_DIV_SHIFT	8
#define SUNXI_CODEC_FMT_ADC_DIV_SHIFT	0
#define SUNXI_CODEC_FMT_DSP_MODE	BIT(30)
#define SUNXI_CODEC_FMT_LRCK_POL	BIT(7)
#define SUNXI_CODEC_FMT_FRAME_MASK	GENMASK(17, 8)

#define SUNXI_CODEC_DAC_CTRL		0x10
#define SUNXI_CODEC_DAC_CTRL_EN		BIT(0)
#define SUNXI_CODEC_DAC_CTRL_UNMUTE	BIT(4)
#define SUNXI_CODEC_DAC_CTRL_WIDE	BIT(5)	/* not a 16-bit sample */
#define SUNXI_CODEC_DAC_CTRL_MONO	BIT(6)
#define SUNXI_CODEC_DAC_CTRL_W16_MASK	GENMASK(25, 24)	/* 16-bit sample */
#define SUNXI_CODEC_DAC_CTRL_W16		(BIT(24) | BIT(25))
#define SUNXI_CODEC_RATE_MASK		GENMASK(31, 29)
#define SUNXI_CODEC_RATE_SHIFT		29

#define SUNXI_CODEC_DAC_FIFO		0x14
#define SUNXI_CODEC_DAC_FIFO_CTRL	0x14
#define SUNXI_CODEC_DAC_FIFO_VAL	0x0e

#define SUNXI_CODEC_ADC_CTRL		0x30
#define SUNXI_CODEC_ADC_CTRL_EN		BIT(0)
#define SUNXI_CODEC_ADC_CTRL_LINEINL	BIT(12)
#define SUNXI_CODEC_ADC_CTRL_LINEINR	BIT(13)
#define SUNXI_CODEC_ADC_CTRL_WIDE	BIT(16)
#define SUNXI_CODEC_ADC_CTRL_ADC_EN	BIT(17)
#define SUNXI_CODEC_ADC_CTRL_W16_MASK	GENMASK(25, 24)
#define SUNXI_CODEC_ADC_CTRL_W16		BIT(24)
#define SUNXI_CODEC_ADC_CTRL_MODE_MASK	GENMASK(27, 26)
#define SUNXI_CODEC_ADC_CTRL_MODE_I2S	GENMASK(27, 26)

#define SUNXI_CODEC_ADC_FIFO_CTRL	0x38
#define SUNXI_CODEC_ADC_FIFO_VAL	0x0a
#define SUNXI_CODEC_ADC_FLUSH		0x44
#define SUNXI_CODEC_DAC_FLUSH		0x24

#define SUNXI_CODEC_ADC1_VOL		0x24
#define SUNXI_CODEC_ADC2_RATE		0x28
#define SUNXI_CODEC_DA2AD_MASK		GENMASK(1, 0)

/* Digital volume: DACL/DACR sit in 0x04, ADC1/ADC2 in 0x34 (8 bits each). */
#define SUNXI_CODEC_DAC_VOL		0x04
#define SUNXI_CODEC_ADC_VOL		0x34
#define SUNXI_CODEC_VOL_MASK		0xff

/* Offset 0 of the separate I2S TX (region 1) and RX (region 2) windows. */
#define SUNXI_CODEC_I2S_CTRL		0x00
#define SUNXI_CODEC_I2S_TX_ADC_DST	(BIT(8) | BIT(2) | BIT(0))
#define SUNXI_CODEC_I2S_RX_DAC_SRC	(BIT(1) | BIT(0))

#define SUNXI_CODEC_ADC1_CTRL		0x300
#define SUNXI_CODEC_ADC2_CTRL		0x304
#define SUNXI_CODEC_ADCX_LINEIN		BIT(23)
#define SUNXI_CODEC_ADCX_EN		BIT(31)
#define SUNXI_CODEC_ADCX_SRC		BIT(19)

#define SUNXI_CODEC_OUT_CTRL		0x310
#define SUNXI_CODEC_OUT_LO_L		BIT(12)
#define SUNXI_CODEC_OUT_LO_R		BIT(13)
#define SUNXI_CODEC_OUT_LO_L2		BIT(10)
#define SUNXI_CODEC_OUT_LO_R2		BIT(11)
#define SUNXI_CODEC_OUT_HP_L		BIT(14)
#define SUNXI_CODEC_OUT_HP_R		BIT(15)
#define SUNXI_CODEC_OUT_GAIN_MASK	GENMASK(2, 0)
#define SUNXI_CODEC_OUT_HP_GAIN_SHIFT	28

#define SUNXI_CODEC_OUT_CTRL2		0x31c
#define SUNXI_CODEC_OUT2_HP_EN		BIT(0)
#define SUNXI_CODEC_OUT2_LO_EN		BIT(1)
#define SUNXI_CODEC_OUT2_RAMP_MASK	GENMASK(6, 4)
#define SUNXI_CODEC_OUT2_LO_EN2		BIT(1)

#define SUNXI_CODEC_DAC_SRC		0x340
#define SUNXI_CODEC_DAC_SRC_I2S		(BIT(15) | BIT(17) | BIT(19))

#define SUNXI_CODEC_MAX_REG		0x34c

/* FIFO blocks (regions 1 and 2) */
#define SUNXI_CODEC_FIFO_CTRL		0x00
#define SUNXI_CODEC_FIFO_MODE_MASK	GENMASK(5, 4)
#define SUNXI_CODEC_FIFO_MODE_I2S	(0x1 << 4)
#define SUNXI_CODEC_FIFO_BCLK_MASTER	BIT(18)
#define SUNXI_CODEC_FIFO_LRCK_MASTER	BIT(17)
#define SUNXI_CODEC_FIFO_FMT		0x04
#define SUNXI_CODEC_FIFO_DSP_MODE	BIT(30)
#define SUNXI_CODEC_FIFO_LRCK_POL	BIT(7)
#define SUNXI_CODEC_FIFO_SAMPLE_MASK	GENMASK(6, 4)
#define SUNXI_CODEC_FIFO_SAMPLE(w)	(((w) / 4) << 4)
#define SUNXI_CODEC_FIFO_FRAME_MASK	GENMASK(17, 8)
#define SUNXI_CODEC_FIFO_FRAME(w)	(((w) - 1) << 8)
#define SUNXI_CODEC_FIFO_TDM_MASK	GENMASK(2, 0)
#define SUNXI_CODEC_FIFO_CLK_DIV	0x24
#define SUNXI_CODEC_FIFO_DIV_MASK	GENMASK(3, 0)
#define SUNXI_CODEC_FIFO_DIV(v)		((v) & 0xf)
#define SUNXI_CODEC_FIFO_DIV2_MASK	GENMASK(7, 4)

struct sunxi_codec {
	struct device *dev;
	struct regmap *regmap;		/* region 0: control */
	struct regmap *regmap_tx;	/* region 1: DAC FIFO */
	struct regmap *regmap_rx;	/* region 2: ADC FIFO */

	struct clk *clk_bus;
	struct clk *clk_pll;
	struct clk *clk_dac;
	struct clk *clk_adc;
	struct reset_control *rst;

	struct mutex lock;

	unsigned int dac_vol;
	unsigned int dacl_vol;
	unsigned int dacr_vol;
	unsigned int adc1_vol;
	unsigned int adc2_vol;
	unsigned int lineout_gain;
	unsigned int hpout_gain;
	unsigned int ramp_up;
	unsigned int ramp_down;

	bool playing;
	bool capturing;

	unsigned int dac_src;		/* 0 = audio FIFO, 1 = I2S */
	unsigned int adc_dst;		/* 0 = audio FIFO, 1 = I2S */
	unsigned int sample_rate;

	/* Speaker amplifier pins from pa-pin-max / pa-pin-N. */
	int pa_pin_count;
	int *pa_pins;
	u32 *pa_levels;
	u32 *pa_msleep;
	u32 *pa_msleep1;

	/* Headphone detection from hp-det-gpio / jack-det-level. */
	struct snd_soc_jack *jack;
	struct snd_soc_jack_gpio jack_gpio;
	unsigned int jack_det_level;
};

/*
 * Sample-rate codes.  The vendor keeps an 11-entry (rate, code) table
 * (sunxi_sample_rate_conv) and shifts the code into DAC/ADC ctrl[25:24]
 * and [31:29] fields; capture is limited to 48 kHz.
 */
static const struct {
	unsigned int rate;
	unsigned int code;
} sunxi_codec_rates[] = {
	{  44100, 0 }, {  48000, 0 }, {   8000, 5 }, {  32000, 1 },
	{  22050, 2 }, {  24000, 2 }, {  16000, 3 }, {  11025, 4 },
	{  12000, 4 }, { 192000, 6 }, {  96000, 7 },
};

static int sunxi_codec_rate_code(unsigned int rate)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(sunxi_codec_rates); i++)
		if (sunxi_codec_rates[i].rate == rate)
			return sunxi_codec_rates[i].code;

	return -EINVAL;
}

static void sunxi_codec_dap_dacdrc(struct regmap *r)
{
	static const struct { u16 reg; u32 val; } tbl[] = {
		{ 0x108, 0x000000db }, { 0x10c, 0x00000000 },
		{ 0x110, 0x00009de7 }, { 0x114, 0x0000000b },
		{ 0x118, 0x000077f0 }, { 0x11c, 0x000000ff },
		{ 0x120, 0x00004f8c }, { 0x124, 0x000000fe },
		{ 0x128, 0x0000d450 }, { 0x12c, 0x00000001 },
		{ 0x130, 0x00002bb0 }, { 0x134, 0x00000001 },
		{ 0x138, 0x00002bb0 }, { 0x13c, 0x00000352 },
		{ 0x140, 0x000069e0 }, { 0x144, 0x000000a2 },
		{ 0x148, 0x00002222 }, { 0x14c, 0x0000fe56 },
		{ 0x150, 0x0000cb10 }, { 0x154, 0x000001a9 },
		{ 0x158, 0x00000033 }, { 0x15c, 0x00000033 },
		{ 0x160, 0x00003334 }, { 0x164, 0x0000ff64 },
		{ 0x168, 0x00001741 }, { 0x16c, 0x000004fb },
		{ 0x170, 0x00009ed1 }, { 0x174, 0x000002aa },
		{ 0x178, 0x0000aaac }, { 0x17c, 0x0000fcad },
		{ 0x180, 0x00009620 }, { 0x184, 0x00000100 },
		{ 0x188, 0x00000000 }, { 0x18c, 0x00000001 },
		{ 0x190, 0x00007665 }, { 0x194, 0x00000000 },
		{ 0x198, 0x00000f04 }, { 0x19c, 0x00000352 },
		{ 0x1a0, 0x000069e0 }, { 0x1a4, 0x0000f95b },
		{ 0x1a8, 0x00002c3f }, { 0x1ac, 0x00000002 },
		{ 0x1b0, 0x00005600 }, { 0x1b4, 0x00000000 },
		{ 0x1b8, 0x00000100 },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(tbl); i++)
		regmap_write(r, tbl[i].reg, tbl[i].val);
}

static void sunxi_codec_dap_adcdrc(struct regmap *r)
{
	static const struct { u16 reg; u32 val; } tbl[] = {
		{ 0x208, 0x000000db }, { 0x20c, 0x00000001 },
		{ 0x210, 0x00007665 }, { 0x214, 0x0000000b },
		{ 0x218, 0x000077f0 }, { 0x21c, 0x000000fe },
		{ 0x220, 0x0000d450 }, { 0x224, 0x000000fe },
		{ 0x228, 0x0000d450 }, { 0x22c, 0x00000001 },
		{ 0x230, 0x00002bb0 }, { 0x234, 0x00000001 },
		{ 0x238, 0x00002bb0 }, { 0x23c, 0x00000352 },
		{ 0x240, 0x000069e0 }, { 0x244, 0x00000099 },
		{ 0x248, 0x0000999a }, { 0x24c, 0x0000fe56 },
		{ 0x250, 0x0000cb10 }, { 0x254, 0x000001a9 },
		{ 0x258, 0x00000033 }, { 0x25c, 0x00000033 },
		{ 0x260, 0x00003333 }, { 0x264, 0x0000ff55 },
		{ 0x268, 0x0000ead3 }, { 0x26c, 0x000004fb },
		{ 0x270, 0x00009ed1 }, { 0x274, 0x00002800 },
		{ 0x278, 0x00000000 }, { 0x27c, 0x0000fcad },
		{ 0x280, 0x00009620 }, { 0x284, 0x00000100 },
		{ 0x288, 0x00000000 }, { 0x28c, 0x00000001 },
		{ 0x290, 0x00007665 }, { 0x294, 0x00000000 },
		{ 0x298, 0x00000f04 }, { 0x29c, 0x000004fb },
		{ 0x2a0, 0x00009ed1 }, { 0x2a4, 0x0000fb04 },
		{ 0x2a8, 0x0000612f }, { 0x2ac, 0x00000002 },
		{ 0x2b0, 0x00005600 }, { 0x2b4, 0x00000000 },
		{ 0x2b8, 0x00000100 },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(tbl); i++)
		regmap_write(r, tbl[i].reg, tbl[i].val);
}

/* DAPM events */

static int sunxi_codec_playback_event(struct snd_soc_dapm_widget *w,
				      struct snd_kcontrol *k, int event)
{
	struct snd_soc_component *c = snd_soc_dapm_to_component(w->dapm);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(c);

	regmap_update_bits(codec->regmap, SUNXI_CODEC_CTRL,
			   SUNXI_CODEC_CTRL_DAC_ON,
			   event == SND_SOC_DAPM_POST_PMU ?
			   SUNXI_CODEC_CTRL_DAC_ON : 0);

	return 0;
}

static int sunxi_codec_lineout_event(struct snd_soc_dapm_widget *w,
				     struct snd_kcontrol *k, int event)
{
	struct snd_soc_component *c = snd_soc_dapm_to_component(w->dapm);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(c);
	bool on = event == SND_SOC_DAPM_POST_PMU;
	u32 lo = 0, lo2 = 0;

	if (!strcmp(w->name, "LINEOUTL")) {
		lo = SUNXI_CODEC_OUT_LO_L;
		lo2 = SUNXI_CODEC_OUT_LO_L2;
	} else {
		lo = SUNXI_CODEC_OUT_LO_R;
		lo2 = SUNXI_CODEC_OUT_LO_R2;
	}

	if (on) {
		/* Enable the line-out pair, then its driver stage. */
		regmap_update_bits(codec->regmap, SUNXI_CODEC_OUT_CTRL2,
				   SUNXI_CODEC_OUT2_LO_EN, SUNXI_CODEC_OUT2_LO_EN);
		regmap_update_bits(codec->regmap, SUNXI_CODEC_OUT_CTRL, lo2, lo2);
		regmap_update_bits(codec->regmap, SUNXI_CODEC_OUT_CTRL, lo, lo);
	} else {
		regmap_update_bits(codec->regmap, SUNXI_CODEC_OUT_CTRL, lo, 0);
		regmap_update_bits(codec->regmap, SUNXI_CODEC_OUT_CTRL, lo2, 0);
		regmap_update_bits(codec->regmap, SUNXI_CODEC_OUT_CTRL2,
				   SUNXI_CODEC_OUT2_LO_EN, 0);
	}

	return 0;
}

static int sunxi_codec_hpout_event(struct snd_soc_dapm_widget *w,
				   struct snd_kcontrol *k, int event)
{
	struct snd_soc_component *c = snd_soc_dapm_to_component(w->dapm);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(c);
	unsigned int t = event == SND_SOC_DAPM_POST_PMU ? codec->ramp_up :
							  codec->ramp_down;

	regmap_update_bits(codec->regmap, SUNXI_CODEC_OUT_CTRL2,
			   SUNXI_CODEC_OUT2_RAMP_MASK, t << 4);
	regmap_update_bits(codec->regmap, SUNXI_CODEC_OUT_CTRL2,
			   SUNXI_CODEC_OUT2_HP_EN,
			   event == SND_SOC_DAPM_POST_PMU ?
			   SUNXI_CODEC_OUT2_HP_EN : 0);

	/* The vendor waits for the ramp to finish before muting. */
	msleep(20);

	return 0;
}

static int sunxi_codec_linein_event(struct snd_soc_dapm_widget *w,
				    struct snd_kcontrol *k, int event)
{
	struct snd_soc_component *c = snd_soc_dapm_to_component(w->dapm);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(c);
	bool left = !strcmp(w->name, "LINEINL");
	u16 reg = left ? SUNXI_CODEC_ADC1_CTRL : SUNXI_CODEC_ADC2_CTRL;
	bool on = event == SND_SOC_DAPM_POST_PMU;

	regmap_update_bits(codec->regmap, reg, SUNXI_CODEC_ADCX_LINEIN,
			   on ? SUNXI_CODEC_ADCX_LINEIN : 0);
	regmap_update_bits(codec->regmap, reg, SUNXI_CODEC_ADCX_EN,
			   on ? SUNXI_CODEC_ADCX_EN : 0);

	return 0;
}

static void sunxi_codec_pa_pins(struct sunxi_codec *codec, bool on)
{
	unsigned int i;

	for (i = 0; i < codec->pa_pin_count; i++) {
		msleep(codec->pa_msleep[i]);
		gpio_set_value(codec->pa_pins[i],
			       on ? (int)codec->pa_levels[i] :
				    !(int)codec->pa_levels[i]);
		msleep(codec->pa_msleep1[i]);
	}
}

static int sunxi_codec_spk_event(struct snd_soc_dapm_widget *w,
				 struct snd_kcontrol *k, int event)
{
	struct snd_soc_component *c = snd_soc_dapm_to_component(w->dapm);
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(c);

	sunxi_codec_pa_pins(codec, event == SND_SOC_DAPM_POST_PMU);

	return 0;
}

static const struct snd_soc_dapm_widget sunxi_codec_dapm_widgets[] = {
	SND_SOC_DAPM_DAC_E("DAC", NULL, SND_SOC_NOPM, 0, 0,
			   sunxi_codec_playback_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_ADC("ADC", NULL, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_OUT_DRV_E("LINEOUTL", SND_SOC_NOPM, 0, 0, NULL, 0,
			       sunxi_codec_lineout_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUT_DRV_E("LINEOUTR", SND_SOC_NOPM, 0, 0, NULL, 0,
			       sunxi_codec_lineout_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUT_DRV_E("HPOUT", SND_SOC_NOPM, 0, 0, NULL, 0,
			       sunxi_codec_hpout_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_OUT_DRV_E("SPK", SND_SOC_NOPM, 0, 0, NULL, 0,
			       sunxi_codec_spk_event,
			       SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_PGA_E("LINEINL", SND_SOC_NOPM, 0, 0, NULL, 0,
			   sunxi_codec_linein_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_PGA_E("LINEINR", SND_SOC_NOPM, 0, 0, NULL, 0,
			   sunxi_codec_linein_event,
			   SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

	SND_SOC_DAPM_OUTPUT("LINEOUTLP_PIN"),
	SND_SOC_DAPM_OUTPUT("LINEOUTLN_PIN"),
	SND_SOC_DAPM_OUTPUT("LINEOUTRP_PIN"),
	SND_SOC_DAPM_OUTPUT("LINEOUTRN_PIN"),
	SND_SOC_DAPM_OUTPUT("HPOUTL_PIN"),
	SND_SOC_DAPM_OUTPUT("HPOUTR_PIN"),
	SND_SOC_DAPM_INPUT("LINEINL1_PIN"),
	SND_SOC_DAPM_INPUT("LINEINL2_PIN"),
	SND_SOC_DAPM_INPUT("LINEINR1_PIN"),
	SND_SOC_DAPM_INPUT("LINEINR2_PIN"),
};

static const struct snd_soc_dapm_route sunxi_codec_dapm_routes[] = {
	/* playback: DAC -> line out / headphone / speaker pins */
	{ "LINEOUTL", NULL, "DAC" },
	{ "LINEOUTR", NULL, "DAC" },
	{ "HPOUT", NULL, "DAC" },
	{ "SPK", NULL, "LINEOUTLP_PIN" },
	{ "SPK", NULL, "LINEOUTLN_PIN" },
	{ "SPK", NULL, "LINEOUTRP_PIN" },
	{ "SPK", NULL, "LINEOUTRN_PIN" },

	{ "LINEOUTLP_PIN", NULL, "LINEOUTL" },
	{ "LINEOUTLN_PIN", NULL, "LINEOUTL" },
	{ "LINEOUTRP_PIN", NULL, "LINEOUTR" },
	{ "LINEOUTRN_PIN", NULL, "LINEOUTR" },
	{ "HPOUTL_PIN", NULL, "HPOUT" },
	{ "HPOUTR_PIN", NULL, "HPOUT" },

	/* capture: line in pins -> ADC */
	{ "LINEINL", NULL, "LINEINL1_PIN" },
	{ "LINEINL", NULL, "LINEINL2_PIN" },
	{ "LINEINR", NULL, "LINEINR1_PIN" },
	{ "LINEINR", NULL, "LINEINR2_PIN" },
	{ "ADC", NULL, "LINEINL" },
	{ "ADC", NULL, "LINEINR" },
};

/*
 * DAC source routing, as done by the vendor's sunxi_set_dacsrc_mode():
 * 0 feeds the DAC from the audio FIFO (the "aaudio" platform), 1 from I2S.
 * The vendor also writes bits 2/1 of a second register window that cannot be
 * identified from the decompilation, so those are left alone here.
 */
static const char * const sunxi_codec_src_texts[] = { "aaudio", "i2s" };

static const struct soc_enum sunxi_codec_dac_src_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sunxi_codec_src_texts),
			    sunxi_codec_src_texts);

static void sunxi_codec_dac_src_apply(struct sunxi_codec *codec,
				      unsigned int src)
{
	if (src) {
		regmap_write(codec->regmap, SUNXI_CODEC_DAC_SRC, 1);
		regmap_update_bits(codec->regmap, SUNXI_CODEC_DAC_SRC,
				   SUNXI_CODEC_DAC_SRC_I2S,
				   SUNXI_CODEC_DAC_SRC_I2S);
		regmap_update_bits(codec->regmap, SUNXI_CODEC_OUT_CTRL2,
				   SUNXI_CODEC_OUT2_LO_EN,
				   SUNXI_CODEC_OUT2_LO_EN);
		regmap_update_bits(codec->regmap, SUNXI_CODEC_CTRL,
				   SUNXI_CODEC_CTRL_DAC_SRC_I2S,
				   SUNXI_CODEC_CTRL_DAC_SRC_I2S);
		/* The vendor also gates the I2S RX window (bits 1:0). */
		regmap_update_bits(codec->regmap_rx, SUNXI_CODEC_I2S_CTRL,
				   SUNXI_CODEC_I2S_RX_DAC_SRC,
				   SUNXI_CODEC_I2S_RX_DAC_SRC);
	} else {
		regmap_write(codec->regmap, SUNXI_CODEC_DAC_SRC, 0);
		regmap_update_bits(codec->regmap, SUNXI_CODEC_DAC_SRC,
				   SUNXI_CODEC_DAC_SRC_I2S, 0);
		regmap_update_bits(codec->regmap, SUNXI_CODEC_OUT_CTRL2,
				   SUNXI_CODEC_OUT2_LO_EN, 0);
		regmap_update_bits(codec->regmap, SUNXI_CODEC_CTRL,
				   SUNXI_CODEC_CTRL_DAC_SRC_I2S, 0);
		regmap_update_bits(codec->regmap_rx, SUNXI_CODEC_I2S_CTRL,
				   SUNXI_CODEC_I2S_RX_DAC_SRC, 0);
	}
}

static int sunxi_codec_dac_src_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(
		snd_soc_kcontrol_component(kcontrol));

	ucontrol->value.enumerated.item[0] = codec->dac_src;

	return 0;
}

static int sunxi_codec_dac_src_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(
		snd_soc_kcontrol_component(kcontrol));
	unsigned int val = ucontrol->value.enumerated.item[0];
	int changed;

	if (val >= ARRAY_SIZE(sunxi_codec_src_texts))
		return -EINVAL;

	mutex_lock(&codec->lock);
	changed = val != codec->dac_src;
	if (changed) {
		sunxi_codec_dac_src_apply(codec, val);
		codec->dac_src = val;
	}
	mutex_unlock(&codec->lock);

	return changed;
}

/*
 * ADC destination routing, as done by the vendor's sunxi_set_adcdst_mode():
 * capture data leaves towards the I2S TX window (region 1, 0x2031000) at
 * offset 0, with bits 8/2/0.  The vendor's getter only reports its cached
 * mode, like the one below.
 */
static const struct soc_enum sunxi_codec_adc_dst_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(sunxi_codec_src_texts),
			    sunxi_codec_src_texts);

static void sunxi_codec_adc_dst_apply(struct sunxi_codec *codec,
				      unsigned int dst)
{
	regmap_update_bits(codec->regmap_tx, SUNXI_CODEC_I2S_CTRL,
			   SUNXI_CODEC_I2S_TX_ADC_DST,
			   dst ? SUNXI_CODEC_I2S_TX_ADC_DST : 0);
}

static int sunxi_codec_adc_dst_get(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(
		snd_soc_kcontrol_component(kcontrol));

	ucontrol->value.enumerated.item[0] = codec->adc_dst;

	return 0;
}

static int sunxi_codec_adc_dst_put(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(
		snd_soc_kcontrol_component(kcontrol));
	unsigned int val = ucontrol->value.enumerated.item[0];
	int changed;

	if (val >= ARRAY_SIZE(sunxi_codec_src_texts))
		return -EINVAL;

	mutex_lock(&codec->lock);
	changed = val != codec->adc_dst;
	if (changed) {
		sunxi_codec_adc_dst_apply(codec, val);
		codec->adc_dst = val;
	}
	mutex_unlock(&codec->lock);

	return changed;
}

static const struct snd_kcontrol_new sunxi_codec_controls[] = {
	SOC_SINGLE("DACL Volume", SUNXI_CODEC_DAC_VOL, 8,
		   SUNXI_CODEC_VOL_MASK, 0),
	SOC_SINGLE("DACR Volume", SUNXI_CODEC_DAC_VOL, 0,
		   SUNXI_CODEC_VOL_MASK, 0),
	SOC_SINGLE("ADC1 Volume", SUNXI_CODEC_ADC_VOL, 8,
		   SUNXI_CODEC_VOL_MASK, 0),
	SOC_SINGLE("ADC2 Volume", SUNXI_CODEC_ADC_VOL, 0,
		   SUNXI_CODEC_VOL_MASK, 0),
	SOC_SINGLE("ADC1 ADC2 Swap", SUNXI_CODEC_ADC1_VOL, 8, 1, 0),
	SOC_SINGLE("DA2AD Loopback", SUNXI_CODEC_ADC2_RATE, 0, 0x3, 0),
	SOC_ENUM_EXT("DAC Src Select", sunxi_codec_dac_src_enum,
		     sunxi_codec_dac_src_get, sunxi_codec_dac_src_put),
	SOC_ENUM_EXT("ADC Dst Select", sunxi_codec_adc_dst_enum,
		     sunxi_codec_adc_dst_get, sunxi_codec_adc_dst_put),
};

/* DAI operations */

static int sunxi_codec_dai_startup(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(dai->component);

	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE &&
	    codec->sample_rate > 48000)
		return -EINVAL;

	return 0;
}

static int sunxi_codec_dai_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params,
				     struct snd_soc_dai *dai)
{
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(dai->component);
	bool playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	unsigned int rate = params_rate(params);
	unsigned int channels = params_channels(params);
	int code, ret;

	code = sunxi_codec_rate_code(rate);
	if (code < 0)
		return code;

	if (!playback && rate > 48000)
		return -EINVAL;

	mutex_lock(&codec->lock);

	codec->sample_rate = rate;

	/* DAC/ADC clock: the vendor reports 24.576 MHz for 48 kHz -> 512 fs. */
	ret = clk_set_parent(codec->clk_dac, codec->clk_pll);
	if (!ret)
		ret = clk_set_rate(codec->clk_dac, rate * 512);
	if (ret)
		goto out;

	ret = clk_set_parent(codec->clk_adc, codec->clk_pll);
	if (!ret)
		ret = clk_set_rate(codec->clk_adc, rate * 512);
	if (ret)
		goto out;

	/*
	 * Sample width, following the vendor's two format classes: S16_LE sets
	 * the 16-bit marker and clears the wide bit, S24_LE/S32_LE/S24_3LE do
	 * the opposite.  Register 0x04 is DACR volume and is not touched here.
	 */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		if (playback) {
			regmap_update_bits(codec->regmap, SUNXI_CODEC_DAC_CTRL,
					   SUNXI_CODEC_DAC_CTRL_W16_MASK,
					   SUNXI_CODEC_DAC_CTRL_W16);
			regmap_update_bits(codec->regmap, SUNXI_CODEC_DAC_CTRL,
					   SUNXI_CODEC_DAC_CTRL_WIDE, 0);
		} else {
			regmap_update_bits(codec->regmap, SUNXI_CODEC_ADC_CTRL,
					   SUNXI_CODEC_ADC_CTRL_W16_MASK,
					   SUNXI_CODEC_ADC_CTRL_W16);
			regmap_update_bits(codec->regmap, SUNXI_CODEC_ADC_CTRL,
					   SUNXI_CODEC_ADC_CTRL_WIDE, 0);
		}
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S32_LE:
	case SNDRV_PCM_FORMAT_S24_3LE:
		if (playback) {
			regmap_update_bits(codec->regmap, SUNXI_CODEC_DAC_CTRL,
					   SUNXI_CODEC_DAC_CTRL_W16_MASK, 0);
			regmap_update_bits(codec->regmap, SUNXI_CODEC_DAC_CTRL,
					   SUNXI_CODEC_DAC_CTRL_WIDE,
					   SUNXI_CODEC_DAC_CTRL_WIDE);
		} else {
			regmap_update_bits(codec->regmap, SUNXI_CODEC_ADC_CTRL,
					   SUNXI_CODEC_ADC_CTRL_W16_MASK, 0);
			regmap_update_bits(codec->regmap, SUNXI_CODEC_ADC_CTRL,
					   SUNXI_CODEC_ADC_CTRL_WIDE,
					   SUNXI_CODEC_ADC_CTRL_WIDE);
		}
		break;
	default:
		break;
	}

	/* Rate band: bits 31:29 of the DAC (0x10) or ADC (0x30) control. */
	regmap_update_bits(codec->regmap,
			   playback ? SUNXI_CODEC_DAC_CTRL : SUNXI_CODEC_ADC_CTRL,
			   SUNXI_CODEC_RATE_MASK, code << SUNXI_CODEC_RATE_SHIFT);

	if (playback)
		regmap_update_bits(codec->regmap, SUNXI_CODEC_DAC_CTRL,
				   SUNXI_CODEC_DAC_CTRL_MONO,
				   channels == 1 ? SUNXI_CODEC_DAC_CTRL_MONO : 0);

	ret = 0;
out:
	mutex_unlock(&codec->lock);
	return ret;
}

static int sunxi_codec_dai_hw_free(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(dai->component);

	mutex_lock(&codec->lock);
	codec->sample_rate = 0;
	mutex_unlock(&codec->lock);

	return 0;
}

static int sunxi_codec_dai_prepare(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(dai->component);
	bool playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;

	if (playback) {
		regmap_update_bits(codec->regmap, SUNXI_CODEC_DAC_CTRL,
				   SUNXI_CODEC_DAC_CTRL_EN,
				   SUNXI_CODEC_DAC_CTRL_EN);
		regmap_write(codec->regmap, SUNXI_CODEC_DAC_FIFO_CTRL,
			     SUNXI_CODEC_DAC_FIFO_VAL);
		regmap_write(codec->regmap, SUNXI_CODEC_DAC_FLUSH, 0);
	} else {
		regmap_update_bits(codec->regmap, SUNXI_CODEC_ADC_CTRL,
				   SUNXI_CODEC_ADC_CTRL_EN,
				   SUNXI_CODEC_ADC_CTRL_EN);
		regmap_write(codec->regmap, SUNXI_CODEC_ADC_FIFO_CTRL,
			     SUNXI_CODEC_ADC_FIFO_VAL);
		regmap_write(codec->regmap, SUNXI_CODEC_ADC_FLUSH, 0);
	}

	return 0;
}

static int sunxi_codec_dai_trigger(struct snd_pcm_substream *substream, int cmd,
				   struct snd_soc_dai *dai)
{
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(dai->component);
	bool playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (playback) {
			regmap_update_bits(codec->regmap, SUNXI_CODEC_DAC_CTRL,
					   SUNXI_CODEC_DAC_CTRL_UNMUTE,
					   SUNXI_CODEC_DAC_CTRL_UNMUTE);
			regmap_update_bits(codec->regmap, SUNXI_CODEC_CTRL,
					   SUNXI_CODEC_CTRL_DAC_ON,
					   SUNXI_CODEC_CTRL_DAC_ON);
			codec->playing = true;
		} else {
			regmap_update_bits(codec->regmap, SUNXI_CODEC_ADC_CTRL,
					   SUNXI_CODEC_ADC_CTRL_LINEINL |
					   SUNXI_CODEC_ADC_CTRL_LINEINR,
					   SUNXI_CODEC_ADC_CTRL_LINEINL |
					   SUNXI_CODEC_ADC_CTRL_LINEINR);
			codec->capturing = true;
		}
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (playback) {
			regmap_update_bits(codec->regmap, SUNXI_CODEC_DAC_CTRL,
					   SUNXI_CODEC_DAC_CTRL_UNMUTE, 0);
			codec->playing = false;
		} else {
			codec->capturing = false;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct snd_soc_dai_ops sunxi_codec_dai_ops = {
	.startup	= sunxi_codec_dai_startup,
	.hw_params	= sunxi_codec_dai_hw_params,
	.hw_free	= sunxi_codec_dai_hw_free,
	.prepare	= sunxi_codec_dai_prepare,
	.trigger	= sunxi_codec_dai_trigger,
};

#define SUNXI_CODEC_RATES	SNDRV_PCM_RATE_8000_192000
#define SUNXI_CODEC_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | \
				 SNDRV_PCM_FMTBIT_S24_3LE | \
				 SNDRV_PCM_FMTBIT_S24_LE | \
				 SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver sunxi_codec_dai = {
	.name	= "sunxi-codec",
	.playback = {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= SUNXI_CODEC_RATES,
		.formats	= SUNXI_CODEC_FORMATS,
	},
	.capture = {
		.stream_name	= "Capture",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= SUNXI_CODEC_RATES,
		.formats	= SUNXI_CODEC_FORMATS,
	},
	.ops = &sunxi_codec_dai_ops,
	.symmetric_rate = 1,
};

/*
 * PA pins: pa-pin-max entries of pa-pin-N (a GPIO specifier), each with
 * pa-pin-level-N and the two delays used by the vendor's pa_pin helper.
 */
static int sunxi_codec_pa_pins_init(struct sunxi_codec *codec)
{
	struct device_node *np = codec->dev->of_node;
	struct device *dev = codec->dev;
	char name[24];
	u32 count = 0, i;
	int gpio, ret;

	if (of_property_read_u32(np, "pa-pin-max", &count) || !count)
		return 0;

	codec->pa_pins = devm_kcalloc(dev, count, sizeof(*codec->pa_pins),
				      GFP_KERNEL);
	codec->pa_levels = devm_kcalloc(dev, count, sizeof(*codec->pa_levels),
					GFP_KERNEL);
	codec->pa_msleep = devm_kcalloc(dev, count, sizeof(*codec->pa_msleep),
					GFP_KERNEL);
	codec->pa_msleep1 = devm_kcalloc(dev, count,
					 sizeof(*codec->pa_msleep1), GFP_KERNEL);
	if (!codec->pa_pins || !codec->pa_levels || !codec->pa_msleep ||
	    !codec->pa_msleep1)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		unsigned int idx = codec->pa_pin_count;

		snprintf(name, sizeof(name), "pa-pin-%u", i);
		gpio = of_get_named_gpio(np, name, 0);
		if (gpio < 0)
			continue;

		ret = devm_gpio_request_one(dev, gpio, GPIOF_OUT_INIT_LOW, name);
		if (ret) {
			dev_warn(dev, "cannot request %s: %d\n", name, ret);
			continue;
		}

		codec->pa_pins[idx] = gpio;
		codec->pa_levels[idx] = 1;

		snprintf(name, sizeof(name), "pa-pin-level-%u", i);
		of_property_read_u32(np, name, &codec->pa_levels[idx]);
		snprintf(name, sizeof(name), "pa-pin-msleep-%u", i);
		of_property_read_u32(np, name, &codec->pa_msleep[idx]);
		snprintf(name, sizeof(name), "pa-pin-msleep1-%u", i);
		of_property_read_u32(np, name, &codec->pa_msleep1[idx]);

		codec->pa_pin_count++;
	}

	dev_dbg(dev, "%d PA pin(s)\n", codec->pa_pin_count);

	return 0;
}

static int sunxi_codec_component_probe(struct snd_soc_component *component)
{
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);

	/* DAC/ADC digital configuration, as the vendor's component probe does. */
	regmap_update_bits(codec->regmap, 0xf0, BIT(31), BIT(31));
	regmap_update_bits(codec->regmap, 0xf8, BIT(31), BIT(31));
	regmap_update_bits(codec->regmap, SUNXI_CODEC_ADC_CTRL,
			   SUNXI_CODEC_ADC_CTRL_W16, SUNXI_CODEC_ADC_CTRL_W16);
	regmap_update_bits(codec->regmap, SUNXI_CODEC_ADC_CTRL,
			   SUNXI_CODEC_ADC_CTRL_MODE_MASK,
			   SUNXI_CODEC_ADC_CTRL_MODE_I2S);
	/* The vendor enables the ADC through bit 17 of 0x30. */
	regmap_update_bits(codec->regmap, SUNXI_CODEC_ADC_CTRL,
			   SUNXI_CODEC_ADC_CTRL_ADC_EN,
			   SUNXI_CODEC_ADC_CTRL_ADC_EN);
	regmap_update_bits(codec->regmap, SUNXI_CODEC_CTRL,
			   SUNXI_CODEC_CTRL_MODE_MASK,
			   SUNXI_CODEC_CTRL_MODE_I2S);

	/* Volume/gain defaults from the device tree. */
	regmap_update_bits(codec->regmap, SUNXI_CODEC_DAC_VOL, 0xff00,
			   codec->dacl_vol << 8);
	regmap_update_bits(codec->regmap, SUNXI_CODEC_DAC_VOL, 0x00ff,
			   codec->dacr_vol);
	regmap_update_bits(codec->regmap, SUNXI_CODEC_ADC_VOL, 0xff00,
			   codec->adc1_vol << 8);
	regmap_update_bits(codec->regmap, SUNXI_CODEC_ADC_VOL, 0x00ff,
			   codec->adc2_vol);
	regmap_update_bits(codec->regmap, SUNXI_CODEC_OUT_CTRL,
			   SUNXI_CODEC_OUT_GAIN_MASK,
			   codec->lineout_gain & SUNXI_CODEC_OUT_GAIN_MASK);
	regmap_update_bits(codec->regmap, SUNXI_CODEC_OUT_CTRL,
			   SUNXI_CODEC_OUT_GAIN_MASK << SUNXI_CODEC_OUT_HP_GAIN_SHIFT,
			   (codec->hpout_gain & SUNXI_CODEC_OUT_GAIN_MASK) <<
			   SUNXI_CODEC_OUT_HP_GAIN_SHIFT);
	regmap_update_bits(codec->regmap, SUNXI_CODEC_OUT_CTRL2,
			   SUNXI_CODEC_OUT2_RAMP_MASK, codec->ramp_up << 4);

	sunxi_codec_dap_dacdrc(codec->regmap);
	sunxi_codec_dap_adcdrc(codec->regmap);
	/* HPF stages: the vendor writes 0xff into their enable registers. */
	regmap_write(codec->regmap, 0x100, 0xff);
	regmap_write(codec->regmap, 0x200, 0xff);

	/* DAC FIFO block. */
	regmap_update_bits(codec->regmap_tx, SUNXI_CODEC_FIFO_CTRL,
			   SUNXI_CODEC_FIFO_MODE_MASK,
			   SUNXI_CODEC_FIFO_MODE_I2S);
	regmap_update_bits(codec->regmap_tx, SUNXI_CODEC_FIFO_CTRL,
			   SUNXI_CODEC_FIFO_BCLK_MASTER |
			   SUNXI_CODEC_FIFO_LRCK_MASTER,
			   SUNXI_CODEC_FIFO_BCLK_MASTER |
			   SUNXI_CODEC_FIFO_LRCK_MASTER);
	regmap_update_bits(codec->regmap_tx, SUNXI_CODEC_FIFO_CLK_DIV,
			   SUNXI_CODEC_FIFO_DIV_MASK, SUNXI_CODEC_FIFO_DIV(4));
	regmap_update_bits(codec->regmap_tx, SUNXI_CODEC_FIFO_CLK_DIV,
			   SUNXI_CODEC_FIFO_DIV2_MASK, 0x80);
	regmap_update_bits(codec->regmap_tx, SUNXI_CODEC_FIFO_FMT,
			   SUNXI_CODEC_FIFO_TDM_MASK, 5);
	regmap_update_bits(codec->regmap_tx, SUNXI_CODEC_FIFO_FMT,
			   SUNXI_CODEC_FIFO_FRAME_MASK, 0x1f00);
	regmap_update_bits(codec->regmap_tx, SUNXI_CODEC_FIFO_FMT,
			   SUNXI_CODEC_FIFO_SAMPLE_MASK,
			   SUNXI_CODEC_FIFO_SAMPLE(20));

	/* ADC FIFO block. */
	regmap_update_bits(codec->regmap_rx, SUNXI_CODEC_FIFO_CTRL,
			   SUNXI_CODEC_FIFO_MODE_MASK,
			   SUNXI_CODEC_FIFO_MODE_I2S);
	regmap_update_bits(codec->regmap_rx, SUNXI_CODEC_FIFO_CLK_DIV,
			   SUNXI_CODEC_FIFO_DIV_MASK, SUNXI_CODEC_FIFO_DIV(4));
	regmap_update_bits(codec->regmap_rx, SUNXI_CODEC_FIFO_FMT,
			   SUNXI_CODEC_FIFO_TDM_MASK, 5);
	regmap_update_bits(codec->regmap_rx, SUNXI_CODEC_FIFO_FMT,
			   SUNXI_CODEC_FIFO_FRAME_MASK, 0x1f00);
	regmap_update_bits(codec->regmap_rx, SUNXI_CODEC_FIFO_FMT,
			   SUNXI_CODEC_FIFO_SAMPLE_MASK,
			   SUNXI_CODEC_FIFO_SAMPLE(20));

	/* Apply the device tree's dac-src/adc-dst selections (stock: "I2S"). */
	sunxi_codec_dac_src_apply(codec, codec->dac_src);
	sunxi_codec_adc_dst_apply(codec, codec->adc_dst);

	return 0;
}

static struct snd_soc_jack_pin sunxi_codec_jack_pins[] = {
	{ .pin = "HPOUT", .mask = SND_JACK_HEADPHONE },
};

static int sunxi_codec_set_jack(struct snd_soc_component *component,
				struct snd_soc_jack *jack, void *data)
{
	struct sunxi_codec *codec = snd_soc_component_get_drvdata(component);
	int ret;

	if (!jack) {
		if (codec->jack && codec->jack_gpio.gpio >= 0)
			snd_soc_jack_free_gpios(codec->jack, 1,
						&codec->jack_gpio);
		codec->jack = NULL;
		return 0;
	}

	ret = snd_soc_jack_add_pins(jack, ARRAY_SIZE(sunxi_codec_jack_pins),
				    sunxi_codec_jack_pins);
	if (ret) {
		dev_err(component->dev, "cannot add jack pins: %d\n", ret);
		return ret;
	}

	codec->jack = jack;

	if (codec->jack_gpio.gpio < 0)
		return 0;

	ret = snd_soc_jack_add_gpios(jack, 1, &codec->jack_gpio);
	if (ret) {
		dev_err(component->dev, "cannot add jack GPIO: %d\n", ret);
		codec->jack = NULL;
	}

	return ret;
}

static const struct snd_soc_component_driver sunxi_codec_component = {
	.name		= "sunxi-codec",
	.probe		= sunxi_codec_component_probe,
	.controls	= sunxi_codec_controls,
	.num_controls	= ARRAY_SIZE(sunxi_codec_controls),
	.dapm_widgets	= sunxi_codec_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(sunxi_codec_dapm_widgets),
	.dapm_routes	= sunxi_codec_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(sunxi_codec_dapm_routes),
	.set_jack	= sunxi_codec_set_jack,
};

static const struct regmap_config sunxi_codec_regmap_config = {
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
	.max_register	= SUNXI_CODEC_MAX_REG,
};

static const struct regmap_config sunxi_codec_fifo_regmap_config = {
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
	.max_register	= 0x7c,
};

static int sunxi_codec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sunxi_codec *codec;
	void __iomem *base;
	const char *str;
	unsigned int val;
	int ret;

	codec = devm_kzalloc(dev, sizeof(*codec), GFP_KERNEL);
	if (!codec)
		return -ENOMEM;

	codec->dev = dev;
	mutex_init(&codec->lock);
	platform_set_drvdata(pdev, codec);

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	codec->regmap = devm_regmap_init_mmio(dev, base,
					      &sunxi_codec_regmap_config);
	if (IS_ERR(codec->regmap))
		return dev_err_probe(dev, PTR_ERR(codec->regmap),
				     "cannot map control registers\n");

	base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(base))
		return dev_err_probe(dev, PTR_ERR(base),
				     "cannot map DAC FIFO registers\n");

	codec->regmap_tx = devm_regmap_init_mmio(dev, base,
						 &sunxi_codec_fifo_regmap_config);
	if (IS_ERR(codec->regmap_tx))
		return dev_err_probe(dev, PTR_ERR(codec->regmap_tx),
				     "cannot map DAC FIFO regmap\n");

	base = devm_platform_ioremap_resource(pdev, 2);
	if (IS_ERR(base))
		return dev_err_probe(dev, PTR_ERR(base),
				     "cannot map ADC FIFO registers\n");

	codec->regmap_rx = devm_regmap_init_mmio(dev, base,
						 &sunxi_codec_fifo_regmap_config);
	if (IS_ERR(codec->regmap_rx))
		return dev_err_probe(dev, PTR_ERR(codec->regmap_rx),
				     "cannot map ADC FIFO regmap\n");

	codec->clk_bus = devm_clk_get(dev, "clk_bus_adda");
	if (IS_ERR(codec->clk_bus))
		return dev_err_probe(dev, PTR_ERR(codec->clk_bus),
				     "cannot get bus clock\n");

	codec->clk_pll = devm_clk_get_optional(dev, "clk_pll_audio_4x");
	if (IS_ERR(codec->clk_pll))
		return dev_err_probe(dev, PTR_ERR(codec->clk_pll),
				     "cannot get audio PLL\n");

	codec->clk_dac = devm_clk_get(dev, "clk_audio_dac");
	if (IS_ERR(codec->clk_dac))
		return dev_err_probe(dev, PTR_ERR(codec->clk_dac),
				     "cannot get DAC clock\n");

	codec->clk_adc = devm_clk_get(dev, "clk_audio_adc");
	if (IS_ERR(codec->clk_adc))
		return dev_err_probe(dev, PTR_ERR(codec->clk_adc),
				     "cannot get ADC clock\n");

	codec->rst = devm_reset_control_get_shared(dev, NULL);
	if (IS_ERR(codec->rst))
		return dev_err_probe(dev, PTR_ERR(codec->rst),
				     "cannot get reset\n");

	/* Defaults; the stock DT supplies dacl-vol/dacr-vol = 0xa0, adc1/adc2 = 0xa0. */
	codec->dacl_vol = 0xa0;
	codec->dacr_vol = 0xa0;
	codec->adc1_vol = 0xa0;
	codec->adc2_vol = 0xa0;
	codec->lineout_gain = 0x7;
	codec->hpout_gain = 0x7;
	codec->ramp_up = 0x3;
	codec->ramp_down = 0x3;
	codec->dac_src = 1;	/* 1 = I2S, which is what the stock DT selects */
	codec->adc_dst = 1;

	if (!of_property_read_u32(dev->of_node, "dacl-vol", &val))
		codec->dacl_vol = val;
	if (!of_property_read_u32(dev->of_node, "dacr-vol", &val))
		codec->dacr_vol = val;
	if (!of_property_read_u32(dev->of_node, "adc1-vol", &val))
		codec->adc1_vol = val;
	if (!of_property_read_u32(dev->of_node, "adc2-vol", &val))
		codec->adc2_vol = val;
	if (!of_property_read_u32(dev->of_node, "lineout-gain", &val))
		codec->lineout_gain = val;
	if (!of_property_read_u32(dev->of_node, "hpout-gain", &val))
		codec->hpout_gain = val;
	if (!of_property_read_u32(dev->of_node, "ramp-time-up", &val))
		codec->ramp_up = val;
	if (!of_property_read_u32(dev->of_node, "ramp-time-down", &val))
		codec->ramp_down = val;
	if (!of_property_read_string(dev->of_node, "dac-src", &str))
		codec->dac_src = !strcasecmp(str, "i2s");
	if (!of_property_read_string(dev->of_node, "adc-dst", &str))
		codec->adc_dst = !strcasecmp(str, "i2s");

	codec->jack_det_level = 1;
	if (!of_property_read_u32(dev->of_node, "jack-det-level", &val))
		codec->jack_det_level = val;

	/*
	 * The jack GPIO is requested by snd_soc_jack_add_gpios() when the
	 * machine driver hands us a jack, so only record it here.
	 */
	codec->jack_gpio.gpio = of_get_named_gpio(dev->of_node,
						  "hp-det-gpio", 0);
	codec->jack_gpio.name = "Headphone";
	codec->jack_gpio.report = SND_JACK_HEADPHONE;
	codec->jack_gpio.invert = !codec->jack_det_level;
	codec->jack_gpio.debounce_time = 200;

	ret = sunxi_codec_pa_pins_init(codec);
	if (ret)
		return ret;

	ret = clk_prepare_enable(codec->clk_bus);
	if (ret)
		return ret;

	ret = reset_control_deassert(codec->rst);
	if (ret)
		goto err_bus;

	ret = clk_prepare_enable(codec->clk_dac);
	if (ret)
		goto err_rst;

	ret = clk_prepare_enable(codec->clk_adc);
	if (ret)
		goto err_dac;

	ret = devm_snd_soc_register_component(dev, &sunxi_codec_component,
					      &sunxi_codec_dai, 1);
	if (ret) {
		dev_err(dev, "cannot register codec component: %d\n", ret);
		goto err_adc;
	}

	return 0;

err_adc:
	clk_disable_unprepare(codec->clk_adc);
err_dac:
	clk_disable_unprepare(codec->clk_dac);
err_rst:
	reset_control_assert(codec->rst);
err_bus:
	clk_disable_unprepare(codec->clk_bus);
	return ret;
}

static int sunxi_codec_remove(struct platform_device *pdev)
{
	struct sunxi_codec *codec = platform_get_drvdata(pdev);

	clk_disable_unprepare(codec->clk_adc);
	clk_disable_unprepare(codec->clk_dac);
	reset_control_assert(codec->rst);
	clk_disable_unprepare(codec->clk_bus);

	return 0;
}

static const struct of_device_id sunxi_codec_of_match[] = {
	{ .compatible = "allwinner,sunxi-snd-codec" },
	{ }
};
MODULE_DEVICE_TABLE(of, sunxi_codec_of_match);

static struct platform_driver sunxi_codec_driver = {
	.probe	= sunxi_codec_probe,
	.remove	= sunxi_codec_remove,
	.driver	= {
		.name		= "sunxi-snd-codec",
		.of_match_table	= sunxi_codec_of_match,
	},
};
module_platform_driver(sunxi_codec_driver);

MODULE_AUTHOR("H726 open kernel effort");
MODULE_DESCRIPTION("Allwinner sun50iw15 (H726) internal audio codec");
MODULE_LICENSE("GPL");
