// SPDX-License-Identifier: GPL-2.0
/*
 * Allwinner sun50iw15 (H726 / HY300) I2S/TDM controller
 *
 * Open replacement for the vendor snd_soc_sunxi_i2s module.
 *
 * The register layout below was taken from the decompiled vendor module
 * (dec_mod/snd_soc_sunxi_i2s.c, functions sunxi_i2s_set_daifmt_fmt(),
 * sunxi_i2s_set_ch_en(), sunxi_i2s_dai_set_fmt(), sunxi_i2s_dai_set_tdm_slot()
 * and sunxi_i2s_dai_hw_params()).  Every offset/mask here appears literally in
 * that decompilation as a regmap_update_bits_base() argument; the symbolic
 * names and the ASoC plumbing around them are ours.
 *
 * Like the vendor driver this block is fed by a separate DMA/FIFO engine, so
 * PCM is served by the shared sun50iw15 engine (sun50iw15-pcm.c) using the
 * "tx"/"rx" channels described in the device tree, with the FIFO data ports
 * recovered from the vendor's DMA params.
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/sun50iw15-pcm.h>

/* Control (0x00) */
#define SUN50IW15_I2S_CTRL			0x00
#define SUN50IW15_I2S_CTRL_MODE_MASK		GENMASK(5, 4)
#define SUN50IW15_I2S_CTRL_MODE_I2S		(0x1 << 4)
#define SUN50IW15_I2S_CTRL_MODE_LEFT_J		(0x2 << 4)
#define SUN50IW15_I2S_CTRL_MODE_PCM		(0x0 << 4)
#define SUN50IW15_I2S_CTRL_LRCK_MASTER		BIT(17)
#define SUN50IW15_I2S_CTRL_BCLK_MASTER		BIT(18)
#define SUN50IW15_I2S_CTRL_GLOBAL_EN		BIT(0)

/* Format (0x04) */
#define SUN50IW15_I2S_FMT			0x04
#define SUN50IW15_I2S_FMT_DSP_MODE		BIT(30)
#define SUN50IW15_I2S_FMT_LRCK_POL		BIT(19)
#define SUN50IW15_I2S_FMT_BCLK_POL		BIT(7)
#define SUN50IW15_I2S_FMT_SAMPLE_RES_MASK	GENMASK(6, 4)
#define SUN50IW15_I2S_FMT_SAMPLE_RES(w)		(((w) / 4) << 4)
#define SUN50IW15_I2S_FMT_FRAME_WIDTH_MASK	GENMASK(17, 8)
#define SUN50IW15_I2S_FMT_FRAME_WIDTH(w)	(((w) - 1) << 8)
#define SUN50IW15_I2S_FMT_TDM_MODE_MASK		GENMASK(2, 0)

/* Mute / loopback (0x08) */
#define SUN50IW15_I2S_MUTE			0x08
#define SUN50IW15_I2S_MUTE_TX			BIT(7)
#define SUN50IW15_I2S_MUTE_LOOPBACK		BIT(6)

/* FIFO / DMA interrupt control (0x14) */
#define SUN50IW15_I2S_DMA_INT			0x14
#define SUN50IW15_I2S_DMA_INT_TX_DRQ		BIT(2)
#define SUN50IW15_I2S_DMA_INT_RX_DRQ		BIT(0)
#define SUN50IW15_I2S_DMA_INT_TX_FLUSH		BIT(24)
#define SUN50IW15_I2S_DMA_INT_RX_FLUSH		BIT(25)

/*
 * Channel configuration (0x30): TX slot count in [3:0], RX slot count in
 * [7:4].  Vendor sunxi_i2s_set_ch_en() writes channels - 1 into each half.
 */
#define SUN50IW15_I2S_CHAN_CFG			0x30
#define SUN50IW15_I2S_CHAN_CFG_TX_MASK		GENMASK(3, 0)
#define SUN50IW15_I2S_CHAN_CFG_TX(ch)		((ch) - 1)
#define SUN50IW15_I2S_CHAN_CFG_RX_MASK		GENMASK(7, 4)
#define SUN50IW15_I2S_CHAN_CFG_RX(ch)		(((ch) - 1) << 4)

/*
 * Per-hub-slot TX channel select: 0x34, 0x38, 0x3c, 0x40.  From the
 * disassembly of the vendor sun8iw17_i2s_set_ch_en() in the stock
 * snd_soc_sunxi_i2s.ko:
 *
 *   write 0x34, mask 0xf0000, value ch << 16       -> [19:16] channel count
 *   write 0x34, mask 0xffff,  value table[ch - 1]  -> [15:0] enable mask
 *
 * where table is .L__const.sun8iw17_i2s_set_ch_en.channels_en_slot[16] in
 * .rodata and holds the identity mask 1, 3, 7, ... 0xffff, i.e. (1 << ch) - 1.
 * The earlier (ch - 1) << 16 was off by one: [19:16] carries the 1-based
 * count, so every hub slot advertised one channel fewer than it really has.
 * These are *not* the map registers.
 */
#define SUN50IW15_I2S_TX_CHAN_SEL(slot)		(0x34 + (slot) * 4)
#define SUN50IW15_I2S_TX_CHAN_SEL_OFF_MASK	GENMASK(21, 20)
#define SUN50IW15_I2S_TX_CHAN_SEL_MASK		GENMASK(19, 16)
#define SUN50IW15_I2S_TX_CHAN_SEL_VAL(ch)	((ch) << 16)
#define SUN50IW15_I2S_TX_CHAN_EN_MASK		GENMASK(15, 0)
/* channels_en_slot[ch - 1]: the vendor table is the identity mask. */
#define SUN50IW15_I2S_TX_CHAN_EN(ch)		((1U << (ch)) - 1)

/*
 * Channel maps, recovered from sun8iw17_i2s_set_ch_map() and the defaults in
 * snd_sunxi_dts_params_init():
 *   TX: eight 4-bit slot entries per register, the low slots at 0x48 + pin*8
 *       and the high slots at 0x44 + pin*8, so pin 0 uses 0x48/0x44 and pin 3
 *       ends at 0x60/0x5c.  Per-slot values default to slot i <- channel i.
 *   RX: one byte per slot in 0x74 (slots 0..3), 0x70, 0x6c and 0x68 (12..15);
 *       the low nibble is the channel, the high nibble comes from
 *       rxfifo-pinmap, which the stock DT leaves out (zero).
 * Both defaults can be overridden per board with tx-pin<N>-chmap and
 * rxfifo-chmap, each holding one entry per slot.
 */
#define SUN50IW15_I2S_MAP_SLOTS			16	/* slots per direction */
#define SUN50IW15_I2S_MAP_PINS			4
#define SUN50IW15_I2S_TX_MAP_LO(pin)		(0x48 + (pin) * 8)
#define SUN50IW15_I2S_TX_MAP_HI(pin)		(0x44 + (pin) * 8)
#define SUN50IW15_I2S_RX_MAP_0_3		0x74
#define SUN50IW15_I2S_RX_MAP_4_7		0x70
#define SUN50IW15_I2S_RX_MAP_8_11		0x6c
#define SUN50IW15_I2S_RX_MAP_12_15		0x68

/* RX channel select (0x64): same field layout as the TX selects. */
#define SUN50IW15_I2S_RX_CHAN_SEL		0x64
#define SUN50IW15_I2S_RX_CHAN_SEL_MASK		GENMASK(19, 16)
#define SUN50IW15_I2S_RX_CHAN_SEL_VAL(ch)	(((ch) - 1) << 16)

/*
 * FIFO data ports.  The vendor's sunxi_i2s_dev_probe() puts the playback DMA
 * address at iomem + 0x20 and the capture one at iomem + 0x10.
 */
#define SUN50IW15_I2S_FIFO_TX_DEFAULT		0x20
#define SUN50IW15_I2S_FIFO_RX_DEFAULT		0x10

/* Maximum register offset described in the vendor module (reg size 0x7c) */
#define SUN50IW15_I2S_MAX_REG			0x7c

#define SUN50IW15_I2S_MAX_CHANNELS		16

struct sun50iw15_i2s {
	struct device *dev;
	struct regmap *regmap;
	struct clk *clk_bus;
	struct clk *clk_mod;
	struct clk *clk_pll_audio;
	struct clk *clk_pll_peri0;
	struct reset_control *rst;

	bool tx_hub;
	unsigned int tx_pin_mask;
	u8 tx_chmap[SUN50IW15_I2S_MAP_PINS][SUN50IW15_I2S_MAP_SLOTS];
	u8 rx_chmap[SUN50IW15_I2S_MAP_SLOTS];
	u8 rx_pinmap[SUN50IW15_I2S_MAP_SLOTS];
	bool rx_sync;
	unsigned int tdm_slots;
	unsigned int slot_width;
	unsigned int mclk_fs;
	unsigned int sysclk;	/* set by the machine driver, if any */

	struct snd_dmaengine_dai_dma_data playback;
	struct snd_dmaengine_dai_dma_data capture;

	unsigned int playback_on;
	unsigned int capture_on;
	struct mutex lock;	/* serialises rate/channel programming */
};

static const struct regmap_config sun50iw15_i2s_regmap_config = {
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
	.max_register	= SUN50IW15_I2S_MAX_REG,
};

/*
 * Clocking.  snd_i2s_clk_rate() in the vendor module does exactly this:
 * parent the module clock to the audio PLL, then set its rate.  We mirror it
 * but let the clock framework reject unsupported rates properly instead of
 * silently clamping.
 */
static int sun50iw15_i2s_set_rate(struct sun50iw15_i2s *i2s, unsigned int rate)
{
	unsigned int mclk = i2s->sysclk ? i2s->sysclk : rate * i2s->mclk_fs;
	int ret;

	if (i2s->clk_pll_audio) {
		ret = clk_set_parent(i2s->clk_mod, i2s->clk_pll_audio);
		if (ret)
			return ret;
	}

	ret = clk_set_rate(i2s->clk_mod, mclk);
	if (ret)
		dev_err(i2s->dev, "cannot set mclk %u Hz: %d\n", mclk, ret);

	return ret;
}

/*
 * Pack one pin's TX map: eight 4-bit channel entries per register, low slots
 * in the 0x48 register and high slots in the 0x44 one, as the vendor does.
 */
static void sun50iw15_i2s_write_tx_map(struct sun50iw15_i2s *i2s,
				       unsigned int pin)
{
	const u8 *map = i2s->tx_chmap[pin];
	u32 lo = 0, hi = 0;
	unsigned int i;

	for (i = 0; i < 8; i++) {
		lo |= (u32)(map[i] & 0xf) << (i * 4);
		hi |= (u32)(map[8 + i] & 0xf) << (i * 4);
	}

	regmap_write(i2s->regmap, SUN50IW15_I2S_TX_MAP_LO(pin), lo);
	regmap_write(i2s->regmap, SUN50IW15_I2S_TX_MAP_HI(pin), hi);
}

/*
 * Four slots per RX map register, one byte each: low nibble from rxfifo-chmap,
 * high nibble from rxfifo-pinmap, as sun8iw17_i2s_set_ch_map() packs them.
 */
static u32 sun50iw15_i2s_rx_map_word(const struct sun50iw15_i2s *i2s,
				     unsigned int first)
{
	u32 word = 0;
	unsigned int i;

	for (i = 0; i < 4; i++) {
		u32 slot = (u32)(i2s->rx_chmap[first + i] & 0xf) |
			   (u32)(i2s->rx_pinmap[first + i] & 0xf) << 4;

		word |= slot << (i * 8);
	}

	return word;
}

/*
 * Fill a slot map from an optional DT property.  The channel maps fall back to
 * the identity one (slot i <- channel i) and rxfifo-pinmap to zero, as the
 * vendor does; entries beyond the slot count are replaced with 0 and warned
 * about.
 */
static void sun50iw15_i2s_read_chmap(struct device *dev, const char *prop,
				     u8 *map, bool identity)
{
	u32 vals[SUN50IW15_I2S_MAP_SLOTS];
	unsigned int i;
	int n;

	for (i = 0; i < SUN50IW15_I2S_MAP_SLOTS; i++)
		map[i] = identity ? i : 0;

	if (!device_property_present(dev, prop))
		return;

	n = device_property_read_u32_array(dev, prop, vals,
					   SUN50IW15_I2S_MAP_SLOTS);
	if (n <= 0) {
		dev_warn(dev, "%s unreadable (%d), keeping identity map\n",
			 prop, n);
		return;
	}

	for (i = 0; i < (unsigned int)n; i++) {
		if (vals[i] >= SUN50IW15_I2S_MAP_SLOTS) {
			dev_warn(dev, "%s[%u] = %u out of range, using 0\n",
				 prop, i, vals[i]);
			map[i] = 0;
		} else {
			map[i] = (u8)vals[i];
		}
	}
}

/*
 * Channel count and map programming, following sunxi_i2s_set_ch_en() and
 * sun8iw17_i2s_set_ch_map():
 *   playback: 0x30[3:0] = channels - 1, the TX map of every enabled pin and
 *             with TX-hub enabled a select (slot offset 0, channels - 1,
 *             enable mask) in that pin's 0x34/0x38/0x3c/0x40 register
 *   capture:  0x30[7:4] = channels - 1, the byte-packed RX map in
 *             0x74/0x70/0x6c/0x68 and 0x64[19:16] = channels - 1
 */
static void sun50iw15_i2s_set_channels(struct sun50iw15_i2s *i2s,
				       bool playback, unsigned int channels)
{
	unsigned int i;

	/* The vendor clamps to the 16 channels the enable table covers. */
	channels = clamp(channels, 1U, 16U);

	if (playback) {
		regmap_update_bits(i2s->regmap, SUN50IW15_I2S_CHAN_CFG,
				   SUN50IW15_I2S_CHAN_CFG_TX_MASK,
				   SUN50IW15_I2S_CHAN_CFG_TX(channels));
		/* The vendor programs the maps on every hw_params. */
		sun50iw15_i2s_write_tx_map(i2s, 0);
		if (!i2s->tx_hub)
			return;

		for (i = 0; i < SUN50IW15_I2S_MAP_PINS; i++) {
			if (!(i2s->tx_pin_mask & (1U << i)))
				continue;
			if (i)
				sun50iw15_i2s_write_tx_map(i2s, i);

			regmap_update_bits(i2s->regmap,
					   SUN50IW15_I2S_TX_CHAN_SEL(i),
					   SUN50IW15_I2S_TX_CHAN_SEL_OFF_MASK |
					   SUN50IW15_I2S_TX_CHAN_SEL_MASK |
					   SUN50IW15_I2S_TX_CHAN_EN_MASK,
					   SUN50IW15_I2S_TX_CHAN_SEL_VAL(channels) |
					   SUN50IW15_I2S_TX_CHAN_EN(channels));
		}
	} else {
		regmap_update_bits(i2s->regmap, SUN50IW15_I2S_CHAN_CFG,
				   SUN50IW15_I2S_CHAN_CFG_RX_MASK,
				   SUN50IW15_I2S_CHAN_CFG_RX(channels));
		regmap_write(i2s->regmap, SUN50IW15_I2S_RX_MAP_0_3,
			     sun50iw15_i2s_rx_map_word(i2s, 0));
		regmap_write(i2s->regmap, SUN50IW15_I2S_RX_MAP_4_7,
			     sun50iw15_i2s_rx_map_word(i2s, 4));
		regmap_write(i2s->regmap, SUN50IW15_I2S_RX_MAP_8_11,
			     sun50iw15_i2s_rx_map_word(i2s, 8));
		regmap_write(i2s->regmap, SUN50IW15_I2S_RX_MAP_12_15,
			     sun50iw15_i2s_rx_map_word(i2s, 12));
		regmap_update_bits(i2s->regmap, SUN50IW15_I2S_RX_CHAN_SEL,
				   SUN50IW15_I2S_RX_CHAN_SEL_MASK,
				   SUN50IW15_I2S_RX_CHAN_SEL_VAL(channels));
	}
}

static int sun50iw15_i2s_dai_startup(struct snd_pcm_substream *substream,
				     struct snd_soc_dai *dai)
{
	return 0;
}

static void sun50iw15_i2s_dai_shutdown(struct snd_pcm_substream *substream,
				       struct snd_soc_dai *dai)
{
}

static int sun50iw15_i2s_dai_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params,
				       struct snd_soc_dai *dai)
{
	struct sun50iw15_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	bool playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	unsigned int width = params_physical_width(params);
	unsigned int channels = params_channels(params);
	unsigned int rate = params_rate(params);
	int ret;

	if (channels < 1 || channels > SUN50IW15_I2S_MAX_CHANNELS)
		return -EINVAL;

	mutex_lock(&i2s->lock);

	ret = sun50iw15_i2s_set_rate(i2s, rate);
	if (ret)
		goto out;

	regmap_update_bits(i2s->regmap, SUN50IW15_I2S_FMT,
			   SUN50IW15_I2S_FMT_SAMPLE_RES_MASK,
			   SUN50IW15_I2S_FMT_SAMPLE_RES(width));

	if (i2s->tdm_slots)
		regmap_update_bits(i2s->regmap, SUN50IW15_I2S_FMT,
				   SUN50IW15_I2S_FMT_FRAME_WIDTH_MASK,
				   SUN50IW15_I2S_FMT_FRAME_WIDTH(i2s->tdm_slots *
								 i2s->slot_width));

	sun50iw15_i2s_set_channels(i2s, playback, channels);

	/* Enable the request line for this direction only. */
	regmap_update_bits(i2s->regmap, SUN50IW15_I2S_DMA_INT,
			   playback ? SUN50IW15_I2S_DMA_INT_TX_DRQ :
				      SUN50IW15_I2S_DMA_INT_RX_DRQ,
			   playback ? SUN50IW15_I2S_DMA_INT_TX_DRQ :
				      SUN50IW15_I2S_DMA_INT_RX_DRQ);

	regmap_update_bits(i2s->regmap, SUN50IW15_I2S_CTRL,
			   SUN50IW15_I2S_CTRL_GLOBAL_EN,
			   SUN50IW15_I2S_CTRL_GLOBAL_EN);

	if (playback)
		i2s->playback_on = channels;
	else
		i2s->capture_on = channels;

out:
	mutex_unlock(&i2s->lock);
	return ret;
}

static int sun50iw15_i2s_dai_hw_free(struct snd_pcm_substream *substream,
				     struct snd_soc_dai *dai)
{
	struct sun50iw15_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	bool playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;

	mutex_lock(&i2s->lock);

	if (playback)
		i2s->playback_on = 0;
	else
		i2s->capture_on = 0;

	regmap_update_bits(i2s->regmap, SUN50IW15_I2S_DMA_INT,
			   playback ? SUN50IW15_I2S_DMA_INT_TX_DRQ :
				      SUN50IW15_I2S_DMA_INT_RX_DRQ, 0);

	/* Only power the module clock down when nothing else is running. */
	if (!i2s->playback_on && !i2s->capture_on) {
		regmap_update_bits(i2s->regmap, SUN50IW15_I2S_CTRL,
				   SUN50IW15_I2S_CTRL_GLOBAL_EN, 0);
		clk_disable_unprepare(i2s->clk_mod);
		i2s->sysclk = 0;
	}

	mutex_unlock(&i2s->lock);
	return 0;
}

static int sun50iw15_i2s_dai_prepare(struct snd_pcm_substream *substream,
				     struct snd_soc_dai *dai)
{
	struct sun50iw15_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	bool playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;

	/* Flush the FIFO for this direction before the DMA starts feeding it. */
	regmap_update_bits(i2s->regmap, SUN50IW15_I2S_DMA_INT,
			   playback ? SUN50IW15_I2S_DMA_INT_TX_FLUSH :
				      SUN50IW15_I2S_DMA_INT_RX_FLUSH,
			   playback ? SUN50IW15_I2S_DMA_INT_TX_FLUSH :
				      SUN50IW15_I2S_DMA_INT_RX_FLUSH);

	return 0;
}

static int sun50iw15_i2s_dai_trigger(struct snd_pcm_substream *substream,
				     int cmd, struct snd_soc_dai *dai)
{
	struct sun50iw15_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	bool playback = substream->stream == SNDRV_PCM_STREAM_PLAYBACK;
	unsigned int drq = playback ? SUN50IW15_I2S_DMA_INT_TX_DRQ :
				      SUN50IW15_I2S_DMA_INT_RX_DRQ;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		regmap_update_bits(i2s->regmap, SUN50IW15_I2S_DMA_INT, drq, drq);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		regmap_update_bits(i2s->regmap, SUN50IW15_I2S_DMA_INT, drq, 0);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * Format.  sunxi_i2s_set_daifmt_fmt() selects the protocol through
 * CTRL[5:4] (0x10 = I2S, 0x20 = left justified, 0x00 = right justified/PCM)
 * and, for DSP modes, FMT[30]; master/slave is CTRL[18] (bit clock) and
 * CTRL[17] (frame clock); polarity is FMT[19] (LRCK) and FMT[7] (BCLK).
 */
static int sun50iw15_i2s_dai_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct sun50iw15_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	unsigned int ctrl = 0, fmt_reg = 0;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		ctrl |= SUN50IW15_I2S_CTRL_MODE_I2S;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		ctrl |= SUN50IW15_I2S_CTRL_MODE_LEFT_J;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		ctrl |= SUN50IW15_I2S_CTRL_MODE_PCM;
		break;
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_DSP_B:
		ctrl |= SUN50IW15_I2S_CTRL_MODE_PCM;
		fmt_reg |= SUN50IW15_I2S_FMT_DSP_MODE;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:	/* codec is slave: we drive both clocks */
		ctrl |= SUN50IW15_I2S_CTRL_BCLK_MASTER |
			SUN50IW15_I2S_CTRL_LRCK_MASTER;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:	/* codec drives both clocks */
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		ctrl |= SUN50IW15_I2S_CTRL_BCLK_MASTER;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		ctrl |= SUN50IW15_I2S_CTRL_LRCK_MASTER;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_NB_IF:
		fmt_reg |= SUN50IW15_I2S_FMT_LRCK_POL;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		fmt_reg |= SUN50IW15_I2S_FMT_BCLK_POL;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		fmt_reg |= SUN50IW15_I2S_FMT_BCLK_POL |
			   SUN50IW15_I2S_FMT_LRCK_POL;
		break;
	default:
		return -EINVAL;
	}

	regmap_update_bits(i2s->regmap, SUN50IW15_I2S_CTRL,
			   SUN50IW15_I2S_CTRL_MODE_MASK |
			   SUN50IW15_I2S_CTRL_BCLK_MASTER |
			   SUN50IW15_I2S_CTRL_LRCK_MASTER, ctrl);
	regmap_update_bits(i2s->regmap, SUN50IW15_I2S_FMT,
			   SUN50IW15_I2S_FMT_DSP_MODE |
			   SUN50IW15_I2S_FMT_LRCK_POL |
			   SUN50IW15_I2S_FMT_BCLK_POL, fmt_reg);

	return 0;
}

static int sun50iw15_i2s_dai_set_sysclk(struct snd_soc_dai *dai, int clk_id,
					unsigned int freq, int dir)
{
	struct sun50iw15_i2s *i2s = snd_soc_dai_get_drvdata(dai);

	if (!freq)
		return 0;

	i2s->sysclk = freq;

	return clk_set_rate(i2s->clk_mod, freq);
}

/*
 * TDM.  sunxi_i2s_dai_set_tdm_slot() programs FMT[2:0] with the slot-count
 * encoding (slots - 8) >> 2 | slots << 30, then FMT[17:8] with
 * slots * slot_width - 1.  Only slot counts that encode to 0..6 are accepted.
 */
static int sun50iw15_i2s_dai_set_tdm_slot(struct snd_soc_dai *dai,
					  unsigned int tx_mask,
					  unsigned int rx_mask,
					  int slots, int slot_width)
{
	struct sun50iw15_i2s *i2s = snd_soc_dai_get_drvdata(dai);
	unsigned int enc;

	if (slots < 0 || slot_width <= 0)
		return -EINVAL;

	enc = (slots << 30) | ((slots - 8) >> 2);
	if (enc >= 7)
		return -EINVAL;

	regmap_update_bits(i2s->regmap, SUN50IW15_I2S_FMT,
			   SUN50IW15_I2S_FMT_TDM_MODE_MASK, enc + 1);
	regmap_update_bits(i2s->regmap, SUN50IW15_I2S_FMT,
			   SUN50IW15_I2S_FMT_FRAME_WIDTH_MASK,
			   SUN50IW15_I2S_FMT_FRAME_WIDTH(slots * slot_width));

	i2s->tdm_slots = slots;
	i2s->slot_width = slot_width;

	return 0;
}

static const struct snd_soc_dai_ops sun50iw15_i2s_dai_ops = {
	.startup	= sun50iw15_i2s_dai_startup,
	.shutdown	= sun50iw15_i2s_dai_shutdown,
	.hw_params	= sun50iw15_i2s_dai_hw_params,
	.hw_free	= sun50iw15_i2s_dai_hw_free,
	.prepare	= sun50iw15_i2s_dai_prepare,
	.trigger	= sun50iw15_i2s_dai_trigger,
	.set_fmt	= sun50iw15_i2s_dai_set_fmt,
	.set_sysclk	= sun50iw15_i2s_dai_set_sysclk,
	.set_tdm_slot	= sun50iw15_i2s_dai_set_tdm_slot,
};

#define SUN50IW15_I2S_RATES	SNDRV_PCM_RATE_8000_192000
#define SUN50IW15_I2S_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | \
				 SNDRV_PCM_FMTBIT_S20_3LE | \
				 SNDRV_PCM_FMTBIT_S24_LE | \
				 SNDRV_PCM_FMTBIT_S32_LE)

static struct snd_soc_dai_driver sun50iw15_i2s_dai = {
	.name		= "sun50iw15-i2s",
	.playback	= {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= SUN50IW15_I2S_MAX_CHANNELS,
		.rates		= SUN50IW15_I2S_RATES,
		.formats	= SUN50IW15_I2S_FORMATS,
	},
	.capture	= {
		.stream_name	= "Capture",
		.channels_min	= 1,
		.channels_max	= SUN50IW15_I2S_MAX_CHANNELS,
		.rates		= SUN50IW15_I2S_RATES,
		.formats	= SUN50IW15_I2S_FORMATS,
	},
	.ops		= &sun50iw15_i2s_dai_ops,
	.symmetric_rate = 1,
};

static int sun50iw15_i2s_component_probe(struct snd_soc_component *component)
{
	struct sun50iw15_i2s *i2s = snd_soc_component_get_drvdata(component);
	struct snd_soc_dai *dai;

	for_each_component_dais(component, dai)
		snd_soc_dai_init_dma_data(dai, &i2s->playback, &i2s->capture);

	/* Leave the block muted and disabled until a stream configures it. */
	regmap_update_bits(i2s->regmap, SUN50IW15_I2S_MUTE,
			   SUN50IW15_I2S_MUTE_TX, SUN50IW15_I2S_MUTE_TX);

	return 0;
}

static int sun50iw15_i2s_component_suspend(struct snd_soc_component *component)
{
	struct sun50iw15_i2s *i2s = snd_soc_component_get_drvdata(component);

	clk_disable_unprepare(i2s->clk_mod);
	clk_disable_unprepare(i2s->clk_bus);

	return 0;
}

static int sun50iw15_i2s_component_resume(struct snd_soc_component *component)
{
	struct sun50iw15_i2s *i2s = snd_soc_component_get_drvdata(component);
	int ret;

	ret = clk_prepare_enable(i2s->clk_bus);
	if (ret)
		return ret;

	return clk_prepare_enable(i2s->clk_mod);
}

static const struct snd_soc_component_driver sun50iw15_i2s_component = {
	.name			= "sun50iw15-i2s",
	.probe			= sun50iw15_i2s_component_probe,
	.suspend		= sun50iw15_i2s_component_suspend,
	.resume			= sun50iw15_i2s_component_resume,
};

static int sun50iw15_i2s_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sun50iw15_i2s *i2s;
	struct resource *res;
	void __iomem *base;
	unsigned int val;
	int ret;

	i2s = devm_kzalloc(dev, sizeof(*i2s), GFP_KERNEL);
	if (!i2s)
		return -ENOMEM;

	i2s->dev = dev;
	i2s->mclk_fs = 256;
	mutex_init(&i2s->lock);
	platform_set_drvdata(pdev, i2s);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	i2s->regmap = devm_regmap_init_mmio(dev, base,
					    &sun50iw15_i2s_regmap_config);
	if (IS_ERR(i2s->regmap))
		return dev_err_probe(dev, PTR_ERR(i2s->regmap),
				     "cannot map registers\n");

	i2s->clk_bus = devm_clk_get(dev, "clk_bus_i2s");
	if (IS_ERR(i2s->clk_bus))
		return dev_err_probe(dev, PTR_ERR(i2s->clk_bus),
				     "cannot get bus clock\n");

	i2s->clk_mod = devm_clk_get(dev, "clk_i2s");
	if (IS_ERR(i2s->clk_mod))
		return dev_err_probe(dev, PTR_ERR(i2s->clk_mod),
				     "cannot get module clock\n");

	i2s->clk_pll_audio = devm_clk_get_optional(dev, "clk_pll_audio_4x");
	if (IS_ERR(i2s->clk_pll_audio))
		return dev_err_probe(dev, PTR_ERR(i2s->clk_pll_audio),
				     "cannot get audio PLL\n");

	i2s->clk_pll_peri0 = devm_clk_get_optional(dev, "clk_pll_peri0_200m");
	if (IS_ERR(i2s->clk_pll_peri0))
		return dev_err_probe(dev, PTR_ERR(i2s->clk_pll_peri0),
				     "cannot get periph0 PLL\n");

	i2s->rst = devm_reset_control_get_shared(dev, NULL);
	if (IS_ERR(i2s->rst))
		return dev_err_probe(dev, PTR_ERR(i2s->rst),
				     "cannot get reset\n");

	i2s->tx_hub = device_property_read_bool(dev, "tx-hub-en");
	i2s->rx_sync = device_property_read_bool(dev, "rx-sync-en");

	/*
	 * The vendor enables one TX hub slot per tx-pin entry
	 * (priv->tx_pin[0..3]); a board without the property behaves like the
	 * codec node's tx-pin = <0>, so slot 0 is the default.
	 */
	i2s->tx_pin_mask = 1U << 0;
	if (device_property_present(dev, "tx-pin")) {
		u32 pins[4];
		int n, j;

		n = device_property_read_u32_array(dev, "tx-pin", pins,
						   ARRAY_SIZE(pins));
		if (n > 0) {
			i2s->tx_pin_mask = 0;
			for (j = 0; j < n; j++)
				if (pins[j] < 4)
					i2s->tx_pin_mask |= 1U << pins[j];
		}
	}

	/*
	 * Optional per-board channel maps; without them the identity map the
	 * vendor uses (and that the stock DT gets) stays in place.
	 */
	{
		char prop[32];
		unsigned int p;

		for (p = 0; p < SUN50IW15_I2S_MAP_PINS; p++) {
			snprintf(prop, sizeof(prop), "tx-pin%u-chmap", p);
			sun50iw15_i2s_read_chmap(dev, prop, i2s->tx_chmap[p],
						  true);
		}
		sun50iw15_i2s_read_chmap(dev, "rxfifo-chmap", i2s->rx_chmap, true);
		sun50iw15_i2s_read_chmap(dev, "rxfifo-pinmap", i2s->rx_pinmap,
					  false);
	}

	/*
	 * FIFO data ports, recovered from the vendor's sunxi_i2s_dev_probe():
	 * it stores iomem + 0x20 in the playback DMA params and iomem + 0x10 in
	 * the capture ones, with 0x808 in the following width/flags word.  That
	 * matches the upstream sun8i/H6 layout (TX FIFO at 0x20, RX at 0x10) and
	 * does not collide with this driver's 0x14 FIFO/DMA control or its 0x30..
	 * channel registers.  Both offsets can be overridden from the DT.
	 */
	val = SUN50IW15_I2S_FIFO_TX_DEFAULT;
	of_property_read_u32(dev->of_node, "tx-fifo-offset", &val);
	i2s->playback.addr = (dma_addr_t)(res->start + val);
	i2s->playback.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	i2s->playback.maxburst = 8;
	of_property_read_u32(dev->of_node, "tx-fifo-size",
			     &i2s->playback.fifo_size);

	val = SUN50IW15_I2S_FIFO_RX_DEFAULT;
	of_property_read_u32(dev->of_node, "rx-fifo-offset", &val);
	i2s->capture.addr = (dma_addr_t)(res->start + val);
	i2s->capture.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	i2s->capture.maxburst = 8;
	of_property_read_u32(dev->of_node, "rx-fifo-size",
			     &i2s->capture.fifo_size);

	if (!device_property_read_u32(dev, "tdm-num", &val))
		i2s->tdm_slots = val;
	if (!device_property_read_u32(dev, "mclk-fs", &val) && val)
		i2s->mclk_fs = val;

	i2s->slot_width = 32;

	ret = clk_prepare_enable(i2s->clk_bus);
	if (ret)
		return ret;

	ret = reset_control_deassert(i2s->rst);
	if (ret)
		goto err_bus;

	ret = clk_prepare_enable(i2s->clk_mod);
	if (ret)
		goto err_reset;

	dev_info(dev, "FIFO ports: tx %#x (%u B), rx %#x (%u B)\n",
		 (unsigned int)(res->start + SUN50IW15_I2S_FIFO_TX_DEFAULT),
		 i2s->playback.fifo_size,
		 (unsigned int)(res->start + SUN50IW15_I2S_FIFO_RX_DEFAULT),
		 i2s->capture.fifo_size);

	dev_dbg(dev, "tx-hub %d, rx-sync %d, tdm slots %u, mclk-fs %u\n",
		i2s->tx_hub, i2s->rx_sync, i2s->tdm_slots, i2s->mclk_fs);

	ret = devm_snd_soc_register_component(dev, &sun50iw15_i2s_component,
					      &sun50iw15_i2s_dai, 1);
	if (ret) {
		dev_err(dev, "cannot register ASoC component: %d\n", ret);
		goto err_clk;
	}

	ret = snd_sunxi_dma_platform_register(dev);
	if (ret) {
		dev_err(dev, "cannot register PCM engine: %d\n", ret);
		goto err_clk;
	}

	return 0;

err_clk:
	clk_disable_unprepare(i2s->clk_mod);
err_reset:
	reset_control_assert(i2s->rst);
err_bus:
	clk_disable_unprepare(i2s->clk_bus);
	return ret;
}

static int sun50iw15_i2s_remove(struct platform_device *pdev)
{
	struct sun50iw15_i2s *i2s = platform_get_drvdata(pdev);

	snd_sunxi_dma_platform_unregister(&pdev->dev);

	clk_disable_unprepare(i2s->clk_mod);
	reset_control_assert(i2s->rst);
	clk_disable_unprepare(i2s->clk_bus);

	return 0;
}

static const struct of_device_id sun50iw15_i2s_of_match[] = {
	{ .compatible = "allwinner,sunxi-snd-plat-i2s" },
	{ .compatible = "allwinner,sun8iw11-i2s" },
	{ .compatible = "allwinner,sun8iw17-i2s" },
	{ }
};
MODULE_DEVICE_TABLE(of, sun50iw15_i2s_of_match);

static struct platform_driver sun50iw15_i2s_driver = {
	.probe	= sun50iw15_i2s_probe,
	.remove	= sun50iw15_i2s_remove,
	.driver	= {
		.name		= "sun50iw15-i2s",
		.of_match_table	= sun50iw15_i2s_of_match,
	},
};
module_platform_driver(sun50iw15_i2s_driver);

MODULE_AUTHOR("H726 open kernel effort");
MODULE_DESCRIPTION("Allwinner sun50iw15 (H726) I2S/TDM controller");
MODULE_LICENSE("GPL");
