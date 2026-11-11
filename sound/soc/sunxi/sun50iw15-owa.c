// SPDX-License-Identifier: GPL-2.0
/*
 * Allwinner sun50iw15 (H726 / HY300) OWA (S/PDIF / HDMI audio) controller
 *
 * Open replacement for the vendor snd_soc_sunxi_owa module (compatible
 * "allwinner,sunxi-snd-plat-owa").  Everything below comes from the
 * decompiled module (dec_mod/snd_soc_sunxi_owa.c):
 *
 *   sunxi_owa_init()        probe-time init sequence
 *   sunxi_owa_dai_hw_params format / rate programming
 *   sunxi_owa_dai_prepare() FIFO flush and start conditions
 *   sunxi_owa_dai_trigger() TX run bit and RX control
 *   sunxi_owa_rxctrl_enable() RX enable/disable
 *   sunxi_owa_set_tx_data_fmt() / sunxi_owa_set_rx_data_fmt()
 *
 * The sample-rate codes are not guesses: sample_rate_freq and sample_rate_orig
 * were read out of the vendor .rodata (72 bytes each, pairs of
 * { rate, code }).  Rates missing from those tables (8 kHz, 11.025 kHz,
 * 12 kHz, 16 kHz) are not representable and are rejected.
 *
 * PCM uses the generic dmaengine backend with the "tx"/"rx" channels from the
 * device tree, like the I2S and aaudio drivers of this SoC.
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/sun50iw15-pcm.h>

/* Global enable (0x00) */
#define SUNXI_OWA_CTRL			0x00
#define SUNXI_OWA_CTRL_TX_EN		BIT(0)
#define SUNXI_OWA_CTRL_RX_EN		BIT(1)
#define SUNXI_OWA_CTRL_EN_ALL		GENMASK(1, 0)

/* TX control (0x04) */
#define SUNXI_OWA_TX_CTRL		0x04
#define SUNXI_OWA_TX_CTRL_RUN		BIT(0)
#define SUNXI_OWA_TX_CTRL_WIDTH_MASK	GENMASK(3, 2)
#define SUNXI_OWA_TX_CTRL_WIDTH(c)	((c) << 2)
#define SUNXI_OWA_TX_CTRL_DIV_MASK	GENMASK(12, 4)
#define SUNXI_OWA_TX_CTRL_DIV(div)	(((div) << 4) & SUNXI_OWA_TX_CTRL_DIV_MASK)
#define SUNXI_OWA_TX_CTRL_FMT_16	BIT(16)
#define SUNXI_OWA_TX_CTRL_HUB		BIT(31)

/* RX control (0x08) */
#define SUNXI_OWA_RX_CTRL		0x08
#define SUNXI_OWA_RX_CTRL_EN		BIT(0)
#define SUNXI_OWA_RX_CTRL_RUN		BIT(1)

/* Interrupt status, write one to clear (0x0c) */
#define SUNXI_OWA_INT_STAT		0x0c

/* DMA / FIFO control (0x14) */
#define SUNXI_OWA_DMA_CTRL		0x14
#define SUNXI_OWA_DMA_TX_DRQ		BIT(0)
#define SUNXI_OWA_DMA_RX_DRQ		BIT(1)
#define SUNXI_OWA_DMA_TX_WIDTH		BIT(2)
#define SUNXI_OWA_DMA_RX_WIDTH_MASK	GENMASK(1, 0)
#define SUNXI_OWA_DMA_RX_WIDTH_24	GENMASK(1, 0)
#define SUNXI_OWA_DMA_RX_SYNC		BIT(21)

/* Path control (0x1c) */
#define SUNXI_OWA_PATH_CTRL		0x1c
#define SUNXI_OWA_PATH_RX		BIT(2)
#define SUNXI_OWA_PATH_TX_HUB		BIT(7)

/* FIFO flush (0x24 TX, 0x28 RX) */
#define SUNXI_OWA_TX_FIFO		0x24
#define SUNXI_OWA_RX_FIFO		0x28

/* TX slot/rate select (0x2c) and TX format (0x30) */
#define SUNXI_OWA_TX_SLOT		0x2c
#define SUNXI_OWA_TX_SLOT_RATE_MASK	GENMASK(27, 24)
#define SUNXI_OWA_TX_SLOT_RATE(c)	((c) << 24)
#define SUNXI_OWA_TX_SLOT_FMT		BIT(1)
#define SUNXI_OWA_TX_FMT		0x30
#define SUNXI_OWA_TX_FMT_RATE_MASK	GENMASK(7, 4)
#define SUNXI_OWA_TX_FMT_RATE(c)	((c) << 4)
#define SUNXI_OWA_TX_FMT_WIDTH_MASK	GENMASK(3, 0)

/* RX slot/rate select (0x34) and RX format (0x38) */
#define SUNXI_OWA_RX_SLOT		0x34
#define SUNXI_OWA_RX_SLOT_RATE_MASK	GENMASK(27, 24)
#define SUNXI_OWA_RX_SLOT_RATE(c)	((c) << 24)
#define SUNXI_OWA_RX_SLOT_FMT		BIT(1)
#define SUNXI_OWA_RX_FMT		0x38
#define SUNXI_OWA_RX_FMT_RATE_MASK	GENMASK(7, 4)
#define SUNXI_OWA_RX_FMT_RATE(c)	((c) << 4)
#define SUNXI_OWA_RX_FMT_WIDTH_MASK	GENMASK(3, 0)

/* RX data format (0x40) */
#define SUNXI_OWA_RX_DATA_FMT		0x40
#define SUNXI_OWA_RX_DATA_FMT_MASK	GENMASK(14, 13)

#define SUNXI_OWA_MAX_REG		0x58

/* Width classes used by the vendor: 16-bit, 24-bit and 32-bit samples. */
enum sunxi_owa_width {
	SUNXI_OWA_WIDTH_16 = 0,
	SUNXI_OWA_WIDTH_24,
	SUNXI_OWA_WIDTH_32,
};

/* { rate, code } pairs extracted from the vendor .rodata. */
static const unsigned int sunxi_owa_rate_orig[][2] = {
	{ 22050,  0xb }, { 24000,  0x9 }, { 32000,  0xc }, { 44100,  0xf },
	{ 48000,  0xd }, { 88200,  0x7 }, { 96000,  0x5 }, { 176400, 0x3 },
	{ 192000, 0x1 },
};

static const unsigned int sunxi_owa_rate_freq[][2] = {
	{ 22050,  0x4 }, { 24000,  0x6 }, { 32000,  0x3 }, { 44100,  0x0 },
	{ 48000,  0x2 }, { 88200,  0x8 }, { 96000,  0xa }, { 176400, 0xc },
	{ 192000, 0xe },
};

struct sun50iw15_owa {
	struct device *dev;
	struct regmap *regmap;
	struct clk *clk_bus;
	struct clk *clk_pll_audio;
	struct clk *clk_pll_peri300;
	struct clk *clk_pll_peri400;
	struct clk *clk_tx;
	struct clk *clk_rx;
	struct reset_control *rst;

	struct snd_dmaengine_dai_dma_data playback;
	struct snd_dmaengine_dai_dma_data capture;

	unsigned int pll_fs;
	bool tx_hub;
	bool clocks_on;
	struct mutex lock;
};

static const struct regmap_config sun50iw15_owa_regmap_config = {
	.reg_bits	= 32,
	.val_bits	= 32,
	.reg_stride	= 4,
	.max_register	= SUNXI_OWA_MAX_REG,
};

static int sunxi_owa_lookup(const unsigned int table[][2], unsigned int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(sunxi_owa_rate_orig); i++)
		if (table[i][0] == rate)
			return table[i][1];

	return -EINVAL;
}

static unsigned int sunxi_owa_width_of(snd_pcm_format_t format)
{
	switch (format) {
	case SNDRV_PCM_FORMAT_S16_LE:
		return SUNXI_OWA_WIDTH_16;
	case SNDRV_PCM_FORMAT_S24_LE:
	case SNDRV_PCM_FORMAT_S24_3LE:
		return SUNXI_OWA_WIDTH_24;
	default:
		return SUNXI_OWA_WIDTH_32;
	}
}

/* TX width codes: 0x0 for 16-bit, 0xc for 24-bit, 0xb for 32-bit. */
static unsigned int sunxi_owa_tx_width_code(unsigned int width)
{
	switch (width) {
	case SUNXI_OWA_WIDTH_16:
		return 0x0;
	case SUNXI_OWA_WIDTH_24:
		return 0xc;
	default:
		return 0xb;
	}
}

/* RX width codes: 0x2 for 16-bit, 0xc for 24-bit, 0xb for 32-bit. */
static unsigned int sunxi_owa_rx_width_code(unsigned int width)
{
	switch (width) {
	case SUNXI_OWA_WIDTH_16:
		return 0x2;
	case SUNXI_OWA_WIDTH_24:
		return 0xc;
	default:
		return 0xb;
	}
}

/* sunxi_owa_init(): the vendor probe-time sequence. */
static void sunxi_owa_init(struct sun50iw15_owa *owa)
{
	regmap_update_bits(owa->regmap, SUNXI_OWA_DMA_CTRL,
			   SUNXI_OWA_DMA_TX_DRQ, SUNXI_OWA_DMA_TX_DRQ);
	regmap_update_bits(owa->regmap, SUNXI_OWA_DMA_CTRL,
			   SUNXI_OWA_DMA_RX_DRQ, SUNXI_OWA_DMA_RX_DRQ);
	regmap_update_bits(owa->regmap, SUNXI_OWA_TX_CTRL,
			   SUNXI_OWA_TX_CTRL_RUN, 0);
	regmap_write(owa->regmap, SUNXI_OWA_TX_SLOT, 0x200000);
	regmap_write(owa->regmap, SUNXI_OWA_RX_SLOT, 0x200000);
	regmap_update_bits(owa->regmap, SUNXI_OWA_CTRL,
			   SUNXI_OWA_CTRL_RX_EN, SUNXI_OWA_CTRL_RX_EN);
}

static int sunxi_owa_clocks_on(struct sun50iw15_owa *owa)
{
	int ret;

	if (owa->clocks_on)
		return 0;

	ret = clk_prepare_enable(owa->clk_pll_audio);
	if (ret)
		return ret;
	ret = clk_prepare_enable(owa->clk_pll_peri300);
	if (ret)
		goto err_audio;
	ret = clk_prepare_enable(owa->clk_pll_peri400);
	if (ret)
		goto err_peri300;
	ret = clk_prepare_enable(owa->clk_tx);
	if (ret)
		goto err_peri400;
	ret = clk_prepare_enable(owa->clk_rx);
	if (ret)
		goto err_tx;

	owa->clocks_on = true;
	return 0;

err_tx:
	clk_disable_unprepare(owa->clk_tx);
err_peri400:
	clk_disable_unprepare(owa->clk_pll_peri400);
err_peri300:
	clk_disable_unprepare(owa->clk_pll_peri300);
err_audio:
	clk_disable_unprepare(owa->clk_pll_audio);
	return ret;
}

static void sunxi_owa_clocks_off(struct sun50iw15_owa *owa)
{
	if (!owa->clocks_on)
		return;

	clk_disable_unprepare(owa->clk_rx);
	clk_disable_unprepare(owa->clk_tx);
	clk_disable_unprepare(owa->clk_pll_peri400);
	clk_disable_unprepare(owa->clk_pll_peri300);
	clk_disable_unprepare(owa->clk_pll_audio);
	owa->clocks_on = false;
}

static int sunxi_owa_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct sun50iw15_owa *owa = snd_soc_component_get_drvdata(dai->component);
	struct snd_dmaengine_dai_dma_data *dma;
	unsigned int rate = params_rate(params);
	unsigned int width = sunxi_owa_width_of(params_format(params));
	unsigned int rcode_orig, rcode_freq;
	int ret;

	if (!owa)
		return -ENODEV;

	rcode_orig = sunxi_owa_lookup(sunxi_owa_rate_orig, rate);
	rcode_freq = sunxi_owa_lookup(sunxi_owa_rate_freq, rate);
	if ((int)rcode_orig < 0 || (int)rcode_freq < 0) {
		dev_err(owa->dev, "unsupported sample rate %u\n", rate);
		return -EINVAL;
	}

	dma = snd_soc_dai_get_dma_data(dai, substream);
	if (dma)
		dma->addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;

	mutex_lock(&owa->lock);

	ret = sunxi_owa_clocks_on(owa);
	if (ret) {
		mutex_unlock(&owa->lock);
		return ret;
	}

	/* The OWA clock runs at the sample rate times the PLL factor. */
	clk_set_parent(owa->clk_tx, owa->clk_pll_audio);
	clk_set_parent(owa->clk_rx, owa->clk_pll_audio);
	clk_set_rate(owa->clk_tx, rate * owa->pll_fs);
	clk_set_rate(owa->clk_rx, rate * owa->pll_fs);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		regmap_update_bits(owa->regmap, SUNXI_OWA_TX_CTRL,
				   SUNXI_OWA_TX_CTRL_WIDTH_MASK,
				   SUNXI_OWA_TX_CTRL_WIDTH(width));
		regmap_update_bits(owa->regmap, SUNXI_OWA_DMA_CTRL,
				   SUNXI_OWA_DMA_TX_WIDTH,
				   width == SUNXI_OWA_WIDTH_16 ? 0 :
				   SUNXI_OWA_DMA_TX_WIDTH);
		regmap_update_bits(owa->regmap, SUNXI_OWA_TX_CTRL,
				   SUNXI_OWA_TX_CTRL_HUB,
				   owa->tx_hub ? SUNXI_OWA_TX_CTRL_HUB : 0);
		regmap_update_bits(owa->regmap, SUNXI_OWA_TX_SLOT,
				   SUNXI_OWA_TX_SLOT_RATE_MASK,
				   SUNXI_OWA_TX_SLOT_RATE(rcode_orig));
		regmap_update_bits(owa->regmap, SUNXI_OWA_TX_FMT,
				   SUNXI_OWA_TX_FMT_RATE_MASK,
				   SUNXI_OWA_TX_FMT_RATE(rcode_freq));
		regmap_update_bits(owa->regmap, SUNXI_OWA_TX_FMT,
				   SUNXI_OWA_TX_FMT_WIDTH_MASK,
				   sunxi_owa_tx_width_code(width));
	} else {
		regmap_update_bits(owa->regmap, SUNXI_OWA_DMA_CTRL,
				   SUNXI_OWA_DMA_RX_WIDTH_MASK,
				   SUNXI_OWA_DMA_RX_WIDTH_24);
		regmap_update_bits(owa->regmap, SUNXI_OWA_RX_SLOT,
				   SUNXI_OWA_RX_SLOT_RATE_MASK,
				   SUNXI_OWA_RX_SLOT_RATE(rcode_orig));
		regmap_update_bits(owa->regmap, SUNXI_OWA_RX_FMT,
				   SUNXI_OWA_RX_FMT_RATE_MASK,
				   SUNXI_OWA_RX_FMT_RATE(rcode_freq));
		regmap_update_bits(owa->regmap, SUNXI_OWA_RX_FMT,
				   SUNXI_OWA_RX_FMT_WIDTH_MASK,
				   sunxi_owa_rx_width_code(width));
	}

	mutex_unlock(&owa->lock);
	return 0;
}

static int sunxi_owa_hw_free(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct sun50iw15_owa *owa = snd_soc_component_get_drvdata(dai->component);

	mutex_lock(&owa->lock);
	sunxi_owa_clocks_off(owa);
	mutex_unlock(&owa->lock);

	return 0;
}

static int sunxi_owa_prepare(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct sun50iw15_owa *owa = snd_soc_component_get_drvdata(dai->component);
	unsigned int stat;

	mutex_lock(&owa->lock);
	regmap_update_bits(owa->regmap, SUNXI_OWA_CTRL,
			   SUNXI_OWA_CTRL_RX_EN, 0);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		regmap_update_bits(owa->regmap, SUNXI_OWA_DMA_CTRL,
				   SUNXI_OWA_DMA_TX_DRQ, SUNXI_OWA_DMA_TX_DRQ);
	else
		regmap_update_bits(owa->regmap, SUNXI_OWA_DMA_CTRL,
				   SUNXI_OWA_DMA_RX_DRQ, SUNXI_OWA_DMA_RX_DRQ);

	regmap_write(owa->regmap,
		     substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
		     SUNXI_OWA_TX_FIFO : SUNXI_OWA_RX_FIFO, 0);

	/* Clear any stale interrupt state, then enable the block. */
	regmap_read(owa->regmap, SUNXI_OWA_INT_STAT, &stat);
	regmap_write(owa->regmap, SUNXI_OWA_INT_STAT, stat);
	regmap_update_bits(owa->regmap, SUNXI_OWA_CTRL,
			   SUNXI_OWA_CTRL_EN_ALL, SUNXI_OWA_CTRL_EN_ALL);

	mutex_unlock(&owa->lock);
	return 0;
}

static void sunxi_owa_rx_enable(struct sun50iw15_owa *owa, bool on)
{
	if (on) {
		regmap_update_bits(owa->regmap, SUNXI_OWA_RX_CTRL,
				   SUNXI_OWA_RX_CTRL_RUN, SUNXI_OWA_RX_CTRL_RUN);
		regmap_update_bits(owa->regmap, SUNXI_OWA_PATH_CTRL,
				   SUNXI_OWA_PATH_RX, SUNXI_OWA_PATH_RX);
		regmap_update_bits(owa->regmap, SUNXI_OWA_RX_CTRL,
				   SUNXI_OWA_RX_CTRL_EN, SUNXI_OWA_RX_CTRL_EN);
	} else {
		regmap_update_bits(owa->regmap, SUNXI_OWA_RX_CTRL,
				   SUNXI_OWA_RX_CTRL_EN, 0);
		regmap_update_bits(owa->regmap, SUNXI_OWA_PATH_CTRL,
				   SUNXI_OWA_PATH_RX, 0);
	}
}

static int sunxi_owa_trigger(struct snd_pcm_substream *substream, int cmd,
			     struct snd_soc_dai *dai)
{
	struct sun50iw15_owa *owa = snd_soc_component_get_drvdata(dai->component);
	bool start = cmd == SNDRV_PCM_TRIGGER_START ||
		     cmd == SNDRV_PCM_TRIGGER_RESUME ||
		     cmd == SNDRV_PCM_TRIGGER_PAUSE_RELEASE;
	bool stop = cmd == SNDRV_PCM_TRIGGER_STOP ||
		    cmd == SNDRV_PCM_TRIGGER_SUSPEND ||
		    cmd == SNDRV_PCM_TRIGGER_PAUSE_PUSH;

	if (!start && !stop)
		return -EINVAL;

	mutex_lock(&owa->lock);
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		regmap_update_bits(owa->regmap, SUNXI_OWA_TX_CTRL,
				   SUNXI_OWA_TX_CTRL_RUN,
				   start ? SUNXI_OWA_TX_CTRL_RUN : 0);
		regmap_update_bits(owa->regmap, SUNXI_OWA_PATH_CTRL,
				   SUNXI_OWA_PATH_TX_HUB,
				   (start && owa->tx_hub) ?
				   SUNXI_OWA_PATH_TX_HUB : 0);
	} else {
		sunxi_owa_rx_enable(owa, start);
	}
	mutex_unlock(&owa->lock);

	return 0;
}

static int sunxi_owa_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct sun50iw15_owa *owa = snd_soc_component_get_drvdata(dai->component);
	bool spdif_fmt = (fmt & SND_SOC_DAIFMT_FORMAT_MASK) ==
			 SND_SOC_DAIFMT_I2S ? false : true;

	/* The vendor's TX/RX "data fmt" bit selects the raw payload mode. */
	regmap_update_bits(owa->regmap, SUNXI_OWA_TX_CTRL, SUNXI_OWA_TX_CTRL_FMT_16,
			   spdif_fmt ? SUNXI_OWA_TX_CTRL_FMT_16 : 0);
	regmap_update_bits(owa->regmap, SUNXI_OWA_TX_SLOT, SUNXI_OWA_TX_SLOT_FMT,
			   spdif_fmt ? SUNXI_OWA_TX_SLOT_FMT : 0);
	regmap_update_bits(owa->regmap, SUNXI_OWA_RX_SLOT, SUNXI_OWA_RX_SLOT_FMT,
			   spdif_fmt ? SUNXI_OWA_RX_SLOT_FMT : 0);

	return 0;
}

static int sunxi_owa_set_clkdiv(struct snd_soc_dai *dai, int div_id, int div)
{
	struct sun50iw15_owa *owa = snd_soc_component_get_drvdata(dai->component);
	unsigned int val;

	/* Vendor: ((div >> 3) & ~0xf) - 0x10, in bits 8:4 of the TX control. */
	val = (((div >> 3) & ~0xfU) - 0x10) & SUNXI_OWA_TX_CTRL_DIV_MASK;
	regmap_update_bits(owa->regmap, SUNXI_OWA_TX_CTRL,
			   SUNXI_OWA_TX_CTRL_DIV_MASK, val);

	return 0;
}

static const struct snd_soc_dai_ops sunxi_owa_dai_ops = {
	.hw_params	= sunxi_owa_hw_params,
	.hw_free	= sunxi_owa_hw_free,
	.prepare	= sunxi_owa_prepare,
	.trigger	= sunxi_owa_trigger,
	.set_fmt	= sunxi_owa_set_fmt,
	.set_clkdiv	= sunxi_owa_set_clkdiv,
};

#define SUNXI_OWA_FORMATS	(SNDRV_PCM_FMTBIT_S16_LE | \
				 SNDRV_PCM_FMTBIT_S24_3LE | \
				 SNDRV_PCM_FMTBIT_S24_LE | \
				 SNDRV_PCM_FMTBIT_S32_LE)

#define SUNXI_OWA_RATES		(SNDRV_PCM_RATE_22050 | \
				 SNDRV_PCM_RATE_32000 | \
				 SNDRV_PCM_RATE_44100 | \
				 SNDRV_PCM_RATE_48000 | \
				 SNDRV_PCM_RATE_88200 | \
				 SNDRV_PCM_RATE_96000 | \
				 SNDRV_PCM_RATE_176400 | \
				 SNDRV_PCM_RATE_192000)

static struct snd_soc_dai_driver sunxi_owa_dai = {
	.name	= "sunxi-owa",
	.playback = {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 8,
		.rates		= SUNXI_OWA_RATES,
		.formats	= SUNXI_OWA_FORMATS,
	},
	.capture = {
		.stream_name	= "Capture",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= SUNXI_OWA_RATES,
		.formats	= SUNXI_OWA_FORMATS,
	},
	.ops		= &sunxi_owa_dai_ops,
	.symmetric_rate	= 1,
};

static int sunxi_owa_component_probe(struct snd_soc_component *component)
{
	struct sun50iw15_owa *owa = snd_soc_component_get_drvdata(component);
	struct snd_soc_dai *dai;

	for_each_component_dais(component, dai)
		snd_soc_dai_init_dma_data(dai, &owa->playback, &owa->capture);

	return 0;
}

static const struct snd_soc_component_driver sunxi_owa_component = {
	.name		= "sunxi-owa",
	.probe		= sunxi_owa_component_probe,
};

static int sunxi_owa_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sun50iw15_owa *owa;
	struct resource *res;
	void __iomem *base;
	u32 tx_offset = 0x20, rx_offset = 0x10;
	int ret;

	owa = devm_kzalloc(dev, sizeof(*owa), GFP_KERNEL);
	if (!owa)
		return -ENOMEM;

	owa->dev = dev;
	owa->pll_fs = 1;
	mutex_init(&owa->lock);
	platform_set_drvdata(pdev, owa);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return dev_err_probe(dev, PTR_ERR(base), "cannot map registers\n");

	owa->regmap = devm_regmap_init_mmio(dev, base,
					    &sun50iw15_owa_regmap_config);
	if (IS_ERR(owa->regmap))
		return dev_err_probe(dev, PTR_ERR(owa->regmap),
				     "cannot create regmap\n");

	owa->clk_bus = devm_clk_get(dev, "clk_bus_owa");
	if (IS_ERR(owa->clk_bus))
		return dev_err_probe(dev, PTR_ERR(owa->clk_bus), "no bus clock\n");

	owa->clk_pll_audio = devm_clk_get(dev, "clk_pll_audio_4x");
	if (IS_ERR(owa->clk_pll_audio))
		return dev_err_probe(dev, PTR_ERR(owa->clk_pll_audio),
				     "no audio PLL\n");

	owa->clk_pll_peri300 = devm_clk_get(dev, "clk_pll_peri0_300m");
	if (IS_ERR(owa->clk_pll_peri300))
		return dev_err_probe(dev, PTR_ERR(owa->clk_pll_peri300),
				     "no 300 MHz PLL\n");

	owa->clk_pll_peri400 = devm_clk_get(dev, "clk_pll_peri0_400m");
	if (IS_ERR(owa->clk_pll_peri400))
		return dev_err_probe(dev, PTR_ERR(owa->clk_pll_peri400),
				     "no 400 MHz PLL\n");

	owa->clk_tx = devm_clk_get(dev, "clk_owa_tx");
	if (IS_ERR(owa->clk_tx))
		return dev_err_probe(dev, PTR_ERR(owa->clk_tx), "no tx clock\n");

	owa->clk_rx = devm_clk_get(dev, "clk_owa_rx");
	if (IS_ERR(owa->clk_rx))
		return dev_err_probe(dev, PTR_ERR(owa->clk_rx), "no rx clock\n");

	owa->rst = devm_reset_control_get_optional_shared(dev, NULL);
	if (IS_ERR(owa->rst))
		return dev_err_probe(dev, PTR_ERR(owa->rst), "no reset\n");

	ret = reset_control_deassert(owa->rst);
	if (ret)
		return ret;

	ret = clk_prepare_enable(owa->clk_bus);
	if (ret)
		return dev_err_probe(dev, ret, "cannot enable bus clock\n");

	owa->tx_hub = of_property_read_bool(dev->of_node, "tx-hub-en");
	of_property_read_u32(dev->of_node, "pll-fs", &owa->pll_fs);
	if (!owa->pll_fs)
		owa->pll_fs = 1;

	/*
	 * FIFO data ports.  RX is the vendor's own value (base + 0x10); TX
	 * mirrors the block layout at base + 0x20.  Both can be overridden
	 * from the device tree if a board turns out to differ.
	 */
	of_property_read_u32(dev->of_node, "tx-fifo-offset", &tx_offset);
	of_property_read_u32(dev->of_node, "rx-fifo-offset", &rx_offset);

	owa->playback.addr = (dma_addr_t)(res->start + tx_offset);
	owa->playback.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	owa->playback.maxburst = 8;

	owa->capture.addr = (dma_addr_t)(res->start + rx_offset);
	owa->capture.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	owa->capture.maxburst = 8;

	/*
	 * Compressed passthrough is opt-in: the stock DT says nothing about
	 * it, so without "iec61937" the engine runs the plain PCM path.
	 */
	if (of_property_read_bool(dev->of_node, "iec61937"))
		owa->playback.flags |= SUNXI_PCM_DAI_FLAG_IEC61937;

	sunxi_owa_init(owa);

	ret = devm_snd_soc_register_component(dev, &sunxi_owa_component,
					      &sunxi_owa_dai, 1);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register DAI\n");

	ret = snd_sunxi_dma_platform_register(dev);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register PCM engine\n");

	dev_info(dev, "OWA ready: tx FIFO %#x, rx FIFO %#x, pll-fs %u%s%s\n",
		 tx_offset, rx_offset, owa->pll_fs,
		 owa->tx_hub ? ", tx hub mode" : "",
		 (owa->playback.flags & SUNXI_PCM_DAI_FLAG_IEC61937) ?
		 ", IEC 61937 passthrough" : "");

	return 0;
}

static const struct of_device_id sunxi_owa_of_match[] = {
	{ .compatible = "allwinner,sunxi-snd-plat-owa" },
	{ }
};
MODULE_DEVICE_TABLE(of, sunxi_owa_of_match);

static int sunxi_owa_remove(struct platform_device *pdev)
{
	snd_sunxi_dma_platform_unregister(&pdev->dev);

	return 0;
}

static struct platform_driver sunxi_owa_driver = {
	.probe	= sunxi_owa_probe,
	.remove	= sunxi_owa_remove,
	.driver	= {
		.name		= "sunxi-snd-plat-owa",
		.of_match_table	= sunxi_owa_of_match,
	},
};
module_platform_driver(sunxi_owa_driver);

MODULE_AUTHOR("H726 open kernel effort");
MODULE_DESCRIPTION("Allwinner sun50iw15 (H726) OWA S/PDIF / HDMI audio");
MODULE_LICENSE("GPL");
