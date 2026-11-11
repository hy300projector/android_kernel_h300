// SPDX-License-Identifier: GPL-2.0
/*
 * Allwinner sun50iw15 (H726 / HY300) sound card
 *
 * Open replacement for the vendor snd_soc_sunxi_machine module
 * (compatible "allwinner,sunxi-snd-mach").
 *
 * The vendor driver is a thin layer over ASoC's simple-card helpers and reads
 * its configuration from "soundcard-mach,*" properties:
 *
 *   soundcard-mach,name               card name
 *   soundcard-mach,format             i2s / left_j / right_j / dsp_a / dsp_b
 *   soundcard-mach,slot-num           TDM slot count
 *   soundcard-mach,slot-width         TDM slot width
 *   soundcard-mach,frame-master       phandle of the DAI that is frame master
 *   soundcard-mach,bitclock-master    phandle of the DAI that is bitclock master
 *
 * and, on the "soundcard-mach,cpu" child:
 *
 *   sound-dai                         phandle of the CPU DAI
 *   soundcard-mach,pll-fs             PLL multiple of the sample rate
 *   soundcard-mach,mclk-fs            mclk multiple of the mclk base
 *   soundcard-mach,mclk-fp            { 44.1k, 48k } mclk base frequencies
 *
 * On the stock HY300 device tree the codec child node is empty (there is no
 * codec behind this link), so the driver registers a local dummy codec to give
 * ASoC a codec DAI to bind.  A real codec is used if one appears in the DT.
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/soc-jack.h>
#include <sound/sun50iw15-snd-common.h>

#define SUNXI_MACH_PREFIX	"soundcard-mach,"
#define SUNXI_MACH_DEFAULT_MCLK_FS	256

struct sunxi_mach {
	struct device *dev;

	struct snd_soc_card card;
	struct snd_soc_dai_link link;
	struct snd_soc_jack jack;
	struct snd_soc_dai_link_component cpu;
	struct snd_soc_dai_link_component codec;
	struct snd_soc_dai_link_component platform;

	struct snd_soc_component_driver dummy_component;
	struct snd_soc_dai_driver dummy_dai;

	struct device_node *cpu_np;
	struct device_node *codec_np;

	unsigned int dai_fmt;
	unsigned int pll_fs;
	unsigned int mclk_fs;
	unsigned int mclk_fp[2];
	unsigned int slot_num;
	unsigned int slot_width;
	bool have_codec;
};

/*
 * The vendor computes the mclk as "mclk_fs * mclk_fp" for the 44.1k and 48k
 * families (its own debug output spells this out).  Without mclk-fp it falls
 * back to a plain multiple of the sample rate.
 */
static unsigned int sunxi_mach_mclk(struct sunxi_mach *m, unsigned int rate)
{
	unsigned int fs;

	if (m->mclk_fp[0] && m->mclk_fp[1]) {
		unsigned int fp = (rate % 44100) ? m->mclk_fp[1] : m->mclk_fp[0];

		/* mclk-fs defaults to 1 when an explicit base is given. */
		fs = m->mclk_fs ? m->mclk_fs : 1;
		return fp * fs;
	}

	fs = m->mclk_fs ? m->mclk_fs : SUNXI_MACH_DEFAULT_MCLK_FS;
	return rate * fs;
}

static int sunxi_mach_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct sunxi_mach *m = snd_soc_card_get_drvdata(rtd->card);
	unsigned int mclk = sunxi_mach_mclk(m, params_rate(params));
	int ret;

	if (m->pll_fs)
		mclk *= m->pll_fs;

	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, mclk, SND_SOC_CLOCK_IN);
	if (ret && ret != -ENOTSUPP)
		return ret;

	if (m->have_codec) {
		struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);

		ret = snd_soc_dai_set_sysclk(codec_dai, 0, mclk,
					     SND_SOC_CLOCK_IN);
		if (ret && ret != -ENOTSUPP)
			return ret;
	}

	return 0;
}

/*
 * TDM setup.  The vendor I2S block only accepts slot counts that encode to
 * 0..6 in its FMT register (8, 12, 16, 20, 24, 28, 32), so a stereo
 * "slot-num = <2>" from the DT must not be pushed into set_tdm_slot().
 */
static int sunxi_mach_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *cpu_dai = asoc_rtd_to_cpu(rtd, 0);
	struct sunxi_mach *m = snd_soc_card_get_drvdata(rtd->card);
	unsigned int mask;
	int ret;

	ret = snd_soc_dai_set_fmt(cpu_dai, m->dai_fmt);
	if (ret && ret != -ENOTSUPP)
		return ret;

	if (m->slot_num >= 8 && m->slot_num <= 32 && !(m->slot_num % 4)) {
		mask = (m->slot_num == 32) ? ~0 : ((1 << m->slot_num) - 1);

		ret = snd_soc_dai_set_tdm_slot(cpu_dai, mask, mask,
					       m->slot_num,
					       m->slot_width ?: 32);
		if (ret && ret != -ENOTSUPP)
			return ret;
	}

	return 0;
}

static const struct snd_soc_ops sunxi_mach_ops = {
	.hw_params	= sunxi_mach_hw_params,
};

static int sunxi_mach_parse_format(struct device *dev, struct device_node *np,
				   unsigned int *fmt)
{
	const char *str;
	int ret;

	ret = of_property_read_string(np, SUNXI_MACH_PREFIX "format", &str);
	if (ret) {
		*fmt = SND_SOC_DAIFMT_I2S;
		return 0;
	}

	if (!strcmp(str, "i2s"))
		*fmt = SND_SOC_DAIFMT_I2S;
	else if (!strcmp(str, "left_j") || !strcmp(str, "left-j"))
		*fmt = SND_SOC_DAIFMT_LEFT_J;
	else if (!strcmp(str, "right_j") || !strcmp(str, "right-j"))
		*fmt = SND_SOC_DAIFMT_RIGHT_J;
	else if (!strcmp(str, "dsp_a") || !strcmp(str, "dsp-a"))
		*fmt = SND_SOC_DAIFMT_DSP_A;
	else if (!strcmp(str, "dsp_b") || !strcmp(str, "dsp-b"))
		*fmt = SND_SOC_DAIFMT_DSP_B;
	else
		return dev_err_probe(dev, -EINVAL,
				     "unknown soundcard-mach,format \"%s\"\n",
				     str);

	return 0;
}

/*
 * A master phandle naming the CPU node means the CPU drives the clocks, which
 * in ASoC terms is "codec is slave": SND_SOC_DAIFMT_CBS_CFS.
 */
static void sunxi_mach_parse_master(struct sunxi_mach *m, struct device_node *np,
				    struct device_node *cpu_child)
{
	struct device_node *master;
	bool cpu_master = false;

	master = of_parse_phandle(np, SUNXI_MACH_PREFIX "frame-master", 0);
	if (master) {
		if (master == cpu_child)
			cpu_master = true;
		of_node_put(master);
	}

	master = of_parse_phandle(np, SUNXI_MACH_PREFIX "bitclock-master", 0);
	if (master) {
		if (master == cpu_child)
			cpu_master = true;
		of_node_put(master);
	}

	m->dai_fmt |= cpu_master ? SND_SOC_DAIFMT_CBS_CFS :
				   SND_SOC_DAIFMT_CBM_CFM;
}

static int sunxi_mach_parse_dai(struct device *dev, struct device_node *child,
				struct snd_soc_dai_link_component *dlc,
				struct device_node **np_out)
{
	struct device_node *np;
	const char *dai_name;
	int ret;

	if (!child)
		return -ENODEV;

	ret = snd_soc_of_get_dai_name(child, &dai_name);
	if (ret)
		return dev_err_probe(dev, ret, "cannot resolve DAI name\n");

	np = of_parse_phandle(child, "sound-dai", 0);
	if (!np)
		return -ENODEV;

	dlc->of_node = np;
	dlc->dai_name = dai_name;
	*np_out = np;

	return 0;
}

static void sunxi_mach_dummy_dai_setup(struct sunxi_mach *m)
{
	m->dummy_dai.name = "sunxi-dummy-dai";
	m->dummy_dai.playback.stream_name = "Dummy Playback";
	m->dummy_dai.playback.channels_min = 1;
	m->dummy_dai.playback.channels_max = 16;
	m->dummy_dai.playback.rates = SNDRV_PCM_RATE_8000_192000;
	m->dummy_dai.playback.formats = SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE;
	m->dummy_dai.capture.stream_name = "Dummy Capture";
	m->dummy_dai.capture.channels_min = 1;
	m->dummy_dai.capture.channels_max = 16;
	m->dummy_dai.capture.rates = SNDRV_PCM_RATE_8000_192000;
	m->dummy_dai.capture.formats = SNDRV_PCM_FMTBIT_S16_LE |
				       SNDRV_PCM_FMTBIT_S24_LE |
				       SNDRV_PCM_FMTBIT_S32_LE;

	m->dummy_component.name = "sunxi-dummy-codec";
}

static int sunxi_mach_late_probe(struct snd_soc_card *card);

static int sunxi_mach_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *cpu_child, *codec_child;
	struct sunxi_mach *m;
	unsigned int val;
	int ret;

	if (!np)
		return -ENODEV;

	m = devm_kzalloc(dev, sizeof(*m), GFP_KERNEL);
	if (!m)
		return -ENOMEM;

	m->dev = dev;

	/* Card name, defaulting to the machine node's name when absent. */
	ret = of_property_read_string(np, SUNXI_MACH_PREFIX "name", &m->card.name);
	if (ret)
		m->card.name = np->name;

	ret = sunxi_mach_parse_format(dev, np, &m->dai_fmt);
	if (ret)
		return ret;

	if (!of_property_read_u32(np, SUNXI_MACH_PREFIX "slot-num", &val))
		m->slot_num = val;
	if (!of_property_read_u32(np, SUNXI_MACH_PREFIX "slot-width", &val))
		m->slot_width = val;

	cpu_child = of_get_child_by_name(np, SUNXI_MACH_PREFIX "cpu");
	ret = sunxi_mach_parse_dai(dev, cpu_child, &m->cpu, &m->cpu_np);
	if (ret) {
		of_node_put(cpu_child);
		return ret;
	}

	sunxi_mach_parse_master(m, np, cpu_child);

	if (!of_property_read_u32(cpu_child, SUNXI_MACH_PREFIX "pll-fs", &val))
		m->pll_fs = val;
	if (!of_property_read_u32(cpu_child, SUNXI_MACH_PREFIX "mclk-fs", &val))
		m->mclk_fs = val;
	if (!of_property_read_u32_array(cpu_child, SUNXI_MACH_PREFIX "mclk-fp",
					m->mclk_fp, 2))
		dev_dbg(dev, "mclk-fp %u / %u, mclk-fs %u, pll-fs %u\n",
			m->mclk_fp[0], m->mclk_fp[1], m->mclk_fs, m->pll_fs);

	of_node_put(cpu_child);

	/* The CPU DAI's component also provides the DMA platform. */
	m->platform.of_node = m->cpu.of_node;
	m->platform.dai_name = m->cpu.dai_name;

	codec_child = of_get_child_by_name(np, SUNXI_MACH_PREFIX "codec");
	if (codec_child && of_find_property(codec_child, "sound-dai", NULL)) {
		ret = sunxi_mach_parse_dai(dev, codec_child, &m->codec,
					   &m->codec_np);
		if (ret) {
			of_node_put(codec_child);
			return ret;
		}
		m->have_codec = true;
	} else {
		sunxi_mach_dummy_dai_setup(m);

		ret = devm_snd_soc_register_component(dev, &m->dummy_component,
						      &m->dummy_dai, 1);
		if (ret) {
			of_node_put(codec_child);
			return dev_err_probe(dev, ret,
					     "cannot register dummy codec\n");
		}

		m->codec.name = "sunxi-dummy-codec";
		m->codec.dai_name = "sunxi-dummy-dai";
	}
	of_node_put(codec_child);

	m->link.name = "sunxi-i2s";
	m->link.stream_name = "sunxi-i2s";
	m->link.cpus = &m->cpu;
	m->link.num_cpus = 1;
	m->link.codecs = &m->codec;
	m->link.num_codecs = 1;
	m->link.platforms = &m->platform;
	m->link.num_platforms = 1;
	m->link.dai_fmt = m->dai_fmt;
	m->link.init = sunxi_mach_dai_init;
	m->link.ops = &sunxi_mach_ops;

	m->card.dev = dev;
	m->card.owner = THIS_MODULE;
	m->card.dai_link = &m->link;
	m->card.num_links = 1;
	m->card.late_probe = sunxi_mach_late_probe;
	snd_soc_card_set_drvdata(&m->card, m);

	platform_set_drvdata(pdev, m);

	dev_info(dev, "registering card \"%s\" (fmt 0x%x)\n",
		 m->card.name, m->dai_fmt);

	return devm_snd_soc_register_card(dev, &m->card);
}

/*
 * The aaudio card owns the internal codec's headphone jack; the I2S card
 * shares the same codec but must not register a second jack on it.
 */
static int sunxi_mach_late_probe(struct snd_soc_card *card)
{
	struct sunxi_mach *m = snd_soc_card_get_drvdata(card);
	struct snd_soc_pcm_runtime *rtd;
	struct snd_soc_component *component;
	int ret;

	if (!m->have_codec ||
	    !of_device_is_compatible(m->cpu_np,
				     "allwinner,sunxi-snd-plat-aaudio"))
		return 0;

	if (!of_find_property(m->codec_np, "hp-det-gpio", NULL))
		return 0;

	/* Pins are registered by the codec's set_jack callback. */
	ret = snd_sunxi_card_jack_new(card, "Headphone Jack",
				      SND_JACK_HEADPHONE, &m->jack);
	if (ret) {
		dev_err(card->dev, "cannot create jack: %d\n", ret);
		return ret;
	}

	rtd = snd_soc_get_pcm_runtime(card, &m->link);
	if (!rtd)
		return -ENODEV;

	component = asoc_rtd_to_codec(rtd, 0)->component;

	return snd_soc_component_set_jack(component, &m->jack, NULL);
}

static int sunxi_mach_remove(struct platform_device *pdev)
{
	struct sunxi_mach *m = platform_get_drvdata(pdev);

	of_node_put(m->cpu_np);
	of_node_put(m->codec_np);

	return 0;
}

static const struct of_device_id sunxi_mach_of_match[] = {
	{ .compatible = "allwinner,sunxi-snd-mach" },
	{ }
};
MODULE_DEVICE_TABLE(of, sunxi_mach_of_match);

static struct platform_driver sunxi_mach_driver = {
	.probe	= sunxi_mach_probe,
	.remove	= sunxi_mach_remove,
	.driver	= {
		.name		= "sun50iw15-snd-mach",
		.of_match_table	= sunxi_mach_of_match,
	},
};
module_platform_driver(sunxi_mach_driver);

MODULE_AUTHOR("H726 open kernel effort");
MODULE_DESCRIPTION("Allwinner sun50iw15 (H726) sound card");
MODULE_LICENSE("GPL");
