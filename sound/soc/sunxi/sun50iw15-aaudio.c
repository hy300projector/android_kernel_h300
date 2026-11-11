// SPDX-License-Identifier: GPL-2.0
/*
 * Allwinner sun50iw15 (H726 / HY300) audio FIFO platform ("aaudio")
 *
 * Open replacement for the vendor snd_soc_sunxi_plat_aaudio driver
 * (compatible "allwinner,sunxi-snd-plat-aaudio").
 *
 * This block has no register window of its own: it is the DMA/FIFO front end
 * of the internal codec, and the device tree hands it the two FIFO data ports
 * inside the codec register block:
 *
 *   dac-txdata = <0x2030020>	playback FIFO data port
 *   adc-txdata = <0x2030040>	capture FIFO data port
 *
 * with "tx"/"rx" dma channels from the sunxi dma-controller.  So the driver is
 * a generic dmaengine PCM plus a DAI that exposes those two fixed addresses;
 * the codec driver owns the digital/analog configuration and the FIFO control
 * registers (0x14/0x24/0x38/0x44 in its control block).
 *
 * The vendor also carries playback-cma/capture-cma/tx-fifo-size/rx-fifo-size
 * properties here; they size the vendor's private CMA allocator and are not
 * needed with the generic dmaengine backend.
 */

#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/sun50iw15-pcm.h>

struct sunxi_aaudio {
	struct device *dev;
	struct snd_dmaengine_dai_dma_data playback;
	struct snd_dmaengine_dai_dma_data capture;
};

static int sunxi_aaudio_dai_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params,
				      struct snd_soc_dai *dai)
{
	struct snd_dmaengine_dai_dma_data *dma;

	dma = snd_soc_dai_get_dma_data(dai, substream);
	if (!dma)
		return -ENODEV;

	/* The FIFO port is 32-bit wide; the codec sets the sample format. */
	dma->addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;

	return 0;
}

static const struct snd_soc_dai_ops sunxi_aaudio_dai_ops = {
	.hw_params	= sunxi_aaudio_dai_hw_params,
};

static struct snd_soc_dai_driver sunxi_aaudio_dai = {
	.name	= "sunxi-aaudio",
	.playback = {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_8000_192000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S24_3LE |
				  SNDRV_PCM_FMTBIT_S24_LE |
				  SNDRV_PCM_FMTBIT_S32_LE,
	},
	.capture = {
		.stream_name	= "Capture",
		.channels_min	= 1,
		.channels_max	= 2,
		/* The codec rejects capture rates above 48 kHz. */
		.rates		= SNDRV_PCM_RATE_8000_48000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE |
				  SNDRV_PCM_FMTBIT_S24_3LE |
				  SNDRV_PCM_FMTBIT_S24_LE |
				  SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops		= &sunxi_aaudio_dai_ops,
	.symmetric_rate	= 1,
};

static int sunxi_aaudio_component_probe(struct snd_soc_component *component)
{
	struct sunxi_aaudio *p = snd_soc_component_get_drvdata(component);
	struct snd_soc_dai *dai;

	for_each_component_dais(component, dai)
		snd_soc_dai_init_dma_data(dai, &p->playback, &p->capture);

	return 0;
}

static const struct snd_soc_component_driver sunxi_aaudio_component = {
	.name		= "sunxi-aaudio",
	.probe		= sunxi_aaudio_component_probe,
};

static int sunxi_aaudio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct sunxi_aaudio *p;
	u32 txdata, rxdata;
	int ret;

	if (!np)
		return -ENODEV;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->dev = dev;
	platform_set_drvdata(pdev, p);

	ret = of_property_read_u32(np, "dac-txdata", &txdata);
	if (ret)
		return dev_err_probe(dev, ret,
				     "missing dac-txdata FIFO address\n");

	ret = of_property_read_u32(np, "adc-txdata", &rxdata);
	if (ret)
		return dev_err_probe(dev, ret,
				     "missing adc-txdata FIFO address\n");

	p->playback.addr = (dma_addr_t)txdata;
	p->playback.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	p->playback.maxburst = 8;

	p->capture.addr = (dma_addr_t)rxdata;
	p->capture.addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	p->capture.maxburst = 8;

	ret = devm_snd_soc_register_component(dev, &sunxi_aaudio_component,
					      &sunxi_aaudio_dai, 1);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register DAI\n");

	ret = snd_sunxi_dma_platform_register(dev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "cannot register PCM engine\n");

	dev_info(dev, "aaudio FIFO: playback @%#x, capture @%#x\n",
		 txdata, rxdata);

	return 0;
}

static const struct of_device_id sunxi_aaudio_of_match[] = {
	{ .compatible = "allwinner,sunxi-snd-plat-aaudio" },
	{ }
};
MODULE_DEVICE_TABLE(of, sunxi_aaudio_of_match);

static int sunxi_aaudio_remove(struct platform_device *pdev)
{
	snd_sunxi_dma_platform_unregister(&pdev->dev);

	return 0;
}

static struct platform_driver sunxi_aaudio_driver = {
	.probe	= sunxi_aaudio_probe,
	.remove	= sunxi_aaudio_remove,
	.driver	= {
		.name		= "sunxi-snd-plat-aaudio",
		.of_match_table	= sunxi_aaudio_of_match,
	},
};
module_platform_driver(sunxi_aaudio_driver);

MODULE_AUTHOR("H726 open kernel effort");
MODULE_DESCRIPTION("Allwinner sun50iw15 (H726) audio FIFO platform");
MODULE_LICENSE("GPL");
