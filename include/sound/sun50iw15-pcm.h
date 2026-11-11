/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * sun50iw15 (H726) audio DMA engine ("sunxi pcm")
 *
 * The vendor splits its audio stack so that the i2s, owa and aaudio modules
 * each register their codec/DAI half and then call into
 * snd_soc_sunxi_pcm.ko for the PCM half.  These two calls are that entry
 * point; the platform device that owns the DMA channels must provide the
 * "tx"/"rx" dma-names, which every sunxi audio node in the stock DT does.
 */

#ifndef _SUN50IW15_PCM_H
#define _SUN50IW15_PCM_H

struct device;

/*
 * Set in snd_dmaengine_dai_dma_data.flags by a DAI whose playback stream
 * carries IEC 61937 bursts to be packed into IEC 60958 subframes (compressed
 * S/PDIF / HDMI passthrough).  Off by default: without it the engine behaves
 * exactly like the generic dmaengine PCM.
 */
#define SUNXI_PCM_DAI_FLAG_IEC61937	BIT(4)

int snd_sunxi_dma_platform_register(struct device *dev);
void snd_sunxi_dma_platform_unregister(struct device *dev);

#endif /* _SUN50IW15_PCM_H */
