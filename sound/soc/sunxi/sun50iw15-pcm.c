// SPDX-License-Identifier: GPL-2.0-only
/*
 * sun50iw15 (H726) audio DMA engine ("sunxi pcm")
 *
 * Open replacement for the vendor snd_soc_sunxi_pcm.ko, recovered from
 * dec_mod/snd_soc_sunxi_pcm.c.  The vendor module has no device tree
 * compatible: it is a library whose snd_sunxi_dma_platform_register() is
 * called by snd_soc_sunxi_i2s, _owa and _aaudio after they register their DAI
 * (which is why modules.dep lists it as a dependency of all three, and why
 * the stock DT has no pcm node - i2s_plat, owa_plat and codec_plat each carry
 * their own compatible and their own "tx"/"rx" dma-names).
 *
 * The PCM half is generic dmaengine PCM: a PCM-only ASoC component, a 64 KiB
 * preallocated managed buffer (clamped at 1 MiB, the values
 * sunxi_pcm_preallocate_dma_buffer() uses), the integer-periods constraint,
 * slave config built from the hw params and the DAI's
 * snd_dmaengine_dai_dma_data, and the dmaengine trigger/pointer/sync helpers.
 *
 * Compressed passthrough
 * ----------------------
 * The vendor's copy() also packs IEC 61937 bursts into IEC 60958 subframes
 * for S/PDIF/HDMI pass-through.  That path is reproduced here, but only for a
 * DAI that sets SUNXI_PCM_DAI_FLAG_IEC61937 in its DMA data (the OWA driver
 * does when its node has an "iec61937" property).  Without the flag the
 * engine is byte-for-byte the plain PCM path above.
 *
 * The packing itself is taken from the decompile: each 16-bit input word
 * becomes a 32-bit subframe with the sample in bits 26:11, a parity bit, and
 * one channel-status bit taken from a 384-frame status table whose entries
 * encode the sample rate.  The vendor allocates a second buffer of twice the
 * ring size, points the PCM's DMA address at it, doubles dma_bytes and
 * buffer_size, and packs into it at twice the byte offset; that geometry is
 * reproduced as recovered.  It expects the application to fill half of the
 * advertised ring with 16-bit words, which is what the vendor HAL does; a
 * stream that writes the whole ring is clamped rather than overrunning the
 * buffer.  None of this has run on hardware, and the bit layout is only as
 * good as the decompile.
 */

#include <linux/atomic.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <sound/dmaengine_pcm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/sun50iw15-pcm.h>

/* Vendor: sunxi_pcm_preallocate_dma_buffer() clamps the buffer size here. */
#define SUNXI_PCM_PREALLOC_SIZE		(64 * 1024)
#define SUNXI_PCM_BUFFER_MAX		(1024 * 1024)

/* Channel status block: one bit per frame, 384 frames per block. */
#define SUNXI_PCM_CS_BYTES		768
#define SUNXI_PCM_BLOCK_FRAMES		384

static const char *const sunxi_pcm_chan_names[] = {
	[SNDRV_PCM_STREAM_PLAYBACK]	= "tx",
	[SNDRV_PCM_STREAM_CAPTURE]	= "rx",
};

struct sunxi_pcm_stream {
	bool		packed;
	void		*pack_area;
	dma_addr_t	pack_addr;
	size_t		pack_size;
	dma_addr_t	saved_addr;
	unsigned int	saved_bytes;
	snd_pcm_uframes_t saved_buffer_size;
};

struct sunxi_pcm {
	struct snd_soc_component component;
	struct dma_chan *chan[SNDRV_PCM_STREAM_LAST + 1];
	struct sunxi_pcm_stream stream[SNDRV_PCM_STREAM_LAST + 1];
};

static inline struct sunxi_pcm *soc_component_to_sunxi_pcm(
	struct snd_soc_component *c)
{
	return container_of(c, struct sunxi_pcm, component);
}

static const struct snd_pcm_hardware sunxi_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED,
	.buffer_bytes_max	= SUNXI_PCM_BUFFER_MAX,
	.period_bytes_min	= 256,
	.period_bytes_max	= SUNXI_PCM_PREALLOC_SIZE,
	.periods_min		= 2,
	.periods_max		= 128,
};

static bool sunxi_pcm_wants_pack(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_dmaengine_dai_dma_data *dma_data;

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return false;

	dma_data = snd_soc_dai_get_dma_data(asoc_rtd_to_cpu(rtd, 0), substream);

	return dma_data && (dma_data->flags & SUNXI_PCM_DAI_FLAG_IEC61937);
}

/*
 * IEC 61937 -> IEC 60958 packing, from the vendor's sunxi_pcm_copy().
 *
 * The vendor's parity expression reduces to the XOR of all 16 payload bits,
 * the block counter runs over 384 frames and restarts one frame early, and
 * the channel-status table marks bit 2 of every block plus the rate bits the
 * decompile sets at byte offsets 0x60/0x64/0x68/0x6c.  44.1 kHz is encoded by
 * leaving them clear.
 */
static u8 sunxi_pcm_parity(u16 v)
{
	v ^= v >> 8;
	v ^= v >> 4;
	v ^= v >> 2;
	v ^= v >> 1;

	return v & 1;
}

static void sunxi_pcm_build_channel_status(u8 *cs, unsigned int rate,
					   unsigned int mode)
{
	memset(cs, 0, SUNXI_PCM_CS_BYTES);
	cs[4] = 1;

	if (rate < 128000) {
		if (rate == 32000) {
			cs[0x60] = 1;
			cs[0x64] = 1;
		} else if (rate != 44100) {
			cs[0x64] = 1;
		}
	} else if (rate == 128000) {
		cs[0x60] = 1;
	} else if (rate == 176400) {
		cs[0x68] = 1;
		cs[0x6c] = 1;
	} else if (rate == 192000) {
		cs[0x64] = 1;
		cs[0x68] = 1;
		cs[0x6c] = 1;
		if (mode - 1 < 2) {
			cs[0x60] = 1;
			cs[0x64] = 0;
			cs[0x68] = 0;
		}
	} else {
		cs[0x64] = 1;
	}
}

static void sunxi_pcm_pack(const u16 *in, u32 *out, unsigned int words,
			   unsigned int rate, unsigned int mode)
{
	static atomic_t block_pos = ATOMIC_INIT(0);
	u8 cs[SUNXI_PCM_CS_BYTES];
	u8 status = 0x88;
	unsigned int i;

	sunxi_pcm_build_channel_status(cs, rate, mode);

	for (i = 0; i < words; i++) {
		int counter = atomic_read(&block_pos);
		unsigned int seq = counter % SUNXI_PCM_BLOCK_FRAMES;

		atomic_set(&block_pos, seq == 0 ? 1 : counter + 1);

		/*
		 * One channel-status bit covers a frame *pair*: the vendor masks
		 * the counter with ~1 before indexing, so both subframes of a
		 * frame carry the same bit.  Indexing with the raw counter made
		 * the second subframe read the next bit instead.
		 */
		status = (status & 0x1f) |
			 ((seq < 2) ? 0x80 : 0) |
			 ((cs[(2 * (seq & ~1U)) % SUNXI_PCM_CS_BYTES] & 1) << 5) |
			 (sunxi_pcm_parity(in[i]) << 6);

		out[i] = ((u32)status << 24) | ((u32)in[i] << 11);
	}
}

static int sunxi_pcm_construct(struct snd_soc_component *component,
			       struct snd_soc_pcm_runtime *rtd)
{
	struct sunxi_pcm *pcm = soc_component_to_sunxi_pcm(component);
	struct device *dev = component->dev;
	unsigned int i;

	for_each_pcm_streams(i) {
		struct snd_pcm_substream *substream = rtd->pcm->streams[i].substream;

		if (!substream)
			continue;

		if (!pcm->chan[i]) {
			struct dma_chan *chan;

			chan = dma_request_chan(dev, sunxi_pcm_chan_names[i]);
			if (IS_ERR(chan)) {
				if (PTR_ERR(chan) == -EPROBE_DEFER)
					return -EPROBE_DEFER;
				dev_err(dev, "cannot get %s DMA channel: %ld\n",
					sunxi_pcm_chan_names[i], PTR_ERR(chan));
				return PTR_ERR(chan);
			}
			pcm->chan[i] = chan;
		}

		snd_pcm_set_managed_buffer(substream, SNDRV_DMA_TYPE_DEV, dev,
					   SUNXI_PCM_PREALLOC_SIZE,
					   SUNXI_PCM_BUFFER_MAX);
	}

	return 0;
}

static void sunxi_pcm_destruct(struct snd_soc_component *component,
			       struct snd_pcm *pcm_dev)
{
	struct sunxi_pcm *pcm = soc_component_to_sunxi_pcm(component);
	unsigned int i;

	for_each_pcm_streams(i) {
		if (pcm->chan[i]) {
			dma_release_channel(pcm->chan[i]);
			pcm->chan[i] = NULL;
		}
	}
}

static int sunxi_pcm_open(struct snd_soc_component *component,
			  struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct sunxi_pcm *pcm = soc_component_to_sunxi_pcm(component);
	struct snd_dmaengine_dai_dma_data *dma_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	int ret;

	dma_data = snd_soc_dai_get_dma_data(asoc_rtd_to_cpu(rtd, 0), substream);

	ret = snd_soc_set_runtime_hwparams(substream, &sunxi_pcm_hardware);
	if (ret)
		return ret;

	if (dma_data && dma_data->fifo_size)
		runtime->hw.fifo_size = dma_data->fifo_size;

	/* Packed playback uses a separate buffer, so no mmap there. */
	if (sunxi_pcm_wants_pack(substream)) {
		pcm->stream[substream->stream].packed = true;
		runtime->hw.info &= ~(SNDRV_PCM_INFO_MMAP |
				      SNDRV_PCM_INFO_MMAP_VALID);
	}

	/* Vendor: snd_pcm_hw_constraint_integer(runtime, PERIODS). */
	ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0) {
		dev_err(component->dev, "cannot constrain periods: %d\n", ret);
		return ret;
	}

	if (!pcm->chan[substream->stream])
		return -ENODEV;

	return snd_dmaengine_pcm_open(substream,
				      pcm->chan[substream->stream]);
}

static void sunxi_pcm_free_pack(struct sunxi_pcm *pcm,
				struct snd_pcm_substream *substream)
{
	struct sunxi_pcm_stream *st = &pcm->stream[substream->stream];

	if (!st->pack_area)
		return;

	dma_free_coherent(pcm->component.dev, st->pack_size, st->pack_area,
			  st->pack_addr);
	st->pack_area = NULL;
	st->pack_size = 0;

	if (substream->runtime) {
		substream->runtime->dma_addr = st->saved_addr;
		substream->runtime->dma_bytes = st->saved_bytes;
		substream->runtime->buffer_size = st->saved_buffer_size;
	}
}

static int sunxi_pcm_close(struct snd_soc_component *component,
			   struct snd_pcm_substream *substream)
{
	struct sunxi_pcm *pcm = soc_component_to_sunxi_pcm(component);

	sunxi_pcm_free_pack(pcm, substream);
	pcm->stream[substream->stream].packed = false;

	return snd_dmaengine_pcm_close(substream);
}

static int sunxi_pcm_hw_params(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params)
{
	struct sunxi_pcm *pcm = soc_component_to_sunxi_pcm(component);
	struct sunxi_pcm_stream *st = &pcm->stream[substream->stream];
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct dma_slave_config slave_config;
	size_t pack_size;
	int ret;

	/*
	 * A DAI that never published DMA data cannot be driven: refuse the
	 * stream instead of dereferencing a NULL address in the generic
	 * helper.
	 */
	if (!snd_soc_dai_get_dma_data(asoc_rtd_to_cpu(asoc_substream_to_rtd(substream), 0),
				      substream))
		return -ENODEV;

	memset(&slave_config, 0, sizeof(slave_config));

	ret = snd_dmaengine_pcm_prepare_slave_config(substream, params,
						     &slave_config);
	if (ret) {
		dev_err(component->dev, "cannot build slave config: %d\n", ret);
		return ret;
	}

	/*
	 * Vendor sunxi_pcm_hw_params(): a second buffer of twice the ring
	 * size, with the PCM's DMA address moved onto it.  The application's
	 * copy area stays where it is; only the DMA target changes.
	 */
	if (st->packed && !st->pack_area) {
		pack_size = params_buffer_bytes(params) * 2;

		st->pack_area = dma_alloc_coherent(component->dev, pack_size,
						   &st->pack_addr, GFP_KERNEL);
		if (!st->pack_area)
			return -ENOMEM;

		st->pack_size = pack_size;
		st->saved_addr = runtime->dma_addr;
		st->saved_bytes = runtime->dma_bytes;
		st->saved_buffer_size = runtime->buffer_size;

		runtime->dma_addr = st->pack_addr;
		runtime->dma_bytes = pack_size;
		runtime->buffer_size *= 2;

		dev_dbg(component->dev,
			"IEC 61937 passthrough: ring %zu, packed buffer %zu\n",
			(size_t)st->saved_bytes, pack_size);
	}

	ret = dmaengine_slave_config(pcm->chan[substream->stream],
				     &slave_config);
	if (ret)
		dev_err(component->dev, "cannot configure DMA channel: %d\n",
			ret);

	return ret;
}

static int sunxi_pcm_trigger(struct snd_soc_component *component,
			     struct snd_pcm_substream *substream, int cmd)
{
	return snd_dmaengine_pcm_trigger(substream, cmd);
}

static int sunxi_pcm_sync_stop(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream)
{
	return snd_dmaengine_pcm_sync_stop(substream);
}

static snd_pcm_uframes_t sunxi_pcm_pointer(struct snd_soc_component *component,
					   struct snd_pcm_substream *substream)
{
	return snd_dmaengine_pcm_pointer(substream);
}

/*
 * Vendor sunxi_pcm_copy(): user data lands in the PCM's own ring, then the
 * packed subframes are written at twice the byte offset into the DMA buffer.
 */
static int sunxi_pcm_copy_user(struct snd_soc_component *component,
			       struct snd_pcm_substream *substream, int channel,
			       unsigned long hwoff, void __user *buf,
			       unsigned long bytes)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sunxi_pcm *pcm = soc_component_to_sunxi_pcm(component);
	struct sunxi_pcm_stream *st = &pcm->stream[substream->stream];
	unsigned int words;
	void *dma_ptr = runtime->dma_area + hwoff +
			channel * (runtime->dma_bytes / runtime->channels);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		if (copy_from_user(dma_ptr, buf, bytes))
			return -EFAULT;

		if (!st->packed || !st->pack_area ||
		    runtime->format != SNDRV_PCM_FORMAT_S16_LE)
			return 0;

		/* The vendor packs only when the stream is at least stereo. */
		if (runtime->channels < 2)
			return 0;

		words = bytes / sizeof(u16);
		if (!words)
			return 0;

		/* The packed copy must fit; the vendor's HAL writes half. */
		if (2 * hwoff + words * sizeof(u32) > st->pack_size) {
			dev_warn_once(component->dev,
				      "IEC 61937: write past packed buffer dropped\n");
			return 0;
		}

		sunxi_pcm_pack(dma_ptr, st->pack_area + 2 * hwoff, words,
			       runtime->rate, 2);
		return 0;
	}

	if (copy_to_user(buf, dma_ptr, bytes))
		return -EFAULT;

	return 0;
}

static const struct snd_soc_component_driver sunxi_pcm_component = {
	.name		= "sunxi-pcm-audio",
	.probe_order	= SND_SOC_COMP_ORDER_LATE,
	.pcm_construct	= sunxi_pcm_construct,
	.pcm_destruct	= sunxi_pcm_destruct,
	.open		= sunxi_pcm_open,
	.close		= sunxi_pcm_close,
	.hw_params	= sunxi_pcm_hw_params,
	.trigger	= sunxi_pcm_trigger,
	.sync_stop	= sunxi_pcm_sync_stop,
	.pointer	= sunxi_pcm_pointer,
	.copy_user	= sunxi_pcm_copy_user,
};

int snd_sunxi_dma_platform_register(struct device *dev)
{
	struct sunxi_pcm *pcm;
	int ret;

	if (!dev)
		return -EINVAL;

	pcm = kzalloc(sizeof(*pcm), GFP_KERNEL);
	if (!pcm)
		return -ENOMEM;

	ret = snd_soc_component_initialize(&pcm->component, &sunxi_pcm_component,
					   dev);
	if (ret)
		goto err_free;

	ret = snd_soc_add_component(&pcm->component, NULL, 0);
	if (ret)
		goto err_free;

	dev_dbg(dev, "registered sunxi PCM engine\n");

	return 0;

err_free:
	kfree(pcm);
	return ret;
}
EXPORT_SYMBOL_GPL(snd_sunxi_dma_platform_register);

void snd_sunxi_dma_platform_unregister(struct device *dev)
{
	struct snd_soc_component *component;
	struct sunxi_pcm *pcm;
	unsigned int i;

	if (!dev)
		return;

	component = snd_soc_lookup_component(dev, "sunxi-pcm-audio");
	if (!component)
		return;

	pcm = soc_component_to_sunxi_pcm(component);

	snd_soc_unregister_component_by_driver(dev, component->driver);

	for_each_pcm_streams(i) {
		if (pcm->chan[i])
			dma_release_channel(pcm->chan[i]);
	}

	kfree(pcm);
}
EXPORT_SYMBOL_GPL(snd_sunxi_dma_platform_unregister);

MODULE_DESCRIPTION("Allwinner sun50iw15 (H726) audio DMA engine");
MODULE_LICENSE("GPL");
