// SPDX-License-Identifier: GPL-2.0
/*
 * sun50iw15 Trident audio bridge ("audbrg") ALSA card.
 *
 * Rewritten from the vendor snd_alsa_trid.ko.  The vendor module is a
 * 15k-line audio middleware; this driver implements the part that the
 * bridge hardware actually needs for the ALSA "TridentPCM" card:
 *
 *   - the bridge register block (channel windows, interrupt enable/status,
 *     high address latch, audio clock setup),
 *   - the 384 KiB shared DMA buffer, split into four 64 KiB input ring
 *     buffers (istream, playback to the bridge) and two 64 KiB output ring
 *     buffers (ostream, capture from the bridge),
 *   - four PCM devices, matching the vendor card layout.
 *
 * Vendor register layout, all offsets from the bridge block base:
 *
 *   ostream channel n: 0x100 + n * 0x40
 *     0x00 start address >> 4      0x04 end address >> 4
 *     0x0c period bytes >> 4       0x10 format word
 *     0x14 flush (1 then 0)        0x18 current position, 16-byte units
 *
 *   istream channel n: 0x280 + n * 0x40
 *     0x00 start address >> 4      0x04 end address >> 4
 *     0x0c period bytes >> 4       0x10 format word
 *     0x14 flush (1 then 0)        0x18 reset (1 then 0)
 *     0x1c current position, 16-byte units
 *
 *   0x384 interrupt enable (bit 0 global, bit n+4 ostream, bit n+10 istream)
 *   0x388 interrupt mask  (same bit layout)
 *   0x38c interrupt status, write-one-to-clear (0x70 ostream, 0x3c00 istream)
 *   0x390 high address latch: bits 3:0 istream, bits 7:4 ostream
 *
 * The current position is read back as
 *     ((latch << 28) | (16 * (reg & 0xffffff))) - channel base address,
 * which is masked to the 64 KiB ring size.
 *
 * Format word, from the vendor audbrg_*_config() functions:
 *     istream: (bytes_per_sample << 8) | (channels << 14) |
 *              (wide << 11) | (mono_pair << 12)
 *     ostream: (bytes_per_sample << 8) | (channels << 11) | (stream << 12)
 * where "wide" is 1 for 8/16 bit and 0 for 32 bit, "mono_pair" is 2 for
 * 16-bit mono and 3 for 32-bit mono (0 otherwise), matching
 * Thal_Alsa_Audio_Play_Start().
 *
 * Not implemented, because the vendor code gives no register evidence:
 * the /dev/trid_audio private ioctl API and the "owa pin src" kcontrol.
 * Android's audio HAL uses the ALSA PCMs, which is what this driver exposes.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>

#define TRID_DRV_NAME		"sun50iw15-trid"

/* Sub-block offsets inside the mapped audio-bridge window. */
#define TRID_MSP_OFF		0x0000
#define TRID_DBP_OFF		0x2000
#define TRID_BRG_OFF		0x4000
#define TRID_CLK_OFF		0x6000
#define TRID_MAP_SIZE		0x7000

/* Bridge channel registers (offset from TRID_BRG_OFF). */
#define BRG_OST_BASE		0x100
#define BRG_IST_BASE		0x280
#define BRG_CH_STRIDE		0x40
#define BRG_CH_START		0x00
#define BRG_CH_END		0x04
#define BRG_CH_PERIOD		0x0c
#define BRG_CH_FMT		0x10
#define BRG_CH_FLUSH		0x14
#define BRG_IST_RESET		0x18
#define BRG_IST_POS		0x1c
#define BRG_OST_POS		0x18

/* Global bridge registers. */
#define BRG_INT_EN		0x384
#define BRG_INT_MASK		0x388
#define BRG_INT_STATUS		0x38c
#define BRG_HIADDR		0x390

#define BRG_INT_GLOBAL		BIT(0)
#define BRG_INT_OST(ch)		BIT((ch) + 4)
#define BRG_INT_IST(ch)		BIT((ch) + 10)
#define BRG_INT_OST_MASK	0x70
#define BRG_INT_IST_MASK	0x3c00

/* Audio clock setup registers (offset from TRID_CLK_OFF). */
#define CLK_CTRL		0x00
#define CLK_PLL			0x0c

#define TRID_CH_BUF		0x10000
#define TRID_IST_CHANNELS	4
#define TRID_OST_CHANNELS	2
#define TRID_BUF_SIZE		(TRID_IST_CHANNELS * TRID_CH_BUF + \
				 TRID_OST_CHANNELS * TRID_CH_BUF)
#define TRID_OST_OFFSET		(TRID_IST_CHANNELS * TRID_CH_BUF)

#define TRID_PERIOD_MIN		256
#define TRID_PERIOD_MAX		TRID_CH_BUF
#define TRID_BUFFER_MAX		(256 * 1024)
#define TRID_PUMP_MS		5

struct trid_bridge;

struct trid_stream {
	struct trid_bridge *tr;
	struct snd_pcm_substream *substream;
	unsigned int ch;
	bool capture;
	bool running;
	unsigned int period_bytes;
	u32 pushed;	/* bytes handed to / taken from the bridge */
	u32 since_report;	/* bytes accounted since the last period */
	u32 hw_off;	/* write/read offset inside the 64 KiB ring */
	u32 al_off;	/* matching offset inside the ALSA ring */
	u8 *ring;	/* the 64 KiB bridge ring for this channel */
};

struct trid_bridge {
	struct device *dev;
	void __iomem *msp;
	void __iomem *dbp;
	void __iomem *brg;
	void __iomem *clk;

	struct snd_card *card;
	struct snd_pcm *pcm[4];

	void *buf;
	dma_addr_t buf_dma;
	dma_addr_t buf_addr;	/* address the bridge is programmed with */

	struct delayed_work pump;
	spinlock_t lock;
	bool pumping;

	struct trid_stream playback[TRID_IST_CHANNELS];
	struct trid_stream capture[TRID_OST_CHANNELS];
};

static inline void brg_write(struct trid_bridge *tr, u32 off, u32 val)
{
	writel(val, tr->brg + off);
}

static inline u32 brg_read(struct trid_bridge *tr, u32 off)
{
	return readl(tr->brg + off);
}

/*
 * Program the high address latch the bridge uses to reach the ring
 * buffers.  The vendor shifts the physical address into the latch, writes
 * the channel window (whose registers only hold bits 27:4), then clears
 * the latch again.
 */
static void trid_latch_hi(struct trid_bridge *tr, bool capture, u32 hi)
{
	u32 val = capture ? ((hi & 0xf) << 4) : (hi & 0xf);

	brg_write(tr, BRG_HIADDR, val);
}

static u32 trid_ch_base(struct trid_stream *s)
{
	return (s->capture ? BRG_OST_BASE : BRG_IST_BASE) + s->ch * BRG_CH_STRIDE;
}

static u32 trid_pos(struct trid_stream *s)
{
	struct trid_bridge *tr = s->tr;
	u32 base = trid_ch_base(s);
	unsigned long flags;
	u32 latch, pos;

	spin_lock_irqsave(&tr->lock, flags);
	latch = brg_read(tr, BRG_HIADDR);
	pos = brg_read(tr, base + (s->capture ? BRG_OST_POS : BRG_IST_POS));
	spin_unlock_irqrestore(&tr->lock, flags);

	return (((latch & 0xf) << 28) | (16 * (pos & 0xffffff))) &
	       (TRID_CH_BUF - 1);
}

static unsigned int trid_format_word(unsigned int bps, unsigned int channels,
				     bool capture)
{
	unsigned int wide, mono_pair;

	if (capture) {
		/* Display side captures SPDIF; byte width and channel count. */
		return (bps << 8) | (channels << 11);
	}

	wide = (bps == 4) ? 0 : 1;
	mono_pair = 0;
	if (channels == 1)
		mono_pair = (bps == 4) ? 1 : (bps == 2 ? 2 : 0);

	return (bps << 8) | (channels << 14) | (wide << 11) |
	       (mono_pair << 12);
}

static void trid_program_channel(struct trid_stream *s)
{
	struct trid_bridge *tr = s->tr;
	struct snd_pcm_runtime *rt = s->substream->runtime;
	u32 base = trid_ch_base(s);
	phys_addr_t start = s->capture ?
		tr->buf_addr + TRID_OST_OFFSET + s->ch * TRID_CH_BUF :
		tr->buf_addr + s->ch * TRID_CH_BUF;
	unsigned int bps = snd_pcm_format_physical_width(rt->format) / 8;
	unsigned int channels = rt->channels;

	trid_latch_hi(tr, s->capture, start >> 28);
	brg_write(tr, base + BRG_CH_START, (start >> 4) & 0xffffff);
	brg_write(tr, base + BRG_CH_END,
		  ((start + TRID_CH_BUF - 1) >> 4) & 0xffffff);
	brg_write(tr, base + BRG_CH_PERIOD, s->period_bytes >> 4);
	brg_write(tr, base + BRG_CH_FMT, trid_format_word(bps, channels,
							 s->capture));
	trid_latch_hi(tr, s->capture, 0);

	/* Flush, and for the input path reset the channel. */
	brg_write(tr, base + BRG_CH_FLUSH, 1);
	brg_write(tr, base + BRG_CH_FLUSH, 0);
	if (!s->capture) {
		brg_write(tr, base + BRG_IST_RESET, 1);
		brg_write(tr, base + BRG_IST_RESET, 0);
	}

	/* Clear any stale status, then unmask this channel. */
	brg_write(tr, BRG_INT_STATUS,
		  s->capture ? BRG_INT_OST(s->ch) : BRG_INT_IST(s->ch));
	brg_write(tr, BRG_INT_MASK,
		  s->capture ? BRG_INT_OST(s->ch) : BRG_INT_IST(s->ch));
}

static bool trid_pump_playback(struct trid_stream *s)
{
	struct snd_pcm_runtime *rt = s->substream->runtime;
	u32 buf_bytes = frames_to_bytes(rt, rt->buffer_size);
	u32 appl = frames_to_bytes(rt, rt->control->appl_ptr);
	u32 avail, space, done = 0, hw, sw;
	bool elapsed = false;

	if (!s->running || !rt->dma_area)
		return false;

	hw = trid_pos(s);
	sw = s->hw_off;
	space = TRID_CH_BUF - ((sw - hw) & (TRID_CH_BUF - 1));

	avail = appl - s->pushed;
	if (avail > buf_bytes)
		avail = buf_bytes;
	avail = min(avail, space);

	while (done < avail) {
		u32 chunk = min(avail - done, buf_bytes - s->al_off);

		chunk = min(chunk, TRID_CH_BUF - sw);
		memcpy(s->ring + sw, rt->dma_area + s->al_off, chunk);

		sw += chunk;
		if (sw >= TRID_CH_BUF)
			sw -= TRID_CH_BUF;
		s->al_off += chunk;
		if (s->al_off >= buf_bytes)
			s->al_off -= buf_bytes;
		done += chunk;
	}

	s->hw_off = sw;
	s->pushed += done;
	s->since_report += done;
	if (s->period_bytes && s->since_report >= s->period_bytes) {
		s->since_report %= s->period_bytes;
		elapsed = true;
	}

	return elapsed;
}

static bool trid_pump_capture(struct trid_stream *s)
{
	struct snd_pcm_runtime *rt = s->substream->runtime;
	u32 buf_bytes = frames_to_bytes(rt, rt->buffer_size);
	u32 appl = frames_to_bytes(rt, rt->control->appl_ptr);
	u32 used, room, done = 0, hw, sw, inflight;
	bool elapsed = false;

	if (!s->running || !rt->dma_area)
		return false;

	hw = trid_pos(s);
	sw = s->hw_off;
	used = (hw - sw) & (TRID_CH_BUF - 1);

	inflight = s->pushed - appl;
	room = buf_bytes - min(inflight, buf_bytes);
	used = min(used, room);

	while (done < used) {
		u32 chunk = min(used - done, buf_bytes - s->al_off);

		chunk = min(chunk, TRID_CH_BUF - sw);
		memcpy(rt->dma_area + s->al_off, s->ring + sw, chunk);

		sw += chunk;
		if (sw >= TRID_CH_BUF)
			sw -= TRID_CH_BUF;
		s->al_off += chunk;
		if (s->al_off >= buf_bytes)
			s->al_off -= buf_bytes;
		done += chunk;
	}

	s->hw_off = sw;
	s->pushed += done;
	s->since_report += done;
	if (s->period_bytes && s->since_report >= s->period_bytes) {
		s->since_report %= s->period_bytes;
		elapsed = true;
	}

	return elapsed;
}

static void trid_pump(struct work_struct *work)
{
	struct trid_bridge *tr = container_of(work, struct trid_bridge,
					      pump.work);
	struct snd_pcm_substream *play[TRID_IST_CHANNELS];
	struct snd_pcm_substream *cap[TRID_OST_CHANNELS];
	unsigned int i, nplay = 0, ncap = 0;
	unsigned long flags;
	bool again = false;

	spin_lock_irqsave(&tr->lock, flags);
	for (i = 0; i < TRID_IST_CHANNELS; i++) {
		if (tr->playback[i].running) {
			play[nplay++] = tr->playback[i].substream;
			again = true;
		}
	}
	for (i = 0; i < TRID_OST_CHANNELS; i++) {
		if (tr->capture[i].running) {
			cap[ncap++] = tr->capture[i].substream;
			again = true;
		}
	}
	spin_unlock_irqrestore(&tr->lock, flags);

	for (i = 0; i < nplay; i++) {
		struct trid_stream *s = play[i]->private_data;

		if (trid_pump_playback(s))
			snd_pcm_period_elapsed(play[i]);
	}
	for (i = 0; i < ncap; i++) {
		struct trid_stream *s = cap[i]->private_data;

		if (trid_pump_capture(s))
			snd_pcm_period_elapsed(cap[i]);
	}

	if (again)
		schedule_delayed_work(&tr->pump,
				      msecs_to_jiffies(TRID_PUMP_MS));
}

static irqreturn_t trid_irq(int irq, void *data)
{
	struct trid_bridge *tr = data;
	u32 status = brg_read(tr, BRG_INT_STATUS);

	if (!status)
		return IRQ_NONE;

	/* Write one to clear. */
	brg_write(tr, BRG_INT_STATUS, status);
	schedule_delayed_work(&tr->pump, 0);

	return IRQ_HANDLED;
}

static int trid_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *rt = substream->runtime;
	struct trid_bridge *tr = snd_pcm_substream_chip(substream);
	struct trid_stream *s;
	bool capture = substream->stream == SNDRV_PCM_STREAM_CAPTURE;

	if (capture) {
		if (substream->pcm->device >= TRID_OST_CHANNELS)
			return -ENODEV;
		s = &tr->capture[substream->pcm->device];
	} else {
		if (substream->pcm->device >= TRID_IST_CHANNELS)
			return -ENODEV;
		s = &tr->playback[substream->pcm->device];
	}

	s->tr = tr;
	s->substream = substream;
	s->ch = substream->pcm->device;
	s->capture = capture;
	s->running = false;
	s->pushed = 0;
	s->since_report = 0;
	s->hw_off = 0;
	s->al_off = 0;
	s->period_bytes = 0;
	s->ring = capture ?
		(u8 *)tr->buf + TRID_OST_OFFSET + s->ch * TRID_CH_BUF :
		(u8 *)tr->buf + s->ch * TRID_CH_BUF;
	substream->private_data = s;

	rt->hw = (struct snd_pcm_hardware){
		.info = SNDRV_PCM_INFO_MMAP |
			SNDRV_PCM_INFO_MMAP_VALID |
			SNDRV_PCM_INFO_INTERLEAVED |
			SNDRV_PCM_INFO_BLOCK_TRANSFER,
		.formats = SNDRV_PCM_FMTBIT_U8 |
			   SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
		.rates = SNDRV_PCM_RATE_8000_192000,
		.rate_min = 8000,
		.rate_max = 192000,
		.channels_min = 1,
		.channels_max = 8,
		.buffer_bytes_max = TRID_BUFFER_MAX,
		.period_bytes_min = TRID_PERIOD_MIN,
		.period_bytes_max = TRID_PERIOD_MAX,
		.periods_min = 2,
		.periods_max = 32,
	};

	return 0;
}

static int trid_pcm_close(struct snd_pcm_substream *substream)
{
	struct trid_stream *s = substream->private_data;
	unsigned long flags;

	spin_lock_irqsave(&s->tr->lock, flags);
	s->running = false;
	spin_unlock_irqrestore(&s->tr->lock, flags);

	return 0;
}

static int trid_pcm_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct trid_stream *s = substream->private_data;

	s->period_bytes = params_period_bytes(params);
	return 0;
}

static int trid_pcm_hw_free(struct snd_pcm_substream *substream)
{
	return 0;
}

static int trid_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct trid_stream *s = substream->private_data;
	unsigned long flags;

	s->pushed = 0;
	s->since_report = 0;
	s->hw_off = 0;
	s->al_off = 0;

	spin_lock_irqsave(&s->tr->lock, flags);
	trid_program_channel(s);
	spin_unlock_irqrestore(&s->tr->lock, flags);

	return 0;
}

static int trid_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct trid_stream *s = substream->private_data;
	unsigned long flags;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		spin_lock_irqsave(&s->tr->lock, flags);
		s->running = true;
		spin_unlock_irqrestore(&s->tr->lock, flags);
		schedule_delayed_work(&s->tr->pump, 0);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		spin_lock_irqsave(&s->tr->lock, flags);
		s->running = false;
		spin_unlock_irqrestore(&s->tr->lock, flags);
		return 0;
	default:
		return -EINVAL;
	}
}

static snd_pcm_uframes_t trid_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct trid_stream *s = substream->private_data;
	struct snd_pcm_runtime *rt = substream->runtime;
	unsigned long flags;
	u32 pushed;

	spin_lock_irqsave(&s->tr->lock, flags);
	pushed = s->pushed;
	spin_unlock_irqrestore(&s->tr->lock, flags);

	return bytes_to_frames(rt, pushed % frames_to_bytes(rt, rt->buffer_size));
}

static const struct snd_pcm_ops trid_pcm_ops = {
	.open		= trid_pcm_open,
	.close		= trid_pcm_close,
	.hw_params	= trid_pcm_hw_params,
	.hw_free	= trid_pcm_hw_free,
	.prepare	= trid_pcm_prepare,
	.trigger	= trid_pcm_trigger,
	.pointer	= trid_pcm_pointer,
};

static void trid_clock_init(struct trid_bridge *tr)
{
	u32 val;

	val = readl(tr->clk + CLK_CTRL);
	writel(val | 0x700, tr->clk + CLK_CTRL);

	val = readl(tr->clk + CLK_PLL);
	writel((val & 0xff000000) | 0x1a5e00, tr->clk + CLK_PLL);

	val = readl(tr->clk + CLK_PLL);
	writel(val | 0x1000000, tr->clk + CLK_PLL);
}

static int trid_card_new(struct trid_bridge *tr)
{
	static const struct {
		unsigned int playback;
		unsigned int capture;
	} layout[4] = {
		{ 1, 1 }, { 1, 1 }, { 1, 0 }, { 1, 0 },
	};
	struct snd_card *card;
	unsigned int i;
	int ret;

	ret = snd_card_new(tr->dev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1,
			   THIS_MODULE, 0, &card);
	if (ret)
		return ret;

	strscpy(card->driver, "TridentPCM", sizeof(card->driver));
	strscpy(card->shortname, "TridentPCM", sizeof(card->shortname));
	snprintf(card->longname, sizeof(card->longname),
		 "sun50iw15 Trident audio bridge");

	for (i = 0; i < ARRAY_SIZE(layout); i++) {
		ret = snd_pcm_new(card, "TridentPCM", i, layout[i].playback,
				  layout[i].capture, &tr->pcm[i]);
		if (ret)
			goto err_free;

		tr->pcm[i]->private_data = tr;
		strscpy(tr->pcm[i]->name, "Trid PCM",
			sizeof(tr->pcm[i]->name));

		if (layout[i].playback)
			snd_pcm_set_ops(tr->pcm[i],
					SNDRV_PCM_STREAM_PLAYBACK, &trid_pcm_ops);
		if (layout[i].capture)
			snd_pcm_set_ops(tr->pcm[i],
					SNDRV_PCM_STREAM_CAPTURE, &trid_pcm_ops);

		snd_pcm_set_managed_buffer_all(tr->pcm[i],
					       SNDRV_DMA_TYPE_DEV, tr->dev,
					       TRID_CH_BUF, TRID_BUFFER_MAX);
	}

	ret = snd_card_register(card);
	if (ret)
		goto err_free;

	tr->card = card;
	return 0;

err_free:
	snd_card_free(card);
	return ret;
}

static int trid_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct trid_bridge *tr;
	struct resource *res;
	int ret;

	tr = devm_kzalloc(dev, sizeof(*tr), GFP_KERNEL);
	if (!tr)
		return -ENOMEM;

	tr->dev = dev;
	spin_lock_init(&tr->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	tr->msp = devm_ioremap(dev, res->start + TRID_MSP_OFF, 0x20);
	tr->dbp = devm_ioremap(dev, res->start + TRID_DBP_OFF, 0x88);
	tr->brg = devm_ioremap(dev, res->start + TRID_BRG_OFF, 0x394);
	tr->clk = devm_ioremap(dev, res->start + TRID_CLK_OFF, 0x0f);

	if (!tr->msp || !tr->dbp || !tr->brg || !tr->clk)
		return dev_err_probe(dev, -ENOMEM,
				     "cannot map audio bridge blocks\n");

	tr->buf = dma_alloc_coherent(dev, TRID_BUF_SIZE, &tr->buf_dma,
				     GFP_KERNEL | __GFP_ZERO);
	if (!tr->buf)
		return dev_err_probe(dev, -ENOMEM,
				     "cannot allocate audio bridge buffer\n");

	/*
	 * The bridge is a bus master behind the SoC IOMMU (the DT node carries
	 * iommus), so it must be programmed with the DMA address the DMA API
	 * returned, not the CPU physical address.  That is also what the
	 * vendor did with the sg_dma_address of its dma-buf attachment.
	 */
	tr->buf_addr = tr->buf_dma;

	trid_clock_init(tr);

	ret = platform_get_irq(pdev, 0);
	if (ret < 0)
		goto err_free_buf;

	ret = devm_request_threaded_irq(dev, ret, NULL, trid_irq, IRQF_ONESHOT,
					"audio bridge irq", tr);
	if (ret)
		goto err_free_buf;

	/* Global interrupt enable, then per-channel masks from prepare(). */
	brg_write(tr, BRG_INT_EN, BRG_INT_GLOBAL);

	INIT_DELAYED_WORK(&tr->pump, trid_pump);

	ret = trid_card_new(tr);
	if (ret)
		goto err_free_buf;

	platform_set_drvdata(pdev, tr);

	dev_info(dev, "Trident audio bridge at %pR, %u/%u rings\n", res,
		 TRID_IST_CHANNELS, TRID_OST_CHANNELS);
	return 0;

err_free_buf:
	dma_free_coherent(dev, TRID_BUF_SIZE, tr->buf, tr->buf_dma);
	return ret;
}

static int trid_remove(struct platform_device *pdev)
{
	struct trid_bridge *tr = platform_get_drvdata(pdev);

	if (!tr)
		return 0;

	cancel_delayed_work_sync(&tr->pump);
	brg_write(tr, BRG_INT_EN, 0);
	brg_write(tr, BRG_INT_MASK, 0);

	if (tr->card)
		snd_card_free(tr->card);

	dma_free_coherent(tr->dev, TRID_BUF_SIZE, tr->buf, tr->buf_dma);
	return 0;
}

static const struct of_device_id trid_of_match[] = {
	{ .compatible = "vs,trid-audio-bridge" },
	{ }
};
MODULE_DEVICE_TABLE(of, trid_of_match);

static struct platform_driver trid_driver = {
	.probe = trid_probe,
	.remove = trid_remove,
	.driver = {
		.name = TRID_DRV_NAME,
		.of_match_table = trid_of_match,
	},
};
module_platform_driver(trid_driver);

MODULE_DESCRIPTION("sun50iw15 Trident audio bridge ALSA card");
MODULE_LICENSE("GPL v2");
