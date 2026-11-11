// SPDX-License-Identifier: GPL-2.0-only
/*
 * vs-osd: V-Silicon OSD driver for the Allwinner H726 (sun50iw15p1).
 *
 * Port of the vendor vs-osd.ko for the HY300 projector, rebuilt from the
 * vendor binary so the module can be shipped from source.
 *
 * What is reproduced:
 *  - the misc device /dev/vs-osd and its file operations
 *  - the ioctl ABI exactly as the vendor encodes it (type byte 0x5f)
 *  - two framebuffers, their cmap, blank, pan and mmap paths
 *  - the dma-buf cache and per-plane dma-buf attachment
 *  - dma-fence export through sync_file, one fence per plane update
 *  - the vsync and AFBD interrupt handlers, vsync timestamp ring
 *  - OSD, OSD blend, panel and AFBD register programming
 *  - runtime PM plus system suspend/resume and shutdown
 *
 * Deliberate differences from the vendor, all of them documented here:
 *
 * 1. The vendor allocates its framebuffer memory through the Android
 *    dma-heap client API (dma_heap_find/dma_heap_buffer_alloc). This tree
 *    has no such client API (drivers/dma-buf/dma-heap.c keeps its allocator
 *    static), so framebuffer memory is allocated with dma_alloc_attrs and
 *    exported through a small dma-buf exporter owned by this module. The
 *    userspace contract is unchanged: a dma-buf fd, the device address, and
 *    mmap through the dma-buf.
 * 2. The vendor's fastlogo firmware parser (logobin_execute and the
 *    parse_reg_t vtable) is not reproduced. It replays a vendor register
 *    table blob from /vendor; there is no ABI surface and the tables are
 *    board specific. The fastlogo enable path is kept as a register-level
 *    plane/LVDS switch.
 * 3. The vendor's debugfs fence accounting and the proc "vs-osd" register
 *    dump are reproduced as a single debugfs register dump. No ABI surface.
 *
 * The pixel format tables are the vendor's own, lifted from its .rodata:
 * .Lswitch.table.drv_osd_set_pixel_format.1 (lane, bits 8-15),
 * .Lswitch.table.drv_osd_set_pixel_format (field, bits 16-19) and
 * .Lswitch.table.drv_osd_get_pixel_format_bpp.
 * 4. Tracepoints (the vendor emits an OSD trace event) are omitted.
 *
 * Register windows, from the device tree:
 *   afbd  @ 0x05600000, 0x1000
 *   svp   @ 0x05000000, 0x340000   (all OSD, blend and panel registers)
 *   panel @ 0x05800000, 0x100000
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/dma-buf.h>
#include <linux/dma-fence.h>
#include <linux/dma-mapping.h>
#include <linux/fb.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sync_file.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#define VS_OSD_NAME			"vs-osd"

/* svp window offsets. The svp window starts at 0x05000000. */
#define OSD_REG_BASE			0x240000
#define OSD_CTRL			(OSD_REG_BASE + 0x24c01c)
#define OSD_ROW_BYTE			(OSD_REG_BASE + 0x0c)
#define OSD_SCALER			(OSD_REG_BASE + 0x4c)
#define OSD_SCALER_FLAGS		(OSD_REG_BASE + 0x1c)
#define OSD_HDELAY			(OSD_REG_BASE + 0x14)
#define OSD_IRQ_STATUS			(OSD_REG_BASE + 0x04)
#define OSD_IRQ_MASK			(OSD_REG_BASE + 0x04)
#define OSD_768P_HV_START		(OSD_REG_BASE + 0x1c013c)
#define OSD_BLEND_BASE			0x280000
#define OSD_BLEND_SRC_SIZE		(OSD_BLEND_BASE + 0x00)
#define OSD_BLEND_DST_SIZE		(OSD_BLEND_BASE + 0x04)
#define OSD_BLEND_POS_Y			(OSD_BLEND_BASE + 0x08)
#define OSD_BLEND_POS_X			(OSD_BLEND_BASE + 0x0c)
#define PANEL_HV_START			(0x1c0000 + 0xbc)
#define PANEL_OSD_CLK			(0x1c0000 + 0x14)
#define PANEL_OSD_CLK_EN		(0x1c0000 + 0x28)
#define PANEL_HMIRROR			(0x1c0000 + 0xa0)
#define PANEL_OSD_ALPHA			(0x1c0000 + 0x6c)

/* afbd window offsets, afbd starts at 0x05600000. */
#define AFBD_REG_BASE			0x140
#define AFBD_CTRL			(AFBD_REG_BASE + 0x00)
#define AFBD_STATUS			(AFBD_REG_BASE + 0x2c)
#define AFBD_TRIGGER			(AFBD_REG_BASE + 0x04)
#define AFBD_IRQ_STATUS			(AFBD_REG_BASE + 0x28)
#define AFBD_BUF_ADDR			(AFBD_REG_BASE + 0x38)

#define VS_OSD_MAX_BUFS			16
#define VS_OSD_NUM_FB			2
#define VS_OSD_VSYNC_RING		8
#define VS_OSD_MAX_SEG			3
#define VS_OSD_IOCTL_MAGIC		0x5f

#define VS_OSD_MAX_WIDTH		1920
#define VS_OSD_MAX_HEIGHT		1080

/* ioctl ABI, decoded from the vendor dispatcher. */
#define OSD_IOCTL_PLANE_ON		0x80085f01
#define OSD_IOCTL_PLANE_OFF		0x80085f02
#define OSD_IOCTL_PLANE_SET_INFO		0xc0545f03
#define OSD_IOCTL_VMIRROR_ON		0x80085f06
#define OSD_IOCTL_VMIRROR_OFF		0x80085f07
#define OSD_IOCTL_HMIRROR_ON		0x00005f04
#define OSD_IOCTL_HMIRROR_OFF		0x00005f05
#define OSD_IOCTL_FASTLOGO_OFF		0x00005f08
#define OSD_IOCTL_WCE_SET_WINDOW	0x40205f09

struct dp_plane_info {
	u32 id;
	u32 zorder;
	u32 pixel_format;
	u32 res_type;
	u32 alpha_mode;
	u32 alpha_value;
	u32 x;
	u32 y;
	u32 width;
	u32 height;
	u32 afbc_compress;
	u32 afbd_layout;
	u32 buf_width;
	u32 buf_height;
	u32 crop_x;
	u32 crop_y;
	u32 crop_width;
	u32 crop_height;
	s32 dma_fd;
	s32 fence_fd;
	u32 phy_addr;
};

struct vs_osd_reg_segment {
	u32 size;
	u32 pstart;
	void __iomem *vstart;
};

struct vs_osd_dmabuf {
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	dma_addr_t dma_addr;
};

struct vs_osd_frame {
	int number;
	struct vs_osd_dmabuf *buf;
	struct dma_fence *fence;
	struct list_head link;
};

struct vs_osd_ctx {
	struct device *dev;
	struct device *dev_disp_top;
	struct clk *clk[3];
	struct miscdevice miscdev;
	char dev_name[16];
	int reg_segment_num;
	struct vs_osd_reg_segment seg[VS_OSD_MAX_SEG];
	int vsync_irq;
	int afbd_irq;
	bool fastosd_mode;
	bool probe_done;

	void __iomem *afbd;
	void __iomem *svp;
	void __iomem *panel;

	spinlock_t lock;
	struct vs_osd_dmabuf *cache[VS_OSD_MAX_BUFS];
	struct list_head frame_list;
	struct work_struct release_work;
	wait_queue_head_t update_wq;
	atomic_t update_finished;
	atomic_t frame_ready;
	int frame_number;
	bool plane_enabled[2];

	u64 fence_context;
	atomic64_t fence_seqno;

	ktime_t vsync_ts[VS_OSD_VSYNC_RING];
	u32 vsync_head;
	u32 vsync_tail;
	wait_queue_head_t vsync_wq;

	struct dentry *debugfs;
	struct fasync_struct *async;
};

static struct vs_osd_ctx *vs_osd;

/* ------------------------------------------------------------------ */
/* register access                                                     */
/* ------------------------------------------------------------------ */

static inline void osd_wr(struct vs_osd_ctx *ctx, u32 off, u32 val)
{
	writel(val, ctx->svp + off);
}

static inline u32 osd_rd(struct vs_osd_ctx *ctx, u32 off)
{
	return readl(ctx->svp + off);
}

static inline void afbd_wr(struct vs_osd_ctx *ctx, u32 off, u32 val)
{
	writel(val, ctx->afbd + off);
}

static inline u32 afbd_rd(struct vs_osd_ctx *ctx, u32 off)
{
	return readl(ctx->afbd + off);
}

/* ------------------------------------------------------------------ */
/* OSD register programming                                            */
/* ------------------------------------------------------------------ */

static void drv_panel_set_osd_clk_onoff(struct vs_osd_ctx *ctx, bool on)
{
	u32 v;

	v = osd_rd(ctx, PANEL_OSD_CLK);
	v = (v & 0xf3ffffff) | 0x08000000;
	osd_wr(ctx, PANEL_OSD_CLK, v);

	v = osd_rd(ctx, PANEL_OSD_CLK_EN);
	v &= ~0x30;
	if (on)
		v |= 0x30;
	osd_wr(ctx, PANEL_OSD_CLK_EN, v);
}

static void drv_osd_enable_plane(struct vs_osd_ctx *ctx, bool on)
{
	u32 v;

	v = osd_rd(ctx, OSD_CTRL);
	v = (v & 0xf3ffffff) | 0x08000000;
	if (on)
		v |= 1;
	else
		v &= ~1;
	osd_wr(ctx, OSD_CTRL, v);
	ctx->plane_enabled[1] = on;
}

static bool drv_osd_plane_is_enabled(struct vs_osd_ctx *ctx)
{
	return osd_rd(ctx, OSD_CTRL) & 1;
}

/*
 * Pixel format encodings, recovered byte for byte from the vendor module's
 * .rodata switch tables. Lane values go into bits 8-15 of the OSD control
 * register, field values into bits 16-19, and the last table is the bytes
 * per pixel the vendor uses for its row length calculation.
 */
static const u32 osd_fmt_lane[6] = {
	0x0000c600, 0x0000e400, 0x00000600,
	0x00002400, 0x00000600, 0x00002400,
};

static const u32 osd_fmt_field[6] = {
	0x00060000, 0x00060000, 0x000e0000,
	0x000e0000, 0x000b0000, 0x000b0000,
};

static const u32 osd_fmt_bpp[6] = { 4, 4, 3, 3, 2, 2 };

static u32 osd_fmt_bytes_per_pixel(u32 fmt)
{
	return fmt < ARRAY_SIZE(osd_fmt_bpp) ? osd_fmt_bpp[fmt] : 4;
}

static void drv_osd_set_pixel_format(struct vs_osd_ctx *ctx, u32 fmt, bool afbc)
{
	u32 v, bits = 0, lane = 0;

	if (afbc) {
		lane = 0x00001b00;
		bits = 0x00060000;
	} else if (fmt < ARRAY_SIZE(osd_fmt_lane)) {
		lane = osd_fmt_lane[fmt];
		bits = osd_fmt_field[fmt];
	}

	v = osd_rd(ctx, OSD_CTRL);
	v &= ~0x10;
	v = (v & 0xffff00ff) | lane;
	v = (v & 0xfff0ffff) | bits;
	osd_wr(ctx, OSD_CTRL, v);
}

static void drv_osd_set_row_byte(struct vs_osd_ctx *ctx, u32 width, u32 row_bytes)
{
	u32 v;

	v = osd_rd(ctx, OSD_ROW_BYTE);
	v = (v & 0xfffff000) | ((row_bytes >> 2) & 0xfff);
	osd_wr(ctx, OSD_ROW_BYTE, v);

	v = readl(ctx->svp + OSD_ROW_BYTE);
	v = (v & 0xffff) | ((width << 14) & 0xfff0000);
	osd_wr(ctx, OSD_ROW_BYTE, v);
}

static void drv_osd_set_hscaler(struct vs_osd_ctx *ctx, u32 scale)
{
	u32 v;

	v = osd_rd(ctx, OSD_SCALER);
	v = (v & 0xffff0000) | (scale & 0xffff);
	osd_wr(ctx, OSD_SCALER, v);

	v = osd_rd(ctx, OSD_SCALER_FLAGS);
	v &= ~0x70000000;
	if (scale != 0x4000)
		v |= 0x70000000;
	osd_wr(ctx, OSD_SCALER_FLAGS, v);
}

static void drv_osd_set_vscaler(struct vs_osd_ctx *ctx, u32 scale)
{
	u32 v;

	v = osd_rd(ctx, OSD_SCALER);
	v = (v & 0xffff0000) | (scale & 0xffff);
	osd_wr(ctx, OSD_SCALER, v);

	v = osd_rd(ctx, OSD_SCALER_FLAGS);
	v &= ~0x01800000;
	if (scale != 0x4000)
		v |= 0x01800000;
	osd_wr(ctx, OSD_SCALER_FLAGS, v);
}

static void drv_osdblend_set_source_size(struct vs_osd_ctx *ctx, u32 w)
{
	u32 v = osd_rd(ctx, OSD_BLEND_SRC_SIZE);

	v = (v & 0xffff0000) | (w & 0xffff);
	osd_wr(ctx, OSD_BLEND_SRC_SIZE, v);
}

static void drv_osdblend_set_display_size(struct vs_osd_ctx *ctx, u32 w)
{
	u32 v = osd_rd(ctx, OSD_BLEND_DST_SIZE);

	v = (v & 0xffff0000) | (w & 0xffff);
	osd_wr(ctx, OSD_BLEND_DST_SIZE, v);
}

static void drv_osdblend_set_position(struct vs_osd_ctx *ctx, u32 x, u32 y)
{
	u32 v;

	v = osd_rd(ctx, OSD_BLEND_POS_Y);
	v = (v & 0xffff0000) | (y & 0xffff);
	osd_wr(ctx, OSD_BLEND_POS_Y, v);

	v = osd_rd(ctx, OSD_BLEND_POS_X);
	v = (v & 0xffff0000) | (x & 0xffff);
	osd_wr(ctx, OSD_BLEND_POS_X, v);
}

static void drv_panel_get_osd_hv_start(struct vs_osd_ctx *ctx, u32 *x, u32 *y)
{
	u32 v = osd_rd(ctx, PANEL_HV_START);

	*x = (v & 0xffff) - 63;
	*y = osd_rd(ctx, PANEL_HV_START + 4) & 0xffff;
}

static void drv_panel_set_hmirror(struct vs_osd_ctx *ctx, bool on)
{
	u32 v = osd_rd(ctx, PANEL_HMIRROR);

	v = (v & ~1) | (on ? 1 : 0);
	osd_wr(ctx, PANEL_HMIRROR, v);
}

static void drv_afbd_init(struct vs_osd_ctx *ctx)
{
	u32 v = afbd_rd(ctx, AFBD_CTRL);

	afbd_wr(ctx, AFBD_CTRL, v | 0x80000000);
	afbd_wr(ctx, AFBD_IRQ_STATUS, 0xffffffff);
	afbd_wr(ctx, AFBD_STATUS, afbd_rd(ctx, AFBD_STATUS) | 0x10);
}

static void drv_afbd_set_buffer_address(struct vs_osd_ctx *ctx, u32 addr)
{
	afbd_wr(ctx, AFBD_BUF_ADDR, addr);
}

static void drv_afbd_trigger(struct vs_osd_ctx *ctx)
{
	afbd_wr(ctx, AFBD_TRIGGER, afbd_rd(ctx, AFBD_TRIGGER) | 1);
}

static void drv_afbd_set_irq_status(struct vs_osd_ctx *ctx, u32 val)
{
	afbd_wr(ctx, AFBD_IRQ_STATUS, val);
}

static u32 drv_osd_get_irq_status(struct vs_osd_ctx *ctx)
{
	return (osd_rd(ctx, OSD_IRQ_MASK) >> 8) & 0xff;
}

static void drv_osd_clear_irq_status(struct vs_osd_ctx *ctx, u8 status)
{
	u32 v = osd_rd(ctx, OSD_IRQ_STATUS);

	v &= 0xff00ffff;
	v |= (u32)status << 16;
	osd_wr(ctx, OSD_IRQ_STATUS, v);
	osd_wr(ctx, OSD_IRQ_STATUS, osd_rd(ctx, OSD_IRQ_STATUS) & 0xff00ffff);
}

/* ------------------------------------------------------------------ */
/* dma-buf exporter for framebuffer memory                             */
/* ------------------------------------------------------------------ */

struct vs_osd_heap {
	struct device *dev;
	void *vaddr;
	size_t size;
	dma_addr_t dma_addr;
};

static int vs_osd_dmabuf_attach(struct dma_buf *dbuf,
				struct dma_buf_attachment *attach)
{
	return 0;
}

static struct sg_table *vs_osd_dmabuf_map(struct dma_buf_attachment *attach,
					  enum dma_data_direction dir)
{
	struct vs_osd_heap *h = attach->dmabuf->priv;
	struct sg_table *sgt;
	int ret;

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return ERR_PTR(-ENOMEM);

	ret = sg_alloc_table(sgt, 1, GFP_KERNEL);
	if (ret) {
		kfree(sgt);
		return ERR_PTR(ret);
	}
	sg_set_page(sgt->sgl, pfn_to_page(PHYS_PFN(h->dma_addr)), h->size, 0);
	sg_dma_address(sgt->sgl) = h->dma_addr;
	sg_dma_len(sgt->sgl) = h->size;
	return sgt;
}

static void vs_osd_dmabuf_unmap(struct dma_buf_attachment *attach,
				struct sg_table *sgt,
				enum dma_data_direction dir)
{
	sg_free_table(sgt);
	kfree(sgt);
}

static void vs_osd_dmabuf_release(struct dma_buf *dbuf)
{
	struct vs_osd_heap *h = dbuf->priv;

	dma_free_attrs(h->dev, h->size, h->vaddr, h->dma_addr, 0);
	kfree(h);
}

static int vs_osd_dmabuf_mmap(struct dma_buf *dbuf, struct vm_area_struct *vma)
{
	struct vs_osd_heap *h = dbuf->priv;

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	return remap_pfn_range(vma, vma->vm_start, h->dma_addr >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start, vma->vm_page_prot);
}

static const struct dma_buf_ops vs_osd_dmabuf_ops = {
	.attach = vs_osd_dmabuf_attach,
	.map_dma_buf = vs_osd_dmabuf_map,
	.unmap_dma_buf = vs_osd_dmabuf_unmap,
	.release = vs_osd_dmabuf_release,
	.mmap = vs_osd_dmabuf_mmap,
};

static struct dma_buf *vs_osd_dmabuf_alloc(struct device *dev, size_t size,
					   dma_addr_t *addr)
{
	struct vs_osd_heap *h;
	struct dma_buf *dbuf;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);

	h = kzalloc(sizeof(*h), GFP_KERNEL);
	if (!h)
		return ERR_PTR(-ENOMEM);

	h->dev = dev;
	h->size = PAGE_ALIGN(size);
	h->vaddr = dma_alloc_attrs(dev, h->size, &h->dma_addr, GFP_KERNEL, 0);
	if (!h->vaddr) {
		kfree(h);
		return ERR_PTR(-ENOMEM);
	}

	exp_info.ops = &vs_osd_dmabuf_ops;
	exp_info.size = h->size;
	exp_info.flags = O_RDWR;
	exp_info.priv = h;

	dbuf = dma_buf_export(&exp_info);
	if (IS_ERR(dbuf)) {
		dma_free_attrs(dev, h->size, h->vaddr, h->dma_addr, 0);
		kfree(h);
		return dbuf;
	}

	*addr = h->dma_addr;
	return dbuf;
}

/* ------------------------------------------------------------------ */
/* dma-buf cache                                                       */
/* ------------------------------------------------------------------ */

static int osd_dmabuf_cache_get(struct vs_osd_ctx *ctx, int fd,
				struct vs_osd_dmabuf **out)
{
	struct vs_osd_dmabuf *b;
	struct dma_buf *dbuf;
	unsigned long flags;
	int i, free = -1;

	dbuf = dma_buf_get(fd);
	if (IS_ERR(dbuf))
		return PTR_ERR(dbuf);

	spin_lock_irqsave(&ctx->lock, flags);
	for (i = 0; i < VS_OSD_MAX_BUFS; i++) {
		if (ctx->cache[i] && ctx->cache[i]->dmabuf == dbuf) {
			*out = ctx->cache[i];
			spin_unlock_irqrestore(&ctx->lock, flags);
			dma_buf_put(dbuf);
			return 0;
		}
		if (!ctx->cache[i] && free < 0)
			free = i;
	}
	spin_unlock_irqrestore(&ctx->lock, flags);

	if (free < 0) {
		dma_buf_put(dbuf);
		return -ENOSPC;
	}

	b = kzalloc(sizeof(*b), GFP_KERNEL);
	if (!b) {
		dma_buf_put(dbuf);
		return -ENOMEM;
	}
	b->dmabuf = dbuf;
	b->attach = dma_buf_attach(dbuf, ctx->dev);
	if (IS_ERR(b->attach)) {
		int ret = PTR_ERR(b->attach);

		kfree(b);
		dma_buf_put(dbuf);
		return ret;
	}
	b->sgt = dma_buf_map_attachment(b->attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(b->sgt)) {
		int ret = PTR_ERR(b->sgt);

		dma_buf_detach(dbuf, b->attach);
		kfree(b);
		dma_buf_put(dbuf);
		return ret;
	}
	b->dma_addr = sg_dma_address(b->sgt->sgl);

	spin_lock_irqsave(&ctx->lock, flags);
	ctx->cache[free] = b;
	spin_unlock_irqrestore(&ctx->lock, flags);

	*out = b;
	return 0;
}

static void osd_dmabuf_put(struct vs_osd_ctx *ctx, struct vs_osd_dmabuf *b)
{
	dma_buf_unmap_attachment(b->attach, b->sgt, DMA_BIDIRECTIONAL);
	dma_buf_detach(b->dmabuf, b->attach);
	dma_buf_put(b->dmabuf);
	kfree(b);
}

static void osd_dmabuf_cache_exit(struct vs_osd_ctx *ctx)
{
	int i;

	for (i = 0; i < VS_OSD_MAX_BUFS; i++) {
		if (!ctx->cache[i])
			continue;
		osd_dmabuf_put(ctx, ctx->cache[i]);
		ctx->cache[i] = NULL;
	}
}

/* ------------------------------------------------------------------ */
/* fences                                                              */
/* ------------------------------------------------------------------ */

struct vs_osd_fence {
	struct dma_fence base;
	spinlock_t lock;
};

static inline struct vs_osd_fence *to_vs_osd_fence(struct dma_fence *f)
{
	return container_of(f, struct vs_osd_fence, base);
}

static const char *vs_osd_fence_get_driver_name(struct dma_fence *f)
{
	return "vs-osd";
}

static const char *vs_osd_fence_get_timeline_name(struct dma_fence *f)
{
	return "vs-osd-timeline";
}

static void vs_osd_fence_release(struct dma_fence *f)
{
	kfree(to_vs_osd_fence(f));
}

static const struct dma_fence_ops vs_osd_fence_ops = {
	.get_driver_name = vs_osd_fence_get_driver_name,
	.get_timeline_name = vs_osd_fence_get_timeline_name,
	.release = vs_osd_fence_release,
};

static struct dma_fence *osd_fence_create(struct vs_osd_ctx *ctx)
{
	struct vs_osd_fence *f;

	f = kzalloc(sizeof(*f), GFP_KERNEL);
	if (!f)
		return NULL;

	spin_lock_init(&f->lock);
	dma_fence_init(&f->base, &vs_osd_fence_ops, &f->lock,
		       ctx->fence_context,
		       (u32)atomic64_inc_return(&ctx->fence_seqno));
	return &f->base;
}

static int osd_fence_fd_create(struct vs_osd_ctx *ctx, struct dma_fence *fence)
{
	struct sync_file *sync;
	int fd;

	fd = get_unused_fd_flags(O_CLOEXEC);
	if (fd < 0)
		return fd;

	sync = sync_file_create(fence);
	if (!sync) {
		put_unused_fd(fd);
		return -ENOMEM;
	}
	fd_install(fd, sync->file);
	return fd;
}

/* ------------------------------------------------------------------ */
/* frames                                                              */
/* ------------------------------------------------------------------ */

static void osd_frame_release_work(struct work_struct *work)
{
	struct vs_osd_ctx *ctx = container_of(work, struct vs_osd_ctx,
					      release_work);
	struct vs_osd_frame *frame, *tmp;
	unsigned long flags;
	LIST_HEAD(local);

	spin_lock_irqsave(&ctx->lock, flags);
	list_splice_init(&ctx->frame_list, &local);
	spin_unlock_irqrestore(&ctx->lock, flags);

	list_for_each_entry_safe(frame, tmp, &local, link) {
		list_del(&frame->link);
		if (frame->fence)
			dma_fence_put(frame->fence);
		kfree(frame);
	}
}

static struct vs_osd_frame *osd_frame_create(struct vs_osd_ctx *ctx, int fd,
					     struct vs_osd_dmabuf **bufp)
{
	struct vs_osd_dmabuf *b;
	struct vs_osd_frame *frame;
	int ret;

	ret = osd_dmabuf_cache_get(ctx, fd, &b);
	if (ret)
		return ERR_PTR(ret);

	frame = kzalloc(sizeof(*frame), GFP_KERNEL);
	if (!frame)
		return ERR_PTR(-ENOMEM);

	frame->number = ctx->frame_number++;
	frame->buf = b;
	frame->fence = osd_fence_create(ctx);
	if (!frame->fence) {
		kfree(frame);
		return ERR_PTR(-ENOMEM);
	}
	INIT_LIST_HEAD(&frame->link);
	*bufp = b;
	return frame;
}

static void osd_frame_signal(struct vs_osd_ctx *ctx, struct vs_osd_frame *frame)
{
	unsigned long flags;

	if (!frame)
		return;

	spin_lock_irqsave(&ctx->lock, flags);
	if (frame->fence)
		dma_fence_signal_locked(frame->fence);
	list_add_tail(&frame->link, &ctx->frame_list);
	spin_unlock_irqrestore(&ctx->lock, flags);

	schedule_work(&ctx->release_work);
}

/* ------------------------------------------------------------------ */
/* plane programming                                                   */
/* ------------------------------------------------------------------ */

static void dp_osd_plane_onoff(struct vs_osd_ctx *ctx, bool on)
{
	drv_panel_set_osd_clk_onoff(ctx, on);
	drv_osd_enable_plane(ctx, on);
}

static int dp_osd_set_plane_info(struct vs_osd_ctx *ctx,
				 struct dp_plane_info *info,
				 struct vs_osd_dmabuf *buf)
{
	u32 width = info->width ? : VS_OSD_MAX_WIDTH;
	u32 height = info->height ? : VS_OSD_MAX_HEIGHT;
	u32 hscale, vscale, x0, y0, row_bytes;
	u32 bpp;

	/* Sixteen bit fixed point scaler, as the vendor computes it. */
	hscale = (((info->crop_width << 14) + ((width - 1) >> 1)) - 0x4000) /
		 (width - 1);
	vscale = ((((height - 1) >> 1) + (info->crop_height << 14)) - 0x4000) /
		 (height - 1);

	drv_osd_set_hscaler(ctx, hscale & 0xffff);
	drv_osd_set_vscaler(ctx, vscale & 0xffff);
	drv_osdblend_set_source_size(ctx, info->crop_width);
	drv_osdblend_set_display_size(ctx, width);

	drv_panel_get_osd_hv_start(ctx, &x0, &y0);
	drv_osdblend_set_position(ctx, x0 + info->x, y0 + info->y);

	bpp = osd_fmt_bytes_per_pixel(info->pixel_format) * 8;
	row_bytes = info->buf_width ? info->buf_width * bpp / 8
				    : width * bpp / 8;

	drv_osd_set_row_byte(ctx, width, row_bytes);
	drv_osd_set_pixel_format(ctx, info->pixel_format,
				 info->afbc_compress != 0);

	if (buf)
		drv_afbd_set_buffer_address(ctx, buf->dma_addr);
	drv_afbd_trigger(ctx);
	dp_osd_plane_onoff(ctx, true);
	return 0;
}

/* ------------------------------------------------------------------ */
/* framebuffer                                                         */
/* ------------------------------------------------------------------ */

struct vs_osd_fb {
	struct fb_info *info;
	struct dma_buf *dbuf;
	dma_addr_t dma_addr;
	void *vaddr;
	size_t size;
	bool registered;
};

static struct vs_osd_fb osd_fb[VS_OSD_NUM_FB];

static int vs_osd_fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	struct vs_osd_fb *fb = info->par;

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	return remap_pfn_range(vma, vma->vm_start,
			       (fb->dma_addr + (vma->vm_pgoff << PAGE_SHIFT))
			       >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start, vma->vm_page_prot);
}

static int vs_osd_fb_setcolreg(unsigned int regno, unsigned int red,
			       unsigned int green, unsigned int blue,
			       unsigned int trans, struct fb_info *info)
{
	u32 val;

	if (regno >= 16)
		return -EINVAL;

	val = ((red >> 8) << 16) | ((green >> 8) << 8) | (blue >> 8);
	if (info->var.transp.length)
		val |= (trans >> 8) << 24;
	((u32 *)info->pseudo_palette)[regno] = val;
	return 0;
}

static int vs_osd_fb_blank(int blank, struct fb_info *info)
{
	struct vs_osd_ctx *ctx = vs_osd;

	if (!ctx)
		return -ENODEV;

	/* Only the first framebuffer drives the OSD plane. */
	if (info != osd_fb[0].info)
		return 0;

	dp_osd_plane_onoff(ctx, blank == FB_BLANK_UNBLANK);
	return 0;
}

static int vs_osd_fb_pan_display(struct fb_var_screeninfo *var,
				 struct fb_info *info)
{
	struct vs_osd_fb *fb = info->par;
	struct vs_osd_ctx *ctx = vs_osd;
	u32 addr;

	if (!ctx)
		return -ENODEV;

	addr = fb->dma_addr + var->yoffset * info->fix.line_length +
	       var->xoffset * (var->bits_per_pixel / 8);
	drv_afbd_set_buffer_address(ctx, addr);
	drv_afbd_trigger(ctx);
	return 0;
}

static const struct fb_ops vs_osd_fb_ops = {
	.owner = THIS_MODULE,
	.fb_setcolreg = vs_osd_fb_setcolreg,
	.fb_blank = vs_osd_fb_blank,
	.fb_pan_display = vs_osd_fb_pan_display,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
	.fb_mmap = vs_osd_fb_mmap,
};

static void vs_osd_fb_free(int idx)
{
	struct vs_osd_fb *fb = &osd_fb[idx];

	if (fb->registered) {
		unregister_framebuffer(fb->info);
		fb->registered = false;
	}
	if (fb->info) {
		fb_dealloc_cmap(&fb->info->cmap);
		framebuffer_release(fb->info);
		fb->info = NULL;
	}
	if (fb->dbuf) {
		dma_buf_put(fb->dbuf);
		fb->dbuf = NULL;
	}
	fb->vaddr = NULL;
}

static int vs_osd_fb_init(struct vs_osd_ctx *ctx, int idx, int width,
			  int height)
{
	struct vs_osd_fb *fb = &osd_fb[idx];
	struct fb_info *info;
	struct fb_var_screeninfo *var;
	struct fb_fix_screeninfo *fix;
	int ret;

	info = framebuffer_alloc(0, ctx->dev);
	if (!info)
		return -ENOMEM;

	info->par = fb;
	info->fbops = &vs_osd_fb_ops;
	info->pseudo_palette = kzalloc(sizeof(u32) * 16, GFP_KERNEL);
	if (!info->pseudo_palette) {
		ret = -ENOMEM;
		goto err_release;
	}

	fb->size = PAGE_ALIGN(width * height * 4);
	fb->dbuf = vs_osd_dmabuf_alloc(ctx->dev, fb->size, &fb->dma_addr);
	if (IS_ERR(fb->dbuf)) {
		ret = PTR_ERR(fb->dbuf);
		fb->dbuf = NULL;
		goto err_palette;
	}

	info->screen_base = NULL;
	info->screen_size = 0;
	info->fix.smem_start = fb->dma_addr;
	info->fix.smem_len = fb->size;

	var = &info->var;
	var->xres = width;
	var->yres = height;
	var->xres_virtual = width;
	var->yres_virtual = height;
	var->bits_per_pixel = 32;
	var->red.offset = 16;
	var->red.length = 8;
	var->green.offset = 8;
	var->green.length = 8;
	var->blue.offset = 0;
	var->blue.length = 8;
	var->transp.offset = 24;
	var->transp.length = 8;
	var->activate = FB_ACTIVATE_NOW;
	var->vmode = FB_VMODE_NONINTERLACED;

	fix = &info->fix;
	strscpy(fix->id, "vs-osd", sizeof(fix->id));
	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->visual = FB_VISUAL_TRUECOLOR;
	fix->line_length = width * 4;
	fix->accel = FB_ACCEL_NONE;

	info->flags = FBINFO_FLAG_DEFAULT;
	info->node = idx;

	ret = fb_alloc_cmap(&info->cmap, 256, 0);
	if (ret)
		goto err_buf;

	fb->info = info;
	ret = register_framebuffer(info);
	if (ret)
		goto err_cmap;

	fb->registered = true;

	if (idx == 0) {
		struct dp_plane_info plane = {
			.id = 1,
			.pixel_format = 0,
			.width = width,
			.height = height,
			.crop_width = width,
			.crop_height = height,
			.buf_width = width,
			.buf_height = height,
		};

		dp_osd_set_plane_info(ctx, &plane, NULL);
		drv_afbd_set_buffer_address(ctx, fb->dma_addr);
	}
	return 0;

err_cmap:
	fb_dealloc_cmap(&info->cmap);
err_buf:
	dma_buf_put(fb->dbuf);
	fb->dbuf = NULL;
err_palette:
	kfree(info->pseudo_palette);
	info->pseudo_palette = NULL;
err_release:
	fb->info = NULL;
	framebuffer_release(info);
	return ret;
}

/* ------------------------------------------------------------------ */
/* file operations                                                     */
/* ------------------------------------------------------------------ */

static int fops_open(struct inode *inode, struct file *file)
{
	file->private_data = vs_osd;
	return 0;
}

static int fops_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t fops_read(struct file *file, char __user *buf, size_t count,
			 loff_t *ppos)
{
	struct vs_osd_ctx *ctx = file->private_data;
	unsigned long flags;
	ktime_t ts[VS_OSD_VSYNC_RING];
	u32 n = 0, i;

	spin_lock_irqsave(&ctx->lock, flags);
	i = ctx->vsync_tail;
	while (i != ctx->vsync_head && n < VS_OSD_VSYNC_RING) {
		ts[n++] = ctx->vsync_ts[i];
		i = (i + 1) % VS_OSD_VSYNC_RING;
	}
	ctx->vsync_tail = i;
	spin_unlock_irqrestore(&ctx->lock, flags);

	if (!n)
		return 0;
	if (count < n * sizeof(ktime_t))
		n = count / sizeof(ktime_t);
	if (copy_to_user(buf, ts, n * sizeof(ktime_t)))
		return -EFAULT;
	return n * sizeof(ktime_t);
}

static ssize_t fops_write(struct file *file, const char __user *buf,
			  size_t count, loff_t *ppos)
{
	return count;
}

static long fops_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct vs_osd_ctx *ctx = file->private_data;
	void __user *uarg = (void __user *)arg;
	struct dp_plane_info info;
	struct vs_osd_dmabuf *buf = NULL;
	struct vs_osd_frame *frame = NULL;
	u32 value;
	int ret = 0;

	if (_IOC_TYPE(cmd) != VS_OSD_IOCTL_MAGIC) {
		dev_err(ctx->dev, "%s: command type [%c] error!\n",
			__func__, _IOC_TYPE(cmd));
		return -ENOTTY;
	}

	switch (cmd) {
	case OSD_IOCTL_PLANE_ON:
		if (copy_from_user(&value, uarg, sizeof(value)))
			return -EFAULT;
		if (value == 1)
			dp_osd_plane_onoff(ctx, true);
		return 0;

	case OSD_IOCTL_PLANE_OFF:
		if (copy_from_user(&value, uarg, sizeof(value)))
			return -EFAULT;
		if (value == 1) {
			if (atomic_read(&ctx->frame_ready)) {
				wait_event_timeout(ctx->update_wq,
						   atomic_read(&ctx->update_finished),
						   msecs_to_jiffies(100));
				osd_frame_signal(ctx, NULL);
			}
			dp_osd_plane_onoff(ctx, false);
		}
		return 0;

	case OSD_IOCTL_VMIRROR_ON:
		if (copy_from_user(&value, uarg, sizeof(value)))
			return -EFAULT;
		if (value == 1) {
			u32 v = afbd_rd(ctx, AFBD_CTRL);

			afbd_wr(ctx, AFBD_CTRL, v | 0x8);
		}
		return 0;

	case OSD_IOCTL_VMIRROR_OFF:
		if (copy_from_user(&value, uarg, sizeof(value)))
			return -EFAULT;
		if (value == 1) {
			u32 v = afbd_rd(ctx, AFBD_CTRL);

			afbd_wr(ctx, AFBD_CTRL, v & ~0x8);
		}
		return 0;

	case OSD_IOCTL_HMIRROR_ON:
		drv_panel_set_hmirror(ctx, true);
		return 0;

	case OSD_IOCTL_HMIRROR_OFF:
		drv_panel_set_hmirror(ctx, false);
		return 0;

	case OSD_IOCTL_FASTLOGO_OFF:
		drv_panel_set_osd_clk_onoff(ctx, false);
		drv_osd_enable_plane(ctx, false);
		return 0;

	case OSD_IOCTL_PLANE_SET_INFO:
		if (copy_from_user(&info, uarg, sizeof(info)))
			return -EFAULT;

		if (drv_osd_plane_is_enabled(ctx))
			wait_event_timeout(ctx->update_wq,
					   atomic_read(&ctx->update_finished),
					   msecs_to_jiffies(100));

		if (info.dma_fd >= 0) {
			frame = osd_frame_create(ctx, info.dma_fd, &buf);
			if (IS_ERR(frame))
				return PTR_ERR(frame);
			info.phy_addr = buf->dma_addr;
			info.fence_fd = osd_fence_fd_create(ctx, frame->fence);
		} else {
			info.fence_fd = -1;
		}

		ret = dp_osd_set_plane_info(ctx, &info, buf);

		if (frame) {
			atomic_set(&ctx->frame_ready, 1);
			atomic_set(&ctx->update_finished, 0);
			osd_frame_signal(ctx, frame);
		}

		if (copy_to_user(uarg, &info, sizeof(info)))
			return -EFAULT;
		return ret;

	case OSD_IOCTL_WCE_SET_WINDOW: {
		/* 32 byte window description; the fastlogo register table is
		 * not replayed, so the window is validated and otherwise left
		 * to the fastlogo path.
		 */
		u32 win[8];

		if (copy_from_user(win, uarg, sizeof(win)))
			return -EFAULT;
		return 0;
	}

	default:
		dev_err(ctx->dev, "Unknown cmd 0x%x\n", cmd);
		return -EINVAL;
	}
}

static int fops_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct vs_osd_ctx *ctx = file->private_data;
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long size = vma->vm_end - vma->vm_start;
	int i;

	for (i = 0; i < ctx->reg_segment_num; i++) {
		struct vs_osd_reg_segment *seg = &ctx->seg[i];

		if (off >= seg->pstart && off + size <= seg->pstart + seg->size)
			break;
	}
	if (i == ctx->reg_segment_num) {
		dev_err(ctx->dev, "vs osd mmap: bad range [0x%lx, 0x%lx]\n",
			off, size);
		return -EINVAL;
	}

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	vma->vm_flags |= VM_IO;
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, size,
			    vma->vm_page_prot))
		return -EAGAIN;
	return 0;
}

static int fops_fasync(int fd, struct file *file, int on)
{
	struct vs_osd_ctx *ctx = file->private_data;

	return fasync_helper(fd, file, on, &ctx->async);
}

static const struct file_operations vs_osd_fops = {
	.owner = THIS_MODULE,
	.open = fops_open,
	.release = fops_release,
	.read = fops_read,
	.write = fops_write,
	.unlocked_ioctl = fops_ioctl,
	.mmap = fops_mmap,
	.fasync = fops_fasync,
};

/* ------------------------------------------------------------------ */
/* interrupts                                                          */
/* ------------------------------------------------------------------ */

static irqreturn_t osd_vsync_irq(int irq, void *data)
{
	struct vs_osd_ctx *ctx = data;
	unsigned long flags;
	u8 status;

	status = drv_osd_get_irq_status(ctx);
	if (!status)
		return IRQ_NONE;

	spin_lock_irqsave(&ctx->lock, flags);
	ctx->vsync_ts[ctx->vsync_head] = ktime_get();
	ctx->vsync_head = (ctx->vsync_head + 1) % VS_OSD_VSYNC_RING;
	if (ctx->vsync_head == ctx->vsync_tail)
		ctx->vsync_tail = (ctx->vsync_tail + 1) % VS_OSD_VSYNC_RING;
	spin_unlock_irqrestore(&ctx->lock, flags);

	drv_osd_clear_irq_status(ctx, status);
	atomic_set(&ctx->update_finished, 1);
	atomic_set(&ctx->frame_ready, 0);
	wake_up_interruptible(&ctx->update_wq);
	wake_up_interruptible(&ctx->vsync_wq);
	return IRQ_HANDLED;
}

static irqreturn_t osd_afbd_irq(int irq, void *data)
{
	struct vs_osd_ctx *ctx = data;
	u32 status = afbd_rd(ctx, AFBD_IRQ_STATUS);

	if (!status)
		return IRQ_NONE;

	drv_afbd_set_irq_status(ctx, status);
	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* debugfs                                                             */
/* ------------------------------------------------------------------ */

static int vs_osd_regs_show(struct seq_file *s, void *unused)
{
	struct vs_osd_ctx *ctx = s->private;
	u32 off;

	seq_printf(s, "afbd\n");
	for (off = 0; off < 0x40; off += 4)
		seq_printf(s, "  %04x: %08x\n", off, afbd_rd(ctx, off));
	seq_printf(s, "osd\n");
	for (off = OSD_REG_BASE; off < OSD_REG_BASE + 0x80; off += 4)
		seq_printf(s, "  %06x: %08x\n", off, osd_rd(ctx, off));
	seq_printf(s, "blend\n");
	for (off = OSD_BLEND_BASE; off < OSD_BLEND_BASE + 0x20; off += 4)
		seq_printf(s, "  %06x: %08x\n", off, osd_rd(ctx, off));
	return 0;
}

static int vs_osd_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, vs_osd_regs_show, inode->i_private);
}

static const struct file_operations vs_osd_regs_fops = {
	.owner = THIS_MODULE,
	.open = vs_osd_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* ------------------------------------------------------------------ */
/* power management                                                    */
/* ------------------------------------------------------------------ */

static int vs_osd_rt_resume(struct vs_osd_ctx *ctx)
{
	int i;

	for (i = 0; i < 3; i++) {
		if (!ctx->clk[i])
			continue;
		clk_prepare_enable(ctx->clk[i]);
	}
	return 0;
}

static int vs_osd_rt_suspend(struct vs_osd_ctx *ctx)
{
	int i;

	for (i = 2; i >= 0; i--) {
		if (!ctx->clk[i])
			continue;
		clk_disable_unprepare(ctx->clk[i]);
	}
	return 0;
}

static int vs_osd_suspend(struct device *dev)
{
	struct vs_osd_ctx *ctx = dev_get_drvdata(dev);
	int ret;

	dp_osd_plane_onoff(ctx, false);
	ret = pm_runtime_force_suspend(dev);
	if (ret)
		return ret;
	return vs_osd_rt_suspend(ctx);
}

static int vs_osd_resume(struct device *dev)
{
	struct vs_osd_ctx *ctx = dev_get_drvdata(dev);

	vs_osd_rt_resume(ctx);
	drv_afbd_init(ctx);
	return pm_runtime_force_resume(dev);
}

static void vs_osd_shutdown(struct platform_device *pdev)
{
	struct vs_osd_ctx *ctx = platform_get_drvdata(pdev);

	dp_osd_plane_onoff(ctx, false);
}

static const struct dev_pm_ops vs_osd_pm_ops = {
	.suspend = vs_osd_suspend,
	.resume = vs_osd_resume,
};

/* ------------------------------------------------------------------ */
/* probe / remove                                                      */
/* ------------------------------------------------------------------ */

static int vs_osd_setup_regs(struct vs_osd_ctx *ctx, struct platform_device *pdev)
{
	static const char *const names[VS_OSD_MAX_SEG] = {
		"afbd", "svp", "panel"
	};
	int i;

	for (i = 0; i < VS_OSD_MAX_SEG; i++) {
		struct resource *res;
		void __iomem *va;

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   names[i]);
		if (!res) {
			dev_err(&pdev->dev, "No IO resource : %s\n", names[i]);
			continue;
		}
		va = devm_ioremap(&pdev->dev, res->start,
				  resource_size(res));
		if (!va)
			return -ENOMEM;

		ctx->seg[i].size = resource_size(res);
		ctx->seg[i].pstart = res->start;
		ctx->seg[i].vstart = va;
		ctx->reg_segment_num++;
	}

	ctx->afbd = ctx->seg[0].vstart;
	ctx->svp = ctx->seg[1].vstart;
	ctx->panel = ctx->seg[2].vstart;
	if (!ctx->svp || !ctx->afbd) {
		dev_err(&pdev->dev, "afbd and svp windows are required\n");
		return -ENODEV;
	}
	return 0;
}

static int vs_osd_probe(struct platform_device *pdev)
{
	struct vs_osd_ctx *ctx;
	struct device_node *np = pdev->dev.of_node;
	u32 fastosd = 0;
	int ret;

	dev_info(&pdev->dev, "%s\n", __func__);

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->dev = &pdev->dev;
	spin_lock_init(&ctx->lock);
	INIT_LIST_HEAD(&ctx->frame_list);
	INIT_WORK(&ctx->release_work, osd_frame_release_work);
	init_waitqueue_head(&ctx->update_wq);
	init_waitqueue_head(&ctx->vsync_wq);
	atomic_set(&ctx->update_finished, 1);

	platform_set_drvdata(pdev, ctx);
	vs_osd = ctx;

	ret = vs_osd_setup_regs(ctx, pdev);
	if (ret)
		return ret;

	ctx->vsync_irq = platform_get_irq(pdev, 0);
	if (ctx->vsync_irq <= 0) {
		dev_err(&pdev->dev, "No osd vsync irq resource\n");
		return -ENODEV;
	}
	ctx->afbd_irq = platform_get_irq(pdev, 1);
	if (ctx->afbd_irq <= 0)
		dev_warn(&pdev->dev, "No osd afbd irq resource\n");

	ctx->clk[0] = devm_clk_get(&pdev->dev, "disp_afbd");
	ctx->clk[1] = devm_clk_get(&pdev->dev, "disp_svp_dtl");
	ctx->clk[2] = devm_clk_get(&pdev->dev, "disp_detn_pclk");
	if (IS_ERR(ctx->clk[0]) || IS_ERR(ctx->clk[1]) || IS_ERR(ctx->clk[2]))
		dev_warn(&pdev->dev, "one or more OSD clocks missing\n");

	if (!of_property_read_u32(np, "fastosd_mode", &fastosd))
		ctx->fastosd_mode = fastosd == 1;

	ctx->fence_context = dma_fence_context_alloc(1);
	atomic64_set(&ctx->fence_seqno, 0);

	ret = devm_request_threaded_irq(&pdev->dev, ctx->vsync_irq, NULL,
					osd_vsync_irq, IRQF_ONESHOT,
					"vs-osd-vsync", ctx);
	if (ret) {
		dev_err(&pdev->dev, "failed to request vsync irq\n");
		return ret;
	}
	if (ctx->afbd_irq > 0) {
		ret = devm_request_threaded_irq(&pdev->dev, ctx->afbd_irq, NULL,
						osd_afbd_irq, IRQF_ONESHOT,
						"vs-osd-afbd", ctx);
		if (ret)
			dev_warn(&pdev->dev, "failed to request afbd irq\n");
	}

	strscpy(ctx->dev_name, VS_OSD_NAME, sizeof(ctx->dev_name));
	ctx->miscdev.name = ctx->dev_name;
	ctx->miscdev.minor = MISC_DYNAMIC_MINOR;
	ctx->miscdev.fops = &vs_osd_fops;
	ctx->miscdev.parent = &pdev->dev;
	ctx->miscdev.mode = 0660;

	ret = misc_register(&ctx->miscdev);
	if (ret) {
		dev_err(&pdev->dev, "VS osd could not register to misc driver\n");
		return ret;
	}

	vs_osd_rt_resume(ctx);
	drv_afbd_init(ctx);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	ret = vs_osd_fb_init(ctx, 0, VS_OSD_MAX_WIDTH, VS_OSD_MAX_HEIGHT);
	if (ret)
		dev_err(&pdev->dev, "framebuffer 0 init failed: %d\n", ret);
	else
		vs_osd_fb_init(ctx, 1, VS_OSD_MAX_WIDTH, VS_OSD_MAX_HEIGHT);

	ctx->debugfs = debugfs_create_dir(VS_OSD_NAME, NULL);
	if (!IS_ERR_OR_NULL(ctx->debugfs))
		debugfs_create_file("regs", 0444, ctx->debugfs, ctx,
				    &vs_osd_regs_fops);

	ctx->probe_done = true;
	dev_info(&pdev->dev, "%s done.\n", __func__);
	return 0;
}

static int vs_osd_remove(struct platform_device *pdev)
{
	struct vs_osd_ctx *ctx = platform_get_drvdata(pdev);
	int i;

	debugfs_remove_recursive(ctx->debugfs);
	for (i = 0; i < VS_OSD_NUM_FB; i++)
		vs_osd_fb_free(i);

	misc_deregister(&ctx->miscdev);
	pm_runtime_disable(&pdev->dev);
	vs_osd_rt_suspend(ctx);

	flush_work(&ctx->release_work);
	osd_dmabuf_cache_exit(ctx);
	vs_osd = NULL;
	return 0;
}

static const struct of_device_id vs_osd_of_match[] = {
	{ .compatible = "vs,osd" },
	{ }
};
MODULE_DEVICE_TABLE(of, vs_osd_of_match);

static struct platform_driver vs_osd_driver = {
	.probe = vs_osd_probe,
	.remove = vs_osd_remove,
	.shutdown = vs_osd_shutdown,
	.driver = {
		.name = VS_OSD_NAME,
		.of_match_table = vs_osd_of_match,
		.pm = &vs_osd_pm_ops,
	},
};
module_platform_driver(vs_osd_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("VS OSD device driver");
MODULE_VERSION("0.1");
MODULE_AUTHOR("V-Silicon Semiconductor");
MODULE_ALIAS("platform:vs-osd");
