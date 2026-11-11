// SPDX-License-Identifier: GPL-2.0
/*
 * sunxi_ksc - Allwinner KSC scaler driver (sun50iw15p1)
 *
 * Reconstruction of the vendor module sunxi_ksc.ko for kernel 5.15.167.
 * The register layout, the parameter ABI (struct sunxi_ksc_para, 340 bytes)
 * and the exported symbol ksc_online_enable follow the vendor binary.
 *
 * The block scales the display/capture image through a front end that reads
 * a source frame and a back end that writes a destination frame. Frames come
 * either from the display pipeline (online mode, mode == 0) or from a dma-buf
 * handed in by userspace (offline, mode != 0).
 */

#include <linux/build_bug.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

/* ------------------------------------------------------------------------- */
/* register map (dword index into the 0x5900000 window)                      */
/* ------------------------------------------------------------------------- */

#define KSC_R_TOP		0
#define KSC_R_FUNC		1
#define KSC_R_INTR		2
#define KSC_R_VERSION		3
#define KSC_R_RESET		4
#define KSC_R_INTR_EN		5
#define KSC_R_STATUS		6
#define KSC_R_SRC2B0_STRIDE	7
#define KSC_R_SRC8B0_STRIDE	8
#define KSC_R_SRC2B1_STRIDE	9
#define KSC_R_SRC8B1_STRIDE	10
#define KSC_R_DST2B_STRIDE	11
#define KSC_R_DST8B_STRIDE	12
#define KSC_R_LUT_STRIDE	13
#define KSC_R_MODE0_HW		14
#define KSC_R_IN_SIZE		15
#define KSC_R_DST_SIZE		16
#define KSC_R_SCALE_RATIO	17
#define KSC_R_ROI_HOR		18
#define KSC_R_ROI_VER		19
#define KSC_R_SRC0_HIGH8	20
#define KSC_R_SRC_LUM0		21
#define KSC_R_SRC_CHM0		22
#define KSC_R_SRC_PIX0		23
#define KSC_R_SRC1_HIGH8	24
#define KSC_R_SRC_LUM1		25
#define KSC_R_SRC_CHM1		26
#define KSC_R_SRC_PIX1		27
#define KSC_R_DST_HIGH8		28
#define KSC_R_DST_LUM		29
#define KSC_R_DST_CHM		30
#define KSC_R_DST_PIX		31
#define KSC_R_LUT_HIGH8		32
#define KSC_R_LUT_ADDR		33
#define KSC_R_BLUR1		34
#define KSC_R_BLUR0		35
#define KSC_R_INTERP_HIGH8	36
#define KSC_R_INTERP_ADDR	37
#define KSC_R_DNSCAL_HIGH8	38
#define KSC_R_DNSCAL_ADDR	39
#define KSC_R_FE_POS		40
#define KSC_R_BE_POS		41
#define KSC_R_CACHE_POS		42
#define KSC_R_PIX_OUT		43
#define KSC_R_MODE1_CFG		44
#define KSC_R_CROP_OFFSET	45
#define KSC_R_SCALE_CROP	46
#define KSC_R_OUT_POS		47
#define KSC_R_DEF_PIX		48
#define KSC_R_DETAIL		49
#define KSC_R_FILT_SEL		50
#define KSC_R_AA_THR		51
#define KSC_R_AA_STR		52
#define KSC_R_DIR_RSID		53
#define KSC_R_DIR_MAX_RSID	54
#define KSC_R_PHASE		55
#define KSC_R_DEF_RGB		56
#define KSC_R_MEM_REQ		57
#define KSC_R_M1_LUM		58
#define KSC_R_M1_CHM		59
#define KSC_R_M1_PIX		60
#define KSC_R_DEBUG0		61
#define KSC_R_DEBUG1		62
#define KSC_R_DEBUG2		63
#define KSC_R_DEBUG3		64

#define KSC_MAX_CLKS	4
#define KSC_LUT_SIZE	0x100000
#define KSC_NAME	"ksc"

/* ------------------------------------------------------------------------- */
/* parameter ABI                                                             */
/* ------------------------------------------------------------------------- */

struct ksc_pq_para {
	unsigned char data[40];
};

struct lut_para {
	int lut_stride;
	unsigned short roi_x_min;
	unsigned short roi_y_min;
	unsigned short roi_x_max;
	unsigned short roi_y_max;
	unsigned short roi_x_max_non_ali;
	unsigned short roi_y_max_non_ali;
	unsigned short roi_w_min;
	unsigned short roi_w_max;
	unsigned short roi_h_min;
	unsigned short roi_h_max;
};

struct ksc_image {
	int w;
	int h;
	int w_align;
	int h_align;
	unsigned char bit_depth;
	unsigned long pdata[4];
	int stride[4];
	int size;
	int fd;
	int pix_fmt;
};

struct sunxi_ksc_para {
	bool ksc_en;
	bool scaler_en;
	bool bypass;
	bool perf_mode_en;
	bool online_wb_en;
	int mode;
	unsigned char crop_en;
	unsigned short crop_x;
	unsigned short crop_y;
	unsigned short crop_w;
	unsigned short crop_h;
	int pos_dst_tl_x;
	int pos_dst_tl_y;
	int pos_dst_tr_x;
	int pos_dst_tr_y;
	int pos_dst_bl_x;
	int pos_dst_bl_y;
	int pos_dst_br_x;
	int pos_dst_br_y;
	int src_w;
	int src_h;
	int dns_w;
	int dns_h;
	int dst_w;
	int dst_h;
	int file_fmt;
	int pix_fmt;
	int wb_fmt;
	int csc_fmt;
	unsigned char bit_depth;
	unsigned char rot_mode;
	int rot_angle;
	short view_angle;
	unsigned char flip_h;
	unsigned char flip_v;
	short ost_h;
	short ost_v;
	short phase_h;
	short phase_v;
	unsigned int stitch_mode_flag;
	unsigned int ksc_default_rgb_ch2;
	unsigned int ksc_default_rgb_ch1;
	unsigned int ksc_default_rgb_ch0;
	int scale_ratio_x;
	int scale_ratio_y;
	struct ksc_pq_para pq_para;
	struct lut_para lut;
	int interp_coef_offset;
	int dnscal_coef_offset;
	struct ksc_image src_img;
	struct ksc_image dst_img;
};

struct sunxi_ksc_online_para {
	bool enable;
	int in_fmt;
	int w;
	int h;
	int bit_depth;
	int fps;
	int clk_freq;
};

struct ksc_dmabuf_item {
	struct list_head list;
	int fd;
	struct dma_buf *buf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	dma_addr_t dma_addr;
	struct device *dev;
};

struct ksc_freeing_buffers {
	void *mem0_virt;
	dma_addr_t mem0_phy;
	void *mem1_virt;
	dma_addr_t mem1_phy;
	unsigned int mem_size;
};

struct ksc_buffers {
	bool alloced;
	struct device *dev;
	void *mem0_virt;
	dma_addr_t mem0_phy;
	void *mem1_virt;
	dma_addr_t mem1_phy;
	unsigned int mem_size;
	unsigned int chm_offset;
	unsigned int pix_2b_offset;
	void *lut_virt;
	dma_addr_t lut_phy;
	unsigned int lut_size;
	void *lut_mem0_virt;
	dma_addr_t lut_mem0_phy;
	void *lut_mem1_virt;
	dma_addr_t lut_mem1_phy;
	void *lut_mem2_virt;
	dma_addr_t lut_mem2_phy;
	struct ksc_dmabuf_item *src_item;
	struct ksc_dmabuf_item *dst_item;
	struct ksc_freeing_buffers buf_to_be_free;
};

struct ksc_band_width_info {
	unsigned int bw_ctrl_en;
	unsigned int bw_ctrl_num;
	int fps;
	int clk_freq;
};

struct ksc_para {
	struct sunxi_ksc_para para;
	struct ksc_buffers buf;
	bool is_first_frame;
	struct ksc_band_width_info bw;
};

struct ksc_drv_data {
	int version;
	bool support_offline_ksc;
	bool support_offline_up_scaler;
	bool support_offline_arbitrary_angel_rotation;
	bool support_offline_img_crop;
	bool support_offline_img_flip;
};

struct ksc_device {
	bool enabled;
	struct mutex dev_lock;
	struct device *dev;
	unsigned char attr_dev[624];
	void *attr_grp;
	void __iomem *reg;
	void __iomem *tv_disp_top_reg;
	struct device *ksc_top;
	unsigned int irq_no;
	struct clk *clks[KSC_MAX_CLKS];
	struct reset_control *rsts[KSC_MAX_CLKS];
	wait_queue_head_t event_wait;
	const struct ksc_drv_data *drv_data;
	unsigned int clk_freq[KSC_MAX_CLKS];
	int be_err_cnt;
	int fe_err_cnt;
	unsigned int be_finish_cnt;
	int svp2ksc_dmt_err_cnt;
	int irq_cnt;
	bool be_finish;
	struct delayed_work free_buffer_work;
	struct delayed_work free_reserve_mem;
	struct ksc_para ksc_para;
	int (*enable)(struct ksc_device *dev, bool on);
	int (*set_ksc_para)(struct ksc_device *dev, struct sunxi_ksc_para *para);
	int (*ksc_set_pq_para)(struct ksc_device *dev, struct sunxi_ksc_para *para);
	struct ksc_buffers *(*get_ksc_buffers)(struct ksc_device *dev);
	struct sunxi_ksc_para *(*get_ksc_para)(struct ksc_device *dev);
};

static_assert(!(sizeof(struct sunxi_ksc_para) != 340), "ksc abi");
static_assert(!(sizeof(struct ksc_buffers) != 100), "ksc abi");
static_assert(!(offsetof(struct sunxi_ksc_para, mode) != 8), "ksc abi");
static_assert(!(offsetof(struct sunxi_ksc_para, src_img) != 212), "ksc abi");
static_assert(!(offsetof(struct sunxi_ksc_para, dst_img) != 276), "ksc abi");
static_assert(!(offsetof(struct ksc_buffers, lut_virt) != 36), "ksc abi");
static_assert(!(offsetof(struct ksc_buffers, src_item) != 72), "ksc abi");

#define KSC_IOC_SET_ONLINE	_IOW('k', 2, struct sunxi_ksc_online_para)
#define KSC_IOC_SET_PARA	_IOW('k', 0, struct sunxi_ksc_para)
#define KSC_IOC_GET_PARA	_IOW('k', 1, struct sunxi_ksc_para)

struct ksc_drv_info {
	struct ksc_device *p_ksc_device;
	struct class *p_class;
	struct device *p_device;
	dev_t dev_id;
};

static struct ksc_device *g_ksc;
static atomic_t g_ksc_open_count = ATOMIC_INIT(0);
static struct class *g_ksc_class;
static struct device *g_ksc_dev;
static dev_t g_ksc_devt;

static bool is_ksc_smooth_disp;
static int ksc_pq_sharpen_level = 1;

static inline void ksc_wr(struct ksc_device *d, int idx, u32 val)
{
	writel(val, d->reg + idx * 4);
}

static inline u32 ksc_rd(struct ksc_device *d, int idx)
{
	return readl(d->reg + idx * 4);
}

/* ------------------------------------------------------------------------- */
/* clocks / resets                                                           */
/* ------------------------------------------------------------------------- */

static void ksc_clk_setting(struct ksc_device *d, bool on)
{
	int i;

	for (i = 0; i < KSC_MAX_CLKS; i++) {
		if (on) {
			if (!IS_ERR_OR_NULL(d->rsts[i]))
				reset_control_deassert(d->rsts[i]);
			if (IS_ERR_OR_NULL(d->clks[i]))
				continue;
			if (d->clk_freq[i])
				clk_set_rate(d->clks[i], d->clk_freq[i]);
			if (clk_prepare_enable(d->clks[i]))
				dev_err(d->dev, "enable clk%d failed\n", i);
			else
				dev_info(d->dev, "clk%d rate:%lu\n", i,
					 clk_get_rate(d->clks[i]));
		} else {
			if (!IS_ERR_OR_NULL(d->clks[i]))
				clk_disable_unprepare(d->clks[i]);
			if (!IS_ERR_OR_NULL(d->rsts[i]))
				reset_control_assert(d->rsts[i]);
		}
	}
}

/* ------------------------------------------------------------------------- */
/* memory                                                                    */
/* ------------------------------------------------------------------------- */

static void *ksc_mem_alloc(struct device *dev, size_t size, dma_addr_t *phy)
{
	return dma_alloc_attrs(dev, size, phy, GFP_KERNEL, 0);
}

static void ksc_mem_free(struct device *dev, size_t size, void *virt,
			 dma_addr_t phy)
{
	dma_free_attrs(dev, size, virt, phy, 0);
}

static int ksc_mem_mmap(struct device *dev, struct vm_area_struct *vma,
			void *virt, dma_addr_t phy, size_t size)
{
	return dma_mmap_attrs(dev, vma, virt, phy, size, 0);
}

static void ksc_mem_sync(struct device *dev, dma_addr_t phy, size_t size)
{
	dma_sync_single_for_device(dev, phy, size, DMA_TO_DEVICE);
}

static int ksc_dma_map(struct ksc_device *d, int fd,
		       struct ksc_dmabuf_item *item)
{
	struct dma_buf *buf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;

	if (fd < 0) {
		dev_err(d->dev, "dma_buf_id %d is invalid\n", fd);
		return -1;
	}

	buf = dma_buf_get(fd);
	if (IS_ERR(buf)) {
		dev_err(d->dev, "dma_buf_get failed, fd=%d\n", fd);
		return -1;
	}

	attach = dma_buf_attach(buf, d->dev);
	if (IS_ERR(attach)) {
		dev_err(d->dev, "dma_buf_attach failed\n");
		dma_buf_put(buf);
		return -1;
	}

	sgt = dma_buf_map_attachment(attach, DMA_TO_DEVICE);
	if (IS_ERR(sgt)) {
		dev_err(d->dev, "dma_buf_map_attachment failed\n");
		dma_buf_detach(buf, attach);
		dma_buf_put(buf);
		return -1;
	}

	item->sgt = sgt;
	item->buf = buf;
	item->fd = fd;
	item->attachment = attach;
	item->dma_addr = sg_dma_address(sgt->sgl);
	return 0;
}

static void ksc_dma_unmap(struct ksc_dmabuf_item *item)
{
	if (!item || !item->buf)
		return;

	dma_buf_unmap_attachment(item->attachment, item->sgt, DMA_TO_DEVICE);
	dma_buf_detach(item->buf, item->attachment);
	dma_buf_put(item->buf);
	item->buf = NULL;
	item->fd = -1;
}

static void ksc_free_memory(struct ksc_device *d)
{
	struct ksc_buffers *b = &d->ksc_para.buf;

	if (b->mem0_virt)
		ksc_mem_free(d->dev, b->mem_size, b->mem0_virt, b->mem0_phy);
	if (b->mem1_virt)
		ksc_mem_free(d->dev, b->mem_size, b->mem1_virt, b->mem1_phy);
	if (b->lut_mem0_virt)
		ksc_mem_free(d->dev, b->lut_size, b->lut_mem0_virt,
			     b->lut_mem0_phy);
	if (b->lut_mem1_virt)
		ksc_mem_free(d->dev, b->lut_size, b->lut_mem1_virt,
			     b->lut_mem1_phy);
	if (b->lut_mem2_virt)
		ksc_mem_free(d->dev, b->lut_size, b->lut_mem2_virt,
			     b->lut_mem2_phy);

	b->mem0_virt = NULL;
	b->mem0_phy = 0;
	b->mem1_virt = NULL;
	b->mem1_phy = 0;
	b->lut_mem0_virt = NULL;
	b->lut_mem0_phy = 0;
	b->lut_mem1_virt = NULL;
	b->lut_mem1_phy = 0;
	b->lut_mem2_virt = NULL;
	b->lut_mem2_phy = 0;
	b->mem_size = 0;
	b->chm_offset = 0;
	b->pix_2b_offset = 0;

	if (b->src_item) {
		ksc_dma_unmap(b->src_item);
		kfree(b->src_item);
		b->src_item = NULL;
	}
	if (b->dst_item) {
		ksc_dma_unmap(b->dst_item);
		kfree(b->dst_item);
		b->dst_item = NULL;
	}

	b->alloced = false;
}

static void ksc_free_buffer_work(struct work_struct *work)
{
	struct ksc_device *d = container_of(to_delayed_work(work),
					    struct ksc_device, free_buffer_work);
	struct ksc_freeing_buffers *f = &d->ksc_para.buf.buf_to_be_free;

	if (!f->mem_size)
		return;

	if (f->mem0_virt)
		ksc_mem_free(d->dev, f->mem_size, f->mem0_virt, f->mem0_phy);
	if (f->mem1_virt)
		ksc_mem_free(d->dev, f->mem_size, f->mem1_virt, f->mem1_phy);

	dev_info(d->dev, "Free memory %u\n", f->mem_size);
	f->mem0_virt = NULL;
	f->mem0_phy = 0;
	f->mem1_virt = NULL;
	f->mem1_phy = 0;
	f->mem_size = 0;
}

/* ------------------------------------------------------------------------- */
/* interrupts                                                                */
/* ------------------------------------------------------------------------- */

static bool ksc_is_reg_update_finish(struct ksc_device *d)
{
	u32 v = ksc_rd(d, KSC_R_INTR);

	if (v & 0x100000) {
		ksc_wr(d, KSC_R_INTR, v | 0x100000);
		if (!(ksc_rd(d, KSC_R_VERSION) & 2)) {
			u32 s = ksc_rd(d, KSC_R_INTR_EN);

			if (!(s & 0x100))
				ksc_wr(d, KSC_R_INTR_EN, s | 0x100);
		}
	}
	return !!(v & 0x100000);
}

static u32 ksc_reg_irq_query(struct ksc_device *d)
{
	u32 v = ksc_rd(d, KSC_R_INTR);
	u32 result;

	if (v & 0x11110)
		ksc_wr(d, KSC_R_INTR, v);

	result = ((v >> 4) & 0x10) | ((v >> 10) & 4) | ((v >> 15) & 2) |
		 ((v & 0x10) ? 1 : 0) | ((v & 0x1000000) >> 21);

	if ((v & 0x1000000) || !result) {
		u32 w = v | 0x1000000;

		if (!result)
			w = 0xffffffff;
		ksc_wr(d, KSC_R_INTR, w);
	}

	return result ? result : 6;
}

static irqreturn_t ksc_irq_handler(int irq, void *data)
{
	struct ksc_device *d = data;
	u32 v = ksc_reg_irq_query(d);

	if (v & 1) {
		d->be_finish = true;
		wake_up(&d->event_wait);
	}
	if (v & 4)
		d->be_finish_cnt++;
	if (v & 0x10)
		d->svp2ksc_dmt_err_cnt++;
	if (v & 2)
		d->fe_err_cnt++;
	d->irq_cnt++;

	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------------- */
/* register programming                                                      */
/* ------------------------------------------------------------------------- */

static void ksc_reg_other_setting(struct ksc_device *d,
				  struct sunxi_ksc_para *q)
{
	u32 v4, v5, v6, v10, v12, v13, v14;
	u32 v30, v31, v32, v33, v34, v36, v37, v38;
	u32 v40, v42, v45, v46, v47;

	if (q->scaler_en) {
		v4 = (ksc_rd(d, KSC_R_SCALE_RATIO) & 0xFFFF8000) |
		     (q->src_w & 0x7FFF);
		ksc_wr(d, KSC_R_SCALE_RATIO, v4);
		v5 = ((q->src_h << 16) & 0x7FFF0000) | (v4 & 0x8000FFFF);
	} else {
		v5 = (ksc_rd(d, KSC_R_SCALE_RATIO) & 0x80008000) | 0xFFF0FFF;
	}
	ksc_wr(d, KSC_R_SCALE_RATIO, v5);

	if (q->mode && q->crop_en == 1) {
		v6 = ksc_rd(d, KSC_R_ROI_HOR);
		ksc_wr(d, KSC_R_ROI_HOR, v6 & 0xFFFFF000);
		v12 = q->dst_w - 1;
		v38 = (v12 >= 0) ? (((u32)v12 >> 11) ? 0x7FF0000 :
				    ((v12 << 16) & 0xFFF0000)) : 0;
		ksc_wr(d, KSC_R_ROI_HOR, v38 | (v6 & 0xF000F000));

		v10 = ksc_rd(d, KSC_R_ROI_VER);
		ksc_wr(d, KSC_R_ROI_VER, v10 & 0xFFFFF000);
		v12 = q->dst_h - 1;
		v40 = (v12 >= 0) ? (((u32)v12 > 0x800) ? 0x8000000 :
				    ((v12 << 16) & 0xFFF0000)) : 0;
		ksc_wr(d, KSC_R_ROI_VER, v40 | (v10 & 0xF000F000));

		v13 = (ksc_rd(d, KSC_R_CROP_OFFSET) & 0xFFFFF000) |
		      (q->crop_x & 0xFFF);
		ksc_wr(d, KSC_R_CROP_OFFSET, v13);
		ksc_wr(d, KSC_R_CROP_OFFSET, (v13 & 0xF000FFFF) |
					     ((q->crop_y & 0xFFF) << 16));

		v45 = ksc_rd(d, KSC_R_SCALE_CROP);
		v46 = ((q->dst_w << 16) + 0x20000000) & 0x1FFF0000;
		ksc_wr(d, KSC_R_SCALE_CROP, v46 | (v45 & 0xE000FFFF));
		v47 = ksc_rd(d, KSC_R_SCALE_CROP);
		ksc_wr(d, KSC_R_SCALE_CROP,
		       (v47 & 0xFFFFE000) | ((q->dst_h - 1) & 0x1FFF));
	} else {
		v33 = q->lut.roi_x_max_non_ali;
		if (v33 >= 0x7FF)
			v33 = 2047;
		v34 = q->lut.roi_y_max_non_ali;
		if (v34 >= 0x800)
			v34 = 2048;

		v6 = ksc_rd(d, KSC_R_ROI_HOR);
		v36 = (ksc_rd(d, KSC_R_ROI_HOR) & 0xFFFFF000) | v33;
		ksc_wr(d, KSC_R_ROI_HOR, v36);
		v37 = v36 & 0xF000F7FF;
		ksc_wr(d, KSC_R_ROI_HOR, ((v34 & 0xFFF) << 16) | v37);

		v10 = ksc_rd(d, KSC_R_ROI_VER);
		v31 = (v10 & 0xFFFFF000) | (q->lut.roi_y_max & 0xFFF);
		ksc_wr(d, KSC_R_ROI_VER, v31);
		v32 = v31 & 0xF000FFFF;
		ksc_wr(d, KSC_R_ROI_VER,
		       ((q->lut.roi_x_max & 0x1FFF) << 16) | v32);
	}

	v30 = (ksc_rd(d, KSC_R_OUT_POS) & 0xE000FFFF) |
	      ((q->ost_v & 0x1FFF) << 16);
	ksc_wr(d, KSC_R_OUT_POS, v30);
	ksc_wr(d, KSC_R_OUT_POS, (v30 & 0xFFFFE000) | (q->ost_h & 0x1FFF));

	v31 = (ksc_rd(d, KSC_R_PHASE) & 0xFFFFE0FF) | ((q->phase_v & 0x1F) << 8);
	ksc_wr(d, KSC_R_PHASE, v31);
	ksc_wr(d, KSC_R_PHASE, (v31 & 0xFFFFFFE0) | (q->phase_h & 0x1F));

	v42 = ksc_rd(d, KSC_R_MEM_REQ);
	ksc_wr(d, KSC_R_DEBUG0, 0);
	ksc_wr(d, KSC_R_DEBUG1, 0);
	ksc_wr(d, KSC_R_DEBUG2, 0);
	ksc_wr(d, KSC_R_DEBUG3, 0);
	ksc_wr(d, KSC_R_MEM_REQ, (v42 | 0x1F) & 0xFFFF);

	v14 = (ksc_rd(d, KSC_R_MODE0_HW) & 0xFFFE0000) | 0x400;
	if (d->ksc_para.is_first_frame)
		v14 |= 1 << 16;
	ksc_wr(d, KSC_R_MODE0_HW, v14);
}

static void ksc_reg_pq_setting(struct ksc_device *d, struct sunxi_ksc_para *q)
{
	unsigned char *pq = q->pq_para.data;
	u32 v, v5, v14, v16, v19, v22, v25, v30, v33, v38, v41;
	u32 v43, v46, v51, v59, v60, v65, v68;
	int t;

	v5 = q->lut.roi_y_min;   /* used as default pixel if not set later */
	(void)v5;

	/* default pixel */
	t = q->lut.roi_x_min < 0xFF ? q->lut.roi_x_min : 255;
	v = (ksc_rd(d, KSC_R_DEF_PIX) & 0xFFFFFC00) | t;
	v = (v & 0xFFF000FF) | ((min_t(u32, q->lut.roi_y_min, 255)) << 10);
	ksc_wr(d, KSC_R_DEF_PIX, v);
	v = (v & 0xC003FCFF) | ((min_t(u32, q->lut.roi_w_min, 255)) << 20);
	ksc_wr(d, KSC_R_DEF_PIX, v);

	/* detail control */
	v14 = ksc_rd(d, KSC_R_DETAIL) & 0xFFFFFF;
	v16 = min_t(u32, pq[5], 128);
	v14 |= v16 << 24;
	v19 = min_t(u32, pq[4], 128);
	v14 = (v14 & 0xFF00FFFF) | (v19 << 16);
	v22 = min_t(u32, pq[3], 128);
	v14 = (v14 & 0xFFFF00FF) | (v22 << 8);
	v25 = min_t(u32, pq[2], 128);
	v14 = (v14 & 0xFFFFFF00) | v25;
	ksc_wr(d, KSC_R_DETAIL, v14);

	/* filter select */
	v30 = (ksc_rd(d, KSC_R_FILT_SEL) & 0xFFFCFFFF) | ((pq[13] & 3) << 16);
	t = pq[16] ? min_t(u32, pq[16], 64) : 1;
	v30 = (v30 & 0xFFFF80FF) | (t << 8);
	v30 = (v30 & 0xFFFFFFC0) | min_t(u32, pq[15], 63);
	ksc_wr(d, KSC_R_FILT_SEL, v30);

	/* anti alias threshold */
	v33 = min_t(u32, q->lut.roi_w_max, 511);
	v38 = (ksc_rd(d, KSC_R_AA_THR) & 0xFE00FFFF) | (v33 << 16);
	ksc_wr(d, KSC_R_AA_THR, v38);
	t = pq[18] ? min_t(u32, pq[18], 64) : 1;
	v38 = (v38 & 0xFFFF80FF) | (t << 8);
	v38 = (v38 & 0xFFFFFFC0) | min_t(u32, pq[17], 63);
	ksc_wr(d, KSC_R_AA_THR, v38);

	/* anti alias strength */
	v41 = pq[19] > 0x80 ? 0x800000 : (pq[19] << 16);
	v41 |= ksc_rd(d, KSC_R_AA_STR) & 0xFF00FFFF;
	ksc_wr(d, KSC_R_AA_STR, v41);
	v41 = (v41 & 0xFFFFFF00) | min_t(u32, pq[12], 128);
	ksc_wr(d, KSC_R_AA_STR, v41);

	/* direction residual */
	v43 = pq[10] > 0x40 ? 0x40000000 : ((pq[10] & 0x7F) << 24);
	v43 |= ksc_rd(d, KSC_R_DIR_RSID) & 0x80FFFFFF;
	ksc_wr(d, KSC_R_DIR_RSID, v43);
	v46 = pq[11] > 0x40 ? 0x400000 : ((pq[11] & 0x7F) << 16);
	v46 |= v43 & 0xFF80FFFF;
	ksc_wr(d, KSC_R_DIR_RSID, v46);
	v51 = pq[8] > 0x40 ? 0x4000 : ((pq[8] & 0x7F) << 8);
	v51 |= v46 & 0xFFFF80FF;
	ksc_wr(d, KSC_R_DIR_RSID, v51);
	v51 = (v51 & 0xFFFFFF80) | min_t(u32, pq[9], 64);
	ksc_wr(d, KSC_R_DIR_RSID, v51);

	/* direction max residual ratio */
	v59 = pq[7] > 0x80 ? 0x800000 : (pq[7] << 16);
	v59 |= ksc_rd(d, KSC_R_DIR_MAX_RSID) & 0xFF00FFFF;
	ksc_wr(d, KSC_R_DIR_MAX_RSID, v59);
	v59 = (v59 & 0xFFFFFF00) | min_t(u32, pq[6], 128);
	ksc_wr(d, KSC_R_DIR_MAX_RSID, v59);

	ksc_wr(d, KSC_R_PHASE, ksc_rd(d, KSC_R_PHASE) | 0xFF0000);

	/* default rgb */
	v60 = min_t(u32, q->ksc_default_rgb_ch0, 255);
	v65 = (ksc_rd(d, KSC_R_DEF_RGB) & 0xFFFFFC00) | v60;
	ksc_wr(d, KSC_R_DEF_RGB, v65);
	v68 = q->ksc_default_rgb_ch1 > 0xFF ? 0x3FC00 :
	      ((q->ksc_default_rgb_ch1 << 10) & 0xFFC00);
	ksc_wr(d, KSC_R_DEF_RGB, v68 | (v65 & 0xFFF000FF));
	v65 = ksc_rd(d, KSC_R_DEF_RGB);
	v68 = q->ksc_default_rgb_ch2 > 0xFF ? 0x3FF00000 :
	      ((q->ksc_default_rgb_ch2 << 20) & 0x3FF00000);
	ksc_wr(d, KSC_R_DEF_RGB, (v65 & 0xC00FFCFF) | v68);

	/* border blur */
	v68 = pq[22] & 0xF;
	v68 = (ksc_rd(d, KSC_R_BLUR1) & 0xFF0FFFFF) | (v68 << 20);
	ksc_wr(d, KSC_R_BLUR1, v68);
	v68 = (v68 & 0xFFFE0FFF) | ((pq[23] & 0x1F) << 12);
	ksc_wr(d, KSC_R_BLUR1, v68);
	v68 = (v68 & 0xFFFFF0FF) | ((pq[26] & 0xF) << 8);
	ksc_wr(d, KSC_R_BLUR1, v68);
	ksc_wr(d, KSC_R_BLUR1, (v68 & 0xFFFFFFE0) | (pq[27] & 0x1F));

	v68 = (ksc_rd(d, KSC_R_BLUR0) & 0xFF0FFFFF) | ((pq[20] & 0xF) << 20);
	ksc_wr(d, KSC_R_BLUR0, v68);
	v68 = (v68 & 0xFFFE0FFF) | ((pq[21] & 0x1F) << 12);
	ksc_wr(d, KSC_R_BLUR0, v68);
	v68 = (v68 & 0xFFFFF0FF) | ((pq[24] & 0xF) << 8);
	ksc_wr(d, KSC_R_BLUR0, v68);
	ksc_wr(d, KSC_R_BLUR0, (v68 & 0xFFFFFFE0) | (pq[25] & 0x1F));
}

static void ksc_reg_switch_lut_buf(struct ksc_device *d,
				   struct sunxi_ksc_para *q)
{
	struct ksc_buffers *b = &d->ksc_para.buf;
	u32 cur = ksc_rd(d, KSC_R_LUT_ADDR);
	u32 virt, phy;

	if (cur == (u32)b->lut_mem0_phy) {
		virt = (u32)(unsigned long)b->lut_mem1_virt;
		phy = (u32)b->lut_mem1_phy;
	} else if (cur == (u32)b->lut_mem1_phy) {
		virt = (u32)(unsigned long)b->lut_mem2_virt;
		phy = (u32)b->lut_mem2_phy;
	} else {
		virt = (u32)(unsigned long)b->lut_mem0_virt;
		phy = (u32)b->lut_mem0_phy;
	}

	b->lut_phy = phy;
	b->lut_virt = (void *)(unsigned long)virt;
	(void)q;
}

static void ksc_reg_update_all(struct ksc_device *d, struct ksc_para *p)
{
	struct sunxi_ksc_para *q = &p->para;
	struct ksc_buffers *b = &p->buf;
	u32 v, v6, v8, v12, v13, v14;
	int v24, v25, v26, v28, v29, v30, v31, v33, v34, v35, v36;
	int v37, v39, v44, v45, v46, v50, v51, v52, v53;
	int v54, v55, v56, v57, v58, v65, v66, v67, v69, v72;

	v = ksc_rd(d, KSC_R_TOP);
	ksc_wr(d, KSC_R_TOP, v & ~0x20000000u);
	v = (v & 0xDFFFFCFF) | ((u32)(q->mode & 3) << 8);
	ksc_wr(d, KSC_R_TOP, v);
	v6 = v & 0xCFFFFFFF;
	if (q->bit_depth != 8)
		v6 |= 0x10000000u;
	ksc_wr(d, KSC_R_TOP, v6);

	if (q->mode) {
		ksc_wr(d, KSC_R_RESET, ksc_rd(d, KSC_R_RESET) | 0x10);
		fsleep(4295);
		ksc_wr(d, KSC_R_RESET, ksc_rd(d, KSC_R_RESET) & ~0x10);
		v6 = ksc_rd(d, KSC_R_TOP);
	}

	v = (v6 & 0xFF8FFFFF) | ((u32)(q->csc_fmt & 7) << 20);
	ksc_wr(d, KSC_R_TOP, v);

	if (q->mode) {
		switch (q->wb_fmt) {
		case 2:  v8 = (v & 0xF0FFFFFF) | 0x4000000; break;
		case 3:  v8 = (v & 0xF0FFFFFF) | 0x5000000; break;
		case 6:  v8 = (v & 0xF0FFFFFF) | 0x2000000; break;
		case 7:  v8 = (v & 0xF0FFFFFF) | 0x3000000; break;
		case 0xb: v8 = (v & 0xF0FFFFFF) | 0x1000000; break;
		case 0xe: v8 = (v & 0xF0FFFFFF) | 0x6000000; break;
		case 0xf: v8 = (v & 0xF0FFFFFF) | 0x7000000; break;
		case 0x10: v8 = (v & 0xF0FFFFFF) | 0x8000000; break;
		case 0x11: v8 = (v & 0xF0FFFFFF) | 0x9000000; break;
		default: v8 = v & 0xF0FFFFFF; break;
		}
	} else {
		v8 = v & 0xF0FFFFFF;
	}
	ksc_wr(d, KSC_R_TOP, v8);

	v = (v8 & 0xFFFBFFFF) | ((u32)(q->pq_para.data[0] & 1) << 18);
	ksc_wr(d, KSC_R_TOP, v);
	v = (v & 0xFFFFEFEF) | ((u32)(q->scaler_en & 1) << 12);
	ksc_wr(d, KSC_R_TOP, v);

	if ((unsigned int)(q->pix_fmt - 4) < 4)
		v12 = (v & 0xFFFCFFEF) | 0x10000;
	else if ((unsigned int)(q->pix_fmt - 8) > 3)
		v12 = (v & 0xFFFCFFEF) | 0x10000;
	else
		v12 = v & 0xFFFCFFEF;
	ksc_wr(d, KSC_R_TOP, v12 | 1);

	v13 = ksc_rd(d, KSC_R_INTR_EN);
	v14 = (ksc_rd(d, KSC_R_FUNC) & 0xEFFFFFFF) |
	     ((u32)(q->ksc_en & 1) << 28);
	v14 = (v14 & 0xF7FFFFFF) | ((u32)(q->flip_v & 1) << 27);
	v14 = (v14 & 0xFBFFFFFF) | ((u32)(q->flip_h & 1) << 26);
	v14 = (v14 & 0xFCFFFFFF) | ((u32)(q->rot_angle & 3) << 24);
	v14 = (v14 & 0xFFEFCEFF) | ((u32)(q->crop_en & 1) << 20) | 0x1000;
	v14 = (v14 & 0xFFFFFFEF) | (16 * (q->perf_mode_en & 0xF));
	ksc_wr(d, KSC_R_FUNC, v14);
	v14 = (v14 & 0xFFFFFFFE) | (q->online_wb_en & 1);
	ksc_wr(d, KSC_R_FUNC, v14);

	{
		u32 ve = v13 | 0x101010;

		if (!(v12 & 0x300))
			ve = (v13 & 0xFFFFFFEF) | 0x101000;
		ksc_wr(d, KSC_R_INTR_EN, ve);
	}

	v24 = q->dns_w;
	if (q->mode) {
		v25 = (v24 + 63) & ~63;
		v26 = v25;
		if (q->mode == 2 && ((u32)(q->wb_fmt & ~1) == 0xC))
			v26 = (4 * v24 + 60) & ~0x3E;

		v28 = 0;
		if (q->pix_fmt <= 0xB) {
			if ((1 << q->pix_fmt) & 0xC) {
				v33 = q->bit_depth;
				if (v33 == 10) {
					v28 = (2 * v24 + 62) & ~0x3E;
					v25 = (v24 + 127) & ~127;
					v26 = v28;
					goto lut_ready;
				}
				if (v33 == 8) {
					v34 = (v24 + 127) & ~127;
					v28 = (v24 + 63) & ~63;
					v25 = v34;
					goto lut_ready;
				}
				v25 = 0;
			} else if ((1 << q->pix_fmt) & 0xC0) {
				v34 = ((v24 + 127) >> 1) & ~63;
				v28 = (v24 + 63) & ~63;
				v25 = v34;
			} else if ((1 << q->pix_fmt) & 0xC00) {
				v28 = (2 * v24 + 62) & ~0x3E;
				goto lut_ready;
			} else {
				v25 = 0;
			}
		}
	} else {
		v29 = q->dns_w < 2048 ? q->dns_w : 2048;
		if (v29 <= 64)
			v29 = 64;
		v30 = q->dns_h < 2048 ? q->dns_h : 2048;
		if (v30 <= 64)
			v30 = 64;
		v31 = (ksc_rd(d, KSC_R_DST_SIZE) & 0xE000FFFF) |
		      ((v29 << 16) & 0x1FFF0000);
		ksc_wr(d, KSC_R_DST_SIZE, v31);
		ksc_wr(d, KSC_R_DST_SIZE, (v31 & 0xFFFFE000) | v30);

		ksc_wr(d, KSC_R_SRC_LUM0, (u32)(unsigned long)b->mem0_virt ?
		       (u32)b->mem0_phy : 0);
		ksc_wr(d, KSC_R_SRC_CHM0, (u32)b->mem0_phy);
		ksc_wr(d, KSC_R_SRC_LUM1, (u32)b->mem1_phy);
		ksc_wr(d, KSC_R_SRC_CHM1, (u32)b->mem1_phy);

		v26 = (2 * v24 + 62) & ~0x3E;
		v28 = v26;
		v25 = v26;
		if (v12 & 0x30000)
			v25 = (v24 + 63) & ~63;
		else
			v28 = (4 * v24 + 60) & ~0x3E;

		if (q->bit_depth == 10) {
			ksc_wr(d, KSC_R_SRC_PIX0, (u32)b->mem0_phy + b->pix_2b_offset);
			ksc_wr(d, KSC_R_SRC_PIX1, (u32)b->mem1_phy + b->pix_2b_offset);
		}
	}

lut_ready:
	v35 = b->dst_item ? (int)b->dst_item->dma_addr : 0;
	v36 = 4096;
	if (v35 >= 4096)
		v35 = 4096;
	if (v35 <= 64)
		v35 = 64;
	v37 = ksc_rd(d, KSC_R_DST8B_STRIDE) & 0xFFFFE000;
	{
		u32 v38 = ((u32)(v35 << 16) & 0x1FFF0000) |
			  (ksc_rd(d, KSC_R_DST_SIZE) & 0xE000FFFF);

		ksc_wr(d, KSC_R_DST_SIZE, v38);
		ksc_wr(d, KSC_R_DST8B_STRIDE, v37);
		v39 = q->dst_img.h;
		if (v39 < 4096)
			v36 = v39;
		if (v36 <= 64)
			v36 = 64;
		ksc_wr(d, KSC_R_DST_SIZE, (v38 & 0xFFFFE000) | v36);
	}

	ksc_wr(d, KSC_R_SRC_LUM0, b->src_item ? (u32)b->src_item->dma_addr : 0);
	ksc_wr(d, KSC_R_DST_LUM, b->dst_item ? (u32)b->dst_item->dma_addr : 0);

	if ((unsigned int)(q->pix_fmt - 12) >= 6) {
		int v41_ = q->dst_img.w_align;
		int shift = (q->bit_depth == 10) ? 1 : 0;

		ksc_wr(d, KSC_R_SRC_CHM0, (b->src_item ?
		       (u32)b->src_item->dma_addr : 0) +
		       ((q->src_img.h * q->src_img.w_align) << shift));
		ksc_wr(d, KSC_R_DST_CHM, (b->dst_item ?
		       (u32)b->dst_item->dma_addr : 0) +
		       ((q->dst_img.h * v41_) << shift));
	}

	v44 = v28 / 4;
	v45 = ((v28 / 4) << 16) & 0x1FFF0000;
	v46 = ((v26 + ((unsigned int)(v26 >> 31) >> 30)) >> 2) & 0x1FFF;
	{
		u32 s = (u32)v45 | (ksc_rd(d, KSC_R_SRC8B1_STRIDE) & 0xE000E000) |
			(u32)v46;

		ksc_wr(d, KSC_R_SRC8B1_STRIDE, (u32)v45 |
		       (ksc_rd(d, KSC_R_SRC8B1_STRIDE) & 0xE000E000) | (u32)v46);
		ksc_wr(d, KSC_R_SRC8B0_STRIDE, s);
	}
	if (q->bit_depth == 10) {
		u32 s2 = (ksc_rd(d, KSC_R_SRC2B1_STRIDE) & 0xFFFFE000) |
			 ((u32)(v25 / 4) & 0x1FFF);

		ksc_wr(d, KSC_R_SRC2B0_STRIDE,
		       (ksc_rd(d, KSC_R_SRC2B0_STRIDE) & 0xFFFFE000) |
		       ((u32)(v25 / 4) & 0x1FFF));
		ksc_wr(d, KSC_R_SRC2B1_STRIDE, s2);
	}

	if (q->mode) {
		if (q->crop_en == 1)
			v50 = q->crop_w;
		else
			v50 = q->dst_w;
		v51 = v50 + 63;
		v52 = v50 + 31;
		v53 = (v50 + 127) & ~127;
		if (q->mode == 2 && ((u32)(q->wb_fmt & ~1) == 0xC)) {
			v54 = (v50 + 15) & ~0xF;
			v55 = (v51 >> 2) & ~0xF;
		} else {
			v55 = (v51 >> 2) & ~0xF;
			v54 = v55;
		}
	} else {
		v56 = q->src_w;
		v51 = v56 + 63;
		v52 = v56 + 31;
		v53 = (v56 + 127) & ~127;
		v55 = (v51 >> 2) & ~0xF;
		v54 = v55;
	}

	v57 = ksc_rd(d, KSC_R_DST8B_STRIDE);
	v58 = (u32)(v57 & 0xFFFFE000) | (u32)v54;
	ksc_wr(d, KSC_R_DST8B_STRIDE, v58);

	if ((u32)(q->wb_fmt & ~3) == 4) {
		u32 t = ksc_rd(d, KSC_R_DST2B_STRIDE) & 0xFFFFE000;

		ksc_wr(d, KSC_R_DST8B_STRIDE,
		       (v58 & 0xE000FFF0) | ((u32)v55 << 16));
		if (q->bit_depth == 10)
			t |= ((u32)v53 >> 3) & 0x1FF0;
		ksc_wr(d, KSC_R_DST2B_STRIDE, t);
	} else if ((u32)(q->wb_fmt & ~3) == 8 || q->wb_fmt > 3) {
		u32 t;

		if (q->wb_fmt > 3 && (u32)(q->wb_fmt & ~1) != 0xC)
			goto lut_addr;
		ksc_wr(d, KSC_R_DST8B_STRIDE,
		       (v58 & 0xE000FFF0) | (((u32)v52 << 15) & 0x1FF00000));
		t = ksc_rd(d, KSC_R_DST2B_STRIDE) & 0xFFFFE000;
		if (q->bit_depth == 10)
			t |= (u32)v55;
		ksc_wr(d, KSC_R_DST2B_STRIDE, t);
	} else {
		if (q->bit_depth == 10) {
			ksc_wr(d, KSC_R_DST2B_STRIDE,
			       (ksc_rd(d, KSC_R_DST2B_STRIDE) & 0xFFFFE000) |
			       (((u32)v53 >> 3) & 0x1FF0));
			ksc_wr(d, KSC_R_DST8B_STRIDE,
			       (((u32)v52 << 15) & 0x1FF00000) |
			       (((u32)v52 >> 1) & 0x1FF0) |
			       (v57 & 0xE000E000));
		} else if (q->bit_depth == 8) {
			ksc_wr(d, KSC_R_DST8B_STRIDE,
			       (v58 & 0xE000FFF0) | ((u32)v55 << 16));
			ksc_wr(d, KSC_R_DST2B_STRIDE,
			       ksc_rd(d, KSC_R_DST2B_STRIDE) & 0xFFFFE000);
		}
	}

lut_addr:
	if (q->mode == 1)
		v65 = (q->src_w + 15) & 0xFF0;
	else
		v65 = q->lut.lut_stride & 0xFFF;
	ksc_wr(d, KSC_R_LUT_STRIDE,
	       (ksc_rd(d, KSC_R_LUT_STRIDE) & 0xFFFFF000) | (u32)v65);

	ksc_wr(d, KSC_R_LUT_ADDR, (u32)b->lut_phy);
	ksc_wr(d, KSC_R_INTERP_ADDR, (u32)b->lut_phy + q->interp_coef_offset);
	ksc_wr(d, KSC_R_DNSCAL_ADDR, (u32)b->lut_phy + q->dnscal_coef_offset);

	v66 = q->src_w;
	v67 = 0x400000;
	if (v66 >= 64) {
		v67 = 0x8000000;
		if ((unsigned int)v66 <= 0x800)
			v67 = (v66 << 16) & 0x1FFF0000;
	}
	v = (ksc_rd(d, KSC_R_IN_SIZE) & 0xE000FFFF) | (u32)v67;
	ksc_wr(d, KSC_R_IN_SIZE, v);
	v69 = q->src_h;
	if (v69 >= 2048)
		v69 = 2048;
	if (v69 <= 64)
		v69 = 64;
	ksc_wr(d, KSC_R_IN_SIZE, (u32)v69 | (v & 0xFFFFE000));

	if (q->mode == 1) {
		ksc_wr(d, KSC_R_M1_LUM,
		       (ksc_rd(d, KSC_R_M1_LUM) & 0xFFF00000) |
		       ((u32)(q->src_h * v46) & 0xFFFFF));
		ksc_wr(d, KSC_R_M1_CHM,
		       (ksc_rd(d, KSC_R_M1_CHM) & 0xFFE00000) |
		       ((u32)(q->src_h * (v44 & 0x1FFF)) & 0x1FFFFF));
		if (q->bit_depth == 10)
			ksc_wr(d, KSC_R_M1_PIX,
			       (ksc_rd(d, KSC_R_M1_PIX) & 0xFFE00000) |
			       ((u32)((ksc_rd(d, KSC_R_SRC2B0_STRIDE) & 0x1FFF) *
				      q->src_h) & 0x1FFFFF));
	}

	ksc_reg_other_setting(d, q);
	ksc_reg_pq_setting(d, q);

	v72 = (q->mode == 0) ? 256 : 1;
	ksc_wr(d, KSC_R_RESET, ksc_rd(d, KSC_R_RESET) | (u32)v72);
}

/* ------------------------------------------------------------------------- */
/* module parameter defaults                                                 */
/* ------------------------------------------------------------------------- */

static int ksc_set_sharpen_para(struct ksc_device *d, struct sunxi_ksc_para *q)
{
	unsigned char *p = (unsigned char *)q;
	const char *name;

	switch (ksc_pq_sharpen_level) {
	case 0:
		p[152] = 92;
		*(u16 *)(p + 142) = 4104;
		*(u32 *)(p + 144) = 538984496;
		*(u32 *)(p + 148) = 71305240;
		name = "NORMAL 1/2x";
		break;
	case 1:
		p[152] = 64;
		*(u16 *)(p + 142) = 2064;
		*(u32 *)(p + 144) = 404230160;
		*(u32 *)(p + 148) = 71307288;
		name = "NORMAL 1x";
		break;
	case 2:
		p[152] = 92;
		*(u16 *)(p + 142) = 12312;
		*(u32 *)(p + 144) = 538984496;
		*(u32 *)(p + 148) = 71309336;
		name = "NORMAL 1.5x";
		break;
	case 3:
		p[152] = 92;
		*(u16 *)(p + 142) = 16416;
		*(u32 *)(p + 144) = 538984496;
		*(u32 *)(p + 148) = 71311384;
		name = "NORMAL 2x";
		break;
	case 4:
		p[152] = 92;
		*(u16 *)(p + 142) = (u16)-32640;
		*(u32 *)(p + 144) = 538984496;
		*(u32 *)(p + 148) = 71319576;
		name = "NORMAL MAX";
		break;
	default:
		dev_err(d->dev, "Invalid ksc sharpen pq param, sharpen param range should be in level 0~4 \n");
		return -1;
	}

	dev_info(d->dev, "Image sharpening %s\n", name);
	return 0;
}

static int ksc_set_pq_para(struct ksc_device *d, struct sunxi_ksc_para *q)
{
	unsigned char *p = (unsigned char *)q;

	if (!q) {
		dev_err(d->dev, "Null ksc_pq_para\n");
		return -1;
	}

	/* ksc_en and scaler_en are set together here */
	*(u16 *)(p + 140) = 257;
	ksc_set_sharpen_para(d, q);
	*(u32 *)(p + 172) = 128;
	*(u8 *)(p + 167) = 24;
	*(u16 *)(p + 165) = 1048;
	*(u32 *)(p + 161) = 68682776;
	*(u32 *)(p + 168) = 0x800000;
	*(u32 *)(p + 157) = 71312150;
	*(u32 *)(p + 176) = 0;
	*(u32 *)(p + 153) = 538247424;

	return 0;
}

/* ------------------------------------------------------------------------- */
/* power on/off                                                              */
/* ------------------------------------------------------------------------- */

static int ksc_do_enable(struct ksc_device *d, bool on)
{
	struct ksc_buffers *b = &d->ksc_para.buf;
	int ret = 0;

	mutex_lock(&d->dev_lock);

	if (d->enabled == on)
		goto out;

	if (!on) {
		free_irq(d->irq_no, d);
		ksc_clk_setting(d, false);
		ksc_free_memory(d);
		d->enabled = false;
		goto out;
	}

	ksc_clk_setting(d, true);

	b->src_item = kzalloc(sizeof(*b->src_item), GFP_KERNEL);
	b->dst_item = kzalloc(sizeof(*b->dst_item), GFP_KERNEL);
	if (!b->src_item || !b->dst_item) {
		ret = -ENOMEM;
		goto err_items;
	}
	b->src_item->dev = d->dev;
	b->dst_item->dev = d->dev;

	ret = request_threaded_irq(d->irq_no, ksc_irq_handler, NULL, 0, KSC_NAME,
				   d);
	if (ret) {
		dev_err(d->dev, "request irq fail:%d\n", d->irq_no);
		ret = -ENODEV;
		goto err_items;
	}

	b->lut_size = KSC_LUT_SIZE;
	b->lut_mem0_virt = ksc_mem_alloc(d->dev, KSC_LUT_SIZE,
					 &b->lut_mem0_phy);
	if (!b->lut_mem0_virt) {
		dev_err(d->dev, "Memory0 alloc %d Byte ksc lut buffer fail!\n",
			KSC_LUT_SIZE);
		ret = -ENOMEM;
		goto err_irq;
	}

	b->lut_mem1_virt = ksc_mem_alloc(d->dev, KSC_LUT_SIZE,
					 &b->lut_mem1_phy);
	if (!b->lut_mem1_virt) {
		dev_err(d->dev, "Memory1 alloc %d Byte ksc lut buffer fail!\n",
			KSC_LUT_SIZE);
		ret = -ENOMEM;
		goto err_irq;
	}

	b->lut_mem2_virt = ksc_mem_alloc(d->dev, KSC_LUT_SIZE,
					 &b->lut_mem2_phy);
	if (!b->lut_mem2_virt) {
		dev_err(d->dev, "Memory2 alloc %d Byte ksc lut buffer fail!\n",
			KSC_LUT_SIZE);
		ret = -ENOMEM;
		goto err_irq;
	}

	b->lut_virt = b->lut_mem0_virt;
	b->lut_phy = b->lut_mem0_phy;
	d->enabled = true;
	goto out;

err_irq:
	free_irq(d->irq_no, d);
	ksc_free_memory(d);
err_items:
	kfree(b->src_item);
	kfree(b->dst_item);
	b->src_item = NULL;
	b->dst_item = NULL;
	ksc_clk_setting(d, false);
	d->enabled = false;
out:
	mutex_unlock(&d->dev_lock);
	return ret;
}

/* ------------------------------------------------------------------------- */
/* frame parameter programming                                               */
/* ------------------------------------------------------------------------- */

static int ksc_calc_mem_size(struct sunxi_ksc_para *q, unsigned int *size,
			     unsigned int *chm, unsigned int *pix2b)
{
	unsigned int line;

	*chm = 0;
	*pix2b = 0;
	*size = 0;

	switch (q->pix_fmt) {
	case 26:
		line = ((q->dns_w + 15) & ~15) * q->dns_h;
		*chm = 2 * line;
		*size = 4 * line;
		if (q->bit_depth == 10) {
			unsigned int line2 = ((q->dns_w + 63) & ~63) * q->dns_h;

			*size += line2;
			*pix2b = 2 * (line + line2);
		}
		return 0;

	case 8:
		line = ((q->dns_w + 63) & ~63) * q->dns_h;
		*chm = line;
		*size = 3 * line;
		if (q->bit_depth == 10) {
			*size += line;
			*pix2b = 3 * line + line;
		}
		return 0;

	case 6:
		line = ((q->dns_w + 63) & ~63) * q->dns_h;
		*chm = line;
		*size = 2 * line;
		if (q->bit_depth == 10)
			*pix2b = line + line;
		return 0;

	default:
		return -1;
	}
}

static int ksc_set_para(struct ksc_device *d, struct sunxi_ksc_para *q)
{
	struct ksc_buffers *b = &d->ksc_para.buf;
	unsigned int size = 0, chm = 0, pix2b = 0;
	int ret;

	mutex_lock(&d->dev_lock);

	if (!d->enabled) {
		dev_err(d->dev, "Ksc has not been enabled yet!\n");
		ret = -1;
		goto out;
	}

	memcpy(&d->ksc_para.para, q, sizeof(*q));

	ret = ksc_calc_mem_size(q, &size, &chm, &pix2b);
	if (ret) {
		if (q->mode == 0) {
			dev_err(d->dev, "Invalid FE output pixel format:%d\n",
				q->pix_fmt);
			goto err;
		}
		size = 0;
	}

	if (b->alloced) {
		if (size == b->mem_size || q->mode)
			goto have_buffer;

		if (b->buf_to_be_free.mem_size) {
			cancel_delayed_work_sync(&d->free_buffer_work);
			dev_info(d->dev, "Last frame's memory has not been free yet!\n");
		}

		b->alloced = false;
		b->buf_to_be_free.mem0_virt = b->mem0_virt;
		b->buf_to_be_free.mem0_phy = b->mem0_phy;
		b->buf_to_be_free.mem1_virt = b->mem1_virt;
		b->buf_to_be_free.mem1_phy = b->mem1_phy;
		b->buf_to_be_free.mem_size = b->mem_size;
		b->mem0_virt = NULL;
		b->mem1_virt = NULL;
	} else if (q->mode) {
		goto have_buffer;
	}

	dev_info(d->dev, "Alloc %u Byte memory for ksc\n", size);
	b->mem_size = size;
	b->chm_offset = chm;
	b->pix_2b_offset = pix2b;

	b->mem0_virt = ksc_mem_alloc(d->dev, size, &b->mem0_phy);
	b->mem1_virt = ksc_mem_alloc(d->dev, size, &b->mem1_phy);
	if (!b->mem0_virt || !b->mem1_virt) {
		dev_err(d->dev, "Memory alloc %d Byte ksc double buffer fail!\n",
			2 * size);
		ksc_free_memory(d);
		goto err;
	}

	if (b->buf_to_be_free.mem_size)
		queue_delayed_work(system_wq, &d->free_buffer_work, 3);

	b->alloced = true;

have_buffer:
	if (b->lut_phy)
		ksc_mem_sync(d->dev, b->lut_phy, b->lut_size);

	if (q->mode) {
		ret = ksc_dma_map(d, q->src_img.fd, b->src_item);
		if (ret) {
			dev_err(d->dev, "ksc_dma_map fail:%d\n", ret);
			ksc_free_memory(d);
			goto err;
		}
	}

	if (q->online_wb_en || q->mode) {
		ret = ksc_dma_map(d, q->dst_img.fd, b->dst_item);
		if (ret) {
			dev_err(d->dev, "ksc_dma_map dst_item fail:%d\n", ret);
			ksc_free_memory(d);
			goto err;
		}
	}

	ksc_reg_update_all(d, &d->ksc_para);

	if (q->mode) {
		wait_event_timeout(d->event_wait, d->be_finish,
				   msecs_to_jiffies(1000));
		d->be_finish = false;
		ksc_dma_unmap(b->dst_item);
		ksc_dma_unmap(b->src_item);
	} else if (q->online_wb_en) {
		unsigned long timeout = jiffies + msecs_to_jiffies(50);

		while (!ksc_is_reg_update_finish(d)) {
			if (time_after(jiffies, timeout)) {
				dev_err(d->dev, "Wait mem sync timout!\n");
				break;
			}
			usleep_range(26, 100);
		}
		ksc_dma_unmap(b->dst_item);
	}

	ret = 0;
	if (is_ksc_smooth_disp)
		is_ksc_smooth_disp = false;
	goto out;

err:
	ret = -1;
out:
	mutex_unlock(&d->dev_lock);
	return ret;
}

static struct ksc_buffers *ksc_get_buffers(struct ksc_device *d)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(150);

	if (!d->enabled && !d->ksc_para.is_first_frame) {
		while (!ksc_is_reg_update_finish(d)) {
			if (time_after(jiffies, timeout)) {
				dev_err(d->dev, "Wait reg update timout!\n");
				break;
			}
			usleep_range(26, 100);
		}
	}

	ksc_reg_switch_lut_buf(d, &d->ksc_para.para);

	return &d->ksc_para.buf;
}

static struct sunxi_ksc_para *ksc_get_para(struct ksc_device *d)
{
	mutex_lock(&d->dev_lock);
	mutex_unlock(&d->dev_lock);

	return &d->ksc_para.para;
}

/* ------------------------------------------------------------------------- */
/* online enable (exported)                                                  */
/* ------------------------------------------------------------------------- */

int ksc_online_enable(bool enable, int pix_fmt, int w, int h, int bit_depth,
		      int fps, int clk_freq)
{
	struct sunxi_ksc_para *q;
	int ret;

	if (!g_ksc) {
		pr_err("ksc device is not ready\n");
		return -1;
	}

	g_ksc->ksc_para.bw.clk_freq = clk_freq;
	g_ksc->ksc_para.bw.fps = fps;

	ret = g_ksc->enable(g_ksc, enable);
	if (ret) {
		dev_err(g_ksc->dev, "Enable %d ksc module fail:%d\n", enable, ret);
		return ret;
	}

	if (!enable) {
		atomic_dec(&g_ksc_open_count);
		return 0;
	}

	atomic_inc(&g_ksc_open_count);

	q = g_ksc->get_ksc_para(g_ksc);
	if (!q) {
		dev_err(g_ksc->dev, "get_ksc_para fail!\n");
		return 0;
	}

	q->ksc_en = false;
	q->scaler_en = false;
	q->mode = 0;
	*(u32 *)((unsigned char *)q + 172) = 128;
	*(u16 *)((unsigned char *)q + 194) = h - 1;
	*(u16 *)((unsigned char *)q + 192) = w - 1;

	if (!q->dst_w)
		q->dst_w = w;
	if (!q->dst_h)
		q->dst_h = h;

	q->src_w = w;
	q->src_h = h;
	q->dns_w = w;
	q->dns_h = h;
	q->bit_depth = bit_depth;

	if (g_ksc->drv_data->version == 0x110 &&
	    ((pix_fmt & ~3) != 8) && ((pix_fmt & ~1) != 12)) {
		dev_err(g_ksc->dev,
			"ksc110 online mode only support RGB888/YUV444:%u\n",
			pix_fmt);
		pix_fmt = 12;
	}

	q->pix_fmt = pix_fmt;
	q->file_fmt = pix_fmt;
	q->wb_fmt = pix_fmt;

	dev_info(g_ksc->dev,
		 "input size:[%u x %u] bpp:%u infmt:%u pixfmt:%u wbfmt:%u\n",
		 w, h, bit_depth, pix_fmt, q->pix_fmt, q->wb_fmt);

	ret = g_ksc->ksc_set_pq_para(g_ksc, q);
	if (ret < 0)
		dev_err(g_ksc->dev, "ksc set pq para fail!:%d\n", ret);

	ret = g_ksc->set_ksc_para(g_ksc, q);
	if (ret) {
		dev_err(g_ksc->dev, "ksc set_ksc_para fail!:%d\n", ret);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(ksc_online_enable);

/* ------------------------------------------------------------------------- */
/* file operations                                                           */
/* ------------------------------------------------------------------------- */

static int ksc_open(struct inode *inode, struct file *file)
{
	if (!g_ksc || !g_ksc->enable)
		return -ENODEV;

	atomic_inc(&g_ksc_open_count);

	return g_ksc->enable(g_ksc, true);
}

static int ksc_release(struct inode *inode, struct file *file)
{
	if (!g_ksc || !g_ksc->enable)
		return -ENODEV;

	if (atomic_dec_and_test(&g_ksc_open_count))
		g_ksc->enable(g_ksc, false);

	return 0;
}

static int ksc_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct ksc_buffers *b;

	if (!g_ksc)
		return -ENODEV;

	b = g_ksc->get_ksc_buffers(g_ksc);
	if (!b || !b->lut_virt || !b->lut_phy || !b->lut_size)
		return -ENOMEM;

	return ksc_mem_mmap(g_ksc->dev, vma, b->lut_virt, b->lut_phy,
			    b->lut_size);
}

/*
 * The vendor passes a 32 byte header whose first word is a user pointer to
 * the real argument structure, so the payload is fetched in a second step.
 */
struct ksc_ioctl_hdr {
	void __user *arg;
	unsigned char pad[28];
};

static long ksc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct ksc_ioctl_hdr hdr;
	struct sunxi_ksc_para *para;
	struct sunxi_ksc_online_para online;
	int ret;

	if (!g_ksc)
		return -ENODEV;

	if (copy_from_user(&hdr, (void __user *)arg, sizeof(hdr))) {
		dev_err(g_ksc->dev, "copy from user fail!\n");
		return -EFAULT;
	}

	switch (cmd) {
	case KSC_IOC_SET_ONLINE:
		if (copy_from_user(&online, hdr.arg, sizeof(online))) {
			dev_err(g_ksc->dev, "copy_from_user fail\n");
			return -EFAULT;
		}
		return ksc_online_enable(online.enable, online.in_fmt, online.w,
					 online.h, online.bit_depth,
					 online.fps, online.clk_freq);

	case KSC_IOC_SET_PARA:
		para = memdup_user(hdr.arg, sizeof(*para));
		if (IS_ERR(para))
			return PTR_ERR(para);
		ret = g_ksc->set_ksc_para(g_ksc, para);
		kfree(para);
		return ret;

	case KSC_IOC_GET_PARA:
		para = g_ksc->get_ksc_para(g_ksc);
		if (!para)
			return 0;
		if (copy_to_user(hdr.arg, para, sizeof(*para))) {
			dev_err(g_ksc->dev, "copy_to_user fail\n");
			return -EFAULT;
		}
		return 0;

	default:
		dev_err(g_ksc->dev, "Unkown cmd :0x%x\n", cmd);
		return 0;
	}
}

static const struct file_operations ksc_fops = {
	.owner = THIS_MODULE,
	.open = ksc_open,
	.release = ksc_release,
	.unlocked_ioctl = ksc_ioctl,
	.mmap = ksc_mmap,
};

/* ------------------------------------------------------------------------- */
/* platform device                                                           */
/* ------------------------------------------------------------------------- */

static void ksc_free_reserve_work(struct work_struct *work)
{
	/* Reserved memory is only used when the smooth display property is
	 * present; the bootmem region is released by the bootloader parse in
	 * that case, so there is nothing to do here. */
}

static int ksc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct device_node *top;
	struct ksc_device *d;
	u32 val;
	int i;

	d = devm_kzalloc(dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;

	d->dev = dev;
	d->drv_data = of_device_get_match_data(dev);
	if (!d->drv_data) {
		dev_err(dev, "Fail to get driver data!\n");
		return -EINVAL;
	}

	d->reg = of_iomap(np, 0);
	if (!d->reg) {
		dev_err(dev, "unable to map ksc registers\n");
		return -ENOMEM;
	}

	top = of_parse_phandle(np, "top", 0);
	if (top) {
		struct platform_device *top_pdev = of_find_device_by_node(top);

		if (top_pdev)
			d->ksc_top = &top_pdev->dev;
		else
			dev_info(dev, "unable to find tvdisp node\n");
	} else {
		dev_info(dev, "unable to find 'top' property\n");
	}

	d->irq_no = irq_of_parse_and_map(np, 0);
	if (!d->irq_no) {
		dev_err(dev, "irq_of_parse_and_map dec irq fail\n");
		return -EINVAL;
	}

	for (i = 0; i < KSC_MAX_CLKS; i++) {
		char name[16];

		snprintf(name, sizeof(name), "clk%d", i);
		d->clks[i] = devm_clk_get(dev, name);
		if (IS_ERR(d->clks[i]))
			dev_info(dev, "Fail to get clk%d\n", i);

		snprintf(name, sizeof(name), "rst%d", i);
		d->rsts[i] = devm_reset_control_get(dev, name);
		if (IS_ERR(d->rsts[i]))
			dev_info(dev, "Fail to get rst%d\n", i);

		snprintf(name, sizeof(name), "clk%d_freq", i);
		if (of_property_read_u32(np, name, &d->clk_freq[i]))
			d->clk_freq[i] = 0;
	}

	mutex_init(&d->dev_lock);
	init_waitqueue_head(&d->event_wait);
	INIT_DELAYED_WORK(&d->free_buffer_work, ksc_free_buffer_work);
	INIT_DELAYED_WORK(&d->free_reserve_mem, ksc_free_reserve_work);

	d->enable = ksc_do_enable;
	d->set_ksc_para = ksc_set_para;
	d->ksc_set_pq_para = ksc_set_pq_para;
	d->get_ksc_buffers = ksc_get_buffers;
	d->get_ksc_para = ksc_get_para;

	platform_set_drvdata(pdev, d);
	g_ksc = d;

	if (!of_property_read_u32(np, "flip_h", &val))
		d->ksc_para.para.flip_h = val;
	else
		dev_err(dev, "Get %s property failed\n", "flip_h");

	if (!of_property_read_u32(np, "flip_v", &val))
		d->ksc_para.para.flip_v = val;
	else
		dev_err(dev, "Get %s property failed\n", "flip_v");

	if (of_property_read_u32(np, "pix_fmt", &val))
		val = 0;

	if (!of_property_read_u32(np, "dst_w", &val))
		d->ksc_para.para.dst_w = val;
	if (!of_property_read_u32(np, "dst_h", &val))
		d->ksc_para.para.dst_h = val;

	dev_info(dev, "flip_h = %d, flip_v = %d\n",
		 d->ksc_para.para.flip_h, d->ksc_para.para.flip_v);

	if (!of_property_read_u32(np, "bw_ctrl_en",
				  &d->ksc_para.bw.bw_ctrl_en)) {
		if (d->ksc_para.bw.bw_ctrl_en) {
			u64 num;

			if (!d->ksc_para.bw.fps || !d->ksc_para.bw.clk_freq) {
				dev_err(dev, "invalid fps and clk_freq value!!!\n");
			} else {
				num = (u64)d->ksc_para.bw.fps *
				      d->ksc_para.bw.clk_freq;
				num = div_u64(num * 4, 1000) + 2;
				if (!of_property_read_u32(np, "bw_ctrl_num",
							  &d->ksc_para.bw.bw_ctrl_num) &&
				    d->ksc_para.bw.bw_ctrl_num >= num)
					dev_info(dev, "bw_ctrl_num = %d\n",
						 d->ksc_para.bw.bw_ctrl_num);
				else
					d->ksc_para.bw.bw_ctrl_num = (u32)num;
			}
		}
	} else {
		d->ksc_para.bw.bw_ctrl_en = 0;
	}

	return 0;
}

static int ksc_remove(struct platform_device *pdev)
{
	struct ksc_device *d = platform_get_drvdata(pdev);

	if (!d)
		return 0;

	if (d->enabled) {
		free_irq(d->irq_no, d);
		ksc_clk_setting(d, false);
		ksc_free_memory(d);
		d->enabled = false;
	}

	cancel_delayed_work_sync(&d->free_buffer_work);
	cancel_delayed_work_sync(&d->free_reserve_mem);
	iounmap(d->reg);
	if (g_ksc == d)
		g_ksc = NULL;

	return 0;
}

static void ksc_shutdown(struct platform_device *pdev)
{
}

static const struct ksc_drv_data ksc100_data = {
	.version = 0x100,
	.support_offline_ksc = true,
	.support_offline_up_scaler = true,
	.support_offline_arbitrary_angel_rotation = true,
	.support_offline_img_crop = true,
	.support_offline_img_flip = true,
};

static const struct ksc_drv_data ksc110_data = {
	.version = 0x110,
	.support_offline_ksc = true,
	.support_offline_up_scaler = true,
	.support_offline_arbitrary_angel_rotation = true,
	.support_offline_img_crop = true,
	.support_offline_img_flip = true,
};

static const struct of_device_id ksc_match[] = {
	{ .compatible = "allwinner,ksc100", .data = &ksc100_data },
	{ .compatible = "allwinner,ksc110", .data = &ksc110_data },
	{ }
};
MODULE_DEVICE_TABLE(of, ksc_match);

static struct platform_driver ksc_driver = {
	.probe = ksc_probe,
	.remove = ksc_remove,
	.shutdown = ksc_shutdown,
	.driver = {
		.name = "ksc",
		.of_match_table = ksc_match,
	},
};

static struct cdev *ksc_cdev;

static int __init ksc_module_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&g_ksc_devt, 0, 1, KSC_NAME);
	if (ret) {
		pr_err("ksc: alloc_chrdev_region fail:%d\n", ret);
		return ret;
	}

	ksc_cdev = cdev_alloc();
	if (!ksc_cdev) {
		pr_err("ksc: cdev_alloc fail\n");
		ret = -ENOMEM;
		goto err_region;
	}

	cdev_init(ksc_cdev, &ksc_fops);
	ksc_cdev->owner = THIS_MODULE;

	ret = cdev_add(ksc_cdev, g_ksc_devt, 1);
	if (ret) {
		pr_err("ksc: cdev_add fail:%d\n", ret);
		goto err_cdev;
	}

	g_ksc_class = class_create(THIS_MODULE, KSC_NAME);
	if (IS_ERR(g_ksc_class)) {
		ret = PTR_ERR(g_ksc_class);
		g_ksc_class = NULL;
		goto err_cdev;
	}

	g_ksc_dev = device_create(g_ksc_class, NULL, g_ksc_devt, NULL, KSC_NAME);
	if (IS_ERR(g_ksc_dev)) {
		ret = PTR_ERR(g_ksc_dev);
		g_ksc_dev = NULL;
		goto err_class;
	}

	return platform_driver_register(&ksc_driver);

err_class:
	class_destroy(g_ksc_class);
	g_ksc_class = NULL;
err_cdev:
	cdev_del(ksc_cdev);
	ksc_cdev = NULL;
err_region:
	unregister_chrdev_region(g_ksc_devt, 1);
	return ret;
}

static void __exit ksc_module_exit(void)
{
	platform_driver_unregister(&ksc_driver);

	if (g_ksc_dev) {
		device_destroy(g_ksc_class, g_ksc_devt);
		g_ksc_dev = NULL;
	}
	if (g_ksc_class) {
		class_destroy(g_ksc_class);
		g_ksc_class = NULL;
	}
	if (ksc_cdev) {
		cdev_del(ksc_cdev);
		ksc_cdev = NULL;
	}
	unregister_chrdev_region(g_ksc_devt, 1);
}

module_init(ksc_module_init);
module_exit(ksc_module_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("KSC driver");
MODULE_VERSION("1.0.0");
MODULE_AUTHOR("AllWinner");
