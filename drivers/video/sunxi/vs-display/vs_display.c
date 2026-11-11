// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allwinner H726 (sun50iw15p1) V-Silicon display backend - from-source port.
 *
 * The vendor module (vs-display.ko: "Display Device Driver" 0.1, Display Team)
 * is the kernel half of the TV display core.  It owns the five register
 * windows of the display block, the nine interrupt lines and forty-four
 * sub-interrupts, the twenty clocks and the dma-buf allocator that the
 * userspace display service drives through /dev/vs-display.
 *
 * Reproduced exactly:
 *   - the ioctl ABI, including the vendor's unusual encoding (two commands
 *     share number 8 with different direction and size) and the four request
 *     structures recovered from the module .BTF
 *   - the register segments, the interrupt group register offsets and bit
 *     lanes, and the enable/disable/ack sequences read out of the binary
 *   - dp_get_reg_segment(), the module's only export, as a plain (non-GPL)
 *     __ksymtab entry writing the five {size, pstart, vstart} triples
 *
 * Deliberately different:
 *   - the vendor's tracepoints (dp_tracing / irq_tracing / map_tracing) are
 *     debug instrumentation with no ABI surface and are not reproduced
 *   - the misc device keeps MISC_DYNAMIC_MINOR instead of the vendor's fixed
 *     minor; /dev/vs-display resolves the same way for userspace
 *   - list handling and dma-buf teardown use the normal kernel helpers rather
 *     than the vendor's open-coded variants
 */
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/eventfd.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

#include <asm/pgtable.h>

/* ------------------------------------------------------------------ */
/* userspace ABI                                                       */
/* ------------------------------------------------------------------ */

/* _IO('_', 5): the display service arms its epoll wait with this. */
#define DP_IOCTL_SET_EPOLL_READY	0x00005f05
/* _IOR('_', 1, struct dp_ioctl_device_info) */
#define DP_IOCTL_GET_DEV_INFO		0x80085f01
/* _IOR('_', 2, struct dp_ioctl_registers) */
#define DP_IOCTL_GET_REGISTERS		0x80185f02
/* _IOW('_', 3, struct dp_ioctl_irq) */
#define DP_IOCTL_ENABLE_IRQ		0x400c5f03
/* _IOW('_', 4, struct dp_ioctl_irq) */
#define DP_IOCTL_DISABLE_IRQ		0x400c5f04
/* _IOWR('_', 6, struct dp_ioctl_buf) */
#define DP_IOCTL_ALLOC_BUFFER		0xc0105f06
/* _IOW('_', 7, int) */
#define DP_IOCTL_FREE_BUFFER		0x40045f07
/* _IOWR('_', 8, struct dp_ioctl_dmabuf_fd_map) */
#define DP_IOCTL_MAP_DMABUF_FD		0xc00c5f08
/* _IOW('_', 8, int): same number as above, different size as in the vendor */
#define DP_IOCTL_UMMAP_DMABUF_FD	0x40045f08

#define DP_IOCTL_MAGIC			0x5f

struct dp_reg_segment {
	u32 size;
	u32 pstart;
	u32 vstart;
};

struct dp_ioctl_device_info {
	u32 argsz;
	u32 num_regions;
};

struct dp_ioctl_registers {
	u32 argsz;
	u32 index;
	u64 size;
	u64 offset;
};

struct dp_ioctl_irq {
	u32 argsz;
	u32 id;
	int fd;
};

struct dp_ioctl_buf {
	u32 argsz;
	u32 size;
	int fd;
	u32 dev_addr;
};

struct dp_ioctl_dmabuf_fd_map {
	u32 argsz;
	int dma_fd;
	u32 dev_addr;
};

/* ------------------------------------------------------------------ */
/* register and interrupt layout                                       */
/* ------------------------------------------------------------------ */

#define DP_SEG_NUM		5
#define DP_SEG_SOC		0
#define DP_SEG_AFBD		1
#define DP_SEG_CAP		2
#define DP_SEG_SVP		3
#define DP_SEG_PANEL		4

#define DP_IRQ_NUM		9
#define DP_IRQ_ID_NUM		44
#define DP_CLK_NUM		20

static const char *const dp_reg_names[DP_SEG_NUM] = {
	"soc", "afbd", "cap", "svp", "panel",
};

static const char *const dp_irq_names[DP_IRQ_NUM] = {
	"cap", "deint", "meter", "vnr", "panel0", "panel1", "proc", "scaler",
	"afbd_dec",
};

static const char *const dp_clk_names[DP_CLK_NUM] = {
	"disp_afbd", "disp_pwm", "disp_svp_dtl", "disp_svp_pclk_deint",
	"disp_svp_pclk_nr_meter", "disp_deint", "cap_dtl", "cap_ahb",
	"cap_mbus", "cap", "cap_dma", "cap_tcd3", "fe_ahb", "fe_mbus", "fe",
	"fe_axi", "fe_axi_bus", "fe_1296m", "fe_ihb", "fe_adc",
};

struct dp_segment {
	u32 size;
	phys_addr_t pstart;
	void __iomem *vaddr;
};

struct dp_buffer {
	int fd;
	size_t size;
	struct dma_buf *dbuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
	dma_addr_t dma_addr;
	struct list_head link;
};

/*
 * One sub-interrupt group.  The forty-four userspace ids do not map to groups
 * in order; the vendor dispatch is
 *   ids 0..4    -> VINCAP   (0)     ids 5..9    -> VNR     (3)
 *   ids 10..14  -> VDEINT   (1)     ids 15..18  -> VMETER  (2)
 *   ids 19..25  -> VSCALER  (7)     ids 26..30  -> VPROC   (6)
 *   ids 31..38  -> VPANEL0  (4)     ids 39..42  -> VPANEL1 (5)
 *   id  43      -> VDEC     (8)
 */
struct dp_irq_group {
	const char *name;
	u16 first_id;
	u8 n_ids;
	u8 seg;
	u8 ctrl_lane;		/* 8, 16, 24, or 0 for the fixed VDEC path */
	u32 ctrl;
	u32 ctrl2;
	u32 status;
	const u8 *shift;
	const u8 *bit;
};

/* s_irq_hw_cg* from the vendor binary, one shift per id of the group. */
static const u8 dp_shift_cap[5] = { 0, 1, 2, 3, 4 };
static const u8 dp_shift_deint[5] = { 0, 2, 4, 5, 6 };
static const u8 dp_shift_meter[4] = { 0, 1, 2, 3 };
static const u8 dp_shift_vnr[5] = { 0, 2, 4, 6, 7 };
static const u8 dp_shift_panel0[8] = { 0, 1, 2, 3, 4, 5, 6, 7 };
static const u8 dp_shift_panel1[4] = { 5, 0, 1, 3 };
static const u8 dp_shift_proc[5] = { 0, 1, 5, 6, 7 };
static const u8 dp_shift_scaler[7] = { 0, 2, 3, 5, 4, 6, 7 };
static const u8 dp_shift_vdec[1] = { 0 };

/* status bit -> userspace id, from the vendor get_sub_interrupt_* functions */
static const u8 dp_bit_cap[5] = { 0x01, 0x02, 0x04, 0x08, 0x10 };
static const u8 dp_bit_deint[5] = { 0x01, 0x04, 0x10, 0x20, 0x40 };
static const u8 dp_bit_meter[4] = { 0x01, 0x04, 0x10, 0x40 };
static const u8 dp_bit_vnr[5] = { 0x01, 0x04, 0x10, 0x40, 0x80 };
static const u8 dp_bit_panel0[8] = { 0x01, 0x02, 0x04, 0x08, 0x10, 0x20,
				     0x40, 0x80 };
static const u8 dp_bit_panel1[4] = { 0x20, 0x01, 0x02, 0x08 };
static const u8 dp_bit_proc[5] = { 0x01, 0x02, 0x20, 0x40, 0x80 };
static const u8 dp_bit_scaler[7] = { 0x01, 0x04, 0x08, 0x20, 0x10, 0x40,
				     0x80 };
static const u8 dp_bit_vdec[1] = { 0x01 };

#define DP_IRQ_GROUP(id0, n, seg_i, lane_i, ctrl_i, ctrl2_i, status_i, sh, bt) \
	{ .first_id = (id0), .n_ids = (n), .seg = (seg_i),		\
	  .ctrl_lane = (lane_i), .ctrl = (ctrl_i), .ctrl2 = (ctrl2_i),	\
	  .status = (status_i), .shift = (sh), .bit = (bt) }

static const struct dp_irq_group dp_groups[] = {
	/* VINCAP: register block at cap + 0x140000 */
	DP_IRQ_GROUP(0, 5, DP_SEG_CAP, 8, 0x14000c, 0, 0x140100,
		     dp_shift_cap, dp_bit_cap),
	/* VDEINT, svp + 0xc0000 */
	DP_IRQ_GROUP(10, 5, DP_SEG_SVP, 24, 0xc0008, 0, 0xc083c,
		     dp_shift_deint, dp_bit_deint),
	/* VMETER, svp + 0x0 */
	DP_IRQ_GROUP(15, 4, DP_SEG_SVP, 8, 0x18, 0, 0xa00,
		     dp_shift_meter, dp_bit_meter),
	/* VNR, svp + 0x1c0000 */
	DP_IRQ_GROUP(5, 5, DP_SEG_SVP, 16, 0x1c000c, 0x1c0008, 0x1c0470,
		     dp_shift_vnr, dp_bit_vnr),
	/* VPANEL0: same block as VNR, upper lane */
	DP_IRQ_GROUP(31, 8, DP_SEG_SVP, 24, 0x1c000c, 0x1c0008, 0x1c0470,
		     dp_shift_panel0, dp_bit_panel0),
	/* VPANEL1, shares the VNR block, upper lane */
	DP_IRQ_GROUP(39, 4, DP_SEG_SVP, 24, 0x1c000c, 0x1c0008, 0x1c0470,
		     dp_shift_panel1, dp_bit_panel1),
	/* VPROC, svp + 0x140000 / 0x140034 */
	DP_IRQ_GROUP(26, 5, DP_SEG_SVP, 16, 0x140034, 0, 0x140800,
		     dp_shift_proc, dp_bit_proc),
	/* VSCALER, svp + 0x300000 */
	DP_IRQ_GROUP(19, 7, DP_SEG_SVP, 16, 0x30004c, 0, 0x2c0100,
		     dp_shift_scaler, dp_bit_scaler),
	/* VDEC, afbd + 0x60, single fixed sub-interrupt */
	DP_IRQ_GROUP(43, 1, DP_SEG_AFBD, 0, 0x60, 0xf8, 0xf4,
		     dp_shift_vdec, dp_bit_vdec),
};

struct dp_ctx {
	struct device *dev;
	struct device *disptop;
	struct dp_segment seg[DP_SEG_NUM];
	struct clk *clk[DP_CLK_NUM];
	int irq[DP_IRQ_NUM];
	struct eventfd_ctx *efd[DP_IRQ_ID_NUM];
	struct miscdevice misc;
	char name[16];
	struct fasync_struct *async;
	bool epoll_ready;
	atomic_t refcnt;
	struct mutex lock;	/* protects the two buffer lists */
	struct list_head alloc_list;
	struct list_head fd_list;
	struct dentry *dbg_dir;
	struct dentry *dbg_reg;
	struct dentry *dbg_buf;
	u32 irq_cnt[DP_IRQ_ID_NUM];
	u32 irq_seen[DP_IRQ_ID_NUM];
};

static inline u32 dp_rd(struct dp_ctx *dp, int seg, u32 off)
{
	return readl(dp->seg[seg].vaddr + off);
}

static inline void dp_wr(struct dp_ctx *dp, int seg, u32 off, u32 val)
{
	writel(val, dp->seg[seg].vaddr + off);
}

static const struct dp_irq_group *dp_group_of(unsigned int id)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(dp_groups); i++) {
		const struct dp_irq_group *g = &dp_groups[i];

		if (id >= g->first_id && id < g->first_id + g->n_ids)
			return g;
	}
	return NULL;
}

static int dp_group_index(const struct dp_irq_group *g)
{
	return (int)(g - dp_groups);
}

/* ------------------------------------------------------------------ */
/* interrupt programming                                               */
/* ------------------------------------------------------------------ */

/*
 * The vendor's group initialisers.  Each one sets the polarity/edge and the
 * sub-interrupt defaults the hardware needs before any mask is opened.
 */
static void dp_irq_init_group(struct dp_ctx *dp, const struct dp_irq_group *g)
{
	u32 v;
	int idx = dp_group_index(g);

	switch (idx) {
	case 0:	/* VINCAP */
		v = dp_rd(dp, g->seg, g->ctrl) & 0xfffff0ffu;
		dp_wr(dp, g->seg, g->ctrl, v | 0x1000000u);
		v = (dp_rd(dp, g->seg, g->ctrl) & 0xffffff0fu) | 0x0fu |
		    (dp_rd(dp, g->seg, g->ctrl) & 0x000000f0u);
		dp_wr(dp, g->seg, g->ctrl, v);
		dp_wr(dp, g->seg, g->ctrl, dp_rd(dp, g->seg, g->ctrl) &
		      0xfffffff0u);
		v = dp_rd(dp, g->seg, 0x140004);
		dp_wr(dp, g->seg, 0x140004,
		      (v & 0xfffa0000u) | (v & 0xffffu) | 0x50000u);
		dp_wr(dp, g->seg, 0x140000, dp_rd(dp, g->seg, 0x140000) |
		      0x2000u);
		dp_wr(dp, g->seg, 0x140000, dp_rd(dp, g->seg, 0x140000) &
		      ~0x4000u);
		break;
	case 1:	/* VDEINT */
		v = dp_rd(dp, g->seg, g->ctrl) & 0xffff00ffu;
		dp_wr(dp, g->seg, g->ctrl, v | 0x10u);
		v = dp_rd(dp, g->seg, g->ctrl) & 0xf0ffffffu;
		dp_wr(dp, g->seg, g->ctrl, v | 0x0f000000u);
		dp_wr(dp, g->seg, g->ctrl, dp_rd(dp, g->seg, g->ctrl) &
		      0xf0ffffffu);
		v = dp_rd(dp, g->seg, 0xc0000) & 0xffff0fffu;
		dp_wr(dp, g->seg, 0xc0000, v);
		dp_wr(dp, g->seg, 0xc0000, dp_rd(dp, g->seg, 0xc0000) &
		      ~0x1000u);
		dp_wr(dp, g->seg, 0xc0000, dp_rd(dp, g->seg, 0xc0000) &
		      ~0x4000u);
		break;
	case 2:	/* VMETER */
		v = (dp_rd(dp, g->seg, g->ctrl) & 0xfffffff0u) | 0x1000000u;
		dp_wr(dp, g->seg, g->ctrl, v);
		v = (dp_rd(dp, g->seg, g->ctrl) & 0xfff0ffffu) | 0xf0000u;
		dp_wr(dp, g->seg, g->ctrl, v);
		dp_wr(dp, g->seg, g->ctrl, dp_rd(dp, g->seg, g->ctrl) &
		      0xfff0ffffu);
		v = dp_rd(dp, g->seg, 0x4);
		v |= 0x20000u | 0x80000u | 0x200000u | 0x4000000u | 0x2000000u;
		v &= ~0x800000u;
		dp_wr(dp, g->seg, 0x4, v);
		break;
	case 3:	/* VNR */
		v = dp_rd(dp, g->seg, g->ctrl2) & 0xfff0ffffu;
		dp_wr(dp, g->seg, g->ctrl2, v & 0xfffffff0u);
		v = dp_rd(dp, g->seg, g->ctrl) | 1u;
		dp_wr(dp, g->seg, g->ctrl, v & 0xfff0ffffu);
		v = (dp_rd(dp, g->seg, g->ctrl) & 0xfff0ffffu) | 0xf0000u;
		dp_wr(dp, g->seg, g->ctrl, v & 0xfff0ffffu);
		break;
	case 4:	/* VPANEL0, shares the VNR block */
		v = dp_rd(dp, g->seg, g->ctrl2) & 0xf0ffffffu;
		dp_wr(dp, g->seg, g->ctrl2, v & 0xfffff0ffu);
		v = dp_rd(dp, g->seg, g->ctrl) | 2u;
		dp_wr(dp, g->seg, g->ctrl, v & 0xf0ffffffu);
		break;
	case 5:	/* VPANEL1, shares the VNR block */
		v = dp_rd(dp, g->seg, g->ctrl2) & 0xf0ffffffu;
		dp_wr(dp, g->seg, g->ctrl2, v & 0xfffff0ffu);
		v = dp_rd(dp, g->seg, g->ctrl) | 2u;
		dp_wr(dp, g->seg, g->ctrl, v & 0xf0ffffffu);
		break;
	case 6:	/* VPROC */
		v = (dp_rd(dp, g->seg, g->ctrl) & 0xfffffff0u) | 0x80000000u;
		dp_wr(dp, g->seg, g->ctrl, v & 0xc0ffffffu);
		v = (dp_rd(dp, g->seg, g->ctrl) & 0xfff0ffffu) | 0xf0000u;
		dp_wr(dp, g->seg, g->ctrl, v & 0xfff0ffffu);
		dp_wr(dp, g->seg, g->ctrl + 8, 1u);
		dp_wr(dp, g->seg, g->ctrl + 12, 1u);
		dp_wr(dp, g->seg, g->ctrl + 16, 1u);
		dp_wr(dp, g->seg, g->ctrl + 20, 1u);
		v = dp_rd(dp, g->seg, g->ctrl) & ~0x1000000u;
		v &= ~0x2000000u;
		v = (v & 0xdfffffffu) | 0x20000000u;
		v &= ~0x40000000u;
		dp_wr(dp, g->seg, g->ctrl, v);
		break;
	case 7:	/* VSCALER */
		v = (dp_rd(dp, g->seg, g->ctrl) & 0xfffffff0u) | 0x80000000u;
		dp_wr(dp, g->seg, g->ctrl, v & 0xc0ffffffu);
		v = (dp_rd(dp, g->seg, g->ctrl) & 0xfff0ffffu) | 0xf0000u;
		dp_wr(dp, g->seg, g->ctrl, v & 0xfff0ffffu);
		dp_wr(dp, g->seg, g->ctrl + 4, 1u);
		dp_wr(dp, g->seg, g->ctrl + 8, 1u);
		dp_wr(dp, g->seg, g->ctrl + 12, 1u);
		v = dp_rd(dp, g->seg, g->ctrl + 16);
		v |= 0x80000000u;
		v = (v & 0x9fffffffu) | 0x20000000u;
		v = (v & 0xe7ffffffu) | 0x8000000u;
		v |= 0x4000000u;
		dp_wr(dp, g->seg, g->ctrl + 16, v);
		v = dp_rd(dp, g->seg, g->ctrl) & ~0x1000000u;
		v &= ~0x4000000u;
		v &= ~0x8000000u;
		v &= ~0x20000000u;
		v &= ~0x10000000u;
		v &= ~0x40000000u;
		v = (v & 0x80ffffffu) | ((v >> 24) & 0x7fu) << 24;
		dp_wr(dp, g->seg, g->ctrl, v);
		break;
	default:	/* VDEC */
		dp_wr(dp, g->seg, g->ctrl, dp_rd(dp, g->seg, g->ctrl) & ~1u);
		dp_wr(dp, g->seg, g->ctrl2, dp_rd(dp, g->seg, g->ctrl2) & ~1u);
		dp_wr(dp, g->seg, 0x10, dp_rd(dp, g->seg, 0x10) | 1u);
		break;
	}
}

static void dp_irq_init(struct dp_ctx *dp)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(dp_groups); i++)
		dp_irq_init_group(dp, &dp_groups[i]);
}

/*
 * Enable one sub-interrupt.  Each group keeps its enable mask in one byte
 * lane of a control register and the vendor pulses that lane (clear, set,
 * clear); only the resulting state is written here, which is the same value.
 */
static bool dp_irq_hw_enable(struct dp_ctx *dp, const struct dp_irq_group *g,
			     unsigned int id)
{
	unsigned int idx = id - g->first_id;
	unsigned int s = g->shift[idx];
	u32 bit = 1u << s;
	u32 v, lane;
	int gi = dp_group_index(g);

	if (!g->ctrl_lane) {
		dp_wr(dp, g->seg, g->ctrl, dp_rd(dp, g->seg, g->ctrl) | 1u);
		dp_wr(dp, g->seg, g->ctrl2, dp_rd(dp, g->seg, g->ctrl2) | 1u);
		dp_wr(dp, g->seg, 0x10, dp_rd(dp, g->seg, 0x10) | 1u);
		return true;
	}

	lane = g->ctrl_lane;
	v = dp_rd(dp, g->seg, g->ctrl);

	if (lane == 8) {
		v = (v & 0xffffff00u) | ((v & ~bit) & 0xffu) | (bit << 8);
		if (gi == 0)
			v &= ~bit;	/* VINCAP leaves the low bit clear */
	} else if (lane == 16) {
		v = (v & 0xff00ffffu) | ((v & ~(bit << 16)) & 0xff0000u) |
		    (bit << 16);
		if (gi == 6 || gi == 7)
			v |= bit;	/* these groups pulse the low bit too */
	} else {
		v = (v & 0x00ffffffu) | ((v & ~(bit << 24)) & 0xff000000u) |
		    (bit << 24);
	}
	dp_wr(dp, g->seg, g->ctrl, v);

	if (g->ctrl2) {
		v = dp_rd(dp, g->seg, g->ctrl2);
		if (lane == 8)
			v = (v & 0xffffff00u) | (bit << 8);
		else if (lane == 16)
			v = (v & 0xff00ffffu) | (bit << 16);
		else
			v = (v & 0x00ffffffu) | (bit << 24);
		dp_wr(dp, g->seg, g->ctrl2, v);
	}
	return true;
}

static bool dp_irq_hw_disable(struct dp_ctx *dp,
			      const struct dp_irq_group *g, unsigned int id)
{
	unsigned int idx = id - g->first_id;
	unsigned int s = g->shift[idx];
	u32 bit = 1u << s;
	u32 v, lane;

	if (!g->ctrl_lane) {
		dp_wr(dp, g->seg, g->ctrl, dp_rd(dp, g->seg, g->ctrl) & ~1u);
		dp_wr(dp, g->seg, g->ctrl2, dp_rd(dp, g->seg, g->ctrl2) & ~1u);
		dp_wr(dp, g->seg, 0x10, dp_rd(dp, g->seg, 0x10) | 1u);
		return true;
	}

	lane = g->ctrl_lane;
	v = dp_rd(dp, g->seg, g->ctrl);

	if (lane == 8)
		v &= ~(bit << 8) & ~bit;
	else if (lane == 16)
		v &= ~(bit << 16);
	else
		v &= ~(bit << 24) & ~(bit << 8);
	dp_wr(dp, g->seg, g->ctrl, v);

	if (g->ctrl2) {
		v = dp_rd(dp, g->seg, g->ctrl2);
		if (lane == 8)
			v &= ~(bit << 8);
		else if (lane == 16)
			v &= ~(bit << 16);
		else
			v &= ~(bit << 24);
		dp_wr(dp, g->seg, g->ctrl2, v);
	}
	return true;
}

static bool dp_irq_enable(struct dp_ctx *dp, unsigned int id)
{
	const struct dp_irq_group *g = dp_group_of(id);

	if (!g)
		return false;
	return dp_irq_hw_enable(dp, g, id);
}

static bool dp_irq_disable(struct dp_ctx *dp, unsigned int id)
{
	const struct dp_irq_group *g = dp_group_of(id);

	if (!g)
		return false;
	return dp_irq_hw_disable(dp, g, id);
}

/*
 * Read the group status, ack what fired and translate the status bits into
 * userspace ids.  Returns the number of ids written to @ids.
 */
static int dp_irq_get_sub(struct dp_ctx *dp, const struct dp_irq_group *g,
			  unsigned int *ids)
{
	unsigned int i, n = 0;
	u32 ctrl, status, fired;
	int gi = dp_group_index(g);

	ctrl = dp_rd(dp, g->seg, g->ctrl);
	status = dp_rd(dp, g->seg, g->status);

	switch (gi) {
	case 0:
	case 1:		/* masks in [15:8] */
		fired = status & (ctrl >> 8);
		break;
	case 2:
	case 6:
	case 7:		/* raw mask */
		fired = status & ctrl;
		break;
	case 3:		/* mask in [23:16] of the second control */
		ctrl = dp_rd(dp, g->seg, g->ctrl2);
		fired = status & (ctrl >> 16);
		break;
	case 4:		/* VPANEL0: upper half of the second control */
		ctrl = dp_rd(dp, g->seg, g->ctrl2);
		fired = status & (ctrl >> 16);
		break;
	case 5:		/* VPANEL1: byte lane 24, status shifted down by 8 */
		ctrl = dp_rd(dp, g->seg, g->ctrl2);
		fired = ((ctrl >> 24) & 0xffu) & (status >> 8);
		break;
	default:	/* VDEC: one bit */
		fired = status & 1u;
		break;
	}

	for (i = 0; i < g->n_ids; i++) {
		if (fired & g->bit[i])
			ids[n++] = g->first_id + i;
	}

	/* ack: clear the fired bits in the control register */
	if (!g->ctrl_lane) {
		dp_wr(dp, g->seg, g->status, status & ~fired);
		return n;
	}

	if (gi == 3 || gi == 4 || gi == 5) {
		u32 c2 = dp_rd(dp, g->seg, g->ctrl2);

		if (g->ctrl_lane == 24)
			c2 &= ~(fired << 24);
		else
			c2 &= ~(fired << 16);
		dp_wr(dp, g->seg, g->ctrl2, c2);
	} else {
		if (g->ctrl_lane == 8)
			ctrl &= ~(fired << 8);
		else if (g->ctrl_lane == 16)
			ctrl &= ~(fired << 16);
		else
			ctrl &= ~(fired << 24);
		dp_wr(dp, g->seg, g->ctrl, ctrl);
	}

	return n;
}

static irqreturn_t dp_irq_handler(int irq, void *data)
{
	struct dp_ctx *dp = data;
	unsigned int ids[8];
	int gi, n, i;

	for (gi = 0; gi < DP_IRQ_NUM; gi++) {
		if (dp->irq[gi] == irq)
			break;
	}
	if (gi == DP_IRQ_NUM) {
		dev_err(dp->dev, "unkown irq: %d\n", irq);
		return IRQ_NONE;
	}

	n = dp_irq_get_sub(dp, &dp_groups[gi], ids);
	if (!n) {
		dev_err(dp->dev, "%s: irq_cnt: %d\n", dp_groups[gi].name, 0);
		return IRQ_NONE;
	}

	for (i = 0; i < n; i++)
		dp_irq_disable(dp, ids[i]);

	for (i = 0; i < n; i++) {
		unsigned int id = ids[i];
		struct eventfd_ctx *efd;

		dp->irq_cnt[id]++;
		dp->irq_seen[id] = dp_rd(dp, dp_groups[gi].seg,
					 dp_groups[gi].status);

		efd = dp->efd[id];
		if (dp->epoll_ready && efd)
			eventfd_signal(efd, 1);
	}

	for (i = 0; i < n; i++)
		dp_irq_enable(dp, ids[i]);

	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* dma-buf buffer manager                                              */
/* ------------------------------------------------------------------ */


/*
 * system-uncached heap replacement.
 *
 * The vendor module allocates through dma_heap_find("system-uncached") and
 * dma_heap_buffer_alloc(), which are Android dma-buf heap client helpers.  This
 * kernel tree has neither the prototypes nor the exported symbols, so the
 * allocation is done here: coherent DMA memory exported as a dma-buf with the
 * same userspace contract (an fd, a DMA address, mmap and free by fd).
 */
struct dp_dmabuf_heap {
	size_t size;
	void *vaddr;
	dma_addr_t dma_addr;
	struct device *dev;
};

static void dp_dmabuf_release(struct dma_buf *dbuf)
{
	struct dp_dmabuf_heap *h = dbuf->priv;

	dma_free_attrs(h->dev, h->size, h->vaddr, h->dma_addr, 0);
	kfree(h);
}

static int dp_dmabuf_attach(struct dma_buf *dbuf,
			    struct dma_buf_attachment *attach)
{
	return 0;
}

static struct sg_table *dp_dmabuf_map(struct dma_buf_attachment *attach,
				      enum dma_data_direction dir)
{
	struct dp_dmabuf_heap *h = attach->dmabuf->priv;
	struct sg_table *sgt;
	int ret;

	sgt = kzalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return ERR_PTR(-ENOMEM);

	ret = dma_get_sgtable(attach->dev, sgt, h->vaddr, h->dma_addr,
			      h->size);
	if (ret) {
		kfree(sgt);
		return ERR_PTR(ret);
	}
	return sgt;
}

static void dp_dmabuf_unmap(struct dma_buf_attachment *attach,
			    struct sg_table *sgt, enum dma_data_direction dir)
{
	sg_free_table(sgt);
	kfree(sgt);
}

static int dp_dmabuf_mmap(struct dma_buf *dbuf, struct vm_area_struct *vma)
{
	struct dp_dmabuf_heap *h = dbuf->priv;

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	return remap_pfn_range(vma, vma->vm_start, h->dma_addr >> PAGE_SHIFT,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot);
}

static const struct dma_buf_ops dp_dmabuf_ops = {
	.attach		= dp_dmabuf_attach,
	.map_dma_buf	= dp_dmabuf_map,
	.unmap_dma_buf	= dp_dmabuf_unmap,
	.release	= dp_dmabuf_release,
	.mmap		= dp_dmabuf_mmap,
};

static struct dma_buf *dp_dmabuf_alloc(struct device *dev, size_t size)
{
	struct dp_dmabuf_heap *h;
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

	exp_info.ops = &dp_dmabuf_ops;
	exp_info.size = h->size;
	exp_info.flags = O_RDWR;
	exp_info.priv = h;

	dbuf = dma_buf_export(&exp_info);
	if (IS_ERR(dbuf)) {
		dma_free_attrs(dev, h->size, h->vaddr, h->dma_addr, 0);
		kfree(h);
		return dbuf;
	}
	return dbuf;
}

static int dp_buf_map(struct dp_ctx *dp, struct dp_buffer *b)
{
	b->attach = dma_buf_attach(b->dbuf, dp->dev);
	if (IS_ERR(b->attach))
		return PTR_ERR(b->attach);

	b->sgt = dma_buf_map_attachment(b->attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(b->sgt)) {
		int ret = PTR_ERR(b->sgt);

		b->sgt = NULL;
		dma_buf_detach(b->dbuf, b->attach);
		b->attach = NULL;
		return ret;
	}

	b->dma_addr = sg_dma_address(b->sgt->sgl);
	return 0;
}

static void dp_buf_unmap(struct dp_buffer *b)
{
	if (b->sgt) {
		dma_buf_unmap_attachment(b->attach, b->sgt, DMA_BIDIRECTIONAL);
		b->sgt = NULL;
	}
	if (b->attach) {
		dma_buf_detach(b->dbuf, b->attach);
		b->attach = NULL;
	}
}

static void dp_buf_destroy(struct dp_buffer *b)
{
	dp_buf_unmap(b);
	dma_buf_put(b->dbuf);
	kfree(b);
}

static int dp_buf_allocate(struct dp_ctx *dp, size_t size, int *out_fd,
			   u32 *out_addr)
{
	struct dma_buf *dbuf;
	struct dp_buffer *b;

	if (!size)
		return -EINVAL;

	dbuf = dp_dmabuf_alloc(dp->dev, size);
	if (IS_ERR(dbuf))
		return PTR_ERR(dbuf);

	b = kzalloc(sizeof(*b), GFP_KERNEL);
	if (!b) {
		dma_buf_put(dbuf);
		return -ENOMEM;
	}

	b->size = size;
	b->dbuf = dbuf;

	if (dp_buf_map(dp, b)) {
		dev_err(dp->dev, "map dma buf failed\n");
		kfree(b);
		dma_buf_put(dbuf);
		return -ENOMEM;
	}

	b->fd = dma_buf_fd(dbuf, O_CLOEXEC);
	if (b->fd < 0) {
		dp_buf_destroy(b);
		return b->fd;
	}

	mutex_lock(&dp->lock);
	list_add_tail(&b->link, &dp->alloc_list);
	mutex_unlock(&dp->lock);

	*out_fd = b->fd;
	*out_addr = (u32)b->dma_addr;
	return 0;
}

static struct dp_buffer *dp_buf_find(struct dp_ctx *dp, int fd,
				     struct list_head *head)
{
	struct dp_buffer *b;

	list_for_each_entry(b, head, link) {
		if (b->fd == fd)
			return b;
	}
	return NULL;
}

static int dp_buf_free(struct dp_ctx *dp, int fd)
{
	struct dp_buffer *b;

	mutex_lock(&dp->lock);
	b = dp_buf_find(dp, fd, &dp->alloc_list);
	if (b)
		list_del(&b->link);
	mutex_unlock(&dp->lock);

	if (!b)
		return 0;	/* the vendor also ignores unknown ids */

	dp_buf_destroy(b);
	return 0;
}

static void dp_buf_free_all(struct dp_ctx *dp)
{
	struct dp_buffer *b, *tmp;

	mutex_lock(&dp->lock);
	list_for_each_entry_safe(b, tmp, &dp->alloc_list, link) {
		list_del(&b->link);
		dp_buf_destroy(b);
	}
	mutex_unlock(&dp->lock);
}

static int dp_buf_map_fd(struct dp_ctx *dp, int fd, u32 *addr)
{
	struct dp_buffer *b;
	int ret;

	if (fd < 0)
		return -EINVAL;

	b = kzalloc(sizeof(*b), GFP_KERNEL);
	if (!b)
		return -ENOMEM;

	b->dbuf = dma_buf_get(fd);
	if (IS_ERR(b->dbuf)) {
		ret = PTR_ERR(b->dbuf);
		kfree(b);
		return ret;
	}

	b->fd = fd;
	b->size = b->dbuf->size;

	ret = dp_buf_map(dp, b);
	if (ret) {
		dma_buf_put(b->dbuf);
		kfree(b);
		return ret;
	}

	mutex_lock(&dp->lock);
	list_add_tail(&b->link, &dp->fd_list);
	mutex_unlock(&dp->lock);

	*addr = (u32)b->dma_addr;
	return 0;
}

static int dp_buf_ummap_fd(struct dp_ctx *dp, int fd)
{
	struct dp_buffer *b;

	mutex_lock(&dp->lock);
	b = dp_buf_find(dp, fd, &dp->fd_list);
	if (b)
		list_del(&b->link);
	mutex_unlock(&dp->lock);

	if (!b)
		return -ENOENT;

	dp_buf_destroy(b);
	return 0;
}

/* ------------------------------------------------------------------ */
/* file operations                                                     */
/* ------------------------------------------------------------------ */

static int dp_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct dp_ctx *dp = container_of(misc, struct dp_ctx, misc);

	file->private_data = dp;
	atomic_inc(&dp->refcnt);
	return 0;
}

static int dp_release(struct inode *inode, struct file *file)
{
	struct dp_ctx *dp = file->private_data;
	unsigned int i;

	if (!atomic_dec_and_test(&dp->refcnt))
		return 0;

	for (i = 0; i < DP_IRQ_ID_NUM; i++) {
		dp_irq_disable(dp, i);
		if (dp->efd[i]) {
			eventfd_ctx_put(dp->efd[i]);
			dp->efd[i] = NULL;
		}
	}
	dp->epoll_ready = false;
	dp_buf_free_all(dp);
	return 0;
}

static ssize_t dp_read(struct file *file, char __user *buf, size_t count,
		       loff_t *ppos)
{
	struct dp_ctx *dp = file->private_data;

	dev_dbg(dp->dev, "%s\n", __func__);
	return 0;
}

static ssize_t dp_write(struct file *file, const char __user *buf,
			size_t count, loff_t *ppos)
{
	struct dp_ctx *dp = file->private_data;

	dev_dbg(dp->dev, "%s\n", __func__);
	return count;
}

static int dp_ioctl_set_epoll(struct dp_ctx *dp)
{
	dp->epoll_ready = true;
	return 0;
}

static int dp_ioctl_dev_info(struct dp_ctx *dp, void __user *arg)
{
	struct dp_ioctl_device_info info;

	if (copy_from_user(&info, arg, sizeof(info)))
		return -EFAULT;

	info.argsz = sizeof(info);
	info.num_regions = DP_SEG_NUM;
	if (copy_to_user(arg, &info, sizeof(info)))
		return -EFAULT;
	return 0;
}

static int dp_ioctl_registers(struct dp_ctx *dp, void __user *arg)
{
	struct dp_ioctl_registers regs;

	if (copy_from_user(&regs, arg, sizeof(regs)))
		return -EFAULT;

	if (regs.index >= DP_SEG_NUM)
		return -EINVAL;

	regs.argsz = sizeof(regs);
	regs.size = dp->seg[regs.index].size;
	regs.offset = dp->seg[regs.index].pstart;
	if (copy_to_user(arg, &regs, sizeof(regs)))
		return -EFAULT;
	return 0;
}

static int dp_ioctl_enable_irq(struct dp_ctx *dp, void __user *arg)
{
	struct dp_ioctl_irq req;
	struct eventfd_ctx *efd;

	if (copy_from_user(&req, arg, sizeof(req)))
		return -EFAULT;
	if (req.id >= DP_IRQ_ID_NUM)
		return -EINVAL;

	efd = eventfd_ctx_fdget(req.fd);
	if (IS_ERR(efd)) {
		dev_err(dp->dev, "eanble isr %s failed, efd error !\n",
			dp_groups[0].name);
		return PTR_ERR(efd);
	}

	if (dp->efd[req.id] && dp->efd[req.id] != efd) {
		eventfd_ctx_put(dp->efd[req.id]);
		dp->efd[req.id] = efd;
		dev_info(dp->dev, "re-eanble isr with different efd !\n");
	} else if (!dp->efd[req.id]) {
		dp->efd[req.id] = efd;
		dev_info(dp->dev, "eanble isr id %u\n", req.id);
		dp_irq_enable(dp, req.id);
	} else {
		eventfd_ctx_put(efd);
	}

	return 0;
}

static int dp_ioctl_disable_irq(struct dp_ctx *dp, void __user *arg)
{
	struct dp_ioctl_irq req;

	if (copy_from_user(&req, arg, sizeof(req)))
		return -EFAULT;
	if (req.id >= DP_IRQ_ID_NUM)
		return -EINVAL;

	if (dp->efd[req.id]) {
		dp_irq_disable(dp, req.id);
		eventfd_ctx_put(dp->efd[req.id]);
		dp->efd[req.id] = NULL;
		return 0;
	}

	dev_info(dp->dev, "re-disable isr id %u\n", req.id);
	dp_irq_disable(dp, req.id);
	return 0;
}

static int dp_ioctl_alloc_buffer(struct dp_ctx *dp, void __user *arg)
{
	struct dp_ioctl_buf req;
	int ret;

	if (copy_from_user(&req, arg, sizeof(req)))
		return -EFAULT;

	ret = dp_buf_allocate(dp, req.size, &req.fd, &req.dev_addr);
	if (ret) {
		dev_err(dp->dev, "alloc buffer failed !\n");
		return ret;
	}

	req.argsz = sizeof(req);
	if (copy_to_user(arg, &req, sizeof(req))) {
		dev_err(dp->dev, "alloc buffer failed(copy to user) !\n");
		return -EFAULT;
	}
	return 0;
}

static int dp_ioctl_free_buffer(struct dp_ctx *dp, void __user *arg)
{
	int fd;

	if (copy_from_user(&fd, arg, sizeof(fd)))
		return -EFAULT;

	return dp_buf_free(dp, fd);
}

static int dp_ioctl_map_dmabuf(struct dp_ctx *dp, void __user *arg)
{
	struct dp_ioctl_dmabuf_fd_map req;
	int ret;

	if (copy_from_user(&req, arg, sizeof(req)))
		return -EFAULT;

	ret = dp_buf_map_fd(dp, req.dma_fd, &req.dev_addr);
	if (ret) {
		dev_err(dp->dev, "get dma map addr failed !\n");
		return ret;
	}

	if (copy_to_user(arg, &req, sizeof(req)))
		return -EFAULT;
	return 0;
}

static int dp_ioctl_ummap_dmabuf(struct dp_ctx *dp, void __user *arg)
{
	int fd;

	if (copy_from_user(&fd, arg, sizeof(fd)))
		return -EFAULT;

	if (!dp_buf_ummap_fd(dp, fd))
		return 0;

	dev_err(dp->dev, "unmap dma buf failed , fd : %d !\n", fd);
	return -EINVAL;
}

static long dp_unlocked_ioctl(struct file *file, unsigned int cmd,
			      unsigned long arg)
{
	struct dp_ctx *dp = file->private_data;
	void __user *uarg = (void __user *)arg;

	if (_IOC_TYPE(cmd) != DP_IOCTL_MAGIC) {
		dev_err(dp->dev, "%s: command type [%c] error!\n", __func__,
			(char)(cmd >> 8));
		return -ENOTTY;
	}

	switch (cmd) {
	case DP_IOCTL_SET_EPOLL_READY:
		return dp_ioctl_set_epoll(dp);
	case DP_IOCTL_GET_DEV_INFO:
		return dp_ioctl_dev_info(dp, uarg);
	case DP_IOCTL_GET_REGISTERS:
		return dp_ioctl_registers(dp, uarg);
	case DP_IOCTL_ENABLE_IRQ:
		return dp_ioctl_enable_irq(dp, uarg);
	case DP_IOCTL_DISABLE_IRQ:
		return dp_ioctl_disable_irq(dp, uarg);
	case DP_IOCTL_ALLOC_BUFFER:
		return dp_ioctl_alloc_buffer(dp, uarg);
	case DP_IOCTL_FREE_BUFFER:
		return dp_ioctl_free_buffer(dp, uarg);
	case DP_IOCTL_MAP_DMABUF_FD:
		return dp_ioctl_map_dmabuf(dp, uarg);
	case DP_IOCTL_UMMAP_DMABUF_FD:
		return dp_ioctl_ummap_dmabuf(dp, uarg);
	default:
		return -ENOTTY;
	}
}

/*
 * The register windows are mapped from their physical base: userspace passes
 * the segment pstart as the mmap offset and the vendor validates it against
 * one of the five windows before remapping.
 */
static int dp_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct dp_ctx *dp = file->private_data;
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	int i;

	for (i = 0; i < DP_SEG_NUM; i++) {
		if (!dp->seg[i].size)
			continue;
		if (off >= dp->seg[i].pstart &&
		    off + size <= dp->seg[i].pstart + dp->seg[i].size)
			break;
	}
	if (i == DP_SEG_NUM) {
		dev_err(dp->dev, "vs display mmap: phys address and size error [0x%lx, %lx]!!!\n",
			off, size);
		return -EINVAL;
	}

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	vma->vm_flags |= VM_IO;
	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff, size,
			    vma->vm_page_prot)) {
		dev_err(dp->dev, "remap_page_range error!!!\n");
		return -EAGAIN;
	}
	return 0;
}

static int dp_fasync(int fd, struct file *file, int on)
{
	struct dp_ctx *dp = file->private_data;

	return fasync_helper(fd, file, on, &dp->async);
}

static const struct file_operations dp_fops = {
	.owner		= THIS_MODULE,
	.open		= dp_open,
	.release	= dp_release,
	.read		= dp_read,
	.write		= dp_write,
	.unlocked_ioctl	= dp_unlocked_ioctl,
	.mmap		= dp_mmap,
	.fasync		= dp_fasync,
};

/* ------------------------------------------------------------------ */
/* debugfs                                                             */
/* ------------------------------------------------------------------ */

static int dp_debug_reg_show(struct seq_file *s, void *unused)
{
	struct dp_ctx *dp = s->private;
	int i;

	seq_puts(s, "\n---- registers ----\n");
	for (i = 0; i < DP_SEG_NUM; i++)
		seq_printf(s, "%-10s: pstart = 0x%pa vstart = %px size = 0x%x\n",
			   dp_reg_names[i], &dp->seg[i].pstart,
			   dp->seg[i].vaddr, dp->seg[i].size);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(dp_debug_reg);

static int dp_debug_buf_show(struct seq_file *s, void *unused)
{
	struct dp_ctx *dp = s->private;
	struct dp_buffer *b;

	seq_puts(s, "\n---- p_item ----\n");
	mutex_lock(&dp->lock);
	list_for_each_entry(b, &dp->alloc_list, link)
		seq_printf(s, "  fd %d dma %pad size %zu\n", b->fd,
			   &b->dma_addr, b->size);
	list_for_each_entry(b, &dp->fd_list, link)
		seq_printf(s, "  fd %d dma %pad size %zu (imported)\n", b->fd,
			   &b->dma_addr, b->size);
	mutex_unlock(&dp->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(dp_debug_buf);

static void dp_debug_init(struct dp_ctx *dp)
{
	dp->dbg_dir = debugfs_create_dir("vs-display", NULL);
	if (IS_ERR_OR_NULL(dp->dbg_dir)) {
		dp->dbg_dir = NULL;
		return;
	}

	dp->dbg_reg = debugfs_create_file("reg", 0444, dp->dbg_dir, dp,
					  &dp_debug_reg_fops);
	dp->dbg_buf = debugfs_create_file("buf", 0444, dp->dbg_dir, dp,
					  &dp_debug_buf_fops);
}

static void dp_debug_deinit(struct dp_ctx *dp)
{
	debugfs_remove(dp->dbg_reg);
	debugfs_remove(dp->dbg_buf);
	debugfs_remove(dp->dbg_dir);
	dp->dbg_reg = NULL;
	dp->dbg_buf = NULL;
	dp->dbg_dir = NULL;
}

/* ------------------------------------------------------------------ */
/* power management                                                    */
/* ------------------------------------------------------------------ */

/* display top / panel helpers exported by the tvpanel and sunxi_tvtop modules */
extern int sunxi_tvpanel_prepare(void);
extern int sunxi_tvpanel_unprepare(void);
extern int sunxi_tvpanel_enable(void);
extern int sunxi_tvpanel_disable(void);
extern bool sunxi_tvpanel_wait_init_done(void);
extern int sunxi_tvpanel_top_clk_enable(struct device *dev);
extern int sunxi_tvpanel_top_clk_disable(struct device *dev);
extern int sunxi_tvdisp_top_svp_enable(struct device *dev, unsigned int on);
extern int sunxi_tvdisp_top_afbd_enable(struct device *dev, unsigned int on);
extern int sunxi_tvtop_clk_get(int id);
extern int sunxi_tvtop_clk_put(int id);

/*
 * The vendor module attaches the display master to its IOMMU with
 * sunxi_enable_device_iommu(2, 1).  That helper is ported into the IOMMU
 * driver (drivers/iommu/sun50i-iommu.c) from bsp/drivers/iommu/
 * sunxi-iommu-v1.c, so the call programmes IOMMU_BYPASS_REG for master 2 the
 * way the vendor does instead of being dropped.
 */
#if IS_ENABLED(CONFIG_SUN50I_IOMMU)
extern void sunxi_enable_device_iommu(unsigned int master, unsigned int enable);
#else
static void sunxi_enable_device_iommu(unsigned int master, unsigned int enable)
{
	pr_info_once("vs-display: no IOMMU driver, master %u %s skipped\n",
		     master, enable ? "enable" : "disable");
}
#endif

static void dp_irq_free_all(struct dp_ctx *dp)
{
	int i;

	for (i = 0; i < DP_IRQ_NUM; i++) {
		if (dp->irq[i] >= 1) {
			devm_free_irq(dp->dev, dp->irq[i], dp);
			dp->irq[i] = 0;
		}
	}
}

static int dp_rt_resume(struct dp_ctx *dp)
{
	unsigned int i;
	int ret;

	for (i = 0; i < DP_CLK_NUM; i++) {
		if (!dp->clk[i])
			continue;
		ret = clk_prepare(dp->clk[i]);
		if (ret)
			goto err;
		ret = clk_enable(dp->clk[i]);
		if (ret) {
			clk_unprepare(dp->clk[i]);
			goto err;
		}
	}

	if (dp->clk[5])
		clk_set_rate(dp->clk[5], 150000000);
	if (dp->clk[6])
		clk_set_rate(dp->clk[6], 200000000);

	sunxi_tvpanel_wait_init_done();
	sunxi_tvpanel_prepare();
	sunxi_tvpanel_top_clk_enable(dp->disptop);
	sunxi_tvdisp_top_svp_enable(dp->disptop, 1);
	sunxi_tvdisp_top_afbd_enable(dp->disptop, 1);
	sunxi_tvtop_clk_get(1);
	sunxi_tvtop_clk_get(0);

	dp_irq_init(dp);
	for (i = 0; i < DP_IRQ_ID_NUM; i++)
		dp_irq_disable(dp, i);

	for (i = 0; i < DP_IRQ_NUM; i++) {
		if (dp->irq[i] < 1)
			continue;
		ret = devm_request_irq(dp->dev, dp->irq[i], dp_irq_handler,
				       IRQF_TRIGGER_HIGH, "vsdp_irq", dp);
		if (ret)
			dev_err(dp->dev, "could not register to interrupt %d\n",
				dp->irq[i]);
	}
	return 0;

err:
	while (i--) {
		clk_disable(dp->clk[i]);
		clk_unprepare(dp->clk[i]);
	}
	dev_err(dp->dev, "failed to enable the module clock %u, ret = %d\n",
		i, ret);
	return ret;
}

static int dp_rt_suspend(struct dp_ctx *dp)
{
	unsigned int i;

	dp_irq_free_all(dp);
	sunxi_tvtop_clk_put(1);
	sunxi_tvtop_clk_put(0);

	for (i = 0; i < DP_CLK_NUM; i++) {
		if (!dp->clk[i])
			continue;
		clk_disable(dp->clk[i]);
		clk_unprepare(dp->clk[i]);
	}

	sunxi_tvpanel_disable();
	sunxi_tvpanel_unprepare();
	sunxi_tvdisp_top_svp_enable(dp->disptop, 0);
	sunxi_tvdisp_top_afbd_enable(dp->disptop, 0);
	sunxi_tvpanel_top_clk_disable(dp->disptop);
	return 0;
}

/* ------------------------------------------------------------------ */
/* export                                                              */
/* ------------------------------------------------------------------ */

/*
 * The display service (and vs-osd) read the register window list through this
 * one export.  It is a plain __ksymtab entry in the vendor module, not GPL.
 */
int dp_get_reg_segment(struct dp_reg_segment *out)
{
	struct dp_ctx *dp;
	int i;

	/* the vendor uses a file-scope context; there is only one instance */
	extern struct dp_ctx *dp_ctx_global;

	dp = dp_ctx_global;
	if (!dp || !out)
		return 0;

	for (i = 0; i < DP_SEG_NUM; i++) {
		/* the u32 vstart field carries the register window base */
		out[i].size = dp->seg[i].size;
		out[i].pstart = (u32)dp->seg[i].pstart;
		out[i].vstart = (u32)(uintptr_t)dp->seg[i].vaddr;
	}
	return 0;
}
EXPORT_SYMBOL(dp_get_reg_segment);

/* ------------------------------------------------------------------ */
/* platform glue                                                       */
/* ------------------------------------------------------------------ */

struct dp_ctx *dp_ctx_global;

static int dp_parse_segments(struct dp_ctx *dp, struct platform_device *pdev)
{
	int i;

	for (i = 0; i < DP_SEG_NUM; i++) {
		struct resource *res;

		res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						   dp_reg_names[i]);
		if (!res) {
			dev_err(&pdev->dev, "No IO resource : %s\n",
				dp_reg_names[i]);
			continue;
		}

		dp->seg[i].pstart = res->start;
		dp->seg[i].size = resource_size(res);
		dp->seg[i].vaddr = devm_ioremap(&pdev->dev, res->start,
						resource_size(res));
		if (!dp->seg[i].vaddr) {
			dev_err(&pdev->dev, "ioremap %s failed\n",
				dp_reg_names[i]);
			dp->seg[i].size = 0;
		}
	}
	return 0;
}

static int dp_parse_irqs(struct dp_ctx *dp, struct platform_device *pdev)
{
	int i;

	for (i = 0; i < DP_IRQ_NUM; i++) {
		int irq;

		irq = platform_get_irq_byname(pdev, dp_irq_names[i]);
		if (irq <= 0)
			irq = platform_get_irq(pdev, i);
		dp->irq[i] = irq;
		if (irq <= 0)
			dev_err(&pdev->dev, "No IRQ resource\n");
	}
	return 0;
}

static int dp_parse_clocks(struct dp_ctx *dp, struct platform_device *pdev)
{
	int i;

	for (i = 0; i < DP_CLK_NUM; i++) {
		struct clk *clk = devm_clk_get(&pdev->dev, dp_clk_names[i]);

		if (IS_ERR(clk)) {
			dp->clk[i] = NULL;
			dev_dbg(&pdev->dev, "Failed to get the module clock %s\n",
				dp_clk_names[i]);
			continue;
		}
		dp->clk[i] = clk;
	}
	return 0;
}

static int dp_parse_disptop(struct dp_ctx *dp, struct platform_device *pdev)
{
	struct device_node *np;
	struct platform_device *top;

	np = of_parse_phandle(pdev->dev.of_node, "disptop", 0);
	if (!np) {
		dev_err(&pdev->dev, "Got node of disp top failed !\n");
		return -ENODEV;
	}

	top = of_find_device_by_node(np);
	if (!top) {
		dev_err(&pdev->dev, "Got pdev of disp top failed !\n");
		return -ENODEV;
	}

	dp->disptop = &top->dev;
	return 0;
}

static int dp_probe(struct platform_device *pdev)
{
	struct dp_ctx *dp;
	int ret;

	dev_info(&pdev->dev, "%s\n", __func__);

	dp = devm_kzalloc(&pdev->dev, sizeof(*dp), GFP_KERNEL);
	if (!dp)
		return -ENOMEM;

	dp->dev = &pdev->dev;
	mutex_init(&dp->lock);
	INIT_LIST_HEAD(&dp->alloc_list);
	INIT_LIST_HEAD(&dp->fd_list);
	atomic_set(&dp->refcnt, 1);

	dp_debug_init(dp);

	dp_parse_segments(dp, pdev);
	dp_parse_irqs(dp, pdev);
	dp_parse_clocks(dp, pdev);
	dp_parse_disptop(dp, pdev);

	strscpy(dp->name, "vs-display", sizeof(dp->name));
	dp->misc.name = dp->name;
	dp->misc.fops = &dp_fops;
	dp->misc.minor = MISC_DYNAMIC_MINOR;

	platform_set_drvdata(pdev, dp);
	dp_ctx_global = dp;

	ret = misc_register(&dp->misc);
	if (ret) {
		dev_err(dp->dev, "VS display could not register to misc driver\n");
		goto err;
	}

	dp_rt_resume(dp);
	sunxi_tvpanel_enable();
	sunxi_enable_device_iommu(2, 1);

	pm_runtime_set_active(dp->dev);
	pm_runtime_enable(dp->dev);

	dev_info(dp->dev, "vs-display ready\n");
	return 0;

err:
	dp_debug_deinit(dp);
	dp_ctx_global = NULL;
	return ret;
}

static int dp_remove(struct platform_device *pdev)
{
	struct dp_ctx *dp = platform_get_drvdata(pdev);

	if (!dp)
		return 0;

	dp_rt_suspend(dp);
	misc_deregister(&dp->misc);
	dp_debug_deinit(dp);
	dp_ctx_global = NULL;
	dev_info(dp->dev, "%s done.\n", __func__);
	return 0;
}

static void dp_shutdown(struct platform_device *pdev)
{
	struct dp_ctx *dp = platform_get_drvdata(pdev);

	if (!dp)
		return;

	dp_rt_suspend(dp);
	misc_deregister(&dp->misc);
	dp_debug_deinit(dp);
	dev_info(dp->dev, "%s done.\n", __func__);
}

static const struct of_device_id dp_of_match[] = {
	{ .compatible = "vs,display" },
	{ }
};
MODULE_DEVICE_TABLE(of, dp_of_match);

static struct platform_driver dp_driver = {
	.probe		= dp_probe,
	.remove		= dp_remove,
	.shutdown	= dp_shutdown,
	.driver		= {
		.name		= "vs-display",
		.of_match_table	= dp_of_match,
	},
};

module_platform_driver(dp_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Display Device Driver");
MODULE_VERSION("0.1");
MODULE_AUTHOR("Display Team");
MODULE_ALIAS("platform:vs-display");
