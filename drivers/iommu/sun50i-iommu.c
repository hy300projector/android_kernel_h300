// SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause)
// Copyright (C) 2016-2018, Allwinner Technology CO., LTD.
// Copyright (C) 2019-2020, Cerno

#include <linux/bitfield.h>
#include <linux/bug.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/property.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/types.h>

#define IOMMU_RESET_REG			0x010
#define IOMMU_RESET_RELEASE_ALL			0xffffffff
#define IOMMU_ENABLE_REG		0x020
#define IOMMU_ENABLE_ENABLE			BIT(0)

#define IOMMU_BYPASS_REG		0x030
#define IOMMU_AUTO_GATING_REG		0x040
#define IOMMU_AUTO_GATING_ENABLE		BIT(0)

#define IOMMU_WBUF_CTRL_REG		0x044
#define IOMMU_OOO_CTRL_REG		0x048
#define IOMMU_4KB_BDY_PRT_CTRL_REG	0x04c
#define IOMMU_TTB_REG			0x050
#define IOMMU_TLB_ENABLE_REG		0x060
#define IOMMU_TLB_PREFETCH_REG		0x070
#define IOMMU_TLB_PREFETCH_MASTER_ENABLE(m)	BIT(m)

#define IOMMU_TLB_FLUSH_REG		0x080
#define IOMMU_TLB_FLUSH_PTW_CACHE		BIT(17)
#define IOMMU_TLB_FLUSH_MACRO_TLB		BIT(16)
#define IOMMU_TLB_FLUSH_MICRO_TLB(i)		(BIT(i) & GENMASK(5, 0))

#define IOMMU_TLB_IVLD_ADDR_REG		0x090
#define IOMMU_TLB_IVLD_ADDR_MASK_REG	0x094
#define IOMMU_TLB_IVLD_ENABLE_REG	0x098
#define IOMMU_TLB_IVLD_ENABLE_ENABLE		BIT(0)

/*
 * From revision 0x12 the TLB invalidation takes a start/end range, and from
 * 0x14 the PC invalidation does too.  The vendor driver uses these instead of
 * the address/mask registers above on this silicon.
 */
#define IOMMU_V14_TLB_IVLD_START_REG	0x088
#define IOMMU_V14_TLB_IVLD_END_REG	0x08c

#define IOMMU_PC_IVLD_ADDR_REG		0x0a0
#define IOMMU_V14_PC_IVLD_START_REG	0x0a4
#define IOMMU_PC_IVLD_ENABLE_REG	0x0a8
#define IOMMU_PC_IVLD_ENABLE_ENABLE		BIT(0)
#define IOMMU_V14_PC_IVLD_END_REG	0x0ac

/*
 * Per-master profiling counters.  Each master owns a 0x10-byte block at
 * 0x230 + 0x10m: access (lo, hi 11 bits), hit (lo, hi 11 bits), total latency
 * (lo, hi 18 bits) and max latency (single word).  The macro-TLB and PTW cache
 * counters live at 0x2d0 and 0x2e0 in the same low/high pairs.
 */
#define IOMMU_V14_PROF_ACCESS(m)	(0x230 + (m) * 0x10)
#define IOMMU_V14_PROF_ACCESS_HI(m)	(0x234 + (m) * 0x10)
#define IOMMU_V14_PROF_HIT(m)		(0x238 + (m) * 0x10)
#define IOMMU_V14_PROF_HIT_HI(m)	(0x23c + (m) * 0x10)
#define IOMMU_V14_PROF_LAT(m)		(0x300 + (m) * 0x10)
#define IOMMU_V14_PROF_LAT_HI(m)	(0x304 + (m) * 0x10)
#define IOMMU_V14_PROF_MAX_LAT(m)	(0x308 + (m) * 0x10)

#define IOMMU_V14_PROF_MACRO_ACCESS	0x2d0
#define IOMMU_V14_PROF_MACRO_ACCESS_HI	0x2d4
#define IOMMU_V14_PROF_MACRO_HIT	0x2d8
#define IOMMU_V14_PROF_MACRO_HIT_HI	0x2dc
#define IOMMU_V14_PROF_PTW_ACCESS	0x2e0
#define IOMMU_V14_PROF_PTW_ACCESS_HI	0x2e4
#define IOMMU_V14_PROF_PTW_HIT		0x2e8
#define IOMMU_V14_PROF_PTW_HIT_HI	0x2ec

#define IOMMU_DM_AUT_CTRL_REG(d)	(0x0b0 + ((d) / 2) * 4)
#define IOMMU_DM_AUT_CTRL_RD_UNAVAIL(d, m)	(1 << (((d & 1) * 16) + ((m) * 2)))
#define IOMMU_DM_AUT_CTRL_WR_UNAVAIL(d, m)	(1 << (((d & 1) * 16) + ((m) * 2) + 1))

#define IOMMU_DM_AUT_OVWT_REG		0x0d0
#define IOMMU_INT_ENABLE_REG		0x100
#define IOMMU_INT_CLR_REG		0x104
#define IOMMU_INT_STA_REG		0x108
#define IOMMU_INT_ERR_ADDR_REG(i)	(0x110 + (i) * 4)
#define IOMMU_INT_ERR_ADDR_L1_REG	0x130
#define IOMMU_INT_ERR_ADDR_L2_REG	0x134
#define IOMMU_INT_ERR_DATA_L1_REG	0x170
#define IOMMU_INT_ERR_DATA_L2_REG	0x174
#define IOMMU_INT_ERR_DATA_REG(i)	(0x150 + (i) * 4)
#define IOMMU_L1PG_INT_REG		0x0180
#define IOMMU_L2PG_INT_REG		0x0184

#define IOMMU_INT_INVALID_L2PG			BIT(17)
#define IOMMU_INT_INVALID_L1PG			BIT(16)
#define IOMMU_INT_MASTER_PERMISSION(m)		BIT(m)
#define IOMMU_INT_MASTER_MASK			(IOMMU_INT_MASTER_PERMISSION(0) | \
						 IOMMU_INT_MASTER_PERMISSION(1) | \
						 IOMMU_INT_MASTER_PERMISSION(2) | \
						 IOMMU_INT_MASTER_PERMISSION(3) | \
						 IOMMU_INT_MASTER_PERMISSION(4) | \
						 IOMMU_INT_MASTER_PERMISSION(5))
#define IOMMU_INT_MASK				(IOMMU_INT_INVALID_L1PG | \
						 IOMMU_INT_INVALID_L2PG | \
						 IOMMU_INT_MASTER_MASK)

/*
 * v14 registers the vendor driver writes from its of_device_id match data.
 * Neither has a name in the H6 register map.
 */
#define IOMMU_VERSION_REG		0x000
#define IOMMU_V14_CTRL_084_REG		0x084
#define IOMMU_V14_CTRL_09C_REG		0x09c

#define PT_ENTRY_SIZE			sizeof(u32)

#define NUM_DT_ENTRIES			4096
#define DT_SIZE				(NUM_DT_ENTRIES * PT_ENTRY_SIZE)

#define NUM_PT_ENTRIES			256
#define PT_SIZE				(NUM_PT_ENTRIES * PT_ENTRY_SIZE)

struct sun50i_iommu_variant {
	u32 version;
	u32 prefetch;
	u32 ctrl_084;
	u32 ctrl_09c;
	/* Permission lives in PTE[3:2] (read, write) instead of an ACI at [7:4] */
	bool direct_prot;
	/* Names the vendor prints for a faulting master, indexed by master id */
	const char *const *master_names;
	unsigned int nr_masters;
};

struct sun50i_iommu {
	struct iommu_device iommu;

	/* Lock to modify the IOMMU registers */
	spinlock_t iommu_lock;

	struct device *dev;
	void __iomem *base;
	struct reset_control *reset;
	struct clk *clk;

	struct iommu_domain *domain;
	struct iommu_group *group;
	struct kmem_cache *pt_pool;

	const struct sun50i_iommu_variant *variant;

	/* IOVA ranges the vendor keeps out of the allocator, from DT */
	struct {
		dma_addr_t start;
		size_t size;
	} resv[4];
	unsigned int nr_resv;
};

struct sun50i_iommu_domain {
	struct iommu_domain domain;

	/* Number of devices attached to the domain */
	refcount_t refcnt;

	/* L1 Page Table */
	u32 *dt;
	dma_addr_t dt_dma;

	struct sun50i_iommu *iommu;
};

static struct sun50i_iommu_domain *to_sun50i_domain(struct iommu_domain *domain)
{
	return container_of(domain, struct sun50i_iommu_domain, domain);
}

static struct sun50i_iommu *sun50i_iommu_from_dev(struct device *dev)
{
	return dev_iommu_priv_get(dev);
}

static u32 iommu_read(struct sun50i_iommu *iommu, u32 offset)
{
	return readl(iommu->base + offset);
}

static void iommu_write(struct sun50i_iommu *iommu, u32 offset, u32 value)
{
	writel(value, iommu->base + offset);
}

/*
 * The Allwinner H6 IOMMU uses a 2-level page table.
 *
 * The first level is the usual Directory Table (DT), that consists of
 * 4096 4-bytes Directory Table Entries (DTE), each pointing to a Page
 * Table (PT).
 *
 * Each PT consits of 256 4-bytes Page Table Entries (PTE), each
 * pointing to a 4kB page of physical memory.
 *
 * The IOMMU supports a single DT, pointed by the IOMMU_TTB_REG
 * register that contains its physical address.
 */

#define SUN50I_IOVA_DTE_MASK	GENMASK(31, 20)
#define SUN50I_IOVA_PTE_MASK	GENMASK(19, 12)
#define SUN50I_IOVA_PAGE_MASK	GENMASK(11, 0)

static u32 sun50i_iova_get_dte_index(dma_addr_t iova)
{
	return FIELD_GET(SUN50I_IOVA_DTE_MASK, iova);
}

static u32 sun50i_iova_get_pte_index(dma_addr_t iova)
{
	return FIELD_GET(SUN50I_IOVA_PTE_MASK, iova);
}

static u32 sun50i_iova_get_page_offset(dma_addr_t iova)
{
	return FIELD_GET(SUN50I_IOVA_PAGE_MASK, iova);
}

/*
 * Each Directory Table Entry has a Page Table address and a valid
 * bit:

 * +---------------------+-----------+-+
 * | PT address          | Reserved  |V|
 * +---------------------+-----------+-+
 *  31:10 - Page Table address
 *   9:2  - Reserved
 *   1:0  - 1 if the entry is valid
 */

#define SUN50I_DTE_PT_ADDRESS_MASK	GENMASK(31, 10)
#define SUN50I_DTE_PT_ATTRS		GENMASK(1, 0)
#define SUN50I_DTE_PT_VALID		1

static phys_addr_t sun50i_dte_get_pt_address(u32 dte)
{
	return (phys_addr_t)dte & SUN50I_DTE_PT_ADDRESS_MASK;
}

static bool sun50i_dte_is_pt_valid(u32 dte)
{
	return (dte & SUN50I_DTE_PT_ATTRS) == SUN50I_DTE_PT_VALID;
}

static u32 sun50i_mk_dte(dma_addr_t pt_dma)
{
	return (pt_dma & SUN50I_DTE_PT_ADDRESS_MASK) | SUN50I_DTE_PT_VALID;
}

/*
 * Each PTE has a Page address, an authority index and a valid bit:
 *
 * +----------------+-----+-----+-----+---+-----+
 * | Page address   | Rsv | ACI | Rsv | V | Rsv |
 * +----------------+-----+-----+-----+---+-----+
 *  31:12 - Page address
 *  11:8  - Reserved
 *   7:4  - Authority Control Index
 *   3:2  - Reserved
 *     1  - 1 if the entry is valid
 *     0  - Reserved
 *
 * The way permissions work is that the IOMMU has 16 "domains" that
 * can be configured to give each masters either read or write
 * permissions through the IOMMU_DM_AUT_CTRL_REG registers. The domain
 * 0 seems like the default domain, and its permissions in the
 * IOMMU_DM_AUT_CTRL_REG are only read-only, so it's not really
 * useful to enforce any particular permission.
 *
 * Each page entry will then have a reference to the domain they are
 * affected to, so that we can actually enforce them on a per-page
 * basis.
 *
 * In order to make it work with the IOMMU framework, we will be using
 * 4 different domains, starting at 1: RD_WR, RD, WR and NONE
 * depending on the permission we want to enforce. Each domain will
 * have each master setup in the same way, since the IOMMU framework
 * doesn't seem to restrict page access on a per-device basis. And
 * then we will use the relevant domain index when generating the page
 * table entry depending on the permissions we want to be enforced.
 */

enum sun50i_iommu_aci {
	SUN50I_IOMMU_ACI_DO_NOT_USE = 0,
	SUN50I_IOMMU_ACI_NONE,
	SUN50I_IOMMU_ACI_RD,
	SUN50I_IOMMU_ACI_WR,
	SUN50I_IOMMU_ACI_RD_WR,
};

#define SUN50I_PTE_PAGE_ADDRESS_MASK	GENMASK(31, 12)
#define SUN50I_PTE_ACI_MASK		GENMASK(7, 4)
#define SUN50I_PTE_PAGE_VALID		BIT(1)

static phys_addr_t sun50i_pte_get_page_address(u32 pte)
{
	return (phys_addr_t)pte & SUN50I_PTE_PAGE_ADDRESS_MASK;
}

static enum sun50i_iommu_aci sun50i_get_pte_aci(u32 pte)
{
	return FIELD_GET(SUN50I_PTE_ACI_MASK, pte);
}

static bool sun50i_pte_is_page_valid(u32 pte)
{
	return pte & SUN50I_PTE_PAGE_VALID;
}

static u32 sun50i_mk_pte(phys_addr_t page, int prot,
			 const struct sun50i_iommu_variant *variant)
{
	enum sun50i_iommu_aci aci;
	u32 flags = 0;

	page &= SUN50I_PTE_PAGE_ADDRESS_MASK;

	if (variant && variant->direct_prot) {
		/*
		 * v14: the vendor encodes the permission directly in PTE[3:2]
		 * (bit 2 = read, bit 3 = write) by storing (prot & 3) << 2, and
		 * never programs DM_AUT_CTRL; its page-table dumper reads the
		 * attribute back as pte & 0xc.
		 */
		flags = FIELD_PREP(GENMASK(3, 2), prot & 3);
		return page | flags | SUN50I_PTE_PAGE_VALID;
	}

	if ((prot & (IOMMU_READ | IOMMU_WRITE)) == (IOMMU_READ | IOMMU_WRITE))
		aci = SUN50I_IOMMU_ACI_RD_WR;
	else if (prot & IOMMU_READ)
		aci = SUN50I_IOMMU_ACI_RD;
	else if (prot & IOMMU_WRITE)
		aci = SUN50I_IOMMU_ACI_WR;
	else
		aci = SUN50I_IOMMU_ACI_NONE;

	flags |= FIELD_PREP(SUN50I_PTE_ACI_MASK, aci);
	page &= SUN50I_PTE_PAGE_ADDRESS_MASK;
	return page | flags | SUN50I_PTE_PAGE_VALID;
}

static void sun50i_table_flush(struct sun50i_iommu_domain *sun50i_domain,
			       void *vaddr, unsigned int count)
{
	struct sun50i_iommu *iommu = sun50i_domain->iommu;
	dma_addr_t dma = virt_to_phys(vaddr);
	size_t size = count * PT_ENTRY_SIZE;

	dma_sync_single_for_device(iommu->dev, dma, size, DMA_TO_DEVICE);
}

static int sun50i_iommu_flush_all_tlb(struct sun50i_iommu *iommu)
{
	u32 reg;
	int ret;

	assert_spin_locked(&iommu->iommu_lock);

	iommu_write(iommu,
		    IOMMU_TLB_FLUSH_REG,
		    IOMMU_TLB_FLUSH_PTW_CACHE |
		    IOMMU_TLB_FLUSH_MACRO_TLB |
		    IOMMU_TLB_FLUSH_MICRO_TLB(5) |
		    IOMMU_TLB_FLUSH_MICRO_TLB(4) |
		    IOMMU_TLB_FLUSH_MICRO_TLB(3) |
		    IOMMU_TLB_FLUSH_MICRO_TLB(2) |
		    IOMMU_TLB_FLUSH_MICRO_TLB(1) |
		    IOMMU_TLB_FLUSH_MICRO_TLB(0));

	ret = readl_poll_timeout_atomic(iommu->base + IOMMU_TLB_FLUSH_REG,
					reg, !reg,
					1, 2000);
	if (ret)
		dev_warn(iommu->dev, "TLB Flush timed out!\n");

	return ret;
}

static void sun50i_iommu_flush_iotlb_all(struct iommu_domain *domain)
{
	struct sun50i_iommu_domain *sun50i_domain = to_sun50i_domain(domain);
	struct sun50i_iommu *iommu = sun50i_domain->iommu;
	unsigned long flags;

	/*
	 * At boot, we'll have a first call into .flush_iotlb_all right after
	 * .probe_device, and since we link our (single) domain to our iommu in
	 * the .attach_device callback, we don't have that pointer set.
	 *
	 * It shouldn't really be any trouble to ignore it though since we flush
	 * all caches as part of the device powerup.
	 */
	if (!iommu)
		return;

	spin_lock_irqsave(&iommu->iommu_lock, flags);
	sun50i_iommu_flush_all_tlb(iommu);
	spin_unlock_irqrestore(&iommu->iommu_lock, flags);
}

static void sun50i_iommu_iotlb_sync(struct iommu_domain *domain,
				    struct iommu_iotlb_gather *gather)
{
	sun50i_iommu_flush_iotlb_all(domain);
}

static int sun50i_iommu_enable(struct sun50i_iommu *iommu)
{
	struct sun50i_iommu_domain *sun50i_domain;
	unsigned long flags;
	int ret;

	if (!iommu->domain)
		return 0;

	sun50i_domain = to_sun50i_domain(iommu->domain);

	ret = reset_control_deassert(iommu->reset);
	if (ret)
		return ret;

	ret = clk_prepare_enable(iommu->clk);
	if (ret)
		goto err_reset_assert;

	spin_lock_irqsave(&iommu->iommu_lock, flags);

	iommu_write(iommu, IOMMU_TTB_REG, sun50i_domain->dt_dma);
	if (iommu->variant) {
		/*
		 * Vendor v14 order: prefetch mask from match data, then the two
		 * revision-gated control registers.
		 */
		iommu_write(iommu, IOMMU_TLB_PREFETCH_REG,
			    iommu->variant->prefetch);
		if (iommu->variant->version > 0x11)
			iommu_write(iommu, IOMMU_V14_CTRL_084_REG,
				    iommu->variant->ctrl_084);
		if (iommu->variant->version > 0x13)
			iommu_write(iommu, IOMMU_V14_CTRL_09C_REG,
				    iommu->variant->ctrl_09c);
	} else {
		iommu_write(iommu, IOMMU_TLB_PREFETCH_REG,
			    IOMMU_TLB_PREFETCH_MASTER_ENABLE(0) |
			    IOMMU_TLB_PREFETCH_MASTER_ENABLE(1) |
			    IOMMU_TLB_PREFETCH_MASTER_ENABLE(2) |
			    IOMMU_TLB_PREFETCH_MASTER_ENABLE(3) |
			    IOMMU_TLB_PREFETCH_MASTER_ENABLE(4) |
			    IOMMU_TLB_PREFETCH_MASTER_ENABLE(5));
	}
	iommu_write(iommu, IOMMU_BYPASS_REG, 0);
	iommu_write(iommu, IOMMU_INT_ENABLE_REG, IOMMU_INT_MASK);
	/*
	 * v14 does not use the domain permission model: the vendor never
	 * programs these registers and relies on their reset defaults.
	 */
	if (!iommu->variant) {
	iommu_write(iommu, IOMMU_DM_AUT_CTRL_REG(SUN50I_IOMMU_ACI_NONE),
		    IOMMU_DM_AUT_CTRL_RD_UNAVAIL(SUN50I_IOMMU_ACI_NONE, 0) |
		    IOMMU_DM_AUT_CTRL_WR_UNAVAIL(SUN50I_IOMMU_ACI_NONE, 0) |
		    IOMMU_DM_AUT_CTRL_RD_UNAVAIL(SUN50I_IOMMU_ACI_NONE, 1) |
		    IOMMU_DM_AUT_CTRL_WR_UNAVAIL(SUN50I_IOMMU_ACI_NONE, 1) |
		    IOMMU_DM_AUT_CTRL_RD_UNAVAIL(SUN50I_IOMMU_ACI_NONE, 2) |
		    IOMMU_DM_AUT_CTRL_WR_UNAVAIL(SUN50I_IOMMU_ACI_NONE, 2) |
		    IOMMU_DM_AUT_CTRL_RD_UNAVAIL(SUN50I_IOMMU_ACI_NONE, 3) |
		    IOMMU_DM_AUT_CTRL_WR_UNAVAIL(SUN50I_IOMMU_ACI_NONE, 3) |
		    IOMMU_DM_AUT_CTRL_RD_UNAVAIL(SUN50I_IOMMU_ACI_NONE, 4) |
		    IOMMU_DM_AUT_CTRL_WR_UNAVAIL(SUN50I_IOMMU_ACI_NONE, 4) |
		    IOMMU_DM_AUT_CTRL_RD_UNAVAIL(SUN50I_IOMMU_ACI_NONE, 5) |
		    IOMMU_DM_AUT_CTRL_WR_UNAVAIL(SUN50I_IOMMU_ACI_NONE, 5));

	iommu_write(iommu, IOMMU_DM_AUT_CTRL_REG(SUN50I_IOMMU_ACI_RD),
		    IOMMU_DM_AUT_CTRL_WR_UNAVAIL(SUN50I_IOMMU_ACI_RD, 0) |
		    IOMMU_DM_AUT_CTRL_WR_UNAVAIL(SUN50I_IOMMU_ACI_RD, 1) |
		    IOMMU_DM_AUT_CTRL_WR_UNAVAIL(SUN50I_IOMMU_ACI_RD, 2) |
		    IOMMU_DM_AUT_CTRL_WR_UNAVAIL(SUN50I_IOMMU_ACI_RD, 3) |
		    IOMMU_DM_AUT_CTRL_WR_UNAVAIL(SUN50I_IOMMU_ACI_RD, 4) |
		    IOMMU_DM_AUT_CTRL_WR_UNAVAIL(SUN50I_IOMMU_ACI_RD, 5));

	iommu_write(iommu, IOMMU_DM_AUT_CTRL_REG(SUN50I_IOMMU_ACI_WR),
		    IOMMU_DM_AUT_CTRL_RD_UNAVAIL(SUN50I_IOMMU_ACI_WR, 0) |
		    IOMMU_DM_AUT_CTRL_RD_UNAVAIL(SUN50I_IOMMU_ACI_WR, 1) |
		    IOMMU_DM_AUT_CTRL_RD_UNAVAIL(SUN50I_IOMMU_ACI_WR, 2) |
		    IOMMU_DM_AUT_CTRL_RD_UNAVAIL(SUN50I_IOMMU_ACI_WR, 3) |
		    IOMMU_DM_AUT_CTRL_RD_UNAVAIL(SUN50I_IOMMU_ACI_WR, 4) |
		    IOMMU_DM_AUT_CTRL_RD_UNAVAIL(SUN50I_IOMMU_ACI_WR, 5));
	}


	ret = sun50i_iommu_flush_all_tlb(iommu);
	if (ret) {
		spin_unlock_irqrestore(&iommu->iommu_lock, flags);
		goto err_clk_disable;
	}

	iommu_write(iommu, IOMMU_AUTO_GATING_REG, IOMMU_AUTO_GATING_ENABLE);
	iommu_write(iommu, IOMMU_ENABLE_REG, IOMMU_ENABLE_ENABLE);

	spin_unlock_irqrestore(&iommu->iommu_lock, flags);

	return 0;

err_clk_disable:
	clk_disable_unprepare(iommu->clk);

err_reset_assert:
	reset_control_assert(iommu->reset);

	return ret;
}

static void sun50i_iommu_disable(struct sun50i_iommu *iommu)
{
	unsigned long flags;

	spin_lock_irqsave(&iommu->iommu_lock, flags);

	iommu_write(iommu, IOMMU_ENABLE_REG, 0);
	iommu_write(iommu, IOMMU_TTB_REG, 0);

	spin_unlock_irqrestore(&iommu->iommu_lock, flags);

	clk_disable_unprepare(iommu->clk);
	reset_control_assert(iommu->reset);
}

static void *sun50i_iommu_alloc_page_table(struct sun50i_iommu *iommu,
					   gfp_t gfp)
{
	dma_addr_t pt_dma;
	u32 *page_table;

	page_table = kmem_cache_zalloc(iommu->pt_pool, gfp);
	if (!page_table)
		return ERR_PTR(-ENOMEM);

	pt_dma = dma_map_single(iommu->dev, page_table, PT_SIZE, DMA_TO_DEVICE);
	if (dma_mapping_error(iommu->dev, pt_dma)) {
		dev_err(iommu->dev, "Couldn't map L2 Page Table\n");
		kmem_cache_free(iommu->pt_pool, page_table);
		return ERR_PTR(-ENOMEM);
	}

	/* We rely on the physical address and DMA address being the same */
	WARN_ON(pt_dma != virt_to_phys(page_table));

	return page_table;
}

static void sun50i_iommu_free_page_table(struct sun50i_iommu *iommu,
					 u32 *page_table)
{
	phys_addr_t pt_phys = virt_to_phys(page_table);

	dma_unmap_single(iommu->dev, pt_phys, PT_SIZE, DMA_TO_DEVICE);
	kmem_cache_free(iommu->pt_pool, page_table);
}

static u32 *sun50i_dte_get_page_table(struct sun50i_iommu_domain *sun50i_domain,
				      dma_addr_t iova, gfp_t gfp)
{
	struct sun50i_iommu *iommu = sun50i_domain->iommu;
	u32 *page_table;
	u32 *dte_addr;
	u32 old_dte;
	u32 dte;

	dte_addr = &sun50i_domain->dt[sun50i_iova_get_dte_index(iova)];
	dte = *dte_addr;
	if (sun50i_dte_is_pt_valid(dte)) {
		phys_addr_t pt_phys = sun50i_dte_get_pt_address(dte);
		return (u32 *)phys_to_virt(pt_phys);
	}

	page_table = sun50i_iommu_alloc_page_table(iommu, gfp);
	if (IS_ERR(page_table))
		return page_table;

	dte = sun50i_mk_dte(virt_to_phys(page_table));
	old_dte = cmpxchg(dte_addr, 0, dte);
	if (old_dte) {
		phys_addr_t installed_pt_phys =
			sun50i_dte_get_pt_address(old_dte);
		u32 *installed_pt = phys_to_virt(installed_pt_phys);
		u32 *drop_pt = page_table;

		page_table = installed_pt;
		dte = old_dte;
		sun50i_iommu_free_page_table(iommu, drop_pt);
	}

	sun50i_table_flush(sun50i_domain, page_table, NUM_PT_ENTRIES);
	sun50i_table_flush(sun50i_domain, dte_addr, 1);

	return page_table;
}

static int sun50i_iommu_map(struct iommu_domain *domain, unsigned long iova,
			    phys_addr_t paddr, size_t size, int prot, gfp_t gfp)
{
	struct sun50i_iommu_domain *sun50i_domain = to_sun50i_domain(domain);
	struct sun50i_iommu *iommu = sun50i_domain->iommu;
	u32 pte_index;
	u32 *page_table, *pte_addr;
	int ret = 0;

	page_table = sun50i_dte_get_page_table(sun50i_domain, iova, gfp);
	if (IS_ERR(page_table)) {
		ret = PTR_ERR(page_table);
		goto out;
	}

	pte_index = sun50i_iova_get_pte_index(iova);
	pte_addr = &page_table[pte_index];
	if (unlikely(sun50i_pte_is_page_valid(*pte_addr))) {
		phys_addr_t page_phys = sun50i_pte_get_page_address(*pte_addr);
		dev_err(iommu->dev,
			"iova %pad already mapped to %pa cannot remap to %pa prot: %#x\n",
			&iova, &page_phys, &paddr, prot);
		ret = -EBUSY;
		goto out;
	}

	*pte_addr = sun50i_mk_pte(paddr, prot, iommu->variant);
	sun50i_table_flush(sun50i_domain, pte_addr, 1);

out:
	return ret;
}

static size_t sun50i_iommu_unmap(struct iommu_domain *domain, unsigned long iova,
				 size_t size, struct iommu_iotlb_gather *gather)
{
	struct sun50i_iommu_domain *sun50i_domain = to_sun50i_domain(domain);
	phys_addr_t pt_phys;
	u32 *pte_addr;
	u32 dte;

	dte = sun50i_domain->dt[sun50i_iova_get_dte_index(iova)];
	if (!sun50i_dte_is_pt_valid(dte))
		return 0;

	pt_phys = sun50i_dte_get_pt_address(dte);
	pte_addr = (u32 *)phys_to_virt(pt_phys) + sun50i_iova_get_pte_index(iova);

	if (!sun50i_pte_is_page_valid(*pte_addr))
		return 0;

	memset(pte_addr, 0, sizeof(*pte_addr));
	sun50i_table_flush(sun50i_domain, pte_addr, 1);

	return SZ_4K;
}

static phys_addr_t sun50i_iommu_iova_to_phys(struct iommu_domain *domain,
					     dma_addr_t iova)
{
	struct sun50i_iommu_domain *sun50i_domain = to_sun50i_domain(domain);
	phys_addr_t pt_phys;
	u32 *page_table;
	u32 dte, pte;

	dte = sun50i_domain->dt[sun50i_iova_get_dte_index(iova)];
	if (!sun50i_dte_is_pt_valid(dte))
		return 0;

	pt_phys = sun50i_dte_get_pt_address(dte);
	page_table = (u32 *)phys_to_virt(pt_phys);
	pte = page_table[sun50i_iova_get_pte_index(iova)];
	if (!sun50i_pte_is_page_valid(pte))
		return 0;

	return sun50i_pte_get_page_address(pte) +
		sun50i_iova_get_page_offset(iova);
}

static struct iommu_domain *sun50i_iommu_domain_alloc(unsigned type)
{
	struct sun50i_iommu_domain *sun50i_domain;

	if (type != IOMMU_DOMAIN_DMA &&
	    type != IOMMU_DOMAIN_UNMANAGED)
		return NULL;

	sun50i_domain = kzalloc(sizeof(*sun50i_domain), GFP_KERNEL);
	if (!sun50i_domain)
		return NULL;

	sun50i_domain->dt = (u32 *)__get_free_pages(GFP_KERNEL | __GFP_ZERO,
						    get_order(DT_SIZE));
	if (!sun50i_domain->dt)
		goto err_free_domain;

	refcount_set(&sun50i_domain->refcnt, 1);

	sun50i_domain->domain.geometry.aperture_start = 0;
	sun50i_domain->domain.geometry.aperture_end = DMA_BIT_MASK(32);
	sun50i_domain->domain.geometry.force_aperture = true;

	return &sun50i_domain->domain;

err_free_domain:
	kfree(sun50i_domain);

	return NULL;
}

static void sun50i_iommu_domain_free(struct iommu_domain *domain)
{
	struct sun50i_iommu_domain *sun50i_domain = to_sun50i_domain(domain);

	free_pages((unsigned long)sun50i_domain->dt, get_order(DT_SIZE));
	sun50i_domain->dt = NULL;

	kfree(sun50i_domain);
}

static int sun50i_iommu_attach_domain(struct sun50i_iommu *iommu,
				      struct sun50i_iommu_domain *sun50i_domain)
{
	iommu->domain = &sun50i_domain->domain;
	sun50i_domain->iommu = iommu;

	sun50i_domain->dt_dma = dma_map_single(iommu->dev, sun50i_domain->dt,
					       DT_SIZE, DMA_TO_DEVICE);
	if (dma_mapping_error(iommu->dev, sun50i_domain->dt_dma)) {
		dev_err(iommu->dev, "Couldn't map L1 Page Table\n");
		return -ENOMEM;
	}

	return sun50i_iommu_enable(iommu);
}

static void sun50i_iommu_detach_domain(struct sun50i_iommu *iommu,
				       struct sun50i_iommu_domain *sun50i_domain)
{
	unsigned int i;

	for (i = 0; i < NUM_DT_ENTRIES; i++) {
		phys_addr_t pt_phys;
		u32 *page_table;
		u32 *dte_addr;
		u32 dte;

		dte_addr = &sun50i_domain->dt[i];
		dte = *dte_addr;
		if (!sun50i_dte_is_pt_valid(dte))
			continue;

		memset(dte_addr, 0, sizeof(*dte_addr));
		sun50i_table_flush(sun50i_domain, dte_addr, 1);

		pt_phys = sun50i_dte_get_pt_address(dte);
		page_table = phys_to_virt(pt_phys);
		sun50i_iommu_free_page_table(iommu, page_table);
	}


	sun50i_iommu_disable(iommu);

	dma_unmap_single(iommu->dev, virt_to_phys(sun50i_domain->dt),
			 DT_SIZE, DMA_TO_DEVICE);

	iommu->domain = NULL;
}

static void sun50i_iommu_detach_device(struct iommu_domain *domain,
				       struct device *dev)
{
	struct sun50i_iommu_domain *sun50i_domain = to_sun50i_domain(domain);
	struct sun50i_iommu *iommu = dev_iommu_priv_get(dev);

	dev_dbg(dev, "Detaching from IOMMU domain\n");

	if (iommu->domain != domain)
		return;

	if (refcount_dec_and_test(&sun50i_domain->refcnt))
		sun50i_iommu_detach_domain(iommu, sun50i_domain);
}

static int sun50i_iommu_attach_device(struct iommu_domain *domain,
				      struct device *dev)
{
	struct sun50i_iommu_domain *sun50i_domain = to_sun50i_domain(domain);
	struct sun50i_iommu *iommu;

	iommu = sun50i_iommu_from_dev(dev);
	if (!iommu)
		return -ENODEV;

	dev_dbg(dev, "Attaching to IOMMU domain\n");

	refcount_inc(&sun50i_domain->refcnt);

	if (iommu->domain == domain)
		return 0;

	if (iommu->domain)
		sun50i_iommu_detach_device(iommu->domain, dev);

	sun50i_iommu_attach_domain(iommu, sun50i_domain);

	return 0;
}

static struct iommu_device *sun50i_iommu_probe_device(struct device *dev)
{
	struct sun50i_iommu *iommu;

	iommu = sun50i_iommu_from_dev(dev);
	if (!iommu)
		return ERR_PTR(-ENODEV);

	return &iommu->iommu;
}

static void sun50i_iommu_release_device(struct device *dev) {}

static struct iommu_group *sun50i_iommu_device_group(struct device *dev)
{
	struct sun50i_iommu *iommu = sun50i_iommu_from_dev(dev);

	return iommu_group_ref_get(iommu->group);
}

static int sun50i_iommu_of_xlate(struct device *dev,
				 struct of_phandle_args *args)
{
	struct platform_device *iommu_pdev = of_find_device_by_node(args->np);
	unsigned id = args->args[0];

	dev_iommu_priv_set(dev, platform_get_drvdata(iommu_pdev));

	return iommu_fwspec_add_ids(dev, &id, 1);
}

static const struct iommu_ops sun50i_iommu_ops = {
	.pgsize_bitmap	= SZ_4K,
	.attach_dev	= sun50i_iommu_attach_device,
	.detach_dev	= sun50i_iommu_detach_device,
	.device_group	= sun50i_iommu_device_group,
	.domain_alloc	= sun50i_iommu_domain_alloc,
	.domain_free	= sun50i_iommu_domain_free,
	.flush_iotlb_all = sun50i_iommu_flush_iotlb_all,
	.iotlb_sync	= sun50i_iommu_iotlb_sync,
	.iova_to_phys	= sun50i_iommu_iova_to_phys,
	.map		= sun50i_iommu_map,
	.of_xlate	= sun50i_iommu_of_xlate,
	.probe_device	= sun50i_iommu_probe_device,
	.release_device	= sun50i_iommu_release_device,
	.unmap		= sun50i_iommu_unmap,
};

static const char *sun50i_iommu_master_name(struct sun50i_iommu *iommu,
					    unsigned int master)
{
	if (!iommu->variant || !iommu->variant->master_names ||
	    master >= iommu->variant->nr_masters)
		return NULL;

	return iommu->variant->master_names[master];
}

static void sun50i_iommu_report_fault(struct sun50i_iommu *iommu,
				      const char *kind, unsigned int master,
				      phys_addr_t iova, unsigned int prot,
				      u32 data)
{
	const char *name = sun50i_iommu_master_name(iommu, master);
	const char *dir = (prot == IOMMU_FAULT_WRITE) ? "wr" : "rd";

	/*
	 * The vendor prints "<master> Invalid Authority" per master, then
	 * "Bug is in <module> module, invalid address/data/id".  Name the
	 * module here so a fault points at the block that caused it.
	 */
	if (name)
		dev_err(iommu->dev,
			"%s for %pad (master %u \"%s\", dir %s, data %#x)\n",
			kind, &iova, master, name, dir, data);
	else
		dev_err(iommu->dev,
			"%s for %pad (master %u, dir %s, data %#x)\n",
			kind, &iova, master, dir, data);

	if (iommu->domain)
		report_iommu_fault(iommu->domain, iommu->dev, iova, prot);
	else
		dev_err(iommu->dev, "Page fault while iommu not attached to any domain?\n");
}

static phys_addr_t sun50i_iommu_handle_pt_irq(struct sun50i_iommu *iommu,
					      const char *kind,
					      unsigned addr_reg,
					      unsigned blame_reg,
					      unsigned data_reg)
{
	phys_addr_t iova;
	unsigned master;
	u32 blame, data;

	assert_spin_locked(&iommu->iommu_lock);

	iova = iommu_read(iommu, addr_reg);
	blame = iommu_read(iommu, blame_reg);
	data = iommu_read(iommu, data_reg);
	master = ilog2(blame & IOMMU_INT_MASTER_MASK);

	/*
	 * If the address is not in the page table, we can't get what
	 * operation triggered the fault. Assume it's a read
	 * operation.
	 */
	sun50i_iommu_report_fault(iommu, kind, master, iova,
				  IOMMU_FAULT_READ, data);

	return iova;
}

static phys_addr_t sun50i_iommu_handle_perm_irq(struct sun50i_iommu *iommu)
{
	enum sun50i_iommu_aci aci;
	phys_addr_t iova;
	unsigned master;
	unsigned dir;
	u32 blame, data;

	assert_spin_locked(&iommu->iommu_lock);

	blame = iommu_read(iommu, IOMMU_INT_STA_REG);
	master = ilog2(blame & IOMMU_INT_MASTER_MASK);
	iova = iommu_read(iommu, IOMMU_INT_ERR_ADDR_REG(master));
	data = iommu_read(iommu, IOMMU_INT_ERR_DATA_REG(master));

	if (iommu->variant && iommu->variant->direct_prot) {
		/*
		 * v14: the fault data carries the permission stored in the PTE
		 * (bit 2 read, bit 3 write), not an ACI.  A write-only entry
		 * failing means the master tried to read, and vice versa.
		 */
		if ((data & BIT(3)) && !(data & BIT(2)))
			dir = IOMMU_FAULT_READ;
		else if ((data & BIT(2)) && !(data & BIT(3)))
			dir = IOMMU_FAULT_WRITE;
		else
			dir = IOMMU_FAULT_READ;
		goto report;
	}

	aci = sun50i_get_pte_aci(data);
	switch (aci) {
		/*
		 * If we are in the read-only domain, then it means we
		 * tried to write.
		 */
	case SUN50I_IOMMU_ACI_RD:
		dir = IOMMU_FAULT_WRITE;
		break;

		/*
		 * If we are in the write-only domain, then it means
		 * we tried to read.
		 */
	case SUN50I_IOMMU_ACI_WR:

		/*
		 * If we are in the domain without any permission, we
		 * can't really tell. Let's default to a read
		 * operation.
		 */
	case SUN50I_IOMMU_ACI_NONE:

		/* WTF? */
	case SUN50I_IOMMU_ACI_RD_WR:
	default:
		dir = IOMMU_FAULT_READ;
		break;
	}

report:
	sun50i_iommu_report_fault(iommu, "Invalid authority", master, iova,
				  dir, data);

	return iova;
}

/*
 * Fault recovery exactly as the vendor's sunxi_iommu_irq does it for a
 * revision >= 0x14: invalidate the TLB over [iova, iova + 16K) through the
 * 0x88/0x8c range, invalidate the page cache over [iova, iova + 2M) through
 * 0xa4/0xac, clear the low 18 status bits, then pulse the reset register with
 * the 16-bit mask the vendor uses.
 */
static void sun50i_iommu_v14_fault_cleanup(struct sun50i_iommu *iommu,
					   u32 status, u32 l1_status,
					   u32 l2_status, phys_addr_t iova)
{
	u32 reg;

	iommu_write(iommu, IOMMU_V14_TLB_IVLD_START_REG, iova);
	iommu_write(iommu, IOMMU_V14_TLB_IVLD_END_REG, iova + SZ_16K);
	iommu_write(iommu, IOMMU_TLB_IVLD_ENABLE_REG,
		    IOMMU_TLB_IVLD_ENABLE_ENABLE);

	if (readl_poll_timeout_atomic(iommu->base + IOMMU_TLB_IVLD_ENABLE_REG,
				      reg, !(reg & IOMMU_TLB_IVLD_ENABLE_ENABLE),
				      1, 1000))
		dev_warn(iommu->dev, "TLB invalidation timed out\n");

	iommu_write(iommu, IOMMU_V14_PC_IVLD_START_REG, iova);
	iommu_write(iommu, IOMMU_V14_PC_IVLD_END_REG, iova + SZ_2M);
	iommu_write(iommu, IOMMU_PC_IVLD_ENABLE_REG,
		    IOMMU_PC_IVLD_ENABLE_ENABLE);

	if (readl_poll_timeout_atomic(iommu->base + IOMMU_PC_IVLD_ENABLE_REG,
				      reg, !(reg & IOMMU_PC_IVLD_ENABLE_ENABLE),
				      1, 1000))
		dev_warn(iommu->dev, "PC invalidation timed out\n");

	iommu_write(iommu, IOMMU_INT_CLR_REG, status & 0x3ffff);

	iommu_write(iommu, IOMMU_RESET_REG,
		    (~(l1_status | l2_status | status) & 0xffff) | 0xffff0000);
	iommu_write(iommu, IOMMU_RESET_REG, IOMMU_RESET_RELEASE_ALL);
}

static irqreturn_t sun50i_iommu_irq(int irq, void *dev_id)
{
	u32 status, l1_status, l2_status, resets;
	phys_addr_t iova;
	struct sun50i_iommu *iommu = dev_id;

	spin_lock(&iommu->iommu_lock);

	status = iommu_read(iommu, IOMMU_INT_STA_REG);
	if (!(status & IOMMU_INT_MASK)) {
		spin_unlock(&iommu->iommu_lock);
		return IRQ_NONE;
	}

	l1_status = iommu_read(iommu, IOMMU_L1PG_INT_REG);
	l2_status = iommu_read(iommu, IOMMU_L2PG_INT_REG);

	if (status & IOMMU_INT_INVALID_L2PG)
		iova = sun50i_iommu_handle_pt_irq(iommu, "Invalid L2 page table",
						  IOMMU_INT_ERR_ADDR_L2_REG,
						  IOMMU_L2PG_INT_REG,
						  IOMMU_INT_ERR_DATA_L2_REG);
	else if (status & IOMMU_INT_INVALID_L1PG)
		iova = sun50i_iommu_handle_pt_irq(iommu, "Invalid L1 page table",
						  IOMMU_INT_ERR_ADDR_L1_REG,
						  IOMMU_L1PG_INT_REG,
						  IOMMU_INT_ERR_DATA_L1_REG);
	else
		iova = sun50i_iommu_handle_perm_irq(iommu);

	if (iommu->variant) {
		sun50i_iommu_v14_fault_cleanup(iommu, status, l1_status,
					       l2_status, iova);
	} else {
		iommu_write(iommu, IOMMU_INT_CLR_REG, status);

		resets = (status | l1_status | l2_status) &
			 IOMMU_INT_MASTER_MASK;
		iommu_write(iommu, IOMMU_RESET_REG, ~resets);
		iommu_write(iommu, IOMMU_RESET_REG, IOMMU_RESET_RELEASE_ALL);
	}

	spin_unlock(&iommu->iommu_lock);

	return IRQ_HANDLED;
}

static const char *sun50i_pte_prot(struct sun50i_iommu *iommu, u32 pte)
{
	if (iommu->variant && iommu->variant->direct_prot) {
		switch (FIELD_GET(GENMASK(3, 2), pte)) {
		case 1: return "rd";
		case 2: return "wr";
		case 3: return "rdwr";
		default: return "none";
		}
	}

	switch (sun50i_get_pte_aci(pte)) {
	case SUN50I_IOMMU_ACI_RD: return "rd";
	case SUN50I_IOMMU_ACI_WR: return "wr";
	case SUN50I_IOMMU_ACI_RD_WR: return "rdwr";
	default: return "none";
	}
}

static ssize_t sun50i_dump_run(char *buf, ssize_t n, dma_addr_t start,
			       unsigned int pages, u32 prot,
			       struct sun50i_iommu *iommu)
{
	return n + sysfs_emit_at(buf, n,
		"iova:0x%08llx-0x%08llx size:0x%zx %s\n",
		(unsigned long long)start,
		(unsigned long long)(start + ((dma_addr_t)pages << 12) - 1),
		(size_t)pages << 12, sun50i_pte_prot(iommu, prot));
}

/*
 * Mirrors the vendor's map attribute: a "reserved" list followed by the live
 * page table under "mapped", walked region by region.  The vendor's own
 * __print_region helper was not part of the decompiled set, so the mapped
 * line format here is readable rather than byte-identical.
 */
static ssize_t map_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct sun50i_iommu *iommu = dev_get_drvdata(dev);
	struct sun50i_iommu_domain *sun50i_domain;
	unsigned long flags;
	ssize_t n = 0;
	unsigned int i, d, p;

	n += sysfs_emit_at(buf, n, "reserved\n");
	for (i = 0; i < iommu->nr_resv; i++)
		n += sysfs_emit_at(buf, n,
			"iova:%pad                            size:0x%zx\n",
			&iommu->resv[i].start, iommu->resv[i].size);

	n += sysfs_emit_at(buf, n, "mapped\n");

	if (!iommu->domain)
		return n;

	sun50i_domain = to_sun50i_domain(iommu->domain);

	spin_lock_irqsave(&iommu->iommu_lock, flags);

	for (d = 0; d < NUM_DT_ENTRIES && n < PAGE_SIZE - 128; d++) {
		u32 dte = sun50i_domain->dt[d];
		u32 *pt = NULL;
		dma_addr_t run_start = 0;
		u32 run_prot = 0;
		unsigned int run_pages = 0;

		if (sun50i_dte_is_pt_valid(dte))
			pt = phys_to_virt(sun50i_dte_get_pt_address(dte));

		for (p = 0; p < NUM_PT_ENTRIES; p++) {
			u32 pte = pt ? pt[p] : 0;

			if (sun50i_pte_is_page_valid(pte)) {
				u32 prot = pte & (iommu->variant &&
						  iommu->variant->direct_prot ?
						  GENMASK(3, 2) :
						  SUN50I_PTE_ACI_MASK);

				if (run_pages && prot == run_prot) {
					run_pages++;
					continue;
				}
				if (run_pages)
					n = sun50i_dump_run(buf, n, run_start,
							    run_pages, run_prot,
							    iommu);
				run_start = ((dma_addr_t)d << 20) |
					    ((dma_addr_t)p << 12);
				run_prot = prot;
				run_pages = 1;
			} else if (run_pages) {
				n = sun50i_dump_run(buf, n, run_start, run_pages,
						    run_prot, iommu);
				run_pages = 0;
			}
		}

		if (run_pages)
			n = sun50i_dump_run(buf, n, run_start, run_pages,
					    run_prot, iommu);
	}

	spin_unlock_irqrestore(&iommu->iommu_lock, flags);

	return n;
}
static DEVICE_ATTR_RO(map);

/* 64-bit counter split as low word plus a masked high word */
static u64 sun50i_prof_counter(struct sun50i_iommu *iommu, unsigned int lo,
			       unsigned int hi, u32 hi_mask)
{
	u32 high = iommu_read(iommu, hi) & hi_mask;
	u32 low = iommu_read(iommu, lo);

	return ((u64)high << 32) | low;
}

/*
 * Vendor "profilling" attribute (their spelling): per-master access and hit
 * counts, the macro-TLB and PTW cache pairs, then total and max latency per
 * master.  Only v14 has these counters, so the file is only created when the
 * variant is present.
 */
static ssize_t profilling_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct sun50i_iommu *iommu = dev_get_drvdata(dev);
	const char *const *names;
	unsigned long flags;
	unsigned int m, nr;
	ssize_t n = 0;

	if (!iommu->variant || !iommu->variant->master_names)
		return sysfs_emit(buf, "no profiling counters\n");

	names = iommu->variant->master_names;
	nr = iommu->variant->nr_masters;

	spin_lock_irqsave(&iommu->iommu_lock, flags);

	for (m = 0; m < nr; m++)
		n += sysfs_emit_at(buf, n,
			"%s_access_count = 0x%llx\n%s_hit_count = 0x%llx\n",
			names[m],
			(unsigned long long)sun50i_prof_counter(iommu,
				IOMMU_V14_PROF_ACCESS(m),
				IOMMU_V14_PROF_ACCESS_HI(m), 0x7ff),
			names[m],
			(unsigned long long)sun50i_prof_counter(iommu,
				IOMMU_V14_PROF_HIT(m),
				IOMMU_V14_PROF_HIT_HI(m), 0x7ff));

	n += sysfs_emit_at(buf, n,
		"macrotlb_access_count = 0x%llx\nmacrotlb_hit_count = 0x%llx\n"
		"ptwcache_access_count = 0x%llx\nptwcache_hit_count = 0x%llx\n",
		(unsigned long long)sun50i_prof_counter(iommu,
			IOMMU_V14_PROF_MACRO_ACCESS,
			IOMMU_V14_PROF_MACRO_ACCESS_HI, 0x7ff),
		(unsigned long long)sun50i_prof_counter(iommu,
			IOMMU_V14_PROF_MACRO_HIT,
			IOMMU_V14_PROF_MACRO_HIT_HI, 0x7ff),
		(unsigned long long)sun50i_prof_counter(iommu,
			IOMMU_V14_PROF_PTW_ACCESS,
			IOMMU_V14_PROF_PTW_ACCESS_HI, 0x7ff),
		(unsigned long long)sun50i_prof_counter(iommu,
			IOMMU_V14_PROF_PTW_HIT,
			IOMMU_V14_PROF_PTW_HIT_HI, 0x7ff));

	for (m = 0; m < nr; m++)
		n += sysfs_emit_at(buf, n, "%s_total_latency = 0x%llx\n",
			names[m],
			(unsigned long long)sun50i_prof_counter(iommu,
				IOMMU_V14_PROF_LAT(m),
				IOMMU_V14_PROF_LAT_HI(m), 0x3ffff));

	for (m = 0; m < nr; m++)
		n += sysfs_emit_at(buf, n, "%s_max_latency = 0x%x\n",
			names[m], iommu_read(iommu, IOMMU_V14_PROF_MAX_LAT(m)));

	spin_unlock_irqrestore(&iommu->iommu_lock, flags);

	return n;
}
static DEVICE_ATTR_RO(profilling);

static int sun50i_iommu_probe(struct platform_device *pdev)
{
	struct sun50i_iommu *iommu;
	int ret, irq;

	iommu = devm_kzalloc(&pdev->dev, sizeof(*iommu), GFP_KERNEL);
	if (!iommu)
		return -ENOMEM;
	spin_lock_init(&iommu->iommu_lock);
	platform_set_drvdata(pdev, iommu);
	iommu->dev = &pdev->dev;

	/*
	 * The vendor reads a pair array of (iova, size) from its node and prints
	 * them under "reserved".  The stock iommu node has no such property, so
	 * this list is empty on this board; the property name is the one part of
	 * the vendor's parser the decompile lost.
	 */
	{
		u64 vals[8];
		int nr = of_property_read_variable_u64_array(pdev->dev.of_node,
							     "iova-reserve", vals, 0,
							     ARRAY_SIZE(vals));
		unsigned int i;

		if (nr > 0 && (nr & 1)) {
			dev_warn(&pdev->dev, "iova-reserve needs pairs of (iova, size)\n");
		} else if (nr > 0) {
			for (i = 0; i < nr / 2 && i < ARRAY_SIZE(iommu->resv); i++) {
				iommu->resv[i].start = vals[i * 2];
				iommu->resv[i].size = vals[i * 2 + 1];
			}
			iommu->nr_resv = i;
		}
	}

	iommu->pt_pool = kmem_cache_create(dev_name(&pdev->dev),
					   PT_SIZE, PT_SIZE,
					   SLAB_HWCACHE_ALIGN,
					   NULL);
	if (!iommu->pt_pool)
		return -ENOMEM;

	iommu->group = iommu_group_alloc();
	if (IS_ERR(iommu->group)) {
		ret = PTR_ERR(iommu->group);
		goto err_free_cache;
	}

	iommu->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(iommu->base)) {
		ret = PTR_ERR(iommu->base);
		goto err_free_group;
	}

	iommu->variant = device_get_match_data(&pdev->dev);
	if (iommu->variant) {
		u32 id = readl(iommu->base + IOMMU_VERSION_REG);

		if (id != iommu->variant->version) {
			dev_err(&pdev->dev,
				"IOMMU version mismatch: expected 0x%x, got 0x%x\n",
				iommu->variant->version, id);
			ret = -ENODEV;
			goto err_free_group;
		}
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		ret = irq;
		goto err_free_group;
	}

	iommu->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(iommu->clk)) {
		dev_err(&pdev->dev, "Couldn't get our clock.\n");
		ret = PTR_ERR(iommu->clk);
		goto err_free_group;
	}

	iommu->reset = devm_reset_control_get_optional(&pdev->dev, NULL);
	if (IS_ERR(iommu->reset)) {
		dev_err(&pdev->dev, "Couldn't get our reset line.\n");
		ret = PTR_ERR(iommu->reset);
		goto err_free_group;
	}

	ret = iommu_device_sysfs_add(&iommu->iommu, &pdev->dev,
				     NULL, dev_name(&pdev->dev));
	if (ret)
		goto err_free_group;

	ret = iommu_device_register(&iommu->iommu, &sun50i_iommu_ops, &pdev->dev);
	if (ret)
		goto err_remove_sysfs;

	ret = devm_request_irq(&pdev->dev, irq, sun50i_iommu_irq, 0,
			       dev_name(&pdev->dev), iommu);
	if (ret < 0)
		goto err_unregister;

	if (device_create_file(&pdev->dev, &dev_attr_map))
		dev_warn(&pdev->dev, "Failed to create the map attribute\n");

	if (iommu->variant &&
	    device_create_file(&pdev->dev, &dev_attr_profilling))
		dev_warn(&pdev->dev, "Failed to create the profilling attribute\n");

	bus_set_iommu(&platform_bus_type, &sun50i_iommu_ops);

	return 0;

err_unregister:
	iommu_device_unregister(&iommu->iommu);

err_remove_sysfs:
	iommu_device_sysfs_remove(&iommu->iommu);

err_free_group:
	iommu_group_put(iommu->group);

err_free_cache:
	kmem_cache_destroy(iommu->pt_pool);

	return ret;
}

/*
 * Taken verbatim from the vendor's of_device_id data for
 * "allwinner,iommu-v14-sun50iw15" in the stock kernel
 * (device_hy300ultra/prebuilts/kernel-vmlinux.elf):
 * { version, prefetch, ctrl_084, ctrl_09c }.  The neighbouring
 * sun50iw12/sun8iw20/sun8iw21 entries have their own structures in the same
 * table; only this SoC's were read.
 */
/*
 * Master names from the vendor's match data, which continues past the four
 * register words with one string pointer per master id (read out of
 * stock kernel-vmlinux.elf at 0xc17d2c78..0xc17d2c94).
 */
static const char *const sun50iw15_master_names[] = {
	"VE", "VE_R", "TVD_MBUS", "TVD_AXI", "TVCAP", "KSC", "TVFE",
};

static const struct sun50i_iommu_variant sun50iw15_iommu_v14 = {
	.version	= 0x14,
	.prefetch	= 0x0003007f,
	.ctrl_084	= 0x1,
	.ctrl_09c	= 0x1,
	.direct_prot	= true,
	.master_names	= sun50iw15_master_names,
	.nr_masters	= 7,
};

static const struct of_device_id sun50i_iommu_dt[] = {
	{ .compatible = "allwinner,sun50i-h6-iommu", },
	{ .compatible = "allwinner,iommu-v14-sun50iw15",
	  .data = &sun50iw15_iommu_v14 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sun50i_iommu_dt);

static struct platform_driver sun50i_iommu_driver = {
	.driver		= {
		.name			= "sun50i-iommu",
		.of_match_table 	= sun50i_iommu_dt,
		.suppress_bind_attrs	= true,
	}
};
builtin_platform_driver_probe(sun50i_iommu_driver, sun50i_iommu_probe);

MODULE_DESCRIPTION("Allwinner H6 IOMMU driver");
MODULE_AUTHOR("Maxime Ripard <maxime@cerno.tech>");
MODULE_AUTHOR("zhuxianbin <zhuxianbin@allwinnertech.com>");
MODULE_LICENSE("Dual BSD/GPL");
