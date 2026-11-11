// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allwinner H726 tvpanel - MIPI DSI controller and combo D-PHY register layer.
 *
 * Every offset below is a named field of the BTF-derived block layout in
 * tvpanel_regs.h, so the "*(_DWORD *)(base + 0xNN)" accesses in the vendor
 * binary are now typed.  The field each vendor offset corresponded to is
 * noted next to the code it came from.
 */
#include <linux/bitops.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>

#include "tvpanel.h"
#include "tvpanel_regs.h"
#include "dsi_seq.h"

/* dsi_set_reg_base() keeps one DSI controller and one D-PHY per id. */
static void __iomem *dsi_base[TVPANEL_MAX_DSI];
static void __iomem *dphy_base[TVPANEL_MAX_DSI];
/* The 4-byte "dsi_res" register (DT reg index 1) holding the LVDS/DSI
 * reset-select fields; stored for the LVDS/DSI mux code.
 */
static void __iomem *dsi_res_base;

#define DSI(id)		((struct __de_dsi_dev_t __iomem *)(dsi_base[(id)]))
#define DPHY(id)	((struct __de_dsi_dphy_dev_t __iomem *)(dphy_base[(id)]))

int tvpanel_dsi_set_reg_base(unsigned int id, void __iomem *base,
			     void __iomem *res_base)
{
	if (id >= TVPANEL_MAX_DSI || !base)
		return -EINVAL;

	dsi_base[id] = base;
	dphy_base[id] = base + TVPANEL_DPHY_OFFSET;
	dsi_res_base = res_base;
	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_set_reg_base);

void __iomem *tvpanel_dsi_get_reg_base(unsigned int id)
{
	return dsi_base[id];
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_get_reg_base);

void __iomem *tvpanel_dsi_get_dphy_base(unsigned int id)
{
	return dphy_base[id];
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_get_dphy_base);

/*
 * Program dsi_res from the "dsi_res" eFuse key, preserving its [15:8] field.
 * Vendor: value = key | (reg & 0xff00) | 0x19110000.
 *
 * sunxi_sid_sram_read32() comes from the SID driver on a real device; the
 * weak fallback keeps the module self-contained and makes a failed read
 * leave dsi_res untouched, which is what the vendor code does too.
 */
int sunxi_sid_sram_read32(const char *key, u32 *val);

int __weak sunxi_sid_sram_read32(const char *key, u32 *val)
{
	return -ENODEV;
}

void tvpanel_dsi_res_select(void)
{
	u32 sid = 0;

	if (!dsi_res_base)
		return;
	if (sunxi_sid_sram_read32("dsi_res", &sid))
		return;
	if (!sid)
		return;

	writel(sid | (readl(dsi_res_base) & 0xff00u) | 0x19110000u,
	       dsi_res_base);
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_res_select);

/*
 * ECC over the 24-bit DSI packet header.
 *
 * The vendor computes it with a long unrolled parity tree (dsi_ecc_pro());
 * these masks are the same parity sets, which are also the standard ones
 * from the MIPI DSI specification.  ECC[7:6] are zero for a 24-bit header,
 * which is why only six masks exist.
 */
u8 tvpanel_dsi_ecc(u32 header)
{
	static const u32 mask[6] = {
		0x00f12cb7, 0x00f2555b, 0x00749a6d,
		0x00b8e38e, 0x00df03f0, 0x00effc00,
	};
	u32 data = header & 0xffffffu;
	u8 ecc = 0;
	int i;

	for (i = 0; i < 6; i++)
		ecc |= (hweight32(data & mask[i]) & 1) << i;
	return ecc;
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_ecc);

/*
 * CRC-16 (poly 0x8408, init 0xffff) over nbytes zero bytes, which is what
 * the vendor computes for the block-packet payload fields.
 */
u16 tvpanel_dsi_crc(u32 nbytes, u16 init)
{
	u16 crc = init;
	u32 i, b;

	for (i = 0; i < nbytes; i++) {
		for (b = 0; b < 8; b++)
			crc = (crc & 1) ? ((crc >> 1) ^ 0x8408) : (crc >> 1);
	}
	return crc;
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_crc);

/*
 * dsi_gint0 packs the interrupt state in two halves: [15:0] enable,
 * [31:16] status, where the status bits are write-1-to-clear.  The vendor
 * code performs these writes as read-modify-write of the whole word, which
 * is what the helpers below reproduce.
 */
void tvpanel_dsi_irq_enable(unsigned int id, unsigned int irq)
{
	u32 v = readl(&DSI(id)->dsi_gint0);

	writel((v & 0xffff0000u) | ((v | BIT(irq)) & 0xffffu),
	       &DSI(id)->dsi_gint0);
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_irq_enable);

void tvpanel_dsi_irq_disable(unsigned int id, unsigned int irq)
{
	u32 v = readl(&DSI(id)->dsi_gint0);

	writel((v & 0xffff0000u) | ((v & ~BIT(irq)) & 0xffffu),
	       &DSI(id)->dsi_gint0);
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_irq_disable);

int tvpanel_dsi_irq_query(unsigned int id, unsigned int irq)
{
	u32 v = readl(&DSI(id)->dsi_gint0);

	/* pending = enable bit AND status bit */
	if (!(v & BIT(irq) & (v >> 16)))
		return 0;

	/* acknowledge by writing the status half */
	writel((v & 0xffff0000u) | (0x10000u << irq) | (v & 0xffffu),
	       &DSI(id)->dsi_gint0);
	return 1;
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_irq_query);

/* dsi_basic_ctl0 bit 0: instruction engine busy */
int tvpanel_dsi_inst_busy(unsigned int id)
{
	return readl(&DSI(id)->dsi_basic_ctl0) & 1;
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_inst_busy);

/*
 * Load the precompiled instruction sequence and kick the sequencer.
 * Vendor dsi_open() writes:
 *   gint0      |= BIT(2)
 *   jump_sel    = 0x63f07006
 *   basic_ctl0  bit0 cleared then set (restart)
 * and for non-continuous clock mode additionally
 *   jump_sel    = 0x52e07306
 *   jump_sel1   = 0x0d68000f
 *   basic_ctl0  bit0 cleared then set
 */
int tvpanel_dsi_open(unsigned int id, const struct disp_panel_para *para)
{
	struct __de_dsi_dev_t __iomem *dsi = DSI(id);
	u32 v;

	if (para->lcd_dsi_if == TVPANEL_DSI_IF_VIDEO)
		return 0;

	v = readl(&dsi->dsi_gint0);
	writel((v & 0xffff0000u) | ((v | BIT(2)) & 0xffffu), &dsi->dsi_gint0);

	writel(DSI_INST_JUMP_SEL_DEFAULT, &dsi->dsi_inst_jump_sel);

	v = readl(&dsi->dsi_basic_ctl0);
	writel(v & ~1u, &dsi->dsi_basic_ctl0);
	writel(v | 1u, &dsi->dsi_basic_ctl0);

	if (para->lcd_dsi_clk_mode == TVPANEL_DSI_CLK_NON_CONTINUOUS) {
		writel(DSI_INST_JUMP_SEL_NONCONT, &dsi->dsi_inst_jump_sel);
		writel(DSI_INST_JUMP_SEL1_NONCONT, &dsi->dsi_inst_jump_sel1);

		v = readl(&dsi->dsi_basic_ctl0);
		writel(v & ~1u, &dsi->dsi_basic_ctl0);
		writel(v | 1u, &dsi->dsi_basic_ctl0);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_open);

/* Vendor dsi_close(): drop the enable bit, self-clear the sequencer, settle. */
int tvpanel_dsi_close(unsigned int id)
{
	struct __de_dsi_dev_t __iomem *dsi = DSI(id);
	u32 v = readl(&dsi->dsi_gint0);

	writel((v & 0xffff0000u) | ((v & ~BIT(2)) & 0xffffu), &dsi->dsi_gint0);
	writel(readl(&dsi->dsi_inst_jump_cfg[0]) | 0x10000000u,
	       &dsi->dsi_inst_jump_cfg[0]);
	msleep(30);
	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_close);

/*
 * Vendor dsi_clk_enable().
 *
 * Non-continuous clock: load the non-continuous jump words and restart.
 *
 * Continuous clock: jump_sel = 0x00000f02, restart, and the clock-lane
 * enable of the first instruction entry is tied to the pixel stream, i.e.
 *   inst_func[0].lane_cen = !dsi_pixel_ctl0.pd_plug_dis
 * The vendor's v2[8] is byte offset 32 - dsi_inst_func[0], whose bit 4 is
 * lane_cen - not dsi_gint1, which sits at offset 8.  It then waits
 * const_udelay(21474800), i.e. 5ms.
 */
int tvpanel_dsi_clk_enable(unsigned int id, const struct disp_panel_para *para)
{
	struct __de_dsi_dev_t __iomem *dsi = DSI(id);
	u32 v;

	if (para->lcd_dsi_clk_mode) {
		writel(DSI_INST_JUMP_SEL_NONCONT, &dsi->dsi_inst_jump_sel);
		writel(DSI_INST_JUMP_SEL1_NONCONT, &dsi->dsi_inst_jump_sel1);
	} else {
		writel(0x00000f02u, &dsi->dsi_inst_jump_sel);

		v = (readl(&dsi->dsi_inst_func[0]) & 0xffffffefu) |
		    (0x10u * ((readl(&dsi->dsi_pixel_ctl0) >> 16) & 1u));
		writel(v ^ 0x10u, &dsi->dsi_inst_func[0]);
	}

	v = readl(&dsi->dsi_basic_ctl0);
	writel(v & ~1u, &dsi->dsi_basic_ctl0);
	writel(v | 1u, &dsi->dsi_basic_ctl0);

	usleep_range(5000, 6000);
	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_clk_enable);

/*
 * dphy_ana4 encoding, extracted from the mask sequence the vendor applies to
 * dphy_ana4 (one read-modify-write per field, all on the same register, so
 * they compose into a single value):
 *
 *   [4:0]   <- cfg[4:0]
 *   [5]     <- cfg[5]
 *   [6]     <- cfg[6]
 *   [10:8]  <- cfg[10:8]
 *   [14:12] <- cfg[14:12]
 *   [18:16] <- cfg[18:16]
 *   [22:20] <- cfg[22:20]
 *   [23]    <- cfg[23]
 *   [26:24] <- cfg[26:24]
 *   [29:28] <- cfg[29:28]
 *   [30]    <- cfg[30]
 *   [7]     <- cfg[7]
 *
 * Bits 27 and 31 are never written by the vendor sequence.
 */
static u32 dphy_ana4_encode(u32 cfg)
{
	u32 v;

	v = (cfg & 0x1fu);
	v |= (cfg & 0x20u);
	v |= (cfg & 0x40u);
	v |= (((cfg >> 8) & 7u) << 8);
	v |= (((cfg >> 12) & 7u) << 12);
	v |= (((cfg >> 16) & 7u) << 16);
	v |= (((cfg >> 20) & 7u) << 20);
	v |= (cfg & BIT(23));
	v |= (((cfg >> 24) & 7u) << 24);
	v |= (((cfg >> 28) & 3u) << 28);
	v |= (cfg & BIT(30));
	v |= (cfg & BIT(7));
	return v;
}

/*
 * Vendor dsi_dphy_open(): program the PHY from the connector's
 * combodphy_config, power the analog block up and enable the active lanes.
 *
 * Delays are the vendor's const_udelay arguments converted to microseconds
 * (they divide by 0x10c7): 4294960 = 1ms, 214748 = 50us.
 */
int tvpanel_dsi_dphy_open(unsigned int id, const struct disp_panel_para *para,
			  const struct combodphy_config *cfg)
{
	struct __de_dsi_dphy_dev_t __iomem *dphy = DPHY(id);
	u32 lane_mask = 0;
	u32 v, i;

	if (!dphy || !para || !cfg)
		return -EINVAL;

	for (i = 0; i < para->lcd_dsi_lane && i < 4; i++)
		lane_mask |= BIT(i);

	writel(dphy_ana4_encode(cfg->dphy_ana4), &dphy->dphy_ana4);

	writel(readl(&dphy->dphy_ana2) | 0x10u, &dphy->dphy_ana2);
	writel(readl(&dphy->dphy_ana2) | 0x02u, &dphy->dphy_ana2);

	writel(readl(&dphy->dphy_ana3) | 0x40000u, &dphy->dphy_ana3);
	writel(readl(&dphy->dphy_ana3) | 0x2000000u, &dphy->dphy_ana3);
	writel(readl(&dphy->dphy_ana3) | 0x1000000u, &dphy->dphy_ana3);

	/*
	 * ana0 takes the lane timing fields from combodphy_config.  Bits 7 is
	 * never written; [15:12] and [11:8] are plain copies of the same bits
	 * of the source value.
	 */
	v = readl(&dphy->dphy_ana0);
	v = (v & 0xffffff8fu) | (((cfg->dphy_ana0 >> 4) & 7u) << 4);
	v = (v & 0xfffffff8u) | (cfg->dphy_ana0 & 7u);
	v = (v & 0xff0fffffu) | (((cfg->dphy_ana0 >> 20) & 0xfu) << 20);
	v = (v & 0xfff0ffffu) | (((cfg->dphy_ana0 >> 16) & 0xfu) << 16);
	v = (v & 0xffff0fffu) | (cfg->dphy_ana0 & 0xf000u);
	v = (v & 0xfffff0ffu) | (cfg->dphy_ana0 & 0xf00u);
	writel(v, &dphy->dphy_ana0);

	/* combo PHY on, then let the analog block settle */
	writel(readl(&dphy->combo_phy_reg0) | 0x1u, &dphy->combo_phy_reg0);
	usleep_range(1000, 1100);

	writel(readl(&dphy->dphy_ana4) | BIT(31), &dphy->dphy_ana4);
	writel(readl(&dphy->combo_phy_reg0) | 0x8u, &dphy->combo_phy_reg0);
	writel(readl(&dphy->combo_phy_reg0) | 0x2u, &dphy->combo_phy_reg0);
	writel((readl(&dphy->combo_phy_reg2) & 0xffffff00u) | 0x14u,
	       &dphy->combo_phy_reg2);
	udelay(50);

	/* enable the active data lanes in ana3[31:28] and ana2[27:24] */
	writel(readl(&dphy->dphy_ana3) | 0x8000000u, &dphy->dphy_ana3);
	v = readl(&dphy->dphy_ana3);
	writel((v & 0x0fffffffu) | ((lane_mask & 0xfu) << 28),
	       &dphy->dphy_ana3);
	writel(readl(&dphy->dphy_ana3) | 0x4000000u, &dphy->dphy_ana3);

	writel(readl(&dphy->dphy_ana2) | 0x10u, &dphy->dphy_ana2);
	writel(readl(&dphy->dphy_ana1) | BIT(31), &dphy->dphy_ana1);
	v = readl(&dphy->dphy_ana2);
	writel((v & 0xf0ffffffu) | ((lane_mask & 0xfu) << 24),
	       &dphy->dphy_ana2);

	writel(readl(&dphy->dphy_gctl) | 0x1u, &dphy->dphy_gctl);
	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_dphy_open);

/*
 * Vendor dsi_io_close(): power the PHY back down, ana1 bit31, ana2 bits
 * [27:24] and [4], ana3 bit26 and bits [27:24], with 214748-loop delays
 * between steps.
 */
int tvpanel_dsi_dphy_close(unsigned int id)
{
	struct __de_dsi_dphy_dev_t __iomem *dphy = DPHY(id);
	u32 v;

	v = readl(&dphy->dphy_ana2);
	writel(v & 0xf0ffffffu, &dphy->dphy_ana2);
	writel(readl(&dphy->dphy_ana1) & ~BIT(31), &dphy->dphy_ana1);
	udelay(50);

	writel(readl(&dphy->dphy_ana2) & ~BIT(4), &dphy->dphy_ana2);
	udelay(50);

	writel(readl(&dphy->dphy_ana3) & ~BIT(26), &dphy->dphy_ana3);
	udelay(50);

	v = readl(&dphy->dphy_ana3);
	writel(v & 0x0fffffffu, &dphy->dphy_ana3);
	writel(readl(&dphy->dphy_ana3) & ~BIT(27), &dphy->dphy_ana3);
	udelay(50);
	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_dphy_close);

int tvpanel_dsi_io_open(unsigned int id, const struct disp_panel_para *para,
			const struct combodphy_config *cfg)
{
	return tvpanel_dsi_dphy_open(id, para, cfg);
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_io_open);

int tvpanel_dsi_io_close(unsigned int id)
{
	return tvpanel_dsi_dphy_close(id);
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_io_close);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Allwinner H726 tvpanel register layer");
