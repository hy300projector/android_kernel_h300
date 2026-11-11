// SPDX-License-Identifier: GPL-2.0-only
/*
 * Maxio MAE0621A / MAE0621A-Q3CI 10/100 Ethernet PHY.
 *
 * Rewritten from the vendor maxio.ko (MAXIO_PHY_VER v1.7.4.2).  The vendor
 * driver carries two phy_driver entries with separate init sequences: the
 * MAE0621A walks a set of vendor paged writes and then polls for range-ready,
 * the MAE0621A-Q3CI uses a different page set and no poll.  Both are
 * reproduced here.
 *
 * Every vendor write is save-page/write-page/write-reg/restore-page through
 * MDIO register 31; maxio_paged_write() reproduces that.
 *
 * PHY IDs from the module's own mdio_device_id table and phy_driver entries:
 *   MAE0621A        0x00004411 / 0x0000ffff
 *   MAE0621A-Q3CI   0x7b744412 / 0x7fffffff
 * The board DT pins "ethernet-phy-idc000.0000", and phylib prefers a
 * phy-id from the DT compatible over the register value, so a third entry
 * keeps that ID bound to the MAE0621A sequence.
 */
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy.h>

#define MAXIO_PAGE_REG		31

#define MAXIO_PAGE_BASE		0x0d8f	/* regs 0, 8, 11, 16, 22 */
#define MAXIO_PAGE_CLK_STRAP	0x0d92	/* reg 2 = clock strap, reg 11 = ready */
#define MAXIO_PAGE_RANGE	0x0d96	/* reg 2, reg 19, reg 21, reg 22 */
#define MAXIO_PAGE_PATCH	0x0d90	/* reg 2 and reg 5, reg 22 */
#define MAXIO_PAGE_TX		0x0d91	/* reg 6 */
#define MAXIO_PAGE_AUX		0x0da0	/* reg 16 */

#define MAXIO_PAGE_ID_REG	2
#define MAXIO_ID_CRYSTAL	0x200a
#define MAXIO_ID_OSC		0x210a

/* vendor ready poll: 50 outer passes, 4 register reads each */
#define MAXIO_POLL_PASSES	50
#define MAXIO_POLL_READS	4
#define MAXIO_POLL_ADDR_BASE	0xf908	/* 0xfffff908, truncated to u16 */
#define MAXIO_POLL_MASK		0x01ff

static int maxio_paged_write(struct phy_device *phydev, u16 page, u16 reg,
			     u16 val)
{
	int saved, ret, ret2;

	saved = phy_read(phydev, MAXIO_PAGE_REG);
	if (saved < 0)
		return saved;

	ret = phy_write(phydev, MAXIO_PAGE_REG, page);
	if (ret)
		return ret;

	ret = phy_write(phydev, reg, val);

	/* always put the previous page back, even if the write failed */
	ret2 = phy_write(phydev, MAXIO_PAGE_REG, saved);

	return ret ? ret : ret2;
}

static int maxio_paged_read(struct phy_device *phydev, u16 page, u16 reg)
{
	int saved, val, ret;

	saved = phy_read(phydev, MAXIO_PAGE_REG);
	if (saved < 0)
		return saved;

	ret = phy_write(phydev, MAXIO_PAGE_REG, page);
	if (ret)
		return ret;

	val = phy_read(phydev, reg);

	ret = phy_write(phydev, MAXIO_PAGE_REG, saved);

	return ret ? ret : val;
}

/*
 * Vendor range-ready poll.  Each pass re-arms the range block, then reads four
 * status words; a zero status ends the read loop and the pass writes the
 * 0x1940/0x1140/0x9140 sequence before the next pass.  Four non-zero reads
 * print "MAE0621A READY" and jump straight to the finishing writes, and so
 * does exhausting all 50 passes.
 */
static int maxio_wait_range_ready(struct phy_device *phydev)
{
	int pass, attempt, val, ret;

	for (pass = 0; pass < MAXIO_POLL_PASSES; pass++) {
		ret = maxio_paged_write(phydev, MAXIO_PAGE_RANGE,
					MAXIO_PAGE_ID_REG, 0x1fff);
		if (ret)
			return ret;
		ret = maxio_paged_write(phydev, MAXIO_PAGE_RANGE,
					MAXIO_PAGE_ID_REG, 0x1000);
		if (ret)
			return ret;

		for (attempt = 0; attempt < MAXIO_POLL_READS; attempt++) {
			ret = maxio_paged_write(phydev, MAXIO_PAGE_BASE, 11,
						MAXIO_POLL_ADDR_BASE +
						(attempt << 8));
			if (ret)
				return ret;

			val = maxio_paged_read(phydev,
					       MAXIO_PAGE_CLK_STRAP, 11);
			if (val < 0)
				val = 0;

			if (!(val & MAXIO_POLL_MASK))
				break;
		}

		if (attempt == MAXIO_POLL_READS) {
			pr_info("MAE0621A READY\n");
			goto finish;
		}

		ret = phy_write(phydev, 0, 0x1940);
		if (ret)
			return ret;
		msleep(10);
		ret = phy_write(phydev, 0, 0x1140);
		if (ret)
			return ret;
		ret = phy_write(phydev, 0, 0x9140);
		if (ret)
			return ret;
	}

finish:
	ret = maxio_paged_write(phydev, MAXIO_PAGE_RANGE, MAXIO_PAGE_ID_REG,
				0x0fff);
	if (ret)
		return ret;
	ret = phy_write(phydev, 0, 0x9140);
	if (ret)
		return ret;
	msleep(100);

	return 0;
}

static int maxio_config_init(struct phy_device *phydev)
{
	struct device_node *np = phydev->mdio.dev.of_node;
	u32 clk_mode = 0;
	int ret;

	/* The vendor defaults a missing property to 0, i.e. oscillator. */
	if (np)
		of_property_read_u32(np, "maxio,phy-clk-mode", &clk_mode);

	/* aux page: register 16 */
	ret = maxio_paged_write(phydev, MAXIO_PAGE_AUX, 16, 0x0f13);
	if (ret)
		return ret;

	/* patch page: register 2 then register 5 */
	ret = maxio_paged_write(phydev, MAXIO_PAGE_PATCH, MAXIO_PAGE_ID_REG,
				0x1555);
	if (ret)
		return ret;
	ret = maxio_paged_write(phydev, MAXIO_PAGE_PATCH, 5, 0x2b15);
	if (ret)
		return ret;

	/* clock strap: 0x210a oscillator (default), 0x200a crystal */
	ret = maxio_paged_write(phydev, MAXIO_PAGE_CLK_STRAP,
				MAXIO_PAGE_ID_REG,
				clk_mode ? MAXIO_ID_CRYSTAL : MAXIO_ID_OSC);
	if (ret)
		return ret;

	phydev_info(phydev, "%s clkmode(%s) PHY_ID: %#x\n",
		    __func__, clk_mode ? "crystal" : "oscillator",
		    phydev->phy_id);

	/* base page: register 0, register 8 */
	ret = maxio_paged_write(phydev, MAXIO_PAGE_BASE, 0, 0x0300);
	if (ret)
		return ret;
	ret = maxio_paged_write(phydev, MAXIO_PAGE_BASE, 8, 0x2500);
	if (ret)
		return ret;

	/* range page: register 19 */
	ret = maxio_paged_write(phydev, MAXIO_PAGE_RANGE, 19, 0x07bc);
	if (ret)
		return ret;

	/* tx page: register 6 */
	ret = maxio_paged_write(phydev, MAXIO_PAGE_TX, 6, 0x6880);
	if (ret)
		return ret;

	ret = phy_write(phydev, MAXIO_PAGE_REG, 0);
	if (ret)
		return ret;

	return maxio_wait_range_ready(phydev);
}

/*
 * MAE0621A-Q3CI: same paged-write shape, different pages/registers and no
 * ready poll.  Values are the immediates from
 * maxio_mae0621aq3ci_config_init in the stock module.
 */
static int maxio_q3ci_config_init(struct phy_device *phydev)
{
	static const struct {
		u16 page;
		u16 reg;
		u16 val;
	} seq[] = {
		{ 0x0dab, 23, 0x0f13 },
		{ 0x0d8f, 16, 0x0300 },
		{ 0x0d96, 21, 0xc08a },
		{ 0x0da4, 18, 0x07bc },
		{ 0x0d8f, 22, 0x2500 },
		{ 0x0d90, 22, 0x1555 },
		{ 0x0d92, 17, 0x2b15 },
		{ 0x0d96, 22, 0x4010 },
		{ 0x0da5, 16, 0x4a12 },
		{ 0x0da5, 17, 0x4a12 },
		{ 0x0da5, 18, 0x4a12 },
		{ 0x0da8, 17, 0x0175 },
		{ 0x0a43, 25, 0x0823 },
		{ 0x0000, 0, 0x9140 },
	};
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(seq); i++) {
		ret = maxio_paged_write(phydev, seq[i].page, seq[i].reg,
					seq[i].val);
		if (ret)
			return ret;
	}

	/* the vendor leaves the PHY on page 0 */
	return phy_write(phydev, MAXIO_PAGE_REG, 0);
}

static int maxio_suspend(struct phy_device *phydev)
{
	int ret = genphy_suspend(phydev);

	return phy_write(phydev, MAXIO_PAGE_REG, 0) | ret;
}

static int maxio_resume(struct phy_device *phydev)
{
	int ret = genphy_resume(phydev);
	int bmcr;

	bmcr = phy_read(phydev, MII_BMCR);
	if (bmcr >= 0)
		phy_write(phydev, MII_BMCR, bmcr | BMCR_RESET);
	msleep(20);

	return ret;
}

static struct phy_driver maxio_phy_drivers[] = {
	{
		/* what the board DT pins via ethernet-phy-idc000.0000 */
		.phy_id		= 0xc0000000,
		.phy_id_mask	= 0xfffffff0,
		.name		= "Maxio MAE0621A",
		.config_init	= maxio_config_init,
		.config_aneg	= genphy_config_aneg,
		.read_status	= genphy_read_status,
		.suspend	= maxio_suspend,
		.resume		= maxio_resume,
	},
	{
		/* vendor ID for the MAE0621A */
		.phy_id		= 0x00004411,
		.phy_id_mask	= 0x0000ffff,
		.name		= "Maxio MAE0621A Gigabit Ethernet",
		.config_init	= maxio_config_init,
		.config_aneg	= genphy_config_aneg,
		.read_status	= genphy_read_status,
		.suspend	= maxio_suspend,
		.resume		= maxio_resume,
	},
	{
		/* vendor ID for the MAE0621A-Q3CI */
		.phy_id		= 0x7b744412,
		.phy_id_mask	= 0x7fffffff,
		.name		= "Maxio MAE0621A-Q3CI",
		.config_init	= maxio_q3ci_config_init,
		.config_aneg	= genphy_config_aneg,
		.read_status	= genphy_read_status,
		.suspend	= maxio_suspend,
		.resume		= maxio_resume,
	},
};

module_phy_driver(maxio_phy_drivers);

static const struct mdio_device_id __maybe_unused maxio_phy_tbl[] = {
	{ 0xc0000000, 0xfffffff0 },
	{ 0x00004411, 0x0000ffff },
	{ 0x7b744412, 0x7fffffff },
	{ }
};
MODULE_DEVICE_TABLE(mdio, maxio_phy_tbl);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Maxio PHY driver");
MODULE_AUTHOR("Allwinner Technology");
