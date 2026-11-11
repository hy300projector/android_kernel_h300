// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allwinner H726 tvpanel - DSI command channel.
 *
 * dsi_start() picks one of the vendor's eleven precompiled sequencer
 * programs by writing dsi_inst_jump_sel/jump_sel1 and pulsing the sequencer
 * start bit.  The DCS helpers push a packet header/payload into dsi_cmd_tx
 * and let that sequencer program send it, or put the controller into read
 * mode and pull the reply out of dsi_cmd_rx.
 *
 * The instruction words themselves are not documented anywhere (the DSI
 * instruction set is absent from every public Allwinner manual), so the jump
 * tables below are kept bit-exact against the vendor module's dsi_start().
 */
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/printk.h>

#include "tvpanel.h"
#include "tvpanel_regs.h"
#include "dsi_seq.h"

#define DSI(id)	((struct __de_dsi_dev_t __iomem *)(tvpanel_dsi_get_reg_base(id)))

/* dsi_cmd_ctl layout, from the vendor's packet builders */
#define DSI_CMD_CTL_LEN(len)		((len) & 0xffu)	/* [7:0]  payload len - 1 */
#define DSI_CMD_CTL_RX_COUNT(ctl)	(((ctl) >> 16) & 0x1fu)
#define DSI_CMD_CTL_RX_FLAG		BIT(25)		/* reply received */
#define DSI_CMD_CTL_RX_OVERFLOW		BIT(26)

/* First byte of a DCS reply the vendor accepts. */
#define DSI_DCS_ACK			0x02u

/*
 * The vendor polls the sequencer busy bit (basic_ctl0 bit 0) up to 0x1f3
 * times, sleeping const_udelay(2147480) - exactly 500us - between polls, and
 * clears the start bit to force it idle once it has polled 0x31 times.
 */
#define DSI_BUSY_POLL_MAX	499
/* The vendor's counter starts at -1 and is pre-incremented, so it is the
 * 0-based poll index: abort once that reaches 0x31, i.e. on the 50th poll.
 */
#define DSI_BUSY_POLL_ABORT	49

/*
 * Sequencer programs.  The index is the vendor's dsi_start() argument; mode 0
 * (and anything past 10) is the vendor switch default, 0x0000000f.  Only
 * modes 7-10 touch dsi_inst_jump_sel1, so a zero sel1 means "leave it alone".
 */
static const struct dsi_seq_mode {
	u32 jump_sel;
	u32 jump_sel1;
} dsi_seq_modes[] = {
	[0]  = { 0x0000000fu, 0 },
	[1]  = { 0x000000f1u, 0 },
	[2]  = { 0x63f07602u, 0 },
	[3]  = { 0x000f0004u, 0 },
	[4]  = { 0x100700f4u, 0 },
	[5]  = { 0x00000f02u, 0 },
	[6]  = { 0x63f07006u, 0 },
	[7]  = { 0x6af07006u, 0x00000300u },
	[8]  = { 0x52e07306u, 0x0d68000fu },
	[9]  = { 0x52e07306u, 0x0d6810fbu },
	[10] = { 0x52b37461u, 0x0c6d8ea9u },
};

/* Pulse the sequencer start bit: clear it, then set it again. */
static void dsi_inst_restart(struct __de_dsi_dev_t __iomem *dsi)
{
	writel(readl(&dsi->dsi_basic_ctl0) & ~1u, &dsi->dsi_basic_ctl0);
	writel(readl(&dsi->dsi_basic_ctl0) | 1u, &dsi->dsi_basic_ctl0);
}

/*
 * Wait for the sequencer to go idle, and force it out if it stays busy -
 * without that abort a wedged sequencer would hang every later packet.
 */
static void dsi_wait_idle(struct __de_dsi_dev_t __iomem *dsi)
{
	unsigned int i;

	if (!(readl(&dsi->dsi_basic_ctl0) & 1u))
		return;

	for (i = 0; i < DSI_BUSY_POLL_MAX; i++) {
		usleep_range(500, 600);
		if (!(readl(&dsi->dsi_basic_ctl0) & 1u))
			break;
	}

	if (i >= DSI_BUSY_POLL_ABORT)
		writel(readl(&dsi->dsi_basic_ctl0) & ~1u, &dsi->dsi_basic_ctl0);
}

int tvpanel_dsi_start(unsigned int id, unsigned int mode)
{
	struct __de_dsi_dev_t __iomem *dsi = DSI(id);

	if (!dsi)
		return -EINVAL;

	if (mode >= ARRAY_SIZE(dsi_seq_modes))
		mode = 0;

	writel(dsi_seq_modes[mode].jump_sel, &dsi->dsi_inst_jump_sel);
	if (dsi_seq_modes[mode].jump_sel1)
		writel(dsi_seq_modes[mode].jump_sel1, &dsi->dsi_inst_jump_sel1);

	dsi_inst_restart(dsi);

	/*
	 * Mode 5 also ties the clock-lane enable (inst_func[0].lane_cen) to
	 * the pixel stream: lane_cen = !pd_plug_dis, i.e. the vendor computes
	 *   v = (v & ~BIT(4) | BIT(4) * pd_plug_dis) ^ BIT(4)
	 */
	if (mode == 5) {
		u32 v = readl(&dsi->dsi_inst_func[0]);
		u32 pd = (readl(&dsi->dsi_pixel_ctl0) >> 16) & 1u;

		writel(((v & ~BIT(4)) | (BIT(4) * pd)) ^ BIT(4),
		       &dsi->dsi_inst_func[0]);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_start);

/*
 * Send a short or long DCS packet.  buf is the whole packet body (command
 * byte first) and len its length; the controller prepends the DSI header and
 * ECC itself once the sequencer runs the write program.
 */
int tvpanel_dsi_dcs_wr(unsigned int id, const u8 *buf, unsigned int len)
{
	struct __de_dsi_dev_t __iomem *dsi = DSI(id);
	unsigned int i;

	if (!dsi || !buf || !len || len > ARRAY_SIZE(dsi->dsi_cmd_tx))
		return -EINVAL;

	dsi_wait_idle(dsi);

	for (i = 0; i < len; i++)
		writeb(buf[i], &dsi->dsi_cmd_tx[i]);

	writel((readl(&dsi->dsi_cmd_ctl) & 0xffffff00u) | DSI_CMD_CTL_LEN(len - 1),
	       &dsi->dsi_cmd_ctl);
	writel(DSI_INST_JUMP_SEL_DCS_WR, &dsi->dsi_inst_jump_sel);
	dsi_inst_restart(dsi);

	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_dcs_wr);

/*
 * Read a DCS reply into buf.
 *
 * The vendor's own error path returns without leaving read mode, which would
 * wedge every later write, so all exits here clear jump_cfg[0] bit 28 and
 * reload the panel program first.
 */
int tvpanel_dsi_dcs_rd(unsigned int id, u8 *buf, unsigned int len)
{
	struct __de_dsi_dev_t __iomem *dsi = DSI(id);
	unsigned int i, count;
	u32 ctl;
	int ret = 0;

	if (!dsi || !buf)
		return -EINVAL;

	writel(readl(&dsi->dsi_inst_jump_cfg[0]) | BIT(28),
	       &dsi->dsi_inst_jump_cfg[0]);
	writel(DSI_INST_JUMP_SEL_DCS_RD, &dsi->dsi_inst_jump_sel);
	dsi_inst_restart(dsi);
	dsi_wait_idle(dsi);

	ctl = readl(&dsi->dsi_cmd_ctl);
	if (!(ctl & DSI_CMD_CTL_RX_FLAG)) {
		/* no reply queued: the vendor reports this as success */
		goto out;
	}

	if ((ctl & DSI_CMD_CTL_RX_OVERFLOW) ||
	    (readl(&dsi->dsi_cmd_rx[0]) & 0xffu) != DSI_DCS_ACK) {
		ret = -EIO;
		goto out;
	}

	count = DSI_CMD_CTL_RX_COUNT(ctl) + 1;
	if (count < len) {
		pr_warn("tvpanel: DSI read wanted %u bytes, only %u available\n",
			len, count);
		len = count;
	}

	for (i = 0; i < len; i++)
		buf[i] = readb(&dsi->dsi_cmd_rx[i]);

out:
	writel(readl(&dsi->dsi_inst_jump_cfg[0]) & ~BIT(28),
	       &dsi->dsi_inst_jump_cfg[0]);
	writel(DSI_INST_JUMP_SEL_PANEL_ON, &dsi->dsi_inst_jump_sel);
	dsi_inst_restart(dsi);
	return ret;
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_dcs_rd);

/*
 * Tell the panel the largest reply we can accept (DCS "set maximum return
 * packet size").  The 4-byte packet header is built in software: dt 0x37,
 * the 16-bit size, then ECC.
 */
int tvpanel_dsi_set_max_ret_size(unsigned int id, u16 size)
{
	struct __de_dsi_dev_t __iomem *dsi = DSI(id);
	u32 hdr;

	if (!dsi)
		return -EINVAL;

	hdr = readl(&dsi->dsi_cmd_tx[0]);
	hdr = (hdr & 0xffffff00u) | 0x37u;
	hdr = (hdr & 0xffff00ffu) | ((u32)(size & 0xffu) << 8);
	hdr = (hdr & 0xff00ffffu) | ((u32)(size >> 8) << 16);
	hdr = (hdr & 0xffffffu) | ((u32)tvpanel_dsi_ecc(hdr) << 24);
	writel(hdr, &dsi->dsi_cmd_tx[0]);

	writel((readl(&dsi->dsi_cmd_ctl) & 0xffffff00u) | DSI_CMD_CTL_LEN(3),
	       &dsi->dsi_cmd_ctl);
	writel(DSI_INST_JUMP_SEL_DCS_WR, &dsi->dsi_inst_jump_sel);
	dsi_inst_restart(dsi);

	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_set_max_ret_size);

/* Toggle read mode (jump_cfg[0] bit 28); leaving it restores the panel program. */
int tvpanel_dsi_read_mode_en(unsigned int id, bool enable)
{
	struct __de_dsi_dev_t __iomem *dsi = DSI(id);
	u32 v;

	if (!dsi)
		return -EINVAL;

	v = readl(&dsi->dsi_inst_jump_cfg[0]);
	writel((v & ~BIT(28)) | ((enable ? 1u : 0u) << 28),
	       &dsi->dsi_inst_jump_cfg[0]);

	if (!enable) {
		writel(DSI_INST_JUMP_SEL_PANEL_ON, &dsi->dsi_inst_jump_sel);
		dsi_inst_restart(dsi);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_read_mode_en);

/* Start the panel-on program: read mode off, jump into the normal sequence. */
int tvpanel_dsi_open_panel(unsigned int id)
{
	struct __de_dsi_dev_t __iomem *dsi = DSI(id);

	if (!dsi)
		return -EINVAL;

	writel(readl(&dsi->dsi_inst_jump_cfg[0]) & ~BIT(28),
	       &dsi->dsi_inst_jump_cfg[0]);
	writel(DSI_INST_JUMP_SEL_PANEL_ON, &dsi->dsi_inst_jump_sel);
	dsi_inst_restart(dsi);
	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_open_panel);

/* Park the sequencer: read mode on, which stops it fetching packets. */
int tvpanel_dsi_close_panel(unsigned int id)
{
	struct __de_dsi_dev_t __iomem *dsi = DSI(id);

	if (!dsi)
		return -EINVAL;

	writel(readl(&dsi->dsi_inst_jump_cfg[0]) | BIT(28),
	       &dsi->dsi_inst_jump_cfg[0]);
	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_close_panel);

/* Restart the panel program without touching read mode. */
int tvpanel_dsi_tri_start(unsigned int id)
{
	struct __de_dsi_dev_t __iomem *dsi = DSI(id);

	if (!dsi)
		return -EINVAL;

	writel(DSI_INST_JUMP_SEL_PANEL_ON, &dsi->dsi_inst_jump_sel);
	dsi_inst_restart(dsi);
	return 0;
}
EXPORT_SYMBOL_GPL(tvpanel_dsi_tri_start);

MODULE_LICENSE("GPL v2");
