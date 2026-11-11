// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allwinner sun50iw15 IR receiver ("allwinner,irrx", s_ir_rx@7040000).
 *
 * Rewritten from the vendor sunxi-ir-rx module.  The block captures the
 * demodulated waveform into a FIFO: each word in the data register holds the
 * level in bit 7 and the pulse width in ticks in bits 6:0, and the status
 * register holds the number of captured words in bits 14:8 plus "last sample"
 * (bit 1) and "FIFO error" (bit 0) flags.
 *
 * The vendor converts a tick count with (10700 * ticks + 500) / 1000 and feeds
 * the result to ir_raw_event_store(); the same conversion is kept here so the
 * timings match the stock remote.
 *
 * Key mapping: the vendor module's static keymap was recovered from its
 * .data section.  It is a 30 entry rc_map_table for an NEC extended remote
 * (16 bit address 0x1234) whose key codes are exactly the ones sunxi-ir.kl
 * maps, including the PRJ_* projector functions (key codes 635..758).  That
 * table is registered as "rc-hy300" and used when the device tree does not
 * name a keymap, so the stock remote reports key events out of the box.  A
 * "linux,rc-map-name" property still overrides it.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/pm_wakeup.h>
#include <media/rc-core.h>
#include <media/rc-map.h>

#define SUNXI_IR_CTRL		0x00
#define SUNXI_IR_SAMPLE		0x10
#define SUNXI_IR_DATA		0x20
#define SUNXI_IR_CFG1		0x2c
#define SUNXI_IR_STAT		0x30
#define SUNXI_IR_CFG2		0x34

#define SUNXI_IR_STAT_LAST	BIT(1)
#define SUNXI_IR_STAT_ERR	BIT(0)
#define SUNXI_IR_STAT_CNT(s)	(((s) >> 8) & 0x7f)
#define SUNXI_IR_DATA_LEVEL	BIT(7)
#define SUNXI_IR_DATA_TICKS(d)	((d) & 0x7f)

/*
 * Recovered from the vendor sunxi-ir-rx.ko (.data, 16 byte rc_map_table
 * entries).  The remote sends NEC extended frames with the 16 bit address
 * 0x1234; the key codes below match projector/vendor/usr/keylayout/sunxi-ir.kl.
 */
static struct rc_map_table sunxi_ir_h726_keymap[] = {
	{ RC_SCANCODE_NECX(0x1234, 0x14), KEY_POWER },
	{ RC_SCANCODE_NECX(0x1234, 0xdc), KEY_MUTE },
	{ RC_SCANCODE_NECX(0x1234, 0x58), KEY_VOLUMEDOWN },
	{ RC_SCANCODE_NECX(0x1234, 0xc1), KEY_VOLUMEUP },
	{ RC_SCANCODE_NECX(0x1234, 0x03), KEY_UP },
	{ RC_SCANCODE_NECX(0x1234, 0x02), KEY_DOWN },
	{ RC_SCANCODE_NECX(0x1234, 0x0e), KEY_LEFT },
	{ RC_SCANCODE_NECX(0x1234, 0x1a), KEY_RIGHT },
	{ RC_SCANCODE_NECX(0x1234, 0x07), KEY_OK },
	{ RC_SCANCODE_NECX(0x1234, 0x5c), KEY_BACK },
	/* PRJ_INFO */
	{ RC_SCANCODE_NECX(0x1234, 0x0b), 653 },
	/* PRJ_SOURCE */
	{ RC_SCANCODE_NECX(0x1234, 0xe2), 646 },
	/* PRJ_KEYSTONE_INIT / PRJ_KEYSTONE_SWITCH */
	{ RC_SCANCODE_NECX(0x1234, 0xd8), 753 },
	{ RC_SCANCODE_NECX(0x1234, 0xe1), 754 },
	/* PRJ_FOCUS_UP / PRJ_FOCUS_DN */
	{ RC_SCANCODE_NECX(0x1234, 0x98), 642 },
	{ RC_SCANCODE_NECX(0x1234, 0x93), 643 },
	/* PRJ_PIC_TEST / PRJ_STRESS_TEST / PRJ_CLR_DATA */
	{ RC_SCANCODE_NECX(0x1234, 0x82), 652 },
	{ RC_SCANCODE_NECX(0x1234, 0x01), 651 },
	{ RC_SCANCODE_NECX(0x1234, 0x13), 654 },
	/* PRJ_FACTORY_RESET / PRJ_FACTORY_TEST / PRJ_CUSTOMER1 */
	{ RC_SCANCODE_NECX(0x1234, 0xdb), 635 },
	{ RC_SCANCODE_NECX(0x1234, 0x48), 636 },
	{ RC_SCANCODE_NECX(0x1234, 0xda), 746 },
	/* PRJ_PIC_WHITE / BLACK / CLEAR / LIGHT */
	{ RC_SCANCODE_NECX(0x1234, 0xdd), 749 },
	{ RC_SCANCODE_NECX(0x1234, 0xde), 750 },
	{ RC_SCANCODE_NECX(0x1234, 0xdf), 751 },
	{ RC_SCANCODE_NECX(0x1234, 0xe0), 752 },
	/* PRJ_TEST_RGB / HUIJIE / WIFI / BLUETOOTH */
	{ RC_SCANCODE_NECX(0x1234, 0xa9), 755 },
	{ RC_SCANCODE_NECX(0x1234, 0xaa), 756 },
	{ RC_SCANCODE_NECX(0x1234, 0xa8), 758 },
	{ RC_SCANCODE_NECX(0x1234, 0xa7), 757 },
};

static struct rc_map_list sunxi_ir_h726_map = {
	.map = {
		.scan		= sunxi_ir_h726_keymap,
		.size		= ARRAY_SIZE(sunxi_ir_h726_keymap),
		.len		= ARRAY_SIZE(sunxi_ir_h726_keymap),
		.alloc		= ARRAY_SIZE(sunxi_ir_h726_keymap),
		.rc_proto	= RC_PROTO_NECX,
		.name		= "rc-hy300",
	},
};

static bool sunxi_ir_map_registered;

struct sunxi_ir {
	void __iomem *base;
	struct clk *bus_clk;
	struct clk *mclk;
	struct reset_control *rst;
	struct rc_dev *rc;
	bool wakeup;
	u32 last_ticks;
	u8 last_level;
	bool started;
};

static inline u32 sunxi_ir_ticks_to_ns(u32 ticks)
{
	return (10700 * ticks + 500) / 1000;
}

static void sunxi_ir_store(struct sunxi_ir *ir, bool level, u32 ticks)
{
	struct ir_raw_event ev = { };

	if (!ticks)
		return;

	ev.duration = sunxi_ir_ticks_to_ns(ticks);
	ev.pulse = level;
	ir_raw_event_store(ir->rc, &ev);
}

static irqreturn_t sunxi_ir_irq(int irq, void *dev_id)
{
	struct sunxi_ir *ir = dev_id;
	u32 status = readl(ir->base + SUNXI_IR_STAT);
	unsigned int count, i;

	writel(status, ir->base + SUNXI_IR_STAT);

	count = SUNXI_IR_STAT_CNT(status);

	for (i = 0; i < count; i++) {
		u32 data = readl(ir->base + SUNXI_IR_DATA);
		bool level = !(data & SUNXI_IR_DATA_LEVEL);
		u32 ticks = SUNXI_IR_DATA_TICKS(data);

		if (!ir->started) {
			ir->last_level = level;
			ir->last_ticks = ticks;
			ir->started = true;
			continue;
		}

		if (level == ir->last_level) {
			ir->last_ticks += ticks;
			continue;
		}

		sunxi_ir_store(ir, ir->last_level, ir->last_ticks);
		ir->last_level = level;
		ir->last_ticks = ticks;
	}

	if (status & SUNXI_IR_STAT_LAST) {
		if (ir->started && ir->last_ticks)
			sunxi_ir_store(ir, ir->last_level, ir->last_ticks);

		ir_raw_event_handle(ir->rc);
		ir->started = false;
		ir->last_ticks = 0;

		if (ir->wakeup)
			pm_wakeup_dev_event(&ir->rc->dev, 0, false);
	}

	if (status & SUNXI_IR_STAT_ERR) {
		dev_dbg(&ir->rc->dev, "receiver fifo error\n");
		ir->started = false;
		ir->last_ticks = 0;
	}

	return IRQ_HANDLED;
}

static int sunxi_ir_hw_init(struct platform_device *pdev, struct sunxi_ir *ir)
{
	struct device *dev = &pdev->dev;
	u32 v;
	int ret;

	ret = clk_prepare_enable(ir->bus_clk);
	if (ret)
		return dev_err_probe(dev, ret, "cannot enable bus clock\n");

	ret = clk_prepare_enable(ir->mclk);
	if (ret) {
		clk_disable_unprepare(ir->bus_clk);
		return dev_err_probe(dev, ret, "cannot enable module clock\n");
	}

	if (!IS_ERR_OR_NULL(ir->rst)) {
		ret = reset_control_reset(ir->rst);
		if (ret)
			dev_warn(dev, "reset failed: %d\n", ret);
	}

	v = readl(ir->base + SUNXI_IR_CTRL);
	writel(v | 0x30, ir->base + SUNXI_IR_CTRL);

	writel(0, ir->base + SUNXI_IR_CFG2);
	writel(readl(ir->base + SUNXI_IR_CFG2) | 0x2, ir->base + SUNXI_IR_CFG2);
	writel(readl(ir->base + SUNXI_IR_CFG2) | 0xb00, ir->base + SUNXI_IR_CFG2);
	v = readl(ir->base + SUNXI_IR_CFG2) & ~0xff5fffff;
	writel(v | 0x200000, ir->base + SUNXI_IR_CFG2);
	writel(readl(ir->base + SUNXI_IR_CFG2) | 0x80, ir->base + SUNXI_IR_CFG2);

	writel(4, ir->base + SUNXI_IR_SAMPLE);
	writel(239, ir->base + SUNXI_IR_STAT);

	writel(readl(ir->base + SUNXI_IR_CFG1) | 0x13, ir->base + SUNXI_IR_CFG1);
	writel(readl(ir->base + SUNXI_IR_CFG1) | 0x1300, ir->base + SUNXI_IR_CFG1);

	writel(readl(ir->base + SUNXI_IR_CTRL) | 0x40, ir->base + SUNXI_IR_CTRL);
	writel(readl(ir->base + SUNXI_IR_CTRL) | 0x3, ir->base + SUNXI_IR_CTRL);

	return 0;
}

static int sunxi_ir_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct sunxi_ir *ir;
	const char *map_name = NULL;
	int irq, ret;

	ir = devm_kzalloc(dev, sizeof(*ir), GFP_KERNEL);
	if (!ir)
		return -ENOMEM;

	ir->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ir->base))
		return PTR_ERR(ir->base);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ir->bus_clk = devm_clk_get(dev, "bus");
	if (IS_ERR(ir->bus_clk))
		return dev_err_probe(dev, PTR_ERR(ir->bus_clk), "no bus clock\n");

	ir->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(ir->mclk))
		return dev_err_probe(dev, PTR_ERR(ir->mclk), "no module clock\n");

	ir->rst = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(ir->rst))
		return dev_err_probe(dev, PTR_ERR(ir->rst), "no reset line\n");

	ir->wakeup = of_property_read_bool(np, "wakeup-source");

	ir->rc = devm_rc_allocate_device(dev, RC_DRIVER_IR_RAW);
	if (!ir->rc)
		return -ENOMEM;

	ir->rc->device_name = "sunxi-ir";
	ir->rc->driver_name = "sunxi-ir";
	ir->rc->input_phys = "sunxi-ir/input0";
	ir->rc->input_id.bustype = BUS_HOST;
	if (!sunxi_ir_map_registered) {
		ret = rc_map_register(&sunxi_ir_h726_map);
		if (ret)
			return dev_err_probe(dev, ret, "cannot register keymap\n");
		sunxi_ir_map_registered = true;
	}

	ir->rc->map_name = of_get_property(np, "linux,rc-map-name", NULL);
	if (!ir->rc->map_name)
		ir->rc->map_name = sunxi_ir_h726_map.map.name;
	ir->rc->allowed_protocols = RC_PROTO_BIT_NEC | RC_PROTO_BIT_NECX |
				    RC_PROTO_BIT_NEC32 | RC_PROTO_BIT_RC5;
	ir->rc->priv = ir;
	ir->rc->dev.parent = dev;

	ret = devm_rc_register_device(dev, ir->rc);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register rc device\n");

	if (ir->rc->map_name)
		map_name = ir->rc->map_name;

	ret = sunxi_ir_hw_init(pdev, ir);
	if (ret)
		return ret;

	ret = devm_request_irq(dev, irq, sunxi_ir_irq, 0, "sunxi-ir", ir);
	if (ret)
		return dev_err_probe(dev, ret, "cannot request irq\n");

	platform_set_drvdata(pdev, ir);

	dev_info(dev, "sunxi IR receiver ready%s%s\n",
		 map_name ? ", keymap " : "", map_name ?: "");


	return 0;
}

static int sunxi_ir_remove(struct platform_device *pdev)
{
	struct sunxi_ir *ir = platform_get_drvdata(pdev);

	writel(0, ir->base + SUNXI_IR_CTRL);
	clk_disable_unprepare(ir->mclk);
	clk_disable_unprepare(ir->bus_clk);

	return 0;
}

static const struct of_device_id sunxi_ir_of_match[] = {
	{ .compatible = "allwinner,irrx" },
	{ }
};
MODULE_DEVICE_TABLE(of, sunxi_ir_of_match);

static struct platform_driver sunxi_ir_driver = {
	.probe = sunxi_ir_probe,
	.remove = sunxi_ir_remove,
	.driver = {
		.name = "sunxi-ir",
		.of_match_table = sunxi_ir_of_match,
	},
};
module_platform_driver(sunxi_ir_driver);

MODULE_DESCRIPTION("Allwinner sun50iw15 IR receiver");
MODULE_LICENSE("GPL");
