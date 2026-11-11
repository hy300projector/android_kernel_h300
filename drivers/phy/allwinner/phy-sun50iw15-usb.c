// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allwinner sun50iw15 (H726 / HY300) USB PHY.
 *
 * One shared block at 0x4100000 serves the OTG controller and the EHCI/OHCI
 * pair of every port.  Ported from the vendor sunxi-hci module:
 *
 *   usb_phyx_tp_write()        -> sunxi_usbphy_tp_write_otg()
 *   usb_new_phyx_tp_write()    -> sunxi_usbphy_tp_write_port()
 *   usb_new_phyx_write()       -> sunxi_usbphy_program()
 *   USBC_SelectPhyToDevice()   -> set_mode(PHY_MODE_USB_DEVICE)
 *   __usb_passby()             -> sunxi_usbphy_passby_get()/_put()
 *   open_clock()/close_clock() -> sunxi_usbphy_siddq_get()/_put()
 *
 * The PHY is programmed through a bit-banged "test port" (TP) interface.
 * On port 0 it lives in the common block at 0x410/0x411 (data/address), and
 * the block's 0x420 bit 0 selects whether the OTG controller or the HCI owns
 * the PHY.  On ports 1..3 the same interface is inside each controller block
 * at 0x810 (bit 0 = data strobe, bit 1 = address phase, bit 7 = data out),
 * with the address at 0x811 and read data at 0x824; bit 3 of that register is
 * SIDDQ, which powers the port's PHY down while no controller uses it.
 *
 * Not ported: the vendor's efuse ("SID" word at offset 0x18) PHY trim, which
 * needs an SID cell this kernel does not expose, and the HSIC passby bits,
 * which no port on this board uses.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

#define SUN50IW15_USBPHY_PORTS		4

/* Common block, 0x4100000. */
#define SUNXI_USBPHY_SELECT		0x420
#define SUNXI_USBPHY_SELECT_DEVICE	BIT(0)
#define SUNXI_USBPHY_OTG_TP_DATA	0x410
#define SUNXI_USBPHY_OTG_TP_ADDR	0x411
#define SUNXI_USBPHY_OTG_TP_READ	0x424

/* Per-port controller block (port 0 block is the EHCI0 window). */
#define SUNXI_USBPHY_PORT_PASSBY	0x800
#define SUNXI_USBPHY_PORT_PASSBY_BITS	0x701
#define SUNXI_USBPHY_PORT_PASSBY_HSIC	0x110703
#define SUNXI_USBPHY_PORT_CTRL		0x810
#define SUNXI_USBPHY_PORT_TP_ADDR	0x811
#define SUNXI_USBPHY_PORT_TP_READ	0x824

#define SUNXI_USBPHY_CTRL_STROBE	BIT(0)
#define SUNXI_USBPHY_CTRL_ADDR		BIT(1)
#define SUNXI_USBPHY_CTRL_SIDDQ		BIT(3)
#define SUNXI_USBPHY_CTRL_DATA		BIT(7)

#define SUNXI_USBPHY_DEFAULT_RANGE	0x218

struct sun50iw15_usbphy;

struct sun50iw15_usbphy_port {
	struct sun50iw15_usbphy *phy;
	struct phy *phy_dev;
	void __iomem *base;
	unsigned int id;
	unsigned int siddq_users;
	unsigned int passby_users;
	bool tuned;
};

struct sun50iw15_usbphy {
	struct device *dev;
	void __iomem *common;
	void __iomem *sid;		/* efuse window, for the PHY trim */
	struct mutex lock;		/* protects the port counters */
	u32 phy_range;			/* efuse offset of the trim word */
	struct sun50iw15_usbphy_port ports[SUN50IW15_USBPHY_PORTS];
};

/*
 * Port 0 TP write: data bit at bit 7, strobe at bit 0, address latched while
 * the strobe is low.  The select bit is raised for the duration of the access
 * and restored afterwards, exactly like usb_phyx_tp_write().
 */
static void sunxi_usbphy_tp_write_otg(struct sun50iw15_usbphy *p, u8 addr,
				      u32 val, unsigned int nbits)
{
	void __iomem *common = p->common;
	u32 select = readl(common + SUNXI_USBPHY_SELECT);
	unsigned int i;

	writel(select | SUNXI_USBPHY_SELECT_DEVICE,
	       common + SUNXI_USBPHY_SELECT);

	for (i = 0; i < nbits; i++) {
		u8 ctl;

		writeb(addr++, common + SUNXI_USBPHY_OTG_TP_ADDR);

		ctl = readb(common + SUNXI_USBPHY_OTG_TP_DATA);
		writeb(ctl & ~SUNXI_USBPHY_CTRL_STROBE,
		       common + SUNXI_USBPHY_OTG_TP_DATA);

		ctl = readb(common + SUNXI_USBPHY_OTG_TP_DATA);
		ctl = (ctl & 0x7f) |
		      ((val & 1) ? SUNXI_USBPHY_CTRL_DATA : 0);
		writeb(ctl, common + SUNXI_USBPHY_OTG_TP_DATA);
		writeb(ctl | SUNXI_USBPHY_CTRL_STROBE,
		       common + SUNXI_USBPHY_OTG_TP_DATA);

		ctl = readb(common + SUNXI_USBPHY_OTG_TP_DATA);
		writeb(ctl & ~SUNXI_USBPHY_CTRL_STROBE,
		       common + SUNXI_USBPHY_OTG_TP_DATA);

		val >>= 1;
	}

	writel(select, common + SUNXI_USBPHY_SELECT);
}

/*
 * Ports 1..3 TP write: bit 1 raises the address phase, bit 7 carries the data
 * bit and bit 0 strobes it.  This is usb_new_phyx_tp_write().
 */
static void sunxi_usbphy_tp_write_port(struct sun50iw15_usbphy_port *port,
				       u8 addr, u32 val, unsigned int nbits)
{
	void __iomem *base = port->base;
	unsigned int i;

	for (i = 0; i < nbits; i++) {
		u8 data = (val & 1) ? SUNXI_USBPHY_CTRL_DATA : 0;
		u8 ctl;

		ctl = readb(base + SUNXI_USBPHY_PORT_CTRL);
		writeb(ctl | SUNXI_USBPHY_CTRL_ADDR,
		       base + SUNXI_USBPHY_PORT_CTRL);
		writeb(addr++, base + SUNXI_USBPHY_PORT_TP_ADDR);

		ctl = readb(base + SUNXI_USBPHY_PORT_CTRL);
		writeb(ctl & ~SUNXI_USBPHY_CTRL_STROBE,
		       base + SUNXI_USBPHY_PORT_CTRL);

		ctl = readb(base + SUNXI_USBPHY_PORT_CTRL);
		writeb((ctl & 0x7f) | data, base + SUNXI_USBPHY_PORT_CTRL);
		writeb((ctl & 0x7f) | data | SUNXI_USBPHY_CTRL_STROBE,
		       base + SUNXI_USBPHY_PORT_CTRL);
		writeb((ctl & 0x7e) | data, base + SUNXI_USBPHY_PORT_CTRL);

		ctl = readb(base + SUNXI_USBPHY_PORT_CTRL);
		writeb(ctl & ~SUNXI_USBPHY_CTRL_ADDR,
		       base + SUNXI_USBPHY_PORT_CTRL);

		val >>= 1;
	}
}

static void sunxi_usbphy_tp(struct sun50iw15_usbphy_port *port, u8 addr,
			    u32 val, unsigned int nbits)
{
	if (port->id == 0)
		sunxi_usbphy_tp_write_otg(port->phy, addr, val, nbits);
	else
		sunxi_usbphy_tp_write_port(port, addr, val, nbits);
}

/*
 * usbc_new_phy_init(): the PHY trim comes from efuse, not from a literal.
 * The vendor reads the 32-bit word at SID + 0x200 + 0x18 - which is why the DT
 * property is named phy_range and carries 0x218 - then:
 *
 *   tp(0x1c, 0, 3)                      always
 *   bit16 set, bit17 clear: tp(0x60,0,1), tp(0x44,sid[21:18],4),
 *                           tp(0x36,sid[24:22],3)
 *   bit16 and bit17 set:    tp(0x60,1,1), tp(0x61,sid[24:22],3),
 *                           tp(0x44,sid[21:18],4)
 *
 * The previous code spread phy_range itself as a 12-bit tuning word, which
 * programmed 0x218 into the trim registers instead of the calibrated value.
 */
static void sunxi_usbphy_load_trim(struct sun50iw15_usbphy_port *port)
{
	struct sun50iw15_usbphy *p = port->phy;
	u32 sid = 0, again;

	if (port->tuned)
		return;

	if (p->sid) {
		sid = readl(p->sid + p->phy_range);
		/* SID reads latch; require two consecutive equal values. */
		again = readl(p->sid + p->phy_range);
		if (again != sid)
			sid = readl(p->sid + p->phy_range);
	} else {
		dev_warn_once(p->dev, "no SID window, PHY trim unavailable\n");
	}

	sunxi_usbphy_tp(port, 0x1c, 0, 3);

	if (sid & BIT(16)) {
		if (!(sid & BIT(17))) {
			sunxi_usbphy_tp(port, 0x60, 0, 1);
			sunxi_usbphy_tp(port, 0x44, (sid & 0x3fffff) >> 18, 4);
			sunxi_usbphy_tp(port, 0x36, (sid & 0x1ffffff) >> 22, 3);
		} else {
			sunxi_usbphy_tp(port, 0x60, 1, 1);
			sunxi_usbphy_tp(port, 0x61, (sid & 0x1ffffff) >> 22, 3);
			sunxi_usbphy_tp(port, 0x44, (sid & 0x3fffff) >> 18, 4);
		}
	}

	port->tuned = true;
	dev_info(p->dev, "port %u: SID trim 0x%08x (offset 0x%x)%s\n",
		 port->id, sid, p->phy_range,
		 (sid & BIT(16)) ? "" : ", no calibration programmed");
}

/* __usb_passby(): 0x701 enables the port's passby path in the HCI block; the
 * vendor programs it on the first user and clears 0xfffff8fe on the last. */
static void sunxi_usbphy_passby_get(struct sun50iw15_usbphy_port *port)
{
	u32 val;

	if (port->passby_users++)
		return;

	val = readl(port->base + SUNXI_USBPHY_PORT_PASSBY);
	writel(val | SUNXI_USBPHY_PORT_PASSBY_BITS,
	       port->base + SUNXI_USBPHY_PORT_PASSBY);
}

static void sunxi_usbphy_passby_put(struct sun50iw15_usbphy_port *port)
{
	u32 val;

	if (!port->passby_users || --port->passby_users)
		return;

	val = readl(port->base + SUNXI_USBPHY_PORT_PASSBY);
	writel(val & ~SUNXI_USBPHY_PORT_PASSBY_BITS,
	       port->base + SUNXI_USBPHY_PORT_PASSBY);
}

/* SIDDQ: bit 3 of 0x810 holds the port PHY in power down until the first
 * controller (EHCI or OHCI of that port) needs it. */
static void sunxi_usbphy_siddq_get(struct sun50iw15_usbphy_port *port)
{
	u32 val;

	if (port->siddq_users++)
		return;

	val = readl(port->base + SUNXI_USBPHY_PORT_CTRL);
	writel(val & ~SUNXI_USBPHY_CTRL_SIDDQ,
	       port->base + SUNXI_USBPHY_PORT_CTRL);
}

static void sunxi_usbphy_siddq_put(struct sun50iw15_usbphy_port *port)
{
	u32 val;

	if (!port->siddq_users || --port->siddq_users)
		return;

	val = readl(port->base + SUNXI_USBPHY_PORT_CTRL);
	writel(val | SUNXI_USBPHY_CTRL_SIDDQ,
	       port->base + SUNXI_USBPHY_PORT_CTRL);
}

static int sunxi_usbphy_init(struct phy *phy)
{
	struct sun50iw15_usbphy_port *port = phy_get_drvdata(phy);
	struct sun50iw15_usbphy *p = port->phy;
	int ret = 0;

	mutex_lock(&p->lock);
	sunxi_usbphy_load_trim(port);
	mutex_unlock(&p->lock);

	return ret;
}

static int sunxi_usbphy_exit(struct phy *phy)
{
	return 0;
}

static int sunxi_usbphy_power_on(struct phy *phy)
{
	struct sun50iw15_usbphy_port *port = phy_get_drvdata(phy);
	struct sun50iw15_usbphy *p = port->phy;

	mutex_lock(&p->lock);
	sunxi_usbphy_load_trim(port);
	sunxi_usbphy_siddq_get(port);
	sunxi_usbphy_passby_get(port);
	mutex_unlock(&p->lock);

	return 0;
}

static int sunxi_usbphy_power_off(struct phy *phy)
{
	struct sun50iw15_usbphy_port *port = phy_get_drvdata(phy);
	struct sun50iw15_usbphy *p = port->phy;

	mutex_lock(&p->lock);
	sunxi_usbphy_passby_put(port);
	sunxi_usbphy_siddq_put(port);
	mutex_unlock(&p->lock);

	return 0;
}

/*
 * 0x420 bit 0 routes the OTG PHY: set means the device controller owns it
 * (USBC_SelectPhyToDevice), clear means the HCI does, which is what
 * open_clock() leaves behind for port 0.  Only port 0 has this bit.
 */
static int sunxi_usbphy_set_mode(struct phy *phy, enum phy_mode mode,
				 int submode)
{
	struct sun50iw15_usbphy_port *port = phy_get_drvdata(phy);
	struct sun50iw15_usbphy *p = port->phy;
	u32 val;

	if (port->id != 0)
		return 0;

	if (mode != PHY_MODE_USB_HOST && mode != PHY_MODE_USB_DEVICE)
		return -EINVAL;

	mutex_lock(&p->lock);
	val = readl(p->common + SUNXI_USBPHY_SELECT);
	if (mode == PHY_MODE_USB_DEVICE)
		val |= SUNXI_USBPHY_SELECT_DEVICE;
	else
		val &= ~SUNXI_USBPHY_SELECT_DEVICE;
	writel(val, p->common + SUNXI_USBPHY_SELECT);
	mutex_unlock(&p->lock);

	return 0;
}

static const struct phy_ops sunxi_usbphy_ops = {
	.init		= sunxi_usbphy_init,
	.exit		= sunxi_usbphy_exit,
	.power_on	= sunxi_usbphy_power_on,
	.power_off	= sunxi_usbphy_power_off,
	.set_mode	= sunxi_usbphy_set_mode,
	.owner		= THIS_MODULE,
};

/* Map a register window without claiming it: the HCI drivers request the
 * very same ranges, so the PHY only needs its own mapping. */
static void __iomem *sunxi_usbphy_map(struct platform_device *pdev,
				      const char *name)
{
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, name);
	if (!res)
		return NULL;

	return devm_ioremap(&pdev->dev, res->start, resource_size(res));
}

/*
 * The DT names a port in the second cell (phys = <&usbphy 0x00>, <&usbphy
 * 0x01>), which is why the node declares #phy-cells = <1>.  Every port's phy
 * is created against this same device node, so of_phy_simple_xlate() would
 * return the first one for every port and silently hand port 1 the port 0
 * window.  Resolve the index against the per-port array instead, and fall back
 * to the first created phy only when this node does not map the requested
 * port, so a DT naming an unmapped port still binds.
 */
static struct phy *sunxi_usbphy_of_xlate(struct device *dev,
					 struct of_phandle_args *args)
{
	struct sun50iw15_usbphy *p = dev_get_drvdata(dev);
	unsigned int id, i;

	if (!p || args->np != dev->of_node)
		return ERR_PTR(-ENODEV);

	id = args->args_count ? args->args[0] : 0;
	if (id < SUN50IW15_USBPHY_PORTS && p->ports[id].phy_dev)
		return p->ports[id].phy_dev;

	for (i = 0; i < SUN50IW15_USBPHY_PORTS; i++) {
		if (p->ports[i].phy_dev) {
			dev_warn_once(dev,
				      "no window for phy %u, using phy %u\n",
				      id, i);
			return p->ports[i].phy_dev;
		}
	}

	return ERR_PTR(-ENODEV);
}

static int sunxi_usbphy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sun50iw15_usbphy *p;
	struct phy_provider *provider;
	unsigned int i;
	int ret;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	p->dev = dev;
	mutex_init(&p->lock);
	platform_set_drvdata(pdev, p);

	/*
	 * phy_range is the efuse offset of the trim word (0x200 + 0x18 on this
	 * SoC), not the trim value itself.
	 */
	p->phy_range = SUNXI_USBPHY_DEFAULT_RANGE;
	of_property_read_u32(dev->of_node, "phy_range", &p->phy_range);
	if (!p->phy_range || (p->phy_range & 3)) {
		dev_warn(dev, "phy_range 0x%x is not a 4-byte aligned offset, using 0x%x\n",
			 p->phy_range, SUNXI_USBPHY_DEFAULT_RANGE);
		p->phy_range = SUNXI_USBPHY_DEFAULT_RANGE;
	}

	{
		struct device_node *sid;
		struct resource res;

		sid = of_find_compatible_node(NULL, NULL, "allwinner,sunxi-sid");
		if (!sid) {
			dev_warn(dev, "no allwinner,sunxi-sid node: PHY trim skipped\n");
		} else if (of_address_to_resource(sid, 0, &res) ||
			   resource_size(&res) <= p->phy_range) {
			dev_warn(dev, "SID window too small for offset 0x%x\n",
				 p->phy_range);
		} else {
			p->sid = of_iomap(sid, 0);
			if (!p->sid)
				dev_warn(dev, "cannot map the SID window\n");
		}
	}

	p->common = sunxi_usbphy_map(pdev, "common");
	if (!p->common) {
		dev_err(dev, "cannot map the common block\n");
		return -ENOMEM;
	}

	for (i = 0; i < SUN50IW15_USBPHY_PORTS; i++) {
		struct sun50iw15_usbphy_port *port = &p->ports[i];
		char name[8];

		snprintf(name, sizeof(name), "usb%u", i);
		port->base = sunxi_usbphy_map(pdev, name);
		if (!port->base)
			continue;

		port->phy = p;
		port->id = i;

		port->phy_dev = devm_phy_create(dev, NULL, &sunxi_usbphy_ops);
		if (IS_ERR(port->phy_dev))
			return dev_err_probe(dev, PTR_ERR(port->phy_dev),
					     "cannot create phy %u\n", i);

		phy_set_drvdata(port->phy_dev, port);
	}

	provider = devm_of_phy_provider_register(dev, sunxi_usbphy_of_xlate);
	if (IS_ERR(provider))
		return dev_err_probe(dev, PTR_ERR(provider),
				     "cannot register the phy provider\n");

	dev_info(dev, "shared USB PHY ready, tuning word 0x%03x\n",
		 p->phy_range);

	ret = 0;
	return ret;
}

static const struct of_device_id sunxi_usbphy_of_match[] = {
	{ .compatible = "allwinner,sun50iw15-usbphy" },
	{ }
};
MODULE_DEVICE_TABLE(of, sunxi_usbphy_of_match);

static struct platform_driver sunxi_usbphy_driver = {
	.probe	= sunxi_usbphy_probe,
	.driver	= {
		.name		= "sun50iw15-usbphy",
		.of_match_table	= sunxi_usbphy_of_match,
	},
};
module_platform_driver(sunxi_usbphy_driver);

MODULE_DESCRIPTION("Allwinner sun50iw15 USB PHY");
MODULE_LICENSE("GPL");
