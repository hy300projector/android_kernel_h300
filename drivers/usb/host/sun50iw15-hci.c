// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allwinner sun50iw15 (H726 / HY300) EHCI/OHCI glue.
 *
 * The vendor modules (sunxi-hci + ehci-sunxi/ohci-sunxi) did everything
 * themselves: open_clock() enabled the clocks and resets, phy_init() and
 * usb_passby() set the PHY up, sunxi_set_host_vbus() drove VBUS and the
 * manager called sunxi_usb_enable_hcd() to register the controller.  This
 * driver keeps that setup but leaves the HCD itself to the generic
 * ehci-platform/ohci-platform drivers: it enables clocks, deasserts resets,
 * turns on drvvbus, powers the shared USB PHY and then registers a child
 * platform device carrying the controller's memory window and interrupt.
 *
 * Port selection follows the vendor manager: it only brought the HCI up for
 * ports that are not device-only, so a port whose usbcN node says
 * usb_port_type = 0 (the HY300's usbc0, the USB device port) is skipped here
 * as well and its PHY stays with the device controller.
 */

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/property.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/slab.h>

enum sunxi_hci_type {
	SUNXI_HCI_TYPE_EHCI = 1,
	SUNXI_HCI_TYPE_OHCI,
};

struct sun50iw15_hci {
	bool ohci;
	struct clk *clk_bus;
	struct clk *clk_ohci;
	struct reset_control *rst_hci;
	struct reset_control *rst_phy;
	struct regulator *vbus;
	bool vbus_on;
	struct phy *phy;
	struct platform_device *hcd;
};

/* The vendor manager only insmod'ed the host side of a port that is not
 * configured as a pure device port. */
static bool sunxi_hci_port_is_device_only(u32 ctrl)
{
	struct device_node *np;
	char type[8];
	u32 port_type;
	bool ret = false;

	snprintf(type, sizeof(type), "usbc%u", ctrl);
	np = of_find_node_by_type(NULL, type);
	if (!np)
		return false;

	if (!of_property_read_u32(np, "usb_port_type", &port_type))
		ret = port_type == 0;

	of_node_put(np);
	return ret;
}

static int sun50iw15_hci_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sun50iw15_hci *hci;
	const void *data = device_get_match_data(dev);
	struct resource *mem, *irq;
	struct resource res[2];
	const char *name;
	u32 ctrl = 0;
	int ret;

	hci = devm_kzalloc(dev, sizeof(*hci), GFP_KERNEL);
	if (!hci)
		return -ENOMEM;

	hci->ohci = (unsigned long)data == SUNXI_HCI_TYPE_OHCI;
	platform_set_drvdata(pdev, hci);

	of_property_read_u32(dev->of_node, "hci_ctrl_no", &ctrl);

	if (sunxi_hci_port_is_device_only(ctrl)) {
		dev_info(dev, "usbc%u is device-only, not starting the host controller\n",
			 ctrl);
		return -ENODEV;
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!mem || !irq) {
		dev_err(dev, "missing memory or interrupt resource\n");
		return -ENODEV;
	}

	hci->clk_bus = devm_clk_get_optional(dev, "bus_hci");
	if (IS_ERR(hci->clk_bus))
		return dev_err_probe(dev, PTR_ERR(hci->clk_bus),
				     "no bus_hci clock\n");

	if (hci->ohci) {
		hci->clk_ohci = devm_clk_get_optional(dev, "ohci");
		if (IS_ERR(hci->clk_ohci))
			return dev_err_probe(dev, PTR_ERR(hci->clk_ohci),
					     "no ohci clock\n");
	}

	ret = clk_prepare_enable(hci->clk_bus);
	if (ret)
		return dev_err_probe(dev, ret, "cannot enable bus_hci\n");

	ret = clk_prepare_enable(hci->clk_ohci);
	if (ret) {
		dev_err_probe(dev, ret, "cannot enable ohci clock\n");
		goto err_bus_clk;
	}

	hci->rst_hci = devm_reset_control_get_optional_shared(dev, "hci");
	if (IS_ERR(hci->rst_hci)) {
		ret = PTR_ERR(hci->rst_hci);
		goto err_ohci_clk;
	}

	/* The PHY reset is shared between the EHCI and OHCI of a port. */
	hci->rst_phy = devm_reset_control_get_optional_shared(dev, "phy");
	if (IS_ERR(hci->rst_phy)) {
		ret = PTR_ERR(hci->rst_phy);
		goto err_ohci_clk;
	}

	ret = reset_control_deassert(hci->rst_hci);
	if (ret)
		goto err_ohci_clk;

	ret = reset_control_deassert(hci->rst_phy);
	if (ret)
		goto err_assert_hci;

	/* drvvbus is the port's VBUS supply; the vendor enabled it here too. */
	hci->vbus = devm_regulator_get_optional(dev, "drvvbus");
	if (IS_ERR(hci->vbus)) {
		ret = PTR_ERR(hci->vbus);
		hci->vbus = NULL;
		if (ret != -ENODEV)
			goto err_assert_phy;
	}

	if (hci->vbus) {
		ret = regulator_enable(hci->vbus);
		if (ret) {
			dev_err_probe(dev, ret, "cannot enable drvvbus\n");
			goto err_assert_phy;
		}
		hci->vbus_on = true;
	}

	hci->phy = devm_phy_optional_get(dev, NULL);
	if (IS_ERR(hci->phy)) {
		ret = PTR_ERR(hci->phy);
		hci->phy = NULL;
		goto err_vbus;
	}

	if (hci->phy) {
		ret = phy_init(hci->phy);
		if (ret)
			goto err_vbus;

		ret = phy_set_mode(hci->phy, PHY_MODE_USB_HOST);
		if (ret)
			goto err_phy_exit;

		ret = phy_power_on(hci->phy);
		if (ret)
			goto err_phy_exit;
	} else {
		dev_warn(dev, "no USB PHY, the port may not work\n");
	}

	res[0] = *mem;
	res[1] = *irq;
	name = hci->ohci ? "ohci-platform" : "ehci-platform";
	hci->hcd = platform_device_register_resndata(dev, name,
						     PLATFORM_DEVID_AUTO,
						     res, ARRAY_SIZE(res),
						     NULL, 0);
	if (IS_ERR(hci->hcd)) {
		ret = PTR_ERR(hci->hcd);
		dev_err_probe(dev, ret, "cannot register %s\n", name);
		goto err_phy_off;
	}

	dev_info(dev, "started %s on port %u\n", name, ctrl);
	return 0;

err_phy_off:
	if (hci->phy)
		phy_power_off(hci->phy);
err_phy_exit:
	if (hci->phy)
		phy_exit(hci->phy);
err_vbus:
	if (hci->vbus_on)
		regulator_disable(hci->vbus);
err_assert_phy:
	reset_control_assert(hci->rst_phy);
err_assert_hci:
	reset_control_assert(hci->rst_hci);
err_ohci_clk:
	clk_disable_unprepare(hci->clk_ohci);
err_bus_clk:
	clk_disable_unprepare(hci->clk_bus);

	return ret;
}

static int sun50iw15_hci_remove(struct platform_device *pdev)
{
	struct sun50iw15_hci *hci = platform_get_drvdata(pdev);

	platform_device_unregister(hci->hcd);

	if (hci->phy) {
		phy_power_off(hci->phy);
		phy_exit(hci->phy);
	}

	if (hci->vbus_on)
		regulator_disable(hci->vbus);

	reset_control_assert(hci->rst_phy);
	reset_control_assert(hci->rst_hci);
	clk_disable_unprepare(hci->clk_ohci);
	clk_disable_unprepare(hci->clk_bus);

	return 0;
}

static const struct of_device_id sun50iw15_hci_of_match[] = {
	{ .compatible = "allwinner,sunxi-ehci0", .data = (void *)SUNXI_HCI_TYPE_EHCI },
	{ .compatible = "allwinner,sunxi-ehci1", .data = (void *)SUNXI_HCI_TYPE_EHCI },
	{ .compatible = "allwinner,sunxi-ehci2", .data = (void *)SUNXI_HCI_TYPE_EHCI },
	{ .compatible = "allwinner,sunxi-ohci0", .data = (void *)SUNXI_HCI_TYPE_OHCI },
	{ .compatible = "allwinner,sunxi-ohci1", .data = (void *)SUNXI_HCI_TYPE_OHCI },
	{ .compatible = "allwinner,sunxi-ohci2", .data = (void *)SUNXI_HCI_TYPE_OHCI },
	{ }
};
MODULE_DEVICE_TABLE(of, sun50iw15_hci_of_match);

static struct platform_driver sun50iw15_hci_driver = {
	.probe	= sun50iw15_hci_probe,
	.remove	= sun50iw15_hci_remove,
	.driver	= {
		.name		= "sun50iw15-hci",
		.of_match_table	= sun50iw15_hci_of_match,
	},
};
module_platform_driver(sun50iw15_hci_driver);

MODULE_DESCRIPTION("Allwinner sun50iw15 EHCI/OHCI glue");
MODULE_LICENSE("GPL");
