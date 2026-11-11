// SPDX-License-Identifier: GPL-2.0-only
/*
 * sunxi_ac_virtual_power - virtual "AC" power supply.
 *
 * Rewritten from the vendor sunxi_ac_virtual_power.ko.  The vendor driver
 * binds "sunxi-virtual-ac-power", registers a mains supply named
 * "sunxi-ac-power" and reports PRESENT and ONLINE as 1 unconditionally, so
 * userspace always sees the box running from mains power.
 *
 * The vendor get_property() returned 0 for every other property without
 * filling the value in; here unknown properties return -EINVAL, which is what
 * the power_supply core expects and stops it reading uninitialised stack.
 */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>

struct sunxi_ac_virtual {
	struct device *dev;
	struct power_supply *ac;
};

static enum power_supply_property sunxi_virtual_ac_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static int sunxi_virtual_ac_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 1;
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct power_supply_desc sunxi_virtual_ac_desc = {
	.name		= "sunxi-ac-power",
	.type		= POWER_SUPPLY_TYPE_MAINS,
	.properties	= sunxi_virtual_ac_props,
	.num_properties	= ARRAY_SIZE(sunxi_virtual_ac_props),
	.get_property	= sunxi_virtual_ac_get_property,
};

static int sunxi_ac_virtual_power_probe(struct platform_device *pdev)
{
	struct power_supply_config cfg = { };
	struct sunxi_ac_virtual *ac;

	ac = devm_kzalloc(&pdev->dev, sizeof(*ac), GFP_KERNEL);
	if (!ac)
		return -ENOMEM;

	ac->dev = &pdev->dev;
	cfg.of_node = pdev->dev.of_node;

	ac->ac = devm_power_supply_register(&pdev->dev, &sunxi_virtual_ac_desc,
					    &cfg);
	if (IS_ERR(ac->ac)) {
		dev_err(&pdev->dev,
			"sunxi_virtual failed to register ac power\n");
		return PTR_ERR(ac->ac);
	}

	platform_set_drvdata(pdev, ac);
	return 0;
}

static int sunxi_ac_virtual_power_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id sunxi_ac_virtual_power_match[] = {
	{ .compatible = "sunxi-virtual-ac-power" },
	{ .compatible = "allwinner,sunxi-virtual-ac-power" },
	{ }
};
MODULE_DEVICE_TABLE(of, sunxi_ac_virtual_power_match);

static struct platform_driver sunxi_ac_virtual_power_driver = {
	.probe	= sunxi_ac_virtual_power_probe,
	.remove	= sunxi_ac_virtual_power_remove,
	.driver	= {
		.name		= "sunxi_ac_virtual_power",
		.of_match_table	= sunxi_ac_virtual_power_match,
	},
};
module_platform_driver(sunxi_ac_virtual_power_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("sunxi_virtual ac power driver");
MODULE_AUTHOR("liufeng <liufeng@allwinnertech.com>");
