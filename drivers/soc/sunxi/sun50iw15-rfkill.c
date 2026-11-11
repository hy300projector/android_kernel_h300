// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allwinner sun50iw15 wireless/Bluetooth power and rfkill control.
 *
 * Replacement for the vendor sunxi_rfkill module, covering the parts this
 * board uses:
 *
 *   rfkill {
 *           compatible = "allwinner,sunxi-rfkill";
 *           chip_en; power_en;
 *           wlan { compatible = "allwinner,sunxi-wlan"; wlan_busnum, wlan_regon,
 *                  wlan_hostwake, wlan_power, wlan_power_vol };
 *           bt   { compatible = "allwinner,sunxi-bt"; bt_power, bt_power_vol,
 *                  bt_rst_n };
 *   };
 *
 * Userspace surface reproduced from the vendor module:
 *   - /dev/sunxi-wlan misc device, with the "rf-ctrl" group containing
 *     power_state (read/write) and scan_device (write);
 *   - an rfkill device named "sunxi-bt" of type Bluetooth, whose block state
 *     drives the BT regulator and reset line.
 *
 * The gnss and modem subnodes of the vendor module are not implemented; this
 * board has neither.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/rfkill.h>
#include <linux/miscdevice.h>
#include <linux/sysfs.h>
#include <linux/capability.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/string.h>

/* provided by the sunxi MMC host driver when it is built in */
extern int sunxi_mmc_rescan_card(unsigned int busnum) __attribute__((weak));

struct sunxi_wlan {
	struct regulator *reg;
	struct gpio_desc *regon;
	struct gpio_desc *hostwake;
	u32 busnum;
	bool powered;
};

struct sunxi_bt {
	struct regulator *reg;
	struct gpio_desc *rst;
	struct rfkill *rfkill;
	bool powered;
};

struct sunxi_rfkill {
	struct device *dev;
	struct gpio_desc *chip_en;
	struct gpio_desc *power_en;
	struct sunxi_wlan wlan;
	struct sunxi_bt bt;
	struct miscdevice misc;
};

static int sunxi_wlan_set_power(struct sunxi_rfkill *priv, bool on)
{
	struct sunxi_wlan *wlan = &priv->wlan;
	int ret;

	if (wlan->powered == on)
		return 0;

	if (on) {
		if (wlan->reg) {
			ret = regulator_enable(wlan->reg);
			if (ret) {
				dev_err(priv->dev, "wlan regulator on failed: %d\n", ret);
				return ret;
			}
		}
		gpiod_set_value_cansleep(wlan->regon, 1);
		msleep(10);
	} else {
		gpiod_set_value_cansleep(wlan->regon, 0);
		if (wlan->reg)
			regulator_disable(wlan->reg);
	}

	wlan->powered = on;
	dev_info(priv->dev, "wlan power %s\n", on ? "on" : "off");

	return 0;
}

static int sunxi_bt_set_power(struct sunxi_rfkill *priv, bool on)
{
	struct sunxi_bt *bt = &priv->bt;
	int ret;

	if (bt->powered == on)
		return 0;

	if (on) {
		if (bt->reg) {
			ret = regulator_enable(bt->reg);
			if (ret) {
				dev_err(priv->dev, "bt regulator on failed: %d\n", ret);
				return ret;
			}
		}
		msleep(10);
		gpiod_set_value_cansleep(bt->rst, 1);
	} else {
		gpiod_set_value_cansleep(bt->rst, 0);
		if (bt->reg)
			regulator_disable(bt->reg);
	}

	bt->powered = on;
	dev_info(priv->dev, "bluetooth power %s\n", on ? "on" : "off");

	return 0;
}

static int sunxi_bt_set_block(void *data, bool blocked)
{
	struct sunxi_rfkill *priv = data;

	return sunxi_bt_set_power(priv, !blocked);
}

static const struct rfkill_ops sunxi_bt_rfkill_ops = {
	.set_block = sunxi_bt_set_block,
};

static ssize_t power_state_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct sunxi_rfkill *priv = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", priv->wlan.powered);
}

static ssize_t power_state_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct sunxi_rfkill *priv = dev_get_drvdata(dev);
	bool on;
	int ret;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;

	ret = sunxi_wlan_set_power(priv, on);
	if (ret)
		return ret;

	return count;
}

static ssize_t scan_device_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct sunxi_rfkill *priv = dev_get_drvdata(dev);
	unsigned int busnum;
	int ret;

	ret = kstrtouint(buf, 0, &busnum);
	if (ret)
		return ret;

	if (sunxi_mmc_rescan_card) {
		/* vendor semantics: the value is the bus number to rescan */
		ret = sunxi_mmc_rescan_card(busnum);
		if (ret)
			return ret;
	} else {
		dev_warn_once(priv->dev,
			      "scan_device: no MMC rescan hook, ignoring\n");
	}

	return count;
}

static DEVICE_ATTR_RW(power_state);
static DEVICE_ATTR_WO(scan_device);

static struct attribute *rf_ctrl_attrs[] = {
	&dev_attr_power_state.attr,
	&dev_attr_scan_device.attr,
	NULL,
};

static const struct attribute_group rf_ctrl_group = {
	.name = "rf-ctrl",
	.attrs = rf_ctrl_attrs,
};

static int sunxi_rfkill_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node, *child;
	struct sunxi_rfkill *priv;
	bool have_bt;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	platform_set_drvdata(pdev, priv);

	priv->chip_en = devm_gpiod_get_from_of_node(dev, np, "chip_en", 0,
						    GPIOD_OUT_HIGH, "chip_en");
	if (IS_ERR(priv->chip_en) && PTR_ERR(priv->chip_en) != -ENOENT)
		return dev_err_probe(dev, PTR_ERR(priv->chip_en), "chip_en gpio\n");

	priv->power_en = devm_gpiod_get_from_of_node(dev, np, "power_en", 0,
						     GPIOD_OUT_HIGH, "power_en");
	if (IS_ERR(priv->power_en) && PTR_ERR(priv->power_en) != -ENOENT)
		return dev_err_probe(dev, PTR_ERR(priv->power_en), "power_en gpio\n");

	/*
	 * Vendor sunxi_wlan_init()/sunxi_bt_init() only set up the subnode they
	 * find: of_find_matching_node_and_match() returning NULL skips the whole
	 * block and the function still returns 0 (dec_mod/sunxi_rfkill.c:2210
	 * and :1474).  So a board with only one of wlan/bt must still get its
	 * rfkill device and misc node, rather than failing probe with -ENODEV.
	 */
	priv->wlan.busnum = 1;
	child = of_get_compatible_child(np, "allwinner,sunxi-wlan");
	if (!child) {
		dev_warn(dev, "no wlan child node, wlan power control disabled\n");
	} else {
		of_property_read_u32(child, "wlan_busnum", &priv->wlan.busnum);

		priv->wlan.regon = devm_gpiod_get_from_of_node(dev, child,
							       "wlan_regon", 0,
							       GPIOD_OUT_LOW,
							       "wlan_regon");
		if (IS_ERR(priv->wlan.regon))
			return dev_err_probe(dev, PTR_ERR(priv->wlan.regon),
					     "wlan_regon gpio\n");

		priv->wlan.hostwake =
			devm_gpiod_get_from_of_node(dev, child, "wlan_hostwake",
						    0, GPIOD_IN, "wlan_hostwake");
		if (IS_ERR(priv->wlan.hostwake) &&
		    PTR_ERR(priv->wlan.hostwake) != -ENOENT)
			return dev_err_probe(dev, PTR_ERR(priv->wlan.hostwake),
					     "wlan_hostwake gpio\n");

		priv->wlan.reg = devm_regulator_get_optional(dev, "wlan_power");
		if (IS_ERR(priv->wlan.reg)) {
			if (PTR_ERR(priv->wlan.reg) != -ENODEV)
				return dev_err_probe(dev,
						     PTR_ERR(priv->wlan.reg),
						     "wlan_power regulator\n");
			priv->wlan.reg = NULL;
		}
		of_node_put(child);
	}

	child = of_get_compatible_child(np, "allwinner,sunxi-bt");
	have_bt = child != NULL;
	if (!have_bt) {
		dev_warn(dev, "no bt child node, bluetooth power control disabled\n");
	} else {
		priv->bt.reg = devm_regulator_get_optional(dev, "bt_power");
		if (IS_ERR(priv->bt.reg)) {
			if (PTR_ERR(priv->bt.reg) != -ENODEV)
				return dev_err_probe(dev, PTR_ERR(priv->bt.reg),
						     "bt_power regulator\n");
			priv->bt.reg = NULL;
		}

		priv->bt.rst = devm_gpiod_get_from_of_node(dev, child,
							   "bt_rst_n", 0,
							   GPIOD_OUT_LOW,
							   "bt_rst_n");
		if (IS_ERR(priv->bt.rst))
			return dev_err_probe(dev, PTR_ERR(priv->bt.rst),
					     "bt_rst_n gpio\n");
		of_node_put(child);
	}

	priv->misc.minor = MISC_DYNAMIC_MINOR;
	priv->misc.name = "sunxi-wlan";
	priv->misc.parent = dev;
	priv->misc.mode = 0600;

	ret = misc_register(&priv->misc);
	if (ret)
		return dev_err_probe(dev, ret, "misc_register failed\n");

	dev_set_drvdata(priv->misc.this_device, priv);

	ret = sysfs_create_group(&priv->misc.this_device->kobj, &rf_ctrl_group);
	if (ret) {
		dev_err(dev, "cannot create rf-ctrl attributes: %d\n", ret);
		goto err_misc;
	}

	/* The vendor only creates the Bluetooth rfkill node from sunxi_bt_init. */
	if (have_bt) {
		priv->bt.rfkill = rfkill_alloc("sunxi-bt", dev,
					       RFKILL_TYPE_BLUETOOTH,
					       &sunxi_bt_rfkill_ops, priv);
		if (!priv->bt.rfkill) {
			ret = -ENOMEM;
			goto err_group;
		}

		rfkill_set_states(priv->bt.rfkill, true, false);

		ret = rfkill_register(priv->bt.rfkill);
		if (ret) {
			dev_err(dev, "rfkill_register failed: %d\n", ret);
			goto err_rfkill;
		}
	}

	dev_info(dev, "sunxi rfkill ready: wlan bus %u (%s), bt rst %s\n",
		 priv->wlan.busnum, priv->wlan.regon ? "gpio" : "absent",
		 priv->bt.rst ? "gpio" : "absent");

	return 0;

err_rfkill:
	rfkill_destroy(priv->bt.rfkill);
err_group:
	sysfs_remove_group(&priv->misc.this_device->kobj, &rf_ctrl_group);
err_misc:
	misc_deregister(&priv->misc);

	return ret;
}

static int sunxi_rfkill_remove(struct platform_device *pdev)
{
	struct sunxi_rfkill *priv = platform_get_drvdata(pdev);

	rfkill_unregister(priv->bt.rfkill);
	rfkill_destroy(priv->bt.rfkill);
	sysfs_remove_group(&priv->misc.this_device->kobj, &rf_ctrl_group);
	misc_deregister(&priv->misc);

	return 0;
}

static const struct of_device_id sunxi_rfkill_of_match[] = {
	{ .compatible = "allwinner,sunxi-rfkill" },
	{ }
};
MODULE_DEVICE_TABLE(of, sunxi_rfkill_of_match);

static struct platform_driver sunxi_rfkill_driver = {
	.probe = sunxi_rfkill_probe,
	.remove = sunxi_rfkill_remove,
	.driver = {
		.name = "sunxi-rfkill",
		.of_match_table = sunxi_rfkill_of_match,
	},
};
module_platform_driver(sunxi_rfkill_driver);

MODULE_DESCRIPTION("Allwinner sun50iw15 wlan/bt power and rfkill control");
MODULE_LICENSE("GPL");
