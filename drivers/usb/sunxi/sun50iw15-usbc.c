// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allwinner sun50iw15 (H726 / HY300) USB OTG manager.
 *
 * The vendor module (sunxi_usbc) owned the whole USB stack: it parsed the
 * usbc0 node, decided between host and device, and then poked the HCI and UDC
 * modules through sunxi_usb_enable_hcd()/sunxi_usb_enable_udc().  A rewritten
 * driver cannot do that, so this one exports the same decision through the
 * standard frameworks instead:
 *
 *   - an extcon device with EXTCON_USB / EXTCON_USB_HOST, always registered,
 *     which the UDC and HCI drivers consume to know when to attach;
 *   - a usb_role_switch when the port is an OTG port using
 *     usb_detect_type = 3 (role switch), as vendor kernels do when the role
 *     is handed to a Type-C controller;
 *   - the otg_role sysfs attribute with the vendor's string interface.
 *
 * The HY300 describes usbc0 as usb_port_type = 0 (device only) with
 * usb_detect_type = 1 and empty usb_id_gpio / usb_det_vbus_gpio properties, so
 * the device path is the one that has to work with no GPIO present at all.
 */

#include <linux/delay.h>
#include <linux/extcon-provider.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/usb/role.h>
#include <linux/workqueue.h>

/* Values of the vendor usb_port_type property. */
enum sunxi_usbc_port_type {
	SUNXI_USBC_PORT_DEVICE = 0,
	SUNXI_USBC_PORT_HOST = 1,
	SUNXI_USBC_PORT_OTG = 2,
};

/* Values of the vendor usb_detect_type property. */
enum sunxi_usbc_detect_type {
	SUNXI_USBC_DETECT_NONE = 0,
	SUNXI_USBC_DETECT_GPIO = 1,
	SUNXI_USBC_DETECT_PMU = 2,
	SUNXI_USBC_DETECT_ROLE = 3,
};

/* Values of the vendor usb_detect_mode property. */
enum sunxi_usbc_detect_mode {
	SUNXI_USBC_DETECT_MODE_THREAD = 0,
	SUNXI_USBC_DETECT_MODE_IRQ = 1,
	SUNXI_USBC_DETECT_MODE_WORK = 2,
};

#define SUNXI_USBC_POLL_MS	100

struct sunxi_usbc {
	struct device *dev;
	struct extcon_dev *extcon;
	struct usb_role_switch *role_sw;
	struct regulator *usbc_supply;
	struct regulator *detvbus_supply;
	struct gpio_desc *id_gpio;
	struct gpio_desc *vbus_gpio;
	struct mutex lock;		/* protects role and the GPIO state */
	struct delayed_work poll_work;
	enum usb_role role;
	int port_type;
	int detect_type;
	int detect_mode;
	int vbus_irq;
	bool wakeup_suspend;
	bool usbc_enabled;
	bool detvbus_enabled;
};

static const unsigned int sunxi_usbc_cables[] = {
	EXTCON_USB,
	EXTCON_USB_HOST,
	EXTCON_NONE,
};

static const char *const sunxi_usbc_role_names[] = {
	[USB_ROLE_NONE] = "null",
	[USB_ROLE_HOST] = "usb_host",
	[USB_ROLE_DEVICE] = "usb_device",
};

/*
 * Read one optional GPIO.  The vendor binding points straight at a GPIO or,
 * for "axp_ctrl", holds a string instead; an empty property ("usb_id_gpio;")
 * is what the HY300 has and simply means "not wired".  Returns NULL for
 * absent/empty/unusable properties, ERR_PTR(-EPROBE_DEFER) to retry.
 */
static struct gpio_desc *sunxi_usbc_get_gpio(struct sunxi_usbc *sc,
					     const char *prop)
{
	struct device *dev = sc->dev;
	struct gpio_desc *desc;
	int gpio;

	if (!of_find_property(dev->of_node, prop, NULL))
		return NULL;

	gpio = of_get_named_gpio(dev->of_node, prop, 0);
	if (gpio == -EPROBE_DEFER)
		return ERR_PTR(-EPROBE_DEFER);
	if (gpio < 0) {
		dev_info(dev, "%s is not a GPIO, ignoring\n", prop);
		return NULL;
	}

	if (devm_gpio_request_one(dev, gpio, GPIOF_IN, prop)) {
		dev_warn(dev, "cannot request %s, ignoring\n", prop);
		return NULL;
	}

	desc = gpio_to_desc(gpio);
	if (!desc)
		return NULL;

	dev_info(dev, "%s = GPIO %d\n", prop, gpio);
	return desc;
}

static int sunxi_usbc_parse_dt(struct sunxi_usbc *sc)
{
	struct device_node *np = sc->dev->of_node;
	u32 val;

	sc->port_type = SUNXI_USBC_PORT_DEVICE;
	if (!of_property_read_u32(np, "usb_port_type", &val) &&
	    val <= SUNXI_USBC_PORT_OTG)
		sc->port_type = val;

	sc->detect_type = SUNXI_USBC_DETECT_NONE;
	if (!of_property_read_u32(np, "usb_detect_type", &val) &&
	    val <= SUNXI_USBC_DETECT_ROLE)
		sc->detect_type = val;

	sc->detect_mode = SUNXI_USBC_DETECT_MODE_THREAD;
	if (!of_property_read_u32(np, "usb_detect_mode", &val) &&
	    val <= SUNXI_USBC_DETECT_MODE_WORK)
		sc->detect_mode = val;

	if (!of_property_read_u32(np, "usb_wakeup_suspend", &val))
		sc->wakeup_suspend = val != 0;

	dev_info(sc->dev, "port_type=%d detect_type=%d detect_mode=%d\n",
		 sc->port_type, sc->detect_type, sc->detect_mode);

	return 0;
}

/*
 * Publish the role on extcon.  Caller holds sc->lock (or owns the device at
 * probe time).  This replaces the vendor hw_insmod/hw_rmmod_usb_* calls: the
 * UDC and HCI drivers attach themselves when their cable goes high.
 */
static void sunxi_usbc_apply_role(struct sunxi_usbc *sc, enum usb_role role)
{
	if (sc->role == role)
		return;

	sc->role = role;
	dev_info(sc->dev, "otg role %s\n", sunxi_usbc_role_names[role]);

	extcon_set_state_sync(sc->extcon, EXTCON_USB, role == USB_ROLE_DEVICE);
	extcon_set_state_sync(sc->extcon, EXTCON_USB_HOST, role == USB_ROLE_HOST);
}

/* Role requested by sysfs or a detect interrupt; goes through the switch if
 * one is registered so its consumers see the change too. */
static int sunxi_usbc_request_role(struct sunxi_usbc *sc, enum usb_role role)
{
	int ret;

	if (sc->role_sw)
		return usb_role_switch_set_role(sc->role_sw, role);

	mutex_lock(&sc->lock);
	sunxi_usbc_apply_role(sc, role);
	mutex_unlock(&sc->lock);

	ret = 0;
	return ret;
}

/* Role the hardware currently asks for. */
static enum usb_role sunxi_usbc_detect_role(struct sunxi_usbc *sc)
{
	bool id, vbus;

	if (sc->port_type == SUNXI_USBC_PORT_HOST)
		return USB_ROLE_HOST;
	if (sc->port_type == SUNXI_USBC_PORT_DEVICE)
		return USB_ROLE_DEVICE;

	/* OTG: ID low means host, ID high means device (vendor convention). */
	if (sc->id_gpio) {
		id = gpiod_get_value_cansleep(sc->id_gpio);
		return id ? USB_ROLE_DEVICE : USB_ROLE_HOST;
	}

	/* No ID pin: only a present VBUS can mean anything. */
	vbus = sc->vbus_gpio ? gpiod_get_value_cansleep(sc->vbus_gpio) : false;
	return vbus ? USB_ROLE_DEVICE : USB_ROLE_NONE;
}

static int sunxi_usbc_role_set(struct usb_role_switch *sw, enum usb_role role)
{
	struct sunxi_usbc *sc = usb_role_switch_get_drvdata(sw);

	if (role > USB_ROLE_DEVICE)
		return -EINVAL;

	mutex_lock(&sc->lock);
	sunxi_usbc_apply_role(sc, role);
	mutex_unlock(&sc->lock);

	return 0;
}

static enum usb_role sunxi_usbc_role_get(struct usb_role_switch *sw)
{
	struct sunxi_usbc *sc = usb_role_switch_get_drvdata(sw);

	return sc->role;
}

static irqreturn_t sunxi_usbc_detect_irq(int irq, void *data)
{
	struct sunxi_usbc *sc = data;

	sunxi_usbc_request_role(sc, sunxi_usbc_detect_role(sc));
	return IRQ_HANDLED;
}

static void sunxi_usbc_poll_work(struct work_struct *work)
{
	struct sunxi_usbc *sc = container_of(work, struct sunxi_usbc,
					     poll_work.work);

	sunxi_usbc_request_role(sc, sunxi_usbc_detect_role(sc));
	schedule_delayed_work(&sc->poll_work,
			      msecs_to_jiffies(SUNXI_USBC_POLL_MS));
}

static ssize_t otg_role_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct sunxi_usbc *sc = dev_get_drvdata(dev);
	enum usb_role role = sc->role;

	if (role > USB_ROLE_DEVICE)
		role = USB_ROLE_NONE;

	return sprintf(buf, "%s\n", sunxi_usbc_role_names[role]);
}

static ssize_t otg_role_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct sunxi_usbc *sc = dev_get_drvdata(dev);
	enum usb_role role;
	int val;

	if (sysfs_streq(buf, "usb_host")) {
		role = USB_ROLE_HOST;
	} else if (sysfs_streq(buf, "usb_device")) {
		role = USB_ROLE_DEVICE;
	} else if (sysfs_streq(buf, "null")) {
		role = USB_ROLE_NONE;
	} else if (!kstrtoint(buf, 0, &val) && val >= USB_ROLE_NONE &&
		   val <= USB_ROLE_DEVICE) {
		role = val;
	} else {
		dev_err(dev, "invalid otg role\n");
		return -EINVAL;
	}

	sunxi_usbc_request_role(sc, role);
	return count;
}
static DEVICE_ATTR_RW(otg_role);

static int sunxi_usbc_get_regulators(struct sunxi_usbc *sc)
{
	struct device *dev = sc->dev;
	int ret = 0;

	sc->usbc_supply = devm_regulator_get_optional(dev, "usbc");
	if (IS_ERR(sc->usbc_supply)) {
		ret = PTR_ERR(sc->usbc_supply);
		sc->usbc_supply = NULL;
		if (ret != -ENODEV)
			return dev_err_probe(dev, ret, "no usbc regulator\n");
	} else {
		ret = regulator_enable(sc->usbc_supply);
		if (ret)
			return dev_err_probe(dev, ret, "cannot enable usbc\n");
		sc->usbc_enabled = true;
	}

	sc->detvbus_supply = devm_regulator_get_optional(dev, "detvbus_io");
	if (IS_ERR(sc->detvbus_supply)) {
		ret = PTR_ERR(sc->detvbus_supply);
		sc->detvbus_supply = NULL;
		if (ret != -ENODEV)
			return dev_err_probe(dev, ret, "no detvbus_io\n");
	} else {
		ret = regulator_enable(sc->detvbus_supply);
		if (ret) {
			dev_warn(dev, "cannot enable detvbus_io: %d\n", ret);
			sc->detvbus_supply = NULL;
		} else {
			sc->detvbus_enabled = true;
		}
	}

	return 0;
}

static int sunxi_usbc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct usb_role_switch_desc desc = { };
	struct sunxi_usbc *sc;
	enum usb_role role;
	int ret;

	sc = devm_kzalloc(dev, sizeof(*sc), GFP_KERNEL);
	if (!sc)
		return -ENOMEM;

	sc->dev = dev;
	sc->role = USB_ROLE_NONE;
	mutex_init(&sc->lock);
	INIT_DELAYED_WORK(&sc->poll_work, sunxi_usbc_poll_work);
	platform_set_drvdata(pdev, sc);

	sunxi_usbc_parse_dt(sc);

	ret = sunxi_usbc_get_regulators(sc);
	if (ret)
		return ret;

	sc->id_gpio = sunxi_usbc_get_gpio(sc, "usb_id_gpio");
	if (IS_ERR(sc->id_gpio))
		return PTR_ERR(sc->id_gpio);

	sc->vbus_gpio = sunxi_usbc_get_gpio(sc, "usb_det_vbus_gpio");
	if (IS_ERR(sc->vbus_gpio))
		return PTR_ERR(sc->vbus_gpio);

	sc->extcon = devm_extcon_dev_allocate(dev, sunxi_usbc_cables);
	if (IS_ERR(sc->extcon))
		return dev_err_probe(dev, PTR_ERR(sc->extcon),
				     "cannot allocate extcon\n");

	ret = devm_extcon_dev_register(dev, sc->extcon);
	if (ret)
		return dev_err_probe(dev, ret, "cannot register extcon\n");

	if (sc->detect_type == SUNXI_USBC_DETECT_ROLE &&
	    device_property_present(dev, "usb-role-switch")) {
		desc.fwnode = dev_fwnode(dev);
		desc.set = sunxi_usbc_role_set;
		desc.get = sunxi_usbc_role_get;
		desc.driver_data = sc;
		desc.allow_userspace_control = true;
		sc->role_sw = usb_role_switch_register(dev, &desc);
		if (IS_ERR(sc->role_sw))
			return dev_err_probe(dev, PTR_ERR(sc->role_sw),
					     "cannot register role switch\n");
	}

	switch (sc->port_type) {
	case SUNXI_USBC_PORT_HOST:
		role = USB_ROLE_HOST;
		break;
	case SUNXI_USBC_PORT_DEVICE:
		role = USB_ROLE_DEVICE;
		break;
	default:
		role = sunxi_usbc_detect_role(sc);
		if (role == USB_ROLE_NONE)
			role = USB_ROLE_DEVICE;
		break;
	}
	sunxi_usbc_apply_role(sc, role);

	if (sc->port_type == SUNXI_USBC_PORT_OTG &&
	    (sc->id_gpio || sc->vbus_gpio)) {
		if (sc->detect_mode == SUNXI_USBC_DETECT_MODE_THREAD) {
			schedule_delayed_work(&sc->poll_work,
					      msecs_to_jiffies(SUNXI_USBC_POLL_MS));
		} else {
			sc->vbus_irq = sc->vbus_gpio ? gpiod_to_irq(sc->vbus_gpio) : -ENXIO;
			if (sc->vbus_irq < 0) {
				sc->vbus_irq = 0;
				schedule_delayed_work(&sc->poll_work,
						      msecs_to_jiffies(SUNXI_USBC_POLL_MS));
			} else {
				ret = devm_request_threaded_irq(dev, sc->vbus_irq,
								NULL,
								sunxi_usbc_detect_irq,
								IRQF_TRIGGER_RISING |
								IRQF_TRIGGER_FALLING |
								IRQF_ONESHOT,
								"sunxi-usbc-det", sc);
				if (ret) {
					dev_warn(dev, "no detect IRQ (%d), polling\n", ret);
					sc->vbus_irq = 0;
					schedule_delayed_work(&sc->poll_work,
							      msecs_to_jiffies(SUNXI_USBC_POLL_MS));
				}
			}
		}
	}

	ret = device_create_file(dev, &dev_attr_otg_role);
	if (ret)
		dev_warn(dev, "cannot create otg_role\n");

	if (sc->wakeup_suspend) {
		device_init_wakeup(dev, true);
		if (sc->vbus_irq)
			enable_irq_wake(sc->vbus_irq);
	}

	dev_info(dev, "sunxi usb otg manager ready, role %s\n",
		 sunxi_usbc_role_names[sc->role]);
	return 0;
}

static int sunxi_usbc_remove(struct platform_device *pdev)
{
	struct sunxi_usbc *sc = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&sc->poll_work);
	device_remove_file(sc->dev, &dev_attr_otg_role);

	if (sc->vbus_irq && sc->wakeup_suspend)
		disable_irq_wake(sc->vbus_irq);

	if (sc->role_sw)
		usb_role_switch_unregister(sc->role_sw);

	if (sc->detvbus_enabled)
		regulator_disable(sc->detvbus_supply);
	if (sc->usbc_enabled)
		regulator_disable(sc->usbc_supply);

	return 0;
}

static const struct of_device_id sunxi_usbc_of_match[] = {
	{ .compatible = "allwinner,sunxi-otg-manager" },
	{ }
};
MODULE_DEVICE_TABLE(of, sunxi_usbc_of_match);

static struct platform_driver sunxi_usbc_driver = {
	.probe = sunxi_usbc_probe,
	.remove = sunxi_usbc_remove,
	.driver = {
		.name = "sun50iw15-usbc",
		.of_match_table = sunxi_usbc_of_match,
	},
};
module_platform_driver(sunxi_usbc_driver);

MODULE_DESCRIPTION("Allwinner sun50iw15 USB OTG manager");
MODULE_LICENSE("GPL");
