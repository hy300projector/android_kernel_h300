// SPDX-License-Identifier: GPL-2.0-only
/*
 * sunxi-direct-gpio - userspace control of a fixed list of GPIOs.
 *
 * Rewritten from the vendor sunxi-direct-gpio.ko.  The vendor driver binds
 * "allwinner,sunxi-direct-gpio", takes the pin list from DT ("pin-names",
 * "gpio-pins", "init-status"), exports every pin as a device of the "gpio_sw"
 * class and gives each device five attributes:
 *
 *   cfg    0 = input, 1 = output driven low
 *   pull   0 = bias disable, 1 = pull-up, 2 = pull-down
 *   drv    drive strength, 0..3
 *   data   raw line level
 *   light  logical level, inverted when the pin is active low
 *
 * The vendor also has normal_led/standby_led handling that just requests two
 * extra pins by name; those are ordinary entries in the same list here.
 */
#include <linux/device.h>
#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sysfs.h>

struct sunxi_dirgpio_pin {
	const char *name;
	int gpio;
	unsigned int cfg;
	unsigned int pull;
	unsigned int drv;
	bool active_low;
};

struct sunxi_dirgpio {
	struct device *dev;
	struct sunxi_dirgpio_pin *pins;
	unsigned int num_pins;
	struct class *cls;
	struct device **pdevs;
};

static const unsigned int pull_config[] = {
	PIN_CONFIG_BIAS_DISABLE,
	PIN_CONFIG_BIAS_PULL_UP,
	PIN_CONFIG_BIAS_PULL_DOWN,
};

static ssize_t cfg_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct sunxi_dirgpio_pin *pin = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", pin->cfg);
}

static ssize_t cfg_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct sunxi_dirgpio_pin *pin = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 1)
		return -EINVAL;

	if (val)
		ret = gpiod_direction_output_raw(gpio_to_desc(pin->gpio), 0);
	else
		ret = gpiod_direction_input(gpio_to_desc(pin->gpio));
	if (ret)
		return ret;

	pin->cfg = val;
	return count;
}
static DEVICE_ATTR_RW(cfg);

static ssize_t pull_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct sunxi_dirgpio_pin *pin = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", pin->pull);
}

static ssize_t pull_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct sunxi_dirgpio_pin *pin = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val >= ARRAY_SIZE(pull_config))
		return -EINVAL;

	ret = pinctrl_gpio_set_config(pin->gpio,
				      pull_config[val] | (val << 8));
	if (ret)
		return ret;

	pin->pull = val;
	return count;
}
static DEVICE_ATTR_RW(pull);

static ssize_t drv_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct sunxi_dirgpio_pin *pin = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", pin->drv);
}

static ssize_t drv_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	struct sunxi_dirgpio_pin *pin = dev_get_drvdata(dev);
	unsigned int val;
	int ret;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;
	if (val > 3)
		return -EINVAL;

	ret = pinctrl_gpio_set_config(pin->gpio,
				      PIN_CONFIG_DRIVE_STRENGTH | (val << 8));
	if (ret)
		return ret;

	pin->drv = val;
	return count;
}
static DEVICE_ATTR_RW(drv);

static ssize_t data_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct sunxi_dirgpio_pin *pin = dev_get_drvdata(dev);

	return sprintf(buf, "%u\n", gpiod_get_raw_value(gpio_to_desc(pin->gpio)));
}

static ssize_t data_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct sunxi_dirgpio_pin *pin = dev_get_drvdata(dev);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	gpiod_set_raw_value(gpio_to_desc(pin->gpio), val);
	return count;
}
static DEVICE_ATTR_RW(data);

/* "light" is the logical level: on means "driven to the active state". */
static ssize_t light_show(struct device *dev, struct device_attribute *attr,
			  char *buf)
{
	struct sunxi_dirgpio_pin *pin = dev_get_drvdata(dev);
	int raw = gpiod_get_raw_value(gpio_to_desc(pin->gpio));

	return sprintf(buf, "%u\n", raw == pin->active_low);
}

static ssize_t light_store(struct device *dev, struct device_attribute *attr,
			   const char *buf, size_t count)
{
	struct sunxi_dirgpio_pin *pin = dev_get_drvdata(dev);
	unsigned int val;

	if (kstrtouint(buf, 10, &val))
		return -EINVAL;

	gpiod_set_raw_value(gpio_to_desc(pin->gpio),
			    val ? pin->active_low : !pin->active_low);
	return count;
}
static DEVICE_ATTR_RW(light);

static struct attribute *sunxi_dirgpio_attrs[] = {
	&dev_attr_cfg.attr,
	&dev_attr_pull.attr,
	&dev_attr_drv.attr,
	&dev_attr_data.attr,
	&dev_attr_light.attr,
	NULL,
};
ATTRIBUTE_GROUPS(sunxi_dirgpio);

static int sunxi_dirgpio_hw_init(struct sunxi_dirgpio *chip)
{
	struct device *dev = chip->dev;
	struct device_node *np = dev->of_node;
	unsigned int i;
	int ret;

	for (i = 0; i < chip->num_pins; i++) {
		unsigned long flags = 0;
		u32 init_status = 0;
		int gpio;

		gpio = of_get_named_gpio_flags(np, "gpio-pins", i,
					       (enum of_gpio_flags *)&flags);
		if (gpio < 0) {
			dev_err(dev, "get gpio error!\n");
			return gpio;
		}

		ret = devm_gpio_request(dev, gpio, dev_name(dev));
		if (ret) {
			dev_err(dev, "gpio_pin_%u(%d) gpio_request failed\n",
				i + 1, gpio);
			return ret;
		}

		ret = of_property_read_u32_index(np, "init-status", i,
						 &init_status);
		if (ret) {
			dev_err(dev, "dts property 'init-status' is not assigned\n");
			return ret;
		}

		/* The vendor drives the inverted level of init-status. */
		ret = gpiod_direction_output_raw(gpio_to_desc(gpio),
						 init_status == 0);
		if (ret) {
			dev_err(dev, "failed to set gpio as output\n");
			return ret;
		}

		{
			const char *pin_name = NULL;

			of_property_read_string_index(np, "pin-names", i,
						      &pin_name);
			chip->pins[i].name = devm_kstrdup(dev, pin_name ? : "gpio",
							 GFP_KERNEL);
			if (!chip->pins[i].name)
				return -ENOMEM;
		}

		chip->pins[i].gpio = gpio;
		chip->pins[i].active_low = !!(flags & OF_GPIO_ACTIVE_LOW);
		chip->pins[i].cfg = 1;
	}

	return 0;
}

static int sunxi_dirgpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct sunxi_dirgpio *chip;
	int count, ret, i;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->dev = dev;

	count = of_property_count_strings(dev->of_node, "pin-names");
	if (count <= 0) {
		dev_err(dev, "get gpio_names failed\n");
		return count ? count : -EINVAL;
	}

	chip->pins = devm_kcalloc(dev, count, sizeof(*chip->pins),
				  GFP_KERNEL);
	chip->pdevs = devm_kcalloc(dev, count, sizeof(*chip->pdevs),
				   GFP_KERNEL);
	if (!chip->pins || !chip->pdevs)
		return -ENOMEM;
	chip->num_pins = count;

	ret = sunxi_dirgpio_hw_init(chip);
	if (ret)
		return ret;

	chip->cls = class_create(THIS_MODULE, "gpio_sw");
	if (IS_ERR(chip->cls)) {
		dev_err(dev, "sunxi_dirgpio class_create failed\n");
		return PTR_ERR(chip->cls);
	}

	for (i = 0; i < chip->num_pins; i++) {
		struct device *pdev_dev;

		pdev_dev = device_create_with_groups(chip->cls, dev, 0,
						     &chip->pins[i],
						     sunxi_dirgpio_groups,
						     "%s", chip->pins[i].name);
		if (IS_ERR(pdev_dev)) {
			dev_err(dev, "Failed to create sysfs file %d\n", i);
			ret = PTR_ERR(pdev_dev);
			goto err_unregister;
		}
		chip->pdevs[i] = pdev_dev;
	}

	dev_info(dev, "%u direct GPIOs exported as gpio_sw devices\n",
		 chip->num_pins);
	return 0;

err_unregister:
	while (i--)
		device_unregister(chip->pdevs[i]);
	class_destroy(chip->cls);
	return ret;
}

static int sunxi_dirgpio_remove(struct platform_device *pdev)
{
	struct sunxi_dirgpio *chip = platform_get_drvdata(pdev);
	unsigned int i;

	for (i = 0; i < chip->num_pins; i++)
		if (chip->pdevs[i])
			device_unregister(chip->pdevs[i]);
	class_destroy(chip->cls);

	dev_info(chip->dev, "sunxi direct gpio remove success\n");
	return 0;
}

static const struct of_device_id sunxi_dirgpio_match[] = {
	{ .compatible = "allwinner,sunxi-direct-gpio" },
	{ }
};
MODULE_DEVICE_TABLE(of, sunxi_dirgpio_match);

static struct platform_driver sunxi_dirgpio_driver = {
	.probe	= sunxi_dirgpio_probe,
	.remove	= sunxi_dirgpio_remove,
	.driver	= {
		.name		= "sunxi_direct_gpio",
		.of_match_table	= sunxi_dirgpio_match,
	},
};
module_platform_driver(sunxi_dirgpio_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("sunxi direct gpio driver");
MODULE_AUTHOR("shaosidi <shaosidi@allwinnertech.com>");
