// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allwinner sun50iw15 network address manager.
 *
 * Replacement for the vendor sunxi_addr module: it owns one MAC address per
 * interface (wifi, bt, eth), hands them to the network stacks and exposes them
 * through the "sunxi-addr-mgt" class.
 *
 * Rewritten from the decompiled vendor module
 * (projector/vendor_boot/ramdisk/lib/modules/sunxi_addr.ko).  The vendor's type
 * switch is reproduced:
 *
 *   1 - the "addr_<name>" string property of the device tree node
 *   2 - derived from the SoC chipid, read from the SID "chipid" cell
 *   4 - random, regenerated on every boot
 *
 * The vendor derives type 2 with HMAC-SHA256 over the chipid.  This version
 * mixes the same chipid into six bytes with crc32 instead, so addresses are
 * stable across boots but are not bit-identical to the stock ones; the
 * locally-administered bit is set either way.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/random.h>
#include <linux/crc32.h>
#include <linux/string.h>
#include <linux/io.h>

#define SUNXI_ADDR_ENTRIES	3
#define SUNXI_ADDR_LEN		6

/* SID layout on this SoC: "chipid" cell at offset 0x200, 16 bytes */
#define SUNXI_SID_CHIPID_OFF	0x200
#define SUNXI_SID_CHIPID_LEN	16

enum sunxi_addr_type {
	SUNXI_ADDR_TYPE_DT	= 1,
	SUNXI_ADDR_TYPE_CHIPID	= 2,
	SUNXI_ADDR_TYPE_RANDOM	= 4,
};

struct sunxi_addr_entry {
	const char *name;
	u8 addr[SUNXI_ADDR_LEN];
	u32 type;		/* type requested by the device tree */
	u32 cur_type;		/* type the address actually came from */
};

struct sunxi_addr {
	struct device *dev;
	struct class *class;
	struct device *class_dev;
	struct mutex lock;
	struct sunxi_addr_entry entries[SUNXI_ADDR_ENTRIES];
};

static struct sunxi_addr *sunxi_addr_priv;

static bool sunxi_addr_is_valid(const u8 *addr)
{
	static const u8 zero[SUNXI_ADDR_LEN];
	static const u8 bcast[SUNXI_ADDR_LEN] = {
		0xff, 0xff, 0xff, 0xff, 0xff, 0xff
	};

	return memcmp(addr, zero, SUNXI_ADDR_LEN) &&
	       memcmp(addr, bcast, SUNXI_ADDR_LEN);
}

static void sunxi_addr_derive(const u8 *chipid, size_t len,
			      const char *name, u8 *out)
{
	u32 crc = crc32_le(~0, chipid, len);
	u32 mix = crc32_le(crc, name, strlen(name));
	unsigned int i;

	for (i = 0; i < SUNXI_ADDR_LEN; i++) {
		crc = crc32_le(crc, (const u8 *)&mix, sizeof(mix));
		out[i] = crc >> 24;
	}

	out[0] &= 0xfc;		/* unicast, local bit handled below */
	out[0] |= 0x02;		/* locally administered */
}

static int sunxi_addr_read_chipid(u8 *out)
{
	struct device_node *np;
	void __iomem *base;
	unsigned int i;

	np = of_find_compatible_node(NULL, NULL, "allwinner,sunxi-sid");
	if (!np)
		return -ENODEV;

	base = of_iomap(np, 0);
	of_node_put(np);
	if (!base)
		return -ENOMEM;

	for (i = 0; i < SUNXI_SID_CHIPID_LEN / sizeof(u32); i++)
		((u32 *)out)[i] = readl(base + SUNXI_SID_CHIPID_OFF + i * 4);

	iounmap(base);

	return 0;
}

static int sunxi_addr_from_dt(struct device *dev, const char *name, u8 *out)
{
	char prop[32];
	const char *str;
	unsigned int a[SUNXI_ADDR_LEN];
	int ret, i;

	snprintf(prop, sizeof(prop), "addr_%s", name);

	ret = of_property_read_string(dev->of_node, prop, &str);
	if (ret)
		return ret;

	ret = sscanf(str, "%x:%x:%x:%x:%x:%x",
		     &a[0], &a[1], &a[2], &a[3], &a[4], &a[5]);
	if (ret != SUNXI_ADDR_LEN)
		return -EINVAL;

	for (i = 0; i < SUNXI_ADDR_LEN; i++) {
		if (a[i] > 0xff)
			return -ERANGE;
		out[i] = a[i];
	}

	return 0;
}

static int sunxi_addr_from_chipid(struct device *dev, const char *name, u8 *out)
{
	u8 chipid[SUNXI_SID_CHIPID_LEN];
	int ret;

	ret = sunxi_addr_read_chipid(chipid);
	if (ret)
		return ret;

	sunxi_addr_derive(chipid, sizeof(chipid), name, out);

	return 0;
}

static void sunxi_addr_fill(struct sunxi_addr *priv, struct sunxi_addr_entry *e)
{
	int ret;

	switch (e->type) {
	case SUNXI_ADDR_TYPE_DT:
		ret = sunxi_addr_from_dt(priv->dev, e->name, e->addr);
		break;
	case SUNXI_ADDR_TYPE_CHIPID:
		ret = sunxi_addr_from_chipid(priv->dev, e->name, e->addr);
		break;
	case SUNXI_ADDR_TYPE_RANDOM:
		ret = -EINVAL;
		break;
	default:
		/*
		 * The stock tree sets type_addr_* to 0, which is not one of the
		 * vendor's explicit cases.  Treat it as "use the default source"
		 * instead of randomising, so the address stays stable across
		 * boots; random is only reached if the chipid cannot be read.
		 */
		if (e->type)
			dev_warn(priv->dev, "addr %s: unknown type %u, using chipid\n",
				 e->name, e->type);
		ret = sunxi_addr_from_chipid(priv->dev, e->name, e->addr);
		break;
	}

	if (!ret && sunxi_addr_is_valid(e->addr)) {
		e->cur_type = e->type;
		return;
	}

	dev_warn(priv->dev,
		 "addr %s: type %u unavailable (%d), using a random address\n",
		 e->name, e->type, ret);

	get_random_bytes(e->addr, SUNXI_ADDR_LEN);
	e->addr[0] &= 0xfc;
	e->addr[0] |= 0x02;
	e->cur_type = SUNXI_ADDR_TYPE_RANDOM;
}

static ssize_t sunxi_addr_show(struct device *dev, unsigned int idx, char *buf)
{
	struct sunxi_addr *priv = dev_get_drvdata(dev);
	const u8 *a = priv->entries[idx].addr;

	return sysfs_emit(buf, "%02x:%02x:%02x:%02x:%02x:%02x\n",
			  a[0], a[1], a[2], a[3], a[4], a[5]);
}

static ssize_t sunxi_addr_store(struct device *dev, unsigned int idx,
				const char *buf, size_t count)
{
	struct sunxi_addr *priv = dev_get_drvdata(dev);
	unsigned int a[SUNXI_ADDR_LEN];
	int ret, i;

	ret = sscanf(buf, "%x:%x:%x:%x:%x:%x",
		     &a[0], &a[1], &a[2], &a[3], &a[4], &a[5]);
	if (ret != SUNXI_ADDR_LEN)
		return -EINVAL;

	for (i = 0; i < SUNXI_ADDR_LEN; i++)
		if (a[i] > 0xff)
			return -ERANGE;

	mutex_lock(&priv->lock);
	for (i = 0; i < SUNXI_ADDR_LEN; i++)
		priv->entries[idx].addr[i] = a[i];
	mutex_unlock(&priv->lock);

	return count;
}

#define SUNXI_ADDR_ATTR(idx, nm)					\
static ssize_t nm##_show(struct device *dev,				\
			 struct device_attribute *attr, char *buf)	\
{									\
	return sunxi_addr_show(dev, idx, buf);				\
}									\
static ssize_t nm##_store(struct device *dev,				\
			  struct device_attribute *attr,		\
			  const char *buf, size_t count)		\
{									\
	return sunxi_addr_store(dev, idx, buf, count);			\
}									\
static DEVICE_ATTR_RW(nm)

SUNXI_ADDR_ATTR(0, addr_wifi);
SUNXI_ADDR_ATTR(1, addr_bt);
SUNXI_ADDR_ATTR(2, addr_eth);

static ssize_t summary_show(struct device *dev, struct device_attribute *attr,
			    char *buf)
{
	struct sunxi_addr *priv = dev_get_drvdata(dev);
	ssize_t n = 0;
	unsigned int i;

	n += sysfs_emit_at(buf, n, "name cfg cur address\n");

	for (i = 0; i < SUNXI_ADDR_ENTRIES; i++) {
		const struct sunxi_addr_entry *e = &priv->entries[i];

		n += sysfs_emit_at(buf, n,
				   "%4s  %u   %u  %02X:%02X:%02X:%02X:%02X:%02X\n",
				   e->name, e->type, e->cur_type,
				   e->addr[0], e->addr[1], e->addr[2],
				   e->addr[3], e->addr[4], e->addr[5]);
	}

	return n;
}
static DEVICE_ATTR_RO(summary);

/**
 * sun50iw15_get_mac() - fetch the address of one interface
 * @name: "wifi", "bt" or "eth"
 * @out: six bytes to fill
 *
 * Returns 0 on success, -ENODEV before probe and -ENOENT for unknown names.
 */
int sun50iw15_get_mac(const char *name, u8 *out)
{
	struct sunxi_addr *priv = sunxi_addr_priv;
	unsigned int i;

	if (!priv || !name || !out)
		return -ENODEV;

	for (i = 0; i < SUNXI_ADDR_ENTRIES; i++) {
		if (!strcmp(priv->entries[i].name, name)) {
			mutex_lock(&priv->lock);
			memcpy(out, priv->entries[i].addr, SUNXI_ADDR_LEN);
			mutex_unlock(&priv->lock);
			return 0;
		}
	}

	return -ENOENT;
}
EXPORT_SYMBOL_GPL(sun50iw15_get_mac);

static const char *const sunxi_addr_names[SUNXI_ADDR_ENTRIES] = {
	"wifi", "bt", "eth",
};

static const u32 sunxi_addr_default_type = SUNXI_ADDR_TYPE_CHIPID;

static int sunxi_addr_probe(struct platform_device *pdev)
{
	struct sunxi_addr *priv;
	unsigned int i;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;
	mutex_init(&priv->lock);

	for (i = 0; i < SUNXI_ADDR_ENTRIES; i++) {
		char prop[32];
		u32 type;

		priv->entries[i].name = sunxi_addr_names[i];

		snprintf(prop, sizeof(prop), "type_addr_%s", sunxi_addr_names[i]);
		if (of_property_read_u32(pdev->dev.of_node, prop, &type)) {
			snprintf(prop, sizeof(prop), "type_def_%s",
				 sunxi_addr_names[i]);
			if (of_property_read_u32(pdev->dev.of_node, prop, &type))
				type = sunxi_addr_default_type;
		}

		priv->entries[i].type = type;
		sunxi_addr_fill(priv, &priv->entries[i]);
	}

	priv->class = class_create(THIS_MODULE, "sunxi-addr-mgt");
	if (IS_ERR(priv->class))
		return PTR_ERR(priv->class);

	priv->class_dev = device_create(priv->class, NULL, 0, priv, "addr_mgt");
	if (IS_ERR(priv->class_dev)) {
		ret = PTR_ERR(priv->class_dev);
		goto err_class;
	}

	ret = device_create_file(priv->class_dev, &dev_attr_addr_wifi);
	if (ret)
		goto err_device;

	ret = device_create_file(priv->class_dev, &dev_attr_addr_bt);
	if (ret)
		goto err_wifi;

	ret = device_create_file(priv->class_dev, &dev_attr_addr_eth);
	if (ret)
		goto err_bt;

	ret = device_create_file(priv->class_dev, &dev_attr_summary);
	if (ret)
		goto err_eth;

	sunxi_addr_priv = priv;

	for (i = 0; i < SUNXI_ADDR_ENTRIES; i++)
		dev_info(&pdev->dev, "%s: %02x:%02x:%02x:%02x:%02x:%02x (type %u)\n",
			 priv->entries[i].name,
			 priv->entries[i].addr[0], priv->entries[i].addr[1],
			 priv->entries[i].addr[2], priv->entries[i].addr[3],
			 priv->entries[i].addr[4], priv->entries[i].addr[5],
			 priv->entries[i].cur_type);

	return 0;

err_eth:
	device_remove_file(priv->class_dev, &dev_attr_summary);
err_bt:
	device_remove_file(priv->class_dev, &dev_attr_addr_eth);
err_wifi:
	device_remove_file(priv->class_dev, &dev_attr_addr_bt);
err_device:
	device_remove_file(priv->class_dev, &dev_attr_addr_wifi);
	device_destroy(priv->class, 0);
err_class:
	class_destroy(priv->class);

	return ret;
}

static int sunxi_addr_remove(struct platform_device *pdev)
{
	struct sunxi_addr *priv = sunxi_addr_priv;

	if (!priv)
		return 0;

	sunxi_addr_priv = NULL;

	device_remove_file(priv->class_dev, &dev_attr_summary);
	device_remove_file(priv->class_dev, &dev_attr_addr_eth);
	device_remove_file(priv->class_dev, &dev_attr_addr_bt);
	device_remove_file(priv->class_dev, &dev_attr_addr_wifi);
	device_destroy(priv->class, 0);
	class_destroy(priv->class);

	return 0;
}

static const struct of_device_id sunxi_addr_of_match[] = {
	{ .compatible = "allwinner,sunxi-addr_mgt" },
	{ }
};
MODULE_DEVICE_TABLE(of, sunxi_addr_of_match);

static struct platform_driver sunxi_addr_driver = {
	.probe	= sunxi_addr_probe,
	.remove	= sunxi_addr_remove,
	.driver	= {
		.name		= "sunxi-addr-mgt",
		.of_match_table	= sunxi_addr_of_match,
	},
};
module_platform_driver(sunxi_addr_driver);

MODULE_DESCRIPTION("Allwinner sun50iw15 network address manager");
MODULE_LICENSE("GPL");
