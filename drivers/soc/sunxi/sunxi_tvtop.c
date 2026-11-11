// SPDX-License-Identifier: GPL-2.0
/*
 * sunxi_tvtop - Allwinner TV input top module (tvfe / tvcap)
 *
 * Reconstruction of the vendor module sunxi_tvtop.ko shipped for the
 * Allwinner H726 (sun50iw15p1) in kernel 5.15.167. The exported symbols and
 * the register sequences follow the vendor binary; all internal structures
 * are private to this module and are not part of the ABI.
 *
 * Two sub modules are managed here and reference counted independently:
 *   module 0 - tvfe, the TV front end (0x06000000, plus CVBS detection)
 *   module 1 - tvcap, the TV capture block (0x06e00000)
 *
 * Consumers call sunxi_tvtop_clk_get(module) before touching the block and
 * sunxi_tvtop_clk_put(module) when done. The first get powers the sub module
 * up, the last put powers it down.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/extcon-provider.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#define TVTOP_MAX_MODULES	2

/*
 * The vendor extcon provider numbers do not follow mainline: the vendor uses
 * 46 for "CVBS video in", while mainline EXTCON_JACK_VIDEO_IN is 24. The raw
 * id is part of the interface with prebuilt consumers, so keep it.
 */
#define TVTOP_EXTCON_CVBS_IN	46

/* tvfe CVBS detection register offsets (byte offsets into window 0) */
#define TVFE_CVBS_CTRL_REG	1280
#define TVFE_CVBS_CFG0_REG	1288
#define TVFE_CVBS_CFG1_REG	1296
#define TVFE_CVBS_CFG2_REG	1300
#define TVFE_CVBS_CFG3_REG	1304
#define TVFE_CVBS_CFG4_REG	1308
#define TVFE_CVBS_CFG5_REG	1312
#define TVFE_CVBS_CFG6_REG	1316
#define TVFE_CVBS_CFG7_REG	1320
#define TVFE_CVBS_EN_REG	1328
#define TVFE_CVBS_STATUS_REG	1332

/* tvcap capture control registers */
#define TVCAP_CTRL0_REG		0
#define TVCAP_CTRL1_REG		4
#define TVCAP_CTRL2_REG		8
#define TVCAP_MODE_SEL_REG	600

struct sunxi_tvtop;

struct tvtop_clk_node {
	struct list_head link;
	struct clk *clk;
	struct clk *parent;
	unsigned long rate;
	unsigned long parent_rate;
	bool is_enabled;
	bool is_parent_enabled;
};

/* One entry per clock the vendor table asks for. parent_rate stays 0 in both
 * tables of the vendor binary, but the power up path honours it, so keep it. */
struct tvtop_clk_info {
	const char *name;
	unsigned long rate;
	unsigned long parent_rate;
};

struct tvtop_hw_private_data {
	const char *name;
	int iomem_index;
	const char *reset_control_name;
	const char *bus_clk_name;
	const struct tvtop_clk_info *clocks;
};

struct tvtop_cvbs_data {
	struct delayed_work work;
	unsigned int detect_state[2];
	unsigned int detect_old_state[2];
};

struct tvtop_driver_data {
	const struct tvtop_hw_private_data *hw;
	void __iomem *base;
	struct reset_control *reset;
	struct clk *bus_clk;
	bool is_busclk_enabled;
	struct list_head clk_list;
	int cvbs_irq;
	struct workqueue_struct *cvbs_wq;
	struct tvtop_cvbs_data cvbs;
	spinlock_t cvbs_lock;
	struct platform_device *pmdev;
	int panel_type;
	struct mutex lock;
	bool enabled;
	bool initialized;
};

struct tvtop_ref_data {
	struct mutex lock;
	int ref_count;
	int (*enable_pfn)(struct device *dev);
};

struct sunxi_tvtop {
	dev_t devt;
	struct device *device;
	struct class *class;
	struct tvtop_driver_data driver[TVTOP_MAX_MODULES];
	struct tvtop_ref_data ref_data[TVTOP_MAX_MODULES];
	struct device *parent_dev;
};

static struct sunxi_tvtop *tvtopdev;
static struct extcon_dev *extcon_cvbs[TVTOP_MAX_MODULES];

/* exported interface, used by the sysfs stores below */
unsigned int *sunxi_tvtop_capclk_sel(int sel);
int sunxi_tvtop_clk_get(unsigned int module);
int sunxi_tvtop_clk_put(unsigned int module);
int sunxi_tvtop_set_adc_ctrl(unsigned int module, int offset, int value);

/* Demodulator resets, deasserted together with tvfe and asserted on power down.
 * The vendor keeps them in a writable static table that is filled in on the
 * first power up and released on the last power down. */
static const char * const demod_reset_names[] = {
	"reset_bus_tvfe_axi",
	"reset_bus_mcsi",
	"reset_bus_tvfe_ahb",
	"reset_bus_tvfe_mbu",
};

static struct reset_control *demod_reset[ARRAY_SIZE(demod_reset_names)];

static const struct tvtop_clk_info tvfe_clks[] = {
	{ .name = "tvfe-axi" },
	{ .name = "tsdm-ts" },
	{ .name = "mcsi" },
	{ .name = "tvfe1296m" },
	{ .name = "tcd3" },
	{ .name = "tvfe-axi-bus" },
	{ .name = "mcsi-bus" },
	{ .name = "tvfe-mbus" },
	{ .name = "tvfe-ahb-sw-cfg-bus" },
	{ .name = "tvfe-mbus-sw-cfg-bus" },
	{ }
};

static const struct tvtop_clk_info tvcap_clks[] = {
	{ .name = "adc", .rate = 27000000 },
	{ .name = "vincap-dma" },
	{ .name = "tvcap-ahb-sw-cfg-bus" },
	{ .name = "tvcap-mbus-sw-cfg-bus" },
	{ }
};

static const struct tvtop_hw_private_data hardware_datas[] = {
	{
		.name = "tvfe",
		.iomem_index = 0,
		.reset_control_name = "reset_bus_demod",
		.bus_clk_name = NULL,
		.clocks = tvfe_clks,
	},
	{
		.name = "tvcap",
		.iomem_index = 1,
		.reset_control_name = "reset_bus_tvcap",
		.bus_clk_name = "clk_bus_tvcap",
		.clocks = tvcap_clks,
	},
};

/* Vendor id 46, see TVTOP_EXTCON_CVBS_IN */
static const unsigned int tv_cable[] = {
	TVTOP_EXTCON_CVBS_IN,
	EXTCON_NONE,
};

static void tvtop_clock_enable(struct tvtop_driver_data *d)
{
	struct tvtop_clk_node *node;

	list_for_each_entry(node, &d->clk_list, link) {
		if (!node->clk || node->is_enabled)
			continue;

		if (clk_prepare_enable(node->clk)) {
			pr_err("sunxi_tvtop: %s enable clk failed\n", d->hw->name);
			continue;
		}
		node->is_enabled = true;

		if (node->parent_rate) {
			node->parent = clk_get_parent(node->clk);
			if (node->parent && !node->is_parent_enabled) {
				if (clk_prepare_enable(node->parent))
					pr_err("sunxi_tvtop: %s enable parent clk failed\n",
					       d->hw->name);
				else
					node->is_parent_enabled = true;
			}
		}

		if (node->rate)
			clk_set_rate(node->clk, node->rate);
	}
}

static void tvtop_clock_disable(struct tvtop_driver_data *d)
{
	struct tvtop_clk_node *node;

	list_for_each_entry(node, &d->clk_list, link) {
		if (node->clk && node->is_enabled) {
			clk_disable_unprepare(node->clk);
			node->is_enabled = false;
		}
		if (node->parent && node->is_parent_enabled) {
			clk_disable_unprepare(node->parent);
			node->is_parent_enabled = false;
		}
	}
}

static void tvtop_bus_clk_enable(struct tvtop_driver_data *d)
{
	if (!d->bus_clk || d->is_busclk_enabled)
		return;

	if (clk_prepare_enable(d->bus_clk))
		pr_err("sunxi_tvtop: %s enable bus clk failed\n", d->hw->name);
	else
		d->is_busclk_enabled = true;
}

static void tvtop_bus_clk_disable(struct tvtop_driver_data *d)
{
	if (!d->bus_clk || !d->is_busclk_enabled)
		return;

	clk_disable_unprepare(d->bus_clk);
	d->is_busclk_enabled = false;
}

static irqreturn_t cvbs_irq_handler(int irq, void *data)
{
	struct tvtop_driver_data *d = data;
	unsigned long flags;

	spin_lock_irqsave(&d->cvbs_lock, flags);
	/* Write back the latched status to clear the detection interrupt. The
	 * vendor performs a byte write here, not a word write. */
	writeb((u8)readl(d->base + TVFE_CVBS_STATUS_REG),
	       d->base + TVFE_CVBS_STATUS_REG);
	spin_unlock_irqrestore(&d->cvbs_lock, flags);

	/* The vendor passes 20 jiffies, which debounces the cable bounce. */
	queue_delayed_work(system_wq, &d->cvbs.work, 20);

	return IRQ_HANDLED;
}

static void cvbs_work_handler(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct tvtop_cvbs_data *cvbs = container_of(dwork, struct tvtop_cvbs_data,
						    work);
	unsigned int state0, state1;
	u32 status;

	if (!tvtopdev)
		return;

	status = readl(tvtopdev->driver[0].base + TVFE_CVBS_STATUS_REG);
	state0 = (status & 0x100) ? 1 : 0;
	state1 = (status & 0x200) ? 1 : 0;

	cvbs->detect_state[1] = state1;
	cvbs->detect_state[0] = state0;

	if (cvbs->detect_old_state[0] != state0) {
		cvbs->detect_old_state[0] = state0;
		if (extcon_cvbs[0])
			extcon_set_state_sync(extcon_cvbs[0], TVTOP_EXTCON_CVBS_IN,
					      state0);
		pr_info("sunxi_tvtop: cvbs0 %s\n", state0 ? "plugin" : "plugout");
	}

	if (cvbs->detect_old_state[1] != cvbs->detect_state[1]) {
		cvbs->detect_old_state[1] = cvbs->detect_state[1];
		if (extcon_cvbs[1])
			extcon_set_state_sync(extcon_cvbs[1], TVTOP_EXTCON_CVBS_IN,
					      state1);
		pr_info("sunxi_tvtop: cvbs1 %s\n", state1 ? "plugin" : "plugout");
	}
}

static void tvtop_demod_reset_release(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(demod_reset); i++) {
		if (!demod_reset[i])
			continue;
		reset_control_assert(demod_reset[i]);
		reset_control_put(demod_reset[i]);
		demod_reset[i] = NULL;
	}
}

static int tvtop_clock_probe(struct device *dev, struct device_node *np,
			     struct tvtop_driver_data *d,
			     const struct tvtop_clk_info *info)
{
	struct tvtop_clk_node *node;
	struct clk *clk;

	for (; info->name; info++) {
		clk = of_clk_get_by_name(np, info->name);
		if (IS_ERR(clk)) {
			dev_err(dev, "%s: failed to get clk '%s'!\n",
				d->hw->name, info->name);
			continue;
		}

		node = kzalloc(sizeof(*node), GFP_KERNEL);
		if (!node) {
			dev_err(dev, "%s: kzalloc clk node error!\n",
				d->hw->name);
			clk_put(clk);
			continue;
		}

		node->clk = clk;
		node->rate = info->rate;
		node->parent_rate = info->parent_rate;
		list_add_tail(&node->link, &d->clk_list);
	}

	return 0;
}

static int tvtop_tvfe_enable(struct device *dev)
{
	struct sunxi_tvtop *hw = dev_get_drvdata(dev);
	struct tvtop_driver_data *d = &hw->driver[0];
	struct reset_control *rst;
	int ret = 0;
	int i;

	mutex_lock(&d->lock);
	if (d->enabled)
		goto out;

	tvtop_clock_enable(d);
	tvtop_bus_clk_enable(d);
	usleep_range(10000, 15000);

	ret = reset_control_deassert(d->reset);
	if (ret) {
		dev_err(dev, "tvfe bus reset deassert fail: %d!\n", ret);
		goto out;
	}

	for (i = 0; i < ARRAY_SIZE(demod_reset); i++) {
		rst = of_reset_control_get(hw->parent_dev->of_node,
					   demod_reset_names[i]);
		if (IS_ERR(rst)) {
			dev_err(dev, "%s reset get fail: %ld!\n",
				demod_reset_names[i], PTR_ERR(rst));
			ret = PTR_ERR(rst);
			continue;
		}
		demod_reset[i] = rst;

		if (reset_control_status(rst) >= 1) {
			int err = reset_control_deassert(rst);

			if (err) {
				dev_err(dev, "%s reset deassert fail: %d!\n",
					demod_reset_names[i], err);
				ret = err;
			}
		}
	}

	usleep_range(10000, 15000);
	writel(0x7f1f, d->base);

	/*
	 * The CVBS detection block is only wired up when the extcon device
	 * exists; without it the front end runs without hotplug reporting.
	 */
	if (extcon_cvbs[0]) {
		ret = 0;
		writel(85, d->base + TVFE_CVBS_CFG1_REG);
		writel(113246320, d->base + TVFE_CVBS_CFG2_REG);
		writel(196609, d->base + TVFE_CVBS_CFG3_REG);
		writel(432000, d->base + TVFE_CVBS_CFG4_REG);
		writel(1, d->base + TVFE_CVBS_CFG5_REG);
		writel(2, d->base + TVFE_CVBS_CFG6_REG);
		writel(2, d->base + TVFE_CVBS_CFG7_REG);
		writel(3, d->base + TVFE_CVBS_EN_REG);
		writel(52432064, d->base + TVFE_CVBS_CTRL_REG);
		writel(2560, d->base + TVFE_CVBS_CFG0_REG);
		writel(119, d->base + TVFE_CVBS_CFG1_REG);

		ret = devm_request_threaded_irq(hw->parent_dev, d->cvbs_irq,
						cvbs_irq_handler, NULL, 0,
						"cvbs_dect_irq", d);
		if (ret) {
			pr_err("sunxi_tvtop: request cvbs irq %d failed: %d\n",
			       d->cvbs_irq, ret);
			if (d->cvbs_wq)
				destroy_workqueue(d->cvbs_wq);
			d->cvbs_wq = NULL;
		}
	}

	/* The vendor marks the block enabled even when the IRQ failed. */
	d->enabled = true;
out:
	mutex_unlock(&d->lock);
	return ret;
}

static int tvtop_tvcap_enable(struct device *dev)
{
	struct sunxi_tvtop *hw = dev_get_drvdata(dev);
	struct tvtop_driver_data *d = &hw->driver[1];
	int ret;

	mutex_lock(&d->lock);
	if (d->enabled) {
		ret = 0;
		goto out;
	}

	tvtop_clock_enable(d);
	usleep_range(10000, 15000);
	tvtop_bus_clk_enable(d);

	if (reset_control_status(d->reset) >= 1) {
		ret = reset_control_deassert(d->reset);
		if (ret) {
			dev_err(dev, "tvcap bus reset deassert fail: %d!\n", ret);
			goto out;
		}
	}

	usleep_range(10000, 15000);
	writel(0, d->base + TVCAP_CTRL0_REG);
	writel(0, d->base + TVCAP_CTRL1_REG);
	writel(0, d->base + TVCAP_CTRL2_REG);
	usleep_range(10000, 15000);
	writel(0x111111, d->base + TVCAP_CTRL0_REG);
	writel(0x1111115, d->base + TVCAP_CTRL1_REG);
	writel(0x404, d->base + TVCAP_CTRL2_REG);
	writel(readl(d->base + TVCAP_MODE_SEL_REG) | 0x10000,
	       d->base + TVCAP_MODE_SEL_REG);
	usleep_range(10000, 15000);

	d->enabled = true;
	ret = 0;
out:
	mutex_unlock(&d->lock);
	return ret;
}

static void tvtop_submodule_disable(struct device *dev, int module)
{
	struct sunxi_tvtop *hw = dev_get_drvdata(dev);
	struct tvtop_driver_data *d = &hw->driver[module];

	mutex_lock(&d->lock);
	if (!d->enabled)
		goto out;

	if (d->hw->iomem_index == 0) {
		devm_free_irq(hw->parent_dev, d->cvbs_irq, d);
		tvtop_demod_reset_release();
	}

	tvtop_clock_disable(d);
	reset_control_assert(d->reset);
	tvtop_bus_clk_disable(d);

	d->enabled = false;
	dev_info(dev, "submodule disable [%d]\n", module);
out:
	mutex_unlock(&d->lock);
}

static int tvtop_driver_data_init(struct device *dev, struct device_node *np,
				  struct sunxi_tvtop *hw, int index)
{
	struct tvtop_driver_data *d = &hw->driver[index];
	const struct tvtop_hw_private_data *data = &hardware_datas[index];

	d->hw = data;
	mutex_init(&d->lock);
	spin_lock_init(&d->cvbs_lock);
	INIT_LIST_HEAD(&d->clk_list);

	if (!np) {
		dev_err(dev, "invalid of_node!\n");
		return -EINVAL;
	}

	d->base = of_iomap(np, data->iomem_index);
	if (!d->base) {
		dev_err(dev, "%s: invalid io memory!\n", data->name);
		return -ENOMEM;
	}

	d->reset = of_reset_control_get(np, data->reset_control_name);
	if (IS_ERR(d->reset)) {
		dev_err(dev, "%s: Could not get reset control %s!\n",
			data->name, data->reset_control_name);
		iounmap(d->base);
		d->base = NULL;
		d->reset = NULL;
		return -EINVAL;
	}

	if (data->bus_clk_name) {
		d->bus_clk = of_clk_get_by_name(np, data->bus_clk_name);
		if (IS_ERR(d->bus_clk)) {
			dev_err(dev, "%s: failed to get bus clk '%s'!\n",
				data->name, data->bus_clk_name);
			d->bus_clk = NULL;
			reset_control_put(d->reset);
			d->reset = NULL;
			iounmap(d->base);
			d->base = NULL;
			return -EINVAL;
		}
	}

	tvtop_clock_probe(dev, np, d, data->clocks);

	d->cvbs_irq = platform_get_irq(to_platform_device(dev), 0);
	if (d->cvbs_irq < 0) {
		dev_err(dev, "%s: platform_get_irq error!\n", __func__);
		if (d->bus_clk)
			clk_put(d->bus_clk);
		d->bus_clk = NULL;
		reset_control_put(d->reset);
		d->reset = NULL;
		iounmap(d->base);
		d->base = NULL;
		return d->cvbs_irq;
	}

	mutex_init(&hw->ref_data[index].lock);
	hw->ref_data[index].ref_count = 0;
	hw->ref_data[index].enable_pfn = index == 0 ? tvtop_tvfe_enable :
						     tvtop_tvcap_enable;

	return 0;
}

static void tvtop_driver_data_exit(struct sunxi_tvtop *hw, int index)
{
	struct tvtop_driver_data *d = &hw->driver[index];
	struct tvtop_clk_node *node;
	struct tvtop_clk_node *tmp;

	iounmap(d->base);
	d->base = NULL;
	reset_control_put(d->reset);
	d->reset = NULL;

	if (d->bus_clk)
		clk_put(d->bus_clk);
	d->bus_clk = NULL;

	list_for_each_entry_safe(node, tmp, &d->clk_list, link) {
		list_del(&node->link);
		clk_put(node->clk);
		if (node->parent)
			clk_put(node->parent);
		kfree(node);
	}

	d->enabled = false;
	d->initialized = false;
}

static ssize_t sunxi_tvtop_store_tvfe(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	if (simple_strtoul(buf, NULL, 0))
		sunxi_tvtop_clk_get(0);
	else
		sunxi_tvtop_clk_put(0);

	return count;
}

static ssize_t sunxi_tvtop_store_tvcap(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	if (simple_strtoul(buf, NULL, 0))
		sunxi_tvtop_clk_get(1);
	else
		sunxi_tvtop_clk_put(1);

	return count;
}

static ssize_t sunxi_tvtop_store_cap_clk_sel(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t count)
{
	sunxi_tvtop_capclk_sel(simple_strtoul(buf, NULL, 0));

	return count;
}

static ssize_t sunxi_tvtop_store_adc_ctrl(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf, size_t count)
{
	unsigned int module = 0;
	unsigned int offset = 0;
	unsigned int value = 0;

	if (sscanf(buf, "%d %x %x", &module, &offset, &value) != 3)
		return -EINVAL;

	pr_info("sunxi_tvtop_store_adc_ctrl : %s, offset : 0x%x, val : 0x%x\n",
		module == 0 ? "tvfe" : "tvcap", offset, value);

	sunxi_tvtop_set_adc_ctrl(module, offset, value);

	return count;
}

static DEVICE_ATTR(tvfe, 0200, NULL, sunxi_tvtop_store_tvfe);
static DEVICE_ATTR(tvcap, 0200, NULL, sunxi_tvtop_store_tvcap);
static DEVICE_ATTR(cap_clk_sel, 0200, NULL, sunxi_tvtop_store_cap_clk_sel);
static DEVICE_ATTR(adc_ctrl, 0200, NULL, sunxi_tvtop_store_adc_ctrl);

static struct attribute *tvtop_attrs[] = {
	&dev_attr_tvfe.attr,
	&dev_attr_tvcap.attr,
	&dev_attr_cap_clk_sel.attr,
	&dev_attr_adc_ctrl.attr,
	NULL,
};

static const struct attribute_group tvtop_attribute_group = {
	.attrs = tvtop_attrs,
};

static void tvtop_extcon_free(struct device *dev)
{
	int i;

	for (i = 0; i < TVTOP_MAX_MODULES; i++) {
		if (!extcon_cvbs[i])
			continue;
		devm_extcon_dev_unregister(dev, extcon_cvbs[i]);
		devm_extcon_dev_free(dev, extcon_cvbs[i]);
		extcon_cvbs[i] = NULL;
	}
}

static int sunxi_tvtop_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct sunxi_tvtop *hw;
	int ret;
	int i;

	hw = devm_kzalloc(dev, sizeof(*hw), GFP_KERNEL);
	if (!hw) {
		dev_err(dev, "kzalloc failed, out of memory!\n");
		return -ENOMEM;
	}

	hw->devt = 0;
	hw->class = class_create(THIS_MODULE, "tvtop");
	if (IS_ERR(hw->class)) {
		dev_err(dev, "class_create 'tvtop' failed!\n");
		return PTR_ERR(hw->class);
	}

	hw->device = device_create(hw->class, NULL, hw->devt, hw, "tvtop");
	if (IS_ERR(hw->device)) {
		ret = PTR_ERR(hw->device);
		dev_err(dev, "device_create 'tvtop' failed!\n");
		class_destroy(hw->class);
		hw->class = NULL;
		return ret;
	}

	hw->parent_dev = dev;
	platform_set_drvdata(pdev, hw);

	for (i = 0; i < TVTOP_MAX_MODULES; i++) {
		ret = tvtop_driver_data_init(dev, np, hw, i);
		if (ret) {
			dev_err(dev, "tvtop_driver_data_init error, type=%d!\n", i);
			goto err_data;
		}
		hw->driver[i].initialized = true;
	}

	tvtopdev = hw;

	hw->driver[0].cvbs_wq = alloc_workqueue("tvtop_cvbs",
						WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!hw->driver[0].cvbs_wq) {
		pr_err("CVBS: Failed to create workqueue\n");
	} else {
		INIT_DELAYED_WORK(&hw->driver[0].cvbs.work, cvbs_work_handler);

		for (i = 0; i < TVTOP_MAX_MODULES; i++) {
			extcon_cvbs[i] = devm_extcon_dev_allocate(hw->device,
								  tv_cable);
			if (IS_ERR(extcon_cvbs[i])) {
				extcon_cvbs[i] = NULL;
				break;
			}
			if (devm_extcon_dev_register(hw->device,
						     extcon_cvbs[i])) {
				devm_extcon_dev_free(hw->device, extcon_cvbs[i]);
				extcon_cvbs[i] = NULL;
				break;
			}
		}
		if (i != TVTOP_MAX_MODULES) {
			tvtop_extcon_free(hw->device);
			destroy_workqueue(hw->driver[0].cvbs_wq);
			hw->driver[0].cvbs_wq = NULL;
		}
	}

	if (sysfs_create_group(&hw->device->kobj, &tvtop_attribute_group))
		dev_warn(hw->device, "failed to create tvtop attr group\n");

	return 0;

err_data:
	while (--i >= 0)
		tvtop_driver_data_exit(hw, i);
	device_destroy(hw->class, hw->devt);
	class_destroy(hw->class);
	hw->class = NULL;
	platform_set_drvdata(pdev, NULL);
	return ret;
}

static int sunxi_tvtop_remove(struct platform_device *pdev)
{
	struct sunxi_tvtop *hw = platform_get_drvdata(pdev);

	if (!hw)
		return 0;

	if (hw->device)
		sysfs_remove_group(&hw->device->kobj, &tvtop_attribute_group);

	tvtop_submodule_disable(hw->device, 0);
	if (hw->driver[0].initialized)
		tvtop_driver_data_exit(hw, 0);

	tvtop_submodule_disable(hw->device, 1);
	if (hw->driver[1].initialized)
		tvtop_driver_data_exit(hw, 1);

	device_destroy(hw->class, hw->devt);
	class_destroy(hw->class);
	hw->class = NULL;
	platform_set_drvdata(pdev, NULL);
	tvtopdev = NULL;

	return 0;
}

/* ---------------------------------------------------------------------------
 * Exported interface
 * ------------------------------------------------------------------------- */

unsigned int *sunxi_tvtop_capclk_sel(int sel)
{
	void __iomem *reg;
	u32 value;

	if (!tvtopdev || !tvtopdev->driver[1].base)
		return NULL;

	reg = tvtopdev->driver[1].base + TVCAP_CTRL2_REG;
	value = readl(reg) & ~0x100u;
	if (sel == 1)
		value |= 0x100u;
	writel(value, reg);

	return (unsigned int *)reg;
}
EXPORT_SYMBOL(sunxi_tvtop_capclk_sel);

int sunxi_tvtop_clk_get(unsigned int module)
{
	struct tvtop_ref_data *ref;
	int ret = 0;

	if (module >= TVTOP_MAX_MODULES) {
		dev_err(tvtopdev->parent_dev, "%s: unknown module type %d\n",
			__func__, module);
		return -EINVAL;
	}

	ref = &tvtopdev->ref_data[module];
	mutex_lock(&ref->lock);
	if (ref->ref_count++ == 0)
		ret = ref->enable_pfn(tvtopdev->device);
	mutex_unlock(&ref->lock);

	return ret;
}
EXPORT_SYMBOL(sunxi_tvtop_clk_get);

int sunxi_tvtop_clk_put(unsigned int module)
{
	struct tvtop_ref_data *ref;
	int ret = 0;

	if (module >= TVTOP_MAX_MODULES) {
		dev_err(tvtopdev->parent_dev, "%s: unknown module type %d\n",
			__func__, module);
		return -EINVAL;
	}

	ref = &tvtopdev->ref_data[module];
	mutex_lock(&ref->lock);
	if (ref->ref_count) {
		if (--ref->ref_count == 0)
			tvtop_submodule_disable(tvtopdev->device, module);
	} else {
		dev_err(tvtopdev->parent_dev, "%s: unbalance clk operation\n",
			__func__);
		WARN(1, "sunxi_tvtop: unbalance clk operation\n");
		ret = -1;
	}
	mutex_unlock(&ref->lock);

	return ret;
}
EXPORT_SYMBOL(sunxi_tvtop_clk_put);

int sunxi_tvtop_client_register(struct device *dev)
{
	if (!tvtopdev || !tvtopdev->parent_dev)
		return -EINVAL;

	device_link_add(dev, tvtopdev->parent_dev,
			DL_FLAG_AUTOREMOVE_CONSUMER);

	return 0;
}
EXPORT_SYMBOL(sunxi_tvtop_client_register);

int sunxi_tvtop_set_adc_ctrl(unsigned int module, int offset, int value)
{
	struct tvtop_driver_data *d;

	if (module >= TVTOP_MAX_MODULES) {
		dev_err(tvtopdev->parent_dev, "%s: unknown module type %d\n",
			__func__, module);
		return -EINVAL;
	}

	if (module == 1) {
		if (offset != 20) {
			dev_err(tvtopdev->parent_dev,
				"%s: TVCAP adc address offset invalid\n",
				__func__);
			return -EINVAL;
		}
	} else if (!(offset == 4 || offset == 1280 || offset == 1284 ||
		     offset == 1288)) {
		dev_err(tvtopdev->parent_dev,
			"%s: TVFE adc address offset invalid\n", __func__);
		return -EINVAL;
	}

	d = &tvtopdev->driver[module];
	writel(value, d->base + offset);

	return 0;
}
EXPORT_SYMBOL(sunxi_tvtop_set_adc_ctrl);

int cvbs_enable_hotplug_detect(char id, int enable)
{
	struct tvtop_driver_data *d;
	u32 value;

	if (!tvtopdev || tvtopdev->driver[0].enabled != 1)
		return -1;

	d = &tvtopdev->driver[0];
	value = readl(d->base + TVFE_CVBS_EN_REG);
	if (enable)
		value |= 1 << id;
	else
		value &= ~(1 << id);
	writel(value, d->base + TVFE_CVBS_EN_REG);

	return 0;
}
EXPORT_SYMBOL(cvbs_enable_hotplug_detect);

int cvbs_extcon_set_state(unsigned int id, int state)
{
	/*
	 * The vendor accepts id 2 here as well, which indexes past its two
	 * extcon devices. Keep the two valid channels and reject the rest.
	 */
	if (id >= TVTOP_MAX_MODULES)
		return -1;
	if (!extcon_cvbs[id])
		return -1;

	return extcon_set_state_sync(extcon_cvbs[id], TVTOP_EXTCON_CVBS_IN,
				     state != 0);
}
EXPORT_SYMBOL(cvbs_extcon_set_state);

static const struct of_device_id sunxi_tvtop_match[] = {
	{ .compatible = "allwinner,sunxi-tvtop" },
	{ .compatible = "allwinner,sunxi-tvtop-pm" },
	{ }
};
MODULE_DEVICE_TABLE(of, sunxi_tvtop_match);

static struct platform_driver sunxi_tvtop_driver = {
	.probe = sunxi_tvtop_probe,
	.remove = sunxi_tvtop_remove,
	.driver = {
		.name = "sunxi-tvtop",
		.of_match_table = sunxi_tvtop_match,
	},
};
module_platform_driver(sunxi_tvtop_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Sunxi tv subsystem top module driver");
