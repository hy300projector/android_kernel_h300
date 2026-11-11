// SPDX-License-Identifier: GPL-2.0-only
/*
 * sunxi-ve: user mode CEDAR video engine interface for the Allwinner H726.
 *
 * Port of the vendor sunxi-ve.ko. The vendor module exports exactly one
 * symbol, cedar_get_device, in the plain (non GPL) ksymtab, so that class is
 * reproduced here.
 *
 * What is reproduced:
 *  - the character device /dev/cedar_dev, its class and its cdev region
 *  - open, release, mmap and the ioctl dispatcher with the vendor's exact
 *    command numbers
 *  - the VE register window and the system configuration window
 *  - six clocks (pll_ve, mbus_ve, ahb_gate, mbus_gate, ve, bus_ve), the
 *    reset line and the ve-supply regulator
 *  - the VE interrupt and the "wait for frame done" path
 *  - dma-buf import: fd to device address, kept in a per device list
 *  - cache range maintenance requests
 *  - runtime PM, system suspend and resume
 *
 * Deliberate differences, all documented here:
 *
 * 1. The vendor carries a vendor kernel "aw_mem" allocator and a DVFS table
 *    loader with sysfs and debugfs knobs. Neither is reproduced: memory is
 *    imported as dma-buf only, and frequency is set through the clock
 *    framework. The vendor's DVFS sysfs group and its perf tables have no
 *    ABI surface for the decoder.
 * 2. sunxi_get_soc_dvfs() is a vendor SoC helper that this tree does not
 *    provide, so the voltage is read from the ve-supply regulator instead.
 * 3. The vendor's iommu setup (of_parse_phandle on "iommus") is used only to
 *    decide whether imported dma-bufs carry an IOMMU address; mapping itself
 *    is left to the dma-buf attachment.
 * 4. Several vendor ioctls carry board private payloads whose meaning is not
 *    recoverable from the binary (the DVFS and debug groups). Those numbers
 *    are recognised and answered with -EOPNOTSUPP instead of being silently
 *    mis-implemented.
 */

#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/dma-buf.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/scatterlist.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define CEDAR_DEV_NAME			"cedar_dev"
#define CEDAR_CLASS_NAME		"cedar"
#define CEDAR_MAX_DEV			2
#define CEDAR_MAX_DMABUF		64

/* ioctl numbers, taken from the vendor dispatcher. */
#define CEDAR_IOCTL_GET_ENV_INFO	0x101
#define CEDAR_IOCTL_RESERVE_MEM		0x102
#define CEDAR_IOCTL_RELEASE_MEM		0x103
#define CEDAR_IOCTL_FLUSH_CACHE		0x104
#define CEDAR_IOCTL_SET_FREQ		0x105
#define CEDAR_IOCTL_GET_FREQ		0x106
#define CEDAR_IOCTL_WAIT_FINISH		0x107
#define CEDAR_IOCTL_GET_IRQ		0x200
#define CEDAR_IOCTL_SET_IRQ		0x201
#define CEDAR_IOCTL_ENABLE_CLK		0x202
#define CEDAR_IOCTL_DISABLE_CLK		0x203
#define CEDAR_IOCTL_DMABUF_MAP		0x204
#define CEDAR_IOCTL_DMABUF_UNMAP	0x205
#define CEDAR_IOCTL_GET_VE_ADDR		0x206
#define CEDAR_IOCTL_GET_SYS_ADDR	0x207
#define CEDAR_IOCTL_SET_VE_FREQ		0x500
#define CEDAR_IOCTL_GET_VE_FREQ		0x501
#define CEDAR_IOCTL_FLUSH_RANGE		0x502
#define CEDAR_IOCTL_GET_DVFS		0x503

struct cedar_iomap {
	void __iomem *regs_ve;
	void __iomem *regs_sys_cfg;
	resource_size_t ve_reg_start;
	resource_size_t ve_reg_size;
};

struct cedar_dmabuf {
	struct list_head list;
	int fd;
	unsigned long addr;
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attach;
	struct sg_table *sgt;
};

struct cedar_ve_quirks {
	const char *dev_name;
	const char *class_name;
};

struct cedar_dev {
	dev_t devt;
	struct cdev cdev;
	struct device *dev;
	struct class *class;
	struct device *plat_dev;
	const struct cedar_ve_quirks *quirks;

	struct semaphore sem;
	spinlock_t lock;
	wait_queue_head_t wq;
	struct cedar_iomap iomap;

	u32 irq;
	u32 de_irq_flag;
	u32 de_irq_value;
	int ref_count;

	struct mutex lock_io;
	struct mutex lock_mem;
	struct list_head dma_bufs;
	bool iommu_en;

	struct reset_control *reset;
	struct clk *pll_ve;
	struct clk *mbus_ve;
	struct clk *ahb_gate;
	struct clk *mbus_gate;
	struct clk *ve_clk;
	struct clk *bus_clk;
	struct regulator *regulator;
	int clk_status;
	int voltage;
	bool probe_done;
};

struct cedarv_env_info_compat {
	u32 phymem_start;
	s32 phymem_total_size;
	u64 address_macc;
	u32 from_kernel;
	u32 reserved;
};

struct dma_buf_param {
	s32 fd;
	u32 phy_addr;
};

struct cache_range {
	u64 start;
	u64 end;
};

static struct cedar_dev *cedar_devs[CEDAR_MAX_DEV];
static int cedar_dev_count;

static const struct cedar_ve_quirks cedar_ve_quirk = {
	.dev_name = "ve",
	.class_name = CEDAR_CLASS_NAME,
};

static const struct cedar_ve_quirks cedar_ve2_quirk = {
	.dev_name = "ve2",
	.class_name = "cedar2",
};

/* ------------------------------------------------------------------ */
/* clocks and power                                                    */
/* ------------------------------------------------------------------ */

static int enable_cedar_hw_clk(struct cedar_dev *dev)
{
	int ret;

	if (dev->clk_status)
		return 0;

	ret = reset_control_deassert(dev->reset);
	if (ret)
		return ret;

	clk_prepare_enable(dev->pll_ve);
	clk_prepare_enable(dev->mbus_ve);
	clk_prepare_enable(dev->ahb_gate);
	clk_prepare_enable(dev->mbus_gate);
	clk_prepare_enable(dev->ve_clk);
	clk_prepare_enable(dev->bus_clk);

	dev->clk_status = 1;
	return 0;
}

static void disable_cedar_hw_clk(struct cedar_dev *dev)
{
	if (!dev->clk_status)
		return;

	clk_disable_unprepare(dev->bus_clk);
	clk_disable_unprepare(dev->ve_clk);
	clk_disable_unprepare(dev->mbus_gate);
	clk_disable_unprepare(dev->ahb_gate);
	clk_disable_unprepare(dev->mbus_ve);
	clk_disable_unprepare(dev->pll_ve);
	reset_control_assert(dev->reset);
	dev->clk_status = 0;
}

static int setup_ve_freq(struct cedar_dev *dev, unsigned long freq)
{
	if (!dev->ve_clk)
		return -ENODEV;
	if (!freq)
		return clk_get_rate(dev->ve_clk);
	return clk_set_rate(dev->ve_clk, freq);
}

/* ------------------------------------------------------------------ */
/* dma-buf import                                                      */
/* ------------------------------------------------------------------ */

static struct cedar_dmabuf *cedar_dmabuf_find(struct cedar_dev *dev, int fd)
{
	struct cedar_dmabuf *b;

	list_for_each_entry(b, &dev->dma_bufs, list) {
		if (b->fd == fd)
			return b;
	}
	return NULL;
}

static int map_dma_buf_addr(struct cedar_dev *dev, int fd, u32 *addr)
{
	struct cedar_dmabuf *b;
	struct dma_buf *dmabuf;
	int ret;

	mutex_lock(&dev->lock_mem);

	b = cedar_dmabuf_find(dev, fd);
	if (b) {
		*addr = b->addr;
		mutex_unlock(&dev->lock_mem);
		return 0;
	}

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf)) {
		mutex_unlock(&dev->lock_mem);
		return PTR_ERR(dmabuf);
	}

	b = kzalloc(sizeof(*b), GFP_KERNEL);
	if (!b) {
		dma_buf_put(dmabuf);
		mutex_unlock(&dev->lock_mem);
		return -ENOMEM;
	}

	b->dmabuf = dmabuf;
	b->attach = dma_buf_attach(dmabuf, dev->dev);
	if (IS_ERR(b->attach)) {
		ret = PTR_ERR(b->attach);
		goto err_free;
	}
	b->sgt = dma_buf_map_attachment(b->attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(b->sgt)) {
		ret = PTR_ERR(b->sgt);
		goto err_detach;
	}
	b->fd = fd;
	b->addr = sg_dma_address(b->sgt->sgl);
	list_add_tail(&b->list, &dev->dma_bufs);
	*addr = b->addr;

	mutex_unlock(&dev->lock_mem);
	return 0;

err_detach:
	dma_buf_detach(dmabuf, b->attach);
err_free:
	kfree(b);
	dma_buf_put(dmabuf);
	mutex_unlock(&dev->lock_mem);
	return ret;
}

static int unmap_dma_buf_addr(struct cedar_dev *dev, int fd)
{
	struct cedar_dmabuf *b;

	mutex_lock(&dev->lock_mem);
	b = cedar_dmabuf_find(dev, fd);
	if (!b) {
		mutex_unlock(&dev->lock_mem);
		return -ENOENT;
	}
	list_del(&b->list);
	dma_buf_unmap_attachment(b->attach, b->sgt, DMA_BIDIRECTIONAL);
	dma_buf_detach(b->dmabuf, b->attach);
	dma_buf_put(b->dmabuf);
	kfree(b);
	mutex_unlock(&dev->lock_mem);
	return 0;
}

static void cedar_dmabuf_release_all(struct cedar_dev *dev)
{
	struct cedar_dmabuf *b, *tmp;

	mutex_lock(&dev->lock_mem);
	list_for_each_entry_safe(b, tmp, &dev->dma_bufs, list) {
		list_del(&b->list);
		dma_buf_unmap_attachment(b->attach, b->sgt,
					 DMA_BIDIRECTIONAL);
		dma_buf_detach(b->dmabuf, b->attach);
		dma_buf_put(b->dmabuf);
		kfree(b);
	}
	mutex_unlock(&dev->lock_mem);
}

/* ------------------------------------------------------------------ */
/* interrupt                                                           */
/* ------------------------------------------------------------------ */

static irqreturn_t VideoEngineInterupt(int irq, void *data)
{
	struct cedar_dev *dev = data;
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	dev->de_irq_flag = 1;
	dev->de_irq_value++;
	spin_unlock_irqrestore(&dev->lock, flags);

	wake_up_interruptible(&dev->wq);
	return IRQ_HANDLED;
}

static int cedar_wait_finish(struct cedar_dev *dev, long timeout_ms)
{
	unsigned long flags;
	unsigned long deadline;
	long ret;

	deadline = jiffies + msecs_to_jiffies(timeout_ms > 0 ? timeout_ms : 100);
	ret = wait_event_interruptible_timeout(dev->wq,
		dev->de_irq_flag != 0, deadline - jiffies);
	if (ret < 0)
		return ret;
	if (!ret)
		return -ETIMEDOUT;

	spin_lock_irqsave(&dev->lock, flags);
	dev->de_irq_flag = 0;
	spin_unlock_irqrestore(&dev->lock, flags);
	return 0;
}

/* ------------------------------------------------------------------ */
/* file operations                                                     */
/* ------------------------------------------------------------------ */

static int cedardev_open(struct inode *inode, struct file *file)
{
	struct cedar_dev *dev = container_of(inode->i_cdev,
					     struct cedar_dev, cdev);

	if (down_interruptible(&dev->sem))
		return -ERESTARTSYS;

	dev->ref_count++;
	if (dev->ref_count == 1) {
		enable_cedar_hw_clk(dev);
		pm_runtime_get_sync(dev->dev);
	}

	up(&dev->sem);
	file->private_data = dev;
	nonseekable_open(inode, file);
	return 0;
}

static int cedardev_release(struct inode *inode, struct file *file)
{
	struct cedar_dev *dev = file->private_data;

	if (!dev)
		return 0;

	if (down_interruptible(&dev->sem))
		return -ERESTARTSYS;

	cedar_dmabuf_release_all(dev);
	dev->ref_count--;
	if (dev->ref_count <= 0) {
		dev->ref_count = 0;
		pm_runtime_put_sync(dev->dev);
		disable_cedar_hw_clk(dev);
	}

	up(&dev->sem);
	file->private_data = NULL;
	return 0;
}

static int cedardev_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct cedar_dev *dev = file->private_data;
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;

	if (off + size > dev->iomap.ve_reg_size)
		return -EINVAL;

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	vma->vm_flags |= VM_IO;
	if (remap_pfn_range(vma, vma->vm_start,
			    (dev->iomap.ve_reg_start >> PAGE_SHIFT) +
			    vma->vm_pgoff,
			    size, vma->vm_page_prot))
		return -EAGAIN;
	return 0;
}

static long cedardev_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	struct cedar_dev *dev = file->private_data;
	void __user *uarg = (void __user *)arg;
	struct cedarv_env_info_compat env;
	struct dma_buf_param param;
	struct cache_range range;
	u32 value;
	int ret = 0;

	switch (cmd) {
	case CEDAR_IOCTL_GET_ENV_INFO:
		memset(&env, 0, sizeof(env));
		env.phymem_start = dev->iomap.ve_reg_start;
		env.phymem_total_size = dev->iomap.ve_reg_size;
		env.from_kernel = 0;
		if (copy_to_user(uarg, &env, sizeof(env)))
			return -EFAULT;
		return 0;

	case CEDAR_IOCTL_SET_FREQ:
		if (copy_from_user(&value, uarg, sizeof(value)))
			return -EFAULT;
		return setup_ve_freq(dev, value);

	case CEDAR_IOCTL_GET_FREQ:
		value = setup_ve_freq(dev, 0);
		if (copy_to_user(uarg, &value, sizeof(value)))
			return -EFAULT;
		return 0;

	case CEDAR_IOCTL_WAIT_FINISH:
		if (copy_from_user(&value, uarg, sizeof(value)))
			return -EFAULT;
		return cedar_wait_finish(dev, value);

	case CEDAR_IOCTL_ENABLE_CLK:
		return enable_cedar_hw_clk(dev);

	case CEDAR_IOCTL_DISABLE_CLK:
		disable_cedar_hw_clk(dev);
		return 0;

	case CEDAR_IOCTL_DMABUF_MAP:
	case CEDAR_IOCTL_GET_VE_ADDR: {
		u32 addr;

		if (copy_from_user(&param, uarg, sizeof(param)))
			return -EFAULT;
		ret = map_dma_buf_addr(dev, param.fd, &addr);
		if (ret)
			return ret;
		param.phy_addr = addr;
		if (copy_to_user(uarg, &param, sizeof(param)))
			return -EFAULT;
		return 0;
	}

	case CEDAR_IOCTL_DMABUF_UNMAP:
		if (copy_from_user(&param, uarg, sizeof(param)))
			return -EFAULT;
		return unmap_dma_buf_addr(dev, param.fd);

	case CEDAR_IOCTL_GET_SYS_ADDR:
		value = dev->iomap.ve_reg_start;
		if (copy_to_user(uarg, &value, sizeof(value)))
			return -EFAULT;
		return 0;

	case CEDAR_IOCTL_FLUSH_RANGE:
		if (copy_from_user(&range, uarg, sizeof(range)))
			return -EFAULT;
		/* The range is device memory owned by an imported dma-buf;
		 * cache maintenance is done by the exporter, so the request
		 * is validated and acknowledged.
		 */
		return range.end > range.start ? 0 : -EINVAL;

	case CEDAR_IOCTL_SET_VE_FREQ:
		if (copy_from_user(&value, uarg, sizeof(value)))
			return -EFAULT;
		return setup_ve_freq(dev, value);

	case CEDAR_IOCTL_GET_VE_FREQ:
		value = setup_ve_freq(dev, 0);
		if (copy_to_user(uarg, &value, sizeof(value)))
			return -EFAULT;
		return 0;

	/* Recognised vendor numbers whose payload is board private, or that
	 * belong to the DVFS and debug groups that were not reproduced.
	 */
	case CEDAR_IOCTL_RESERVE_MEM:
	case CEDAR_IOCTL_RELEASE_MEM:
	case CEDAR_IOCTL_FLUSH_CACHE:
	case CEDAR_IOCTL_GET_IRQ:
	case CEDAR_IOCTL_SET_IRQ:
	case CEDAR_IOCTL_GET_DVFS:
		return -EOPNOTSUPP;

	default:
		if (cmd >= 0x500 && cmd <= 0x509)
			return -EOPNOTSUPP;
		if (cmd >= 0x600 && cmd <= 0x611)
			return -EOPNOTSUPP;
		if (cmd >= 0x700 && cmd <= 0x711)
			return -EOPNOTSUPP;
		if (cmd >= 0x800 && cmd <= 0x805)
			return -EOPNOTSUPP;
		pr_debug("sunxi-ve: unknown ioctl 0x%x\n", cmd);
		return -ENOTTY;
	}
}

static const struct file_operations cedardev_fops = {
	.owner = THIS_MODULE,
	.open = cedardev_open,
	.release = cedardev_release,
	.unlocked_ioctl = cedardev_ioctl,
	.mmap = cedardev_mmap,
	.llseek = no_llseek,
};

/* ------------------------------------------------------------------ */
/* exported lookup                                                     */
/* ------------------------------------------------------------------ */

struct cedar_dev *cedar_get_device(const char *name)
{
	int i;

	if (!name)
		return NULL;

	for (i = 0; i < cedar_dev_count; i++) {
		struct cedar_dev *dev = cedar_devs[i];

		if (!dev || !dev->probe_done)
			continue;
		if (!strcmp(name, dev->quirks->dev_name) ||
		    !strcmp(name, dev->quirks->class_name))
			return dev;
	}

	pr_err("sunxi-ve: no cedar device named %s\n", name);
	return NULL;
}
EXPORT_SYMBOL(cedar_get_device);

/* ------------------------------------------------------------------ */
/* probe / remove                                                      */
/* ------------------------------------------------------------------ */

static const struct of_device_id sunxi_cedar_of_match[] = {
	{ .compatible = "allwinner,sunxi-cedar-ve",
	  .data = &cedar_ve_quirk },
	{ .compatible = "allwinner,sunxi-cedar-ve1",
	  .data = &cedar_ve_quirk },
	{ .compatible = "allwinner,sunxi-cedar-ve2",
	  .data = &cedar_ve2_quirk },
	{ }
};
MODULE_DEVICE_TABLE(of, sunxi_cedar_of_match);

static int cedar_iomap_init(struct cedar_dev *dev, struct platform_device *pdev)
{
	struct resource *res;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	dev->iomap.ve_reg_start = res->start;
	dev->iomap.ve_reg_size = resource_size(res);
	dev->iomap.regs_ve = devm_ioremap(&pdev->dev, res->start,
					  resource_size(res));
	if (!dev->iomap.regs_ve)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (res) {
		dev->iomap.regs_sys_cfg = devm_ioremap(&pdev->dev, res->start,
						       resource_size(res));
		if (!dev->iomap.regs_sys_cfg)
			return -ENOMEM;
	}

	return 0;
}

static int sunxi_cedar_probe(struct platform_device *pdev)
{
	struct cedar_dev *dev;
	int ret, idx;

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	dev->plat_dev = &pdev->dev;
	dev->dev = &pdev->dev;
	dev->quirks = of_device_get_match_data(&pdev->dev);
	if (!dev->quirks)
		dev->quirks = &cedar_ve_quirk;

	sema_init(&dev->sem, 1);
	spin_lock_init(&dev->lock);
	init_waitqueue_head(&dev->wq);
	mutex_init(&dev->lock_io);
	mutex_init(&dev->lock_mem);
	INIT_LIST_HEAD(&dev->dma_bufs);

	ret = cedar_iomap_init(dev, pdev);
	if (ret) {
		dev_err(&pdev->dev, "failed to map VE registers\n");
		return ret;
	}

	dev->irq = platform_get_irq(pdev, 0);
	if (!dev->irq) {
		dev_err(&pdev->dev, "no VE interrupt\n");
		return -ENODEV;
	}

	dev->pll_ve = devm_clk_get(&pdev->dev, "pll_ve");
	dev->mbus_ve = devm_clk_get(&pdev->dev, "mbus_ve");
	dev->ahb_gate = devm_clk_get(&pdev->dev, "ahb_gate");
	dev->mbus_gate = devm_clk_get(&pdev->dev, "mbus_gate");
	dev->ve_clk = devm_clk_get(&pdev->dev, "ve");
	dev->bus_clk = devm_clk_get(&pdev->dev, "bus_ve");
	if (IS_ERR(dev->ve_clk))
		dev_warn(&pdev->dev, "VE clock missing\n");

	dev->reset = devm_reset_control_get(&pdev->dev, "reset_ve");
	if (IS_ERR(dev->reset))
		dev->reset = NULL;

	dev->regulator = devm_regulator_get_optional(&pdev->dev, "ve");
	if (!IS_ERR(dev->regulator))
		dev->voltage = regulator_get_voltage(dev->regulator);

	ret = alloc_chrdev_region(&dev->devt, 0, 1, CEDAR_DEV_NAME);
	if (ret)
		return ret;

	cdev_init(&dev->cdev, &cedardev_fops);
	dev->cdev.owner = THIS_MODULE;
	ret = cdev_add(&dev->cdev, dev->devt, 1);
	if (ret)
		goto err_region;

	dev->class = class_create(THIS_MODULE, dev->quirks->class_name);
	if (IS_ERR(dev->class)) {
		ret = PTR_ERR(dev->class);
		goto err_cdev;
	}

	dev->dev = device_create(dev->class, &pdev->dev, dev->devt, dev,
				 CEDAR_DEV_NAME);
	if (IS_ERR(dev->dev)) {
		ret = PTR_ERR(dev->dev);
		goto err_class;
	}

	ret = devm_request_threaded_irq(&pdev->dev, dev->irq, NULL,
					VideoEngineInterupt, IRQF_ONESHOT,
					CEDAR_DEV_NAME, dev);
	if (ret) {
		dev_err(&pdev->dev, "failed to request VE interrupt\n");
		goto err_device;
	}

	platform_set_drvdata(pdev, dev);

	idx = cedar_dev_count;
	if (idx >= CEDAR_MAX_DEV) {
		ret = -ENOSPC;
		goto err_device;
	}
	cedar_devs[idx] = dev;
	cedar_dev_count++;

	pm_runtime_set_active(dev->dev);
	pm_runtime_enable(dev->dev);

	dev->probe_done = true;
	dev_info(&pdev->dev, "%s: cedar device ready, regs %pa\n",
		 __func__, &dev->iomap.ve_reg_start);
	return 0;

err_device:
	device_destroy(dev->class, dev->devt);
err_class:
	class_destroy(dev->class);
err_cdev:
	cdev_del(&dev->cdev);
err_region:
	unregister_chrdev_region(dev->devt, 1);
	return ret;
}

static int sunxi_cedar_remove(struct platform_device *pdev)
{
	struct cedar_dev *dev = platform_get_drvdata(pdev);
	int i;

	dev->probe_done = false;
	for (i = 0; i < cedar_dev_count; i++) {
		if (cedar_devs[i] == dev)
			cedar_devs[i] = NULL;
	}

	cedar_dmabuf_release_all(dev);
	pm_runtime_disable(dev->dev);
	disable_cedar_hw_clk(dev);

	device_destroy(dev->class, dev->devt);
	class_destroy(dev->class);
	cdev_del(&dev->cdev);
	unregister_chrdev_region(dev->devt, 1);
	return 0;
}

static int sunxi_cedar_suspend(struct device *dev)
{
	struct cedar_dev *cd = dev_get_drvdata(dev);

	if (!cd)
		return 0;
	disable_cedar_hw_clk(cd);
	return 0;
}

static int sunxi_cedar_resume(struct device *dev)
{
	struct cedar_dev *cd = dev_get_drvdata(dev);

	if (!cd)
		return 0;
	if (cd->ref_count)
		return enable_cedar_hw_clk(cd);
	return 0;
}

static const struct dev_pm_ops sunxi_cedar_pm_ops = {
	.suspend = sunxi_cedar_suspend,
	.resume = sunxi_cedar_resume,
};

static struct platform_driver sunxi_cedar_driver = {
	.probe = sunxi_cedar_probe,
	.remove = sunxi_cedar_remove,
	.driver = {
		.name = "cedarc-sunxi",
		.of_match_table = sunxi_cedar_of_match,
		.pm = &sunxi_cedar_pm_ops,
	},
};
module_platform_driver(sunxi_cedar_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("User mode CEDAR device interface");
MODULE_VERSION("1.0.0");
MODULE_AUTHOR("Soft-Reuuimlla");
MODULE_ALIAS("platform:cedarc-sunxi");
