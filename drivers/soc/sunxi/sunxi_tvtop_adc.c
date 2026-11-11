// SPDX-License-Identifier: GPL-2.0
/*
 * sunxi_tvtop_adc - CVBS ADC control node for the Allwinner H726
 *
 * Reconstruction of the vendor module sunxi_tvtop_adc.ko (kernel 5.15.167).
 * The module creates /dev/tvtop_adc_dev and forwards the vendor ioctl
 * commands to the TV front end exported by sunxi_tvtop.
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>

/* Provided by sunxi_tvtop */
int sunxi_tvtop_set_adc_ctrl(unsigned int module, int offset, int value);
int cvbs_enable_hotplug_detect(char id, int enable);

/*
 * Vendor ioctl numbers. The two _IOW commands take an int in user space, the
 * two _IO commands take no argument. Keep the exact numbers of the vendor
 * binary: they are the interface used by the vendor TV application.
 */
#define TVTOP_ADC_SET_TVCAP_MODE	_IOW('a', 1, int)	/* 0x40046101 */
#define TVTOP_ADC_SET_TVFE_REG		_IOW('a', 2, int)	/* 0x40046102 */
#define TVTOP_ADC_ADC_ON		_IO('a', 0)		/* 0x00006100 */
#define TVTOP_ADC_ADC_OFF		_IO('a', 3)		/* 0x00006103 */

/* tvfe register 0x500 (byte offset 1280) mode values used by the vendor */
#define TVFE_CVBS_MODE_ON		52432064	/* 0x032003c0 */
#define TVFE_CVBS_MODE_TVCAP		(-920647040)	/* 0xc91c1c00 */
#define TVFE_CVBS_MODE_TVFE		(-887092608)	/* 0xcb201280 */
#define TVFE_CVBS_MODE_ADC_ON		(-1908932608)	/* 0x8e38e38e */

#define TVCAP_ADC_CTRL_OFFSET		20
#define TVCAP_ADC_CTRL_VALUE		16777232	/* 0x01000010 */

struct tvtop_adc_device {
	struct cdev cdev;
	int threshold;
	struct mutex lock;
};

static dev_t tvtop_adc_devt;
static int tvtop_adc_major;
static struct class *tvtop_adc_class;
static struct tvtop_adc_device *tvtop_adc_dev;
static struct device *tvtop_adc_device;

/* Switch the capture path into TVCAP mode and hand the ADC over to tvcap. */
static int sunxi_cvbs_in_adc_tvcap_control(int id)
{
	int ret;

	cvbs_enable_hotplug_detect(id, 0);

	if (sunxi_tvtop_set_adc_ctrl(0, 1280, TVFE_CVBS_MODE_TVCAP))
		pr_err("sunxi_tvtop_adc: set tvfe cvbs mode failed\n");

	ret = sunxi_tvtop_set_adc_ctrl(1, TVCAP_ADC_CTRL_OFFSET,
				       TVCAP_ADC_CTRL_VALUE);
	if (ret)
		pr_err("sunxi_tvtop_adc: set tvcap adc ctrl failed\n");

	return ret;
}

/*
 * Set the tvfe ADC register window. id 0 uses the TVFE sampling setup, id 1
 * leaves the TVCAP setup in place. The vendor returns the status of the first
 * write for id 1 and the status of the second write for id 0.
 */
static int sunxi_cvbs_in_adc_reg_control(int id)
{
	int ret;

	cvbs_enable_hotplug_detect(id, 0);

	ret = sunxi_tvtop_set_adc_ctrl(0, 1280, TVFE_CVBS_MODE_TVFE);
	if (ret)
		pr_err("sunxi_tvtop_adc: set tvfe cvbs mode failed\n");

	if (id != 1) {
		int err = sunxi_tvtop_set_adc_ctrl(0, 1288, 2560);

		if (err)
			pr_err("sunxi_tvtop_adc: set tvfe cvbs cfg failed\n");
		return err;
	}

	/* The vendor ignores the result of this write. */
	sunxi_tvtop_set_adc_ctrl(0, 1288, 8391168);

	return ret;
}

static long adc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int value = 0;
	int ret;

	switch (cmd) {
	case TVTOP_ADC_SET_TVCAP_MODE:
		if (copy_from_user(&value, (void __user *)arg, sizeof(value)))
			return -EFAULT;
		ret = sunxi_cvbs_in_adc_tvcap_control(value);
		if (ret) {
			pr_err("sunxi_tvtop_adc: tvcap control failed: %d\n", ret);
			return ret;
		}
		pr_info("sunxi_tvtop_adc: switch to tvcap ok\n");
		return 0;

	case TVTOP_ADC_SET_TVFE_REG:
		if (copy_from_user(&value, (void __user *)arg, sizeof(value)))
			return -EFAULT;
		ret = sunxi_cvbs_in_adc_reg_control(value);
		if (ret) {
			pr_err("sunxi_tvtop_adc: tvfe reg control failed: %d\n",
			       ret);
			return ret;
		}
		pr_info("sunxi_tvtop_adc: switch to tvfe ok\n");
		return 0;

	case TVTOP_ADC_ADC_ON:
		if (sunxi_tvtop_set_adc_ctrl(0, 4, 0))
			pr_err("sunxi_tvtop_adc: clear tvfe adc ctrl failed\n");
		ret = sunxi_tvtop_set_adc_ctrl(0, 1280, TVFE_CVBS_MODE_ADC_ON);
		if (ret) {
			pr_err("sunxi_tvtop_adc: adc on failed: %d\n", ret);
			return ret;
		}
		pr_info("sunxi_tvtop_adc: adc on\n");
		return 0;

	case TVTOP_ADC_ADC_OFF:
		ret = sunxi_tvtop_set_adc_ctrl(0, 1280, TVFE_CVBS_MODE_ON);
		if (ret)
			pr_err("sunxi_tvtop_adc: adc off failed: %d\n", ret);
		/* The vendor arms both hotplug detectors in either case. */
		cvbs_enable_hotplug_detect(0, 1);
		cvbs_enable_hotplug_detect(1, 1);
		return ret;

	default:
		return -ENOTTY;
	}
}

static int adc_open(struct inode *inode, struct file *file)
{
	struct tvtop_adc_device *device;

	if (!inode->i_cdev)
		return -ENODEV;

	device = container_of(inode->i_cdev, struct tvtop_adc_device, cdev);

	mutex_lock(&device->lock);
	pr_info("sunxi_tvtop_adc: device open\n");
	file->private_data = device;
	mutex_unlock(&device->lock);

	return 0;
}

static int adc_release(struct inode *inode, struct file *file)
{
	struct tvtop_adc_device *device = file->private_data;

	if (device)
		device->threshold = 0;
	pr_info("sunxi_tvtop_adc: device release\n");

	return 0;
}

static const struct file_operations adc_fops = {
	.owner = THIS_MODULE,
	.open = adc_open,
	.release = adc_release,
	.unlocked_ioctl = adc_ioctl,
};

static int __init tvtop_adc_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&tvtop_adc_devt, 0, 1, "tvtop_adc_dev");
	if (ret) {
		pr_err("sunxi_tvtop_adc: alloc_chrdev_region failed: %d\n", ret);
		return ret;
	}
	tvtop_adc_major = MAJOR(tvtop_adc_devt);

	tvtop_adc_class = class_create(THIS_MODULE, "tvtop_adc");
	if (IS_ERR(tvtop_adc_class)) {
		ret = PTR_ERR(tvtop_adc_class);
		tvtop_adc_class = NULL;
		pr_err("sunxi_tvtop_adc: class_create failed: %d\n", ret);
		goto err_region;
	}

	tvtop_adc_dev = kzalloc(sizeof(*tvtop_adc_dev), GFP_KERNEL);
	if (!tvtop_adc_dev) {
		ret = -ENOMEM;
		goto err_class;
	}

	cdev_init(&tvtop_adc_dev->cdev, &adc_fops);
	tvtop_adc_dev->cdev.owner = THIS_MODULE;
	mutex_init(&tvtop_adc_dev->lock);

	ret = cdev_add(&tvtop_adc_dev->cdev, tvtop_adc_devt, 1);
	if (ret) {
		pr_err("sunxi_tvtop_adc: cdev_add failed: %d\n", ret);
		goto err_dev;
	}

	tvtop_adc_device = device_create(tvtop_adc_class, NULL, tvtop_adc_devt,
					 NULL, "tvtop_adc_dev%d", 0);
	if (IS_ERR(tvtop_adc_device)) {
		ret = PTR_ERR(tvtop_adc_device);
		tvtop_adc_device = NULL;
		pr_err("sunxi_tvtop_adc: device_create failed: %d\n", ret);
		goto err_cdev;
	}

	pr_info("sunxi_tvtop_adc: init ok, major %d\n", tvtop_adc_major);
	return 0;

err_cdev:
	cdev_del(&tvtop_adc_dev->cdev);
err_dev:
	kfree(tvtop_adc_dev);
	tvtop_adc_dev = NULL;
err_class:
	class_destroy(tvtop_adc_class);
	tvtop_adc_class = NULL;
err_region:
	unregister_chrdev_region(tvtop_adc_devt, 1);
	return ret;
}

static void __exit tvtop_adc_exit(void)
{
	if (tvtop_adc_dev) {
		cdev_del(&tvtop_adc_dev->cdev);
		kfree(tvtop_adc_dev);
		tvtop_adc_dev = NULL;
	}
	if (tvtop_adc_device) {
		device_destroy(tvtop_adc_class, tvtop_adc_devt);
		tvtop_adc_device = NULL;
	}
	if (tvtop_adc_class) {
		class_unregister(tvtop_adc_class);
		class_destroy(tvtop_adc_class);
		tvtop_adc_class = NULL;
	}
	unregister_chrdev_region(tvtop_adc_devt, 1);
	pr_info("sunxi_tvtop_adc: exit\n");
}

module_init(tvtop_adc_init);
module_exit(tvtop_adc_exit);

MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:sunxi-tvtop-adc");
MODULE_DESCRIPTION("tvtop adc driver");
