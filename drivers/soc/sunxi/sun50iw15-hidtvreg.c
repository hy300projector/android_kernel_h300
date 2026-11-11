// SPDX-License-Identifier: GPL-2.0-only
/*
 * Trident/Allwinner H726 "hidtvreg" register access device.
 *
 * Replacement for the vendor hidtvreg_dev.ko ("HiDTV Register Access Driver",
 * Trident Microsystems, dual BSD/GPL), recovered from dec_mod/hidtvreg_dev.c.
 * It is the register window the TV/display HAL (lib_dp_db's TTFDCompReg) uses
 * to reach TVTOP/TCON registers that no other driver claims, plus a cache
 * maintenance helper for buffers handed to the display hardware.
 *
 * Vendor ABI, kept exactly:
 *
 *   /dev/hidtvreg, char device, dynamic major, 256 minors, class "hidtvreg".
 *   ueventd.rc gives it 0666 system:media and SELinux labels it dtv_device.
 *
 *   ioctl 0x1700  arg is a u32 holding a physical address.  The driver maps
 *                 those four bytes, reads them, and returns the value through
 *                 the same u32.  Unrestricted like the vendor driver, because
 *                 the TV blocks are not owned by anything else; this is a
 *                 privileged-register interface, not a general debug hole.
 *
 *   ioctl 0x5f04  arg points to {count, addr0, len0, addr1, len1, ...}, where
 *                 count is the number of u32 entries (1 + 2 * pairs).  The
 *                 vendor invalidates the data cache over each [addr, addr+len)
 *                 in 32-byte steps; this implementation flushes the ranges,
 *                 which gives the caller the same (or a stronger) guarantee.
 *
 *   mmap          maps the physical range starting at vm_pgoff, non-cached, so
 *                 the HAL can touch a window without an ioctl per access.
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/uaccess.h>

#include <asm/cacheflush.h>

#define HIDTVREG_NAME			"hidtvreg"

/* Raw vendor request numbers: no _IO* encoding, arg is a userspace pointer. */
#define HIDTVREG_IOCTL_READ_PHYS	0x1700
#define HIDTVREG_IOCTL_CACHE		0x5f04

#define HIDTVREG_MAX_CACHE_ENTRIES	0x4000

static unsigned int hidtvreg_major;
static struct class *hidtvreg_class;
static struct device *hidtvreg_device;

static int hidtvreg_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int hidtvreg_release(struct inode *inode, struct file *file)
{
	return 0;
}

/*
 * Read one 32-bit register at a physical address supplied by the caller and
 * hand the value back through the same u32.  This mirrors the vendor's
 * ioremap(addr, 4) / read / iounmap sequence.
 */
static long hidtvreg_read_phys(unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	void __iomem *reg;
	u32 addr, val;

	if (copy_from_user(&addr, uarg, sizeof(addr)))
		return -EFAULT;

	addr &= ~3U;

	reg = ioremap(addr, sizeof(u32));
	if (!reg)
		return -ENOMEM;

	val = readl(reg);
	iounmap(reg);

	if (copy_to_user(uarg, &val, sizeof(val)))
		return -EFAULT;

	return 0;
}

/*
 * Cache maintenance over a list of {count, addr, len} entries.  Only complete
 * pairs are touched; the vendor loop starts at index 1 and steps by two, so a
 * well-formed list has an odd count.
 */
static long hidtvreg_cache_list(unsigned long arg)
{
	u32 __user *list = (u32 __user *)arg;
	u32 count, i;
	u32 addr, len;

	if (get_user(count, list))
		return -EFAULT;

	if (count < 3 || !(count & 1) || count > HIDTVREG_MAX_CACHE_ENTRIES)
		return -EINVAL;

	for (i = 1; i + 1 < count; i += 2) {
		if (get_user(addr, list + i) || get_user(len, list + i + 1))
			return -EFAULT;

		if (!addr || !len)
			continue;

		/* Addresses are the caller's virtual addresses, as in the vendor. */
		__cpuc_flush_dcache_area((void *)(uintptr_t)addr, len);
	}

	return 0;
}

static long hidtvreg_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	switch (cmd) {
	case HIDTVREG_IOCTL_READ_PHYS:
		return hidtvreg_read_phys(arg);
	case HIDTVREG_IOCTL_CACHE:
		return hidtvreg_cache_list(arg);
	default:
		return -EINVAL;
	}
}

static int hidtvreg_mmap(struct file *file, struct vm_area_struct *vma)
{
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	vma->vm_flags |= VM_IO | VM_DONTEXPAND | VM_DONTDUMP;

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static const struct file_operations hidtvreg_fops = {
	.owner		= THIS_MODULE,
	.open		= hidtvreg_open,
	.release	= hidtvreg_release,
	.unlocked_ioctl	= hidtvreg_ioctl,
	.mmap		= hidtvreg_mmap,
};

static int __init hidtvreg_init(void)
{
	int ret;

	ret = register_chrdev(0, HIDTVREG_NAME, &hidtvreg_fops);
	if (ret < 0) {
		pr_err("hidtvreg: unable to register char device: %d\n", ret);
		return ret;
	}
	hidtvreg_major = ret;

	hidtvreg_class = class_create(THIS_MODULE, HIDTVREG_NAME);
	if (IS_ERR(hidtvreg_class)) {
		ret = PTR_ERR(hidtvreg_class);
		pr_err("hidtvreg: unable to create class: %d\n", ret);
		goto err_chrdev;
	}

	hidtvreg_device = device_create(hidtvreg_class, NULL,
					MKDEV(hidtvreg_major, 0), NULL,
					HIDTVREG_NAME);
	if (IS_ERR(hidtvreg_device)) {
		ret = PTR_ERR(hidtvreg_device);
		pr_err("hidtvreg: unable to create device: %d\n", ret);
		goto err_class;
	}

	pr_info("hidtvreg: init done Ver: 3.0 (major %u)\n", hidtvreg_major);

	return 0;

err_class:
	class_destroy(hidtvreg_class);
err_chrdev:
	unregister_chrdev(hidtvreg_major, HIDTVREG_NAME);
	return ret;
}

static void __exit hidtvreg_exit(void)
{
	device_destroy(hidtvreg_class, MKDEV(hidtvreg_major, 0));
	class_destroy(hidtvreg_class);
	unregister_chrdev(hidtvreg_major, HIDTVREG_NAME);
	pr_info("hidtvreg: cleanup done\n");
}

module_init(hidtvreg_init);
module_exit(hidtvreg_exit);

MODULE_DESCRIPTION("HiDTV register access driver");
MODULE_LICENSE("GPL");
