// SPDX-License-Identifier: GPL-2.0-only
/*
 * aw-config - expose a gzipped copy of the kernel configuration as
 * /proc/aw-config.gz, the file the vendor aw-config.ko provides.
 *
 * The vendor module carries its own 44 KB copy of the blob (aw_config_data)
 * and publishes it read-only (0444) through procfs.  Ours is regenerated from
 * the .config used for this build, so it always describes the running kernel
 * instead of whatever config the vendor shipped.
 */
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#ifndef AW_CONFIG_BLOB
#error "AW_CONFIG_BLOB must point at the gzipped .config"
#endif

asm(".section .rodata,\"a\"\n"
    "aw_config_blob:\n"
    ".incbin \"" AW_CONFIG_BLOB "\"\n"
    "aw_config_blob_end:\n"
    ".previous\n");

extern const char aw_config_blob[];
extern const char aw_config_blob_end[];

static ssize_t aw_config_read(struct file *file, char __user *buf,
			      size_t count, loff_t *ppos)
{
	return simple_read_from_buffer(buf, count, ppos, aw_config_blob,
				       aw_config_blob_end - aw_config_blob);
}

static const struct proc_ops aw_config_proc_ops = {
	.proc_read	= aw_config_read,
	.proc_lseek	= default_llseek,
};

static int __init aw_config_init(void)
{
	struct proc_dir_entry *entry;

	entry = proc_create("aw-config.gz", 0444, NULL, &aw_config_proc_ops);
	if (!entry)
		return -ENOMEM;

	proc_set_size(entry, aw_config_blob_end - aw_config_blob);
	pr_info("aw-config: %ld bytes at /proc/aw-config.gz\n",
		(long)(aw_config_blob_end - aw_config_blob));
	return 0;
}

static void __exit aw_config_exit(void)
{
	remove_proc_entry("aw-config.gz", NULL);
}

module_init(aw_config_init);
module_exit(aw_config_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Echo the kernel .config file really used to build the kernel");
MODULE_AUTHOR("Allwinner Technology");
