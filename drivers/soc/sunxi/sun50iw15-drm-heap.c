// SPDX-License-Identifier: GPL-2.0-only
/*
 * Allwinner H726/HY300 DRM heap: /dev/sunxi_drm_heap.
 *
 * Replacement for the vendor's in-tree bsp/drivers/drm_heap/drm_heap.c
 * (vmlinux symbols sunxi_drm_heap_{ioctl_allocate,ioctl_phys,callback_free,
 * probe,remove}), recovered from h726_extracted/drm_secure.c and the HAL side
 * (hal_rewrite/gralloc.hermes, hal_rewrite/hwcomposer.hermes/drm_heap.h).
 *
 * The device is a gen_pool carveout exported as a dma-buf.  One ioctl is used
 * by userspace, and it is reproduced exactly:
 *
 *   ioctl 0xC0184800  _IOWR('H', 0, 24) with
 *       struct { __u64 size; __s32 fd; __u32 flags; __u32 r0; __u32 r1; }
 *
 *     size    requested bytes; the vendor requires page alignment and logs
 *             "size 0x%zx is not page aligned" otherwise;
 *     flags   the clients send 0x80002 (gralloc's kSecureHeapFlags and the
 *             composer's constant); 0 is accepted too, anything else is
 *             rejected like the vendor's heap_flags check;
 *     fd      output: the dma-buf descriptor, written back at offset 8.
 *
 * The vendor takes the carveout from OP-TEE (optee_probe_drm_configure) and
 * gates allocations on a TEE session.  There is no TEE on this tree, so the
 * region comes from either an "allwinner,sunxi-drm-heap" node with a reg
 * property, or the module parameters below.  Without a region the char device
 * still exists (so opens succeed and the failure is visible), and allocation
 * returns -ENOSPC.
 *
 * Differences from the vendor, all deliberate:
 *  - no TEE session and no secure/display MPU programming; buffers are only as
 *    protected as the carveout itself;
 *  - freeing happens in the dma-buf release callback, which the dma-buf core
 *    only runs once the last descriptor and mapping are gone, so the vendor's
 *    explicit "buffer still mapped in the kernel" refusal is unnecessary;
 *  - sunxi_drm_heap_ioctl_phys is not implemented: its command number was not
 *    recoverable (gap C48), and nothing in the recovered HAL paths calls it.
 */

#include <linux/dma-buf.h>
#include <linux/fs.h>
#include <linux/genalloc.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include <asm/pgtable.h>

#define SUNXI_DRM_HEAP_NAME		"sunxi_drm_heap"

/* gralloc's kSecureHeapFlags and the composer's constant. */
#define SUNXI_DRM_HEAP_FLAGS		0x80002

/* Page-aligned requests only, as the vendor enforced. */
#define SUNXI_DRM_HEAP_MIN_SHIFT	PAGE_SHIFT

struct sunxi_drm_heap_alloc {
	__u64 size;
	__s32 fd;
	__u32 flags;
	__u32 reserved0;
	__u32 reserved1;
};

#define SUNXI_DRM_HEAP_IOC_ALLOC \
	_IOWR('H', 0, struct sunxi_drm_heap_alloc)

static_assert(sizeof(struct sunxi_drm_heap_alloc) == 24);

struct sunxi_drm_heap_buf {
	struct gen_pool *pool;
	unsigned long offset;
	phys_addr_t phys;
	size_t size;
};

struct sunxi_drm_heap {
	struct miscdevice misc;
	struct gen_pool *pool;
	void *vaddr;
	phys_addr_t phys;
	size_t size;
};

/* Physical base/size when the device tree has no node.  Zero disables the pool. */
static unsigned long long heap_base;
static unsigned long long heap_size;
module_param(heap_base, ullong, 0444);
MODULE_PARM_DESC(heap_base, "Physical base of the DRM heap carveout");
module_param(heap_size, ullong, 0444);
MODULE_PARM_DESC(heap_size, "Size in bytes of the DRM heap carveout");

static int sunxi_drm_heap_attach(struct dma_buf *dmabuf,
				 struct dma_buf_attachment *attachment)
{
	return 0;
}

static void sunxi_drm_heap_detach(struct dma_buf *dmabuf,
				  struct dma_buf_attachment *attachment)
{
}

static void sunxi_drm_heap_dmabuf_release(struct dma_buf *dmabuf)
{
	struct sunxi_drm_heap_buf *buf = dmabuf->priv;

	gen_pool_free(buf->pool, buf->offset, buf->size);
	kfree(buf);
}

static int sunxi_drm_heap_mmap(struct dma_buf *dmabuf,
			       struct vm_area_struct *vma)
{
	struct sunxi_drm_heap_buf *buf = dmabuf->priv;
	unsigned long size = vma->vm_end - vma->vm_start;
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	phys_addr_t phys;

	if (off + size > buf->size)
		return -EINVAL;

	phys = buf->phys + off;

	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	vma->vm_flags |= VM_IO | VM_DONTEXPAND | VM_DONTDUMP;

	if (remap_pfn_range(vma, vma->vm_start, phys >> PAGE_SHIFT, size,
			    vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static const struct dma_buf_ops sunxi_drm_heap_dmabuf_ops = {
	.attach		= sunxi_drm_heap_attach,
	.detach		= sunxi_drm_heap_detach,
	.mmap		= sunxi_drm_heap_mmap,
	.release	= sunxi_drm_heap_dmabuf_release,
};

static int sunxi_drm_heap_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;

	file->private_data = container_of(misc, struct sunxi_drm_heap, misc);

	return 0;
}

static long sunxi_drm_heap_allocate(struct sunxi_drm_heap *heap,
				    unsigned long arg)
{
	struct sunxi_drm_heap_alloc req;
	struct sunxi_drm_heap_buf *buf;
	struct dma_buf *dmabuf;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	unsigned long offset;
	int fd;

	if (copy_from_user(&req, (void __user *)arg, sizeof(req)))
		return -EFAULT;

	if (!req.size || (req.size & (PAGE_SIZE - 1)))
		return -EINVAL;

	if (req.flags != SUNXI_DRM_HEAP_FLAGS && req.flags != 0)
		return -EINVAL;

	if (!heap->pool) {
		dev_warn_ratelimited(heap->misc.this_device,
				     "no carveout configured (heap_base/heap_size or a reg property), cannot allocate 0x%llx bytes\n",
				     req.size);
		return -ENOSPC;
	}

	if (req.size > heap->size)
		return -ENOSPC;

	offset = gen_pool_alloc(heap->pool, req.size);
	if (!offset)
		return -ENOSPC;

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf) {
		gen_pool_free(heap->pool, offset, req.size);
		return -ENOMEM;
	}

	buf->pool = heap->pool;
	buf->offset = offset;
	buf->size = req.size;
	buf->phys = gen_pool_virt_to_phys(heap->pool, offset);

	exp_info.ops = &sunxi_drm_heap_dmabuf_ops;
	exp_info.size = buf->size;
	exp_info.flags = O_RDWR;
	exp_info.priv = buf;

	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(dmabuf)) {
		gen_pool_free(heap->pool, offset, req.size);
		kfree(buf);
		return PTR_ERR(dmabuf);
	}

	fd = dma_buf_fd(dmabuf, O_CLOEXEC);
	if (fd < 0) {
		dma_buf_put(dmabuf);
		return fd;
	}

	req.fd = fd;

	if (copy_to_user((void __user *)arg, &req, sizeof(req))) {
		/*
		 * The descriptor is already installed; closing it through the
		 * dma-buf release path would need the fd table, and userspace
		 * never learned about it, so drop the reference we hold.
		 */
		dma_buf_put(dmabuf);
		return -EFAULT;
	}

	return 0;
}

static long sunxi_drm_heap_ioctl(struct file *file, unsigned int cmd,
				 unsigned long arg)
{
	struct sunxi_drm_heap *heap = file->private_data;

	switch (cmd) {
	case SUNXI_DRM_HEAP_IOC_ALLOC:
		return sunxi_drm_heap_allocate(heap, arg);
	default:
		return -ENOTTY;
	}
}

static const struct file_operations sunxi_drm_heap_fops = {
	.owner		= THIS_MODULE,
	.open		= sunxi_drm_heap_open,
	.unlocked_ioctl	= sunxi_drm_heap_ioctl,
	.compat_ioctl	= compat_ptr_ioctl,
	.llseek		= noop_llseek,
};

static struct sunxi_drm_heap sunxi_drm_heap = {
	.misc = {
		.minor	= MISC_DYNAMIC_MINOR,
		.name	= SUNXI_DRM_HEAP_NAME,
		.fops	= &sunxi_drm_heap_fops,
		.mode	= 0666,
	},
};

/*
 * The carveout: a "memory-region" phandle to the reserved-memory node is the
 * normal form, but a plain reg on the heap node works too.
 */
static void sunxi_drm_heap_of_region(struct device_node *np)
{
	struct device_node *mem;
	struct resource res;

	if (!np)
		return;

	mem = of_parse_phandle(np, "memory-region", 0);
	if (mem) {
		if (!of_address_to_resource(mem, 0, &res)) {
			sunxi_drm_heap.phys = res.start;
			sunxi_drm_heap.size = resource_size(&res);
		}
		of_node_put(mem);
		return;
	}

	if (of_address_to_resource(np, 0, &res))
		return;

	sunxi_drm_heap.phys = res.start;
	sunxi_drm_heap.size = resource_size(&res);
}


/*
 * Startup walk of the path that can be exercised from the kernel: take a range
 * from the pool, write and read it through the carveout mapping, export it to a
 * dma-buf, drop the reference (the release callback returns the range) and take
 * it again.  The userspace fd/mmap half is walked by
 * tools/sunxi_drm_heap_test.c on the device.
 */
static int sunxi_drm_heap_selftest(struct sunxi_drm_heap *heap)
{
	const size_t size = 64 * 1024;
	struct sunxi_drm_heap_buf *buf;
	struct dma_buf *dmabuf;
	DEFINE_DMA_BUF_EXPORT_INFO(exp_info);
	phys_addr_t phys;
	unsigned long first, second;
	u32 *p;
	int i, ret = 0;

	first = gen_pool_alloc(heap->pool, size);
	if (!first)
		return -ENOSPC;

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf) {
		gen_pool_free(heap->pool, first, size);
		return -ENOMEM;
	}

	buf->pool = heap->pool;
	buf->offset = first;
	buf->size = size;
	buf->phys = gen_pool_virt_to_phys(heap->pool, first);
	phys = buf->phys;

	p = (u32 *)first;
	for (i = 0; i < size / 4; i++)
		p[i] = 0xa5a50000u | i;
	/* Drain the stores before reading the carveout back. */
	mb();
	for (i = 0; i < size / 4; i++) {
		if (p[i] != (0xa5a50000u | i)) {
			ret = -EIO;
			break;
		}
	}

	if (ret)
		pr_err("sunxi_drm_heap: selftest readback failed at word %d\n", i);

	exp_info.ops = &sunxi_drm_heap_dmabuf_ops;
	exp_info.size = size;
	exp_info.flags = O_RDWR;
	exp_info.priv = buf;

	dmabuf = dma_buf_export(&exp_info);
	if (IS_ERR(dmabuf)) {
		gen_pool_free(heap->pool, first, size);
		kfree(buf);
		pr_err("sunxi_drm_heap: selftest export failed: %ld\n",
		       PTR_ERR(dmabuf));
		return PTR_ERR(dmabuf);
	}

	dma_buf_put(dmabuf);

	second = gen_pool_alloc(heap->pool, size);
	if (!second) {
		pr_err("sunxi_drm_heap: selftest lost the range after free\n");
		return -ENOSPC;
	}
	if (second != first) {
		pr_err("sunxi_drm_heap: selftest range not reused (0x%lx vs 0x%lx)\n",
		       first, second);
		ret = -EIO;
	}
	gen_pool_free(heap->pool, second, size);

	if (!ret)
		pr_info("sunxi_drm_heap: selftest ok, 64 KiB at phys 0x%llx\n",
			(u64)phys);

	return ret;
}

static int __init sunxi_drm_heap_init(void)
{
	struct device_node *np;
	int ret;

	np = of_find_compatible_node(NULL, NULL, "allwinner,sunxi-drm-heap");
	sunxi_drm_heap_of_region(np);
	of_node_put(np);

	if (!sunxi_drm_heap.size && heap_size) {
		sunxi_drm_heap.phys = heap_base;
		sunxi_drm_heap.size = heap_size;
	}

	if (sunxi_drm_heap.size) {
		sunxi_drm_heap.vaddr = memremap(sunxi_drm_heap.phys,
						sunxi_drm_heap.size,
						MEMREMAP_WC);
		if (!sunxi_drm_heap.vaddr) {
			pr_err("sunxi_drm_heap: cannot map 0x%llx+0x%zx\n",
			       (u64)sunxi_drm_heap.phys, sunxi_drm_heap.size);
			sunxi_drm_heap.size = 0;
		}
	}

	if (sunxi_drm_heap.size) {
		sunxi_drm_heap.pool = gen_pool_create(SUNXI_DRM_HEAP_MIN_SHIFT,
						      NUMA_NO_NODE);
		if (!sunxi_drm_heap.pool) {
			pr_err("sunxi_drm_heap: cannot create the pool\n");
			memunmap(sunxi_drm_heap.vaddr);
			sunxi_drm_heap.size = 0;
		} else if (gen_pool_add_virt(sunxi_drm_heap.pool,
					     (unsigned long)sunxi_drm_heap.vaddr,
					     sunxi_drm_heap.phys,
					     sunxi_drm_heap.size,
					     NUMA_NO_NODE)) {
			pr_err("sunxi_drm_heap: cannot add the carveout\n");
			gen_pool_destroy(sunxi_drm_heap.pool);
			sunxi_drm_heap.pool = NULL;
			memunmap(sunxi_drm_heap.vaddr);
			sunxi_drm_heap.size = 0;
		}
	}

	if (sunxi_drm_heap.pool)
		sunxi_drm_heap_selftest(&sunxi_drm_heap);

	ret = misc_register(&sunxi_drm_heap.misc);
	if (ret) {
		pr_err("sunxi_drm_heap: cannot register the device: %d\n", ret);
		if (sunxi_drm_heap.pool)
			gen_pool_destroy(sunxi_drm_heap.pool);
		if (sunxi_drm_heap.vaddr)
			memunmap(sunxi_drm_heap.vaddr);
		return ret;
	}

	if (sunxi_drm_heap.pool)
		pr_info("sunxi_drm_heap: /dev/%s ready, phys 0x%llx, size 0x%zx\n",
			SUNXI_DRM_HEAP_NAME, (u64)sunxi_drm_heap.phys,
			sunxi_drm_heap.size);
	else
		pr_warn("sunxi_drm_heap: /dev/%s registered without a carveout; allocations will fail with ENOSPC\n",
			SUNXI_DRM_HEAP_NAME);

	return 0;
}

static void __exit sunxi_drm_heap_exit(void)
{
	misc_deregister(&sunxi_drm_heap.misc);

	if (sunxi_drm_heap.pool)
		gen_pool_destroy(sunxi_drm_heap.pool);
	if (sunxi_drm_heap.vaddr)
		memunmap(sunxi_drm_heap.vaddr);
}

module_init(sunxi_drm_heap_init);
module_exit(sunxi_drm_heap_exit);

MODULE_DESCRIPTION("Allwinner sun50iw15 DRM heap (/dev/sunxi_drm_heap)");
MODULE_LICENSE("GPL");
