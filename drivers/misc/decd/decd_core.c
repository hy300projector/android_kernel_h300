// SPDX-License-Identifier: GPL-2.0-only
/*
 * /dev/decd reconstruction for the H726 (sun50iw15p1) projector.
 *
 * There is no stock decd.ko in the shipped ROM: none of the 87 modules under
 * vendor_boot/vendor_dlkm contains "decd", vmlinux.nm has no decd symbol, and
 * the shipped config carries "# CONFIG_TV_DECODER_DISPLAY is not set". The
 * userspace contract therefore comes from the stock hwcomposer.hermes.so
 * (DecoderDisplay, :31021-:31247), which is the authority for this ABI. The
 * public H713 port of decd.ko (drivers/decd, GPL-2.0) is a structural
 * analogue only; it is not evidence for H726 register or queue behaviour.
 *
 * This module makes the node exist and implements that contract. It does not
 * program the AFBD/display decoder: the H726 register layout is owned by the
 * reconstructed vs-display driver and the stock decd AFBD block has not been
 * located in this SoC. Frames are accepted, validated and completed with an
 * already-signalled sync_file fence, so the userspace commit rotation works.
 */
#include <linux/dma-buf.h>
#include <linux/dma-fence.h>
#include <linux/fs.h>
#include <linux/ktime.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sync_file.h>
#include <linux/uaccess.h>

#include "decd_abi.h"

#define DECD_NAME "decd"

struct decd_fence {
  struct dma_fence base;
};

static spinlock_t decd_fence_lock;
static u64 decd_fence_context;
static u32 decd_fence_seqno;
static struct decd_state decd_state;

static const char *decd_fence_get_driver_name(struct dma_fence *fence) {
  return DECD_NAME;
}

static const char *decd_fence_get_timeline_name(struct dma_fence *fence) {
  return DECD_NAME;
}

static const struct dma_fence_ops decd_fence_ops = {
    .get_driver_name = decd_fence_get_driver_name,
    .get_timeline_name = decd_fence_get_timeline_name,
};

/* The stub creates a sync_file fence and signals it immediately. The stock
 * H713 driver signals at vsync; without the H726 AFBD block there is no vsync
 * to wait for, and this timing is not H726-proven. */
static int decd_fence_create(void) {
  struct decd_fence *fence;
  struct sync_file *sync;
  unsigned long flags;
  int fd;

  fence = kzalloc(sizeof(*fence), GFP_KERNEL);
  if (fence == 0) {
    return -1;
  }
  spin_lock_irqsave(&decd_fence_lock, flags);
  dma_fence_init(&fence->base, &decd_fence_ops, &decd_fence_lock,
                 decd_fence_context, ++decd_fence_seqno);
  spin_unlock_irqrestore(&decd_fence_lock, flags);
  dma_fence_signal(&fence->base);

  fd = get_unused_fd_flags(O_CLOEXEC);
  if (fd < 0) {
    dma_fence_put(&fence->base);
    return -1;
  }
  sync = sync_file_create(&fence->base);
  dma_fence_put(&fence->base);
  if (sync == 0) {
    put_unused_fd(fd);
    return -1;
  }
  fd_install(fd, sync->file);
  return fd;
}

static int decd_backend_submit(void *ctx, const struct decd_frame_config *cfg,
                               int repeat, int *fence_fd) {
  struct dma_buf *buffer;

  (void)ctx;
  (void)repeat; /* stock enqueues the same item `repeat` times */
  *fence_fd = -1;
  if (cfg == 0) {
    return 0;
  }
  /* Stub policy modelled on the H713 frame_item_create, not proven for H726:
   * an unsupported class is dropped with fence -1 and submit still returns 0. */
  if (decd_format_multiplier(cfg->format_class) == 0) {
    pr_debug("decd: format class %d dropped\n", cfg->format_class);
    return 0;
  }
  if (cfg->buffer_fd < 0) {
    return 0;
  }
  /* The client passes a dup() of the tunnel dma-buf. Validating it is a stub
   * policy; the H726 driver's map path is not in the ROM. */
  buffer = dma_buf_get(cfg->buffer_fd);
  if (IS_ERR(buffer)) {
    pr_debug("decd: buffer fd %d is not a dma-buf\n", cfg->buffer_fd);
    return 0;
  }
  dma_buf_put(buffer);
  *fence_fd = decd_fence_create();
  return 0;
}

static int decd_backend_stop(void *ctx) {
  (void)ctx;
  pr_debug("decd: stream stopped\n");
  return 0;
}

static int decd_backend_pm_hint(void *ctx, int on) {
  (void)ctx;
  pr_debug("decd: %s\n", on ? "enabled" : "disabled");
  return 0;
}

static int decd_backend_bypass(void *ctx, decd_u32 value) {
  (void)ctx;
  pr_debug("decd: bypass value %u\n", value);
  return 0;
}

static int decd_backend_set_overscan(void *ctx,
                                     const struct decd_overscan *overscan) {
  (void)ctx;
  pr_debug("decd: overscan %d %d %d %d\n", overscan->left, overscan->top,
           overscan->right, overscan->bottom);
  return 0;
}

static int decd_backend_get_vsync_ts(void *ctx, decd_u64 *ts) {
  (void)ctx;
  /* No AFBD interrupt is connected; report now so callers do not stall. */
  *ts = ktime_get_ns();
  return 0;
}

static int decd_backend_map_linear(void *ctx,
                                   const struct decd_linear_map *req,
                                   struct decd_linear_map *out) {
  (void)ctx;
  (void)req;
  (void)out;
  /* Stock exports a dma-buf over the physical range through decd_reserved.
   * That reserved pool has not been located on the H726, so fail loudly
   * instead of handing out an unusable address. */
  return -EOPNOTSUPP;
}

static int decd_backend_interlace(void *ctx,
                                  const struct decd_frame_config *top,
                                  const struct decd_frame_config *bottom) {
  struct decd_frame_config cfg;
  int fence = -1;

  (void)bottom;
  cfg = *top;
  return decd_backend_submit(ctx, &cfg, 1, &fence);
}

static const struct decd_backend_ops decd_backend = {
    .submit = decd_backend_submit,
    .stop = decd_backend_stop,
    .pm_hint = decd_backend_pm_hint,
    .bypass = decd_backend_bypass,
    .set_overscan = decd_backend_set_overscan,
    .get_vsync_ts = decd_backend_get_vsync_ts,
    .map_linear = decd_backend_map_linear,
    .interlace = decd_backend_interlace,
};

static int decd_from_user(void *dst, const void *src, unsigned long n) {
  return copy_from_user(dst, src, n) ? -1 : 0;
}

static int decd_to_user(void *dst, const void *src, unsigned long n) {
  return copy_to_user(dst, src, n) ? -1 : 0;
}

static const struct decd_copy_ops decd_copy = {
    .from_user = decd_from_user,
    .to_user = decd_to_user,
};

static long decd_ioctl_file(struct file *file, unsigned int cmd,
                            unsigned long arg) {
  (void)file;
  return decd_ioctl(&decd_state, cmd, (void *)arg, &decd_copy, &decd_backend,
                    NULL);
}

static int decd_open(struct inode *inode, struct file *file) {
  (void)inode;
  file->private_data = &decd_state;
  return 0;
}

static const struct file_operations decd_fops = {
    .owner = THIS_MODULE,
    .open = decd_open,
    .unlocked_ioctl = decd_ioctl_file,
    .llseek = noop_llseek,
};

static struct miscdevice decd_misc = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = DECD_NAME,
    .fops = &decd_fops,
    .mode = 0660,
};

static int __init decd_init(void) {
  spin_lock_init(&decd_fence_lock);
  decd_fence_context = dma_fence_context_alloc(1);
  return misc_register(&decd_misc);
}

static void __exit decd_exit(void) { misc_deregister(&decd_misc); }

module_init(decd_init);
module_exit(decd_exit);

MODULE_DESCRIPTION("H726 /dev/decd ABI reconstruction (hardware-stubbed)");
MODULE_LICENSE("GPL");
