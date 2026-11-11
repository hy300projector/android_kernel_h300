/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * /dev/decd ABI, H726 (sun50iw15p1).
 *
 * The five command numbers and the argument-block layouts below are compiled
 * into the shipped H726 hwcomposer.hermes.so (DecoderDisplay, decompile
 * :31085-:31230). That binary is the authority for this ABI.
 *
 * No H726 decd driver survives in the ROM: no module or vmlinux symbol
 * mentions decd and the shipped config has CONFIG_TV_DECODER_DISPLAY unset.
 * The H713 mainline port (drivers/decd, GPL-2.0) is a structural analogue
 * only; its device-side behaviour is not evidence for this SoC.
 *
 * H726 submit block, read from present (00034930): +0 config pointer,
 * +8 fence pointer, +16 repeat count. The repeat is at +16, not at the
 * H713 header's arg1 (+24); the H726 client leaves +24 zero.
 */
#ifndef DECD_ABI_H
#define DECD_ABI_H

#if defined(__KERNEL__)
#include <linux/types.h>
typedef __u8 decd_u8;
typedef __u32 decd_u32;
typedef __s32 decd_s32;
typedef __u64 decd_u64;
typedef __s64 decd_s64;
#else
#include <stdint.h>
typedef uint8_t decd_u8;
typedef uint32_t decd_u32;
typedef int32_t decd_s32;
typedef uint64_t decd_u64;
typedef int64_t decd_s64;
#endif

/* _IOC(_IOC_WRITE, 'd' = 0x64, nr, size), as compiled into the library. */
#define DECD_IOC_PM_HINT 0x40046401u            /* nr 1,    4: enable latch  */
#define DECD_IOC_FRAME_SUBMIT 0x40706400u       /* nr 0,  112: submit        */
#define DECD_IOC_INTERLACE_SETUP 0x40706407u    /* nr 7,  112: top+bottom    */
#define DECD_IOC_STOP_VIDEO_STREAM 0x40046408u  /* nr 8,    4: stop          */
#define DECD_IOC_BYPASS_CONFIG 0x40046409u      /* nr 9,    4: bypass        */
#define DECD_IOC_GET_VSYNC_TS 0x8008640au       /* nr 10,   8: timestamp     */
#define DECD_IOC_MAP_LINEAR_BUFFER 0xc010640bu  /* nr 11,  16: phys -> iova  */
#define DECD_IOC_SET_OVERSCAN 0x4020640cu       /* nr 12,  32: pointer block */

#define DECD_IOCTL_HEADER_SIZE 32
#define DECD_FRAME_CONFIG_SIZE 112
#define DECD_OVERSCAN_SIZE 16
#define DECD_LINEAR_MAP_SIZE 16

/* Every command starts by copying this header from the ioctl argument, even
 * the commands whose _IOC size is 4 (stock dec_ioctl_file does the same). */
struct decd_ioctl_header {
  decd_u64 user_ptr;   /* +0  config pointer / enable / bypass value     */
  decd_u64 user_ptr2;  /* +8  fence output pointer for submit            */
  decd_u32 arg0;       /* +16 H726 submit repeat count                   */
  decd_u32 reserved0;  /* +20                                             */
  decd_u32 arg1;       /* +24 zero in the H726 client (H713 used it)     */
  decd_u32 reserved1;  /* +28                                             */
};

/* The 112 bytes DecoderDisplay::present hands to DEC_FRAME_SUBMIT. */
struct decd_frame_config {
  decd_u8 protected_content; /* +0  protected-content flag                */
  decd_u8 reserved0[3];      /* +1                                        */
  decd_s32 buffer_fd;        /* +4  dup() of the tunnel buffer dma-buf    */
  decd_s32 format_class;     /* +8  0, 1, 20 or 21 (21 is dropped)        */
  decd_s32 reserved1;        /* +12                                       */
  decd_s32 reserved2[6];     /* +16..+39                                  */
  decd_s32 picture_width;    /* +40                                       */
  decd_s32 picture_height;   /* +44                                       */
  decd_s32 half_width;       /* +48                                       */
  decd_s32 half_height;      /* +52                                       */
  decd_s32 half_width_2;     /* +56                                       */
  decd_s32 half_height_2;    /* +60                                       */
  decd_s32 scene;            /* +64                                       */
  decd_s32 scene_2;          /* +68                                       */
  decd_s32 scene_3;          /* +72                                       */
  decd_s32 private_fd;       /* +76  VideoInfo private buffer dma-buf     */
  decd_s32 private_size;     /* +80                                       */
  decd_s32 private_flag;     /* +84                                       */
  decd_s32 reserved3[2];     /* +88..+95                                  */
  decd_s32 interlace;        /* +96                                       */
  decd_s32 ones;             /* +100 always 1 in the binary               */
  decd_s32 reserved4[2];     /* +104..+111                                */
};

struct decd_interlace_pair {
  struct decd_frame_config top;
  struct decd_frame_config bottom;
};

/* DEC_SET_OVERSCAN passes a 32-byte block whose first word is a user pointer
 * to this 16-byte record (VideoInfo::queryOverscan fills 8 words; the HAL
 * passes the array address and the kernel logs it as "uptr"). */
struct decd_overscan {
  decd_s32 left;
  decd_s32 top;
  decd_s32 right;
  decd_s32 bottom;
};

struct decd_linear_map {
  decd_u32 phys;
  decd_u32 size;
  decd_u32 dma_addr;
  decd_u32 reserved;
};

struct decd_state {
  int enabled;
  decd_u32 bypass; /* raw low word from the client; no H726 bit mapping */
  int stopped;
  unsigned int submits;
  struct decd_overscan overscan;
};

struct decd_copy_ops {
  int (*from_user)(void *dst, const void *src, unsigned long n);
  int (*to_user)(void *dst, const void *src, unsigned long n);
};

struct decd_backend_ops {
  /* The stub returns 0 even when it drops a frame; fence_fd stays -1. */
  int (*submit)(void *ctx, const struct decd_frame_config *cfg, int repeat,
                int *fence_fd);
  int (*stop)(void *ctx);
  int (*pm_hint)(void *ctx, int on);
  int (*bypass)(void *ctx, decd_u32 value);
  int (*set_overscan)(void *ctx, const struct decd_overscan *overscan);
  int (*get_vsync_ts)(void *ctx, decd_u64 *ts);
  int (*map_linear)(void *ctx, const struct decd_linear_map *req,
                    struct decd_linear_map *out);
  int (*interlace)(void *ctx, const struct decd_frame_config *top,
                   const struct decd_frame_config *bottom);
};

/* Returns 0 or a negative errno. Unknown commands return 0 (stock logs and
 * succeeds). */
long decd_ioctl(struct decd_state *state, unsigned int cmd, void *arg,
                const struct decd_copy_ops *ops,
                const struct decd_backend_ops *backend, void *ctx);

/* Stock fmt_attr_tbl bpp class -> Y/C multiplier: 8bpc paths use one byte per
 * pixel, 10bpc paths two. Returns 0 for ids the stock driver rejects. */
int decd_format_multiplier(decd_s32 format_class);

#define DECD_ASSERT(name, cond) typedef char decd_assert_##name[(cond) ? 1 : -1]
DECD_ASSERT(header_size, sizeof(struct decd_ioctl_header) == DECD_IOCTL_HEADER_SIZE);
DECD_ASSERT(config_size, sizeof(struct decd_frame_config) == DECD_FRAME_CONFIG_SIZE);
DECD_ASSERT(overscan_size, sizeof(struct decd_overscan) == DECD_OVERSCAN_SIZE);
DECD_ASSERT(linear_map_size, sizeof(struct decd_linear_map) == DECD_LINEAR_MAP_SIZE);
DECD_ASSERT(buffer_fd_at_4, __builtin_offsetof(struct decd_frame_config, buffer_fd) == 4);
DECD_ASSERT(format_at_8, __builtin_offsetof(struct decd_frame_config, format_class) == 8);
DECD_ASSERT(width_at_40, __builtin_offsetof(struct decd_frame_config, picture_width) == 40);
DECD_ASSERT(private_fd_at_76, __builtin_offsetof(struct decd_frame_config, private_fd) == 76);
DECD_ASSERT(interlace_at_96, __builtin_offsetof(struct decd_frame_config, interlace) == 96);
#undef DECD_ASSERT

#endif /* DECD_ABI_H */
