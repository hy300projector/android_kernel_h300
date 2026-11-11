// SPDX-License-Identifier: GPL-2.0-only
/*
 * /dev/decd ioctl dispatch for the H726 client contract.
 *
 * The wire layouts are taken from the shipped hwcomposer.hermes.so; the
 * device-side actions are a stub because no H726 decd driver exists. The H713
 * port is a structural cross-check only, not proof for this SoC.
 *
 * Client-proven details:
 *   - submit: 32-byte block, config at +0, fence at +8, repeat at +16;
 *   - enable and bypass put their value in the first word;
 *   - stop sends 32 zero bytes;
 *   - overscan's first word points at 16 bytes of signed LTRB;
 *   - success returns 0.
 */
#include "decd_abi.h"

#define DECD_EFAULT 14
#define DECD_ENOTSUPP 95

static int decd_copy_in(const struct decd_copy_ops *ops, void *dst,
                        decd_u64 src, unsigned long n) {
  if (ops == 0 || ops->from_user == 0 || src == 0) {
    return -DECD_EFAULT;
  }
  return ops->from_user(dst, (const void *)(unsigned long)src, n) ? -DECD_EFAULT
                                                                 : 0;
}

static int decd_copy_out(const struct decd_copy_ops *ops, decd_u64 dst,
                         const void *src, unsigned long n) {
  if (ops == 0 || ops->to_user == 0 || dst == 0) {
    return -DECD_EFAULT;
  }
  return ops->to_user((void *)(unsigned long)dst, src, n) ? -DECD_EFAULT : 0;
}

/* The H726 client supplies 32 bytes for submit, enable, stop and overscan,
 * but only 16 for bypass (bypassConfig's v6[4]). Only submit needs the word
 * at +16, so every other command copies the 16-byte prefix the client owns. */
static unsigned long decd_header_size(unsigned int cmd) {
  return cmd == DECD_IOC_FRAME_SUBMIT ? DECD_IOCTL_HEADER_SIZE : 16;
}

/* H713 fmt table analogue; used only for the stub's drop policy. */
int decd_format_multiplier(decd_s32 format_class) {
  switch (format_class) {
    case 0:
    case 2:
    case 4:
    case 6:
      return 1;
    case 1:
    case 3:
    case 5:
    case 20:
      return 2;
    default:
      return 0;
  }
}

long decd_ioctl(struct decd_state *state, unsigned int cmd, void *arg,
                const struct decd_copy_ops *ops,
                const struct decd_backend_ops *backend, void *ctx) {
  struct decd_ioctl_header hdr;
  int ret;

  if (state == 0 || arg == 0) {
    return -DECD_EFAULT;
  }
  ret = decd_copy_in(ops, &hdr, (decd_u64)(unsigned long)arg,
                     decd_header_size(cmd));
  if (ret != 0) {
    return ret;
  }

  switch (cmd) {
    case DECD_IOC_FRAME_SUBMIT: {
      struct decd_frame_config cfg;
      int fence_fd = -1;

      ret = decd_copy_in(ops, &cfg, hdr.user_ptr, sizeof cfg);
      if (ret != 0) {
        return ret;
      }
      if (backend == 0 || backend->submit == 0) {
        return -DECD_ENOTSUPP;
      }
      /* H726 present writes repeat at +16 (arg0), not the H713 arg1. */
      ret = backend->submit(ctx, &cfg, (int)hdr.arg0, &fence_fd);
      if (ret != 0) {
        return ret;
      }
      ++state->submits;
      return decd_copy_out(ops, hdr.user_ptr2, &fence_fd, sizeof fence_fd);
    }

    case DECD_IOC_PM_HINT: {
      /* The client writes a 32-bit value at +0; non-zero means enable. The
       * H713 runtime-PM action is not reproduced by this stub. */
      int on = hdr.user_ptr != 0;

      if (backend != 0 && backend->pm_hint != 0) {
        ret = backend->pm_hint(ctx, on);
        if (ret != 0) {
          return ret;
        }
      }
      state->enabled = on;
      return 0;
    }

    case DECD_IOC_STOP_VIDEO_STREAM:
      /* The client sends 32 zero bytes; no payload is read. */
      if (backend != 0 && backend->stop != 0) {
        ret = backend->stop(ctx);
        if (ret != 0) {
          return ret;
        }
      }
      state->stopped = 1;
      return 0;

    case DECD_IOC_BYPASS_CONFIG: {
      decd_u32 value = (decd_u32)hdr.user_ptr;

      if (backend != 0 && backend->bypass != 0) {
        ret = backend->bypass(ctx, value);
        if (ret != 0) {
          return ret;
        }
      }
      /* No H726 driver defines the register mapping; keep the raw value. */
      state->bypass = value;
      return 0;
    }

    case DECD_IOC_GET_VSYNC_TS: {
      decd_u64 ts = 0;

      if (backend == 0 || backend->get_vsync_ts == 0) {
        return -DECD_ENOTSUPP;
      }
      ret = backend->get_vsync_ts(ctx, &ts);
      if (ret != 0) {
        return ret;
      }
      return decd_copy_out(ops, hdr.user_ptr, &ts, sizeof ts);
    }

    case DECD_IOC_MAP_LINEAR_BUFFER: {
      struct decd_linear_map req;
      struct decd_linear_map out;

      ret = decd_copy_in(ops, &req, hdr.user_ptr, sizeof req);
      if (ret != 0) {
        return ret;
      }
      if (backend == 0 || backend->map_linear == 0) {
        return -DECD_ENOTSUPP;
      }
      out = req;
      ret = backend->map_linear(ctx, &req, &out);
      if (ret != 0) {
        return ret;
      }
      return decd_copy_out(ops, hdr.user_ptr, &out, sizeof out);
    }

    case DECD_IOC_INTERLACE_SETUP: {
      struct decd_interlace_pair pair;

      ret = decd_copy_in(ops, &pair, hdr.user_ptr, sizeof pair);
      if (ret != 0) {
        return ret;
      }
      if (backend == 0 || backend->interlace == 0) {
        return -DECD_ENOTSUPP;
      }
      return backend->interlace(ctx, &pair.top, &pair.bottom);
    }

    case DECD_IOC_SET_OVERSCAN: {
      struct decd_overscan overscan;
      decd_u32 ptr = (decd_u32)hdr.user_ptr;

      ret = decd_copy_in(ops, &overscan, ptr, sizeof overscan);
      if (ret != 0) {
        return ret;
      }
      /* No H726 driver defines LTRB validation, so negative values are
       * accepted and passed through rather than inventing an H713 policy. */
      if (backend != 0 && backend->set_overscan != 0) {
        ret = backend->set_overscan(ctx, &overscan);
        if (ret != 0) {
          return ret;
        }
      }
      state->overscan = overscan;
      return 0;
    }

    default:
      /* The H726 client never issues an unknown command; return 0 as a
       * stub policy, not as proven device behaviour. */
      return 0;
  }
}
