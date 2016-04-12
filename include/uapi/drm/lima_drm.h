/* SPDX-License-Identifier: (GPL-2.0 WITH Linux-syscall-note) OR MIT */
/* Copyright 2017-2018 Qiang Yu <yuq825@gmail.com> */

#ifndef __LIMA_DRM_H__
#define __LIMA_DRM_H__

#include "drm.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define LIMA_INFO_GPU_MALI400 0x00
#define LIMA_INFO_GPU_MALI450 0x01

struct drm_lima_info {
	__u32 gpu_id;   /* out */
	__u32 num_pp;   /* out */
	__u32 valid;    /* out */
	__u32 _resv[7];
};

struct drm_lima_gem_create {
	__u32 size;    /* in */
	__u32 flags;   /* in */
	__u32 handle;  /* out */
	__u32 pad;
};

struct drm_lima_gem_info {
	__u32 handle;  /* in */
	__u32 va;      /* out */
	__u64 offset;  /* out */
};

#define LIMA_SUBMIT_BO_READ   0x01
#define LIMA_SUBMIT_BO_WRITE  0x02

struct drm_lima_gem_submit_bo {
	__u32 handle;  /* in */
	__u32 flags;   /* in */
};

#define LIMA_GP_FRAME_REG_NUM 6

struct drm_lima_gp_frame {
	__u32 frame[LIMA_GP_FRAME_REG_NUM];
};

#define LIMA_PP_FRAME_REG_NUM 23
#define LIMA_PP_WB_REG_NUM 12

struct drm_lima_m400_pp_frame {
	__u32 frame[LIMA_PP_FRAME_REG_NUM];
	__u32 num_pp;
	__u32 wb[3 * LIMA_PP_WB_REG_NUM];
	__u32 plbu_array_address[4];
	__u32 fragment_stack_address[4];
};

struct drm_lima_m450_pp_frame {
	__u32 frame[LIMA_PP_FRAME_REG_NUM];
	__u32 num_pp;
	__u32 wb[3 * LIMA_PP_WB_REG_NUM];
	__u32 use_dlbu;
	__u32 _pad;
	union {
		__u32 plbu_array_address[8];
		__u32 dlbu_regs[4];
	};
	__u32 fragment_stack_address[8];
};

#define LIMA_PIPE_GP  0x00
#define LIMA_PIPE_PP  0x01

#define LIMA_SUBMIT_FLAG_EXPLICIT_FENCE (1 << 0)

struct drm_lima_gem_submit {
	__u32 ctx;         /* in */
	__u32 pipe;        /* in */
	__u32 nr_bos;      /* in */
	__u32 frame_size;  /* in */
	__u64 bos;         /* in */
	__u64 frame;       /* in */
	__u32 flags;       /* in */
	__u32 out_sync;    /* in */
	__u32 in_sync[2];  /* in */
};

#define LIMA_GEM_WAIT_READ   0x01
#define LIMA_GEM_WAIT_WRITE  0x02

struct drm_lima_gem_wait {
	__u32 handle;      /* in */
	__u32 op;          /* in */
	__s64 timeout_ns;  /* in */
};

#define LIMA_CTX_OP_CREATE 1
#define LIMA_CTX_OP_FREE   2

struct drm_lima_ctx {
	__u32 op;          /* in */
	__u32 id;          /* in/out */
};

#define DRM_LIMA_INFO        0x00
#define DRM_LIMA_GEM_CREATE  0x01
#define DRM_LIMA_GEM_INFO    0x02
#define DRM_LIMA_GEM_SUBMIT  0x03
#define DRM_LIMA_GEM_WAIT    0x04
#define DRM_LIMA_CTX         0x05

#define DRM_IOCTL_LIMA_INFO DRM_IOR(DRM_COMMAND_BASE + DRM_LIMA_INFO, struct drm_lima_info)
#define DRM_IOCTL_LIMA_GEM_CREATE DRM_IOWR(DRM_COMMAND_BASE + DRM_LIMA_GEM_CREATE, struct drm_lima_gem_create)
#define DRM_IOCTL_LIMA_GEM_INFO DRM_IOWR(DRM_COMMAND_BASE + DRM_LIMA_GEM_INFO, struct drm_lima_gem_info)
#define DRM_IOCTL_LIMA_GEM_SUBMIT DRM_IOW(DRM_COMMAND_BASE + DRM_LIMA_GEM_SUBMIT, struct drm_lima_gem_submit)
#define DRM_IOCTL_LIMA_GEM_WAIT DRM_IOW(DRM_COMMAND_BASE + DRM_LIMA_GEM_WAIT, struct drm_lima_gem_wait)
#define DRM_IOCTL_LIMA_CTX DRM_IOWR(DRM_COMMAND_BASE + DRM_LIMA_CTX, struct drm_lima_ctx)

#if defined(__cplusplus)
}
#endif

#endif /* __LIMA_DRM_H__ */
