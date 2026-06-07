/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * WonderMedia WM8505 DRM/KMS Userspace ABI
 *
 * Copyright (C) 2026 Logan Russell <me@lrussell.net>
 */

#ifndef _UAPI_WMT_DRM_H_
#define _UAPI_WMT_DRM_H_

#include "drm.h"

#if defined(__cplusplus)
extern "C" {
#endif

/* Limits */
#define WMT_GE_MAX_DIM		2048
#define WMT_GE_MAX_OPS		1024
#define WMT_GE_WAIT_MAX_US	1000000

/* OP Types */
#define WMT_GE_OP_FILL		0x1
#define WMT_GE_OP_BLIT		0x2
#define WMT_GE_OP_CONVERT	0x3

/* ROP Codes */
#define WMT_GE_ROP_PAT_XOR	0x5a
#define WMT_GE_ROP_SRC_XOR	0x66
#define WMT_GE_ROP_SRC_COPY	0xcc
#define WMT_GE_ROP_PAT_COPY	0xf0

/* GE Operation */
struct drm_wmt_ge_op {
	__u32 type;
	__u32 rop;
	__u32 dst_handle;
	__u32 dst_pitch;
	__u32 dst_x;
	__u32 dst_y;
	__u32 width;
	__u32 height;
	__u32 color;
	__u32 src_handle;
	__u32 src_pitch;
	__u32 src_x;
	__u32 src_y;
	__u32 dst_format;
	__u32 src_format;
};

/* GE Submit Request */
struct drm_wmt_ge_submit {
	__u64 ops;
	__u32 num_ops;
	__u32 flags;
	__u32 out_seqno;
	__u32 pad;
};

/* GE Wait Request */
struct drm_wmt_ge_wait {
	__u32 seqno;		/* 0 waits for submit ring space */
	__u32 timeout_us;	/* 0 for the default */
};

#define DRM_WMT_GE_SUBMIT	0x0
#define DRM_WMT_GE_WAIT		0x1

#define DRM_IOCTL_WMT_GE_SUBMIT \
	DRM_IOWR(DRM_COMMAND_BASE + DRM_WMT_GE_SUBMIT, struct drm_wmt_ge_submit)
#define DRM_IOCTL_WMT_GE_WAIT \
	DRM_IOW(DRM_COMMAND_BASE + DRM_WMT_GE_WAIT, struct drm_wmt_ge_wait)

#if defined(__cplusplus)
}
#endif

#endif /* _UAPI_WMT_DRM_H_ */
