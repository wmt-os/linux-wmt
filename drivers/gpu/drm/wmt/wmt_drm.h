/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * WonderMedia WM8505 DRM/KMS Graphics Driver
 *
 * Copyright (C) 2026 Logan Russell <me@lrussell.net>
 */

#ifndef _WMT_DRM_H_
#define _WMT_DRM_H_

#include <linux/container_of.h>
#include <linux/irqreturn.h>
#include <linux/regmap.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <drm/drm_device.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_simple_kms_helper.h>

#include <drm/wmt_drm.h>
#include "wmt_regs.h"

/* Limits and Timeouts */
#define WMT_GOVRH_SETTLE_MS	200
#define WMT_GE_RING		16
#define WMT_GE_MAX_CPP		4
#define WMT_GE_TIMEOUT_US	100000
#define WMT_GE_RESET_US		10000
#define WMT_GE_SPIN_US		100

/* VDMA Descriptor Table */
#define WMT_VDMA_DESCS		(WMT_GE_MAX_DIM * WMT_GE_MAX_DIM * WMT_GE_MAX_CPP / WMT_VDMA_CHUNK)

struct drm_gem_object;
struct clk;
struct drm_pending_vblank_event;

/* Queued GE Job */
struct wmt_ge_job {
	u32 seqno;
	u32 num_ops;
	u32 op_cursor;
	bool errored;
	struct drm_wmt_ge_op *ops;
	struct drm_gem_object *dst;
	struct drm_gem_object *src;
	dma_addr_t dst_addr;
	dma_addr_t src_addr;
};

/* VDMA Read Descriptor */
struct wmt_vdma_desc {
	u16 count;
	u16 flags;
	u32 addr;
};

/* GE Completion Timeline */
static inline bool wmt_ge_passed(u32 done, u32 target)
{
	return (s32)(done - target) >= 0;
}

/* Engine Pixel Formats */
static inline u32 wmt_ge_cpp(u32 format)
{
	switch (format) {
	case DRM_FORMAT_RGB565:
		return 2;
	case DRM_FORMAT_XRGB8888:
		return 4;
	default:
		return 0;
	}
}

/* Primary Device Context */
struct wmt_drm_device {
	struct drm_device drm;
	struct drm_simple_display_pipe pipe;

	void __iomem *govrh_regs;
	struct regmap *vpp;
	void __iomem *vdma_regs;
	void __iomem *ge_regs;
	int vdma_irq;
	int ge_irq;

	struct clk *clk_dvo;

	/* VDMA read descriptor table */
	struct wmt_vdma_desc *vdma_desc;
	dma_addr_t vdma_desc_dma;

	/* GE async job ring */
	spinlock_t ge_lock;
	struct wmt_ge_job ge_ring[WMT_GE_RING];
	u32 ge_head;
	u32 ge_tail;
	u32 ge_rtail;
	u32 ge_seq;
	u32 ge_done;
	bool ge_reset_pending;
	bool ge_dead;
	wait_queue_head_t ge_wait;
	struct work_struct ge_retire_work;
	struct work_struct ge_reset_work;

	/* GOVRH page-flip */
	struct drm_pending_vblank_event *pending_event;
};

#define to_wmt_drm(x) container_of(x, struct wmt_drm_device, drm)

/* Display Pipe and VBlank Interrupt */
int wmt_pipe_init(struct wmt_drm_device *wmt);
irqreturn_t wmt_vblank_irq(int irq, void *data);

/* Video DMA Functions */
irqreturn_t wmt_vdma_irq(int irq, void *data);
void wmt_vdma_start(struct wmt_drm_device *wmt, dma_addr_t src, dma_addr_t dst,
		    struct drm_wmt_ge_op *op);

/* 2D Engine Functions */
irqreturn_t wmt_ge_irq(int irq, void *data);
void wmt_ge_configure(struct wmt_drm_device *wmt);
bool wmt_ge_advance(struct wmt_drm_device *wmt, bool errored);
void wmt_ge_retire_work(struct work_struct *work);
void wmt_ge_reset_work(struct work_struct *work);
void wmt_ge_teardown(void *data);
void wmt_ge_latch_drain(struct wmt_drm_device *wmt, struct drm_gem_object *gem);
int wmt_drm_ioctl_ge_submit(struct drm_device *dev, void *data, struct drm_file *file_priv);
int wmt_drm_ioctl_ge_wait(struct drm_device *dev, void *data, struct drm_file *file_priv);

#endif /* _WMT_DRM_H_ */
