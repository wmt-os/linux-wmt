// SPDX-License-Identifier: GPL-2.0-only
/*
 * WonderMedia WM8505 DRM/KMS Graphics Driver
 *
 * 2D Graphics Engine (GE)
 *
 * Copyright (C) 2026 Logan Russell <me@lrussell.net>
 */

#include <linux/cleanup.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/minmax.h>
#include <linux/overflow.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <linux/wordpart.h>
#include <linux/workqueue.h>

#include <drm/drm_file.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_dma_helper.h>

#include "wmt_drm.h"

#define WMT_GE_RING_MASK	(WMT_GE_RING - 1)

void wmt_ge_configure(struct wmt_drm_device *wmt)
{
	writel(WMT_GE_DELAY_DEFAULT, wmt->ge_regs + WMT_GE_DELAY);
	writel(WMT_GE_HM_SEL_MEM, wmt->ge_regs + WMT_GE_HM_SEL);
}

static void wmt_ge_reset(struct wmt_drm_device *wmt)
{
	void __iomem *reg = wmt->ge_regs + WMT_GE_STATUS;
	u32 status;

	writel(0, wmt->ge_regs + WMT_GE_INT_EN);
	regmap_update_bits(wmt->vpp, WMT_VPP_SW_RESET, WMT_VPP_SW_RESET_GE, 0);
	regmap_update_bits(wmt->vpp, WMT_VPP_SW_RESET, WMT_VPP_SW_RESET_GE, WMT_VPP_SW_RESET_GE);
	writel(WMT_GE_ENABLE, wmt->ge_regs + WMT_GE_ENG_EN);
	readl_poll_timeout_atomic(reg, status, !(status & WMT_GE_STATUS_RESET),
				  1, WMT_GE_RESET_US);

	wmt_ge_configure(wmt);
	writel(WMT_GE_INT_COMPLETE | WMT_GE_INT_TIMEOUT, wmt->ge_regs + WMT_GE_INT_EN);
}

static void wmt_ge_set_dst(void __iomem *regs, dma_addr_t dst, struct drm_wmt_ge_op *op)
{
	u32 cpp = wmt_ge_cpp(op->dst_format);

	writel(cpp == 2 ? WMT_GE_DEPTH_16BPP : WMT_GE_DEPTH_32BPP, regs + WMT_GE_COLOR_DEPTH);
	writel(dst, regs + WMT_GE_DES_BADDR);
	writel((op->dst_pitch / cpp) - 1, regs + WMT_GE_DES_DISP_W);
	writel((op->dst_y + op->height) - 1, regs + WMT_GE_DES_DISP_H);
	writel(op->dst_x, regs + WMT_GE_DES_X_START);
	writel(op->dst_y, regs + WMT_GE_DES_Y_START);
	writel(op->width - 1, regs + WMT_GE_DES_WIDTH);
	writel(op->height - 1, regs + WMT_GE_DES_HEIGHT);
}

static void wmt_ge_fill(void __iomem *regs, dma_addr_t dst, struct drm_wmt_ge_op *op)
{
	u32 color = op->color;

	wmt_ge_set_dst(regs, dst, op);

	/* The pattern register holds a whole word of pixels */
	if (wmt_ge_cpp(op->dst_format) == 2)
		color = lower_16_bits(color) | ((u32)lower_16_bits(color) << 16);

	writel(color, regs + WMT_GE_PAT0_COLOR);
	writel(WMT_GE_CMD_BLIT, regs + WMT_GE_COMMAND);
	writel(op->rop, regs + WMT_GE_ROP_CODE);
	writel(WMT_GE_FIRE_GO, regs + WMT_GE_FIRE);
}

static void wmt_ge_blit(void __iomem *regs, dma_addr_t src, dma_addr_t dst,
			struct drm_wmt_ge_op *op)
{
	writel(src, regs + WMT_GE_SRC_BADDR);
	writel((op->src_pitch / wmt_ge_cpp(op->src_format)) - 1, regs + WMT_GE_SRC_DISP_W);
	writel((op->src_y + op->height) - 1, regs + WMT_GE_SRC_DISP_H);
	writel(op->src_x, regs + WMT_GE_SRC_X_START);
	writel(op->src_y, regs + WMT_GE_SRC_Y_START);
	writel(op->width - 1, regs + WMT_GE_SRC_WIDTH);
	writel(op->height - 1, regs + WMT_GE_SRC_HEIGHT);

	wmt_ge_set_dst(regs, dst, op);

	writel(WMT_GE_CMD_BLIT, regs + WMT_GE_COMMAND);
	writel(op->rop, regs + WMT_GE_ROP_CODE);
	writel(WMT_GE_FIRE_GO, regs + WMT_GE_FIRE);
}

static bool wmt_ge_bounds_ok(u32 size, u32 pitch, u32 cpp, u32 x, u32 y, u32 w, u32 h)
{
	u32 pitch_pixels = pitch / cpp;

	return !(!w || !h || pitch % cpp ||
		 pitch_pixels > WMT_GE_MAX_DIM ||
		 h > WMT_GE_MAX_DIM ||
		 y > WMT_GE_MAX_DIM - h ||
		 w > pitch_pixels ||
		 x > pitch_pixels - w ||
		 (y + h) * pitch > size);
}

static bool wmt_ge_validate_op(struct drm_wmt_ge_op *op, u32 dst_size, u32 src_size)
{
	u32 dcpp = wmt_ge_cpp(op->dst_format);
	u32 scpp = wmt_ge_cpp(op->src_format);

	if (!dcpp || !wmt_ge_bounds_ok(dst_size, op->dst_pitch, dcpp, op->dst_x, op->dst_y,
				       op->width, op->height))
		return false;

	if (op->type == WMT_GE_OP_FILL)
		return op->rop == WMT_GE_ROP_PAT_COPY || op->rop == WMT_GE_ROP_PAT_XOR;

	if (!scpp || !wmt_ge_bounds_ok(src_size, op->src_pitch, scpp, op->src_x, op->src_y,
				       op->width, op->height))
		return false;

	/* The GE blits within one pixel format */
	if (op->type == WMT_GE_OP_BLIT)
		return op->src_format == op->dst_format &&
		       (op->rop == WMT_GE_ROP_SRC_COPY || op->rop == WMT_GE_ROP_SRC_XOR);

	return op->type == WMT_GE_OP_CONVERT && op->rop == WMT_GE_ROP_SRC_COPY;
}

/* Caller holds ge_lock */
static void wmt_ge_kick(struct wmt_drm_device *wmt)
{
	struct wmt_ge_job *job = &wmt->ge_ring[wmt->ge_tail & WMT_GE_RING_MASK];
	struct drm_wmt_ge_op *op = &job->ops[job->op_cursor];

	switch (op->type) {
	case WMT_GE_OP_CONVERT:
		wmt_vdma_start(wmt, job->src_addr, job->dst_addr, op);
		break;
	case WMT_GE_OP_BLIT:
		wmt_ge_blit(wmt->ge_regs, job->src_addr, job->dst_addr, op);
		break;
	default:
		wmt_ge_fill(wmt->ge_regs, job->dst_addr, op);
	}
}

/* Caller holds ge_lock */
static void wmt_ge_finish_job(struct wmt_drm_device *wmt, struct wmt_ge_job *job)
{
	/* Drain posted GE writes to DRAM before publishing ge_done */
	dsb();
	readl(wmt->ge_regs + WMT_GE_STATUS);
	WRITE_ONCE(wmt->ge_done, job->seqno);
	wmt->ge_tail++;
}

/* Caller holds ge_lock */
bool wmt_ge_advance(struct wmt_drm_device *wmt, bool errored)
{
	struct wmt_ge_job *job = &wmt->ge_ring[wmt->ge_tail & WMT_GE_RING_MASK];
	bool retire = false;

	if (errored)
		job->errored = true;
	if (errored || ++job->op_cursor >= job->num_ops) {
		wmt_ge_finish_job(wmt, job);
		/* Retire at quiesce, on error, or as the ring nears full */
		retire = errored || wmt->ge_tail == wmt->ge_head ||
			 wmt->ge_head - wmt->ge_rtail >= WMT_GE_RING - 1;
	}
	if (wmt->ge_tail != wmt->ge_head)
		wmt_ge_kick(wmt);

	return retire;
}

irqreturn_t wmt_ge_irq(int irq, void *data)
{
	struct wmt_drm_device *wmt = data;
	u32 flag = readl(wmt->ge_regs + WMT_GE_INT_FLAG);
	bool retire = false, reset = false;

	if (!(flag & (WMT_GE_INT_COMPLETE | WMT_GE_INT_TIMEOUT)))
		return IRQ_NONE;
	writel(WMT_GE_INT_CLEAR, wmt->ge_regs + WMT_GE_INT_FLAG);

	spin_lock(&wmt->ge_lock);
	/* Skip ring if busy or reset pending */
	if (!wmt->ge_reset_pending) {
		if (flag & WMT_GE_INT_TIMEOUT) {
			wmt->ge_reset_pending = true;
			reset = true;
		} else if (wmt->ge_tail != wmt->ge_head) {
			retire = wmt_ge_advance(wmt, false);
		}
	}
	spin_unlock(&wmt->ge_lock);

	if (reset)
		schedule_work(&wmt->ge_reset_work);
	if (retire)
		schedule_work(&wmt->ge_retire_work);
	wake_up(&wmt->ge_wait);

	return IRQ_HANDLED;
}

void wmt_ge_reset_work(struct work_struct *work)
{
	struct wmt_drm_device *wmt = container_of(work, struct wmt_drm_device, ge_reset_work);
	unsigned long flags;
	bool retire = false;

	spin_lock_irqsave(&wmt->ge_lock, flags);
	if (wmt->ge_dead) {
		wmt->ge_reset_pending = false;
		spin_unlock_irqrestore(&wmt->ge_lock, flags);
		return;
	}
	spin_unlock_irqrestore(&wmt->ge_lock, flags);

	wmt_ge_reset(wmt);

	spin_lock_irqsave(&wmt->ge_lock, flags);
	if (wmt->ge_tail != wmt->ge_head)
		retire = wmt_ge_advance(wmt, true);
	wmt->ge_reset_pending = false;
	spin_unlock_irqrestore(&wmt->ge_lock, flags);

	if (retire)
		schedule_work(&wmt->ge_retire_work);
	wake_up(&wmt->ge_wait);
}

void wmt_ge_retire_work(struct work_struct *work)
{
	struct wmt_drm_device *wmt = container_of(work, struct wmt_drm_device, ge_retire_work);
	unsigned long flags;
	u32 r, t, i;

	/*
	 * Frees (GEM put, kvfree) can sleep, so run them outside ge_lock;
	 * re-loop to drain jobs that finish while we free.
	 */
	for (;;) {
		spin_lock_irqsave(&wmt->ge_lock, flags);
		r = wmt->ge_rtail;
		t = wmt->ge_tail;
		spin_unlock_irqrestore(&wmt->ge_lock, flags);

		if (r == t)
			break;

		/* Free finished jobs */
		for (i = r; i != t; i++) {
			struct wmt_ge_job *job = &wmt->ge_ring[i & WMT_GE_RING_MASK];

			drm_gem_object_put(job->dst);
			if (job->src)
				drm_gem_object_put(job->src);
			kvfree(job->ops);
		}

		spin_lock_irqsave(&wmt->ge_lock, flags);
		wmt->ge_rtail = t;
		spin_unlock_irqrestore(&wmt->ge_lock, flags);
		wake_up(&wmt->ge_wait);
	}
}

static bool wmt_ge_spin(struct wmt_drm_device *wmt, u32 target)
{
	ktime_t end = ktime_add_us(ktime_get(), WMT_GE_SPIN_US);

	while (!wmt_ge_passed(READ_ONCE(wmt->ge_done), target)) {
		if (need_resched() || ktime_after(ktime_get(), end))
			return false;
		cpu_relax();
	}

	return true;
}

void wmt_ge_latch_drain(struct wmt_drm_device *wmt, struct drm_gem_object *gem)
{
	unsigned long flags;
	u32 target = 0;
	u32 i;

	spin_lock_irqsave(&wmt->ge_lock, flags);
	/*
	 * Latest seqno writing this buffer (the ring is in seqno order). job->dst
	 * may be freed by retire_work, so only value-compare it, never dereference.
	 */
	for (i = wmt->ge_rtail; i != wmt->ge_head; i++) {
		struct wmt_ge_job *job = &wmt->ge_ring[i & WMT_GE_RING_MASK];

		if (job->dst == gem)
			target = job->seqno;
	}
	spin_unlock_irqrestore(&wmt->ge_lock, flags);

	if (target && !wmt_ge_spin(wmt, target))
		wait_event(wmt->ge_wait, wmt_ge_passed(READ_ONCE(wmt->ge_done), target));
}

void wmt_ge_teardown(void *data)
{
	struct wmt_drm_device *wmt = data;
	unsigned long flags;
	u32 i;

	spin_lock_irqsave(&wmt->ge_lock, flags);
	wmt->ge_dead = true;
	spin_unlock_irqrestore(&wmt->ge_lock, flags);

	writel(0, wmt->ge_regs + WMT_GE_INT_EN);
	writel(0, wmt->vdma_regs + WMT_VDMA_IER);
	writel(WMT_VDMA_GCR_RESET, wmt->vdma_regs + WMT_VDMA_GCR);
	synchronize_irq(wmt->ge_irq);
	synchronize_irq(wmt->vdma_irq);
	cancel_work_sync(&wmt->ge_reset_work);
	cancel_work_sync(&wmt->ge_retire_work);
	writel(0, wmt->ge_regs + WMT_GE_INT_EN);
	writel(0, wmt->ge_regs + WMT_GE_ENG_EN);
	synchronize_irq(wmt->ge_irq);

	/* Retire all queued jobs */
	WRITE_ONCE(wmt->ge_done, wmt->ge_seq);
	wake_up(&wmt->ge_wait);

	/* Free all jobs */
	for (i = wmt->ge_rtail; i != wmt->ge_head; i++) {
		struct wmt_ge_job *job = &wmt->ge_ring[i & WMT_GE_RING_MASK];

		drm_gem_object_put(job->dst);
		if (job->src)
			drm_gem_object_put(job->src);
		kvfree(job->ops);
	}
}

int wmt_drm_ioctl_ge_submit(struct drm_device *dev, void *data, struct drm_file *file_priv)
{
	struct wmt_drm_device *wmt = to_wmt_drm(dev);
	struct drm_wmt_ge_submit *req = data;
	struct drm_wmt_ge_op *ops __free(kvfree) = NULL;
	struct drm_gem_object *dst_obj, *src_obj = NULL;
	u32 dst_handle, src_handle = 0;
	u32 dst_size, src_size = 0;
	bool has_src = false, start;
	struct wmt_ge_job *job;
	unsigned long flags;
	int ret;
	u32 i;

	req->out_seqno = 0;

	/* Validate number of operations */
	if (!req->num_ops || req->num_ops > WMT_GE_MAX_OPS || req->flags || req->pad)
		return -EINVAL;

	ops = vmemdup_user(u64_to_user_ptr(req->ops), array_size(req->num_ops, sizeof(*ops)));
	if (IS_ERR(ops))
		return PTR_ERR(ops);

	/* Validate buffer handles */
	dst_handle = ops[0].dst_handle;
	for (i = 0; i < req->num_ops; i++) {
		struct drm_wmt_ge_op *op = &ops[i];

		if (op->dst_handle != dst_handle)
			return -EINVAL;
		if (op->type == WMT_GE_OP_BLIT || op->type == WMT_GE_OP_CONVERT) {
			if (has_src && op->src_handle != src_handle)
				return -EINVAL;
			src_handle = op->src_handle;
			has_src = true;
		}
	}

	dst_obj = drm_gem_object_lookup(file_priv, dst_handle);
	if (!dst_obj)
		return -ENOENT;
	dst_size = dst_obj->size;

	if (has_src) {
		src_obj = drm_gem_object_lookup(file_priv, src_handle);
		if (!src_obj) {
			ret = -ENOENT;
			goto err_put;
		}
		src_size = src_obj->size;
	}

	for (i = 0; i < req->num_ops; i++) {
		if (!wmt_ge_validate_op(&ops[i], dst_size, src_size)) {
			ret = -EINVAL;
			goto err_put;
		}
	}

	spin_lock_irqsave(&wmt->ge_lock, flags);
	if (wmt->ge_dead) {
		spin_unlock_irqrestore(&wmt->ge_lock, flags);
		ret = -ENODEV;
		goto err_put;
	}
	if (wmt->ge_head - wmt->ge_rtail == WMT_GE_RING) {
		spin_unlock_irqrestore(&wmt->ge_lock, flags);
		/* Start reclaiming finished jobs for the retry */
		schedule_work(&wmt->ge_retire_work);
		ret = -EAGAIN;
		goto err_put;
	}

	wmt->ge_seq++;
	if (!wmt->ge_seq)
		wmt->ge_seq = 1;

	job = &wmt->ge_ring[wmt->ge_head & WMT_GE_RING_MASK];
	job->seqno = wmt->ge_seq;
	job->num_ops = req->num_ops;
	job->op_cursor = 0;
	job->errored = false;
	job->ops = no_free_ptr(ops);
	job->dst = dst_obj;
	job->src = src_obj;
	job->dst_addr = to_drm_gem_dma_obj(dst_obj)->dma_addr;
	job->src_addr = has_src ? to_drm_gem_dma_obj(src_obj)->dma_addr : 0;
	req->out_seqno = job->seqno;

	/* Kick engine if idle */
	start = wmt->ge_head == wmt->ge_tail;
	wmt->ge_head++;
	if (start)
		wmt_ge_kick(wmt);
	spin_unlock_irqrestore(&wmt->ge_lock, flags);

	return 0;

err_put:
	drm_gem_object_put(dst_obj);
	if (src_obj)
		drm_gem_object_put(src_obj);
	return ret;
}

static bool wmt_ge_seqno_errored(struct wmt_drm_device *wmt, u32 seqno)
{
	unsigned long flags;
	bool errored = false;
	u32 i;

	spin_lock_irqsave(&wmt->ge_lock, flags);
	for (i = 0; i < WMT_GE_RING; i++) {
		struct wmt_ge_job *job = &wmt->ge_ring[i];

		if (job->seqno == seqno) {
			errored = job->errored;
			break;
		}
	}
	spin_unlock_irqrestore(&wmt->ge_lock, flags);

	return errored;
}

int wmt_drm_ioctl_ge_wait(struct drm_device *dev, void *data, struct drm_file *file_priv)
{
	struct wmt_drm_device *wmt = to_wmt_drm(dev);
	struct drm_wmt_ge_wait *req = data;
	u32 timeout_us = req->timeout_us ? req->timeout_us : WMT_GE_TIMEOUT_US;
	unsigned long t = usecs_to_jiffies(min_t(u32, timeout_us, WMT_GE_WAIT_MAX_US));
	unsigned long flags;
	bool future;
	long ret;

	if (req->seqno) {
		spin_lock_irqsave(&wmt->ge_lock, flags);
		future = !wmt_ge_passed(wmt->ge_seq, req->seqno);
		spin_unlock_irqrestore(&wmt->ge_lock, flags);
		if (future)
			return -EINVAL;
		if (!wmt_ge_spin(wmt, req->seqno)) {
			ret = wait_event_killable_timeout(wmt->ge_wait,
							  wmt_ge_passed(READ_ONCE(wmt->ge_done),
									req->seqno), t);
			if (ret < 0)
				return ret;
			if (!ret)
				return -ETIMEDOUT;
		}
		if (wmt_ge_seqno_errored(wmt, req->seqno))
			return -EIO;
		return 0;
	}

	ret = wait_event_killable_timeout(wmt->ge_wait,
					  READ_ONCE(wmt->ge_head) - READ_ONCE(wmt->ge_rtail) <
					  WMT_GE_RING, t);
	if (ret < 0)
		return ret;
	if (!ret)
		return -ETIMEDOUT;
	return 0;
}
