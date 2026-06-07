// SPDX-License-Identifier: GPL-2.0-only
/*
 * WonderMedia WM8505 DRM/KMS Graphics Driver
 *
 * Video DMA (VDMA)
 *
 * Copyright (C) 2026 Logan Russell <me@lrussell.net>
 */

#include <linux/bitfield.h>
#include <linux/dev_printk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/minmax.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include "wmt_drm.h"

void wmt_vdma_start(struct wmt_drm_device *wmt, dma_addr_t src, dma_addr_t dst,
		    struct drm_wmt_ge_op *op)
{
	void __iomem *regs = wmt->vdma_regs;
	u32 scpp = wmt_ge_cpp(op->src_format);
	u32 dcpp = wmt_ge_cpp(op->dst_format);
	struct wmt_vdma_desc *desc = wmt->vdma_desc;
	dma_addr_t addr = src + op->src_y * op->src_pitch;
	u32 bytes = (op->height - 1) * op->src_pitch + (op->src_x + op->width) * scpp;
	u32 ccr;

	/* Descriptors cover full scanlines; skip registers clip the active rectangle */
	while (bytes) {
		u32 chunk = min(bytes, WMT_VDMA_CHUNK);

		desc->count = chunk;
		desc->flags = 0;
		desc->addr = addr;
		desc++;
		addr += chunk;
		bytes -= chunk;
	}
	desc[-1].flags = WMT_VDMA_DESC_END | WMT_VDMA_DESC_INT;

	writel(WMT_VDMA_GCR_RESET, regs + WMT_VDMA_GCR);
	writel(WMT_VDMA_GCR_EN, regs + WMT_VDMA_GCR);
	writel(WMT_VDMA_INT, regs + WMT_VDMA_IER);
	writel(WMT_VDMA_INT, regs + WMT_VDMA_ISR);

	writel(wmt->vdma_desc_dma, regs + WMT_VDMA_RDP_DES);
	writel(op->src_x * scpp, regs + WMT_VDMA_RDP_ISKIP);
	writel(op->src_pitch - op->width * scpp, regs + WMT_VDMA_RDP_LSKIP);
	writel(op->width * scpp, regs + WMT_VDMA_RDP_LINE);
	writel(op->height, regs + WMT_VDMA_PIC_H);

	writel(dst, regs + WMT_VDMA_PS_START);
	writel(op->dst_y * op->dst_pitch + op->dst_x * dcpp, regs + WMT_VDMA_PS_ISKIP);
	writel(op->dst_pitch - op->width * dcpp, regs + WMT_VDMA_PS_LSKIP);
	writel(op->width * dcpp, regs + WMT_VDMA_PS_LINE);
	writel(0, regs + WMT_VDMA_DES_FIX);

	ccr = FIELD_PREP(WMT_VDMA_CCR_RDP_BPP,
			 scpp == 2 ? WMT_VDMA_RDP_16BPP : WMT_VDMA_RDP_32BPP) |
	      WMT_VDMA_CCR_INT_EN | WMT_VDMA_CCR_DONE;
	if (dcpp == 4)
		ccr |= WMT_VDMA_CCR_PS_32BPP;
	writel(ccr, regs + WMT_VDMA_CCR);
	writel(ccr | WMT_VDMA_CCR_RUN, regs + WMT_VDMA_CCR);
}

irqreturn_t wmt_vdma_irq(int irq, void *data)
{
	struct wmt_drm_device *wmt = data;
	/* Acknowledging the interrupt clears the CCR event, so read it first */
	u32 ccr = readl(wmt->vdma_regs + WMT_VDMA_CCR);
	bool errored, retire = false;

	if (!(readl(wmt->vdma_regs + WMT_VDMA_ISR) & WMT_VDMA_INT))
		return IRQ_NONE;
	writel(WMT_VDMA_INT, wmt->vdma_regs + WMT_VDMA_ISR);

	errored = FIELD_GET(WMT_VDMA_CCR_EVENT, ccr) != WMT_VDMA_EVENT_DONE;
	if (errored) {
		dev_err_ratelimited(wmt->drm.dev, "VDMA error, CCR %#x\n", ccr);
		writel(WMT_VDMA_GCR_RESET, wmt->vdma_regs + WMT_VDMA_GCR);
	}

	spin_lock(&wmt->ge_lock);
	if (wmt->ge_tail != wmt->ge_head)
		retire = wmt_ge_advance(wmt, errored);
	spin_unlock(&wmt->ge_lock);

	if (retire)
		schedule_work(&wmt->ge_retire_work);
	wake_up(&wmt->ge_wait);

	return IRQ_HANDLED;
}
