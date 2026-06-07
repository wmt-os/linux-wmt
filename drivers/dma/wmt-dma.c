// SPDX-License-Identifier: GPL-2.0-only
/*
 * WonderMedia WM8505 DMA Controller Driver
 *
 * Copyright (C) 2026 Logan Russell <me@lrussell.net>
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_dma.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>
#include <linux/string.h>

#include "dmaengine.h"
#include "virt-dma.h"

/* DMA Controller Register Offsets */
#define WMT_DMA_GCR			0x40
#define WMT_DMA_MRPR			0x44
#define WMT_DMA_IER			0x48
#define WMT_DMA_ISR			0x4c
#define WMT_DMA_TMR			0x50
#define WMT_DMA_CCR_BASE		0x80
#define WMT_DMA_CCR_STRIDE		0x4

/* Global Control Register (GCR) Flags */
#define WMT_DMA_GCR_GLOBAL_EN		BIT(0)
#define WMT_DMA_GCR_BIG_ENDIAN		BIT(1)
#define WMT_DMA_GCR_SW_RST		BIT(8)

/* Channel Control Register (CCR) Bitfields */
#define WMT_DMA_CCR_SYSTEM_RUN		BIT(7)
#define WMT_DMA_CCR_DMA_ACTIVE		BIT(4)
#define WMT_DMA_CCR_UP_MEMREG_EN	BIT(15)
#define WMT_DMA_CCR_DEVICE_TO_MEM	BIT(22)
#define WMT_DMA_CCR_SW_REQ		BIT(23)
#define WMT_DMA_CCR_SG_MODE		BIT(25)
#define WMT_DMA_CCR_INC_MODE		BIT(24)

#define WMT_DMA_CCR_SIZE_8		0x0
#define WMT_DMA_CCR_SIZE_16		BIT(26)
#define WMT_DMA_CCR_SIZE_32		BIT(27)

#define WMT_DMA_CCR_BURST_1		0x0
#define WMT_DMA_CCR_BURST_4		BIT(28)
#define WMT_DMA_CCR_BURST_8		BIT(29)

#define WMT_DMA_CCR_WRAP_1		0x0
#define WMT_DMA_CCR_REQ_ID_SHIFT	16
#define WMT_DMA_CCR_REQ_ID_MASK		GENMASK(20, 16)
#define WMT_DMA_CCR_EVT_ID_MASK		GENMASK(3, 0)
#define WMT_DMA_EVT_NO_STATUS		0
#define WMT_DMA_EVT_SUCCESS		15

/* Hardware Descriptor Flags */
#define WMT_DMA_DES_END			BIT(31)
#define WMT_DMA_FORMAT_DES1		BIT(30)
#define WMT_DMA_INTEN_DES		BIT(16)
#define WMT_DMA_DES_REQCNT_MASK		GENMASK(15, 0)

#define WMT_DMA_CHANNELS		16
#define WMT_DMA_MAX_DESC_CNT		128
#define WMT_DMA_MAX_CHUNK		(WMT_DMA_DES_REQCNT_MASK & ~0x3)

/* Format 1 hardware descriptor (16 bytes); branch_addr links chains and rings */
struct wmt_dma_hw_desc {
	u32	req_cnt;
	u32	data_addr;
	u32	branch_addr;
	u32	reserved;
};

/* Context RAM block; the controller fetches and mirrors per-channel context here */
struct wmt_dma_mem_reg {
	u32	if0_rbr;
	u32	if0_dar;
	u32	if0_bar;
	u32	if0_cpr;
	u32	if1_rbr;
	u32	if1_dar;
	u32	if1_bar;
	u32	if1_cpr;
};

/* Software Descriptor */
struct wmt_dma_desc {
	struct virt_dma_desc		vdesc;
	struct wmt_dma_hw_desc		*hw_desc;
	dma_addr_t			hw_desc_phys;
	dma_addr_t			hw_desc2_phys;
	u32				desc_count;
	u32				ccr_value;
	u32				total_len;
	enum dma_transfer_direction	dir;
	bool				cyclic;
};

/* Hardware Channel Context */
struct wmt_dma_chan {
	struct virt_dma_chan		vchan;
	u32				id;
	u32				req_id;	/* Peripheral request line */
	int				irq;
	void __iomem			*ccr_reg;
	struct wmt_dma_desc		*active_desc;
	struct wmt_dma_dev		*dmadev;
	struct dma_slave_config		dev_cfg;
};

/* Primary Device Context */
struct wmt_dma_dev {
	struct dma_device		ddev;
	void __iomem			*base;
	struct clk			*clk;
	struct dma_pool			*desc_pool;

	/* Coherent Context RAM Block */
	struct wmt_dma_mem_reg		*mem_regs;
	dma_addr_t			mem_regs_phys;

	struct wmt_dma_chan		chans[WMT_DMA_CHANNELS];
};

static inline struct wmt_dma_chan *to_wmt_dma_chan(struct dma_chan *c)
{
	return container_of(c, struct wmt_dma_chan, vchan.chan);
}

static inline struct wmt_dma_desc *to_wmt_dma_desc(struct virt_dma_desc *vd)
{
	return container_of(vd, struct wmt_dma_desc, vdesc);
}

static void wmt_dma_free_chan_resources(struct dma_chan *chan)
{
	struct wmt_dma_chan *c = to_wmt_dma_chan(chan);

	vchan_free_chan_resources(&c->vchan);
}

static int wmt_dma_device_config(struct dma_chan *chan, struct dma_slave_config *cfg)
{
	struct wmt_dma_chan *c = to_wmt_dma_chan(chan);

	memcpy(&c->dev_cfg, cfg, sizeof(*cfg));
	return 0;
}

static void wmt_dma_free_desc(struct virt_dma_desc *vd)
{
	struct wmt_dma_desc *desc = to_wmt_dma_desc(vd);
	struct wmt_dma_dev *dmadev = to_wmt_dma_chan(vd->tx.chan)->dmadev;

	dma_pool_free(dmadev->desc_pool, desc->hw_desc, desc->hw_desc_phys);
	kfree(desc);
}

static void wmt_dma_start_desc(struct wmt_dma_chan *c)
{
	struct wmt_dma_desc *desc = c->active_desc;
	struct wmt_dma_dev *dmadev = c->dmadev;

	/* Quiesce the channel; RUN persists after completion and blocks reprogramming */
	writel(0, c->ccr_reg);

	/* Clear stale end-of-transfer context before rearming */
	memset(&dmadev->mem_regs[c->id], 0, sizeof(dmadev->mem_regs[c->id]));

	/* IF0 runs the memory descriptor chain */
	dmadev->mem_regs[c->id].if0_cpr = desc->hw_desc_phys;

	/* IF1 holds the device FIFO address, or the destination chain for memcpy */
	if (desc->dir == DMA_MEM_TO_MEM)
		dmadev->mem_regs[c->id].if1_cpr = desc->hw_desc2_phys;
	else if (desc->dir == DMA_DEV_TO_MEM)
		dmadev->mem_regs[c->id].if1_cpr = c->dev_cfg.src_addr;
	else
		dmadev->mem_regs[c->id].if1_cpr = c->dev_cfg.dst_addr;

	writel(BIT(c->id), dmadev->base + WMT_DMA_ISR);

	/* Start the channel; UP_MEMREG mirrors live context for residue reads */
	writel(desc->ccr_value | WMT_DMA_CCR_SYSTEM_RUN | WMT_DMA_CCR_UP_MEMREG_EN, c->ccr_reg);
}

/* Caller holds the vchan lock */
static void wmt_dma_start_next(struct wmt_dma_chan *c)
{
	struct virt_dma_desc *vd = vchan_next_desc(&c->vchan);

	if (vd) {
		list_del(&vd->node);
		c->active_desc = to_wmt_dma_desc(vd);
		wmt_dma_start_desc(c);
	}
}

static void wmt_dma_issue_pending(struct dma_chan *chan)
{
	struct wmt_dma_chan *c = to_wmt_dma_chan(chan);
	unsigned long flags;

	spin_lock_irqsave(&c->vchan.lock, flags);
	if (vchan_issue_pending(&c->vchan) && !c->active_desc)
		wmt_dma_start_next(c);
	spin_unlock_irqrestore(&c->vchan.lock, flags);
}

static u32 wmt_dma_get_ccr(struct wmt_dma_chan *c, enum dma_transfer_direction dir)
{
	u32 ccr = 0;
	enum dma_slave_buswidth buswidth = (dir == DMA_DEV_TO_MEM) ?
		c->dev_cfg.src_addr_width : c->dev_cfg.dst_addr_width;
	u32 maxburst = (dir == DMA_DEV_TO_MEM) ?
		c->dev_cfg.src_maxburst : c->dev_cfg.dst_maxburst;

	if (dir == DMA_DEV_TO_MEM)
		ccr |= WMT_DMA_CCR_DEVICE_TO_MEM;

	switch (buswidth) {
	case DMA_SLAVE_BUSWIDTH_1_BYTE:
		ccr |= WMT_DMA_CCR_SIZE_8;
		break;
	case DMA_SLAVE_BUSWIDTH_2_BYTES:
		ccr |= WMT_DMA_CCR_SIZE_16;
		break;
	case DMA_SLAVE_BUSWIDTH_4_BYTES:
	default:
		ccr |= WMT_DMA_CCR_SIZE_32;
		break;
	}

	switch (maxburst) {
	case 1:
		ccr |= WMT_DMA_CCR_BURST_1;
		break;
	case 4:
		ccr |= WMT_DMA_CCR_BURST_4;
		break;
	case 8:
	default:
		ccr |= WMT_DMA_CCR_BURST_8;
		break;
	}

	ccr |= (c->req_id << WMT_DMA_CCR_REQ_ID_SHIFT) & WMT_DMA_CCR_REQ_ID_MASK;
	return ccr;
}

static struct dma_async_tx_descriptor *wmt_dma_prep_dma_cyclic(struct dma_chan *chan,
							       dma_addr_t buf_addr,
							       size_t buf_len, size_t period_len,
							       enum dma_transfer_direction dir,
							       unsigned long flags)
{
	struct wmt_dma_chan *c = to_wmt_dma_chan(chan);
	struct wmt_dma_desc *desc;
	size_t periods = buf_len / period_len;
	int i;

	if (!is_slave_direction(dir))
		return NULL;

	if (period_len > WMT_DMA_DES_REQCNT_MASK)
		return NULL;

	if (buf_len % period_len)
		return NULL;

	if (periods > WMT_DMA_MAX_DESC_CNT)
		return NULL;

	desc = kzalloc(sizeof(*desc), GFP_NOWAIT);
	if (!desc)
		return NULL;

	desc->desc_count = periods;
	desc->total_len = buf_len;
	desc->dir = dir;
	desc->cyclic = true;

	desc->hw_desc = dma_pool_alloc(c->dmadev->desc_pool, GFP_NOWAIT, &desc->hw_desc_phys);
	if (!desc->hw_desc) {
		kfree(desc);
		return NULL;
	}

	for (i = 0; i < periods; i++) {
		struct wmt_dma_hw_desc *hw = &desc->hw_desc[i];
		dma_addr_t next_phys = desc->hw_desc_phys + ((i + 1) * sizeof(*hw));

		/* Close the ring at the final period */
		if (i == periods - 1)
			next_phys = desc->hw_desc_phys;

		hw->data_addr = buf_addr + (i * period_len);
		hw->branch_addr = next_phys;
		hw->req_cnt = period_len | WMT_DMA_FORMAT_DES1;

		/* Interrupt per period */
		if (flags & DMA_PREP_INTERRUPT)
			hw->req_cnt |= WMT_DMA_INTEN_DES;
	}

	desc->ccr_value = wmt_dma_get_ccr(c, dir);
	return vchan_tx_prep(&c->vchan, &desc->vdesc, flags);
}

static struct dma_async_tx_descriptor *wmt_dma_prep_slave_sg(struct dma_chan *chan,
							     struct scatterlist *sgl,
							     unsigned int sg_len,
							     enum dma_transfer_direction dir,
							     unsigned long flags, void *context)
{
	struct wmt_dma_chan *c = to_wmt_dma_chan(chan);
	struct wmt_dma_desc *desc;
	struct scatterlist *sg;
	unsigned int num_descs = 0;
	int i, d = 0;

	if (!is_slave_direction(dir))
		return NULL;

	/* Count descriptors after chunking */
	for_each_sg(sgl, sg, sg_len, i)
		num_descs += DIV_ROUND_UP(sg_dma_len(sg), WMT_DMA_MAX_CHUNK);

	if (num_descs == 0 || num_descs > WMT_DMA_MAX_DESC_CNT)
		return NULL;

	desc = kzalloc(sizeof(*desc), GFP_NOWAIT);
	if (!desc)
		return NULL;

	desc->desc_count = num_descs;
	desc->dir = dir;
	desc->cyclic = false;
	desc->total_len = 0;

	desc->hw_desc = dma_pool_alloc(c->dmadev->desc_pool, GFP_NOWAIT, &desc->hw_desc_phys);
	if (!desc->hw_desc) {
		kfree(desc);
		return NULL;
	}

	for_each_sg(sgl, sg, sg_len, i) {
		dma_addr_t addr = sg_dma_address(sg);
		size_t len = sg_dma_len(sg);

		while (len) {
			size_t chunk = min_t(size_t, len, WMT_DMA_MAX_CHUNK);
			struct wmt_dma_hw_desc *hw = &desc->hw_desc[d];

			hw->data_addr = addr;
			hw->req_cnt = chunk | WMT_DMA_FORMAT_DES1;
			hw->branch_addr = desc->hw_desc_phys + ((d + 1) * sizeof(*hw));

			addr += chunk;
			len -= chunk;
			desc->total_len += chunk;
			d++;
		}
	}

	desc->hw_desc[d - 1].req_cnt |= WMT_DMA_DES_END | WMT_DMA_INTEN_DES;

	desc->ccr_value = wmt_dma_get_ccr(c, dir);
	return vchan_tx_prep(&c->vchan, &desc->vdesc, flags);
}

static struct dma_async_tx_descriptor *wmt_dma_prep_dma_memcpy(struct dma_chan *chan,
							       dma_addr_t dst, dma_addr_t src,
							       size_t len, unsigned long flags)
{
	struct wmt_dma_chan *c = to_wmt_dma_chan(chan);
	struct wmt_dma_desc *desc;
	size_t off = 0;
	u32 chunks, i;

	if (!len || (src | dst | len) & 0x3)
		return NULL;

	chunks = DIV_ROUND_UP(len, WMT_DMA_MAX_CHUNK);
	if (chunks * 2 > WMT_DMA_MAX_DESC_CNT)
		return NULL;

	desc = kzalloc(sizeof(*desc), GFP_NOWAIT);
	if (!desc)
		return NULL;

	desc->desc_count = chunks;
	desc->total_len = len;
	desc->dir = DMA_MEM_TO_MEM;

	desc->hw_desc = dma_pool_alloc(c->dmadev->desc_pool, GFP_NOWAIT, &desc->hw_desc_phys);
	if (!desc->hw_desc) {
		kfree(desc);
		return NULL;
	}
	desc->hw_desc2_phys = desc->hw_desc_phys + chunks * sizeof(*desc->hw_desc);

	/* Source chain on IF0 and destination chain on IF1, halves of one block */
	for (i = 0; i < chunks; i++) {
		struct wmt_dma_hw_desc *s = &desc->hw_desc[i];
		struct wmt_dma_hw_desc *d = &desc->hw_desc[chunks + i];
		u32 chunk = min_t(size_t, len - off, WMT_DMA_MAX_CHUNK);

		s->req_cnt = chunk | WMT_DMA_FORMAT_DES1;
		s->data_addr = src + off;
		s->branch_addr = desc->hw_desc_phys + ((i + 1) * sizeof(*s));
		d->req_cnt = chunk | WMT_DMA_FORMAT_DES1;
		d->data_addr = dst + off;
		d->branch_addr = desc->hw_desc2_phys + ((i + 1) * sizeof(*d));
		off += chunk;
	}
	desc->hw_desc[chunks - 1].req_cnt |= WMT_DMA_DES_END;

	/* Completion signals on the write side */
	desc->hw_desc[(chunks * 2) - 1].req_cnt |= WMT_DMA_DES_END | WMT_DMA_INTEN_DES;

	desc->ccr_value = WMT_DMA_CCR_SIZE_32 | WMT_DMA_CCR_BURST_8 |
			  WMT_DMA_CCR_SG_MODE | WMT_DMA_CCR_SW_REQ;

	return vchan_tx_prep(&c->vchan, &desc->vdesc, flags);
}

static int wmt_dma_terminate_all(struct dma_chan *chan)
{
	struct wmt_dma_chan *c = to_wmt_dma_chan(chan);
	unsigned long flags;
	LIST_HEAD(head);
	u32 ccr;
	int ret;

	spin_lock_irqsave(&c->vchan.lock, flags);

	writel(0, c->ccr_reg);

	if (c->active_desc) {
		vchan_terminate_vdesc(&c->active_desc->vdesc);
		c->active_desc = NULL;
	}

	vchan_get_all_descriptors(&c->vchan, &head);
	spin_unlock_irqrestore(&c->vchan.lock, flags);

	ret = readl_poll_timeout_atomic(c->ccr_reg, ccr,
					!(ccr & WMT_DMA_CCR_DMA_ACTIVE), 10, 2000);
	if (ret)
		dev_err(c->dmadev->ddev.dev, "channel %u failed to halt\n", c->id);

	vchan_dma_desc_free_list(&c->vchan, &head);

	return ret;
}

static void wmt_dma_synchronize(struct dma_chan *chan)
{
	struct wmt_dma_chan *c = to_wmt_dma_chan(chan);

	synchronize_irq(c->irq);
	vchan_synchronize(&c->vchan);
}

static enum dma_status wmt_dma_tx_status(struct dma_chan *chan,
					 dma_cookie_t cookie,
					 struct dma_tx_state *txstate)
{
	struct wmt_dma_chan *c = to_wmt_dma_chan(chan);
	struct wmt_dma_dev *dmadev = c->dmadev;
	struct virt_dma_desc *vd;
	enum dma_status ret;
	unsigned long flags;
	size_t residue = 0;

	ret = dma_cookie_status(chan, cookie, txstate);
	if (ret == DMA_COMPLETE || !txstate)
		return ret;

	spin_lock_irqsave(&c->vchan.lock, flags);

	if (c->active_desc && c->active_desc->vdesc.tx.cookie == cookie) {
		size_t transferred = 0;
		bool found = false;
		/*
		 * IF0 mirrors the memory-side address in Context RAM
		 * regardless of direction; memcpy reports source progress.
		 */
		u32 current_dar = dmadev->mem_regs[c->id].if0_dar;
		int i;

		/* Locate the chunk holding the current address */
		for (i = 0; i < c->active_desc->desc_count; i++) {
			struct wmt_dma_hw_desc *hw = &c->active_desc->hw_desc[i];
			size_t chunk_len = hw->req_cnt & WMT_DMA_DES_REQCNT_MASK;

			if (current_dar >= hw->data_addr &&
			    current_dar <= hw->data_addr + chunk_len) {
				transferred += (current_dar - hw->data_addr);
				found = true;
				break;
			}
			transferred += chunk_len;
		}
		residue = c->active_desc->total_len - (found ? transferred : 0);
	} else {
		vd = vchan_find_desc(&c->vchan, cookie);
		if (vd)
			residue = to_wmt_dma_desc(vd)->total_len;
	}

	dma_set_residue(txstate, residue);
	spin_unlock_irqrestore(&c->vchan.lock, flags);

	return ret;
}

static irqreturn_t wmt_dma_interrupt(int irq, void *dev_id)
{
	struct wmt_dma_chan *c = dev_id;
	struct wmt_dma_dev *dmadev = c->dmadev;
	u32 isr, ccr, evt;
	unsigned long flags;

	isr = readl(dmadev->base + WMT_DMA_ISR);
	if (!(isr & BIT(c->id)))
		return IRQ_NONE;

	writel(BIT(c->id), dmadev->base + WMT_DMA_ISR);

	ccr = readl(c->ccr_reg);
	evt = ccr & WMT_DMA_CCR_EVT_ID_MASK;

	spin_lock_irqsave(&c->vchan.lock, flags);
	if (c->active_desc) {
		if (evt == WMT_DMA_EVT_SUCCESS || evt == WMT_DMA_EVT_NO_STATUS) {
			if (c->active_desc->cyclic) {
				vchan_cyclic_callback(&c->active_desc->vdesc);
			} else {
				vchan_cookie_complete(&c->active_desc->vdesc);
				c->active_desc = NULL;
				wmt_dma_start_next(c);
			}
		} else {
			/* Quiesce the faulted channel and fail the transfer */
			dev_err(dmadev->ddev.dev, "channel %u fault event %u\n", c->id, evt);
			writel(0, c->ccr_reg);
			c->active_desc->vdesc.tx_result.result = DMA_TRANS_ABORTED;
			if (!c->active_desc->cyclic) {
				vchan_cookie_complete(&c->active_desc->vdesc);
				c->active_desc = NULL;
				wmt_dma_start_next(c);
			}
		}
	}
	spin_unlock_irqrestore(&c->vchan.lock, flags);

	return IRQ_HANDLED;
}

static struct dma_chan *wmt_dma_xlate(struct of_phandle_args *dma_spec, struct of_dma *of_dma)
{
	struct wmt_dma_dev *dmadev = of_dma->of_dma_data;
	struct dma_chan *chan;

	if (dma_spec->args_count != 1 ||
	    dma_spec->args[0] > FIELD_MAX(WMT_DMA_CCR_REQ_ID_MASK))
		return NULL;

	chan = dma_get_any_slave_channel(&dmadev->ddev);
	if (chan)
		to_wmt_dma_chan(chan)->req_id = dma_spec->args[0];

	return chan;
}

static int wmt_dma_probe(struct platform_device *pdev)
{
	struct wmt_dma_dev *dmadev;
	struct device *dev = &pdev->dev;
	int i, ret, irq;
	char *name;

	dmadev = devm_kzalloc(dev, sizeof(*dmadev), GFP_KERNEL);
	if (!dmadev)
		return -ENOMEM;

	dmadev->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dmadev->base))
		return PTR_ERR(dmadev->base);

	dmadev->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(dmadev->clk))
		return dev_err_probe(dev, PTR_ERR(dmadev->clk), "failed to get clock\n");

	/* Coherent Context RAM: one register block per channel */
	dmadev->mem_regs = dmam_alloc_coherent(dev,
					       sizeof(struct wmt_dma_mem_reg) * WMT_DMA_CHANNELS,
					       &dmadev->mem_regs_phys, GFP_KERNEL);
	if (!dmadev->mem_regs)
		return -ENOMEM;

	writel(WMT_DMA_GCR_SW_RST, dmadev->base + WMT_DMA_GCR);
	writel(WMT_DMA_GCR_GLOBAL_EN, dmadev->base + WMT_DMA_GCR);

	/* Point the controller at the Context RAM */
	writel(dmadev->mem_regs_phys, dmadev->base + WMT_DMA_MRPR);

	/* Pool of per-transfer descriptor blocks */
	dmadev->desc_pool = dmam_pool_create("wmt_dma_desc", dev,
					     sizeof(struct wmt_dma_hw_desc) * WMT_DMA_MAX_DESC_CNT,
					     16, 0);
	if (!dmadev->desc_pool)
		return -ENOMEM;

	INIT_LIST_HEAD(&dmadev->ddev.channels);
	dma_cap_set(DMA_SLAVE, dmadev->ddev.cap_mask);
	dma_cap_set(DMA_PRIVATE, dmadev->ddev.cap_mask);
	dma_cap_set(DMA_CYCLIC, dmadev->ddev.cap_mask);
	dma_cap_set(DMA_MEMCPY, dmadev->ddev.cap_mask);

	dmadev->ddev.dev = dev;
	dmadev->ddev.device_free_chan_resources = wmt_dma_free_chan_resources;
	dmadev->ddev.device_prep_dma_cyclic = wmt_dma_prep_dma_cyclic;
	dmadev->ddev.device_prep_slave_sg = wmt_dma_prep_slave_sg;
	dmadev->ddev.device_prep_dma_memcpy = wmt_dma_prep_dma_memcpy;
	dmadev->ddev.device_issue_pending = wmt_dma_issue_pending;
	dmadev->ddev.device_terminate_all = wmt_dma_terminate_all;
	dmadev->ddev.device_synchronize = wmt_dma_synchronize;
	dmadev->ddev.device_tx_status = wmt_dma_tx_status;
	dmadev->ddev.device_config = wmt_dma_device_config;

	dmadev->ddev.src_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |
				       BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
				       BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	dmadev->ddev.dst_addr_widths = BIT(DMA_SLAVE_BUSWIDTH_1_BYTE) |
				       BIT(DMA_SLAVE_BUSWIDTH_2_BYTES) |
				       BIT(DMA_SLAVE_BUSWIDTH_4_BYTES);
	dmadev->ddev.directions = BIT(DMA_DEV_TO_MEM) | BIT(DMA_MEM_TO_DEV) |
				  BIT(DMA_MEM_TO_MEM);
	dmadev->ddev.residue_granularity = DMA_RESIDUE_GRANULARITY_BURST;
	dmadev->ddev.copy_align = DMAENGINE_ALIGN_4_BYTES;

	for (i = 0; i < WMT_DMA_CHANNELS; i++) {
		struct wmt_dma_chan *c = &dmadev->chans[i];

		c->id = i;
		c->dmadev = dmadev;
		c->ccr_reg = dmadev->base + WMT_DMA_CCR_BASE + (i * WMT_DMA_CCR_STRIDE);

		c->vchan.desc_free = wmt_dma_free_desc;
		vchan_init(&c->vchan, &dmadev->ddev);

		irq = platform_get_irq(pdev, i);
		if (irq < 0)
			return irq;

		c->irq = irq;

		name = devm_kasprintf(dev, GFP_KERNEL, "wmt-dma%d", i);
		if (!name)
			return -ENOMEM;

		ret = devm_request_irq(dev, irq, wmt_dma_interrupt, 0, name, c);
		if (ret)
			return ret;
	}

	/* Unmask channel interrupts */
	writel(0xffff, dmadev->base + WMT_DMA_IER);

	ret = dma_async_device_register(&dmadev->ddev);
	if (ret)
		return ret;

	ret = of_dma_controller_register(dev->of_node, wmt_dma_xlate, dmadev);
	if (ret) {
		dma_async_device_unregister(&dmadev->ddev);
		return ret;
	}

	platform_set_drvdata(pdev, dmadev);
	return 0;
}

static void wmt_dma_remove(struct platform_device *pdev)
{
	struct wmt_dma_dev *dmadev = platform_get_drvdata(pdev);
	int i;

	/* Mask and disable the hardware before teardown */
	writel(0, dmadev->base + WMT_DMA_IER);
	writel(0, dmadev->base + WMT_DMA_GCR);

	of_dma_controller_free(pdev->dev.of_node);
	dma_async_device_unregister(&dmadev->ddev);

	for (i = 0; i < WMT_DMA_CHANNELS; i++)
		tasklet_kill(&dmadev->chans[i].vchan.task);
}

static const struct of_device_id wmt_dma_match[] = {
	{ .compatible = "wm,wm8505-dma", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wmt_dma_match);

static struct platform_driver wmt_dma_driver = {
	.probe = wmt_dma_probe,
	.remove = wmt_dma_remove,
	.driver = {
		.name = "wmt-dma",
		.of_match_table = wmt_dma_match,
	},
};
module_platform_driver(wmt_dma_driver);

MODULE_DESCRIPTION("WonderMedia WM8505 DMA Controller Driver");
MODULE_AUTHOR("Logan Russell <me@lrussell.net>");
MODULE_LICENSE("GPL");
