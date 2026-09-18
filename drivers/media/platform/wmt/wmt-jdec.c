// SPDX-License-Identifier: GPL-2.0-only
/*
 * WonderMedia WM8505 JPEG Decoder (JDEC) V4L2 mem2mem driver
 *
 * Copyright (C) 2026 Logan Russell <me@lrussell.net>
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/timer.h>

#include <media/jpeg.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-jpeg.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

#define WMT_JDEC_CLK		0x0
#define WMT_JDEC_CLK_EN		BIT(0)
#define WMT_JDEC_SW_RESET	0x10
#define WMT_JDEC_INT_EN		0x20
#define WMT_JDEC_INT_STS	0x24
#define WMT_JDEC_INT_DONE	BIT(0)
#define WMT_JDEC_INT_ERROR	BIT(4)
#define WMT_JDEC_INT_ALL	GENMASK(4, 0)
#define WMT_JDEC_BSDMA_PRD	0x100
#define WMT_JDEC_BSDMA_FLUSH	0x104
#define WMT_JDEC_BSDMA_START	0x108
#define WMT_JDEC_INIT		0x200
#define WMT_JDEC_SRC_MCU_W	0x210
#define WMT_JDEC_SRC_MCU_H	0x214
#define WMT_JDEC_SRC_FMT	0x218
#define WMT_JDEC_PARTIAL_EN	0x220
#define WMT_JDEC_DST_MCU_W	0x240
#define WMT_JDEC_DST_MCU_H	0x244
#define WMT_JDEC_YBASE		0x250
#define WMT_JDEC_YLINE		0x254
#define WMT_JDEC_CBASE		0x258
#define WMT_JDEC_CLINE		0x25c
#define WMT_JDEC_YSCALE		0x300
#define WMT_JDEC_CSCALE_H	0x304
#define WMT_JDEC_CSCALE_V	0x308
#define WMT_JDEC_CHROMA_EN	0x30c
#define WMT_JDEC_RGB_EN		0x400
#define WMT_JDEC_RGB_ALPHA	0x404
#define WMT_JDEC_RGB_COEF(n)	(0x410 + 4 * (n))
#define WMT_JDEC_RGB_YSUB16	0x42c

/* WMT_JDEC_SRC_FMT chroma layouts */
#define WMT_JDEC_FMT_420	0
#define WMT_JDEC_FMT_422H	1
#define WMT_JDEC_FMT_444	3

/* Output line widths must be a multiple of 64 bytes, heights a whole MCU row */
#define WMT_JDEC_LINE_ALIGN	64
#define WMT_JDEC_MCU_MAX	16
#define WMT_JDEC_MIN_DIM	16
#define WMT_JDEC_MAX_DIM	2048
#define WMT_JDEC_DEF_WIDTH	640
#define WMT_JDEC_DEF_HEIGHT	480

/*
 * Bitstream DMA descriptor: address, then length with bit 31 marking the last
 * entry. Every entry but the last must be 32-byte aligned in address and length.
 */
#define WMT_JDEC_PRD_MAX_LEN	(SZ_64K - 32)
#define WMT_JDEC_PRD_END	BIT(31)
#define WMT_JDEC_PRD_ALIGN	32

/*
 * One page per context: PRD descriptors at the start, then scratch for no-DHT
 * injection, then the tail patch the silicon needs ahead of the EOI.
 */
#define WMT_JDEC_SCRATCH_OFF	768
#define WMT_JDEC_TAIL_OFF	2048

/* Decoder stalls short of frame-done without zero padding ahead of the EOI */
#define WMT_JDEC_EOI_PAD	32

/* Four DHT segments (marker, length, class) carry reference table pairs */
#define WMT_JDEC_DHT_LEN	(4 * 5 + 2 * V4L2_JPEG_REF_HT_DC_LEN + 2 * V4L2_JPEG_REF_HT_AC_LEN)

/*
 * Cap the accepted stream so the worst-case PRD chain fits below the scratch
 * window (65 chunks for 4 MB plus the scratch and tail descriptors, 67 of 96).
 */
#define WMT_JDEC_MAX_STREAM	(SZ_4M - (WMT_JDEC_DHT_LEN + WMT_JDEC_EOI_PAD + 4))

/* A 1280x720 frame of 554 KB decodes in 15 ms; a stalled decode is the only slower case */
#define WMT_JDEC_TIMEOUT_MS	500

/* JFIF full-range YCbCr to RGB, Q10 fixed point */
static const u32 wmt_jdec_csc_jfif[7] = { 0x400, 0x59c, 0x400, 0x2db, 0x160, 0x400, 0x717 };

struct wmt_jdec {
	struct device *dev;
	void __iomem *regs;
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct v4l2_m2m_dev *m2m_dev;
	struct mutex lock;		/* Serializes ioctls and queue operations */
	spinlock_t irq_lock;		/* Protects curr and the decode registers */
	struct wmt_jdec_ctx *curr;
	struct timer_list timeout;
	struct clk *clk;		/* AHB bus clock for pm_runtime */
	int irq;
};

struct wmt_jdec_ctx {
	struct v4l2_fh fh;
	struct wmt_jdec *jdec;
	struct v4l2_pix_format out;
	struct v4l2_pix_format cap;
	u32 sequence;

	/* PRD table and stream tail patch */
	u32 *prd;
	dma_addr_t prd_dma;
};

#define fh_to_ctx(x)	container_of(x, struct wmt_jdec_ctx, fh)

static const u32 wmt_jdec_cap_formats[] = { V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_XBGR32 };

/*
 * wmt_jdec_fill_cap - Capture geometry of the MCU-padded picture
 *
 * The visible size is the compose rectangle.
 */
static void wmt_jdec_fill_cap(struct v4l2_pix_format *pix)
{
	u32 bpp = pix->pixelformat == V4L2_PIX_FMT_NV12 ? 1 : 4;

	pix->width = ALIGN(pix->width, WMT_JDEC_MCU_MAX);
	pix->height = ALIGN(pix->height, WMT_JDEC_MCU_MAX);
	pix->bytesperline = ALIGN(pix->width * bpp, WMT_JDEC_LINE_ALIGN);
	pix->sizeimage = pix->bytesperline * pix->height;
	if (pix->pixelformat == V4L2_PIX_FMT_NV12)
		pix->sizeimage += pix->sizeimage / 2;
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->ycbcr_enc = bpp == 1 ? V4L2_YCBCR_ENC_601 : V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->xfer_func = V4L2_XFER_FUNC_SRGB;
}

/*
 * wmt_jdec_fill_out - Compressed input geometry with requested size
 */
static void wmt_jdec_fill_out(struct v4l2_pix_format *pix)
{
	pix->pixelformat = V4L2_PIX_FMT_JPEG;
	pix->bytesperline = 0;
	if (!pix->sizeimage)
		pix->sizeimage = pix->width * pix->height;
	pix->sizeimage = max_t(u32, pix->sizeimage, SZ_64K);
	pix->sizeimage = min_t(u32, pix->sizeimage, WMT_JDEC_MAX_STREAM);
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_JPEG;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_DEFAULT;
	pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;
}

static struct v4l2_pix_format *wmt_jdec_get_pix(struct wmt_jdec_ctx *ctx, enum v4l2_buf_type type)
{
	return V4L2_TYPE_IS_OUTPUT(type) ? &ctx->out : &ctx->cap;
}

/*
 * wmt_jdec_write_dht - Emit the reference Huffman tables as DHT segments
 */
static void wmt_jdec_write_dht(u8 *p)
{
	static const struct {
		const u8 *table;
		u16 len;
		u8 class_id;
	} tables[4] = {
		{ v4l2_jpeg_ref_table_luma_dc_ht, V4L2_JPEG_REF_HT_DC_LEN,
		  V4L2_JPEG_LUM_HT | V4L2_JPEG_DC_HT },
		{ v4l2_jpeg_ref_table_luma_ac_ht, V4L2_JPEG_REF_HT_AC_LEN,
		  V4L2_JPEG_LUM_HT | V4L2_JPEG_AC_HT },
		{ v4l2_jpeg_ref_table_chroma_dc_ht, V4L2_JPEG_REF_HT_DC_LEN,
		  V4L2_JPEG_CHR_HT | V4L2_JPEG_DC_HT },
		{ v4l2_jpeg_ref_table_chroma_ac_ht, V4L2_JPEG_REF_HT_AC_LEN,
		  V4L2_JPEG_CHR_HT | V4L2_JPEG_AC_HT },
	};
	int i;

	for (i = 0; i < 4; i++) {
		u16 seg_len = tables[i].len + 3;

		*p++ = 0xff;
		*p++ = JPEG_MARKER_DHT;
		*p++ = seg_len >> 8;
		*p++ = seg_len;
		*p++ = tables[i].class_id;
		memcpy(p, tables[i].table, tables[i].len);
		p += tables[i].len;
	}
}

/*
 * wmt_jdec_prd_chain - Append descriptors for a run of the stream
 */
static u32 *wmt_jdec_prd_chain(u32 *prd, dma_addr_t addr, size_t len, bool last)
{
	while (len) {
		size_t chunk = min_t(size_t, len, WMT_JDEC_PRD_MAX_LEN);

		*prd++ = addr;
		*prd++ = chunk | (last && chunk == len ? WMT_JDEC_PRD_END : 0);
		addr += chunk;
		len -= chunk;
	}
	return prd;
}

/*
 * wmt_jdec_put_eoi - Append zero padding and EOI marker
 */
static size_t wmt_jdec_put_eoi(u8 *base, size_t n)
{
	static const u8 eoi[] = { 0xff, JPEG_MARKER_EOI, 0xff, 0xff };
	size_t len;

	memset(base + n, 0, WMT_JDEC_EOI_PAD);
	n += WMT_JDEC_EOI_PAD;
	memcpy(base + n, eoi, sizeof(eoi));
	n += sizeof(eoi);
	len = ALIGN(n, WMT_JDEC_PRD_ALIGN);
	memset(base + n, 0xff, len - n);
	return len;
}

/*
 * wmt_jdec_prepare_stream - Chain the bitstream for the decoder
 *
 * In-place read to the last 32-byte boundary; tail patch carries remainder.
 * Without DHT, reference Huffman tables fill the PRD scratch window; ECS stays
 * in the source buffer for zero-copy DMA.
 */
static int wmt_jdec_prepare_stream(struct wmt_jdec_ctx *ctx, struct vb2_buffer *vb,
				   struct v4l2_jpeg_header *hdr)
{
	const u8 *src = vb2_plane_vaddr(vb, 0);
	dma_addr_t addr = vb2_dma_contig_plane_dma_addr(vb, 0);
	size_t len = vb2_get_plane_payload(vb, 0);
	size_t eoi, cut, scratch_len, stream_off;
	u8 *tail = (u8 *)ctx->prd + WMT_JDEC_TAIL_OFF;
	u8 *scratch = (u8 *)ctx->prd + WMT_JDEC_SCRATCH_OFF;
	u32 *prd = ctx->prd;
	int ret;

	if (!src || !IS_ALIGNED(addr, WMT_JDEC_PRD_ALIGN))
		return -EINVAL;
	memset(hdr, 0, sizeof(*hdr));
	ret = v4l2_jpeg_parse_header((void *)src, len, hdr);
	if (ret < 0)
		return ret;
	if (hdr->frame.precision != 8 || hdr->frame.num_components != 3)
		return -EINVAL;

	/* The stream ends at its EOI, or with the payload when it has none */
	for (eoi = len - 2; eoi > hdr->ecs_offset; eoi--)
		if (src[eoi] == 0xff && src[eoi + 1] == JPEG_MARKER_EOI)
			break;
	if (eoi <= hdr->ecs_offset)
		eoi = len;

	if (hdr->num_dht) {
		cut = round_down(eoi, WMT_JDEC_PRD_ALIGN);
		memcpy(tail, src + cut, eoi - cut);
		prd = wmt_jdec_prd_chain(prd, addr, cut, false);
		wmt_jdec_prd_chain(prd, ctx->prd_dma + WMT_JDEC_TAIL_OFF,
				   wmt_jdec_put_eoi(tail, eoi - cut), true);
		return 0;
	}

	/*
	 * Inject reference Huffman tables via the PRD scratch window for zero-copy.
	 * PRD entries require 32-byte aligned addresses and lengths (except the last).
	 * Copy headers into scratch, then chain from a 32-byte aligned stream offset.
	 */
	memcpy(scratch, src, 2);
	memset(scratch + 2, 0xff, 16);
	wmt_jdec_write_dht(scratch + 18);

	stream_off = ALIGN(hdr->ecs_offset, WMT_JDEC_PRD_ALIGN);
	if (stream_off >= eoi) {
		size_t total = 16 + WMT_JDEC_DHT_LEN + eoi;

		if (total > WMT_JDEC_TAIL_OFF - WMT_JDEC_SCRATCH_OFF)
			return -EINVAL;
		memcpy(scratch + 18 + WMT_JDEC_DHT_LEN, src + 2, eoi - 2);
		scratch_len = ALIGN(total, WMT_JDEC_PRD_ALIGN);
		memset(scratch + total, 0xff, scratch_len - total);
		prd = wmt_jdec_prd_chain(prd, ctx->prd_dma + WMT_JDEC_SCRATCH_OFF,
					 scratch_len, false);
		wmt_jdec_prd_chain(prd, ctx->prd_dma + WMT_JDEC_TAIL_OFF,
				   wmt_jdec_put_eoi(tail, 0), true);
	} else {
		/* 16 + DHT_LEN is a multiple of 32, as is stream_off */
		size_t total = 16 + WMT_JDEC_DHT_LEN + stream_off;

		cut = round_down(eoi, WMT_JDEC_PRD_ALIGN);
		if (total > WMT_JDEC_TAIL_OFF - WMT_JDEC_SCRATCH_OFF)
			return -EINVAL;
		memcpy(scratch + 18 + WMT_JDEC_DHT_LEN, src + 2, stream_off - 2);
		memcpy(tail, src + cut, eoi - cut);
		prd = wmt_jdec_prd_chain(prd, ctx->prd_dma + WMT_JDEC_SCRATCH_OFF, total, false);
		prd = wmt_jdec_prd_chain(prd, addr + stream_off, cut - stream_off, false);
		wmt_jdec_prd_chain(prd, ctx->prd_dma + WMT_JDEC_TAIL_OFF,
				   wmt_jdec_put_eoi(tail, eoi - cut), true);
	}
	return 0;
}

static const struct v4l2_event wmt_jdec_eos = { .type = V4L2_EVENT_EOS };

/*
 * wmt_jdec_complete - Return buffers of current job and flag drain end
 */
static void wmt_jdec_complete(struct wmt_jdec_ctx *ctx, enum vb2_buffer_state state)
{
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(m2m_ctx);

	if (state == VB2_BUF_STATE_DONE)
		vb2_set_plane_payload(&dst->vb2_buf, 0, ctx->cap.sizeimage);
	if (v4l2_m2m_is_last_draining_src_buf(m2m_ctx, v4l2_m2m_next_src_buf(m2m_ctx))) {
		dst->flags |= V4L2_BUF_FLAG_LAST;
		v4l2_m2m_mark_stopped(m2m_ctx);
		v4l2_event_queue_fh(&ctx->fh, &wmt_jdec_eos);
	}
	v4l2_m2m_buf_done_and_job_finish(ctx->jdec->m2m_dev, m2m_ctx, state);
}

/*
 * wmt_jdec_device_run - Program and start one decode
 */
static void wmt_jdec_device_run(void *priv)
{
	struct wmt_jdec_ctx *ctx = priv;
	struct wmt_jdec *jdec = ctx->jdec;
	void __iomem *regs = jdec->regs;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	struct v4l2_jpeg_header hdr;
	bool rgb = ctx->cap.pixelformat == V4L2_PIX_FMT_XBGR32;
	dma_addr_t dst_dma = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0);
	u32 mcu_w = 8, mcu_h = 8, cscale_h = 0, cscale_v = 0, fmt;
	unsigned long flags;
	int i, ret;

	ret = wmt_jdec_prepare_stream(ctx, &src->vb2_buf, &hdr);
	if (!ret && (hdr.frame.width != ctx->out.width || hdr.frame.height != ctx->out.height))
		ret = -EINVAL;
	if (!ret) {
		switch (hdr.frame.subsampling) {
		case V4L2_JPEG_CHROMA_SUBSAMPLING_420:
			fmt = WMT_JDEC_FMT_420;
			mcu_w = 16;
			mcu_h = 16;
			break;
		case V4L2_JPEG_CHROMA_SUBSAMPLING_422:
			fmt = WMT_JDEC_FMT_422H;
			mcu_w = 16;
			cscale_v = 1;
			break;
		case V4L2_JPEG_CHROMA_SUBSAMPLING_444:
			fmt = WMT_JDEC_FMT_444;
			cscale_h = 1;
			cscale_v = 1;
			break;
		default:
			ret = -EINVAL;
		}
	}
	if (ret) {
		wmt_jdec_complete(ctx, VB2_BUF_STATE_ERROR);
		return;
	}

	src->sequence = ctx->sequence;
	dst->sequence = ctx->sequence++;
	v4l2_m2m_buf_copy_metadata(src, dst, true);

	spin_lock_irqsave(&jdec->irq_lock, flags);
	jdec->curr = ctx;

	writel(1, regs + WMT_JDEC_SW_RESET);
	writel(0, regs + WMT_JDEC_SW_RESET);
	writel(WMT_JDEC_INT_ALL, regs + WMT_JDEC_INT_STS);
	writel(0, regs + WMT_JDEC_PARTIAL_EN);

	writel(DIV_ROUND_UP(hdr.frame.width, mcu_w), regs + WMT_JDEC_SRC_MCU_W);
	writel(DIV_ROUND_UP(hdr.frame.height, mcu_h), regs + WMT_JDEC_SRC_MCU_H);
	writel(fmt, regs + WMT_JDEC_SRC_FMT);
	writel(DIV_ROUND_UP(hdr.frame.width, mcu_w), regs + WMT_JDEC_DST_MCU_W);
	writel(DIV_ROUND_UP(hdr.frame.height, mcu_h), regs + WMT_JDEC_DST_MCU_H);

	/* Chroma is decimated to 4:2:0 for NV12; RGB conversion uses decoded chroma */
	writel(0, regs + WMT_JDEC_YSCALE);
	writel(rgb ? 0 : cscale_h, regs + WMT_JDEC_CSCALE_H);
	writel(rgb ? 0 : cscale_v, regs + WMT_JDEC_CSCALE_V);
	writel(0, regs + WMT_JDEC_CHROMA_EN);

	if (rgb) {
		for (i = 0; i < ARRAY_SIZE(wmt_jdec_csc_jfif); i++)
			writel(wmt_jdec_csc_jfif[i], regs + WMT_JDEC_RGB_COEF(i));
		writel(0, regs + WMT_JDEC_RGB_YSUB16);
		writel(0, regs + WMT_JDEC_RGB_ALPHA);
	}
	writel(rgb, regs + WMT_JDEC_RGB_EN);

	writel(ctx->cap.bytesperline, regs + WMT_JDEC_YLINE);
	writel(rgb ? 0 : ctx->cap.bytesperline, regs + WMT_JDEC_CLINE);
	writel(dst_dma, regs + WMT_JDEC_YBASE);
	writel(rgb ? 0 : dst_dma + ctx->cap.bytesperline * ctx->cap.height, regs + WMT_JDEC_CBASE);

	writel(0, regs + WMT_JDEC_BSDMA_FLUSH);
	writel(ctx->prd_dma, regs + WMT_JDEC_BSDMA_PRD);

	writel(1, regs + WMT_JDEC_INIT);
	writel(0, regs + WMT_JDEC_INIT);
	writel(WMT_JDEC_INT_DONE | WMT_JDEC_INT_ERROR, regs + WMT_JDEC_INT_EN);
	writel(1, regs + WMT_JDEC_BSDMA_START);
	mod_timer(&jdec->timeout, jiffies + msecs_to_jiffies(WMT_JDEC_TIMEOUT_MS));
	spin_unlock_irqrestore(&jdec->irq_lock, flags);
}

/*
 * wmt_jdec_finish - Detach running job from engine (caller holds irq_lock)
 *
 * Completion runs outside the lock so the framework can start the next job.
 */
static struct wmt_jdec_ctx *wmt_jdec_finish(struct wmt_jdec *jdec)
{
	struct wmt_jdec_ctx *ctx = jdec->curr;

	writel(0, jdec->regs + WMT_JDEC_INT_EN);
	writel(0, jdec->regs + WMT_JDEC_BSDMA_START);
	jdec->curr = NULL;
	return ctx;
}

/*
 * wmt_jdec_irq - Interrupt handler for decode completion and errors
 */
static irqreturn_t wmt_jdec_irq(int irq, void *data)
{
	struct wmt_jdec *jdec = data;
	struct wmt_jdec_ctx *ctx = NULL;
	u32 sts;

	if (pm_runtime_get_if_active(jdec->dev) <= 0)
		return IRQ_NONE;

	sts = readl(jdec->regs + WMT_JDEC_INT_STS);
	if (!(sts & (WMT_JDEC_INT_DONE | WMT_JDEC_INT_ERROR))) {
		pm_runtime_put(jdec->dev);
		return IRQ_NONE;
	}
	writel(sts, jdec->regs + WMT_JDEC_INT_STS);

	spin_lock(&jdec->irq_lock);
	if (jdec->curr) {
		timer_delete(&jdec->timeout);
		ctx = wmt_jdec_finish(jdec);
	}
	spin_unlock(&jdec->irq_lock);

	pm_runtime_put(jdec->dev);

	if (ctx)
		wmt_jdec_complete(ctx, sts & WMT_JDEC_INT_ERROR ? VB2_BUF_STATE_ERROR :
				  VB2_BUF_STATE_DONE);
	return IRQ_HANDLED;
}

/*
 * wmt_jdec_timeout - Handle decode timeout and reset engine
 */
static void wmt_jdec_timeout(struct timer_list *t)
{
	struct wmt_jdec *jdec = from_timer(jdec, t, timeout);
	struct wmt_jdec_ctx *ctx = NULL;
	unsigned long flags;

	spin_lock_irqsave(&jdec->irq_lock, flags);
	if (jdec->curr) {
		dev_warn(jdec->dev, "Decode timed out\n");
		writel(1, jdec->regs + WMT_JDEC_SW_RESET);
		writel(0, jdec->regs + WMT_JDEC_SW_RESET);
		ctx = wmt_jdec_finish(jdec);
	}
	spin_unlock_irqrestore(&jdec->irq_lock, flags);

	if (ctx)
		wmt_jdec_complete(ctx, VB2_BUF_STATE_ERROR);
}

/*
 * wmt_jdec_job_abort - Terminate a running job on streamoff
 *
 * Software reset clears engine state; completion is called outside irq_lock.
 */
static void wmt_jdec_job_abort(void *priv)
{
	struct wmt_jdec_ctx *ctx = priv;
	struct wmt_jdec *jdec = ctx->jdec;
	struct wmt_jdec_ctx *finished_ctx = NULL;
	unsigned long flags;

	spin_lock_irqsave(&jdec->irq_lock, flags);
	if (jdec->curr == ctx) {
		timer_delete(&jdec->timeout);
		writel(1, jdec->regs + WMT_JDEC_SW_RESET);
		writel(0, jdec->regs + WMT_JDEC_SW_RESET);
		finished_ctx = wmt_jdec_finish(jdec);
	}
	spin_unlock_irqrestore(&jdec->irq_lock, flags);

	if (finished_ctx)
		wmt_jdec_complete(finished_ctx, VB2_BUF_STATE_ERROR);
}

static const struct v4l2_m2m_ops wmt_jdec_m2m_ops = {
	.device_run = wmt_jdec_device_run,
	.job_abort = wmt_jdec_job_abort,
};

/* Videobuf2 queues */

static int wmt_jdec_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
				unsigned int *nplanes, unsigned int sizes[],
				struct device *alloc_devs[])
{
	struct wmt_jdec_ctx *ctx = vb2_get_drv_priv(vq);
	u32 size = wmt_jdec_get_pix(ctx, vq->type)->sizeimage;

	*nbuffers = clamp_val(*nbuffers, 2, 8);
	if (*nplanes)
		return sizes[0] < size ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = size;
	return 0;
}

static int wmt_jdec_buf_prepare(struct vb2_buffer *vb)
{
	struct wmt_jdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	u32 size = wmt_jdec_get_pix(ctx, vb->vb2_queue->type)->sizeimage;

	if (vb2_plane_size(vb, 0) < size)
		return -EINVAL;
	if (vbuf->field == V4L2_FIELD_ANY)
		vbuf->field = V4L2_FIELD_NONE;
	if (vbuf->field != V4L2_FIELD_NONE)
		return -EINVAL;
	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type)) {
		if (!vb2_get_plane_payload(vb, 0) || vb2_get_plane_payload(vb, 0) > size)
			return -EINVAL;
	}
	return 0;
}

static void wmt_jdec_buf_queue(struct vb2_buffer *vb)
{
	struct wmt_jdec_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	if (V4L2_TYPE_IS_CAPTURE(vb->vb2_queue->type) && vb2_is_streaming(vb->vb2_queue) &&
	    v4l2_m2m_dst_buf_is_last(ctx->fh.m2m_ctx)) {
		vb2_set_plane_payload(vb, 0, 0);
		v4l2_m2m_last_buffer_done(ctx->fh.m2m_ctx, vbuf);
		v4l2_event_queue_fh(&ctx->fh, &wmt_jdec_eos);
		return;
	}
	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}

static int wmt_jdec_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct wmt_jdec_ctx *ctx = vb2_get_drv_priv(vq);
	struct vb2_v4l2_buffer *vb;
	int ret;

	ret = pm_runtime_resume_and_get(ctx->jdec->dev);
	if (ret < 0) {
		while ((vb = V4L2_TYPE_IS_OUTPUT(vq->type) ?
			v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
			v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
			v4l2_m2m_buf_done(vb, VB2_BUF_STATE_QUEUED);
		return ret;
	}
	v4l2_m2m_update_start_streaming_state(ctx->fh.m2m_ctx, vq);

	/* Drain residue survives an OUTPUT restart, gating jobs and CAPTURE DQBUF */
	if (V4L2_TYPE_IS_OUTPUT(vq->type) &&
	    (v4l2_m2m_has_stopped(ctx->fh.m2m_ctx) ||
	     v4l2_m2m_dst_buf_is_last(ctx->fh.m2m_ctx))) {
		v4l2_m2m_clear_state(ctx->fh.m2m_ctx);
		vb2_clear_last_buffer_dequeued(&ctx->fh.m2m_ctx->cap_q_ctx.q);
	}
	return 0;
}

static void wmt_jdec_stop_streaming(struct vb2_queue *vq)
{
	struct wmt_jdec_ctx *ctx = vb2_get_drv_priv(vq);
	struct vb2_v4l2_buffer *vb;

	for (;;) {
		vb = V4L2_TYPE_IS_OUTPUT(vq->type) ?
			v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
			v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!vb)
			break;
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
	}
	v4l2_m2m_update_stop_streaming_state(ctx->fh.m2m_ctx, vq);
	if (V4L2_TYPE_IS_OUTPUT(vq->type) && v4l2_m2m_has_stopped(ctx->fh.m2m_ctx))
		v4l2_event_queue_fh(&ctx->fh, &wmt_jdec_eos);
	pm_runtime_put(ctx->jdec->dev);
}

static const struct vb2_ops wmt_jdec_vb2_ops = {
	.queue_setup		= wmt_jdec_queue_setup,
	.buf_prepare		= wmt_jdec_buf_prepare,
	.buf_queue		= wmt_jdec_buf_queue,
	.start_streaming	= wmt_jdec_start_streaming,
	.stop_streaming		= wmt_jdec_stop_streaming,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

static int wmt_jdec_queue_init(void *priv, struct vb2_queue *src_vq, struct vb2_queue *dst_vq)
{
	struct wmt_jdec_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &wmt_jdec_vb2_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->jdec->lock;
	src_vq->dev = ctx->jdec->dev;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &wmt_jdec_vb2_ops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->jdec->lock;
	dst_vq->dev = ctx->jdec->dev;
	dst_vq->min_reqbufs_allocation = 2;
	return vb2_queue_init(dst_vq);
}

/* V4L2 ioctls */

static int wmt_jdec_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strscpy(cap->driver, "wmt-jdec", sizeof(cap->driver));
	strscpy(cap->card, "WM8505 JPEG Decoder", sizeof(cap->card));
	return 0;
}

static int wmt_jdec_enum_fmt_out(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_JPEG;
	return 0;
}

static int wmt_jdec_enum_fmt_cap(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(wmt_jdec_cap_formats))
		return -EINVAL;
	f->pixelformat = wmt_jdec_cap_formats[f->index];
	return 0;
}

static int wmt_jdec_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	f->fmt.pix = *wmt_jdec_get_pix(fh_to_ctx(priv), f->type);
	return 0;
}

static int wmt_jdec_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct wmt_jdec_ctx *ctx = fh_to_ctx(priv);
	struct v4l2_pix_format *pix = &f->fmt.pix;
	unsigned int i;

	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		pix->width = clamp(pix->width, WMT_JDEC_MIN_DIM, WMT_JDEC_MAX_DIM);
		pix->height = clamp(pix->height, WMT_JDEC_MIN_DIM, WMT_JDEC_MAX_DIM);
		wmt_jdec_fill_out(pix);
	} else {
		/* The decode is 1:1; CAPTURE geometry follows the coded size */
		pix->width = ctx->out.width;
		pix->height = ctx->out.height;
		for (i = 0; i < ARRAY_SIZE(wmt_jdec_cap_formats); i++)
			if (pix->pixelformat == wmt_jdec_cap_formats[i])
				break;
		if (i == ARRAY_SIZE(wmt_jdec_cap_formats))
			pix->pixelformat = wmt_jdec_cap_formats[0];
		wmt_jdec_fill_cap(pix);
	}
	return 0;
}

static int wmt_jdec_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct wmt_jdec_ctx *ctx = fh_to_ctx(priv);

	if (vb2_is_busy(v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type)))
		return -EBUSY;
	wmt_jdec_try_fmt(file, priv, f);
	*wmt_jdec_get_pix(ctx, f->type) = f->fmt.pix;

	/* Propagate coded size to CAPTURE queue */
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		ctx->cap.width = f->fmt.pix.width;
		ctx->cap.height = f->fmt.pix.height;
		wmt_jdec_fill_cap(&ctx->cap);
	}
	return 0;
}

static int wmt_jdec_g_selection(struct file *file, void *priv, struct v4l2_selection *s)
{
	struct wmt_jdec_ctx *ctx = fh_to_ctx(priv);

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		s->r.width = ctx->out.width;
		s->r.height = ctx->out.height;
		break;
	case V4L2_SEL_TGT_COMPOSE_PADDED:
		s->r.width = ctx->cap.width;
		s->r.height = ctx->cap.height;
		break;
	default:
		return -EINVAL;
	}
	s->r.left = 0;
	s->r.top = 0;
	return 0;
}

static int wmt_jdec_subscribe_event(struct v4l2_fh *fh, const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_EOS:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	default:
		return v4l2_ctrl_subscribe_event(fh, sub);
	}
}

/*
 * wmt_jdec_decoder_cmd - Wrapper that posts EOS deterministically on STOP
 */
static int wmt_jdec_decoder_cmd(struct file *file, void *fh, struct v4l2_decoder_cmd *cmd)
{
	struct wmt_jdec_ctx *ctx = fh_to_ctx(fh);
	int ret;

	ret = v4l2_m2m_ioctl_try_decoder_cmd(file, fh, cmd);
	if (ret)
		return ret;
	if (!vb2_is_streaming(v4l2_m2m_get_vq(ctx->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT)))
		return 0;
	ret = v4l2_m2m_ioctl_decoder_cmd(file, fh, cmd);
	if (ret)
		return ret;
	if (cmd->cmd == V4L2_DEC_CMD_STOP && v4l2_m2m_has_stopped(ctx->fh.m2m_ctx))
		v4l2_event_queue_fh(&ctx->fh, &wmt_jdec_eos);
	if (cmd->cmd == V4L2_DEC_CMD_START)
		vb2_clear_last_buffer_dequeued(&ctx->fh.m2m_ctx->cap_q_ctx.q);
	return 0;
}

static const struct v4l2_ioctl_ops wmt_jdec_ioctl_ops = {
	.vidioc_querycap		= wmt_jdec_querycap,
	.vidioc_enum_fmt_vid_out	= wmt_jdec_enum_fmt_out,
	.vidioc_enum_fmt_vid_cap	= wmt_jdec_enum_fmt_cap,
	.vidioc_g_fmt_vid_out		= wmt_jdec_g_fmt,
	.vidioc_g_fmt_vid_cap		= wmt_jdec_g_fmt,
	.vidioc_try_fmt_vid_out		= wmt_jdec_try_fmt,
	.vidioc_try_fmt_vid_cap		= wmt_jdec_try_fmt,
	.vidioc_s_fmt_vid_out		= wmt_jdec_s_fmt,
	.vidioc_s_fmt_vid_cap		= wmt_jdec_s_fmt,
	.vidioc_g_selection		= wmt_jdec_g_selection,
	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf		= v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs		= v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf			= v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon		= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_m2m_ioctl_streamoff,
	.vidioc_try_decoder_cmd		= v4l2_m2m_ioctl_try_decoder_cmd,
	.vidioc_decoder_cmd		= wmt_jdec_decoder_cmd,
	.vidioc_subscribe_event		= wmt_jdec_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
};

/* File operations */

static int wmt_jdec_open(struct file *file)
{
	struct wmt_jdec *jdec = video_drvdata(file);
	struct wmt_jdec_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->jdec = jdec;
	ctx->prd = dma_alloc_coherent(jdec->dev, PAGE_SIZE, &ctx->prd_dma, GFP_KERNEL);
	if (!ctx->prd) {
		kfree(ctx);
		return -ENOMEM;
	}
	ctx->out.width = WMT_JDEC_DEF_WIDTH;
	ctx->out.height = WMT_JDEC_DEF_HEIGHT;
	wmt_jdec_fill_out(&ctx->out);
	ctx->cap.width = ctx->out.width;
	ctx->cap.height = ctx->out.height;
	ctx->cap.pixelformat = V4L2_PIX_FMT_NV12;
	wmt_jdec_fill_cap(&ctx->cap);

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(jdec->m2m_dev, ctx, wmt_jdec_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		v4l2_fh_exit(&ctx->fh);
		dma_free_coherent(jdec->dev, PAGE_SIZE, ctx->prd, ctx->prd_dma);
		kfree(ctx);
		return ret;
	}
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);
	return 0;
}

static int wmt_jdec_release(struct file *file)
{
	struct wmt_jdec_ctx *ctx = fh_to_ctx(file->private_data);

	mutex_lock(&ctx->jdec->lock);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	mutex_unlock(&ctx->jdec->lock);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	dma_free_coherent(ctx->jdec->dev, PAGE_SIZE, ctx->prd, ctx->prd_dma);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations wmt_jdec_fops = {
	.owner		= THIS_MODULE,
	.open		= wmt_jdec_open,
	.release	= wmt_jdec_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

/* Platform driver */

static int wmt_jdec_runtime_suspend(struct device *dev)
{
	struct wmt_jdec *jdec = dev_get_drvdata(dev);

	clk_disable_unprepare(jdec->clk);
	return 0;
}

static int wmt_jdec_runtime_resume(struct device *dev)
{
	struct wmt_jdec *jdec = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(jdec->clk);
	if (ret)
		return ret;
	writel(WMT_JDEC_CLK_EN, jdec->regs + WMT_JDEC_CLK);
	return 0;
}

/*
 * Quiesce m2m before the AHB clock gates. A scheduled job or the armed
 * watchdog would otherwise touch registers with the clock off.
 */
static int wmt_jdec_suspend(struct device *dev)
{
	struct wmt_jdec *jdec = dev_get_drvdata(dev);

	v4l2_m2m_suspend(jdec->m2m_dev);
	return pm_runtime_force_suspend(dev);
}

static int wmt_jdec_resume(struct device *dev)
{
	struct wmt_jdec *jdec = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret < 0)
		return ret;

	v4l2_m2m_resume(jdec->m2m_dev);
	return 0;
}

static const struct dev_pm_ops wmt_jdec_pm_ops = {
	RUNTIME_PM_OPS(wmt_jdec_runtime_suspend, wmt_jdec_runtime_resume, NULL)
	SYSTEM_SLEEP_PM_OPS(wmt_jdec_suspend, wmt_jdec_resume)
};

static int wmt_jdec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct wmt_jdec *jdec;
	int irq, ret;

	jdec = devm_kzalloc(dev, sizeof(*jdec), GFP_KERNEL);
	if (!jdec)
		return -ENOMEM;
	jdec->dev = dev;
	mutex_init(&jdec->lock);
	spin_lock_init(&jdec->irq_lock);
	timer_setup(&jdec->timeout, wmt_jdec_timeout, 0);

	jdec->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(jdec->regs))
		return PTR_ERR(jdec->regs);

	jdec->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(jdec->clk))
		return dev_err_probe(dev, PTR_ERR(jdec->clk), "Failed to get clock\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	jdec->irq = irq;
	ret = devm_request_irq(dev, irq, wmt_jdec_irq, 0, dev_name(dev), jdec);
	if (ret)
		return ret;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	ret = v4l2_device_register(dev, &jdec->v4l2_dev);
	if (ret)
		return ret;

	jdec->m2m_dev = v4l2_m2m_init(&wmt_jdec_m2m_ops);
	if (IS_ERR(jdec->m2m_dev)) {
		ret = PTR_ERR(jdec->m2m_dev);
		goto err_v4l2;
	}

	strscpy(jdec->vdev.name, "wmt-jdec", sizeof(jdec->vdev.name));
	jdec->vdev.fops = &wmt_jdec_fops;
	jdec->vdev.ioctl_ops = &wmt_jdec_ioctl_ops;
	jdec->vdev.release = video_device_release_empty;
	jdec->vdev.lock = &jdec->lock;
	jdec->vdev.v4l2_dev = &jdec->v4l2_dev;
	jdec->vdev.vfl_dir = VFL_DIR_M2M;
	jdec->vdev.device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	video_set_drvdata(&jdec->vdev, jdec);
	platform_set_drvdata(pdev, jdec);

	ret = video_register_device(&jdec->vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_m2m;

	pm_runtime_enable(dev);

	return 0;

err_m2m:
	v4l2_m2m_release(jdec->m2m_dev);
err_v4l2:
	v4l2_device_unregister(&jdec->v4l2_dev);
	return ret;
}

static void wmt_jdec_remove(struct platform_device *pdev)
{
	struct wmt_jdec *jdec = platform_get_drvdata(pdev);

	v4l2_m2m_suspend(jdec->m2m_dev);
	video_unregister_device(&jdec->vdev);
	devm_free_irq(&pdev->dev, jdec->irq, jdec);
	pm_runtime_get_sync(&pdev->dev);
	timer_delete_sync(&jdec->timeout);
	writel(0, jdec->regs + WMT_JDEC_CLK);
	pm_runtime_put(&pdev->dev);
	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		clk_disable_unprepare(jdec->clk);
	v4l2_m2m_release(jdec->m2m_dev);
	v4l2_device_unregister(&jdec->v4l2_dev);
}

static const struct of_device_id wmt_jdec_dt_ids[] = {
	{ .compatible = "wm,wm8505-jdec", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wmt_jdec_dt_ids);

static struct platform_driver wmt_jdec_platform_driver = {
	.probe = wmt_jdec_probe,
	.remove = wmt_jdec_remove,
	.driver = {
		.name = "wmt-jdec",
		.of_match_table = wmt_jdec_dt_ids,
		.pm = pm_ptr(&wmt_jdec_pm_ops),
	},
};
module_platform_driver(wmt_jdec_platform_driver);

MODULE_DESCRIPTION("WonderMedia WM8505 JPEG Decoder Driver");
MODULE_AUTHOR("Logan Russell <me@lrussell.net>");
MODULE_LICENSE("GPL");
