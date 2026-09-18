// SPDX-License-Identifier: GPL-2.0-only
/*
 * WonderMedia WM8505 Scaler (SCL) V4L2 mem2mem driver
 *
 * Copyright (C) 2026 Logan Russell <me@lrussell.net>
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/gcd.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/timer.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>

/* SCL registers */
#define WMT_SCL_EN		0x0
#define WMT_SCL_UPD		0x4
#define WMT_SCL_SEL		0x8
#define WMT_SCL_HSCL_TB(n)	(0xc + 4 * (n))
#define WMT_SCL_HPTR		0x14
#define WMT_SCL_HRES_TB		0x18
#define WMT_SCL_HDIV_TB(n)	(0x1c + 4 * (n))
#define WMT_SCL_HXWIDTH		0x3c
#define WMT_SCL_VSCL_TB		0x40
#define WMT_SCL_VPTR		0x44
#define WMT_SCL_VRES_TB		0x48
#define WMT_SCL_VDIV_TB(n)	(0x4c + 4 * (n))
#define WMT_SCL_VXWIDTH		0x5c
#define WMT_SCL_SCLUP_EN	0x60
#define WMT_SCL_SCLUP_V		BIT(16)
#define WMT_SCL_SCLUP_H		BIT(0)
#define WMT_SCL_CSC(n)		(0x64 + 4 * (n))
#define WMT_SCL_VSCALE1		0x78
#define WMT_SCL_VSCALE2		0x7c
#define WMT_SCL_VSCALE3		0x80
#define WMT_SCL_HSCALE1		0x84
#define WMT_SCL_HSCALE2		0x88
#define WMT_SCL_HSCALE3		0x8c
#define WMT_SCL_CSC_CTL		0x90
#define WMT_SCL_CSC6		0x98
#define WMT_SCL_TG_CTL		0xa0
#define WMT_SCL_TG_RDCYC	GENMASK(23, 16)
#define WMT_SCL_TG_WATCHDOG_EN	BIT(8)
#define WMT_SCL_TG_EN		BIT(0)
#define WMT_SCL_TG_TOTAL	0xa4
#define WMT_SCL_TG_V_ACTIVE	0xa8
#define WMT_SCL_TG_H_ACTIVE	0xac
#define WMT_SCL_TG_VBI		0xb0
#define WMT_SCL_TG_PVBI		GENMASK(12, 8)
#define WMT_SCL_TG_VBIE		GENMASK(6, 0)
#define WMT_SCL_TG_WATCHDOG	0xb4
#define WMT_SCL_TG_STS		0xb8
#define WMT_SCL_TG_ERR		BIT(0)
#define WMT_SCL_TG_GOVW		0xbc
#define WMT_SCLR_CTL		0xc0
#define WMT_SCLR_MIF_EN		BIT(0)
#define WMT_SCLR_NV12		BIT(9)
#define WMT_SCLR_RGB		BIT(11)
#define WMT_SCLR_YSA		0xc4
#define WMT_SCLR_CSA		0xc8
#define WMT_SCLR_H_SIZE		0xcc
#define WMT_SCLR_CROP		0xd0
#define WMT_SCLR_FIFO_CTL	0xd4
#define WMT_SCLW_CTL		0xe0
#define WMT_SCLW_MIF_EN		BIT(0)
#define WMT_SCLW_RGB		BIT(9)
#define WMT_SCLW_YSA		0xe4
#define WMT_SCLW_CSA		0xe8
#define WMT_SCLW_Y_TIME		0xec
#define WMT_SCLW_C_TIME		0xf0
#define WMT_SCLW_FF_CTL		0xf4
#define WMT_SCLW_INT		0xf8

/* Engine initialization and frame timing constants */
#define WMT_SCL_WATCHDOG_MAX	0x1fff
#define WMT_SCLR_FIFO_THR_MAX	0xf
#define WMT_SCL_FF_CTL_CLEAR	0x10101
#define WMT_SCL_HPORCH		100
#define WMT_SCL_VPORCH		8
#define WMT_SCL_PVBI_LINES	10
#define WMT_SCL_VBIE_LINES	4

/* Shared VPP interrupt and reset registers */
#define WMT_VPP_INTSTS		0x4
#define WMT_VPP_INTEN		0x8
#define WMT_VPP_SCL_VBIE	BIT(18)
#define WMT_VPP_SW_RESET	0x10
#define WMT_VPP_SW_RESET_SCL	BIT(0)

/* Scale factors are Q4 */
#define WMT_SCL_STEP_ONE	16

/* Reduction table entries, horizontal and vertical */
#define WMT_SCL_HTAB		32
#define WMT_SCL_VTAB		16

/* Source and destination widths must be a multiple of 32, writer line starts a multiple of 16 */
#define WMT_SCL_HALIGN		32
#define WMT_SCL_XALIGN		16

/* Timing generator cycles per written pixel, and the floor below which reduction fails */
#define WMT_SCL_WRITE_CYCLES	5
#define WMT_SCL_MIN_CYCLES	4

#define WMT_SCL_TIMEOUT_MS	500

#define WMT_SCL_MIN_DIM		WMT_SCL_HALIGN
#define WMT_SCL_MAX_DIM		2048
#define WMT_SCL_DEF_WIDTH	640
#define WMT_SCL_DEF_HEIGHT	480

#define WMT_SCL_CSC_CTL_EN	0x10001

/* JFIF full-range YCbCr to RGB, Q10 fixed point */
static const u32 wmt_scl_csc_jfif[6] = {
	0x00000400, 0x0400059c, 0x1d251ea0, 0x07170400, 0x00010000, 0x00010001
};

struct wmt_scl {
	struct device *dev;
	void __iomem *regs;
	struct regmap *vpp;
	struct v4l2_device v4l2_dev;
	struct video_device vdev;
	struct v4l2_m2m_dev *m2m_dev;
	struct mutex lock;		/* Serializes ioctls and queue operations */
	spinlock_t irq_lock;		/* Protects curr, frames, and the engine registers */
	struct wmt_scl_ctx *curr;
	u32 frames;
	struct timer_list timeout;
	struct clk *clk;		/* Engine clock for pm_runtime */
};

struct wmt_scl_ctx {
	struct v4l2_fh fh;
	struct wmt_scl *scl;
	struct v4l2_pix_format out;
	struct v4l2_pix_format cap;
	struct v4l2_rect crop;
	struct v4l2_rect compose;
	u32 sequence[2];		/* OUTPUT, CAPTURE */
};

#define fh_to_ctx(x)	container_of(x, struct wmt_scl_ctx, fh)

static const u32 wmt_scl_out_formats[] = { V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_XBGR32 };

static u32 wmt_scl_bpp(u32 pixelformat)
{
	return pixelformat == V4L2_PIX_FMT_NV12 ? 1 : 4;
}

static void wmt_scl_copy_color(struct v4l2_pix_format *dst, const struct v4l2_pix_format *src)
{
	dst->colorspace = src->colorspace;
	dst->xfer_func = src->xfer_func;
	dst->ycbcr_enc = src->ycbcr_enc;
	dst->quantization = src->quantization;
}

/*
 * Step a downscale to a ratio the reduction table can hold. A source with none
 * keeps the request and the engine decimates without averaging.
 */
static u32 wmt_scl_fit(u32 src, u32 dst, u32 step, u32 tab)
{
	u32 d;

	for (d = dst; d >= WMT_SCL_MIN_DIM && d < src; d -= step)
		if (src / gcd(src, d) <= tab)
			return d;
	return dst;
}

static void wmt_scl_fill_pix(struct v4l2_pix_format *pix)
{
	u32 bpp = wmt_scl_bpp(pix->pixelformat);
	u32 min_bpl;

	pix->width = clamp(pix->width, WMT_SCL_MIN_DIM, WMT_SCL_MAX_DIM);
	pix->height = clamp(pix->height, WMT_SCL_MIN_DIM, WMT_SCL_MAX_DIM);
	if (bpp == 1)
		pix->height = round_down(pix->height, 2);
	min_bpl = ALIGN(pix->width * bpp, 4);
	if (pix->bytesperline < min_bpl || pix->bytesperline > WMT_SCL_MAX_DIM * bpp)
		pix->bytesperline = min_bpl;
	pix->bytesperline = ALIGN(pix->bytesperline, 4);
	pix->sizeimage = pix->bytesperline * pix->height;
	if (bpp == 1)
		pix->sizeimage += pix->sizeimage / 2;
	pix->field = V4L2_FIELD_NONE;

	/* Unset colorimetry defaults to the JFIF YCbCr the CSC matrix converts */
	if (pix->colorspace == V4L2_COLORSPACE_DEFAULT) {
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		pix->xfer_func = V4L2_XFER_FUNC_SRGB;
		pix->ycbcr_enc = V4L2_YCBCR_ENC_601;
		pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	}
}

static struct v4l2_pix_format *wmt_scl_get_pix(struct wmt_scl_ctx *ctx, enum v4l2_buf_type type)
{
	return V4L2_TYPE_IS_OUTPUT(type) ? &ctx->out : &ctx->cap;
}

static void wmt_scl_whole(struct v4l2_rect *r, const struct v4l2_pix_format *pix)
{
	r->left = 0;
	r->top = 0;
	r->width = pix->width;
	r->height = pix->height;
}

static void wmt_scl_set_compose(struct wmt_scl_ctx *ctx, struct v4l2_rect *r)
{
	r->width = clamp(r->width, WMT_SCL_MIN_DIM, ctx->cap.width);
	r->width = round_down(r->width, WMT_SCL_HALIGN);
	r->width = wmt_scl_fit(ctx->crop.width, r->width, WMT_SCL_HALIGN, WMT_SCL_HTAB);
	r->height = clamp(r->height, WMT_SCL_MIN_DIM, ctx->cap.height);
	r->height = wmt_scl_fit(ctx->crop.height, r->height, 1, WMT_SCL_VTAB);
	r->left = clamp(r->left, 0, (s32)(ctx->cap.width - r->width));
	r->left = round_down(r->left, WMT_SCL_XALIGN);
	r->top = clamp(r->top, 0, (s32)(ctx->cap.height - r->height));
	ctx->compose = *r;
}

static void wmt_scl_set_crop(struct wmt_scl_ctx *ctx, struct v4l2_rect *r)
{
	r->left = 0;
	r->top = 0;
	r->width = clamp(r->width, WMT_SCL_MIN_DIM, ctx->out.width);
	r->width = round_down(r->width, WMT_SCL_HALIGN);
	r->height = clamp(r->height, WMT_SCL_MIN_DIM, ctx->out.height);
	ctx->crop = *r;

	/* A new crop changes both scale ratios the compose was fitted to */
	wmt_scl_set_compose(ctx, &ctx->compose);
}

/* Scale tables */

/* Empty the tables of one axis, leaving the mode in entry 0 */
static void wmt_scl_clear_tables(void __iomem *regs, bool h, u32 mode)
{
	int i;

	if (h) {
		writel(mode, regs + WMT_SCL_HSCL_TB(0));
		writel(0, regs + WMT_SCL_HSCL_TB(1));
		writel(0, regs + WMT_SCL_HPTR);
		writel(0, regs + WMT_SCL_HRES_TB);
		for (i = 0; i < WMT_SCL_HTAB / 4; i++)
			writel(0, regs + WMT_SCL_HDIV_TB(i));
	} else {
		writel(mode, regs + WMT_SCL_VSCL_TB);
		writel(0, regs + WMT_SCL_VPTR);
		writel(0, regs + WMT_SCL_VRES_TB);
		for (i = 0; i < WMT_SCL_VTAB / 4; i++)
			writel(0, regs + WMT_SCL_VDIV_TB(i));
	}
}

/* Program the Bresenham reduction and division tables for downscaling */
static void wmt_scl_set_rt_table(void __iomem *regs, bool h, u32 src, u32 dst)
{
	u32 g = gcd(src, dst);
	u32 a = dst / g, b = src / g;
	u32 scl_tb[2] = { 0 }, res_tb = 0, div_tb[8] = { 0 };
	u32 pre = 0, idx = 0;
	int i, j;

	wmt_scl_clear_tables(regs, h, 1);
	if (b > (h ? WMT_SCL_HTAB : WMT_SCL_VTAB))
		return;

	for (i = 0; i < a; i++) {
		u32 cur = (b * (i + 1)) / a;
		u32 diff = cur - pre;

		pre = cur;
		if (diff == 1) {
			scl_tb[idx / 16] |= 1u << (2 * (idx % 16));
			idx++;
		} else {
			idx++;
			for (j = 0; j < diff - 2; j++) {
				res_tb |= 1u << idx;
				idx++;
			}
			scl_tb[idx / 16] |= 3u << (2 * (idx % 16));
			res_tb |= 1u << idx;
			div_tb[idx / 4] |= (256u / diff) << (8 * (idx % 4));
			idx++;
		}
	}

	if (h) {
		writel(scl_tb[0], regs + WMT_SCL_HSCL_TB(0));
		writel(scl_tb[1], regs + WMT_SCL_HSCL_TB(1));
		writel(res_tb, regs + WMT_SCL_HRES_TB);
		for (i = 0; i < WMT_SCL_HTAB / 4; i++)
			writel(div_tb[i], regs + WMT_SCL_HDIV_TB(i));
		writel(b - 1, regs + WMT_SCL_HPTR);
	} else {
		writel(scl_tb[0], regs + WMT_SCL_VSCL_TB);
		writel(res_tb, regs + WMT_SCL_VRES_TB);
		for (i = 0; i < WMT_SCL_VTAB / 4; i++)
			writel(div_tb[i], regs + WMT_SCL_VDIV_TB(i));
		writel(b - 1, regs + WMT_SCL_VPTR);
	}
}

/* Program the scale step and tables for one axis */
static void wmt_scl_set_axis(void __iomem *regs, bool h, u32 src, u32 dst)
{
	bool up = dst > src;
	u32 step, sub;

	if (up) {
		step = (src - 1) * WMT_SCL_STEP_ONE / dst;
		sub = (src - 1) * WMT_SCL_STEP_ONE % dst;
	} else {
		step = WMT_SCL_STEP_ONE * src / dst;
		sub = WMT_SCL_STEP_ONE * src % dst;
	}

	if (h) {
		writel(max(src, dst), regs + WMT_SCL_HXWIDTH);
		writel(step << 16, regs + WMT_SCL_HSCALE2);
		writel(sub << 16 | dst, regs + WMT_SCL_HSCALE1);
		writel(0, regs + WMT_SCL_HSCALE3);
	} else {
		writel(dst, regs + WMT_SCL_VXWIDTH);
		writel(step << 16, regs + WMT_SCL_VSCALE2);
		writel(sub << 16 | dst, regs + WMT_SCL_VSCALE1);
		writel(0, regs + WMT_SCL_VSCALE3);
	}

	/* Enlargement interpolates, reduction accumulates through the tables */
	if (up)
		wmt_scl_clear_tables(regs, h, 3);
	else
		wmt_scl_set_rt_table(regs, h, src, dst);
}

/* Jobs */

static void wmt_scl_complete(struct wmt_scl_ctx *ctx, enum vb2_buffer_state state)
{
	struct v4l2_m2m_ctx *m2m_ctx = ctx->fh.m2m_ctx;
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(m2m_ctx);

	vb2_set_plane_payload(&dst->vb2_buf, 0, ctx->cap.sizeimage);
	v4l2_m2m_buf_done_and_job_finish(ctx->scl->m2m_dev, m2m_ctx, state);
}

/*
 * The timing generator runs frames back to back. The writer is enabled for the
 * first frame and the pass completes when the second frame starts.
 */
static void wmt_scl_device_run(void *priv)
{
	struct wmt_scl_ctx *ctx = priv;
	struct wmt_scl *scl = ctx->scl;
	void __iomem *regs = scl->regs;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	struct vb2_v4l2_buffer *dst = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	dma_addr_t src_dma = vb2_dma_contig_plane_dma_addr(&src->vb2_buf, 0);
	dma_addr_t dst_dma = vb2_dma_contig_plane_dma_addr(&dst->vb2_buf, 0);
	bool yuv = ctx->out.pixelformat == V4L2_PIX_FMT_NV12;
	u32 sw = ctx->crop.width, sh = ctx->crop.height;
	u32 dw = ctx->compose.width, dh = ctx->compose.height;
	u32 dbpp = wmt_scl_bpp(ctx->cap.pixelformat);
	u32 spitch = ctx->out.bytesperline / wmt_scl_bpp(ctx->out.pixelformat);
	u32 dpitch = ctx->cap.bytesperline / dbpp;
	u32 tw = max(sw, dw), th = max(sh, dh);
	u32 cycles = max(WMT_SCL_MIN_CYCLES, DIV_ROUND_UP(WMT_SCL_WRITE_CYCLES * dw, tw)) + !yuv;
	unsigned long flags;
	int i;

	src->sequence = ctx->sequence[0]++;
	dst->sequence = ctx->sequence[1]++;
	v4l2_m2m_buf_copy_metadata(src, dst, true);
	dst_dma += ctx->compose.top * ctx->cap.bytesperline + ctx->compose.left * dbpp;

	spin_lock_irqsave(&scl->irq_lock, flags);
	scl->curr = ctx;
	scl->frames = 0;

	writel(0, regs + WMT_SCL_SEL);
	writel(0, regs + WMT_SCL_EN);
	writel(0, regs + WMT_SCLW_INT);
	writel(0, regs + WMT_SCLR_CTL);
	writel(WMT_SCL_WATCHDOG_MAX, regs + WMT_SCL_TG_WATCHDOG);
	writel(WMT_SCL_TG_WATCHDOG_EN, regs + WMT_SCL_TG_CTL);
	writel(WMT_SCLR_FIFO_THR_MAX, regs + WMT_SCLR_FIFO_CTL);
	writel(0, regs + WMT_SCL_TG_GOVW);

	/* Reader reads one extra pixel per line for interpolation during enlargement */
	writel(WMT_SCLR_MIF_EN | (yuv ? WMT_SCLR_NV12 : WMT_SCLR_RGB), regs + WMT_SCLR_CTL);
	writel(0, regs + WMT_SCLR_CROP);
	writel((sw + (dw > sw)) << 16 | spitch, regs + WMT_SCLR_H_SIZE);
	writel(src_dma, regs + WMT_SCLR_YSA);
	writel(src_dma + (yuv ? ctx->out.bytesperline * ctx->out.height : 0), regs + WMT_SCLR_CSA);

	/* Writer */
	writel(dst_dma, regs + WMT_SCLW_YSA);
	writel(dst_dma, regs + WMT_SCLW_CSA);
	writel(WMT_SCLW_RGB, regs + WMT_SCLW_CTL);
	writel(dw << 16 | dpitch, regs + WMT_SCLW_Y_TIME);
	writel((dw / 2) << 16 | dpitch, regs + WMT_SCLW_C_TIME);

	wmt_scl_set_axis(regs, true, sw, dw);
	wmt_scl_set_axis(regs, false, sh, dh);
	writel((dh > sh ? WMT_SCL_SCLUP_V : 0) | (dw > sw ? WMT_SCL_SCLUP_H : 0),
	       regs + WMT_SCL_SCLUP_EN);

	/* The last coefficient register sits apart, past the scale registers */
	if (yuv) {
		for (i = 0; i < ARRAY_SIZE(wmt_scl_csc_jfif) - 1; i++)
			writel(wmt_scl_csc_jfif[i], regs + WMT_SCL_CSC(i));
		writel(wmt_scl_csc_jfif[i], regs + WMT_SCL_CSC6);
	}
	writel(yuv ? WMT_SCL_CSC_CTL_EN : 0, regs + WMT_SCL_CSC_CTL);

	/* Timing generator active window and porches */
	writel((th + 2 * WMT_SCL_VPORCH) << 16 | (tw + 2 * WMT_SCL_HPORCH),
	       regs + WMT_SCL_TG_TOTAL);
	writel((th + WMT_SCL_VPORCH) << 16 | WMT_SCL_VPORCH, regs + WMT_SCL_TG_V_ACTIVE);
	writel((tw + WMT_SCL_HPORCH) << 16 | WMT_SCL_HPORCH, regs + WMT_SCL_TG_H_ACTIVE);
	writel(FIELD_PREP(WMT_SCL_TG_PVBI, WMT_SCL_PVBI_LINES) |
	       FIELD_PREP(WMT_SCL_TG_VBIE, WMT_SCL_VBIE_LINES), regs + WMT_SCL_TG_VBI);
	writel(1, regs + WMT_SCL_EN);
	writel(1, regs + WMT_SCL_UPD);

	writel(WMT_SCLW_RGB | WMT_SCLW_MIF_EN, regs + WMT_SCLW_CTL);
	writel(WMT_SCL_TG_ERR, regs + WMT_SCL_TG_STS);

	/* Clear the sticky writer FIFO error bits */
	writel(WMT_SCL_FF_CTL_CLEAR, regs + WMT_SCLW_FF_CTL);
	regmap_write(scl->vpp, WMT_VPP_INTSTS, WMT_VPP_SCL_VBIE);
	regmap_update_bits(scl->vpp, WMT_VPP_INTEN, WMT_VPP_SCL_VBIE, WMT_VPP_SCL_VBIE);
	writel(FIELD_PREP(WMT_SCL_TG_RDCYC, cycles - 1) | WMT_SCL_TG_WATCHDOG_EN | WMT_SCL_TG_EN,
	       regs + WMT_SCL_TG_CTL);
	mod_timer(&scl->timeout, jiffies + msecs_to_jiffies(WMT_SCL_TIMEOUT_MS));
	spin_unlock_irqrestore(&scl->irq_lock, flags);
}

/*
 * Caller holds irq_lock. Completion runs outside the lock so the framework
 * can start the next job.
 */
static struct wmt_scl_ctx *wmt_scl_finish(struct wmt_scl *scl)
{
	struct wmt_scl_ctx *ctx = scl->curr;
	void __iomem *regs = scl->regs;

	regmap_update_bits(scl->vpp, WMT_VPP_INTEN, WMT_VPP_SCL_VBIE, 0);
	writel(WMT_SCLW_RGB, regs + WMT_SCLW_CTL);
	writel(WMT_SCL_TG_WATCHDOG_EN, regs + WMT_SCL_TG_CTL);

	/* Pulse the timing generator once more to work around a memory guard issue */
	writel(WMT_SCL_TG_WATCHDOG_EN | WMT_SCL_TG_EN, regs + WMT_SCL_TG_CTL);
	writel(WMT_SCL_TG_WATCHDOG_EN, regs + WMT_SCL_TG_CTL);
	scl->curr = NULL;
	return ctx;
}

static irqreturn_t wmt_scl_irq(int irq, void *data)
{
	struct wmt_scl *scl = data;
	struct wmt_scl_ctx *ctx = NULL;
	bool err = false;
	u32 en, sts;

	/* The line is shared and INTSTS latches SCL_VBIE even while it is masked */
	regmap_read(scl->vpp, WMT_VPP_INTEN, &en);
	if (!(en & WMT_VPP_SCL_VBIE))
		return IRQ_NONE;

	regmap_read(scl->vpp, WMT_VPP_INTSTS, &sts);
	if (!(sts & WMT_VPP_SCL_VBIE))
		return IRQ_NONE;
	regmap_write(scl->vpp, WMT_VPP_INTSTS, WMT_VPP_SCL_VBIE);

	spin_lock(&scl->irq_lock);
	if (scl->curr && ++scl->frames == 2 && timer_delete(&scl->timeout)) {
		err = readl(scl->regs + WMT_SCL_TG_STS) & WMT_SCL_TG_ERR;
		ctx = wmt_scl_finish(scl);
	}
	spin_unlock(&scl->irq_lock);

	if (ctx)
		wmt_scl_complete(ctx, err ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
	return IRQ_HANDLED;
}

static void wmt_scl_timeout(struct timer_list *t)
{
	struct wmt_scl *scl = from_timer(scl, t, timeout);
	struct wmt_scl_ctx *ctx = NULL;
	unsigned long flags;

	spin_lock_irqsave(&scl->irq_lock, flags);
	if (scl->curr) {
		dev_warn(scl->dev, "pass timed out\n");
		regmap_update_bits(scl->vpp, WMT_VPP_SW_RESET, WMT_VPP_SW_RESET_SCL, 0);
		regmap_update_bits(scl->vpp, WMT_VPP_SW_RESET, WMT_VPP_SW_RESET_SCL,
				   WMT_VPP_SW_RESET_SCL);
		ctx = wmt_scl_finish(scl);
	}
	spin_unlock_irqrestore(&scl->irq_lock, flags);

	if (ctx)
		wmt_scl_complete(ctx, VB2_BUF_STATE_ERROR);
}

static const struct v4l2_m2m_ops wmt_scl_m2m_ops = {
	.device_run = wmt_scl_device_run,
};

/* Videobuf2 queues */

static int wmt_scl_queue_setup(struct vb2_queue *vq, unsigned int *nbuffers,
			       unsigned int *nplanes, unsigned int sizes[],
			       struct device *alloc_devs[])
{
	struct wmt_scl_ctx *ctx = vb2_get_drv_priv(vq);
	u32 size = wmt_scl_get_pix(ctx, vq->type)->sizeimage;

	if (*nplanes)
		return sizes[0] < size ? -EINVAL : 0;
	*nplanes = 1;
	sizes[0] = size;
	return 0;
}

static int wmt_scl_buf_prepare(struct vb2_buffer *vb)
{
	struct wmt_scl_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	u32 size = wmt_scl_get_pix(ctx, vb->vb2_queue->type)->sizeimage;

	if (vb2_plane_size(vb, 0) < size)
		return -EINVAL;
	vbuf->field = V4L2_FIELD_NONE;
	if (V4L2_TYPE_IS_OUTPUT(vb->vb2_queue->type))
		vb2_set_plane_payload(vb, 0, size);
	return 0;
}

static void wmt_scl_buf_queue(struct vb2_buffer *vb)
{
	struct wmt_scl_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static int wmt_scl_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct wmt_scl_ctx *ctx = vb2_get_drv_priv(vq);
	struct vb2_v4l2_buffer *vb;
	int ret;

	ctx->sequence[V4L2_TYPE_IS_CAPTURE(vq->type)] = 0;
	ret = pm_runtime_resume_and_get(ctx->scl->dev);
	if (ret) {
		while ((vb = V4L2_TYPE_IS_OUTPUT(vq->type) ?
			v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
			v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
			v4l2_m2m_buf_done(vb, VB2_BUF_STATE_QUEUED);
	}
	return ret;
}

static void wmt_scl_stop_streaming(struct vb2_queue *vq)
{
	struct wmt_scl_ctx *ctx = vb2_get_drv_priv(vq);
	struct vb2_v4l2_buffer *vb;

	for (;;) {
		vb = V4L2_TYPE_IS_OUTPUT(vq->type) ?
			v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
			v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
		if (!vb)
			break;
		v4l2_m2m_buf_done(vb, VB2_BUF_STATE_ERROR);
	}
	pm_runtime_put(ctx->scl->dev);
}

static const struct vb2_ops wmt_scl_vb2_ops = {
	.queue_setup		= wmt_scl_queue_setup,
	.buf_prepare		= wmt_scl_buf_prepare,
	.buf_queue		= wmt_scl_buf_queue,
	.start_streaming	= wmt_scl_start_streaming,
	.stop_streaming		= wmt_scl_stop_streaming,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
};

static int wmt_scl_queue_init(void *priv, struct vb2_queue *src_vq, struct vb2_queue *dst_vq)
{
	struct wmt_scl_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &wmt_scl_vb2_ops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->scl->lock;
	src_vq->dev = ctx->scl->dev;
	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_DMABUF;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &wmt_scl_vb2_ops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->scl->lock;
	dst_vq->dev = ctx->scl->dev;
	return vb2_queue_init(dst_vq);
}

/* V4L2 ioctls */

static int wmt_scl_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strscpy(cap->driver, "wmt-scl", sizeof(cap->driver));
	strscpy(cap->card, "WM8505 Scaler", sizeof(cap->card));
	return 0;
}

static int wmt_scl_enum_fmt_out(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index >= ARRAY_SIZE(wmt_scl_out_formats))
		return -EINVAL;
	f->pixelformat = wmt_scl_out_formats[f->index];
	return 0;
}

static int wmt_scl_enum_fmt_cap(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_XBGR32;
	return 0;
}

static int wmt_scl_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	f->fmt.pix = *wmt_scl_get_pix(fh_to_ctx(priv), f->type);
	return 0;
}

static int wmt_scl_try_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct wmt_scl_ctx *ctx = fh_to_ctx(priv);
	struct v4l2_pix_format *pix = &f->fmt.pix;

	if (V4L2_TYPE_IS_CAPTURE(f->type) || pix->pixelformat != V4L2_PIX_FMT_NV12)
		pix->pixelformat = V4L2_PIX_FMT_XBGR32;
	wmt_scl_fill_pix(pix);
	if (V4L2_TYPE_IS_CAPTURE(f->type))
		wmt_scl_copy_color(pix, &ctx->out);
	return 0;
}

static int wmt_scl_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct wmt_scl_ctx *ctx = fh_to_ctx(priv);
	struct v4l2_rect r;

	if (vb2_is_busy(v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type)))
		return -EBUSY;
	wmt_scl_try_fmt(file, priv, f);
	*wmt_scl_get_pix(ctx, f->type) = f->fmt.pix;

	/* A new size reads or composes the whole picture */
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		wmt_scl_copy_color(&ctx->cap, &ctx->out);
		wmt_scl_whole(&r, &ctx->out);
		wmt_scl_set_crop(ctx, &r);
	} else {
		wmt_scl_whole(&r, &ctx->cap);
		wmt_scl_set_compose(ctx, &r);
	}
	return 0;
}

static int wmt_scl_g_selection(struct file *file, void *priv, struct v4l2_selection *s)
{
	struct wmt_scl_ctx *ctx = fh_to_ctx(priv);
	bool out = s->type == V4L2_BUF_TYPE_VIDEO_OUTPUT;

	if (!out && s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_COMPOSE:
		if (out ? s->target != V4L2_SEL_TGT_CROP : s->target != V4L2_SEL_TGT_COMPOSE)
			return -EINVAL;
		s->r = out ? ctx->crop : ctx->compose;
		break;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		wmt_scl_whole(&s->r, out ? &ctx->out : &ctx->cap);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int wmt_scl_s_selection(struct file *file, void *priv, struct v4l2_selection *s)
{
	struct wmt_scl_ctx *ctx = fh_to_ctx(priv);
	bool out = s->type == V4L2_BUF_TYPE_VIDEO_OUTPUT;

	if (!out && s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;
	if (s->target != (out ? V4L2_SEL_TGT_CROP : V4L2_SEL_TGT_COMPOSE))
		return -EINVAL;
	if (vb2_is_busy(v4l2_m2m_get_vq(ctx->fh.m2m_ctx, s->type)))
		return -EBUSY;

	if (out)
		wmt_scl_set_crop(ctx, &s->r);
	else
		wmt_scl_set_compose(ctx, &s->r);
	return 0;
}

static const struct v4l2_ioctl_ops wmt_scl_ioctl_ops = {
	.vidioc_querycap		= wmt_scl_querycap,
	.vidioc_enum_fmt_vid_out	= wmt_scl_enum_fmt_out,
	.vidioc_enum_fmt_vid_cap	= wmt_scl_enum_fmt_cap,
	.vidioc_g_fmt_vid_out		= wmt_scl_g_fmt,
	.vidioc_g_fmt_vid_cap		= wmt_scl_g_fmt,
	.vidioc_try_fmt_vid_out		= wmt_scl_try_fmt,
	.vidioc_try_fmt_vid_cap		= wmt_scl_try_fmt,
	.vidioc_s_fmt_vid_out		= wmt_scl_s_fmt,
	.vidioc_s_fmt_vid_cap		= wmt_scl_s_fmt,
	.vidioc_g_selection		= wmt_scl_g_selection,
	.vidioc_s_selection		= wmt_scl_s_selection,
	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf		= v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs		= v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf			= v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon		= v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff		= v4l2_m2m_ioctl_streamoff,
};

/* File operations */

static int wmt_scl_open(struct file *file)
{
	struct wmt_scl *scl = video_drvdata(file);
	struct wmt_scl_ctx *ctx;
	int ret;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	ctx->scl = scl;
	ctx->out.width = WMT_SCL_DEF_WIDTH;
	ctx->out.height = WMT_SCL_DEF_HEIGHT;
	ctx->out.pixelformat = V4L2_PIX_FMT_NV12;
	wmt_scl_fill_pix(&ctx->out);
	ctx->cap = ctx->out;
	ctx->cap.pixelformat = V4L2_PIX_FMT_XBGR32;
	ctx->cap.bytesperline = 0;
	wmt_scl_fill_pix(&ctx->cap);
	wmt_scl_whole(&ctx->crop, &ctx->out);
	wmt_scl_whole(&ctx->compose, &ctx->cap);

	v4l2_fh_init(&ctx->fh, video_devdata(file));
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(scl->m2m_dev, ctx, wmt_scl_queue_init);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		v4l2_fh_exit(&ctx->fh);
		kfree(ctx);
		return ret;
	}
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);
	return 0;
}

static int wmt_scl_release(struct file *file)
{
	struct wmt_scl_ctx *ctx = fh_to_ctx(file->private_data);

	mutex_lock(&ctx->scl->lock);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	mutex_unlock(&ctx->scl->lock);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations wmt_scl_fops = {
	.owner		= THIS_MODULE,
	.open		= wmt_scl_open,
	.release	= wmt_scl_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};

/* Platform driver */

static int wmt_scl_runtime_suspend(struct device *dev)
{
	struct wmt_scl *scl = dev_get_drvdata(dev);

	clk_disable_unprepare(scl->clk);
	return 0;
}

static int wmt_scl_runtime_resume(struct device *dev)
{
	struct wmt_scl *scl = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(scl->clk);
	if (ret)
		return ret;
	writel(WMT_SCL_TG_WATCHDOG_EN, scl->regs + WMT_SCL_TG_CTL);
	return 0;
}

static const struct dev_pm_ops wmt_scl_pm_ops = {
	RUNTIME_PM_OPS(wmt_scl_runtime_suspend, wmt_scl_runtime_resume, NULL)
};

static int wmt_scl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct wmt_scl *scl;
	int irq, ret;

	scl = devm_kzalloc(dev, sizeof(*scl), GFP_KERNEL);
	if (!scl)
		return -ENOMEM;
	scl->dev = dev;
	platform_set_drvdata(pdev, scl);
	mutex_init(&scl->lock);
	spin_lock_init(&scl->irq_lock);
	timer_setup(&scl->timeout, wmt_scl_timeout, 0);

	scl->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(scl->regs))
		return PTR_ERR(scl->regs);

	scl->vpp = syscon_regmap_lookup_by_phandle(dev->of_node, "wm,vpp");
	if (IS_ERR(scl->vpp))
		return dev_err_probe(dev, PTR_ERR(scl->vpp), "failed to get VPP syscon\n");

	scl->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(scl->clk))
		return dev_err_probe(dev, PTR_ERR(scl->clk), "failed to get clock\n");

	regmap_update_bits(scl->vpp, WMT_VPP_INTEN, WMT_VPP_SCL_VBIE, 0);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = devm_request_irq(dev, irq, wmt_scl_irq, IRQF_SHARED, dev_name(dev), scl);
	if (ret)
		return ret;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	ret = v4l2_device_register(dev, &scl->v4l2_dev);
	if (ret)
		return ret;

	scl->m2m_dev = v4l2_m2m_init(&wmt_scl_m2m_ops);
	if (IS_ERR(scl->m2m_dev)) {
		ret = PTR_ERR(scl->m2m_dev);
		goto err_v4l2;
	}

	strscpy(scl->vdev.name, "wmt-scl", sizeof(scl->vdev.name));
	scl->vdev.fops = &wmt_scl_fops;
	scl->vdev.ioctl_ops = &wmt_scl_ioctl_ops;
	scl->vdev.release = video_device_release_empty;
	scl->vdev.lock = &scl->lock;
	scl->vdev.v4l2_dev = &scl->v4l2_dev;
	scl->vdev.vfl_dir = VFL_DIR_M2M;
	scl->vdev.device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	video_set_drvdata(&scl->vdev, scl);
	pm_runtime_enable(dev);

	ret = video_register_device(&scl->vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_pm;

	return 0;

err_pm:
	pm_runtime_disable(dev);
	v4l2_m2m_release(scl->m2m_dev);
err_v4l2:
	v4l2_device_unregister(&scl->v4l2_dev);
	return ret;
}

static void wmt_scl_remove(struct platform_device *pdev)
{
	struct wmt_scl *scl = platform_get_drvdata(pdev);

	v4l2_m2m_suspend(scl->m2m_dev);
	video_unregister_device(&scl->vdev);
	timer_delete_sync(&scl->timeout);
	regmap_update_bits(scl->vpp, WMT_VPP_INTEN, WMT_VPP_SCL_VBIE, 0);
	pm_runtime_disable(&pdev->dev);

	/* Disabling runtime PM leaves an active device clocked */
	if (!pm_runtime_status_suspended(&pdev->dev))
		clk_disable_unprepare(scl->clk);
	v4l2_m2m_release(scl->m2m_dev);
	v4l2_device_unregister(&scl->v4l2_dev);
}

static const struct of_device_id wmt_scl_dt_ids[] = {
	{ .compatible = "wm,wm8505-scl", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wmt_scl_dt_ids);

static struct platform_driver wmt_scl_platform_driver = {
	.probe = wmt_scl_probe,
	.remove = wmt_scl_remove,
	.driver = {
		.name = "wmt-scl",
		.of_match_table = wmt_scl_dt_ids,
		.pm = pm_ptr(&wmt_scl_pm_ops),
	},
};
module_platform_driver(wmt_scl_platform_driver);

MODULE_DESCRIPTION("WonderMedia WM8505 Scaler Driver");
MODULE_AUTHOR("Logan Russell <me@lrussell.net>");
MODULE_LICENSE("GPL");
