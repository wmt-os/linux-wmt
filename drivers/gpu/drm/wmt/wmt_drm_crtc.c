// SPDX-License-Identifier: GPL-2.0-only
/*
 * WonderMedia WM8505 DRM/KMS Graphics Driver
 *
 * CRTC, Display Pipeline, and VBlank
 *
 * Copyright (C) 2026 Logan Russell <me@lrussell.net>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/limits.h>
#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/spinlock.h>

#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>

#include "wmt_drm.h"

static void wmt_govrh_set_timing(struct wmt_drm_device *wmt,
				 const struct drm_display_mode *mode)
{
	unsigned long t_rate = mode->clock * 1000;
	int h_sync, h_bp, h_fp, h_start, h_end, h_all;
	int v_sync, v_bp, v_fp, v_start, v_end, v_all;

	/* Set pixel clock */
	clk_set_rate(wmt->clk_dvo, t_rate);

	/* Apply the divider */
	writel(min_t(u32, max_t(u32, DIV_ROUND_CLOSEST(clk_get_rate(wmt->clk_dvo), t_rate), 1) - 1,
		     FIELD_MAX(WMT_GOVRH_READ_CYC_MASK)),
	       wmt->govrh_regs + WMT_GOVRH_READ_CYC);

	/* Calculate display geometry offsets */
	h_sync = mode->hsync_end - mode->hsync_start;
	h_bp = mode->htotal - mode->hsync_end;
	h_fp = mode->hsync_start - mode->hdisplay;

	v_sync = mode->vsync_end - mode->vsync_start;
	v_bp = mode->vtotal - mode->vsync_end;
	v_fp = mode->vsync_start - mode->vdisplay;

	h_start = h_sync + h_bp;
	h_end = h_start + mode->hdisplay;
	h_all = h_end + h_fp;

	v_start = v_sync + v_bp + 1;
	v_end = v_start + mode->vdisplay;
	v_all = v_end + v_fp - 1;

	writel(h_start, wmt->govrh_regs + WMT_GOVRH_ACTPX_BG);
	writel(h_end, wmt->govrh_regs + WMT_GOVRH_ACTPX_END);
	writel(h_all, wmt->govrh_regs + WMT_GOVRH_H_ALLPXL);
	writel(min_t(int, h_sync, FIELD_MAX(WMT_GOVRH_HSYNW_MASK)),
	       wmt->govrh_regs + WMT_GOVRH_HSYNW);

	writel(v_start, wmt->govrh_regs + WMT_GOVRH_ACTLN_BG);
	writel(v_end, wmt->govrh_regs + WMT_GOVRH_ACTLN_END);
	writel(v_all, wmt->govrh_regs + WMT_GOVRH_V_ALLLN);

	writel(min_t(int, v_sync + 1, FIELD_MAX(WMT_GOVRH_VBISW_MASK)),
	       wmt->govrh_regs + WMT_GOVRH_VBISW);
	writel(min_t(int, v_sync + 1, FIELD_MAX(WMT_GOVRH_VBIE_LINE_MASK)),
	       wmt->govrh_regs + WMT_GOVRH_VBIE_LINE);

	/* Pre-vblank interrupt fires in the front porch before vsync */
	writel(clamp_t(int, v_fp - 2, 1, FIELD_MAX(WMT_GOVRH_PVBI_LINE_MASK)),
	       wmt->govrh_regs + WMT_GOVRH_PVBI_LINE);
}

irqreturn_t wmt_vblank_irq(int irq, void *data)
{
	struct wmt_drm_device *wmt = data;
	unsigned long flags;
	u32 en, status;

	/* The line is shared and INTSTS latches PVBI even while it is masked */
	regmap_read(wmt->vpp, WMT_VPP_INTEN, &en);
	if (!(en & WMT_VPP_GOVRH_PVBI))
		return IRQ_NONE;

	regmap_read(wmt->vpp, WMT_VPP_INTSTS, &status);
	if (!(status & WMT_VPP_GOVRH_PVBI))
		return IRQ_NONE;

	regmap_write(wmt->vpp, WMT_VPP_INTSTS, WMT_VPP_GOVRH_PVBI);

	drm_crtc_handle_vblank(&wmt->pipe.crtc);

	spin_lock_irqsave(&wmt->drm.event_lock, flags);

	if (wmt->pending_event) {
		drm_crtc_send_vblank_event(&wmt->pipe.crtc, wmt->pending_event);
		drm_crtc_vblank_put(&wmt->pipe.crtc);
		wmt->pending_event = NULL;
	}

	spin_unlock_irqrestore(&wmt->drm.event_lock, flags);

	return IRQ_HANDLED;
}

static enum drm_mode_status wmt_pipe_mode_valid(struct drm_simple_display_pipe *pipe,
						const struct drm_display_mode *mode)
{
	if (mode->hdisplay > FIELD_MAX(WMT_GOVRH_WIDTH_MASK) ||
	    mode->htotal > FIELD_MAX(WMT_GOVRH_TIMING_MASK))
		return MODE_BAD_HVALUE;

	if (mode->vtotal > FIELD_MAX(WMT_GOVRH_TIMING_MASK))
		return MODE_BAD_VVALUE;

	/* Keep the kHz to Hz conversion from overflowing */
	if (mode->clock > INT_MAX / 1000)
		return MODE_CLOCK_HIGH;

	return MODE_OK;
}

static void wmt_pipe_enable(struct drm_simple_display_pipe *pipe,
			    struct drm_crtc_state *crtc_state,
			    struct drm_plane_state *plane_state)
{
	struct wmt_drm_device *wmt = to_wmt_drm(pipe->crtc.dev);
	struct drm_gem_dma_object *gem = drm_fb_dma_get_gem_obj(plane_state->fb, 0);

	/* Disable hardware pipeline */
	writel(0, wmt->govrh_regs + WMT_GOVRH_DVO_SET);
	writel(0, wmt->govrh_regs + WMT_GOVRH_MIF);
	writel(0, wmt->govrh_regs + WMT_GOVRH_TG_ENABLE);
	writel(0, wmt->govrh_regs + WMT_GOVRH_CB_ENABLE);

	/* Set physical buffer address */
	writel(gem->dma_addr, wmt->govrh_regs + WMT_GOVRH_YSA);
	writel(gem->dma_addr, wmt->govrh_regs + WMT_GOVRH_CSA);

	/* Set format to ARGB8888 */
	writel(WMT_GOVRH_RGB_MODE | WMT_GOVRH_DAC_CLKINV | WMT_GOVRH_BLANK_ZERO,
	       wmt->govrh_regs + WMT_GOVRH_YUV2RGB);
	writel(WMT_GOVRH_DVO_RGB, wmt->govrh_regs + WMT_GOVRH_DVO_PIX);

	/* Set resolution boundaries */
	writel(crtc_state->adjusted_mode.hdisplay, wmt->govrh_regs + WMT_GOVRH_PIXWID);
	writel(plane_state->fb->pitches[0] / plane_state->fb->format->cpp[0],
	       wmt->govrh_regs + WMT_GOVRH_BUFWID);

	wmt_govrh_set_timing(wmt, &crtc_state->adjusted_mode);

	/* Set contrast and brightness */
	writel(FIELD_PREP(WMT_GOVRH_CONTRAST_YAF, WMT_GOVRH_CONTRAST_DEFAULT) |
	       FIELD_PREP(WMT_GOVRH_CONTRAST_PBAF, WMT_GOVRH_CONTRAST_DEFAULT) |
	       FIELD_PREP(WMT_GOVRH_CONTRAST_PRAF, WMT_GOVRH_CONTRAST_DEFAULT),
	       wmt->govrh_regs + WMT_GOVRH_CONTRAST);
	writel(0, wmt->govrh_regs + WMT_GOVRH_BRIGHTNESS);

	/* Set FIFO index */
	writel(WMT_GOVRH_FHI_DEFAULT, wmt->govrh_regs + WMT_GOVRH_FHI);

	/* Enable hardware pipeline */
	writel(WMT_GOVRH_REG_UPDATE, wmt->govrh_regs + WMT_GOVRH_REG_STS);
	writel(WMT_GOVRH_TG_EN, wmt->govrh_regs + WMT_GOVRH_TG_ENABLE);
	writel(WMT_GOVRH_MIF_EN, wmt->govrh_regs + WMT_GOVRH_MIF);
	writel(WMT_GOVRH_DVO_ENABLE, wmt->govrh_regs + WMT_GOVRH_DVO_SET);

	/* Let scanout settle before the backlight lights */
	msleep(WMT_GOVRH_SETTLE_MS);

	drm_crtc_vblank_on(&pipe->crtc);
}

static void wmt_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct wmt_drm_device *wmt = to_wmt_drm(pipe->crtc.dev);
	struct drm_crtc *crtc = &pipe->crtc;
	unsigned long flags;

	drm_crtc_vblank_off(crtc);

	/* Disable memory fetch */
	writel(0, wmt->govrh_regs + WMT_GOVRH_MIF);

	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	if (wmt->pending_event) {
		drm_crtc_send_vblank_event(crtc, wmt->pending_event);
		drm_crtc_vblank_put(crtc);
		wmt->pending_event = NULL;
	}
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
}

static int wmt_pipe_enable_vblank(struct drm_simple_display_pipe *pipe)
{
	struct wmt_drm_device *wmt = to_wmt_drm(pipe->crtc.dev);

	regmap_write(wmt->vpp, WMT_VPP_INTSTS, WMT_VPP_GOVRH_PVBI);
	regmap_update_bits(wmt->vpp, WMT_VPP_INTEN, WMT_VPP_GOVRH_PVBI, WMT_VPP_GOVRH_PVBI);

	return 0;
}

static void wmt_pipe_disable_vblank(struct drm_simple_display_pipe *pipe)
{
	struct wmt_drm_device *wmt = to_wmt_drm(pipe->crtc.dev);

	regmap_update_bits(wmt->vpp, WMT_VPP_INTEN, WMT_VPP_GOVRH_PVBI, 0);
}

static void wmt_pipe_update(struct drm_simple_display_pipe *pipe,
			    struct drm_plane_state *old_state)
{
	struct wmt_drm_device *wmt = to_wmt_drm(pipe->crtc.dev);
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_gem_dma_object *gem = state->fb ? drm_fb_dma_get_gem_obj(state->fb, 0) : NULL;
	bool fb_changed = state->fb && state->fb != old_state->fb;
	unsigned long flags;

	/* Ensure GE operations complete before latching address */
	if (fb_changed)
		wmt_ge_latch_drain(wmt, &gem->base);

	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	if (fb_changed) {
		writel(gem->dma_addr, wmt->govrh_regs + WMT_GOVRH_YSA);
		writel(gem->dma_addr, wmt->govrh_regs + WMT_GOVRH_CSA);
		writel(state->src_x >> 16, wmt->govrh_regs + WMT_GOVRH_VCROP);
		writel(state->src_y >> 16, wmt->govrh_regs + WMT_GOVRH_HCROP);
	}

	if (crtc->state->event) {
		wmt->pending_event = crtc->state->event;
		crtc->state->event = NULL;

		if (drm_crtc_vblank_get(crtc) != 0) {
			drm_crtc_send_vblank_event(crtc, wmt->pending_event);
			wmt->pending_event = NULL;
		}
	}
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
}

static const struct drm_simple_display_pipe_funcs wmt_pipe_funcs = {
	.mode_valid		= wmt_pipe_mode_valid,
	.enable			= wmt_pipe_enable,
	.disable		= wmt_pipe_disable,
	.update			= wmt_pipe_update,
	.enable_vblank		= wmt_pipe_enable_vblank,
	.disable_vblank		= wmt_pipe_disable_vblank,
};

static const u32 wmt_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

int wmt_pipe_init(struct wmt_drm_device *wmt)
{
	return drm_simple_display_pipe_init(&wmt->drm, &wmt->pipe, &wmt_pipe_funcs,
					    wmt_formats, ARRAY_SIZE(wmt_formats), NULL, NULL);
}
