// SPDX-License-Identifier: GPL-2.0-only
/*
 * WonderMedia WM8505 DRM/KMS Graphics Driver
 *
 * FBCon Hardware Acceleration
 *
 * Copyright (C) 2026 Logan Russell <me@lrussell.net>
 */

#include <linux/fb.h>
#include <linux/printk.h>

#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fb_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_dma_helper.h>

#include "wmt_drm.h"

static void wmt_fb_fillrect(struct fb_info *info, const struct fb_fillrect *rect)
{
	struct drm_fb_helper *fb_helper = info->par;
	struct wmt_drm_device *wmt = to_wmt_drm(fb_helper->dev);
	struct drm_gem_dma_object *gem = drm_fb_dma_get_gem_obj(fb_helper->fb, 0);
	struct drm_wmt_ge_op op = {
		.type		= WMT_GE_OP_FILL,
		.dst_format	= fb_helper->fb->format->format,
		.dst_pitch	= info->fix.line_length,
		.dst_x		= rect->dx,
		.dst_y		= rect->dy,
		.width		= rect->width,
		.height		= rect->height,
		.rop		= (rect->rop == ROP_XOR) ? WMT_GE_ROP_PAT_XOR : WMT_GE_ROP_PAT_COPY,
		.color		= (info->fix.visual == FB_VISUAL_TRUECOLOR ||
				   info->fix.visual == FB_VISUAL_DIRECTCOLOR) ?
				   ((u32 *)info->pseudo_palette)[rect->color] : rect->color
	};
	int ret = -EBUSY;

	if (likely(!oops_in_progress))
		ret = wmt_ge_console_op(wmt, &op, gem);
	if (ret)
		sys_fillrect(info, rect);
}

static void wmt_fb_copyarea(struct fb_info *info, const struct fb_copyarea *area)
{
	struct drm_fb_helper *fb_helper = info->par;
	struct wmt_drm_device *wmt = to_wmt_drm(fb_helper->dev);
	struct drm_gem_dma_object *gem = drm_fb_dma_get_gem_obj(fb_helper->fb, 0);
	struct drm_wmt_ge_op op = {
		.type		= WMT_GE_OP_BLIT,
		.src_format	= fb_helper->fb->format->format,
		.dst_format	= fb_helper->fb->format->format,
		.src_pitch	= info->fix.line_length,
		.dst_pitch	= info->fix.line_length,
		.src_x		= area->sx,
		.src_y		= area->sy,
		.dst_x		= area->dx,
		.dst_y		= area->dy,
		.width		= area->width,
		.height		= area->height,
		.rop		= WMT_GE_ROP_SRC_COPY
	};
	int ret = -EBUSY;

	if (likely(!oops_in_progress))
		ret = wmt_ge_console_op(wmt, &op, gem);
	if (ret)
		sys_copyarea(info, area);
}

static void wmt_fb_imageblit(struct fb_info *info, const struct fb_image *image)
{
	struct drm_fb_helper *fb_helper = info->par;
	struct wmt_drm_device *wmt = to_wmt_drm(fb_helper->dev);

	if (likely(!oops_in_progress))
		wmt_ge_console_idle(wmt);
	sys_imageblit(info, image);
}

/* Inject hardware-accelerated GE operations into the fbdev helper */
void wmt_fbdev_probe_hook(struct drm_fb_helper *fb_helper)
{
	struct fb_ops *wmt_accelerated_fb_ops;

	wmt_accelerated_fb_ops = devm_kmemdup(fb_helper->dev->dev, fb_helper->info->fbops,
					      sizeof(*wmt_accelerated_fb_ops), GFP_KERNEL);
	if (!wmt_accelerated_fb_ops)
		return;

	wmt_accelerated_fb_ops->fb_fillrect = wmt_fb_fillrect;
	wmt_accelerated_fb_ops->fb_copyarea = wmt_fb_copyarea;
	wmt_accelerated_fb_ops->fb_imageblit = wmt_fb_imageblit;

	fb_helper->info->fbops = wmt_accelerated_fb_ops;
	dev_info(fb_helper->dev->dev, "console rendering offloaded to WMT GE\n");
}
