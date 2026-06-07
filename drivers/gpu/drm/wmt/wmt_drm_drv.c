// SPDX-License-Identifier: GPL-2.0-only
/*
 * WonderMedia WM8505 DRM/KMS Graphics Driver
 *
 * Copyright (C) 2026 Logan Russell <me@lrussell.net>
 */

#include <linux/align.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/limits.h>
#include <linux/math.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_drv.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_module.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_vblank.h>

#include "wmt_drm.h"

#define DRIVER_DESC "WonderMedia WM8505 DRM Driver"

static const struct drm_ioctl_desc wmt_ioctls[] = {
	DRM_IOCTL_DEF_DRV(WMT_GE_SUBMIT, wmt_drm_ioctl_ge_submit, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(WMT_GE_WAIT, wmt_drm_ioctl_ge_wait, DRM_RENDER_ALLOW),
};

DEFINE_DRM_GEM_DMA_FOPS(wmt_drm_fops);

static int wmt_drm_dumb_create(struct drm_file *file_priv, struct drm_device *drm,
			       struct drm_mode_create_dumb *args)
{
	/* Pitch must be word-aligned for VDMA */
	args->pitch = ALIGN(DIV_ROUND_UP(args->width * args->bpp, 8), 4);
	if (args->pitch > U32_MAX / args->height)
		return -EINVAL;

	return drm_gem_dma_dumb_create_internal(file_priv, drm, args);
}

static const struct drm_driver wmt_drm_driver = {
	.driver_features	= DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC | DRIVER_RENDER,
	.ioctls			= wmt_ioctls,
	.num_ioctls		= ARRAY_SIZE(wmt_ioctls),
	.fops			= &wmt_drm_fops,
	.name			= "wmt-drm",
	.desc			= DRIVER_DESC,
	.major			= 1,
	.minor			= 2,
	DRM_GEM_DMA_DRIVER_OPS_WITH_DUMB_CREATE(wmt_drm_dumb_create),
};

static const struct drm_mode_config_funcs wmt_mode_config_funcs = {
	.fb_create		= drm_gem_fb_create,
	.atomic_check		= drm_atomic_helper_check,
	.atomic_commit		= drm_atomic_helper_commit,
};

static int wmt_drm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct wmt_drm_device *wmt;
	struct device_node *ge_node;
	struct drm_panel *panel;
	struct drm_bridge *bridge;
	struct clk_bulk_data *clks;
	int vpp_irq, vdma_irq, ge_irq, ret;

	wmt = devm_drm_dev_alloc(dev, &wmt_drm_driver, struct wmt_drm_device, drm);
	if (IS_ERR(wmt))
		return PTR_ERR(wmt);
	platform_set_drvdata(pdev, &wmt->drm);

	spin_lock_init(&wmt->ge_lock);
	init_waitqueue_head(&wmt->ge_wait);
	INIT_WORK(&wmt->ge_retire_work, wmt_ge_retire_work);
	INIT_WORK(&wmt->ge_reset_work, wmt_ge_reset_work);

	wmt->govrh_regs = devm_platform_ioremap_resource_byname(pdev, "govrh");
	if (IS_ERR(wmt->govrh_regs))
		return PTR_ERR(wmt->govrh_regs);

	wmt->vpp = syscon_regmap_lookup_by_phandle(dev->of_node, "wm,vpp");
	if (IS_ERR(wmt->vpp))
		return dev_err_probe(dev, PTR_ERR(wmt->vpp), "failed to get VPP syscon\n");

	wmt->vdma_regs = devm_platform_ioremap_resource_byname(pdev, "vdma");
	if (IS_ERR(wmt->vdma_regs))
		return PTR_ERR(wmt->vdma_regs);

	ge_node = of_parse_phandle(dev->of_node, "wm,ge", 0);
	if (!ge_node)
		return dev_err_probe(dev, -ENODEV, "missing wm,ge phandle\n");
	wmt->ge_regs = devm_of_iomap(dev, ge_node, 0, NULL);
	ge_irq = of_irq_get(ge_node, 0);
	of_node_put(ge_node);
	if (IS_ERR(wmt->ge_regs))
		return PTR_ERR(wmt->ge_regs);
	if (ge_irq < 0)
		return ge_irq;

	ret = drm_of_find_panel_or_bridge(dev->of_node, 0, 0, &panel, NULL);
	if (ret)
		return ret;

	ret = devm_clk_bulk_get_all_enable(dev, &clks);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable clocks\n");

	/* The mode set programs the DVO rate */
	wmt->clk_dvo = devm_clk_get(dev, "dvo");
	if (IS_ERR(wmt->clk_dvo))
		return dev_err_probe(dev, PTR_ERR(wmt->clk_dvo), "failed to get DVO clock\n");

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	regmap_update_bits(wmt->vpp, WMT_VPP_INTEN, WMT_VPP_GOVRH_INTEN, 0);
	vpp_irq = platform_get_irq_byname(pdev, "vpp");
	if (vpp_irq < 0)
		return vpp_irq;
	ret = devm_request_irq(dev, vpp_irq, wmt_vblank_irq, IRQF_SHARED, "wmt-drm", wmt);
	if (ret)
		return ret;

	wmt->vdma_desc = dmam_alloc_coherent(dev, WMT_VDMA_DESCS * sizeof(*wmt->vdma_desc),
					     &wmt->vdma_desc_dma, GFP_KERNEL);
	if (!wmt->vdma_desc)
		return -ENOMEM;
	writel(0, wmt->vdma_regs + WMT_VDMA_IER);
	writel(WMT_VDMA_INT, wmt->vdma_regs + WMT_VDMA_ISR);
	vdma_irq = platform_get_irq_byname(pdev, "vdma");
	if (vdma_irq < 0)
		return vdma_irq;
	wmt->vdma_irq = vdma_irq;
	ret = devm_request_irq(dev, vdma_irq, wmt_vdma_irq, 0, "wmt-vdma", wmt);
	if (ret)
		return ret;

	writel(WMT_GE_ENABLE, wmt->ge_regs + WMT_GE_ENG_EN);
	wmt_ge_configure(wmt);
	writel(WMT_GE_INT_CLEAR, wmt->ge_regs + WMT_GE_INT_FLAG);
	wmt->ge_irq = ge_irq;
	ret = devm_request_irq(dev, ge_irq, wmt_ge_irq, 0, "wmt-ge", wmt);
	if (ret)
		return ret;
	writel(WMT_GE_INT_COMPLETE | WMT_GE_INT_TIMEOUT, wmt->ge_regs + WMT_GE_INT_EN);

	ret = devm_add_action_or_reset(dev, wmt_ge_teardown, wmt);
	if (ret)
		return ret;

	ret = drmm_mode_config_init(&wmt->drm);
	if (ret)
		return ret;
	wmt->drm.mode_config.min_width = 1;
	wmt->drm.mode_config.min_height = 1;
	wmt->drm.mode_config.max_width = FIELD_MAX(WMT_GOVRH_WIDTH_MASK);
	wmt->drm.mode_config.max_height = WMT_GE_MAX_DIM;
	wmt->drm.mode_config.funcs = &wmt_mode_config_funcs;

	ret = wmt_pipe_init(wmt);
	if (ret)
		return ret;

	bridge = devm_drm_panel_bridge_add_typed(dev, panel, DRM_MODE_CONNECTOR_DPI);
	if (IS_ERR(bridge))
		return PTR_ERR(bridge);
	ret = drm_simple_display_pipe_attach_bridge(&wmt->pipe, bridge);
	if (ret)
		return ret;

	drm_mode_config_reset(&wmt->drm);

	ret = drm_vblank_init(&wmt->drm, 1);
	if (ret)
		return ret;

	ret = drm_dev_register(&wmt->drm, 0);
	if (ret)
		return ret;

	drm_fbdev_dma_setup(&wmt->drm, 32);

	return 0;
}

static void wmt_drm_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);

	drm_dev_unplug(drm);
	drm_atomic_helper_shutdown(drm);
}

static void wmt_drm_shutdown(struct platform_device *pdev)
{
	drm_atomic_helper_shutdown(platform_get_drvdata(pdev));
}

static const struct of_device_id wmt_drm_dt_ids[] = {
	{ .compatible = "wm,wm8505-drm", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wmt_drm_dt_ids);

static struct platform_driver wmt_drm_platform_driver = {
	.probe = wmt_drm_probe,
	.remove = wmt_drm_remove,
	.shutdown = wmt_drm_shutdown,
	.driver = {
		.name = "wmt-drm",
		.of_match_table = wmt_drm_dt_ids,
	},
};

drm_module_platform_driver(wmt_drm_platform_driver);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR("Logan Russell <me@lrussell.net>");
MODULE_LICENSE("GPL");
