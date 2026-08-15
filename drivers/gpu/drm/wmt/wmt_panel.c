// SPDX-License-Identifier: GPL-2.0-only
/*
 * WonderMedia WM8505 DRM/KMS Graphics Driver
 *
 * DPI Panel and Mode Source
 *
 * Copyright (C) 2026 Logan Russell <me@lrussell.net>
 */

#include <linux/bitfield.h>
#include <linux/container_of.h>
#include <linux/err.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/sprintf.h>

#include <drm/drm_connector.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

#include <drm/wmt_drm.h>
#include "wmt_regs.h"

struct wmt_panel {
	struct drm_panel panel;
	struct regulator *supply;
	struct drm_display_mode mode;
};

#define to_wmt_panel(x)		container_of(x, struct wmt_panel, panel)

static char *lcd;
module_param(lcd, charp, 0444);
MODULE_PARM_DESC(lcd, "Panel timings as a vendor lcdparam string");

/* Build a mode from a vendor lcdparam string */
static int wmt_panel_parse(const char *param, struct drm_display_mode *mode)
{
	u32 ver, clk, bpp, xres, yres, hpw, hbp, hfp, vpw, vbp, vfp;

	if (sscanf(param, "%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u", &ver, &clk, &bpp,
		   &xres, &yres, &hpw, &hbp, &hfp, &vpw, &vbp, &vfp) != 11)
		return -EINVAL;

	if (ver != 1 || !clk || !xres || !yres)
		return -EINVAL;

	/* Bound every value by the register that has to hold it, the clock by its Hz conversion */
	if (clk > INT_MAX / 1000 ||
	    xres > FIELD_MAX(WMT_GOVRH_WIDTH_MASK) || yres > WMT_GE_MAX_DIM ||
	    (u64)xres + hpw + hbp + hfp > FIELD_MAX(WMT_GOVRH_TIMING_MASK) ||
	    (u64)yres + vpw + vbp + vfp > FIELD_MAX(WMT_GOVRH_TIMING_MASK))
		return -EINVAL;

	mode->clock = clk;
	mode->hdisplay = xres;
	mode->hsync_start = xres + hfp;
	mode->hsync_end = mode->hsync_start + hpw;
	mode->htotal = mode->hsync_end + hbp;
	mode->vdisplay = yres;
	mode->vsync_start = yres + vfp;
	mode->vsync_end = mode->vsync_start + vpw;
	mode->vtotal = mode->vsync_end + vbp;
	drm_mode_set_name(mode);

	return 0;
}

static int wmt_panel_prepare(struct drm_panel *panel)
{
	return regulator_enable(to_wmt_panel(panel)->supply);
}

static int wmt_panel_unprepare(struct drm_panel *panel)
{
	return regulator_disable(to_wmt_panel(panel)->supply);
}

static int wmt_panel_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	struct wmt_panel *wp = to_wmt_panel(panel);
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &wp->mode);
	if (!mode)
		return 0;

	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs wmt_panel_funcs = {
	.prepare	= wmt_panel_prepare,
	.unprepare	= wmt_panel_unprepare,
	.get_modes	= wmt_panel_get_modes,
};

static int wmt_panel_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct drm_display_mode dt_mode;
	struct wmt_panel *wp;
	const char *src = "cmdline";
	int ret;

	wp = devm_kzalloc(dev, sizeof(*wp), GFP_KERNEL);
	if (!wp)
		return -ENOMEM;
	platform_set_drvdata(pdev, wp);

	wp->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(wp->supply))
		return PTR_ERR(wp->supply);

	/* The DT timing is the fallback */
	ret = of_get_drm_panel_display_mode(dev->of_node, &dt_mode, NULL);
	if (ret)
		return dev_err_probe(dev, ret, "invalid panel-timing\n");

	if (!lcd || wmt_panel_parse(lcd, &wp->mode)) {
		drm_mode_copy(&wp->mode, &dt_mode);
		src = "devicetree";
	}
	wp->mode.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;

	drm_panel_init(&wp->panel, dev, &wmt_panel_funcs, DRM_MODE_CONNECTOR_DPI);

	ret = drm_panel_of_backlight(&wp->panel);
	if (ret)
		return ret;

	drm_panel_add(&wp->panel);
	dev_info(dev, "%s@%dHz from %s\n", wp->mode.name, drm_mode_vrefresh(&wp->mode), src);

	return 0;
}

static void wmt_panel_remove(struct platform_device *pdev)
{
	struct wmt_panel *wp = platform_get_drvdata(pdev);

	drm_panel_remove(&wp->panel);
}

static const struct of_device_id wmt_panel_dt_ids[] = {
	{ .compatible = "wm,wm8505-panel", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wmt_panel_dt_ids);

static struct platform_driver wmt_panel_platform_driver = {
	.probe = wmt_panel_probe,
	.remove = wmt_panel_remove,
	.driver = {
		.name = "wmt-panel",
		.of_match_table = wmt_panel_dt_ids,
	},
};
module_platform_driver(wmt_panel_platform_driver);

MODULE_DESCRIPTION("WonderMedia WM8505 Panel Driver");
MODULE_AUTHOR("Logan Russell <me@lrussell.net>");
MODULE_LICENSE("GPL");
