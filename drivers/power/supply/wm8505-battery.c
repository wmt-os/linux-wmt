// SPDX-License-Identifier: GPL-2.0-only
/*
 * WonderMedia WM8505 Battery Driver
 *
 * Copyright (C) 2026 Logan Russell <me@lrussell.net>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/workqueue.h>

#define WM8505_BATTERY_DISCHARGE_MS	250
#define WM8505_BATTERY_RISE_TIMEOUT_US	1000000
#define WM8505_BATTERY_SAMPLES		3
#define WM8505_BATTERY_AVG_WINDOW	4
#define WM8505_BATTERY_POLL		(5 * HZ)
#define WM8505_BATTERY_SAMPLE_INTERVAL	(60 * HZ)
#define WM8505_BATTERY_SETTLED_FRAC	32
#define WM8505_BATTERY_TEMP_C		25	/* No temp sensor; the OCV table's reference */

struct wm8505_battery {
	struct power_supply *psy;
	struct power_supply_battery_info *info;
	struct delayed_work work;
	struct mutex lock;	/* Protects the state fields */
	struct gpio_desc *sense;
	struct gpio_desc *done;
	struct gpio_desc *alarm;
	u32 anchor_rise;	/* Rise at the alarm edge; 0 = uncalibrated */
	u32 threshold_uv;
	u32 x_anchor;		/* Q20 t/tau at the alarm voltage; 0 = unsolved */
	unsigned long next_sample;
	int uv_win[WM8505_BATTERY_AVG_WINDOW];
	unsigned int uv_count;
	int status;
	int capacity;		/* Reported, -1 = unknown */
	int sample_cap;		/* Last measured, -1 = none */
	int last_rise;		/* Previous rise time, <= 0 = none */
	int voltage_uv;		/* Last measured, -1 = none */
	int ocv_uv;		/* Windowed average, -1 = none */
	int done_level;		/* Charge-done level meaning full */
	int done_prev;		/* Done level last poll on AC; -1 = none */
	bool armed;		/* Alarm seen deasserted on battery */
	bool alarm_on;
};

/*
 * No ADC: the sense GPIO discharges an RC network on the battery rail, and
 * the recovery time is the voltage measurement.
 */
static int wm8505_battery_rise_us(struct wm8505_battery *bat)
{
	ktime_t start, timeout;
	int ret;

	ret = gpiod_direction_output(bat->sense, 0);
	if (ret)
		return ret;
	msleep(WM8505_BATTERY_DISCHARGE_MS);

	start = ktime_get();
	ret = gpiod_direction_input(bat->sense);
	if (ret)
		return ret;
	timeout = ktime_add_us(start, WM8505_BATTERY_RISE_TIMEOUT_US);

	while (!gpiod_get_value_cansleep(bat->sense)) {
		if (ktime_after(ktime_get(), timeout))
			return -ETIMEDOUT;
		usleep_range(200, 500);
	}

	return ktime_us_delta(ktime_get(), start);
}

/*
 * Evaluate V = Vth / (1 - e^(-x)), x = t/tau in Q20. Pade(3,3) for e^(-x)
 * turns it into V = Vth * (120 + 60x + 12x^2 + x^3) / (120x + 2x^3).
 */
static int wm8505_battery_curve_uv(u32 threshold_uv, u64 x)
{
	u64 x2, x3, num, denom, uv;

	x = min_t(u64, x, 20ULL << 20);
	x2 = (x * x) >> 20;
	x3 = (x2 * x) >> 20;

	num = (120ULL << 20) + 60 * x + 12 * x2 + x3;
	denom = max_t(u64, 120 * x + 2 * x3, 1);

	uv = div64_u64((u64)threshold_uv * num, denom);

	return min_t(u64, uv, INT_MAX);
}

/* The anchor rise was captured at a known voltage; any rise converts by ratio to it */
static int wm8505_battery_rise_to_uv(struct wm8505_battery *bat, u32 t_us)
{
	u64 x;

	if (!bat->anchor_rise || !bat->x_anchor)
		return -1;

	x = div64_u64((u64)t_us * bat->x_anchor, bat->anchor_rise);

	return wm8505_battery_curve_uv(bat->threshold_uv, x);
}

/* Bisect the rise curve for the alarm voltage from the constants alone */
static u32 wm8505_battery_solve_anchor(u32 threshold_uv, u32 alarm_uv)
{
	u64 lo = 1, hi = 20ULL << 20;

	while (hi - lo > 1) {
		u64 mid = lo + (hi - lo) / 2;

		if (wm8505_battery_curve_uv(threshold_uv, mid) > alarm_uv)
			lo = mid;
		else
			hi = mid;
	}

	return hi;
}

/* Bank the rise time at the alarm edge to fix the unit's entire scale */
static void wm8505_battery_calibrate(struct wm8505_battery *bat, u32 t_us)
{
	if (bat->anchor_rise && abs_diff(t_us, bat->anchor_rise) > bat->anchor_rise / 8)
		dev_warn(&bat->psy->dev, "alarm rise moved: %u -> %u us\n",
			 bat->anchor_rise, t_us);
	bat->anchor_rise = t_us;
	bat->uv_count = 0;
}

/* Take a median-filtered rise sample */
static int wm8505_battery_sample_us(struct wm8505_battery *bat)
{
	int t[WM8505_BATTERY_SAMPLES];
	int i, j, n = 0, ret;

	for (i = 0; i < WM8505_BATTERY_SAMPLES; i++) {
		ret = wm8505_battery_rise_us(bat);
		if (ret == -ETIMEDOUT)
			continue;
		if (ret < 0)
			return ret;
		for (j = n; j > 0 && t[j - 1] > ret; j--)
			t[j] = t[j - 1];
		t[j] = ret;
		n++;
	}
	if (!n)
		return -ENODEV;

	return t[n / 2];
}

/* Poll charge status and sample the battery */
static void wm8505_battery_work(struct work_struct *work)
{
	struct wm8505_battery *bat = container_of(work, struct wm8505_battery,
						  work.work);
	int status, capacity, uv, sample = -1, t = -1;
	bool supplied, alarm, raw_done, driven, changed, measured = false;
	u32 old_rise;

	if (!READ_ONCE(bat->psy))
		return;

	supplied = power_supply_am_i_supplied(bat->psy) > 0;
	raw_done = gpiod_get_value_cansleep(bat->done);
	alarm = gpiod_get_value_cansleep(bat->alarm);

	/* Sample on entering discharge, periodically, and at the alarm edge */
	if (!supplied && (bat->status != POWER_SUPPLY_STATUS_DISCHARGING ||
			  time_after(jiffies, bat->next_sample) ||
			  (alarm && !bat->alarm_on))) {
		bat->next_sample = jiffies + WM8505_BATTERY_SAMPLE_INTERVAL;
		t = wm8505_battery_sample_us(bat);
		measured = true;
	}

	/* A charger arriving mid-sample makes the rise meaningless */
	if (t >= 0 && power_supply_am_i_supplied(bat->psy) > 0) {
		measured = false;
		t = -1;
	}

	mutex_lock(&bat->lock);

	old_rise = bat->anchor_rise;

	/*
	 * Only the charger IC can pull the pin to 0; a 1 also matches
	 * the undriven pull-up, so a single reading proves nothing.
	 */
	driven = supplied && (!raw_done || raw_done == bat->done_prev);
	bat->done_prev = supplied ? raw_done : -1;

	if (!supplied)
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (driven && raw_done == bat->done_level)
		status = POWER_SUPPLY_STATUS_FULL;
	else
		status = POWER_SUPPLY_STATUS_CHARGING;

	if (status != POWER_SUPPLY_STATUS_DISCHARGING) {
		/* Charger holds the rail; battery voltage is unmeasurable */
		bat->voltage_uv = -1;
		bat->ocv_uv = -1;
	}

	/*
	 * Calibrate only on a witnessed crossing of a settled rail:
	 * a collapsing pack lands the anchor below the trip voltage.
	 */
	if (alarm && !supplied && bat->armed && t >= 0 && bat->last_rise > 0 &&
	    abs(t - bat->last_rise) * WM8505_BATTERY_SETTLED_FRAC < bat->last_rise)
		wm8505_battery_calibrate(bat, t);
	bat->armed = !supplied && !alarm;

	if (measured) {
		/* New discharge, drop stale samples */
		if (bat->status != POWER_SUPPLY_STATUS_DISCHARGING)
			bat->uv_count = 0;
		bat->last_rise = t;
		uv = t >= 0 ? wm8505_battery_rise_to_uv(bat, t) : -1;
		if (uv >= 0) {
			int i, n;
			s64 avg = 0;

			bat->uv_win[bat->uv_count % WM8505_BATTERY_AVG_WINDOW] = uv;
			bat->uv_count++;
			n = min(bat->uv_count, WM8505_BATTERY_AVG_WINDOW);
			for (i = 0; i < n; i++)
				avg += bat->uv_win[i];
			avg = div_s64(avg, n);

			sample = power_supply_batinfo_ocv2cap(bat->info, avg,
							      WM8505_BATTERY_TEMP_C);
			bat->voltage_uv = uv;
			bat->ocv_uv = avg;
		} else {
			bat->uv_count = 0;
			bat->voltage_uv = -1;
			bat->ocv_uv = -1;
		}
		bat->sample_cap = sample;
	}

	capacity = (status == POWER_SUPPLY_STATUS_FULL) ? 100 : bat->sample_cap;

	changed = status != bat->status || capacity != bat->capacity ||
		  alarm != bat->alarm_on || bat->anchor_rise != old_rise;
	bat->status = status;
	bat->capacity = capacity;
	bat->alarm_on = alarm;

	mutex_unlock(&bat->lock);

	if (changed)
		power_supply_changed(bat->psy);

	queue_delayed_work(system_wq, &bat->work, WM8505_BATTERY_POLL);
}

static enum power_supply_property wm8505_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_CALIBRATE,
};

static int wm8505_battery_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct wm8505_battery *bat = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&bat->lock);

	if (!bat->psy) {
		mutex_unlock(&bat->lock);
		return -EAGAIN;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = bat->status;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = bat->info->technology;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		if (bat->voltage_uv < 0)
			ret = -ENODATA;
		else
			val->intval = bat->voltage_uv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		if (bat->ocv_uv < 0)
			ret = -ENODATA;
		else
			val->intval = bat->ocv_uv;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		if (bat->capacity < 0)
			ret = -ENODATA;
		else
			val->intval = bat->capacity;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		if (bat->alarm_on)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		else if (bat->status == POWER_SUPPLY_STATUS_FULL)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else if (bat->capacity < 0)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
		else
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		break;
	case POWER_SUPPLY_PROP_CALIBRATE:
		val->intval = bat->anchor_rise;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&bat->lock);

	return ret;
}

/* Restore or clear the banked alarm rise */
static int wm8505_battery_set_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       const union power_supply_propval *val)
{
	struct wm8505_battery *bat = power_supply_get_drvdata(psy);

	if (psp != POWER_SUPPLY_PROP_CALIBRATE)
		return -EINVAL;
	if (val->intval < 0 || val->intval > WM8505_BATTERY_RISE_TIMEOUT_US)
		return -EINVAL;

	mutex_lock(&bat->lock);
	bat->anchor_rise = val->intval;
	bat->uv_count = 0;
	mutex_unlock(&bat->lock);

	bat->next_sample = jiffies - 1;
	mod_delayed_work(system_wq, &bat->work, 0);
	power_supply_changed(psy);

	return 0;
}

static int wm8505_battery_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CALIBRATE;
}

static void wm8505_battery_external_power_changed(struct power_supply *psy)
{
	struct wm8505_battery *bat = power_supply_get_drvdata(psy);

	mod_delayed_work(system_wq, &bat->work, 0);
}

static void wm8505_battery_work_disable(void *work)
{
	disable_delayed_work_sync(work);
}

static const struct power_supply_desc wm8505_battery_desc = {
	.name			= "wm8505-battery",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.get_property		= wm8505_battery_get_property,
	.set_property		= wm8505_battery_set_property,
	.property_is_writeable	= wm8505_battery_property_is_writeable,
	.external_power_changed	= wm8505_battery_external_power_changed,
	.properties		= wm8505_battery_props,
	.num_properties		= ARRAY_SIZE(wm8505_battery_props),
};

static int wm8505_battery_probe(struct platform_device *pdev)
{
	struct power_supply_config psy_cfg = {};
	struct device *dev = &pdev->dev;
	struct wm8505_battery *bat;
	struct power_supply *psy;
	struct gpio_desc *strap;
	int table_len, ret;
	u32 alarm_uv[2];

	bat = devm_kzalloc(dev, sizeof(*bat), GFP_KERNEL);
	if (!bat)
		return -ENOMEM;

	bat->status = POWER_SUPPLY_STATUS_UNKNOWN;
	bat->capacity = -1;
	bat->sample_cap = -1;
	bat->voltage_uv = -1;
	bat->ocv_uv = -1;
	bat->done_prev = -1;
	mutex_init(&bat->lock);
	INIT_DELAYED_WORK(&bat->work, wm8505_battery_work);

	bat->sense = devm_gpiod_get(dev, "sense", GPIOD_IN);
	if (IS_ERR(bat->sense))
		return dev_err_probe(dev, PTR_ERR(bat->sense),
				     "failed to request sense GPIO\n");

	bat->done = devm_gpiod_get(dev, "charge-done", GPIOD_IN);
	if (IS_ERR(bat->done))
		return dev_err_probe(dev, PTR_ERR(bat->done),
				     "failed to request charge-done GPIO\n");

	bat->alarm = devm_gpiod_get(dev, "low-battery", GPIOD_IN);
	if (IS_ERR(bat->alarm))
		return dev_err_probe(dev, PTR_ERR(bat->alarm),
				     "failed to request low-battery GPIO\n");

	strap = devm_gpiod_get(dev, "revision", GPIOD_ASIS);
	if (IS_ERR(strap))
		return dev_err_probe(dev, PTR_ERR(strap),
				     "failed to request revision GPIO\n");

	/* Revision strap: tied high when charge-done completes at 0, low when at 1 */
	bat->done_level = !gpiod_get_value_cansleep(strap);

	ret = device_property_read_u32(dev, "wm,threshold-microvolt",
				       &bat->threshold_uv);
	if (ret || !bat->threshold_uv)
		return dev_err_probe(dev, ret ?: -EINVAL,
				     "missing wm,threshold-microvolt property\n");

	ret = device_property_read_u32_array(dev, "wm,alarm-microvolt", alarm_uv,
					     ARRAY_SIZE(alarm_uv));
	if (ret || min(alarm_uv[0], alarm_uv[1]) <= bat->threshold_uv)
		return dev_err_probe(dev, ret ?: -EINVAL,
				     "missing or invalid wm,alarm-microvolt property\n");

	bat->x_anchor = wm8505_battery_solve_anchor(bat->threshold_uv,
						    alarm_uv[bat->done_level]);

	psy_cfg.drv_data = bat;
	psy_cfg.fwnode = dev_fwnode(dev);
	psy = devm_power_supply_register(dev, &wm8505_battery_desc, &psy_cfg);
	if (IS_ERR(psy))
		return dev_err_probe(dev, PTR_ERR(psy),
				     "failed to register power supply\n");

	ret = devm_add_action_or_reset(dev, wm8505_battery_work_disable,
				       &bat->work);
	if (ret)
		return ret;

	ret = power_supply_get_battery_info(psy, &bat->info);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get battery info\n");

	if (!power_supply_find_ocv2cap_table(bat->info, WM8505_BATTERY_TEMP_C,
					     &table_len))
		return dev_err_probe(dev, -ENODEV, "failed to find OCV capacity table\n");

	mutex_lock(&bat->lock);
	WRITE_ONCE(bat->psy, psy);
	mutex_unlock(&bat->lock);

	platform_set_drvdata(pdev, bat);
	queue_delayed_work(system_wq, &bat->work, 0);

	return 0;
}

static int wm8505_battery_suspend(struct device *dev)
{
	struct wm8505_battery *bat = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&bat->work);

	return 0;
}

static int wm8505_battery_resume(struct device *dev)
{
	struct wm8505_battery *bat = dev_get_drvdata(dev);

	bat->armed = false;
	bat->done_prev = -1;
	bat->status = POWER_SUPPLY_STATUS_UNKNOWN;
	mod_delayed_work(system_wq, &bat->work, 0);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(wm8505_battery_pm_ops, wm8505_battery_suspend,
				wm8505_battery_resume);

static const struct of_device_id wm8505_battery_of_match[] = {
	{ .compatible = "wm,wm8505-battery", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wm8505_battery_of_match);

static struct platform_driver wm8505_battery_driver = {
	.probe = wm8505_battery_probe,
	.driver	= {
		.name = "wm8505-battery",
		.of_match_table = wm8505_battery_of_match,
		.pm = pm_sleep_ptr(&wm8505_battery_pm_ops),
	},
};
module_platform_driver(wm8505_battery_driver);

MODULE_DESCRIPTION("WonderMedia WM8505 Battery Driver");
MODULE_AUTHOR("Logan Russell <me@lrussell.net>");
MODULE_LICENSE("GPL");
