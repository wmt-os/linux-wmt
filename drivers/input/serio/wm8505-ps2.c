// SPDX-License-Identifier: GPL-2.0-only
/*
 * WonderMedia WM8505 PS/2 Driver
 *
 * Copyright (C) 2026 Logan Russell <me@lrussell.net>
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i8042.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/serio.h>
#include <linux/slab.h>

/* Hardware Register Offsets */
#define WM8505_KBDC_DATA	0x0
#define WM8505_KBDC_CMD_STAT	0x4

/* 8042 Expected Responses */
#define KBDC_RET_SELF_TEST	0x55

struct wm8505_ps2 {
	struct device	*dev;
	void __iomem	*base;
	struct clk	*clk;
	struct serio	*kbd_port;
	struct serio	*aux_port;
	spinlock_t	lock;		/* Protects hardware registers */
	struct mutex	cmd_mutex;	/* Serializes commands across both ports */
	u8		ctr;		/* Cached control register */
};

static int wm8505_ps2_wait_ibf(struct wm8505_ps2 *ps2)
{
	u32 val;

	return readl_poll_timeout_atomic(ps2->base + WM8505_KBDC_CMD_STAT, val,
					 !(val & I8042_STR_IBF), 50, 100000);
}

static int wm8505_ps2_wait_obf(struct wm8505_ps2 *ps2)
{
	u32 val;

	return readl_poll_timeout_atomic(ps2->base + WM8505_KBDC_CMD_STAT, val,
					 (val & I8042_STR_OBF), 50, 100000);
}

static void wm8505_ps2_flush(struct wm8505_ps2 *ps2)
{
	int max_read = 16;

	while ((readl(ps2->base + WM8505_KBDC_CMD_STAT) & I8042_STR_OBF) && max_read--) {
		usleep_range(50, 100);
		readl(ps2->base + WM8505_KBDC_DATA);
	}
}

static int wm8505_ps2_write_ctr(struct wm8505_ps2 *ps2, u8 ctr)
{
	if (wm8505_ps2_wait_ibf(ps2))
		return -ETIMEDOUT;
	writeb(I8042_CMD_CTL_WCTR & 0xff, ps2->base + WM8505_KBDC_CMD_STAT);

	if (wm8505_ps2_wait_ibf(ps2))
		return -ETIMEDOUT;
	writeb(ctr, ps2->base + WM8505_KBDC_DATA);

	ps2->ctr = ctr;
	return 0;
}

static int wm8505_ps2_hw_init(struct wm8505_ps2 *ps2)
{
	u8 val;

	wm8505_ps2_flush(ps2);

	/* Controller self-test */
	if (wm8505_ps2_wait_ibf(ps2))
		return -ETIMEDOUT;
	writeb(I8042_CMD_CTL_TEST & 0xff, ps2->base + WM8505_KBDC_CMD_STAT);

	if (wm8505_ps2_wait_obf(ps2))
		return -ETIMEDOUT;

	val = readl(ps2->base + WM8505_KBDC_DATA) & 0xff;
	if (val != KBDC_RET_SELF_TEST)
		dev_warn(ps2->dev, "controller self-test failed (%#x)\n", val);

	/* Read current control register state from hardware */
	if (wm8505_ps2_wait_ibf(ps2))
		return -ETIMEDOUT;
	writeb(I8042_CMD_CTL_RCTR & 0xff, ps2->base + WM8505_KBDC_CMD_STAT);

	if (wm8505_ps2_wait_obf(ps2))
		return -ETIMEDOUT;
	ps2->ctr = readl(ps2->base + WM8505_KBDC_DATA) & 0xff;

	/* Keep the keyboard in scan set 2, ports disabled and interrupts masked */
	ps2->ctr |= I8042_CTR_KBDDIS | I8042_CTR_AUXDIS;
	ps2->ctr &= ~(I8042_CTR_KBDINT | I8042_CTR_AUXINT | I8042_CTR_XLATE);

	return wm8505_ps2_write_ctr(ps2, ps2->ctr);
}

static irqreturn_t wm8505_ps2_interrupt(int irq, void *dev_id)
{
	struct wm8505_ps2 *ps2 = dev_id;
	unsigned long flags;
	unsigned int dfl;
	u32 status;
	u8 data;

	spin_lock_irqsave(&ps2->lock, flags);
	status = readl(ps2->base + WM8505_KBDC_CMD_STAT);

	if (!(status & I8042_STR_OBF)) {
		spin_unlock_irqrestore(&ps2->lock, flags);
		return IRQ_NONE;
	}

	data = readl(ps2->base + WM8505_KBDC_DATA) & 0xff;
	spin_unlock_irqrestore(&ps2->lock, flags);

	/* The auxiliary port flags its own recovered retries, not corrupt data */
	if (status & I8042_STR_AUXDATA) {
		serio_interrupt(ps2->aux_port, data, 0);
		return IRQ_HANDLED;
	}

	dfl = ((status & I8042_STR_PARITY) ? SERIO_PARITY : 0) |
	      ((status & I8042_STR_TIMEOUT) ? SERIO_TIMEOUT : 0);
	serio_interrupt(ps2->kbd_port, data, dfl);

	return IRQ_HANDLED;
}

static int wm8505_ps2_open(struct serio *port)
{
	struct wm8505_ps2 *ps2 = port->port_data;
	unsigned long flags;
	u8 ctr;
	int ret;

	spin_lock_irqsave(&ps2->lock, flags);

	/* Enable port and hardware interrupts in the CTR */
	ctr = ps2->ctr;
	if (port == ps2->kbd_port) {
		ctr &= ~I8042_CTR_KBDDIS;
		ctr |= I8042_CTR_KBDINT;
	} else {
		ctr &= ~I8042_CTR_AUXDIS;
		ctr |= I8042_CTR_AUXINT;
	}

	ret = wm8505_ps2_write_ctr(ps2, ctr);
	spin_unlock_irqrestore(&ps2->lock, flags);

	if (ret == 0)
		wm8505_ps2_interrupt(0, ps2);

	return ret;
}

static void wm8505_ps2_close(struct serio *port)
{
	struct wm8505_ps2 *ps2 = port->port_data;
	unsigned long flags;
	u8 ctr;

	spin_lock_irqsave(&ps2->lock, flags);

	/* Disable port and hardware interrupts */
	ctr = ps2->ctr;
	if (port == ps2->kbd_port) {
		ctr |= I8042_CTR_KBDDIS;
		ctr &= ~I8042_CTR_KBDINT;
	} else {
		ctr |= I8042_CTR_AUXDIS;
		ctr &= ~I8042_CTR_AUXINT;
	}

	if (wm8505_ps2_write_ctr(ps2, ctr))
		dev_warn(ps2->dev, "failed to disable %s\n", port->name);

	spin_unlock_irqrestore(&ps2->lock, flags);

	wm8505_ps2_interrupt(0, ps2);
}

static int wm8505_ps2_kbd_write(struct serio *port, unsigned char c)
{
	struct wm8505_ps2 *ps2 = port->port_data;
	unsigned long flags;
	int ret = -ETIMEDOUT;

	spin_lock_irqsave(&ps2->lock, flags);
	if (wm8505_ps2_wait_ibf(ps2) == 0) {
		writeb(c, ps2->base + WM8505_KBDC_DATA);
		ret = 0;
	}
	spin_unlock_irqrestore(&ps2->lock, flags);

	return ret;
}

static int wm8505_ps2_aux_write(struct serio *port, unsigned char c)
{
	struct wm8505_ps2 *ps2 = port->port_data;
	unsigned long flags;
	int ret = -ETIMEDOUT;

	spin_lock_irqsave(&ps2->lock, flags);
	if (wm8505_ps2_wait_ibf(ps2) == 0) {
		writeb(I8042_CMD_AUX_SEND & 0xff, ps2->base + WM8505_KBDC_CMD_STAT);
		if (wm8505_ps2_wait_ibf(ps2) == 0) {
			writeb(c, ps2->base + WM8505_KBDC_DATA);
			ret = 0;
		}
	}
	spin_unlock_irqrestore(&ps2->lock, flags);

	if (ret == 0)
		wm8505_ps2_interrupt(0, ps2);

	return ret;
}

static struct serio *wm8505_ps2_allocate_port(struct wm8505_ps2 *ps2,
					      const char *name,
					      const char *phys,
					      int type,
					      int (*write_fn)(struct serio *, unsigned char))
{
	struct serio *port;

	port = kzalloc(sizeof(*port), GFP_KERNEL);
	if (!port)
		return NULL;

	port->id.type = type;
	port->write = write_fn;
	port->open = wm8505_ps2_open;
	port->close = wm8505_ps2_close;
	port->port_data = ps2;
	port->ps2_cmd_mutex = &ps2->cmd_mutex;
	port->dev.parent = ps2->dev;
	strscpy(port->name, name, sizeof(port->name));
	strscpy(port->phys, phys, sizeof(port->phys));

	return port;
}

static int wm8505_ps2_probe(struct platform_device *pdev)
{
	struct wm8505_ps2 *ps2;
	int kbd_irq, aux_irq;
	int error;

	ps2 = devm_kzalloc(&pdev->dev, sizeof(*ps2), GFP_KERNEL);
	if (!ps2)
		return -ENOMEM;

	spin_lock_init(&ps2->lock);
	mutex_init(&ps2->cmd_mutex);
	ps2->dev = &pdev->dev;

	ps2->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ps2->base))
		return PTR_ERR(ps2->base);

	kbd_irq = platform_get_irq_byname(pdev, "kbd");
	if (kbd_irq < 0)
		return kbd_irq;

	aux_irq = platform_get_irq_byname(pdev, "aux");
	if (aux_irq < 0)
		return aux_irq;

	ps2->clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(ps2->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(ps2->clk), "failed to get clock\n");

	/* Reset the 8042 state machine and CTR */
	error = wm8505_ps2_hw_init(ps2);
	if (error)
		return dev_err_probe(&pdev->dev, error, "hardware initialization failed\n");

	/* KBD port uses SERIO_8042 so atkbd receives raw scan set 2 scancodes */
	ps2->kbd_port = wm8505_ps2_allocate_port(ps2, "WM8505 KBD port", "wm8505/serio0",
						 SERIO_8042, wm8505_ps2_kbd_write);
	/* AUX port uses SERIO_PS_PSTHRU to bind only psmouse, preventing atkbd probes */
	ps2->aux_port = wm8505_ps2_allocate_port(ps2, "WM8505 AUX port", "wm8505/serio1",
						 SERIO_PS_PSTHRU, wm8505_ps2_aux_write);

	if (!ps2->kbd_port || !ps2->aux_port) {
		kfree(ps2->kbd_port);
		kfree(ps2->aux_port);
		return -ENOMEM;
	}

	error = devm_request_irq(&pdev->dev, kbd_irq, wm8505_ps2_interrupt,
				 IRQF_SHARED, "wm8505-kbd", ps2);
	if (error) {
		kfree(ps2->kbd_port);
		kfree(ps2->aux_port);
		return dev_err_probe(&pdev->dev, error, "failed to request KBD IRQ\n");
	}

	error = devm_request_irq(&pdev->dev, aux_irq, wm8505_ps2_interrupt,
				 IRQF_SHARED, "wm8505-aux", ps2);
	if (error) {
		kfree(ps2->kbd_port);
		kfree(ps2->aux_port);
		return dev_err_probe(&pdev->dev, error, "failed to request AUX IRQ\n");
	}

	serio_register_port(ps2->kbd_port);
	serio_register_port(ps2->aux_port);

	platform_set_drvdata(pdev, ps2);
	return 0;
}

static void wm8505_ps2_remove(struct platform_device *pdev)
{
	struct wm8505_ps2 *ps2 = platform_get_drvdata(pdev);

	serio_unregister_port(ps2->kbd_port);
	serio_unregister_port(ps2->aux_port);
}

static const struct of_device_id wm8505_ps2_of_match[] = {
	{ .compatible = "wm,wm8505-ps2", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wm8505_ps2_of_match);

static struct platform_driver wm8505_ps2_driver = {
	.probe = wm8505_ps2_probe,
	.remove = wm8505_ps2_remove,
	.driver = {
		.name = "wm8505-ps2",
		.of_match_table = wm8505_ps2_of_match,
	},
};
module_platform_driver(wm8505_ps2_driver);

MODULE_DESCRIPTION("WonderMedia WM8505 PS/2 Driver");
MODULE_AUTHOR("Logan Russell <me@lrussell.net>");
MODULE_LICENSE("GPL");
