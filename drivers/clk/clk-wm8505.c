// SPDX-License-Identifier: GPL-2.0-only
/*
 * WonderMedia WM8505 Common Clock Framework Driver
 *
 * Copyright (C) 2026 Logan Russell <me@lrussell.net>
 *
 * Based on drivers/clk/clk-vt8500.c
 * Copyright (C) 2012 Tony Prisk <linux@prisktech.co.nz>
 */

#include <linux/bitops.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

#include <dt-bindings/clock/wm8505-clock.h>

/* PMC Clock-Enable Registers */
#define WM8505_PMCEL_REG		0x250
#define WM8505_PMCEU_REG		0x254

/* PMC Status / Busy Poll (0x0) */
#define WM8505_PMC_STATUS_REG		0x0
#define WM8505_PMC_DIV_BUSY		BIT(3)	/* Divisor update busy */
#define WM8505_PMC_PLL_BUSY		BIT(4)	/* PLL multiplier update busy */
#define WM8505_PMC_BUSY_MASK		(WM8505_PMC_DIV_BUSY | WM8505_PMC_PLL_BUSY)
#define WM8505_PMC_POLL_DELAY_US	1
#define WM8505_PMC_POLL_TIMEOUT_US	1000

/* PLL Multiplier / Range Registers */
#define WM8505_PLLA_REG			0x200
#define WM8505_PLLB_REG			0x204
#define WM8505_PLLC_REG			0x208
#define WM8505_PLLD_REG			0x20c

/* PLL Fields (0x200-0x20c) */
#define WM8505_PLL_MUL_MASK		GENMASK(4, 0)	/* N => 2*N */
#define WM8505_PLL_PREDIV_BYPASS	BIT(8)		/* 0 => predivide /2, 1 => /1 */
#define WM8505_PLL_FIELD_MASK		(WM8505_PLL_PREDIV_BYPASS | WM8505_PLL_MUL_MASK)
#define WM8505_PLL_MIN_MUL		4		/* Min multiplier */
#define WM8505_PLL_MAX_MUL		62		/* Max multiplier */
#define WM8505_PLL_PREDIV2_MAX_MUL	31		/* Max multiplier with /2 prediv */

/* Clock Divisor Registers */
#define WM8505_ARM_DIV_REG		0x300
#define WM8505_AHB_DIV_REG		0x304
#define WM8505_APB_DIV_REG		0x350
#define WM8505_DDR_DIV_REG		0x310
#define WM8505_SFC_DIV_REG		0x314
#define WM8505_KBDC_PRE_REG		0x318
#define WM8505_KBDC_DIV_REG		0x31c
#define WM8505_SDHC_DIV_REG		0x328
#define WM8505_MAC0_DIV_REG		0x32c
#define WM8505_NAND_DIV_REG		0x330
#define WM8505_NORGUP_DIV_REG		0x334
#define WM8505_SPI0_DIV_REG		0x33c
#define WM8505_SPI1_DIV_REG		0x340
#define WM8505_SPI2_DIV_REG		0x344
#define WM8505_PWM_DIV_REG		0x348
#define WM8505_NA0_DIV_REG		0x358
#define WM8505_NA12_DIV_REG		0x35c
#define WM8505_I2C0_DIV_REG		0x36c
#define WM8505_I2C1_DIV_REG		0x370
#define WM8505_DVO_DIV_REG		0x374

/* Clock Divisor Masks (value 0 => divide-by-(mask+1)) */
#define WM8505_DIV_MASK			GENMASK(4, 0)	/* 5-bit, 0 => /32 */
#define WM8505_AHB_DIV_MASK		GENMASK(2, 0)	/* 3-bit, 0 => /8 */
#define WM8505_SDMMC_DIV_MASK		GENMASK(5, 0)	/* 5-bit divisor; bit5 enables /64 */

/* SD/MMC clock: /64 prescaler via bit5 (0x328) */
#define WM8505_SDMMC_DIV_FIELD		GENMASK(4, 0)	/* Base divisor field */
#define WM8505_SDMMC_DIV_ZERO		32		/* Field 0 => /32 */
#define WM8505_SDMMC_PREDIV_EN		BIT(5)		/* Enable /64 prescaler */
#define WM8505_SDMMC_PREDIV		64

static DEFINE_SPINLOCK(wm8505_clk_lock);
static void __iomem *pmc_base;

/* Hardware Clock Structures */
struct wm8505_clk {
	struct clk_hw	hw;
	u32		div_reg_offset;
	unsigned int	div_mask;
	u32		en_reg_offset;
	int		en_bit;
};

struct wm8505_pll_clk {
	struct clk_hw	hw;
	u32		reg_offset;
};

#define to_wm8505_clk(_hw) container_of(_hw, struct wm8505_clk, hw)
#define to_wm8505_pll_clk(_hw) container_of(_hw, struct wm8505_pll_clk, hw)

/* Poll PMC busy status to prevent bus hangs */
static void wm8505_pmc_wait_busy(void)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout_atomic(pmc_base + WM8505_PMC_STATUS_REG, val,
					!(val & WM8505_PMC_BUSY_MASK),
					WM8505_PMC_POLL_DELAY_US,
					WM8505_PMC_POLL_TIMEOUT_US);
	WARN_ON(ret);
}

static int wm8505_gated_div_enable(struct clk_hw *hw)
{
	struct wm8505_clk *cdev = to_wm8505_clk(hw);
	unsigned long flags;

	spin_lock_irqsave(&wm8505_clk_lock, flags);
	writel(readl(pmc_base + cdev->en_reg_offset) | BIT(cdev->en_bit),
	       pmc_base + cdev->en_reg_offset);
	spin_unlock_irqrestore(&wm8505_clk_lock, flags);

	return 0;
}

static void wm8505_gated_div_disable(struct clk_hw *hw)
{
	struct wm8505_clk *cdev = to_wm8505_clk(hw);
	unsigned long flags;

	spin_lock_irqsave(&wm8505_clk_lock, flags);
	writel(readl(pmc_base + cdev->en_reg_offset) & ~BIT(cdev->en_bit),
	       pmc_base + cdev->en_reg_offset);
	spin_unlock_irqrestore(&wm8505_clk_lock, flags);
}

static int wm8505_gated_div_is_enabled(struct clk_hw *hw)
{
	struct wm8505_clk *cdev = to_wm8505_clk(hw);

	return readl(pmc_base + cdev->en_reg_offset) & BIT(cdev->en_bit);
}

static unsigned long wm8505_clk_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct wm8505_clk *cdev = to_wm8505_clk(hw);
	u32 div = readl(pmc_base + cdev->div_reg_offset) & cdev->div_mask;
	u32 val;

	/* The SD/MMC clock has a /64 prescaler stage */
	if (cdev->div_mask == WM8505_SDMMC_DIV_MASK) {
		val = div & WM8505_SDMMC_DIV_FIELD;
		if (!val)
			val = WM8505_SDMMC_DIV_ZERO;
		return parent_rate / (val * ((div & WM8505_SDMMC_PREDIV_EN) ?
					      WM8505_SDMMC_PREDIV : 1));
	}

	return parent_rate / (div ? div : cdev->div_mask + 1);
}

static long wm8505_clk_round_rate(struct clk_hw *hw, unsigned long rate, unsigned long *prate)
{
	struct wm8505_clk *cdev = to_wm8505_clk(hw);
	u32 divisor;

	if (rate == 0)
		return 0;

	divisor = DIV_ROUND_CLOSEST(*prate, rate);

	if (cdev->div_mask == WM8505_SDMMC_DIV_MASK) {
		if (divisor > WM8505_SDMMC_DIV_ZERO)
			divisor = clamp_val(DIV_ROUND_CLOSEST(divisor, WM8505_SDMMC_PREDIV),
					    1U, WM8505_SDMMC_DIV_ZERO) * WM8505_SDMMC_PREDIV;
		else
			divisor = clamp_val(divisor, 1U, WM8505_SDMMC_DIV_ZERO);
	} else {
		divisor = clamp_val(divisor, 1U, cdev->div_mask + 1);
	}

	return *prate / divisor;
}

static int wm8505_clk_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct wm8505_clk *cdev = to_wm8505_clk(hw);
	u32 divisor, val;
	unsigned long flags;

	if (rate == 0)
		return 0;

	divisor = DIV_ROUND_CLOSEST(parent_rate, rate);

	if (cdev->div_mask == WM8505_SDMMC_DIV_MASK) {
		if (divisor > WM8505_SDMMC_DIV_ZERO) {
			val = DIV_ROUND_CLOSEST(divisor, WM8505_SDMMC_PREDIV);
			val = WM8505_SDMMC_PREDIV_EN | (val == WM8505_SDMMC_DIV_ZERO ? 0 : val);
		} else {
			val = (divisor == WM8505_SDMMC_DIV_ZERO) ? 0 : divisor;
		}
	} else {
		val = (divisor == cdev->div_mask + 1) ? 0 : divisor;
	}

	spin_lock_irqsave(&wm8505_clk_lock, flags);
	wm8505_pmc_wait_busy();
	writel((readl(pmc_base + cdev->div_reg_offset) & ~cdev->div_mask) | val,
	       pmc_base + cdev->div_reg_offset);
	wm8505_pmc_wait_busy();
	spin_unlock_irqrestore(&wm8505_clk_lock, flags);

	return 0;
}

static const struct clk_ops wm8505_div_ops = {
	.recalc_rate	= wm8505_clk_recalc_rate,
	.round_rate	= wm8505_clk_round_rate,
	.set_rate	= wm8505_clk_set_rate,
};

static const struct clk_ops wm8505_gated_div_ops = {
	.enable		= wm8505_gated_div_enable,
	.disable	= wm8505_gated_div_disable,
	.is_enabled	= wm8505_gated_div_is_enabled,
	.recalc_rate	= wm8505_clk_recalc_rate,
	.round_rate	= wm8505_clk_round_rate,
	.set_rate	= wm8505_clk_set_rate,
};

/* PLL Configuration Macros */
#define WM8505_PLL_MUL(x)		(((x) & WM8505_PLL_MUL_MASK) << 1)
#define WM8505_PLL_DIV(x)		(((x) & WM8505_PLL_PREDIV_BYPASS) ? 1 : 2)
#define WM8505_BITS_TO_FREQ(r, m, d)	(((r) / (d)) * (m))
#define WM8505_BITS_TO_VAL(m, d)	(((d) == 2 ? 0 : WM8505_PLL_PREDIV_BYPASS) | \
					 (((m) >> 1) & WM8505_PLL_MUL_MASK))

static unsigned long wm8505_pll_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct wm8505_pll_clk *pll = to_wm8505_pll_clk(hw);
	u32 val = readl(pmc_base + pll->reg_offset);
	u32 mul = WM8505_PLL_MUL(val);
	u32 div = WM8505_PLL_DIV(val);

	return WM8505_BITS_TO_FREQ(parent_rate, mul, div);
}

/* Derive PLL multiplier and predivider for a target rate */
static void wm8505_find_pll_bits(unsigned long rate, unsigned long parent_rate,
				 u32 *multiplier, u32 *prediv)
{
	rate = clamp(rate, parent_rate * WM8505_PLL_MIN_MUL,
		     parent_rate * WM8505_PLL_MAX_MUL);

	if (rate <= parent_rate * WM8505_PLL_PREDIV2_MAX_MUL) {
		*prediv = 2;
		*multiplier = DIV_ROUND_CLOSEST(rate, parent_rate) * 2;
	} else {
		*prediv = 1;
		*multiplier = DIV_ROUND_CLOSEST(rate, parent_rate * 2) * 2;
	}
}

static long wm8505_pll_round_rate(struct clk_hw *hw, unsigned long rate, unsigned long *prate)
{
	u32 mul, div;

	wm8505_find_pll_bits(rate, *prate, &mul, &div);
	return WM8505_BITS_TO_FREQ(*prate, mul, div);
}

static int wm8505_pll_set_rate(struct clk_hw *hw, unsigned long rate, unsigned long parent_rate)
{
	struct wm8505_pll_clk *pll = to_wm8505_pll_clk(hw);
	u32 mul, div, reg_val;
	unsigned long flags;

	wm8505_find_pll_bits(rate, parent_rate, &mul, &div);

	spin_lock_irqsave(&wm8505_clk_lock, flags);
	wm8505_pmc_wait_busy();
	reg_val = readl(pmc_base + pll->reg_offset);
	reg_val &= ~WM8505_PLL_FIELD_MASK;
	reg_val |= WM8505_BITS_TO_VAL(mul, div);
	writel(reg_val, pmc_base + pll->reg_offset);
	wm8505_pmc_wait_busy();
	spin_unlock_irqrestore(&wm8505_clk_lock, flags);

	return 0;
}

static const struct clk_ops wm8505_pll_ops = {
	.recalc_rate = wm8505_pll_recalc_rate,
	.round_rate  = wm8505_pll_round_rate,
	.set_rate    = wm8505_pll_set_rate,
};

/* Clock Descriptor Tables */
struct wm8505_gate_desc {
	int		id;
	const char	*name;
	const char	*parent_name;
	u32		reg_offset;
	u8		bit_idx;
	unsigned long	flags;
};

struct wm8505_clk_desc {
	int		id;
	const char	*name;
	const char	*parent_name;
	u32		en_reg_offset;
	int		en_bit;
	u32		div_reg_offset;
	unsigned int	div_mask;
	unsigned long	flags;
};

struct wm8505_pll_desc {
	int		id;
	const char	*name;
	const char	*parent_name;
	u32		reg_offset;
};

#define DEF_GATE(_id, _name, _parent, _reg, _bit, _flags) \
	{ .id = _id, .name = _name, .parent_name = _parent, \
	  .reg_offset = _reg, .bit_idx = _bit, .flags = _flags }

#define DEF_DIV(_id, _name, _parent, _div_reg, _div_mask) \
	{ .id = _id, .name = _name, .parent_name = _parent, \
	  .div_reg_offset = _div_reg, .div_mask = _div_mask }

#define DEF_GATED_DIV(_id, _name, _parent, _en_reg, _en_bit, _div_reg, _div_mask, _flags) \
	{ .id = _id, .name = _name, .parent_name = _parent, \
	  .en_reg_offset = _en_reg, .en_bit = _en_bit, \
	  .div_reg_offset = _div_reg, .div_mask = _div_mask, .flags = _flags }

static const struct wm8505_pll_desc wm8505_pll_clks[] __initconst = {
	{ WM8505_CLK_PLLA, "plla", "osc25m", WM8505_PLLA_REG },
	{ WM8505_CLK_PLLB, "pllb", "osc25m", WM8505_PLLB_REG },
	{ WM8505_CLK_PLLC, "pllc", "osc25m", WM8505_PLLC_REG },
	{ WM8505_CLK_PLLD, "plld", "osc25m", WM8505_PLLD_REG },
};

static const struct wm8505_gate_desc wm8505_gate_clks[] __initconst = {
	/* WM8505_PMCEL_REG (0x250) */
	DEF_GATE(WM8505_CLK_UART0, "uart0", "osc24m", WM8505_PMCEL_REG, 1, 0),
	DEF_GATE(WM8505_CLK_UART1, "uart1", "osc24m", WM8505_PMCEL_REG, 2, 0),
	DEF_GATE(WM8505_CLK_UART2, "uart2", "osc24m", WM8505_PMCEL_REG, 3, 0),
	DEF_GATE(WM8505_CLK_UART3, "uart3", "osc24m", WM8505_PMCEL_REG, 4, 0),
	DEF_GATE(WM8505_CLK_I2CSLAVE, "i2c_slave", "apb", WM8505_PMCEL_REG, 6, 0),
	DEF_GATE(WM8505_CLK_RTC, "rtc", "ahb", WM8505_PMCEL_REG, 7, CLK_IS_CRITICAL),
	DEF_GATE(WM8505_CLK_KEYPAD, "keypad", "ahb", WM8505_PMCEL_REG, 9, 0),
	DEF_GATE(WM8505_CLK_GPIO, "gpio", "ahb", WM8505_PMCEL_REG, 11, CLK_IS_CRITICAL),
	DEF_GATE(WM8505_CLK_I2S, "i2s", "apb", WM8505_PMCEL_REG, 16, 0),
	DEF_GATE(WM8505_CLK_CIR, "cir", "apb", WM8505_PMCEL_REG, 17, 0),
	DEF_GATE(WM8505_CLK_AC97, "ac97", "apb", WM8505_PMCEL_REG, 19, 0),
	DEF_GATE(WM8505_CLK_SCC, "scc", "ahb", WM8505_PMCEL_REG, 21, CLK_IS_CRITICAL),
	DEF_GATE(WM8505_CLK_UART4, "uart4", "osc24m", WM8505_PMCEL_REG, 22, 0),
	DEF_GATE(WM8505_CLK_UART5, "uart5", "osc24m", WM8505_PMCEL_REG, 23, CLK_IS_CRITICAL),
	DEF_GATE(WM8505_CLK_AMP, "amp", "apb", WM8505_PMCEL_REG, 24, CLK_IS_CRITICAL),
	DEF_GATE(WM8505_CLK_JENC, "jenc", "ahb", WM8505_PMCEL_REG, 27, 0),
	DEF_GATE(WM8505_CLK_GE, "ge", "ahb", WM8505_PMCEL_REG, 29, 0),
	DEF_GATE(WM8505_CLK_GOVRHD, "govrhd", "ahb", WM8505_PMCEL_REG, 30, 0),

	/* WM8505_PMCEU_REG (0x254) */
	DEF_GATE(WM8505_CLK_DMA, "dma", "ahb", WM8505_PMCEU_REG, 5, 0),
	DEF_GATE(WM8505_CLK_UHC, "uhc", "osc24m", WM8505_PMCEU_REG, 7, CLK_IS_CRITICAL),
	DEF_GATE(WM8505_CLK_UDC, "udc", "osc24m", WM8505_PMCEU_REG, 8, 0),
	DEF_GATE(WM8505_CLK_PDMA, "pdma", "ahb", WM8505_PMCEU_REG, 9, 0),
	DEF_GATE(WM8505_CLK_AHBBRIDGE, "ahb_bridge", "ahb", WM8505_PMCEU_REG, 13, CLK_IS_CRITICAL),
	DEF_GATE(WM8505_CLK_SDTV, "sdtv", "ahb", WM8505_PMCEU_REG, 14, CLK_IS_CRITICAL),
	DEF_GATE(WM8505_CLK_SYS, "sys", "ahb", WM8505_PMCEU_REG, 21, 0),
	DEF_GATE(WM8505_CLK_SAE, "sae", "ahb", WM8505_PMCEU_REG, 24, 0),
	DEF_GATE(WM8505_CLK_ETHPHY, "eth_phy", "osc25m", WM8505_PMCEU_REG, 26, CLK_IS_CRITICAL),
	DEF_GATE(WM8505_CLK_SCL444U, "scl444u", "ahb", WM8505_PMCEU_REG, 28, 0),
	DEF_GATE(WM8505_CLK_GOVW, "govw", "ahb", WM8505_PMCEU_REG, 29, 0),
	DEF_GATE(WM8505_CLK_VID, "vid", "ahb", WM8505_PMCEU_REG, 30, 0),
	DEF_GATE(WM8505_CLK_VPP, "vpp", "ahb", WM8505_PMCEU_REG, 31, 0),
};

static const struct wm8505_clk_desc wm8505_div_clks[] __initconst = {
	DEF_DIV(WM8505_CLK_ARM, "arm", "plla", WM8505_ARM_DIV_REG, WM8505_DIV_MASK),
	DEF_DIV(WM8505_CLK_AHB, "ahb", "arm", WM8505_AHB_DIV_REG, WM8505_AHB_DIV_MASK),
	DEF_DIV(WM8505_CLK_APB, "apb", "ahb", WM8505_APB_DIV_REG, WM8505_DIV_MASK),
	DEF_DIV(WM8505_CLK_PS2KBDC_PRE, "ps2_kbdc_pre", "pllb",
		WM8505_KBDC_PRE_REG, WM8505_DIV_MASK),
};

static const struct wm8505_clk_desc wm8505_gated_div_clks[] __initconst = {
	DEF_GATED_DIV(WM8505_CLK_DDR, "ddr", "plld", WM8505_PMCEU_REG, 0,
		      WM8505_DDR_DIV_REG, WM8505_DIV_MASK, CLK_IS_CRITICAL),
	DEF_GATED_DIV(WM8505_CLK_SFC, "sfc", "pllb", WM8505_PMCEU_REG, 23,
		      WM8505_SFC_DIV_REG, WM8505_DIV_MASK, CLK_IS_CRITICAL),
	DEF_GATED_DIV(WM8505_CLK_PS2KBDC, "ps2_kbdc", "ps2_kbdc_pre", WM8505_PMCEU_REG, 4,
		      WM8505_KBDC_DIV_REG, WM8505_DIV_MASK, 0),
	DEF_GATED_DIV(WM8505_CLK_SDHC, "sdhc", "pllb", WM8505_PMCEU_REG, 18,
		      WM8505_SDHC_DIV_REG, WM8505_SDMMC_DIV_MASK, 0),
	DEF_GATED_DIV(WM8505_CLK_MAC0, "mac0", "osc25m", WM8505_PMCEU_REG, 20,
		      WM8505_MAC0_DIV_REG, WM8505_DIV_MASK, CLK_IS_CRITICAL),
	DEF_GATED_DIV(WM8505_CLK_NAND, "nand", "pllb", WM8505_PMCEU_REG, 16,
		      WM8505_NAND_DIV_REG, WM8505_DIV_MASK, CLK_IS_CRITICAL),
	DEF_GATED_DIV(WM8505_CLK_NORGUP, "nor_gup", "pllb", WM8505_PMCEU_REG, 3,
		      WM8505_NORGUP_DIV_REG, WM8505_DIV_MASK, CLK_IS_CRITICAL),
	DEF_GATED_DIV(WM8505_CLK_SPI0, "spi0", "pllb", WM8505_PMCEL_REG, 12,
		      WM8505_SPI0_DIV_REG, WM8505_DIV_MASK, 0),
	DEF_GATED_DIV(WM8505_CLK_SPI1, "spi1", "pllb", WM8505_PMCEL_REG, 13,
		      WM8505_SPI1_DIV_REG, WM8505_DIV_MASK, 0),
	DEF_GATED_DIV(WM8505_CLK_SPI2, "spi2", "pllb", WM8505_PMCEL_REG, 14,
		      WM8505_SPI2_DIV_REG, WM8505_DIV_MASK, 0),
	DEF_GATED_DIV(WM8505_CLK_PWM, "pwm", "pllb", WM8505_PMCEL_REG, 10,
		      WM8505_PWM_DIV_REG, WM8505_DIV_MASK, 0),
	DEF_GATED_DIV(WM8505_CLK_NA0, "na0", "pllb", WM8505_PMCEU_REG, 1,
		      WM8505_NA0_DIV_REG, WM8505_DIV_MASK, CLK_IS_CRITICAL),
	DEF_GATED_DIV(WM8505_CLK_NA12, "na12", "pllb", WM8505_PMCEU_REG, 2,
		      WM8505_NA12_DIV_REG, WM8505_DIV_MASK, CLK_IS_CRITICAL),
	DEF_GATED_DIV(WM8505_CLK_I2C0, "i2c0", "pllb", WM8505_PMCEL_REG, 5,
		      WM8505_I2C0_DIV_REG, WM8505_DIV_MASK, 0),
	DEF_GATED_DIV(WM8505_CLK_I2C1, "i2c1", "pllb", WM8505_PMCEL_REG, 0,
		      WM8505_I2C1_DIV_REG, WM8505_DIV_MASK, 0),
	DEF_GATED_DIV(WM8505_CLK_DVO, "dvo", "pllc", WM8505_PMCEL_REG, 18,
		      WM8505_DVO_DIV_REG, WM8505_DIV_MASK, CLK_IS_CRITICAL),
};

static void __init wm8505_clk_init(struct device_node *np)
{
	struct clk_hw_onecell_data *clk_data;
	int i, ret;

	pmc_base = of_iomap(np, 0);
	if (!pmc_base) {
		pr_err("%s: failed to map PMC base\n", __func__);
		return;
	}

	clk_data = kzalloc(struct_size(clk_data, hws, WM8505_CLK_MAX), GFP_KERNEL);
	if (!clk_data) {
		iounmap(pmc_base);
		return;
	}

	for (i = 0; i < WM8505_CLK_MAX; i++)
		clk_data->hws[i] = ERR_PTR(-ENOENT);

	clk_data->num = WM8505_CLK_MAX;

	/* Initialize phase-locked loops */
	for (i = 0; i < ARRAY_SIZE(wm8505_pll_clks); i++) {
		const struct wm8505_pll_desc *desc = &wm8505_pll_clks[i];
		struct wm8505_pll_clk *pll = kzalloc(sizeof(*pll), GFP_KERNEL);
		struct clk_parent_data pdata = { .name = desc->parent_name };
		struct clk_init_data init = {
			.name = desc->name,
			.ops = &wm8505_pll_ops,
			.parent_data = &pdata,
			.num_parents = 1,
		};

		if (!pll)
			continue;

		pll->reg_offset = desc->reg_offset;
		pll->hw.init = &init;

		ret = clk_hw_register(NULL, &pll->hw);
		if (!ret) {
			clk_data->hws[desc->id] = &pll->hw;
		} else {
			pr_err("%s: failed to register %s: %d\n", __func__, desc->name, ret);
			kfree(pll);
		}
	}

	/* Initialize pure divisors */
	for (i = 0; i < ARRAY_SIZE(wm8505_div_clks); i++) {
		const struct wm8505_clk_desc *desc = &wm8505_div_clks[i];
		struct wm8505_clk *cdev = kzalloc(sizeof(*cdev), GFP_KERNEL);
		struct clk_parent_data pdata = { .name = desc->parent_name };
		struct clk_init_data init = {
			.name = desc->name,
			.ops = &wm8505_div_ops,
			.parent_data = &pdata,
			.num_parents = 1,
		};

		if (!cdev)
			continue;

		cdev->div_reg_offset = desc->div_reg_offset;
		cdev->div_mask = desc->div_mask;
		cdev->hw.init = &init;

		ret = clk_hw_register(NULL, &cdev->hw);
		if (!ret) {
			clk_data->hws[desc->id] = &cdev->hw;
		} else {
			pr_err("%s: failed to register %s: %d\n", __func__, desc->name, ret);
			kfree(cdev);
		}
	}

	/* Initialize standard gated clocks */
	for (i = 0; i < ARRAY_SIZE(wm8505_gate_clks); i++) {
		const struct wm8505_gate_desc *desc = &wm8505_gate_clks[i];
		struct clk_hw *hw;

		hw = clk_hw_register_gate(NULL, desc->name, desc->parent_name,
					  desc->flags, pmc_base + desc->reg_offset,
					  desc->bit_idx, 0, &wm8505_clk_lock);

		if (IS_ERR(hw))
			pr_err("%s: failed to register %s: %ld\n", __func__,
			       desc->name, PTR_ERR(hw));
		else
			clk_data->hws[desc->id] = hw;
	}

	/* Initialize composite gated divisors */
	for (i = 0; i < ARRAY_SIZE(wm8505_gated_div_clks); i++) {
		const struct wm8505_clk_desc *desc = &wm8505_gated_div_clks[i];
		struct wm8505_clk *cdev = kzalloc(sizeof(*cdev), GFP_KERNEL);
		struct clk_parent_data pdata = { .name = desc->parent_name };
		struct clk_init_data init = {
			.name = desc->name,
			.ops = &wm8505_gated_div_ops,
			.parent_data = &pdata,
			.num_parents = 1,
			.flags = desc->flags,
		};

		if (!cdev)
			continue;

		cdev->en_reg_offset = desc->en_reg_offset;
		cdev->en_bit = desc->en_bit;
		cdev->div_reg_offset = desc->div_reg_offset;
		cdev->div_mask = desc->div_mask;
		cdev->hw.init = &init;

		ret = clk_hw_register(NULL, &cdev->hw);
		if (!ret) {
			clk_data->hws[desc->id] = &cdev->hw;
		} else {
			pr_err("%s: failed to register %s: %d\n", __func__, desc->name, ret);
			kfree(cdev);
		}
	}

	ret = of_clk_add_hw_provider(np, of_clk_hw_onecell_get, clk_data);
	if (ret)
		pr_err("%s: failed to register clock provider: %d\n", __func__, ret);
}

CLK_OF_DECLARE(wm8505_pmc, "wm,wm8505-pmc", wm8505_clk_init);
