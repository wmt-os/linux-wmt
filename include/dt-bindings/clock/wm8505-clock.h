/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Device Tree bindings for WonderMedia WM8505 Clock Controller
 *
 * Copyright (C) 2026 Logan Russell <me@lrussell.net>
 */

#ifndef __DT_BINDINGS_CLOCK_WM8505_H
#define __DT_BINDINGS_CLOCK_WM8505_H

/* Phase-Locked Loops */
#define WM8505_CLK_PLLA		0
#define WM8505_CLK_PLLB		1
#define WM8505_CLK_PLLC		2
#define WM8505_CLK_PLLD		3

/* Pure Divisor Clocks */
#define WM8505_CLK_ARM		4
#define WM8505_CLK_AHB		5
#define WM8505_CLK_APB		6
#define WM8505_CLK_PS2KBDC_PRE	7

/* PMCEL_REG (0x250) Gated Clocks */
#define WM8505_CLK_UART0	8
#define WM8505_CLK_UART1	9
#define WM8505_CLK_UART2	10
#define WM8505_CLK_UART3	11
#define WM8505_CLK_I2CSLAVE	12
#define WM8505_CLK_RTC		13
#define WM8505_CLK_KEYPAD	14
#define WM8505_CLK_GPIO		15
#define WM8505_CLK_I2S		16
#define WM8505_CLK_CIR		17
#define WM8505_CLK_AC97		18
#define WM8505_CLK_SCC		19
#define WM8505_CLK_UART4	20
#define WM8505_CLK_UART5	21
#define WM8505_CLK_AMP		22
#define WM8505_CLK_JENC		23
#define WM8505_CLK_GE		24
#define WM8505_CLK_GOVRHD	25

/* PMCEU_REG (0x254) Gated Clocks */
#define WM8505_CLK_DMA		26
#define WM8505_CLK_UHC		27
#define WM8505_CLK_UDC		28
#define WM8505_CLK_PDMA		29
#define WM8505_CLK_AHBBRIDGE	30
#define WM8505_CLK_SDTV		31
#define WM8505_CLK_SYS		32
#define WM8505_CLK_SAE		33
#define WM8505_CLK_ETHPHY	34
#define WM8505_CLK_SCL444U	35
#define WM8505_CLK_GOVW		36
#define WM8505_CLK_VID		37
#define WM8505_CLK_VPP		38

/* Gated Divisor Clocks (Composite) */
#define WM8505_CLK_DDR		39
#define WM8505_CLK_SFC		40
#define WM8505_CLK_PS2KBDC	41
#define WM8505_CLK_SDHC		42
#define WM8505_CLK_MAC0		43
#define WM8505_CLK_NAND		44
#define WM8505_CLK_NORGUP	45
#define WM8505_CLK_SPI0		46
#define WM8505_CLK_SPI1		47
#define WM8505_CLK_SPI2		48
#define WM8505_CLK_PWM		49
#define WM8505_CLK_NA0		50
#define WM8505_CLK_NA12		51
#define WM8505_CLK_I2C0		52
#define WM8505_CLK_I2C1		53
#define WM8505_CLK_DVO		54

#define WM8505_CLK_MAX		55

#endif /* __DT_BINDINGS_CLOCK_WM8505_H */
