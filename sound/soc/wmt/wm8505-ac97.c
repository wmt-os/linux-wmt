// SPDX-License-Identifier: GPL-2.0-only
/*
 * WonderMedia WM8505 AC97 Machine Driver
 *
 * Copyright (C) 2026 Logan Russell <me@lrussell.net>
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/dmaengine.h>
#include <linux/gpio/consumer.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>
#include <sound/ac97_codec.h>

/* WM8505 AC97 Controller Registers */
#define WM8505_ACCR		0x0
#define WM8505_ACSR		0x4
#define WM8505_CCR		0xc
#define WM8505_CSDR		0x10
#define WM8505_PTCR		0x20
#define WM8505_PRCR		0x28
#define WM8505_PTFIFO		0x80
#define WM8505_PRFIFO		0xc0

/* Register Bitfields */
#define ACCR_COLD_RESET		BIT(0)
#define ACCR_WARM_RESET		BIT(1)

#define ACSR_CRDY		BIT(0)
#define ACSR_CWD		BIT(1)
#define ACSR_CRD		BIT(2)
#define ACSR_CST		BIT(3)

#define CCR_READ_FLAG		BIT(7)
#define CCR_REG_MASK		0x7f
#define CSDR_DATA_MASK		0xffff

#define P_CTRL_ENABLE		BIT(0)
#define P_CTRL_DMA_EN		BIT(3)
#define P_CTRL_FIFO_THRESH(x)	((x) << 8)

/* AC97 Pinmux Select */
#define WMT_PINMUX_SEL		0x200
#define WMT_PINMUX_AC97		BIT(1)

#define AC97_POLL_US		21
#define AC97_TIMEOUT_US		1000
#define DMA_MAXBURST		8

struct wm8505_ac97 {
	struct device				*dev;
	void __iomem				*regs;
	struct mutex				lock;	/* Protects hardware registers */
	struct snd_dmaengine_dai_dma_data	playback_dma_data;
	struct snd_dmaengine_dai_dma_data	capture_dma_data;
	struct platform_device			*ac97_pdev;
	struct gpio_desc			*speaker_gpio;
};

static struct wm8505_ac97 *wm8505_ac97_priv;

/*
 * wm8505_ac97_read - Read a register from the AC97 codec
 */
static unsigned short wm8505_ac97_read(struct snd_ac97 *ac97, unsigned short reg)
{
	struct wm8505_ac97 *priv = wm8505_ac97_priv;
	u32 val;
	int ret;

	mutex_lock(&priv->lock);

	ret = readl_poll_timeout_atomic(priv->regs + WM8505_ACSR, val,
					val & ACSR_CRDY, AC97_POLL_US, AC97_TIMEOUT_US);
	if (ret) {
		dev_err(priv->dev, "AC97 read timeout waiting for CRDY\n");
		mutex_unlock(&priv->lock);
		return 0xffff;
	}

	writel((reg & CCR_REG_MASK) | CCR_READ_FLAG, priv->regs + WM8505_CCR);

	ret = readl_poll_timeout_atomic(priv->regs + WM8505_ACSR, val,
					val & (ACSR_CRD | ACSR_CST),
					AC97_POLL_US, AC97_TIMEOUT_US);
	if (ret || (val & ACSR_CST)) {
		dev_err(priv->dev, "AC97 read command failed (CST or timeout)\n");
		writel(ACSR_CST, priv->regs + WM8505_ACSR);
		mutex_unlock(&priv->lock);
		return 0xffff;
	}

	writel(ACSR_CRD, priv->regs + WM8505_ACSR);
	val = readl(priv->regs + WM8505_CSDR) & CSDR_DATA_MASK;

	mutex_unlock(&priv->lock);
	return (unsigned short)val;
}

/*
 * wm8505_ac97_write - Write a register to the AC97 codec
 */
static void wm8505_ac97_write(struct snd_ac97 *ac97, unsigned short reg, unsigned short data)
{
	struct wm8505_ac97 *priv = wm8505_ac97_priv;
	u32 val;
	int ret;

	mutex_lock(&priv->lock);

	ret = readl_poll_timeout_atomic(priv->regs + WM8505_ACSR, val,
					val & ACSR_CRDY, AC97_POLL_US, AC97_TIMEOUT_US);
	if (ret) {
		dev_err(priv->dev, "AC97 write timeout waiting for CRDY\n");
		mutex_unlock(&priv->lock);
		return;
	}

	writel((data << 16) | (reg & CCR_REG_MASK), priv->regs + WM8505_CCR);

	ret = readl_poll_timeout_atomic(priv->regs + WM8505_ACSR, val,
					val & ACSR_CWD, AC97_POLL_US, AC97_TIMEOUT_US);
	if (ret)
		dev_err(priv->dev, "AC97 write command timeout (CWD)\n");
	else
		writel(ACSR_CWD, priv->regs + WM8505_ACSR);

	mutex_unlock(&priv->lock);
}

/*
 * wm8505_ac97_cold_reset - Issue a cold reset to the AC97 codec
 */
static void wm8505_ac97_cold_reset(struct snd_ac97 *ac97)
{
	struct wm8505_ac97 *priv = wm8505_ac97_priv;

	writel(ACCR_COLD_RESET, priv->regs + WM8505_ACCR);
	udelay(2);
	writel(0, priv->regs + WM8505_ACCR);

	/* Set FIFO thresholds */
	writel(P_CTRL_FIFO_THRESH(DMA_MAXBURST), priv->regs + WM8505_PTCR);
	writel(P_CTRL_FIFO_THRESH(DMA_MAXBURST), priv->regs + WM8505_PRCR);
}

/*
 * wm8505_ac97_warm_reset - Issue a warm reset to the AC97 codec
 */
static void wm8505_ac97_warm_reset(struct snd_ac97 *ac97)
{
	struct wm8505_ac97 *priv = wm8505_ac97_priv;

	writel(ACCR_WARM_RESET, priv->regs + WM8505_ACCR);
	udelay(2);
	writel(0, priv->regs + WM8505_ACCR);
}

static struct snd_ac97_bus_ops wm8505_ac97_bus_ops = {
	.read		= wm8505_ac97_read,
	.write		= wm8505_ac97_write,
	.reset		= wm8505_ac97_cold_reset,
	.warm_reset	= wm8505_ac97_warm_reset,
};

/*
 * wm8505_ac97_trigger - Start/stop DMA and audio data path
 */
static int wm8505_ac97_trigger(struct snd_pcm_substream *substream, int cmd,
			       struct snd_soc_dai *dai)
{
	struct wm8505_ac97 *priv = wm8505_ac97_priv;
	u32 val;
	void __iomem *ctrl_reg = (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
				 (priv->regs + WM8505_PTCR) : (priv->regs + WM8505_PRCR);

	val = readl(ctrl_reg);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		val |= P_CTRL_ENABLE | P_CTRL_DMA_EN;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		val &= ~P_CTRL_DMA_EN;
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
			val &= ~P_CTRL_ENABLE;
		break;
	default:
		return -EINVAL;
	}

	writel(val, ctrl_reg);
	return 0;
}

/*
 * wm8505_ac97_dai_probe - Initialize DAI DMA data
 */
static int wm8505_ac97_dai_probe(struct snd_soc_dai *dai)
{
	struct wm8505_ac97 *priv = wm8505_ac97_priv;

	snd_soc_dai_init_dma_data(dai, &priv->playback_dma_data,
				  &priv->capture_dma_data);
	return 0;
}

static const struct snd_soc_dai_ops wm8505_ac97_dai_ops = {
	.probe		= wm8505_ac97_dai_probe,
	.trigger	= wm8505_ac97_trigger,
};

/*
 * Hardware expects (Left << 16) | Right in 32-bit registers.
 * ALSA provides S16_LE which DMA reads as (Right << 16) | Left.
 * Expose the resulting channel map to userspace.
 */
static const struct snd_pcm_chmap_elem wm8505_chmap[] = {
	{ .channels = 2,
	  .map = { SNDRV_CHMAP_FR, SNDRV_CHMAP_FL } },
	{ }
};

/*
 * wm8505_ac97_pcm_construct - Install ALSA channel map controls
 */
static int wm8505_ac97_pcm_construct(struct snd_soc_component *component,
				     struct snd_soc_pcm_runtime *rtd)
{
	int ret;

	ret = snd_pcm_add_chmap_ctls(rtd->pcm, SNDRV_PCM_STREAM_PLAYBACK,
				     wm8505_chmap, 2, 0, NULL);
	if (ret)
		return ret;

	return snd_pcm_add_chmap_ctls(rtd->pcm, SNDRV_PCM_STREAM_CAPTURE,
				      wm8505_chmap, 2, 0, NULL);
}

static struct snd_soc_dai_driver wm8505_ac97_dai_drv = {
	.name = "wm8505-ac97",
	.playback = {
		.stream_name	= "Playback",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_8000_48000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name	= "Capture",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_8000_48000,
		.formats	= SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &wm8505_ac97_dai_ops,
};

static const struct snd_soc_component_driver wm8505_ac97_component = {
	.name		= "wm8505-ac97-cpu",
	.pcm_construct	= wm8505_ac97_pcm_construct,
};

/*
 * wm8505_spk_event - DAPM event handler for the speaker amplifier
 */
static int wm8505_spk_event(struct snd_soc_dapm_widget *w,
			    struct snd_kcontrol *k, int event)
{
	struct wm8505_ac97 *priv = snd_soc_card_get_drvdata(w->dapm->card);

	gpiod_set_value_cansleep(priv->speaker_gpio, SND_SOC_DAPM_EVENT_ON(event));
	return 0;
}

/* Only the speaker path is user-switchable */
static const struct snd_kcontrol_new wm8505_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speaker"),
};

static const struct snd_soc_dapm_widget wm8505_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Speaker", wm8505_spk_event),
	SND_SOC_DAPM_HP("Headphone", NULL),
};

static const struct snd_soc_dapm_route wm8505_dapm_routes[] = {
	{ "Speaker", NULL, "TX" },
	{ "Headphone", NULL, "TX" },
};

SND_SOC_DAILINK_DEFS(ac97_hifi,
		     DAILINK_COMP_ARRAY(COMP_CPU("wm8505-ac97")),
		     DAILINK_COMP_ARRAY(COMP_CODEC("ac97-codec", "ac97-hifi")),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

static struct snd_soc_dai_link wm8505_ac97_dai_link = {
	.name		= "AC97",
	.stream_name	= "AC97 HiFi",
	SND_SOC_DAILINK_REG(ac97_hifi),
};

static struct snd_soc_card wm8505_ac97_card = {
	.name			= "wm8505-ac97",
	.owner			= THIS_MODULE,
	.dai_link		= &wm8505_ac97_dai_link,
	.num_links		= 1,
	.controls		= wm8505_controls,
	.num_controls		= ARRAY_SIZE(wm8505_controls),
	.dapm_widgets		= wm8505_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(wm8505_dapm_widgets),
	.dapm_routes		= wm8505_dapm_routes,
	.num_dapm_routes	= ARRAY_SIZE(wm8505_dapm_routes),
};

/*
 * wm8505_dma_prepare_slave_config - Setup DMA bus width for AC97 FIFOs
 */
static int wm8505_dma_prepare_slave_config(struct snd_pcm_substream *substream,
					   struct snd_pcm_hw_params *params,
					   struct dma_slave_config *slave_config)
{
	int ret = snd_dmaengine_pcm_prepare_slave_config(substream, params, slave_config);

	if (ret)
		return ret;

	slave_config->src_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	slave_config->dst_addr_width = DMA_SLAVE_BUSWIDTH_4_BYTES;
	return 0;
}

/* Advertise hardware limits to avoid overrunning the DMA descriptor pool. */
static const struct snd_pcm_hardware wm8505_pcm_hardware = {
	.info			= SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID |
				  SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_PAUSE,
	.period_bytes_min	= 64,
	.period_bytes_max	= 32768,
	.periods_min		= 2,
	.periods_max		= 32,
	.buffer_bytes_max	= 65536,
};

static const struct snd_dmaengine_pcm_config wm8505_dmaengine_pcm_config = {
	.pcm_hardware		= &wm8505_pcm_hardware,
	.prealloc_buffer_size	= 65536,
	.prepare_slave_config	= wm8505_dma_prepare_slave_config,
};

/*
 * Devres teardown handlers
 */
static void wm8505_ac97_ops_teardown(void *data)
{
	snd_soc_set_ac97_ops(NULL);
}

static void wm8505_ac97_codec_teardown(void *data)
{
	platform_device_unregister(data);
}

/*
 * wm8505_ac97_probe - Platform device probe
 */
static int wm8505_ac97_probe(struct platform_device *pdev)
{
	struct wm8505_ac97 *priv;
	struct resource *res;
	struct regmap *pinmux;
	struct clk *clk;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;

	ret = devm_mutex_init(&pdev->dev, &priv->lock);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, priv);

	priv->regs = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(priv->regs))
		return PTR_ERR(priv->regs);

	/* Enable clocks */
	clk = devm_clk_get_enabled(&pdev->dev, "ac97");
	if (IS_ERR(clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(clk),
				     "Failed to get/enable AC97 clock\n");

	/*
	 * Route hardware pins to AC97 instead of I2S.
	 * The pinctrl-wm8505 driver does not manage this global switch.
	 */
	pinmux = syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "wm,pinmux");
	if (IS_ERR(pinmux))
		return dev_err_probe(&pdev->dev, PTR_ERR(pinmux),
				     "Failed to get pinmux syscon\n");

	ret = regmap_set_bits(pinmux, WMT_PINMUX_SEL, WMT_PINMUX_AC97);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to set AC97 pinmux\n");

	priv->playback_dma_data.addr		= res->start + WM8505_PTFIFO;
	priv->playback_dma_data.addr_width	= DMA_SLAVE_BUSWIDTH_4_BYTES;
	priv->playback_dma_data.maxburst	= DMA_MAXBURST;

	priv->capture_dma_data.addr		= res->start + WM8505_PRFIFO;
	priv->capture_dma_data.addr_width	= DMA_SLAVE_BUSWIDTH_4_BYTES;
	priv->capture_dma_data.maxburst		= DMA_MAXBURST;

	ret = devm_snd_dmaengine_pcm_register(&pdev->dev, &wm8505_dmaengine_pcm_config, 0);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to register DMA engine PCM\n");

	wm8505_ac97_priv = priv;
	ret = snd_soc_set_ac97_ops(&wm8505_ac97_bus_ops);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to set AC97 operations\n");

	ret = devm_add_action_or_reset(&pdev->dev, wm8505_ac97_ops_teardown, NULL);
	if (ret)
		return ret;

	priv->ac97_pdev = platform_device_register_simple("ac97-codec", -1, NULL, 0);
	if (IS_ERR(priv->ac97_pdev))
		return dev_err_probe(&pdev->dev, PTR_ERR(priv->ac97_pdev),
				     "Failed to register ac97-codec\n");

	ret = devm_add_action_or_reset(&pdev->dev, wm8505_ac97_codec_teardown, priv->ac97_pdev);
	if (ret)
		return ret;

	priv->speaker_gpio = devm_gpiod_get_optional(&pdev->dev, "speaker", GPIOD_OUT_LOW);
	if (IS_ERR(priv->speaker_gpio))
		return dev_err_probe(&pdev->dev, PTR_ERR(priv->speaker_gpio),
				     "Failed to request speaker GPIO\n");
	snd_soc_card_set_drvdata(&wm8505_ac97_card, priv);

	ret = devm_snd_soc_register_component(&pdev->dev, &wm8505_ac97_component,
					      &wm8505_ac97_dai_drv, 1);
	if (ret)
		return ret;

	wm8505_ac97_dai_link.cpus->of_node	= pdev->dev.of_node;
	wm8505_ac97_dai_link.platforms->of_node	= pdev->dev.of_node;
	wm8505_ac97_card.dev			= &pdev->dev;

	ret = devm_snd_soc_register_card(&pdev->dev, &wm8505_ac97_card);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to register sound card\n");

	dev_info(&pdev->dev, "WM8505 AC97 Audio Subsystem initialized\n");
	return 0;
}

static const struct of_device_id wm8505_ac97_of_match[] = {
	{ .compatible = "wm,wm8505-ac97", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wm8505_ac97_of_match);

static struct platform_driver wm8505_ac97_driver = {
	.probe = wm8505_ac97_probe,
	.driver = {
		.name = "wm8505-ac97",
		.of_match_table = wm8505_ac97_of_match,
	},
};
module_platform_driver(wm8505_ac97_driver);

MODULE_DESCRIPTION("WonderMedia WM8505 AC97 Machine Driver");
MODULE_AUTHOR("Logan Russell <me@lrussell.net>");
MODULE_LICENSE("GPL");
