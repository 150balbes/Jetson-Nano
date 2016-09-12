/*
 * sound/soc/aml/m8/aml_m8.c
 *
 * Copyright (C) 2015 Amlogic, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
/* #include <sound/soc-dapm.h> */
#include <sound/jack.h>
#include <linux/switch.h>
/* #include <linux/amlogic/saradc.h> */
#include <linux/amlogic/iomap.h>

/* #include "aml_i2s_dai.h" */
#include "aml_i2s.h"
#include "odroid_dac2.h"
#include "aml_audio_hw.h"
#include <linux/amlogic/sound/audin_regs.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/of_gpio.h>
#include <linux/io.h>
/* extern struct device *spdif_dev; */

#define DRV_NAME "odroid_dac2"

static int dac2_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
	SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
	SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set cpu DAI clock */
	ret = snd_soc_dai_set_sysclk(
			cpu_dai, 0, params_rate(params)*256, SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;
	return 0;
}

static struct snd_soc_ops odroid_ops = {
	.hw_params = dac2_hw_params,
};

static int dac2_set_bias_level(struct snd_soc_card *card,
	struct snd_soc_dapm_context *dapm, enum snd_soc_bias_level level)
{
	int ret = 0;
	struct dac2_private_data *p_dac2;

	p_dac2 = snd_soc_card_get_drvdata(card);
	if (p_dac2->bias_level == (int)level)
		return 0;

	p_dac2->bias_level = (int)level;
	return ret;
}
static struct snd_soc_dai_link dac2_dai_link[] = {
	{
		.name = "SND_PCM5242",
		.stream_name = "I2S",
		.cpu_dai_name = "I2S",
		.platform_name = "i2s_platform",
		.codec_name = "pcm512x.1-004c",
		.codec_dai_name = "pcm512x-hifi",
		.ops = &odroid_ops,
	},
};

static struct snd_soc_card aml_snd_soc_card = {
	.driver_name = "SOC-Audio",
	.dai_link = &dac2_dai_link[0],
	.num_links = ARRAY_SIZE(dac2_dai_link),
	.set_bias_level = dac2_set_bias_level,
};

static int dac2_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &aml_snd_soc_card;
	struct dac2_private_data *p_dac2;
	int ret = 0;

#ifdef CONFIG_OF
	p_dac2 = devm_kzalloc(&pdev->dev,
	sizeof(struct dac2_private_data), GFP_KERNEL);
	if (!p_dac2) {
		dev_err(&pdev->dev, "Can't allocate dac2_private_data\n");
		ret = -ENOMEM;
		goto err;
	}

	card->dev = &pdev->dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, p_dac2);
	if (!(pdev->dev.of_node)) {
		dev_err(&pdev->dev, "Must be instantiated using device tree\n");
		ret = -EINVAL;
		goto err;
	}
	ret = of_property_read_string(pdev->dev.of_node,
					"pinctrl-names",
					&p_dac2->pinctrl_name);
	p_dac2->pin_ctl =
		devm_pinctrl_get_select(&pdev->dev, p_dac2->pinctrl_name);

	ret = snd_soc_of_parse_card_name(card, "aml,sound_card");
	if (ret)
		goto err;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
		ret);
		goto err;
	}
	return 0;
#endif

err:
	return ret;
}

static int dac2_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct dac2_private_data *p_dac2;

	p_dac2 = snd_soc_card_get_drvdata(card);
	if (p_dac2->pin_ctl)
		devm_pinctrl_put(p_dac2->pin_ctl);

	snd_soc_unregister_card(card);
	return 0;
}

#ifdef CONFIG_USE_OF
static const struct of_device_id dac2_dt_match[] = {
	{ .compatible = "sound_card, odroid_dac2", },
	{},
};
#else
#define dac2_dt_match NULL
#endif

static struct platform_driver dac2_driver = {
	.probe  = dac2_probe,
	.remove = dac2_remove,
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = dac2_dt_match,
	},
};

static int __init dac2_init(void)
{
	return platform_driver_register(&dac2_driver);
}

static void __exit dac2_exit(void)
{
	platform_driver_unregister(&dac2_driver);
}

#ifdef CONFIG_DEFERRED_MODULE_INIT
deferred_module_init(dac2_init);
#else
module_init(dac2_init);
#endif
module_exit(dac2_exit);

/* Module information */
MODULE_AUTHOR("Hardkernel, Inc.");
MODULE_DESCRIPTION("ODROID HiFi Shield-2 Asoc driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
