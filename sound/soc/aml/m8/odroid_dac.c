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
#include "odroid_dac.h"
#include "aml_audio_hw.h"
#include <linux/amlogic/sound/audin_regs.h>
#include <linux/of.h>
#include <linux/pinctrl/consumer.h>
#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/of_gpio.h>
#include <linux/io.h>
#define DRV_NAME "odroid_dac_snd"
/* extern struct device *spdif_dev; */

static int aml_suspend_pre(struct snd_soc_card *card)
{
	pr_info(KERN_INFO "enter %s\n", __func__);
	return 0;
}

static int aml_suspend_post(struct snd_soc_card *card)
{
	pr_info(KERN_INFO "enter %s\n", __func__);
	/* if(ext_codec) */
	/* i2s_gpio_set(card); */
	return 0;
}

static int aml_resume_pre(struct snd_soc_card *card)
{
	pr_info(KERN_INFO "enter %s\n", __func__);

	return 0;
}

static int aml_resume_post(struct snd_soc_card *card)
{
	pr_info(KERN_INFO "enter %s\n", __func__);
	return 0;
}

static void aml_i2s_pinmux_init(struct snd_soc_card *card)
{
	struct odroid_audio_private_data *p_aml_audio;

	p_aml_audio = snd_soc_card_get_drvdata(card);

	p_aml_audio->pin_ctl =
		devm_pinctrl_get_select(card->dev, "aml_snd_i2s");
	if (IS_ERR(p_aml_audio->pin_ctl))
		pr_info("%s,aml_i2s_pinmux_init error!\n", __func__);
}

static int aml_card_dai_parse_of(struct device *dev,
				 struct snd_soc_dai_link *dai_link,
				 int (*init)(struct snd_soc_pcm_runtime *rtd),
				 struct device_node *cpu_node,
				 struct device_node *codec_node,
				 struct device_node *plat_node)
{
	int ret;

	/* get cpu dai->name */
	ret = snd_soc_of_get_dai_name(cpu_node, &dai_link->cpu_dai_name);
	if (ret < 0)
		goto parse_error;

	/* get codec dai->name */
	ret = snd_soc_of_get_dai_name(codec_node, &dai_link->codec_dai_name);
	if (ret < 0)
		goto parse_error;

	dai_link->name = dai_link->stream_name = dai_link->cpu_dai_name;
	dai_link->codec_of_node = of_parse_phandle(codec_node, "sound-dai", 0);
	dai_link->platform_of_node = plat_node;
	dai_link->init = init;

	return 0;

 parse_error:
	return ret;
}

static int aml_card_dais_parse_of(struct snd_soc_card *card)
{
	struct device_node *np = card->dev->of_node;
	struct device_node *cpu_node, *codec_node, *plat_node;
	struct device *dev = card->dev;
	struct snd_soc_dai_link *dai_links;
	int num_dai_links, cpu_num, codec_num, plat_num;
	int i, ret;
	int (*init)(struct snd_soc_pcm_runtime *rtd);

	ret = of_count_phandle_with_args(np, "cpu_list", NULL);
	if (ret < 0) {
		dev_err(dev, "AML sound card no cpu_list errno: %d\n", ret);
		goto err;
	} else {
		cpu_num = ret;
	}
	ret = of_count_phandle_with_args(np, "codec_list", NULL);
	if (ret < 0) {
		dev_err(dev, "AML sound card no codec_list errno: %d\n", ret);
		goto err;
	} else {
		codec_num = ret;
	}
	ret = of_count_phandle_with_args(np, "plat_list", NULL);
	if (ret < 0) {
		dev_err(dev, "AML sound card no plat_list errno: %d\n", ret);
		goto err;
	} else {
		plat_num = ret;
	}
	if ((cpu_num == codec_num) && (cpu_num == plat_num)) {
		num_dai_links = cpu_num;
	} else {
		dev_err(dev,
			"AML sound card cpu_dai num, codec_dai num, platform num don't match: %d\n",
			ret);
		ret = -EINVAL;
		goto err;
	}

	dai_links =
	    devm_kzalloc(dev, num_dai_links * sizeof(struct snd_soc_dai_link),
			 GFP_KERNEL);
	if (!dai_links) {
		dev_err(dev, "Can't allocate snd_soc_dai_links\n");
		ret = -ENOMEM;
		goto err;
	}
	card->dai_link = dai_links;
	card->num_links = num_dai_links;
	for (i = 0; i < num_dai_links; i++) {
		init = NULL;
		/* CPU sub-node */
		cpu_node = of_parse_phandle(np, "cpu_list", i);
		if (cpu_node < 0) {
			dev_err(dev, "parse aml sound card cpu list error\n");
			return -EINVAL;
		}
		/* CODEC sub-node */
		codec_node = of_parse_phandle(np, "codec_list", i);
		if (codec_node < 0) {
			dev_err(dev, "parse aml sound card codec list error\n");
			return ret;
		}
		/* Platform sub-node */
		plat_node = of_parse_phandle(np, "plat_list", i);
		if (plat_node < 0) {
			dev_err(dev,
				"parse aml sound card platform list error\n");
			return ret;
		}

		ret =
		    aml_card_dai_parse_of(dev, &dai_links[i], init, cpu_node,
					  codec_node, plat_node);
	}

 err:
	return ret;
}

static int odroid_dac_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct snd_soc_card *card;
	struct odroid_audio_private_data *p_aml_audio;
	int ret;

	p_aml_audio =
	    devm_kzalloc(dev, sizeof(struct odroid_audio_private_data),
			 GFP_KERNEL);
	if (!p_aml_audio) {
		dev_err(&pdev->dev, "Can't allocate odroid_audio_private_data\n");
		ret = -ENOMEM;
		goto err;
	}

	card = devm_kzalloc(dev, sizeof(struct snd_soc_card), GFP_KERNEL);
	if (!card) {
		dev_err(dev, "Can't allocate snd_soc_card\n");
		ret = -ENOMEM;
		goto err;
	}

	snd_soc_card_set_drvdata(card, p_aml_audio);
	card->dev = dev;
	ret = snd_soc_of_parse_card_name(card, "aml_sound_card,name");
	if (ret < 0) {
		dev_err(dev, "no specific snd_soc_card name\n");
		goto err;
	}

	ret = aml_card_dais_parse_of(card);
	if (ret < 0) {
		dev_err(dev, "parse aml sound card routing error %d\n",
			ret);
		goto err;
	}

	card->suspend_pre = aml_suspend_pre,
	card->suspend_post = aml_suspend_post,
	card->resume_pre = aml_resume_pre,
	card->resume_post = aml_resume_post,
	ret = devm_snd_soc_register_card(&pdev->dev, card);
	if (ret < 0) {
		dev_err(dev, "register aml sound card error %d\n", ret);
		goto err;
	}

	aml_i2s_pinmux_init(card);
	return 0;
 err:
	dev_err(dev, "Can't probe snd_soc_card\n");
	return ret;
}

static const struct of_device_id odroid_dac_of_match[] = {
	{.compatible = "sound_card, odroid_dac",},
	{},
};

static struct platform_driver odroid_dac_audio_driver = {
	.driver = {
		   .name = DRV_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = odroid_dac_of_match,
		   },
	.probe = odroid_dac_probe,
};

static int __init odroid_audio_init(void)
{
	return platform_driver_register(&odroid_dac_audio_driver);
}

static void __exit odroid_audio_exit(void)
{
	platform_driver_unregister(&odroid_dac_audio_driver);
}

#ifdef CONFIG_DEFERRED_MODULE_INIT
deferred_module_init(odroid_audio_init);
#else
module_init(odroid_audio_init);
#endif
module_exit(odroid_audio_exit);

MODULE_AUTHOR("Hardkernel, Inc.");
MODULE_DESCRIPTION("ODROID audio machine Asoc driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
