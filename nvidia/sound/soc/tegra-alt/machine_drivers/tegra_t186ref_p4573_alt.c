/*
 * tegra_t186ref_p4573_alt.c - Tegra t186 Machine driver for P4573 board
 *
 * Copyright (c) 2016-2019 NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/version.h>

#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "tegra_asoc_utils_alt.h"
#include "tegra_asoc_machine_alt.h"

#include "../codecs/rt5659.h"

#define DRV_NAME "t186-p4573-audio"

#define CS53L30_MAX_INDEX 2
#define RT5658_INDEX CS53L30_MAX_INDEX

static const char * const dai_link_names[] = {
	"cs53l30-link1", "cs53l30-link2", "rt5658-link",
};

struct tegra_t186 {
	struct tegra_asoc_audio_clock_info audio_clock;
	struct snd_soc_dai_link *codec_link;
	unsigned int num_codec_links;
	struct snd_soc_card *pcard;
	int unmute_count;
	bool dai_link_probed[ARRAY_SIZE(dai_link_names)];
};

static int tegra186_set_params(struct snd_soc_pcm_stream *dai_params,
			       struct snd_soc_dai *cpu_dai,
			       struct snd_soc_pcm_runtime *rtd,
			       struct snd_soc_card *card, int rate,
			       int channels, int formats)
{
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int fmt = rtd->dai_link->dai_fmt;
	unsigned int tx_mask = (1 << channels) - 1;
	unsigned int rx_mask = (1 << channels) - 1;
	int ret = 0, slot_size;

	/* Update link_param to update hw_param for DAPM */
	dai_params->rate_min = rate;
	dai_params->channels_min = channels;
	dai_params->formats = 1ULL << formats;

	switch (formats) {
	case SNDRV_PCM_FORMAT_S8:
		slot_size = 8;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		slot_size = 16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		slot_size = 24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		slot_size = 32;
		break;
	default:
		return -EINVAL;
	}

	/* Only apply TDM related configurations to DSP_A format */
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_DSP_A)
		return 0;

	ret = snd_soc_dai_set_tdm_slot(cpu_dai,	tx_mask, rx_mask, 0, 0);
	if (ret) {
		dev_err(card->dev, "%s cpu DAI slot mask not set\n",
				cpu_dai->name);
		return ret;
	}

	/* Set tdm slot of CODEC dai using hard-code provisionally */
	ret = snd_soc_dai_set_tdm_slot(codec_dai, tx_mask, rx_mask,
			channels, slot_size);
	if (ret) {
		dev_err(card->dev, "%s codec DAI slot mask not set\n",
				codec_dai->name);
		return ret;
	}

	return 0;
}

static int tegra186_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_t186 *machine = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_card *card = rtd->card;
	struct snd_soc_pcm_stream *dai_params;
	struct snd_soc_dai_link *dai_link;
	int channels = params_channels(params);
	int formats = params_format(params);
	int rate = params_rate(params);
	unsigned int clk_out_rate, bclk_ratio;
	int ret, i;

	ret = tegra_alt_asoc_utils_set_rate(&machine->audio_clock, rate, 0, 0);
	if (ret < 0) {
		dev_err(card->dev, "Can't configure clocks\n");
		return ret;
	}

	clk_out_rate = machine->audio_clock.set_aud_mclk_rate;

	dev_dbg(card->dev,
		"pll_a_out0 = %d Hz, aud_mclk = %d Hz, codec rate = %d Hz\n",
		machine->audio_clock.set_pll_out_rate, clk_out_rate, rate);

	/* Update dai link hw_params for non pcm links */
	i = 0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	for ( ; i < TEGRA186_DAI_LINK_ADMAIF20; ) {
		rtd = &card->rtd[i];
#else
	list_for_each_entry(rtd, &card->rtd_list, list) {
		if (i >= TEGRA186_DAI_LINK_ADMAIF20)
			break;
#endif
		if (!rtd->dai_link->params) {
			i++;
			continue;
		}
		dai_link = rtd->dai_link;
		dai_params = (struct snd_soc_pcm_stream *) dai_link->params;
		dai_params->rate_min = rate;
		dai_params->channels_min = channels;
		i++;
	}

	for (i = 0; i < ARRAY_SIZE(dai_link_names); i++) {
		if (!machine->dai_link_probed[i])
			continue;

		rtd = snd_soc_get_pcm_runtime(card, dai_link_names[i]);
		if (!rtd) {
			dev_warn(card->dev, "failed to get pcm runtime: %s\n",
				 dai_link_names[i]);
			continue;
		}

		dai_link = rtd->dai_link;
		dai_params = (struct snd_soc_pcm_stream *)dai_link->params;

		ret = snd_soc_dai_set_sysclk(rtd->codec_dai, 0,
					     clk_out_rate, SND_SOC_CLOCK_IN);
		if (ret) {
			dev_err(card->dev, "codec_dai clock not set\n");
			return ret;
		}

		ret = tegra_machine_get_bclk_ratio_t18x(rtd, &bclk_ratio);
		if (ret) {
			dev_err(card->dev, "Failed to get cpu dai bclk ratio for %s\n",
					rtd->dai_link->name);
			return ret;
		}

		ret = snd_soc_dai_set_bclk_ratio(rtd->cpu_dai, bclk_ratio);
		if (ret) {
			dev_err(card->dev, "Failed to set cpu dai bclk ratio for %s\n",
					rtd->dai_link->name);
			return ret;
		}

		ret = tegra186_set_params(dai_params, rtd->cpu_dai, rtd, card,
					  rate, channels, formats);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int tegra186_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_t186 *machine = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_card *card = rtd->card;
	int i;

	if (!machine->unmute_count) {
		for (i = 0; i < CS53L30_MAX_INDEX; i++) {
			rtd = snd_soc_get_pcm_runtime(card, dai_link_names[i]);
			if (rtd)
				snd_soc_dai_digital_mute(rtd->codec_dai, 0, 0);
		}
	}

	machine->unmute_count++;

	return 0;
}

static void tegra186_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct tegra_t186 *machine = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_card *card = rtd->card;
	int i;

	machine->unmute_count--;

	if (!machine->unmute_count) {
		for (i = 0; i < CS53L30_MAX_INDEX; i++) {
			rtd = snd_soc_get_pcm_runtime(card, dai_link_names[i]);
			if (rtd)
				snd_soc_dai_digital_mute(rtd->codec_dai, 1, 0);
		}
	}
}

static struct snd_soc_ops tegra186_ops = {
	.startup = tegra186_startup,
	.shutdown = tegra186_shutdown,
	.hw_params = tegra186_hw_params,
};

static int tegra186_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct snd_soc_dapm_context *dapm = snd_soc_codec_get_dapm(codec);

	return snd_soc_dapm_sync(dapm);
}

static const struct snd_soc_dapm_widget tegra186_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("AMIC1", NULL),
	SND_SOC_DAPM_MIC("AMIC2", NULL),
	SND_SOC_DAPM_MIC("AMIC3", NULL),
	SND_SOC_DAPM_MIC("AMIC4", NULL),
	SND_SOC_DAPM_MIC("DMIC1", NULL),
	SND_SOC_DAPM_MIC("DMIC2", NULL),
	SND_SOC_DAPM_MIC("AMIC5", NULL),
	SND_SOC_DAPM_MIC("AMIC6", NULL),
	SND_SOC_DAPM_MIC("AMIC7", NULL),
	SND_SOC_DAPM_MIC("AMIC8", NULL),
	SND_SOC_DAPM_MIC("DMIC3", NULL),
	SND_SOC_DAPM_MIC("DMIC4", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_HP("Headset Stereo", NULL),
};

static int tegra186_suspend_pre(struct snd_soc_card *card)
{
	struct snd_soc_pcm_runtime *rtd;

	/* DAPM dai link stream work for non pcm links */
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	unsigned int idx;

	for (idx = 0; idx < card->num_rtd; idx++) {
		rtd = &card->rtd[idx];
#else
	list_for_each_entry(rtd, &card->rtd_list, list) {
#endif
		if (rtd->dai_link->params)
			INIT_DELAYED_WORK(&rtd->delayed_work, NULL);
	}

	return 0;
}

static struct snd_soc_card snd_soc_tegra_t186 = {
	.name = DRV_NAME,
	.owner = THIS_MODULE,
	.suspend_pre = tegra186_suspend_pre,
	.dapm_widgets = tegra186_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tegra186_dapm_widgets),
	.fully_routed = true,
};

static bool tegra186_search_dai_link_name(const char *name)
{
	int i = 0;

	for (i = 0; i < ARRAY_SIZE(dai_link_names); i++) {
		if (strstr(name, dai_link_names[i]))
			break;
	}

	return i < ARRAY_SIZE(dai_link_names);
}

static void dai_link_setup(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_t186 *machine = snd_soc_card_get_drvdata(card);
	struct snd_soc_codec_conf *tegra_machine_codec_conf = NULL;
	struct snd_soc_codec_conf *tegra186_codec_conf = NULL;
	struct snd_soc_dai_link *tegra_machine_dai_links = NULL;
	struct snd_soc_dai_link *tegra186_codec_links = NULL;
	int i;

	/* Set new codec links and conf */
	tegra186_codec_links = tegra_machine_new_codec_links(pdev,
			tegra186_codec_links, &machine->num_codec_links);
	if (!tegra186_codec_links)
		goto err_alloc_dai_link;

	machine->codec_link = tegra186_codec_links;

	/* Set codec init */
	for (i = 0; i < machine->num_codec_links; i++) {
		if (tegra186_search_dai_link_name(tegra186_codec_links[i].name))
			tegra186_codec_links[i].init = tegra186_init;
	}

	tegra186_codec_conf = tegra_machine_new_codec_conf(pdev,
			tegra186_codec_conf, &machine->num_codec_links);
	if (!tegra186_codec_conf)
		goto err_alloc_dai_link;

	/* Get the xbar dai link/codec conf structure */
	tegra_machine_dai_links = tegra_machine_get_dai_link_t18x();
	if (!tegra_machine_dai_links)
		goto err_alloc_dai_link;

	tegra_machine_codec_conf = tegra_machine_get_codec_conf_t18x();
	if (!tegra_machine_codec_conf)
		goto err_alloc_dai_link;

	/* Set ADMAIF dai_ops */
	for (i = TEGRA186_DAI_LINK_ADMAIF1;
		i <= TEGRA186_DAI_LINK_ADMAIF20; i++)
		tegra_machine_set_dai_ops(i, &tegra186_ops);

	/* Set ADSP PCM/COMPR */
#if IS_ENABLED(CONFIG_SND_SOC_TEGRA210_ADSP_ALT)
	for (i = TEGRA186_DAI_LINK_ADSP_PCM1;
		i <= TEGRA186_DAI_LINK_ADSP_PCM2; i++) {
		tegra_machine_set_dai_ops(i, &tegra186_ops);
	}
#endif

	/* Append t186 specific dai_links */
	card->num_links =
		tegra_machine_append_dai_link_t18x(tegra186_codec_links,
				2 * machine->num_codec_links);
	tegra_machine_dai_links = tegra_machine_get_dai_link_t18x();
	card->dai_link = tegra_machine_dai_links;

	/* Append t186 specific codec_conf */
	card->num_configs =
		tegra_machine_append_codec_conf_t18x(tegra186_codec_conf,
				machine->num_codec_links);
	tegra_machine_codec_conf = tegra_machine_get_codec_conf_t18x();
	card->codec_conf = tegra_machine_codec_conf;

	return;

err_alloc_dai_link:
	tegra_machine_remove_dai_link();
	tegra_machine_remove_codec_conf();
	return;
}

static int tegra186_driver_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &snd_soc_tegra_t186;
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_pcm_runtime *rtd;
	struct device *dev = &pdev->dev;
	struct tegra_t186 *machine;
	int ret = 0, i;

	if (!np) {
		dev_err(dev, "No device tree node for t186 driver");
		return -ENODEV;
	}

	machine = devm_kzalloc(dev, sizeof(*machine), GFP_KERNEL);
	if (!machine) {
		dev_err(dev, "Can't allocate t186 struct\n");
		ret = -ENOMEM;
		goto err;
	}

	card->dev = dev;
	platform_set_drvdata(pdev, card);
	snd_soc_card_set_drvdata(card, machine);
	machine->audio_clock.clk_cdev1_state = 0;
	card->dapm.idle_bias_off = true;

	ret = snd_soc_of_parse_card_name(card, "nvidia,model");
	if (ret)
		goto err;

	ret = snd_soc_of_parse_audio_routing(card,
				"nvidia,audio-routing");
	if (ret)
		goto err;

	if (of_property_read_u32(np, "nvidia,num-clk",
			       &machine->audio_clock.num_clk) < 0) {
		dev_err(dev, "Missing property nvidia,num-clk\n");
		ret = -ENODEV;
		goto err;
	}

	if (of_property_read_u32_array(np, "nvidia,clk-rates",
				(u32 *)&machine->audio_clock.clk_rates,
				machine->audio_clock.num_clk) < 0) {
		dev_err(dev, "Missing property nvidia,clk-rates\n");
		ret = -ENODEV;
		goto err;
	}

	tegra_machine_dma_set_mask(pdev);

	dai_link_setup(pdev);

	machine->pcard = card;

	ret = tegra_alt_asoc_utils_init(&machine->audio_clock, dev, card);
	if (ret)
		goto err_alloc_dai_link;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(dev, "snd_soc_register_card failed (%d)\n", ret);
		goto err_alloc_dai_link;
	}

	for (i = 0; i < ARRAY_SIZE(dai_link_names); i++) {
		rtd = snd_soc_get_pcm_runtime(card, dai_link_names[i]);
		if (!rtd) {
			dev_warn(card->dev, "failed to get pcm runtime: %s\n",
				 dai_link_names[i]);
			machine->dai_link_probed[i] = false;
			continue;
		}

		machine->dai_link_probed[i] = true;
		dev_info(dev, "%s <-> %s (%s) mapping ok\n",
			 dev_name(rtd->cpu_dai->dev),
			 rtd->codec_dai->name,
			 dev_name(rtd->codec->dev));

		if (i == RT5658_INDEX)
			rt5659_set_jack_detect(rtd->codec, NULL);
	}

	return 0;

err_alloc_dai_link:
	tegra_machine_remove_dai_link();
	tegra_machine_remove_codec_conf();
err:
	return ret;
}

static int tegra186_driver_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	tegra_machine_remove_dai_link();
	tegra_machine_remove_codec_conf();

	return 0;
}

static const struct of_device_id tegra186_of_match[] = {
	{ .compatible = "nvidia,tegra186-p4573-audio", },
	{},
};

static struct platform_driver tegra186_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = tegra186_of_match,
	},
	.probe = tegra186_driver_probe,
	.remove = tegra186_driver_remove,
};
module_platform_driver(tegra186_driver);

MODULE_DESCRIPTION("Nvidia Tegra186+P4573 ASoC Sound Card driver");
MODULE_AUTHOR("Nicolin Chen <nicolinc@nvidia.com>");
MODULE_DEVICE_TABLE(of, tegra186_of_match);
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_LICENSE("GPL");
