/*
 * tegra210_afc_alt.c - Tegra210 AFC driver
 *
 * Copyright (c) 2014-2019 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/of_device.h>

#include "tegra210_xbar_alt.h"
#include "tegra210_afc_alt.h"

#define DRV_NAME "tegra210-afc"

static const struct reg_default tegra210_afc_reg_defaults[] = {
	{ TEGRA210_AFC_AXBAR_RX_CIF_CTRL, 0x00007700},
	{ TEGRA210_AFC_AXBAR_TX_INT_MASK, 0x00000001},
	{ TEGRA210_AFC_AXBAR_TX_CIF_CTRL, 0x00007000},
	{ TEGRA210_AFC_CG, 0x1},
	{ TEGRA210_AFC_INT_MASK, 0x1},
	{ TEGRA210_AFC_DEST_I2S_PARAMS, 0x01190e0c },
	{ TEGRA210_AFC_TXCIF_FIFO_PARAMS, 0x00190e0c },
	{ TEGRA210_AFC_CLK_PPM_DIFF, 0x0000001e},
	{ TEGRA210_AFC_LCOEF_1_4_0, 0x0000002e},
	{ TEGRA210_AFC_LCOEF_1_4_1, 0x0000f9e6},
	{ TEGRA210_AFC_LCOEF_1_4_2, 0x000020ca},
	{ TEGRA210_AFC_LCOEF_1_4_3, 0x00007147},
	{ TEGRA210_AFC_LCOEF_1_4_4, 0x0000f17e},
	{ TEGRA210_AFC_LCOEF_1_4_5, 0x000001e0},
	{ TEGRA210_AFC_LCOEF_2_4_0, 0x00000117},
	{ TEGRA210_AFC_LCOEF_2_4_1, 0x0000f26b},
	{ TEGRA210_AFC_LCOEF_2_4_2, 0x00004c07},
};

static void tegra210_afc_init(struct tegra210_afc *afc)
{
	afc->ppm_diff = AFC_CLK_PPM_DIFF;
	afc->threshold_type = TH_DEFAULT;
	afc->src_burst = 0;
	afc->start_threshold = 0;
	afc->dest_module_num = 0;
}

static int tegra210_afc_runtime_suspend(struct device *dev)
{
	struct tegra210_afc *afc = dev_get_drvdata(dev);

	regcache_cache_only(afc->regmap, true);
	regcache_mark_dirty(afc->regmap);

	return 0;
}

static int tegra210_afc_runtime_resume(struct device *dev)
{
	struct tegra210_afc *afc = dev_get_drvdata(dev);

	regcache_cache_only(afc->regmap, false);
	regcache_sync(afc->regmap);

	return 0;
}

static int tegra210_afc_controls_get(struct snd_kcontrol *kctl,
	struct snd_ctl_elem_value *uctl)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kctl);
	struct tegra210_afc *afc = snd_soc_codec_get_drvdata(codec);

	if (strstr(kctl->id.name, "ppm diff"))
		uctl->value.integer.value[0] = afc->ppm_diff;
	else if (strstr(kctl->id.name, "src burst"))
		uctl->value.integer.value[0] = afc->src_burst;
	else if (strstr(kctl->id.name, "start threshold"))
		uctl->value.integer.value[0] = afc->start_threshold;
	else if (strstr(kctl->id.name, "threshold type"))
		uctl->value.integer.value[0] = afc->threshold_type;
	else if (strstr(kctl->id.name, "dest module name"))
		uctl->value.integer.value[0] = afc->dest_module_num;

	return 0;
}

static int tegra210_afc_controls_put(struct snd_kcontrol *kctl,
	struct snd_ctl_elem_value *uctl)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kctl);
	struct tegra210_afc *afc = snd_soc_codec_get_drvdata(codec);
	int value = uctl->value.integer.value[0];

	if (strstr(kctl->id.name, "ppm diff")) {
		if (value >= 0 && value <= 100)
			afc->ppm_diff = value;
		else
			return -EINVAL;
	} else if (strstr(kctl->id.name, "src burst")) {
		if (value >= 0 && value <= 24)
			afc->src_burst = value;
		else
			return -EINVAL;
	} else if (strstr(kctl->id.name, "start threshold")) {
		if (value >= 0 && value <= 63)
			afc->start_threshold = value;
		else
			return -EINVAL;
	} else if (strstr(kctl->id.name, "dest module name"))
		afc->dest_module_num = value;
	else if (strstr(kctl->id.name, "threshold type"))
		afc->threshold_type = value;

	return 0;
}

static const char *const tegra210_afc_threshold_type_text[] = {
	"None", "NO-SFC", "SFC", "SFC-AMX",
};

static const char *const tegra186_afc_dst_mod_type_text[] = {
	"None", "I2S1", "I2S2", "I2S3", "I2S4", "I2S5",
	"I2S6", "DSPK1", "DSPK2",
};

static const char *const tegra210_afc_dst_mod_type_text[] = {
	"None", "I2S1", "I2S2", "I2S3", "I2S4", "I2S5",
};

static const struct soc_enum tegra210_afc_threshold_config_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tegra210_afc_threshold_type_text),
		tegra210_afc_threshold_type_text);

static const struct soc_enum tegra210_afc_dst_mod_type_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tegra210_afc_dst_mod_type_text),
		tegra210_afc_dst_mod_type_text);

static const struct soc_enum tegra186_afc_dst_mod_type_enum =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tegra186_afc_dst_mod_type_text),
		tegra186_afc_dst_mod_type_text);

#define NV_SOC_SINGLE_RANGE_EXT(xname, xmin, xmax, xget, xput) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.info = snd_soc_info_xr_sx, .get = xget, .put = xput, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
		{.invert = 0, .min = xmin, .max = xmax, \
		.platform_max = xmax}}

static const struct snd_kcontrol_new tegra210_afc_controls[] = {
	NV_SOC_SINGLE_RANGE_EXT("ppm diff", 0, 100, tegra210_afc_controls_get,
		tegra210_afc_controls_put),
	NV_SOC_SINGLE_RANGE_EXT("src burst", 0, 24, tegra210_afc_controls_get,
		tegra210_afc_controls_put),
	NV_SOC_SINGLE_RANGE_EXT("start threshold", 0, 63,
		tegra210_afc_controls_get, tegra210_afc_controls_put),
	SOC_ENUM_EXT("threshold type", tegra210_afc_threshold_config_enum,
		tegra210_afc_controls_get, tegra210_afc_controls_put),
	SOC_ENUM_EXT("dest module name", tegra210_afc_dst_mod_type_enum,
		tegra210_afc_controls_get, tegra210_afc_controls_put),
};

static const struct snd_kcontrol_new tegra186_afc_controls[] = {
	NV_SOC_SINGLE_RANGE_EXT("ppm diff", 0, 100, tegra210_afc_controls_get,
		tegra210_afc_controls_put),
	NV_SOC_SINGLE_RANGE_EXT("src burst", 0, 24, tegra210_afc_controls_get,
		tegra210_afc_controls_put),
	NV_SOC_SINGLE_RANGE_EXT("start threshold", 0, 63,
		tegra210_afc_controls_get, tegra210_afc_controls_put),
	SOC_ENUM_EXT("threshold type", tegra210_afc_threshold_config_enum,
		tegra210_afc_controls_get, tegra210_afc_controls_put),
	SOC_ENUM_EXT("dest module name", tegra186_afc_dst_mod_type_enum,
		tegra210_afc_controls_get, tegra210_afc_controls_put),
};

static int tegra210_afc_set_thresholds(struct tegra210_afc *afc,
				unsigned int afc_id)
{
	unsigned int val_afc, val_dst_mod, dest_module, dest_module_id;

	if (afc->dest_module_num == 0) {
		pr_err("destination module for afc not selected\n");
		return -EINVAL;
	}

	if (afc->dest_module_num > afc->soc_data->num_i2s) {
		dest_module = 1;
		dest_module_id = afc->dest_module_num - afc->soc_data->num_i2s;
	} else {
		dest_module = 0;
		dest_module_id = afc->dest_module_num;
	}

	switch (afc->threshold_type) {
	case TH_NON_SFC:
		if (afc->start_threshold == 0) {
			pr_err("case %s: threshold not defined\n",
				tegra210_afc_threshold_type_text[TH_NON_SFC]);
			goto err_exit;
		}
		val_afc = (afc->start_threshold + 1) <<
			TEGRA210_AFC_FIFO_HIGH_THRESHOLD_SHIFT;
		val_afc |= afc->start_threshold <<
			TEGRA210_AFC_FIFO_START_THRESHOLD_SHIFT;
		val_afc |= afc->start_threshold - 1;

		val_dst_mod = val_afc;
		break;

	/* use src_burst when SFC is in the path*/
	case TH_SFC:
		if (afc->src_burst == 0) {
			pr_err("case %s: src_burst not defined\n",
				tegra210_afc_threshold_type_text[TH_SFC]);
			goto err_exit;
		}
		val_afc = (afc->src_burst + 1) <<
			TEGRA210_AFC_FIFO_HIGH_THRESHOLD_SHIFT;
		val_afc |= (afc->src_burst + 1) <<
			TEGRA210_AFC_FIFO_START_THRESHOLD_SHIFT;
		val_afc |= afc->src_burst + 1;

		val_dst_mod = ((afc->src_burst << 1) + 1) <<
			TEGRA210_AFC_FIFO_HIGH_THRESHOLD_SHIFT;
		val_dst_mod |= (afc->src_burst + 2) <<
			TEGRA210_AFC_FIFO_START_THRESHOLD_SHIFT;
		val_dst_mod |= afc->src_burst;
		break;

	case TH_SFC_AMX:
		if (afc->src_burst == 0) {
			pr_err("case %s: src_burst not defined\n",
				tegra210_afc_threshold_type_text[TH_SFC_AMX]);
			goto err_exit;
		}
		val_afc = ((afc->src_burst << 1) + 4) <<
			TEGRA210_AFC_FIFO_HIGH_THRESHOLD_SHIFT;
		val_afc |= (afc->src_burst + 4) <<
			TEGRA210_AFC_FIFO_START_THRESHOLD_SHIFT;
		val_afc |= 4;

		val_dst_mod = ((afc->src_burst << 1) + 1) <<
			TEGRA210_AFC_FIFO_HIGH_THRESHOLD_SHIFT;
		val_dst_mod |= (afc->src_burst + 2) <<
			TEGRA210_AFC_FIFO_START_THRESHOLD_SHIFT;
		val_dst_mod |= afc->src_burst;
		break;

	/* default threshold settings */
	case TH_DEFAULT:
		val_afc = 4 << TEGRA210_AFC_FIFO_HIGH_THRESHOLD_SHIFT;
		val_afc |= 3 << TEGRA210_AFC_FIFO_START_THRESHOLD_SHIFT;
		val_afc |= 2;

		val_dst_mod = val_afc;
		break;

	default:
		pr_err("unsupported threshold type\n");
		goto err_exit;
	}
	regmap_write(afc->regmap, TEGRA210_AFC_TXCIF_FIFO_PARAMS, val_afc);

	if (afc->soc_data->flag_module_select)
		val_dst_mod |= dest_module  << 	TEGRA186_AFC_MODULE_SELECT_SHIFT;

	val_dst_mod |= dest_module_id << TEGRA210_AFC_DEST_MODULE_ID_SHIFT;
	regmap_write(afc->regmap, TEGRA210_AFC_DEST_I2S_PARAMS, val_dst_mod);

	return 0;

err_exit:
	return -EINVAL;
}

static int tegra210_afc_set_audio_cif(struct tegra210_afc *afc,
				struct snd_pcm_hw_params *params,
				unsigned int reg)
{
	int channels, audio_bits;
	struct tegra210_xbar_cif_conf cif_conf;

	memset(&cif_conf, 0, sizeof(struct tegra210_xbar_cif_conf));

	channels = params_channels(params);
	if (channels < 2)
		return -EINVAL;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		audio_bits = TEGRA210_AUDIOCIF_BITS_16;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		audio_bits = TEGRA210_AUDIOCIF_BITS_32;
		break;
	default:
		return -EINVAL;
	}

	cif_conf.audio_channels = channels;
	cif_conf.client_channels = channels;
	cif_conf.audio_bits = audio_bits;
	cif_conf.client_bits = audio_bits;

	tegra210_xbar_set_cif(afc->regmap, reg, &cif_conf);

	return 0;
}

static int tegra210_afc_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	struct tegra210_afc *afc = snd_soc_dai_get_drvdata(dai);
	int ret;

	/* set RX cif and TX cif */
	ret = tegra210_afc_set_audio_cif(afc, params,
				TEGRA210_AFC_AXBAR_RX_CIF_CTRL);
	if (ret) {
		dev_err(dev, "Can't set AFC RX CIF: %d\n", ret);
		return ret;
	}
	ret = tegra210_afc_set_audio_cif(afc, params,
				TEGRA210_AFC_AXBAR_TX_CIF_CTRL);
	if (ret) {
		dev_err(dev, "Can't set AFC TX CIF: %d\n", ret);
		return ret;
	}

	/* update expected ppm difference */
	regmap_update_bits(afc->regmap,
		TEGRA210_AFC_CLK_PPM_DIFF, 0xFFFF, afc->ppm_diff);

	/* program thresholds, dest module depending on the mode*/
	if (tegra210_afc_set_thresholds(afc, dev->id) == -EINVAL)
		dev_err(dev, "Can't set AFC threshold: %d\n", ret);

	return ret;

}

static struct snd_soc_dai_ops tegra210_afc_dai_ops = {
	.hw_params	= tegra210_afc_hw_params,
};

static struct snd_soc_dai_driver tegra210_afc_dais[] = {
	{
		.name = "AFC IN",
		.playback = {
			.stream_name = "AFC Receive",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
	},
	{
		.name = "AFC OUT",
		.capture = {
			.stream_name = "AFC Transmit",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_96000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &tegra210_afc_dai_ops,
	}
};

static const struct snd_soc_dapm_widget tegra210_afc_widgets[] = {
	SND_SOC_DAPM_AIF_IN("AFC RX", NULL, 0, SND_SOC_NOPM,
				0, 0),
	SND_SOC_DAPM_AIF_OUT("AFC TX", NULL, 0, TEGRA210_AFC_ENABLE,
				TEGRA210_AFC_EN_SHIFT, 0),
};

static const struct snd_soc_dapm_route tegra210_afc_routes[] = {
	{ "AFC RX",       NULL, "AFC Receive" },
	{ "AFC TX",       NULL, "AFC RX" },
	{ "AFC Transmit", NULL, "AFC TX" },
};

static const struct snd_soc_codec_driver tegra210_afc_codec = {
	.idle_bias_off = 1,
	.component_driver = {
		.dapm_widgets = tegra210_afc_widgets,
		.num_dapm_widgets = ARRAY_SIZE(tegra210_afc_widgets),
		.dapm_routes = tegra210_afc_routes,
		.num_dapm_routes = ARRAY_SIZE(tegra210_afc_routes),
	},
};

static const struct snd_soc_codec_driver tegra186_afc_codec = {
	.idle_bias_off = 1,
	.component_driver = {
		.dapm_widgets = tegra210_afc_widgets,
		.num_dapm_widgets = ARRAY_SIZE(tegra210_afc_widgets),
		.dapm_routes = tegra210_afc_routes,
		.num_dapm_routes = ARRAY_SIZE(tegra210_afc_routes),
		.controls = tegra186_afc_controls,
		.num_controls = ARRAY_SIZE(tegra186_afc_controls),
	},
};

static bool tegra210_afc_wr_rd_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_AFC_AXBAR_RX_STATUS:
	case TEGRA210_AFC_AXBAR_RX_CIF_CTRL:
	case TEGRA210_AFC_AXBAR_RX_CYA:
	case TEGRA210_AFC_AXBAR_TX_STATUS:
	case TEGRA210_AFC_AXBAR_TX_INT_STATUS:
	case TEGRA210_AFC_AXBAR_TX_INT_MASK:
	case TEGRA210_AFC_AXBAR_TX_INT_SET:
	case TEGRA210_AFC_AXBAR_TX_INT_CLEAR:
	case TEGRA210_AFC_AXBAR_TX_CIF_CTRL:
	case TEGRA210_AFC_AXBAR_TX_CYA:
	case TEGRA210_AFC_ENABLE:
	case TEGRA210_AFC_SOFT_RESET:
	case TEGRA210_AFC_CG:
	case TEGRA210_AFC_STATUS:
	case TEGRA210_AFC_INT_STATUS:
	case TEGRA210_AFC_INT_MASK:
	case TEGRA210_AFC_INT_SET:
	case TEGRA210_AFC_INT_CLEAR:
	case TEGRA210_AFC_DEST_I2S_PARAMS:
	case TEGRA210_AFC_TXCIF_FIFO_PARAMS:
	case TEGRA210_AFC_CLK_PPM_DIFF:
	case TEGRA210_AFC_DBG_CTRL:
	case TEGRA210_AFC_TOTAL_SAMPLES:
	case TEGRA210_AFC_DECIMATION_SAMPLES:
	case TEGRA210_AFC_INTERPOLATION_SAMPLES:
	case TEGRA210_AFC_DBG_INTERNAL:
	case TEGRA210_AFC_LCOEF_1_4_0:
	case TEGRA210_AFC_LCOEF_1_4_1:
	case TEGRA210_AFC_LCOEF_1_4_2:
	case TEGRA210_AFC_LCOEF_1_4_3:
	case TEGRA210_AFC_LCOEF_1_4_4:
	case TEGRA210_AFC_LCOEF_1_4_5:
	case TEGRA210_AFC_LCOEF_2_4_0:
	case TEGRA210_AFC_LCOEF_2_4_1:
	case TEGRA210_AFC_LCOEF_2_4_2:
	case TEGRA210_AFC_CYA:
		return true;
	default:
		return false;
	};
}

static bool tegra210_afc_volatile_reg(struct device *dev, unsigned int reg)
{
	return false;
}

static const struct regmap_config tegra210_afc_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = TEGRA210_AFC_CYA,
	.writeable_reg = tegra210_afc_wr_rd_reg,
	.readable_reg = tegra210_afc_wr_rd_reg,
	.volatile_reg = tegra210_afc_volatile_reg,
	.reg_defaults = tegra210_afc_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(tegra210_afc_reg_defaults),
	.cache_type = REGCACHE_FLAT,
};

static const struct tegra210_afc_soc_data soc_data_tegra210 = {
	.afc_codec = &tegra210_afc_codec,
	.num_i2s = 5,
	.flag_module_select = false,
};

static const struct tegra210_afc_soc_data soc_data_tegra186 = {
	.afc_codec = &tegra186_afc_codec,
	.num_i2s = 6,
	.flag_module_select = true,
};

static const struct of_device_id tegra210_afc_of_match[] = {
	{ .compatible = "nvidia,tegra210-afc", .data = &soc_data_tegra210 },
	{ .compatible = "nvidia,tegra186-afc", .data = &soc_data_tegra186 },
	{},
};

static int tegra210_afc_platform_probe(struct platform_device *pdev)
{
	struct tegra210_afc *afc;
	struct resource *mem;
	void __iomem *regs;
	int ret = 0;
	const struct of_device_id *match;
	struct tegra210_afc_soc_data *soc_data;

	match = of_match_device(tegra210_afc_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "Error: No device match found\n");
		return -ENODEV;
	}
	soc_data = (struct tegra210_afc_soc_data *)match->data;

	afc = devm_kzalloc(&pdev->dev, sizeof(*afc), GFP_KERNEL);
	if (!afc)
		return -ENOMEM;

	afc->soc_data = soc_data;
	tegra210_afc_init(afc);

	dev_set_drvdata(&pdev->dev, afc);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(regs))
		return PTR_ERR(regs);
	afc->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
					    &tegra210_afc_regmap_config);
	if (IS_ERR(afc->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		return PTR_ERR(afc->regmap);
	}
	regcache_cache_only(afc->regmap, true);

	ret = of_property_read_u32(pdev->dev.of_node,
				   "nvidia,ahub-afc-id",
				   &pdev->dev.id);
	if (ret < 0) {
		dev_err(&pdev->dev, "Missing property nvidia,ahub-afc-id\n");
		return -ret;
	}

	/* Disable SLGC */
	regmap_write(afc->regmap, TEGRA210_AFC_CG, 0);

	pm_runtime_enable(&pdev->dev);
	ret = snd_soc_register_codec(&pdev->dev, afc->soc_data->afc_codec,
				     tegra210_afc_dais,
				     ARRAY_SIZE(tegra210_afc_dais));
	if (ret != 0) {
		dev_err(&pdev->dev, "Could not register CODEC: %d\n", ret);
		pm_runtime_disable(&pdev->dev);
		return ret;
	}

	return 0;
}

static int tegra210_afc_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra210_afc_runtime_suspend(&pdev->dev);

	return 0;
}

static const struct dev_pm_ops tegra210_afc_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra210_afc_runtime_suspend,
			   tegra210_afc_runtime_resume, NULL)
	SET_LATE_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				     pm_runtime_force_resume)
};

static struct platform_driver tegra210_afc_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra210_afc_of_match,
		.pm = &tegra210_afc_pm_ops,
	},
	.probe = tegra210_afc_platform_probe,
	.remove = tegra210_afc_platform_remove,
};
module_platform_driver(tegra210_afc_driver)

MODULE_AUTHOR("Arun Shamanna Lakshmi <aruns@nvidia.com>");
MODULE_DESCRIPTION("Tegra210 AFC ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, tegra210_afc_of_match);
