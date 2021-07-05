/*
 * tegra186_dspk_alt.c - Tegra186 DSPK driver
 *
 * Copyright (c) 2015-2019 NVIDIA CORPORATION.  All rights reserved.
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
#include <soc/tegra/chip-id.h>
#include <linux/pinctrl/pinconf-tegra.h>

#include "tegra210_xbar_alt.h"
#include "tegra186_dspk_alt.h"
#include "ahub_unit_fpga_clock.h"

#define DRV_NAME "tegra186-dspk"

static const struct reg_default tegra186_dspk_reg_defaults[] = {
	{ TEGRA186_DSPK_AXBAR_RX_INT_MASK, 0x00000007},
	{ TEGRA186_DSPK_AXBAR_RX_CIF_CTRL, 0x00007700},
	{ TEGRA186_DSPK_CG,                0x00000001},
	{ TEGRA186_DSPK_CORE_CTRL,         0x00000310},
	{ TEGRA186_DSPK_CODEC_CTRL,        0x03000000},
	{ TEGRA186_DSPK_SDM_COEF_A_2,      0x000013bb},
	{ TEGRA186_DSPK_SDM_COEF_A_3,      0x00001cbf},
	{ TEGRA186_DSPK_SDM_COEF_A_4,      0x000029d7},
	{ TEGRA186_DSPK_SDM_COEF_A_5,      0x00003782},
	{ TEGRA186_DSPK_SDM_COEF_C_1,      0x000000a6},
	{ TEGRA186_DSPK_SDM_COEF_C_2,      0x00001959},
	{ TEGRA186_DSPK_SDM_COEF_C_3,      0x00002b9f},
	{ TEGRA186_DSPK_SDM_COEF_C_4,      0x00004218},
	{ TEGRA186_DSPK_SDM_COEF_G_1,      0x00000074},
	{ TEGRA186_DSPK_SDM_COEF_G_2,      0x0000007d},
};

static int tegra186_dspk_get_control(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tegra186_dspk *dspk = snd_soc_codec_get_drvdata(codec);

	if (strstr(kcontrol->id.name, "Rx fifo threshold"))
		ucontrol->value.integer.value[0] = dspk->rx_fifo_th;
	else if (strstr(kcontrol->id.name, "OSR Value"))
		ucontrol->value.integer.value[0] = dspk->osr_val;

	return 0;
}

static int tegra186_dspk_put_control(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tegra186_dspk *dspk = snd_soc_codec_get_drvdata(codec);
	int val = ucontrol->value.integer.value[0];

	if (strstr(kcontrol->id.name, "Rx fifo threshold")) {
		if (val >= 0 && val < TEGRA186_DSPK_RX_FIFO_DEPTH)
			dspk->rx_fifo_th = val;
		else
			return -EINVAL;
	} else if (strstr(kcontrol->id.name, "OSR Value"))
		dspk->osr_val = val;

	return 0;
}

static int tegra186_dspk_runtime_suspend(struct device *dev)
{
	struct tegra186_dspk *dspk = dev_get_drvdata(dev);

	regcache_cache_only(dspk->regmap, true);
	regcache_mark_dirty(dspk->regmap);

	if (!(tegra_platform_is_unit_fpga() || tegra_platform_is_fpga()))
		clk_disable_unprepare(dspk->clk_dspk);

	return 0;
}

static int tegra186_dspk_runtime_resume(struct device *dev)
{
	struct tegra186_dspk *dspk = dev_get_drvdata(dev);
	int ret;

	if (!(tegra_platform_is_unit_fpga() || tegra_platform_is_fpga())) {
		ret = clk_prepare_enable(dspk->clk_dspk);
		if (ret) {
			dev_err(dev, "clk_enable failed: %d\n", ret);
			return ret;
		}
	}

	regcache_cache_only(dspk->regmap, false);
	regcache_sync(dspk->regmap);

	return 0;
}

static int tegra186_dspk_set_audio_cif(struct tegra186_dspk *dspk,
		struct snd_pcm_hw_params *params,
		unsigned int reg, struct snd_soc_dai *dai)
{
	int channels, max_th;
	struct tegra210_xbar_cif_conf cif_conf;
	struct device *dev = dai->dev;

	channels = params_channels(params);
	memset(&cif_conf, 0, sizeof(struct tegra210_xbar_cif_conf));
	cif_conf.audio_channels = channels;
	cif_conf.client_channels = channels;
	cif_conf.client_bits = TEGRA210_AUDIOCIF_BITS_24;

	/* RX FIFO threshold interms of frames */
	max_th = (TEGRA186_DSPK_RX_FIFO_DEPTH / channels) - 1;
	max_th = (max_th < 0) ? 0 : max_th;
	if (dspk->rx_fifo_th > max_th) { /* error handling */
		cif_conf.threshold = max_th;
		dspk->rx_fifo_th = max_th;
	} else
		cif_conf.threshold = dspk->rx_fifo_th;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		cif_conf.audio_bits = TEGRA210_AUDIOCIF_BITS_16;
		cif_conf.client_bits = TEGRA210_AUDIOCIF_BITS_16;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		cif_conf.audio_bits = TEGRA210_AUDIOCIF_BITS_32;
		break;
	default:
		dev_err(dev, "Wrong format!\n");
		return -EINVAL;
	}

	tegra210_xbar_set_cif(dspk->regmap, TEGRA186_DSPK_AXBAR_RX_CIF_CTRL,
			      &cif_conf);
	return 0;
}

static int tegra186_dspk_startup(struct snd_pcm_substream *substream,
					struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	struct tegra186_dspk *dspk = snd_soc_dai_get_drvdata(dai);
	int ret;

	if (dspk->prod_name != NULL) {
		ret = tegra_pinctrl_config_prod(dev, dspk->prod_name);
		if (ret < 0)
			dev_warn(dev, "Failed to set %s setting\n",
					dspk->prod_name);
	}

	return 0;
}

static int tegra186_dspk_hw_params(struct snd_pcm_substream *substream,
		    struct snd_pcm_hw_params *params,
		    struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	struct tegra186_dspk *dspk = snd_soc_dai_get_drvdata(dai);
	int channels, srate, ret, dspk_clk;
	int osr = dspk->osr_val;
	int interface_clk_ratio = 4; /* dspk interface clock should be fsout*4 */

	channels = params_channels(params);
	srate = params_rate(params);
	dspk_clk = (1 << (5+osr)) * srate * interface_clk_ratio;

	if ((tegra_platform_is_unit_fpga() || tegra_platform_is_fpga())) {
		program_dspk_clk(dspk_clk);
	} else {
		ret = clk_set_rate(dspk->clk_dspk, dspk_clk);
		if (ret) {
			dev_err(dev, "Can't set dspk clock rate: %d\n", ret);
			return ret;
		}
	}

	regmap_update_bits(dspk->regmap,
			TEGRA186_DSPK_CORE_CTRL,
			TEGRA186_DSPK_OSR_MASK,
			osr << TEGRA186_DSPK_OSR_SHIFT);

	regmap_update_bits(dspk->regmap,
			TEGRA186_DSPK_CORE_CTRL,
			TEGRA186_DSPK_CHANNEL_SELECT_MASK,
			((1 << channels) - 1) <<
			TEGRA186_DSPK_CHANNEL_SELECT_SHIFT);

	/* program cif control register */
	ret = tegra186_dspk_set_audio_cif(dspk, params,
				TEGRA186_DSPK_AXBAR_RX_CIF_CTRL,
				dai);

	if (ret)
		dev_err(dev, "Can't set dspk RX CIF: %d\n", ret);
	return ret;
}

static struct snd_soc_dai_ops tegra186_dspk_dai_ops = {
	.hw_params	= tegra186_dspk_hw_params,
	.startup	= tegra186_dspk_startup,
};

static struct snd_soc_dai_driver tegra186_dspk_dais[] = {
	{
	    .name = "DAP",
	    .capture = {
		.stream_name = "DAP Transmit",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	    },
	    .ops = &tegra186_dspk_dai_ops,
	    .symmetric_rates = 1,
	},
	{
	    .name = "CIF",
	    .playback = {
		.stream_name = "CIF Receive",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	    },
	},
	/* The second DAI is used when the output of the DSPK is connected
	 * to two mono codecs. When the output of the DSPK is connected to
	 * a single stereo codec, then only the first DAI should be used.
	 */
	{
	    .name = "DAP2",
	    .capture = {
		.stream_name = "DAP2 Transmit",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	    },
	    .symmetric_rates = 1,
	},
	{
	    .name = "CIF2",
	    .playback = {
		.stream_name = "CIF2 Receive",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	    },
	}
};

static const struct snd_soc_dapm_widget tegra186_dspk_widgets[] = {
	SND_SOC_DAPM_AIF_OUT("DAP TX", NULL, 0, TEGRA186_DSPK_ENABLE, 0, 0),
	SND_SOC_DAPM_AIF_OUT("DAP2 TX", NULL, 0, 0, 0, 0),
};

static const struct snd_soc_dapm_route tegra186_dspk_routes[] = {
	{ "DAP TX",	   NULL, "CIF Receive" },
	{ "DAP Transmit", NULL, "DAP TX" },
	{ "DAP2 TX", NULL, "CIF2 Receive" },
	{ "DAP2 Transmit", NULL, "DAP2 TX" },
};

static const char * const tegra186_dspk_osr_text[] = {
	"OSR_32", "OSR_64", "OSR_128", "OSR_256",
};

static const struct soc_enum tegra186_dspk_osr_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0,
		ARRAY_SIZE(tegra186_dspk_osr_text),
		tegra186_dspk_osr_text);

#define NV_SOC_SINGLE_RANGE_EXT(xname, xmin, xmax, xget, xput) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.info = snd_soc_info_xr_sx, .get = xget, .put = xput, \
	.private_value = (unsigned long)&(struct soc_mixer_control) \
	{.invert = 0, .min = xmin, .max = xmax, \
		.platform_max = xmax} \
}

static const struct snd_kcontrol_new tegrat186_dspk_controls[] = {
	NV_SOC_SINGLE_RANGE_EXT("Rx fifo threshold", 0,
		TEGRA186_DSPK_RX_FIFO_DEPTH - 1, tegra186_dspk_get_control,
		tegra186_dspk_put_control),
	SOC_ENUM_EXT("OSR Value", tegra186_dspk_osr_enum,
		tegra186_dspk_get_control, tegra186_dspk_put_control),
};

static struct snd_soc_codec_driver tegra186_dspk_codec = {
	.idle_bias_off = 1,
	.component_driver = {
		.dapm_widgets = tegra186_dspk_widgets,
		.num_dapm_widgets = ARRAY_SIZE(tegra186_dspk_widgets),
		.dapm_routes = tegra186_dspk_routes,
		.num_dapm_routes = ARRAY_SIZE(tegra186_dspk_routes),
		.controls = tegrat186_dspk_controls,
		.num_controls = ARRAY_SIZE(tegrat186_dspk_controls),
	},
};

/* Regmap callback functions */
static bool tegra186_dspk_wr_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA186_DSPK_AXBAR_RX_INT_MASK:
	case TEGRA186_DSPK_AXBAR_RX_INT_SET:
	case TEGRA186_DSPK_AXBAR_RX_INT_CLEAR:
	case TEGRA186_DSPK_AXBAR_RX_CIF_CTRL:
	case TEGRA186_DSPK_AXBAR_RX_CYA:
	case TEGRA186_DSPK_ENABLE:
	case TEGRA186_DSPK_SOFT_RESET:
	case TEGRA186_DSPK_CG:
		return true;
	default:
		if (((reg % 4) == 0) && (reg >= TEGRA186_DSPK_CORE_CTRL) &&
		    (reg <= TEGRA186_DSPK_SDM_COEF_G_2))
			return true;
		else
			return false;
	};
}

static bool tegra186_dspk_rd_reg(struct device *dev, unsigned int reg)
{
	if (tegra186_dspk_wr_reg(dev, reg))
		return true;

	switch (reg) {
	case TEGRA186_DSPK_AXBAR_RX_STATUS:
	case TEGRA186_DSPK_AXBAR_RX_INT_STATUS:
	case TEGRA186_DSPK_AXBAR_RX_CIF_FIFO_STATUS:
	case TEGRA186_DSPK_STATUS:
	case TEGRA186_DSPK_INT_STATUS:
		return true;
	default:
		if (((reg % 4) == 0) && (reg >= TEGRA186_DSPK_DEBUG_STATUS) &&
		    (reg <= TEGRA186_DSPK_DEBUG_STAGE4_CNTR))
			return true;
		else
			return false;
	};
}

static bool tegra186_dspk_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA186_DSPK_AXBAR_RX_STATUS:
	case TEGRA186_DSPK_AXBAR_RX_INT_STATUS:
	case TEGRA186_DSPK_AXBAR_RX_CIF_FIFO_STATUS:
	case TEGRA186_DSPK_STATUS:
	case TEGRA186_DSPK_INT_STATUS:
		return true;
	default:
		return false;
	};
}

static const struct regmap_config tegra186_dspk_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = TEGRA186_DSPK_DEBUG_STAGE4_CNTR,
	.writeable_reg = tegra186_dspk_wr_reg,
	.readable_reg = tegra186_dspk_rd_reg,
	.volatile_reg = tegra186_dspk_volatile_reg,
	.precious_reg = NULL,
	.reg_defaults = tegra186_dspk_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(tegra186_dspk_reg_defaults),
	.cache_type = REGCACHE_FLAT,
};

static const struct of_device_id tegra186_dspk_of_match[] = {
	{ .compatible = "nvidia,tegra186-dspk" },
	{},
};

static int tegra186_dspk_platform_probe(struct platform_device *pdev)
{
	struct tegra186_dspk *dspk;
	struct device_node *np = pdev->dev.of_node;
	struct resource *mem;
	void __iomem *regs;
	int ret = 0;
	const struct of_device_id *match;

	match = of_match_device(tegra186_dspk_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "Error: No device match found\n");
		return -ENODEV;
	}

	dspk = devm_kzalloc(&pdev->dev, sizeof(*dspk), GFP_KERNEL);
	if (!dspk)
		return -ENOMEM;

	dspk->prod_name = NULL;
	dspk->rx_fifo_th = 0;
	dspk->osr_val = TEGRA186_DSPK_OSR_64;
	dev_set_drvdata(&pdev->dev, dspk);

	if (!(tegra_platform_is_unit_fpga() || tegra_platform_is_fpga())) {
		dspk->clk_dspk = devm_clk_get(&pdev->dev, "dspk");
		if (IS_ERR(dspk->clk_dspk)) {
			dev_err(&pdev->dev, "Can't retrieve dspk clock\n");
			return PTR_ERR(dspk->clk_dspk);
		}
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(regs))
		return PTR_ERR(regs);
	dspk->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
					     &tegra186_dspk_regmap_config);
	if (IS_ERR(dspk->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		return PTR_ERR(dspk->regmap);
	}
	regcache_cache_only(dspk->regmap, true);

	ret = of_property_read_u32(np, "nvidia,ahub-dspk-id",
				   &pdev->dev.id);
	if (ret < 0) {
		dev_err(&pdev->dev, "Missing property nvidia,ahub-dspk-id\n");
		return ret;
	}

	pm_runtime_enable(&pdev->dev);
	ret = snd_soc_register_codec(&pdev->dev, &tegra186_dspk_codec,
				     tegra186_dspk_dais,
				     ARRAY_SIZE(tegra186_dspk_dais));
	if (ret != 0) {
		dev_err(&pdev->dev, "Could not register CODEC: %d\n", ret);
		pm_runtime_disable(&pdev->dev);
		return ret;
	}

	if (of_property_read_string(np, "prod-name", &dspk->prod_name) == 0) {
		ret = tegra_pinctrl_config_prod(&pdev->dev, dspk->prod_name);
		if (ret < 0)
			dev_warn(&pdev->dev, "Failed to set %s setting\n",
				 dspk->prod_name);
	}

	return 0;
}

static int tegra186_dspk_platform_remove(struct platform_device *pdev)
{
	struct tegra186_dspk *dspk;

	dspk = dev_get_drvdata(&pdev->dev);
	snd_soc_unregister_codec(&pdev->dev);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra186_dspk_runtime_suspend(&pdev->dev);

	return 0;
}

static const struct dev_pm_ops tegra186_dspk_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra186_dspk_runtime_suspend,
				tegra186_dspk_runtime_resume, NULL)
	SET_LATE_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				     pm_runtime_force_resume)
};

static struct platform_driver tegra186_dspk_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra186_dspk_of_match,
		.pm = &tegra186_dspk_pm_ops,
	},
	.probe = tegra186_dspk_platform_probe,
	.remove = tegra186_dspk_platform_remove,
};
module_platform_driver(tegra186_dspk_driver);


MODULE_AUTHOR("Mohan Kumar <mkumard@nvidia.com>");
MODULE_DESCRIPTION("Tegra186 DSPK ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, tegra186_dspk_of_match);
