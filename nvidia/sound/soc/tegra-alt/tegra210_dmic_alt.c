/*
 * tegra210_dmic_alt.c - Tegra210 DMIC driver
 *
 * Copyright (c) 2014-2020 NVIDIA CORPORATION.  All rights reserved.
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
#include <soc/tegra/chip-id.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinconf-tegra.h>
#include <linux/pinctrl/consumer.h>

#include "tegra210_xbar_alt.h"
#include "tegra210_dmic_alt.h"
#include "ahub_unit_fpga_clock.h"

#define DRV_NAME "tegra210-dmic"

static const struct reg_default tegra210_dmic_reg_defaults[] = {
	{ TEGRA210_DMIC_TX_INT_MASK, 0x00000001},
	{ TEGRA210_DMIC_TX_CIF_CTRL, 0x00007700},
	{ TEGRA210_DMIC_CG, 0x1},
	{ TEGRA210_DMIC_CTRL, 0x00000301},
	{ TEGRA210_DMIC_DCR_FILTER_GAIN, 0x00800000},
	{ TEGRA210_DMIC_DCR_BIQUAD_0_COEF_0, 0x00800000},
	{ TEGRA210_DMIC_DCR_BIQUAD_0_COEF_1, 0xff800000},
	{ TEGRA210_DMIC_DCR_BIQUAD_0_COEF_3, 0xff800347},
	{ TEGRA210_DMIC_DCR_BIQUAD_0_COEF_4, 0xffc0ff97},
	{ TEGRA210_DMIC_LP_FILTER_GAIN, 0x004c255a},
	{ TEGRA210_DMIC_LP_BIQUAD_0_COEF_0, 0x00800000},
	{ TEGRA210_DMIC_LP_BIQUAD_0_COEF_1, 0x00ffa74b},
	{ TEGRA210_DMIC_LP_BIQUAD_0_COEF_2, 0x00800000},
	{ TEGRA210_DMIC_LP_BIQUAD_0_COEF_3, 0x009e382a},
	{ TEGRA210_DMIC_LP_BIQUAD_0_COEF_4, 0x00380f38},
	{ TEGRA210_DMIC_LP_BIQUAD_1_COEF_0, 0x00800000},
	{ TEGRA210_DMIC_LP_BIQUAD_1_COEF_1, 0x00fe1178},
	{ TEGRA210_DMIC_LP_BIQUAD_1_COEF_2, 0x00800000},
	{ TEGRA210_DMIC_LP_BIQUAD_1_COEF_3, 0x00e05f02},
	{ TEGRA210_DMIC_LP_BIQUAD_1_COEF_4, 0x006fc80d},
	{ TEGRA210_DMIC_CORRECTION_FILTER_GAIN, 0x010628f6},
	{ TEGRA210_DMIC_CORRECTION_BIQUAD_0_COEF_0, 0x00800000},
	{ TEGRA210_DMIC_CORRECTION_BIQUAD_0_COEF_3, 0x0067ffff},
	{ TEGRA210_DMIC_CORRECTION_BIQUAD_1_COEF_0, 0x00800000},
	{ TEGRA210_DMIC_CORRECTION_BIQUAD_1_COEF_1, 0x0048f5c2},
	{ TEGRA210_DMIC_CORRECTION_BIQUAD_1_COEF_3, 0x00562394},
	{ TEGRA210_DMIC_CORRECTION_BIQUAD_1_COEF_4, 0x00169446},
};

static int tegra210_dmic_runtime_suspend(struct device *dev)
{
	struct tegra210_dmic *dmic = dev_get_drvdata(dev);

	regcache_cache_only(dmic->regmap, true);
	regcache_mark_dirty(dmic->regmap);

	if (!(tegra_platform_is_unit_fpga() || tegra_platform_is_fpga()))
		clk_disable_unprepare(dmic->clk_dmic);

	return 0;
}

static int tegra210_dmic_runtime_resume(struct device *dev)
{
	struct tegra210_dmic *dmic = dev_get_drvdata(dev);
	int ret;

	if (!(tegra_platform_is_unit_fpga() || tegra_platform_is_fpga())) {
		ret = clk_prepare_enable(dmic->clk_dmic);
		if (ret) {
			dev_err(dev, "clk_enable failed: %d\n", ret);
			return ret;
		}
	}

	regcache_cache_only(dmic->regmap, false);
	regcache_sync(dmic->regmap);

	return 0;
}

static const int tegra210_dmic_fmt_values[] = {
	0,
	TEGRA210_AUDIOCIF_BITS_16,
	TEGRA210_AUDIOCIF_BITS_32,
};

static int tegra210_dmic_startup(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	struct tegra210_dmic *dmic = snd_soc_dai_get_drvdata(dai);
	int ret;

	if (dmic->prod_name != NULL) {
		ret = tegra_pinctrl_config_prod(dev, dmic->prod_name);
		if (ret < 0) {
			dev_warn(dev, "Failed to set %s setting\n",
				 dmic->prod_name);
		}
	}

	if (!(tegra_platform_is_unit_fpga() || tegra_platform_is_fpga())) {
		if (!IS_ERR_OR_NULL(dmic->pin_active_state)) {
			ret = pinctrl_select_state(dmic->pinctrl,
						dmic->pin_active_state);
			if (ret < 0) {
				dev_err(dev,
				"Setting dmic pinctrl active state failed\n");
				return -EINVAL;
			}
		}
	}

	return 0;
}

static void tegra210_dmic_shutdown(struct snd_pcm_substream *substream,
				   struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	struct tegra210_dmic *dmic = snd_soc_dai_get_drvdata(dai);
	int ret;

	if (!(tegra_platform_is_unit_fpga() || tegra_platform_is_fpga())) {
		if (!IS_ERR_OR_NULL(dmic->pin_idle_state)) {
			ret = pinctrl_select_state(
				dmic->pinctrl, dmic->pin_idle_state);
			if (ret < 0)
				dev_err(dev,
				"Setting dmic pinctrl idle state failed\n");
		}
	}
}

static int tegra210_dmic_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	struct tegra210_dmic *dmic = snd_soc_dai_get_drvdata(dai);
	int srate, dmic_clk, osr = dmic->osr_val, ret;
	struct tegra210_xbar_cif_conf cif_conf;
	unsigned long long boost_gain;
	unsigned int channels;

	memset(&cif_conf, 0, sizeof(struct tegra210_xbar_cif_conf));

	srate = params_rate(params);
	if (dmic->sample_rate_via_control)
		srate = dmic->sample_rate_via_control;
	dmic_clk = (1 << (6+osr)) * srate;

	channels = params_channels(params);
	if (dmic->channels_via_control)
		channels = dmic->channels_via_control;
	cif_conf.audio_channels = channels;

	switch (dmic->ch_select) {
	case DMIC_CH_SELECT_LEFT:
	case DMIC_CH_SELECT_RIGHT:
		cif_conf.client_channels = 1;
		break;
	case DMIC_CH_SELECT_STEREO:
		cif_conf.client_channels = 2;
		break;
	default:
		dev_err(dev, "unsupported ch_select value\n");
		return -EINVAL;
	}

	if ((tegra_platform_is_unit_fpga() || tegra_platform_is_fpga())) {
		program_dmic_gpio();
		program_dmic_clk(dmic_clk);
	} else {
		ret = clk_set_rate(dmic->clk_dmic, dmic_clk);
		if (ret) {
			dev_err(dev, "Can't set dmic clock rate: %d\n", ret);
			return ret;
		}
	}

	regmap_update_bits(dmic->regmap, TEGRA210_DMIC_CTRL,
			   TEGRA210_DMIC_CTRL_LRSEL_POLARITY_MASK,
			   dmic->lrsel << LRSEL_POL_SHIFT);
	regmap_update_bits(dmic->regmap, TEGRA210_DMIC_CTRL,
			   TEGRA210_DMIC_CTRL_OSR_MASK, osr << OSR_SHIFT);
	regmap_update_bits(dmic->regmap, TEGRA210_DMIC_DBG_CTRL,
			   TEGRA210_DMIC_DBG_CTRL_SC_ENABLE,
			   TEGRA210_DMIC_DBG_CTRL_SC_ENABLE);
	regmap_update_bits(dmic->regmap, TEGRA210_DMIC_DBG_CTRL,
			   TEGRA210_DMIC_DBG_CTRL_DCR_ENABLE,
			   TEGRA210_DMIC_DBG_CTRL_DCR_ENABLE);
	regmap_update_bits(dmic->regmap, TEGRA210_DMIC_CTRL,
			   TEGRA210_DMIC_CTRL_CHANNEL_SELECT_MASK,
			   (dmic->ch_select + 1) << CH_SEL_SHIFT);

	/* Configure LPF for passthrough and use */
	/* its gain register for applying boost; */
	/* Boost Gain control has 100x factor    */
	boost_gain = 0x00800000;
	if (dmic->boost_gain > 0) {
		boost_gain = ((boost_gain * dmic->boost_gain) / 100);
		if (boost_gain > 0x7FFFFFFF) {
			dev_warn(dev, "Boost Gain overflow\n");
			boost_gain = 0x7FFFFFFF;
		}
	}
	regmap_write(dmic->regmap, TEGRA210_DMIC_LP_FILTER_GAIN,
		     (unsigned int)boost_gain);

	regmap_update_bits(dmic->regmap, TEGRA210_DMIC_DBG_CTRL,
			   TEGRA210_DMIC_DBG_CTRL_LP_ENABLE,
			   TEGRA210_DMIC_DBG_CTRL_LP_ENABLE);

	/* Configure the two biquads for passthrough, */
	/* i.e. b0=1, b1=0, b2=0, a1=0, a2=0          */
	regmap_write(dmic->regmap, TEGRA210_DMIC_LP_BIQUAD_0_COEF_0,
		     0x00800000);
	regmap_write(dmic->regmap, TEGRA210_DMIC_LP_BIQUAD_0_COEF_1,
		     0x00000000);
	regmap_write(dmic->regmap, TEGRA210_DMIC_LP_BIQUAD_0_COEF_2,
		     0x00000000);
	regmap_write(dmic->regmap, TEGRA210_DMIC_LP_BIQUAD_0_COEF_3,
		     0x00000000);
	regmap_write(dmic->regmap, TEGRA210_DMIC_LP_BIQUAD_0_COEF_4,
		     0x00000000);
	regmap_write(dmic->regmap, TEGRA210_DMIC_LP_BIQUAD_1_COEF_0,
		     0x00800000);
	regmap_write(dmic->regmap, TEGRA210_DMIC_LP_BIQUAD_1_COEF_1,
		     0x00000000);
	regmap_write(dmic->regmap, TEGRA210_DMIC_LP_BIQUAD_1_COEF_2,
		     0x00000000);
	regmap_write(dmic->regmap, TEGRA210_DMIC_LP_BIQUAD_1_COEF_3,
		     0x00000000);
	regmap_write(dmic->regmap, TEGRA210_DMIC_LP_BIQUAD_1_COEF_4,
		     0x00000000);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		cif_conf.audio_bits = TEGRA210_AUDIOCIF_BITS_16;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		cif_conf.audio_bits = TEGRA210_AUDIOCIF_BITS_32;
		break;
	default:
		dev_err(dev, "Wrong format!\n");
		return -EINVAL;
	}

	if (dmic->format_out)
		cif_conf.audio_bits = tegra210_dmic_fmt_values[dmic->format_out];
	cif_conf.client_bits = TEGRA210_AUDIOCIF_BITS_24;
	cif_conf.mono_conv = dmic->mono_to_stereo;
	cif_conf.stereo_conv = dmic->stereo_to_mono;

	tegra210_xbar_set_cif(dmic->regmap, TEGRA210_DMIC_TX_CIF_CTRL,
			      &cif_conf);

	return 0;
}

static int tegra210_dmic_get_control(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tegra210_dmic *dmic = snd_soc_codec_get_drvdata(codec);

	if (strstr(kcontrol->id.name, "Boost"))
		ucontrol->value.integer.value[0] = dmic->boost_gain;
	else if (strstr(kcontrol->id.name, "Controller Channel Select"))
		ucontrol->value.integer.value[0] = dmic->ch_select;
	else if (strstr(kcontrol->id.name, "Capture mono to stereo"))
		ucontrol->value.integer.value[0] = dmic->mono_to_stereo;
	else if (strstr(kcontrol->id.name, "Capture stereo to mono"))
		ucontrol->value.integer.value[0] = dmic->stereo_to_mono;
	else if (strstr(kcontrol->id.name, "output bit format"))
		ucontrol->value.integer.value[0] = dmic->format_out;
	else if (strstr(kcontrol->id.name, "Sample Rate"))
		ucontrol->value.integer.value[0] =
					dmic->sample_rate_via_control;
	else if (strstr(kcontrol->id.name, "Channels"))
		ucontrol->value.integer.value[0] =
					dmic->channels_via_control;
	else if (strstr(kcontrol->id.name, "OSR Value"))
		ucontrol->value.integer.value[0] = dmic->osr_val;
	else if (strstr(kcontrol->id.name, "LR Select"))
		ucontrol->value.integer.value[0] = dmic->lrsel;

	return 0;
}

static int tegra210_dmic_put_control(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tegra210_dmic *dmic = snd_soc_codec_get_drvdata(codec);
	int value = ucontrol->value.integer.value[0];

	if (strstr(kcontrol->id.name, "Boost"))
		dmic->boost_gain = value;
	else if (strstr(kcontrol->id.name, "Controller Channel Select"))
		dmic->ch_select = ucontrol->value.integer.value[0];
	else if (strstr(kcontrol->id.name, "Capture mono to stereo"))
		dmic->mono_to_stereo = value;
	else if (strstr(kcontrol->id.name, "Capture stereo to mono"))
		dmic->stereo_to_mono = value;
	else if (strstr(kcontrol->id.name, "output bit format"))
		dmic->format_out = value;
	else if (strstr(kcontrol->id.name, "Sample Rate"))
		dmic->sample_rate_via_control = value;
	else if (strstr(kcontrol->id.name, "Channels"))
		dmic->channels_via_control = value;
	else if (strstr(kcontrol->id.name, "OSR Value"))
		dmic->osr_val = value;
	else if (strstr(kcontrol->id.name, "LR Select"))
		dmic->lrsel = value;

	return 0;
}

static struct snd_soc_dai_ops tegra210_dmic_dai_ops = {
	.hw_params	= tegra210_dmic_hw_params,
	.startup	= tegra210_dmic_startup,
	.shutdown       = tegra210_dmic_shutdown,
};

static struct snd_soc_dai_driver tegra210_dmic_dais[] = {
	{
		.name = "CIF",
		.capture = {
			.stream_name = "DMIC Transmit",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &tegra210_dmic_dai_ops,
		.symmetric_rates = 1,
	},
	{
		.name = "DAP",
		.playback = {
			.stream_name = "DMIC Receive",
			.channels_min = 1,
			.channels_max = 2,
			.rates = SNDRV_PCM_RATE_8000_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &tegra210_dmic_dai_ops,
		.symmetric_rates = 1,
	}
};

static const struct snd_soc_dapm_widget tegra210_dmic_widgets[] = {
	SND_SOC_DAPM_AIF_OUT("DMIC TX", NULL, 0, SND_SOC_NOPM,
			     0, 0),
	SND_SOC_DAPM_AIF_IN("DMIC RX", NULL, 0, TEGRA210_DMIC_ENABLE,
			    0, 0),
};

static const struct snd_soc_dapm_route tegra210_dmic_routes[] = {
	{ "DMIC RX",       NULL, "DMIC Receive" },
	{ "DMIC TX",       NULL, "DMIC RX" },
	{ "DMIC Transmit", NULL, "DMIC TX" },
};

static const char * const tegra210_dmic_ch_select[] = {
	"L", "R", "Stereo",
};

static const struct soc_enum tegra210_dmic_ch_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(tegra210_dmic_ch_select),
			tegra210_dmic_ch_select);

static const char * const tegra210_dmic_mono_conv_text[] = {
	"ZERO", "COPY",
};

static const char * const tegra210_dmic_stereo_conv_text[] = {
	"CH0", "CH1", "AVG",
};

static const struct soc_enum tegra210_dmic_mono_conv_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0,
			ARRAY_SIZE(tegra210_dmic_mono_conv_text),
			tegra210_dmic_mono_conv_text);

static const struct soc_enum tegra210_dmic_stereo_conv_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0,
			ARRAY_SIZE(tegra210_dmic_stereo_conv_text),
			tegra210_dmic_stereo_conv_text);

static const char * const tegra210_dmic_format_text[] = {
	"None",
	"16",
	"32",
};

static const struct soc_enum tegra210_dmic_format_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(tegra210_dmic_format_text),
			tegra210_dmic_format_text);

static const char * const tegra210_dmic_osr_text[] = {
	"OSR_64", "OSR_128", "OSR_256",
};

static const struct soc_enum tegra210_dmic_osr_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(tegra210_dmic_osr_text),
			tegra210_dmic_osr_text);

static const char * const tegra210_dmic_lrsel_text[] = {
	"Left", "Right",
};

static const struct soc_enum tegra210_dmic_lrsel_enum =
	SOC_ENUM_SINGLE(SND_SOC_NOPM, 0, ARRAY_SIZE(tegra210_dmic_lrsel_text),
			tegra210_dmic_lrsel_text);

static const struct snd_kcontrol_new tegra210_dmic_controls[] = {
	SOC_SINGLE_EXT("Boost Gain", 0, 0, 25599, 0, tegra210_dmic_get_control,
		       tegra210_dmic_put_control),
	SOC_ENUM_EXT("Controller Channel Select", tegra210_dmic_ch_enum,
		     tegra210_dmic_get_control, tegra210_dmic_put_control),
	SOC_ENUM_EXT("Capture mono to stereo conv",
		     tegra210_dmic_mono_conv_enum, tegra210_dmic_get_control,
		     tegra210_dmic_put_control),
	SOC_ENUM_EXT("Capture stereo to mono conv",
		     tegra210_dmic_stereo_conv_enum, tegra210_dmic_get_control,
		     tegra210_dmic_put_control),
	SOC_ENUM_EXT("output bit format", tegra210_dmic_format_enum,
		     tegra210_dmic_get_control, tegra210_dmic_put_control),
	SOC_SINGLE_EXT("Sample Rate", 0, 0, 48000, 0, tegra210_dmic_get_control,
		       tegra210_dmic_put_control),
	SOC_SINGLE_EXT("Channels", 0, 0, 2, 0, tegra210_dmic_get_control,
		       tegra210_dmic_put_control),
	SOC_ENUM_EXT("OSR Value", tegra210_dmic_osr_enum,
		     tegra210_dmic_get_control, tegra210_dmic_put_control),
	SOC_ENUM_EXT("LR Select", tegra210_dmic_lrsel_enum,
		     tegra210_dmic_get_control, tegra210_dmic_put_control),
};

static struct snd_soc_codec_driver tegra210_dmic_codec = {
	.idle_bias_off = 1,
	.component_driver = {
		.dapm_widgets = tegra210_dmic_widgets,
		.num_dapm_widgets = ARRAY_SIZE(tegra210_dmic_widgets),
		.dapm_routes = tegra210_dmic_routes,
		.num_dapm_routes = ARRAY_SIZE(tegra210_dmic_routes),
		.controls = tegra210_dmic_controls,
		.num_controls = ARRAY_SIZE(tegra210_dmic_controls),
	},
};

/* Regmap callback functions */
static bool tegra210_dmic_wr_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_DMIC_TX_INT_MASK:
	case TEGRA210_DMIC_TX_INT_SET:
	case TEGRA210_DMIC_TX_INT_CLEAR:
	case TEGRA210_DMIC_TX_CIF_CTRL:
	case TEGRA210_DMIC_ENABLE:
	case TEGRA210_DMIC_SOFT_RESET:
	case TEGRA210_DMIC_CG:
	case TEGRA210_DMIC_CTRL:
		return true;
	default:
		if (((reg % 4) == 0) && (reg >= TEGRA210_DMIC_DBG_CTRL) &&
		    (reg <= TEGRA210_DMIC_CORRECTION_BIQUAD_1_COEF_4))
			return true;
		else
			return false;
	};
}

static bool tegra210_dmic_rd_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_DMIC_TX_STATUS:
	case TEGRA210_DMIC_TX_INT_STATUS:
	case TEGRA210_DMIC_TX_INT_MASK:
	case TEGRA210_DMIC_TX_INT_SET:
	case TEGRA210_DMIC_TX_INT_CLEAR:
	case TEGRA210_DMIC_TX_CIF_CTRL:
	case TEGRA210_DMIC_ENABLE:
	case TEGRA210_DMIC_SOFT_RESET:
	case TEGRA210_DMIC_CG:
	case TEGRA210_DMIC_STATUS:
	case TEGRA210_DMIC_INT_STATUS:
	case TEGRA210_DMIC_CTRL:
		return true;
	default:
		if (((reg % 4) == 0) && (reg >= TEGRA210_DMIC_DBG_CTRL) &&
		    (reg <= TEGRA210_DMIC_CORRECTION_BIQUAD_1_COEF_4))
			return true;
		else
			return false;
	};
}

static bool tegra210_dmic_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case TEGRA210_DMIC_TX_STATUS:
	case TEGRA210_DMIC_TX_INT_STATUS:
	case TEGRA210_DMIC_TX_INT_SET:
	case TEGRA210_DMIC_SOFT_RESET:
	case TEGRA210_DMIC_STATUS:
	case TEGRA210_DMIC_INT_STATUS:
		return true;
	default:
		return false;
	};
}

static const struct regmap_config tegra210_dmic_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = TEGRA210_DMIC_CORRECTION_BIQUAD_1_COEF_4,
	.writeable_reg = tegra210_dmic_wr_reg,
	.readable_reg = tegra210_dmic_rd_reg,
	.volatile_reg = tegra210_dmic_volatile_reg,
	.precious_reg = NULL,
	.reg_defaults = tegra210_dmic_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(tegra210_dmic_reg_defaults),
	.cache_type = REGCACHE_FLAT,
};

static const struct of_device_id tegra210_dmic_of_match[] = {
	{ .compatible = "nvidia,tegra210-dmic" },
	{},
};

static int tegra210_dmic_platform_probe(struct platform_device *pdev)
{
	struct tegra210_dmic *dmic;
	struct device_node *np = pdev->dev.of_node;
	struct resource *mem;
	void __iomem *regs;
	int ret = 0;
	const struct of_device_id *match;

	match = of_match_device(tegra210_dmic_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "Error: No device match found\n");
		return -ENODEV;
	}

	dmic = devm_kzalloc(&pdev->dev, sizeof(*dmic), GFP_KERNEL);
	if (!dmic)
		return -ENOMEM;

	dmic->prod_name = NULL;
	dmic->osr_val = DMIC_OSR_64;
	dmic->ch_select = DMIC_CH_SELECT_STEREO;
	dev_set_drvdata(&pdev->dev, dmic);

	if (!(tegra_platform_is_unit_fpga() || tegra_platform_is_fpga())) {
		dmic->clk_dmic = devm_clk_get(&pdev->dev, "dmic");
		if (IS_ERR(dmic->clk_dmic)) {
			dev_err(&pdev->dev, "Can't retrieve dmic clock\n");
			return PTR_ERR(dmic->clk_dmic);
		}
	}

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(regs))
		return PTR_ERR(regs);
	dmic->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
					     &tegra210_dmic_regmap_config);
	if (IS_ERR(dmic->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		return PTR_ERR(dmic->regmap);
	}
	regcache_cache_only(dmic->regmap, true);

	/* Below patch is as per latest POR value */
	regmap_write(dmic->regmap, TEGRA210_DMIC_DCR_BIQUAD_0_COEF_4,
		     0x00000000);

	ret = of_property_read_u32(np, "nvidia,ahub-dmic-id",
				   &pdev->dev.id);
	if (ret < 0) {
		dev_err(&pdev->dev, "Missing property nvidia,ahub-dmic-id\n");
		return ret;
	}

	pm_runtime_enable(&pdev->dev);
	ret = snd_soc_register_codec(&pdev->dev, &tegra210_dmic_codec,
				     tegra210_dmic_dais,
				     ARRAY_SIZE(tegra210_dmic_dais));
	if (ret != 0) {
		dev_err(&pdev->dev, "Could not register CODEC: %d\n", ret);
		pm_runtime_disable(&pdev->dev);
		return ret;
	}

	if (of_property_read_string(np, "prod-name", &dmic->prod_name) == 0) {
		ret = tegra_pinctrl_config_prod(&pdev->dev, dmic->prod_name);
		if (ret < 0)
			dev_warn(&pdev->dev, "Failed to set %s setting\n",
				 dmic->prod_name);
	}

	dmic->pinctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(dmic->pinctrl)) {
		dev_dbg(&pdev->dev, "Missing pinctrl device\n");
		return 0;
	}

	dmic->pin_active_state = pinctrl_lookup_state(dmic->pinctrl,
							"dap_active");
	dmic->pin_idle_state = pinctrl_lookup_state(dmic->pinctrl,
							"dap_inactive");
	if (IS_ERR(dmic->pin_active_state) && IS_ERR(dmic->pin_idle_state)) {
		dev_dbg(&pdev->dev, "Pinctrl: No DAP states found\n");
		devm_pinctrl_put(dmic->pinctrl);
	}

	return 0;
}

static int tegra210_dmic_platform_remove(struct platform_device *pdev)
{
	struct tegra210_dmic *dmic;

	dmic = dev_get_drvdata(&pdev->dev);
	snd_soc_unregister_codec(&pdev->dev);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra210_dmic_runtime_suspend(&pdev->dev);

	return 0;
}

static const struct dev_pm_ops tegra210_dmic_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra210_dmic_runtime_suspend,
			   tegra210_dmic_runtime_resume, NULL)
	SET_LATE_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				     pm_runtime_force_resume)
};

static struct platform_driver tegra210_dmic_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra210_dmic_of_match,
		.pm = &tegra210_dmic_pm_ops,
	},
	.probe = tegra210_dmic_platform_probe,
	.remove = tegra210_dmic_platform_remove,
};
module_platform_driver(tegra210_dmic_driver)

MODULE_AUTHOR("Rahul Mittal <rmittal@nvidia.com>");
MODULE_DESCRIPTION("Tegra210 DMIC ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, tegra210_dmic_of_match);
