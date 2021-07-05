/*
 * tegra210_adx_alt.c - Tegra210 ADX driver
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
#include <soc/tegra/chip-id.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <linux/of_device.h>

#include "tegra210_xbar_alt.h"
#include "tegra210_adx_alt.h"

#define DRV_NAME "tegra210-adx"

static const struct reg_default tegra210_adx_reg_defaults[] = {
	{ TEGRA210_ADX_AXBAR_RX_INT_MASK, 0x00000001},
	{ TEGRA210_ADX_AXBAR_RX_CIF_CTRL, 0x00007000},
	{ TEGRA210_ADX_AXBAR_TX_INT_MASK, 0x0000000f },
	{ TEGRA210_ADX_AXBAR_TX1_CIF_CTRL, 0x00007000},
	{ TEGRA210_ADX_AXBAR_TX2_CIF_CTRL, 0x00007000},
	{ TEGRA210_ADX_AXBAR_TX3_CIF_CTRL, 0x00007000},
	{ TEGRA210_ADX_AXBAR_TX4_CIF_CTRL, 0x00007000},
	{ TEGRA210_ADX_CG, 0x1},
	{ TEGRA210_ADX_AHUBRAMCTL_ADX_CTRL, 0x00004000},
};

/**
 * tegra210_adx_enable_outstream - enable output stream
 * @adx: struct of tegra210_adx
 * @stream_id: adx output stream id for enabling
 */
static void tegra210_adx_enable_outstream(struct tegra210_adx *adx,
					unsigned int stream_id)
{
	int reg;

	reg = TEGRA210_ADX_CTRL;

	regmap_update_bits(adx->regmap, reg,
			TEGRA210_ADX_TX_ENABLE << stream_id,
			TEGRA210_ADX_TX_ENABLE << stream_id);
}

/**
 * tegra210_adx_disable_outstream - disable output stream
 * @adx: struct of tegra210_adx
 * @stream_id: adx output stream id for disabling
 */
static void tegra210_adx_disable_outstream(struct tegra210_adx *adx,
					unsigned int stream_id)
{
	int reg;

	reg = TEGRA210_ADX_CTRL;

	regmap_update_bits(adx->regmap, reg,
			TEGRA210_ADX_TX_ENABLE << stream_id,
			TEGRA210_ADX_TX_DISABLE);
}

/**
 * tegra210_adx_set_in_byte_mask - set byte mask for input frame
 * @adx: struct of tegra210_adx
 * @mask1: enable for bytes 31 ~ 0 of input frame
 * @mask2: enable for bytes 63 ~ 32 of input frame
 */
static void tegra210_adx_set_in_byte_mask(struct tegra210_adx *adx)
{
	regmap_write(adx->regmap,
		TEGRA210_ADX_IN_BYTE_EN0, adx->byte_mask[0]);
	regmap_write(adx->regmap,
		TEGRA210_ADX_IN_BYTE_EN1, adx->byte_mask[1]);
}

/**
 * tegra210_adx_set_map_table - set map table not RAM
 * @adx: struct of tegra210_adx
 * @out_byte_addr: byte address in one frame
 * @stream_id: input stream id
 * @nth_word: n-th word in the input stream
 * @nth_byte: n-th byte in the word
 */
static void tegra210_adx_set_map_table(struct tegra210_adx *adx,
			unsigned int out_byte_addr,
			unsigned int stream_id,
			unsigned int nth_word,
			unsigned int nth_byte)
{
	unsigned char *bytes_map = (unsigned char *)&adx->map;

	bytes_map[out_byte_addr] =
			(stream_id << TEGRA210_ADX_MAP_STREAM_NUMBER_SHIFT) |
			(nth_word << TEGRA210_ADX_MAP_WORD_NUMBER_SHIFT) |
			(nth_byte << TEGRA210_ADX_MAP_BYTE_NUMBER_SHIFT);
}

/**
 * tegra210_adx_write_map_ram - write map information in RAM
 * @adx: struct of tegra210_adx
 * @addr: n-th word of input stream
 * @val : bytes mapping information of the word
 */
static void tegra210_adx_write_map_ram(struct tegra210_adx *adx,
				unsigned int addr,
				unsigned int val)
{
	unsigned int reg;

	regmap_write(adx->regmap, TEGRA210_ADX_AHUBRAMCTL_ADX_CTRL,
		 (addr << TEGRA210_ADX_AHUBRAMCTL_ADX_CTRL_RAM_ADDR_SHIFT));

	regmap_write(adx->regmap, TEGRA210_ADX_AHUBRAMCTL_ADX_DATA, val);

	regmap_read(adx->regmap, TEGRA210_ADX_AHUBRAMCTL_ADX_CTRL, &reg);
	reg |= TEGRA210_ADX_AHUBRAMCTL_ADX_CTRL_ADDR_INIT_EN;

	regmap_write(adx->regmap, TEGRA210_ADX_AHUBRAMCTL_ADX_CTRL, reg);

	regmap_read(adx->regmap, TEGRA210_ADX_AHUBRAMCTL_ADX_CTRL, &reg);
	reg |= TEGRA210_ADX_AHUBRAMCTL_ADX_CTRL_RW_WRITE;

	regmap_write(adx->regmap, TEGRA210_ADX_AHUBRAMCTL_ADX_CTRL, reg);
}

static void tegra210_adx_update_map_ram(struct tegra210_adx *adx)
{
	int i;

	for (i = 0; i < TEGRA210_ADX_RAM_DEPTH; i++)
		tegra210_adx_write_map_ram(adx, i, adx->map[i]);
}

static int tegra210_adx_sw_reset(struct tegra210_adx *adx,
				int timeout)
{
	unsigned int val;
	int wait = timeout;

	regmap_update_bits(adx->regmap, TEGRA210_ADX_SOFT_RESET,
		TEGRA210_ADX_SOFT_RESET_SOFT_RESET_MASK,
		TEGRA210_ADX_SOFT_RESET_SOFT_EN);

	do {
		regmap_read(adx->regmap, TEGRA210_ADX_SOFT_RESET, &val);
		wait--;
		if (!wait)
			return -EINVAL;
	} while (val & 0x00000001);

	regmap_update_bits(adx->regmap, TEGRA210_ADX_SOFT_RESET,
		TEGRA210_ADX_SOFT_RESET_SOFT_RESET_MASK,
		TEGRA210_ADX_SOFT_RESET_SOFT_DEFAULT);

	return 0;
}

static int tegra210_adx_get_status(struct tegra210_adx *adx)
{
	unsigned int val;

	regmap_read(adx->regmap, TEGRA210_ADX_STATUS, &val);
	val = (val & 0x00000001);

	return val;
}

static int tegra210_adx_stop(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = snd_soc_dapm_to_codec(w->dapm);
	struct device *dev = codec->dev;
	struct tegra210_adx *adx = dev_get_drvdata(dev);
	int dcnt = 10, ret = 0;

	/* wait until ADX status is disabled */
	while (tegra210_adx_get_status(adx) && dcnt--)
		udelay(100);

	/* HW needs sw reset to make sure previous transaction be clean */
	ret = tegra210_adx_sw_reset(adx, 0xffff);
	if (ret) {
		dev_err(dev, "Failed at ADX%d sw reset\n", dev->id);
		return ret;
	}

	return (dcnt < 0) ? -ETIMEDOUT : 0;
}

static unsigned int __maybe_unused tegra210_adx_read_map_ram(
						struct tegra210_adx *adx,
						unsigned int addr)
{
	unsigned int val, wait;
	wait = 0xffff;

	regmap_write(adx->regmap, TEGRA210_ADX_AHUBRAMCTL_ADX_CTRL,
			(addr << TEGRA210_ADX_AHUBRAMCTL_ADX_CTRL_RAM_ADDR_SHIFT));

	regmap_read(adx->regmap, TEGRA210_ADX_AHUBRAMCTL_ADX_CTRL, &val);
	val |= TEGRA210_ADX_AHUBRAMCTL_ADX_CTRL_ADDR_INIT_EN;
	regmap_write(adx->regmap, TEGRA210_ADX_AHUBRAMCTL_ADX_CTRL, val);
	regmap_read(adx->regmap, TEGRA210_ADX_AHUBRAMCTL_ADX_CTRL, &val);
	val &= ~(TEGRA210_ADX_AHUBRAMCTL_ADX_CTRL_RW_WRITE);
	regmap_write(adx->regmap, TEGRA210_ADX_AHUBRAMCTL_ADX_CTRL, val);

	do {
		regmap_read(adx->regmap, TEGRA210_ADX_AHUBRAMCTL_ADX_CTRL, &val);
		wait--;
		if (!wait)
			return -EINVAL;
	} while (val & 0x80000000);

	regmap_read(adx->regmap, TEGRA210_ADX_AHUBRAMCTL_ADX_DATA, &val);

	return val;
}

static int tegra210_adx_runtime_suspend(struct device *dev)
{
	struct tegra210_adx *adx = dev_get_drvdata(dev);

	regcache_cache_only(adx->regmap, true);
	regcache_mark_dirty(adx->regmap);

	return 0;
}

static int tegra210_adx_runtime_resume(struct device *dev)
{
	struct tegra210_adx *adx = dev_get_drvdata(dev);

	regcache_cache_only(adx->regmap, false);
	regcache_sync(adx->regmap);
	/* update the map ram */
	tegra210_adx_update_map_ram(adx);
	tegra210_adx_set_in_byte_mask(adx);

	return 0;
}

static int tegra210_adx_set_audio_cif(struct snd_soc_dai *dai,
				      int channels, int format,
				      unsigned int reg)
{
	struct tegra210_adx *adx = snd_soc_dai_get_drvdata(dai);
	struct tegra210_xbar_cif_conf cif_conf;
	int audio_bits;

	memset(&cif_conf, 0, sizeof(struct tegra210_xbar_cif_conf));

	if (channels < 1 || channels > 16)
		return -EINVAL;

	switch (format) {
	case SNDRV_PCM_FORMAT_S8:
		audio_bits = TEGRA210_AUDIOCIF_BITS_8;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		audio_bits = TEGRA210_AUDIOCIF_BITS_16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		audio_bits = TEGRA210_AUDIOCIF_BITS_24;
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

	tegra210_xbar_set_cif(adx->regmap, reg, &cif_conf);

	return 0;
}

static int tegra210_adx_out_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct tegra210_adx *adx = snd_soc_dai_get_drvdata(dai);
	int channels;

	if (adx->output_channels[dai->id] > 0)
		channels = adx->output_channels[dai->id];
	else
		channels = params_channels(params);

	return tegra210_adx_set_audio_cif(dai, channels, params_format(params),
			TEGRA210_ADX_AXBAR_TX1_CIF_CTRL +
			(dai->id * TEGRA210_ADX_AUDIOCIF_CH_STRIDE));
}

static int tegra210_adx_out_trigger(struct snd_pcm_substream *substream,
				 int cmd,
				 struct snd_soc_dai *dai)
{
	struct tegra210_adx *adx = snd_soc_dai_get_drvdata(dai);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		tegra210_adx_enable_outstream(adx, dai->id);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		tegra210_adx_disable_outstream(adx, dai->id);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int tegra210_adx_in_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct tegra210_adx *adx = snd_soc_dai_get_drvdata(dai);
	int channels;

	if (tegra_platform_is_unit_fpga() || tegra_platform_is_fpga()) {
		/* update the map ram */
		tegra210_adx_update_map_ram(adx);
		tegra210_adx_set_in_byte_mask(adx);
	}

	if (adx->input_channels > 0)
		channels = adx->input_channels;
	else
		channels = params_channels(params);

	return tegra210_adx_set_audio_cif(dai, channels, params_format(params),
					  TEGRA210_ADX_AXBAR_RX_CIF_CTRL);
}

static int tegra210_adx_set_channel_map(struct snd_soc_dai *dai,
				unsigned int tx_num, unsigned int *tx_slot,
				unsigned int rx_num, unsigned int *rx_slot)
{
	struct device *dev = dai->dev;
	struct tegra210_adx *adx = snd_soc_dai_get_drvdata(dai);
	unsigned int out_stream_idx, out_ch_idx, out_byte_idx;
	int i;

	if ((rx_num < 1) || (rx_num > 64)) {
		dev_err(dev, "Doesn't support %d rx_num, need to be 1 to 64\n",
			rx_num);
		return -EINVAL;
	}

	if (!rx_slot) {
		dev_err(dev, "rx_slot is NULL\n");
		return -EINVAL;
	}

	memset(adx->map, 0, sizeof(adx->map));
	memset(adx->byte_mask, 0, sizeof(adx->byte_mask));

	for (i = 0; i < rx_num; i++) {
		if (rx_slot[i] != 0) {
			/* getting mapping information */
			/* n-th output stream : 0 to 3 */
			out_stream_idx = (rx_slot[i] >> 16) & 0x3;
			/* n-th audio channel of output stream : 1 to 16 */
			out_ch_idx = (rx_slot[i] >> 8) & 0x1f;
			/* n-th byte of audio channel : 0 to 3 */
			out_byte_idx = rx_slot[i] & 0x3;
			tegra210_adx_set_map_table(adx, i, out_stream_idx,
					out_ch_idx - 1,
					out_byte_idx);

			/* making byte_mask */
			if (i > 31)
				adx->byte_mask[1] |= (1 << (i - 32));
			else
				adx->byte_mask[0] |= (1 << i);
		}
	}

	return 0;
}

static int tegra210_adx_get_byte_map(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tegra210_adx *adx = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc;
	unsigned char *bytes_map = (unsigned char *)&adx->map;
	int enabled;

	mc = (struct soc_mixer_control *)kcontrol->private_value;
	enabled = adx->byte_mask[mc->reg / 32] & (1 << (mc->reg % 32));

	if (enabled)
		ucontrol->value.integer.value[0] = bytes_map[mc->reg];
	else
		ucontrol->value.integer.value[0] = 256;

	return 0;
}

static int tegra210_adx_put_byte_map(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tegra210_adx *adx = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc;
	unsigned char *bytes_map = (unsigned char *)&adx->map;
	int value = ucontrol->value.integer.value[0];

	mc = (struct soc_mixer_control *)kcontrol->private_value;

	if (value >= 0 && value <= 255) {
		/* update byte map and enable slot */
		bytes_map[mc->reg] = value;
		adx->byte_mask[mc->reg / 32] |= (1 << (mc->reg % 32));
	} else {
		/* reset byte map and disable slot */
		bytes_map[mc->reg] = 0;
		adx->byte_mask[mc->reg / 32] &= ~(1 << (mc->reg % 32));
	}

	return 0;
}

static int tegra210_adx_get_in_channels(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tegra210_adx *adx = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = adx->input_channels;

	return 0;
}

static int tegra210_adx_put_in_channels(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tegra210_adx *adx = snd_soc_codec_get_drvdata(codec);
	int value = ucontrol->value.integer.value[0];

	if (value < 0 || value > 16)
		return -EINVAL;

	adx->input_channels = value;

	return 0;
}

static int tegra210_adx_get_out_channels(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tegra210_adx *adx = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc;

	mc = (struct soc_mixer_control *)kcontrol->private_value;

	ucontrol->value.integer.value[0] = adx->output_channels[mc->reg - 1];

	return 0;
}

static int tegra210_adx_put_out_channels(struct snd_kcontrol *kcontrol,
					 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_kcontrol_codec(kcontrol);
	struct tegra210_adx *adx = snd_soc_codec_get_drvdata(codec);
	struct soc_mixer_control *mc;
	int value = ucontrol->value.integer.value[0];

	mc = (struct soc_mixer_control *)kcontrol->private_value;

	if (value < 0 || value > 16)
		return -EINVAL;

	adx->output_channels[mc->reg - 1] = value;

	return 0;
}

static struct snd_soc_dai_ops tegra210_adx_in_dai_ops = {
	.hw_params	= tegra210_adx_in_hw_params,
	.set_channel_map = tegra210_adx_set_channel_map,
};

static struct snd_soc_dai_ops tegra210_adx_out_dai_ops = {
	.hw_params	= tegra210_adx_out_hw_params,
	.trigger	= tegra210_adx_out_trigger,
};

#define OUT_DAI(id)						\
	{							\
		.name = "OUT" #id,				\
		.capture = {					\
			.stream_name = "OUT" #id " Transmit",	\
			.channels_min = 1,			\
			.channels_max = 16,			\
			.rates = SNDRV_PCM_RATE_8000_96000,	\
			.formats = SNDRV_PCM_FMTBIT_S16_LE,	\
		},						\
		.ops = &tegra210_adx_out_dai_ops,		\
	}

#define IN_DAI(sname, dai_ops)					\
	{							\
		.name = #sname,					\
		.playback = {					\
			.stream_name = #sname " Receive",	\
			.channels_min = 1,			\
			.channels_max = 16,			\
			.rates = SNDRV_PCM_RATE_8000_96000,	\
			.formats = SNDRV_PCM_FMTBIT_S16_LE,	\
		},						\
		.ops = dai_ops,					\
	}

static struct snd_soc_dai_driver tegra210_adx_dais[] = {
	OUT_DAI(1),
	OUT_DAI(2),
	OUT_DAI(3),
	OUT_DAI(4),
	IN_DAI(IN, &tegra210_adx_in_dai_ops),
};

static const struct snd_soc_dapm_widget tegra210_adx_widgets[] = {
	SND_SOC_DAPM_AIF_IN_E("IN", NULL, 0, TEGRA210_ADX_ENABLE,
				TEGRA210_ADX_ENABLE_SHIFT, 0,
				tegra210_adx_stop, SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_AIF_OUT("OUT1", NULL, 0, TEGRA210_ADX_CTRL, 0, 0),
	SND_SOC_DAPM_AIF_OUT("OUT2", NULL, 0, TEGRA210_ADX_CTRL, 1, 0),
	SND_SOC_DAPM_AIF_OUT("OUT3", NULL, 0, TEGRA210_ADX_CTRL, 2, 0),
	SND_SOC_DAPM_AIF_OUT("OUT4", NULL, 0, TEGRA210_ADX_CTRL, 3, 0),
};

static const struct snd_soc_dapm_route tegra210_adx_routes[] = {
	{ "IN", NULL, "IN Receive" },
	{ "OUT1", NULL, "IN" },
	{ "OUT2", NULL, "IN" },
	{ "OUT3", NULL, "IN" },
	{ "OUT4", NULL, "IN" },
	{ "OUT1 Transmit", NULL, "OUT1" },
	{ "OUT2 Transmit", NULL, "OUT2" },
	{ "OUT3 Transmit", NULL, "OUT3" },
	{ "OUT4 Transmit", NULL, "OUT4" },
};

#define TEGRA210_ADX_BYTE_MAP_CTRL(reg) \
	SOC_SINGLE_EXT("Byte Map " #reg, reg, 0, 256, 0, \
		       tegra210_adx_get_byte_map, \
		       tegra210_adx_put_byte_map)

#define TEGRA210_ADX_OUTPUT_CHANNELS_CTRL(reg) \
	SOC_SINGLE_EXT("Output" #reg " Channels", reg, 0, 16, 0, \
		       tegra210_adx_get_out_channels, \
		       tegra210_adx_put_out_channels)

#define TEGRA210_ADX_INPUT_CHANNELS_CTRL(reg) \
	SOC_SINGLE_EXT("Input Channels", reg, 0, 16, 0, \
		       tegra210_adx_get_in_channels, \
		       tegra210_adx_put_in_channels)

static struct snd_kcontrol_new tegra210_adx_controls[] = {
	TEGRA210_ADX_BYTE_MAP_CTRL(0),
	TEGRA210_ADX_BYTE_MAP_CTRL(1),
	TEGRA210_ADX_BYTE_MAP_CTRL(2),
	TEGRA210_ADX_BYTE_MAP_CTRL(3),
	TEGRA210_ADX_BYTE_MAP_CTRL(4),
	TEGRA210_ADX_BYTE_MAP_CTRL(5),
	TEGRA210_ADX_BYTE_MAP_CTRL(6),
	TEGRA210_ADX_BYTE_MAP_CTRL(7),
	TEGRA210_ADX_BYTE_MAP_CTRL(8),
	TEGRA210_ADX_BYTE_MAP_CTRL(9),
	TEGRA210_ADX_BYTE_MAP_CTRL(10),
	TEGRA210_ADX_BYTE_MAP_CTRL(11),
	TEGRA210_ADX_BYTE_MAP_CTRL(12),
	TEGRA210_ADX_BYTE_MAP_CTRL(13),
	TEGRA210_ADX_BYTE_MAP_CTRL(14),
	TEGRA210_ADX_BYTE_MAP_CTRL(15),
	TEGRA210_ADX_BYTE_MAP_CTRL(16),
	TEGRA210_ADX_BYTE_MAP_CTRL(17),
	TEGRA210_ADX_BYTE_MAP_CTRL(18),
	TEGRA210_ADX_BYTE_MAP_CTRL(19),
	TEGRA210_ADX_BYTE_MAP_CTRL(20),
	TEGRA210_ADX_BYTE_MAP_CTRL(21),
	TEGRA210_ADX_BYTE_MAP_CTRL(22),
	TEGRA210_ADX_BYTE_MAP_CTRL(23),
	TEGRA210_ADX_BYTE_MAP_CTRL(24),
	TEGRA210_ADX_BYTE_MAP_CTRL(25),
	TEGRA210_ADX_BYTE_MAP_CTRL(26),
	TEGRA210_ADX_BYTE_MAP_CTRL(27),
	TEGRA210_ADX_BYTE_MAP_CTRL(28),
	TEGRA210_ADX_BYTE_MAP_CTRL(29),
	TEGRA210_ADX_BYTE_MAP_CTRL(30),
	TEGRA210_ADX_BYTE_MAP_CTRL(31),
	TEGRA210_ADX_BYTE_MAP_CTRL(32),
	TEGRA210_ADX_BYTE_MAP_CTRL(33),
	TEGRA210_ADX_BYTE_MAP_CTRL(34),
	TEGRA210_ADX_BYTE_MAP_CTRL(35),
	TEGRA210_ADX_BYTE_MAP_CTRL(36),
	TEGRA210_ADX_BYTE_MAP_CTRL(37),
	TEGRA210_ADX_BYTE_MAP_CTRL(38),
	TEGRA210_ADX_BYTE_MAP_CTRL(39),
	TEGRA210_ADX_BYTE_MAP_CTRL(40),
	TEGRA210_ADX_BYTE_MAP_CTRL(41),
	TEGRA210_ADX_BYTE_MAP_CTRL(42),
	TEGRA210_ADX_BYTE_MAP_CTRL(43),
	TEGRA210_ADX_BYTE_MAP_CTRL(44),
	TEGRA210_ADX_BYTE_MAP_CTRL(45),
	TEGRA210_ADX_BYTE_MAP_CTRL(46),
	TEGRA210_ADX_BYTE_MAP_CTRL(47),
	TEGRA210_ADX_BYTE_MAP_CTRL(48),
	TEGRA210_ADX_BYTE_MAP_CTRL(49),
	TEGRA210_ADX_BYTE_MAP_CTRL(50),
	TEGRA210_ADX_BYTE_MAP_CTRL(51),
	TEGRA210_ADX_BYTE_MAP_CTRL(52),
	TEGRA210_ADX_BYTE_MAP_CTRL(53),
	TEGRA210_ADX_BYTE_MAP_CTRL(54),
	TEGRA210_ADX_BYTE_MAP_CTRL(55),
	TEGRA210_ADX_BYTE_MAP_CTRL(56),
	TEGRA210_ADX_BYTE_MAP_CTRL(57),
	TEGRA210_ADX_BYTE_MAP_CTRL(58),
	TEGRA210_ADX_BYTE_MAP_CTRL(59),
	TEGRA210_ADX_BYTE_MAP_CTRL(60),
	TEGRA210_ADX_BYTE_MAP_CTRL(61),
	TEGRA210_ADX_BYTE_MAP_CTRL(62),
	TEGRA210_ADX_BYTE_MAP_CTRL(63),

	TEGRA210_ADX_OUTPUT_CHANNELS_CTRL(1),
	TEGRA210_ADX_OUTPUT_CHANNELS_CTRL(2),
	TEGRA210_ADX_OUTPUT_CHANNELS_CTRL(3),
	TEGRA210_ADX_OUTPUT_CHANNELS_CTRL(4),
	TEGRA210_ADX_INPUT_CHANNELS_CTRL(1),
};

static struct snd_soc_codec_driver tegra210_adx_codec = {
	.idle_bias_off = 1,
	.component_driver = {
		.dapm_widgets = tegra210_adx_widgets,
		.num_dapm_widgets = ARRAY_SIZE(tegra210_adx_widgets),
		.dapm_routes = tegra210_adx_routes,
		.num_dapm_routes = ARRAY_SIZE(tegra210_adx_routes),
		.controls = tegra210_adx_controls,
		.num_controls = ARRAY_SIZE(tegra210_adx_controls),
	},
};

static bool tegra210_adx_wr_reg(struct device *dev,
				unsigned int reg)
{
	switch (reg) {
	case TEGRA210_ADX_AXBAR_TX_INT_MASK:
	case TEGRA210_ADX_AXBAR_TX_INT_SET:
	case TEGRA210_ADX_AXBAR_TX_INT_CLEAR:
	case TEGRA210_ADX_AXBAR_TX1_CIF_CTRL:
	case TEGRA210_ADX_AXBAR_TX2_CIF_CTRL:
	case TEGRA210_ADX_AXBAR_TX3_CIF_CTRL:
	case TEGRA210_ADX_AXBAR_TX4_CIF_CTRL:
	case TEGRA210_ADX_AXBAR_RX_INT_MASK:
	case TEGRA210_ADX_AXBAR_RX_INT_SET:
	case TEGRA210_ADX_AXBAR_RX_INT_CLEAR:
	case TEGRA210_ADX_AXBAR_RX_CIF_CTRL:
	case TEGRA210_ADX_ENABLE:
	case TEGRA210_ADX_SOFT_RESET:
	case TEGRA210_ADX_CG:
	case TEGRA210_ADX_CTRL:
	case TEGRA210_ADX_IN_BYTE_EN0:
	case TEGRA210_ADX_IN_BYTE_EN1:
	case TEGRA210_ADX_CYA:
	case TEGRA210_ADX_AHUBRAMCTL_ADX_CTRL:
	case TEGRA210_ADX_AHUBRAMCTL_ADX_DATA:
		return true;
	default:
		return false;
	};
}

static bool tegra210_adx_rd_reg(struct device *dev,
				unsigned int reg)
{
	switch (reg) {
	case TEGRA210_ADX_AXBAR_RX_STATUS:
	case TEGRA210_ADX_AXBAR_RX_INT_STATUS:
	case TEGRA210_ADX_AXBAR_RX_INT_MASK:
	case TEGRA210_ADX_AXBAR_RX_INT_SET:
	case TEGRA210_ADX_AXBAR_RX_INT_CLEAR:
	case TEGRA210_ADX_AXBAR_RX_CIF_CTRL:
	case TEGRA210_ADX_AXBAR_TX_STATUS:
	case TEGRA210_ADX_AXBAR_TX_INT_STATUS:
	case TEGRA210_ADX_AXBAR_TX_INT_MASK:
	case TEGRA210_ADX_AXBAR_TX_INT_SET:
	case TEGRA210_ADX_AXBAR_TX_INT_CLEAR:
	case TEGRA210_ADX_AXBAR_TX1_CIF_CTRL:
	case TEGRA210_ADX_AXBAR_TX2_CIF_CTRL:
	case TEGRA210_ADX_AXBAR_TX3_CIF_CTRL:
	case TEGRA210_ADX_AXBAR_TX4_CIF_CTRL:
	case TEGRA210_ADX_ENABLE:
	case TEGRA210_ADX_SOFT_RESET:
	case TEGRA210_ADX_CG:
	case TEGRA210_ADX_STATUS:
	case TEGRA210_ADX_INT_STATUS:
	case TEGRA210_ADX_CTRL:
	case TEGRA210_ADX_IN_BYTE_EN0:
	case TEGRA210_ADX_IN_BYTE_EN1:
	case TEGRA210_ADX_CYA:
	case TEGRA210_ADX_AHUBRAMCTL_ADX_CTRL:
	case TEGRA210_ADX_AHUBRAMCTL_ADX_DATA:
		return true;
	default:
		return false;
	};
}

static bool tegra210_adx_volatile_reg(struct device *dev,
				unsigned int reg)
{
	switch (reg) {
	case TEGRA210_ADX_AXBAR_RX_STATUS:
	case TEGRA210_ADX_AXBAR_RX_INT_STATUS:
	case TEGRA210_ADX_AXBAR_RX_INT_SET:
	case TEGRA210_ADX_AXBAR_TX_STATUS:
	case TEGRA210_ADX_AXBAR_TX_INT_STATUS:
	case TEGRA210_ADX_AXBAR_TX_INT_SET:
	case TEGRA210_ADX_SOFT_RESET:
	case TEGRA210_ADX_STATUS:
	case TEGRA210_ADX_INT_STATUS:
	case TEGRA210_ADX_AHUBRAMCTL_ADX_CTRL:
	case TEGRA210_ADX_AHUBRAMCTL_ADX_DATA:
		return true;
	default:
		break;
	};

	return false;
}

static const struct regmap_config tegra210_adx_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = TEGRA210_ADX_AHUBRAMCTL_ADX_DATA,
	.writeable_reg = tegra210_adx_wr_reg,
	.readable_reg = tegra210_adx_rd_reg,
	.volatile_reg = tegra210_adx_volatile_reg,
	.reg_defaults = tegra210_adx_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(tegra210_adx_reg_defaults),
	.cache_type = REGCACHE_FLAT,
};

static const struct of_device_id tegra210_adx_of_match[] = {
	{ .compatible = "nvidia,tegra210-adx" },
	{},
};

static int tegra210_adx_platform_probe(struct platform_device *pdev)
{
	struct tegra210_adx *adx;
	struct resource *mem;
	void __iomem *regs;
	int ret = 0;
	const struct of_device_id *match;

	match = of_match_device(tegra210_adx_of_match, &pdev->dev);
	if (!match) {
		dev_err(&pdev->dev, "Error: No device match found\n");
		return -ENODEV;
	}

	adx = devm_kzalloc(&pdev->dev, sizeof(*adx), GFP_KERNEL);
	if (!adx)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, adx);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(regs))
		return PTR_ERR(regs);
	adx->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
					    &tegra210_adx_regmap_config);
	if (IS_ERR(adx->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		return PTR_ERR(adx->regmap);
	}
	regcache_cache_only(adx->regmap, true);

	ret = of_property_read_u32(pdev->dev.of_node,
				   "nvidia,ahub-adx-id",
				   &pdev->dev.id);
	if (ret < 0) {
		dev_err(&pdev->dev, "Missing property nvidia,ahub-adx-id\n");
		return ret;
	}

	pm_runtime_enable(&pdev->dev);
	ret = snd_soc_register_codec(&pdev->dev, &tegra210_adx_codec,
				     tegra210_adx_dais,
				     ARRAY_SIZE(tegra210_adx_dais));
	if (ret != 0) {
		dev_err(&pdev->dev, "Could not register CODEC: %d\n", ret);
		pm_runtime_disable(&pdev->dev);
		return ret;
	}

	return 0;
}

static int tegra210_adx_platform_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra210_adx_runtime_suspend(&pdev->dev);

	return 0;
}

static const struct dev_pm_ops tegra210_adx_pm_ops = {
	SET_RUNTIME_PM_OPS(tegra210_adx_runtime_suspend,
			   tegra210_adx_runtime_resume, NULL)
	SET_LATE_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				     pm_runtime_force_resume)
};

static struct platform_driver tegra210_adx_driver = {
	.driver = {
		.name = DRV_NAME,
		.owner = THIS_MODULE,
		.of_match_table = tegra210_adx_of_match,
		.pm = &tegra210_adx_pm_ops,
	},
	.probe = tegra210_adx_platform_probe,
	.remove = tegra210_adx_platform_remove,
};
module_platform_driver(tegra210_adx_driver);

MODULE_AUTHOR("Arun Shamanna Lakshmi <aruns@nvidia.com>");
MODULE_DESCRIPTION("Tegra210 ADX ASoC driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, tegra210_adx_of_match);
