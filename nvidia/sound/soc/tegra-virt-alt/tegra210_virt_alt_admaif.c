/*
 * tegra210_virt_alt_admaif.c - Tegra ADMAIF component driver
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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_platform.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>

#include "tegra210_virt_alt_admaif.h"
#include "tegra_virt_alt_ivc.h"
#include "tegra_pcm_alt.h"
#include "tegra_asoc_xbar_virt_alt.h"
#include "tegra_asoc_util_virt_alt.h"

#define NUM_META_CONTROLS	3

static const unsigned int tegra210_rates[] = {
	8000, 11025, 12000, 16000, 22050,
	24000, 32000, 44100, 48000, 64000,
	88200, 96000, 176400, 192000
};

static const struct snd_pcm_hw_constraint_list tegra210_rate_constraints = {
	.count = ARRAY_SIZE(tegra210_rates),
	.list = tegra210_rates,
};

static struct tegra210_admaif *admaif;
static int tegra210_admaif_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
{
	struct device *dev = dai->dev;
	struct tegra210_virt_admaif_client_data *data =
				&admaif->client_data;
	struct tegra210_virt_audio_cif cif_conf;
	struct nvaudio_ivc_msg	msg;
	unsigned int value;
	int err;

	memset(&cif_conf, 0, sizeof(struct tegra210_virt_audio_cif));
	cif_conf.audio_channels = params_channels(params);
	cif_conf.client_channels = params_channels(params);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		cif_conf.client_bits = TEGRA210_AUDIOCIF_BITS_8;
		cif_conf.audio_bits = TEGRA210_AUDIOCIF_BITS_8;
		break;
	case SNDRV_PCM_FORMAT_S16_LE:
		cif_conf.client_bits = TEGRA210_AUDIOCIF_BITS_16;
		cif_conf.audio_bits = TEGRA210_AUDIOCIF_BITS_16;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		cif_conf.client_bits = TEGRA210_AUDIOCIF_BITS_24;
		cif_conf.audio_bits = TEGRA210_AUDIOCIF_BITS_24;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		cif_conf.client_bits = TEGRA210_AUDIOCIF_BITS_32;
		cif_conf.audio_bits = TEGRA210_AUDIOCIF_BITS_32;
		break;
	default:
		dev_err(dev, "Wrong format!\n");
		return -EINVAL;
	}
	cif_conf.direction = substream->stream;

	value = (cif_conf.threshold <<
			TEGRA210_AUDIOCIF_CTRL_FIFO_THRESHOLD_SHIFT) |
		((cif_conf.audio_channels - 1) <<
			TEGRA210_AUDIOCIF_CTRL_AUDIO_CHANNELS_SHIFT) |
		((cif_conf.client_channels - 1) <<
			TEGRA210_AUDIOCIF_CTRL_CLIENT_CHANNELS_SHIFT) |
		(cif_conf.audio_bits <<
			TEGRA210_AUDIOCIF_CTRL_AUDIO_BITS_SHIFT) |
		(cif_conf.client_bits <<
			TEGRA210_AUDIOCIF_CTRL_CLIENT_BITS_SHIFT) |
		(cif_conf.expand <<
			TEGRA210_AUDIOCIF_CTRL_EXPAND_SHIFT) |
		(cif_conf.stereo_conv <<
			TEGRA210_AUDIOCIF_CTRL_STEREO_CONV_SHIFT) |
		(cif_conf.replicate <<
			TEGRA210_AUDIOCIF_CTRL_REPLICATE_SHIFT) |
		(cif_conf.truncate <<
			TEGRA210_AUDIOCIF_CTRL_TRUNCATE_SHIFT) |
		(cif_conf.mono_conv <<
			TEGRA210_AUDIOCIF_CTRL_MONO_CONV_SHIFT);

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.params.dmaif_info.id        = dai->id;
	msg.params.dmaif_info.value     = value;
	if (!cif_conf.direction)
		msg.cmd = NVAUDIO_DMAIF_SET_TXCIF;
	else
		msg.cmd = NVAUDIO_DMAIF_SET_RXCIF;

	err = nvaudio_ivc_send(data->hivc_client,
				&msg,
				sizeof(struct nvaudio_ivc_msg));

	if (err < 0)
		pr_err("%s: error on ivc_send\n", __func__);

	return 0;
}

static void tegra210_admaif_start_playback(struct snd_soc_dai *dai)
{
	struct tegra210_virt_admaif_client_data *data =
				&admaif->client_data;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_START_PLAYBACK;
	msg.params.dmaif_info.id = dai->id;
	msg.ack_required = true;
	err = nvaudio_ivc_send_receive(data->hivc_client,
			&msg, sizeof(struct nvaudio_ivc_msg));

	if (err < 0)
		pr_err("%s: error on ivc_send\n", __func__);
}

static void tegra210_admaif_stop_playback(struct snd_soc_dai *dai)
{
	struct tegra210_virt_admaif_client_data *data =
				&admaif->client_data;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_STOP_PLAYBACK;
	msg.params.dmaif_info.id = dai->id;

	msg.ack_required = true;
	err = nvaudio_ivc_send_receive(data->hivc_client,
			&msg, sizeof(struct nvaudio_ivc_msg));

	if (err < 0)
		pr_err("%s: error on ivc_send\n", __func__);
}

static void tegra210_admaif_start_capture(struct snd_soc_dai *dai)
{
	struct tegra210_virt_admaif_client_data *data =
				&admaif->client_data;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_START_CAPTURE;
	msg.params.dmaif_info.id = dai->id;

	msg.ack_required = true;
	err = nvaudio_ivc_send_receive(data->hivc_client,
			&msg, sizeof(struct nvaudio_ivc_msg));

	if (err < 0)
		pr_err("%s: error on ivc_send\n", __func__);
}

static void tegra210_admaif_stop_capture(struct snd_soc_dai *dai)
{
	struct tegra210_virt_admaif_client_data *data =
				&admaif->client_data;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_STOP_CAPTURE;
	msg.params.dmaif_info.id = dai->id;

	msg.ack_required = true;
	err = nvaudio_ivc_send_receive(data->hivc_client,
			&msg, sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("%s: error on ivc_send\n", __func__);
}

static int tegra210_admaif_trigger(struct snd_pcm_substream *substream, int cmd,
				 struct snd_soc_dai *dai)
{
	pr_info("Pcm trigger for admaif %d : cmd_id %d \n", dai->id +1 , cmd);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			tegra210_admaif_start_playback(dai);
		else
			tegra210_admaif_start_capture(dai);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			tegra210_admaif_stop_playback(dai);
		else
			tegra210_admaif_stop_capture(dai);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int tegra210_admaif_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *cpu_dai)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE, &tegra210_rate_constraints);
}

static struct snd_soc_dai_ops tegra210_admaif_dai_ops = {
	.hw_params	= tegra210_admaif_hw_params,
	.trigger	= tegra210_admaif_trigger,
	.startup	= tegra210_admaif_startup,
};

static int tegra210_admaif_dai_probe(struct snd_soc_dai *dai)
{

	dai->capture_dma_data = &admaif->capture_dma_data[dai->id];
	dai->playback_dma_data = &admaif->playback_dma_data[dai->id];

	return 0;
}

static int tegra_bytes_info(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_info *uinfo)
{
	struct soc_bytes *params = (void *)kcontrol->private_value;

	uinfo->type = params->mask;
	uinfo->count = params->num_regs;

	return 0;
}

#define ADMAIF_DAI(id)							\
	{							\
		.name = "ADMAIF" #id,				\
		.probe = tegra210_admaif_dai_probe,		\
		.playback = {					\
			.stream_name = "Playback " #id,		\
			.channels_min = 1,			\
			.channels_max = 16,			\
			.rates = SNDRV_PCM_RATE_8000_192000,	\
			.formats = SNDRV_PCM_FMTBIT_S8 |	\
				SNDRV_PCM_FMTBIT_S16_LE |	\
				SNDRV_PCM_FMTBIT_S24_LE |	\
				SNDRV_PCM_FMTBIT_S32_LE,	\
		},						\
		.capture = {					\
			.stream_name = "Capture " #id,		\
			.channels_min = 1,			\
			.channels_max = 16,			\
			.rates = SNDRV_PCM_RATE_8000_192000,		\
			.formats = SNDRV_PCM_FMTBIT_S8 |		\
				SNDRV_PCM_FMTBIT_S16_LE |		\
				SNDRV_PCM_FMTBIT_S24_LE |		\
				SNDRV_PCM_FMTBIT_S32_LE,		\
		},						\
		.ops = &tegra210_admaif_dai_ops,			\
	}

static struct snd_soc_dai_driver tegra210_admaif_dais[] = {
	ADMAIF_DAI(1),
	ADMAIF_DAI(2),
	ADMAIF_DAI(3),
	ADMAIF_DAI(4),
	ADMAIF_DAI(5),
	ADMAIF_DAI(6),
	ADMAIF_DAI(7),
	ADMAIF_DAI(8),
	ADMAIF_DAI(9),
	ADMAIF_DAI(10),
	ADMAIF_DAI(11),
	ADMAIF_DAI(12),
	ADMAIF_DAI(13),
	ADMAIF_DAI(14),
	ADMAIF_DAI(15),
	ADMAIF_DAI(16),
	ADMAIF_DAI(17),
	ADMAIF_DAI(18),
	ADMAIF_DAI(19),
	ADMAIF_DAI(20),
};

static const struct soc_enum tegra_virt_t186_asrc_source =
	SOC_ENUM_SINGLE_EXT(NUM_ASRC_MODE, tegra186_asrc_ratio_source_text);

static const struct soc_enum tegra_virt_t186_arad_source =
	SOC_VALUE_ENUM_SINGLE(0, 0, 0, NUM_ARAD_SOURCES,
					tegra186_arad_mux_text,
					tegra186_arad_mux_value);

static const struct soc_enum tegra_virt_t210_mvc_curvetype =
	SOC_ENUM_SINGLE_EXT(NUM_MVC_CURVETYPE, tegra210_mvc_curve_type_text);

static const struct snd_kcontrol_new tegra_virt_t210ref_controls[] = {
MIXER_GAIN_CTRL_DECL("RX1 Gain", 0x00),
MIXER_GAIN_CTRL_DECL("RX2 Gain", 0x01),
MIXER_GAIN_CTRL_DECL("RX3 Gain", 0x02),
MIXER_GAIN_CTRL_DECL("RX4 Gain", 0x03),
MIXER_GAIN_CTRL_DECL("RX5 Gain", 0x04),
MIXER_GAIN_CTRL_DECL("RX6 Gain", 0x05),
MIXER_GAIN_CTRL_DECL("RX7 Gain", 0x06),
MIXER_GAIN_CTRL_DECL("RX8 Gain", 0x07),
MIXER_GAIN_CTRL_DECL("RX9 Gain", 0x08),
MIXER_GAIN_CTRL_DECL("RX10 Gain", 0x09),

MIXER_GAIN_INSTANT_CTRL_DECL("RX1 Gain Instant", 0x00),
MIXER_GAIN_INSTANT_CTRL_DECL("RX2 Gain Instant", 0x01),
MIXER_GAIN_INSTANT_CTRL_DECL("RX3 Gain Instant", 0x02),
MIXER_GAIN_INSTANT_CTRL_DECL("RX4 Gain Instant", 0x03),
MIXER_GAIN_INSTANT_CTRL_DECL("RX5 Gain Instant", 0x04),
MIXER_GAIN_INSTANT_CTRL_DECL("RX6 Gain Instant", 0x05),
MIXER_GAIN_INSTANT_CTRL_DECL("RX7 Gain Instant", 0x06),
MIXER_GAIN_INSTANT_CTRL_DECL("RX8 Gain Instant", 0x07),
MIXER_GAIN_INSTANT_CTRL_DECL("RX9 Gain Instant", 0x08),
MIXER_GAIN_INSTANT_CTRL_DECL("RX10 Gain Instant", 0x09),

MIXER_DURATION_CTRL_DECL("RX1 Duration", 0x00),
MIXER_DURATION_CTRL_DECL("RX2 Duration", 0x01),
MIXER_DURATION_CTRL_DECL("RX3 Duration", 0x02),
MIXER_DURATION_CTRL_DECL("RX4 Duration", 0x03),
MIXER_DURATION_CTRL_DECL("RX5 Duration", 0x04),
MIXER_DURATION_CTRL_DECL("RX6 Duration", 0x05),
MIXER_DURATION_CTRL_DECL("RX7 Duration", 0x06),
MIXER_DURATION_CTRL_DECL("RX8 Duration", 0x07),
MIXER_DURATION_CTRL_DECL("RX9 Duration", 0x08),
MIXER_DURATION_CTRL_DECL("RX10 Duration", 0x09),

MIXER_ENABLE_CTRL_DECL("Mixer Enable", 0x00),
MIXER_SET_FADE("Mixer fade", 0x00),
MIXER_GET_FADE_STATUS("Mixer fade status", 0x00),

SFC_IN_FREQ_CTRL_DECL("SFC1 input rate", 0x00),
SFC_IN_FREQ_CTRL_DECL("SFC2 input rate", 0x01),
SFC_IN_FREQ_CTRL_DECL("SFC3 input rate", 0x02),
SFC_IN_FREQ_CTRL_DECL("SFC4 input rate", 0x03),

SFC_OUT_FREQ_CTRL_DECL("SFC1 output rate", 0x00),
SFC_OUT_FREQ_CTRL_DECL("SFC2 output rate", 0x01),
SFC_OUT_FREQ_CTRL_DECL("SFC3 output rate", 0x02),
SFC_OUT_FREQ_CTRL_DECL("SFC4 output rate", 0x03),

MVC_CURVE_TYPE_CTRL_DECL("MVC1 Curve Type", 0x00,
			&tegra_virt_t210_mvc_curvetype),
MVC_CURVE_TYPE_CTRL_DECL("MVC2 Curve Type", 0x01,
			&tegra_virt_t210_mvc_curvetype),

MVC_TAR_VOL_CTRL_DECL("MVC1 Vol", 0x00),
MVC_TAR_VOL_CTRL_DECL("MVC2 Vol", 0x01),

MVC_MUTE_CTRL_DECL("MVC1 Mute", 0x00),
MVC_MUTE_CTRL_DECL("MVC2 Mute", 0x01),

AMX_ENABLE_CTRL_DECL("AMX1-1 Enable", 0x01, 0x01),
AMX_ENABLE_CTRL_DECL("AMX1-2 Enable", 0x01, 0x02),
AMX_ENABLE_CTRL_DECL("AMX1-3 Enable", 0x01, 0x03),
AMX_ENABLE_CTRL_DECL("AMX1-4 Enable", 0x01, 0x04),

AMX_ENABLE_CTRL_DECL("AMX2-1 Enable", 0x02, 0x01),
AMX_ENABLE_CTRL_DECL("AMX2-2 Enable", 0x02, 0x02),
AMX_ENABLE_CTRL_DECL("AMX2-3 Enable", 0x02, 0x03),
AMX_ENABLE_CTRL_DECL("AMX2-4 Enable", 0x02, 0x04),

I2S_LOOPBACK_ENABLE_CTRL_DECL("I2S1 Loopback", 0x01),
I2S_LOOPBACK_ENABLE_CTRL_DECL("I2S2 Loopback", 0x02),
I2S_LOOPBACK_ENABLE_CTRL_DECL("I2S3 Loopback", 0x03),
I2S_LOOPBACK_ENABLE_CTRL_DECL("I2S4 Loopback", 0x04),
I2S_LOOPBACK_ENABLE_CTRL_DECL("I2S5 Loopback", 0x05),

REGDUMP_CTRL_DECL("ADMAIF1 regdump", ADMAIF1, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF2 regdump", ADMAIF2, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF3 regdump", ADMAIF3, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF4 regdump", ADMAIF4, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF5 regdump", ADMAIF5, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF6 regdump", ADMAIF6, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF7 regdump", ADMAIF7, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF8 regdump", ADMAIF8, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF9 regdump", ADMAIF9, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF10 regdump", ADMAIF10, 0, NVAUDIO_REGDUMP_RX_TX),

REGDUMP_CTRL_DECL("AMX1 regdump", AMX1, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("AMX2 regdump", AMX2, 0, NVAUDIO_REGDUMP_RX_TX),

REGDUMP_CTRL_DECL("ADX1 regdump", ADX1, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADX2 regdump", ADX2, 0, NVAUDIO_REGDUMP_RX_TX),

REGDUMP_CTRL_DECL("MIXER1-1 RX regdump", MIXER1, 0, NVAUDIO_REGDUMP_RX),
REGDUMP_CTRL_DECL("MIXER1-2 RX regdump", MIXER1, 1, NVAUDIO_REGDUMP_RX),
REGDUMP_CTRL_DECL("MIXER1-3 RX regdump", MIXER1, 2, NVAUDIO_REGDUMP_RX),
REGDUMP_CTRL_DECL("MIXER1-4 RX regdump", MIXER1, 3, NVAUDIO_REGDUMP_RX),
REGDUMP_CTRL_DECL("MIXER1-5 RX regdump", MIXER1, 4, NVAUDIO_REGDUMP_RX),
REGDUMP_CTRL_DECL("MIXER1-6 RX regdump", MIXER1, 5, NVAUDIO_REGDUMP_RX),
REGDUMP_CTRL_DECL("MIXER1-7 RX regdump", MIXER1, 6, NVAUDIO_REGDUMP_RX),
REGDUMP_CTRL_DECL("MIXER1-8 RX regdump", MIXER1, 7, NVAUDIO_REGDUMP_RX),
REGDUMP_CTRL_DECL("MIXER1-9 RX regdump", MIXER1, 8, NVAUDIO_REGDUMP_RX),
REGDUMP_CTRL_DECL("MIXER1-10 RX regdump", MIXER1, 9, NVAUDIO_REGDUMP_RX),

REGDUMP_CTRL_DECL("MIXER1-1 TX regdump", MIXER1, 0, NVAUDIO_REGDUMP_TX),
REGDUMP_CTRL_DECL("MIXER1-2 TX regdump", MIXER1, 1, NVAUDIO_REGDUMP_TX),
REGDUMP_CTRL_DECL("MIXER1-3 TX regdump", MIXER1, 2, NVAUDIO_REGDUMP_TX),
REGDUMP_CTRL_DECL("MIXER1-4 TX regdump", MIXER1, 3, NVAUDIO_REGDUMP_TX),
REGDUMP_CTRL_DECL("MIXER1-5 TX regdump", MIXER1, 4, NVAUDIO_REGDUMP_TX),

REGDUMP_CTRL_DECL("I2S1 regdump", I2S1, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("I2S2 regdump", I2S2, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("I2S3 regdump", I2S3, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("I2S4 regdump", I2S4, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("I2S5 regdump", I2S5, 0, NVAUDIO_REGDUMP_RX_TX),

REGDUMP_CTRL_DECL("SFC1 regdump", SFC1, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("SFC2 regdump", SFC2, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("SFC3 regdump", SFC3, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("SFC4 regdump", SFC4, 0, NVAUDIO_REGDUMP_RX_TX),

REGDUMP_CTRL_DECL("MVC1 regdump", MVC1, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("MVC2 regdump", MVC2, 0, NVAUDIO_REGDUMP_RX_TX),

ADMA_REGDUMP_CTRL_DECL("ADMA1 regdump", 1),
ADMA_REGDUMP_CTRL_DECL("ADMA2 regdump", 2),
ADMA_REGDUMP_CTRL_DECL("ADMA3 regdump", 3),
ADMA_REGDUMP_CTRL_DECL("ADMA4 regdump", 4),
ADMA_REGDUMP_CTRL_DECL("ADMA5 regdump", 5),
ADMA_REGDUMP_CTRL_DECL("ADMA6 regdump", 6),
ADMA_REGDUMP_CTRL_DECL("ADMA7 regdump", 7),
ADMA_REGDUMP_CTRL_DECL("ADMA8 regdump", 8),
ADMA_REGDUMP_CTRL_DECL("ADMA9 regdump", 9),
ADMA_REGDUMP_CTRL_DECL("ADMA10 regdump", 10),
ADMA_REGDUMP_CTRL_DECL("ADMA11 regdump", 11),
ADMA_REGDUMP_CTRL_DECL("ADMA12 regdump", 12),
ADMA_REGDUMP_CTRL_DECL("ADMA13 regdump", 13),
ADMA_REGDUMP_CTRL_DECL("ADMA14 regdump", 14),
ADMA_REGDUMP_CTRL_DECL("ADMA15 regdump", 15),
ADMA_REGDUMP_CTRL_DECL("ADMA16 regdump", 16),
ADMA_REGDUMP_CTRL_DECL("ADMA17 regdump", 17),
ADMA_REGDUMP_CTRL_DECL("ADMA18 regdump", 18),
ADMA_REGDUMP_CTRL_DECL("ADMA19 regdump", 19),
ADMA_REGDUMP_CTRL_DECL("ADMA20 regdump", 20),

};

static const struct snd_kcontrol_new tegra_virt_t186ref_controls[] = {
MIXER_GAIN_CTRL_DECL("RX1 Gain", 0x00),
MIXER_GAIN_CTRL_DECL("RX2 Gain", 0x01),
MIXER_GAIN_CTRL_DECL("RX3 Gain", 0x02),
MIXER_GAIN_CTRL_DECL("RX4 Gain", 0x03),
MIXER_GAIN_CTRL_DECL("RX5 Gain", 0x04),
MIXER_GAIN_CTRL_DECL("RX6 Gain", 0x05),
MIXER_GAIN_CTRL_DECL("RX7 Gain", 0x06),
MIXER_GAIN_CTRL_DECL("RX8 Gain", 0x07),
MIXER_GAIN_CTRL_DECL("RX9 Gain", 0x08),
MIXER_GAIN_CTRL_DECL("RX10 Gain", 0x09),

MIXER_GAIN_INSTANT_CTRL_DECL("RX1 Gain Instant", 0x00),
MIXER_GAIN_INSTANT_CTRL_DECL("RX2 Gain Instant", 0x01),
MIXER_GAIN_INSTANT_CTRL_DECL("RX3 Gain Instant", 0x02),
MIXER_GAIN_INSTANT_CTRL_DECL("RX4 Gain Instant", 0x03),
MIXER_GAIN_INSTANT_CTRL_DECL("RX5 Gain Instant", 0x04),
MIXER_GAIN_INSTANT_CTRL_DECL("RX6 Gain Instant", 0x05),
MIXER_GAIN_INSTANT_CTRL_DECL("RX7 Gain Instant", 0x06),
MIXER_GAIN_INSTANT_CTRL_DECL("RX8 Gain Instant", 0x07),
MIXER_GAIN_INSTANT_CTRL_DECL("RX9 Gain Instant", 0x08),
MIXER_GAIN_INSTANT_CTRL_DECL("RX10 Gain Instant", 0x09),

MIXER_DURATION_CTRL_DECL("RX1 Duration", 0x00),
MIXER_DURATION_CTRL_DECL("RX2 Duration", 0x01),
MIXER_DURATION_CTRL_DECL("RX3 Duration", 0x02),
MIXER_DURATION_CTRL_DECL("RX4 Duration", 0x03),
MIXER_DURATION_CTRL_DECL("RX5 Duration", 0x04),
MIXER_DURATION_CTRL_DECL("RX6 Duration", 0x05),
MIXER_DURATION_CTRL_DECL("RX7 Duration", 0x06),
MIXER_DURATION_CTRL_DECL("RX8 Duration", 0x07),
MIXER_DURATION_CTRL_DECL("RX9 Duration", 0x08),
MIXER_DURATION_CTRL_DECL("RX10 Duration", 0x09),

MIXER_ENABLE_CTRL_DECL("Mixer Enable", 0x00),
MIXER_SET_FADE("Mixer fade", 0x00),
MIXER_GET_FADE_STATUS("Mixer fade status", 0x00),

SFC_IN_FREQ_CTRL_DECL("SFC1 input rate", 0x00),
SFC_IN_FREQ_CTRL_DECL("SFC2 input rate", 0x01),
SFC_IN_FREQ_CTRL_DECL("SFC3 input rate", 0x02),
SFC_IN_FREQ_CTRL_DECL("SFC4 input rate", 0x03),

SFC_OUT_FREQ_CTRL_DECL("SFC1 output rate", 0x00),
SFC_OUT_FREQ_CTRL_DECL("SFC2 output rate", 0x01),
SFC_OUT_FREQ_CTRL_DECL("SFC3 output rate", 0x02),
SFC_OUT_FREQ_CTRL_DECL("SFC4 output rate", 0x03),

MVC_CURVE_TYPE_CTRL_DECL("MVC1 Curve Type", 0x00,
			&tegra_virt_t210_mvc_curvetype),
MVC_CURVE_TYPE_CTRL_DECL("MVC2 Curve Type", 0x01,
			&tegra_virt_t210_mvc_curvetype),

MVC_TAR_VOL_CTRL_DECL("MVC1 Vol", 0x00),
MVC_TAR_VOL_CTRL_DECL("MVC2 Vol", 0x01),

MVC_MUTE_CTRL_DECL("MVC1 Mute", 0x00),
MVC_MUTE_CTRL_DECL("MVC2 Mute", 0x01),

ASRC_RATIO_CTRL_DECL("ASRC1 Ratio1", 0x01),
ASRC_RATIO_CTRL_DECL("ASRC1 Ratio2", 0x02),
ASRC_RATIO_CTRL_DECL("ASRC1 Ratio3", 0x03),
ASRC_RATIO_CTRL_DECL("ASRC1 Ratio4", 0x04),
ASRC_RATIO_CTRL_DECL("ASRC1 Ratio5", 0x05),
ASRC_RATIO_CTRL_DECL("ASRC1 Ratio6", 0x06),

ASRC_STREAM_RATIO_CTRL_DECL("ASRC1 Ratio1 SRC", 0x01,
			&tegra_virt_t186_asrc_source),
ASRC_STREAM_RATIO_CTRL_DECL("ASRC1 Ratio2 SRC", 0x02,
			&tegra_virt_t186_asrc_source),
ASRC_STREAM_RATIO_CTRL_DECL("ASRC1 Ratio3 SRC", 0x03,
			&tegra_virt_t186_asrc_source),
ASRC_STREAM_RATIO_CTRL_DECL("ASRC1 Ratio4 SRC", 0x04,
			&tegra_virt_t186_asrc_source),
ASRC_STREAM_RATIO_CTRL_DECL("ASRC1 Ratio5 SRC", 0x05,
			&tegra_virt_t186_asrc_source),
ASRC_STREAM_RATIO_CTRL_DECL("ASRC1 Ratio6 SRC", 0x06,
			&tegra_virt_t186_asrc_source),

ASRC_STREAM_ENABLE_CTRL_DECL("ASRC1 Stream1 Enable", 0x01),
ASRC_STREAM_ENABLE_CTRL_DECL("ASRC1 Stream2 Enable", 0x02),
ASRC_STREAM_ENABLE_CTRL_DECL("ASRC1 Stream3 Enable", 0x03),
ASRC_STREAM_ENABLE_CTRL_DECL("ASRC1 Stream4 Enable", 0x04),
ASRC_STREAM_ENABLE_CTRL_DECL("ASRC1 Stream5 Enable", 0x05),
ASRC_STREAM_ENABLE_CTRL_DECL("ASRC1 Stream6 Enable", 0x06),

ASRC_STREAM_HWCOMP_CTRL_DECL("ASRC1 Stream1 Hwcomp Disable", 0x01),
ASRC_STREAM_HWCOMP_CTRL_DECL("ASRC1 Stream2 Hwcomp Disable", 0x02),
ASRC_STREAM_HWCOMP_CTRL_DECL("ASRC1 Stream3 Hwcomp Disable", 0x03),
ASRC_STREAM_HWCOMP_CTRL_DECL("ASRC1 Stream4 Hwcomp Disable", 0x04),
ASRC_STREAM_HWCOMP_CTRL_DECL("ASRC1 Stream5 Hwcomp Disable", 0x05),
ASRC_STREAM_HWCOMP_CTRL_DECL("ASRC1 Stream6 Hwcomp Disable", 0x06),

ASRC_STREAM_INPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream1 Input Thresh", 0x01),
ASRC_STREAM_INPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream2 Input Thresh", 0x02),
ASRC_STREAM_INPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream3 Input Thresh", 0x03),
ASRC_STREAM_INPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream4 Input Thresh", 0x04),
ASRC_STREAM_INPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream5 Input Thresh", 0x05),
ASRC_STREAM_INPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream6 Input Thresh", 0x06),

ASRC_STREAM_OUTPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream1 Output Thresh", 0x01),
ASRC_STREAM_OUTPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream2 Output Thresh", 0x02),
ASRC_STREAM_OUTPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream3 Output Thresh", 0x03),
ASRC_STREAM_OUTPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream4 Output Thresh", 0x04),
ASRC_STREAM_OUTPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream5 Output Thresh", 0x05),
ASRC_STREAM_OUTPUT_THRESHOLD_CTRL_DECL("ASRC1 Stream6 Output Thresh", 0x06),

ARAD_LANE_SOURCE_CTRL_DECL("Numerator1 Mux", numerator1_enum,
				&tegra_virt_t186_arad_source),
ARAD_LANE_SOURCE_CTRL_DECL("Numerator2 Mux", numerator2_enum,
				&tegra_virt_t186_arad_source),
ARAD_LANE_SOURCE_CTRL_DECL("Numerator3 Mux", numerator3_enum,
				&tegra_virt_t186_arad_source),
ARAD_LANE_SOURCE_CTRL_DECL("Numerator4 Mux", numerator4_enum,
				&tegra_virt_t186_arad_source),
ARAD_LANE_SOURCE_CTRL_DECL("Numerator5 Mux", numerator5_enum,
				&tegra_virt_t186_arad_source),
ARAD_LANE_SOURCE_CTRL_DECL("Numerator6 Mux", numerator6_enum,
				&tegra_virt_t186_arad_source),

ARAD_LANE_SOURCE_CTRL_DECL("Denominator1 Mux", denominator1_enum,
				&tegra_virt_t186_arad_source),
ARAD_LANE_SOURCE_CTRL_DECL("Denominator2 Mux", denominator2_enum,
				&tegra_virt_t186_arad_source),
ARAD_LANE_SOURCE_CTRL_DECL("Denominator3 Mux", denominator3_enum,
				&tegra_virt_t186_arad_source),
ARAD_LANE_SOURCE_CTRL_DECL("Denominator4 Mux", denominator4_enum,
				&tegra_virt_t186_arad_source),
ARAD_LANE_SOURCE_CTRL_DECL("Denominator5 Mux", denominator5_enum,
				&tegra_virt_t186_arad_source),
ARAD_LANE_SOURCE_CTRL_DECL("Denominator6 Mux", denominator6_enum,
				&tegra_virt_t186_arad_source),

ARAD_LANE_PRESCALAR_CTRL_DECL("Numerator1 Prescalar", numerator1_enum),
ARAD_LANE_PRESCALAR_CTRL_DECL("Numerator2 Prescalar", numerator2_enum),
ARAD_LANE_PRESCALAR_CTRL_DECL("Numerator3 Prescalar", numerator3_enum),
ARAD_LANE_PRESCALAR_CTRL_DECL("Numerator4 Prescalar", numerator4_enum),
ARAD_LANE_PRESCALAR_CTRL_DECL("Numerator5 Prescalar", numerator5_enum),
ARAD_LANE_PRESCALAR_CTRL_DECL("Numerator6 Prescalar", numerator6_enum),

ARAD_LANE_PRESCALAR_CTRL_DECL("Denominator1 Prescalar", denominator1_enum),
ARAD_LANE_PRESCALAR_CTRL_DECL("Denominator2 Prescalar", denominator2_enum),
ARAD_LANE_PRESCALAR_CTRL_DECL("Denominator3 Prescalar", denominator3_enum),
ARAD_LANE_PRESCALAR_CTRL_DECL("Denominator4 Prescalar", denominator4_enum),
ARAD_LANE_PRESCALAR_CTRL_DECL("Denominator5 Prescalar", denominator5_enum),
ARAD_LANE_PRESCALAR_CTRL_DECL("Denominator6 Prescalar", denominator6_enum),

ARAD_LANE_ENABLE_CTRL_DECL("Lane1 enable", 0x00),
ARAD_LANE_ENABLE_CTRL_DECL("Lane2 enable", 0x01),
ARAD_LANE_ENABLE_CTRL_DECL("Lane3 enable", 0x02),
ARAD_LANE_ENABLE_CTRL_DECL("Lane4 enable", 0x03),
ARAD_LANE_ENABLE_CTRL_DECL("Lane5 enable", 0x04),
ARAD_LANE_ENABLE_CTRL_DECL("Lane6 enable", 0x05),

ARAD_LANE_RATIO_CTRL_DECL("Lane1 Ratio", 0x00),
ARAD_LANE_RATIO_CTRL_DECL("Lane2 Ratio", 0x01),
ARAD_LANE_RATIO_CTRL_DECL("Lane3 Ratio", 0x02),
ARAD_LANE_RATIO_CTRL_DECL("Lane4 Ratio", 0x03),
ARAD_LANE_RATIO_CTRL_DECL("Lane5 Ratio", 0x04),
ARAD_LANE_RATIO_CTRL_DECL("Lane6 Ratio", 0x05),

AMX_ENABLE_CTRL_DECL("AMX1-1 Enable", 0x01, 0x01),
AMX_ENABLE_CTRL_DECL("AMX1-2 Enable", 0x01, 0x02),
AMX_ENABLE_CTRL_DECL("AMX1-3 Enable", 0x01, 0x03),
AMX_ENABLE_CTRL_DECL("AMX1-4 Enable", 0x01, 0x04),

AMX_ENABLE_CTRL_DECL("AMX2-1 Enable", 0x02, 0x01),
AMX_ENABLE_CTRL_DECL("AMX2-2 Enable", 0x02, 0x02),
AMX_ENABLE_CTRL_DECL("AMX2-3 Enable", 0x02, 0x03),
AMX_ENABLE_CTRL_DECL("AMX2-4 Enable", 0x02, 0x04),

AMX_ENABLE_CTRL_DECL("AMX3-1 Enable", 0x03, 0x01),
AMX_ENABLE_CTRL_DECL("AMX3-2 Enable", 0x03, 0x02),
AMX_ENABLE_CTRL_DECL("AMX3-3 Enable", 0x03, 0x03),
AMX_ENABLE_CTRL_DECL("AMX3-4 Enable", 0x03, 0x04),

AMX_ENABLE_CTRL_DECL("AMX4-1 Enable", 0x04, 0x01),
AMX_ENABLE_CTRL_DECL("AMX4-2 Enable", 0x04, 0x02),
AMX_ENABLE_CTRL_DECL("AMX4-3 Enable", 0x04, 0x03),
AMX_ENABLE_CTRL_DECL("AMX4-4 Enable", 0x04, 0x04),

I2S_SET_RATE("I2S1 rate", 0x01),
I2S_SET_RATE("I2S2 rate", 0x02),
I2S_SET_RATE("I2S3 rate", 0x03),
I2S_SET_RATE("I2S4 rate", 0x04),
I2S_SET_RATE("I2S5 rate", 0x05),
I2S_SET_RATE("I2S6 rate", 0x06),

I2S_LOOPBACK_ENABLE_CTRL_DECL("I2S1 Loopback", 0x01),
I2S_LOOPBACK_ENABLE_CTRL_DECL("I2S2 Loopback", 0x02),
I2S_LOOPBACK_ENABLE_CTRL_DECL("I2S3 Loopback", 0x03),
I2S_LOOPBACK_ENABLE_CTRL_DECL("I2S4 Loopback", 0x04),
I2S_LOOPBACK_ENABLE_CTRL_DECL("I2S5 Loopback", 0x05),
I2S_LOOPBACK_ENABLE_CTRL_DECL("I2S6 Loopback", 0x06),

REGDUMP_CTRL_DECL("ADMAIF1 regdump", ADMAIF1, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF2 regdump", ADMAIF2, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF3 regdump", ADMAIF3, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF4 regdump", ADMAIF4, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF5 regdump", ADMAIF5, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF6 regdump", ADMAIF6, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF7 regdump", ADMAIF7, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF8 regdump", ADMAIF8, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF9 regdump", ADMAIF9, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF10 regdump", ADMAIF10, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF11 regdump", ADMAIF11, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF12 regdump", ADMAIF12, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF13 regdump", ADMAIF13, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF14 regdump", ADMAIF14, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF15 regdump", ADMAIF15, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF16 regdump", ADMAIF16, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF17 regdump", ADMAIF17, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF18 regdump", ADMAIF18, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF19 regdump", ADMAIF19, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADMAIF20 regdump", ADMAIF20, 0, NVAUDIO_REGDUMP_RX_TX),

REGDUMP_CTRL_DECL("AMX1 regdump", AMX1, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("AMX2 regdump", AMX2, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("AMX3 regdump", AMX3, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("AMX4 regdump", AMX4, 0, NVAUDIO_REGDUMP_RX_TX),

REGDUMP_CTRL_DECL("ADX1 regdump", ADX1, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADX2 regdump", ADX2, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADX3 regdump", ADX3, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ADX4 regdump", ADX4, 0, NVAUDIO_REGDUMP_RX_TX),

REGDUMP_CTRL_DECL("MIXER1-1 RX regdump", MIXER1, 0, NVAUDIO_REGDUMP_RX),
REGDUMP_CTRL_DECL("MIXER1-2 RX regdump", MIXER1, 1, NVAUDIO_REGDUMP_RX),
REGDUMP_CTRL_DECL("MIXER1-3 RX regdump", MIXER1, 2, NVAUDIO_REGDUMP_RX),
REGDUMP_CTRL_DECL("MIXER1-4 RX regdump", MIXER1, 3, NVAUDIO_REGDUMP_RX),
REGDUMP_CTRL_DECL("MIXER1-5 RX regdump", MIXER1, 4, NVAUDIO_REGDUMP_RX),
REGDUMP_CTRL_DECL("MIXER1-6 RX regdump", MIXER1, 5, NVAUDIO_REGDUMP_RX),
REGDUMP_CTRL_DECL("MIXER1-7 RX regdump", MIXER1, 6, NVAUDIO_REGDUMP_RX),
REGDUMP_CTRL_DECL("MIXER1-8 RX regdump", MIXER1, 7, NVAUDIO_REGDUMP_RX),
REGDUMP_CTRL_DECL("MIXER1-9 RX regdump", MIXER1, 8, NVAUDIO_REGDUMP_RX),
REGDUMP_CTRL_DECL("MIXER1-10 RX regdump", MIXER1, 9, NVAUDIO_REGDUMP_RX),

REGDUMP_CTRL_DECL("MIXER1-1 TX regdump", MIXER1, 0, NVAUDIO_REGDUMP_TX),
REGDUMP_CTRL_DECL("MIXER1-2 TX regdump", MIXER1, 1, NVAUDIO_REGDUMP_TX),
REGDUMP_CTRL_DECL("MIXER1-3 TX regdump", MIXER1, 2, NVAUDIO_REGDUMP_TX),
REGDUMP_CTRL_DECL("MIXER1-4 TX regdump", MIXER1, 3, NVAUDIO_REGDUMP_TX),
REGDUMP_CTRL_DECL("MIXER1-5 TX regdump", MIXER1, 4, NVAUDIO_REGDUMP_TX),

REGDUMP_CTRL_DECL("I2S1 regdump", I2S1, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("I2S2 regdump", I2S2, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("I2S3 regdump", I2S3, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("I2S4 regdump", I2S4, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("I2S5 regdump", I2S5, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("I2S6 regdump", I2S6, 0, NVAUDIO_REGDUMP_RX_TX),

REGDUMP_CTRL_DECL("ASRC1-1 regdump", ASRC1, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ASRC1-2 regdump", ASRC1, 1, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ASRC1-3 regdump", ASRC1, 2, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ASRC1-4 regdump", ASRC1, 3, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ASRC1-5 regdump", ASRC1, 4, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ASRC1-6 regdump", ASRC1, 5, NVAUDIO_REGDUMP_RX_TX),

REGDUMP_CTRL_DECL("SFC1 regdump", SFC1, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("SFC2 regdump", SFC2, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("SFC3 regdump", SFC3, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("SFC4 regdump", SFC4, 0, NVAUDIO_REGDUMP_RX_TX),

REGDUMP_CTRL_DECL("MVC1 regdump", MVC1, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("MVC2 regdump", MVC2, 0, NVAUDIO_REGDUMP_RX_TX),

REGDUMP_CTRL_DECL("ARAD1 Lane1 regdump", ARAD1, 0, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ARAD1 Lane2 regdump", ARAD1, 1, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ARAD1 Lane3 regdump", ARAD1, 2, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ARAD1 Lane4 regdump", ARAD1, 3, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ARAD1 Lane5 regdump", ARAD1, 4, NVAUDIO_REGDUMP_RX_TX),
REGDUMP_CTRL_DECL("ARAD1 Lane6 regdump", ARAD1, 5, NVAUDIO_REGDUMP_RX_TX),

ADMA_REGDUMP_CTRL_DECL("ADMA1 regdump", 1),
ADMA_REGDUMP_CTRL_DECL("ADMA2 regdump", 2),
ADMA_REGDUMP_CTRL_DECL("ADMA3 regdump", 3),
ADMA_REGDUMP_CTRL_DECL("ADMA4 regdump", 4),
ADMA_REGDUMP_CTRL_DECL("ADMA5 regdump", 5),
ADMA_REGDUMP_CTRL_DECL("ADMA6 regdump", 6),
ADMA_REGDUMP_CTRL_DECL("ADMA7 regdump", 7),
ADMA_REGDUMP_CTRL_DECL("ADMA8 regdump", 8),
ADMA_REGDUMP_CTRL_DECL("ADMA9 regdump", 9),
ADMA_REGDUMP_CTRL_DECL("ADMA10 regdump", 10),
ADMA_REGDUMP_CTRL_DECL("ADMA11 regdump", 11),
ADMA_REGDUMP_CTRL_DECL("ADMA12 regdump", 12),
ADMA_REGDUMP_CTRL_DECL("ADMA13 regdump", 13),
ADMA_REGDUMP_CTRL_DECL("ADMA14 regdump", 14),
ADMA_REGDUMP_CTRL_DECL("ADMA15 regdump", 15),
ADMA_REGDUMP_CTRL_DECL("ADMA16 regdump", 16),
ADMA_REGDUMP_CTRL_DECL("ADMA17 regdump", 17),
ADMA_REGDUMP_CTRL_DECL("ADMA18 regdump", 18),
ADMA_REGDUMP_CTRL_DECL("ADMA19 regdump", 19),
ADMA_REGDUMP_CTRL_DECL("ADMA20 regdump", 20),
ADMA_REGDUMP_CTRL_DECL("ADMA21 regdump", 21),
ADMA_REGDUMP_CTRL_DECL("ADMA22 regdump", 22),
ADMA_REGDUMP_CTRL_DECL("ADMA23 regdump", 23),
ADMA_REGDUMP_CTRL_DECL("ADMA24 regdump", 24),
ADMA_REGDUMP_CTRL_DECL("ADMA25 regdump", 25),
ADMA_REGDUMP_CTRL_DECL("ADMA26 regdump", 26),
ADMA_REGDUMP_CTRL_DECL("ADMA27 regdump", 27),
ADMA_REGDUMP_CTRL_DECL("ADMA28 regdump", 28),
ADMA_REGDUMP_CTRL_DECL("ADMA29 regdump", 29),
ADMA_REGDUMP_CTRL_DECL("ADMA30 regdump", 30),
ADMA_REGDUMP_CTRL_DECL("ADMA31 regdump", 31),
ADMA_REGDUMP_CTRL_DECL("ADMA32 regdump", 32),

/* Metadata controls should be always the last ones */
SOC_SINGLE_BOOL_EXT("SAD Init", 0,
		tegra_metadata_get_init, tegra_metadata_set_init),
SOC_SINGLE_BOOL_EXT("SAD Enable", 0,
		tegra_metadata_get_enable, tegra_metadata_set_enable),
METADATA_CTRL_DECL("SAD Metadata"),
};

static struct snd_soc_component_driver tegra210_admaif_dai_driver = {
	.name		= "tegra210-virt-pcm",
	.controls = tegra_virt_t186ref_controls,
	.num_controls = ARRAY_SIZE(tegra_virt_t186ref_controls),
};

int tegra210_virt_admaif_register_component(struct platform_device *pdev,
				struct tegra_virt_admaif_soc_data *data)
{
	int i = 0;
	int ret;
	int admaif_ch_num = 0;
	unsigned int admaif_ch_list[MAX_ADMAIF_IDS] = {0};
	struct tegra_virt_admaif_soc_data *soc_data = data;
	int adma_count = 0;
	bool meta_enabled = false;
	unsigned int buffer_size;

	admaif = devm_kzalloc(&pdev->dev, sizeof(*admaif), GFP_KERNEL);
	if (admaif == NULL) {
		ret = -ENOMEM;
		goto err;
	}
	admaif->client_data.hivc_client =
			nvaudio_ivc_alloc_ctxt(&pdev->dev);

	if (!admaif->client_data.hivc_client) {
		dev_err(&pdev->dev, "Failed to allocate IVC context\n");
		ret = -ENODEV;
		goto err;
	}

	admaif->capture_dma_data = devm_kzalloc(&pdev->dev,
			sizeof(struct tegra_alt_pcm_dma_params) *
				soc_data->num_ch,
			GFP_KERNEL);
	if (admaif->capture_dma_data == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	admaif->playback_dma_data = devm_kzalloc(&pdev->dev,
			sizeof(struct tegra_alt_pcm_dma_params) *
				soc_data->num_ch,
			GFP_KERNEL);
	if (admaif->playback_dma_data == NULL) {
		ret = -ENOMEM;
		goto err;
	}

	if (of_property_read_u32(pdev->dev.of_node,
				"admaif_ch_num", &admaif_ch_num)) {
		dev_err(&pdev->dev, "number of admaif channels is not set\n");
		return -EINVAL;
	}

	if (of_property_read_u32_array(pdev->dev.of_node,
						"admaif_ch_list",
						admaif_ch_list,
						admaif_ch_num)) {
		dev_err(&pdev->dev, "admaif_ch_list is not populated\n");
		return -EINVAL;
	}


	for (i = 0; i < soc_data->num_ch; i++) {
		if ((i + 1) != admaif_ch_list[adma_count])
			continue;
	if (of_device_is_compatible(pdev->dev.of_node,
		"nvidia,tegra186-virt-pcm")) {
		admaif->playback_dma_data[i].addr = TEGRA186_ADMAIF_BASE +
				TEGRA186_ADMAIF_XBAR_TX_FIFO_WRITE +
				(i * TEGRA186_ADMAIF_CHANNEL_REG_STRIDE);
		admaif->capture_dma_data[i].addr = TEGRA186_ADMAIF_BASE +
				TEGRA186_ADMAIF_XBAR_RX_FIFO_READ +
				(i * TEGRA186_ADMAIF_CHANNEL_REG_STRIDE);
		} else if (of_device_is_compatible(pdev->dev.of_node,
		"nvidia,tegra210-virt-pcm")) {
		admaif->playback_dma_data[i].addr = TEGRA210_ADMAIF_BASE +
				TEGRA210_ADMAIF_XBAR_TX_FIFO_WRITE +
				(i * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE);
		admaif->capture_dma_data[i].addr = TEGRA210_ADMAIF_BASE +
				TEGRA210_ADMAIF_XBAR_RX_FIFO_READ +
				(i * TEGRA210_ADMAIF_CHANNEL_REG_STRIDE);
		} else {
			dev_err(&pdev->dev,
				"Uncompatible device driver\n");
			ret = -ENODEV;
			goto err;
		}

		buffer_size = 0;
		if (of_property_read_u32_index(pdev->dev.of_node,
				"dma-buffer-size",
				(i * 2) + 1,
				&buffer_size) < 0)
			dev_dbg(&pdev->dev,
				"Missing property nvidia,dma-buffer-size\n");
		admaif->playback_dma_data[i].buffer_size = buffer_size;
		admaif->playback_dma_data[i].width = 32;
		admaif->playback_dma_data[i].req_sel = i + 1;

		if (of_property_read_string_index(pdev->dev.of_node,
				"dma-names",
				(adma_count * 2) + 1,
				&admaif->playback_dma_data[i].chan_name) < 0) {
			dev_err(&pdev->dev,
				"Missing property nvidia,dma-names\n");
			ret = -ENODEV;
			goto err;
		}

		buffer_size = 0;
		if (of_property_read_u32_index(pdev->dev.of_node,
				"dma-buffer-size",
				(i * 2),
				&buffer_size) < 0)
			dev_dbg(&pdev->dev,
				"Missing property nvidia,dma-buffer-size\n");
		admaif->capture_dma_data[i].buffer_size = buffer_size;
		admaif->capture_dma_data[i].width = 32;
		admaif->capture_dma_data[i].req_sel = i + 1;
		if (of_property_read_string_index(pdev->dev.of_node,
				"dma-names",
				(adma_count * 2),
				&admaif->capture_dma_data[i].chan_name) < 0) {
			dev_err(&pdev->dev,
				"Missing property nvidia,dma-names\n");
			ret = -ENODEV;
			goto err;
		}
		adma_count++;
	}

	/* Remove exposing metadata controls if not enabled in device node */
	meta_enabled = of_property_read_bool(pdev->dev.of_node,
		"sad_enabled");
	if (!meta_enabled) {
		tegra210_admaif_dai_driver.num_controls =
		ARRAY_SIZE(tegra_virt_t186ref_controls) - NUM_META_CONTROLS;
	}

	ret = snd_soc_register_component(&pdev->dev,
					&tegra210_admaif_dai_driver,
					tegra210_admaif_dais,
					soc_data->num_ch);
	if (ret) {
		dev_err(&pdev->dev, "Could not register DAIs %d: %d\n",
			i, ret);
		goto err;
	}

	ret = tegra_alt_pcm_platform_register(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "Could not register PCM: %d\n", ret);
		goto err_unregister_dais;
	}

	return 0;
err_unregister_dais:
	snd_soc_unregister_component(&pdev->dev);
err:
	return ret;
}
EXPORT_SYMBOL_GPL(tegra210_virt_admaif_register_component);

void tegra210_virt_admaif_unregister_component(struct platform_device *pdev)
{
	tegra_alt_pcm_platform_unregister(&pdev->dev);
	snd_soc_unregister_component(&pdev->dev);
}
EXPORT_SYMBOL_GPL(tegra210_virt_admaif_unregister_component);

MODULE_LICENSE("GPL");
