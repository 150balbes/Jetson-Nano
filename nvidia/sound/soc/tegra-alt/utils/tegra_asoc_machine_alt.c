/*
 * tegra_asoc_machine_alt.c - Tegra xbar dai link for machine drivers
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
#include <linux/dma-mapping.h>
#include <linux/of.h>
#include <linux/version.h>
#include <sound/jack.h>
#include <sound/soc.h>

#include "tegra_asoc_machine_alt.h"
#include "tegra_asoc_utils_alt.h"

#define MAX_STR_SIZE		20

static struct snd_soc_dai_link *tegra_asoc_machine_links;
static struct snd_soc_codec_conf *tegra_asoc_codec_conf;
static unsigned int *tx_mask;
static unsigned int *rx_mask;
static unsigned int num_dai_links;

/* used by only t18x specific APIs */
static int num_links = TEGRA186_XBAR_DAI_LINKS;

static struct snd_soc_pcm_stream default_link_params = {
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};

static struct snd_soc_pcm_stream tdm_link_params = {
	.formats = SNDRV_PCM_FMTBIT_S32_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 8,
	.channels_max = 8,
};

__maybe_unused static struct snd_soc_pcm_stream arad_link_params = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};

static const char * const bit_format[] = {
	"s8", "u8", "s16_le", "s16_be",
	"u16_le", "u16_be", "s24_le", "s24_be",
	"u24_le", "u24_be", "s32_le", "s32_be",
	"u32_le", "u32_be", "float_le", "float_be",

	"float64_le", "float64_be", "iec958_subframe_le", "iec958_subframe_be",
	"mu_law", "a_law", "ima_adpcm", "mpeg",
	"gsm", "", "", "",
	"", "", "", "special",

	"s24_3l", "s24_3be", "u24_3le", "u24_3be",
	"s20_3le", "s20_3be", "u20_3le", "u20_3be",
	"s18_3le", "s18_3b", "u18_3le", "u18_3be",
	"g723_24", "g723_24_1b", "g723_40", "g723_40_1b",

	"dsd_u8", "dsd_u16_le",
};

struct snd_soc_dai_link tegra210_xbar_dai_links[TEGRA210_XBAR_DAI_LINKS] = {
	[TEGRA210_DAI_LINK_ADMAIF1] = {
		.name = "ADMAIF1 CIF",
		.stream_name = "ADMAIF1 CIF",
		.cpu_dai_name = "ADMAIF1",
		.codec_dai_name = "ADMAIF1",
		.cpu_name = "tegra210-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA210_DAI_LINK_ADMAIF2] = {
		.name = "ADMAIF2 CIF",
		.stream_name = "ADMAIF2 CIF",
		.cpu_dai_name = "ADMAIF2",
		.codec_dai_name = "ADMAIF2",
		.cpu_name = "tegra210-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA210_DAI_LINK_ADMAIF3] = {
		.name = "ADMAIF3 CIF",
		.stream_name = "ADMAIF3 CIF",
		.cpu_dai_name = "ADMAIF3",
		.codec_dai_name = "ADMAIF3",
		.cpu_name = "tegra210-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA210_DAI_LINK_ADMAIF4] = {
		.name = "ADMAIF4 CIF",
		.stream_name = "ADMAIF4 CIF",
		.cpu_dai_name = "ADMAIF4",
		.codec_dai_name = "ADMAIF4",
		.cpu_name = "tegra210-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA210_DAI_LINK_ADMAIF5] = {
		.name = "ADMAIF5 CIF",
		.stream_name = "ADMAIF5 CIF",
		.cpu_dai_name = "ADMAIF5",
		.codec_dai_name = "ADMAIF5",
		.cpu_name = "tegra210-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA210_DAI_LINK_ADMAIF6] = {
		.name = "ADMAIF6 CIF",
		.stream_name = "ADMAIF6 CIF",
		.cpu_dai_name = "ADMAIF6",
		.codec_dai_name = "ADMAIF6",
		.cpu_name = "tegra210-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA210_DAI_LINK_ADMAIF7] = {
		.name = "ADMAIF7 CIF",
		.stream_name = "ADMAIF7 CIF",
		.cpu_dai_name = "ADMAIF7",
		.codec_dai_name = "ADMAIF7",
		.cpu_name = "tegra210-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA210_DAI_LINK_ADMAIF8] = {
		.name = "ADMAIF8 CIF",
		.stream_name = "ADMAIF8 CIF",
		.cpu_dai_name = "ADMAIF8",
		.codec_dai_name = "ADMAIF8",
		.cpu_name = "tegra210-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA210_DAI_LINK_ADMAIF9] = {
		.name = "ADMAIF9 CIF",
		.stream_name = "ADMAIF9 CIF",
		.cpu_dai_name = "ADMAIF9",
		.codec_dai_name = "ADMAIF9",
		.cpu_name = "tegra210-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA210_DAI_LINK_ADMAIF10] = {
		.name = "ADMAIF10 CIF",
		.stream_name = "ADMAIF10 CIF",
		.cpu_dai_name = "ADMAIF10",
		.codec_dai_name = "ADMAIF10",
		.cpu_name = "tegra210-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
	},
#if IS_ENABLED(CONFIG_SND_SOC_TEGRA210_AMX_ALT)
	[TEGRA210_DAI_LINK_AMX1_1] = {
		.name = "AMX1 IN1",
		.stream_name = "AMX1 IN",
		.cpu_dai_name = "AMX1-1",
		.codec_dai_name = "IN1",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.0",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_AMX1_2] = {
		.name = "AMX1 IN2",
		.stream_name = "AMX1 IN",
		.cpu_dai_name = "AMX1-2",
		.codec_dai_name = "IN2",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.0",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_AMX1_3] = {
		.name = "AMX1 IN3",
		.stream_name = "AMX1 IN",
		.cpu_dai_name = "AMX1-3",
		.codec_dai_name = "IN3",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.0",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_AMX1_4] = {
		.name = "AMX1 IN4",
		.stream_name = "AMX1 IN",
		.cpu_dai_name = "AMX1-4",
		.codec_dai_name = "IN4",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.0",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_AMX1] = {
		.name = "AMX1 CIF",
		.stream_name = "AMX1 CIF",
		.cpu_dai_name = "OUT",
		.codec_dai_name = "AMX1",
		.cpu_name = "tegra210-amx.0",
		.codec_name = "tegra210-axbar",
		.params = &tdm_link_params,
	},
	[TEGRA210_DAI_LINK_AMX2_1] = {
		.name = "AMX2 IN1",
		.stream_name = "AMX2 IN",
		.cpu_dai_name = "AMX2-1",
		.codec_dai_name = "IN1",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.1",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_AMX2_2] = {
		.name = "AMX2 IN2",
		.stream_name = "AMX2 IN",
		.cpu_dai_name = "AMX2-2",
		.codec_dai_name = "IN2",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.1",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_AMX2_3] = {
		.name = "AMX2 IN3",
		.stream_name = "AMX2 IN",
		.cpu_dai_name = "AMX2-3",
		.codec_dai_name = "IN3",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.1",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_AMX2_4] = {
		.name = "AMX2 IN4",
		.stream_name = "AMX2 IN",
		.cpu_dai_name = "AMX2-4",
		.codec_dai_name = "IN4",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.1",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_AMX2] = {
		.name = "AMX2 CIF",
		.stream_name = "AMX2 CIF",
		.cpu_dai_name = "OUT",
		.codec_dai_name = "AMX2",
		.cpu_name = "tegra210-amx.1",
		.codec_name = "tegra210-axbar",
		.params = &tdm_link_params,
	},
#endif /* IS_ENABLED(CONFIG_SND_SOC_TEGRA210_AMX_ALT) */
#if IS_ENABLED(CONFIG_SND_SOC_TEGRA210_ADX_ALT)
	[TEGRA210_DAI_LINK_ADX1] = {
		.name = "ADX1 CIF",
		.stream_name = "ADX1 IN",
		.cpu_dai_name = "ADX1",
		.codec_dai_name = "IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-adx.0",
		.params = &tdm_link_params,
	},
	[TEGRA210_DAI_LINK_ADX1_1] = {
		.name = "ADX1 OUT1",
		.stream_name = "ADX1 OUT",
		.cpu_dai_name = "OUT1",
		.codec_dai_name = "ADX1-1",
		.cpu_name = "tegra210-adx.0",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADX1_2] = {
		.name = "ADX1 OUT2",
		.stream_name = "ADX1 OUT",
		.cpu_dai_name = "OUT2",
		.codec_dai_name = "ADX1-2",
		.cpu_name = "tegra210-adx.0",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADX1_3] = {
		.name = "ADX1 OUT3",
		.stream_name = "ADX1 OUT",
		.cpu_dai_name = "OUT3",
		.codec_dai_name = "ADX1-3",
		.cpu_name = "tegra210-adx.0",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADX1_4] = {
		.name = "ADX1 OUT4",
		.stream_name = "ADX1 OUT",
		.cpu_dai_name = "OUT4",
		.codec_dai_name = "ADX1-4",
		.cpu_name = "tegra210-adx.0",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADX2] = {
		.name = "ADX2 CIF",
		.stream_name = "ADX2 IN",
		.cpu_dai_name = "ADX2",
		.codec_dai_name = "IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-adx.1",
		.params = &tdm_link_params,
	},
	[TEGRA210_DAI_LINK_ADX2_1] = {
		.name = "ADX2 OUT1",
		.stream_name = "ADX2 OUT",
		.cpu_dai_name = "OUT1",
		.codec_dai_name = "ADX2-1",
		.cpu_name = "tegra210-adx.1",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADX2_2] = {
		.name = "ADX2 OUT2",
		.stream_name = "ADX2 OUT",
		.cpu_dai_name = "OUT2",
		.codec_dai_name = "ADX2-2",
		.cpu_name = "tegra210-adx.1",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADX2_3] = {
		.name = "ADX2 OUT3",
		.stream_name = "ADX2 OUT",
		.cpu_dai_name = "OUT3",
		.codec_dai_name = "ADX2-3",
		.cpu_name = "tegra210-adx.1",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADX2_4] = {
		.name = "ADX2 OUT4",
		.stream_name = "ADX2 OUT",
		.cpu_dai_name = "OUT4",
		.codec_dai_name = "ADX2-4",
		.cpu_name = "tegra210-adx.1",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
#endif /* IS_ENABLED(CONFIG_SND_SOC_TEGRA210_ADX_ALT) */
#if IS_ENABLED(CONFIG_SND_SOC_TEGRA210_MIXER_ALT)
	[TEGRA210_DAI_LINK_MIXER1_RX1] = {
		.name = "MIXER1 RX1",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-1",
		.codec_dai_name = "RX1",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_MIXER1_RX2] = {
		.name = "MIXER1 RX2",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-2",
		.codec_dai_name = "RX2",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_MIXER1_RX3] = {
		.name = "MIXER1 RX3",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-3",
		.codec_dai_name = "RX3",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_MIXER1_RX4] = {
		.name = "MIXER1 RX4",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-4",
		.codec_dai_name = "RX4",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_MIXER1_RX5] = {
		.name = "MIXER1 RX5",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-5",
		.codec_dai_name = "RX5",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_MIXER1_RX6] = {
		.name = "MIXER1 RX6",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-6",
		.codec_dai_name = "RX6",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_MIXER1_RX7] = {
		.name = "MIXER1 RX7",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-7",
		.codec_dai_name = "RX7",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_MIXER1_RX8] = {
		.name = "MIXER1 RX8",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-8",
		.codec_dai_name = "RX8",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_MIXER1_RX9] = {
		.name = "MIXER1 RX9",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-9",
		.codec_dai_name = "RX9",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_MIXER1_RX10] = {
		.name = "MIXER1 RX10",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-10",
		.codec_dai_name = "RX10",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_MIXER1_TX1] = {
		.name = "MIXER1 TX1",
		.stream_name = "MIXER1 TX",
		.cpu_dai_name = "TX1",
		.codec_dai_name = "MIXER1-1",
		.cpu_name = "tegra210-mixer",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_MIXER1_TX2] = {
		.name = "MIXER1 TX2",
		.stream_name = "MIXER1 TX",
		.cpu_dai_name = "TX2",
		.codec_dai_name = "MIXER1-2",
		.cpu_name = "tegra210-mixer",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_MIXER1_TX3] = {
		.name = "MIXER1 TX3",
		.stream_name = "MIXER1 TX",
		.cpu_dai_name = "TX3",
		.codec_dai_name = "MIXER1-3",
		.cpu_name = "tegra210-mixer",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_MIXER1_TX4] = {
		.name = "MIXER1 TX4",
		.stream_name = "MIXER1 TX",
		.cpu_dai_name = "TX4",
		.codec_dai_name = "MIXER1-4",
		.cpu_name = "tegra210-mixer",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_MIXER1_TX5] = {
		.name = "MIXER1 TX5",
		.stream_name = "MIXER1 TX",
		.cpu_dai_name = "TX5",
		.codec_dai_name = "MIXER1-5",
		.cpu_name = "tegra210-mixer",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
#endif /* IS_ENABLED(CONFIG_SND_SOC_TEGRA210_MIXER_ALT) */
#if IS_ENABLED(CONFIG_SND_SOC_TEGRA210_SFC_ALT)
	[TEGRA210_DAI_LINK_SFC1_RX] = {
		.name = "SFC1 RX",
		.stream_name = "SFC1 RX",
		.cpu_dai_name = "SFC1",
		.codec_dai_name = "CIF",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-sfc.0",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_SFC1_TX] = {
		.name = "SFC1 TX",
		.stream_name = "SFC1 TX",
		.cpu_dai_name = "DAP",
		.codec_dai_name = "SFC1",
		.cpu_name = "tegra210-sfc.0",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_SFC2_RX] = {
		.name = "SFC2 RX",
		.stream_name = "SFC2 RX",
		.cpu_dai_name = "SFC2",
		.codec_dai_name = "CIF",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-sfc.1",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_SFC2_TX] = {
		.name = "SFC2 TX",
		.stream_name = "SFC2 TX",
		.cpu_dai_name = "DAP",
		.codec_dai_name = "SFC2",
		.cpu_name = "tegra210-sfc.1",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_SFC3_RX] = {
		.name = "SFC3 RX",
		.stream_name = "SFC3 RX",
		.cpu_dai_name = "SFC3",
		.codec_dai_name = "CIF",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-sfc.2",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_SFC3_TX] = {
		.name = "SFC3 TX",
		.stream_name = "SFC3 TX",
		.cpu_dai_name = "DAP",
		.codec_dai_name = "SFC3",
		.cpu_name = "tegra210-sfc.2",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_SFC4_RX] = {
		.name = "SFC4 RX",
		.stream_name = "SFC4 RX",
		.cpu_dai_name = "SFC4",
		.codec_dai_name = "CIF",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-sfc.3",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_SFC4_TX] = {
		.name = "SFC4 TX",
		.stream_name = "SFC4 TX",
		.cpu_dai_name = "DAP",
		.codec_dai_name = "SFC4",
		.cpu_name = "tegra210-sfc.3",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
#endif /* IS_ENABLED(CONFIG_SND_SOC_TEGRA210_SFC_ALT) */
#if IS_ENABLED(CONFIG_SND_SOC_TEGRA210_AFC_ALT)
	[TEGRA210_DAI_LINK_AFC1_RX] = {
		.name = "AFC1 RX",
		.stream_name = "AFC1 RX",
		.cpu_dai_name = "AFC1",
		.codec_dai_name = "AFC IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-afc.0",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_AFC1_TX] = {
		.name = "AFC1 TX",
		.stream_name = "AFC1 TX",
		.cpu_dai_name = "AFC OUT",
		.codec_dai_name = "AFC1",
		.cpu_name = "tegra210-afc.0",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_AFC2_RX] = {
		.name = "AFC2 RX",
		.stream_name = "AFC2 RX",
		.cpu_dai_name = "AFC2",
		.codec_dai_name = "AFC IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-afc.1",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_AFC2_TX] = {
		.name = "AFC2 TX",
		.stream_name = "AFC2 TX",
		.cpu_dai_name = "AFC OUT",
		.codec_dai_name = "AFC2",
		.cpu_name = "tegra210-afc.1",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_AFC3_RX] = {
		.name = "AFC3 RX",
		.stream_name = "AFC3 RX",
		.cpu_dai_name = "AFC3",
		.codec_dai_name = "AFC IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-afc.2",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_AFC3_TX] = {
		.name = "AFC3 TX",
		.stream_name = "AFC3 TX",
		.cpu_dai_name = "AFC OUT",
		.codec_dai_name = "AFC3",
		.cpu_name = "tegra210-afc.2",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_AFC4_RX] = {
		.name = "AFC4 RX",
		.stream_name = "AFC4 RX",
		.cpu_dai_name = "AFC4",
		.codec_dai_name = "AFC IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-afc.3",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_AFC4_TX] = {
		.name = "AFC4 TX",
		.stream_name = "AFC4 TX",
		.cpu_dai_name = "AFC OUT",
		.codec_dai_name = "AFC4",
		.cpu_name = "tegra210-afc.3",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_AFC5_RX] = {
		.name = "AFC5 RX",
		.stream_name = "AFC5 RX",
		.cpu_dai_name = "AFC5",
		.codec_dai_name = "AFC IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-afc.4",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_AFC5_TX] = {
		.name = "AFC5 TX",
		.stream_name = "AFC5 TX",
		.cpu_dai_name = "AFC OUT",
		.codec_dai_name = "AFC5",
		.cpu_name = "tegra210-afc.4",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_AFC6_RX] = {
		.name = "AFC6 RX",
		.stream_name = "AFC6 RX",
		.cpu_dai_name = "AFC6",
		.codec_dai_name = "AFC IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-afc.5",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_AFC6_TX] = {
		.name = "AFC6 TX",
		.stream_name = "AFC6 TX",
		.cpu_dai_name = "AFC OUT",
		.codec_dai_name = "AFC6",
		.cpu_name = "tegra210-afc.5",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
#endif /* IS_ENABLED(CONFIG_SND_SOC_TEGRA210_AFC_ALT) */
#if IS_ENABLED(CONFIG_SND_SOC_TEGRA210_MVC_ALT)
	[TEGRA210_DAI_LINK_MVC1_RX] = {
		.name = "MVC1 RX",
		.stream_name = "MVC1 RX",
		.cpu_dai_name = "MVC1",
		.codec_dai_name = "MVC IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mvc.0",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_MVC1_TX] = {
		.name = "MVC1 TX",
		.stream_name = "MVC1 TX",
		.cpu_dai_name = "MVC OUT",
		.codec_dai_name = "MVC1",
		.cpu_name = "tegra210-mvc.0",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_MVC2_RX] = {
		.name = "MVC2 RX",
		.stream_name = "MVC2 RX",
		.cpu_dai_name = "MVC2",
		.codec_dai_name = "MVC IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mvc.1",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_MVC2_TX] = {
		.name = "MVC2 TX",
		.stream_name = "AFC2 TX",
		.cpu_dai_name = "MVC OUT",
		.codec_dai_name = "MVC2",
		.cpu_name = "tegra210-mvc.1",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
#endif /* IS_ENABLED(CONFIG_SND_SOC_TEGRA210_MVC_ALT) */
#if IS_ENABLED(CONFIG_SND_SOC_TEGRA210_OPE_ALT)
	[TEGRA210_DAI_LINK_OPE1_RX] = {
		.name = "OPE1 RX",
		.stream_name = "OPE1 RX",
		.cpu_dai_name = "OPE1",
		.codec_dai_name = "OPE IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-ope.0",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_OPE1_TX] = {
		.name = "OPE1 TX",
		.stream_name = "OPE1 TX",
		.cpu_dai_name = "OPE OUT",
		.codec_dai_name = "OPE1",
		.cpu_name = "tegra210-ope.0",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_OPE2_RX] = {
		.name = "OPE2 RX",
		.stream_name = "OPE2 RX",
		.cpu_dai_name = "OPE2",
		.codec_dai_name = "OPE IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-ope.1",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_OPE2_TX] = {
		.name = "OPE2 TX",
		.stream_name = "OPE2 TX",
		.cpu_dai_name = "OPE OUT",
		.codec_dai_name = "OPE2",
		.cpu_name = "tegra210-ope.1",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
	},
#endif /* IS_ENABLED(CONFIG_SND_SOC_TEGRA210_OPE_ALT) */
	[TEGRA210_DAI_LINK_ADMAIF1_CODEC] = {
		.name = "ADMAIF1 CODEC",
		.stream_name = "ADMAIF1 CODEC",
		.cpu_dai_name = "ADMAIF1 CIF",
		.codec_dai_name = "ADMAIF1",
		.cpu_name = "tegra210-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADMAIF2_CODEC] = {
		.name = "ADMAIF2 CODEC",
		.stream_name = "ADMAIF2 CODEC",
		.cpu_dai_name = "ADMAIF2 CIF",
		.codec_dai_name = "ADMAIF2",
		.cpu_name = "tegra210-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADMAIF3_CODEC] = {
		.name = "ADMAIF3 CODEC",
		.stream_name = "ADMAIF3 CODEC",
		.cpu_dai_name = "ADMAIF3 CIF",
		.codec_dai_name = "ADMAIF3",
		.cpu_name = "tegra210-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADMAIF4_CODEC] = {
		.name = "ADMAIF4 CODEC",
		.stream_name = "ADMAIF4 CODEC",
		.cpu_dai_name = "ADMAIF4 CIF",
		.codec_dai_name = "ADMAIF4",
		.cpu_name = "tegra210-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADMAIF5_CODEC] = {
		.name = "ADMAIF5 CODEC",
		.stream_name = "ADMAIF5 CODEC",
		.cpu_dai_name = "ADMAIF5 CIF",
		.codec_dai_name = "ADMAIF5",
		.cpu_name = "tegra210-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADMAIF6_CODEC] = {
		.name = "ADMAIF6 CODEC",
		.stream_name = "ADMAIF6 CODEC",
		.cpu_dai_name = "ADMAIF6 CIF",
		.codec_dai_name = "ADMAIF6",
		.cpu_name = "tegra210-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADMAIF7_CODEC] = {
		.name = "ADMAIF7 CODEC",
		.stream_name = "ADMAIF7 CODEC",
		.cpu_dai_name = "ADMAIF7 CIF",
		.codec_dai_name = "ADMAIF7",
		.cpu_name = "tegra210-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADMAIF8_CODEC] = {
		.name = "ADMAIF8 CODEC",
		.stream_name = "ADMAIF8 CODEC",
		.cpu_dai_name = "ADMAIF8 CIF",
		.codec_dai_name = "ADMAIF8",
		.cpu_name = "tegra210-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADMAIF9_CODEC] = {
		.name = "ADMAIF9 CODEC",
		.stream_name = "ADMAIF9 CODEC",
		.cpu_dai_name = "ADMAIF9 CIF",
		.codec_dai_name = "ADMAIF9",
		.cpu_name = "tegra210-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADMAIF10_CODEC] = {
		.name = "ADMAIF10 CODEC",
		.stream_name = "ADMAIF10 CODEC",
		.cpu_dai_name = "ADMAIF10 CIF",
		.codec_dai_name = "ADMAIF10",
		.cpu_name = "tegra210-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra210-admaif",
		.ignore_pmdown_time = 1,
		.params = &default_link_params,
	},
#if IS_ENABLED(CONFIG_SND_SOC_TEGRA210_ADSP_ALT)
	[TEGRA210_DAI_LINK_ADSP_ADMAIF1] = {
		.name = "ADSP ADMAIF1",
		.stream_name = "ADSP ADMAIF1",
		.cpu_dai_name = "ADSP-ADMAIF1",
		.codec_dai_name = "ADMAIF1 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra210-admaif",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADSP_ADMAIF2] = {
		.name = "ADSP ADMAIF2",
		.stream_name = "ADSP ADMAIF2",
		.cpu_dai_name = "ADSP-ADMAIF2",
		.codec_dai_name = "ADMAIF2 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra210-admaif",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADSP_ADMAIF3] = {
		.name = "ADSP ADMAIF3",
		.stream_name = "ADSP ADMAIF3",
		.cpu_dai_name = "ADSP-ADMAIF3",
		.codec_dai_name = "ADMAIF3 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra210-admaif",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADSP_ADMAIF4] = {
		.name = "ADSP ADMAIF4",
		.stream_name = "ADSP ADMAIF4",
		.cpu_dai_name = "ADSP-ADMAIF4",
		.codec_dai_name = "ADMAIF4 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra210-admaif",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADSP_ADMAIF5] = {
		.name = "ADSP ADMAIF5",
		.stream_name = "ADSP ADMAIF5",
		.cpu_dai_name = "ADSP-ADMAIF5",
		.codec_dai_name = "ADMAIF5 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra210-admaif",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADSP_ADMAIF6] = {
		.name = "ADSP ADMAIF6",
		.stream_name = "ADSP ADMAIF6",
		.cpu_dai_name = "ADSP-ADMAIF6",
		.codec_dai_name = "ADMAIF6 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra210-admaif",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADSP_ADMAIF7] = {
		.name = "ADSP ADMAIF7",
		.stream_name = "ADSP ADMAIF7",
		.cpu_dai_name = "ADSP-ADMAIF7",
		.codec_dai_name = "ADMAIF7 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra210-admaif",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADSP_ADMAIF8] = {
		.name = "ADSP ADMAIF8",
		.stream_name = "ADSP ADMAIF8",
		.cpu_dai_name = "ADSP-ADMAIF8",
		.codec_dai_name = "ADMAIF8 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra210-admaif",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADSP_ADMAIF9] = {
		.name = "ADSP ADMAIF9",
		.stream_name = "ADSP ADMAIF9",
		.cpu_dai_name = "ADSP-ADMAIF9",
		.codec_dai_name = "ADMAIF9 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra210-admaif",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADSP_ADMAIF10] = {
		.name = "ADSP ADMAIF10",
		.stream_name = "ADSP ADMAIF10",
		.cpu_dai_name = "ADSP-ADMAIF10",
		.codec_dai_name = "ADMAIF10 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra210-admaif",
		.params = &default_link_params,
	},
	[TEGRA210_DAI_LINK_ADSP_PCM1] = {
		.name = "ADSP PCM1",
		.stream_name = "ADSP PCM1",
		.cpu_dai_name = "ADSP PCM1",
		.codec_dai_name = "ADSP-FE1",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra210-adsp",
		.platform_name = "tegra210-adsp",
		.ignore_pmdown_time = 1,
	},
	[TEGRA210_DAI_LINK_ADSP_PCM2] = {
		.name = "ADSP PCM2",
		.stream_name = "ADSP PCM2",
		.cpu_dai_name = "ADSP PCM2",
		.codec_dai_name = "ADSP-FE2",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra210-adsp",
		.platform_name = "tegra210-adsp",
		.ignore_pmdown_time = 1,
	},
	[TEGRA210_DAI_LINK_ADSP_COMPR1] = {
		.name = "ADSP COMPR1",
		.stream_name = "ADSP COMPR1",
		.cpu_dai_name = "ADSP COMPR1",
		.codec_dai_name = "ADSP-FE3",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra210-adsp",
		.platform_name = "tegra210-adsp",
		.ignore_pmdown_time = 1,
	},
	[TEGRA210_DAI_LINK_ADSP_COMPR2] = {
		.name = "ADSP COMPR2",
		.stream_name = "ADSP COMPR2",
		.cpu_dai_name = "ADSP COMPR2",
		.codec_dai_name = "ADSP-FE4",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra210-adsp",
		.platform_name = "tegra210-adsp",
		.ignore_pmdown_time = 1,
	},
#endif
};
EXPORT_SYMBOL_GPL(tegra210_xbar_dai_links);

struct snd_soc_codec_conf tegra210_xbar_codec_conf[TEGRA210_XBAR_CODEC_CONF] = {
	[TEGRA210_CODEC_AMX1_CONF] = {
		.dev_name = "tegra210-amx.0",
		.name_prefix = "AMX1",
	},
	[TEGRA210_CODEC_AMX2_CONF] = {
		.dev_name = "tegra210-amx.1",
		.name_prefix = "AMX2",
	},
	[TEGRA210_CODEC_ADX1_CONF] = {
		.dev_name = "tegra210-adx.0",
		.name_prefix = "ADX1",
	},
	[TEGRA210_CODEC_ADX2_CONF] = {
		.dev_name = "tegra210-adx.1",
		.name_prefix = "ADX2",
	},
	[TEGRA210_CODEC_SFC1_CONF] = {
		.dev_name = "tegra210-sfc.0",
		.name_prefix = "SFC1",
	},
	[TEGRA210_CODEC_SFC2_CONF] = {
		.dev_name = "tegra210-sfc.1",
		.name_prefix = "SFC2",
	},
	[TEGRA210_CODEC_SFC3_CONF] = {
		.dev_name = "tegra210-sfc.2",
		.name_prefix = "SFC3",
	},
	[TEGRA210_CODEC_SFC4_CONF] = {
		.dev_name = "tegra210-sfc.3",
		.name_prefix = "SFC4",
	},
	[TEGRA210_CODEC_MVC1_CONF] = {
		.dev_name = "tegra210-mvc.0",
		.name_prefix = "MVC1",
	},
	[TEGRA210_CODEC_MVC2_CONF] = {
		.dev_name = "tegra210-mvc.1",
		.name_prefix = "MVC2",
	},
	[TEGRA210_CODEC_OPE1_CONF] = {
		.dev_name = "tegra210-ope.0",
		.name_prefix = "OPE1",
	},
	[TEGRA210_CODEC_OPE2_CONF] = {
		.dev_name = "tegra210-ope.1",
		.name_prefix = "OPE2",
	},
	[TEGRA210_CODEC_AFC1_CONF] = {
		.dev_name = "tegra210-afc.0",
		.name_prefix = "AFC1",
	},
	[TEGRA210_CODEC_AFC2_CONF] = {
		.dev_name = "tegra210-afc.1",
		.name_prefix = "AFC2",
	},
	[TEGRA210_CODEC_AFC3_CONF] = {
		.dev_name = "tegra210-afc.2",
		.name_prefix = "AFC3",
	},
	[TEGRA210_CODEC_AFC4_CONF] = {
		.dev_name = "tegra210-afc.3",
		.name_prefix = "AFC4",
	},
	[TEGRA210_CODEC_AFC5_CONF] = {
		.dev_name = "tegra210-afc.4",
		.name_prefix = "AFC5",
	},
	[TEGRA210_CODEC_AFC6_CONF] = {
		.dev_name = "tegra210-afc.5",
		.name_prefix = "AFC6",
	},
	[TEGRA210_CODEC_I2S1_CONF] = {
		.dev_name = "tegra210-i2s.0",
		.name_prefix = "I2S1",
	},
	[TEGRA210_CODEC_I2S2_CONF] = {
		.dev_name = "tegra210-i2s.1",
		.name_prefix = "I2S2",
	},
	[TEGRA210_CODEC_I2S3_CONF] = {
		.dev_name = "tegra210-i2s.2",
		.name_prefix = "I2S3",
	},
	[TEGRA210_CODEC_I2S4_CONF] = {
		.dev_name = "tegra210-i2s.3",
		.name_prefix = "I2S4",
	},
	[TEGRA210_CODEC_I2S5_CONF] = {
		.dev_name = "tegra210-i2s.4",
		.name_prefix = "I2S5",
	},
	[TEGRA210_CODEC_DMIC1_CONF] = {
		.dev_name = "tegra210-dmic.0",
		.name_prefix = "DMIC1",
	},
	[TEGRA210_CODEC_DMIC2_CONF] = {
		.dev_name = "tegra210-dmic.1",
		.name_prefix = "DMIC2",
	},
	[TEGRA210_CODEC_DMIC3_CONF] = {
		.dev_name = "tegra210-dmic.2",
		.name_prefix = "DMIC3",
	},
};
EXPORT_SYMBOL_GPL(tegra210_xbar_codec_conf);

struct snd_soc_dai_link tegra186_xbar_dai_links[TEGRA186_XBAR_DAI_LINKS] = {
	[TEGRA186_DAI_LINK_ADMAIF1] = {
		.name = "ADMAIF1 CIF",
		.stream_name = "ADMAIF1 CIF",
		.cpu_dai_name = "ADMAIF1",
		.codec_dai_name = "ADMAIF1",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF2] = {
		.name = "ADMAIF2 CIF",
		.stream_name = "ADMAIF2 CIF",
		.cpu_dai_name = "ADMAIF2",
		.codec_dai_name = "ADMAIF2",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF3] = {
		.name = "ADMAIF3 CIF",
		.stream_name = "ADMAIF3 CIF",
		.cpu_dai_name = "ADMAIF3",
		.codec_dai_name = "ADMAIF3",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF4] = {
		.name = "ADMAIF4 CIF",
		.stream_name = "ADMAIF4 CIF",
		.cpu_dai_name = "ADMAIF4",
		.codec_dai_name = "ADMAIF4",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF5] = {
		.name = "ADMAIF5 CIF",
		.stream_name = "ADMAIF5 CIF",
		.cpu_dai_name = "ADMAIF5",
		.codec_dai_name = "ADMAIF5",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF6] = {
		.name = "ADMAIF6 CIF",
		.stream_name = "ADMAIF6 CIF",
		.cpu_dai_name = "ADMAIF6",
		.codec_dai_name = "ADMAIF6",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF7] = {
		.name = "ADMAIF7 CIF",
		.stream_name = "ADMAIF7 CIF",
		.cpu_dai_name = "ADMAIF7",
		.codec_dai_name = "ADMAIF7",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF8] = {
		.name = "ADMAIF8 CIF",
		.stream_name = "ADMAIF8 CIF",
		.cpu_dai_name = "ADMAIF8",
		.codec_dai_name = "ADMAIF8",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF9] = {
		.name = "ADMAIF9 CIF",
		.stream_name = "ADMAIF9 CIF",
		.cpu_dai_name = "ADMAIF9",
		.codec_dai_name = "ADMAIF9",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF10] = {
		.name = "ADMAIF10 CIF",
		.stream_name = "ADMAIF10 CIF",
		.cpu_dai_name = "ADMAIF10",
		.codec_dai_name = "ADMAIF10",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF11] = {
		.name = "ADMAIF11 CIF",
		.stream_name = "ADMAIF11 CIF",
		.cpu_dai_name = "ADMAIF11",
		.codec_dai_name = "ADMAIF11",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF12] = {
		.name = "ADMAIF12 CIF",
		.stream_name = "ADMAIF12 CIF",
		.cpu_dai_name = "ADMAIF12",
		.codec_dai_name = "ADMAIF12",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF13] = {
		.name = "ADMAIF13 CIF",
		.stream_name = "ADMAIF13 CIF",
		.cpu_dai_name = "ADMAIF13",
		.codec_dai_name = "ADMAIF13",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF14] = {
		.name = "ADMAIF14 CIF",
		.stream_name = "ADMAIF14 CIF",
		.cpu_dai_name = "ADMAIF14",
		.codec_dai_name = "ADMAIF14",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF15] = {
		.name = "ADMAIF15 CIF",
		.stream_name = "ADMAIF15 CIF",
		.cpu_dai_name = "ADMAIF15",
		.codec_dai_name = "ADMAIF15",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF16] = {
		.name = "ADMAIF16 CIF",
		.stream_name = "ADMAIF16 CIF",
		.cpu_dai_name = "ADMAIF16",
		.codec_dai_name = "ADMAIF16",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF17] = {
		.name = "ADMAIF17 CIF",
		.stream_name = "ADMAIF17 CIF",
		.cpu_dai_name = "ADMAIF17",
		.codec_dai_name = "ADMAIF17",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF18] = {
		.name = "ADMAIF18 CIF",
		.stream_name = "ADMAIF18 CIF",
		.cpu_dai_name = "ADMAIF18",
		.codec_dai_name = "ADMAIF18",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF19] = {
		.name = "ADMAIF19 CIF",
		.stream_name = "ADMAIF19 CIF",
		.cpu_dai_name = "ADMAIF19",
		.codec_dai_name = "ADMAIF19",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF20] = {
		.name = "ADMAIF20 CIF",
		.stream_name = "ADMAIF20 CIF",
		.cpu_dai_name = "ADMAIF20",
		.codec_dai_name = "ADMAIF20",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
	},
	[TEGRA186_DAI_LINK_ADMAIF1_CODEC] = {
		.name = "ADMAIF1 CODEC",
		.stream_name = "ADMAIF1 CODEC",
		.cpu_dai_name = "ADMAIF1 CIF",
		.codec_dai_name = "ADMAIF1",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ADMAIF2_CODEC] = {
		.name = "ADMAIF2 CODEC",
		.stream_name = "ADMAIF2 CODEC",
		.cpu_dai_name = "ADMAIF2 CIF",
		.codec_dai_name = "ADMAIF2",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ADMAIF3_CODEC] = {
		.name = "ADMAIF3 CODEC",
		.stream_name = "ADMAIF3 CODEC",
		.cpu_dai_name = "ADMAIF3 CIF",
		.codec_dai_name = "ADMAIF3",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ADMAIF4_CODEC] = {
		.name = "ADMAIF4 CODEC",
		.stream_name = "ADMAIF4 CODEC",
		.cpu_dai_name = "ADMAIF4 CIF",
		.codec_dai_name = "ADMAIF4",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ADMAIF5_CODEC] = {
		.name = "ADMAIF5 CODEC",
		.stream_name = "ADMAIF5 CODEC",
		.cpu_dai_name = "ADMAIF5 CIF",
		.codec_dai_name = "ADMAIF5",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ADMAIF6_CODEC] = {
		.name = "ADMAIF6 CODEC",
		.stream_name = "ADMAIF6 CODEC",
		.cpu_dai_name = "ADMAIF6 CIF",
		.codec_dai_name = "ADMAIF6",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ADMAIF7_CODEC] = {
		.name = "ADMAIF7 CODEC",
		.stream_name = "ADMAIF7 CODEC",
		.cpu_dai_name = "ADMAIF7 CIF",
		.codec_dai_name = "ADMAIF7",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ADMAIF8_CODEC] = {
		.name = "ADMAIF8 CODEC",
		.stream_name = "ADMAIF8 CODEC",
		.cpu_dai_name = "ADMAIF8 CIF",
		.codec_dai_name = "ADMAIF8",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ADMAIF9_CODEC] = {
		.name = "ADMAIF9 CODEC",
		.stream_name = "ADMAIF9 CODEC",
		.cpu_dai_name = "ADMAIF9 CIF",
		.codec_dai_name = "ADMAIF9",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ADMAIF10_CODEC] = {
		.name = "ADMAIF10 CODEC",
		.stream_name = "ADMAIF10 CODEC",
		.cpu_dai_name = "ADMAIF10 CIF",
		.codec_dai_name = "ADMAIF10",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ADMAIF11_CODEC] = {
		.name = "ADMAIF11 CODEC",
		.stream_name = "ADMAIF11 CODEC",
		.cpu_dai_name = "ADMAIF11 CIF",
		.codec_dai_name = "ADMAIF11",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ADMAIF12_CODEC] = {
		.name = "ADMAIF12 CODEC",
		.stream_name = "ADMAIF12 CODEC",
		.cpu_dai_name = "ADMAIF12 CIF",
		.codec_dai_name = "ADMAIF12",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ADMAIF13_CODEC] = {
		.name = "ADMAIF13 CODEC",
		.stream_name = "ADMAIF13 CODEC",
		.cpu_dai_name = "ADMAIF13 CIF",
		.codec_dai_name = "ADMAIF13",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ADMAIF14_CODEC] = {
		.name = "ADMAIF14 CODEC",
		.stream_name = "ADMAIF14 CODEC",
		.cpu_dai_name = "ADMAIF14 CIF",
		.codec_dai_name = "ADMAIF14",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ADMAIF15_CODEC] = {
		.name = "ADMAIF15 CODEC",
		.stream_name = "ADMAIF15 CODEC",
		.cpu_dai_name = "ADMAIF15 CIF",
		.codec_dai_name = "ADMAIF15",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ADMAIF16_CODEC] = {
		.name = "ADMAIF16 CODEC",
		.stream_name = "ADMAIF16 CODEC",
		.cpu_dai_name = "ADMAIF16 CIF",
		.codec_dai_name = "ADMAIF16",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ADMAIF17_CODEC] = {
		.name = "ADMAIF17 CODEC",
		.stream_name = "ADMAIF17 CODEC",
		.cpu_dai_name = "ADMAIF17 CIF",
		.codec_dai_name = "ADMAIF17",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ADMAIF18_CODEC] = {
		.name = "ADMAIF18 CODEC",
		.stream_name = "ADMAIF18 CODEC",
		.cpu_dai_name = "ADMAIF18 CIF",
		.codec_dai_name = "ADMAIF18",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ADMAIF19_CODEC] = {
		.name = "ADMAIF19 CODEC",
		.stream_name = "ADMAIF19 CODEC",
		.cpu_dai_name = "ADMAIF19 CIF",
		.codec_dai_name = "ADMAIF19",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_ADMAIF20_CODEC] = {
		.name = "ADMAIF20 CODEC",
		.stream_name = "ADMAIF20 CODEC",
		.cpu_dai_name = "ADMAIF20 CIF",
		.codec_dai_name = "ADMAIF20",
		.cpu_name = "tegra186-admaif",
		.codec_name = "tegra210-axbar",
		.platform_name = "tegra186-admaif",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
		.params = &default_link_params,
	},
	[TEGRA186_DAI_LINK_AMX1_1] = {
		.name = "AMX1 IN1",
		.stream_name = "AMX1 IN",
		.cpu_dai_name = "AMX1-1",
		.codec_dai_name = "IN1",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.0",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AMX1_2] = {
		.name = "AMX1 IN2",
		.stream_name = "AMX1 IN",
		.cpu_dai_name = "AMX1-2",
		.codec_dai_name = "IN2",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.0",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AMX1_3] = {
		.name = "AMX1 IN3",
		.stream_name = "AMX1 IN",
		.cpu_dai_name = "AMX1-3",
		.codec_dai_name = "IN3",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.0",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AMX1_4] = {
		.name = "AMX1 IN4",
		.stream_name = "AMX1 IN",
		.cpu_dai_name = "AMX1-4",
		.codec_dai_name = "IN4",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.0",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AMX1] = {
		.name = "AMX1 CIF",
		.stream_name = "AMX1 CIF",
		.cpu_dai_name = "OUT",
		.codec_dai_name = "AMX1",
		.cpu_name = "tegra210-amx.0",
		.codec_name = "tegra210-axbar",
		.params = &tdm_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AMX2_1] = {
		.name = "AMX2 IN1",
		.stream_name = "AMX2 IN",
		.cpu_dai_name = "AMX2-1",
		.codec_dai_name = "IN1",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.1",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AMX2_2] = {
		.name = "AMX2 IN2",
		.stream_name = "AMX2 IN",
		.cpu_dai_name = "AMX2-2",
		.codec_dai_name = "IN2",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.1",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AMX2_3] = {
		.name = "AMX2 IN3",
		.stream_name = "AMX2 IN",
		.cpu_dai_name = "AMX2-3",
		.codec_dai_name = "IN3",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.1",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AMX2_4] = {
		.name = "AMX2 IN4",
		.stream_name = "AMX2 IN",
		.cpu_dai_name = "AMX2-4",
		.codec_dai_name = "IN4",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.1",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AMX2] = {
		.name = "AMX2 CIF",
		.stream_name = "AMX2 CIF",
		.cpu_dai_name = "OUT",
		.codec_dai_name = "AMX2",
		.cpu_name = "tegra210-amx.1",
		.codec_name = "tegra210-axbar",
		.params = &tdm_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AMX3_1] = {
		.name = "AMX3 IN1",
		.stream_name = "AMX3 IN",
		.cpu_dai_name = "AMX3-1",
		.codec_dai_name = "IN1",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.2",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AMX3_2] = {
		.name = "AMX3 IN2",
		.stream_name = "AMX3 IN",
		.cpu_dai_name = "AMX3-2",
		.codec_dai_name = "IN2",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.2",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AMX3_3] = {
		.name = "AMX3 IN3",
		.stream_name = "AMX3 IN",
		.cpu_dai_name = "AMX3-3",
		.codec_dai_name = "IN3",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.2",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AMX3_4] = {
		.name = "AMX3 IN4",
		.stream_name = "AMX3 IN",
		.cpu_dai_name = "AMX3-4",
		.codec_dai_name = "IN4",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.2",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AMX3] = {
		.name = "AMX3 CIF",
		.stream_name = "AMX3 CIF",
		.cpu_dai_name = "OUT",
		.codec_dai_name = "AMX3",
		.cpu_name = "tegra210-amx.2",
		.codec_name = "tegra210-axbar",
		.params = &tdm_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AMX4_1] = {
		.name = "AMX4 IN1",
		.stream_name = "AMX4 IN",
		.cpu_dai_name = "AMX4-1",
		.codec_dai_name = "IN1",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.3",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AMX4_2] = {
		.name = "AMX4 IN2",
		.stream_name = "AMX4 IN",
		.cpu_dai_name = "AMX4-2",
		.codec_dai_name = "IN2",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.3",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AMX4_3] = {
		.name = "AMX4 IN3",
		.stream_name = "AMX4 IN",
		.cpu_dai_name = "AMX4-3",
		.codec_dai_name = "IN3",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.3",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AMX4_4] = {
		.name = "AMX4 IN4",
		.stream_name = "AMX4 IN",
		.cpu_dai_name = "AMX4-4",
		.codec_dai_name = "IN4",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-amx.3",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AMX4] = {
		.name = "AMX4 CIF",
		.stream_name = "AMX4 CIF",
		.cpu_dai_name = "OUT",
		.codec_dai_name = "AMX4",
		.cpu_name = "tegra210-amx.3",
		.codec_name = "tegra210-axbar",
		.params = &tdm_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADX1] = {
		.name = "ADX1 CIF",
		.stream_name = "ADX1 IN",
		.cpu_dai_name = "ADX1",
		.codec_dai_name = "IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-adx.0",
		.params = &tdm_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADX1_1] = {
		.name = "ADX1 OUT1",
		.stream_name = "ADX1 OUT",
		.cpu_dai_name = "OUT1",
		.codec_dai_name = "ADX1-1",
		.cpu_name = "tegra210-adx.0",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADX1_2] = {
		.name = "ADX1 OUT2",
		.stream_name = "ADX1 OUT",
		.cpu_dai_name = "OUT2",
		.codec_dai_name = "ADX1-2",
		.cpu_name = "tegra210-adx.0",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADX1_3] = {
		.name = "ADX1 OUT3",
		.stream_name = "ADX1 OUT",
		.cpu_dai_name = "OUT3",
		.codec_dai_name = "ADX1-3",
		.cpu_name = "tegra210-adx.0",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADX1_4] = {
		.name = "ADX1 OUT4",
		.stream_name = "ADX1 OUT",
		.cpu_dai_name = "OUT4",
		.codec_dai_name = "ADX1-4",
		.cpu_name = "tegra210-adx.0",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADX2] = {
		.name = "ADX2 CIF",
		.stream_name = "ADX2 IN",
		.cpu_dai_name = "ADX2",
		.codec_dai_name = "IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-adx.1",
		.params = &tdm_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADX2_1] = {
		.name = "ADX2 OUT1",
		.stream_name = "ADX2 OUT",
		.cpu_dai_name = "OUT1",
		.codec_dai_name = "ADX2-1",
		.cpu_name = "tegra210-adx.1",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADX2_2] = {
		.name = "ADX2 OUT2",
		.stream_name = "ADX2 OUT",
		.cpu_dai_name = "OUT2",
		.codec_dai_name = "ADX2-2",
		.cpu_name = "tegra210-adx.1",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADX2_3] = {
		.name = "ADX2 OUT3",
		.stream_name = "ADX2 OUT",
		.cpu_dai_name = "OUT3",
		.codec_dai_name = "ADX2-3",
		.cpu_name = "tegra210-adx.1",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADX2_4] = {
		.name = "ADX2 OUT4",
		.stream_name = "ADX2 OUT",
		.cpu_dai_name = "OUT4",
		.codec_dai_name = "ADX2-4",
		.cpu_name = "tegra210-adx.1",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADX3] = {
		.name = "ADX3 CIF",
		.stream_name = "ADX3 IN",
		.cpu_dai_name = "ADX3",
		.codec_dai_name = "IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-adx.2",
		.params = &tdm_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADX3_1] = {
		.name = "ADX3 OUT1",
		.stream_name = "ADX3 OUT",
		.cpu_dai_name = "OUT1",
		.codec_dai_name = "ADX3-1",
		.cpu_name = "tegra210-adx.2",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADX3_2] = {
		.name = "ADX3 OUT2",
		.stream_name = "ADX3 OUT",
		.cpu_dai_name = "OUT2",
		.codec_dai_name = "ADX3-2",
		.cpu_name = "tegra210-adx.2",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADX3_3] = {
		.name = "ADX3 OUT3",
		.stream_name = "ADX3 OUT",
		.cpu_dai_name = "OUT3",
		.codec_dai_name = "ADX3-3",
		.cpu_name = "tegra210-adx.2",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADX3_4] = {
		.name = "ADX3 OUT4",
		.stream_name = "ADX3 OUT",
		.cpu_dai_name = "OUT4",
		.codec_dai_name = "ADX3-4",
		.cpu_name = "tegra210-adx.2",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADX4] = {
		.name = "ADX4 CIF",
		.stream_name = "ADX4 IN",
		.cpu_dai_name = "ADX4",
		.codec_dai_name = "IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-adx.3",
		.params = &tdm_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADX4_1] = {
		.name = "ADX4 OUT1",
		.stream_name = "ADX4 OUT",
		.cpu_dai_name = "OUT1",
		.codec_dai_name = "ADX4-1",
		.cpu_name = "tegra210-adx.3",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADX4_2] = {
		.name = "ADX4 OUT2",
		.stream_name = "ADX4 OUT",
		.cpu_dai_name = "OUT2",
		.codec_dai_name = "ADX4-2",
		.cpu_name = "tegra210-adx.3",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADX4_3] = {
		.name = "ADX4 OUT3",
		.stream_name = "ADX4 OUT",
		.cpu_dai_name = "OUT3",
		.codec_dai_name = "ADX4-3",
		.cpu_name = "tegra210-adx.3",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADX4_4] = {
		.name = "ADX4 OUT4",
		.stream_name = "ADX4 OUT",
		.cpu_dai_name = "OUT4",
		.codec_dai_name = "ADX4-4",
		.cpu_name = "tegra210-adx.3",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_MIXER1_RX1] = {
		.name = "MIXER1 RX1",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-1",
		.codec_dai_name = "RX1",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_MIXER1_RX2] = {
		.name = "MIXER1 RX2",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-2",
		.codec_dai_name = "RX2",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_MIXER1_RX3] = {
		.name = "MIXER1 RX3",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-3",
		.codec_dai_name = "RX3",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_MIXER1_RX4] = {
		.name = "MIXER1 RX4",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-4",
		.codec_dai_name = "RX4",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_MIXER1_RX5] = {
		.name = "MIXER1 RX5",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-5",
		.codec_dai_name = "RX5",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_MIXER1_RX6] = {
		.name = "MIXER1 RX6",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-6",
		.codec_dai_name = "RX6",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_MIXER1_RX7] = {
		.name = "MIXER1 RX7",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-7",
		.codec_dai_name = "RX7",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_MIXER1_RX8] = {
		.name = "MIXER1 RX8",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-8",
		.codec_dai_name = "RX8",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_MIXER1_RX9] = {
		.name = "MIXER1 RX9",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-9",
		.codec_dai_name = "RX9",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_MIXER1_RX10] = {
		.name = "MIXER1 RX10",
		.stream_name = "MIXER1 RX",
		.cpu_dai_name = "MIXER1-10",
		.codec_dai_name = "RX10",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mixer",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_MIXER1_TX1] = {
		.name = "MIXER1 TX1",
		.stream_name = "MIXER1 TX",
		.cpu_dai_name = "TX1",
		.codec_dai_name = "MIXER1-1",
		.cpu_name = "tegra210-mixer",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_MIXER1_TX2] = {
		.name = "MIXER1 TX2",
		.stream_name = "MIXER1 TX",
		.cpu_dai_name = "TX2",
		.codec_dai_name = "MIXER1-2",
		.cpu_name = "tegra210-mixer",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_MIXER1_TX3] = {
		.name = "MIXER1 TX3",
		.stream_name = "MIXER1 TX",
		.cpu_dai_name = "TX3",
		.codec_dai_name = "MIXER1-3",
		.cpu_name = "tegra210-mixer",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_MIXER1_TX4] = {
		.name = "MIXER1 TX4",
		.stream_name = "MIXER1 TX",
		.cpu_dai_name = "TX4",
		.codec_dai_name = "MIXER1-4",
		.cpu_name = "tegra210-mixer",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_MIXER1_TX5] = {
		.name = "MIXER1 TX5",
		.stream_name = "MIXER1 TX",
		.cpu_dai_name = "TX5",
		.codec_dai_name = "MIXER1-5",
		.cpu_name = "tegra210-mixer",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_SFC1_RX] = {
		.name = "SFC1 RX",
		.stream_name = "SFC1 RX",
		.cpu_dai_name = "SFC1",
		.codec_dai_name = "CIF",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-sfc.0",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_SFC1_TX] = {
		.name = "SFC1 TX",
		.stream_name = "SFC1 TX",
		.cpu_dai_name = "DAP",
		.codec_dai_name = "SFC1",
		.cpu_name = "tegra210-sfc.0",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_SFC2_RX] = {
		.name = "SFC2 RX",
		.stream_name = "SFC2 RX",
		.cpu_dai_name = "SFC2",
		.codec_dai_name = "CIF",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-sfc.1",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_SFC2_TX] = {
		.name = "SFC2 TX",
		.stream_name = "SFC2 TX",
		.cpu_dai_name = "DAP",
		.codec_dai_name = "SFC2",
		.cpu_name = "tegra210-sfc.1",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_SFC3_RX] = {
		.name = "SFC3 RX",
		.stream_name = "SFC3 RX",
		.cpu_dai_name = "SFC3",
		.codec_dai_name = "CIF",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-sfc.2",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_SFC3_TX] = {
		.name = "SFC3 TX",
		.stream_name = "SFC3 TX",
		.cpu_dai_name = "DAP",
		.codec_dai_name = "SFC3",
		.cpu_name = "tegra210-sfc.2",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_SFC4_RX] = {
		.name = "SFC4 RX",
		.stream_name = "SFC4 RX",
		.cpu_dai_name = "SFC4",
		.codec_dai_name = "CIF",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-sfc.3",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_SFC4_TX] = {
		.name = "SFC4 TX",
		.stream_name = "SFC4 TX",
		.cpu_dai_name = "DAP",
		.codec_dai_name = "SFC4",
		.cpu_name = "tegra210-sfc.3",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AFC1_RX] = {
		.name = "AFC1 RX",
		.stream_name = "AFC1 RX",
		.cpu_dai_name = "AFC1",
		.codec_dai_name = "AFC IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra186-afc.0",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AFC1_TX] = {
		.name = "AFC1 TX",
		.stream_name = "AFC1 TX",
		.cpu_dai_name = "AFC OUT",
		.codec_dai_name = "AFC1",
		.cpu_name = "tegra186-afc.0",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AFC2_RX] = {
		.name = "AFC2 RX",
		.stream_name = "AFC2 RX",
		.cpu_dai_name = "AFC2",
		.codec_dai_name = "AFC IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra186-afc.1",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AFC2_TX] = {
		.name = "AFC2 TX",
		.stream_name = "AFC2 TX",
		.cpu_dai_name = "AFC OUT",
		.codec_dai_name = "AFC2",
		.cpu_name = "tegra186-afc.1",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AFC3_RX] = {
		.name = "AFC3 RX",
		.stream_name = "AFC3 RX",
		.cpu_dai_name = "AFC3",
		.codec_dai_name = "AFC IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra186-afc.2",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AFC3_TX] = {
		.name = "AFC3 TX",
		.stream_name = "AFC3 TX",
		.cpu_dai_name = "AFC OUT",
		.codec_dai_name = "AFC3",
		.cpu_name = "tegra186-afc.2",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AFC4_RX] = {
		.name = "AFC4 RX",
		.stream_name = "AFC4 RX",
		.cpu_dai_name = "AFC4",
		.codec_dai_name = "AFC IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra186-afc.3",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AFC4_TX] = {
		.name = "AFC4 TX",
		.stream_name = "AFC4 TX",
		.cpu_dai_name = "AFC OUT",
		.codec_dai_name = "AFC4",
		.cpu_name = "tegra186-afc.3",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AFC5_RX] = {
		.name = "AFC5 RX",
		.stream_name = "AFC5 RX",
		.cpu_dai_name = "AFC5",
		.codec_dai_name = "AFC IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra186-afc.4",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AFC5_TX] = {
		.name = "AFC5 TX",
		.stream_name = "AFC5 TX",
		.cpu_dai_name = "AFC OUT",
		.codec_dai_name = "AFC5",
		.cpu_name = "tegra186-afc.4",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AFC6_RX] = {
		.name = "AFC6 RX",
		.stream_name = "AFC6 RX",
		.cpu_dai_name = "AFC6",
		.codec_dai_name = "AFC IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra186-afc.5",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_AFC6_TX] = {
		.name = "AFC6 TX",
		.stream_name = "AFC6 TX",
		.cpu_dai_name = "AFC OUT",
		.codec_dai_name = "AFC6",
		.cpu_name = "tegra186-afc.5",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_MVC1_RX] = {
		.name = "MVC1 RX",
		.stream_name = "MVC1 RX",
		.cpu_dai_name = "MVC1",
		.codec_dai_name = "MVC IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mvc.0",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_MVC1_TX] = {
		.name = "MVC1 TX",
		.stream_name = "MVC1 TX",
		.cpu_dai_name = "MVC OUT",
		.codec_dai_name = "MVC1",
		.cpu_name = "tegra210-mvc.0",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_MVC2_RX] = {
		.name = "MVC2 RX",
		.stream_name = "MVC2 RX",
		.cpu_dai_name = "MVC2",
		.codec_dai_name = "MVC IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-mvc.1",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_MVC2_TX] = {
		.name = "MVC2 TX",
		.stream_name = "MVC2 TX",
		.cpu_dai_name = "MVC OUT",
		.codec_dai_name = "MVC2",
		.cpu_name = "tegra210-mvc.1",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_OPE1_RX] = {
		.name = "OPE1 RX",
		.stream_name = "OPE1 RX",
		.cpu_dai_name = "OPE1",
		.codec_dai_name = "OPE IN",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra210-ope.0",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_OPE1_TX] = {
		.name = "OPE1 TX",
		.stream_name = "OPE1 TX",
		.cpu_dai_name = "OPE OUT",
		.codec_dai_name = "OPE1",
		.cpu_name = "tegra210-ope.0",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ASRC1_RX1] = {
		.name = "ASRC1 RX1",
		.stream_name = "ASRC1 RX",
		.cpu_dai_name = "ASRC1-1",
		.codec_dai_name = "RX1",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra186-asrc",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ASRC1_RX2] = {
		.name = "ASRC1 RX2",
		.stream_name = "ASRC1 RX",
		.cpu_dai_name = "ASRC1-2",
		.codec_dai_name = "RX2",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra186-asrc",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ASRC1_RX3] = {
		.name = "ASRC1 RX3",
		.stream_name = "ASRC1 RX",
		.cpu_dai_name = "ASRC1-3",
		.codec_dai_name = "RX3",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra186-asrc",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ASRC1_RX4] = {
		.name = "ASRC1 RX4",
		.stream_name = "ASRC1 RX",
		.cpu_dai_name = "ASRC1-4",
		.codec_dai_name = "RX4",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra186-asrc",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ASRC1_RX5] = {
		.name = "ASRC1 RX5",
		.stream_name = "ASRC1 RX",
		.cpu_dai_name = "ASRC1-5",
		.codec_dai_name = "RX5",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra186-asrc",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ASRC1_RX6] = {
		.name = "ASRC1 RX6",
		.stream_name = "ASRC1 RX",
		.cpu_dai_name = "ASRC1-6",
		.codec_dai_name = "RX6",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra186-asrc",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ASRC1_RX7] = {
		.name = "ASRC1 RX7",
		.stream_name = "ASRC1 RX",
		.cpu_dai_name = "ASRC1-7",
		.codec_dai_name = "RX7",
		.cpu_name = "tegra210-axbar",
		.codec_name = "tegra186-asrc",
		.params = &arad_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ASRC1_TX1] = {
		.name = "ASRC1 TX1",
		.stream_name = "ASRC1 TX",
		.cpu_dai_name = "TX1",
		.codec_dai_name = "ASRC1-1",
		.cpu_name = "tegra186-asrc",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ASRC1_TX2] = {
		.name = "ASRC1 TX2",
		.stream_name = "ASRC1 TX",
		.cpu_dai_name = "TX2",
		.codec_dai_name = "ASRC1-2",
		.cpu_name = "tegra186-asrc",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ASRC1_TX3] = {
		.name = "ASRC1 TX3",
		.stream_name = "ASRC1 TX",
		.cpu_dai_name = "TX3",
		.codec_dai_name = "ASRC1-3",
		.cpu_name = "tegra186-asrc",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ASRC1_TX4] = {
		.name = "ASRC1 TX4",
		.stream_name = "ASRC1 TX",
		.cpu_dai_name = "TX4",
		.codec_dai_name = "ASRC1-4",
		.cpu_name = "tegra186-asrc",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ASRC1_TX5] = {
		.name = "ASRC1 TX5",
		.stream_name = "ASRC1 TX",
		.cpu_dai_name = "TX5",
		.codec_dai_name = "ASRC1-5",
		.cpu_name = "tegra186-asrc",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ASRC1_TX6] = {
		.name = "ASRC1 TX6",
		.stream_name = "ASRC1 TX",
		.cpu_dai_name = "TX6",
		.codec_dai_name = "ASRC1-6",
		.cpu_name = "tegra186-asrc",
		.codec_name = "tegra210-axbar",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
#if IS_ENABLED(CONFIG_SND_SOC_TEGRA210_ADSP_ALT)
	[TEGRA186_DAI_LINK_ADSP_ADMAIF1] = {
		.name = "ADSP ADMAIF1",
		.stream_name = "ADSP ADMAIF1",
		.cpu_dai_name = "ADSP-ADMAIF1",
		.codec_dai_name = "ADMAIF1 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra186-admaif",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADSP_ADMAIF2] = {
		.name = "ADSP ADMAIF2",
		.stream_name = "ADSP ADMAIF2",
		.cpu_dai_name = "ADSP-ADMAIF2",
		.codec_dai_name = "ADMAIF2 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra186-admaif",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADSP_ADMAIF3] = {
		.name = "ADSP ADMAIF3",
		.stream_name = "ADSP ADMAIF3",
		.cpu_dai_name = "ADSP-ADMAIF3",
		.codec_dai_name = "ADMAIF3 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra186-admaif",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADSP_ADMAIF4] = {
		.name = "ADSP ADMAIF4",
		.stream_name = "ADSP ADMAIF4",
		.cpu_dai_name = "ADSP-ADMAIF4",
		.codec_dai_name = "ADMAIF4 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra186-admaif",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADSP_ADMAIF5] = {
		.name = "ADSP ADMAIF5",
		.stream_name = "ADSP ADMAIF5",
		.cpu_dai_name = "ADSP-ADMAIF5",
		.codec_dai_name = "ADMAIF5 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra186-admaif",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADSP_ADMAIF6] = {
		.name = "ADSP ADMAIF6",
		.stream_name = "ADSP ADMAIF6",
		.cpu_dai_name = "ADSP-ADMAIF6",
		.codec_dai_name = "ADMAIF6 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra186-admaif",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADSP_ADMAIF7] = {
		.name = "ADSP ADMAIF7",
		.stream_name = "ADSP ADMAIF7",
		.cpu_dai_name = "ADSP-ADMAIF7",
		.codec_dai_name = "ADMAIF7 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra186-admaif",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADSP_ADMAIF8] = {
		.name = "ADSP ADMAIF8",
		.stream_name = "ADSP ADMAIF8",
		.cpu_dai_name = "ADSP-ADMAIF8",
		.codec_dai_name = "ADMAIF8 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra186-admaif",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADSP_ADMAIF9] = {
		.name = "ADSP ADMAIF9",
		.stream_name = "ADSP ADMAIF9",
		.cpu_dai_name = "ADSP-ADMAIF9",
		.codec_dai_name = "ADMAIF9 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra186-admaif",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADSP_ADMAIF10] = {
		.name = "ADSP ADMAIF10",
		.stream_name = "ADSP ADMAIF10",
		.cpu_dai_name = "ADSP-ADMAIF10",
		.codec_dai_name = "ADMAIF10 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra186-admaif",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADSP_ADMAIF11] = {
		.name = "ADSP ADMAIF11",
		.stream_name = "ADSP ADMAIF11",
		.cpu_dai_name = "ADSP-ADMAIF11",
		.codec_dai_name = "ADMAIF11 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra186-admaif",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADSP_ADMAIF12] = {
		.name = "ADSP ADMAIF12",
		.stream_name = "ADSP ADMAIF12",
		.cpu_dai_name = "ADSP-ADMAIF12",
		.codec_dai_name = "ADMAIF12 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra186-admaif",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADSP_ADMAIF13] = {
		.name = "ADSP ADMAIF13",
		.stream_name = "ADSP ADMAIF13",
		.cpu_dai_name = "ADSP-ADMAIF13",
		.codec_dai_name = "ADMAIF13 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra186-admaif",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADSP_ADMAIF14] = {
		.name = "ADSP ADMAIF14",
		.stream_name = "ADSP ADMAIF14",
		.cpu_dai_name = "ADSP-ADMAIF14",
		.codec_dai_name = "ADMAIF14 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra186-admaif",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADSP_ADMAIF15] = {
		.name = "ADSP ADMAIF15",
		.stream_name = "ADSP ADMAIF15",
		.cpu_dai_name = "ADSP-ADMAIF15",
		.codec_dai_name = "ADMAIF15 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra186-admaif",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADSP_ADMAIF16] = {
		.name = "ADSP ADMAIF16",
		.stream_name = "ADSP ADMAIF16",
		.cpu_dai_name = "ADSP-ADMAIF16",
		.codec_dai_name = "ADMAIF16 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra186-admaif",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADSP_ADMAIF17] = {
		.name = "ADSP ADMAIF17",
		.stream_name = "ADSP ADMAIF17",
		.cpu_dai_name = "ADSP-ADMAIF17",
		.codec_dai_name = "ADMAIF17 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra186-admaif",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADSP_ADMAIF18] = {
		.name = "ADSP ADMAIF18",
		.stream_name = "ADSP ADMAIF18",
		.cpu_dai_name = "ADSP-ADMAIF18",
		.codec_dai_name = "ADMAIF18 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra186-admaif",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADSP_ADMAIF19] = {
		.name = "ADSP ADMAIF19",
		.stream_name = "ADSP ADMAIF19",
		.cpu_dai_name = "ADSP-ADMAIF19",
		.codec_dai_name = "ADMAIF19 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra186-admaif",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADSP_ADMAIF20] = {
		.name = "ADSP ADMAIF20",
		.stream_name = "ADSP ADMAIF20",
		.cpu_dai_name = "ADSP-ADMAIF20",
		.codec_dai_name = "ADMAIF20 FIFO",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra186-admaif",
		.params = &default_link_params,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADSP_PCM1] = {
		.name = "ADSP PCM1",
		.stream_name = "ADSP PCM1",
		.cpu_dai_name = "ADSP PCM1",
		.codec_dai_name = "ADSP-FE1",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra210-adsp",
		.platform_name = "tegra210-adsp",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADSP_PCM2] = {
		.name = "ADSP PCM2",
		.stream_name = "ADSP PCM2",
		.cpu_dai_name = "ADSP PCM2",
		.codec_dai_name = "ADSP-FE2",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra210-adsp",
		.platform_name = "tegra210-adsp",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADSP_COMPR1] = {
		.name = "ADSP COMPR1",
		.stream_name = "ADSP COMPR1",
		.cpu_dai_name = "ADSP COMPR1",
		.codec_dai_name = "ADSP-FE3",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra210-adsp",
		.platform_name = "tegra210-adsp",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	[TEGRA186_DAI_LINK_ADSP_COMPR2] = {
		.name = "ADSP COMPR2",
		.stream_name = "ADSP COMPR2",
		.cpu_dai_name = "ADSP COMPR2",
		.codec_dai_name = "ADSP-FE4",
		.cpu_name = "tegra210-adsp",
		.codec_name = "tegra210-adsp",
		.platform_name = "tegra210-adsp",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
#endif
	[TEGRA186_DAI_LINK_ARAD] = {
		.name = "ARAD",
		.stream_name = "ARAD ratio info",
		.cpu_dai_name = "ARAD OUT",
		.codec_dai_name = "ARAD1",
		.cpu_name = "tegra186-arad",
		.codec_name = "tegra210-axbar",
		.params = &arad_link_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
};
EXPORT_SYMBOL_GPL(tegra186_xbar_dai_links);

struct snd_soc_codec_conf tegra186_xbar_codec_conf[TEGRA186_XBAR_CODEC_CONF] = {
	[TEGRA186_CODEC_AMX1_CONF] = {
		.dev_name = "tegra210-amx.0",
		.name_prefix = "AMX1",
	},
	[TEGRA186_CODEC_AMX2_CONF] = {
		.dev_name = "tegra210-amx.1",
		.name_prefix = "AMX2",
	},
	[TEGRA186_CODEC_AMX3_CONF] = {
		.dev_name = "tegra210-amx.2",
		.name_prefix = "AMX3",
	},
	[TEGRA186_CODEC_AMX4_CONF] = {
		.dev_name = "tegra210-amx.3",
		.name_prefix = "AMX4",
	},
	[TEGRA186_CODEC_ADX1_CONF] = {
		.dev_name = "tegra210-adx.0",
		.name_prefix = "ADX1",
	},
	[TEGRA186_CODEC_ADX2_CONF] = {
		.dev_name = "tegra210-adx.1",
		.name_prefix = "ADX2",
	},
	[TEGRA186_CODEC_ADX3_CONF] = {
		.dev_name = "tegra210-adx.2",
		.name_prefix = "ADX3",
	},
	[TEGRA186_CODEC_ADX4_CONF] = {
		.dev_name = "tegra210-adx.3",
		.name_prefix = "ADX4",
	},
	[TEGRA186_CODEC_SFC1_CONF] = {
		.dev_name = "tegra210-sfc.0",
		.name_prefix = "SFC1",
	},
	[TEGRA186_CODEC_SFC2_CONF] = {
		.dev_name = "tegra210-sfc.1",
		.name_prefix = "SFC2",
	},
	[TEGRA186_CODEC_SFC3_CONF] = {
		.dev_name = "tegra210-sfc.2",
		.name_prefix = "SFC3",
	},
	[TEGRA186_CODEC_SFC4_CONF] = {
		.dev_name = "tegra210-sfc.3",
		.name_prefix = "SFC4",
	},
	[TEGRA186_CODEC_MVC1_CONF] = {
		.dev_name = "tegra210-mvc.0",
		.name_prefix = "MVC1",
	},
	[TEGRA186_CODEC_MVC2_CONF] = {
		.dev_name = "tegra210-mvc.1",
		.name_prefix = "MVC2",
	},
	[TEGRA186_CODEC_OPE1_CONF] = {
		.dev_name = "tegra210-ope.0",
		.name_prefix = "OPE1",
	},
	[TEGRA186_CODEC_AFC1_CONF] = {
		.dev_name = "tegra186-afc.0",
		.name_prefix = "AFC1",
	},
	[TEGRA186_CODEC_AFC2_CONF] = {
		.dev_name = "tegra186-afc.1",
		.name_prefix = "AFC2",
	},
	[TEGRA186_CODEC_AFC3_CONF] = {
		.dev_name = "tegra186-afc.2",
		.name_prefix = "AFC3",
	},
	[TEGRA186_CODEC_AFC4_CONF] = {
		.dev_name = "tegra186-afc.3",
		.name_prefix = "AFC4",
	},
	[TEGRA186_CODEC_AFC5_CONF] = {
		.dev_name = "tegra186-afc.4",
		.name_prefix = "AFC5",
	},
	[TEGRA186_CODEC_AFC6_CONF] = {
		.dev_name = "tegra186-afc.5",
		.name_prefix = "AFC6",
	},
	[TEGRA186_CODEC_I2S1_CONF] = {
		.dev_name = "tegra210-i2s.0",
		.name_prefix = "I2S1",
	},
	[TEGRA186_CODEC_I2S2_CONF] = {
		.dev_name = "tegra210-i2s.1",
		.name_prefix = "I2S2",
	},
	[TEGRA186_CODEC_I2S3_CONF] = {
		.dev_name = "tegra210-i2s.2",
		.name_prefix = "I2S3",
	},
	[TEGRA186_CODEC_I2S4_CONF] = {
		.dev_name = "tegra210-i2s.3",
		.name_prefix = "I2S4",
	},
	[TEGRA186_CODEC_I2S5_CONF] = {
		.dev_name = "tegra210-i2s.4",
		.name_prefix = "I2S5",
	},
	[TEGRA186_CODEC_I2S6_CONF] = {
		.dev_name = "tegra210-i2s.5",
		.name_prefix = "I2S6",
	},
	[TEGRA186_CODEC_DMIC1_CONF] = {
		.dev_name = "tegra210-dmic.0",
		.name_prefix = "DMIC1",
	},
	[TEGRA186_CODEC_DMIC2_CONF] = {
		.dev_name = "tegra210-dmic.1",
		.name_prefix = "DMIC2",
	},
	[TEGRA186_CODEC_DMIC3_CONF] = {
		.dev_name = "tegra210-dmic.2",
		.name_prefix = "DMIC3",
	},
	[TEGRA186_CODEC_DMIC4_CONF] = {
		.dev_name = "tegra210-dmic.3",
		.name_prefix = "DMIC4",
	},
	[TEGRA186_CODEC_DSPK1_CONF] = {
		.dev_name = "tegra186-dspk.0",
		.name_prefix = "DSPK1",
	},
	[TEGRA186_CODEC_DSPK2_CONF] = {
		.dev_name = "tegra186-dspk.1",
		.name_prefix = "DSPK2",
	},
	[TEGRA186_CODEC_ASRC1_CONF] = {
		.dev_name = "tegra186-asrc",
		.name_prefix = "ASRC1",
	},
};
EXPORT_SYMBOL_GPL(tegra186_xbar_codec_conf);

struct snd_soc_dai_link *tegra_machine_get_dai_link(void)
{
	struct snd_soc_dai_link *link = NULL;
	unsigned int size = 0;

	if (tegra_asoc_machine_links)
		return tegra_asoc_machine_links;

	if (of_machine_is_compatible("nvidia,tegra210")  ||
		of_machine_is_compatible("nvidia,tegra210b01")) {
		link = tegra210_xbar_dai_links;
		size = TEGRA210_XBAR_DAI_LINKS;
	} else {
		return NULL;
	}

	num_dai_links = size;

	tegra_asoc_machine_links = kzalloc(size *
		sizeof(struct snd_soc_dai_link), GFP_KERNEL);

	memcpy(tegra_asoc_machine_links, link,
		size * sizeof(struct snd_soc_dai_link));

	return tegra_asoc_machine_links;
}
EXPORT_SYMBOL_GPL(tegra_machine_get_dai_link);

void tegra_machine_remove_dai_link(void)
{
	kfree(tegra_asoc_machine_links);
	tegra_asoc_machine_links = NULL;
	tx_mask = NULL;
	rx_mask = NULL;
}
EXPORT_SYMBOL_GPL(tegra_machine_remove_dai_link);

/* @link: input structure to append
 * @link_size: size of the input structure
 * Returns the total size after appending
 */
int tegra_machine_append_dai_link(struct snd_soc_dai_link *link,
		unsigned int link_size)
{
	unsigned int size1 = (of_machine_is_compatible("nvidia,tegra210") ||
			of_machine_is_compatible("nvidia,tegra210b01")) ?
			TEGRA210_XBAR_DAI_LINKS : 0;
	unsigned int size2 = link_size;

	if (!tegra_asoc_machine_links) {
		if (link) {
			tegra_asoc_machine_links = link;
			num_dai_links = size2;
			return size2;
		} else {
			return 0;
		}
	} else {
		if (link) {
			tegra_asoc_machine_links =
				(struct snd_soc_dai_link *) krealloc(
				tegra_asoc_machine_links, (size1 + size2) *
				sizeof(struct snd_soc_dai_link), GFP_KERNEL);
			memcpy(&tegra_asoc_machine_links[size1], link,
				size2 * sizeof(struct snd_soc_dai_link));
			num_dai_links = size1+size2;
			return size1+size2;
		} else {
			num_dai_links = size1;
			return size1;
		}
	}
}
EXPORT_SYMBOL_GPL(tegra_machine_append_dai_link);

void tegra_machine_set_dai_ops(int link, struct snd_soc_ops *ops)
{
	if (tegra_asoc_machine_links)
		tegra_asoc_machine_links[link].ops = ops;
}
EXPORT_SYMBOL_GPL(tegra_machine_set_dai_ops);

void tegra_machine_set_dai_compr_ops(int link, struct snd_soc_compr_ops *ops)
{
	if (tegra_asoc_machine_links)
		tegra_asoc_machine_links[link].compr_ops = ops;
}
EXPORT_SYMBOL_GPL(tegra_machine_set_dai_compr_ops);

void tegra_machine_set_dai_init(int link, void *ptr)
{
	if (tegra_asoc_machine_links)
		tegra_asoc_machine_links[link].init = ptr;
}
EXPORT_SYMBOL_GPL(tegra_machine_set_dai_init);

void tegra_machine_set_dai_params(int link,
		struct snd_soc_pcm_stream *params)
{
	if (tegra_asoc_machine_links && (NULL == params))
		tegra_asoc_machine_links[link].params = &default_link_params;
	else if (tegra_asoc_machine_links)
		tegra_asoc_machine_links[link].params = params;
}
EXPORT_SYMBOL_GPL(tegra_machine_set_dai_params);

void tegra_machine_set_dai_fmt(int link, unsigned int fmt)
{
	if (tegra_asoc_machine_links)
		tegra_asoc_machine_links[link].dai_fmt = fmt;
}
EXPORT_SYMBOL_GPL(tegra_machine_set_dai_fmt);

struct snd_soc_codec_conf *tegra_machine_get_codec_conf(void)
{
	struct snd_soc_codec_conf *conf = NULL;
	unsigned int size = 0;

	if (tegra_asoc_codec_conf)
		return tegra_asoc_codec_conf;

	if (of_machine_is_compatible("nvidia,tegra210")  ||
		of_machine_is_compatible("nvidia,tegra210b01")) {
		conf = tegra210_xbar_codec_conf;
		size = TEGRA210_XBAR_CODEC_CONF;
	} else {
		return NULL;
	}

	tegra_asoc_codec_conf = kzalloc(size *
		sizeof(struct snd_soc_codec_conf), GFP_KERNEL);

	memcpy(tegra_asoc_codec_conf, conf,
		size * sizeof(struct snd_soc_codec_conf));

	return tegra_asoc_codec_conf;
}
EXPORT_SYMBOL_GPL(tegra_machine_get_codec_conf);

void tegra_machine_remove_codec_conf(void)
{
	kfree(tegra_asoc_codec_conf);
	tegra_asoc_codec_conf = NULL;
}
EXPORT_SYMBOL_GPL(tegra_machine_remove_codec_conf);

/* @link: input structure to append
 * @link_size: size of the input structure
 * Returns the total size after appending
 */
int tegra_machine_append_codec_conf(struct snd_soc_codec_conf *conf,
		unsigned int conf_size)
{
	unsigned int size1 = (of_machine_is_compatible("nvidia,tegra210") ||
			of_machine_is_compatible("nvidia,tegra210b01")) ?
			TEGRA210_XBAR_CODEC_CONF : 0;
	unsigned int size2 = conf_size;

	if (!tegra_asoc_codec_conf) {
		if (conf) {
			tegra_asoc_codec_conf = conf;
			return size2;
		} else {
			return 0;
		}
	} else {
		if (conf) {
			tegra_asoc_codec_conf =
				(struct snd_soc_codec_conf *) krealloc(
				tegra_asoc_codec_conf, (size1 + size2) *
				sizeof(struct snd_soc_codec_conf), GFP_KERNEL);
			memcpy(&tegra_asoc_codec_conf[size1], conf,
				size2 * sizeof(struct snd_soc_codec_conf));
			return size1+size2;
		} else {
			return size1;
		}
	}
}
EXPORT_SYMBOL_GPL(tegra_machine_append_codec_conf);


static int tegra_machine_get_format(u64 *p_formats, char *fmt)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(bit_format); i++) {
		if (strcmp(bit_format[i], fmt) == 0) {
			*p_formats = (u64)1 << i;
			return 0;
		}
	}
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(tegra_machine_get_format);

struct snd_soc_dai_link *tegra_machine_new_codec_links(
	struct platform_device *pdev,
	struct snd_soc_dai_link *tegra_codec_links,
	unsigned int *pnum_codec_links)
{
	unsigned int i, j, num_codec_links;
	struct device_node *np = pdev->dev.of_node, *subnp;
	struct snd_soc_pcm_stream *params;
	char dai_link_name[MAX_STR_SIZE];
	char *str;
	const char *prefix;
	struct device_node *bitclkmaster = NULL, *framemaster = NULL;

	if (tegra_codec_links)
		return tegra_codec_links;

	if (!np)
		goto err;

	if (of_property_read_u32(np,
		"nvidia,num-codec-link", (u32 *)&num_codec_links)) {
		dev_err(&pdev->dev,
			"Property 'nvidia,num-codec-link' missing or invalid\n");
		goto err;
	}

	tegra_codec_links = devm_kzalloc(&pdev->dev, 2 * num_codec_links *
		sizeof(struct snd_soc_dai_link), GFP_KERNEL);
	if (!tegra_codec_links) {
		dev_err(&pdev->dev, "Can't allocate tegra_codec_links\n");
		goto err;
	}

	if (rx_mask == NULL) {
		rx_mask = devm_kzalloc(&pdev->dev, num_codec_links *
			sizeof(unsigned int), GFP_KERNEL);
		if (!rx_mask) {
			dev_err(&pdev->dev, "Can't allocate rx_mask\n");
			goto err;
		}
	}

	if (tx_mask == NULL) {
		tx_mask = devm_kzalloc(&pdev->dev, num_codec_links *
			sizeof(unsigned int), GFP_KERNEL);
		if (!tx_mask) {
			dev_err(&pdev->dev, "Can't allocate tx_mask\n");
			goto err;
		}
	}
	/* variable i is for DAP and j is for CIF */
	for (i = 0, j = num_codec_links; i < num_codec_links; i++, j++) {
		memset((void *)dai_link_name, '\0', MAX_STR_SIZE);
		sprintf(dai_link_name, "nvidia,dai-link-%d", i+1);
		subnp = of_get_child_by_name(np, dai_link_name);
		if (subnp) {
			tegra_codec_links[i].codec_of_node =
				of_parse_phandle(subnp, "codec-dai", 0);
			if (!tegra_codec_links[i].codec_of_node) {
				dev_err(&pdev->dev,
					"Property '%s.codec-dai' missing or invalid\n",
					dai_link_name);
				goto err;
			}

			tegra_codec_links[i].cpu_of_node
				= of_parse_phandle(subnp, "cpu-dai", 0);
			if (!tegra_codec_links[i].cpu_of_node) {
				dev_err(&pdev->dev,
					"Property '%s.cpu-dai' missing or invalid\n",
					dai_link_name);
				goto err;
			}

			/* DAP configuration */
			if (of_property_read_string(subnp, "name-prefix",
				&prefix)) {
				dev_err(&pdev->dev,
					"Property 'name-prefix' missing or invalid\n");
				goto err;
			}

			if (of_property_read_string(subnp, "link-name",
				&tegra_codec_links[i].name)) {
				dev_err(&pdev->dev,
					"Property 'link-name' missing or invalid\n");
				goto err;
			}

			tegra_codec_links[i].stream_name = "Playback";
			tegra_codec_links[i].ignore_suspend =
				of_property_read_bool(subnp, "ignore_suspend");

			/* special case to handle specifically for dspk, connected to
			two mono amplifiers */
			if (!strcmp(tegra_codec_links[i].name, "dspk-playback-r"))
				tegra_codec_links[i].cpu_dai_name = "DAP2";
			else
				tegra_codec_links[i].cpu_dai_name = "DAP";

			if (of_property_read_string(subnp, "codec-dai-name",
				&tegra_codec_links[i].codec_dai_name)) {
				dev_err(&pdev->dev,
					"Property 'codec-dai-name' missing or invalid\n");
				goto err;
			}
			tegra_codec_links[i].dai_fmt =
				snd_soc_of_parse_daifmt(subnp, NULL,
					&bitclkmaster, &framemaster);

			params = devm_kzalloc(&pdev->dev,
				sizeof(struct snd_soc_pcm_stream), GFP_KERNEL);

			if (of_property_read_string(subnp,
				"bit-format", (const char **)&str)) {
				dev_err(&pdev->dev,
					"Property 'bit-format' missing or invalid\n");
				goto err;
			}
			if (tegra_machine_get_format(&params->formats, str)) {
				dev_err(&pdev->dev,
					"Wrong codec format\n");
				goto err;
			}

			if (of_property_read_u32(subnp,
				"srate", &params->rate_min)) {
				dev_err(&pdev->dev,
					"Property 'srate' missing or invalid\n");
				goto err;
			}
			params->rate_max = params->rate_min;

			if (of_property_read_u32(subnp,
				"num-channel", &params->channels_min)) {
				dev_err(&pdev->dev,
					"Property 'num-channel' missing or invalid\n");
				goto err;
			}
			params->channels_max = params->channels_min;
			tegra_codec_links[i].params = params;

			of_property_read_u32(subnp,
				"rx-mask", (u32 *)&rx_mask[i]);
			of_property_read_u32(subnp,
				"tx-mask", (u32 *)&tx_mask[i]);

			/* CIF configuration */
			tegra_codec_links[j].codec_of_node =
				tegra_codec_links[i].cpu_of_node;
			tegra_codec_links[j].cpu_of_node =
				of_parse_phandle(np, "nvidia,xbar", 0);
			if (!tegra_codec_links[j].cpu_of_node) {
				dev_err(&pdev->dev,
					"Property 'nvidia,xbar' missing or invalid\n");
				goto err;
			}

			if (!strcmp(tegra_codec_links[i].name, "dspk-playback-r"))
				tegra_codec_links[j].codec_dai_name = "CIF2";
			else
				tegra_codec_links[j].codec_dai_name = "CIF";

			if (of_property_read_string(subnp, "cpu-dai-name",
				&tegra_codec_links[j].cpu_dai_name)) {
				dev_err(&pdev->dev,
					"Property 'cpu-dai-name' missing or invalid\n");
				goto err;
			}

			str = devm_kzalloc(&pdev->dev,
				sizeof(tegra_codec_links[j].cpu_dai_name) +
				1 + sizeof(tegra_codec_links[j].codec_dai_name),
				GFP_KERNEL);
			str = strcat(str, tegra_codec_links[j].cpu_dai_name);
			str = strcat(str, " ");
			str = strcat(str, tegra_codec_links[j].codec_dai_name);

			tegra_codec_links[j].name =
				tegra_codec_links[j].stream_name = str;
			tegra_codec_links[j].params =
				tegra_codec_links[i].params;
			tegra_codec_links[j].ignore_suspend =
				tegra_codec_links[i].ignore_suspend;
		} else {
			dev_err(&pdev->dev,
				"Property '%s' missing or invalid\n",
				dai_link_name);
			goto err;
		}
	}

	*pnum_codec_links = num_codec_links;

	return tegra_codec_links;
err:
	return NULL;
}
EXPORT_SYMBOL_GPL(tegra_machine_new_codec_links);


struct snd_soc_codec_conf *tegra_machine_new_codec_conf(
	struct platform_device *pdev,
	struct snd_soc_codec_conf *tegra_codec_conf,
	unsigned int *pnum_codec_links)
{
	unsigned int i, num_codec_links;
	struct device_node *np = pdev->dev.of_node, *subnp;
	struct device_node *of_node;
	char dai_link_name[MAX_STR_SIZE];

	if (tegra_codec_conf)
		return tegra_codec_conf;

	if (!np)
		goto err;

	if (of_property_read_u32(np,
		"nvidia,num-codec-link", (u32 *)&num_codec_links)) {
		dev_err(&pdev->dev,
			"Property 'nvidia,num-codec-link' missing or invalid\n");
		goto err;
	}

	tegra_codec_conf =  devm_kzalloc(&pdev->dev, num_codec_links *
		sizeof(struct snd_soc_codec_conf), GFP_KERNEL);

	for (i = 0; i < num_codec_links; i++) {
		memset((void *)dai_link_name, '\0', MAX_STR_SIZE);
		sprintf(dai_link_name, "nvidia,dai-link-%d", i+1);
		subnp = of_get_child_by_name(np, dai_link_name);
		if (subnp) {
			of_node = of_parse_phandle(subnp, "codec-dai", 0);

			/* specify device by DT/OF node,
			   rather than device name */
			tegra_codec_conf[i].dev_name = NULL;
			tegra_codec_conf[i].of_node = of_node;

			if (of_property_read_string(subnp, "name-prefix",
				&tegra_codec_conf[i].name_prefix)) {
				dev_err(&pdev->dev,
					"Property 'name-prefix' missing or invalid\n");
				goto err;
			}
		}
	}

	*pnum_codec_links = num_codec_links;

	return tegra_codec_conf;
err:
	return NULL;
}
EXPORT_SYMBOL_GPL(tegra_machine_new_codec_conf);

/* This function is valid when dai_link is initiated from the DT */
unsigned int tegra_machine_get_codec_dai_link_idx(const char *codec_name)
{
	unsigned int idx = (of_machine_is_compatible("nvidia,tegra210")   ||
			of_machine_is_compatible("nvidia,tegra210b01")) ?
			TEGRA210_XBAR_DAI_LINKS : 0;

	if (num_dai_links <= idx)
		goto err;

	while (idx < num_dai_links) {
		if (tegra_asoc_machine_links[idx].name)
			if (!strcmp(tegra_asoc_machine_links[idx].name,
				codec_name))
				return idx;
		idx++;
	}

err:
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(tegra_machine_get_codec_dai_link_idx);

unsigned int tegra_machine_get_rx_mask(
	struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai_link *codec_dai_link = rtd->dai_link;
	char *codec_name = (char *)codec_dai_link->name;
	unsigned int idx =
		tegra_machine_get_codec_dai_link_idx(codec_name);

	if (idx == -EINVAL)
		goto err;

	if (!rx_mask)
		goto err;

	idx = idx - ((of_machine_is_compatible("nvidia,tegra210") ||
			of_machine_is_compatible("nvidia,tegra210b01")) ?
			TEGRA210_XBAR_DAI_LINKS : 0);

	return rx_mask[idx];

err:
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(tegra_machine_get_rx_mask);

unsigned int tegra_machine_get_tx_mask(
	struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai_link *codec_dai_link = rtd->dai_link;
	char *codec_name = (char *)codec_dai_link->name;
	unsigned int idx =
		tegra_machine_get_codec_dai_link_idx(codec_name);

	if (idx == -EINVAL)
		goto err;

	if (!tx_mask)
		goto err;

	idx = idx - ((of_machine_is_compatible("nvidia,tegra210")   ||
			of_machine_is_compatible("nvidia,tegra210b01")) ?
			TEGRA210_XBAR_DAI_LINKS : 0);

	return tx_mask[idx];

err:
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(tegra_machine_get_tx_mask);

#if IS_ENABLED(CONFIG_SND_SOC_TEGRA210_ADSP_ALT)
void tegra_machine_remove_adsp_links_t18x(void)
{
	num_links = TEGRA186_DAI_LINK_ADSP_ADMAIF1;
}
#else
void tegra_machine_remove_adsp_links_t18x(void)
{
}
#endif

EXPORT_SYMBOL_GPL(tegra_machine_remove_adsp_links_t18x);

/* t18x specific APIs*/
struct snd_soc_dai_link *tegra_machine_get_dai_link_t18x(void)
{
	struct snd_soc_dai_link *link = tegra186_xbar_dai_links;
	unsigned int size = num_links;
	struct snd_soc_dai_link *tegra_asoc_machine_links_t18x =
		tegra_asoc_machine_links;

	if (tegra_asoc_machine_links_t18x)
		return tegra_asoc_machine_links_t18x;

	num_dai_links = size;

	tegra_asoc_machine_links_t18x = kzalloc(size *
		sizeof(struct snd_soc_dai_link), GFP_KERNEL);

	memcpy(tegra_asoc_machine_links_t18x, link,
		size * sizeof(struct snd_soc_dai_link));

	tegra_asoc_machine_links = tegra_asoc_machine_links_t18x;

	return tegra_asoc_machine_links_t18x;
}
EXPORT_SYMBOL_GPL(tegra_machine_get_dai_link_t18x);

int tegra_machine_append_dai_link_t18x(struct snd_soc_dai_link *link,
		unsigned int link_size)
{
	unsigned int size1 = num_dai_links;
	unsigned int size2 = link_size;
	struct snd_soc_dai_link *tegra_asoc_machine_links_t18x =
		tegra_asoc_machine_links;

	if (!tegra_asoc_machine_links_t18x) {
		if (link) {
			tegra_asoc_machine_links = link;
			num_dai_links = size2;
			return size2;
		} else {
			return 0;
		}
	} else {
		if (link) {
			tegra_asoc_machine_links_t18x =
				(struct snd_soc_dai_link *) krealloc(
				tegra_asoc_machine_links_t18x, (size1 + size2) *
				sizeof(struct snd_soc_dai_link), GFP_KERNEL);
			tegra_asoc_machine_links =
				tegra_asoc_machine_links_t18x;
			memcpy(&tegra_asoc_machine_links_t18x[size1], link,
				size2 * sizeof(struct snd_soc_dai_link));
			num_dai_links = size1 + size2;
			return size1+size2;
		} else {
			num_dai_links = size1;
			return size1;
		}
	}
}
EXPORT_SYMBOL_GPL(tegra_machine_append_dai_link_t18x);

struct snd_soc_codec_conf *tegra_machine_get_codec_conf_t18x(void)
{
	struct snd_soc_codec_conf *conf = tegra186_xbar_codec_conf;
	struct snd_soc_codec_conf *tegra_asoc_codec_conf_t18x =
		tegra_asoc_codec_conf;
	unsigned int size = TEGRA186_XBAR_CODEC_CONF;

	if (tegra_asoc_codec_conf_t18x)
		return tegra_asoc_codec_conf_t18x;

	tegra_asoc_codec_conf_t18x = kzalloc(size *
		sizeof(struct snd_soc_codec_conf), GFP_KERNEL);

	memcpy(tegra_asoc_codec_conf_t18x, conf,
		size * sizeof(struct snd_soc_codec_conf));

	tegra_asoc_codec_conf = tegra_asoc_codec_conf_t18x;

	return tegra_asoc_codec_conf_t18x;
}
EXPORT_SYMBOL_GPL(tegra_machine_get_codec_conf_t18x);

int tegra_machine_append_codec_conf_t18x(struct snd_soc_codec_conf *conf,
		unsigned int conf_size)
{
	unsigned int size1 = TEGRA186_XBAR_CODEC_CONF;
	unsigned int size2 = conf_size;
	struct snd_soc_codec_conf *tegra_asoc_codec_conf_t18x =
		tegra_asoc_codec_conf;

	if (!tegra_asoc_codec_conf_t18x) {
		if (conf) {
			tegra_asoc_codec_conf = conf;
			return size2;
		} else {
			return 0;
		}
	} else {
		if (conf) {
			tegra_asoc_codec_conf_t18x =
				(struct snd_soc_codec_conf *) krealloc(
				tegra_asoc_codec_conf_t18x, (size1 + size2) *
				sizeof(struct snd_soc_codec_conf), GFP_KERNEL);
			tegra_asoc_codec_conf = tegra_asoc_codec_conf_t18x;
			memcpy(&tegra_asoc_codec_conf_t18x[size1], conf,
				size2 * sizeof(struct snd_soc_codec_conf));
			return size1+size2;
		} else
			return size1;
	}
}
EXPORT_SYMBOL_GPL(tegra_machine_append_codec_conf_t18x);

unsigned int tegra_machine_get_codec_dai_link_idx_t18x(const char *codec_name)
{
	unsigned int idx = num_links;
	struct snd_soc_dai_link *tegra_asoc_machine_links_t18x =
		tegra_asoc_machine_links;

	if (num_dai_links <= idx)
		goto err;

	while (idx < num_dai_links) {
		if (tegra_asoc_machine_links_t18x[idx].name)
			if (!strcmp(tegra_asoc_machine_links_t18x[idx].name,
				codec_name))
				return idx;
		idx++;
	}

err:
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(tegra_machine_get_codec_dai_link_idx_t18x);

unsigned int tegra_machine_get_rx_mask_t18x(
	struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai_link *codec_dai_link = rtd->dai_link;
	char *codec_name = (char *)codec_dai_link->name;
	unsigned int idx =
		tegra_machine_get_codec_dai_link_idx_t18x(codec_name);
	unsigned int *rx_mask_t18x = rx_mask;

	if (idx == -EINVAL)
		goto err;

	if (!rx_mask_t18x)
		goto err;

	idx = idx - num_links;

	return rx_mask_t18x[idx];

err:
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(tegra_machine_get_rx_mask_t18x);

unsigned int tegra_machine_get_tx_mask_t18x(
	struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai_link *codec_dai_link = rtd->dai_link;
	char *codec_name = (char *)codec_dai_link->name;
	unsigned int idx =
		tegra_machine_get_codec_dai_link_idx_t18x(codec_name);
	unsigned int *tx_mask_t18x = tx_mask;

	if (idx == -EINVAL)
		goto err;

	if (!tx_mask_t18x)
		goto err;

	idx = idx - num_links;

	return tx_mask_t18x[idx];

err:
	return -EINVAL;
}
EXPORT_SYMBOL_GPL(tegra_machine_get_tx_mask_t18x);

struct tegra_machine_control_data {
	struct snd_soc_pcm_runtime *rtd;
	unsigned int frame_mode;
	unsigned int master_mode;
};

static int tegra_machine_codec_set_dai_fmt(struct snd_soc_pcm_runtime *rtd,
					   unsigned int frame_mode,
					   unsigned int master_mode)
{
	unsigned int fmt = rtd->dai_link->dai_fmt;

	if (frame_mode) {
		fmt &= ~SND_SOC_DAIFMT_FORMAT_MASK;
		fmt |= frame_mode;
	}

	if (master_mode) {
		fmt &= ~SND_SOC_DAIFMT_MASTER_MASK;
		master_mode <<= ffs(SND_SOC_DAIFMT_MASTER_MASK) - 1;

		if (master_mode == SND_SOC_DAIFMT_CBM_CFM)
			fmt |= SND_SOC_DAIFMT_CBM_CFM;
		else
			fmt |= SND_SOC_DAIFMT_CBS_CFS;
	}

	return snd_soc_runtime_set_dai_fmt(rtd, fmt);
}

/*
 * The order of the below must not be changed as this
 * aligns with the SND_SOC_DAIFMT_XXX definitions in
 * include/sound/soc-dai.h.
 */
static const char * const tegra_machine_frame_mode_text[] = {
	"None",
	"i2s",
	"right-j",
	"left-j",
	"dsp-a",
	"dsp-b",
};

static int tegra_machine_codec_get_frame_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_machine_control_data *data = kcontrol->private_data;

	ucontrol->value.integer.value[0] = data->frame_mode;

	return 0;
}

static int tegra_machine_codec_put_frame_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_machine_control_data *data = kcontrol->private_data;
	int err;

	err = tegra_machine_codec_set_dai_fmt(data->rtd,
					      ucontrol->value.integer.value[0],
					      data->master_mode);
	if (err)
		return err;

	data->frame_mode = ucontrol->value.integer.value[0];

	return 0;
}

static const char * const tegra_machine_master_mode_text[] = {
	"None",
	"cbm-cfm",
	"cbs-cfs",
};

static int tegra_machine_codec_get_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_machine_control_data *data = kcontrol->private_data;

	ucontrol->value.integer.value[0] = data->master_mode;

	return 0;
}

static int tegra_machine_codec_put_master_mode(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct tegra_machine_control_data *data = kcontrol->private_data;
	int err;

	err = tegra_machine_codec_set_dai_fmt(data->rtd,
					      data->frame_mode,
					      ucontrol->value.integer.value[0]);
	if (err)
		return err;

	data->master_mode = ucontrol->value.integer.value[0];

	return 0;
}

static const struct soc_enum tegra_machine_codec_frame_mode =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tegra_machine_frame_mode_text),
		tegra_machine_frame_mode_text);

static const struct soc_enum tegra_machine_codec_master_mode =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tegra_machine_master_mode_text),
		tegra_machine_master_mode_text);

static int tegra_machine_add_ctl(struct snd_soc_card *card,
				 struct snd_kcontrol_new *knew,
				 void *private_data,
				 const unsigned char *name)
{
	struct snd_kcontrol *kctl;
	int ret;

	kctl = snd_ctl_new1(knew, private_data);
	if (!kctl)
		return -ENOMEM;

	ret = snd_ctl_add(card->snd_card, kctl);
	if (ret < 0)
		return ret;

	return 0;
}

static int tegra_machine_add_frame_mode_ctl(struct snd_soc_card *card,
	struct snd_soc_pcm_runtime *rtd, const unsigned char *name,
	struct tegra_machine_control_data *data)
{
	struct snd_kcontrol_new knew = {
		.iface		= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name		= name,
		.info		= snd_soc_info_enum_double,
		.index		= 0,
		.get		= tegra_machine_codec_get_frame_mode,
		.put		= tegra_machine_codec_put_frame_mode,
		.private_value	=
				(unsigned long)&tegra_machine_codec_frame_mode,
	};

	return tegra_machine_add_ctl(card, &knew, data, name);
}

static int tegra_machine_add_master_mode_ctl(struct snd_soc_card *card,
	struct snd_soc_pcm_runtime *rtd, const unsigned char *name,
	struct tegra_machine_control_data *data)
{
	struct snd_kcontrol_new knew = {
		.iface		= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name		= name,
		.info		= snd_soc_info_enum_double,
		.index		= 0,
		.get		= tegra_machine_codec_get_master_mode,
		.put		= tegra_machine_codec_put_master_mode,
		.private_value	=
				(unsigned long)&tegra_machine_codec_master_mode,
	};

	return tegra_machine_add_ctl(card, &knew, data, name);
}

int tegra_machine_add_i2s_codec_controls(struct snd_soc_card *card,
					 unsigned int num_dai_links)
{
	struct tegra_machine_control_data *data;
	struct snd_soc_pcm_runtime *rtd;
	struct device_node *np;
	char name[SNDRV_CTL_ELEM_ID_NAME_MAXLEN];
	unsigned int id;
	int ret;
#if KERNEL_VERSION(4, 5, 0) > LINUX_VERSION_CODE
	unsigned int i;

	for (i = 0; i < num_dai_links; i++) {
		rtd = &card->rtd[i];
#else
	list_for_each_entry(rtd, &card->rtd_list, list) {
#endif
		np = rtd->dai_link->cpu_of_node;

		if (!np)
			continue;

		data = devm_kzalloc(card->dev, sizeof(*data), GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		data->rtd = rtd;
		data->frame_mode = 0;
		data->master_mode = 0;

		if (of_property_read_u32(np, "nvidia,ahub-i2s-id", &id) < 0)
			continue;

		snprintf(name, sizeof(name), "I2S%d codec frame mode", id+1);

		ret = tegra_machine_add_frame_mode_ctl(card, rtd, name, data);
		if (ret)
			dev_warn(card->dev, "Failed to add control: %s!\n",
				 name);

		snprintf(name, sizeof(name), "I2S%d codec master mode", id+1);

		ret = tegra_machine_add_master_mode_ctl(card, rtd, name, data);
		if (ret) {
			dev_warn(card->dev, "Failed to add control: %s!\n",
				 name);
			return ret;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_machine_add_i2s_codec_controls);

/*
 * The order of the following definitions should align with
 * the 'snd_jack_types' enum as defined in include/sound/jack.h.
 */
static const char * const tegra_machine_jack_state_text[] = {
	"None",
	"HP",
	"MIC",
	"HS",
};

static const struct soc_enum tegra_machine_jack_state =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(tegra_machine_jack_state_text),
		tegra_machine_jack_state_text);

static int tegra_machine_codec_get_jack_state(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_jack *jack = kcontrol->private_data;

	ucontrol->value.integer.value[0] = jack->status;

	return 0;
}

static int tegra_machine_codec_put_jack_state(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_jack *jack = kcontrol->private_data;

	snd_soc_jack_report(jack, ucontrol->value.integer.value[0],
			    SND_JACK_HEADSET);

	return 0;
}

int tegra_machine_add_codec_jack_control(struct snd_soc_card *card,
					 struct snd_soc_pcm_runtime *rtd,
					 struct snd_soc_jack *jack)
{
	char name[SNDRV_CTL_ELEM_ID_NAME_MAXLEN];
	struct snd_kcontrol_new knew = {
		.iface		= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name		= name,
		.info		= snd_soc_info_enum_double,
		.index		= 0,
		.get		= tegra_machine_codec_get_jack_state,
		.put		= tegra_machine_codec_put_jack_state,
		.private_value	= (unsigned long)&tegra_machine_jack_state,
	};

	if (rtd->codec->component.name_prefix)
		snprintf(name, sizeof(name), "%s Jack-state",
			 rtd->codec->component.name_prefix);
	else
		snprintf(name, sizeof(name), "Jack-state");

	return tegra_machine_add_ctl(card, &knew, jack, name);
}
EXPORT_SYMBOL_GPL(tegra_machine_add_codec_jack_control);

void tegra_machine_dma_set_mask(struct platform_device *pdev)
{
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
	struct device_node *np = pdev->dev.of_node;
	uint64_t dma_mask;
	int ret;

	ret = of_property_read_u64(np, "dma-mask", &dma_mask);
	if (ret)
		dev_err(&pdev->dev, "Missing property dma-mask\n");
	else
		dma_set_mask_and_coherent(&pdev->dev, dma_mask);
#endif
}
EXPORT_SYMBOL_GPL(tegra_machine_dma_set_mask);

void release_asoc_phandles(struct tegra_machine *machine)
{
	unsigned int i;

	if (machine->asoc->dai_links) {
		for (i = 0; i < machine->asoc->num_links; i++) {
			of_node_put(machine->asoc->dai_links[i].cpu_of_node);
			of_node_put(machine->asoc->dai_links[i].codec_of_node);
		}
	}

	if (machine->asoc->codec_confs) {
		for (i = 0; i < machine->asoc->num_confs; i++)
			of_node_put(machine->asoc->codec_confs[i].of_node);
	}
}
EXPORT_SYMBOL_GPL(release_asoc_phandles);

int tegra_asoc_populate_dai_links(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node, *subnp = NULL;
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_machine *machine = snd_soc_card_get_drvdata(card);
	struct snd_soc_dai_link *dai_links, *ahub_links;
	struct snd_soc_pcm_stream *params;
	char dai_link_name[MAX_STR_SIZE], *str;
	unsigned int num_codec_links, num_ahub_links, num_links,
		link_count = 0, i, j, *rx_slot, *tx_slot;
	int ret;

	num_ahub_links = machine->soc_data->num_ahub_links;
	ahub_links = machine->soc_data->ahub_links;

	if (!np || !num_ahub_links || !ahub_links)
		return -EINVAL;

	/* read number of codec links exposed via DT */
	ret = of_property_read_u32(np, "nvidia,num-codec-link",
				   &machine->num_codec_links);
	if (ret < 0) {
		dev_err(&pdev->dev,
			"Property 'nvidia,num-codec-link' missing\n");
		return ret;
	}
	num_codec_links = machine->num_codec_links;

	/*
	 * each codec link specified in device tree will result into one DAP
	 * and one CIF link. For example, i2s dai link will look like below.
	 * ahub <----CIF----> i2s <----DAP----> codec
	 */
	num_links = num_ahub_links + (num_codec_links << 1);

	machine->asoc->num_links = num_links;
	machine->asoc->dai_links = devm_kzalloc(&pdev->dev,
						sizeof(*dai_links) * num_links,
						GFP_KERNEL);
	dai_links = machine->asoc->dai_links;
	if (!dai_links)
		return -ENOMEM;

	machine->asoc->rx_slot = devm_kzalloc(&pdev->dev,
					      sizeof(*rx_slot) * num_links,
					      GFP_KERNEL);
	rx_slot = machine->asoc->rx_slot;
	if (!rx_slot)
		return -ENOMEM;

	machine->asoc->tx_slot = devm_kzalloc(&pdev->dev,
					      sizeof(*tx_slot) * num_links,
					      GFP_KERNEL);
	tx_slot = machine->asoc->tx_slot;
	if (!tx_slot)
		return -ENOMEM;

	/* populate now ahub links */
	memcpy(dai_links, ahub_links, num_ahub_links * sizeof(*dai_links));

	/* populate now CIF and DAP links from device tree */
	for (i = num_ahub_links, j = num_ahub_links + num_codec_links;
	     i < num_ahub_links + num_codec_links; i++, j++) {
		memset((void *)dai_link_name, '\0', MAX_STR_SIZE);
		sprintf(dai_link_name, "nvidia,dai-link-%d", ++link_count);
		subnp = of_get_child_by_name(np, dai_link_name);
		if (!subnp)
			return -ENOENT;

		/* DAP DAI link configuration */
		dai_links[i].stream_name = "Playback";
		dai_links[i].codec_of_node = of_parse_phandle(subnp,
							      "codec-dai", 0);
		if (!dai_links[i].codec_of_node) {
			dev_err(&pdev->dev,
				"property 'codec-dai' is missing\n");
			ret = -ENOENT;
			break;
		}

		dai_links[i].cpu_of_node = of_parse_phandle(subnp, "cpu-dai",
							    0);
		if (!dai_links[i].cpu_of_node) {
			dev_err(&pdev->dev, "property 'cpu-dai' is missing\n");
			ret = -ENOENT;
			break;
		}

		of_property_read_string(subnp, "link-name", &dai_links[i].name);

		/*
		 * special case for DSPK
		 * Two mono codecs can be connected to the controller
		 * DAP2 is required for DAPM path completion
		 * TODO revisit this when ASoC has multi-codec support
		 */
		if (!strcmp(dai_links[i].name, "dspk-playback-r"))
			dai_links[i].cpu_dai_name = "DAP2";
		else
			dai_links[i].cpu_dai_name = "DAP";
		dai_links[i].dai_fmt = snd_soc_of_parse_daifmt(subnp, NULL,
							       NULL, NULL);

		params = devm_kzalloc(&pdev->dev, sizeof(*params), GFP_KERNEL);
		if (!params) {
			ret = -ENOMEM;
			break;
		}

		ret = of_property_read_string(subnp, "bit-format",
					      (const char **)&str);
		if (ret < 0) {
			dev_err(&pdev->dev, "Property 'bit-format' missing\n");
			break;
		}

		ret = tegra_machine_get_format(&params->formats, str);
		if (ret < 0) {
			dev_err(&pdev->dev, "Wrong codec format\n");
			break;
		}

		ret = of_property_read_u32(subnp, "srate", &params->rate_min);
		if (ret < 0) {
			dev_err(&pdev->dev, "Property 'srate' missing\n");
			break;
		}
		params->rate_max = params->rate_min;

		ret = of_property_read_u32(subnp, "num-channel",
					   &params->channels_min);
		if (ret < 0) {
			dev_err(&pdev->dev, "Property 'num-channel' missing\n");
			break;
		}
		params->channels_max = params->channels_min;

		dai_links[i].params = params;
		ret = of_property_read_string(subnp, "codec-dai-name",
					      &dai_links[i].codec_dai_name);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"property 'codec-dai-name' is missing\n");
			break;
		}

		of_property_read_u32(subnp, "rx-mask", (u32 *)&rx_slot[i]);
		of_property_read_u32(subnp, "tx-mask", (u32 *)&tx_slot[i]);

		/*
		 * CIF DAI link configuration
		 * CIF link is towards XBAR, hence xbar node is cpu_of_node
		 * and codec_of_node is same as DAP's cpu_of_node.
		 */
		dai_links[j].codec_of_node = of_parse_phandle(subnp, "cpu-dai",
							      0);
		dai_links[j].cpu_of_node = of_parse_phandle(np, "nvidia,xbar",
							    0);
		if (!dai_links[j].cpu_of_node) {
			dev_err(&pdev->dev,
				"property 'nvidia,xbar' is missing\n");
			ret = -ENOENT;
			break;
		}

		/*
		 * special case for DSPK
		 * Two mono codecs can be connected to the controller
		 * CIF2 is required for DAPM path completion
		 * TODO revist this when ASoC has multi-codec support
		 */
		if (!strcmp(dai_links[i].name, "dspk-playback-r"))
			dai_links[j].codec_dai_name = "CIF2";
		else
			dai_links[j].codec_dai_name = "CIF";

		ret = of_property_read_string(subnp, "cpu-dai-name",
					      &dai_links[j].cpu_dai_name);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"property 'cpu-dai-name' is missing\n");
			break;
		}

		str = devm_kzalloc(&pdev->dev,
			sizeof(dai_links[j].cpu_dai_name) +
			1 + sizeof(dai_links[j].codec_dai_name),
			GFP_KERNEL);
		str = strcat(str, dai_links[j].cpu_dai_name);
		str = strcat(str, " ");
		str = strcat(str, dai_links[j].codec_dai_name);

		dai_links[j].name = dai_links[j].stream_name = str;
		dai_links[j].params = dai_links[i].params;

		of_node_put(subnp);
	}

	/*
	 * release subnp here. DAI links and codec conf release will be
	 * taken care during error exit of machine driver probe()
	 */
	if (ret < 0 && subnp) {
		of_node_put(subnp);
		return ret;
	}

	card->num_links = num_links;
	card->dai_link = dai_links;

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_asoc_populate_dai_links);

int tegra_asoc_populate_codec_confs(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct tegra_machine *machine = snd_soc_card_get_drvdata(card);
	unsigned int num_codec_confs, num_ahub_confs, num_confs, i;
	struct snd_soc_codec_conf *codec_confs, *ahub_confs;
	char dai_link_name[MAX_STR_SIZE];
	struct device_node *of_node;
	struct device_node *np = pdev->dev.of_node, *subnp;

	ahub_confs = machine->soc_data->ahub_confs;
	num_ahub_confs = machine->soc_data->num_ahub_confs;
	num_codec_confs = machine->num_codec_links;
	if (!ahub_confs || !num_codec_confs || !num_ahub_confs)
		return -EINVAL;
	num_confs = num_codec_confs + num_ahub_confs;
	machine->asoc->num_confs = num_confs;

	machine->asoc->codec_confs =
		devm_kzalloc(&pdev->dev, sizeof(*codec_confs) * num_confs,
			     GFP_KERNEL);
	codec_confs = machine->asoc->codec_confs;
	if (!codec_confs)
		return -ENOMEM;

	/* add codec confs from ahub */
	memcpy(codec_confs, ahub_confs, num_ahub_confs * sizeof(*codec_confs));

	/* append codec confs from device tree */
	for (i = 0; i < num_codec_confs; i++) {
		memset((void *)dai_link_name, '\0', MAX_STR_SIZE);
		sprintf(dai_link_name, "nvidia,dai-link-%d", i+1);
		subnp = of_get_child_by_name(np, dai_link_name);
		if (!subnp)
			return -ENOENT;

		of_node = of_parse_phandle(subnp, "codec-dai", 0);
		if (!of_node) {
			dev_err(&pdev->dev,
				"property 'codec-dai' is missing\n");
			of_node_put(subnp);
			return -ENOENT;
		}

		codec_confs[i + num_ahub_confs].dev_name = NULL;
		codec_confs[i + num_ahub_confs].of_node = of_node;
		of_property_read_string(subnp, "name-prefix",
			&codec_confs[i + num_ahub_confs].name_prefix);

		of_node_put(subnp);
	}

	card->num_configs = num_confs;
	card->codec_conf = codec_confs;

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_asoc_populate_codec_confs);

MODULE_AUTHOR("Arun Shamanna Lakshmi <aruns@nvidia.com>");
MODULE_AUTHOR("Junghyun Kim <juskim@nvidia.com>");
MODULE_LICENSE("GPL");
