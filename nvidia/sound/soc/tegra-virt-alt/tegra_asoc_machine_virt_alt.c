/*
 * tegra_asoc_machine_virt_alt.c - Tegra xbar dai link for machine drivers
 *
 * Copyright (c) 2017-2019 NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/of.h>
#include <linux/export.h>
#include <sound/soc.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/tegra_pm_domains.h>


#include "tegra_asoc_machine_virt_alt.h"

#define CODEC_NAME		NULL

#define DAI_NAME(i)		"AUDIO" #i
#define STREAM_NAME		"playback"
#define LINK_CPU_NAME		DRV_NAME
#define CPU_DAI_NAME(i)		"ADMAIF" #i
#define CODEC_DAI_NAME		"dit-hifi"
#define PLATFORM_NAME		LINK_CPU_NAME

static unsigned int num_dai_links;
static const struct snd_soc_pcm_stream default_params = {
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};

static const struct snd_soc_pcm_stream adsp_default_params = {
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};

static struct snd_soc_pcm_stream adsp_admaif_params[MAX_ADMAIF_IDS];

static struct snd_soc_dai_link tegra_virt_t186ref_pcm_links[] = {
	{
		/* 0 */
		.name = DAI_NAME(1),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(1),
		.codec_dai_name = "ADMAIF1 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		/* 1 */
		.name = DAI_NAME(2),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(2),
		.codec_dai_name = "ADMAIF2 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		/* 2 */
		.name = DAI_NAME(3),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(3),
		.codec_dai_name = "ADMAIF3 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		/* 3 */
		.name = DAI_NAME(4),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(4),
		.codec_dai_name = "ADMAIF4 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		/* 4 */
		.name = DAI_NAME(5),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(5),
		.codec_dai_name = "ADMAIF5 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		/* 5 */
		.name = DAI_NAME(6),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(6),
		.codec_dai_name = "ADMAIF6 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		/* 6 */
		.name = DAI_NAME(7),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(7),
		.codec_dai_name = "ADMAIF7 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		/* 7 */
		.name = DAI_NAME(8),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(8),
		.codec_dai_name = "ADMAIF8 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		/* 8 */
		.name = DAI_NAME(9),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(9),
		.codec_dai_name = "ADMAIF9 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		/* 9 */
		.name = DAI_NAME(10),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(10),
		.codec_dai_name = "ADMAIF10 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		/* 10 */
		.name = DAI_NAME(11),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(11),
		.codec_dai_name = "ADMAIF11 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		/* 11 */
		.name = DAI_NAME(12),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(12),
		.codec_dai_name = "ADMAIF12 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		/* 12 */
		.name = DAI_NAME(13),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(13),
		.codec_dai_name = "ADMAIF13 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		/* 13 */
		.name = DAI_NAME(14),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(14),
		.codec_dai_name = "ADMAIF14 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		/* 14 */
		.name = DAI_NAME(15),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(15),
		.codec_dai_name = "ADMAIF15 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		/* 15 */
		.name = DAI_NAME(16),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(16),
		.codec_dai_name = "ADMAIF16 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		/* 16 */
		.name = DAI_NAME(17),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(17),
		.codec_dai_name = "ADMAIF17 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		/* 17 */
		.name = DAI_NAME(18),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(18),
		.codec_dai_name = "ADMAIF18 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		/* 18 */
		.name = DAI_NAME(19),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(19),
		.codec_dai_name = "ADMAIF19 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		/* 19 */
		.name = DAI_NAME(20),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(20),
		.codec_dai_name = "ADMAIF20 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		/* 20 */
		.name = "ADSP ADMAIF1",
		.stream_name = "ADSP AFMAIF1",
		.codec_name = LINK_CPU_NAME,
		.cpu_name = "tegra210-adsp-virt",
		.cpu_dai_name = "ADSP-ADMAIF1",
		.codec_dai_name = "ADMAIF1 CIF",
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 21 */
		.name = "ADSP ADMAIF2",
		.stream_name = "ADSP AFMAIF2",
		.codec_name = LINK_CPU_NAME,
		.cpu_name = "tegra210-adsp-virt",
		.cpu_dai_name = "ADSP-ADMAIF2",
		.codec_dai_name = "ADMAIF2 CIF",
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 22 */
		.name = "ADSP ADMAIF3",
		.stream_name = "ADSP AFMAIF3",
		.codec_name = LINK_CPU_NAME,
		.cpu_name = "tegra210-adsp-virt",
		.cpu_dai_name = "ADSP-ADMAIF3",
		.codec_dai_name = "ADMAIF3 CIF",
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 23 */
		.name = "ADSP ADMAIF4",
		.stream_name = "ADSP AFMAIF4",
		.codec_name = LINK_CPU_NAME,
		.cpu_name = "tegra210-adsp-virt",
		.cpu_dai_name = "ADSP-ADMAIF4",
		.codec_dai_name = "ADMAIF4 CIF",
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 24 */
		.name = "ADSP ADMAIF5",
		.stream_name = "ADSP AFMAIF5",
		.codec_name = LINK_CPU_NAME,
		.cpu_name = "tegra210-adsp-virt",
		.cpu_dai_name = "ADSP-ADMAIF5",
		.codec_dai_name = "ADMAIF5 CIF",
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 25 */
		.name = "ADSP ADMAIF6",
		.stream_name = "ADSP AFMAIF6",
		.codec_name = LINK_CPU_NAME,
		.cpu_name = "tegra210-adsp-virt",
		.cpu_dai_name = "ADSP-ADMAIF6",
		.codec_dai_name = "ADMAIF6 CIF",
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 26 */
		.name = "ADSP ADMAIF7",
		.stream_name = "ADSP AFMAIF7",
		.codec_name = LINK_CPU_NAME,
		.cpu_name = "tegra210-adsp-virt",
		.cpu_dai_name = "ADSP-ADMAIF7",
		.codec_dai_name = "ADMAIF7 CIF",
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 27 */
		.name = "ADSP ADMAIF8",
		.stream_name = "ADSP AFMAIF8",
		.codec_name = LINK_CPU_NAME,
		.cpu_name = "tegra210-adsp-virt",
		.cpu_dai_name = "ADSP-ADMAIF8",
		.codec_dai_name = "ADMAIF8 CIF",
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 28 */
		.name = "ADSP ADMAIF9",
		.stream_name = "ADSP AFMAIF9",
		.codec_name = LINK_CPU_NAME,
		.cpu_name = "tegra210-adsp-virt",
		.cpu_dai_name = "ADSP-ADMAIF9",
		.codec_dai_name = "ADMAIF9 CIF",
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 29 */
		.name = "ADSP ADMAIF10",
		.stream_name = "ADSP AFMAIF10",
		.codec_name = LINK_CPU_NAME,
		.cpu_name = "tegra210-adsp-virt",
		.cpu_dai_name = "ADSP-ADMAIF10",
		.codec_dai_name = "ADMAIF10 CIF",
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 30 */
		.name = "ADSP ADMAIF11",
		.stream_name = "ADSP AFMAIF11",
		.codec_name = LINK_CPU_NAME,
		.cpu_name = "tegra210-adsp-virt",
		.cpu_dai_name = "ADSP-ADMAIF11",
		.codec_dai_name = "ADMAIF11 CIF",
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 31 */
		.name = "ADSP ADMAIF12",
		.stream_name = "ADSP AFMAIF12",
		.codec_name = LINK_CPU_NAME,
		.cpu_name = "tegra210-adsp-virt",
		.cpu_dai_name = "ADSP-ADMAIF12",
		.codec_dai_name = "ADMAIF12 CIF",
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 32 */
		.name = "ADSP ADMAIF13",
		.stream_name = "ADSP AFMAIF13",
		.codec_name = LINK_CPU_NAME,
		.cpu_name = "tegra210-adsp-virt",
		.cpu_dai_name = "ADSP-ADMAIF13",
		.codec_dai_name = "ADMAIF13 CIF",
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 33 */
		.name = "ADSP ADMAIF14",
		.stream_name = "ADSP AFMAIF14",
		.codec_name = LINK_CPU_NAME,
		.cpu_name = "tegra210-adsp-virt",
		.cpu_dai_name = "ADSP-ADMAIF14",
		.codec_dai_name = "ADMAIF14 CIF",
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 34 */
		.name = "ADSP ADMAIF15",
		.stream_name = "ADSP AFMAIF15",
		.codec_name = LINK_CPU_NAME,
		.cpu_name = "tegra210-adsp-virt",
		.cpu_dai_name = "ADSP-ADMAIF15",
		.codec_dai_name = "ADMAIF15 CIF",
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 35 */
		.name = "ADSP ADMAIF16",
		.stream_name = "ADSP AFMAIF16",
		.codec_name = LINK_CPU_NAME,
		.cpu_name = "tegra210-adsp-virt",
		.cpu_dai_name = "ADSP-ADMAIF16",
		.codec_dai_name = "ADMAIF16 CIF",
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 36 */
		.name = "ADSP ADMAIF17",
		.stream_name = "ADSP AFMAIF17",
		.codec_name = LINK_CPU_NAME,
		.cpu_name = "tegra210-adsp-virt",
		.cpu_dai_name = "ADSP-ADMAIF17",
		.codec_dai_name = "ADMAIF17 CIF",
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 37 */
		.name = "ADSP ADMAIF18",
		.stream_name = "ADSP AFMAIF18",
		.codec_name = LINK_CPU_NAME,
		.cpu_name = "tegra210-adsp-virt",
		.cpu_dai_name = "ADSP-ADMAIF18",
		.codec_dai_name = "ADMAIF18 CIF",
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 38 */
		.name = "ADSP ADMAIF19",
		.stream_name = "ADSP AFMAIF19",
		.codec_name = LINK_CPU_NAME,
		.cpu_name = "tegra210-adsp-virt",
		.cpu_dai_name = "ADSP-ADMAIF19",
		.codec_dai_name = "ADMAIF19 CIF",
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 39 */
		.name = "ADSP ADMAIF20",
		.stream_name = "ADSP AFMAIF20",
		.codec_name = LINK_CPU_NAME,
		.cpu_name = "tegra210-adsp-virt",
		.cpu_dai_name = "ADSP-ADMAIF20",
		.codec_dai_name = "ADMAIF20 CIF",
		.params = &adsp_default_params,
		.ignore_suspend = 1,
	},
	{
		/* 40 */
		.name = "ADSP PCM1",
		.stream_name = "ADSP PCM1",
		.codec_name = "tegra210-adsp-virt",
		.cpu_name = "tegra210-adsp-virt",
		.cpu_dai_name = "ADSP PCM1",
		.codec_dai_name = "ADSP-FE1",
		.platform_name = "tegra210-adsp-virt",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		/* 41 */
		.name = "ADSP PCM2",
		.stream_name = "ADSP PCM2",
		.codec_name = "tegra210-adsp-virt",
		.cpu_name = "tegra210-adsp-virt",
		.cpu_dai_name = "ADSP PCM2",
		.codec_dai_name = "ADSP-FE2",
		.platform_name = "tegra210-adsp-virt",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		.name = "ADSP PCM3",
		.stream_name = "ADSP PCM3",
		.cpu_dai_name = "ADSP PCM3",
		.codec_dai_name = "ADSP-FE3",
		.cpu_name = "tegra210-adsp-virt",
		.codec_name = "tegra210-adsp-virt",
		.platform_name = "tegra210-adsp-virt",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		.name = "ADSP PCM4",
		.stream_name = "ADSP PCM4",
		.cpu_dai_name = "ADSP PCM4",
		.codec_dai_name = "ADSP-FE4",
		.cpu_name = "tegra210-adsp-virt",
		.codec_name = "tegra210-adsp-virt",
		.platform_name = "tegra210-adsp-virt",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		.name = "ADSP PCM5",
		.stream_name = "ADSP PCM5",
		.cpu_dai_name = "ADSP PCM5",
		.codec_dai_name = "ADSP-FE5",
		.cpu_name = "tegra210-adsp-virt",
		.codec_name = "tegra210-adsp-virt",
		.platform_name = "tegra210-adsp-virt",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		.name = "ADSP PCM6",
		.stream_name = "ADSP PCM6",
		.cpu_dai_name = "ADSP PCM6",
		.codec_dai_name = "ADSP-FE6",
		.cpu_name = "tegra210-adsp-virt",
		.codec_name = "tegra210-adsp-virt",
		.platform_name = "tegra210-adsp-virt",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		.name = "ADSP PCM7",
		.stream_name = "ADSP PCM7",
		.cpu_dai_name = "ADSP PCM7",
		.codec_dai_name = "ADSP-FE7",
		.cpu_name = "tegra210-adsp-virt",
		.codec_name = "tegra210-adsp-virt",
		.platform_name = "tegra210-adsp-virt",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		.name = "ADSP PCM8",
		.stream_name = "ADSP PCM8",
		.cpu_dai_name = "ADSP PCM8",
		.codec_dai_name = "ADSP-FE8",
		.cpu_name = "tegra210-adsp-virt",
		.codec_name = "tegra210-adsp-virt",
		.platform_name = "tegra210-adsp-virt",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		.name = "ADSP PCM9",
		.stream_name = "ADSP PCM9",
		.cpu_dai_name = "ADSP PCM9",
		.codec_dai_name = "ADSP-FE9",
		.cpu_name = "tegra210-adsp-virt",
		.codec_name = "tegra210-adsp-virt",
		.platform_name = "tegra210-adsp-virt",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		.name = "ADSP PCM10",
		.stream_name = "ADSP PCM10",
		.cpu_dai_name = "ADSP PCM10",
		.codec_dai_name = "ADSP-FE10",
		.cpu_name = "tegra210-adsp-virt",
		.codec_name = "tegra210-adsp-virt",
		.platform_name = "tegra210-adsp-virt",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		.name = "ADSP PCM11",
		.stream_name = "ADSP PCM11",
		.cpu_dai_name = "ADSP PCM11",
		.codec_dai_name = "ADSP-FE11",
		.cpu_name = "tegra210-adsp-virt",
		.codec_name = "tegra210-adsp-virt",
		.platform_name = "tegra210-adsp-virt",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		.name = "ADSP PCM12",
		.stream_name = "ADSP PCM12",
		.cpu_dai_name = "ADSP PCM12",
		.codec_dai_name = "ADSP-FE12",
		.cpu_name = "tegra210-adsp-virt",
		.codec_name = "tegra210-adsp-virt",
		.platform_name = "tegra210-adsp-virt",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		.name = "ADSP PCM13",
		.stream_name = "ADSP PCM13",
		.cpu_dai_name = "ADSP PCM13",
		.codec_dai_name = "ADSP-FE13",
		.cpu_name = "tegra210-adsp-virt",
		.codec_name = "tegra210-adsp-virt",
		.platform_name = "tegra210-adsp-virt",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		.name = "ADSP PCM14",
		.stream_name = "ADSP PCM14",
		.cpu_dai_name = "ADSP PCM14",
		.codec_dai_name = "ADSP-FE14",
		.cpu_name = "tegra210-adsp-virt",
		.codec_name = "tegra210-adsp-virt",
		.platform_name = "tegra210-adsp-virt",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
	{
		.name = "ADSP PCM15",
		.stream_name = "ADSP PCM15",
		.cpu_dai_name = "ADSP PCM15",
		.codec_dai_name = "ADSP-FE15",
		.cpu_name = "tegra210-adsp-virt",
		.codec_name = "tegra210-adsp-virt",
		.platform_name = "tegra210-adsp-virt",
		.ignore_pmdown_time = 1,
		.ignore_suspend = 0,
	},
};

static struct snd_soc_dai_link tegra_virt_t210ref_pcm_links[] = {
	{
		/* 0 */
		.name = DAI_NAME(1),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(1),
		.codec_dai_name = "ADMAIF1 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 1 */
		.name = DAI_NAME(2),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(2),
		.codec_dai_name = "ADMAIF1 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 2 */
		.name = DAI_NAME(3),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(3),
		.codec_dai_name = "ADMAIF1 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 3 */
		.name = DAI_NAME(4),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(4),
		.codec_dai_name = "ADMAIF1 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 4 */
		.name = DAI_NAME(5),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(5),
		.codec_dai_name = "ADMAIF1 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 5 */
		.name = DAI_NAME(6),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(6),
		.codec_dai_name = "ADMAIF1 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 6 */
		.name = DAI_NAME(7),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(7),
		.codec_dai_name = "ADMAIF1 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 7 */
		.name = DAI_NAME(8),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(8),
		.codec_dai_name = "ADMAIF1 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 8 */
		.name = DAI_NAME(9),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(9),
		.codec_dai_name = "ADMAIF1 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
	{
		/* 9 */
		.name = DAI_NAME(10),
		.stream_name = STREAM_NAME,
		.codec_name = LINK_CPU_NAME,
		.cpu_name = LINK_CPU_NAME,
		.cpu_dai_name = CPU_DAI_NAME(10),
		.codec_dai_name = "ADMAIF1 CIF",
		.platform_name = PLATFORM_NAME,
		.params = &default_params,
		.ignore_pmdown_time = 1,
		.ignore_suspend = 1,
	},
};

void tegra_virt_machine_set_num_dai_links(unsigned int val)
{
	num_dai_links = val;
}
EXPORT_SYMBOL(tegra_virt_machine_set_num_dai_links);

unsigned int tegra_virt_machine_get_num_dai_links(void)
{
	return num_dai_links;
}
EXPORT_SYMBOL(tegra_virt_machine_get_num_dai_links);

struct snd_soc_dai_link *tegra_virt_machine_get_dai_link(void)
{
	struct snd_soc_dai_link *link = tegra_virt_t186ref_pcm_links;
	unsigned int size = TEGRA186_XBAR_DAI_LINKS;

	if (of_machine_is_compatible("nvidia,tegra210")) {
		link = tegra_virt_t210ref_pcm_links;
		size = TEGRA210_XBAR_DAI_LINKS;
	}

	tegra_virt_machine_set_num_dai_links(size);
	return link;
}
EXPORT_SYMBOL(tegra_virt_machine_get_dai_link);

void tegra_virt_machine_set_adsp_admaif_dai_params(
		uint32_t id, struct snd_soc_pcm_stream *params)
{
	struct snd_soc_dai_link *link = tegra_virt_t186ref_pcm_links;

	/* Check for valid ADSP ADMAIF ID */
	if (id >= MAX_ADMAIF_IDS) {
		pr_err("Invalid ADSP ADMAIF ID: %d\n", id);
		return;
	}

	/* Find DAI link corresponding to ADSP ADMAIF */
	link += id + MAX_ADMAIF_IDS;

	memcpy(&adsp_admaif_params[id], params,
		sizeof(struct snd_soc_pcm_stream));

	link->params = &adsp_admaif_params[id];
}
EXPORT_SYMBOL(tegra_virt_machine_set_adsp_admaif_dai_params);

MODULE_AUTHOR("Dipesh Gandhi <dipeshg@nvidia.com>");
MODULE_DESCRIPTION("Tegra Virt ASoC machine code");
MODULE_LICENSE("GPL");
