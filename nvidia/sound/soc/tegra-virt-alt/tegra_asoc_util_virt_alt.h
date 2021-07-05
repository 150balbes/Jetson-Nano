/*
 * tegra_asoc_util_virt_alt.h - Tegra xbar dai link for machine drivers
 *
 * Copyright (c) 2017-2019, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __LINUX_VIRT_UTIL_H
#define __LINUX_VIRT_UTIL_H

#include <sound/soc.h>
#include "tegra_asoc_metadata_util_alt.h"

#define MIXER_CONFIG_SHIFT_VALUE 16
#define STREAM_ID_SHIFT_VALUE    16
#define REGDUMP_CMD_SHIFT_VALUE  24
#define MIXER_MAX_RX_GAIN        0x7FFFFFFF
#define TEGRA186_ASRC_STREAM_RATIO_INTEGER_PART_MASK		0x1F
#define TEGRA186_ASRC_STREAM_RATIO_FRAC_PART_MASK		0xFFFFFFFF
#define TEGRA186_ASRC_STREAM_RATIO_MASK				0x1FFFFFFFFF
#define NUM_ARAD_SOURCES	11
#define NUM_ARAD_LANES		6
#define NUM_ASRC_MODE		2
#define NUM_MVC_CURVETYPE	2
#define MAX_MVC_TAR_VOL         16000

#define MIXER_GAIN_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, MIXER_MAX_RX_GAIN, 0,	\
	tegra_virt_t210mixer_get_gain,	\
	tegra_virt_t210mixer_set_gain)

#define MIXER_GAIN_INSTANT_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, MIXER_MAX_RX_GAIN, 0,	\
	tegra_virt_t210mixer_get_gain,	\
	tegra_virt_t210mixer_set_gain_instant)

#define MIXER_DURATION_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	64, 0x7FFFFFFF, 0,	\
	tegra_virt_t210mixer_get_duration,	\
	tegra_virt_t210mixer_set_duration)

#define REG_PACK(id1, id2) ((id1 << MIXER_CONFIG_SHIFT_VALUE) | id2)
#define MIXER_ADDER_CTRL_DECL(ename, reg1, reg2) \
	SOC_SINGLE_EXT(ename, REG_PACK(reg1, reg2),  \
	0, 1, 0,	\
	tegra_virt_t210mixer_get_adder_config,	\
	tegra_virt_t210mixer_set_adder_config)

#define MIXER_ENABLE_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, 1, 0,	\
	tegra_virt_t210mixer_get_enable,	\
	tegra_virt_t210mixer_set_enable)

#define SFC_IN_FREQ_CTRL_DECL(ename, id) \
	SOC_SINGLE_EXT(ename, id,	\
	0, 192000, 0,	\
	tegra_virt_t210sfc_get_in_freq,	\
	tegra_virt_t210sfc_set_in_freq)

#define SFC_OUT_FREQ_CTRL_DECL(ename, id) \
	SOC_SINGLE_EXT(ename, id,	\
	0, 192000, 0,	\
	tegra_virt_t210sfc_get_out_freq,	\
	tegra_virt_t210sfc_set_out_freq)

#define MVC_CURVE_TYPE_CTRL_DECL(ename, reg, src) \
	SOC_ENUM_EXT_REG(ename, reg,	\
	src,	\
	tegra_virt_t210mvc_get_curve_type,	\
	tegra_virt_t210mvc_set_curve_type)

#define MVC_TAR_VOL_CTRL_DECL(ename, id) \
	SOC_SINGLE_EXT(ename, id,	\
	0, MAX_MVC_TAR_VOL, 0,	\
	tegra_virt_t210mvc_get_tar_vol,	\
	tegra_virt_t210mvc_set_tar_vol)

#define MVC_MUTE_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, 1, 0,	\
	tegra_virt_t210mvc_get_mute,	\
	tegra_virt_t210mvc_set_mute)

#define SOC_SINGLE_EXT_FRAC(xname, xregbase, xmax, xget, xput) \
{       .iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.info = snd_soc_info_xr_sx, .get = xget, \
	.put  = xput, \
	.private_value = (unsigned long)&(struct soc_mreg_control) \
		{.regbase = xregbase, .regcount = 1, .nbits = 32, \
		.invert = 0, .min = 0, .max = xmax} }

#define ASRC_RATIO_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT_FRAC(ename, reg,	\
	TEGRA186_ASRC_STREAM_RATIO_MASK,	\
	tegra186_virt_asrc_get_ratio,	\
	tegra186_virt_asrc_set_ratio)

#define ASRC_STREAM_RATIO_CTRL_DECL(ename, reg, src) \
	SOC_ENUM_EXT_REG(ename, reg,	\
	src,	\
	tegra186_virt_asrc_get_ratio_source,	\
	tegra186_virt_asrc_set_ratio_source)

#define ASRC_STREAM_ENABLE_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, 1, 0,	\
	tegra186_virt_asrc_get_stream_enable,	\
	tegra186_virt_asrc_set_stream_enable)

#define ASRC_STREAM_HWCOMP_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, 1, 0,	\
	tegra186_virt_asrc_get_hwcomp_disable,	\
	tegra186_virt_asrc_set_hwcomp_disable)

#define ASRC_STREAM_INPUT_THRESHOLD_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, 3, 0,	\
	tegra186_virt_asrc_get_input_threshold,	\
	tegra186_virt_asrc_set_input_threshold)

#define ASRC_STREAM_OUTPUT_THRESHOLD_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, 3, 0,	\
	tegra186_virt_asrc_get_output_threshold,	\
	tegra186_virt_asrc_set_output_threshold)

#define AMX_ENABLE_CTRL_DECL(ename, reg1, reg2) \
	SOC_SINGLE_EXT(ename, REG_PACK(reg1, reg2),  \
	0, 1, 0,	\
	tegra_virt_t210_amx_get_input_stream_enable,	\
	tegra_virt_t210_amx_set_input_stream_enable)

#define ARAD_LANE_SOURCE_CTRL_DECL(ename, reg, src) \
	SOC_ENUM_EXT_REG(ename, reg,	\
	src,	\
	tegra186_virt_arad_get_lane_source,	\
	tegra186_virt_arad_set_lane_source)

#define ARAD_LANE_PRESCALAR_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, 65535, 0,	\
	tegra186_virt_arad_get_lane_prescalar,	\
	tegra186_virt_arad_set_lane_prescalar)

#define ARAD_LANE_ENABLE_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, 1, 0,	\
	tegra186_virt_arad_get_lane_enable,	\
	tegra186_virt_arad_set_lane_enable)

#define ARAD_LANE_RATIO_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, 0xFFFFFFFF, 0,	\
	tegra186_virt_arad_get_lane_ratio, NULL)

#define I2S_LOOPBACK_ENABLE_CTRL_DECL(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, 1, 0,	\
	tegra_virt_i2s_get_loopback_enable,	\
	tegra_virt_i2s_set_loopback_enable)

#define I2S_SET_RATE(ename, reg) \
	SOC_SINGLE_EXT(ename, reg,	\
	0, 96000, 0,	\
	tegra_virt_i2s_get_rate,	\
	tegra_virt_i2s_set_rate)


#define MIXER_SET_FADE(xname, xbase) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	\
	.info =  tegra_virt_t210mixer_param_info,	\
	.name = xname,	\
	.put = tegra_virt_t210mixer_set_fade,	\
	.get = tegra_virt_t210mixer_get_fade,	\
	.private_value =	\
	((unsigned long)&(struct soc_bytes)	\
	{.base = xbase, .num_regs = 128,	\
	.mask = SNDRV_CTL_ELEM_TYPE_INTEGER}) }

#define MIXER_GET_FADE_STATUS(xname, xbase) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER,	\
	.info =  tegra_virt_t210mixer_param_info,	\
	.access = SNDRV_CTL_ELEM_ACCESS_READ,	\
	.name = xname,	\
	.get = tegra_virt_t210mixer_get_fade_status,	\
	.private_value =	\
	((unsigned long)&(struct soc_bytes)	\
	{.base = xbase, .num_regs = 128,	\
	.mask = SNDRV_CTL_ELEM_TYPE_INTEGER}) }

#define REGDUMP_PACK(id1, id2, id3) \
	(id1 | (id2 << STREAM_ID_SHIFT_VALUE) | (id3 << REGDUMP_CMD_SHIFT_VALUE))
#define REGDUMP_CTRL_DECL(ename, id, stream_id, cmd) \
	SOC_SINGLE_EXT(ename, REGDUMP_PACK(id, stream_id, cmd),  \
	0, 1, 0,	\
	tegra_virt_t210ahub_get_regdump, \
	tegra_virt_t210ahub_set_regdump)

#define ADMA_REGDUMP_CTRL_DECL(ename, channel_id) \
	SOC_SINGLE_EXT(ename, channel_id,  \
	0, 1, 0,	\
	tegra_virt_t210adma_get_regdump, \
	tegra_virt_t210adma_set_regdump)

#define METADATA_CTRL_DECL(ename) \
	{.iface = SNDRV_CTL_ELEM_IFACE_MIXER, \
	.name = ename, .info = tegra_bytes_info, \
	.get = tegra_virt_get_metadata, \
	.put = tegra_virt_set_metadata, \
	.private_value = ((unsigned long)&(struct soc_bytes) \
		{.base = 1, .mask = SNDRV_CTL_ELEM_TYPE_BYTES, \
		.num_regs = (sizeof(uint16_t) * \
		(TEGRA_AUDIO_METADATA_HDR_LENGTH + 1)), \
		 })}

#define ADDER_CTRL_DECL(name, id)	\
	static const struct snd_kcontrol_new name[] = {	\
MIXER_ADDER_CTRL_DECL("RX1", id, 0x01),	\
MIXER_ADDER_CTRL_DECL("RX2", id, 0x02),	\
MIXER_ADDER_CTRL_DECL("RX3", id, 0x03),	\
MIXER_ADDER_CTRL_DECL("RX4", id, 0x04),	\
MIXER_ADDER_CTRL_DECL("RX5", id, 0x05),	\
MIXER_ADDER_CTRL_DECL("RX6", id, 0x06),	\
MIXER_ADDER_CTRL_DECL("RX7", id, 0x07),	\
MIXER_ADDER_CTRL_DECL("RX8", id, 0x08),	\
MIXER_ADDER_CTRL_DECL("RX9", id, 0x09),	\
MIXER_ADDER_CTRL_DECL("RX10", id, 0x0a),	\
}

enum {
	numerator1_enum = 0,
	numerator2_enum,
	numerator3_enum,
	numerator4_enum,
	numerator5_enum,
	numerator6_enum,
	denominator1_enum = NUM_ARAD_LANES,
	denominator2_enum,
	denominator3_enum,
	denominator4_enum,
	denominator5_enum,
	denominator6_enum,
};

extern const int tegra186_arad_mux_value[];
extern const char * const tegra186_arad_mux_text[];
extern const char * const tegra186_asrc_ratio_source_text[];
extern const char * const tegra210_mvc_curve_type_text[];

int tegra_virt_t210mixer_get_gain(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_t210mixer_set_gain(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_t210mixer_set_gain_instant(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210mixer_get_duration(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_t210mixer_set_duration(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210mixer_get_adder_config(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210mixer_set_adder_config(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210mixer_get_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210mixer_set_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210sfc_get_in_freq(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210sfc_set_in_freq(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210sfc_get_out_freq(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_t210sfc_set_out_freq(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210mvc_get_curve_type(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_t210mvc_set_curve_type(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210mvc_get_tar_vol(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_t210mvc_set_tar_vol(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210mvc_get_mute(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_t210mvc_set_mute(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_asrc_get_ratio(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_asrc_set_ratio(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_asrc_get_ratio_source(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra186_virt_asrc_set_ratio_source(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra186_virt_asrc_get_stream_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_asrc_set_stream_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_asrc_get_hwcomp_disable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_asrc_set_hwcomp_disable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra186_virt_asrc_get_input_threshold(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_asrc_set_input_threshold(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_asrc_get_output_threshold(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra186_virt_asrc_set_output_threshold(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210_amx_get_input_stream_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_t210_amx_set_input_stream_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);


int tegra186_virt_arad_get_lane_source(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra186_virt_arad_set_lane_source(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_arad_get_lane_prescalar(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_arad_set_lane_prescalar(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_arad_get_lane_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_arad_set_lane_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra186_virt_arad_get_lane_ratio(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_i2s_set_loopback_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_i2s_get_loopback_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_i2s_set_rate(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_i2s_get_rate(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
//Mixer fade
int tegra_virt_t210mixer_get_fade_status(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_t210mixer_set_fade(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_t210mixer_get_fade(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_t210mixer_param_info(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_info *uinfo);
int tegra_metadata_get_init(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_metadata_set_init(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_metadata_get_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_metadata_set_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_metadata_setup(struct platform_device *pdev,
	struct tegra_audio_metadata_cntx *psad,	struct snd_soc_card *card);
int tegra_virt_set_metadata(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_get_metadata(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210ahub_get_regdump(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_t210ahub_set_regdump(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

int tegra_virt_t210adma_set_regdump(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int tegra_virt_t210adma_get_regdump(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

#endif
