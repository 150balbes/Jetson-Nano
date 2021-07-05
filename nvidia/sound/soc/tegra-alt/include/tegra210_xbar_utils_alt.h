/*
 * tegra210_xbar_utils_alt.h - TEGRA XBAR Utils
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

#ifndef __TEGRA210_XBAR_UTILS_ALT_H__
#define __TEGRA210_XBAR_UTILS_ALT_H__


#define DAI(sname)						\
	{							\
		.name = #sname,					\
		.playback = {					\
			.stream_name = #sname " Receive",	\
			.channels_min = 1,			\
			.channels_max = 16,			\
			.rates = SNDRV_PCM_RATE_8000_192000,		\
			.formats = SNDRV_PCM_FMTBIT_S8 |		\
				SNDRV_PCM_FMTBIT_S16_LE |		\
				SNDRV_PCM_FMTBIT_S24_LE |		\
				SNDRV_PCM_FMTBIT_S32_LE,		\
		},						\
		.capture = {					\
			.stream_name = #sname " Transmit",	\
			.channels_min = 1,			\
			.channels_max = 16,			\
			.rates = SNDRV_PCM_RATE_8000_192000,		\
			.formats = SNDRV_PCM_FMTBIT_S8 |		\
				SNDRV_PCM_FMTBIT_S16_LE |		\
				SNDRV_PCM_FMTBIT_S24_LE |		\
				SNDRV_PCM_FMTBIT_S32_LE,		\
		},						\
	}

#define MUX_REG(id) (TEGRA210_XBAR_RX_STRIDE * (id))

#define SOC_VALUE_ENUM_WIDE(xreg, shift, xmax, xtexts, xvalues) \
{	.reg = xreg, .shift_l = shift, .shift_r = shift, \
	.items = xmax, .texts = xtexts, .values = xvalues, \
	.mask = xmax ? roundup_pow_of_two(xmax) - 1 : 0}

#define SOC_VALUE_ENUM_WIDE_DECL(name, xreg, shift, \
		xtexts, xvalues) \
	static struct soc_enum name = SOC_VALUE_ENUM_WIDE(xreg, shift, \
					ARRAY_SIZE(xtexts), xtexts, xvalues)

#define MUX_ENUM_CTRL_DECL(ename, id) \
	SOC_VALUE_ENUM_WIDE_DECL(ename##_enum, MUX_REG(id), 0,	\
			tegra210_xbar_mux_texts, tegra210_xbar_mux_values); \
	static const struct snd_kcontrol_new ename##_control = \
		SOC_DAPM_ENUM_EXT("Route", ename##_enum,\
				tegra_xbar_get_value_enum,\
				tegra_xbar_put_value_enum)

#define MUX_ENUM_CTRL_DECL_186(ename, id) \
	SOC_VALUE_ENUM_WIDE_DECL(ename##_enum, MUX_REG(id), 0,	\
			tegra186_xbar_mux_texts, tegra186_xbar_mux_values); \
	static const struct snd_kcontrol_new ename##_control = \
		SOC_DAPM_ENUM_EXT("Route", ename##_enum,\
				tegra_xbar_get_value_enum,\
				tegra_xbar_put_value_enum)

#define WIDGETS(sname, ename) \
	SND_SOC_DAPM_AIF_IN(sname " RX", NULL, 0, SND_SOC_NOPM, 0, 0), \
	SND_SOC_DAPM_AIF_OUT(sname " TX", NULL, 0, SND_SOC_NOPM, 0, 0), \
	SND_SOC_DAPM_MUX(sname " Mux", SND_SOC_NOPM, 0, 0, &ename##_control)

#define TX_WIDGETS(sname) \
	SND_SOC_DAPM_AIF_IN(sname " RX", NULL, 0, SND_SOC_NOPM, 0, 0), \
	SND_SOC_DAPM_AIF_OUT(sname " TX", NULL, 0, SND_SOC_NOPM, 0, 0)

#define MUX_VALUE(npart, nbit) (1 + nbit + npart * 32)

#define IN_OUT_ROUTES(name)				\
	{ name " RX",       NULL,	name " Receive" },	\
	{ name " Transmit", NULL,	name " TX" },

int tegra210_xbar_read_reg (unsigned int reg, unsigned int *val);
int tegra_xbar_get_value_enum(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol);
int tegra_xbar_put_value_enum(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol);
bool tegra_xbar_volatile_reg(struct device *dev, unsigned int reg);
int tegra_xbar_remove(struct platform_device *pdev);
int tegra_xbar_runtime_resume(struct device *dev);
int tegra_xbar_runtime_suspend(struct device *dev);
int tegra_xbar_resume(struct device *dev);
int tegra_xbar_suspend(struct device *dev);
int tegra_xbar_probe(struct platform_device *pdev,
					struct tegra_xbar_soc_data *soc_data);
#endif
