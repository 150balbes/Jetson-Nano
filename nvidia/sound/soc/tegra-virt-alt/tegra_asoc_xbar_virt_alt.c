/*
 * tegra_asoc_xbar_virt_alt.c - Tegra xbar dai link for machine drivers
 *
 * Copyright (c) 2017-2018 NVIDIA CORPORATION.  All rights reserved.
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

#include "tegra_virt_alt_ivc.h"
#include "tegra_asoc_util_virt_alt.h"
#include "tegra_asoc_xbar_virt_alt.h"

static const struct soc_enum *tegra_virt_enum_source;

#define DAI(sname)						\
	{							\
		.name = #sname " CIF",				\
		.playback = {					\
			.stream_name = #sname " CIF Receive",	\
			.channels_min = 1,			\
			.channels_max = 16,			\
			.rates = SNDRV_PCM_RATE_8000_192000,	\
			.formats = SNDRV_PCM_FMTBIT_S8 |	\
				SNDRV_PCM_FMTBIT_S16_LE |	\
				SNDRV_PCM_FMTBIT_S24_LE |	\
				SNDRV_PCM_FMTBIT_S32_LE,	\
		},						\
		.capture = {					\
			.stream_name = #sname " CIF Transmit",	\
			.channels_min = 1,			\
			.channels_max = 16,			\
			.rates = SNDRV_PCM_RATE_8000_192000,	\
			.formats = SNDRV_PCM_FMTBIT_S8 |	\
				SNDRV_PCM_FMTBIT_S16_LE |	\
				SNDRV_PCM_FMTBIT_S24_LE |	\
				SNDRV_PCM_FMTBIT_S32_LE,	\
		},						\
	}

static struct snd_soc_dai_driver tegra186_virt_xbar_dais[] = {
	DAI(ADMAIF1),
	DAI(ADMAIF2),
	DAI(ADMAIF3),
	DAI(ADMAIF4),
	DAI(ADMAIF5),
	DAI(ADMAIF6),
	DAI(ADMAIF7),
	DAI(ADMAIF8),
	DAI(ADMAIF9),
	DAI(ADMAIF10),
	DAI(ADMAIF11),
	DAI(ADMAIF12),
	DAI(ADMAIF13),
	DAI(ADMAIF14),
	DAI(ADMAIF15),
	DAI(ADMAIF16),
	DAI(ADMAIF17),
	DAI(ADMAIF18),
	DAI(ADMAIF19),
	DAI(ADMAIF20),
};

const int tegra_virt_t210ref_source_value[] = {
	/* Mux0 input,  Mux1 input, Mux2 input */
	0,
	MUX_VALUE(0, 0),
	MUX_VALUE(0, 1),
	MUX_VALUE(0, 2),
	MUX_VALUE(0, 3),
	MUX_VALUE(0, 4),
	MUX_VALUE(0, 5),
	MUX_VALUE(0, 6),
	MUX_VALUE(0, 7),
	MUX_VALUE(0, 8),
	MUX_VALUE(0, 9),
	MUX_VALUE(0, 16),
	MUX_VALUE(0, 17),
	MUX_VALUE(0, 18),
	MUX_VALUE(0, 19),
	MUX_VALUE(0, 20),
	MUX_VALUE(0, 24),
	MUX_VALUE(0, 25),
	MUX_VALUE(0, 26),
	MUX_VALUE(0, 27),
	MUX_VALUE(1, 0),
	MUX_VALUE(1, 1),
	MUX_VALUE(1, 2),
	MUX_VALUE(1, 3),
	MUX_VALUE(1, 4),
	MUX_VALUE(1, 8),
	MUX_VALUE(1, 9),
	MUX_VALUE(1, 20),
	MUX_VALUE(1, 21),
	MUX_VALUE(1, 24),
	MUX_VALUE(1, 25),
	MUX_VALUE(1, 26),
	MUX_VALUE(1, 27),
	MUX_VALUE(1, 28),
	MUX_VALUE(1, 29),
	MUX_VALUE(2, 0),
	MUX_VALUE(2, 1),
	MUX_VALUE(2, 4),
	MUX_VALUE(2, 8),
	MUX_VALUE(2, 9),
	MUX_VALUE(2, 12),
	MUX_VALUE(2, 13),
	MUX_VALUE(2, 14),
	MUX_VALUE(2, 15),
	MUX_VALUE(2, 18),
	MUX_VALUE(2, 19),
	MUX_VALUE(2, 20),
	MUX_VALUE(2, 24),
	MUX_VALUE(2, 25),
	MUX_VALUE(2, 26),
	MUX_VALUE(2, 27),
	MUX_VALUE(2, 28),
	MUX_VALUE(2, 29),
	MUX_VALUE(2, 30),
	MUX_VALUE(2, 31),
	/* index 35..53 above are inputs of PART2 Mux */
};

const char * const tegra_virt_t210ref_source_text[] = {
	"None",
	"ADMAIF1",
	"ADMAIF2",
	"ADMAIF3",
	"ADMAIF4",
	"ADMAIF5",
	"ADMAIF6",
	"ADMAIF7",
	"ADMAIF8",
	"ADMAIF9",
	"ADMAIF10",
	"I2S1",
	"I2S2",
	"I2S3",
	"I2S4",
	"I2S5",
	"SFC1",
	"SFC2",
	"SFC3",
	"SFC4",
	"MIXER1-1",
	"MIXER1-2",
	"MIXER1-3",
	"MIXER1-4",
	"MIXER1-5",
	"AMX1",
	"AMX2",
	"SPDIF1-1",
	"SPDIF1-2",
	"AFC1",
	"AFC2",
	"AFC3",
	"AFC4",
	"AFC5",
	"AFC6",
	"OPE1",
	"OPE2",
	"SPKPROT1",
	"MVC1",
	"MVC2",
	"IQC1-1",
	"IQC1-2",
	"IQC2-1",
	"IQC2-2",
	"DMIC1",
	"DMIC2",
	"DMIC3",
	"ADX1-1",
	"ADX1-2",
	"ADX1-3",
	"ADX1-4",
	"ADX2-1",
	"ADX2-2",
	"ADX2-3",
	"ADX2-4",
};

const int tegra_virt_t186ref_source_value[] = {
	0,
	MUX_VALUE(0, 0),
	MUX_VALUE(0, 1),
	MUX_VALUE(0, 2),
	MUX_VALUE(0, 3),
	MUX_VALUE(0, 4),
	MUX_VALUE(0, 5),
	MUX_VALUE(0, 6),
	MUX_VALUE(0, 7),
	MUX_VALUE(0, 8),
	MUX_VALUE(0, 9),
	MUX_VALUE(0, 10),
	MUX_VALUE(0, 11),
	MUX_VALUE(0, 12),
	MUX_VALUE(0, 13),
	MUX_VALUE(0, 14),
	MUX_VALUE(0, 15),
	MUX_VALUE(0, 16),
	MUX_VALUE(0, 17),
	MUX_VALUE(0, 18),
	MUX_VALUE(0, 19),
	MUX_VALUE(0, 20),
	MUX_VALUE(0, 21),
	MUX_VALUE(0, 24),
	MUX_VALUE(0, 25),
	MUX_VALUE(0, 26),
	MUX_VALUE(0, 27),
	MUX_VALUE(1, 0),
	MUX_VALUE(1, 1),
	MUX_VALUE(1, 2),
	MUX_VALUE(1, 3),
	MUX_VALUE(1, 4),
	MUX_VALUE(1, 8),
	MUX_VALUE(1, 9),
	MUX_VALUE(1, 10),
	MUX_VALUE(1, 11),
	MUX_VALUE(1, 16),
	MUX_VALUE(1, 20),
	MUX_VALUE(1, 21),
	MUX_VALUE(1, 24),
	MUX_VALUE(1, 25),
	MUX_VALUE(1, 26),
	MUX_VALUE(1, 27),
	MUX_VALUE(1, 28),
	MUX_VALUE(1, 29),
	MUX_VALUE(2, 0),
	MUX_VALUE(2, 4),
	MUX_VALUE(2, 8),
	MUX_VALUE(2, 9),
	MUX_VALUE(2, 12),
	MUX_VALUE(2, 13),
	MUX_VALUE(2, 14),
	MUX_VALUE(2, 15),
	MUX_VALUE(2, 18),
	MUX_VALUE(2, 19),
	MUX_VALUE(2, 20),
	MUX_VALUE(2, 21),
	MUX_VALUE(2, 24),
	MUX_VALUE(2, 25),
	MUX_VALUE(2, 26),
	MUX_VALUE(2, 27),
	MUX_VALUE(2, 28),
	MUX_VALUE(2, 29),
	MUX_VALUE(2, 30),
	MUX_VALUE(2, 31),
	MUX_VALUE(3, 0),
	MUX_VALUE(3, 1),
	MUX_VALUE(3, 2),
	MUX_VALUE(3, 3),
	MUX_VALUE(3, 4),
	MUX_VALUE(3, 5),
	MUX_VALUE(3, 6),
	MUX_VALUE(3, 7),
	MUX_VALUE(3, 16),
	MUX_VALUE(3, 17),
	MUX_VALUE(3, 18),
	MUX_VALUE(3, 19),
	MUX_VALUE(3, 24),
	MUX_VALUE(3, 25),
	MUX_VALUE(3, 26),
	MUX_VALUE(3, 27),
	MUX_VALUE(3, 28),
	MUX_VALUE(3, 29),
};

const char * const tegra_virt_t186ref_source_text[] = {
	"None",
	"ADMAIF1",
	"ADMAIF2",
	"ADMAIF3",
	"ADMAIF4",
	"ADMAIF5",
	"ADMAIF6",
	"ADMAIF7",
	"ADMAIF8",
	"ADMAIF9",
	"ADMAIF10",
	"ADMAIF11",
	"ADMAIF12",
	"ADMAIF13",
	"ADMAIF14",
	"ADMAIF15",
	"ADMAIF16",
	"I2S1",
	"I2S2",
	"I2S3",
	"I2S4",
	"I2S5",
	"I2S6",
	"SFC1",
	"SFC2",
	"SFC3",
	"SFC4",
	"MIXER1-1",
	"MIXER1-2",
	"MIXER1-3",
	"MIXER1-4",
	"MIXER1-5",
	"AMX1",
	"AMX2",
	"AMX3",
	"AMX4",
	"ARAD1",
	"SPDIF1-1",
	"SPDIF1-2",
	"AFC1",
	"AFC2",
	"AFC3",
	"AFC4",
	"AFC5",
	"AFC6",
	"OPE1",
	"SPKPROT1",
	"MVC1",
	"MVC2",
	"IQC1-1",
	"IQC1-2",
	"IQC2-1",
	"IQC2-2",
	"DMIC1",
	"DMIC2",
	"DMIC3",
	"DMIC4",
	"ADX1-1",
	"ADX1-2",
	"ADX1-3",
	"ADX1-4",
	"ADX2-1",
	"ADX2-2",
	"ADX2-3",
	"ADX2-4",
	"ADX3-1",
	"ADX3-2",
	"ADX3-3",
	"ADX3-4",
	"ADX4-1",
	"ADX4-2",
	"ADX4-3",
	"ADX4-4",
	"ADMAIF17",
	"ADMAIF18",
	"ADMAIF19",
	"ADMAIF20",
	"ASRC1-1",
	"ASRC1-2",
	"ASRC1-3",
	"ASRC1-4",
	"ASRC1-5",
	"ASRC1-6",
};

MUX_ENUM_CTRL_DECL_186(admaif1_tx, 0x00);
MUX_ENUM_CTRL_DECL_186(admaif2_tx, 0x01);
MUX_ENUM_CTRL_DECL_186(admaif3_tx, 0x02);
MUX_ENUM_CTRL_DECL_186(admaif4_tx, 0x03);
MUX_ENUM_CTRL_DECL_186(admaif5_tx, 0x04);
MUX_ENUM_CTRL_DECL_186(admaif6_tx, 0x05);
MUX_ENUM_CTRL_DECL_186(admaif7_tx, 0x06);
MUX_ENUM_CTRL_DECL_186(admaif8_tx, 0x07);
MUX_ENUM_CTRL_DECL_186(admaif9_tx, 0x08);
MUX_ENUM_CTRL_DECL_186(admaif10_tx, 0x09);
MUX_ENUM_CTRL_DECL_186(i2s1_tx, 0x10);
MUX_ENUM_CTRL_DECL_186(i2s2_tx, 0x11);
MUX_ENUM_CTRL_DECL_186(i2s3_tx, 0x12);
MUX_ENUM_CTRL_DECL_186(i2s4_tx, 0x13);
MUX_ENUM_CTRL_DECL_186(i2s5_tx, 0x14);
MUX_ENUM_CTRL_DECL_186(sfc1_tx, 0x18);
MUX_ENUM_CTRL_DECL_186(sfc2_tx, 0x19);
MUX_ENUM_CTRL_DECL_186(sfc3_tx, 0x1a);
MUX_ENUM_CTRL_DECL_186(sfc4_tx, 0x1b);
MUX_ENUM_CTRL_DECL_186(mixer11_tx, 0x20);
MUX_ENUM_CTRL_DECL_186(mixer12_tx, 0x21);
MUX_ENUM_CTRL_DECL_186(mixer13_tx, 0x22);
MUX_ENUM_CTRL_DECL_186(mixer14_tx, 0x23);
MUX_ENUM_CTRL_DECL_186(mixer15_tx, 0x24);
MUX_ENUM_CTRL_DECL_186(mixer16_tx, 0x25);
MUX_ENUM_CTRL_DECL_186(mixer17_tx, 0x26);
MUX_ENUM_CTRL_DECL_186(mixer18_tx, 0x27);
MUX_ENUM_CTRL_DECL_186(mixer19_tx, 0x28);
MUX_ENUM_CTRL_DECL_186(mixer110_tx, 0x29);
MUX_ENUM_CTRL_DECL_186(spdif11_tx, 0x34);
MUX_ENUM_CTRL_DECL_186(spdif12_tx, 0x35);
MUX_ENUM_CTRL_DECL_186(afc1_tx, 0x38);
MUX_ENUM_CTRL_DECL_186(afc2_tx, 0x39);
MUX_ENUM_CTRL_DECL_186(afc3_tx, 0x3a);
MUX_ENUM_CTRL_DECL_186(afc4_tx, 0x3b);
MUX_ENUM_CTRL_DECL_186(afc5_tx, 0x3c);
MUX_ENUM_CTRL_DECL_186(afc6_tx, 0x3d);
MUX_ENUM_CTRL_DECL_186(ope1_tx, 0x40);
MUX_ENUM_CTRL_DECL_186(spkprot_tx, 0x44);
MUX_ENUM_CTRL_DECL_186(mvc1_tx, 0x48);
MUX_ENUM_CTRL_DECL_186(mvc2_tx, 0x49);
MUX_ENUM_CTRL_DECL_186(amx11_tx, 0x50);
MUX_ENUM_CTRL_DECL_186(amx12_tx, 0x51);
MUX_ENUM_CTRL_DECL_186(amx13_tx, 0x52);
MUX_ENUM_CTRL_DECL_186(amx14_tx, 0x53);
MUX_ENUM_CTRL_DECL_186(amx21_tx, 0x54);
MUX_ENUM_CTRL_DECL_186(amx22_tx, 0x55);
MUX_ENUM_CTRL_DECL_186(amx23_tx, 0x56);
MUX_ENUM_CTRL_DECL_186(amx24_tx, 0x57);
MUX_ENUM_CTRL_DECL_186(adx1_tx, 0x60);
MUX_ENUM_CTRL_DECL_186(adx2_tx, 0x61);
MUX_ENUM_CTRL_DECL_186(dspk1_tx, 0x30);
MUX_ENUM_CTRL_DECL_186(dspk2_tx, 0x31);
MUX_ENUM_CTRL_DECL_186(amx31_tx, 0x58);
MUX_ENUM_CTRL_DECL_186(amx32_tx, 0x59);
MUX_ENUM_CTRL_DECL_186(amx33_tx, 0x5a);
MUX_ENUM_CTRL_DECL_186(amx34_tx, 0x5b);
MUX_ENUM_CTRL_DECL_186(amx41_tx, 0x64);
MUX_ENUM_CTRL_DECL_186(amx42_tx, 0x65);
MUX_ENUM_CTRL_DECL_186(amx43_tx, 0x66);
MUX_ENUM_CTRL_DECL_186(amx44_tx, 0x67);
MUX_ENUM_CTRL_DECL_186(admaif11_tx, 0x0a);
MUX_ENUM_CTRL_DECL_186(admaif12_tx, 0x0b);
MUX_ENUM_CTRL_DECL_186(admaif13_tx, 0x0c);
MUX_ENUM_CTRL_DECL_186(admaif14_tx, 0x0d);
MUX_ENUM_CTRL_DECL_186(admaif15_tx, 0x0e);
MUX_ENUM_CTRL_DECL_186(admaif16_tx, 0x0f);
MUX_ENUM_CTRL_DECL_186(i2s6_tx, 0x15);
MUX_ENUM_CTRL_DECL_186(adx3_tx, 0x62);
MUX_ENUM_CTRL_DECL_186(adx4_tx, 0x63);
MUX_ENUM_CTRL_DECL_186(admaif17_tx, 0x68);
MUX_ENUM_CTRL_DECL_186(admaif18_tx, 0x69);
MUX_ENUM_CTRL_DECL_186(admaif19_tx, 0x6a);
MUX_ENUM_CTRL_DECL_186(admaif20_tx, 0x6b);
MUX_ENUM_CTRL_DECL_186(asrc11_tx, 0x6c);
MUX_ENUM_CTRL_DECL_186(asrc12_tx, 0x6d);
MUX_ENUM_CTRL_DECL_186(asrc13_tx, 0x6e);
MUX_ENUM_CTRL_DECL_186(asrc14_tx, 0x6f);
MUX_ENUM_CTRL_DECL_186(asrc15_tx, 0x70);
MUX_ENUM_CTRL_DECL_186(asrc16_tx, 0x71);
MUX_ENUM_CTRL_DECL_186(asrc17_tx, 0x72);


ADDER_CTRL_DECL(Adder1, 0x0);
ADDER_CTRL_DECL(Adder2, 0x1);
ADDER_CTRL_DECL(Adder3, 0x2);
ADDER_CTRL_DECL(Adder4, 0x3);
ADDER_CTRL_DECL(Adder5, 0x4);


static struct snd_soc_dapm_widget tegra186_virt_xbar_widgets[] = {
	WIDGETS("ADMAIF1", admaif1_tx),
	WIDGETS("ADMAIF2", admaif2_tx),
	WIDGETS("ADMAIF3", admaif3_tx),
	WIDGETS("ADMAIF4", admaif4_tx),
	WIDGETS("ADMAIF5", admaif5_tx),
	WIDGETS("ADMAIF6", admaif6_tx),
	WIDGETS("ADMAIF7", admaif7_tx),
	WIDGETS("ADMAIF8", admaif8_tx),
	WIDGETS("ADMAIF9", admaif9_tx),
	WIDGETS("ADMAIF10", admaif10_tx),
	WIDGETS("I2S1", i2s1_tx),
	WIDGETS("I2S2", i2s2_tx),
	WIDGETS("I2S3", i2s3_tx),
	WIDGETS("I2S4", i2s4_tx),
	WIDGETS("I2S5", i2s5_tx),
	WIDGETS("SFC1", sfc1_tx),
	WIDGETS("SFC2", sfc2_tx),
	WIDGETS("SFC3", sfc3_tx),
	WIDGETS("SFC4", sfc4_tx),
	MIXER_IN_WIDGETS("MIXER1-1", mixer11_tx),
	MIXER_IN_WIDGETS("MIXER1-2", mixer12_tx),
	MIXER_IN_WIDGETS("MIXER1-3", mixer13_tx),
	MIXER_IN_WIDGETS("MIXER1-4", mixer14_tx),
	MIXER_IN_WIDGETS("MIXER1-5", mixer15_tx),
	MIXER_IN_WIDGETS("MIXER1-6", mixer16_tx),
	MIXER_IN_WIDGETS("MIXER1-7", mixer17_tx),
	MIXER_IN_WIDGETS("MIXER1-8", mixer18_tx),
	MIXER_IN_WIDGETS("MIXER1-9", mixer19_tx),
	MIXER_IN_WIDGETS("MIXER1-10", mixer110_tx),

	MIXER_OUT_WIDGETS("MIXER1-1"),
	MIXER_OUT_WIDGETS("MIXER1-2"),
	MIXER_OUT_WIDGETS("MIXER1-3"),
	MIXER_OUT_WIDGETS("MIXER1-4"),
	MIXER_OUT_WIDGETS("MIXER1-5"),
	SND_SOC_DAPM_MIXER("Adder1", SND_SOC_NOPM, 1, 0,
		Adder1, ARRAY_SIZE(Adder1)),
	SND_SOC_DAPM_MIXER("Adder2", SND_SOC_NOPM, 1, 0,
		Adder2, ARRAY_SIZE(Adder2)),
	SND_SOC_DAPM_MIXER("Adder3", SND_SOC_NOPM, 1, 0,
		Adder3, ARRAY_SIZE(Adder3)),
	SND_SOC_DAPM_MIXER("Adder4", SND_SOC_NOPM, 1, 0,
		Adder4, ARRAY_SIZE(Adder4)),
	SND_SOC_DAPM_MIXER("Adder5", SND_SOC_NOPM, 1, 0,
		Adder5, ARRAY_SIZE(Adder5)),
	WIDGETS("SPDIF1-1", spdif11_tx),
	WIDGETS("SPDIF1-2", spdif12_tx),
	WIDGETS("AFC1", afc1_tx),
	WIDGETS("AFC2", afc2_tx),
	WIDGETS("AFC3", afc3_tx),
	WIDGETS("AFC4", afc4_tx),
	WIDGETS("AFC5", afc5_tx),
	WIDGETS("AFC6", afc6_tx),
	WIDGETS("OPE1", ope1_tx),
	WIDGETS("SPKPROT1", spkprot_tx),
	WIDGETS("MVC1", mvc1_tx),
	WIDGETS("MVC2", mvc2_tx),
	WIDGETS("AMX1-1", amx11_tx),
	WIDGETS("AMX1-2", amx12_tx),
	WIDGETS("AMX1-3", amx13_tx),
	WIDGETS("AMX1-4", amx14_tx),
	WIDGETS("AMX2-1", amx21_tx),
	WIDGETS("AMX2-2", amx22_tx),
	WIDGETS("AMX2-3", amx23_tx),
	WIDGETS("AMX2-4", amx24_tx),
	WIDGETS("ADX1", adx1_tx),
	WIDGETS("ADX2", adx2_tx),
	TX_WIDGETS("IQC1-1"),
	TX_WIDGETS("IQC1-2"),
	TX_WIDGETS("IQC2-1"),
	TX_WIDGETS("IQC2-2"),
	TX_WIDGETS("DMIC1"),
	TX_WIDGETS("DMIC2"),
	TX_WIDGETS("DMIC3"),
	TX_WIDGETS("AMX1"),
	TX_WIDGETS("ADX1-1"),
	TX_WIDGETS("ADX1-2"),
	TX_WIDGETS("ADX1-3"),
	TX_WIDGETS("ADX1-4"),
	TX_WIDGETS("AMX2"),
	TX_WIDGETS("ADX2-1"),
	TX_WIDGETS("ADX2-2"),
	TX_WIDGETS("ADX2-3"),
	TX_WIDGETS("ADX2-4"),
	WIDGETS("ADMAIF11", admaif11_tx),
	WIDGETS("ADMAIF12", admaif12_tx),
	WIDGETS("ADMAIF13", admaif13_tx),
	WIDGETS("ADMAIF14", admaif14_tx),
	WIDGETS("ADMAIF15", admaif15_tx),
	WIDGETS("ADMAIF16", admaif16_tx),
	WIDGETS("ADMAIF17", admaif17_tx),
	WIDGETS("ADMAIF18", admaif18_tx),
	WIDGETS("ADMAIF19", admaif19_tx),
	WIDGETS("ADMAIF20", admaif20_tx),
	WIDGETS("I2S6", i2s6_tx),
	WIDGETS("AMX3-1", amx31_tx),
	WIDGETS("AMX3-2", amx32_tx),
	WIDGETS("AMX3-3", amx33_tx),
	WIDGETS("AMX3-4", amx34_tx),
	WIDGETS("AMX4-1", amx41_tx),
	WIDGETS("AMX4-2", amx42_tx),
	WIDGETS("AMX4-3", amx43_tx),
	WIDGETS("AMX4-4", amx44_tx),
	WIDGETS("ADX3", adx3_tx),
	WIDGETS("ADX4", adx4_tx),
	WIDGETS("ASRC1-1", asrc11_tx),
	WIDGETS("ASRC1-2", asrc12_tx),
	WIDGETS("ASRC1-3", asrc13_tx),
	WIDGETS("ASRC1-4", asrc14_tx),
	WIDGETS("ASRC1-5", asrc15_tx),
	WIDGETS("ASRC1-6", asrc16_tx),
	WIDGETS("ASRC1-7", asrc17_tx),
	TX_WIDGETS("AMX3"),
	TX_WIDGETS("ADX3-1"),
	TX_WIDGETS("ADX3-2"),
	TX_WIDGETS("ADX3-3"),
	TX_WIDGETS("ADX3-4"),
	TX_WIDGETS("AMX4"),
	TX_WIDGETS("ADX4-1"),
	TX_WIDGETS("ADX4-2"),
	TX_WIDGETS("ADX4-3"),
	TX_WIDGETS("ADX4-4"),
	TX_WIDGETS("DMIC4"),
	TX_WIDGETS("ARAD1"),
	CODEC_WIDGET("I2S1"),
	CODEC_WIDGET("I2S2"),
	CODEC_WIDGET("I2S3"),
	CODEC_WIDGET("I2S4"),
	CODEC_WIDGET("I2S5"),
	CODEC_WIDGET("I2S6"),
};


#define MUX_ROUTES(name)						\
	{ name " Mux",      "ADMAIF1",		"ADMAIF1 RX" },		\
	{ name " Mux",      "ADMAIF2",		"ADMAIF2 RX" },		\
	{ name " Mux",      "ADMAIF3",		"ADMAIF3 RX" },		\
	{ name " Mux",      "ADMAIF4",		"ADMAIF4 RX" },		\
	{ name " Mux",      "ADMAIF5",		"ADMAIF5 RX" },		\
	{ name " Mux",      "ADMAIF6",		"ADMAIF6 RX" },		\
	{ name " Mux",      "ADMAIF7",		"ADMAIF7 RX" },		\
	{ name " Mux",      "ADMAIF8",		"ADMAIF8 RX" },		\
	{ name " Mux",      "ADMAIF9",		"ADMAIF9 RX" },		\
	{ name " Mux",      "ADMAIF10",		"ADMAIF10 RX" },	\
	{ name " Mux",      "I2S1",		"I2S1 RX" },		\
	{ name " Mux",      "I2S2",		"I2S2 RX" },		\
	{ name " Mux",      "I2S3",		"I2S3 RX" },		\
	{ name " Mux",      "I2S4",		"I2S4 RX" },		\
	{ name " Mux",      "I2S5",		"I2S5 RX" },		\
	{ name " Mux",      "SFC1",		"SFC1 RX" },		\
	{ name " Mux",      "SFC2",		"SFC2 RX" },		\
	{ name " Mux",      "SFC3",		"SFC3 RX" },		\
	{ name " Mux",      "SFC4",		"SFC4 RX" },		\
	{ name " Mux",      "MIXER1-1",		"MIXER1-1 RX" },	\
	{ name " Mux",      "MIXER1-2",		"MIXER1-2 RX" },	\
	{ name " Mux",      "MIXER1-3",		"MIXER1-3 RX" },	\
	{ name " Mux",      "MIXER1-4",		"MIXER1-4 RX" },	\
	{ name " Mux",      "MIXER1-5",		"MIXER1-5 RX" },	\
	{ name " Mux",      "SPDIF1-1",		"SPDIF1-1 RX" },	\
	{ name " Mux",      "SPDIF1-2",		"SPDIF1-2 RX" },	\
	{ name " Mux",      "AFC1",		"AFC1 RX" },		\
	{ name " Mux",      "AFC2",		"AFC2 RX" },		\
	{ name " Mux",      "AFC3",		"AFC3 RX" },		\
	{ name " Mux",      "AFC4",		"AFC4 RX" },		\
	{ name " Mux",      "AFC5",		"AFC5 RX" },		\
	{ name " Mux",      "AFC6",		"AFC6 RX" },		\
	{ name " Mux",      "OPE1",		"OPE1 RX" },		\
	{ name " Mux",      "MVC1",		"MVC1 RX" },		\
	{ name " Mux",      "MVC2",		"MVC2 RX" },		\
	{ name " Mux",      "IQC1-1",		"IQC1-1 RX" },		\
	{ name " Mux",      "IQC1-2",		"IQC1-2 RX" },		\
	{ name " Mux",      "IQC2-1",		"IQC2-1 RX" },		\
	{ name " Mux",      "IQC2-2",		"IQC2-2 RX" },		\
	{ name " Mux",      "DMIC1",		"DMIC1 RX" },		\
	{ name " Mux",      "DMIC2",		"DMIC2 RX" },		\
	{ name " Mux",      "DMIC3",		"DMIC3 RX" },		\
	{ name " Mux",      "AMX1",		"AMX1 RX" },		\
	{ name " Mux",      "ADX1-1",		"ADX1-1 RX" },		\
	{ name " Mux",      "ADX1-2",		"ADX1-2 RX" },		\
	{ name " Mux",      "ADX1-3",		"ADX1-3 RX" },		\
	{ name " Mux",      "ADX1-4",		"ADX1-4 RX" },		\
	{ name " Mux",      "AMX2",		"AMX2 RX" },		\
	{ name " Mux",      "ADX2-1",		"ADX2-1 RX" },		\
	{ name " Mux",      "ADX2-2",		"ADX2-2 RX" },		\
	{ name " Mux",      "ADX2-3",		"ADX2-3 RX" },		\
	{ name " Mux",      "ADX2-4",		"ADX2-4 RX" },		\
	{ name " Mux",      "ADMAIF11",		"ADMAIF11 RX" },	\
	{ name " Mux",      "ADMAIF12",		"ADMAIF12 RX" },	\
	{ name " Mux",      "ADMAIF13",		"ADMAIF13 RX" },	\
	{ name " Mux",      "ADMAIF14",		"ADMAIF14 RX" },	\
	{ name " Mux",      "ADMAIF15",		"ADMAIF15 RX" },	\
	{ name " Mux",      "ADMAIF16",		"ADMAIF16 RX" },	\
	{ name " Mux",      "ADMAIF17",		"ADMAIF17 RX" },	\
	{ name " Mux",      "ADMAIF18",		"ADMAIF18 RX" },	\
	{ name " Mux",      "ADMAIF19",		"ADMAIF19 RX" },	\
	{ name " Mux",      "ADMAIF20",		"ADMAIF20 RX" },	\
	{ name " Mux",      "DMIC4",		"DMIC4 RX" },		\
	{ name " Mux",      "I2S6",		"I2S6 RX" },		\
	{ name " Mux",      "ASRC1-1",		"ASRC1-1 RX" },		\
	{ name " Mux",      "ASRC1-2",		"ASRC1-2 RX" },		\
	{ name " Mux",      "ASRC1-3",		"ASRC1-3 RX" },		\
	{ name " Mux",      "ASRC1-4",		"ASRC1-4 RX" },		\
	{ name " Mux",      "ASRC1-5",		"ASRC1-5 RX" },		\
	{ name " Mux",      "ASRC1-6",		"ASRC1-6 RX" },		\
	{ name " Mux",      "AMX3",		"AMX3 RX" },		\
	{ name " Mux",      "ADX3-1",		"ADX3-1 RX" },		\
	{ name " Mux",      "ADX3-2",		"ADX3-2 RX" },		\
	{ name " Mux",      "ADX3-3",		"ADX3-3 RX" },		\
	{ name " Mux",      "ADX3-4",		"ADX3-4 RX" },		\
	{ name " Mux",      "AMX4",		"AMX4 RX" },		\
	{ name " Mux",      "ADX4-1",		"ADX4-1 RX" },		\
	{ name " Mux",      "ADX4-2",		"ADX4-2 RX" },		\
	{ name " Mux",      "ADX4-3",		"ADX4-3 RX" },		\
	{ name " Mux",      "ADX4-4",		"ADX4-4 RX" },		\
	{ name " Mux",      "ARAD1",		"ARAD1 RX" },

#define AMX_OUT_ROUTES(name)						\
	{ name " RX",      NULL,		name "-1 Mux" },	\
	{ name " RX",      NULL,		name "-2 Mux" },	\
	{ name " RX",      NULL,		name "-3 Mux" },	\
	{ name " RX",      NULL,		name "-4 Mux" },

#define ADX_IN_ROUTES(name)						\
	{ name "-1 RX",      NULL,		name " Mux" },		\
	{ name "-2 RX",      NULL,		name " Mux" },		\
	{ name "-3 RX",      NULL,		name " Mux" },		\
	{ name "-4 RX",      NULL,		name " Mux" },		\
	TEGRA210_ROUTES(name)

#define IN_OUT_ROUTES(name)						\
	{ name " RX",       NULL,		name " CIF Receive"},	\
	{ name " CIF Transmit", NULL,		name " Mux"},		\
	MUX_ROUTES(name)

#define TEGRA210_ROUTES(name)						\
	{ name " RX", NULL,		name " Mux"},			\
	MUX_ROUTES(name)

#define MIXER_IN_ROUTES(name)						\
	MUX_ROUTES(name)

#define MIC_SPK_ROUTES(name)						\
	{ name " RX",       NULL,		name " MIC"},		\
	{ name " HEADPHONE", NULL,		name " Mux"},		\
	MUX_ROUTES(name)

#define MIXER_ROUTES(name, id)	\
	{name,	"RX1",	"MIXER1-1 Mux",},	\
	{name,	"RX2",	"MIXER1-2 Mux",},	\
	{name,	"RX3",	"MIXER1-3 Mux",},	\
	{name,	"RX4",	"MIXER1-4 Mux",},	\
	{name,	"RX5",	"MIXER1-5 Mux",},	\
	{name,	"RX6",	"MIXER1-6 Mux",},	\
	{name,	"RX7",	"MIXER1-7 Mux",},	\
	{name,	"RX8",	"MIXER1-8 Mux",},	\
	{name,	"RX9",	"MIXER1-9 Mux",},	\
	{name,	"RX10",	"MIXER1-10 Mux"},	\
	{"MIXER1-"#id " RX",	NULL,	name}



static struct snd_soc_dapm_route tegra186_virt_xbar_routes[] = {
	IN_OUT_ROUTES("ADMAIF1")
	IN_OUT_ROUTES("ADMAIF2")
	IN_OUT_ROUTES("ADMAIF3")
	IN_OUT_ROUTES("ADMAIF4")
	IN_OUT_ROUTES("ADMAIF5")
	IN_OUT_ROUTES("ADMAIF6")
	IN_OUT_ROUTES("ADMAIF7")
	IN_OUT_ROUTES("ADMAIF8")
	IN_OUT_ROUTES("ADMAIF9")
	IN_OUT_ROUTES("ADMAIF10")
	MIC_SPK_ROUTES("I2S1")
	MIC_SPK_ROUTES("I2S2")
	MIC_SPK_ROUTES("I2S3")
	MIC_SPK_ROUTES("I2S4")
	MIC_SPK_ROUTES("I2S5")
	TEGRA210_ROUTES("SFC1")
	TEGRA210_ROUTES("SFC2")
	TEGRA210_ROUTES("SFC3")
	TEGRA210_ROUTES("SFC4")
	MIXER_IN_ROUTES("MIXER1-1")
	MIXER_IN_ROUTES("MIXER1-2")
	MIXER_IN_ROUTES("MIXER1-3")
	MIXER_IN_ROUTES("MIXER1-4")
	MIXER_IN_ROUTES("MIXER1-5")
	MIXER_IN_ROUTES("MIXER1-6")
	MIXER_IN_ROUTES("MIXER1-7")
	MIXER_IN_ROUTES("MIXER1-8")
	MIXER_IN_ROUTES("MIXER1-9")
	MIXER_IN_ROUTES("MIXER1-10")

	MIXER_ROUTES("Adder1", 1),
	MIXER_ROUTES("Adder2", 2),
	MIXER_ROUTES("Adder3", 3),
	MIXER_ROUTES("Adder4", 4),
	MIXER_ROUTES("Adder5", 5),

	TEGRA210_ROUTES("SPDIF1-1")
	TEGRA210_ROUTES("SPDIF1-2")
	TEGRA210_ROUTES("AFC1")
	TEGRA210_ROUTES("AFC2")
	TEGRA210_ROUTES("AFC3")
	TEGRA210_ROUTES("AFC4")
	TEGRA210_ROUTES("AFC5")
	TEGRA210_ROUTES("AFC6")
	TEGRA210_ROUTES("OPE1")
	TEGRA210_ROUTES("SPKPROT1")
	TEGRA210_ROUTES("MVC1")
	TEGRA210_ROUTES("MVC2")
	TEGRA210_ROUTES("AMX1-1")
	TEGRA210_ROUTES("AMX1-2")
	TEGRA210_ROUTES("AMX1-3")
	TEGRA210_ROUTES("AMX1-4")
	TEGRA210_ROUTES("AMX2-1")
	TEGRA210_ROUTES("AMX2-2")
	TEGRA210_ROUTES("AMX2-3")
	TEGRA210_ROUTES("AMX2-4")
	ADX_IN_ROUTES("ADX1")
	ADX_IN_ROUTES("ADX2")
	AMX_OUT_ROUTES("AMX1")
	AMX_OUT_ROUTES("AMX2")
	IN_OUT_ROUTES("ADMAIF11")
	IN_OUT_ROUTES("ADMAIF12")
	IN_OUT_ROUTES("ADMAIF13")
	IN_OUT_ROUTES("ADMAIF14")
	IN_OUT_ROUTES("ADMAIF15")
	IN_OUT_ROUTES("ADMAIF16")
	IN_OUT_ROUTES("ADMAIF17")
	IN_OUT_ROUTES("ADMAIF18")
	IN_OUT_ROUTES("ADMAIF19")
	IN_OUT_ROUTES("ADMAIF20")
	TEGRA210_ROUTES("AMX3-1")
	TEGRA210_ROUTES("AMX3-2")
	TEGRA210_ROUTES("AMX3-3")
	TEGRA210_ROUTES("AMX3-4")
	TEGRA210_ROUTES("AMX4-1")
	TEGRA210_ROUTES("AMX4-2")
	TEGRA210_ROUTES("AMX4-3")
	TEGRA210_ROUTES("AMX4-4")
	ADX_IN_ROUTES("ADX3")
	ADX_IN_ROUTES("ADX4")
	MIC_SPK_ROUTES("I2S6")
	TEGRA210_ROUTES("ASRC1-1")
	TEGRA210_ROUTES("ASRC1-2")
	TEGRA210_ROUTES("ASRC1-3")
	TEGRA210_ROUTES("ASRC1-4")
	TEGRA210_ROUTES("ASRC1-5")
	TEGRA210_ROUTES("ASRC1-6")
	TEGRA210_ROUTES("ASRC1-7")
	AMX_OUT_ROUTES("AMX3")
	AMX_OUT_ROUTES("AMX4")
};

static int tegra_virt_xbar_read(struct snd_soc_component *component,
		unsigned int reg, unsigned int *val)
{
	*val = 0;

	return 0;
}

static int tegra_virt_xbar_write(struct snd_soc_component *component,
		unsigned int reg, unsigned int val)
{
	return 0;
}

static int tegra_virt_xbar_component_probe(struct snd_soc_component *component)
{
	component->read = tegra_virt_xbar_read;
	component->write = tegra_virt_xbar_write;

	return 0;
}

static struct snd_soc_codec_driver tegra186_virt_xbar_codec = {
	.idle_bias_off = 1,
	.component_driver = {
		.probe = tegra_virt_xbar_component_probe,
		.dapm_widgets = tegra186_virt_xbar_widgets,
		.num_dapm_widgets = ARRAY_SIZE(tegra186_virt_xbar_widgets),
		.dapm_routes = tegra186_virt_xbar_routes,
		.num_dapm_routes = ARRAY_SIZE(tegra186_virt_xbar_routes),
	},
};

int tegra_virt_get_route(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	uint64_t reg = (uint64_t)e->reg;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err, i = 0;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_XBAR_GET_ROUTE;
	msg.params.xbar_info.rx_reg = (int) reg;

	err = nvaudio_ivc_send_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("%s: error on ivc_send_receive\n", __func__);

	for (i = 0; i < e->items; i++) {
		if (msg.params.xbar_info.bit_pos ==
			e->values[i])
			break;
		}

	if (i == e->items)
		ucontrol->value.integer.value[0] = 0;

	ucontrol->value.integer.value[0] = i;

	if (err < 0)
		return err;

	return 0;
}
EXPORT_SYMBOL(tegra_virt_get_route);


int tegra_virt_put_route(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	uint64_t reg = (uint64_t)e->reg;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;
	struct snd_soc_dapm_context *dapm =
				snd_soc_dapm_kcontrol_dapm(kcontrol);

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_XBAR_SET_ROUTE;
	msg.params.xbar_info.rx_reg = (int) reg;
	msg.params.xbar_info.tx_value =
	e->values[ucontrol->value.integer.value[0]];

	msg.params.xbar_info.tx_idx =
		ucontrol->value.integer.value[0] - 1;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	snd_soc_dapm_mux_update_power(dapm, kcontrol,
				ucontrol->value.integer.value[0], e, NULL);

	return 0;
}
EXPORT_SYMBOL(tegra_virt_put_route);

void tegra_virt_set_enum_source(const struct soc_enum *enum_virt)
{
	tegra_virt_enum_source = enum_virt;
}
EXPORT_SYMBOL(tegra_virt_set_enum_source);

static inline const struct soc_enum *tegra_virt_get_enum_source(void)
{
	return tegra_virt_enum_source;
}

int tegra_virt_xbar_register_codec(struct platform_device *pdev)
{

	int ret;

	ret = snd_soc_register_codec(&pdev->dev,
			&tegra186_virt_xbar_codec,
			tegra186_virt_xbar_dais,
			ARRAY_SIZE(tegra186_virt_xbar_dais));

	if (ret != 0) {
		dev_err(&pdev->dev, "Could not register CODEC: %d\n", ret);
		return -EBUSY;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_xbar_register_codec);
MODULE_AUTHOR("Dipesh Gandhi <dipeshg@nvidia.com>");
MODULE_DESCRIPTION("Tegra Virt ASoC XBAR code");
MODULE_LICENSE("GPL");
