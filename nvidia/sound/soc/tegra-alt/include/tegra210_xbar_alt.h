/*
 * tegra210_xbar_alt.h - TEGRA210 XBAR registers
 *
 * Copyright (c) 2014-2019, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __TEGRA210_XBAR_ALT_H__
#define __TEGRA210_XBAR_ALT_H__

#define TEGRA210_XBAR_PART0_RX					0x0
#define TEGRA210_XBAR_PART1_RX					0x200
#define TEGRA210_XBAR_PART2_RX					0x400
#define TEGRA210_XBAR_RX_STRIDE					0x4
#define TEGRA210_XBAR_AUDIO_RX_COUNT				90

/* This register repeats twice for each XBAR TX CIF */
/* The fields in this register are 1 bit per XBAR RX CIF */

/* Fields in *_CIF_RX/TX_CTRL; used by AHUB FIFOs, and all other audio modules */

#define TEGRA210_AUDIOCIF_CTRL_FIFO_THRESHOLD_SHIFT	24
/* Channel count minus 1 */
#define TEGRA210_AUDIOCIF_CTRL_AUDIO_CHANNELS_SHIFT	20
/* Channel count minus 1 */
#define TEGRA210_AUDIOCIF_CTRL_CLIENT_CHANNELS_SHIFT	16

#define TEGRA210_AUDIOCIF_BITS_8			1
#define TEGRA210_AUDIOCIF_BITS_16			3
#define TEGRA210_AUDIOCIF_BITS_24			5
#define TEGRA210_AUDIOCIF_BITS_32			7

#define TEGRA210_AUDIOCIF_CTRL_AUDIO_BITS_SHIFT		12
#define TEGRA210_AUDIOCIF_CTRL_CLIENT_BITS_SHIFT	8
#define TEGRA210_AUDIOCIF_CTRL_EXPAND_SHIFT		6
#define TEGRA210_AUDIOCIF_CTRL_STEREO_CONV_SHIFT	4
#define TEGRA210_AUDIOCIF_CTRL_REPLICATE_SHIFT		3
#define TEGRA210_AUDIOCIF_CTRL_TRUNCATE_SHIFT		1
#define TEGRA210_AUDIOCIF_CTRL_MONO_CONV_SHIFT		0

/* Fields in *AHUBRAMCTL_CTRL; used by different AHUB modules */
#define TEGRA210_AHUBRAMCTL_CTRL_RW_READ		0
#define TEGRA210_AHUBRAMCTL_CTRL_RW_WRITE		(1 << 14)
#define TEGRA210_AHUBRAMCTL_CTRL_ADDR_INIT_EN		(1 << 13)
#define TEGRA210_AHUBRAMCTL_CTRL_SEQ_ACCESS_EN		(1 << 12)
#define TEGRA210_AHUBRAMCTL_CTRL_RAM_ADDR_MASK		0x1ff

#define TEGRA210_NUM_DAIS				67
#define TEGRA210_NUM_MUX_WIDGETS			50

/* size of TEGRA210_ROUTES */
#define TEGRA210_NUM_MUX_INPUT				54

#define TEGRA186_NUM_DAIS				108
#define TEGRA186_NUM_MUX_WIDGETS			79

/* size of TEGRA_ROUTES + TEGRA186_ROUTES */
#define TEGRA186_NUM_MUX_INPUT				82

#define TEGRA210_MAX_REGISTER_ADDR (TEGRA210_XBAR_PART2_RX +		\
	(TEGRA210_XBAR_RX_STRIDE * (TEGRA210_XBAR_AUDIO_RX_COUNT - 1)))

#define TEGRA186_XBAR_PART3_RX				0x600
#define TEGRA186_XBAR_AUDIO_RX_COUNT			115

#define TEGRA186_MAX_REGISTER_ADDR (TEGRA186_XBAR_PART3_RX +\
	(TEGRA210_XBAR_RX_STRIDE * (TEGRA186_XBAR_AUDIO_RX_COUNT - 1)))

#define TEGRA210_XBAR_REG_MASK_0		0xf1f03ff
#define TEGRA210_XBAR_REG_MASK_1		0x3f30031f
#define TEGRA210_XBAR_REG_MASK_2		0xff1cf313
#define TEGRA210_XBAR_REG_MASK_3		0x0
#define TEGRA210_XBAR_UPDATE_MAX_REG		3

#define TEGRA186_XBAR_REG_MASK_0		0xF3FFFFF
#define TEGRA186_XBAR_REG_MASK_1		0x3F310F1F
#define TEGRA186_XBAR_REG_MASK_2		0xFF3CF311
#define TEGRA186_XBAR_REG_MASK_3		0x3F0F00FF
#define TEGRA186_XBAR_UPDATE_MAX_REG		4

#define TEGRA_XBAR_UPDATE_MAX_REG		(TEGRA186_XBAR_UPDATE_MAX_REG)

/* T210 Modules Base address */
#define T210_ADMAIF_BASE_ADDR		0x702d0000
#define T210_I2S1_BASE_ADDR		0x702d1000
#define T210_I2S2_BASE_ADDR		0x702d1100
#define T210_I2S3_BASE_ADDR		0x702d1200
#define T210_I2S4_BASE_ADDR		0x702d1300
#define T210_I2S5_BASE_ADDR		0x702d1400
#define T210_AMX1_BASE_ADDR		0x702d3000
#define T210_AMX2_BASE_ADDR		0x702d3100
#define T210_ADX1_BASE_ADDR		0x702d3800
#define T210_ADX2_BASE_ADDR		0x702d3900
#define T210_AFC1_BASE_ADDR		0x702d7000
#define T210_AFC2_BASE_ADDR		0x702d7100
#define T210_AFC3_BASE_ADDR		0x702d7200
#define T210_AFC4_BASE_ADDR		0x702d7300
#define T210_AFC5_BASE_ADDR		0x702d7400
#define T210_AFC6_BASE_ADDR		0x702d7500
#define T210_SFC1_BASE_ADDR		0x702d2000
#define T210_SFC2_BASE_ADDR		0x702d2200
#define T210_SFC3_BASE_ADDR		0x702d2400
#define T210_SFC4_BASE_ADDR		0x702d2600
#define T210_MVC1_BASE_ADDR		0x702da000
#define T210_MVC2_BASE_ADDR		0x702da200
#define T210_IQC1_BASE_ADDR		0x702de000
#define T210_IQC2_BASE_ADDR		0x702de200
#define T210_DMIC1_BASE_ADDR		0x702d4000
#define T210_DMIC2_BASE_ADDR		0x702d4100
#define T210_DMIC3_BASE_ADDR		0x702d4200
#define T210_OPE1_BASE_ADDR		0x702d8000
#define T210_OPE2_BASE_ADDR		0x702d8400
#define T210_AMIXER1_BASE_ADDR		0x702dbb00

/* T186 Modules Base address */
#define T186_ADMAIF_BASE_ADDR		0x0290F000
#define T186_I2S1_BASE_ADDR		0x02901000
#define T186_I2S2_BASE_ADDR		0x02901100
#define T186_I2S3_BASE_ADDR		0x02901200
#define T186_I2S4_BASE_ADDR		0x02901300
#define T186_I2S5_BASE_ADDR		0x02901400
#define T186_I2S6_BASE_ADDR		0x02901500
#define T186_AMX1_BASE_ADDR		0x02903000
#define T186_AMX2_BASE_ADDR		0x02903100
#define T186_AMX3_BASE_ADDR		0x02903200
#define T186_AMX4_BASE_ADDR		0x02903300
#define T186_ADX1_BASE_ADDR		0x02903800
#define T186_ADX2_BASE_ADDR		0x02903900
#define T186_ADX3_BASE_ADDR		0x02903a00
#define T186_ADX4_BASE_ADDR		0x02903b00
#define T186_AFC1_BASE_ADDR		0x02907000
#define T186_AFC2_BASE_ADDR		0x02907100
#define T186_AFC3_BASE_ADDR		0x02907200
#define T186_AFC4_BASE_ADDR		0x02907300
#define T186_AFC5_BASE_ADDR		0x02907400
#define T186_AFC6_BASE_ADDR		0x02907500
#define T186_SFC1_BASE_ADDR		0x02902000
#define T186_SFC2_BASE_ADDR		0x02902200
#define T186_SFC3_BASE_ADDR		0x02902400
#define T186_SFC4_BASE_ADDR		0x02902600
#define T186_MVC1_BASE_ADDR		0x0290A000
#define T186_MVC2_BASE_ADDR		0x0290A200
#define T186_IQC1_BASE_ADDR		0x0290E000
#define T186_IQC2_BASE_ADDR		0x0290E200
#define T186_DMIC1_BASE_ADDR		0x02904000
#define T186_DMIC2_BASE_ADDR		0x02904100
#define T186_DMIC3_BASE_ADDR		0x02904200
#define T186_DMIC4_BASE_ADDR		0x02904300
#define T186_OPE1_BASE_ADDR		0x02908000
#define T186_AMIXER1_BASE_ADDR		0x0290BB00
#define T186_ASRC1_BASE_ADDR		0x02910000
#define T186_ARAD1_BASE_ADDR		0x0290E400
#define T186_DSPK1_BASE_ADDR		0x02905000
#define T186_DSPK2_BASE_ADDR		0x02905100


struct tegra210_xbar_cif_conf {
	unsigned int threshold;
	unsigned int audio_channels;
	unsigned int client_channels;
	unsigned int audio_bits;
	unsigned int client_bits;
	unsigned int expand;
	unsigned int stereo_conv;
	union {
		unsigned int fifo_size_downshift;
		unsigned int replicate;
	};
	unsigned int truncate;
	unsigned int mono_conv;
};

struct tegra_xbar_soc_data {
	const struct regmap_config *regmap_config;
	unsigned int mask[4];
	unsigned int reg_count;
	unsigned int reg_offset;
	int (*xbar_registration)(struct platform_device *pdev);
};

struct tegra_xbar {
	struct clk *clk;
	struct regmap *regmap;
	const struct tegra_xbar_soc_data *soc_data;
};

/* Extension of soc_bytes structure defined in sound/soc.h */
struct tegra_soc_bytes {
	struct soc_bytes soc;
	u32 shift; /* Used as offset for ahub ram related programing */
};

void tegra210_xbar_set_cif(struct regmap *regmap, unsigned int reg,
			  struct tegra210_xbar_cif_conf *conf);
void tegra210_xbar_write_ahubram(struct regmap *regmap, unsigned int reg_ctrl,
				unsigned int reg_data, unsigned int ram_offset,
				unsigned int *data, size_t size);
void tegra210_xbar_read_ahubram(struct regmap *regmap, unsigned int reg_ctrl,
				unsigned int reg_data, unsigned int ram_offset,
				unsigned int *data, size_t size);

/* Utility structures for using mixer control of type snd_soc_bytes */
#define TEGRA_SOC_BYTES_EXT(xname, xbase, xregs, xshift, xmask, \
	xhandler_get, xhandler_put, xinfo) \
	{.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	 .info = xinfo, .get = xhandler_get, .put = xhandler_put, \
	 .private_value = ((unsigned long)&(struct tegra_soc_bytes) \
		{.soc.base = xbase, .soc.num_regs = xregs, .soc.mask = xmask, \
		 .shift = xshift })}
#endif
