/*
 * tegra_asoc_utils_alt.c - MCLK and DAP Utility driver
 *
 * Author: Stephen Warren <swarren@nvidia.com>
 * Copyright (c) 2010-2019 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/clk/tegra.h>
#include <linux/reset.h>
#include <sound/soc.h>
#include "tegra_asoc_utils_alt.h"

enum rate_type {
	ODD_RATE,
	EVEN_RATE,
	NUM_RATE_TYPE,
};

unsigned int tegra210_pll_base_rate[NUM_RATE_TYPE] = {
	338688000,
	368640000,
};

unsigned int tegra186_pll_base_rate[NUM_RATE_TYPE] = {
	270950400,
	245760000,
};

unsigned int default_pll_out_rate[NUM_RATE_TYPE] = {
	45158400,
	49152000,
};

int tegra_alt_asoc_utils_set_rate(struct tegra_asoc_audio_clock_info *data,
				  unsigned int srate, unsigned int pll_out,
				  unsigned int aud_mclk)
{
	unsigned int new_pll_base;
	int err;

	switch (srate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
	case 176400:
		new_pll_base = data->pll_base_rate[ODD_RATE];
		pll_out = default_pll_out_rate[ODD_RATE];
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
	case 192000:
		new_pll_base = data->pll_base_rate[EVEN_RATE];
		pll_out = default_pll_out_rate[EVEN_RATE];
		break;
	default:
		return -EINVAL;
	}

	/* reduce pll_out rate to support lower sampling rates */
	if (srate <= 11025)
		pll_out = pll_out >> 1;
	if (data->mclk_scale)
		aud_mclk = srate * data->mclk_scale;

	if (data->set_pll_base_rate != new_pll_base) {
		err = clk_set_rate(data->clk_pll_base, new_pll_base);
		if (err) {
			dev_err(data->dev, "Can't set clk_pll_base rate: %d\n",
				err);
			return err;
		}
		data->set_pll_base_rate = new_pll_base;
	}

	if (data->set_pll_out_rate != pll_out) {
		err = clk_set_rate(data->clk_pll_out, pll_out);
		if (err) {
			dev_err(data->dev, "Can't set clk_pll_out rate: %d\n",
				err);
			return err;
		}

		data->set_pll_out_rate = pll_out;
	}

	if (data->set_aud_mclk_rate != aud_mclk) {
		err = clk_set_rate(data->clk_aud_mclk, aud_mclk);
		if (err) {
			dev_err(data->dev, "Can't set clk_cdev1 rate: %d\n",
				err);
			return err;
		}
		data->set_aud_mclk_rate = aud_mclk;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_alt_asoc_utils_set_rate);

int tegra_alt_asoc_utils_clk_enable(struct tegra_asoc_audio_clock_info *data)
{
	int err;

	if (data->soc == TEGRA_ASOC_UTILS_SOC_TEGRA186)
		reset_control_reset(data->clk_cdev1_rst);

	err = clk_prepare_enable(data->clk_aud_mclk);
	if (err) {
		dev_err(data->dev, "Can't enable cdev1: %d\n", err);
		return err;
	}
	data->clk_cdev1_state = 1;

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_alt_asoc_utils_clk_enable);

int tegra_alt_asoc_utils_clk_disable(struct tegra_asoc_audio_clock_info *data)
{
	clk_disable_unprepare(data->clk_aud_mclk);
	data->clk_cdev1_state = 0;

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_alt_asoc_utils_clk_disable);

int tegra_alt_asoc_utils_init(struct tegra_asoc_audio_clock_info *data,
			  struct device *dev, struct snd_soc_card *card)
{
	data->dev = dev;
	data->card = card;

	if (of_machine_is_compatible("nvidia,tegra210")  ||
		of_machine_is_compatible("nvidia,tegra210b01"))
		data->soc = TEGRA_ASOC_UTILS_SOC_TEGRA210;
	else if (of_machine_is_compatible("nvidia,tegra186"))
		data->soc = TEGRA_ASOC_UTILS_SOC_TEGRA186;
	else if (of_machine_is_compatible("nvidia,tegra194"))
		data->soc = TEGRA_ASOC_UTILS_SOC_TEGRA194;
	else
		/* DT boot, but unknown SoC */
		return -EINVAL;

	data->clk_pll_base = devm_clk_get(dev, "pll_a");
	if (IS_ERR(data->clk_pll_base)) {
		dev_err(data->dev, "Can't retrieve clk pll_a\n");
		return PTR_ERR(data->clk_pll_base);
	}

	data->clk_pll_out = devm_clk_get(dev, "pll_a_out0");
	if (IS_ERR(data->clk_pll_out)) {
		dev_err(data->dev, "Can't retrieve clk pll_a_out0\n");
		return PTR_ERR(data->clk_pll_out);
	}

	data->clk_aud_mclk = devm_clk_get(dev, "extern1");
	if (IS_ERR(data->clk_aud_mclk)) {
		dev_err(data->dev, "Can't retrieve clk cdev1\n");
		return PTR_ERR(data->clk_aud_mclk);
	}

	if (data->soc == TEGRA_ASOC_UTILS_SOC_TEGRA186) {
		data->clk_cdev1_rst = devm_reset_control_get(dev,
							     "extern1_rst");
		if (IS_ERR(data->clk_cdev1_rst)) {
			dev_err(dev, "Reset control is not found, err: %ld\n",
				PTR_ERR(data->clk_cdev1_rst));
			return PTR_ERR(data->clk_cdev1_rst);
		}
		reset_control_reset(data->clk_cdev1_rst);
	}

	if (data->soc < TEGRA_ASOC_UTILS_SOC_TEGRA186)
		data->pll_base_rate = tegra210_pll_base_rate;
	else
		data->pll_base_rate = tegra186_pll_base_rate;

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_alt_asoc_utils_init);

MODULE_AUTHOR("Stephen Warren <swarren@nvidia.com>");
MODULE_DESCRIPTION("Tegra ASoC utility code");
MODULE_LICENSE("GPL");
