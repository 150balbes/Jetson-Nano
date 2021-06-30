/*
 * tegra_asoc_utils_alt.c - MCLK and DAP Utility driver
 *
 * Author: Stephen Warren <swarren@nvidia.com>
 * Copyright (c) 2010-2018 NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinconf-tegra.h>

#include "tegra_asoc_utils_alt.h"

int tegra_alt_asoc_utils_set_rate(struct tegra_asoc_audio_clock_info *data,
				int srate,
				int mclk,
				u32 clk_out_rate)
{
	int new_baseclock;
	int ahub_rate = 0;
	bool clk_change;
	int err;

	switch (srate) {
	case 11025:
	case 22050:
	case 44100:
	case 88200:
	case 176400:
		if (data->soc < TEGRA_ASOC_UTILS_SOC_TEGRA186)
			new_baseclock = 338688000;
		else {
			new_baseclock = data->clk_rates[PLLA_x11025_RATE];
			mclk = data->clk_rates[PLLA_OUT0_x11025_RATE];
			ahub_rate = data->clk_rates[AHUB_x11025_RATE];

			if (srate <= 11025) {
				/* half the pll_a_out0 to support lower
				 * sampling rate divider
				 */
				mclk = mclk >> 1;
				ahub_rate = ahub_rate >> 1;
			}
			clk_out_rate = srate * data->mclk_scale;
		}
		break;
	case 8000:
	case 16000:
	case 32000:
	case 48000:
	case 64000:
	case 96000:
	case 192000:
		if (data->soc < TEGRA_ASOC_UTILS_SOC_TEGRA186)
			new_baseclock = 368640000;
		else {
			new_baseclock = data->clk_rates[PLLA_x8000_RATE];
			mclk = data->clk_rates[PLLA_OUT0_x8000_RATE];
			ahub_rate = data->clk_rates[AHUB_x8000_RATE];

			if (srate <= 8000) {
				/* half the pll_a_out0 to support lower
				 * sampling rate divider
				 */
				mclk = mclk >> 1;
				ahub_rate = ahub_rate >> 1;
			}
			clk_out_rate = srate * data->mclk_scale;
		}
		break;
	default:
		return -EINVAL;
	}

	clk_out_rate = data->mclk_rate ? data->mclk_rate : clk_out_rate;

	clk_change = ((new_baseclock != data->set_baseclock) ||
			(mclk != data->set_mclk) ||
			(clk_out_rate != data->set_clk_out_rate));

	if (!clk_change)
		return 0;

	/* Don't change rate if already one dai-link is using it */
	if (data->lock_count)
		return -EINVAL;

	data->set_baseclock = 0;
	data->set_mclk = 0;

	err = clk_set_rate(data->clk_pll_a, new_baseclock);
	if (err) {
		dev_err(data->dev, "Can't set pll_a rate: %d\n", err);
		return err;
	}

	err = clk_set_rate(data->clk_pll_a_out0, mclk);
	if (err) {
		dev_err(data->dev, "Can't set clk_pll_a_out0 rate: %d\n", err);
		return err;
	}

	if (data->soc > TEGRA_ASOC_UTILS_SOC_TEGRA210) {
		err = clk_set_rate(data->clk_ahub, ahub_rate);
		if (err) {
			dev_err(data->dev, "Can't set clk_cdev1 rate: %d\n",
				err);
			return err;
		}
	}

	err = clk_set_rate(data->clk_cdev1, clk_out_rate);
	if (err) {
		dev_err(data->dev, "Can't set clk_cdev1 rate: %d\n", err);
		return err;
	}

	data->set_baseclock = new_baseclock;
	data->set_mclk = mclk;
	data->set_clk_out_rate = clk_out_rate;

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_alt_asoc_utils_set_rate);

void tegra_alt_asoc_utils_lock_clk_rate(struct tegra_asoc_audio_clock_info *data,
				    int lock)
{
	if (lock)
		data->lock_count++;
	else if (data->lock_count)
		data->lock_count--;
}
EXPORT_SYMBOL_GPL(tegra_alt_asoc_utils_lock_clk_rate);

int tegra_alt_asoc_utils_clk_enable(struct tegra_asoc_audio_clock_info *data)
{
	int err;

	if (data->soc == TEGRA_ASOC_UTILS_SOC_TEGRA186)
		reset_control_reset(data->clk_cdev1_rst);

	err = clk_prepare_enable(data->clk_cdev1);
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
	clk_disable_unprepare(data->clk_cdev1);
	data->clk_cdev1_state = 0;

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_alt_asoc_utils_clk_disable);

int tegra_alt_asoc_utils_init(struct tegra_asoc_audio_clock_info *data,
			  struct device *dev, struct snd_soc_card *card)
{
	int ret;

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

	data->clk_m = devm_clk_get(dev, "clk_m");
	if (IS_ERR(data->clk_m)) {
		dev_err(data->dev, "Can't retrieve clk clk_m\n");
		ret = PTR_ERR(data->clk_m);
		goto err;
	}

	data->clk_pll_a = devm_clk_get(dev, "pll_a");
	if (IS_ERR(data->clk_pll_a)) {
		dev_err(data->dev, "Can't retrieve clk pll_a\n");
		ret = PTR_ERR(data->clk_pll_a);
		goto err;
	}

	data->clk_pll_a_out0 = devm_clk_get(dev, "pll_a_out0");
	if (IS_ERR(data->clk_pll_a_out0)) {
		dev_err(data->dev, "Can't retrieve clk pll_a_out0\n");
		ret = PTR_ERR(data->clk_pll_a_out0);
		goto err;
	}

	data->clk_cdev1 = devm_clk_get(dev, "extern1");
	if (IS_ERR(data->clk_cdev1)) {
		dev_err(data->dev, "Can't retrieve clk cdev1\n");
		ret = PTR_ERR(data->clk_cdev1);
		goto err;
	}

	/* Control the aud mclk rate and parent for usecases which might
	 * need fixed rate and needs to be derived from other possible
	 * parents of aud mclk clk source
	 */
	data->clk_mclk_parent = devm_clk_get(dev, "mclk_parent");
	if (IS_ERR(data->clk_mclk_parent))
		dev_dbg(data->dev, "Can't retrieve mclk parent clk\n");

	if (data->soc > TEGRA_ASOC_UTILS_SOC_TEGRA210) {
		data->clk_ahub = devm_clk_get(dev, "ahub");
		if (IS_ERR(data->clk_ahub)) {
			dev_err(data->dev, "Can't retrieve clk ahub\n");
			ret = PTR_ERR(data->clk_ahub);
			goto err;
		}

		if (data->soc == TEGRA_ASOC_UTILS_SOC_TEGRA186) {
			data->clk_cdev1_rst = devm_reset_control_get(dev,
							"extern1_rst");
			if (IS_ERR(data->clk_cdev1_rst)) {
				dev_err(dev,
				"Reset control is not found, err: %ld\n",
				PTR_ERR(data->clk_cdev1_rst));
				return PTR_ERR(data->clk_cdev1_rst);
			}
			reset_control_reset(data->clk_cdev1_rst);
		}
	}

	return 0;

err:
	return ret;
}
EXPORT_SYMBOL_GPL(tegra_alt_asoc_utils_init);

int tegra_alt_asoc_utils_set_parent(struct tegra_asoc_audio_clock_info *data,
			int is_i2s_master)
{
	int ret = -ENODEV;

	if (is_i2s_master) {
		ret = clk_set_parent(data->clk_cdev1, data->clk_pll_a_out0);
		if (ret) {
			dev_err(data->dev, "Can't set clk cdev1/extern1 parent");
			return ret;
		}
	} else {
		ret = clk_set_parent(data->clk_cdev1, data->clk_m);
		if (ret) {
			dev_err(data->dev, "Can't set clk cdev1/extern1 parent");
			return ret;
		}

		ret = clk_set_rate(data->clk_cdev1, 13000000);
		if (ret) {
			dev_err(data->dev, "Can't set clk rate");
			return ret;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_alt_asoc_utils_set_parent);

int tegra_alt_asoc_utils_set_extern_parent(
	struct tegra_asoc_audio_clock_info *data, const char *parent)
{
	unsigned long rate;
	int err = 0;

	rate = clk_get_rate(data->clk_cdev1);
	if (!IS_ERR(data->clk_mclk_parent))
		err = clk_set_parent(data->clk_cdev1, data->clk_mclk_parent);
	else if (!strcmp(parent, "clk_m"))
		err = clk_set_parent(data->clk_cdev1, data->clk_m);
	else if (!strcmp(parent, "pll_a_out0"))
		err = clk_set_parent(data->clk_cdev1, data->clk_pll_a_out0);

	if (err) {
		dev_err(data->dev, "Can't set aud mclk clock parent");
		return err;
	}

	err = clk_set_rate(data->clk_cdev1, rate);
	if (err) {
		dev_err(data->dev, "Can't set clk rate");
		return err;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_alt_asoc_utils_set_extern_parent);

MODULE_AUTHOR("Stephen Warren <swarren@nvidia.com>");
MODULE_DESCRIPTION("Tegra ASoC utility code");
MODULE_LICENSE("GPL");
