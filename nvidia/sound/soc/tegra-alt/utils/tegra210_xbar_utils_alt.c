/*
 * tegra210_xbar_utils_alt.c - Tegra XBAR driver utils
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

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/tegra_pm_domains.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <soc/tegra/chip-id.h>
#include <sound/soc.h>
#include <linux/clk/tegra.h>
#include "tegra210_xbar_alt.h"
#include "tegra210_xbar_utils_alt.h"

static struct tegra_xbar *xbar;

int tegra210_xbar_set_clock(unsigned long rate)
{
	int ret = 0;

	ret = clk_set_rate(xbar->clk_parent, rate);
	if (ret)
		pr_info("Failed to set clock rate of pll_a_out0\n");

	ret = clk_set_rate(xbar->clk, rate);
	if (ret)
		pr_info("Failed to set clock rate of ahub\n");

	return 0;
}
EXPORT_SYMBOL_GPL(tegra210_xbar_set_clock);

void tegra210_xbar_set_cif(struct regmap *regmap, unsigned int reg,
			  struct tegra210_xbar_cif_conf *conf)
{
	unsigned int value;

	value = (conf->threshold <<
			TEGRA210_AUDIOCIF_CTRL_FIFO_THRESHOLD_SHIFT) |
		((conf->audio_channels - 1) <<
			TEGRA210_AUDIOCIF_CTRL_AUDIO_CHANNELS_SHIFT) |
		((conf->client_channels - 1) <<
			TEGRA210_AUDIOCIF_CTRL_CLIENT_CHANNELS_SHIFT) |
		(conf->audio_bits <<
			TEGRA210_AUDIOCIF_CTRL_AUDIO_BITS_SHIFT) |
		(conf->client_bits <<
			TEGRA210_AUDIOCIF_CTRL_CLIENT_BITS_SHIFT) |
		(conf->expand <<
			TEGRA210_AUDIOCIF_CTRL_EXPAND_SHIFT) |
		(conf->stereo_conv <<
			TEGRA210_AUDIOCIF_CTRL_STEREO_CONV_SHIFT) |
		(conf->replicate <<
			TEGRA210_AUDIOCIF_CTRL_REPLICATE_SHIFT) |
		(conf->truncate <<
			TEGRA210_AUDIOCIF_CTRL_TRUNCATE_SHIFT) |
		(conf->mono_conv <<
			TEGRA210_AUDIOCIF_CTRL_MONO_CONV_SHIFT);

	regmap_update_bits(regmap, reg, 0x3fffffff, value);
}
EXPORT_SYMBOL_GPL(tegra210_xbar_set_cif);

void tegra210_xbar_write_ahubram(struct regmap *regmap, unsigned int reg_ctrl,
				unsigned int reg_data, unsigned int ram_offset,
				unsigned int *data, size_t size)
{
	unsigned int val = 0;
	int i = 0;

	val = (ram_offset << TEGRA210_AHUBRAMCTL_CTRL_RAM_ADDR_SHIFT) &
		TEGRA210_AHUBRAMCTL_CTRL_RAM_ADDR_MASK;
	val |= TEGRA210_AHUBRAMCTL_CTRL_ADDR_INIT_EN;
	val |= TEGRA210_AHUBRAMCTL_CTRL_SEQ_ACCESS_EN;
	val |= TEGRA210_AHUBRAMCTL_CTRL_RW_WRITE;

	regmap_write(regmap, reg_ctrl, val);
	for (i = 0; i < size; i++)
		regmap_write(regmap, reg_data, data[i]);

	return;
}
EXPORT_SYMBOL_GPL(tegra210_xbar_write_ahubram);

void tegra210_xbar_read_ahubram(struct regmap *regmap, unsigned int reg_ctrl,
				unsigned int reg_data, unsigned int ram_offset,
				unsigned int *data, size_t size)
{
	unsigned int val = 0;
	int i = 0;

	val = (ram_offset << TEGRA210_AHUBRAMCTL_CTRL_RAM_ADDR_SHIFT) &
		TEGRA210_AHUBRAMCTL_CTRL_RAM_ADDR_MASK;
	val |= TEGRA210_AHUBRAMCTL_CTRL_ADDR_INIT_EN;
	val |= TEGRA210_AHUBRAMCTL_CTRL_SEQ_ACCESS_EN;
	val |= TEGRA210_AHUBRAMCTL_CTRL_RW_READ;

	regmap_write(regmap, reg_ctrl, val);
	/* Since all ahub non-io modules work under same ahub clock it is not
	   necessary to check ahub read busy bit after every read */
	for (i = 0; i < size; i++)
		regmap_read(regmap, reg_data, &data[i]);

	return;
}
EXPORT_SYMBOL_GPL(tegra210_xbar_read_ahubram);

int tegra210_xbar_read_reg (unsigned int reg, unsigned int *val)
{
	int ret;

	ret = regmap_read (xbar->regmap, reg, val);
	return ret;
}
EXPORT_SYMBOL_GPL(tegra210_xbar_read_reg);

int tegra_xbar_get_value_enum(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_dapm_kcontrol_codec(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int reg_count, reg_val, val, bit_pos = 0, i;
	unsigned int reg[TEGRA_XBAR_UPDATE_MAX_REG];

	reg_count = xbar->soc_data->reg_count;

	if (reg_count > TEGRA_XBAR_UPDATE_MAX_REG)
		return -EINVAL;

	for (i = 0; i < reg_count; i++) {
		reg[i] = (e->reg +
			xbar->soc_data->reg_offset * i);
		reg_val = snd_soc_read(codec, reg[i]);
		val = reg_val & xbar->soc_data->mask[i];
		if (val != 0) {
			bit_pos = ffs(val) +
					(8*codec->component.val_bytes * i);
			break;
		}
	}

	for (i = 0; i < e->items; i++) {
		if (bit_pos == e->values[i]) {
			ucontrol->value.enumerated.item[0] = i;
			break;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_xbar_get_value_enum);

int tegra_xbar_put_value_enum(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_soc_dapm_kcontrol_codec(kcontrol);
	struct snd_soc_dapm_context *dapm =
				snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned int *item = ucontrol->value.enumerated.item;
	unsigned int change = 0, reg_idx = 0, value, *mask, bit_pos = 0;
	unsigned int i, reg_count, reg_val = 0, update_idx = 0;
	unsigned int reg[TEGRA_XBAR_UPDATE_MAX_REG];
	struct snd_soc_dapm_update update[TEGRA_XBAR_UPDATE_MAX_REG] = {
				{ NULL } };

	/* initialize the reg_count and mask from soc_data */
	reg_count = xbar->soc_data->reg_count;
	mask = (unsigned int *)xbar->soc_data->mask;

	if (item[0] >= e->items || reg_count > TEGRA_XBAR_UPDATE_MAX_REG)
		return -EINVAL;

	value = e->values[item[0]];

	if (value) {
		/* get the register index and value to set */
		reg_idx = (value - 1) / (8 * codec->component.val_bytes);
		bit_pos = (value - 1) % (8 * codec->component.val_bytes);
		reg_val = BIT(bit_pos);
	}

	for (i = 0; i < reg_count; i++) {
		reg[i] = e->reg + xbar->soc_data->reg_offset * i;
		if (i == reg_idx) {
			change |= snd_soc_test_bits(codec, reg[i],
							mask[i], reg_val);
			/* set the selected register */
			update[reg_count - 1].reg = reg[reg_idx];
			update[reg_count - 1].mask = mask[reg_idx];
			update[reg_count - 1].val = reg_val;
		} else {
			/* accumulate the change to update the DAPM path
			    when none is selected */
			change |= snd_soc_test_bits(codec, reg[i],
							mask[i], 0);

			/* clear the register when not selected */
			update[update_idx].reg = reg[i];
			update[update_idx].mask = mask[i];
			update[update_idx++].val = 0;
		}
	}

	/* power the widgets */
	if (change) {
		for (i = 0; i < reg_count; i++) {
			update[i].kcontrol = kcontrol;
			snd_soc_dapm_mux_update_power(dapm,
				kcontrol, item[0], e, &update[i]);
		}
	}

	return change;
}
EXPORT_SYMBOL_GPL(tegra_xbar_put_value_enum);

bool tegra_xbar_volatile_reg(struct device *dev, unsigned int reg)
{
	return false;
}
EXPORT_SYMBOL_GPL(tegra_xbar_volatile_reg);

int tegra_xbar_runtime_suspend(struct device *dev)
{

#ifdef CONFIG_TEGRA186_AHC
	tegra186_free_ahc_interrupts();
#endif
	regcache_cache_only(xbar->regmap, true);
	regcache_mark_dirty(xbar->regmap);

	if (!(tegra_platform_is_unit_fpga() || tegra_platform_is_fpga())) {
		clk_disable_unprepare(xbar->clk);
		clk_disable_unprepare(xbar->clk_ape);
		clk_disable_unprepare(xbar->clk_apb2ape);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(tegra_xbar_runtime_suspend);

int tegra_xbar_runtime_resume(struct device *dev)
{
	int ret;

	if (!(tegra_platform_is_unit_fpga() || tegra_platform_is_fpga())) {
		ret = clk_prepare_enable(xbar->clk_ape);
		if (ret) {
			dev_err(dev, "clk_prepare_enable failed: %d\n", ret);
			return ret;
		}

		ret = clk_prepare_enable(xbar->clk_apb2ape);
		if (ret) {
			dev_err(dev, "clk_prepare_enable failed: %d\n", ret);
			return ret;
		}

		ret = clk_prepare_enable(xbar->clk);
		if (ret) {
			dev_err(dev, "clk_prepare_enable failed: %d\n", ret);
			return ret;
		}
	}
#ifdef CONFIG_TEGRA186_AHC
	tegra186_setup_ahc_interrupts();
#endif
	regcache_cache_only(xbar->regmap, false);

	if (!xbar->is_shutdown)
		regcache_sync(xbar->regmap);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_xbar_runtime_resume);

void tegra_xbar_shutdown(struct platform_device *pdev)
{
	xbar->is_shutdown = true;
}
EXPORT_SYMBOL_GPL(tegra_xbar_shutdown);

int tegra_xbar_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra_xbar_runtime_suspend(&pdev->dev);

	tegra_pd_remove_device(&pdev->dev);

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_xbar_remove);

int tegra_xbar_codec_probe(struct snd_soc_codec *codec)
{
	codec->control_data = xbar->regmap;

	return 0;
}
EXPORT_SYMBOL_GPL(tegra_xbar_codec_probe);


int tegra_xbar_probe(struct platform_device *pdev,
					struct tegra_xbar_soc_data *soc_data)
{
	void __iomem *regs;
	struct resource *res;
	struct clk *parent_clk;
	int ret;

	xbar = devm_kzalloc(&pdev->dev, sizeof(*xbar), GFP_KERNEL);
	if (!xbar) {
		dev_err(&pdev->dev, "Can't allocate xbar\n");
		ret = -ENOMEM;
		goto err;
	}

	xbar->soc_data = soc_data;
	xbar->is_shutdown = false;

	platform_set_drvdata(pdev, xbar);

	if (!(tegra_platform_is_unit_fpga() || tegra_platform_is_fpga())) {
		xbar->clk = devm_clk_get(&pdev->dev, "ahub");
		if (IS_ERR(xbar->clk)) {
			dev_err(&pdev->dev, "Can't retrieve ahub clock\n");
			ret = PTR_ERR(xbar->clk);
			goto err;
		}

		xbar->clk_parent = devm_clk_get(&pdev->dev, "pll_a_out0");
		if (IS_ERR(xbar->clk_parent)) {
			dev_err(&pdev->dev, "Can't retrieve pll_a_out0 clock\n");
			ret = PTR_ERR(xbar->clk_parent);
			goto err;
		}

		xbar->clk_apb2ape = devm_clk_get(&pdev->dev, "apb2ape");
		if (IS_ERR(xbar->clk_apb2ape)) {
			dev_err(&pdev->dev, "Can't retrieve apb2ape clock\n");
			ret = PTR_ERR(xbar->clk_apb2ape);
			goto err;
		}

		xbar->clk_ape = devm_clk_get(&pdev->dev, "xbar.ape");
		if (IS_ERR(xbar->clk_ape)) {
			dev_err(&pdev->dev, "Can't retrieve ape clock\n");
			ret = PTR_ERR(xbar->clk_ape);
			goto err;
		}
	}

	parent_clk = clk_get_parent(xbar->clk);
	if (IS_ERR(parent_clk)) {
		dev_err(&pdev->dev, "Can't get parent clock for xbar\n");
		ret = PTR_ERR(parent_clk);
		goto err;
	}

	if (!(tegra_platform_is_unit_fpga() || tegra_platform_is_fpga())) {
		ret = clk_set_parent(xbar->clk, xbar->clk_parent);
		if (ret) {
			dev_err(&pdev->dev, "Failed to set parent clock with pll_a_out0\n");
			goto err;
		}
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "No memory resource for admaif\n");
		ret = -ENODEV;
		goto err;
	}

	regs = devm_ioremap_resource(&pdev->dev, res);
	if (!regs) {
		dev_err(&pdev->dev, "request/iomap region failed\n");
		ret = -ENODEV;
		goto err_clk_set_parent;
	}

	xbar->regmap = devm_regmap_init_mmio(&pdev->dev, regs,
					     soc_data->regmap_config);
	if (IS_ERR(xbar->regmap)) {
		dev_err(&pdev->dev, "regmap init failed\n");
		ret = PTR_ERR(xbar->regmap);
		goto err_clk_set_parent;
	}
	regcache_cache_only(xbar->regmap, true);

	tegra_pd_add_device(&pdev->dev);

	pm_runtime_enable(&pdev->dev);
	if (!pm_runtime_enabled(&pdev->dev)) {
		ret = tegra_xbar_runtime_resume(&pdev->dev);
		if (ret)
			goto err_pm_disable;
	}

	ret = soc_data->xbar_registration(pdev);
	if (ret == -EBUSY)
		goto err_suspend;

	return 0;

err_suspend:
	if (!pm_runtime_status_suspended(&pdev->dev))
		tegra_xbar_runtime_suspend(&pdev->dev);
err_pm_disable:
	pm_runtime_disable(&pdev->dev);
	tegra_pd_remove_device(&pdev->dev);
err_clk_set_parent:
	clk_set_parent(xbar->clk, parent_clk);
err:
	return ret;
}
EXPORT_SYMBOL_GPL(tegra_xbar_probe);
MODULE_LICENSE("GPL");
