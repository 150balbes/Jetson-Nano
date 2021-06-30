/*
 * Copyright (c) 2014-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <dt-bindings/soc/tegra-io-pads.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/padctrl/padctrl.h>
#include <linux/slab.h>
#include <soc/tegra/pmc.h>

struct tegra_io_pad_info {
	const char *name;
	int id;
};

#define TEGRA_IO_PAD_INFO(_id, _name)		\
{						\
	.name = #_name,				\
	.id = TEGRA_IO_PAD_GROUP_##_id,		\
}

static struct tegra_io_pad_info tegra_io_pads_info[] = {
	TEGRA_IO_PAD_INFO(AUDIO, audio),
	TEGRA_IO_PAD_INFO(AUDIO_HV, audio-hv),
	TEGRA_IO_PAD_INFO(CAM, cam),
	TEGRA_IO_PAD_INFO(CONN, conn),
	TEGRA_IO_PAD_INFO(CSIA, csia),
	TEGRA_IO_PAD_INFO(CSIB, csib),
	TEGRA_IO_PAD_INFO(CSIC, csic),
	TEGRA_IO_PAD_INFO(CSID, csid),
	TEGRA_IO_PAD_INFO(CSIE, csie),
	TEGRA_IO_PAD_INFO(CSIF, csif),
	TEGRA_IO_PAD_INFO(DBG, dbg),
	TEGRA_IO_PAD_INFO(DEBUG_NONAO, debug-nonao),
	TEGRA_IO_PAD_INFO(DMIC, dmic),
	TEGRA_IO_PAD_INFO(DMIC_HV, dmic-hv),
	TEGRA_IO_PAD_INFO(DP, dp),
	TEGRA_IO_PAD_INFO(DSI, dsi),
	TEGRA_IO_PAD_INFO(DSIB, dsib),
	TEGRA_IO_PAD_INFO(DSIC, dsic),
	TEGRA_IO_PAD_INFO(DSID, dsid),
	TEGRA_IO_PAD_INFO(EMMC, emmc),
	TEGRA_IO_PAD_INFO(EMMC2, emmc2),
	TEGRA_IO_PAD_INFO(GPIO, gpio),
	TEGRA_IO_PAD_INFO(EDP, edp),
	TEGRA_IO_PAD_INFO(HDMI, hdmi),
	TEGRA_IO_PAD_INFO(HDMI_DP0, hdmi-dp0),
	TEGRA_IO_PAD_INFO(HDMI_DP1, hdmi-dp1),
	TEGRA_IO_PAD_INFO(HSIC, hsic),
	TEGRA_IO_PAD_INFO(LVDS, lvds),
	TEGRA_IO_PAD_INFO(MIPI_BIAS, mipi-bias),
	TEGRA_IO_PAD_INFO(PEX_BIAS, pex-bias),
	TEGRA_IO_PAD_INFO(PEX_CLK_BIAS, pex-clk-bias),
	TEGRA_IO_PAD_INFO(PEX_CLK1, pex-clk1),
	TEGRA_IO_PAD_INFO(PEX_CLK2, pex-clk2),
	TEGRA_IO_PAD_INFO(PEX_CLK3, pex-clk3),
	TEGRA_IO_PAD_INFO(PEX_CTRL, pex-ctrl),
	TEGRA_IO_PAD_INFO(SDMMC1, sdmmc1),
	TEGRA_IO_PAD_INFO(SDMMC1_HV, sdmmc1-hv),
	TEGRA_IO_PAD_INFO(SDMMC2_HV, sdmmc2-hv),
	TEGRA_IO_PAD_INFO(SDMMC3, sdmmc3),
	TEGRA_IO_PAD_INFO(SDMMC3_HV, sdmmc3-hv),
	TEGRA_IO_PAD_INFO(SDMMC4, sdmmc4),
	TEGRA_IO_PAD_INFO(SPI, spi),
	TEGRA_IO_PAD_INFO(SPI_HV, spi-hv),
	TEGRA_IO_PAD_INFO(UART, uart),
	TEGRA_IO_PAD_INFO(USB_BIAS, usb-bias),
	TEGRA_IO_PAD_INFO(USB0, usb0),
	TEGRA_IO_PAD_INFO(USB1, usb1),
	TEGRA_IO_PAD_INFO(USB2, usb2),
	TEGRA_IO_PAD_INFO(USB3, usb3),
	TEGRA_IO_PAD_INFO(BB, bb),
	TEGRA_IO_PAD_INFO(SYS, sys),
	TEGRA_IO_PAD_INFO(HV, hv),
	TEGRA_IO_PAD_INFO(UFS, ufs),
	TEGRA_IO_PAD_INFO(AO_HV, ao-hv),
	TEGRA_IO_PAD_INFO(DDR_DVI, ddr_dvi),
	TEGRA_IO_PAD_INFO(DDR_GMI, ddr-gmi),
	TEGRA_IO_PAD_INFO(DDR_SDMMC2, ddr-sdmmc2),
	TEGRA_IO_PAD_INFO(DDR_SPI, ddr-spi),
	TEGRA_IO_PAD_INFO(HDMI_DP2, hdmi-dp2),
	TEGRA_IO_PAD_INFO(HDMI_DP3, hdmi-dp3),
};

struct tegra_io_pads_padcontrol {
	struct device *dev;
	struct padctrl_dev *pad_dev;
};

static struct tegra_io_pad_info *tegra_get_io_pad_info(int id)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(tegra_io_pads_info); ++i) {
		if (tegra_io_pads_info[i].id == id)
			break;
	}

	if (i == ARRAY_SIZE(tegra_io_pads_info))
		return NULL;

	return &tegra_io_pads_info[i];
}

static int tegra_io_pad_set_voltage(struct padctrl_dev *pad_dev,
				    int id, u32 voltage)
{
	struct tegra_io_pads_padcontrol *padctrl = padctrl_get_drvdata(pad_dev);
	struct tegra_io_pad_info *pad;
	u32 curr_volt;
	int ret;

	switch (voltage) {
	case 1200000:
	case 1800000:
	case 3300000:
		break;
	default:
		dev_err(padctrl->dev, "Pad voltage %u is not valid\n", voltage);
		return -EINVAL;
	}

	pad = tegra_get_io_pad_info(id);
	if (!pad) {
		dev_err(padctrl->dev, "Pad ID %d not identified\n", id);
		return -EINVAL;
	}

	ret = tegra_pmc_io_pad_get_voltage(pad->name);
	if (ret < 0)
		return ret;

	curr_volt = ret;
	if (voltage == curr_volt)
		return 0;

	ret = tegra_pmc_io_pad_set_voltage(pad->name, voltage);
	if (!ret)
		udelay(100);

	return ret;
}

static int tegra_io_pad_get_voltage(struct padctrl_dev *pad_dev,
				    int id, u32 *voltage)
{
	struct tegra_io_pads_padcontrol *padctrl = padctrl_get_drvdata(pad_dev);
	struct tegra_io_pad_info *pad;
	int ret;

	pad = tegra_get_io_pad_info(id);
	if (!pad) {
		dev_err(padctrl->dev, "Pad ID %d not identified\n", id);
		return -EINVAL;
	}

	ret = tegra_pmc_io_pad_get_voltage(pad->name);
	if (ret < 0)
		return ret;

	*voltage = ret;

	return 0;
}

static int tegra_io_pad_set_power(struct padctrl_dev *pad_dev,
				  int id, u32 enable)
{
	struct tegra_io_pads_padcontrol *padctrl = padctrl_get_drvdata(pad_dev);
	struct tegra_io_pad_info *pad;

	pad = tegra_get_io_pad_info(id);
	if (!pad) {
		dev_err(padctrl->dev, "Pad ID %d not identified\n", id);
		return -EINVAL;
	}

	if (enable)
		return tegra_pmc_io_pad_low_power_disable(pad->name);

	return tegra_pmc_io_pad_low_power_enable(pad->name);
}

static int tegra_io_pad_power_enable(struct padctrl_dev *pad_dev, int id)
{
	return tegra_io_pad_set_power(pad_dev, id, 1);
}

static int tegra_io_pad_power_disable(struct padctrl_dev *pad_dev,
					      int id)
{
	return tegra_io_pad_set_power(pad_dev, id, 0);
}

static struct padctrl_ops tegra_io_pad_padctrl_ops = {
	.set_voltage = &tegra_io_pad_set_voltage,
	.get_voltage = &tegra_io_pad_get_voltage,
	.power_enable = &tegra_io_pad_power_enable,
	.power_disable = &tegra_io_pad_power_disable,
};

static struct padctrl_desc tegra_io_pads_padctrl_desc = {
	.name = "tegra-pmc-padctrl",
	.ops = &tegra_io_pad_padctrl_ops,
};

int tegra_io_pads_padctrl_init(struct device *dev)
{
	struct tegra_io_pads_padcontrol *padctrl;
	struct padctrl_config config = { };
	struct device_node *pad_np;
	int ret;

	padctrl = devm_kzalloc(dev, sizeof(*padctrl), GFP_KERNEL);
	if (!padctrl)
		return -ENOMEM;

	config.of_node = dev->of_node;
	padctrl->dev = dev;
	padctrl->pad_dev = padctrl_register(dev, &tegra_io_pads_padctrl_desc,
					    &config);
	if (IS_ERR(padctrl->pad_dev)) {
		ret = PTR_ERR(padctrl->pad_dev);
		dev_err(dev, "Failed to register padcontrol: %d\n", ret);
		return ret;
	}

	padctrl_set_drvdata(padctrl->pad_dev, padctrl);

	/* Clear all DPD */
	if (of_property_read_bool(dev->of_node, "clear-all-io-pads-dpd"))
		tegra_pmc_io_dpd_clear();

	pad_np = of_get_child_by_name(dev->of_node, "io-pad-defaults");
	if (pad_np) {
		dev_warn(dev, "WARNING: IO pad defaults is supported via pinctrl\n");
		WARN_ON(1);
		return 0;
	}

	dev_info(dev, "IO padctrl driver initialized\n");

	return 0;
}
