/*
 * Copyright (c) 2015-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * Authors:
 *      VenkataJagadish.p	<vjagadish@nvidia.com>
 *      Naveen Kumar Arepalli	<naveenk@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/time.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <soc/tegra/chip-id.h>
#include <linux/reset.h>
#include <soc/tegra/pmc.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <linux/jiffies.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#endif

#include "ufshcd-pltfrm.h"
#include "ufshcd.h"
#include "unipro.h"
#include "ufs-tegra.h"
#include "ufshci.h"
#include "ufs-provision.h"

#ifdef CONFIG_DEBUG_FS
static int ufs_tegra_show_configuration(struct seq_file *s, void *data)
{
	struct ufs_hba *hba = s->private;
	u32 major_version;
	u32 minor_version;
	u32 rx_gear;
	u32 tx_gear;
	const char *freq_series = "";
	struct ufs_pa_layer_attr *configured_params;

	configured_params = &hba->pwr_info;
	rx_gear = configured_params->gear_rx;
	tx_gear = configured_params->gear_tx;

	seq_puts(s, "UFS Configuration:\n");
	if ((hba->ufs_version == UFSHCI_VERSION_10) ||
			(hba->ufs_version == UFSHCI_VERSION_11)) {
		major_version = hba->ufs_version >> 16;
		minor_version = ((hba->ufs_version) & 0xffff);
	} else {
		major_version = ((hba->ufs_version) & 0xff00) >> 8;
		minor_version = ((hba->ufs_version) & 0xf0) >> 4;
	}

	seq_puts(s, "\n");
	seq_puts(s, "UFSHCI Version Information:\n");
	seq_printf(s, "Major Version Number: %u\n", major_version);
	seq_printf(s, "Minor Version Number: %u\n", minor_version);

	seq_puts(s, "\n");
	seq_puts(s, "Number of UTP Transfer Request Slots:\n");
	seq_printf(s, "NUTRS: %u\n", hba->nutrs);

	seq_puts(s, "\n");
	seq_puts(s, "UTP Task Management Request Slots:\n");
	seq_printf(s, "NUTMRS: %u\n", hba->nutmrs);

	seq_puts(s, "\n");
	seq_puts(s, "UTP Power Info:\n");
	if (configured_params->hs_rate) {
		if (configured_params->hs_rate == PA_HS_MODE_A)
			freq_series = "RATE_A";
		else if (configured_params->hs_rate == PA_HS_MODE_B)
			freq_series = "RATE_B";
		seq_printf(s,
			"HS Mode RX_Gear:gear_%u TX_Gear:gear_%u %s series\n",
				rx_gear, tx_gear, freq_series);
		seq_printf(s,
			"LANE_RX:%u LANE_TX:%u PWR_RX:%u PWR_TX:%u\n",
				configured_params->lane_rx,
				configured_params->lane_tx,
				configured_params->pwr_rx,
				configured_params->pwr_tx);
	} else {
		seq_printf(s,
			"PWM Mode RX_Gear:gear_%u TX_Gear:gear_%u\n",
				rx_gear, tx_gear);
	}

	return 0;
}

static int ufs_tegra_open_configuration(struct inode *inode, struct file *file)
{
	return single_open(file, ufs_tegra_show_configuration, inode->i_private);
}

static const struct file_operations ufs_tegra_debugfs_ops = {
	.open           = ufs_tegra_open_configuration,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

void ufs_tegra_init_debugfs(struct ufs_hba *hba)
{
	struct dentry *device_root;
	struct ufs_tegra_host *ufs_tegra = hba->priv;

	device_root = debugfs_create_dir(dev_name(hba->dev), NULL);
	debugfs_create_file("configuration", S_IFREG | S_IRUGO,
			device_root, hba, &ufs_tegra_debugfs_ops);
	if (ufs_tegra->enable_ufs_provisioning)
		debugfs_provision_init(hba, device_root);
}
#endif

static int ufs_schedule_delayed_work(struct delayed_work *work,
				     unsigned long delay)
{
	/*
	 * We use the system_freezable_wq, because of two reasons.
	 * First, it allows several works (not the same work item) to be
	 * executed simultaneously. Second, the queue becomes frozen when
	 * userspace becomes frozen during system PM.
	 */
	return queue_delayed_work(system_freezable_wq, work, delay);
}

static bool ufs_tegra_get_cd(struct gpio_desc *cd_gpio_desc)
{
	/* If card present then gpio value low, else high */
	return (gpiod_get_value_cansleep(cd_gpio_desc) == 0);
}

static irqreturn_t ufs_cd_gpio_isr(int irq, void *dev_id)
{
	struct ufs_tegra_host *ufs_tegra = dev_id;

	cancel_delayed_work(&ufs_tegra->detect);
	ufs_schedule_delayed_work(&ufs_tegra->detect, msecs_to_jiffies(200));

	return IRQ_HANDLED;

}

void ufs_rescan(struct work_struct *work)
{
	struct ufs_tegra_host *ufs_tegra =
		container_of(work, struct ufs_tegra_host, detect.work);

	if(ufs_tegra->hba->card_present != ufs_tegra_get_cd(ufs_tegra->cd_gpio_desc)) {
		ufs_tegra->hba->card_present =
		                ufs_tegra_get_cd(ufs_tegra->cd_gpio_desc);
		ufshcd_rescan(ufs_tegra->hba);
	}
}

/**
 * ufs_tegra_cfg_vendor_registers
 * @hba: host controller instance
 */
static void ufs_tegra_cfg_vendor_registers(struct ufs_hba *hba)
{
	ufshcd_writel(hba, UFS_VNDR_HCLKDIV_1US_TICK, REG_UFS_VNDR_HCLKDIV);
}

static int ufs_tegra_host_clk_get(struct device *dev,
		const char *name, struct clk **clk_out)
{
	struct clk *clk;
	int err = 0;

	clk = devm_clk_get(dev, name);
	if (IS_ERR(clk)) {
		err = PTR_ERR(clk);
		dev_err(dev, "%s: failed to get %s err %d",
				__func__, name, err);
	} else {
		*clk_out = clk;
	}

	return err;
}

static int ufs_tegra_host_clk_enable(struct device *dev,
		const char *name, struct clk *clk)
{
	int err = 0;

	err = clk_prepare_enable(clk);
	if (err)
		dev_err(dev, "%s: %s enable failed %d\n", __func__, name, err);

	return err;
}

/**
 * ufs_tegra_mphy_receiver_calibration
 * @ufs_tegra: ufs_tegra_host controller instance
 *
 * Implements MPhy Receiver Calibration Sequence
 *
 * Returns -1 if receiver calibration fails
 * and returns zero on success.
 */
static int ufs_tegra_mphy_receiver_calibration(struct ufs_tegra_host *ufs_tegra)
{
	struct device *dev = ufs_tegra->hba->dev;
	int timeout = 0;
	u32 mphy_rx_vendor2;

	if (!ufs_tegra->enable_mphy_rx_calib)
		return 0;

	if (ufs_tegra->x2config)
		mphy_update(ufs_tegra->mphy_l1_base,
				MPHY_RX_APB_VENDOR2_0_RX_CAL_EN,
				MPHY_RX_APB_VENDOR2_0);

	mphy_update(ufs_tegra->mphy_l0_base,
			MPHY_RX_APB_VENDOR2_0_RX_CAL_EN, MPHY_RX_APB_VENDOR2_0);

	if (ufs_tegra->x2config)
		mphy_update(ufs_tegra->mphy_l1_base,
				MPHY_GO_BIT, MPHY_RX_APB_VENDOR2_0);
	mphy_update(ufs_tegra->mphy_l0_base, MPHY_GO_BIT,
			MPHY_RX_APB_VENDOR2_0);
	timeout = 10;
	while (timeout--) {
		mdelay(1);

		mphy_rx_vendor2 = mphy_readl(ufs_tegra->mphy_l0_base,
				MPHY_RX_APB_VENDOR2_0);

		if (!(mphy_rx_vendor2 & MPHY_RX_APB_VENDOR2_0_RX_CAL_EN)) {
			dev_info(dev, "MPhy Receiver Calibration passed\n");
			break;
		}
	}
	if (timeout < 0) {
		dev_err(dev, "MPhy Receiver Calibration failed\n");
		return -1;
	}
	return 0;
}

static void ufs_tegra_disable_mphylane_clks(struct ufs_tegra_host *host)
{
	if (!host->is_lane_clks_enabled)
		return;

	clk_disable_unprepare(host->mphy_core_pll_fixed);
	clk_disable_unprepare(host->mphy_l0_tx_symb);
	clk_disable_unprepare(host->mphy_tx_1mhz_ref);
	clk_disable_unprepare(host->mphy_l0_rx_ana);
	clk_disable_unprepare(host->mphy_l0_rx_symb);
	clk_disable_unprepare(host->mphy_l0_tx_ls_3xbit);
	clk_disable_unprepare(host->mphy_l0_rx_ls_bit);

	if (host->x2config)
		clk_disable_unprepare(host->mphy_l1_rx_ana);

	host->is_lane_clks_enabled = false;
}

static int ufs_tegra_enable_mphylane_clks(struct ufs_tegra_host *host)
{
	int err = 0;
	struct device *dev = host->hba->dev;

	if (host->is_lane_clks_enabled)
		return 0;

	err = ufs_tegra_host_clk_enable(dev, "mphy_core_pll_fixed",
		host->mphy_core_pll_fixed);
	if (err)
		goto out;

	err = ufs_tegra_host_clk_enable(dev, "mphy_l0_tx_symb",
		host->mphy_l0_tx_symb);
	if (err)
		goto disable_l0_tx_symb;

	err = ufs_tegra_host_clk_enable(dev, "mphy_tx_1mhz_ref",
		host->mphy_tx_1mhz_ref);
	if (err)
		goto disable_tx_1mhz_ref;

	err = ufs_tegra_host_clk_enable(dev, "mphy_l0_rx_ana",
		host->mphy_l0_rx_ana);
	if (err)
		goto disable_l0_rx_ana;

	err = ufs_tegra_host_clk_enable(dev, "mphy_l0_rx_symb",
		host->mphy_l0_rx_symb);
	if (err)
		goto disable_l0_rx_symb;

	err = ufs_tegra_host_clk_enable(dev, "mphy_l0_tx_ls_3xbit",
		host->mphy_l0_tx_ls_3xbit);
	if (err)
		goto disable_l0_tx_ls_3xbit;

	err = ufs_tegra_host_clk_enable(dev, "mphy_l0_rx_ls_bit",
		host->mphy_l0_rx_ls_bit);
	if (err)
		goto disable_l0_rx_ls_bit;

	if (host->x2config) {
		err = ufs_tegra_host_clk_enable(dev, "mphy_l1_rx_ana",
			host->mphy_l1_rx_ana);
		if (err)
			goto disable_l1_rx_ana;
	}

	host->is_lane_clks_enabled = true;
	goto out;

disable_l1_rx_ana:
	clk_disable_unprepare(host->mphy_l0_rx_ls_bit);
disable_l0_rx_ls_bit:
	clk_disable_unprepare(host->mphy_l0_tx_ls_3xbit);
disable_l0_tx_ls_3xbit:
	clk_disable_unprepare(host->mphy_l0_rx_symb);
disable_l0_rx_symb:
	clk_disable_unprepare(host->mphy_l0_rx_ana);
disable_l0_rx_ana:
	clk_disable_unprepare(host->mphy_tx_1mhz_ref);
disable_tx_1mhz_ref:
	clk_disable_unprepare(host->mphy_l0_tx_symb);
disable_l0_tx_symb:
	clk_disable_unprepare(host->mphy_core_pll_fixed);
out:
	return err;
}

static int ufs_tegra_init_mphy_lane_clks(struct ufs_tegra_host *host)
{
	int err = 0;
	struct device *dev = host->hba->dev;

	err = ufs_tegra_host_clk_get(dev,
			"mphy_core_pll_fixed", &host->mphy_core_pll_fixed);
	if (err)
		goto out;

	err = ufs_tegra_host_clk_get(dev,
			"mphy_l0_tx_symb", &host->mphy_l0_tx_symb);
	if (err)
		goto out;

	err = ufs_tegra_host_clk_get(dev, "mphy_tx_1mhz_ref",
		&host->mphy_tx_1mhz_ref);
	if (err)
		goto out;

	err = ufs_tegra_host_clk_get(dev, "mphy_l0_rx_ana",
		&host->mphy_l0_rx_ana);
	if (err)
		goto out;

	err = ufs_tegra_host_clk_get(dev, "mphy_l0_rx_symb",
		&host->mphy_l0_rx_symb);
	if (err)
		goto out;

	err = ufs_tegra_host_clk_get(dev, "mphy_l0_tx_ls_3xbit",
		&host->mphy_l0_tx_ls_3xbit);
	if (err)
		goto out;

	err = ufs_tegra_host_clk_get(dev, "mphy_l0_rx_ls_bit",
		&host->mphy_l0_rx_ls_bit);
	if (err)
		goto out;

	err = ufs_tegra_host_clk_get(dev, "mphy_force_ls_mode",
		&host->mphy_force_ls_mode);
	if (err)
		goto out;

	if (host->x2config)
		err = ufs_tegra_host_clk_get(dev, "mphy_l1_rx_ana",
			&host->mphy_l1_rx_ana);
out:
	return err;
}

static int ufs_tegra_enable_ufs_uphy_pll3(struct ufs_tegra_host *ufs_tegra,
			bool is_rate_b)
{
	int err = 0;
	struct device *dev = ufs_tegra->hba->dev;

	if (!ufs_tegra->configure_uphy_pll3)
		return 0;

	err = ufs_tegra_host_clk_enable(dev, "uphy_pll3",
		ufs_tegra->ufs_uphy_pll3);
	if (err)
		return err;

	if (is_rate_b) {
		if (ufs_tegra->ufs_uphy_pll3)
			err = clk_set_rate(ufs_tegra->ufs_uphy_pll3,
				UFS_CLK_UPHY_PLL3_RATEB);
	} else {
		if (ufs_tegra->ufs_uphy_pll3)
			err = clk_set_rate(ufs_tegra->ufs_uphy_pll3,
					UFS_CLK_UPHY_PLL3_RATEA);
	}
	if (err)
		dev_err(dev, "%s: failed to set ufs_uphy_pll3 freq err %d",
				__func__, err);
	return err;
}

static int ufs_tegra_init_uphy_pll3(struct ufs_tegra_host *ufs_tegra)
{
	int err = 0;
	struct device *dev = ufs_tegra->hba->dev;

	if (!ufs_tegra->configure_uphy_pll3)
		return 0;

	err = ufs_tegra_host_clk_get(dev,
		"uphy_pll3", &ufs_tegra->ufs_uphy_pll3);
	return err;
}

static int ufs_tegra_init_ufs_clks(struct ufs_tegra_host *ufs_tegra)
{
	int err = 0;
	struct device *dev = ufs_tegra->hba->dev;

	err = ufs_tegra_host_clk_get(dev,
		"pll_p", &ufs_tegra->ufshc_parent);
	if (err)
		goto out;
	err = ufs_tegra_host_clk_get(dev,
		"ufshc", &ufs_tegra->ufshc_clk);
	if (err)
		goto out;

	err = ufs_tegra_host_clk_get(dev,
		"clk_m", &ufs_tegra->ufsdev_parent);
	if (err)
		goto out;
	err = ufs_tegra_host_clk_get(dev,
		"ufsdev_ref", &ufs_tegra->ufsdev_ref_clk);
	if (err)
		goto out;

out:
	return err;
}

static int ufs_tegra_enable_ufs_clks(struct ufs_tegra_host *ufs_tegra)
{
	struct device *dev = ufs_tegra->hba->dev;
	int err = 0;

	err = ufs_tegra_host_clk_enable(dev, "ufshc",
		ufs_tegra->ufshc_clk);
	if (err)
		goto out;
	err = clk_set_parent(ufs_tegra->ufshc_clk,
				ufs_tegra->ufshc_parent);
	if (err)
		goto out;
	err = clk_set_rate(ufs_tegra->ufshc_clk, UFSHC_CLK_FREQ);
	if (err)
		goto out;

	/* clk_m is the parent for ufsdev_ref
         * Frequency is 19.2 MHz.
	 */
	err = ufs_tegra_host_clk_enable(dev, "ufsdev_ref",
		ufs_tegra->ufsdev_ref_clk);
	if (err)
		goto disable_ufshc;

	ufs_tegra->hba->clk_gating.state = CLKS_ON;

	return err;

disable_ufshc:
	clk_disable_unprepare(ufs_tegra->ufshc_clk);
out:
	return err;
}

static void ufs_tegra_disable_ufs_clks(struct ufs_tegra_host *ufs_tegra)
{
	if (ufs_tegra->hba->clk_gating.state == CLKS_OFF)
		return;

	clk_disable_unprepare(ufs_tegra->ufshc_clk);
	clk_disable_unprepare(ufs_tegra->ufsdev_ref_clk);

	ufs_tegra->hba->clk_gating.state = CLKS_OFF;
}

static int ufs_tegra_set_ufs_mphy_clocks(struct ufs_hba *hba, bool enable)
{
	struct ufs_tegra_host *ufs_tegra = hba->priv;
	int ret = 0;

	if (enable) {
		ret = ufs_tegra_enable_ufs_clks(ufs_tegra);
		if (ret) {
			dev_err(hba->dev, "failed to set ufs clks, ret: %d\n",
				ret);
			goto out;
		}

		ret = ufs_tegra_enable_mphylane_clks(ufs_tegra);
		if (ret) {
			dev_err(hba->dev, "failed to set mphy clks, ret: %d\n",
				ret);
			goto out;
		}
	} else {
		ufs_tegra_disable_mphylane_clks(ufs_tegra);
		ufs_tegra_disable_ufs_clks(ufs_tegra);
	}
out:
	return ret;
}

static int ufs_tegra_ufs_reset_init(struct ufs_tegra_host *ufs_tegra)
{
	struct device *dev = ufs_tegra->hba->dev;
	int ret = 0;

	ufs_tegra->ufs_rst = devm_reset_control_get(dev, "ufs-rst");
	if (IS_ERR(ufs_tegra->ufs_rst)) {
		ret = PTR_ERR(ufs_tegra->ufs_rst);
		dev_err(dev,
			"Reset control for ufs-rst not found: %d\n", ret);
	}
	ufs_tegra->ufs_axi_m_rst = devm_reset_control_get(dev, "ufs-axi-m-rst");
	if (IS_ERR(ufs_tegra->ufs_axi_m_rst)) {
		ret = PTR_ERR(ufs_tegra->ufs_axi_m_rst);
		dev_err(dev,
			"Reset control for ufs-axi-m-rst not found: %d\n", ret);
	}
	ufs_tegra->ufshc_lp_rst = devm_reset_control_get(dev, "ufshc-lp-rst");
	if (IS_ERR(ufs_tegra->ufshc_lp_rst)) {
		ret = PTR_ERR(ufs_tegra->ufshc_lp_rst);
		dev_err(dev,
			"Reset control for ufshc-lp-rst not found: %d\n", ret);
	}
	return ret;
}

static void ufs_tegra_ufs_deassert_reset(struct ufs_tegra_host *ufs_tegra)
{
	reset_control_deassert(ufs_tegra->ufs_rst);
	reset_control_deassert(ufs_tegra->ufs_axi_m_rst);
	reset_control_deassert(ufs_tegra->ufshc_lp_rst);
}

static int ufs_tegra_mphy_reset_init(struct ufs_tegra_host *ufs_tegra)
{
	struct device *dev = ufs_tegra->hba->dev;
	int ret = 0;

	ufs_tegra->mphy_l0_rx_rst =
				devm_reset_control_get(dev, "mphy-l0-rx-rst");
	if (IS_ERR(ufs_tegra->mphy_l0_rx_rst)) {
		ret = PTR_ERR(ufs_tegra->mphy_l0_rx_rst);
		dev_err(dev,
			"Reset control for mphy-l0-rx-rst not found: %d\n",
									ret);
	}

	ufs_tegra->mphy_l0_tx_rst =
				devm_reset_control_get(dev, "mphy-l0-tx-rst");
	if (IS_ERR(ufs_tegra->mphy_l0_tx_rst)) {
		ret = PTR_ERR(ufs_tegra->mphy_l0_tx_rst);
		dev_err(dev,
			"Reset control for mphy-l0-tx-rst not found: %d\n",
									ret);
	}

	ufs_tegra->mphy_clk_ctl_rst =
				devm_reset_control_get(dev, "mphy-clk-ctl-rst");
	if (IS_ERR(ufs_tegra->mphy_clk_ctl_rst)) {
		ret = PTR_ERR(ufs_tegra->mphy_clk_ctl_rst);
		dev_err(dev,
			"Reset control for mphy-clk-ctl-rst not found: %d\n",
									ret);
	}

	if (ufs_tegra->x2config) {
		ufs_tegra->mphy_l1_rx_rst =
				devm_reset_control_get(dev, "mphy-l1-rx-rst");
		if (IS_ERR(ufs_tegra->mphy_l1_rx_rst)) {
			ret = PTR_ERR(ufs_tegra->mphy_l1_rx_rst);
			dev_err(dev,
			"Reset control for mphy-l1-rx-rst not found: %d\n",
									ret);
		}

		ufs_tegra->mphy_l1_tx_rst =
				devm_reset_control_get(dev, "mphy-l1-tx-rst");
		if (IS_ERR(ufs_tegra->mphy_l1_tx_rst)) {
			ret = PTR_ERR(ufs_tegra->mphy_l1_tx_rst);
			dev_err(dev,
			"Reset control for mphy_l1_tx_rst not found: %d\n",
									ret);
		}
	}

	return ret;
}

static void ufs_tegra_mphy_assert_reset(struct ufs_tegra_host *ufs_tegra)
{
	reset_control_assert(ufs_tegra->mphy_l0_rx_rst);
	reset_control_assert(ufs_tegra->mphy_l0_tx_rst);
	reset_control_assert(ufs_tegra->mphy_clk_ctl_rst);
	if (ufs_tegra->x2config) {
		reset_control_assert(ufs_tegra->mphy_l1_rx_rst);
		reset_control_assert(ufs_tegra->mphy_l1_tx_rst);
	}
}

static void ufs_tegra_mphy_deassert_reset(struct ufs_tegra_host *ufs_tegra)
{
	reset_control_deassert(ufs_tegra->mphy_l0_rx_rst);
	reset_control_deassert(ufs_tegra->mphy_l0_tx_rst);
	reset_control_deassert(ufs_tegra->mphy_clk_ctl_rst);
	if (ufs_tegra->x2config) {
		reset_control_deassert(ufs_tegra->mphy_l1_rx_rst);
		reset_control_deassert(ufs_tegra->mphy_l1_tx_rst);
	}
}

void ufs_tegra_disable_mphy_slcg(struct ufs_tegra_host *ufs_tegra)
{
	u32 val = 0;

	val = (MPHY_TX_CLK_EN_SYMB | MPHY_TX_CLK_EN_SLOW |
			MPHY_TX_CLK_EN_FIXED | MPHY_TX_CLK_EN_3X);
	mphy_writel(ufs_tegra->mphy_l0_base, val, MPHY_TX_APB_TX_CG_OVR0_0);
	mphy_writel(ufs_tegra->mphy_l0_base, MPHY_GO_BIT,
						MPHY_TX_APB_TX_VENDOR0_0);

	if (ufs_tegra->x2config) {
		mphy_writel(ufs_tegra->mphy_l1_base, val,
						MPHY_TX_APB_TX_CG_OVR0_0);
		mphy_writel(ufs_tegra->mphy_l1_base, MPHY_GO_BIT,
						MPHY_TX_APB_TX_VENDOR0_0);
	}

}


static void ufs_tegra_mphy_rx_sync_capabity(struct ufs_tegra_host *ufs_tegra)
{
	u32 val_88_8b = 0;
	u32 val_94_97 = 0;
	u32 val_8c_8f = 0;
	u32 val_98_9b = 0;

	/* MPHY RX sync lengths capability changes */

	/*Update HS_G1 Sync Length MPHY_RX_APB_CAPABILITY_88_8B_0*/
	val_88_8b = mphy_readl(ufs_tegra->mphy_l0_base,
			MPHY_RX_APB_CAPABILITY_88_8B_0);
	val_88_8b &= ~RX_HS_G1_SYNC_LENGTH_CAPABILITY(~0);
	val_88_8b |= RX_HS_G1_SYNC_LENGTH_CAPABILITY(RX_HS_SYNC_LENGTH);

	/*Update HS_G2&G3 Sync Length MPHY_RX_APB_CAPABILITY_94_97_0*/
	val_94_97 = mphy_readl(ufs_tegra->mphy_l0_base,
			MPHY_RX_APB_CAPABILITY_94_97_0);
	val_94_97 &= ~RX_HS_G2_SYNC_LENGTH_CAPABILITY(~0);
	val_94_97 |= RX_HS_G2_SYNC_LENGTH_CAPABILITY(RX_HS_SYNC_LENGTH);
	val_94_97 &= ~RX_HS_G3_SYNC_LENGTH_CAPABILITY(~0);
	val_94_97 |= RX_HS_G3_SYNC_LENGTH_CAPABILITY(RX_HS_SYNC_LENGTH);

	/* MPHY RX TActivate_capability changes */

	/* Update MPHY_RX_APB_CAPABILITY_8C_8F_0 */
	val_8c_8f = mphy_readl(ufs_tegra->mphy_l0_base,
			MPHY_RX_APB_CAPABILITY_8C_8F_0);
	val_8c_8f &= ~RX_MIN_ACTIVATETIME_CAP(~0);
	val_8c_8f |= RX_MIN_ACTIVATETIME_CAP(RX_MIN_ACTIVATETIME);

	/* Update MPHY_RX_APB_CAPABILITY_98_9B_0 */
	val_98_9b = mphy_readl(ufs_tegra->mphy_l0_base,
			MPHY_RX_APB_CAPABILITY_98_9B_0);
	val_98_9b &= ~RX_ADVANCED_FINE_GRANULARITY(~0);
	val_98_9b &= ~RX_ADVANCED_GRANULARITY(~0);
	val_98_9b &= ~RX_ADVANCED_MIN_ACTIVATETIME(~0);
	val_98_9b |= RX_ADVANCED_MIN_ACTIVATETIME(RX_ADVANCED_MIN_AT);

	mphy_writel(ufs_tegra->mphy_l0_base, val_88_8b,
			MPHY_RX_APB_CAPABILITY_88_8B_0);
	mphy_writel(ufs_tegra->mphy_l0_base, val_94_97,
			MPHY_RX_APB_CAPABILITY_94_97_0);
	mphy_writel(ufs_tegra->mphy_l0_base, val_8c_8f,
			MPHY_RX_APB_CAPABILITY_8C_8F_0);
	mphy_writel(ufs_tegra->mphy_l0_base, val_98_9b,
			MPHY_RX_APB_CAPABILITY_98_9B_0);
	mphy_update(ufs_tegra->mphy_l0_base,
				MPHY_GO_BIT, MPHY_RX_APB_VENDOR2_0);

	if (ufs_tegra->x2config) {
		mphy_writel(ufs_tegra->mphy_l1_base, val_88_8b,
			MPHY_RX_APB_CAPABILITY_88_8B_0);
		mphy_writel(ufs_tegra->mphy_l1_base, val_94_97,
			MPHY_RX_APB_CAPABILITY_94_97_0);
		mphy_writel(ufs_tegra->mphy_l1_base, val_8c_8f,
			MPHY_RX_APB_CAPABILITY_8C_8F_0);
		mphy_writel(ufs_tegra->mphy_l1_base, val_98_9b,
			MPHY_RX_APB_CAPABILITY_98_9B_0);
		/* set gobit */
		mphy_update(ufs_tegra->mphy_l1_base,
				MPHY_GO_BIT, MPHY_RX_APB_VENDOR2_0);
	}
}

void ufs_tegra_mphy_tx_advgran(struct ufs_tegra_host *ufs_tegra)
{
	u32 val = 0;

	val = (TX_ADVANCED_GRANULARITY | TX_ADVANCED_GRANULARITY_SETTINGS);
	mphy_update(ufs_tegra->mphy_l0_base, val,
					MPHY_TX_APB_TX_ATTRIBUTE_34_37_0);
	mphy_writel(ufs_tegra->mphy_l0_base, MPHY_GO_BIT,
						MPHY_TX_APB_TX_VENDOR0_0);

	if (ufs_tegra->x2config) {
		mphy_update(ufs_tegra->mphy_l1_base, val,
					MPHY_TX_APB_TX_ATTRIBUTE_34_37_0);
		mphy_writel(ufs_tegra->mphy_l1_base, MPHY_GO_BIT,
						MPHY_TX_APB_TX_VENDOR0_0);
	}
}


void ufs_tegra_mphy_rx_advgran(struct ufs_tegra_host *ufs_tegra)
{
	u32 val = 0;

	val = mphy_readl(ufs_tegra->mphy_l0_base, MPHY_RX_APB_CAPABILITY_98_9B_0);
	val &= ~RX_ADVANCED_GRANULARITY(~0);
	val |= RX_ADVANCED_GRANULARITY(0x1);

	val &= ~RX_ADVANCED_MIN_ACTIVATETIME(~0);
	val |= RX_ADVANCED_MIN_ACTIVATETIME(0x8);

	mphy_writel(ufs_tegra->mphy_l0_base, val,
					MPHY_RX_APB_CAPABILITY_98_9B_0);
	mphy_update(ufs_tegra->mphy_l0_base, MPHY_GO_BIT,
			MPHY_RX_APB_VENDOR2_0);

	if (ufs_tegra->x2config) {
		val = mphy_readl(ufs_tegra->mphy_l1_base,
				MPHY_RX_APB_CAPABILITY_98_9B_0);
		val &= ~RX_ADVANCED_GRANULARITY(~0);
		val |= RX_ADVANCED_GRANULARITY(0x1);

		val &= ~RX_ADVANCED_MIN_ACTIVATETIME(~0);
		val |= RX_ADVANCED_MIN_ACTIVATETIME(0x8);

		mphy_writel(ufs_tegra->mphy_l1_base, val,
					MPHY_RX_APB_CAPABILITY_98_9B_0);
		mphy_update(ufs_tegra->mphy_l1_base, MPHY_GO_BIT,
			MPHY_RX_APB_VENDOR2_0);
	}
}

void ufs_tegra_ufs_aux_ref_clk_enable(struct ufs_tegra_host *ufs_tegra)
{
	ufs_aux_update(ufs_tegra->ufs_aux_base, UFSHC_DEV_CLK_EN,
						UFSHC_AUX_UFSHC_DEV_CTRL_0);
}

void ufs_tegra_ufs_aux_ref_clk_disable(struct ufs_tegra_host *ufs_tegra)
{
	ufs_aux_clear_bits(ufs_tegra->ufs_aux_base, UFSHC_DEV_CLK_EN,
						UFSHC_AUX_UFSHC_DEV_CTRL_0);
}

void ufs_tegra_aux_reset_enable(struct ufs_tegra_host *ufs_tegra)
{
	ufs_aux_clear_bits(ufs_tegra->ufs_aux_base,
					UFSHC_DEV_RESET,
					UFSHC_AUX_UFSHC_DEV_CTRL_0);
}

void ufs_tegra_ufs_aux_prog(struct ufs_tegra_host *ufs_tegra)
{

	/*
	 * Release the reset to UFS device on pin ufs_rst_n
	 */

	if (ufs_tegra->ufshc_state != UFSHC_SUSPEND)
		ufs_aux_update(ufs_tegra->ufs_aux_base, UFSHC_DEV_RESET,
						UFSHC_AUX_UFSHC_DEV_CTRL_0);


	if (ufs_tegra->ufshc_state == UFSHC_SUSPEND) {
		/*
		 * Disable reference clock to Device
		 */
		ufs_tegra_ufs_aux_ref_clk_disable(ufs_tegra);

	} else {
		/*
		 * Enable reference clock to Device
		 */
		ufs_tegra_ufs_aux_ref_clk_enable(ufs_tegra);
	}
}

static void ufs_tegra_context_save(struct ufs_tegra_host *ufs_tegra)
{
	u32 reg_len = 0;
	u32 len = 0;
	u32 *mphy_context_save = ufs_tegra->mphy_context;

	reg_len = ARRAY_SIZE(mphy_rx_apb);
	/*
	 * Save mphy_rx_apb lane0 and lane1 context
	 */
	ufs_save_regs(ufs_tegra->mphy_l0_base, mphy_context_save,
							mphy_rx_apb, reg_len);
	len += reg_len;

	if (ufs_tegra->x2config) {
		ufs_save_regs(ufs_tegra->mphy_l1_base, mphy_context_save + len,
							mphy_rx_apb, reg_len);
		len += reg_len;
	}

	reg_len = ARRAY_SIZE(mphy_tx_apb);
	/*
	 * Save mphy_tx_apb lane0 and lane1 context
	 */
	ufs_save_regs(ufs_tegra->mphy_l0_base, mphy_context_save + len,
							mphy_tx_apb, reg_len);
	len += reg_len;
	if (ufs_tegra->x2config)
		ufs_save_regs(ufs_tegra->mphy_l1_base,
			mphy_context_save + len, mphy_tx_apb, reg_len);
}

static void ufs_tegra_context_restore(struct ufs_tegra_host *ufs_tegra)
{
	u32 reg_len = 0;
	u32 len = 0;
	u32 *mphy_context_restore = ufs_tegra->mphy_context;

	reg_len = ARRAY_SIZE(mphy_rx_apb);
	/*
	 * Restore mphy_rx_apb lane0 and lane1 context
	 */
	ufs_restore_regs(ufs_tegra->mphy_l0_base, mphy_context_restore,
							mphy_rx_apb, reg_len);
	mphy_update(ufs_tegra->mphy_l0_base, MPHY_GO_BIT,
				MPHY_RX_APB_VENDOR2_0);

	len += reg_len;
	if (ufs_tegra->x2config) {
		ufs_restore_regs(ufs_tegra->mphy_l1_base,
			mphy_context_restore + len, mphy_rx_apb, reg_len);
		mphy_update(ufs_tegra->mphy_l1_base, MPHY_GO_BIT,
				MPHY_RX_APB_VENDOR2_0);
		len += reg_len;
	}

	reg_len = ARRAY_SIZE(mphy_tx_apb);
	/*
	 * Restore mphy_tx_apb lane0 and lane1 context
	 */
	ufs_restore_regs(ufs_tegra->mphy_l0_base, mphy_context_restore + len,
							mphy_tx_apb, reg_len);
	mphy_writel(ufs_tegra->mphy_l0_base, MPHY_GO_BIT,
			MPHY_TX_APB_TX_VENDOR0_0);

	len += reg_len;
	if (ufs_tegra->x2config) {
		ufs_restore_regs(ufs_tegra->mphy_l1_base,
			mphy_context_restore + len, mphy_tx_apb, reg_len);
		mphy_writel(ufs_tegra->mphy_l1_base, MPHY_GO_BIT,
				MPHY_TX_APB_TX_VENDOR0_0);
	}
}

static int ufs_tegra_suspend(struct ufs_hba *hba, enum ufs_pm_op pm_op)
{
	struct ufs_tegra_host *ufs_tegra = hba->priv;
	struct device *dev = hba->dev;
	u32 val;
	int ret = 0;
	int timeout = 500;
	bool is_ufs_lp_pwr_gated = false;

	if(!hba->card_present)
		return 0;

	if (pm_op != UFS_SYSTEM_PM)
		return 0;

	ufs_tegra->ufshc_state = UFSHC_SUSPEND;
	/*
	 * Enable DPD for UFS
	 */
	if (ufs_tegra->ufs_pinctrl && !IS_ERR_OR_NULL(ufs_tegra->dpd_enable)) {
		ret = pinctrl_select_state(ufs_tegra->ufs_pinctrl,
					   ufs_tegra->dpd_enable);
		if (ret)
			dev_err(dev, "pinctrl power down fail %d\n", ret);

	}

	do {
		udelay(100);
		val = ufs_aux_readl(ufs_tegra->ufs_aux_base,
				UFSHC_AUX_UFSHC_STATUS_0);
		if (val & UFSHC_HIBERNATE_STATUS) {
			is_ufs_lp_pwr_gated = true;
			break;
		}
		timeout--;
	} while (timeout > 0);

	if (timeout <= 0) {
		dev_err(dev, "UFSHC_AUX_UFSHC_STATUS_0 = %x\n", val);
		return -ETIMEDOUT;
	}

	if (is_ufs_lp_pwr_gated) {
		/*
		 * Save all armphy_rx_apb and armphy_tx_apb registers
		 */
		ufs_tegra_context_save(ufs_tegra);
		reset_control_assert(ufs_tegra->ufshc_lp_rst);
	}

	/* Enable wake irq at end of suspend */
	if (device_may_wakeup(dev)) {
		ret = enable_irq_wake(ufs_tegra->cd_irq);
		if (ret) {
			dev_err(dev, "Failed to enable wake irq %u, ret: %d\n",
				ufs_tegra->cd_irq, ret);
			ufs_tegra->wake_enable_failed = true;
		} else {
			ufs_tegra->wake_enable_failed = false;
		}
	}

	/*
	 * Disable ufs, mphy tx/rx lane clocks if they are on
	 * and assert the reset
	 */

	ufs_tegra_disable_mphylane_clks(ufs_tegra);
	ufs_tegra_mphy_assert_reset(ufs_tegra);
	ufs_tegra_disable_ufs_clks(ufs_tegra);
	reset_control_assert(ufs_tegra->ufs_axi_m_rst);

	return ret;
}


static int ufs_tegra_resume(struct ufs_hba *hba, enum ufs_pm_op pm_op)
{
	struct ufs_tegra_host *ufs_tegra = hba->priv;
	struct device *dev = hba->dev;
	int ret = 0;

	if(!hba->card_present)
		return 0;

	if (pm_op != UFS_SYSTEM_PM)
		return 0;

	if (device_may_wakeup(dev) && !ufs_tegra->wake_enable_failed) {
		ret = disable_irq_wake(ufs_tegra->cd_irq);
		if (ret)
			dev_err(dev, "Failed to disable wakeirq %u,err %d\n",
				ufs_tegra->cd_irq, ret);
	}

	ufs_tegra->ufshc_state = UFSHC_RESUME;

	ret = ufs_tegra_enable_ufs_clks(ufs_tegra);
	if (ret)
		return ret;

	ret = ufs_tegra_enable_mphylane_clks(ufs_tegra);
	if (ret)
		goto out_disable_ufs_clks;
	ufs_tegra_mphy_deassert_reset(ufs_tegra);
	ufs_tegra_ufs_deassert_reset(ufs_tegra);
	ufs_tegra_ufs_aux_prog(ufs_tegra);

	if (ufs_tegra->ufs_pinctrl &&
		!IS_ERR_OR_NULL(ufs_tegra->dpd_disable)) {
		ret = pinctrl_select_state(ufs_tegra->ufs_pinctrl,
					   ufs_tegra->dpd_disable);
		if (ret) {
			dev_err(dev, "pinctrl power up fail %d\n", ret);
			goto out_disable_mphylane_clks;
		}

	}

	ufs_tegra_context_restore(ufs_tegra);
	ufs_tegra_cfg_vendor_registers(hba);
	ret = ufs_tegra_mphy_receiver_calibration(ufs_tegra);
	if (ret < 0)
		goto out_disable_mphylane_clks;

	pm_runtime_disable(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	return ret;

out_disable_mphylane_clks:
	ufs_tegra_disable_mphylane_clks(ufs_tegra);
out_disable_ufs_clks:
	ufs_tegra_disable_ufs_clks(ufs_tegra);

	return ret;
}


static void ufs_tegra_hibern8_entry_notify(struct ufs_hba *hba)
{
	struct ufs_tegra_host *ufs_tegra = hba->priv;

	if (!(ufs_tegra->nvquirks & NVQUIRK_BROKEN_HIBERN8_ENTRY))
		return;

	ufs_tegra_context_save(ufs_tegra);

	reset_control_assert(ufs_tegra->mphy_l0_rx_rst);
	if (ufs_tegra->x2config)
		reset_control_assert(ufs_tegra->mphy_l1_rx_rst);

	udelay(50);

	reset_control_deassert(ufs_tegra->mphy_l0_rx_rst);
	if (ufs_tegra->x2config)
		reset_control_deassert(ufs_tegra->mphy_l1_rx_rst);

	ufs_tegra_context_restore(ufs_tegra);
}


static void ufs_tegra_print_power_mode_config(struct ufs_hba *hba,
			struct ufs_pa_layer_attr *configured_params)
{
	u32 rx_gear;
	u32 tx_gear;
	const char *freq_series = "";

	rx_gear = configured_params->gear_rx;
	tx_gear = configured_params->gear_tx;

	if (configured_params->hs_rate) {
		if (configured_params->hs_rate == PA_HS_MODE_A)
			freq_series = "RATE_A";
		else if (configured_params->hs_rate == PA_HS_MODE_B)
			freq_series = "RATE_B";
		dev_info(hba->dev,
			"HS Mode RX_Gear:gear_%u TX_Gear:gear_%u %s series\n",
				rx_gear, tx_gear, freq_series);
	} else {
		dev_info(hba->dev,
			"PWM Mode RX_Gear:gear_%u TX_Gear:gear_%u\n",
				rx_gear, tx_gear);
	}
}

static void ufs_tegra_scramble_enable(struct ufs_hba *hba)
{
	u32 pa_val;

	ufshcd_dme_get(hba, UIC_ARG_MIB(PA_PEERSCRAMBLING), &pa_val);
	if (pa_val & SCREN) {
		ufshcd_dme_get(hba, UIC_ARG_MIB(PA_SCRAMBLING), &pa_val);
		pa_val |= SCREN;
		ufshcd_dme_set(hba, UIC_ARG_MIB(PA_SCRAMBLING), pa_val);
	}
}

static int ufs_tegra_pwr_change_notify(struct ufs_hba *hba,
		enum ufs_notify_change_status status,
		struct ufs_pa_layer_attr *dev_max_params,
		struct ufs_pa_layer_attr *dev_req_params)
{
	struct ufs_tegra_host *ufs_tegra = hba->priv;
	u32 vs_save_config;
	int ret = 0;
	u32 pa_reg_check;

	if (!dev_req_params) {
		pr_err("%s: incoming dev_req_params is NULL\n", __func__);
		ret = -EINVAL;
		goto out;
	}

	switch (status) {
	case PRE_CHANGE:
		/* Update VS_DebugSaveConfigTime Tref */
		ufshcd_dme_get(hba, UIC_ARG_MIB(VS_DEBUGSAVECONFIGTIME),
			&vs_save_config);
		/* Update VS_DebugSaveConfigTime st_sct */
		vs_save_config &= ~SET_ST_SCT(~0);
		vs_save_config |= SET_ST_SCT(VS_DEBUGSAVECONFIGTIME_ST_SCT);
		/* Update VS_DebugSaveConfigTime Tref */
		vs_save_config &= ~SET_TREF(~0);
		vs_save_config |= SET_TREF(VS_DEBUGSAVECONFIGTIME_TREF);

		ufshcd_dme_set(hba, UIC_ARG_MIB(VS_DEBUGSAVECONFIGTIME),
				vs_save_config);

		memcpy(dev_req_params, dev_max_params,
			sizeof(struct ufs_pa_layer_attr));
		if (hba->init_prefetch_data.ref_clk_freq)
			ufs_tegra->enable_hs_mode = false;
		if ((ufs_tegra->enable_hs_mode) && (dev_max_params->hs_rate)) {
			if (ufs_tegra->max_hs_gear) {
				if (dev_max_params->gear_rx >
						ufs_tegra->max_hs_gear)
					dev_req_params->gear_rx =
						ufs_tegra->max_hs_gear;
				if (dev_max_params->gear_tx >
						ufs_tegra->max_hs_gear)
					dev_req_params->gear_tx =
						ufs_tegra->max_hs_gear;
			} else {
				dev_req_params->gear_rx = UFS_HS_G1;
				dev_req_params->gear_tx = UFS_HS_G1;
			}
			if (ufs_tegra->mask_fast_auto_mode) {
				dev_req_params->pwr_rx = FAST_MODE;
				dev_req_params->pwr_tx = FAST_MODE;
			}
			if (ufs_tegra->mask_hs_mode_b) {
				dev_req_params->hs_rate = PA_HS_MODE_A;
				ufs_tegra_enable_ufs_uphy_pll3(ufs_tegra,
								false);
			} else {
				ufs_tegra_enable_ufs_uphy_pll3(ufs_tegra, true);
			}
			if (ufs_tegra->enable_scramble)
				ufs_tegra_scramble_enable(hba);
		} else {
			if (ufs_tegra->max_pwm_gear) {
				ufshcd_dme_get(hba,
					UIC_ARG_MIB(PA_MAXRXPWMGEAR),
					&dev_req_params->gear_rx);
				ufshcd_dme_peer_get(hba,
					UIC_ARG_MIB(PA_MAXRXPWMGEAR),
					&dev_req_params->gear_tx);
				if (dev_req_params->gear_rx >
						ufs_tegra->max_pwm_gear)
					dev_req_params->gear_rx =
						ufs_tegra->max_pwm_gear;
				if (dev_req_params->gear_tx >
						ufs_tegra->max_pwm_gear)
					dev_req_params->gear_tx =
						ufs_tegra->max_pwm_gear;
			} else {
				dev_req_params->gear_rx = UFS_PWM_G1;
				dev_req_params->gear_tx = UFS_PWM_G1;
			}
			dev_req_params->pwr_rx = SLOWAUTO_MODE;
			dev_req_params->pwr_tx = SLOWAUTO_MODE;
			dev_req_params->hs_rate = 0;
		}
		memcpy(&hba->max_pwr_info.info, dev_req_params,
			sizeof(struct ufs_pa_layer_attr));
		break;
	case POST_CHANGE:
		ufs_tegra_print_power_mode_config(hba, dev_req_params);
		ufshcd_dme_get(hba, UIC_ARG_MIB(PA_SCRAMBLING), &pa_reg_check);
		if (pa_reg_check & SCREN)
			dev_info(hba->dev, "ufs scrambling feature enabled\n");
		break;
	default:
		break;
	}
out:
	return ret;
}

static void ufs_tegra_hce_disable_notify(struct ufs_hba *hba,
		enum ufs_notify_change_status status)
{
	struct ufs_tegra_host *ufs_tegra = hba->priv;

	switch (status) {
	case PRE_CHANGE:
		ufs_aux_update(ufs_tegra->ufs_aux_base,
					UFSHC_CG_SYS_CLK_OVR_ON,
					UFSHC_AUX_UFSHC_SW_EN_CLK_SLCG_0);
		break;
	default:
		break;
	}
}

static int ufs_tegra_hce_enable_notify(struct ufs_hba *hba,
		enum ufs_notify_change_status status)
{
	struct ufs_tegra_host *ufs_tegra = hba->priv;
	struct device *dev = ufs_tegra->hba->dev;
	int err = 0;

	switch (status) {
	case PRE_CHANGE:
		err = ufs_tegra_host_clk_enable(dev,
			"mphy_force_ls_mode",
			ufs_tegra->mphy_force_ls_mode);
		if (err)
			return err;
		udelay(500);
		ufs_aux_clear_bits(ufs_tegra->ufs_aux_base,
				UFSHC_DEV_RESET,
				UFSHC_AUX_UFSHC_DEV_CTRL_0);
		break;
	case POST_CHANGE:
		ufs_aux_clear_bits(ufs_tegra->ufs_aux_base,
					UFSHC_CG_SYS_CLK_OVR_ON,
					UFSHC_AUX_UFSHC_SW_EN_CLK_SLCG_0);
		ufs_tegra_ufs_aux_prog(ufs_tegra);
		ufs_tegra_cfg_vendor_registers(hba);
		clk_disable_unprepare(ufs_tegra->mphy_force_ls_mode);
		break;
	default:
		break;
	}
	return err;
}

static void ufs_tegra_unipro_post_linkup(struct ufs_hba *hba)
{
	struct ufs_tegra_host *ufs_tegra = hba->priv;
	/* set cport connection status = 1 */
	ufshcd_dme_set(hba, UIC_ARG_MIB(T_CONNECTIONSTATE), 0x1);

	/* MPHY TX sync length changes to MAX */
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_TxHsG1SyncLength), 0x4f);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_TxHsG2SyncLength), 0x4f);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_TxHsG3SyncLength), 0x4f);

	/* Local Timer Value Changes */
	ufshcd_dme_set(hba, UIC_ARG_MIB(DME_FC0PROTECTIONTIMEOUTVAL), 0x1fff);
	ufshcd_dme_set(hba, UIC_ARG_MIB(DME_TC0REPLAYTIMEOUTVAL), 0xffff);
	ufshcd_dme_set(hba, UIC_ARG_MIB(DME_AFC0REQTIMEOUTVAL), 0x7fff);

	/* PEER TIMER values changes - PA_PWRModeUserData */
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA0), 0x1fff);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA1), 0xffff);
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_PWRMODEUSERDATA2), 0x7fff);

	/* After link start configuration request from Host controller,
	 * burst closure delay needs to be configured.
	 */
	ufshcd_dme_set(hba, UIC_ARG_MIB(VS_TXBURSTCLOSUREDELAY),
					ufs_tegra->vs_burst);

}

static void ufs_tegra_unipro_pre_linkup(struct ufs_hba *hba)
{
	struct ufs_tegra_host *ufs_tegra = hba->priv;
	/* Unipro LCC disable */
	ufshcd_dme_set(hba, UIC_ARG_MIB(PA_Local_TX_LCC_Enable), 0x0);
	/* Before link start configuration request from Host controller,
	 * burst closure delay needs to be configured to 0[7:0]
	 */
	ufshcd_dme_get(hba, UIC_ARG_MIB(VS_TXBURSTCLOSUREDELAY),
			&ufs_tegra->vs_burst);
	ufshcd_dme_set(hba, UIC_ARG_MIB(VS_TXBURSTCLOSUREDELAY), 0x0);
}

static int ufs_tegra_link_startup_notify(struct ufs_hba *hba,
			enum ufs_notify_change_status status)
{
	struct ufs_tegra_host *ufs_tegra = hba->priv;
	int err = 0;

	switch (status) {
	case PRE_CHANGE:
		ufs_tegra_mphy_rx_sync_capabity(ufs_tegra);
		ufs_tegra_unipro_pre_linkup(hba);
		break;
	case POST_CHANGE:
		/*POST_CHANGE case is called on success of link start-up*/
		dev_info(hba->dev, "UFS card detected - dme-link-startup Successful\n");
		ufs_tegra_unipro_post_linkup(hba);
		err = ufs_tegra_mphy_receiver_calibration(ufs_tegra);
		break;
	default:
		break;
	}
	return err;
}

static int ufs_tegra_context_save_init(struct ufs_tegra_host *ufs_tegra)
{
	u32 context_save_size = 0;
	struct device *dev = ufs_tegra->hba->dev;
	int err = 0;

	/**
	 * Allocate memory for 2-mphy lanes (lane 0 and lane 1)rx and
	 * tx registers
	 */
	context_save_size = 2 * (ARRAY_SIZE(mphy_rx_apb) +
				ARRAY_SIZE(mphy_tx_apb));
	context_save_size *= sizeof(u32);

	ufs_tegra->mphy_context = devm_kzalloc(dev, context_save_size,
								GFP_KERNEL);
	if (!ufs_tegra->mphy_context) {
		err = -ENOMEM;
		dev_err(dev, "no memory for tegra ufs mphy_context\n");
	}
	return err;
}


static void ufs_tegra_config_soc_data(struct ufs_tegra_host *ufs_tegra)
{
	struct device *dev = ufs_tegra->hba->dev;
	struct device_node *np = dev->of_node;

	ufs_tegra->enable_mphy_rx_calib =
		of_property_read_bool(np, "nvidia,enable-rx-calib");

	if (of_property_read_bool(np, "nvidia,enable-hibern8-war"))
		ufs_tegra->nvquirks |= NVQUIRK_BROKEN_HIBERN8_ENTRY;

	ufs_tegra->x2config =
		of_property_read_bool(np, "nvidia,enable-x2-config");

	ufs_tegra->enable_hs_mode =
		of_property_read_bool(np, "nvidia,enable-hs-mode");

	ufs_tegra->mask_fast_auto_mode =
		of_property_read_bool(np, "nvidia,mask-fast-auto-mode");

	ufs_tegra->mask_hs_mode_b =
		of_property_read_bool(np, "nvidia,mask-hs-mode-b");

	ufs_tegra->enable_ufs_provisioning =
		of_property_read_bool(np, "nvidia,enable-ufs-provisioning");
	ufs_tegra->configure_uphy_pll3 =
		of_property_read_bool(np, "nvidia,configure-uphy-pll3");


	of_property_read_u32(np, "nvidia,max-hs-gear", &ufs_tegra->max_hs_gear);
	of_property_read_u32(np, "nvidia,max-pwm-gear",
					&ufs_tegra->max_pwm_gear);

	if(of_property_read_bool(np, "nvidia,enable-wlu-scsi-device-add"))
		ufs_tegra->hba->quirks |= UFSHCD_QUIRK_ENABLE_WLUNS;

	ufs_tegra->cd_gpio = of_get_named_gpio(np, "nvidia,cd-gpios", 0);
	ufs_tegra->cd_wakeup_capable =
		of_property_read_bool(np, "nvidia,cd-wakeup-capable");
	ufs_tegra->enable_scramble =
		of_property_read_bool(np, "nvidia,enable-scramble");
}

/**
 * ufs_tegra_init - bind phy with controller
 * @hba: host controller instance
 *
 * Binds PHY with controller and powers up UPHY enabling clocks
 * and regulators.
 *
 * Returns -EPROBE_DEFER if binding fails, returns negative error
 * on phy power up failure and returns zero on success.
 */
static int ufs_tegra_init(struct ufs_hba *hba)
{
	struct ufs_tegra_host *ufs_tegra;
	struct device *dev = hba->dev;
	int err = 0;

	ufs_tegra = devm_kzalloc(dev, sizeof(*ufs_tegra), GFP_KERNEL);
	if (!ufs_tegra) {
		err = -ENOMEM;
		dev_err(dev, "no memory for tegra ufs host\n");
		goto out;
	}
	ufs_tegra->ufshc_state = UFSHC_INIT;
	ufs_tegra->hba = hba;
	hba->priv = (void *)ufs_tegra;

	ufs_tegra_config_soc_data(ufs_tegra);
	hba->spm_lvl = UFS_PM_LVL_3;
	hba->rpm_lvl = UFS_PM_LVL_1;
	hba->caps |= UFSHCD_CAP_INTR_AGGR;

	ufs_tegra->ufs_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR_OR_NULL(ufs_tegra->ufs_pinctrl)) {
		err = PTR_ERR(ufs_tegra->ufs_pinctrl);
		ufs_tegra->ufs_pinctrl = NULL;
		dev_err(dev, "pad control get failed, error:%d\n", err);
	}

	ufs_tegra->dpd_enable = pinctrl_lookup_state(ufs_tegra->ufs_pinctrl,
						     "ufs_dpd_enable");
	if (IS_ERR_OR_NULL(ufs_tegra->dpd_enable))
		dev_err(dev, "Missing dpd_enable state, err: %ld\n",
			PTR_ERR(ufs_tegra->dpd_enable));

	ufs_tegra->dpd_disable = pinctrl_lookup_state(ufs_tegra->ufs_pinctrl,
						     "ufs_dpd_disable");
	if (IS_ERR_OR_NULL(ufs_tegra->dpd_disable))
		dev_err(dev, "Missing dpd_disable state, err: %ld\n",
			PTR_ERR(ufs_tegra->dpd_disable));

	if (gpio_is_valid(ufs_tegra->cd_gpio) &&
			ufs_tegra->cd_wakeup_capable) {
		ufs_tegra->cd_gpio_desc = gpio_to_desc(ufs_tegra->cd_gpio);
		hba->card_present = ufs_tegra_get_cd(ufs_tegra->cd_gpio_desc);

		ufs_tegra->cd_irq = gpio_to_irq(ufs_tegra->cd_gpio);
		if (ufs_tegra->cd_irq <= 0) {
			dev_err(dev, "failed to get gpio irq %d, hence wakeup"
				" with UFS can't be supported\n",
				ufs_tegra->cd_irq);
			ufs_tegra->cd_irq = 0;
		} else {
			device_init_wakeup(dev, 1);
			dev_info(dev, "wakeup init done, cdirq %d\n",
				 ufs_tegra->cd_irq);
			err = devm_request_irq(dev, ufs_tegra->cd_irq,
				ufs_cd_gpio_isr, IRQF_TRIGGER_RISING |
				IRQF_TRIGGER_FALLING,
				"ufs_cd_gpio", ufs_tegra);
			if (err)
				dev_err(dev, "request irq failed ret: %d,"
					"wakeup with UFS can't be supported\n",
					err);
		}
		INIT_DELAYED_WORK(&ufs_tegra->detect, ufs_rescan);
	} else {
		/*
		 * If card detect gpio is not specified then assume
		 * UFS device is non-removable and it is always present
		 */
		hba->card_present = 1;
	}

	ufs_tegra->ufs_aux_base = devm_ioremap(dev,
			NV_ADDRESS_MAP_UFSHC_AUX_BASE, UFS_AUX_ADDR_RANGE);
	if (!ufs_tegra->ufs_aux_base) {
		err = -ENOMEM;
		dev_err(dev, "ufs_aux_base ioremap failed\n");
		goto out;
	}

	if (tegra_platform_is_silicon()) {
		ufs_tegra->mphy_l0_base = devm_ioremap(dev,
				NV_ADDRESS_MAP_MPHY_L0_BASE, MPHY_ADDR_RANGE);
		if (!ufs_tegra->ufs_aux_base) {
			err = -ENOMEM;
			dev_err(dev, "mphy_l0_base ioremap failed\n");
			goto out;
		}
		ufs_tegra->mphy_l1_base = devm_ioremap(dev,
				NV_ADDRESS_MAP_MPHY_L1_BASE, MPHY_ADDR_RANGE);
		if (!ufs_tegra->ufs_aux_base) {
			err = -ENOMEM;
			dev_err(dev, "mphy_l1_base ioremap failed\n");
			goto out;
		}

		err = ufs_tegra_context_save_init(ufs_tegra);
		if (err)
			goto out;
	}

	if (tegra_platform_is_silicon()) {
		err = ufs_tegra_init_ufs_clks(ufs_tegra);
		if (err)
			goto out_host_free;

		err = ufs_tegra_init_mphy_lane_clks(ufs_tegra);
		if (err)
			goto out_host_free;
		err = ufs_tegra_init_uphy_pll3(ufs_tegra);
		if (err)
			goto out_host_free;
		err = ufs_tegra_host_clk_enable(dev, "mphy_force_ls_mode",
				ufs_tegra->mphy_force_ls_mode);
		if (err)
			goto out_host_free;
		usleep_range(1000, 2000);
		clk_disable_unprepare(ufs_tegra->mphy_force_ls_mode);
		usleep_range(1000, 2000);

		err = ufs_tegra_enable_ufs_clks(ufs_tegra);
		if (err)
			goto out_host_free;

		err = ufs_tegra_enable_mphylane_clks(ufs_tegra);
		if (err)
			goto out_disable_ufs_clks;

		err = ufs_tegra_mphy_reset_init(ufs_tegra);
		if (err)
			goto out_disable_mphylane_clks;

		ufs_tegra_mphy_deassert_reset(ufs_tegra);

		err = ufs_tegra_ufs_reset_init(ufs_tegra);
		if (err)
			goto out_disable_mphylane_clks;

		ufs_tegra_ufs_deassert_reset(ufs_tegra);
		ufs_tegra_mphy_rx_advgran(ufs_tegra);
		ufs_tegra_ufs_aux_ref_clk_disable(ufs_tegra);
		ufs_tegra_aux_reset_enable(ufs_tegra);
		ufs_tegra_ufs_aux_prog(ufs_tegra);
		ufs_tegra_cfg_vendor_registers(hba);
#ifdef CONFIG_DEBUG_FS
		ufs_tegra_init_debugfs(hba);
#endif
	}
	return err;

out_disable_mphylane_clks:
	if (tegra_platform_is_silicon())
		ufs_tegra_disable_mphylane_clks(ufs_tegra);
out_disable_ufs_clks:
	ufs_tegra_disable_ufs_clks(ufs_tegra);
out_host_free:
	if (tegra_platform_is_silicon())
		hba->priv = NULL;
out:
	return err;
}

static void ufs_tegra_exit(struct ufs_hba *hba)
{
	struct ufs_tegra_host *ufs_tegra = hba->priv;

	if (tegra_platform_is_silicon())
		ufs_tegra_disable_mphylane_clks(ufs_tegra);
#ifdef CONFIG_DEBUG_FS
	if (ufs_tegra->enable_ufs_provisioning)
		debugfs_provision_exit(hba);
#endif
}

/**
 * struct ufs_hba_tegra_vops - UFS TEGRA specific variant operations
 *
 * The variant operations configure the necessary controller and PHY
 * handshake during initialization.
 */
struct ufs_hba_variant_ops ufs_hba_tegra_vops = {
	.name                   = "ufs-tegra",
	.init                   = ufs_tegra_init,
	.exit                   = ufs_tegra_exit,
	.suspend		= ufs_tegra_suspend,
	.resume			= ufs_tegra_resume,
	.hce_disable_notify     = ufs_tegra_hce_disable_notify,
	.hce_enable_notify      = ufs_tegra_hce_enable_notify,
	.link_startup_notify	= ufs_tegra_link_startup_notify,
	.pwr_change_notify      = ufs_tegra_pwr_change_notify,
	.hibern8_entry_notify   = ufs_tegra_hibern8_entry_notify,
	.set_ufs_mphy_clocks	= ufs_tegra_set_ufs_mphy_clocks,
};

static int ufs_tegra_probe(struct platform_device *pdev)
{
	int err;
	struct device *dev = &pdev->dev;

	/* Perform generic probe */
	err = ufshcd_pltfrm_init(pdev, &ufs_hba_tegra_vops);
	if (err)
		dev_err(dev, "ufshcd_pltfrm_init() failed %d\n", err);

	return err;
}

static int ufs_tegra_remove(struct platform_device *pdev)
{
	struct ufs_hba *hba =  platform_get_drvdata(pdev);

	pm_runtime_get_sync(&(pdev)->dev);
	ufshcd_remove(hba);
	return 0;

}

static const struct of_device_id ufs_tegra_of_match[] = {
	{ .compatible = "tegra,ufs_variant"},
	{},
};

static const struct dev_pm_ops ufs_tegra_pm_ops = {
	.suspend        = ufshcd_pltfrm_suspend,
	.resume         = ufshcd_pltfrm_resume,
	.runtime_suspend = ufshcd_pltfrm_runtime_suspend,
	.runtime_resume  = ufshcd_pltfrm_runtime_resume,
	.runtime_idle    = ufshcd_pltfrm_runtime_idle,
};

static struct platform_driver ufs_tegra_platform = {
	.probe = ufs_tegra_probe,
	.shutdown = ufshcd_pltfrm_shutdown,
	.remove = ufs_tegra_remove,
	.driver = {
		.name = "ufs_tegra",
		.pm   = &ufs_tegra_pm_ops,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ufs_tegra_of_match),
	},
};
module_platform_driver(ufs_tegra_platform);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Naveen Kumar Arepalli <naveenk@nvidia.com>");
MODULE_AUTHOR("Venkata Jagadish <vjagadish@nvidia.com>");
MODULE_DEVICE_TABLE(of, ufs_tegra_of_match);
