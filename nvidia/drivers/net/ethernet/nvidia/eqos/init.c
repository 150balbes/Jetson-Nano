/* =========================================================================
 * The Synopsys DWC ETHER QOS Software Driver and documentation (hereinafter
 * "Software") is an unsupported proprietary work of Synopsys, Inc. unless
 * otherwise expressly agreed to in writing between Synopsys and you.
 *
 * The Software IS NOT an item of Licensed Software or Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Product
 * with Synopsys or any supplement thereto.  Permission is hereby granted,
 * free of charge, to any person obtaining a copy of this software annotated
 * with this license and the Software, to deal in the Software without
 * restriction, including without limitation the rights to use, copy, modify,
 * merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject
 * to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS" BASIS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 * =========================================================================
 */
/*
 * Copyright (c) 2015-2020, NVIDIA CORPORATION.  All rights reserved.
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


/*!@file: eqos_init.c
 * @brief: Driver functions.
 */
#include "yheader.h"
#include "init.h"
#include "yregacc.h"
#include "nvregacc.h"
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <soc/tegra/chip-id.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/tegra_prod.h>
#include <linux/of_net.h>

#define LP_SUPPORTED 0
static const struct of_device_id eqos_of_match[] = {
	{ .compatible = "nvidia,eqos" },
	{},
};
MODULE_DEVICE_TABLE(of, eqos_of_match);

static DEVICE_ATTR(mac_loopback, S_IRUGO | S_IWUSR,
				eqos_mac_loopback_show,
				eqos_mac_loopback_store);

static struct device_attribute *eqos_sysfs_attrs[] = {
				&dev_attr_mac_loopback,
};

ULONG eqos_base_addr;

void eqos_init_all_fptrs(struct eqos_prv_data *pdata)
{
	eqos_init_function_ptrs_dev(&pdata->hw_if);
	eqos_init_function_ptrs_desc(&pdata->desc_if);
}

/*!
* \brief POWER Interrupt Service Routine
* \details POWER Interrupt Service Routine
*
* \param[in] irq         - interrupt number for particular device
* \param[in] device_id   - pointer to device structure
* \return returns positive integer
* \retval IRQ_HANDLED
*/
irqreturn_t EQOS_ISR_SW_EQOS_POWER(int irq, void *device_id)
{
	struct eqos_prv_data *pdata = (struct eqos_prv_data *)device_id;
	ULONG mac_isr;
	ULONG mac_imr;
	ULONG mac_pmtcsr;
	ULONG clk_ctrl = 0;

	if (tegra_platform_is_unit_fpga())
		CLK_CRTL0_RD(clk_ctrl);

	if (clk_ctrl & BIT(31)) {
		pr_debug("power_isr: phy_intr received\n");
		return IRQ_NONE;
	} else {
		MAC_ISR_RD(mac_isr);
		MAC_IMR_RD(mac_imr);
		pr_debug("power_isr: power_intr received, MAC_ISR =%#lx, MAC_IMR =%#lx\n",
				mac_isr, mac_imr);

		mac_isr = (mac_isr & mac_imr);

		/* RemoteWake and MagicPacket events will be received by PHY
		 * supporting these features on silicon and can be used to wake
		 * up Tegra. Still let the below code be there in case we ever
		 * get this interrupt.
		 */
		if (GET_VALUE(mac_isr, MAC_ISR_PMTIS_LPOS, MAC_ISR_PMTIS_HPOS)
			& 1) {
			pdata->xstats.pmt_irq_n++;
			MAC_PMTCSR_RD(mac_pmtcsr);
			pr_debug("power_isr: PMTCSR : %#lx\n", mac_pmtcsr);
		}

		/* RxLPI exit EEE interrupts */
		if (GET_VALUE(mac_isr, MAC_ISR_LPI_LPOS, MAC_ISR_LPI_HPOS)
			& 1) {
			pr_debug("power_isr: LPI intr received\n");
			eqos_handle_eee_interrupt(pdata);
#ifdef HWA_NV_1650337
		/* FIXME: remove once root cause of HWA_NV_1650337 is known */
		} else {
			/* We have seen power_intr flood without LPIIS set in
			 * MAC_ISR and need to still read MAC_LPI_CONTROL_STS
			 * register to get rid of interrupt storm issue.
			 */
			pr_debug("power_isr: LPIIS not set in MAC_ISR but still"
				" reading MAC_LPI_CONTROL_STS\n");
			eqos_handle_eee_interrupt(pdata);
#endif
		}

		return IRQ_HANDLED;
	}
}

void get_dt_u32(struct eqos_prv_data *pdata, char *pdt_prop, u32 *pval,
		u32 val_def, u32 val_max)
{
	struct device_node *pnode = pdata->pdev->dev.of_node;
	int ret = 0;

	ret = of_property_read_u32(pnode, pdt_prop, pval);

	if (ret < 0)
		dev_err(&pdata->pdev->dev,
			"%s(): \"%s\" read failed %d. Using default\n",
			__func__, pdt_prop, ret);

	if (*pval > val_max) {
		dev_err(&pdata->pdev->dev,
			"%s(): %d is invalid value for \"%s\".  Using default.\n",
			__func__, *pval, pdt_prop);
		*pval = val_def;
	}
}


void get_dt_u32_array(struct eqos_prv_data *pdata, char *pdt_prop, u32 *pval,
			u32 val_def, u32 val_max, u32 num_entries)
{
	struct device_node *pnode = pdata->pdev->dev.of_node;
	int i, ret = 0;

	ret = of_property_read_u32_array(pnode, pdt_prop, pval, num_entries);

	if (ret < 0) {
		pr_err("%s(): \"%s\" read failed %d. Using default\n",
			__func__, pdt_prop, ret);
		for (i = 0; i < num_entries; i++)
			pval[i] = val_def;
	}
	for (i = 0; i < num_entries; i++) {
		if (!strcmp(pdt_prop, "nvidia,queue_prio")) {
			int j;

			if (pval[i] > val_max) {
				dev_err(&pdata->pdev->dev,
					"%d is invalid value for"
					" queue_prio[%d], using default %d\n",
					pval[i], i, i + val_def);
				pval[i] = val_def + i;
			}
			/* q_prio need to be exclusive for each queue */
			for (j = 0; j < i; j++) {
				if (i == j)
					continue;
				/* use default if two priorities are same */
				if (pval[i] == pval[j]) {
					dev_err(&pdata->pdev->dev,
						"queue_prio %d same"
						" for q%d and q%d, using default\n",
						pval[i], i, j);
					pval[i] = val_def + i;
					pval[j] = val_def + j;
				}
			}
		} else if (pval[i] > val_max) {
			dev_err(&pdata->pdev->dev,
				"%d is invalid value for \"%s[%d]\"."
				"  Using default.\n",
				pval[i], pdt_prop, i);
			pval[i] = val_def;
		}
	}
}

static void eqos_clock_deinit(struct eqos_prv_data *pdata)
{
	struct platform_device *pdev = pdata->pdev;

	if (!IS_ERR_OR_NULL(pdata->tx_clk))
		clk_disable_unprepare(pdata->tx_clk);
	if (!IS_ERR_OR_NULL(pdata->ptp_ref_clk))
		clk_disable_unprepare(pdata->ptp_ref_clk);
	if (!IS_ERR_OR_NULL(pdata->rx_clk))
		clk_disable_unprepare(pdata->rx_clk);
	if (!IS_ERR_OR_NULL(pdata->axi_clk))
		clk_disable_unprepare(pdata->axi_clk);
	if (!IS_ERR_OR_NULL(pdata->axi_cbb_clk))
		clk_disable_unprepare(pdata->axi_cbb_clk);
	if (!IS_ERR_OR_NULL(pdata->pllrefe_clk))
		clk_disable_unprepare(pdata->pllrefe_clk);

	if (!IS_ERR_OR_NULL(pdata->tx_clk))
		devm_clk_put(&pdev->dev, pdata->tx_clk);
	if (!IS_ERR_OR_NULL(pdata->ptp_ref_clk))
		devm_clk_put(&pdev->dev, pdata->ptp_ref_clk);
	if (!IS_ERR_OR_NULL(pdata->rx_clk))
		devm_clk_put(&pdev->dev, pdata->rx_clk);
	if (!IS_ERR_OR_NULL(pdata->axi_clk))
		devm_clk_put(&pdev->dev, pdata->axi_clk);
	if (!IS_ERR_OR_NULL(pdata->axi_cbb_clk))
		devm_clk_put(&pdev->dev, pdata->axi_cbb_clk);
	if (!IS_ERR_OR_NULL(pdata->pllrefe_clk))
		devm_clk_put(&pdev->dev, pdata->pllrefe_clk);
}

static int eqos_set_ptp_ref_clk(struct eqos_prv_data *pdata,
				struct device_node *node)
{
	struct platform_device *pdev = pdata->pdev;
	u32 ptp_ref_clock_speed;
	int ret;

	ret = of_property_read_u32(node, "nvidia,ptp_ref_clock_speed",
				   &ptp_ref_clock_speed);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to read PTP ref clk (%d)\n", ret);
		/* Setting default values */
		if (pdata->mac_ver > EQOS_MAC_CORE_4_10)
			ptp_ref_clock_speed = 312500000;
		else
			ptp_ref_clock_speed = 125;
	}

	if (pdata->mac_ver > EQOS_MAC_CORE_4_10) {
		ret = clk_set_rate(pdata->ptp_ref_clk, ptp_ref_clock_speed);
		if (!ret)
			pdata->ptp_ref_clk_rate = ptp_ref_clock_speed;
	} else {
		ret = clk_set_rate(pdata->ptp_ref_clk,
				   ptp_ref_clock_speed * 1000000);
		if (!ret)
			pdata->ptp_ref_clk_rate = ptp_ref_clock_speed * 1000000;
	}

	return ret;
}

int eqos_clock_enable(struct eqos_prv_data *pdata)
{
	int ret;

	if (!IS_ERR_OR_NULL(pdata->pllrefe_clk)) {
		ret = clk_prepare_enable(pdata->pllrefe_clk);
		if (ret < 0)
			return ret;
	}

	if (!IS_ERR_OR_NULL(pdata->axi_cbb_clk)) {
		ret = clk_prepare_enable(pdata->axi_cbb_clk);
		if (ret)
			goto err_axi_cbb;
	}

	if (!IS_ERR_OR_NULL(pdata->axi_clk)) {
		ret = clk_prepare_enable(pdata->axi_clk);
		if (ret < 0)
			goto err_axi;
	}

	if (!IS_ERR_OR_NULL(pdata->rx_clk)) {
		ret = clk_prepare_enable(pdata->rx_clk);
		if (ret < 0)
			goto err_rx;
	}

	if (!IS_ERR_OR_NULL(pdata->ptp_ref_clk)) {
		ret = clk_prepare_enable(pdata->ptp_ref_clk);
		if (ret < 0)
			goto err_ptp_ref;
	}

	if (!IS_ERR_OR_NULL(pdata->tx_clk)) {
		ret = clk_prepare_enable(pdata->tx_clk);
		if (ret < 0)
			goto err_tx;
	}

	pdata->clks_enable = true;

	return 0;

err_axi_cbb:
	if (!IS_ERR_OR_NULL(pdata->pllrefe_clk))
		clk_disable_unprepare(pdata->pllrefe_clk);
err_tx:
	if (!IS_ERR_OR_NULL(pdata->ptp_ref_clk))
		clk_disable_unprepare(pdata->ptp_ref_clk);
err_ptp_ref:
	if (!IS_ERR_OR_NULL(pdata->rx_clk))
		clk_disable_unprepare(pdata->rx_clk);
err_rx:
	if (!IS_ERR_OR_NULL(pdata->axi_clk))
		clk_disable_unprepare(pdata->axi_clk);
err_axi:
	if (!IS_ERR_OR_NULL(pdata->axi_cbb_clk))
		clk_disable_unprepare(pdata->axi_cbb_clk);

	return ret;
}

void eqos_clock_disable(struct eqos_prv_data *pdata)
{
	if (!IS_ERR_OR_NULL(pdata->axi_cbb_clk))
		clk_disable_unprepare(pdata->axi_cbb_clk);

	if (!IS_ERR_OR_NULL(pdata->axi_clk))
		clk_disable_unprepare(pdata->axi_clk);

	if (!IS_ERR_OR_NULL(pdata->rx_clk))
		clk_disable_unprepare(pdata->rx_clk);

	if (!IS_ERR_OR_NULL(pdata->ptp_ref_clk))
		clk_disable_unprepare(pdata->ptp_ref_clk);

	if (!IS_ERR_OR_NULL(pdata->tx_clk))
		clk_disable_unprepare(pdata->tx_clk);

	if (!IS_ERR_OR_NULL(pdata->pllrefe_clk))
		clk_disable_unprepare(pdata->pllrefe_clk);

	pdata->clks_enable = false;
}

static int eqos_get_clocks(struct eqos_prv_data *pdata)
{
	struct platform_device *pdev = pdata->pdev;
	int ret;

	pdata->pllrefe_clk = devm_clk_get(&pdev->dev, "pllrefe_vcoout");
	if (IS_ERR(pdata->pllrefe_clk)) {
		ret = PTR_ERR(pdata->pllrefe_clk);
		dev_info(&pdev->dev, "can't get pllrefe_vcoout clk (%d)\n", ret);
	}

	pdata->axi_cbb_clk = devm_clk_get(&pdev->dev, "axi_cbb");
	if (IS_ERR(pdata->axi_cbb_clk)) {
		ret = PTR_ERR(pdata->axi_cbb_clk);
		dev_err(&pdev->dev, "can't get axi_cbb clk (%d)\n", ret);
		goto axi_cbb_get_fail;
	}
	pdata->axi_clk = devm_clk_get(&pdev->dev, "eqos_axi");
	if (IS_ERR(pdata->axi_clk)) {
		ret = PTR_ERR(pdata->axi_clk);
		dev_err(&pdev->dev, "can't get eqos_axi clk (%d)\n", ret);
		goto axi_get_fail;
	}
	pdata->rx_clk = devm_clk_get(&pdev->dev, "eqos_rx");
	if (IS_ERR(pdata->rx_clk)) {
		ret = PTR_ERR(pdata->rx_clk);
		dev_err(&pdev->dev, "can't get eqos_rx clk (%d)\n", ret);
		goto rx_get_fail;
	}
	pdata->ptp_ref_clk = devm_clk_get(&pdev->dev, "eqos_ptp_ref");
	if (IS_ERR(pdata->ptp_ref_clk)) {
		ret = PTR_ERR(pdata->ptp_ref_clk);
		dev_err(&pdev->dev, "can't get eqos_ptp_ref clk (%d)\n", ret);
		goto ptp_ref_get_fail;
	}
	pdata->tx_clk = devm_clk_get(&pdev->dev, "eqos_tx");
	if (IS_ERR(pdata->tx_clk)) {
		ret = PTR_ERR(pdata->tx_clk);
		dev_err(&pdev->dev, "can't get eqos_tx clk (%d)\n", ret);
		goto tx_get_fail;
	}

	return 0;

tx_get_fail:
	devm_clk_put(&pdev->dev, pdata->ptp_ref_clk);
ptp_ref_get_fail:
	devm_clk_put(&pdev->dev, pdata->rx_clk);
rx_get_fail:
	devm_clk_put(&pdev->dev, pdata->axi_clk);
axi_get_fail:
	devm_clk_put(&pdev->dev, pdata->axi_cbb_clk);
axi_cbb_get_fail:
	devm_clk_put(&pdev->dev, pdata->pllrefe_clk);
	return ret;
}

static void eqos_regulator_deinit(struct eqos_prv_data *pdata)
{
	if (!IS_ERR_OR_NULL(pdata->vddio_sys_enet_bias)) {
		regulator_disable(pdata->vddio_sys_enet_bias);
		devm_regulator_put(pdata->vddio_sys_enet_bias);
	}
	if (!IS_ERR_OR_NULL(pdata->vddio_enet)) {
		regulator_disable(pdata->vddio_enet);
		devm_regulator_put(pdata->vddio_enet);
	}
	if (!IS_ERR_OR_NULL(pdata->phy_pllvdd)) {
		regulator_disable(pdata->phy_pllvdd);
		devm_regulator_put(pdata->phy_pllvdd);
	}
	if (!IS_ERR_OR_NULL(pdata->phy_ovdd_rgmii)) {
		regulator_disable(pdata->phy_ovdd_rgmii);
		devm_regulator_put(pdata->phy_ovdd_rgmii);
	}
	if (!IS_ERR_OR_NULL(pdata->phy_vdd_1v8)) {
		regulator_disable(pdata->phy_vdd_1v8);
		devm_regulator_put(pdata->phy_vdd_1v8);
	}
}

static int eqos_regulator_init(struct eqos_prv_data *pdata)
{
	struct platform_device *pdev = pdata->pdev;
	int ret = 0;

	pdata->phy_vdd_1v8 = devm_regulator_get(&pdev->dev, "phy_vdd_1v8");
	if (IS_ERR(pdata->phy_vdd_1v8)) {
		ret = PTR_ERR(pdata->phy_vdd_1v8);
		dev_err(&pdev->dev, "phy_vdd_1v8 get failed %d\n", ret);
		return ret;
	}

	pdata->phy_ovdd_rgmii =
		devm_regulator_get(&pdev->dev, "phy_ovdd_rgmii");
	if (IS_ERR(pdata->phy_ovdd_rgmii)) {
		ret = PTR_ERR(pdata->phy_ovdd_rgmii);
		dev_err(&pdev->dev, "phy_ovdd_rgmii get failed %d\n", ret);
		goto phy_ovdd_rgmii_get_failed;
	}

	pdata->phy_pllvdd = devm_regulator_get(&pdev->dev,
		"phy_pllvdd");
	if (IS_ERR(pdata->phy_pllvdd)) {
		ret = PTR_ERR(pdata->phy_pllvdd);
		dev_err(&pdev->dev, "phy_pllvdd get failed %d\n", ret);
		goto phy_pllvdd_get_failed;
	}

	pdata->vddio_enet = devm_regulator_get(&pdev->dev, "vddio_enet");
	if (IS_ERR(pdata->vddio_enet)) {
		ret = PTR_ERR(pdata->vddio_enet);
		dev_err(&pdev->dev, "vddio_enet get failed %d\n", ret);
		goto vddio_enet_get_failed;
	}

	pdata->vddio_sys_enet_bias = devm_regulator_get(&pdev->dev,
		"vddio_sys_enet_bias");
	if (IS_ERR(pdata->vddio_sys_enet_bias)) {
		ret = PTR_ERR(pdata->vddio_sys_enet_bias);
		dev_err(&pdev->dev, "vddio_sys_enet_bias get failed %d\n", ret);
		goto vddio_sys_enet_bias_get_failed;
	}

	ret = regulator_enable(pdata->phy_vdd_1v8);
	if (ret) {
		dev_err(&pdev->dev, "phy_vdd_1v8 enable failed %d\n", ret);
		goto phy_vdd_1v8_enable_failed;
	}

	ret = regulator_enable(pdata->phy_ovdd_rgmii);
	if (ret) {
		dev_err(&pdev->dev, "phy_ovdd_rgmii enable failed %d\n", ret);
		goto phy_ovdd_rgmii_enable_failed;
	}

	ret = regulator_enable(pdata->phy_pllvdd);
	if (ret) {
		dev_err(&pdev->dev, "phy_pllvdd enable failed %d\n", ret);
		goto phy_pllvdd_enable_failed;
	}

	ret = regulator_enable(pdata->vddio_enet);
	if (ret) {
		dev_err(&pdev->dev, "vddio_enet enable failed %d\n", ret);
		goto vddio_enet_enable_failed;
	}

	ret = regulator_enable(pdata->vddio_sys_enet_bias);
	if (ret) {
		dev_err(&pdev->dev, "vddio_sys_enet_bias enable failed %d\n",
			ret);
		goto vddio_sys_enet_bias_enable_failed;
	}

	return 0;

vddio_sys_enet_bias_enable_failed:
	regulator_disable(pdata->vddio_enet);
vddio_enet_enable_failed:
	regulator_disable(pdata->phy_pllvdd);
phy_pllvdd_enable_failed:
	regulator_disable(pdata->phy_ovdd_rgmii);
phy_ovdd_rgmii_enable_failed:
	regulator_disable(pdata->phy_vdd_1v8);
phy_vdd_1v8_enable_failed:
	devm_regulator_put(pdata->vddio_sys_enet_bias);
vddio_sys_enet_bias_get_failed:
	devm_regulator_put(pdata->vddio_enet);
vddio_enet_get_failed:
	devm_regulator_put(pdata->phy_pllvdd);
phy_pllvdd_get_failed:
	devm_regulator_put(pdata->phy_ovdd_rgmii);
phy_ovdd_rgmii_get_failed:
	devm_regulator_put(pdata->phy_vdd_1v8);
	return ret;
}

static int eqos_get_phyreset_from_gpio(struct eqos_prv_data *pdata)
{
	struct platform_device *pdev = pdata->pdev;
	struct device_node *node = pdev->dev.of_node;
	int ret;

	pdata->phy_reset_gpio =
		of_get_named_gpio(node, "nvidia,phy-reset-gpio", 0);
	if (pdata->phy_reset_gpio < 0)
		return -ENODEV;

	if (gpio_is_valid(pdata->phy_reset_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev, pdata->phy_reset_gpio,
				GPIOF_OUT_INIT_HIGH, "eqos_phy_reset");
		if (ret < 0) {
			dev_err(&pdev->dev, "phy_reset gpio_request failed\n");
			return ret;
		}
	} else {
		dev_err(&pdev->dev, "invalid phy_reset_gpio\n");
		return -ENODEV;
	}
	return 0;
}

static int eqos_get_phyirq_from_gpio(struct eqos_prv_data *pdata)
{
	struct platform_device *pdev = pdata->pdev;
	struct device_node *node = pdev->dev.of_node;
	int ret;

	pdata->phy_intr_gpio =
			of_get_named_gpio(node, "nvidia,phy-intr-gpio", 0);
	if (pdata->phy_intr_gpio < 0)
		return -ENODEV;

	if (gpio_is_valid(pdata->phy_intr_gpio)) {
		ret = devm_gpio_request_one(&pdev->dev, pdata->phy_intr_gpio,
				GPIOF_IN, "eqos_phy_intr");
		if (ret < 0) {
			dev_err(&pdev->dev, "phy_intr gpio_request failed\n");
			return ret;
		}
		ret = gpio_to_irq(pdata->phy_intr_gpio);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"gpio_to_irq failed for phy_intr\n");
			return ret;
		}
	} else {
		dev_err(&pdev->dev, "invalid phy_intr_gpio\n");
		return -ENODEV;
	}
	return ret;
}

/* Get MAC address from the specified DTB path */
static int eqos_get_mac_address_dtb(const char *node_name,
	const char *property_name, unsigned char *mac_addr)
{
	struct device_node *np = of_find_node_by_path(node_name);
	const char *mac_str = NULL;
	int values[6] = {0};
	unsigned char mac_temp[6] = {0};
	int i, ret = 0;

	if (!np)
		return -EADDRNOTAVAIL;

	/* If the property is present but contains an invalid value,
	 * then something is wrong. Log the error in that case.
	 */
	if (of_property_read_string(np, property_name, &mac_str)) {
		ret = -EADDRNOTAVAIL;
		goto err_out;
	}

	/* The DTB property is a string of the form xx:xx:xx:xx:xx:xx
	 * Convert to an array of bytes.
	 */
	if (sscanf(mac_str, "%x:%x:%x:%x:%x:%x",
		&values[0], &values[1], &values[2],
		&values[3], &values[4], &values[5]) != 6) {
		ret = -EINVAL;
		goto err_out;
	}

	for (i = 0; i < 6; ++i)
		mac_temp[i] = (unsigned char)values[i];

	if (!is_valid_ether_addr(mac_temp)) {
		ret = -EINVAL;
		goto err_out;
	}

	memcpy(mac_addr, mac_temp, 6);

	of_node_put(np);

	return ret;

err_out:
	pr_err("%s: bad mac address at %s/%s: %s.\n",
		__func__, node_name, property_name,
		(mac_str) ? mac_str : "null");

	of_node_put(np);

	return ret;
}

static void eqos_get_slot_num_check_queues(struct eqos_prv_data *pdata,
					   u32 *slot_num)
{
	struct device_node *node = pdata->pdev->dev.of_node;
	int qinx, ret = 0;

	if (pdata->num_chans == 1)
		return;

	ret = of_property_read_u32_array(node, "nvidia,slot_num_check",
					 slot_num, pdata->num_chans);
	if (ret)
		return;

	for (qinx = 0; qinx < pdata->num_chans; qinx++) {
		struct eqos_tx_queue *tx_queue = GET_TX_QUEUE_PTR(qinx);

		if (slot_num[qinx] == 1)
			tx_queue->slot_num_check = true;
	}
}

#ifdef CONFIG_THERMAL
#define TEGRA_EQOS_THERM_MAX_STATE     5
static int tegra_eqos_max_state(struct thermal_cooling_device *tcd,
				   unsigned long *state)
{
	struct eqos_prv_data *pdata = tcd->devdata;

	pr_debug("%s state=%d\n", __func__, pdata->therm_state.counter);
	*state = TEGRA_EQOS_THERM_MAX_STATE;

	return 0;
}

static int tegra_eqos_cur_state(struct thermal_cooling_device *tcd,
				   unsigned long *state)
{
	struct eqos_prv_data *pdata = tcd->devdata;

	pr_debug("%s state=%d\n", __func__, pdata->therm_state.counter);
	*state = (unsigned long)atomic_read(&pdata->therm_state);

	return 0;
}

static int tegra_eqos_set_state(struct thermal_cooling_device *tcd,
				   unsigned long state)
{
	struct eqos_prv_data *pdata = tcd->devdata;
	struct hw_if_struct *hw_if = &(pdata->hw_if);

	if (pdata->suspended || pdata->hw_stopped)
		return -ENODEV;

	mutex_lock(&pdata->hw_change_lock);
	if (!pdata->hw_stopped)
		netif_tx_stop_all_queues(pdata->dev);

	pr_debug("%s cur state=%d new state=%ld, recalibrating eqos pads\n",
		__func__, pdata->therm_state.counter, state);
	atomic_set(&pdata->therm_state, state);

	/* read-to-clear and save error counters if any */
	hw_if->read_err_counter(pdata, true);

	/* increment counter for temp based pad recalibration */
	pdata->xstats.temp_pad_recalib_count++;

	/* trigger the pad re-calibration */
	hw_if->pad_calibrate(pdata);

	/* read-to-clear error counters */
	hw_if->read_err_counter(pdata, false);

	if (!pdata->hw_stopped)
		netif_tx_wake_all_queues(pdata->dev);
	mutex_unlock(&pdata->hw_change_lock);

	return 0;
}

/* Cooling device support */
static struct thermal_cooling_device_ops eqos_cdev_ops = {
	.get_max_state = tegra_eqos_max_state,
	.get_cur_state = tegra_eqos_cur_state,
	.set_cur_state = tegra_eqos_set_state,
};
#endif

static int eqos_therm_init(struct eqos_prv_data *pdata)
{
#ifdef CONFIG_THERMAL
	struct device_node *np = NULL;

	np = of_find_node_by_name(NULL, "eqos-cool-dev");
	if (!np) {
		pr_err("failed to get eqos-cool-dev\n");
		return -ENODEV;
	}

	pdata->cdev = thermal_of_cooling_device_register(np,
			"tegra-eqos", pdata,
			&eqos_cdev_ops);
	if (IS_ERR(pdata->cdev))
		return PTR_ERR(pdata->cdev);
	if (pdata->cdev == NULL)
		return -ENODEV;

	pr_debug("EQOS cooling dev registered\n");
#endif
	return 0;
}

ssize_t eqos_mac_loopback_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	ssize_t ret = 1;
	struct net_device *ndev = NULL;
	struct eqos_prv_data *pdata = NULL;

	ndev = (struct net_device *)dev_get_drvdata(dev);
	pdata = netdev_priv(ndev);

	ret = sprintf(buf, "%d\n", pdata->mac_loopback_mode);
	return ret;
}

ssize_t eqos_mac_loopback_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct net_device *ndev = NULL;
	struct eqos_prv_data *pdata = NULL;

	ndev = (struct net_device *)dev_get_drvdata(dev);
	pdata = netdev_priv(ndev);

	if (!(pdata->hw_stopped)) {
		if (buf[0] == '1')
			eqos_config_mac_loopback_mode(ndev, 1);
		else if (buf[0] == '0')
			eqos_config_mac_loopback_mode(ndev, 0);
		else
			pr_err("incorrect entry\n");
	} else {
		pr_err("Not Allowed. Eth interface is not up\n");
	}
	return size;
}

/* Converts csr clock to MDC clock.
 * Values come from CR field in MAC_MDIO_Address register
 */
static void save_mdc(struct eqos_prv_data *pdata)
{
	if (pdata->csr_clock_speed > 500) {
		pdata->mdc_cr = EQOS_CSR_500_800M;
		return;
	}
	if (pdata->csr_clock_speed > 300) {
		pdata->mdc_cr = EQOS_CSR_300_500M;
		return;
	}
	if (pdata->csr_clock_speed > 250) {
		pdata->mdc_cr = EQOS_CSR_250_300M;
		return;
	}
	if (pdata->csr_clock_speed > 150) {
		pdata->mdc_cr = EQOS_CSR_150_250M;
		return;
	}
	if (pdata->csr_clock_speed > 100) {
		pdata->mdc_cr = EQOS_CSR_100_150M;
		return;
	}
	if (pdata->csr_clock_speed > 60) {
		pdata->mdc_cr = EQOS_CSR_60_100M;
		return;
	}
	if (pdata->csr_clock_speed > 35) {
		pdata->mdc_cr = EQOS_CSR_35_60M;
		return;
	}
	/* for CSR < 35mhz */
	pdata->mdc_cr = EQOS_CSR_20_35M;
}

static inline u32 eqos_get_mac_version(void)
{
	u32 hwid;

	/* read MAC version */
	MAC_VR_RD(hwid);
	if (likely(hwid)) {
		u32 uid = ((hwid & 0x0000ff00) >> 8);
		u32 synid = (hwid & 0x000000ff);

		pr_debug("mac - user ID: 0x%x, Synopsys ID: 0x%x\n",
			 uid, synid);

		return synid;
	}
	return 0;
}

static void eqos_get_phy_delays(struct eqos_prv_data *pdata)
{
	struct platform_device *pdev = pdata->pdev;
	struct device_node *node = pdev->dev.of_node;
	int ret = 0;

	ret = of_property_read_u32(node, "nvidia,phy-reset-duration",
				   &pdata->phy_reset_duration);
	if (ret < 0)
		pdata->phy_reset_duration = 10;

	ret = of_property_read_u32(node, "nvidia,phy-reset-post-delay",
				   &pdata->phy_reset_post_delay);
	if (ret < 0)
		pdata->phy_reset_post_delay = 0;
}

static enum hrtimer_restart eqos_tx_usecs_hrtimer(struct hrtimer *data)
{
	struct eqos_tx_queue *tx_queue = container_of(data,
			struct eqos_tx_queue,
			tx_usecs_timer);
	struct eqos_prv_data *pdata = tx_queue->pdata;

	pdata->xstats.tx_usecs_swtimer_n[tx_queue->chan_num]++;

	atomic_set(&tx_queue->tx_usecs_timer_armed,
		   EQOS_HRTIMER_DISABLE);
	if (likely(napi_schedule_prep(&tx_queue->napi)))
		 __napi_schedule_irqoff(&tx_queue->napi);

	return HRTIMER_NORESTART;
}

static int eqos_read_coalesc_params(struct platform_device *pdev,
				    struct eqos_prv_data *pdata,
				    struct device_node *np)
{
	int ret, i;
	u32 tx_usecs, rx_riwt, tx_frames, rx_frames;
	bool use_tx_usecs = EQOS_COAELSCING_DISABLE;
	bool use_tx_frames = EQOS_COAELSCING_DISABLE;
	bool use_riwt = EQOS_COAELSCING_DISABLE;
	bool use_rx_frames = EQOS_COAELSCING_DISABLE;

	/* tx_usecs value to be set */
	ret = of_property_read_u32(np, "nvidia,tx_usecs", &tx_usecs);
	if (ret < 0) {
		use_tx_usecs = EQOS_COAELSCING_DISABLE;
	} else {
		if (tx_usecs > EQOS_MAX_TX_COALESCE_USEC ||
		    tx_usecs < EQOS_MIN_TX_COALESCE_USEC) {
			dev_err(&pdev->dev,
				"invalid tx_riwt, must be in range %d to %d\n",
				EQOS_MIN_TX_COALESCE_USEC,
				EQOS_MAX_TX_COALESCE_USEC);
			return -EINVAL;
		}
		use_tx_usecs = EQOS_COAELSCING_ENABLE;
	}

	/* tx_frames value to be set */
	ret = of_property_read_u32(np, "nvidia,tx_frames", &tx_frames);
	if (ret < 0) {
		use_tx_frames = EQOS_COAELSCING_DISABLE;
	} else {
		if (tx_frames > EQOS_TX_MAX_FRAME ||
		    tx_frames < EQOS_MIN_TX_COALESCE_FRAMES) {
			dev_err(&pdev->dev,
				"invalid tx-frames, must be in range %d to %ld",
				EQOS_MIN_TX_COALESCE_FRAMES,
				EQOS_TX_MAX_FRAME);
			return -EINVAL;
		}
		use_tx_frames = EQOS_COAELSCING_ENABLE;
	}

	if (use_tx_usecs == EQOS_COAELSCING_DISABLE &&
	    use_tx_frames == EQOS_COAELSCING_ENABLE) {
		dev_err(&pdev->dev, "invalid settings : tx_frames must be enabled along with tx_usecs in DT\n");
		return -EINVAL;
	}

	/* RIWT value to be set */
	ret = of_property_read_u32(np, "nvidia,rx_riwt", &rx_riwt);
	if (ret < 0) {
		use_riwt = EQOS_COAELSCING_ENABLE;
	} else {
		if ((rx_riwt > EQOS_MAX_RX_COALESCE_USEC) ||
		    (rx_riwt < EQOS_MIN_RX_COALESCE_USEC)) {
			dev_err(&pdev->dev,
				"invalid rx_riwt, must be inrange %d to %d\n",
				EQOS_MIN_RX_COALESCE_USEC,
				EQOS_MAX_RX_COALESCE_USEC);
			return -EINVAL;
		}
		use_riwt = EQOS_COAELSCING_ENABLE;
	}

	/* rx_frames value to be set */
	ret = of_property_read_u32(np, "nvidia,rx_frames", &rx_frames);
	if (ret < 0) {
		use_rx_frames = EQOS_COAELSCING_DISABLE;
	} else {
		if (rx_frames > RX_DESC_CNT ||
		    rx_frames < EQOS_MIN_RX_COALESCE_FRAMES) {
			dev_err(&pdev->dev,
				"invalid rx-frames, must be inrange %d to %d",
				EQOS_MIN_RX_COALESCE_FRAMES, RX_DESC_CNT);
			return -EINVAL;
		}
		use_rx_frames = EQOS_COAELSCING_ENABLE;
	}

	/*  On Rx side we support Rx_usecs and Rx-frames together only  */
	if ((use_riwt == EQOS_COAELSCING_DISABLE &&
	     use_rx_frames == EQOS_COAELSCING_ENABLE) ||
	    (use_riwt == EQOS_COAELSCING_ENABLE &&
	     use_rx_frames == EQOS_COAELSCING_DISABLE)) {
		dev_err(&pdev->dev,
			"invalid settings : rx-frames must be enabled along with use_riwt in DT\n");
		return -EINVAL;
	}

	for (i = 0; i < pdata->num_chans; i++) {
		pdata->tx_queue[i].ptx_ring.tx_coal_frames = tx_frames;
		pdata->tx_queue[i].ptx_ring.use_tx_frames = use_tx_frames;
		pdata->tx_queue[i].ptx_ring.use_tx_usecs = use_tx_usecs;
		pdata->tx_queue[i].ptx_ring.tx_usecs = tx_usecs;
	}
	for (i = 0; i < pdata->num_chans; i++) {
		pdata->rx_queue[i].prx_ring.rx_coal_frames = rx_frames;
		pdata->rx_queue[i].prx_ring.use_riwt = use_riwt;
		pdata->rx_queue[i].prx_ring.rx_riwt = rx_riwt;
	}

	return 0;
}

/*!
* \brief API to initialize the device.
*
* \details This probing function gets called by the kernel when a node with a
* matching compatible string is found in the device tree.
*
* \param[in] pdev - pointer to platform_dev structure.
*
* \return integer
*
* \retval 0 on success & -ve number on failure.
*/

int eqos_probe(struct platform_device *pdev)
{
	struct eqos_prv_data *pdata = NULL;
	struct net_device *ndev = NULL;
	int i, j, ret = 0;
	int irq, power_irq;
	int phyirq;
	int rx_irqs[MAX_CHANS];
	int tx_irqs[MAX_CHANS];
	struct hw_if_struct *hw_if = NULL;
	struct desc_if_struct *desc_if = NULL;
	struct resource *res;
	struct device_attribute *attr = NULL;
	const struct of_device_id *match;
	struct device_node *node = pdev->dev.of_node;
	u8 mac_addr[6];
	struct eqos_cfg *pdt_cfg;
	bool	use_multi_q;
	uint	num_chans, chan;

	pr_debug("-->%s()\n", __func__);

	match = of_match_device(eqos_of_match, &pdev->dev);
	if (!match)
		return -EINVAL;

	/* get base addr */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "eqos_base");

	if (unlikely(res == NULL)) {
		dev_err(&pdev->dev, "invalid resource\n");
		return -EINVAL;
	}

	/* get IRQ */
	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "invalid irq at index 0\n");
		return irq;
	}

	power_irq = platform_get_irq(pdev, 1);
	if (power_irq < 0) {
		dev_err(&pdev->dev, "invalid irq at index 1\n");
		return power_irq;
	}

	for (i = IRQ_CHAN0_RX_IDX, j = 0; i <= IRQ_MAX_IDX; i += 2, j++) {
		rx_irqs[j] = platform_get_irq(pdev, i);
		if (rx_irqs[j] < 0) {
			dev_err(&pdev->dev, "invalid irq at index %d\n", i);
			return rx_irqs[j];
		}

		tx_irqs[j] = platform_get_irq(pdev, i + 1);
		if (tx_irqs[j] < 0) {
			dev_err(&pdev->dev, "invalid irq at index %d\n", i + 1);
			return tx_irqs[j];
		}
	}

#if defined(CONFIG_PHYS_ADDR_T_64BIT)
	pr_debug("res->start = 0x%lx\n", (unsigned long)res->start);
	pr_debug("res->end = 0x%lx\n", (unsigned long)res->end);
#else
	pr_debug("res->start = 0x%x\n", (unsigned int)res->start);
	pr_debug("res->end = 0x%x\n", (unsigned int)res->end);
#endif
	pr_debug("irq = %d\n", irq);
	pr_debug("power_irq = %d\n", power_irq);

	for (j = 0; j < MAX_CHANS; j++)
		pr_debug("rx_irq[%d]=%d, tx_irq[%d]=%d\n",
			j, rx_irqs[j], j, tx_irqs[j]);

	pr_debug("============================================================\n");
	pr_debug("Sizeof tx context desc %lu\n", sizeof(struct s_tx_context_desc));
	pr_debug("Sizeof rx normal desc %lu\n", sizeof(struct s_rx_desc));
	pr_debug("Sizeof tx normal desc %lu\n\n", sizeof(struct s_tx_desc));
	pr_debug("============================================================\n");

	/* remap base address */
	eqos_base_addr = (ULONG) devm_ioremap_nocache(&pdev->dev,
		res->start, (res->end - res->start) + 1);

	/* Set DMA addressing limitations */
	if (dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32))) {
		dev_err(&pdev->dev, "dma_set_mask_and_coherent failed\n");
		goto err_out_dev_failed;
	}

	use_multi_q = of_property_read_bool(node, "nvidia,use_multi_queues");
	if (use_multi_q)
		num_chans = MAX_CHANS;
	else
		num_chans = 1;

	/* allocate and set up the ethernet device */
	ndev = alloc_etherdev_mqs(sizeof(struct eqos_prv_data), num_chans,
				  num_chans);
	if (ndev == NULL) {
		pr_err("%s:Unable to alloc new net device\n",
		    DEV_NAME);
		ret = -ENOMEM;
		goto err_out_dev_failed;
	}

	ndev->base_addr = eqos_base_addr;
	SET_NETDEV_DEV(ndev, &pdev->dev);
	pdata = netdev_priv(ndev);
	eqos_init_all_fptrs(pdata);
	hw_if = &(pdata->hw_if);
	desc_if = &(pdata->desc_if);

	platform_set_drvdata(pdev, ndev);
	pdata->pdev = pdev;

	pdata->dev = ndev;

	for (i = 0; i < num_chans; i++)
		spin_lock_init(&pdata->chan_irq_lock[i]);

	mutex_init(&pdata->hw_change_lock);
	pdata->hw_stopped = true;

	pdata->num_chans = num_chans;
	/* PMT and PHY irqs are shared on FPGA system */
	if (tegra_platform_is_unit_fpga()) {
		phyirq = power_irq;
		/* issue sw reset to device */
		hw_if->exit();
	} else {
		/* regulator init */
		ret = eqos_regulator_init(pdata);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"failed to enable regulator %d\n", ret);
			goto err_out_regulator_en_failed;
		}

		/* On silicon the phy_intr line is handled through a wake
		 * capable GPIO input. DMIC4_CLK is the GPIO input port.
		 */
		phyirq = eqos_get_phyirq_from_gpio(pdata);
		if (phyirq < 0)
			phyirq = PHY_POLL;

		/* read phy delays from DT */
		eqos_get_phy_delays(pdata);

		/* setup PHY reset gpio */
		eqos_get_phyreset_from_gpio(pdata);
		if (gpio_is_valid(pdata->phy_reset_gpio)) {
			/* reset Broadcom PHY needs minimum of 2us delay */
			gpio_set_value(pdata->phy_reset_gpio, 0);
			usleep_range(pdata->phy_reset_duration,
				     pdata->phy_reset_duration + 1);
			gpio_set_value(pdata->phy_reset_gpio, 1);
			msleep(pdata->phy_reset_post_delay);
		}

		/* CAR reset */
		pdata->eqos_rst =
			devm_reset_control_get(&pdev->dev, "eqos_rst");
		if (IS_ERR_OR_NULL(pdata->eqos_rst)) {
			ret = PTR_ERR(pdata->eqos_rst);
			dev_err(&pdev->dev,
				"failed to get eqos reset %d\n", ret);
			goto err_out_reset_get_failed;
		}

		/* clock initialization */
		ret = eqos_get_clocks(pdata);
		if (ret < 0) {
			dev_err(&pdev->dev, "eqos_clock_init failed\n");
			goto err_out_clock_init_failed;
		}

		ret = eqos_clock_enable(pdata);
		if (ret < 0) {
			dev_err(&pdev->dev, "eqos_clock_enable failed\n");
			goto err_out_clock_init_failed;
		}

		/* issue CAR reset to device */
		hw_if->car_reset(pdata);
	}
	pr_debug("phyirq = %d\n", phyirq);

	pdata->prod_list = devm_tegra_prod_get(&pdev->dev);
	if (IS_ERR(pdata->prod_list)) {
		dev_info(&pdev->dev, "No prod values found\n");
		pdata->prod_list = NULL;
	}
	if (pdata->prod_list) {
		if (tegra_prod_set_by_name((void *)&eqos_base_addr, "prod",
					   pdata->prod_list))
			dev_info(&pdev->dev,
				 "fail to enable eqos pad ctrl prod settings\n");
	}
	/* set the pad auto calibration reg settings value as per
	 * recommended values from BUG:3112028
	 */
	ret = of_property_read_u32(node, "nvidia,eqos_auto_cal_config_0_reg",
			&(pdata->dt_cfg.reg_auto_cal_config_0_val));
	if (ret < 0) {
		dev_info(&pdev->dev, "failed to read eqos_auto_cal_config_0_reg\n");
		pdata->dt_cfg.reg_auto_cal_config_0_val = 0x0;
	}

	/* calibrate pad */
	ret = hw_if->pad_calibrate(pdata);
	if (ret < 0)
		goto err_out_pad_calibrate_failed;

#ifdef EQOS_CONFIG_DEBUGFS
	/* to give prv data to debugfs */
	eqos_get_pdata(pdata);
#endif

	ndev->irq = irq;
	pdata->common_irq = irq;

	pdata->power_irq = power_irq;
	pdata->phyirq = phyirq;

	for (j = 0; j < MAX_CHANS; j++) {
		pdata->rx_irqs[j] = rx_irqs[j];
		pdata->tx_irqs[j] = tx_irqs[j];
	}

	/* Get synopsys chip ID */
	pdata->mac_ver = eqos_get_mac_version();

	eqos_get_all_hw_features(pdata);

#ifdef YDEBUG
	eqos_print_all_hw_features(pdata);
#endif

	ret = desc_if->alloc_queue_struct(pdata);
	if (ret < 0) {
		pr_err("ERROR: Unable to alloc Tx/Rx queue\n");
		goto err_out_q_alloc_failed;
	}

	ret = eqos_read_coalesc_params(pdev, pdata, node);
	if (ret) {
		pr_err("Coalescing parameters incorrect\n");
		goto err_out_q_alloc_failed;
	}

	ndev->netdev_ops = eqos_get_netdev_ops();

	pdata->dt_cfg.use_multi_q = use_multi_q;

	pdata->ptp_cfg.use_tagged_ptp = of_property_read_bool(node,
			"nvidia,use_tagged_ptp");
	get_dt_u32(pdata, "nvidia,ptp_dma_ch",
		&(pdata->ptp_cfg.ptp_dma_ch_id),
		PTP_DMA_CH_DEFAULT, PTP_DMA_CH_MAX);

	pdt_cfg = (struct eqos_cfg *)&pdata->dt_cfg;
	get_dt_u32(pdata, "nvidia,pause_frames", &pdt_cfg->pause_frames,
			PAUSE_FRAMES_DEFAULT, PAUSE_FRAMES_MAX);
	get_dt_u32_array(pdata, "nvidia,chan_napi_quota",
			pdt_cfg->chan_napi_quota,
			CHAN_NAPI_QUOTA_DEFAULT, CHAN_NAPI_QUOTA_MAX, 4);
	get_dt_u32_array(pdata, "nvidia,rxq_enable_ctrl", pdt_cfg->rxq_ctrl,
			RXQ_CTRL_DEFAULT, RXQ_CTRL_MAX, 4);
	get_dt_u32_array(pdata, "nvidia,queue_prio", pdt_cfg->q_prio,
			QUEUE_PRIO_DEFAULT, QUEUE_PRIO_MAX, 4);
	get_dt_u32(pdata, "nvidia,iso_bw", &pdt_cfg->iso_bw, ISO_BW_DEFAULT,
		ISO_BW_DEFAULT);
	get_dt_u32(pdata, "nvidia,eth_iso_enable", &pdt_cfg->eth_iso_enable, 0,
		1);
	if (pdata->mac_ver > EQOS_MAC_CORE_4_10)
		get_dt_u32(pdata, "nvidia,slot_intvl_val",
			   &pdt_cfg->slot_intvl_val,
			   SLOT_INTVL_DEFAULT, SLOT_INTVL_MAX);
	pdata->dt_cfg.phy_apd_mode = of_property_read_bool(node,
							   "nvidia,brcm_phy_apd_mode");
	eqos_get_slot_num_check_queues(pdata, pdt_cfg->slot_num_check);

#ifndef DISABLE_TRISTATE
	/* enable tx tri state to save power during init */
	ret = pinctrl_pm_select_idle_state(&pdev->dev);
	if (ret < 0)
		dev_err(&pdev->dev, "enable txrx trisate failed(%d)\n", ret);
#endif
	pdata->rx_buffer_len = EQOS_RX_BUF_LEN;
	pdata->rx_max_frame_size = EQOS_MAX_ETH_FRAME_LEN_DEFAULT;

	/* csr_clock_speed is axi_cbb_clk rate */
	pdata->csr_clock_speed = clk_get_rate(pdata->axi_cbb_clk) / 1000000;
	if (pdata->csr_clock_speed <= 0) {
		dev_err(&pdev->dev, "fail to read axi_cbb_clk rate\n");
	} else {
		pr_debug("setting MAC_1US_TIC to %d MHz\n",
			pdata->csr_clock_speed);
			MAC_1US_TIC_WR(pdata->csr_clock_speed - 1);
	}
	save_mdc(pdata);

	if (eqos_set_ptp_ref_clk(pdata, node)) {
		dev_err(&pdev->dev, "failed to set PTP ref clk\n");
		goto err_out_q_alloc_failed;
	}

	ret = eqos_get_mac_address_dtb("/chosen", "nvidia,ether-mac", mac_addr);
	if (ret < 0) {
		netdev_err(ndev, "ether-mac read from DT failed %d\n", ret);
		eth_hw_addr_random(ndev);
		netdev_info(ndev, "EQOS using random MAC address: %pM\n",
			    ndev->dev_addr);
	} else {
		dev_info(&pdev->dev, "Setting local MAC: %x %x %x %x %x %x\n",
			mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3],
			mac_addr[4], mac_addr[5]);
		/* Set up MAC address */
		ndev->dev_addr[0] = mac_addr[0];
		ndev->dev_addr[1] = mac_addr[1];
		ndev->dev_addr[2] = mac_addr[2];
		ndev->dev_addr[3] = mac_addr[3];
		ndev->dev_addr[4] = mac_addr[4];
		ndev->dev_addr[5] = mac_addr[5];
	}
	pdata->interface = of_get_phy_mode(node);

	pdata->phy_node = of_parse_phandle(node, "phy-handle", 0);
	if (!pdata->phy_node)
		pr_debug("%s(): phy handle not found\n", __func__);

	/* If nvidia,eqos-mdio is passed from DT, always register the MDIO */
	for_each_child_of_node(node, pdata->mdio_node) {
		if (of_device_is_compatible(pdata->mdio_node,
					    "nvidia,eqos-mdio"))
			break;
	}

	/* In the case of a fixed PHY, the DT node associated
	 * to the PHY is the Ethernet MAC DT node.
	 */
	if (!pdata->phy_node && of_phy_is_fixed_link(node)) {
		if ((of_phy_register_fixed_link(node) < 0)) {
			netdev_err(ndev, "Failed to register fixed PHY device\n");
			ret = ENODEV;
			goto err_out_q_alloc_failed;
		}
		pdata->phy_node = of_node_get(node);
	}

	if (pdata->mdio_node) {
		ret = eqos_mdio_register(ndev);
		if (ret < 0) {
			pr_err("MDIO bus (id %d) registration failed\n",
			       pdata->bus_id);
			goto err_out_mdio_reg;
		}
	}

	for (i = 0; i < pdata->num_chans; i++) {
		pdata->rx_queue[i].pdata = pdata;
		pdata->rx_queue[i].chan_num = i;
		netif_napi_add(ndev, &pdata->rx_queue[i].napi,
			       eqos_napi_poll_rx, pdt_cfg->chan_napi_quota[i]);

		pdata->tx_queue[i].pdata = pdata;
		pdata->tx_queue[i].chan_num = i;
		netif_napi_add(ndev, &pdata->tx_queue[i].napi,
			       eqos_napi_poll_tx, pdt_cfg->chan_napi_quota[i]);
	}

	/* Setup the tx_usecs timer */
	for (i = 0; i < pdata->num_chans; i++) {
		chan = pdata->tx_queue[i].chan_num;
		atomic_set(&pdata->tx_queue[chan].tx_usecs_timer_armed,
			   EQOS_HRTIMER_DISABLE);
		hrtimer_init(&pdata->tx_queue[chan].tx_usecs_timer,
			     CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		pdata->tx_queue[chan].tx_usecs_timer.function =
					eqos_tx_usecs_hrtimer;
	}
	ndev->ethtool_ops = (eqos_get_ethtool_ops());

	if (pdata->hw_feat.tso_en) {
		ndev->hw_features = NETIF_F_TSO;
		ndev->hw_features |= NETIF_F_SG;
		ndev->hw_features |= NETIF_F_IP_CSUM;
		ndev->hw_features |= NETIF_F_IPV6_CSUM;
	} else if (pdata->hw_feat.tx_coe_sel) {
		ndev->hw_features = NETIF_F_IP_CSUM;
		ndev->hw_features |= NETIF_F_IPV6_CSUM;
	}

	ndev->hw_features |= NETIF_F_RXCSUM;
	ndev->hw_features |= NETIF_F_GRO;

#ifdef EQOS_ENABLE_VLAN_TAG
	ndev->vlan_features |= ndev->hw_features;
	ndev->hw_features |= NETIF_F_HW_VLAN_CTAG_RX;
	if (pdata->hw_feat.sa_vlan_ins)
		ndev->hw_features |= NETIF_F_HW_VLAN_CTAG_TX;
	if (pdata->hw_feat.vlan_hash_en)
		ndev->hw_features |= NETIF_F_HW_VLAN_CTAG_FILTER;
#endif /* end of EQOS_ENABLE_VLAN_TAG */
	ndev->features |= ndev->hw_features;
	pdata->dev_state |= ndev->features;

	eqos_init_rx_coalesce(pdata);

	for (i = 0; i < ARRAY_SIZE(eqos_sysfs_attrs); i++) {
		attr = eqos_sysfs_attrs[i];
		ret = device_create_file(&pdata->pdev->dev, attr);

		if (ret) {
			dev_err(&pdata->pdev->dev,
				"Failed to register attributes: %d\n", ret);
			goto err_sysfs_create_failed;
			}
	}

	spin_lock_init(&pdata->lock);
	spin_lock_init(&pdata->tx_lock);
	spin_lock_init(&pdata->pmt_lock);

	ret = register_netdev(ndev);
	if (ret) {
		pr_err("%s: Net device registration failed\n",
		    DEV_NAME);
		goto err_out_netdev_failed;
	}

	pr_debug("<-- eqos_probe\n");

	netif_carrier_off(ndev);

	if (tegra_platform_is_unit_fpga()) {
		ret = request_irq(power_irq, EQOS_ISR_SW_EQOS_POWER,
			IRQF_SHARED, DEV_NAME, pdata);

		if (ret != 0) {
			pr_err("Unable to register PMT IRQ %d\n", power_irq);
			ret = -EBUSY;
			goto err_out_pmt_irq_failed;
		}
	}

	INIT_WORK(&pdata->fbe_work, eqos_fbe_work);
	INIT_WORK(&pdata->iso_work, eqos_iso_work);

	if (pdt_cfg->eth_iso_enable) {
		/* Bandwidth Negotiation call back is NULL as of now */
		pdata->isomgr_handle =
			tegra_isomgr_register(TEGRA_ISO_CLIENT_EQOS,
					pdata->dt_cfg.iso_bw, NULL, NULL);
		if (!pdata->isomgr_handle) {
			dev_err(&pdata->pdev->dev, "EQOS ISO Bandwidth allocation failed\n");
			ret = -EIO;
			goto err_isomgr_reg_failed;
		}
	}

	/* register cooling device */
	ret = eqos_therm_init(pdata);
	if (ret != 0) {
		pr_err("unable to register cooling device err: %d\n", ret);
		goto err_therm_init;
	}

	eqos_clock_disable(pdata);

	/* put Ethernet PHY in reset for power save */
	if (gpio_is_valid(pdata->phy_reset_gpio) &&
	    (pdata->mac_ver > EQOS_MAC_CORE_4_10))
		gpio_set_value(pdata->phy_reset_gpio, 0);

	return 0;

 err_therm_init:
	if (pdt_cfg->eth_iso_enable)
		tegra_isomgr_unregister(pdata->isomgr_handle);

 err_isomgr_reg_failed:
	if ((tegra_platform_is_unit_fpga()) &&
		(pdata->power_irq != 0)) {
		free_irq(pdata->power_irq, pdata);
		pdata->power_irq = 0;
	}

 err_out_pmt_irq_failed:
	unregister_netdev(ndev);

 err_out_netdev_failed:
	/* remove rx napi */
	for (i = 0; i < EQOS_RX_QUEUE_CNT; i++) {
		netif_napi_del(&pdata->rx_queue[i].napi);
		netif_napi_del(&pdata->tx_queue[i].napi);
	}
	if (1 == pdata->hw_feat.sma_sel)
		eqos_mdio_unregister(ndev);

 err_sysfs_create_failed:
 err_out_mdio_reg:
	desc_if->free_queue_struct(pdata);

 err_out_q_alloc_failed:
 err_out_pad_calibrate_failed:
	if (!tegra_platform_is_unit_fpga())
		eqos_clock_deinit(pdata);
 err_out_clock_init_failed:
	if (!tegra_platform_is_unit_fpga() &&
		!IS_ERR_OR_NULL(pdata->eqos_rst)) {
		reset_control_assert(pdata->eqos_rst);
	}
 err_out_reset_get_failed:
	if (!tegra_platform_is_unit_fpga())
		eqos_regulator_deinit(pdata);
 err_out_regulator_en_failed:
	free_netdev(ndev);
	platform_set_drvdata(pdev, NULL);

 err_out_dev_failed:
	devm_iounmap(&pdev->dev, (void *) eqos_base_addr);

	return ret;
}

/*!
* \brief API to release all the resources from the driver.
*
* \details The remove function gets called whenever a device being handled
* by this driver is removed (either during deregistration of the driver or
* when it is manually pulled out of a hot-pluggable slot). This function
* should reverse operations performed at probe time. The remove function
* always gets called from process context, so it can sleep.
*
* \param[in] pdev - pointer to platform_dev structure.
*
* \return void
*/

int eqos_remove(struct platform_device *pdev)
{
	struct net_device *ndev;
	struct eqos_prv_data *pdata;
	struct desc_if_struct *desc_if;
	int i, ret_val = 0;
	struct eqos_cfg *pdt_cfg;

	pr_debug("--> eqos_remove\n");

	if (pdev == NULL) {
		pr_debug("Remove called on invalid device\n");
		return -1;
	}

	ndev = platform_get_drvdata(pdev);
	pdata = netdev_priv(ndev);
	pdt_cfg = (struct eqos_cfg *)&pdata->dt_cfg;
	desc_if = &(pdata->desc_if);

#ifdef CONFIG_THERMAL
	thermal_cooling_device_unregister(pdata->cdev);
#endif

	/* free tx skb's */
	desc_if->tx_skb_free_mem(pdata, EQOS_TX_QUEUE_CNT);

	if ((tegra_platform_is_unit_fpga()) &&
	    (pdata->power_irq != 0)) {
		free_irq(pdata->power_irq, pdata);
		pdata->power_irq = 0;
	}

	if (pdt_cfg->eth_iso_enable)
		tegra_isomgr_unregister(pdata->isomgr_handle);

	unregister_netdev(ndev);

#ifdef EQOS_CONFIG_PTP
	eqos_ptp_remove(pdata);
#endif	/* end of EQOS_CONFIG_PTP */

	/* remove rx napi */
	for (i = 0; i < EQOS_RX_QUEUE_CNT; i++) {
		netif_napi_del(&pdata->rx_queue[i].napi);
		netif_napi_del(&pdata->tx_queue[i].napi);
	}

	if (1 == pdata->hw_feat.sma_sel)
		eqos_mdio_unregister(ndev);

	desc_if->free_queue_struct(pdata);

	if (!tegra_platform_is_unit_fpga()) {
		eqos_clock_deinit(pdata);

		if (!IS_ERR_OR_NULL(pdata->eqos_rst))
			reset_control_assert(pdata->eqos_rst);
		devm_gpio_free(&pdev->dev, pdata->phy_reset_gpio);
		devm_gpio_free(&pdev->dev, pdata->phy_intr_gpio);
		eqos_regulator_deinit(pdata);
	}

	free_netdev(ndev);

	platform_set_drvdata(pdev, NULL);

	devm_iounmap(&pdev->dev, (void *) eqos_base_addr);

	pr_debug("<-- eqos_remove\n");

	return ret_val;
}

#ifdef CONFIG_PM
static int eqos_suspend_noirq(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct eqos_prv_data *pdata = netdev_priv(ndev);

	if (!netif_running(ndev))
		return 0;

	if (pdata->suspended) {
		pr_err("eqos already suspended\n");
		return -EINVAL;
	}

	pdata->suspended = 1;

	if (pdata->phydev) {
		if (device_may_wakeup(&ndev->dev)) {
			enable_irq_wake(pdata->phydev->irq);
		} else {
			phy_stop(pdata->phydev);
			if ((gpio_is_valid(pdata->phy_reset_gpio) &&
			    (pdata->mac_ver > EQOS_MAC_CORE_4_10))) {
				gpio_set_value(pdata->phy_reset_gpio, 0);
				usleep_range(pdata->phy_reset_duration,
					     pdata->phy_reset_duration + 1);
			}
		}
	}

	eqos_stop_dev(pdata);

	pdata->hw_stopped = true;

	/* Assert MAC RST gpio */
	if (pdata->eqos_rst)
		reset_control_assert(pdata->eqos_rst);

	/* cancel iso work */
	cancel_work_sync(&pdata->iso_work);
	/* Cancel FBE handling work */
	cancel_work_sync(&pdata->fbe_work);
	/* disable clocks */
	eqos_clock_disable(pdata);
	/* disable regulators */
	eqos_regulator_deinit(pdata);

	return 0;
}

static int eqos_resume_noirq(struct device *dev)
{
	struct net_device *ndev = dev_get_drvdata(dev);
	struct eqos_prv_data *pdata = netdev_priv(ndev);
	struct hw_if_struct *hw_if = &pdata->hw_if;
	int ret = Y_SUCCESS;

	if (!netif_running(ndev))
		return 0;

	if (!pdata->suspended) {
		pr_err("eqos already resumed\n");
		return -EINVAL;
	}

	/* start regulators */
	eqos_regulator_init(pdata);

	/* enable clocks */
	ret = eqos_clock_enable(pdata);
	if (ret < 0) {
		dev_err(&pdata->pdev->dev, "resume clocks not enabled\n");
		return -EINVAL;
	}

	if (device_may_wakeup(&ndev->dev)) {
		disable_irq_wake(pdata->phydev->irq);
		/* issue CAR reset to device */
		ret = hw_if->car_reset(pdata);
		if (ret < 0) {
			dev_err(&pdata->pdev->dev, "WoL Failed to reset MAC\n");
			return -ENODEV;
		}
		eqos_start_dev(pdata);
	} else {
		if (gpio_is_valid(pdata->phy_reset_gpio)) {
			/* reset Broadcom PHY needs minimum of 2us delay */
			gpio_set_value(pdata->phy_reset_gpio, 0);
			usleep_range(10, 11);
			gpio_set_value(pdata->phy_reset_gpio, 1);
			msleep(pdata->phy_reset_post_delay);
		}

		/* issue CAR reset to device */
		ret = hw_if->car_reset(pdata);
		if (ret < 0) {
			dev_err(&pdata->pdev->dev, "Failed to reset MAC\n");
			return -ENODEV;
		}

		eqos_start_dev(pdata);

		/* Init the PHY */
		pdata->phydev->drv->config_init(pdata->phydev);
		phy_start(pdata->phydev);
	}

	netif_tx_start_all_queues(pdata->dev);

	pdata->suspended = 0;
	pdata->hw_stopped = false;

	return 0;
}

static const struct dev_pm_ops eqos_pm_ops = {
	.suspend_noirq = eqos_suspend_noirq,
	.resume_noirq = eqos_resume_noirq,
};
#endif

static struct platform_driver eqos_driver = {
	.probe = eqos_probe,
	.remove = eqos_remove,
	.driver = {
		.name = DEV_NAME,
		.owner = THIS_MODULE,
#ifdef CONFIG_PM
		.pm    = &eqos_pm_ops,
#endif
#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 9, 0)
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
#endif
		.of_match_table = eqos_of_match,
	},
};

/*!
* \brief API to register the driver.
*
* \details This is the first function called when the driver is loaded.
* It register the driver with the kernel
*
* \return void.
*/

static int eqos_init_module(void)
{
	INT ret = 0;

	pr_debug("-->eqos_init_module\n");

	ret = platform_driver_register(&eqos_driver);
	if (ret < 0) {
		pr_debug("eqos:driver registration failed\n");
		return ret;
	}
	pr_debug("eqos:driver registration sucessful\n");

#ifdef EQOS_CONFIG_DEBUGFS
	create_debug_files();
#endif

	pr_debug("<--eqos_init_module\n");

	return ret;
}

/*!
* \brief API to unregister the driver.
*
* \details This is the first function called when the driver is removed.
* It unregister the driver from the kernel
*
* \return void.
*/

static void __exit eqos_exit_module(void)
{
	pr_debug("-->eqos_exit_module\n");

#ifdef EQOS_CONFIG_DEBUGFS
	remove_debug_files();
#endif

	platform_driver_unregister(&eqos_driver);

	pr_debug("<--eqos_exit_module\n");
}

/*!
* \brief Macro to register the driver registration function.
*
* \details A module always begin with either the init_module or the function
* you specify with module_init call. This is the entry function for modules;
* it tells the kernel what functionality the module provides and sets up the
* kernel to run the module's functions when they're needed. Once it does this,
* entry function returns and the module does nothing until the kernel wants
* to do something with the code that the module provides.
*/
module_init(eqos_init_module);

/*!
* \brief Macro to register the driver un-registration function.
*
* \details All modules end by calling either cleanup_module or the function
* you specify with the module_exit call. This is the exit function for modules;
* it undoes whatever entry function did. It unregisters the functionality
* that the entry function registered.
*/
module_exit(eqos_exit_module);

/*!
* \brief Macro to declare the module author.
*
* \details This macro is used to declare the module's authore.
*/
MODULE_AUTHOR("Synopsys India Pvt Ltd");

/*!
* \brief Macro to describe what the module does.
*
* \details This macro is used to describe what the module does.
*/
MODULE_DESCRIPTION("eqos Driver");

/*!
* \brief Macro to describe the module license.
*
* \details This macro is used to describe the module license.
*/
MODULE_LICENSE("GPL");
