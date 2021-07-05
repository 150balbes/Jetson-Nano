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

/*!@file: eqos_mdio.c
 * @brief: Driver functions.
 */
#include "yheader.h"
#include <linux/tegra_prod.h>
#include <linux/brcmphy.h>

/*!
* \brief read MII PHY register, function called by the driver alone
*
* \details Read MII registers through the API read_phy_reg where the
* related MAC registers can be configured.
*
* \param[in] pdata - pointer to driver private data structure.
* \param[in] phyaddr - the phy address to read
* \param[in] phyreg - the phy regiester id to read
* \param[out] phydata - pointer to the value that is read from the phy registers
*
* \return int
*
* \retval  0 - successfully read data from register
* \retval -1 - error occurred
* \retval  1 - if the feature is not defined.
*/

INT eqos_mdio_read_direct(struct eqos_prv_data *pdata,
			  int phyaddr, int phyreg, int *phydata)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	int phy_reg_read_status;

	DBGPR_MDIO("--> eqos_mdio_read_direct\n");
	if (pdata->suspended)
		return -ENODEV;

	if (hw_if->read_phy_regs) {
		phy_reg_read_status =
		    hw_if->read_phy_regs(phyaddr, phyreg, phydata,
			pdata->mdc_cr);
	} else {
		phy_reg_read_status = 1;
		pr_err("%s: hw_if->read_phy_regs not defined", DEV_NAME);
	}

	DBGPR_MDIO("<-- eqos_mdio_read_direct\n");

	return phy_reg_read_status;
}

/*!
* \brief write MII PHY register, function called by the driver alone
*
* \details Writes MII registers through the API write_phy_reg where the
* related MAC registers can be configured.
*
* \param[in] pdata - pointer to driver private data structure.
* \param[in] phyaddr - the phy address to write
* \param[in] phyreg - the phy regiester id to write
* \param[out] phydata - actual data to be written into the phy registers
*
* \return void
*
* \retval  0 - successfully read data from register
* \retval -1 - error occurred
* \retval  1 - if the feature is not defined.
*/

INT eqos_mdio_write_direct(struct eqos_prv_data *pdata,
			   int phyaddr, int phyreg, int phydata)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	int phy_reg_write_status;

	if (pdata->suspended)
		return -ENODEV;

	DBGPR_MDIO("--> eqos_mdio_write_direct\n");

	if (hw_if->write_phy_regs) {
		phy_reg_write_status =
		    hw_if->write_phy_regs(phyaddr, phyreg,
			phydata, pdata->mdc_cr);
	} else {
		phy_reg_write_status = 1;
		pr_err("%s: hw_if->write_phy_regs not defined", DEV_NAME);
	}

	DBGPR_MDIO("<-- eqos_mdio_write_direct\n");

	return phy_reg_write_status;
}

/*!
* \brief read MII PHY register.
*
* \details Read MII registers through the API read_phy_reg where the
* related MAC registers can be configured.
*
* \param[in] bus - points to the mii_bus structure
* \param[in] phyaddr - the phy address to write
* \param[in] phyreg - the phy register offset to write
*
* \return int
*
* \retval  - value read from given phy register
*/

static INT eqos_mdio_read(struct mii_bus *bus, int phyaddr, int phyreg)
{
	struct net_device *dev = bus->priv;
	struct eqos_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	int phydata = 0;

	if (!pdata->clks_enable) {
		pr_err("%s:No clks available, skipping PHY read\n", __func__);
		return -ENODEV;
	}

	DBGPR_MDIO("--> eqos_mdio_read: phyaddr = %d, phyreg = %d\n",
		   phyaddr, phyreg);

	if (hw_if->read_phy_regs)
		hw_if->read_phy_regs(phyaddr, phyreg, &phydata,
			pdata->mdc_cr);
	else
		pr_err("%s: hw_if->read_phy_regs not defined", DEV_NAME);

	DBGPR_MDIO("<-- eqos_mdio_read: phydata = %#x\n", phydata);

	return phydata;
}

/*!
* \brief API to write MII PHY register
*
* \details This API is expected to write MII registers with the value being
* passed as the last argument which is done in write_phy_regs API
* called by this function.
*
* \param[in] bus - points to the mii_bus structure
* \param[in] phyaddr - the phy address to write
* \param[in] phyreg - the phy register offset to write
* \param[in] phydata - the register value to write with
*
* \return 0 on success and -ve number on failure.
*/

static INT eqos_mdio_write(struct mii_bus *bus, int phyaddr, int phyreg,
			   u16 phydata)
{
	struct net_device *dev = bus->priv;
	struct eqos_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	INT ret = Y_SUCCESS;

	if (!pdata->clks_enable) {
		pr_err("%s:No clks available, skipping PHY write\n", __func__);
		return -ENODEV;
	}

	DBGPR_MDIO("--> eqos_mdio_write\n");

	if (hw_if->write_phy_regs) {
		hw_if->write_phy_regs(phyaddr, phyreg, phydata,
			pdata->mdc_cr);
	} else {
		ret = -1;
		pr_err("%s: hw_if->write_phy_regs not defined", DEV_NAME);
	}

	DBGPR_MDIO("<-- eqos_mdio_write\n");

	return ret;
}

#ifdef YDEBUG
/*!
 * \details This function is invoked by other functions to get the PHY register
 * dump. This function is used during development phase for debug purpose.
 *
 * \param[in] pdata – pointer to private data structure.
 *
 * \return 0
 */

void dump_phy_registers(struct eqos_prv_data *pdata)
{
	int phydata = 0;

	pr_err("\n************* PHY Reg dump *************************\n");
	eqos_mdio_read_direct(pdata, pdata->phyaddr, MII_BMCR, &phydata);
	pr_err("Phy Control Reg(Basic Mode Control Reg) (%#x) = %#x\n",
	       MII_BMCR, phydata);

	eqos_mdio_read_direct(pdata, pdata->phyaddr, MII_BMSR, &phydata);
	pr_err("Phy Status Reg(Basic Mode Status Reg) (%#x) = %#x\n",
	       MII_BMSR, phydata);

	eqos_mdio_read_direct(pdata, pdata->phyaddr, MII_PHYSID1, &phydata);
	pr_err("Phy Id (PHYS ID 1) (%#x)= %#x\n", MII_PHYSID1, phydata);

	eqos_mdio_read_direct(pdata, pdata->phyaddr, MII_PHYSID2, &phydata);
	pr_err("Phy Id (PHYS ID 2) (%#x)= %#x\n", MII_PHYSID2, phydata);

	eqos_mdio_read_direct(pdata, pdata->phyaddr, MII_ADVERTISE, &phydata);
	pr_err("Auto-nego Adv (Advertisement Control Reg) (%#x) = %#x\n",
	       MII_ADVERTISE, phydata);

	/* read Phy Control Reg */
	eqos_mdio_read_direct(pdata, pdata->phyaddr, MII_LPA, &phydata);
	pr_err("Auto-nego Lap (Link Partner Ability Reg) (%#x)= %#x\n",
	       MII_LPA, phydata);

	eqos_mdio_read_direct(pdata, pdata->phyaddr, MII_EXPANSION, &phydata);
	pr_err("Auto-nego Exp (Extension Reg)(%#x) = %#x\n",
	       MII_EXPANSION, phydata);

	eqos_mdio_read_direct(pdata, pdata->phyaddr,
			      EQOS_AUTO_NEGO_NP, &phydata);
	pr_err("Auto-nego Np (%#x) = %#x\n", EQOS_AUTO_NEGO_NP, phydata);

	eqos_mdio_read_direct(pdata, pdata->phyaddr, MII_ESTATUS, &phydata);
	pr_err("Extended Status Reg (%#x) = %#x\n", MII_ESTATUS, phydata);

	eqos_mdio_read_direct(pdata, pdata->phyaddr, MII_CTRL1000, &phydata);

	pr_err("1000 Ctl Reg (1000BASE-T Control Reg)(%#x) = %#x\n",
	       MII_CTRL1000, phydata);

	eqos_mdio_read_direct(pdata, pdata->phyaddr, MII_STAT1000, &phydata);
	pr_err("1000 Sts Reg (1000BASE-T Status)(%#x) = %#x\n",
	       MII_STAT1000, phydata);

	eqos_mdio_read_direct(pdata, pdata->phyaddr, EQOS_PHY_CTL, &phydata);
	pr_err("PHY Ctl Reg (%#x) = %#x\n", EQOS_PHY_CTL, phydata);

	eqos_mdio_read_direct(pdata, pdata->phyaddr, EQOS_PHY_STS, &phydata);
	pr_err("PHY Sts Reg (%#x) = %#x\n", EQOS_PHY_STS, phydata);

	pr_err("\n****************************************************\n");
}
#endif

/*!
* \brief API to adjust link parameters.
*
* \details This function will be called by PAL to inform the driver
* about various link parameters like duplex and speed. This function
* will configure the MAC based on link parameters.
*
* \param[in] dev - pointer to net_device structure
*
* \return void
*/

static void eqos_adjust_link(struct net_device *dev)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct phy_device *phydev = pdata->phydev;
	int new_state = 0, speed_changed = 0, tx_tristate_disable = 0;
#ifndef DISABLE_TRISTATE
	int ret = 0;
#endif

	if (phydev == NULL)
		return;

	DBGPR_MDIO("-->eqos_adjust_link. address %d link %d\n",
		   phydev->mdio.addr, phydev->link);

	spin_lock_bh(&pdata->lock);

	if (phydev->link) {
		/* Now we make sure that we can be in full duplex mode.
		 * If not, we operate in half-duplex mode
		 */
		if (phydev->duplex != pdata->oldduplex) {
			new_state = 1;
			if (phydev->duplex)
				hw_if->set_full_duplex();
			else
				hw_if->set_half_duplex(pdata);
			pdata->oldduplex = phydev->duplex;
		}

		/* FLOW ctrl operation */
		if (pdata->dt_cfg.pause_frames == PAUSE_FRAMES_ENABLED)
			if (phydev->pause || phydev->asym_pause) {
				if (pdata->flow_ctrl != pdata->oldflow_ctrl)
					eqos_configure_flow_ctrl(pdata);
			}

		if (phydev->speed != pdata->speed) {
			new_state = 1;
			speed_changed = 1;
			switch (phydev->speed) {
			case SPEED_1000:
				hw_if->set_gmii_speed(pdata);
				break;
			case SPEED_100:
				hw_if->set_mii_speed_100(pdata);
				break;
			case SPEED_10:
				hw_if->set_mii_speed_10(pdata);
				/* disable auto calibration only for MAC4.10 */
				if (pdata->mac_ver == EQOS_MAC_CORE_4_10)
					hw_if->disable_pad_cal(pdata);
				break;
			}
			pdata->speed = phydev->speed;
		}

		if (!pdata->oldlink) {
			new_state = 1;
			pdata->oldlink = 1;
			pdata->xstats.link_connect_count++;
#ifndef DISABLE_TRISTATE
			ret = pinctrl_pm_select_default_state(
							&pdata->pdev->dev);
			if (ret < 0) {
				dev_err(&pdata->pdev->dev,
					"disable txrx tristate failed\n");
			}
			tx_tristate_disable = 1;
#endif
			schedule_work(&pdata->iso_work);
		}
	} else if (pdata->oldlink) {
		new_state = 1;
		pdata->oldlink = 0;
		pdata->speed = 0;
		pdata->oldduplex = -1;
		pdata->xstats.link_disconnect_count++;
#ifndef DISABLE_TRISTATE
		ret = pinctrl_pm_select_idle_state(&pdata->pdev->dev);
		if (ret < 0) {
			dev_err(&pdata->pdev->dev,
				"enable txrx tristate failed\n");
		}
#endif
		schedule_work(&pdata->iso_work);
	}

	if (new_state)
		phy_print_status(phydev);

	spin_unlock_bh(&pdata->lock);

	/* At this stage, it could be need to setup the EEE or adjust some
	 * MAC related HW registers.
	 */
#ifdef EQOS_ENABLE_EEE
	pdata->eee_enabled = eqos_eee_init(pdata);
#endif

	if (speed_changed) {
		hw_if->set_tx_clk_speed(pdata, phydev->speed);
		/* recalibrate if speed 10 to 100 or 1000mbps */
		if (pdata->oldspeed == SPEED_10)
			hw_if->pad_calibrate(pdata);
		pdata->oldspeed = pdata->speed;
	}
	if (tx_tristate_disable) {
		/* recalibrate once we disable tristate*/
		hw_if->pad_calibrate(pdata);
	}


	DBGPR_MDIO("<--eqos_adjust_link\n");
}

/*!
* \brief API to initialize PHY.
*
* \details This function will initializes the driver's PHY state and attaches
* the PHY to the MAC driver.
*
* \param[in] dev - pointer to net_device structure
*
* \return integer
*
* \retval 0 on success & negative number on failure.
*/

int eqos_init_phy(struct net_device *dev)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);
	struct phy_device *phydev = NULL;
	char phy_id_fmt[MII_BUS_ID_SIZE + 3];
	char bus_id[MII_BUS_ID_SIZE];

	DBGPR_MDIO("-->eqos_init_phy\n");

	pdata->oldlink = false;
	pdata->speed = SPEED_UNKNOWN;
	pdata->oldduplex = SPEED_UNKNOWN;

	if (pdata->phy_node)
		phydev = of_phy_connect(dev, pdata->phy_node,
					&eqos_adjust_link, 0, pdata->interface);
	else {
		snprintf(bus_id, MII_BUS_ID_SIZE, "dwc_phy-%x", pdata->bus_id);

		snprintf(phy_id_fmt, MII_BUS_ID_SIZE + 3, PHY_ID_FMT, bus_id,
			 pdata->phyaddr);

		phydev = phy_connect(dev, phy_id_fmt, &eqos_adjust_link,
				     pdata->interface);
	}

	if (!phydev) {
		pr_err("%s: Could not attach to PHY\n", dev->name);
		return -ENODEV;
	}

	if (!pdata->phy_node && phydev->phy_id == 0) {
		phy_disconnect(phydev);
		return -ENODEV;
	}

	if ((pdata->interface == PHY_INTERFACE_MODE_GMII) ||
	    (pdata->interface == PHY_INTERFACE_MODE_RGMII)) {
		phydev->supported = PHY_GBIT_FEATURES;
	} else if ((pdata->interface == PHY_INTERFACE_MODE_MII) ||
		   (pdata->interface == PHY_INTERFACE_MODE_RMII)) {
		phydev->supported = PHY_BASIC_FEATURES;
	}

	/* if multi channels are enabled, disable half duplex
	 * advertisement
	 */
	if (pdata->dt_cfg.use_multi_q) {
		phydev->supported &= ~(SUPPORTED_1000baseT_Half |
			SUPPORTED_100baseT_Half | SUPPORTED_10baseT_Half);
	}

	phydev->supported |= (SUPPORTED_Pause | SUPPORTED_Asym_Pause);
	if (pdata->dt_cfg.pause_frames == PAUSE_FRAMES_DISABLED)
		phydev->supported &= ~(SUPPORTED_Pause | SUPPORTED_Asym_Pause);

	phydev->advertising = phydev->supported;

	if (pdata->dt_cfg.phy_apd_mode &&
	    (phydev->drv->phy_id & phydev->drv->phy_id_mask) == PHY_ID_BCM89610)
		phydev->dev_flags |= PHY_BRCM_AUTO_PWRDWN_ENABLE;

	DBGPR_MDIO("%s: attached to PHY (UID 0x%x) Link = %d\n", dev->name,
		   phydev->phy_id, phydev->link);

	pdata->phydev = phydev;

	DBGPR_MDIO("<--eqos_init_phy\n");

	return 0;
}

/*!
* \brief API to register mdio.
*
* \details This function will allocate mdio bus and register it
* phy layer.
*
* \param[in] dev - pointer to net_device structure
*
* \return 0 on success and -ve on failure.
*/

int eqos_mdio_register(struct net_device *dev)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);
	struct mii_bus *new_bus = NULL;
	int phyaddr = 0;
	int ret = Y_SUCCESS, i;

	DBGPR_MDIO("-->eqos_mdio_register\n");

	pdata->phyaddr = phyaddr;
	pdata->bus_id = 0x1;

	new_bus = devm_mdiobus_alloc(&pdata->pdev->dev);
	if (!new_bus) {
		netdev_err(dev, "failed to allocate MDIO bus\n");
		return -ENOMEM;
	}

	new_bus->name = "dwc_phy";
	new_bus->read = eqos_mdio_read;
	new_bus->write = eqos_mdio_write;
	snprintf(new_bus->id, MII_BUS_ID_SIZE, "%s-%x", new_bus->name,
		 pdata->bus_id);
	new_bus->priv = dev;
	new_bus->phy_mask = ~(1 << pdata->phyaddr);
	new_bus->parent = &pdata->pdev->dev;

	for (i = 0; i < PHY_MAX_ADDR; ++i) {
		if (i == pdata->phyaddr)
			new_bus->irq[i] = pdata->phyirq;
		else
			new_bus->irq[i] = PHY_POLL;
	}

	if (pdata->mdio_node)
		ret = of_mdiobus_register(new_bus, pdata->mdio_node);
	else
		ret = mdiobus_register(new_bus);

	if (ret) {
		netdev_err(dev, "%s: failed to register MDIO bus\n", new_bus->name);
		return ret;
	}

	pdata->mii = new_bus;

	DBGPR_MDIO("<--eqos_mdio_register\n");

	return ret;
}

/*!
* \brief API to unregister mdio.
*
* \details This function will unregister mdio bus and free's the memory
* allocated to it.
*
* \param[in] dev - pointer to net_device structure
*
* \return void
*/

void eqos_mdio_unregister(struct net_device *dev)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);

	DBGPR_MDIO("-->eqos_mdio_unregister\n");

	if (pdata->phydev) {
		phy_stop(pdata->phydev);
		phy_disconnect(pdata->phydev);
		pdata->phydev = NULL;
	}

	mdiobus_unregister(pdata->mii);
	pdata->mii->priv = NULL;
	pdata->mii = NULL;

	DBGPR_MDIO("<--eqos_mdio_unregister\n");
}
