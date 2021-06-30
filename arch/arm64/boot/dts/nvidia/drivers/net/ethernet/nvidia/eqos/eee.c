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
 * Copyright (c) 2015, NVIDIA CORPORATION.  All rights reserved.
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
/*!@file: eqos_eee.c
 * @brief: Driver functions.
 */
#include "yheader.h"
#include <linux/brcmphy.h>

void eqos_enable_eee_mode(struct eqos_prv_data *pdata)
{
	struct tx_ring *ptx_ring = NULL;
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	int tx_idle = 0, qinx;

	DBGPR_EEE("-->eqos_enable_eee_mode\n");

	for (qinx = 0; qinx < pdata->num_chans; qinx++) {
		ptx_ring = GET_TX_WRAPPER_DESC(qinx);

		if ((ptx_ring->dirty_tx == ptx_ring->cur_tx) &&
		    (!pdata->tx_path_in_lpi_mode)) {
			tx_idle = 1;
		} else {
			tx_idle = 0;
			break;
		}
	}

	if (tx_idle)
		hw_if->set_eee_mode();

	DBGPR_EEE("<--eqos_enable_eee_mode\n");
}

void eqos_disable_eee_mode(struct eqos_prv_data *pdata)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);

	DBGPR_EEE("-->eqos_disable_eee_mode\n");

	hw_if->reset_eee_mode();
	del_timer_sync(&pdata->eee_ctrl_timer);
	pdata->tx_path_in_lpi_mode = false;

	DBGPR_EEE("-->eqos_disable_eee_mode\n");
}


/*!
* \brief API to control EEE mode.
*
* \details This function will move the MAC transmitter in LPI mode
* if there is no data transfer and MAC is not already in LPI state.
*
* \param[in] data - data hook
*
* \return void
*/

static void eqos_eee_ctrl_timer(unsigned long data)
{
	struct eqos_prv_data *pdata =
		(struct eqos_prv_data *)data;

	DBGPR_EEE("-->eqos_eee_ctrl_timer\n");

	eqos_enable_eee_mode(pdata);

	DBGPR_EEE("<--eqos_eee_ctrl_timer\n");
}

/*!
* \brief API to initialize EEE mode.
*
* \details This function enables the LPI state and start the timer
* to verify whether the tx path can enter in LPI state if
* a. GMAC supports EEE mode &
* b. phy can also manage EEE.
*
* \param[in] pdata - pointer to private data structure
*
* \return bool
*
* \retval true on success & false on failure.
*/
bool eqos_eee_init(struct eqos_prv_data *pdata)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	int twt_timer = EQOS_DEFAULT_LPI_TWT_TIMER;
	unsigned long flags;

	DBGPR_EEE("-->eqos_eee_init\n");

	/* Check if MAC supports EEE */
	if (!pdata->hw_feat.eee_sel)
		return false;

	/* Check if PHY supports EEE */
	if (phy_init_eee(pdata->phydev, 1)) {
		/* If EEE disabled at PHY during runtime
		 * then disable MAC EEE timers
		 */
		spin_lock_irqsave(&pdata->lock, flags);
		if (pdata->eee_active) {
			del_timer_sync(&pdata->eee_ctrl_timer);
			hw_if->set_eee_timer(0, twt_timer);
		}
		pdata->eee_active = 0;
		spin_unlock_irqrestore(&pdata->lock, flags);
		return false;
	}

	spin_lock_irqsave(&pdata->lock, flags);
	if (!pdata->eee_active) {
		pdata->eee_active = 1;
		setup_timer(&pdata->eee_ctrl_timer, eqos_eee_ctrl_timer,
			    (unsigned long)pdata);
		mod_timer(&pdata->eee_ctrl_timer,
			  EQOS_LPI_TIMER(EQOS_DEFAULT_LPI_TIMER));

		hw_if->set_eee_timer(EQOS_DEFAULT_LPI_LS_TIMER, twt_timer);
		if (pdata->use_lpi_tx_automate)
			hw_if->set_lpi_tx_automate();
	}

	hw_if->set_eee_pls(pdata->phydev->link);

	spin_unlock_irqrestore(&pdata->lock, flags);

	DBGPR_EEE("EEE initialized\n");

	DBGPR_EEE("<--eqos_eee_init\n");

	return true;
}

#define MAC_LPS_TLPIEN 0x00000001
#define MAC_LPS_TLPIEX 0x00000002
#define MAC_LPS_RLPIEN 0x00000004
#define MAC_LPS_RLPIEX 0x00000008
void eqos_handle_eee_interrupt(struct eqos_prv_data *pdata)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	u32 lpi_status;

	DBGPR_EEE("-->eqos_handle_eee_interrupt\n");

	lpi_status = hw_if->get_lpi_status();
	DBGPR_EEE("MAC_LPI_Control_Status = %#x\n", lpi_status);

	if (lpi_status & MAC_LPS_TLPIEN) {
		pdata->tx_path_in_lpi_mode = 1;
		pdata->xstats.tx_path_in_lpi_mode_irq_n++;
		DBGPR_EEE("MAC Transmitter has entered the LPI state\n");
	}

	if (lpi_status & MAC_LPS_TLPIEX) {
		pdata->tx_path_in_lpi_mode = 0;
		pdata->xstats.tx_path_exit_lpi_mode_irq_n++;
		DBGPR_EEE("MAC Transmitter has exited the LPI state\n");
	}

	if (lpi_status & MAC_LPS_RLPIEN) {
		pdata->xstats.rx_path_in_lpi_mode_irq_n++;
		DBGPR_EEE("MAC Receiver has entered the LPI state\n");
	}

	if (lpi_status & MAC_LPS_RLPIEX) {
		pdata->xstats.rx_path_exit_lpi_mode_irq_n++;
		DBGPR_EEE("MAC Receiver has exited the LPI state\n");
	}

	DBGPR_EEE("<--eqos_handle_eee_interrupt\n");
}
