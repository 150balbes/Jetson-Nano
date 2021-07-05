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
 * ========================================================================= */
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
/*!@file: eqos_drv.c
 * @brief: Driver functions.
 */

#include <linux/gpio.h>
#include <linux/time.h>
#include <linux/platform/tegra/ptp-notifier.h>
#include <linux/reset.h>
#include "yheader.h"
#include "yapphdr.h"
#include "drv.h"

extern ULONG eqos_base_addr;
#include "yregacc.h"
#include "nvregacc.h"
#include <soc/tegra/chip-id.h>
#include <linux/nospec.h>

static INT eqos_status;

/* raw spinlock to get HW PTP time and kernel time atomically */
static DEFINE_RAW_SPINLOCK(eqos_ts_lock);

/* SA(Source Address) operations on TX */
unsigned char mac_addr0[6] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55 };
unsigned char mac_addr1[6] = { 0x00, 0x66, 0x77, 0x88, 0x99, 0xaa };

/* module parameters for configuring the queue modes
 * set default mode as GENERIC
 * */
/* Value of "2" enables mtl tx q */
static int q_op_mode[MAX_CHANS] = {
	EQOS_Q_DCB,
	EQOS_Q_AVB,
	EQOS_Q_AVB,
	EQOS_Q_AVB
};

/* Store the IRQ names to be used by /proc/interrupts */
static char irq_names[8][32];

module_param_array(q_op_mode, int, NULL, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(q_op_mode,
		 "MTL queue operation mode [0-DISABLED, 1-AVB, 2-DCB, 3-GENERIC]");

u64 eqos_get_ptptime(void *data)
{
	struct eqos_prv_data *pdata = data;
	struct hw_if_struct *hw_if = &pdata->hw_if;
	unsigned long flags;
	u64 ns;

	raw_spin_lock_irqsave(&pdata->ptp_lock, flags);

	ns = hw_if->get_systime();

	raw_spin_unlock_irqrestore(&pdata->ptp_lock, flags);

	return ns;
}

void eqos_stop_all_ch_tx_dma(struct eqos_prv_data *pdata)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	UINT qinx;

	pr_debug("-->eqos_stop_all_ch_tx_dma\n");

	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++)
		hw_if->stop_dma_tx(pdata, qinx);

	pr_debug("<--eqos_stop_all_ch_tx_dma\n");
}

static int is_ptp_addr(char *addr)
{
	if ((addr[0] == PTP1_MAC0) &&
	    (addr[1] == PTP1_MAC1) &&
	    (addr[2] == PTP1_MAC2) &&
	    (addr[3] == PTP1_MAC3) &&
	    (addr[4] == PTP1_MAC4) && (addr[5] == PTP1_MAC5))
		return 1;
	else if ((addr[0] == PTP2_MAC0) &&
		 (addr[1] == PTP2_MAC1) &&
		 (addr[2] == PTP2_MAC2) &&
		 (addr[3] == PTP2_MAC3) &&
		 (addr[4] == PTP2_MAC4) && (addr[5] == PTP2_MAC5))
		return 1;
	else
		return 0;
}

static void eqos_stop_all_ch_rx_dma(struct eqos_prv_data *pdata)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	UINT qinx;

	pr_debug("-->eqos_stop_all_ch_rx_dma\n");

	for (qinx = 0; qinx < EQOS_RX_QUEUE_CNT; qinx++)
		hw_if->stop_dma_rx(qinx);

	pr_debug("<--eqos_stop_all_ch_rx_dma\n");
}

static void eqos_napi_enable_mq(struct eqos_prv_data *pdata)
{
	int qinx;

	pr_debug("-->eqos_napi_enable_mq\n");

	for (qinx = 0; qinx < pdata->num_chans; qinx++) {
		napi_enable(&pdata->rx_queue[qinx].napi);
		napi_enable(&pdata->tx_queue[qinx].napi);
	}

	pr_debug("<--eqos_napi_enable_mq\n");
}

static void eqos_all_ch_napi_disable(struct eqos_prv_data *pdata)
{
	int qinx;

	pr_debug("-->eqos_napi_disable\n");

	for (qinx = 0; qinx < EQOS_RX_QUEUE_CNT; qinx++) {
		napi_synchronize(&pdata->rx_queue[qinx].napi);
		napi_disable(&pdata->rx_queue[qinx].napi);
		napi_synchronize(&pdata->tx_queue[qinx].napi);
		napi_disable(&pdata->tx_queue[qinx].napi);
	}

	pr_debug("<--eqos_napi_disable\n");
}

void eqos_disable_chan_rx_interrupt(struct eqos_prv_data *pdata, int chan)
{
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&pdata->chan_irq_lock[chan], flags);
	VIRT_INTR_CH_CRTL_RD(chan, reg);
	reg &= ~VIRT_INTR_CH_CRTL_RX_WR_MASK;
	VIRT_INTR_CH_CRTL_WR(chan, reg);
	spin_unlock_irqrestore(&pdata->chan_irq_lock[chan], flags);
}

void eqos_enable_chan_rx_interrupt(struct eqos_prv_data *pdata, int chan)
{
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&pdata->chan_irq_lock[chan], flags);
	VIRT_INTR_CH_CRTL_RD(chan, reg);
	reg |= VIRT_INTR_CH_CRTL_RX_WR_MASK;
	VIRT_INTR_CH_CRTL_WR(chan, reg);
	spin_unlock_irqrestore(&pdata->chan_irq_lock[chan], flags);
}

void eqos_disable_chan_tx_interrupt(struct eqos_prv_data *pdata, int chan)
{
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&pdata->chan_irq_lock[chan], flags);
	VIRT_INTR_CH_CRTL_RD(chan, reg);
	reg &= ~VIRT_INTR_CH_CRTL_TX_WR_MASK;
	VIRT_INTR_CH_CRTL_WR(chan, reg);
	spin_unlock_irqrestore(&pdata->chan_irq_lock[chan], flags);
}

void eqos_enable_chan_tx_interrupt(struct eqos_prv_data *pdata, int chan)
{
	unsigned long flags;
	u32 reg;

	spin_lock_irqsave(&pdata->chan_irq_lock[chan], flags);
	VIRT_INTR_CH_CRTL_RD(chan, reg);
	reg |= VIRT_INTR_CH_CRTL_TX_WR_MASK;
	VIRT_INTR_CH_CRTL_WR(chan, reg);
	spin_unlock_irqrestore(&pdata->chan_irq_lock[chan], flags);
}

void handle_non_ti_ri_chan_intrs(struct eqos_prv_data *pdata, int qinx)
{
	ULONG dma_sr;
	ULONG dma_ier;

	pr_debug("-->%s(), chan=%d\n", __func__, qinx);

	DMA_SR_RD(qinx, dma_sr);

	DMA_IER_RD(qinx, dma_ier);

	pr_debug("DMA_SR[%d] = %#lx, DMA_IER= %#lx\n", qinx, dma_sr, dma_ier);

	/*on ufpga, update of DMA_IER is really slow, such that interrupt
	 * would happen, but read of IER returns old value.  This would
	 * cause driver to return when there really was an interrupt asserted.
	 * so for now, comment this out.
	 */
	/* process only those interrupts which we
	 * have enabled.
	 */
	if (!(tegra_platform_is_unit_fpga()))
		dma_sr = (dma_sr & dma_ier);

	/* mask off ri and ti */
	dma_sr &= ~(((0x1) << 6) | 1);

	if (dma_sr == 0)
		return;

	/* ack non ti/ri ints */
	DMA_SR_WR(qinx, dma_sr);

	if ((GET_VALUE(dma_sr, DMA_SR_RBU_LPOS, DMA_SR_RBU_HPOS) & 1))
		pdata->xstats.rx_buf_unavailable_irq_n[qinx]++;

	if (tegra_platform_is_unit_fpga())
		dma_sr = (dma_sr & dma_ier);

	if (GET_VALUE(dma_sr, DMA_SR_TPS_LPOS, DMA_SR_TPS_HPOS) & 1) {
		pdata->xstats.tx_process_stopped_irq_n[qinx]++;
		eqos_status = -E_DMA_SR_TPS;
	}
	if (GET_VALUE(dma_sr, DMA_SR_TBU_LPOS, DMA_SR_TBU_HPOS) & 1) {
		pdata->xstats.tx_buf_unavailable_irq_n[qinx]++;
		eqos_status = -E_DMA_SR_TBU;
	}
	if (GET_VALUE(dma_sr, DMA_SR_RPS_LPOS, DMA_SR_RPS_HPOS) & 1) {
		pdata->xstats.rx_process_stopped_irq_n[qinx]++;
		eqos_status = -E_DMA_SR_RPS;
	}
	if (GET_VALUE(dma_sr, DMA_SR_RWT_LPOS, DMA_SR_RWT_HPOS) & 1) {
		pdata->xstats.rx_watchdog_irq_n++;
		eqos_status = S_DMA_SR_RWT;
	}
	if (GET_VALUE(dma_sr, DMA_SR_FBE_LPOS, DMA_SR_FBE_HPOS) & 1) {
		pdata->xstats.fatal_bus_error_irq_n++;
		pdata->fbe_chan_mask |= (1 << qinx);
		eqos_status = -E_DMA_SR_FBE;
		schedule_work(&pdata->fbe_work);
	}

	pr_debug("<--%s()\n", __func__);
}

void handle_mac_intrs(struct eqos_prv_data *pdata, ULONG dma_isr)
{
	ULONG mac_imr;
	ULONG mac_pmtcsr;
	ULONG mac_ans = 0;
	ULONG mac_pcs = 0;
	ULONG mac_isr;
	struct net_device *dev = pdata->dev;

	pr_debug("-->%s()\n", __func__);

	MAC_ISR_RD(mac_isr);

	/* Handle MAC interrupts */
	if (GET_VALUE(dma_isr, DMA_ISR_MACIS_LPOS, DMA_ISR_MACIS_HPOS) & 1) {
		/* handle only those MAC interrupts which are enabled */
		MAC_IMR_RD(mac_imr);
		mac_isr = (mac_isr & mac_imr);

		/* PMT interrupt
		 * RemoteWake and MagicPacket events will be received by PHY supporting
		 * these features on silicon and can be used to wake up Tegra.
		 * Still let the below code be here in case we ever get this interrupt.
		 */
		if (GET_VALUE(mac_isr, MAC_ISR_PMTIS_LPOS, MAC_ISR_PMTIS_HPOS) &
		    1) {
			pdata->xstats.pmt_irq_n++;
			eqos_status = S_MAC_ISR_PMTIS;
			MAC_PMTCSR_RD(mac_pmtcsr);
			pr_debug("commonisr: PMTCSR : %#lx\n", mac_pmtcsr);
		}

		/* RGMII/SMII interrupt */
		if (GET_VALUE
		    (mac_isr, MAC_ISR_RGSMIIS_LPOS, MAC_ISR_RGSMIIS_HPOS) & 1) {
			MAC_PCS_RD(mac_pcs);
			pr_debug("RGMII/SMII interrupt: MAC_PCS = %#lx\n",
			       mac_pcs);
#ifdef HWA_NV_1637630

#else
			/* Comment out this block of code(1637630)
			 * as it was preventing 10mb to work.
			 */
			if ((mac_pcs & 0x80000) == 0x80000) {
				pdata->pcs_link = 1;
				netif_carrier_on(dev);
				if ((mac_pcs & 0x10000) == 0x10000) {
					pdata->pcs_duplex = 1;
					hw_if->set_full_duplex();
				} else {
					pdata->pcs_duplex = 0;
					hw_if->set_half_duplex(pdata);
				}

				if ((mac_pcs & 0x60000) == 0x0) {
					pdata->pcs_speed = SPEED_10;
					hw_if->set_mii_speed_10();
				} else if ((mac_pcs & 0x60000) == 0x20000) {
					pdata->pcs_speed = SPEED_100;
					hw_if->set_mii_speed_100();
				} else if ((mac_pcs & 0x60000) == 0x30000) {
					pdata->pcs_speed = SPEED_1000;
					hw_if->set_gmii_speed();
				}
				pr_err("Link is UP:%dMbps & %s duplex\n",
				       pdata->pcs_speed,
				       pdata->pcs_duplex ? "Full" : "Half");
			} else {
				pr_err("Link is Down\n");
				pdata->pcs_link = 0;
				netif_carrier_off(dev);
			}
#endif
		}

		/* PCS Link Status interrupt */
		if (GET_VALUE
		    (mac_isr, MAC_ISR_PCSLCHGIS_LPOS,
		     MAC_ISR_PCSLCHGIS_HPOS) & 1) {
			pr_err("PCS Link Status interrupt\n");
			MAC_ANS_RD(mac_ans);
			if (GET_VALUE(mac_ans, MAC_ANS_LS_LPOS, MAC_ANS_LS_HPOS)
			    & 1) {
				pr_err("Link: Up\n");
				netif_carrier_on(dev);
				pdata->pcs_link = 1;
			} else {
				pr_err("Link: Down\n");
				netif_carrier_off(dev);
				pdata->pcs_link = 0;
			}
		}

		/* PCS Auto-Negotiation Complete interrupt */
		if (GET_VALUE
		    (mac_isr, MAC_ISR_PCSANCIS_LPOS,
		     MAC_ISR_PCSANCIS_HPOS) & 1) {
			pr_err("PCS Auto-Negotiation Complete interrupt\n");
			MAC_ANS_RD(mac_ans);
		}

		/* EEE interrupts */
		if (GET_VALUE(mac_isr, MAC_ISR_LPI_LPOS, MAC_ISR_LPI_HPOS) & 1) {
			eqos_handle_eee_interrupt(pdata);
		}
	}

	pr_debug("<--%s()\n", __func__);
}


/*!
* Only used when multi irq is enabled
*/

irqreturn_t eqos_common_isr(int irq, void *device_id)
{
	ULONG dma_isr;
	struct eqos_prv_data *pdata = (struct eqos_prv_data *)device_id;
	UINT qinx;

	pr_debug("-->%s()\n", __func__);

	DMA_ISR_RD(dma_isr);
	if (dma_isr == 0x0)
		return IRQ_NONE;

	pr_debug("DMA_ISR = %#lx\n", dma_isr);

	if (dma_isr & 0xf)
		for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++)
			handle_non_ti_ri_chan_intrs(pdata, qinx);

	handle_mac_intrs(pdata, dma_isr);

	pr_debug("<--%s()\n", __func__);

	return IRQ_HANDLED;

}

irqreturn_t eqos_rx_chan_isr(int irq, void *data)
{
	struct eqos_rx_queue *rx_queue = (struct eqos_rx_queue *)data;
	struct eqos_prv_data *pdata = rx_queue->pdata;
	unsigned int qinx = rx_queue->chan_num;
	u32 dma_ier, dma_sr, ch_crtl, ch_stat;
	unsigned long flags;

	spin_lock_irqsave(&pdata->chan_irq_lock[qinx], flags);
	DMA_SR_RD(qinx, dma_sr);
	DMA_IER_RD(qinx, dma_ier);
	VIRT_INTR_CH_STAT_RD(qinx, ch_stat);
	VIRT_INTR_CH_CRTL_RD(qinx, ch_crtl);

	netdev_dbg(pdata->dev, "DMA_SR[%d] = %#x, DMA_IER= %#x\n",
		   qinx, dma_sr, dma_ier);
	netdev_dbg(pdata->dev,
		   "VIRT_INTR_CH_STAT[%d] = %#x, VIRT_INTR_CH_CRTL= %#x\n",
		   qinx, ch_stat, ch_crtl);

	if (ch_stat & VIRT_INTR_CH_CRTL_RX_WR_MASK) {
		DMA_SR_WR(qinx, ((0x1) << 6) | ((0x1) << 15));
		VIRT_INTR_CH_STAT_WR(qinx, VIRT_INTR_CH_CRTL_RX_WR_MASK);
		pdata->xstats.rx_normal_irq_n[qinx]++;
	}
	spin_unlock_irqrestore(&pdata->chan_irq_lock[qinx], flags);

	if (likely(napi_schedule_prep(&rx_queue->napi))) {
		eqos_disable_chan_rx_interrupt(pdata, qinx);
		__napi_schedule(&rx_queue->napi);
	}

	return IRQ_HANDLED;
}

irqreturn_t eqos_tx_chan_isr(int irq, void *data)
{
	struct eqos_tx_queue *tx_queue = (struct eqos_tx_queue *)data;
	struct eqos_prv_data *pdata = tx_queue->pdata;
	unsigned int qinx = tx_queue->chan_num;
	u32 dma_ier, dma_sr, ch_crtl, ch_stat;
	unsigned long flags;

	spin_lock_irqsave(&pdata->chan_irq_lock[qinx], flags);
	DMA_SR_RD(qinx, dma_sr);
	DMA_IER_RD(qinx, dma_ier);
	VIRT_INTR_CH_STAT_RD(qinx, ch_stat);
	VIRT_INTR_CH_CRTL_RD(qinx, ch_crtl);

	netdev_dbg(pdata->dev, "DMA_SR[%d] = %#x, DMA_IER= %#x\n",
		   qinx, dma_sr, dma_ier);
	netdev_dbg(pdata->dev,
		   "VIRT_INTR_CH_STAT[%d] = %#x, VIRT_INTR_CH_CRTL= %#x\n",
		   qinx, ch_stat, ch_crtl);

	if (ch_stat & VIRT_INTR_CH_CRTL_TX_WR_MASK) {
		DMA_SR_WR(qinx, ((0x1) << 0) | ((0x1) << 15));
		VIRT_INTR_CH_STAT_WR(qinx, VIRT_INTR_CH_CRTL_TX_WR_MASK);
		pdata->xstats.tx_normal_irq_n[qinx]++;
	}
	spin_unlock_irqrestore(&pdata->chan_irq_lock[qinx], flags);

	if (likely(napi_schedule_prep(&tx_queue->napi))) {
		eqos_disable_chan_tx_interrupt(pdata, qinx);
		__napi_schedule(&tx_queue->napi);
	}

	return IRQ_HANDLED;
}

/*!
* \brief API to get all hw features.
*
* \details This function is used to check what are all the different
* features the device supports.
*
* \param[in] pdata - pointer to driver private structure
*
* \return none
*/

void eqos_get_all_hw_features(struct eqos_prv_data *pdata)
{
	unsigned int mac_hfr0;
	unsigned int mac_hfr1;
	unsigned int mac_hfr2;

	pr_debug("-->eqos_get_all_hw_features\n");

	MAC_HFR0_RD(mac_hfr0);
	MAC_HFR1_RD(mac_hfr1);
	MAC_HFR2_RD(mac_hfr2);

	memset(&pdata->hw_feat, 0, sizeof(pdata->hw_feat));
	pdata->hw_feat.mii_sel = ((mac_hfr0 >> 0) & MAC_HFR0_MIISEL_MASK);
	pdata->hw_feat.gmii_sel = ((mac_hfr0 >> 1) & MAC_HFR0_GMIISEL_MASK);
	pdata->hw_feat.hd_sel = ((mac_hfr0 >> 2) & MAC_HFR0_HDSEL_MASK);
	pdata->hw_feat.pcs_sel = ((mac_hfr0 >> 3) & MAC_HFR0_PCSSEL_MASK);
	pdata->hw_feat.vlan_hash_en = 0;
	pdata->hw_feat.sma_sel = ((mac_hfr0 >> 5) & MAC_HFR0_SMASEL_MASK);
	pdata->hw_feat.rwk_sel = ((mac_hfr0 >> 6) & MAC_HFR0_RWKSEL_MASK);
	pdata->hw_feat.mgk_sel = ((mac_hfr0 >> 7) & MAC_HFR0_MGKSEL_MASK);
	pdata->hw_feat.mmc_sel = ((mac_hfr0 >> 8) & MAC_HFR0_MMCSEL_MASK);
	pdata->hw_feat.arp_offld_en =
	    ((mac_hfr0 >> 9) & MAC_HFR0_ARPOFFLDEN_MASK);
	pdata->hw_feat.ts_sel = ((mac_hfr0 >> 12) & MAC_HFR0_TSSSEL_MASK);
	pdata->hw_feat.eee_sel = ((mac_hfr0 >> 13) & MAC_HFR0_EEESEL_MASK);
	pdata->hw_feat.tx_coe_sel = ((mac_hfr0 >> 14) & MAC_HFR0_TXCOESEL_MASK);
	pdata->hw_feat.rx_coe_sel = ((mac_hfr0 >> 16) & MAC_HFR0_RXCOE_MASK);
	pdata->hw_feat.mac_addr16_sel =
	    ((mac_hfr0 >> 18) & MAC_HFR0_ADDMACADRSEL_MASK);
	pdata->hw_feat.mac_addr32_sel =
	    ((mac_hfr0 >> 23) & MAC_HFR0_MACADR32SEL_MASK);
	pdata->hw_feat.mac_addr64_sel =
	    ((mac_hfr0 >> 24) & MAC_HFR0_MACADR64SEL_MASK);
	pdata->hw_feat.tsstssel = ((mac_hfr0 >> 25) & MAC_HFR0_TSINTSEL_MASK);
	pdata->hw_feat.sa_vlan_ins =
	    ((mac_hfr0 >> 27) & MAC_HFR0_SAVLANINS_MASK);
	pdata->hw_feat.act_phy_sel =
	    ((mac_hfr0 >> 28) & MAC_HFR0_ACTPHYSEL_MASK);

	pdata->hw_feat.rx_fifo_size =
	    ((mac_hfr1 >> 0) & MAC_HFR1_RXFIFOSIZE_MASK);
	pdata->hw_feat.tx_fifo_size =
	    ((mac_hfr1 >> 6) & MAC_HFR1_TXFIFOSIZE_MASK);
	pdata->hw_feat.adv_ts_hword =
	    ((mac_hfr1 >> 13) & MAC_HFR1_ADVTHWORD_MASK);
	pdata->hw_feat.dcb_en = ((mac_hfr1 >> 16) & MAC_HFR1_DCBEN_MASK);
	pdata->hw_feat.sph_en = ((mac_hfr1 >> 17) & MAC_HFR1_SPHEN_MASK);
	pdata->hw_feat.tso_en = ((mac_hfr1 >> 18) & MAC_HFR1_TSOEN_MASK);
	pdata->hw_feat.dma_debug_gen =
	    ((mac_hfr1 >> 19) & MAC_HFR1_DMADEBUGEN_MASK);
	pdata->hw_feat.av_sel = ((mac_hfr1 >> 20) & MAC_HFR1_AVSEL_MASK);
	pdata->hw_feat.lp_mode_en = ((mac_hfr1 >> 23) & MAC_HFR1_LPMODEEN_MASK);
#ifdef ENABLE_PERFECT_L2_FILTER
	pdata->hw_feat.hash_tbl_sz = 0;
#else
	pdata->hw_feat.hash_tbl_sz =
	    ((mac_hfr1 >> 24) & MAC_HFR1_HASHTBLSZ_MASK);
#endif
	pdata->hw_feat.l3l4_filter_num =
	    ((mac_hfr1 >> 27) & MAC_HFR1_L3L4FILTERNUM_MASK);

	pdata->hw_feat.rx_q_cnt = ((mac_hfr2 >> 0) & MAC_HFR2_RXQCNT_MASK);
	pdata->hw_feat.tx_q_cnt = ((mac_hfr2 >> 6) & MAC_HFR2_TXQCNT_MASK);
	pdata->hw_feat.rx_ch_cnt = ((mac_hfr2 >> 12) & MAC_HFR2_RXCHCNT_MASK);
	pdata->hw_feat.tx_ch_cnt = ((mac_hfr2 >> 18) & MAC_HFR2_TXCHCNT_MASK);
	pdata->hw_feat.pps_out_num =
	    ((mac_hfr2 >> 24) & MAC_HFR2_PPSOUTNUM_MASK);
	pdata->hw_feat.aux_snap_num =
	    ((mac_hfr2 >> 28) & MAC_HFR2_AUXSNAPNUM_MASK);

	if (pdata->hw_feat.mac_addr64_sel)
		pdata->max_addr_reg_cnt = 128;
	else if (pdata->hw_feat.mac_addr32_sel)
		pdata->max_addr_reg_cnt = 64;
	else if (pdata->hw_feat.mac_addr16_sel)
		pdata->max_addr_reg_cnt = 32;
	else
		pdata->max_addr_reg_cnt = 1;

	switch (pdata->hw_feat.hash_tbl_sz) {
	case 0:
		pdata->max_hash_table_size = 0;
		break;
	case 1:
		pdata->max_hash_table_size = 64;
		break;
	case 2:
		pdata->max_hash_table_size = 128;
		break;
	case 3:
		pdata->max_hash_table_size = 256;
		break;
	}

	pr_debug("<--eqos_get_all_hw_features\n");
}

#ifdef YDEBUG
/*!
* \brief API to print all hw features.
*
* \details This function is used to print all the device feature.
*
* \param[in] pdata - pointer to driver private structure
*
* \return none
*/

void eqos_print_all_hw_features(struct eqos_prv_data *pdata)
{
	char *str = NULL;

	pr_debug("-->eqos_print_all_hw_features\n");

	pr_err("\n");
	pr_err("=====================================================/\n");
	pr_err("\n");
	pr_err("10/100 Mbps Support                         : %s\n",
	       pdata->hw_feat.mii_sel ? "YES" : "NO");
	pr_err("1000 Mbps Support                           : %s\n",
	       pdata->hw_feat.gmii_sel ? "YES" : "NO");
	pr_err("Half-duplex Support                         : %s\n",
	       pdata->hw_feat.hd_sel ? "YES" : "NO");
	pr_err("PCS Registers(TBI/SGMII/RTBI PHY interface) : %s\n",
	       pdata->hw_feat.pcs_sel ? "YES" : "NO");
	pr_err("VLAN Hash Filter Selected                   : %s\n",
	       pdata->hw_feat.vlan_hash_en ? "YES" : "NO");
	pdata->vlan_hash_filtering = pdata->hw_feat.vlan_hash_en;
	pr_err("SMA (MDIO) Interface                        : %s\n",
	       pdata->hw_feat.sma_sel ? "YES" : "NO");
	pr_err("PMT Remote Wake-up Packet Enable            : %s\n",
	       pdata->hw_feat.rwk_sel ? "YES" : "NO");
	pr_err("PMT Magic Packet Enable                     : %s\n",
	       pdata->hw_feat.mgk_sel ? "YES" : "NO");
	pr_err("RMON/MMC Module Enable                      : %s\n",
	       pdata->hw_feat.mmc_sel ? "YES" : "NO");
	pr_err("ARP Offload Enabled                         : %s\n",
	       pdata->hw_feat.arp_offld_en ? "YES" : "NO");
	pr_err("IEEE 1588-2008 Timestamp Enabled            : %s\n",
	       pdata->hw_feat.ts_sel ? "YES" : "NO");
	pr_err("Energy Efficient Ethernet Enabled           : %s\n",
	       pdata->hw_feat.eee_sel ? "YES" : "NO");
	pr_err("Transmit Checksum Offload Enabled           : %s\n",
	       pdata->hw_feat.tx_coe_sel ? "YES" : "NO");
	pr_err("Receive Checksum Offload Enabled            : %s\n",
	       pdata->hw_feat.rx_coe_sel ? "YES" : "NO");
	pr_err("MAC Addresses 16–31 Selected                : %s\n",
	       pdata->hw_feat.mac_addr16_sel ? "YES" : "NO");
	pr_err("MAC Addresses 32–63 Selected                : %s\n",
	       pdata->hw_feat.mac_addr32_sel ? "YES" : "NO");
	pr_err("MAC Addresses 64–127 Selected               : %s\n",
	       pdata->hw_feat.mac_addr64_sel ? "YES" : "NO");

	switch (pdata->hw_feat.tsstssel) {
	case 0:
		str = "RESERVED";
		break;
	case 1:
		str = "INTERNAL";
		break;
	case 2:
		str = "EXTERNAL";
		break;
	case 3:
		str = "BOTH";
		break;
	}
	pr_err("Timestamp System Time Source                : %s\n", str);
	pr_err("Source Address or VLAN Insertion Enable     : %s\n",
	       pdata->hw_feat.sa_vlan_ins ? "YES" : "NO");

	switch (pdata->hw_feat.act_phy_sel) {
	case 0:
		str = "GMII/MII";
		break;
	case 1:
		str = "RGMII";
		break;
	case 2:
		str = "SGMII";
		break;
	case 3:
		str = "TBI";
		break;
	case 4:
		str = "RMII";
		break;
	case 5:
		str = "RTBI";
		break;
	case 6:
		str = "SMII";
		break;
	case 7:
		str = "RevMII";
		break;
	default:
		str = "RESERVED";
	}
	pr_err("Active PHY Selected                         : %s\n", str);

	switch (pdata->hw_feat.rx_fifo_size) {
	case 0:
		str = "128 bytes";
		break;
	case 1:
		str = "256 bytes";
		break;
	case 2:
		str = "512 bytes";
		break;
	case 3:
		str = "1 KBytes";
		break;
	case 4:
		str = "2 KBytes";
		break;
	case 5:
		str = "4 KBytes";
		break;
	case 6:
		str = "8 KBytes";
		break;
	case 7:
		str = "16 KBytes";
		break;
	case 8:
		str = "32 kBytes";
		break;
	case 9:
		/* EQOS_IP_V5  and above supports 36KB FIFO size */
		str = "36 KBytes";
		break;
	case 10:
		str = "128 KBytes";
		break;
	case 11:
		str = "256 KBytes";
		break;
	default:
		str = "RESERVED";
	}
	pr_err("MTL Receive FIFO Size                       : %s\n", str);

	switch (pdata->hw_feat.tx_fifo_size) {
	case 0:
		str = "128 bytes";
		break;
	case 1:
		str = "256 bytes";
		break;
	case 2:
		str = "512 bytes";
		break;
	case 3:
		str = "1 KBytes";
		break;
	case 4:
		str = "2 KBytes";
		break;
	case 5:
		str = "4 KBytes";
		break;
	case 6:
		str = "8 KBytes";
		break;
	case 7:
		str = "16 KBytes";
		break;
	case 8:
		str = "32 kBytes";
		break;
	case 9:
		/* EQOS_IP_V5  and above supports 36KB FIFO size */
		str = "36 KBytes";
		break;
	case 10:
		str = "128 KBytes";
		break;
	case 11:
		str = "256 KBytes";
		break;
	default:
		str = "RESERVED";
	}
	pr_err("MTL Transmit FIFO Size                       : %s\n", str);
	pr_err("IEEE 1588 High Word Register Enable          : %s\n",
	       pdata->hw_feat.adv_ts_hword ? "YES" : "NO");
	pr_err("DCB Feature Enable                           : %s\n",
	       pdata->hw_feat.dcb_en ? "YES" : "NO");
	pr_err("Split Header Feature Enable                  : %s\n",
	       pdata->hw_feat.sph_en ? "YES" : "NO");
	pr_err("TCP Segmentation Offload Enable              : %s\n",
	       pdata->hw_feat.tso_en ? "YES" : "NO");
	pr_err("DMA Debug Registers Enabled                  : %s\n",
	       pdata->hw_feat.dma_debug_gen ? "YES" : "NO");
	pr_err("AV Feature Enabled                           : %s\n",
	       pdata->hw_feat.av_sel ? "YES" : "NO");
	pr_err("Low Power Mode Enabled                       : %s\n",
	       pdata->hw_feat.lp_mode_en ? "YES" : "NO");

	switch (pdata->hw_feat.hash_tbl_sz) {
	case 0:
		str = "No hash table selected";
		break;
	case 1:
		str = "64";
		break;
	case 2:
		str = "128";
		break;
	case 3:
		str = "256";
		break;
	}
	pr_err("Hash Table Size                              : %s\n", str);
	pr_err
	    ("Total number of L3 or L4 Filters             : %d L3/L4 Filter\n",
	     pdata->hw_feat.l3l4_filter_num);
	pr_err("Number of MTL Receive Queues                 : %d\n",
	       (pdata->hw_feat.rx_q_cnt + 1));
	pr_err("Number of MTL Transmit Queues                : %d\n",
	       (pdata->hw_feat.tx_q_cnt + 1));
	pr_err("Number of DMA Receive Channels               : %d\n",
	       (pdata->hw_feat.rx_ch_cnt + 1));
	pr_err("Number of DMA Transmit Channels              : %d\n",
	       (pdata->hw_feat.tx_ch_cnt + 1));

	switch (pdata->hw_feat.pps_out_num) {
	case 0:
		str = "No PPS output";
		break;
	case 1:
		str = "1 PPS output";
		break;
	case 2:
		str = "2 PPS output";
		break;
	case 3:
		str = "3 PPS output";
		break;
	case 4:
		str = "4 PPS output";
		break;
	default:
		str = "RESERVED";
	}
	pr_err("Number of PPS Outputs                        : %s\n", str);

	switch (pdata->hw_feat.aux_snap_num) {
	case 0:
		str = "No auxillary input";
		break;
	case 1:
		str = "1 auxillary input";
		break;
	case 2:
		str = "2 auxillary input";
		break;
	case 3:
		str = "3 auxillary input";
		break;
	case 4:
		str = "4 auxillary input";
		break;
	default:
		str = "RESERVED";
	}
	pr_err("Number of Auxiliary Snapshot Inputs          : %s", str);

	pr_err("\n");
	pr_err("=====================================================/\n");

	pr_debug("<--eqos_print_all_hw_features\n");
}
#endif

/*!
 * \brief api to initialize default values.
 *
 * \details This function is used to initialize differnet parameters to
 * default values which are common parameters between Tx and Rx path.
 *
 * \param[in] pdata – pointer to private data structure.
 *
 * \return void
 */

static void eqos_default_common_confs(struct eqos_prv_data *pdata)
{
	pr_debug("-->eqos_default_common_confs\n");

	pdata->drop_tx_pktburstcnt = 1;
	pdata->mac_enable_count = 0;
	pdata->incr_incrx = EQOS_INCR_ENABLE;
	pdata->flow_ctrl = EQOS_FLOW_CTRL_TX_RX;
	pdata->oldflow_ctrl = EQOS_FLOW_CTRL_TX_RX;
	pdata->tx_sa_ctrl_via_desc = EQOS_SA0_NONE;
	pdata->tx_sa_ctrl_via_reg = EQOS_SA0_NONE;
	pdata->hwts_tx_en = 0;
	pdata->hwts_rx_en = 0;
	pdata->l3_l4_filter = 0;
	pdata->l2_filtering_mode = !!pdata->hw_feat.hash_tbl_sz;
	pdata->tx_path_in_lpi_mode = 0;
	pdata->use_lpi_tx_automate = true;
	pdata->eee_active = 0;
	pdata->one_nsec_accuracy = 1;
	pdata->mac_addr_idx = 0;

	pr_debug("<--eqos_default_common_confs\n");
}

/*!
 * \brief api to initialize Tx parameters.
 *
 * \details This function is used to initialize all Tx
 * parameters to default values on reset.
 *
 * \param[in] pdata – pointer to private data structure.
 * \param[in] qinx – DMA channel/queue number to be initialized.
 *
 * \return void
 */

static void eqos_default_tx_confs_single_q(struct eqos_prv_data *pdata,
					   UINT qinx)
{
	struct eqos_tx_queue *queue_data = GET_TX_QUEUE_PTR(qinx);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct tx_ring *ptx_ring =
	    GET_TX_WRAPPER_DESC(qinx);

	pr_debug("-->eqos_default_tx_confs_single_q\n");

	queue_data->q_op_mode = q_op_mode[qinx];

	ptx_ring->tx_threshold_val = EQOS_TX_THRESHOLD_32;
	ptx_ring->tsf_on = EQOS_TSF_ENABLE;
	ptx_ring->osf_on = EQOS_OSF_ENABLE;
	ptx_ring->tx_pbl = EQOS_PBL_16;
	ptx_ring->tx_vlan_tag_via_reg = Y_FALSE;
	ptx_ring->tx_vlan_tag_ctrl = EQOS_TX_VLAN_TAG_INSERT;
	ptx_ring->vlan_tag_present = 0;
	ptx_ring->context_setup = 0;
	ptx_ring->default_mss = 0;
	hw_if->enable_vlan_desc_control(pdata);

	pr_debug("<--eqos_default_tx_confs_single_q\n");
}

/*!
 * \brief api to initialize Rx parameters.
 *
 * \details This function is used to initialize all Rx
 * parameters to default values on reset.
 *
 * \param[in] pdata – pointer to private data structure.
 * \param[in] qinx – DMA queue/channel number to be initialized.
 *
 * \return void
 */

static void eqos_default_rx_confs_single_q(struct eqos_prv_data *pdata,
					   UINT qinx)
{
	struct rx_ring *prx_ring =
	    GET_RX_WRAPPER_DESC(qinx);

	pr_debug("-->eqos_default_rx_confs_single_q\n");

	prx_ring->rx_threshold_val = EQOS_RX_THRESHOLD_64;
	prx_ring->rsf_on = EQOS_RSF_DISABLE;
	prx_ring->rx_pbl = EQOS_PBL_16;
	prx_ring->rx_outer_vlan_strip = EQOS_RX_VLAN_STRIP_ALWAYS;
	prx_ring->rx_inner_vlan_strip = EQOS_RX_VLAN_STRIP_ALWAYS;

	pr_debug("<--eqos_default_rx_confs_single_q\n");
}

static void eqos_default_tx_confs(struct eqos_prv_data *pdata)
{
	UINT qinx;

	pr_debug("-->eqos_default_tx_confs\n");

	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		eqos_default_tx_confs_single_q(pdata, qinx);
	}

	pr_debug("<--eqos_default_tx_confs\n");
}

static void eqos_default_rx_confs(struct eqos_prv_data *pdata)
{
	UINT qinx;

	pr_debug("-->eqos_default_rx_confs\n");

	for (qinx = 0; qinx < EQOS_RX_QUEUE_CNT; qinx++) {
		eqos_default_rx_confs_single_q(pdata, qinx);
	}

	pr_debug("<--eqos_default_rx_confs\n");
}

void free_txrx_irqs(struct eqos_prv_data *pdata)
{
	uint i;

	pr_debug("-->%s()\n", __func__);

	free_irq(pdata->common_irq, pdata);

	for (i = 0; i < pdata->num_chans; i++) {
		if (pdata->rx_irq_alloc_mask & (1 << i)) {
			free_irq(pdata->rx_irqs[i], &pdata->rx_queue[i]);
		}
		if (pdata->tx_irq_alloc_mask & (1 << i)) {
			free_irq(pdata->tx_irqs[i], &pdata->tx_queue[i]);
		}
	}

	pr_debug("<--%s()\n", __func__);
}

int request_txrx_irqs(struct eqos_prv_data *pdata)
{
	struct platform_device *pdev = pdata->pdev;
	int ret = 0, i, j = 0;

	pdata->irq_number = pdata->dev->irq;

	ret = request_irq(pdata->common_irq, eqos_common_isr,
			  IRQF_SHARED, "ether_qos.common_irq", pdata);
	if (unlikely(ret < 0)) {
		netdev_err(pdata->dev, "failed to register common interrupt - %d\n",
			   pdata->common_irq);
		goto err_common_irq;
	}

	for (i = 0; i < pdata->num_chans; i++) {
		snprintf(irq_names[j], 32, "%s.rx%d", dev_name(&pdev->dev), i);
		ret = request_irq(pdata->rx_irqs[i], eqos_rx_chan_isr,
				  IRQF_TRIGGER_NONE, irq_names[j++],
				  &pdata->rx_queue[i]);
		if (unlikely(ret < 0)) {
			netdev_err(pdata->dev, "failed to register Rx channel interrupt - %d\n",
				   pdata->rx_irqs[i]);
			goto err_chan_irq;
		}

		pdata->rx_irq_alloc_mask |= (1 << i);

		snprintf(irq_names[j], 32, "%s.tx%d", dev_name(&pdev->dev), i);
		ret = request_irq(pdata->tx_irqs[i], eqos_tx_chan_isr,
				  IRQF_TRIGGER_NONE, irq_names[j++],
				  &pdata->tx_queue[i]);
		if (unlikely(ret < 0)) {
			netdev_err(pdata->dev, "failed to register Tx channel interrupt - %d\n",
				   pdata->rx_irqs[i]);
			goto err_chan_irq;
		}

		pdata->tx_irq_alloc_mask |= (1 << i);
	}

	return ret;

err_chan_irq:
	free_txrx_irqs(pdata);
	free_irq(pdata->common_irq, pdata);

err_common_irq:
	return ret;
}


/*!
* \brief API to open a deivce for data transmission & reception.
*
* \details Opens the interface. The interface is opned whenever
* ifconfig activates it. The open method should register any
* system resource it needs like I/O ports, IRQ, DMA, etc,
* turn on the hardware, and perform any other setup your device requires.
*
* \param[in] dev - pointer to net_device structure
*
* \return integer
*
* \retval 0 on success & negative number on failure.
*/

static int eqos_open(struct net_device *dev)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);
	int ret = Y_SUCCESS;
	struct desc_if_struct *desc_if = &pdata->desc_if;
	struct hw_if_struct *hw_if = &pdata->hw_if;

	pr_debug("-->eqos_open\n");

	if (!is_valid_ether_addr(dev->dev_addr))
		return -EADDRNOTAVAIL;

	/* Reset the PHY */
	if (gpio_is_valid(pdata->phy_reset_gpio)) {
		gpio_set_value(pdata->phy_reset_gpio, 0);
		usleep_range(pdata->phy_reset_duration,
			     pdata->phy_reset_duration + 1);
		gpio_set_value(pdata->phy_reset_gpio, 1);
		msleep(pdata->phy_reset_post_delay);
	}

	ret = eqos_clock_enable(pdata);
	if (ret)
		return ret;

	/* issue CAR reset to device */
	ret = hw_if->car_reset(pdata);
	if (ret < 0) {
		dev_err(&dev->dev, "Failed to reset MAC\n");
		return -ENODEV;
	}

	/* PHY initialisation */
	ret = eqos_init_phy(dev);
	if (ret) {
		dev_err(&dev->dev, "%s: Cannot attach to PHY (error: %d)\n",
			__func__, ret);
		return ret;
	}

	ret = request_txrx_irqs(pdata);
	if (ret != Y_SUCCESS)
		goto err_irq_0;

	ret = desc_if->alloc_buff_and_desc(pdata);
	if (ret < 0) {
		dev_err(&pdata->pdev->dev,
			"Failed to allocate buffer/descriptor memory\n");
		ret = -ENOMEM;
		goto err_out_desc_buf_alloc_failed;
	}

#ifdef EQOS_CONFIG_PTP
	ret = eqos_ptp_init(pdata);
	if (ret < 0) {
		dev_err(&dev->dev, "failed to init PTP\n");
		goto err_ptp;
	}
#endif

	mutex_lock(&pdata->hw_change_lock);
	eqos_start_dev(pdata);

	pdata->hw_stopped = false;
	mutex_unlock(&pdata->hw_change_lock);

	if (!pdata->resv_skb || pdata->resv_dma == 0) {
		dev_err(&dev->dev, "failed to reserve SKB\n");
		ret = -ENOMEM;
		goto err_ptp;
	}
	phy_start(pdata->phydev);

	netif_tx_start_all_queues(pdata->dev);

	pr_debug("<--%s()\n", __func__);
	return Y_SUCCESS;

 err_ptp:
	desc_if->free_buff_and_desc(pdata);

 err_out_desc_buf_alloc_failed:
	free_txrx_irqs(pdata);

 err_irq_0:
	pr_debug("<--%s()\n", __func__);
	return ret;
}

/*!
* \brief API to close a device.
*
* \details Stops the interface. The interface is stopped when it is brought
* down. This function should reverse operations performed at open time.
*
* \param[in] dev - pointer to net_device structure
*
* \return integer
*
* \retval 0 on success & negative number on failure.
*/

static int eqos_close(struct net_device *dev)
{
	int i;
	struct eqos_prv_data *pdata = netdev_priv(dev);
	struct desc_if_struct *desc_if = &pdata->desc_if;

	pr_debug("-->%s\n", __func__);

	/* Stop and disconnect the PHY */
	if (pdata->phydev) {
		phy_stop(pdata->phydev);
		phy_disconnect(pdata->phydev);
		if (gpio_is_valid(pdata->phy_reset_gpio) &&
				 (pdata->mac_ver > EQOS_MAC_CORE_4_10))
			gpio_set_value(pdata->phy_reset_gpio, 0);
		pdata->phydev = NULL;
	}

#ifdef EQOS_CONFIG_PTP
	eqos_ptp_remove(pdata);
#endif
	mutex_lock(&pdata->hw_change_lock);
	eqos_stop_dev(pdata);

	desc_if->free_buff_and_desc(pdata);
	free_txrx_irqs(pdata);

	/* Cancel hrtimer */
	for (i = 0; i < pdata->num_chans; i++) {
		if (atomic_read(&pdata->tx_queue[i].tx_usecs_timer_armed)
				== EQOS_HRTIMER_ENABLE) {
			hrtimer_cancel(&pdata->tx_queue[i].tx_usecs_timer);
		}
	}

	pdata->hw_stopped = true;
	mutex_unlock(&pdata->hw_change_lock);

	/* Assert MAC RST gpio */
	if (pdata->eqos_rst)
		reset_control_assert(pdata->eqos_rst);

	eqos_clock_disable(pdata);

	/* cancel iso work */
	cancel_work_sync(&pdata->iso_work);
	/* Cancel FBE handling work */
	cancel_work_sync(&pdata->fbe_work);

	pr_debug("<--%s\n", __func__);
	return Y_SUCCESS;
}

/*!
* \brief API to configure the multicast address in device.
*
* \details This function collects all the multicast addresse
* and updates the device.
*
* \param[in] dev - pointer to net_device structure.
*
* \retval 0 if perfect filtering is seleted & 1 if hash
* filtering is seleted.
*/
static int eqos_prepare_mc_list(struct net_device *dev,
				unsigned int *mac_addr_idx)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	u32 mc_filter[EQOS_HTR_CNT + 1];
	struct netdev_hw_addr *ha = NULL;
	int crc32_val = 0;
	int ret = 0, i = 1;

	DBGPR_FILTER("-->eqos_prepare_mc_list\n");

	if (pdata->l2_filtering_mode) {
		DBGPR_FILTER
		    ("select HASH FILTERING for mc addresses: mc_count = %d\n",
		     netdev_mc_count(dev));
		ret = 1;
		memset(mc_filter, 0, sizeof(mc_filter));

		if (pdata->max_hash_table_size == 64) {
			netdev_for_each_mc_addr(ha, dev) {
				DBGPR_FILTER
				    ("mc addr[%d] = %#x:%#x:%#x:%#x:%#x:%#x\n",
				     i++, ha->addr[0], ha->addr[1], ha->addr[2],
				     ha->addr[3], ha->addr[4], ha->addr[5]);
				/* The upper 6 bits of the calculated CRC are used to
				 * index the content of the Hash Table Reg 0 and 1.
				 * */
				crc32_val =
				    (bitrev32(~crc32_le(~0, ha->addr, 6)) >>
				     26);
				/* The most significant bit determines the register
				 * to use (Hash Table Reg X, X = 0 and 1) while the
				 * other 5(0x1F) bits determines the bit within the
				 * selected register
				 * */
				mc_filter[crc32_val >> 5] |=
				    (1 << (crc32_val & 0x1F));
			}
		} else if (pdata->max_hash_table_size == 128) {
			netdev_for_each_mc_addr(ha, dev) {
				DBGPR_FILTER
				    ("mc addr[%d] = %#x:%#x:%#x:%#x:%#x:%#x\n",
				     i++, ha->addr[0], ha->addr[1], ha->addr[2],
				     ha->addr[3], ha->addr[4], ha->addr[5]);
				/* The upper 7 bits of the calculated CRC are used to
				 * index the content of the Hash Table Reg 0,1,2 and 3.
				 * */
				crc32_val =
				    (bitrev32(~crc32_le(~0, ha->addr, 6)) >>
				     25);

				pr_err("crc_le = %#x, crc_be = %#x\n",
				       bitrev32(~crc32_le(~0, ha->addr, 6)),
				       bitrev32(~crc32_be(~0, ha->addr, 6)));

				/* The most significant 2 bits determines the register
				 * to use (Hash Table Reg X, X = 0,1,2 and 3) while the
				 * other 5(0x1F) bits determines the bit within the
				 * selected register
				 * */
				mc_filter[crc32_val >> 5] |=
				    (1 << (crc32_val & 0x1F));
			}
		} else if (pdata->max_hash_table_size == 256) {
			netdev_for_each_mc_addr(ha, dev) {
				DBGPR_FILTER
				    ("mc addr[%d] = %#x:%#x:%#x:%#x:%#x:%#x\n",
				     i++, ha->addr[0], ha->addr[1], ha->addr[2],
				     ha->addr[3], ha->addr[4], ha->addr[5]);
				/* The upper 8 bits of the calculated CRC are used to
				 * index the content of the Hash Table Reg 0,1,2,3,4,
				 * 5,6, and 7.
				 * */
				crc32_val =
				    (bitrev32(~crc32_le(~0, ha->addr, 6)) >>
				     24);
				/* The most significant 3 bits determines the register
				 * to use (Hash Table Reg X, X = 0,1,2,3,4,5,6 and 7) while
				 * the other 5(0x1F) bits determines the bit within the
				 * selected register
				 * */
				mc_filter[crc32_val >> 5] |=
				    (1 << (crc32_val & 0x1F));
			}
		}

		for (i = 0; i < EQOS_HTR_CNT; i++)
			hw_if->update_hash_table_reg(i, mc_filter[i]);

	} else {
		DBGPR_FILTER
		    ("select PERFECT FILTERING for mc addresses, mc_count = %d, max_addr_reg_cnt = %d\n",
		     netdev_mc_count(dev), pdata->max_addr_reg_cnt);

		netdev_for_each_mc_addr(ha, dev) {
			DBGPR_FILTER("mc addr[%d] = %#x:%#x:%#x:%#x:%#x:%#x\n",
				     i,
				     ha->addr[0], ha->addr[1], ha->addr[2],
				     ha->addr[3], ha->addr[4], ha->addr[5]);
			if (i < 32)
				hw_if->update_mac_addr1_31_low_high_reg(i,
									ha->
									addr);
			else
				hw_if->update_mac_addr32_127_low_high_reg(i,
									  ha->
									  addr);

			if ((pdata->ptp_cfg.use_tagged_ptp) &&
			    (is_ptp_addr(ha->addr)) &&
			    pdata->dt_cfg.use_multi_q)
				hw_if->config_ptp_channel(pdata->ptp_cfg.
							  ptp_dma_ch_id, i);

			i++;
		}

		*mac_addr_idx = i;
	}

	DBGPR_FILTER("<--eqos_prepare_mc_list\n");

	return ret;
}

/*!
* \brief API to configure the unicast address in device.
*
* \details This function collects all the unicast addresses
* and updates the device.
*
* \param[in] dev - pointer to net_device structure.
*
* \retval 0 if perfect filtering is seleted  & 1 if hash
* filtering is seleted.
*/
static int eqos_prepare_uc_list(struct net_device *dev,
				unsigned int *mac_addr_idx)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	u32 uc_filter[EQOS_HTR_CNT + 1];
	struct netdev_hw_addr *ha = NULL;
	int crc32_val = 0;
	int ret = 0, i = 1;

	DBGPR_FILTER("-->eqos_prepare_uc_list\n");

	if (pdata->l2_filtering_mode) {
		DBGPR_FILTER
		    ("select HASH FILTERING for uc addresses: uc_count = %d\n",
		     netdev_uc_count(dev));
		ret = 1;
		memset(uc_filter, 0, sizeof(uc_filter));

		if (pdata->max_hash_table_size == 64) {
			netdev_for_each_uc_addr(ha, dev) {
				DBGPR_FILTER
				    ("uc addr[%d] = %#x:%#x:%#x:%#x:%#x:%#x\n",
				     i++, ha->addr[0], ha->addr[1], ha->addr[2],
				     ha->addr[3], ha->addr[4], ha->addr[5]);
				crc32_val =
				    (bitrev32(~crc32_le(~0, ha->addr, 6)) >>
				     26);
				uc_filter[crc32_val >> 5] |=
				    (1 << (crc32_val & 0x1F));
			}
		} else if (pdata->max_hash_table_size == 128) {
			netdev_for_each_uc_addr(ha, dev) {
				DBGPR_FILTER
				    ("uc addr[%d] = %#x:%#x:%#x:%#x:%#x:%#x\n",
				     i++, ha->addr[0], ha->addr[1], ha->addr[2],
				     ha->addr[3], ha->addr[4], ha->addr[5]);
				crc32_val =
				    (bitrev32(~crc32_le(~0, ha->addr, 6)) >>
				     25);
				uc_filter[crc32_val >> 5] |=
				    (1 << (crc32_val & 0x1F));
			}
		} else if (pdata->max_hash_table_size == 256) {
			netdev_for_each_uc_addr(ha, dev) {
				DBGPR_FILTER
				    ("uc addr[%d] = %#x:%#x:%#x:%#x:%#x:%#x\n",
				     i++, ha->addr[0], ha->addr[1], ha->addr[2],
				     ha->addr[3], ha->addr[4], ha->addr[5]);
				crc32_val =
				    (bitrev32(~crc32_le(~0, ha->addr, 6)) >>
				     24);
				uc_filter[crc32_val >> 5] |=
				    (1 << (crc32_val & 0x1F));
			}
		}

		/* configure hash value of real/default interface also */
		DBGPR_FILTER
		    ("real/default dev_addr = %#x:%#x:%#x:%#x:%#x:%#x\n",
		     dev->dev_addr[0], dev->dev_addr[1], dev->dev_addr[2],
		     dev->dev_addr[3], dev->dev_addr[4], dev->dev_addr[5]);

		if (pdata->max_hash_table_size == 64) {
			crc32_val =
			    (bitrev32(~crc32_le(~0, dev->dev_addr, 6)) >> 26);
			uc_filter[crc32_val >> 5] |= (1 << (crc32_val & 0x1F));
		} else if (pdata->max_hash_table_size == 128) {
			crc32_val =
			    (bitrev32(~crc32_le(~0, dev->dev_addr, 6)) >> 25);
			uc_filter[crc32_val >> 5] |= (1 << (crc32_val & 0x1F));

		} else if (pdata->max_hash_table_size == 256) {
			crc32_val =
			    (bitrev32(~crc32_le(~0, dev->dev_addr, 6)) >> 24);
			uc_filter[crc32_val >> 5] |= (1 << (crc32_val & 0x1F));
		}

		for (i = 0; i < EQOS_HTR_CNT; i++)
			hw_if->update_hash_table_reg(i, uc_filter[i]);

	} else {
		DBGPR_FILTER
		    ("select PERFECT FILTERING for uc addresses: uc_count = %d\n",
		     netdev_uc_count(dev));

		netdev_for_each_uc_addr(ha, dev) {
			DBGPR_FILTER("uc addr[%d] = %#x:%#x:%#x:%#x:%#x:%#x\n",
				     i, ha->addr[0], ha->addr[1], ha->addr[2],
				     ha->addr[3], ha->addr[4], ha->addr[5]);
			if (i < 32)
				hw_if->update_mac_addr1_31_low_high_reg(i,
									ha->
									addr);
			else
				hw_if->update_mac_addr32_127_low_high_reg(i,
									  ha->
									  addr);
			i++;
		}

		*mac_addr_idx = i;
	}

	DBGPR_FILTER("<--eqos_prepare_uc_list\n");

	return ret;
}

/*!
* \brief API to set the device receive mode
*
* \details The set_multicast_list function is called when the multicast list
* for the device changes and when the flags change.
*
* \param[in] dev - pointer to net_device structure.
*
* \return void
*/
static void eqos_set_rx_mode(struct net_device *dev)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	unsigned char pr_mode = 0;
	unsigned char huc_mode = 0;
	unsigned char hmc_mode = 0;
	unsigned char pm_mode = 0;
	unsigned char hpf_mode = 0;
	unsigned int mac_addr_idx = 1;
	int mode, i;

	DBGPR_FILTER("-->eqos_set_rx_mode\n");

	spin_lock_bh(&pdata->lock);

	if (dev->flags & IFF_PROMISC) {
		DBGPR_FILTER
		    ("PROMISCUOUS MODE (Accept all packets irrespective of DA)\n");
		pr_mode = 1;
#ifdef ENABLE_PERFECT_L2_FILTER
	} else if ((dev->flags & IFF_ALLMULTI)) {
#else
	} else if ((dev->flags & IFF_ALLMULTI) ||
		   (netdev_mc_count(dev) > (pdata->max_hash_table_size))) {
#endif
		DBGPR_FILTER("pass all multicast pkt\n");
		pm_mode = 1;
		if (pdata->max_hash_table_size) {
			for (i = 0; i < EQOS_HTR_CNT; i++)
				hw_if->update_hash_table_reg(i, 0xffffffff);
		}
	} else if (!netdev_mc_empty(dev)) {
		DBGPR_FILTER("pass list of multicast pkt\n");
		if ((netdev_mc_count(dev) > (pdata->max_addr_reg_cnt - 1)) &&
		    (!pdata->max_hash_table_size)) {
			/* switch to PROMISCUOUS mode */
			pr_mode = 1;
		} else {
			mode = eqos_prepare_mc_list(dev, &mac_addr_idx);
			if (mode) {
				/* Hash filtering for multicast */
				hmc_mode = 1;
			} else {
				/* Perfect filtering for multicast */
				hmc_mode = 0;
				hpf_mode = 1;
			}
		}
	}

	/* Handle multiple unicast addresses */
	if ((netdev_uc_count(dev) > (pdata->max_addr_reg_cnt - 1)) &&
	    (!pdata->max_hash_table_size)) {
		/* switch to PROMISCUOUS mode */
		pr_mode = 1;
	} else if (!netdev_uc_empty(dev)) {
		mode = eqos_prepare_uc_list(dev, &mac_addr_idx);
		if (mode) {
			/* Hash filtering for unicast */
			huc_mode = 1;
		} else {
			/* Perfect filtering for unicast */
			huc_mode = 0;
			hpf_mode = 1;
		}
	}

	/* reset filter MAC addresses which are deleted */
	if (pdata->mac_addr_idx > mac_addr_idx) {
		for (i = mac_addr_idx; i <= pdata->mac_addr_idx; i++) {
			if (i < 32)
				hw_if->update_mac_addr1_31_low_high_reg(i,
						NULL);
			else
				hw_if->update_mac_addr32_127_low_high_reg(i,
						NULL);
		}
	}

	pdata->mac_addr_idx = mac_addr_idx;

	hw_if->config_mac_pkt_filter_reg(pr_mode, huc_mode,
					 hmc_mode, pm_mode, hpf_mode);

	spin_unlock_bh(&pdata->lock);

	pr_debug("<--eqos_set_rx_mode\n");
}

static inline int eqos_tx_avail(struct tx_ring *ptx_ring)
{
	BUILD_BUG_ON_NOT_POWER_OF_2(TX_DESC_CNT);

	return (ptx_ring->dirty_tx - ptx_ring->cur_tx - 1) & (TX_DESC_CNT - 1);
}

/*!
* \brief API to transmit the packets
*
* \details The start_xmit function initiates the transmission of a packet.
* The full packet (protocol headers and all) is contained in a socket buffer
* (sk_buff) structure.
*
* \param[in] skb - pointer to sk_buff structure
* \param[in] dev - pointer to net_device structure
*
* \return integer
*
* \retval 0
*/

static int eqos_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);
	unsigned int qinx = skb_get_queue_mapping(skb);
	struct netdev_queue *txq = netdev_get_tx_queue(dev, qinx);
	struct tx_ring *ptx_ring = GET_TX_WRAPPER_DESC(qinx);
	struct s_tx_pkt_features *tx_pkt_features = GET_TX_PKT_FEATURES_PTR(qinx);
	struct desc_if_struct *desc_if = &pdata->desc_if;
	struct hw_if_struct *hw_if = &pdata->hw_if;
	INT retval = NETDEV_TX_OK;
	int cnt = 0;
	int tso;
	unsigned long timer_val;

	pr_debug("-->eqos_start_xmit: skb->len = %d, qinx = %u\n", skb->len, qinx);

	if (skb->len <= 0) {
		netdev_err(dev, "empty skb received from stack\n");
		dev_kfree_skb_any(skb);
		goto tx_netdev_return;
	}

	memset(tx_pkt_features, 0, sizeof(struct s_tx_pkt_features));

#ifdef EQOS_ENABLE_VLAN_TAG
	ptx_ring->vlan_tag_present = 0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 4, 0)
	if (vlan_tx_tag_present(skb)) {
		USHORT vlan_tag = vlan_tx_tag_get(skb);
#else
	if (skb_vlan_tag_present(skb)) {
		USHORT vlan_tag = skb_vlan_tag_get(skb);
#endif
		vlan_tag |= (skb->priority << 13);
		ptx_ring->vlan_tag_present = 1;
		ptx_ring->vlan_tag_id = vlan_tag;
		if (Y_TRUE == ptx_ring->tx_vlan_tag_via_reg) {
			pr_err("VLAN control info update via reg\n");
			hw_if->enable_vlan_reg_control(ptx_ring);
		} else {
			TX_PKT_FEATURES_PKT_ATTRIBUTES_VLAN_PKT_WR
			    (tx_pkt_features->pkt_attributes, 1);
			TX_PKT_FEATURES_VLAN_TAG_VT_WR
			    (tx_pkt_features->vlan_tag, vlan_tag);
		}
		pdata->xstats.tx_vlan_pkt_n++;
	}
#endif

	/* check for hw tstamping */
	if (pdata->hw_feat.tsstssel && pdata->hwts_tx_en) {
		if (skb_shinfo(skb)->tx_flags & SKBTX_HW_TSTAMP) {
			/* declare that device is doing timestamping */
			skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
			TX_PKT_FEATURES_PKT_ATTRIBUTES_PTP_ENABLE_WR
			    (tx_pkt_features->pkt_attributes, 1);
			DBGPR_PTP
			    ("Got PTP pkt to transmit [qinx = %d, cur_tx = %d]\n",
			     qinx, ptx_ring->cur_tx);
		}
	}

	tso = desc_if->handle_tso(dev, skb);
	if (tso < 0) {
		pr_err("Unable to handle TSO\n");
		dev_kfree_skb_any(skb);
		retval = NETDEV_TX_OK;
		goto tx_netdev_return;
	}
	if (tso) {
		pdata->xstats.tx_tso_pkt_n++;
		TX_PKT_FEATURES_PKT_ATTRIBUTES_TSO_ENABLE_WR(tx_pkt_features->
							     pkt_attributes, 1);
	} else if (skb->ip_summed == CHECKSUM_PARTIAL) {
		TX_PKT_FEATURES_PKT_ATTRIBUTES_CSUM_ENABLE_WR(tx_pkt_features->
							      pkt_attributes,
							      1);
	}

	cnt = desc_if->tx_swcx_alloc(dev, skb);
	if (cnt <= 0) {
		if (cnt == 0) {
			netif_stop_subqueue(dev, qinx);
			netdev_err(dev, "%s(): TX ring full for queue %d\n",
				   __func__, qinx);
			retval = NETDEV_TX_BUSY;
			goto tx_netdev_return;
		}
		dev_kfree_skb_any(skb);
		retval = NETDEV_TX_OK;
		goto tx_netdev_return;
	}

	txq->trans_start = jiffies;

#ifdef EQOS_ENABLE_TX_PKT_DUMP
	print_pkt(skb, skb->len, 1, (ptx_ring->cur_tx - 1));
#endif

	if ((pdata->eee_enabled) && (pdata->tx_path_in_lpi_mode) &&
	    (!pdata->use_lpi_tx_automate))
		eqos_disable_eee_mode(pdata);

	/* fallback to software time stamping if core doesn't
	 * support hardware time stamping */
	if ((pdata->hw_feat.tsstssel == 0) || (pdata->hwts_tx_en == 0))
		skb_tx_timestamp(skb);

	/* configure required descriptor fields for transmission */
	hw_if->pre_xmit(pdata, qinx);

	/* Stop the queue if there might not be enough descriptors for another
	 * packet.
	 */
	if (eqos_tx_avail(ptx_ring) <= EQOS_TX_DESC_THRESHOLD) {
		netif_stop_subqueue(dev, qinx);
		netdev_dbg(dev, "%s(): Stopping TX ring %d\n", __func__, qinx);
	}

	if (ptx_ring->use_tx_usecs == EQOS_COAELSCING_ENABLE &&
	    atomic_read(&pdata->tx_queue[qinx].tx_usecs_timer_armed) ==
	    EQOS_HRTIMER_DISABLE) {
		atomic_set(&pdata->tx_queue[qinx].tx_usecs_timer_armed,
			   EQOS_COAELSCING_ENABLE);
		timer_val = ptx_ring->tx_usecs * NSEC_PER_USEC;
		hrtimer_start(&pdata->tx_queue[qinx].tx_usecs_timer,
			      (ktime_set(0, timer_val)), HRTIMER_MODE_REL);
	}

tx_netdev_return:
	return retval;
}

static void eqos_print_rx_tstamp_info(struct s_rx_desc *rxdesc)
{
	u32 ptp_status = 0;
	u32 pkt_type = 0;
	char *tstamp_dropped = NULL;
	char *tstamp_available = NULL;
	char *ptp_version = NULL;
	char *ptp_pkt_type = NULL;
	char *ptp_msg_type = NULL;

	DBGPR_PTP("-->eqos_print_rx_tstamp_info\n");

	/* status in rdes1 is not valid */
	if (!(rxdesc->rdes3 & EQOS_RDESC3_RS1V))
		return;

	ptp_status = rxdesc->rdes1;
	tstamp_dropped = ((ptp_status & 0x8000) ? "YES" : "NO");
	tstamp_available = ((ptp_status & 0x4000) ? "YES" : "NO");
	ptp_version =
	    ((ptp_status & 0x2000) ? "v2 (1588-2008)" : "v1 (1588-2002)");
	ptp_pkt_type =
	    ((ptp_status & 0x1000) ? "ptp over Eth" : "ptp over IPv4/6");

	pkt_type = ((ptp_status & 0xF00) >> 8);
	switch (pkt_type) {
	case 0:
		ptp_msg_type = "NO PTP msg received";
		break;
	case 1:
		ptp_msg_type = "SYNC";
		break;
	case 2:
		ptp_msg_type = "Follow_Up";
		break;
	case 3:
		ptp_msg_type = "Delay_Req";
		break;
	case 4:
		ptp_msg_type = "Delay_Resp";
		break;
	case 5:
		ptp_msg_type = "Pdelay_Req";
		break;
	case 6:
		ptp_msg_type = "Pdelay_Resp";
		break;
	case 7:
		ptp_msg_type = "Pdelay_Resp_Follow_up";
		break;
	case 8:
		ptp_msg_type = "Announce";
		break;
	case 9:
		ptp_msg_type = "Management";
		break;
	case 10:
		ptp_msg_type = "Signaling";
		break;
	case 11:
	case 12:
	case 13:
	case 14:
		ptp_msg_type = "Reserved";
		break;
	case 15:
		ptp_msg_type = "PTP pkr with Reserved Msg Type";
		break;
	}

	DBGPR_PTP("Rx timestamp detail for queue %d\n"
		  "tstamp dropped    = %s\n"
		  "tstamp available  = %s\n"
		  "PTP version       = %s\n"
		  "PTP Pkt Type      = %s\n"
		  "PTP Msg Type      = %s\n",
		  qinx, tstamp_dropped, tstamp_available,
		  ptp_version, ptp_pkt_type, ptp_msg_type);

	DBGPR_PTP("<--eqos_print_rx_tstamp_info\n");
}

static inline int eqos_get_rx_tstamp_status(struct s_rx_desc *context_desc)
{
	if (likely(!(context_desc->rdes3 & EQOS_RDESC3_OWN) &&
		   (context_desc->rdes3 & EQOS_RDESC3_CTXT))) {
		if (unlikely((context_desc->rdes0 == 0xffffffff) &&
			     (context_desc->rdes1 == 0xffffffff)))
			return -EINVAL;

		return 0;
	}

	return -EBUSY;
}

/*!
* \brief API to get rx time stamp value.
*
* \details This function will read received packet's timestamp from
* the descriptor and pass it to stack and also perform some sanity checks.
*
* \param[in] pdata - pointer to private data structure.
* \param[in] skb - pointer to sk_buff structure.
* \param[in] prx_ring - pointer to wrapper receive descriptor structure.
* \param[in] qinx - Queue/Channel number.
*
* \return integer
*
* \retval 0 if no context descriptor
* \retval 1 if timestamp is valid
* \retval 2 if time stamp is corrupted
*/

static int eqos_get_rx_hwtstamp(struct eqos_prv_data *pdata,
				struct sk_buff *skb,
				struct s_rx_desc *prx_desc,
				struct s_rx_desc *context_desc)
{
	struct skb_shared_hwtstamps *shhwtstamp;
	int retry, ret = 0;
	u64 ns;

	if (!pdata->hw_feat.tsstssel || !pdata->hwts_rx_en)
		return -ENOTSUPP;

	if (unlikely(!(prx_desc->rdes3 & EQOS_RDESC3_RS1V) ||
		     !(prx_desc->rdes1 & EQOS_RDESC1_TSA)) ||
		      (prx_desc->rdes1 & EQOS_RDESC1_TD))
		return -ENOTSUPP;

	eqos_print_rx_tstamp_info(prx_desc);

	for (retry = 0; retry < 10; retry++) {
		ret = eqos_get_rx_tstamp_status(context_desc);
		if (ret == 0) {
			/* Timestamp can be read. */
			break;
		} else if (ret != -EBUSY) {
			netdev_err(pdata->dev,
				   "Failed to get RX timestamp: %d\n", ret);
			return ret;
		}

		netdev_dbg(pdata->dev,
			   "RX timestamp not yet available; retrying...\n");
	}
	if (ret != 0) {
		netdev_err(pdata->dev, "Timed out waiting for RX timestamp\n");
		return ret;
	}

	ns = context_desc->rdes0 + 1000000000ULL * context_desc->rdes1;
	shhwtstamp = skb_hwtstamps(skb);
	memset(shhwtstamp, 0, sizeof(struct skb_shared_hwtstamps));
	shhwtstamp->hwtstamp = ns_to_ktime(ns);

	pdata->xstats.rx_timestamp_captured_n++;

	return 0;
}

/*!
* \brief API to get tx time stamp value.
*
* \details This function will read timestamp from the descriptor
* and pass it to stack and also perform some sanity checks.
*
* \param[in] pdata - pointer to private data structure.
* \param[in] txdesc - pointer to transmit descriptor structure.
* \param[in] skb - pointer to sk_buff structure.
*
* \return integer
*
* \retval 1 if time stamp is taken
* \retval 0 if time stamp in not taken/valid
*/

static unsigned int eqos_get_tx_hwtstamp(struct eqos_prv_data *pdata,
					 struct s_tx_desc *txdesc,
					 struct sk_buff *skb)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct skb_shared_hwtstamps shhwtstamp;
	u64 ns;

	DBGPR_PTP("-->eqos_get_tx_hwtstamp\n");

	if (hw_if->drop_tx_status_enabled() == 0) {
		/* check tx tstamp status */
		if (!hw_if->get_tx_tstamp_status(txdesc)) {
			pr_err
			    ("tx timestamp is not captured for this packet\n");
			return 0;
		}

		/* get the valid tstamp */
		ns = hw_if->get_tx_tstamp(txdesc);
	} else {
		/* drop tx status mode is enabled, hence read time
		 * stamp from register instead of descriptor */

		/* check tx tstamp status */
		if (!hw_if->get_tx_tstamp_status_via_reg()) {
			pr_err
			    ("tx timestamp is not captured for this packet\n");
			return 0;
		}

		/* get the valid tstamp */
		ns = hw_if->get_tx_tstamp_via_reg();
	}

	pdata->xstats.tx_timestamp_captured_n++;
	memset(&shhwtstamp, 0, sizeof(struct skb_shared_hwtstamps));
	shhwtstamp.hwtstamp = ns_to_ktime(ns);
	/* pass tstamp to stack */
	skb_tstamp_tx(skb, &shhwtstamp);

	DBGPR_PTP("<--eqos_get_tx_hwtstamp\n");

	return 1;
}

/*!
* \brief API to update the tx status.
*
* \details This function is called in isr handler once after getting
* transmit complete interrupt to update the transmited packet status
* and it does some house keeping work like updating the
* private data structure variables.
*
* \param[in] dev - pointer to net_device structure
* \param[in] pdata - pointer to private data structure.
*
* \return void
*/

static int process_tx_completions(struct eqos_tx_queue *tx_queue, int budget)
{
	struct eqos_prv_data *pdata = tx_queue->pdata;
	unsigned int qinx = tx_queue->chan_num;
	struct net_device *dev = pdata->dev;
	struct netdev_queue *txq = netdev_get_tx_queue(dev, qinx);
	struct tx_ring *ptx_ring = GET_TX_WRAPPER_DESC(qinx);
	struct desc_if_struct *desc_if = &pdata->desc_if;
	struct hw_if_struct *hw_if = &pdata->hw_if;
	struct tx_swcx_desc *ptx_swcx_desc = NULL;
	struct s_tx_desc *ptx_desc = NULL;
	int entry = ptx_ring->dirty_tx;
	unsigned int tstamp_taken = 0;
	int err_incremented;
	int processed = 0;

	pr_debug("-->%s(): dirty_tx = %d, qinx = %u\n", __func__, entry, qinx);

	pdata->xstats.tx_clean_n[qinx]++;
	while (entry != ptx_ring->cur_tx && processed < budget) {
		ptx_desc = GET_TX_DESC_PTR(qinx, entry);
		ptx_swcx_desc = GET_TX_BUF_PTR(qinx, entry);
		tstamp_taken = 0;

		/* ensure we actually got Tx desc */
		rmb();
		if (!hw_if->tx_complete(ptx_desc))
			break;

#ifdef EQOS_ENABLE_TX_DESC_DUMP
		dump_tx_desc(pdata, entry, entry, 0, qinx);
#endif

		/* update the tx error if any by looking at last segment
		 * for NORMAL descriptors
		 * */
		if ((hw_if->get_tx_desc_ls(ptx_desc)) &&
		    !(hw_if->get_tx_desc_ctxt(ptx_desc))) {
			if (ptx_swcx_desc->skb == NULL) {
				dev_err(&pdata->pdev->dev,
				"NULL SKB in process_tx_completions()\n");
			}
			/* check whether skb support hw tstamp */
			if ((pdata->hw_feat.tsstssel) &&
			    (skb_shinfo(ptx_swcx_desc->skb)->
			     tx_flags & SKBTX_IN_PROGRESS)) {
				tstamp_taken =
				    eqos_get_tx_hwtstamp(pdata, ptx_desc,
							 ptx_swcx_desc->skb);
				if (tstamp_taken) {
					DBGPR_PTP
					    ("passed tx timestamp to stack[qinx = %d, dirty_tx = %d]\n",
					     qinx, entry);
				}
			}

			err_incremented = 0;
			if (hw_if->tx_window_error) {
				if (hw_if->tx_window_error(ptx_desc)) {
					err_incremented = 1;
					dev->stats.tx_window_errors++;
				}
			}
			if (hw_if->tx_aborted_error) {
				if (hw_if->tx_aborted_error(ptx_desc)) {
					err_incremented = 1;
					dev->stats.tx_aborted_errors++;
					if (hw_if->tx_handle_aborted_error)
						hw_if->
						    tx_handle_aborted_error
						    (ptx_desc);
				}
			}
			if (hw_if->tx_carrier_lost_error) {
				if (hw_if->tx_carrier_lost_error(ptx_desc)) {
					err_incremented = 1;
					dev->stats.tx_carrier_errors++;
				}
			}
			if (hw_if->tx_fifo_underrun) {
				if (hw_if->tx_fifo_underrun(ptx_desc)) {
					err_incremented = 1;
					dev->stats.tx_fifo_errors++;
					if (hw_if->tx_update_fifo_threshold)
						hw_if->
						    tx_update_fifo_threshold
						    (ptx_desc);
				}
			}
			if (hw_if->tx_get_collision_count)
				dev->stats.collisions +=
				    hw_if->tx_get_collision_count(ptx_desc);

			if (err_incremented == 1)
				dev->stats.tx_errors++;

			pdata->xstats.q_tx_pkt_n[qinx]++;
			pdata->xstats.tx_pkt_n++;
			dev->stats.tx_packets++;
			processed++;
		}

		/* CTXT descriptors set their len to -1, which is an unsigned
		 * 16bit field. In case of VLAN, the HW is inserting the vlan
		 * tag (type + tag) so we should account for that in our
		 * transmitted total.
		 */
		if (ptx_swcx_desc->len != (unsigned short)-1) {
			dev->stats.tx_bytes += ptx_swcx_desc->len;
		} else if (hw_if->get_tx_desc_ctxt(ptx_desc)) {
			unsigned int vltv;
			TX_CONTEXT_DESC_TDES3_VLTV_RD(ptx_desc->tdes3, vltv);
			if (vltv == 0x1)
				dev->stats.tx_bytes += VLAN_HLEN;
		}

		desc_if->tx_swcx_free(pdata, ptx_swcx_desc);

		/* reset the descriptor so that driver/host can reuse it */
		hw_if->tx_desc_reset(entry, pdata, qinx);

		INCR_TX_DESC_INDEX(entry, 1);
	}

	__netif_tx_lock(txq, smp_processor_id());
	/* Update the dirty pointer and wake up the TX queue, if necessary. */
	ptx_ring->dirty_tx = entry;
	if (netif_tx_queue_stopped(txq) &&
	    eqos_tx_avail(ptx_ring) > EQOS_TX_DESC_THRESHOLD) {
		netif_tx_wake_queue(txq);
	}

	if ((pdata->eee_enabled) && (!pdata->tx_path_in_lpi_mode) &&
	    (!pdata->use_lpi_tx_automate)) {
		eqos_enable_eee_mode(pdata);
		mod_timer(&pdata->eee_ctrl_timer,
			  EQOS_LPI_TIMER(EQOS_DEFAULT_LPI_TIMER));
	}
	__netif_tx_unlock(txq);

	return processed;
}

#ifdef YDEBUG_FILTER
static void eqos_check_rx_filter_status(struct s_rx_desc *prx_desc)
{
	u32 rdes2 = prx_desc->rdes2;
	u32 rdes3 = prx_desc->rdes3;

	/* Receive Status rdes2 Valid ? */
	if ((rdes3 & 0x8000000) == 0x8000000) {
		if ((rdes2 & 0x400) == 0x400)
			pr_err("ARP pkt received\n");
		if ((rdes2 & 0x800) == 0x800)
			pr_err("ARP reply not generated\n");
		if ((rdes2 & 0x8000) == 0x8000)
			pr_err("VLAN pkt passed VLAN filter\n");
		if ((rdes2 & 0x10000) == 0x10000)
			pr_err("SA Address filter fail\n");
		if ((rdes2 & 0x20000) == 0x20000)
			pr_err("DA Addess filter fail\n");
		if ((rdes2 & 0x40000) == 0x40000)
			pr_err
			    ("pkt passed the HASH filter in MAC and HASH value = %#x\n",
			     (rdes2 >> 19) & 0xff);
		if ((rdes2 & 0x8000000) == 0x8000000)
			pr_err("L3 filter(%d) Match\n", ((rdes2 >> 29) & 0x7));
		if ((rdes2 & 0x10000000) == 0x10000000)
			pr_err("L4 filter(%d) Match\n", ((rdes2 >> 29) & 0x7));
	}
}
#endif				/* YDEBUG_FILTER */

/* pass skb to upper layer */
static void eqos_receive_skb(struct eqos_prv_data *pdata,
			     struct net_device *dev, struct sk_buff *skb,
			     UINT qinx)
{
	struct eqos_rx_queue *rx_queue = GET_RX_QUEUE_PTR(qinx);

	skb_record_rx_queue(skb, qinx);
	skb->dev = dev;
	skb->protocol = eth_type_trans(skb, dev);

#if LINUX_VERSION_CODE <= KERNEL_VERSION(4, 9, 0)
	dev->last_rx = jiffies;
#endif
	dev->stats.rx_packets++;
	dev->stats.rx_bytes += skb->len;

	if (dev->features & NETIF_F_GRO)
		napi_gro_receive(&rx_queue->napi, skb);
	else
		netif_receive_skb(skb);
}

/* Receive Checksum Offload configuration */
static inline void eqos_get_rx_csum(struct eqos_prv_data *pdata,
				    struct sk_buff *skb,
				    struct s_rx_desc *prx_desc)
{
	skb->ip_summed = CHECKSUM_NONE;

	if (unlikely(!(pdata->dev_state & NETIF_F_RXCSUM)))
		return;

	if ((prx_desc->rdes3 & EQOS_RDESC3_RS1V) &&
	    !(prx_desc->rdes1 & (EQOS_RDESC1_IPCE | EQOS_RDESC1_IPCB |
				 EQOS_RDESC1_IPHE))) {
		skb->ip_summed = CHECKSUM_UNNECESSARY;
	}
}

static inline void eqos_get_rx_vlan(struct eqos_prv_data *pdata,
				    struct sk_buff *skb,
				    struct s_rx_desc *prx_desc)
{
	if (unlikely(!(pdata->dev_state & NETIF_F_HW_VLAN_CTAG_RX)))
		return;

	/* Receive Status rdes0 Valid ? */
	if (prx_desc->rdes3 & EQOS_RDESC3_RS0V) {
		u32 lt = prx_desc->rdes3 & EQOS_RDESC3_LT;

		if (lt == EQOS_RDESC3_LT_VT || lt == EQOS_RDESC3_LT_DVT) {
			u16 vlan_tag = prx_desc->rdes0 & EQOS_RDESC0_OVT;

			__vlan_hwaccel_put_tag(skb, htons(ETH_P_8021Q),
					       vlan_tag);
			pdata->xstats.rx_vlan_pkt_n++;
		}
	}
}

static inline int eqos_rx_dirty(struct rx_ring *prx_ring)
{
	BUILD_BUG_ON_NOT_POWER_OF_2(RX_DESC_CNT);

	return (prx_ring->cur_rx - prx_ring->dirty_rx) & (RX_DESC_CNT - 1);
}
/*!
* \brief API to pass the Rx packets to stack if default mode
* is enabled.
*
* \details This function is invoked by main NAPI function in default
* Rx mode. This function checks the
* device descriptor for the packets and passes it to stack if any packtes
* are received by device.
*
* \param[in] pdata - pointer to private data structure.
* \param[in] quota - maximum no. of packets that we are allowed to pass
* to into the kernel.
* \param[in] qinx - DMA channel/queue no. to be checked for packet.
*
* \return integer
*
* \retval number of packets received.
*/

static int process_rx_completions(struct eqos_rx_queue *rx_queue, int quota)
{
	struct eqos_prv_data *pdata = rx_queue->pdata;
	unsigned int qinx = rx_queue->chan_num;
	struct rx_ring *prx_ring =
	    GET_RX_WRAPPER_DESC(qinx);
	struct desc_if_struct *desc_if = &pdata->desc_if;
	struct net_device *dev = pdata->dev;
	int received = 0;
	int received_resv = 0;
	int ret;

	pr_debug("-->%s(): qinx = %u, quota = %d\n", __func__, qinx, quota);

	while (received < quota && received < RX_DESC_CNT &&
	       received_resv < quota) {
		struct rx_swcx_desc *prx_swcx_desc;
		struct s_rx_desc *prx_desc, *context_desc;
		struct sk_buff *skb;
		u32 status, pkt_len;
		int entry = prx_ring->cur_rx;

		prx_swcx_desc = GET_RX_BUF_PTR(qinx, entry);
		prx_desc = GET_RX_DESC_PTR(qinx, entry);

		status = prx_desc->rdes3;
		if (status & EQOS_RDESC3_OWN)
			break;

		INCR_RX_DESC_INDEX(prx_ring->cur_rx, 1);

		if (unlikely(prx_swcx_desc->skb == pdata->resv_skb)) {
			pr_debug("%s(): Reserved SKB used\n", __func__);
			prx_swcx_desc->skb = NULL;
			prx_swcx_desc->dma = 0;
			/* Reservered skb used */
			received_resv++;
			/* This is unlikely case so try to
			 *  get memory whenever we hit this loop
			 */
			desc_if->realloc_skb(pdata, qinx);
			continue;
		}

#ifdef EQOS_ENABLE_RX_DESC_DUMP
		dump_rx_desc(qinx, prx_desc, entry);
#endif
		if (likely(!(status & EQOS_RDESC3_ES_BITS) &&
			   (status & EQOS_RDESC3_LD))) {
			/* Unmap the SKB */
			skb = prx_swcx_desc->skb;
			prx_swcx_desc->skb = NULL;

			dma_unmap_single(&pdata->pdev->dev, prx_swcx_desc->dma,
					 pdata->rx_buffer_len, DMA_FROM_DEVICE);

			prx_swcx_desc->dma = 0;

			pkt_len = (status & EQOS_RDESC3_PL);
			skb_put(skb, pkt_len);

#ifdef EQOS_ENABLE_RX_PKT_DUMP
			print_pkt(skb, pkt_len, 0, entry);
#endif
			eqos_get_rx_csum(pdata, skb, prx_desc);

			eqos_get_rx_vlan(pdata, skb, prx_desc);

			#ifdef YDEBUG_FILTER
			eqos_check_rx_filter_status(prx_desc);
#endif
			context_desc = GET_RX_DESC_PTR(qinx, prx_ring->cur_rx);
			ret = eqos_get_rx_hwtstamp(pdata, skb, prx_desc,
						   context_desc);
			if (ret == 0) {
				/* Context descriptor was consumed. Its skb
				 * and DMA mapping will be recycled.
				 */
				INCR_RX_DESC_INDEX(prx_ring->cur_rx, 1);
			}

			eqos_receive_skb(pdata, dev, skb, qinx);
		} else {
			eqos_update_rx_errors(dev, status);
			dev_kfree_skb_any(prx_swcx_desc->skb);
		}

		received++;
		if (eqos_rx_dirty(prx_ring) >=
		    prx_ring->skb_realloc_threshold)
			desc_if->realloc_skb(pdata, qinx);
	}

	desc_if->realloc_skb(pdata, qinx);

	pdata->xstats.rx_pkt_n += received;
	pdata->xstats.q_rx_pkt_n[qinx] += received;

	return received;
}

void eqos_update_rx_errors(struct net_device *dev, unsigned int rx_status)
{
	if (rx_status & EQOS_RDESC3_LD) {
		/* CRC Error */
		if (rx_status & EQOS_RDESC3_CRC)
			dev->stats.rx_crc_errors++;
		/* Receive Error */
		if (rx_status & EQOS_RDESC3_RE)
			dev->stats.rx_frame_errors++;
		/* RX FIFO Overrun */
		if (rx_status & EQOS_RDESC3_OF)
			dev->stats.rx_fifo_errors++;
	} else {
		/* Packet truncated */
		dev->stats.rx_over_errors++;
	}

	dev->stats.rx_errors++;
}

int eqos_napi_poll_rx(struct napi_struct *napi, int budget)
{
	struct eqos_rx_queue *rx_queue =
	    container_of(napi, struct eqos_rx_queue, napi);
	struct eqos_prv_data *pdata = rx_queue->pdata;
	int qinx = rx_queue->chan_num;
	int received = 0;

	received = process_rx_completions(rx_queue, budget);
	if (received < budget) {
		napi_complete(napi);
		eqos_enable_chan_rx_interrupt(pdata, qinx);
	}

	return received;
}

static inline int eqos_txring_empty(struct tx_ring *ptx_ring)
{
	return (ptx_ring->dirty_tx == ptx_ring->cur_tx);
}

int eqos_napi_poll_tx(struct napi_struct *napi, int budget)
{
	struct eqos_tx_queue *tx_queue =
	    container_of(napi, struct eqos_tx_queue, napi);
	struct eqos_prv_data *pdata = tx_queue->pdata;
	int qinx = tx_queue->chan_num;
	struct tx_ring *ptx_ring = GET_TX_WRAPPER_DESC(qinx);
	int processed;
	unsigned long timer_val;

	processed = process_tx_completions(tx_queue, budget);
	/* re-arm the timer if tx ring is not empty */
	if ((!eqos_txring_empty(ptx_ring)) &&
	    (ptx_ring->use_tx_usecs == EQOS_COAELSCING_ENABLE) &&
	     (atomic_read(&tx_queue->tx_usecs_timer_armed) ==
	     EQOS_HRTIMER_DISABLE)) {
		timer_val = ptx_ring->tx_usecs * NSEC_PER_USEC;
		atomic_set(&tx_queue->tx_usecs_timer_armed,
			   EQOS_HRTIMER_ENABLE);
		hrtimer_start(&pdata->tx_queue[qinx].tx_usecs_timer,
			      (ktime_set(0, timer_val)), HRTIMER_MODE_REL);
	}

	if (processed < budget) {
		napi_complete(napi);
		eqos_enable_chan_tx_interrupt(pdata, qinx);
	}

	return processed;
}

static inline void eqos_enable_slot_function_ctrl(struct eqos_prv_data *pdata)
{
	struct hw_if_struct *hw_if = &pdata->hw_if;
	int qinx;

	for (qinx = 0; qinx < pdata->num_chans; qinx++) {
		struct eqos_tx_queue *tx_queue = GET_TX_QUEUE_PTR(qinx);

		if (tx_queue->slot_num_check) {
			pr_info("slot number function enabled for Queue=%d\n",
				qinx);
			hw_if->config_slot_num_check(qinx, 1);
		}
	}
}

/*!
* \brief API to return the device/interface status.
*
* \details The get_stats function is called whenever an application needs to
* get statistics for the interface. For example, this happend when ifconfig
* or netstat -i is run.
*
* \param[in] dev - pointer to net_device structure.
*
* \return net_device_stats structure
*
* \retval net_device_stats - returns pointer to net_device_stats structure.
*/

static struct net_device_stats *eqos_get_stats(struct net_device *dev)
{

	return &dev->stats;
}


/*!
 * \brief User defined parameter setting API
 *
 * \details This function is invoked by kernel to update the device
 * configuration to new features. This function supports enabling and
 * disabling of TX and RX csum features.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] features – device feature to be enabled/disabled.
 *
 * \return int
 *
 * \retval 0
 */

static int eqos_set_features(struct net_device *dev, netdev_features_t features)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	UINT dev_rxcsum_enable;
#ifdef EQOS_ENABLE_VLAN_TAG
	UINT dev_rxvlan_enable, dev_txvlan_enable;
#endif
	pr_debug("-->eqos_set_features\n");

	if (pdata->hw_feat.rx_coe_sel) {
		dev_rxcsum_enable = !!(pdata->dev_state & NETIF_F_RXCSUM);

		if (((features & NETIF_F_RXCSUM) == NETIF_F_RXCSUM)
		    && !dev_rxcsum_enable) {
			hw_if->enable_rx_csum();
			pdata->dev_state |= NETIF_F_RXCSUM;
			pr_err("State change - rxcsum enable\n");
		} else if (((features & NETIF_F_RXCSUM) == 0)
			   && dev_rxcsum_enable) {
			hw_if->disable_rx_csum();
			pdata->dev_state &= ~NETIF_F_RXCSUM;
			pr_err("State change - rxcsum disable\n");
		}
	}
#ifdef EQOS_ENABLE_VLAN_TAG
	dev_rxvlan_enable = !!(pdata->dev_state & NETIF_F_HW_VLAN_CTAG_RX);
	if (((features & NETIF_F_HW_VLAN_CTAG_RX) == NETIF_F_HW_VLAN_CTAG_RX)
	    && !dev_rxvlan_enable) {
		pdata->dev_state |= NETIF_F_HW_VLAN_CTAG_RX;
		hw_if->
		    config_rx_outer_vlan_stripping(EQOS_RX_VLAN_STRIP_ALWAYS);
		pr_err("State change - rxvlan enable\n");
	} else if (((features & NETIF_F_HW_VLAN_CTAG_RX) == 0) &&
		   dev_rxvlan_enable) {
		pdata->dev_state &= ~NETIF_F_HW_VLAN_CTAG_RX;
		hw_if->config_rx_outer_vlan_stripping(EQOS_RX_NO_VLAN_STRIP);
		pr_err("State change - rxvlan disable\n");
	}

	dev_txvlan_enable = !!(pdata->dev_state & NETIF_F_HW_VLAN_CTAG_TX);
	if (((features & NETIF_F_HW_VLAN_CTAG_TX) == NETIF_F_HW_VLAN_CTAG_TX)
	    && !dev_txvlan_enable) {
		pdata->dev_state |= NETIF_F_HW_VLAN_CTAG_TX;
		pr_err("State change - txvlan enable\n");
	} else if (((features & NETIF_F_HW_VLAN_CTAG_TX) == 0) &&
		   dev_txvlan_enable) {
		pdata->dev_state &= ~NETIF_F_HW_VLAN_CTAG_TX;
		pr_err("State change - txvlan disable\n");
	}
#endif				/* EQOS_ENABLE_VLAN_TAG */

	pr_debug("<--eqos_set_features\n");

	return 0;
}

/*!
 * \details This function is invoked by ioctl function when user issues
 * an ioctl command to enable/disable L3/L4 filtering.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] flags – flag to indicate whether L3/L4 filtering to be
 *                  enabled/disabled.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
static int eqos_config_l3_l4_filtering(struct net_device *dev,
				       unsigned int flags)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	int ret = 0;

	DBGPR_FILTER("-->eqos_config_l3_l4_filtering\n");

	if (flags && pdata->l3_l4_filter) {
		pr_err("L3/L4 filtering is already enabled\n");
		return -EINVAL;
	}

	if (!flags && !pdata->l3_l4_filter) {
		pr_err("L3/L4 filtering is already disabled\n");
		return -EINVAL;
	}

	pdata->l3_l4_filter = !!flags;
	hw_if->config_l3_l4_filter_enable(pdata->l3_l4_filter);

	DBGPR_FILTER("Succesfully %s L3/L4 filtering\n",
		     (flags ? "ENABLED" : "DISABLED"));

	DBGPR_FILTER("<--eqos_config_l3_l4_filtering\n");

	return ret;
}

/*!
 * \details This function is invoked by ioctl function when user issues an
 * ioctl command to configure L3(IPv4) filtering. This function does following,
 * - enable/disable IPv4 filtering.
 * - select source/destination address matching.
 * - select perfect/inverse matching.
 * - Update the IPv4 address into MAC register.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] req – pointer to IOCTL specific structure.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
static int eqos_config_ip4_filters(struct net_device *dev,
				   struct ifr_data_struct *req)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct eqos_l3_l4_filter *u_l3_filter =
	    (struct eqos_l3_l4_filter *)req->ptr;
	struct eqos_l3_l4_filter l_l3_filter;
	int ret = 0;

	DBGPR_FILTER("-->eqos_config_ip4_filters\n");

	if (pdata->hw_feat.l3l4_filter_num == 0)
		return EQOS_NO_HW_SUPPORT;

	if (copy_from_user(&l_l3_filter, u_l3_filter,
			   sizeof(struct eqos_l3_l4_filter)))
		return -EFAULT;

	if ((l_l3_filter.filter_no + 1) > pdata->hw_feat.l3l4_filter_num) {
		pr_err("%d filter is not supported in the HW\n",
		       l_l3_filter.filter_no);
		return EQOS_NO_HW_SUPPORT;
	}

	if (!pdata->l3_l4_filter) {
		hw_if->config_l3_l4_filter_enable(1);
		pdata->l3_l4_filter = 1;
	}

	/* configure the L3 filters */
	hw_if->config_l3_filters(l_l3_filter.filter_no,
				 l_l3_filter.filter_enb_dis, 0,
				 l_l3_filter.src_dst_addr_match,
				 l_l3_filter.perfect_inverse_match);

	if (!l_l3_filter.src_dst_addr_match)
		hw_if->update_ip4_addr0(l_l3_filter.filter_no,
					l_l3_filter.ip4_addr);
	else
		hw_if->update_ip4_addr1(l_l3_filter.filter_no,
					l_l3_filter.ip4_addr);

	DBGPR_FILTER
	    ("Successfully %s IPv4 %s %s addressing filtering on %d filter\n",
	     (l_l3_filter.filter_enb_dis ? "ENABLED" : "DISABLED"),
	     (l_l3_filter.perfect_inverse_match ? "INVERSE" : "PERFECT"),
	     (l_l3_filter.src_dst_addr_match ? "DESTINATION" : "SOURCE"),
	     l_l3_filter.filter_no);

	DBGPR_FILTER("<--eqos_config_ip4_filters\n");

	return ret;
}

/*!
 * \details This function is invoked by ioctl function when user issues an
 * ioctl command to configure L3(IPv6) filtering. This function does following,
 * - enable/disable IPv6 filtering.
 * - select source/destination address matching.
 * - select perfect/inverse matching.
 * - Update the IPv6 address into MAC register.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] req – pointer to IOCTL specific structure.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
static int eqos_config_ip6_filters(struct net_device *dev,
				   struct ifr_data_struct *req)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct eqos_l3_l4_filter *u_l3_filter =
	    (struct eqos_l3_l4_filter *)req->ptr;
	struct eqos_l3_l4_filter l_l3_filter;
	int ret = 0;

	DBGPR_FILTER("-->eqos_config_ip6_filters\n");

	if (pdata->hw_feat.l3l4_filter_num == 0)
		return EQOS_NO_HW_SUPPORT;

	if (copy_from_user(&l_l3_filter, u_l3_filter,
			   sizeof(struct eqos_l3_l4_filter)))
		return -EFAULT;

	if ((l_l3_filter.filter_no + 1) > pdata->hw_feat.l3l4_filter_num) {
		pr_err("%d filter is not supported in the HW\n",
		       l_l3_filter.filter_no);
		return EQOS_NO_HW_SUPPORT;
	}

	if (!pdata->l3_l4_filter) {
		hw_if->config_l3_l4_filter_enable(1);
		pdata->l3_l4_filter = 1;
	}

	/* configure the L3 filters */
	hw_if->config_l3_filters(l_l3_filter.filter_no,
				 l_l3_filter.filter_enb_dis, 1,
				 l_l3_filter.src_dst_addr_match,
				 l_l3_filter.perfect_inverse_match);

	hw_if->update_ip6_addr(l_l3_filter.filter_no, l_l3_filter.ip6_addr);

	DBGPR_FILTER
	    ("Successfully %s IPv6 %s %s addressing filtering on %d filter\n",
	     (l_l3_filter.filter_enb_dis ? "ENABLED" : "DISABLED"),
	     (l_l3_filter.perfect_inverse_match ? "INVERSE" : "PERFECT"),
	     (l_l3_filter.src_dst_addr_match ? "DESTINATION" : "SOURCE"),
	     l_l3_filter.filter_no);

	DBGPR_FILTER("<--eqos_config_ip6_filters\n");

	return ret;
}

/*!
 * \details This function is invoked by ioctl function when user issues an
 * ioctl command to configure L4(TCP/UDP) filtering. This function does following,
 * - enable/disable L4 filtering.
 * - select TCP/UDP filtering.
 * - select source/destination port matching.
 * - select perfect/inverse matching.
 * - Update the port number into MAC register.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] req – pointer to IOCTL specific structure.
 * \param[in] tcp_udp – flag to indicate TCP/UDP filtering.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
static int eqos_config_tcp_udp_filters(struct net_device *dev,
				       struct ifr_data_struct *req, int tcp_udp)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct eqos_l3_l4_filter *u_l4_filter =
	    (struct eqos_l3_l4_filter *)req->ptr;
	struct eqos_l3_l4_filter l_l4_filter;
	int ret = 0;

	DBGPR_FILTER("-->eqos_config_tcp_udp_filters\n");

	if (pdata->hw_feat.l3l4_filter_num == 0)
		return EQOS_NO_HW_SUPPORT;

	if (copy_from_user(&l_l4_filter, u_l4_filter,
			   sizeof(struct eqos_l3_l4_filter)))
		return -EFAULT;

	if ((l_l4_filter.filter_no + 1) > pdata->hw_feat.l3l4_filter_num) {
		pr_err("%d filter is not supported in the HW\n",
		       l_l4_filter.filter_no);
		return EQOS_NO_HW_SUPPORT;
	}

	if (!pdata->l3_l4_filter) {
		hw_if->config_l3_l4_filter_enable(1);
		pdata->l3_l4_filter = 1;
	}

	/* configure the L4 filters */
	hw_if->config_l4_filters(l_l4_filter.filter_no,
				 l_l4_filter.filter_enb_dis,
				 tcp_udp,
				 l_l4_filter.src_dst_addr_match,
				 l_l4_filter.perfect_inverse_match);

	if (l_l4_filter.src_dst_addr_match)
		hw_if->update_l4_da_port_no(l_l4_filter.filter_no,
					    l_l4_filter.port_no);
	else
		hw_if->update_l4_sa_port_no(l_l4_filter.filter_no,
					    l_l4_filter.port_no);

	DBGPR_FILTER
	    ("Successfully %s %s %s %s Port number filtering on %d filter\n",
	     (l_l4_filter.filter_enb_dis ? "ENABLED" : "DISABLED"),
	     (tcp_udp ? "UDP" : "TCP"),
	     (l_l4_filter.perfect_inverse_match ? "INVERSE" : "PERFECT"),
	     (l_l4_filter.src_dst_addr_match ? "DESTINATION" : "SOURCE"),
	     l_l4_filter.filter_no);

	DBGPR_FILTER("<--eqos_config_tcp_udp_filters\n");

	return ret;
}

/*!
 * \details This function is invoked by ioctl function when user issues an
 * ioctl command to configure VALN filtering. This function does following,
 * - enable/disable VLAN filtering.
 * - select perfect/hash filtering.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] req – pointer to IOCTL specific structure.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
static int eqos_config_vlan_filter(struct net_device *dev,
				   struct ifr_data_struct *req)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct eqos_vlan_filter *u_vlan_filter =
	    (struct eqos_vlan_filter *)req->ptr;
	struct eqos_vlan_filter l_vlan_filter;
	int ret = 0;

	DBGPR_FILTER("-->eqos_config_vlan_filter\n");

	if (copy_from_user(&l_vlan_filter, u_vlan_filter,
			   sizeof(struct eqos_vlan_filter)))
		return -EFAULT;

	if ((l_vlan_filter.perfect_hash) && (pdata->hw_feat.vlan_hash_en == 0)) {
		pr_err("VLAN HASH filtering is not supported\n");
		return EQOS_NO_HW_SUPPORT;
	}

	/* configure the vlan filter */
	hw_if->config_vlan_filtering(l_vlan_filter.filter_enb_dis,
				     l_vlan_filter.perfect_hash,
				     l_vlan_filter.perfect_inverse_match);
	pdata->vlan_hash_filtering = l_vlan_filter.perfect_hash;

	DBGPR_FILTER("Successfully %s VLAN %s filtering and %s matching\n",
		     (l_vlan_filter.filter_enb_dis ? "ENABLED" : "DISABLED"),
		     (l_vlan_filter.perfect_hash ? "HASH" : "PERFECT"),
		     (l_vlan_filter.
		      perfect_inverse_match ? "INVERSE" : "PERFECT"));

	DBGPR_FILTER("<--eqos_config_vlan_filter\n");

	return ret;
}

/*!
 * \details This function is invoked by ioctl function when user issues an
 * ioctl command to enable/disable ARP offloading feature.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] req – pointer to IOCTL specific structure.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
static int eqos_config_arp_offload(struct net_device *dev,
				   struct ifr_data_struct *req)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct eqos_arp_offload *u_arp_offload =
	    (struct eqos_arp_offload *)req->ptr;
	struct eqos_arp_offload l_arp_offload;
	int ret = 0;

	pr_err("-->eqos_config_arp_offload\n");

	if (pdata->hw_feat.arp_offld_en == 0)
		return EQOS_NO_HW_SUPPORT;

	if (copy_from_user(&l_arp_offload, u_arp_offload,
			   sizeof(struct eqos_arp_offload)))
		return -EFAULT;

	/* configure the L3 filters */
	hw_if->config_arp_offload(req->flags);
	hw_if->update_arp_offload_ip_addr(l_arp_offload.ip_addr);
	pdata->arp_offload = req->flags;

	pr_err("Successfully %s arp Offload\n",
	       (req->flags ? "ENABLED" : "DISABLED"));

	pr_err("<--eqos_config_arp_offload\n");

	return ret;
}

/*!
 * \details This function is invoked by ioctl function when user issues an
 * ioctl command to configure L2 destination addressing filtering mode. This
 * function dose following,
 * - selects perfect/hash filtering.
 * - selects perfect/inverse matching.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] req – pointer to IOCTL specific structure.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
static int eqos_confing_l2_da_filter(struct net_device *dev,
				     struct ifr_data_struct *req)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct eqos_l2_da_filter *u_l2_da_filter =
	    (struct eqos_l2_da_filter *)req->ptr;
	struct eqos_l2_da_filter l_l2_da_filter;
	int ret = 0;

	DBGPR_FILTER("-->eqos_confing_l2_da_filter\n");

	if (copy_from_user(&l_l2_da_filter, u_l2_da_filter,
			   sizeof(struct eqos_l2_da_filter)))
		return -EFAULT;

	if (l_l2_da_filter.perfect_hash) {
		if (pdata->hw_feat.hash_tbl_sz > 0)
			pdata->l2_filtering_mode = 1;
		else
			ret = EQOS_NO_HW_SUPPORT;
	} else {
		if (pdata->max_addr_reg_cnt > 1)
			pdata->l2_filtering_mode = 0;
		else
			ret = EQOS_NO_HW_SUPPORT;
	}

	/* configure L2 DA perfect/inverse_matching */
	hw_if->config_l2_da_perfect_inverse_match(l_l2_da_filter.
						  perfect_inverse_match);

	DBGPR_FILTER
	    ("Successfully selected L2 %s filtering and %s DA matching\n",
	     (l_l2_da_filter.perfect_hash ? "HASH" : "PERFECT"),
	     (l_l2_da_filter.perfect_inverse_match ? "INVERSE" : "PERFECT"));

	DBGPR_FILTER("<--eqos_confing_l2_da_filter\n");

	return ret;
}

/*!
 * \details This function is invoked by ioctl function when user issues
 * an ioctl command to enable/disable mac loopback mode.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] flags – flag to indicate whether mac loopback mode to be
 *                  enabled/disabled.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
int eqos_config_mac_loopback_mode(struct net_device *dev,
				  unsigned int flags)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	int ret = 0;

	pr_debug("-->eqos_config_mac_loopback_mode\n");

	if (flags && pdata->mac_loopback_mode) {
		pr_err("MAC loopback mode is already enabled\n");
		return -EINVAL;
	}
	if (!flags && !pdata->mac_loopback_mode) {
		pr_err("MAC loopback mode is already disabled\n");
		return -EINVAL;
	}
	pdata->mac_loopback_mode = !!flags;
	hw_if->config_mac_loopback_mode(flags);

	pr_err("Succesfully %s MAC loopback mode\n",
	       (flags ? "enabled" : "disabled"));

	pr_debug("<--eqos_config_mac_loopback_mode\n");

	return ret;
}

static VOID eqos_config_timer_registers(struct eqos_prv_data *pdata)
{
	struct timespec now;
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	u64 temp;

	pr_debug("-->eqos_config_timer_registers\n");

	/* program Sub Second Increment Reg */
	hw_if->config_sub_second_increment(EQOS_SYSCLOCK);

	/* formula is :
	 * addend = 2^32/freq_div_ratio;
	 *
	 * where, freq_div_ratio = EQOS_SYSCLOCK/50MHz
	 *
	 * hence, addend = ((2^32) * 50MHz)/EQOS_SYSCLOCK;
	 *
	 * NOTE: EQOS_SYSCLOCK should be >= 50MHz to
	 *       achive 20ns accuracy.
	 *
	 * 2^x * y == (y << x), hence
	 * 2^32 * 6250000 ==> (6250000 << 32)
	 * */
	temp = (u64) (62500000ULL << 32);
	pdata->default_addend = div_u64(temp, pdata->ptp_ref_clk_rate);

	hw_if->config_addend(pdata->default_addend);

	/* initialize system time */
	getnstimeofday(&now);
	hw_if->init_systime(now.tv_sec, now.tv_nsec);

	pr_debug("-->eqos_config_timer_registers\n");
}

/*!
 * \details This function is invoked by ioctl function when user issues
 * an ioctl command to configure PTP offloading feature.
 *
 * \param[in] pdata - pointer to private data structure.
 * \param[in] flags – Each bit in this variable carry some information related
 *		      double vlan processing.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
static int eqos_config_ptpoffload(struct eqos_prv_data *pdata,
				  struct eqos_config_ptpoffloading *u_conf_ptp)
{
	UINT pto_cntrl;
	UINT mac_tcr;
	struct eqos_config_ptpoffloading l_conf_ptp;
	struct hw_if_struct *hw_if = &(pdata->hw_if);

	if (copy_from_user(&l_conf_ptp, u_conf_ptp,
			   sizeof(struct eqos_config_ptpoffloading))) {
		pr_err("Failed to fetch Double vlan Struct info from user\n");
		return EQOS_CONFIG_FAIL;
	}

	pr_err("-->eqos_config_ptpoffload - %d\n", l_conf_ptp.mode);

	pto_cntrl = MAC_PTOCR_PTOEN;	/* enable ptp offloading */
	mac_tcr = MAC_TCR_TSENA | MAC_TCR_TSIPENA | MAC_TCR_TSVER2ENA
	    | MAC_TCR_TSCFUPDT | MAC_TCR_TSCTRLSSR;
	if (l_conf_ptp.mode == EQOS_PTP_ORDINARY_SLAVE) {

		mac_tcr |= MAC_TCR_TSEVENTENA;
		pdata->ptp_offloading_mode = EQOS_PTP_ORDINARY_SLAVE;

	} else if (l_conf_ptp.mode == EQOS_PTP_TRASPARENT_SLAVE) {

		pto_cntrl |= MAC_PTOCR_APDREQEN;
		mac_tcr |= MAC_TCR_TSEVENTENA;
		mac_tcr |= MAC_TCR_SNAPTYPSEL_1;
		pdata->ptp_offloading_mode = EQOS_PTP_TRASPARENT_SLAVE;

	} else if (l_conf_ptp.mode == EQOS_PTP_ORDINARY_MASTER) {

		pto_cntrl |= MAC_PTOCR_ASYNCEN;
		mac_tcr |= MAC_TCR_TSEVENTENA;
		mac_tcr |= MAC_TCR_TSMASTERENA;
		pdata->ptp_offloading_mode = EQOS_PTP_ORDINARY_MASTER;

	} else if (l_conf_ptp.mode == EQOS_PTP_TRASPARENT_MASTER) {

		pto_cntrl |= MAC_PTOCR_ASYNCEN | MAC_PTOCR_APDREQEN;
		mac_tcr |= MAC_TCR_SNAPTYPSEL_1;
		mac_tcr |= MAC_TCR_TSEVENTENA;
		mac_tcr |= MAC_TCR_TSMASTERENA;
		pdata->ptp_offloading_mode = EQOS_PTP_TRASPARENT_MASTER;

	} else if (l_conf_ptp.mode == EQOS_PTP_PEER_TO_PEER_TRANSPARENT) {

		pto_cntrl |= MAC_PTOCR_APDREQEN;
		mac_tcr |= MAC_TCR_SNAPTYPSEL_3;
		pdata->ptp_offloading_mode = EQOS_PTP_PEER_TO_PEER_TRANSPARENT;
	}

	pdata->ptp_offload = 1;
	if (l_conf_ptp.en_dis == EQOS_PTP_OFFLOADING_DISABLE) {
		pto_cntrl = 0;
		mac_tcr = 0;
		pdata->ptp_offload = 0;
	}

	pto_cntrl |= (l_conf_ptp.domain_num << 8);
	hw_if->config_hw_time_stamping(mac_tcr);
	eqos_config_timer_registers(pdata);
	hw_if->config_ptpoffload_engine(pto_cntrl, l_conf_ptp.mc_uc);

	pr_err("<--eqos_config_ptpoffload\n");

	return Y_SUCCESS;
}

/*!
 * \details This function is invoked by ioctl function when user issues
 * an ioctl command to enable/disable pfc.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] flags – flag to indicate whether pfc to be enabled/disabled.
 *
 * \return integer
 *
 * \retval zero on success and -ve number on failure.
 */
static int eqos_config_pfc(struct net_device *dev, unsigned int flags)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	int ret = 0;

	pr_debug("-->eqos_config_pfc\n");

	if (!pdata->hw_feat.dcb_en) {
		pr_err("PFC is not supported\n");
		return EQOS_NO_HW_SUPPORT;
	}

	hw_if->config_pfc(flags);

	pr_err("Succesfully %s PFC(Priority Based Flow Control)\n",
	       (flags ? "enabled" : "disabled"));

	pr_debug("<--eqos_config_pfc\n");

	return ret;
}

static int eqos_handle_prv_ts_ioctl(struct eqos_prv_data *pdata,
				    struct ifreq *ifr)
{
	struct ifr_data_timestamp_struct req;
	unsigned long flags;
	u64 ns;
	u32 reminder;
	int ret = 0;

	pr_debug("-->eqos_handle_prv_ts_ioctl\n");

	if (copy_from_user(&req, ifr->ifr_data, sizeof(req)))
		return -EFAULT;

	raw_spin_lock_irqsave(&eqos_ts_lock, flags);

	switch (req.clockid) {
	case CLOCK_REALTIME:
		ktime_get_real_ts(&req.kernel_ts);
		break;

	case CLOCK_MONOTONIC:
		ktime_get_ts(&req.kernel_ts);
		break;

	default:
		pr_err("eqos ioctl: Unsupported clockid\n");
	}

	ret = get_ptp_hwtime(&ns);

	raw_spin_unlock_irqrestore(&eqos_ts_lock, flags);

	if (ret != 0) {
		pr_err("eqos ioctl: HW PTP not running\n");
		return ret;
	}

	req.hw_ptp_ts.tv_sec = div_u64_rem(ns, 1000000000ULL, &reminder);
	req.hw_ptp_ts.tv_nsec = reminder;

	pr_debug("<--eqos_ptp_get_time: tv_sec = %ld, tv_nsec = %ld\n",
		 req.hw_ptp_ts.tv_sec, req.hw_ptp_ts.tv_nsec);

	if (copy_to_user(ifr->ifr_data, &req, sizeof(req)))
		return -EFAULT;

	pr_debug("<--eqos_handle_prv_ts_ioctl\n");

	return ret;
}

/*!
 * \brief Driver IOCTL routine
 *
 * \details This function is invoked by main ioctl function when
 * users request to configure various device features like,
 * PMT module, TX and RX PBL, TX and RX FIFO threshold level,
 * TX and RX OSF mode, SA insert/replacement, L2/L3/L4 and
 * VLAN filtering, AVB/DCB algorithm etc.
 *
 * \param[in] pdata – pointer to private data structure.
 * \param[in] req – pointer to ioctl structure.
 *
 * \return int
 *
 * \retval 0 - success
 * \retval negative - failure
 */

static int eqos_handle_prv_ioctl(struct eqos_prv_data *pdata,
				 struct ifreq *ifr)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct net_device *dev = pdata->dev;
	struct ifr_data_struct req;
	struct tx_ring *ptx_ring;
	struct rx_ring *prx_ring;
	unsigned int qinx;
	int ret = 0;

	pr_debug("-->eqos_handle_prv_ioctl\n");

	if (copy_from_user(&req, ifr->ifr_data, sizeof(req)))
		return -EFAULT;

	qinx = req.qinx;
	if (qinx >= EQOS_QUEUE_CNT) {
		pr_err("Queue number %d is invalid\n"
		       "Hardware has only %d Tx/Rx Queues\n",
		       qinx, EQOS_QUEUE_CNT);
		ret = EQOS_NO_HW_SUPPORT;
		return ret;
	}

	qinx = array_index_nospec(qinx, EQOS_QUEUE_CNT);
	ptx_ring = GET_TX_WRAPPER_DESC(qinx);
	prx_ring = GET_RX_WRAPPER_DESC(qinx);

	switch (req.cmd) {
	case EQOS_RX_THRESHOLD_CMD:
		prx_ring->rx_threshold_val = req.flags;
		hw_if->config_rx_threshold(qinx,
					   prx_ring->rx_threshold_val);
		pr_err("Configured Rx threshold with %d\n",
		       prx_ring->rx_threshold_val);
		break;

	case EQOS_TX_THRESHOLD_CMD:
		ptx_ring->tx_threshold_val = req.flags;
		hw_if->config_tx_threshold(qinx,
					   ptx_ring->tx_threshold_val);
		pr_err("Configured Tx threshold with %d\n",
		       ptx_ring->tx_threshold_val);
		break;

	case EQOS_RSF_CMD:
		prx_ring->rsf_on = req.flags;
		hw_if->config_rsf_mode(qinx, prx_ring->rsf_on);
		pr_err("Receive store and forward mode %s\n",
		       (prx_ring->rsf_on) ? "enabled" : "disabled");
		break;

	case EQOS_TSF_CMD:
		ptx_ring->tsf_on = req.flags;
		hw_if->config_tsf_mode(qinx, ptx_ring->tsf_on);
		pr_err("Transmit store and forward mode %s\n",
		       (ptx_ring->tsf_on) ? "enabled" : "disabled");
		break;

	case EQOS_OSF_CMD:
		ptx_ring->osf_on = req.flags;
		hw_if->config_osf_mode(qinx, ptx_ring->osf_on);
		pr_err("Transmit DMA OSF mode is %s\n",
		       (ptx_ring->osf_on) ? "enabled" : "disabled");
		break;

	case EQOS_INCR_INCRX_CMD:
		pdata->incr_incrx = req.flags;
		hw_if->config_incr_incrx_mode(pdata->incr_incrx);
		pr_err("%s mode is enabled\n",
		       (pdata->incr_incrx) ? "INCRX" : "INCR");
		break;

	case EQOS_RX_PBL_CMD:
		prx_ring->rx_pbl = req.flags;
		eqos_config_rx_pbl(pdata, prx_ring->rx_pbl, qinx);
		break;

	case EQOS_TX_PBL_CMD:
		ptx_ring->tx_pbl = req.flags;
		eqos_config_tx_pbl(pdata, ptx_ring->tx_pbl, qinx);
		break;

	case EQOS_PTPOFFLOADING_CMD:
		if (pdata->hw_feat.tsstssel) {
			ret = eqos_config_ptpoffload(pdata, req.ptr);
		} else {
			pr_err("No HW support for PTP\n");
			ret = EQOS_NO_HW_SUPPORT;
		}
		break;

	case EQOS_SA0_DESC_CMD:
		if (pdata->hw_feat.sa_vlan_ins) {
			pdata->tx_sa_ctrl_via_desc = req.flags;
			pdata->tx_sa_ctrl_via_reg = EQOS_SA0_NONE;
			if (req.flags == EQOS_SA0_NONE) {
				memcpy(pdata->mac_addr, pdata->dev->dev_addr,
				       EQOS_MAC_ADDR_LEN);
			} else {
				memcpy(pdata->mac_addr, mac_addr0,
				       EQOS_MAC_ADDR_LEN);
			}
			hw_if->configure_mac_addr0_reg(pdata->mac_addr);
			hw_if->configure_sa_via_reg(pdata->tx_sa_ctrl_via_reg);
			pr_err
			    ("SA will use MAC0 with descriptor for configuration %d\n",
			     pdata->tx_sa_ctrl_via_desc);
		} else {
			pr_err
			    ("Device doesn't supports SA Insertion/Replacement\n");
			ret = EQOS_NO_HW_SUPPORT;
		}
		break;

	case EQOS_SA1_DESC_CMD:
		if (pdata->hw_feat.sa_vlan_ins) {
			pdata->tx_sa_ctrl_via_desc = req.flags;
			pdata->tx_sa_ctrl_via_reg = EQOS_SA1_NONE;
			if (req.flags == EQOS_SA1_NONE) {
				memcpy(pdata->mac_addr, pdata->dev->dev_addr,
				       EQOS_MAC_ADDR_LEN);
			} else {
				memcpy(pdata->mac_addr, mac_addr1,
				       EQOS_MAC_ADDR_LEN);
			}
			hw_if->configure_mac_addr1_reg(pdata->mac_addr);
			hw_if->configure_sa_via_reg(pdata->tx_sa_ctrl_via_reg);
			pr_err
			    ("SA will use MAC1 with descriptor for configuration %d\n",
			     pdata->tx_sa_ctrl_via_desc);
		} else {
			pr_err
			    ("Device doesn't supports SA Insertion/Replacement\n");
			ret = EQOS_NO_HW_SUPPORT;
		}
		break;

	case EQOS_SA0_REG_CMD:
		if (pdata->hw_feat.sa_vlan_ins) {
			pdata->tx_sa_ctrl_via_reg = req.flags;
			pdata->tx_sa_ctrl_via_desc = EQOS_SA0_NONE;
			if (req.flags == EQOS_SA0_NONE) {
				memcpy(pdata->mac_addr, pdata->dev->dev_addr,
				       EQOS_MAC_ADDR_LEN);
			} else {
				memcpy(pdata->mac_addr, mac_addr0,
				       EQOS_MAC_ADDR_LEN);
			}
			hw_if->configure_mac_addr0_reg(pdata->mac_addr);
			hw_if->configure_sa_via_reg(pdata->tx_sa_ctrl_via_reg);
			pr_err
			    ("SA will use MAC0 with register for configuration %d\n",
			     pdata->tx_sa_ctrl_via_desc);
		} else {
			pr_err
			    ("Device doesn't supports SA Insertion/Replacement\n");
			ret = EQOS_NO_HW_SUPPORT;
		}
		break;

	case EQOS_SA1_REG_CMD:
		if (pdata->hw_feat.sa_vlan_ins) {
			pdata->tx_sa_ctrl_via_reg = req.flags;
			pdata->tx_sa_ctrl_via_desc = EQOS_SA1_NONE;
			if (req.flags == EQOS_SA1_NONE) {
				memcpy(pdata->mac_addr, pdata->dev->dev_addr,
				       EQOS_MAC_ADDR_LEN);
			} else {
				memcpy(pdata->mac_addr, mac_addr1,
				       EQOS_MAC_ADDR_LEN);
			}
			hw_if->configure_mac_addr1_reg(pdata->mac_addr);
			hw_if->configure_sa_via_reg(pdata->tx_sa_ctrl_via_reg);
			pr_err
			    ("SA will use MAC1 with register for configuration %d\n",
			     pdata->tx_sa_ctrl_via_desc);
		} else {
			pr_err
			    ("Device doesn't supports SA Insertion/Replacement\n");
			ret = EQOS_NO_HW_SUPPORT;
		}
		break;

	case EQOS_SETUP_CONTEXT_DESCRIPTOR:
		if (pdata->hw_feat.sa_vlan_ins) {
			ptx_ring->context_setup = req.context_setup;
			if (ptx_ring->context_setup == 1) {
				pr_err("Context descriptor will be transmitted"
				       " with every normal descriptor on %d DMA Channel\n",
				       qinx);
			} else {
				pr_err("Context descriptor will be setup"
				       " only if VLAN id changes %d\n", qinx);
			}
		} else {
			pr_err("Device doesn't support VLAN operations\n");
			ret = EQOS_NO_HW_SUPPORT;
		}
		break;

	case EQOS_GET_RX_QCNT:
		req.qinx = EQOS_RX_QUEUE_CNT;
		break;

	case EQOS_GET_TX_QCNT:
		req.qinx = EQOS_TX_QUEUE_CNT;
		break;

	case EQOS_GET_CONNECTED_SPEED:
		req.connected_speed = pdata->speed;
		break;

	case EQOS_DCB_ALGORITHM:
		eqos_program_dcb_algorithm(pdata, &req);
		break;

	case EQOS_AVB_ALGORITHM:
		eqos_program_avb_algorithm(pdata, &req);
		break;

	case EQOS_L3_L4_FILTER_CMD:
		if (pdata->hw_feat.l3l4_filter_num > 0) {
			ret = eqos_config_l3_l4_filtering(dev, req.flags);
			if (ret == 0)
				ret = EQOS_CONFIG_SUCCESS;
			else
				ret = EQOS_CONFIG_FAIL;
		} else {
			ret = EQOS_NO_HW_SUPPORT;
		}
		break;
	case EQOS_IPV4_FILTERING_CMD:
		ret = eqos_config_ip4_filters(dev, &req);
		break;
	case EQOS_IPV6_FILTERING_CMD:
		ret = eqos_config_ip6_filters(dev, &req);
		break;
	case EQOS_UDP_FILTERING_CMD:
		ret = eqos_config_tcp_udp_filters(dev, &req, 1);
		break;
	case EQOS_TCP_FILTERING_CMD:
		ret = eqos_config_tcp_udp_filters(dev, &req, 0);
		break;
	case EQOS_VLAN_FILTERING_CMD:
		ret = eqos_config_vlan_filter(dev, &req);
		break;
	case EQOS_L2_DA_FILTERING_CMD:
		ret = eqos_confing_l2_da_filter(dev, &req);
		break;
	case EQOS_ARP_OFFLOAD_CMD:
		ret = eqos_config_arp_offload(dev, &req);
		break;
	case EQOS_AXI_PBL_CMD:
		pdata->axi_pbl = req.flags;
		hw_if->config_axi_pbl_val(pdata->axi_pbl);
		pr_err("AXI PBL value: %d\n", pdata->axi_pbl);
		break;
	case EQOS_AXI_WORL_CMD:
		pdata->axi_worl = req.flags;
		hw_if->config_axi_worl_val(pdata->axi_worl);
		pr_err("AXI WORL value: %d\n", pdata->axi_worl);
		break;
	case EQOS_AXI_RORL_CMD:
		pdata->axi_rorl = req.flags;
		hw_if->config_axi_rorl_val(pdata->axi_rorl);
		pr_err("AXI RORL value: %d\n", pdata->axi_rorl);
		break;
	case EQOS_MAC_LOOPBACK_MODE_CMD:
		ret = eqos_config_mac_loopback_mode(dev, req.flags);
		if (ret == 0)
			ret = EQOS_CONFIG_SUCCESS;
		else
			ret = EQOS_CONFIG_FAIL;
		break;
	case EQOS_PFC_CMD:
		ret = eqos_config_pfc(dev, req.flags);
		break;
	case EQOS_PHY_LOOPBACK:
		ret = eqos_handle_phy_loopback(pdata, (void *)&req);
		break;
	case EQOS_MEM_ISO_TEST:
		ret = eqos_handle_mem_iso_ioctl(pdata, (void *)&req);
		break;
	case EQOS_CSR_ISO_TEST:
		ret = eqos_handle_csr_iso_ioctl(pdata, (void *)&req);
		break;
	default:
		ret = -EOPNOTSUPP;
		pr_err("Unsupported command call\n");
	}

	req.command_error = ret;
	if (copy_to_user(ifr->ifr_data, &req, sizeof(req)))
		return -EFAULT;

	pr_debug("<--eqos_handle_prv_ioctl\n");

	return ret;
}

/*!
 * \brief control hw timestamping.
 *
 * \details This function is used to configure the MAC to enable/disable both
 * outgoing(Tx) and incoming(Rx) packets time stamping based on user input.
 *
 * \param[in] pdata – pointer to private data structure.
 * \param[in] ifr – pointer to IOCTL specific structure.
 *
 * \return int
 *
 * \retval 0 - success
 * \retval negative - failure
 */

static int eqos_handle_hwtstamp_ioctl(struct eqos_prv_data *pdata,
				      struct ifreq *ifr)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	struct hwtstamp_config config;
	u32 ptp_v2 = 0;
	u32 tstamp_all = 0;
	u32 ptp_over_ipv4_udp = 0;
	u32 ptp_over_ipv6_udp = 0;
	u32 ptp_over_ethernet = 0;
	u32 snap_type_sel = 0;
	u32 ts_master_en = 0;
	u32 ts_event_en = 0;
	u32 av_8021asm_en = 0;
	u32 mac_tcr = 0;
	u64 temp = 0;
	struct timespec now;

	DBGPR_PTP("-->eqos_handle_hwtstamp_ioctl\n");

	if (!pdata->hw_feat.tsstssel) {
		pr_err("No hw timestamping is available in this core\n");
		return -EOPNOTSUPP;
	}

	if (copy_from_user(&config, ifr->ifr_data,
			   sizeof(struct hwtstamp_config)))
		return -EFAULT;

	DBGPR_PTP("config.flags = %#x, tx_type = %#x, rx_filter = %#x\n",
		  config.flags, config.tx_type, config.rx_filter);

	/* reserved for future extensions */
	if (config.flags)
		return -EINVAL;

	switch (config.tx_type) {
	case HWTSTAMP_TX_OFF:
		pdata->hwts_tx_en = 0;
		break;
	case HWTSTAMP_TX_ON:
		pdata->hwts_tx_en = 1;
		break;
	default:
		return -ERANGE;
	}

	switch (config.rx_filter) {
		/* time stamp no incoming packet at all */
	case HWTSTAMP_FILTER_NONE:
		config.rx_filter = HWTSTAMP_FILTER_NONE;
		break;

		/* PTP v1, UDP, any kind of event packet */
	case HWTSTAMP_FILTER_PTP_V1_L4_EVENT:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_EVENT;
		/* take time stamp for all event messages */
		snap_type_sel = MAC_TCR_SNAPTYPSEL_1;

		ptp_over_ipv4_udp = MAC_TCR_TSIPV4ENA;
		ptp_over_ipv6_udp = MAC_TCR_TSIPV6ENA;
		break;

		/* PTP v1, UDP, Sync packet */
	case HWTSTAMP_FILTER_PTP_V1_L4_SYNC:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_SYNC;
		/* take time stamp for SYNC messages only */
		ts_event_en = MAC_TCR_TSEVENTENA;

		ptp_over_ipv4_udp = MAC_TCR_TSIPV4ENA;
		ptp_over_ipv6_udp = MAC_TCR_TSIPV6ENA;
		break;

		/* PTP v1, UDP, Delay_req packet */
	case HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V1_L4_DELAY_REQ;
		/* take time stamp for Delay_Req messages only */
		ts_master_en = MAC_TCR_TSMASTERENA;
		ts_event_en = MAC_TCR_TSEVENTENA;

		ptp_over_ipv4_udp = MAC_TCR_TSIPV4ENA;
		ptp_over_ipv6_udp = MAC_TCR_TSIPV6ENA;
		break;

		/* PTP v2, UDP, any kind of event packet */
	case HWTSTAMP_FILTER_PTP_V2_L4_EVENT:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_EVENT;
		ptp_v2 = MAC_TCR_TSVER2ENA;
		/* take time stamp for all event messages */
		snap_type_sel = MAC_TCR_SNAPTYPSEL_1;

		ptp_over_ipv4_udp = MAC_TCR_TSIPV4ENA;
		ptp_over_ipv6_udp = MAC_TCR_TSIPV6ENA;
		break;

		/* PTP v2, UDP, Sync packet */
	case HWTSTAMP_FILTER_PTP_V2_L4_SYNC:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_SYNC;
		ptp_v2 = MAC_TCR_TSVER2ENA;
		/* take time stamp for SYNC messages only */
		ts_event_en = MAC_TCR_TSEVENTENA;

		ptp_over_ipv4_udp = MAC_TCR_TSIPV4ENA;
		ptp_over_ipv6_udp = MAC_TCR_TSIPV6ENA;
		break;

		/* PTP v2, UDP, Delay_req packet */
	case HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_L4_DELAY_REQ;
		ptp_v2 = MAC_TCR_TSVER2ENA;
		/* take time stamp for Delay_Req messages only */
		ts_master_en = MAC_TCR_TSMASTERENA;
		ts_event_en = MAC_TCR_TSEVENTENA;

		ptp_over_ipv4_udp = MAC_TCR_TSIPV4ENA;
		ptp_over_ipv6_udp = MAC_TCR_TSIPV6ENA;
		break;

		/* PTP v2/802.AS1, any layer, any kind of event packet */
	case HWTSTAMP_FILTER_PTP_V2_EVENT:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
		ptp_v2 = MAC_TCR_TSVER2ENA;
		/* take time stamp for all event messages */
		snap_type_sel = MAC_TCR_SNAPTYPSEL_1;

		ptp_over_ipv4_udp = MAC_TCR_TSIPV4ENA;
		ptp_over_ipv6_udp = MAC_TCR_TSIPV6ENA;
		ptp_over_ethernet = MAC_TCR_TSIPENA;
		/* for VLAN tagged PTP, AV8021ASMEN bit should not be set */
#ifdef DWC_1588_VLAN_UNTAGGED
		av_8021asm_en = MAC_TCR_AV8021ASMEN;
#endif
		break;

		/* PTP v2/802.AS1, any layer, Sync packet */
	case HWTSTAMP_FILTER_PTP_V2_SYNC:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_SYNC;
		ptp_v2 = MAC_TCR_TSVER2ENA;
		/* take time stamp for SYNC messages only */
		ts_event_en = MAC_TCR_TSEVENTENA;

		ptp_over_ipv4_udp = MAC_TCR_TSIPV4ENA;
		ptp_over_ipv6_udp = MAC_TCR_TSIPV6ENA;
		ptp_over_ethernet = MAC_TCR_TSIPENA;
		av_8021asm_en = MAC_TCR_AV8021ASMEN;
		break;

		/* PTP v2/802.AS1, any layer, Delay_req packet */
	case HWTSTAMP_FILTER_PTP_V2_DELAY_REQ:
		config.rx_filter = HWTSTAMP_FILTER_PTP_V2_DELAY_REQ;
		ptp_v2 = MAC_TCR_TSVER2ENA;
		/* take time stamp for Delay_Req messages only */
		ts_master_en = MAC_TCR_TSMASTERENA;
		ts_event_en = MAC_TCR_TSEVENTENA;

		ptp_over_ipv4_udp = MAC_TCR_TSIPV4ENA;
		ptp_over_ipv6_udp = MAC_TCR_TSIPV6ENA;
		ptp_over_ethernet = MAC_TCR_TSIPENA;
		av_8021asm_en = MAC_TCR_AV8021ASMEN;
		break;

		/* time stamp any incoming packet */
	case HWTSTAMP_FILTER_ALL:
		config.rx_filter = HWTSTAMP_FILTER_ALL;
		tstamp_all = MAC_TCR_TSENALL;
		break;

	default:
		return -ERANGE;
	}
	pdata->hwts_rx_en =
	    ((config.rx_filter == HWTSTAMP_FILTER_NONE) ? 0 : 1);

	if (!pdata->hwts_tx_en && !pdata->hwts_rx_en) {
		/* disable hw time stamping */
		hw_if->config_hw_time_stamping(mac_tcr);
	} else {
		mac_tcr =
		    (MAC_TCR_TSENA | MAC_TCR_TSCFUPDT | MAC_TCR_TSCTRLSSR |
		     tstamp_all | ptp_v2 | ptp_over_ethernet | ptp_over_ipv6_udp
		     | ptp_over_ipv4_udp | ts_event_en | ts_master_en |
		     snap_type_sel | av_8021asm_en);

		if (!pdata->one_nsec_accuracy)
			mac_tcr &= ~MAC_TCR_TSCTRLSSR;

		hw_if->config_hw_time_stamping(mac_tcr);

		/* program Sub Second Increment Reg */
		hw_if->config_sub_second_increment(EQOS_SYSCLOCK);

		/* formula is :
		 * addend = 2^32/freq_div_ratio;
		 *
		 * where, freq_div_ratio = EQOS_SYSCLOCK/50MHz
		 *
		 * hence, addend = ((2^32) * 50MHz)/EQOS_SYSCLOCK;
		 *
		 * NOTE: EQOS_SYSCLOCK should be >= 50MHz to
		 *       achive 20ns accuracy.
		 *
		 * 2^x * y == (y << x), hence
		 * 2^32 * 6250000 ==> (6250000 << 32)
		 * */
		temp = (u64) (62500000ULL << 32);
		pdata->default_addend = div_u64(temp, pdata->ptp_ref_clk_rate);

		hw_if->config_addend(pdata->default_addend);

		/* initialize system time */
		getnstimeofday(&now);
		hw_if->init_systime(now.tv_sec, now.tv_nsec);

		DBGPR_PTP("-->eqos registering get_ptp function\n");
		/* Register broadcasting MAC timestamp to clients */
		tegra_register_hwtime_source(eqos_get_ptptime, pdata);

		/* Enable slot function control */
		eqos_enable_slot_function_ctrl(pdata);
	}

	DBGPR_PTP("config.flags = %#x, tx_type = %#x, rx_filter = %#x\n",
		  config.flags, config.tx_type, config.rx_filter);

	DBGPR_PTP("<--eqos_handle_hwtstamp_ioctl\n");

	return (copy_to_user(ifr->ifr_data, &config,
			     sizeof(struct hwtstamp_config))) ? -EFAULT : 0;
}

/*!
 * \brief Driver IOCTL routine
 *
 * \details This function is invoked by kernel when a user request an ioctl
 * which can't be handled by the generic interface code. Following operations
 * are performed in this functions.
 * - Configuring the PMT module.
 * - Configuring TX and RX PBL.
 * - Configuring the TX and RX FIFO threshold level.
 * - Configuring the TX and RX OSF mode.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] ifr – pointer to IOCTL specific structure.
 * \param[in] cmd – IOCTL command.
 *
 * \return int
 *
 * \retval 0 - success
 * \retval negative - failure
 */

static int eqos_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);
	int ret = 0;

	pr_debug("-->eqos_ioctl\n");

	if ((!netif_running(dev)) || (!pdata->phydev)) {
		pr_debug("<--eqos_ioctl - error\n");
		return -EINVAL;
	}

	switch (cmd) {
	case SIOCGMIIPHY:
	case SIOCGMIIREG:
	case SIOCSMIIREG:
		ret = phy_mii_ioctl(pdata->phydev, ifr, cmd);
		break;

	case EQOS_PRV_IOCTL:
		spin_lock_bh(&pdata->lock);
		ret = eqos_handle_prv_ioctl(pdata, ifr);
		spin_unlock_bh(&pdata->lock);
		break;

	case EQOS_PRV_TS_IOCTL:
		spin_lock_bh(&pdata->lock);
		ret = eqos_handle_prv_ts_ioctl(pdata, ifr);
		spin_unlock_bh(&pdata->lock);
		break;

	case SIOCSHWTSTAMP:
		spin_lock_bh(&pdata->lock);
		ret = eqos_handle_hwtstamp_ioctl(pdata, ifr);
		spin_unlock_bh(&pdata->lock);
		break;

	default:
		ret = -EOPNOTSUPP;
		pr_debug("Unsupported IOCTL %d is called\n", cmd);
	}

	pr_debug("<--eqos_ioctl\n");

	return ret;
}

/*!
* \brief API to change MTU.
*
* \details This function is invoked by upper layer when user changes
* MTU (Maximum Transfer Unit). The MTU is used by the Network layer
* to driver packet transmission. Ethernet has a default MTU of
* 1500Bytes. This value can be changed with ifconfig -
* ifconfig <interface_name> mtu <new_mtu_value>
*
* \param[in] dev - pointer to net_device structure
* \param[in] new_mtu - the new MTU for the device.
*
* \return integer
*
* \retval 0 - on success and -ve on failure.
*/

static INT eqos_change_mtu(struct net_device *dev, INT new_mtu)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);
	struct platform_device *pdev = pdata->pdev;
	int max_frame = (new_mtu + ETH_HLEN + ETH_FCS_LEN + VLAN_HLEN);

	if (!netif_running(dev)) {
		dev_info(&pdev->dev, "network interface is not running\n");
		return 0;
	}

#ifdef EQOS_CONFIG_PGTEST
	dev_err(&pdev->dev, "jumbo frames not supported with PG test\n");
	return -EOPNOTSUPP;
#endif
	if (pdata->dt_cfg.use_multi_q && (pdata->mac_ver < EQOS_MAC_CORE_5_00)) {
		dev_err(&pdev->dev,
			"mtu cannot be modified in multi queue mode\n");
		return -EOPNOTSUPP;
	}

	if (new_mtu > 9000) {
		dev_err(&pdev->dev, "Got unsupported MTU size %d, \
			MAX supported MTU size is 9000 bytes\n", new_mtu);
		return -EINVAL;
	}

	if (dev->mtu == new_mtu) {
		dev_err(&pdev->dev, "already configured to mtu %d\n", new_mtu);
		return 0;
	}

	dev_info(&pdev->dev, "changing MTU from %d to %d\n", dev->mtu, new_mtu);

	eqos_close(dev);

	if (max_frame <= 2048) {
		pdata->rx_buffer_len = 2048;
	} else {
		pdata->rx_buffer_len = ALIGN_SIZE(max_frame);
	}
	pdata->rx_max_frame_size = max_frame;

	dev->mtu = new_mtu;

	return eqos_open(dev);
}

#ifdef EQOS_QUEUE_SELECT_ALGO
u16 eqos_select_queue(struct net_device *dev,
		      struct sk_buff *skb, void *accel_priv,
		      select_queue_fallback_t fallback)
{
	int  txqueue_select = -1;
	struct eqos_prv_data *pdata = netdev_priv(dev);
	struct eqos_cfg *pdt_cfg = (struct eqos_cfg *)&pdata->dt_cfg;
	UINT i;

	pr_debug("-->eqos_select_queue\n");

	for (i = 0; i < EQOS_TX_QUEUE_CNT; i++) {
		if (pdt_cfg->q_prio[i] == skb->priority) {
			txqueue_select = i;
			break;
		}
	}

	if (txqueue_select < 0)
		txqueue_select = 0;

	pr_debug("<--eqos_select_queue txqueue-select:%d\n", txqueue_select);

	return txqueue_select;
}
#endif

unsigned int crc32_snps_le(unsigned int initval, unsigned char *data,
			   unsigned int size)
{
	unsigned int crc = initval;
	unsigned int poly = 0x04c11db7;
	unsigned int temp = 0;
	unsigned char my_data = 0;
	int bit_count;
	for (bit_count = 0; bit_count < size; bit_count++) {
		if ((bit_count % 8) == 0)
			my_data = data[bit_count / 8];
		DBGPR_FILTER("%s my_data = %x crc=%x\n", __func__, my_data,
			     crc);
		temp = ((crc >> 31) ^ my_data) & 0x1;
		crc <<= 1;
		if (temp != 0)
			crc ^= poly;
		my_data >>= 1;
	}
	DBGPR_FILTER("%s my_data = %x crc=%x\n", __func__, my_data, crc);
	return ~crc;
}

/*!
* \brief API to delete vid to HW filter.
*
* \details This function is invoked by upper layer when a VLAN id is removed.
* This function deletes the VLAN id from the HW filter.
* vlan id can be removed with vconfig -
* vconfig rem <interface_name > <vlan_id>
*
* \param[in] dev - pointer to net_device structure
* \param[in] vid - vlan id to be removed.
*
* \return void
*/
static int eqos_vlan_rx_kill_vid(struct net_device *dev, __be16 proto, u16 vid)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	unsigned short new_index, old_index;
	int crc32_val = 0;
	unsigned int enb_12bit_vhash;

	pr_err("-->eqos_vlan_rx_kill_vid: vid = %d\n", vid);

	if (pdata->vlan_hash_filtering) {
		crc32_val =
		    (bitrev32(~crc32_le(~0, (unsigned char *)&vid, 2)) >> 28);

		enb_12bit_vhash = hw_if->get_vlan_tag_comparison();
		if (enb_12bit_vhash) {
			/* neget 4-bit crc value for 12-bit VLAN hash comparison */
			new_index = (1 << (~crc32_val & 0xF));
		} else {
			new_index = (1 << (crc32_val & 0xF));
		}

		old_index = hw_if->get_vlan_hash_table_reg();
		old_index &= ~new_index;
		hw_if->update_vlan_hash_table_reg(old_index);
		pdata->vlan_ht_or_id = old_index;
	} else {
		/* By default, receive only VLAN pkt with VID = 1
		 * becasue writting 0 will pass all VLAN pkt */
		hw_if->update_vlan_id(1);
		pdata->vlan_ht_or_id = 1;
	}

	pr_err("<--eqos_vlan_rx_kill_vid\n");

	/* FIXME: Check if any errors need to be returned in case of failure */
	return 0;
}

/*!
* \brief API to add vid to HW filter.
*
* \details This function is invoked by upper layer when a new VALN id is
* registered. This function updates the HW filter with new VLAN id.
* New vlan id can be added with vconfig -
* vconfig add <interface_name > <vlan_id>
*
* \param[in] dev - pointer to net_device structure
* \param[in] vid - new vlan id.
*
* \return void
*/
static int eqos_vlan_rx_add_vid(struct net_device *dev, __be16 proto, u16 vid)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	unsigned short new_index, old_index;
	int crc32_val = 0;
	unsigned int enb_12bit_vhash;

	pr_err("-->eqos_vlan_rx_add_vid: vid = %d\n", vid);

	if (pdata->vlan_hash_filtering) {
		/* The upper 4 bits of the calculated CRC are used to
		 * index the content of the VLAN Hash Table Reg.
		 * */
		crc32_val =
		    (bitrev32(~crc32_le(~0, (unsigned char *)&vid, 2)) >> 28);

		/* These 4(0xF) bits determines the bit within the
		 * VLAN Hash Table Reg 0
		 * */
		enb_12bit_vhash = hw_if->get_vlan_tag_comparison();
		if (enb_12bit_vhash) {
			/* neget 4-bit crc value for 12-bit VLAN hash comparison */
			new_index = (1 << (~crc32_val & 0xF));
		} else {
			new_index = (1 << (crc32_val & 0xF));
		}

		old_index = hw_if->get_vlan_hash_table_reg();
		old_index |= new_index;
		hw_if->update_vlan_hash_table_reg(old_index);
		pdata->vlan_ht_or_id = old_index;
	} else {
		hw_if->update_vlan_id(vid);
		pdata->vlan_ht_or_id = vid;
	}

	pr_err("<--eqos_vlan_rx_add_vid\n");

	/* FIXME: Check if any errors need to be returned in case of failure */
	return 0;
}


/*!
 * \details This function is invoked by ioctl function when the user issues an
 * ioctl command to change the RX DMA PBL value. This function will program
 * the device to configure the user specified RX PBL value.
 *
 * \param[in] pdata – pointer to private data structure.
 * \param[in] rx_pbl – RX DMA pbl value to be programmed.
 *
 * \return void
 *
 * \retval none
 */

static void eqos_config_rx_pbl(struct eqos_prv_data *pdata,
			       UINT rx_pbl, UINT qinx)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	UINT pblx8_val = 0;

	pr_debug("-->eqos_config_rx_pbl: %d\n", rx_pbl);

	switch (rx_pbl) {
	case EQOS_PBL_1:
	case EQOS_PBL_2:
	case EQOS_PBL_4:
	case EQOS_PBL_8:
	case EQOS_PBL_16:
	case EQOS_PBL_32:
		hw_if->config_rx_pbl_val(qinx, rx_pbl);
		hw_if->config_pblx8(qinx, 0);
		break;
	case EQOS_PBL_64:
	case EQOS_PBL_128:
	case EQOS_PBL_256:
		hw_if->config_rx_pbl_val(qinx, rx_pbl / 8);
		hw_if->config_pblx8(qinx, 1);
		pblx8_val = 1;
		break;
	}

	switch (pblx8_val) {
	case 0:
		pr_err("Tx PBL[%d] value: %d\n",
		       qinx, hw_if->get_tx_pbl_val(qinx));
		pr_err("Rx PBL[%d] value: %d\n",
		       qinx, hw_if->get_rx_pbl_val(qinx));
		break;
	case 1:
		pr_err("Tx PBL[%d] value: %d\n",
		       qinx, (hw_if->get_tx_pbl_val(qinx) * 8));
		pr_err("Rx PBL[%d] value: %d\n",
		       qinx, (hw_if->get_rx_pbl_val(qinx) * 8));
		break;
	}

	pr_debug("<--eqos_config_rx_pbl\n");
}

/*!
 * \details This function is invoked by ioctl function when the user issues an
 * ioctl command to change the TX DMA PBL value. This function will program
 * the device to configure the user specified TX PBL value.
 *
 * \param[in] pdata – pointer to private data structure.
 * \param[in] tx_pbl – TX DMA pbl value to be programmed.
 *
 * \return void
 *
 * \retval none
 */

static void eqos_config_tx_pbl(struct eqos_prv_data *pdata,
			       UINT tx_pbl, UINT qinx)
{
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	UINT pblx8_val = 0;

	pr_debug("-->eqos_config_tx_pbl: %d\n", tx_pbl);

	switch (tx_pbl) {
	case EQOS_PBL_1:
	case EQOS_PBL_2:
	case EQOS_PBL_4:
	case EQOS_PBL_8:
	case EQOS_PBL_16:
	case EQOS_PBL_32:
		hw_if->config_tx_pbl_val(qinx, tx_pbl);
		hw_if->config_pblx8(qinx, 0);
		break;
	case EQOS_PBL_64:
	case EQOS_PBL_128:
	case EQOS_PBL_256:
		hw_if->config_tx_pbl_val(qinx, tx_pbl / 8);
		hw_if->config_pblx8(qinx, 1);
		pblx8_val = 1;
		break;
	}

	switch (pblx8_val) {
	case 0:
		pr_err("Tx PBL[%d] value: %d\n",
		       qinx, hw_if->get_tx_pbl_val(qinx));
		pr_err("Rx PBL[%d] value: %d\n",
		       qinx, hw_if->get_rx_pbl_val(qinx));
		break;
	case 1:
		pr_err("Tx PBL[%d] value: %d\n",
		       qinx, (hw_if->get_tx_pbl_val(qinx) * 8));
		pr_err("Rx PBL[%d] value: %d\n",
		       qinx, (hw_if->get_rx_pbl_val(qinx) * 8));
		break;
	}

	pr_debug("<--eqos_config_tx_pbl\n");
}

/*!
 * \details This function is invoked by ioctl function when the user issues an
 * ioctl command to select the DCB algorithm.
 *
 * \param[in] pdata – pointer to private data structure.
 * \param[in] req – pointer to ioctl data structure.
 *
 * \return void
 *
 * \retval none
 */

static void eqos_program_dcb_algorithm(struct eqos_prv_data *pdata,
				       struct ifr_data_struct *req)
{
	struct eqos_dcb_algorithm l_dcb_struct, *u_dcb_struct =
	    (struct eqos_dcb_algorithm *)req->ptr;
	struct hw_if_struct *hw_if = &pdata->hw_if;

	pr_debug("-->eqos_program_dcb_algorithm\n");

	if (copy_from_user(&l_dcb_struct, u_dcb_struct,
			   sizeof(struct eqos_dcb_algorithm)))
		pr_err("Failed to fetch DCB Struct info from user\n");

	hw_if->set_tx_queue_operating_mode(l_dcb_struct.qinx,
					   (UINT) l_dcb_struct.op_mode);
	hw_if->set_dcb_algorithm(l_dcb_struct.algorithm);
	hw_if->set_dcb_queue_weight(l_dcb_struct.qinx, l_dcb_struct.weight);

	pr_debug("<--eqos_program_dcb_algorithm\n");

	return;
}

/*!
 * \details This function is invoked by ioctl function when the user issues an
 * ioctl command to select the AVB algorithm. This function also configures other
 * parameters like send and idle slope, high and low credit.
 *
 * \param[in] pdata – pointer to private data structure.
 * \param[in] req – pointer to ioctl data structure.
 *
 * \return void
 *
 * \retval none
 */

static void eqos_program_avb_algorithm(struct eqos_prv_data *pdata,
				       struct ifr_data_struct *req)
{
	struct eqos_avb_algorithm l_avb_struct, *u_avb_struct =
	    (struct eqos_avb_algorithm *)req->ptr;
	struct hw_if_struct *hw_if = &pdata->hw_if;

	pr_debug("-->eqos_program_avb_algorithm\n");

	if (copy_from_user(&l_avb_struct, u_avb_struct,
			   sizeof(struct eqos_avb_algorithm)))
		pr_err("Failed to fetch AVB Struct info from user\n");

	hw_if->set_tx_queue_operating_mode(l_avb_struct.qinx,
					   (UINT) l_avb_struct.op_mode);
	hw_if->set_avb_algorithm(l_avb_struct.qinx, l_avb_struct.algorithm);
	hw_if->config_credit_control(l_avb_struct.qinx, l_avb_struct.cc);
	hw_if->config_send_slope(l_avb_struct.qinx, l_avb_struct.send_slope);
	hw_if->config_idle_slope(l_avb_struct.qinx, l_avb_struct.idle_slope);
	hw_if->config_high_credit(l_avb_struct.qinx, l_avb_struct.hi_credit);
	hw_if->config_low_credit(l_avb_struct.qinx, l_avb_struct.low_credit);

	pr_debug("<--eqos_program_avb_algorithm\n");

	return;
}

/*!
* \brief API to read the registers & prints the value.
* \details This function will read all the device register except
* data register & prints the values.
*
* \return none
*/
#if 0
void dbgpr_regs(void)
{
	UINT val0;
	UINT val1;
	UINT val2;
	UINT val3;
	UINT val4;
	UINT val5;

	MAC_PMTCSR_RD(val0);
	MMC_RXICMP_ERR_OCTETS_RD(val1);
	MMC_RXICMP_GD_OCTETS_RD(val2);
	MMC_RXTCP_ERR_OCTETS_RD(val3);
	MMC_RXTCP_GD_OCTETS_RD(val4);
	MMC_RXUDP_ERR_OCTETS_RD(val5);

	pr_debug("dbgpr_regs: MAC_PMTCSR:%#x\n"
	      "dbgpr_regs: MMC_RXICMP_ERR_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXICMP_GD_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXTCP_ERR_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXTCP_GD_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXUDP_ERR_OCTETS:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_RXUDP_GD_OCTETS_RD(val0);
	MMC_RXIPV6_NOPAY_OCTETS_RD(val1);
	MMC_RXIPV6_HDRERR_OCTETS_RD(val2);
	MMC_RXIPV6_GD_OCTETS_RD(val3);
	MMC_RXIPV4_UDSBL_OCTETS_RD(val4);
	MMC_RXIPV4_FRAG_OCTETS_RD(val5);

	pr_debug("dbgpr_regs: MMC_RXUDP_GD_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV6_NOPAY_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV6_HDRERR_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV6_GD_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV4_UDSBL_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV4_FRAG_OCTETS:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_RXIPV4_NOPAY_OCTETS_RD(val0);
	MMC_RXIPV4_HDRERR_OCTETS_RD(val1);
	MMC_RXIPV4_GD_OCTETS_RD(val2);
	MMC_RXICMP_ERR_PKTS_RD(val3);
	MMC_RXICMP_GD_PKTS_RD(val4);
	MMC_RXTCP_ERR_PKTS_RD(val5);

	pr_debug("dbgpr_regs: MMC_RXIPV4_NOPAY_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV4_HDRERR_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV4_GD_OCTETS:%#x\n"
	      "dbgpr_regs: MMC_RXICMP_ERR_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXICMP_GD_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXTCP_ERR_PKTS:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_RXTCP_GD_PKTS_RD(val0);
	MMC_RXUDP_ERR_PKTS_RD(val1);
	MMC_RXUDP_GD_PKTS_RD(val2);
	MMC_RXIPV6_NOPAY_PKTS_RD(val3);
	MMC_RXIPV6_HDRERR_PKTS_RD(val4);
	MMC_RXIPV6_GD_PKTS_RD(val5);

	pr_debug("dbgpr_regs: MMC_RXTCP_GD_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXUDP_ERR_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXUDP_GD_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV6_NOPAY_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV6_HDRERR_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV6_GD_PKTS:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_RXIPV4_UBSBL_PKTS_RD(val0);
	MMC_RXIPV4_FRAG_PKTS_RD(val1);
	MMC_RXIPV4_NOPAY_PKTS_RD(val2);
	MMC_RXIPV4_HDRERR_PKTS_RD(val3);
	MMC_RXIPV4_GD_PKTS_RD(val4);
	MMC_RXCTRLPACKETS_G_RD(val5);

	pr_debug("dbgpr_regs: MMC_RXIPV4_UBSBL_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV4_FRAG_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV4_NOPAY_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV4_HDRERR_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXIPV4_GD_PKTS:%#x\n"
	      "dbgpr_regs: MMC_RXCTRLPACKETS_G:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_RXRCVERROR_RD(val0);
	MMC_RXWATCHDOGERROR_RD(val1);
	MMC_RXVLANPACKETS_GB_RD(val2);
	MMC_RXFIFOOVERFLOW_RD(val3);
	MMC_RXPAUSEPACKETS_RD(val4);
	MMC_RXOUTOFRANGETYPE_RD(val5);

	pr_debug("dbgpr_regs: MMC_RXRCVERROR:%#x\n"
	      "dbgpr_regs: MMC_RXWATCHDOGERROR:%#x\n"
	      "dbgpr_regs: MMC_RXVLANPACKETS_GB:%#x\n"
	      "dbgpr_regs: MMC_RXFIFOOVERFLOW:%#x\n"
	      "dbgpr_regs: MMC_RXPAUSEPACKETS:%#x\n"
	      "dbgpr_regs: MMC_RXOUTOFRANGETYPE:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_RXLENGTHERROR_RD(val0);
	MMC_RXUNICASTPACKETS_G_RD(val1);
	MMC_RX1024TOMAXOCTETS_GB_RD(val2);
	MMC_RX512TO1023OCTETS_GB_RD(val3);
	MMC_RX256TO511OCTETS_GB_RD(val4);
	MMC_RX128TO255OCTETS_GB_RD(val5);

	pr_debug("dbgpr_regs: MMC_RXLENGTHERROR:%#x\n"
	      "dbgpr_regs: MMC_RXUNICASTPACKETS_G:%#x\n"
	      "dbgpr_regs: MMC_RX1024TOMAXOCTETS_GB:%#x\n"
	      "dbgpr_regs: MMC_RX512TO1023OCTETS_GB:%#x\n"
	      "dbgpr_regs: MMC_RX256TO511OCTETS_GB:%#x\n"
	      "dbgpr_regs: MMC_RX128TO255OCTETS_GB:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_RX65TO127OCTETS_GB_RD(val0);
	MMC_RX64OCTETS_GB_RD(val1);
	MMC_RXOVERSIZE_G_RD(val2);
	MMC_RXUNDERSIZE_G_RD(val3);
	MMC_RXJABBERERROR_RD(val4);
	MMC_RXRUNTERROR_RD(val5);

	pr_debug("dbgpr_regs: MMC_RX65TO127OCTETS_GB:%#x\n"
	      "dbgpr_regs: MMC_RX64OCTETS_GB:%#x\n"
	      "dbgpr_regs: MMC_RXOVERSIZE_G:%#x\n"
	      "dbgpr_regs: MMC_RXUNDERSIZE_G:%#x\n"
	      "dbgpr_regs: MMC_RXJABBERERROR:%#x\n"
	      "dbgpr_regs: MMC_RXRUNTERROR:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_RXALIGNMENTERROR_RD(val0);
	MMC_RXCRCERROR_RD(val1);
	MMC_RXMULTICASTPACKETS_G_RD(val2);
	MMC_RXBROADCASTPACKETS_G_RD(val3);
	MMC_RXOCTETCOUNT_G_RD(val4);
	MMC_RXOCTETCOUNT_GB_RD(val5);

	pr_debug("dbgpr_regs: MMC_RXALIGNMENTERROR:%#x\n"
	      "dbgpr_regs: MMC_RXCRCERROR:%#x\n"
	      "dbgpr_regs: MMC_RXMULTICASTPACKETS_G:%#x\n"
	      "dbgpr_regs: MMC_RXBROADCASTPACKETS_G:%#x\n"
	      "dbgpr_regs: MMC_RXOCTETCOUNT_G:%#x\n"
	      "dbgpr_regs: MMC_RXOCTETCOUNT_GB:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_RXPACKETCOUNT_GB_RD(val0);
	MMC_TXOVERSIZE_G_RD(val1);
	MMC_TXVLANPACKETS_G_RD(val2);
	MMC_TXPAUSEPACKETS_RD(val3);
	MMC_TXEXCESSDEF_RD(val4);
	MMC_TXPACKETSCOUNT_G_RD(val5);

	pr_debug("dbgpr_regs: MMC_RXPACKETCOUNT_GB:%#x\n"
	      "dbgpr_regs: MMC_TXOVERSIZE_G:%#x\n"
	      "dbgpr_regs: MMC_TXVLANPACKETS_G:%#x\n"
	      "dbgpr_regs: MMC_TXPAUSEPACKETS:%#x\n"
	      "dbgpr_regs: MMC_TXEXCESSDEF:%#x\n"
	      "dbgpr_regs: MMC_TXPACKETSCOUNT_G:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_TXOCTETCOUNT_G_RD(val0);
	MMC_TXCARRIERERROR_RD(val1);
	MMC_TXEXESSCOL_RD(val2);
	MMC_TXLATECOL_RD(val3);
	MMC_TXDEFERRED_RD(val4);
	MMC_TXMULTICOL_G_RD(val5);

	pr_debug("dbgpr_regs: MMC_TXOCTETCOUNT_G:%#x\n"
	      "dbgpr_regs: MMC_TXCARRIERERROR:%#x\n"
	      "dbgpr_regs: MMC_TXEXESSCOL:%#x\n"
	      "dbgpr_regs: MMC_TXLATECOL:%#x\n"
	      "dbgpr_regs: MMC_TXDEFERRED:%#x\n"
	      "dbgpr_regs: MMC_TXMULTICOL_G:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_TXSINGLECOL_G_RD(val0);
	MMC_TXUNDERFLOWERROR_RD(val1);
	MMC_TXBROADCASTPACKETS_GB_RD(val2);
	MMC_TXMULTICASTPACKETS_GB_RD(val3);
	MMC_TXUNICASTPACKETS_GB_RD(val4);
	MMC_TX1024TOMAXOCTETS_GB_RD(val5);

	pr_debug("dbgpr_regs: MMC_TXSINGLECOL_G:%#x\n"
	      "dbgpr_regs: MMC_TXUNDERFLOWERROR:%#x\n"
	      "dbgpr_regs: MMC_TXBROADCASTPACKETS_GB:%#x\n"
	      "dbgpr_regs: MMC_TXMULTICASTPACKETS_GB:%#x\n"
	      "dbgpr_regs: MMC_TXUNICASTPACKETS_GB:%#x\n"
	      "dbgpr_regs: MMC_TX1024TOMAXOCTETS_GB:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_TX512TO1023OCTETS_GB_RD(val0);
	MMC_TX256TO511OCTETS_GB_RD(val1);
	MMC_TX128TO255OCTETS_GB_RD(val2);
	MMC_TX65TO127OCTETS_GB_RD(val3);
	MMC_TX64OCTETS_GB_RD(val4);
	MMC_TXMULTICASTPACKETS_G_RD(val5);

	pr_debug("dbgpr_regs: MMC_TX512TO1023OCTETS_GB:%#x\n"
	      "dbgpr_regs: MMC_TX256TO511OCTETS_GB:%#x\n"
	      "dbgpr_regs: MMC_TX128TO255OCTETS_GB:%#x\n"
	      "dbgpr_regs: MMC_TX65TO127OCTETS_GB:%#x\n"
	      "dbgpr_regs: MMC_TX64OCTETS_GB:%#x\n"
	      "dbgpr_regs: MMC_TXMULTICASTPACKETS_G:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_TXBROADCASTPACKETS_G_RD(val0);
	MMC_TXPACKETCOUNT_GB_RD(val1);
	MMC_TXOCTETCOUNT_GB_RD(val2);
	MMC_IPC_INTR_RX_RD(val3);
	MMC_IPC_INTR_MASK_RX_RD(val4);
	MMC_INTR_MASK_TX_RD(val5);

	pr_debug("dbgpr_regs: MMC_TXBROADCASTPACKETS_G:%#x\n"
	      "dbgpr_regs: MMC_TXPACKETCOUNT_GB:%#x\n"
	      "dbgpr_regs: MMC_TXOCTETCOUNT_GB:%#x\n"
	      "dbgpr_regs: MMC_IPC_INTR_RX:%#x\n"
	      "dbgpr_regs: MMC_IPC_INTR_MASK_RX:%#x\n"
	      "dbgpr_regs: MMC_INTR_MASK_TX:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MMC_INTR_MASK_RX_RD(val0);
	MMC_INTR_TX_RD(val1);
	MMC_INTR_RX_RD(val2);
	MMC_CNTRL_RD(val3);
	MAC_MA1LR_RD(val4);
	MAC_MA1HR_RD(val5);

	pr_debug("dbgpr_regs: MMC_INTR_MASK_RX:%#x\n"
	      "dbgpr_regs: MMC_INTR_TX:%#x\n"
	      "dbgpr_regs: MMC_INTR_RX:%#x\n"
	      "dbgpr_regs: MMC_CNTRL:%#x\n"
	      "dbgpr_regs: MAC_MA1LR:%#x\n"
	      "dbgpr_regs: MAC_MA1HR:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MAC_MA0LR_RD(val0);
	MAC_MA0HR_RD(val1);
	MAC_GPIOR_RD(val2);
	MAC_GMIIDR_RD(val3);
	MAC_GMIIAR_RD(val4);
	MAC_HFR2_RD(val5);

	pr_debug("dbgpr_regs: MAC_MA0LR:%#x\n"
	      "dbgpr_regs: MAC_MA0HR:%#x\n"
	      "dbgpr_regs: MAC_GPIOR:%#x\n"
	      "dbgpr_regs: MAC_GMIIDR:%#x\n"
	      "dbgpr_regs: MAC_GMIIAR:%#x\n"
	      "dbgpr_regs: MAC_HFR2:%#x\n", val0, val1, val2, val3, val4, val5);

	MAC_HFR1_RD(val0);
	MAC_HFR0_RD(val1);
	MAC_MDR_RD(val2);
	MAC_VR_RD(val3);
	MAC_HTR7_RD(val4);
	MAC_HTR6_RD(val5);

	pr_debug("dbgpr_regs: MAC_HFR1:%#x\n"
	      "dbgpr_regs: MAC_HFR0:%#x\n"
	      "dbgpr_regs: MAC_MDR:%#x\n"
	      "dbgpr_regs: MAC_VR:%#x\n"
	      "dbgpr_regs: MAC_HTR7:%#x\n"
	      "dbgpr_regs: MAC_HTR6:%#x\n", val0, val1, val2, val3, val4, val5);

	MAC_HTR5_RD(val0);
	MAC_HTR4_RD(val1);
	MAC_HTR3_RD(val2);
	MAC_HTR2_RD(val3);
	MAC_HTR1_RD(val4);
	MAC_HTR0_RD(val5);

	pr_debug("dbgpr_regs: MAC_HTR5:%#x\n"
	      "dbgpr_regs: MAC_HTR4:%#x\n"
	      "dbgpr_regs: MAC_HTR3:%#x\n"
	      "dbgpr_regs: MAC_HTR2:%#x\n"
	      "dbgpr_regs: MAC_HTR1:%#x\n"
	      "dbgpr_regs: MAC_HTR0:%#x\n", val0, val1, val2, val3, val4, val5);

	DMA_RIWTR7_RD(val0);
	DMA_RIWTR6_RD(val1);
	DMA_RIWTR5_RD(val2);
	DMA_RIWTR4_RD(val3);
	DMA_RIWTR3_RD(val4);
	DMA_RIWTR2_RD(val5);

	pr_debug("dbgpr_regs: DMA_RIWTR7:%#x\n"
	      "dbgpr_regs: DMA_RIWTR6:%#x\n"
	      "dbgpr_regs: DMA_RIWTR5:%#x\n"
	      "dbgpr_regs: DMA_RIWTR4:%#x\n"
	      "dbgpr_regs: DMA_RIWTR3:%#x\n"
	      "dbgpr_regs: DMA_RIWTR2:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_RIWTR1_RD(val0);
	DMA_RIWTR0_RD(val1);
	DMA_RDRLR7_RD(val2);
	DMA_RDRLR6_RD(val3);
	DMA_RDRLR5_RD(val4);
	DMA_RDRLR4_RD(val5);

	pr_debug("dbgpr_regs: DMA_RIWTR1:%#x\n"
	      "dbgpr_regs: DMA_RIWTR0:%#x\n"
	      "dbgpr_regs: DMA_RDRLR7:%#x\n"
	      "dbgpr_regs: DMA_RDRLR6:%#x\n"
	      "dbgpr_regs: DMA_RDRLR5:%#x\n"
	      "dbgpr_regs: DMA_RDRLR4:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_RDRLR3_RD(val0);
	DMA_RDRLR2_RD(val1);
	DMA_RDRLR1_RD(val2);
	DMA_RDRLR0_RD(val3);
	DMA_TDRLR7_RD(val4);
	DMA_TDRLR6_RD(val5);

	pr_debug("dbgpr_regs: DMA_RDRLR3:%#x\n"
	      "dbgpr_regs: DMA_RDRLR2:%#x\n"
	      "dbgpr_regs: DMA_RDRLR1:%#x\n"
	      "dbgpr_regs: DMA_RDRLR0:%#x\n"
	      "dbgpr_regs: DMA_TDRLR7:%#x\n"
	      "dbgpr_regs: DMA_TDRLR6:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_TDRLR5_RD(val0);
	DMA_TDRLR4_RD(val1);
	DMA_TDRLR3_RD(val2);
	DMA_TDRLR2_RD(val3);
	DMA_TDRLR1_RD(val4);
	DMA_TDRLR0_RD(val5);

	pr_debug("dbgpr_regs: DMA_TDRLR5:%#x\n"
	      "dbgpr_regs: DMA_TDRLR4:%#x\n"
	      "dbgpr_regs: DMA_TDRLR3:%#x\n"
	      "dbgpr_regs: DMA_TDRLR2:%#x\n"
	      "dbgpr_regs: DMA_TDRLR1:%#x\n"
	      "dbgpr_regs: DMA_TDRLR0:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_RDTP_RPDR7_RD(val0);
	DMA_RDTP_RPDR6_RD(val1);
	DMA_RDTP_RPDR5_RD(val2);
	DMA_RDTP_RPDR4_RD(val3);
	DMA_RDTP_RPDR3_RD(val4);
	DMA_RDTP_RPDR2_RD(val5);

	pr_debug("dbgpr_regs: DMA_RDTP_RPDR7:%#x\n"
	      "dbgpr_regs: DMA_RDTP_RPDR6:%#x\n"
	      "dbgpr_regs: DMA_RDTP_RPDR5:%#x\n"
	      "dbgpr_regs: DMA_RDTP_RPDR4:%#x\n"
	      "dbgpr_regs: DMA_RDTP_RPDR3:%#x\n"
	      "dbgpr_regs: DMA_RDTP_RPDR2:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_RDTP_RPDR1_RD(val0);
	DMA_RDTP_RPDR0_RD(val1);
	DMA_TDTP_TPDR7_RD(val2);
	DMA_TDTP_TPDR6_RD(val3);
	DMA_TDTP_TPDR5_RD(val4);
	DMA_TDTP_TPDR4_RD(val5);

	pr_debug("dbgpr_regs: DMA_RDTP_RPDR1:%#x\n"
	      "dbgpr_regs: DMA_RDTP_RPDR0:%#x\n"
	      "dbgpr_regs: DMA_TDTP_TPDR7:%#x\n"
	      "dbgpr_regs: DMA_TDTP_TPDR6:%#x\n"
	      "dbgpr_regs: DMA_TDTP_TPDR5:%#x\n"
	      "dbgpr_regs: DMA_TDTP_TPDR4:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_TDTP_TPDR3_RD(val0);
	DMA_TDTP_TPDR2_RD(val1);
	DMA_TDTP_TPDR1_RD(val2);
	DMA_TDTP_TPDR0_RD(val3);
	DMA_RDLAR7_RD(val4);
	DMA_RDLAR6_RD(val5);

	pr_debug("dbgpr_regs: DMA_TDTP_TPDR3:%#x\n"
	      "dbgpr_regs: DMA_TDTP_TPDR2:%#x\n"
	      "dbgpr_regs: DMA_TDTP_TPDR1:%#x\n"
	      "dbgpr_regs: DMA_TDTP_TPDR0:%#x\n"
	      "dbgpr_regs: DMA_RDLAR7:%#x\n"
	      "dbgpr_regs: DMA_RDLAR6:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_RDLAR5_RD(val0);
	DMA_RDLAR4_RD(val1);
	DMA_RDLAR3_RD(val2);
	DMA_RDLAR2_RD(val3);
	DMA_RDLAR1_RD(val4);
	DMA_RDLAR0_RD(val5);

	pr_debug("dbgpr_regs: DMA_RDLAR5:%#x\n"
	      "dbgpr_regs: DMA_RDLAR4:%#x\n"
	      "dbgpr_regs: DMA_RDLAR3:%#x\n"
	      "dbgpr_regs: DMA_RDLAR2:%#x\n"
	      "dbgpr_regs: DMA_RDLAR1:%#x\n"
	      "dbgpr_regs: DMA_RDLAR0:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_TDLAR7_RD(val0);
	DMA_TDLAR6_RD(val1);
	DMA_TDLAR5_RD(val2);
	DMA_TDLAR4_RD(val3);
	DMA_TDLAR3_RD(val4);
	DMA_TDLAR2_RD(val5);

	pr_debug("dbgpr_regs: DMA_TDLAR7:%#x\n"
	      "dbgpr_regs: DMA_TDLAR6:%#x\n"
	      "dbgpr_regs: DMA_TDLAR5:%#x\n"
	      "dbgpr_regs: DMA_TDLAR4:%#x\n"
	      "dbgpr_regs: DMA_TDLAR3:%#x\n"
	      "dbgpr_regs: DMA_TDLAR2:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_TDLAR1_RD(val0);
	DMA_TDLAR0_RD(val1);
	DMA_IER7_RD(val2);
	DMA_IER6_RD(val3);
	DMA_IER5_RD(val4);
	DMA_IER4_RD(val5);

	pr_debug("dbgpr_regs: DMA_TDLAR1:%#x\n"
	      "dbgpr_regs: DMA_TDLAR0:%#x\n"
	      "dbgpr_regs: DMA_IER7:%#x\n"
	      "dbgpr_regs: DMA_IER6:%#x\n"
	      "dbgpr_regs: DMA_IER5:%#x\n"
	      "dbgpr_regs: DMA_IER4:%#x\n", val0, val1, val2, val3, val4, val5);

	DMA_IER3_RD(val0);
	DMA_IER2_RD(val1);
	DMA_IER1_RD(val2);
	DMA_IER0_RD(val3);
	MAC_IMR_RD(val4);
	MAC_ISR_RD(val5);

	pr_debug("dbgpr_regs: DMA_IER3:%#x\n"
	      "dbgpr_regs: DMA_IER2:%#x\n"
	      "dbgpr_regs: DMA_IER1:%#x\n"
	      "dbgpr_regs: DMA_IER0:%#x\n"
	      "dbgpr_regs: MAC_IMR:%#x\n"
	      "dbgpr_regs: MAC_ISR:%#x\n", val0, val1, val2, val3, val4, val5);

	MTL_ISR_RD(val0);
	DMA_SR7_RD(val1);
	DMA_SR6_RD(val2);
	DMA_SR5_RD(val3);
	DMA_SR4_RD(val4);
	DMA_SR3_RD(val5);

	pr_debug("dbgpr_regs: MTL_ISR:%#x\n"
	      "dbgpr_regs: DMA_SR7:%#x\n"
	      "dbgpr_regs: DMA_SR6:%#x\n"
	      "dbgpr_regs: DMA_SR5:%#x\n"
	      "dbgpr_regs: DMA_SR4:%#x\n"
	      "dbgpr_regs: DMA_SR3:%#x\n", val0, val1, val2, val3, val4, val5);

	DMA_SR2_RD(val0);
	DMA_SR1_RD(val1);
	DMA_SR0_RD(val2);
	DMA_ISR_RD(val3);
	DMA_DSR2_RD(val4);
	DMA_DSR1_RD(val5);

	pr_debug("dbgpr_regs: DMA_SR2:%#x\n"
	      "dbgpr_regs: DMA_SR1:%#x\n"
	      "dbgpr_regs: DMA_SR0:%#x\n"
	      "dbgpr_regs: DMA_ISR:%#x\n"
	      "dbgpr_regs: DMA_DSR2:%#x\n"
	      "dbgpr_regs: DMA_DSR1:%#x\n", val0, val1, val2, val3, val4, val5);

	DMA_DSR0_RD(val0);
	MTL_Q0RDR_RD(val1);
	MTL_Q0ESR_RD(val2);
	MTL_Q0TDR_RD(val3);
	DMA_CHRBAR7_RD(val4);
	DMA_CHRBAR6_RD(val5);

	pr_debug("dbgpr_regs: DMA_DSR0:%#x\n"
	      "dbgpr_regs: MTL_Q0RDR:%#x\n"
	      "dbgpr_regs: MTL_Q0ESR:%#x\n"
	      "dbgpr_regs: MTL_Q0TDR:%#x\n"
	      "dbgpr_regs: DMA_CHRBAR7:%#x\n"
	      "dbgpr_regs: DMA_CHRBAR6:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_CHRBAR5_RD(val0);
	DMA_CHRBAR4_RD(val1);
	DMA_CHRBAR3_RD(val2);
	DMA_CHRBAR2_RD(val3);
	DMA_CHRBAR1_RD(val4);
	DMA_CHRBAR0_RD(val5);

	pr_debug("dbgpr_regs: DMA_CHRBAR5:%#x\n"
	      "dbgpr_regs: DMA_CHRBAR4:%#x\n"
	      "dbgpr_regs: DMA_CHRBAR3:%#x\n"
	      "dbgpr_regs: DMA_CHRBAR2:%#x\n"
	      "dbgpr_regs: DMA_CHRBAR1:%#x\n"
	      "dbgpr_regs: DMA_CHRBAR0:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_CHTBAR7_RD(val0);
	DMA_CHTBAR6_RD(val1);
	DMA_CHTBAR5_RD(val2);
	DMA_CHTBAR4_RD(val3);
	DMA_CHTBAR3_RD(val4);
	DMA_CHTBAR2_RD(val5);

	pr_debug("dbgpr_regs: DMA_CHTBAR7:%#x\n"
	      "dbgpr_regs: DMA_CHTBAR6:%#x\n"
	      "dbgpr_regs: DMA_CHTBAR5:%#x\n"
	      "dbgpr_regs: DMA_CHTBAR4:%#x\n"
	      "dbgpr_regs: DMA_CHTBAR3:%#x\n"
	      "dbgpr_regs: DMA_CHTBAR2:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_CHTBAR1_RD(val0);
	DMA_CHTBAR0_RD(val1);
	DMA_CHRDR7_RD(val2);
	DMA_CHRDR6_RD(val3);
	DMA_CHRDR5_RD(val4);
	DMA_CHRDR4_RD(val5);

	pr_debug("dbgpr_regs: DMA_CHTBAR1:%#x\n"
	      "dbgpr_regs: DMA_CHTBAR0:%#x\n"
	      "dbgpr_regs: DMA_CHRDR7:%#x\n"
	      "dbgpr_regs: DMA_CHRDR6:%#x\n"
	      "dbgpr_regs: DMA_CHRDR5:%#x\n"
	      "dbgpr_regs: DMA_CHRDR4:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_CHRDR3_RD(val0);
	DMA_CHRDR2_RD(val1);
	DMA_CHRDR1_RD(val2);
	DMA_CHRDR0_RD(val3);
	DMA_CHTDR7_RD(val4);
	DMA_CHTDR6_RD(val5);

	pr_debug("dbgpr_regs: DMA_CHRDR3:%#x\n"
	      "dbgpr_regs: DMA_CHRDR2:%#x\n"
	      "dbgpr_regs: DMA_CHRDR1:%#x\n"
	      "dbgpr_regs: DMA_CHRDR0:%#x\n"
	      "dbgpr_regs: DMA_CHTDR7:%#x\n"
	      "dbgpr_regs: DMA_CHTDR6:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_CHTDR5_RD(val0);
	DMA_CHTDR4_RD(val1);
	DMA_CHTDR3_RD(val2);
	DMA_CHTDR2_RD(val3);
	DMA_CHTDR1_RD(val4);
	DMA_CHTDR0_RD(val5);

	pr_debug("dbgpr_regs: DMA_CHTDR5:%#x\n"
	      "dbgpr_regs: DMA_CHTDR4:%#x\n"
	      "dbgpr_regs: DMA_CHTDR3:%#x\n"
	      "dbgpr_regs: DMA_CHTDR2:%#x\n"
	      "dbgpr_regs: DMA_CHTDR1:%#x\n"
	      "dbgpr_regs: DMA_CHTDR0:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_SFCSR7_RD(val0);
	DMA_SFCSR6_RD(val1);
	DMA_SFCSR5_RD(val2);
	DMA_SFCSR4_RD(val3);
	DMA_SFCSR3_RD(val4);
	DMA_SFCSR2_RD(val5);

	pr_debug("dbgpr_regs: DMA_SFCSR7:%#x\n"
	      "dbgpr_regs: DMA_SFCSR6:%#x\n"
	      "dbgpr_regs: DMA_SFCSR5:%#x\n"
	      "dbgpr_regs: DMA_SFCSR4:%#x\n"
	      "dbgpr_regs: DMA_SFCSR3:%#x\n"
	      "dbgpr_regs: DMA_SFCSR2:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_SFCSR1_RD(val0);
	DMA_SFCSR0_RD(val1);
	MAC_IVLANTIRR_RD(val2);
	MAC_VLANTIRR_RD(val3);
	MAC_VLANHTR_RD(val4);
	MAC_VLANTR_RD(val5);

	pr_debug("dbgpr_regs: DMA_SFCSR1:%#x\n"
	      "dbgpr_regs: DMA_SFCSR0:%#x\n"
	      "dbgpr_regs: MAC_IVLANTIRR:%#x\n"
	      "dbgpr_regs: MAC_VLANTIRR:%#x\n"
	      "dbgpr_regs: MAC_VLANHTR:%#x\n"
	      "dbgpr_regs: MAC_VLANTR:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_SBUS_RD(val0);
	DMA_BMR_RD(val1);
	MTL_Q0RCR_RD(val2);
	MTL_Q0OCR_RD(val3);
	MTL_Q0ROMR_RD(val4);
	MTL_Q0QR_RD(val5);

	pr_debug("dbgpr_regs: DMA_SBUS:%#x\n"
	      "dbgpr_regs: DMA_BMR:%#x\n"
	      "dbgpr_regs: MTL_Q0RCR:%#x\n"
	      "dbgpr_regs: MTL_Q0OCR:%#x\n"
	      "dbgpr_regs: MTL_Q0ROMR:%#x\n"
	      "dbgpr_regs: MTL_Q0QR:%#x\n", val0, val1, val2, val3, val4, val5);

	MTL_Q0ECR_RD(val0);
	MTL_Q0UCR_RD(val1);
	MTL_Q0TOMR_RD(val2);
	MTL_RQDCM1R_RD(val3);
	MTL_RQDCM0R_RD(val4);
	MTL_FDDR_RD(val5);

	pr_debug("dbgpr_regs: MTL_Q0ECR:%#x\n"
	      "dbgpr_regs: MTL_Q0UCR:%#x\n"
	      "dbgpr_regs: MTL_Q0TOMR:%#x\n"
	      "dbgpr_regs: MTL_RQDCM1R:%#x\n"
	      "dbgpr_regs: MTL_RQDCM0R:%#x\n"
	      "dbgpr_regs: MTL_FDDR:%#x\n", val0, val1, val2, val3, val4, val5);

	MTL_FDACS_RD(val0);
	MTL_OMR_RD(val1);
	MAC_RQC1R_RD(val2);
	MAC_RQC0R_RD(val3);
	MAC_TQPM1R_RD(val4);
	MAC_TQPM0R_RD(val5);

	pr_debug("dbgpr_regs: MTL_FDACS:%#x\n"
	      "dbgpr_regs: MTL_OMR:%#x\n"
	      "dbgpr_regs: MAC_RQC1R:%#x\n"
	      "dbgpr_regs: MAC_RQC0R:%#x\n"
	      "dbgpr_regs: MAC_TQPM1R:%#x\n"
	      "dbgpr_regs: MAC_TQPM0R:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MAC_RFCR_RD(val0);
	MAC_QTFCR7_RD(val1);
	MAC_QTFCR6_RD(val2);
	MAC_QTFCR5_RD(val3);
	MAC_QTFCR4_RD(val4);
	MAC_QTFCR3_RD(val5);

	pr_debug("dbgpr_regs: MAC_RFCR:%#x\n"
	      "dbgpr_regs: MAC_QTFCR7:%#x\n"
	      "dbgpr_regs: MAC_QTFCR6:%#x\n"
	      "dbgpr_regs: MAC_QTFCR5:%#x\n"
	      "dbgpr_regs: MAC_QTFCR4:%#x\n"
	      "dbgpr_regs: MAC_QTFCR3:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	MAC_QTFCR2_RD(val0);
	MAC_QTFCR1_RD(val1);
	MAC_Q0TFCR_RD(val2);
	DMA_AXI4CR7_RD(val3);
	DMA_AXI4CR6_RD(val4);
	DMA_AXI4CR5_RD(val5);

	pr_debug("dbgpr_regs: MAC_QTFCR2:%#x\n"
	      "dbgpr_regs: MAC_QTFCR1:%#x\n"
	      "dbgpr_regs: MAC_Q0TFCR:%#x\n"
	      "dbgpr_regs: DMA_AXI4CR7:%#x\n"
	      "dbgpr_regs: DMA_AXI4CR6:%#x\n"
	      "dbgpr_regs: DMA_AXI4CR5:%#x\n",
	      val0, val1, val2, val3, val4, val5);

	DMA_AXI4CR4_RD(val0);
	DMA_AXI4CR3_RD(val1);
	DMA_AXI4CR2_RD(val2);
	DMA_AXI4CR1_RD(val3);
	DMA_AXI4CR0_RD(val4);
	DMA_RCR7_RD(val5);

	pr_debug("dbgpr_regs: DMA_AXI4CR4:%#x\n"
	      "dbgpr_regs: DMA_AXI4CR3:%#x\n"
	      "dbgpr_regs: DMA_AXI4CR2:%#x\n"
	      "dbgpr_regs: DMA_AXI4CR1:%#x\n"
	      "dbgpr_regs: DMA_AXI4CR0:%#x\n"
	      "dbgpr_regs: DMA_RCR7:%#x\n", val0, val1, val2, val3, val4, val5);

	DMA_RCR6_RD(val0);
	DMA_RCR5_RD(val1);
	DMA_RCR4_RD(val2);
	DMA_RCR3_RD(val3);
	DMA_RCR2_RD(val4);
	DMA_RCR1_RD(val5);

	pr_debug("dbgpr_regs: DMA_RCR6:%#x\n"
	      "dbgpr_regs: DMA_RCR5:%#x\n"
	      "dbgpr_regs: DMA_RCR4:%#x\n"
	      "dbgpr_regs: DMA_RCR3:%#x\n"
	      "dbgpr_regs: DMA_RCR2:%#x\n"
	      "dbgpr_regs: DMA_RCR1:%#x\n", val0, val1, val2, val3, val4, val5);

	DMA_RCR0_RD(val0);
	DMA_TCR7_RD(val1);
	DMA_TCR6_RD(val2);
	DMA_TCR5_RD(val3);
	DMA_TCR4_RD(val4);
	DMA_TCR3_RD(val5);

	pr_debug("dbgpr_regs: DMA_RCR0:%#x\n"
	      "dbgpr_regs: DMA_TCR7:%#x\n"
	      "dbgpr_regs: DMA_TCR6:%#x\n"
	      "dbgpr_regs: DMA_TCR5:%#x\n"
	      "dbgpr_regs: DMA_TCR4:%#x\n"
	      "dbgpr_regs: DMA_TCR3:%#x\n", val0, val1, val2, val3, val4, val5);

	DMA_TCR2_RD(val0);
	DMA_TCR1_RD(val1);
	DMA_TCR0_RD(val2);
	DMA_CR7_RD(val3);
	DMA_CR6_RD(val4);
	DMA_CR5_RD(val5);

	pr_debug("dbgpr_regs: DMA_TCR2:%#x\n"
	      "dbgpr_regs: DMA_TCR1:%#x\n"
	      "dbgpr_regs: DMA_TCR0:%#x\n"
	      "dbgpr_regs: DMA_CR7:%#x\n"
	      "dbgpr_regs: DMA_CR6:%#x\n"
	      "dbgpr_regs: DMA_CR5:%#x\n", val0, val1, val2, val3, val4, val5);

	DMA_CR4_RD(val0);
	DMA_CR3_RD(val1);
	DMA_CR2_RD(val2);
	DMA_CR1_RD(val3);
	DMA_CR0_RD(val4);
	MAC_WTR_RD(val5);

	pr_debug("dbgpr_regs: DMA_CR4:%#x\n"
	      "dbgpr_regs: DMA_CR3:%#x\n"
	      "dbgpr_regs: DMA_CR2:%#x\n"
	      "dbgpr_regs: DMA_CR1:%#x\n"
	      "dbgpr_regs: DMA_CR0:%#x\n"
	      "dbgpr_regs: MAC_WTR:%#x\n", val0, val1, val2, val3, val4, val5);

	MAC_MPFR_RD(val0);
	MAC_MECR_RD(val1);
	MAC_MCR_RD(val2);

	pr_debug("dbgpr_regs: MAC_MPFR:%#x\n"
	      "dbgpr_regs: MAC_MECR:%#x\n"
	      "dbgpr_regs: MAC_MCR:%#x\n", val0, val1, val2);

	return;
}
#endif

/*!
 * \details This function is invoked by eqos_start_xmit and
 * process_tx_completions function for dumping the TX descriptor contents
 * which are prepared for packet transmission and which are transmitted by
 * device. It is mainly used during development phase for debug purpose. Use
 * of these function may affect the performance during normal operation.
 *
 * \param[in] pdata – pointer to private data structure.
 * \param[in] first_desc_idx – first descriptor index for the current
 *		transfer.
 * \param[in] last_desc_idx – last descriptor index for the current transfer.
 * \param[in] flag – to indicate from which function it is called.
 *
 * \return void
 */

void dump_tx_desc(struct eqos_prv_data *pdata, int first_desc_idx,
		  int last_desc_idx, int flag, UINT qinx)
{
	int i;
	struct s_tx_desc *desc = NULL;
	UINT ctxt;

	if (first_desc_idx == last_desc_idx) {
		desc = GET_TX_DESC_PTR(qinx, first_desc_idx);

		TX_NORMAL_DESC_TDES3_CTXT_RD(desc->tdes3, ctxt);

		pr_err("\n%s[%02d %4p %03d %s] = %#x:%#x:%#x:%#x\n",
		       (ctxt == 1) ? "TX_CONTXT_DESC" : "ptx_desc",
		       qinx, desc, first_desc_idx,
		       ((flag == 1) ? "QUEUED FOR TRANSMISSION" :
			((flag ==
			  0) ? "FREED/FETCHED BY DEVICE" : "DEBUG DESC DUMP")),
		       desc->tdes0, desc->tdes1, desc->tdes2, desc->tdes3);
	} else {
		int lp_cnt;
		if (first_desc_idx > last_desc_idx)
			lp_cnt = last_desc_idx + TX_DESC_CNT - first_desc_idx;
		else
			lp_cnt = last_desc_idx - first_desc_idx;

		for (i = first_desc_idx; lp_cnt >= 0; lp_cnt--) {
			desc = GET_TX_DESC_PTR(qinx, i);

			TX_NORMAL_DESC_TDES3_CTXT_RD(desc->tdes3, ctxt);

			pr_err("\n%s[%02d %4p %03d %s] = %#x:%#x:%#x:%#x\n",
			       (ctxt ==
				1) ? "TX_CONTXT_DESC" : "ptx_desc", qinx,
			       desc, i,
			       ((flag ==
				 1) ? "QUEUED FOR TRANSMISSION" :
				"FREED/FETCHED BY DEVICE"), desc->tdes0,
			       desc->tdes1, desc->tdes2, desc->tdes3);
			INCR_TX_DESC_INDEX(i, 1);
		}
	}
}

/*!
 * \details This function is invoked by poll function for dumping the
 * RX descriptor contents. It is mainly used during development phase for
 * debug purpose. Use of these function may affect the performance during
 * normal operation
 *
 * \param[in] pdata – pointer to private data structure.
 *
 * \return void
 */

void dump_rx_desc(UINT qinx, struct s_rx_desc *desc, int desc_idx)
{
	pr_err("\nprx_desc[%02d %4p %03d RECEIVED FROM DEVICE]"
	       " = %#x:%#x:%#x:%#x",
	       qinx, desc, desc_idx, desc->rdes0, desc->rdes1,
	       desc->rdes2, desc->rdes3);
}

/*!
 * \details This function is invoked by start_xmit and poll function for
 * dumping the content of packet to be transmitted by device or received
 * from device. It is mainly used during development phase for debug purpose.
 * Use of these functions may affect the performance during normal operation.
 *
 * \param[in] skb – pointer to socket buffer structure.
 * \param[in] len – length of packet to be transmitted/received.
 * \param[in] tx_rx – packet to be transmitted or received.
 * \param[in] desc_idx – descriptor index to be used for transmission or
 *			reception of packet.
 *
 * \return void
 */

void print_pkt(struct sk_buff *skb, int len, bool tx_rx, int desc_idx)
{
	int i, j = 0;
	unsigned char *buf = skb->data;

	pr_err
	    ("\n\n/***********************************************************/\n");

	pr_err("%s pkt of %d Bytes [DESC index = %d]\n\n",
	       (tx_rx ? "TX" : "RX"), len, desc_idx);
	pr_err("Dst MAC addr(6 bytes)\n");
	for (i = 0; i < 6; i++)
		printk("%#.2x%s", buf[i], (((i == 5) ? "" : ":")));
	pr_err("\nSrc MAC addr(6 bytes)\n");
	for (i = 6; i <= 11; i++)
		printk("%#.2x%s", buf[i], (((i == 11) ? "" : ":")));
	i = (buf[12] << 8 | buf[13]);
	pr_err("\nType/Length(2 bytes)\n%#x", i);

	pr_err("\nPay Load : %d bytes\n", (len - 14));
	for (i = 14, j = 1; i < len; i++, j++) {
		printk("%#.2x%s", buf[i], (((i == (len - 1)) ? "" : ":")));
		if ((j % 16) == 0)
			pr_err("");
	}

	pr_err
	    ("/*************************************************************/\n\n");
}

/*!
 * \details This function is invoked by probe function. This function will
 * initialize default receive coalesce parameters and sw timer value and store
 * it in respective receive data structure.
 *
 * \param[in] pdata – pointer to private data structure.
 *
 * \return void
 */

void eqos_init_rx_coalesce(struct eqos_prv_data *pdata)
{
	struct rx_ring *prx_ring = NULL;
	UINT i;

	pr_debug("-->eqos_init_rx_coalesce\n");

	/* If RX coalescing parameters are not set in DT, set to default */
	for (i = 0; i < EQOS_RX_QUEUE_CNT; i++) {
		prx_ring = GET_RX_WRAPPER_DESC(i);
		if (prx_ring->use_riwt == EQOS_COAELSCING_DISABLE) {
			prx_ring->use_riwt = EQOS_COAELSCING_DISABLE;
			prx_ring->rx_riwt =
				eqos_usec2riwt(EQOS_OPTIMAL_DMA_RIWT_USEC,
					       pdata);
			prx_ring->rx_coal_frames = EQOS_RX_MAX_FRAMES;
		}
	}

	pr_debug("<--eqos_init_rx_coalesce\n");
}

/*!
 * \details This function is invoked by open() function. This function will
 * clear MMC structure.
 *
 * \param[in] pdata – pointer to private data structure.
 *
 * \return void
 */

static void eqos_mmc_setup(struct eqos_prv_data *pdata)
{
	pr_debug("-->eqos_mmc_setup\n");

	if (pdata->hw_feat.mmc_sel) {
		memset(&pdata->mmc, 0, sizeof(struct eqos_mmc_counters));
	} else
		pr_err("No MMC/RMON module available in the HW\n");

	pr_debug("<--eqos_mmc_setup\n");
}

inline unsigned int eqos_reg_read(volatile ULONG *ptr)
{
	return ioread32((void *)ptr);
}

/*!
 * \details This function is invoked by ethtool function when user wants to
 * read MMC counters. This function will read the MMC if supported by core
 * and store it in eqos_mmc_counters structure. By default all the
 * MMC are programmed "read on reset" hence all the fields of the
 * eqos_mmc_counters are incremented.
 *
 * open() function. This function will
 * initialize MMC control register ie it disable all MMC interrupt and all
 * MMC register are configured to clear on read.
 *
 * \param[in] pdata – pointer to private data structure.
 *
 * \return void
 */

void eqos_mmc_read(struct eqos_mmc_counters *mmc)
{
	pr_debug("-->eqos_mmc_read\n");

	/* MMC TX counter registers */
	mmc->mmc_tx_octetcount_gb += eqos_reg_read(MMC_TXOCTETCOUNT_GB_OFFSET);
	mmc->mmc_tx_framecount_gb += eqos_reg_read(MMC_TXPACKETCOUNT_GB_OFFSET);
	mmc->mmc_tx_broadcastframe_g +=
	    eqos_reg_read(MMC_TXBROADCASTPACKETS_G_OFFSET);
	mmc->mmc_tx_multicastframe_g +=
	    eqos_reg_read(MMC_TXMULTICASTPACKETS_G_OFFSET);
	mmc->mmc_tx_64_octets_gb += eqos_reg_read(MMC_TX64OCTETS_GB_OFFSET);
	mmc->mmc_tx_65_to_127_octets_gb +=
	    eqos_reg_read(MMC_TX65TO127OCTETS_GB_OFFSET);
	mmc->mmc_tx_128_to_255_octets_gb +=
	    eqos_reg_read(MMC_TX128TO255OCTETS_GB_OFFSET);
	mmc->mmc_tx_256_to_511_octets_gb +=
	    eqos_reg_read(MMC_TX256TO511OCTETS_GB_OFFSET);
	mmc->mmc_tx_512_to_1023_octets_gb +=
	    eqos_reg_read(MMC_TX512TO1023OCTETS_GB_OFFSET);
	mmc->mmc_tx_1024_to_max_octets_gb +=
	    eqos_reg_read(MMC_TX1024TOMAXOCTETS_GB_OFFSET);
	mmc->mmc_tx_unicast_gb += eqos_reg_read(MMC_TXUNICASTPACKETS_GB_OFFSET);
	mmc->mmc_tx_multicast_gb +=
	    eqos_reg_read(MMC_TXMULTICASTPACKETS_GB_OFFSET);
	mmc->mmc_tx_broadcast_gb +=
	    eqos_reg_read(MMC_TXBROADCASTPACKETS_GB_OFFSET);
	mmc->mmc_tx_underflow_error +=
	    eqos_reg_read(MMC_TXUNDERFLOWERROR_OFFSET);
	mmc->mmc_tx_singlecol_g += eqos_reg_read(MMC_TXSINGLECOL_G_OFFSET);
	mmc->mmc_tx_multicol_g += eqos_reg_read(MMC_TXMULTICOL_G_OFFSET);
	mmc->mmc_tx_deferred += eqos_reg_read(MMC_TXDEFERRED_OFFSET);
	mmc->mmc_tx_latecol += eqos_reg_read(MMC_TXLATECOL_OFFSET);
	mmc->mmc_tx_exesscol += eqos_reg_read(MMC_TXEXESSCOL_OFFSET);
	mmc->mmc_tx_carrier_error += eqos_reg_read(MMC_TXCARRIERERROR_OFFSET);
	mmc->mmc_tx_octetcount_g += eqos_reg_read(MMC_TXOCTETCOUNT_G_OFFSET);
	mmc->mmc_tx_framecount_g += eqos_reg_read(MMC_TXPACKETSCOUNT_G_OFFSET);
	mmc->mmc_tx_excessdef += eqos_reg_read(MMC_TXEXCESSDEF_OFFSET);
	mmc->mmc_tx_pause_frame += eqos_reg_read(MMC_TXPAUSEPACKETS_OFFSET);
	mmc->mmc_tx_vlan_frame_g += eqos_reg_read(MMC_TXVLANPACKETS_G_OFFSET);
	mmc->mmc_tx_osize_frame_g += eqos_reg_read(MMC_TXOVERSIZE_G_OFFSET);

	/* MMC RX counter registers */
	mmc->mmc_rx_framecount_gb += eqos_reg_read(MMC_RXPACKETCOUNT_GB_OFFSET);
	mmc->mmc_rx_octetcount_gb += eqos_reg_read(MMC_RXOCTETCOUNT_GB_OFFSET);
	mmc->mmc_rx_octetcount_g += eqos_reg_read(MMC_RXOCTETCOUNT_G_OFFSET);
	mmc->mmc_rx_broadcastframe_g +=
	    eqos_reg_read(MMC_RXBROADCASTPACKETS_G_OFFSET);
	mmc->mmc_rx_multicastframe_g +=
	    eqos_reg_read(MMC_RXMULTICASTPACKETS_G_OFFSET);
	mmc->mmc_rx_crc_error += eqos_reg_read(MMC_RXCRCERROR_OFFSET);
	mmc->mmc_rx_align_error += eqos_reg_read(MMC_RXALIGNMENTERROR_OFFSET);
	mmc->mmc_rx_run_error += eqos_reg_read(MMC_RXRUNTERROR_OFFSET);
	mmc->mmc_rx_jabber_error += eqos_reg_read(MMC_RXJABBERERROR_OFFSET);
	mmc->mmc_rx_undersize_g += eqos_reg_read(MMC_RXUNDERSIZE_G_OFFSET);
	mmc->mmc_rx_oversize_g += eqos_reg_read(MMC_RXOVERSIZE_G_OFFSET);
	mmc->mmc_rx_64_octets_gb += eqos_reg_read(MMC_RX64OCTETS_GB_OFFSET);
	mmc->mmc_rx_65_to_127_octets_gb +=
	    eqos_reg_read(MMC_RX65TO127OCTETS_GB_OFFSET);
	mmc->mmc_rx_128_to_255_octets_gb +=
	    eqos_reg_read(MMC_RX128TO255OCTETS_GB_OFFSET);
	mmc->mmc_rx_256_to_511_octets_gb +=
	    eqos_reg_read(MMC_RX256TO511OCTETS_GB_OFFSET);
	mmc->mmc_rx_512_to_1023_octets_gb +=
	    eqos_reg_read(MMC_RX512TO1023OCTETS_GB_OFFSET);
	mmc->mmc_rx_1024_to_max_octets_gb +=
	    eqos_reg_read(MMC_RX1024TOMAXOCTETS_GB_OFFSET);
	mmc->mmc_rx_unicast_g += eqos_reg_read(MMC_RXUNICASTPACKETS_G_OFFSET);
	mmc->mmc_rx_length_error += eqos_reg_read(MMC_RXLENGTHERROR_OFFSET);
	mmc->mmc_rx_outofrangetype +=
	    eqos_reg_read(MMC_RXOUTOFRANGETYPE_OFFSET);
	mmc->mmc_rx_pause_frames += eqos_reg_read(MMC_RXPAUSEPACKETS_OFFSET);
	mmc->mmc_rx_fifo_overflow += eqos_reg_read(MMC_RXFIFOOVERFLOW_OFFSET);
	mmc->mmc_rx_vlan_frames_gb +=
	    eqos_reg_read(MMC_RXVLANPACKETS_GB_OFFSET);
	mmc->mmc_rx_watchdog_error += eqos_reg_read(MMC_RXWATCHDOGERROR_OFFSET);
	mmc->mmc_rx_receive_error += eqos_reg_read(MMC_RXRCVERROR_OFFSET);
	mmc->mmc_rx_ctrl_frames_g += eqos_reg_read(MMC_RXCTRLPACKETS_G_OFFSET);

	/* IPC */
	mmc->mmc_rx_ipc_intr_mask += eqos_reg_read(MMC_IPC_INTR_MASK_RX_OFFSET);
	mmc->mmc_rx_ipc_intr += eqos_reg_read(MMC_IPC_INTR_RX_OFFSET);

	/* IPv4 */
	mmc->mmc_rx_ipv4_gd += eqos_reg_read(MMC_RXIPV4_GD_PKTS_OFFSET);
	mmc->mmc_rx_ipv4_hderr += eqos_reg_read(MMC_RXIPV4_HDRERR_PKTS_OFFSET);
	mmc->mmc_rx_ipv4_nopay += eqos_reg_read(MMC_RXIPV4_NOPAY_PKTS_OFFSET);
	mmc->mmc_rx_ipv4_frag += eqos_reg_read(MMC_RXIPV4_FRAG_PKTS_OFFSET);
	mmc->mmc_rx_ipv4_udsbl += eqos_reg_read(MMC_RXIPV4_UBSBL_PKTS_OFFSET);

	/* IPV6 */
	mmc->mmc_rx_ipv6_gd += eqos_reg_read(MMC_RXIPV6_GD_PKTS_OFFSET);
	mmc->mmc_rx_ipv6_hderr += eqos_reg_read(MMC_RXIPV6_HDRERR_PKTS_OFFSET);
	mmc->mmc_rx_ipv6_nopay += eqos_reg_read(MMC_RXIPV6_NOPAY_PKTS_OFFSET);

	/* Protocols */
	mmc->mmc_rx_udp_gd += eqos_reg_read(MMC_RXUDP_GD_PKTS_OFFSET);
	mmc->mmc_rx_udp_err += eqos_reg_read(MMC_RXUDP_ERR_PKTS_OFFSET);
	mmc->mmc_rx_tcp_gd += eqos_reg_read(MMC_RXTCP_GD_PKTS_OFFSET);
	mmc->mmc_rx_tcp_err += eqos_reg_read(MMC_RXTCP_ERR_PKTS_OFFSET);
	mmc->mmc_rx_icmp_gd += eqos_reg_read(MMC_RXICMP_GD_PKTS_OFFSET);
	mmc->mmc_rx_icmp_err += eqos_reg_read(MMC_RXICMP_ERR_PKTS_OFFSET);

	/* IPv4 */
	mmc->mmc_rx_ipv4_gd_octets +=
	    eqos_reg_read(MMC_RXIPV4_GD_OCTETS_OFFSET);
	mmc->mmc_rx_ipv4_hderr_octets +=
	    eqos_reg_read(MMC_RXIPV4_HDRERR_OCTETS_OFFSET);
	mmc->mmc_rx_ipv4_nopay_octets +=
	    eqos_reg_read(MMC_RXIPV4_NOPAY_OCTETS_OFFSET);
	mmc->mmc_rx_ipv4_frag_octets +=
	    eqos_reg_read(MMC_RXIPV4_FRAG_OCTETS_OFFSET);
	mmc->mmc_rx_ipv4_udsbl_octets +=
	    eqos_reg_read(MMC_RXIPV4_UDSBL_OCTETS_OFFSET);

	/* IPV6 */
	mmc->mmc_rx_ipv6_gd_octets +=
	    eqos_reg_read(MMC_RXIPV6_GD_OCTETS_OFFSET);
	mmc->mmc_rx_ipv6_hderr_octets +=
	    eqos_reg_read(MMC_RXIPV6_HDRERR_OCTETS_OFFSET);
	mmc->mmc_rx_ipv6_nopay_octets +=
	    eqos_reg_read(MMC_RXIPV6_NOPAY_OCTETS_OFFSET);

	/* Protocols */
	mmc->mmc_rx_udp_gd_octets += eqos_reg_read(MMC_RXUDP_GD_OCTETS_OFFSET);
	mmc->mmc_rx_udp_err_octets +=
	    eqos_reg_read(MMC_RXUDP_ERR_OCTETS_OFFSET);
	mmc->mmc_rx_tcp_gd_octets += eqos_reg_read(MMC_RXTCP_GD_OCTETS_OFFSET);
	mmc->mmc_rx_tcp_err_octets +=
	    eqos_reg_read(MMC_RXTCP_ERR_OCTETS_OFFSET);
	mmc->mmc_rx_icmp_gd_octets +=
	    eqos_reg_read(MMC_RXICMP_GD_OCTETS_OFFSET);
	mmc->mmc_rx_icmp_err_octets +=
	    eqos_reg_read(MMC_RXICMP_ERR_OCTETS_OFFSET);

	pr_debug("<--eqos_mmc_read\n");
}

static const struct net_device_ops eqos_netdev_ops = {
	.ndo_open = eqos_open,
	.ndo_stop = eqos_close,
	.ndo_start_xmit = eqos_start_xmit,
	.ndo_get_stats = eqos_get_stats,
	.ndo_set_rx_mode = eqos_set_rx_mode,
	.ndo_set_features = eqos_set_features,
	.ndo_do_ioctl = eqos_ioctl,
	.ndo_change_mtu = eqos_change_mtu,
#ifdef EQOS_QUEUE_SELECT_ALGO
	.ndo_select_queue = eqos_select_queue,
#endif
	.ndo_vlan_rx_add_vid = eqos_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid = eqos_vlan_rx_kill_vid,
	.ndo_set_mac_address = eth_mac_addr,
};

struct net_device_ops *eqos_get_netdev_ops(void)
{
	return (struct net_device_ops *)&eqos_netdev_ops;
}


static void eqos_disable_all_irqs(struct eqos_prv_data *pdata)
{
	int i;

	pr_debug("-->%s()\n", __func__);

	for (i = 0; i < pdata->num_chans; i++) {
		eqos_disable_chan_rx_interrupt(pdata, i);
		eqos_disable_chan_tx_interrupt(pdata, i);
	}

	/* disable mac interrupts */
	MAC_IMR_WR(0);

	/* ensure irqs are not executing */
	synchronize_irq(pdata->common_irq);
	for (i = 0; i < pdata->num_chans; i++) {
		if (pdata->rx_irq_alloc_mask & (1 << i))
			synchronize_irq(pdata->rx_irqs[i]);
		if (pdata->tx_irq_alloc_mask & (1 << i))
			synchronize_irq(pdata->tx_irqs[i]);
	}

	pr_debug("<--%s()\n", __func__);
}

void eqos_stop_dev(struct eqos_prv_data *pdata)
{
	struct hw_if_struct *hw_if = &pdata->hw_if;
	struct desc_if_struct *desc_if = &pdata->desc_if;

	pr_debug("-->%s()\n", __func__);

#ifdef CONFIG_TEGRA_PTP_NOTIFIER
	/* Unregister broadcasting MAC timestamp to clients */
	tegra_unregister_hwtime_source();
#endif
	/* turn off sources of data into dev */
	netif_tx_disable(pdata->dev);

	hw_if->stop_mac_rx();
	eqos_disable_all_irqs(pdata);
	eqos_all_ch_napi_disable(pdata);

	/* stop DMA TX */
	eqos_stop_all_ch_tx_dma(pdata);

	/* disable MAC TX */
	hw_if->stop_mac_tx();

	/* stop DMA RX */
	eqos_stop_all_ch_rx_dma(pdata);

	if (pdata->eee_active)
		del_timer_sync(&pdata->eee_ctrl_timer);

	/* return tx skbs */
	desc_if->tx_skb_free_mem(pdata, pdata->num_chans);

	/* free rx skb's */
	desc_if->rx_skb_free_mem(pdata, pdata->num_chans);

	pr_debug("<--%s()\n", __func__);
}

void eqos_start_dev(struct eqos_prv_data *pdata)
{
	struct hw_if_struct *hw_if = &pdata->hw_if;
	struct desc_if_struct *desc_if = &pdata->desc_if;

	pr_debug("-->%s()\n", __func__);

	hw_if->pad_calibrate(pdata);

	/* default configuration */
	eqos_default_common_confs(pdata);
	eqos_default_tx_confs(pdata);
	eqos_default_rx_confs(pdata);

	desc_if->wrapper_tx_desc_init(pdata);
	desc_if->wrapper_rx_desc_init(pdata);

	eqos_napi_enable_mq(pdata);

	eqos_set_rx_mode(pdata->dev);
	eqos_mmc_setup(pdata);

	/* initializes MAC and DMA */
	hw_if->init(pdata);

	MAC_1US_TIC_WR(pdata->csr_clock_speed - 1);

	if (pdata->hw_feat.pcs_sel)
		hw_if->control_an(1, 0);

#ifdef EQOS_ENABLE_EEE
	pdata->eee_enabled = eqos_eee_init(pdata);
#else
	pdata->eee_enabled = false;
#endif

	pr_debug("<--%s()\n", __func__);
}

void eqos_iso_work(struct work_struct *work)
{
	struct eqos_prv_data *pdata =
	    container_of(work, struct eqos_prv_data, iso_work);
	struct phy_device *phydev = pdata->phydev;
	struct eqos_cfg *pdt_cfg = (struct eqos_cfg *)&pdata->dt_cfg;
	int ret;
	uint iso_bw;

	pr_debug("-->%s()\n", __func__);

	if (pdt_cfg->eth_iso_enable) {
		if (phydev->link)
			iso_bw = pdata->dt_cfg.iso_bw;
		else
			iso_bw = 0;

		ret = tegra_isomgr_reserve(pdata->isomgr_handle, iso_bw, 0);
		if (!ret) {
			dev_err(&pdata->pdev->dev,
				"EQOS ISO BW %d reservation failed with %d\n",
				iso_bw, ret);
			return;
		}

		ret = tegra_isomgr_realize(pdata->isomgr_handle);
		if (!ret)
			dev_err(&pdata->pdev->dev,
				"EQOS ISO BW realize failed with %d\n", ret);
	}

	pr_debug("<--%s()\n", __func__);
}
void eqos_fbe_work(struct work_struct *work)
{
	struct eqos_prv_data *pdata =
	    container_of(work, struct eqos_prv_data, fbe_work);
	int i;
	u32 dma_sr_reg;

	pr_debug("-->%s()\n", __func__);

	mutex_lock(&pdata->hw_change_lock);
	if (pdata->hw_stopped)
		goto out;

	i = 0;
	while (pdata->fbe_chan_mask) {
		if (pdata->fbe_chan_mask & 1) {
			DMA_SR_RD(i, dma_sr_reg);

			dev_err(&pdata->pdev->dev,
				"Fatal Bus Error on chan %d, SRreg=0x%.8x\n",
				i, dma_sr_reg);
		}
		pdata->fbe_chan_mask >>= 1;
		i++;
	}
	eqos_stop_dev(pdata);
	eqos_start_dev(pdata);
out:
	mutex_unlock(&pdata->hw_change_lock);

	pr_debug("<--%s()\n", __func__);
}
