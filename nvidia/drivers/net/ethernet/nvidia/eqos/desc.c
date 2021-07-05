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
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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
/*!@file: eqos_desc.c
 * @brief: Driver functions.
 */
#include "yheader.h"
#include "desc.h"
extern ULONG eqos_base_addr;
#include "yregacc.h"

/*!
* \brief API to free the transmit descriptor memory.
*
* \details This function is used to free the transmit descriptor memory.
*
* \param[in] pdata - pointer to private data structure.
*
* \retval void.
*/

static void eqos_tx_desc_free_mem(struct eqos_prv_data *pdata,
					 UINT tx_qcnt)
{
	struct tx_ring *ptx_ring = NULL;
	UINT qinx;
	uint tx_ring_size = sizeof(struct s_tx_desc) * TX_DESC_CNT;

	pr_debug("-->eqos_tx_desc_free_mem: tx_qcnt = %d\n", tx_qcnt);

	for (qinx = 0; qinx < tx_qcnt; qinx++) {
		ptx_ring = GET_TX_WRAPPER_DESC(qinx);

		if (GET_TX_DESC_PTR(qinx, 0)) {
			dma_free_coherent(&pdata->pdev->dev,
					  tx_ring_size,
					  GET_TX_DESC_PTR(qinx, 0),
					  GET_TX_DESC_DMA_ADDR(qinx, 0));
			GET_TX_DESC_PTR(qinx, 0) = NULL;
		}
	}
#ifdef DO_TX_ALIGN_TEST
	if (pdata->ptst_buf)
		dma_free_coherent(&pdata->pdev->dev, pdata->tst_buf_size,
		pdata->ptst_buf, (dma_addr_t)pdata->tst_buf_dma_addr);
#endif

	pr_debug("<--eqos_tx_desc_free_mem\n");
}

/*!
* \brief API to free the receive descriptor memory.
*
* \details This function is used to free the receive descriptor memory.
*
* \param[in] pdata - pointer to private data structure.
*
* \retval void.
*/

static void eqos_rx_desc_free_mem(struct eqos_prv_data *pdata,
					 UINT rx_qcnt)
{
	struct rx_ring *prx_ring = NULL;
	UINT qinx = 0;
	uint rx_ring_size = sizeof(struct s_rx_desc) * RX_DESC_CNT;

	pr_debug("-->eqos_rx_desc_free_mem: rx_qcnt = %d\n", rx_qcnt);

	for (qinx = 0; qinx < rx_qcnt; qinx++) {
		prx_ring = GET_RX_WRAPPER_DESC(qinx);

		if (GET_RX_DESC_PTR(qinx, 0)) {
			dma_free_coherent(&pdata->pdev->dev,
					  rx_ring_size,
					  GET_RX_DESC_PTR(qinx, 0),
					  GET_RX_DESC_DMA_ADDR(qinx, 0));
			GET_RX_DESC_PTR(qinx, 0) = NULL;
		}
	}

	pr_debug("<--eqos_rx_desc_free_mem\n");
}

/*!
* \brief API to alloc the queue memory.
*
* \details This function allocates the queue structure memory.
*
* \param[in] pdata - pointer to private data structure.
*
* \return integer
*
* \retval 0 on success & -ve number on failure.
*/

static int eqos_alloc_queue_struct(struct eqos_prv_data *pdata)
{
	int ret = 0;

	pr_debug("-->eqos_alloc_queue_struct: number of channels = %d\n",
		 pdata->num_chans);

	pdata->tx_queue =
		kzalloc(sizeof(struct eqos_tx_queue) * pdata->num_chans,
			GFP_KERNEL);
	if (pdata->tx_queue == NULL) {
		pr_err("ERROR: Unable to allocate Tx queue structure\n");
		ret = -ENOMEM;
		goto err_out_tx_q_alloc_failed;
	}

	pdata->rx_queue =
		kzalloc(sizeof(struct eqos_rx_queue) * pdata->num_chans,
			GFP_KERNEL);
	if (pdata->rx_queue == NULL) {
		pr_err("ERROR: Unable to allocate Rx queue structure\n");
		ret = -ENOMEM;
		goto err_out_rx_q_alloc_failed;
	}

	pr_debug("<--eqos_alloc_queue_struct\n");

	return ret;

err_out_rx_q_alloc_failed:
	kfree(pdata->tx_queue);

err_out_tx_q_alloc_failed:
	return ret;
}


/*!
* \brief API to free the queue memory.
*
* \details This function free the queue structure memory.
*
* \param[in] pdata - pointer to private data structure.
*
* \return void
*/

static void eqos_free_queue_struct(struct eqos_prv_data *pdata)
{
	pr_debug("-->eqos_free_queue_struct\n");

	if (pdata->tx_queue != NULL) {
		kfree(pdata->tx_queue);
		pdata->tx_queue = NULL;
	}

	if (pdata->rx_queue != NULL) {
		kfree(pdata->rx_queue);
		pdata->rx_queue = NULL;
	}

	pr_debug("<--eqos_free_queue_struct\n");
}

/*!
* \brief API to free the memory for descriptor & buffers.
*
* \details This function is used to free the memory for device
* descriptors & buffers
* which are used by device for data transmission & reception.
*
* \param[in] pdata - pointer to private data structure.
*
* \return void
*
*/
static void free_buffer_and_desc(struct eqos_prv_data *pdata)
{
	eqos_tx_free_mem(pdata);
	eqos_rx_free_mem(pdata);
}

/*!
* \brief API to allocate the memory for descriptor & buffers.
*
* \details This function is used to allocate the memory for device
* descriptors & buffers
* which are used by device for data transmission & reception.
*
* \param[in] pdata - pointer to private data structure.
*
* \return integer
*
* \retval 0 on success & -ENOMEM number on failure.
*/

static INT allocate_buffer_and_desc(struct eqos_prv_data *pdata)
{
	INT ret = 0;
	UINT qinx;
	uint tx_ring_size = sizeof(struct s_tx_desc) * TX_DESC_CNT;
	uint rx_ring_size = sizeof(struct s_rx_desc) * RX_DESC_CNT;
	uint tx_swcx_size = sizeof(struct tx_swcx_desc) * TX_DESC_CNT;
	uint rx_swcx_size = sizeof(struct rx_swcx_desc) * RX_DESC_CNT;

	pr_debug("-->allocate_buffer_and_desc: TX_QUEUE_CNT = %d, "\
		"RX_QUEUE_CNT = %d\n", EQOS_TX_QUEUE_CNT,
		EQOS_RX_QUEUE_CNT);

	/* Allocate descriptors and buffers memory for all TX queues */
	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		/* TX descriptors */
		GET_TX_DESC_PTR(qinx, 0) =
			dma_alloc_coherent(&pdata->pdev->dev,
					   tx_ring_size,
					   &(GET_TX_DESC_DMA_ADDR(qinx, 0)),
					   GFP_KERNEL);
		if (GET_TX_DESC_PTR(qinx, 0) == NULL) {
			ret = -ENOMEM;
			goto err_out_tx_desc;
		}
		/* Check if address is greater than 32 bit */
		BUG_ON((uint64_t)GET_TX_DESC_DMA_ADDR(qinx, 0) >> 32);
	}

	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		/* TX wrapper buffer */
		GET_TX_BUF_PTR(qinx, 0) = kzalloc(tx_swcx_size, GFP_KERNEL);
		if (GET_TX_BUF_PTR(qinx, 0) == NULL) {
			ret = -ENOMEM;
			goto err_out_tx_buf;
		}
	}

	/* Allocate descriptors and buffers memory for all RX queues */
	for (qinx = 0; qinx < EQOS_RX_QUEUE_CNT; qinx++) {
		/* RX descriptors */
		GET_RX_DESC_PTR(qinx, 0) =
			dma_alloc_coherent(&pdata->pdev->dev,
					   rx_ring_size,
					   &(GET_RX_DESC_DMA_ADDR(qinx, 0)),
					   GFP_KERNEL);
		if (GET_RX_DESC_PTR(qinx, 0) == NULL) {
			ret = -ENOMEM;
			goto rx_alloc_failure;
		}
		/* Check if address is greater than 32 bit */
		BUG_ON((uint64_t)GET_RX_DESC_DMA_ADDR(qinx, 0) >> 32);
	}

	for (qinx = 0; qinx < EQOS_RX_QUEUE_CNT; qinx++) {
		/* RX wrapper buffer */
		GET_RX_BUF_PTR(qinx, 0) = kzalloc(rx_swcx_size, GFP_KERNEL);
		if (GET_RX_BUF_PTR(qinx, 0) == NULL) {
			ret = -ENOMEM;
			goto err_out_rx_buf;
		}
	}
#ifdef DO_TX_ALIGN_TEST
	{
		UINT cnt;

		pdata->tst_buf_size = 2048;
		pdata->ptst_buf = dma_alloc_coherent(&pdata->pdev->dev,
			pdata->tst_buf_size,
			(dma_addr_t *)&pdata->tst_buf_dma_addr,
			GFP_KERNEL);

		for (cnt = 0; cnt < 2048; cnt++)
			pdata->ptst_buf[cnt] = cnt;
	}
#endif

	pr_debug("<--allocate_buffer_and_desc\n");

	return ret;

 err_out_rx_buf:
	eqos_rx_buf_free_mem(pdata, qinx);
	qinx = EQOS_RX_QUEUE_CNT;

 rx_alloc_failure:
	eqos_rx_desc_free_mem(pdata, qinx);
	qinx = EQOS_TX_QUEUE_CNT;

 err_out_tx_buf:
	eqos_tx_buf_free_mem(pdata, qinx);
	qinx = EQOS_TX_QUEUE_CNT;

 err_out_tx_desc:
	eqos_tx_desc_free_mem(pdata, qinx);

	return ret;
}

/*!
* \brief API to initialize the transmit descriptors.
*
* \details This function is used to initialize transmit descriptors.
* Each descriptors are assigned a buffer. The base/starting address
* of the descriptors is updated in device register if required & all
* the private data structure variables related to transmit
* descriptor handling are updated in this function.
*
* \param[in] pdata - pointer to private data structure.
*
* \return void.
*/

static void eqos_wrapper_tx_descriptor_init_single_q(
			struct eqos_prv_data *pdata,
			UINT qinx)
{
	int i;
	struct tx_ring *ptx_ring =
	    GET_TX_WRAPPER_DESC(qinx);
	struct tx_swcx_desc *ptx_swcx_desc = GET_TX_BUF_PTR(qinx, 0);
	struct s_tx_desc *desc = GET_TX_DESC_PTR(qinx, 0);
	dma_addr_t desc_dma = GET_TX_DESC_DMA_ADDR(qinx, 0);
	struct hw_if_struct *hw_if = &(pdata->hw_if);

	pr_debug("-->eqos_wrapper_tx_descriptor_init_single_q: "\
		"qinx = %u\n", qinx);

	for (i = 0; i < TX_DESC_CNT; i++) {
		GET_TX_DESC_PTR(qinx, i) = &desc[i];
		GET_TX_DESC_DMA_ADDR(qinx, i) =
		    (desc_dma + sizeof(struct s_tx_desc) * i);
		GET_TX_BUF_PTR(qinx, i) = &ptx_swcx_desc[i];
	}

	ptx_ring->cur_tx = 0;
	ptx_ring->dirty_tx = 0;

	hw_if->tx_desc_init(pdata, qinx);
	ptx_ring->cur_tx = 0;

	pr_debug("<--eqos_wrapper_tx_descriptor_init_single_q\n");
}

static int desc_alloc_skb(struct eqos_prv_data *pdata,
			  struct rx_swcx_desc *prx_swcx_desc, gfp_t gfp,
			  unsigned int qinx)
{
	struct sk_buff *skb = prx_swcx_desc->skb;
	dma_addr_t dma = prx_swcx_desc->dma;

	if (skb) {
		/* Recycled skbs should have zero length and their buffer
		 * still DMA mapped.
		 */
		if (WARN_ON(skb->len != 0))
			skb_trim(skb, 0);
		if (WARN_ON(!dma))
			goto dma_map;

		return 0;
	}

	skb = __netdev_alloc_skb_ip_align(pdata->dev, pdata->rx_buffer_len,
					  gfp);
	if (unlikely(!skb)) {
		netdev_err(pdata->dev, "RX skb allocation failed, using reserved buffer\n");
		prx_swcx_desc->skb = pdata->resv_skb;
		prx_swcx_desc->dma = pdata->resv_dma;
		pdata->xstats.q_re_alloc_rx_buf_failed[qinx]++;
		return 0;
	}

dma_map:
	dma = dma_map_single(&pdata->pdev->dev, skb->data, pdata->rx_buffer_len,
			     DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(&pdata->pdev->dev, dma))) {
		netdev_err(pdata->dev, "RX skb dma map failed\n");
		dev_kfree_skb_any(skb);
		return -ENOMEM;
	}

	prx_swcx_desc->skb = skb;
	prx_swcx_desc->dma = dma;

	return 0;
}

/*!
* \brief API to initialize the receive descriptors.
*
* \details This function is used to initialize receive descriptors.
* skb buffer is allocated & assigned for each descriptors. The base/starting
* address of the descriptors is updated in device register if required and
* all the private data structure variables related to receive descriptor
* handling are updated in this function.
*
* \param[in] pdata - pointer to private data structure.
*
* \return void.
*/

static void eqos_wrapper_rx_descriptor_init_single_q(
			struct eqos_prv_data *pdata,
			UINT qinx)
{
	struct rx_ring *prx_ring =
	    GET_RX_WRAPPER_DESC(qinx);
	struct rx_swcx_desc *prx_swcx_desc = GET_RX_BUF_PTR(qinx, 0);
	struct s_rx_desc *desc = GET_RX_DESC_PTR(qinx, 0);
	dma_addr_t desc_dma = GET_RX_DESC_DMA_ADDR(qinx, 0);
	struct hw_if_struct *hw_if = &(pdata->hw_if);
	int i, ret;

	pr_debug("-->eqos_wrapper_rx_descriptor_init_single_q: "\
		"qinx = %u\n", qinx);

	memset(prx_swcx_desc, 0, (sizeof(struct rx_swcx_desc) * RX_DESC_CNT));

	for (i = 0; i < RX_DESC_CNT; i++) {
		GET_RX_DESC_PTR(qinx, i) = &desc[i];
		GET_RX_DESC_DMA_ADDR(qinx, i) =
		    (desc_dma + sizeof(struct s_rx_desc) * i);
		GET_RX_BUF_PTR(qinx, i) = &prx_swcx_desc[i];

		ret = desc_alloc_skb(pdata, GET_RX_BUF_PTR(qinx, i),
				     GFP_KERNEL, qinx);
		if (ret < 0)
			break;
	}

	prx_ring->cur_rx = 0;
	prx_ring->dirty_rx = 0;
	prx_ring->skb_realloc_threshold = MIN_RX_DESC_CNT;

	hw_if->rx_desc_init(pdata, qinx);
	prx_ring->cur_rx = 0;

	pr_debug("<--eqos_wrapper_rx_descriptor_init_single_q\n");
}

static void eqos_wrapper_tx_descriptor_init(struct eqos_prv_data
						   *pdata)
{
	UINT qinx;

	pr_debug("-->eqos_wrapper_tx_descriptor_init\n");

	for (qinx = 0; qinx < EQOS_TX_QUEUE_CNT; qinx++) {
		eqos_wrapper_tx_descriptor_init_single_q(pdata, qinx);
	}

	pr_debug("<--eqos_wrapper_tx_descriptor_init\n");
}

static void eqos_wrapper_rx_descriptor_init(struct eqos_prv_data *pdata)
{
	unsigned int qinx;

	pr_debug("-->eqos_wrapper_rx_descriptor_init\n");
	pdata->resv_skb = __netdev_alloc_skb_ip_align(pdata->dev,
						      pdata->rx_buffer_len,
						      GFP_KERNEL);
	if (unlikely(!pdata->resv_skb))
		netdev_err(pdata->dev, "Reserved RX skb allocation failed\n");

	pdata->resv_dma = dma_map_single(&pdata->pdev->dev,
					 pdata->resv_skb->data,
					 pdata->rx_buffer_len,
					 DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(&pdata->pdev->dev, pdata->resv_dma))) {
		netdev_err(pdata->dev, "Reserved RX skb dma map failed\n");
		dev_kfree_skb_any(pdata->resv_skb);
		pdata->resv_skb = NULL;
		pdata->resv_dma = 0;
	}

	for (qinx = 0; qinx < EQOS_RX_QUEUE_CNT; qinx++)
		eqos_wrapper_rx_descriptor_init_single_q(pdata, qinx);

	pr_debug("<--eqos_wrapper_rx_descriptor_init\n");
}

/*!
* \brief API to free the receive descriptor & buffer memory.
*
* \details This function is used to free the receive descriptor & buffer memory.
*
* \param[in] pdata - pointer to private data structure.
*
* \retval void.
*/

static void eqos_rx_free_mem(struct eqos_prv_data *pdata)
{
	pr_debug("-->eqos_rx_free_mem\n");

	/* free RX descriptor */
	eqos_rx_desc_free_mem(pdata, EQOS_RX_QUEUE_CNT);

	/* free RX wrapper buffer */
	eqos_rx_buf_free_mem(pdata, EQOS_RX_QUEUE_CNT);

	pr_debug("<--eqos_rx_free_mem\n");
}

/*!
* \brief API to free the transmit descriptor & buffer memory.
*
* \details This function is used to free the transmit descriptor
* & buffer memory.
*
* \param[in] pdata - pointer to private data structure.
*
* \retval void.
*/

static void eqos_tx_free_mem(struct eqos_prv_data *pdata)
{
	pr_debug("-->eqos_tx_free_mem\n");

	/* free TX descriptor */
	eqos_tx_desc_free_mem(pdata, EQOS_TX_QUEUE_CNT);

	/* free TX buffer */
	eqos_tx_buf_free_mem(pdata, EQOS_TX_QUEUE_CNT);

	pr_debug("<--eqos_tx_free_mem\n");
}

/*!
 * \details This function is invoked by other function to free
 * the tx socket buffers.
 *
 * \param[in] pdata – pointer to private data structure.
 *
 * \return void
 */

static void eqos_tx_skb_free_mem_single_q(struct eqos_prv_data *pdata,
							UINT qinx)
{
	struct tx_ring *ptx_ring =
	    GET_TX_WRAPPER_DESC(qinx);

	pr_debug("-->%s(): qinx = %u\n", __func__, qinx);

	/* Unmap and return skb for tx desc/bufs owned by hw.
	 * Caller ensures that hw is no longer accessing these descriptors
	 */
	while (ptx_ring->dirty_tx != ptx_ring->cur_tx) {
		tx_swcx_free(pdata,
			     GET_TX_BUF_PTR(qinx, ptx_ring->dirty_tx));

		INCR_TX_DESC_INDEX(ptx_ring->dirty_tx, 1);
	}
	pr_debug("<--%s()\n", __func__);
}

/*!
* \brief API to free the transmit descriptor skb memory.
*
* \details This function is used to free the transmit descriptor skb memory.
*
* \param[in] pdata - pointer to private data structure.
*
* \retval void.
*/

static void eqos_tx_skb_free_mem(struct eqos_prv_data *pdata,
					UINT tx_qcnt)
{
	UINT qinx;

	pr_debug("-->eqos_tx_skb_free_mem: tx_qcnt = %d\n", tx_qcnt);

	for (qinx = 0; qinx < tx_qcnt; qinx++)
		eqos_tx_skb_free_mem_single_q(pdata, qinx);

	pr_debug("<--eqos_tx_skb_free_mem\n");
}


/*!
 * \details This function is invoked by other function to free
 * the rx socket buffers.
 *
 * \param[in] pdata – pointer to private data structure.
 *
 * \return void
 */

static void eqos_rx_skb_free_mem_single_q(struct eqos_prv_data *pdata,
							UINT qinx)
{
	UINT i;

	pr_debug("-->eqos_rx_skb_free_mem_single_q: qinx = %u\n", qinx);

	for (i = 0; i < RX_DESC_CNT; i++)
		eqos_unmap_rx_skb(pdata, GET_RX_BUF_PTR(qinx, i));

	pr_debug("<--eqos_rx_skb_free_mem_single_q\n");
}

/*!
* \brief API to free the receive descriptor skb memory.
*
* \details This function is used to free the receive descriptor skb memory.
*
* \param[in] pdata - pointer to private data structure.
*
* \retval void.
*/

static void eqos_rx_skb_free_mem(struct eqos_prv_data *pdata,
					UINT rx_qcnt)
{
	UINT qinx;

	pr_debug("-->eqos_rx_skb_free_mem: rx_qcnt = %d\n", rx_qcnt);

	for (qinx = 0; qinx < rx_qcnt; qinx++)
		eqos_rx_skb_free_mem_single_q(pdata, qinx);

	/* unmap reserved DMA*/
	if (pdata->resv_dma) {
		dma_unmap_single(&pdata->pdev->dev, pdata->resv_dma,
				 pdata->rx_buffer_len,
				 DMA_FROM_DEVICE);
		pdata->resv_dma = 0;
	}

	if (pdata->resv_skb) {
		dev_kfree_skb_any(pdata->resv_skb);
		pdata->resv_skb = NULL;
	}

	pr_debug("<--eqos_rx_skb_free_mem\n");
}

/*!
* \brief API to free the transmit descriptor wrapper buffer memory.
*
* \details This function is used to free the transmit descriptor wrapper buffer memory.
*
* \param[in] pdata - pointer to private data structure.
*
* \retval void.
*/

static void eqos_tx_buf_free_mem(struct eqos_prv_data *pdata,
					UINT tx_qcnt)
{
	UINT qinx;

	pr_debug("-->eqos_tx_buf_free_mem: tx_qcnt = %d\n", tx_qcnt);

	for (qinx = 0; qinx < tx_qcnt; qinx++) {
		/* free TX buffer */
		if (GET_TX_BUF_PTR(qinx, 0)) {
			kfree(GET_TX_BUF_PTR(qinx, 0));
			GET_TX_BUF_PTR(qinx, 0) = NULL;
		}
	}

	pr_debug("<--eqos_tx_buf_free_mem\n");
}

/*!
* \brief API to free the receive descriptor wrapper buffer memory.
*
* \details This function is used to free the receive descriptor wrapper buffer memory.
*
* \param[in] pdata - pointer to private data structure.
*
* \retval void.
*/

static void eqos_rx_buf_free_mem(struct eqos_prv_data *pdata,
					UINT rx_qcnt)
{
	UINT qinx = 0;

	pr_debug("-->eqos_rx_buf_free_mem: rx_qcnt = %d\n", rx_qcnt);

	for (qinx = 0; qinx < rx_qcnt; qinx++) {
		if (GET_RX_BUF_PTR(qinx, 0)) {
			kfree(GET_RX_BUF_PTR(qinx, 0));
			GET_RX_BUF_PTR(qinx, 0) = NULL;
		}
	}

	pr_debug("<--eqos_rx_buf_free_mem\n");
}

/*!
 * \brief api to tcp_udp_hdrlen
 *
 * \details This function is invoked by eqos_handle_tso.
 * This function will get the header type and return the header length.
 *
 * \param[in] skb – pointer to socket buffer structure.
 *
 * \return integer
 *
 * \retval tcp or udp header length
 */

static unsigned int tcp_udp_hdrlen(struct sk_buff *skb)
{
	if (skb_shinfo(skb)->gso_type & (SKB_GSO_UDP))
		return sizeof(struct udphdr);
	else
		return tcp_hdrlen(skb);
}

/*!
 * \brief api to handle tso
 *
 * \details This function is invoked by start_xmit functions. This function
 * will get all the tso details like MSS(Maximum Segment Size), packet header length,
 * packet pay load length and tcp header length etc if the given skb has tso
 * packet and store it in other wrapper tx structure for later usage.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] skb – pointer to socket buffer structure.
 *
 * \return integer
 *
 * \retval 1 on success, -ve no failure and 0 if not tso pkt
 * */

static int eqos_handle_tso(struct net_device *dev,
			   struct sk_buff *skb)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);
	UINT qinx = skb_get_queue_mapping(skb);
	struct s_tx_pkt_features *tx_pkt_features = GET_TX_PKT_FEATURES_PTR(qinx);
	int ret = 1;

	pr_debug("-->eqos_handle_tso\n");

	if (skb_is_gso(skb) == 0) {
		pr_debug("This is not a TSO/LSO/GSO packet\n");
		return 0;
	}

	pr_debug("Got TSO packet\n");

	if (skb_header_cloned(skb)) {
		ret = pskb_expand_head(skb, 0, 0, GFP_ATOMIC);
		if (ret)
			return ret;
	}

	/* get TSO or UFO details */
	tx_pkt_features->hdr_len = skb_transport_offset(skb) +
				   tcp_udp_hdrlen(skb);
	tx_pkt_features->tcp_udp_hdr_len = tcp_udp_hdrlen(skb);

	if (skb_shinfo(skb)->gso_type & (SKB_GSO_UDP))
		tx_pkt_features->mss = skb_shinfo(skb)->gso_size -
				       sizeof(struct udphdr);
	else
		tx_pkt_features->mss = skb_shinfo(skb)->gso_size;

	tx_pkt_features->pay_len = (skb->len - tx_pkt_features->hdr_len);

	pr_debug("mss			= %lu\n", tx_pkt_features->mss);
	pr_debug("hdr_len		= %lu\n", tx_pkt_features->hdr_len);
	pr_debug("pay_len		= %lu\n", tx_pkt_features->pay_len);
	pr_debug("tcp_udp_hdr_len	= %lu\n",
		 tx_pkt_features->tcp_udp_hdr_len);

	pr_debug("<--eqos_handle_tso\n");

	return ret;
}

/* returns 0 on success and -ve on failure */
static int eqos_map_non_page_buffs_64(struct eqos_prv_data *pdata,
	struct tx_swcx_desc *ptx_swcx_desc, struct sk_buff *skb,
	unsigned int offset, unsigned int size)
{
	pr_debug("-->eqos_map_non_page_buffs_64");

	if (size > EQOS_MAX_DATA_PER_TX_BUF) {
		pr_err("failed to allocate buffer(size = %d) with %d size\n",
				EQOS_MAX_DATA_PER_TX_BUF,
				size);
		return -ENOMEM;
	}

	ptx_swcx_desc->dma = dma_map_single((&pdata->pdev->dev),
			(skb->data + offset),
			size, DMA_TO_DEVICE);

	if (dma_mapping_error((&pdata->pdev->dev), ptx_swcx_desc->dma)) {
		pr_err("failed to do the dma map\n");
		ptx_swcx_desc->dma = 0;
		return -ENOMEM;
	}
	ptx_swcx_desc->len = size;
	ptx_swcx_desc->buf1_mapped_as_page = Y_FALSE;

	pr_debug("<--eqos_map_non_page_buffs_64");
	return 0;
}

static int eqos_map_page_buffs_64(struct eqos_prv_data *pdata,
			struct tx_swcx_desc *ptx_swcx_desc,
			struct skb_frag_struct *frag,
			unsigned int offset,
			unsigned int size)
{
	unsigned int page_idx = (frag->page_offset + offset) >> PAGE_SHIFT;
	unsigned int page_offset = (frag->page_offset + offset) & ~PAGE_MASK;
	pr_debug("-->eqos_map_page_buffs_64\n");
	/* fill the first buffer pointer in buffer->dma */
	ptx_swcx_desc->dma = dma_map_page((&pdata->pdev->dev),
				(frag->page.p + page_idx),
				page_offset,
				size, DMA_TO_DEVICE);
	if (dma_mapping_error((&pdata->pdev->dev),
				ptx_swcx_desc->dma)) {
		pr_err("failed to do the dma map\n");
		ptx_swcx_desc->dma = 0;
		return -ENOMEM;
	}
	ptx_swcx_desc->len = size;
	ptx_swcx_desc->buf1_mapped_as_page = Y_TRUE;

	pr_debug("<--eqos_map_page_buffs_64\n");
	return 0;
}


/*!
 * \details This function is invoked by start_xmit functions. Given a skb
 * this function will allocate and initialize tx_swcx entries.
 * Function needs to handle case where there is not enough free tx_swcx to
 * handle the skb.  A free tx_swcx entry is one with len set to zero.
 * Note that since tx_swcx mirrors the tx descriptor ring, an entry must
 * be used if a context descriptor is needed.  A len value of "-1" is used for
 * these tx_swcx entries.
 *
 * \param[in] dev – pointer to net device structure.
 * \param[in] skb – pointer to socket buffer structure.
 *
 * \return unsigned int
 *
 * \retval count of number of tx_swcx entries allocated.  "-1" is returned
 * if there is a map failure.  "0" is returned if there is not a free
 * tx_swcx entry.
 */

static int tx_swcx_alloc(struct net_device *dev, struct sk_buff *skb)
{
	struct eqos_prv_data *pdata = netdev_priv(dev);

	uint qinx = skb_get_queue_mapping(skb);

	struct tx_ring *ptx_ring = GET_TX_WRAPPER_DESC(qinx);
	int idx = (int)ptx_ring->cur_tx;

	struct tx_swcx_desc *ptx_swcx = NULL;
	struct s_tx_pkt_features *ppkt_opts = GET_TX_PKT_FEATURES_PTR(qinx);

	uint frag_cnt;
	uint hdr_len = 0;
	uint i;
	uint cnt = 0, offset = 0, size;
	int len;
	int totlen = 0;
	int ret = -1;
	bool is_pkt_tso, is_pkt_vlan;

	pr_debug("-->%s(): cur_tx = %d, qinx = %u\n", __func__, idx, qinx);

	TX_PKT_FEATURES_PKT_ATTRIBUTES_TSO_ENABLE_RD(
		ppkt_opts->pkt_attributes, is_pkt_tso);
	TX_PKT_FEATURES_PKT_ATTRIBUTES_VLAN_PKT_RD(
		ppkt_opts->pkt_attributes, is_pkt_vlan);

	if ((is_pkt_vlan) ||
	    (is_pkt_tso)) {
		ptx_swcx = GET_TX_BUF_PTR(qinx, idx);
		if (ptx_swcx->len)
			goto tx_swcx_alloc_failed;

		ptx_swcx->len = -1;
		cnt++;
		INCR_TX_DESC_INDEX(idx, 1);
	}

	if (is_pkt_tso) {
		hdr_len = skb_transport_offset(skb) + tcp_udp_hdrlen(skb);
		len = hdr_len;
	} else {
		len = (skb->len - skb->data_len);
	}

	pr_debug("%s(): skb->len - skb->data_len = %d, hdr_len = %d\n",
	      __func__, len, hdr_len);

	totlen += len;
	while (len) {
		ptx_swcx = GET_TX_BUF_PTR(qinx, idx);
		if (ptx_swcx->len)
			goto tx_swcx_alloc_failed;

		size = min(len, EQOS_MAX_DATA_PER_TXD);

		ret = eqos_map_non_page_buffs_64(pdata, ptx_swcx,
						 skb, offset, size);
		if (ret < 0)
			goto tx_swcx_map_failed;

		len -= size;
		offset += size;
		cnt++;

		INCR_TX_DESC_INDEX(idx, 1);
	}

	/* Process remaining pay load in skb->data in case of TSO packet */
	if (is_pkt_tso) {
		len = ((skb->len - skb->data_len) - hdr_len);
		totlen += len;
		while (len > 0) {
			ptx_swcx = GET_TX_BUF_PTR(qinx, idx);
			if (ptx_swcx->len)
				goto tx_swcx_alloc_failed;

			size = min(len, EQOS_MAX_DATA_PER_TXD);

			ret = eqos_map_non_page_buffs_64(pdata, ptx_swcx,
							 skb, offset, size);
			if (ret < 0)
				goto tx_swcx_map_failed;

			len -= size;
			offset += size;
			cnt++;

			INCR_TX_DESC_INDEX(idx, 1);
		}
	}

	/* Process fragmented skb's */
	frag_cnt = skb_shinfo(skb)->nr_frags;
	for (i = 0; i < frag_cnt; i++) {
		struct skb_frag_struct *frag = &skb_shinfo(skb)->frags[i];

		len = frag->size;
		totlen += len;
		offset = 0;
		while (len) {
			ptx_swcx = GET_TX_BUF_PTR(qinx, idx);
			if (ptx_swcx->len)
				goto tx_swcx_alloc_failed;

			size = min(len, EQOS_MAX_DATA_PER_TXD);

			ret = eqos_map_page_buffs_64(pdata, ptx_swcx,
						     frag, offset, size);
			if (ret < 0)
				goto tx_swcx_map_failed;

			len -= size;
			offset += size;
			cnt++;

			INCR_TX_DESC_INDEX(idx, 1);
		}
	}
	ptx_swcx->skb = skb;

	ppkt_opts->desc_cnt = cnt;
	if (!is_pkt_tso)
		ppkt_opts->pay_len = totlen;

	pr_debug("<--%s(): ptx_swcx->dma = %#llx\n",
	      __func__, (ULONG_LONG) ptx_swcx->dma);

	return cnt;

tx_swcx_alloc_failed:

	ret = 0;
	DECR_TX_DESC_INDEX(idx);

tx_swcx_map_failed:
	while (cnt) {
		ptx_swcx = GET_TX_BUF_PTR(qinx, idx);
		tx_swcx_free(pdata, ptx_swcx);
		DECR_TX_DESC_INDEX(idx);
		cnt--;
	}
	return ret;
}


static void tx_swcx_free(struct eqos_prv_data *pdata,
			 struct tx_swcx_desc *ptx_swcx)
{
	pr_debug("-->%s()\n", __func__);
	if (ptx_swcx->dma) {
		if (ptx_swcx->buf1_mapped_as_page == Y_TRUE)
			dma_unmap_page((&pdata->pdev->dev), ptx_swcx->dma,
				       ptx_swcx->len,
				       DMA_TO_DEVICE);
		else
			dma_unmap_single((&pdata->pdev->dev),
					 ptx_swcx->dma,
					 ptx_swcx->len,
					 DMA_TO_DEVICE);

		ptx_swcx->dma = 0;
	}

	if (ptx_swcx->skb != NULL) {
		dev_kfree_skb_any(ptx_swcx->skb);
		ptx_swcx->skb = NULL;
	}
	ptx_swcx->len = 0;

	pr_debug("<--%s()\n", __func__);
}

/*!
 * \details This function is invoked by other function for releasing the socket
 * buffer which are received by device and passed to upper layer.
 *
 * \param[in] pdata – pointer to private device structure.
 * \param[in] buffer – pointer to rx wrapper buffer structure.
 *
 * \return void
 */

static void eqos_unmap_rx_skb(struct eqos_prv_data *pdata,
				     struct rx_swcx_desc *prx_swcx_desc)
{
	pr_debug("-->eqos_unmap_rx_skb\n");

	/* unmap the first buffer */
	if (prx_swcx_desc->dma) {
		if (pdata->resv_dma != prx_swcx_desc->dma) {
			dma_unmap_single(&pdata->pdev->dev, prx_swcx_desc->dma,
					 pdata->rx_buffer_len,
					 DMA_FROM_DEVICE);
		}
		prx_swcx_desc->dma = 0;
	}

	if (prx_swcx_desc->skb) {
		if (pdata->resv_skb != prx_swcx_desc->skb) {
			dev_kfree_skb_any(prx_swcx_desc->skb);
		}
		prx_swcx_desc->skb = NULL;
	}

	pr_debug("<--eqos_unmap_rx_skb\n");
}

/*!
* \brief API to re-allocate the new skb to rx descriptors.
*
* \details This function is used to re-allocate & re-assign the new skb to
* receive descriptors from which driver has read the data. Also ownership bit
* and other bits are reset so that device can reuse the descriptors.
*
* \param[in] pdata - pointer to private data structure.
*
* \return void.
*/

static void eqos_re_alloc_skb(struct eqos_prv_data *pdata,
				UINT qinx)
{
	struct rx_ring *prx_ring =
	    GET_RX_WRAPPER_DESC(qinx);
	struct rx_swcx_desc *prx_swcx_desc = NULL;
	struct hw_if_struct *hw_if = &pdata->hw_if;
	int tail_idx, ret;

	while (prx_ring->dirty_rx != prx_ring->cur_rx) {
		prx_swcx_desc = GET_RX_BUF_PTR(qinx, prx_ring->dirty_rx);

		ret = desc_alloc_skb(pdata, prx_swcx_desc, GFP_ATOMIC, qinx);
		if (ret < 0) {
			break;
		}

		hw_if->rx_desc_reset(prx_ring->dirty_rx, pdata,
				     prx_swcx_desc->inte, qinx);
		INCR_RX_DESC_INDEX(prx_ring->dirty_rx, 1);
	}

	tail_idx = prx_ring->dirty_rx;
	DECR_RX_DESC_INDEX(tail_idx);

	/* make sure Rx ring tail index update */
	wmb();
	hw_if->update_rx_tail_ptr(qinx,
		GET_RX_DESC_DMA_ADDR(qinx, tail_idx));

	return;
}

/*!
* \brief API to initialize the function pointers.
*
* \details This function is called in probe to initialize all the function
* pointers which are used in other functions to manage edscriptors.
*
* \param[in] desc_if - pointer to desc_if_struct structure.
*
* \return void.
*/

void eqos_init_function_ptrs_desc(struct desc_if_struct *desc_if)
{

	pr_debug("-->eqos_init_function_ptrs_desc\n");

	desc_if->alloc_queue_struct = eqos_alloc_queue_struct;
	desc_if->free_queue_struct = eqos_free_queue_struct;
	desc_if->alloc_buff_and_desc = allocate_buffer_and_desc;
	desc_if->free_buff_and_desc = free_buffer_and_desc;
	desc_if->realloc_skb = eqos_re_alloc_skb;
	desc_if->unmap_rx_skb = eqos_unmap_rx_skb;
	desc_if->tx_swcx_free = tx_swcx_free;
	desc_if->tx_swcx_alloc = tx_swcx_alloc;
	desc_if->tx_free_mem = eqos_tx_free_mem;
	desc_if->rx_free_mem = eqos_rx_free_mem;
	desc_if->wrapper_tx_desc_init = eqos_wrapper_tx_descriptor_init;
	desc_if->wrapper_tx_desc_init_single_q =
	    eqos_wrapper_tx_descriptor_init_single_q;
	desc_if->wrapper_rx_desc_init = eqos_wrapper_rx_descriptor_init;
	desc_if->wrapper_rx_desc_init_single_q =
	    eqos_wrapper_rx_descriptor_init_single_q;

	desc_if->rx_skb_free_mem = eqos_rx_skb_free_mem;
	desc_if->rx_skb_free_mem_single_q = eqos_rx_skb_free_mem_single_q;
	desc_if->tx_skb_free_mem = eqos_tx_skb_free_mem;
	desc_if->tx_skb_free_mem_single_q = eqos_tx_skb_free_mem_single_q;

	desc_if->handle_tso = eqos_handle_tso;

	pr_debug("<--eqos_init_function_ptrs_desc\n");
}
