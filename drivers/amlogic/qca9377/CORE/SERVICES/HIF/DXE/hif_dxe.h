/*
 * Copyright (c) 2012 The Linux Foundation. All rights reserved.
 *
 * Previously licensed under the ISC license by Qualcomm Atheros, Inc.
 *
 *
 * Permission to use, copy, modify, and/or distribute this software for
 * any purpose with or without fee is hereby granted, provided that the
 * above copyright notice and this permission notice appear in all
 * copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
 * WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 * AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL
 * DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR
 * PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER
 * TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
 * PERFORMANCE OF THIS SOFTWARE.
 */

/*
 * This file was originally distributed by Qualcomm Atheros, Inc.
 * under proprietary terms before Copyright ownership was assigned
 * to the Linux Foundation.
 */

/**
 * @file hif_dxe.h
 * @brief Provide functions for DXE HIF module for RIVA FW.
 * @details
 *  This file Provides the HIF DXE Module API For RIVA Integrated WLAN Platform  .
 *  DXE software module communicates with the RIVA DXE HW block for data path which
 *  is a DMA engine to transfer Data between [ Host DDR <--> Target DDR]
 *  Provides API to :
 *  - Initialize the DMA block registers , Allocate DMA Memory for Descriptors/Rx Network Packets.
 *  - Register Tx Complete , Rx and Low Resource Callbacks.
 *  - Send Tx Data packets and report Rx Packets
 *  - Report Available Free Slots in Tx Ring
 *  - Flush Tx Packets
 *  - Set DXE Power State when Host  transitions in and out of Power Save
 *  - Stop DXE and Deallocate resources on Driver Unloads
 */
#ifndef _HIF_DXE_H_
#define _HIF_DXE_H_
#include "adf_os_types.h"
#include "athdefs.h"
#include "a_types.h" // for A_BOOL/TRUE/FALSE
#include "adf_nbuf.h" /* adf_nbuf_t */

typedef void * hif_dxe_handle;

typedef enum _E_HIFDXE_CHANNELTYPE
{
    HIFDXE_CHANNEL_TX_LOW_PRI,
    HIFDXE_CHANNEL_TX_HIGH_PRI,
    HIFDXE_CHANNEL_RX_LOW_PRI,
    HIFDXE_CHANNEL_RX_HIGH_PRI,
    HIFDXE_CHANNEL_MAX
} E_HIFDXE_CHANNELTYPE;

typedef A_STATUS (*HIFDXE_TxCompleteCb)(void *pContext, adf_nbuf_t tx_list, E_HIFDXE_CHANNELTYPE channel ,A_STATUS status);
typedef A_STATUS (*HIFDXE_RxFrameReadyCb) (void *pContext, adf_nbuf_t rx_list, E_HIFDXE_CHANNELTYPE channel);
typedef A_STATUS (*HIFDXE_LowResourceCb)(void *pContext, E_HIFDXE_CHANNELTYPE channel, A_BOOL is_low_resource);


typedef struct _S_HIFDXE_CALLBACK
{
    HIFDXE_TxCompleteCb HifTxCompleteCb;
    HIFDXE_RxFrameReadyCb HifRxReadyCb;
    HIFDXE_LowResourceCb HifLowResourceCb;
    void *HifTxCompleteCtx;
    void *HifRxReadyCtx;
    void *HifLowResourceCtx;
} S_HIFDXE_CALLBACK;


/**
 * @ HIF DXE Attach . One Time Initialize of DXE HW / Allocate DXE SW Module Context / Alloc DXE Descriptors , Rx Buffers , Control Block /Attach OS Interrupts etc
 *
 * @param[in]dev - OS Dev Handle
 *
 * @retval   hif_dxe_handle  Allocates HIF DEX Context Block and returns handle for subsequent calls to DXE
 */
hif_dxe_handle hif_dxe_attach(adf_os_device_t dev);

/**
 * @ HIF DXE Start . Initialize the DMA Channels and per channel DMA Registers to Valid Values.
 *
 * @param[in]hif_dxe_pdev -  HIF DXE Specific Context
 *
 * @retval   A_STATUS
 */

A_STATUS hif_dxe_start(hif_dxe_handle hif_dxe_pdev);

/**
 * @ HIF DXE Register . Register Packet Callbacks
 *
 * @param[in]hif_dxe_pdev -  HIF DXE Specific Context
 *
 * @retval   A_STATUS
 */
A_STATUS hif_dxe_client_registration(hif_dxe_handle hif_dxe_pdev,  S_HIFDXE_CALLBACK *hif_dxe_cb);

/**
 * @ HIF DXE Send . Send packet on DXE
 *
 * @param[in]hif_dxe_pdev -  HIF DXE Specific Context
 * @param[in]eHifDxeChannel -  DXE Channel for Send
 * @param[in]nbuf -  SDU to send
 *
 * @retval   A_STATUS
 */
A_STATUS hif_dxe_send(hif_dxe_handle hif_dxe_pdev, E_HIFDXE_CHANNELTYPE channel, adf_nbuf_t nbuf);



//FIXME_RT : Do We Need This
//A_STATUS HIF_DXE_CompleteTX(void *hif_dxe_pdev);


/**
 * @ HIF DXE GetResources . Return Available Free Tx Slots in DXE DMA  Tx Ring for specified channel
 *
 * @param[in]hif_dxe_pdev -  HIF DXE Specific Context
 * @param[in]eHifDxeChannel -  DXE Channel
 *
 * @retval   a_uint32_t Num Available Resources
 */
u_int32_t hif_dxe_get_resources(hif_dxe_handle hif_dxe_pdev, E_HIFDXE_CHANNELTYPE channel);


/**
 * @ HIF DXE Flush Tx Packets . Flush And return Tx Packets Pending in DMA through Tx Completion Callbacks.
 *
 * @param[in]hif_dxe_pdev -  HIF DXE Specific Context
 *
 * @retval   A_STATUS
 */
A_STATUS hif_dxe_flush_packets(hif_dxe_handle hif_dxe_pdev);


/**
 * @ HIF DXE Set Power State . Synchronously Set Power State relevant for DXE / Ensure No references into DXE are Outstanding
 *
 * @param[in]hif_dxe_pdev -  HIF DXE Specific Context
 * @param[in]PowerState    -  New Power State
 *
 * @retval   A_STATUS
 */
A_STATUS hif_dxe_set_power_state(hif_dxe_handle hif_dxe_pdev, u_int8_t power_state );


/**
 * @brief HIF DXE Read register
 *
 * @param[in] hif_dxe_pdev -  HIF DXE Specific Context
 * @param[in] Addr
 *
 * @return Read Value
 */
u_int32_t hif_dxe_readreg(hif_dxe_handle hif_dxe_pdev, u_int32_t addr);


/**
 * @brief HIF DXE Write register
 *
 * @param[in] hif_dxe_pdev - HIF DXE Specific Context
 * @param[in] Addr
 * @param[in] val
 *
 * @return void
 */
void
hif_dxe_writereg(hif_dxe_handle hif_dxe_pdev, u_int32_t addr, u_int32_t val);


/**
 * @ HIF DXE Stop . Stop DMA
 *
 * @param[in]hif_dxe_pdev -  HIF DXE Specific Context
 *
 * @retval   A_STATUS
 */
A_STATUS hif_dxe_stop(hif_dxe_handle hif_dxe_pdev);


/**
 * @ HIF DXE Detach . UnInitialize DXE Resources / HW / Free Memory  / Detach OS Interrupts etc
 *
 * @param[in]hif_dxe_pdev -  HIF DXE Specific Context
 *
 * @retval   A_STATUS
 */
A_STATUS hif_dxe_detach(hif_dxe_handle hif_dxe_pdev);

#endif
