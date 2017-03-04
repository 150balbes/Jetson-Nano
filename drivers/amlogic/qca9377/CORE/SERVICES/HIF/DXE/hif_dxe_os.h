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
 * @file hif_dxe_os.h
 * @brief Provide  OS Specific Shim Layer API for DXE HIF module.
 * @details
 *  This file Provides the  OS Specific Layer API for HIF DXE Module.
 *  DXE software module communicates with this API for Windows OS specific Initialization
 *  Provides API For :
 *  1. OS Initialization required for DXE Module
 *  2. Read/Write DXE registers in OS Specific Manner
 *  3. Allocate/Free DMA Memory for DXE Descriptors
 *  4. Communicate with SMSM Driver
 *  5. Other
 */
#ifndef _HIF_DXE_OS_H_
#define _HIF_DXE_OS_H_

#include "adf_os_types.h"
#include "athdefs.h"
#include "a_debug.h"

struct _S_HIF_DXE_OSCONTEXT;
typedef struct  _S_HIF_DXE_OSCONTEXT * hif_dxe_oshandle;

typedef enum _E_DXE_INTTYPE
{
  E_HIFDXE_TX_INTERRUPT,
  E_HIFDXE_RX_INTERRUPT
}E_DXE_INTTYPE;

typedef enum _E_DXE_INT_ACTION
{
  E_HIFDXE_INT_ENABLE,
  E_HIFDXE_INT_DISABLE,
  E_HIFDXE_INT_ACTIVATE,
  E_HIFDXE_INT_DEACTIVATE
}E_DXE_INT_ACTION;

typedef A_STATUS (*hif_dxe_os_cb)(void *ctx);



typedef struct _S_HIF_DXE_OS_PARAMS
{
   hif_dxe_os_cb dxe_tx_cb;
   hif_dxe_os_cb dxe_rx_cb;
   void          *pvcontext;
}S_HIF_DXE_OS_PARAMS;

#define     HIFDXE_SMSM_WLAN_TX_ENABLE          0x00000400
#define     HIFDXE_SMSM_WLAN_TX_RINGS_NOT_EMPTY     0x00000200

A_STATUS ol_ath_dxe_initialize(void* Adapter);


/**
 * @ HIF OS Init Dxe
 *
 * @param[in/out] osdev  OS Specific Context
 *
 * @retval    hif_dxe_oshandle Returns OS Specific Allocation Handle
 */
hif_dxe_oshandle hif_dxe_os_init(adf_os_device_t osdev,
                                   S_HIF_DXE_OS_PARAMS *params);


/**
 * @ HIF OS DeInit Dxe
 *
 * @param[in]hif_dxe_osdev  OS Specific Context
 *
 * @retval   void
 */
void hif_dxe_os_deinit(hif_dxe_oshandle hif_dxe_osdev);

/**
 * @ HIF OS Enable / Disable TXRX Interrupt
 *
 * @param[in]hif_dxe_osdev - OS Specific Context
 * @param[in]eIntType
 * @param[in]bEnable
 *
 * @retval   A_STATUS
 */
A_STATUS hif_dxe_os_program_int(hif_dxe_oshandle hif_dxe_osdev ,
                           E_DXE_INTTYPE eIntType , E_DXE_INT_ACTION eAction);


/**
 * @ HIF OS Notify SMSM
 *
 * @param[in]hif_dxe_osdev - OS Specific Context
 * @param[in]clrSt
 * @param[in]setSt
 *
 * @retval   A_STATUS
 */
A_STATUS hif_dxe_os_notifysmsm(hif_dxe_oshandle hif_dxe_osdev,
                                    u_int32_t clrSt, u_int32_t setSt);

/**
 * @brief HIF Os Read register
 *
 * @param[in] OS Context
 * @param[in] Addr
 *
 * @return Read Value
 */
u_int32_t
hif_dxe_os_readreg(hif_dxe_oshandle hif_dxe_osdev, u_int32_t addr);


/**
 * @brief HIF Os Write register
 *
 * @param[in] OS Context
 * @param[in] Addr
 * @param[in] val
 *
 * @return void
 */
void
hif_dxe_os_writereg(hif_dxe_oshandle hif_dxe_osdev, u_int32_t addr,
                       u_int32_t val);

/**
 * @brief HIF Mem Barrier
 *
 * @param[in] void
 *
 * @return void
 */
void hif_dxe_os_mem_barrier(void);

void hif_dxe_os_dbgdump(hif_dxe_oshandle hif_dxe_osdev);

void hif_dxe_os_stop(hif_dxe_oshandle hif_dxe_osdev);

#endif /* _HIF_DXE_OS_H_ */