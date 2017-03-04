/*
 * Copyright (c) 2013 The Linux Foundation. All rights reserved.
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

/**========================================================================

  \file     hif_dxe_ospvt.h
  \brief    OS Private Layer used by Linux DXE OS Specific Module

  ========================================================================*/
/**=========================================================================
  EDIT HISTORY FOR FILE


  This section contains comments describing changes made to the module.
  Notice that changes are listed in reverse chronological order.

  $Header:$   $DateTime: $ $Author: $


  when              who           what, where, why
  --------          ---           -----------------------------------------
  05/03/2013        Ganesh        Created module for HIF Dxe Linux Specific
                    Babu
  ==========================================================================*/

#ifndef _HIF_DXE_PVTOS_H_
#define _HIF_DXE_PVTOS_H_
#include <linux/irqreturn.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include "athdefs.h"
#include "adf_os_types.h"
#include "adf_os_atomic.h"
#include <linux/wcnss_wlan.h>


#define DXE_READ_REG(ctx,addr)   hif_dxe_os_readreg(ctx,addr)
#define DXE_WRITE_REG(ctx,addr,val) hif_dxe_os_writereg(ctx,addr,val)


typedef struct _S_HIF_DXE_OSCONTEXT
{
	struct resource       *wcnss_memory;
	void __iomem          *mmio;

	/***************************************/
	/*        Tx Irq related Params                         */
	/***************************************/

	int   tx_irq;

	/* Whether Tx Irq is registered with the OS or not */
	unsigned char   bTxRegistered;

	/* Tasklet to do the Tx Processing */
	struct tasklet_struct hif_dxe_tx_tasklet;


	/***************************************/
	/*        Rx Irq related Params                         */
	/***************************************/

	int   rx_irq;

	/* Whether Rx Irq is registered with the OS or not */
	unsigned char   bRxRegistered;

	/* Tasklet to do the Rx Processing */
	struct tasklet_struct hif_dxe_rx_tasklet;

	unsigned char      bTxIntEnabled , bRxIntEnabled;
	S_HIF_DXE_OS_PARAMS dxe_hif_params;
	adf_os_atomic_t ref_count;

	/* Currently not used. Is it required ??? */
	unsigned long        dxe_unloaded;
}S_HIF_DXE_OSCONTEXT;
#endif
