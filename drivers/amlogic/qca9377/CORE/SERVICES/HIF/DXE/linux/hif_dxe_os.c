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

  \file     hif_dxe_os.c
  \brief    Implementation of WMA
		Provide Linux OS Specific Shim Layer Implementation for
		DXE HIF module.

		This file Provides the Linux OS Specific Layer API for
		HIF DXE Module.

		DXE software module communicates with this layer
		for Linux Specific Functionality

		Implements :
		1. Linux OS Initialization required for DXE Module
			- Get hold of and Initialize DXE OS Resources(
				Interrupt , Shared memory)/SMSM Device
			- Register ISR/Tasklets for TXRX Interrupts
			- Enable/Disable TXRX Interrupts
			- DMA Adapter Registration / Initialization
		2. Read/Write DXE registers in Windows Platform Specific Manner
		3. Allocate/Free DMA Memory for DXE Descriptors

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

#include "adf_os_types.h"
#include "adf_os_lock.h"
#include "adf_os_mem.h"
#include "athdefs.h"
#include "a_debug.h"
#include "hif_dxe_os.h"
#include "hif_dxe_ospvt.h"
#include "hif_dxe.h"
#include "hif_dxe_hw_pvt.h"
#include "hif_dxe_pvt.h"
#include <mach/msm_smsm.h>
#include "vos_threads.h"

#define ATH_MODULE_NAME hif_dxe_linuxos

#define HIF_DXE_LINUXOS_DEBUG   ATH_DEBUG_MAKE_MODULE_MASK(0)

#if defined(DEBUG)
static ATH_DEBUG_MASK_DESCRIPTION g_HIFOSDebugDescription[] = {
	{HIF_DXE_LINUXOS_DEBUG,"hif_dxe_linuxos"},
};

ATH_DEBUG_INSTANTIATE_MODULE_VAR(hif_dxe_linuxos,
		"hif_dxe_linuxos",
		"DXE HIF Linux OS Shim Interface",
		ATH_DEBUG_MASK_DEFAULTS | ATH_DEBUG_INFO | HIF_DXE_LINUXOS_DEBUG,
		ATH_DEBUG_DESCRIPTION_COUNT(g_HIFOSDebugDescription),
		g_HIFOSDebugDescription);
#endif

irqreturn_t hif_dxe_os_tx_isr(int irq,void* pSrvcCtx);
irqreturn_t hif_dxe_os_rx_isr(int irq,void* pSrvcCtx);


/**
 * @ HIF OS Enable Tx Interrupt
 *
 * @param[in/out] osdev  OS Specific Context
 *
 * @retval     TRUE/FALSE [Success/Fail]
 */
unsigned char hif_dxe_os_enable_tx_intr(void* pctx)
{
	S_HIF_DXE_OSCONTEXT * hif_dxeos_ctx = pctx;
	u_int32_t val;
	int ret;

	/* Enable Tx IRQ at OS level */
	if(!hif_dxeos_ctx->bTxRegistered)
	{
		ret = request_irq(hif_dxeos_ctx->tx_irq, hif_dxe_os_tx_isr,
				IRQF_TRIGGER_HIGH, "wcnss_wlan", hif_dxeos_ctx);

		if(ret)
		{
			/* TX IRQ request failed */
			AR_DEBUG_PRINTF(ATH_DEBUG_ERR,
					("hif_dxe_os_enable_tx_interrupt : "
					 "TX IRQ request failed\n"));

			return FALSE;
		}

		hif_dxeos_ctx->bTxRegistered = TRUE;
		hif_dxeos_ctx->bTxIntEnabled = TRUE;

		enable_irq_wake(hif_dxeos_ctx->tx_irq);
	}
	else if(!hif_dxeos_ctx->bTxIntEnabled)
	{
		hif_dxeos_ctx->bTxIntEnabled = TRUE;
		enable_irq(hif_dxeos_ctx->tx_irq);
	}

        /* Enable Tx Channel Interrupt */
        val = TX_INT_SELECT(CH_TX_LOW_PRI) | TX_INT_SELECT(CH_TX_HIGH_PRI) |
                DXE_READ_REG(hif_dxeos_ctx,WLANDXE_CCU_DXE_INT_SELECT);

        DXE_WRITE_REG(hif_dxeos_ctx,WLANDXE_CCU_DXE_INT_SELECT,val);

	return TRUE;
}

/**
 * @ HIF OS Disable Tx Interrupt
 *
 * @param[in/out] osdev  OS Specific Context
 *
 * @retval     void
 */
unsigned char hif_dxe_os_disable_tx_intr(void* pctx)
{
	S_HIF_DXE_OSCONTEXT * hif_dxeos_ctx = pctx;
	u_int32_t val = 0;

	/* Disable Tx Channel Interrupt */
	val = ~TX_INT_SELECT(CH_TX_LOW_PRI) & ~TX_INT_SELECT(CH_TX_HIGH_PRI) &
		DXE_READ_REG(hif_dxeos_ctx,WLANDXE_CCU_DXE_INT_SELECT);

	DXE_WRITE_REG(hif_dxeos_ctx,WLANDXE_CCU_DXE_INT_SELECT,val);

	/* Disable Tx Irq at OS level */
	disable_irq_nosync(hif_dxeos_ctx->tx_irq);

	hif_dxeos_ctx->bTxIntEnabled = FALSE;

	return TRUE;
}

/**
 * @ HIF OS Enable Rx Interrupt
 *
 * @param[in/out] osdev  OS Specific Context
 *
 * @retval     TRUE/FALSE [Success/Fail]
 */
unsigned char hif_dxe_os_enable_rx_intr(void* pctx)
{
	S_HIF_DXE_OSCONTEXT * hif_dxeos_ctx = pctx;
	u_int32_t val = 0;
	int ret = 0;

	/* Enable Rx IRQ at OS level */
	if(!hif_dxeos_ctx->bRxRegistered)
	{
		ret = request_irq(hif_dxeos_ctx->rx_irq, hif_dxe_os_rx_isr,
				IRQF_TRIGGER_HIGH, "wcnss_wlan", hif_dxeos_ctx);

		if(ret)
		{
			/* RX IRQ request failed */
			AR_DEBUG_PRINTF(ATH_DEBUG_ERR,
				("hif_dxe_os_enable_tx_interrupt :"
				 "RX IRQ request failed\n"));

			return FALSE;
		}

		hif_dxeos_ctx->bRxRegistered = TRUE;
		hif_dxeos_ctx->bRxIntEnabled = TRUE;

		enable_irq_wake(hif_dxeos_ctx->rx_irq);
	}
	else if(!hif_dxeos_ctx->bRxIntEnabled)
	{
		hif_dxeos_ctx->bRxIntEnabled = TRUE;
		enable_irq(hif_dxeos_ctx->rx_irq);
	}

        /* Enable Rx Channel Interrupt */
        val = RX_INT_SELECT(CH_RX_LOW_PRI) | RX_INT_SELECT(CH_RX_HIGH_PRI) |
                DXE_READ_REG(hif_dxeos_ctx,WLANDXE_CCU_DXE_INT_SELECT);

        DXE_WRITE_REG(hif_dxeos_ctx,WLANDXE_CCU_DXE_INT_SELECT,val);

	return TRUE;
}

/**
 * @ HIF OS Disable Rx Interrupt
 *
 * @param[in/out] osdev  OS Specific Context
 *
 * @retval     void
 */
unsigned char hif_dxe_os_disable_rx_intr(void* pctx)
{
	S_HIF_DXE_OSCONTEXT * hif_dxeos_ctx = pctx;
	u_int32_t val = 0;

	/* Disable Rx Channel Interrupt */
	val = ~RX_INT_SELECT(CH_RX_LOW_PRI) & ~RX_INT_SELECT(CH_RX_HIGH_PRI) &
		DXE_READ_REG(hif_dxeos_ctx,WLANDXE_CCU_DXE_INT_SELECT);

	DXE_WRITE_REG(hif_dxeos_ctx,WLANDXE_CCU_DXE_INT_SELECT,val);

	/* Disable Rx Irq at OS level */
	disable_irq_nosync(hif_dxeos_ctx->rx_irq);

	hif_dxeos_ctx->bRxIntEnabled = FALSE;

	return TRUE;
}

/**
  @brief hif_dxe_os_tx_isr is the interrupt service routine which handles
  the DXE TX Complete interrupt

  hif_dxe_os_tx_isr is registered with the Operating System to handle the
  DXE TX Complete interrupt during system initialization.  When a DXE
  TX Complete interrupt occurs, it is dispatched to the handler which
  had previously been registered via wpalRegisterInterrupt.

  @param  dev_id: User-supplied data passed back via the ISR

  @return void
 */
irqreturn_t hif_dxe_os_tx_isr(int irq,void* pSrvcCtx)
{
	S_HIF_DXE_OSCONTEXT * hif_dxeos_ctx = (S_HIF_DXE_OSCONTEXT *)pSrvcCtx;
	u_int32_t val = 0;

	AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s \n",__FUNCTION__));

	if ((NULL == hif_dxeos_ctx) ||
			(NULL == hif_dxeos_ctx->dxe_hif_params.dxe_tx_cb) ||
			(FALSE == hif_dxeos_ctx->bTxIntEnabled))
	{
		AR_DEBUG_PRINTF(ATH_DEBUG_ERR,(
			"%s : Tx Intr Not Initalized "
			"or Enabled \n",__FUNCTION__));

		/* Whether we can return without Setting Processing Bit ???*/
		return IRQ_HANDLED;
	}

	/*
	   Set Interrupt processing bit
	   During this bit set, WLAN HW may not power collapse
	 */
	val = DXE_READ_REG(hif_dxeos_ctx,WLANDXE_INT_MASK_REG_ADDRESS);
	val |= HIFDXE_TX_INTERRUPT_PRO_MASK;
	DXE_WRITE_REG(hif_dxeos_ctx,WLANDXE_INT_MASK_REG_ADDRESS, val);

	/* Disable TX Complete Interrupt at here */
	hif_dxe_os_disable_tx_intr(hif_dxeos_ctx);

	/*  Deferred the processing to Tasklet */
	tasklet_schedule(&hif_dxeos_ctx->hif_dxe_tx_tasklet);

	AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s \n",__FUNCTION__));
	return IRQ_HANDLED;
}


/**
  @brief hif_dxe_tx_tasklet_proc

  Function which actually does the DXE TX Complete Processing.
  This tasklet is scheduled from hif_dxe_os_tx_isr.

  @param  dev_id: User-supplied data passed back via the ISR

  @return void
 */
void hif_dxe_os_tx_processing(unsigned long DeferredContext)
{
	S_HIF_DXE_OSCONTEXT * hif_dxeos_ctx = (S_HIF_DXE_OSCONTEXT *)DeferredContext;

	AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s \n",__FUNCTION__));

	adf_os_atomic_inc(&hif_dxeos_ctx->ref_count);

	/* Call the Dxe Tx Callback */
	if ((NULL != hif_dxeos_ctx) &&
		(NULL != hif_dxeos_ctx->dxe_hif_params.dxe_tx_cb))
	{
		hif_dxeos_ctx->dxe_hif_params.dxe_tx_cb(
			hif_dxeos_ctx->dxe_hif_params.pvcontext);
	}

	adf_os_atomic_dec(&hif_dxeos_ctx->ref_count);

	AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));
}

/**
  @brief hif_dxe_rx_tasklet_proc

  Function which actually does the DXE Rx Data Avail Processing.
  This tasklet is scheduled from hif_dxe_os_rx_isr.

  @param  dev_id: User-supplied data passed back via the ISR

  @return void
 */
void hif_dxe_os_rx_processing(unsigned long DeferredContext)
{
	S_HIF_DXE_OSCONTEXT * hif_dxeos_ctx =
				(S_HIF_DXE_OSCONTEXT *)DeferredContext;

	AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s \n",__FUNCTION__));

	adf_os_atomic_inc(&hif_dxeos_ctx->ref_count);

	/* Call the Dxe Rx Callback */
	if ((NULL != hif_dxeos_ctx) && (
		NULL != hif_dxeos_ctx->dxe_hif_params.dxe_rx_cb))
	{
		hif_dxeos_ctx->dxe_hif_params.dxe_rx_cb(
			hif_dxeos_ctx->dxe_hif_params.pvcontext);
	}

	adf_os_atomic_dec(&hif_dxeos_ctx->ref_count);

	AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));
}

/**
  @brief hif_dxe_os_rx_isr is the interrupt service routine which handles
  the DXE RX Available interrupt

  hif_dxe_os_rx_isr is registered with the Operating System to handle the
  DXE RX Available interrupt during system initalization.  When a DXE
  RX Available interrupt occurs, it is dispatched to the handler which
  had previously been registered via wpalRegisterInterrupt.

  @param  irq:    Enumeration of the interrupt that occurred
  @param  dev_id: User-supplied data passed back via the ISR

  @see    hif_dxe_os_enable_rx_intr

  @return IRQ_HANDLED since it is a dedicated interrupt
 */
irqreturn_t hif_dxe_os_rx_isr(int irq, void* pSrvcCtx)
{
	S_HIF_DXE_OSCONTEXT * hif_dxeos_ctx =
			(S_HIF_DXE_OSCONTEXT *)pSrvcCtx;
	u_int32_t val = 0;

	AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s \n",__FUNCTION__));

	if ((NULL == hif_dxeos_ctx) ||
			(NULL == hif_dxeos_ctx->dxe_hif_params.dxe_rx_cb) ||
			(FALSE == hif_dxeos_ctx->bRxIntEnabled))
	{
		AR_DEBUG_PRINTF(ATH_DEBUG_ERR,
			("%s : Rx Intr Not Initalized or "
				"Enabled \n",__FUNCTION__));

		/* Whether we can return without Setting Processing Bit ???*/
		return IRQ_HANDLED;
	}

	/*
	   Set Interrupt processing bit
	   During this bit set, WLAN HW may not power collapse
	 */
	val = DXE_READ_REG(hif_dxeos_ctx,WLANDXE_INT_MASK_REG_ADDRESS);
	val |= HIFDXE_RX_INTERRUPT_PRO_MASK;
	DXE_WRITE_REG(hif_dxeos_ctx,WLANDXE_INT_MASK_REG_ADDRESS, val);

	/* Disable RX Complete Interrupt at here */
	hif_dxe_os_disable_rx_intr(hif_dxeos_ctx);

	/*  Deferred the processing to Tasklet */
	tasklet_schedule(&hif_dxeos_ctx->hif_dxe_rx_tasklet);

	AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s \n",__FUNCTION__));

	return IRQ_HANDLED;
}

A_STATUS ol_ath_dxe_initialize(void* Adapter)
{
	//gpAdapter = (PADAPTER)Adapter;
	/* To Do */
	return A_OK;
}


/**
 * @ HIF OS Init Dxe
 *
 * @param[in/out] osdev  OS Specific Context
 *
 * @retval    hif_dxe_oshandle Returns OS Specific Allocation Handle
 */
hif_dxe_oshandle hif_dxe_os_init(adf_os_device_t osdev ,
					S_HIF_DXE_OS_PARAMS *hif_dxe_os_params)
{
	S_HIF_DXE_OSCONTEXT * hif_dxeos_ctx = NULL;
	struct device *wcnss_device;

	if (NULL == osdev->dev)
	{
		AR_DEBUG_PRINTF(ATH_DEBUG_ERR,(
			"hif_dxe_os_init : Invalid Dev - Null\n"));
		return NULL;
	}

	/* Get the parent dev  ????????? Need to check */
	wcnss_device = osdev->dev;

	hif_dxeos_ctx = (S_HIF_DXE_OSCONTEXT *)
			adf_os_mem_alloc(osdev,sizeof(S_HIF_DXE_OSCONTEXT));

	if (NULL == hif_dxeos_ctx)
	{
		AR_DEBUG_PRINTF(ATH_DEBUG_ERR,(
			"hif_dxe_os_init : Failed To "
				"Allocate Memory for hif_dxeos_ctx \n"));
		return NULL;
	}

	adf_os_mem_set(hif_dxeos_ctx, 0, sizeof(S_HIF_DXE_OSCONTEXT));
	hif_dxeos_ctx->dxe_unloaded = 0;
	adf_os_mem_copy(&hif_dxeos_ctx->dxe_hif_params,
			hif_dxe_os_params,sizeof(S_HIF_DXE_OS_PARAMS));

	do
	{
		/* Get the WCNSS Memory Map */
		hif_dxeos_ctx->wcnss_memory =
			wcnss_wlan_get_memory_map(wcnss_device);

		if (NULL == hif_dxeos_ctx->wcnss_memory)
		{
			AR_DEBUG_PRINTF(ATH_DEBUG_ERR,(
				"hif_dxe_os_init : WCNSS memory map "
				"unavailable\n"));
			break;
		}

		/* Get the wcnss Tx Irq */
		hif_dxeos_ctx->tx_irq =
				wcnss_wlan_get_dxe_tx_irq(wcnss_device);

		if (0 > hif_dxeos_ctx->tx_irq)
		{
			AR_DEBUG_PRINTF(ATH_DEBUG_ERR,(
				"hif_dxe_os_init : "
					"WCNSS TX IRQ unavailable\n"));
			break;
		}

		/* Get the wcnss Rx Irq */
		hif_dxeos_ctx->rx_irq =
				wcnss_wlan_get_dxe_rx_irq(wcnss_device);

		if (0 > hif_dxeos_ctx->tx_irq)
		{
			AR_DEBUG_PRINTF(ATH_DEBUG_ERR,(
					"hif_dxe_os_init :"
					 "WCNSS RX IRQ unavailable\n"));
			break;
		}

		/*
		   note the we don't invoke request_mem_region().
		   the memory described by wcnss_memory encompases the entire
		   register space (including BT and FM) and we do not want
		   exclusive access to that memory
		 */
		hif_dxeos_ctx->mmio = ioremap(
					hif_dxeos_ctx->wcnss_memory->start,
					resource_size(
						hif_dxeos_ctx->wcnss_memory));

		if (NULL == hif_dxeos_ctx->mmio)
		{
			AR_DEBUG_PRINTF(ATH_DEBUG_ERR,(
				"hif_dxe_os_init : "
				"IO Memory Mapping Failed\n"));
			break;
		}

		/* Initialize Hif Dxe Tx Tasklet */
		tasklet_init(&hif_dxeos_ctx->hif_dxe_tx_tasklet,
				hif_dxe_os_tx_processing,
				(unsigned long)hif_dxeos_ctx);

		/* Initialize  Hif Dxe Rx Tasklet */
		tasklet_init(&hif_dxeos_ctx->hif_dxe_rx_tasklet,
				hif_dxe_os_rx_processing,
				(unsigned long)hif_dxeos_ctx);

		/* successfully allocated environment, memory and IRQs */
		return hif_dxeos_ctx;

	}while(0);

	hif_dxe_os_deinit(hif_dxeos_ctx);

	return NULL;
}


/**
 * @ HIF OS DeInit Dxe
 *
 * @param[in]hif_dxe_osdev  OS Specific Context
 *
 * @retval   void
 */
void hif_dxe_os_deinit(hif_dxe_oshandle hif_dxe_osdev)
{
	S_HIF_DXE_OSCONTEXT * hif_dxeos_ctx = hif_dxe_osdev;

	if ((NULL == hif_dxeos_ctx))
	{
		AR_DEBUG_PRINTF(ATH_DEBUG_ERR,(
			"%s : Invalid DXE Context\n",__FUNCTION__));
		return;
	}

	/* Free up registered Tx IRQ */
	if(hif_dxeos_ctx->bTxRegistered)
	{
		free_irq(hif_dxeos_ctx->tx_irq, hif_dxeos_ctx);

		/* Kill the Tx Tasklet */
		tasklet_kill(&hif_dxeos_ctx->hif_dxe_tx_tasklet);
	}

	/* Free up registered Rx IRQ */
	if(hif_dxeos_ctx->bRxRegistered)
	{
		free_irq(hif_dxeos_ctx->rx_irq, hif_dxeos_ctx);

		/* Kill the Rx Tasklet */
		tasklet_kill(&hif_dxeos_ctx->hif_dxe_rx_tasklet);
	}

	/* Unmap the IO Mapping */
	iounmap(hif_dxeos_ctx->mmio);

	/* Free up the allocated memory */
	adf_os_mem_free(hif_dxeos_ctx);
	hif_dxeos_ctx = NULL;
}

/**
 * @ HIF Dxe OS Specific Initialization
 *
 * @param[in]hif_dxe_osdev - OS Specific Context
 * @param[in]eIntType
 * @param[in]bEnable
 *
 * @retval   A_STATUS
 */
A_STATUS hif_dxe_os_program_int(hif_dxe_oshandle hif_dxe_osdev ,
				E_DXE_INTTYPE eIntType ,
				E_DXE_INT_ACTION eAction)
{
	S_HIF_DXE_OSCONTEXT * hif_dxeos_ctx = hif_dxe_osdev;

	switch (eIntType)
	{
		case E_HIFDXE_TX_INTERRUPT:
			{
				if (eAction == E_HIFDXE_INT_ENABLE)
				{
					if(FALSE ==
						hif_dxe_os_enable_tx_intr
								(hif_dxeos_ctx))
					{
						AR_DEBUG_PRINTF(ATH_DEBUG_ERR,(
							"%s : Enable Tx "
							"Intr Failed\n",__FUNCTION__));
						return A_ERROR;
					}
				}
				else if (eAction == E_HIFDXE_INT_DISABLE)
				{
					if(FALSE ==
						hif_dxe_os_disable_tx_intr
								(hif_dxeos_ctx))
					{
						AR_DEBUG_PRINTF(ATH_DEBUG_ERR,(
							"%s : Disable Tx "
							"Intr Failed\n",__FUNCTION__));
						return A_ERROR;
					}
				}
				else if (eAction == E_HIFDXE_INT_ACTIVATE)
				{
					/* To Do */
					AR_DEBUG_PRINTF(ATH_DEBUG_ERR,(
						"%s : E_HIFDXE_INT_ACTIVATE "
						"Not Supported\n",__FUNCTION__));
				}
				else if (eAction == E_HIFDXE_INT_DEACTIVATE)
				{
					/* To Do */
					AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("%s : "
						"E_HIFDXE_INT_DEACTIVATE "
						"Not Supported\n",__FUNCTION__));
				}
			}
			break;
		case E_HIFDXE_RX_INTERRUPT:
			{
				if (eAction == E_HIFDXE_INT_ENABLE)
				{
					if(FALSE ==
						hif_dxe_os_enable_rx_intr(
								hif_dxeos_ctx))
					{
						return A_ERROR;
					}
				}
				else if (eAction == E_HIFDXE_INT_DISABLE)
				{
					if(FALSE ==
						hif_dxe_os_disable_rx_intr(
								hif_dxeos_ctx))
					{
						return A_ERROR;
					}
				}
				else if (eAction == E_HIFDXE_INT_ACTIVATE)
				{
					/* To Do */
					AR_DEBUG_PRINTF(ATH_DEBUG_ERR,(
						"%s : E_HIFDXE_INT_ACTIVATE "
						"Not Supported\n",__FUNCTION__));
				}
				else if (eAction == E_HIFDXE_INT_DEACTIVATE)
				{
					/* To Do */
					AR_DEBUG_PRINTF(ATH_DEBUG_ERR,(
						"%s : E_HIFDXE_INT_DEACTIVATE "
						"Not Supported\n",__FUNCTION__));
				}
			}
			break;
	}
	return A_OK;
}

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
				u_int32_t clrSt, u_int32_t setSt)
{
	int rc;

	AR_DEBUG_PRINTF(ATH_DEBUG_TRC, (
			"+%s Clear : 0x%x Set : 0x%x\n",__FUNCTION__,(
				unsigned int)clrSt,(unsigned int)setSt));

	rc = smsm_change_state(SMSM_APPS_STATE, clrSt, setSt);

	if(0 != rc)
	{
		AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("%s : smsm_change_state Failed\n"
				,__FUNCTION__));
		return A_ERROR;
	}

	return A_OK;
}

/**
 * @brief HIF OS Read register
 *
 * @param[in] OS Context
 * @param[in] Addr
 *
 * @return Read Value
 */
u_int32_t hif_dxe_os_readreg(hif_dxe_oshandle hif_dxe_osdev, u_int32_t addr)
{
	u_int32_t val = 0;
	S_HIF_DXE_OSCONTEXT * hif_dxeos_ctx = hif_dxe_osdev;

	AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s osdev : 0x%x mem : 0x%x addr : "
			"0x%x\n",__FUNCTION__,(unsigned int)hif_dxeos_ctx,
			(unsigned int)(hif_dxeos_ctx->mmio),(
			unsigned int)addr));

	val = readl_relaxed(hif_dxeos_ctx->mmio + (
					addr - WLANDXE_RIVA_BASE_ADDRESS));
	rmb();

	AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));

	return val;
}

/**
 * @brief HIF OS Write register
 *
 * @param[in] OS Context
 * @param[in] Addr
 * @param[in] val
 *
 * @return Read Value
 */
void hif_dxe_os_writereg(hif_dxe_oshandle hif_dxe_osdev,
				u_int32_t addr, u_int32_t val)
{
	S_HIF_DXE_OSCONTEXT * hif_dxeos_ctx = hif_dxe_osdev;

	AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s osdev : 0x%x mem : 0x%x "
			"addr : 0x%x\n",__FUNCTION__,(unsigned int)hif_dxeos_ctx,
			(unsigned int)hif_dxeos_ctx->mmio,(unsigned int)addr));

	wmb();
	writel_relaxed(val, hif_dxeos_ctx->mmio +
					(addr - WLANDXE_RIVA_BASE_ADDRESS));

	AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));
}

/**
 * @brief HIF Mem Barrier
 *
 * @param[in] void
 *
 * @return void
 */
void hif_dxe_os_mem_barrier(void)
{
	/* Memory Barrier */
	mb();
}

#define DXE_STOP_WAIT_CNT 10
#define DXE_STOP_WAIT_SLEEP_UNIT_MS 100
/**
 * @brief Notify HIF DXE OS Layer Of Stop
 *
 * @param[in] OS Context
 *
 * @return void
 */
void hif_dxe_os_stop(hif_dxe_oshandle hif_dxe_osdev)
{
	S_HIF_DXE_OSCONTEXT * hif_dxeos_ctx = hif_dxe_osdev;
	u_int8_t max_wait_cnt = DXE_STOP_WAIT_CNT;

	/* Wait For DXE Reference to complete (1 sec max)*/
	while (adf_os_atomic_read(&(hif_dxeos_ctx->ref_count))
		&& (max_wait_cnt)) {
		vos_sleep(DXE_STOP_WAIT_SLEEP_UNIT_MS);
		max_wait_cnt--;
	}
}

/**
 * @brief  HIF DXE OS Layer Dump for Debug
 *
 * @param[in] OS Context
 *
 * @return void
 */
void hif_dxe_os_dbgdump(hif_dxe_oshandle hif_dxe_osdev)
{
	S_HIF_DXE_OSCONTEXT * hif_dxeos_ctx = hif_dxe_osdev;
	AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("hif_dxeos_ctx->mem : %x \n",(
					unsigned int)hif_dxeos_ctx->mmio));
	AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("hif_dxeos_ctx->mem_phy_start : %x \n",(
			unsigned int)hif_dxeos_ctx->wcnss_memory->start));
	AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("hif_dxeos_ctx->mem_phy_end : %x \n",(
			unsigned int)hif_dxeos_ctx->wcnss_memory->end));
}
