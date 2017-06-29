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
* @file hif_dxe.c
* @brief Provide functions for DXE HIF module for RIVA FW.
* @details
*  This file Implements the HIF DXE Module For RIVA Integrated WLAN Platform  .
*  DXE software module communicates with the RIVA DXE HW block for data path which
*  is a DMA engine to transfer Data from Host DDR to Target DDR.
*  Provides API to :
*  - Initialize the DMA block registers, Allocate DMA Memory for Descriptors/Rx Network Packets.
*  - Send Recieve Network Packets over the DXE.
*  - Register Tx Complete, Rx and Low Resource Callbacks.
*  - Send Tx Data packets and report Rx Packets
*  - Report Available Free Slots in Tx Ring
*  - Flush Tx Packets
*  - Set DXE Power State when Host  transitions in and out of Power Save
*  - Stop and Deallocate resources on Driver Unloads
*/
#include "adf_os_types.h"
#include "adf_os_lock.h"
#include "adf_os_time.h"
#include "hif_dxe.h"
#include "hif_dxe_hw_pvt.h"
#include "hif_dxe_pvt.h"
#include "hif_dxe_os.h"
#define ATH_MODULE_NAME hif_dxe
#include <a_debug.h>

S_HIFDXE_CONTEXT * g_hif_dxe_ctx = NULL;

#define HIF_DXE_DEBUG   ATH_DEBUG_MAKE_MODULE_MASK(0)

#if defined(DEBUG)
static ATH_DEBUG_MASK_DESCRIPTION g_HIFDebugDescription[] = {
    {HIF_DXE_DEBUG,"hif_dxe"},
};


ATH_DEBUG_INSTANTIATE_MODULE_VAR(hif_dxe,
                                 "hif_dxe",
                                 "DXE HIF Host Interface",
                                 ATH_DEBUG_MASK_DEFAULTS | ATH_DEBUG_INFO | ATH_DEBUG_ERR | HIF_DXE_DEBUG,
                                 ATH_DEBUG_DESCRIPTION_COUNT(g_HIFDebugDescription),
                                 g_HIFDebugDescription);
#endif

//externs
extern A_STATUS dxe_cmn_def_config(S_HIFDXE_CONTEXT * dxe_ctx);
extern A_STATUS dxe_chan_def_config(S_HIFDXE_CONTEXT * pdxectx,WLANDXE_ChannelCBType   *channelEntry);

//Local Variables
static char *channelType[HIFDXE_CHANNEL_MAX] =
{
    "TX_LOW_PRI",
    "TX_HIGH_PRI",
    "RX_LOW_PRI",
    "RX_HIGH_PRI",
};

//Local Functions

A_STATUS hif_dxe_notify_smsm(S_HIFDXE_CONTEXT *dxe_ctx,u_int8_t kickDxe,u_int8_t ringEmpty)
{
   u_int32_t clrSt = 0;
   u_int32_t setSt = 0;

   if(kickDxe)
   {
     AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s Set Kick off DXE\n",__FUNCTION__));
     setSt |= HIFDXE_SMSM_WLAN_TX_ENABLE;
   }
   else
   {
     AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s Clear Kick off DXE\n",__FUNCTION__));
     clrSt |= HIFDXE_SMSM_WLAN_TX_ENABLE;
   }

   if(ringEmpty)
   {
     AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s SMSM Tx Ring Empty\n",__FUNCTION__));
     clrSt |= HIFDXE_SMSM_WLAN_TX_RINGS_NOT_EMPTY;
   }
   else
   {
     AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s SMSM Tx Ring Not Empty\n",__FUNCTION__));
     setSt |= HIFDXE_SMSM_WLAN_TX_RINGS_NOT_EMPTY;
   }

   AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s SMSM C : %x S : %x\n",__FUNCTION__,clrSt,setSt));

   hif_dxe_os_notifysmsm(dxe_ctx->hif_os_handle,clrSt, setSt);

   return A_OK;
}


void dxe_ps_complete(S_HIFDXE_CONTEXT *dxe_ctx, u_int8_t intr_based)
{

}

void dxe_channel_clean_int(S_HIFDXE_CONTEXT *dxe_ctx,WLANDXE_ChannelCBType *channelEntry,u_int32_t *chstatus)
{

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));

    /* Read Channel Status Register to know why INT Happen */
    *chstatus = hif_dxe_os_readreg(dxe_ctx->hif_os_handle,channelEntry->channelRegister.chDXEStatusRegAddr);

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC,(" dxe_channel_clean_int : Channel : %d Channel Status 0x%x \n",channelEntry->channelType, *chstatus));

    /* Clean up all the INT within this channel */
    hif_dxe_os_writereg(dxe_ctx->hif_os_handle,WLANDXE_INT_CLR_ADDRESS,(1 << channelEntry->assignedDMAChannel));


    /* Clean up Error INT Bit */
    if (WLANDXE_CH_STAT_INT_ERR_MASK & *chstatus)
    {
        hif_dxe_os_writereg(dxe_ctx->hif_os_handle,WLANDXE_INT_ERR_CLR_ADDRESS,(1 << channelEntry->assignedDMAChannel));

    }

    /* Clean up DONE INT Bit */
    if (WLANDXE_CH_STAT_INT_DONE_MASK & *chstatus)
    {
        hif_dxe_os_writereg(dxe_ctx->hif_os_handle,WLANDXE_INT_DONE_CLR_ADDRESS,(1 << channelEntry->assignedDMAChannel));
    }

    /* Clean up ED INT Bit */
    if (WLANDXE_CH_STAT_INT_ED_MASK & *chstatus)
    {
        hif_dxe_os_writereg(dxe_ctx->hif_os_handle,WLANDXE_INT_ED_CLR_ADDRESS,(1 << channelEntry->assignedDMAChannel));
    }

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));
}

A_STATUS dxe_tx_pull_frames(S_HIFDXE_CONTEXT *dxe_ctx, WLANDXE_ChannelCBType *channelEntry, u_int8_t flush)
{
    A_STATUS                status = A_OK;
    WLANDXE_DescCtrlBlkType  *currentCtrlBlk = NULL;
    volatile WLANDXE_DescType *currentDesc    = NULL;
    u_int32_t                   descCtrlValue  = 0;
    u_int32_t                  *lowThreshold   = NULL;
    adf_nbuf_t                txframe_head   = NULL;
    adf_nbuf_t                txframe_cur   = NULL;


    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));


    /* Sanity Check */
    if ((NULL == dxe_ctx) || (NULL == channelEntry))
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("dxe_channel_start :  dxe ctx/ Channel Entry is not valid"));
        return A_EINVAL;
    }

    channelEntry->txCompIntTracing |= 0x080000;

    if (NULL == dxe_ctx->hif_client_cb.HifTxCompleteCb)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("dxe_tx_complete_frames HifTxCompleteCb is not registered"));
        channelEntry->txIntTracing |= 0x080000;
        adf_os_assert(0);
        return A_ERROR;
    }
    //FIXME_RT The Spin Lock must be held only for DMA Channel operation and client cb must not be invoked under this spin lock
    adf_os_spin_lock(&channelEntry->dxeChannelLock);

    currentCtrlBlk = channelEntry->tailCtrlBlk;
    currentDesc    = currentCtrlBlk->linkedDesc;
    if ( flush )
    {
        currentDesc->descCtrl.valid = 0;
    }

    channelEntry->txCompIntTracing |= 0x100000;
    channelEntry->txIntTracing |= 0x4000;
    if ( currentCtrlBlk == channelEntry->headCtrlBlk )
    {
        adf_os_spin_unlock(&channelEntry->dxeChannelLock);
        return A_OK;
    }


    while (1)
    {
        //      HDXE_ASSERT(WLAN_PAL_IS_STATUS_SUCCESS(WLAN_RivaValidateDesc(currentDesc)));
        descCtrlValue = currentDesc->descCtrl.ctrl;
        if ((descCtrlValue & WLANDXE_DESC_CTRL_VALID))
        {
            /* caught up with head, bail out */
            AR_DEBUG_PRINTF(ATH_DEBUG_TRC,("dxe_tx_complete_frames caught up with head - next DESC has VALID set \n"));
            channelEntry->txIntTracing |= 0x010000;
            channelEntry->txCompIntTracing |= 0x200000;
            break;
        }
        channelEntry->txIntTracing |= 0x8000;
        channelEntry->txCompIntTracing |= 0x400000;

        adf_os_assert(currentCtrlBlk->xfrFrame != NULL);
        adf_os_atomic_inc(&channelEntry->numFreeDesc);
        adf_os_atomic_dec(&channelEntry->numRsvdDesc);

        /* Send Frame TX Complete notification with frame start fragment location */
        if (WLANDXE_U32_SWAP_ENDIAN(descCtrlValue) & WLANDXE_DESC_CTRL_EOP)
        {
            adf_os_atomic_dec(&dxe_ctx->tx_pkts_pending);
            adf_os_atomic_dec(&channelEntry->tx_pkts_pending);
            adf_os_atomic_inc(&dxe_ctx->tx_pk_count);
            channelEntry->txIntTracing |= 0x200000;
            channelEntry->txCompIntTracing |= 0x01000000;
			//Link Completed Tx packet into Network Packet List to return to Client Tx Complete handler
            if (NULL == txframe_head)
            {
                txframe_head = currentCtrlBlk->xfrFrame;
	        }
            else
            {
                //Link this frame to txframe_head
                adf_nbuf_set_next(txframe_cur,currentCtrlBlk->xfrFrame);
            }
            txframe_cur   = currentCtrlBlk->xfrFrame;

		    adf_os_atomic_set(&channelEntry->numFragmentCurrentChain, 0);

        }
        currentCtrlBlk = currentCtrlBlk->nextCtrlBlk;
        currentDesc    = currentCtrlBlk->linkedDesc;

        /* Break condition
        * Head control block is the control block must be programed for the next TX
        * so, head control block is not programmed control block yet
        * if loop encounte head control block, stop to complete
        * in theory, COMP CB must be called already ??? */
        if (currentCtrlBlk == channelEntry->headCtrlBlk)
        {
            AR_DEBUG_PRINTF(ATH_DEBUG_TRC,("dxe_tx_complete_frames: caught up with head ptr \n"));
            channelEntry->txIntTracing |= 0x040000;
            channelEntry->txCompIntTracing |= 0x02000000;
            break;
        }
        /* VALID Bit check ???? */
    }

    /* Tail and Head Control block must be same */
    channelEntry->tailCtrlBlk = currentCtrlBlk;
    adf_os_spin_unlock(&channelEntry->dxeChannelLock);

    //Invoke Client Tx Complete Handler with completed Tx packets
    if (txframe_head)
    {
        adf_nbuf_set_next(txframe_cur,NULL);
        dxe_ctx->hif_client_cb.HifTxCompleteCb(dxe_ctx->hif_client_cb.HifTxCompleteCtx,txframe_head,channelEntry->channelType,A_OK);
    }

    lowThreshold = channelEntry->channelType == HIFDXE_CHANNEL_TX_LOW_PRI?
        &(dxe_ctx->txCompInt.txLowResourceThreshold_LoPriCh):
    &(dxe_ctx->txCompInt.txLowResourceThreshold_HiPriCh);

    /* If specific channel hit low resource condition send notification to upper layer */
    if ((TRUE == channelEntry->hitLowResource) &&
        (adf_os_atomic_read(&channelEntry->numFreeDesc) > *lowThreshold))
    {
        /* Change it back if we raised it for fetching a remaining packet from TL */
        if (WLANDXE_TX_LOW_RES_THRESHOLD > *lowThreshold)
        {
            *lowThreshold = WLANDXE_TX_LOW_RES_THRESHOLD;
        }
        AR_DEBUG_PRINTF(ATH_DEBUG_TRC,("dxe_tx_complete_frames: recovered from TX low res with free desc %d \n",adf_os_atomic_read(&channelEntry->numFreeDesc)));

        if (dxe_ctx->hif_client_cb.HifLowResourceCb)
        {
            dxe_ctx->hif_client_cb.HifLowResourceCb(dxe_ctx->hif_client_cb.HifLowResourceCtx,
                channelEntry->channelType,
                FALSE);
        }
        channelEntry->hitLowResource = FALSE;
    }

    channelEntry->txIntTracing |= 0x100000;
    channelEntry->txCompIntTracing |= 0x04000000;
    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));
    return status;
}

A_STATUS dxe_tx_complete_handler(void *pvcontext)
{
    S_HIFDXE_CONTEXT * pdxectx =(S_HIFDXE_CONTEXT *) pvcontext;
    A_STATUS                status  = A_OK;
    u_int32_t                intSrc     = 0;
    u_int32_t                chStat     = 0;
    WLANDXE_ChannelCBType    *channelCb  = NULL;

    u_int8_t                    bEnableISR = FALSE;
    static u_int32_t          successiveIntWithIMPS;
    u_int8_t idx = 0;

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));

    /* Return from here if the RIVA is in IMPS, to avoid register access */
    if (HIF_DXE_POWER_STATE_DOWN == pdxectx->hostPowerState)
    {
        /* Disable interrupt at here,
        IMPS or IMPS Pending state should not access RIVA register */
        pdxectx->txIntDisabledByIMPS = TRUE;
        AR_DEBUG_PRINTF(ATH_DEBUG_TRC,("dxe_tx_complete_handler : Host Power State is %d , Return From Here \n",pdxectx->hostPowerState));
        return status;
    }

    /* Return from here if the RIVA is in IMPS, to avoid register access */
    if (HIF_DXE_POWER_STATE_IMPS == pdxectx->hostPowerState)
    {
        successiveIntWithIMPS++;
        AR_DEBUG_PRINTF(ATH_DEBUG_TRC,("dxe_tx_complete_handler IMPS TX COMP INT successiveIntWithIMPS %d \n", successiveIntWithIMPS));
        status = dxe_tx_pull_frames(pdxectx, &pdxectx->dxeChannel[HIFDXE_CHANNEL_TX_HIGH_PRI], FALSE);
        if (A_OK != status)
        {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("dxe_tx_complete_handler Tx Completion Handling Failed on Channel : %d \n",HIFDXE_CHANNEL_TX_HIGH_PRI));
        }

        status = dxe_tx_pull_frames(pdxectx, &pdxectx->dxeChannel[HIFDXE_CHANNEL_TX_LOW_PRI], FALSE);
        if (A_OK != status)
        {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("dxe_tx_complete_handler Tx Completion Handling Failed on Channel : %d \n",HIFDXE_CHANNEL_TX_HIGH_PRI));
        }

        if (adf_os_atomic_read(&pdxectx->tx_pkts_pending) && (successiveIntWithIMPS == 1))
        {
            status = hif_dxe_os_program_int(pdxectx->hif_os_handle,E_HIFDXE_TX_INTERRUPT,E_HIFDXE_INT_ENABLE);
            if (A_OK != status)
            {
                AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("dxe_tx_complete_handler : Enable TX complete interrupt FAIL! "));
                return status;
            }

            AR_DEBUG_PRINTF(ATH_DEBUG_TRC,("dxe_tx_complete_handler TX COMP INT Enabled, remain TX frame count on ring %d\n",  adf_os_atomic_read(&pdxectx->tx_pkts_pending)));
            /*Kicking the DXE after the TX Complete interrupt was enabled - to avoid
            the posibility of a race*/
            dxe_ps_complete(pdxectx, TRUE);
        }
        else
        {
		    AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("dxe_tx_complete_handler TX COMP INT NOT Enabled, RIVA still wake up? remain TX frame count on ring %d, successiveIntWithIMPS %d",
			    adf_os_atomic_read(&pdxectx->tx_pkts_pending), successiveIntWithIMPS));

        }
        return status;
    }

    successiveIntWithIMPS = 0;

    /* Disable device interrupt */
    /* Read whole interrupt mask register and exclusive only this channel int */
    intSrc = hif_dxe_os_readreg(pdxectx->hif_os_handle,WLANDXE_INT_SRC_RAW_ADDRESS);
    AR_DEBUG_PRINTF(ATH_DEBUG_TRC,("dxe_tx_complete_handler Int Src 0x%x\n", intSrc));

    /* Test High Priority Channel is the INT source or not */
    for (idx = HIFDXE_CHANNEL_TX_LOW_PRI; idx <= HIFDXE_CHANNEL_TX_HIGH_PRI ;idx++)
    {
        channelCb = &pdxectx->dxeChannel[idx];
        if (intSrc & (1 << channelCb->assignedDMAChannel))
        {
            dxe_channel_clean_int(pdxectx,channelCb,&chStat);

            if (WLANDXE_CH_STAT_INT_ERR_MASK & chStat)
            {
                AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("dxe_tx_complete_handler : Fatal Error DXE Channel : %d ChanState Invalid : %d",idx,chStat));
                adf_os_assert(0);
            }
            else if ((WLANDXE_CH_STAT_INT_DONE_MASK & chStat) || (WLANDXE_CH_STAT_INT_ED_MASK & chStat))
            {
                /* Handle TX complete for high priority channel */
                status = dxe_tx_pull_frames(pdxectx,channelCb, FALSE);
                if (A_OK != status)
                {
                    AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("dxe_tx_complete_handler Tx Completion Handling Failed on Channel : %d \n",channelCb->channelType));
                }
                bEnableISR = TRUE;
            }
            else
            {
                AR_DEBUG_PRINTF(ATH_DEBUG_TRC,("dxe_tx_complete_handler DXE Channel : %d ChanState = %x\n",idx,chStat));
            }

            if (WLANDXE_CH_STAT_MASKED_MASK & chStat)
            {
                AR_DEBUG_PRINTF(ATH_DEBUG_TRC,("dxe_tx_complete_handler DXE Channel : %d Masked Unmask it ChanState : %x!!\n",idx,chStat));
            }
            AR_DEBUG_PRINTF(ATH_DEBUG_TRC,( "dxe_tx_complete_handler DXE Channel : %d  ChanState = %x RESRVD %d\n",idx,chStat,adf_os_atomic_read(&channelCb->numRsvdDesc)));
        }
    }

    if (bEnableISR || adf_os_atomic_read(&pdxectx->tx_pkts_pending))
    {
        status = hif_dxe_os_program_int(pdxectx->hif_os_handle,E_HIFDXE_TX_INTERRUPT,E_HIFDXE_INT_ENABLE);
        if (A_OK != status)
        {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("dxe_tx_complete_handler : Enable TX complete interrupt FAIL! "));
            return status;
        }

        AR_DEBUG_PRINTF(ATH_DEBUG_TRC,("dxe_tx_complete_handler TX COMP INT Enabled, remaining TX frame count on ring %d\n",  adf_os_atomic_read(&pdxectx->tx_pkts_pending)));
    }

    /*Kicking the DXE after the TX Complete interrupt was enabled - to avoid
    the posibility of a race*/
    dxe_ps_complete(pdxectx, TRUE);

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));
    return status;
}

/* Fill RX ring with adf_nbufs.
* DOES NOT KICK THE HARDWARE. This is the caller's responsibility.
*
* Return TRUE if at least one buffer added to ring.
*/
static u_int8_t dxe_rx_ring_fill_n(S_HIFDXE_CONTEXT *dxe_ctx, WLANDXE_ChannelCBType *channel, int32_t num)
{
    A_STATUS status = A_OK;
    WLANDXE_DescCtrlBlkType *ctrlblk;
    volatile WLANDXE_DescType *desc = NULL;
    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s: num = %d\n",__FUNCTION__, num));

    adf_os_assert(adf_os_atomic_read(&channel->numFreeDesc) >= num);

    adf_os_spin_lock(&channel->dxeChannelLock);

    ctrlblk = channel->tailCtrlBlk;

    while (num > 0) {
        adf_nbuf_t rx_netbuf;
        u_int32_t paddr;

        /* Allocate adf_nbuf and add to rx ring */
        rx_netbuf = adf_nbuf_alloc(dxe_ctx->osdev, HIF_DXE_RX_BUFFER_SIZE, 0, 4, FALSE);
        if (!rx_netbuf) {
            /*
            * Failed to fill it to the desired level -
            * we'll start a timer and try again next time.
            * As long as enough buffers are left in the ring for
            * another A-MPDU rx, no special recovery is needed.
            */
            AR_DEBUG_PRINTF(HIF_DXE_DEBUG,("dxe_rx_ring_fill_n: Failed to Retrieve Free Rx Buffers . Starting Rx Refill  Timer  Num Rx Desc : %d \n",adf_os_atomic_read(&channel->numFreeDesc)));
            adf_os_timer_start(&channel->rx_refill_retry_timer,
                HIF_DXE_RX_RING_REFILL_RETRY_TIME_MS);
            break;
        }

        status = adf_nbuf_map(dxe_ctx->osdev, rx_netbuf, ADF_OS_DMA_FROM_DEVICE);
        if (status != A_OK) {
            adf_nbuf_free(rx_netbuf);
            break;
        }
        paddr = adf_nbuf_get_frag_paddr_lo(rx_netbuf, 0);

        /* Update SW Control Block */
        ctrlblk->xfrFrame = rx_netbuf;

        /* Fill in hardware DXE descriptor */
        desc = ctrlblk->linkedDesc;
        desc->dxedesc.dxe_short_desc.dstMemAddrL = WLANDXE_U32_SWAP_ENDIAN(paddr);
        desc->descCtrl.ctrl = channel->extraConfig.cw_ctrl_read;

        /* Next */
        adf_os_atomic_dec(&channel->numFreeDesc);
        num--;
        ctrlblk = ctrlblk->nextCtrlBlk;
    }

    /* Update tail */
    channel->tailCtrlBlk = ctrlblk;

    adf_os_spin_unlock(&channel->dxeChannelLock);

    if (desc)
    {
        /* Issue a dummy read from the DXE descriptor DDR location to ensure
        * that any posted writes are reflected in memory before DXE looks at
        * the descriptor.
        */
        if (channel->extraConfig.cw_ctrl_read != desc->descCtrl.ctrl)
        {
            //HDXE_ASSERT(0);
        }
    }

    /* MEMORY BARRIER - forces writes to memory before hw is kicked. */
    hif_dxe_os_mem_barrier();

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));

    return desc ? TRUE : FALSE;
}

static A_BOOL dxe_rx_ring_replenish(S_HIFDXE_CONTEXT *dxe_ctx, WLANDXE_ChannelCBType *channel)
{
    A_BOOL new_frame = FALSE;

    if (adf_os_atomic_dec_and_test(&channel->rx_refill_ref_cnt)) {
        int32_t num_to_fill;

        num_to_fill = adf_os_atomic_read(&channel->numFreeDesc);
        new_frame = dxe_rx_ring_fill_n(dxe_ctx, channel, num_to_fill /* okay if <= 0 */);
    }
    adf_os_atomic_inc(&channel->rx_refill_ref_cnt);

    return new_frame;
}

/* RX timer function (per channel) - refill RX DXE ring */
static void dxe_rx_ring_refill_retry(void *arg)
{
    WLANDXE_ChannelCBType *channel = (WLANDXE_ChannelCBType *)arg;
    S_HIFDXE_CONTEXT *dxe_ctx = channel->dxe_ctx;
    u_int8_t new_frame;

    AR_DEBUG_PRINTF(HIF_DXE_DEBUG,("dxe_rx_ring_refill_retry  Before Replenish NumDesc : %d",adf_os_atomic_read(&channel->numFreeDesc)));

    new_frame = dxe_rx_ring_replenish(channel->dxe_ctx, channel);

    AR_DEBUG_PRINTF(HIF_DXE_DEBUG,("dxe_rx_ring_refill_retry  After Replenish NumDesc : %d",adf_os_atomic_read(&channel->numFreeDesc)));

    if (!new_frame)
    {
        return;
    }

    /* New frames on RX ring => kick the hardware */
    switch (dxe_ctx->hostPowerState)
    {
    case HIF_DXE_POWER_STATE_IMPS:
    case HIF_DXE_POWER_STATE_DOWN:
        //PS TEST - will this also kick the DXE RX ring(s)?
        //PS TEST - FIGURE OUT WHAT TO DO HERE.
        dxe_ctx->rxIntDisabledByIMPS = TRUE;
        break;
    case HIF_DXE_POWER_STATE_BMPS:
        //PS TEST - TODO - kick ring.  This is a timer function.
        //The power state if the chip is unknown. Need to do SMSM kick.
        break;
    case HIF_DXE_POWER_STATE_FULL:
        /* Enable RX ring i.e. kick/start the DXE channel */
        hif_dxe_os_writereg(dxe_ctx->hif_os_handle, channel->channelRegister.chDXECtrlRegAddr,
            channel->extraConfig.chan_mask);
        break;
    default:
        adf_os_assert(0);
    }
}

static inline adf_nbuf_t dxe_rx_ring_unlink_frame(S_HIFDXE_CONTEXT *dxe_ctx, WLANDXE_ChannelCBType *channelEntry,
                                                  WLANDXE_DescCtrlBlkType *ctrlblk)
{
    adf_nbuf_t rx_netbuf = ctrlblk->xfrFrame;

    if (rx_netbuf == NULL)
        return NULL;

    adf_nbuf_unmap_single(dxe_ctx->osdev, rx_netbuf, ADF_OS_DMA_FROM_DEVICE);

    ctrlblk->xfrFrame = NULL;
    adf_os_atomic_inc(&channelEntry->numFreeDesc);

    return rx_netbuf;
}

static void dxe_rx_ring_cleanup(S_HIFDXE_CONTEXT *dxe_ctx, WLANDXE_ChannelCBType *channelEntry)
{
    u_int32_t idx;
    WLANDXE_DescCtrlBlkType  *currentCtrlBlk;
    adf_nbuf_t rx_netbuf;

    adf_os_assert((HIFDXE_CHANNEL_RX_LOW_PRI  == channelEntry->channelType) ||
        (HIFDXE_CHANNEL_RX_HIGH_PRI == channelEntry->channelType));

    currentCtrlBlk = channelEntry->headCtrlBlk;

    for (idx = 0; idx < channelEntry->numDesc; idx++)
    {
        rx_netbuf = dxe_rx_ring_unlink_frame(dxe_ctx, channelEntry, currentCtrlBlk);
        if (rx_netbuf)
        {
            adf_nbuf_free(rx_netbuf);
        }

        currentCtrlBlk = currentCtrlBlk->nextCtrlBlk;
    }
}

static adf_nbuf_t dxe_rx_ring_reap(S_HIFDXE_CONTEXT *dxe_ctx, WLANDXE_ChannelCBType *channelEntry)
{
    WLANDXE_DescCtrlBlkType  *ctrlblk;  // sw desc
    volatile WLANDXE_DescType *desc;    // hw desc
    u_int32_t descCtrl;
    adf_nbuf_t rx_buf_list_head = NULL;
    adf_nbuf_t rx_buf_prior = NULL;

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));

    adf_os_spin_lock(&channelEntry->dxeChannelLock);

    ctrlblk = channelEntry->headCtrlBlk;
    desc =  ctrlblk->linkedDesc;
    descCtrl = WLANDXE_U32_SWAP_ENDIAN(desc->descCtrl.ctrl);

    /* Process hardware RX ring */
    while (!(descCtrl & WLANDXE_DESC_CTRL_VALID))
    {
        adf_nbuf_t rx_buf;

        /* Remove RX frame from RX ring */
        rx_buf = dxe_rx_ring_unlink_frame(dxe_ctx, channelEntry, ctrlblk);
        if (rx_buf == NULL)
        {
            break;
        }

        /* Append to list of reaped RX frames */
        if (rx_buf_prior)
        {
            adf_nbuf_set_next(rx_buf_prior, rx_buf);
        }
        else
        {
            rx_buf_list_head = rx_buf;
        }

        rx_buf_prior = rx_buf;
        channelEntry->numTotalFrame++;

        /* Next */
        ctrlblk = ctrlblk->nextCtrlBlk;
        desc    = ctrlblk->linkedDesc;
        descCtrl = WLANDXE_U32_SWAP_ENDIAN(desc->descCtrl.ctrl);
    }

    /* terminate rx buf list */
    if (rx_buf_prior)
    {
        adf_nbuf_set_next(rx_buf_prior, NULL);
    }

    /* update SW RX ring head */
    channelEntry->headCtrlBlk = ctrlblk;

    adf_os_spin_unlock(&channelEntry->dxeChannelLock);

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));

    return rx_buf_list_head;
}

/*
* Attempt to resync SW RX ring head ptr with HW RX ring head.
*
* This is called as part of a software workaround for a hardware bug.
* Sometimes when Riva is powered up, the hardware RX ring head is
* not programmed correctly.
*/
static A_BOOL dxe_rx_ring_resync(S_HIFDXE_CONTEXT *dxe_ctx,
                                 WLANDXE_ChannelCBType *channel)
{
    WLANDXE_DescCtrlBlkType *ctrlblk;
    volatile WLANDXE_DescType *desc;
    u_int32_t desc_ctrl;
    u_int32_t desc_loop;
    u_int8_t invalidated_found = FALSE;

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));

    adf_os_spin_lock(&channel->dxeChannelLock);

    ctrlblk = channel->headCtrlBlk;
    desc    = ctrlblk->linkedDesc;
    desc_ctrl = desc->descCtrl.ctrl;

    AR_DEBUG_PRINTF(ATH_DEBUG_INFO,("dxe_rx_ring_resync: RX ISR called but no frame handled PWS %d, channel %s\n",
        (int)dxe_ctx->hostPowerState, channelType[channel->channelType]));

#if 0 //PS TEST - TODO - debug
    dxeChannelMonitor("RX Ready", channelEntry);
    dxeDescriptorDump(channelEntry, channelEntry->headCtrlBlk->linkedDesc, 0);
    dxeChannelRegisterDump(channelEntry, "RX successive empty interrupt");
    dxeChannelAllDescDump(channelEntry, channelEntry->channelType);
#endif

    /* Abnormal interrupt detected, try to find invalidated descriptor
    * (i.e. hardware completed rx frame)
    * This would be the hardware head ptr.
    */
    for (desc_loop = 0; desc_loop < channel->numDesc; desc_loop++)
    {
        if (!(WLANDXE_U32_SWAP_ENDIAN(desc_ctrl) & WLANDXE_DESC_CTRL_VALID))
        {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("dxe_rx_ring_resync: Found invalidated desc, resync SW and HW\n"));

            channel->headCtrlBlk = ctrlblk;
            invalidated_found = TRUE;
            break;
        }

        ctrlblk = ctrlblk->nextCtrlBlk;
        desc    = ctrlblk->linkedDesc;
        desc_ctrl = desc->descCtrl.ctrl;
    }

    adf_os_spin_unlock(&channel->dxeChannelLock);

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));

    return invalidated_found;
}

/* Return TRUE if RX ring has completed frames to be processed */
static u_int8_t dxe_rx_channel_read_and_clear_intr(S_HIFDXE_CONTEXT *dxe_ctx,
                                                 WLANDXE_ChannelCBType *channel,
                                                 u_int32_t *chstatus)
{
    u_int8_t channel_process = FALSE;

    /* Clear RX interrupt and get channel status */
    dxe_channel_clean_int(dxe_ctx, channel, chstatus);

    if (WLANDXE_CH_STAT_INT_ERR_MASK & *chstatus)
    {
        channel->errorIntCount++;
#if 0   //PS TEST - TODO - debug stuff
        dxeChannelMonitor("RX High Event", channelCb);
        dxeDescriptorDump(channelCb, channelCb->headCtrlBlk->linkedDesc, 0);
        dxeChannelRegisterDump(channelCb, "RX HC error interrupt");
        dxeChannelAllDescDump(channelCb);
#endif
    }
    else if ((WLANDXE_CH_STAT_INT_DONE_MASK | WLANDXE_CH_STAT_INT_ED_MASK | WLANDXE_CH_STAT_MASKED_MASK) & *chstatus)
    {
        channel_process = TRUE;
    }

    return channel_process;
}

A_STATUS dxe_rx_handler(void *pvcontext)
{
    S_HIFDXE_CONTEXT * dxe_ctx =(S_HIFDXE_CONTEXT *) pvcontext;
    WLANDXE_ChannelCBType *channel_array[HIF_DXE_NUM_RX_CHANNEL];
    adf_nbuf_t rx_buf_list_head_array[HIF_DXE_NUM_RX_CHANNEL] = {NULL, NULL};
    u_int32_t intr_source;
    u_int32_t regValue;
    A_STATUS status  = A_OK;
    u_int32_t i;

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));

    channel_array[0] = &dxe_ctx->dxeChannel[HIFDXE_CHANNEL_RX_HIGH_PRI];
    channel_array[1] = &dxe_ctx->dxeChannel[HIFDXE_CHANNEL_RX_LOW_PRI];

    if ((HIF_DXE_POWER_STATE_IMPS == dxe_ctx->hostPowerState) ||
        (HIF_DXE_POWER_STATE_DOWN == dxe_ctx->hostPowerState))
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_WARN,("dxe_rx_handler: Riva is in %d, Just Pull frames without any register touch\n", dxe_ctx->hostPowerState));

        /* Riva powered down - DO NOT TOUCH REGISTERS */

        /* Process RX ring(s) */
        for (i = 0; i < HIF_DXE_NUM_RX_CHANNEL; i++)
        {
            WLANDXE_ChannelCBType *channel = channel_array[i];

            if (!channel->extraConfig.chEnabled)
            {
                continue;
            }

            /* unlink RX bufs from RX ring */
            rx_buf_list_head_array[i] = dxe_rx_ring_reap(dxe_ctx, channel);

            if (rx_buf_list_head_array[i])
            {
                /* Refill RX ring */
                dxe_rx_ring_replenish(dxe_ctx, channel);

                /* Indicate RX bufs */
                dxe_ctx->hif_client_cb.HifRxReadyCb(dxe_ctx->hif_client_cb.HifRxReadyCtx,
                    rx_buf_list_head_array[i],
                    channel_array[i]->channelType);
            }
        }

        /* Interrupt will not enabled at here, it will be enabled at PS mode change */
        //PS TEST - will this also kick the DXE RX ring(s)?  look at patch 1330 code
        dxe_ctx->rxIntDisabledByIMPS = TRUE;

        return status;
    }

    /* Read DXE interrupt source */
    intr_source = hif_dxe_os_readreg(dxe_ctx->hif_os_handle, WLANDXE_INT_SRC_RAW_ADDRESS);
    AR_DEBUG_PRINTF(ATH_DEBUG_TRC,("dxe_rx_handler: Intr Src 0x%x\n", intr_source));

    /* Process RX ring(s) */
    for (i = 0; i < HIF_DXE_NUM_RX_CHANNEL; i++)
    {
        WLANDXE_ChannelCBType *channel = channel_array[i];
        u_int8_t rx_process_channel = channel->extraConfig.chEnabled;
        u_int32_t channelStat;

        rx_process_channel = rx_process_channel && (intr_source & (1 << channel->assignedDMAChannel));

        if (!rx_process_channel)
        {
            continue;
        }

        /* Read and Clear RX Channel interrupts */
        //PS TEST - can this be placed after filling the RX ring???? might be a race between filling ring and hw setting interrupt
        //          might need to clear interrupt before kicking DXE channel
        if (!dxe_rx_channel_read_and_clear_intr(dxe_ctx, channel, &channelStat))
        {
            break;
        }

        /* Reap - unlink RX bufs from RX ring */
        rx_buf_list_head_array[i] = dxe_rx_ring_reap(dxe_ctx, channel);

        /* START - SW WAR
        * Workaround for hw and sw's head ptr out of sync after Riva power up.
        * If two successive RX interrupts with no completed frames in RX ring,
        * try to manually resync SW ring head ptr to HW ring head ptr.
        */
        if (rx_buf_list_head_array[i] == NULL && !channel->rx_prev_intr_completed_frames)
        {
            u_int8_t resync_success = dxe_rx_ring_resync(dxe_ctx, channel);
            if (resync_success)
            {
                rx_buf_list_head_array[i] = dxe_rx_ring_reap(dxe_ctx, channel);
            }
            else
            {
                adf_os_assert(resync_success);
                //PS TEST - TODO - what to do here.
                // rel1.2 code restarts the wlan driver. reset instead?
            }
        }
        channel->rx_prev_intr_completed_frames = (rx_buf_list_head_array[i] != NULL);
        /* END - SW WAR */

        if (rx_buf_list_head_array[i])
        {
            /* Refill RX ring */
            dxe_rx_ring_replenish(dxe_ctx, channel);

            /* Enable RX ring i.e. kick/start the DXE channel */
            hif_dxe_os_writereg(dxe_ctx->hif_os_handle, channel->channelRegister.chDXECtrlRegAddr,
                channel->extraConfig.chan_mask);
        }
    }

    /* Indicate RX bufs */
    for (i = 0; i < HIF_DXE_NUM_RX_CHANNEL; i++)
    {
        if (rx_buf_list_head_array[i])
        {
            dxe_ctx->hif_client_cb.HifRxReadyCb(dxe_ctx->hif_client_cb.HifRxReadyCtx,
                rx_buf_list_head_array[i],
                channel_array[i]->channelType);
        }
    }

    /* Re-enable rx interrupts */
    status = hif_dxe_os_program_int(dxe_ctx->hif_os_handle, E_HIFDXE_RX_INTERRUPT, E_HIFDXE_INT_ENABLE);
    if (A_OK != status)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("dxe_rx_handler: Enable RX Ready interrupt fail\n"));
        return status;
    }

    /* Let Riva go back to sleep */
    //PS TEST - TODO - this needs to be synchronized with ISR
    regValue = hif_dxe_os_readreg(dxe_ctx->hif_os_handle, WLANDXE_INT_MASK_REG_ADDRESS);
    regValue &= HIFDXE_RX_INTERRUPT_PRO_UNMASK;
    hif_dxe_os_writereg(dxe_ctx->hif_os_handle, WLANDXE_INT_MASK_REG_ADDRESS, regValue);

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));
	return status;
}

A_STATUS dxe_dma_core_start(S_HIFDXE_CONTEXT *dxe_ctx)
{
    A_STATUS  status = A_OK;
    u_int32_t  registerData = 0;

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));

//    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("dxe_ctx->hif_os_handle : %x \n",dxe_ctx->hif_os_handle));

    hif_dxe_os_dbgdump(dxe_ctx->hif_os_handle);

    /* START This core init is not needed for the integrated system */
    /* Reset First */
    registerData = WLANDXE_DMA_CSR_RESET_MASK;
    registerData = hif_dxe_os_readreg(dxe_ctx->hif_os_handle,WALNDEX_DMA_CSR_ADDRESS);
    registerData  = WLANDXE_DMA_CSR_EN_MASK;
    registerData |= WLANDXE_DMA_CSR_ECTR_EN_MASK;
    registerData |= WLANDXE_DMA_CSR_TSTMP_EN_MASK;
    registerData |= WLANDXE_DMA_CSR_H2H_SYNC_EN_MASK;
    //FIXME_RT Not Sure why this is hardcoded . Check with Danlin
    registerData = 0x00005c89;
    hif_dxe_os_writereg(dxe_ctx->hif_os_handle,WALNDEX_DMA_CSR_ADDRESS,registerData);

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));
    return status;
}

A_STATUS dxe_channel_start(S_HIFDXE_CONTEXT *dxe_ctx, WLANDXE_ChannelCBType *channelEntry)
{
    A_STATUS                status     = A_OK;
    u_int32_t                regValue   = 0;
    u_int32_t                intMaskVal = 0;
    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));

    /* Sanity Check */
    if ((NULL == dxe_ctx) || (NULL == channelEntry))
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("dxe_channel_start :  dxe ctx/ Channel Entry is not valid"));
        return A_EINVAL;
    }

    /* Program Source address and destination adderss */
    if (!channelEntry->channelConfig.useShortDescFmt)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("dxe_channel_start Long Descriptor not support yet"));
        adf_os_assert(0);
        return A_ERROR;
    }

    /* Common register area */
    /* Next linked list Descriptor pointer */
    hif_dxe_os_writereg(dxe_ctx->hif_os_handle,channelEntry->channelRegister.chDXEDesclRegAddr,channelEntry->headCtrlBlk->linkedDescPhyAddr);

    if ((HIFDXE_CHANNEL_TX_LOW_PRI  == channelEntry->channelType) ||
        (HIFDXE_CHANNEL_TX_HIGH_PRI == channelEntry->channelType))
    {
        /* Program default registers */
        /* TX DMA channel, DMA destination address is work Q */
        hif_dxe_os_writereg(dxe_ctx->hif_os_handle,channelEntry->channelRegister.chDXEDadrlRegAddr,channelEntry->channelConfig.refWQ);
    }
    else if ((HIFDXE_CHANNEL_RX_LOW_PRI  == channelEntry->channelType) ||
        (HIFDXE_CHANNEL_RX_HIGH_PRI == channelEntry->channelType))
    {
        /* RX DMA channel, DMA source address is work Q */
        hif_dxe_os_writereg(dxe_ctx->hif_os_handle,channelEntry->channelRegister.chDXESadrlRegAddr,channelEntry->channelConfig.refWQ);

        /* RX DMA channel, Program pre allocated destination Address */
        hif_dxe_os_writereg(dxe_ctx->hif_os_handle,channelEntry->channelRegister.chDXEDadrlRegAddr,
            WLANDXE_U32_SWAP_ENDIAN(channelEntry->DescBottomLoc->dxedesc.dxe_short_desc.phyNextL));

        /* RX Channels, default Control registers MUST BE ENABLED */
        hif_dxe_os_writereg(dxe_ctx->hif_os_handle,channelEntry->channelRegister.chDXECtrlRegAddr,
            channelEntry->extraConfig.chan_mask);
    }


    channelEntry->extraConfig.chEnabled    = TRUE;
    channelEntry->extraConfig.chConfigured = TRUE;

    /* Enable individual channel
    * not to break current channel setup, first read register */
    regValue = hif_dxe_os_readreg(dxe_ctx->hif_os_handle, WALNDEX_DMA_CH_EN_ADDRESS);
    /* Enable Channel specific Interrupt */
    intMaskVal = hif_dxe_os_readreg(dxe_ctx->hif_os_handle, WLANDXE_INT_MASK_REG_ADDRESS);
    intMaskVal |= channelEntry->extraConfig.intMask;
    hif_dxe_os_writereg(dxe_ctx->hif_os_handle,WLANDXE_INT_MASK_REG_ADDRESS,intMaskVal);

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));
    return status;

}

A_STATUS dxe_channel_stop(S_HIFDXE_CONTEXT *dxe_ctx, WLANDXE_ChannelCBType *channelEntry)
{
    A_STATUS                status     = A_OK;
    u_int32_t                intMaskVal = 0;

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));

    /* Sanity Check */
    if ((NULL == dxe_ctx) || (NULL == channelEntry))
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("dxe_channel_start :  dxe ctx/ Channel Entry is not valid"));
        return A_EINVAL;
    }

    /* Maskout interrupt */
    intMaskVal = hif_dxe_os_readreg(dxe_ctx->hif_os_handle,WLANDXE_INT_MASK_REG_ADDRESS);
    intMaskVal ^= channelEntry->extraConfig.intMask;
    hif_dxe_os_writereg(dxe_ctx->hif_os_handle,WLANDXE_INT_MASK_REG_ADDRESS,intMaskVal);

    channelEntry->extraConfig.chEnabled    = FALSE;

    /* Stop Channel ??? */
    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));
    return status;
}



A_STATUS dxe_tx_push_frame(S_HIFDXE_CONTEXT *dxe_ctx, WLANDXE_ChannelCBType *channelEntry, adf_nbuf_t nbuf)
{
    A_STATUS                   status = A_OK;
    WLANDXE_DescCtrlBlkType    *currentCtrlBlk = NULL;
    WLANDXE_DescCtrlBlkType    *startCtrlBlk = NULL;
    volatile WLANDXE_DescType           *currentDesc    = NULL;
    volatile WLANDXE_DescType           *firstDesc      = NULL;
    volatile WLANDXE_DescType           *LastDesc       = NULL;
    adf_os_dma_addr_t          frag_paddr = 0;
    u_int32_t                    frag_bytes = 0;
    u_int32_t                    num_frags;
    u_int32_t                    frag_idx=0;

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));

    if (HIF_DXE_POWER_STATE_BMPS == dxe_ctx->hostPowerState)
    {
        dxe_ctx->fwPowerState = HIF_DXE_FW_POWER_STATE_BMPS_UNKNOWN;
        hif_dxe_notify_smsm(dxe_ctx, FALSE, TRUE);
    }

    adf_os_spin_lock(&channelEntry->dxeChannelLock);
    adf_os_atomic_set(&channelEntry->numFragmentCurrentChain, 0);
    num_frags = adf_nbuf_get_num_frags(nbuf);

    currentCtrlBlk = channelEntry->headCtrlBlk;
    startCtrlBlk = channelEntry->headCtrlBlk;

    // Walk through NetBuf Physical Fragments and program DMA Descriptors
    for (frag_idx=0;frag_idx<num_frags;frag_idx++)
    {
        /* Get current descriptor pointer from current control block */
        currentDesc = currentCtrlBlk->linkedDesc;
        if (NULL == firstDesc)
        {
            firstDesc = currentCtrlBlk->linkedDesc;
        }
        /* All control block will have same palPacket Pointer
        * to make logic simpler */
        currentCtrlBlk->xfrFrame = nbuf;

        /* Get next fragment physical address and fragment size
        * if this is the first trial, will get first physical address
        * if no more fragment, Descriptor src address will be set as NULL, OK??? */
        frag_paddr = adf_nbuf_get_frag_paddr_lo(nbuf,frag_idx);
        frag_bytes = adf_nbuf_get_frag_len(nbuf,frag_idx);
        if ((0 == frag_paddr) ||
            (0    == frag_bytes))
        {
            continue;
        }

        /* This is the LAST descriptor valid for this transaction */
        LastDesc    = currentCtrlBlk->linkedDesc;

        /* Program DXE descriptor */
        currentDesc->dxedesc.dxe_short_desc.srcMemAddrL = WLANDXE_U32_SWAP_ENDIAN(frag_paddr);

        /* Just normal data transfer from aCPU Flat Memory to BMU Q */
        if ((HIFDXE_CHANNEL_TX_LOW_PRI  == channelEntry->channelType) ||
            (HIFDXE_CHANNEL_TX_HIGH_PRI == channelEntry->channelType))
        {
            currentDesc->dxedesc.dxe_short_desc.dstMemAddrL =
                WLANDXE_U32_SWAP_ENDIAN(channelEntry->channelConfig.refWQ);
        }
        else
        {
            /* Test specific H2H transfer, destination address already set
            * Do Nothing */
        }
        currentDesc->xfrSize = WLANDXE_U32_SWAP_ENDIAN(frag_bytes);

        /* Program channel control register */
        /* First frame not set VAL bit, why ??? */
        if (0 == adf_os_atomic_read(&channelEntry->numFragmentCurrentChain))
        {
            currentDesc->descCtrl.ctrl = channelEntry->extraConfig.cw_ctrl_write;
        }
        else
        {
            currentDesc->descCtrl.ctrl = channelEntry->extraConfig.cw_ctrl_write_valid;
        }

        /* Update statistics */
        adf_os_atomic_inc(&channelEntry->numFragmentCurrentChain);
        adf_os_atomic_dec(&channelEntry->numFreeDesc);
        adf_os_atomic_inc(&channelEntry->numRsvdDesc);

        /* Get next control block */
        currentCtrlBlk = currentCtrlBlk->nextCtrlBlk;
    }

    channelEntry->numTotalFrame++;
    AR_DEBUG_PRINTF(ATH_DEBUG_TRC,("dxe_tx_push_frame NUM TX FRAG %d, Total Frame %d\n",adf_os_atomic_read(&channelEntry->numFragmentCurrentChain), channelEntry->numTotalFrame));

    /* Program Channel control register
    * Set as end of packet
    * Enable interrupt also for first code lock down
    * performace optimization, this will be revisited */
    if (NULL == LastDesc)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("dxe_tx_push_frame NULL Last Descriptor, broken chain"));
        adf_os_spin_unlock(&channelEntry->dxeChannelLock);
        adf_os_assert(0);
        return A_EFAULT;
    }
    /* Now First one also Valid ????
       * this procedure will prevent over handle descriptor from previous
       * TX trigger */
    firstDesc->descCtrl.ctrl = channelEntry->extraConfig.cw_ctrl_write_valid;
	LastDesc->descCtrl.ctrl  = channelEntry->extraConfig.cw_ctrl_write_eop_int;


    /* Update channel head as next avaliable linked slot */
    channelEntry->headCtrlBlk = currentCtrlBlk;

    adf_os_spin_unlock(&channelEntry->dxeChannelLock);

    //Ensure Above DMA Channel programming is complete before Notify DXE through SMSM
    hif_dxe_os_mem_barrier();

    /* If in BMPS mode no need to notify the DXE Engine, notify SMSM instead */
    if (HIF_DXE_FW_POWER_STATE_BMPS_UNKNOWN == dxe_ctx->fwPowerState)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_TRC,( "dxe_tx_push_frame SMSM_ret LO=%d HI=%d\n", adf_os_atomic_read(&dxe_ctx->dxeChannel[HIFDXE_CHANNEL_TX_LOW_PRI].numRsvdDesc),
            adf_os_atomic_read(&dxe_ctx->dxeChannel[HIFDXE_CHANNEL_TX_HIGH_PRI].numRsvdDesc )));

        hif_dxe_notify_smsm(dxe_ctx, TRUE, FALSE);
        return status;
    }

    /* If DXE use external descriptor, registers are not needed to be programmed
    * Just after finish to program descriptor, tirigger to send */
    if (channelEntry->extraConfig.chan_mask & WLANDXE_CH_CTRL_EDEN_MASK)
    {
        /* Issue a dummy read from the DXE descriptor DDR location to
        ensure that any previously posted write to the descriptor
        completes. */
        if (channelEntry->extraConfig.cw_ctrl_write_valid != firstDesc->descCtrl.ctrl)
        {
            //HDXE_ASSERT(0);
        }

        /* Everything is ready
        * Trigger to start DMA */
        hif_dxe_os_writereg(dxe_ctx->hif_os_handle,channelEntry->channelRegister.chDXECtrlRegAddr,
            channelEntry->extraConfig.chan_mask);

        AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));
        return status;
    }

    /* If DXE not use external descriptor, program each registers */
    /* Circular buffer handle not need to program DESC register???
    * GEN5 code not programed RING buffer case
    * REVISIT THIS !!!!!! */
    if ((HIFDXE_CHANNEL_TX_LOW_PRI  == channelEntry->channelType) ||
        (HIFDXE_CHANNEL_TX_HIGH_PRI == channelEntry->channelType))
    {
        /* Destination address, assigned Work Q */
        hif_dxe_os_writereg(dxe_ctx->hif_os_handle,channelEntry->channelRegister.chDXEDadrlRegAddr,
            channelEntry->channelConfig.refWQ);
        hif_dxe_os_writereg(dxe_ctx->hif_os_handle,channelEntry->channelRegister.chDXEDadrhRegAddr,0);

    }

    /* Program Source address register
    * This address is already programmed into DXE Descriptor
    * But register also upadte */
    hif_dxe_os_writereg(dxe_ctx->hif_os_handle,channelEntry->channelRegister.chDXESadrlRegAddr,
        WLANDXE_U32_SWAP_ENDIAN(firstDesc->dxedesc.dxe_short_desc.srcMemAddrL));

    hif_dxe_os_writereg(dxe_ctx->hif_os_handle,channelEntry->channelRegister.chDXESadrhRegAddr,0);

    /* Linked list Descriptor pointer */
    hif_dxe_os_writereg(dxe_ctx->hif_os_handle,channelEntry->channelRegister.chDXEDesclRegAddr,
        startCtrlBlk->linkedDescPhyAddr);

    hif_dxe_os_writereg(dxe_ctx->hif_os_handle,channelEntry->channelRegister.chDXEDeschRegAddr,0);

    /* Transfer Size */
    frag_bytes = WLANDXE_U32_SWAP_ENDIAN(firstDesc->xfrSize);
    hif_dxe_os_writereg(dxe_ctx->hif_os_handle,channelEntry->channelRegister.chDXESzRegAddr,
        frag_bytes);


    /* Everything is ready
    * Trigger to start DMA */
    hif_dxe_os_writereg(dxe_ctx->hif_os_handle,channelEntry->channelRegister.chDXECtrlRegAddr,
        channelEntry->extraConfig.chan_mask);

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));
    return status;

}

A_STATUS dxe_alloc_dma_ring(S_HIFDXE_CONTEXT *dxe_ctx, WLANDXE_ChannelCBType *channelEntry)
{
    A_STATUS                status = A_OK;
    u_int32_t                idx;
    WLANDXE_DescCtrlBlkType  *currentCtrlBlk = NULL;
    WLANDXE_DescCtrlBlkType  *prevCtrlBlk = NULL;
//    WLANDXE_DescCtrlBlkType  *nextCtrlBlk = NULL;
    volatile WLANDXE_DescType         *currentDesc = NULL;
    volatile WLANDXE_DescType         *prevDesc    = NULL;
    adf_os_dma_addr_t         physAddress;
//    adf_os_dma_addr_t         desc_pool_paddr;

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));

    /* Sanity check */
    if ((NULL == dxe_ctx) || (NULL == channelEntry))
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("dxe_alloc_dmaring :  Channel Entry / DXE Ctx is not valid \n"));
        return A_EINVAL;
    }

    //Allocate the Control Block Mem Pool
    channelEntry->ctrlblk_mem_pool = adf_os_mem_alloc(dxe_ctx->osdev,sizeof(WLANDXE_DescCtrlBlkType) * channelEntry->numDesc);
    if (NULL == channelEntry->ctrlblk_mem_pool)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("dxe_alloc_dmaring : ctrlblk_mem_pool Allocation Failed for channel %d",channelEntry->channelType));
        return A_NO_MEMORY;
    }
    adf_os_mem_set(channelEntry->ctrlblk_mem_pool, 0, sizeof(WLANDXE_DescCtrlBlkType)* channelEntry->numDesc);
    currentCtrlBlk = (WLANDXE_DescCtrlBlkType *)channelEntry->ctrlblk_mem_pool;

    //Allocate the Descriptor Block Mem Pool
    channelEntry->descblk_mem_pool.vaddr = adf_os_mem_alloc_consistent(dxe_ctx->osdev,sizeof(WLANDXE_DescType)*channelEntry->numDesc,
        &channelEntry->descblk_mem_pool.paddr,
        adf_os_get_dma_mem_context((&channelEntry->descblk_mem_pool), memctx));
    if (NULL == channelEntry->descblk_mem_pool.vaddr)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("dxe_alloc_dmaring : descblk_mem_pool Allocation Failed for channel %d",channelEntry->channelType));
        return A_NO_MEMORY;
    }
    adf_os_mem_set(channelEntry->descblk_mem_pool.vaddr, 0, sizeof(WLANDXE_DescType)* channelEntry->numDesc);
    currentDesc = (WLANDXE_DescType *)channelEntry->descblk_mem_pool.vaddr;
    physAddress = channelEntry->descblk_mem_pool.paddr;

    AR_DEBUG_PRINTF(HIF_DXE_DEBUG,("dxe_alloc_dmaring : Allocating %d Descriptors For Channel %d\n",channelEntry->numDesc,channelEntry->channelType));
    /* Allocate pre asigned number of control blocks */
    for (idx = 0; idx < channelEntry->numDesc; idx++)
    {
        /* Initialize Control Block elements first */
        currentCtrlBlk->xfrFrame          = NULL;
        currentCtrlBlk->linkedDesc        = NULL;
        currentCtrlBlk->linkedDescPhyAddr = 0;
        currentCtrlBlk->ctrlBlkOrder      = idx;

        //Initialize Desc
        //AR_DEBUG_PRINTF(HIF_DXE_DEBUG,("dxe_alloc_dmaring : Allocated Descriptor %d VA 0x%x, PA 0x%x\n", idx,currentDesc, physAddress));
        if ((HIFDXE_CHANNEL_TX_LOW_PRI == channelEntry->channelType) ||
            (HIFDXE_CHANNEL_TX_HIGH_PRI == channelEntry->channelType))
        {
            currentDesc->descCtrl.ctrl = channelEntry->extraConfig.cw_ctrl_write;
            currentDesc->dxedesc.dxe_short_desc.dstMemAddrL = channelEntry->extraConfig.refWQ_swapped;
        }
        else if ((HIFDXE_CHANNEL_RX_LOW_PRI == channelEntry->channelType) ||
            (HIFDXE_CHANNEL_RX_HIGH_PRI == channelEntry->channelType))
        {
            currentDesc->descCtrl.ctrl = channelEntry->extraConfig.cw_ctrl_read;
            currentDesc->dxedesc.dxe_short_desc.srcMemAddrL = channelEntry->extraConfig.refWQ_swapped;
        }

        currentCtrlBlk->linkedDesc        = currentDesc;
        currentCtrlBlk->linkedDescPhyAddr = physAddress;


        /* This is the first control block allocated
        * Next Control block is not allocated yet
        * head and tail must be first control block */
        if (0 == idx)
        {
            currentCtrlBlk->nextCtrlBlk = NULL;
            channelEntry->headCtrlBlk   = currentCtrlBlk;
            channelEntry->tailCtrlBlk   = currentCtrlBlk;
            currentDesc->dxedesc.dxe_short_desc.phyNextL = 0;
            channelEntry->DescBottomLoc                  = currentDesc;
            channelEntry->descBottomLocPhyAddr           = physAddress;
        }
        /* This is not first, not last control block
        * previous control block may has next linked block */
        else if ((0 < idx) && (idx < (channelEntry->numDesc - 1)))
        {
            prevCtrlBlk->nextCtrlBlk = currentCtrlBlk;
            prevDesc->dxedesc.dxe_short_desc.phyNextL = WLANDXE_U32_SWAP_ENDIAN(physAddress);
        }
        /* This is last control block
        * next control block for the last control block is head, first control block
        * then whole linked list made RING */
        else
        {
            prevCtrlBlk->nextCtrlBlk    = currentCtrlBlk;
            currentCtrlBlk->nextCtrlBlk = channelEntry->headCtrlBlk;
            prevDesc->dxedesc.dxe_short_desc.phyNextL =
                WLANDXE_U32_SWAP_ENDIAN(physAddress);
            currentDesc->dxedesc.dxe_short_desc.phyNextL =
                WLANDXE_U32_SWAP_ENDIAN(channelEntry->headCtrlBlk->linkedDescPhyAddr);
        }

        prevDesc    = currentDesc;
        prevCtrlBlk = currentCtrlBlk;

        // advance to the next pre-allocated descriptor in the chunk
        adf_os_atomic_inc(&channelEntry->numFreeDesc);
        currentDesc++;
        currentCtrlBlk++;
        physAddress = physAddress + sizeof(WLANDXE_DescType);
    }

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));
    return status;
}


A_STATUS dxe_free_dma_ring(S_HIFDXE_CONTEXT *dxe_ctx, WLANDXE_ChannelCBType *channelEntry)
{
//    a_uint32_t idx = 0;
//    WLANDXE_DescCtrlBlkType  *currentCtrlBlk = NULL;

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));

    /* Sanity check */
    if ((NULL == dxe_ctx) || (NULL == channelEntry))
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("dxe_free_dmaring: Channel Entry/dxe ctx is not valid \n"));
        return A_EINVAL;
    }

    // Free The rx Buffer pool
    if ((HIFDXE_CHANNEL_RX_LOW_PRI  == channelEntry->channelType) ||
        (HIFDXE_CHANNEL_RX_HIGH_PRI == channelEntry->channelType))
    {
        dxe_rx_ring_cleanup(dxe_ctx, channelEntry );
    }

    if (channelEntry->ctrlblk_mem_pool)
    {
        adf_os_mem_free(channelEntry->ctrlblk_mem_pool);
        channelEntry->ctrlblk_mem_pool = NULL;
    }

    if (channelEntry->descblk_mem_pool.vaddr)
    {
        adf_os_mem_free_consistent(
            dxe_ctx->osdev,
            sizeof(WLANDXE_DescType)*channelEntry->numDesc, /* pool_size */
            channelEntry->descblk_mem_pool.vaddr,
            channelEntry->descblk_mem_pool.paddr,
            adf_os_get_dma_mem_context((&channelEntry->descblk_mem_pool), memctx));
    }


    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));
    return A_OK;
}

//End Local Functions

/**
* @ HIF DXE Attach . One Time Initialize of DXE HW / Allocate DXE SW Module Context / Alloc DXE Descriptors , Rx Buffers , Control Block /Attach OS Interrupts etc
*
* @param[in]dev - OS Dev Handle
*
* @retval   hif_dxe_handle  Allocates HIF DEX Context Block and returns handle for subsequent calls to DXE
*/
hif_dxe_handle hif_dxe_attach(adf_os_device_t dev)
{
    A_STATUS status = A_OK;
    static S_HIFDXE_CONTEXT * dxe_ctx = NULL;
    u_int32_t            idx;
    WLANDXE_ChannelCBType  *currentChannel = NULL;
    int32_t                     smsmInitState;
    S_HIF_DXE_OS_PARAMS     hif_dxe_os_params;

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));

    if(dxe_ctx)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("%s : Hif Dxe Context Already Allocated. Return Existing.\n",__FUNCTION__));
        adf_os_atomic_inc(&dxe_ctx->ref_count);
        return dxe_ctx;
    }

    /* This is temporary allocation */
    dxe_ctx = adf_os_mem_alloc(dev,sizeof(S_HIFDXE_CONTEXT));
    if (NULL == dxe_ctx)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("WLANDXE_Open Control Block Alloc Fail \n"));
        return NULL;
    }
    adf_os_mem_zero(dxe_ctx, sizeof(S_HIFDXE_CONTEXT));

    adf_os_atomic_init(&dxe_ctx->ref_count);
    adf_os_atomic_inc(&dxe_ctx->ref_count);

    dxe_ctx->osdev = dev;

    hif_dxe_os_params.dxe_tx_cb = dxe_tx_complete_handler;
    hif_dxe_os_params.dxe_rx_cb = dxe_rx_handler;
    hif_dxe_os_params.pvcontext = dxe_ctx;
    dxe_ctx->hif_os_handle = hif_dxe_os_init(dev,&hif_dxe_os_params);
    if (NULL == dxe_ctx->hif_os_handle)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("WLANDXE_Open : hif_dxe_os_init Failure! \n"));
        adf_os_mem_free(dxe_ctx);
        return NULL;
    }

    status = dxe_cmn_def_config(dxe_ctx);
    if (A_OK != status)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("WLANDXE_Open Common Configuration Fail"));
        hif_dxe_detach(dxe_ctx);
        return NULL;
    }

    dxe_ctx->dxeChannel[HIFDXE_CHANNEL_TX_LOW_PRI].channelType = HIFDXE_CHANNEL_TX_LOW_PRI;
    dxe_ctx->dxeChannel[HIFDXE_CHANNEL_TX_HIGH_PRI].channelType = HIFDXE_CHANNEL_TX_HIGH_PRI;
    dxe_ctx->dxeChannel[HIFDXE_CHANNEL_RX_LOW_PRI].channelType = HIFDXE_CHANNEL_RX_LOW_PRI;
    dxe_ctx->dxeChannel[HIFDXE_CHANNEL_RX_HIGH_PRI].channelType = HIFDXE_CHANNEL_RX_HIGH_PRI;

    for (idx = 0; idx < HIFDXE_CHANNEL_MAX; idx++)
    {
        AR_DEBUG_PRINTF(HIF_DXE_DEBUG,("WLANDXE_Open Channel %d Open Start \n", idx));
        currentChannel = &dxe_ctx->dxeChannel[idx];

        /* Config individual channels from channel default setup table */
        status = dxe_chan_def_config(dxe_ctx,
            currentChannel);
        if (A_OK != status)
        {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("WLANDXE_Open Channel Basic Configuration Fail for channel %d", idx));
            hif_dxe_detach(dxe_ctx);
            return NULL;
        }

        adf_os_spinlock_init(&currentChannel->dxeChannelLock);

        /* Allocate DXE Control Block will be used by host DXE driver */
        status = dxe_alloc_dma_ring(dxe_ctx, currentChannel);
        if (A_OK != status)
        {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("WLANDXE_Open Alloc DXE Control Block Fail for channel %d", idx));

            hif_dxe_detach(dxe_ctx);
            return NULL;
        }

        /* RX specific attach */
        if ((HIFDXE_CHANNEL_RX_LOW_PRI  == currentChannel->channelType) ||
            (HIFDXE_CHANNEL_RX_HIGH_PRI == currentChannel->channelType))
        {
            adf_os_atomic_init(&currentChannel->rx_refill_ref_cnt);
            adf_os_atomic_inc(&currentChannel->rx_refill_ref_cnt);

            dxe_rx_ring_fill_n(dxe_ctx, currentChannel, currentChannel->numDesc);

            adf_os_timer_init(dxe_ctx->osdev, &currentChannel->rx_refill_retry_timer,
                dxe_rx_ring_refill_retry, (void *)currentChannel);
        }

        AR_DEBUG_PRINTF(HIF_DXE_DEBUG,("WLANDXE_Open Channel %d Open Success \n", idx));
    }

    dxe_ctx->dxeCookie    = WLANDXE_CTXT_COOKIE;
    dxe_ctx->rxIntDisabledByIMPS = FALSE;
    dxe_ctx->txIntDisabledByIMPS = FALSE;

    /* Initializing default BMPS host power state and firmware Power state*/
    dxe_ctx->hostPowerState = HIF_DXE_POWER_STATE_BMPS;
    dxe_ctx->fwPowerState = HIF_DXE_FW_POWER_STATE_BMPS_UNKNOWN;

    /* Initialize SMSM state
    * Init State is
    *    Clear TX Enable
    *    RING EMPTY STATE */

    smsmInitState = hif_dxe_notify_smsm(dxe_ctx,FALSE, TRUE);
    if (0 != smsmInitState)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("SMSM Channel init fail %d", smsmInitState));
        hif_dxe_detach(dxe_ctx);
        return NULL;
    }

    AR_DEBUG_PRINTF(HIF_DXE_DEBUG,("HIF_DXE Attach Success : \n"));

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));

    g_hif_dxe_ctx = dxe_ctx;

    return dxe_ctx;
}

/**
* @ HIF DXE Start . Initialize DXE Channels For DMA
*
* @param[in]hif_dxe_pdev -  HIF DXE Specific Context
*
* @retval   A_STATUS
*/

A_STATUS hif_dxe_start(hif_dxe_handle hif_dxe_pdev)
{
    A_STATUS status = A_OK;
    u_int32_t idx = 0;
    S_HIFDXE_CONTEXT * pdxectx =(S_HIFDXE_CONTEXT *) hif_dxe_pdev;

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));

    /* Sanity */
    if (NULL == pdxectx)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_start : Invalid DXE Context \n"));
        return A_EINVAL;
    }

    /* WLANDXE_Start called means DXE engine already initiates
    * And DXE HW is reset and init finished
    * But here to make sure HW is initialized, reset again */
    status = dxe_dma_core_start(pdxectx);
    if (A_OK != status)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_start : DXE HW init Fail \n"));
        return status;
    }

    /* Individual Channel Start */
    for (idx = 0; idx < HIFDXE_CHANNEL_MAX; idx++)
    {
        AR_DEBUG_PRINTF(HIF_DXE_DEBUG,("hif_dxe_start : Start DXE Channel %d \n", idx));

        /* Program each channel register with configuration arguments */
        //Trigger Start DMA Channel
        status = dxe_channel_start(pdxectx, &pdxectx->dxeChannel[idx]);
        if (A_OK != status)
        {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_start :  Start DMA channel %d Fail \n", idx));
            return status;
        }
        AR_DEBUG_PRINTF(HIF_DXE_DEBUG,("hif_dxe_start : Start DXE Channel %d SUCCESS! \n", idx));
    }

    //FIXME_RT Check if we need to notify hif_dxe_os to Start Routing Interrupts to us only after this point

    /* Enable system level ISR */
    /* Enable RX ready Interrupt at here */
    status = hif_dxe_os_program_int(pdxectx->hif_os_handle,E_HIFDXE_RX_INTERRUPT,E_HIFDXE_INT_ENABLE);
    if (A_OK != status)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_start : Enable TX complete interrupt FAIL! "));
        return status;
    }

	/* Disable TX ready Interrupt and Re-Enable Only when required  */
    status = hif_dxe_os_program_int(pdxectx->hif_os_handle,E_HIFDXE_TX_INTERRUPT,E_HIFDXE_INT_DISABLE);
    if (A_OK != status)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_start : Enable TX complete interrupt FAIL! "));
        return status;
    }

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));
    return status;
}

/**
* @ HIF DXE Register . Register Packet Callbacks
*
* @param[in]hif_dxe_pdev -  HIF DXE Specific Context
*
* @retval   A_STATUS
*/
A_STATUS hif_dxe_client_registration(hif_dxe_handle hif_dxe_pdev, S_HIFDXE_CALLBACK *hif_dxe_cb)
{
    S_HIFDXE_CONTEXT * pdxectx =(S_HIFDXE_CONTEXT *) hif_dxe_pdev;
    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));

    /* Sanity */
    if ((NULL == pdxectx) || (NULL == hif_dxe_cb))
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_client_registration : Invalid DXE Context \n"));
        return A_EINVAL;
    }

    if (NULL != hif_dxe_cb->HifTxCompleteCb) {
        pdxectx->hif_client_cb.HifTxCompleteCb = hif_dxe_cb->HifTxCompleteCb;
        pdxectx->hif_client_cb.HifTxCompleteCtx = hif_dxe_cb->HifTxCompleteCtx;
    }

    if (NULL != hif_dxe_cb->HifRxReadyCb) {
        pdxectx->hif_client_cb.HifRxReadyCb = hif_dxe_cb->HifRxReadyCb;
        pdxectx->hif_client_cb.HifRxReadyCtx = hif_dxe_cb->HifRxReadyCtx;
    }

    if (NULL != hif_dxe_cb->HifLowResourceCb) {
        pdxectx->hif_client_cb.HifLowResourceCb = hif_dxe_cb->HifLowResourceCb;
        pdxectx->hif_client_cb.HifLowResourceCtx = hif_dxe_cb->HifLowResourceCtx;
    }

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));
    return A_OK;
}

/**
* @ HIF DXE Send . Send packet on DXE
*
* @param[in]hif_dxe_pdev -  HIF DXE Specific Context
* @param[in]eHifDxeChannel -  DXE Channel for Send
* @param[in]nbuf -  SDU to send
*
* @retval   A_STATUS
*/
A_STATUS hif_dxe_send(hif_dxe_handle hif_dxe_pdev, E_HIFDXE_CHANNELTYPE channel, adf_nbuf_t nbuf)
{
    A_STATUS status = A_OK;
    WLANDXE_ChannelCBType  *currentChannel = NULL;
    S_HIFDXE_CONTEXT * pdxectx =(S_HIFDXE_CONTEXT *) hif_dxe_pdev;
    u_int32_t lowThreshold   = 0;

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));

    /* Sanity */
    if (NULL == pdxectx)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_send : Invalid DXE Context \n"));
        return A_EINVAL;
    }

    if (NULL == nbuf)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_send : Invalid Data packet \n"));
        return A_EINVAL;
    }

    if ((HIFDXE_CHANNEL_MAX <= channel))
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_send : Invalid Channel \n"));
        return A_EINVAL;
    }

    currentChannel = &pdxectx->dxeChannel[channel];


    lowThreshold = currentChannel->channelType == HIFDXE_CHANNEL_TX_LOW_PRI?
        (pdxectx->txCompInt.txLowResourceThreshold_LoPriCh):
    (pdxectx->txCompInt.txLowResourceThreshold_HiPriCh);

    /* Decide have to activate TX complete event or not */
    switch (pdxectx->txCompInt.txIntEnable)
    {
        /* TX complete interrupt will be activated when low DXE resource */
    case WLANDXE_TX_COMP_INT_LR_THRESHOLD:
        if (adf_os_atomic_read(&currentChannel->numFreeDesc) <= lowThreshold)
        {
            if (pdxectx->hif_client_cb.HifLowResourceCb)
            {
                pdxectx->hif_client_cb.HifLowResourceCb(pdxectx->hif_client_cb.HifLowResourceCtx,
                    channel,
                    TRUE);
            }
            currentChannel->hitLowResource = TRUE;
        }
        break;
        //Do We need to support the other Interrupt Mechanisms?

    case WLANDXE_TX_COMP_INT_PER_K_FRAMES:
        break;

        /* TX complete interrupt will be activated periodically */
    case WLANDXE_TX_COMP_INT_TIMER:
        break;
    default:
        adf_os_assert(0); //Eunexpected Configuration

    }


    /* Update DXE descriptor, this is frame based
    * if a frame consist of N fragments, N Descriptor will be programed */
    status = dxe_tx_push_frame(pdxectx,currentChannel, nbuf);
    if (A_OK != status)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_send : TX Push Frame failure  : %d\n",status));
        return status;
    }
    adf_os_atomic_inc(&pdxectx->tx_pkts_pending);

    adf_os_atomic_inc(&currentChannel->tx_pkts_pending);


#if 0
    //Try Drain Tx Queues
    currentChannel = &pdxectx->dxeChannel[HIFDXE_CHANNEL_TX_HIGH_PRI];

    /* Handle TX complete for high priority channel */
    status = dxe_tx_pull_frames(pdxectx, currentChannel, FALSE);
    if (A_OK != status)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_send : Tx Completion Handling Failed on Channel : %d \n",HIFDXE_CHANNEL_TX_HIGH_PRI));
    }

    currentChannel = &pdxectx->dxeChannel[HIFDXE_CHANNEL_TX_LOW_PRI];

    /* Handle TX complete for low priority channel */
    status = dxe_tx_pull_frames(pdxectx, currentChannel, FALSE);
    if (A_OK != status)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_send : Tx Completion Handling Failed on Channel : %d \n",HIFDXE_CHANNEL_TX_LOW_PRI));
    }
#endif
    //Enable Tx Interrupt if Pkts Pending
    if (adf_os_atomic_read(&pdxectx->tx_pkts_pending))
    {
        status = hif_dxe_os_program_int(pdxectx->hif_os_handle,E_HIFDXE_TX_INTERRUPT,E_HIFDXE_INT_ENABLE);
        if (A_OK != status)
        {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_send : Enable TX complete interrupt FAIL! "));
            return status;
        }
        AR_DEBUG_PRINTF(ATH_DEBUG_TRC,("hif_dxe_send TX COMP INT Enabled, remain TX frame count on ring %d\n",  adf_os_atomic_read(&pdxectx->tx_pkts_pending)));
        /*Kicking the DXE after the TX Complete interrupt was enabled - to avoid
        the posibility of a race*/
    }

    //Invoke PS Handler to check if SMSM Notify required
    dxe_ps_complete(pdxectx, FALSE);

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));

    return status;
}

//FIXME_RT : Do We Need This
//A_STATUS HIF_DXE_CompleteTX(void *dxe_ctx);


/**
* @ HIF DXE GetResources . Return Available Free Tx Slots in DXE DMA  Tx Ring for specified channel
*
* @param[in]hif_dxe_pdev -  HIF DXE Specific Context
* @param[in]eHifDxeChannel -  DXE Channel
*
* @retval   a_uint32_t Num Available Resources
*/
u_int32_t hif_dxe_get_resources(hif_dxe_handle hif_dxe_pdev, E_HIFDXE_CHANNELTYPE channel)
{
    S_HIFDXE_CONTEXT * pdxectx =(S_HIFDXE_CONTEXT *) hif_dxe_pdev;
    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));

    /* Sanity */
    if (NULL == pdxectx)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_get_resources : Invalid DXE Context \n"));
        adf_os_assert(0);
        return A_EINVAL;
    }

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));
    return adf_os_atomic_read(&pdxectx->dxeChannel[channel].numFreeDesc);
}
/**
* @ HIF DXE Flush Tx Packets . Flush And return Tx Packets Pending in DMA through Tx Completion Callbacks.
*
* @param[in]dxe_ctx -  HIF DXE Specific Context
*
* @retval   A_STATUS
*/
A_STATUS hif_dxe_flush_txpackets(hif_dxe_handle hif_dxe_pdev)
{
    S_HIFDXE_CONTEXT * pdxectx =(S_HIFDXE_CONTEXT *) hif_dxe_pdev;
    WLANDXE_ChannelCBType    *txChannel;
    A_STATUS                 status = A_OK;
    u_int32_t                txChannelsDisableRegValue;
    u_int8_t                 txChannelDisableWaitLoop;

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));

    /* Sanity */
    if (NULL == pdxectx)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_flush_txpackets : Invalid DXE Context \n"));
        adf_os_assert(0);
        return A_EINVAL;
    }

    /* To ensure DXE wake up, Sedn SMSM Notification */
    status = hif_dxe_notify_smsm(pdxectx, FALSE, FALSE);
    if (A_OK != status)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_flush_txpackets, SMSM Kick RIVA Fail %d", status));
        return status;
    }

    /* Disable TX Channels first */
    txChannelsDisableRegValue = hif_dxe_os_readreg(pdxectx->hif_os_handle,WALNDEX_DMA_CH_EN_ADDRESS);
    txChannelsDisableRegValue &= (~HIFDXE_TX_CHANNELS_EN_BIT_MASK);
    hif_dxe_os_writereg(pdxectx->hif_os_handle,WALNDEX_DMA_ENCH_ADDRESS, txChannelsDisableRegValue);

    /* Check TX channels are disabled or not
    * To flush TX channels, it should be disabled. */
    for (txChannelDisableWaitLoop = 0; txChannelDisableWaitLoop < HIFDXE_MAX_TX_CH_DISABLE_WAIT; txChannelDisableWaitLoop++)
    {
        txChannelsDisableRegValue = hif_dxe_os_readreg(pdxectx->hif_os_handle,WALNDEX_DMA_CH_EN_ADDRESS);

        if (0 == (txChannelsDisableRegValue & HIFDXE_TX_CHANNELS_EN_BIT_MASK))
        {
            break;
        }
        else
        {
            adf_os_udelay(1);
        }
    }
    /* MAX Time wait but could not disabled TX channel */
    if (HIFDXE_MAX_TX_CH_DISABLE_WAIT == txChannelDisableWaitLoop)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_flush_txpackets, During %d usec, failed to disable TX Channel",HIFDXE_MAX_TX_CH_DISABLE_WAIT));
    }

    //If Channels Could not be disabled continue to Force Pull out Tx Frames Anyway .

    /* Low Channel First */
    txChannel = &pdxectx->dxeChannel[HIFDXE_CHANNEL_TX_LOW_PRI];
    status = dxe_tx_pull_frames(pdxectx,txChannel, TRUE);
    if (A_OK != status)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_flush_txpackets Failed to Pull Frames from Channel : %d Status %d\n",txChannel->channelType,status));
        return status;
    }

    /* High Channel */
    txChannel = &pdxectx->dxeChannel[HIFDXE_CHANNEL_TX_HIGH_PRI];
    status = dxe_tx_pull_frames(pdxectx, txChannel, TRUE);
    if (A_OK != status)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_flush_txpackets, Failed to Pull Frames from Channel : %d Status %d\n",txChannel->channelType,status));
        return status;
    }
    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));
    return status;
}

/**
* @ HIF DXE Set Power State . Synchronously Set Power State relevant for DXE / Ensure No references into DXE are Outstanding
*
* @param[in]hif_dxe_pdev -  HIF DXE Specific Context
* @param[in]PowerState    -  New Power State
*
* @retval   A_STATUS
*/
A_STATUS hif_dxe_set_power_state(hif_dxe_handle hif_dxe_pdev, a_uint8_t PowerState )
{
    A_STATUS status = A_OK;
    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));
    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));
	return status;
}

/**
* @ HIF DXE Stop . Stop DMA
*
* @param[in]hif_dxe_pdev -  HIF DXE Specific Context
*
* @retval   A_STATUS
*/
A_STATUS hif_dxe_stop(hif_dxe_handle hif_dxe_pdev)
{
    A_STATUS  status = A_OK;
    u_int32_t  idx = 0;
    S_HIFDXE_CONTEXT * pdxectx =(S_HIFDXE_CONTEXT *) hif_dxe_pdev;

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));

    /* Sanity */
    if (NULL == pdxectx)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_stop : Invalid DXE Context \n"));
        return A_EINVAL;
    }

    //FIXME_RT Make sure No Outstanding References to DXE . Synchronize with dxe_stopped
    hif_dxe_os_stop(pdxectx->hif_os_handle);

    for (idx = 0; idx < HIFDXE_CHANNEL_MAX; idx++)
    {
        status = dxe_channel_stop(pdxectx, &pdxectx->dxeChannel[idx]);
        if (A_OK != status)
        {
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_stop : channel %d stop failed \n",idx));
            return status;
        }
    }

    /* During Stop Disable interrupts For Good (INTR Disable is syncronized at Interrupt IRQL)*/
    status = hif_dxe_os_program_int(pdxectx->hif_os_handle,E_HIFDXE_TX_INTERRUPT,E_HIFDXE_INT_DISABLE);
    if (A_OK != status)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_start : Disable TX complete interrupt FAIL! "));
    }
    status = hif_dxe_os_program_int(pdxectx->hif_os_handle,E_HIFDXE_RX_INTERRUPT,E_HIFDXE_INT_DISABLE);
    if (A_OK != status)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_start : Disable TX complete interrupt FAIL! "));
    }


    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));
    return status;
}

/**
 * @brief HIF DXE Read register
 *
 * @param[in] hif_dxe_pdev -  HIF DXE Specific Context
 * @param[in] Addr
 *
 * @return Read Value
 */
u_int32_t hif_dxe_readreg(hif_dxe_handle hif_dxe_pdev, u_int32_t addr)
{
    S_HIFDXE_CONTEXT * pdxectx =(S_HIFDXE_CONTEXT *) hif_dxe_pdev;
    return hif_dxe_os_readreg(pdxectx->hif_os_handle,addr);
}

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
hif_dxe_writereg(hif_dxe_handle hif_dxe_pdev, u_int32_t addr, u_int32_t val)
{
    S_HIFDXE_CONTEXT * pdxectx =(S_HIFDXE_CONTEXT *) hif_dxe_pdev;
    hif_dxe_os_writereg(pdxectx->hif_os_handle,addr,val);
}

/**
* @ HIF DXE Detach . UnInitialize DXE Resources / HW / Free Memory  / Detach OS Interrupts etc
*
* @param[in]dxe_ctx -  HIF DXE Specific Context
*
* @retval   A_STATUS
*/
A_STATUS hif_dxe_detach(hif_dxe_handle hif_dxe_pdev)
{
    u_int32_t idx = 0;
    S_HIFDXE_CONTEXT * pdxectx =(S_HIFDXE_CONTEXT *) hif_dxe_pdev;
    WLANDXE_ChannelCBType  *currentChannel = NULL;
	A_STATUS   status;
    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));

    /* Sanity */
    if (NULL == pdxectx)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_detach Invalid DXE Context \n"));
        return A_EINVAL;
    }


    if (!adf_os_atomic_dec_and_test(&pdxectx->ref_count)) {
        /* there are other clients still using dmux_dxe */
        return A_OK;
    }

    /*Stop DXE After making Sure OS Is DeInitialized ie No More DPC's are firing*/
    status = hif_dxe_stop(pdxectx);
    if (A_OK != status)
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("hif_dxe_detach : hif_dxe_stop Failed \n"));
    }

    hif_dxe_os_deinit(pdxectx->hif_os_handle);

    for (idx = 0; idx < HIFDXE_CHANNEL_MAX; idx++)
    {
        currentChannel = &pdxectx->dxeChannel[idx];
        if (currentChannel)
        {
            adf_os_spinlock_destroy(&currentChannel->dxeChannelLock);
            dxe_free_dma_ring(pdxectx, &pdxectx->dxeChannel[idx]);

            /* RX specific detach */
            if ((HIFDXE_CHANNEL_RX_LOW_PRI  == currentChannel->channelType) ||
                (HIFDXE_CHANNEL_RX_HIGH_PRI == currentChannel->channelType))
            {
                adf_os_timer_cancel(&currentChannel->rx_refill_retry_timer);
                adf_os_timer_free(&currentChannel->rx_refill_retry_timer);
            }
        }
    }
    adf_os_mem_free(pdxectx);
    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));
    return A_OK;
}

#if 0
void hif_dxe_dump()
{
    S_HIFDXE_CONTEXT * pdxectx =(S_HIFDXE_CONTEXT *) g_hif_dxe_ctx;
    WLANDXE_ChannelCBType  *currentChannel;
    A_UINT8 idx=0;

    AR_DEBUG_PRINTF(HIF_DXE_DEBUG, ("+%s\n",__FUNCTION__));
    for (idx = 0; idx < HIFDXE_CHANNEL_MAX; idx++)
    {
        currentChannel = &pdxectx->dxeChannel[idx];
        if (currentChannel)
        {
            A_UINT32 regval = hif_dxe_os_readreg(pdxectx->hif_os_handle,currentChannel->channelRegister.chDXEStatusRegAddr);
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("Channel Idx : %d\n",idx));
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("Channel Type : %d\n",currentChannel->channelType));
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("Num Desc : %d\n",currentChannel->numDesc));
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("Num Free Desc : %d\n",currentChannel->numFreeDesc));
            AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("Channel Status : 0x%x\n",regval));
        }

    //For Debugging Tx Stall Scenario check and print Avalibale BD's in BMU
        if(((currentChannel->channelType == HIFDXE_CHANNEL_TX_LOW_PRI) || (currentChannel->channelType == HIFDXE_CHANNEL_TX_HIGH_PRI)) &&
            currentChannel->tx_pkts_pending > HIF_DXE_TX_PENDING_DEBUG_THRESHOLD )
    {
        A_UINT32 avail_bds , bd_thres0 , regval ;
        bd_thres0 = hif_dxe_os_readreg(pdxectx->hif_os_handle, WLANDXE_BMU_PDU_THRES0);
        avail_bds = hif_dxe_os_readreg(pdxectx->hif_os_handle, WLANDXE_BMU_AVAILABLE_BD_PDU);
        if(avail_bds < ((bd_thres0 & WLANDXE_BMU_BD_THRES0_MASK) * 10))
        {
            A_UINT32 wq_addr = 0;
            A_UINT8 wq_idx = 0;
            //Currently dump only if bd's less than 10 times Threshold
             AR_DEBUG_PRINTF(ATH_DEBUG_ERR,(" Channel : %d TxPending : %d Avail BD-PDU : 0x%x Threshold : 0x%x \n",idx,currentChannel->tx_pkts_pending,avail_bds,bd_thres0));

             for(wq_idx=0; wq_idx < 27; wq_idx++)
             {
                 wq_addr = (0xFB80001c | ((wq_idx << 8) & 0xff00));
                 regval = hif_dxe_os_readreg(pdxectx->hif_os_handle,wq_addr);
                 AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("WQ8 : Addr : 0x%x 0x%x \n",wq_addr,regval));
             }
        }
      }
    }
}
#endif
