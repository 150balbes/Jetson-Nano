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
* @file hif_dxe_config.c
* @brief Provide functions for DXE HIF module DMA Channel configuration
* @details
*  This file Implements the HIF DXE DMA Channel Configuration Values like :
*  -  DMA Channel Base Addresses
*  -  DMA Channel Interrupt Maskshif
*  -  Provide DMA Channel to DMA Channel Configuration mapping
*  -  Per Channel Config Values like : NumDesc , NumBuffers , Priority , DMA xfr direction etc
*  -
*  -
*/
#include "adf_os_types.h"
#include "hif_dxe.h"
#include "hif_dxe_hw_pvt.h"
#include "hif_dxe_pvt.h"
#define ATH_MODULE_NAME hif_dxe_config
#include <a_debug.h>

#define HIF_DXE_CONFIG_DEBUG   ATH_DEBUG_MAKE_MODULE_MASK(0)

#if defined(DEBUG)
static ATH_DEBUG_MASK_DESCRIPTION g_HIFConfigDebugDescription[] = {
    {HIF_DXE_CONFIG_DEBUG,"hif_dxe_config"},
};

ATH_DEBUG_INSTANTIATE_MODULE_VAR(hif_dxe_config,
                                 "hif_dxe_config",
                                 "DXE HIF Config Module",
                                 ATH_DEBUG_MASK_DEFAULTS | ATH_DEBUG_INFO | HIF_DXE_CONFIG_DEBUG,
                                 ATH_DEBUG_DESCRIPTION_COUNT(g_HIFConfigDebugDescription),
                                 g_HIFConfigDebugDescription);
#endif

typedef struct
{
    E_HIFDXE_CHANNELTYPE       wlanChannel;
    WLANDXE_DMAChannelType     DMAChannel;
    WLANDXE_ChannelConfigType *channelConfig;
} WLANDXE_ChannelMappingType;

u_int32_t channelBaseAddressList[WLANDXE_DMA_CHANNEL_MAX] =
{
    WLANDXE_DMA_CHAN0_BASE_ADDRESS,
    WLANDXE_DMA_CHAN1_BASE_ADDRESS,
    WLANDXE_DMA_CHAN2_BASE_ADDRESS,
    WLANDXE_DMA_CHAN3_BASE_ADDRESS,
    WLANDXE_DMA_CHAN4_BASE_ADDRESS,
    WLANDXE_DMA_CHAN5_BASE_ADDRESS,
    WLANDXE_DMA_CHAN6_BASE_ADDRESS
};

u_int32_t channelInterruptMask[WLANDXE_DMA_CHANNEL_MAX] =
{
    WLANDXE_INT_MASK_CHAN_0,
    WLANDXE_INT_MASK_CHAN_1,
    WLANDXE_INT_MASK_CHAN_2,
    WLANDXE_INT_MASK_CHAN_3,
    WLANDXE_INT_MASK_CHAN_4,
    WLANDXE_INT_MASK_CHAN_5,
    WLANDXE_INT_MASK_CHAN_6
};

WLANDXE_ChannelConfigType chanTXLowPriConfig =
{
    /* Q handle type, Circular */
    WLANDXE_CHANNEL_HANDLE_CIRCULA,

    /* Number of Descriptor, NOT CLEAR YET !!! */
    HIFDXE_TX_LO_PRI_RES_NUM ,

    /* MAX num RX Buffer */
    0,

    /* Reference WQ, TX23 */
#ifdef HIF_DXE_TEST
    11,
#else
    23,
#endif

    /* USB Only, End point info */
    0,

    /* Transfer Type */
    WLANDXE_DESC_CTRL_XTYPE_H2B,

    /* Channel Priority 7(Highest) - 0(Lowest) NOT CLEAR YET !!! */
    4,

    /* BD attached to frames for this pipe */
    TRUE,

    /* chk_size, NOT CLEAR YET !!!*/
    0,

    /* bmuThdSel, NOT CLEAR YET !!! */
    5,

    /* Added in Gen5 for Prefetch, NOT CLEAR YET !!! */
    TRUE,

    /* Use short Descriptor */
    TRUE
};

WLANDXE_ChannelConfigType chanTXHighPriConfig =
{
    /* Q handle type, Circular */
    WLANDXE_CHANNEL_HANDLE_CIRCULA,

    /* Number of Descriptor, NOT CLEAR YET !!! */
    HIFDXE_TX_HI_PRI_RES_NUM ,

    /* MAX num RX Buffer */
    0,

    /* Reference WQ, TX23 */
    23,

    /* USB Only, End point info */
    0,

    /* Transfer Type */
    WLANDXE_DESC_CTRL_XTYPE_H2B,

    /* Channel Priority 7(Highest) - 0(Lowest), NOT CLEAR YET !!! */
    6,

    /* BD attached to frames for this pipe */
    TRUE,

    /* chk_size, NOT CLEAR YET !!!*/
    0,

    /* bmuThdSel, NOT CLEAR YET !!! */
    7,

    /* Added in Gen5 for Prefetch, NOT CLEAR YET !!!*/
    TRUE,

    /* Use short Descriptor */
    TRUE
};

WLANDXE_ChannelConfigType chanRXLowPriConfig =
{
    /* Q handle type, Circular */
    WLANDXE_CHANNEL_HANDLE_CIRCULA,

    /* Number of Descriptor, NOT CLEAR YET !!! */
    256,

    /* MAX num RX Buffer, NOT CLEAR YET !!! */
    1,

    /* Reference WQ, NOT CLEAR YET !!! */
    /* Temporary BMU Work Q 4 */

    11,


    /* USB Only, End point info */
    0,

    /* Transfer Type */
    WLANDXE_DESC_CTRL_XTYPE_B2H,

    /* Channel Priority 7(Highest) - 0(Lowest), NOT CLEAR YET !!! */
    5,

    /* BD attached to frames for this pipe */
    TRUE,

    /* chk_size, NOT CLEAR YET !!!*/
    0,

    /* bmuThdSel, NOT CLEAR YET !!! */
    6,

    /* Added in Gen5 for Prefetch, NOT CLEAR YET !!!*/
    TRUE,

    /* Use short Descriptor */
    TRUE
};

WLANDXE_ChannelConfigType chanRXHighPriConfig =
{
    /* Q handle type, Circular */
    WLANDXE_CHANNEL_HANDLE_CIRCULA,

    /* Number of Descriptor, NOT CLEAR YET !!! */
    256,

    /* MAX num RX Buffer, NOT CLEAR YET !!! */
    1,

    /* Reference WQ, RX11 */
    4,

    /* USB Only, End point info */
    0,

    /* Transfer Type */
    WLANDXE_DESC_CTRL_XTYPE_B2H,

    /* Channel Priority 7(Highest) - 0(Lowest), NOT CLEAR YET !!! */
    6,

    /* BD attached to frames for this pipe */
    TRUE,

    /* chk_size, NOT CLEAR YET !!!*/
    0,

    /* bmuThdSel, NOT CLEAR YET !!! */
    8,

    /* Added in Gen5 for Prefetch, NOT CLEAR YET !!!*/
    TRUE,

    /* Use short Descriptor */
    TRUE
};


WLANDXE_ChannelMappingType channelList[HIFDXE_CHANNEL_MAX] =
{
    {HIFDXE_CHANNEL_TX_LOW_PRI,  WLANDXE_DMA_CHANNEL_0, &chanTXLowPriConfig},
    {HIFDXE_CHANNEL_TX_HIGH_PRI, WLANDXE_DMA_CHANNEL_4, &chanTXHighPriConfig},
    {HIFDXE_CHANNEL_RX_LOW_PRI,  WLANDXE_DMA_CHANNEL_1, &chanRXLowPriConfig},
    {HIFDXE_CHANNEL_RX_HIGH_PRI, WLANDXE_DMA_CHANNEL_3, &chanRXHighPriConfig},
};

WLANDXE_TxCompIntConfigType txCompInt =
{
    /* TX Complete Interrupt enable method */
    WLANDXE_TX_COMP_INT_PER_K_FRAMES,

    /* TX Low Resource remaining resource threshold for Low Pri Ch */
    WLANDXE_TX_LOW_RES_THRESHOLD,

    /* TX Low Resource remaining resource threshold for High Pri Ch */
    WLANDXE_TX_LOW_RES_THRESHOLD,

    /* RX Low Resource remaining resource threshold */
    5,

    /* Per K frame enable Interrupt */
    /*WLANDXE_HI_PRI_RES_NUM*/ 5,

    /* Periodic timer msec */
    10
};

A_STATUS dxe_cmn_def_config(S_HIFDXE_CONTEXT * dxe_ctx)
{
    A_MEMCPY(&dxe_ctx->txCompInt,&txCompInt,sizeof(WLANDXE_TxCompIntConfigType));
    return A_OK;
}


A_STATUS dxe_chan_def_config(S_HIFDXE_CONTEXT * pdxectx,WLANDXE_ChannelCBType   *channelEntry)
{
    A_STATUS                    status = A_OK;
    u_int32_t                  baseAddress;
    u_int32_t                  dxeControlRead  = 0;
    u_int32_t                  dxeControlWrite = 0;
    u_int32_t                  dxeControlWriteValid = 0;
    u_int32_t                  dxeControlWriteEop = 0;
    u_int32_t                  dxeControlWriteEopInt = 0;
    u_int32_t                  idx;
    WLANDXE_ChannelMappingType *mappedChannel = NULL;

    /* Sanity */
    if ((NULL == pdxectx) || (NULL == channelEntry))
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("dxe_chan_def_config : Invalid DXE Context / DMA Channel Entry \n"));
        return A_EINVAL;
    }

    for(idx = 0; idx < HIFDXE_CHANNEL_MAX; idx++)
    {
        if(channelEntry->channelType == channelList[idx].wlanChannel)
        {
            mappedChannel = &channelList[idx];
            break;
        }
    }

    if((NULL == mappedChannel) || (HIFDXE_CHANNEL_MAX == idx))
    {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR,("dxe_chan_def_config : Failed to map channel \n"));
        return A_EINVAL;
    }

    A_MEMCPY(&channelEntry->channelConfig,mappedChannel->channelConfig,sizeof(WLANDXE_ChannelConfigType));

    baseAddress = channelBaseAddressList[mappedChannel->DMAChannel];
    channelEntry->channelRegister.chDXEBaseAddr        = baseAddress;
    channelEntry->channelRegister.chDXEStatusRegAddr   = baseAddress + WLANDXE_DMA_CH_STATUS_REG;
    channelEntry->channelRegister.chDXEDesclRegAddr    = baseAddress + WLANDXE_DMA_CH_DESCL_REG;
    channelEntry->channelRegister.chDXEDeschRegAddr    = baseAddress + WLANDXE_DMA_CH_DESCH_REG;
    channelEntry->channelRegister.chDXELstDesclRegAddr = baseAddress + WLANDXE_DMA_CH_LST_DESCL_REG;
    channelEntry->channelRegister.chDXECtrlRegAddr     = baseAddress + WLANDXE_DMA_CH_CTRL_REG;
    channelEntry->channelRegister.chDXESzRegAddr       = baseAddress + WLANDXE_DMA_CH_SZ_REG;
    channelEntry->channelRegister.chDXEDadrlRegAddr    = baseAddress + WLANDXE_DMA_CH_DADRL_REG;
    channelEntry->channelRegister.chDXEDadrhRegAddr    = baseAddress + WLANDXE_DMA_CH_DADRH_REG;
    channelEntry->channelRegister.chDXESadrlRegAddr    = baseAddress + WLANDXE_DMA_CH_SADRL_REG;
    channelEntry->channelRegister.chDXESadrhRegAddr    = baseAddress + WLANDXE_DMA_CH_SADRH_REG;

    /* Channel Mask?
    * This value will control channel control register.
    * This register will be set to trigger actual DMA transfer activate
    * CH_N_CTRL */
    channelEntry->extraConfig.chan_mask = 0;
    /* Check VAL bit before processing descriptor */
    channelEntry->extraConfig.chan_mask |= WLANDXE_CH_CTRL_EDVEN_MASK;
    /* Use External Descriptor Linked List */
    channelEntry->extraConfig.chan_mask |= WLANDXE_CH_CTRL_EDEN_MASK;
    /* Enable Channel Interrupt on error */
    channelEntry->extraConfig.chan_mask |= WLANDXE_CH_CTRL_INE_ERR_MASK;
    /* Enable INT after XFER done */
    channelEntry->extraConfig.chan_mask |= WLANDXE_CH_CTRL_INE_DONE_MASK;
    /* Enable INT External Descriptor */
    channelEntry->extraConfig.chan_mask |= WLANDXE_CH_CTRL_INE_ED_MASK;
    /* Set Channel This is not channel, event counter, somthing wrong */
    channelEntry->extraConfig.chan_mask |=
        mappedChannel->DMAChannel << WLANDXE_CH_CTRL_CTR_SEL_OFFSET;
    /* Transfer Type */
    channelEntry->extraConfig.chan_mask |= mappedChannel->channelConfig->xfrType;
    /* Use Short Descriptor, THIS LOOKS SOME WIERD, REVISIT */
    if(!channelEntry->channelConfig.useShortDescFmt)
    {
        channelEntry->extraConfig.chan_mask |= WLANDXE_DESC_CTRL_DFMT;
    }
    /* TX Channel, Set DIQ bit, Clear SIQ bit since source is not WQ */
    if((HIFDXE_CHANNEL_TX_LOW_PRI  == channelEntry->channelType) ||
        (HIFDXE_CHANNEL_TX_HIGH_PRI == channelEntry->channelType))
    {
        channelEntry->extraConfig.chan_mask |= WLANDXE_CH_CTRL_DIQ_MASK;
    }
    /* RX Channel, Set SIQ bit, Clear DIQ bit since source is not WQ */
    else if((HIFDXE_CHANNEL_RX_LOW_PRI  == channelEntry->channelType) ||
        (HIFDXE_CHANNEL_RX_HIGH_PRI == channelEntry->channelType))
    {
        channelEntry->extraConfig.chan_mask |= WLANDXE_CH_CTRL_SIQ_MASK;
    }
    else
    {
        /* This is test H2H channel, TX, RX not use work Q
        * Do Nothing */
    }
    /* Frame Contents Swap */
    channelEntry->extraConfig.chan_mask |= WLANDXE_CH_CTRL_SWAP_MASK;
    /* Host System Using Little Endian */
    channelEntry->extraConfig.chan_mask |= WLANDXE_CH_CTRL_ENDIAN_MASK;
    /* BMU Threshold select */
    channelEntry->extraConfig.chan_mask |=
        channelEntry->channelConfig.bmuThdSel << WLANDXE_CH_CTRL_BTHLD_SEL_OFFSET;
    /* EOP for control register ??? */
    channelEntry->extraConfig.chan_mask |= WLANDXE_CH_CTRL_EOP_MASK;
    /* Channel Priority */
    channelEntry->extraConfig.chan_mask |= channelEntry->channelConfig.chPriority << WLANDXE_CH_CTRL_PRIO_OFFSET;
    /* PDU REL */
    channelEntry->extraConfig.chan_mask |= WLANDXE_DESC_CTRL_PDU_REL;
    /* Disable DMA transfer on this channel */
    channelEntry->extraConfig.chan_mask_read_disable = channelEntry->extraConfig.chan_mask;
    /* Enable DMA transfer on this channel */
    channelEntry->extraConfig.chan_mask |= WLANDXE_CH_CTRL_EN_MASK;
    /* Channel Mask done */

    /* Control Read
    * Default Descriptor control Word value for RX ready DXE descriptor
    * DXE engine will reference this value before DMA transfer */
    dxeControlRead = 0;
    /* Source is a Queue ID, not flat memory address */
    dxeControlRead |= WLANDXE_DESC_CTRL_SIQ;
    /* Transfer direction is BMU 2 Host */
    dxeControlRead |= WLANDXE_DESC_CTRL_XTYPE_B2H;
    /* End of Packet, RX is single fragment */
    dxeControlRead |= WLANDXE_DESC_CTRL_EOP;
    /* BD Present, default YES, B2H case it must be 0 to insert BD */
    if(!channelEntry->channelConfig.bdPresent)
    {
        dxeControlRead |= WLANDXE_DESC_CTRL_BDH;
    }
    /* Channel Priority */
    dxeControlRead |= channelEntry->channelConfig.chPriority << WLANDXE_CH_CTRL_PRIO_OFFSET;
    /* BMU Threshold select, only used H2B, not this case??? */
    dxeControlRead |= channelEntry->channelConfig.bmuThdSel << WLANDXE_CH_CTRL_BTHLD_SEL_OFFSET;
    /* PDU Release, Release BD/PDU when DMA done */
    dxeControlRead |= WLANDXE_DESC_CTRL_PDU_REL;
    /* Use Short Descriptor, THIS LOOKS SOME WIERD, REVISIT */
    if(!channelEntry->channelConfig.useShortDescFmt)
    {
        dxeControlRead |= WLANDXE_DESC_CTRL_DFMT;
    }
    /* Interrupt on Descriptor done */
    dxeControlRead |= WLANDXE_DESC_CTRL_INT;
    /* For ready status, this Control WORD must be VALID */
    dxeControlRead |= WLANDXE_DESC_CTRL_VALID;
    /* Frame Contents Swap */
    dxeControlRead |= WLANDXE_DESC_CTRL_BDT_SWAP;
    /* Host Little Endian */
    if((HIFDXE_CHANNEL_TX_LOW_PRI  == channelEntry->channelType) ||
        (HIFDXE_CHANNEL_TX_HIGH_PRI == channelEntry->channelType))
    {
        dxeControlRead |= WLANDXE_DESC_CTRL_ENDIANNESS;
    }

    /* SWAP if needed */
    channelEntry->extraConfig.cw_ctrl_read = WLANDXE_U32_SWAP_ENDIAN(dxeControlRead);
    /* Control Read Done */

    /* Control Write
    * Write into DXE descriptor control word to TX frame
    * DXE engine will reference this word to contorl TX DMA channel */
    channelEntry->extraConfig.cw_ctrl_write = 0;
    /* Transfer type, from Host 2 BMU */
    dxeControlWrite |= mappedChannel->channelConfig->xfrType;
    /* BD Present, this looks some weird ??? */
    if(!channelEntry->channelConfig.bdPresent)
    {
        dxeControlWrite |= WLANDXE_DESC_CTRL_BDH;
    }
    /* Channel Priority */
    dxeControlWrite |= channelEntry->channelConfig.chPriority << WLANDXE_CH_CTRL_PRIO_OFFSET;
    /* Use Short Descriptor, THIS LOOKS SOME WIERD, REVISIT */
    if(!channelEntry->channelConfig.useShortDescFmt)
    {
        dxeControlWrite |= WLANDXE_DESC_CTRL_DFMT;
    }
    /* BMU Threshold select, only used H2B, not this case??? */
    dxeControlWrite |= channelEntry->channelConfig.bmuThdSel << WLANDXE_CH_CTRL_BTHLD_SEL_OFFSET;
    /* Destination is WQ */
    dxeControlWrite |= WLANDXE_DESC_CTRL_DIQ;
    /* Frame Contents Swap */
    dxeControlWrite |= WLANDXE_DESC_CTRL_BDT_SWAP;
    /* Host Little Endian */
    dxeControlWrite |= WLANDXE_DESC_CTRL_ENDIANNESS;
    /* Interrupt Enable */
    dxeControlWrite |= WLANDXE_DESC_CTRL_INT;

    dxeControlWriteValid  = dxeControlWrite | WLANDXE_DESC_CTRL_VALID;
    dxeControlWriteEop    = dxeControlWriteValid | WLANDXE_DESC_CTRL_EOP;
    dxeControlWriteEopInt = dxeControlWriteEop | WLANDXE_DESC_CTRL_INT;

    /* DXE Descriptor must has Endian swapped value */
    channelEntry->extraConfig.cw_ctrl_write = WLANDXE_U32_SWAP_ENDIAN(dxeControlWrite);
    /* Control Write DONE */

    /* Control Write include VAL bit
    * This Control word used to set valid bit and
    * trigger DMA transfer for specific descriptor */
    channelEntry->extraConfig.cw_ctrl_write_valid =
        WLANDXE_U32_SWAP_ENDIAN(dxeControlWriteValid);

    /* Control Write include EOP
    * End of Packet */
    channelEntry->extraConfig.cw_ctrl_write_eop =
        WLANDXE_U32_SWAP_ENDIAN(dxeControlWriteEop);

    /* Control Write include EOP and INT
    * indicate End Of Packet and generate interrupt on descriptor Done */
    channelEntry->extraConfig.cw_ctrl_write_eop_int =
        WLANDXE_U32_SWAP_ENDIAN(dxeControlWriteEopInt);


    /* size mask???? */
    channelEntry->extraConfig.chk_size_mask =
        mappedChannel->channelConfig->chk_size << 10;

    channelEntry->extraConfig.refWQ_swapped =
        WLANDXE_U32_SWAP_ENDIAN(channelEntry->channelConfig.refWQ);

    /* Set Channel specific Interrupt mask */
    channelEntry->extraConfig.intMask = channelInterruptMask[mappedChannel->DMAChannel];


    channelEntry->numDesc            = mappedChannel->channelConfig->nDescs;
    channelEntry->assignedDMAChannel = mappedChannel->DMAChannel;
    adf_os_atomic_set(&channelEntry->numFreeDesc, 0);
    adf_os_atomic_set(&channelEntry->numRsvdDesc, 0);
    adf_os_atomic_set(&channelEntry->numFragmentCurrentChain, 0);
    channelEntry->numTotalFrame           = 0;
    channelEntry->hitLowResource          = FALSE;
    return status;
}
