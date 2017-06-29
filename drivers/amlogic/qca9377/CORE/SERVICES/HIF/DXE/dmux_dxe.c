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
 * @file dmux_dxe.c
 * @brief Provide functions to demux receive frames.
 * @details
 *  This file contains functions for demuxing receive frames (from HIF_DXE) into 3 categories
 *      RX data frames
 *      RX management frames
 *      RX messages (HTT_ISOC_T2H_*,  e.g. HTT_ISOC_T2H_MSG_TYPE_PEER_INFO)
 *
 *  Clients can register callbacks for each type of receive frame.
 */

#include "adf_os_mem.h"      /* adf_os_mem_alloc */
#include "isoc_hw_desc.h"    /* RX BD */

#include "ieee80211_common.h"       /* struct ieee80211_frame */

#include "dmux_dxe_api.h"
#include "dmux_dxe_internal.h"

#define ATH_MODULE_NAME dmux_dxe
#include "a_debug.h"

#define DMUX_DXE_DEBUG   ATH_DEBUG_MAKE_MODULE_MASK(0)

#if defined(DEBUG)
static ATH_DEBUG_MASK_DESCRIPTION g_DebugDescription[] = {
    {DMUX_DXE_DEBUG, "dmux_dxe"},
};

ATH_DEBUG_INSTANTIATE_MODULE_VAR(dmux_dxe,
                                 "dmux_dxe",
                                 "DMUX DXE",
                                 ATH_DEBUG_MASK_DEFAULTS | ATH_DEBUG_INFO | DMUX_DXE_DEBUG,
                                 ATH_DEBUG_DESCRIPTION_COUNT(g_DebugDescription),
                                 g_DebugDescription);
#endif

//PS TEST - need to put this in a common location
#define HIF_DXE_RX_BUFFER_SIZE                  2612

enum dmux_dxe_frame_type {
    DMUX_DXE_FRAME_TYPE_DATA    = 0,
    DMUX_DXE_FRAME_TYPE_MGMT    = 1,
    DMUX_DXE_FRAME_TYPE_CTRL    = 2,
    DMUX_DXE_FRAME_TYPE_MESSAGE = 3,
    DMUX_DXE_FRAME_TYPE_UNKNOWN = 255
};

/* Local globals */
static struct dmux_dxe_pdev_t *g_pdev = NULL;


#ifdef BIG_ENDIAN_HOST
#define dmux_dxe_rx_bd_swap(rx_bd)
#else
static inline void dmux_dxe_rx_bd_swap(isoc_rx_bd_t *rx_bd)
{
    isoc_hw_bd_swap_bytes32((char *) rx_bd, sizeof(*rx_bd));
}
#endif

//static enum A_BOOL
static A_BOOL
dmux_dxe_validate_frame(adf_nbuf_t rx_frame)
{
    isoc_rx_bd_t *rx_bd;
    struct ieee80211_frame *dot11frame;
    a_uint8_t mpdu_header_offset, mpdu_header_length;
    a_uint16_t mpdu_data_offset;
    a_uint16_t mpdu_length; /* entire mpdu length (header + data) */
    A_BOOL amsdu_subframe;
    A_BOOL amsdu_subframe_first;
    a_uint8_t frame_type_subtype;

    rx_bd = (isoc_rx_bd_t *)adf_nbuf_data(rx_frame);

    /* Convert to host byte format */
    dmux_dxe_rx_bd_swap(rx_bd);

    if (rx_bd->htt_t2h_msg) {
        //PS TEST - TODO - any special validation for HTT_T2H messages?
        return TRUE;
    }

    /* Validate BD */
    mpdu_header_offset = rx_bd->mpdu_header_offset;
    mpdu_header_length = rx_bd->mpdu_header_length;
    mpdu_data_offset   = rx_bd->mpdu_data_offset;
    mpdu_length        = rx_bd->mpdu_length;

    if ((mpdu_data_offset <= mpdu_header_offset) || (mpdu_length < mpdu_header_length)) {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("%s: BE header corrupt - dropping frame.\n", __FUNCTION__));
        return FALSE;
    }

    amsdu_subframe = rx_bd->amsdu;
    amsdu_subframe_first = rx_bd->amsdu_first;

    if ((mpdu_header_offset < sizeof(*rx_bd)) &&  (!(amsdu_subframe && !amsdu_subframe_first))) {
        /* AMSDU case, ucMPDUHOffset = 0  it should be handled separately. Drop packet */
        return FALSE;
    }

    /* A-MSDU frame, but not first sub-frame
     * No MPDU header, MPDU header offset is 0
     * Total frame size is actual frame size + MPDU data offset
     */
    if ((mpdu_header_offset < sizeof(*rx_bd)) && (amsdu_subframe && !amsdu_subframe_first)) {
        mpdu_header_offset = mpdu_data_offset;
    }

    if (HIF_DXE_RX_BUFFER_SIZE < (mpdu_length + mpdu_header_offset)) {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("%s: Invalid frame size, possible memory corruption - dropping frame.\n", __FUNCTION__));
        return FALSE;
    }

    /* Update RX BD */
    //PS TEST - NOTE - this is uncached memory.  We might consider keeping a cached RX SW DESC
    rx_bd->mpdu_header_offset = mpdu_header_offset;

    /*
     * Fix HW reported frame type as typeSubtype in BD is not reliable.
     * Determine frame type (data/mgmt) from 802.11 MAC header.
     */
    frame_type_subtype = IEEE80211_FC0_TYPE_DATA | IEEE80211_FC0_SUBTYPE_QOS;
    if (0 == rx_bd->frame_translate)
    {
        if(!amsdu_subframe)
        {
            dot11frame = (struct ieee80211_frame *)(((a_uint8_t *)rx_bd) + mpdu_header_offset);
            frame_type_subtype = dot11frame->i_fc[0];
        }
        else if(amsdu_subframe_first)
        {
            //Only the first subframe in AMSDU Contains the 80211 Header . For Following Subframes the frame_type_subtype is already initialized above
            dot11frame = (struct ieee80211_frame *)(((a_uint8_t *)rx_bd) + sizeof(*rx_bd));
            frame_type_subtype = dot11frame->i_fc[0];
        }
    }

    rx_bd->frame_type_subtype = frame_type_subtype;

    return TRUE;
}

static inline enum dmux_dxe_frame_type
dmux_dxe_determine_frame_type(adf_nbuf_t rx_frame, E_HIFDXE_CHANNELTYPE channel_type)
{
    isoc_rx_bd_t *rx_bd;

    /* DXE channel allocation
     * HIFDXE_CHANNEL_RX_LOW_PRI  - data, ctrl (BAR), HTT_T2H messages
     * HIFDXE_CHANNEL_RX_HIGH_PRI - data, ctrl (BAR), HTT_T2H messages, mgmt
     */

    rx_bd = (isoc_rx_bd_t *)adf_nbuf_data(rx_frame);

    if (rx_bd->htt_t2h_msg) {
        return DMUX_DXE_FRAME_TYPE_MESSAGE;
    }

    switch ((rx_bd->frame_type_subtype & IEEE80211_FC0_TYPE_MASK)) {
    case IEEE80211_FC0_TYPE_DATA:
        return DMUX_DXE_FRAME_TYPE_DATA;
    case IEEE80211_FC0_TYPE_MGT:
        return DMUX_DXE_FRAME_TYPE_MGMT;
    case IEEE80211_FC0_TYPE_CTL:
        return DMUX_DXE_FRAME_TYPE_CTRL;
    default:
        return DMUX_DXE_FRAME_TYPE_UNKNOWN;
    }
}

static void
dmux_dxe_indicate_frame_list(dmux_dxe_handle pdev, adf_nbuf_t rx_frame_list,
                             enum dmux_dxe_frame_type frame_type,
                             E_HIFDXE_CHANNELTYPE chan_type)
{
    adf_nbuf_t rx_frame;

    switch (frame_type)
    {
    case DMUX_DXE_FRAME_TYPE_DATA:
        if (pdev->rx_data_cb) {
            pdev->rx_data_cb(pdev->rx_data_context, rx_frame_list, chan_type);
            return;
        }
        break;
    case DMUX_DXE_FRAME_TYPE_MGMT:
        if (pdev->rx_mgmt_cb) {
            pdev->rx_mgmt_cb(pdev->rx_mgmt_context, rx_frame_list);
            return;
        }
        break;
    case DMUX_DXE_FRAME_TYPE_CTRL:
        if (pdev->rx_ctrl_cb) {
            pdev->rx_ctrl_cb(pdev->rx_ctrl_context, rx_frame_list);
            return;
        }
        break;
    case DMUX_DXE_FRAME_TYPE_MESSAGE:
        if (pdev->msg_cb) {
            pdev->msg_cb( pdev->msg_context, rx_frame_list);
            return;
        }
        break;
    default:
        adf_os_assert(0);
        break;
    }

    /* No callback registered or unknown frame type - free frame list */
    rx_frame = rx_frame_list;
    while (rx_frame) {
        adf_nbuf_t rx_frame_next = adf_nbuf_next(rx_frame);
        adf_nbuf_set_next(rx_frame, NULL);
        adf_nbuf_free(rx_frame);
        rx_frame = rx_frame_next;
    }
}

/* RX Callback from HIF_DXE */
static A_STATUS
dmux_dxe_rx(void *pContext, adf_nbuf_t rx_list, E_HIFDXE_CHANNELTYPE channel_type)
{
    dmux_dxe_handle pdev = pContext;
    adf_nbuf_t rx_frame, rx_frame_prev, indication_list;
    enum dmux_dxe_frame_type frame_type, frame_type_prev = DMUX_DXE_FRAME_TYPE_UNKNOWN;

    if (!pdev || !rx_list) {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("%s: Invalid handle.\n", __FUNCTION__));
        return A_EINVAL;
    }

    rx_frame = indication_list = rx_list;
    rx_frame_prev = NULL;

    while (rx_frame) {

        /* Validate frame */
        if (!dmux_dxe_validate_frame(rx_frame)) {

            /* Invalid frame - drop */
            adf_nbuf_t next = adf_nbuf_next(rx_frame);

            if (rx_frame_prev) {
                /* remove invalid frame from indication list */
                adf_nbuf_set_next(rx_frame_prev,  next);
            } else {
                /* start of indication list -> update indication list head */
                indication_list = next;
            }

            adf_nbuf_free(rx_frame);

            /* next */
            rx_frame = next;
            continue;
        }

        /* Determine frame type (rx data, rx mgmt, HTT message) */
        frame_type = dmux_dxe_determine_frame_type(rx_frame, channel_type);

        /* If a different frame type is encountered, indicate the prior frames as a single list */
        if (rx_frame_prev && (frame_type != frame_type_prev)) {
            /* terminate list */
            adf_nbuf_set_next(rx_frame_prev, NULL);

            /* Indicate frames */
            dmux_dxe_indicate_frame_list(
                pdev, indication_list, frame_type_prev, channel_type);

            /* new indication list */
            indication_list = rx_frame;
        }

        /* next */
        frame_type_prev = frame_type;
        rx_frame_prev = rx_frame;
        rx_frame = adf_nbuf_next(rx_frame);
    }

    if (rx_frame_prev) {
        /* terminate list */
        adf_nbuf_set_next(rx_frame_prev, NULL);

        /* Indicate frames */
        dmux_dxe_indicate_frame_list(pdev, indication_list, frame_type_prev, channel_type);
    }

    return A_OK;
}


dmux_dxe_handle
dmux_dxe_attach(adf_os_device_t osdev)
{
    S_HIFDXE_CALLBACK hif_dxe_callbacks = {0};
    A_STATUS status;

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));

    if (g_pdev) {
       AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("%s : dmux_dxe object already allocated. Return existing object.\n", __FUNCTION__));
       adf_os_atomic_inc(&g_pdev->ref_count);
       return g_pdev;
    }

    g_pdev = adf_os_mem_alloc(osdev, sizeof(struct dmux_dxe_pdev_t));
    if (NULL == g_pdev) {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("%s : dmux_dxe object allocation failure.\n", __FUNCTION__));
        return NULL;
    }
    adf_os_mem_zero(g_pdev, sizeof(*g_pdev));

    adf_os_atomic_init(&g_pdev->ref_count);
    adf_os_atomic_inc(&g_pdev->ref_count);

    /* Attach to HIF_DXE */
    g_pdev->h_hif_dxe = hif_dxe_attach(osdev);
    if (!g_pdev->h_hif_dxe) {
        goto fail;
    }

    /* Register RX callback with HIF_DXE */
    hif_dxe_callbacks.HifRxReadyCb = dmux_dxe_rx;
    hif_dxe_callbacks.HifRxReadyCtx = g_pdev;
    status = hif_dxe_client_registration(g_pdev->h_hif_dxe, &hif_dxe_callbacks);
    if (A_OK != status) {
        goto fail;
    }

    return g_pdev;

fail:
    dmux_dxe_detach(g_pdev);
    return NULL;
}

void dmux_dxe_detach(dmux_dxe_handle pdev)
{
    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("+%s\n",__FUNCTION__));

    if ((NULL == pdev) || (pdev != g_pdev)) {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("dmux_dxe_detach: Invalid handle.\n"));
        return;
    }

    if (!adf_os_atomic_dec_and_test(&pdev->ref_count)) {
        /* there are other clients still using dmux_dxe */
        return;
    }

    if (pdev->h_hif_dxe) {
        hif_dxe_detach(pdev->h_hif_dxe);
    }

    /*
     * Zero out the pdev object before freeing it.
     * This will make it more obvious if anyone tries to use it
     * after it has been freed.
     */
    adf_os_mem_zero(pdev, sizeof(*pdev));
    adf_os_mem_free(pdev);
    pdev = g_pdev = NULL;

    AR_DEBUG_PRINTF(ATH_DEBUG_TRC, ("-%s\n",__FUNCTION__));
}

A_STATUS
dmux_dxe_register_callback_rx_mgmt(
    dmux_dxe_handle pdev, dmux_dxe_mgmt_cb rx_mgmt_cb, void *rx_mgmt_context)
{
    if ((NULL == pdev) || (pdev != g_pdev)) {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("%s: Invalid handle.\n", __FUNCTION__));
        return A_EINVAL;
    }

    pdev->rx_mgmt_cb = rx_mgmt_cb;
    pdev->rx_mgmt_context = rx_mgmt_context;

    return A_OK;
}

A_STATUS
dmux_dxe_register_callback_rx_ctrl(
    dmux_dxe_handle pdev, dmux_dxe_ctrl_cb rx_ctrl_cb, void *rx_ctrl_context)
{
    if ((NULL == pdev) || (pdev != g_pdev)) {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("%s: Invalid handle.\n", __FUNCTION__));
        return A_EINVAL;
    }

    pdev->rx_ctrl_cb = rx_ctrl_cb;
    pdev->rx_ctrl_context = rx_ctrl_context;

    return A_OK;
}

A_STATUS
dmux_dxe_register_callback_rx_data(
    dmux_dxe_handle pdev, dmux_dxe_data_cb rx_data_cb, void *rx_data_context)
{
    if ((NULL == pdev) || (pdev != g_pdev)) {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("%s: Invalid handle.\n", __FUNCTION__));
        return A_EINVAL;
    }

    pdev->rx_data_cb = rx_data_cb;
    pdev->rx_data_context = rx_data_context;

    return A_OK;
}

A_STATUS
dmux_dxe_register_callback_msg(
    dmux_dxe_handle pdev, dmux_dxe_msg_cb msg_cb, void *msg_context)
{
    if ((NULL == pdev) || (pdev != g_pdev)) {
        AR_DEBUG_PRINTF(ATH_DEBUG_ERR, ("%s: Invalid handle.\n", __FUNCTION__));
        return A_EINVAL;
    }

    pdev->msg_cb = msg_cb;
    pdev->msg_context = msg_context;

    return A_OK;
}
