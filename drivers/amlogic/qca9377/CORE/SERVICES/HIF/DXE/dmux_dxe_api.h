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
 * @file dmux_dxe_api.h
 * @brief Define the API to the dmux_dxe module
 */
#ifndef _DMUX_DXE_API__H_
#define _DMUX_DXE_API__H_

#include <adf_os_types.h> /* adf_os_device_t */
#include <adf_nbuf.h>     /* adf_nbuf_t */

#include <athdefs.h>      /* A_STATUS */

#include "hif_dxe.h"      /* E_HIFDXE_CHANNELTYPE */

/* define an opaque handle to a dmux_dxe object */
struct dmux_dxe_pdev_t;
typedef struct dmux_dxe_pdev_t *dmux_dxe_handle;


dmux_dxe_handle
dmux_dxe_attach(adf_os_device_t osdev);

void dmux_dxe_detach(dmux_dxe_handle pdev);


typedef void (*dmux_dxe_ctrl_cb)(void *context, adf_nbuf_t info);

/**
 * @brief
 * allow clients to register for rx control frame callbacks
 */
A_STATUS
dmux_dxe_register_callback_rx_ctrl(
    dmux_dxe_handle pdev, dmux_dxe_ctrl_cb rx_ctrl_cb, void *rx_ctrl_context);


typedef void (*dmux_dxe_mgmt_cb)(void *context, adf_nbuf_t info);

/**
 * @brief
 * allow clients to register for rx management frame callbacks
 */
A_STATUS
dmux_dxe_register_callback_rx_mgmt(
    dmux_dxe_handle pdev, dmux_dxe_mgmt_cb rx_mgmt_cb, void *rx_mgmt_context);


typedef void
(*dmux_dxe_data_cb)(void *context, adf_nbuf_t info, E_HIFDXE_CHANNELTYPE chan);

/**
 * @brief
 * allow clients to register for rx data frame callbacks
 */
A_STATUS
dmux_dxe_register_callback_rx_data(
    dmux_dxe_handle pdev, dmux_dxe_data_cb rx_data_cb, void *rx_data_context);


typedef void (*dmux_dxe_msg_cb)(void *context, adf_nbuf_t info);

/**
 * @brief
 * allow clients to register for HTT T2H message callbacks
 */
A_STATUS
dmux_dxe_register_callback_msg(
    dmux_dxe_handle pdev, dmux_dxe_msg_cb msg_cb, void *msg_context);


#endif /* _DMUX_DXE_API__H_ */
