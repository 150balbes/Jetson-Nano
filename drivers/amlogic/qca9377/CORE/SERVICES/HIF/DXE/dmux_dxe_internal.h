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

#ifndef _DMUX_DXE_INTERNAL__H_
#define _DMUX_DXE_INTERNAL__H_

#include <athdefs.h>     /* A_STATUS */
#include <adf_nbuf.h>    /* adf_nbuf_t */
#include <adf_os_types.h>    /* adf_os_device_t, adf_os_print */
#include <adf_os_atomic.h>

#include <hif_dxe.h>     /* E_HIFDXE_CHANNELTYPE */

struct dmux_dxe_pdev_t {
    adf_os_atomic_t  ref_count;  /* module reference count */
    hif_dxe_handle   h_hif_dxe;
    dmux_dxe_data_cb rx_data_cb;
    dmux_dxe_mgmt_cb rx_mgmt_cb;
    dmux_dxe_ctrl_cb rx_ctrl_cb;
    dmux_dxe_msg_cb  msg_cb;     /* callback for HTT T2H message */
    void             *rx_data_context;
    void             *rx_mgmt_context;
    void             *rx_ctrl_context;
    void             *msg_context;
};

#endif /* _DMUX_DXE_INTERNAL__H_ */
