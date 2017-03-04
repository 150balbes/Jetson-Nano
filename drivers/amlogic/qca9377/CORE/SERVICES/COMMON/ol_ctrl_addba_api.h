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

#ifndef _OL_CTRL_ADDBA_API_H_
#define _OL_CTRL_ADDBA_API_H_
#ifdef QCA_WIFI_ISOC
#include "adf_os_types.h"
#include "adf_os_mem.h"
#include "ol_if_athvar.h"
#include "ol_ctrl_addba_api.h"
#include "wmi_unified.h"
#include "ol_if_athvar.h"
typedef struct S_OL_CTRL_ADDBA_CTX *ol_ctrl_addba_handle;

/**
 * @brief ol_ctrl_addba_attach : Allocate OL ADDBA Context for managing ADDBA Negotiation Process
 *
 * @param[in] void
 *
 * @return ol_txrx_addba_handle
 */
ol_ctrl_addba_handle ol_ctrl_addba_attach(adf_os_device_t osdev,void * ni,A_UINT32 vdev_id,u_int8_t *mac_addr,wmi_unified_t wmi_handle);

/**
 * @brief ol_ctrl_addba_detach : Free OL ADDBA Context and cleanup
 *
 * @param[in]
 *
 * @return void
 */
void ol_ctrl_addba_detach(ol_ctrl_addba_handle ol_addba_handle);

/**
 * @brief ol_ctrl_addba_init : Initialize aggr state for the Tx / Rx
 *
* @param ni      : the Node for which the aggr params need to be initialized

* @return void
*/
void ol_ctrl_addba_init(ol_ctrl_addba_handle ol_addba_handle,adf_os_device_t osdev,void *ni,A_UINT32 vdev_id,u_int8_t *mac_addr,wmi_unified_t wmi_handle);

/**
 * @brief ol_addba_cleanup : DeInitialize aggr state for the Tx / Rx
*
* @param ni      : the Node for which the aggr params need to be De-initialized
*
* @return void
*/
void ol_ctrl_addba_cleanup(ol_ctrl_addba_handle ol_addba_handle);

/**
 * @brief ol_ctrl_addba_request_setup : (Initiator Role) umac needs to call this API to get
 * the Block ack request parameters and use them to construct Addba request.
 *Setsup ADDBA Request Parameters and Start ADDBA Timer for Retry/Cancel ADDBA negotiation.
 *
* @param tidno      : the TID number for this ADDBA request
* @param baparamset : the data structure for lmac to fill in ADDBA request info
* @param batimeout  : the pointer for lmac to fill in ADDBA timeout
* @param basequencectrl : the data structure for lmac to fill in frame sequence control info
* @param buffersize : BAW(block ack window size) for transmit AMPDU.
* @return void
*/
void ol_ctrl_addba_request_setup(ol_ctrl_addba_handle ol_addba_handle,u_int8_t tidno,
                       struct ieee80211_ba_parameterset *baparamset,
                       u_int16_t *batimeout,
                       struct ieee80211_ba_seqctrl *basequencectrl,
                       u_int16_t buffersize);

/**
 * @brief ol_ctrl_addba_response_setup : (Responder Role)Setup ADDBA Response Parameters
 *
* @param tidno      : the TID number for this ADDBA response
* @param dialogtoken: The dialog token for the response (filled by lmac)
* @param statuscode : The status code for this response (filled by lmac)
* @param baparamset : The BA parameters for this response (filled by lmac)
* @param batimeout  : The BA timeout for this response (filled by lmac)
 *
 * @return void
 */
void ol_ctrl_addba_response_setup(ol_ctrl_addba_handle ol_addba_handle,u_int8_t tidno, u_int8_t *dialogtoken,
                       u_int16_t *statuscode,
                       struct ieee80211_ba_parameterset *baparamset,
                       u_int16_t *batimeout);

/**
 * @brief ol_ctrl_addba_request_process : (Responder Role)Process ADDBA Request From Peer
* save response information in per-TID data structure
* umac needs to call this function when it receives addba request.
 *
 * @param dialogtoken : the dialog token saved for future ADDBA response
 * @param baparamset  : pointer to the BA parameters from  received the ADDBA request
 * @param batimeout   : the BA timeout value from ADDBA request
 * @param basequencectrl : the BA sequence control structure from received	ADDBA request
 * @return 0 on success; -EINVAL indicates that aggregation is not enabled.
 *umac needs to send an addba response with success code if the return value is 0.
 */
int  ol_ctrl_addba_request_process(ol_ctrl_addba_handle ol_addba_handle,u_int8_t dialogtoken,
                       struct ieee80211_ba_parameterset *baparamset,
                       u_int16_t batimeout,
                       struct ieee80211_ba_seqctrl basequencectrl);

/**
 * @brief ol_ctrl_addba_response_process : (Initiator Role) Process ADDBA Response and store Negotiated ADDBA Params.
 *                                                       Send WMI Command to FW to Enable Aggregation for TID if all ok.
 *
* @param statuscode : the status code from the received ADDBA response
* @param baparamset : pointer to the BA parameter structure from the received  ADDBA response
* @param batimeout  : the BA timeout value from the received ADDBA response
 *
 * @return void
 */
void  ol_ctrl_addba_response_process(ol_ctrl_addba_handle ol_addba_handle,u_int16_t statuscode,
                       struct ieee80211_ba_parameterset *baparamset,
                       u_int16_t batimeout);

/**
 * @brief ol_ctrl_addba_clear : Clear ADD BA Session for  all the TID's in Session
 *
 * @param[in]
 *
 * @return void
 */
void  ol_ctrl_addba_clear(ol_ctrl_addba_handle ol_addba_handle);

/**
 * @brief ol_ctrl_delba_process : (Responder Role)Process DELBA Request from Peer TID
 *umac needs to call this function when it receives DELBA.
 *
 * @param delbaparamset : point to the DELBA parameter structure
 * @param reasoncode	: reason code from this DELBA
 *
 * @return void
 */
void  ol_ctrl_delba_process(ol_ctrl_addba_handle ol_addba_handle,struct ieee80211_delba_parameterset *delbaparamset,
                        u_int16_t reasoncode);

/**
 * @brief ol_ctrl_addba_get_status : Returns Status of ADDBA Negotiation with Peer for  TID
 *
 * @param tidno : the TID number
 * @return ADDBA status code
 */
u_int16_t ol_ctrl_addba_get_status(ol_ctrl_addba_handle ol_addba_handle, u_int8_t tidno);


void  ol_ctrl_addba_set_response(ol_ctrl_addba_handle ol_addba_handle,u_int8_t tidno,u_int16_t statuscode);

void  ol_ctrl_addba_clear_response(ol_ctrl_addba_handle ol_addba_handle);

#else //QCA_WIFI_ISOC
#define ol_ctrl_addba_attach(a,b,c,d,e)             0
#define ol_ctrl_addba_detach(a)                     0
#define ol_ctrl_addba_init(a,b,c,d,e)               0
#define ol_ctrl_addba_cleanup(a)                    0
#define ol_ctrl_addba_request_setup(a,b,c,d,e,f)    0
#define ol_ctrl_addba_response_setup(a,b,c,d,e,f)   0
#define ol_ctrl_addba_request_process(a,b,c,d,e)    0
#define ol_ctrl_addba_response_process(a,b,c,d)     0
#define ol_ctrl_addba_clear(a)                      0
#define ol_ctrl_delba_process(a,b,c)                0
#define ol_ctrl_addba_get_status(a,b)               0
#define ol_ctrl_addba_set_response(a,b,c);          0
#define ol_ctrl_addba_clear_response(a);            0
#endif //QCA_WIFI_ISOC
#endif
