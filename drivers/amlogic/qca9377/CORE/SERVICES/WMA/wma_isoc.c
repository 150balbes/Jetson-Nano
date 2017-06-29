/*
 * Copyright (c) 2013-2014 The Linux Foundation. All rights reserved.
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

  \file     wma_isoc.c
  \brief    Implementation of WMA

  ========================================================================*/
/**=========================================================================
  EDIT HISTORY FOR FILE


  This section contains comments describing changes made to the module.
  Notice that changes are listed in reverse chronological order.

  $Header:$   $DateTime: $ $Author: $


  when              who           what, where, why
  --------          ---           -----------------------------------------
  12/03/2013        Ganesh        Implementation of WMA APIs.
                    Kondabattini
  ==========================================================================*/

/* ################ Header files ################ */
#include "wma.h"
#include "wma_api.h"
#include "vos_api.h"
#include "wmi_unified_api.h"
#include "wlan_qct_sys.h"
#include "wniApi.h"
#include "aniGlobal.h"
#include "wmi_unified.h"
#include "wniCfgAp.h"
#include "wlan_hal_cfg.h"
#include "cfgApi.h"
#include "wlan_qct_wdi.h"
/* ############# function definitions ############ */

/* function   : wma_cfg_nv_get_hal_message_buffer
 * Descriptin :
 * Args       :
 * Retruns    :
 */
VOS_STATUS wma_cfg_nv_get_hal_message_buffer(tp_wma_handle wma_handle,
		v_U16_t req_type, v_U16_t buf_len, v_U8_t **msgbuf,
		v_U16_t *data_offset,	v_U16_t *buf_size)
{
	tHalMsgHeader  hal_msg_header;
	VOS_STATUS vos_status = VOS_STATUS_SUCCESS;
	*buf_size = sizeof(hal_msg_header) + buf_len;
	*msgbuf = vos_mem_malloc(*buf_size);
	if (NULL ==  *msgbuf) {
		WMA_LOGP("failed to allocate memory");
		VOS_ASSERT(0);
		vos_status = VOS_STATUS_E_NOMEM;
		goto end;
	}

	hal_msg_header.msgType = req_type;
	hal_msg_header.msgLen = sizeof(hal_msg_header) + buf_len;
	*data_offset = sizeof(hal_msg_header);
	vos_mem_copy(*msgbuf, &hal_msg_header, sizeof(hal_msg_header));
end:
	return vos_status;
}

/* TODO: Cleanup this function */
/*
 * FUNCTION: wma_prepare_config_tlv
 * Function to prepare CFG for DAL(WDA)
 */
VOS_STATUS wma_prepare_config_tlv(v_VOID_t *vos_context,
		t_wma_start_req  *wma_start_params)
{
	/* get mac to acess CFG data base */
	struct sAniSirGlobal *mac = (struct sAniSirGlobal*)vos_get_context(VOS_MODULE_ID_PE, vos_context);
	tp_wma_handle  wma_handle = (tp_wma_handle)vos_get_context(VOS_MODULE_ID_WDA, vos_context);
	tHalCfg        *tlv_struct = NULL ;
	v_U8_t        *tlv_struct_start = NULL ;
	//tANI_U32       str_len = WNI_CFG_STA_ID_LEN;
	tANI_U32        str_len = WNI_CFG_STA_ID_LEN;
	v_PVOID_t      *cfg_param;
	tANI_U32       cfg_param_sz;
	tANI_U32       *cfg_data_val;
	/*TODO: */
#if 0
	WDI_WlanVersionType wcnssCompiledApiVersion;
#endif
	if ((NULL == mac)||(NULL == wma_handle)) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"%s: Invoked with invalid wma_handle or mac", __func__ );
		VOS_ASSERT(0);
		return VOS_STATUS_E_FAILURE;
	}
	cfg_param_sz = (sizeof(tHalCfg) * QWLAN_HAL_CFG_MAX_PARAMS) +
		WNI_CFG_STA_ID_LEN +
		WNI_CFG_EDCA_WME_ACBK_LEN +
		WNI_CFG_EDCA_WME_ACBE_LEN +
		WNI_CFG_EDCA_WME_ACVI_LEN +
		WNI_CFG_EDCA_WME_ACVO_LEN +
		+ (QWLAN_HAL_CFG_INTEGER_PARAM * sizeof(tANI_U32));
	/* malloc memory for all configs in one shot */
	cfg_param = vos_mem_malloc(cfg_param_sz);

	if(NULL == cfg_param ) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"%s:cfg_param is NULL", __func__);
		VOS_ASSERT(0) ;
		return VOS_STATUS_E_NOMEM;
	}
	vos_mem_set(cfg_param, cfg_param_sz, 0);
	wma_start_params->pConfigBuffer = cfg_param;
	tlv_struct = (tHalCfg *)cfg_param;
	tlv_struct_start = (v_U8_t *)cfg_param;
	/* TODO: Remove Later */
	/* QWLAN_HAL_CFG_STA_ID */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_STA_ID;
	cfg_data_val = (tANI_U32*)((v_U8_t *) tlv_struct + sizeof(tHalCfg));
	if(wlan_cfgGetStr(mac, WNI_CFG_STA_ID, (v_U8_t*)cfg_data_val, &str_len) !=
			eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_STA_ID");
		goto handle_failure;
	}
	tlv_struct->uCfgLen = str_len ;
	/* calculate the pad bytes to have the CFG in aligned format */
	tlv_struct->uCfgPadBytes = ALIGNED_WORD_SIZE -
		(tlv_struct->uCfgLen & (ALIGNED_WORD_SIZE - 1));
	tlv_struct = (tHalCfg *)(( (v_U8_t *) tlv_struct
				+ sizeof(tHalCfg) + tlv_struct->uCfgLen + tlv_struct->uCfgPadBytes)) ;
	/* QWLAN_HAL_CFG_CURRENT_TX_ANTENNA */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_CURRENT_TX_ANTENNA;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_CURRENT_TX_ANTENNA, cfg_data_val )
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_CURRENT_TX_ANTENNA");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)(( (v_U8_t *) tlv_struct
				+ sizeof(tHalCfg) + tlv_struct->uCfgLen)) ;
	/* QWLAN_HAL_CFG_CURRENT_RX_ANTENNA */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_CURRENT_RX_ANTENNA;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_CURRENT_RX_ANTENNA, cfg_data_val) !=
			eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_CURRENT_RX_ANTENNA");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)(( (v_U8_t *) tlv_struct
				+ sizeof(tHalCfg) + tlv_struct->uCfgLen)) ;
	/* QWLAN_HAL_CFG_LOW_GAIN_OVERRIDE */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_LOW_GAIN_OVERRIDE;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_LOW_GAIN_OVERRIDE, cfg_data_val )
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_LOW_GAIN_OVERRIDE");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)(( (v_U8_t *) tlv_struct
				+ sizeof(tHalCfg) + tlv_struct->uCfgLen)) ;

	/* QWLAN_HAL_CFG_POWER_STATE_PER_CHAIN */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_POWER_STATE_PER_CHAIN;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_POWER_STATE_PER_CHAIN,
				cfg_data_val ) != eSIR_SUCCESS)	{
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_POWER_STATE_PER_CHAIN");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)(( (v_U8_t *) tlv_struct
				+ sizeof(tHalCfg) + tlv_struct->uCfgLen));
	/* QWLAN_HAL_CFG_CAL_PERIOD */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_CAL_PERIOD;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_CAL_PERIOD, cfg_data_val )
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_CAL_PERIOD");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)(( (v_U8_t *) tlv_struct
				+ sizeof(tHalCfg) + tlv_struct->uCfgLen));
	/* QWLAN_HAL_CFG_CAL_CONTROL  */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_CAL_CONTROL ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_CAL_CONTROL, cfg_data_val )
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_CAL_CONTROL");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)(( (v_U8_t *) tlv_struct
				+ sizeof(tHalCfg) + tlv_struct->uCfgLen));
	/* QWLAN_HAL_CFG_PROXIMITY  */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_PROXIMITY ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_PROXIMITY, cfg_data_val )
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_PROXIMITY");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)(( (v_U8_t *) tlv_struct
				+ sizeof(tHalCfg) + tlv_struct->uCfgLen)) ;
	/* QWLAN_HAL_CFG_NETWORK_DENSITY  */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_NETWORK_DENSITY ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_NETWORK_DENSITY, cfg_data_val )
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_NETWORK_DENSITY");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)(( (v_U8_t *) tlv_struct
				+ sizeof(tHalCfg) + tlv_struct->uCfgLen));
	/* QWLAN_HAL_CFG_MAX_MEDIUM_TIME  */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_MAX_MEDIUM_TIME ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_MAX_MEDIUM_TIME, cfg_data_val ) !=
			eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_MAX_MEDIUM_TIME");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)(( (v_U8_t *) tlv_struct
				+ sizeof(tHalCfg) + tlv_struct->uCfgLen));
	/* QWLAN_HAL_CFG_MAX_MPDUS_IN_AMPDU   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_MAX_MPDUS_IN_AMPDU  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_MAX_MPDUS_IN_AMPDU,
				cfg_data_val ) != eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_MAX_MPDUS_IN_AMPDU");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)(( (v_U8_t *) tlv_struct
				+ sizeof(tHalCfg) + tlv_struct->uCfgLen)) ;
	/* QWLAN_HAL_CFG_RTS_THRESHOLD   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_RTS_THRESHOLD  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_RTS_THRESHOLD, cfg_data_val ) !=
			eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_RTS_THRESHOLD");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)(( (v_U8_t *) tlv_struct
				+ sizeof(tHalCfg) + tlv_struct->uCfgLen));
	/* QWLAN_HAL_CFG_SHORT_RETRY_LIMIT   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_SHORT_RETRY_LIMIT  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_SHORT_RETRY_LIMIT, cfg_data_val ) !=
			eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_SHORT_RETRY_LIMIT");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)(( (v_U8_t *) tlv_struct
				+ sizeof(tHalCfg) + tlv_struct->uCfgLen)) ;
	/* QWLAN_HAL_CFG_LONG_RETRY_LIMIT   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_LONG_RETRY_LIMIT  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_LONG_RETRY_LIMIT, cfg_data_val ) !=
			eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_LONG_RETRY_LIMIT");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)(( (v_U8_t *) tlv_struct
				+ sizeof(tHalCfg) + tlv_struct->uCfgLen)) ;
	/* QWLAN_HAL_CFG_FRAGMENTATION_THRESHOLD   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_FRAGMENTATION_THRESHOLD  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_FRAGMENTATION_THRESHOLD,
				cfg_data_val ) != eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_FRAGMENTATION_THRESHOLD");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)(( (v_U8_t *) tlv_struct
				+ sizeof(tHalCfg) + tlv_struct->uCfgLen)) ;
	/* QWLAN_HAL_CFG_DYNAMIC_THRESHOLD_ZERO   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_DYNAMIC_THRESHOLD_ZERO  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_DYNAMIC_THRESHOLD_ZERO,
				cfg_data_val ) != eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_DYNAMIC_THRESHOLD_ZERO");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)(( (v_U8_t *) tlv_struct
				+ sizeof(tHalCfg) + tlv_struct->uCfgLen));

	/* QWLAN_HAL_CFG_DYNAMIC_THRESHOLD_ONE   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_DYNAMIC_THRESHOLD_ONE  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_DYNAMIC_THRESHOLD_ONE,
				cfg_data_val ) != eSIR_SUCCESS)	{
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_DYNAMIC_THRESHOLD_ONE");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)(( (v_U8_t *) tlv_struct
				+ sizeof(tHalCfg) + tlv_struct->uCfgLen));
	/* QWLAN_HAL_CFG_DYNAMIC_THRESHOLD_TWO   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_DYNAMIC_THRESHOLD_TWO  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_DYNAMIC_THRESHOLD_TWO,
				cfg_data_val ) != eSIR_SUCCESS)	{
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_DYNAMIC_THRESHOLD_TWO");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)(( (v_U8_t *) tlv_struct
				+ sizeof(tHalCfg) + tlv_struct->uCfgLen));

	/* QWLAN_HAL_CFG_FIXED_RATE   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_FIXED_RATE  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_FIXED_RATE, cfg_data_val)
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_FIXED_RATE");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)(( (v_U8_t *) tlv_struct
				+ sizeof(tHalCfg) + tlv_struct->uCfgLen));

	/* QWLAN_HAL_CFG_RETRYRATE_POLICY   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_RETRYRATE_POLICY  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_RETRYRATE_POLICY, cfg_data_val )
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_RETRYRATE_POLICY");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)(( (v_U8_t *) tlv_struct
				+ sizeof(tHalCfg) + tlv_struct->uCfgLen));

	/* QWLAN_HAL_CFG_RETRYRATE_SECONDARY   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_RETRYRATE_SECONDARY  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_RETRYRATE_SECONDARY,
				cfg_data_val ) != eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_RETRYRATE_SECONDARY");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)(( (v_U8_t *) tlv_struct
				+ sizeof(tHalCfg) + tlv_struct->uCfgLen)) ;
	/* QWLAN_HAL_CFG_RETRYRATE_TERTIARY   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_RETRYRATE_TERTIARY  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_RETRYRATE_TERTIARY,
				cfg_data_val ) != eSIR_SUCCESS)	{
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_RETRYRATE_TERTIARY");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)(( (v_U8_t *) tlv_struct
				+ sizeof(tHalCfg) + tlv_struct->uCfgLen)) ;
	/* QWLAN_HAL_CFG_FORCE_POLICY_PROTECTION   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_FORCE_POLICY_PROTECTION  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_FORCE_POLICY_PROTECTION,
				cfg_data_val ) != eSIR_SUCCESS)	{
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_FORCE_POLICY_PROTECTION");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen);
	/* QWLAN_HAL_CFG_FIXED_RATE_MULTICAST_24GHZ   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_FIXED_RATE_MULTICAST_24GHZ  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_FIXED_RATE_MULTICAST_24GHZ,
				cfg_data_val ) != eSIR_SUCCESS)	{
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_FIXED_RATE_MULTICAST_24GHZ");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen);
	/* QWLAN_HAL_CFG_FIXED_RATE_MULTICAST_5GHZ   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_FIXED_RATE_MULTICAST_5GHZ  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_FIXED_RATE_MULTICAST_5GHZ,
				cfg_data_val ) != eSIR_SUCCESS)	{
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_FIXED_RATE_MULTICAST_5GHZ");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen);

	/* QWLAN_HAL_CFG_DEFAULT_RATE_INDEX_24GHZ   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_DEFAULT_RATE_INDEX_24GHZ  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_DEFAULT_RATE_INDEX_24GHZ,
				cfg_data_val ) != eSIR_SUCCESS)
	{
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_DEFAULT_RATE_INDEX_24GHZ");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen);

	/* QWLAN_HAL_CFG_DEFAULT_RATE_INDEX_5GHZ   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_DEFAULT_RATE_INDEX_5GHZ  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_DEFAULT_RATE_INDEX_5GHZ,
				cfg_data_val ) != eSIR_SUCCESS)	{
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_DEFAULT_RATE_INDEX_5GHZ");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen);
	/* QWLAN_HAL_CFG_MAX_BA_SESSIONS   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_MAX_BA_SESSIONS  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_MAX_BA_SESSIONS, cfg_data_val ) !=
			eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_MAX_BA_SESSIONS");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen);

	/* QWLAN_HAL_CFG_PS_DATA_INACTIVITY_TIMEOUT   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_PS_DATA_INACTIVITY_TIMEOUT  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_PS_DATA_INACTIVITY_TIMEOUT,
				cfg_data_val ) != eSIR_SUCCESS)	{
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_PS_DATA_INACTIVITY_TIMEOUT");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen);
	/* QWLAN_HAL_CFG_PS_ENABLE_BCN_FILTER   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_PS_ENABLE_BCN_FILTER  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_PS_ENABLE_BCN_FILTER,
				cfg_data_val ) != eSIR_SUCCESS)	{
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_PS_ENABLE_BCN_FILTER");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen);
	/* QWLAN_HAL_CFG_PS_ENABLE_RSSI_MONITOR   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_PS_ENABLE_RSSI_MONITOR  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_PS_ENABLE_RSSI_MONITOR,
				cfg_data_val ) != eSIR_SUCCESS)	{
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_PS_ENABLE_RSSI_MONITOR");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen);
	/* QWLAN_HAL_CFG_NUM_BEACON_PER_RSSI_AVERAGE   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_NUM_BEACON_PER_RSSI_AVERAGE  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_NUM_BEACON_PER_RSSI_AVERAGE,
				cfg_data_val ) != eSIR_SUCCESS)	{
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_NUM_BEACON_PER_RSSI_AVERAGE");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen);

	/* QWLAN_HAL_CFG_STATS_PERIOD   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_STATS_PERIOD  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_STATS_PERIOD, cfg_data_val ) !=
			eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_STATS_PERIOD");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen);
	/* QWLAN_HAL_CFG_CFP_MAX_DURATION   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_CFP_MAX_DURATION  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_CFP_MAX_DURATION, cfg_data_val ) !=
			eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_CFP_MAX_DURATION");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;
	/* QWLAN_HAL_CFG_FRAME_TRANS_ENABLED */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_FRAME_TRANS_ENABLED  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	vos_mem_copy(cfg_data_val, &wma_handle->frameTransRequired,
			sizeof(tANI_U32));
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;
	/* QWLAN_HAL_CFG_DTIM_PERIOD */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_DTIM_PERIOD  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_DTIM_PERIOD, cfg_data_val)
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_DTIM_PERIOD");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;
	/* QWLAN_HAL_CFG_EDCA_WMM_ACBK */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_EDCA_WMM_ACBK  ;
	str_len = WNI_CFG_EDCA_WME_ACBK_LEN;
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetStr(mac, WNI_CFG_EDCA_WME_ACBK, (v_U8_t *)cfg_data_val,
				&str_len) != eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_EDCA_WME_ACBK");
		goto handle_failure;
	}
	tlv_struct->uCfgLen = str_len;
	/* calculate the pad bytes to have the CFG in aligned format */
	tlv_struct->uCfgPadBytes = ALIGNED_WORD_SIZE -
		(tlv_struct->uCfgLen & (ALIGNED_WORD_SIZE - 1));
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen + tlv_struct->uCfgPadBytes) ;
	/* QWLAN_HAL_CFG_EDCA_WMM_ACBE */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_EDCA_WMM_ACBE  ;
	str_len = WNI_CFG_EDCA_WME_ACBE_LEN;
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetStr(mac, WNI_CFG_EDCA_WME_ACBE, (v_U8_t *)cfg_data_val,
				&str_len) != eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_EDCA_WME_ACBE");
		goto handle_failure;
	}
	tlv_struct->uCfgLen = str_len;
	/* calculate the pad bytes to have the CFG in aligned format */
	tlv_struct->uCfgPadBytes = ALIGNED_WORD_SIZE -
		(tlv_struct->uCfgLen & (ALIGNED_WORD_SIZE - 1));
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen + tlv_struct->uCfgPadBytes) ;
	/* QWLAN_HAL_CFG_EDCA_WMM_ACVI */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_EDCA_WMM_ACVO  ;
	str_len = WNI_CFG_EDCA_WME_ACVI_LEN;
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetStr(mac, WNI_CFG_EDCA_WME_ACVO, (v_U8_t *)cfg_data_val,
				&str_len) != eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_EDCA_WME_ACVI");
		goto handle_failure;
	}
	tlv_struct->uCfgLen = str_len;
	/* calculate the pad bytes to have the CFG in aligned format */
	tlv_struct->uCfgPadBytes = ALIGNED_WORD_SIZE -
		(tlv_struct->uCfgLen & (ALIGNED_WORD_SIZE - 1));
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen + tlv_struct->uCfgPadBytes) ;
	/* QWLAN_HAL_CFG_EDCA_WMM_ACVO */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_EDCA_WMM_ACVI  ;
	str_len = WNI_CFG_EDCA_WME_ACVO_LEN;
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetStr(mac, WNI_CFG_EDCA_WME_ACVI, (v_U8_t *)cfg_data_val,
				&str_len) != eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_EDCA_WME_ACVO");
		goto handle_failure;
	}
	tlv_struct->uCfgLen = str_len;
	/* calculate the pad bytes to have the CFG in aligned format */
	tlv_struct->uCfgPadBytes = ALIGNED_WORD_SIZE -
		(tlv_struct->uCfgLen & (ALIGNED_WORD_SIZE - 1));
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen + tlv_struct->uCfgPadBytes) ;
	/* QWLAN_HAL_CFG_BA_THRESHOLD_HIGH */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_BA_THRESHOLD_HIGH  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_BA_THRESHOLD_HIGH, cfg_data_val)
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_BA_THRESHOLD_HIGH");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;
	/* QWLAN_HAL_CFG_MAX_BA_BUFFERS */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_MAX_BA_BUFFERS  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_MAX_BA_BUFFERS, cfg_data_val)
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_MAX_BA_BUFFERS");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;
	/* QWLAN_HAL_CFG_DYNAMIC_PS_POLL_VALUE */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_DYNAMIC_PS_POLL_VALUE  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_DYNAMIC_PS_POLL_VALUE, cfg_data_val)
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_DYNAMIC_PS_POLL_VALUE");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;
	/* QWLAN_HAL_CFG_TELE_BCN_TRANS_LI */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_TELE_BCN_TRANS_LI  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_TELE_BCN_TRANS_LI, cfg_data_val)
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_TELE_BCN_TRANS_LI");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;
	/* QWLAN_HAL_CFG_TELE_BCN_TRANS_LI_IDLE_BCNS */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_TELE_BCN_TRANS_LI_IDLE_BCNS  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_TELE_BCN_TRANS_LI_IDLE_BCNS, cfg_data_val)
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_TELE_BCN_TRANS_LI_IDLE_BCNS");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;
	/* QWLAN_HAL_CFG_TELE_BCN_MAX_LI */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_TELE_BCN_MAX_LI  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_TELE_BCN_MAX_LI, cfg_data_val)
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_TELE_BCN_MAX_LI");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;
	/* QWLAN_HAL_CFG_TELE_BCN_MAX_LI_IDLE_BCNS */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_TELE_BCN_MAX_LI_IDLE_BCNS  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_TELE_BCN_MAX_LI_IDLE_BCNS, cfg_data_val)
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_TELE_BCN_MAX_LI_IDLE_BCNS");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;
	/* QWLAN_HAL_CFG_TELE_BCN_WAKEUP_EN */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_TELE_BCN_WAKEUP_EN  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_TELE_BCN_WAKEUP_EN, cfg_data_val)
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_TELE_BCN_WAKEUP_EN");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;
	/* QWLAN_HAL_CFG_INFRA_STA_KEEP_ALIVE_PERIOD */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_INFRA_STA_KEEP_ALIVE_PERIOD  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_INFRA_STA_KEEP_ALIVE_PERIOD, cfg_data_val)
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_INFRA_STA_KEEP_ALIVE_PERIOD");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;
	/*QWLAN_HAL_CFG_TX_PWR_CTRL_ENABLE*/
	tlv_struct->uCfgId = QWLAN_HAL_CFG_TX_PWR_CTRL_ENABLE  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_TX_PWR_CTRL_ENABLE, cfg_data_val)
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_TX_PWR_CTRL_ENABLE");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;
	/* QWLAN_HAL_CFG_ENABLE_CLOSE_LOOP   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_ENABLE_CLOSE_LOOP  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_ENABLE_CLOSE_LOOP, cfg_data_val)
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_ENABLE_CLOSE_LOOP");
		goto handle_failure;
	}
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;
	/* [COEX] strictly speaking, the Coex parameters are not part of the WLAN_CFG_FILE binary,
	 * but are from the WLAN_INI_FILE file.  However, this is the only parameter download routine
	 * into FW, so the parameters are added here.
	 */
	/* [COEX] QWLAN_HAL_CFG_BTC_EXECUTION_MODE */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_BTC_EXECUTION_MODE  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	*cfg_data_val = mac->btc.btcConfig.btcExecutionMode;
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;
	/* [COEX] QWLAN_HAL_CFG_BTC_DHCP_BT_SLOTS_TO_BLOCK */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_BTC_DHCP_BT_SLOTS_TO_BLOCK  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	*cfg_data_val = mac->btc.btcConfig.btcConsBtSlotsToBlockDuringDhcp;
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;
	/* [COEX] QWLAN_HAL_CFG_BTC_A2DP_DHCP_BT_SUB_INTERVALS */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_BTC_A2DP_DHCP_BT_SUB_INTERVALS  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	*cfg_data_val = mac->btc.btcConfig.btcA2DPBtSubIntervalsDuringDhcp;
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;
	/* [COEX] QWLAN_HAL_CFG_BTC_STATIC_LEN_INQ_BT */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_BTC_STATIC_LEN_INQ_BT  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	*cfg_data_val = mac->btc.btcConfig.btcStaticLenInqBt;
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;

	/* [COEX] QWLAN_HAL_CFG_BTC_STATIC_LEN_PAGE_BT */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_BTC_STATIC_LEN_PAGE_BT  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	*cfg_data_val = mac->btc.btcConfig.btcStaticLenPageBt;
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;

	/* [COEX] QWLAN_HAL_CFG_BTC_STATIC_LEN_CONN_BT */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_BTC_STATIC_LEN_CONN_BT  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	*cfg_data_val = mac->btc.btcConfig.btcStaticLenConnBt;
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;

	/* [COEX] QWLAN_HAL_CFG_BTC_STATIC_LEN_LE_BT */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_BTC_STATIC_LEN_LE_BT  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	*cfg_data_val = mac->btc.btcConfig.btcStaticLenLeBt;
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;

	/* [COEX] QWLAN_HAL_CFG_BTC_STATIC_LEN_INQ_WLAN */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_BTC_STATIC_LEN_INQ_WLAN  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	*cfg_data_val = mac->btc.btcConfig.btcStaticLenInqWlan;
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;

	/* [COEX] QWLAN_HAL_CFG_BTC_STATIC_LEN_PAGE_WLAN */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_BTC_STATIC_LEN_PAGE_WLAN  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	*cfg_data_val = mac->btc.btcConfig.btcStaticLenPageWlan;
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;

	/* [COEX] QWLAN_HAL_CFG_BTC_STATIC_LEN_CONN_WLAN */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_BTC_STATIC_LEN_CONN_WLAN  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	*cfg_data_val = mac->btc.btcConfig.btcStaticLenConnWlan;
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;

	/* [COEX] QWLAN_HAL_CFG_BTC_STATIC_LEN_LE_WLAN */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_BTC_STATIC_LEN_LE_WLAN  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	*cfg_data_val = mac->btc.btcConfig.btcStaticLenLeWlan;
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;

	/* [COEX] QWLAN_HAL_CFG_BTC_DYN_MAX_LEN_BT */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_BTC_DYN_MAX_LEN_BT  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	*cfg_data_val = mac->btc.btcConfig.btcDynMaxLenBt;
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;

	/* [COEX] QWLAN_HAL_CFG_BTC_DYN_MAX_LEN_WLAN */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_BTC_DYN_MAX_LEN_WLAN  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	*cfg_data_val = mac->btc.btcConfig.btcDynMaxLenWlan;
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;

	/* [COEX] QWLAN_HAL_CFG_BTC_MAX_SCO_BLOCK_PERC */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_BTC_MAX_SCO_BLOCK_PERC  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	*cfg_data_val = mac->btc.btcConfig.btcMaxScoBlockPerc;
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;

	/* [COEX] QWLAN_HAL_CFG_BTC_DHCP_PROT_ON_A2DP */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_BTC_DHCP_PROT_ON_A2DP  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	*cfg_data_val = mac->btc.btcConfig.btcDhcpProtOnA2dp;
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;

	/* [COEX] QWLAN_HAL_CFG_BTC_DHCP_PROT_ON_SCO */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_BTC_DHCP_PROT_ON_SCO  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	*cfg_data_val = mac->btc.btcConfig.btcDhcpProtOnSco;
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;
	/*TODO: what to send ?*/
#if 0
	/* QWLAN_HAL_CFG_WCNSS_API_VERSION */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_WCNSS_API_VERSION  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	WDI_GetWcnssCompiledApiVersion(&wcnssCompiledApiVersion);
	*cfg_data_val = WLAN_HAL_CONSTRUCT_API_VERSION(wcnssCompiledApiVersion.major,
			wcnssCompiledApiVersion.minor,
			wcnssCompiledApiVersion.version,
			wcnssCompiledApiVersion.revision);
	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;
#endif
	/* QWLAN_HAL_CFG_AP_KEEPALIVE_TIMEOUT   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_AP_KEEPALIVE_TIMEOUT  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_AP_KEEP_ALIVE_TIMEOUT,
				cfg_data_val ) != eSIR_SUCCESS)	{
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_AP_KEEP_ALIVE_TIMEOUT");
		goto handle_failure;
	}

	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;
	/* QWLAN_HAL_CFG_GO_KEEPALIVE_TIMEOUT   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_GO_KEEPALIVE_TIMEOUT  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_GO_KEEP_ALIVE_TIMEOUT,
				cfg_data_val ) != eSIR_SUCCESS)	{
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_GO_KEEP_ALIVE_TIMEOUT");
		goto handle_failure;
	}

	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;

	/* QWLAN_HAL_CFG_ENABLE_MC_ADDR_LIST */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_ENABLE_MC_ADDR_LIST;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_ENABLE_MC_ADDR_LIST, cfg_data_val)
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_ENABLE_MC_ADDR_LIST");
		goto handle_failure;
	}

	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;

	/* QWLAN_HAL_CFG_ENABLE_UNICAST_FILTER */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_ENABLE_UNICAST_FILTER;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_ENABLE_UC_FILTER, cfg_data_val)
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_ENABLE_UC_FILTER");
		goto handle_failure;
	}

	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;
#ifdef WLAN_SOFTAP_VSTA_FEATURE
	tlv_struct->uCfgId = QWLAN_HAL_CFG_MAX_ASSOC_LIMIT;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_ASSOC_STA_LIMIT, cfg_data_val)
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_ASSOC_STA_LIMIT");
		goto handle_failure;
	}

	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;
#endif
	/* QWLAN_HAL_CFG_ENABLE_LPWR_IMG_TRANSITION   */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_ENABLE_LPWR_IMG_TRANSITION  ;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);
	if(wlan_cfgGetInt(mac, WNI_CFG_ENABLE_LPWR_IMG_TRANSITION, cfg_data_val)
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_ENABLE_LPWR_IMG_TRANSITION");
		goto handle_failure;
	}

	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;

	/* QWLAN_HAL_CFG_ENABLE_MCC_ADAPTIVE_SCHEDULER */
	tlv_struct->uCfgId = QWLAN_HAL_CFG_ENABLE_MCC_ADAPTIVE_SCHEDULER;
	tlv_struct->uCfgLen = sizeof(tANI_U32);
	cfg_data_val = (tANI_U32 *)(tlv_struct + 1);

	if(wlan_cfgGetInt(mac, WNI_CFG_ENABLE_MCC_ADAPTIVE_SCHED, cfg_data_val)
			!= eSIR_SUCCESS) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"Failed to get value for WNI_CFG_ENABLE_MCC_ADAPTIVE_SCHED");
		goto handle_failure;
	}

	tlv_struct = (tHalCfg *)( (v_U8_t *) tlv_struct
			+ sizeof(tHalCfg) + tlv_struct->uCfgLen) ;

	wma_start_params->usConfigBufferLen = (v_U8_t *)tlv_struct - tlv_struct_start ;
#ifdef WLAN_DEBUG
	{
		int i;
		VOS_TRACE(VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_INFO,
				"****** Dumping CFG TLV ***** ");
		for (i=0; (i+7) < wma_start_params->usConfigBufferLen; i+=8) {
			VOS_TRACE(VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_INFO,
					"%02x %02x %02x %02x %02x %02x %02x %02x",
					tlv_struct_start[i],
					tlv_struct_start[i+1],
					tlv_struct_start[i+2],
					tlv_struct_start[i+3],
					tlv_struct_start[i+4],
					tlv_struct_start[i+5],
					tlv_struct_start[i+6],
					tlv_struct_start[i+7]);
		}
		/* Dump the bytes in the last line*/
		for (; i < wma_start_params->usConfigBufferLen; i++) {
			VOS_TRACE(VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_INFO,
					"%02x ",tlv_struct_start[i]);
		}
		VOS_TRACE(VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_INFO,
				"**************************** ");
	}
#endif
	return VOS_STATUS_SUCCESS ;
handle_failure:
	vos_mem_free(cfg_param);
	return VOS_STATUS_E_FAILURE;
}

/* function   : wma_get_hal_drv_type
 * Descriptin :
 * Args       :
 * Retruns    :
 */
tDriverType wma_get_hal_drv_type(t_wma_drv_type wma_driver_type)
{
	switch (wma_driver_type) {
		case WMA_DRIVER_TYPE_PRODUCTION:
			return eDRIVER_TYPE_PRODUCTION;
		case WMA_DRIVER_TYPE_MFG:
			return eDRIVER_TYPE_MFG;
		case WMA_DRIVER_TYPE_DVT:
			return eDRIVER_TYPE_DVT;
		default:
			return 0xFF;
	}
}

/* function   : wma_cfg_download_isoc
 * Descriptin :
 * Args       :
 * Retruns    :
 */
VOS_STATUS wma_cfg_download_isoc(v_VOID_t *vos_context, tp_wma_handle wma_handle)
{
	A_STATUS status = A_OK;
	VOS_STATUS vos_status = VOS_STATUS_SUCCESS;
	t_wma_start_req wma_start_param = {0};
	tHalMacStartReqMsg halStartReq;
	v_U8_t *pSendBuffer = NULL;
	v_U16_t usDataOffset = 0;
	v_U16_t usSendSize = 0;
	v_U16_t usLen = 0;
	adf_nbuf_t msg = NULL;
	v_U32_t *msg_word = NULL;
	HTC_PACKET htc_packet;

	vos_mem_set(&wma_start_param, sizeof(wma_start_param), 0);
	wma_start_param.driver_type = wma_handle->driver_type;
	/* prepare the config TLV for the WDI */
	vos_status = wma_prepare_config_tlv(vos_context, &wma_start_param);
	if ( !VOS_IS_STATUS_SUCCESS(vos_status) ) {
		VOS_TRACE( VOS_MODULE_ID_WDA, VOS_TRACE_LEVEL_ERROR,
				"%s: Unable to prepare Config TLV", __func__ );
		return VOS_STATUS_E_FAILURE;
	}

	/* Fill hal msg*/
	/* halMsgHeader+driverType+configbuffer*/
	/*-----------------------------------------------------------------------
	  Get message buffer
	  -----------------------------------------------------------------------*/
	usLen = sizeof(halStartReq.startReqParams) +
		wma_start_param.usConfigBufferLen;

	if (( VOS_STATUS_SUCCESS != wma_cfg_nv_get_hal_message_buffer( wma_handle, FW_CFG_DOWNLOAD_REQ,
					usLen,
					&pSendBuffer, &usDataOffset, &usSendSize))||
			( usSendSize < (usDataOffset + usLen ))) {
		vos_status = VOS_STATUS_E_FAILURE;
		VOS_ASSERT(0);
		goto end;
	}

	/*-----------------------------------------------------------------------
	  Fill in the message
	  -----------------------------------------------------------------------*/
	halStartReq.startReqParams.driverType =	wma_get_hal_drv_type(wma_start_param.driver_type);

	halStartReq.startReqParams.uConfigBufferLen =
		wma_start_param.usConfigBufferLen;
	vos_mem_copy( pSendBuffer+usDataOffset,
			&halStartReq.startReqParams,
			sizeof(halStartReq.startReqParams));

	usDataOffset  += sizeof(halStartReq.startReqParams);
	vos_mem_copy( pSendBuffer+usDataOffset,
			wma_start_param.pConfigBuffer,
			wma_start_param.usConfigBufferLen);


	/* Fill HTC packet */
	msg = adf_nbuf_alloc(
			wma_handle->adf_dev,
			usSendSize,
			/* reserve room for HTC header */
			HTC_HEADER_LEN + HTC_HDR_ALIGNMENT_PADDING, 4, FALSE);
	if (!msg) {
		vos_status = VOS_STATUS_E_NOMEM;
		goto end;
	}
	/* set the length of the message */
	adf_nbuf_put_tail(msg, usSendSize);
	/* fill in the message contents */
	msg_word = (v_U32_t *) adf_nbuf_data(msg);
	vos_mem_copy(msg_word, pSendBuffer, usSendSize);
	/* rewind beyond alignment pad to get to the HTC header reserved area */
	adf_nbuf_push_head(msg, HTC_HDR_ALIGNMENT_PADDING);


	SET_HTC_PACKET_INFO_TX(
			&htc_packet,
			wma_handle,
			adf_nbuf_data(msg),
			adf_nbuf_len(msg),
			wma_handle->cfg_nv.endpoint_id,
			0);

	SET_HTC_PACKET_NET_BUF_CONTEXT(&htc_packet, msg);

	vos_event_reset(&wma_handle->cfg_nv_tx_complete);
	status = HTCSendPkt(wma_handle->htc_handle, &htc_packet);
	if(status != A_OK) {
		WMA_LOGE("download cfg failed!\n");
		vos_status = VOS_STATUS_E_FAILURE;
		goto end;
	}

	// wait for cfg sending complete
	vos_status = vos_wait_single_event( &(wma_handle->cfg_nv_tx_complete),
					WMA_CFG_NV_DNLD_TIMEOUT);

end:
	if(wma_start_param.pConfigBuffer != NULL) {
		vos_mem_free(wma_start_param.pConfigBuffer);
		wma_start_param.pConfigBuffer = NULL;
	}
	if(pSendBuffer)	{
		vos_mem_free(pSendBuffer);
		pSendBuffer = NULL;
	}
	return vos_status;
}

/* function   : wma_cfg_nv_download_complete
 * Descriptin :
 * Args       :
 * Retruns    :
 */
v_VOID_t wma_cfg_nv_download_complete(WMA_HANDLE handle, HTC_PACKET *htc_pkt)
{
	adf_nbuf_t msg = (adf_nbuf_t)GET_HTC_PACKET_NET_BUF_CONTEXT(htc_pkt);
	tp_wma_handle wma_handle = (tp_wma_handle)(htc_pkt->pPktContext);

	WMA_LOGA("#### cfg_nv tx complete! ####\n");

	VOS_ASSERT(msg);
	adf_nbuf_free(msg);

	/* release mutex*/
	VOS_ASSERT(wma_handle);

	if (vos_event_set(&wma_handle->cfg_nv_tx_complete)!= VOS_STATUS_SUCCESS) {
		WMA_LOGE("Failed to set the event");
	}
	return;
}

void wma_cfg_nv_rx(WMA_HANDLE handle, HTC_PACKET *htc_pkt)
{
	adf_nbuf_t rx_buf = (adf_nbuf_t)htc_pkt->pPktContext;
	void *msg = (void *)adf_nbuf_data(rx_buf);
	tHalMsgHeader *pMsgHeader = (tHalMsgHeader *)msg;
	tp_wma_handle wma_handle = (tp_wma_handle)(handle);
	tHalMacStopRspMsg *pHalStopRsp = NULL;
	tHalMacStartRspMsg *pHalStartRsp = NULL;
	tHalNvImgDownloadRspMsg *pHalNvDnldRsp = NULL;

	bool bReleaselock = true;

	if (!msg || pMsgHeader->msgLen == 0) {
		WMA_LOGE(" Receive NULL msg!\n");
		return;
	}

	/* handle each RSP ID*/
	switch (pMsgHeader->msgType) {
	case WLAN_HAL_START_RSP:
		/* save FW version here*/
		pHalStartRsp = (tHalMacStartRspMsg *)msg;
		WMA_LOGA("Received HalStart RSP!\n"
			"status = %d\n"
			"wcnssCrmVersionString=%s\n"
			"wcnssWlanVersionString=%s\n",
			pHalStartRsp->startRspParams.status,
			pHalStartRsp->startRspParams.wcnssCrmVersionString,
			pHalStartRsp->startRspParams.wcnssWlanVersionString);
		break;
	case WLAN_HAL_STOP_RSP:
		pHalStopRsp = (tHalMacStopRspMsg *)msg;
		WMA_LOGA("Received HalStop RSP with status = %u\n",
				pHalStopRsp->stopRspParams.status);
		break;
	case WLAN_HAL_DOWNLOAD_NV_RSP:
		pHalNvDnldRsp = (tHalNvImgDownloadRspMsg *)msg;
		WMA_LOGA("Received NV download RSP with status = %u\n",
				pHalNvDnldRsp->nvImageRspParams.status);
		break;
	default:
		WMA_LOGA("Received Unknown HAL response = %d\n",
				pMsgHeader->msgType);
		bReleaselock = false;
		break;
	}

	/* free rx buffer*/
	if (rx_buf)
		adf_nbuf_free(rx_buf);

	VOS_ASSERT(wma_handle);
	if ((bReleaselock != false) &&
		(vos_event_set(&wma_handle->cfg_nv_rx_complete)
					!= VOS_STATUS_SUCCESS))
		WMA_LOGE("Failed to set the event");
}
/* function   : wma_htc_cfg_nv_connect_service
 * Descriptin :
 * Args       :
 * Retruns    :
 */
VOS_STATUS wma_htc_cfg_nv_connect_service(tp_wma_handle wma_handle)
{
	HTC_SERVICE_CONNECT_REQ connect_req;
	HTC_SERVICE_CONNECT_RESP connect_resp;
	A_STATUS status = A_OK;
	VOS_STATUS vos_status = VOS_STATUS_SUCCESS;

	WMA_LOGD("Enter");

	/* meta data is unused for now */
	connect_req.pMetaData = NULL;
	connect_req.MetaDataLength = 0;

	/* these fields are the same for all service endpoints */
	connect_req.EpCallbacks.pContext = wma_handle;
	connect_req.EpCallbacks.EpTxCompleteMultiple = NULL;
	connect_req.EpCallbacks.EpRecv = wma_cfg_nv_rx;
	connect_req.EpCallbacks.EpRecvRefill = NULL;
	connect_req.EpCallbacks.EpSendFull = NULL;
	connect_req.EpCallbacks.EpTxComplete = wma_cfg_nv_download_complete;

	/* connect to control service */
	connect_req.ServiceID = CFG_NV_SVC;

	status = HTCConnectService(wma_handle->htc_handle, &connect_req, &connect_resp);

	if (A_OK != status) {
		WMA_LOGP("Failed to connect htc endpoint for cfg nv");
		vos_status = VOS_STATUS_SUCCESS;
		goto end;
	}

	if (HTC_SERVICE_SUCCESS == connect_resp.ConnectRespCode) {
		WMA_LOGA("WMA --> HTCConnectService(CFG_NV_SVC) - success");
		wma_handle->cfg_nv.endpoint_id = connect_resp.Endpoint;
		vos_status = VOS_STATUS_SUCCESS;
	}
	else {
		WMA_LOGP("HTCConnectService failed for cfg nv");
		vos_status = VOS_STATUS_E_FAILURE;
	}
end:
	WMA_LOGD("Exit");
	return vos_status;
}

VOS_STATUS wma_hal_stop_isoc(tp_wma_handle wma_handle)
{
	A_STATUS      status = A_ERROR;
	VOS_STATUS vos_status = VOS_STATUS_SUCCESS;
	tHalMacStopReqMsg      halStopReq;
	v_U8_t     *pSendBuffer = NULL;
	v_U16_t    usDataOffset       = 0;
	v_U16_t    usSendSize         = 0;
	v_U16_t    usLen              = 0;
	adf_nbuf_t  msg = NULL;
	v_U32_t   *msg_word = NULL;
	HTC_PACKET  HtcPkt;

	/*Get message buffer*/
	usLen = sizeof(halStopReq.stopReqParams);
	if ((VOS_STATUS_SUCCESS !=
			wma_cfg_nv_get_hal_message_buffer(wma_handle,
						FW_WLAN_HAL_STOP_REQ,
						usLen,
						&pSendBuffer,
						&usDataOffset,
						&usSendSize)) ||
			(usSendSize < (usDataOffset + usLen))) {
		vos_status = VOS_STATUS_E_FAILURE;
		VOS_ASSERT(0);
		goto end;
	}

	/* Fill in the message*/
	halStopReq.stopReqParams.reason = HAL_STOP_TYPE_RF_KILL;
	vos_mem_copy(pSendBuffer+usDataOffset,
			&halStopReq.stopReqParams,
			sizeof(halStopReq.stopReqParams));

	/* Fill HTC packet*/
	msg = adf_nbuf_alloc(
			wma_handle->adf_dev,
			usSendSize,
			/* reserve room for HTC header */
			HTC_HEADER_LEN + HTC_HDR_ALIGNMENT_PADDING, 4, FALSE);
	if (!msg) {
		vos_status = VOS_STATUS_E_NOMEM;
		goto end;
	}
	/* set the length of the message */
	adf_nbuf_put_tail(msg, usSendSize);
	/* fill in the message contents */
	msg_word = (v_U32_t *) adf_nbuf_data(msg);
	adf_os_mem_copy(msg_word, pSendBuffer, usSendSize);

	SET_HTC_PACKET_INFO_TX(
			&HtcPkt,
			wma_handle,
			adf_nbuf_data(msg),
			adf_nbuf_len(msg),
			wma_handle->cfg_nv.endpoint_id,
			0);

	SET_HTC_PACKET_NET_BUF_CONTEXT(&HtcPkt, msg);
	status = HTCSendPkt(wma_handle->htc_handle, &HtcPkt);
	if (status != A_OK) {
		WMA_LOGE("download hal_stop failed!\n");
		vos_status = VOS_STATUS_E_FAILURE;
		goto end;
	}

	vos_event_reset(&wma_handle->cfg_nv_rx_complete);
	/* wait for cfg response*/
	WMA_LOGA("check HAL_STOP response start...\n");
	vos_status = vos_wait_single_event(&(wma_handle->cfg_nv_rx_complete),
					WMA_CFG_NV_DNLD_TIMEOUT);
	if (VOS_STATUS_SUCCESS != vos_status)
		WMA_LOGP("failed to stop hal\n");

end:
	if (pSendBuffer) {
		vos_mem_free(pSendBuffer);
		pSendBuffer = NULL;
	}

	return vos_status;
}
