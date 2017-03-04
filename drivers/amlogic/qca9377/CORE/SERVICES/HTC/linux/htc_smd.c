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

  \file     htc_smd.c
  \brief    htc smd module implmentation

  ========================================================================*/
/**=========================================================================
  EDIT HISTORY FOR FILE


  This section contains comments describing changes made to the module.
  Notice that changes are listed in reverse chronological order.

  $Header:$   $DateTime: $ $Author: $


  when              who           what, where, why
  --------          ---           -----------------------------------------
  01/03/2013        Ganesh        HTC SMD module implementation
                    Kondabattini
  ==========================================================================*/


#include "htc_api.h"
#include "htc_smd.h"
#include "htc_services.h"
#include "vos_memory.h"
#include "vos_mq.h"
#include "wmi_unified.h"

#define HTC_LOGD(args...) \
	VOS_TRACE( VOS_MODULE_ID_HTC, VOS_TRACE_LEVEL_INFO, ## args)
#define HTC_LOGE(args...) \
	VOS_TRACE( VOS_MODULE_ID_HTC, VOS_TRACE_LEVEL_ERROR, ## args)
#define HTC_LOGP(args...) \
	VOS_TRACE( VOS_MODULE_ID_HTC, VOS_TRACE_LEVEL_FATAL, ## args)

/*******************************************
 ************* Declarations ****************
 *******************************************/
static tp_htc_handle g_htc_handle;

/*******************************************
 ********** Function Definition*************
 *******************************************/

/* function:     htc_smd_open_callback
 * Descriptin:
 * Args:         msg
 * Retruns:      nothing
 */
static void htc_smd_open_callback(t_htc_msg *msg)
{
	tp_htc_handle htc_handle;
	HTC_LOGD("Enter");
	if (NULL == msg) {
		WARN_ON(1);
		HTC_LOGE("Invalid parameters received.");
		return;
	}
	/* extract our context from the message */
	htc_handle = (tp_htc_handle)msg->pContext;

	/*--------------------------------------------------------------------
	  Sanity check
	  --------------------------------------------------------------------*/
	if ((NULL == htc_handle) || (HTC_CB_MAGIC != htc_handle->htc_magic)) {
		HTC_LOGE("Invalid parameters received.");
		return;
	}

	if (HTC_SMD_STATE_OPEN_PENDING != htc_handle->htc_smd_state) {
		HTC_LOGE("Invoked from invalid state.");
		return;
	}

	/* notified registered client that the channel is open */
	htc_handle->htc_smd_state = HTC_SMD_STATE_OPEN;

	/* signal event for WCTS_OpenTransport to proceed */
	complete(&htc_handle->htc_smd_event);
}


/* function:   htc_get_end_point
 * Descriptin: extracts pointer to endpoint from wmi cmd id
 * Args:       htc_handle, wmi cmd id
 * Retruns:    point to endpoint
 */
HTC_ENDPOINT* htc_get_end_point(t_htc_handle *htc_handle, HTC_SERVICE_ID id)
{
	/* currently we don't get HTT message from SMD driver.
	   HTT message will be routed by DXE */
	if (id >= WMI_EVT_GRP_START_ID(WMI_GRP_START)) {
		return &htc_handle->end_point[HTC_ENDPOINT_WMI];
	} else {
		return &htc_handle->end_point[HTC_ENDPOINT_INIT];
	}
}

/* function:   htc_smd_call_endpoint
 * Descriptin:
 * Args:       htc_handle, rx_buf
 * Retruns:    nothing
 */
void htc_smd_call_endpoint(tp_htc_handle htc_handle, adf_nbuf_t rx_buf)
{
	unsigned short cmd_id;
	HTC_ENDPOINT *endpoint;
	unsigned char *data = adf_nbuf_data(rx_buf);
	unsigned int long len = adf_nbuf_len(rx_buf);

	HTC_LOGD("Enter");
	cmd_id = data[1] << 8 | data[0];
	endpoint = htc_get_end_point(htc_handle, cmd_id);

	HTC_LOGD("WMI command id = %hu, endpoint = %p,"
			"endpoint->Id = %d", cmd_id,
			endpoint, endpoint->Id);

	if (endpoint && endpoint->EpCallBacks.EpRecv) {
		vos_mem_zero(&htc_handle->htc_packet, sizeof (HTC_PACKET));
		htc_handle->htc_packet.Status = 0;
		htc_handle->htc_packet.pPktContext = rx_buf;
		htc_handle->htc_packet.pBuffer = data;
		htc_handle->htc_packet.ActualLength = len;
		htc_handle->htc_packet.Endpoint = endpoint->Id;
		HTC_LOGD("calling endpoint id %d", endpoint->Id);
		endpoint->EpCallBacks.EpRecv(endpoint->EpCallBacks.pContext,
				&htc_handle->htc_packet);
		/*TODO: freee rx_buf ??*/
	} else {
		HTC_LOGE("received unknown message from FW with id = 0x%x\n", cmd_id);
		adf_nbuf_free(rx_buf);
	}

	HTC_LOGD("Exit");
}

/* function:    htc_smd_read_callback
 * Descriptin:
 * Args:        htc_handle
 * Retruns:     nothing
 */
static void htc_smd_read_callback(tp_htc_handle htc_handle)
{
	int packet_size;
	int available;
	int bytes_read;
	adf_nbuf_t rx_buf;
	void *data;

	HTC_LOGD("Enter");

	if ((NULL == htc_handle) || (HTC_CB_MAGIC != htc_handle->htc_magic)) {
		HTC_LOGE("Invalid parameters.");
		return;
	}

	/* iterate until no more packets are available */
	while (1) {
		/* check the length of the next packet */
		packet_size = smd_cur_packet_size(htc_handle->smd_channel);
		if (0 == packet_size) {
			/* No more data to be read */
			return;
		}

		/* Check how much of the data is available */
		available = smd_read_avail(htc_handle->smd_channel);
		if (available < packet_size) {
			return;
		}

		rx_buf = adf_nbuf_alloc(htc_handle->osdev, packet_size, 0, 1, 0);
		data = adf_nbuf_data(rx_buf);
		bytes_read = smd_read(htc_handle->smd_channel,
				data,
				packet_size);

		if (bytes_read != packet_size) {
			HTC_LOGE("Failed to read the data from the SMD.");
			adf_nbuf_free(rx_buf);
			WARN_ON(1);
			return;
		}

		adf_nbuf_set_pktlen(rx_buf, (uint32_t)packet_size);

		HTC_LOGD("packet_size  = %d", packet_size);

		htc_smd_call_endpoint(htc_handle, rx_buf);
	}

	HTC_LOGD("Exit");
}

/* function:    htc_smd_write_callback
 * Descriptin:
 * Args:        htc_handle
 * Retruns:     nothing
 */
static void htc_smd_write_callback(tp_htc_handle htc_handle)
{
	vos_list_node_t *pnode;
	t_htc_buffer *bufq;
	HTC_PACKET *htc_packet;
	void *buf;
	int len;
	int available;
	int written;
	HTC_ENDPOINT *endpoint;
	adf_nbuf_t tx_buf;

	HTC_LOGD("Enter");

	if ((NULL == htc_handle) || (HTC_CB_MAGIC != htc_handle->htc_magic)) {
		HTC_LOGE("Invalid parameter");
		return;
	}

	/* if we are not deferred, then there are no pending packets */
	if (HTC_SMD_STATE_DEFERRED != htc_handle->htc_smd_state) {
		HTC_LOGD("No Pending packets");
		return;
	}

	/* Keep sending deferred messages as long as there is room in
	   the channel.  Note that we initially peek at the head of the
	   list to access the parameters for the next message; we don't
	   actually remove the next message from the deferred list until
	   we know the channel can handle it */
	while (VOS_STATUS_SUCCESS ==
			vos_list_peek_front(&htc_handle->htc_write_pending_q, &pnode)) {
		bufq = container_of(pnode, t_htc_buffer, node);
		htc_packet = bufq->htc_packet;
		tx_buf = GET_HTC_PACKET_NET_BUF_CONTEXT(htc_packet);
		buf = adf_nbuf_data(tx_buf);
		len = adf_nbuf_len(tx_buf);
		endpoint = &htc_handle->end_point[htc_packet->Endpoint];
		available = smd_write_avail(htc_handle->smd_channel);
		if (available < len) {
			/* channel has no room for the next packet so we are done */
			HTC_LOGD("channel has no room for next packet");
			return;
		}

		/* there is room for the next message, so we can now remove
		   it from the deferred message queue and send it */
		vos_list_remove_front(&htc_handle->htc_write_pending_q, &pnode);

		/* note that pNode will be the same as when we peeked, so
		   there is no need to update pBuffer or len */

		written = smd_write(htc_handle->smd_channel, buf, len);
		if (written != len) {
			/* Something went wrong */
			HTC_LOGE("Channel write failure");

			/* we were unable to send the message that was at the head
			   of the deferred list.  there is nothing else we can do
			   other than drop it, so we will just fall through to the
			   "success" processing.
			   hopefully the client can recover from this since there is
			   nothing else we can do here */
		}

		/* we'll continue to iterate until the channel is full or all
		   of the deferred messages have been sent */
		htc_packet->Status = 0;
		endpoint->EpCallBacks.EpTxComplete(endpoint->EpCallBacks.pContext, htc_packet);
	}

	/* if we've exited the loop, then we have drained the deferred queue.
	   set the state to indicate we are no longer deferred, and turn off
	   the remote read interrupt */
	htc_handle->htc_smd_state  = HTC_SMD_STATE_OPEN;
	smd_disable_read_intr(htc_handle->smd_channel);

	HTC_LOGD("Exit");
}

/* function:    htc_smd_data_callback
 * Descriptin:
 * Args:        pMsg
 * Retruns:     nothing
 */
static void htc_smd_data_callback(t_htc_msg *pMsg)
{
	/* extract our context from the message */
	tp_htc_handle htc_handle = pMsg->pContext;

	HTC_LOGD("Enter");

	/* process any incoming messages */
	htc_smd_read_callback(htc_handle);

	/* send any deferred messages */
	htc_smd_write_callback(htc_handle);

	HTC_LOGD("Exit");
}

/* function:     htc_post_ctrl_msg
 * Descriptin:
 * Args:         htc_handle, pMsg
 * Retruns:      failure or success
 */
int htc_post_ctrl_msg(void *htc_handle, t_htc_msg *pMsg)
{
	vos_msg_t msg;

	HTC_LOGD("Enter");

	if (NULL == pMsg)
	{
		HTC_LOGP("NULL message pointer");
		WARN_ON(1);
		return -EINVAL;
	}

	msg.type = 0;  //This field is not used because VOSS doesn't check it.
	msg.reserved = 0;
	msg.bodyval = 0;
	msg.bodyptr = pMsg;
	if(!VOS_IS_STATUS_SUCCESS(vos_mq_post_message(VOS_MQ_ID_HTC, &msg)))
	{
		HTC_LOGE("fail to post msg %d", pMsg->type);
		return -EINVAL;
	}
	else
	{
		HTC_LOGD("Successfully posted msg to MC thread");
	}
	HTC_LOGD("Exit");
	return 0;
}

/* function:    htc_smd_notify_callback
 * Descriptin:
 * Args:        data, event
 * Retruns:     nothing
 */
static void htc_smd_notify_callback(void *data, unsigned event)
{
	t_htc_msg *htc_msg;
	tp_htc_handle htc_handle = (tp_htc_handle) data;

	HTC_LOGD("Enter");

	if (NULL == htc_handle)
	{
		HTC_LOGE("Invalid htc_handle");
		WARN_ON(1);
		return;
	}

	if (HTC_CB_MAGIC != htc_handle->htc_magic) {
		HTC_LOGE("Received unexpected SMD event");
		return;
	}

	/* Serialize processing in the control thread */
	switch (event) {
		case SMD_EVENT_OPEN:
			HTC_LOGD("received SMD_EVENT_OPEN from SMD");
			/* If the prev state was 'remote closed' then it is a Riva 'restart',
			 * subsystem restart re-init
			 */
			if (HTC_SMD_STATE_REM_CLOSED == htc_handle->htc_smd_state)
			{
				HTC_LOGD("received SMD_EVENT_OPEN in HTC_SMD_STATE_REM_CLOSED state");
				/* call subsystem restart re-init function */
				vos_wlanReInit();
				return;
			}
			htc_msg = &htc_handle->htc_open_msg;
			break;

		case SMD_EVENT_DATA:
			if (HTC_SMD_STATE_REM_CLOSED == htc_handle->htc_smd_state)
			{
				HTC_LOGD("received smd data in HTC_SMD_STATE_REM_CLOSED state");
				/* we should not be getting any data now */
				return;
			}
			HTC_LOGD("received SMD_EVENT_DATA from SMD");
			htc_msg = &htc_handle->htc_data_msg;
			break;

		case SMD_EVENT_CLOSE:
			HTC_LOGD("received SMD_EVENT_CLOSE from SMD");
			/* SMD channel was closed from the remote side,
			 * this would happen only when Riva crashed and SMD is
			 * closing the channel on behalf of Riva */
			htc_handle->htc_smd_state = HTC_SMD_STATE_REM_CLOSED;
			/* subsystem restart: shutdown */
			vos_wlanShutdown();
			return;

		case SMD_EVENT_STATUS:
			HTC_LOGD("received SMD_EVENT_STATUS from SMD");
			return;

		case SMD_EVENT_REOPEN_READY:
			HTC_LOGD("received SMD_EVENT_REOPEN_READY from SMD");

			/* TODO:unlike other events which occur when our kernel threads are
			   running, this one is received when the threads are closed and
			   the rmmod thread is waiting.  so just unblock that thread */
			complete(&htc_handle->htc_smd_event);
			return;

		default:
			HTC_LOGD("Unexpected event %u received from SMD", event);
			return;
	}

	htc_post_ctrl_msg(htc_handle, htc_msg);
	HTC_LOGD("Exit");
}

/* function: htc_create
 * Descriptin:
 * Args: name -> HTC_SMD_CTRL_PORT
 osdev
 * Retruns: tp_htc_handle
 */
HTC_HANDLE HTCCreate(void *hHIF, HTC_INIT_INFO *pInfo, adf_os_device_t osdev)
{
	int status = 0;
	tp_htc_handle htc_handle = NULL;

	HTC_LOGD("Enter");

	/* This open is coming after a SSR, we don't need to reopen SMD,
	 * the SMD port was never closed during SSR*/
	if (g_htc_handle) {
		HTC_LOGD("\n SMD port is already opened");

		htc_handle  = g_htc_handle;
		if (HTC_CB_MAGIC != htc_handle->htc_magic) {
			HTC_LOGP("\n Invalid magic");
			return NULL;
		}
		htc_handle->htc_smd_state = HTC_SMD_STATE_OPEN;

		/*TODO: if caller of this function registers notify callback
		  then call the function here
		 */
#if 0
		pWCTSCb->wctsNotifyCB((WCTS_HandleType)pWCTSCb,
				WCTS_EVENT_OPEN,
				pWCTSCb->wctsNotifyCBData);
#endif
		/* we initially don't want read interrupts
		   (we only want them if we get into deferred write mode) */
		smd_disable_read_intr(htc_handle->smd_channel);

		return htc_handle;
	}

	/* allocate the memory for HTC handle */
	htc_handle = vos_mem_malloc(sizeof (t_htc_handle));
	if (NULL == htc_handle) {
		HTC_LOGE("Memory allocation failed for htc_handle");
		return NULL;
	}

	vos_mem_zero(htc_handle, sizeof(t_htc_handle));
        init_completion(&htc_handle->htc_smd_event);


	vos_list_init(&htc_handle->htc_write_pending_q);

	/* initialize the remaining fields */
	htc_handle->osdev = osdev;
	htc_handle->htc_magic = HTC_CB_MAGIC;
	htc_handle->htc_smd_state = HTC_SMD_STATE_OPEN_PENDING;
	htc_handle->smd_channel = NULL;

	/* since SMD will callback in interrupt context, we will used
	 * canned messages to serialize the SMD events into a thread
	 * context
	 */
	htc_handle->htc_open_msg.callback = htc_smd_open_callback;
	htc_handle->htc_open_msg.pContext = htc_handle;
	htc_handle->htc_data_msg.callback = htc_smd_data_callback;
	htc_handle->htc_data_msg.pContext = htc_handle;

	HTC_LOGD("htc_handle = %p", htc_handle);

	INIT_COMPLETION(htc_handle->htc_smd_event);
	/* Open the SMD channel */
	status = smd_named_open_on_edge(HTC_SMD_CTRL_PORT, SMD_APPS_WCNSS,
			&htc_handle->smd_channel, htc_handle,
			htc_smd_notify_callback);
	if (0 != status) {
		HTC_LOGE("Failed to open SMD Channel");
		goto fail;
	}
	/* Wait for the event */
	status = wait_for_completion_interruptible_timeout(
			&htc_handle->htc_smd_event,
			msecs_to_jiffies(HTC_SMD_OPEN_TIMEOUT));

	if (!status) {
		HTC_LOGE("Timeout occurred while waiting for SMD_SS_OPENED event.");
		/* since we opened one end of the channel, close it */
		status = smd_close(htc_handle->smd_channel);
		if (0 != status) {
			HTC_LOGE("smd_close failed.");
		}
		goto fail;
	}

	/* we initially don't want read interrupts
	   (we only want them if we get into deferred write mode) */
	smd_disable_read_intr(htc_handle->smd_channel);

	/* we have successfully opened the SMD channel */
	g_htc_handle = htc_handle;
	HTC_LOGD("Exit");
	return (void*)htc_handle;

fail:
	/* Failed to open the SMD Channel */
	HTC_LOGE("Failure");
	vos_mem_free(htc_handle);
	return NULL;
}

/* function:    htc_connect_service
 * Descriptin:
 * Args:       htc_handle, pConnectReq, pConnectResp
 * Retruns:    success or failure
 */
A_STATUS HTCConnectService(HTC_HANDLE               HTCHandle,
                           HTC_SERVICE_CONNECT_REQ  *pConnectReq,
                           HTC_SERVICE_CONNECT_RESP *pConnectResp)
{
	tp_htc_handle htc_handle = (tp_htc_handle)HTCHandle;
	HTC_ENDPOINT *endpoint = NULL;

	HTC_LOGD("Enter");

	WARN_ON(!pConnectReq->ServiceID);

	switch (pConnectReq->ServiceID) {
		case CFG_NV_SVC:
			HTC_LOGD("CFG_NV_SVC");
			endpoint = &htc_handle->end_point[HTC_ENDPOINT_INIT];
			pConnectResp->Endpoint = HTC_ENDPOINT_INIT;
			break;
		case WMI_CONTROL_SVC:
			HTC_LOGD("WMI_CONTROL_SVC");
			endpoint = &htc_handle->end_point[HTC_ENDPOINT_WMI];
			pConnectResp->Endpoint = HTC_ENDPOINT_WMI;
			break;
		default:
			HTC_LOGD("Invalid service");
			return -EINVAL;
	}

	pConnectResp->ConnectRespCode = HTC_SERVICE_SUCCESS;
	endpoint->ServiceID = pConnectReq->ServiceID;
	endpoint->EpCallBacks = pConnectReq->EpCallbacks;
	endpoint->Id = pConnectResp->Endpoint;
    /*TODO: */
	//endpoint->target = htc_handle;

	HTC_LOGD("Exit");
	return 0;
}

/* function:     htc_start
 * Descriptin:
 * Args:         htc_handle
 * Retruns:      success or failure
 */
A_STATUS HTCStart(HTC_HANDLE HTCHandle)
{
	tp_htc_handle htc_handle = (tp_htc_handle)HTCHandle;
	if (HTC_SMD_STATE_OPEN == htc_handle->htc_smd_state) {
		HTC_LOGD("htc_start success");
		/* TODO: ? */
		return A_OK;
	}
	else {
		HTC_LOGD("smd is not in open state; htc_start failed");
		return A_ERROR;
	}
}

/* function:    htc_stop
 * Descriptin:
 * Args:        htc_handle
 * Retruns:     nothing
 */
void HTCStop(HTC_HANDLE HTCHandle)
{
	/* TODO: ? */
	return;
}

/* function:     htc_destroy
 * Descriptin:   Closes the SMD session.
 * Args:         HTC context
 * Retruns:      success or failure
 */
void  HTCDestroy(HTC_HANDLE HTCHandle)
{
	tp_htc_handle htc_handle = (tp_htc_handle) HTCHandle;
	vos_list_node_t     *pnode = NULL;
	t_htc_buffer        *bufq = NULL;
	HTC_PACKET          *htc_packet;
	int                 smd_status;
	int                 status;
	HTC_ENDPOINT      *endpoint;

	if ((NULL == htc_handle) || (HTC_CB_MAGIC != htc_handle->htc_magic)) {
		HTC_LOGE("Invalid parameter");
		return;
	}

	/*Free the buffers in the pending queue.*/
	while (VOS_STATUS_SUCCESS ==
			vos_list_remove_front(&htc_handle->htc_write_pending_q, &pnode)) {
		bufq = container_of(pnode, t_htc_buffer, node);
		htc_packet = (HTC_PACKET*)bufq->htc_packet;

		endpoint = &htc_handle->end_point[htc_packet->Endpoint];
		endpoint->EpCallBacks.EpRecv(endpoint->EpCallBacks.pContext, htc_packet);
		vos_mem_free(bufq);
	}


	INIT_COMPLETION(htc_handle->htc_smd_event);
	smd_status = smd_close(htc_handle->smd_channel);
	if (0 != smd_status) {
		HTC_LOGE("smd_close failed with status %d", smd_status);
		/* SMD did not successfully close the channel, therefore we
		   won't receive an asynchronous close notification so don't
		   bother to wait for an event that won't come */

	} else {
		/* close command was sent -- wait for the callback to complete */
		status = wait_for_completion_interruptible_timeout(
				&htc_handle->htc_smd_event,
				msecs_to_jiffies(HTC_SMD_OPEN_TIMEOUT));
		if (!status) {
			HTC_LOGE("failed to receive SMD_EVENT_REOPEN_READY %d", smd_status);
		}

		/* During the close sequence we deregistered from SMD.  As part
		   of deregistration SMD will call back into our driver with an
		   event to let us know the channel is closed.  We need to
		   insert a brief delay to allow that thread of execution to
		   exit our module.  Otherwise our module may be unloaded while
		   there is still code running within the address space, and
		   that code will crash when the memory is unmapped  */
		msleep(50);
	}

	/* Reset the state */
	htc_handle->htc_smd_state = HTC_SMD_STATE_CLOSED;

	/* TODO: If caller registers any function then call it */
#if 0
	/* channel has (hopefully) been closed */
	htc_handle->wctsNotifyCB((WCTS_HandleType)pWCTSCb,
			WCTS_EVENT_CLOSE,
			pWCTSCb->wctsNotifyCBData);
#endif
	/* release the resource */
	htc_handle->htc_magic = 0;
	vos_mem_free(htc_handle);

	g_htc_handle = NULL;

	return;
}

/* function:    htc_send_packet
 * Descriptin:
 * Args:        handle, htc_pakcet
 * Retruns:     success or failure
 */
A_STATUS HTCSendPkt(HTC_HANDLE HTCHandle, HTC_PACKET *pPacket)
{
	tp_htc_handle htc_handle = (tp_htc_handle)HTCHandle;
	HTC_ENDPOINT *endpoint = &htc_handle->end_point[pPacket->Endpoint];
	adf_nbuf_t txBuf = (adf_nbuf_t)GET_HTC_PACKET_NET_BUF_CONTEXT(pPacket);
	t_htc_buffer *bufq;
	int len;
	int written = 0;
	int available;

	if ((NULL == htc_handle) || (HTC_CB_MAGIC != htc_handle->htc_magic) ||
			(NULL == pPacket)) {
		HTC_LOGE("Invalid parameters received");
		WARN_ON(1);
		return A_ERROR;
	}

	/* the SMD API uses int instead of uint, so change types here */
	len = adf_nbuf_len(txBuf);

	if (HTC_SMD_STATE_OPEN == htc_handle->htc_smd_state) {
		available = smd_write_avail(htc_handle->smd_channel);
		if (available >= len) {
			written = smd_write(htc_handle->smd_channel, adf_nbuf_data(txBuf), len);
		}
	} else if (HTC_SMD_STATE_DEFERRED == htc_handle->htc_smd_state) {
		HTC_LOGD("FIFO space not available, the packets will be queued");
	} else {
		HTC_LOGE("Channel in invalid state %d", htc_handle->htc_smd_state);
		/* force following logic to reclaim the buffer */
		written = -1;
	}

	if (-1 == written) {
		/*Something wrong*/
		HTC_LOGD("Failed to send message over the bus");
		WARN_ON(1);
		return A_ERROR;
	} else if (written == len) {
		HTC_LOGD("Message sent");
		/* Message sent! No deferred state, free the buffer*/
		if ((NULL != endpoint) && (NULL != endpoint->EpCallBacks.EpTxComplete)) {
			HTC_LOGD("calling EpTxComplete");
			pPacket->Status = A_OK; //SUCCESS
			endpoint->EpCallBacks.EpTxComplete(endpoint->EpCallBacks.pContext,
					pPacket);
		}
	} else {
		/* This much data cannot be written at this time,
		   queue the rest of the data for later*/
		HTC_LOGD("Defer message");
		bufq = vos_mem_malloc(sizeof(t_htc_buffer));
		if (NULL == bufq) {
			HTC_LOGE("Cannot allocate memory for queuing the buffer");
			WARN_ON(1);
			return A_MEMORY_NOT_AVAIL;
		}
		bufq->htc_packet = pPacket;
		vos_list_insert_back(&htc_handle->htc_write_pending_q, &bufq->node);

		/* if we are not already in the deferred state, then transition
		   to that state.  when we do so, we enable the remote read
		   interrupt so that we'll be notified when messages are read
		   from the remote end */
		if (HTC_SMD_STATE_DEFERRED != htc_handle->htc_smd_state) {

			/* Mark the state as deferred.
			 */
			htc_handle->htc_smd_state = HTC_SMD_STATE_DEFERRED;

			smd_enable_read_intr(htc_handle->smd_channel);
		}
	}

	return A_OK;
}
