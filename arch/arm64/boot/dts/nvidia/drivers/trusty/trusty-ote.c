/*
 * Copyright (c) 2016-2018 NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */
#include <linux/completion.h>
#include <linux/err.h>
#include <linux/slab.h>

#include <linux/trusty/trusty.h>
#include <linux/trusty/trusty_ipc.h>

#include "trusty-ote.h"

/* Time out in milli seconds */
#define REPLY_TIMEOUT	5000
#define TXBUF_TIMEOUT	15000

//#define DEBUG

#ifdef DEBUG
#define trusty_ote_debug(...)	\
		pr_err("trusty-ote: " __VA_ARGS__)
#else
#define trusty_ote_debug(...)		\
({						\
	if (0)					\
		pr_info("trusty-ote: " __VA_ARGS__); \
	0;					\
})
#endif

/*
 * Currently assumed that only one buffer is sent using these APIs
 * per operation apart from header
 */
#define MAX_TIPC_MSG_NUM (1 + 1)

struct tipc_chan_ctx {
	struct tipc_chan *chan;
	struct completion reply_comp;
	int state;
	/*
	 * Stores the data that is sent/received over channel
	 */
	void *data[MAX_TIPC_MSG_NUM];
	size_t len[MAX_TIPC_MSG_NUM];
	int cur_msg;
	int total_msg;
};

enum tipc_chan_state {
	TIPC_DISCONNECTED = 0,
	TIPC_CONNECTED,
	TIPC_NOT_FOUND
};

static int wait_for_response(struct  tipc_chan_ctx *chan_ctx, int timeout)
{
	int ret;

	ret = wait_for_completion_interruptible_timeout(&chan_ctx->reply_comp,
					msecs_to_jiffies(timeout));
	if (ret > 0)
		/* Received reply */
		ret = 0;
	else if (ret == 0)
		/* No reply from remote */
		ret = -ETIMEDOUT;

	return ret;
}

static struct tipc_msg_buf *_handle_msg(void *data, struct tipc_msg_buf *rxbuf)
{
	struct tipc_chan_ctx *chan_ctx = (struct tipc_chan_ctx *)data;

	trusty_ote_debug("%s\n", __func__);
	if (mb_avail_data(rxbuf) != chan_ctx->len[chan_ctx->cur_msg]) {
		pr_err("%s:ERROR: expected msg(%d) len %zu: actual %zu",
				__func__, chan_ctx->cur_msg, chan_ctx->len[chan_ctx->cur_msg],
				mb_avail_data(rxbuf));
		return rxbuf;
	}

	/* copy data received over channel */
	memcpy(chan_ctx->data[chan_ctx->cur_msg],
			mb_get_data(rxbuf, chan_ctx->len[chan_ctx->cur_msg]),
			chan_ctx->len[chan_ctx->cur_msg]);

	chan_ctx->cur_msg++;
	/* wake up client if all messages are received */
	if (chan_ctx->cur_msg == chan_ctx->total_msg)
		complete(&chan_ctx->reply_comp);

	return rxbuf;
}

void handle_connect_event(struct tipc_chan_ctx *chan_ctx)
{
	chan_ctx->state = TIPC_CONNECTED;
	complete(&chan_ctx->reply_comp);
}

static void _handle_event(void *data, int event)
{
	struct tipc_chan_ctx *chan_ctx = data;

	trusty_ote_debug("%s: event %d state %d\n", __func__,
			event, chan_ctx->state);
	switch (event) {
	case TIPC_CHANNEL_SHUTDOWN:
		pr_err("%s: channel(state:%d) shutting down\n",
				__func__, chan_ctx->state);
		chan_ctx->state = TIPC_DISCONNECTED;
		/* wake up the pending client */
		complete(&chan_ctx->reply_comp);
		break;

	case TIPC_CHANNEL_DISCONNECTED:
		pr_err("%s: channel(state:%d) disconnected\n",
				__func__, chan_ctx->state);
		chan_ctx->state = TIPC_DISCONNECTED;
		/* wake up the pending client */
		complete(&chan_ctx->reply_comp);
		break;

	case TIPC_CHANNEL_NOT_FOUND:
		chan_ctx->state = TIPC_NOT_FOUND;
		/* wake up the pending client */
		complete(&chan_ctx->reply_comp);
		break;

	case TIPC_CHANNEL_CONNECTED:
		handle_connect_event(chan_ctx);
		break;

	default:
		pr_err("%s: unhandled event %d\n", __func__, event);
		break;
	}
	return;
}

static void _handle_release(void *data)
{
	trusty_ote_debug("%s\n", __func__);
	kfree(data);
}

struct tipc_chan_ops chan_ops = {
	.handle_msg = _handle_msg,
	.handle_event = _handle_event,
	.handle_release = _handle_release,
};

/*
 * Embeds the data in TIPC buffer and queue the message
 * to send to secure world.
 */
static int queue_msg(struct tipc_chan_ctx *chan_ctx, void *data, size_t len)
{
	int ret;
	struct tipc_msg_buf *txbuf = NULL;

	trusty_ote_debug("%s: data %p len %zu\n", __func__, data, len);

	txbuf = tipc_chan_get_txbuf_timeout(chan_ctx->chan, TXBUF_TIMEOUT);
	if (IS_ERR(txbuf)) {
		pr_err("%s:error(%ld) in get txbuf\n", __func__, PTR_ERR(txbuf));
		return PTR_ERR(txbuf);
	}

	/* check available space */
	if (len > mb_avail_space(txbuf)) {
		ret = -EMSGSIZE;
		goto err;
	}

	/* copy in message data */
	memcpy(mb_put_data(txbuf, len), data, len);

	/* queue message */
	ret = tipc_chan_queue_msg(chan_ctx->chan, txbuf);
	if (ret)
		goto err;

	return ret;
err:
	tipc_chan_put_txbuf(chan_ctx->chan, txbuf);
	return ret;
}

static int construct_default_stream_header(stream_header_t *stream_header) {

	if (NULL == stream_header) {
		return -EINVAL;
	}

	memset(stream_header, 0, STREAM_META_HEADER_LEN);

	stream_header->magic = STREAM_HEADER_MAGIC;
	stream_header->version = STREAM_HEADER_CUR_VERSION;

	return 0;
}

static int construct_default_payload_header(payload_header_t *payload_header) {

	if (NULL == payload_header) {
		return -EINVAL;
	}

	memset(payload_header, 0, PAYLOAD_META_HEADER_LEN);

	payload_header->magic = PAYLOAD_HEADER_MAGIC;

	return 0;
}

/*
 * Constructs OTE message and sent it to TA over TIPC channel.
 * And then wait till response is received from TA.
 */
static int handle_ote_msg(struct tipc_chan_ctx *chan_ctx, void *buf,
		size_t len, uint32_t cmd, te_error_t *op_status)
{
	int ret = 0;
	stream_header_t stream_header;
	payload_header_t payload_header;
	uint8_t *payload_buffer = NULL;

	if (NULL == chan_ctx) {
		return -EINVAL;
	}

	if (len > TIPC_MAX_CHUNK_SIZE) {
		pr_err("%s: passing buffers of size > %u not supported. len(%zu)"
			, __func__, TIPC_MAX_CHUNK_SIZE, len);
		return -EINVAL;
	}

	ret = construct_default_stream_header(&stream_header);
	if (ret < 0) {
		return ret;
	}

	stream_header.command = cmd;

	trusty_ote_debug("%s: buf %p len %zu\n", __func__, buf, len);

	if (NULL != buf) {
		stream_header.num_entries = 1;
		stream_header.total_length = PAYLOAD_META_HEADER_LEN + len;

		ret = construct_default_payload_header(&payload_header);
		if (ret < 0) {
			return ret;
		}

		payload_header.type = TE_PARAM_TYPE_MEM_RW;
		payload_header.length = len;

		payload_buffer = kzalloc((len + PAYLOAD_META_HEADER_LEN), GFP_KERNEL);
		if (NULL == payload_buffer) {
			ret = -ENOMEM;
			return ret;
		}

		memcpy(payload_buffer, &payload_header, PAYLOAD_META_HEADER_LEN);
		memcpy(&payload_buffer[PAYLOAD_META_HEADER_LEN], buf, len);
	}

	chan_ctx->cur_msg = 0;
	chan_ctx->total_msg = 1;
	chan_ctx->data[0] = &stream_header;
	chan_ctx->len[0] = STREAM_META_HEADER_LEN;
	if (NULL != buf) {
		chan_ctx->data[1] = payload_buffer;
		chan_ctx->len[1] = len + PAYLOAD_META_HEADER_LEN;
		chan_ctx->total_msg++;
	}

	/* queue stream header */
	trusty_ote_debug("%s: queue ote header\n", __func__);
	ret = queue_msg(chan_ctx, chan_ctx->data[0], chan_ctx->len[0]);
	if (ret) {
		pr_err("%s:error(%d) in queue header\n", __func__, ret);
		goto err_exit;
	}

	/* queue payload if present */
	if (NULL != buf) {
		trusty_ote_debug("%s: queue payload\n", __func__);
		ret = queue_msg(chan_ctx, chan_ctx->data[1], chan_ctx->len[1]);
		if (ret) {
			pr_err("%s:error(%d) in queue payload\n", __func__, ret);
			goto err_exit;
		}
	}

	trusty_ote_debug("%s: waiting for response\n", __func__);
	ret = wait_for_response(chan_ctx, REPLY_TIMEOUT);
	if (ret < 0) {
		pr_err("%s:ERROR(%d) in receiving header\n", __func__, ret);
		goto err_exit;
	}

	/* sanity check */
	WARN_ON(chan_ctx->state != TIPC_CONNECTED);

	if (NULL != buf) {
		memcpy(&payload_header, payload_buffer, PAYLOAD_META_HEADER_LEN);
		if (payload_header.magic != PAYLOAD_HEADER_MAGIC) {
			ret = -EINVAL;
			pr_err("%s: payload header magic mismatch. received != expected "
				"(%x != %x)\n", __func__, payload_header.magic,
				PAYLOAD_HEADER_MAGIC);
			goto err_exit;
		}
		memcpy(buf, &payload_buffer[PAYLOAD_META_HEADER_LEN], len);
	}

	*op_status = stream_header.status;
	trusty_ote_debug("%s: op status 0x%08x\n", __func__, *op_status);

err_exit:
	kfree(payload_buffer);
	return ret;
}

/*
 * te_open_trusted_session - Establishes the session with TA
 * @name(in): name of the TA to connect to.
 * @ctx(out): pointer to the private data associated to the open session
 * Returns 0 on Success else error code.
 */
int te_open_trusted_session(char *name, void **ctx)
{
	int ret;
	te_error_t op_status = OTE_ERROR_GENERIC;
	struct tipc_chan_ctx *chan_ctx;

	trusty_ote_debug("%s: service %s\n", __func__, name);

	if (!ctx || !name)
		return -EINVAL;

	chan_ctx = kzalloc(sizeof(struct tipc_chan_ctx), GFP_KERNEL);
	if (!chan_ctx) {
		ret = -ENOMEM;
		return ret;
	}

	chan_ctx->chan = tipc_create_channel(NULL, &chan_ops, chan_ctx);
	if(IS_ERR(chan_ctx->chan)) {
		ret = PTR_ERR(chan_ctx->chan);
		pr_err("%s:ERROR(%d) in tipc_create_channel\n", __func__, ret);
		goto err_chan;
	}

	init_completion(&chan_ctx->reply_comp);
	chan_ctx->state = TIPC_DISCONNECTED;

	ret = tipc_chan_connect(chan_ctx->chan, name);
	if (ret) {
		pr_err("%s:ERROR(%d) in tipc_chan_connect\n", __func__, ret);
		goto err_conn;
	}

	ret = wait_for_response(chan_ctx, REPLY_TIMEOUT);
	if (ret < 0) {
		pr_err("%s:ERROR(%d) in receiving response from service\n",
								__func__, ret);
		goto err_conn;
	}

	if (chan_ctx->state == TIPC_NOT_FOUND) {
		ret = -EOPNOTSUPP;
		goto err_conn;
	}

	if (chan_ctx->state != TIPC_CONNECTED) {
		pr_err("%s:Invalid channel state %d\n",
				__func__, chan_ctx->state);
		ret = -ENOTCONN;
		goto err_conn;
	}

	ret = handle_ote_msg(chan_ctx, NULL, 0, 0, &op_status);
	if (ret) {
		pr_err("%s:ERROR(%d) in handle_ote_msg\n", __func__, ret);
		goto err;
	}

	if (op_status) {
		pr_err("%s: ERROR in operation 0x%08x", __func__, op_status);
		ret = -EINVAL;
		goto err;
	}
	*ctx = chan_ctx;
	return 0;

err:
	tipc_chan_shutdown(chan_ctx->chan);
err_conn:
	tipc_chan_destroy(chan_ctx->chan);
	/* chan_ctx is freed in a callback, no need to explicitly free it here */
	return ret;

err_chan:
	kfree(chan_ctx);
	return ret;
}
EXPORT_SYMBOL(te_open_trusted_session);

/*
 * te_close_trusted_session - Closes the session established
 * @ctx: ctx returned by open session
 */
void te_close_trusted_session(void *ctx)
{
	struct tipc_chan_ctx *chan_ctx;

	trusty_ote_debug("%s \n", __func__);
	if (!ctx)
		return;

	chan_ctx = (struct tipc_chan_ctx *)ctx;

	/* wake up the pending client */
	complete(&chan_ctx->reply_comp);

	if (chan_ctx->state == TIPC_CONNECTED)
		tipc_chan_shutdown(chan_ctx->chan);
	tipc_chan_destroy(chan_ctx->chan);
}
EXPORT_SYMBOL(te_close_trusted_session);

/*
 * te_launch_trusted_oper - Communicate with TA to perform any operation
 * @buf: Buffer to be sent to secure world (NULL if a buffer is not required)
 * @buf_len: length of the buffer. (0 if a buffer is not required)
 * @ta_cmd: command to sent to secure world.
 * @ctx: ctx returned by open session.
 * Returns 0 on Success else error code.
 */
int te_launch_trusted_oper(void *buf, size_t buf_len, uint32_t ta_cmd,
		void *ctx)
{
	int ret;
	te_error_t op_status = OTE_ERROR_GENERIC;
	struct tipc_chan_ctx *chan_ctx;

	trusty_ote_debug("%s: cmd %u\n", __func__, ta_cmd);

	if (!ctx)
		return -EINVAL;

	chan_ctx = (struct tipc_chan_ctx *)ctx;

	if (chan_ctx->state != TIPC_CONNECTED) {
		pr_err("%s:Invalid channel state %d\n",
				__func__, chan_ctx->state);
		return -ENOTCONN;
	}

	ret = handle_ote_msg(chan_ctx, buf, buf_len, ta_cmd, &op_status);
	if (ret) {
		pr_err("%s:ERROR(%d) in handle_ote_msg\n", __func__, ret);
		return ret;
	}

	if (op_status) {
		pr_err("%s: ERROR in operation 0x%08x", __func__, op_status);
		return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(te_launch_trusted_oper);
