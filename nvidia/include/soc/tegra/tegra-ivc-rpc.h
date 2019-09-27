/*
 * Copyright (c) 2017 NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef _LINUX_TEGRA_IVC_RPC_H
#define _LINUX_TEGRA_IVC_RPC_H

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/tegra-ivc.h>
#include <linux/tegra-ivc-bus.h>

#include <soc/tegra/tegra-ivc-rpc-common.h>

/*
 * RPC over IVC
 *
 * RPC/IVC provides easy to use interface to send requests to peer
 * and to get responses. The RPC call may be blocking or non-blocking.
 *
 * CCPLEX acts as an initiator and the peer acts as a responder.
 * RPC/IVC sends a message with sequence number to the peer, and when
 * it receives a response message with same sequence number, the result
 * is sent back to the caller.
 *
 * If the RPC call is blocking, the result is returned when returning from
 * the RPC call. If the call is non-blocking and callback function is
 * provided, the callback function is called with response message.
 *
 * The callback functions are executed in the software interrupt context,
 * in a tasklet to be precise.
 *
 * If the IVC channel receives an unexpected message, the channel
 * driver may get a notification. Then the message is discarded.
 */

/*
 * Driver interface
 *
 * IVC channel driver that uses RPC/IVC needs to implement following
 * driver match table.
 *
 * struct of_device_id  tegra_ivc_rpc_ ## channel_name ## _of_match
 *
 * And it requires to implement following to functions.
 *
 * int tegra_ivc_rpc_ ## channel_name ## _probe(struct tegra_ivc_channel *);
 * void tegra_ivc_rpc_ ## channel_name ## _remove(struct tegra_ivc_channel *);
 *
 * To start and clean up the RPC/IVC, above functions should call
 * corresponding RPC start and clean up functions.
 *
 * tegra_ivc_rpc_channel_probe()
 * tegra_ivc_rpc_channel_remove()
 *
 * The channel driver may decide to register callbacks using tegra_ivc_rpc_ops
 * structure. For simple drivers, this can be NULL.
 */

struct tegra_ivc_rpc_ops {
	/* debug */
	void (*create_debugfs)(
		struct tegra_ivc_channel *chan,
		struct dentry *debugfs_root);
	/* after RTCPU gets ready */
	void (*ready)(struct tegra_ivc_channel *chan, bool ready);
	/* notify */
	int (*non_rpc_msg)(
		struct tegra_ivc_channel *chan,
		const void *rsp);
	int (*unexpected_response)(
		struct tegra_ivc_channel *chan,
		const struct tegra_ivc_rpc_response_frame *rsp);
	/* power management */
	int (*pm_prepare)(struct device *dev);
	void (*pm_complete)(struct device *dev);
	int (*pm_suspend)(struct device *dev);
	int (*pm_resume)(struct device *dev);
};

int tegra_ivc_rpc_channel_probe(
	struct tegra_ivc_channel *chan, struct tegra_ivc_rpc_ops *ops);

int tegra_ivc_rpc_channel_remove(
	struct tegra_ivc_channel *chan);

bool tegra_ivc_rpc_channel_is_suspended(
	struct tegra_ivc_channel *chan);

/*
 * Create driver interface structure and function prototypes.
 *
 * channel_name: Name of channel that uses RPC/IVC
 */

void tegra_ivc_rpc_channel_ready(struct tegra_ivc_channel *chan, bool ready);
void tegra_ivc_rpc_channel_notify(struct tegra_ivc_channel *chan);
int tegra_ivc_rpc_channel_pm_prepare(struct device *dev);
void tegra_ivc_rpc_channel_pm_complete(struct device *dev);
int tegra_ivc_rpc_channel_pm_suspend(struct device *dev);
int tegra_ivc_rpc_channel_pm_resume(struct device *dev);

#define TEGRA_IVC_RPC_DRIVER_DEFINE(channel_name, driver_name) \
	static int tegra_ivc_rpc_ ## channel_name ## _probe( \
		struct tegra_ivc_channel *chan); \
	static void tegra_ivc_rpc_ ## channel_name ## _remove( \
		struct tegra_ivc_channel *chan); \
	MODULE_DEVICE_TABLE(of, tegra_ivc_rpc_ ## channel_name ## _of_match); \
	static const struct tegra_ivc_channel_ops \
	tegra_ivc_rpc_ ## channel_name ## _chan_ops = { \
		.probe = tegra_ivc_rpc_ ## channel_name ## _probe, \
		.remove = tegra_ivc_rpc_ ## channel_name ## _remove, \
		.ready = tegra_ivc_rpc_channel_ready, \
		.notify = tegra_ivc_rpc_channel_notify, \
	}; \
	static const struct dev_pm_ops \
	tegra_ivc_rpc_ ## channel_name ## _pm_ops = { \
		.prepare = tegra_ivc_rpc_channel_pm_prepare, \
		.complete = tegra_ivc_rpc_channel_pm_complete, \
		SET_SYSTEM_SLEEP_PM_OPS( \
			tegra_ivc_rpc_channel_pm_suspend, \
			tegra_ivc_rpc_channel_pm_resume) \
	}; \
	static struct tegra_ivc_driver \
	tegra_ivc_channel_ ## channel_name ## _driver = { \
		.driver = { \
			.name = driver_name, \
			.bus = &tegra_ivc_bus_type, \
			.owner = THIS_MODULE, \
			.of_match_table = \
				tegra_ivc_rpc_ ## channel_name ## _of_match, \
			.pm = &tegra_ivc_rpc_ ## channel_name ## _pm_ops, \
		}, \
		.dev_type = &tegra_ivc_channel_type, \
		.ops.channel = &tegra_ivc_rpc_ ## channel_name ## _chan_ops, \
	}; \
	tegra_ivc_subsys_driver_default( \
			tegra_ivc_channel_ ## channel_name ## _driver);

/*
 * Remote procedure call
 *
 * Parameters:
 *   chan: IVC channel handle
 *   request_id: Request message ID
 *   request_len: Length of payload of request message
 *   request: Pointer to payload of request message. NULL if length is 0
 *   reponse_id: Expecting response message.
 *   response_len: Length of payload of response message
 *   response: Pointer to payload of response message. NULL if length is 0
 *   async: False for blocking, true for non-blocking call
 *   callback: Call back function when async is true. Can be NULL
 *   timeout_ms: Timeout in miliseconds when async is false.
 */

/* Error codes generated by RPC driver in initiators */
#define TEGRA_IVC_RPC_ERR_PARAM \
	(TEGRA_IVC_RPC_ERR_RANGE_INIT_BEGIN - 0) /* Invalid parameters */
#define TEGRA_IVC_RPC_ERR_MEMORY \
	(TEGRA_IVC_RPC_ERR_RANGE_INIT_BEGIN - 1) /* Memory error */
#define TEGRA_IVC_RPC_ERR_TIMEOUT \
	(TEGRA_IVC_RPC_ERR_RANGE_INIT_BEGIN - 2) /* No response in time */
#define TEGRA_IVC_RPC_ERR_WRONG_RSP \
	(TEGRA_IVC_RPC_ERR_RANGE_INIT_BEGIN - 3) /* Wrong response received */
#define TEGRA_IVC_RPC_ERR_WRITE_FAILED \
	(TEGRA_IVC_RPC_ERR_RANGE_INIT_BEGIN - 4) /* Cannot write to IVC */

typedef void (*tegra_ivc_rpc_call_callback)(
	int ret,
	const struct tegra_ivc_rpc_response_frame *rep,
	void *param);

struct tegra_ivc_rpc_call_param {
	uint32_t request_id;
	uint32_t request_len;
	void *request;
	uint32_t response_id;
	uint32_t response_len;
	void *response;
	tegra_ivc_rpc_call_callback callback;
	void *callback_param;
	uint32_t timeout_ms;
};

int tegra_ivc_rpc_call(
	struct tegra_ivc_channel *chan,
	const struct tegra_ivc_rpc_call_param *param);

static inline int tegra_ivc_rpc_call_pl(
	struct tegra_ivc_channel *chan,
	uint32_t request_id, uint32_t request_len, void *request,
	uint32_t response_id, uint32_t response_len, void *response,
	tegra_ivc_rpc_call_callback callback,
	void *callback_param, uint32_t timeout_ms)
{
	struct tegra_ivc_rpc_call_param param;

	param.request_id = request_id;
	param.request_len = request_len;
	param.request = request;
	param.response_id = response_id;
	param.response_len = response_len;
	param.response = response;
	param.callback = callback;
	param.callback_param = callback_param;
	param.timeout_ms = timeout_ms;

	return tegra_ivc_rpc_call(chan, &param);
}

#endif /* _LINUX_TEGRA_IVC_RPC_H */
