/*
 * Tegra capture IVC driver.
 *
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef INCLUDE_CAPTURE_IVC_H
#define INCLUDE_CAPTURE_IVC_H

#include <linux/types.h>

/*
 * Submit the control message binary blob to capture-IVC driver,
 * which is to be transfered over control IVC channel to RTCPU.
 *
 * @param[in] control_desc: binary blob containing control message descriptor,
 * is opaque to capture-IVC driver.
 * @param[in] len: size of control_desc.
 */
int tegra_capture_ivc_control_submit(const void *control_desc, size_t len);

/*
 * Submit the capture message binary blob to capture-IVC driver,
 * which is to be transfered over capture IVC channel to RTCPU.
 *
 * @param[in] capture_desc: binary blob containing capture message descriptor,
 * is opaque to KMDs.
 * @param[in] len: size of capture_desc.
 */
int tegra_capture_ivc_capture_submit(const void *capture_desc, size_t len);

/*
 * Callback function to be registered by client to receive the rtcpu
 * notifications through control or capture IVC channel.
 *
 * @param[in] resp_desc: binary blob containing the response message received
 * from rtcpu through control or capture IVC channel, its opaque to KMDs.
 * @param[in] priv_context: Client's private context, opaque to
 * capture-IVC driver.
 */
typedef void (*tegra_capture_ivc_cb_func)(const void *resp_desc,
		const void *priv_context);

/*
 * Register callback function to receive response messages from rtcpu through
 * control IVC channel.
 *
 * @param[in] control_resp_cb: callback function to be registered for
 * control IVC channel.
 * @param[in] priv_context: client's private context, opaque to
 * capture-IVC driver.
 * @param[out] trans_id: temporary id assigned by capture-IVC driver, for the
 * clients whose unique chan_id is not yet allocated by RTCPU, to match their
 * responses with the requests.
 *
 */
int tegra_capture_ivc_register_control_cb(
		tegra_capture_ivc_cb_func control_resp_cb,
		uint32_t *trans_id, const void *priv_context);

/*
 * Notify client’s channel ID to capture-IVC driver.
 *
 * Once client gets the newly allocated channel ID from RTCPU, it has to
 * notify it to capture-IVC driver also, so that it can replace the temporary
 * ID trans_id with the new channel ID chan_id in its internal context.
 * IVC driver uses this unique channel ID for mapping upcoming responses
 * with the client requests.
 *
 * @param[in] chan_id: new channel id allocated by RTCPU for the client,
 * capture-IVC driver uses to refer the client for its future control responses.
 * @param[in] trans_id: temporary id assigned by capture-IVC driver,
 * for the client.
 */
int tegra_capture_ivc_notify_chan_id(uint32_t chan_id, uint32_t trans_id);

/*
 * Register callback function to receive status-indication messages from
 * rtcpu through capture IVC channel.
 *
 * @param[in] capture_status_ind_cb: callback function to be registered for
 * capture ivc channel.
 * @param[in] chan_id: client's channel id, capture-IVC driver uses it refer
 * the client for its capture responses.
 * @param[in] priv_context: client's private context, opaque to
 * capture-IVC driver.
 */
int tegra_capture_ivc_register_capture_cb(
		tegra_capture_ivc_cb_func capture_status_ind_cb,
		uint32_t chan_id, const void *priv_context);

/*
 * Un-register callback function to stop receiving messages over control ivc
 * channel.
 *
 * @param[in] id: client's channel id or transaction id, for which the callback
 * needs to be unregistered.
 */
int tegra_capture_ivc_unregister_control_cb(uint32_t id);

/*
 * Un-register callback function to stop receiving messages over capture ivc
 * channel.
 *
 * @param[in] chan_id: client's channel id, for which the callback needs to be
 * unregistered.
 */
int tegra_capture_ivc_unregister_capture_cb(uint32_t chan_id);

#endif /* INCLUDE_CAPTURE_IVC_H */
