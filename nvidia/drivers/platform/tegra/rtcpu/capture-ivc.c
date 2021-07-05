/*
 * Capture IVC driver
 *
 * Copyright (c) 2017-2020 NVIDIA Corporation.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/tegra-capture-ivc.h>

#include <linux/completion.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/tegra-ivc.h>
#include <linux/tegra-ivc-bus.h>
#include <linux/nospec.h>

#include <asm/barrier.h>

#include <soc/tegra/camrtc-capture-messages.h>

/* Referred from capture-scheduler.c defined in rtcpu-fw */
#define NUM_CAPTURE_CHANNELS 64

/* Temporary ids for the clients whose channel-id is not yet allocated */
#define NUM_CAPTURE_TRANSACTION_IDS 64

#define TOTAL_CHANNELS (NUM_CAPTURE_CHANNELS + NUM_CAPTURE_TRANSACTION_IDS)
#define TRANS_ID_START_IDX NUM_CAPTURE_CHANNELS

/* Temporay csi channel-id */
#define CSI_TEMP_CHANNEL_ID 65

struct tegra_capture_ivc_cb_ctx {
	struct list_head node;
	tegra_capture_ivc_cb_func cb_func;
	const void *priv_context;
};

struct tegra_capture_ivc {
	struct tegra_ivc_channel *chan;
	struct mutex cb_ctx_lock;
	struct mutex ivc_wr_lock;
	struct work_struct work;
	wait_queue_head_t write_q;
	struct tegra_capture_ivc_cb_ctx cb_ctx[TOTAL_CHANNELS];
	spinlock_t avl_ctx_list_lock;
	struct list_head avl_ctx_list;
};

/*
 * Referred from CAPTURE_MSG_HEADER structure defined
 * in camrtc-capture-messages.h, in rtcpu and UMD.
 */
struct tegra_capture_ivc_msg_header {
	uint32_t msg_id;
	union {
		uint32_t channel_id;
		uint32_t transaction;
	};
} __aligned(8);

/*
 * Referred from CAPTURE_CONTROL_MSG and CAPTURE_MSG structures defined
 * in camrtc-capture-messages.h, in rtcpu and UMD. Only exception is,
 * the msg-id specific structures are opaque here.
 */
struct tegra_capture_ivc_resp {
	struct tegra_capture_ivc_msg_header header;
	void *resp;
};

static int tegra_capture_ivc_tx(struct tegra_capture_ivc *civc,
				const void *req, size_t len)
{
	struct tegra_ivc_channel *chan = civc->chan;
	int ret;

	if (WARN_ON(!chan->is_ready))
		return -EIO;

	ret = mutex_lock_interruptible(&civc->ivc_wr_lock);
	if (unlikely(ret == -EINTR))
		return -ERESTARTSYS;
	if (unlikely(ret))
		return ret;

	ret = wait_event_interruptible(civc->write_q,
				tegra_ivc_can_write(&chan->ivc));
	if (likely(ret == 0))
		ret = tegra_ivc_write(&chan->ivc, req, len);

	mutex_unlock(&civc->ivc_wr_lock);

	if (unlikely(ret < 0))
		dev_err(&chan->dev, "tegra_ivc_write: error %d\n", ret);

	return ret;
}

static struct tegra_capture_ivc *__scivc_control;
static struct tegra_capture_ivc *__scivc_capture;

int tegra_capture_ivc_control_submit(const void *control_desc, size_t len)
{
	if (WARN_ON(__scivc_control == NULL))
		return -ENODEV;

	return tegra_capture_ivc_tx(__scivc_control, control_desc, len);
}
EXPORT_SYMBOL(tegra_capture_ivc_control_submit);

int tegra_capture_ivc_capture_submit(const void *capture_desc, size_t len)
{
	if (WARN_ON(__scivc_capture == NULL))
		return -ENODEV;

	return tegra_capture_ivc_tx(__scivc_capture, capture_desc, len);
}
EXPORT_SYMBOL(tegra_capture_ivc_capture_submit);

int tegra_capture_ivc_register_control_cb(
		tegra_capture_ivc_cb_func control_resp_cb,
		uint32_t *trans_id, const void *priv_context)
{
	struct tegra_capture_ivc *civc;
	struct tegra_capture_ivc_cb_ctx *cb_ctx;
	size_t ctx_id;
	int ret;

	/* Check if inputs are valid */
	if (WARN(control_resp_cb == NULL, "callback function is NULL"))
		return -EINVAL;
	if (WARN(trans_id == NULL, "return value trans_id is NULL"))
		return -EINVAL;
	if (WARN_ON(!__scivc_control))
		return -ENODEV;

	civc = __scivc_control;

	ret = tegra_ivc_channel_runtime_get(civc->chan);
	if (unlikely(ret < 0))
		return ret;

	spin_lock(&civc->avl_ctx_list_lock);
	if (unlikely(list_empty(&civc->avl_ctx_list))) {
		spin_unlock(&civc->avl_ctx_list_lock);
		ret = -EAGAIN;
		goto fail;
	}


	cb_ctx = list_first_entry(&civc->avl_ctx_list,
			struct tegra_capture_ivc_cb_ctx, node);

	list_del(&cb_ctx->node);
	spin_unlock(&civc->avl_ctx_list_lock);

	ctx_id = cb_ctx - &civc->cb_ctx[0];

	if (WARN(ctx_id < TRANS_ID_START_IDX ||
			ctx_id >= ARRAY_SIZE(civc->cb_ctx),
			"invalid cb_ctx %zu", ctx_id)) {
		ret = -EIO;
		goto fail;
	}

	mutex_lock(&civc->cb_ctx_lock);

	if (WARN(cb_ctx->cb_func != NULL, "cb_ctx is busy")) {
		ret = -EIO;
		goto locked_fail;
	}

	*trans_id = (uint32_t)ctx_id;
	cb_ctx->cb_func = control_resp_cb;
	cb_ctx->priv_context = priv_context;

	mutex_unlock(&civc->cb_ctx_lock);

	return 0;

locked_fail:
	mutex_unlock(&civc->cb_ctx_lock);
fail:
	tegra_ivc_channel_runtime_put(civc->chan);
	return ret;
}
EXPORT_SYMBOL(tegra_capture_ivc_register_control_cb);

int tegra_capture_ivc_notify_chan_id(uint32_t chan_id, uint32_t trans_id)
{
	struct tegra_capture_ivc *civc;

	if (WARN(chan_id >= NUM_CAPTURE_CHANNELS, "invalid chan_id"))
		return -EINVAL;
	if (WARN(trans_id < TRANS_ID_START_IDX ||
			trans_id >= TOTAL_CHANNELS, "invalid trans_id"))
		return -EINVAL;
	if (WARN_ON(!__scivc_control))
		return -ENODEV;

	chan_id  = array_index_nospec(chan_id,  NUM_CAPTURE_CHANNELS);
	trans_id = array_index_nospec(trans_id, TOTAL_CHANNELS);

	civc = __scivc_control;

	mutex_lock(&civc->cb_ctx_lock);

	if (WARN(civc->cb_ctx[trans_id].cb_func == NULL,
			"transaction context at %u is idle", trans_id)) {
		mutex_unlock(&civc->cb_ctx_lock);
		return -EBADF;
	}

	if (WARN(civc->cb_ctx[chan_id].cb_func != NULL,
			"channel context at %u is busy", chan_id)) {
		mutex_unlock(&civc->cb_ctx_lock);
		return -EBUSY;
	}

	/* Update cb_ctx index */
	civc->cb_ctx[chan_id].cb_func = civc->cb_ctx[trans_id].cb_func;
	civc->cb_ctx[chan_id].priv_context =
			civc->cb_ctx[trans_id].priv_context;

	/* Reset trans_id cb_ctx fields */
	civc->cb_ctx[trans_id].cb_func = NULL;
	civc->cb_ctx[trans_id].priv_context = NULL;

	mutex_unlock(&civc->cb_ctx_lock);

	spin_lock(&civc->avl_ctx_list_lock);
	list_add_tail(&civc->cb_ctx[trans_id].node, &civc->avl_ctx_list);
	spin_unlock(&civc->avl_ctx_list_lock);

	return 0;
}
EXPORT_SYMBOL(tegra_capture_ivc_notify_chan_id);

int tegra_capture_ivc_register_capture_cb(
		tegra_capture_ivc_cb_func capture_status_ind_cb,
		uint32_t chan_id, const void *priv_context)
{
	struct tegra_capture_ivc *civc;
	int ret;

	if (WARN(capture_status_ind_cb == NULL, "callback function is NULL"))
		return -EINVAL;

	if (WARN(chan_id >= NUM_CAPTURE_CHANNELS,
			"invalid channel id %u", chan_id))
		return -EINVAL;
	chan_id = array_index_nospec(chan_id, NUM_CAPTURE_CHANNELS);

	if (!__scivc_capture)
		return -ENODEV;

	civc = __scivc_capture;

	ret = tegra_ivc_channel_runtime_get(civc->chan);
	if (ret < 0)
		return ret;

	mutex_lock(&civc->cb_ctx_lock);

	if (WARN(civc->cb_ctx[chan_id].cb_func != NULL,
			"capture channel %u is busy", chan_id)) {
		ret = -EBUSY;
		goto fail;
	}

	civc->cb_ctx[chan_id].cb_func = capture_status_ind_cb;
	civc->cb_ctx[chan_id].priv_context = priv_context;
	mutex_unlock(&civc->cb_ctx_lock);

	return 0;
fail:
	mutex_unlock(&civc->cb_ctx_lock);
	tegra_ivc_channel_runtime_put(civc->chan);

	return ret;
}
EXPORT_SYMBOL(tegra_capture_ivc_register_capture_cb);

int tegra_capture_ivc_unregister_control_cb(uint32_t id)
{
	struct tegra_capture_ivc *civc;

	/* id could be temporary trans_id or rtcpu-allocated chan_id */
	if (WARN(id >= TOTAL_CHANNELS, "invalid id %u", id))
		return -EINVAL;
	if (WARN_ON(!__scivc_control))
		return -ENODEV;

	id = array_index_nospec(id, TOTAL_CHANNELS);

	civc = __scivc_control;

	mutex_lock(&civc->cb_ctx_lock);

	if (WARN(civc->cb_ctx[id].cb_func == NULL,
			"control channel %u is idle", id)) {
		mutex_unlock(&civc->cb_ctx_lock);
		return -EBADF;
	}

	civc->cb_ctx[id].cb_func = NULL;
	civc->cb_ctx[id].priv_context = NULL;

	mutex_unlock(&civc->cb_ctx_lock);

	/*
	 * If it's trans_id, client encountered an error before or during
	 * chan_id update, in that case the corresponding cb_ctx
	 * needs to be added back in the avilable cb_ctx list.
	 */
	if (id >= TRANS_ID_START_IDX) {
		spin_lock(&civc->avl_ctx_list_lock);
		list_add_tail(&civc->cb_ctx[id].node, &civc->avl_ctx_list);
		spin_unlock(&civc->avl_ctx_list_lock);
	}

	tegra_ivc_channel_runtime_put(civc->chan);

	return 0;
}
EXPORT_SYMBOL(tegra_capture_ivc_unregister_control_cb);

int tegra_capture_ivc_unregister_capture_cb(uint32_t chan_id)
{
	struct tegra_capture_ivc *civc;

	if (chan_id >= NUM_CAPTURE_CHANNELS)
		return -EINVAL;

	if (!__scivc_capture)
		return -ENODEV;

	chan_id = array_index_nospec(chan_id, NUM_CAPTURE_CHANNELS);

	civc = __scivc_capture;

	mutex_lock(&civc->cb_ctx_lock);

	if (WARN(civc->cb_ctx[chan_id].cb_func == NULL,
			"capture channel %u is idle", chan_id)) {
		mutex_unlock(&civc->cb_ctx_lock);
		return -EBADF;
	}

	civc->cb_ctx[chan_id].cb_func = NULL;
	civc->cb_ctx[chan_id].priv_context = NULL;

	mutex_unlock(&civc->cb_ctx_lock);

	tegra_ivc_channel_runtime_put(civc->chan);

	return 0;
}
EXPORT_SYMBOL(tegra_capture_ivc_unregister_capture_cb);

static void tegra_capture_ivc_worker(struct work_struct *work)
{
	struct tegra_capture_ivc *civc = container_of(work,
					struct tegra_capture_ivc, work);
	struct tegra_ivc_channel *chan = civc->chan;

	WARN_ON(!chan->is_ready);

	while (tegra_ivc_can_read(&chan->ivc)) {
		const struct tegra_capture_ivc_resp *msg =
			tegra_ivc_read_get_next_frame(&chan->ivc);
		uint32_t id = msg->header.channel_id;

		/* Check if message is valid */
		if (WARN(id >= TOTAL_CHANNELS, "Invalid rtcpu response id %u", id))
			goto skip;

		id = array_index_nospec(id, TOTAL_CHANNELS);

		/* Check if callback function available */
		if (unlikely(!civc->cb_ctx[id].cb_func)) {
			dev_dbg(&chan->dev, "No callback for id %u\n", id);
			goto skip;
		}

		/* WAR: Skip the callback if channel-id is 65, and msg-id is
		 * greater than CAPTURE_CHANNEL_ISP_RELEASE_RESP. Channel id
		 * 65 is used for csi and it is specific to v4l2.
		 * TODO: Bug 200619454
		 */
		/* Invoke client callback.*/
		if (msg->header.msg_id >= CAPTURE_CHANNEL_ISP_RELEASE_RESP &&
			id == CSI_TEMP_CHANNEL_ID) {
			dev_err(&chan->dev,
				"No callback found for msg id: 0x%x",
				msg->header.msg_id);
		} else {
			civc->cb_ctx[id].cb_func(msg,
				civc->cb_ctx[id].priv_context);
		}
skip:
		tegra_ivc_read_advance(&chan->ivc);
	}
}

static void tegra_capture_ivc_notify(struct tegra_ivc_channel *chan)
{
	struct tegra_capture_ivc *civc = tegra_ivc_channel_get_drvdata(chan);

	/* Only 1 thread can wait on write_q, rest wait for write_lock */
	wake_up(&civc->write_q);
	schedule_work(&civc->work);
}

#define NV(x) "nvidia," #x

static int tegra_capture_ivc_probe(struct tegra_ivc_channel *chan)
{
	struct device *dev = &chan->dev;
	struct tegra_capture_ivc *civc;
	const char *service;
	int ret;
	uint32_t i;

	civc = devm_kzalloc(dev, (sizeof(*civc)), GFP_KERNEL);
	if (unlikely(civc == NULL))
		return -ENOMEM;

	ret = of_property_read_string(dev->of_node, NV(service),
			&service);
	if (unlikely(ret)) {
		dev_err(dev, "missing <%s> property\n", NV(service));
		return ret;
	}

	civc->chan = chan;

	mutex_init(&civc->cb_ctx_lock);
	mutex_init(&civc->ivc_wr_lock);

	/* Initialize ivc_work */
	INIT_WORK(&civc->work, tegra_capture_ivc_worker);

	/* Initialize wait queue */
	init_waitqueue_head(&civc->write_q);

	/* transaction-id list of available callback contexts */
	spin_lock_init(&civc->avl_ctx_list_lock);
	INIT_LIST_HEAD(&civc->avl_ctx_list);

	/* Add the transaction cb-contexts to the available list */
	for (i = TRANS_ID_START_IDX; i < ARRAY_SIZE(civc->cb_ctx); i++)
		list_add_tail(&civc->cb_ctx[i].node, &civc->avl_ctx_list);

	tegra_ivc_channel_set_drvdata(chan, civc);

	if (!strcmp("capture-control", service)) {
		if (WARN_ON(__scivc_control != NULL))
			return -EEXIST;
		__scivc_control = civc;
	} else if (!strcmp("capture", service)) {
		if (WARN_ON(__scivc_capture != NULL))
			return -EEXIST;
		__scivc_capture = civc;
	} else {
		dev_err(dev, "Unknown ivc channel %s\n", service);
		return -EINVAL;
	}

	return 0;
}

static void tegra_capture_ivc_remove(struct tegra_ivc_channel *chan)
{
	struct tegra_capture_ivc *civc = tegra_ivc_channel_get_drvdata(chan);

	cancel_work_sync(&civc->work);

	if (__scivc_control == civc)
		__scivc_control = NULL;
	else if (__scivc_capture == civc)
		__scivc_capture = NULL;
	else
		dev_WARN(&chan->dev, "Unknown ivc channel\n");
}

static struct of_device_id tegra_capture_ivc_channel_of_match[] = {
	{ .compatible = "nvidia,tegra186-camera-ivc-protocol-capture-control" },
	{ .compatible = "nvidia,tegra186-camera-ivc-protocol-capture" },
	{ },
};

static const struct tegra_ivc_channel_ops tegra_capture_ivc_ops = {
	.probe	= tegra_capture_ivc_probe,
	.remove	= tegra_capture_ivc_remove,
	.notify	= tegra_capture_ivc_notify,
};

static struct tegra_ivc_driver tegra_capture_ivc_driver = {
	.driver = {
		.name	= "tegra-capture-ivc",
		.bus	= &tegra_ivc_bus_type,
		.owner	= THIS_MODULE,
		.of_match_table = tegra_capture_ivc_channel_of_match,
	},
	.dev_type	= &tegra_ivc_channel_type,
	.ops.channel	= &tegra_capture_ivc_ops,
};

tegra_ivc_subsys_driver_default(tegra_capture_ivc_driver);
MODULE_AUTHOR("Sudhir Vyas <svyas@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra Capture IVC driver");
MODULE_LICENSE("GPL");
