/*
 * VI NOTIFY driver for Tegra186
 *
 * Copyright (c) 2015-2017 NVIDIA Corporation.  All rights reserved.
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

#include <linux/tegra-camera-rtcpu.h>

#include <linux/completion.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <linux/sched/signal.h>
#endif
#include <linux/slab.h>
#include <linux/tegra-ivc.h>
#include <linux/tegra-ivc-bus.h>
#include <linux/workqueue.h>
#include <linux/time.h>
#include <linux/time64.h>
#include <asm/arch_timer.h>
#include <linux/timekeeping.h>

#include "drivers/video/tegra/host/vi/vi_notify.h"
#include "drivers/video/tegra/host/nvhost_acm.h"

#include "vi-notify.h"

#define BUG_200219206

#define ADJUST_TS_FREQUENCY (100)

struct vi_notify_req {
	union {
		struct {
			u8 type;
			u8 channel;
			u8 stream;
			u8 vc;
			union {
				u32 syncpt_ids[3];
				u32 mask;
			};
			u64 status_base_addr;
			u16 status_entries;
		};
		char size[64];
	};
};

/* Extended VI notify message */
struct vi_notify_msg_ex {
	u32 type;	/* message type (LSB=0) */
	u32 dest;	/* destination channels (bitmask) */
	u32 size;	/* data size */
	u8 data[];	/* payload data */
};


enum {
	TEGRA_IVC_VI_CLASSIFY,
	TEGRA_IVC_VI_SET_SYNCPTS,
	TEGRA_IVC_VI_RESET_CHANNEL,
	TEGRA_IVC_VI_ENABLE_REPORTS,
	TEGRA_IVC_VI_ENABLE_REPORTS_2,
};

struct tegra_ivc_vi_notify {
	struct tegra_ivc_channel *chan;
	struct vi_notify_dev *vi_notify;
	struct platform_device *vi;
	u32 tags;
	u16 channels_mask;
	wait_queue_head_t write_q;
	struct completion ack;
	struct work_struct notify_work;
	size_t status_mem_size;
	struct vi_capture_status __iomem *status_mem;
	dma_addr_t status_dmaptr;
	u16 status_entries;
	u32 adjust_ts_counter;
	u64 ts_res_ns;
	s64 ts_adjustment;
};

static int tegra_ivc_vi_notify_status(struct tegra_ivc_channel *chan,
				const struct vi_notify_msg_ex *msg);

static inline s64 get_ts_adjustment(u64 tsc_res)
{
	s64 tsc = 0;
	struct timespec64 ts;
	s64 delta1 = 0, delta2 = 0;
	s64 mono_time = 0;
	u8 tries = 0;
	#define _MAX_ADJUSTMENT_TRIES (5)
	#define _DELTA_DIFF_THRESHOLD (5000)

	do {
		tsc = (s64)(arch_counter_get_cntvct() * tsc_res);

		ktime_get_ts64(&ts);
		mono_time = timespec_to_ns(&ts);

		delta1 = mono_time - tsc;
		tsc = (s64)(arch_counter_get_cntvct() * tsc_res);
		delta2 = mono_time - tsc;

		tries++;
	} while ((tries < _MAX_ADJUSTMENT_TRIES) &&
		    (abs(delta2 - delta1) > _DELTA_DIFF_THRESHOLD));

	WARN_ON(tries == _MAX_ADJUSTMENT_TRIES);
	#undef _MAX_ADJUSTMENT_TRIES
	#undef _DELTA_DIFF_THRESHOLD

	return delta1;
}

static void tegra_ivc_vi_notify_process(struct tegra_ivc_channel *chan,
					const struct vi_notify_msg_ex *msg,
					size_t len)
{
	struct tegra_ivc_vi_notify *ivn = tegra_ivc_channel_get_drvdata(chan);

	if (sizeof(*msg) > len) {
		dev_warn(&chan->dev, "Invalid extended message.\n");
		return;
	}

	switch (msg->type) {
	case VI_NOTIFY_MSG_ACK:
		complete(&ivn->ack);
		break;
	case VI_NOTIFY_MSG_STATUS:
		tegra_ivc_vi_notify_status(chan, msg);
		break;
	default:
		dev_warn(&chan->dev, "Unknown message type: %u\n", msg->type);
		break;
	}
}

static void tegra_ivc_vi_notify_recv(struct tegra_ivc_channel *chan,
					const void *data, size_t len)
{
	struct tegra_ivc_vi_notify *ivn = tegra_ivc_channel_get_drvdata(chan);
	const struct vi_notify_msg *msg = data;

	if (len >= sizeof(*msg) && VI_NOTIFY_TAG_VALID(msg->tag))
		vi_notify_dev_recv(ivn->vi_notify, msg);
	else
		tegra_ivc_vi_notify_process(chan, data, len);
}

static void tegra_ivc_channel_vi_notify_worker(struct work_struct *work)
{
	struct tegra_ivc_channel *chan;

	chan = container_of(work,
			struct tegra_ivc_vi_notify, notify_work)->chan;

	while (tegra_ivc_can_read(&chan->ivc)) {
		const void *data = tegra_ivc_read_get_next_frame(&chan->ivc);
		size_t length = chan->ivc.frame_size;

		tegra_ivc_vi_notify_recv(chan, data, length);
		tegra_ivc_read_advance(&chan->ivc);
	}
}

/* Called from interrupt handler */
static void tegra_ivc_channel_vi_notify_process(struct tegra_ivc_channel *chan)
{
	struct tegra_ivc_vi_notify *ivn = tegra_ivc_channel_get_drvdata(chan);

	WARN_ON(!chan->is_ready);

	wake_up(&ivn->write_q);
	schedule_work(&ivn->notify_work);
}

/* VI Notify */
static int tegra_ivc_vi_notify_send(struct tegra_ivc_channel *chan,
					const struct vi_notify_req *req)
{
	struct tegra_ivc_vi_notify *ivn = tegra_ivc_channel_get_drvdata(chan);
	int ret = 0;

	if (WARN_ON(!chan->is_ready))
		return -EIO;

	while (ret == 0) {
		DEFINE_WAIT(wait);

		prepare_to_wait(&ivn->write_q, &wait, TASK_INTERRUPTIBLE);

		ret = tegra_ivc_write(&chan->ivc, req, sizeof(*req));
		if (ret >= 0)
			;
		else if (ret != -ENOMEM)
			dev_err(&chan->dev, "cannot send request: %d\n", ret);
		else if (signal_pending(current))
			ret = -ERESTARTSYS;
		else {
			ret = 0;
			schedule();
		}

		finish_wait(&ivn->write_q, &wait);
	}

	if (ret < 0)
		return ret;

	/* Wait for RTCPU to acknowledge the request. This fixes races such as:
	 * - RTCPU attempting to use a powered-off VI,
	 * - VI emitting an event before the request is processed. */
	ret = wait_for_completion_killable_timeout(&ivn->ack, HZ);
	if (ret <= 0) {
		dev_err(&chan->dev, "no reply from camera processor\n");
#ifndef BUG_200219206
		WARN_ON(1);
#endif
		return -ETIMEDOUT;
	}

	return 0;
}

static int tegra_ivc_vi_notify_probe(struct device *dev,
					struct vi_notify_dev *vnd)
{
	struct tegra_ivc_vi_notify *ivn = dev_get_drvdata(dev);

	ivn->vi_notify = vnd;
	return 0;
}

static int tegra_ivc_vi_notify_classify(struct device *dev, u32 mask)
{
	struct tegra_ivc_channel *chan = to_tegra_ivc_channel(dev);
	struct tegra_ivc_vi_notify *ivn = tegra_ivc_channel_get_drvdata(chan);
	struct vi_notify_req req = {
		.type = TEGRA_IVC_VI_CLASSIFY,
		.mask = mask,
	};
	int err;

	if (ivn->tags == mask)
		return 0; /* nothing to do */

	err = tegra_ivc_vi_notify_send(chan, &req);
	if (likely(err == 0))
		ivn->tags = mask;

	return err;
}

static int tegra_ivc_vi_notify_set_syncpts(struct device *dev, u8 ch,
					const u32 ids[3])
{
	struct tegra_ivc_channel *chan = to_tegra_ivc_channel(dev);
	struct tegra_ivc_vi_notify *ivn = tegra_ivc_channel_get_drvdata(chan);
	struct vi_notify_req msg = {
		.type = TEGRA_IVC_VI_SET_SYNCPTS,
		.channel = ch,
	};
	int err;

	memcpy(msg.syncpt_ids, ids, sizeof(msg.syncpt_ids));

	err = tegra_ivc_vi_notify_send(chan, &msg);
	if (likely(err == 0))
		ivn->channels_mask |= 1u << ch;

	return err;
}

static int tegra_ivc_vi_notify_enable_reports(struct device *dev, u8 ch,
					u8 st, u8 vc, const u32 ids[3])
{
	struct tegra_ivc_channel *chan = to_tegra_ivc_channel(dev);
	struct tegra_ivc_vi_notify *ivn = tegra_ivc_channel_get_drvdata(chan);

	u64 status_dmaptr = ivn->status_dmaptr +
		ch * ivn->status_entries * sizeof(*ivn->status_mem);

	struct vi_notify_req msg = {
		.type = TEGRA_IVC_VI_ENABLE_REPORTS_2,
		.channel = ch,
		.stream = st,
		.vc = vc,
		.status_base_addr = status_dmaptr,
		.status_entries = ivn->status_entries,
	};
	int err;

	memcpy(msg.syncpt_ids, ids, sizeof(msg.syncpt_ids));

	err = tegra_ivc_vi_notify_send(chan, &msg);
	if (likely(err == 0))
		ivn->channels_mask |= 1u << ch;

	ivn->adjust_ts_counter = 0;
	ivn->ts_adjustment = 0;
	#define _PICO_SECS (1000000000000ULL)
	ivn->ts_res_ns = (_PICO_SECS/(u64)arch_timer_get_cntfrq())/1000;
	#undef _PICO_SECS

	return err;
}

static void tegra_ivc_vi_notify_reset_channel(struct device *dev, u8 ch)
{
	struct tegra_ivc_channel *chan = to_tegra_ivc_channel(dev);
	struct tegra_ivc_vi_notify *ivn = tegra_ivc_channel_get_drvdata(chan);
	struct vi_notify_req msg = {
		.type = TEGRA_IVC_VI_RESET_CHANNEL,
		.channel = ch,
	};
	int err;

	if (unlikely((ivn->channels_mask & (1u << ch)) == 0))
		return;

	err = tegra_ivc_vi_notify_send(chan, &msg);
	if (likely(err == 0))
		ivn->channels_mask &= ~(1u << ch);
}

/* Get a camera-rtcpu device */
static struct device *camrtc_get_device(struct tegra_ivc_channel *ch)
{
	if (unlikely(ch == NULL))
		return NULL;

	BUG_ON(ch->dev.parent == NULL);
	BUG_ON(ch->dev.parent->parent == NULL);

	return ch->dev.parent->parent;
}

static bool tegra_ivc_vi_notify_has_notifier_backend(struct device *dev)
{
	struct tegra_ivc_channel *chan = to_tegra_ivc_channel(dev);
	struct device *rce_dev = camrtc_get_device(chan);
	bool alive = tegra_camrtc_is_rtcpu_alive(rce_dev);

	if (!alive)
		dev_err(dev, "vi_notifier_backend is down\n");

	return alive;
}

static int tegra_ivc_vi_notify_get_capture_status(struct device *dev,
				unsigned ch,
				u64 index,
				struct vi_capture_status *status)
{
	struct tegra_ivc_channel *chan = to_tegra_ivc_channel(dev);
	struct tegra_ivc_vi_notify *ivn = tegra_ivc_channel_get_drvdata(chan);
	struct vi_capture_status __iomem *status_mem =
		&ivn->status_mem[ch * ivn->status_entries];
	int err = 0;

	if (ivn->status_entries != 0) {
		index &= ivn->status_entries - 1;
		memcpy_fromio(status, &status_mem[index], sizeof(*status));

		if (ivn->adjust_ts_counter % ADJUST_TS_FREQUENCY == 0)
			ivn->ts_adjustment = get_ts_adjustment(ivn->ts_res_ns);

		if (status->sof_ts != 0) {
			s64 ts = (s64)status->sof_ts;
			ts += ivn->ts_adjustment;
			status->sof_ts = (u64)ts;
		}

		if (status->eof_ts != 0) {
			s64 ts = (s64)status->eof_ts;
			ts += ivn->ts_adjustment;
			status->eof_ts = (u64)ts;
		}

		ivn->adjust_ts_counter++;
	} else {
		err = -ENOTSUPP;
	}
	return err;
}

static int tegra_ivc_vi_notify_runtime_get(struct device *dev)
{
	struct tegra_ivc_channel *chan = to_tegra_ivc_channel(dev);
	struct tegra_ivc_vi_notify *ivn = tegra_ivc_channel_get_drvdata(chan);
	int err = nvhost_module_busy(ivn->vi);

	if (err < 0)
		return err;

	err = tegra_ivc_channel_runtime_get(chan);
	if (err < 0)
		nvhost_module_idle(ivn->vi);

	return err;
}

static void tegra_ivc_vi_notify_runtime_put(struct device *dev)
{
	struct tegra_ivc_channel *chan = to_tegra_ivc_channel(dev);
	struct tegra_ivc_vi_notify *ivn = tegra_ivc_channel_get_drvdata(chan);

	tegra_ivc_channel_runtime_put(chan);

	nvhost_module_idle(ivn->vi);
}

static struct vi_notify_driver tegra_ivc_vi_notify_driver = {
	.owner		= THIS_MODULE,
	.probe		= tegra_ivc_vi_notify_probe,
	.classify	= tegra_ivc_vi_notify_classify,
	.set_syncpts	= tegra_ivc_vi_notify_set_syncpts,
	.enable_reports = tegra_ivc_vi_notify_enable_reports,
	.reset_channel	= tegra_ivc_vi_notify_reset_channel,
	.has_notifier_backend = tegra_ivc_vi_notify_has_notifier_backend,
	.get_capture_status = tegra_ivc_vi_notify_get_capture_status,
	.runtime_get    = tegra_ivc_vi_notify_runtime_get,
	.runtime_put    = tegra_ivc_vi_notify_runtime_put,
};

/* Platform device */
static struct platform_device *tegra_vi_get(struct device *dev)
{
	struct device_node *vi_node;
	struct platform_device *vi_pdev;

	vi_node = of_parse_phandle(dev->of_node, "device", 0);
	if (vi_node == NULL) {
		dev_err(dev, "cannot get VI device");
		return ERR_PTR(-ENODEV);
	}

	vi_pdev = of_find_device_by_node(vi_node);
	of_node_put(vi_node);

	if (vi_pdev == NULL)
		return ERR_PTR(-EPROBE_DEFER);

	if (&vi_pdev->dev.driver == NULL) {
		platform_device_put(vi_pdev);
		return ERR_PTR(-EPROBE_DEFER);
	}
	return vi_pdev;
}

static int tegra_ivc_vi_notify_status(struct tegra_ivc_channel *chan,
				const struct vi_notify_msg_ex *msg)
{
	struct tegra_ivc_vi_notify *ivn = tegra_ivc_channel_get_drvdata(chan);
	u32 mask;
	u8 ch;

	if (!ivn) {
		pr_err("ivc-vi-notify does not exist!\n");
		return -ENODEV;
	}

	if (!ivn->channels_mask) {
		pr_info("No vi channel is active\n");
		return 0;
	}


	if (msg->size != sizeof(struct vi_capture_status)) {
		pr_warn("vi-notify: Invalid status message.\n");
		return -EINVAL;
	}

	mask = msg->dest & ivn->channels_mask;

	for (ch = 0; mask; mask >>= 1, ch++) {
		if (!(mask & 1u))
			continue;
		vi_notify_dev_report(ivn->vi_notify, ch,
				(struct vi_capture_status *)msg->data);
	}

	return 0;
}

static void tegra_ivc_channel_vi_notify_ready(struct tegra_ivc_channel *chan,
	bool online)
{
	struct tegra_ivc_vi_notify *ivn = tegra_ivc_channel_get_drvdata(chan);
	const struct vi_capture_status ev = {
		/*
		 * NOTIFIER_BACKEND_DOWN is a special all-channel error message
		 */
		.status = VI_CAPTURE_STATUS_NOTIFIER_BACKEND_DOWN,
	};
	u8 ch;
	u32 channels_mask = ivn->channels_mask;

	if (ivn->vi_notify == NULL || channels_mask == 0)
		return;

	if (!online) {
		dev_info(&chan->dev, "notify backend down");

		/* Broadcast the error to all active channels */
		for (ch = 0; channels_mask != 0; channels_mask >>= 1, ch++)
			if ((channels_mask & 1U) != 0)
				vi_notify_dev_report(ivn->vi_notify, ch, &ev);
	}
}


static int tegra_ivc_channel_vi_notify_probe(struct tegra_ivc_channel *chan)
{
	struct tegra_ivc_vi_notify *ivn = tegra_ivc_channel_get_drvdata(chan);
	struct device *dev = tegra_ivc_channel_to_camrtc_dev(chan);
	int err;

	ivn = devm_kzalloc(&chan->dev, sizeof(*ivn), GFP_KERNEL);
	if (unlikely(ivn == NULL))
		return -ENOMEM;

	chan->is_ready = false;
	ivn->vi = tegra_vi_get(&chan->dev);
	if (IS_ERR(ivn->vi))
		return PTR_ERR(ivn->vi);

	/* This must be power of 2 */
	BUG_ON(VI_NOTIFY_STATUS_ENTRIES & (VI_NOTIFY_STATUS_ENTRIES - 1));

	ivn->status_mem_size = sizeof(*ivn->status_mem)
		* VI_NOTIFY_STATUS_ENTRIES * VI_NOTIFY_MAX_VI_CHANS;

	ivn->status_mem = dma_alloc_coherent(dev,
		ivn->status_mem_size,
		&ivn->status_dmaptr, GFP_KERNEL | __GFP_ZERO);

	if (unlikely(ivn->status_mem == NULL))
		ivn->status_dmaptr = 0;
	else
		ivn->status_entries = VI_NOTIFY_STATUS_ENTRIES;

	ivn->chan = chan;
	init_waitqueue_head(&ivn->write_q);
	init_completion(&ivn->ack);
	INIT_WORK(&ivn->notify_work, tegra_ivc_channel_vi_notify_worker);

	tegra_ivc_channel_set_drvdata(chan, ivn);

	err = vi_notify_register(&tegra_ivc_vi_notify_driver,
		 &chan->dev, VI_NOTIFY_MAX_VI_CHANS);
	if (err)
		platform_device_put(ivn->vi);
	return err;
}

static void tegra_ivc_channel_vi_notify_remove(struct tegra_ivc_channel *chan)
{
	struct tegra_ivc_vi_notify *ivn = tegra_ivc_channel_get_drvdata(chan);
	struct device *dev = tegra_ivc_channel_to_camrtc_dev(chan);

	if (likely(ivn->status_mem != NULL))
		dma_free_coherent(dev,
			ivn->status_mem_size,
			ivn->status_mem, ivn->status_dmaptr);

	vi_notify_unregister(&tegra_ivc_vi_notify_driver, &chan->dev);
	platform_device_put(ivn->vi);
}

static struct of_device_id tegra_ivc_channel_vi_notify_of_match[] = {
	{ .compatible = "nvidia,tegra186-camera-ivc-protocol-vinotify" },
	{ },
};

static const struct tegra_ivc_channel_ops tegra_ivc_channel_vi_notify_ops = {
	.probe	= tegra_ivc_channel_vi_notify_probe,
	.ready = tegra_ivc_channel_vi_notify_ready,
	.remove	= tegra_ivc_channel_vi_notify_remove,
	.notify	= tegra_ivc_channel_vi_notify_process,
};

static struct tegra_ivc_driver tegra_ivc_channel_vi_notify_driver = {
	.driver = {
		.name	= "tegra-ivc-vi-notify",
		.bus	= &tegra_ivc_bus_type,
		.owner	= THIS_MODULE,
		.of_match_table = tegra_ivc_channel_vi_notify_of_match,
	},
	.dev_type	= &tegra_ivc_channel_type,
	.ops.channel	= &tegra_ivc_channel_vi_notify_ops,
};
tegra_ivc_subsys_driver_default(tegra_ivc_channel_vi_notify_driver);
MODULE_AUTHOR("Remi Denis-Courmont <remid@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra IVC VI Notify driver");
MODULE_LICENSE("GPL");
