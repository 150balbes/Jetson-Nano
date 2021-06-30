/*
 * Copyright (C) 2016-2017 NVIDIA Corporation.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

/*
 * Flow of operation in synchronous mode.
 *
 * tegra_ivc_rpc_call()
 *   Allocates a tx descriptor, fills necessary information.
 *   Creates a request frame in the descriptor.
 *   Sends out the request frame.
 *   Adds the descriptor to tx_list.
 *   Calls wait_for_completion_timeout().
 *
 * tegra_ivc_rpc_channel_notify()
 *   Schedules the tasklet.
 *
 * tegra_ivc_rpc_rx_tasklet()
 *   Reads the received frame.
 *   Locates the tx descriptor with matching sequence number.
 *   Collects the response information.
 *   Calls complete().
 *
 * back to tegra_ivc_rpc_call()
 *   Frees the tx descriptor.
 *   Return the result back to caller.
 *
 * Difference in asynchronous mode.
 *
 * tegra_ivc_rpc_call()
 *   Instead of calling wait_for_completion_timeout(), set up a timer,
 *   and returns to the caller immediately.
 *
 * tegra_ivc_rpc_rx_tasklet()
 *   Instead of calling complete(), calls the callback function.
 *   Frees the tx descriptor.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/time.h>
#include <linux/sched.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <linux/sched/signal.h>
#include <linux/sched/clock.h>
#endif

#include <soc/tegra/tegra-ivc-rpc.h>

/*
 * ENABLE_IVC_RPC_TRACE is used to enable tracing
 * at boot time.
 * For tracing at boot. Makse sure to update bootargs
 * in DT file and add trace_event=tegra_rpc:*
 */
#define ENABLE_IVC_RPC_TRACE 0
#define CREATE_TRACE_POINTS
#include <trace/events/tegra_ivc_rpc.h>

/*
 * Configuration
 */

#define TEGRA_IVC_RPC_TIMEOUT_MS_DEFAULT	500

/*
 * RPC private data structures
 */

struct tegra_ivc_rpc_data {
	/* IVC channel */
	struct tegra_ivc_channel *chan;
	/* general purpose lock */
	struct mutex lock;
	wait_queue_head_t ivc_wq;
	/* sequence number */
	atomic_t next_seq_num;
	/* registered operations */
	struct tegra_ivc_rpc_ops *ops;
	/* request list */
	spinlock_t tx_list_lock;
	struct list_head tx_list;
	/* deferred procedure call */
	struct tasklet_struct rx_tasklet;
	/* debugfs */
	struct dentry *debugfs_root;
	/* statistics counters */
	unsigned int count_tx;
	unsigned int count_rx_good;
	unsigned int count_rx_timeout;
	unsigned int count_rx_non_rpc;
	unsigned int count_rx_unexpected;
	unsigned int count_rx_wrong_rsp;
	u64 travel_sum;
	u64 travel_min, travel_max;
};

/*
 * RPC TX descriptor
 */

struct tegra_ivc_rpc_tx_desc {
	/* linked list */
	struct list_head node;
	bool in_list;
	/* control block */
	struct tegra_ivc_channel *chan;
	struct completion rx_complete;
	struct timer_list rx_timer;
	/* request/response information */
	uint32_t seq_num;
	uint32_t response_id;
	uint32_t response_len;
	void *response;
	tegra_ivc_rpc_call_callback callback;
	void *callback_param;
	u64 sent_time;
	/* response collected by RX handler (synchronous) */
	int32_t ret_code;
};

static struct kmem_cache *tx_desc_cache;

/*
 * Power management
 */

bool tegra_ivc_rpc_channel_is_suspended(
	struct tegra_ivc_channel *chan)
{
	return chan->dev.power.is_prepared;
}
EXPORT_SYMBOL(tegra_ivc_rpc_channel_is_suspended);

int tegra_ivc_rpc_channel_pm_prepare(struct device *dev)
{
	struct tegra_ivc_channel *chan = to_tegra_ivc_channel(dev);
	struct tegra_ivc_rpc_data *rpc = chan->rpc_priv;
	int ret = 0;

	/* If RPC has any outstanding communication (tx_list is not empty),
	 * then tells not to enter power save mode.
	 */
	if (rpc) {
		if (rpc->ops && rpc->ops->pm_prepare)
			ret = rpc->ops->pm_prepare(dev);

		if (ret == 0) {
			spin_lock_bh(&rpc->tx_list_lock);
			if (!list_empty(&rpc->tx_list))
				ret = -EAGAIN;
			spin_unlock_bh(&rpc->tx_list_lock);
		}
	}

	return ret;
}
EXPORT_SYMBOL(tegra_ivc_rpc_channel_pm_prepare);

void tegra_ivc_rpc_channel_pm_complete(struct device *dev)
{
	struct tegra_ivc_channel *chan = to_tegra_ivc_channel(dev);
	struct tegra_ivc_rpc_data *rpc = chan->rpc_priv;

	if (rpc) {
		if (rpc->ops && rpc->ops->pm_complete)
			rpc->ops->pm_complete(dev);
	}
}
EXPORT_SYMBOL(tegra_ivc_rpc_channel_pm_complete);

int tegra_ivc_rpc_channel_pm_suspend(struct device *dev)
{
	struct tegra_ivc_channel *chan = to_tegra_ivc_channel(dev);
	struct tegra_ivc_rpc_data *rpc = chan->rpc_priv;
	int ret = 0;

	if (rpc) {
		if (rpc->ops && rpc->ops->pm_suspend)
			ret = rpc->ops->pm_suspend(dev);
	}

	return ret;
}
EXPORT_SYMBOL(tegra_ivc_rpc_channel_pm_suspend);

int tegra_ivc_rpc_channel_pm_resume(struct device *dev)
{
	struct tegra_ivc_channel *chan = to_tegra_ivc_channel(dev);
	struct tegra_ivc_rpc_data *rpc = chan->rpc_priv;
	int ret = 0;

	if (rpc) {
		if (rpc->ops && rpc->ops->pm_resume)
			ret = rpc->ops->pm_resume(dev);
	}

	return ret;
}
EXPORT_SYMBOL(tegra_ivc_rpc_channel_pm_resume);

/*
 * RX
 */

/* Software interrupt context */
static void tegra_ivc_rpc_rx_tasklet_rpc(
	struct tegra_ivc_rpc_data *rpc,
	const struct tegra_ivc_rpc_response_frame *rsp)
{
	struct tegra_ivc_rpc_tx_desc *tx_desc;
	int found = false;

	/* Find TX descriptor with matching sequence number */
	spin_lock(&rpc->tx_list_lock);
	list_for_each_entry(tx_desc, &rpc->tx_list, node) {
		if (rsp->hdr.seq_num == tx_desc->seq_num) {
			list_del(&tx_desc->node);
			tx_desc->in_list = false;
			found = true;
			break;
		}
	}
	/* In asynchronous mode, response is received here. Cancel the timer
	 * so the timeout handler is not to be called. Even if it entered the
	 * timer handler, it won't find the descriptor.
	 */
	if (found && tx_desc->callback != NULL)
		del_timer(&tx_desc->rx_timer);
	spin_unlock(&rpc->tx_list_lock);

	/* If the sequence number is not found, the message is treated as
	 * an unexpected response message.
	 */
	if (unlikely(!found)) {
		if (rpc->ops && rpc->ops->unexpected_response)
			rpc->ops->unexpected_response(rpc->chan, rsp);
		++rpc->count_rx_unexpected;
		return;
	}

	/* The response buffer is freed in the tasklet. Before exiting
	 * the tasklet, collect the information about the response.
	 */
	tx_desc->ret_code = rsp->hdr.ret_code;
	if (rsp->hdr.response_id == tx_desc->response_id) {
		u64 travel_time = local_clock() - tx_desc->sent_time;

		if (rpc->count_rx_good == 0) {
			rpc->travel_sum = travel_time;
			rpc->travel_min = travel_time;
			rpc->travel_max = travel_time;
		} else {
			rpc->travel_sum += travel_time;
			if (travel_time < rpc->travel_min)
				rpc->travel_min = travel_time;
			if (travel_time > rpc->travel_max)
				rpc->travel_max = travel_time;
		}
		++rpc->count_rx_good;

		if (tx_desc->ret_code >= 0 && tx_desc->response_len > 0) {
			if (rsp->hdr.response_id ==
			    TEGRA_IVC_RPC_RSP_RET_CODE) {
				if (tx_desc->response_len == 4)
					*(uint32_t *) tx_desc->response =
						rsp->hdr.ret_data;
				else
					tx_desc->ret_code =
						TEGRA_IVC_RPC_ERR_PARAM;
			} else if (tx_desc->response_len !=
			    rsp->hdr.response_len) {
				tx_desc->ret_code = TEGRA_IVC_RPC_ERR_PARAM;
			} else {
				memcpy(tx_desc->response, rsp->payload8,
					tx_desc->response_len);
			}
		}
	} else {
		++rpc->count_rx_wrong_rsp;
		if (tx_desc->ret_code >= 0)
			tx_desc->ret_code = TEGRA_IVC_RPC_ERR_WRONG_RSP;
	}

	/* In synchronous mode, tegra_ivc_rpc_call() is waiting for the
	 * completion. It gets woken up.
	 */
	if (tx_desc->callback == NULL) {
		complete(&tx_desc->rx_complete);
		return;
	}

	/* In asynchronous mode, no one is waiting for the response.
	 * The registered callback function is invoked here.
	 */
	trace_rpc_callback(tx_desc->seq_num);
	tx_desc->callback(tx_desc->ret_code, rsp,
		tx_desc->callback_param);

	kmem_cache_free(tx_desc_cache, tx_desc);
}

static void tegra_ivc_rpc_rx_tasklet(unsigned long data)
{
	struct tegra_ivc_channel *chan = (struct tegra_ivc_channel *) data;
	struct tegra_ivc_rpc_data *rpc = chan->rpc_priv;

	while (tegra_ivc_can_read(&chan->ivc)) {
		const struct tegra_ivc_rpc_response_frame *rsp =
			tegra_ivc_read_get_next_frame(&chan->ivc);

		if (rsp->hdr.rpc_rsp_sign == TEGRA_IVC_RPC_RSP_SIGN) {
			/* An RPC type of message */
			tegra_ivc_rpc_rx_tasklet_rpc(rpc, rsp);
		} else {
			/* Not an RPC type of message */
			++rpc->count_rx_non_rpc;
			if (rpc->ops && rpc->ops->non_rpc_msg)
				rpc->ops->non_rpc_msg(rpc->chan, rsp);
		}

		tegra_ivc_read_advance(&chan->ivc);
	}

	wake_up_all(&rpc->ivc_wq);
}

/* Hardware interrupt context */
void tegra_ivc_rpc_channel_notify(struct tegra_ivc_channel *chan)
{
	struct tegra_ivc_rpc_data *rpc = chan->rpc_priv;

	/* This notify handler is called on full interrupts
	 * of the underlying shared mailbox.
	 */
	tasklet_schedule(&rpc->rx_tasklet);
}
EXPORT_SYMBOL(tegra_ivc_rpc_channel_notify);

/*
 * RPC APIs
 */

static void tegra_ivc_rpc_timer(unsigned long arg)
{
	struct tegra_ivc_rpc_tx_desc *tx_desc =
		(struct tegra_ivc_rpc_tx_desc *) arg;
	struct tegra_ivc_channel *chan = tx_desc->chan;
	struct tegra_ivc_rpc_data *rpc = chan->rpc_priv;
	bool found = false;

	/* Find the TX descriptor in the TX list */
	spin_lock(&rpc->tx_list_lock);
	if (tx_desc->in_list) {
		list_del(&tx_desc->node);
		tx_desc->in_list = false;
		found = true;
	}
	spin_unlock(&rpc->tx_list_lock);

	if (!found)
		return;

	trace_rpc_timer(tx_desc->seq_num);
	/* Callback function is called with error code */
	tx_desc->callback(TEGRA_IVC_RPC_ERR_TIMEOUT, NULL,
		tx_desc->callback_param);

	/* All is done. Free the descriptor */
	kmem_cache_free(tx_desc_cache, tx_desc);
}

int tegra_ivc_rpc_call(
	struct tegra_ivc_channel *chan,
	const struct tegra_ivc_rpc_call_param *param)
{
	struct tegra_ivc_rpc_data *rpc = chan->rpc_priv;
	uint32_t seq_num;
	struct tegra_ivc_rpc_tx_desc *tx_desc;
	unsigned int timeout_jiffies;
	int ret;
	struct tegra_ivc_rpc_request_frame *req;

	if (rpc == NULL)
		return TEGRA_IVC_RPC_ERR_PARAM;

	/* Allocate a TX descriptor */
	tx_desc = kmem_cache_alloc(tx_desc_cache, GFP_KERNEL);
	if (tx_desc == NULL)
		return TEGRA_IVC_RPC_ERR_MEMORY;

#if ENABLE_IVC_RPC_TRACE
    /*
	 * This function is called before system boots and
	 * tracing can be turned on. So, turning on it here
	 * for tracing at boot. Make sure to update bootargs
	 * in DT file and add trace_event=tegra_rpc:*
	 */
	tracing_on();
#endif

	/* Fill TX descriptor */
	seq_num = atomic_add_return(1, &rpc->next_seq_num);
	++rpc->count_tx;

	timeout_jiffies = param->timeout_ms;
	if (timeout_jiffies == 0)
		timeout_jiffies = TEGRA_IVC_RPC_TIMEOUT_MS_DEFAULT;
	timeout_jiffies = msecs_to_jiffies(timeout_jiffies);

	tx_desc->seq_num = seq_num;
	tx_desc->chan = chan;
	tx_desc->in_list = false;
	tx_desc->response_id = param->response_id;
	tx_desc->response_len = param->response_len;
	tx_desc->response = param->response;
	tx_desc->callback = param->callback;
	tx_desc->callback_param = param->callback_param;
	if (tx_desc->callback == NULL)
		init_completion(&tx_desc->rx_complete);
	tx_desc->sent_time = local_clock();

	/* Get a pointer to write buffer */
	for (;;) {
		/* Wait until IVC channel gets writable */
		ret = wait_event_interruptible(rpc->ivc_wq,
			tegra_ivc_can_write(&chan->ivc));
		if (ret < 0) {
			ret = TEGRA_IVC_RPC_ERR_WRITE_FAILED;
			goto exit;
		}

		/* Attempts to get a pointer to frame buffer */
		mutex_lock(&chan->ivc_wr_lock);

		req = tegra_ivc_write_get_next_frame(&chan->ivc);
		if (!IS_ERR_OR_NULL(req))
			break;

		mutex_unlock(&chan->ivc_wr_lock);
	}

	/* Fill IVC message frame */
	req->hdr.rpc_req_sign = TEGRA_IVC_RPC_REQ_SIGN;
	req->hdr.seq_num = seq_num;
	req->hdr.flags = 0;
	req->hdr.reserved = 0;
	req->hdr.request_id = param->request_id;
	req->hdr.request_len = param->request_len;
	if (param->request_len > 0)
		memcpy(req->payload8, param->request,
			param->request_len);

	/* Add the request to outstanding TX list */
	spin_lock_bh(&rpc->tx_list_lock);
	list_add_tail(&tx_desc->node, &rpc->tx_list);
	tx_desc->in_list = true;
	spin_unlock_bh(&rpc->tx_list_lock);

	/* Creating timer before sending out msg, as to have timer
	 * running to avoid a race condition of no timer on reply
	 * back for this msg
	 */
	if (param->callback) {
		setup_timer(&tx_desc->rx_timer, tegra_ivc_rpc_timer,
			(unsigned long) tx_desc);
		mod_timer(&tx_desc->rx_timer, jiffies + timeout_jiffies);
	}

	/* Send out the request frame */
	trace_rpc_send_msg(seq_num, param->callback ? false : true);
	tegra_ivc_write_advance(&chan->ivc);

	mutex_unlock(&chan->ivc_wr_lock);

	/* In case of asynchronous mode, do not wait for response */
	if (param->callback) {
		return 0;
	}

	/* In case of synchronous mode, wait for a response with same
	 * sequence number.
	 */
	ret = wait_for_completion_timeout(&tx_desc->rx_complete,
		timeout_jiffies);
	if (ret <= 0) {
		/* In case of RX timeout, tx_desc still resides in
		 * tx_list. Remove the descriptor from the list.
		 */
		++rpc->count_rx_timeout;
		spin_lock_bh(&rpc->tx_list_lock);
		if (tx_desc->in_list) {
			list_del(&tx_desc->node);
			tx_desc->in_list = false;
		}
		spin_unlock_bh(&rpc->tx_list_lock);
		ret = TEGRA_IVC_RPC_ERR_TIMEOUT;
	} else
		ret = tx_desc->ret_code;

exit:
	kmem_cache_free(tx_desc_cache, tx_desc);

	return ret;
}
EXPORT_SYMBOL(tegra_ivc_rpc_call);

/*
 * Debugfs
 */

#define DEFINE_SEQ_FOPS(_fops_, _show_) \
	static int _fops_ ## _open(struct inode *inode, struct file *file) \
	{ \
		return single_open(file, _show_, inode->i_private); \
	} \
	static const struct file_operations _fops_ = { \
		.open = _fops_ ## _open, \
		.read = seq_read, \
		.llseek = seq_lseek, \
		.release = single_release }

static int tegra_ivc_rpc_debugfs_stats_read(
	struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *chan = file->private;
	struct tegra_ivc_rpc_data *rpc = chan->rpc_priv;

	seq_printf(file, "Requests: %u\n", rpc->count_tx);
	seq_puts(file, "Responses:\n");
	seq_printf(file, "  Good: %u\n", rpc->count_rx_good);
	seq_printf(file, "  Non-RPC: %u\n", rpc->count_rx_non_rpc);
	seq_printf(file, "  Unexpected: %u\n", rpc->count_rx_unexpected);
	seq_printf(file, "  Wrong: %u\n", rpc->count_rx_wrong_rsp);
	seq_printf(file, "Timeouts: %u\n", rpc->count_rx_timeout);
	seq_puts(file, "Travel time:\n");
	seq_printf(file, "  Average: %llu us\n",
		rpc->travel_sum / (rpc->count_rx_good * 1000));
	seq_printf(file, "  Min: %llu us\n", rpc->travel_min / 1000);
	seq_printf(file, "  Max: %llu us\n", rpc->travel_max / 1000);

	return 0;
}

DEFINE_SEQ_FOPS(tegra_ivc_rpc_debugfs_stats,
	tegra_ivc_rpc_debugfs_stats_read);

/*
 * Initialization / Cleanup
 */

void tegra_ivc_rpc_create_test_debugfs(
	struct tegra_ivc_channel *chan,
	struct dentry *debugfs_root);

int tegra_ivc_rpc_channel_probe(
	struct tegra_ivc_channel *chan, struct tegra_ivc_rpc_ops *ops)
{
	int ret = 0;
	struct tegra_ivc_rpc_data *rpc;

	BUILD_BUG_ON(sizeof(struct tegra_ivc_rpc_request_header) >
		TEGRA_IVC_RPC_MSG_HEADER_MAX);
	BUILD_BUG_ON(sizeof(struct tegra_ivc_rpc_response_header) >
		TEGRA_IVC_RPC_MSG_HEADER_MAX);

	/* RPC private data */
	chan->rpc_priv = kzalloc(sizeof(struct tegra_ivc_rpc_data),
		GFP_KERNEL);
	if (chan->rpc_priv == NULL) {
		ret = -ENOMEM;
		goto fail;
	}

	rpc = chan->rpc_priv;
	rpc->chan = chan;
	rpc->ops = ops;
	mutex_init(&rpc->lock);
	init_waitqueue_head(&rpc->ivc_wq);

	/* TX descriptor cache */
	if (tx_desc_cache == NULL) {
		tx_desc_cache = kmem_cache_create("tegra-ivc-rpc-tx-desc",
			sizeof(struct tegra_ivc_rpc_tx_desc), 0,
			SLAB_RECLAIM_ACCOUNT | SLAB_MEM_SPREAD, NULL);
		if (tx_desc_cache == NULL) {
			ret = -ENOMEM;
			goto fail;
		}
	}

	/* list of outstanding requests */
	spin_lock_init(&rpc->tx_list_lock);
	INIT_LIST_HEAD(&rpc->tx_list);

	/* deferred procedure call */
	tasklet_init(&rpc->rx_tasklet, tegra_ivc_rpc_rx_tasklet,
		(unsigned long) chan);

	/* debugfs */
	rpc->debugfs_root = debugfs_create_dir(dev_name(&chan->dev), NULL);
	if (!IS_ERR_OR_NULL(rpc->debugfs_root)) {
		debugfs_create_file("rpc-stats", S_IRUGO,
			rpc->debugfs_root, chan,
			&tegra_ivc_rpc_debugfs_stats);
		tegra_ivc_rpc_create_test_debugfs(chan, rpc->debugfs_root);
		if (rpc->ops && rpc->ops->create_debugfs)
			rpc->ops->create_debugfs(chan, rpc->debugfs_root);
	}

	return 0;

fail:
	dev_err(&chan->dev, "Failed to initialize RPC IVC channel driver: %d\n",
		ret);
	return ret;
}
EXPORT_SYMBOL(tegra_ivc_rpc_channel_probe);

int tegra_ivc_rpc_channel_remove(struct tegra_ivc_channel *chan)
{
	struct tegra_ivc_rpc_data *rpc = chan->rpc_priv;

	if (rpc) {
		debugfs_remove_recursive(rpc->debugfs_root);
		kfree(rpc);
		chan->rpc_priv = NULL;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_ivc_rpc_channel_remove);

void tegra_ivc_rpc_channel_ready(struct tegra_ivc_channel *chan,
				bool ready)
{
	struct tegra_ivc_rpc_data *rpc = chan->rpc_priv;

	if (rpc->ops && rpc->ops->ready)
		rpc->ops->ready(chan, ready);
}
EXPORT_SYMBOL(tegra_ivc_rpc_channel_ready);
