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

#include <soc/tegra/tegra-ivc-rpc.h>

#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/sched.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <linux/sched/signal.h>
#include <linux/sched/clock.h>
#endif

/*
 * RPC Self test
 */

static void tegra_ivc_rpc_self_test_print(struct seq_file *file,
	const char *name, int passed)
{
	seq_printf(file, "  %s: %s\n", name,
		passed ? "Success" : "Failure");
}

static int tegra_ivc_rpc_self_test_sync(struct tegra_ivc_channel *chan,
	struct seq_file *file)
{
	int ret;
	int passed;
	uint32_t ret_data;
	uint32_t rsp_msg;
	int fail_cnt = 0;

	seq_puts(file, "Synchronous RPC calls\n");

	ret = tegra_ivc_rpc_call_pl(chan,
		TEGRA_IVC_RPC_REQ_TEST_NODATA_ACK, 0, NULL,
		TEGRA_IVC_RPC_RSP_RET_CODE, sizeof(ret_data), &ret_data,
		NULL, NULL, 0);
	passed = (ret == 0 && ret_data == TEGRA_IVC_RPC_REQ_SIGN);
	tegra_ivc_rpc_self_test_print(file, "NoData Ack", passed);
	if (!passed)
		++fail_cnt;

	ret = tegra_ivc_rpc_call_pl(chan,
		TEGRA_IVC_RPC_REQ_TEST_NODATA_NACK, 0, NULL,
		TEGRA_IVC_RPC_RSP_RET_CODE, sizeof(ret_data), &ret_data,
		NULL, NULL, 0);
	passed = (ret == TEGRA_IVC_RPC_ERR_TIMEOUT);
	tegra_ivc_rpc_self_test_print(file, "NoData Nack", passed);
	if (!passed)
		++fail_cnt;

	ret = tegra_ivc_rpc_call_pl(chan,
		TEGRA_IVC_RPC_REQ_TEST_DATA_ACK, 0, NULL,
		TEGRA_IVC_RPC_RSP_TEST_DATA_ACK, sizeof(rsp_msg), &rsp_msg,
		NULL, NULL, 0);
	passed = (ret == 0 && rsp_msg == TEGRA_IVC_RPC_REQ_SIGN);
	tegra_ivc_rpc_self_test_print(file, "Data Ack", passed);
	if (!passed)
		++fail_cnt;

	ret = tegra_ivc_rpc_call_pl(chan,
		TEGRA_IVC_RPC_REQ_TEST_DATA_NACK, 0, NULL,
		TEGRA_IVC_RPC_RSP_TEST_DATA_ACK, sizeof(rsp_msg), &rsp_msg,
		NULL, NULL, 0);
	passed = (ret == TEGRA_IVC_RPC_ERR_TIMEOUT);
	tegra_ivc_rpc_self_test_print(file, "Data Nack", passed);
	if (!passed)
		++fail_cnt;

	return fail_cnt;
}

struct self_test_callback_param {
	unsigned int *result;
	uint32_t request_id;
	wait_queue_head_t *wq;
};

static void tegra_ivc_rpc_self_test_callback(
	int ret,
	const struct tegra_ivc_rpc_response_frame *rsp,
	void *param)
{
	struct self_test_callback_param *cb_param = param;

	switch (cb_param->request_id) {
	case 0:
		if (ret == 0 && rsp->hdr.ret_code == 0 &&
		    rsp->hdr.response_id == TEGRA_IVC_RPC_RSP_RET_CODE &&
		    rsp->hdr.ret_data == TEGRA_IVC_RPC_REQ_SIGN)
			++*cb_param->result;
		else
			--*cb_param->result;
		break;
	case 2:
		if (ret == 0 && rsp->hdr.ret_code == 0 &&
		    rsp->hdr.response_id == TEGRA_IVC_RPC_RSP_TEST_DATA_ACK &&
		    rsp->payload32[0] == TEGRA_IVC_RPC_REQ_SIGN)
			++*cb_param->result;
		else
			--*cb_param->result;
		break;

	case 1:
	case 3:
		if (ret == TEGRA_IVC_RPC_ERR_TIMEOUT)
			++*cb_param->result;
		else
			--*cb_param->result;
		break;
	}

	wake_up_interruptible(cb_param->wq);
}

static int tegra_ivc_rpc_self_test_async(struct tegra_ivc_channel *chan,
	struct seq_file *file)
{
	static const char *s_case_str[4] = {
		"NoData Ack",
		"NoData Nack",
		"Data Ack",
		"Data Nack",
	};

	int callback_cnt[4] = { 0, 0, 0, 0 };
	int i, ret;
	uint32_t rsp_msg;
	int fail_cnt = 0;
	wait_queue_head_t wq;
	struct self_test_callback_param cb_param;

	init_waitqueue_head(&wq);
	cb_param.wq = &wq;

	seq_puts(file, "Asynchronous RPC calls\n");

	for (i = 0; i < 4; ++i) {
		int count;

		seq_printf(file, "  %s: ", s_case_str[i]);
		count = callback_cnt[i];
		cb_param.request_id = i;
		cb_param.result = callback_cnt + i;

		switch (i) {
		case 0:
			ret = tegra_ivc_rpc_call_pl(chan,
				TEGRA_IVC_RPC_REQ_TEST_NODATA_ACK,
				0, NULL,
				TEGRA_IVC_RPC_RSP_RET_CODE,
				0, NULL,
				tegra_ivc_rpc_self_test_callback,
				&cb_param, 0);
			break;
		case 1:
			ret = tegra_ivc_rpc_call_pl(chan,
				TEGRA_IVC_RPC_REQ_TEST_NODATA_NACK,
				0, NULL,
				TEGRA_IVC_RPC_RSP_RET_CODE,
				0, NULL,
				tegra_ivc_rpc_self_test_callback,
				&cb_param, 0);
			break;
		case 2:
			ret = tegra_ivc_rpc_call_pl(chan,
				TEGRA_IVC_RPC_REQ_TEST_DATA_ACK,
				0, NULL,
				TEGRA_IVC_RPC_RSP_TEST_DATA_ACK,
				sizeof(rsp_msg), &rsp_msg,
				tegra_ivc_rpc_self_test_callback,
				&cb_param, 0);
			break;
		case 3:
			ret = tegra_ivc_rpc_call_pl(chan,
				TEGRA_IVC_RPC_REQ_TEST_DATA_NACK,
				0, NULL,
				TEGRA_IVC_RPC_RSP_TEST_DATA_ACK,
				sizeof(rsp_msg), &rsp_msg,
				tegra_ivc_rpc_self_test_callback,
				&cb_param, 0);
			break;
		default:
			ret = TEGRA_IVC_RPC_ERR_RSP_UNKNOWN_REQ;
			break;
		}

		if (ret < 0) {
			seq_puts(file, "Failed to send\n");
			++fail_cnt;
			continue;
		}

		ret = wait_event_interruptible_timeout(wq,
			callback_cnt[i] != count,
			msecs_to_jiffies(100));
		if (ret <= 0) {
			seq_puts(file, "No callback\n");
			++fail_cnt;
			continue;
		}

		if (callback_cnt[i] <= count)
			++fail_cnt;

		seq_printf(file, "%s\n",
			(callback_cnt[i] > count) ?
			"Success" : "Failure");
	}

	return fail_cnt;
}

/*
 * RPC Performance test
 */

struct perf_test_callback_param {
	unsigned int *result;
	wait_queue_head_t *wq;
};

static void tegra_ivc_rpc_perf_test_callback(
	int ret,
	const struct tegra_ivc_rpc_response_frame *rsp,
	void *param)
{
	struct perf_test_callback_param *cb_param = param;

	*cb_param->result = 1;
	wake_up_interruptible(cb_param->wq);
}

static void tegra_ivc_rpc_perf_test(struct tegra_ivc_channel *chan,
	struct seq_file *file, int async)
{
	const int perf_loop = 128;

	u64 time_all = 0, time_min = (u64) -1, time_max = 0;
	unsigned int result;
	wait_queue_head_t wq;
	struct perf_test_callback_param cb_param;
	int i;

	seq_printf(file, "%s RPC calls\n",
		async ? "Asynchronous" : "Synchronous");

	init_waitqueue_head(&wq);
	cb_param.result = &result;
	cb_param.wq = &wq;

	for (i = 0; i < perf_loop; ++i) {
		int ret;
		int passed;
		uint32_t ret_data;
		u64 tstamp_req, tstamp_rsp, diff;

		ret_data = 0;
		result = 0;

		tstamp_req = sched_clock();
		ret = tegra_ivc_rpc_call_pl(chan,
			TEGRA_IVC_RPC_REQ_TEST_NODATA_ACK, 0, NULL,
			TEGRA_IVC_RPC_RSP_RET_CODE, sizeof(ret_data),
			(void *) &ret_data,
			async ? tegra_ivc_rpc_perf_test_callback : NULL,
			&cb_param, 0);

		if (async) {
			wait_event_interruptible_timeout(wq,
			    result != 0,
			    msecs_to_jiffies(100));
		}

		tstamp_rsp = sched_clock();

		passed = (ret == 0 && ret_data == TEGRA_IVC_RPC_REQ_SIGN);
		if (!passed) {
			seq_printf(file, "Test failed at %d: %d\n", i, ret);
			return;
		}

		diff = tstamp_rsp - tstamp_req;
		time_all += diff;
		if (diff < time_min)
			time_min = diff;
		if (diff > time_max)
			time_max = diff;
	}

	seq_printf(file, "  Average: %llu us\n"
		"  Minimum: %llu us\n"
		"  Maximum: %llu us\n",
		(time_all / perf_loop) / 1000,
		time_min / 1000, time_max / 1000);
}

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

static int tegra_ivc_rpc_debugfs_selftest_read(
	struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *chan = file->private;

	tegra_ivc_rpc_self_test_sync(chan, file);
	tegra_ivc_rpc_self_test_async(chan, file);

	return 0;
}

DEFINE_SEQ_FOPS(tegra_ivc_rpc_debugfs_selftest,
	tegra_ivc_rpc_debugfs_selftest_read);

static int tegra_ivc_rpc_debugfs_perftest_read(
	struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *chan = file->private;

	tegra_ivc_rpc_perf_test(chan, file, false);
	tegra_ivc_rpc_perf_test(chan, file, true);

	return 0;
}

DEFINE_SEQ_FOPS(tegra_ivc_rpc_debugfs_perftest,
	tegra_ivc_rpc_debugfs_perftest_read);

void tegra_ivc_rpc_create_test_debugfs(
	struct tegra_ivc_channel *chan,
	struct dentry *debugfs_root)
{
	debugfs_create_file("rpc-selftest", S_IRUGO,
		debugfs_root, chan,
		&tegra_ivc_rpc_debugfs_selftest);
	debugfs_create_file("rpc-perftest", S_IRUGO,
		debugfs_root, chan,
		&tegra_ivc_rpc_debugfs_perftest);
}
EXPORT_SYMBOL(tegra_ivc_rpc_create_test_debugfs);
