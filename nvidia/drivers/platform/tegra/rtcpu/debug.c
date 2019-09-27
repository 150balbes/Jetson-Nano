/*
 * Copyright (c) 2016-2019, NVIDIA CORPORATION. All rights reserved.
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
 * You should have eeceived a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "soc/tegra/camrtc-dbg-messages.h"
#include "soc/tegra/camrtc-commands.h"

#include <linux/debugfs.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <linux/sched/signal.h>
#include <linux/sched/clock.h>
#endif
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/tegra-camera-rtcpu.h>
#include <linux/tegra-ivc.h>
#include <linux/tegra-ivc-bus.h>
#include <linux/platform/tegra/emc_bwmgr.h>
#include <linux/platform/tegra/tegra-mc-sid.h>

struct camrtc_debug {
	struct tegra_ivc_channel *channel;
	struct mutex mutex;
	struct dentry *root;
	wait_queue_head_t waitq;
	struct {
		u32 completion_timeout;
		u32 mods_loops;
		char *test_case;
		size_t test_case_size;
		u32 test_timeout;
		unsigned long test_bw;
	} parameters;
	struct camrtc_test_mem {
		u32 index;
		size_t used;
		size_t size;
		void *ptr;
		dma_addr_t iova;
	} mem[CAMRTC_DBG_NUM_MEM_TEST_MEM];
	struct device *mem_devices[3];
	struct camrtc_dbg_streamids streamids;
	struct ast_regset {
		struct debugfs_regset32 common, region[8];
	} ast_regsets[2];
};

#define NV(x) "nvidia," #x

/* Get a camera-rtcpu device */
static struct device *camrtc_get_device(struct tegra_ivc_channel *ch)
{
	if (unlikely(ch == NULL))
		return NULL;

	BUG_ON(ch->dev.parent == NULL);
	BUG_ON(ch->dev.parent->parent == NULL);

	return ch->dev.parent->parent;
}

#define INIT_OPEN_FOPS(_open) { \
	.open = _open, \
	.read = seq_read, \
	.llseek = seq_lseek, \
	.release = single_release \
}

#define DEFINE_SEQ_FOPS(_fops_, _show_) \
static int _fops_ ## _open(struct inode *inode, struct file *file) \
{ \
	return single_open(file, _show_, inode->i_private); \
} \
static const struct file_operations _fops_ = INIT_OPEN_FOPS(_fops_ ## _open)

static int camrtc_show_version(struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *ch = file->private;
	struct device *rce_dev = camrtc_get_device(ch);
	char version[TEGRA_CAMRTC_VERSION_LEN];

	tegra_camrtc_print_version(rce_dev, version, sizeof(version));

	seq_puts(file, version);
	seq_puts(file, "\n");

	return 0;
}

DEFINE_SEQ_FOPS(camrtc_dbgfs_fops_version, camrtc_show_version);

static int camrtc_show_reboot(struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *ch = file->private;
	struct device *rce_dev = camrtc_get_device(ch);
	int ret = 0;

	/* Make rtcpu online */
	ret = tegra_ivc_channel_runtime_get(ch);
	if (ret < 0)
		goto error;

	ret = tegra_camrtc_reboot(rce_dev);
	if (ret)
		goto error;

	seq_puts(file, "0\n");

error:
	tegra_ivc_channel_runtime_put(ch);
	return ret;
}

DEFINE_SEQ_FOPS(camrtc_dbgfs_fops_reboot, camrtc_show_reboot);

static void camrtc_debug_notify(struct tegra_ivc_channel *ch)
{
	struct camrtc_debug *crd = tegra_ivc_channel_get_drvdata(ch);

	wake_up_all(&crd->waitq);
}

static int camrtc_show_forced_reset_restore(struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *ch = file->private;
	struct device *rce_dev = camrtc_get_device(ch);
	int ret = 0;

	/* Make rtcpu online */
	ret = tegra_ivc_channel_runtime_get(ch);
	if (ret < 0)
		goto error;

	ret = tegra_camrtc_restore(rce_dev);
	if (ret)
		goto error;

	seq_puts(file, "0\n");

error:
	tegra_ivc_channel_runtime_put(ch);
	return ret;
}

DEFINE_SEQ_FOPS(camrtc_dbgfs_fops_forced_reset_restore,
			camrtc_show_forced_reset_restore);

static int camrtc_ivc_dbg_full_frame_xact(
	struct tegra_ivc_channel *ch,
	struct camrtc_dbg_request *req,
	size_t req_size,
	struct camrtc_dbg_response *resp,
	size_t resp_size,
	long timeout)
{
	struct camrtc_debug *crd = tegra_ivc_channel_get_drvdata(ch);
	int ret;

	if (timeout == 0)
		timeout = crd->parameters.completion_timeout;

	timeout = msecs_to_jiffies(timeout);

	ret = mutex_lock_interruptible(&crd->mutex);
	if (ret)
		return ret;

	ret = tegra_ivc_channel_runtime_get(ch);
	if (ret < 0)
		goto unlock;

	if (WARN_ON(!tegra_ivc_channel_online_check(ch))) {
		ret = -ECONNRESET;
		goto out;
	}

	while (tegra_ivc_can_read(&ch->ivc)) {
		tegra_ivc_read_advance(&ch->ivc);
		dev_warn(&ch->dev, "stray response\n");
	}

	timeout = wait_event_interruptible_timeout(crd->waitq,
			tegra_ivc_channel_has_been_reset(ch) ||
			tegra_ivc_can_write(&ch->ivc), timeout);
	if (timeout <= 0) {
		ret = timeout ?: -ETIMEDOUT;
		goto out;
	}
	if (tegra_ivc_channel_has_been_reset(ch)) {
		ret = -ECONNRESET;
		goto out;
	}

	ret = tegra_ivc_write(&ch->ivc, req, req_size);
	if (ret < 0) {
		dev_err(&ch->dev, "IVC write error: %d\n", ret);
		goto out;
	}

	for (;;) {
		timeout = wait_event_interruptible_timeout(crd->waitq,
				tegra_ivc_channel_has_been_reset(ch) ||
				tegra_ivc_can_read(&ch->ivc),
				timeout);
		if (timeout <= 0) {
			ret = timeout ?: -ETIMEDOUT;
			break;
		}
		if (tegra_ivc_channel_has_been_reset(ch)) {
			ret = -ECONNRESET;
			break;
		}

		dev_dbg(&ch->dev, "rx msg\n");

		ret = tegra_ivc_read_peek(&ch->ivc, resp, 0, resp_size);
		if (ret < 0) {
			dev_err(&ch->dev, "IVC read error: %d\n", ret);
			break;
		}

		tegra_ivc_read_advance(&ch->ivc);

		if (resp->resp_type == req->req_type) {
			ret = 0;
			break;
		}

		dev_err(&ch->dev, "unexpected response\n");
	}

out:
	tegra_ivc_channel_runtime_put(ch);
unlock:
	mutex_unlock(&crd->mutex);
	return ret;
}

static inline int camrtc_ivc_dbg_xact(
	struct tegra_ivc_channel *ch,
	struct camrtc_dbg_request *req,
	struct camrtc_dbg_response *resp,
	long timeout)
{
	return camrtc_ivc_dbg_full_frame_xact(ch, req, sizeof(*req),
					resp, sizeof(*resp),
					timeout);
}

static int camrtc_show_ping(struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *ch = file->private;
	struct camrtc_dbg_request req = {
		.req_type = CAMRTC_REQ_PING,
	};
	struct camrtc_dbg_response resp;
	u64 sent, recv, tsc;
	int ret;

	sent = sched_clock();
	req.data.ping_data.ts_req = sent;

	ret = camrtc_ivc_dbg_xact(ch, &req, &resp, 0);
	if (ret)
		return ret;

	recv = sched_clock();
	tsc = resp.data.ping_data.ts_resp;
	seq_printf(file,
		"roundtrip=%llu.%03llu us "
		"(sent=%llu.%09llu recv=%llu.%09llu)\n",
		(recv - sent) / 1000, (recv - sent) % 1000,
		sent / 1000000000, sent % 1000000000,
		recv / 1000000000, recv % 1000000000);
	seq_printf(file,
		"rtcpu tsc=%llu.%09llu offset=%llu.%09llu\n",
		tsc / (1000000000 / 32), tsc % (1000000000 / 32),
		(tsc * 32ULL - sent) / 1000000000,
		(tsc * 32ULL - sent) % 1000000000);
	seq_printf(file, "%.*s\n",
		(int)sizeof(resp.data.ping_data.data),
		(char *)resp.data.ping_data.data);

	return 0;
}

DEFINE_SEQ_FOPS(camrtc_dbgfs_fops_ping, camrtc_show_ping);

static int camrtc_show_sm_ping(struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *ch = file->private;
	struct device *camrtc = camrtc_get_device(ch);
	u64 sent, recv;
	u32 command;
	int err;

	err = tegra_ivc_channel_runtime_get(ch);
	if (err < 0)
		return err;

	sent = sched_clock();

	command = RTCPU_COMMAND(PING, sent & 0xffffff);

	err = tegra_camrtc_command(camrtc, command, 0);
	if (err < 0)
		goto error;

	recv = sched_clock();
	err = 0;

	seq_printf(file,
		"roundtrip=%llu.%03llu us "
		"(sent=%llu.%09llu recv=%llu.%09llu)\n",
		(recv - sent) / 1000, (recv - sent) % 1000,
		sent / 1000000000, sent % 1000000000,
		recv / 1000000000, recv % 1000000000);

error:
	tegra_ivc_channel_runtime_put(ch);

	return err;
}

DEFINE_SEQ_FOPS(camrtc_dbgfs_fops_sm_ping, camrtc_show_sm_ping);

static int camrtc_dbgfs_show_loglevel(void *data, u64 *val)
{
	struct tegra_ivc_channel *ch = data;
	struct camrtc_dbg_request req = {
		.req_type = CAMRTC_REQ_GET_LOGLEVEL,
	};
	struct camrtc_dbg_response resp;
	int ret;

	ret = camrtc_ivc_dbg_xact(ch, &req, &resp, 0);
	if (ret)
		return ret;

	if (resp.status != CAMRTC_STATUS_OK)
		return -EPROTO;

	*val = resp.data.log_data.level;

	return 0;
}

static int camrtc_dbgfs_store_loglevel(void *data, u64 val)
{
	struct tegra_ivc_channel *ch = data;
	struct camrtc_dbg_request req = {
		.req_type = CAMRTC_REQ_SET_LOGLEVEL,
	};
	struct camrtc_dbg_response resp;
	int ret;

	if ((u32)val != val)
		return -EINVAL;

	req.data.log_data.level = val;

	ret = camrtc_ivc_dbg_xact(ch, &req, &resp, 0);
	if (ret)
		return ret;

	if (resp.status == CAMRTC_STATUS_INVALID_PARAM)
		return -EINVAL;
	else if (resp.status != CAMRTC_STATUS_OK)
		return -EPROTO;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(camrtc_dbgfs_fops_loglevel,
			camrtc_dbgfs_show_loglevel,
			camrtc_dbgfs_store_loglevel,
			"%lld\n");

static int camrtc_show_mods_result(struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *ch = file->private;
	struct camrtc_debug *crd = tegra_ivc_channel_get_drvdata(ch);
	struct camrtc_dbg_request req = {
		.req_type = CAMRTC_REQ_MODS_TEST,
	};
	struct camrtc_dbg_response resp;
	int ret;
	unsigned long timeout = crd->parameters.completion_timeout;
	u32 loops = crd->parameters.mods_loops;

	req.data.mods_data.mods_loops = loops;

	ret = camrtc_ivc_dbg_xact(ch, &req, &resp, loops * timeout);
	if (ret == 0)
		seq_printf(file, "mods=%u\n", resp.status);

	return ret;
}

DEFINE_SEQ_FOPS(camrtc_dbgfs_fops_mods_result, camrtc_show_mods_result);

static int camrtc_dbgfs_show_freertos_state(struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *ch = file->private;
	struct camrtc_dbg_request req = {
		.req_type = CAMRTC_REQ_RTOS_STATE,
	};
	struct camrtc_dbg_response resp;
	int ret = 0;

	ret = camrtc_ivc_dbg_xact(ch, &req, &resp, 0);
	if (ret == 0) {
		seq_printf(file, "%.*s",
			(int) sizeof(resp.data.rtos_state_data.rtos_state),
			resp.data.rtos_state_data.rtos_state);
	}

	return ret;
}

DEFINE_SEQ_FOPS(camrtc_dbgfs_fops_freertos_state,
		camrtc_dbgfs_show_freertos_state);

static size_t camrtc_dbgfs_get_max_test_size(
	const struct tegra_ivc_channel *ch)
{
	return ch->ivc.frame_size - offsetof(struct camrtc_dbg_request,
					data.run_mem_test_data.data);
}

static ssize_t camrtc_dbgfs_read_test_case(struct file *file,
		char __user *buf, size_t count, loff_t *f_pos)
{
	struct tegra_ivc_channel *ch = file->f_inode->i_private;
	struct camrtc_debug *crd = tegra_ivc_channel_get_drvdata(ch);

	return simple_read_from_buffer(buf, count, f_pos,
				crd->parameters.test_case,
				crd->parameters.test_case_size);
}

static ssize_t camrtc_dbgfs_write_test_case(struct file *file,
		const char __user *buf, size_t count, loff_t *f_pos)
{
	struct tegra_ivc_channel *ch = file->f_inode->i_private;
	struct camrtc_debug *crd = tegra_ivc_channel_get_drvdata(ch);
	char *test_case = crd->parameters.test_case;
	size_t max_size = camrtc_dbgfs_get_max_test_size(ch);
	int i;
	ssize_t ret;

	ret = simple_write_to_buffer(test_case, max_size, f_pos, buf, count);

	if (ret >= 0)
		crd->parameters.test_case_size = *f_pos;

	/* Mark input buffers empty */
	for (i = 0; i < ARRAY_SIZE(crd->mem); i++)
		crd->mem[i].used = 0;

	return ret;
}

static const struct file_operations camrtc_dbgfs_fops_test_case = {
	.read = camrtc_dbgfs_read_test_case,
	.write = camrtc_dbgfs_write_test_case,
};

static ssize_t camrtc_dbgfs_read_test_mem(struct file *file,
		char __user *buf, size_t count, loff_t *f_pos)
{
	struct camrtc_test_mem *mem = file->f_inode->i_private;

	return simple_read_from_buffer(buf, count, f_pos, mem->ptr, mem->used);
}

static ssize_t camrtc_dbgfs_write_test_mem(struct file *file,
		const char __user *buf, size_t count, loff_t *f_pos)
{
	struct camrtc_test_mem *mem = file->f_inode->i_private;
	struct camrtc_debug *crd = container_of(
		mem, struct camrtc_debug, mem[mem->index]);
	struct device *dev = crd->mem_devices[0];
	ssize_t ret;

	if (*f_pos + count > mem->size) {
		size_t size = round_up(*f_pos + count, 64 * 1024);
		dma_addr_t iova;
		void *ptr = dma_alloc_coherent(dev, size, &iova,
					GFP_KERNEL | __GFP_ZERO);
		if (ptr == NULL)
			return -ENOMEM;
		if (mem->ptr) {
			memcpy(ptr, mem->ptr, mem->used);
			dma_free_coherent(dev, mem->size, mem->ptr,
					mem->iova);
		}
		mem->ptr = ptr;
		mem->size = size;
		mem->iova = iova;
	}

	ret = simple_write_to_buffer(mem->ptr, mem->size, f_pos, buf, count);

	if (ret >= 0) {
		mem->used = *f_pos;

		if (mem->used == 0 && mem->ptr != NULL) {
			dma_free_coherent(dev, mem->size, mem->ptr,
					mem->iova);
			memset(mem, 0, sizeof(*mem));
		}
	}

	return ret;
}

static const struct file_operations camrtc_dbgfs_fops_test_mem = {
	.read = camrtc_dbgfs_read_test_mem,
	.write = camrtc_dbgfs_write_test_mem,
};

#define BUILD_BUG_ON_MISMATCH(s1, f1, s2, f2) \
	BUILD_BUG_ON(offsetof(s1, data.f1) != offsetof(s2, data.f2))

static int camrtc_test_run_and_show_result(struct seq_file *file,
				struct camrtc_dbg_request *req,
				struct camrtc_dbg_response *resp,
				size_t data_offset)
{
	struct tegra_ivc_channel *ch = file->private;
	struct camrtc_debug *crd = tegra_ivc_channel_get_drvdata(ch);
	const char *test_case = crd->parameters.test_case;
	size_t test_case_size = crd->parameters.test_case_size;
	unsigned long timeout = crd->parameters.test_timeout;
	uint64_t ns;
	size_t req_size = ch->ivc.frame_size;
	size_t resp_size = ch->ivc.frame_size;
	int ret;
	const char *result = (const void *)resp + data_offset;
	size_t result_size = resp_size - data_offset;
	const char *nul;

	if (WARN_ON(test_case_size > camrtc_dbgfs_get_max_test_size(ch)))
		test_case_size = camrtc_dbgfs_get_max_test_size(ch);

	memcpy((char *)req + data_offset, test_case, test_case_size);

	/* Timeout is in ms, run_test_data.timeout in ns */
	if (timeout > 40)
		ns = 1000000ULL * (timeout - 20);
	else
		ns = 1000000ULL * (timeout / 2);

	BUILD_BUG_ON_MISMATCH(
		struct camrtc_dbg_request, run_mem_test_data.timeout,
		struct camrtc_dbg_request, run_test_data.timeout);

	ret = tegra_ivc_channel_runtime_get(ch);
	if (ret < 0)
		return ret;

	req->data.run_test_data.timeout = ns;

	ret = camrtc_ivc_dbg_full_frame_xact(ch, req, req_size,
					resp, resp_size, timeout);

	tegra_camrtc_flush_trace(camrtc_get_device(ch));

	if (ret < 0) {
		if (ret != -ECONNRESET) {
			dev_info(&ch->dev, "rebooting after a failed test run");
			(void)tegra_camrtc_reboot(camrtc_get_device(ch));
		}
		goto runtime_put;
	}

	BUILD_BUG_ON_MISMATCH(
		struct camrtc_dbg_response, run_mem_test_data.timeout,
		struct camrtc_dbg_response, run_test_data.timeout);

	ns = resp->data.run_test_data.timeout;

	seq_printf(file, "result=%u runtime=%llu.%06llu ms\n\n",
		resp->status, ns / 1000000, ns % 1000000);

	nul = memchr(result, '\0', result_size);
	if (nul)
		seq_write(file, result, nul - result);
	else
		seq_write(file, result, result_size);

runtime_put:
	tegra_ivc_channel_runtime_put(ch);

	return ret;
}

static int camrtc_run_mem_map(struct tegra_ivc_channel *ch,
			struct device *dev,
			struct sg_table *sgt,
			struct camrtc_test_mem *mem)
{
	int ret;

	if (dev == NULL)
		return 0;

	ret = dma_get_sgtable(dev, sgt, mem->ptr, mem->iova, mem->size);
	if (ret < 0) {
		dev_err(&ch->dev, "dma_get_sgtable for %s failed\n",
			dev_name(dev));
		return ret;
	}

	if (!dma_map_sg(dev, sgt->sgl, sgt->orig_nents, DMA_BIDIRECTIONAL)) {
		dev_err(&ch->dev, "failed to map %s mem at 0x%llx\n",
			dev_name(dev), (u64)mem->iova);
		sg_free_table(sgt);
		ret = -ENXIO;
	}

	return ret;
}

static int camrtc_run_mem_test(struct seq_file *file,
			struct camrtc_dbg_request *req,
			struct camrtc_dbg_response *resp)
{
	struct tegra_ivc_channel *ch = file->private;
	struct camrtc_debug *crd = tegra_ivc_channel_get_drvdata(ch);
	struct camrtc_dbg_test_mem *testmem;
	size_t i;
	int ret;
	struct device *dev = crd->mem_devices[0];
	struct device *vi_dev = crd->mem_devices[1];
	struct sg_table vi_sgt[ARRAY_SIZE(crd->mem)];
	struct device *isp_dev = crd->mem_devices[2];
	struct sg_table isp_sgt[ARRAY_SIZE(crd->mem)];
	struct camrtc_test_mem *mem0 = &crd->mem[0];
	struct tegra_bwmgr_client *bwmgr = NULL;

	memset(vi_sgt, 0, sizeof(vi_sgt));
	memset(isp_sgt, 0, sizeof(isp_sgt));

	req->req_type = CAMRTC_REQ_RUN_MEM_TEST;
	req->data.run_mem_test_data.streamids = crd->streamids;

	/* Allocate 6MB scratch memory in mem0 by default */
	if (!mem0->used) {
		const size_t size = 6U << 20U; /* 6 MB */
		dma_addr_t iova;
		void *ptr;

		if (mem0->ptr) {
			dma_free_coherent(dev, mem0->size, mem0->ptr,
					mem0->iova);
			memset(mem0, 0, sizeof(*mem0));
		}
		ptr = dma_alloc_coherent(dev, size, &iova,
					GFP_KERNEL | __GFP_ZERO);
		if (ptr == NULL)
			return -ENOMEM;
		mem0->ptr = ptr;
		mem0->size = size;
		mem0->iova = iova;
		mem0->used = size;
	}

	for (i = 0; i < ARRAY_SIZE(crd->mem); i++) {
		struct camrtc_test_mem *mem = &crd->mem[i];

		if (mem->used == 0)
			continue;

		testmem = &req->data.run_mem_test_data.mem[i];

		testmem->size = mem->used;
		testmem->rtcpu_iova = mem->iova;

		ret = camrtc_run_mem_map(ch, vi_dev, &vi_sgt[i], mem);
		if (ret < 0)
			goto unmap;

		if (vi_sgt[i].sgl)
			testmem->vi_iova = vi_sgt[i].sgl->dma_address;

		ret = camrtc_run_mem_map(ch, isp_dev, &isp_sgt[i], mem);
		if (ret < 0)
			goto unmap;

		if (isp_sgt[i].sgl)
			testmem->isp_iova = isp_sgt[i].sgl->dma_address;

		dma_sync_single_for_device(dev, mem->iova, mem->used,
						DMA_TO_DEVICE);
	}

	if (crd->parameters.test_bw != 0)
		bwmgr = tegra_bwmgr_register(TEGRA_BWMGR_CLIENT_CAMERA_NON_ISO);

	if (!IS_ERR_OR_NULL(bwmgr)) {
		ret = tegra_bwmgr_set_emc(bwmgr, crd->parameters.test_bw,
				TEGRA_BWMGR_SET_EMC_SHARED_BW);
		if (ret < 0)
			dev_info(dev, "emc request rate %lu failed, %d\n",
				crd->parameters.test_bw, ret);
		else
			dev_dbg(dev, "requested emc rate %lu\n",
				crd->parameters.test_bw);
	}


	BUILD_BUG_ON_MISMATCH(
		struct camrtc_dbg_request, run_mem_test_data.data,
		struct camrtc_dbg_response, run_mem_test_data.data);

	ret = camrtc_test_run_and_show_result(file, req, resp,
					offsetof(struct camrtc_dbg_response,
						data.run_mem_test_data.data));
	if (ret < 0)
		goto unmap;

	for (i = 0; i < ARRAY_SIZE(crd->mem); i++) {
		struct camrtc_test_mem *mem = &crd->mem[i];

		if (mem->size == 0)
			continue;

		testmem = &resp->data.run_mem_test_data.mem[i];
		if (!WARN_ON(testmem->size > mem->size))
			mem->used = testmem->size;

		dma_sync_single_for_device(dev, mem->iova, mem->used,
					DMA_FROM_DEVICE);
	}

unmap:
	if (!IS_ERR_OR_NULL(bwmgr)) {
		tegra_bwmgr_unregister(bwmgr);
	}

	for (i = 0; i < ARRAY_SIZE(vi_sgt); i++) {
		if (vi_sgt[i].sgl) {
			dma_unmap_sg(vi_dev, vi_sgt[i].sgl,
				vi_sgt[i].orig_nents, DMA_BIDIRECTIONAL);
			sg_free_table(&vi_sgt[i]);
		}
		if (isp_sgt[i].sgl) {
			dma_unmap_sg(isp_dev, isp_sgt[i].sgl,
				isp_sgt[i].orig_nents, DMA_BIDIRECTIONAL);
			sg_free_table(&isp_sgt[i]);
		}
	}

	return ret;
}

static int camrtc_dbgfs_show_test_result(struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *ch = file->private;
	void *mem = kzalloc(2 * ch->ivc.frame_size, GFP_KERNEL | __GFP_ZERO);
	struct camrtc_dbg_request *req = mem;
	struct camrtc_dbg_response *resp = mem + ch->ivc.frame_size;
	int ret;

	if (mem == NULL)
		return -ENOMEM;

	ret = camrtc_run_mem_test(file, req, resp);
	kfree(mem);

	return ret;
}

DEFINE_SEQ_FOPS(camrtc_dbgfs_fops_test_result, camrtc_dbgfs_show_test_result);

static int camrtc_dbgfs_show_test_list(struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *ch = file->private;
	struct camrtc_dbg_request req = {
		.req_type = CAMRTC_REQ_RUN_TEST,
	};
	struct camrtc_dbg_response *resp;
	int ret;

	resp = kzalloc(ch->ivc.frame_size, GFP_KERNEL | __GFP_ZERO);
	if (resp == NULL)
		return -ENOMEM;

	memset(req.data.run_test_data.data, 0,
		sizeof(req.data.run_test_data.data));
	strcpy(req.data.run_test_data.data, "list\n");

	ret = camrtc_ivc_dbg_full_frame_xact(ch, &req, sizeof(req),
					resp, ch->ivc.frame_size, 0);
	if (ret == 0 && resp->status == CAMRTC_STATUS_OK) {
		char const *list = (char const *)resp->data.run_test_data.data;
		size_t textsize = ch->ivc.frame_size -
			offsetof(struct camrtc_dbg_response,
				data.run_test_data.data);
		size_t i;

		/* Remove first line */
		for (i = 0; i < textsize; i++)
			if (list[i] == '\n')
				break;
		for (; i < textsize; i++)
			if (list[i] != '\n' && list[i] != '\r')
				break;

		seq_printf(file, "%.*s", (int)(textsize - i), list + i);
	}

	kfree(resp);

	return ret;
}

DEFINE_SEQ_FOPS(camrtc_dbgfs_fops_test_list, camrtc_dbgfs_show_test_list);

#define TEGRA_APS_AST_CONTROL			0x0
#define TEGRA_APS_AST_STREAMID_CTL		0x20
#define TEGRA_APS_AST_REGION_0_SLAVE_BASE_LO	0x100
#define TEGRA_APS_AST_REGION_0_SLAVE_BASE_HI	0x104
#define TEGRA_APS_AST_REGION_0_MASK_LO		0x108
#define TEGRA_APS_AST_REGION_0_MASK_HI		0x10c
#define TEGRA_APS_AST_REGION_0_MASTER_BASE_LO	0x110
#define TEGRA_APS_AST_REGION_0_MASTER_BASE_HI	0x114
#define TEGRA_APS_AST_REGION_0_CONTROL		0x118

#define TEGRA_APS_AST_REGION_STRIDE		0x20

#define AST_RGN_CTRL_VM_INDEX		15
#define AST_RGN_CTRL_SNOOP		BIT(2)

#define AST_ADDR_MASK64			(~0xfffULL)

struct tegra_ast_region_info {
	u8  enabled;
	u8  lock;
	u8  snoop;
	u8  non_secure;

	u8  ns_passthru;
	u8  carveout_id;
	u8  carveout_al;
	u8  vpr_rd;

	u8  vpr_wr;
	u8  vpr_passthru;
	u8  vm_index;
	u8  physical;

	u8  stream_id;
	u8  stream_id_enabled;
	u8  pad[2];

	u64 slave;
	u64 mask;
	u64 master;
	u32 control;
};

static void tegra_ast_get_region_info(void __iomem *base,
			u32 region,
			struct tegra_ast_region_info *info)
{
	u32 offset = region * TEGRA_APS_AST_REGION_STRIDE;
	u32 vmidx, stream_id, gcontrol, control;
	u64 lo, hi;

	control = readl(base + TEGRA_APS_AST_REGION_0_CONTROL + offset);
	info->control = control;

	info->lock = (control & BIT(0)) != 0;
	info->snoop = (control & BIT(2)) != 0;
	info->non_secure = (control & BIT(3)) != 0;
	info->ns_passthru = (control & BIT(4)) != 0;
	info->carveout_id = (control >> 5) & (0x1f);
	info->carveout_al = (control >> 10) & 0x3;
	info->vpr_rd = (control & BIT(12)) != 0;
	info->vpr_wr = (control & BIT(13)) != 0;
	info->vpr_passthru = (control & BIT(14)) != 0;
	vmidx = (control >> AST_RGN_CTRL_VM_INDEX) & 0xf;
	info->vm_index = vmidx;
	info->physical = (control & BIT(19)) != 0;

	if (info->physical) {
		gcontrol = readl(base + TEGRA_APS_AST_CONTROL);
		info->stream_id = (gcontrol >> 22) & 0x7F;
		info->stream_id_enabled = 1;
	} else {
		stream_id = readl(base + TEGRA_APS_AST_STREAMID_CTL +
				(4 * vmidx));
		info->stream_id = (stream_id >> 8) & 0xFF;
		info->stream_id_enabled = (stream_id & BIT(0)) != 0;
	}

	lo = readl(base + TEGRA_APS_AST_REGION_0_SLAVE_BASE_LO + offset);
	hi = readl(base + TEGRA_APS_AST_REGION_0_SLAVE_BASE_HI + offset);

	info->slave = ((hi << 32U) + lo) & AST_ADDR_MASK64;
	info->enabled = (lo & BIT(0)) != 0;

	hi = readl(base + TEGRA_APS_AST_REGION_0_MASK_HI + offset);
	lo = readl(base + TEGRA_APS_AST_REGION_0_MASK_LO + offset);

	info->mask = ((hi << 32) + lo) | ~AST_ADDR_MASK64;

	hi = readl(base + TEGRA_APS_AST_REGION_0_MASTER_BASE_HI + offset);
	lo = readl(base + TEGRA_APS_AST_REGION_0_MASTER_BASE_LO + offset);

	info->master = ((hi << 32U) + lo) & AST_ADDR_MASK64;
}

static void __iomem *iomap_byname(struct device *dev, const char *name)
{
	int index = of_property_match_string(dev->of_node, "reg-names", name);
	if (index < 0)
		return IOMEM_ERR_PTR(-ENOENT);

	return of_iomap(dev->of_node, index);
}

static void camrtc_dbgfs_show_ast_region(struct seq_file *file,
						void __iomem *base, u32 index)
{
	struct tegra_ast_region_info info;

	tegra_ast_get_region_info(base, index, &info);

	seq_printf(file, "ast region %u %s\n", index,
		info.enabled ? "enabled" : "disabled");

	if (!info.enabled)
		return;

	seq_printf(file,
		"\tslave=0x%llx\n"
		"\tmaster=0x%llx\n"
		"\tsize=0x%llx\n"
		"\tlock=%u snoop=%u non_secure=%u ns_passthru=%u\n"
		"\tcarveout_id=%u carveout_al=%u\n"
		"\tvpr_rd=%u vpr_wr=%u vpr_passthru=%u\n"
		"\tvm_index=%u physical=%u\n"
		"\tstream_id=%u (enabled=%u)\n",
		info.slave, info.master, info.mask + 1,
		info.lock, info.snoop,
		info.non_secure, info.ns_passthru,
		info.carveout_id, info.carveout_al,
		info.vpr_rd, info.vpr_wr, info.vpr_passthru,
		info.vm_index, info.physical,
		info.stream_id, info.stream_id_enabled);
}

struct camrtc_dbgfs_ast_node
{
	struct tegra_ivc_channel *ch;
	const char *name;
	uint8_t mask;
};

static int camrtc_dbgfs_show_ast(struct seq_file *file,
				void *data)
{
	struct camrtc_dbgfs_ast_node *node = file->private;
	void __iomem *ast;
	int i;

	ast = iomap_byname(camrtc_get_device(node->ch), node->name);
	if (ast == NULL)
		return -ENOMEM;

	for (i = 0; i <= 7; i++) {
		if (!(node->mask & BIT(i)))
			continue;

		camrtc_dbgfs_show_ast_region(file, ast, i);

		if (node->mask & (node->mask - 1)) /* are multiple bits set? */
			seq_puts(file, "\n");
	}

	iounmap(ast);
	return 0;
}

DEFINE_SEQ_FOPS(camrtc_dbgfs_fops_ast, camrtc_dbgfs_show_ast);

static const struct debugfs_reg32 ast_common_regs[] = {
	{ .name = "control", 0x0 },
	{ .name = "error_status", 0x4 },
	{ .name = "error_addr_lo", 0x8 },
	{ .name = "error_addr_h", 0xC },
	{ .name = "streamid_ctl_0", 0x20 },
	{ .name = "streamid_ctl_1", 0x24 },
	{ .name = "streamid_ctl_2", 0x28 },
	{ .name = "streamid_ctl_3", 0x2C },
	{ .name = "streamid_ctl_4", 0x30 },
	{ .name = "streamid_ctl_5", 0x34 },
	{ .name = "streamid_ctl_6", 0x38 },
	{ .name = "streamid_ctl_7", 0x3C },
	{ .name = "streamid_ctl_8", 0x40 },
	{ .name = "streamid_ctl_9", 0x44 },
	{ .name = "streamid_ctl_10", 0x48 },
	{ .name = "streamid_ctl_11", 0x4C },
	{ .name = "streamid_ctl_12", 0x50 },
	{ .name = "streamid_ctl_13", 0x54 },
	{ .name = "streamid_ctl_14", 0x58 },
	{ .name = "streamid_ctl_15", 0x5C },
	{ .name = "write_block_status", 0x60 },
	{ .name = "read_block_status", 0x64 },
};

static const struct debugfs_reg32 ast_region_regs[] = {
	{ .name = "slave_lo", 0x100 },
	{ .name = "slave_hi", 0x104 },
	{ .name = "mask_lo", 0x108 },
	{ .name = "mask_hi", 0x10C },
	{ .name = "master_lo", 0x110 },
	{ .name = "master_hi", 0x114 },
	{ .name = "control", 0x118 },
};

static int ast_regset_create_files(struct tegra_ivc_channel *ch,
				struct dentry *dir,
				struct ast_regset *ars,
				char const *ast_name)
{
	void __iomem *base;
	int i;

	base = iomap_byname(camrtc_get_device(ch), ast_name);
	if (IS_ERR_OR_NULL(base))
		return -ENOMEM;

	ars->common.base = base;
	ars->common.regs = ast_common_regs;
	ars->common.nregs = ARRAY_SIZE(ast_common_regs);

	debugfs_create_regset32("regs-common", 0444, dir, &ars->common);

	for (i = 0; i < ARRAY_SIZE(ars->region); i++) {
		char name[16];

		snprintf(name, sizeof(name), "regs-region%u", i);

		ars->region[i].base = base + i * TEGRA_APS_AST_REGION_STRIDE;
		ars->region[i].regs = ast_region_regs;
		ars->region[i].nregs = ARRAY_SIZE(ast_region_regs);

		debugfs_create_regset32(name, 0444, dir, &ars->region[i]);
	}

	return 0;
}

static int camrtc_debug_populate(struct tegra_ivc_channel *ch)
{
	struct camrtc_debug *crd = tegra_ivc_channel_get_drvdata(ch);
	struct dentry *dir;
	struct camrtc_dbgfs_ast_node *ast_nodes;
	unsigned int i, dma, region;
	char const *name = "camrtc";

	of_property_read_string(ch->dev.of_node, NV(debugfs), &name);

	crd->root = dir = debugfs_create_dir(name, NULL);
	if (dir == NULL)
		return -ENOMEM;

	if (!debugfs_create_file("version", 0444, dir, ch,
			&camrtc_dbgfs_fops_version))
		goto error;
	if (!debugfs_create_file("reboot", 0400, dir, ch,
			&camrtc_dbgfs_fops_reboot))
		goto error;
	if (!debugfs_create_file("ping", 0444, dir, ch,
			&camrtc_dbgfs_fops_ping))
		goto error;
	if (!debugfs_create_file("sm-ping", 0444, dir, ch,
			&camrtc_dbgfs_fops_sm_ping))
		goto error;
	if (!debugfs_create_file("log-level", 0644, dir, ch,
			&camrtc_dbgfs_fops_loglevel))
		goto error;
	if (!debugfs_create_u32("timeout", 0644, dir,
			&crd->parameters.completion_timeout))
		goto error;
	if (!debugfs_create_file("forced-reset-restore", 0400, dir, ch,
			&camrtc_dbgfs_fops_forced_reset_restore))
		goto error;

	dir = debugfs_create_dir("mods", crd->root);
	if (!dir)
		goto error;
	if (!debugfs_create_u32("loops", 0644, dir,
			&crd->parameters.mods_loops))
		goto error;
	if (!debugfs_create_file("result", 0400, dir, ch,
			&camrtc_dbgfs_fops_mods_result))
		goto error;

	dir = debugfs_create_dir("rtos", crd->root);
	if (!dir)
		goto error;
	if (!debugfs_create_file("state", 0444, dir, ch,
			&camrtc_dbgfs_fops_freertos_state))
		goto error;

	dir = debugfs_create_dir("test", crd->root);
	if (!dir)
		goto error;
	if (!debugfs_create_file("available", 0444, dir, ch,
			&camrtc_dbgfs_fops_test_list))
		goto error;
	if (!debugfs_create_file("case", 0644, dir, ch,
			&camrtc_dbgfs_fops_test_case))
		goto error;
	if (!debugfs_create_file("result", 0400, dir, ch,
			&camrtc_dbgfs_fops_test_result))
		goto error;
	if (!debugfs_create_u32("timeout", 0644, dir,
			&crd->parameters.test_timeout))
		goto error;
	for (i = 0; i < ARRAY_SIZE(crd->mem); i++) {
		char name[8];

		crd->mem[i].index = i;
		snprintf(name, sizeof(name), "mem%u", i);
		if (!debugfs_create_file(name, 0644, dir,
			&crd->mem[i], &camrtc_dbgfs_fops_test_mem))
			goto error;
	}

	ast_nodes = devm_kzalloc(&ch->dev, 18 * sizeof(*ast_nodes),
					GFP_KERNEL);
	if (unlikely(ast_nodes == NULL))
		goto error;

	for (dma = 0; dma <= 1; dma++) {
		const char *ast_name = dma ? "ast-dma" : "ast-cpu";

		dir = debugfs_create_dir(ast_name, crd->root);
		if (dir == NULL)
			goto error;

		ast_regset_create_files(ch, dir, &crd->ast_regsets[dma],
					ast_name);

		ast_nodes->ch = ch;
		ast_nodes->name = ast_name;
		ast_nodes->mask = 0xff;

		if (!debugfs_create_file("all", 0444, dir, ast_nodes,
						&camrtc_dbgfs_fops_ast))
			goto error;

		ast_nodes++;

		for (region = 0; region < 8; region++) {
			char name[8];

			snprintf(name, sizeof name, "%u", region);

			ast_nodes->ch = ch;
			ast_nodes->name = ast_name;
			ast_nodes->mask = BIT(region);

			if (!debugfs_create_file(name, 0444, dir, ast_nodes,
						&camrtc_dbgfs_fops_ast))
				goto error;

			ast_nodes++;
		}
	}

	return 0;
error:
	debugfs_remove_recursive(crd->root);
	return -ENOMEM;
}

static struct device *camrtc_get_linked_device(
	struct device *dev, char const *name, int index)
{
	struct device_node *np;
	struct platform_device *pdev;

	np = of_parse_phandle(dev->of_node, name, index);
	if (np == NULL)
		return NULL;

	pdev = of_find_device_by_node(np);
	of_node_put(np);

	if (pdev == NULL) {
		dev_info(dev, "%s[%u] node has no device\n", name, index);
		return NULL;
	}

	return &pdev->dev;
}

static int camrtc_get_streamid(struct device *dev)
{
	int streamid = -ENODEV;

	if (dev == NULL)
		return 0;

	if (dev->archdata.iommu != NULL)
		streamid = iommu_get_hwid(dev->archdata.iommu, dev, 0);

	if (streamid < 0)
		streamid = tegra_mc_get_smmu_bypass_sid();

	return streamid;
}

static int camrtc_debug_probe(struct tegra_ivc_channel *ch)
{
	struct device *dev = &ch->dev;
	struct camrtc_debug *crd;
	uint32_t bw;

	BUG_ON(ch->ivc.frame_size < sizeof(struct camrtc_dbg_request));
	BUG_ON(ch->ivc.frame_size < sizeof(struct camrtc_dbg_response));

	crd = devm_kzalloc(dev, sizeof(*crd) + ch->ivc.frame_size, GFP_KERNEL);
	if (unlikely(crd == NULL))
		return -ENOMEM;

	crd->channel = ch;
	crd->parameters.test_case = (char *)(crd + 1);
	crd->parameters.mods_loops = 20;

	if (of_property_read_u32(dev->of_node,
			NV(ivc-timeout),
			&crd->parameters.completion_timeout))
		crd->parameters.completion_timeout = 50;

	if (of_property_read_u32(dev->of_node,
			NV(test-timeout),
			&crd->parameters.test_timeout))
		crd->parameters.test_timeout = 1000;

	mutex_init(&crd->mutex);
	init_waitqueue_head(&crd->waitq);

	tegra_ivc_channel_set_drvdata(ch, crd);

	crd->mem_devices[0] = camrtc_get_linked_device(dev, NV(mem-map), 0);
	crd->mem_devices[1] = camrtc_get_linked_device(dev, NV(mem-map), 1);
	crd->mem_devices[2] = camrtc_get_linked_device(dev, NV(mem-map), 2);

	if (of_property_read_u32(dev->of_node, NV(test-bw), &bw) == 0) {
		unsigned long test_bw;

		if (bw == 0xFFFFFFFFU)
			test_bw = tegra_bwmgr_get_max_emc_rate();
		else
			test_bw = tegra_bwmgr_round_rate(bw);

		crd->parameters.test_bw = test_bw;

		dev_dbg(dev, "using emc rate %lu for tests\n", test_bw);
	}

	if (crd->mem_devices[0] == NULL) {
		dev_dbg(dev, "missing %s\n", NV(mem-map));
		crd->mem_devices[0] = get_device(camrtc_get_device(ch));
	}

	crd->streamids.rtcpu = camrtc_get_streamid(crd->mem_devices[0]);
	crd->streamids.vi = camrtc_get_streamid(crd->mem_devices[1]);
	crd->streamids.isp = camrtc_get_streamid(crd->mem_devices[2]);

	if (camrtc_debug_populate(ch))
		return -ENOMEM;
	return 0;
}

static void camrtc_debug_remove(struct tegra_ivc_channel *ch)
{
	struct camrtc_debug *crd = tegra_ivc_channel_get_drvdata(ch);
	int i;
	struct device *mem_dev = crd->mem_devices[0];

	for (i = 0; i < ARRAY_SIZE(crd->mem); i++) {
		struct camrtc_test_mem *mem = &crd->mem[i];

		if (mem->size == 0)
			continue;

		dma_free_coherent(mem_dev, mem->size, mem->ptr, mem->iova);
		memset(mem, 0, sizeof(*mem));
	}

	put_device(crd->mem_devices[0]);
	put_device(crd->mem_devices[1]);
	put_device(crd->mem_devices[2]);

	debugfs_remove_recursive(crd->root);
}

static const struct tegra_ivc_channel_ops tegra_ivc_channel_debug_ops = {
	.probe	= camrtc_debug_probe,
	.remove	= camrtc_debug_remove,
	.notify	= camrtc_debug_notify,
};

static const struct of_device_id camrtc_debug_of_match[] = {
	{ .compatible = "nvidia,tegra186-camera-ivc-protocol-debug" },
	{ },
};

static struct tegra_ivc_driver camrtc_debug_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.bus	= &tegra_ivc_bus_type,
		.name	= "tegra-camera-rtcpu-debugfs",
		.of_match_table = camrtc_debug_of_match,
	},
	.dev_type	= &tegra_ivc_channel_type,
	.ops.channel	= &tegra_ivc_channel_debug_ops,
};
tegra_ivc_subsys_driver_default(camrtc_debug_driver);

MODULE_DESCRIPTION("Debug Driver for Camera RTCPU");
MODULE_AUTHOR("Pekka Pessi <ppessi@nvidia.com>");
MODULE_LICENSE("GPL v2");
