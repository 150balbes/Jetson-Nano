/*
 * drivers/video/tegra/dc/dc_common.c
 *
 * Copyright (c) 2017-2020, NVIDIA CORPORATION, All rights reserved.
 * Author: Arun Swain <arswain@nvidia.com>
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
#include <linux/pm_runtime.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include "dc.h"
#include "dc_reg.h"
#include "dc_priv.h"
#include <linux/nvhost.h>
#include "dc_common.h"
#include "nvhost_channel.h"
#include "nvhost_job.h"
#include "host1x/host1x03_hardware.h"
#include <trace/events/display.h>

#define CMDBUF_SIZE		128
#define NV_DISPLAY_CLASS_ID	0x70
#define NVDISP_HEAD_WORD_OFFSET	0x4000
#define T210_HEAD_WORD_OFFSET	0x10000
#define HEAD_WORD_OFFSET	head_offset

#define WAIT_TYPE_PROGRAM_REG			(1<<0)
#define WAIT_TYPE_CHECK_GEN_ACT_PROMOTION	(1<<1)
#define WRITE_READ_MUX_ACTIVE			(0x05)
#define WRITE_MUX_ACTIVE_READ_MUX_ASSEMBLY	(0x01)
/**
 * __nvhost_opcode_nonincr_write_reg - Fill the command buffer with
 * host1x opcode and value.
 * @buff: the command buffer to filled by dc_common
 * @len: length to keep track of the no. of words written to @buff
 * @offset: register offset host1x to write to.
 * @value: value to be written.
 *
 * __nvhost_opcode_nonincr_write_reg prepares the command buffer.
 */
#define __nvhost_opcode_nonincr_write_reg(buff, len, offset, value)	\
({									\
	buff[len++] = (9 << 28) | 1;					\
	buff[len++] = (11 << 28) | offset;				\
	buff[len++] = value;						\
})

/**
 * __received_bit_set - Sets a particular bit
 * @nr: bit number to be set
 * @addr: address of destination variable
 */
#define __received_bit_set(nr, addr) __set_bit(nr, addr)

/**
 * __received_bit_clear - Clears a particular bit
 * @nr: bit number to be cleared
 * @addr: address of destination variable
 */
#define __received_bit_clear(nr, addr) __clear_bit(nr, addr)

#define __all_req_rcvd(req_rcvd, valid_heads)	\
	(req_rcvd == valid_heads)
/**
 * __get_word_offset - gives the word offset for a register
 * @offset: word offset from @HEAD_WORD_OFFSET
 * @head_id: head no.
 */
#define __get_word_offset(offset, head_id)	\
	(offset + head_id * HEAD_WORD_OFFSET)

/**
 * __get_byte_offset - gives the byte offset for a register
 * @offset: word offset from @HEAD_WORD_OFFSET
 * @head_id: head no.
 */
#define __get_byte_offset(offset, head_id)	\
	(__get_word_offset(offset, head_id) * 4)

/**
 * __wait_timed_out - Checks if the process woke up because of timeout.
 * @req_rcvd: the bit-mapped data for received requests.
 * @valid_heads: the bit mapped data for heads ids involved in framelock.
 * @addr: address of @dc_common->flags
 * @bit: bit to be checked in the flags. Can be either
 *	DC_COMMON_JOB_SUBMITTED or DC_COMMON_ERROR_CHECK_DONE.
 */
#define __wait_timed_out(req_rcvd, valid_heads, addr, bit)	\
	(!__all_req_rcvd(req_rcvd, valid_heads) &&		\
	!(test_bit(bit, addr)))

/**
 * __valid_request - Checks if the request received from a head is valid.
 * @valid_heads: the bit mapped data for heads ids involved in framelock.
 * @head_id: head id of the caller head.
 */
#define __valid_request(valid_heads, head_id)	\
	(valid_heads & (0x01 << head_id))

#ifdef CONFIG_OF
static struct of_device_id tegra_display_common_of_match[] = {
	{.compatible = "nvidia,tegra_dc_common", },
	{ },
};
#endif

static int max_heads;
static int head_offset;
static struct tegra_dc_common *dc_common;
static bool probe_success;

/**
 * tegra_dc_common_probe_status - Returns status
 * of dc_common module probe.
 */
bool tegra_dc_common_probe_status(void)
{
	return probe_success;
}

/**
 * tegra_dc_common_get_imp_table - Returns a pointer
 * to global imp table if provided in device tree.
 */
struct nvdisp_imp_table *tegra_dc_common_get_imp_table(void)
{
	if (!dc_common)
		return NULL;
	return dc_common->imp_table;
}
/**
 * dc_common_channel_submit_gather - prepares/submits the hsot1x job and
 *					waits for completion too.
 * @dsp_cmd_reg: Word offset of DC_CMD_STATE_CONTROL from @HEAD_WORD_OFFSET.
 * @dsp_cmd_state_access_reg: Word offset of DC_CMD_STATE_ACCESS from
 *				@HEAD_WORD_OFFSET.
 * @lock_type: Could be frame_lock or flip_lock
 *
 * Fills the command buffer, gathers the address, submits the job, and
 * waits for the job completion. It programs the immediate syncpt condition
 * and asks host1x to submit the job immediately for this channel.
 *
 * Return: 0 if no errors else corresponding error value.
 */
static inline int _dc_common_channel_submit_gather(ulong dsp_cmd_reg,
			ulong dsp_cmd_state_access_reg, int lock_type)
{
	int i;
	int ret;
	int ofst;
	int head_id = 0;
	u32 buf_length = 0;
	struct nvhost_job *job;
	struct nvhost_device_data *pdata;

	if (!dc_common)
		return -ENODEV;

	pdata = platform_get_drvdata(dc_common->pdev);

	if (lock_type == LOCK_TYPE_FLIP) {
		for_each_set_bit(i, &dc_common->valid_heads, max_heads) {
			head_id = i;
			ofst = __get_word_offset(dsp_cmd_reg, i);
			__nvhost_opcode_nonincr_write_reg(dc_common->cpuvaddr,
				buf_length, ofst, dc_common->upd_val[i]);
		}
	} else {
		for_each_set_bit(i, &dc_common->valid_heads, max_heads) {
			head_id = i;
			ofst = __get_word_offset(dsp_cmd_state_access_reg, i);
			__nvhost_opcode_nonincr_write_reg(dc_common->cpuvaddr,
				buf_length, ofst, WRITE_READ_MUX_ACTIVE);

			ofst = __get_word_offset(dsp_cmd_reg, i);
			__nvhost_opcode_nonincr_write_reg(dc_common->cpuvaddr,
				buf_length, ofst,
				dc_common->dsp_cmd_reg_val[i]);

			ofst = __get_word_offset(dsp_cmd_state_access_reg, i);
			__nvhost_opcode_nonincr_write_reg(dc_common->cpuvaddr,
				buf_length, ofst,
				WRITE_MUX_ACTIVE_READ_MUX_ASSEMBLY);

		}
	}

	ofst = __get_word_offset(DC_CMD_GENERAL_INCR_SYNCPT, head_id);
	__nvhost_opcode_nonincr_write_reg(dc_common->cpuvaddr, buf_length, ofst,
					IMMEDIATE_COND | dc_common->syncpt_id);

	job = nvhost_job_alloc(dc_common->channel, 1, 0, 0, 1);
	if (!job) {
		dev_err(&dc_common->pdev->dev, "failed to allocate job\n");
		ret = -ENOMEM;
		goto err_handle;
	}

	trace_host1x_job_allocated(dc_common, &dc_common->head_data, -1);

	job->sp->id = dc_common->syncpt_id;
	job->sp->incrs = 1;
	job->num_syncpts = 1;

	nvhost_job_add_client_gather_address(job, buf_length,
				pdata->class, dc_common->dma_handle);

	ret = nvhost_channel_submit(job);
	if (ret)
		goto err_handle_free_job;

	trace_host1x_job_submitted(dc_common, &dc_common->head_data, -1);

	nvhost_syncpt_wait_timeout_ext(dc_common->pdev, dc_common->syncpt_id,
				job->sp->fence, (u32)MAX_SCHEDULE_TIMEOUT,
				NULL, NULL);

	trace_host1x_job_executed(dc_common, &dc_common->head_data, -1);
	return 0;

err_handle_free_job:
	nvhost_job_put(job);
	job = NULL;
err_handle:
	dev_err(&dc_common->pdev->dev,
			"%s: gather and job submit failed\n", __func__);
	return ret;
}

/**
 * _dc_common_wait - Puts the process into wait queue based on a conditon
 *			and after the process wakes up checks if it's a
 *			gracefull wake-up or because of timeout.
 * @head_id: Head id no.
 *
 * Puts the requesting process in the corresponding wait_queue as
 * TASK_UNINTERRUPTIBLE when it waits. The process waits as long as dc_common
 * hasn't received requests from all the heads or if time-out happens. In case
 * of the latter, it lets the caller know after clearing the corresponding
 * head's bit from @fl_lck_req_rcvd.
 *
 * Static inline to reduce overheads. Might slightly increase the code
 * memory footprint. But even readablity inreases a bit.
 *
 * Return: -ETIME if timeout happens else 0. -EINVAL if @wait_type is invalid.
 */
static inline int _dc_common_wait(int head_id)
{
	int ret = 0;

	___wait_event(dc_common->prgrm_reg_reqs_wq,
		___wait_cond_timeout(__all_req_rcvd(
		dc_common->head_data.fl_lck_req_rcvd,
		dc_common->valid_heads)), TASK_UNINTERRUPTIBLE,
		0, msecs_to_jiffies(6000),
		mutex_unlock(&dc_common->lock);
		__ret = schedule_timeout(__ret);
		mutex_lock(&dc_common->lock));

	if (__wait_timed_out(dc_common->head_data.fl_lck_req_rcvd,
		dc_common->valid_heads, &dc_common->flags,
		DC_COMMON_JOB_SUBMITTED)) {
		/* the body of "if" starts here. */
		ret = -ETIME;
		__received_bit_clear(
			head_id, &dc_common->head_data.fl_lck_req_rcvd);
	} /* End of if(__wait_timed_out)*/

	return ret;
}
/**
 * _tegra_dc_common_sync_frames - Internal function to synchronize frames of
 *				multiple heads participating in frame lock.
 * @dc: Pointer to struct tegra_dc.
 * @dsp_cmd_reg: Word offset of DC_CMD_STATE_CONTROL from @HEAD_WORD_OFFSET.
 * @dsp_cmd_state_access_reg: Word offset of DC_CMD_STATE_ACCESS from
 *				@HEAD_WORD_OFFSET.
 * @dsp_cmd_reg_val: Value to be written to DC_CMD_STATE_CONTROL.
 * @cmd_st_accss_reg_val: Value to be written to DC_CMD_STATE_ACCESS.
 *
 * On T186 it uses host1x pushbuffers to start the heads at the same time.
 */
static int _tegra_dc_common_sync_frames(struct tegra_dc *dc, ulong dsp_cmd_reg,
		ulong dsp_cmd_state_access_reg, ulong dsp_cmd_reg_val)
{
	int i;
	int ret = 0;

	__received_bit_set(dc->ctrl_num, &dc_common->head_data.fr_lck_req_rcvd);

	trace_received_fr_lock_request(dc_common, &dc_common->head_data,
								dc->ctrl_num);

	dc_common->dsp_cmd_reg_val[dc->ctrl_num] = dsp_cmd_reg_val;

	if (tegra_dc_is_t19x()) {
		ret = nvdisp_t19x_program_raster_lock_seq(dc, dsp_cmd_reg_val);
		if (ret)
			goto err_handle;
	}

	if (!__all_req_rcvd(dc_common->head_data.fr_lck_req_rcvd,
						dc_common->valid_heads))
		return 0;

	if (tegra_dc_is_t19x())
		nvdisp_t19x_enable_raster_lock(dc, dc_common->valid_heads);
	else
		ret = _dc_common_channel_submit_gather(dsp_cmd_reg,
				dsp_cmd_state_access_reg, LOCK_TYPE_FRAME);
	if (ret)
		goto err_handle;

	for_each_set_bit(i, &dc_common->valid_heads, max_heads)
		__received_bit_clear(i,	&dc_common->head_data.fr_lck_req_rcvd);

	dc_common->heads_locked = true;

#if 0
	/* For Debug*/
	for_each_set_bit(i, &dc_common->valid_heads, max_heads) {
		int value;

		dc_head = tegra_dc_get_dc(i);
		value = tegra_dc_get_v_count(dc_head);
	}
#endif
	trace_completed_fr_lock_request(dc_common, &dc_common->head_data,
								dc->ctrl_num);
	return 0;

err_handle:
	dev_err(&dc_common->pdev->dev, "%s: failed to sync flips\n", __func__);
	return ret;
}
/**
 * tegra_dc_common_sync_frames - Fucntion exposed to dc heads participating in
 *				frame lock to lock their rasters.
 * @dc: Pointer to struct tegra_dc.
 * @dsp_cmd_reg: Word offset of DC_CMD_STATE_CONTROL from @HEAD_WORD_OFFSET.
 * @dsp_cmd_state_access_reg: Word offset of DC_CMD_STATE_ACCESS from
 *				@HEAD_WORD_OFFSET.
 * @dsp_cmd_reg_val: Value to be written to DC_CMD_STATE_CONTROL.
 *
 * On T186 it uses host1x pushbuffers to start the heads at the same time.
 *
 */

int tegra_dc_common_sync_frames(struct tegra_dc *dc, ulong dsp_cmd_reg,
		ulong dsp_cmd_state_access_reg, ulong dsp_cmd_reg_val)
{
	int ret = 0;

	if (!dc_common || !dc)
		return -ENODEV;

	mutex_lock(&dc_common->lock);

	if (!__valid_request(dc_common->valid_heads, dc->ctrl_num)) {
		mutex_unlock(&dc_common->lock);
		ret = -EINVAL;
		goto err_handle;
	}
	ret = _tegra_dc_common_sync_frames(dc, dsp_cmd_reg,
			dsp_cmd_state_access_reg, dsp_cmd_reg_val);

	mutex_unlock(&dc_common->lock);
	if (ret)
		goto err_handle;

	return 0;

err_handle:
	dev_err(&dc_common->pdev->dev, "%s: failed to sync frames for tegradc:%d\n",
								__func__, dc->ctrl_num);
	return ret;
}
EXPORT_SYMBOL(tegra_dc_common_sync_frames);

/**
 * _tegra_dc_common_sync_flips- Internal function to process requests for flip
 *				lock. This is acheived through programming
 *				DC_CMD_STATE_CONTROL through host1x for
 *				"almost" atomic register writes.
 * @dc: Pointer to the caller head's tegra_dc struct.
 * @upd_val: Value to be written to the head's DC_CMD_STATE_CONTROL reg.
 * @reg:  Word offset of DC_CMD_STATE_CONTROL from @HEAD_WORD_OFFSET.
 *
 * Blocks the requesting processes until all the heads have requested. Submits
 * the host1x job of wrting to the DC_CMD_STATE_CONTROL of mulitple heads
 * through the last process that calls @tegra_dc_common_sync_flips().Currently
 * also handles the case of just 1 head participating in frame-lock for debug
 * purpose.
 *
 * Return: 0 if successful else the corresponding error value.
 */
static int _tegra_dc_common_sync_flips(struct tegra_dc *dc, ulong upd_val,
								ulong reg)
{
	int ret;

	__received_bit_set(dc->ctrl_num, &dc_common->head_data.fl_lck_req_rcvd);
	dc_common->upd_val[dc->ctrl_num] = upd_val;

	trace_received_fl_lock_request(dc_common, &dc_common->head_data,
								dc->ctrl_num);

	ret = _dc_common_wait(dc->ctrl_num);
	if (ret)
		goto err_handle;

	if (!dc_common->head_data.fl_lck_req_completed) {
		dc_common->head_data.fl_lck_req_completed =
					dc_common->valid_heads;
		wake_up(&dc_common->prgrm_reg_reqs_wq);
	}

	__received_bit_clear(dc->ctrl_num,
			&dc_common->head_data.fl_lck_req_completed);

	if (test_bit(DC_COMMON_JOB_SUBMITTED, &dc_common->flags)) {
		if (!dc_common->head_data.fl_lck_req_completed) {
			__clear_bit(DC_COMMON_JOB_SUBMITTED, &dc_common->flags);
			dc_common->head_data.fl_lck_req_rcvd = 0UL;
		}
		return 0;
	}

	ret = _dc_common_channel_submit_gather(reg, 0UL, LOCK_TYPE_FLIP);
	if (ret)
		goto err_handle;

	if (dc_common->head_data.fl_lck_req_completed) {
		__set_bit(DC_COMMON_JOB_SUBMITTED, &dc_common->flags);
		__set_bit(DC_COMMON_ERROR_CHECK_REQ, &dc_common->flags);
	} else {
		dev_info(&dc_common->pdev->dev,
			"%s: Need not have frame/flip lock with just one head\n",
			__func__);
		dc_common->head_data.fl_lck_req_rcvd = 0UL;
	}

	trace_completed_fl_lock_request(dc_common, &dc_common->head_data,
								dc->ctrl_num);

#if 0
	/* For Debugf*/
	for_each_set_bit(i, &dc_common->valid_heads, max_heads) {
		struct tegra_dc *dc_head;
		int value;

		dc_head = tegra_dc_get_dc(i);
		value = tegra_dc_get_v_count(dc_head);
	}
#endif
	return 0;

err_handle:
	dev_err(&dc_common->pdev->dev, "%s: failed to program gen_act for tegradc:%d\n",
						__func__, dc->ctrl_num);
	return ret;
}

/**
 * tegra_dc_common_sync_flips- Function exposed to dc heads participating in
 *				flip lock to sync their flips. This is acheived
 *				through programming DC_CMD_STATE_CONTROL
 *				through host1x for "almost" atomic register
 *				writes.
 * @dc: Pointer to the caller head's tegra_dc struct.
 * @upd_val: Value to be written to the head's DC_CMD_STATE_CONTROL reg.
 * @reg:  Word offset of DC_CMD_STATE_CONTROL from @HEAD_WORD_OFFSET.
 *
 * Please refer to _tegra_dc_common_sync_flips a bit detail description of the
 * algorithm used.
 *
 * Return: 0 if successful else the corresponding error value.
 */

int tegra_dc_common_sync_flips(struct tegra_dc *dc, ulong val, ulong reg)
{
	int ret = 0;

	if (!dc_common || !dc)
		return -ENODEV;

	mutex_lock(&dc_common->lock);

	if (!__valid_request(dc_common->valid_heads, dc->ctrl_num)) {
		mutex_unlock(&dc_common->lock);
		ret = -EINVAL;
		goto err_handle;
	}

	if (test_bit(DC_COMMON_JOB_SUBMITTED, &dc_common->flags)) {
		do {
			mutex_unlock(&dc_common->lock);
			udelay(10);
			mutex_lock(&dc_common->lock);
		} while (test_bit(DC_COMMON_JOB_SUBMITTED, &dc_common->flags));
	}

	ret = _tegra_dc_common_sync_flips(dc, val, reg);
	if (ret) {
		mutex_unlock(&dc_common->lock);
		goto err_handle;
	}

	mutex_unlock(&dc_common->lock);
	return 0;

err_handle:
	dev_err(&dc_common->pdev->dev,
		"%s: failed to sync flips for tegradc:%d\n", __func__,
		dc->ctrl_num);
	return ret;
}
EXPORT_SYMBOL(tegra_dc_common_sync_flips);

/**
 * dc_common_get_gen_act_status - Reads the DC_CMD_STATE_CONTROL register
 *				for all the heads that are frame locked
 *				and figures out if any of the GEN_ACT_REQ
 *				is not promoted.
 * @dc_common: Pointer to struct tegra_dc_common.
 * @reg: The register value to be read. Mostly DC_CMD_STATE_CONTROL.
 *
 * If any of the gen_act_req is not promoted then it returns error.
 *
 * Return: 0 if successful else -1.
 */
static int dc_common_get_gen_act_status(struct tegra_dc_common *dc_common,
						unsigned long reg)
{
	int i;
	int ret = 0;
	int val = 0;
	unsigned long res = 0x00;

	for_each_set_bit(i, &dc_common->valid_heads, max_heads) {
		int offset = __get_byte_offset(reg, i);
		val = readl(dc_common->base + offset);
		if (!(val & GENERAL_ACT_REQ))
			__set_bit(i, &res);
	}
	dc_common->head_data.gen_act_read_result = res;

	if (res == dc_common->valid_heads)
		ret = 0;
	else
		ret = -1;

	return ret;
}

/**
 * tegra_dc_common_handle_flip_lock_error - Checks for error in flip lock.
 * @dc: Pointer to the caller head's tegra_dc struct.
 *
 * tegra_dc_common_handle_flip_lock_error figures if writing to multiple
 * head's DC_CMD_STATE_CONTROL registers still straddled across vsync. If
 * so then it stalls the heads to make sure that all the heads are flip-locked
 * from the next frame onwards. It does the error check only if
 * @DC_COMMON_ERROR_CHECK_REQ flag is set. @DC_COMMON_ERROR_CHECK_REQ is set
 * after submitting a host1x job for syncing flips successfully.
 *
 * TODO : Inform usersapce to drop frames to avoid bubbles in the graphics
 * pipeline.
 *
 * Return: -ENODEV/-EINVAL/-EAGAIN from this function. Other error codes
 * depending on the function called from here.
 */
int tegra_dc_common_handle_flip_lock_error(struct tegra_dc *dc)
{
	int i;
	int ret = 0;
	struct tegra_dc *dc_head = NULL;

	if (!dc_common || !dc)
		return -ENODEV;

	trace_received_err_check_request(dc_common, &dc_common->head_data,
								dc->ctrl_num);
	mutex_lock(&dc_common->lock);

	if (!__valid_request(dc_common->valid_heads, dc->ctrl_num) ||
		!dc_common->heads_locked) {
		mutex_unlock(&dc_common->lock);
		ret = -EINVAL;
		goto err_handle;
	}

	if (!test_bit(DC_COMMON_ERROR_CHECK_REQ, &dc_common->flags)) {
		mutex_unlock(&dc_common->lock);
		return ret;
	}

	if (dc_common_get_gen_act_status(dc_common, DC_CMD_STATE_CONTROL)) {
		ret = -EAGAIN;
		mutex_unlock(&dc_common->lock);
		return ret;
	}

	for_each_set_bit(i, &dc_common->valid_heads, max_heads) {
		dc_head = tegra_dc_get_dc(i);
		if (!dc_head) {
			mutex_unlock(&dc_common->lock);
			ret = -ENODEV;
			goto err_handle;
		}
		tegra_dc_request_trigger_wins(dc_head);
	}

	__clear_bit(DC_COMMON_ERROR_CHECK_REQ,	&dc_common->flags);

	trace_completed_err_check_request(dc_common, &dc_common->head_data,
								dc->ctrl_num);
	mutex_unlock(&dc_common->lock);
	return 0;

err_handle:
	if (ret)
		dev_err(&dc_common->pdev->dev,
			"%s: failed to sync frames\n", __func__);

	return ret;
}
EXPORT_SYMBOL(tegra_dc_common_handle_flip_lock_error);

/**
 * _tegra_dc_common_enable_frame_lock - Enables framelock in all the
 *					participating heads.
 * @dc_common: Pointer to struct tegra_dc_common.
 *
 * Expects the locking/unlocking of mutex to be handled outside this
 * function.
 *
 * Return: -EINVAL if the any of the heads are unreachable.
 */
static int _tegra_dc_common_enable_frame_lock(struct tegra_dc_common *dc_common)
{
	int i;
	int ret = 0;
	unsigned long frame_lock = 0;
	struct tegra_dc *dc = NULL;

	for_each_set_bit(i, &dc_common->valid_heads, max_heads) {
		dc = tegra_dc_get_dc(i);
		if (!dc)
			break;
		tegra_dc_enable_disable_frame_lock(dc, true);
		__set_bit(i, &frame_lock);
	}

	if (frame_lock != dc_common->valid_heads) {
		for_each_set_bit(i, &frame_lock, max_heads) {
			dc = tegra_dc_get_dc(i);
			if (dc)
				tegra_dc_enable_disable_frame_lock(dc, false);
		}
		ret = -EINVAL;
		goto err_handle;
	}

	return 0;

err_handle:
	dev_err(&dc_common->pdev->dev,
		"%s: failed to enable frame lock\n", __func__);
	return ret;
}

/**
 * _tegra_dc_common_disable_frame_lock - Disables frame/flip lock in all the
 *					participating heads and in dc_common.
 * @dc_common: Pointer to struct tegra_dc_common.
 *
 * Expects the locking/unlocking of mutex to be handled outside this
 * function. Also wakes up any pending processes in the wait queues.
 *
 * Return: -EINVAL if the any of the heads are unreachable.
 */
static int _tegra_dc_common_disable_frame_lock(
			struct tegra_dc_common *dc_common)
{
	int i;
	int ret = 0;
	unsigned long frame_lock = 0;
	struct tegra_dc *dc = NULL;

	for_each_set_bit(i, &dc_common->valid_heads, max_heads) {
		dc = tegra_dc_get_dc(i);
		if (!dc)
			break;
		tegra_dc_enable_disable_frame_lock(dc, false);
		__set_bit(i, &frame_lock);
	}

	if (frame_lock != dc_common->valid_heads) {
		for_each_set_bit(i, &frame_lock, max_heads) {
			dc = tegra_dc_get_dc(i);
			if (dc)
				tegra_dc_enable_disable_frame_lock(dc, true);
		}
		ret = -EINVAL;
		goto err_handle;
	}

	dc_common->heads_locked = false;
	dc_common->head_data.fr_lck_req_rcvd = 0UL;
	if (dc_common->head_data.fl_lck_req_rcvd)
		wake_up(&dc_common->prgrm_reg_reqs_wq);

	return 0;

err_handle:
	dev_err(&dc_common->pdev->dev,
		"%s: failed to disable frame lock with ret val :%d \n",
		__func__, ret);
	return ret;

}

/**
 * _is_frame_lock_enable - Checks if current state of frame/flip lock.
 * @dc_common: Pointer to struct tegra_dc_common.
 *
 * Expects the locking/unlocking of mutex to be handled outside this
 * function.
 *
 * Return: True if frame/flip lock is enabled else False otherwise.
 */
static bool _is_frame_lock_enable(struct tegra_dc_common *dc_common)
{
	int i;
	bool ret = false;
	unsigned long frame_lock = 0x00;

	for_each_set_bit(i, &dc_common->valid_heads, max_heads) {
		struct tegra_dc *dc = tegra_dc_get_dc(i);
		if (!dc)
			break;
		if (dc->frm_lck_info.frame_lock_enable)
			__set_bit(i, &frame_lock);
	}

	if (frame_lock == dc_common->valid_heads)
		ret = true;

	return ret;
}

static inline unsigned long _tegra_dc_common_get_head_mask(
				struct tegra_dc_common *dc_common)
{
	return dc_common->valid_heads;
}

/**
 * _tegra_dc_common_upd_valid_heads_mask - Updates the @valid_heads mask
 *						in dc_common.
 * @dc_common: Pointer to struct tegra_dc_common.
 * @new_mask: The new valid_heads mask value.
 *
 * Expects the locking/unlocking of mutex to be handled outside this
 * function. And also the user space needs to blank/unblank when the
 * valid_heads value is being updated.
 *
 * Return: 0 if succesfull else error value from
 * @_tegra_dc_common_enable_frame_lock.
 */
static int _tegra_dc_common_upd_valid_heads_mask(
		struct tegra_dc_common *dc_common, unsigned long new_mask)
{
	if (dc_common->valid_heads == new_mask)
		return 0;

	dc_common->valid_heads = new_mask;

	return 0;

}

/**
 * tegra_dc_common_get_frm_lock_params- Gets the current valid_heads and
 *					frame_lock status for userspace
 * @params: Pointer to struct tegra_dc_ext_control_frm_lck_params.
 *
 * Return: 0 if succesfull else -ENODEV is dc_common isn't present.
 */

int tegra_dc_common_get_frm_lock_params(
			struct tegra_dc_ext_control_frm_lck_params *params)
{
	if (!dc_common)
		return -ENODEV;

	mutex_lock(&dc_common->lock);

	params->frame_lock_status = _is_frame_lock_enable(dc_common);
	params->valid_heads = _tegra_dc_common_get_head_mask(dc_common);

	mutex_unlock(&dc_common->lock);

	return 0;
}
EXPORT_SYMBOL(tegra_dc_common_get_frm_lock_params);

/**
 * tegra_dc_common_set_frm_lock_params- Used from control.c to update
 *					frame_lock related parameters from
 *					user sapce.
 * @params: Pointer to struct tegra_dc_ext_control_frm_lck_params.
 *
 * Expects the user space to blank/unblank when it sets new values.
 *
 * Return: 0 if succesfull else the corresponding error value.
 */

int tegra_dc_common_set_frm_lock_params(
			struct tegra_dc_ext_control_frm_lck_params *params)
{
	int ret = 0;

	if (!dc_common)
		return -ENODEV;

	mutex_lock(&dc_common->lock);

	if (params->frame_lock_status != _is_frame_lock_enable(dc_common)) {
		if (params->frame_lock_status)
			ret = _tegra_dc_common_enable_frame_lock(dc_common);
		else
			ret = _tegra_dc_common_disable_frame_lock(dc_common);
	}
	if (ret) {
		mutex_unlock(&dc_common->lock);
		return ret;
	}

	if (hweight_long(params->valid_heads) > max_heads) {
		mutex_unlock(&dc_common->lock);
		return -EINVAL;
	}

	ret = _tegra_dc_common_upd_valid_heads_mask(dc_common,
						params->valid_heads);

	mutex_unlock(&dc_common->lock);

	return ret;
}
EXPORT_SYMBOL(tegra_dc_common_set_frm_lock_params);

#ifdef CONFIG_DEBUG_FS
static int dbg_dc_common_frame_lock_enable_show(struct seq_file *m,
							void *unused)
{
	struct tegra_dc_common *dc_common = m->private;

	if (!dc_common)
		return -ENODEV;

	if (mutex_lock_killable(&dc_common->lock))
		return -EINVAL;

	if (_is_frame_lock_enable(dc_common))
		seq_printf(m, "1\n");
	else
		seq_printf(m, "0\n");

	mutex_unlock(&dc_common->lock);

	return 0;
}

static int dbg_dc_common_frame_lock_enable_open(struct inode *inode,
							struct file *file)
{
	return single_open(file, dbg_dc_common_frame_lock_enable_show,
							inode->i_private);
}

static ssize_t dbg_dc_common_frame_lock_enable_write(struct file *file,
			const char __user *addr, size_t len, loff_t *pos)
{
	int ret;
	long enable_val;
	struct seq_file *m = file->private_data;
	struct tegra_dc_common *dc_common = m->private;

	if (!dc_common)
		return -ENODEV;

	ret = kstrtol_from_user(addr, len, 10, &enable_val);
	if (ret < 0)
		return ret;

	mutex_lock(&dc_common->lock);
	if (enable_val)
		ret = _tegra_dc_common_enable_frame_lock(dc_common);
	else
		ret = _tegra_dc_common_disable_frame_lock(dc_common);

	mutex_unlock(&dc_common->lock);
	return len;
}

static const struct file_operations frame_lock_enable_fops = {
	.open	= dbg_dc_common_frame_lock_enable_open,
	.read	= seq_read,
	.write	= dbg_dc_common_frame_lock_enable_write,
	.llseek	= seq_lseek,
	.release = single_release,
};

static int dbg_dc_common_frame_lock_head_mask_show(struct seq_file *m,
								void *unused)
{
	u8 head_mask;
	struct tegra_dc_common *dc_common = m->private;

	if (!dc_common)
		return -ENODEV;

	if (mutex_lock_killable(&dc_common->lock))
		return -EINVAL;

	head_mask = _tegra_dc_common_get_head_mask(dc_common);

	seq_printf(m, "Head Mask : %d\n", head_mask);

	mutex_unlock(&dc_common->lock);

	return 0;
}

static int dbg_dc_common_frame_lock_head_mask_open(struct inode *inode,
							struct file *file)
{
	return single_open(file, dbg_dc_common_frame_lock_head_mask_show,
							inode->i_private);
}

static ssize_t dbg_dc_common_frame_lock_head_mask_write(struct file *file,
			const char __user *addr, size_t len, loff_t *pos)
{
	int ret;
	long new_head_mask;
	struct seq_file *m = file->private_data;
	struct tegra_dc_common *dc_common = m->private;

	if (!dc_common)
		return -ENODEV;

	ret = kstrtol_from_user(addr, len, 10, &new_head_mask);
	if (ret < 0)
		return ret;

	if (hweight_long(new_head_mask) > max_heads)
		return -EINVAL;

	if (new_head_mask) {
		mutex_lock(&dc_common->lock);
		ret = _tegra_dc_common_upd_valid_heads_mask(dc_common,
								new_head_mask);
		mutex_unlock(&dc_common->lock);
		if (ret)
			return ret;
	}

	return len;
}

static const struct file_operations frame_lock_head_mask_fops = {
	.open	= dbg_dc_common_frame_lock_head_mask_open,
	.read	= seq_read,
	.write	= dbg_dc_common_frame_lock_head_mask_write,
	.llseek	= seq_lseek,
	.release = single_release,
};

static void tegra_dc_common_remove_debugfs(struct tegra_dc_common *dc_common)
{
	if (dc_common->debugdir)
		debugfs_remove(dc_common->debugdir);

	dc_common->debugdir = NULL;
}

static void tegra_dc_common_create_debugfs(struct tegra_dc_common *dc_common)
{
	struct dentry *retval;

	dc_common->debugdir = debugfs_create_dir("dc_common", NULL);
	if (!dc_common->debugdir)
		goto err_handle;

	retval = debugfs_create_file("frame_lock_enable", 0444,
		dc_common->debugdir, dc_common,	&frame_lock_enable_fops);
	if (!retval)
		goto err_handle;

	retval = debugfs_create_file("frame_lock_head_mask", 0444,
		dc_common->debugdir, dc_common,	&frame_lock_head_mask_fops);
	if (!retval)
		goto err_handle;

	return;

err_handle:
	dev_err(&dc_common->pdev->dev, "could not create debugfs\n");
	tegra_dc_common_remove_debugfs(dc_common);
}
#else /* !CONFIG_DEBUGFS */
static inline void tegra_dc_common_create_debugfs(
		struct tegra_dc_common *dc_common) { };
static inline void tegra_dc_common_remove_debugfs(
		struct tegra_dc_common *dc_common) { };
#endif /* CONFIG_DEBUGFS */

static int tegra_dc_common_assign_head_offset(void)
{
	int ret = 0;

	if (tegra_dc_is_t21x())
		head_offset = T210_HEAD_WORD_OFFSET;
	else if (tegra_dc_is_nvdisplay())
		head_offset = NVDISP_HEAD_WORD_OFFSET;
	else
		ret = -ENOENT;

	return ret;
}

static int tegra_dc_common_probe(struct platform_device *pdev)
{
	int ret = 0;
	void __iomem *base;
	struct resource dt_res;
	struct resource *res;
	struct nvhost_device_data *pdata = NULL;
	struct device_node *np = pdev->dev.of_node;
	struct tegra_dc_common_platform_data *dt_pdata = NULL;

	if (!np) {
		dev_err(&pdev->dev, "no platform data\n");
		return -ENOENT;
	}

	max_heads = tegra_dc_get_numof_dispheads();
	if (max_heads <= 0) {
		dev_err(&pdev->dev, "max no. of heads isn't configured\n");
		return -ENOENT;
	}

	ret = tegra_dc_common_assign_head_offset();
	if (ret) {
		dev_err(&pdev->dev, "can't assign head_offset in tegra_dc_common\n");
		return ret;
	}

	dc_common = devm_kzalloc(&pdev->dev, sizeof(*dc_common), GFP_KERNEL);
	if (!dc_common) {
		dev_err(&pdev->dev, "can't allocate memory for tegra_dc_common\n");
		return -ENOMEM;
	}

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!dc_common) {
		dev_err(&pdev->dev, "can't allocate memory for nvhost_device_data\n");
		goto err_free_dc_common;
	}

	ret = of_address_to_resource(np, 0, &dt_res);
	if (ret)
		goto err_free;
	res = &dt_res;
	base = ioremap(res->start, resource_size(res));
	if (!base) {
		dev_err(&pdev->dev, "registers can't be mapped\n");
		ret = -EBUSY;
		goto err_free;
	}
	dc_common->base = base;

	dt_pdata = of_dc_common_parse_platform_data(pdev);
	if (IS_ERR_OR_NULL(dt_pdata)) {
		if (dt_pdata)
			ret = PTR_ERR(dt_pdata);
		goto err_iounmap_reg;
	}

	dc_common->pdev = pdev;
	pdata->pdev = pdev;
	platform_set_drvdata(pdev, pdata);
	pdata->private_data = dc_common;
	pdata->class = NV_DISPLAY_CLASS_ID;

	ret = nvhost_channel_map(pdata, &dc_common->channel, pdata);
	if (ret) {
		dev_err(&pdev->dev, "Nvhost Channel map failed\n");
		goto err_iounmap_reg;
	}
	dev_info(&pdev->dev, "host1x channel mapped\n");

	dc_common->syncpt_id = nvhost_get_syncpt_host_managed(pdev, 0,
				pdev->name);
	if (!dc_common->syncpt_id) {
		dev_err(&pdev->dev, "failed to get syncpoint\n");
		ret = -ENOMEM;
		goto err_drop_channel;
	}
	dev_info(&pdev->dev, "dc_common syncpt # %d allocated\n",
			dc_common->syncpt_id);

	dc_common->cpuvaddr = dma_alloc_coherent(
				dc_common->pdev->dev.parent, CMDBUF_SIZE,
				&dc_common->dma_handle, GFP_KERNEL);
	if (!dc_common->cpuvaddr) {
		dev_err(&pdev->dev, "failed to allocate command buffer\n");
		ret = -ENOMEM;
		goto err_syncpt_drop;
	}
	dev_info(&dc_common->pdev->dev, "dma mapping done\n");

	mutex_init(&dc_common->lock);
	init_waitqueue_head(&dc_common->prgrm_reg_reqs_wq);

	dc_common->valid_heads = dt_pdata->valid_heads;
	dc_common->imp_table = dt_pdata->imp_table;


	dc_common->upd_val = devm_kzalloc(&pdev->dev,
		sizeof(*dc_common->upd_val) * max_heads, GFP_KERNEL);
	if (!dc_common->upd_val) {
		dev_err(&pdev->dev, "can't allocate memory for upd_val\n");
		ret = -ENOMEM;
		goto err_syncpt_drop;
	}

	dc_common->dsp_cmd_reg_val = devm_kzalloc(&pdev->dev,
		sizeof(*dc_common->dsp_cmd_reg_val) * max_heads, GFP_KERNEL);
	if (!dc_common->dsp_cmd_reg_val) {
		dev_err(&pdev->dev, "can't allocate memory for dsp_cmd_reg_val\n");
		ret = -ENOMEM;
		goto err_free_upd_val;
	}

	pm_runtime_enable(&pdev->dev);

	tegra_dc_common_create_debugfs(dc_common);

	probe_success = true;

	return 0;

err_free_upd_val:
	kfree(dc_common->upd_val);
err_syncpt_drop:
	nvhost_syncpt_put_ref_ext(pdev, dc_common->syncpt_id);
err_drop_channel:
	nvhost_putchannel(dc_common->channel, 1);
err_iounmap_reg:
	iounmap(dc_common->base);
err_free:
	kfree(pdata);
err_free_dc_common:
	kfree(dc_common);
	return ret;
}

static int tegra_dc_common_remove(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct tegra_dc_common *dc_common = pdata->private_data;

	mutex_lock(&dc_common->lock);
	_tegra_dc_common_disable_frame_lock(dc_common);
	mutex_unlock(&dc_common->lock);
	dma_free_coherent(pdev->dev.parent, CMDBUF_SIZE,
			dc_common->cpuvaddr, dc_common->dma_handle);
	nvhost_syncpt_put_ref_ext(pdev, dc_common->syncpt_id);
	nvhost_putchannel(dc_common->channel, 1);

	probe_success = false;

	iounmap(dc_common->base);
	kfree(dc_common->dsp_cmd_reg_val);
	kfree(dc_common->upd_val);
	kfree(dc_common);
	return 0;
}

static struct platform_driver tegra_dc_common_driver = {
	.driver = {
		.name = "tegradccommon",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table =
			of_match_ptr(tegra_display_common_of_match),
#endif
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = tegra_dc_common_probe,
	.remove = tegra_dc_common_remove,
};


static int __init tegra_dc_common_module_init(void)
{
	int ret = 0;

	ret = tegra_dc_hw_init();
	if (ret) {
		printk(KERN_ERR "tegradccommon module_init failed\n");
		return ret;
	}

	return platform_driver_register(&tegra_dc_common_driver);
}

static void __exit tegra_dc_common_module_exit(void)
{
	platform_driver_unregister(&tegra_dc_common_driver);
}

module_init(tegra_dc_common_module_init);
module_exit(tegra_dc_common_module_exit);

MODULE_DESCRIPTION("Tegra DC Common for framelock/flipllock support using Host1x Interface");
MODULE_AUTHOR("NVIDIA Corporation");
MODULE_LICENSE("GPL");
MODULE_ALIAS("tegra_dc_common");
