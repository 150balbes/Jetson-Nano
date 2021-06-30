/*
 * Copyright (c) 2014-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/fs.h>
#include <linux/file.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <uapi/linux/nvgpu.h>
#include <linux/anon_inodes.h>

#include <nvgpu/kmem.h>
#include <nvgpu/log.h>
#include <nvgpu/os_sched.h>
#include <nvgpu/gk20a.h>
#include <nvgpu/channel.h>
#include <nvgpu/tsg.h>

#include "gv11b/fifo_gv11b.h"
#include "platform_gk20a.h"
#include "ioctl_tsg.h"
#include "ioctl_channel.h"
#include "os_linux.h"

struct tsg_private {
	struct gk20a *g;
	struct tsg_gk20a *tsg;
};

static int gk20a_tsg_bind_channel_fd(struct tsg_gk20a *tsg, int ch_fd)
{
	struct channel_gk20a *ch;
	int err;

	ch = gk20a_get_channel_from_file(ch_fd);
	if (!ch)
		return -EINVAL;

	err = ch->g->ops.fifo.tsg_bind_channel(tsg, ch);

	gk20a_channel_put(ch);
	return err;
}

static int gk20a_tsg_ioctl_bind_channel_ex(struct gk20a *g,
	struct tsg_gk20a *tsg, struct nvgpu_tsg_bind_channel_ex_args *arg)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct gk20a_sched_ctrl *sched = &l->sched_ctrl;
	struct channel_gk20a *ch;
	struct gr_gk20a *gr = &g->gr;
	int err = 0;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_sched, "tsgid=%u", tsg->tsgid);

	nvgpu_mutex_acquire(&sched->control_lock);
	if (sched->control_locked) {
		err = -EPERM;
		goto mutex_release;
	}
	err = gk20a_busy(g);
	if (err) {
		nvgpu_err(g, "failed to power on gpu");
		goto mutex_release;
	}

	ch = gk20a_get_channel_from_file(arg->channel_fd);
	if (!ch) {
		err = -EINVAL;
		goto idle;
	}

	if (arg->tpc_pg_enabled && (!tsg->tpc_num_initialized)) {
		if ((arg->num_active_tpcs > gr->max_tpc_count) ||
				!(arg->num_active_tpcs)) {
			nvgpu_err(g, "Invalid num of active TPCs");
			err = -EINVAL;
			goto ch_put;
		}
		tsg->tpc_num_initialized = true;
		tsg->num_active_tpcs = arg->num_active_tpcs;
		tsg->tpc_pg_enabled = true;
	} else {
		tsg->tpc_pg_enabled = false; nvgpu_log(g, gpu_dbg_info, "dynamic TPC-PG not enabled");
	}

	if (arg->subcontext_id < g->fifo.max_subctx_count) {
		ch->subctx_id = arg->subcontext_id;
	} else {
		err = -EINVAL;
		goto ch_put;
	}

	nvgpu_log(g, gpu_dbg_info, "channel id : %d : subctx: %d",
				ch->chid, ch->subctx_id);

	/* Use runqueue selector 1 for all ASYNC ids */
	if (ch->subctx_id > CHANNEL_INFO_VEID0)
		ch->runqueue_sel = 1;

	err = ch->g->ops.fifo.tsg_bind_channel(tsg, ch);
ch_put:
	gk20a_channel_put(ch);
idle:
	gk20a_idle(g);
mutex_release:
	nvgpu_mutex_release(&sched->control_lock);
	return err;
}

static int gk20a_tsg_unbind_channel_fd(struct tsg_gk20a *tsg, int ch_fd)
{
	struct channel_gk20a *ch;
	int err = 0;

	ch = gk20a_get_channel_from_file(ch_fd);
	if (!ch)
		return -EINVAL;

	if (ch->tsgid != tsg->tsgid) {
		err = -EINVAL;
		goto out;
	}

	err = gk20a_tsg_unbind_channel(ch);

	/*
	 * Mark the channel timedout since channel unbound from TSG
	 * has no context of its own so it can't serve any job
	 */
	gk20a_channel_set_timedout(ch);

out:
	gk20a_channel_put(ch);
	return err;
}

static int gk20a_tsg_get_event_data_from_id(struct tsg_gk20a *tsg,
				unsigned int event_id,
				struct gk20a_event_id_data **event_id_data)
{
	struct gk20a_event_id_data *local_event_id_data;
	bool event_found = false;

	nvgpu_mutex_acquire(&tsg->event_id_list_lock);
	nvgpu_list_for_each_entry(local_event_id_data, &tsg->event_id_list,
					gk20a_event_id_data, event_id_node) {
		if (local_event_id_data->event_id == event_id) {
			event_found = true;
			break;
		}
	}
	nvgpu_mutex_release(&tsg->event_id_list_lock);

	if (event_found) {
		*event_id_data = local_event_id_data;
		return 0;
	} else {
		return -1;
	}
}

/*
 * Convert common event_id of the form NVGPU_EVENT_ID_* to Linux specific
 * event_id of the form NVGPU_IOCTL_CHANNEL_EVENT_ID_* which is used in IOCTLs
 */
static u32 nvgpu_event_id_to_ioctl_channel_event_id(u32 event_id)
{
	switch (event_id) {
	case NVGPU_EVENT_ID_BPT_INT:
		return NVGPU_IOCTL_CHANNEL_EVENT_ID_BPT_INT;
	case NVGPU_EVENT_ID_BPT_PAUSE:
		return NVGPU_IOCTL_CHANNEL_EVENT_ID_BPT_PAUSE;
	case NVGPU_EVENT_ID_BLOCKING_SYNC:
		return NVGPU_IOCTL_CHANNEL_EVENT_ID_BLOCKING_SYNC;
	case NVGPU_EVENT_ID_CILP_PREEMPTION_STARTED:
		return NVGPU_IOCTL_CHANNEL_EVENT_ID_CILP_PREEMPTION_STARTED;
	case NVGPU_EVENT_ID_CILP_PREEMPTION_COMPLETE:
		return NVGPU_IOCTL_CHANNEL_EVENT_ID_CILP_PREEMPTION_COMPLETE;
	case NVGPU_EVENT_ID_GR_SEMAPHORE_WRITE_AWAKEN:
		return NVGPU_IOCTL_CHANNEL_EVENT_ID_GR_SEMAPHORE_WRITE_AWAKEN;
	}

	return NVGPU_IOCTL_CHANNEL_EVENT_ID_MAX;
}

void gk20a_tsg_event_id_post_event(struct tsg_gk20a *tsg,
				       int __event_id)
{
	struct gk20a_event_id_data *event_id_data;
	u32 event_id;
	int err = 0;
	struct gk20a *g = tsg->g;

	event_id = nvgpu_event_id_to_ioctl_channel_event_id(__event_id);
	if (event_id >= NVGPU_IOCTL_CHANNEL_EVENT_ID_MAX)
		return;

	err = gk20a_tsg_get_event_data_from_id(tsg, event_id,
						&event_id_data);
	if (err)
		return;

	nvgpu_mutex_acquire(&event_id_data->lock);

	nvgpu_log_info(g,
		"posting event for event_id=%d on tsg=%d\n",
		event_id, tsg->tsgid);
	event_id_data->event_posted = true;

	nvgpu_cond_broadcast_interruptible(&event_id_data->event_id_wq);

	nvgpu_mutex_release(&event_id_data->lock);
}

static unsigned int gk20a_event_id_poll(struct file *filep, poll_table *wait)
{
	unsigned int mask = 0;
	struct gk20a_event_id_data *event_id_data = filep->private_data;
	struct gk20a *g = event_id_data->g;
	u32 event_id = event_id_data->event_id;
	struct tsg_gk20a *tsg = g->fifo.tsg + event_id_data->id;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_info, " ");

	poll_wait(filep, &event_id_data->event_id_wq.wq, wait);

	nvgpu_mutex_acquire(&event_id_data->lock);

	if (event_id_data->event_posted) {
		nvgpu_log_info(g,
			"found pending event_id=%d on TSG=%d\n",
			event_id, tsg->tsgid);
		mask = (POLLPRI | POLLIN);
		event_id_data->event_posted = false;
	}

	nvgpu_mutex_release(&event_id_data->lock);

	return mask;
}

static int gk20a_event_id_release(struct inode *inode, struct file *filp)
{
	struct gk20a_event_id_data *event_id_data = filp->private_data;
	struct gk20a *g = event_id_data->g;
	struct tsg_gk20a *tsg = g->fifo.tsg + event_id_data->id;

	nvgpu_mutex_acquire(&tsg->event_id_list_lock);
	nvgpu_list_del(&event_id_data->event_id_node);
	nvgpu_mutex_release(&tsg->event_id_list_lock);

	nvgpu_mutex_destroy(&event_id_data->lock);
	gk20a_put(g);
	nvgpu_kfree(g, event_id_data);
	filp->private_data = NULL;

	return 0;
}

const struct file_operations gk20a_event_id_ops = {
	.owner = THIS_MODULE,
	.poll = gk20a_event_id_poll,
	.release = gk20a_event_id_release,
};

static int gk20a_tsg_event_id_enable(struct tsg_gk20a *tsg,
					 int event_id,
					 int *fd)
{
	int err = 0;
	int local_fd;
	struct file *file;
	char name[64];
	struct gk20a_event_id_data *event_id_data;
	struct gk20a *g;

	g = gk20a_get(tsg->g);
	if (!g)
		return -ENODEV;

	err = gk20a_tsg_get_event_data_from_id(tsg,
				event_id, &event_id_data);
	if (err == 0) {
		/* We already have event enabled */
		err = -EINVAL;
		goto free_ref;
	}

	err = get_unused_fd_flags(O_RDWR);
	if (err < 0)
		goto free_ref;
	local_fd = err;

	snprintf(name, sizeof(name), "nvgpu-event%d-fd%d",
		 event_id, local_fd);

	file = anon_inode_getfile(name, &gk20a_event_id_ops,
				  NULL, O_RDWR);
	if (IS_ERR(file)) {
		err = PTR_ERR(file);
		goto clean_up;
	}

	event_id_data = nvgpu_kzalloc(tsg->g, sizeof(*event_id_data));
	if (!event_id_data) {
		err = -ENOMEM;
		goto clean_up_file;
	}
	event_id_data->g = g;
	event_id_data->id = tsg->tsgid;
	event_id_data->event_id = event_id;

	nvgpu_cond_init(&event_id_data->event_id_wq);
	err = nvgpu_mutex_init(&event_id_data->lock);
	if (err)
		goto clean_up_free;

	nvgpu_init_list_node(&event_id_data->event_id_node);

	nvgpu_mutex_acquire(&tsg->event_id_list_lock);
	nvgpu_list_add_tail(&event_id_data->event_id_node, &tsg->event_id_list);
	nvgpu_mutex_release(&tsg->event_id_list_lock);

	fd_install(local_fd, file);
	file->private_data = event_id_data;

	*fd = local_fd;

	return 0;

clean_up_free:
	nvgpu_kfree(g, event_id_data);
clean_up_file:
	fput(file);
clean_up:
	put_unused_fd(local_fd);
free_ref:
	gk20a_put(g);
	return err;
}

static int gk20a_tsg_event_id_ctrl(struct gk20a *g, struct tsg_gk20a *tsg,
		struct nvgpu_event_id_ctrl_args *args)
{
	int err = 0;
	int fd = -1;

	if (args->event_id >= NVGPU_IOCTL_CHANNEL_EVENT_ID_MAX)
		return -EINVAL;

	switch (args->cmd) {
	case NVGPU_IOCTL_CHANNEL_EVENT_ID_CMD_ENABLE:
		err = gk20a_tsg_event_id_enable(tsg, args->event_id, &fd);
		if (!err)
			args->event_fd = fd;
		break;

	default:
		nvgpu_err(tsg->g, "unrecognized tsg event id cmd: 0x%x",
			   args->cmd);
		err = -EINVAL;
		break;
	}

	return err;
}

int nvgpu_ioctl_tsg_open(struct gk20a *g, struct file *filp)
{
	struct tsg_private *priv;
	struct tsg_gk20a *tsg;
	struct device *dev;
	int err;

	g = gk20a_get(g);
	if (!g)
		return -ENODEV;

	dev  = dev_from_gk20a(g);

	nvgpu_log(g, gpu_dbg_fn, "tsg: %s", dev_name(dev));

	priv = nvgpu_kmalloc(g, sizeof(*priv));
	if (!priv) {
		err = -ENOMEM;
		goto free_ref;
	}

	err = gk20a_busy(g);
	if (err) {
		nvgpu_err(g, "failed to power on, %d", err);
		goto free_mem;
	}

	tsg = gk20a_tsg_open(g, nvgpu_current_pid(g));
	gk20a_idle(g);
	if (!tsg) {
		err = -ENOMEM;
		goto free_mem;
	}

	priv->g = g;
	priv->tsg = tsg;
	filp->private_data = priv;

	gk20a_sched_ctrl_tsg_added(g, tsg);

	return 0;

free_mem:
	nvgpu_kfree(g, priv);
free_ref:
	gk20a_put(g);
	return err;
}

int nvgpu_ioctl_tsg_dev_open(struct inode *inode, struct file *filp)
{
	struct nvgpu_os_linux *l;
	struct gk20a *g;
	int ret;

	l = container_of(inode->i_cdev,
			 struct nvgpu_os_linux, tsg.cdev);
	g = &l->g;

	nvgpu_log_fn(g, " ");

	ret = gk20a_busy(g);
	if (ret) {
		nvgpu_err(g, "failed to power on, %d", ret);
		return ret;
	}

	ret = nvgpu_ioctl_tsg_open(&l->g, filp);

	gk20a_idle(g);
	nvgpu_log_fn(g, "done");
	return ret;
}

void nvgpu_ioctl_tsg_release(struct nvgpu_ref *ref)
{
	struct tsg_gk20a *tsg = container_of(ref, struct tsg_gk20a, refcount);
	struct gk20a *g = tsg->g;

	gk20a_sched_ctrl_tsg_removed(g, tsg);

	gk20a_tsg_release(ref);
	gk20a_put(g);
}

int nvgpu_ioctl_tsg_dev_release(struct inode *inode, struct file *filp)
{
	struct tsg_private *priv = filp->private_data;
	struct tsg_gk20a *tsg;

	if (!priv) {
		/* open failed, never got a tsg for this file */
		return 0;
	}

	tsg = priv->tsg;

	nvgpu_ref_put(&tsg->refcount, nvgpu_ioctl_tsg_release);
	nvgpu_kfree(tsg->g, priv);
	return 0;
}

static int gk20a_tsg_ioctl_set_runlist_interleave(struct gk20a *g,
	struct tsg_gk20a *tsg, struct nvgpu_runlist_interleave_args *arg)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct gk20a_sched_ctrl *sched = &l->sched_ctrl;
	u32 level = arg->level;
	int err;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_sched, "tsgid=%u", tsg->tsgid);

	nvgpu_mutex_acquire(&sched->control_lock);
	if (sched->control_locked) {
		err = -EPERM;
		goto done;
	}
	err = gk20a_busy(g);
	if (err) {
		nvgpu_err(g, "failed to power on gpu");
		goto done;
	}

	level = nvgpu_get_common_runlist_level(level);
	err = gk20a_tsg_set_runlist_interleave(tsg, level);

	gk20a_idle(g);
done:
	nvgpu_mutex_release(&sched->control_lock);
	return err;
}

static int gk20a_tsg_ioctl_set_timeslice(struct gk20a *g,
	struct tsg_gk20a *tsg, struct nvgpu_timeslice_args *arg)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct gk20a_sched_ctrl *sched = &l->sched_ctrl;
	int err;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_sched, "tsgid=%u", tsg->tsgid);

	nvgpu_mutex_acquire(&sched->control_lock);
	if (sched->control_locked) {
		err = -EPERM;
		goto done;
	}
	err = gk20a_busy(g);
	if (err) {
		nvgpu_err(g, "failed to power on gpu");
		goto done;
	}
	err = gk20a_tsg_set_timeslice(tsg, arg->timeslice_us);
	gk20a_idle(g);
done:
	nvgpu_mutex_release(&sched->control_lock);
	return err;
}

static int gk20a_tsg_ioctl_get_timeslice(struct gk20a *g,
	struct tsg_gk20a *tsg, struct nvgpu_timeslice_args *arg)
{
	arg->timeslice_us = gk20a_tsg_get_timeslice(tsg);
	return 0;
}

static int gk20a_tsg_ioctl_read_single_sm_error_state(struct gk20a *g,
		struct tsg_gk20a *tsg,
		struct nvgpu_tsg_read_single_sm_error_state_args *args)
{
	struct gr_gk20a *gr = &g->gr;
	struct nvgpu_tsg_sm_error_state *sm_error_state;
	struct nvgpu_tsg_sm_error_state_record sm_error_state_record;
	u32 sm_id;
	int err = 0;

	sm_id = args->sm_id;
	if (sm_id >= gr->no_of_sm)
		return -EINVAL;

	nvgpu_speculation_barrier();

	sm_error_state = tsg->sm_error_states + sm_id;
	sm_error_state_record.global_esr =
		sm_error_state->hww_global_esr;
	sm_error_state_record.warp_esr =
		sm_error_state->hww_warp_esr;
	sm_error_state_record.warp_esr_pc =
		sm_error_state->hww_warp_esr_pc;
	sm_error_state_record.global_esr_report_mask =
		sm_error_state->hww_global_esr_report_mask;
	sm_error_state_record.warp_esr_report_mask =
		sm_error_state->hww_warp_esr_report_mask;

	if (args->record_size > 0) {
		size_t write_size = sizeof(*sm_error_state);

		if (write_size > args->record_size)
			write_size = args->record_size;

		nvgpu_mutex_acquire(&g->dbg_sessions_lock);
		err = copy_to_user((void __user *)(uintptr_t)
						args->record_mem,
				   &sm_error_state_record,
				   write_size);
		nvgpu_mutex_release(&g->dbg_sessions_lock);
		if (err) {
			nvgpu_err(g, "copy_to_user failed!");
			return err;
		}

		args->record_size = write_size;
	}

	return 0;
}

long nvgpu_ioctl_tsg_dev_ioctl(struct file *filp, unsigned int cmd,
			     unsigned long arg)
{
	struct tsg_private *priv = filp->private_data;
	struct tsg_gk20a *tsg = priv->tsg;
	struct gk20a *g = tsg->g;
	u8 __maybe_unused buf[NVGPU_TSG_IOCTL_MAX_ARG_SIZE];
	int err = 0;

	nvgpu_log_fn(g, "start %d", _IOC_NR(cmd));

	if ((_IOC_TYPE(cmd) != NVGPU_TSG_IOCTL_MAGIC) ||
	    (_IOC_NR(cmd) == 0) ||
	    (_IOC_NR(cmd) > NVGPU_TSG_IOCTL_LAST) ||
	    (_IOC_SIZE(cmd) > NVGPU_TSG_IOCTL_MAX_ARG_SIZE))
		return -EINVAL;

	memset(buf, 0, sizeof(buf));
	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(buf, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	if (!g->sw_ready) {
		err = gk20a_busy(g);
		if (err)
			return err;

		gk20a_idle(g);
	}

	switch (cmd) {
	case NVGPU_TSG_IOCTL_BIND_CHANNEL:
		{
		int ch_fd = *(int *)buf;
		if (ch_fd < 0) {
			err = -EINVAL;
			break;
		}
		err = gk20a_tsg_bind_channel_fd(tsg, ch_fd);
		break;
		}

	case NVGPU_TSG_IOCTL_BIND_CHANNEL_EX:
	{
		err = gk20a_tsg_ioctl_bind_channel_ex(g, tsg,
			(struct nvgpu_tsg_bind_channel_ex_args *)buf);
		break;
	}

	case NVGPU_TSG_IOCTL_UNBIND_CHANNEL:
		{
		int ch_fd = *(int *)buf;

		if (ch_fd < 0) {
			err = -EINVAL;
			break;
		}
		err = gk20a_busy(g);
		if (err) {
			nvgpu_err(g,
			   "failed to host gk20a for ioctl cmd: 0x%x", cmd);
			break;
		}
		err = gk20a_tsg_unbind_channel_fd(tsg, ch_fd);
		gk20a_idle(g);
		break;
		}

	case NVGPU_IOCTL_TSG_ENABLE:
		{
		err = gk20a_busy(g);
		if (err) {
			nvgpu_err(g,
			   "failed to host gk20a for ioctl cmd: 0x%x", cmd);
			return err;
		}
		g->ops.fifo.enable_tsg(tsg);
		gk20a_idle(g);
		break;
		}

	case NVGPU_IOCTL_TSG_DISABLE:
		{
		err = gk20a_busy(g);
		if (err) {
			nvgpu_err(g,
			   "failed to host gk20a for ioctl cmd: 0x%x", cmd);
			return err;
		}
		g->ops.fifo.disable_tsg(tsg);
		gk20a_idle(g);
		break;
		}

	case NVGPU_IOCTL_TSG_PREEMPT:
		{
		err = gk20a_busy(g);
		if (err) {
			nvgpu_err(g,
			   "failed to host gk20a for ioctl cmd: 0x%x", cmd);
			return err;
		}
		/* preempt TSG */
		err = g->ops.fifo.preempt_tsg(g, tsg);
		gk20a_idle(g);
		break;
		}

	case NVGPU_IOCTL_TSG_EVENT_ID_CTRL:
		{
		err = gk20a_tsg_event_id_ctrl(g, tsg,
			(struct nvgpu_event_id_ctrl_args *)buf);
		break;
		}

	case NVGPU_IOCTL_TSG_SET_RUNLIST_INTERLEAVE:
		err = gk20a_tsg_ioctl_set_runlist_interleave(g, tsg,
			(struct nvgpu_runlist_interleave_args *)buf);
		break;

	case NVGPU_IOCTL_TSG_SET_TIMESLICE:
		{
		err = gk20a_tsg_ioctl_set_timeslice(g, tsg,
			(struct nvgpu_timeslice_args *)buf);
		break;
		}
	case NVGPU_IOCTL_TSG_GET_TIMESLICE:
		{
		err = gk20a_tsg_ioctl_get_timeslice(g, tsg,
			(struct nvgpu_timeslice_args *)buf);
		break;
		}

	case NVGPU_TSG_IOCTL_READ_SINGLE_SM_ERROR_STATE:
		{
		err = gk20a_tsg_ioctl_read_single_sm_error_state(g, tsg,
			(struct nvgpu_tsg_read_single_sm_error_state_args *)buf);
		break;
		}

	default:
		nvgpu_err(g, "unrecognized tsg gpu ioctl cmd: 0x%x",
			   cmd);
		err = -ENOTTY;
		break;
	}

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *)arg,
				   buf, _IOC_SIZE(cmd));

	return err;
}
