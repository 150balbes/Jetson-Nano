/*
 * Copyright (c) 2016-2020, NVIDIA CORPORATION.  All rights reserved.
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
#include <asm/barrier.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <uapi/linux/nvgpu.h>

#include <nvgpu/kmem.h>
#include <nvgpu/log.h>
#include <nvgpu/bug.h>
#include <nvgpu/barrier.h>
#include <nvgpu/gk20a.h>

#include "gk20a/gr_gk20a.h"
#include "sched.h"
#include "os_linux.h"
#include "ioctl_tsg.h"

#include <nvgpu/hw/gk20a/hw_ctxsw_prog_gk20a.h>
#include <nvgpu/hw/gk20a/hw_gr_gk20a.h>

ssize_t gk20a_sched_dev_read(struct file *filp, char __user *buf,
	size_t size, loff_t *off)
{
	struct gk20a *g = filp->private_data;
	struct nvgpu_sched_ctrl *sched = &g->sched_ctrl;
	struct nvgpu_sched_event_arg event = { 0 };
	int err;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_sched,
		"filp=%p buf=%p size=%zu", filp, buf, size);

	if (size < sizeof(event))
		return -EINVAL;
	size = sizeof(event);

	nvgpu_mutex_acquire(&sched->status_lock);
	while (!sched->status) {
		nvgpu_mutex_release(&sched->status_lock);
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		err = NVGPU_COND_WAIT_INTERRUPTIBLE(&sched->readout_wq,
			sched->status, 0);
		if (err)
			return err;
		nvgpu_mutex_acquire(&sched->status_lock);
	}

	event.reserved = 0;
	event.status = sched->status;

	if (copy_to_user(buf, &event, size)) {
		nvgpu_mutex_release(&sched->status_lock);
		return -EFAULT;
	}

	sched->status = 0;

	nvgpu_mutex_release(&sched->status_lock);

	return size;
}

unsigned int gk20a_sched_dev_poll(struct file *filp, poll_table *wait)
{
	struct gk20a *g = filp->private_data;
	struct nvgpu_sched_ctrl *sched = &g->sched_ctrl;
	unsigned int mask = 0;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_sched, " ");

	nvgpu_mutex_acquire(&sched->status_lock);
	poll_wait(filp, &sched->readout_wq.wq, wait);
	if (sched->status)
		mask |= POLLIN | POLLRDNORM;
	nvgpu_mutex_release(&sched->status_lock);

	return mask;
}

static int gk20a_sched_dev_ioctl_get_tsgs(struct gk20a *g,
	struct nvgpu_sched_get_tsgs_args *arg)
{
	struct nvgpu_sched_ctrl *sched = &g->sched_ctrl;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_sched, "size=%u buffer=%llx",
			arg->size, arg->buffer);

	if ((arg->size < sched->bitmap_size) || (!arg->buffer)) {
		arg->size = sched->bitmap_size;
		return -ENOSPC;
	}

	nvgpu_mutex_acquire(&sched->status_lock);
	if (copy_to_user((void __user *)(uintptr_t)arg->buffer,
		sched->active_tsg_bitmap, sched->bitmap_size)) {
		nvgpu_mutex_release(&sched->status_lock);
		return -EFAULT;
	}
	nvgpu_mutex_release(&sched->status_lock);

	return 0;
}

static int gk20a_sched_dev_ioctl_get_recent_tsgs(struct gk20a *g,
	struct nvgpu_sched_get_tsgs_args *arg)
{
	struct nvgpu_sched_ctrl *sched = &g->sched_ctrl;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_sched, "size=%u buffer=%llx",
			arg->size, arg->buffer);

	if ((arg->size < sched->bitmap_size) || (!arg->buffer)) {
		arg->size = sched->bitmap_size;
		return -ENOSPC;
	}

	nvgpu_mutex_acquire(&sched->status_lock);
	if (copy_to_user((void __user *)(uintptr_t)arg->buffer,
		sched->recent_tsg_bitmap, sched->bitmap_size)) {
		nvgpu_mutex_release(&sched->status_lock);
		return -EFAULT;
	}

	memset(sched->recent_tsg_bitmap, 0, sched->bitmap_size);
	nvgpu_mutex_release(&sched->status_lock);

	return 0;
}

static int gk20a_sched_dev_ioctl_get_tsgs_by_pid(struct gk20a *g,
	struct nvgpu_sched_get_tsgs_by_pid_args *arg)
{
	struct nvgpu_sched_ctrl *sched = &g->sched_ctrl;
	struct fifo_gk20a *f = &g->fifo;
	struct tsg_gk20a *tsg;
	u64 *bitmap;
	unsigned int tsgid;
	/* pid at user level corresponds to kernel tgid */
	pid_t tgid = (pid_t)arg->pid;
	int err = 0;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_sched, "pid=%d size=%u buffer=%llx",
			(pid_t)arg->pid, arg->size, arg->buffer);

	if ((arg->size < sched->bitmap_size) || (!arg->buffer)) {
		arg->size = sched->bitmap_size;
		return -ENOSPC;
	}

	bitmap = nvgpu_kzalloc(g, sched->bitmap_size);
	if (!bitmap)
		return -ENOMEM;

	nvgpu_mutex_acquire(&sched->status_lock);
	for (tsgid = 0; tsgid < f->num_channels; tsgid++) {
		if (NVGPU_SCHED_ISSET(tsgid, sched->active_tsg_bitmap)) {
			tsg = &f->tsg[tsgid];
			if (tsg->tgid == tgid)
				NVGPU_SCHED_SET(tsgid, bitmap);
		}
	}
	nvgpu_mutex_release(&sched->status_lock);

	if (copy_to_user((void __user *)(uintptr_t)arg->buffer,
		bitmap, sched->bitmap_size))
		err = -EFAULT;

	nvgpu_kfree(g, bitmap);

	return err;
}

static int gk20a_sched_dev_ioctl_get_params(struct gk20a *g,
	struct nvgpu_sched_tsg_get_params_args *arg)
{
	struct fifo_gk20a *f = &g->fifo;
	struct tsg_gk20a *tsg;
	u32 tsgid = arg->tsgid;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_sched, "tsgid=%u", tsgid);

	if (tsgid >= f->num_channels)
		return -EINVAL;

	nvgpu_speculation_barrier();

	tsg = &f->tsg[tsgid];
	if (!nvgpu_ref_get_unless_zero(&tsg->refcount))
		return -ENXIO;

	arg->pid = tsg->tgid;	/* kernel tgid corresponds to user pid */
	arg->runlist_interleave = tsg->interleave_level;
	arg->timeslice = gk20a_tsg_get_timeslice(tsg);

	arg->graphics_preempt_mode =
		tsg->gr_ctx.graphics_preempt_mode;
	arg->compute_preempt_mode =
		tsg->gr_ctx.compute_preempt_mode;

	nvgpu_ref_put(&tsg->refcount, nvgpu_ioctl_tsg_release);

	return 0;
}

static int gk20a_sched_dev_ioctl_tsg_set_timeslice(
	struct gk20a *g,
	struct nvgpu_sched_tsg_timeslice_args *arg)
{
	struct fifo_gk20a *f = &g->fifo;
	struct tsg_gk20a *tsg;
	u32 tsgid = arg->tsgid;
	int err;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_sched, "tsgid=%u", tsgid);

	if (tsgid >= f->num_channels)
		return -EINVAL;

	nvgpu_speculation_barrier();

	tsg = &f->tsg[tsgid];
	if (!nvgpu_ref_get_unless_zero(&tsg->refcount))
		return -ENXIO;

	err = gk20a_busy(g);
	if (err)
		goto done;

	err = gk20a_tsg_set_timeslice(tsg, arg->timeslice);

	gk20a_idle(g);

done:
	nvgpu_ref_put(&tsg->refcount, nvgpu_ioctl_tsg_release);

	return err;
}

static int gk20a_sched_dev_ioctl_tsg_set_runlist_interleave(
	struct gk20a *g,
	struct nvgpu_sched_tsg_runlist_interleave_args *arg)
{
	struct fifo_gk20a *f = &g->fifo;
	struct tsg_gk20a *tsg;
	u32 tsgid = arg->tsgid;
	int err;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_sched, "tsgid=%u", tsgid);

	if (tsgid >= f->num_channels)
		return -EINVAL;

	nvgpu_speculation_barrier();

	tsg = &f->tsg[tsgid];
	if (!nvgpu_ref_get_unless_zero(&tsg->refcount))
		return -ENXIO;

	err = gk20a_busy(g);
	if (err)
		goto done;

	err = gk20a_tsg_set_runlist_interleave(tsg, arg->runlist_interleave);

	gk20a_idle(g);

done:
	nvgpu_ref_put(&tsg->refcount, nvgpu_ioctl_tsg_release);

	return err;
}

static int gk20a_sched_dev_ioctl_lock_control(struct gk20a *g)
{
	struct nvgpu_sched_ctrl *sched = &g->sched_ctrl;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_sched, " ");

	nvgpu_mutex_acquire(&sched->control_lock);
	sched->control_locked = true;
	nvgpu_mutex_release(&sched->control_lock);
	return 0;
}

static int gk20a_sched_dev_ioctl_unlock_control(struct gk20a *g)
{
	struct nvgpu_sched_ctrl *sched = &g->sched_ctrl;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_sched, " ");

	nvgpu_mutex_acquire(&sched->control_lock);
	sched->control_locked = false;
	nvgpu_mutex_release(&sched->control_lock);
	return 0;
}

static int gk20a_sched_dev_ioctl_get_api_version(struct gk20a *g,
	struct nvgpu_sched_api_version_args *args)
{
	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_sched, " ");

	args->version = NVGPU_SCHED_API_VERSION;
	return 0;
}

static int gk20a_sched_dev_ioctl_get_tsg(struct gk20a *g,
	struct nvgpu_sched_tsg_refcount_args *arg)
{
	struct nvgpu_sched_ctrl *sched = &g->sched_ctrl;
	struct fifo_gk20a *f = &g->fifo;
	struct tsg_gk20a *tsg;
	u32 tsgid = arg->tsgid;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_sched, "tsgid=%u", tsgid);

	if (tsgid >= f->num_channels)
		return -EINVAL;

	nvgpu_speculation_barrier();

	tsg = &f->tsg[tsgid];
	if (!nvgpu_ref_get_unless_zero(&tsg->refcount))
		return -ENXIO;

	nvgpu_mutex_acquire(&sched->status_lock);
	if (NVGPU_SCHED_ISSET(tsgid, sched->ref_tsg_bitmap)) {
		nvgpu_warn(g, "tsgid=%d already referenced", tsgid);
		/* unlock status_lock as nvgpu_ioctl_tsg_release locks it */
		nvgpu_mutex_release(&sched->status_lock);
		nvgpu_ref_put(&tsg->refcount, nvgpu_ioctl_tsg_release);
		return -ENXIO;
	}

	/* keep reference on TSG, will be released on
	 * NVGPU_SCHED_IOCTL_PUT_TSG ioctl, or close
	 */
	NVGPU_SCHED_SET(tsgid, sched->ref_tsg_bitmap);
	nvgpu_mutex_release(&sched->status_lock);

	return 0;
}

static int gk20a_sched_dev_ioctl_put_tsg(struct gk20a *g,
	struct nvgpu_sched_tsg_refcount_args *arg)
{
	struct nvgpu_sched_ctrl *sched = &g->sched_ctrl;
	struct fifo_gk20a *f = &g->fifo;
	struct tsg_gk20a *tsg;
	u32 tsgid = arg->tsgid;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_sched, "tsgid=%u", tsgid);

	if (tsgid >= f->num_channels)
		return -EINVAL;

	nvgpu_speculation_barrier();

	nvgpu_mutex_acquire(&sched->status_lock);
	if (!NVGPU_SCHED_ISSET(tsgid, sched->ref_tsg_bitmap)) {
		nvgpu_mutex_release(&sched->status_lock);
		nvgpu_warn(g, "tsgid=%d not previously referenced", tsgid);
		return -ENXIO;
	}
	NVGPU_SCHED_CLR(tsgid, sched->ref_tsg_bitmap);
	nvgpu_mutex_release(&sched->status_lock);

	tsg = &f->tsg[tsgid];
	nvgpu_ref_put(&tsg->refcount, nvgpu_ioctl_tsg_release);

	return 0;
}

int gk20a_sched_dev_open(struct inode *inode, struct file *filp)
{
	struct nvgpu_os_linux *l = container_of(inode->i_cdev,
				struct nvgpu_os_linux, sched.cdev);
	struct gk20a *g;
	struct nvgpu_sched_ctrl *sched;
	int err = 0;

	g = gk20a_get(&l->g);
	if (!g)
		return -ENODEV;
	sched = &g->sched_ctrl;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_sched, "g=%p", g);

	if (!sched->sw_ready) {
		err = gk20a_busy(g);
		if (err)
			goto free_ref;

		gk20a_idle(g);
	}

	if (!nvgpu_mutex_tryacquire(&sched->busy_lock)) {
		err = -EBUSY;
		goto free_ref;
	}

	memcpy(sched->recent_tsg_bitmap, sched->active_tsg_bitmap,
			sched->bitmap_size);
	memset(sched->ref_tsg_bitmap, 0, sched->bitmap_size);

	filp->private_data = g;
	nvgpu_log(g, gpu_dbg_sched, "filp=%p sched=%p", filp, sched);

free_ref:
	if (err)
		gk20a_put(g);
	return err;
}

long gk20a_sched_dev_ioctl(struct file *filp, unsigned int cmd,
	unsigned long arg)
{
	struct gk20a *g = filp->private_data;
	u8 buf[NVGPU_CTXSW_IOCTL_MAX_ARG_SIZE];
	int err = 0;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_sched, "nr=%d", _IOC_NR(cmd));

	if ((_IOC_TYPE(cmd) != NVGPU_SCHED_IOCTL_MAGIC) ||
		(_IOC_NR(cmd) == 0) ||
		(_IOC_NR(cmd) > NVGPU_SCHED_IOCTL_LAST) ||
		(_IOC_SIZE(cmd) > NVGPU_SCHED_IOCTL_MAX_ARG_SIZE))
		return -EINVAL;

	memset(buf, 0, sizeof(buf));
	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(buf, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	nvgpu_speculation_barrier();
	switch (cmd) {
	case NVGPU_SCHED_IOCTL_GET_TSGS:
		err = gk20a_sched_dev_ioctl_get_tsgs(g,
			(struct nvgpu_sched_get_tsgs_args *)buf);
		break;
	case NVGPU_SCHED_IOCTL_GET_RECENT_TSGS:
		err = gk20a_sched_dev_ioctl_get_recent_tsgs(g,
			(struct nvgpu_sched_get_tsgs_args *)buf);
		break;
	case NVGPU_SCHED_IOCTL_GET_TSGS_BY_PID:
		err = gk20a_sched_dev_ioctl_get_tsgs_by_pid(g,
			(struct nvgpu_sched_get_tsgs_by_pid_args *)buf);
		break;
	case NVGPU_SCHED_IOCTL_TSG_GET_PARAMS:
		err = gk20a_sched_dev_ioctl_get_params(g,
			(struct nvgpu_sched_tsg_get_params_args *)buf);
		break;
	case NVGPU_SCHED_IOCTL_TSG_SET_TIMESLICE:
		err = gk20a_sched_dev_ioctl_tsg_set_timeslice(g,
			(struct nvgpu_sched_tsg_timeslice_args *)buf);
		break;
	case NVGPU_SCHED_IOCTL_TSG_SET_RUNLIST_INTERLEAVE:
		err = gk20a_sched_dev_ioctl_tsg_set_runlist_interleave(g,
			(struct nvgpu_sched_tsg_runlist_interleave_args *)buf);
		break;
	case NVGPU_SCHED_IOCTL_LOCK_CONTROL:
		err = gk20a_sched_dev_ioctl_lock_control(g);
		break;
	case NVGPU_SCHED_IOCTL_UNLOCK_CONTROL:
		err = gk20a_sched_dev_ioctl_unlock_control(g);
		break;
	case NVGPU_SCHED_IOCTL_GET_API_VERSION:
		err = gk20a_sched_dev_ioctl_get_api_version(g,
			(struct nvgpu_sched_api_version_args *)buf);
		break;
	case NVGPU_SCHED_IOCTL_GET_TSG:
		err = gk20a_sched_dev_ioctl_get_tsg(g,
			(struct nvgpu_sched_tsg_refcount_args *)buf);
		break;
	case NVGPU_SCHED_IOCTL_PUT_TSG:
		err = gk20a_sched_dev_ioctl_put_tsg(g,
			(struct nvgpu_sched_tsg_refcount_args *)buf);
		break;
	default:
		nvgpu_log_info(g, "unrecognized gpu ioctl cmd: 0x%x", cmd);
		err = -ENOTTY;
	}

	/* Some ioctls like NVGPU_SCHED_IOCTL_GET_TSGS might be called on
	 * purpose with NULL buffer and/or zero size to discover TSG bitmap
	 * size. We need to update user arguments in this case too, even
	 * if we return an error.
	 */
	if ((!err || (err == -ENOSPC)) && (_IOC_DIR(cmd) & _IOC_READ)) {
		if (copy_to_user((void __user *)arg, buf, _IOC_SIZE(cmd)))
			err = -EFAULT;
	}

	return err;
}

int gk20a_sched_dev_release(struct inode *inode, struct file *filp)
{
	struct gk20a *g = filp->private_data;
	struct nvgpu_sched_ctrl *sched = &g->sched_ctrl;
	struct fifo_gk20a *f = &g->fifo;
	struct tsg_gk20a *tsg;
	unsigned int tsgid;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_sched, "sched: %p", sched);

	/* release any reference to TSGs */
	for (tsgid = 0; tsgid < f->num_channels; tsgid++) {
		if (NVGPU_SCHED_ISSET(tsgid, sched->ref_tsg_bitmap)) {
			tsg = &f->tsg[tsgid];
			nvgpu_ref_put(&tsg->refcount, nvgpu_ioctl_tsg_release);
		}
	}

	/* unlock control */
	nvgpu_mutex_acquire(&sched->control_lock);
	sched->control_locked = false;
	nvgpu_mutex_release(&sched->control_lock);

	nvgpu_mutex_release(&sched->busy_lock);
	gk20a_put(g);
	return 0;
}

void gk20a_sched_ctrl_tsg_added(struct gk20a *g, struct tsg_gk20a *tsg)
{
	struct nvgpu_sched_ctrl *sched = &g->sched_ctrl;
	int err;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_sched, "tsgid=%u", tsg->tsgid);

	if (!sched->sw_ready) {
		err = gk20a_busy(g);
		if (err) {
			WARN_ON(err);
			return;
		}

		gk20a_idle(g);
	}

	nvgpu_mutex_acquire(&sched->status_lock);
	NVGPU_SCHED_SET(tsg->tsgid, sched->active_tsg_bitmap);
	NVGPU_SCHED_SET(tsg->tsgid, sched->recent_tsg_bitmap);
	sched->status |= NVGPU_SCHED_STATUS_TSG_OPEN;
	nvgpu_mutex_release(&sched->status_lock);
	nvgpu_cond_signal_interruptible(&sched->readout_wq);
}

void gk20a_sched_ctrl_tsg_removed(struct gk20a *g, struct tsg_gk20a *tsg)
{
	struct nvgpu_sched_ctrl *sched = &g->sched_ctrl;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_sched, "tsgid=%u", tsg->tsgid);

	nvgpu_mutex_acquire(&sched->status_lock);
	NVGPU_SCHED_CLR(tsg->tsgid, sched->active_tsg_bitmap);

	/* clear recent_tsg_bitmap as well: if app manager did not
	 * notice that TSG was previously added, no need to notify it
	 * if the TSG has been released in the meantime. If the
	 * TSG gets reallocated, app manager will be notified as usual.
	 */
	NVGPU_SCHED_CLR(tsg->tsgid, sched->recent_tsg_bitmap);

	/* do not set event_pending, we only want to notify app manager
	 * when TSGs are added, so that it can apply sched params
	 */
	nvgpu_mutex_release(&sched->status_lock);
}

int gk20a_sched_ctrl_init(struct gk20a *g)
{
	struct nvgpu_sched_ctrl *sched = &g->sched_ctrl;
	struct fifo_gk20a *f = &g->fifo;
	int err;

	if (sched->sw_ready)
		return 0;

	sched->bitmap_size = roundup(f->num_channels, 64) / 8;
	sched->status = 0;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_sched, "g=%p sched=%p size=%zu",
			g, sched, sched->bitmap_size);

	sched->active_tsg_bitmap = nvgpu_kzalloc(g, sched->bitmap_size);
	if (!sched->active_tsg_bitmap)
		return -ENOMEM;

	sched->recent_tsg_bitmap = nvgpu_kzalloc(g, sched->bitmap_size);
	if (!sched->recent_tsg_bitmap) {
		err = -ENOMEM;
		goto free_active;
	}

	sched->ref_tsg_bitmap = nvgpu_kzalloc(g, sched->bitmap_size);
	if (!sched->ref_tsg_bitmap) {
		err = -ENOMEM;
		goto free_recent;
	}

	nvgpu_cond_init(&sched->readout_wq);

	err = nvgpu_mutex_init(&sched->status_lock);
	if (err)
		goto free_ref;

	err = nvgpu_mutex_init(&sched->control_lock);
	if (err)
		goto free_status_lock;

	err = nvgpu_mutex_init(&sched->busy_lock);
	if (err)
		goto free_control_lock;

	sched->sw_ready = true;

	return 0;

free_control_lock:
	nvgpu_mutex_destroy(&sched->control_lock);
free_status_lock:
	nvgpu_mutex_destroy(&sched->status_lock);
free_ref:
	nvgpu_kfree(g, sched->ref_tsg_bitmap);
free_recent:
	nvgpu_kfree(g, sched->recent_tsg_bitmap);
free_active:
	nvgpu_kfree(g, sched->active_tsg_bitmap);

	return err;
}

void gk20a_sched_ctrl_cleanup(struct gk20a *g)
{
	struct nvgpu_sched_ctrl *sched = &g->sched_ctrl;

	nvgpu_kfree(g, sched->active_tsg_bitmap);
	nvgpu_kfree(g, sched->recent_tsg_bitmap);
	nvgpu_kfree(g, sched->ref_tsg_bitmap);
	sched->active_tsg_bitmap = NULL;
	sched->recent_tsg_bitmap = NULL;
	sched->ref_tsg_bitmap = NULL;

	nvgpu_mutex_destroy(&sched->status_lock);
	nvgpu_mutex_destroy(&sched->control_lock);
	nvgpu_mutex_destroy(&sched->busy_lock);

	sched->sw_ready = false;
}
