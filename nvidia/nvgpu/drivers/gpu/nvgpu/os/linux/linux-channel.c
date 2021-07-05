/*
 * Copyright (c) 2017-2018, NVIDIA Corporation.  All rights reserved.
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

#include <nvgpu/enabled.h>
#include <nvgpu/debug.h>
#include <nvgpu/error_notifier.h>
#include <nvgpu/os_sched.h>
#include <nvgpu/gk20a.h>
#include <nvgpu/channel.h>
#include <nvgpu/dma.h>

/*
 * This is required for nvgpu_vm_find_buf() which is used in the tracing
 * code. Once we can get and access userspace buffers without requiring
 * direct dma_buf usage this can be removed.
 */
#include <nvgpu/linux/vm.h>

#include "channel.h"
#include "ioctl_channel.h"
#include "os_linux.h"
#include "dmabuf.h"

#include <nvgpu/hw/gk20a/hw_pbdma_gk20a.h>

#include <linux/uaccess.h>
#include <linux/dma-buf.h>
#include <trace/events/gk20a.h>
#include <uapi/linux/nvgpu.h>

#include "sync_sema_android.h"

u32 nvgpu_submit_gpfifo_user_flags_to_common_flags(u32 user_flags)
{
	u32 flags = 0;

	if (user_flags & NVGPU_SUBMIT_GPFIFO_FLAGS_FENCE_WAIT)
		flags |= NVGPU_SUBMIT_FLAGS_FENCE_WAIT;

	if (user_flags & NVGPU_SUBMIT_GPFIFO_FLAGS_FENCE_GET)
		flags |= NVGPU_SUBMIT_FLAGS_FENCE_GET;

	if (user_flags & NVGPU_SUBMIT_GPFIFO_FLAGS_HW_FORMAT)
		flags |= NVGPU_SUBMIT_FLAGS_HW_FORMAT;

	if (user_flags & NVGPU_SUBMIT_GPFIFO_FLAGS_SYNC_FENCE)
		flags |= NVGPU_SUBMIT_FLAGS_SYNC_FENCE;

	if (user_flags & NVGPU_SUBMIT_GPFIFO_FLAGS_SUPPRESS_WFI)
		flags |= NVGPU_SUBMIT_FLAGS_SUPPRESS_WFI;

	if (user_flags & NVGPU_SUBMIT_GPFIFO_FLAGS_SKIP_BUFFER_REFCOUNTING)
		flags |= NVGPU_SUBMIT_FLAGS_SKIP_BUFFER_REFCOUNTING;

	return flags;
}

/*
 * API to convert error_notifiers in common code and of the form
 * NVGPU_ERR_NOTIFIER_* into Linux specific error_notifiers exposed to user
 * space and of the form  NVGPU_CHANNEL_*
 */
static u32 nvgpu_error_notifier_to_channel_notifier(u32 error_notifier)
{
	switch (error_notifier) {
	case NVGPU_ERR_NOTIFIER_FIFO_ERROR_IDLE_TIMEOUT:
		return NVGPU_CHANNEL_FIFO_ERROR_IDLE_TIMEOUT;
	case NVGPU_ERR_NOTIFIER_GR_ERROR_SW_METHOD:
		return NVGPU_CHANNEL_GR_ERROR_SW_METHOD;
	case NVGPU_ERR_NOTIFIER_GR_ERROR_SW_NOTIFY:
		return NVGPU_CHANNEL_GR_ERROR_SW_NOTIFY;
	case NVGPU_ERR_NOTIFIER_GR_EXCEPTION:
		return NVGPU_CHANNEL_GR_EXCEPTION;
	case NVGPU_ERR_NOTIFIER_GR_SEMAPHORE_TIMEOUT:
		return NVGPU_CHANNEL_GR_SEMAPHORE_TIMEOUT;
	case NVGPU_ERR_NOTIFIER_GR_ILLEGAL_NOTIFY:
		return NVGPU_CHANNEL_GR_ILLEGAL_NOTIFY;
	case NVGPU_ERR_NOTIFIER_FIFO_ERROR_MMU_ERR_FLT:
		return NVGPU_CHANNEL_FIFO_ERROR_MMU_ERR_FLT;
	case NVGPU_ERR_NOTIFIER_PBDMA_ERROR:
		return NVGPU_CHANNEL_PBDMA_ERROR;
	case NVGPU_ERR_NOTIFIER_FECS_ERR_UNIMP_FIRMWARE_METHOD:
		return NVGPU_CHANNEL_FECS_ERR_UNIMP_FIRMWARE_METHOD;
	case NVGPU_ERR_NOTIFIER_RESETCHANNEL_VERIF_ERROR:
		return NVGPU_CHANNEL_RESETCHANNEL_VERIF_ERROR;
	case NVGPU_ERR_NOTIFIER_PBDMA_PUSHBUFFER_CRC_MISMATCH:
		return NVGPU_CHANNEL_PBDMA_PUSHBUFFER_CRC_MISMATCH;
	}

	pr_warn("%s: invalid error_notifier requested %u\n", __func__, error_notifier);

	return error_notifier;
}

/**
 * nvgpu_set_error_notifier_locked()
 * Should be called with ch->error_notifier_mutex held
 *
 * error should be of the form  NVGPU_ERR_NOTIFIER_*
 */
void nvgpu_set_error_notifier_locked(struct channel_gk20a *ch, u32 error)
{
	struct nvgpu_channel_linux *priv = ch->os_priv;

	error = nvgpu_error_notifier_to_channel_notifier(error);

	if (priv->error_notifier.dmabuf) {
		struct nvgpu_notification *notification =
			priv->error_notifier.notification;
		struct timespec time_data;
		u64 nsec;

		getnstimeofday(&time_data);
		nsec = ((u64)time_data.tv_sec) * 1000000000u +
				(u64)time_data.tv_nsec;
		notification->time_stamp.nanoseconds[0] =
				(u32)nsec;
		notification->time_stamp.nanoseconds[1] =
				(u32)(nsec >> 32);
		notification->info32 = error;
		notification->status = 0xffff;

		nvgpu_err(ch->g,
		    "error notifier set to %d for ch %d", error, ch->chid);
	}
}

/* error should be of the form  NVGPU_ERR_NOTIFIER_* */
void nvgpu_set_error_notifier(struct channel_gk20a *ch, u32 error)
{
	struct nvgpu_channel_linux *priv = ch->os_priv;

	nvgpu_mutex_acquire(&priv->error_notifier.mutex);
	nvgpu_set_error_notifier_locked(ch, error);
	nvgpu_mutex_release(&priv->error_notifier.mutex);
}

void nvgpu_set_error_notifier_if_empty(struct channel_gk20a *ch, u32 error)
{
	struct nvgpu_channel_linux *priv = ch->os_priv;

	nvgpu_mutex_acquire(&priv->error_notifier.mutex);
	if (priv->error_notifier.dmabuf) {
		struct nvgpu_notification *notification =
			priv->error_notifier.notification;

		/* Don't overwrite error flag if it is already set */
		if (notification->status != 0xffff)
			nvgpu_set_error_notifier_locked(ch, error);
	}
	nvgpu_mutex_release(&priv->error_notifier.mutex);
}

/* error_notifier should be of the form  NVGPU_ERR_NOTIFIER_* */
bool nvgpu_is_error_notifier_set(struct channel_gk20a *ch, u32 error_notifier)
{
	struct nvgpu_channel_linux *priv = ch->os_priv;
	bool notifier_set = false;

	error_notifier = nvgpu_error_notifier_to_channel_notifier(error_notifier);

	nvgpu_mutex_acquire(&priv->error_notifier.mutex);
	if (priv->error_notifier.dmabuf) {
		struct nvgpu_notification *notification =
			priv->error_notifier.notification;
		u32 err = notification->info32;

		if (err == error_notifier)
			notifier_set = true;
	}
	nvgpu_mutex_release(&priv->error_notifier.mutex);

	return notifier_set;
}

static void gk20a_channel_update_runcb_fn(struct work_struct *work)
{
	struct nvgpu_channel_completion_cb *completion_cb =
		container_of(work, struct nvgpu_channel_completion_cb, work);
	struct nvgpu_channel_linux *priv =
		container_of(completion_cb,
				struct nvgpu_channel_linux, completion_cb);
	struct channel_gk20a *ch = priv->ch;
	void (*fn)(struct channel_gk20a *, void *);
	void *user_data;

	nvgpu_spinlock_acquire(&completion_cb->lock);
	fn = completion_cb->fn;
	user_data = completion_cb->user_data;
	nvgpu_spinlock_release(&completion_cb->lock);

	if (fn)
		fn(ch, user_data);
}

static void nvgpu_channel_work_completion_init(struct channel_gk20a *ch)
{
	struct nvgpu_channel_linux *priv = ch->os_priv;

	priv->completion_cb.fn = NULL;
	priv->completion_cb.user_data = NULL;
	nvgpu_spinlock_init(&priv->completion_cb.lock);
	INIT_WORK(&priv->completion_cb.work, gk20a_channel_update_runcb_fn);
}

static void nvgpu_channel_work_completion_clear(struct channel_gk20a *ch)
{
	struct nvgpu_channel_linux *priv = ch->os_priv;

	nvgpu_spinlock_acquire(&priv->completion_cb.lock);
	priv->completion_cb.fn = NULL;
	priv->completion_cb.user_data = NULL;
	nvgpu_spinlock_release(&priv->completion_cb.lock);
	cancel_work_sync(&priv->completion_cb.work);
}

static void nvgpu_channel_work_completion_signal(struct channel_gk20a *ch)
{
	struct nvgpu_channel_linux *priv = ch->os_priv;

	if (priv->completion_cb.fn)
		schedule_work(&priv->completion_cb.work);
}

static void nvgpu_channel_work_completion_cancel_sync(struct channel_gk20a *ch)
{
	struct nvgpu_channel_linux *priv = ch->os_priv;

	if (priv->completion_cb.fn)
		cancel_work_sync(&priv->completion_cb.work);
}

struct channel_gk20a *gk20a_open_new_channel_with_cb(struct gk20a *g,
		void (*update_fn)(struct channel_gk20a *, void *),
		void *update_fn_data,
		int runlist_id,
		bool is_privileged_channel)
{
	struct channel_gk20a *ch;
	struct nvgpu_channel_linux *priv;

	ch = gk20a_open_new_channel(g, runlist_id, is_privileged_channel,
				nvgpu_current_pid(g), nvgpu_current_tid(g));

	if (ch) {
		priv = ch->os_priv;
		nvgpu_spinlock_acquire(&priv->completion_cb.lock);
		priv->completion_cb.fn = update_fn;
		priv->completion_cb.user_data = update_fn_data;
		nvgpu_spinlock_release(&priv->completion_cb.lock);
	}

	return ch;
}

static void nvgpu_channel_open_linux(struct channel_gk20a *ch)
{
}

static void nvgpu_channel_close_linux(struct channel_gk20a *ch)
{
	nvgpu_channel_work_completion_clear(ch);

#if defined(CONFIG_GK20A_CYCLE_STATS)
	gk20a_channel_free_cycle_stats_buffer(ch);
	gk20a_channel_free_cycle_stats_snapshot(ch);
#endif
}

static int nvgpu_channel_alloc_linux(struct gk20a *g, struct channel_gk20a *ch)
{
	struct nvgpu_channel_linux *priv;
	int err;

	priv = nvgpu_kzalloc(g, sizeof(*priv));
	if (!priv)
		return -ENOMEM;

	ch->os_priv = priv;
	priv->ch = ch;

#ifdef CONFIG_SYNC
	ch->has_os_fence_framework_support = true;
#endif

	err = nvgpu_mutex_init(&priv->error_notifier.mutex);
	if (err) {
		nvgpu_kfree(g, priv);
		return err;
	}

	nvgpu_channel_work_completion_init(ch);

	return 0;
}

static void nvgpu_channel_free_linux(struct gk20a *g, struct channel_gk20a *ch)
{
	struct nvgpu_channel_linux *priv = ch->os_priv;

	nvgpu_mutex_destroy(&priv->error_notifier.mutex);
	nvgpu_kfree(g, priv);

	ch->os_priv = NULL;

#ifdef CONFIG_SYNC
	ch->has_os_fence_framework_support = false;
#endif
}

static int nvgpu_channel_init_os_fence_framework(struct channel_gk20a *ch,
	const char *fmt, ...)
{
	struct nvgpu_channel_linux *priv = ch->os_priv;
	struct nvgpu_os_fence_framework *fence_framework;
	char name[30];
	va_list args;

	fence_framework = &priv->fence_framework;

	va_start(args, fmt);
	vsnprintf(name, sizeof(name), fmt, args);
	va_end(args);

	fence_framework->timeline = gk20a_sync_timeline_create(name);

	if (!fence_framework->timeline)
		return -EINVAL;

	return 0;
}
static void nvgpu_channel_signal_os_fence_framework(struct channel_gk20a *ch)
{
	struct nvgpu_channel_linux *priv = ch->os_priv;
	struct nvgpu_os_fence_framework *fence_framework;

	fence_framework = &priv->fence_framework;

	gk20a_sync_timeline_signal(fence_framework->timeline);
}

static void nvgpu_channel_destroy_os_fence_framework(struct channel_gk20a *ch)
{
	struct nvgpu_channel_linux *priv = ch->os_priv;
	struct nvgpu_os_fence_framework *fence_framework;

	fence_framework = &priv->fence_framework;

	gk20a_sync_timeline_destroy(fence_framework->timeline);
	fence_framework->timeline = NULL;
}

static bool nvgpu_channel_fence_framework_exists(struct channel_gk20a *ch)
{
	struct nvgpu_channel_linux *priv = ch->os_priv;
	struct nvgpu_os_fence_framework *fence_framework;

	fence_framework = &priv->fence_framework;

	return (fence_framework->timeline != NULL);
}

static int nvgpu_channel_copy_user_gpfifo(struct nvgpu_gpfifo_entry *dest,
		struct nvgpu_gpfifo_userdata userdata, u32 start, u32 length)
{
	struct nvgpu_gpfifo_entry __user *user_gpfifo = userdata.entries;
	unsigned long n;

	n = copy_from_user(dest, user_gpfifo + start,
			length * sizeof(struct nvgpu_gpfifo_entry));

	return n == 0 ? 0 : -EFAULT;
}

int nvgpu_usermode_buf_from_dmabuf(struct gk20a *g, int dmabuf_fd,
		struct nvgpu_mem *mem, struct nvgpu_usermode_buf_linux *buf)
{
	struct device *dev = dev_from_gk20a(g);
	struct dma_buf *dmabuf;
	struct sg_table *sgt;
	struct dma_buf_attachment *attachment;
	int err;

	dmabuf = dma_buf_get(dmabuf_fd);
	if (IS_ERR(dmabuf)) {
		return PTR_ERR(dmabuf);
	}

	if (gk20a_dmabuf_aperture(g, dmabuf) == APERTURE_INVALID) {
		err = -EINVAL;
		goto put_dmabuf;
	}

	err = gk20a_dmabuf_alloc_drvdata(dmabuf, dev);
	if (err != 0) {
		goto put_dmabuf;
	}

	sgt = gk20a_mm_pin(dev, dmabuf, &attachment);
	if (IS_ERR(sgt)) {
		nvgpu_warn(g, "Failed to pin dma_buf!");
		err = PTR_ERR(sgt);
		goto put_dmabuf;
	}

	buf->dmabuf = dmabuf;
	buf->attachment = attachment;
	buf->sgt = sgt;

	/*
	 * This mem is unmapped and freed in a common path; for Linux, we'll
	 * also need to unref the dmabuf stuff (above) but the sgt here is only
	 * borrowed, so it cannot be freed by nvgpu_mem_*.
	 */
	mem->mem_flags  = NVGPU_MEM_FLAG_FOREIGN_SGT;
	mem->aperture   = APERTURE_SYSMEM;
	mem->skip_wmb   = 0;
	mem->size       = dmabuf->size;

	mem->priv.flags = 0;
	mem->priv.pages = NULL;
	mem->priv.sgt   = sgt;

	return 0;
put_dmabuf:
	dma_buf_put(dmabuf);
	return err;
}

void nvgpu_channel_free_usermode_buffers(struct channel_gk20a *c)
{
	struct nvgpu_channel_linux *priv = c->os_priv;
	struct gk20a *g = c->g;
	struct device *dev = dev_from_gk20a(g);

	if (priv->usermode.gpfifo.dmabuf != NULL) {
		gk20a_mm_unpin(dev, priv->usermode.gpfifo.dmabuf,
			       priv->usermode.gpfifo.attachment,
			       priv->usermode.gpfifo.sgt);
		dma_buf_put(priv->usermode.gpfifo.dmabuf);
		priv->usermode.gpfifo.dmabuf = NULL;
	}

	if (priv->usermode.userd.dmabuf != NULL) {
		gk20a_mm_unpin(dev, priv->usermode.userd.dmabuf,
		       priv->usermode.userd.attachment,
		       priv->usermode.userd.sgt);
		dma_buf_put(priv->usermode.userd.dmabuf);
		priv->usermode.userd.dmabuf = NULL;
	}
}

static int nvgpu_channel_alloc_usermode_buffers(struct channel_gk20a *c,
		struct nvgpu_setup_bind_args *args)
{
	struct nvgpu_channel_linux *priv = c->os_priv;
	struct gk20a *g = c->g;
	struct device *dev = dev_from_gk20a(g);
	size_t gpfifo_size;
	int err;

	if (args->gpfifo_dmabuf_fd == 0 || args->userd_dmabuf_fd == 0) {
		return -EINVAL;
	}

	if (args->gpfifo_dmabuf_offset != 0 ||
			args->userd_dmabuf_offset != 0) {
		/* TODO - not yet supported */
		return -EINVAL;
	}

	err = nvgpu_usermode_buf_from_dmabuf(g, args->gpfifo_dmabuf_fd,
			&c->usermode_gpfifo, &priv->usermode.gpfifo);
	if (err < 0) {
		return err;
	}

	gpfifo_size = max_t(u32, SZ_4K,
			args->num_gpfifo_entries *
			nvgpu_get_gpfifo_entry_size());

	if (c->usermode_gpfifo.size < gpfifo_size) {
		err = -EINVAL;
		goto free_gpfifo;
	}

	c->usermode_gpfifo.gpu_va = nvgpu_gmmu_map(c->vm, &c->usermode_gpfifo,
			c->usermode_gpfifo.size, 0, gk20a_mem_flag_none,
			false, c->usermode_gpfifo.aperture);

	if (c->usermode_gpfifo.gpu_va == 0) {
		err = -ENOMEM;
		goto unmap_free_gpfifo;
	}

	err = nvgpu_usermode_buf_from_dmabuf(g, args->userd_dmabuf_fd,
			&c->usermode_userd, &priv->usermode.userd);
	if (err < 0) {
		goto unmap_free_gpfifo;
	}

	args->work_submit_token = g->fifo.channel_base + c->chid;

	return 0;
unmap_free_gpfifo:
	nvgpu_dma_unmap_free(c->vm, &c->usermode_gpfifo);
free_gpfifo:
	gk20a_mm_unpin(dev, priv->usermode.gpfifo.dmabuf,
		       priv->usermode.gpfifo.attachment,
		       priv->usermode.gpfifo.sgt);
	dma_buf_put(priv->usermode.gpfifo.dmabuf);
	priv->usermode.gpfifo.dmabuf = NULL;
	return err;
}

int nvgpu_init_channel_support_linux(struct nvgpu_os_linux *l)
{
	struct gk20a *g = &l->g;
	struct fifo_gk20a *f = &g->fifo;
	int chid;
	int err;

	for (chid = 0; chid < (int)f->num_channels; chid++) {
		struct channel_gk20a *ch = &f->channel[chid];

		err = nvgpu_channel_alloc_linux(g, ch);
		if (err)
			goto err_clean;
	}

	g->os_channel.open = nvgpu_channel_open_linux;
	g->os_channel.close = nvgpu_channel_close_linux;
	g->os_channel.work_completion_signal =
		nvgpu_channel_work_completion_signal;
	g->os_channel.work_completion_cancel_sync =
		nvgpu_channel_work_completion_cancel_sync;

	g->os_channel.os_fence_framework_inst_exists =
		nvgpu_channel_fence_framework_exists;
	g->os_channel.init_os_fence_framework =
		nvgpu_channel_init_os_fence_framework;
	g->os_channel.signal_os_fence_framework =
		nvgpu_channel_signal_os_fence_framework;
	g->os_channel.destroy_os_fence_framework =
		nvgpu_channel_destroy_os_fence_framework;

	g->os_channel.copy_user_gpfifo =
		nvgpu_channel_copy_user_gpfifo;

	g->os_channel.alloc_usermode_buffers =
		nvgpu_channel_alloc_usermode_buffers;

	g->os_channel.free_usermode_buffers =
		nvgpu_channel_free_usermode_buffers;

	return 0;

err_clean:
	for (; chid >= 0; chid--) {
		struct channel_gk20a *ch = &f->channel[chid];

		nvgpu_channel_free_linux(g, ch);
	}
	return err;
}

void nvgpu_remove_channel_support_linux(struct nvgpu_os_linux *l)
{
	struct gk20a *g = &l->g;
	struct fifo_gk20a *f = &g->fifo;
	unsigned int chid;

	for (chid = 0; chid < f->num_channels; chid++) {
		struct channel_gk20a *ch = &f->channel[chid];

		nvgpu_channel_free_linux(g, ch);
	}

	g->os_channel.os_fence_framework_inst_exists = NULL;
	g->os_channel.init_os_fence_framework = NULL;
	g->os_channel.signal_os_fence_framework = NULL;
	g->os_channel.destroy_os_fence_framework = NULL;
}

u32 nvgpu_get_gpfifo_entry_size(void)
{
	return sizeof(struct nvgpu_gpfifo_entry);
}

#ifdef CONFIG_DEBUG_FS
static void trace_write_pushbuffer(struct channel_gk20a *c,
				   struct nvgpu_gpfifo_entry *g)
{
	void *mem = NULL;
	unsigned int words;
	u64 offset;
	struct dma_buf *dmabuf = NULL;

	if (gk20a_debug_trace_cmdbuf) {
		u64 gpu_va = (u64)g->entry0 |
			(u64)((u64)pbdma_gp_entry1_get_hi_v(g->entry1) << 32);
		int err;

		words = pbdma_gp_entry1_length_v(g->entry1);
		err = nvgpu_vm_find_buf(c->vm, gpu_va, &dmabuf, &offset);
		if (!err)
			mem = dma_buf_vmap(dmabuf);
	}

	if (mem) {
		u32 i;
		/*
		 * Write in batches of 128 as there seems to be a limit
		 * of how much you can output to ftrace at once.
		 */
		for (i = 0; i < words; i += 128U) {
			trace_gk20a_push_cmdbuf(
				c->g->name,
				0,
				min(words - i, 128U),
				offset + i * sizeof(u32),
				mem);
		}
		dma_buf_vunmap(dmabuf, mem);
	}
}

void trace_write_pushbuffers(struct channel_gk20a *c, u32 count)
{
	struct nvgpu_gpfifo_entry *gp = c->gpfifo.mem.cpu_va;
	u32 n = c->gpfifo.entry_num;
	u32 start = c->gpfifo.put;
	u32 i;

	if (!gk20a_debug_trace_cmdbuf)
		return;

	if (!gp)
		return;

	for (i = 0; i < count; i++)
		trace_write_pushbuffer(c, &gp[(start + i) % n]);
}
#endif
