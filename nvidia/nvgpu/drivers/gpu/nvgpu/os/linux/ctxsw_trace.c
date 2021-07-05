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

#include <linux/wait.h>
#include <linux/ktime.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <trace/events/gk20a.h>
#include <uapi/linux/nvgpu.h>
#include <nvgpu/ctxsw_trace.h>
#include <nvgpu/kmem.h>
#include <nvgpu/log.h>
#include <nvgpu/atomic.h>
#include <nvgpu/barrier.h>
#include <nvgpu/gk20a.h>
#include <nvgpu/channel.h>

#include "gk20a/gr_gk20a.h"
#include "gk20a/fecs_trace_gk20a.h"

#include "platform_gk20a.h"
#include "os_linux.h"
#include "ctxsw_trace.h"

#include <nvgpu/hw/gk20a/hw_ctxsw_prog_gk20a.h>
#include <nvgpu/hw/gk20a/hw_gr_gk20a.h>

#define GK20A_CTXSW_TRACE_MAX_VM_RING_SIZE	(128*PAGE_SIZE)

/* Userland-facing FIFO (one global + eventually one per VM) */
struct gk20a_ctxsw_dev {
	struct gk20a *g;

	struct nvgpu_ctxsw_ring_header *hdr;
	struct nvgpu_gpu_ctxsw_trace_entry *ents;
	struct nvgpu_gpu_ctxsw_trace_filter filter;
	bool write_enabled;
	struct nvgpu_cond readout_wq;
	size_t size;
	u32 num_ents;

	nvgpu_atomic_t vma_ref;

	struct nvgpu_mutex write_lock;
};


struct gk20a_ctxsw_trace {
	struct gk20a_ctxsw_dev devs[GK20A_CTXSW_TRACE_NUM_DEVS];
};

static inline int ring_is_empty(struct nvgpu_ctxsw_ring_header *hdr)
{
	return (hdr->write_idx == hdr->read_idx);
}

static inline int ring_is_full(struct nvgpu_ctxsw_ring_header *hdr)
{
	return ((hdr->write_idx + 1) % hdr->num_ents) == hdr->read_idx;
}

static inline int ring_len(struct nvgpu_ctxsw_ring_header *hdr)
{
	return (hdr->write_idx - hdr->read_idx) % hdr->num_ents;
}

static void nvgpu_set_ctxsw_trace_entry(struct nvgpu_ctxsw_trace_entry *entry_dst,
        struct nvgpu_gpu_ctxsw_trace_entry *entry_src)
{
	entry_dst->tag = entry_src->tag;
	entry_dst->vmid = entry_src->vmid;
	entry_dst->seqno = entry_src->seqno;
	entry_dst->context_id = entry_src->context_id;
	entry_dst->pid = entry_src->pid;
	entry_dst->timestamp = entry_src->timestamp;
}

ssize_t gk20a_ctxsw_dev_read(struct file *filp, char __user *buf, size_t size,
	loff_t *off)
{
	struct gk20a_ctxsw_dev *dev = filp->private_data;
	struct gk20a *g = dev->g;
	struct nvgpu_ctxsw_ring_header *hdr = dev->hdr;
	struct nvgpu_ctxsw_trace_entry __user *entry =
		(struct nvgpu_ctxsw_trace_entry *) buf;
	struct nvgpu_ctxsw_trace_entry user_entry;
	size_t copied = 0;
	int err;

	nvgpu_log(g, gpu_dbg_fn|gpu_dbg_ctxsw,
		"filp=%p buf=%p size=%zu", filp, buf, size);

	nvgpu_mutex_acquire(&dev->write_lock);
	while (ring_is_empty(hdr)) {
		nvgpu_mutex_release(&dev->write_lock);
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		err = NVGPU_COND_WAIT_INTERRUPTIBLE(&dev->readout_wq,
			!ring_is_empty(hdr), 0);
		if (err)
			return err;
		nvgpu_mutex_acquire(&dev->write_lock);
	}

	while (size >= sizeof(struct nvgpu_gpu_ctxsw_trace_entry)) {
		if (ring_is_empty(hdr))
			break;

		nvgpu_set_ctxsw_trace_entry(&user_entry, &dev->ents[hdr->read_idx]);
		if (copy_to_user(entry, &user_entry,
			sizeof(*entry))) {
			nvgpu_mutex_release(&dev->write_lock);
			return -EFAULT;
		}

		hdr->read_idx++;
		if (hdr->read_idx >= hdr->num_ents)
			hdr->read_idx = 0;

		entry++;
		copied += sizeof(*entry);
		size -= sizeof(*entry);
	}

	nvgpu_log(g, gpu_dbg_ctxsw, "copied=%zu read_idx=%d", copied,
		hdr->read_idx);

	*off = hdr->read_idx;
	nvgpu_mutex_release(&dev->write_lock);

	return copied;
}

static int gk20a_ctxsw_dev_ioctl_trace_enable(struct gk20a_ctxsw_dev *dev)
{
	struct gk20a *g = dev->g;

	nvgpu_log(g, gpu_dbg_fn|gpu_dbg_ctxsw, "trace enabled");
	nvgpu_mutex_acquire(&dev->write_lock);
	dev->write_enabled = true;
	nvgpu_mutex_release(&dev->write_lock);
	dev->g->ops.fecs_trace.enable(dev->g);
	return 0;
}

static int gk20a_ctxsw_dev_ioctl_trace_disable(struct gk20a_ctxsw_dev *dev)
{
	struct gk20a *g = dev->g;

	nvgpu_log(g, gpu_dbg_fn|gpu_dbg_ctxsw, "trace disabled");
	dev->g->ops.fecs_trace.disable(dev->g);
	nvgpu_mutex_acquire(&dev->write_lock);
	dev->write_enabled = false;
	nvgpu_mutex_release(&dev->write_lock);
	return 0;
}

static int gk20a_ctxsw_dev_alloc_buffer(struct gk20a_ctxsw_dev *dev,
					size_t size)
{
	struct gk20a *g = dev->g;
	void *buf;
	int err;

	if ((dev->write_enabled) || (nvgpu_atomic_read(&dev->vma_ref)))
		return -EBUSY;

	err = g->ops.fecs_trace.alloc_user_buffer(g, &buf, &size);
	if (err)
		return err;


	dev->hdr = buf;
	dev->ents = (struct nvgpu_gpu_ctxsw_trace_entry *) (dev->hdr + 1);
	dev->size = size;
	dev->num_ents = dev->hdr->num_ents;

	nvgpu_log(g, gpu_dbg_ctxsw, "size=%zu hdr=%p ents=%p num_ents=%d",
		dev->size, dev->hdr, dev->ents, dev->hdr->num_ents);
	return 0;
}

int gk20a_ctxsw_dev_ring_alloc(struct gk20a *g,
		void **buf, size_t *size)
{
	struct nvgpu_ctxsw_ring_header *hdr;

	*size = roundup(*size, PAGE_SIZE);
	hdr = vmalloc_user(*size);
	if (!hdr)
		return -ENOMEM;

	hdr->magic = NVGPU_CTXSW_RING_HEADER_MAGIC;
	hdr->version = NVGPU_CTXSW_RING_HEADER_VERSION;
	hdr->num_ents = (*size - sizeof(struct nvgpu_ctxsw_ring_header))
		/ sizeof(struct nvgpu_gpu_ctxsw_trace_entry);
	hdr->ent_size = sizeof(struct nvgpu_gpu_ctxsw_trace_entry);
	hdr->drop_count = 0;
	hdr->read_idx = 0;
	hdr->write_idx = 0;
	hdr->write_seqno = 0;

	*buf = hdr;
	return 0;
}

int gk20a_ctxsw_dev_ring_free(struct gk20a *g)
{
	struct gk20a_ctxsw_dev *dev = &g->ctxsw_trace->devs[0];

	nvgpu_vfree(g, dev->hdr);
	return 0;
}

static int gk20a_ctxsw_dev_ioctl_ring_setup(struct gk20a_ctxsw_dev *dev,
	struct nvgpu_ctxsw_ring_setup_args *args)
{
	struct gk20a *g = dev->g;
	size_t size = args->size;
	int ret;

	nvgpu_log(g, gpu_dbg_fn|gpu_dbg_ctxsw, "size=%zu", size);

	if (size > GK20A_CTXSW_TRACE_MAX_VM_RING_SIZE)
		return -EINVAL;

	nvgpu_mutex_acquire(&dev->write_lock);
	ret = gk20a_ctxsw_dev_alloc_buffer(dev, size);
	nvgpu_mutex_release(&dev->write_lock);

	return ret;
}

static void nvgpu_set_ctxsw_trace_filter_args(struct nvgpu_gpu_ctxsw_trace_filter *filter_dst,
        struct nvgpu_ctxsw_trace_filter *filter_src)
{
	memcpy(filter_dst->tag_bits, filter_src->tag_bits, (NVGPU_CTXSW_FILTER_SIZE + 63) / 64);
}

static void nvgpu_get_ctxsw_trace_filter_args(struct nvgpu_ctxsw_trace_filter *filter_dst,
	struct nvgpu_gpu_ctxsw_trace_filter *filter_src)
{
	memcpy(filter_dst->tag_bits, filter_src->tag_bits, (NVGPU_CTXSW_FILTER_SIZE + 63) / 64);
}

static int gk20a_ctxsw_dev_ioctl_set_filter(struct gk20a_ctxsw_dev *dev,
	struct nvgpu_ctxsw_trace_filter_args *args)
{
	struct gk20a *g = dev->g;

	nvgpu_mutex_acquire(&dev->write_lock);
	nvgpu_set_ctxsw_trace_filter_args(&dev->filter, &args->filter);
	nvgpu_mutex_release(&dev->write_lock);

	if (g->ops.fecs_trace.set_filter)
		g->ops.fecs_trace.set_filter(g, &dev->filter);
	return 0;
}

static int gk20a_ctxsw_dev_ioctl_get_filter(struct gk20a_ctxsw_dev *dev,
	struct nvgpu_ctxsw_trace_filter_args *args)
{
	nvgpu_mutex_acquire(&dev->write_lock);
	nvgpu_get_ctxsw_trace_filter_args(&args->filter, &dev->filter);
	nvgpu_mutex_release(&dev->write_lock);

	return 0;
}

static int gk20a_ctxsw_dev_ioctl_poll(struct gk20a_ctxsw_dev *dev)
{
	struct gk20a *g = dev->g;
	int err;

	nvgpu_log(g, gpu_dbg_fn|gpu_dbg_ctxsw, " ");

	err = gk20a_busy(g);
	if (err)
		return err;

	if (g->ops.fecs_trace.flush)
		err = g->ops.fecs_trace.flush(g);

	if (likely(!err))
		err = g->ops.fecs_trace.poll(g);

	gk20a_idle(g);
	return err;
}

int gk20a_ctxsw_dev_open(struct inode *inode, struct file *filp)
{
	struct nvgpu_os_linux *l;
	struct gk20a *g;
	struct gk20a_ctxsw_trace *trace;
	struct gk20a_ctxsw_dev *dev;
	int err;
	size_t size;
	u32 n;

	/* only one VM for now */
	const int vmid = 0;

	l = container_of(inode->i_cdev, struct nvgpu_os_linux, ctxsw.cdev);
	g = gk20a_get(&l->g);
	if (!g)
		return -ENODEV;

	nvgpu_log(g, gpu_dbg_fn|gpu_dbg_ctxsw, "g=%p", g);

	err = gk20a_busy(g);
	if (err)
		goto free_ref;

	trace = g->ctxsw_trace;
	if (!trace) {
		err = -ENODEV;
		goto idle;
	}

	/* Allow only one user for this device */
	dev = &trace->devs[vmid];
	nvgpu_mutex_acquire(&dev->write_lock);
	if (dev->hdr) {
		err = -EBUSY;
		goto done;
	}

	/* By default, allocate ring buffer big enough to accommodate
	 * FECS records with default event filter */

	/* enable all traces by default */
	NVGPU_CTXSW_FILTER_SET_ALL(&dev->filter);

	/* compute max number of entries generated with this filter */
	n = g->ops.fecs_trace.max_entries(g, &dev->filter);

	size = sizeof(struct nvgpu_ctxsw_ring_header) +
			n * sizeof(struct nvgpu_gpu_ctxsw_trace_entry);
	nvgpu_log(g, gpu_dbg_ctxsw, "size=%zu entries=%d ent_size=%zu",
		size, n, sizeof(struct nvgpu_gpu_ctxsw_trace_entry));

	err = gk20a_ctxsw_dev_alloc_buffer(dev, size);
	if (!err) {
		filp->private_data = dev;
		nvgpu_log(g, gpu_dbg_ctxsw, "filp=%p dev=%p size=%zu",
			filp, dev, size);
	}

done:
	nvgpu_mutex_release(&dev->write_lock);

idle:
	gk20a_idle(g);
free_ref:
	if (err)
		gk20a_put(g);
	return err;
}

int gk20a_ctxsw_dev_release(struct inode *inode, struct file *filp)
{
	struct gk20a_ctxsw_dev *dev = filp->private_data;
	struct gk20a *g = dev->g;

	nvgpu_log(g, gpu_dbg_fn|gpu_dbg_ctxsw, "dev: %p", dev);

	g->ops.fecs_trace.disable(g);

	nvgpu_mutex_acquire(&dev->write_lock);
	dev->write_enabled = false;
	nvgpu_mutex_release(&dev->write_lock);

	if (dev->hdr) {
		dev->g->ops.fecs_trace.free_user_buffer(dev->g);
		dev->hdr = NULL;
	}
	gk20a_put(g);
	return 0;
}

long gk20a_ctxsw_dev_ioctl(struct file *filp, unsigned int cmd,
	unsigned long arg)
{
	struct gk20a_ctxsw_dev *dev = filp->private_data;
	struct gk20a *g = dev->g;
	u8 buf[NVGPU_CTXSW_IOCTL_MAX_ARG_SIZE];
	int err = 0;

	nvgpu_log(g, gpu_dbg_fn|gpu_dbg_ctxsw, "nr=%d", _IOC_NR(cmd));

	if ((_IOC_TYPE(cmd) != NVGPU_CTXSW_IOCTL_MAGIC) ||
		(_IOC_NR(cmd) == 0) ||
		(_IOC_NR(cmd) > NVGPU_CTXSW_IOCTL_LAST) ||
		(_IOC_SIZE(cmd) > NVGPU_CTXSW_IOCTL_MAX_ARG_SIZE))
		return -EINVAL;

	memset(buf, 0, sizeof(buf));
	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(buf, (void __user *) arg, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	switch (cmd) {
	case NVGPU_CTXSW_IOCTL_TRACE_ENABLE:
		err = gk20a_ctxsw_dev_ioctl_trace_enable(dev);
		break;
	case NVGPU_CTXSW_IOCTL_TRACE_DISABLE:
		err = gk20a_ctxsw_dev_ioctl_trace_disable(dev);
		break;
	case NVGPU_CTXSW_IOCTL_RING_SETUP:
		err = gk20a_ctxsw_dev_ioctl_ring_setup(dev,
			(struct nvgpu_ctxsw_ring_setup_args *) buf);
		break;
	case NVGPU_CTXSW_IOCTL_SET_FILTER:
		err = gk20a_ctxsw_dev_ioctl_set_filter(dev,
			(struct nvgpu_ctxsw_trace_filter_args *) buf);
		break;
	case NVGPU_CTXSW_IOCTL_GET_FILTER:
		err = gk20a_ctxsw_dev_ioctl_get_filter(dev,
			(struct nvgpu_ctxsw_trace_filter_args *) buf);
		break;
	case NVGPU_CTXSW_IOCTL_POLL:
		err = gk20a_ctxsw_dev_ioctl_poll(dev);
		break;
	default:
		dev_dbg(dev_from_gk20a(g), "unrecognized gpu ioctl cmd: 0x%x",
			cmd);
		err = -ENOTTY;
	}

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *) arg, buf, _IOC_SIZE(cmd));

	return err;
}

unsigned int gk20a_ctxsw_dev_poll(struct file *filp, poll_table *wait)
{
	struct gk20a_ctxsw_dev *dev = filp->private_data;
	struct gk20a *g = dev->g;
	struct nvgpu_ctxsw_ring_header *hdr = dev->hdr;
	unsigned int mask = 0;

	nvgpu_log(g, gpu_dbg_fn|gpu_dbg_ctxsw, " ");

	nvgpu_mutex_acquire(&dev->write_lock);
	poll_wait(filp, &dev->readout_wq.wq, wait);
	if (!ring_is_empty(hdr))
		mask |= POLLIN | POLLRDNORM;
	nvgpu_mutex_release(&dev->write_lock);

	return mask;
}

static void gk20a_ctxsw_dev_vma_open(struct vm_area_struct *vma)
{
	struct gk20a_ctxsw_dev *dev = vma->vm_private_data;
	struct gk20a *g = dev->g;

	nvgpu_atomic_inc(&dev->vma_ref);
	nvgpu_log(g, gpu_dbg_fn|gpu_dbg_ctxsw, "vma_ref=%d",
		nvgpu_atomic_read(&dev->vma_ref));
}

static void gk20a_ctxsw_dev_vma_close(struct vm_area_struct *vma)
{
	struct gk20a_ctxsw_dev *dev = vma->vm_private_data;
	struct gk20a *g = dev->g;

	nvgpu_atomic_dec(&dev->vma_ref);
	nvgpu_log(g, gpu_dbg_fn|gpu_dbg_ctxsw, "vma_ref=%d",
		nvgpu_atomic_read(&dev->vma_ref));
}

static struct vm_operations_struct gk20a_ctxsw_dev_vma_ops = {
	.open = gk20a_ctxsw_dev_vma_open,
	.close = gk20a_ctxsw_dev_vma_close,
};

int gk20a_ctxsw_dev_mmap_buffer(struct gk20a *g,
				struct vm_area_struct *vma)
{
	return remap_vmalloc_range(vma, g->ctxsw_trace->devs[0].hdr, 0);
}

int gk20a_ctxsw_dev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct gk20a_ctxsw_dev *dev = filp->private_data;
	struct gk20a *g = dev->g;
	int ret;

	nvgpu_log(g, gpu_dbg_fn|gpu_dbg_ctxsw, "vm_start=%lx vm_end=%lx",
		vma->vm_start, vma->vm_end);

	ret = dev->g->ops.fecs_trace.mmap_user_buffer(dev->g, vma);
	if (likely(!ret)) {
		vma->vm_private_data = dev;
		vma->vm_ops = &gk20a_ctxsw_dev_vma_ops;
		vma->vm_ops->open(vma);
	}

	return ret;
}

#ifdef CONFIG_GK20A_CTXSW_TRACE
static int gk20a_ctxsw_init_devs(struct gk20a *g)
{
	struct gk20a_ctxsw_trace *trace = g->ctxsw_trace;
	struct gk20a_ctxsw_dev *dev = trace->devs;
	int err;
	int i;

	for (i = 0; i < GK20A_CTXSW_TRACE_NUM_DEVS; i++) {
		dev->g = g;
		dev->hdr = NULL;
		dev->write_enabled = false;
		nvgpu_cond_init(&dev->readout_wq);
		err = nvgpu_mutex_init(&dev->write_lock);
		if (err)
			return err;
		nvgpu_atomic_set(&dev->vma_ref, 0);
		dev++;
	}
	return 0;
}
#endif

int gk20a_ctxsw_trace_init(struct gk20a *g)
{
#ifdef CONFIG_GK20A_CTXSW_TRACE
	struct gk20a_ctxsw_trace *trace = g->ctxsw_trace;
	int err;

	nvgpu_log(g, gpu_dbg_fn|gpu_dbg_ctxsw, "g=%p trace=%p", g, trace);

	/* if tracing is not supported, skip this */
	if (!g->ops.fecs_trace.init)
		return 0;

	if (likely(trace)) {
		__nvgpu_set_enabled(g, NVGPU_SUPPORT_FECS_CTXSW_TRACE, true);
		return 0;
	}

	trace = nvgpu_kzalloc(g, sizeof(*trace));
	if (unlikely(!trace))
		return -ENOMEM;
	g->ctxsw_trace = trace;

	err = gk20a_ctxsw_init_devs(g);
	if (err)
		goto fail;

	err = g->ops.fecs_trace.init(g);
	if (unlikely(err))
		goto fail;

	return 0;

fail:
	memset(&g->ops.fecs_trace, 0, sizeof(g->ops.fecs_trace));
	nvgpu_kfree(g, trace);
	g->ctxsw_trace = NULL;
	return err;
#else
	return 0;
#endif
}

void gk20a_ctxsw_trace_cleanup(struct gk20a *g)
{
#ifdef CONFIG_GK20A_CTXSW_TRACE
	struct gk20a_ctxsw_trace *trace;
	struct gk20a_ctxsw_dev *dev;
	int i;

	if (!g->ctxsw_trace)
		return;

	trace = g->ctxsw_trace;
	dev = trace->devs;

	for (i = 0; i < GK20A_CTXSW_TRACE_NUM_DEVS; i++) {
		nvgpu_mutex_destroy(&dev->write_lock);
		dev++;
	}

	nvgpu_kfree(g, g->ctxsw_trace);
	g->ctxsw_trace = NULL;

	g->ops.fecs_trace.deinit(g);
#endif
}

int gk20a_ctxsw_trace_write(struct gk20a *g,
		struct nvgpu_gpu_ctxsw_trace_entry *entry)
{
	struct nvgpu_ctxsw_ring_header *hdr;
	struct gk20a_ctxsw_dev *dev;
	int ret = 0;
	const char *reason;
	u32 write_idx;

	if (!g->ctxsw_trace)
		return 0;

	if (unlikely(entry->vmid >= GK20A_CTXSW_TRACE_NUM_DEVS))
		return -ENODEV;

	dev = &g->ctxsw_trace->devs[entry->vmid];
	hdr = dev->hdr;

	nvgpu_log(g, gpu_dbg_fn | gpu_dbg_ctxsw,
		"dev=%p hdr=%p", dev, hdr);

	nvgpu_mutex_acquire(&dev->write_lock);

	if (unlikely(!hdr)) {
		/* device has been released */
		ret = -ENODEV;
		goto done;
	}

	write_idx = hdr->write_idx;
	if (write_idx >= dev->num_ents) {
		nvgpu_err(dev->g,
			"write_idx=%u out of range [0..%u]",
			write_idx, dev->num_ents);
		ret = -ENOSPC;
		reason = "write_idx out of range";
		goto disable;
	}

	entry->seqno = hdr->write_seqno++;

	if (!dev->write_enabled) {
		ret = -EBUSY;
		reason = "write disabled";
		goto drop;
	}

	if (unlikely(ring_is_full(hdr))) {
		ret = -ENOSPC;
		reason = "user fifo full";
		goto drop;
	}

	if (!NVGPU_GPU_CTXSW_FILTER_ISSET(entry->tag, &dev->filter)) {
		reason = "filtered out";
		goto filter;
	}

	nvgpu_log(g, gpu_dbg_ctxsw,
		"seqno=%d context_id=%08x pid=%lld tag=%x timestamp=%llx",
		entry->seqno, entry->context_id, entry->pid,
		entry->tag, entry->timestamp);

	dev->ents[write_idx] = *entry;

	/* ensure record is written before updating write index */
	nvgpu_smp_wmb();

	write_idx++;
	if (unlikely(write_idx >= hdr->num_ents))
		write_idx = 0;
	hdr->write_idx = write_idx;
	nvgpu_log(g, gpu_dbg_ctxsw, "added: read=%d write=%d len=%d",
		hdr->read_idx, hdr->write_idx, ring_len(hdr));

	nvgpu_mutex_release(&dev->write_lock);
	return ret;

disable:
	g->ops.fecs_trace.disable(g);

drop:
	hdr->drop_count++;

filter:
	nvgpu_log(g, gpu_dbg_ctxsw,
			"dropping seqno=%d context_id=%08x pid=%lld "
			"tag=%x time=%llx (%s)",
			entry->seqno, entry->context_id, entry->pid,
			entry->tag, entry->timestamp, reason);

done:
	nvgpu_mutex_release(&dev->write_lock);
	return ret;
}

void gk20a_ctxsw_trace_wake_up(struct gk20a *g, int vmid)
{
	struct gk20a_ctxsw_dev *dev;

	if (!g->ctxsw_trace)
		return;

	dev = &g->ctxsw_trace->devs[vmid];
	nvgpu_cond_signal_interruptible(&dev->readout_wq);
}

void gk20a_ctxsw_trace_channel_reset(struct gk20a *g, struct channel_gk20a *ch)
{
#ifdef CONFIG_GK20A_CTXSW_TRACE
	struct nvgpu_gpu_ctxsw_trace_entry entry = {
		.vmid = 0,
		.tag = NVGPU_CTXSW_TAG_ENGINE_RESET,
		.context_id = 0,
		.pid = ch->tgid,
	};

	if (!g->ctxsw_trace)
		return;

	g->ops.ptimer.read_ptimer(g, &entry.timestamp);
	gk20a_ctxsw_trace_write(g, &entry);
	gk20a_ctxsw_trace_wake_up(g, 0);
#endif
	trace_gk20a_channel_reset(ch->chid, ch->tsgid);
}

void gk20a_ctxsw_trace_tsg_reset(struct gk20a *g, struct tsg_gk20a *tsg)
{
#ifdef CONFIG_GK20A_CTXSW_TRACE
	struct nvgpu_gpu_ctxsw_trace_entry entry = {
		.vmid = 0,
		.tag = NVGPU_CTXSW_TAG_ENGINE_RESET,
		.context_id = 0,
		.pid = tsg->tgid,
	};

	if (!g->ctxsw_trace)
		return;

	g->ops.ptimer.read_ptimer(g, &entry.timestamp);
	gk20a_ctxsw_trace_write(g, &entry);
	gk20a_ctxsw_trace_wake_up(g, 0);
#endif
	trace_gk20a_channel_reset(~0, tsg->tsgid);
}

/*
 * Convert linux nvgpu ctxsw tags type of the form of NVGPU_CTXSW_TAG_*
 * into common nvgpu ctxsw tags type of the form of NVGPU_GPU_CTXSW_TAG_*
 */

u8 nvgpu_gpu_ctxsw_tags_to_common_tags(u8 tags)
{
	switch (tags){
	case NVGPU_CTXSW_TAG_SOF:
		return NVGPU_GPU_CTXSW_TAG_SOF;
	case NVGPU_CTXSW_TAG_CTXSW_REQ_BY_HOST:
		return NVGPU_GPU_CTXSW_TAG_CTXSW_REQ_BY_HOST;
	case NVGPU_CTXSW_TAG_FE_ACK:
		return NVGPU_GPU_CTXSW_TAG_FE_ACK;
	case NVGPU_CTXSW_TAG_FE_ACK_WFI:
		return NVGPU_GPU_CTXSW_TAG_FE_ACK_WFI;
	case NVGPU_CTXSW_TAG_FE_ACK_GFXP:
		return NVGPU_GPU_CTXSW_TAG_FE_ACK_GFXP;
	case NVGPU_CTXSW_TAG_FE_ACK_CTAP:
		return NVGPU_GPU_CTXSW_TAG_FE_ACK_CTAP;
	case NVGPU_CTXSW_TAG_FE_ACK_CILP:
		return NVGPU_GPU_CTXSW_TAG_FE_ACK_CILP;
	case NVGPU_CTXSW_TAG_SAVE_END:
		return NVGPU_GPU_CTXSW_TAG_SAVE_END;
	case NVGPU_CTXSW_TAG_RESTORE_START:
		return NVGPU_GPU_CTXSW_TAG_RESTORE_START;
	case NVGPU_CTXSW_TAG_CONTEXT_START:
		return NVGPU_GPU_CTXSW_TAG_CONTEXT_START;
	case NVGPU_CTXSW_TAG_ENGINE_RESET:
		return NVGPU_GPU_CTXSW_TAG_ENGINE_RESET;
	case NVGPU_CTXSW_TAG_INVALID_TIMESTAMP:
		return NVGPU_GPU_CTXSW_TAG_INVALID_TIMESTAMP;
	}

	WARN_ON(1);
	return tags;
}
