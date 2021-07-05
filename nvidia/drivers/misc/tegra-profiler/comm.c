/*
 * drivers/misc/tegra-profiler/comm.c
 *
 * Copyright (c) 2013-2019, NVIDIA CORPORATION.  All rights reserved.
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
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/circ_buf.h>
#include <linux/uaccess.h>
#include <linux/version.h>

#include <linux/tegra_profiler.h>

#include "comm.h"
#include "quadd.h"
#include "version.h"

struct quadd_ring_buffer {
	struct quadd_ring_buffer_hdr *rb_hdr;
	char *buf;

	size_t max_fill_count;
	size_t nr_skipped_samples;

	struct quadd_mmap_area *mmap;

	raw_spinlock_t lock;
};

struct quadd_comm_ctx {
	struct quadd_ctx *ctx;
	struct quadd_comm_control_interface *control;

	atomic_t active;

	struct mutex io_mutex;
	int nr_users;

	int params_ok;

	struct miscdevice *misc_dev;
};

struct comm_cpu_context {
	struct quadd_ring_buffer rb;
	int params_ok;
};

static struct quadd_comm_ctx comm_ctx;
static DEFINE_PER_CPU(struct comm_cpu_context, cpu_ctx);

static void
rb_write(struct quadd_ring_buffer_hdr *rb_hdr,
	 char *buf, const void *data, size_t length)
{
	size_t len, head = rb_hdr->pos_write;
	const char *s = data;

	if (length == 0)
		return;

	len = min_t(size_t, rb_hdr->size - head, length);

	memcpy(buf + head, s, len);
	head = (head + len) & (rb_hdr->size - 1);

	if (length > len) {
		s += len;
		len = length - len;

		memcpy(buf + head, s, len);
		head += len;
	}

	rb_hdr->pos_write = head;
}

static ssize_t
write_sample(struct quadd_ring_buffer *rb,
	     struct quadd_record_data *sample,
	     const struct quadd_iovec *vec, int vec_count)
{
	int i;
	size_t len = 0, c;
	struct quadd_ring_buffer_hdr hdr, *rb_hdr = rb->rb_hdr;

	if (!rb_hdr)
		return -EIO;

	if (vec) {
		for (i = 0; i < vec_count; i++)
			len += vec[i].len;
	}

	sample->extra_size = len;
	len += sizeof(*sample);

	hdr.size = rb_hdr->size;
	hdr.pos_write = rb_hdr->pos_write;
	hdr.pos_read = READ_ONCE(rb_hdr->pos_read);

	c = CIRC_SPACE(hdr.pos_write, hdr.pos_read, hdr.size);
	if (len > c) {
		pr_err_once("[cpu: %d] warning: buffer has been overflowed\n",
			    smp_processor_id());
		return -ENOSPC;
	}

	rb_write(&hdr, rb->buf, sample, sizeof(*sample));

	if (vec) {
		for (i = 0; i < vec_count; i++)
			rb_write(&hdr, rb->buf, vec[i].base, vec[i].len);
	}

	c = CIRC_CNT(hdr.pos_write, hdr.pos_read, hdr.size);
	if (c > rb->max_fill_count) {
		rb->max_fill_count = c;
		rb_hdr->max_fill_count = c;
	}

	/* Use smp_store_release() to update circle buffer write pointers to
	 * ensure the data is stored before we update write pointer.
	 */
	smp_store_release(&rb_hdr->pos_write, hdr.pos_write);

	return len;
}

static size_t get_data_size(void)
{
	int cpu_id;
	unsigned long flags;
	size_t size = 0, tail;
	struct comm_cpu_context *cc;
	struct quadd_ring_buffer *rb;
	struct quadd_ring_buffer_hdr *rb_hdr;

	for_each_possible_cpu(cpu_id) {
		cc = &per_cpu(cpu_ctx, cpu_id);

		rb = &cc->rb;

		rb_hdr = rb->rb_hdr;
		if (!rb_hdr)
			continue;

		raw_spin_lock_irqsave(&rb->lock, flags);

		tail = READ_ONCE(rb_hdr->pos_read);
		size += CIRC_CNT(rb_hdr->pos_write, tail, rb_hdr->size);

		raw_spin_unlock_irqrestore(&rb->lock, flags);
	}

	return size;
}

static ssize_t
put_sample(struct quadd_record_data *data,
	   struct quadd_iovec *vec,
	   int vec_count, int cpu_id)
{
	ssize_t err = 0;
	unsigned long flags;
	struct comm_cpu_context *cc;
	struct quadd_ring_buffer *rb;
	struct quadd_ring_buffer_hdr *rb_hdr;

	if (!atomic_read(&comm_ctx.active))
		return -EIO;

	cc = cpu_id < 0 ? this_cpu_ptr(&cpu_ctx) :
		&per_cpu(cpu_ctx, cpu_id);

	rb = &cc->rb;

	raw_spin_lock_irqsave(&rb->lock, flags);

	err = write_sample(rb, data, vec, vec_count);
	if (err < 0) {
		pr_err_once("%s: error: write sample\n", __func__);
		rb->nr_skipped_samples++;

		rb_hdr = rb->rb_hdr;
		if (rb_hdr)
			rb_hdr->skipped_samples++;
	}

	raw_spin_unlock_irqrestore(&rb->lock, flags);

	return err;
}

static void comm_reset(void)
{
	pr_debug("Comm reset\n");
}

static int is_active(void)
{
	return atomic_read(&comm_ctx.active) != 0;
}

static struct quadd_comm_data_interface comm_data = {
	.put_sample = put_sample,
	.reset = comm_reset,
	.is_active = is_active,
};

static struct quadd_mmap_area *find_mmap_by_vma(unsigned long vm_start)
{
	struct quadd_mmap_area *entry;

	list_for_each_entry(entry, &comm_ctx.ctx->mmap_areas, list) {
		struct vm_area_struct *mmap_vma = entry->mmap_vma;

		if (vm_start == mmap_vma->vm_start)
			return entry;
	}

	return NULL;
}

static struct quadd_mmap_area *find_mmap_by_hash(unsigned int hash)
{
	struct quadd_mmap_area *entry;

	list_for_each_entry(entry, &comm_ctx.ctx->mmap_areas, list) {
		if (hash == entry->fi.file_hash)
			return entry;
	}

	return NULL;
}

static int device_open(struct inode *inode, struct file *file)
{
	int err;

	mutex_lock(&comm_ctx.io_mutex);

	err = quadd_late_init();
	if (!err)
		comm_ctx.nr_users++;

	mutex_unlock(&comm_ctx.io_mutex);

	return err;
}

static int device_release(struct inode *inode, struct file *file)
{
	mutex_lock(&comm_ctx.io_mutex);
	comm_ctx.nr_users--;

	if (comm_ctx.nr_users == 0) {
		if (atomic_cmpxchg(&comm_ctx.active, 1, 0)) {
			comm_ctx.control->stop();
			pr_info("Stop profiling: daemon is closed\n");
		}
	}
	mutex_unlock(&comm_ctx.io_mutex);

	return 0;
}

static int
init_mmap_hdr(struct quadd_mmap_rb_info *mmap_rb,
	      struct quadd_mmap_area *mmap)
{
	unsigned int cpu_id;
	size_t size;
	unsigned long flags;
	struct vm_area_struct *vma;
	struct quadd_ring_buffer *rb;
	struct quadd_ring_buffer_hdr *rb_hdr;
	struct quadd_mmap_header *mmap_hdr;
	struct comm_cpu_context *cc;

	if (mmap->type != QUADD_MMAP_TYPE_RB)
		return -EIO;

	cpu_id = mmap_rb->cpu_id;

	if (!cpu_possible(cpu_id))
		return -EINVAL;

	cc = &per_cpu(cpu_ctx, cpu_id);

	rb = &cc->rb;

	vma = mmap->mmap_vma;
	size = vma->vm_end - vma->vm_start;

	if (size <= PAGE_SIZE || !is_power_of_2(size - PAGE_SIZE))
		return -EINVAL;

	size -= PAGE_SIZE;

	raw_spin_lock_irqsave(&rb->lock, flags);

	mmap->rb = rb;

	rb->mmap = mmap;
	rb->buf = (char *)mmap->data + PAGE_SIZE;

	rb->max_fill_count = 0;
	rb->nr_skipped_samples = 0;

	mmap_hdr = mmap->data;

	mmap_hdr->magic = QUADD_MMAP_HEADER_MAGIC;
	mmap_hdr->version = QUADD_MMAP_HEADER_VERSION;
	mmap_hdr->cpu_id = cpu_id;
	mmap_hdr->samples_version = QUADD_SAMPLES_VERSION;

	rb_hdr = (struct quadd_ring_buffer_hdr *)(mmap_hdr + 1);
	rb->rb_hdr = rb_hdr;

	rb_hdr->size = size;
	rb_hdr->pos_read = 0;
	rb_hdr->pos_write = 0;

	rb_hdr->max_fill_count = 0;
	rb_hdr->skipped_samples = 0;

	rb_hdr->state = QUADD_RB_STATE_ACTIVE;

	raw_spin_unlock_irqrestore(&rb->lock, flags);

	pr_debug("[cpu: %d] init_mmap_hdr: vma: %#lx - %#lx, data: %p - %p\n",
		 cpu_id,
		 vma->vm_start, vma->vm_end,
		 mmap->data, mmap->data + vma->vm_end - vma->vm_start);

	return 0;
}

static void rb_stop(void)
{
	int cpu_id;
	struct quadd_ring_buffer *rb;
	struct quadd_ring_buffer_hdr *rb_hdr;
	struct comm_cpu_context *cc;

	for_each_possible_cpu(cpu_id) {
		cc = &per_cpu(cpu_ctx, cpu_id);

		rb = &cc->rb;
		rb_hdr = rb->rb_hdr;

		if (!rb_hdr)
			continue;

		pr_info("[%d] skipped samples/max filling: %zu/%zu\n",
			cpu_id, rb->nr_skipped_samples, rb->max_fill_count);

		rb_hdr->state = QUADD_RB_STATE_STOPPED;
	}
}

static void rb_reset(struct quadd_ring_buffer *rb)
{
	unsigned long flags;

	if (!rb)
		return;

	raw_spin_lock_irqsave(&rb->lock, flags);

	rb->mmap = NULL;
	rb->buf = NULL;
	rb->rb_hdr = NULL;

	raw_spin_unlock_irqrestore(&rb->lock, flags);
}

static int
ready_to_profile(void)
{
	int cpuid, is_cpu_present;
	struct comm_cpu_context *cc;

	if (!comm_ctx.params_ok)
		return 0;

	if (quadd_mode_is_sampling(comm_ctx.ctx)) {
		for_each_possible_cpu(cpuid) {
			is_cpu_present =
				comm_ctx.control->is_cpu_present(cpuid);

			if (is_cpu_present) {
				cc = &per_cpu(cpu_ctx, cpuid);

				if (!cc->params_ok)
					return 0;
			}
		}
	}

	return 1;
}

static void
reset_params_ok_flag(void)
{
	int cpu_id;

	comm_ctx.params_ok = 0;
	for_each_possible_cpu(cpu_id) {
		struct comm_cpu_context *cc = &per_cpu(cpu_ctx, cpu_id);

		cc->params_ok = 0;
	}
}

static long
device_ioctl(struct file *file,
	     unsigned int ioctl_num,
	     unsigned long ioctl_param)
{
	int err = 0;
	unsigned int cpuid, file_hash;
	unsigned long mmap_start;
	struct quadd_mmap_area *mmap;
	struct quadd_parameters *user_params;
	struct quadd_pmu_setup_for_cpu *cpu_pmu_params;
	struct quadd_comm_cap_for_cpu *per_cpu_cap;
	struct quadd_comm_cap cap;
	struct quadd_module_state state;
	struct quadd_module_version versions;
	struct quadd_sections extabs;
	struct quadd_mmap_rb_info mmap_rb;

	mutex_lock(&comm_ctx.io_mutex);

	if (ioctl_num != IOCTL_SETUP &&
	    ioctl_num != IOCTL_GET_CAP &&
	    ioctl_num != IOCTL_GET_STATE &&
	    ioctl_num != IOCTL_SETUP_PMU_FOR_CPU &&
	    ioctl_num != IOCTL_GET_CAP_FOR_CPU &&
	    ioctl_num != IOCTL_GET_VERSION) {
		if (!ready_to_profile()) {
			err = -EACCES;
			goto error_out;
		}
	}

	switch (ioctl_num) {
	case IOCTL_SETUP_PMU_FOR_CPU:
		if (atomic_read(&comm_ctx.active)) {
			pr_err("error: tegra profiler is active\n");
			err = -EBUSY;
			goto error_out;
		}

		if (!comm_ctx.params_ok ||
		    !quadd_mode_is_sampling(comm_ctx.ctx)) {
			pr_err("error: incorrect setup ioctl\n");
			err = -EPERM;
			goto error_out;
		}

		cpu_pmu_params = vmalloc(sizeof(*cpu_pmu_params));
		if (!cpu_pmu_params) {
			err = -ENOMEM;
			goto error_out;
		}

		if (copy_from_user(cpu_pmu_params,
				   (void __user *)ioctl_param,
				   sizeof(*cpu_pmu_params))) {
			pr_err("setup failed\n");
			vfree(cpu_pmu_params);
			err = -EFAULT;
			goto error_out;
		}

		cpuid = cpu_pmu_params->cpuid;

		if (!cpu_possible(cpuid)) {
			vfree(cpu_pmu_params);
			err = -EINVAL;
			goto error_out;
		}

		per_cpu(cpu_ctx, cpuid).params_ok = 0;

		err = comm_ctx.control->set_parameters_for_cpu(cpu_pmu_params);
		if (err) {
			pr_err("error: setup failed\n");
			vfree(cpu_pmu_params);
			goto error_out;
		}

		per_cpu(cpu_ctx, cpuid).params_ok = 1;

		pr_info("setup PMU: success for cpu: %d\n", cpuid);

		vfree(cpu_pmu_params);
		break;

	case IOCTL_SETUP:
		if (atomic_read(&comm_ctx.active)) {
			pr_err("error: tegra profiler is active\n");
			err = -EBUSY;
			goto error_out;
		}
		reset_params_ok_flag();

		user_params = vmalloc(sizeof(*user_params));
		if (!user_params) {
			err = -ENOMEM;
			goto error_out;
		}

		if (copy_from_user(user_params, (void __user *)ioctl_param,
				   sizeof(struct quadd_parameters))) {
			pr_err("setup failed\n");
			vfree(user_params);
			err = -EFAULT;
			goto error_out;
		}

		err = comm_ctx.control->set_parameters(user_params);
		if (err) {
			pr_err("error: setup failed\n");
			vfree(user_params);
			goto error_out;
		}

		if (user_params->reserved[QUADD_PARAM_IDX_SIZE_OF_RB] == 0) {
			pr_err("error: too old version of daemon\n");
			vfree(user_params);
			err = -EINVAL;
			goto error_out;
		}

		comm_ctx.params_ok = 1;

		pr_info("setup success: freq/mafreq: %u/%u, backtrace: %d, pid: %d\n",
			user_params->freq,
			user_params->ma_freq,
			user_params->backtrace,
			user_params->pids[0]);

		vfree(user_params);
		break;

	case IOCTL_GET_CAP:
		memset(&cap, 0, sizeof(cap));
		comm_ctx.control->get_capabilities(&cap);
		if (copy_to_user((void __user *)ioctl_param, &cap,
				 sizeof(struct quadd_comm_cap))) {
			pr_err("error: get_capabilities failed\n");
			err = -EFAULT;
			goto error_out;
		}
		break;

	case IOCTL_GET_CAP_FOR_CPU:
		per_cpu_cap = vmalloc(sizeof(*per_cpu_cap));
		if (!per_cpu_cap) {
			err = -ENOMEM;
			goto error_out;
		}

		if (copy_from_user(per_cpu_cap, (void __user *)ioctl_param,
				   sizeof(*per_cpu_cap))) {
			pr_err("setup failed\n");
			vfree(per_cpu_cap);
			err = -EFAULT;
			goto error_out;
		}

		cpuid = per_cpu_cap->cpuid;

		if (!cpu_possible(cpuid)) {
			vfree(per_cpu_cap);
			err = -EINVAL;
			goto error_out;
		}

		comm_ctx.control->get_capabilities_for_cpu(cpuid, per_cpu_cap);

		if (copy_to_user((void __user *)ioctl_param, per_cpu_cap,
				 sizeof(*per_cpu_cap))) {
			pr_err("error: get_capabilities failed\n");
			vfree(per_cpu_cap);
			err = -EFAULT;
			goto error_out;
		}

		vfree(per_cpu_cap);
		break;

	case IOCTL_GET_VERSION:
		memset(&versions, 0, sizeof(versions));

		strlcpy((char *)versions.branch, QUADD_MODULE_BRANCH,
			sizeof(versions.branch));
		strlcpy((char *)versions.version, QUADD_MODULE_VERSION,
			sizeof(versions.version));

		versions.samples_version = QUADD_SAMPLES_VERSION;
		versions.io_version = QUADD_IO_VERSION;

		if (copy_to_user((void __user *)ioctl_param, &versions,
				 sizeof(struct quadd_module_version))) {
			pr_err("error: get version failed\n");
			err = -EFAULT;
			goto error_out;
		}
		break;

	case IOCTL_GET_STATE:
		memset(&state, 0, sizeof(state));
		comm_ctx.control->get_state(&state);

		state.buffer_size = 0;
		state.buffer_fill_size = get_data_size();
		state.reserved[QUADD_MOD_STATE_IDX_RB_MAX_FILL_COUNT] = 0;

		if (copy_to_user((void __user *)ioctl_param, &state,
				 sizeof(struct quadd_module_state))) {
			pr_err("error: get_state failed\n");
			err = -EFAULT;
			goto error_out;
		}
		break;

	case IOCTL_START:
		if (!atomic_cmpxchg(&comm_ctx.active, 0, 1)) {
			err = comm_ctx.control->start();
			if (err) {
				pr_err("error: start failed\n");
				atomic_set(&comm_ctx.active, 0);
				goto error_out;
			}
			pr_info("Start profiling: success\n");
		}
		break;

	case IOCTL_STOP:
		if (atomic_cmpxchg(&comm_ctx.active, 1, 0)) {
			reset_params_ok_flag();
			comm_ctx.control->stop();
			rb_stop();
			pr_info("Stop profiling: success\n");
		}
		break;

	case IOCTL_SET_SECTIONS_INFO:
		if (!atomic_read(&comm_ctx.active)) {
			err = -EPERM;
			goto error_out;
		}

		if (copy_from_user(&extabs, (void __user *)ioctl_param,
				   sizeof(extabs))) {
			pr_err("error: set_sections_info failed\n");
			err = -EFAULT;
			goto error_out;
		}

		mmap_start = (unsigned long)extabs.user_mmap_start;
		file_hash = extabs.file_hash;

		pr_debug("%s: mmap_start: %#lx, hash: %#x, sec: %#lx - %#lx\n",
			 __func__, mmap_start, file_hash,
			 (unsigned long)extabs.vm_start,
			 (unsigned long)extabs.vm_end);

		raw_spin_lock(&comm_ctx.ctx->mmaps_lock);

		mmap = mmap_start ? find_mmap_by_vma(mmap_start) :
			find_mmap_by_hash(file_hash);
		if (!mmap) {
			pr_err("%s: error: !mmap, start/hash: %#lx/%#x\n",
			       __func__, mmap_start, file_hash);
			err = -ENXIO;
			raw_spin_unlock(&comm_ctx.ctx->mmaps_lock);
			goto error_out;
		}

		if ((mmap_start && mmap->type != QUADD_MMAP_TYPE_NONE) ||
		    (!mmap_start && mmap->type != QUADD_MMAP_TYPE_EXTABS)) {
			pr_err("%s: wrong mmap: %d, start/hash: %#lx/%#x\n",
				__func__, mmap->type, mmap_start, file_hash);
			err = -ENXIO;
			raw_spin_unlock(&comm_ctx.ctx->mmaps_lock);
			goto error_out;
		}

		mmap->type = QUADD_MMAP_TYPE_EXTABS;
		mmap->rb = NULL;

		if (mmap_start)
			mmap->fi.file_hash = extabs.file_hash;

		err = comm_ctx.control->set_extab(&extabs, mmap);
		raw_spin_unlock(&comm_ctx.ctx->mmaps_lock);
		if (err) {
			pr_err("error: set_sections_info\n");
			goto error_out;
		}
		break;

	case IOCTL_SET_MMAP_RB:
		if (copy_from_user(&mmap_rb, (void __user *)ioctl_param,
				   sizeof(mmap_rb))) {
			err = -EFAULT;
			goto error_out;
		}

		raw_spin_lock(&comm_ctx.ctx->mmaps_lock);

		mmap = find_mmap_by_vma((unsigned long)mmap_rb.vm_start);
		if (!mmap) {
			pr_err("set_mmap_rb: mmap is not found, start: %#lx\n",
			       (unsigned long)mmap_rb.vm_start);
			err = -ENXIO;
			raw_spin_unlock(&comm_ctx.ctx->mmaps_lock);
			goto error_out;
		}

		if (mmap->type != QUADD_MMAP_TYPE_NONE) {
			pr_err("set_mmap_rb: wrong mmap: %d, start: %#lx\n",
			       mmap->type, mmap->mmap_vma->vm_start);
			err = -ENXIO;
			raw_spin_unlock(&comm_ctx.ctx->mmaps_lock);
			goto error_out;
		}

		mmap->type = QUADD_MMAP_TYPE_RB;

		err = init_mmap_hdr(&mmap_rb, mmap);
		raw_spin_unlock(&comm_ctx.ctx->mmaps_lock);
		if (err) {
			pr_err("set_mmap_rb: error: init_mmap_hdr\n");
			goto error_out;
		}

		break;

	default:
		pr_err("error: ioctl %u is unsupported in this version of module\n",
		       ioctl_num);
		err = -EFAULT;
		goto error_out;
	}

error_out:
	mutex_unlock(&comm_ctx.io_mutex);
	return err;
}

static void
remove_mmap_entry(struct quadd_mmap_area *mmap)
{
	struct quadd_mmap_area *entry, *next;

	list_for_each_entry_safe(entry, next, &comm_ctx.ctx->mmap_areas, list) {
		if (entry == mmap) {
			list_del(&entry->list);
			break;
		}
	}
}

static void mmap_open(struct vm_area_struct *vma)
{
	pr_debug("%s: mmap_open: vma: %#lx - %#lx\n",
		__func__, vma->vm_start, vma->vm_end);
}

static void mmap_close(struct vm_area_struct *vma)
{
	struct quadd_mmap_area *mmap;

	raw_spin_lock(&comm_ctx.ctx->mmaps_lock);

	mmap = find_mmap_by_vma(vma->vm_start);
	if (!mmap) {
		pr_err("%s: error: mmap is not found: vma: %#lx - %#lx\n",
			__func__, vma->vm_start, vma->vm_end);
		goto out;
	}

	pr_debug("%s: mmap: %p: type: %d, %#lx - %#lx, data: %p\n",
		__func__, mmap, mmap->type, mmap->mmap_vma->vm_start,
		mmap->mmap_vma->vm_end, mmap->data);

	if (mmap->type == QUADD_MMAP_TYPE_EXTABS)
		comm_ctx.control->delete_mmap(mmap);
	else if (mmap->type == QUADD_MMAP_TYPE_RB)
		rb_reset(mmap->rb);
	else
		pr_warn("warning: mmap area is uninitialized\n");

	remove_mmap_entry(mmap);

out:
	raw_spin_unlock(&comm_ctx.ctx->mmaps_lock);

	if (mmap) {
		vfree(mmap->data);
		kfree(mmap);
	}
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
static int mmap_fault(struct vm_fault *vmf)
#else
static int mmap_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
#endif
{
	void *data;
	struct quadd_mmap_area *mmap;
	unsigned long offset = vmf->pgoff << PAGE_SHIFT;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
	struct vm_area_struct *vma = vmf->vma;
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
	pr_debug("mmap_fault: vma: %#lx - %#lx, pgoff: %#lx, vaddr: %#lx\n",
		 vma->vm_start, vma->vm_end, vmf->pgoff, vmf->address);
#else
	pr_debug("mmap_fault: vma: %#lx - %#lx, pgoff: %#lx, vaddr: %p\n",
		 vma->vm_start, vma->vm_end, vmf->pgoff, vmf->virtual_address);
#endif

	raw_spin_lock(&comm_ctx.ctx->mmaps_lock);

	mmap = find_mmap_by_vma(vma->vm_start);
	if (!mmap) {
		raw_spin_unlock(&comm_ctx.ctx->mmaps_lock);
		return VM_FAULT_SIGBUS;
	}

	data = mmap->data;

	vmf->page = vmalloc_to_page(data + offset);
	get_page(vmf->page);

	raw_spin_unlock(&comm_ctx.ctx->mmaps_lock);
	return 0;
}

static const struct vm_operations_struct mmap_vm_ops = {
	.open	= mmap_open,
	.close	= mmap_close,
	.fault	= mmap_fault,
};

static int
device_mmap(struct file *filp, struct vm_area_struct *vma)
{
	unsigned long vma_size, nr_pages;
	struct quadd_mmap_area *entry;

	if (vma->vm_pgoff != 0)
		return -EINVAL;

	vma->vm_private_data = filp->private_data;

	vma_size = vma->vm_end - vma->vm_start;
	nr_pages = vma_size / PAGE_SIZE;

	entry = kzalloc(sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	entry->mmap_vma = vma;

	atomic_set(&entry->state, QUADD_MMAP_STATE_ACTIVE);
	atomic_set(&entry->ref_count, 0);
	raw_spin_lock_init(&entry->state_lock);

	INIT_LIST_HEAD(&entry->list);
	INIT_LIST_HEAD(&entry->ex_entries);

	entry->data = vmalloc_user(nr_pages * PAGE_SIZE);
	if (!entry->data) {
		pr_err("%s: error: vmalloc_user", __func__);
		kfree(entry);
		return -ENOMEM;
	}

	entry->type = QUADD_MMAP_TYPE_NONE;

	pr_debug("%s: mmap: %p, vma: %#lx - %#lx, data: %p\n",
		 __func__, entry, vma->vm_start, vma->vm_end, entry->data);

	raw_spin_lock(&comm_ctx.ctx->mmaps_lock);
	list_add_tail(&entry->list, &comm_ctx.ctx->mmap_areas);
	raw_spin_unlock(&comm_ctx.ctx->mmaps_lock);

	vma->vm_ops = &mmap_vm_ops;
	vma->vm_flags |= VM_DONTCOPY | VM_DONTEXPAND | VM_DONTDUMP;

	vma->vm_ops->open(vma);

	return 0;
}

static void unregister(void)
{
	misc_deregister(comm_ctx.misc_dev);
	kfree(comm_ctx.misc_dev);
}

static const struct file_operations qm_fops = {
	.open		= device_open,
	.release	= device_release,
	.unlocked_ioctl	= device_ioctl,
	.compat_ioctl	= device_ioctl,
	.mmap		= device_mmap,
};

static int comm_init(void)
{
	int res, cpu_id;
	struct miscdevice *misc_dev;

	misc_dev = kzalloc(sizeof(*misc_dev), GFP_KERNEL);
	if (!misc_dev)
		return -ENOMEM;

	misc_dev->minor = MISC_DYNAMIC_MINOR;
	misc_dev->name = QUADD_DEVICE_NAME;
	misc_dev->fops = &qm_fops;

	res = misc_register(misc_dev);
	if (res < 0) {
		pr_err("Error: misc_register: %d\n", res);
		kfree(misc_dev);
		return res;
	}
	comm_ctx.misc_dev = misc_dev;

	mutex_init(&comm_ctx.io_mutex);
	atomic_set(&comm_ctx.active, 0);

	comm_ctx.nr_users = 0;

	INIT_LIST_HEAD(&comm_ctx.ctx->mmap_areas);
	raw_spin_lock_init(&comm_ctx.ctx->mmaps_lock);

	for_each_possible_cpu(cpu_id) {
		struct comm_cpu_context *cc = &per_cpu(cpu_ctx, cpu_id);
		struct quadd_ring_buffer *rb = &cc->rb;

		rb->mmap = NULL;
		rb->buf = NULL;
		rb->rb_hdr = NULL;

		rb->max_fill_count = 0;
		rb->nr_skipped_samples = 0;

		raw_spin_lock_init(&rb->lock);
	}

	reset_params_ok_flag();

	return 0;
}

struct quadd_comm_data_interface *
quadd_comm_init(struct quadd_ctx *ctx,
		struct quadd_comm_control_interface *control)
{
	int err;

	comm_ctx.ctx = ctx;

	err = comm_init();
	if (err < 0)
		return ERR_PTR(err);

	comm_ctx.control = control;
	return &comm_data;
}

void quadd_comm_exit(void)
{
	mutex_lock(&comm_ctx.io_mutex);
	unregister();
	mutex_unlock(&comm_ctx.io_mutex);
}
