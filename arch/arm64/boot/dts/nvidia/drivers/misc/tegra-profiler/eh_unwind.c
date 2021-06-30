/*
 * drivers/misc/tegra-profiler/eh_unwind.c
 *
 * Copyright (c) 2015-2019, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/mm.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/err.h>
#include <linux/rcupdate.h>

#include <linux/tegra_profiler.h>

#include "quadd.h"
#include "hrt.h"
#include "tegra.h"
#include "eh_unwind.h"
#include "backtrace.h"
#include "comm.h"
#include "dwarf_unwind.h"
#include "disassembler.h"

#define QUADD_EXTABS_SIZE	32

#define GET_NR_PAGES(a, l) \
	((PAGE_ALIGN((a) + (l)) - ((a) & PAGE_MASK)) / PAGE_SIZE)

enum regs {
	FP_THUMB = 7,
	FP_ARM = 11,

	SP = 13,
	LR = 14,
	PC = 15
};

struct regions_data {
	struct ex_region_info *entries;

	unsigned long nr_entries;
	unsigned long size;

	pid_t pid;

	struct rcu_head rcu;

	struct list_head list;
};

struct quadd_unwind_ctx {
	struct quadd_ctx *quadd_ctx;

	unsigned long ex_tables_size;

	struct list_head mm_ex_list;
	raw_spinlock_t mm_ex_list_lock;
};

struct unwind_idx {
	u32 addr_offset;
	u32 insn;
};

struct stackframe {
	unsigned long fp_thumb;
	unsigned long fp_arm;

	unsigned long sp;
	unsigned long lr;
	unsigned long pc;
};

struct unwind_ctrl_block {
	u32 vrs[16];		/* virtual register set */
	const u32 *insn;	/* pointer to the current instr word */
	int entries;		/* number of entries left */
	int byte;		/* current byte in the instr word */
};

struct pin_pages_work {
	struct work_struct work;
	unsigned long vm_start;
};

struct ex_entry_node {
	struct list_head list;
	pid_t pid;
	unsigned long vm_start;
	unsigned long vm_end;
};

static struct quadd_unwind_ctx ctx;

static inline int is_debug_frame(int secid)
{
	return (secid == QUADD_SEC_TYPE_DEBUG_FRAME ||
		secid == QUADD_SEC_TYPE_DEBUG_FRAME_HDR) ? 1 : 0;
}

unsigned long
get_ex_sec_address(struct ex_region_info *ri, struct extab_info *ti, int secid)
{
	struct quadd_mmap_area *mmap;
	unsigned long res = ti->addr;

	mmap = ri->mmap;
	if (unlikely(!mmap)) {
		pr_warn_once("%s: !mmap\n", __func__);
		return 0;
	}

	if (!(is_debug_frame(secid) && !mmap->fi.is_shared))
		res += ri->vm_start;

	return res;
}

static inline int
validate_mmap_addr(struct quadd_mmap_area *mmap,
		   unsigned long addr, unsigned long nbytes)
{
	struct vm_area_struct *vma;
	unsigned long data, size;

	if (atomic_read(&mmap->state) != QUADD_MMAP_STATE_ACTIVE)
		return 0;

	vma = mmap->mmap_vma;
	size = vma->vm_end - vma->vm_start;
	data = (unsigned long)mmap->data;

	if (addr & 0x03) {
		pr_err_once("%s: error: unaligned address: %#lx, data: %#lx-%#lx, vma: %#lx-%#lx\n",
			    __func__, addr, data, data + size,
			    vma->vm_start, vma->vm_end);
		return 0;
	}

	if (addr < data || addr >= data + (size - nbytes)) {
		pr_err_once("%s: error: addr: %#lx, data: %#lx-%#lx, vma: %#lx-%#lx\n",
			    __func__, addr, data, data + size,
			    vma->vm_start, vma->vm_end);
		return 0;
	}

	return 1;
}

static inline long
read_mmap_data(struct quadd_mmap_area *mmap, const u32 *addr, u32 *retval)
{
	if (!validate_mmap_addr(mmap, (unsigned long)addr, sizeof(u32))) {
		*retval = 0;
		return -QUADD_URC_EACCESS;
	}

	*retval = *addr;
	return 0;
}

static inline unsigned long
ex_addr_to_mmap_addr(unsigned long addr,
		     struct ex_region_info *ri,
		     int sec_type)
{
	unsigned long offset;
	struct extab_info *ti;
	struct quadd_mmap_area *mmap;

	mmap = ri->mmap;
	if (unlikely(!mmap)) {
		pr_warn_once("%s: !mmap\n", __func__);
		return 0;
	}

	ti = &mmap->fi.ex_sec[sec_type];
	if (unlikely(!ti->length))
		return 0;

	offset = addr - get_ex_sec_address(ri, ti, sec_type);
	return ti->mmap_offset + offset + (unsigned long)mmap->data;
}

static inline unsigned long
mmap_addr_to_ex_addr(unsigned long addr,
		     struct ex_region_info *ri,
		     int sec_type)
{
	unsigned long offset;
	struct extab_info *ti;
	struct quadd_mmap_area *mmap;

	mmap = ri->mmap;
	if (unlikely(!mmap)) {
		pr_warn_once("%s: !mmap\n", __func__);
		return 0;
	}

	ti = &mmap->fi.ex_sec[sec_type];
	if (unlikely(!ti->length))
		return 0;

	offset = addr - ti->mmap_offset - (unsigned long)mmap->data;
	return get_ex_sec_address(ri, ti, sec_type) + offset;
}

static inline u32 __maybe_unused
prel31_to_addr(const u32 *ptr)
{
	long err;
	u32 value;
	s32 offset;

	err = read_user_data(&value, ptr, sizeof(*ptr));
	if (err < 0)
		return 0;

	/* sign-extend to 32 bits */
	offset = (((s32)value) << 1) >> 1;
	return (u32)(unsigned long)ptr + offset;
}

static unsigned long
mmap_prel31_to_addr(const u32 *ptr, struct ex_region_info *ri,
		    int src_type, int dst_type, int to_mmap)
{
	s32 offset;
	u32 value, addr;
	unsigned long addr_res;

	value = *ptr;
	offset = (((s32)value) << 1) >> 1;

	addr = mmap_addr_to_ex_addr((unsigned long)ptr, ri, src_type);
	if (unlikely(!addr))
		return 0;

	addr += offset;
	addr_res = addr;

	if (to_mmap)
		addr_res = ex_addr_to_mmap_addr(addr_res, ri, dst_type);

	return addr_res;
}

static struct regions_data *rd_alloc(unsigned long size)
{
	struct regions_data *rd;

	rd = kzalloc(sizeof(*rd), GFP_ATOMIC);
	if (!rd)
		return ERR_PTR(-ENOMEM);

	rd->entries = kcalloc(size, sizeof(*rd->entries), GFP_ATOMIC);
	if (!rd->entries) {
		kfree(rd);
		return ERR_PTR(-ENOMEM);
	}

	rd->size = size;
	rd->nr_entries = 0;

	return rd;
}

static void rd_free(struct regions_data *rd)
{
	if (rd)
		kfree(rd->entries);

	kfree(rd);
}

static void mm_ex_list_free_rcu(struct rcu_head *head)
{
	struct regions_data *entry =
		container_of(head, struct regions_data, rcu);

	rd_free(entry);
}

static int
add_ex_region(struct regions_data *rd,
	      struct ex_region_info *new_entry)
{
	unsigned int i_min, i_max, mid;
	struct ex_region_info *array = rd->entries;
	unsigned long size = rd->nr_entries;

	if (!array)
		return -ENOMEM;

	if (size == 0) {
		memcpy(&array[0], new_entry, sizeof(*new_entry));
		return 0;
	} else if (size == 1 && array[0].vm_start == new_entry->vm_start) {
		return -EEXIST;
	}

	i_min = 0;
	i_max = size;

	if (array[0].vm_start > new_entry->vm_start) {
		memmove(array + 1, array,
			size * sizeof(*array));
		memcpy(&array[0], new_entry, sizeof(*new_entry));
		return 0;
	} else if (array[size - 1].vm_start < new_entry->vm_start) {
		memcpy(&array[size], new_entry, sizeof(*new_entry));
		return 0;
	}

	while (i_min < i_max) {
		mid = i_min + (i_max - i_min) / 2;

		if (new_entry->vm_start <= array[mid].vm_start)
			i_max = mid;
		else
			i_min = mid + 1;
	}

	if (array[i_max].vm_start == new_entry->vm_start)
		return -EEXIST;

	memmove(array + i_max + 1,
		array + i_max,
		(size - i_max) * sizeof(*array));
	memcpy(&array[i_max], new_entry, sizeof(*new_entry));

	return 0;
}

static int
remove_ex_region(struct regions_data *rd,
		 struct ex_region_info *entry)
{
	unsigned int i_min, i_max, mid;
	struct ex_region_info *array = rd->entries;
	unsigned long size = rd->nr_entries;

	if (!array)
		return 0;

	if (size == 0)
		return 0;

	if (size == 1) {
		if (array[0].vm_start == entry->vm_start)
			return 1;
		else
			return 0;
	}

	if (array[0].vm_start > entry->vm_start)
		return 0;
	else if (array[size - 1].vm_start < entry->vm_start)
		return 0;

	i_min = 0;
	i_max = size;

	while (i_min < i_max) {
		mid = i_min + (i_max - i_min) / 2;

		if (entry->vm_start <= array[mid].vm_start)
			i_max = mid;
		else
			i_min = mid + 1;
	}

	if (array[i_max].vm_start == entry->vm_start) {
		memmove(array + i_max,
			array + i_max + 1,
			(size - i_max) * sizeof(*array));
		return 1;
	} else {
		return 0;
	}
}

static struct ex_region_info *
__search_ex_region(struct ex_region_info *array,
		   unsigned long size,
		   unsigned long key)
{
	unsigned int i_min, i_max, mid;

	if (size == 0)
		return NULL;

	i_min = 0;
	i_max = size;

	while (i_min < i_max) {
		mid = i_min + (i_max - i_min) / 2;

		if (key <= array[mid].vm_start)
			i_max = mid;
		else
			i_min = mid + 1;
	}

	if (array[i_max].vm_start == key)
		return &array[i_max];

	return NULL;
}

static long
search_ex_region(pid_t pid, unsigned long key, struct ex_region_info *ri)
{
	struct regions_data *entry;
	struct ex_region_info *ri_p = NULL;

	rcu_read_lock();
	list_for_each_entry_rcu(entry, &ctx.mm_ex_list, list) {
		if (entry->pid == pid) {
			ri_p = __search_ex_region(entry->entries,
						  entry->nr_entries, key);
			if (ri_p)
				memcpy(ri, ri_p, sizeof(*ri));
			break;
		}
	}
	rcu_read_unlock();

	return ri_p ? 0 : -ENOENT;
}

static inline int
validate_sections(struct quadd_sections *et)
{
	int i;
	unsigned long size = 0;

	if (et->vm_start >= et->vm_end)
		return -EINVAL;

	for (i = 0; i < QUADD_SEC_TYPE_MAX; i++)
		size += et->sec[i].length;

	return size < et->vm_end - et->vm_start;
}

static void
mmap_ex_entry_del(struct quadd_mmap_area *mmap,
		  pid_t pid, unsigned long vm_start)
{
	struct ex_entry_node *e, *n;

	list_for_each_entry_safe(e, n, &mmap->ex_entries, list) {
		if (e->pid == pid && e->vm_start == vm_start) {
			list_del(&e->list);
			kfree(e);
			break;
		}
	}
}

static int
is_overlapped(unsigned long a_start, unsigned long a_end,
	      unsigned long b_start, unsigned long b_end)
{
	return ((a_start >= b_start && a_start < b_end) ||
		(b_start >= a_start && b_start < a_end));
}

static int
remove_overlapped_regions(struct regions_data *rd, struct ex_region_info *ri)
{
	long idx_from = -1, idx_to = -1;
	unsigned long i, start, end, nr, nr_removed = 0;
	struct ex_region_info *array = rd->entries;

	nr = rd->nr_entries;
	if (nr == 0)
		return 0;

	for (i = 0; i < nr; i++) {
		start = array[i].vm_start;
		end = array[i].vm_end;

		if (is_overlapped(start, end, ri->vm_start, ri->vm_end)) {
			idx_from = idx_to = i;
			for (idx_to = i + 1; idx_to < nr; idx_to++) {
				start = array[idx_to].vm_start;
				end = array[idx_to].vm_end;
				if (!is_overlapped(start, end, ri->vm_start,
						   ri->vm_end))
					break;
			}
			break;
		}
	}

	if (idx_from >= 0) {
		struct ex_region_info *ri_del;
		unsigned long nr_copied = nr - idx_to;

		nr_removed = idx_to - idx_from;

		pr_debug("%s: [%u] new: %#lx-%#lx, rm:%#lx-%#lx...%#lx-%#lx\n",
			__func__, rd->pid, ri->vm_start, ri->vm_end,
			array[idx_from].vm_start, array[idx_from].vm_end,
			array[idx_to - 1].vm_start, array[idx_to - 1].vm_end);

		for (i = idx_from; i < idx_to; i++) {
			ri_del = &array[i];
			mmap_ex_entry_del(ri_del->mmap, rd->pid,
					  ri_del->vm_start);
		}

		if (nr_copied > 0)
			memmove(array + idx_from, array + idx_to,
				nr_copied * sizeof(*array));

		rd->nr_entries -= nr_removed;
	}

	return nr_removed;
}

static int
mm_ex_list_add(struct ex_region_info *ri, pid_t pid)
{
	int err = 0;
	struct regions_data *entry, *rd, *rd_old = NULL;
	unsigned long nr_entries_new;

	raw_spin_lock(&ctx.mm_ex_list_lock);

	list_for_each_entry(entry, &ctx.mm_ex_list, list) {
		if (entry->pid == pid) {
			rd_old = entry;
			break;
		}
	}

	nr_entries_new = rd_old ? rd_old->nr_entries + 1 : 1;

	rd = rd_alloc(nr_entries_new);
	if (IS_ERR(rd)) {
		err = PTR_ERR(rd);
		goto out_unlock;
	}

	if (rd_old) {
		memcpy(rd->entries, rd_old->entries,
		       rd_old->nr_entries * sizeof(*rd_old->entries));
		rd->nr_entries = rd_old->nr_entries;
		rd->pid = pid;
		remove_overlapped_regions(rd, ri);
	}

	err = add_ex_region(rd, ri);
	if (err < 0)
		goto out_free;

	rd->nr_entries++;

	rd->pid = pid;
	INIT_LIST_HEAD(&rd->list);

	if (rd_old) {
		list_replace_rcu(&rd_old->list, &rd->list);
		call_rcu(&rd_old->rcu, mm_ex_list_free_rcu);
	} else {
		list_add_tail_rcu(&rd->list, &ctx.mm_ex_list);
	}

	raw_spin_unlock(&ctx.mm_ex_list_lock);
	return 0;

out_free:
	rd_free(rd);
out_unlock:
	raw_spin_unlock(&ctx.mm_ex_list_lock);

	return err;
}

static int
mm_ex_list_del(unsigned long vm_start, pid_t pid)
{
	int err = 0, nr_removed;
	unsigned long nr_entries;
	struct regions_data *rd_entry, *rd_new;
	struct ex_region_info ex_entry;

	ex_entry.vm_start = vm_start;

	raw_spin_lock(&ctx.mm_ex_list_lock);

	list_for_each_entry(rd_entry, &ctx.mm_ex_list, list) {
		if (rd_entry->pid == pid) {
			nr_entries = rd_entry->nr_entries;

			if (unlikely(nr_entries == 0)) {
				pr_warn_once("%s: !nr_entries\n", __func__);
				err = -ENOENT;
				goto out_unlock;
			}

			rd_new = rd_alloc(nr_entries);
			if (IS_ERR(rd_new)) {
				err = PTR_ERR(rd_new);
				goto out_unlock;
			}

			memcpy(rd_new->entries, rd_entry->entries,
			       nr_entries * sizeof(*rd_entry->entries));
			rd_new->nr_entries = nr_entries;

			rd_new->pid = pid;
			INIT_LIST_HEAD(&rd_new->list);

			nr_removed = remove_ex_region(rd_new, &ex_entry);
			rd_new->nr_entries -= nr_removed;

			if (rd_new->nr_entries > 0) {
				list_replace_rcu(&rd_entry->list,
						 &rd_new->list);
				call_rcu(&rd_entry->rcu, mm_ex_list_free_rcu);
			} else {
				rd_free(rd_new);
				list_del_rcu(&rd_entry->list);
				call_rcu(&rd_entry->rcu, mm_ex_list_free_rcu);
			}
		}
	}

out_unlock:
	raw_spin_unlock(&ctx.mm_ex_list_lock);

	return err;
}

static long
get_extabs_ehabi(pid_t pid, unsigned long key, struct ex_region_info *ri)
{
	long err = 0;
	struct extab_info *ti_exidx;
	struct quadd_mmap_area *mmap;

	err = search_ex_region(pid, key, ri);
	if (err < 0)
		return err;

	mmap = ri->mmap;

	raw_spin_lock(&mmap->state_lock);

	if (atomic_read(&mmap->state) != QUADD_MMAP_STATE_ACTIVE) {
		err = -ENOENT;
		goto out;
	}

	ti_exidx = &mmap->fi.ex_sec[QUADD_SEC_TYPE_EXIDX];

	if (ti_exidx->length)
		atomic_inc(&mmap->ref_count);
	else
		err = -ENOENT;

out:
	raw_spin_unlock(&mmap->state_lock);
	return err;
}

static void put_extabs_ehabi(struct ex_region_info *ri)
{
	struct quadd_mmap_area *mmap = ri->mmap;

	raw_spin_lock(&mmap->state_lock);

	if (atomic_dec_and_test(&mmap->ref_count) &&
	    atomic_read(&mmap->state) == QUADD_MMAP_STATE_CLOSING)
		atomic_set(&mmap->state, QUADD_MMAP_STATE_CLOSED);

	raw_spin_unlock(&mmap->state_lock);

	if (atomic_read(&mmap->ref_count) < 0)
		pr_err_once("%s: error: mmap ref_count\n", __func__);
}

long
quadd_get_dw_frames(unsigned long key, struct ex_region_info *ri,
		    struct task_struct *task)
{
	pid_t pid;
	long err = 0;
	struct extab_info *ti, *ti_hdr;
	struct quadd_mmap_area *mmap;

	pid = task_tgid_nr(task);

	err = search_ex_region(pid, key, ri);
	if (err < 0)
		return err;

	mmap = ri->mmap;

	raw_spin_lock(&mmap->state_lock);

	if (atomic_read(&mmap->state) != QUADD_MMAP_STATE_ACTIVE) {
		err = -ENOENT;
		goto out;
	}

	ti = &mmap->fi.ex_sec[QUADD_SEC_TYPE_EH_FRAME];
	ti_hdr = &mmap->fi.ex_sec[QUADD_SEC_TYPE_EH_FRAME_HDR];

	if (ti->length && ti_hdr->length) {
		atomic_inc(&mmap->ref_count);
		goto out;
	}

	ti = &mmap->fi.ex_sec[QUADD_SEC_TYPE_DEBUG_FRAME];
	ti_hdr = &mmap->fi.ex_sec[QUADD_SEC_TYPE_DEBUG_FRAME_HDR];

	if (ti->length && ti_hdr->length)
		atomic_inc(&mmap->ref_count);
	else
		err = -ENOENT;

out:
	raw_spin_unlock(&mmap->state_lock);
	return err;
}

void quadd_put_dw_frames(struct ex_region_info *ri)
{
	struct quadd_mmap_area *mmap = ri->mmap;

	raw_spin_lock(&mmap->state_lock);

	if (atomic_dec_and_test(&mmap->ref_count) &&
	    atomic_read(&mmap->state) == QUADD_MMAP_STATE_CLOSING)
		atomic_set(&mmap->state, QUADD_MMAP_STATE_CLOSED);

	raw_spin_unlock(&mmap->state_lock);

	if (atomic_read(&mmap->ref_count) < 0)
		pr_err_once("%s: error: mmap ref_count\n", __func__);
}

int quadd_unwind_set_extab(struct quadd_sections *extabs,
			   struct quadd_mmap_area *mmap)
{
	int i, err;
	struct ex_region_info ri_entry;
	struct ex_entry_node *mmap_ex_entry;

	if (mmap->type != QUADD_MMAP_TYPE_EXTABS)
		return -EIO;

	err = validate_sections(extabs);
	if (err < 0)
		return err;

	if (extabs->user_mmap_start) {
		mmap->fi.is_shared =
			(extabs->flags & QUADD_SECTIONS_FLAG_IS_SHARED) != 0;

		for (i = 0; i < QUADD_SEC_TYPE_MAX; i++) {
			struct quadd_sec_info *si = &extabs->sec[i];
			struct extab_info *ti = &mmap->fi.ex_sec[i];

			ti->tf_start = 0;
			ti->tf_end = 0;

			if (!si->addr) {
				ti->addr = 0;
				ti->length = 0;
				ti->mmap_offset = 0;

				continue;
			}

			ti->addr = si->addr;
			ti->length = si->length;
			ti->mmap_offset = si->mmap_offset;
		}
	}

	ri_entry.vm_start = extabs->vm_start;
	ri_entry.vm_end = extabs->vm_end;
	ri_entry.file_hash = extabs->file_hash;
	ri_entry.mmap = mmap;

	mmap_ex_entry = kzalloc(sizeof(*mmap_ex_entry), GFP_ATOMIC);
	if (!mmap_ex_entry) {
		err = -ENOMEM;
		goto err_out;
	}

	mmap_ex_entry->vm_start = ri_entry.vm_start;
	mmap_ex_entry->vm_end = ri_entry.vm_end;
	mmap_ex_entry->pid = extabs->pid;

	err = mm_ex_list_add(&ri_entry, extabs->pid);
	if (err < 0)
		goto out_ex_entry_free;

	INIT_LIST_HEAD(&mmap_ex_entry->list);
	list_add_tail(&mmap_ex_entry->list, &mmap->ex_entries);

	pr_debug("%s: pid: %u: vma: %#lx - %#lx, file_hash: %#x\n",
		 __func__, extabs->pid, (unsigned long)extabs->vm_start,
		 (unsigned long)extabs->vm_end, extabs->file_hash);

	return 0;

out_ex_entry_free:
	kfree(mmap_ex_entry);
err_out:
	return err;
}

void
quadd_unwind_set_tail_info(struct ex_region_info *ri,
			   int secid,
			   unsigned long tf_start,
			   unsigned long tf_end,
			   struct task_struct *task)
{
	struct quadd_mmap_area *mmap;

	raw_spin_lock(&ctx.quadd_ctx->mmaps_lock);

	mmap = ri->mmap;
	mmap->fi.ex_sec[secid].tf_start = tf_start;
	mmap->fi.ex_sec[secid].tf_end = tf_end;

	pr_debug("%s: pid: %u, secid: %d, tf: %#lx - %#lx\n",
		 __func__, task_tgid_nr(task), secid, tf_start, tf_end);

	raw_spin_unlock(&ctx.quadd_ctx->mmaps_lock);
}

static void
clean_mmap(struct quadd_mmap_area *mmap)
{
	struct ex_entry_node *entry, *next;

	if (atomic_read(&mmap->ref_count)) {
		pr_warn_once("%s: ref_count != 0\n", __func__);
		return;
	}

	if (!mmap || mmap->type != QUADD_MMAP_TYPE_EXTABS)
		return;

	list_for_each_entry_safe(entry, next, &mmap->ex_entries, list) {
		mm_ex_list_del(entry->vm_start, entry->pid);
		list_del(&entry->list);
		kfree(entry);
	}
}

static void mmap_wait_for_close(struct quadd_mmap_area *mmap)
{
	int state;

	raw_spin_lock(&mmap->state_lock);

	state = atomic_read(&mmap->ref_count) > 0 ?
		QUADD_MMAP_STATE_CLOSING : QUADD_MMAP_STATE_CLOSED;
	atomic_set(&mmap->state, state);

	raw_spin_unlock(&mmap->state_lock);

	while (atomic_read(&mmap->state) != QUADD_MMAP_STATE_CLOSED)
		cpu_relax();
}

void quadd_unwind_clean_mmap(struct quadd_mmap_area *mmap)
{
	mmap_wait_for_close(mmap);
	clean_mmap(mmap);
}

static const struct unwind_idx *
unwind_find_idx(struct ex_region_info *ri, u32 addr, unsigned long *lowaddr)
{
	u32 value;
	unsigned long length;
	struct extab_info *ti;
	struct unwind_idx *start;
	struct unwind_idx *stop;
	struct unwind_idx *mid = NULL;

	ti = &ri->mmap->fi.ex_sec[QUADD_SEC_TYPE_EXIDX];
	length = ti->length / sizeof(*start);

	if (unlikely(!length))
		return NULL;

	start = (struct unwind_idx *)((char *)ri->mmap->data + ti->mmap_offset);
	stop = start + length - 1;

	value = (u32)mmap_prel31_to_addr(&start->addr_offset, ri,
					 QUADD_SEC_TYPE_EXIDX,
					 QUADD_SEC_TYPE_EXTAB, 0);
	if (!value || addr < value)
		return NULL;

	value = (u32)mmap_prel31_to_addr(&stop->addr_offset, ri,
					 QUADD_SEC_TYPE_EXIDX,
					 QUADD_SEC_TYPE_EXTAB, 0);
	if (!value || addr >= value)
		return NULL;

	while (start < stop - 1) {
		mid = start + ((stop - start) >> 1);

		value = (u32)mmap_prel31_to_addr(&mid->addr_offset, ri,
						 QUADD_SEC_TYPE_EXIDX,
						 QUADD_SEC_TYPE_EXTAB, 0);
		if (!value)
			return NULL;

		if (addr < value)
			stop = mid;
		else
			start = mid;
	}

	if (lowaddr)
		*lowaddr = mmap_prel31_to_addr(&start->addr_offset,
					       ri, 1, 0, 0);
	return start;
}

static unsigned long
unwind_get_byte(struct quadd_mmap_area *mmap,
		struct unwind_ctrl_block *ctrl, long *err)
{
	unsigned long ret;
	u32 insn_word;

	*err = 0;

	if (ctrl->entries <= 0) {
		pr_err_once("%s: error: corrupt unwind table\n", __func__);
		*err = -QUADD_URC_TBL_IS_CORRUPT;
		return 0;
	}

	*err = read_mmap_data(mmap, ctrl->insn, &insn_word);
	if (*err < 0)
		return 0;

	ret = (insn_word >> (ctrl->byte * 8)) & 0xff;

	if (ctrl->byte == 0) {
		ctrl->insn++;
		ctrl->entries--;
		ctrl->byte = 3;
	} else
		ctrl->byte--;

	return ret;
}

static long
read_uleb128(struct quadd_mmap_area *mmap,
	     struct unwind_ctrl_block *ctrl,
	     unsigned long *ret)
{
	long err = 0;
	unsigned long result;
	unsigned char byte;
	int shift, count;

	result = 0;
	shift = 0;
	count = 0;

	while (1) {
		byte = unwind_get_byte(mmap, ctrl, &err);
		if (err < 0)
			return err;

		count++;

		result |= (byte & 0x7f) << shift;
		shift += 7;

		if (!(byte & 0x80))
			break;
	}

	*ret = result;

	return count;
}

/*
 * Execute the current unwind instruction.
 */
static long
unwind_exec_insn(struct quadd_mmap_area *mmap,
		 struct unwind_ctrl_block *ctrl,
		 struct quadd_disasm_data *qd)
{
	long err;
	unsigned int i;
	unsigned long insn = unwind_get_byte(mmap, ctrl, &err);

	if (err < 0)
		return err;

	pr_debug("%s: insn = %08lx\n", __func__, insn);

	if ((insn & 0xc0) == 0x00) {
		ctrl->vrs[SP] += ((insn & 0x3f) << 2) + 4;
		qd->stacksize -= ((insn & 0x3f) << 2) + 4;

		pr_debug("CMD_DATA_POP: vsp = vsp + %lu (new: %#x)\n",
			((insn & 0x3f) << 2) + 4, ctrl->vrs[SP]);
	} else if ((insn & 0xc0) == 0x40) {
		ctrl->vrs[SP] -= ((insn & 0x3f) << 2) + 4;
		qd->stackoff -= ((insn & 0x3f) << 2) + 4;
		pr_debug("CMD_DATA_PUSH: vsp = vsp – %lu (new: %#x)\n",
			((insn & 0x3f) << 2) + 4, ctrl->vrs[SP]);
	} else if ((insn & 0xf0) == 0x80) {
		unsigned long mask;
		u32 __user *vsp = (u32 __user *)(unsigned long)ctrl->vrs[SP];
		int load_sp, reg = 4;

		insn = (insn << 8) | unwind_get_byte(mmap, ctrl, &err);
		if (err < 0)
			return err;

		mask = insn & 0x0fff;
		if (mask == 0) {
			pr_debug("CMD_REFUSED: unwind: 'Refuse to unwind' instruction %04lx\n",
				   insn);
			return -QUADD_URC_REFUSE_TO_UNWIND;
		}

		/* pop R4-R15 according to mask */
		load_sp = mask & (1 << (13 - 4));
		while (mask) {
			if (mask & 1) {
				err = read_user_data(&ctrl->vrs[reg], vsp++,
						     sizeof(u32));
				if (err < 0)
					return err;

				qd->r_regset &= ~(1 << reg);
				pr_debug("CMD_REG_POP: pop {r%d}\n", reg);
			}
			mask >>= 1;
			reg++;
		}
		if (!load_sp)
			ctrl->vrs[SP] = (unsigned long)vsp;

		pr_debug("new vsp: %#x\n", ctrl->vrs[SP]);
	} else if ((insn & 0xf0) == 0x90 &&
		   (insn & 0x0d) != 0x0d) {
		ctrl->vrs[SP] = ctrl->vrs[insn & 0x0f];
		qd->ustackreg = (insn & 0xf);
		pr_debug("CMD_REG_TO_SP: vsp = {r%lu}\n", insn & 0x0f);
	} else if ((insn & 0xf0) == 0xa0) {
		u32 __user *vsp = (u32 __user *)(unsigned long)ctrl->vrs[SP];
		unsigned int reg;

		/* pop R4-R[4+bbb] */
		for (reg = 4; reg <= 4 + (insn & 7); reg++) {
			err = read_user_data(&ctrl->vrs[reg], vsp++,
					     sizeof(u32));
			if (err < 0)
				return err;

			qd->r_regset &= ~(1 << reg);
			pr_debug("CMD_REG_POP: pop {r%u}\n", reg);
		}

		if (insn & 0x08) {
			err = read_user_data(&ctrl->vrs[14], vsp++,
					     sizeof(u32));
			if (err < 0)
				return err;

			qd->r_regset &= ~(1 << 14);
			pr_debug("CMD_REG_POP: pop {r14}\n");
		}

		ctrl->vrs[SP] = (u32)(unsigned long)vsp;
		pr_debug("new vsp: %#x\n", ctrl->vrs[SP]);
	} else if (insn == 0xb0) {
		if (ctrl->vrs[PC] == 0)
			ctrl->vrs[PC] = ctrl->vrs[LR];
		/* no further processing */
		ctrl->entries = 0;

		pr_debug("CMD_FINISH\n");
	} else if (insn == 0xb1) {
		unsigned long mask = unwind_get_byte(mmap, ctrl, &err);
		u32 __user *vsp = (u32 __user *)(unsigned long)ctrl->vrs[SP];
		int reg = 0;

		if (err < 0)
			return err;

		if (mask == 0 || mask & 0xf0) {
			pr_debug("unwind: Spare encoding %04lx\n",
			       (insn << 8) | mask);
			return -QUADD_URC_SPARE_ENCODING;
		}

		/* pop R0-R3 according to mask */
		while (mask) {
			if (mask & 1) {
				err = read_user_data(&ctrl->vrs[reg], vsp++,
						     sizeof(u32));
				if (err < 0)
					return err;

				qd->r_regset &= ~(1 << reg);
				pr_debug("CMD_REG_POP: pop {r%d}\n", reg);
			}
			mask >>= 1;
			reg++;
		}

		ctrl->vrs[SP] = (u32)(unsigned long)vsp;
		pr_debug("new vsp: %#x\n", ctrl->vrs[SP]);
	} else if (insn == 0xb2) {
		long count;
		unsigned long uleb128 = 0;

		count = read_uleb128(mmap, ctrl, &uleb128);
		if (count < 0)
			return count;

		if (count == 0)
			return -QUADD_URC_TBL_IS_CORRUPT;

		ctrl->vrs[SP] += 0x204 + (uleb128 << 2);

		qd->stacksize -= 0x204 + (uleb128 << 2);
		pr_debug("CMD_DATA_POP: vsp = vsp + %lu (%#lx), new vsp: %#x\n",
			 0x204 + (uleb128 << 2), 0x204 + (uleb128 << 2),
			 ctrl->vrs[SP]);
	} else if (insn == 0xb3 || insn == 0xc8 || insn == 0xc9) {
		unsigned long data, reg_from, reg_to;
		u32 __user *vsp = (u32 __user *)(unsigned long)ctrl->vrs[SP];

		data = unwind_get_byte(mmap, ctrl, &err);
		if (err < 0)
			return err;

		reg_from = (data & 0xf0) >> 4;
		reg_to = reg_from + (data & 0x0f);

		if (insn == 0xc8) {
			reg_from += 16;
			reg_to += 16;
		}

		for (i = reg_from; i <= reg_to; i++)
			vsp += 2, qd->d_regset &= ~(1 << i);

		if (insn == 0xb3)
			vsp++;

		ctrl->vrs[SP] = (u32)(unsigned long)vsp;

		pr_debug("CMD_VFP_POP (%#lx %#lx): pop {D%lu-D%lu}\n",
			 insn, data, reg_from, reg_to);
		pr_debug("new vsp: %#x\n", ctrl->vrs[SP]);
	} else if ((insn & 0xf8) == 0xb8 || (insn & 0xf8) == 0xd0) {
		unsigned long reg_to;
		unsigned long data = insn & 0x07;
		u32 __user *vsp = (u32 __user *)(unsigned long)ctrl->vrs[SP];

		reg_to = 8 + data;

		for (i = 8; i <= reg_to; i++)
			vsp += 2, qd->d_regset &= ~(1 << i);

		if ((insn & 0xf8) == 0xb8)
			vsp++;

		ctrl->vrs[SP] = (u32)(unsigned long)vsp;

		pr_debug("CMD_VFP_POP (%#lx): pop {D8-D%lu}\n",
			 insn, reg_to);
		pr_debug("new vsp: %#x\n", ctrl->vrs[SP]);
	} else {
		pr_debug("error: unhandled instruction %02lx\n", insn);
		return -QUADD_URC_UNHANDLED_INSTRUCTION;
	}

	pr_debug("%s: fp_arm: %#x, fp_thumb: %#x, sp: %#x, lr = %#x, pc: %#x\n",
		 __func__,
		 ctrl->vrs[FP_ARM], ctrl->vrs[FP_THUMB], ctrl->vrs[SP],
		 ctrl->vrs[LR], ctrl->vrs[PC]);

	return 0;
}

/*
 * Unwind a single frame starting with *sp for the symbol at *pc. It
 * updates the *pc and *sp with the new values.
 */
static long
unwind_frame(struct quadd_unw_methods um,
	     struct ex_region_info *ri,
	     struct stackframe *frame,
	     struct vm_area_struct *vma_sp,
	     int thumbflag,
	     struct task_struct *task)
{
	unsigned long high, low, min, max;
	const struct unwind_idx *idx;
	struct unwind_ctrl_block ctrl;
	struct quadd_disasm_data qd;
#ifdef QM_DEBUG_DISASSEMBLER
	struct quadd_disasm_data orig;
#endif
	long err = 0;
	u32 val;

	if (!validate_stack_addr(frame->sp, vma_sp, sizeof(u32), 0))
		return -QUADD_URC_SP_INCORRECT;

	/* only go to a higher address on the stack */
	low = frame->sp;
	high = vma_sp->vm_end;

	pr_debug("pc: %#lx, lr: %#lx, sp:%#lx, low/high: %#lx/%#lx, thumb: %d\n",
		 frame->pc, frame->lr, frame->sp, low, high, thumbflag);

	idx = unwind_find_idx(ri, frame->pc, &min);
	if (IS_ERR_OR_NULL(idx))
		return -QUADD_URC_IDX_NOT_FOUND;

	pr_debug("index was found by pc (%#lx): %p\n", frame->pc, idx);

	ctrl.vrs[FP_THUMB] = frame->fp_thumb;
	ctrl.vrs[FP_ARM] = frame->fp_arm;

	ctrl.vrs[SP] = frame->sp;
	ctrl.vrs[LR] = frame->lr;
	ctrl.vrs[PC] = 0;

	err = read_mmap_data(ri->mmap, &idx->insn, &val);
	if (err < 0)
		return err;

	if (val == 1) {
		/* can't unwind */
		return -QUADD_URC_CANTUNWIND;
	} else if ((val & 0x80000000) == 0) {
		/* prel31 to the unwind table */
		ctrl.insn = (u32 *)(unsigned long)
				mmap_prel31_to_addr(&idx->insn, ri,
						    QUADD_SEC_TYPE_EXIDX,
						    QUADD_SEC_TYPE_EXTAB, 1);
		if (!ctrl.insn)
			return -QUADD_URC_TBL_LINK_INCORRECT;
	} else if ((val & 0xff000000) == 0x80000000) {
		/* only personality routine 0 supported in the index */
		ctrl.insn = &idx->insn;
	} else {
		pr_debug("unsupported personality routine %#x in the index at %p\n",
			 val, idx);
		return -QUADD_URC_UNSUPPORTED_PR;
	}

	err = read_mmap_data(ri->mmap, ctrl.insn, &val);
	if (err < 0)
		return err;

	/* check the personality routine */
	if ((val & 0xff000000) == 0x80000000) {
		ctrl.byte = 2;
		ctrl.entries = 1;
	} else if ((val & 0xff000000) == 0x81000000) {
		ctrl.byte = 1;
		ctrl.entries = 1 + ((val & 0x00ff0000) >> 16);
	} else {
		pr_debug("unsupported personality routine %#x at %p\n",
			 val, ctrl.insn);
		return -QUADD_URC_UNSUPPORTED_PR;
	}

	if (um.ut_ce) {
		/* guess for the boundaries to disassemble */
		if (frame->pc - min < QUADD_DISASM_MIN)
			max = min + QUADD_DISASM_MIN;
		else
			max = (frame->pc - min < QUADD_DISASM_MAX)
				? frame->pc : min + QUADD_DISASM_MAX;
		err = quadd_disassemble(&qd, min, max, thumbflag);
		if (err < 0)
			return err;
#ifdef QM_DEBUG_DISASSEMBLER
		/* saved for verbose unwind mismatch reporting */
		orig = qd;
		qd.orig = &orig;
#endif
	}

	while (ctrl.entries > 0) {
		err = unwind_exec_insn(ri->mmap, &ctrl, &qd);
		if (err < 0)
			return err;

		if (ctrl.vrs[SP] & 0x03 ||
		    ctrl.vrs[SP] < low || ctrl.vrs[SP] >= high)
			return -QUADD_URC_SP_INCORRECT;
	}

	if (um.ut_ce && quadd_check_unwind_result(frame->pc, &qd) < 0)
		return -QUADD_URC_UNWIND_MISMATCH;

	if (ctrl.vrs[PC] == 0)
		ctrl.vrs[PC] = ctrl.vrs[LR];

	if (!validate_pc_addr(ctrl.vrs[PC], sizeof(u32)))
		return -QUADD_URC_PC_INCORRECT;

	frame->fp_thumb = ctrl.vrs[FP_THUMB];
	frame->fp_arm = ctrl.vrs[FP_ARM];

	frame->sp = ctrl.vrs[SP];
	frame->lr = ctrl.vrs[LR];
	frame->pc = ctrl.vrs[PC];

	return 0;
}

static void
unwind_backtrace(struct quadd_callchain *cc,
		 struct ex_region_info *ri,
		 struct stackframe *frame,
		 struct vm_area_struct *vma_sp,
		 struct task_struct *task,
		 int thumbflag)
{
	struct ex_region_info ri_new, *prev_ri = NULL;

	cc->urc_ut = QUADD_URC_FAILURE;

	pr_debug("fp_arm: %#lx, fp_thumb: %#lx, sp: %#lx, lr: %#lx, pc: %#lx\n",
		 frame->fp_arm, frame->fp_thumb,
		 frame->sp, frame->lr, frame->pc);
	pr_debug("vma_sp: %#lx - %#lx, length: %#lx\n",
		 vma_sp->vm_start, vma_sp->vm_end,
		 vma_sp->vm_end - vma_sp->vm_start);

	while (1) {
		long err;
		int nr_added;
		struct extab_info *ti;
		unsigned long addr, where = frame->pc;
		struct vm_area_struct *vma_pc;
		struct mm_struct *mm = task->mm;

		if (!mm)
			break;

		if (!validate_stack_addr(frame->sp, vma_sp, sizeof(u32), 0)) {
			cc->urc_ut = QUADD_URC_SP_INCORRECT;
			break;
		}

		vma_pc = find_vma(mm, frame->pc);
		if (!vma_pc)
			break;

		ti = &ri->mmap->fi.ex_sec[QUADD_SEC_TYPE_EXIDX];
		addr = get_ex_sec_address(ri, ti, QUADD_SEC_TYPE_EXIDX);

		if (!is_vma_addr(addr, vma_pc, sizeof(u32))) {
			if (prev_ri) {
				put_extabs_ehabi(prev_ri);
				prev_ri = NULL;
			}

			err = get_extabs_ehabi(task_tgid_nr(task),
					       vma_pc->vm_start, &ri_new);
			if (err) {
				cc->urc_ut = QUADD_URC_TBL_NOT_EXIST;
				break;
			}

			prev_ri = ri = &ri_new;
		}

		err = unwind_frame(cc->um, ri, frame, vma_sp, thumbflag, task);
		if (err < 0) {
			pr_debug("end unwind, urc: %ld\n", err);
			cc->urc_ut = -err;
			break;
		}

		/* determine whether outer frame is ARM or Thumb */
		thumbflag = (frame->lr & 0x1);

		pr_debug("function at [<%08lx>] from [<%08lx>]\n",
			 where, frame->pc);

		cc->curr_sp = frame->sp;
		cc->curr_fp = frame->fp_arm;
		cc->curr_fp_thumb = frame->fp_thumb;
		cc->curr_pc = frame->pc;
		cc->curr_lr = frame->lr;

		nr_added = quadd_callchain_store(cc, frame->pc,
						 QUADD_UNW_TYPE_UT);
		if (nr_added == 0)
			break;
	}

	if (prev_ri)
		put_extabs_ehabi(prev_ri);
}

unsigned int
quadd_get_user_cc_arm32_ehabi(struct quadd_event_context *event_ctx,
			      struct quadd_callchain *cc)
{
	long err;
	int nr_prev = cc->nr, thumbflag;
	unsigned long ip, sp, lr;
	struct vm_area_struct *vma, *vma_sp;
	struct ex_region_info ri;
	struct stackframe frame;
	struct pt_regs *regs = event_ctx->regs;
	struct task_struct *task = event_ctx->task;
	struct mm_struct *mm = task->mm;

	if (!regs || !mm)
		return 0;

#ifdef CONFIG_ARM64
	if (!compat_user_mode(regs))
		return 0;
#endif

	if (cc->urc_ut == QUADD_URC_LEVEL_TOO_DEEP)
		return nr_prev;

	cc->urc_ut = QUADD_URC_FAILURE;

	if (cc->curr_sp) {
		ip = cc->curr_pc;
		sp = cc->curr_sp;
		lr = cc->curr_lr;
		thumbflag = (lr & 1);

		frame.fp_thumb = cc->curr_fp_thumb;
		frame.fp_arm = cc->curr_fp;
	} else {
		ip = instruction_pointer(regs);
		sp = quadd_user_stack_pointer(regs);
		lr = quadd_user_link_register(regs);
		thumbflag = is_thumb_mode(regs);

#ifdef CONFIG_ARM64
		frame.fp_thumb = regs->compat_usr(7);
		frame.fp_arm = regs->compat_usr(11);
#else
		frame.fp_thumb = regs->ARM_r7;
		frame.fp_arm = regs->ARM_fp;
#endif
	}

	frame.pc = ip;
	frame.sp = sp;
	frame.lr = lr;

	pr_debug("pc: %#lx, lr: %#lx\n", ip, lr);
	pr_debug("sp: %#lx, fp_arm: %#lx, fp_thumb: %#lx\n",
		 sp, frame.fp_arm, frame.fp_thumb);

	vma = find_vma(mm, ip);
	if (!vma)
		return 0;

	vma_sp = find_vma(mm, sp);
	if (!vma_sp)
		return 0;

	err = get_extabs_ehabi(task_tgid_nr(task), vma->vm_start, &ri);
	if (err) {
		cc->urc_ut = QUADD_URC_TBL_NOT_EXIST;
		return 0;
	}

	unwind_backtrace(cc, &ri, &frame, vma_sp, task, thumbflag);

	put_extabs_ehabi(&ri);

	pr_debug("%s: exit, cc->nr: %d --> %d\n",
		 __func__, nr_prev, cc->nr);

	return cc->nr;
}

int
quadd_is_ex_entry_exist_arm32_ehabi(struct quadd_event_context *event_ctx,
				    unsigned long addr)
{
	int ret;
	long err;
	u32 value;
	const struct unwind_idx *idx;
	struct ex_region_info ri;
	struct vm_area_struct *vma;
	struct pt_regs *regs = event_ctx->regs;
	struct mm_struct *mm = event_ctx->task->mm;

	if (!regs || !mm)
		return 0;

	vma = find_vma(mm, addr);
	if (!vma)
		return 0;

	err = get_extabs_ehabi(task_tgid_nr(event_ctx->task),
			       vma->vm_start, &ri);
	if (err)
		return 0;

	idx = unwind_find_idx(&ri, addr, NULL);
	if (IS_ERR_OR_NULL(idx)) {
		ret = 0;
		goto out;
	}

	err = read_mmap_data(ri.mmap, &idx->insn, &value);
	if (err < 0) {
		ret = 0;
		goto out;
	}

	/* EXIDX_CANTUNWIND */
	if (value == 1) {
		ret = 0;
		goto out;
	}

	ret = 1;
out:
	put_extabs_ehabi(&ri);
	return ret;
}

int quadd_unwind_start(struct task_struct *task)
{
	int err;

	err = quadd_dwarf_unwind_start();
	if (err < 0)
		return err;

	ctx.ex_tables_size = 0;

	return 0;
}

void quadd_unwind_stop(void)
{
	struct quadd_mmap_area *entry;

	raw_spin_lock(&ctx.quadd_ctx->mmaps_lock);

	list_for_each_entry(entry, &ctx.quadd_ctx->mmap_areas, list)
		quadd_unwind_clean_mmap(entry);

	raw_spin_unlock(&ctx.quadd_ctx->mmaps_lock);

	quadd_dwarf_unwind_stop();
}

int quadd_unwind_init(struct quadd_ctx *quadd_ctx)
{
	int err;

	ctx.quadd_ctx = quadd_ctx;

	err = quadd_dwarf_unwind_init();
	if (err)
		return err;

	INIT_LIST_HEAD(&ctx.mm_ex_list);
	raw_spin_lock_init(&ctx.mm_ex_list_lock);

	return 0;
}

void quadd_unwind_deinit(void)
{
	quadd_unwind_stop();
	rcu_barrier();
}
