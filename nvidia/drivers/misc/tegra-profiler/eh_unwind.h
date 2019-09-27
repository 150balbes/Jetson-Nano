/*
 * drivers/misc/tegra-profiler/eh_unwind.h
 *
 * Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __QUADD_EH_UNWIND_H__
#define __QUADD_EH_UNWIND_H__

#include <linux/tegra_profiler.h>

struct quadd_callchain;
struct quadd_ctx;
struct task_struct;
struct quadd_mmap_area;
struct quadd_event_context;

struct extab_info {
	unsigned long addr;
	unsigned long length;

	unsigned long mmap_offset;

	unsigned long tf_start;
	unsigned long tf_end;
};

struct file_ex_region_info {
	struct extab_info ex_sec[QUADD_SEC_TYPE_MAX];
	u32 file_hash;

	unsigned int is_shared:1;
};

struct ex_region_info {
	unsigned long vm_start;
	unsigned long vm_end;

	struct quadd_mmap_area *mmap;

	u32 file_hash;
};

unsigned int
quadd_get_user_cc_arm32_ehabi(struct quadd_event_context *event_ctx,
			      struct quadd_callchain *cc);

int quadd_unwind_init(struct quadd_ctx *quadd_ctx);
void quadd_unwind_deinit(void);

int quadd_unwind_start(struct task_struct *task);
void quadd_unwind_stop(void);

int quadd_unwind_set_extab(struct quadd_sections *extabs,
			   struct quadd_mmap_area *mmap);
void quadd_unwind_clean_mmap(struct quadd_mmap_area *mmap);

int
quadd_is_ex_entry_exist_arm32_ehabi(struct quadd_event_context *event_ctx,
				    unsigned long addr);

void
quadd_unwind_set_tail_info(struct ex_region_info *ri,
			   int secid,
			   unsigned long tf_start,
			   unsigned long tf_end,
			   struct task_struct *task);

long
quadd_get_dw_frames(unsigned long key, struct ex_region_info *ri,
		    struct task_struct *task);
void quadd_put_dw_frames(struct ex_region_info *ri);

unsigned long
get_ex_sec_address(struct ex_region_info *ri,
		   struct extab_info *ti, int secid);

#endif	/* __QUADD_EH_UNWIND_H__ */
