/*
 * drivers/misc/tegra-profiler/dwarf_unwind.h
 *
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __QUADD_DWARF_UNWIND_H
#define __QUADD_DWARF_UNWIND_H

struct quadd_callchain;
struct quadd_event_context;

int
quadd_is_ex_entry_exist_dwarf(struct quadd_event_context *event_ctx,
			      unsigned long addr);

unsigned int
quadd_get_user_cc_dwarf(struct quadd_event_context *event_ctx,
			struct quadd_callchain *cc);

int quadd_dwarf_unwind_start(void);
void quadd_dwarf_unwind_stop(void);
int quadd_dwarf_unwind_init(void);

#endif  /* __QUADD_DWARF_UNWIND_H */
