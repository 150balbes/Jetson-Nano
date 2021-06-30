/*
 * Copyright (C) 2017 NVIDIA Corporation.  All rights reserved.
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

#include "debug_ce.h"
#include "os_linux.h"

#include <linux/debugfs.h>

void gk20a_ce_debugfs_init(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	debugfs_create_u32("ce_app_ctx_count", S_IWUSR | S_IRUGO,
			   l->debugfs, &g->ce_app.ctx_count);
	debugfs_create_u32("ce_app_state", S_IWUSR | S_IRUGO,
			   l->debugfs, &g->ce_app.app_state);
	debugfs_create_u32("ce_app_next_ctx_id", S_IWUSR | S_IRUGO,
			   l->debugfs, &g->ce_app.next_ctx_id);
}
