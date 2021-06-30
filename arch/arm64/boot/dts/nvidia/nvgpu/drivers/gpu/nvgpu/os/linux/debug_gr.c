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

#include "debug_gr.h"
#include "os_linux.h"

#include <linux/debugfs.h>

int gr_gk20a_debugfs_init(struct gk20a *g)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	l->debugfs_gr_default_attrib_cb_size =
		debugfs_create_u32("gr_default_attrib_cb_size",
				   S_IRUGO|S_IWUSR, l->debugfs,
				   &g->gr.attrib_cb_default_size);

	return 0;
}

