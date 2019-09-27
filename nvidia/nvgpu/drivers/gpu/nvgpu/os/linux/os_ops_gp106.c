/*
 * Copyright (c) 2018, NVIDIA Corporation.  All rights reserved.
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

#include "os_linux.h"

#include "debug_clk_gp106.h"
#include "debug_therm_gp106.h"
#include "debug_fecs_trace.h"

static struct nvgpu_os_linux_ops gp106_os_linux_ops = {
	.clk = {
		.init_debugfs = gp106_clk_init_debugfs,
	},
	.therm = {
		.init_debugfs = gp106_therm_init_debugfs,
	},
	.fecs_trace = {
		.init_debugfs = nvgpu_fecs_trace_init_debugfs,
	},
};

void nvgpu_gp106_init_os_ops(struct nvgpu_os_linux *l)
{
	l->ops.clk = gp106_os_linux_ops.clk;
	l->ops.therm = gp106_os_linux_ops.therm;
	l->ops.fecs_trace = gp106_os_linux_ops.fecs_trace;
}
