/*
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
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

#ifndef LINUX_DEBUG_FECS_TRACE_H
#define LINUX_DEBUG_FECS_TRACE_H

struct gk20a;

#if defined(CONFIG_DEBUG_FS) && defined(CONFIG_GK20A_CTXSW_TRACE)
int nvgpu_fecs_trace_init_debugfs(struct gk20a *g);
#else
static int nvgpu_fecs_trace_init_debugfs(struct gk20a *g)
{
	return 0;
}
#endif
#endif
