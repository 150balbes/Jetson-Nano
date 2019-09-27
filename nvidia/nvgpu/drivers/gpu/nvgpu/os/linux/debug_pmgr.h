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

#ifndef __LINUX_DEBUG_PMGR_H
#define __LINUX_DEBUG_PMGR_H

#ifdef CONFIG_DEBUG_FS
int nvgpu_pmgr_init_debugfs_linux(struct nvgpu_os_linux *l);
#else
int nvgpu_pmgr_init_debugfs_linux(struct nvgpu_os_linux *l)
{
	return 0;
}
#endif
#endif
