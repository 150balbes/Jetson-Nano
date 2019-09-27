/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef __NVGPU_BARRIER_LINUX_H__
#define __NVGPU_BARRIER_LINUX_H__

#include <asm/barrier.h>

#define __nvgpu_mb()	mb()
#define __nvgpu_rmb()	rmb()
#define __nvgpu_wmb()	wmb()

#define __nvgpu_smp_mb()	smp_mb()
#define __nvgpu_smp_rmb()	smp_rmb()
#define __nvgpu_smp_wmb()	smp_wmb()

#define __nvgpu_read_barrier_depends()	read_barrier_depends()
#define __nvgpu_smp_read_barrier_depends()	smp_read_barrier_depends()

#define __NV_ACCESS_ONCE(x)	ACCESS_ONCE(x)

#define __nvgpu_speculation_barrier() speculation_barrier()

#endif /* __NVGPU_BARRIER_LINUX_H__ */
