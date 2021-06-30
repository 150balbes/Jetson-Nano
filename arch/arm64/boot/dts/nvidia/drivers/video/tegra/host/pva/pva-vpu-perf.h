/*
 * Copyright (c) 2018 NVIDIA Corporation.  All rights reserved.
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

#ifndef _PVA_VPU_PERF_H_
#define _PVA_VPU_PERF_H_

#define PVA_TASK_VPU_NUM_PERF_COUNTERS 8

struct pva_task_vpu_perf_counter {
	u32 count;
	u32 sum;
	u64 sum_squared;
	u32 min;
	u32 max;
};

#endif

