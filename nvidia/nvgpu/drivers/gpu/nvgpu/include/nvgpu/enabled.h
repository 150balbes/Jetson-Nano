/*
 * Copyright (c) 2017-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef NVGPU_ENABLED_H
#define NVGPU_ENABLED_H

struct gk20a;

#include <nvgpu/types.h>

/*
 * Available flags that describe what's enabled and what's not in the GPU. Each
 * flag here is defined by it's offset in a bitmap.
 */
#define NVGPU_IS_FMODEL				1
#define NVGPU_DRIVER_IS_DYING			2
#define NVGPU_GR_USE_DMA_FOR_FW_BOOTSTRAP 3
#define NVGPU_FECS_TRACE_VA			4
#define NVGPU_CAN_RAILGATE			5
#define NVGPU_KERNEL_IS_DYING			6
#define NVGPU_FECS_TRACE_FEATURE_CONTROL	7

/*
 * ECC flags
 */
/* SM LRF ECC is enabled */
#define NVGPU_ECC_ENABLED_SM_LRF		8
/* SM SHM ECC is enabled */
#define NVGPU_ECC_ENABLED_SM_SHM		9
/* TEX ECC is enabled */
#define NVGPU_ECC_ENABLED_TEX			10
/* L2 ECC is enabled */
#define NVGPU_ECC_ENABLED_LTC			11
/* SM L1 DATA ECC is enabled */
#define NVGPU_ECC_ENABLED_SM_L1_DATA		12
/* SM L1 TAG ECC is enabled */
#define NVGPU_ECC_ENABLED_SM_L1_TAG		13
/* SM CBU ECC is enabled */
#define NVGPU_ECC_ENABLED_SM_CBU		14
/* SM ICAHE ECC is enabled */
#define NVGPU_ECC_ENABLED_SM_ICACHE		15

/*
 * MM flags.
 */
#define NVGPU_MM_UNIFY_ADDRESS_SPACES		16
/* false if vidmem aperture actually points to sysmem */
#define NVGPU_MM_HONORS_APERTURE		17
/* unified or split memory with separate vidmem? */
#define NVGPU_MM_UNIFIED_MEMORY			18
/* User-space managed address spaces support */
#define NVGPU_SUPPORT_USERSPACE_MANAGED_AS	20
/* IO coherence support is available */
#define NVGPU_SUPPORT_IO_COHERENCE		21
/* MAP_BUFFER_EX with partial mappings */
#define NVGPU_SUPPORT_PARTIAL_MAPPINGS		22
/* MAP_BUFFER_EX with sparse allocations */
#define NVGPU_SUPPORT_SPARSE_ALLOCS		23
/* Direct PTE kind control is supported (map_buffer_ex) */
#define NVGPU_SUPPORT_MAP_DIRECT_KIND_CTRL	24
/* Support batch mapping */
#define NVGPU_SUPPORT_MAP_BUFFER_BATCH		25
/* Use coherent aperture for sysmem. */
#define NVGPU_USE_COHERENT_SYSMEM		26
/* Use physical scatter tables instead of IOMMU */
#define NVGPU_MM_USE_PHYSICAL_SG		27
/* WAR for gm20b chips. */
#define NVGPU_MM_FORCE_128K_PMU_VM		28

/*
 * Host flags
 */
#define NVGPU_HAS_SYNCPOINTS			30
/* sync fence FDs are available in, e.g., submit_gpfifo */
#define NVGPU_SUPPORT_SYNC_FENCE_FDS		31
/* NVGPU_DBG_GPU_IOCTL_CYCLE_STATS is available */
#define NVGPU_SUPPORT_CYCLE_STATS		32
/* NVGPU_DBG_GPU_IOCTL_CYCLE_STATS_SNAPSHOT is available */
#define NVGPU_SUPPORT_CYCLE_STATS_SNAPSHOT	33
/* Both gpu driver and device support TSG */
#define NVGPU_SUPPORT_TSG			34
/* Fast deterministic submits with no job tracking are supported */
#define NVGPU_SUPPORT_DETERMINISTIC_SUBMIT_NO_JOBTRACKING 35
/* Deterministic submits are supported even with job tracking */
#define NVGPU_SUPPORT_DETERMINISTIC_SUBMIT_FULL	36
/* NVGPU_IOCTL_CHANNEL_RESCHEDULE_RUNLIST is available */
#define NVGPU_SUPPORT_RESCHEDULE_RUNLIST	37

/* NVGPU_GPU_IOCTL_GET_EVENT_FD is available */
#define NVGPU_SUPPORT_DEVICE_EVENTS		38
/* FECS context switch tracing is available */
#define NVGPU_SUPPORT_FECS_CTXSW_TRACE		39

/* NVGPU_GPU_IOCTL_SET_DETERMINISTIC_OPTS is available */
#define NVGPU_SUPPORT_DETERMINISTIC_OPTS	40

/*
 * Security flags
 */

#define NVGPU_SEC_SECUREGPCCS			41
#define NVGPU_SEC_PRIVSECURITY			42
/* VPR is supported */
#define NVGPU_SUPPORT_VPR			43

/*
 * Nvlink flags
 */

#define NVGPU_SUPPORT_NVLINK			45
/*
 * PMU flags.
 */
/* perfmon enabled or disabled for PMU */
#define NVGPU_PMU_PERFMON				48
#define NVGPU_PMU_PSTATE				49
#define NVGPU_PMU_ZBC_SAVE				50
#define NVGPU_PMU_FECS_BOOTSTRAP_DONE			51
#define NVGPU_GPU_CAN_BLCG                              52
#define NVGPU_GPU_CAN_SLCG                              53
#define NVGPU_GPU_CAN_ELCG                              54
/* Clock control support */
#define NVGPU_SUPPORT_CLOCK_CONTROLS		55
/* NVGPU_GPU_IOCTL_GET_VOLTAGE is available */
#define NVGPU_SUPPORT_GET_VOLTAGE		56
/* NVGPU_GPU_IOCTL_GET_CURRENT is available */
#define NVGPU_SUPPORT_GET_CURRENT		57
/* NVGPU_GPU_IOCTL_GET_POWER is available */
#define NVGPU_SUPPORT_GET_POWER			58
/* NVGPU_GPU_IOCTL_GET_TEMPERATURE is available */
#define NVGPU_SUPPORT_GET_TEMPERATURE		59
/* NVGPU_GPU_IOCTL_SET_THERM_ALERT_LIMIT is available */
#define NVGPU_SUPPORT_SET_THERM_ALERT_LIMIT	60

/* whether to run PREOS binary on dGPUs */
#define NVGPU_PMU_RUN_PREOS			61

/* set if ASPM is enabled; only makes sense for PCI */
#define NVGPU_SUPPORT_ASPM			62
/* subcontexts are available */
#define NVGPU_SUPPORT_TSG_SUBCONTEXTS		63
/* Simultaneous Compute and Graphics (SCG) is available */
#define NVGPU_SUPPORT_SCG			64

/* GPU_VA address of a syncpoint is supported */
#define NVGPU_SUPPORT_SYNCPOINT_ADDRESS		65
/* Allocating per-channel syncpoint in user space is supported */
#define NVGPU_SUPPORT_USER_SYNCPOINT		66

/* USERMODE enable bit */
#define NVGPU_SUPPORT_USERMODE_SUBMIT		67

/* Multiple WPR support */
#define NVGPU_SUPPORT_MULTIPLE_WPR	68

/* SEC2 RTOS support*/
#define NVGPU_SUPPORT_SEC2_RTOS		69

/* NVGPU_GPU_IOCTL_GET_GPU_LOAD is available */
#define NVGPU_SUPPORT_GET_GPU_LOAD	70

/* PLATFORM_ATOMIC support */
#define NVGPU_SUPPORT_PLATFORM_ATOMIC		71

/* NVGPU_GPU_IOCTL_SET_MMU_DEBUG_MODE is available */
#define NVGPU_SUPPORT_SET_CTX_MMU_DEBUG_MODE	72

/*
 * Must be greater than the largest bit offset in the above list.
 */
#define NVGPU_MAX_ENABLED_BITS			73U

/**
 * nvgpu_is_enabled - Check if the passed flag is enabled.
 *
 * @g     - The GPU.
 * @flag  - Which flag to check.
 *
 * Returns true if the passed @flag is true; false otherwise.
 */
bool nvgpu_is_enabled(struct gk20a *g, int flag);

/**
 * __nvgpu_set_enabled - Set the state of a flag.
 *
 * @g     - The GPU.
 * @flag  - Which flag to modify.
 * @state - The state to set the flag to.
 *
 * Set the state of the passed @flag to @state.
 */
void __nvgpu_set_enabled(struct gk20a *g, int flag, bool state);

int nvgpu_init_enabled_flags(struct gk20a *g);
void nvgpu_free_enabled_flags(struct gk20a *g);

#endif /* NVGPU_ENABLED_H */
