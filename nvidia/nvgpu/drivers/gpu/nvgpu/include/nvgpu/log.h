/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef NVGPU_LOG_H
#define NVGPU_LOG_H

#include <nvgpu/types.h>
#include <nvgpu/bitops.h>

struct gk20a;

enum nvgpu_log_type {
	NVGPU_ERROR,
	NVGPU_WARNING,
	NVGPU_DEBUG,
	NVGPU_INFO,
};

/*
 * Each OS must implement these functions. They handle the OS specific nuances
 * of printing data to a UART, log, whatever.
 */
__attribute__((format (printf, 5, 6)))
void __nvgpu_log_msg(struct gk20a *g, const char *func_name, int line,
		     enum nvgpu_log_type type, const char *fmt, ...);

__attribute__((format (printf, 5, 6)))
void __nvgpu_log_dbg(struct gk20a *g, u64 log_mask,
		     const char *func_name, int line,
		     const char *fmt, ...);

/*
 * Use this define to set a default mask.
 */
#define NVGPU_DEFAULT_DBG_MASK		(0)

#define	gpu_dbg_info 		BIT(0) 	/* Lightly verbose info. */
#define	gpu_dbg_fn		BIT(1) 	/* Function name tracing. */
#define	gpu_dbg_reg		BIT(2) 	/* Register accesses; very verbose. */
#define	gpu_dbg_pte		BIT(3) 	/* GMMU PTEs. */
#define	gpu_dbg_intr		BIT(4) 	/* Interrupts. */
#define	gpu_dbg_pmu		BIT(5) 	/* gk20a pmu. */
#define	gpu_dbg_clk		BIT(6) 	/* gk20a clk. */
#define	gpu_dbg_map		BIT(7) 	/* Memory mappings. */
#define	gpu_dbg_map_v		BIT(8) 	/* Verbose mem mappings. */
#define	gpu_dbg_gpu_dbg		BIT(9) 	/* GPU debugger/profiler. */
#define	gpu_dbg_cde		BIT(10) /* cde info messages. */
#define	gpu_dbg_cde_ctx		BIT(11) /* cde context usage messages. */
#define	gpu_dbg_ctxsw		BIT(12) /* ctxsw tracing. */
#define	gpu_dbg_sched		BIT(13) /* Sched control tracing. */
#define	gpu_dbg_sema		BIT(14) /* Semaphore debugging. */
#define	gpu_dbg_sema_v		BIT(15) /* Verbose semaphore debugging. */
#define	gpu_dbg_pmu_pstate	BIT(16) /* p state controlled by pmu. */
#define	gpu_dbg_xv		BIT(17) /* XVE debugging. */
#define	gpu_dbg_shutdown	BIT(18) /* GPU shutdown tracing. */
#define	gpu_dbg_kmem		BIT(19) /* Kmem tracking debugging. */
#define	gpu_dbg_pd_cache	BIT(20) /* PD cache traces. */
#define	gpu_dbg_alloc		BIT(21) /* Allocator debugging. */
#define	gpu_dbg_dma		BIT(22) /* DMA allocation prints. */
#define	gpu_dbg_sgl		BIT(23) /* SGL related traces. */
#define	gpu_dbg_vidmem		BIT(24) /* VIDMEM tracing. */
#define	gpu_dbg_nvlink		BIT(25) /* nvlink Operation tracing. */
#define	gpu_dbg_clk_arb		BIT(26) /* Clk arbiter debugging. */
#define	gpu_dbg_mem		BIT(31) /* memory accesses; very verbose. */

/**
 * nvgpu_log_mask_enabled - Check if logging is enabled
 *
 * @g        - The GPU.
 * @log_mask - The mask the check against.
 *
 * Check if, given the passed mask, logging would actually happen. This is
 * useful for avoiding calling the logging function many times when we know that
 * said prints would not happen. For example for-loops of log statements in
 * critical paths.
 */
int nvgpu_log_mask_enabled(struct gk20a *g, u64 log_mask);

/**
 * nvgpu_log - Print a debug message
 *
 * @g        - The GPU.
 * @log_mask - A mask defining when the print should happen. See enum
 *             %nvgpu_log_categories.
 * @fmt      - A format string (printf style).
 * @arg...   - Arguments for the format string.
 *
 * Print a message if the log_mask matches the enabled debugging.
 */
#define nvgpu_log(g, log_mask, fmt, arg...)				\
	__nvgpu_log_dbg(g, (u32)log_mask, __func__, __LINE__, fmt, ##arg)

/**
 * nvgpu_err - Print an error
 *
 * @g        - The GPU.
 * @fmt      - A format string (printf style).
 * @arg...   - Arguments for the format string.
 *
 * Uncondtionally print an error message.
 */
#define nvgpu_err(g, fmt, arg...)					\
	__nvgpu_log_msg(g, __func__, __LINE__, NVGPU_ERROR, fmt, ##arg)

/**
 * nvgpu_err - Print a warning
 *
 * @g        - The GPU.
 * @fmt      - A format string (printf style).
 * @arg...   - Arguments for the format string.
 *
 * Uncondtionally print a warming message.
 */
#define nvgpu_warn(g, fmt, arg...)					\
	__nvgpu_log_msg(g, __func__, __LINE__, NVGPU_WARNING, fmt, ##arg)

/**
 * nvgpu_info - Print an info message
 *
 * @g        - The GPU.
 * @fmt      - A format string (printf style).
 * @arg...   - Arguments for the format string.
 *
 * Unconditionally print an information message.
 */
#define nvgpu_info(g, fmt, arg...)					\
	__nvgpu_log_msg(g, __func__, __LINE__, NVGPU_INFO, fmt, ##arg)

/*
 * Some convenience macros.
 */
#define nvgpu_log_fn(g, fmt, arg...)	nvgpu_log(g, gpu_dbg_fn, fmt, ##arg)
#define nvgpu_log_info(g, fmt, arg...)	nvgpu_log(g, gpu_dbg_info, fmt, ##arg)

/******************************************************************************
 * The old legacy debugging API minus some parts that are unnecessary.        *
 * Please, please, please do not use this!!! This is still around to aid      *
 * transitioning to the new API.                                              *
 *                                                                            *
 * This changes up the print formats to be closer to the new APIs formats.    *
 * Also it removes the dev_warn() and dev_err() usage. Those arguments are    *
 * ignored now.                                                               *
 ******************************************************************************/

/*
 * This exist for backwards compatibility with the old debug/logging API. If you
 * want ftrace support use the new API!
 */
extern u64 nvgpu_dbg_mask;

#define gk20a_dbg(log_mask, fmt, arg...)				\
	do {								\
		if (((log_mask) & nvgpu_dbg_mask) != 0)			\
			__nvgpu_log_msg(NULL, __func__, __LINE__,	\
					NVGPU_DEBUG, fmt "\n", ##arg);	\
	} while (0)

/*
 * Some convenience macros.
 */
#define gk20a_dbg_fn(fmt, arg...)	gk20a_dbg(gpu_dbg_fn, fmt, ##arg)
#define gk20a_dbg_info(fmt, arg...)	gk20a_dbg(gpu_dbg_info, fmt, ##arg)

#endif /* NVGPU_LOG_H */
