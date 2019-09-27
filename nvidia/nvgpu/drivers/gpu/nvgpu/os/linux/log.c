/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kernel.h>
#include <linux/device.h>

#include <nvgpu/log.h>
#include <nvgpu/gk20a.h>

#include "platform_gk20a.h"
#include "os_linux.h"

/*
 * Define a length for log buffers. This is the buffer that the 'fmt, ...' part
 * of __nvgpu_do_log_print() prints into. This buffer lives on the stack so it
 * needs to not be overly sized since we have limited kernel stack space. But at
 * the same time we don't want it to be restrictive either.
 */
#define LOG_BUFFER_LENGTH	160

/*
 * Annoying quirk of Linux: this has to be a string literal since the printk()
 * function and friends use the preprocessor to concatenate stuff to the start
 * of this string when printing.
 */
#define LOG_FMT			"nvgpu: %s %33s:%-4d [%s]  %s\n"

static const char *log_types[] = {
	"ERR",
	"WRN",
	"DBG",
	"INFO",
};

int nvgpu_log_mask_enabled(struct gk20a *g, u64 log_mask)
{
	return !!(g->log_mask & log_mask);
}

static inline const char *nvgpu_log_name(struct gk20a *g)
{
	return dev_name(dev_from_gk20a(g));
}

#ifdef CONFIG_GK20A_TRACE_PRINTK
static void __nvgpu_trace_printk_log(u32 trace, const char *gpu_name,
				     const char *func_name, int line,
				     const char *log_type, const char *log)
{
	trace_printk(LOG_FMT, gpu_name, func_name, line, log_type, log);
}
#endif

static void __nvgpu_really_print_log(u32 trace, const char *gpu_name,
				     const char *func_name, int line,
				     enum nvgpu_log_type type, const char *log)
{
	const char *name = gpu_name ? gpu_name : "";
	const char *log_type = log_types[type];

#ifdef CONFIG_GK20A_TRACE_PRINTK
	if (trace)
		return __nvgpu_trace_printk_log(trace, name, func_name,
						line, log_type, log);
#endif
	switch (type) {
	case NVGPU_DEBUG:
		/*
		 * We could use pr_debug() here but we control debug enablement
		 * separately from the Linux kernel. Perhaps this is a bug in
		 * nvgpu.
		 */
		pr_info(LOG_FMT, name, func_name, line, log_type, log);
		break;
	case NVGPU_INFO:
		pr_info(LOG_FMT, name, func_name, line, log_type, log);
		break;
	case NVGPU_WARNING:
		pr_warn(LOG_FMT, name, func_name, line, log_type, log);
		break;
	case NVGPU_ERROR:
		pr_err(LOG_FMT, name, func_name, line, log_type, log);
		break;
	}
}

__attribute__((format (printf, 5, 6)))
void __nvgpu_log_msg(struct gk20a *g, const char *func_name, int line,
		     enum nvgpu_log_type type, const char *fmt, ...)
{
	char log[LOG_BUFFER_LENGTH];
	va_list args;

	va_start(args, fmt);
	vsnprintf(log, LOG_BUFFER_LENGTH, fmt, args);
	va_end(args);

	__nvgpu_really_print_log(0, g ? nvgpu_log_name(g) : "",
				 func_name, line, type, log);
}

__attribute__((format (printf, 5, 6)))
void __nvgpu_log_dbg(struct gk20a *g, u64 log_mask,
		     const char *func_name, int line,
		     const char *fmt, ...)
{
	char log[LOG_BUFFER_LENGTH];
	va_list args;

	if ((log_mask & g->log_mask) == 0)
		return;

	va_start(args, fmt);
	vsnprintf(log, LOG_BUFFER_LENGTH, fmt, args);
	va_end(args);

	__nvgpu_really_print_log(g->log_trace, nvgpu_log_name(g),
				 func_name, line, NVGPU_DEBUG, log);
}
