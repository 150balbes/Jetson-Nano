/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/log.h>
#include <nvgpu/types.h>

#include <nvgpu/gk20a.h>

/*
 * Define a length for log buffers. This is the buffer that the 'fmt, ...' part
 * of __nvgpu_do_log_print() prints into.
 */
#define LOG_BUFFER_LENGTH	160

/*
 * Keep this roughly the same as the kernel log format.
 */
#define LOG_FMT			"nvgpu: %s %33s:%-4d [%-4s]  %s\n"

u64 nvgpu_dbg_mask = NVGPU_DEFAULT_DBG_MASK;

static const char *log_types[] = {
	"ERR",
	"WRN",
	"DBG",
	"INFO",
};

static inline const char *nvgpu_log_name(struct gk20a *g)
{
	return "gpu.USS";
}

static void __nvgpu_really_print_log(const char *gpu_name,
				     const char *func_name, int line,
				     enum nvgpu_log_type type, const char *log)
{
	const char *name = gpu_name ? gpu_name : "";
	const char *log_type = log_types[type];

	printf(LOG_FMT, name, func_name, line, log_type, log);
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

	__nvgpu_really_print_log(nvgpu_log_name(g),
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

	__nvgpu_really_print_log(nvgpu_log_name(g),
				 func_name, line, NVGPU_DEBUG, log);
}
