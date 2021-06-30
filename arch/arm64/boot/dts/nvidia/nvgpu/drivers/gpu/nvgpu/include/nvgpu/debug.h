/*
 * GK20A Debug functionality
 *
 * Copyright (c) 2014-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef NVGPU_DEBUG_H
#define NVGPU_DEBUG_H

#include <nvgpu/types.h>

struct gk20a;
struct gpu_ops;

struct gk20a_debug_output {
	void (*fn)(void *ctx, const char *str, size_t len);
	void *ctx;
	char buf[256];
};

#ifdef CONFIG_DEBUG_FS
extern unsigned int gk20a_debug_trace_cmdbuf;

void gk20a_debug_output(struct gk20a_debug_output *o,
					const char *fmt, ...);

void gk20a_debug_dump(struct gk20a *g);
void gk20a_debug_show_dump(struct gk20a *g, struct gk20a_debug_output *o);
int gk20a_gr_debug_dump(struct gk20a *g);
void gk20a_init_debug_ops(struct gpu_ops *gops);

void gk20a_debug_init(struct gk20a *g, const char *debugfs_symlink);
void gk20a_debug_deinit(struct gk20a *g);
#else
static inline void gk20a_debug_output(struct gk20a_debug_output *o,
					const char *fmt, ...) {}

static inline void gk20a_debug_dump(struct gk20a *g) {}
static inline void gk20a_debug_show_dump(struct gk20a *g, struct gk20a_debug_output *o) {}
static inline int gk20a_gr_debug_dump(struct gk20a *g) { return 0;}
static inline void gk20a_debug_init(struct gk20a *g, const char *debugfs_symlink) {}
static inline void gk20a_debug_deinit(struct gk20a *g) {}
#endif

#endif /* NVGPU_DEBUG_H */
