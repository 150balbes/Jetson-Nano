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

#ifndef NVGPU_FECS_TRACE_H
#define NVGPU_FECS_TRACE_H

struct gk20a;

/*
 * If HW circular buffer is getting too many "buffer full" conditions,
 * increasing this constant should help (it drives Linux' internal buffer size).
 */
#define GK20A_FECS_TRACE_NUM_RECORDS		(1 << 10)
#define GK20A_FECS_TRACE_HASH_BITS		8 /* 2^8 */
#define GK20A_FECS_TRACE_FRAME_PERIOD_US	(1000000ULL/60ULL)
#define GK20A_FECS_TRACE_PTIMER_SHIFT		5

struct gk20a_fecs_trace_record {
	u32 magic_lo;
	u32 magic_hi;
	u32 context_id;
	u32 context_ptr;
	u32 new_context_id;
	u32 new_context_ptr;
	u64 ts[];
};

#ifdef CONFIG_GK20A_CTXSW_TRACE
u32 gk20a_fecs_trace_record_ts_tag_invalid_ts_v(void);
u32 gk20a_fecs_trace_record_ts_tag_v(u64 ts);
u64 gk20a_fecs_trace_record_ts_timestamp_v(u64 ts);
int gk20a_fecs_trace_num_ts(void);
struct gk20a_fecs_trace_record *gk20a_fecs_trace_get_record(struct gk20a *g,
	int idx);
bool gk20a_fecs_trace_is_valid_record(struct gk20a_fecs_trace_record *r);
int gk20a_fecs_trace_get_read_index(struct gk20a *g);
int gk20a_fecs_trace_get_write_index(struct gk20a *g);

#endif /* CONFIG_GK20A_CTXSW_TRACE */

#endif
