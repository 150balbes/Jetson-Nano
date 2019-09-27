/*
 * GK20A Cycle stats snapshots support (subsystem for gr_gk20a).
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef CSS_GR_GK20A_H
#define CSS_GR_GK20A_H

#include <nvgpu/nvgpu_mem.h>
#include <nvgpu/list.h>

/* the minimal size of HW buffer - should be enough to avoid HW overflows */
#define CSS_MIN_HW_SNAPSHOT_SIZE	(8 * 1024 * 1024)

struct gk20a;
struct gr_gk20a;
struct channel_gk20a;

/* cycle stats fifo header (must match NvSnapshotBufferFifo) */
struct gk20a_cs_snapshot_fifo {
	/* layout description of the buffer */
	u32	start;
	u32	end;

	/* snafu bits */
	u32	hw_overflow_events_occured;
	u32	sw_overflow_events_occured;

	/* the kernel copies new entries to put and
	 * increment the put++. if put == get then
	 * overflowEventsOccured++
	 */
	u32	put;
	u32	_reserved10;
	u32	_reserved11;
	u32	_reserved12;

	/* the driver/client reads from get until
	 * put==get, get++ */
	u32	get;
	u32	_reserved20;
	u32	_reserved21;
	u32	_reserved22;

	/* unused */
	u32	_reserved30;
	u32	_reserved31;
	u32	_reserved32;
	u32	_reserved33;
};

/* cycle stats fifo entry (must match NvSnapshotBufferFifoEntry) */
struct gk20a_cs_snapshot_fifo_entry {
	/* global 48 timestamp */
	u32	timestamp31_00:32;
	u32	timestamp39_32:8;

	/* id of perfmon, should correlate with CSS_MAX_PERFMON_IDS */
	u32	perfmon_id:8;

	/* typically samples_counter is wired to #pmtrigger count */
	u32	samples_counter:12;

	/* DS=Delay Sample, SZ=Size (0=32B, 1=16B) */
	u32	ds:1;
	u32	sz:1;
	u32	zero0:1;
	u32	zero1:1;

	/* counter results */
	u32	event_cnt:32;
	u32	trigger0_cnt:32;
	u32	trigger1_cnt:32;
	u32	sample_cnt:32;

	/* Local PmTrigger results for Maxwell+ or padding otherwise */
	u16	local_trigger_b_count:16;
	u16	book_mark_b:16;
	u16	local_trigger_a_count:16;
	u16	book_mark_a:16;
};

/* cycle stats snapshot client data (e.g. associated with channel) */
struct gk20a_cs_snapshot_client {
	struct nvgpu_list_node	list;
	struct gk20a_cs_snapshot_fifo	*snapshot;
	u32			snapshot_size;
	u32			perfmon_start;
	u32			perfmon_count;
};

static inline struct gk20a_cs_snapshot_client *
gk20a_cs_snapshot_client_from_list(struct nvgpu_list_node *node)
{
	return (struct gk20a_cs_snapshot_client *)
		((uintptr_t)node - offsetof(struct gk20a_cs_snapshot_client, list));
};

/* should correlate with size of gk20a_cs_snapshot_fifo_entry::perfmon_id */
#define CSS_MAX_PERFMON_IDS	256

/* local definitions to avoid hardcodes sizes and shifts */
#define PM_BITMAP_SIZE	DIV_ROUND_UP(CSS_MAX_PERFMON_IDS, BITS_PER_LONG)

/* cycle stats snapshot control structure for one HW entry and many clients */
struct gk20a_cs_snapshot {
	unsigned long perfmon_ids[PM_BITMAP_SIZE];
	struct nvgpu_list_node	clients;
	struct nvgpu_mem	hw_memdesc;
	/* pointer to allocated cpu_va memory where GPU place data */
	struct gk20a_cs_snapshot_fifo_entry	*hw_snapshot;
	struct gk20a_cs_snapshot_fifo_entry	*hw_end;
	struct gk20a_cs_snapshot_fifo_entry	*hw_get;
};

bool css_hw_get_overflow_status(struct gk20a *g);
u32 css_hw_get_pending_snapshots(struct gk20a *g);
void css_hw_set_handled_snapshots(struct gk20a *g, u32 done);
int css_hw_enable_snapshot(struct channel_gk20a *ch,
				struct gk20a_cs_snapshot_client *cs_client);
void css_hw_disable_snapshot(struct gr_gk20a *gr);
u32 css_gr_allocate_perfmon_ids(struct gk20a_cs_snapshot *data,
				       u32 count);
u32 css_gr_release_perfmon_ids(struct gk20a_cs_snapshot *data,
				      u32 start,
				      u32 count);
int css_hw_check_data_available(struct channel_gk20a *ch, u32 *pending,
					bool *hw_overflow);
struct gk20a_cs_snapshot_client*
css_gr_search_client(struct nvgpu_list_node *clients, u32 perfmon);

#endif /* CSS_GR_GK20A_H */
