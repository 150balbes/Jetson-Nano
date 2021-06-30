/*
 * PVA trace log for T194
 *
 * Copyright (c) 2017-2018, NVIDIA Corporation.  All rights reserved.
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

#define CREATE_TRACE_POINTS
#include <trace/events/nvhost_pva.h>

#include "dev.h"
#include "pva.h"
#include "pva_trace.h"

static void read_linear(const char *name, struct pva_trace_log *trace, u32 toff)
{
	struct pva_trace_header *th = NULL;
	struct pva_trace_block_hdr *bh = NULL;
	struct pva_trace_point *tp = NULL;
	u64 dt;
	u32 i;

	th = (struct pva_trace_header *)trace->addr;
	bh = (struct pva_trace_block_hdr *)((u8 *)th + th->head_offset);
	while (th->head_offset < toff) {
		tp = (struct pva_trace_point *) ((u8 *)bh + sizeof(*bh));
		dt = bh->start_time;
		for (i = 0 ; i < bh->n_entries ; i++) {
			dt = dt + tp->delta_time;
			nvhost_dbg_info("delta_time: %llu\t %s\t major: %u\t"
				"minor: %u\t flags: %u\tsequence: %u\targ1:"
				" %u\targ2: %u\n",
				dt, name, tp->major, tp->minor, tp->flags,
				tp->sequence, tp->arg1, tp->arg2);

			trace_nvhost_pva_write(dt, name, tp->major,
				tp->minor, tp->flags, tp->sequence,
				tp->arg1, tp->arg2);
			tp = tp + 1;
		}

		th->head_offset += th->block_size;

		/* head reached end of trace log buffer, break */
		if (th->head_offset >= trace->size) {
			th->head_offset = sizeof(*th);
			break;
		}
		bh = (struct pva_trace_block_hdr *) ((u8 *)th +
			th->head_offset);
	}
}

/* Read trace points from head to tail pointer */
void pva_trace_copy_to_ftrace(struct pva *pva)
{
	struct pva_trace_log *trace;
	struct pva_trace_header *th;
	u32 toff;
	const char *dev_name = pva->pdev->name;

	trace = &pva->pva_trace;
	th = (struct pva_trace_header *)trace->addr;

	/*
	 * Read from current head to tail offset. Though tail offset might
	 * get change in background by FW. Read till current tail ONLY.
	 */
	if ((th == NULL) || !th->block_size || !th->head_offset
		|| !th->tail_offset)
		return;

	nvhost_dbg_info("th->block_size: %u\tth->head_offset: %u\tth->tail_offset: %u\n",
			th->block_size, th->head_offset, th->tail_offset);

	/*
	 * If head_offset and tail_offset are same, nothing to read.
	 */
	if (th->head_offset == th->tail_offset)
		return;

	toff = th->tail_offset;

	if (th->head_offset < toff) {
		/* No circular read */
		read_linear(dev_name, trace, toff);
	} else {
		/*
		 * Circular read
		 * Read from head to trace_log buffer size
		 */
		read_linear(dev_name, trace, trace->size);
		/* Read from head to tail  */
		read_linear(dev_name, trace, toff);
	}
}
