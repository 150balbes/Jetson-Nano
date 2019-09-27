/*
 * Copyright (c) 2017 NVIDIA Corporation.  All rights reserved.
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

#ifndef _PVA_TRACE_H_
#define _PVA_TRACE_H_

/*
 * Individual Trace point
 *
 * The delta time recorded in each trace point is the time from the previous
 * trace point.  The first trace point in a block of trace points will have
 * a delta time of 0 (it is referencing the absolute time of the block).
 */
struct pva_trace_point {
	u32 delta_time;
	u8 major;
	u8 minor;
	u8 flags;
	u8 sequence;
	u32 arg1;
	u32 arg2;
};

/*
 * Trace block header that is written to DRAM, the indicated number of
 * trace points immediately follows the header.
 */
struct pva_trace_block_hdr {
	u64 start_time;
	u16 n_entries;
	u16 reserved_1;
	u32 reserved_2;
	u8 align[48];
};

struct pva_trace_header {
	u32 block_size;
	u32 head_offset;
	u32 tail_offset;
	u8 align[52];

};

#endif
