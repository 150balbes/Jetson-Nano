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
#ifndef NVGPU_PMUIF_GPMUIF_PERFMON_H
#define NVGPU_PMUIF_GPMUIF_PERFMON_H

/*perfmon task defines*/

#define PMU_DOMAIN_GROUP_PSTATE		0
#define PMU_DOMAIN_GROUP_GPC2CLK	1
#define PMU_DOMAIN_GROUP_NUM		2

#define PMU_PERFMON_FLAG_ENABLE_INCREASE	(0x00000001)
#define PMU_PERFMON_FLAG_ENABLE_DECREASE	(0x00000002)
#define PMU_PERFMON_FLAG_CLEAR_PREV		(0x00000004)

#define NV_PMU_PERFMON_MAX_COUNTERS     10U

enum pmu_perfmon_cmd_start_fields {
	COUNTER_ALLOC
};

enum {
	PMU_PERFMON_CMD_ID_START = 0,
	PMU_PERFMON_CMD_ID_STOP  = 1,
	PMU_PERFMON_CMD_ID_INIT  = 2
};

struct pmu_perfmon_counter_v2 {
	u8 index;
	u8 flags;
	u8 group_id;
	u8 valid;
	u16 upper_threshold; /* units of 0.01% */
	u16 lower_threshold; /* units of 0.01% */
	u32 scale;
};

struct pmu_perfmon_counter_v3 {
	u8 index;
	u8 group_id;
	u16 flags;
	u16 upper_threshold; /* units of 0.01% */
	u16 lower_threshold; /* units of 0.01% */
	u32 scale;
};

struct pmu_perfmon_cmd_start_v3 {
	u8 cmd_type;
	u8 group_id;
	u8 state_id;
	u8 flags;
	struct pmu_allocation_v3 counter_alloc;
};

struct pmu_perfmon_cmd_start_v2 {
	u8 cmd_type;
	u8 group_id;
	u8 state_id;
	u8 flags;
	struct pmu_allocation_v2 counter_alloc;
};

struct pmu_perfmon_cmd_start_v1 {
	u8 cmd_type;
	u8 group_id;
	u8 state_id;
	u8 flags;
	struct pmu_allocation_v1 counter_alloc;
};

struct pmu_perfmon_cmd_stop {
	u8 cmd_type;
};

struct pmu_perfmon_cmd_init_v3 {
	u8 cmd_type;
	u8 to_decrease_count;
	u8 base_counter_id;
	u32 sample_period_us;
	struct pmu_allocation_v3 counter_alloc;
	u8 num_counters;
	u8 samples_in_moving_avg;
	u16 sample_buffer;
};

struct pmu_perfmon_cmd_init_v2 {
	u8 cmd_type;
	u8 to_decrease_count;
	u8 base_counter_id;
	u32 sample_period_us;
	struct pmu_allocation_v2 counter_alloc;
	u8 num_counters;
	u8 samples_in_moving_avg;
	u16 sample_buffer;
};

struct pmu_perfmon_cmd_init_v1 {
	u8 cmd_type;
	u8 to_decrease_count;
	u8 base_counter_id;
	u32 sample_period_us;
	struct pmu_allocation_v1 counter_alloc;
	u8 num_counters;
	u8 samples_in_moving_avg;
	u16 sample_buffer;
};

struct pmu_perfmon_cmd {
	union {
		u8 cmd_type;
		struct pmu_perfmon_cmd_start_v1 start_v1;
		struct pmu_perfmon_cmd_start_v2 start_v2;
		struct pmu_perfmon_cmd_start_v3 start_v3;
		struct pmu_perfmon_cmd_stop stop;
		struct pmu_perfmon_cmd_init_v1 init_v1;
		struct pmu_perfmon_cmd_init_v2 init_v2;
		struct pmu_perfmon_cmd_init_v3 init_v3;
	};
};

struct pmu_zbc_cmd {
	u8 cmd_type;
	u8 pad;
	u16 entry_mask;
};

/* PERFMON MSG */
enum {
	PMU_PERFMON_MSG_ID_INCREASE_EVENT = 0,
	PMU_PERFMON_MSG_ID_DECREASE_EVENT = 1,
	PMU_PERFMON_MSG_ID_INIT_EVENT     = 2,
	PMU_PERFMON_MSG_ID_ACK            = 3
};

struct pmu_perfmon_msg_generic {
	u8 msg_type;
	u8 state_id;
	u8 group_id;
	u8 data;
};

struct pmu_perfmon_msg {
	union {
		u8 msg_type;
		struct pmu_perfmon_msg_generic gen;
	};
};

/* PFERMON RPC interface*/
/*
 * RPC calls serviced by PERFMON unit.
 */
#define NV_PMU_RPC_ID_PERFMON_T18X_INIT                 0x00
#define NV_PMU_RPC_ID_PERFMON_T18X_DEINIT               0x01
#define NV_PMU_RPC_ID_PERFMON_T18X_START                0x02
#define NV_PMU_RPC_ID_PERFMON_T18X_STOP                 0x03
#define NV_PMU_RPC_ID_PERFMON_T18X_QUERY                0x04
#define NV_PMU_RPC_ID_PERFMON_T18X__COUNT               0x05

/*
 * structure that holds data used to
 * execute Perfmon INIT RPC.
 * hdr - RPC header
 * sample_periodus - Desired period in between samples.
 * to_decrease_count - Consecutive samples before decrease event.
 * base_counter_id - Index of the base counter.
 * samples_in_moving_avg - Number of values in moving average.
 * num_counters - Num of counters PMU should use.
 * counter - Counters.
 */
struct nv_pmu_rpc_struct_perfmon_init {
	struct nv_pmu_rpc_header hdr;
	u32 sample_periodus;
	u8 to_decrease_count;
	u8 base_counter_id;
	u8 samples_in_moving_avg;
	u8 num_counters;
	struct pmu_perfmon_counter_v3 counter[NV_PMU_PERFMON_MAX_COUNTERS];
	u32 scratch[1];
};

/*
 * structure that holds data used to
 * execute Perfmon START RPC.
 * hdr - RPC header
 * group_id - NV group ID
 * state_id - NV state ID
 * flags - PMU_PERFON flags
 * counters - Counters.
 */
struct nv_pmu_rpc_struct_perfmon_start {
	struct nv_pmu_rpc_header hdr;
	u8 group_id;
	u8 state_id;
	u8 flags;
	struct pmu_perfmon_counter_v3 counter[NV_PMU_PERFMON_MAX_COUNTERS];
	u32 scratch[1];
};

/*
 * structure that holds data used to
 * execute Perfmon STOP RPC.
 * hdr - RPC header
 */
struct nv_pmu_rpc_struct_perfmon_stop {
	struct nv_pmu_rpc_header hdr;
	u32 scratch[1];
};

/*
 * structure that holds data used to
 * execute QUERY RPC.
 * hdr - RPC header
 * sample_buffer - Output buffer from pmu containing utilization samples.
 */
struct nv_pmu_rpc_struct_perfmon_query {
	struct nv_pmu_rpc_header hdr;
	u16 sample_buffer[NV_PMU_PERFMON_MAX_COUNTERS];
	u32 scratch[1];
};

#endif /* NVGPU_PMUIF_GPMUIF_PERFMON_H */
