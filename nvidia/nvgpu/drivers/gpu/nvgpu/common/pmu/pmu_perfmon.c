/*
 * Copyright (c) 2017-2019, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/enabled.h>
#include <nvgpu/pmu.h>
#include <nvgpu/log.h>
#include <nvgpu/bug.h>
#include <nvgpu/pmuif/nvgpu_gpmu_cmdif.h>
#include <nvgpu/gk20a.h>

static u8 get_perfmon_id(struct nvgpu_pmu *pmu)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	u32 ver = g->params.gpu_arch + g->params.gpu_impl;
	u8 unit_id;

	switch (ver) {
	case GK20A_GPUID_GK20A:
	case GK20A_GPUID_GM20B:
	case GK20A_GPUID_GM20B_B:
		unit_id = PMU_UNIT_PERFMON;
		break;
	case NVGPU_GPUID_GP10B:
	case NVGPU_GPUID_GP104:
	case NVGPU_GPUID_GP106:
		unit_id = PMU_UNIT_PERFMON_T18X;
		break;
	case NVGPU_GPUID_GV11B:
		unit_id = PMU_UNIT_PERFMON_T18X;
		break;
	default:
		unit_id = PMU_UNIT_INVALID;
		nvgpu_err(g, "no support for %x", ver);
		WARN_ON(true);
	}

	return unit_id;
}

int nvgpu_pmu_init_perfmon(struct nvgpu_pmu *pmu)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct pmu_v *pv = &g->ops.pmu_ver;
	struct pmu_cmd cmd;
	struct pmu_payload payload;
	u32 seq;

	if (!nvgpu_is_enabled(g, NVGPU_PMU_PERFMON)) {
		return 0;
	}

	nvgpu_log_fn(g, " ");

	pmu->perfmon_ready = 0;

	g->ops.pmu.pmu_init_perfmon_counter(g);

	if (pmu->sample_buffer == 0U) {
		pmu->sample_buffer = nvgpu_alloc(&pmu->dmem,
						  2U * sizeof(u16));
	}
	if (pmu->sample_buffer == 0U) {
		nvgpu_err(g, "failed to allocate perfmon sample buffer");
		return -ENOMEM;
	}

	/* init PERFMON */
	memset(&cmd, 0, sizeof(struct pmu_cmd));

	cmd.hdr.unit_id = get_perfmon_id(pmu);
	if (cmd.hdr.unit_id == PMU_UNIT_INVALID) {
		nvgpu_err(g, "failed to get perfmon UNIT ID, command skipped");
		return -EINVAL;
	}

	cmd.hdr.size = PMU_CMD_HDR_SIZE + pv->get_pmu_perfmon_cmd_init_size();
	cmd.cmd.perfmon.cmd_type = PMU_PERFMON_CMD_ID_INIT;
	/* buffer to save counter values for pmu perfmon */
	pv->perfmon_cmd_init_set_sample_buffer(&cmd.cmd.perfmon,
	(u16)pmu->sample_buffer);
	/* number of sample periods below lower threshold
	 * before pmu triggers perfmon decrease event
	 * TBD: = 15
	 */
	pv->perfmon_cmd_init_set_dec_cnt(&cmd.cmd.perfmon, 15);
	/* index of base counter, aka. always ticking counter */
	pv->perfmon_cmd_init_set_base_cnt_id(&cmd.cmd.perfmon, 6);
	/* microseconds interval between pmu polls perf counters */
	pv->perfmon_cmd_init_set_samp_period_us(&cmd.cmd.perfmon, 16700);
	/* number of perfmon counters
	 * counter #3 (GR and CE2) for gk20a
	 */
	pv->perfmon_cmd_init_set_num_cnt(&cmd.cmd.perfmon, 1);
	/* moving average window for sample periods
	 * TBD: = 3000000 / sample_period_us = 17
	 */
	pv->perfmon_cmd_init_set_mov_avg(&cmd.cmd.perfmon, 17);

	memset(&payload, 0, sizeof(struct pmu_payload));
	payload.in.buf = pv->get_perfmon_cntr_ptr(pmu);
	payload.in.size = pv->get_perfmon_cntr_sz(pmu);
	payload.in.offset = pv->get_perfmon_cmd_init_offsetofvar(COUNTER_ALLOC);

	nvgpu_pmu_dbg(g, "cmd post PMU_PERFMON_CMD_ID_INIT");
	nvgpu_pmu_cmd_post(g, &cmd, NULL, &payload, PMU_COMMAND_QUEUE_LPQ,
			NULL, NULL, &seq, ~0);

	return 0;
}

int nvgpu_pmu_perfmon_start_sampling(struct nvgpu_pmu *pmu)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct pmu_v *pv = &g->ops.pmu_ver;
	struct pmu_cmd cmd;
	struct pmu_payload payload;
	u32 seq;

	if (!nvgpu_is_enabled(g, NVGPU_PMU_PERFMON)) {
		return 0;
	}

	/* PERFMON Start */
	memset(&cmd, 0, sizeof(struct pmu_cmd));
	cmd.hdr.unit_id = get_perfmon_id(pmu);
	if (cmd.hdr.unit_id == PMU_UNIT_INVALID) {
		nvgpu_err(g, "failed to get perfmon UNIT ID, command skipped");
		return -EINVAL;
	}
	cmd.hdr.size = PMU_CMD_HDR_SIZE + pv->get_pmu_perfmon_cmd_start_size();
	pv->perfmon_start_set_cmd_type(&cmd.cmd.perfmon,
		PMU_PERFMON_CMD_ID_START);
	pv->perfmon_start_set_group_id(&cmd.cmd.perfmon,
		PMU_DOMAIN_GROUP_PSTATE);
	pv->perfmon_start_set_state_id(&cmd.cmd.perfmon,
		pmu->perfmon_state_id[PMU_DOMAIN_GROUP_PSTATE]);

	pv->perfmon_start_set_flags(&cmd.cmd.perfmon,
		PMU_PERFMON_FLAG_ENABLE_INCREASE |
		PMU_PERFMON_FLAG_ENABLE_DECREASE |
		PMU_PERFMON_FLAG_CLEAR_PREV);

	memset(&payload, 0, sizeof(struct pmu_payload));

	/* TBD: PMU_PERFMON_PCT_TO_INC * 100 */
	pv->set_perfmon_cntr_ut(pmu, 3000); /* 30% */
	/* TBD: PMU_PERFMON_PCT_TO_DEC * 100 */
	pv->set_perfmon_cntr_lt(pmu, 1000); /* 10% */
	pv->set_perfmon_cntr_valid(pmu, true);

	payload.in.buf = pv->get_perfmon_cntr_ptr(pmu);
	payload.in.size = pv->get_perfmon_cntr_sz(pmu);
	payload.in.offset =
		pv->get_perfmon_cmd_start_offsetofvar(COUNTER_ALLOC);

	nvgpu_pmu_dbg(g, "cmd post PMU_PERFMON_CMD_ID_START");
	nvgpu_pmu_cmd_post(g, &cmd, NULL, &payload, PMU_COMMAND_QUEUE_LPQ,
			NULL, NULL, &seq, ~0);

	return 0;
}

int nvgpu_pmu_perfmon_stop_sampling(struct nvgpu_pmu *pmu)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct pmu_cmd cmd;
	u32 seq;

	if (!nvgpu_is_enabled(g, NVGPU_PMU_PERFMON)) {
		return 0;
	}

	/* PERFMON Stop */
	memset(&cmd, 0, sizeof(struct pmu_cmd));
	cmd.hdr.unit_id = get_perfmon_id(pmu);
	if (cmd.hdr.unit_id == PMU_UNIT_INVALID) {
		nvgpu_err(g, "failed to get perfmon UNIT ID, command skipped");
		return -EINVAL;
	}
	cmd.hdr.size = PMU_CMD_HDR_SIZE + sizeof(struct pmu_perfmon_cmd_stop);
	cmd.cmd.perfmon.stop.cmd_type = PMU_PERFMON_CMD_ID_STOP;

	nvgpu_pmu_dbg(g, "cmd post PMU_PERFMON_CMD_ID_STOP");
	nvgpu_pmu_cmd_post(g, &cmd, NULL, NULL, PMU_COMMAND_QUEUE_LPQ,
			NULL, NULL, &seq, ~0);
	return 0;
}

int nvgpu_pmu_load_norm(struct gk20a *g, u32 *load)
{
	*load = g->pmu.load_shadow;
	return 0;
}

int nvgpu_pmu_load_update(struct gk20a *g)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	u32 load = 0;

	if (!pmu->perfmon_ready) {
		pmu->load_shadow = 0;
		pmu->load = 0;
		return 0;
	}

	if (g->ops.pmu.pmu_perfmon_get_samples_rpc) {
		nvgpu_pmu_perfmon_get_samples_rpc(pmu);
		load = pmu->load;
	} else {
		nvgpu_flcn_copy_from_dmem(pmu->flcn, pmu->sample_buffer,
		(u8 *)&load, 2 * 1, 0);
	}

	pmu->load_shadow = load / 10U;
	pmu->load_avg = (((9U*pmu->load_avg) + pmu->load_shadow) / 10U);

	return 0;
}

int nvgpu_pmu_busy_cycles_norm(struct gk20a *g, u32 *norm)
{
	u64 busy_cycles, total_cycles;
	u32 intr_status;

	gk20a_busy_noresume(g);
	if (!g->power_on) {
		*norm = 0;
		goto exit;
	}

	if (g->ops.pmu.pmu_read_idle_counter == NULL ||
	    g->ops.pmu.pmu_reset_idle_counter == NULL ||
	    g->ops.pmu.pmu_read_idle_intr_status == NULL ||
	    g->ops.pmu.pmu_clear_idle_intr_status == NULL) {
		*norm = PMU_BUSY_CYCLES_NORM_MAX;
		goto exit;
	}

	busy_cycles = g->ops.pmu.pmu_read_idle_counter(g, 4);
	total_cycles = g->ops.pmu.pmu_read_idle_counter(g, 0);
	intr_status = g->ops.pmu.pmu_read_idle_intr_status(g);

	g->ops.pmu.pmu_reset_idle_counter(g, 4);
	g->ops.pmu.pmu_reset_idle_counter(g, 0);

	if (intr_status != 0UL) {
		g->ops.pmu.pmu_clear_idle_intr_status(g);
	}

	if (total_cycles == 0ULL || busy_cycles > total_cycles) {
		*norm = PMU_BUSY_CYCLES_NORM_MAX;
	} else {
		*norm = (u32)(busy_cycles * PMU_BUSY_CYCLES_NORM_MAX
			      / total_cycles);
	}

exit:
	gk20a_idle_nosuspend(g);

	return 0;
}

void nvgpu_pmu_get_load_counters(struct gk20a *g, u32 *busy_cycles,
				 u32 *total_cycles)
{
	if (!g->power_on || gk20a_busy(g) != 0) {
		*busy_cycles = 0;
		*total_cycles = 0;
		return;
	}

	*busy_cycles = g->ops.pmu.pmu_read_idle_counter(g, 1);
	*total_cycles = g->ops.pmu.pmu_read_idle_counter(g, 2);

	gk20a_idle(g);
}

void nvgpu_pmu_reset_load_counters(struct gk20a *g)
{
	if (!g->power_on || gk20a_busy(g) != 0) {
		return;
	}

	g->ops.pmu.pmu_reset_idle_counter(g, 2);
	g->ops.pmu.pmu_reset_idle_counter(g, 1);

	gk20a_idle(g);
}

int nvgpu_pmu_handle_perfmon_event(struct nvgpu_pmu *pmu,
			struct pmu_perfmon_msg *msg)
{
	struct gk20a *g = gk20a_from_pmu(pmu);

	nvgpu_log_fn(g, " ");

	switch (msg->msg_type) {
	case PMU_PERFMON_MSG_ID_INCREASE_EVENT:
		nvgpu_pmu_dbg(g, "perfmon increase event: ");
		nvgpu_pmu_dbg(g, "state_id %d, ground_id %d, pct %d",
			msg->gen.state_id, msg->gen.group_id, msg->gen.data);
		(pmu->perfmon_events_cnt)++;
		break;
	case PMU_PERFMON_MSG_ID_DECREASE_EVENT:
		nvgpu_pmu_dbg(g, "perfmon decrease event: ");
		nvgpu_pmu_dbg(g, "state_id %d, ground_id %d, pct %d",
			msg->gen.state_id, msg->gen.group_id, msg->gen.data);
		(pmu->perfmon_events_cnt)++;
		break;
	case PMU_PERFMON_MSG_ID_INIT_EVENT:
		pmu->perfmon_ready = 1;
		nvgpu_pmu_dbg(g, "perfmon init event");
		break;
	default:
		break;
	}

	/* restart sampling */
	if (pmu->perfmon_sampling_enabled) {
		return g->ops.pmu.pmu_perfmon_start_sampling(&(g->pmu));
	}

	return 0;
}

/* Perfmon RPC */
int nvgpu_pmu_init_perfmon_rpc(struct nvgpu_pmu *pmu)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct nv_pmu_rpc_struct_perfmon_init rpc;
	int status = 0;

	if (!nvgpu_is_enabled(g, NVGPU_PMU_PERFMON)) {
		return 0;
	}

	nvgpu_log_fn(g, " ");

	memset(&rpc, 0, sizeof(struct nv_pmu_rpc_struct_perfmon_init));
	pmu->perfmon_ready = 0;

	g->ops.pmu.pmu_init_perfmon_counter(g);

	/* microseconds interval between pmu polls perf counters */
	rpc.sample_periodus = 16700;
	/* number of sample periods below lower threshold
	 * before pmu triggers perfmon decrease event
	 */
	rpc.to_decrease_count = 15;
	/* index of base counter, aka. always ticking counter */
	rpc.base_counter_id = 6;
	/* moving average window for sample periods */
	rpc.samples_in_moving_avg = 17;
	/* number of perfmon counters
	 * counter #3 (GR and CE2) for gk20a
	 */
	rpc.num_counters = 1;

	memset(rpc.counter, 0, sizeof(struct pmu_perfmon_counter_v3) *
		NV_PMU_PERFMON_MAX_COUNTERS);
	/* Counter used to count GR busy cycles */
	rpc.counter[0].index = 3;

	nvgpu_pmu_dbg(g, "RPC post NV_PMU_RPC_ID_PERFMON_INIT");
	PMU_RPC_EXECUTE(status, pmu, PERFMON_T18X, INIT, &rpc, 0);
	if (status) {
		nvgpu_err(g, "Failed to execute RPC, status=0x%x", status);
		goto exit;
	}

exit:
	return 0;
}

int nvgpu_pmu_perfmon_start_sampling_rpc(struct nvgpu_pmu *pmu)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct nv_pmu_rpc_struct_perfmon_start rpc;
	int status = 0;

	if (!nvgpu_is_enabled(g, NVGPU_PMU_PERFMON)) {
		return 0;
	}

	nvgpu_log_fn(g, " ");

	memset(&rpc, 0, sizeof(struct nv_pmu_rpc_struct_perfmon_start));
	rpc.group_id = PMU_DOMAIN_GROUP_PSTATE;
	rpc.state_id = pmu->perfmon_state_id[PMU_DOMAIN_GROUP_PSTATE];
	rpc.flags = PMU_PERFMON_FLAG_ENABLE_INCREASE |
				PMU_PERFMON_FLAG_ENABLE_DECREASE |
				PMU_PERFMON_FLAG_CLEAR_PREV;

	rpc.counter[0].upper_threshold = 3000;
	rpc.counter[0].lower_threshold = 1000;

	nvgpu_pmu_dbg(g, "RPC post NV_PMU_RPC_ID_PERFMON_START\n");
	PMU_RPC_EXECUTE(status, pmu, PERFMON_T18X, START, &rpc, 0);
	if (status) {
		nvgpu_err(g, "Failed to execute RPC, status=0x%x", status);
	}

	return status;
}

int nvgpu_pmu_perfmon_stop_sampling_rpc(struct nvgpu_pmu *pmu)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct nv_pmu_rpc_struct_perfmon_stop rpc;
	int status = 0;

	if (!nvgpu_is_enabled(g, NVGPU_PMU_PERFMON)) {
		return 0;
	}

	nvgpu_log_fn(g, " ");

	memset(&rpc, 0, sizeof(struct nv_pmu_rpc_struct_perfmon_stop));
	/* PERFMON Stop */
	nvgpu_pmu_dbg(g, "RPC post NV_PMU_RPC_ID_PERFMON_STOP\n");
	PMU_RPC_EXECUTE(status, pmu, PERFMON_T18X, STOP, &rpc, 0);
	if (status) {
		nvgpu_err(g, "Failed to execute RPC, status=0x%x", status);
	}

	return status;
}

int nvgpu_pmu_perfmon_get_samples_rpc(struct nvgpu_pmu *pmu)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct nv_pmu_rpc_struct_perfmon_query rpc;
	int status = 0;

	if (!nvgpu_is_enabled(g, NVGPU_PMU_PERFMON)) {
		return 0;
	}

	nvgpu_log_fn(g, " ");
	pmu->perfmon_query = 0;
	memset(&rpc, 0, sizeof(struct nv_pmu_rpc_struct_perfmon_query));
	/* PERFMON QUERY */
	nvgpu_pmu_dbg(g, "RPC post NV_PMU_RPC_ID_PERFMON_QUERY\n");
	PMU_RPC_EXECUTE(status, pmu, PERFMON_T18X, QUERY, &rpc, 0);
	if (status) {
		nvgpu_err(g, "Failed to execute RPC, status=0x%x", status);
	}

	pmu_wait_message_cond(pmu, gk20a_get_gr_idle_timeout(g),
				      &pmu->perfmon_query, 1);

	return status;
}
