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

#include <nvgpu/enabled.h>
#include <nvgpu/pmu.h>
#include <nvgpu/log.h>
#include <nvgpu/timers.h>
#include <nvgpu/bug.h>
#include <nvgpu/pmuif/nvgpu_gpmu_cmdif.h>
#include <nvgpu/falcon.h>
#include <nvgpu/gk20a.h>

void nvgpu_pmu_seq_init(struct nvgpu_pmu *pmu)
{
	u32 i;

	memset(pmu->seq, 0,
		sizeof(struct pmu_sequence) * PMU_MAX_NUM_SEQUENCES);
	memset(pmu->pmu_seq_tbl, 0,
		sizeof(pmu->pmu_seq_tbl));

	for (i = 0; i < PMU_MAX_NUM_SEQUENCES; i++) {
		pmu->seq[i].id = i;
	}
}

static int pmu_seq_acquire(struct nvgpu_pmu *pmu,
			struct pmu_sequence **pseq)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct pmu_sequence *seq;
	u32 index;

	nvgpu_mutex_acquire(&pmu->pmu_seq_lock);
	index = find_first_zero_bit(pmu->pmu_seq_tbl,
				sizeof(pmu->pmu_seq_tbl));
	if (index >= sizeof(pmu->pmu_seq_tbl)) {
		nvgpu_err(g, "no free sequence available");
		nvgpu_mutex_release(&pmu->pmu_seq_lock);
		return -EAGAIN;
	}
	set_bit(index, pmu->pmu_seq_tbl);
	nvgpu_mutex_release(&pmu->pmu_seq_lock);

	seq = &pmu->seq[index];
	seq->state = PMU_SEQ_STATE_PENDING;

	*pseq = seq;
	return 0;
}

static void pmu_seq_release(struct nvgpu_pmu *pmu,
			struct pmu_sequence *seq)
{
	struct gk20a *g = gk20a_from_pmu(pmu);

	seq->state	= PMU_SEQ_STATE_FREE;
	seq->desc	= PMU_INVALID_SEQ_DESC;
	seq->callback	= NULL;
	seq->cb_params	= NULL;
	seq->msg	= NULL;
	seq->out_payload = NULL;
	g->ops.pmu_ver.pmu_allocation_set_dmem_size(pmu,
		g->ops.pmu_ver.get_pmu_seq_in_a_ptr(seq), 0);
	g->ops.pmu_ver.pmu_allocation_set_dmem_size(pmu,
		g->ops.pmu_ver.get_pmu_seq_out_a_ptr(seq), 0);

	clear_bit(seq->id, pmu->pmu_seq_tbl);
}
/* mutex */
int nvgpu_pmu_mutex_acquire(struct nvgpu_pmu *pmu, u32 id, u32 *token)
{
	struct gk20a *g = gk20a_from_pmu(pmu);

	return g->ops.pmu.pmu_mutex_acquire(pmu, id, token);
}

int nvgpu_pmu_mutex_release(struct nvgpu_pmu *pmu, u32 id, u32 *token)
{
	struct gk20a *g = gk20a_from_pmu(pmu);

	return g->ops.pmu.pmu_mutex_release(pmu, id, token);
}

/* PMU falcon queue init */
int nvgpu_pmu_queue_init(struct nvgpu_pmu *pmu,
		u32 id, union pmu_init_msg_pmu *init)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct nvgpu_falcon_queue *queue = NULL;
	u32 oflag = 0;
	int err = 0;

	if (PMU_IS_COMMAND_QUEUE(id)) {
		/*
		 * set OFLAG_WRITE for command queue
		 * i.e, push from nvgpu &
		 * pop form falcon ucode
		 */
		oflag = OFLAG_WRITE;
	} else if (PMU_IS_MESSAGE_QUEUE(id)) {
		/*
		 * set OFLAG_READ for message queue
		 * i.e, push from falcon ucode &
		 * pop form nvgpu
		 */
		oflag = OFLAG_READ;
	} else {
		nvgpu_err(g, "invalid queue-id %d", id);
		err = -EINVAL;
		goto exit;
	}

	/* init queue parameters */
	queue = &pmu->queue[id];
	queue->id = id;
	queue->oflag = oflag;
	queue->queue_type = QUEUE_TYPE_DMEM;
	g->ops.pmu_ver.get_pmu_init_msg_pmu_queue_params(queue, id, init);

	err = nvgpu_flcn_queue_init(pmu->flcn, queue);
	if (err != 0) {
		nvgpu_err(g, "queue-%d init failed", queue->id);
	}

exit:
	return err;
}

static bool pmu_validate_cmd(struct nvgpu_pmu *pmu, struct pmu_cmd *cmd,
			struct pmu_msg *msg, struct pmu_payload *payload,
			u32 queue_id)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct nvgpu_falcon_queue *queue;
	u32 in_size, out_size;

	if (!PMU_IS_SW_COMMAND_QUEUE(queue_id)) {
		goto invalid_cmd;
	}

	queue = &pmu->queue[queue_id];
	if (cmd->hdr.size < PMU_CMD_HDR_SIZE) {
		goto invalid_cmd;
	}

	if (cmd->hdr.size > (queue->size >> 1)) {
		goto invalid_cmd;
	}

	if (msg != NULL && msg->hdr.size < PMU_MSG_HDR_SIZE) {
		goto invalid_cmd;
	}

	if (!PMU_UNIT_ID_IS_VALID(cmd->hdr.unit_id)) {
		goto invalid_cmd;
	}

	if (payload == NULL) {
		return true;
	}

	if (payload->in.buf == NULL && payload->out.buf == NULL &&
		payload->rpc.prpc == NULL) {
		goto invalid_cmd;
	}

	if ((payload->in.buf != NULL && payload->in.size == 0U) ||
	    (payload->out.buf != NULL && payload->out.size == 0U) ||
		(payload->rpc.prpc != NULL && payload->rpc.size_rpc == 0U)) {
		goto invalid_cmd;
	}

	in_size = PMU_CMD_HDR_SIZE;
	if (payload->in.buf) {
		in_size += payload->in.offset;
		in_size += g->ops.pmu_ver.get_pmu_allocation_struct_size(pmu);
	}

	out_size = PMU_CMD_HDR_SIZE;
	if (payload->out.buf) {
		out_size += payload->out.offset;
		out_size += g->ops.pmu_ver.get_pmu_allocation_struct_size(pmu);
	}

	if (in_size > cmd->hdr.size || out_size > cmd->hdr.size) {
		goto invalid_cmd;
	}


	if ((payload->in.offset != 0U && payload->in.buf == NULL) ||
	    (payload->out.offset != 0U && payload->out.buf == NULL)) {
		goto invalid_cmd;
	}

	return true;

invalid_cmd:
	nvgpu_err(g, "invalid pmu cmd :\n"
		"queue_id=%d,\n"
		"cmd_size=%d, cmd_unit_id=%d, msg=%p, msg_size=%d,\n"
		"payload in=%p, in_size=%d, in_offset=%d,\n"
		"payload out=%p, out_size=%d, out_offset=%d",
		queue_id, cmd->hdr.size, cmd->hdr.unit_id,
		msg, (msg != NULL) ? msg->hdr.unit_id : ~0,
		&payload->in, payload->in.size, payload->in.offset,
		&payload->out, payload->out.size, payload->out.offset);

	return false;
}

static int pmu_write_cmd(struct nvgpu_pmu *pmu, struct pmu_cmd *cmd,
			u32 queue_id, unsigned long timeout_ms)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct nvgpu_falcon_queue *queue;
	struct nvgpu_timeout timeout;
	int err;

	nvgpu_log_fn(g, " ");

	queue = &pmu->queue[queue_id];
	nvgpu_timeout_init(g, &timeout, timeout_ms, NVGPU_TIMER_CPU_TIMER);

	do {
		err = nvgpu_flcn_queue_push(pmu->flcn, queue, cmd, cmd->hdr.size);
		if (err == -EAGAIN && nvgpu_timeout_expired(&timeout) == 0) {
			nvgpu_usleep_range(1000, 2000);
		} else {
			break;
		}
	} while (1);

	if (err) {
		nvgpu_err(g, "fail to write cmd to queue %d", queue_id);
	} else {
		nvgpu_log_fn(g, "done");
	}

	return err;
}

static int pmu_cmd_payload_extract_rpc(struct gk20a *g, struct pmu_cmd *cmd,
	struct pmu_payload *payload, struct pmu_sequence *seq)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct pmu_v *pv = &g->ops.pmu_ver;
	u16 dmem_alloc_size = 0;
	u32 dmem_alloc_offset = 0;
	int err = 0;

	nvgpu_log_fn(g, " ");

	dmem_alloc_size = payload->rpc.size_rpc +
		payload->rpc.size_scratch;
	dmem_alloc_offset = nvgpu_alloc(&pmu->dmem, dmem_alloc_size);
	if (dmem_alloc_offset == 0U) {
		err = -ENOMEM;
		goto clean_up;
	}

	nvgpu_flcn_copy_to_dmem(pmu->flcn, dmem_alloc_offset,
		payload->rpc.prpc, payload->rpc.size_rpc, 0);

	cmd->cmd.rpc.rpc_dmem_size = payload->rpc.size_rpc;
	cmd->cmd.rpc.rpc_dmem_ptr  = dmem_alloc_offset;

	seq->out_payload = payload->rpc.prpc;
	pv->pmu_allocation_set_dmem_size(pmu,
		pv->get_pmu_seq_out_a_ptr(seq),
		payload->rpc.size_rpc);
	pv->pmu_allocation_set_dmem_offset(pmu,
		pv->get_pmu_seq_out_a_ptr(seq),
		dmem_alloc_offset);

clean_up:
	if (err) {
		nvgpu_log_fn(g, "fail");
	} else {
		nvgpu_log_fn(g, "done");
	}

	return err;
}

static int pmu_cmd_payload_extract(struct gk20a *g, struct pmu_cmd *cmd,
	struct pmu_payload *payload, struct pmu_sequence *seq)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct pmu_v *pv = &g->ops.pmu_ver;
	void *in = NULL, *out = NULL;
	int err = 0;

	nvgpu_log_fn(g, " ");

	if (payload != NULL) {
		seq->out_payload = payload->out.buf;
	}

	if (payload != NULL && payload->in.offset != 0U) {
		pv->set_pmu_allocation_ptr(pmu, &in,
		((u8 *)&cmd->cmd + payload->in.offset));

		if (payload->in.buf != payload->out.buf) {
			pv->pmu_allocation_set_dmem_size(pmu, in,
			(u16)payload->in.size);
		} else {
			pv->pmu_allocation_set_dmem_size(pmu, in,
			(u16)max(payload->in.size, payload->out.size));
		}

		*(pv->pmu_allocation_get_dmem_offset_addr(pmu, in)) =
			nvgpu_alloc(&pmu->dmem,
				pv->pmu_allocation_get_dmem_size(pmu, in));
		if (*(pv->pmu_allocation_get_dmem_offset_addr(pmu, in)) == 0U) {
			goto clean_up;
		}

		if (payload->in.fb_size != 0x0U) {
			seq->in_mem = nvgpu_kzalloc(g,
					sizeof(struct nvgpu_mem));
			if (seq->in_mem == NULL) {
				err = -ENOMEM;
				goto clean_up;
			}

			nvgpu_pmu_vidmem_surface_alloc(g, seq->in_mem,
				payload->in.fb_size);
			nvgpu_pmu_surface_describe(g, seq->in_mem,
				(struct flcn_mem_desc_v0 *)
				pv->pmu_allocation_get_fb_addr(pmu, in));

			nvgpu_mem_wr_n(g, seq->in_mem, 0,
				payload->in.buf, payload->in.fb_size);

		} else {
			nvgpu_flcn_copy_to_dmem(pmu->flcn,
				(pv->pmu_allocation_get_dmem_offset(pmu, in)),
				payload->in.buf, payload->in.size, 0);
		}
		pv->pmu_allocation_set_dmem_size(pmu,
		pv->get_pmu_seq_in_a_ptr(seq),
		pv->pmu_allocation_get_dmem_size(pmu, in));
		pv->pmu_allocation_set_dmem_offset(pmu,
		pv->get_pmu_seq_in_a_ptr(seq),
		pv->pmu_allocation_get_dmem_offset(pmu, in));
	}

	if (payload != NULL && payload->out.offset != 0U) {
		pv->set_pmu_allocation_ptr(pmu, &out,
		((u8 *)&cmd->cmd + payload->out.offset));
		pv->pmu_allocation_set_dmem_size(pmu, out,
		(u16)payload->out.size);

		if (payload->in.buf != payload->out.buf) {
			*(pv->pmu_allocation_get_dmem_offset_addr(pmu, out)) =
				nvgpu_alloc(&pmu->dmem,
					pv->pmu_allocation_get_dmem_size(pmu,
					out));
			if (*(pv->pmu_allocation_get_dmem_offset_addr(pmu,
					out)) == 0U) {
				goto clean_up;
			}

			if (payload->out.fb_size != 0x0U) {
				seq->out_mem = nvgpu_kzalloc(g,
					sizeof(struct nvgpu_mem));
				if (seq->out_mem == NULL) {
					err = -ENOMEM;
					goto clean_up;
				}
				nvgpu_pmu_vidmem_surface_alloc(g, seq->out_mem,
					payload->out.fb_size);
				nvgpu_pmu_surface_describe(g, seq->out_mem,
					(struct flcn_mem_desc_v0 *)
					pv->pmu_allocation_get_fb_addr(pmu,
					out));
			}
		} else {
			BUG_ON(in == NULL);
			seq->out_mem = seq->in_mem;
			pv->pmu_allocation_set_dmem_offset(pmu, out,
			pv->pmu_allocation_get_dmem_offset(pmu, in));
		}
		pv->pmu_allocation_set_dmem_size(pmu,
		pv->get_pmu_seq_out_a_ptr(seq),
		pv->pmu_allocation_get_dmem_size(pmu, out));
		pv->pmu_allocation_set_dmem_offset(pmu,
		pv->get_pmu_seq_out_a_ptr(seq),
		pv->pmu_allocation_get_dmem_offset(pmu, out));

	}

clean_up:
	if (err) {
		nvgpu_log_fn(g, "fail");
		if (in) {
			nvgpu_free(&pmu->dmem,
				pv->pmu_allocation_get_dmem_offset(pmu, in));
		}
		if (out) {
			nvgpu_free(&pmu->dmem,
				pv->pmu_allocation_get_dmem_offset(pmu, out));
		}
	} else {
		nvgpu_log_fn(g, "done");
	}

	return err;
}

int nvgpu_pmu_cmd_post(struct gk20a *g, struct pmu_cmd *cmd,
		struct pmu_msg *msg, struct pmu_payload *payload,
		u32 queue_id, pmu_callback callback, void *cb_param,
		u32 *seq_desc, unsigned long timeout)
{
	struct nvgpu_pmu *pmu = &g->pmu;
	struct pmu_sequence *seq;
	int err;

	nvgpu_log_fn(g, " ");

	if (cmd == NULL || seq_desc == NULL || !pmu->pmu_ready) {
		if (cmd == NULL) {
			nvgpu_warn(g, "%s(): PMU cmd buffer is NULL", __func__);
		} else if (seq_desc == NULL) {
			nvgpu_warn(g, "%s(): Seq descriptor is NULL", __func__);
		} else {
			nvgpu_warn(g, "%s(): PMU is not ready", __func__);
		}

		WARN_ON(true);
		return -EINVAL;
	}

	if (!pmu_validate_cmd(pmu, cmd, msg, payload, queue_id)) {
		return -EINVAL;
	}

	err = pmu_seq_acquire(pmu, &seq);
	if (err) {
		return err;
	}

	cmd->hdr.seq_id = seq->id;

	cmd->hdr.ctrl_flags = 0;
	cmd->hdr.ctrl_flags |= PMU_CMD_FLAGS_STATUS;
	cmd->hdr.ctrl_flags |= PMU_CMD_FLAGS_INTR;

	seq->callback = callback;
	seq->cb_params = cb_param;
	seq->msg = msg;
	seq->out_payload = NULL;
	seq->desc = pmu->next_seq_desc++;

	*seq_desc = seq->desc;

	if (cmd->cmd.rpc.cmd_type == NV_PMU_RPC_CMD_ID) {
		err = pmu_cmd_payload_extract_rpc(g, cmd, payload, seq);
	} else {
		err = pmu_cmd_payload_extract(g, cmd, payload, seq);
	}

	if (err) {
		goto clean_up;
	}

	seq->state = PMU_SEQ_STATE_USED;

	err = pmu_write_cmd(pmu, cmd, queue_id, timeout);
	if (err) {
		seq->state = PMU_SEQ_STATE_PENDING;
	}

	nvgpu_log_fn(g, "done");

	return err;

clean_up:
	nvgpu_log_fn(g, "fail");

	pmu_seq_release(pmu, seq);
	return err;
}

static int pmu_response_handle(struct nvgpu_pmu *pmu,
			struct pmu_msg *msg)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct pmu_sequence *seq;
	struct pmu_v *pv = &g->ops.pmu_ver;
	int ret = 0;

	nvgpu_log_fn(g, " ");

	seq = &pmu->seq[msg->hdr.seq_id];

	if (seq->state != PMU_SEQ_STATE_USED &&
	    seq->state != PMU_SEQ_STATE_CANCELLED) {
		nvgpu_err(g, "msg for an unknown sequence %d", seq->id);
		return -EINVAL;
	}

	if (msg->hdr.unit_id == PMU_UNIT_RC &&
	    msg->msg.rc.msg_type == PMU_RC_MSG_TYPE_UNHANDLED_CMD) {
		nvgpu_err(g, "unhandled cmd: seq %d", seq->id);
	} else if (seq->state != PMU_SEQ_STATE_CANCELLED) {
		if (seq->msg) {
			if (seq->msg->hdr.size >= msg->hdr.size) {
				memcpy(seq->msg, msg, msg->hdr.size);
			}  else {
				nvgpu_err(g, "sequence %d msg buffer too small",
					seq->id);
			}
		}
		if (pv->pmu_allocation_get_dmem_size(pmu,
		pv->get_pmu_seq_out_a_ptr(seq)) != 0U) {
			nvgpu_flcn_copy_from_dmem(pmu->flcn,
			pv->pmu_allocation_get_dmem_offset(pmu,
			pv->get_pmu_seq_out_a_ptr(seq)),
			seq->out_payload,
			pv->pmu_allocation_get_dmem_size(pmu,
			pv->get_pmu_seq_out_a_ptr(seq)), 0);
		}
	} else {
		seq->callback = NULL;
	}
	if (pv->pmu_allocation_get_dmem_size(pmu,
			pv->get_pmu_seq_in_a_ptr(seq)) != 0U) {
		nvgpu_free(&pmu->dmem,
			pv->pmu_allocation_get_dmem_offset(pmu,
			pv->get_pmu_seq_in_a_ptr(seq)));
	}
	if (pv->pmu_allocation_get_dmem_size(pmu,
			pv->get_pmu_seq_out_a_ptr(seq)) != 0U) {
		nvgpu_free(&pmu->dmem,
			pv->pmu_allocation_get_dmem_offset(pmu,
			pv->get_pmu_seq_out_a_ptr(seq)));
	}

	if (seq->out_mem != NULL) {
		memset(pv->pmu_allocation_get_fb_addr(pmu,
			pv->get_pmu_seq_out_a_ptr(seq)), 0x0,
			pv->pmu_allocation_get_fb_size(pmu,
				pv->get_pmu_seq_out_a_ptr(seq)));

		nvgpu_pmu_surface_free(g, seq->out_mem);
		if (seq->out_mem != seq->in_mem) {
			nvgpu_kfree(g, seq->out_mem);
		} else {
			seq->out_mem = NULL;
		}
	}

	if (seq->in_mem != NULL) {
		memset(pv->pmu_allocation_get_fb_addr(pmu,
			pv->get_pmu_seq_in_a_ptr(seq)), 0x0,
			pv->pmu_allocation_get_fb_size(pmu,
				pv->get_pmu_seq_in_a_ptr(seq)));

		nvgpu_pmu_surface_free(g, seq->in_mem);
		nvgpu_kfree(g, seq->in_mem);
		seq->in_mem = NULL;
	}

	if (seq->callback) {
		seq->callback(g, msg, seq->cb_params, seq->desc, ret);
	}

	pmu_seq_release(pmu, seq);

	/* TBD: notify client waiting for available dmem */

	nvgpu_log_fn(g, "done");

	return 0;
}

static int pmu_handle_event(struct nvgpu_pmu *pmu, struct pmu_msg *msg)
{
	int err = 0;
	struct gk20a *g = gk20a_from_pmu(pmu);

	nvgpu_log_fn(g, " ");
	switch (msg->hdr.unit_id) {
	case PMU_UNIT_PERFMON:
	case PMU_UNIT_PERFMON_T18X:
		err = nvgpu_pmu_handle_perfmon_event(pmu, &msg->msg.perfmon);
		break;
	case PMU_UNIT_PERF:
		if (g->ops.pmu_perf.handle_pmu_perf_event != NULL) {
			err = g->ops.pmu_perf.handle_pmu_perf_event(g,
				(void *)&msg->msg.perf);
		} else {
			WARN_ON(true);
		}
		break;
	case PMU_UNIT_THERM:
		err = nvgpu_pmu_handle_therm_event(pmu, &msg->msg.therm);
		break;
	default:
		break;
	}

	return err;
}

static bool pmu_read_message(struct nvgpu_pmu *pmu,
	struct nvgpu_falcon_queue *queue,
	struct pmu_msg *msg, int *status)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	u32 read_size, bytes_read;
	int err;

	*status = 0;

	if (nvgpu_flcn_queue_is_empty(pmu->flcn, queue)) {
		return false;
	}

	err = nvgpu_flcn_queue_pop(pmu->flcn, queue, &msg->hdr,
			PMU_MSG_HDR_SIZE, &bytes_read);
	if (err != 0 || bytes_read != PMU_MSG_HDR_SIZE) {
		nvgpu_err(g, "fail to read msg from queue %d", queue->id);
		*status = err | -EINVAL;
		goto clean_up;
	}

	if (msg->hdr.unit_id == PMU_UNIT_REWIND) {
		err = nvgpu_flcn_queue_rewind(pmu->flcn, queue);
		if (err != 0) {
			nvgpu_err(g, "fail to rewind queue %d", queue->id);
			*status = err | -EINVAL;
			goto clean_up;
		}
		/* read again after rewind */
		err = nvgpu_flcn_queue_pop(pmu->flcn, queue, &msg->hdr,
				PMU_MSG_HDR_SIZE, &bytes_read);
		if (err != 0 || bytes_read != PMU_MSG_HDR_SIZE) {
			nvgpu_err(g,
				"fail to read msg from queue %d", queue->id);
			*status = err | -EINVAL;
			goto clean_up;
		}
	}

	if (!PMU_UNIT_ID_IS_VALID(msg->hdr.unit_id)) {
		nvgpu_err(g, "read invalid unit_id %d from queue %d",
			msg->hdr.unit_id, queue->id);
			*status = -EINVAL;
			goto clean_up;
	}

	if (msg->hdr.size > PMU_MSG_HDR_SIZE) {
		read_size = msg->hdr.size - PMU_MSG_HDR_SIZE;
		err = nvgpu_flcn_queue_pop(pmu->flcn, queue, &msg->msg,
			read_size, &bytes_read);
		if (err != 0 || bytes_read != read_size) {
			nvgpu_err(g,
				"fail to read msg from queue %d", queue->id);
			*status = err;
			goto clean_up;
		}
	}

	return true;

clean_up:
	return false;
}

int nvgpu_pmu_process_message(struct nvgpu_pmu *pmu)
{
	struct pmu_msg msg;
	int status;
	struct gk20a *g = gk20a_from_pmu(pmu);

	if (unlikely(!pmu->pmu_ready)) {
		nvgpu_pmu_process_init_msg(pmu, &msg);
		if (g->ops.pmu.init_wpr_region != NULL) {
			g->ops.pmu.init_wpr_region(g);
		}

		if (nvgpu_is_enabled(g, NVGPU_PMU_PERFMON)) {
			g->ops.pmu.pmu_init_perfmon(pmu);
		}

		return 0;
	}

	while (pmu_read_message(pmu,
		&pmu->queue[PMU_MESSAGE_QUEUE], &msg, &status)) {

		nvgpu_pmu_dbg(g, "read msg hdr: ");
		nvgpu_pmu_dbg(g, "unit_id = 0x%08x, size = 0x%08x",
			msg.hdr.unit_id, msg.hdr.size);
		nvgpu_pmu_dbg(g, "ctrl_flags = 0x%08x, seq_id = 0x%08x",
			msg.hdr.ctrl_flags, msg.hdr.seq_id);

		msg.hdr.ctrl_flags &= ~PMU_CMD_FLAGS_PMU_MASK;

		if (msg.hdr.ctrl_flags == PMU_CMD_FLAGS_EVENT) {
			pmu_handle_event(pmu, &msg);
		} else {
			pmu_response_handle(pmu, &msg);
		}
	}

	return 0;
}

int pmu_wait_message_cond(struct nvgpu_pmu *pmu, u32 timeout_ms,
				 void *var, u8 val)
{
	struct gk20a *g = gk20a_from_pmu(pmu);
	struct nvgpu_timeout timeout;
	unsigned long delay = GR_IDLE_CHECK_DEFAULT;

	nvgpu_timeout_init(g, &timeout, timeout_ms, NVGPU_TIMER_CPU_TIMER);

	do {
		nvgpu_rmb();

		if (*(volatile u8 *)var == val) {
			return 0;
		}

		if (g->ops.pmu.pmu_is_interrupted(pmu)) {
			g->ops.pmu.pmu_isr(g);
		}

		nvgpu_usleep_range(delay, delay * 2U);
		delay = min_t(u32, delay << 1, GR_IDLE_CHECK_MAX);
	} while (nvgpu_timeout_expired(&timeout) == 0);

	return -ETIMEDOUT;
}

static void pmu_rpc_handler(struct gk20a *g, struct pmu_msg *msg,
		void *param, u32 handle, u32 status)
{
	struct nv_pmu_rpc_header rpc;
	struct nvgpu_pmu *pmu = &g->pmu;
	struct rpc_handler_payload *rpc_payload =
		(struct rpc_handler_payload *)param;
	struct nv_pmu_rpc_struct_perfmon_query *rpc_param;

	memset(&rpc, 0, sizeof(struct nv_pmu_rpc_header));
	memcpy(&rpc, rpc_payload->rpc_buff,	sizeof(struct nv_pmu_rpc_header));

	if (rpc.flcn_status) {
		nvgpu_err(g, " failed RPC response, status=0x%x, func=0x%x",
			rpc.flcn_status, rpc.function);
		goto exit;
	}

	switch (msg->hdr.unit_id) {
	case PMU_UNIT_ACR:
		switch (rpc.function) {
		case NV_PMU_RPC_ID_ACR_INIT_WPR_REGION:
			nvgpu_pmu_dbg(g,
				"reply NV_PMU_RPC_ID_ACR_INIT_WPR_REGION");
			g->pmu_lsf_pmu_wpr_init_done = 1;
			break;
		case NV_PMU_RPC_ID_ACR_BOOTSTRAP_GR_FALCONS:
			nvgpu_pmu_dbg(g,
				"reply NV_PMU_RPC_ID_ACR_BOOTSTRAP_GR_FALCONS");
			g->pmu_lsf_loaded_falcon_id = 1;
			break;
		}
		break;
	case PMU_UNIT_PERFMON_T18X:
	case PMU_UNIT_PERFMON:
		switch (rpc.function) {
		case NV_PMU_RPC_ID_PERFMON_T18X_INIT:
			nvgpu_pmu_dbg(g,
				"reply NV_PMU_RPC_ID_PERFMON_INIT");
			pmu->perfmon_ready = 1;
			break;
		case NV_PMU_RPC_ID_PERFMON_T18X_START:
			nvgpu_pmu_dbg(g,
				"reply NV_PMU_RPC_ID_PERFMON_START");
			break;
		case NV_PMU_RPC_ID_PERFMON_T18X_STOP:
			nvgpu_pmu_dbg(g,
				"reply NV_PMU_RPC_ID_PERFMON_STOP");
			break;
		case NV_PMU_RPC_ID_PERFMON_T18X_QUERY:
			nvgpu_pmu_dbg(g,
				"reply NV_PMU_RPC_ID_PERFMON_QUERY");
			rpc_param = (struct nv_pmu_rpc_struct_perfmon_query *)
				rpc_payload->rpc_buff;
			pmu->load = rpc_param->sample_buffer[0];
			pmu->perfmon_query = 1;
			/* set perfmon_query to 1 after load is copied */
			break;
		}
		break;
	case PMU_UNIT_VOLT:
		switch (rpc.function) {
		case NV_PMU_RPC_ID_VOLT_BOARD_OBJ_GRP_CMD:
			nvgpu_pmu_dbg(g,
				"reply NV_PMU_RPC_ID_VOLT_BOARD_OBJ_GRP_CMD");
			break;
		case NV_PMU_RPC_ID_VOLT_VOLT_SET_VOLTAGE:
			nvgpu_pmu_dbg(g,
				"reply NV_PMU_RPC_ID_VOLT_VOLT_SET_VOLTAGE");
			break;
		case NV_PMU_RPC_ID_VOLT_VOLT_RAIL_GET_VOLTAGE:
			nvgpu_pmu_dbg(g,
				"reply NV_PMU_RPC_ID_VOLT_VOLT_RAIL_GET_VOLTAGE");
			break;
		case NV_PMU_RPC_ID_VOLT_LOAD:
			nvgpu_pmu_dbg(g,
				"reply NV_PMU_RPC_ID_VOLT_LOAD");
		}
		break;
	case PMU_UNIT_CLK:
		nvgpu_pmu_dbg(g, "reply PMU_UNIT_CLK");
		break;
	case PMU_UNIT_PERF:
		nvgpu_pmu_dbg(g, "reply PMU_UNIT_PERF");
		break;
	case PMU_UNIT_THERM:
			switch (rpc.function) {
			case NV_PMU_RPC_ID_THERM_BOARD_OBJ_GRP_CMD:
				nvgpu_pmu_dbg(g,
					"reply NV_PMU_RPC_ID_THERM_BOARD_OBJ_GRP_CMD");
				break;
			default:
				nvgpu_pmu_dbg(g, "reply PMU_UNIT_THERM");
				break;
			}
			break;
		/* TBD case will be added */
	default:
		nvgpu_err(g, " Invalid RPC response, stats 0x%x",
			rpc.flcn_status);
		break;
	}

exit:
	rpc_payload->complete = true;

	/* free allocated memory */
	if (rpc_payload->is_mem_free_set) {
		nvgpu_kfree(g, rpc_payload);
	}
}

int nvgpu_pmu_rpc_execute(struct nvgpu_pmu *pmu, struct nv_pmu_rpc_header *rpc,
	u16 size_rpc, u16 size_scratch, pmu_callback caller_cb,
	void *caller_cb_param, bool is_copy_back)
{
	struct gk20a *g = pmu->g;
	struct pmu_cmd cmd;
	struct pmu_payload payload;
	struct rpc_handler_payload *rpc_payload = NULL;
	pmu_callback callback = NULL;
	void *rpc_buff = NULL;
	u32 seq = 0;
	int status = 0;

	if (!pmu->pmu_ready) {
		nvgpu_warn(g, "PMU is not ready to process RPC");
		status = EINVAL;
		goto exit;
	}

	if (caller_cb == NULL) {
		rpc_payload = nvgpu_kzalloc(g,
			sizeof(struct rpc_handler_payload) + size_rpc);
		if (rpc_payload == NULL) {
			status = ENOMEM;
			goto exit;
		}

		rpc_payload->rpc_buff = (u8 *)rpc_payload +
			sizeof(struct rpc_handler_payload);
		rpc_payload->is_mem_free_set =
			is_copy_back ? false : true;

		/* assign default RPC handler*/
		callback = pmu_rpc_handler;
	} else {
		if (caller_cb_param == NULL) {
			nvgpu_err(g, "Invalid cb param addr");
			status = EINVAL;
			goto exit;
		}
		rpc_payload = nvgpu_kzalloc(g,
			sizeof(struct rpc_handler_payload));
		if (rpc_payload == NULL) {
			status = ENOMEM;
			goto exit;
		}
		rpc_payload->rpc_buff = caller_cb_param;
		rpc_payload->is_mem_free_set = true;
		callback = caller_cb;
		WARN_ON(is_copy_back);
	}

	rpc_buff = rpc_payload->rpc_buff;
	memset(&cmd, 0, sizeof(struct pmu_cmd));
	memset(&payload, 0, sizeof(struct pmu_payload));

	cmd.hdr.unit_id = rpc->unit_id;
	cmd.hdr.size = PMU_CMD_HDR_SIZE + sizeof(struct nv_pmu_rpc_cmd);
	cmd.cmd.rpc.cmd_type = NV_PMU_RPC_CMD_ID;
	cmd.cmd.rpc.flags = rpc->flags;

	memcpy(rpc_buff, rpc, size_rpc);
	payload.rpc.prpc = rpc_buff;
	payload.rpc.size_rpc = size_rpc;
	payload.rpc.size_scratch = size_scratch;

	status = nvgpu_pmu_cmd_post(g, &cmd, NULL, &payload,
			PMU_COMMAND_QUEUE_LPQ, callback,
			rpc_payload, &seq, ~0);
	if (status) {
		nvgpu_err(g, "Failed to execute RPC status=0x%x, func=0x%x",
				status, rpc->function);
		goto exit;
	}

	/*
	 * Option act like blocking call, which waits till RPC request
	 * executes on PMU & copy back processed data to rpc_buff
	 * to read data back in nvgpu
	 */
	if (is_copy_back) {
		/* wait till RPC execute in PMU & ACK */
		pmu_wait_message_cond(pmu, gk20a_get_gr_idle_timeout(g),
			&rpc_payload->complete, true);
		/* copy back data to caller */
		memcpy(rpc, rpc_buff, size_rpc);
		/* free allocated memory */
		nvgpu_kfree(g, rpc_payload);
	}

exit:
	if (status) {
		if (rpc_payload) {
			nvgpu_kfree(g, rpc_payload);
		}
	}

	return status;
}
