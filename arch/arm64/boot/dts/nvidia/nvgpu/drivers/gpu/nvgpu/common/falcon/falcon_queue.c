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

#include <nvgpu/lock.h>
#include <nvgpu/timers.h>
#include <nvgpu/pmu.h>
#include <nvgpu/falcon.h>

/* common falcon queue ops */
static int flcn_queue_head(struct nvgpu_falcon *flcn,
	struct nvgpu_falcon_queue *queue, u32 *head, bool set)
{
	int err = -ENOSYS;

	if (flcn->flcn_engine_dep_ops.queue_head != NULL) {
		err = flcn->flcn_engine_dep_ops.queue_head(flcn->g, queue,
			head, set);
	}

	return err;
}

static int flcn_queue_tail(struct nvgpu_falcon *flcn,
	struct nvgpu_falcon_queue *queue, u32 *tail, bool set)
{
	int err = -ENOSYS;

	if (flcn->flcn_engine_dep_ops.queue_tail != NULL) {
		err = flcn->flcn_engine_dep_ops.queue_tail(flcn->g, queue,
			tail, set);
	}

	return err;
}

static bool flcn_queue_has_room(struct nvgpu_falcon *flcn,
	struct nvgpu_falcon_queue *queue, u32 size, bool *need_rewind)
{
	u32 q_head = 0;
	u32 q_tail = 0;
	u32 q_free = 0;
	bool q_rewind = false;
	int err = 0;

	size = ALIGN(size, QUEUE_ALIGNMENT);

	err = queue->head(flcn, queue, &q_head, QUEUE_GET);
	if (err != 0) {
		nvgpu_err(flcn->g, "queue head GET failed");
		goto exit;
	}

	err = queue->tail(flcn, queue, &q_tail, QUEUE_GET);
	if (err != 0) {
		nvgpu_err(flcn->g, "queue tail GET failed");
		goto exit;
	}

	if (q_head >= q_tail) {
		q_free = queue->offset + queue->size - q_head;
		q_free -= (u32)PMU_CMD_HDR_SIZE;

		if (size > q_free) {
			q_rewind = true;
			q_head = queue->offset;
		}
	}

	if (q_head < q_tail) {
		q_free = q_tail - q_head - 1U;
	}

	if (need_rewind != NULL) {
		*need_rewind = q_rewind;
	}

exit:
	return size <= q_free;
}

static int flcn_queue_rewind(struct nvgpu_falcon *flcn,
	struct nvgpu_falcon_queue *queue)
{
	struct gk20a *g = flcn->g;
	struct pmu_cmd cmd;
	int err = 0;

	if (queue->oflag == OFLAG_WRITE) {
		cmd.hdr.unit_id = PMU_UNIT_REWIND;
		cmd.hdr.size = (u8)PMU_CMD_HDR_SIZE;
		err = queue->push(flcn, queue, &cmd, cmd.hdr.size);
		if (err != 0) {
			nvgpu_err(g, "flcn-%d queue-%d, rewind request failed",
				flcn->flcn_id, queue->id);
			goto exit;
		} else {
			nvgpu_pmu_dbg(g, "flcn-%d queue-%d, rewinded",
			flcn->flcn_id, queue->id);
		}
	}

	/* update queue position */
	queue->position = queue->offset;

	if (queue->oflag == OFLAG_READ) {
		err = queue->tail(flcn, queue, &queue->position,
			QUEUE_SET);
		if (err != 0){
			nvgpu_err(flcn->g, "flcn-%d queue-%d, position SET failed",
				flcn->flcn_id, queue->id);
			goto exit;
		}
	}

exit:
	return err;
}

/* EMEM-Q specific ops */
static int flcn_queue_push_emem(struct nvgpu_falcon *flcn,
	struct nvgpu_falcon_queue *queue, void *data, u32 size)
{
	int err = 0;

	err = nvgpu_flcn_copy_to_emem(flcn, queue->position, data, size, 0);
	if (err != 0) {
		nvgpu_err(flcn->g, "flcn-%d, queue-%d", flcn->flcn_id,
			queue->id);
		nvgpu_err(flcn->g, "emem queue write failed");
		goto exit;
	}

	queue->position += ALIGN(size, QUEUE_ALIGNMENT);

exit:
	return err;
}

static int flcn_queue_pop_emem(struct nvgpu_falcon *flcn,
	struct nvgpu_falcon_queue *queue, void *data, u32 size,
	u32 *bytes_read)
{
	struct gk20a *g = flcn->g;
	u32 q_tail = queue->position;
	u32 q_head = 0;
	u32 used = 0;
	int err = 0;

	*bytes_read = 0;

	err = queue->head(flcn, queue, &q_head, QUEUE_GET);
	if (err != 0) {
		nvgpu_err(flcn->g, "flcn-%d, queue-%d, head GET failed",
			flcn->flcn_id, queue->id);
		goto exit;
	}

	if (q_head == q_tail) {
		goto exit;
	} else if (q_head > q_tail) {
		used = q_head - q_tail;
	} else {
		used = queue->offset + queue->size - q_tail;
	}

	if (size > used) {
		nvgpu_warn(g, "queue size smaller than request read");
		size = used;
	}

	err = nvgpu_flcn_copy_from_emem(flcn, q_tail, data, size, 0);
	if (err != 0) {
		nvgpu_err(g, "flcn-%d, queue-%d", flcn->flcn_id,
			queue->id);
		nvgpu_err(flcn->g, "emem queue read failed");
		goto exit;
	}

	queue->position += ALIGN(size, QUEUE_ALIGNMENT);
	*bytes_read = size;

exit:
	return err;
}

/* assign EMEM queue type specific ops */
static void flcn_queue_init_emem_queue(struct nvgpu_falcon *flcn,
		struct nvgpu_falcon_queue *queue)
{
	queue->head = flcn_queue_head;
	queue->tail = flcn_queue_tail;
	queue->has_room = flcn_queue_has_room;
	queue->rewind = flcn_queue_rewind;
	queue->push = flcn_queue_push_emem;
	queue->pop = flcn_queue_pop_emem;
}

/* DMEM-Q specific ops */
static int flcn_queue_push_dmem(struct nvgpu_falcon *flcn,
	struct nvgpu_falcon_queue *queue, void *data, u32 size)
{
	int err = 0;

	err = nvgpu_flcn_copy_to_dmem(flcn, queue->position, data, size, 0);
	if (err != 0) {
		nvgpu_err(flcn->g, "flcn-%d, queue-%d", flcn->flcn_id,
			queue->id);
		nvgpu_err(flcn->g, "dmem queue write failed");
		goto exit;
	}

	queue->position += ALIGN(size, QUEUE_ALIGNMENT);

exit:
	return err;
}

static int flcn_queue_pop_dmem(struct nvgpu_falcon *flcn,
	struct nvgpu_falcon_queue *queue, void *data, u32 size,
	u32 *bytes_read)
{
	struct gk20a *g = flcn->g;
	u32 q_tail = queue->position;
	u32 q_head = 0;
	u32 used = 0;
	int err = 0;

	*bytes_read = 0;

	err = queue->head(flcn, queue, &q_head, QUEUE_GET);
	if (err != 0) {
		nvgpu_err(flcn->g, "flcn-%d, queue-%d, head GET failed",
			flcn->flcn_id, queue->id);
		goto exit;
	}

	if (q_head == q_tail) {
		goto exit;
	} else if (q_head > q_tail) {
		used = q_head - q_tail;
	} else {
		used = queue->offset + queue->size - q_tail;
	}

	if (size > used) {
		nvgpu_warn(g, "queue size smaller than request read");
		size = used;
	}

	err = nvgpu_flcn_copy_from_dmem(flcn, q_tail, data, size, 0);
	if (err != 0) {
		nvgpu_err(g, "flcn-%d, queue-%d", flcn->flcn_id,
			queue->id);
		nvgpu_err(flcn->g, "dmem queue read failed");
		goto exit;
	}

	queue->position += ALIGN(size, QUEUE_ALIGNMENT);
	*bytes_read = size;

exit:
	return err;
}

/* assign DMEM queue type specific ops */
static void flcn_queue_init_dmem_queue(struct nvgpu_falcon *flcn,
		struct nvgpu_falcon_queue *queue)
{
	queue->head = flcn_queue_head;
	queue->tail = flcn_queue_tail;
	queue->has_room = flcn_queue_has_room;
	queue->push = flcn_queue_push_dmem;
	queue->pop = flcn_queue_pop_dmem;
	queue->rewind = flcn_queue_rewind;
}

static int flcn_queue_prepare_write(struct nvgpu_falcon *flcn,
	struct nvgpu_falcon_queue *queue, u32 size)
{
	bool q_rewind = false;
	int err = 0;

	/* make sure there's enough free space for the write */
	if (!queue->has_room(flcn, queue, size, &q_rewind)) {
		nvgpu_pmu_dbg(flcn->g, "queue full: queue-id %d: index %d",
			queue->id, queue->index);
		err = -EAGAIN;
		goto exit;
	}

	err = queue->head(flcn, queue, &queue->position, QUEUE_GET);
	if (err != 0) {
		nvgpu_err(flcn->g, "flcn-%d queue-%d, position GET failed",
			flcn->flcn_id, queue->id);
		goto exit;
	}

	if (q_rewind) {
		err = queue->rewind(flcn, queue);
	}

exit:
	return err;
}

/* queue public functions */

/* queue push operation with lock */
int nvgpu_flcn_queue_push(struct nvgpu_falcon *flcn,
	struct nvgpu_falcon_queue *queue, void *data, u32 size)
{
	int err = 0;

	if (queue->oflag != OFLAG_WRITE) {
		nvgpu_err(flcn->g, "flcn-%d, queue-%d not opened for write",
			flcn->flcn_id, queue->id);
		err = -EINVAL;
		goto exit;
	}

	/* acquire mutex */
	nvgpu_mutex_acquire(&queue->mutex);

	err = flcn_queue_prepare_write(flcn, queue, size);
	if (err != 0) {
		goto unlock_mutex;
	}

	err = queue->push(flcn, queue, data, size);
	if (err != 0) {
		nvgpu_err(flcn->g, "flcn-%d queue-%d, fail to write",
			flcn->flcn_id, queue->id);
	}

	err = queue->head(flcn, queue, &queue->position, QUEUE_SET);
	if (err != 0){
		nvgpu_err(flcn->g, "flcn-%d queue-%d, position SET failed",
			flcn->flcn_id, queue->id);
	}

unlock_mutex:
	/* release mutex */
	nvgpu_mutex_release(&queue->mutex);
exit:
	return err;
}

/* queue pop operation with lock */
int nvgpu_flcn_queue_pop(struct nvgpu_falcon *flcn,
	struct nvgpu_falcon_queue *queue, void *data, u32 size,
	u32 *bytes_read)
{
	int err = 0;

	if (queue->oflag != OFLAG_READ) {
		nvgpu_err(flcn->g, "flcn-%d, queue-%d, not opened for read",
			flcn->flcn_id, queue->id);
		err = -EINVAL;
		goto exit;
	}

	/* acquire mutex */
	nvgpu_mutex_acquire(&queue->mutex);

	err = queue->tail(flcn, queue, &queue->position, QUEUE_GET);
	if (err != 0) {
		nvgpu_err(flcn->g, "flcn-%d queue-%d, position GET failed",
			flcn->flcn_id, queue->id);
		goto unlock_mutex;
	}

	err = queue->pop(flcn, queue, data, size, bytes_read);
	if (err != 0) {
		nvgpu_err(flcn->g, "flcn-%d queue-%d, fail to read",
			flcn->flcn_id, queue->id);
	}

	err = queue->tail(flcn, queue, &queue->position, QUEUE_SET);
	if (err != 0){
		nvgpu_err(flcn->g, "flcn-%d queue-%d, position SET failed",
			flcn->flcn_id, queue->id);
	}

unlock_mutex:
	/* release mutex */
	nvgpu_mutex_release(&queue->mutex);
exit:
	return err;
}

int nvgpu_flcn_queue_rewind(struct nvgpu_falcon *flcn,
	struct nvgpu_falcon_queue *queue)
{
	int err = 0;

	/* acquire mutex */
	nvgpu_mutex_acquire(&queue->mutex);

	if (queue->rewind != NULL) {
		err = queue->rewind(flcn, queue);
	}

	/* release mutex */
	nvgpu_mutex_release(&queue->mutex);

	return err;
}

/* queue is_empty check with lock */
bool nvgpu_flcn_queue_is_empty(struct nvgpu_falcon *flcn,
	struct nvgpu_falcon_queue *queue)
{
	u32 q_head = 0;
	u32 q_tail = 0;
	int err = 0;

	/* acquire mutex */
	nvgpu_mutex_acquire(&queue->mutex);

	err = queue->head(flcn, queue, &q_head, QUEUE_GET);
	if (err != 0) {
		nvgpu_err(flcn->g, "flcn-%d queue-%d, head GET failed",
			flcn->flcn_id, queue->id);
		goto exit;
	}

	err = queue->tail(flcn, queue, &q_tail, QUEUE_GET);
	if (err != 0) {
		nvgpu_err(flcn->g, "flcn-%d queue-%d, tail GET failed",
			flcn->flcn_id, queue->id);
		goto exit;
	}

exit:
	/* release mutex */
	nvgpu_mutex_release(&queue->mutex);

	return q_head == q_tail;
}

void nvgpu_flcn_queue_free(struct nvgpu_falcon *flcn,
	struct nvgpu_falcon_queue *queue)
{
	nvgpu_log(flcn->g, gpu_dbg_pmu, "flcn id-%d q-id %d: index %d ",
		flcn->flcn_id, queue->id, queue->index);

	/* destroy mutex */
	nvgpu_mutex_destroy(&queue->mutex);

	/* clear data*/
	memset(queue, 0, sizeof(struct nvgpu_falcon_queue));
}

int nvgpu_flcn_queue_init(struct nvgpu_falcon *flcn,
	struct nvgpu_falcon_queue *queue)
{
	struct gk20a *g = flcn->g;
	int err = 0;

	nvgpu_log(g, gpu_dbg_pmu,
		"flcn id-%d q-id %d: index %d, offset 0x%08x, size 0x%08x",
		flcn->flcn_id, queue->id, queue->index,
		queue->offset, queue->size);

	switch (queue->queue_type) {
	case QUEUE_TYPE_DMEM:
		flcn_queue_init_dmem_queue(flcn, queue);
		break;
	case QUEUE_TYPE_EMEM:
		flcn_queue_init_emem_queue(flcn, queue);
		break;
	default:
		err = -EINVAL;
		goto exit;
		break;
	}

	/* init mutex */
	err = nvgpu_mutex_init(&queue->mutex);

exit:
	if (err != 0) {
		nvgpu_err(flcn->g, "flcn-%d queue-%d, init failed",
			flcn->flcn_id, queue->id);
	}

	return err;
}

