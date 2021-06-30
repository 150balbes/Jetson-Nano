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
#include <nvgpu/falcon.h>
#include <nvgpu/gk20a.h>

/* Dealy depends on memory size and pwr_clk
 * delay = MAX {IMEM_SIZE, DMEM_SIZE} * 64 + 1) / pwr_clk
 * Timeout set is 1msec & status check at interval 10usec
 */
#define MEM_SCRUBBING_TIMEOUT_MAX 1000
#define MEM_SCRUBBING_TIMEOUT_DEFAULT 10

int nvgpu_flcn_wait_idle(struct nvgpu_falcon *flcn)
{
	struct gk20a *g = flcn->g;
	struct nvgpu_falcon_ops *flcn_ops = &flcn->flcn_ops;
	struct nvgpu_timeout timeout;
	u32 idle_stat;

	if (!flcn_ops->is_falcon_idle) {
		nvgpu_warn(g, "Invalid op on falcon 0x%x ", flcn->flcn_id);
		return -EINVAL;
	}

	nvgpu_timeout_init(g, &timeout, 2000, NVGPU_TIMER_RETRY_TIMER);

	/* wait for falcon idle */
	do {
		idle_stat = flcn_ops->is_falcon_idle(flcn);

		if (idle_stat) {
			break;
		}

		if (nvgpu_timeout_expired_msg(&timeout,
			"waiting for falcon idle: 0x%08x", idle_stat)) {
			return -EBUSY;
		}

		nvgpu_usleep_range(100, 200);
	} while (1);

	return 0;
}

int nvgpu_flcn_mem_scrub_wait(struct nvgpu_falcon *flcn)
{
	struct nvgpu_timeout timeout;
	int status = 0;

	/* check IMEM/DMEM scrubbing complete status */
	nvgpu_timeout_init(flcn->g, &timeout,
		MEM_SCRUBBING_TIMEOUT_MAX /
		MEM_SCRUBBING_TIMEOUT_DEFAULT,
		NVGPU_TIMER_RETRY_TIMER);
	do {
		if (nvgpu_flcn_get_mem_scrubbing_status(flcn)) {
			goto exit;
		}
		nvgpu_udelay(MEM_SCRUBBING_TIMEOUT_DEFAULT);
	} while (!nvgpu_timeout_expired(&timeout));

	if (nvgpu_timeout_peek_expired(&timeout)) {
		status = -ETIMEDOUT;
	}

exit:
	return status;
}

int nvgpu_flcn_reset(struct nvgpu_falcon *flcn)
{
	int status = 0;

	if (flcn->flcn_ops.reset) {
		status = flcn->flcn_ops.reset(flcn);
		if (!status) {
			status = nvgpu_flcn_mem_scrub_wait(flcn);
                }
	} else {
		nvgpu_warn(flcn->g, "Invalid op on falcon 0x%x ",
			flcn->flcn_id);
		status = -EINVAL;
	}

	return status;
}

void nvgpu_flcn_set_irq(struct nvgpu_falcon *flcn, bool enable,
	u32 intr_mask, u32 intr_dest)
{
	struct nvgpu_falcon_ops *flcn_ops = &flcn->flcn_ops;

	if (flcn_ops->set_irq) {
		flcn->intr_mask = intr_mask;
		flcn->intr_dest = intr_dest;
		flcn_ops->set_irq(flcn, enable);
	} else {
		nvgpu_warn(flcn->g, "Invalid op on falcon 0x%x ",
			flcn->flcn_id);
	}
}

bool nvgpu_flcn_get_mem_scrubbing_status(struct nvgpu_falcon *flcn)
{
	struct nvgpu_falcon_ops *flcn_ops = &flcn->flcn_ops;
	bool status = false;

	if (flcn_ops->is_falcon_scrubbing_done) {
		status = flcn_ops->is_falcon_scrubbing_done(flcn);
	} else {
		nvgpu_warn(flcn->g, "Invalid op on falcon 0x%x ",
			flcn->flcn_id);
	}

	return status;
}

bool nvgpu_flcn_get_cpu_halted_status(struct nvgpu_falcon *flcn)
{
	struct nvgpu_falcon_ops *flcn_ops = &flcn->flcn_ops;
	bool status = false;

	if (flcn_ops->is_falcon_cpu_halted) {
		status = flcn_ops->is_falcon_cpu_halted(flcn);
	} else {
		nvgpu_warn(flcn->g, "Invalid op on falcon 0x%x ",
			flcn->flcn_id);
	}

	return status;
}

int nvgpu_flcn_wait_for_halt(struct nvgpu_falcon *flcn, unsigned int timeout)
{
	struct gk20a *g = flcn->g;
	struct nvgpu_timeout to;
	int status = 0;

	nvgpu_timeout_init(g, &to, timeout, NVGPU_TIMER_CPU_TIMER);
	do {
		if (nvgpu_flcn_get_cpu_halted_status(flcn)) {
			break;
		}

		nvgpu_udelay(10);
	} while (!nvgpu_timeout_expired(&to));

	if (nvgpu_timeout_peek_expired(&to)) {
		status = -EBUSY;
	}

	return status;
}

int nvgpu_flcn_clear_halt_intr_status(struct nvgpu_falcon *flcn,
	unsigned int timeout)
{
	struct gk20a *g = flcn->g;
	struct nvgpu_falcon_ops *flcn_ops = &flcn->flcn_ops;
	struct nvgpu_timeout to;
	int status = 0;

	if (!flcn_ops->clear_halt_interrupt_status) {
		nvgpu_warn(flcn->g, "Invalid op on falcon 0x%x ",
			flcn->flcn_id);
		return -EINVAL;
	}

	nvgpu_timeout_init(g, &to, timeout, NVGPU_TIMER_CPU_TIMER);
	do {
		if (flcn_ops->clear_halt_interrupt_status(flcn)) {
			break;
		}

		nvgpu_udelay(1);
	} while (!nvgpu_timeout_expired(&to));

	if (nvgpu_timeout_peek_expired(&to)) {
		status = -EBUSY;
	}

	return status;
}

bool nvgpu_flcn_get_idle_status(struct nvgpu_falcon *flcn)
{
	struct nvgpu_falcon_ops *flcn_ops = &flcn->flcn_ops;
	bool status = false;

	if (flcn_ops->is_falcon_idle) {
		status = flcn_ops->is_falcon_idle(flcn);
	} else {
		nvgpu_warn(flcn->g, "Invalid op on falcon 0x%x ",
			flcn->flcn_id);
	}

	return status;
}

int nvgpu_flcn_copy_from_emem(struct nvgpu_falcon *flcn,
	u32 src, u8 *dst, u32 size, u8 port)
{
	struct nvgpu_falcon_engine_dependency_ops *flcn_dops =
		&flcn->flcn_engine_dep_ops;
	int status = -EINVAL;

	if (flcn_dops->copy_from_emem != NULL) {
		status = flcn_dops->copy_from_emem(flcn, src, dst, size, port);
	}

	return status;
}

int nvgpu_flcn_copy_to_emem(struct nvgpu_falcon *flcn,
	u32 dst, u8 *src, u32 size, u8 port)
{
	struct nvgpu_falcon_engine_dependency_ops *flcn_dops =
		&flcn->flcn_engine_dep_ops;
	int status = -EINVAL;

	if (flcn_dops->copy_to_emem != NULL) {
		status = flcn_dops->copy_to_emem(flcn, dst, src, size, port);
	}

	return status;
}

int nvgpu_flcn_copy_from_dmem(struct nvgpu_falcon *flcn,
	u32 src, u8 *dst, u32 size, u8 port)
{
	struct nvgpu_falcon_ops *flcn_ops = &flcn->flcn_ops;

	return flcn_ops->copy_from_dmem(flcn, src, dst, size, port);
}

int nvgpu_flcn_copy_to_dmem(struct nvgpu_falcon *flcn,
	u32 dst, u8 *src, u32 size, u8 port)
{
	struct nvgpu_falcon_ops *flcn_ops = &flcn->flcn_ops;

	return flcn_ops->copy_to_dmem(flcn, dst, src, size, port);
}

int nvgpu_flcn_copy_from_imem(struct nvgpu_falcon *flcn,
	u32 src, u8 *dst, u32 size, u8 port)
{
	struct nvgpu_falcon_ops *flcn_ops = &flcn->flcn_ops;
	int status = -EINVAL;

	if (flcn_ops->copy_from_imem) {
		status = flcn_ops->copy_from_imem(flcn, src, dst, size, port);
	} else {
		nvgpu_warn(flcn->g, "Invalid op on falcon 0x%x ",
			flcn->flcn_id);
	}

	return status;
}

int nvgpu_flcn_copy_to_imem(struct nvgpu_falcon *flcn,
	u32 dst, u8 *src, u32 size, u8 port, bool sec, u32 tag)
{
	struct nvgpu_falcon_ops *flcn_ops = &flcn->flcn_ops;
	int status = -EINVAL;

	if (flcn_ops->copy_to_imem) {
		status = flcn_ops->copy_to_imem(flcn, dst, src, size, port,
					sec, tag);
	} else {
		nvgpu_warn(flcn->g, "Invalid op on falcon 0x%x ",
			flcn->flcn_id);
	}

	return status;
}

static void nvgpu_flcn_print_mem(struct nvgpu_falcon *flcn, u32 src,
	u32 size, enum flcn_mem_type mem_type)
{
	u32 buff[64] = {0};
	u32 total_block_read = 0;
	u32 byte_read_count = 0;
	u32 i = 0;
	u32 status = 0;

	nvgpu_info(flcn->g, " offset 0x%x  size %d bytes", src, size);

	total_block_read = size >> 8;
	do {
		byte_read_count = total_block_read ? sizeof(buff) : size;

		if (!byte_read_count) {
			break;
		}

		if (mem_type == MEM_DMEM) {
			status = nvgpu_flcn_copy_from_dmem(flcn, src,
				(u8 *)buff, byte_read_count, 0);
		} else {
			status = nvgpu_flcn_copy_from_imem(flcn, src,
				(u8 *)buff, byte_read_count, 0);
		}

		if (status) {
			nvgpu_err(flcn->g, "MEM print failed");
			break;
		}

		for (i = 0; i < (byte_read_count >> 2); i += 4) {
			nvgpu_info(flcn->g, "%#06x: %#010x %#010x %#010x %#010x",
				src + (i << 2), buff[i], buff[i+1],
				buff[i+2], buff[i+3]);
		}

		src += byte_read_count;
		size -= byte_read_count;
	} while (total_block_read--);
}

void nvgpu_flcn_print_dmem(struct nvgpu_falcon *flcn, u32 src, u32 size)
{
	nvgpu_info(flcn->g, " PRINT DMEM ");
	nvgpu_flcn_print_mem(flcn, src, size, MEM_DMEM);
}

void nvgpu_flcn_print_imem(struct nvgpu_falcon *flcn, u32 src, u32 size)
{
	nvgpu_info(flcn->g, " PRINT IMEM ");
	nvgpu_flcn_print_mem(flcn, src, size, MEM_IMEM);
}

int nvgpu_flcn_bootstrap(struct nvgpu_falcon *flcn, u32 boot_vector)
{
	struct nvgpu_falcon_ops *flcn_ops = &flcn->flcn_ops;
	int status = -EINVAL;

	if (flcn_ops->bootstrap) {
		status = flcn_ops->bootstrap(flcn, boot_vector);
	} else {
		nvgpu_warn(flcn->g, "Invalid op on falcon 0x%x ",
			flcn->flcn_id);
	}

	return status;
}

u32 nvgpu_flcn_mailbox_read(struct nvgpu_falcon *flcn, u32 mailbox_index)
{
	struct nvgpu_falcon_ops *flcn_ops = &flcn->flcn_ops;
	u32 data = 0;

	if (flcn_ops->mailbox_read) {
		data = flcn_ops->mailbox_read(flcn, mailbox_index);
	} else {
		nvgpu_warn(flcn->g, "Invalid op on falcon 0x%x ",
			flcn->flcn_id);
	}

	return data;
}

void nvgpu_flcn_mailbox_write(struct nvgpu_falcon *flcn, u32 mailbox_index,
		u32 data)
{
	struct nvgpu_falcon_ops *flcn_ops = &flcn->flcn_ops;

	if (flcn_ops->mailbox_write) {
		flcn_ops->mailbox_write(flcn, mailbox_index, data);
	} else {
		nvgpu_warn(flcn->g, "Invalid op on falcon 0x%x ",
			flcn->flcn_id);
	}
}

void nvgpu_flcn_dump_stats(struct nvgpu_falcon *flcn)
{
	struct nvgpu_falcon_ops *flcn_ops = &flcn->flcn_ops;

	if (flcn_ops->dump_falcon_stats) {
		flcn_ops->dump_falcon_stats(flcn);
	} else {
		nvgpu_warn(flcn->g, "Invalid op on falcon 0x%x ",
			flcn->flcn_id);
	}
}

int nvgpu_flcn_bl_bootstrap(struct nvgpu_falcon *flcn,
	struct nvgpu_falcon_bl_info *bl_info)
{
	struct nvgpu_falcon_ops *flcn_ops = &flcn->flcn_ops;
	int status = 0;

	if (flcn_ops->bl_bootstrap != NULL) {
		status = flcn_ops->bl_bootstrap(flcn, bl_info);
	}
	else {
		nvgpu_warn(flcn->g, "Invalid op on falcon 0x%x ",
			flcn->flcn_id);
		status = -EINVAL;
	}

	return status;
}

int nvgpu_flcn_sw_init(struct gk20a *g, u32 flcn_id)
{
	struct nvgpu_falcon *flcn = NULL;
	struct gpu_ops *gops = &g->ops;
	int err = 0;

	switch (flcn_id) {
	case FALCON_ID_PMU:
		flcn = &g->pmu_flcn;
		flcn->flcn_id = flcn_id;
		g->pmu.flcn = &g->pmu_flcn;
		g->pmu.g = g;
		break;
	case FALCON_ID_SEC2:
		flcn = &g->sec2_flcn;
		flcn->flcn_id = flcn_id;
		g->sec2.flcn = &g->sec2_flcn;
		g->sec2.g = g;
		break;
	case FALCON_ID_FECS:
		flcn = &g->fecs_flcn;
		flcn->flcn_id = flcn_id;
		break;
	case FALCON_ID_GPCCS:
		flcn = &g->gpccs_flcn;
		flcn->flcn_id = flcn_id;
		break;
	case FALCON_ID_NVDEC:
		flcn = &g->nvdec_flcn;
		flcn->flcn_id = flcn_id;
		break;
	case FALCON_ID_MINION:
		flcn = &g->minion_flcn;
		flcn->flcn_id = flcn_id;
		break;
	case FALCON_ID_GSPLITE:
		flcn = &g->gsp_flcn;
		flcn->flcn_id = flcn_id;
		break;
	default:
		nvgpu_err(g, "Invalid/Unsupported falcon ID %x", flcn_id);
		err = -ENODEV;
		break;
	};

	if (err != 0) {
		return err;
	}

	/* call to HAL method to assign flcn base & ops to selected falcon */
	flcn->g = g;
	return gops->falcon.falcon_hal_sw_init(flcn);
}
