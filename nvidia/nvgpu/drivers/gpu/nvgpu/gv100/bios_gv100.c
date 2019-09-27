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

#include <nvgpu/bios.h>
#include <nvgpu/nvgpu_common.h>
#include <nvgpu/timers.h>
#include <nvgpu/io.h>
#include <nvgpu/gk20a.h>

#include "gp106/bios_gp106.h"
#include "bios_gv100.h"

#include <nvgpu/hw/gv100/hw_pwr_gv100.h>

#define PMU_BOOT_TIMEOUT_DEFAULT	100 /* usec */
#define PMU_BOOT_TIMEOUT_MAX		2000000 /* usec */

#define SCRATCH_PREOS_PROGRESS	6
#define PREOS_PROGRESS_MASK(r)		((r >> 12) & 0xf)
#define PREOS_PROGRESS_NOT_STARTED	0
#define PREOS_PROGRESS_STARTED		1
#define PREOS_PROGRESS_EXIT		2
#define PREOS_PROGRESS_EXIT_SECUREMODE	3
#define PREOS_PROGRESS_ABORTED		6

#define SCRATCH_PMU_EXIT_AND_HALT	1
#define PMU_EXIT_AND_HALT_SET(r, v)	((r & ~0x200UL) | v)
#define PMU_EXIT_AND_HALT_YES		(0x1UL << 9)

#define SCRATCH_PRE_OS_RELOAD		1
#define PRE_OS_RELOAD_SET(r, v)		((r & ~0x100UL) | v)
#define PRE_OS_RELOAD_YES		(0x1UL << 8)


void gv100_bios_preos_reload_check(struct gk20a *g)
{
	u32 progress = g->ops.bus.read_sw_scratch(g, SCRATCH_PREOS_PROGRESS);

	if (PREOS_PROGRESS_MASK(progress) != PREOS_PROGRESS_NOT_STARTED) {
		u32 reload = g->ops.bus.read_sw_scratch(g,
				SCRATCH_PRE_OS_RELOAD);

		g->ops.bus.write_sw_scratch(g, SCRATCH_PRE_OS_RELOAD,
			PRE_OS_RELOAD_SET(reload, PRE_OS_RELOAD_YES));
	}
}

int gv100_bios_preos_wait_for_halt(struct gk20a *g)
{
	int err = -EINVAL;
	u32 progress;
	u32 tmp;
	int preos_completed;
	struct nvgpu_timeout timeout;

	nvgpu_udelay(PMU_BOOT_TIMEOUT_DEFAULT);

	/* Check the progress */
	progress = g->ops.bus.read_sw_scratch(g, SCRATCH_PREOS_PROGRESS);

	if (PREOS_PROGRESS_MASK(progress) == PREOS_PROGRESS_STARTED) {
		err = 0;

		/* Complete the handshake */
		tmp = g->ops.bus.read_sw_scratch(g, SCRATCH_PMU_EXIT_AND_HALT);

		g->ops.bus.write_sw_scratch(g, SCRATCH_PMU_EXIT_AND_HALT,
			PMU_EXIT_AND_HALT_SET(tmp, PMU_EXIT_AND_HALT_YES));

		nvgpu_timeout_init(g, &timeout,
			   PMU_BOOT_TIMEOUT_MAX /
				PMU_BOOT_TIMEOUT_DEFAULT,
			   NVGPU_TIMER_RETRY_TIMER);

		do {
			progress = g->ops.bus.read_sw_scratch(g, SCRATCH_PREOS_PROGRESS);
			preos_completed = pwr_falcon_cpuctl_halt_intr_v(
				gk20a_readl(g, pwr_falcon_cpuctl_r())) &&
					(PREOS_PROGRESS_MASK(progress) ==
					PREOS_PROGRESS_EXIT);
			nvgpu_udelay(PMU_BOOT_TIMEOUT_DEFAULT);
		} while (!preos_completed && !nvgpu_timeout_expired(&timeout));
	}

	return err;
}
