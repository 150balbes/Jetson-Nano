/*
 * Copyright (c) 2015-2018, NVIDIA CORPORATION.  All rights reserved.
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
#include <nvgpu/kmem.h>
#include <nvgpu/nvgpu_common.h>
#include <nvgpu/timers.h>
#include <nvgpu/falcon.h>
#include <nvgpu/enabled.h>
#include <nvgpu/io.h>
#include <nvgpu/gk20a.h>

#include "gm20b/fifo_gm20b.h"
#include "bios_gp106.h"
#include "gp106/mclk_gp106.h"

#include <nvgpu/hw/gp106/hw_pwr_gp106.h>
#include <nvgpu/hw/gp106/hw_top_gp106.h>

#define PMU_BOOT_TIMEOUT_DEFAULT	100 /* usec */
#define PMU_BOOT_TIMEOUT_MAX		2000000 /* usec */
#define BIOS_OVERLAY_NAME "bios-%04x.rom"
#define BIOS_OVERLAY_NAME_FORMATTED "bios-xxxx.rom"
#define ROM_FILE_PAYLOAD_OFFSET 0xa00
#define BIOS_SIZE 0x90000

static void upload_code(struct gk20a *g, u32 dst,
			u8 *src, u32 size, u8 port, bool sec)
{
	nvgpu_flcn_copy_to_imem(g->pmu.flcn, dst, src, size, port, sec,
		dst >> 8);
}

static void upload_data(struct gk20a *g, u32 dst, u8 *src, u32 size, u8 port)
{
	u32 i, words;
	u32 *src_u32 = (u32 *)src;
	u32 blk;

	nvgpu_log_info(g, "upload %d bytes to %x", size, dst);

	words = DIV_ROUND_UP(size, 4);

	blk = dst >> 8;

	nvgpu_log_info(g, "upload %d words to %x blk %d",
			words, dst, blk);
	gk20a_writel(g, pwr_falcon_dmemc_r(port),
		pwr_falcon_dmemc_offs_f(dst >> 2) |
		pwr_falcon_dmemc_blk_f(blk) |
		pwr_falcon_dmemc_aincw_f(1));

	for (i = 0; i < words; i++) {
		gk20a_writel(g, pwr_falcon_dmemd_r(port), src_u32[i]);
	}
}

int gp106_bios_devinit(struct gk20a *g)
{
	int err = 0;
	int devinit_completed;
	struct nvgpu_timeout timeout;

	nvgpu_log_fn(g, " ");

	if (nvgpu_flcn_reset(g->pmu.flcn)) {
		err = -ETIMEDOUT;
		goto out;
	}

	upload_code(g, g->bios.devinit.bootloader_phys_base,
			g->bios.devinit.bootloader,
			g->bios.devinit.bootloader_size,
			0, 0);
	upload_code(g, g->bios.devinit.phys_base,
			g->bios.devinit.ucode,
			g->bios.devinit.size,
			0, 1);
	upload_data(g, g->bios.devinit.dmem_phys_base,
			g->bios.devinit.dmem,
			g->bios.devinit.dmem_size,
			0);
	upload_data(g, g->bios.devinit_tables_phys_base,
			g->bios.devinit_tables,
			g->bios.devinit_tables_size,
			0);
	upload_data(g, g->bios.devinit_script_phys_base,
			g->bios.bootscripts,
			g->bios.bootscripts_size,
			0);

	nvgpu_flcn_bootstrap(g->pmu.flcn, g->bios.devinit.code_entry_point);

	nvgpu_timeout_init(g, &timeout,
			   PMU_BOOT_TIMEOUT_MAX /
				PMU_BOOT_TIMEOUT_DEFAULT,
			   NVGPU_TIMER_RETRY_TIMER);
	do {
		devinit_completed = pwr_falcon_cpuctl_halt_intr_v(
				gk20a_readl(g, pwr_falcon_cpuctl_r())) &&
				    top_scratch1_devinit_completed_v(
				gk20a_readl(g, top_scratch1_r()));
		nvgpu_udelay(PMU_BOOT_TIMEOUT_DEFAULT);
	} while (!devinit_completed && !nvgpu_timeout_expired(&timeout));

	if (nvgpu_timeout_peek_expired(&timeout)) {
		err = -ETIMEDOUT;
	}

	nvgpu_flcn_clear_halt_intr_status(g->pmu.flcn,
		gk20a_get_gr_idle_timeout(g));

out:
	nvgpu_log_fn(g, "done");
	return err;
}

int gp106_bios_preos_wait_for_halt(struct gk20a *g)
{
	int err = 0;

	if (nvgpu_flcn_wait_for_halt(g->pmu.flcn, PMU_BOOT_TIMEOUT_MAX / 1000)) {
		err = -ETIMEDOUT;
	}

	return err;
}

int gp106_bios_preos(struct gk20a *g)
{
	int err = 0;

	nvgpu_log_fn(g, " ");

	if (nvgpu_flcn_reset(g->pmu.flcn)) {
		err = -ETIMEDOUT;
		goto out;
	}

	if (g->ops.bios.preos_reload_check) {
		g->ops.bios.preos_reload_check(g);
	}

	upload_code(g, g->bios.preos.bootloader_phys_base,
			g->bios.preos.bootloader,
			g->bios.preos.bootloader_size,
			0, 0);
	upload_code(g, g->bios.preos.phys_base,
			g->bios.preos.ucode,
			g->bios.preos.size,
			0, 1);
	upload_data(g, g->bios.preos.dmem_phys_base,
			g->bios.preos.dmem,
			g->bios.preos.dmem_size,
			0);

	nvgpu_flcn_bootstrap(g->pmu.flcn, g->bios.preos.code_entry_point);

	err = g->ops.bios.preos_wait_for_halt(g);

	nvgpu_flcn_clear_halt_intr_status(g->pmu.flcn,
			gk20a_get_gr_idle_timeout(g));

out:
	nvgpu_log_fn(g, "done");
	return err;
}

int gp106_bios_init(struct gk20a *g)
{
	unsigned int i;
	int err;

	nvgpu_log_fn(g, " ");

	if (g->bios_is_init) {
		return 0;
	}

	nvgpu_log_info(g, "reading bios from EEPROM");
	g->bios.size = BIOS_SIZE;
	g->bios.data = nvgpu_vmalloc(g, BIOS_SIZE);
	if (!g->bios.data) {
		return -ENOMEM;
	}

	if (g->ops.xve.disable_shadow_rom) {
		g->ops.xve.disable_shadow_rom(g);
	}
	for (i = 0; i < g->bios.size/4; i++) {
		u32 val = be32_to_cpu(gk20a_readl(g, 0x300000 + i*4));

		g->bios.data[(i*4)] = (val >> 24) & 0xff;
		g->bios.data[(i*4)+1] = (val >> 16) & 0xff;
		g->bios.data[(i*4)+2] = (val >> 8) & 0xff;
		g->bios.data[(i*4)+3] = val & 0xff;
	}
	if (g->ops.xve.enable_shadow_rom) {
		g->ops.xve.enable_shadow_rom(g);
	}

	err = nvgpu_bios_parse_rom(g);
	if (err) {
		goto free_firmware;
	}

	if (g->bios.vbios_version < g->vbios_min_version) {
		nvgpu_err(g, "unsupported VBIOS version %08x",
				g->bios.vbios_version);
		err = -EINVAL;
		goto free_firmware;
	} else {
		nvgpu_info(g, "VBIOS version %08x", g->bios.vbios_version);
	}

	nvgpu_log_fn(g, "done");

	if (g->ops.bios.devinit) {
		err = g->ops.bios.devinit(g);
		if (err) {
			nvgpu_err(g, "devinit failed");
			goto free_firmware;
		}
	}

	if (nvgpu_is_enabled(g, NVGPU_PMU_RUN_PREOS) &&
	    g->ops.bios.preos) {
		err = g->ops.bios.preos(g);
		if (err) {
			nvgpu_err(g, "pre-os failed");
			goto free_firmware;
		}
	}

	if (g->ops.bios.verify_devinit) {
		err = g->ops.bios.verify_devinit(g);
		if (err) {
			nvgpu_err(g, "devinit status verification failed");
			goto free_firmware;
		}
	}

	g->bios_is_init = true;

	return 0;
free_firmware:
	if (g->bios.data) {
		nvgpu_vfree(g, g->bios.data);
	}
	return err;
}
