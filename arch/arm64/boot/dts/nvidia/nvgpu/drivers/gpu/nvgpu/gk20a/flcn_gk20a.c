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
#include <nvgpu/falcon.h>
#include <nvgpu/pmu.h>
#include <nvgpu/io.h>

#include "gk20a/gk20a.h"
#include "gk20a/flcn_gk20a.h"

#include <nvgpu/hw/gm20b/hw_falcon_gm20b.h>

static int gk20a_flcn_reset(struct nvgpu_falcon *flcn)
{
	struct gk20a *g = flcn->g;
	u32 base_addr = flcn->flcn_base;
	u32 unit_status = 0;
	int status = 0;

	if (flcn->flcn_engine_dep_ops.reset_eng) {
		/* falcon & engine reset */
		status = flcn->flcn_engine_dep_ops.reset_eng(g);
	} else {
		/* do falcon CPU hard reset */
		unit_status = gk20a_readl(g, base_addr +
				falcon_falcon_cpuctl_r());
		gk20a_writel(g, base_addr + falcon_falcon_cpuctl_r(),
			(unit_status | falcon_falcon_cpuctl_hreset_f(1)));
	}

	return status;
}

static bool gk20a_flcn_clear_halt_interrupt_status(struct nvgpu_falcon *flcn)
{
	struct gk20a *g = flcn->g;
	u32 base_addr = flcn->flcn_base;
	u32 data = 0;
	bool status = false;

	gk20a_writel(g, base_addr + falcon_falcon_irqsclr_r(),
		gk20a_readl(g, base_addr + falcon_falcon_irqsclr_r()) |
		(0x10));
	data = gk20a_readl(g, (base_addr + falcon_falcon_irqstat_r()));

	if ((data & falcon_falcon_irqstat_halt_true_f()) !=
		falcon_falcon_irqstat_halt_true_f()) {
		/*halt irq is clear*/
		status = true;
	}

	return status;
}

static void gk20a_flcn_set_irq(struct nvgpu_falcon *flcn, bool enable)
{
	struct gk20a *g = flcn->g;
	u32 base_addr = flcn->flcn_base;

	if (!flcn->is_interrupt_enabled) {
		nvgpu_warn(g, "Interrupt not supported on flcn 0x%x ",
			flcn->flcn_id);
		/* Keep interrupt disabled */
		enable = false;
	}

	if (enable) {
		gk20a_writel(g, base_addr + falcon_falcon_irqmset_r(),
			flcn->intr_mask);
		gk20a_writel(g, base_addr + falcon_falcon_irqdest_r(),
			flcn->intr_dest);
	} else {
		gk20a_writel(g, base_addr + falcon_falcon_irqmclr_r(),
			0xffffffff);
	}
}

static bool gk20a_is_falcon_cpu_halted(struct nvgpu_falcon *flcn)
{
	struct gk20a *g = flcn->g;
	u32 base_addr = flcn->flcn_base;

	return (gk20a_readl(g, base_addr + falcon_falcon_cpuctl_r()) &
			falcon_falcon_cpuctl_halt_intr_m() ?
			true : false);
}

static bool gk20a_is_falcon_idle(struct nvgpu_falcon *flcn)
{
	struct gk20a *g = flcn->g;
	u32 base_addr = flcn->flcn_base;
	u32 unit_status = 0;
	bool status = false;

	unit_status = gk20a_readl(g,
		base_addr + falcon_falcon_idlestate_r());

	if (falcon_falcon_idlestate_falcon_busy_v(unit_status) == 0 &&
		falcon_falcon_idlestate_ext_busy_v(unit_status) == 0) {
		status = true;
	} else {
		status = false;
	}

	return status;
}

static bool gk20a_is_falcon_scrubbing_done(struct nvgpu_falcon *flcn)
{
	struct gk20a *g = flcn->g;
	u32 base_addr = flcn->flcn_base;
	u32 unit_status = 0;
	bool status = false;

	unit_status = gk20a_readl(g,
		base_addr + falcon_falcon_dmactl_r());

	if (unit_status & (falcon_falcon_dmactl_dmem_scrubbing_m() |
		 falcon_falcon_dmactl_imem_scrubbing_m())) {
		status = false;
	} else {
		status = true;
	}

	return status;
}

static u32 gk20a_falcon_get_mem_size(struct nvgpu_falcon *flcn,
		enum flcn_mem_type mem_type)
{
	struct gk20a *g = flcn->g;
	u32 mem_size = 0;
	u32 hw_cfg_reg = gk20a_readl(g,
		flcn->flcn_base + falcon_falcon_hwcfg_r());

	if (mem_type == MEM_DMEM) {
		mem_size = falcon_falcon_hwcfg_dmem_size_v(hw_cfg_reg)
			<< GK20A_PMU_DMEM_BLKSIZE2;
	} else {
		mem_size = falcon_falcon_hwcfg_imem_size_v(hw_cfg_reg)
			<< GK20A_PMU_DMEM_BLKSIZE2;
	}

	return mem_size;
}

static int flcn_mem_overflow_check(struct nvgpu_falcon *flcn,
		u32 offset, u32 size, enum flcn_mem_type mem_type)
{
	struct gk20a *g = flcn->g;
	u32 mem_size = 0;

	if (size == 0) {
		nvgpu_err(g, "size is zero");
		return -EINVAL;
	}

	if (offset & 0x3) {
		nvgpu_err(g, "offset (0x%08x) not 4-byte aligned", offset);
		return -EINVAL;
	}

	mem_size = gk20a_falcon_get_mem_size(flcn, mem_type);
	if (!(offset <= mem_size && (offset + size) <= mem_size)) {
		nvgpu_err(g, "flcn-id 0x%x, copy overflow ",
			flcn->flcn_id);
		nvgpu_err(g, "total size 0x%x, offset 0x%x, copy size 0x%x",
			mem_size, offset, size);
		return -EINVAL;
	}

	return 0;
}

static int gk20a_flcn_copy_from_dmem(struct nvgpu_falcon *flcn,
		u32 src, u8 *dst, u32 size, u8 port)
{
	struct gk20a *g = flcn->g;
	u32 base_addr = flcn->flcn_base;
	u32 i, words, bytes;
	u32 data, addr_mask;
	u32 *dst_u32 = (u32 *)dst;

	nvgpu_log_fn(g, " src dmem offset - %x, size - %x", src, size);

	if (flcn_mem_overflow_check(flcn, src, size, MEM_DMEM)) {
		nvgpu_err(g, "incorrect parameters");
		return -EINVAL;
	}

	nvgpu_mutex_acquire(&flcn->copy_lock);

	words = size >> 2;
	bytes = size & 0x3;

	addr_mask = falcon_falcon_dmemc_offs_m() |
			    falcon_falcon_dmemc_blk_m();

	src &= addr_mask;

	gk20a_writel(g, base_addr + falcon_falcon_dmemc_r(port),
		src | falcon_falcon_dmemc_aincr_f(1));

	for (i = 0; i < words; i++) {
		dst_u32[i] = gk20a_readl(g,
			base_addr + falcon_falcon_dmemd_r(port));
	}

	if (bytes > 0) {
		data = gk20a_readl(g, base_addr + falcon_falcon_dmemd_r(port));
		for (i = 0; i < bytes; i++) {
			dst[(words << 2) + i] = ((u8 *)&data)[i];
		}
	}

	nvgpu_mutex_release(&flcn->copy_lock);
	return 0;
}

static int gk20a_flcn_copy_to_dmem(struct nvgpu_falcon *flcn,
		u32 dst, u8 *src, u32 size, u8 port)
{
	struct gk20a *g = flcn->g;
	u32 base_addr = flcn->flcn_base;
	u32 i, words, bytes;
	u32 data, addr_mask;
	u32 *src_u32 = (u32 *)src;

	nvgpu_log_fn(g, "dest dmem offset - %x, size - %x", dst, size);

	if (flcn_mem_overflow_check(flcn, dst, size, MEM_DMEM)) {
		nvgpu_err(g, "incorrect parameters");
		return -EINVAL;
	}

	nvgpu_mutex_acquire(&flcn->copy_lock);

	words = size >> 2;
	bytes = size & 0x3;

	addr_mask = falcon_falcon_dmemc_offs_m() |
		falcon_falcon_dmemc_blk_m();

	dst &= addr_mask;

	gk20a_writel(g, base_addr + falcon_falcon_dmemc_r(port),
		dst | falcon_falcon_dmemc_aincw_f(1));

	for (i = 0; i < words; i++) {
		gk20a_writel(g,
			base_addr + falcon_falcon_dmemd_r(port), src_u32[i]);
	}

	if (bytes > 0) {
		data = 0;
		for (i = 0; i < bytes; i++) {
			((u8 *)&data)[i] = src[(words << 2) + i];
                }
		gk20a_writel(g, base_addr + falcon_falcon_dmemd_r(port), data);
	}

	size = ALIGN(size, 4);
	data = gk20a_readl(g,
		base_addr + falcon_falcon_dmemc_r(port)) & addr_mask;
	if (data != ((dst + size) & addr_mask)) {
		nvgpu_warn(g, "copy failed. bytes written %d, expected %d",
			data - dst, size);
	}

	nvgpu_mutex_release(&flcn->copy_lock);

	return 0;
}

static int gk20a_flcn_copy_from_imem(struct nvgpu_falcon *flcn, u32 src,
	u8 *dst, u32 size, u8 port)
{
	struct gk20a *g = flcn->g;
	u32 base_addr = flcn->flcn_base;
	u32 *dst_u32 = (u32 *)dst;
	u32 words = 0;
	u32 bytes = 0;
	u32 data = 0;
	u32 blk = 0;
	u32 i = 0;

	nvgpu_log_info(g, "download %d bytes from 0x%x", size, src);

	if (flcn_mem_overflow_check(flcn, src, size, MEM_IMEM)) {
		nvgpu_err(g, "incorrect parameters");
		return -EINVAL;
	}

	nvgpu_mutex_acquire(&flcn->copy_lock);

	words = size >> 2;
	bytes = size & 0x3;
	blk = src >> 8;

	nvgpu_log_info(g, "download %d words from 0x%x block %d",
			words, src, blk);

	gk20a_writel(g, base_addr + falcon_falcon_imemc_r(port),
		falcon_falcon_imemc_offs_f(src >> 2) |
		falcon_falcon_imemc_blk_f(blk) |
		falcon_falcon_dmemc_aincr_f(1));

	for (i = 0; i < words; i++) {
		dst_u32[i] = gk20a_readl(g,
			base_addr + falcon_falcon_imemd_r(port));
	}

	if (bytes > 0) {
		data = gk20a_readl(g, base_addr + falcon_falcon_imemd_r(port));
		for (i = 0; i < bytes; i++) {
			dst[(words << 2) + i] = ((u8 *)&data)[i];
		}
	}

	nvgpu_mutex_release(&flcn->copy_lock);

	return 0;
}

static int gk20a_flcn_copy_to_imem(struct nvgpu_falcon *flcn, u32 dst,
		u8 *src, u32 size, u8 port, bool sec, u32 tag)
{
	struct gk20a *g = flcn->g;
	u32 base_addr = flcn->flcn_base;
	u32 *src_u32 = (u32 *)src;
	u32 words = 0;
	u32 blk = 0;
	u32 i = 0;

	nvgpu_log_info(g, "upload %d bytes to 0x%x", size, dst);

	if (flcn_mem_overflow_check(flcn, dst, size, MEM_IMEM)) {
		nvgpu_err(g, "incorrect parameters");
		return -EINVAL;
	}

	nvgpu_mutex_acquire(&flcn->copy_lock);

	words = size >> 2;
	blk = dst >> 8;

	nvgpu_log_info(g, "upload %d words to 0x%x block %d, tag 0x%x",
			words, dst, blk, tag);

	gk20a_writel(g, base_addr + falcon_falcon_imemc_r(port),
			falcon_falcon_imemc_offs_f(dst >> 2) |
			falcon_falcon_imemc_blk_f(blk) |
			/* Set Auto-Increment on write */
			falcon_falcon_imemc_aincw_f(1) |
			falcon_falcon_imemc_secure_f(sec ? 1U : 0U));

	for (i = 0; i < words; i++) {
		if (i % 64 == 0) {
			/* tag is always 256B aligned */
			gk20a_writel(g, base_addr + falcon_falcon_imemt_r(0),
				tag);
			tag++;
		}

		gk20a_writel(g, base_addr + falcon_falcon_imemd_r(port),
			src_u32[i]);
	}

	/* WARNING : setting remaining bytes in block to 0x0 */
	while (i % 64) {
		gk20a_writel(g, base_addr + falcon_falcon_imemd_r(port), 0);
		i++;
	}

	nvgpu_mutex_release(&flcn->copy_lock);

	return 0;
}

static int gk20a_falcon_bootstrap(struct nvgpu_falcon *flcn,
	u32 boot_vector)
{
	struct gk20a *g = flcn->g;
	u32 base_addr = flcn->flcn_base;

	nvgpu_log_info(g, "boot vec 0x%x", boot_vector);

	gk20a_writel(g, base_addr + falcon_falcon_dmactl_r(),
		falcon_falcon_dmactl_require_ctx_f(0));

	gk20a_writel(g, base_addr + falcon_falcon_bootvec_r(),
		falcon_falcon_bootvec_vec_f(boot_vector));

	gk20a_writel(g, base_addr + falcon_falcon_cpuctl_r(),
		falcon_falcon_cpuctl_startcpu_f(1));

	return 0;
}

static u32 gk20a_falcon_mailbox_read(struct nvgpu_falcon *flcn,
		u32 mailbox_index)
{
	struct gk20a *g = flcn->g;
	u32 data = 0;

	if (mailbox_index < FALCON_MAILBOX_COUNT) {
		data =  gk20a_readl(g, flcn->flcn_base + (mailbox_index ?
			falcon_falcon_mailbox1_r() :
			falcon_falcon_mailbox0_r()));
	} else {
		nvgpu_err(g, "incorrect mailbox id %d", mailbox_index);
	}

	return data;
}

static void gk20a_falcon_mailbox_write(struct nvgpu_falcon *flcn,
		u32 mailbox_index, u32 data)
{
	struct gk20a *g = flcn->g;

	if (mailbox_index < FALCON_MAILBOX_COUNT) {
		gk20a_writel(g, flcn->flcn_base + (mailbox_index ?
			falcon_falcon_mailbox1_r() :
			falcon_falcon_mailbox0_r()),
			data);
	} else {
		nvgpu_err(g, "incorrect mailbox id %d", mailbox_index);
	}
}

static int gk20a_falcon_bl_bootstrap(struct nvgpu_falcon *flcn,
	struct nvgpu_falcon_bl_info *bl_info)
{
	struct gk20a *g = flcn->g;
	u32 base_addr = flcn->flcn_base;
	u32 virt_addr = 0;
	u32 dst = 0;
	int err = 0;

	/*copy bootloader interface structure to dmem*/
	err = gk20a_flcn_copy_to_dmem(flcn, 0, (u8 *)bl_info->bl_desc,
		bl_info->bl_desc_size, (u8)0);
	if (err != 0) {
		goto exit;
	}

	/* copy bootloader to TOP of IMEM */
	dst = (falcon_falcon_hwcfg_imem_size_v(gk20a_readl(g,
		base_addr + falcon_falcon_hwcfg_r())) << 8) - bl_info->bl_size;

	err = gk20a_flcn_copy_to_imem(flcn, dst, (u8 *)(bl_info->bl_src),
		bl_info->bl_size, (u8)0, false, bl_info->bl_start_tag);
	if (err != 0) {
		goto exit;
	}

	gk20a_falcon_mailbox_write(flcn, FALCON_MAILBOX_0, 0xDEADA5A5U);

	virt_addr = bl_info->bl_start_tag << 8;

	err = gk20a_falcon_bootstrap(flcn, virt_addr);

exit:
	if (err != 0) {
		nvgpu_err(g, "falcon id-0x%x bootstrap failed", flcn->flcn_id);
	}

	return err;
}

static void gk20a_falcon_dump_imblk(struct nvgpu_falcon *flcn)
{
	struct gk20a *g = flcn->g;
	u32 base_addr = flcn->flcn_base;
	u32 i = 0, j = 0;
	u32 data[8] = {0};
	u32 block_count = 0;

	block_count = falcon_falcon_hwcfg_imem_size_v(gk20a_readl(g,
		flcn->flcn_base + falcon_falcon_hwcfg_r()));

	/* block_count must be multiple of 8 */
	block_count &= ~0x7;
	nvgpu_err(g, "FALCON IMEM BLK MAPPING (PA->VA) (%d TOTAL):",
		block_count);

	for (i = 0; i < block_count; i += 8) {
		for (j = 0; j < 8; j++) {
			gk20a_writel(g, flcn->flcn_base +
			falcon_falcon_imctl_debug_r(),
			falcon_falcon_imctl_debug_cmd_f(0x2) |
			falcon_falcon_imctl_debug_addr_blk_f(i + j));

			data[j] = gk20a_readl(g, base_addr +
				falcon_falcon_imstat_r());
		}

		nvgpu_err(g, " %#04x: %#010x %#010x %#010x %#010x",
				i, data[0], data[1], data[2], data[3]);
		nvgpu_err(g, " %#04x: %#010x %#010x %#010x %#010x",
				i + 4, data[4], data[5], data[6], data[7]);
	}
}

static void gk20a_falcon_dump_pc_trace(struct nvgpu_falcon *flcn)
{
	struct gk20a *g = flcn->g;
	u32 base_addr = flcn->flcn_base;
	u32 trace_pc_count = 0;
	u32 pc = 0;
	u32 i = 0;

	if (gk20a_readl(g, base_addr + falcon_falcon_sctl_r()) & 0x02) {
		nvgpu_err(g, " falcon is in HS mode, PC TRACE dump not supported");
		return;
	}

	trace_pc_count = falcon_falcon_traceidx_maxidx_v(gk20a_readl(g,
		base_addr + falcon_falcon_traceidx_r()));
	nvgpu_err(g,
		"PC TRACE (TOTAL %d ENTRIES. entry 0 is the most recent branch):",
		trace_pc_count);

	for (i = 0; i < trace_pc_count; i++) {
		gk20a_writel(g, base_addr + falcon_falcon_traceidx_r(),
			falcon_falcon_traceidx_idx_f(i));

		pc = falcon_falcon_tracepc_pc_v(gk20a_readl(g,
			base_addr + falcon_falcon_tracepc_r()));
		nvgpu_err(g, "FALCON_TRACEPC(%d)  :  %#010x", i, pc);
	}
}

void gk20a_falcon_dump_stats(struct nvgpu_falcon *flcn)
{
	struct gk20a *g = flcn->g;
	u32 base_addr = flcn->flcn_base;
	unsigned int i;

	nvgpu_err(g, "<<< FALCON id-%d DEBUG INFORMATION - START >>>",
			flcn->flcn_id);

	/* imblk dump */
	gk20a_falcon_dump_imblk(flcn);
	/* PC trace dump */
	gk20a_falcon_dump_pc_trace(flcn);

	nvgpu_err(g, "FALCON ICD REGISTERS DUMP");

	for (i = 0; i < 4; i++) {
		gk20a_writel(g, base_addr + falcon_falcon_icd_cmd_r(),
			falcon_falcon_icd_cmd_opc_rreg_f() |
			falcon_falcon_icd_cmd_idx_f(FALCON_REG_PC));
		nvgpu_err(g, "FALCON_REG_PC : 0x%x",
			gk20a_readl(g, base_addr +
			falcon_falcon_icd_rdata_r()));

		gk20a_writel(g, base_addr + falcon_falcon_icd_cmd_r(),
			falcon_falcon_icd_cmd_opc_rreg_f() |
			falcon_falcon_icd_cmd_idx_f(FALCON_REG_SP));
		nvgpu_err(g, "FALCON_REG_SP : 0x%x",
			gk20a_readl(g, base_addr +
			falcon_falcon_icd_rdata_r()));
	}

	gk20a_writel(g, base_addr + falcon_falcon_icd_cmd_r(),
		falcon_falcon_icd_cmd_opc_rreg_f() |
		falcon_falcon_icd_cmd_idx_f(FALCON_REG_IMB));
	nvgpu_err(g, "FALCON_REG_IMB : 0x%x",
		gk20a_readl(g, base_addr + falcon_falcon_icd_rdata_r()));

	gk20a_writel(g, base_addr + falcon_falcon_icd_cmd_r(),
		falcon_falcon_icd_cmd_opc_rreg_f() |
		falcon_falcon_icd_cmd_idx_f(FALCON_REG_DMB));
	nvgpu_err(g, "FALCON_REG_DMB : 0x%x",
		gk20a_readl(g, base_addr + falcon_falcon_icd_rdata_r()));

	gk20a_writel(g, base_addr + falcon_falcon_icd_cmd_r(),
		falcon_falcon_icd_cmd_opc_rreg_f() |
		falcon_falcon_icd_cmd_idx_f(FALCON_REG_CSW));
	nvgpu_err(g, "FALCON_REG_CSW : 0x%x",
		gk20a_readl(g, base_addr + falcon_falcon_icd_rdata_r()));

	gk20a_writel(g, base_addr + falcon_falcon_icd_cmd_r(),
		falcon_falcon_icd_cmd_opc_rreg_f() |
		falcon_falcon_icd_cmd_idx_f(FALCON_REG_CTX));
	nvgpu_err(g, "FALCON_REG_CTX : 0x%x",
		gk20a_readl(g, base_addr + falcon_falcon_icd_rdata_r()));

	gk20a_writel(g, base_addr + falcon_falcon_icd_cmd_r(),
		falcon_falcon_icd_cmd_opc_rreg_f() |
		falcon_falcon_icd_cmd_idx_f(FALCON_REG_EXCI));
	nvgpu_err(g, "FALCON_REG_EXCI : 0x%x",
		gk20a_readl(g, base_addr + falcon_falcon_icd_rdata_r()));

	for (i = 0; i < 6; i++) {
		gk20a_writel(g, base_addr + falcon_falcon_icd_cmd_r(),
			falcon_falcon_icd_cmd_opc_rreg_f() |
			falcon_falcon_icd_cmd_idx_f(
			falcon_falcon_icd_cmd_opc_rstat_f()));
		nvgpu_err(g, "FALCON_REG_RSTAT[%d] : 0x%x", i,
			gk20a_readl(g, base_addr +
				falcon_falcon_icd_rdata_r()));
	}

	nvgpu_err(g, " FALCON REGISTERS DUMP");
	nvgpu_err(g, "falcon_falcon_os_r : %d",
		gk20a_readl(g, base_addr + falcon_falcon_os_r()));
	nvgpu_err(g, "falcon_falcon_cpuctl_r : 0x%x",
		gk20a_readl(g, base_addr + falcon_falcon_cpuctl_r()));
	nvgpu_err(g, "falcon_falcon_idlestate_r : 0x%x",
		gk20a_readl(g, base_addr + falcon_falcon_idlestate_r()));
	nvgpu_err(g, "falcon_falcon_mailbox0_r : 0x%x",
		gk20a_readl(g, base_addr + falcon_falcon_mailbox0_r()));
	nvgpu_err(g, "falcon_falcon_mailbox1_r : 0x%x",
		gk20a_readl(g, base_addr + falcon_falcon_mailbox1_r()));
	nvgpu_err(g, "falcon_falcon_irqstat_r : 0x%x",
		gk20a_readl(g, base_addr + falcon_falcon_irqstat_r()));
	nvgpu_err(g, "falcon_falcon_irqmode_r : 0x%x",
		gk20a_readl(g, base_addr + falcon_falcon_irqmode_r()));
	nvgpu_err(g, "falcon_falcon_irqmask_r : 0x%x",
		gk20a_readl(g, base_addr + falcon_falcon_irqmask_r()));
	nvgpu_err(g, "falcon_falcon_irqdest_r : 0x%x",
		gk20a_readl(g, base_addr + falcon_falcon_irqdest_r()));
	nvgpu_err(g, "falcon_falcon_debug1_r : 0x%x",
		gk20a_readl(g, base_addr + falcon_falcon_debug1_r()));
	nvgpu_err(g, "falcon_falcon_debuginfo_r : 0x%x",
		gk20a_readl(g, base_addr + falcon_falcon_debuginfo_r()));
	nvgpu_err(g, "falcon_falcon_bootvec_r : 0x%x",
		gk20a_readl(g, base_addr + falcon_falcon_bootvec_r()));
	nvgpu_err(g, "falcon_falcon_hwcfg_r : 0x%x",
		gk20a_readl(g, base_addr + falcon_falcon_hwcfg_r()));
	nvgpu_err(g, "falcon_falcon_engctl_r : 0x%x",
		gk20a_readl(g, base_addr + falcon_falcon_engctl_r()));
	nvgpu_err(g, "falcon_falcon_curctx_r : 0x%x",
		gk20a_readl(g, base_addr + falcon_falcon_curctx_r()));
	nvgpu_err(g, "falcon_falcon_nxtctx_r : 0x%x",
		gk20a_readl(g, base_addr + falcon_falcon_nxtctx_r()));
	nvgpu_err(g, "falcon_falcon_exterrstat_r : 0x%x",
		gk20a_readl(g, base_addr + falcon_falcon_exterrstat_r()));
	nvgpu_err(g, "falcon_falcon_exterraddr_r : 0x%x",
		gk20a_readl(g, base_addr + falcon_falcon_exterraddr_r()));
}

static void gk20a_falcon_engine_dependency_ops(struct nvgpu_falcon *flcn)
{
	struct gk20a *g = flcn->g;
	struct nvgpu_falcon_engine_dependency_ops *flcn_eng_dep_ops =
			&flcn->flcn_engine_dep_ops;

	switch (flcn->flcn_id) {
	case FALCON_ID_PMU:
		flcn_eng_dep_ops->reset_eng = nvgpu_pmu_reset;
		flcn_eng_dep_ops->queue_head = g->ops.pmu.pmu_queue_head;
		flcn_eng_dep_ops->queue_tail = g->ops.pmu.pmu_queue_tail;
		break;
	default:
		/* NULL assignment make sure
		 * CPU hard reset in gk20a_flcn_reset() gets execute
		 * if falcon doesn't need specific reset implementation
		 */
		flcn_eng_dep_ops->reset_eng = NULL;
		break;
	}
}

void gk20a_falcon_ops(struct nvgpu_falcon *flcn)
{
	struct nvgpu_falcon_ops *flcn_ops = &flcn->flcn_ops;

	flcn_ops->reset = gk20a_flcn_reset;
	flcn_ops->set_irq = gk20a_flcn_set_irq;
	flcn_ops->clear_halt_interrupt_status =
		gk20a_flcn_clear_halt_interrupt_status;
	flcn_ops->is_falcon_cpu_halted =  gk20a_is_falcon_cpu_halted;
	flcn_ops->is_falcon_idle =  gk20a_is_falcon_idle;
	flcn_ops->is_falcon_scrubbing_done =  gk20a_is_falcon_scrubbing_done;
	flcn_ops->copy_from_dmem = gk20a_flcn_copy_from_dmem;
	flcn_ops->copy_to_dmem = gk20a_flcn_copy_to_dmem;
	flcn_ops->copy_to_imem = gk20a_flcn_copy_to_imem;
	flcn_ops->copy_from_imem = gk20a_flcn_copy_from_imem;
	flcn_ops->bootstrap = gk20a_falcon_bootstrap;
	flcn_ops->dump_falcon_stats = gk20a_falcon_dump_stats;
	flcn_ops->mailbox_read = gk20a_falcon_mailbox_read;
	flcn_ops->mailbox_write = gk20a_falcon_mailbox_write;
	flcn_ops->bl_bootstrap = gk20a_falcon_bl_bootstrap;

	gk20a_falcon_engine_dependency_ops(flcn);
}

int gk20a_falcon_hal_sw_init(struct nvgpu_falcon *flcn)
{
	struct gk20a *g = flcn->g;
	int err = 0;

	switch (flcn->flcn_id) {
	case FALCON_ID_PMU:
		flcn->flcn_base = FALCON_PWR_BASE;
		flcn->is_falcon_supported = true;
		flcn->is_interrupt_enabled = true;
		break;
	case FALCON_ID_SEC2:
		flcn->flcn_base = FALCON_SEC_BASE;
		flcn->is_falcon_supported = false;
		flcn->is_interrupt_enabled = false;
		break;
	case FALCON_ID_FECS:
		flcn->flcn_base = FALCON_FECS_BASE;
		flcn->is_falcon_supported = true;
		flcn->is_interrupt_enabled = false;
		break;
	case FALCON_ID_GPCCS:
		flcn->flcn_base = FALCON_GPCCS_BASE;
		flcn->is_falcon_supported = true;
		flcn->is_interrupt_enabled = false;
		break;
	case FALCON_ID_NVDEC:
		flcn->flcn_base = FALCON_NVDEC_BASE;
		flcn->is_falcon_supported = false;
		flcn->is_interrupt_enabled = false;
		break;
	default:
		flcn->is_falcon_supported = false;
		break;
	}

	if (flcn->is_falcon_supported) {
		err = nvgpu_mutex_init(&flcn->copy_lock);
		if (err != 0) {
			nvgpu_err(g, "Error in flcn.copy_lock mutex initialization");
		} else {
			gk20a_falcon_ops(flcn);
		}
	} else {
		nvgpu_log_info(g, "falcon 0x%x not supported on %s",
			flcn->flcn_id, g->name);
	}

	return err;
}
