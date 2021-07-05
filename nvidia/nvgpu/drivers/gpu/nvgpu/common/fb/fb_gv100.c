/*
 * GV100 FB
 *
 * Copyright (c) 2017-2020, NVIDIA CORPORATION.  All rights reserved.
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

#include <nvgpu/types.h>

#include <nvgpu/dma.h>
#include <nvgpu/log.h>
#include <nvgpu/enabled.h>
#include <nvgpu/gmmu.h>
#include <nvgpu/nvgpu_common.h>
#include <nvgpu/kmem.h>
#include <nvgpu/nvgpu_mem.h>
#include <nvgpu/acr/nvgpu_acr.h>
#include <nvgpu/firmware.h>
#include <nvgpu/pmu.h>
#include <nvgpu/falcon.h>
#include <nvgpu/io.h>
#include <nvgpu/utils.h>
#include <nvgpu/timers.h>
#include <nvgpu/gk20a.h>
#include <nvgpu/unit.h>

#include "fb_gv100.h"

#include <nvgpu/hw/gv100/hw_fb_gv100.h>

#define HW_SCRUB_TIMEOUT_DEFAULT	100 /* usec */
#define HW_SCRUB_TIMEOUT_MAX		2000000 /* usec */
#define MEM_UNLOCK_TIMEOUT			3500 /* msec */

#define MEM_UNLOCK_PROD_BIN		"mem_unlock.bin"
#define MEM_UNLOCK_DBG_BIN		"mem_unlock_dbg.bin"

void gv100_fb_reset(struct gk20a *g)
{
	u32 val;
	int retries = HW_SCRUB_TIMEOUT_MAX / HW_SCRUB_TIMEOUT_DEFAULT;

	nvgpu_log_info(g, "reset gv100 fb");

	/* wait for memory to be accessible */
	do {
		u32 w = gk20a_readl(g, fb_niso_scrub_status_r());
		if (fb_niso_scrub_status_flag_v(w)) {
			nvgpu_log_info(g, "done");
			break;
		}
		nvgpu_udelay(HW_SCRUB_TIMEOUT_DEFAULT);
	} while (--retries);

	val = gk20a_readl(g, fb_mmu_priv_level_mask_r());
	val &= ~fb_mmu_priv_level_mask_write_violation_m();
	gk20a_writel(g, fb_mmu_priv_level_mask_r(), val);
}

void gv100_fb_enable_hub_intr(struct gk20a *g)
{
	u32 mask = 0;

	mask = fb_niso_intr_en_set_mmu_other_fault_notify_m() |
			fb_niso_intr_en_set_mmu_nonreplayable_fault_notify_m() |
			fb_niso_intr_en_set_mmu_nonreplayable_fault_overflow_m() |
			fb_niso_intr_en_set_mmu_replayable_fault_notify_m() |
			fb_niso_intr_en_set_mmu_replayable_fault_overflow_m();

	gk20a_writel(g, fb_niso_intr_en_set_r(0),
			mask);
}

void gv100_fb_disable_hub_intr(struct gk20a *g)
{
	u32 mask = 0;

	mask = fb_niso_intr_en_set_mmu_other_fault_notify_m() |
			fb_niso_intr_en_set_mmu_nonreplayable_fault_notify_m() |
			fb_niso_intr_en_set_mmu_nonreplayable_fault_overflow_m() |
			fb_niso_intr_en_set_mmu_replayable_fault_notify_m() |
			fb_niso_intr_en_set_mmu_replayable_fault_overflow_m();

	gk20a_writel(g, fb_niso_intr_en_clr_r(0),
			mask);
}

/*
 * @brief Patch signatures into ucode image
 */
static int gv100_fb_acr_ucode_patch_sig(struct gk20a *g,
		u32 *p_img,
		u32 *p_prod_sig,
		u32 *p_dbg_sig,
		u32 *p_patch_loc,
		u32 *p_patch_ind)
{
	u32 *p_sig;

	if (!g->ops.pmu.is_debug_mode_enabled(g)) {
		p_sig = p_prod_sig;
	} else {
		p_sig = p_dbg_sig;
	}

	/* Patching logic. We have just one location to patch. */
	p_img[(*p_patch_loc>>2)] = p_sig[(*p_patch_ind<<2)];
	p_img[(*p_patch_loc>>2)+1U] = p_sig[(*p_patch_ind<<2)+1U];
	p_img[(*p_patch_loc>>2)+2U] = p_sig[(*p_patch_ind<<2)+2U];
	p_img[(*p_patch_loc>>2)+3U] = p_sig[(*p_patch_ind<<2)+3U];
	return 0;
}

int gv100_fb_memory_unlock(struct gk20a *g)
{
	struct nvgpu_firmware *mem_unlock_fw = NULL;
	struct bin_hdr *hsbin_hdr = NULL;
	struct acr_fw_header *fw_hdr = NULL;
	u32 *mem_unlock_ucode = NULL;
	u32 *mem_unlock_ucode_header = NULL;
	u32 sec_imem_dest = 0;
	u32 val = 0;
	int err = 0;

	nvgpu_log_fn(g, " ");

	/*
	 * mem_unlock.bin should be written to install
	 * traps even if VPR isn’t actually supported
	 */
	if (!g->ops.pmu.is_debug_mode_enabled(g)) {
		mem_unlock_fw = nvgpu_request_firmware(g, MEM_UNLOCK_PROD_BIN, 0);
	} else {
		mem_unlock_fw = nvgpu_request_firmware(g, MEM_UNLOCK_DBG_BIN, 0);
	}
	if (!mem_unlock_fw) {
		nvgpu_err(g, "mem unlock ucode get fail");
		err = -ENOENT;
		goto exit;
	}

	/* Enable nvdec */
	g->ops.mc.enable(g, g->ops.mc.reset_mask(g, NVGPU_UNIT_NVDEC));

	/* nvdec falcon reset */
	nvgpu_flcn_reset(&g->nvdec_flcn);

	hsbin_hdr = (struct bin_hdr *)mem_unlock_fw->data;
	fw_hdr = (struct acr_fw_header *)(mem_unlock_fw->data +
			hsbin_hdr->header_offset);

	mem_unlock_ucode_header = (u32 *)(mem_unlock_fw->data +
		fw_hdr->hdr_offset);
	mem_unlock_ucode = (u32 *)(mem_unlock_fw->data +
		hsbin_hdr->data_offset);

	/* Patch Ucode signatures */
	if (gv100_fb_acr_ucode_patch_sig(g, mem_unlock_ucode,
		(u32 *)(mem_unlock_fw->data + fw_hdr->sig_prod_offset),
		(u32 *)(mem_unlock_fw->data + fw_hdr->sig_dbg_offset),
		(u32 *)(mem_unlock_fw->data + fw_hdr->patch_loc),
		(u32 *)(mem_unlock_fw->data + fw_hdr->patch_sig)) < 0) {
		nvgpu_err(g, "mem unlock patch signatures fail");
		err = -EPERM;
		goto exit;
	}

	/* Clear interrupts */
	nvgpu_flcn_set_irq(&g->nvdec_flcn, false, 0x0, 0x0);

	/* Copy Non Secure IMEM code */
	nvgpu_flcn_copy_to_imem(&g->nvdec_flcn, 0,
		(u8 *)&mem_unlock_ucode[
			mem_unlock_ucode_header[OS_CODE_OFFSET] >> 2],
		mem_unlock_ucode_header[OS_CODE_SIZE], 0, false,
		GET_IMEM_TAG(mem_unlock_ucode_header[OS_CODE_OFFSET]));

	/* Put secure code after non-secure block */
	sec_imem_dest = GET_NEXT_BLOCK(mem_unlock_ucode_header[OS_CODE_SIZE]);

	nvgpu_flcn_copy_to_imem(&g->nvdec_flcn, sec_imem_dest,
		(u8 *)&mem_unlock_ucode[
			mem_unlock_ucode_header[APP_0_CODE_OFFSET] >> 2],
		mem_unlock_ucode_header[APP_0_CODE_SIZE], 0, true,
		GET_IMEM_TAG(mem_unlock_ucode_header[APP_0_CODE_OFFSET]));

	/* load DMEM: ensure that signatures are patched */
	nvgpu_flcn_copy_to_dmem(&g->nvdec_flcn, 0, (u8 *)&mem_unlock_ucode[
		mem_unlock_ucode_header[OS_DATA_OFFSET] >> 2],
		mem_unlock_ucode_header[OS_DATA_SIZE], 0);

	/* Write non-zero value to mailbox register which is updated by
	 * mem_unlock bin to denote its return status.
	 */
	nvgpu_flcn_mailbox_write(&g->nvdec_flcn, 0, 0xdeadbeef);

	/* set BOOTVEC to start of non-secure code */
	nvgpu_flcn_bootstrap(&g->nvdec_flcn, 0);

	/* wait for complete & halt */
	nvgpu_flcn_wait_for_halt(&g->nvdec_flcn, MEM_UNLOCK_TIMEOUT);

	/* check mem unlock status */
	val = nvgpu_flcn_mailbox_read(&g->nvdec_flcn, 0);
	if (val) {
		nvgpu_err(g, "memory unlock failed, err %x", val);
		nvgpu_flcn_dump_stats(&g->nvdec_flcn);
		err = -1;
		goto exit;
	}

exit:
	if (mem_unlock_fw) {
		nvgpu_release_firmware(g, mem_unlock_fw);
	}

	nvgpu_log_fn(g, "done, status - %d", err);

	return err;
}

int gv100_fb_init_nvlink(struct gk20a *g)
{
	u32 data;
	u32 mask = g->nvlink.enabled_links;

	/* Map enabled link to SYSMEM */
	data = nvgpu_readl(g, fb_hshub_config0_r());
	data = set_field(data, fb_hshub_config0_sysmem_nvlink_mask_m(),
			fb_hshub_config0_sysmem_nvlink_mask_f(mask));
	nvgpu_writel(g, fb_hshub_config0_r(), data);

	return 0;
}

int gv100_fb_enable_nvlink(struct gk20a *g)
{
	u32 data;

	nvgpu_log(g, gpu_dbg_nvlink|gpu_dbg_info, "enabling nvlink");

	/* Enable nvlink for NISO FBHUB */
	data = nvgpu_readl(g, fb_niso_cfg1_r());
	data = set_field(data, fb_niso_cfg1_sysmem_nvlink_m(),
		fb_niso_cfg1_sysmem_nvlink_enabled_f());
	nvgpu_writel(g, fb_niso_cfg1_r(), data);

	/* Setup atomics */
	data = nvgpu_readl(g, fb_mmu_ctrl_r());
	data = set_field(data, fb_mmu_ctrl_atomic_capability_mode_m(),
		fb_mmu_ctrl_atomic_capability_mode_rmw_f());
	nvgpu_writel(g, fb_mmu_ctrl_r(), data);

	data = nvgpu_readl(g, fb_hsmmu_pri_mmu_ctrl_r());
	data = set_field(data, fb_hsmmu_pri_mmu_ctrl_atomic_capability_mode_m(),
		    fb_hsmmu_pri_mmu_ctrl_atomic_capability_mode_rmw_f());
	nvgpu_writel(g, fb_hsmmu_pri_mmu_ctrl_r(), data);

	data = nvgpu_readl(g, fb_fbhub_num_active_ltcs_r());
	data = set_field(data, fb_fbhub_num_active_ltcs_hub_sys_atomic_mode_m(),
		    fb_fbhub_num_active_ltcs_hub_sys_atomic_mode_use_rmw_f());
	nvgpu_writel(g, fb_fbhub_num_active_ltcs_r(), data);

	data = nvgpu_readl(g, fb_hshub_num_active_ltcs_r());
	data = set_field(data, fb_hshub_num_active_ltcs_hub_sys_atomic_mode_m(),
		    fb_hshub_num_active_ltcs_hub_sys_atomic_mode_use_rmw_f());
	nvgpu_writel(g, fb_hshub_num_active_ltcs_r(), data);

	return 0;
}

size_t gv100_fb_get_vidmem_size(struct gk20a *g)
{
	u32 range = gk20a_readl(g, fb_mmu_local_memory_range_r());
	u32 mag = fb_mmu_local_memory_range_lower_mag_v(range);
	u32 scale = fb_mmu_local_memory_range_lower_scale_v(range);
	u32 ecc = fb_mmu_local_memory_range_ecc_mode_v(range);
	size_t bytes = ((size_t)mag << scale) * SZ_1M;

	if (ecc) {
		bytes = bytes / 16U * 15U;
	}

	return bytes;
}

void gv100_fb_set_mmu_debug_mode(struct gk20a *g, bool enable)
{
	u32 data, fb_ctrl, hsmmu_ctrl;

	if (enable) {
		fb_ctrl = fb_mmu_debug_ctrl_debug_enabled_f();
		hsmmu_ctrl = fb_hsmmu_pri_mmu_debug_ctrl_debug_enabled_f();
		g->mmu_debug_ctrl = true;
	} else {
		fb_ctrl = fb_mmu_debug_ctrl_debug_disabled_f();
		hsmmu_ctrl = fb_hsmmu_pri_mmu_debug_ctrl_debug_disabled_f();
		g->mmu_debug_ctrl = false;
	}

	data = nvgpu_readl(g, fb_mmu_debug_ctrl_r());
	data = set_field(data, fb_mmu_debug_ctrl_debug_m(), fb_ctrl);
	nvgpu_writel(g, fb_mmu_debug_ctrl_r(), data);

	data = nvgpu_readl(g, fb_hsmmu_pri_mmu_debug_ctrl_r());
	data = set_field(data,
			fb_hsmmu_pri_mmu_debug_ctrl_debug_m(), hsmmu_ctrl);
	nvgpu_writel(g, fb_hsmmu_pri_mmu_debug_ctrl_r(), data);
}
