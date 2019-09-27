/*
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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
/*
 * Function naming determines intended use:
 *
 *     <x>_r(void) : Returns the offset for register <x>.
 *
 *     <x>_o(void) : Returns the offset for element <x>.
 *
 *     <x>_w(void) : Returns the word offset for word (4 byte) element <x>.
 *
 *     <x>_<y>_s(void) : Returns size of field <y> of register <x> in bits.
 *
 *     <x>_<y>_f(u32 v) : Returns a value based on 'v' which has been shifted
 *         and masked to place it at field <y> of register <x>.  This value
 *         can be |'d with others to produce a full register value for
 *         register <x>.
 *
 *     <x>_<y>_m(void) : Returns a mask for field <y> of register <x>.  This
 *         value can be ~'d and then &'d to clear the value of field <y> for
 *         register <x>.
 *
 *     <x>_<y>_<z>_f(void) : Returns the constant value <z> after being shifted
 *         to place it at field <y> of register <x>.  This value can be |'d
 *         with others to produce a full register value for <x>.
 *
 *     <x>_<y>_v(u32 r) : Returns the value of field <y> from a full register
 *         <x> value 'r' after being shifted to place its LSB at bit 0.
 *         This value is suitable for direct comparison with other unshifted
 *         values appropriate for use in field <y> of register <x>.
 *
 *     <x>_<y>_<z>_v(void) : Returns the constant value for <z> defined for
 *         field <y> of register <x>.  This value is suitable for direct
 *         comparison with unshifted values appropriate for use in field <y>
 *         of register <x>.
 */
#ifndef _hw_pgsp_gv100_h_
#define _hw_pgsp_gv100_h_

static inline u32 pgsp_falcon_irqsset_r(void)
{
	return 0x00110000U;
}
static inline u32 pgsp_falcon_irqsset_swgen0_set_f(void)
{
	return 0x40U;
}
static inline u32 pgsp_falcon_irqsclr_r(void)
{
	return 0x00110004U;
}
static inline u32 pgsp_falcon_irqstat_r(void)
{
	return 0x00110008U;
}
static inline u32 pgsp_falcon_irqstat_halt_true_f(void)
{
	return 0x10U;
}
static inline u32 pgsp_falcon_irqstat_exterr_true_f(void)
{
	return 0x20U;
}
static inline u32 pgsp_falcon_irqstat_swgen0_true_f(void)
{
	return 0x40U;
}
static inline u32 pgsp_falcon_irqmode_r(void)
{
	return 0x0011000cU;
}
static inline u32 pgsp_falcon_irqmset_r(void)
{
	return 0x00110010U;
}
static inline u32 pgsp_falcon_irqmset_gptmr_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 pgsp_falcon_irqmset_wdtmr_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 pgsp_falcon_irqmset_mthd_f(u32 v)
{
	return (v & 0x1U) << 2U;
}
static inline u32 pgsp_falcon_irqmset_ctxsw_f(u32 v)
{
	return (v & 0x1U) << 3U;
}
static inline u32 pgsp_falcon_irqmset_halt_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 pgsp_falcon_irqmset_exterr_f(u32 v)
{
	return (v & 0x1U) << 5U;
}
static inline u32 pgsp_falcon_irqmset_swgen0_f(u32 v)
{
	return (v & 0x1U) << 6U;
}
static inline u32 pgsp_falcon_irqmset_swgen1_f(u32 v)
{
	return (v & 0x1U) << 7U;
}
static inline u32 pgsp_falcon_irqmclr_r(void)
{
	return 0x00110014U;
}
static inline u32 pgsp_falcon_irqmclr_gptmr_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 pgsp_falcon_irqmclr_wdtmr_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 pgsp_falcon_irqmclr_mthd_f(u32 v)
{
	return (v & 0x1U) << 2U;
}
static inline u32 pgsp_falcon_irqmclr_ctxsw_f(u32 v)
{
	return (v & 0x1U) << 3U;
}
static inline u32 pgsp_falcon_irqmclr_halt_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 pgsp_falcon_irqmclr_exterr_f(u32 v)
{
	return (v & 0x1U) << 5U;
}
static inline u32 pgsp_falcon_irqmclr_swgen0_f(u32 v)
{
	return (v & 0x1U) << 6U;
}
static inline u32 pgsp_falcon_irqmclr_swgen1_f(u32 v)
{
	return (v & 0x1U) << 7U;
}
static inline u32 pgsp_falcon_irqmclr_ext_f(u32 v)
{
	return (v & 0xffU) << 8U;
}
static inline u32 pgsp_falcon_irqmask_r(void)
{
	return 0x00110018U;
}
static inline u32 pgsp_falcon_irqdest_r(void)
{
	return 0x0011001cU;
}
static inline u32 pgsp_falcon_irqdest_host_gptmr_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 pgsp_falcon_irqdest_host_wdtmr_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 pgsp_falcon_irqdest_host_mthd_f(u32 v)
{
	return (v & 0x1U) << 2U;
}
static inline u32 pgsp_falcon_irqdest_host_ctxsw_f(u32 v)
{
	return (v & 0x1U) << 3U;
}
static inline u32 pgsp_falcon_irqdest_host_halt_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 pgsp_falcon_irqdest_host_exterr_f(u32 v)
{
	return (v & 0x1U) << 5U;
}
static inline u32 pgsp_falcon_irqdest_host_swgen0_f(u32 v)
{
	return (v & 0x1U) << 6U;
}
static inline u32 pgsp_falcon_irqdest_host_swgen1_f(u32 v)
{
	return (v & 0x1U) << 7U;
}
static inline u32 pgsp_falcon_irqdest_host_ext_f(u32 v)
{
	return (v & 0xffU) << 8U;
}
static inline u32 pgsp_falcon_irqdest_target_gptmr_f(u32 v)
{
	return (v & 0x1U) << 16U;
}
static inline u32 pgsp_falcon_irqdest_target_wdtmr_f(u32 v)
{
	return (v & 0x1U) << 17U;
}
static inline u32 pgsp_falcon_irqdest_target_mthd_f(u32 v)
{
	return (v & 0x1U) << 18U;
}
static inline u32 pgsp_falcon_irqdest_target_ctxsw_f(u32 v)
{
	return (v & 0x1U) << 19U;
}
static inline u32 pgsp_falcon_irqdest_target_halt_f(u32 v)
{
	return (v & 0x1U) << 20U;
}
static inline u32 pgsp_falcon_irqdest_target_exterr_f(u32 v)
{
	return (v & 0x1U) << 21U;
}
static inline u32 pgsp_falcon_irqdest_target_swgen0_f(u32 v)
{
	return (v & 0x1U) << 22U;
}
static inline u32 pgsp_falcon_irqdest_target_swgen1_f(u32 v)
{
	return (v & 0x1U) << 23U;
}
static inline u32 pgsp_falcon_irqdest_target_ext_f(u32 v)
{
	return (v & 0xffU) << 24U;
}
static inline u32 pgsp_falcon_curctx_r(void)
{
	return 0x00110050U;
}
static inline u32 pgsp_falcon_nxtctx_r(void)
{
	return 0x00110054U;
}
static inline u32 pgsp_falcon_nxtctx_ctxptr_f(u32 v)
{
	return (v & 0xfffffffU) << 0U;
}
static inline u32 pgsp_falcon_nxtctx_ctxtgt_fb_f(void)
{
	return 0x0U;
}
static inline u32 pgsp_falcon_nxtctx_ctxtgt_sys_coh_f(void)
{
	return 0x20000000U;
}
static inline u32 pgsp_falcon_nxtctx_ctxtgt_sys_ncoh_f(void)
{
	return 0x30000000U;
}
static inline u32 pgsp_falcon_nxtctx_ctxvalid_f(u32 v)
{
	return (v & 0x1U) << 30U;
}
static inline u32 pgsp_falcon_mailbox0_r(void)
{
	return 0x00110040U;
}
static inline u32 pgsp_falcon_mailbox1_r(void)
{
	return 0x00110044U;
}
static inline u32 pgsp_falcon_itfen_r(void)
{
	return 0x00110048U;
}
static inline u32 pgsp_falcon_itfen_ctxen_enable_f(void)
{
	return 0x1U;
}
static inline u32 pgsp_falcon_idlestate_r(void)
{
	return 0x0011004cU;
}
static inline u32 pgsp_falcon_idlestate_falcon_busy_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 pgsp_falcon_idlestate_ext_busy_v(u32 r)
{
	return (r >> 1U) & 0x7fffU;
}
static inline u32 pgsp_falcon_os_r(void)
{
	return 0x00110080U;
}
static inline u32 pgsp_falcon_engctl_r(void)
{
	return 0x001100a4U;
}
static inline u32 pgsp_falcon_engctl_switch_context_true_f(void)
{
	return 0x8U;
}
static inline u32 pgsp_falcon_engctl_switch_context_false_f(void)
{
	return 0x0U;
}
static inline u32 pgsp_falcon_cpuctl_r(void)
{
	return 0x00110100U;
}
static inline u32 pgsp_falcon_cpuctl_startcpu_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 pgsp_falcon_cpuctl_halt_intr_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 pgsp_falcon_cpuctl_halt_intr_m(void)
{
	return 0x1U << 4U;
}
static inline u32 pgsp_falcon_cpuctl_halt_intr_v(u32 r)
{
	return (r >> 4U) & 0x1U;
}
static inline u32 pgsp_falcon_cpuctl_cpuctl_alias_en_f(u32 v)
{
	return (v & 0x1U) << 6U;
}
static inline u32 pgsp_falcon_cpuctl_cpuctl_alias_en_m(void)
{
	return 0x1U << 6U;
}
static inline u32 pgsp_falcon_cpuctl_cpuctl_alias_en_v(u32 r)
{
	return (r >> 6U) & 0x1U;
}
static inline u32 pgsp_falcon_cpuctl_alias_r(void)
{
	return 0x00110130U;
}
static inline u32 pgsp_falcon_cpuctl_alias_startcpu_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 pgsp_falcon_imemc_r(u32 i)
{
	return 0x00110180U + i*16U;
}
static inline u32 pgsp_falcon_imemc_offs_f(u32 v)
{
	return (v & 0x3fU) << 2U;
}
static inline u32 pgsp_falcon_imemc_blk_f(u32 v)
{
	return (v & 0xffU) << 8U;
}
static inline u32 pgsp_falcon_imemc_aincw_f(u32 v)
{
	return (v & 0x1U) << 24U;
}
static inline u32 pgsp_falcon_imemd_r(u32 i)
{
	return 0x00110184U + i*16U;
}
static inline u32 pgsp_falcon_imemt_r(u32 i)
{
	return 0x00110188U + i*16U;
}
static inline u32 pgsp_falcon_sctl_r(void)
{
	return 0x00110240U;
}
static inline u32 pgsp_falcon_mmu_phys_sec_r(void)
{
	return 0x00100ce4U;
}
static inline u32 pgsp_falcon_bootvec_r(void)
{
	return 0x00110104U;
}
static inline u32 pgsp_falcon_bootvec_vec_f(u32 v)
{
	return (v & 0xffffffffU) << 0U;
}
static inline u32 pgsp_falcon_dmactl_r(void)
{
	return 0x0011010cU;
}
static inline u32 pgsp_falcon_dmactl_dmem_scrubbing_m(void)
{
	return 0x1U << 1U;
}
static inline u32 pgsp_falcon_dmactl_imem_scrubbing_m(void)
{
	return 0x1U << 2U;
}
static inline u32 pgsp_falcon_dmactl_require_ctx_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 pgsp_falcon_hwcfg_r(void)
{
	return 0x00110108U;
}
static inline u32 pgsp_falcon_hwcfg_imem_size_v(u32 r)
{
	return (r >> 0U) & 0x1ffU;
}
static inline u32 pgsp_falcon_hwcfg_dmem_size_v(u32 r)
{
	return (r >> 9U) & 0x1ffU;
}
static inline u32 pgsp_falcon_dmatrfbase_r(void)
{
	return 0x00110110U;
}
static inline u32 pgsp_falcon_dmatrfbase1_r(void)
{
	return 0x00110128U;
}
static inline u32 pgsp_falcon_dmatrfmoffs_r(void)
{
	return 0x00110114U;
}
static inline u32 pgsp_falcon_dmatrfcmd_r(void)
{
	return 0x00110118U;
}
static inline u32 pgsp_falcon_dmatrfcmd_imem_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 pgsp_falcon_dmatrfcmd_write_f(u32 v)
{
	return (v & 0x1U) << 5U;
}
static inline u32 pgsp_falcon_dmatrfcmd_size_f(u32 v)
{
	return (v & 0x7U) << 8U;
}
static inline u32 pgsp_falcon_dmatrfcmd_ctxdma_f(u32 v)
{
	return (v & 0x7U) << 12U;
}
static inline u32 pgsp_falcon_dmatrffboffs_r(void)
{
	return 0x0011011cU;
}
static inline u32 pgsp_falcon_exterraddr_r(void)
{
	return 0x00110168U;
}
static inline u32 pgsp_falcon_exterrstat_r(void)
{
	return 0x0011016cU;
}
static inline u32 pgsp_falcon_exterrstat_valid_m(void)
{
	return 0x1U << 31U;
}
static inline u32 pgsp_falcon_exterrstat_valid_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 pgsp_falcon_exterrstat_valid_true_v(void)
{
	return 0x00000001U;
}
static inline u32 pgsp_sec2_falcon_icd_cmd_r(void)
{
	return 0x00110200U;
}
static inline u32 pgsp_sec2_falcon_icd_cmd_opc_s(void)
{
	return 4U;
}
static inline u32 pgsp_sec2_falcon_icd_cmd_opc_f(u32 v)
{
	return (v & 0xfU) << 0U;
}
static inline u32 pgsp_sec2_falcon_icd_cmd_opc_m(void)
{
	return 0xfU << 0U;
}
static inline u32 pgsp_sec2_falcon_icd_cmd_opc_v(u32 r)
{
	return (r >> 0U) & 0xfU;
}
static inline u32 pgsp_sec2_falcon_icd_cmd_opc_rreg_f(void)
{
	return 0x8U;
}
static inline u32 pgsp_sec2_falcon_icd_cmd_opc_rstat_f(void)
{
	return 0xeU;
}
static inline u32 pgsp_sec2_falcon_icd_cmd_idx_f(u32 v)
{
	return (v & 0x1fU) << 8U;
}
static inline u32 pgsp_sec2_falcon_icd_rdata_r(void)
{
	return 0x0011020cU;
}
static inline u32 pgsp_falcon_dmemc_r(u32 i)
{
	return 0x001101c0U + i*8U;
}
static inline u32 pgsp_falcon_dmemc_offs_f(u32 v)
{
	return (v & 0x3fU) << 2U;
}
static inline u32 pgsp_falcon_dmemc_offs_m(void)
{
	return 0x3fU << 2U;
}
static inline u32 pgsp_falcon_dmemc_blk_f(u32 v)
{
	return (v & 0xffU) << 8U;
}
static inline u32 pgsp_falcon_dmemc_blk_m(void)
{
	return 0xffU << 8U;
}
static inline u32 pgsp_falcon_dmemc_aincw_f(u32 v)
{
	return (v & 0x1U) << 24U;
}
static inline u32 pgsp_falcon_dmemc_aincr_f(u32 v)
{
	return (v & 0x1U) << 25U;
}
static inline u32 pgsp_falcon_dmemd_r(u32 i)
{
	return 0x001101c4U + i*8U;
}
static inline u32 pgsp_falcon_debug1_r(void)
{
	return 0x00110090U;
}
static inline u32 pgsp_falcon_debug1_ctxsw_mode_s(void)
{
	return 1U;
}
static inline u32 pgsp_falcon_debug1_ctxsw_mode_f(u32 v)
{
	return (v & 0x1U) << 16U;
}
static inline u32 pgsp_falcon_debug1_ctxsw_mode_m(void)
{
	return 0x1U << 16U;
}
static inline u32 pgsp_falcon_debug1_ctxsw_mode_v(u32 r)
{
	return (r >> 16U) & 0x1U;
}
static inline u32 pgsp_falcon_debug1_ctxsw_mode_init_f(void)
{
	return 0x0U;
}
static inline u32 pgsp_fbif_transcfg_r(u32 i)
{
	return 0x00110600U + i*4U;
}
static inline u32 pgsp_fbif_transcfg_target_local_fb_f(void)
{
	return 0x0U;
}
static inline u32 pgsp_fbif_transcfg_target_coherent_sysmem_f(void)
{
	return 0x1U;
}
static inline u32 pgsp_fbif_transcfg_target_noncoherent_sysmem_f(void)
{
	return 0x2U;
}
static inline u32 pgsp_fbif_transcfg_mem_type_s(void)
{
	return 1U;
}
static inline u32 pgsp_fbif_transcfg_mem_type_f(u32 v)
{
	return (v & 0x1U) << 2U;
}
static inline u32 pgsp_fbif_transcfg_mem_type_m(void)
{
	return 0x1U << 2U;
}
static inline u32 pgsp_fbif_transcfg_mem_type_v(u32 r)
{
	return (r >> 2U) & 0x1U;
}
static inline u32 pgsp_fbif_transcfg_mem_type_virtual_f(void)
{
	return 0x0U;
}
static inline u32 pgsp_fbif_transcfg_mem_type_physical_f(void)
{
	return 0x4U;
}
static inline u32 pgsp_falcon_engine_r(void)
{
	return 0x001103c0U;
}
static inline u32 pgsp_falcon_engine_reset_true_f(void)
{
	return 0x1U;
}
static inline u32 pgsp_falcon_engine_reset_false_f(void)
{
	return 0x0U;
}
static inline u32 pgsp_fbif_ctl_r(void)
{
	return 0x00110624U;
}
static inline u32 pgsp_fbif_ctl_allow_phys_no_ctx_init_f(void)
{
	return 0x0U;
}
static inline u32 pgsp_fbif_ctl_allow_phys_no_ctx_disallow_f(void)
{
	return 0x0U;
}
static inline u32 pgsp_fbif_ctl_allow_phys_no_ctx_allow_f(void)
{
	return 0x80U;
}
#endif
