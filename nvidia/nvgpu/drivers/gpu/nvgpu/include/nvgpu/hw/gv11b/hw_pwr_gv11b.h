/*
 * Copyright (c) 2016-2020, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _hw_pwr_gv11b_h_
#define _hw_pwr_gv11b_h_

static inline u32 pwr_falcon_irqsset_r(void)
{
	return 0x0010a000U;
}
static inline u32 pwr_falcon_irqsset_swgen0_set_f(void)
{
	return 0x40U;
}
static inline u32 pwr_falcon_irqsclr_r(void)
{
	return 0x0010a004U;
}
static inline u32 pwr_falcon_irqstat_r(void)
{
	return 0x0010a008U;
}
static inline u32 pwr_falcon_irqstat_halt_true_f(void)
{
	return 0x10U;
}
static inline u32 pwr_falcon_irqstat_exterr_true_f(void)
{
	return 0x20U;
}
static inline u32 pwr_falcon_irqstat_swgen0_true_f(void)
{
	return 0x40U;
}
static inline u32 pwr_falcon_irqstat_ext_second_true_f(void)
{
	return 0x800U;
}
static inline u32 pwr_falcon_irqstat_ext_ecc_parity_true_f(void)
{
	return 0x400U;
}
static inline u32 pwr_pmu_ecc_intr_status_r(void)
{
	return 0x0010abfcU;
}
static inline u32 pwr_pmu_ecc_intr_status_corrected_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 pwr_pmu_ecc_intr_status_corrected_m(void)
{
	return 0x1U << 0U;
}
static inline u32 pwr_pmu_ecc_intr_status_uncorrected_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 pwr_pmu_ecc_intr_status_uncorrected_m(void)
{
	return 0x1U << 1U;
}
static inline u32 pwr_falcon_irqmode_r(void)
{
	return 0x0010a00cU;
}
static inline u32 pwr_falcon_irqmset_r(void)
{
	return 0x0010a010U;
}
static inline u32 pwr_falcon_irqmset_gptmr_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 pwr_falcon_irqmset_wdtmr_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 pwr_falcon_irqmset_mthd_f(u32 v)
{
	return (v & 0x1U) << 2U;
}
static inline u32 pwr_falcon_irqmset_ctxsw_f(u32 v)
{
	return (v & 0x1U) << 3U;
}
static inline u32 pwr_falcon_irqmset_halt_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 pwr_falcon_irqmset_exterr_f(u32 v)
{
	return (v & 0x1U) << 5U;
}
static inline u32 pwr_falcon_irqmset_swgen0_f(u32 v)
{
	return (v & 0x1U) << 6U;
}
static inline u32 pwr_falcon_irqmset_swgen1_f(u32 v)
{
	return (v & 0x1U) << 7U;
}
static inline u32 pwr_falcon_irqmset_ext_f(u32 v)
{
	return (v & 0xffU) << 8U;
}
static inline u32 pwr_falcon_irqmset_ext_ctxe_f(u32 v)
{
	return (v & 0x1U) << 8U;
}
static inline u32 pwr_falcon_irqmset_ext_limitv_f(u32 v)
{
	return (v & 0x1U) << 9U;
}
static inline u32 pwr_falcon_irqmset_ext_second_f(u32 v)
{
	return (v & 0x1U) << 11U;
}
static inline u32 pwr_falcon_irqmset_ext_therm_f(u32 v)
{
	return (v & 0x1U) << 12U;
}
static inline u32 pwr_falcon_irqmset_ext_miscio_f(u32 v)
{
	return (v & 0x1U) << 13U;
}
static inline u32 pwr_falcon_irqmset_ext_rttimer_f(u32 v)
{
	return (v & 0x1U) << 14U;
}
static inline u32 pwr_falcon_irqmset_ext_rsvd8_f(u32 v)
{
	return (v & 0x1U) << 15U;
}
static inline u32 pwr_falcon_irqmset_ext_ecc_parity_f(u32 v)
{
	return (v & 0x1U) << 10U;
}
static inline u32 pwr_falcon_irqmclr_r(void)
{
	return 0x0010a014U;
}
static inline u32 pwr_falcon_irqmclr_gptmr_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 pwr_falcon_irqmclr_wdtmr_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 pwr_falcon_irqmclr_mthd_f(u32 v)
{
	return (v & 0x1U) << 2U;
}
static inline u32 pwr_falcon_irqmclr_ctxsw_f(u32 v)
{
	return (v & 0x1U) << 3U;
}
static inline u32 pwr_falcon_irqmclr_halt_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 pwr_falcon_irqmclr_exterr_f(u32 v)
{
	return (v & 0x1U) << 5U;
}
static inline u32 pwr_falcon_irqmclr_swgen0_f(u32 v)
{
	return (v & 0x1U) << 6U;
}
static inline u32 pwr_falcon_irqmclr_swgen1_f(u32 v)
{
	return (v & 0x1U) << 7U;
}
static inline u32 pwr_falcon_irqmclr_ext_f(u32 v)
{
	return (v & 0xffU) << 8U;
}
static inline u32 pwr_falcon_irqmclr_ext_ctxe_f(u32 v)
{
	return (v & 0x1U) << 8U;
}
static inline u32 pwr_falcon_irqmclr_ext_limitv_f(u32 v)
{
	return (v & 0x1U) << 9U;
}
static inline u32 pwr_falcon_irqmclr_ext_second_f(u32 v)
{
	return (v & 0x1U) << 11U;
}
static inline u32 pwr_falcon_irqmclr_ext_therm_f(u32 v)
{
	return (v & 0x1U) << 12U;
}
static inline u32 pwr_falcon_irqmclr_ext_miscio_f(u32 v)
{
	return (v & 0x1U) << 13U;
}
static inline u32 pwr_falcon_irqmclr_ext_rttimer_f(u32 v)
{
	return (v & 0x1U) << 14U;
}
static inline u32 pwr_falcon_irqmclr_ext_rsvd8_f(u32 v)
{
	return (v & 0x1U) << 15U;
}
static inline u32 pwr_falcon_irqmclr_ext_ecc_parity_f(u32 v)
{
	return (v & 0x1U) << 10U;
}
static inline u32 pwr_falcon_irqmask_r(void)
{
	return 0x0010a018U;
}
static inline u32 pwr_falcon_irqdest_r(void)
{
	return 0x0010a01cU;
}
static inline u32 pwr_falcon_irqdest_host_gptmr_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 pwr_falcon_irqdest_host_wdtmr_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 pwr_falcon_irqdest_host_mthd_f(u32 v)
{
	return (v & 0x1U) << 2U;
}
static inline u32 pwr_falcon_irqdest_host_ctxsw_f(u32 v)
{
	return (v & 0x1U) << 3U;
}
static inline u32 pwr_falcon_irqdest_host_halt_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 pwr_falcon_irqdest_host_exterr_f(u32 v)
{
	return (v & 0x1U) << 5U;
}
static inline u32 pwr_falcon_irqdest_host_swgen0_f(u32 v)
{
	return (v & 0x1U) << 6U;
}
static inline u32 pwr_falcon_irqdest_host_swgen1_f(u32 v)
{
	return (v & 0x1U) << 7U;
}
static inline u32 pwr_falcon_irqdest_host_ext_f(u32 v)
{
	return (v & 0xffU) << 8U;
}
static inline u32 pwr_falcon_irqdest_host_ext_ctxe_f(u32 v)
{
	return (v & 0x1U) << 8U;
}
static inline u32 pwr_falcon_irqdest_host_ext_limitv_f(u32 v)
{
	return (v & 0x1U) << 9U;
}
static inline u32 pwr_falcon_irqdest_host_ext_second_f(u32 v)
{
	return (v & 0x1U) << 11U;
}
static inline u32 pwr_falcon_irqdest_host_ext_therm_f(u32 v)
{
	return (v & 0x1U) << 12U;
}
static inline u32 pwr_falcon_irqdest_host_ext_miscio_f(u32 v)
{
	return (v & 0x1U) << 13U;
}
static inline u32 pwr_falcon_irqdest_host_ext_rttimer_f(u32 v)
{
	return (v & 0x1U) << 14U;
}
static inline u32 pwr_falcon_irqdest_host_ext_rsvd8_f(u32 v)
{
	return (v & 0x1U) << 15U;
}
static inline u32 pwr_falcon_irqdest_host_ext_ecc_parity_f(u32 v)
{
	return (v & 0x1U) << 10U;
}
static inline u32 pwr_falcon_irqdest_target_gptmr_f(u32 v)
{
	return (v & 0x1U) << 16U;
}
static inline u32 pwr_falcon_irqdest_target_wdtmr_f(u32 v)
{
	return (v & 0x1U) << 17U;
}
static inline u32 pwr_falcon_irqdest_target_mthd_f(u32 v)
{
	return (v & 0x1U) << 18U;
}
static inline u32 pwr_falcon_irqdest_target_ctxsw_f(u32 v)
{
	return (v & 0x1U) << 19U;
}
static inline u32 pwr_falcon_irqdest_target_halt_f(u32 v)
{
	return (v & 0x1U) << 20U;
}
static inline u32 pwr_falcon_irqdest_target_exterr_f(u32 v)
{
	return (v & 0x1U) << 21U;
}
static inline u32 pwr_falcon_irqdest_target_swgen0_f(u32 v)
{
	return (v & 0x1U) << 22U;
}
static inline u32 pwr_falcon_irqdest_target_swgen1_f(u32 v)
{
	return (v & 0x1U) << 23U;
}
static inline u32 pwr_falcon_irqdest_target_ext_f(u32 v)
{
	return (v & 0xffU) << 24U;
}
static inline u32 pwr_falcon_irqdest_target_ext_ctxe_f(u32 v)
{
	return (v & 0x1U) << 24U;
}
static inline u32 pwr_falcon_irqdest_target_ext_limitv_f(u32 v)
{
	return (v & 0x1U) << 25U;
}
static inline u32 pwr_falcon_irqdest_target_ext_second_f(u32 v)
{
	return (v & 0x1U) << 27U;
}
static inline u32 pwr_falcon_irqdest_target_ext_therm_f(u32 v)
{
	return (v & 0x1U) << 28U;
}
static inline u32 pwr_falcon_irqdest_target_ext_miscio_f(u32 v)
{
	return (v & 0x1U) << 29U;
}
static inline u32 pwr_falcon_irqdest_target_ext_rttimer_f(u32 v)
{
	return (v & 0x1U) << 30U;
}
static inline u32 pwr_falcon_irqdest_target_ext_rsvd8_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 pwr_falcon_irqdest_target_ext_ecc_parity_f(u32 v)
{
	return (v & 0x1U) << 26U;
}
static inline u32 pwr_falcon_curctx_r(void)
{
	return 0x0010a050U;
}
static inline u32 pwr_falcon_nxtctx_r(void)
{
	return 0x0010a054U;
}
static inline u32 pwr_falcon_mailbox0_r(void)
{
	return 0x0010a040U;
}
static inline u32 pwr_falcon_mailbox1_r(void)
{
	return 0x0010a044U;
}
static inline u32 pwr_falcon_itfen_r(void)
{
	return 0x0010a048U;
}
static inline u32 pwr_falcon_itfen_ctxen_enable_f(void)
{
	return 0x1U;
}
static inline u32 pwr_falcon_idlestate_r(void)
{
	return 0x0010a04cU;
}
static inline u32 pwr_falcon_idlestate_falcon_busy_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 pwr_falcon_idlestate_ext_busy_v(u32 r)
{
	return (r >> 1U) & 0x7fffU;
}
static inline u32 pwr_falcon_os_r(void)
{
	return 0x0010a080U;
}
static inline u32 pwr_falcon_engctl_r(void)
{
	return 0x0010a0a4U;
}
static inline u32 pwr_falcon_cpuctl_r(void)
{
	return 0x0010a100U;
}
static inline u32 pwr_falcon_cpuctl_startcpu_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 pwr_falcon_cpuctl_halt_intr_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 pwr_falcon_cpuctl_halt_intr_m(void)
{
	return 0x1U << 4U;
}
static inline u32 pwr_falcon_cpuctl_halt_intr_v(u32 r)
{
	return (r >> 4U) & 0x1U;
}
static inline u32 pwr_falcon_cpuctl_cpuctl_alias_en_f(u32 v)
{
	return (v & 0x1U) << 6U;
}
static inline u32 pwr_falcon_cpuctl_cpuctl_alias_en_m(void)
{
	return 0x1U << 6U;
}
static inline u32 pwr_falcon_cpuctl_cpuctl_alias_en_v(u32 r)
{
	return (r >> 6U) & 0x1U;
}
static inline u32 pwr_falcon_cpuctl_alias_r(void)
{
	return 0x0010a130U;
}
static inline u32 pwr_falcon_cpuctl_alias_startcpu_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 pwr_pmu_scpctl_stat_r(void)
{
	return 0x0010ac08U;
}
static inline u32 pwr_pmu_scpctl_stat_debug_mode_f(u32 v)
{
	return (v & 0x1U) << 20U;
}
static inline u32 pwr_pmu_scpctl_stat_debug_mode_m(void)
{
	return 0x1U << 20U;
}
static inline u32 pwr_pmu_scpctl_stat_debug_mode_v(u32 r)
{
	return (r >> 20U) & 0x1U;
}
static inline u32 pwr_falcon_imemc_r(u32 i)
{
	return 0x0010a180U + i*16U;
}
static inline u32 pwr_falcon_imemc_offs_f(u32 v)
{
	return (v & 0x3fU) << 2U;
}
static inline u32 pwr_falcon_imemc_blk_f(u32 v)
{
	return (v & 0xffU) << 8U;
}
static inline u32 pwr_falcon_imemc_aincw_f(u32 v)
{
	return (v & 0x1U) << 24U;
}
static inline u32 pwr_falcon_imemd_r(u32 i)
{
	return 0x0010a184U + i*16U;
}
static inline u32 pwr_falcon_imemt_r(u32 i)
{
	return 0x0010a188U + i*16U;
}
static inline u32 pwr_falcon_sctl_r(void)
{
	return 0x0010a240U;
}
static inline u32 pwr_falcon_mmu_phys_sec_r(void)
{
	return 0x00100ce4U;
}
static inline u32 pwr_falcon_bootvec_r(void)
{
	return 0x0010a104U;
}
static inline u32 pwr_falcon_bootvec_vec_f(u32 v)
{
	return (v & 0xffffffffU) << 0U;
}
static inline u32 pwr_falcon_dmactl_r(void)
{
	return 0x0010a10cU;
}
static inline u32 pwr_falcon_dmactl_dmem_scrubbing_m(void)
{
	return 0x1U << 1U;
}
static inline u32 pwr_falcon_dmactl_imem_scrubbing_m(void)
{
	return 0x1U << 2U;
}
static inline u32 pwr_falcon_hwcfg_r(void)
{
	return 0x0010a108U;
}
static inline u32 pwr_falcon_hwcfg_imem_size_v(u32 r)
{
	return (r >> 0U) & 0x1ffU;
}
static inline u32 pwr_falcon_hwcfg_dmem_size_v(u32 r)
{
	return (r >> 9U) & 0x1ffU;
}
static inline u32 pwr_falcon_dmatrfbase_r(void)
{
	return 0x0010a110U;
}
static inline u32 pwr_falcon_dmatrfbase1_r(void)
{
	return 0x0010a128U;
}
static inline u32 pwr_falcon_dmatrfmoffs_r(void)
{
	return 0x0010a114U;
}
static inline u32 pwr_falcon_dmatrfcmd_r(void)
{
	return 0x0010a118U;
}
static inline u32 pwr_falcon_dmatrfcmd_imem_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 pwr_falcon_dmatrfcmd_write_f(u32 v)
{
	return (v & 0x1U) << 5U;
}
static inline u32 pwr_falcon_dmatrfcmd_size_f(u32 v)
{
	return (v & 0x7U) << 8U;
}
static inline u32 pwr_falcon_dmatrfcmd_ctxdma_f(u32 v)
{
	return (v & 0x7U) << 12U;
}
static inline u32 pwr_falcon_dmatrffboffs_r(void)
{
	return 0x0010a11cU;
}
static inline u32 pwr_falcon_exterraddr_r(void)
{
	return 0x0010a168U;
}
static inline u32 pwr_falcon_exterrstat_r(void)
{
	return 0x0010a16cU;
}
static inline u32 pwr_falcon_exterrstat_valid_m(void)
{
	return 0x1U << 31U;
}
static inline u32 pwr_falcon_exterrstat_valid_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 pwr_falcon_exterrstat_valid_true_v(void)
{
	return 0x00000001U;
}
static inline u32 pwr_pmu_falcon_icd_cmd_r(void)
{
	return 0x0010a200U;
}
static inline u32 pwr_pmu_falcon_icd_cmd_opc_s(void)
{
	return 4U;
}
static inline u32 pwr_pmu_falcon_icd_cmd_opc_f(u32 v)
{
	return (v & 0xfU) << 0U;
}
static inline u32 pwr_pmu_falcon_icd_cmd_opc_m(void)
{
	return 0xfU << 0U;
}
static inline u32 pwr_pmu_falcon_icd_cmd_opc_v(u32 r)
{
	return (r >> 0U) & 0xfU;
}
static inline u32 pwr_pmu_falcon_icd_cmd_opc_rreg_f(void)
{
	return 0x8U;
}
static inline u32 pwr_pmu_falcon_icd_cmd_opc_rstat_f(void)
{
	return 0xeU;
}
static inline u32 pwr_pmu_falcon_icd_cmd_idx_f(u32 v)
{
	return (v & 0x1fU) << 8U;
}
static inline u32 pwr_pmu_falcon_icd_rdata_r(void)
{
	return 0x0010a20cU;
}
static inline u32 pwr_falcon_dmemc_r(u32 i)
{
	return 0x0010a1c0U + i*8U;
}
static inline u32 pwr_falcon_dmemc_offs_f(u32 v)
{
	return (v & 0x3fU) << 2U;
}
static inline u32 pwr_falcon_dmemc_offs_m(void)
{
	return 0x3fU << 2U;
}
static inline u32 pwr_falcon_dmemc_blk_f(u32 v)
{
	return (v & 0xffU) << 8U;
}
static inline u32 pwr_falcon_dmemc_blk_m(void)
{
	return 0xffU << 8U;
}
static inline u32 pwr_falcon_dmemc_aincw_f(u32 v)
{
	return (v & 0x1U) << 24U;
}
static inline u32 pwr_falcon_dmemc_aincr_f(u32 v)
{
	return (v & 0x1U) << 25U;
}
static inline u32 pwr_falcon_dmemd_r(u32 i)
{
	return 0x0010a1c4U + i*8U;
}
static inline u32 pwr_pmu_new_instblk_r(void)
{
	return 0x0010a480U;
}
static inline u32 pwr_pmu_new_instblk_ptr_f(u32 v)
{
	return (v & 0xfffffffU) << 0U;
}
static inline u32 pwr_pmu_new_instblk_target_fb_f(void)
{
	return 0x0U;
}
static inline u32 pwr_pmu_new_instblk_target_sys_coh_f(void)
{
	return 0x20000000U;
}
static inline u32 pwr_pmu_new_instblk_target_sys_ncoh_f(void)
{
	return 0x30000000U;
}
static inline u32 pwr_pmu_new_instblk_valid_f(u32 v)
{
	return (v & 0x1U) << 30U;
}
static inline u32 pwr_pmu_mutex_id_r(void)
{
	return 0x0010a488U;
}
static inline u32 pwr_pmu_mutex_id_value_v(u32 r)
{
	return (r >> 0U) & 0xffU;
}
static inline u32 pwr_pmu_mutex_id_value_init_v(void)
{
	return 0x00000000U;
}
static inline u32 pwr_pmu_mutex_id_value_not_avail_v(void)
{
	return 0x000000ffU;
}
static inline u32 pwr_pmu_mutex_id_release_r(void)
{
	return 0x0010a48cU;
}
static inline u32 pwr_pmu_mutex_id_release_value_f(u32 v)
{
	return (v & 0xffU) << 0U;
}
static inline u32 pwr_pmu_mutex_id_release_value_m(void)
{
	return 0xffU << 0U;
}
static inline u32 pwr_pmu_mutex_id_release_value_init_v(void)
{
	return 0x00000000U;
}
static inline u32 pwr_pmu_mutex_id_release_value_init_f(void)
{
	return 0x0U;
}
static inline u32 pwr_pmu_mutex_r(u32 i)
{
	return 0x0010a580U + i*4U;
}
static inline u32 pwr_pmu_mutex__size_1_v(void)
{
	return 0x00000010U;
}
static inline u32 pwr_pmu_mutex_value_f(u32 v)
{
	return (v & 0xffU) << 0U;
}
static inline u32 pwr_pmu_mutex_value_v(u32 r)
{
	return (r >> 0U) & 0xffU;
}
static inline u32 pwr_pmu_mutex_value_initial_lock_f(void)
{
	return 0x0U;
}
static inline u32 pwr_pmu_queue_head_r(u32 i)
{
	return 0x0010a800U + i*4U;
}
static inline u32 pwr_pmu_queue_head__size_1_v(void)
{
	return 0x00000008U;
}
static inline u32 pwr_pmu_queue_head_address_f(u32 v)
{
	return (v & 0xffffffffU) << 0U;
}
static inline u32 pwr_pmu_queue_head_address_v(u32 r)
{
	return (r >> 0U) & 0xffffffffU;
}
static inline u32 pwr_pmu_queue_tail_r(u32 i)
{
	return 0x0010a820U + i*4U;
}
static inline u32 pwr_pmu_queue_tail__size_1_v(void)
{
	return 0x00000008U;
}
static inline u32 pwr_pmu_queue_tail_address_f(u32 v)
{
	return (v & 0xffffffffU) << 0U;
}
static inline u32 pwr_pmu_queue_tail_address_v(u32 r)
{
	return (r >> 0U) & 0xffffffffU;
}
static inline u32 pwr_pmu_msgq_head_r(void)
{
	return 0x0010a4c8U;
}
static inline u32 pwr_pmu_msgq_head_val_f(u32 v)
{
	return (v & 0xffffffffU) << 0U;
}
static inline u32 pwr_pmu_msgq_head_val_v(u32 r)
{
	return (r >> 0U) & 0xffffffffU;
}
static inline u32 pwr_pmu_msgq_tail_r(void)
{
	return 0x0010a4ccU;
}
static inline u32 pwr_pmu_msgq_tail_val_f(u32 v)
{
	return (v & 0xffffffffU) << 0U;
}
static inline u32 pwr_pmu_msgq_tail_val_v(u32 r)
{
	return (r >> 0U) & 0xffffffffU;
}
static inline u32 pwr_pmu_idle_mask_r(u32 i)
{
	return 0x0010a504U + i*16U;
}
static inline u32 pwr_pmu_idle_mask_gr_enabled_f(void)
{
	return 0x1U;
}
static inline u32 pwr_pmu_idle_mask_ce_2_enabled_f(void)
{
	return 0x200000U;
}
static inline u32 pwr_pmu_idle_mask_1_r(u32 i)
{
	return 0x0010aa34U + i*8U;
}
static inline u32 pwr_pmu_idle_mask_2_r(u32 i)
{
	return 0x0010a840U + i*4U;
}
static inline u32 pwr_pmu_idle_count_r(u32 i)
{
	return 0x0010a508U + i*16U;
}
static inline u32 pwr_pmu_idle_count_value_f(u32 v)
{
	return (v & 0x7fffffffU) << 0U;
}
static inline u32 pwr_pmu_idle_count_value_v(u32 r)
{
	return (r >> 0U) & 0x7fffffffU;
}
static inline u32 pwr_pmu_idle_count_reset_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 pwr_pmu_idle_ctrl_r(u32 i)
{
	return 0x0010a50cU + i*16U;
}
static inline u32 pwr_pmu_idle_ctrl_value_m(void)
{
	return 0x3U << 0U;
}
static inline u32 pwr_pmu_idle_ctrl_value_busy_f(void)
{
	return 0x2U;
}
static inline u32 pwr_pmu_idle_ctrl_value_always_f(void)
{
	return 0x3U;
}
static inline u32 pwr_pmu_idle_ctrl_filter_m(void)
{
	return 0x1U << 2U;
}
static inline u32 pwr_pmu_idle_ctrl_filter_disabled_f(void)
{
	return 0x0U;
}
static inline u32 pwr_pmu_idle_threshold_r(u32 i)
{
	return 0x0010a8a0U + i*4U;
}
static inline u32 pwr_pmu_idle_threshold_value_f(u32 v)
{
	return (v & 0x7fffffffU) << 0U;
}
static inline u32 pwr_pmu_idle_intr_r(void)
{
	return 0x0010a9e8U;
}
static inline u32 pwr_pmu_idle_intr_en_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 pwr_pmu_idle_intr_en_disabled_v(void)
{
	return 0x00000000U;
}
static inline u32 pwr_pmu_idle_intr_en_enabled_v(void)
{
	return 0x00000001U;
}
static inline u32 pwr_pmu_idle_intr_status_r(void)
{
	return 0x0010a9ecU;
}
static inline u32 pwr_pmu_idle_intr_status_intr_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 pwr_pmu_idle_intr_status_intr_m(void)
{
	return 0x1U << 0U;
}
static inline u32 pwr_pmu_idle_intr_status_intr_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 pwr_pmu_idle_intr_status_intr_pending_v(void)
{
	return 0x00000001U;
}
static inline u32 pwr_pmu_idle_intr_status_intr_clear_v(void)
{
	return 0x00000001U;
}
static inline u32 pwr_pmu_idle_mask_supp_r(u32 i)
{
	return 0x0010a9f0U + i*8U;
}
static inline u32 pwr_pmu_idle_mask_1_supp_r(u32 i)
{
	return 0x0010a9f4U + i*8U;
}
static inline u32 pwr_pmu_idle_mask_2_supp_r(u32 i)
{
	return 0x0010a690U + i*4U;
}
static inline u32 pwr_pmu_idle_ctrl_supp_r(u32 i)
{
	return 0x0010aa30U + i*8U;
}
static inline u32 pwr_pmu_debug_r(u32 i)
{
	return 0x0010a5c0U + i*4U;
}
static inline u32 pwr_pmu_debug__size_1_v(void)
{
	return 0x00000004U;
}
static inline u32 pwr_pmu_mailbox_r(u32 i)
{
	return 0x0010a450U + i*4U;
}
static inline u32 pwr_pmu_mailbox__size_1_v(void)
{
	return 0x0000000cU;
}
static inline u32 pwr_pmu_bar0_addr_r(void)
{
	return 0x0010a7a0U;
}
static inline u32 pwr_pmu_bar0_data_r(void)
{
	return 0x0010a7a4U;
}
static inline u32 pwr_pmu_bar0_ctl_r(void)
{
	return 0x0010a7acU;
}
static inline u32 pwr_pmu_bar0_timeout_r(void)
{
	return 0x0010a7a8U;
}
static inline u32 pwr_pmu_bar0_fecs_error_r(void)
{
	return 0x0010a988U;
}
static inline u32 pwr_pmu_bar0_error_status_r(void)
{
	return 0x0010a7b0U;
}
static inline u32 pwr_pmu_pg_idlefilth_r(u32 i)
{
	return 0x0010a6c0U + i*4U;
}
static inline u32 pwr_pmu_pg_ppuidlefilth_r(u32 i)
{
	return 0x0010a6e8U + i*4U;
}
static inline u32 pwr_pmu_pg_idle_cnt_r(u32 i)
{
	return 0x0010a710U + i*4U;
}
static inline u32 pwr_pmu_pg_intren_r(u32 i)
{
	return 0x0010a760U + i*4U;
}
static inline u32 pwr_pmu_falcon_ecc_status_r(void)
{
	return 0x0010a6b0U;
}
static inline u32 pwr_pmu_falcon_ecc_status_corrected_err_imem_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 pwr_pmu_falcon_ecc_status_corrected_err_imem_m(void)
{
	return 0x1U << 0U;
}
static inline u32 pwr_pmu_falcon_ecc_status_corrected_err_dmem_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 pwr_pmu_falcon_ecc_status_corrected_err_dmem_m(void)
{
	return 0x1U << 1U;
}
static inline u32 pwr_pmu_falcon_ecc_status_uncorrected_err_imem_f(u32 v)
{
	return (v & 0x1U) << 8U;
}
static inline u32 pwr_pmu_falcon_ecc_status_uncorrected_err_imem_m(void)
{
	return 0x1U << 8U;
}
static inline u32 pwr_pmu_falcon_ecc_status_uncorrected_err_dmem_f(u32 v)
{
	return (v & 0x1U) << 9U;
}
static inline u32 pwr_pmu_falcon_ecc_status_uncorrected_err_dmem_m(void)
{
	return 0x1U << 9U;
}
static inline u32 pwr_pmu_falcon_ecc_status_corrected_err_total_counter_overflow_f(u32 v)
{
	return (v & 0x1U) << 16U;
}
static inline u32 pwr_pmu_falcon_ecc_status_corrected_err_total_counter_overflow_m(void)
{
	return 0x1U << 16U;
}
static inline u32 pwr_pmu_falcon_ecc_status_uncorrected_err_total_counter_overflow_f(u32 v)
{
	return (v & 0x1U) << 18U;
}
static inline u32 pwr_pmu_falcon_ecc_status_uncorrected_err_total_counter_overflow_m(void)
{
	return 0x1U << 18U;
}
static inline u32 pwr_pmu_falcon_ecc_status_reset_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 pwr_pmu_falcon_ecc_status_reset_task_f(void)
{
	return 0x80000000U;
}
static inline u32 pwr_pmu_falcon_ecc_address_r(void)
{
	return 0x0010a6b4U;
}
static inline u32 pwr_pmu_falcon_ecc_address_index_f(u32 v)
{
	return (v & 0xffffffU) << 0U;
}
static inline u32 pwr_pmu_falcon_ecc_address_type_f(u32 v)
{
	return (v & 0xfU) << 20U;
}
static inline u32 pwr_pmu_falcon_ecc_address_type_imem_f(void)
{
	return 0x0U;
}
static inline u32 pwr_pmu_falcon_ecc_address_type_dmem_f(void)
{
	return 0x100000U;
}
static inline u32 pwr_pmu_falcon_ecc_address_row_address_s(void)
{
	return 16U;
}
static inline u32 pwr_pmu_falcon_ecc_address_row_address_f(u32 v)
{
	return (v & 0xffffU) << 0U;
}
static inline u32 pwr_pmu_falcon_ecc_address_row_address_m(void)
{
	return 0xffffU << 0U;
}
static inline u32 pwr_pmu_falcon_ecc_address_row_address_v(u32 r)
{
	return (r >> 0U) & 0xffffU;
}
static inline u32 pwr_pmu_falcon_ecc_corrected_err_count_r(void)
{
	return 0x0010a6b8U;
}
static inline u32 pwr_pmu_falcon_ecc_corrected_err_count_total_s(void)
{
	return 16U;
}
static inline u32 pwr_pmu_falcon_ecc_corrected_err_count_total_f(u32 v)
{
	return (v & 0xffffU) << 0U;
}
static inline u32 pwr_pmu_falcon_ecc_corrected_err_count_total_m(void)
{
	return 0xffffU << 0U;
}
static inline u32 pwr_pmu_falcon_ecc_corrected_err_count_total_v(u32 r)
{
	return (r >> 0U) & 0xffffU;
}
static inline u32 pwr_pmu_falcon_ecc_corrected_err_count_unique_total_s(void)
{
	return 16U;
}
static inline u32 pwr_pmu_falcon_ecc_corrected_err_count_unique_total_f(u32 v)
{
	return (v & 0xffffU) << 16U;
}
static inline u32 pwr_pmu_falcon_ecc_corrected_err_count_unique_total_m(void)
{
	return 0xffffU << 16U;
}
static inline u32 pwr_pmu_falcon_ecc_corrected_err_count_unique_total_v(u32 r)
{
	return (r >> 16U) & 0xffffU;
}
static inline u32 pwr_pmu_falcon_ecc_uncorrected_err_count_r(void)
{
	return 0x0010a6bcU;
}
static inline u32 pwr_pmu_falcon_ecc_uncorrected_err_count_total_s(void)
{
	return 16U;
}
static inline u32 pwr_pmu_falcon_ecc_uncorrected_err_count_total_f(u32 v)
{
	return (v & 0xffffU) << 0U;
}
static inline u32 pwr_pmu_falcon_ecc_uncorrected_err_count_total_m(void)
{
	return 0xffffU << 0U;
}
static inline u32 pwr_pmu_falcon_ecc_uncorrected_err_count_total_v(u32 r)
{
	return (r >> 0U) & 0xffffU;
}
static inline u32 pwr_pmu_falcon_ecc_uncorrected_err_count_unique_total_s(void)
{
	return 16U;
}
static inline u32 pwr_pmu_falcon_ecc_uncorrected_err_count_unique_total_f(u32 v)
{
	return (v & 0xffffU) << 16U;
}
static inline u32 pwr_pmu_falcon_ecc_uncorrected_err_count_unique_total_m(void)
{
	return 0xffffU << 16U;
}
static inline u32 pwr_pmu_falcon_ecc_uncorrected_err_count_unique_total_v(u32 r)
{
	return (r >> 16U) & 0xffffU;
}
static inline u32 pwr_fbif_transcfg_r(u32 i)
{
	return 0x0010ae00U + i*4U;
}
static inline u32 pwr_fbif_transcfg_target_local_fb_f(void)
{
	return 0x0U;
}
static inline u32 pwr_fbif_transcfg_target_coherent_sysmem_f(void)
{
	return 0x1U;
}
static inline u32 pwr_fbif_transcfg_target_noncoherent_sysmem_f(void)
{
	return 0x2U;
}
static inline u32 pwr_fbif_transcfg_mem_type_s(void)
{
	return 1U;
}
static inline u32 pwr_fbif_transcfg_mem_type_f(u32 v)
{
	return (v & 0x1U) << 2U;
}
static inline u32 pwr_fbif_transcfg_mem_type_m(void)
{
	return 0x1U << 2U;
}
static inline u32 pwr_fbif_transcfg_mem_type_v(u32 r)
{
	return (r >> 2U) & 0x1U;
}
static inline u32 pwr_fbif_transcfg_mem_type_virtual_f(void)
{
	return 0x0U;
}
static inline u32 pwr_fbif_transcfg_mem_type_physical_f(void)
{
	return 0x4U;
}
#endif
