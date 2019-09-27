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
#ifndef _hw_minion_gv100_h_
#define _hw_minion_gv100_h_

static inline u32 minion_minion_status_r(void)
{
	return 0x00000830U;
}
static inline u32 minion_minion_status_status_f(u32 v)
{
	return (v & 0xffU) << 0U;
}
static inline u32 minion_minion_status_status_m(void)
{
	return 0xffU << 0U;
}
static inline u32 minion_minion_status_status_v(u32 r)
{
	return (r >> 0U) & 0xffU;
}
static inline u32 minion_minion_status_status_boot_v(void)
{
	return 0x00000001U;
}
static inline u32 minion_minion_status_status_boot_f(void)
{
	return 0x1U;
}
static inline u32 minion_minion_status_intr_code_f(u32 v)
{
	return (v & 0xffffffU) << 8U;
}
static inline u32 minion_minion_status_intr_code_m(void)
{
	return 0xffffffU << 8U;
}
static inline u32 minion_minion_status_intr_code_v(u32 r)
{
	return (r >> 8U) & 0xffffffU;
}
static inline u32 minion_falcon_irqstat_r(void)
{
	return 0x00000008U;
}
static inline u32 minion_falcon_irqstat_halt_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 minion_falcon_irqstat_halt_v(u32 r)
{
	return (r >> 4U) & 0x1U;
}
static inline u32 minion_falcon_irqstat_exterr_f(u32 v)
{
	return (v & 0x1U) << 5U;
}
static inline u32 minion_falcon_irqstat_exterr_v(u32 r)
{
	return (r >> 5U) & 0x1U;
}
static inline u32 minion_falcon_irqstat_exterr_true_v(void)
{
	return 0x00000001U;
}
static inline u32 minion_falcon_irqstat_exterr_true_f(void)
{
	return 0x20U;
}
static inline u32 minion_falcon_irqmask_r(void)
{
	return 0x00000018U;
}
static inline u32 minion_falcon_irqsclr_r(void)
{
	return 0x00000004U;
}
static inline u32 minion_falcon_irqsset_r(void)
{
	return 0x00000000U;
}
static inline u32 minion_falcon_irqmset_r(void)
{
	return 0x00000010U;
}
static inline u32 minion_falcon_irqmset_wdtmr_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 minion_falcon_irqmset_wdtmr_m(void)
{
	return 0x1U << 1U;
}
static inline u32 minion_falcon_irqmset_wdtmr_v(u32 r)
{
	return (r >> 1U) & 0x1U;
}
static inline u32 minion_falcon_irqmset_wdtmr_set_v(void)
{
	return 0x00000001U;
}
static inline u32 minion_falcon_irqmset_wdtmr_set_f(void)
{
	return 0x2U;
}
static inline u32 minion_falcon_irqmset_halt_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 minion_falcon_irqmset_halt_m(void)
{
	return 0x1U << 4U;
}
static inline u32 minion_falcon_irqmset_halt_v(u32 r)
{
	return (r >> 4U) & 0x1U;
}
static inline u32 minion_falcon_irqmset_halt_set_v(void)
{
	return 0x00000001U;
}
static inline u32 minion_falcon_irqmset_halt_set_f(void)
{
	return 0x10U;
}
static inline u32 minion_falcon_irqmset_exterr_f(u32 v)
{
	return (v & 0x1U) << 5U;
}
static inline u32 minion_falcon_irqmset_exterr_m(void)
{
	return 0x1U << 5U;
}
static inline u32 minion_falcon_irqmset_exterr_v(u32 r)
{
	return (r >> 5U) & 0x1U;
}
static inline u32 minion_falcon_irqmset_exterr_set_v(void)
{
	return 0x00000001U;
}
static inline u32 minion_falcon_irqmset_exterr_set_f(void)
{
	return 0x20U;
}
static inline u32 minion_falcon_irqmset_swgen0_f(u32 v)
{
	return (v & 0x1U) << 6U;
}
static inline u32 minion_falcon_irqmset_swgen0_m(void)
{
	return 0x1U << 6U;
}
static inline u32 minion_falcon_irqmset_swgen0_v(u32 r)
{
	return (r >> 6U) & 0x1U;
}
static inline u32 minion_falcon_irqmset_swgen0_set_v(void)
{
	return 0x00000001U;
}
static inline u32 minion_falcon_irqmset_swgen0_set_f(void)
{
	return 0x40U;
}
static inline u32 minion_falcon_irqmset_swgen1_f(u32 v)
{
	return (v & 0x1U) << 7U;
}
static inline u32 minion_falcon_irqmset_swgen1_m(void)
{
	return 0x1U << 7U;
}
static inline u32 minion_falcon_irqmset_swgen1_v(u32 r)
{
	return (r >> 7U) & 0x1U;
}
static inline u32 minion_falcon_irqmset_swgen1_set_v(void)
{
	return 0x00000001U;
}
static inline u32 minion_falcon_irqmset_swgen1_set_f(void)
{
	return 0x80U;
}
static inline u32 minion_falcon_irqdest_r(void)
{
	return 0x0000001cU;
}
static inline u32 minion_falcon_irqdest_host_wdtmr_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 minion_falcon_irqdest_host_wdtmr_m(void)
{
	return 0x1U << 1U;
}
static inline u32 minion_falcon_irqdest_host_wdtmr_v(u32 r)
{
	return (r >> 1U) & 0x1U;
}
static inline u32 minion_falcon_irqdest_host_wdtmr_host_v(void)
{
	return 0x00000001U;
}
static inline u32 minion_falcon_irqdest_host_wdtmr_host_f(void)
{
	return 0x2U;
}
static inline u32 minion_falcon_irqdest_host_halt_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 minion_falcon_irqdest_host_halt_m(void)
{
	return 0x1U << 4U;
}
static inline u32 minion_falcon_irqdest_host_halt_v(u32 r)
{
	return (r >> 4U) & 0x1U;
}
static inline u32 minion_falcon_irqdest_host_halt_host_v(void)
{
	return 0x00000001U;
}
static inline u32 minion_falcon_irqdest_host_halt_host_f(void)
{
	return 0x10U;
}
static inline u32 minion_falcon_irqdest_host_exterr_f(u32 v)
{
	return (v & 0x1U) << 5U;
}
static inline u32 minion_falcon_irqdest_host_exterr_m(void)
{
	return 0x1U << 5U;
}
static inline u32 minion_falcon_irqdest_host_exterr_v(u32 r)
{
	return (r >> 5U) & 0x1U;
}
static inline u32 minion_falcon_irqdest_host_exterr_host_v(void)
{
	return 0x00000001U;
}
static inline u32 minion_falcon_irqdest_host_exterr_host_f(void)
{
	return 0x20U;
}
static inline u32 minion_falcon_irqdest_host_swgen0_f(u32 v)
{
	return (v & 0x1U) << 6U;
}
static inline u32 minion_falcon_irqdest_host_swgen0_m(void)
{
	return 0x1U << 6U;
}
static inline u32 minion_falcon_irqdest_host_swgen0_v(u32 r)
{
	return (r >> 6U) & 0x1U;
}
static inline u32 minion_falcon_irqdest_host_swgen0_host_v(void)
{
	return 0x00000001U;
}
static inline u32 minion_falcon_irqdest_host_swgen0_host_f(void)
{
	return 0x40U;
}
static inline u32 minion_falcon_irqdest_host_swgen1_f(u32 v)
{
	return (v & 0x1U) << 7U;
}
static inline u32 minion_falcon_irqdest_host_swgen1_m(void)
{
	return 0x1U << 7U;
}
static inline u32 minion_falcon_irqdest_host_swgen1_v(u32 r)
{
	return (r >> 7U) & 0x1U;
}
static inline u32 minion_falcon_irqdest_host_swgen1_host_v(void)
{
	return 0x00000001U;
}
static inline u32 minion_falcon_irqdest_host_swgen1_host_f(void)
{
	return 0x80U;
}
static inline u32 minion_falcon_irqdest_target_wdtmr_f(u32 v)
{
	return (v & 0x1U) << 17U;
}
static inline u32 minion_falcon_irqdest_target_wdtmr_m(void)
{
	return 0x1U << 17U;
}
static inline u32 minion_falcon_irqdest_target_wdtmr_v(u32 r)
{
	return (r >> 17U) & 0x1U;
}
static inline u32 minion_falcon_irqdest_target_wdtmr_host_normal_v(void)
{
	return 0x00000000U;
}
static inline u32 minion_falcon_irqdest_target_wdtmr_host_normal_f(void)
{
	return 0x0U;
}
static inline u32 minion_falcon_irqdest_target_halt_f(u32 v)
{
	return (v & 0x1U) << 20U;
}
static inline u32 minion_falcon_irqdest_target_halt_m(void)
{
	return 0x1U << 20U;
}
static inline u32 minion_falcon_irqdest_target_halt_v(u32 r)
{
	return (r >> 20U) & 0x1U;
}
static inline u32 minion_falcon_irqdest_target_halt_host_normal_v(void)
{
	return 0x00000000U;
}
static inline u32 minion_falcon_irqdest_target_halt_host_normal_f(void)
{
	return 0x0U;
}
static inline u32 minion_falcon_irqdest_target_exterr_f(u32 v)
{
	return (v & 0x1U) << 21U;
}
static inline u32 minion_falcon_irqdest_target_exterr_m(void)
{
	return 0x1U << 21U;
}
static inline u32 minion_falcon_irqdest_target_exterr_v(u32 r)
{
	return (r >> 21U) & 0x1U;
}
static inline u32 minion_falcon_irqdest_target_exterr_host_normal_v(void)
{
	return 0x00000000U;
}
static inline u32 minion_falcon_irqdest_target_exterr_host_normal_f(void)
{
	return 0x0U;
}
static inline u32 minion_falcon_irqdest_target_swgen0_f(u32 v)
{
	return (v & 0x1U) << 22U;
}
static inline u32 minion_falcon_irqdest_target_swgen0_m(void)
{
	return 0x1U << 22U;
}
static inline u32 minion_falcon_irqdest_target_swgen0_v(u32 r)
{
	return (r >> 22U) & 0x1U;
}
static inline u32 minion_falcon_irqdest_target_swgen0_host_normal_v(void)
{
	return 0x00000000U;
}
static inline u32 minion_falcon_irqdest_target_swgen0_host_normal_f(void)
{
	return 0x0U;
}
static inline u32 minion_falcon_irqdest_target_swgen1_f(u32 v)
{
	return (v & 0x1U) << 23U;
}
static inline u32 minion_falcon_irqdest_target_swgen1_m(void)
{
	return 0x1U << 23U;
}
static inline u32 minion_falcon_irqdest_target_swgen1_v(u32 r)
{
	return (r >> 23U) & 0x1U;
}
static inline u32 minion_falcon_irqdest_target_swgen1_host_normal_v(void)
{
	return 0x00000000U;
}
static inline u32 minion_falcon_irqdest_target_swgen1_host_normal_f(void)
{
	return 0x0U;
}
static inline u32 minion_falcon_os_r(void)
{
	return 0x00000080U;
}
static inline u32 minion_falcon_mailbox1_r(void)
{
	return 0x00000044U;
}
static inline u32 minion_minion_intr_r(void)
{
	return 0x00000810U;
}
static inline u32 minion_minion_intr_fatal_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 minion_minion_intr_fatal_m(void)
{
	return 0x1U << 0U;
}
static inline u32 minion_minion_intr_fatal_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 minion_minion_intr_nonfatal_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 minion_minion_intr_nonfatal_m(void)
{
	return 0x1U << 1U;
}
static inline u32 minion_minion_intr_nonfatal_v(u32 r)
{
	return (r >> 1U) & 0x1U;
}
static inline u32 minion_minion_intr_falcon_stall_f(u32 v)
{
	return (v & 0x1U) << 2U;
}
static inline u32 minion_minion_intr_falcon_stall_m(void)
{
	return 0x1U << 2U;
}
static inline u32 minion_minion_intr_falcon_stall_v(u32 r)
{
	return (r >> 2U) & 0x1U;
}
static inline u32 minion_minion_intr_falcon_nostall_f(u32 v)
{
	return (v & 0x1U) << 3U;
}
static inline u32 minion_minion_intr_falcon_nostall_m(void)
{
	return 0x1U << 3U;
}
static inline u32 minion_minion_intr_falcon_nostall_v(u32 r)
{
	return (r >> 3U) & 0x1U;
}
static inline u32 minion_minion_intr_link_f(u32 v)
{
	return (v & 0xffffU) << 16U;
}
static inline u32 minion_minion_intr_link_m(void)
{
	return 0xffffU << 16U;
}
static inline u32 minion_minion_intr_link_v(u32 r)
{
	return (r >> 16U) & 0xffffU;
}
static inline u32 minion_minion_intr_nonstall_en_r(void)
{
	return 0x0000081cU;
}
static inline u32 minion_minion_intr_stall_en_r(void)
{
	return 0x00000818U;
}
static inline u32 minion_minion_intr_stall_en_fatal_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 minion_minion_intr_stall_en_fatal_m(void)
{
	return 0x1U << 0U;
}
static inline u32 minion_minion_intr_stall_en_fatal_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 minion_minion_intr_stall_en_fatal_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 minion_minion_intr_stall_en_fatal_enable_f(void)
{
	return 0x1U;
}
static inline u32 minion_minion_intr_stall_en_fatal_disable_v(void)
{
	return 0x00000000U;
}
static inline u32 minion_minion_intr_stall_en_fatal_disable_f(void)
{
	return 0x0U;
}
static inline u32 minion_minion_intr_stall_en_nonfatal_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 minion_minion_intr_stall_en_nonfatal_m(void)
{
	return 0x1U << 1U;
}
static inline u32 minion_minion_intr_stall_en_nonfatal_v(u32 r)
{
	return (r >> 1U) & 0x1U;
}
static inline u32 minion_minion_intr_stall_en_nonfatal_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 minion_minion_intr_stall_en_nonfatal_enable_f(void)
{
	return 0x2U;
}
static inline u32 minion_minion_intr_stall_en_nonfatal_disable_v(void)
{
	return 0x00000000U;
}
static inline u32 minion_minion_intr_stall_en_nonfatal_disable_f(void)
{
	return 0x0U;
}
static inline u32 minion_minion_intr_stall_en_falcon_stall_f(u32 v)
{
	return (v & 0x1U) << 2U;
}
static inline u32 minion_minion_intr_stall_en_falcon_stall_m(void)
{
	return 0x1U << 2U;
}
static inline u32 minion_minion_intr_stall_en_falcon_stall_v(u32 r)
{
	return (r >> 2U) & 0x1U;
}
static inline u32 minion_minion_intr_stall_en_falcon_stall_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 minion_minion_intr_stall_en_falcon_stall_enable_f(void)
{
	return 0x4U;
}
static inline u32 minion_minion_intr_stall_en_falcon_stall_disable_v(void)
{
	return 0x00000000U;
}
static inline u32 minion_minion_intr_stall_en_falcon_stall_disable_f(void)
{
	return 0x0U;
}
static inline u32 minion_minion_intr_stall_en_falcon_nostall_f(u32 v)
{
	return (v & 0x1U) << 3U;
}
static inline u32 minion_minion_intr_stall_en_falcon_nostall_m(void)
{
	return 0x1U << 3U;
}
static inline u32 minion_minion_intr_stall_en_falcon_nostall_v(u32 r)
{
	return (r >> 3U) & 0x1U;
}
static inline u32 minion_minion_intr_stall_en_falcon_nostall_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 minion_minion_intr_stall_en_falcon_nostall_enable_f(void)
{
	return 0x8U;
}
static inline u32 minion_minion_intr_stall_en_falcon_nostall_disable_v(void)
{
	return 0x00000000U;
}
static inline u32 minion_minion_intr_stall_en_falcon_nostall_disable_f(void)
{
	return 0x0U;
}
static inline u32 minion_minion_intr_stall_en_link_f(u32 v)
{
	return (v & 0xffffU) << 16U;
}
static inline u32 minion_minion_intr_stall_en_link_m(void)
{
	return 0xffffU << 16U;
}
static inline u32 minion_minion_intr_stall_en_link_v(u32 r)
{
	return (r >> 16U) & 0xffffU;
}
static inline u32 minion_nvlink_dl_cmd_r(u32 i)
{
	return 0x00000900U + i*4U;
}
static inline u32 minion_nvlink_dl_cmd___size_1_v(void)
{
	return 0x00000006U;
}
static inline u32 minion_nvlink_dl_cmd_command_f(u32 v)
{
	return (v & 0xffU) << 0U;
}
static inline u32 minion_nvlink_dl_cmd_command_v(u32 r)
{
	return (r >> 0U) & 0xffU;
}
static inline u32 minion_nvlink_dl_cmd_command_configeom_v(void)
{
	return 0x00000040U;
}
static inline u32 minion_nvlink_dl_cmd_command_configeom_f(void)
{
	return 0x40U;
}
static inline u32 minion_nvlink_dl_cmd_command_nop_v(void)
{
	return 0x00000000U;
}
static inline u32 minion_nvlink_dl_cmd_command_nop_f(void)
{
	return 0x0U;
}
static inline u32 minion_nvlink_dl_cmd_command_initphy_v(void)
{
	return 0x00000001U;
}
static inline u32 minion_nvlink_dl_cmd_command_initphy_f(void)
{
	return 0x1U;
}
static inline u32 minion_nvlink_dl_cmd_command_initlaneenable_v(void)
{
	return 0x00000003U;
}
static inline u32 minion_nvlink_dl_cmd_command_initlaneenable_f(void)
{
	return 0x3U;
}
static inline u32 minion_nvlink_dl_cmd_command_initdlpl_v(void)
{
	return 0x00000004U;
}
static inline u32 minion_nvlink_dl_cmd_command_initdlpl_f(void)
{
	return 0x4U;
}
static inline u32 minion_nvlink_dl_cmd_command_lanedisable_v(void)
{
	return 0x00000008U;
}
static inline u32 minion_nvlink_dl_cmd_command_lanedisable_f(void)
{
	return 0x8U;
}
static inline u32 minion_nvlink_dl_cmd_command_fastlanedisable_v(void)
{
	return 0x00000009U;
}
static inline u32 minion_nvlink_dl_cmd_command_fastlanedisable_f(void)
{
	return 0x9U;
}
static inline u32 minion_nvlink_dl_cmd_command_laneshutdown_v(void)
{
	return 0x0000000cU;
}
static inline u32 minion_nvlink_dl_cmd_command_laneshutdown_f(void)
{
	return 0xcU;
}
static inline u32 minion_nvlink_dl_cmd_command_setacmode_v(void)
{
	return 0x0000000aU;
}
static inline u32 minion_nvlink_dl_cmd_command_setacmode_f(void)
{
	return 0xaU;
}
static inline u32 minion_nvlink_dl_cmd_command_clracmode_v(void)
{
	return 0x0000000bU;
}
static inline u32 minion_nvlink_dl_cmd_command_clracmode_f(void)
{
	return 0xbU;
}
static inline u32 minion_nvlink_dl_cmd_command_enablepm_v(void)
{
	return 0x00000010U;
}
static inline u32 minion_nvlink_dl_cmd_command_enablepm_f(void)
{
	return 0x10U;
}
static inline u32 minion_nvlink_dl_cmd_command_disablepm_v(void)
{
	return 0x00000011U;
}
static inline u32 minion_nvlink_dl_cmd_command_disablepm_f(void)
{
	return 0x11U;
}
static inline u32 minion_nvlink_dl_cmd_command_savestate_v(void)
{
	return 0x00000018U;
}
static inline u32 minion_nvlink_dl_cmd_command_savestate_f(void)
{
	return 0x18U;
}
static inline u32 minion_nvlink_dl_cmd_command_restorestate_v(void)
{
	return 0x00000019U;
}
static inline u32 minion_nvlink_dl_cmd_command_restorestate_f(void)
{
	return 0x19U;
}
static inline u32 minion_nvlink_dl_cmd_command_initpll_0_v(void)
{
	return 0x00000020U;
}
static inline u32 minion_nvlink_dl_cmd_command_initpll_0_f(void)
{
	return 0x20U;
}
static inline u32 minion_nvlink_dl_cmd_command_initpll_1_v(void)
{
	return 0x00000021U;
}
static inline u32 minion_nvlink_dl_cmd_command_initpll_1_f(void)
{
	return 0x21U;
}
static inline u32 minion_nvlink_dl_cmd_command_initpll_2_v(void)
{
	return 0x00000022U;
}
static inline u32 minion_nvlink_dl_cmd_command_initpll_2_f(void)
{
	return 0x22U;
}
static inline u32 minion_nvlink_dl_cmd_command_initpll_3_v(void)
{
	return 0x00000023U;
}
static inline u32 minion_nvlink_dl_cmd_command_initpll_3_f(void)
{
	return 0x23U;
}
static inline u32 minion_nvlink_dl_cmd_command_initpll_4_v(void)
{
	return 0x00000024U;
}
static inline u32 minion_nvlink_dl_cmd_command_initpll_4_f(void)
{
	return 0x24U;
}
static inline u32 minion_nvlink_dl_cmd_command_initpll_5_v(void)
{
	return 0x00000025U;
}
static inline u32 minion_nvlink_dl_cmd_command_initpll_5_f(void)
{
	return 0x25U;
}
static inline u32 minion_nvlink_dl_cmd_command_initpll_6_v(void)
{
	return 0x00000026U;
}
static inline u32 minion_nvlink_dl_cmd_command_initpll_6_f(void)
{
	return 0x26U;
}
static inline u32 minion_nvlink_dl_cmd_command_initpll_7_v(void)
{
	return 0x00000027U;
}
static inline u32 minion_nvlink_dl_cmd_command_initpll_7_f(void)
{
	return 0x27U;
}
static inline u32 minion_nvlink_dl_cmd_fault_f(u32 v)
{
	return (v & 0x1U) << 30U;
}
static inline u32 minion_nvlink_dl_cmd_fault_v(u32 r)
{
	return (r >> 30U) & 0x1U;
}
static inline u32 minion_nvlink_dl_cmd_ready_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 minion_nvlink_dl_cmd_ready_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 minion_misc_0_r(void)
{
	return 0x000008b0U;
}
static inline u32 minion_misc_0_scratch_swrw_0_f(u32 v)
{
	return (v & 0xffffffffU) << 0U;
}
static inline u32 minion_misc_0_scratch_swrw_0_v(u32 r)
{
	return (r >> 0U) & 0xffffffffU;
}
static inline u32 minion_nvlink_link_intr_r(u32 i)
{
	return 0x00000a00U + i*4U;
}
static inline u32 minion_nvlink_link_intr___size_1_v(void)
{
	return 0x00000006U;
}
static inline u32 minion_nvlink_link_intr_code_f(u32 v)
{
	return (v & 0xffU) << 0U;
}
static inline u32 minion_nvlink_link_intr_code_m(void)
{
	return 0xffU << 0U;
}
static inline u32 minion_nvlink_link_intr_code_v(u32 r)
{
	return (r >> 0U) & 0xffU;
}
static inline u32 minion_nvlink_link_intr_code_na_v(void)
{
	return 0x00000000U;
}
static inline u32 minion_nvlink_link_intr_code_na_f(void)
{
	return 0x0U;
}
static inline u32 minion_nvlink_link_intr_code_swreq_v(void)
{
	return 0x00000001U;
}
static inline u32 minion_nvlink_link_intr_code_swreq_f(void)
{
	return 0x1U;
}
static inline u32 minion_nvlink_link_intr_code_dlreq_v(void)
{
	return 0x00000002U;
}
static inline u32 minion_nvlink_link_intr_code_dlreq_f(void)
{
	return 0x2U;
}
static inline u32 minion_nvlink_link_intr_code_pmdisabled_v(void)
{
	return 0x00000003U;
}
static inline u32 minion_nvlink_link_intr_code_pmdisabled_f(void)
{
	return 0x3U;
}
static inline u32 minion_nvlink_link_intr_subcode_f(u32 v)
{
	return (v & 0xffU) << 8U;
}
static inline u32 minion_nvlink_link_intr_subcode_m(void)
{
	return 0xffU << 8U;
}
static inline u32 minion_nvlink_link_intr_subcode_v(u32 r)
{
	return (r >> 8U) & 0xffU;
}
static inline u32 minion_nvlink_link_intr_state_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 minion_nvlink_link_intr_state_m(void)
{
	return 0x1U << 31U;
}
static inline u32 minion_nvlink_link_intr_state_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
#endif
