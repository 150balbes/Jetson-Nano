/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _hw_pbdma_gp106_h_
#define _hw_pbdma_gp106_h_

static inline u32 pbdma_gp_entry1_r(void)
{
	return 0x10000004U;
}
static inline u32 pbdma_gp_entry1_get_hi_v(u32 r)
{
	return (r >> 0U) & 0xffU;
}
static inline u32 pbdma_gp_entry1_length_f(u32 v)
{
	return (v & 0x1fffffU) << 10U;
}
static inline u32 pbdma_gp_entry1_length_v(u32 r)
{
	return (r >> 10U) & 0x1fffffU;
}
static inline u32 pbdma_gp_base_r(u32 i)
{
	return 0x00040048U + i*8192U;
}
static inline u32 pbdma_gp_base__size_1_v(void)
{
	return 0x00000004U;
}
static inline u32 pbdma_gp_base_offset_f(u32 v)
{
	return (v & 0x1fffffffU) << 3U;
}
static inline u32 pbdma_gp_base_rsvd_s(void)
{
	return 3U;
}
static inline u32 pbdma_gp_base_hi_r(u32 i)
{
	return 0x0004004cU + i*8192U;
}
static inline u32 pbdma_gp_base_hi_offset_f(u32 v)
{
	return (v & 0xffU) << 0U;
}
static inline u32 pbdma_gp_base_hi_limit2_f(u32 v)
{
	return (v & 0x1fU) << 16U;
}
static inline u32 pbdma_gp_fetch_r(u32 i)
{
	return 0x00040050U + i*8192U;
}
static inline u32 pbdma_gp_get_r(u32 i)
{
	return 0x00040014U + i*8192U;
}
static inline u32 pbdma_gp_put_r(u32 i)
{
	return 0x00040000U + i*8192U;
}
static inline u32 pbdma_pb_fetch_r(u32 i)
{
	return 0x00040054U + i*8192U;
}
static inline u32 pbdma_pb_fetch_hi_r(u32 i)
{
	return 0x00040058U + i*8192U;
}
static inline u32 pbdma_get_r(u32 i)
{
	return 0x00040018U + i*8192U;
}
static inline u32 pbdma_get_hi_r(u32 i)
{
	return 0x0004001cU + i*8192U;
}
static inline u32 pbdma_put_r(u32 i)
{
	return 0x0004005cU + i*8192U;
}
static inline u32 pbdma_put_hi_r(u32 i)
{
	return 0x00040060U + i*8192U;
}
static inline u32 pbdma_formats_r(u32 i)
{
	return 0x0004009cU + i*8192U;
}
static inline u32 pbdma_formats_gp_fermi0_f(void)
{
	return 0x0U;
}
static inline u32 pbdma_formats_pb_fermi1_f(void)
{
	return 0x100U;
}
static inline u32 pbdma_formats_mp_fermi0_f(void)
{
	return 0x0U;
}
static inline u32 pbdma_pb_header_r(u32 i)
{
	return 0x00040084U + i*8192U;
}
static inline u32 pbdma_pb_header_priv_user_f(void)
{
	return 0x0U;
}
static inline u32 pbdma_pb_header_method_zero_f(void)
{
	return 0x0U;
}
static inline u32 pbdma_pb_header_subchannel_zero_f(void)
{
	return 0x0U;
}
static inline u32 pbdma_pb_header_level_main_f(void)
{
	return 0x0U;
}
static inline u32 pbdma_pb_header_first_true_f(void)
{
	return 0x400000U;
}
static inline u32 pbdma_pb_header_type_inc_f(void)
{
	return 0x20000000U;
}
static inline u32 pbdma_pb_header_type_non_inc_f(void)
{
	return 0x60000000U;
}
static inline u32 pbdma_hdr_shadow_r(u32 i)
{
	return 0x00040118U + i*8192U;
}
static inline u32 pbdma_gp_shadow_0_r(u32 i)
{
	return 0x00040110U + i*8192U;
}
static inline u32 pbdma_gp_shadow_1_r(u32 i)
{
	return 0x00040114U + i*8192U;
}
static inline u32 pbdma_subdevice_r(u32 i)
{
	return 0x00040094U + i*8192U;
}
static inline u32 pbdma_subdevice_id_f(u32 v)
{
	return (v & 0xfffU) << 0U;
}
static inline u32 pbdma_subdevice_status_active_f(void)
{
	return 0x10000000U;
}
static inline u32 pbdma_subdevice_channel_dma_enable_f(void)
{
	return 0x20000000U;
}
static inline u32 pbdma_method0_r(u32 i)
{
	return 0x000400c0U + i*8192U;
}
static inline u32 pbdma_method0_fifo_size_v(void)
{
	return 0x00000004U;
}
static inline u32 pbdma_method0_addr_f(u32 v)
{
	return (v & 0xfffU) << 2U;
}
static inline u32 pbdma_method0_addr_v(u32 r)
{
	return (r >> 2U) & 0xfffU;
}
static inline u32 pbdma_method0_subch_v(u32 r)
{
	return (r >> 16U) & 0x7U;
}
static inline u32 pbdma_method0_first_true_f(void)
{
	return 0x400000U;
}
static inline u32 pbdma_method0_valid_true_f(void)
{
	return 0x80000000U;
}
static inline u32 pbdma_method1_r(u32 i)
{
	return 0x000400c8U + i*8192U;
}
static inline u32 pbdma_method2_r(u32 i)
{
	return 0x000400d0U + i*8192U;
}
static inline u32 pbdma_method3_r(u32 i)
{
	return 0x000400d8U + i*8192U;
}
static inline u32 pbdma_data0_r(u32 i)
{
	return 0x000400c4U + i*8192U;
}
static inline u32 pbdma_target_r(u32 i)
{
	return 0x000400acU + i*8192U;
}
static inline u32 pbdma_target_engine_sw_f(void)
{
	return 0x1fU;
}
static inline u32 pbdma_acquire_r(u32 i)
{
	return 0x00040030U + i*8192U;
}
static inline u32 pbdma_acquire_retry_man_2_f(void)
{
	return 0x2U;
}
static inline u32 pbdma_acquire_retry_exp_2_f(void)
{
	return 0x100U;
}
static inline u32 pbdma_acquire_timeout_exp_max_f(void)
{
	return 0x7800U;
}
static inline u32 pbdma_acquire_timeout_man_max_f(void)
{
	return 0x7fff8000U;
}
static inline u32 pbdma_acquire_timeout_en_disable_f(void)
{
	return 0x0U;
}
static inline u32 pbdma_status_r(u32 i)
{
	return 0x00040100U + i*8192U;
}
static inline u32 pbdma_channel_r(u32 i)
{
	return 0x00040120U + i*8192U;
}
static inline u32 pbdma_signature_r(u32 i)
{
	return 0x00040010U + i*8192U;
}
static inline u32 pbdma_signature_hw_valid_f(void)
{
	return 0xfaceU;
}
static inline u32 pbdma_signature_sw_zero_f(void)
{
	return 0x0U;
}
static inline u32 pbdma_userd_r(u32 i)
{
	return 0x00040008U + i*8192U;
}
static inline u32 pbdma_userd_target_vid_mem_f(void)
{
	return 0x0U;
}
static inline u32 pbdma_userd_target_sys_mem_coh_f(void)
{
	return 0x2U;
}
static inline u32 pbdma_userd_target_sys_mem_ncoh_f(void)
{
	return 0x3U;
}
static inline u32 pbdma_userd_addr_f(u32 v)
{
	return (v & 0x7fffffU) << 9U;
}
static inline u32 pbdma_userd_hi_r(u32 i)
{
	return 0x0004000cU + i*8192U;
}
static inline u32 pbdma_userd_hi_addr_f(u32 v)
{
	return (v & 0xffU) << 0U;
}
static inline u32 pbdma_config_r(u32 i)
{
	return 0x000400f4U + i*8192U;
}
static inline u32 pbdma_config_auth_level_privileged_f(void)
{
	return 0x100U;
}
static inline u32 pbdma_hce_ctrl_r(u32 i)
{
	return 0x000400e4U + i*8192U;
}
static inline u32 pbdma_hce_ctrl_hce_priv_mode_yes_f(void)
{
	return 0x20U;
}
static inline u32 pbdma_intr_0_r(u32 i)
{
	return 0x00040108U + i*8192U;
}
static inline u32 pbdma_intr_0_memreq_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 pbdma_intr_0_memreq_pending_f(void)
{
	return 0x1U;
}
static inline u32 pbdma_intr_0_memack_timeout_pending_f(void)
{
	return 0x2U;
}
static inline u32 pbdma_intr_0_memack_extra_pending_f(void)
{
	return 0x4U;
}
static inline u32 pbdma_intr_0_memdat_timeout_pending_f(void)
{
	return 0x8U;
}
static inline u32 pbdma_intr_0_memdat_extra_pending_f(void)
{
	return 0x10U;
}
static inline u32 pbdma_intr_0_memflush_pending_f(void)
{
	return 0x20U;
}
static inline u32 pbdma_intr_0_memop_pending_f(void)
{
	return 0x40U;
}
static inline u32 pbdma_intr_0_lbconnect_pending_f(void)
{
	return 0x80U;
}
static inline u32 pbdma_intr_0_lbreq_pending_f(void)
{
	return 0x100U;
}
static inline u32 pbdma_intr_0_lback_timeout_pending_f(void)
{
	return 0x200U;
}
static inline u32 pbdma_intr_0_lback_extra_pending_f(void)
{
	return 0x400U;
}
static inline u32 pbdma_intr_0_lbdat_timeout_pending_f(void)
{
	return 0x800U;
}
static inline u32 pbdma_intr_0_lbdat_extra_pending_f(void)
{
	return 0x1000U;
}
static inline u32 pbdma_intr_0_gpfifo_pending_f(void)
{
	return 0x2000U;
}
static inline u32 pbdma_intr_0_gpptr_pending_f(void)
{
	return 0x4000U;
}
static inline u32 pbdma_intr_0_gpentry_pending_f(void)
{
	return 0x8000U;
}
static inline u32 pbdma_intr_0_gpcrc_pending_f(void)
{
	return 0x10000U;
}
static inline u32 pbdma_intr_0_pbptr_pending_f(void)
{
	return 0x20000U;
}
static inline u32 pbdma_intr_0_pbentry_pending_f(void)
{
	return 0x40000U;
}
static inline u32 pbdma_intr_0_pbcrc_pending_f(void)
{
	return 0x80000U;
}
static inline u32 pbdma_intr_0_xbarconnect_pending_f(void)
{
	return 0x100000U;
}
static inline u32 pbdma_intr_0_method_pending_f(void)
{
	return 0x200000U;
}
static inline u32 pbdma_intr_0_methodcrc_pending_f(void)
{
	return 0x400000U;
}
static inline u32 pbdma_intr_0_device_pending_f(void)
{
	return 0x800000U;
}
static inline u32 pbdma_intr_0_semaphore_pending_f(void)
{
	return 0x2000000U;
}
static inline u32 pbdma_intr_0_acquire_pending_f(void)
{
	return 0x4000000U;
}
static inline u32 pbdma_intr_0_pri_pending_f(void)
{
	return 0x8000000U;
}
static inline u32 pbdma_intr_0_no_ctxsw_seg_pending_f(void)
{
	return 0x20000000U;
}
static inline u32 pbdma_intr_0_pbseg_pending_f(void)
{
	return 0x40000000U;
}
static inline u32 pbdma_intr_0_signature_pending_f(void)
{
	return 0x80000000U;
}
static inline u32 pbdma_intr_1_r(u32 i)
{
	return 0x00040148U + i*8192U;
}
static inline u32 pbdma_intr_en_0_r(u32 i)
{
	return 0x0004010cU + i*8192U;
}
static inline u32 pbdma_intr_en_0_lbreq_enabled_f(void)
{
	return 0x100U;
}
static inline u32 pbdma_intr_en_1_r(u32 i)
{
	return 0x0004014cU + i*8192U;
}
static inline u32 pbdma_intr_stall_r(u32 i)
{
	return 0x0004013cU + i*8192U;
}
static inline u32 pbdma_intr_stall_lbreq_enabled_f(void)
{
	return 0x100U;
}
static inline u32 pbdma_intr_stall_1_r(u32 i)
{
	return 0x00040140U + i*8192U;
}
static inline u32 pbdma_intr_stall_1_hce_illegal_op_enabled_f(void)
{
	return 0x1U;
}
static inline u32 pbdma_udma_nop_r(void)
{
	return 0x00000008U;
}
static inline u32 pbdma_runlist_timeslice_r(u32 i)
{
	return 0x000400f8U + i*8192U;
}
static inline u32 pbdma_runlist_timeslice_timeout_128_f(void)
{
	return 0x80U;
}
static inline u32 pbdma_runlist_timeslice_timescale_3_f(void)
{
	return 0x3000U;
}
static inline u32 pbdma_runlist_timeslice_enable_true_f(void)
{
	return 0x10000000U;
}
#endif
