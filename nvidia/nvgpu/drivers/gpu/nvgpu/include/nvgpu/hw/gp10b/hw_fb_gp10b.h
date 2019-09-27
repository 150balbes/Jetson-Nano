/*
 * Copyright (c) 2014-2018, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _hw_fb_gp10b_h_
#define _hw_fb_gp10b_h_

static inline u32 fb_fbhub_num_active_ltcs_r(void)
{
	return 0x00100800U;
}
static inline u32 fb_mmu_ctrl_r(void)
{
	return 0x00100c80U;
}
static inline u32 fb_mmu_ctrl_pri_fifo_empty_v(u32 r)
{
	return (r >> 15U) & 0x1U;
}
static inline u32 fb_mmu_ctrl_pri_fifo_empty_false_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_ctrl_pri_fifo_space_v(u32 r)
{
	return (r >> 16U) & 0xffU;
}
static inline u32 fb_priv_mmu_phy_secure_r(void)
{
	return 0x00100ce4U;
}
static inline u32 fb_mmu_invalidate_pdb_r(void)
{
	return 0x00100cb8U;
}
static inline u32 fb_mmu_invalidate_pdb_aperture_vid_mem_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_invalidate_pdb_aperture_sys_mem_f(void)
{
	return 0x2U;
}
static inline u32 fb_mmu_invalidate_pdb_addr_f(u32 v)
{
	return (v & 0xfffffffU) << 4U;
}
static inline u32 fb_mmu_invalidate_r(void)
{
	return 0x00100cbcU;
}
static inline u32 fb_mmu_invalidate_all_va_true_f(void)
{
	return 0x1U;
}
static inline u32 fb_mmu_invalidate_all_pdb_true_f(void)
{
	return 0x2U;
}
static inline u32 fb_mmu_invalidate_hubtlb_only_s(void)
{
	return 1U;
}
static inline u32 fb_mmu_invalidate_hubtlb_only_f(u32 v)
{
	return (v & 0x1U) << 2U;
}
static inline u32 fb_mmu_invalidate_hubtlb_only_m(void)
{
	return 0x1U << 2U;
}
static inline u32 fb_mmu_invalidate_hubtlb_only_v(u32 r)
{
	return (r >> 2U) & 0x1U;
}
static inline u32 fb_mmu_invalidate_hubtlb_only_true_f(void)
{
	return 0x4U;
}
static inline u32 fb_mmu_invalidate_replay_s(void)
{
	return 3U;
}
static inline u32 fb_mmu_invalidate_replay_f(u32 v)
{
	return (v & 0x7U) << 3U;
}
static inline u32 fb_mmu_invalidate_replay_m(void)
{
	return 0x7U << 3U;
}
static inline u32 fb_mmu_invalidate_replay_v(u32 r)
{
	return (r >> 3U) & 0x7U;
}
static inline u32 fb_mmu_invalidate_replay_none_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_invalidate_replay_start_f(void)
{
	return 0x8U;
}
static inline u32 fb_mmu_invalidate_replay_start_ack_all_f(void)
{
	return 0x10U;
}
static inline u32 fb_mmu_invalidate_replay_cancel_targeted_f(void)
{
	return 0x18U;
}
static inline u32 fb_mmu_invalidate_replay_cancel_global_f(void)
{
	return 0x20U;
}
static inline u32 fb_mmu_invalidate_replay_cancel_f(void)
{
	return 0x20U;
}
static inline u32 fb_mmu_invalidate_sys_membar_s(void)
{
	return 1U;
}
static inline u32 fb_mmu_invalidate_sys_membar_f(u32 v)
{
	return (v & 0x1U) << 6U;
}
static inline u32 fb_mmu_invalidate_sys_membar_m(void)
{
	return 0x1U << 6U;
}
static inline u32 fb_mmu_invalidate_sys_membar_v(u32 r)
{
	return (r >> 6U) & 0x1U;
}
static inline u32 fb_mmu_invalidate_sys_membar_true_f(void)
{
	return 0x40U;
}
static inline u32 fb_mmu_invalidate_ack_s(void)
{
	return 2U;
}
static inline u32 fb_mmu_invalidate_ack_f(u32 v)
{
	return (v & 0x3U) << 7U;
}
static inline u32 fb_mmu_invalidate_ack_m(void)
{
	return 0x3U << 7U;
}
static inline u32 fb_mmu_invalidate_ack_v(u32 r)
{
	return (r >> 7U) & 0x3U;
}
static inline u32 fb_mmu_invalidate_ack_ack_none_required_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_invalidate_ack_ack_intranode_f(void)
{
	return 0x100U;
}
static inline u32 fb_mmu_invalidate_ack_ack_globally_f(void)
{
	return 0x80U;
}
static inline u32 fb_mmu_invalidate_cancel_client_id_s(void)
{
	return 6U;
}
static inline u32 fb_mmu_invalidate_cancel_client_id_f(u32 v)
{
	return (v & 0x3fU) << 9U;
}
static inline u32 fb_mmu_invalidate_cancel_client_id_m(void)
{
	return 0x3fU << 9U;
}
static inline u32 fb_mmu_invalidate_cancel_client_id_v(u32 r)
{
	return (r >> 9U) & 0x3fU;
}
static inline u32 fb_mmu_invalidate_cancel_gpc_id_s(void)
{
	return 5U;
}
static inline u32 fb_mmu_invalidate_cancel_gpc_id_f(u32 v)
{
	return (v & 0x1fU) << 15U;
}
static inline u32 fb_mmu_invalidate_cancel_gpc_id_m(void)
{
	return 0x1fU << 15U;
}
static inline u32 fb_mmu_invalidate_cancel_gpc_id_v(u32 r)
{
	return (r >> 15U) & 0x1fU;
}
static inline u32 fb_mmu_invalidate_cancel_client_type_s(void)
{
	return 1U;
}
static inline u32 fb_mmu_invalidate_cancel_client_type_f(u32 v)
{
	return (v & 0x1U) << 20U;
}
static inline u32 fb_mmu_invalidate_cancel_client_type_m(void)
{
	return 0x1U << 20U;
}
static inline u32 fb_mmu_invalidate_cancel_client_type_v(u32 r)
{
	return (r >> 20U) & 0x1U;
}
static inline u32 fb_mmu_invalidate_cancel_client_type_gpc_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_invalidate_cancel_client_type_hub_f(void)
{
	return 0x100000U;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_s(void)
{
	return 3U;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_f(u32 v)
{
	return (v & 0x7U) << 24U;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_m(void)
{
	return 0x7U << 24U;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_v(u32 r)
{
	return (r >> 24U) & 0x7U;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_all_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_pte_only_f(void)
{
	return 0x1000000U;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_up_to_pde0_f(void)
{
	return 0x2000000U;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_up_to_pde1_f(void)
{
	return 0x3000000U;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_up_to_pde2_f(void)
{
	return 0x4000000U;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_up_to_pde3_f(void)
{
	return 0x5000000U;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_up_to_pde4_f(void)
{
	return 0x6000000U;
}
static inline u32 fb_mmu_invalidate_cancel_cache_level_up_to_pde5_f(void)
{
	return 0x7000000U;
}
static inline u32 fb_mmu_invalidate_trigger_s(void)
{
	return 1U;
}
static inline u32 fb_mmu_invalidate_trigger_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 fb_mmu_invalidate_trigger_m(void)
{
	return 0x1U << 31U;
}
static inline u32 fb_mmu_invalidate_trigger_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 fb_mmu_invalidate_trigger_true_f(void)
{
	return 0x80000000U;
}
static inline u32 fb_mmu_debug_wr_r(void)
{
	return 0x00100cc8U;
}
static inline u32 fb_mmu_debug_wr_aperture_s(void)
{
	return 2U;
}
static inline u32 fb_mmu_debug_wr_aperture_f(u32 v)
{
	return (v & 0x3U) << 0U;
}
static inline u32 fb_mmu_debug_wr_aperture_m(void)
{
	return 0x3U << 0U;
}
static inline u32 fb_mmu_debug_wr_aperture_v(u32 r)
{
	return (r >> 0U) & 0x3U;
}
static inline u32 fb_mmu_debug_wr_aperture_vid_mem_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_debug_wr_aperture_sys_mem_coh_f(void)
{
	return 0x2U;
}
static inline u32 fb_mmu_debug_wr_aperture_sys_mem_ncoh_f(void)
{
	return 0x3U;
}
static inline u32 fb_mmu_debug_wr_vol_false_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_debug_wr_vol_true_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_debug_wr_vol_true_f(void)
{
	return 0x4U;
}
static inline u32 fb_mmu_debug_wr_addr_f(u32 v)
{
	return (v & 0xfffffffU) << 4U;
}
static inline u32 fb_mmu_debug_wr_addr_alignment_v(void)
{
	return 0x0000000cU;
}
static inline u32 fb_mmu_debug_rd_r(void)
{
	return 0x00100cccU;
}
static inline u32 fb_mmu_debug_rd_aperture_vid_mem_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_debug_rd_aperture_sys_mem_coh_f(void)
{
	return 0x2U;
}
static inline u32 fb_mmu_debug_rd_aperture_sys_mem_ncoh_f(void)
{
	return 0x3U;
}
static inline u32 fb_mmu_debug_rd_vol_false_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_debug_rd_addr_f(u32 v)
{
	return (v & 0xfffffffU) << 4U;
}
static inline u32 fb_mmu_debug_rd_addr_alignment_v(void)
{
	return 0x0000000cU;
}
static inline u32 fb_mmu_debug_ctrl_r(void)
{
	return 0x00100cc4U;
}
static inline u32 fb_mmu_debug_ctrl_debug_v(u32 r)
{
	return (r >> 16U) & 0x1U;
}
static inline u32 fb_mmu_debug_ctrl_debug_m(void)
{
	return 0x1U << 16U;
}
static inline u32 fb_mmu_debug_ctrl_debug_enabled_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_debug_ctrl_debug_disabled_v(void)
{
	return 0x00000000U;
}
static inline u32 fb_mmu_vpr_info_r(void)
{
	return 0x00100cd0U;
}
static inline u32 fb_mmu_vpr_info_fetch_v(u32 r)
{
	return (r >> 2U) & 0x1U;
}
static inline u32 fb_mmu_vpr_info_fetch_false_v(void)
{
	return 0x00000000U;
}
static inline u32 fb_mmu_vpr_info_fetch_true_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_niso_flush_sysmem_addr_r(void)
{
	return 0x00100c10U;
}
#endif
