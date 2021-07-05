/*
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
#ifndef _hw_fb_gv100_h_
#define _hw_fb_gv100_h_

static inline u32 fb_fbhub_num_active_ltcs_r(void)
{
	return 0x00100800U;
}
static inline u32 fb_fbhub_num_active_ltcs_use_nvlink_f(u32 v)
{
	return (v & 0xffU) << 16U;
}
static inline u32 fb_fbhub_num_active_ltcs_use_nvlink_m(void)
{
	return 0xffU << 16U;
}
static inline u32 fb_fbhub_num_active_ltcs_use_nvlink_v(u32 r)
{
	return (r >> 16U) & 0xffU;
}
static inline u32 fb_fbhub_num_active_ltcs_use_nvlink_peer_f(u32 v, u32 i)
{
	return (v & 0x1U) << (16U + i*1U);
}
static inline u32 fb_fbhub_num_active_ltcs_use_nvlink_peer_m(u32 i)
{
	return 0x1U << (16U + i*1U);
}
static inline u32 fb_fbhub_num_active_ltcs_use_nvlink_peer_v(u32 r, u32 i)
{
	return (r >> (16U + i*1U)) & 0x1U;
}
static inline u32 fb_fbhub_num_active_ltcs_use_nvlink_peer___size_1_v(void)
{
	return 0x00000008U;
}
static inline u32 fb_fbhub_num_active_ltcs_use_nvlink_peer___size_1_f(u32 i)
{
	return 0x0U << (32U + i*1U);
}
static inline u32 fb_fbhub_num_active_ltcs_use_nvlink_peer_enabled_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_fbhub_num_active_ltcs_use_nvlink_peer_enabled_f(u32 i)
{
	return 0x1U << (32U + i*1U);
}
static inline u32 fb_fbhub_num_active_ltcs_use_nvlink_peer_disabled_v(void)
{
	return 0x00000000U;
}
static inline u32 fb_fbhub_num_active_ltcs_use_nvlink_peer_disabled_f(u32 i)
{
	return 0x0U << (32U + i*1U);
}
static inline u32 fb_fbhub_num_active_ltcs_hub_sys_atomic_mode_f(u32 v)
{
	return (v & 0x1U) << 25U;
}
static inline u32 fb_fbhub_num_active_ltcs_hub_sys_atomic_mode_m(void)
{
	return 0x1U << 25U;
}
static inline u32 fb_fbhub_num_active_ltcs_hub_sys_atomic_mode_v(u32 r)
{
	return (r >> 25U) & 0x1U;
}
static inline u32 fb_fbhub_num_active_ltcs_hub_sys_atomic_mode_use_read_v(void)
{
	return 0x00000000U;
}
static inline u32 fb_fbhub_num_active_ltcs_hub_sys_atomic_mode_use_read_f(void)
{
	return 0x0U;
}
static inline u32 fb_fbhub_num_active_ltcs_hub_sys_atomic_mode_use_rmw_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_fbhub_num_active_ltcs_hub_sys_atomic_mode_use_rmw_f(void)
{
	return 0x2000000U;
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
static inline u32 fb_mmu_ctrl_atomic_capability_mode_f(u32 v)
{
	return (v & 0x3U) << 24U;
}
static inline u32 fb_mmu_ctrl_atomic_capability_mode_m(void)
{
	return 0x3U << 24U;
}
static inline u32 fb_mmu_ctrl_atomic_capability_mode_v(u32 r)
{
	return (r >> 24U) & 0x3U;
}
static inline u32 fb_mmu_ctrl_atomic_capability_mode_l2_v(void)
{
	return 0x00000000U;
}
static inline u32 fb_mmu_ctrl_atomic_capability_mode_l2_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_ctrl_atomic_capability_mode_atomic_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_ctrl_atomic_capability_mode_atomic_f(void)
{
	return 0x1000000U;
}
static inline u32 fb_mmu_ctrl_atomic_capability_mode_rmw_v(void)
{
	return 0x00000002U;
}
static inline u32 fb_mmu_ctrl_atomic_capability_mode_rmw_f(void)
{
	return 0x2000000U;
}
static inline u32 fb_mmu_ctrl_atomic_capability_mode_power_v(void)
{
	return 0x00000003U;
}
static inline u32 fb_mmu_ctrl_atomic_capability_mode_power_f(void)
{
	return 0x3000000U;
}
static inline u32 fb_hsmmu_pri_mmu_ctrl_r(void)
{
	return 0x001fac80U;
}
static inline u32 fb_hsmmu_pri_mmu_ctrl_atomic_capability_mode_f(u32 v)
{
	return (v & 0x3U) << 24U;
}
static inline u32 fb_hsmmu_pri_mmu_ctrl_atomic_capability_mode_m(void)
{
	return 0x3U << 24U;
}
static inline u32 fb_hsmmu_pri_mmu_ctrl_atomic_capability_mode_v(u32 r)
{
	return (r >> 24U) & 0x3U;
}
static inline u32 fb_hsmmu_pri_mmu_ctrl_atomic_capability_mode_l2_v(void)
{
	return 0x00000000U;
}
static inline u32 fb_hsmmu_pri_mmu_ctrl_atomic_capability_mode_l2_f(void)
{
	return 0x0U;
}
static inline u32 fb_hsmmu_pri_mmu_ctrl_atomic_capability_mode_atomic_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_hsmmu_pri_mmu_ctrl_atomic_capability_mode_atomic_f(void)
{
	return 0x1000000U;
}
static inline u32 fb_hsmmu_pri_mmu_ctrl_atomic_capability_mode_rmw_v(void)
{
	return 0x00000002U;
}
static inline u32 fb_hsmmu_pri_mmu_ctrl_atomic_capability_mode_rmw_f(void)
{
	return 0x2000000U;
}
static inline u32 fb_hsmmu_pri_mmu_ctrl_atomic_capability_mode_power_v(void)
{
	return 0x00000003U;
}
static inline u32 fb_hsmmu_pri_mmu_ctrl_atomic_capability_mode_power_f(void)
{
	return 0x3000000U;
}
static inline u32 fb_hsmmu_pri_mmu_debug_ctrl_r(void)
{
	return 0x001facc4U;
}
static inline u32 fb_hsmmu_pri_mmu_debug_ctrl_debug_v(u32 r)
{
	return (r >> 16U) & 0x1U;
}
static inline u32 fb_hsmmu_pri_mmu_debug_ctrl_debug_m(void)
{
	return 0x1U << 16U;
}
static inline u32 fb_hsmmu_pri_mmu_debug_ctrl_debug_enabled_f(void)
{
	return 0x10000U;
}
static inline u32 fb_hsmmu_pri_mmu_debug_ctrl_debug_disabled_f(void)
{
	return 0x0U;
}
static inline u32 fb_hshub_num_active_ltcs_r(void)
{
	return 0x001fbc20U;
}
static inline u32 fb_hshub_num_active_ltcs_use_nvlink_f(u32 v)
{
	return (v & 0xffU) << 16U;
}
static inline u32 fb_hshub_num_active_ltcs_use_nvlink_m(void)
{
	return 0xffU << 16U;
}
static inline u32 fb_hshub_num_active_ltcs_use_nvlink_v(u32 r)
{
	return (r >> 16U) & 0xffU;
}
static inline u32 fb_hshub_num_active_ltcs_use_nvlink_peer_f(u32 v, u32 i)
{
	return (v & 0x1U) << (16U + i*1U);
}
static inline u32 fb_hshub_num_active_ltcs_use_nvlink_peer_m(u32 i)
{
	return 0x1U << (16U + i*1U);
}
static inline u32 fb_hshub_num_active_ltcs_use_nvlink_peer_v(u32 r, u32 i)
{
	return (r >> (16U + i*1U)) & 0x1U;
}
static inline u32 fb_hshub_num_active_ltcs_use_nvlink_peer___size_1_v(void)
{
	return 0x00000008U;
}
static inline u32 fb_hshub_num_active_ltcs_use_nvlink_peer___size_1_f(u32 i)
{
	return 0x0U << (32U + i*1U);
}
static inline u32 fb_hshub_num_active_ltcs_use_nvlink_peer_enabled_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_hshub_num_active_ltcs_use_nvlink_peer_enabled_f(u32 i)
{
	return 0x1U << (32U + i*1U);
}
static inline u32 fb_hshub_num_active_ltcs_use_nvlink_peer_disabled_v(void)
{
	return 0x00000000U;
}
static inline u32 fb_hshub_num_active_ltcs_use_nvlink_peer_disabled_f(u32 i)
{
	return 0x0U << (32U + i*1U);
}
static inline u32 fb_hshub_num_active_ltcs_hub_sys_atomic_mode_f(u32 v)
{
	return (v & 0x1U) << 25U;
}
static inline u32 fb_hshub_num_active_ltcs_hub_sys_atomic_mode_m(void)
{
	return 0x1U << 25U;
}
static inline u32 fb_hshub_num_active_ltcs_hub_sys_atomic_mode_v(u32 r)
{
	return (r >> 25U) & 0x1U;
}
static inline u32 fb_hshub_num_active_ltcs_hub_sys_atomic_mode_use_read_v(void)
{
	return 0x00000000U;
}
static inline u32 fb_hshub_num_active_ltcs_hub_sys_atomic_mode_use_read_f(void)
{
	return 0x0U;
}
static inline u32 fb_hshub_num_active_ltcs_hub_sys_atomic_mode_use_rmw_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_hshub_num_active_ltcs_hub_sys_atomic_mode_use_rmw_f(void)
{
	return 0x2000000U;
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
static inline u32 fb_mmu_invalidate_replay_cancel_global_f(void)
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
static inline u32 fb_mmu_debug_ctrl_debug_enabled_f(void)
{
	return 0x10000U;
}
static inline u32 fb_mmu_debug_ctrl_debug_disabled_v(void)
{
	return 0x00000000U;
}
static inline u32 fb_mmu_debug_ctrl_debug_disabled_f(void)
{
	return 0x0U;
}
static inline u32 fb_niso_cfg1_r(void)
{
	return 0x00100c14U;
}
static inline u32 fb_niso_cfg1_sysmem_nvlink_f(u32 v)
{
	return (v & 0x1U) << 17U;
}
static inline u32 fb_niso_cfg1_sysmem_nvlink_m(void)
{
	return 0x1U << 17U;
}
static inline u32 fb_niso_cfg1_sysmem_nvlink_v(u32 r)
{
	return (r >> 17U) & 0x1U;
}
static inline u32 fb_niso_cfg1_sysmem_nvlink_enabled_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_niso_cfg1_sysmem_nvlink_enabled_f(void)
{
	return 0x20000U;
}
static inline u32 fb_niso_flush_sysmem_addr_r(void)
{
	return 0x00100c10U;
}
static inline u32 fb_niso_intr_r(void)
{
	return 0x00100a20U;
}
static inline u32 fb_niso_intr_hub_access_counter_notify_m(void)
{
	return 0x1U << 0U;
}
static inline u32 fb_niso_intr_hub_access_counter_notify_pending_f(void)
{
	return 0x1U;
}
static inline u32 fb_niso_intr_hub_access_counter_error_m(void)
{
	return 0x1U << 1U;
}
static inline u32 fb_niso_intr_hub_access_counter_error_pending_f(void)
{
	return 0x2U;
}
static inline u32 fb_niso_intr_mmu_replayable_fault_notify_m(void)
{
	return 0x1U << 27U;
}
static inline u32 fb_niso_intr_mmu_replayable_fault_notify_pending_f(void)
{
	return 0x8000000U;
}
static inline u32 fb_niso_intr_mmu_replayable_fault_overflow_m(void)
{
	return 0x1U << 28U;
}
static inline u32 fb_niso_intr_mmu_replayable_fault_overflow_pending_f(void)
{
	return 0x10000000U;
}
static inline u32 fb_niso_intr_mmu_nonreplayable_fault_notify_m(void)
{
	return 0x1U << 29U;
}
static inline u32 fb_niso_intr_mmu_nonreplayable_fault_notify_pending_f(void)
{
	return 0x20000000U;
}
static inline u32 fb_niso_intr_mmu_nonreplayable_fault_overflow_m(void)
{
	return 0x1U << 30U;
}
static inline u32 fb_niso_intr_mmu_nonreplayable_fault_overflow_pending_f(void)
{
	return 0x40000000U;
}
static inline u32 fb_niso_intr_mmu_other_fault_notify_m(void)
{
	return 0x1U << 31U;
}
static inline u32 fb_niso_intr_mmu_other_fault_notify_pending_f(void)
{
	return 0x80000000U;
}
static inline u32 fb_niso_intr_en_r(u32 i)
{
	return 0x00100a24U + i*4U;
}
static inline u32 fb_niso_intr_en__size_1_v(void)
{
	return 0x00000002U;
}
static inline u32 fb_niso_intr_en_hub_access_counter_notify_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 fb_niso_intr_en_hub_access_counter_notify_enabled_f(void)
{
	return 0x1U;
}
static inline u32 fb_niso_intr_en_hub_access_counter_error_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 fb_niso_intr_en_hub_access_counter_error_enabled_f(void)
{
	return 0x2U;
}
static inline u32 fb_niso_intr_en_mmu_replayable_fault_notify_f(u32 v)
{
	return (v & 0x1U) << 27U;
}
static inline u32 fb_niso_intr_en_mmu_replayable_fault_notify_enabled_f(void)
{
	return 0x8000000U;
}
static inline u32 fb_niso_intr_en_mmu_replayable_fault_overflow_f(u32 v)
{
	return (v & 0x1U) << 28U;
}
static inline u32 fb_niso_intr_en_mmu_replayable_fault_overflow_enabled_f(void)
{
	return 0x10000000U;
}
static inline u32 fb_niso_intr_en_mmu_nonreplayable_fault_notify_f(u32 v)
{
	return (v & 0x1U) << 29U;
}
static inline u32 fb_niso_intr_en_mmu_nonreplayable_fault_notify_enabled_f(void)
{
	return 0x20000000U;
}
static inline u32 fb_niso_intr_en_mmu_nonreplayable_fault_overflow_f(u32 v)
{
	return (v & 0x1U) << 30U;
}
static inline u32 fb_niso_intr_en_mmu_nonreplayable_fault_overflow_enabled_f(void)
{
	return 0x40000000U;
}
static inline u32 fb_niso_intr_en_mmu_other_fault_notify_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 fb_niso_intr_en_mmu_other_fault_notify_enabled_f(void)
{
	return 0x80000000U;
}
static inline u32 fb_niso_intr_en_set_r(u32 i)
{
	return 0x00100a2cU + i*4U;
}
static inline u32 fb_niso_intr_en_set__size_1_v(void)
{
	return 0x00000002U;
}
static inline u32 fb_niso_intr_en_set_hub_access_counter_notify_m(void)
{
	return 0x1U << 0U;
}
static inline u32 fb_niso_intr_en_set_hub_access_counter_notify_set_f(void)
{
	return 0x1U;
}
static inline u32 fb_niso_intr_en_set_hub_access_counter_error_m(void)
{
	return 0x1U << 1U;
}
static inline u32 fb_niso_intr_en_set_hub_access_counter_error_set_f(void)
{
	return 0x2U;
}
static inline u32 fb_niso_intr_en_set_mmu_replayable_fault_notify_m(void)
{
	return 0x1U << 27U;
}
static inline u32 fb_niso_intr_en_set_mmu_replayable_fault_notify_set_f(void)
{
	return 0x8000000U;
}
static inline u32 fb_niso_intr_en_set_mmu_replayable_fault_overflow_m(void)
{
	return 0x1U << 28U;
}
static inline u32 fb_niso_intr_en_set_mmu_replayable_fault_overflow_set_f(void)
{
	return 0x10000000U;
}
static inline u32 fb_niso_intr_en_set_mmu_nonreplayable_fault_notify_m(void)
{
	return 0x1U << 29U;
}
static inline u32 fb_niso_intr_en_set_mmu_nonreplayable_fault_notify_set_f(void)
{
	return 0x20000000U;
}
static inline u32 fb_niso_intr_en_set_mmu_nonreplayable_fault_overflow_m(void)
{
	return 0x1U << 30U;
}
static inline u32 fb_niso_intr_en_set_mmu_nonreplayable_fault_overflow_set_f(void)
{
	return 0x40000000U;
}
static inline u32 fb_niso_intr_en_set_mmu_other_fault_notify_m(void)
{
	return 0x1U << 31U;
}
static inline u32 fb_niso_intr_en_set_mmu_other_fault_notify_set_f(void)
{
	return 0x80000000U;
}
static inline u32 fb_niso_intr_en_clr_r(u32 i)
{
	return 0x00100a34U + i*4U;
}
static inline u32 fb_niso_intr_en_clr__size_1_v(void)
{
	return 0x00000002U;
}
static inline u32 fb_niso_intr_en_clr_hub_access_counter_notify_m(void)
{
	return 0x1U << 0U;
}
static inline u32 fb_niso_intr_en_clr_hub_access_counter_notify_set_f(void)
{
	return 0x1U;
}
static inline u32 fb_niso_intr_en_clr_hub_access_counter_error_m(void)
{
	return 0x1U << 1U;
}
static inline u32 fb_niso_intr_en_clr_hub_access_counter_error_set_f(void)
{
	return 0x2U;
}
static inline u32 fb_niso_intr_en_clr_mmu_replayable_fault_notify_m(void)
{
	return 0x1U << 27U;
}
static inline u32 fb_niso_intr_en_clr_mmu_replayable_fault_notify_set_f(void)
{
	return 0x8000000U;
}
static inline u32 fb_niso_intr_en_clr_mmu_replayable_fault_overflow_m(void)
{
	return 0x1U << 28U;
}
static inline u32 fb_niso_intr_en_clr_mmu_replayable_fault_overflow_set_f(void)
{
	return 0x10000000U;
}
static inline u32 fb_niso_intr_en_clr_mmu_nonreplayable_fault_notify_m(void)
{
	return 0x1U << 29U;
}
static inline u32 fb_niso_intr_en_clr_mmu_nonreplayable_fault_notify_set_f(void)
{
	return 0x20000000U;
}
static inline u32 fb_niso_intr_en_clr_mmu_nonreplayable_fault_overflow_m(void)
{
	return 0x1U << 30U;
}
static inline u32 fb_niso_intr_en_clr_mmu_nonreplayable_fault_overflow_set_f(void)
{
	return 0x40000000U;
}
static inline u32 fb_niso_intr_en_clr_mmu_other_fault_notify_m(void)
{
	return 0x1U << 31U;
}
static inline u32 fb_niso_intr_en_clr_mmu_other_fault_notify_set_f(void)
{
	return 0x80000000U;
}
static inline u32 fb_niso_intr_en_clr_mmu_non_replay_fault_buffer_v(void)
{
	return 0x00000000U;
}
static inline u32 fb_niso_intr_en_clr_mmu_replay_fault_buffer_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_buffer_lo_r(u32 i)
{
	return 0x00100e24U + i*20U;
}
static inline u32 fb_mmu_fault_buffer_lo__size_1_v(void)
{
	return 0x00000002U;
}
static inline u32 fb_mmu_fault_buffer_lo_addr_mode_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 fb_mmu_fault_buffer_lo_addr_mode_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 fb_mmu_fault_buffer_lo_addr_mode_virtual_v(void)
{
	return 0x00000000U;
}
static inline u32 fb_mmu_fault_buffer_lo_addr_mode_virtual_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_fault_buffer_lo_addr_mode_physical_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_buffer_lo_addr_mode_physical_f(void)
{
	return 0x1U;
}
static inline u32 fb_mmu_fault_buffer_lo_phys_aperture_f(u32 v)
{
	return (v & 0x3U) << 1U;
}
static inline u32 fb_mmu_fault_buffer_lo_phys_aperture_v(u32 r)
{
	return (r >> 1U) & 0x3U;
}
static inline u32 fb_mmu_fault_buffer_lo_phys_aperture_sys_coh_v(void)
{
	return 0x00000002U;
}
static inline u32 fb_mmu_fault_buffer_lo_phys_aperture_sys_coh_f(void)
{
	return 0x4U;
}
static inline u32 fb_mmu_fault_buffer_lo_phys_aperture_sys_nocoh_v(void)
{
	return 0x00000003U;
}
static inline u32 fb_mmu_fault_buffer_lo_phys_aperture_sys_nocoh_f(void)
{
	return 0x6U;
}
static inline u32 fb_mmu_fault_buffer_lo_phys_vol_f(u32 v)
{
	return (v & 0x1U) << 3U;
}
static inline u32 fb_mmu_fault_buffer_lo_phys_vol_v(u32 r)
{
	return (r >> 3U) & 0x1U;
}
static inline u32 fb_mmu_fault_buffer_lo_addr_f(u32 v)
{
	return (v & 0xfffffU) << 12U;
}
static inline u32 fb_mmu_fault_buffer_lo_addr_v(u32 r)
{
	return (r >> 12U) & 0xfffffU;
}
static inline u32 fb_mmu_fault_buffer_hi_r(u32 i)
{
	return 0x00100e28U + i*20U;
}
static inline u32 fb_mmu_fault_buffer_hi__size_1_v(void)
{
	return 0x00000002U;
}
static inline u32 fb_mmu_fault_buffer_hi_addr_f(u32 v)
{
	return (v & 0xffffffffU) << 0U;
}
static inline u32 fb_mmu_fault_buffer_hi_addr_v(u32 r)
{
	return (r >> 0U) & 0xffffffffU;
}
static inline u32 fb_mmu_fault_buffer_get_r(u32 i)
{
	return 0x00100e2cU + i*20U;
}
static inline u32 fb_mmu_fault_buffer_get__size_1_v(void)
{
	return 0x00000002U;
}
static inline u32 fb_mmu_fault_buffer_get_ptr_f(u32 v)
{
	return (v & 0xfffffU) << 0U;
}
static inline u32 fb_mmu_fault_buffer_get_ptr_m(void)
{
	return 0xfffffU << 0U;
}
static inline u32 fb_mmu_fault_buffer_get_ptr_v(u32 r)
{
	return (r >> 0U) & 0xfffffU;
}
static inline u32 fb_mmu_fault_buffer_get_getptr_corrupted_f(u32 v)
{
	return (v & 0x1U) << 30U;
}
static inline u32 fb_mmu_fault_buffer_get_getptr_corrupted_m(void)
{
	return 0x1U << 30U;
}
static inline u32 fb_mmu_fault_buffer_get_getptr_corrupted_clear_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_buffer_get_getptr_corrupted_clear_f(void)
{
	return 0x40000000U;
}
static inline u32 fb_mmu_fault_buffer_get_overflow_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 fb_mmu_fault_buffer_get_overflow_m(void)
{
	return 0x1U << 31U;
}
static inline u32 fb_mmu_fault_buffer_get_overflow_clear_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_buffer_get_overflow_clear_f(void)
{
	return 0x80000000U;
}
static inline u32 fb_mmu_fault_buffer_put_r(u32 i)
{
	return 0x00100e30U + i*20U;
}
static inline u32 fb_mmu_fault_buffer_put__size_1_v(void)
{
	return 0x00000002U;
}
static inline u32 fb_mmu_fault_buffer_put_ptr_f(u32 v)
{
	return (v & 0xfffffU) << 0U;
}
static inline u32 fb_mmu_fault_buffer_put_ptr_v(u32 r)
{
	return (r >> 0U) & 0xfffffU;
}
static inline u32 fb_mmu_fault_buffer_put_getptr_corrupted_f(u32 v)
{
	return (v & 0x1U) << 30U;
}
static inline u32 fb_mmu_fault_buffer_put_getptr_corrupted_v(u32 r)
{
	return (r >> 30U) & 0x1U;
}
static inline u32 fb_mmu_fault_buffer_put_getptr_corrupted_yes_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_buffer_put_getptr_corrupted_yes_f(void)
{
	return 0x40000000U;
}
static inline u32 fb_mmu_fault_buffer_put_getptr_corrupted_no_v(void)
{
	return 0x00000000U;
}
static inline u32 fb_mmu_fault_buffer_put_getptr_corrupted_no_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_fault_buffer_put_overflow_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 fb_mmu_fault_buffer_put_overflow_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 fb_mmu_fault_buffer_put_overflow_yes_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_buffer_put_overflow_yes_f(void)
{
	return 0x80000000U;
}
static inline u32 fb_mmu_fault_buffer_size_r(u32 i)
{
	return 0x00100e34U + i*20U;
}
static inline u32 fb_mmu_fault_buffer_size__size_1_v(void)
{
	return 0x00000002U;
}
static inline u32 fb_mmu_fault_buffer_size_val_f(u32 v)
{
	return (v & 0xfffffU) << 0U;
}
static inline u32 fb_mmu_fault_buffer_size_val_v(u32 r)
{
	return (r >> 0U) & 0xfffffU;
}
static inline u32 fb_mmu_fault_buffer_size_overflow_intr_f(u32 v)
{
	return (v & 0x1U) << 29U;
}
static inline u32 fb_mmu_fault_buffer_size_overflow_intr_v(u32 r)
{
	return (r >> 29U) & 0x1U;
}
static inline u32 fb_mmu_fault_buffer_size_overflow_intr_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_buffer_size_overflow_intr_enable_f(void)
{
	return 0x20000000U;
}
static inline u32 fb_mmu_fault_buffer_size_set_default_f(u32 v)
{
	return (v & 0x1U) << 30U;
}
static inline u32 fb_mmu_fault_buffer_size_set_default_v(u32 r)
{
	return (r >> 30U) & 0x1U;
}
static inline u32 fb_mmu_fault_buffer_size_set_default_yes_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_buffer_size_set_default_yes_f(void)
{
	return 0x40000000U;
}
static inline u32 fb_mmu_fault_buffer_size_enable_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 fb_mmu_fault_buffer_size_enable_m(void)
{
	return 0x1U << 31U;
}
static inline u32 fb_mmu_fault_buffer_size_enable_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 fb_mmu_fault_buffer_size_enable_true_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_buffer_size_enable_true_f(void)
{
	return 0x80000000U;
}
static inline u32 fb_mmu_fault_addr_lo_r(void)
{
	return 0x00100e4cU;
}
static inline u32 fb_mmu_fault_addr_lo_phys_aperture_f(u32 v)
{
	return (v & 0x3U) << 0U;
}
static inline u32 fb_mmu_fault_addr_lo_phys_aperture_v(u32 r)
{
	return (r >> 0U) & 0x3U;
}
static inline u32 fb_mmu_fault_addr_lo_phys_aperture_sys_coh_v(void)
{
	return 0x00000002U;
}
static inline u32 fb_mmu_fault_addr_lo_phys_aperture_sys_coh_f(void)
{
	return 0x2U;
}
static inline u32 fb_mmu_fault_addr_lo_phys_aperture_sys_nocoh_v(void)
{
	return 0x00000003U;
}
static inline u32 fb_mmu_fault_addr_lo_phys_aperture_sys_nocoh_f(void)
{
	return 0x3U;
}
static inline u32 fb_mmu_fault_addr_lo_addr_f(u32 v)
{
	return (v & 0xfffffU) << 12U;
}
static inline u32 fb_mmu_fault_addr_lo_addr_v(u32 r)
{
	return (r >> 12U) & 0xfffffU;
}
static inline u32 fb_mmu_fault_addr_hi_r(void)
{
	return 0x00100e50U;
}
static inline u32 fb_mmu_fault_addr_hi_addr_f(u32 v)
{
	return (v & 0xffffffffU) << 0U;
}
static inline u32 fb_mmu_fault_addr_hi_addr_v(u32 r)
{
	return (r >> 0U) & 0xffffffffU;
}
static inline u32 fb_mmu_fault_inst_lo_r(void)
{
	return 0x00100e54U;
}
static inline u32 fb_mmu_fault_inst_lo_engine_id_v(u32 r)
{
	return (r >> 0U) & 0x1ffU;
}
static inline u32 fb_mmu_fault_inst_lo_aperture_v(u32 r)
{
	return (r >> 10U) & 0x3U;
}
static inline u32 fb_mmu_fault_inst_lo_aperture_sys_coh_v(void)
{
	return 0x00000002U;
}
static inline u32 fb_mmu_fault_inst_lo_aperture_sys_nocoh_v(void)
{
	return 0x00000003U;
}
static inline u32 fb_mmu_fault_inst_lo_addr_f(u32 v)
{
	return (v & 0xfffffU) << 12U;
}
static inline u32 fb_mmu_fault_inst_lo_addr_v(u32 r)
{
	return (r >> 12U) & 0xfffffU;
}
static inline u32 fb_mmu_fault_inst_hi_r(void)
{
	return 0x00100e58U;
}
static inline u32 fb_mmu_fault_inst_hi_addr_v(u32 r)
{
	return (r >> 0U) & 0xffffffffU;
}
static inline u32 fb_mmu_fault_info_r(void)
{
	return 0x00100e5cU;
}
static inline u32 fb_mmu_fault_info_fault_type_v(u32 r)
{
	return (r >> 0U) & 0x1fU;
}
static inline u32 fb_mmu_fault_info_replayable_fault_v(u32 r)
{
	return (r >> 7U) & 0x1U;
}
static inline u32 fb_mmu_fault_info_client_v(u32 r)
{
	return (r >> 8U) & 0x7fU;
}
static inline u32 fb_mmu_fault_info_access_type_v(u32 r)
{
	return (r >> 16U) & 0xfU;
}
static inline u32 fb_mmu_fault_info_client_type_v(u32 r)
{
	return (r >> 20U) & 0x1U;
}
static inline u32 fb_mmu_fault_info_gpc_id_v(u32 r)
{
	return (r >> 24U) & 0x1fU;
}
static inline u32 fb_mmu_fault_info_protected_mode_v(u32 r)
{
	return (r >> 29U) & 0x1U;
}
static inline u32 fb_mmu_fault_info_replayable_fault_en_v(u32 r)
{
	return (r >> 30U) & 0x1U;
}
static inline u32 fb_mmu_fault_info_valid_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 fb_mmu_fault_status_r(void)
{
	return 0x00100e60U;
}
static inline u32 fb_mmu_fault_status_dropped_bar1_phys_m(void)
{
	return 0x1U << 0U;
}
static inline u32 fb_mmu_fault_status_dropped_bar1_phys_set_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_dropped_bar1_phys_set_f(void)
{
	return 0x1U;
}
static inline u32 fb_mmu_fault_status_dropped_bar1_phys_clear_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_dropped_bar1_phys_clear_f(void)
{
	return 0x1U;
}
static inline u32 fb_mmu_fault_status_dropped_bar1_virt_m(void)
{
	return 0x1U << 1U;
}
static inline u32 fb_mmu_fault_status_dropped_bar1_virt_set_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_dropped_bar1_virt_set_f(void)
{
	return 0x2U;
}
static inline u32 fb_mmu_fault_status_dropped_bar1_virt_clear_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_dropped_bar1_virt_clear_f(void)
{
	return 0x2U;
}
static inline u32 fb_mmu_fault_status_dropped_bar2_phys_m(void)
{
	return 0x1U << 2U;
}
static inline u32 fb_mmu_fault_status_dropped_bar2_phys_set_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_dropped_bar2_phys_set_f(void)
{
	return 0x4U;
}
static inline u32 fb_mmu_fault_status_dropped_bar2_phys_clear_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_dropped_bar2_phys_clear_f(void)
{
	return 0x4U;
}
static inline u32 fb_mmu_fault_status_dropped_bar2_virt_m(void)
{
	return 0x1U << 3U;
}
static inline u32 fb_mmu_fault_status_dropped_bar2_virt_set_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_dropped_bar2_virt_set_f(void)
{
	return 0x8U;
}
static inline u32 fb_mmu_fault_status_dropped_bar2_virt_clear_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_dropped_bar2_virt_clear_f(void)
{
	return 0x8U;
}
static inline u32 fb_mmu_fault_status_dropped_ifb_phys_m(void)
{
	return 0x1U << 4U;
}
static inline u32 fb_mmu_fault_status_dropped_ifb_phys_set_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_dropped_ifb_phys_set_f(void)
{
	return 0x10U;
}
static inline u32 fb_mmu_fault_status_dropped_ifb_phys_clear_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_dropped_ifb_phys_clear_f(void)
{
	return 0x10U;
}
static inline u32 fb_mmu_fault_status_dropped_ifb_virt_m(void)
{
	return 0x1U << 5U;
}
static inline u32 fb_mmu_fault_status_dropped_ifb_virt_set_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_dropped_ifb_virt_set_f(void)
{
	return 0x20U;
}
static inline u32 fb_mmu_fault_status_dropped_ifb_virt_clear_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_dropped_ifb_virt_clear_f(void)
{
	return 0x20U;
}
static inline u32 fb_mmu_fault_status_dropped_other_phys_m(void)
{
	return 0x1U << 6U;
}
static inline u32 fb_mmu_fault_status_dropped_other_phys_set_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_dropped_other_phys_set_f(void)
{
	return 0x40U;
}
static inline u32 fb_mmu_fault_status_dropped_other_phys_clear_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_dropped_other_phys_clear_f(void)
{
	return 0x40U;
}
static inline u32 fb_mmu_fault_status_dropped_other_virt_m(void)
{
	return 0x1U << 7U;
}
static inline u32 fb_mmu_fault_status_dropped_other_virt_set_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_dropped_other_virt_set_f(void)
{
	return 0x80U;
}
static inline u32 fb_mmu_fault_status_dropped_other_virt_clear_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_dropped_other_virt_clear_f(void)
{
	return 0x80U;
}
static inline u32 fb_mmu_fault_status_replayable_m(void)
{
	return 0x1U << 8U;
}
static inline u32 fb_mmu_fault_status_replayable_set_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_replayable_set_f(void)
{
	return 0x100U;
}
static inline u32 fb_mmu_fault_status_replayable_reset_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_fault_status_non_replayable_m(void)
{
	return 0x1U << 9U;
}
static inline u32 fb_mmu_fault_status_non_replayable_set_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_non_replayable_set_f(void)
{
	return 0x200U;
}
static inline u32 fb_mmu_fault_status_non_replayable_reset_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_fault_status_replayable_error_m(void)
{
	return 0x1U << 10U;
}
static inline u32 fb_mmu_fault_status_replayable_error_set_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_replayable_error_set_f(void)
{
	return 0x400U;
}
static inline u32 fb_mmu_fault_status_replayable_error_reset_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_fault_status_non_replayable_error_m(void)
{
	return 0x1U << 11U;
}
static inline u32 fb_mmu_fault_status_non_replayable_error_set_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_non_replayable_error_set_f(void)
{
	return 0x800U;
}
static inline u32 fb_mmu_fault_status_non_replayable_error_reset_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_fault_status_replayable_overflow_m(void)
{
	return 0x1U << 12U;
}
static inline u32 fb_mmu_fault_status_replayable_overflow_set_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_replayable_overflow_set_f(void)
{
	return 0x1000U;
}
static inline u32 fb_mmu_fault_status_replayable_overflow_reset_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_fault_status_non_replayable_overflow_m(void)
{
	return 0x1U << 13U;
}
static inline u32 fb_mmu_fault_status_non_replayable_overflow_set_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_non_replayable_overflow_set_f(void)
{
	return 0x2000U;
}
static inline u32 fb_mmu_fault_status_non_replayable_overflow_reset_f(void)
{
	return 0x0U;
}
static inline u32 fb_mmu_fault_status_replayable_getptr_corrupted_m(void)
{
	return 0x1U << 14U;
}
static inline u32 fb_mmu_fault_status_replayable_getptr_corrupted_set_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_replayable_getptr_corrupted_set_f(void)
{
	return 0x4000U;
}
static inline u32 fb_mmu_fault_status_non_replayable_getptr_corrupted_m(void)
{
	return 0x1U << 15U;
}
static inline u32 fb_mmu_fault_status_non_replayable_getptr_corrupted_set_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_non_replayable_getptr_corrupted_set_f(void)
{
	return 0x8000U;
}
static inline u32 fb_mmu_fault_status_busy_m(void)
{
	return 0x1U << 30U;
}
static inline u32 fb_mmu_fault_status_busy_true_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_busy_true_f(void)
{
	return 0x40000000U;
}
static inline u32 fb_mmu_fault_status_valid_m(void)
{
	return 0x1U << 31U;
}
static inline u32 fb_mmu_fault_status_valid_set_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_valid_set_f(void)
{
	return 0x80000000U;
}
static inline u32 fb_mmu_fault_status_valid_clear_v(void)
{
	return 0x00000001U;
}
static inline u32 fb_mmu_fault_status_valid_clear_f(void)
{
	return 0x80000000U;
}
static inline u32 fb_mmu_local_memory_range_r(void)
{
	return 0x00100ce0U;
}
static inline u32 fb_mmu_local_memory_range_lower_scale_v(u32 r)
{
	return (r >> 0U) & 0xfU;
}
static inline u32 fb_mmu_local_memory_range_lower_mag_v(u32 r)
{
	return (r >> 4U) & 0x3fU;
}
static inline u32 fb_mmu_local_memory_range_ecc_mode_v(u32 r)
{
	return (r >> 30U) & 0x1U;
}
static inline u32 fb_niso_scrub_status_r(void)
{
	return 0x00100b20U;
}
static inline u32 fb_niso_scrub_status_flag_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 fb_mmu_priv_level_mask_r(void)
{
	return 0x00100cdcU;
}
static inline u32 fb_mmu_priv_level_mask_write_violation_f(u32 v)
{
	return (v & 0x1U) << 7U;
}
static inline u32 fb_mmu_priv_level_mask_write_violation_m(void)
{
	return 0x1U << 7U;
}
static inline u32 fb_mmu_priv_level_mask_write_violation_v(u32 r)
{
	return (r >> 7U) & 0x1U;
}
static inline u32 fb_hshub_config0_r(void)
{
	return 0x001fbc00U;
}
static inline u32 fb_hshub_config0_sysmem_nvlink_mask_f(u32 v)
{
	return (v & 0xffffU) << 0U;
}
static inline u32 fb_hshub_config0_sysmem_nvlink_mask_m(void)
{
	return 0xffffU << 0U;
}
static inline u32 fb_hshub_config0_sysmem_nvlink_mask_v(u32 r)
{
	return (r >> 0U) & 0xffffU;
}
static inline u32 fb_hshub_config0_peer_pcie_mask_f(u32 v)
{
	return (v & 0xffffU) << 16U;
}
static inline u32 fb_hshub_config0_peer_pcie_mask_v(u32 r)
{
	return (r >> 16U) & 0xffffU;
}
static inline u32 fb_hshub_config1_r(void)
{
	return 0x001fbc04U;
}
static inline u32 fb_hshub_config1_peer_0_nvlink_mask_f(u32 v)
{
	return (v & 0xffU) << 0U;
}
static inline u32 fb_hshub_config1_peer_0_nvlink_mask_v(u32 r)
{
	return (r >> 0U) & 0xffU;
}
static inline u32 fb_hshub_config1_peer_1_nvlink_mask_f(u32 v)
{
	return (v & 0xffU) << 8U;
}
static inline u32 fb_hshub_config1_peer_1_nvlink_mask_v(u32 r)
{
	return (r >> 8U) & 0xffU;
}
static inline u32 fb_hshub_config1_peer_2_nvlink_mask_f(u32 v)
{
	return (v & 0xffU) << 16U;
}
static inline u32 fb_hshub_config1_peer_2_nvlink_mask_v(u32 r)
{
	return (r >> 16U) & 0xffU;
}
static inline u32 fb_hshub_config1_peer_3_nvlink_mask_f(u32 v)
{
	return (v & 0xffU) << 24U;
}
static inline u32 fb_hshub_config1_peer_3_nvlink_mask_v(u32 r)
{
	return (r >> 24U) & 0xffU;
}
static inline u32 fb_hshub_config2_r(void)
{
	return 0x001fbc08U;
}
static inline u32 fb_hshub_config2_peer_4_nvlink_mask_f(u32 v)
{
	return (v & 0xffU) << 0U;
}
static inline u32 fb_hshub_config2_peer_4_nvlink_mask_v(u32 r)
{
	return (r >> 0U) & 0xffU;
}
static inline u32 fb_hshub_config2_peer_5_nvlink_mask_f(u32 v)
{
	return (v & 0xffU) << 8U;
}
static inline u32 fb_hshub_config2_peer_5_nvlink_mask_v(u32 r)
{
	return (r >> 8U) & 0xffU;
}
static inline u32 fb_hshub_config2_peer_6_nvlink_mask_f(u32 v)
{
	return (v & 0xffU) << 16U;
}
static inline u32 fb_hshub_config2_peer_6_nvlink_mask_v(u32 r)
{
	return (r >> 16U) & 0xffU;
}
static inline u32 fb_hshub_config2_peer_7_nvlink_mask_f(u32 v)
{
	return (v & 0xffU) << 24U;
}
static inline u32 fb_hshub_config2_peer_7_nvlink_mask_v(u32 r)
{
	return (r >> 24U) & 0xffU;
}
static inline u32 fb_hshub_config6_r(void)
{
	return 0x001fbc18U;
}
static inline u32 fb_hshub_config7_r(void)
{
	return 0x001fbc1cU;
}
static inline u32 fb_hshub_config7_nvlink_logical_0_physical_portmap_f(u32 v)
{
	return (v & 0xfU) << 0U;
}
static inline u32 fb_hshub_config7_nvlink_logical_0_physical_portmap_v(u32 r)
{
	return (r >> 0U) & 0xfU;
}
static inline u32 fb_hshub_config7_nvlink_logical_1_physical_portmap_f(u32 v)
{
	return (v & 0xfU) << 4U;
}
static inline u32 fb_hshub_config7_nvlink_logical_1_physical_portmap_v(u32 r)
{
	return (r >> 4U) & 0xfU;
}
static inline u32 fb_hshub_config7_nvlink_logical_2_physical_portmap_f(u32 v)
{
	return (v & 0xfU) << 8U;
}
static inline u32 fb_hshub_config7_nvlink_logical_2_physical_portmap_v(u32 r)
{
	return (r >> 8U) & 0xfU;
}
static inline u32 fb_hshub_config7_nvlink_logical_3_physical_portmap_f(u32 v)
{
	return (v & 0xfU) << 12U;
}
static inline u32 fb_hshub_config7_nvlink_logical_3_physical_portmap_v(u32 r)
{
	return (r >> 12U) & 0xfU;
}
static inline u32 fb_hshub_config7_nvlink_logical_4_physical_portmap_f(u32 v)
{
	return (v & 0xfU) << 16U;
}
static inline u32 fb_hshub_config7_nvlink_logical_4_physical_portmap_v(u32 r)
{
	return (r >> 16U) & 0xfU;
}
static inline u32 fb_hshub_config7_nvlink_logical_5_physical_portmap_f(u32 v)
{
	return (v & 0xfU) << 20U;
}
static inline u32 fb_hshub_config7_nvlink_logical_5_physical_portmap_v(u32 r)
{
	return (r >> 20U) & 0xfU;
}
static inline u32 fb_hshub_config7_nvlink_logical_6_physical_portmap_f(u32 v)
{
	return (v & 0xfU) << 24U;
}
static inline u32 fb_hshub_config7_nvlink_logical_6_physical_portmap_v(u32 r)
{
	return (r >> 24U) & 0xfU;
}
static inline u32 fb_hshub_config7_nvlink_logical_7_physical_portmap_f(u32 v)
{
	return (v & 0xfU) << 28U;
}
static inline u32 fb_hshub_config7_nvlink_logical_7_physical_portmap_v(u32 r)
{
	return (r >> 28U) & 0xfU;
}
static inline u32 fb_hshub_nvl_cfg_priv_level_mask_r(void)
{
	return 0x001fbc50U;
}
static inline u32 fb_hshub_nvl_cfg_priv_level_mask_write_protection_f(u32 v)
{
	return (v & 0x7U) << 4U;
}
static inline u32 fb_hshub_nvl_cfg_priv_level_mask_write_protection_v(u32 r)
{
	return (r >> 4U) & 0x7U;
}
#endif
