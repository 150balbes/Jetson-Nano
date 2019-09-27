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
#ifndef _hw_ltc_gv100_h_
#define _hw_ltc_gv100_h_

static inline u32 ltc_pltcg_base_v(void)
{
	return 0x00140000U;
}
static inline u32 ltc_pltcg_extent_v(void)
{
	return 0x0017ffffU;
}
static inline u32 ltc_ltc0_ltss_v(void)
{
	return 0x00140200U;
}
static inline u32 ltc_ltc0_lts0_v(void)
{
	return 0x00140400U;
}
static inline u32 ltc_ltcs_ltss_v(void)
{
	return 0x0017e200U;
}
static inline u32 ltc_ltcs_lts0_cbc_ctrl1_r(void)
{
	return 0x0014046cU;
}
static inline u32 ltc_ltc0_lts0_dstg_cfg0_r(void)
{
	return 0x00140518U;
}
static inline u32 ltc_ltcs_ltss_dstg_cfg0_r(void)
{
	return 0x0017e318U;
}
static inline u32 ltc_ltcs_ltss_dstg_cfg0_vdc_4to2_disable_m(void)
{
	return 0x1U << 15U;
}
static inline u32 ltc_ltc0_lts0_tstg_cfg1_r(void)
{
	return 0x00140494U;
}
static inline u32 ltc_ltc0_lts0_tstg_cfg1_active_ways_v(u32 r)
{
	return (r >> 0U) & 0xffffU;
}
static inline u32 ltc_ltc0_lts0_tstg_cfg1_active_sets_v(u32 r)
{
	return (r >> 16U) & 0x3U;
}
static inline u32 ltc_ltc0_lts0_tstg_cfg1_active_sets_all_v(void)
{
	return 0x00000000U;
}
static inline u32 ltc_ltc0_lts0_tstg_cfg1_active_sets_half_v(void)
{
	return 0x00000001U;
}
static inline u32 ltc_ltc0_lts0_tstg_cfg1_active_sets_quarter_v(void)
{
	return 0x00000002U;
}
static inline u32 ltc_ltcs_ltss_cbc_ctrl1_r(void)
{
	return 0x0017e26cU;
}
static inline u32 ltc_ltcs_ltss_cbc_ctrl1_clean_active_f(void)
{
	return 0x1U;
}
static inline u32 ltc_ltcs_ltss_cbc_ctrl1_invalidate_active_f(void)
{
	return 0x2U;
}
static inline u32 ltc_ltcs_ltss_cbc_ctrl1_clear_v(u32 r)
{
	return (r >> 2U) & 0x1U;
}
static inline u32 ltc_ltcs_ltss_cbc_ctrl1_clear_active_v(void)
{
	return 0x00000001U;
}
static inline u32 ltc_ltcs_ltss_cbc_ctrl1_clear_active_f(void)
{
	return 0x4U;
}
static inline u32 ltc_ltc0_lts0_cbc_ctrl1_r(void)
{
	return 0x0014046cU;
}
static inline u32 ltc_ltcs_ltss_cbc_ctrl2_r(void)
{
	return 0x0017e270U;
}
static inline u32 ltc_ltcs_ltss_cbc_ctrl2_clear_lower_bound_f(u32 v)
{
	return (v & 0x3ffffU) << 0U;
}
static inline u32 ltc_ltcs_ltss_cbc_ctrl3_r(void)
{
	return 0x0017e274U;
}
static inline u32 ltc_ltcs_ltss_cbc_ctrl3_clear_upper_bound_f(u32 v)
{
	return (v & 0x3ffffU) << 0U;
}
static inline u32 ltc_ltcs_ltss_cbc_ctrl3_clear_upper_bound_init_v(void)
{
	return 0x0003ffffU;
}
static inline u32 ltc_ltcs_ltss_cbc_base_r(void)
{
	return 0x0017e278U;
}
static inline u32 ltc_ltcs_ltss_cbc_base_alignment_shift_v(void)
{
	return 0x0000000bU;
}
static inline u32 ltc_ltcs_ltss_cbc_base_address_v(u32 r)
{
	return (r >> 0U) & 0x3ffffffU;
}
static inline u32 ltc_ltcs_ltss_cbc_num_active_ltcs_r(void)
{
	return 0x0017e27cU;
}
static inline u32 ltc_ltcs_ltss_cbc_num_active_ltcs__v(u32 r)
{
	return (r >> 0U) & 0x1fU;
}
static inline u32 ltc_ltcs_ltss_cbc_num_active_ltcs_nvlink_peer_through_l2_f(u32 v)
{
	return (v & 0x1U) << 24U;
}
static inline u32 ltc_ltcs_ltss_cbc_num_active_ltcs_nvlink_peer_through_l2_v(u32 r)
{
	return (r >> 24U) & 0x1U;
}
static inline u32 ltc_ltcs_ltss_cbc_num_active_ltcs_serialize_f(u32 v)
{
	return (v & 0x1U) << 25U;
}
static inline u32 ltc_ltcs_ltss_cbc_num_active_ltcs_serialize_v(u32 r)
{
	return (r >> 25U) & 0x1U;
}
static inline u32 ltc_ltcs_misc_ltc_num_active_ltcs_r(void)
{
	return 0x0017e000U;
}
static inline u32 ltc_ltcs_ltss_cbc_param_r(void)
{
	return 0x0017e280U;
}
static inline u32 ltc_ltcs_ltss_cbc_param_comptags_per_cache_line_v(u32 r)
{
	return (r >> 0U) & 0xffffU;
}
static inline u32 ltc_ltcs_ltss_cbc_param_cache_line_size_v(u32 r)
{
	return (r >> 24U) & 0xfU;
}
static inline u32 ltc_ltcs_ltss_cbc_param_slices_per_ltc_v(u32 r)
{
	return (r >> 28U) & 0xfU;
}
static inline u32 ltc_ltcs_ltss_cbc_param2_r(void)
{
	return 0x0017e3f4U;
}
static inline u32 ltc_ltcs_ltss_cbc_param2_gobs_per_comptagline_per_slice_v(u32 r)
{
	return (r >> 0U) & 0xffffU;
}
static inline u32 ltc_ltcs_ltss_tstg_set_mgmt_r(void)
{
	return 0x0017e2acU;
}
static inline u32 ltc_ltcs_ltss_tstg_set_mgmt_max_ways_evict_last_f(u32 v)
{
	return (v & 0x1fU) << 16U;
}
static inline u32 ltc_ltcs_ltss_dstg_zbc_index_r(void)
{
	return 0x0017e338U;
}
static inline u32 ltc_ltcs_ltss_dstg_zbc_index_address_f(u32 v)
{
	return (v & 0xfU) << 0U;
}
static inline u32 ltc_ltcs_ltss_dstg_zbc_color_clear_value_r(u32 i)
{
	return 0x0017e33cU + i*4U;
}
static inline u32 ltc_ltcs_ltss_dstg_zbc_color_clear_value__size_1_v(void)
{
	return 0x00000004U;
}
static inline u32 ltc_ltcs_ltss_dstg_zbc_depth_clear_value_r(void)
{
	return 0x0017e34cU;
}
static inline u32 ltc_ltcs_ltss_dstg_zbc_depth_clear_value_field_s(void)
{
	return 32U;
}
static inline u32 ltc_ltcs_ltss_dstg_zbc_depth_clear_value_field_f(u32 v)
{
	return (v & 0xffffffffU) << 0U;
}
static inline u32 ltc_ltcs_ltss_dstg_zbc_depth_clear_value_field_m(void)
{
	return 0xffffffffU << 0U;
}
static inline u32 ltc_ltcs_ltss_dstg_zbc_depth_clear_value_field_v(u32 r)
{
	return (r >> 0U) & 0xffffffffU;
}
static inline u32 ltc_ltcs_ltss_dstg_zbc_stencil_clear_value_r(void)
{
	return 0x0017e204U;
}
static inline u32 ltc_ltcs_ltss_dstg_zbc_stencil_clear_value_field_s(void)
{
	return 8U;
}
static inline u32 ltc_ltcs_ltss_dstg_zbc_stencil_clear_value_field_f(u32 v)
{
	return (v & 0xffU) << 0U;
}
static inline u32 ltc_ltcs_ltss_dstg_zbc_stencil_clear_value_field_m(void)
{
	return 0xffU << 0U;
}
static inline u32 ltc_ltcs_ltss_dstg_zbc_stencil_clear_value_field_v(u32 r)
{
	return (r >> 0U) & 0xffU;
}
static inline u32 ltc_ltcs_ltss_tstg_set_mgmt_2_r(void)
{
	return 0x0017e2b0U;
}
static inline u32 ltc_ltcs_ltss_tstg_set_mgmt_2_l2_bypass_mode_enabled_f(void)
{
	return 0x10000000U;
}
static inline u32 ltc_ltcs_ltss_g_elpg_r(void)
{
	return 0x0017e214U;
}
static inline u32 ltc_ltcs_ltss_g_elpg_flush_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 ltc_ltcs_ltss_g_elpg_flush_pending_v(void)
{
	return 0x00000001U;
}
static inline u32 ltc_ltcs_ltss_g_elpg_flush_pending_f(void)
{
	return 0x1U;
}
static inline u32 ltc_ltc0_ltss_g_elpg_r(void)
{
	return 0x00140214U;
}
static inline u32 ltc_ltc0_ltss_g_elpg_flush_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 ltc_ltc0_ltss_g_elpg_flush_pending_v(void)
{
	return 0x00000001U;
}
static inline u32 ltc_ltc0_ltss_g_elpg_flush_pending_f(void)
{
	return 0x1U;
}
static inline u32 ltc_ltc1_ltss_g_elpg_r(void)
{
	return 0x00142214U;
}
static inline u32 ltc_ltc1_ltss_g_elpg_flush_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 ltc_ltc1_ltss_g_elpg_flush_pending_v(void)
{
	return 0x00000001U;
}
static inline u32 ltc_ltc1_ltss_g_elpg_flush_pending_f(void)
{
	return 0x1U;
}
static inline u32 ltc_ltcs_ltss_intr_r(void)
{
	return 0x0017e20cU;
}
static inline u32 ltc_ltcs_ltss_intr_ecc_sec_error_pending_f(void)
{
	return 0x100U;
}
static inline u32 ltc_ltcs_ltss_intr_ecc_ded_error_pending_f(void)
{
	return 0x200U;
}
static inline u32 ltc_ltcs_ltss_intr_en_evicted_cb_m(void)
{
	return 0x1U << 20U;
}
static inline u32 ltc_ltcs_ltss_intr_en_illegal_compstat_m(void)
{
	return 0x1U << 21U;
}
static inline u32 ltc_ltcs_ltss_intr_en_illegal_compstat_enabled_f(void)
{
	return 0x200000U;
}
static inline u32 ltc_ltcs_ltss_intr_en_illegal_compstat_disabled_f(void)
{
	return 0x0U;
}
static inline u32 ltc_ltcs_ltss_intr_en_illegal_compstat_access_m(void)
{
	return 0x1U << 30U;
}
static inline u32 ltc_ltcs_ltss_intr_en_ecc_sec_error_enabled_f(void)
{
	return 0x1000000U;
}
static inline u32 ltc_ltcs_ltss_intr_en_ecc_ded_error_enabled_f(void)
{
	return 0x2000000U;
}
static inline u32 ltc_ltc0_lts0_intr_r(void)
{
	return 0x0014040cU;
}
static inline u32 ltc_ltc0_lts0_dstg_ecc_report_r(void)
{
	return 0x0014051cU;
}
static inline u32 ltc_ltc0_lts0_dstg_ecc_report_sec_count_m(void)
{
	return 0xffU << 0U;
}
static inline u32 ltc_ltc0_lts0_dstg_ecc_report_sec_count_v(u32 r)
{
	return (r >> 0U) & 0xffU;
}
static inline u32 ltc_ltc0_lts0_dstg_ecc_report_ded_count_m(void)
{
	return 0xffU << 16U;
}
static inline u32 ltc_ltc0_lts0_dstg_ecc_report_ded_count_v(u32 r)
{
	return (r >> 16U) & 0xffU;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt0_r(void)
{
	return 0x0017e2a0U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt0_invalidate_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt0_invalidate_pending_v(void)
{
	return 0x00000001U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt0_invalidate_pending_f(void)
{
	return 0x1U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt0_max_cycles_between_invalidates_v(u32 r)
{
	return (r >> 8U) & 0xfU;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt0_max_cycles_between_invalidates_3_v(void)
{
	return 0x00000003U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt0_max_cycles_between_invalidates_3_f(void)
{
	return 0x300U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt0_invalidate_evict_last_class_v(u32 r)
{
	return (r >> 28U) & 0x1U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt0_invalidate_evict_last_class_true_v(void)
{
	return 0x00000001U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt0_invalidate_evict_last_class_true_f(void)
{
	return 0x10000000U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt0_invalidate_evict_normal_class_v(u32 r)
{
	return (r >> 29U) & 0x1U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt0_invalidate_evict_normal_class_true_v(void)
{
	return 0x00000001U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt0_invalidate_evict_normal_class_true_f(void)
{
	return 0x20000000U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt0_invalidate_evict_first_class_v(u32 r)
{
	return (r >> 30U) & 0x1U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt0_invalidate_evict_first_class_true_v(void)
{
	return 0x00000001U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt0_invalidate_evict_first_class_true_f(void)
{
	return 0x40000000U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt1_r(void)
{
	return 0x0017e2a4U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt1_clean_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt1_clean_pending_v(void)
{
	return 0x00000001U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt1_clean_pending_f(void)
{
	return 0x1U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt1_max_cycles_between_cleans_v(u32 r)
{
	return (r >> 8U) & 0xfU;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt1_max_cycles_between_cleans_3_v(void)
{
	return 0x00000003U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt1_max_cycles_between_cleans_3_f(void)
{
	return 0x300U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt1_clean_wait_for_fb_to_pull_v(u32 r)
{
	return (r >> 16U) & 0x1U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt1_clean_wait_for_fb_to_pull_true_v(void)
{
	return 0x00000001U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt1_clean_wait_for_fb_to_pull_true_f(void)
{
	return 0x10000U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt1_clean_evict_last_class_v(u32 r)
{
	return (r >> 28U) & 0x1U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt1_clean_evict_last_class_true_v(void)
{
	return 0x00000001U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt1_clean_evict_last_class_true_f(void)
{
	return 0x10000000U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt1_clean_evict_normal_class_v(u32 r)
{
	return (r >> 29U) & 0x1U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt1_clean_evict_normal_class_true_v(void)
{
	return 0x00000001U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt1_clean_evict_normal_class_true_f(void)
{
	return 0x20000000U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt1_clean_evict_first_class_v(u32 r)
{
	return (r >> 30U) & 0x1U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt1_clean_evict_first_class_true_v(void)
{
	return 0x00000001U;
}
static inline u32 ltc_ltcs_ltss_tstg_cmgmt1_clean_evict_first_class_true_f(void)
{
	return 0x40000000U;
}
static inline u32 ltc_ltc0_ltss_tstg_cmgmt0_r(void)
{
	return 0x001402a0U;
}
static inline u32 ltc_ltc0_ltss_tstg_cmgmt0_invalidate_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 ltc_ltc0_ltss_tstg_cmgmt0_invalidate_pending_v(void)
{
	return 0x00000001U;
}
static inline u32 ltc_ltc0_ltss_tstg_cmgmt0_invalidate_pending_f(void)
{
	return 0x1U;
}
static inline u32 ltc_ltc0_ltss_tstg_cmgmt1_r(void)
{
	return 0x001402a4U;
}
static inline u32 ltc_ltc0_ltss_tstg_cmgmt1_clean_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 ltc_ltc0_ltss_tstg_cmgmt1_clean_pending_v(void)
{
	return 0x00000001U;
}
static inline u32 ltc_ltc0_ltss_tstg_cmgmt1_clean_pending_f(void)
{
	return 0x1U;
}
static inline u32 ltc_ltc1_ltss_tstg_cmgmt0_r(void)
{
	return 0x001422a0U;
}
static inline u32 ltc_ltc1_ltss_tstg_cmgmt0_invalidate_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 ltc_ltc1_ltss_tstg_cmgmt0_invalidate_pending_v(void)
{
	return 0x00000001U;
}
static inline u32 ltc_ltc1_ltss_tstg_cmgmt0_invalidate_pending_f(void)
{
	return 0x1U;
}
static inline u32 ltc_ltc1_ltss_tstg_cmgmt1_r(void)
{
	return 0x001422a4U;
}
static inline u32 ltc_ltc1_ltss_tstg_cmgmt1_clean_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 ltc_ltc1_ltss_tstg_cmgmt1_clean_pending_v(void)
{
	return 0x00000001U;
}
static inline u32 ltc_ltc1_ltss_tstg_cmgmt1_clean_pending_f(void)
{
	return 0x1U;
}
static inline u32 ltc_ltc0_lts0_tstg_info_1_r(void)
{
	return 0x0014058cU;
}
static inline u32 ltc_ltc0_lts0_tstg_info_1_slice_size_in_kb_v(u32 r)
{
	return (r >> 0U) & 0xffffU;
}
static inline u32 ltc_ltc0_lts0_tstg_info_1_slices_per_l2_v(u32 r)
{
	return (r >> 16U) & 0x1fU;
}
#endif
