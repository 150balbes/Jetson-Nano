/*
 * Copyright (c) 2014-2020, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _hw_gmmu_gp10b_h_
#define _hw_gmmu_gp10b_h_

static inline u32 gmmu_new_pde_is_pte_w(void)
{
	return 0U;
}
static inline u32 gmmu_new_pde_is_pte_false_f(void)
{
	return 0x0U;
}
static inline u32 gmmu_new_pde_aperture_w(void)
{
	return 0U;
}
static inline u32 gmmu_new_pde_aperture_invalid_f(void)
{
	return 0x0U;
}
static inline u32 gmmu_new_pde_aperture_video_memory_f(void)
{
	return 0x2U;
}
static inline u32 gmmu_new_pde_aperture_sys_mem_coh_f(void)
{
	return 0x4U;
}
static inline u32 gmmu_new_pde_aperture_sys_mem_ncoh_f(void)
{
	return 0x6U;
}
static inline u32 gmmu_new_pde_address_sys_f(u32 v)
{
	return (v & 0xffffffU) << 8U;
}
static inline u32 gmmu_new_pde_address_sys_w(void)
{
	return 0U;
}
static inline u32 gmmu_new_pde_vol_w(void)
{
	return 0U;
}
static inline u32 gmmu_new_pde_vol_true_f(void)
{
	return 0x8U;
}
static inline u32 gmmu_new_pde_vol_false_f(void)
{
	return 0x0U;
}
static inline u32 gmmu_new_pde_address_shift_v(void)
{
	return 0x0000000cU;
}
static inline u32 gmmu_new_pde__size_v(void)
{
	return 0x00000008U;
}
static inline u32 gmmu_new_dual_pde_is_pte_w(void)
{
	return 0U;
}
static inline u32 gmmu_new_dual_pde_is_pte_false_f(void)
{
	return 0x0U;
}
static inline u32 gmmu_new_dual_pde_aperture_big_w(void)
{
	return 0U;
}
static inline u32 gmmu_new_dual_pde_aperture_big_invalid_f(void)
{
	return 0x0U;
}
static inline u32 gmmu_new_dual_pde_aperture_big_video_memory_f(void)
{
	return 0x2U;
}
static inline u32 gmmu_new_dual_pde_aperture_big_sys_mem_coh_f(void)
{
	return 0x4U;
}
static inline u32 gmmu_new_dual_pde_aperture_big_sys_mem_ncoh_f(void)
{
	return 0x6U;
}
static inline u32 gmmu_new_dual_pde_address_big_sys_f(u32 v)
{
	return (v & 0xfffffffU) << 4U;
}
static inline u32 gmmu_new_dual_pde_address_big_sys_w(void)
{
	return 0U;
}
static inline u32 gmmu_new_dual_pde_aperture_small_w(void)
{
	return 2U;
}
static inline u32 gmmu_new_dual_pde_aperture_small_invalid_f(void)
{
	return 0x0U;
}
static inline u32 gmmu_new_dual_pde_aperture_small_video_memory_f(void)
{
	return 0x2U;
}
static inline u32 gmmu_new_dual_pde_aperture_small_sys_mem_coh_f(void)
{
	return 0x4U;
}
static inline u32 gmmu_new_dual_pde_aperture_small_sys_mem_ncoh_f(void)
{
	return 0x6U;
}
static inline u32 gmmu_new_dual_pde_vol_small_w(void)
{
	return 2U;
}
static inline u32 gmmu_new_dual_pde_vol_small_true_f(void)
{
	return 0x8U;
}
static inline u32 gmmu_new_dual_pde_vol_small_false_f(void)
{
	return 0x0U;
}
static inline u32 gmmu_new_dual_pde_vol_big_w(void)
{
	return 0U;
}
static inline u32 gmmu_new_dual_pde_vol_big_true_f(void)
{
	return 0x8U;
}
static inline u32 gmmu_new_dual_pde_vol_big_false_f(void)
{
	return 0x0U;
}
static inline u32 gmmu_new_dual_pde_address_small_sys_f(u32 v)
{
	return (v & 0xffffffU) << 8U;
}
static inline u32 gmmu_new_dual_pde_address_small_sys_w(void)
{
	return 2U;
}
static inline u32 gmmu_new_dual_pde_address_shift_v(void)
{
	return 0x0000000cU;
}
static inline u32 gmmu_new_dual_pde_address_big_shift_v(void)
{
	return 0x00000008U;
}
static inline u32 gmmu_new_dual_pde__size_v(void)
{
	return 0x00000010U;
}
static inline u32 gmmu_new_pte__size_v(void)
{
	return 0x00000008U;
}
static inline u32 gmmu_new_pte_valid_w(void)
{
	return 0U;
}
static inline u32 gmmu_new_pte_valid_true_f(void)
{
	return 0x1U;
}
static inline u32 gmmu_new_pte_valid_false_f(void)
{
	return 0x0U;
}
static inline u32 gmmu_new_pte_privilege_w(void)
{
	return 0U;
}
static inline u32 gmmu_new_pte_privilege_true_f(void)
{
	return 0x20U;
}
static inline u32 gmmu_new_pte_privilege_false_f(void)
{
	return 0x0U;
}
static inline u32 gmmu_new_pte_address_sys_f(u32 v)
{
	return (v & 0xffffffU) << 8U;
}
static inline u32 gmmu_new_pte_address_sys_w(void)
{
	return 0U;
}
static inline u32 gmmu_new_pte_address_vid_f(u32 v)
{
	return (v & 0xffffffU) << 8U;
}
static inline u32 gmmu_new_pte_address_vid_w(void)
{
	return 0U;
}
static inline u32 gmmu_new_pte_vol_w(void)
{
	return 0U;
}
static inline u32 gmmu_new_pte_vol_true_f(void)
{
	return 0x8U;
}
static inline u32 gmmu_new_pte_vol_false_f(void)
{
	return 0x0U;
}
static inline u32 gmmu_new_pte_aperture_w(void)
{
	return 0U;
}
static inline u32 gmmu_new_pte_aperture_video_memory_f(void)
{
	return 0x0U;
}
static inline u32 gmmu_new_pte_aperture_sys_mem_coh_f(void)
{
	return 0x4U;
}
static inline u32 gmmu_new_pte_aperture_sys_mem_ncoh_f(void)
{
	return 0x6U;
}
static inline u32 gmmu_new_pte_read_only_w(void)
{
	return 0U;
}
static inline u32 gmmu_new_pte_read_only_true_f(void)
{
	return 0x40U;
}
static inline u32 gmmu_new_pte_comptagline_f(u32 v)
{
	return (v & 0x3ffffU) << 4U;
}
static inline u32 gmmu_new_pte_comptagline_w(void)
{
	return 1U;
}
static inline u32 gmmu_new_pte_kind_f(u32 v)
{
	return (v & 0xffU) << 24U;
}
static inline u32 gmmu_new_pte_kind_w(void)
{
	return 1U;
}
static inline u32 gmmu_new_pte_address_shift_v(void)
{
	return 0x0000000cU;
}
static inline u32 gmmu_pte_kind_f(u32 v)
{
	return (v & 0xffU) << 4U;
}
static inline u32 gmmu_pte_kind_w(void)
{
	return 1U;
}
static inline u32 gmmu_pte_kind_invalid_v(void)
{
	return 0x000000ffU;
}
static inline u32 gmmu_pte_kind_pitch_v(void)
{
	return 0x00000000U;
}
#endif
