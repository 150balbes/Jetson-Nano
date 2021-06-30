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
#ifndef _hw_ioctrl_gv100_h_
#define _hw_ioctrl_gv100_h_

static inline u32 ioctrl_reset_r(void)
{
	return 0x00000140U;
}
static inline u32 ioctrl_reset_sw_post_reset_delay_microseconds_v(void)
{
	return 0x00000008U;
}
static inline u32 ioctrl_reset_linkreset_f(u32 v)
{
	return (v & 0x3fU) << 8U;
}
static inline u32 ioctrl_reset_linkreset_m(void)
{
	return 0x3fU << 8U;
}
static inline u32 ioctrl_reset_linkreset_v(u32 r)
{
	return (r >> 8U) & 0x3fU;
}
static inline u32 ioctrl_debug_reset_r(void)
{
	return 0x00000144U;
}
static inline u32 ioctrl_debug_reset_link_f(u32 v)
{
	return (v & 0x3fU) << 0U;
}
static inline u32 ioctrl_debug_reset_link_m(void)
{
	return 0x3fU << 0U;
}
static inline u32 ioctrl_debug_reset_link_v(u32 r)
{
	return (r >> 0U) & 0x3fU;
}
static inline u32 ioctrl_debug_reset_common_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 ioctrl_debug_reset_common_m(void)
{
	return 0x1U << 31U;
}
static inline u32 ioctrl_debug_reset_common_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 ioctrl_clock_control_r(u32 i)
{
	return 0x00000180U + i*4U;
}
static inline u32 ioctrl_clock_control__size_1_v(void)
{
	return 0x00000006U;
}
static inline u32 ioctrl_clock_control_clkdis_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 ioctrl_clock_control_clkdis_m(void)
{
	return 0x1U << 0U;
}
static inline u32 ioctrl_clock_control_clkdis_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 ioctrl_top_intr_0_status_r(void)
{
	return 0x00000200U;
}
static inline u32 ioctrl_top_intr_0_status_link_f(u32 v)
{
	return (v & 0x3fU) << 0U;
}
static inline u32 ioctrl_top_intr_0_status_link_m(void)
{
	return 0x3fU << 0U;
}
static inline u32 ioctrl_top_intr_0_status_link_v(u32 r)
{
	return (r >> 0U) & 0x3fU;
}
static inline u32 ioctrl_top_intr_0_status_common_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 ioctrl_top_intr_0_status_common_m(void)
{
	return 0x1U << 31U;
}
static inline u32 ioctrl_top_intr_0_status_common_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 ioctrl_common_intr_0_mask_r(void)
{
	return 0x00000220U;
}
static inline u32 ioctrl_common_intr_0_mask_fatal_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 ioctrl_common_intr_0_mask_fatal_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 ioctrl_common_intr_0_mask_nonfatal_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 ioctrl_common_intr_0_mask_nonfatal_v(u32 r)
{
	return (r >> 1U) & 0x1U;
}
static inline u32 ioctrl_common_intr_0_mask_correctable_f(u32 v)
{
	return (v & 0x1U) << 2U;
}
static inline u32 ioctrl_common_intr_0_mask_correctable_v(u32 r)
{
	return (r >> 2U) & 0x1U;
}
static inline u32 ioctrl_common_intr_0_mask_intra_f(u32 v)
{
	return (v & 0x1U) << 3U;
}
static inline u32 ioctrl_common_intr_0_mask_intra_v(u32 r)
{
	return (r >> 3U) & 0x1U;
}
static inline u32 ioctrl_common_intr_0_mask_intrb_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 ioctrl_common_intr_0_mask_intrb_v(u32 r)
{
	return (r >> 4U) & 0x1U;
}
static inline u32 ioctrl_common_intr_0_status_r(void)
{
	return 0x00000224U;
}
static inline u32 ioctrl_common_intr_0_status_fatal_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 ioctrl_common_intr_0_status_fatal_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 ioctrl_common_intr_0_status_nonfatal_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 ioctrl_common_intr_0_status_nonfatal_v(u32 r)
{
	return (r >> 1U) & 0x1U;
}
static inline u32 ioctrl_common_intr_0_status_correctable_f(u32 v)
{
	return (v & 0x1U) << 2U;
}
static inline u32 ioctrl_common_intr_0_status_correctable_v(u32 r)
{
	return (r >> 2U) & 0x1U;
}
static inline u32 ioctrl_common_intr_0_status_intra_f(u32 v)
{
	return (v & 0x1U) << 3U;
}
static inline u32 ioctrl_common_intr_0_status_intra_v(u32 r)
{
	return (r >> 3U) & 0x1U;
}
static inline u32 ioctrl_common_intr_0_status_intrb_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 ioctrl_common_intr_0_status_intrb_v(u32 r)
{
	return (r >> 4U) & 0x1U;
}
static inline u32 ioctrl_link_intr_0_mask_r(u32 i)
{
	return 0x00000240U + i*20U;
}
static inline u32 ioctrl_link_intr_0_mask_fatal_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 ioctrl_link_intr_0_mask_fatal_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 ioctrl_link_intr_0_mask_nonfatal_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 ioctrl_link_intr_0_mask_nonfatal_v(u32 r)
{
	return (r >> 1U) & 0x1U;
}
static inline u32 ioctrl_link_intr_0_mask_correctable_f(u32 v)
{
	return (v & 0x1U) << 2U;
}
static inline u32 ioctrl_link_intr_0_mask_correctable_v(u32 r)
{
	return (r >> 2U) & 0x1U;
}
static inline u32 ioctrl_link_intr_0_mask_intra_f(u32 v)
{
	return (v & 0x1U) << 3U;
}
static inline u32 ioctrl_link_intr_0_mask_intra_v(u32 r)
{
	return (r >> 3U) & 0x1U;
}
static inline u32 ioctrl_link_intr_0_mask_intrb_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 ioctrl_link_intr_0_mask_intrb_v(u32 r)
{
	return (r >> 4U) & 0x1U;
}
static inline u32 ioctrl_link_intr_0_status_r(u32 i)
{
	return 0x00000244U + i*20U;
}
static inline u32 ioctrl_link_intr_0_status_fatal_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 ioctrl_link_intr_0_status_fatal_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 ioctrl_link_intr_0_status_nonfatal_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 ioctrl_link_intr_0_status_nonfatal_v(u32 r)
{
	return (r >> 1U) & 0x1U;
}
static inline u32 ioctrl_link_intr_0_status_correctable_f(u32 v)
{
	return (v & 0x1U) << 2U;
}
static inline u32 ioctrl_link_intr_0_status_correctable_v(u32 r)
{
	return (r >> 2U) & 0x1U;
}
static inline u32 ioctrl_link_intr_0_status_intra_f(u32 v)
{
	return (v & 0x1U) << 3U;
}
static inline u32 ioctrl_link_intr_0_status_intra_v(u32 r)
{
	return (r >> 3U) & 0x1U;
}
static inline u32 ioctrl_link_intr_0_status_intrb_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 ioctrl_link_intr_0_status_intrb_v(u32 r)
{
	return (r >> 4U) & 0x1U;
}
#endif
