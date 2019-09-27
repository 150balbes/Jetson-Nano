/*
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _hw_top_gm20b_h_
#define _hw_top_gm20b_h_

static inline u32 top_num_gpcs_r(void)
{
	return 0x00022430U;
}
static inline u32 top_num_gpcs_value_v(u32 r)
{
	return (r >> 0U) & 0x1fU;
}
static inline u32 top_tpc_per_gpc_r(void)
{
	return 0x00022434U;
}
static inline u32 top_tpc_per_gpc_value_v(u32 r)
{
	return (r >> 0U) & 0x1fU;
}
static inline u32 top_num_fbps_r(void)
{
	return 0x00022438U;
}
static inline u32 top_num_fbps_value_v(u32 r)
{
	return (r >> 0U) & 0x1fU;
}
static inline u32 top_ltc_per_fbp_r(void)
{
	return 0x00022450U;
}
static inline u32 top_ltc_per_fbp_value_v(u32 r)
{
	return (r >> 0U) & 0x1fU;
}
static inline u32 top_slices_per_ltc_r(void)
{
	return 0x0002245cU;
}
static inline u32 top_slices_per_ltc_value_v(u32 r)
{
	return (r >> 0U) & 0x1fU;
}
static inline u32 top_num_ltcs_r(void)
{
	return 0x00022454U;
}
static inline u32 top_device_info_r(u32 i)
{
	return 0x00022700U + i*4U;
}
static inline u32 top_device_info__size_1_v(void)
{
	return 0x00000040U;
}
static inline u32 top_device_info_chain_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 top_device_info_chain_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 top_device_info_engine_enum_v(u32 r)
{
	return (r >> 26U) & 0xfU;
}
static inline u32 top_device_info_runlist_enum_v(u32 r)
{
	return (r >> 21U) & 0xfU;
}
static inline u32 top_device_info_intr_enum_v(u32 r)
{
	return (r >> 15U) & 0x1fU;
}
static inline u32 top_device_info_reset_enum_v(u32 r)
{
	return (r >> 9U) & 0x1fU;
}
static inline u32 top_device_info_type_enum_v(u32 r)
{
	return (r >> 2U) & 0x1fffffffU;
}
static inline u32 top_device_info_type_enum_graphics_v(void)
{
	return 0x00000000U;
}
static inline u32 top_device_info_type_enum_graphics_f(void)
{
	return 0x0U;
}
static inline u32 top_device_info_type_enum_copy0_v(void)
{
	return 0x00000001U;
}
static inline u32 top_device_info_type_enum_copy0_f(void)
{
	return 0x4U;
}
static inline u32 top_device_info_type_enum_copy1_v(void)
{
	return 0x00000002U;
}
static inline u32 top_device_info_type_enum_copy1_f(void)
{
	return 0x8U;
}
static inline u32 top_device_info_type_enum_copy2_v(void)
{
	return 0x00000003U;
}
static inline u32 top_device_info_type_enum_copy2_f(void)
{
	return 0xcU;
}
static inline u32 top_device_info_engine_v(u32 r)
{
	return (r >> 5U) & 0x1U;
}
static inline u32 top_device_info_runlist_v(u32 r)
{
	return (r >> 4U) & 0x1U;
}
static inline u32 top_device_info_intr_v(u32 r)
{
	return (r >> 3U) & 0x1U;
}
static inline u32 top_device_info_reset_v(u32 r)
{
	return (r >> 2U) & 0x1U;
}
static inline u32 top_device_info_entry_v(u32 r)
{
	return (r >> 0U) & 0x3U;
}
static inline u32 top_device_info_entry_not_valid_v(void)
{
	return 0x00000000U;
}
static inline u32 top_device_info_entry_enum_v(void)
{
	return 0x00000002U;
}
static inline u32 top_device_info_entry_engine_type_v(void)
{
	return 0x00000003U;
}
static inline u32 top_device_info_entry_data_v(void)
{
	return 0x00000001U;
}
static inline u32 top_device_info_data_type_v(u32 r)
{
	return (r >> 30U) & 0x1U;
}
static inline u32 top_device_info_data_type_enum2_v(void)
{
	return 0x00000000U;
}
static inline u32 top_device_info_data_pri_base_v(u32 r)
{
	return (r >> 12U) & 0x7ffU;
}
static inline u32 top_device_info_data_pri_base_align_v(void)
{
	return 0x0000000cU;
}
static inline u32 top_device_info_data_fault_id_enum_v(u32 r)
{
	return (r >> 3U) & 0x1fU;
}
static inline u32 top_device_info_data_fault_id_v(u32 r)
{
	return (r >> 2U) & 0x1U;
}
static inline u32 top_device_info_data_fault_id_valid_v(void)
{
	return 0x00000001U;
}
#endif
