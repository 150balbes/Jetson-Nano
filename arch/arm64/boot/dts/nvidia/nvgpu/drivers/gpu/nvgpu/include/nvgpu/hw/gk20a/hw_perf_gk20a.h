/*
 * Copyright (c) 2015-2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _hw_perf_gk20a_h_
#define _hw_perf_gk20a_h_

static inline u32 perf_pmasys_control_r(void)
{
	return 0x001b4000U;
}
static inline u32 perf_pmasys_control_membuf_status_v(u32 r)
{
	return (r >> 4U) & 0x1U;
}
static inline u32 perf_pmasys_control_membuf_status_overflowed_v(void)
{
	return 0x00000001U;
}
static inline u32 perf_pmasys_control_membuf_status_overflowed_f(void)
{
	return 0x10U;
}
static inline u32 perf_pmasys_control_membuf_clear_status_f(u32 v)
{
	return (v & 0x1U) << 5U;
}
static inline u32 perf_pmasys_control_membuf_clear_status_v(u32 r)
{
	return (r >> 5U) & 0x1U;
}
static inline u32 perf_pmasys_control_membuf_clear_status_doit_v(void)
{
	return 0x00000001U;
}
static inline u32 perf_pmasys_control_membuf_clear_status_doit_f(void)
{
	return 0x20U;
}
static inline u32 perf_pmasys_mem_block_r(void)
{
	return 0x001b4070U;
}
static inline u32 perf_pmasys_mem_block_base_f(u32 v)
{
	return (v & 0xfffffffU) << 0U;
}
static inline u32 perf_pmasys_mem_block_target_f(u32 v)
{
	return (v & 0x3U) << 28U;
}
static inline u32 perf_pmasys_mem_block_target_v(u32 r)
{
	return (r >> 28U) & 0x3U;
}
static inline u32 perf_pmasys_mem_block_target_lfb_v(void)
{
	return 0x00000000U;
}
static inline u32 perf_pmasys_mem_block_target_lfb_f(void)
{
	return 0x0U;
}
static inline u32 perf_pmasys_mem_block_target_sys_coh_v(void)
{
	return 0x00000002U;
}
static inline u32 perf_pmasys_mem_block_target_sys_coh_f(void)
{
	return 0x20000000U;
}
static inline u32 perf_pmasys_mem_block_target_sys_ncoh_v(void)
{
	return 0x00000003U;
}
static inline u32 perf_pmasys_mem_block_target_sys_ncoh_f(void)
{
	return 0x30000000U;
}
static inline u32 perf_pmasys_mem_block_valid_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 perf_pmasys_mem_block_valid_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 perf_pmasys_mem_block_valid_true_v(void)
{
	return 0x00000001U;
}
static inline u32 perf_pmasys_mem_block_valid_true_f(void)
{
	return 0x80000000U;
}
static inline u32 perf_pmasys_mem_block_valid_false_v(void)
{
	return 0x00000000U;
}
static inline u32 perf_pmasys_mem_block_valid_false_f(void)
{
	return 0x0U;
}
static inline u32 perf_pmasys_outbase_r(void)
{
	return 0x001b4074U;
}
static inline u32 perf_pmasys_outbase_ptr_f(u32 v)
{
	return (v & 0x7ffffffU) << 5U;
}
static inline u32 perf_pmasys_outbaseupper_r(void)
{
	return 0x001b4078U;
}
static inline u32 perf_pmasys_outbaseupper_ptr_f(u32 v)
{
	return (v & 0xffU) << 0U;
}
static inline u32 perf_pmasys_outsize_r(void)
{
	return 0x001b407cU;
}
static inline u32 perf_pmasys_outsize_numbytes_f(u32 v)
{
	return (v & 0x7ffffffU) << 5U;
}
static inline u32 perf_pmasys_mem_bytes_r(void)
{
	return 0x001b4084U;
}
static inline u32 perf_pmasys_mem_bytes_numbytes_f(u32 v)
{
	return (v & 0xfffffffU) << 4U;
}
static inline u32 perf_pmasys_mem_bump_r(void)
{
	return 0x001b4088U;
}
static inline u32 perf_pmasys_mem_bump_numbytes_f(u32 v)
{
	return (v & 0xfffffffU) << 4U;
}
static inline u32 perf_pmasys_enginestatus_r(void)
{
	return 0x001b40a4U;
}
static inline u32 perf_pmasys_enginestatus_rbufempty_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 perf_pmasys_enginestatus_rbufempty_empty_v(void)
{
	return 0x00000001U;
}
static inline u32 perf_pmasys_enginestatus_rbufempty_empty_f(void)
{
	return 0x10U;
}
#endif
