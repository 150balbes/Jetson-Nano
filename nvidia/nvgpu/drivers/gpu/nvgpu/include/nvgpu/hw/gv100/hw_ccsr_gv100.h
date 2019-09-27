/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _hw_ccsr_gv100_h_
#define _hw_ccsr_gv100_h_

static inline u32 ccsr_channel_inst_r(u32 i)
{
	return 0x00800000U + i*8U;
}
static inline u32 ccsr_channel_inst__size_1_v(void)
{
	return 0x00001000U;
}
static inline u32 ccsr_channel_inst_ptr_f(u32 v)
{
	return (v & 0xfffffffU) << 0U;
}
static inline u32 ccsr_channel_inst_target_vid_mem_f(void)
{
	return 0x0U;
}
static inline u32 ccsr_channel_inst_target_sys_mem_coh_f(void)
{
	return 0x20000000U;
}
static inline u32 ccsr_channel_inst_target_sys_mem_ncoh_f(void)
{
	return 0x30000000U;
}
static inline u32 ccsr_channel_inst_bind_false_f(void)
{
	return 0x0U;
}
static inline u32 ccsr_channel_inst_bind_true_f(void)
{
	return 0x80000000U;
}
static inline u32 ccsr_channel_r(u32 i)
{
	return 0x00800004U + i*8U;
}
static inline u32 ccsr_channel__size_1_v(void)
{
	return 0x00001000U;
}
static inline u32 ccsr_channel_enable_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 ccsr_channel_enable_set_f(u32 v)
{
	return (v & 0x1U) << 10U;
}
static inline u32 ccsr_channel_enable_set_true_f(void)
{
	return 0x400U;
}
static inline u32 ccsr_channel_enable_clr_true_f(void)
{
	return 0x800U;
}
static inline u32 ccsr_channel_status_v(u32 r)
{
	return (r >> 24U) & 0xfU;
}
static inline u32 ccsr_channel_status_pending_ctx_reload_v(void)
{
	return 0x00000002U;
}
static inline u32 ccsr_channel_status_pending_acq_ctx_reload_v(void)
{
	return 0x00000004U;
}
static inline u32 ccsr_channel_status_on_pbdma_ctx_reload_v(void)
{
	return 0x0000000aU;
}
static inline u32 ccsr_channel_status_on_pbdma_and_eng_ctx_reload_v(void)
{
	return 0x0000000bU;
}
static inline u32 ccsr_channel_status_on_eng_ctx_reload_v(void)
{
	return 0x0000000cU;
}
static inline u32 ccsr_channel_status_on_eng_pending_ctx_reload_v(void)
{
	return 0x0000000dU;
}
static inline u32 ccsr_channel_status_on_eng_pending_acq_ctx_reload_v(void)
{
	return 0x0000000eU;
}
static inline u32 ccsr_channel_next_v(u32 r)
{
	return (r >> 1U) & 0x1U;
}
static inline u32 ccsr_channel_next_true_v(void)
{
	return 0x00000001U;
}
static inline u32 ccsr_channel_force_ctx_reload_true_f(void)
{
	return 0x100U;
}
static inline u32 ccsr_channel_pbdma_faulted_f(u32 v)
{
	return (v & 0x1U) << 22U;
}
static inline u32 ccsr_channel_pbdma_faulted_reset_f(void)
{
	return 0x400000U;
}
static inline u32 ccsr_channel_eng_faulted_f(u32 v)
{
	return (v & 0x1U) << 23U;
}
static inline u32 ccsr_channel_eng_faulted_v(u32 r)
{
	return (r >> 23U) & 0x1U;
}
static inline u32 ccsr_channel_eng_faulted_reset_f(void)
{
	return 0x800000U;
}
static inline u32 ccsr_channel_eng_faulted_true_v(void)
{
	return 0x00000001U;
}
static inline u32 ccsr_channel_busy_v(u32 r)
{
	return (r >> 28U) & 0x1U;
}
#endif
