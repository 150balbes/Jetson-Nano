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
#ifndef _hw_timer_gv100_h_
#define _hw_timer_gv100_h_

static inline u32 timer_pri_timeout_r(void)
{
	return 0x00009080U;
}
static inline u32 timer_pri_timeout_period_f(u32 v)
{
	return (v & 0xffffffU) << 0U;
}
static inline u32 timer_pri_timeout_period_m(void)
{
	return 0xffffffU << 0U;
}
static inline u32 timer_pri_timeout_period_v(u32 r)
{
	return (r >> 0U) & 0xffffffU;
}
static inline u32 timer_pri_timeout_en_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 timer_pri_timeout_en_m(void)
{
	return 0x1U << 31U;
}
static inline u32 timer_pri_timeout_en_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 timer_pri_timeout_en_en_enabled_f(void)
{
	return 0x80000000U;
}
static inline u32 timer_pri_timeout_en_en_disabled_f(void)
{
	return 0x0U;
}
static inline u32 timer_pri_timeout_save_0_r(void)
{
	return 0x00009084U;
}
static inline u32 timer_pri_timeout_save_1_r(void)
{
	return 0x00009088U;
}
static inline u32 timer_pri_timeout_fecs_errcode_r(void)
{
	return 0x0000908cU;
}
static inline u32 timer_time_0_r(void)
{
	return 0x00009400U;
}
static inline u32 timer_time_1_r(void)
{
	return 0x00009410U;
}
#endif
