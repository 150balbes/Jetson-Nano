/*
 * Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef _hw_flush_gp106_h_
#define _hw_flush_gp106_h_

static inline u32 flush_l2_system_invalidate_r(void)
{
	return 0x00070004U;
}
static inline u32 flush_l2_system_invalidate_pending_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 flush_l2_system_invalidate_pending_busy_v(void)
{
	return 0x00000001U;
}
static inline u32 flush_l2_system_invalidate_pending_busy_f(void)
{
	return 0x1U;
}
static inline u32 flush_l2_system_invalidate_outstanding_v(u32 r)
{
	return (r >> 1U) & 0x1U;
}
static inline u32 flush_l2_system_invalidate_outstanding_true_v(void)
{
	return 0x00000001U;
}
static inline u32 flush_l2_flush_dirty_r(void)
{
	return 0x00070010U;
}
static inline u32 flush_l2_flush_dirty_pending_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 flush_l2_flush_dirty_pending_empty_v(void)
{
	return 0x00000000U;
}
static inline u32 flush_l2_flush_dirty_pending_empty_f(void)
{
	return 0x0U;
}
static inline u32 flush_l2_flush_dirty_pending_busy_v(void)
{
	return 0x00000001U;
}
static inline u32 flush_l2_flush_dirty_pending_busy_f(void)
{
	return 0x1U;
}
static inline u32 flush_l2_flush_dirty_outstanding_v(u32 r)
{
	return (r >> 1U) & 0x1U;
}
static inline u32 flush_l2_flush_dirty_outstanding_false_v(void)
{
	return 0x00000000U;
}
static inline u32 flush_l2_flush_dirty_outstanding_false_f(void)
{
	return 0x0U;
}
static inline u32 flush_l2_flush_dirty_outstanding_true_v(void)
{
	return 0x00000001U;
}
static inline u32 flush_l2_clean_comptags_r(void)
{
	return 0x0007000cU;
}
static inline u32 flush_l2_clean_comptags_pending_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 flush_l2_clean_comptags_pending_empty_v(void)
{
	return 0x00000000U;
}
static inline u32 flush_l2_clean_comptags_pending_empty_f(void)
{
	return 0x0U;
}
static inline u32 flush_l2_clean_comptags_pending_busy_v(void)
{
	return 0x00000001U;
}
static inline u32 flush_l2_clean_comptags_pending_busy_f(void)
{
	return 0x1U;
}
static inline u32 flush_l2_clean_comptags_outstanding_v(u32 r)
{
	return (r >> 1U) & 0x1U;
}
static inline u32 flush_l2_clean_comptags_outstanding_false_v(void)
{
	return 0x00000000U;
}
static inline u32 flush_l2_clean_comptags_outstanding_false_f(void)
{
	return 0x0U;
}
static inline u32 flush_l2_clean_comptags_outstanding_true_v(void)
{
	return 0x00000001U;
}
static inline u32 flush_fb_flush_r(void)
{
	return 0x00070000U;
}
static inline u32 flush_fb_flush_pending_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 flush_fb_flush_pending_busy_v(void)
{
	return 0x00000001U;
}
static inline u32 flush_fb_flush_pending_busy_f(void)
{
	return 0x1U;
}
static inline u32 flush_fb_flush_outstanding_v(u32 r)
{
	return (r >> 1U) & 0x1U;
}
static inline u32 flush_fb_flush_outstanding_true_v(void)
{
	return 0x00000001U;
}
#endif
