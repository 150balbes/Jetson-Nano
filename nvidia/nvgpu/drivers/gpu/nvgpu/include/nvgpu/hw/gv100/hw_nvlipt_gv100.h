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
#ifndef _hw_nvlipt_gv100_h_
#define _hw_nvlipt_gv100_h_

static inline u32 nvlipt_intr_control_link0_r(void)
{
	return 0x000004b4U;
}
static inline u32 nvlipt_intr_control_link0_stallenable_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 nvlipt_intr_control_link0_stallenable_m(void)
{
	return 0x1U << 0U;
}
static inline u32 nvlipt_intr_control_link0_stallenable_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 nvlipt_intr_control_link0_nostallenable_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 nvlipt_intr_control_link0_nostallenable_m(void)
{
	return 0x1U << 1U;
}
static inline u32 nvlipt_intr_control_link0_nostallenable_v(u32 r)
{
	return (r >> 1U) & 0x1U;
}
static inline u32 nvlipt_err_uc_status_link0_r(void)
{
	return 0x00000524U;
}
static inline u32 nvlipt_err_uc_status_link0_dlprotocol_f(u32 v)
{
	return (v & 0x1U) << 4U;
}
static inline u32 nvlipt_err_uc_status_link0_dlprotocol_v(u32 r)
{
	return (r >> 4U) & 0x1U;
}
static inline u32 nvlipt_err_uc_status_link0_datapoisoned_f(u32 v)
{
	return (v & 0x1U) << 12U;
}
static inline u32 nvlipt_err_uc_status_link0_datapoisoned_v(u32 r)
{
	return (r >> 12U) & 0x1U;
}
static inline u32 nvlipt_err_uc_status_link0_flowcontrol_f(u32 v)
{
	return (v & 0x1U) << 13U;
}
static inline u32 nvlipt_err_uc_status_link0_flowcontrol_v(u32 r)
{
	return (r >> 13U) & 0x1U;
}
static inline u32 nvlipt_err_uc_status_link0_responsetimeout_f(u32 v)
{
	return (v & 0x1U) << 14U;
}
static inline u32 nvlipt_err_uc_status_link0_responsetimeout_v(u32 r)
{
	return (r >> 14U) & 0x1U;
}
static inline u32 nvlipt_err_uc_status_link0_targeterror_f(u32 v)
{
	return (v & 0x1U) << 15U;
}
static inline u32 nvlipt_err_uc_status_link0_targeterror_v(u32 r)
{
	return (r >> 15U) & 0x1U;
}
static inline u32 nvlipt_err_uc_status_link0_unexpectedresponse_f(u32 v)
{
	return (v & 0x1U) << 16U;
}
static inline u32 nvlipt_err_uc_status_link0_unexpectedresponse_v(u32 r)
{
	return (r >> 16U) & 0x1U;
}
static inline u32 nvlipt_err_uc_status_link0_receiveroverflow_f(u32 v)
{
	return (v & 0x1U) << 17U;
}
static inline u32 nvlipt_err_uc_status_link0_receiveroverflow_v(u32 r)
{
	return (r >> 17U) & 0x1U;
}
static inline u32 nvlipt_err_uc_status_link0_malformedpacket_f(u32 v)
{
	return (v & 0x1U) << 18U;
}
static inline u32 nvlipt_err_uc_status_link0_malformedpacket_v(u32 r)
{
	return (r >> 18U) & 0x1U;
}
static inline u32 nvlipt_err_uc_status_link0_stompedpacketreceived_f(u32 v)
{
	return (v & 0x1U) << 19U;
}
static inline u32 nvlipt_err_uc_status_link0_stompedpacketreceived_v(u32 r)
{
	return (r >> 19U) & 0x1U;
}
static inline u32 nvlipt_err_uc_status_link0_unsupportedrequest_f(u32 v)
{
	return (v & 0x1U) << 20U;
}
static inline u32 nvlipt_err_uc_status_link0_unsupportedrequest_v(u32 r)
{
	return (r >> 20U) & 0x1U;
}
static inline u32 nvlipt_err_uc_status_link0_ucinternal_f(u32 v)
{
	return (v & 0x1U) << 22U;
}
static inline u32 nvlipt_err_uc_status_link0_ucinternal_v(u32 r)
{
	return (r >> 22U) & 0x1U;
}
static inline u32 nvlipt_err_uc_mask_link0_r(void)
{
	return 0x00000528U;
}
static inline u32 nvlipt_err_uc_severity_link0_r(void)
{
	return 0x0000052cU;
}
static inline u32 nvlipt_err_uc_first_link0_r(void)
{
	return 0x00000530U;
}
static inline u32 nvlipt_err_uc_advisory_link0_r(void)
{
	return 0x00000534U;
}
static inline u32 nvlipt_err_c_status_link0_r(void)
{
	return 0x00000538U;
}
static inline u32 nvlipt_err_c_mask_link0_r(void)
{
	return 0x0000053cU;
}
static inline u32 nvlipt_err_c_first_link0_r(void)
{
	return 0x00000540U;
}
static inline u32 nvlipt_err_control_link0_r(void)
{
	return 0x00000544U;
}
static inline u32 nvlipt_err_control_link0_fatalenable_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 nvlipt_err_control_link0_fatalenable_m(void)
{
	return 0x1U << 1U;
}
static inline u32 nvlipt_err_control_link0_fatalenable_v(u32 r)
{
	return (r >> 1U) & 0x1U;
}
static inline u32 nvlipt_err_control_link0_nonfatalenable_f(u32 v)
{
	return (v & 0x1U) << 2U;
}
static inline u32 nvlipt_err_control_link0_nonfatalenable_m(void)
{
	return 0x1U << 2U;
}
static inline u32 nvlipt_err_control_link0_nonfatalenable_v(u32 r)
{
	return (r >> 2U) & 0x1U;
}
static inline u32 nvlipt_intr_control_common_r(void)
{
	return 0x000004b0U;
}
static inline u32 nvlipt_intr_control_common_stallenable_f(u32 v)
{
	return (v & 0x1U) << 0U;
}
static inline u32 nvlipt_intr_control_common_stallenable_m(void)
{
	return 0x1U << 0U;
}
static inline u32 nvlipt_intr_control_common_stallenable_v(u32 r)
{
	return (r >> 0U) & 0x1U;
}
static inline u32 nvlipt_intr_control_common_nonstallenable_f(u32 v)
{
	return (v & 0x1U) << 1U;
}
static inline u32 nvlipt_intr_control_common_nonstallenable_m(void)
{
	return 0x1U << 1U;
}
static inline u32 nvlipt_intr_control_common_nonstallenable_v(u32 r)
{
	return (r >> 1U) & 0x1U;
}
static inline u32 nvlipt_scratch_cold_r(void)
{
	return 0x000007d4U;
}
static inline u32 nvlipt_scratch_cold_data_f(u32 v)
{
	return (v & 0xffffffffU) << 0U;
}
static inline u32 nvlipt_scratch_cold_data_v(u32 r)
{
	return (r >> 0U) & 0xffffffffU;
}
static inline u32 nvlipt_scratch_cold_data_init_v(void)
{
	return 0xdeadbaadU;
}
#endif
