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
#ifndef _hw_nvlinkip_discovery_gv100_h_
#define _hw_nvlinkip_discovery_gv100_h_

static inline u32 nvlinkip_discovery_common_r(void)
{
	return 0x00000000U;
}
static inline u32 nvlinkip_discovery_common_entry_f(u32 v)
{
	return (v & 0x3U) << 0U;
}
static inline u32 nvlinkip_discovery_common_entry_v(u32 r)
{
	return (r >> 0U) & 0x3U;
}
static inline u32 nvlinkip_discovery_common_entry_invalid_v(void)
{
	return 0x00000000U;
}
static inline u32 nvlinkip_discovery_common_entry_enum_v(void)
{
	return 0x00000001U;
}
static inline u32 nvlinkip_discovery_common_entry_data1_v(void)
{
	return 0x00000002U;
}
static inline u32 nvlinkip_discovery_common_entry_data2_v(void)
{
	return 0x00000003U;
}
static inline u32 nvlinkip_discovery_common_contents_f(u32 v)
{
	return (v & 0x1fffffffU) << 2U;
}
static inline u32 nvlinkip_discovery_common_contents_v(u32 r)
{
	return (r >> 2U) & 0x1fffffffU;
}
static inline u32 nvlinkip_discovery_common_chain_f(u32 v)
{
	return (v & 0x1U) << 31U;
}
static inline u32 nvlinkip_discovery_common_chain_v(u32 r)
{
	return (r >> 31U) & 0x1U;
}
static inline u32 nvlinkip_discovery_common_chain_enable_v(void)
{
	return 0x00000001U;
}
static inline u32 nvlinkip_discovery_common_device_f(u32 v)
{
	return (v & 0x3fU) << 2U;
}
static inline u32 nvlinkip_discovery_common_device_v(u32 r)
{
	return (r >> 2U) & 0x3fU;
}
static inline u32 nvlinkip_discovery_common_device_invalid_v(void)
{
	return 0x00000000U;
}
static inline u32 nvlinkip_discovery_common_device_ioctrl_v(void)
{
	return 0x00000001U;
}
static inline u32 nvlinkip_discovery_common_device_nvltl_v(void)
{
	return 0x00000002U;
}
static inline u32 nvlinkip_discovery_common_device_nvlink_v(void)
{
	return 0x00000003U;
}
static inline u32 nvlinkip_discovery_common_device_minion_v(void)
{
	return 0x00000004U;
}
static inline u32 nvlinkip_discovery_common_device_nvlipt_v(void)
{
	return 0x00000005U;
}
static inline u32 nvlinkip_discovery_common_device_nvltlc_v(void)
{
	return 0x00000006U;
}
static inline u32 nvlinkip_discovery_common_device_dlpl_v(void)
{
	return 0x0000000bU;
}
static inline u32 nvlinkip_discovery_common_device_ioctrlmif_v(void)
{
	return 0x00000007U;
}
static inline u32 nvlinkip_discovery_common_device_dlpl_multicast_v(void)
{
	return 0x00000008U;
}
static inline u32 nvlinkip_discovery_common_device_nvltlc_multicast_v(void)
{
	return 0x00000009U;
}
static inline u32 nvlinkip_discovery_common_device_ioctrlmif_multicast_v(void)
{
	return 0x0000000aU;
}
static inline u32 nvlinkip_discovery_common_device_sioctrl_v(void)
{
	return 0x0000000cU;
}
static inline u32 nvlinkip_discovery_common_device_tioctrl_v(void)
{
	return 0x0000000dU;
}
static inline u32 nvlinkip_discovery_common_id_f(u32 v)
{
	return (v & 0xffU) << 8U;
}
static inline u32 nvlinkip_discovery_common_id_v(u32 r)
{
	return (r >> 8U) & 0xffU;
}
static inline u32 nvlinkip_discovery_common_version_f(u32 v)
{
	return (v & 0x7ffU) << 20U;
}
static inline u32 nvlinkip_discovery_common_version_v(u32 r)
{
	return (r >> 20U) & 0x7ffU;
}
static inline u32 nvlinkip_discovery_common_pri_base_f(u32 v)
{
	return (v & 0xfffU) << 12U;
}
static inline u32 nvlinkip_discovery_common_pri_base_v(u32 r)
{
	return (r >> 12U) & 0xfffU;
}
static inline u32 nvlinkip_discovery_common_intr_f(u32 v)
{
	return (v & 0x1fU) << 7U;
}
static inline u32 nvlinkip_discovery_common_intr_v(u32 r)
{
	return (r >> 7U) & 0x1fU;
}
static inline u32 nvlinkip_discovery_common_reset_f(u32 v)
{
	return (v & 0x1fU) << 2U;
}
static inline u32 nvlinkip_discovery_common_reset_v(u32 r)
{
	return (r >> 2U) & 0x1fU;
}
static inline u32 nvlinkip_discovery_common_ioctrl_length_f(u32 v)
{
	return (v & 0x3fU) << 24U;
}
static inline u32 nvlinkip_discovery_common_ioctrl_length_v(u32 r)
{
	return (r >> 24U) & 0x3fU;
}
static inline u32 nvlinkip_discovery_common_dlpl_num_tx_f(u32 v)
{
	return (v & 0x7U) << 24U;
}
static inline u32 nvlinkip_discovery_common_dlpl_num_tx_v(u32 r)
{
	return (r >> 24U) & 0x7U;
}
static inline u32 nvlinkip_discovery_common_dlpl_num_rx_f(u32 v)
{
	return (v & 0x7U) << 27U;
}
static inline u32 nvlinkip_discovery_common_dlpl_num_rx_v(u32 r)
{
	return (r >> 27U) & 0x7U;
}
static inline u32 nvlinkip_discovery_common_data1_ioctrl_length_f(u32 v)
{
	return (v & 0x7ffffU) << 12U;
}
static inline u32 nvlinkip_discovery_common_data1_ioctrl_length_v(u32 r)
{
	return (r >> 12U) & 0x7ffffU;
}
static inline u32 nvlinkip_discovery_common_data2_type_f(u32 v)
{
	return (v & 0x1fU) << 26U;
}
static inline u32 nvlinkip_discovery_common_data2_type_v(u32 r)
{
	return (r >> 26U) & 0x1fU;
}
static inline u32 nvlinkip_discovery_common_data2_type_invalid_v(void)
{
	return 0x00000000U;
}
static inline u32 nvlinkip_discovery_common_data2_type_pllcontrol_v(void)
{
	return 0x00000001U;
}
static inline u32 nvlinkip_discovery_common_data2_type_resetreg_v(void)
{
	return 0x00000002U;
}
static inline u32 nvlinkip_discovery_common_data2_type_intrreg_v(void)
{
	return 0x00000003U;
}
static inline u32 nvlinkip_discovery_common_data2_type_discovery_v(void)
{
	return 0x00000004U;
}
static inline u32 nvlinkip_discovery_common_data2_type_unicast_v(void)
{
	return 0x00000005U;
}
static inline u32 nvlinkip_discovery_common_data2_type_broadcast_v(void)
{
	return 0x00000006U;
}
static inline u32 nvlinkip_discovery_common_data2_addr_f(u32 v)
{
	return (v & 0xffffffU) << 2U;
}
static inline u32 nvlinkip_discovery_common_data2_addr_v(u32 r)
{
	return (r >> 2U) & 0xffffffU;
}
static inline u32 nvlinkip_discovery_common_dlpl_data2_type_f(u32 v)
{
	return (v & 0x1fU) << 26U;
}
static inline u32 nvlinkip_discovery_common_dlpl_data2_type_v(u32 r)
{
	return (r >> 26U) & 0x1fU;
}
static inline u32 nvlinkip_discovery_common_dlpl_data2_master_f(u32 v)
{
	return (v & 0x1U) << 15U;
}
static inline u32 nvlinkip_discovery_common_dlpl_data2_master_v(u32 r)
{
	return (r >> 15U) & 0x1U;
}
static inline u32 nvlinkip_discovery_common_dlpl_data2_masterid_f(u32 v)
{
	return (v & 0x7fU) << 8U;
}
static inline u32 nvlinkip_discovery_common_dlpl_data2_masterid_v(u32 r)
{
	return (r >> 8U) & 0x7fU;
}
#endif
