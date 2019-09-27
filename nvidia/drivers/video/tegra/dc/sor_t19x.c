/*
 * drivers/video/tegra/dc/sor_t19x.c
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION, All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "dc_priv.h"
#include "edid.h"
#include "hdmi2.0.h"
#include "hw_nvdisp_nvdisp.h"
#include "nvdisp.h"
#include "sor.h"
#include "sor_regs_t19x.h"

inline u32 nv_sor_head_state0_t19x(u32 i)
{
	return NV_HEAD_STATE0_T19X(i);
}

inline u32 nv_sor_head_state1_t19x(u32 i)
{
	return NV_HEAD_STATE1_T19X(i);
}

inline u32 nv_sor_head_state2_t19x(u32 i)
{
	return NV_HEAD_STATE2_T19X(i);
}

inline u32 nv_sor_head_state3_t19x(u32 i)
{
	return NV_HEAD_STATE3_T19X(i);
}

inline u32 nv_sor_head_state4_t19x(u32 i)
{
	return NV_HEAD_STATE4_T19X(i);
}

inline u32 nv_sor_head_state5_t19x(u32 i)
{
	return NV_HEAD_STATE5_T19X(i);
}

inline u32 nv_sor_pll0_t19x(void)
{
	return NV_SOR_PLL0_T19X;
}

inline u32 nv_sor_pll1_t19x(void)
{
	return NV_SOR_PLL1_T19X;
}

inline u32 nv_sor_pll2_t19x(void)
{
	return NV_SOR_PLL2_T19X;
}

inline u32 nv_sor_pll3_t19x(void)
{
	return NV_SOR_PLL3_T19X;
}

inline u32 nv_sor_pll4_t19x(void)
{
	return NV_SOR_PLL4_T19X;
}

inline u32 nv_sor_pll5_t19x(void)
{
	return NV_SOR_PLL5_T19X;
}

inline u32 nv_sor_dp_padctl_t19x(u32 i)
{
	return NV_SOR_DP_PADCTL_T19X(i);
}

inline u32 nv_sor_dp_misc1_override_t19x(void)
{
	return NV_SOR_DP_MISC1_OVERRIDE_T19X;
}

inline u32 nv_sor_dp_misc1_bit6_t19x(void)
{
	return NV_SOR_DP_MISC1_BIT6_T19X;
}

inline u32 nv_sor_dp_int_enable_t19x(void)
{
	return NV_SOR_DP_INT_ENABLE_T19X;
}

/* This function either blocks or unblocks the SOR AFIFO. */
inline void tegra_sor_clk_switch_setup_t19x(struct tegra_dc_sor_data *sor,
					bool unblock)
{
	tegra_sor_write_field(sor, NV_SOR_DP_LINKCTL(sor->portnum),
			NV_SOR_DP_LINKCTL_ASYNC_FIFO_BLOCK_MASK,
			(unblock ? NV_SOR_DP_LINKCTL_ASYNC_FIFO_BLOCK_NO :
			NV_SOR_DP_LINKCTL_ASYNC_FIFO_BLOCK_YES));
}

inline void tegra_sor_program_fpga_clk_mux_t19x(
					struct tegra_dc_sor_data *sor)
{
	struct tegra_dc *dc = sor->dc;
	u32 reg_val = 0;

	if (dc->out->type == TEGRA_DC_OUT_HDMI) {
		reg_val |= NV_SOR_FPGA_CLK_SEL_FPGA_PCLK_MUX_SEL_HDMI;
		if (tegra_dc_is_yuv420_8bpc(&dc->mode))
			reg_val |= NV_SOR_FPGA_CLK_SEL_FPGA_HDMI420_SEL_ENABLE;
	} else if (dc->out->type == TEGRA_DC_OUT_DP ||
			dc->out->type == TEGRA_DC_OUT_FAKE_DP) {
		reg_val |= NV_SOR_FPGA_CLK_SEL_FPGA_PCLK_MUX_SEL_DP;
	} else {
		return;
	}

	tegra_sor_writel(sor, NV_SOR_FPGA_CLK_SEL, reg_val);
}

inline u32 tegra_sor_yuv420_8bpc_pixel_depth_t19x(void)
{
	return (NV_SOR_STATE1_ASY_PIXELDEPTH_BPP_12_420 |
		NV_SOR_STATE1_ASY_CHROMA_V_DECIMATE_ENABLE);
}
