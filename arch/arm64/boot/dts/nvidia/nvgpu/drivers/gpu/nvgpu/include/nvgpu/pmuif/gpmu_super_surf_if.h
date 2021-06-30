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
#ifndef NVGPU_PMUIF_GPMU_SUPER_SURF_IF_H
#define NVGPU_PMUIF_GPMU_SUPER_SURF_IF_H

struct nv_pmu_super_surface_hdr {
    u32 memberMask;
    u16 dmemBufferSizeMax;
};

NV_PMU_MAKE_ALIGNED_STRUCT(nv_pmu_super_surface_hdr,
		sizeof(struct nv_pmu_super_surface_hdr));

/*
 * Global Super Surface structure for combined INIT data required by PMU.
 * NOTE: Any new substructures or entries must be aligned.
 */
struct nv_pmu_super_surface {
	union nv_pmu_super_surface_hdr_aligned hdr;

	struct {
		struct nv_pmu_volt_volt_device_boardobj_grp_set	volt_device_grp_set;
		struct nv_pmu_volt_volt_policy_boardobj_grp_set	volt_policy_grp_set;
		struct nv_pmu_volt_volt_rail_boardobj_grp_set volt_rail_grp_set;

		struct nv_pmu_volt_volt_policy_boardobj_grp_get_status	volt_policy_grp_get_status;
		struct nv_pmu_volt_volt_rail_boardobj_grp_get_status	volt_rail_grp_get_status;
		struct nv_pmu_volt_volt_device_boardobj_grp_get_status	volt_device_grp_get_status;
	} volt;
	struct  {
		struct nv_pmu_clk_clk_vin_device_boardobj_grp_set clk_vin_device_grp_set;
		struct nv_pmu_clk_clk_domain_boardobj_grp_set clk_domain_grp_set;
		struct nv_pmu_clk_clk_freq_controller_boardobj_grp_set clk_freq_controller_grp_set;
		u8 clk_rsvd2[0x200];
		struct nv_pmu_clk_clk_fll_device_boardobj_grp_set clk_fll_device_grp_set;
		struct nv_pmu_clk_clk_prog_boardobj_grp_set clk_prog_grp_set;
		struct nv_pmu_clk_clk_vf_point_boardobj_grp_set clk_vf_point_grp_set;
		struct nv_pmu_clk_clk_vin_device_boardobj_grp_get_status clk_vin_device_grp_get_status;
		struct nv_pmu_clk_clk_fll_device_boardobj_grp_get_status clk_fll_device_grp_get_status;
		struct nv_pmu_clk_clk_vf_point_boardobj_grp_get_status clk_vf_point_grp_get_status;
		u8 clk_rsvd[0x4660];
	} clk;
	struct {
		struct nv_pmu_perf_vfe_equ_boardobj_grp_set vfe_equ_grp_set;
		struct nv_pmu_perf_vfe_var_boardobj_grp_set vfe_var_grp_set;

		struct nv_pmu_perf_vfe_var_boardobj_grp_get_status vfe_var_grp_get_status;
		u8 perf_rsvd[0x40790];
		u8 perfcf_rsvd[0x1eb0];
	} perf;
	struct {
		struct nv_pmu_therm_therm_channel_boardobj_grp_set therm_channel_grp_set;
		struct nv_pmu_therm_therm_device_boardobj_grp_set therm_device_grp_set;
		u8 therm_rsvd[0x1460];
	} therm;
};

#endif /* NVGPU_PMUIF_GPMU_SUPER_SURF_IF_H */
