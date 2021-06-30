/*
 * general p state infrastructure
 *
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef NVGPU_CTRLPERF_H
#define NVGPU_CTRLPERF_H

struct ctrl_perf_volt_rail_list_item {
	u8 volt_domain;
	u32 voltage_uv;
	u32 voltage_min_noise_unaware_uv;
};

struct ctrl_perf_volt_rail_list {
	u8    num_rails;
	struct ctrl_perf_volt_rail_list_item
		rails[CTRL_VOLT_VOLT_RAIL_MAX_RAILS];
};

union ctrl_perf_vfe_var_single_sensed_fuse_value_data {
	int signed_value;
	u32 unsigned_value;
};

struct ctrl_perf_vfe_var_single_sensed_fuse_value {
	bool b_signed;
	union ctrl_perf_vfe_var_single_sensed_fuse_value_data data;
};

struct ctrl_bios_vfield_register_segment_super {
	u8 low_bit;
	u8 high_bit;
};

struct ctrl_bios_vfield_register_segment_reg {
	struct ctrl_bios_vfield_register_segment_super super;
	u32 addr;
};

struct ctrl_bios_vfield_register_segment_index_reg {
	struct ctrl_bios_vfield_register_segment_super super;
	u32 addr;
	u32 reg_index;
	u32 index;
};

union ctrl_bios_vfield_register_segment_data {
	struct ctrl_bios_vfield_register_segment_reg reg;
	struct ctrl_bios_vfield_register_segment_index_reg index_reg;
};

struct ctrl_bios_vfield_register_segment {
	u8 type;
	union ctrl_bios_vfield_register_segment_data data;
};

#define NV_PMU_VFE_VAR_SINGLE_SENSED_FUSE_SEGMENTS_MAX                         1

struct ctrl_perf_vfe_var_single_sensed_fuse_info {
	u8 segment_count;
	struct ctrl_bios_vfield_register_segment segments[NV_PMU_VFE_VAR_SINGLE_SENSED_FUSE_SEGMENTS_MAX];
};

struct ctrl_perf_vfe_var_single_sensed_fuse_override_info {
	u32 fuse_val_override;
        u8 b_fuse_regkey_override;
};

struct ctrl_perf_vfe_var_single_sensed_fuse_vfield_info {
	struct ctrl_perf_vfe_var_single_sensed_fuse_info fuse;
	u32 fuse_val_default;
	u32 hw_correction_scale;
	int hw_correction_offset;
	u8 v_field_id;
};

struct ctrl_perf_vfe_var_single_sensed_fuse_ver_vfield_info {
	struct ctrl_perf_vfe_var_single_sensed_fuse_info fuse;
	u8 ver_expected;
	bool b_ver_check;
	bool b_use_default_on_ver_check_fail;
	u8 v_field_id_ver;
};
#endif /* NVGPU_CTRLPERF_H */
