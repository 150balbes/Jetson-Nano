/*
 * general p state infrastructure
 *
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
#ifndef NVGPU_CTRLCLK_H
#define NVGPU_CTRLCLK_H

#include "ctrlboardobj.h"
#include "ctrlclkavfs.h"
#include "ctrlvolt.h"

#define CTRL_CLK_CLK_DELTA_MAX_VOLT_RAILS 4

/* valid clock domain values */
#define CTRL_CLK_DOMAIN_MCLK                                (0x00000010)
#define CTRL_CLK_DOMAIN_HOSTCLK                             (0x00000020)
#define CTRL_CLK_DOMAIN_DISPCLK                             (0x00000040)
#define CTRL_CLK_DOMAIN_GPC2CLK                             (0x00010000)
#define CTRL_CLK_DOMAIN_XBAR2CLK                            (0x00040000)
#define CTRL_CLK_DOMAIN_SYS2CLK                             (0x00800000)
#define CTRL_CLK_DOMAIN_HUB2CLK                             (0x01000000)
#define CTRL_CLK_DOMAIN_PWRCLK                              (0x00080000)
#define CTRL_CLK_DOMAIN_NVDCLK                              (0x00100000)
#define CTRL_CLK_DOMAIN_PCIEGENCLK                          (0x00200000)

#define CTRL_CLK_DOMAIN_GPCCLK                              (0x00000001)
#define CTRL_CLK_DOMAIN_XBARCLK                             (0x00000002)
#define CTRL_CLK_DOMAIN_SYSCLK                              (0x00000004)
#define CTRL_CLK_DOMAIN_HUBCLK                              (0x00000008)

#define CTRL_CLK_CLK_DOMAIN_TYPE_3X                                  0x01
#define CTRL_CLK_CLK_DOMAIN_TYPE_3X_FIXED                            0x02
#define CTRL_CLK_CLK_DOMAIN_TYPE_3X_PROG                             0x03
#define CTRL_CLK_CLK_DOMAIN_TYPE_3X_MASTER                           0x04
#define CTRL_CLK_CLK_DOMAIN_TYPE_3X_SLAVE                            0x05
#define CTRL_CLK_CLK_DOMAIN_TYPE_30_PROG                             0x06
#define CTRL_CLK_CLK_DOMAIN_TYPE_35_MASTER                           0x07
#define CTRL_CLK_CLK_DOMAIN_TYPE_35_SLAVE                            0x08
#define CTRL_CLK_CLK_DOMAIN_TYPE_35_PROG                             0x09

#define CTRL_CLK_CLK_DOMAIN_3X_PROG_ORDERING_INDEX_INVALID 0xFF
#define CTRL_CLK_CLK_DOMAIN_INDEX_INVALID                           0xFF

#define CTRL_CLK_CLK_PROG_TYPE_1X                              0x01
#define CTRL_CLK_CLK_PROG_TYPE_1X_MASTER                       0x02
#define CTRL_CLK_CLK_PROG_TYPE_1X_MASTER_RATIO                 0x03
#define CTRL_CLK_CLK_PROG_TYPE_1X_MASTER_TABLE                 0x04
#define CTRL_CLK_CLK_PROG_TYPE_UNKNOWN                         255

/*!
 * Enumeration of CLK_PROG source types.
 */
#define CTRL_CLK_PROG_1X_SOURCE_PLL                            0x00
#define CTRL_CLK_PROG_1X_SOURCE_ONE_SOURCE                     0x01
#define CTRL_CLK_PROG_1X_SOURCE_FLL                            0x02
#define CTRL_CLK_PROG_1X_SOURCE_INVALID                        255

#define CTRL_CLK_CLK_PROG_1X_MASTER_VF_ENTRY_MAX_ENTRIES 4
#define CTRL_CLK_PROG_1X_MASTER_MAX_SLAVE_ENTRIES 6

#define CTRL_CLK_CLK_VF_POINT_IDX_INVALID                      255

#define CTRL_CLK_CLK_VF_POINT_TYPE_FREQ                         0x01
#define CTRL_CLK_CLK_VF_POINT_TYPE_VOLT                         0x02
#define CTRL_CLK_CLK_VF_POINT_TYPE_UNKNOWN                      255

struct ctrl_clk_clk_prog_1x_master_source_fll {
	u32 base_vfsmooth_volt_uv;
	u32 max_vf_ramprate;
	u32 max_freq_stepsize_mhz;
};

union ctrl_clk_clk_prog_1x_master_source_data {
	struct ctrl_clk_clk_prog_1x_master_source_fll fll;
};

struct ctrl_clk_clk_vf_point_info_freq {
	u16 freq_mhz;
};

struct ctrl_clk_clk_vf_point_info_volt {
	u32  sourceVoltageuV;
	u8  vfGainVfeEquIdx;
	u8  clkDomainIdx;
};

struct ctrl_clk_clk_prog_1x_master_vf_entry {
	u8 vfe_idx;
	u8 gain_vfe_idx;
	u8 vf_point_idx_first;
	u8 vf_point_idx_last;
};

struct ctrl_clk_clk_prog_1x_master_ratio_slave_entry {
	u8 clk_dom_idx;
	u8 ratio;
};

struct ctrl_clk_clk_prog_1x_master_table_slave_entry {
	u8 clk_dom_idx;
	u16 freq_mhz;
};

struct ctrl_clk_clk_prog_1x_source_pll {
	u8 pll_idx;
	u8 freq_step_size_mhz;
};

union ctrl_clk_freq_delta_data {
	s32 delta_khz;
	s16 delta_percent;
};
struct ctrl_clk_freq_delta {
	u8 type;
	union ctrl_clk_freq_delta_data data;
};

struct ctrl_clk_clk_delta {
	struct ctrl_clk_freq_delta freq_delta;
	int volt_deltauv[CTRL_CLK_CLK_DELTA_MAX_VOLT_RAILS];
};

struct ctrl_clk_vin_v10 {
	u32 slope;
	u32 intercept;
};

struct ctrl_clk_vin_v20 {
	s8 offset;
	s8 gain;
};

union ctrl_clk_vin_data_v20 {
	struct ctrl_clk_vin_v10 cal_v10;
	struct ctrl_clk_vin_v20 cal_v20;
};

struct ctrl_clk_vin_device_info_data_v10 {
	struct ctrl_clk_vin_v10 vin_cal;
};

struct ctrl_clk_vin_device_info_data_v20 {
	u8 cal_type;
	union ctrl_clk_vin_data_v20 vin_cal;
};

union ctrl_clk_clk_prog_1x_source_data {
	struct ctrl_clk_clk_prog_1x_source_pll pll;
};

struct ctrl_clk_vf_pair {
	u16 freq_mhz;
	u32 voltage_uv;
};

struct ctrl_clk_clk_domain_list_item {
	u32  clk_domain;
	u32  clk_freq_khz;
	u32  clk_flags;
	u8   current_regime_id;
	u8   target_regime_id;
};

struct ctrl_clk_clk_domain_list_item_v1 {
	u32  clk_domain;
	u32  clk_freq_khz;
	u8   regime_id;
	u8   source;
};

struct ctrl_clk_clk_domain_list {
	u8 num_domains;
	struct ctrl_clk_clk_domain_list_item_v1
		clk_domains[CTRL_BOARDOBJ_MAX_BOARD_OBJECTS];
};

#define CTRL_CLK_VF_PAIR_FREQ_MHZ_GET(pvfpair)                          \
	((pvfpair)->freq_mhz)

#define CTRL_CLK_VF_PAIR_VOLTAGE_UV_GET(pvfpair)                        \
	((pvfpair)->voltage_uv)

#define CTRL_CLK_VF_PAIR_FREQ_MHZ_SET(pvfpair, _freqmhz)                \
	(((pvfpair)->freq_mhz) = (_freqmhz))

#define CTRL_CLK_VF_PAIR_FREQ_MHZ_SET(pvfpair, _freqmhz)                \
	(((pvfpair)->freq_mhz) = (_freqmhz))


#define CTRL_CLK_VF_PAIR_VOLTAGE_UV_SET(pvfpair, _voltageuv)	        \
	(((pvfpair)->voltage_uv) = (_voltageuv))

#endif /* NVGPU_CTRLCLK_H */
