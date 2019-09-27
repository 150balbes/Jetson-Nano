/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef NVGPU_PMUIF_GPMUIF_PG_H
#define NVGPU_PMUIF_GPMUIF_PG_H

#include "gpmuif_ap.h"
#include "gpmuif_pg_rppg.h"

/*PG defines*/

/* Identifier for each PG */
#define PMU_PG_ELPG_ENGINE_ID_GRAPHICS 		(0x00000000U)
#define PMU_PG_ELPG_ENGINE_ID_MS       		(0x00000004U)
#define PMU_PG_ELPG_ENGINE_ID_INVALID_ENGINE 	(0x00000005U)
#define PMU_PG_ELPG_ENGINE_MAX    PMU_PG_ELPG_ENGINE_ID_INVALID_ENGINE

/* PG message */
enum {
	PMU_PG_ELPG_MSG_INIT_ACK,
	PMU_PG_ELPG_MSG_DISALLOW_ACK,
	PMU_PG_ELPG_MSG_ALLOW_ACK,
	PMU_PG_ELPG_MSG_FREEZE_ACK,
	PMU_PG_ELPG_MSG_FREEZE_ABORT,
	PMU_PG_ELPG_MSG_UNFREEZE_ACK,
};

struct pmu_pg_msg_elpg_msg {
	u8 msg_type;
	u8 engine_id;
	u16 msg;
};

enum {
	PMU_PG_STAT_MSG_RESP_DMEM_OFFSET = 0,
};

struct pmu_pg_msg_stat {
	u8 msg_type;
	u8 engine_id;
	u16 sub_msg_id;
	u32 data;
};

enum {
	PMU_PG_MSG_ENG_BUF_LOADED,
	PMU_PG_MSG_ENG_BUF_UNLOADED,
	PMU_PG_MSG_ENG_BUF_FAILED,
};

struct pmu_pg_msg_eng_buf_stat {
	u8 msg_type;
	u8 engine_id;
	u8 buf_idx;
	u8 status;
};

struct pmu_pg_msg {
	union {
		u8 msg_type;
		struct pmu_pg_msg_elpg_msg elpg_msg;
		struct pmu_pg_msg_stat stat;
		struct pmu_pg_msg_eng_buf_stat eng_buf_stat;
		/* TBD: other pg messages */
		union pmu_ap_msg ap_msg;
		struct nv_pmu_rppg_msg rppg_msg;
	};
};

/* PG commands */
enum {
	PMU_PG_ELPG_CMD_INIT,
	PMU_PG_ELPG_CMD_DISALLOW,
	PMU_PG_ELPG_CMD_ALLOW,
	PMU_PG_ELPG_CMD_FREEZE,
	PMU_PG_ELPG_CMD_UNFREEZE,
};

enum {
	PMU_PG_CMD_ID_ELPG_CMD = 0,
	PMU_PG_CMD_ID_ENG_BUF_LOAD,
	PMU_PG_CMD_ID_ENG_BUF_UNLOAD,
	PMU_PG_CMD_ID_PG_STAT,
	PMU_PG_CMD_ID_PG_LOG_INIT,
	PMU_PG_CMD_ID_PG_LOG_FLUSH,
	PMU_PG_CMD_ID_PG_PARAM,
	PMU_PG_CMD_ID_ELPG_INIT,
	PMU_PG_CMD_ID_ELPG_POLL_CTXSAVE,
	PMU_PG_CMD_ID_ELPG_ABORT_POLL,
	PMU_PG_CMD_ID_ELPG_PWR_UP,
	PMU_PG_CMD_ID_ELPG_DISALLOW,
	PMU_PG_CMD_ID_ELPG_ALLOW,
	PMU_PG_CMD_ID_AP,
	RM_PMU_PG_CMD_ID_PSI,
	RM_PMU_PG_CMD_ID_CG,
	PMU_PG_CMD_ID_ZBC_TABLE_UPDATE,
	PMU_PG_CMD_ID_PWR_RAIL_GATE_DISABLE = 0x20,
	PMU_PG_CMD_ID_PWR_RAIL_GATE_ENABLE,
	PMU_PG_CMD_ID_PWR_RAIL_SMU_MSG_DISABLE,
	PMU_PMU_PG_CMD_ID_RPPG = 0x24,
};

enum {
	PMU_PG_STAT_CMD_ALLOC_DMEM = 0,
};

enum {
	SLOWDOWN_FACTOR_FPDIV_BY1 = 0,
	SLOWDOWN_FACTOR_FPDIV_BY1P5,
	SLOWDOWN_FACTOR_FPDIV_BY2,
	SLOWDOWN_FACTOR_FPDIV_BY2P5,
	SLOWDOWN_FACTOR_FPDIV_BY3,
	SLOWDOWN_FACTOR_FPDIV_BY3P5,
	SLOWDOWN_FACTOR_FPDIV_BY4,
	SLOWDOWN_FACTOR_FPDIV_BY4P5,
	SLOWDOWN_FACTOR_FPDIV_BY5,
	SLOWDOWN_FACTOR_FPDIV_BY5P5,
	SLOWDOWN_FACTOR_FPDIV_BY6,
	SLOWDOWN_FACTOR_FPDIV_BY6P5,
	SLOWDOWN_FACTOR_FPDIV_BY7,
	SLOWDOWN_FACTOR_FPDIV_BY7P5,
	SLOWDOWN_FACTOR_FPDIV_BY8,
	SLOWDOWN_FACTOR_FPDIV_BY8P5,
	SLOWDOWN_FACTOR_FPDIV_BY9,
	SLOWDOWN_FACTOR_FPDIV_BY9P5,
	SLOWDOWN_FACTOR_FPDIV_BY10,
	SLOWDOWN_FACTOR_FPDIV_BY10P5,
	SLOWDOWN_FACTOR_FPDIV_BY11,
	SLOWDOWN_FACTOR_FPDIV_BY11P5,
	SLOWDOWN_FACTOR_FPDIV_BY12,
	SLOWDOWN_FACTOR_FPDIV_BY12P5,
	SLOWDOWN_FACTOR_FPDIV_BY13,
	SLOWDOWN_FACTOR_FPDIV_BY13P5,
	SLOWDOWN_FACTOR_FPDIV_BY14,
	SLOWDOWN_FACTOR_FPDIV_BY14P5,
	SLOWDOWN_FACTOR_FPDIV_BY15,
	SLOWDOWN_FACTOR_FPDIV_BY15P5,
	SLOWDOWN_FACTOR_FPDIV_BY16,
	SLOWDOWN_FACTOR_FPDIV_BY16P5,
	SLOWDOWN_FACTOR_FPDIV_BY17 = 0x20,
	SLOWDOWN_FACTOR_FPDIV_BY18 = 0x22,
	SLOWDOWN_FACTOR_FPDIV_BY19 = 0x24,
	SLOWDOWN_FACTOR_FPDIV_BY20 = 0x26,
	SLOWDOWN_FACTOR_FPDIV_BY21 = 0x28,
	SLOWDOWN_FACTOR_FPDIV_BY22 = 0x2a,
	SLOWDOWN_FACTOR_FPDIV_BY23 = 0x2c,
	SLOWDOWN_FACTOR_FPDIV_BY24 = 0x2e,
	SLOWDOWN_FACTOR_FPDIV_BY25 = 0x30,
	SLOWDOWN_FACTOR_FPDIV_BY26 = 0x32,
	SLOWDOWN_FACTOR_FPDIV_BY27 = 0x34,
	SLOWDOWN_FACTOR_FPDIV_BY28 = 0x36,
	SLOWDOWN_FACTOR_FPDIV_BY29 = 0x38,
	SLOWDOWN_FACTOR_FPDIV_BY30 = 0x3a,
	SLOWDOWN_FACTOR_FPDIV_BY31 = 0x3c,
	SLOWDOWN_FACTOR_FPDIV_BYMAX,
};

#define PMU_PG_PARAM_CMD_GR_INIT_PARAM  		0x0U
#define PMU_PG_PARAM_CMD_MS_INIT_PARAM  		0x01U
#define PMU_PG_PARAM_CMD_MCLK_CHANGE  			0x04U
#define PMU_PG_PARAM_CMD_POST_INIT  			0x06U
#define PMU_PG_PARAM_CMD_SUB_FEATURE_MASK_UPDATE  	0x07U

#define NVGPU_PMU_GR_FEATURE_MASK_SDIV_SLOWDOWN     BIT32(0)
#define NVGPU_PMU_GR_FEATURE_MASK_POWER_GATING      BIT32(2)
#define NVGPU_PMU_GR_FEATURE_MASK_RPPG              BIT32(3)
#define NVGPU_PMU_GR_FEATURE_MASK_PRIV_RING         BIT32(5)
#define NVGPU_PMU_GR_FEATURE_MASK_UNBIND            BIT32(6)
#define NVGPU_PMU_GR_FEATURE_MASK_SAVE_GLOBAL_STATE BIT32(7)
#define NVGPU_PMU_GR_FEATURE_MASK_RESET_ENTRY       BIT32(8)
#define NVGPU_PMU_GR_FEATURE_MASK_HW_SEQUENCE       BIT32(9)
#define NVGPU_PMU_GR_FEATURE_MASK_ELPG_SRAM         BIT32(10)
#define NVGPU_PMU_GR_FEATURE_MASK_ELPG_LOGIC        BIT32(11)
#define NVGPU_PMU_GR_FEATURE_MASK_ELPG_L2RPPG       BIT32(12)

#define NVGPU_PMU_GR_FEATURE_MASK_ALL	\
	(	\
		NVGPU_PMU_GR_FEATURE_MASK_SDIV_SLOWDOWN         |\
		NVGPU_PMU_GR_FEATURE_MASK_POWER_GATING          |\
		NVGPU_PMU_GR_FEATURE_MASK_RPPG                  |\
		NVGPU_PMU_GR_FEATURE_MASK_PRIV_RING             |\
		NVGPU_PMU_GR_FEATURE_MASK_UNBIND                |\
		NVGPU_PMU_GR_FEATURE_MASK_SAVE_GLOBAL_STATE     |\
		NVGPU_PMU_GR_FEATURE_MASK_RESET_ENTRY           |\
		NVGPU_PMU_GR_FEATURE_MASK_HW_SEQUENCE           |\
		NVGPU_PMU_GR_FEATURE_MASK_ELPG_SRAM             |\
		NVGPU_PMU_GR_FEATURE_MASK_ELPG_LOGIC            |\
		NVGPU_PMU_GR_FEATURE_MASK_ELPG_L2RPPG           \
	)

#define NVGPU_PMU_MS_FEATURE_MASK_CLOCK_GATING  BIT32(0)
#define NVGPU_PMU_MS_FEATURE_MASK_SW_ASR        BIT32(1)
#define NVGPU_PMU_MS_FEATURE_MASK_RPPG          BIT32(8)
#define NVGPU_PMU_MS_FEATURE_MASK_FB_TRAINING   BIT32(5)

#define NVGPU_PMU_MS_FEATURE_MASK_ALL	\
	(	\
		NVGPU_PMU_MS_FEATURE_MASK_CLOCK_GATING  |\
		NVGPU_PMU_MS_FEATURE_MASK_SW_ASR        |\
		NVGPU_PMU_MS_FEATURE_MASK_RPPG          |\
		NVGPU_PMU_MS_FEATURE_MASK_FB_TRAINING   \
	)


struct pmu_pg_cmd_elpg_cmd {
	u8 cmd_type;
	u8 engine_id;
	u16 cmd;
};

struct pmu_pg_cmd_eng_buf_load_v0 {
	u8 cmd_type;
	u8 engine_id;
	u8 buf_idx;
	u8 pad;
	u16 buf_size;
	u32 dma_base;
	u8 dma_offset;
	u8 dma_idx;
};

struct pmu_pg_cmd_eng_buf_load_v1 {
	u8 cmd_type;
	u8 engine_id;
	u8 buf_idx;
	u8 pad;
	struct flcn_mem_desc {
		struct falc_u64 dma_addr;
		u16 dma_size;
		u8 dma_idx;
	} dma_desc;
};

struct pmu_pg_cmd_eng_buf_load_v2 {
	u8 cmd_type;
	u8 engine_id;
	u8 buf_idx;
	u8 pad;
	struct flcn_mem_desc_v0 dma_desc;
};

struct pmu_pg_cmd_gr_init_param {
	u8 cmd_type;
	u16 sub_cmd_id;
	u8 featuremask;
};

struct pmu_pg_cmd_gr_init_param_v2 {
	u8 cmd_type;
	u16 sub_cmd_id;
	u8 featuremask;
	u8 ldiv_slowdown_factor;
};

struct pmu_pg_cmd_gr_init_param_v1 {
	u8 cmd_type;
	u16 sub_cmd_id;
	u32 featuremask;
};

struct pmu_pg_cmd_sub_feature_mask_update {
	u8 cmd_type;
	u16 sub_cmd_id;
	u8 ctrl_id;
	u32 enabled_mask;
};

struct pmu_pg_cmd_ms_init_param {
	u8 cmd_type;
	u16 cmd_id;
	u8 psi;
	u8 idle_flipped_test_enabled;
	u16 psiSettleTimeUs;
	u8 rsvd[2];
	u32 support_mask;
	u32 abort_timeout_us;
};

struct pmu_pg_cmd_mclk_change {
	u8 cmd_type;
	u16 cmd_id;
	u8 rsvd;
	u32 data;
};

#define PG_VOLT_RAIL_IDX_MAX 2

struct pmu_pg_volt_rail {
	u8    volt_rail_idx;
	u8    sleep_volt_dev_idx;
	u8    sleep_vfe_idx;
	u32   sleep_voltage_uv;
	u32   therm_vid0_cache;
	u32   therm_vid1_cache;
};

struct pmu_pg_cmd_post_init_param {
	u8 cmd_type;
	u16 cmd_id;
	struct pmu_pg_volt_rail pg_volt_rail[PG_VOLT_RAIL_IDX_MAX];
};

struct pmu_pg_cmd_stat {
	u8 cmd_type;
	u8 engine_id;
	u16 sub_cmd_id;
	u32 data;
};

struct pmu_pg_cmd {
	union {
		u8 cmd_type;
		struct pmu_pg_cmd_elpg_cmd elpg_cmd;
		struct pmu_pg_cmd_eng_buf_load_v0 eng_buf_load_v0;
		struct pmu_pg_cmd_eng_buf_load_v1 eng_buf_load_v1;
		struct pmu_pg_cmd_eng_buf_load_v2 eng_buf_load_v2;
		struct pmu_pg_cmd_stat stat;
		struct pmu_pg_cmd_gr_init_param gr_init_param;
		struct pmu_pg_cmd_gr_init_param_v1 gr_init_param_v1;
		struct pmu_pg_cmd_gr_init_param_v2 gr_init_param_v2;
		struct pmu_pg_cmd_ms_init_param ms_init_param;
		struct pmu_pg_cmd_mclk_change mclk_change;
		struct pmu_pg_cmd_post_init_param post_init;
		/* TBD: other pg commands */
		union pmu_ap_cmd ap_cmd;
		struct nv_pmu_rppg_cmd rppg_cmd;
		struct pmu_pg_cmd_sub_feature_mask_update sf_mask_update;
	};
};

/* Statistics structure for PG features */
struct pmu_pg_stats_v2 {
	u32 entry_count;
	u32 exit_count;
	u32 abort_count;
	u32 detection_count;
	u32 prevention_activate_count;
	u32 prevention_deactivate_count;
	u32 powered_up_time_us;
	u32 entry_latency_us;
	u32 exit_latency_us;
	u32 resident_time_us;
	u32 entry_latency_avg_us;
	u32 exit_latency_avg_us;
	u32 entry_latency_max_us;
	u32 exit_latency_max_us;
	u32 total_sleep_time_us;
	u32 total_non_sleep_time_us;
};

struct pmu_pg_stats_v1 {
	/* Number of time PMU successfully engaged sleep state */
	u32 entry_count;
	/* Number of time PMU exit sleep state */
	u32 exit_count;
	/* Number of time PMU aborted in entry sequence */
	u32 abort_count;
	/*
	* Time for which GPU was neither in Sleep state not
	* executing sleep sequence.
	*/
	u32 poweredup_timeus;
	/* Entry and exit latency of current sleep cycle */
	u32 entry_latency_us;
	u32 exitlatencyus;
	/* Resident time for current sleep cycle. */
	u32 resident_timeus;
	/* Rolling average entry and exit latencies */
	u32 entrylatency_avgus;
	u32 exitlatency_avgus;
	/* Max entry and exit latencies */
	u32 entrylatency_maxus;
	u32 exitlatency_maxus;
	/* Total time spent in sleep and non-sleep state */
	u32 total_sleep_timeus;
	u32 total_nonsleep_timeus;
};

struct pmu_pg_stats {
	u64 pg_entry_start_timestamp;
	u64 pg_ingating_start_timestamp;
	u64 pg_exit_start_timestamp;
	u64 pg_ungating_start_timestamp;
	u32 pg_avg_entry_time_us;
	u32 pg_ingating_cnt;
	u32 pg_ingating_time_us;
	u32 pg_avg_exit_time_us;
	u32 pg_ungating_count;
	u32 pg_ungating_time_us;
	u32 pg_gating_cnt;
	u32 pg_gating_deny_cnt;
};

#endif /* NVGPU_PMUIF_GPMUIF_PG_H*/
