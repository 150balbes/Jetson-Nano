/*
 * Tegra GPU Virtualization Interfaces to Server
 *
 * Copyright (c) 2014-2020, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef TEGRA_VGPU_H
#define TEGRA_VGPU_H

#include <nvgpu/types.h>
#include <nvgpu/ecc.h>	/* For NVGPU_ECC_STAT_NAME_MAX_SIZE */

enum {
	TEGRA_VGPU_MODULE_GPU = 0,
};

enum {
	/* Needs to follow last entry in TEGRA_VHOST_QUEUE_* list,
	 * in tegra_vhost.h
	 */
	TEGRA_VGPU_QUEUE_CMD = 3,
	TEGRA_VGPU_QUEUE_INTR
};

enum {
	TEGRA_VGPU_CMD_CONNECT = 0,
	TEGRA_VGPU_CMD_DISCONNECT = 1,
	TEGRA_VGPU_CMD_ABORT = 2,
	TEGRA_VGPU_CMD_CHANNEL_ALLOC_HWCTX = 3,
	TEGRA_VGPU_CMD_CHANNEL_FREE_HWCTX = 4,
	TEGRA_VGPU_CMD_GET_ATTRIBUTE = 5,
	TEGRA_VGPU_CMD_MAP_BAR1 = 6,
	TEGRA_VGPU_CMD_AS_ALLOC_SHARE = 7,
	TEGRA_VGPU_CMD_AS_BIND_SHARE = 8,
	TEGRA_VGPU_CMD_AS_FREE_SHARE = 9,
	TEGRA_VGPU_CMD_AS_UNMAP = 11,
	TEGRA_VGPU_CMD_CHANNEL_BIND = 13,
	TEGRA_VGPU_CMD_CHANNEL_UNBIND = 14,
	TEGRA_VGPU_CMD_CHANNEL_DISABLE = 15,
	TEGRA_VGPU_CMD_CHANNEL_PREEMPT = 16,
	TEGRA_VGPU_CMD_CHANNEL_SETUP_RAMFC = 17,
	TEGRA_VGPU_CMD_CHANNEL_COMMIT_GR_CTX = 20,
	TEGRA_VGPU_CMD_CHANNEL_ALLOC_GR_PATCH_CTX = 21,
	TEGRA_VGPU_CMD_CHANNEL_FREE_GR_PATCH_CTX = 22,
	TEGRA_VGPU_CMD_CHANNEL_MAP_GR_GLOBAL_CTX = 23,
	TEGRA_VGPU_CMD_CHANNEL_UNMAP_GR_GLOBAL_CTX = 24,
	TEGRA_VGPU_CMD_CHANNEL_COMMIT_GR_GLOBAL_CTX = 25,
	TEGRA_VGPU_CMD_CHANNEL_LOAD_GR_GOLDEN_CTX = 26,
	TEGRA_VGPU_CMD_CHANNEL_BIND_ZCULL = 27,
	TEGRA_VGPU_CMD_CACHE_MAINT = 28,
	TEGRA_VGPU_CMD_SUBMIT_RUNLIST = 29,
	TEGRA_VGPU_CMD_GET_ZCULL_INFO = 30,
	TEGRA_VGPU_CMD_ZBC_SET_TABLE = 31,
	TEGRA_VGPU_CMD_ZBC_QUERY_TABLE = 32,
	TEGRA_VGPU_CMD_AS_MAP_EX = 33,
	TEGRA_VGPU_CMD_CHANNEL_BIND_GR_CTXSW_BUFFERS = 34,
	TEGRA_VGPU_CMD_SET_MMU_DEBUG_MODE = 35,
	TEGRA_VGPU_CMD_SET_SM_DEBUG_MODE = 36,
	TEGRA_VGPU_CMD_REG_OPS = 37,
	TEGRA_VGPU_CMD_CHANNEL_SET_PRIORITY = 38,
	TEGRA_VGPU_CMD_CHANNEL_SET_RUNLIST_INTERLEAVE = 39,
	TEGRA_VGPU_CMD_CHANNEL_SET_TIMESLICE = 40,
	TEGRA_VGPU_CMD_FECS_TRACE_ENABLE = 41,
	TEGRA_VGPU_CMD_FECS_TRACE_DISABLE = 42,
	TEGRA_VGPU_CMD_FECS_TRACE_POLL = 43,
	TEGRA_VGPU_CMD_FECS_TRACE_SET_FILTER = 44,
	TEGRA_VGPU_CMD_CHANNEL_SET_SMPC_CTXSW_MODE = 45,
	TEGRA_VGPU_CMD_CHANNEL_SET_HWPM_CTXSW_MODE = 46,
	TEGRA_VGPU_CMD_CHANNEL_FREE_HWPM_CTX = 47,
	TEGRA_VGPU_CMD_GR_CTX_ALLOC = 48,
	TEGRA_VGPU_CMD_GR_CTX_FREE = 49,
	TEGRA_VGPU_CMD_CHANNEL_BIND_GR_CTX = 50,
	TEGRA_VGPU_CMD_TSG_BIND_GR_CTX = 51,
	TEGRA_VGPU_CMD_TSG_BIND_CHANNEL = 52,
	TEGRA_VGPU_CMD_TSG_UNBIND_CHANNEL = 53,
	TEGRA_VGPU_CMD_TSG_PREEMPT = 54,
	TEGRA_VGPU_CMD_TSG_SET_TIMESLICE = 55,
	TEGRA_VGPU_CMD_TSG_SET_RUNLIST_INTERLEAVE = 56,
	TEGRA_VGPU_CMD_CHANNEL_FORCE_RESET = 57,
	TEGRA_VGPU_CMD_CHANNEL_ENABLE = 58,
	TEGRA_VGPU_CMD_READ_PTIMER = 59,
	TEGRA_VGPU_CMD_SET_POWERGATE = 60,
	TEGRA_VGPU_CMD_SET_GPU_CLK_RATE = 61,
	TEGRA_VGPU_CMD_GET_CONSTANTS = 62,
	TEGRA_VGPU_CMD_CHANNEL_CYCLESTATS_SNAPSHOT = 63,
	TEGRA_VGPU_CMD_TSG_OPEN = 64,
	TEGRA_VGPU_CMD_GET_GPU_LOAD = 65,
	TEGRA_VGPU_CMD_SUSPEND_CONTEXTS = 66,
	TEGRA_VGPU_CMD_RESUME_CONTEXTS = 67,
	TEGRA_VGPU_CMD_CLEAR_SM_ERROR_STATE = 68,
	TEGRA_VGPU_CMD_GET_GPU_CLK_RATE = 69,
	TEGRA_VGPU_CMD_GET_GPU_FREQ_TABLE = 70,
	TEGRA_VGPU_CMD_CAP_GPU_CLK_RATE = 71,
	TEGRA_VGPU_CMD_PROF_MGT = 72,
	TEGRA_VGPU_CMD_PERFBUF_MGT = 73,
	TEGRA_VGPU_CMD_GET_TIMESTAMPS_ZIPPER = 74,
	TEGRA_VGPU_CMD_TSG_RELEASE = 75,
	TEGRA_VGPU_CMD_GET_VSMS_MAPPING = 76,
	TEGRA_VGPU_CMD_ALLOC_CTX_HEADER = 77,
	TEGRA_VGPU_CMD_FREE_CTX_HEADER = 78,
	TEGRA_VGPU_CMD_MAP_SYNCPT = 79,
	TEGRA_VGPU_CMD_TSG_BIND_CHANNEL_EX = 80,
	TEGRA_VGPU_CMD_UPDATE_PC_SAMPLING = 81,
	TEGRA_VGPU_CMD_SUSPEND = 82,
	TEGRA_VGPU_CMD_RESUME = 83,
	TEGRA_VGPU_CMD_GET_ECC_INFO = 84,
	TEGRA_VGPU_CMD_GET_ECC_COUNTER_VALUE = 85,
	TEGRA_VGPU_CMD_FB_SET_MMU_DEBUG_MODE = 88,
	TEGRA_VGPU_CMD_GR_SET_MMU_DEBUG_MODE = 89,
};

struct tegra_vgpu_connect_params {
	u32 module;
	u64 handle;
};

struct tegra_vgpu_channel_hwctx_params {
	u32 id;
	u64 pid;
	u64 handle;
};

struct tegra_vgpu_attrib_params {
	u32 attrib;
	u32 value;
};

struct tegra_vgpu_as_share_params {
	u64 size;
	u64 handle;
	u32 big_page_size;
};

struct tegra_vgpu_as_bind_share_params {
	u64 as_handle;
	u64 chan_handle;
};

enum {
	TEGRA_VGPU_MAP_PROT_NONE = 0,
	TEGRA_VGPU_MAP_PROT_READ_ONLY,
	TEGRA_VGPU_MAP_PROT_WRITE_ONLY
};

struct tegra_vgpu_as_map_params {
	u64 handle;
	u64 addr;
	u64 gpu_va;
	u64 size;
	u8 pgsz_idx;
	u8 iova;
	u8 kind;
	u8 cacheable;
	u8 clear_ctags;
	u8 prot;
	u32 ctag_offset;
};

#define TEGRA_VGPU_MAP_CACHEABLE	(1 << 0)
#define TEGRA_VGPU_MAP_IO_COHERENT	(1 << 1)
#define TEGRA_VGPU_MAP_L3_ALLOC		(1 << 2)
#define TEGRA_VGPU_MAP_PLATFORM_ATOMIC	(1 << 3)

struct tegra_vgpu_as_map_ex_params {
	u64 handle;
	u64 gpu_va;
	u64 size;
	u32 mem_desc_count;
	u8 pgsz_idx;
	u8 iova;
	u8 kind;
	u32 flags;
	u8 clear_ctags;
	u8 prot;
	u32 ctag_offset;
};

struct tegra_vgpu_mem_desc {
	u64 addr;
	u64 length;
};

struct tegra_vgpu_channel_config_params {
	u64 handle;
};

struct tegra_vgpu_ramfc_params {
	u64 handle;
	u64 gpfifo_va;
	u32 num_entries;
	u64 userd_addr;
	u8 iova;
};

struct tegra_vgpu_ch_ctx_params {
	u64 handle;
	u64 gr_ctx_va;
	u64 patch_ctx_va;
	u64 cb_va;
	u64 attr_va;
	u64 page_pool_va;
	u64 priv_access_map_va;
	u64 fecs_trace_va;
	u32 class_num;
};

struct tegra_vgpu_zcull_bind_params {
	u64 handle;
	u64 zcull_va;
	u32 mode;
};

enum {
	TEGRA_VGPU_L2_MAINT_FLUSH = 0,
	TEGRA_VGPU_L2_MAINT_INV,
	TEGRA_VGPU_L2_MAINT_FLUSH_INV,
	TEGRA_VGPU_FB_FLUSH
};

struct tegra_vgpu_cache_maint_params {
	u8 op;
};

struct tegra_vgpu_runlist_params {
	u8 runlist_id;
	u32 num_entries;
};

struct tegra_vgpu_golden_ctx_params {
	u32 size;
};

struct tegra_vgpu_zcull_info_params {
	u32 width_align_pixels;
	u32 height_align_pixels;
	u32 pixel_squares_by_aliquots;
	u32 aliquot_total;
	u32 region_byte_multiplier;
	u32 region_header_size;
	u32 subregion_header_size;
	u32 subregion_width_align_pixels;
	u32 subregion_height_align_pixels;
	u32 subregion_count;
};

#define TEGRA_VGPU_ZBC_COLOR_VALUE_SIZE		4
#define TEGRA_VGPU_ZBC_TYPE_INVALID		0
#define TEGRA_VGPU_ZBC_TYPE_COLOR		1
#define TEGRA_VGPU_ZBC_TYPE_DEPTH		2

struct tegra_vgpu_zbc_set_table_params {
	u32 color_ds[TEGRA_VGPU_ZBC_COLOR_VALUE_SIZE];
	u32 color_l2[TEGRA_VGPU_ZBC_COLOR_VALUE_SIZE];
	u32 depth;
	u32 format;
	u32 type;     /* color or depth */
};

struct tegra_vgpu_zbc_query_table_params {
	u32 color_ds[TEGRA_VGPU_ZBC_COLOR_VALUE_SIZE];
	u32 color_l2[TEGRA_VGPU_ZBC_COLOR_VALUE_SIZE];
	u32 depth;
	u32 ref_cnt;
	u32 format;
	u32 type;             /* color or depth */
	u32 index_size;       /* [out] size, [in] index */
};

enum {
	TEGRA_VGPU_GR_BIND_CTXSW_BUFFER_MAIN,
	TEGRA_VGPU_GR_BIND_CTXSW_BUFFER_SPILL,
	TEGRA_VGPU_GR_BIND_CTXSW_BUFFER_PAGEPOOL,
	TEGRA_VGPU_GR_BIND_CTXSW_BUFFER_BETACB,
	TEGRA_VGPU_GR_BIND_CTXSW_BUFFER_LAST
};

enum {
	TEGRA_VGPU_GR_CTXSW_PREEMPTION_MODE_WFI,
	TEGRA_VGPU_GR_CTXSW_PREEMPTION_MODE_GFX_GFXP,
	TEGRA_VGPU_GR_CTXSW_PREEMPTION_MODE_COMPUTE_CTA,
	TEGRA_VGPU_GR_CTXSW_PREEMPTION_MODE_COMPUTE_CILP,
	TEGRA_VGPU_GR_CTXSW_PREEMPTION_MODE_LAST
};

struct tegra_vgpu_gr_bind_ctxsw_buffers_params {
	u64 handle;	/* deprecated */
	u64 gpu_va[TEGRA_VGPU_GR_BIND_CTXSW_BUFFER_LAST];
	u64 size[TEGRA_VGPU_GR_BIND_CTXSW_BUFFER_LAST];
	u32 mode;
	u64 gr_ctx_handle;
};

struct tegra_vgpu_mmu_debug_mode {
	u32 enable;
};

struct tegra_vgpu_sm_debug_mode {
	u64 handle;
	u64 sms;
	u32 enable;
};

struct tegra_vgpu_reg_op {
	u8 op;
	u8 type;
	u8 status;
	u8 quad;
	u32 group_mask;
	u32 sub_group_mask;
	u32 offset;
	u32 value_lo;
	u32 value_hi;
	u32 and_n_mask_lo;
	u32 and_n_mask_hi;
};

struct tegra_vgpu_reg_ops_params {
	u64 handle;
	u64 num_ops;
	u32 is_profiler;
};

struct tegra_vgpu_channel_priority_params {
	u64 handle;
	u32 priority;
};

/* level follows nvgpu.h definitions */
struct tegra_vgpu_channel_runlist_interleave_params {
	u64 handle;
	u32 level;
};

struct tegra_vgpu_channel_timeslice_params {
	u64 handle;
	u32 timeslice_us;
};

#define TEGRA_VGPU_FECS_TRACE_FILTER_SIZE 256
struct tegra_vgpu_fecs_trace_filter {
	u64 tag_bits[(TEGRA_VGPU_FECS_TRACE_FILTER_SIZE + 63) / 64];
};

enum {
	TEGRA_VGPU_CTXSW_MODE_NO_CTXSW = 0,
	TEGRA_VGPU_CTXSW_MODE_CTXSW,
	TEGRA_VGPU_CTXSW_MODE_STREAM_OUT_CTXSW,
};

enum {
	TEGRA_VGPU_DISABLE_SAMPLING = 0,
	TEGRA_VGPU_ENABLE_SAMPLING,
};
struct tegra_vgpu_channel_set_ctxsw_mode {
	u64 handle;
	u64 gpu_va;
	u32 mode;
};

struct tegra_vgpu_channel_update_pc_sampling {
	u64 handle;
	u32 mode;
};

struct tegra_vgpu_channel_free_hwpm_ctx {
	u64 handle;
};

struct tegra_vgpu_ecc_info_params {
	u32 ecc_stats_count;
};

struct tegra_vgpu_ecc_info_entry {
	u32 ecc_id;
	char name[NVGPU_ECC_STAT_NAME_MAX_SIZE];
};

struct tegra_vgpu_ecc_counter_params {
	u32 ecc_id;
	u32 value;
};

struct tegra_vgpu_gr_ctx_params {
	u64 gr_ctx_handle;
	u64 as_handle;
	u64 gr_ctx_va;
	u32 class_num;
	u32 tsg_id;
};

struct tegra_vgpu_channel_bind_gr_ctx_params {
	u64 ch_handle;
	u64 gr_ctx_handle;
};

struct tegra_vgpu_tsg_bind_gr_ctx_params {
	u32 tsg_id;
	u64 gr_ctx_handle;
};

struct tegra_vgpu_tsg_bind_unbind_channel_params {
	u32 tsg_id;
	u64 ch_handle;
};

struct tegra_vgpu_tsg_preempt_params {
	u32 tsg_id;
};

struct tegra_vgpu_tsg_timeslice_params {
	u32 tsg_id;
	u32 timeslice_us;
};

struct tegra_vgpu_tsg_open_rel_params {
	u32 tsg_id;
};

/* level follows nvgpu.h definitions */
struct tegra_vgpu_tsg_runlist_interleave_params {
	u32 tsg_id;
	u32 level;
};

struct tegra_vgpu_read_ptimer_params {
	u64 time;
};

#define TEGRA_VGPU_GET_TIMESTAMPS_ZIPPER_MAX_COUNT      16
#define TEGRA_VGPU_GET_TIMESTAMPS_ZIPPER_SRC_ID_TSC     1
struct tegra_vgpu_get_timestamps_zipper_params {
	/* timestamp pairs */
	struct {
		/* gpu timestamp value */
		u64 cpu_timestamp;
		/* raw GPU counter (PTIMER) value */
		u64 gpu_timestamp;
	} samples[TEGRA_VGPU_GET_TIMESTAMPS_ZIPPER_MAX_COUNT];
	/* number of pairs to read */
	u32 count;
	/* cpu clock source id */
	u32 source_id;
};

#define TEGRA_VGPU_POWERGATE_MODE_ENABLE	1
#define TEGRA_VGPU_POWERGATE_MODE_DISABLE	2
struct tegra_vgpu_set_powergate_params {
	u32 mode;
};

struct tegra_vgpu_gpu_clk_rate_params {
	u32 rate; /* in kHz */
};

/* TEGRA_VGPU_MAX_ENGINES must be equal or greater than num_engines */
#define TEGRA_VGPU_MAX_ENGINES			4
struct tegra_vgpu_engines_info {
	u32 num_engines;
	struct engineinfo {
		u32 engine_id;
		u32 intr_mask;
		u32 reset_mask;
		u32 runlist_id;
		u32 pbdma_id;
		u32 inst_id;
		u32 pri_base;
		u32 engine_enum;
		u32 fault_id;
	} info[TEGRA_VGPU_MAX_ENGINES];
};

#define TEGRA_VGPU_MAX_GPC_COUNT 16
#define TEGRA_VGPU_MAX_TPC_COUNT_PER_GPC 16
#define TEGRA_VGPU_L2_EN_MASK 32

struct tegra_vgpu_constants_params {
	u32 arch;
	u32 impl;
	u32 rev;
	u32 max_freq;
	u32 num_channels;
	u32 golden_ctx_size;
	u32 zcull_ctx_size;
	u32 l2_size;
	u32 ltc_count;
	u32 cacheline_size;
	u32 slices_per_ltc;
	u32 comptags_per_cacheline;
	u32 comptag_lines;
	u32 sm_arch_sm_version;
	u32 sm_arch_spa_version;
	u32 sm_arch_warp_count;
	u32 max_gpc_count;
	u32 gpc_count;
	u32 max_tpc_per_gpc_count;
	u32 num_fbps;
	u32 fbp_en_mask;
	u32 ltc_per_fbp;
	u32 max_lts_per_ltc;
	u8 gpc_tpc_count[TEGRA_VGPU_MAX_GPC_COUNT];
	/* mask bits should be equal or larger than
	 * TEGRA_VGPU_MAX_TPC_COUNT_PER_GPC
	 */
	u16 gpc_tpc_mask[TEGRA_VGPU_MAX_GPC_COUNT];
	u32 hwpm_ctx_size;
	u8 force_preempt_mode;
	u8 can_set_clkrate;
	u32 default_timeslice_us;
	u32 preempt_ctx_size;
	u32 channel_base;
	struct tegra_vgpu_engines_info engines_info;
	u32 num_pce;
	u32 sm_per_tpc;
	u32 max_subctx_count;
	u32 l2_en_mask[TEGRA_VGPU_L2_EN_MASK];
};

enum {
	TEGRA_VGPU_CYCLE_STATS_SNAPSHOT_CMD_FLUSH = 0,
	TEGRA_VGPU_CYCLE_STATS_SNAPSHOT_CMD_ATTACH = 1,
	TEGRA_VGPU_CYCLE_STATS_SNAPSHOT_CMD_DETACH = 2,
};

struct tegra_vgpu_channel_cyclestats_snapshot_params {
	u64 handle;
	u32 perfmon_start;
	u32 perfmon_count;
	u32 buf_info; /* client->srvr: get ptr; srvr->client: num pending */
	u8 subcmd;
	u8 hw_overflow;
};

struct tegra_vgpu_gpu_load_params {
	u32 load;
};

struct tegra_vgpu_suspend_resume_contexts {
	u32 num_channels;
	u16 resident_chid;
};

struct tegra_vgpu_clear_sm_error_state {
	u64 handle;
	u32 sm_id;
};

enum {
	TEGRA_VGPU_PROF_GET_GLOBAL = 0,
	TEGRA_VGPU_PROF_GET_CONTEXT,
	TEGRA_VGPU_PROF_RELEASE
};

struct tegra_vgpu_prof_mgt_params {
	u32 mode;
};

struct tegra_vgpu_perfbuf_mgt_params {
	u64 vm_handle;
	u64 offset;
	u32 size;
};

#define TEGRA_VGPU_GPU_FREQ_TABLE_SIZE		25

struct tegra_vgpu_get_gpu_freq_table_params {
	u32 num_freqs;
};

struct tegra_vgpu_vsms_mapping_params {
	u32 num_sm;
};

struct tegra_vgpu_vsms_mapping_entry {
	u32 gpc_index;
	u32 tpc_index;
	u32 sm_index;
	u32 global_tpc_index;
};

struct tegra_vgpu_alloc_ctx_header_params {
	u64 ch_handle;
	u64 ctx_header_va;
};

struct tegra_vgpu_free_ctx_header_params {
	u64 ch_handle;
};

struct tegra_vgpu_map_syncpt_params {
	u64 as_handle;
	u64 gpu_va;
	u64 len;
	u64 offset;
	u8 prot;
};

struct tegra_vgpu_tsg_bind_channel_ex_params {
	u32 tsg_id;
	u64 ch_handle;
	u32 subctx_id;
	u32 runqueue_sel;
};

struct tegra_vgpu_fb_set_mmu_debug_mode_params {
	u8 enable;
};

struct tegra_vgpu_gr_set_mmu_debug_mode_params {
	u64 ch_handle;
	u8 enable;
};

struct tegra_vgpu_cmd_msg {
	u32 cmd;
	int ret;
	u64 handle;
	union {
		struct tegra_vgpu_connect_params connect;
		struct tegra_vgpu_channel_hwctx_params channel_hwctx;
		struct tegra_vgpu_attrib_params attrib;
		struct tegra_vgpu_as_share_params as_share;
		struct tegra_vgpu_as_bind_share_params as_bind_share;
		struct tegra_vgpu_as_map_params as_map;
		struct tegra_vgpu_as_map_ex_params as_map_ex;
		struct tegra_vgpu_channel_config_params channel_config;
		struct tegra_vgpu_ramfc_params ramfc;
		struct tegra_vgpu_ch_ctx_params ch_ctx;
		struct tegra_vgpu_zcull_bind_params zcull_bind;
		struct tegra_vgpu_cache_maint_params cache_maint;
		struct tegra_vgpu_runlist_params runlist;
		struct tegra_vgpu_golden_ctx_params golden_ctx;
		struct tegra_vgpu_zcull_info_params zcull_info;
		struct tegra_vgpu_zbc_set_table_params zbc_set_table;
		struct tegra_vgpu_zbc_query_table_params zbc_query_table;
		struct tegra_vgpu_gr_bind_ctxsw_buffers_params gr_bind_ctxsw_buffers;
		struct tegra_vgpu_mmu_debug_mode mmu_debug_mode;
		struct tegra_vgpu_sm_debug_mode sm_debug_mode;
		struct tegra_vgpu_reg_ops_params reg_ops;
		struct tegra_vgpu_channel_priority_params channel_priority;
		struct tegra_vgpu_channel_runlist_interleave_params channel_interleave;
		struct tegra_vgpu_channel_timeslice_params channel_timeslice;
		struct tegra_vgpu_fecs_trace_filter fecs_trace_filter;
		struct tegra_vgpu_channel_set_ctxsw_mode set_ctxsw_mode;
		struct tegra_vgpu_channel_free_hwpm_ctx free_hwpm_ctx;
		struct tegra_vgpu_gr_ctx_params gr_ctx;
		struct tegra_vgpu_channel_bind_gr_ctx_params ch_bind_gr_ctx;
		struct tegra_vgpu_tsg_bind_gr_ctx_params tsg_bind_gr_ctx;
		struct tegra_vgpu_tsg_bind_unbind_channel_params tsg_bind_unbind_channel;
		struct tegra_vgpu_tsg_open_rel_params tsg_open;
		struct tegra_vgpu_tsg_open_rel_params tsg_release;
		struct tegra_vgpu_tsg_preempt_params tsg_preempt;
		struct tegra_vgpu_tsg_timeslice_params tsg_timeslice;
		struct tegra_vgpu_tsg_runlist_interleave_params tsg_interleave;
		struct tegra_vgpu_read_ptimer_params read_ptimer;
		struct tegra_vgpu_set_powergate_params set_powergate;
		struct tegra_vgpu_gpu_clk_rate_params gpu_clk_rate;
		struct tegra_vgpu_constants_params constants;
		struct tegra_vgpu_channel_cyclestats_snapshot_params cyclestats_snapshot;
		struct tegra_vgpu_gpu_load_params gpu_load;
		struct tegra_vgpu_suspend_resume_contexts suspend_contexts;
		struct tegra_vgpu_suspend_resume_contexts resume_contexts;
		struct tegra_vgpu_clear_sm_error_state clear_sm_error_state;
		struct tegra_vgpu_prof_mgt_params prof_management;
		struct tegra_vgpu_perfbuf_mgt_params perfbuf_management;
		struct tegra_vgpu_get_timestamps_zipper_params get_timestamps_zipper;
		struct tegra_vgpu_get_gpu_freq_table_params get_gpu_freq_table;
		struct tegra_vgpu_vsms_mapping_params vsms_mapping;
		struct tegra_vgpu_alloc_ctx_header_params alloc_ctx_header;
		struct tegra_vgpu_free_ctx_header_params free_ctx_header;
		struct tegra_vgpu_map_syncpt_params map_syncpt;
		struct tegra_vgpu_tsg_bind_channel_ex_params tsg_bind_channel_ex;
		struct tegra_vgpu_channel_update_pc_sampling update_pc_sampling;
		struct tegra_vgpu_ecc_info_params ecc_info;
		struct tegra_vgpu_ecc_counter_params ecc_counter;
		struct tegra_vgpu_fb_set_mmu_debug_mode_params fb_set_mmu_debug_mode;
		struct tegra_vgpu_gr_set_mmu_debug_mode_params gr_set_mmu_debug_mode;
		char padding[192];
	} params;
};

enum {
	TEGRA_VGPU_GR_INTR_NOTIFY = 0,
	TEGRA_VGPU_GR_INTR_SEMAPHORE_TIMEOUT = 1,
	TEGRA_VGPU_GR_INTR_ILLEGAL_NOTIFY = 2,
	TEGRA_VGPU_GR_INTR_ILLEGAL_METHOD = 3,
	TEGRA_VGPU_GR_INTR_ILLEGAL_CLASS = 4,
	TEGRA_VGPU_GR_INTR_FECS_ERROR = 5,
	TEGRA_VGPU_GR_INTR_CLASS_ERROR = 6,
	TEGRA_VGPU_GR_INTR_FIRMWARE_METHOD = 7,
	TEGRA_VGPU_GR_INTR_EXCEPTION = 8,
	TEGRA_VGPU_GR_INTR_SEMAPHORE = 9,
	TEGRA_VGPU_FIFO_INTR_PBDMA = 10,
	TEGRA_VGPU_FIFO_INTR_CTXSW_TIMEOUT = 11,
	TEGRA_VGPU_FIFO_INTR_MMU_FAULT = 12,
	TEGRA_VGPU_GR_INTR_SM_EXCEPTION = 16,
};

struct tegra_vgpu_gr_intr_info {
	u32 type;
	u32 chid;
};

struct tegra_vgpu_gr_nonstall_intr_info {
	u32 type;
};

struct tegra_vgpu_fifo_intr_info {
	u32 type;
	u32 chid;
};

struct tegra_vgpu_fifo_nonstall_intr_info {
	u32 type;
};

struct tegra_vgpu_ce2_nonstall_intr_info {
	u32 type;
};

enum {
	TEGRA_VGPU_FECS_TRACE_DATA_UPDATE = 0
};

struct tegra_vgpu_fecs_trace_event_info {
	u32 type;
};

#define TEGRA_VGPU_CHANNEL_EVENT_ID_MAX 6
struct tegra_vgpu_channel_event_info {
	u32 event_id;
	u32 is_tsg;
	u32 id; /* channel id or tsg id */
};

struct tegra_vgpu_sm_esr_info {
	u32 tsg_id;
	u32 sm_id;
	u32 hww_global_esr;
	u32 hww_warp_esr;
	u64 hww_warp_esr_pc;
	u32 hww_global_esr_report_mask;
	u32 hww_warp_esr_report_mask;
};

struct tegra_vgpu_semaphore_wakeup {
	u32 post_events;
};

struct tegra_vgpu_channel_cleanup {
	u32 chid;
};

struct tegra_vgpu_channel_set_error_notifier {
	u32 chid;
	u32 error;
};

enum {

	TEGRA_VGPU_INTR_GR = 0,
	TEGRA_VGPU_INTR_FIFO = 1,
	TEGRA_VGPU_INTR_CE2 = 2,
};

enum {
	TEGRA_VGPU_EVENT_INTR = 0,
	TEGRA_VGPU_EVENT_ABORT = 1,
	TEGRA_VGPU_EVENT_FECS_TRACE = 2,
	TEGRA_VGPU_EVENT_CHANNEL = 3,
	TEGRA_VGPU_EVENT_SM_ESR = 4,
	TEGRA_VGPU_EVENT_SEMAPHORE_WAKEUP = 5,
	TEGRA_VGPU_EVENT_CHANNEL_CLEANUP = 6,
	TEGRA_VGPU_EVENT_SET_ERROR_NOTIFIER = 7,
};

struct tegra_vgpu_intr_msg {
	unsigned int event;
	u32 unit;
	union {
		struct tegra_vgpu_gr_intr_info gr_intr;
		struct tegra_vgpu_gr_nonstall_intr_info gr_nonstall_intr;
		struct tegra_vgpu_fifo_intr_info fifo_intr;
		struct tegra_vgpu_fifo_nonstall_intr_info fifo_nonstall_intr;
		struct tegra_vgpu_ce2_nonstall_intr_info ce2_nonstall_intr;
		struct tegra_vgpu_fecs_trace_event_info fecs_trace;
		struct tegra_vgpu_channel_event_info channel_event;
		struct tegra_vgpu_sm_esr_info sm_esr;
		struct tegra_vgpu_semaphore_wakeup sem_wakeup;
		struct tegra_vgpu_channel_cleanup ch_cleanup;
		struct tegra_vgpu_channel_set_error_notifier set_error_notifier;
		char padding[32];
	} info;
};

#define TEGRA_VGPU_QUEUE_SIZES	\
	512,			\
	sizeof(struct tegra_vgpu_intr_msg)

#endif
