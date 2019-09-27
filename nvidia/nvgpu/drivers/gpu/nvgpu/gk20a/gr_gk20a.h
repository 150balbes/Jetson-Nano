/*
 * GK20A Graphics Engine
 *
 * Copyright (c) 2011-2018, NVIDIA CORPORATION.  All rights reserved.
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
#ifndef GR_GK20A_H
#define GR_GK20A_H

#include <nvgpu/types.h>

#include "gr_ctx_gk20a.h"
#include "mm_gk20a.h"
#include <nvgpu/power_features/pg.h>

#include <nvgpu/comptags.h>
#include <nvgpu/cond.h>

#define GR_IDLE_CHECK_DEFAULT		10 /* usec */
#define GR_IDLE_CHECK_MAX		200 /* usec */
#define GR_FECS_POLL_INTERVAL		5 /* usec */

#define INVALID_SCREEN_TILE_ROW_OFFSET	0xFFFFFFFF
#define INVALID_MAX_WAYS		0xFFFFFFFF

#define GK20A_FECS_UCODE_IMAGE	"fecs.bin"
#define GK20A_GPCCS_UCODE_IMAGE	"gpccs.bin"

#define GK20A_GR_MAX_PES_PER_GPC 3

#define GK20A_TIMEOUT_FPGA		100000 /* 100 sec */

/* Flags to be passed to g->ops.gr.alloc_obj_ctx() */
#define NVGPU_OBJ_CTX_FLAGS_SUPPORT_GFXP		(1 << 1)
#define NVGPU_OBJ_CTX_FLAGS_SUPPORT_CILP		(1 << 2)

/*
 * allocate a minimum of 1 page (4KB) worth of patch space, this is 512 entries
 * of address and data pairs
 */
#define PATCH_CTX_SLOTS_REQUIRED_PER_ENTRY	2
#define PATCH_CTX_SLOTS_PER_PAGE \
	(PAGE_SIZE/(PATCH_CTX_SLOTS_REQUIRED_PER_ENTRY * sizeof(u32)))
#define PATCH_CTX_ENTRIES_FROM_SIZE(size) (size/sizeof(u32))

#define NVGPU_PREEMPTION_MODE_GRAPHICS_WFI	(1 << 0)
#define NVGPU_PREEMPTION_MODE_GRAPHICS_GFXP	(1 << 1)

#define NVGPU_PREEMPTION_MODE_COMPUTE_WFI	(1 << 0)
#define NVGPU_PREEMPTION_MODE_COMPUTE_CTA	(1 << 1)
#define NVGPU_PREEMPTION_MODE_COMPUTE_CILP	(1 << 2)

#define CTXSW_INTR0				BIT32(0)
#define CTXSW_INTR1				BIT32(1)

#define MAILBOX_VALUE_TIMESTAMP_BUFFER_FULL	0x26

struct tsg_gk20a;
struct channel_gk20a;
struct nvgpu_warpstate;

enum ctxsw_addr_type;

enum /* global_ctx_buffer */ {
	CIRCULAR		= 0,
	PAGEPOOL		= 1,
	ATTRIBUTE		= 2,
	CIRCULAR_VPR		= 3,
	PAGEPOOL_VPR		= 4,
	ATTRIBUTE_VPR		= 5,
	GOLDEN_CTX		= 6,
	PRIV_ACCESS_MAP		= 7,
	/* #8 is reserved */
	FECS_TRACE_BUFFER	= 9,
	NR_GLOBAL_CTX_BUF	= 10
};

/* either ATTRIBUTE or ATTRIBUTE_VPR maps to ATTRIBUTE_VA */
enum  /*global_ctx_buffer_va */ {
	CIRCULAR_VA		= 0,
	PAGEPOOL_VA		= 1,
	ATTRIBUTE_VA		= 2,
	GOLDEN_CTX_VA		= 3,
	PRIV_ACCESS_MAP_VA	= 4,
	/* #5 is reserved */
	FECS_TRACE_BUFFER_VA	= 6,
	NR_GLOBAL_CTX_BUF_VA	= 7
};

enum {
	WAIT_UCODE_LOOP,
	WAIT_UCODE_TIMEOUT,
	WAIT_UCODE_ERROR,
	WAIT_UCODE_OK
};

enum {
	GR_IS_UCODE_OP_EQUAL,
	GR_IS_UCODE_OP_NOT_EQUAL,
	GR_IS_UCODE_OP_AND,
	GR_IS_UCODE_OP_LESSER,
	GR_IS_UCODE_OP_LESSER_EQUAL,
	GR_IS_UCODE_OP_SKIP
};

enum {
	eUcodeHandshakeInitComplete = 1,
	eUcodeHandshakeMethodFinished
};

enum {
	ELCG_MODE = (1 << 0),
	BLCG_MODE = (1 << 1),
	INVALID_MODE = (1 << 2)
};

enum {
	NVGPU_EVENT_ID_BPT_INT = 0,
	NVGPU_EVENT_ID_BPT_PAUSE,
	NVGPU_EVENT_ID_BLOCKING_SYNC,
	NVGPU_EVENT_ID_CILP_PREEMPTION_STARTED,
	NVGPU_EVENT_ID_CILP_PREEMPTION_COMPLETE,
	NVGPU_EVENT_ID_GR_SEMAPHORE_WRITE_AWAKEN,
	NVGPU_EVENT_ID_MAX,
};

#ifndef GR_GO_IDLE_BUNDLE
#define GR_GO_IDLE_BUNDLE	0x0000e100 /* --V-B */
#endif

struct gr_channel_map_tlb_entry {
	u32 curr_ctx;
	u32 chid;
	u32 tsgid;
};

struct gr_zcull_gk20a {
	u32 aliquot_width;
	u32 aliquot_height;
	u32 aliquot_size;
	u32 total_aliquots;

	u32 width_align_pixels;
	u32 height_align_pixels;
	u32 pixel_squares_by_aliquots;
};

struct gr_zcull_info {
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

#define GK20A_ZBC_COLOR_VALUE_SIZE	4  /* RGBA */

#define GK20A_STARTOF_ZBC_TABLE		1U   /* index zero reserved to indicate "not ZBCd" */
#define GK20A_SIZEOF_ZBC_TABLE		16  /* match ltcs_ltss_dstg_zbc_index_address width (4) */
#define GK20A_ZBC_TABLE_SIZE		(16 - 1)

#define GK20A_ZBC_TYPE_INVALID		0
#define GK20A_ZBC_TYPE_COLOR		1
#define GK20A_ZBC_TYPE_DEPTH		2
#define T19X_ZBC			3

struct zbc_color_table {
	u32 color_ds[GK20A_ZBC_COLOR_VALUE_SIZE];
	u32 color_l2[GK20A_ZBC_COLOR_VALUE_SIZE];
	u32 format;
	u32 ref_cnt;
};

struct zbc_depth_table {
	u32 depth;
	u32 format;
	u32 ref_cnt;
};

struct zbc_s_table {
	u32 stencil;
	u32 format;
	u32 ref_cnt;
};

struct zbc_entry {
	u32 color_ds[GK20A_ZBC_COLOR_VALUE_SIZE];
	u32 color_l2[GK20A_ZBC_COLOR_VALUE_SIZE];
	u32 depth;
	u32 type;	/* color or depth */
	u32 format;
};

struct zbc_query_params {
	u32 color_ds[GK20A_ZBC_COLOR_VALUE_SIZE];
	u32 color_l2[GK20A_ZBC_COLOR_VALUE_SIZE];
	u32 depth;
	u32 ref_cnt;
	u32 format;
	u32 type;	/* color or depth */
	u32 index_size;	/* [out] size, [in] index */
};

struct sm_info {
	u32 gpc_index;
	u32 tpc_index;
	u32 sm_index;
	u32 global_tpc_index;
};

#if defined(CONFIG_GK20A_CYCLE_STATS)
struct gk20a_cs_snapshot_client;
struct gk20a_cs_snapshot;
#endif

struct gr_gk20a_isr_data {
	u32 addr;
	u32 data_lo;
	u32 data_hi;
	u32 curr_ctx;
	struct channel_gk20a *ch;
	u32 offset;
	u32 sub_chan;
	u32 class_num;
};

struct gr_ctx_buffer_desc {
	void (*destroy)(struct gk20a *, struct gr_ctx_buffer_desc *);
	struct nvgpu_mem mem;
	void *priv;
};

struct nvgpu_preemption_modes_rec {
	u32 graphics_preemption_mode_flags; /* supported preemption modes */
	u32 compute_preemption_mode_flags; /* supported preemption modes */

	u32 default_graphics_preempt_mode; /* default mode */
	u32 default_compute_preempt_mode; /* default mode */
};

struct gr_gk20a {
	struct gk20a *g;
	struct {
		bool dynamic;

		u32 buffer_size;
		u32 buffer_total_size;

		bool golden_image_initialized;
		u32 golden_image_size;
		u32 *local_golden_image;

		u32 hwpm_ctxsw_buffer_offset_map_count;
		struct ctxsw_buf_offset_map_entry *hwpm_ctxsw_buffer_offset_map;

		u32 zcull_ctxsw_image_size;

		u32 pm_ctxsw_image_size;

		u32 buffer_header_size;

		u32 priv_access_map_size;

		u32 fecs_trace_buffer_size;

		struct gr_ucode_gk20a ucode;

		struct av_list_gk20a  sw_bundle_init;
		struct av_list_gk20a  sw_method_init;
		struct aiv_list_gk20a sw_ctx_load;
		struct av_list_gk20a  sw_non_ctx_load;
		struct av_list_gk20a  sw_veid_bundle_init;
		struct av64_list_gk20a sw_bundle64_init;
		struct {
			struct aiv_list_gk20a sys;
			struct aiv_list_gk20a gpc;
			struct aiv_list_gk20a tpc;
			struct aiv_list_gk20a zcull_gpc;
			struct aiv_list_gk20a ppc;
			struct aiv_list_gk20a pm_sys;
			struct aiv_list_gk20a pm_gpc;
			struct aiv_list_gk20a pm_tpc;
			struct aiv_list_gk20a pm_ppc;
			struct aiv_list_gk20a perf_sys;
			struct aiv_list_gk20a perf_gpc;
			struct aiv_list_gk20a fbp;
			struct aiv_list_gk20a fbp_router;
			struct aiv_list_gk20a gpc_router;
			struct aiv_list_gk20a pm_ltc;
			struct aiv_list_gk20a pm_fbpa;
			struct aiv_list_gk20a perf_sys_router;
			struct aiv_list_gk20a perf_pma;
			struct aiv_list_gk20a pm_rop;
			struct aiv_list_gk20a pm_ucgpc;
			struct aiv_list_gk20a etpc;
			struct aiv_list_gk20a pm_cau;
		} ctxsw_regs;
		u32 regs_base_index;
		bool valid;

		u32 preempt_image_size;
		bool force_preemption_gfxp;
		bool force_preemption_cilp;
		bool dump_ctxsw_stats_on_channel_close;
	} ctx_vars;

	struct nvgpu_mutex ctx_mutex; /* protect golden ctx init */
	struct nvgpu_mutex fecs_mutex; /* protect fecs method */

#define GR_NETLIST_DYNAMIC	-1
#define GR_NETLIST_STATIC_A	'A'
	int netlist;

	struct nvgpu_cond init_wq;
	int initialized;

	u32 num_fbps;

	u32 max_comptag_lines;
	u32 compbit_backing_size;
	u32 comptags_per_cacheline;
	u32 slices_per_ltc;
	u32 cacheline_size;
	u32 gobs_per_comptagline_per_slice;

	u32 max_gpc_count;
	u32 max_fbps_count;
	u32 max_tpc_per_gpc_count;
	u32 max_zcull_per_gpc_count;
	u32 max_tpc_count;

	u32 sys_count;
	u32 gpc_count;
	u32 pe_count_per_gpc;
	u32 ppc_count;
	u32 *gpc_ppc_count;
	u32 tpc_count;
	u32 *gpc_tpc_count;
	u32 *gpc_tpc_mask;
	u32 zcb_count;
	u32 *gpc_zcb_count;
	u32 *pes_tpc_count[GK20A_GR_MAX_PES_PER_GPC];
	u32 *pes_tpc_mask[GK20A_GR_MAX_PES_PER_GPC];
	u32 *gpc_skip_mask;

	u32 bundle_cb_default_size;
	u32 min_gpm_fifo_depth;
	u32 bundle_cb_token_limit;
	u32 attrib_cb_default_size;
	u32 attrib_cb_size;
	u32 attrib_cb_gfxp_default_size;
	u32 attrib_cb_gfxp_size;
	u32 alpha_cb_default_size;
	u32 alpha_cb_size;
	u32 timeslice_mode;
	u32 czf_bypass;
	u32 pd_max_batches;
	u32 gfxp_wfi_timeout_count;
	u32 gfxp_wfi_timeout_unit;

	/*
	 * The deductible memory size for max_comptag_mem (in MBytes)
	 * Usually close to memory size that running system is taking
	 */
	u32 comptag_mem_deduct;

	struct gr_ctx_buffer_desc global_ctx_buffer[NR_GLOBAL_CTX_BUF];

	u8 *map_tiles;
	u32 map_tile_count;
	u32 map_row_offset;

	u32 max_comptag_mem; /* max memory size (MB) for comptag */
	struct compbit_store_desc compbit_store;
	struct gk20a_comptag_allocator comp_tags;

	struct gr_zcull_gk20a zcull;

	struct nvgpu_mutex zbc_lock;
	struct zbc_color_table zbc_col_tbl[GK20A_ZBC_TABLE_SIZE];
	struct zbc_depth_table zbc_dep_tbl[GK20A_ZBC_TABLE_SIZE];
	struct zbc_s_table zbc_s_tbl[GK20A_ZBC_TABLE_SIZE];
	s32 max_default_color_index;
	s32 max_default_depth_index;
	s32 max_default_s_index;

	u32 max_used_color_index;
	u32 max_used_depth_index;
	u32 max_used_s_index;

#define GR_CHANNEL_MAP_TLB_SIZE		2 /* must of power of 2 */
	struct gr_channel_map_tlb_entry chid_tlb[GR_CHANNEL_MAP_TLB_SIZE];
	u32 channel_tlb_flush_index;
	struct nvgpu_spinlock ch_tlb_lock;

	void (*remove_support)(struct gr_gk20a *gr);
	bool sw_ready;
	bool skip_ucode_init;

	struct nvgpu_preemption_modes_rec preemption_mode_rec;

	u32 fecs_feature_override_ecc_val;

	int cilp_preempt_pending_chid;

	u32 fbp_en_mask;
	u32 *fbp_rop_l2_en_mask;
	u32 no_of_sm;
	struct sm_info *sm_to_cluster;

#if defined(CONFIG_GK20A_CYCLE_STATS)
	struct nvgpu_mutex			cs_lock;
	struct gk20a_cs_snapshot	*cs_data;
#endif
	u32 max_css_buffer_size;
};

void gk20a_fecs_dump_falcon_stats(struct gk20a *g);

/* contexts associated with a TSG */
struct nvgpu_gr_ctx {
	struct nvgpu_mem mem;

	u32 graphics_preempt_mode;
	u32 compute_preempt_mode;

	struct nvgpu_mem preempt_ctxsw_buffer;
	struct nvgpu_mem spill_ctxsw_buffer;
	struct nvgpu_mem betacb_ctxsw_buffer;
	struct nvgpu_mem pagepool_ctxsw_buffer;
	u32 ctx_id;
	bool ctx_id_valid;
	bool cilp_preempt_pending;
	bool boosted_ctx;
	bool golden_img_loaded;

#ifdef CONFIG_TEGRA_GR_VIRTUALIZATION
	u64 virt_ctx;
#endif

	struct patch_desc	patch_ctx;
	struct zcull_ctx_desc	zcull_ctx;
	struct pm_ctx_desc	pm_ctx;
	u64	global_ctx_buffer_va[NR_GLOBAL_CTX_BUF_VA];
	u64	global_ctx_buffer_size[NR_GLOBAL_CTX_BUF_VA];
	int	global_ctx_buffer_index[NR_GLOBAL_CTX_BUF_VA];
	bool	global_ctx_buffer_mapped;

	u32 tsgid;
};

struct gk20a_ctxsw_ucode_segment {
	u32 offset;
	u32 size;
};

struct gk20a_ctxsw_ucode_segments {
	u32 boot_entry;
	u32 boot_imem_offset;
	u32 boot_signature;
	struct gk20a_ctxsw_ucode_segment boot;
	struct gk20a_ctxsw_ucode_segment code;
	struct gk20a_ctxsw_ucode_segment data;
};

/* sums over the ucode files as sequences of u32, computed to the
 * boot_signature field in the structure above */

/* T18X FECS remains same as T21X,
 * so FALCON_UCODE_SIG_T21X_FECS_WITH_RESERVED used
 * for T18X*/
#define FALCON_UCODE_SIG_T18X_GPCCS_WITH_RESERVED	0x68edab34
#define FALCON_UCODE_SIG_T21X_FECS_WITH_DMEM_SIZE	0x9121ab5c
#define FALCON_UCODE_SIG_T21X_FECS_WITH_RESERVED	0x9125ab5c
#define FALCON_UCODE_SIG_T12X_FECS_WITH_RESERVED	0x8a621f78
#define FALCON_UCODE_SIG_T12X_FECS_WITHOUT_RESERVED	0x67e5344b
#define FALCON_UCODE_SIG_T12X_FECS_OLDER		0x56da09f

#define FALCON_UCODE_SIG_T21X_GPCCS_WITH_RESERVED	0x3d3d65e2
#define FALCON_UCODE_SIG_T12X_GPCCS_WITH_RESERVED	0x303465d5
#define FALCON_UCODE_SIG_T12X_GPCCS_WITHOUT_RESERVED	0x3fdd33d3
#define FALCON_UCODE_SIG_T12X_GPCCS_OLDER		0x53d7877

#define FALCON_UCODE_SIG_T21X_FECS_WITHOUT_RESERVED	0x93671b7d
#define FALCON_UCODE_SIG_T21X_FECS_WITHOUT_RESERVED2	0x4d6cbc10

#define FALCON_UCODE_SIG_T21X_GPCCS_WITHOUT_RESERVED	0x393161da

struct gk20a_ctxsw_ucode_info {
	u64 *p_va;
	struct nvgpu_mem inst_blk_desc;
	struct nvgpu_mem surface_desc;
	struct gk20a_ctxsw_ucode_segments fecs;
	struct gk20a_ctxsw_ucode_segments gpccs;
};

struct gk20a_ctxsw_bootloader_desc {
	u32 start_offset;
	u32 size;
	u32 imem_offset;
	u32 entry_point;
};

struct fecs_method_op_gk20a {
	struct {
		u32 addr;
		u32 data;
	} method;

	struct {
		u32 id;
		u32 data;
		u32 clr;
		u32 *ret;
		u32 ok;
		u32 fail;
	} mailbox;

	struct {
		u32 ok;
		u32 fail;
	} cond;

};

struct nvgpu_warpstate {
	u64 valid_warps[2];
	u64 trapped_warps[2];
	u64 paused_warps[2];
};

struct gpu_ops;
int gr_gk20a_load_golden_ctx_image(struct gk20a *g,
					struct channel_gk20a *c);
void gk20a_init_gr(struct gk20a *g);
int gk20a_init_gr_support(struct gk20a *g);
int gk20a_enable_gr_hw(struct gk20a *g);
int gk20a_gr_reset(struct gk20a *g);
void gk20a_gr_wait_initialized(struct gk20a *g);

int gk20a_init_gr_channel(struct channel_gk20a *ch_gk20a);

int gk20a_alloc_obj_ctx(struct channel_gk20a  *c, u32 class_num, u32 flags);

int gk20a_gr_isr(struct gk20a *g);
u32 gk20a_gr_nonstall_isr(struct gk20a *g);

/* zcull */
u32 gr_gk20a_get_ctxsw_zcull_size(struct gk20a *g, struct gr_gk20a *gr);
int gr_gk20a_bind_ctxsw_zcull(struct gk20a *g, struct gr_gk20a *gr,
			struct channel_gk20a *c, u64 zcull_va, u32 mode);
int gr_gk20a_get_zcull_info(struct gk20a *g, struct gr_gk20a *gr,
			struct gr_zcull_info *zcull_params);
void gr_gk20a_program_zcull_mapping(struct gk20a *g, u32 zcull_num_entries,
					u32 *zcull_map_tiles);
/* zbc */
int gr_gk20a_add_zbc(struct gk20a *g, struct gr_gk20a *gr,
			struct zbc_entry *zbc_val);
int gr_gk20a_query_zbc(struct gk20a *g, struct gr_gk20a *gr,
			struct zbc_query_params *query_params);
int gk20a_gr_zbc_set_table(struct gk20a *g, struct gr_gk20a *gr,
			struct zbc_entry *zbc_val);
int gr_gk20a_load_zbc_default_table(struct gk20a *g, struct gr_gk20a *gr);

/* pmu */
int gr_gk20a_fecs_get_reglist_img_size(struct gk20a *g, u32 *size);
int gr_gk20a_fecs_set_reglist_bind_inst(struct gk20a *g,
		struct nvgpu_mem *inst_block);
int gr_gk20a_fecs_set_reglist_virtual_addr(struct gk20a *g, u64 pmu_va);

void gr_gk20a_init_cg_mode(struct gk20a *g, u32 cgmode, u32 mode_config);

/* sm */
bool gk20a_gr_sm_debugger_attached(struct gk20a *g);
u32 gk20a_gr_get_sm_no_lock_down_hww_global_esr_mask(struct gk20a *g);

#define gr_gk20a_elpg_protected_call(g, func) \
	({ \
		int err = 0; \
		if (g->support_pmu) {\
			err = nvgpu_pg_elpg_disable(g);\
			if (err != 0) {\
				err = nvgpu_pg_elpg_enable(g); \
			} \
		} \
		if (err == 0) { \
			err = func; \
			if (g->support_pmu) {\
				(void)nvgpu_pg_elpg_enable(g); \
			} \
		} \
		err; \
	})

int gk20a_gr_suspend(struct gk20a *g);

struct nvgpu_dbg_reg_op;
int gr_gk20a_exec_ctx_ops(struct channel_gk20a *ch,
			  struct nvgpu_dbg_reg_op *ctx_ops, u32 num_ops,
			  u32 num_ctx_wr_ops, u32 num_ctx_rd_ops,
			  bool *is_curr_ctx);
int __gr_gk20a_exec_ctx_ops(struct channel_gk20a *ch,
			    struct nvgpu_dbg_reg_op *ctx_ops, u32 num_ops,
			    u32 num_ctx_wr_ops, u32 num_ctx_rd_ops,
			    bool ch_is_curr_ctx);
int gr_gk20a_get_ctx_buffer_offsets(struct gk20a *g,
				    u32 addr,
				    u32 max_offsets,
				    u32 *offsets, u32 *offset_addrs,
				    u32 *num_offsets,
				    bool is_quad, u32 quad);
int gr_gk20a_get_pm_ctx_buffer_offsets(struct gk20a *g,
				       u32 addr,
				       u32 max_offsets,
				       u32 *offsets, u32 *offset_addrs,
				       u32 *num_offsets);
int gr_gk20a_update_smpc_ctxsw_mode(struct gk20a *g,
				    struct channel_gk20a *c,
				    bool enable_smpc_ctxsw);
int gr_gk20a_update_hwpm_ctxsw_mode(struct gk20a *g,
				  struct channel_gk20a *c,
				  u64 gpu_va,
				  u32 mode);

struct nvgpu_gr_ctx;
void gr_gk20a_ctx_patch_write(struct gk20a *g, struct nvgpu_gr_ctx *ch_ctx,
				    u32 addr, u32 data, bool patch);
int gr_gk20a_ctx_patch_write_begin(struct gk20a *g,
					  struct nvgpu_gr_ctx *ch_ctx,
					  bool update_patch_count);
void gr_gk20a_ctx_patch_write_end(struct gk20a *g,
					struct nvgpu_gr_ctx *ch_ctx,
					bool update_patch_count);
void gr_gk20a_commit_global_pagepool(struct gk20a *g,
				     struct nvgpu_gr_ctx *ch_ctx,
				     u64 addr, u32 size, bool patch);
void gk20a_gr_set_shader_exceptions(struct gk20a *g, u32 data);
void gr_gk20a_enable_hww_exceptions(struct gk20a *g);
int gr_gk20a_init_fs_state(struct gk20a *g);
int gr_gk20a_setup_rop_mapping(struct gk20a *g, struct gr_gk20a *gr);
int gr_gk20a_init_ctxsw_ucode(struct gk20a *g);
int gr_gk20a_load_ctxsw_ucode(struct gk20a *g);
void gr_gk20a_load_falcon_bind_instblk(struct gk20a *g);
void gr_gk20a_load_ctxsw_ucode_header(struct gk20a *g, u64 addr_base,
	struct gk20a_ctxsw_ucode_segments *segments, u32 reg_offset);
void gr_gk20a_load_ctxsw_ucode_boot(struct gk20a *g, u64 addr_base,
	struct gk20a_ctxsw_ucode_segments *segments, u32 reg_offset);


void gr_gk20a_free_tsg_gr_ctx(struct tsg_gk20a *c);
int gr_gk20a_disable_ctxsw(struct gk20a *g);
int gr_gk20a_enable_ctxsw(struct gk20a *g);
void gk20a_gr_resume_single_sm(struct gk20a *g,
		u32 gpc, u32 tpc, u32 sm);
void gk20a_gr_resume_all_sms(struct gk20a *g);
void gk20a_gr_suspend_single_sm(struct gk20a *g,
		u32 gpc, u32 tpc, u32 sm,
		u32 global_esr_mask, bool check_errors);
void gk20a_gr_suspend_all_sms(struct gk20a *g,
		u32 global_esr_mask, bool check_errors);
u32 gr_gk20a_get_tpc_count(struct gr_gk20a *gr, u32 gpc_index);
int gr_gk20a_set_sm_debug_mode(struct gk20a *g,
	struct channel_gk20a *ch, u64 sms, bool enable);
bool gk20a_is_channel_ctx_resident(struct channel_gk20a *ch);
int gr_gk20a_add_zbc_color(struct gk20a *g, struct gr_gk20a *gr,
			   struct zbc_entry *color_val, u32 index);
int gr_gk20a_add_zbc_depth(struct gk20a *g, struct gr_gk20a *gr,
			   struct zbc_entry *depth_val, u32 index);
int _gk20a_gr_zbc_set_table(struct gk20a *g, struct gr_gk20a *gr,
			struct zbc_entry *zbc_val);
void gr_gk20a_pmu_save_zbc(struct gk20a *g, u32 entries);
int gr_gk20a_wait_idle(struct gk20a *g, unsigned long duration_ms,
		       u32 expect_delay);
int gr_gk20a_handle_sm_exception(struct gk20a *g, u32 gpc, u32 tpc, u32 sm,
		bool *post_event, struct channel_gk20a *fault_ch,
		u32 *hww_global_esr);
int gr_gk20a_handle_tex_exception(struct gk20a *g, u32 gpc, u32 tpc,
					bool *post_event);
int gr_gk20a_init_ctx_state(struct gk20a *g);
int gr_gk20a_submit_fecs_method_op(struct gk20a *g,
				   struct fecs_method_op_gk20a op,
				   bool sleepduringwait);
int gr_gk20a_submit_fecs_sideband_method_op(struct gk20a *g,
		struct fecs_method_op_gk20a op);
int gr_gk20a_alloc_gr_ctx(struct gk20a *g,
			  struct nvgpu_gr_ctx *gr_ctx, struct vm_gk20a *vm,
			  u32 class, u32 padding);
void gr_gk20a_free_gr_ctx(struct gk20a *g,
		       struct vm_gk20a *vm, struct nvgpu_gr_ctx *gr_ctx);
int gr_gk20a_halt_pipe(struct gk20a *g);

#if defined(CONFIG_GK20A_CYCLE_STATS)
int gr_gk20a_css_attach(struct channel_gk20a *ch,   /* in - main hw structure */
			u32 perfmon_id_count,	    /* in - number of perfmons*/
			u32 *perfmon_id_start,	    /* out- index of first pm */
			/* in/out - pointer to client data used in later     */
			struct gk20a_cs_snapshot_client *css_client);

int gr_gk20a_css_detach(struct channel_gk20a *ch,
				struct gk20a_cs_snapshot_client *css_client);
int gr_gk20a_css_flush(struct channel_gk20a *ch,
				struct gk20a_cs_snapshot_client *css_client);

void gr_gk20a_free_cyclestats_snapshot_data(struct gk20a *g);

#else
/* fake empty cleanup function if no cyclestats snapshots enabled */
static inline void gr_gk20a_free_cyclestats_snapshot_data(struct gk20a *g)
{
	(void)g;
}
#endif

void gr_gk20a_fecs_host_int_enable(struct gk20a *g);
int gk20a_gr_handle_fecs_error(struct gk20a *g, struct channel_gk20a *ch,
		struct gr_gk20a_isr_data *isr_data);
int gk20a_gr_lock_down_sm(struct gk20a *g,
			 u32 gpc, u32 tpc, u32 sm, u32 global_esr_mask,
			 bool check_errors);
int gk20a_gr_wait_for_sm_lock_down(struct gk20a *g, u32 gpc, u32 tpc, u32 sm,
		u32 global_esr_mask, bool check_errors);
int gr_gk20a_ctx_wait_ucode(struct gk20a *g, u32 mailbox_id,
			    u32 *mailbox_ret, u32 opc_success,
			    u32 mailbox_ok, u32 opc_fail,
			    u32 mailbox_fail, bool sleepduringwait);

int gr_gk20a_get_ctx_id(struct gk20a *g,
		struct channel_gk20a *c,
		u32 *ctx_id);

u32 gk20a_gr_get_sm_hww_warp_esr(struct gk20a *g, u32 gpc, u32 tpc, u32 sm);
u32 gk20a_gr_get_sm_hww_global_esr(struct gk20a *g, u32 gpc, u32 tpc, u32 sm);

int gr_gk20a_wait_fe_idle(struct gk20a *g, unsigned long duration_ms,
			  u32 expect_delay);

struct dbg_session_gk20a;

bool gr_gk20a_suspend_context(struct channel_gk20a *ch);
bool gr_gk20a_resume_context(struct channel_gk20a *ch);
int gr_gk20a_suspend_contexts(struct gk20a *g,
			      struct dbg_session_gk20a *dbg_s,
			      int *ctx_resident_ch_fd);
int gr_gk20a_resume_contexts(struct gk20a *g,
			      struct dbg_session_gk20a *dbg_s,
			      int *ctx_resident_ch_fd);
void gk20a_gr_enable_gpc_exceptions(struct gk20a *g);
void gk20a_gr_enable_exceptions(struct gk20a *g);
int gr_gk20a_trigger_suspend(struct gk20a *g);
int gr_gk20a_wait_for_pause(struct gk20a *g, struct nvgpu_warpstate *w_state);
int gr_gk20a_resume_from_pause(struct gk20a *g);
int gr_gk20a_clear_sm_errors(struct gk20a *g);
u32 gr_gk20a_tpc_enabled_exceptions(struct gk20a *g);

int gr_gk20a_commit_global_timeslice(struct gk20a *g, struct channel_gk20a *c);

int gr_gk20a_init_sm_id_table(struct gk20a *g);

int gr_gk20a_commit_inst(struct channel_gk20a *c, u64 gpu_va);

void gr_gk20a_write_zcull_ptr(struct gk20a *g,
				struct nvgpu_mem *mem, u64 gpu_va);

void gr_gk20a_write_pm_ptr(struct gk20a *g,
				struct nvgpu_mem *mem, u64 gpu_va);

u32 gk20a_gr_gpc_offset(struct gk20a *g, u32 gpc);
u32 gk20a_gr_tpc_offset(struct gk20a *g, u32 tpc);
void gk20a_gr_get_esr_sm_sel(struct gk20a *g, u32 gpc, u32 tpc,
				u32 *esr_sm_sel);
void gk20a_gr_init_ovr_sm_dsm_perf(void);
void gk20a_gr_get_ovr_perf_regs(struct gk20a *g, u32 *num_ovr_perf_regs,
					       u32 **ovr_perf_regs);
void gk20a_gr_init_ctxsw_hdr_data(struct gk20a *g,
					struct nvgpu_mem *mem);
u32 gr_gk20a_get_patch_slots(struct gk20a *g);
int gk20a_gr_handle_notify_pending(struct gk20a *g,
				struct gr_gk20a_isr_data *isr_data);

int gr_gk20a_alloc_global_ctx_buffers(struct gk20a *g);
int gr_gk20a_map_global_ctx_buffers(struct gk20a *g,
				struct channel_gk20a *c);
int gr_gk20a_commit_global_ctx_buffers(struct gk20a *g,
			struct channel_gk20a *c, bool patch);

int gr_gk20a_fecs_ctx_bind_channel(struct gk20a *g,
					struct channel_gk20a *c);
u32 gk20a_init_sw_bundle(struct gk20a *g);
int gr_gk20a_fecs_ctx_image_save(struct channel_gk20a *c, u32 save_type);
int gk20a_gr_handle_semaphore_pending(struct gk20a *g,
				struct gr_gk20a_isr_data *isr_data);
int gr_gk20a_add_ctxsw_reg_pm_fbpa(struct gk20a *g,
				struct ctxsw_buf_offset_map_entry *map,
				struct aiv_list_gk20a *regs,
				u32 *count, u32 *offset,
				u32 max_cnt, u32 base,
				u32 num_fbpas, u32 stride, u32 mask);
int gr_gk20a_add_ctxsw_reg_perf_pma(struct ctxsw_buf_offset_map_entry *map,
	struct aiv_list_gk20a *regs,
	u32 *count, u32 *offset,
	u32 max_cnt, u32 base, u32 mask);
int gr_gk20a_decode_priv_addr(struct gk20a *g, u32 addr,
	enum ctxsw_addr_type *addr_type,
	u32 *gpc_num, u32 *tpc_num, u32 *ppc_num, u32 *be_num,
	u32 *broadcast_flags);
int gr_gk20a_split_ppc_broadcast_addr(struct gk20a *g, u32 addr,
	u32 gpc_num,
	u32 *priv_addr_table, u32 *t);
int gr_gk20a_create_priv_addr_table(struct gk20a *g,
	u32 addr,
	u32 *priv_addr_table,
	u32 *num_registers);
void gr_gk20a_split_fbpa_broadcast_addr(struct gk20a *g, u32 addr,
	u32 num_fbpas,
	u32 *priv_addr_table, u32 *t);
int gr_gk20a_get_offset_in_gpccs_segment(struct gk20a *g,
	enum ctxsw_addr_type addr_type, u32 num_tpcs, u32 num_ppcs,
	u32 reg_list_ppc_count, u32 *__offset_in_segment);

void gk20a_gr_destroy_ctx_buffer(struct gk20a *g,
	struct gr_ctx_buffer_desc *desc);
int gk20a_gr_alloc_ctx_buffer(struct gk20a *g,
	struct gr_ctx_buffer_desc *desc, size_t size);
void gk20a_gr_flush_channel_tlb(struct gr_gk20a *gr);
#endif /*__GR_GK20A_H__*/
