/*
 * include/trace/events/display.h
 *
 * Display event logging to ftrace.
 *
 * Copyright (c) 2012-2017, NVIDIA CORPORATION, All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#undef TRACE_SYSTEM
#define TRACE_SYSTEM display

#if !defined(_TRACE_DISPLAY_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_DISPLAY_H

#include "../../../drivers/video/tegra/dc/dc_priv_defs.h"
#include "../../../drivers/video/tegra/dc/dc_common.h"
#include <linux/tracepoint.h>

DECLARE_EVENT_CLASS(display_basic_template,
	TP_PROTO(struct tegra_dc *dc),
	TP_ARGS(dc),
	TP_STRUCT__entry(
		__field(	bool,		enabled)
		__field(	u8,		dev_id)
		__field(	int,		bw_rate)
		__field(	int,		new_bw_rate)
		__field(	int,		underflows_a)
		__field(	int,		underflows_b)
		__field(	int,		underflows_c)
	),
	TP_fast_assign(
		__entry->enabled = dc->enabled;
		__entry->dev_id = dc->ndev->id;
		__entry->bw_rate = dc->bw_kbps;
		__entry->new_bw_rate = dc->new_bw_kbps;
		__entry->underflows_a = dc->stats.underflows_a;
		__entry->underflows_b = dc->stats.underflows_b;
		__entry->underflows_c = dc->stats.underflows_c;
	),
	TP_printk("dc%u enabled=%d bw_rate=%d new_bw_rate=%d underflows=%d/%d/%d",
		__entry->dev_id, __entry->enabled,
		__entry->bw_rate, __entry->new_bw_rate,
		__entry->underflows_a, __entry->underflows_b,
		__entry->underflows_c)
);

DEFINE_EVENT(display_basic_template, display_enable,
	TP_PROTO(struct tegra_dc *dc),
	TP_ARGS(dc)
);

DEFINE_EVENT(display_basic_template, display_disable,
	TP_PROTO(struct tegra_dc *dc),
	TP_ARGS(dc)
);

DEFINE_EVENT(display_basic_template, display_suspend,
	TP_PROTO(struct tegra_dc *dc),
	TP_ARGS(dc)
);

DEFINE_EVENT(display_basic_template, display_resume,
	TP_PROTO(struct tegra_dc *dc),
	TP_ARGS(dc)
);

DEFINE_EVENT(display_basic_template, display_reset,
	TP_PROTO(struct tegra_dc *dc),
	TP_ARGS(dc)
);

DEFINE_EVENT(display_basic_template, update_windows,
	TP_PROTO(struct tegra_dc *dc),
	TP_ARGS(dc)
);

DEFINE_EVENT(display_basic_template, sync_windows,
	TP_PROTO(struct tegra_dc *dc),
	TP_ARGS(dc)
);

DEFINE_EVENT(display_basic_template, clear_bandwidth,
	TP_PROTO(struct tegra_dc *dc),
	TP_ARGS(dc)
);

DEFINE_EVENT(display_basic_template, program_bandwidth,
	TP_PROTO(struct tegra_dc *dc),
	TP_ARGS(dc)
);

DEFINE_EVENT(display_basic_template, set_dynamic_emc,
	TP_PROTO(struct tegra_dc *dc),
	TP_ARGS(dc)
);

DEFINE_EVENT(display_basic_template, underflow,
	TP_PROTO(struct tegra_dc *dc),
	TP_ARGS(dc)
);

TRACE_EVENT(display_syncpt_flush,
	TP_PROTO(struct tegra_dc *dc, u32 id, u32 min, u32 max),
	TP_ARGS(dc, id, min, max),
	TP_STRUCT__entry(
		__field(	bool,		enabled)
		__field(	u8,		dev_id)
		__field(	u32,		syncpt_id)
		__field(	u32,		syncpt_min)
		__field(	u32,		syncpt_max)
	),
	TP_fast_assign(
		__entry->enabled = dc->enabled;
		__entry->dev_id = dc->ndev->id;
		__entry->syncpt_id = id;
		__entry->syncpt_min = min;
		__entry->syncpt_max = max;
	),
	TP_printk("dc%u enabled=%d syncpt: id=%x min=%x max=%x",
		__entry->dev_id, __entry->enabled,
		__entry->syncpt_id, __entry->syncpt_min, __entry->syncpt_max)
);

DECLARE_EVENT_CLASS(display_io_template,
	TP_PROTO(struct tegra_dc *dc, unsigned long val, const void *reg),
	TP_ARGS(dc, val, reg),
	TP_STRUCT__entry(
		__field(	bool,		enabled)
		__field(	u8,		dev_id)
		__field(	const void *,	reg)
		__field(	u32,		val)
	),
	TP_fast_assign(
		__entry->enabled = dc->enabled;
		__entry->dev_id = dc->ndev->id;
		__entry->reg = reg;
		__entry->val = val;
	),
	TP_printk("dc%u enabled=%d reg=%p val=0x%08x",
		__entry->dev_id, __entry->enabled,
		__entry->reg, __entry->val)
);

DEFINE_EVENT(display_io_template, display_writel,
	TP_PROTO(struct tegra_dc *dc, unsigned long val, const void *reg),
	TP_ARGS(dc, val, reg)
);

DEFINE_EVENT(display_io_template, display_readl,
	TP_PROTO(struct tegra_dc *dc, unsigned long val, const void *reg),
	TP_ARGS(dc, val, reg)
);

TRACE_EVENT(display_mode,
	TP_PROTO(struct tegra_dc *dc, struct tegra_dc_mode *mode),
	TP_ARGS(dc, mode),
	TP_STRUCT__entry(
		__field(	bool,		enabled)
		__field(	u8,		dev_id)
		__field(	unsigned long,	pclk)
		__field(	unsigned short,	h_active)
		__field(	unsigned short,	v_active)
		__field(	unsigned short,	h_front_porch)
		__field(	unsigned short,	v_front_porch)
		__field(	unsigned short,	h_back_porch)
		__field(	unsigned short,	v_back_porch)
		__field(	unsigned short,	h_ref_to_sync)
		__field(	unsigned short,	v_ref_to_sync)
		__field(	unsigned short,	h_sync_width)
		__field(	unsigned short,	v_sync_width)
		__field(	bool,		stereo_mode)
	),
	TP_fast_assign(
		__entry->enabled = dc->enabled;
		__entry->dev_id = dc->ndev->id;
		__entry->pclk = mode->pclk;
		__entry->stereo_mode = mode->stereo_mode;
		__entry->h_active = mode->h_active;
		__entry->v_active = mode->v_active;
		__entry->h_front_porch = mode->h_front_porch;
		__entry->v_front_porch = mode->v_front_porch;
		__entry->h_back_porch = mode->h_back_porch;
		__entry->v_back_porch = mode->v_back_porch;
		__entry->h_sync_width = mode->h_sync_width;
		__entry->v_sync_width = mode->v_sync_width;
		__entry->h_ref_to_sync = mode->h_ref_to_sync;
		__entry->v_ref_to_sync = mode->v_ref_to_sync;
	),
	TP_printk("dc%u enabled=%d "
                "ref_to_sync: H=%d V=%d "
                "sync_width: H=%d V=%d "
                "back_porch: H=%d V=%d "
                "active: H=%d V=%d "
                "front_porch: H=%d V=%d "
                "pclk=%ld stereo mode=%d\n",
		__entry->dev_id, __entry->enabled,
                __entry->h_ref_to_sync, __entry->v_ref_to_sync,
                __entry->h_sync_width, __entry->v_sync_width,
                __entry->h_back_porch, __entry->v_back_porch,
                __entry->h_active, __entry->v_active,
                __entry->h_front_porch, __entry->v_front_porch,
                __entry->pclk, __entry->stereo_mode
	)
);

TRACE_EVENT(window_update,
	TP_PROTO(struct tegra_dc *dc, struct tegra_dc_win *win),
	TP_ARGS(dc, win),
	TP_STRUCT__entry(
		__field(	bool,		enabled)
		__field(	u8,		dev_id)
		__field(	u32,		win_fmt)
		__field(	unsigned short,	win_x)
		__field(	unsigned short,	win_y)
		__field(	unsigned short,	win_w)
		__field(	unsigned short,	win_h)
		__field(	unsigned short,	win_out_x)
		__field(	unsigned short,	win_out_y)
		__field(	unsigned short,	win_out_w)
		__field(	unsigned short,	win_out_h)
	),
	TP_fast_assign(
		__entry->enabled = dc->enabled;
		__entry->dev_id = dc->ndev->id;
		__entry->win_fmt = win->fmt;
		__entry->win_x = dfixed_trunc(win->x);
		__entry->win_y = dfixed_trunc(win->y);
		__entry->win_w = dfixed_trunc(win->w);
		__entry->win_h = dfixed_trunc(win->h);
		__entry->win_out_x = win->out_x;
		__entry->win_out_y = win->out_y;
		__entry->win_out_w = win->out_w;
		__entry->win_out_h = win->out_h;
	),
	TP_printk("dc%u enabled=%d fmt=%#x in=[x:%u y:%u w:%u h:%u] "
		"out=[x:%u y:%u w:%u h:%u] ",
		__entry->dev_id, __entry->enabled, __entry->win_fmt,
		__entry->win_x, __entry->win_y,
		__entry->win_w, __entry->win_h,
		__entry->win_out_x, __entry->win_out_y,
		__entry->win_out_w, __entry->win_out_h
	)
);

typedef void (*display_syncpt_notifier)(unsigned int, unsigned int,
				unsigned int, unsigned int, unsigned long long);

DECLARE_EVENT_CLASS(display_syncpt_notifier,
	TP_PROTO(unsigned int ctrl_num, unsigned int win_num,
		unsigned int syncpt_val, unsigned int buf_handle,
		unsigned long long timestamp),
	TP_ARGS(ctrl_num, win_num, syncpt_val, buf_handle, timestamp),
	TP_STRUCT__entry(
		__field(u32, ctrl_num)
		__field(u32, win_num)
		__field(u32, syncpt_val_value)
		__field(u32, buf_handle)
		__field(u64, timestamp)
	),
	TP_fast_assign(
		__entry->ctrl_num = ctrl_num;
		__entry->win_num = win_num;
		__entry->syncpt_val_value = syncpt_val;
		__entry->buf_handle = buf_handle;
		__entry->timestamp = timestamp;
	),
	TP_printk("Flip: ctrl_num=%u win_num=%u"
		 " syncpt=%u buf_handle=%u timestamp=%lld",
		__entry->ctrl_num, __entry->win_num,
		__entry->syncpt_val_value,
		__entry->buf_handle, __entry->timestamp)
);

DEFINE_EVENT(display_syncpt_notifier, flip_rcvd_syncpt_upd,
	TP_PROTO(unsigned int ctrl_num, unsigned int win_num,
		unsigned int syncpt_val, unsigned int buf_handle,
		unsigned long long timestamp),
	TP_ARGS(ctrl_num, win_num, syncpt_val, buf_handle, timestamp)
);

DEFINE_EVENT(display_syncpt_notifier, sync_wt_ovr_syncpt_upd,
	TP_PROTO(unsigned int ctrl_num, unsigned int win_num,
		unsigned int syncpt_val, unsigned int buf_handle,
		unsigned long long timestamp),
	TP_ARGS(ctrl_num, win_num, syncpt_val, buf_handle, timestamp)
);

DEFINE_EVENT(display_syncpt_notifier, scanout_syncpt_upd,
	TP_PROTO(unsigned int ctrl_num, unsigned int win_num,
		unsigned int syncpt_val, unsigned int buf_handle,
		unsigned long long timestamp),
	TP_ARGS(ctrl_num, win_num, syncpt_val, buf_handle, timestamp)
);

TRACE_EVENT(scanout_vrr_stats,
	TP_PROTO(unsigned int syncpt_val, int db_val),
	TP_ARGS(syncpt_val, db_val),
	TP_STRUCT__entry(
		__field(	u32,		syncpt_val_value)
		__field(	int,		db_val_value)
	),
	TP_fast_assign(
		__entry->syncpt_val_value = syncpt_val;
		__entry->db_val_value = db_val;
	),
	TP_printk("Sync Point Value for Dbalance Measurement:%u,Db_value:%d",
		__entry->syncpt_val_value,__entry->db_val_value)
);

TRACE_EVENT(dc_flip_dropped,
	TP_PROTO(bool dc_enabled, bool flip_skipped),
	TP_ARGS(dc_enabled, flip_skipped),
	TP_STRUCT__entry(
		__field(const char*, dc_disabled_value)
		__field(const char*, flip_skipped_value)
	),
	TP_fast_assign(
		__entry->dc_disabled_value = dc_enabled ? "false" : "true";
		__entry->flip_skipped_value = flip_skipped ? "true" : "false";
	),
	TP_printk("dc disabled:%s, flip skipped:%s",
		__entry->dc_disabled_value, __entry->flip_skipped_value)
);


TRACE_EVENT(hdr_data_update,
	TP_PROTO(struct tegra_dc *dc, struct tegra_dc_hdr *hdr),
	TP_ARGS(dc, hdr),
	TP_STRUCT__entry(
		__field(	bool,	hdr_enabled_dc)
		__field(	bool,	hdr_enabled_flip)
		__field(	u32,	eotf)
		__field(	u32,	static_metadata_id)
		__array(	u8,	static_metadata,	24)
	),
	TP_fast_assign(
		__entry->hdr_enabled_dc = dc->hdr.enabled;
		__entry->hdr_enabled_flip = hdr->enabled;
		__entry->eotf = hdr->eotf;
		__entry->static_metadata_id = hdr->static_metadata_id;
		memcpy(__entry->static_metadata, hdr->static_metadata, 24);
	),
	TP_printk("hdr enabled in dc=%d hdr enabled rcvd in flip=%d"
		" eotf=%d static_metadata_id=%d green_pri_x_lsb=%u"
		" green_pri_x_msb=%u green_pri_y_lsb=%u green_pri_y_msb=%u"
		" blue_pri_x_lsb=%u blue_pri_x_msb=%u blue_pri_y_lsb=%u"
		" blue_pri_y_msb=%u red_pri_x_lsb=%u red_pri_x_msb=%u"
		" red_pri_y_lsb=%u red_pri_y_msb=%u white_pri_x_lsb=%u"
		" white_pri_x_msb=%u" "white_pri_y_lsb=%u" "white_pri_y_msb=%u"
		" max_lum_lsb=%u max_lum_msb=%u min_lum_lsb=%u"
		" min_lum_msb=%umax_con_light_level_lsb=%u"
		" max_con_light_level_msb=%u max_frm_avg_light_level_lsb=%u"
		" max_frm_avg_light_level_msb=%u",
		__entry->hdr_enabled_dc, __entry->hdr_enabled_flip,
		__entry->eotf, __entry->static_metadata_id,
		__entry->static_metadata[0], __entry->static_metadata[1],
		__entry->static_metadata[2], __entry->static_metadata[3],
		__entry->static_metadata[4], __entry->static_metadata[5],
		__entry->static_metadata[6], __entry->static_metadata[7],
		__entry->static_metadata[8], __entry->static_metadata[9],
		__entry->static_metadata[10], __entry->static_metadata[11],
		__entry->static_metadata[12], __entry->static_metadata[13],
		__entry->static_metadata[14], __entry->static_metadata[15],
		__entry->static_metadata[16], __entry->static_metadata[17],
		__entry->static_metadata[18], __entry->static_metadata[19],
		__entry->static_metadata[20], __entry->static_metadata[21],
		__entry->static_metadata[22], __entry->static_metadata[23]
	)
);

/*
	Do not use this with a traceName that will be deleted!
	(only use with the macros below)
*/
TRACE_EVENT(function_frame,
	TP_PROTO(const char *trace_name, char trace_prefix),
	TP_ARGS(trace_name, trace_prefix),
	TP_STRUCT__entry(
		__field(const char*,	name)
		__field(char,			prefix)
	),
	TP_fast_assign(
		__entry->name = trace_name;
		__entry->prefix = trace_prefix;
	),
	TP_printk("%c|%s\n", __entry->prefix, __entry->name)
);

TRACE_EVENT(display_vblank,
	TP_PROTO(u32 ctrl_num, u16 vblank_count),
	TP_ARGS(ctrl_num, vblank_count),
	TP_STRUCT__entry(
		__field(u32, ctrl_num)
		__field(u16, vblank_count)
	),
	TP_fast_assign(
		__entry->ctrl_num = ctrl_num;
		__entry->vblank_count = vblank_count;
	),
	TP_printk("ctrl_num=%u vblank_count=%u",
		__entry->ctrl_num, __entry->vblank_count)
);

TRACE_EVENT(display_scanline,
	TP_PROTO(u32 ctrl_num, u16 line_num, u16 frame_num, s64 timestamp),
	TP_ARGS(ctrl_num, line_num, frame_num, timestamp),
	TP_STRUCT__entry(
		__field(u32, ctrl_num)
		__field(u16, line_num)
		__field(u16, frame_num)
		__field(u64, timestamp)
	),
	TP_fast_assign(
		__entry->ctrl_num = ctrl_num;
		__entry->line_num = line_num;
		__entry->frame_num = frame_num;
		__entry->timestamp = timestamp;
	),
	TP_printk("ctrl_num=%u line_num=%u frame_num=%u timestamp=%lld",
		__entry->ctrl_num, __entry->line_num,
		__entry->frame_num, __entry->timestamp)
);

DECLARE_EVENT_CLASS(display_dc_common_events_notifier,
	TP_PROTO(struct tegra_dc_common *dc_cmmn, struct bit_mapped_data *data,
		int head_no),
	TP_ARGS(dc_cmmn, data, head_no),
	TP_STRUCT__entry(
		__field(ulong, flags)
		__field(ulong, valid_heads)
		__field(ulong, fr_lck_req_rcvd)
		__field(ulong, fl_lck_req_rcvd)
		__field(ulong, fl_lck_req_completed)
		__field(ulong, gen_act_read_result)
		__field(bool, heads_locked)
		__field(int, head_no)
	),
	TP_fast_assign(
		__entry->flags = dc_cmmn->flags;
		__entry->valid_heads = dc_cmmn->valid_heads;
		__entry->fr_lck_req_rcvd = data->fr_lck_req_rcvd;
		__entry->fl_lck_req_rcvd = data->fl_lck_req_rcvd;
		__entry->fl_lck_req_completed = data->fl_lck_req_completed;
		__entry->gen_act_read_result = data->gen_act_read_result;
		__entry->heads_locked = dc_cmmn->heads_locked;
		__entry->head_no = head_no;
	),
	TP_printk("flags = %lu, valid_heads = %lu, fr_lck_req_rcvd = %lu, "
		"fl_lck_req_rcvd = %lu, fl_lck_req_completed = %lu, "
		"gen_act_read_result = %lu, heads_locked = %d, head = %d",
		__entry->flags, __entry->valid_heads, __entry->fr_lck_req_rcvd,
		__entry->fl_lck_req_rcvd, __entry->fl_lck_req_completed,
		__entry->gen_act_read_result, __entry->heads_locked,
		__entry->head_no)
);

DEFINE_EVENT(display_dc_common_events_notifier, received_fl_lock_request,
	TP_PROTO(struct tegra_dc_common *dc_cmmn, struct bit_mapped_data *data,
		int head_no),
	TP_ARGS(dc_cmmn, data, head_no)
);

DEFINE_EVENT(display_dc_common_events_notifier, received_fr_lock_request,
	TP_PROTO(struct tegra_dc_common *dc_cmmn, struct bit_mapped_data *data,
		int head_no),
	TP_ARGS(dc_cmmn, data, head_no)
);

DEFINE_EVENT(display_dc_common_events_notifier, received_err_check_request,
	TP_PROTO(struct tegra_dc_common *dc_cmmn, struct bit_mapped_data *data,
		int head_no),
	TP_ARGS(dc_cmmn, data, head_no)
);

DEFINE_EVENT(display_dc_common_events_notifier, completed_fr_lock_request,
	TP_PROTO(struct tegra_dc_common *dc_cmmn, struct bit_mapped_data *data,
		int head_no),
	TP_ARGS(dc_cmmn, data, head_no)
);

DEFINE_EVENT(display_dc_common_events_notifier, completed_fl_lock_request,
	TP_PROTO(struct tegra_dc_common *dc_cmmn, struct bit_mapped_data *data,
		int head_no),
	TP_ARGS(dc_cmmn, data, head_no)
);

DEFINE_EVENT(display_dc_common_events_notifier, completed_err_check_request,
	TP_PROTO(struct tegra_dc_common *dc_cmmn, struct bit_mapped_data *data,
		int head_no),
	TP_ARGS(dc_cmmn, data, head_no)
);

DEFINE_EVENT(display_dc_common_events_notifier, host1x_job_allocated,
	TP_PROTO(struct tegra_dc_common *dc_cmmn, struct bit_mapped_data *data,
		int head_no),
	TP_ARGS(dc_cmmn, data, head_no)
);

DEFINE_EVENT(display_dc_common_events_notifier, host1x_job_submitted,
	TP_PROTO(struct tegra_dc_common *dc_cmmn, struct bit_mapped_data *data,
		int head_no),
	TP_ARGS(dc_cmmn, data, head_no)
);

DEFINE_EVENT(display_dc_common_events_notifier, host1x_job_executed,
	TP_PROTO(struct tegra_dc_common *dc_cmmn, struct bit_mapped_data *data,
		int head_no),
	TP_ARGS(dc_cmmn, data, head_no)
);

DECLARE_EVENT_CLASS(display_imp_request_id,
	TP_PROTO(int ctrl_num, u64 session_id),
	TP_ARGS(ctrl_num, session_id),
	TP_STRUCT__entry(
		__field(int, ctrl_num)
		__field(u64, session_id)
	),
	TP_fast_assign(
		__entry->ctrl_num = ctrl_num;
		__entry->session_id = session_id;
	),
	TP_printk("DC ctrl_num=%d IMP session id=%llu",
		__entry->ctrl_num, __entry->session_id
	)
);

DEFINE_EVENT(display_imp_request_id, display_imp_propose_queued,
	TP_PROTO(int ctrl_num, u64 session_id),
	TP_ARGS(ctrl_num, session_id)
);

DEFINE_EVENT(display_imp_request_id, display_imp_flip_queued,
	TP_PROTO(int ctrl_num, u64 session_id),
	TP_ARGS(ctrl_num, session_id)
);

DEFINE_EVENT(display_imp_request_id, display_imp_flip_started,
	TP_PROTO(int ctrl_num, u64 session_id),
	TP_ARGS(ctrl_num, session_id)
);

DEFINE_EVENT(display_imp_request_id, display_imp_flip_completed,
	TP_PROTO(int ctrl_num, u64 session_id),
	TP_ARGS(ctrl_num, session_id)
);

DECLARE_EVENT_CLASS(display_imp_bw_settings,
	TP_PROTO(int ctrl_num, u64 total_disp_bw,
		u64 total_disp_bw_no_catchup, u64 emc_floor,
		u64 minimum_hubclk),
	TP_ARGS(ctrl_num, total_disp_bw,
		total_disp_bw_no_catchup, emc_floor,
		minimum_hubclk),
	TP_STRUCT__entry(
		__field(int, ctrl_num)
		__field(u64, total_disp_bw)
		__field(u64, total_disp_bw_no_catchup)
		__field(u64, emc_floor)
		__field(u64, minimum_hubclk)
	),
	TP_fast_assign(
		__entry->ctrl_num = ctrl_num;
		__entry->total_disp_bw = total_disp_bw;
		__entry->total_disp_bw_no_catchup = total_disp_bw_no_catchup;
		__entry->emc_floor = emc_floor;
		__entry->minimum_hubclk = minimum_hubclk
	),
	TP_printk("DC ctrl_num=%d IMP bw settings="
		"[total disp ISO bw=%llu KBps, %llu KBps (no catchup) "
		"emc floor=%llu Hz minimum hubclk=%llu Hz]",
		__entry->ctrl_num, __entry->total_disp_bw,
		__entry->total_disp_bw_no_catchup, __entry->emc_floor,
		__entry->minimum_hubclk
	)
);

DEFINE_EVENT(display_imp_bw_settings, display_imp_bw_reserved,
	TP_PROTO(int ctrl_num, u64 total_disp_bw,
		u64 total_disp_bw_no_catchup, u64 emc_floor,
		u64 minimum_hubclk),
	TP_ARGS(ctrl_num, total_disp_bw,
		total_disp_bw_no_catchup, emc_floor,
		minimum_hubclk)
);

DEFINE_EVENT(display_imp_bw_settings, display_imp_bw_programmed,
	TP_PROTO(int ctrl_num, u64 total_disp_bw,
		u64 total_disp_bw_no_catchup, u64 emc_floor,
		u64 minimum_hubclk),
	TP_ARGS(ctrl_num, total_disp_bw,
		total_disp_bw_no_catchup, emc_floor,
		minimum_hubclk)
);

DECLARE_EVENT_CLASS(display_imp_global_settings,
	TP_PROTO(int ctrl_num, u32 total_cursor_fetch_meter,
		u32 total_win_fetch_meter),
	TP_ARGS(ctrl_num, total_cursor_fetch_meter,
		total_win_fetch_meter),
	TP_STRUCT__entry(
		__field(int, ctrl_num)
		__field(u32, total_cursor_fetch_meter)
		__field(u32, total_win_fetch_meter)
	),
	TP_fast_assign(
		__entry->ctrl_num = ctrl_num;
		__entry->total_cursor_fetch_meter = total_cursor_fetch_meter;
		__entry->total_win_fetch_meter = total_win_fetch_meter;
	),
	TP_printk("DC ctrl_num=%d Total cursor fetch meter=0x%x "
		"Total win fetch meter=0x%x",
		__entry->ctrl_num, __entry->total_cursor_fetch_meter,
		__entry->total_win_fetch_meter
	)
);

DEFINE_EVENT(display_imp_global_settings, display_imp_program_global,
	TP_PROTO(int ctrl_num, u32 total_cursor_fetch_meter,
		u32 total_win_fetch_meter),
	TP_ARGS(ctrl_num, total_cursor_fetch_meter,
		total_win_fetch_meter)
);

DECLARE_EVENT_CLASS(display_imp_cursor_settings,
	TP_PROTO(int ctrl_num, u32 fetch_meter, u32 pipe_meter,
		u32 mempool_entries, u32 dvfs_watermark),
	TP_ARGS(ctrl_num, fetch_meter, pipe_meter,
		mempool_entries, dvfs_watermark),
	TP_STRUCT__entry(
		__field(int, ctrl_num)
		__field(u32, fetch_meter)
		__field(u32, pipe_meter)
		__field(u32, mempool_entries)
		__field(u32, dvfs_watermark)
	),
	TP_fast_assign(
		__entry->ctrl_num = ctrl_num;
		__entry->fetch_meter = fetch_meter;
		__entry->pipe_meter = pipe_meter;
		__entry->mempool_entries = mempool_entries;
		__entry->dvfs_watermark = dvfs_watermark;
	),
	TP_printk("DC ctrl_num=%d cursor IMP settings="
		"[fetch meter=0x%x pipe_meter=0x%x mempool_entries=0x%x "
		"dvfs_watermark=0x%x]",
		__entry->ctrl_num, __entry->fetch_meter, __entry->pipe_meter,
		__entry->mempool_entries, __entry->dvfs_watermark
	)
);

DEFINE_EVENT(display_imp_cursor_settings, display_imp_program_cursor,
	TP_PROTO(int ctrl_num, u32 fetch_meter, u32 pipe_meter,
		u32 mempool_entries, u32 dvfs_watermark),
	TP_ARGS(ctrl_num, fetch_meter, pipe_meter,
		mempool_entries, dvfs_watermark)
);

DECLARE_EVENT_CLASS(display_imp_win_settings,
	TP_PROTO(int ctrl_num, int win_num, u32 fetch_meter, u32 pipe_meter,
		u32 mempool_entries, u32 dvfs_watermark, int thread_group),
	TP_ARGS(ctrl_num, win_num, fetch_meter, pipe_meter,
		mempool_entries, dvfs_watermark, thread_group),
	TP_STRUCT__entry(
		__field(int, ctrl_num)
		__field(int, win_num)
		__field(u32, fetch_meter)
		__field(u32, pipe_meter)
		__field(u32, mempool_entries)
		__field(u32, dvfs_watermark)
		__field(int, thread_group)
	),
	TP_fast_assign(
		__entry->ctrl_num = ctrl_num;
		__entry->win_num = win_num;
		__entry->fetch_meter = fetch_meter;
		__entry->pipe_meter = pipe_meter;
		__entry->mempool_entries = mempool_entries;
		__entry->dvfs_watermark = dvfs_watermark;
		__entry->thread_group = thread_group
	),
	TP_printk("DC ctrl_num=%d win_num=%d win IMP settings="
		"[fetch meter=0x%x pipe_meter=0x%x mempool_entries=0x%x "
		"dvfs_watermark=0x%x thread_group=%d]",
		__entry->ctrl_num, __entry->win_num, __entry->fetch_meter,
		__entry->pipe_meter, __entry->mempool_entries,
		__entry->dvfs_watermark, __entry->thread_group
	)
);

DEFINE_EVENT(display_imp_win_settings, display_imp_program_win,
	TP_PROTO(int ctrl_num, int win_num, u32 fetch_meter, u32 pipe_meter,
		u32 mempool_entries, u32 dvfs_watermark, int thread_group),
	TP_ARGS(ctrl_num, win_num, fetch_meter, pipe_meter,
		mempool_entries, dvfs_watermark, thread_group)
);

DECLARE_EVENT_CLASS(display_imp_cursor_mempool,
	TP_PROTO(int ctrl_num, u32 mempool_entries),
	TP_ARGS(ctrl_num, mempool_entries),
	TP_STRUCT__entry(
		__field(int, ctrl_num)
		__field(u32, mempool_entries)
	),
	TP_fast_assign(
		__entry->ctrl_num = ctrl_num;
		__entry->mempool_entries = mempool_entries;
	),
	TP_printk("DC ctrl_num=%d Cursor mempool=0x%x",
		__entry->ctrl_num, __entry->mempool_entries
	)
);

DEFINE_EVENT(display_imp_cursor_mempool, display_imp_cursor_mempool_programmed,
	TP_PROTO(int ctrl_num, u32 mempool_entries),
	TP_ARGS(ctrl_num, mempool_entries)
);

DECLARE_EVENT_CLASS(display_imp_win_mempool,
	TP_PROTO(int ctrl_num, int win_num, u32 mempool_entries),
	TP_ARGS(ctrl_num, win_num, mempool_entries),
	TP_STRUCT__entry(
		__field(int, ctrl_num)
		__field(int, win_num)
		__field(u32, mempool_entries)
	),
	TP_fast_assign(
		__entry->ctrl_num = ctrl_num;
		__entry->win_num = win_num;
		__entry->mempool_entries = mempool_entries
	),
	TP_printk("DC ctrl_num=%d win_num=%d mempool=0x%x",
		__entry->ctrl_num, __entry->win_num, __entry->mempool_entries
	)
);

DEFINE_EVENT(display_imp_win_mempool, display_imp_win_mempool_programmed,
	TP_PROTO(int ctrl_num, int win_num, u32 mempool_entries),
	TP_ARGS(ctrl_num, win_num, mempool_entries)
);

TRACE_EVENT(display_embedded_latency,
	TP_PROTO(u32 ctrl_num, u16 line_num, u64 timestamp),
	TP_ARGS(ctrl_num, line_num, timestamp),
	TP_STRUCT__entry(
		__field(u32, ctrl_num)
		__field(u16, line_num)
		__field(u64, timestamp)
	),
	TP_fast_assign(
		__entry->ctrl_num = ctrl_num;
		__entry->line_num = line_num;
		__entry->timestamp = timestamp;
	),
	TP_printk("ctrl_num=%u line_num=%u timestamp=%lld",
		__entry->ctrl_num, __entry->line_num, __entry->timestamp)
);

DECLARE_EVENT_CLASS(display_crc_regs,
	TP_PROTO(int ctrl_num, u32 frame_cnt, char *type, u32 crca, u32 crcb),
	TP_ARGS(ctrl_num, frame_cnt, type, crca, crcb),
	TP_STRUCT__entry(
		__field(int, ctrl_num)
		__field(u32, frame_cnt)
		__field(char *, type)
		__field(u32, crca)
		__field(u32, crcb)
	),
	TP_fast_assign(
		__entry->ctrl_num = ctrl_num;
		__entry->frame_cnt = frame_cnt;
		__entry->type = type;
		__entry->crca = crca;
		__entry->crcb = crcb;
	),
	TP_printk("DC%d frame_cnt=%u read %s status=%#x CRCB=%u",
		__entry->ctrl_num,
		__entry->frame_cnt,
		__entry->type,
		__entry->crca,
		__entry->crcb
	)
);

DEFINE_EVENT(display_crc_regs, display_crc_read,
	TP_PROTO(int ctrl_num, u32 frame_cnt, char *type, u32 crca, u32 crcb),
	TP_ARGS(ctrl_num, frame_cnt, type, crca, crcb)
);

DECLARE_EVENT_CLASS(display_crc_rgnl,
	TP_PROTO(int ctrl_num, u32 frame_cnt, char *type, u32 id, u32 status, u32 crc),
	TP_ARGS(ctrl_num, frame_cnt, type, id, status, crc),
	TP_STRUCT__entry(
		__field(int, ctrl_num)
		__field(u32, frame_cnt)
		__field(char *, type)
		__field(u32, id)
		__field(u32, status)
		__field(u32, crc)
	),
	TP_fast_assign(
		__entry->ctrl_num = ctrl_num;
		__entry->frame_cnt = frame_cnt;
		__entry->type   = type;
		__entry->id     = id;
		__entry->status = status;
		__entry->crc    = crc;
	),
	TP_printk("DC%d frame_cnt=%u read %s%u status=%#x CRC=%u",
		__entry->ctrl_num,
		__entry->frame_cnt,
		__entry->type,
		__entry->id,
		__entry->status,
		__entry->crc
	)
);

DEFINE_EVENT(display_crc_rgnl, display_crc_rgnl,
	TP_PROTO(int ctrl_num, u32 frame_cnt, char *type, u32 id, u32 status, u32 crcb),
	TP_ARGS(ctrl_num, frame_cnt, type, id, status, crcb)
);

DECLARE_EVENT_CLASS(display_crc_late,
	TP_PROTO(int ctrl_num, u32 frame_cnt, char *type, bool valid, u32 crc),
	TP_ARGS(ctrl_num, frame_cnt, type, valid, crc),
	TP_STRUCT__entry(
		__field(int, ctrl_num)
		__field(u32, frame_cnt)
		__field(char *, type)
		__field(bool, valid)
		__field(u32, crc)
	),
	TP_fast_assign(
		__entry->ctrl_num = ctrl_num;
		__entry->frame_cnt = frame_cnt;
		__entry->type = type;
		__entry->valid = valid;
		__entry->crc = crc;
	),
	TP_printk("DC%d frame_cnt=%u %s CRC   valid=%s %u",
		__entry->ctrl_num,
		__entry->frame_cnt,
		__entry->type,
		__entry->valid ? "1:keep":"0:clr",
		__entry->crc
	)
);

DEFINE_EVENT(display_crc_late, display_crc_late,
	TP_PROTO(int ctrl_num, u32 frame_cnt, char *type, bool valid, u32 crc),
	TP_ARGS(ctrl_num, frame_cnt, type, valid, crc)
);

DECLARE_EVENT_CLASS(display_crc_rgnl_stat,
	TP_PROTO(int ctrl_num, u32 frame_cnt, char *type, u32 id, u32 status),
	TP_ARGS(ctrl_num, frame_cnt, type, id, status),
	TP_STRUCT__entry(
		__field(int, ctrl_num)
		__field(u32, frame_cnt)
		__field(char *, type)
		__field(u32, id)
		__field(u32, status)
	),
	TP_fast_assign(
		__entry->ctrl_num = ctrl_num;
		__entry->frame_cnt = frame_cnt;
		__entry->type   = type;
		__entry->id     = id;
		__entry->status = status;
	),
	TP_printk("DC%d frame_cnt=%u %s%u status=%#x",
		__entry->ctrl_num,
		__entry->frame_cnt,
		__entry->type,
		__entry->id,
		__entry->status
	)
);

DEFINE_EVENT(display_crc_rgnl_stat, display_crc_rgnl_stat,
	TP_PROTO(int ctrl_num, u32 frame_cnt, char *type, u32 id, u32 status),
	TP_ARGS(ctrl_num, frame_cnt, type, id, status)
);

TRACE_EVENT(display_crc_boundary_crossed,
	TP_PROTO(int ctrl_num, u32 start_frame_cnt, u32 end_frame_cnt, char *desc),

	TP_ARGS(ctrl_num, start_frame_cnt, end_frame_cnt, desc),
	TP_STRUCT__entry(
		__field(u32, ctrl_num)
		__field(u32, start_frame_cnt)
		__field(u32, end_frame_cnt)
		__field(char *, desc)
	),
	TP_fast_assign(
		__entry->ctrl_num = ctrl_num;
		__entry->start_frame_cnt = start_frame_cnt;
		__entry->end_frame_cnt = end_frame_cnt;
		__entry->desc = desc;
	),
	TP_printk("DC%d start_frame=%u end_frame=%u %s",
		__entry->ctrl_num,
		__entry->start_frame_cnt,
		__entry->end_frame_cnt,
		__entry->desc
	)
);

#endif /* _TRACE_DISPLAY_H */

/* This part must be outside protection */
#include <trace/define_trace.h>
