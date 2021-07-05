/*
 * dc_priv.h: dc private interface.
 *
 * Copyright (C) 2010 Google, Inc.
 * Author: Erik Gilling <konkers@android.com>
 *
 * Copyright (c) 2010-2020, NVIDIA CORPORATION, All rights reserved.
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

#ifndef __DRIVERS_VIDEO_TEGRA_DC_DC_PRIV_H
#define __DRIVERS_VIDEO_TEGRA_DC_DC_PRIV_H

#include <uapi/video/tegra_dc_ext.h>
#include "dc_priv_defs.h"
#ifndef CREATE_TRACE_POINTS
#include <trace/events/display.h>
#define WIN_IS_BLOCKLINEAR(win)	((win)->flags & TEGRA_WIN_FLAG_BLOCKLINEAR)
#endif
#include <soc/tegra/tegra_powergate.h>
#include <uapi/video/tegra_dc_ext.h>
#include <video/tegra_dc_ext_kernel.h>
#include <soc/tegra/tegra_bpmp.h>

#include <linux/clk-provider.h>

#define CHAR_BUF_SIZE_MAX	50

#define WIN_IS_TILED(win)	((win)->flags & TEGRA_WIN_FLAG_TILED)
#define WIN_IS_ENABLED(win)	((win)->flags & TEGRA_WIN_FLAG_ENABLED)
#define WIN_IS_FB(win)		((win)->flags & TEGRA_WIN_FLAG_FB)

#define WIN_IS_INTERLACE(win) ((win)->flags & TEGRA_WIN_FLAG_INTERLACE)

#define WIN_ALL_ACT_REQ (WIN_A_ACT_REQ | WIN_B_ACT_REQ | WIN_C_ACT_REQ)

/* Note: the hw reset value for background color on T210 is 0x00000000.
 * We are using a sw reset value here, which is 0xff000000.
 * Bits 31:24 correspond to background alpha and 0xff value represents opaque.
 */

#define DISP_BLEND_BACKGROUND_COLOR_DEFAULT 0xff000000
#define RGB_TO_YUV420_8BPC_BLACK_PIX 0x00801010
#define RGB_TO_YUV420_10BPC_BLACK_PIX 0x00000000
#define RGB_TO_YUV422_10BPC_BLACK_PIX 0x00001080
#define RGB_TO_YUV444_8BPC_BLACK_PIX 0x00801080

#define YUV_MASK (FB_VMODE_Y420 | FB_VMODE_Y420_ONLY | \
				FB_VMODE_Y422 | FB_VMODE_Y444)
#define IS_RGB(yuv_flag) (!(yuv_flag & YUV_MASK))

#define is_hotplug_supported(x) \
({ \
	tegra_dc_is_ext_panel(x->dc); \
})

#define TEGRA_DC_POLL_TIMEOUT_MS       50

extern struct tegra_dc_out_ops tegra_dc_rgb_ops;
extern struct tegra_dc_out_ops tegra_dc_dsi_ops;

#if defined(CONFIG_TEGRA_HDMI2_0)
extern struct tegra_dc_out_ops tegra_dc_hdmi2_0_ops;
#endif

#ifdef CONFIG_TEGRA_DP
extern struct tegra_dc_out_ops tegra_dc_dp_ops;
#endif
#ifdef CONFIG_TEGRA_NVSR
extern struct tegra_dc_out_ops tegra_dc_nvsr_ops;
#endif

extern struct tegra_dc_out_ops tegra_dc_null_ops;

/* defined in dc.c */
bool tegra_dc_in_cmode(struct tegra_dc *dc);

/* defined in dc_sysfs.c, used by dc.c */
void tegra_dc_remove_sysfs(struct device *dev);
void tegra_dc_create_sysfs(struct device *dev);

/* defined in dc.c, used by dc_sysfs.c */
void tegra_dc_stats_enable(struct tegra_dc *dc, bool enable);
bool tegra_dc_stats_get(struct tegra_dc *dc);

/* defined in dc.c, used by dc_sysfs.c */
u32 tegra_dc_sysfs_read_checksum_latched(struct tegra_dc *dc);
void tegra_dc_sysfs_enable_crc(struct tegra_dc *dc);
void tegra_dc_sysfs_disable_crc(struct tegra_dc *dc);

void tegra_dc_set_out_pin_polars(struct tegra_dc *dc,
				const struct tegra_dc_out_pin *pins,
				const unsigned int n_pins);
/* defined in dc.c, used in bandwidth.c and ext/dev.c */
unsigned int tegra_dc_has_multiple_dc(void);

/* defined in dc.c, used in hdmi2.0.c and hpd.c */
void tegra_dc_extcon_hpd_notify(struct tegra_dc *dc);

/* defined in dc.c, used in hdmihdcp.c */
int tegra_dc_ddc_enable(struct tegra_dc *dc, bool enabled);

/* defined in dc.c, used in dsi.c */
void tegra_dc_clk_enable(struct tegra_dc *dc);
void tegra_dc_clk_disable(struct tegra_dc *dc);

/* defined in dc.c, used in dsi.c */
void tegra_dc_get(struct tegra_dc *dc);
void tegra_dc_put(struct tegra_dc *dc);

/* defined in dc.c, used in tegra_adf.c */
void tegra_dc_hold_dc_out(struct tegra_dc *dc);
void tegra_dc_release_dc_out(struct tegra_dc *dc);

bool tegra_dc_hotplug_supported(struct tegra_dc *dc);

/* defined in dc.c, used in ext/dev.c */
void tegra_dc_call_flip_callback(void);

/* defined in dc.c, used in dsi.c, nvdisp.c, nvdisp_win.c */
unsigned long tegra_dc_poll_register(struct tegra_dc *dc,
u32 reg, u32 mask, u32 exp_val, u32 poll_interval_us,
u32 timeout_ms);

/* defined in dc.c, used in sor.c, nvdisp.c, nvdisp_lut.c */
void tegra_dc_enable_general_act(struct tegra_dc *dc);

/* defined in dc.c, used by hdmi2.0.c */
void tegra_dc_enable_disp_ctrl_mode(struct tegra_dc *dc);
void tegra_dc_disable_disp_ctrl_mode(struct tegra_dc *dc);

/* defined in dc.c, used in nvdisp.c */
int tegra_dc_enable_update_and_act(struct tegra_dc *dc, u32 update_mask,
							u32 act_req_mask);

/* defined in dc.c, used in dsi.c */
void tegra_dc_dsc_init(struct tegra_dc *dc);
void tegra_dc_en_dis_dsc(struct tegra_dc *dc, bool enable);

/* defined in dc.c, used by ext/dev.c */
extern int no_vsync;

/* defined in dc.c, used in ext/dev.c */
int tegra_dc_config_frame_end_intr(struct tegra_dc *dc, bool enable);

/* defined in dc.c, used in dsi.c */
int _tegra_dc_wait_for_frame_end(struct tegra_dc *dc,
	u32 timeout_ms);

/* defined in bandwidth.c, used in dc.c */
void tegra_dc_clear_bandwidth(struct tegra_dc *dc);
void tegra_dc_program_bandwidth(struct tegra_dc *dc, bool use_new);
int tegra_dc_set_dynamic_emc(struct tegra_dc *dc);
#ifdef CONFIG_TEGRA_ISOMGR
void tegra_dc_bandwidth_renegotiate(void *p, u32 avail_bw);
#endif
unsigned long tegra_dc_get_bandwidth(struct tegra_dc_win *windows[], int n);
long tegra_calc_min_bandwidth(struct tegra_dc *dc);
long tegra_nvdisp_calc_min_bandwidth(struct tegra_dc *dc);

/* defined in mode.c, used in dc.c, window.c and hdmi2.0.c */
int tegra_dc_program_mode(struct tegra_dc *dc, struct tegra_dc_mode *mode);
int tegra_dc_calc_refresh(const struct tegra_dc_mode *m);
int tegra_dc_calc_fb_refresh(const struct fb_videomode *fbmode);
int tegra_dc_update_mode(struct tegra_dc *dc);
u32 tegra_dc_get_aspect_ratio(struct tegra_dc *dc);

/* defined in mode.c, used in hdmi.c and hdmi2.0.c */
bool check_fb_videomode_timings(const struct tegra_dc *dc,
				const struct fb_videomode *fbmode);

/* defined in mode.c, used in nvsr.c */
int _tegra_dc_set_mode(struct tegra_dc *dc, const struct tegra_dc_mode *mode);

/* defined in clock.c, used in dc.c, rgb.c, dsi.c and hdmi.c */
void tegra_dc_setup_clk(struct tegra_dc *dc, struct clk *clk);
unsigned long tegra_dc_pclk_round_rate(struct tegra_dc *dc, int pclk);

/* defined in lut.c, used in dc.c */
void tegra_dc_init_lut_defaults(struct tegra_dc_lut *lut);
void tegra_dc_set_lut(struct tegra_dc *dc, struct tegra_dc_win *win);

/* defined in csc.c, used in dc.c */
void tegra_dc_init_win_csc_defaults(struct tegra_dc_win_csc *win_csc);
void tegra_dc_set_win_csc(struct tegra_dc *dc,
			struct tegra_dc_win_csc *win_csc);

/* defined in window.c, used in dc.c and nvdisp_win.c */
void tegra_dc_trigger_windows(struct tegra_dc *dc);
bool update_is_hsync_safe(struct tegra_dc_win *cur_win,
	struct tegra_dc_win *new_win);

void tegra_dc_set_color_control(struct tegra_dc *dc);
void tegra_dc_cmu_enable(struct tegra_dc *dc, bool cmu_enable);
void _tegra_dc_cmu_enable(struct tegra_dc *dc, bool cmu_enable);

int tegra_dc_update_cmu(struct tegra_dc *dc, struct tegra_dc_cmu *cmu);
int tegra_dc_update_cmu_aligned(struct tegra_dc *dc, struct tegra_dc_cmu *cmu);

int tegra_dc_set_hdr(struct tegra_dc *dc, struct tegra_dc_hdr *hdr,
					bool cache_dirty);

struct tegra_dsi_cmd *dsi_parse_cmd_dt(struct device *dev,
		const struct device_node *node,
		struct property *prop,
		u32 n_cmd);

struct tegra_dc_platform_data
	*of_dc_parse_platform_data(struct platform_device *ndev,
	struct tegra_dc_platform_data *boot_pdata);

struct tegra_dc *find_dc_by_ctrl_num(u32 ctrl_num);
int of_tegra_get_fb_resource(struct device_node *np,
		struct resource *res,
		const char *reg_name);

struct tegra_panel_ops *tegra_dc_get_panel_ops(struct device_node *panel_np);
void tegra_panel_register_ops(struct tegra_dc_out *dc_out,
				struct tegra_panel_ops *p_ops);
void tegra_panel_unregister_ops(struct tegra_dc_out *dc_out);
void tegra_dc_out_destroy(struct tegra_dc *dc);
unsigned int tegra_dc_get_numof_reg_disps(void);
int tegra_panel_get_panel_id(const char *comp_str, struct device_node *dnode,
				int *panel_id);
int tegra_panel_regulator_get_dt(struct device *dev,
				struct tegra_panel_reg *panel_reg);

/* defined in dc.c, used in dc.c and dev.c */
void tegra_dc_set_act_vfp(struct tegra_dc *dc, int vfp);

/* defined in dc.c, used in dc.c and window.c */
bool tegra_dc_windows_are_dirty(struct tegra_dc *dc, u32 win_act_req_mask);
int tegra_dc_get_v_count(struct tegra_dc *dc);

/* defined in dc.c, used in vrr.c */
s32 tegra_dc_calc_v_front_porch(struct tegra_dc_mode *mode,
				int desired_fps);

/* defined in cursor.c, used in dc.c and ext/cursor.c */
int tegra_dc_cursor_image(struct tegra_dc *dc,
	enum tegra_dc_cursor_blend_format blendfmt,
	enum tegra_dc_cursor_size size,
	u32 fg, u32 bg, dma_addr_t phys_addr,
	enum tegra_dc_cursor_color_format colorfmt, u32 alpha, u32 flags,
	bool wait_for_activation);
int tegra_dc_cursor_set(struct tegra_dc *dc, bool enable, int x, int y);
int tegra_dc_cursor_clip(struct tegra_dc *dc, unsigned clip);
int tegra_dc_cursor_suspend(struct tegra_dc *dc);
int tegra_dc_cursor_resume(struct tegra_dc *dc);
void tegra_dc_win_partial_update(struct tegra_dc *dc, struct tegra_dc_win *win,
	unsigned int xoff, unsigned int yoff, unsigned int width,
	unsigned int height);
void tegra_dc_set_background_color(struct tegra_dc *dc, u32 background_color);
int tegra_dc_slgc_disp0(struct notifier_block *nb, unsigned long unused0,
	void *unused1);

/* defined in dc.c, used in dc_sysfs.c and ext/dev.c */
int tegra_dc_update_winmask(struct tegra_dc *dc, unsigned long winmask);

/* defined in dc.c, used in sor.c */
struct tegra_dc_sor_info *tegra_dc_get_sor_cap(void);

/* common display clock calls */
struct clk *tegra_disp_clk_get(struct device *dev, const char *id);
void tegra_disp_clk_put(struct device *dev, struct clk *clk);
struct clk *tegra_disp_of_clk_get_by_name(struct device_node *np,
						const char *name);
/* defined in fb.c */
bool tegra_fb_is_console_enabled(struct tegra_dc_platform_data *pdata);

/* core IMP calls */
int tegra_dc_reserve_common_channel(struct tegra_dc *dc);
void tegra_dc_release_common_channel(struct tegra_dc *dc);
int tegra_dc_validate_imp_queue(struct tegra_dc *dc, u64 session_id);
void tegra_dc_adjust_imp(struct tegra_dc *dc, bool before_win_update);
bool tegra_dc_handle_common_channel_promotion(struct tegra_dc *dc);
int tegra_dc_queue_imp_propose(struct tegra_dc *dc,
			struct tegra_dc_ext_flip_user_data *flip_user_data);
void tegra_dc_reset_imp_state(void);
struct dentry *tegra_nvdisp_create_imp_lock_debugfs(struct tegra_dc *dc);

/** Frame-Flip Lock API
 * Defined in dc.c. Used in dc_common.c
 */
void tegra_dc_enable_disable_frame_lock(struct tegra_dc *dc, bool enable);
void tegra_dc_upd_frame_flip_lock_job_stauts(struct tegra_dc *dc, bool status);
void tegra_dc_request_trigger_wins(struct tegra_dc *dc);

#ifdef CONFIG_TEGRA_ISOMGR
void tegra_nvdisp_bandwidth_attach(struct tegra_dc *dc);
int tegra_nvdisp_bandwidth_register(enum tegra_iso_client iso_client,
				enum tegra_bwmgr_client_id bwmgr_client);
void tegra_nvdisp_bandwidth_unregister(void);
#endif

/* Nvdisplay specific */
int tegra_nvdisp_init(struct tegra_dc *dc);
int tegra_nvdisp_update_windows(struct tegra_dc *dc,
	struct tegra_dc_win *windows[], int n,
	u16 *dirty_rect, bool wait_for_vblank, bool lock_flip);
int tegra_nvdisp_assign_win(struct tegra_dc *dc, unsigned idx);
int tegra_nvdisp_detach_win(struct tegra_dc *dc, unsigned idx);
int tegra_nvdisp_disable_wins(struct tegra_dc *dc,
			struct tegra_dc_win_detach_state *win_state_arr);
int tegra_nvdisp_restore_wins(struct tegra_dc *dc,
			struct tegra_dc_win_detach_state *win_state_arr);
int tegra_nvdisp_head_enable(struct tegra_dc *dc);
int tegra_nvdisp_head_disable(struct tegra_dc *dc);
int tegra_nvdisp_get_linestride(struct tegra_dc *dc, int win);
void tegra_nvdisp_sysfs_enable_crc(struct tegra_dc *dc);
void tegra_nvdisp_sysfs_disable_crc(struct tegra_dc *dc);
u32 tegra_nvdisp_sysfs_read_rg_crc(struct tegra_dc *dc);
void tegra_nvdisp_underflow_handler(struct tegra_dc *dc);
int tegra_nvdisp_set_compclk(struct tegra_dc *dc);
void tegra_dc_reg_dump(struct tegra_dc *dc, void *data,
	void (*print)(void *data, const char *str));
void tegra_nvdisp_reg_dump(struct tegra_dc *dc, void *data,
	void (*print)(void *data, const char *str));

int tegra_nvdisp_get_imp_user_info(struct tegra_dc_ext_imp_user_info *info);
int nvdisp_register_backlight_notifier(struct tegra_dc *dc);
void tegra_nvdisp_stop_display(struct tegra_dc *dc);
void tegra_nvdisp_vrr_work(struct work_struct *work);

int tegra_dc_hw_init(void);
bool tegra_dc_is_t21x(void);
bool tegra_dc_is_t18x(void);
bool tegra_dc_is_t19x(void);
bool tegra_dc_is_nvdisplay(void);
void __attribute__((weak)) tegra_nvdisp_init_win_csc_defaults(
			struct tegra_dc_nvdisp_win_csc *nvdisp_win_csc);
void __attribute__((weak)) tegra_dc_cache_nvdisp_cmu(struct tegra_dc *dc,
			struct tegra_dc_nvdisp_cmu *src_cmu);
void __attribute__((weak)) tegra_dc_init_nvdisp_lut_defaults(
				struct tegra_dc_nvdisp_lut *nvdisp_lut);
void __attribute__((weak)) tegra_dc_set_nvdisp_lut(struct tegra_dc *dc,
						struct tegra_dc_win *win);
void __attribute__((weak)) tegra_dc_set_nvdisp_win_csc(struct tegra_dc *dc,
			struct tegra_dc_nvdisp_win_csc *nvdisp_win_csc);
int __attribute__((weak)) tegra_nvdisp_update_cmu(struct tegra_dc *dc,
			struct tegra_dc_nvdisp_lut *nvdisp_lut);
void __attribute__((weak)) tegra_nvdisp_get_default_cmu(
			struct tegra_dc_nvdisp_cmu *default_cmu);

void __attribute__((weak)) tegra_dc_populate_t18x_hw_data(
	struct tegra_dc_hw_data *);
void __attribute__((weak)) tegra_dc_populate_t19x_hw_data(
	struct tegra_dc_hw_data *);

void __attribute__((weak)) tegra_dc_enable_sor_t18x(struct tegra_dc *dc,
			int sor_num, bool enable);
void __attribute__((weak)) tegra_dc_enable_sor_t19x(struct tegra_dc *dc,
			int sor_num, bool enable);

void __attribute__((weak))
	tegra_nvdisp_set_rg_unstall_t19x(struct tegra_dc *dc);

uint64_t __attribute__((weak))
	tegra_dc_get_vsync_timestamp_t19x(struct tegra_dc *dc);
uint64_t tegra_dc_get_vsync_timestamp(struct tegra_dc *dc);

int __attribute__((weak)) nvdisp_t19x_program_raster_lock_seq(
				struct tegra_dc *dc, u32 value);

void __attribute__((weak)) nvdisp_t19x_enable_raster_lock(
			struct tegra_dc *dc, const ulong valid_heads);

void __attribute__((weak)) tegra_nvdisp_program_common_win_batch_size_t19x(
							struct tegra_dc *dc);

void __attribute__((weak))
	tegra_nvdisp_set_msrmnt_mode(struct tegra_dc *dc, bool enable);

int tegra_dc_en_dis_latency_msrmnt_mode(struct tegra_dc *dc, int enable);

struct tegra_dc_pd_table *tegra_dc_get_disp_pd_table(void);

int tegra_dc_client_handle_event(struct tegra_dc *dc,
		enum tegra_dc_client_cllbck_event_type event_type);

void tegra_dc_activate_general_channel(struct tegra_dc *dc);

int tegra_nvdisp_crc_enable(struct tegra_dc *dc,
			    struct tegra_dc_ext_crc_conf *conf);
int tegra_nvdisp_crc_disable(struct tegra_dc *dc,
			     struct tegra_dc_ext_crc_conf *conf);
int tegra_nvdisp_crc_collect(struct tegra_dc *dc,
			     struct tegra_dc_crc_buf_ele *crc_ele);
void tegra_nvdisp_crc_reset(struct tegra_dc *dc);

void tegra_nvdisp_set_output_lut(struct tegra_dc *dc,
	struct tegra_dc_ext_nvdisp_cmu *user_nvdisp_cmu, bool new_cmu_values);
void tegra_nvdisp_set_output_colorspace(struct tegra_dc *dc, u16 colorspace);
void tegra_nvdisp_set_output_range(struct tegra_dc *dc, u8 lim_range_enable);
void tegra_nvdisp_set_csc2(struct tegra_dc *dc);
void tegra_nvdisp_set_chroma_lpf(struct tegra_dc *dc);
void tegra_nvdisp_set_ocsc(struct tegra_dc *dc, struct tegra_dc_mode *mode);
void tegra_nvdisp_set_background_color(struct tegra_dc *dc,
					u32 background_color);
void tegra_nvdisp_activate_general_channel(struct tegra_dc *dc);
void tegra_nvdisp_set_vrr_mode(struct tegra_dc *dc);
void nvdisp_dc_feature_register(struct tegra_dc *dc);
int tegra_nvdisp_test_and_set_compclk(unsigned long rate,
					struct tegra_dc *dc);
int tegra_nvdisp_program_mode(struct tegra_dc *dc,
			struct tegra_dc_mode *mode);
int tegra_nvdisp_powergate_dc(struct tegra_dc *dc);
int tegra_nvdisp_unpowergate_dc(struct tegra_dc *dc);
int tegra_nvdisp_set_compclk(struct tegra_dc *dc);
int tegra_nvdisp_is_powered(struct tegra_dc *dc);
int nvdisp_set_cursor_position(struct tegra_dc *dc, s16 x, s16 y);
int nvdisp_set_cursor_colorfmt(struct tegra_dc *dc);
int tegra_nvdisp_set_degamma_user_config(struct tegra_dc_win *win,
				    long degamma_flag);
int tegra_nvdisp_get_degamma_user_config(struct tegra_dc_win *win);
int tegra_nvdisp_get_imp_caps(struct tegra_dc_ext_imp_caps *imp_caps);
int tegra_nvdisp_verify_win_properties(struct tegra_dc *dc,
				struct tegra_dc_ext_flip_windowattr *win);

static inline int tegra_dc_io_start(struct tegra_dc *dc)
{
	int ret = 0;
	ret = nvhost_module_busy_ext(dc->ndev);
	if (ret < 0) {
		dev_warn(&dc->ndev->dev,
			"Host1x powerup failed with err=%d\n", ret);
	}
	return ret;
}

static inline void tegra_dc_io_end(struct tegra_dc *dc)
{
	nvhost_module_idle_ext(dc->ndev);
}

static inline int tegra_dc_is_clk_enabled(struct clk *clk)
{
	return __clk_get_enable_count(clk);
}

static inline u32 ALL_UF_INT(void)
{
	if (tegra_dc_is_nvdisplay())
		return NVDISP_UF_INT;
	else
		return WIN_A_UF_INT | WIN_B_UF_INT | WIN_C_UF_INT | HC_UF_INT |
			WIN_D_UF_INT | WIN_T_UF_INT;
}

static inline long tegra_dc_calc_min_bandwidth(struct tegra_dc *dc)
{
	if (tegra_dc_is_nvdisplay())
		return tegra_nvdisp_calc_min_bandwidth(dc);
	else
		return tegra_calc_min_bandwidth(dc);
}

static inline void reg_dump(struct tegra_dc *dc, void *data,
			void (*print)(void *data, const char *str))
{
	if (tegra_dc_is_nvdisplay())
		return tegra_nvdisp_reg_dump(dc, data, print);
	else
		return tegra_dc_reg_dump(dc, data, print);
}

#if IS_ENABLED(CONFIG_PM_GENERIC_DOMAINS)
static inline void tegra_dc_powergate_locked(struct tegra_dc *dc)
{
	if (tegra_platform_is_sim() || tegra_platform_is_fpga())
		return;

	if (tegra_dc_is_nvdisplay())
		tegra_nvdisp_powergate_dc(dc);
	else
		tegra_powergate_partition(dc->powergate_id);
}

static inline void tegra_dc_unpowergate_locked(struct tegra_dc *dc)
{
	int ret;

	if (tegra_platform_is_sim() || tegra_platform_is_fpga())
		return;

	if (tegra_dc_is_nvdisplay())
		ret = tegra_nvdisp_unpowergate_dc(dc);
	else
		ret = tegra_unpowergate_partition(dc->powergate_id);

	if (ret < 0)
		dev_err(&dc->ndev->dev, "%s: could not unpowergate %d\n",
							__func__, ret);
}

static inline bool tegra_dc_is_powered(struct tegra_dc *dc)
{
	if (tegra_platform_is_sim() || tegra_platform_is_fpga())
		return true;

	if (tegra_dc_is_nvdisplay())
		return tegra_nvdisp_is_powered(dc);
	else
		return tegra_powergate_is_powered(dc->powergate_id);
}

void tegra_dc_powergate_locked(struct tegra_dc *dc);
void tegra_dc_unpowergate_locked(struct tegra_dc *dc);
#else /* !CONFIG_PM_GENERIC_DOMAINS */
static inline void tegra_dc_powergate_locked(struct tegra_dc *dc) { }
static inline void tegra_dc_unpowergate_locked(struct tegra_dc *dc) { }
static inline bool tegra_dc_is_powered(struct tegra_dc *dc)
{
	return true;
}
#endif /* CONFIG_PM_GENERIC_DOMAINS */

static inline unsigned long tegra_dc_is_accessible(struct tegra_dc *dc)
{
	if (!tegra_dc_is_nvdisplay()) {
		if (likely(tegra_platform_is_silicon())) {
			if (WARN(!nvhost_module_powered_ext(dc->ndev),
						"nvhost isn't powered\n"))
				return 1;
			if (WARN(!tegra_dc_is_clk_enabled(dc->clk),
						"DC is clock-gated.\n") ||
					WARN(!tegra_dc_is_powered(dc),
						"DC is power-gated.\n"))
				return 1;
		}
	}
	return 0;
}

static inline unsigned long tegra_dc_readl(struct tegra_dc *dc,
					   unsigned long reg)
{
	unsigned long ret;

	if (tegra_dc_is_accessible(dc))
		return 0;

	ret = readl(dc->base + reg * 4);
	trace_display_readl(dc, ret, (char *)dc->base + reg * 4);
	return ret;
}

static inline void tegra_dc_writel(struct tegra_dc *dc, unsigned long val,
				   unsigned long reg)
{
	if (tegra_dc_is_accessible(dc))
		return;

	trace_display_writel(dc, val, (char *)dc->base + reg * 4);
	writel(val, dc->base + reg * 4);
}

static inline void tegra_dc_power_on(struct tegra_dc *dc)
{
	if (!tegra_dc_is_nvdisplay())
		tegra_dc_writel(dc, PW0_ENABLE | PW1_ENABLE | PW2_ENABLE |
				PW3_ENABLE | PW4_ENABLE | PM0_ENABLE |
				PM1_ENABLE, DC_CMD_DISPLAY_POWER_CONTROL);
}

static inline void _tegra_dc_write_table(struct tegra_dc *dc, const u32 *table,
					 unsigned len)
{
	int i;

	for (i = 0; i < len; i++)
		tegra_dc_writel(dc, table[i * 2 + 1], table[i * 2]);
}

static inline u32 tegra_dc_get_frame_cnt(struct tegra_dc *dc)
{
	return tegra_dc_readl(dc, DC_COM_RG_DPCA) >> 16;
}

#define tegra_dc_write_table(dc, table)		\
	_tegra_dc_write_table(dc, table, ARRAY_SIZE(table) / 2)

static inline struct device_node *tegra_dc_get_conn_np(struct tegra_dc *dc)
{
	return dc->pdata->conn_np;
}

static inline struct device_node *tegra_dc_get_panel_np(struct tegra_dc *dc)
{
	return dc->pdata->panel_np;
}

static inline struct device_node *tegra_dc_get_out_np(struct tegra_dc *dc)
{
	return dc->pdata->def_out_np;
}

static inline void tegra_dc_set_outdata(struct tegra_dc *dc, void *data)
{
	dc->out_data = data;
}

static inline void *tegra_dc_get_outdata(struct tegra_dc *dc)
{
	return dc->out_data;
}

static inline unsigned long tegra_dc_get_default_emc_clk_rate(
	struct tegra_dc *dc)
{
	return dc->pdata->emc_clk_rate ? dc->pdata->emc_clk_rate : ULONG_MAX;
}

/* return the color format field */
static inline int tegra_dc_fmt(int fmt)
{
	return (fmt & TEGRA_DC_EXT_FMT_MASK) >> TEGRA_DC_EXT_FMT_SHIFT;
}

/* return the byte swap field */
static inline int tegra_dc_fmt_byteorder(int fmt)
{
	return (fmt & TEGRA_DC_EXT_FMT_BYTEORDER_MASK) >>
		TEGRA_DC_EXT_FMT_BYTEORDER_SHIFT;
}

static inline int tegra_dc_fmt_bpp(int fmt)
{
	switch (tegra_dc_fmt(fmt)) {
	case TEGRA_DC_EXT_FMT_T_P8:
		return 8;

	case TEGRA_DC_EXT_FMT_T_A4R4G4B4:
	case TEGRA_DC_EXT_FMT_T_A1R5G5B5:
	case TEGRA_DC_EXT_FMT_T_R5G6B5:
	case TEGRA_DC_EXT_FMT_T_R5G5B5A1:
	case TEGRA_DC_EXT_FMT_T_A4B4G4R4:
		return 16;

	case TEGRA_DC_EXT_FMT_T_A8R8G8B8:
	case TEGRA_DC_EXT_FMT_T_A8B8G8R8:
	case TEGRA_DC_EXT_FMT_T_A2R10G10B10:
	case TEGRA_DC_EXT_FMT_T_A2B10G10R10:
	case TEGRA_DC_EXT_FMT_T_X2BL10GL10RL10_XRBIAS:
	case TEGRA_DC_EXT_FMT_T_X2BL10GL10RL10_XVYCC:
		return 32;

	/* for planar formats, size of the Y plane, 8bit */
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N420:
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N420_TRUE:
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N422:
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N422_TRUE:
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N422R:
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N422R_TRUE:
	case TEGRA_DC_EXT_FMT_T_V8_Y8__U8_Y8:
	case TEGRA_DC_EXT_FMT_T_V8_Y8__U8_Y8_TRUE:
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N444:
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N444_TRUE:
	case TEGRA_DC_EXT_FMT_T_Y8___V8U8_N422_TRUE:
	case TEGRA_DC_EXT_FMT_T_Y8___V8U8_N420_TRUE:
	case TEGRA_DC_EXT_FMT_T_Y8___V8U8_N420:
	case TEGRA_DC_EXT_FMT_T_Y8___V8U8_N422:
	case TEGRA_DC_EXT_FMT_T_Y8___V8U8_N444:
	case TEGRA_DC_EXT_FMT_T_Y8___U8V8_N420:
	case TEGRA_DC_EXT_FMT_T_Y8___U8V8_N422:
	case TEGRA_DC_EXT_FMT_T_Y8___U8V8_N444:
	case TEGRA_DC_EXT_FMT_T_Y8___U8V8_N422R:
		return 8;

	case TEGRA_DC_EXT_FMT_T_Y10___U10___V10_N420:
	case TEGRA_DC_EXT_FMT_T_Y10___U10___V10_N444:
	case TEGRA_DC_EXT_FMT_T_Y10___V10U10_N420:
	case TEGRA_DC_EXT_FMT_T_Y10___U10V10_N422:
	case TEGRA_DC_EXT_FMT_T_Y10___U10V10_N422R:
	case TEGRA_DC_EXT_FMT_T_Y10___U10V10_N444:
	case TEGRA_DC_EXT_FMT_T_Y10___U10V10_N420:
	case TEGRA_DC_EXT_FMT_T_Y10___V10U10_N422:
	case TEGRA_DC_EXT_FMT_T_Y10___V10U10_N422R:
	case TEGRA_DC_EXT_FMT_T_Y10___V10U10_N444:
	case TEGRA_DC_EXT_FMT_T_Y10___V10___U10_N420:
	case TEGRA_DC_EXT_FMT_T_Y10___V10___U10_N444:
	case TEGRA_DC_EXT_FMT_T_Y12___U12___V12_N420:
	case TEGRA_DC_EXT_FMT_T_Y12___U12___V12_N444:
	case TEGRA_DC_EXT_FMT_T_Y12___V12U12_N420:
	case TEGRA_DC_EXT_FMT_T_Y12___U12V12_N422:
	case TEGRA_DC_EXT_FMT_T_Y12___U12V12_N422R:
	case TEGRA_DC_EXT_FMT_T_Y12___U12V12_N420:
	case TEGRA_DC_EXT_FMT_T_Y12___V12U12_N422:
	case TEGRA_DC_EXT_FMT_T_Y12___V12U12_N422R:
	case TEGRA_DC_EXT_FMT_T_Y12___U12V12_N444:
	case TEGRA_DC_EXT_FMT_T_Y12___V12U12_N444:
	case TEGRA_DC_EXT_FMT_T_Y12___V12___U12_N420:
	case TEGRA_DC_EXT_FMT_T_Y12___V12___U12_N444:
		return 16;

	/* YUV packed into 32-bits */
	case TEGRA_DC_EXT_FMT_T_U8_Y8__V8_Y8:
	case TEGRA_DC_EXT_FMT_T_Y8_U8__Y8_V8:
	case TEGRA_DC_EXT_FMT_T_U8_Y8__V8_Y8_TRUE:
		return 16;

	/* RGB with 64-bits size */
	case TEGRA_DC_EXT_FMT_T_R16_G16_B16_A16:
		return 64;

	}
	return 0;
}

static inline bool tegra_dc_is_yuv(int fmt)
{
	switch (tegra_dc_fmt(fmt)) {
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N420_TRUE:
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N420:
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N422:
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N422_TRUE:
	case TEGRA_DC_EXT_FMT_T_U8_Y8__V8_Y8:
	case TEGRA_DC_EXT_FMT_T_Y8_U8__Y8_V8:
	case TEGRA_DC_EXT_FMT_T_U8_Y8__V8_Y8_TRUE:
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N422R:
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N422R_TRUE:
	case TEGRA_DC_EXT_FMT_T_V8_Y8__U8_Y8:
	case TEGRA_DC_EXT_FMT_T_V8_Y8__U8_Y8_TRUE:
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N444:
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N444_TRUE:
	case TEGRA_DC_EXT_FMT_T_Y8___V8U8_N444:
	case TEGRA_DC_EXT_FMT_T_Y8___V8U8_N422_TRUE:
	case TEGRA_DC_EXT_FMT_T_Y8___V8U8_N422:
	case TEGRA_DC_EXT_FMT_T_Y8___V8U8_N420:
	case TEGRA_DC_EXT_FMT_T_Y8___V8U8_N420_TRUE:
	case TEGRA_DC_EXT_FMT_T_Y8___U8V8_N420:
	case TEGRA_DC_EXT_FMT_T_Y8___U8V8_N422:
	case TEGRA_DC_EXT_FMT_T_Y8___U8V8_N444:
	case TEGRA_DC_EXT_FMT_T_Y8___U8V8_N422R:

	case TEGRA_DC_EXT_FMT_T_Y10___U10___V10_N420:
	case TEGRA_DC_EXT_FMT_T_Y10___U10___V10_N444:
	case TEGRA_DC_EXT_FMT_T_Y10___V10U10_N420:
	case TEGRA_DC_EXT_FMT_T_Y10___U10V10_N422:
	case TEGRA_DC_EXT_FMT_T_Y10___U10V10_N422R:
	case TEGRA_DC_EXT_FMT_T_Y10___U10V10_N444:
	case TEGRA_DC_EXT_FMT_T_Y10___U10V10_N420:
	case TEGRA_DC_EXT_FMT_T_Y10___V10U10_N422:
	case TEGRA_DC_EXT_FMT_T_Y10___V10U10_N422R:
	case TEGRA_DC_EXT_FMT_T_Y10___V10U10_N444:
	case TEGRA_DC_EXT_FMT_T_Y10___V10___U10_N420:
	case TEGRA_DC_EXT_FMT_T_Y10___V10___U10_N444:
	case TEGRA_DC_EXT_FMT_T_Y12___U12V12_N420:
	case TEGRA_DC_EXT_FMT_T_Y12___V12U12_N422:
	case TEGRA_DC_EXT_FMT_T_Y12___V12U12_N422R:
	case TEGRA_DC_EXT_FMT_T_Y12___V12U12_N444:
	case TEGRA_DC_EXT_FMT_T_Y12___V12___U12_N420:
	case TEGRA_DC_EXT_FMT_T_Y12___V12___U12_N444:
	case TEGRA_DC_EXT_FMT_T_Y12___U12___V12_N420:
	case TEGRA_DC_EXT_FMT_T_Y12___U12___V12_N444:
	case TEGRA_DC_EXT_FMT_T_Y12___V12U12_N420:
	case TEGRA_DC_EXT_FMT_T_Y12___U12V12_N422:
	case TEGRA_DC_EXT_FMT_T_Y12___U12V12_N422R:
	case TEGRA_DC_EXT_FMT_T_Y12___U12V12_N444:

		return true;
	}
	return false;
}

static inline bool tegra_dc_is_yuv_planar(int fmt)
{
	switch (tegra_dc_fmt(fmt)) {
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N420_TRUE:
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N420:
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N422:
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N422_TRUE:
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N422R:
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N422R_TRUE:
	case TEGRA_DC_EXT_FMT_T_V8_Y8__U8_Y8:
	case TEGRA_DC_EXT_FMT_T_V8_Y8__U8_Y8_TRUE:
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N444:
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N444_TRUE:
	case TEGRA_DC_EXT_FMT_T_Y8___U8V8_N422:
	case TEGRA_DC_EXT_FMT_T_Y8___U8V8_N444:
	case TEGRA_DC_EXT_FMT_T_Y8___V8U8_N444:
	case TEGRA_DC_EXT_FMT_T_Y8___U8V8_N422R:
	case TEGRA_DC_EXT_FMT_T_Y10___U10___V10_N420:
	case TEGRA_DC_EXT_FMT_T_Y10___U10___V10_N444:
	case TEGRA_DC_EXT_FMT_T_Y10___V10U10_N420:
	case TEGRA_DC_EXT_FMT_T_Y10___U10V10_N422:
	case TEGRA_DC_EXT_FMT_T_Y10___U10V10_N422R:
	case TEGRA_DC_EXT_FMT_T_Y10___U10V10_N444:
	case TEGRA_DC_EXT_FMT_T_Y10___U10V10_N420:
	case TEGRA_DC_EXT_FMT_T_Y10___V10U10_N422:
	case TEGRA_DC_EXT_FMT_T_Y10___V10U10_N422R:
	case TEGRA_DC_EXT_FMT_T_Y10___V10U10_N444:
	case TEGRA_DC_EXT_FMT_T_Y10___V10___U10_N420:
	case TEGRA_DC_EXT_FMT_T_Y10___V10___U10_N444:
	case TEGRA_DC_EXT_FMT_T_Y12___V12___U12_N420:
	case TEGRA_DC_EXT_FMT_T_Y12___V12___U12_N444:
	case TEGRA_DC_EXT_FMT_T_Y12___U12V12_N420:
	case TEGRA_DC_EXT_FMT_T_Y12___U12___V12_N420:
	case TEGRA_DC_EXT_FMT_T_Y12___U12___V12_N444:
	case TEGRA_DC_EXT_FMT_T_Y12___V12U12_N420:
	case TEGRA_DC_EXT_FMT_T_Y12___U12V12_N422:
	case TEGRA_DC_EXT_FMT_T_Y12___U12V12_N422R:
	case TEGRA_DC_EXT_FMT_T_Y12___U12V12_N444:
	case TEGRA_DC_EXT_FMT_T_Y12___V12U12_N422:
	case TEGRA_DC_EXT_FMT_T_Y12___V12U12_N422R:
	case TEGRA_DC_EXT_FMT_T_Y12___V12U12_N444:
		return true;
	}
	return false;
}

static inline bool tegra_dc_is_yuv_full_planar(int fmt)
{
	switch (fmt) {
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N444:
	case TEGRA_DC_EXT_FMT_T_Y8___U8___V8_N444_TRUE:
	case TEGRA_DC_EXT_FMT_T_Y10___U10___V10_N420:
	case TEGRA_DC_EXT_FMT_T_Y10___U10___V10_N444:
	case TEGRA_DC_EXT_FMT_T_Y12___U12___V12_N420:
	case TEGRA_DC_EXT_FMT_T_Y12___U12___V12_N444:
	case TEGRA_DC_EXT_FMT_T_Y10___V10___U10_N420:
	case TEGRA_DC_EXT_FMT_T_Y10___V10___U10_N444:
	case TEGRA_DC_EXT_FMT_T_Y12___V12___U12_N420:
	case TEGRA_DC_EXT_FMT_T_Y12___V12___U12_N444:
		return true;
	}
	return false;
}

static inline bool tegra_dc_is_yuv_semi_planar(int fmt)
{
	switch (fmt) {
	case TEGRA_DC_EXT_FMT_T_Y8___V8U8_N420_TRUE:
	case TEGRA_DC_EXT_FMT_T_Y8___V8U8_N420:
	case TEGRA_DC_EXT_FMT_T_Y8___V8U8_N422:
	case TEGRA_DC_EXT_FMT_T_Y8___V8U8_N422_TRUE:
	case TEGRA_DC_EXT_FMT_T_Y8___U8V8_N420:
	case TEGRA_DC_EXT_FMT_T_Y8___U8V8_N422:
	case TEGRA_DC_EXT_FMT_T_Y8___U8V8_N444:
	case TEGRA_DC_EXT_FMT_T_Y8___V8U8_N444:
	case TEGRA_DC_EXT_FMT_T_Y8___U8V8_N422R:
	case TEGRA_DC_EXT_FMT_T_Y10___V10U10_N420:
	case TEGRA_DC_EXT_FMT_T_Y10___U10V10_N422:
	case TEGRA_DC_EXT_FMT_T_Y10___U10V10_N422R:
	case TEGRA_DC_EXT_FMT_T_Y10___U10V10_N444:
	case TEGRA_DC_EXT_FMT_T_Y10___U10V10_N420:
	case TEGRA_DC_EXT_FMT_T_Y10___V10U10_N422:
	case TEGRA_DC_EXT_FMT_T_Y10___V10U10_N422R:
	case TEGRA_DC_EXT_FMT_T_Y10___V10U10_N444:
	case TEGRA_DC_EXT_FMT_T_Y12___U12V12_N420:
	case TEGRA_DC_EXT_FMT_T_Y12___V12U12_N422:
	case TEGRA_DC_EXT_FMT_T_Y12___V12U12_N422R:
	case TEGRA_DC_EXT_FMT_T_Y12___V12U12_N444:
	case TEGRA_DC_EXT_FMT_T_Y12___V12U12_N420:
	case TEGRA_DC_EXT_FMT_T_Y12___U12V12_N422:
	case TEGRA_DC_EXT_FMT_T_Y12___U12V12_N422R:
	case TEGRA_DC_EXT_FMT_T_Y12___U12V12_N444:
		return true;
	}
	return false;
}

static inline bool tegra_dc_is_yuv420_8bpc(const struct tegra_dc_mode *mode)
{
	int yuv_flag = mode->vmode & FB_VMODE_YUV_MASK;

	if (yuv_flag & (FB_VMODE_Y420 | FB_VMODE_Y420_ONLY))
		return (yuv_flag & FB_VMODE_Y24);

	return false;
}

static inline bool tegra_dc_is_yuv420_10bpc(const struct tegra_dc_mode *mode)
{
	int yuv_flag = mode->vmode & FB_VMODE_YUV_MASK;

	if (yuv_flag & (FB_VMODE_Y420 | FB_VMODE_Y420_ONLY))
		return (yuv_flag & FB_VMODE_Y30);

	return false;
}

static inline bool tegra_dc_is_yuv422_12bpc(const struct tegra_dc_mode *mode)
{
	int yuv_flag = mode->vmode & FB_VMODE_YUV_MASK;

	return (yuv_flag == (FB_VMODE_Y422 | FB_VMODE_Y36));
}

static inline bool tegra_dc_is_yuv444_8bpc(const struct tegra_dc_mode *mode)
{
	int yuv_flag = mode->vmode & FB_VMODE_YUV_MASK;

	return (yuv_flag == (FB_VMODE_Y444 | FB_VMODE_Y24));
}

static inline u32 tegra_dc_unmask_interrupt(struct tegra_dc *dc, u32 int_val)
{
	u32 val;

	val = tegra_dc_readl(dc, DC_CMD_INT_MASK);
	tegra_dc_writel(dc, val | int_val, DC_CMD_INT_MASK);
	return val;
}

static inline u32 tegra_dc_flush_interrupt(struct tegra_dc *dc, u32 val)
{
	unsigned long flag;

	local_irq_save(flag);

	tegra_dc_writel(dc, val, DC_CMD_INT_STATUS);

	local_irq_restore(flag);

	return val;
}

static inline u32 tegra_dc_mask_interrupt(struct tegra_dc *dc, u32 int_val)
{
	u32 val;

	val = tegra_dc_readl(dc, DC_CMD_INT_MASK);
	tegra_dc_writel(dc, val & ~int_val, DC_CMD_INT_MASK);
	return val;
}

static inline void tegra_dc_restore_interrupt(struct tegra_dc *dc, u32 val)
{
	tegra_dc_writel(dc, val, DC_CMD_INT_MASK);
}

static inline int tegra_dc_clk_set_rate(struct tegra_dc *dc, unsigned long rate)
{
	if (!tegra_dc_is_nvdisplay())
		return 0;

	if (!tegra_platform_is_silicon() || !tegra_bpmp_running())
		return 0;

	if (clk_set_rate(dc->clk, rate)) {
		dev_err(&dc->ndev->dev, "Failed to set dc clk to %ld\n", rate);
		return -EINVAL;
	}

	tegra_nvdisp_set_compclk(dc);
	return 0;
}

static inline unsigned long tegra_dc_clk_get_rate(struct tegra_dc *dc)
{
	if (tegra_dc_is_nvdisplay()) {
		if (!tegra_platform_is_silicon() || !tegra_bpmp_running())
			return dc->mode.pclk;
	} else {
		if (!tegra_platform_is_silicon())
			return dc->mode.pclk;
	}

	return clk_get_rate(dc->clk);
}

static inline int tegra_disp_clk_prepare_enable(struct clk *clk)
{
	if (tegra_dc_is_nvdisplay()) {
		if (tegra_platform_is_silicon() && tegra_bpmp_running())
			return clk_prepare_enable(clk);
	} else {
		if (tegra_platform_is_silicon())
			return clk_prepare_enable(clk);
	}
	return 0;
}

static inline void tegra_disp_clk_disable_unprepare(struct clk *clk)
{
	if (tegra_dc_is_nvdisplay()) {
		if (tegra_platform_is_silicon() && tegra_bpmp_running())
			clk_disable_unprepare(clk);
	} else {
		if (tegra_platform_is_silicon())
			clk_disable_unprepare(clk);
	}
}

static inline void tegra_dc_set_edid(struct tegra_dc *dc,
	struct tegra_edid *edid)
{
	dc->edid = edid;
}

int tegra_dc_set_fbcon_boot_mode(struct tegra_dc *dc, struct tegra_edid *edid);

#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
static inline u32 tegra_dc_reg_l32(dma_addr_t v)
{
	return v & 0xffffffff;
}

static inline u32 tegra_dc_reg_h32(dma_addr_t v)
{
	return v >> 32;
}
#else
static inline u32 tegra_dc_reg_l32(dma_addr_t v)
{
	return v;
}

static inline u32 tegra_dc_reg_h32(dma_addr_t v)
{
	return 0;
}
#endif

#endif
