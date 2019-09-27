/*
 * tegra_dc_ext_kernel.h: tegra dc ext kernel module interface.
 *
 * Copyright (c) 2016-2018, NVIDIA CORPORATION, All rights reserved.
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

#ifndef VIDEO_DC_EXT_KERNEL_H
#define VIDEO_DC_EXT_KERNEL_H

#include <linux/types.h>
#include <drm/drm_fixed.h>

#define DC_N_WINDOWS		6

struct tegra_dc;
struct nvmap_handle_ref;

#define TEGRA_WIN_FLAG_ENABLED		(1 << 0)
#define TEGRA_WIN_FLAG_BLEND_PREMULT	(1 << 1)
#define TEGRA_WIN_FLAG_BLEND_COVERAGE	(1 << 2)
#define TEGRA_WIN_FLAG_INVERT_H		(1 << 3)
#define TEGRA_WIN_FLAG_INVERT_V		(1 << 4)
#define TEGRA_WIN_FLAG_TILED		(1 << 5)
#define TEGRA_WIN_FLAG_H_FILTER		(1 << 6)
#define TEGRA_WIN_FLAG_V_FILTER		(1 << 7)
#define TEGRA_WIN_FLAG_BLOCKLINEAR	(1 << 8)
#define TEGRA_WIN_FLAG_SCAN_COLUMN	(1 << 9)
#define TEGRA_WIN_FLAG_INTERLACE	(1 << 10)
#define TEGRA_WIN_FLAG_FB		(1 << 11)
#define TEGRA_WIN_FLAG_INPUT_RANGE_MASK	(3 << 12)
#define TEGRA_WIN_FLAG_INPUT_RANGE_FULL	(0 << 12)
#define TEGRA_WIN_FLAG_INPUT_RANGE_LIMITED	(1 << 12)
#define TEGRA_WIN_FLAG_INPUT_RANGE_BYPASS	(2 << 12)
#define TEGRA_WIN_FLAG_CS_MASK		(7 << 14)
#define TEGRA_WIN_FLAG_CS_NONE		(0 << 14)
#define TEGRA_WIN_FLAG_CS_REC601	(1 << 14)
#define TEGRA_WIN_FLAG_CS_REC709	(2 << 14)
#define TEGRA_WIN_FLAG_CS_REC2020	(4 << 14)
#define TEGRA_WIN_FLAG_DEGAMMA_MASK	(15 << 17)
#define TEGRA_WIN_FLAG_DEGAMMA_DEFAULT	(0 << 17) /* driver selects */
#define TEGRA_WIN_FLAG_DEGAMMA_NONE	(1 << 17)
#define TEGRA_WIN_FLAG_DEGAMMA_SRGB	(2 << 17)
#define TEGRA_WIN_FLAG_DEGAMMA_YUV_8_10	(4 << 17)
#define TEGRA_WIN_FLAG_DEGAMMA_YUV_12	(8 << 17)
#define TEGRA_WIN_FLAG_BLEND_ADD	(1 << 21)
#define TEGRA_WIN_FLAG_INVALID		(1 << 31) /* window does not exist. */

#define TEGRA_WIN_BLEND_FLAGS_MASK \
	(TEGRA_WIN_FLAG_BLEND_PREMULT | \
	 TEGRA_WIN_FLAG_BLEND_COVERAGE | \
	 TEGRA_WIN_FLAG_BLEND_ADD)

/* CSC struct for nvdisplay */
struct tegra_dc_nvdisp_win_csc {
	u32 r2r;
	u32 g2r;
	u32 b2r;
	u32 const2r;
	u32 r2g;
	u32 g2g;
	u32 b2g;
	u32 const2g;
	u32 r2b;
	u32 g2b;
	u32 b2b;
	u32 const2b;
	u32 csc_enable;
};

/* CSC struct for T210 */
struct tegra_dc_win_csc {
	unsigned short yof;
	unsigned short kyrgb;
	unsigned short kur;
	unsigned short kvr;
	unsigned short kug;
	unsigned short kvg;
	unsigned short kub;
	unsigned short kvb;
};

/* palette lookup table (T210)*/
struct tegra_dc_lut {
	u8 r[256];
	u8 g[256];
	u8 b[256];
};

/* lut table for nvdisplay */
struct tegra_dc_nvdisp_lut {
	u64 *rgb;
	dma_addr_t phy_addr;
	size_t size;
};

struct tegra_dc_win_cached_settings {
	bool clamp_before_blend;
	bool color_expand_enable;
};

struct tegra_dc_win {
	u8			idx;
	u8			ppflags; /* see TEGRA_WIN_PPFLAG* */
	u8			global_alpha;
	u32			fmt;
	u32			flags;

	void			*virt_addr;
	dma_addr_t		phys_addr;
	dma_addr_t		phys_addr_u;
	dma_addr_t		phys_addr_v;
	/* field 2 starting address */
	dma_addr_t		phys_addr2;
	dma_addr_t		phys_addr_u2;
	dma_addr_t		phys_addr_v2;
	unsigned		stride;
	unsigned		stride_uv;
	fixed20_12		x;
	fixed20_12		y;
	fixed20_12		w;
	fixed20_12		h;
	unsigned		out_x;
	unsigned		out_y;
	unsigned		out_w;
	unsigned		out_h;
	unsigned		z;

	struct tegra_dc_win_cached_settings cached_settings;

	struct tegra_dc_nvdisp_win_csc	nvdisp_win_csc;
	bool force_user_csc;
	bool force_user_degamma;
	struct tegra_dc_win_csc	win_csc;
	bool			csc_dirty;

	int			dirty;
	int			underflows;
	struct tegra_dc		*dc;

	struct nvmap_handle_ref	*cur_handle;
	unsigned		bandwidth;
	unsigned		new_bandwidth;
	struct tegra_dc_lut	lut;
	struct tegra_dc_nvdisp_lut	nvdisp_lut;
	u8	block_height_log2;
	struct {
		dma_addr_t cde_addr;
		unsigned offset_x;
		unsigned offset_y;
		u32 zbc_color;
		unsigned ctb_entry;
	} cde;
	struct {
		u32			id;
		u32			min;
		u32			max;
	} syncpt;

	bool		is_scaler_coeff_set;
	bool		color_expand_enable;
	bool		clamp_before_blend;

	/* Used only on Nvdisplay */
	bool		precomp_caps_read;
	u32		precomp_capc;
	u32		precomp_cape;
};

struct tegra_dc *tegra_dc_get_dc(unsigned idx);
#ifdef CONFIG_TEGRA_ISOMGR
int tegra_dc_bandwidth_negotiate_bw(struct tegra_dc *dc,
			struct tegra_dc_win *windows[], int n);
#endif
int tegra_dc_get_numof_dispheads(void);
int tegra_dc_get_numof_dispwindows(void);
int tegra_dc_get_numof_dispsors(void);

/* needed by tegra-throughput */
int tegra_dc_set_flip_callback(void (*callback)(void));
int tegra_dc_unset_flip_callback(void);
int tegra_dc_get_panel_sync_rate(void);

/* needed by tegra-mods */
unsigned long tegra_dc_readl_exported(struct tegra_dc *, unsigned long);
void tegra_dc_writel_exported(struct tegra_dc *, unsigned long, unsigned long);

/* Forward Declaration. Please see below for structure details */
struct tegra_dc_client;

/*
 * tegra_dc_notify_dc_enabled_event - callback function to notify client
 * when dc is enabled.
 * @disp_id: logical id represented by suffix to fb device, i.e. 0 for
 * fb0, 1 for fb1 etc.
 * @usr_ctx: Any user context if present.
 *
 * Return: void
 */
typedef void (*tegra_dc_notify_dc_enabled_event)(int disp_id, void *usr_ctx);

/*
 * tegra_dc_notify_dc_disabled_event- callback function to notify client
 * when dc is disabled.
 * @disp_id: logical id represented by suffix to fb device, i.e. 0 for
 * fb0, 1 for fb1 etc.
 * @usr_ctx: Any user context if present.
 *
 * Return: void
 */
typedef void (*tegra_dc_notify_dc_disabled_event)(int disp_id, void *usr_ctx);

/*
 * tegra_dc_notify_modeset_event - callback function to notify the
 * client when modeset occurs.
 * @disp_id: logical id represented by suffix to fb device, i.e. 0 for
 * fb0, 1 for fb1 etc.
 * @usr_ctx: Any user context if present.
 */
typedef void (*tegra_dc_notify_modeset_event)(int disp_id, void *usr_ctx);

/*
 * tegra_dc_get_max_lines() - Get the base address of a head
 * @disp_id: logical id represented by suffix to fb device, i.e. 0 for
 * fb0, 1 for fb1 etc.
 *
 * Return : v_total for the current display if successful else an error value.
 */
int tegra_dc_get_max_lines(int disp_id);

/*
 * tegra_dc_get_addr_info() - Get the base address of a head
 * @disp_id: logical id represented by suffix to fb device, i.e. 0 for
 * fb0, 1 for fb1 etc.
 * @res : ptr to resource info to be filled by dc driver.
 *
 * Return : 0 if successful else an error value.
 */
int tegra_dc_get_addr_info(int disp_id, struct resource *res);

/*
 * tegra_dc_register_client() - used by clients to register to dc driver
 * @client : pointer to client's data
 *
 * Return: 0 if no errors else corresponding error value.
 */
int tegra_dc_register_client(struct tegra_dc_client *client);

/*
 * tegra_dc_unregister_client() - used by clients to unregister to dc driver
 * @client : pointer to client's data
 *
 * Return: 0 if no errors else corresponding error value.
 */
int tegra_dc_unregister_client(struct tegra_dc_client *client);

/*
 * enum tegra_dc_client_cllbck_event_type - defines all the supported events
 * on which a dc_client can request a callback
 * @NOTIFY_DC_ENABLED_EVENT : when dc is enabled
 * @NOTIFY_DC_DISABLED_EVENT : when dc is disabled
 * @NOTIFY_MODESET_EVENT : when a modeset occurs
 * @MAX_EVENT : max no. of supported events.
 */
enum tegra_dc_client_cllbck_event_type {
	NOTIFY_DC_ENABLED_EVENT,
	NOTIFY_DC_DISABLED_EVENT,
	NOTIFY_MODESET_EVENT,
	MAX_EVENT
};

/*
 * struct tegra_dc_client_callbck_data - data structure to be used by dc
 * clients for registering callback functions.
 * @callback_type : states the type of callback function.
 * @reserved0 : reserved for future pointer.
 * @reserved1 : reserved for future use (data_type of the future pointer above)
 * @callback_fn : ptr to the callback function.
 *
 * This is meant to be scalable and backward compatible in future if we plan to
 * extend the number of callback functions rather than going of fixed array or
 * list of callback functions.
 */
struct tegra_dc_client_callbck_data {
	u32 callback_type;
	u64 reserved0;
	u32 reserved1;
	void *callback_fn;
};

/*
 * struct tegra_dc_client - structure to be used by external clients
 * registering with dc driver for notifications.
 */
struct tegra_dc_client {
	/*
	 * @disp_id: Used to specify the head to which the client has
	 * registered to receive notifications. To be filled by the client.
	 * This particularly is the fb#id of the head and not the hardware id
	 * for the heads.
	 */
	int disp_id;
	/*
	 * @client_id: Allocated by dc driver to keep track of regsitered
	 * clients per head.
	 */
	int client_id;
	/*
	 * @user_ctx: Stores any context provided by the client.
	 */
	void *usr_ctx;
	/*
	 * @nr_callbacks: Used to specify the number of callback functions
	 * the client wants to register for.
	 */
	int nr_callbacks;
	/*
	 * @callback_data: Points to the callback data that has info regarding
	 * the callback fucntions registered by the client.
	 */
	struct tegra_dc_client_callbck_data *callback_data;
};

#endif
