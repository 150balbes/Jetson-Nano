/*
 * dc_priv.h: dc priv interface.
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

#ifndef __DRIVERS_VIDEO_TEGRA_DC_DC_PRIV_DEFS_H
#define __DRIVERS_VIDEO_TEGRA_DC_DC_PRIV_DEFS_H
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/fb.h>
#include <linux/clk.h>
#include <linux/completion.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif
#include <linux/nvhost.h>
#include <linux/types.h>
#include <linux/clk/tegra.h>
#include <soc/tegra/chip-id.h>
#include <linux/reset.h>
#include <uapi/video/tegra_dc_ext.h>
#include <linux/platform/tegra/isomgr.h>

#include "dc.h"
#include "dc_reg.h"

#define NEED_UPDATE_EMC_ON_EVERY_FRAME (windows_idle_detection_time == 0)

/* 28 bit offset for window clip number */
#define CURSOR_CLIP_SHIFT_BITS(win)	(win << 28)
#define CURSOR_CLIP_GET_WINDOW(reg)	((reg >> 28) & 3)

#define BLANK_ALL	(~0)

#if defined(CONFIG_TEGRA_EMC_TO_DDR_CLOCK)
#define EMC_BW_TO_FREQ(bw) (DDR_BW_TO_FREQ(bw) * CONFIG_TEGRA_EMC_TO_DDR_CLOCK)
#else
#define EMC_BW_TO_FREQ(bw) (DDR_BW_TO_FREQ(bw) * 2)
#endif

#define MAX_NO_DC_CLIENTS 8

struct tegra_dc;

struct tegra_dc_blend {
	unsigned z[DC_N_WINDOWS];
	unsigned flags[DC_N_WINDOWS];
	u8 alpha[DC_N_WINDOWS];
};

struct tegra_dc_out_ops {
	/* initialize output.  dc clocks are not on at this point */
	int (*init)(struct tegra_dc *dc);
	/* destroy output.  dc clocks are not on at this point */
	void (*destroy)(struct tegra_dc *dc);
	/* shutdown output.  dc clocks are on at this point */
	void (*shutdown)(struct tegra_dc *dc);
	/* detect connected display.  can sleep.*/
	bool (*detect)(struct tegra_dc *dc);
	/* enable output.  dc clocks are on at this point */
	void (*enable)(struct tegra_dc *dc);
	/* enable dc client.  Panel is enable at this point */
	void (*postpoweron)(struct tegra_dc *dc);
	/* disable output.  dc clocks are on at this point */
	void (*disable)(struct tegra_dc *dc);
	/* dc client is disabled.  dc clocks are on at this point */
	void (*postpoweroff) (struct tegra_dc *dc);
	/* hold output.  keeps dc clocks on. */
	void (*hold)(struct tegra_dc *dc);
	/* release output.  dc clocks may turn off after this. */
	void (*release)(struct tegra_dc *dc);
	/* idle routine of output.  dc clocks may turn off after this. */
	void (*idle)(struct tegra_dc *dc);
	/* suspend output.  dc clocks are on at this point */
	void (*suspend)(struct tegra_dc *dc);
	/* resume output.  dc clocks are on at this point */
	void (*resume)(struct tegra_dc *dc);
	/* mode filter. to provide a list of supported modes*/
	bool (*mode_filter)(const struct tegra_dc *dc,
			struct fb_videomode *mode);
	/* setup pixel clock and parent clock programming */
	long (*setup_clk)(struct tegra_dc *dc, struct clk *clk);
	/*
	 * return true if display client is suspended during OSidle.
	 * If true, dc will not wait on any display client event
	 * during OSidle.
	 */
	bool (*osidle)(struct tegra_dc *dc);
	/* callback before new mode is programmed.
	 * dc clocks are on at this point */
	void (*modeset_notifier)(struct tegra_dc *dc);
	/* Set up interface and sink for partial frame update.
	 */
	int (*partial_update) (struct tegra_dc *dc, unsigned int *xoff,
		unsigned int *yoff, unsigned int *width, unsigned int *height);
	/* refcounted enable of pads and clocks before performing DDC/I2C. */
	int (*ddc_enable)(struct tegra_dc *dc);
	/* refcounted disable of pads and clocks after performing DDC/I2C. */
	int (*ddc_disable)(struct tegra_dc *dc);
	/* Enable/disable VRR */
	void (*vrr_enable)(struct tegra_dc *dc, bool enable);
	/* Mark VRR-compatible modes in fb_info->info->modelist */
	void (*vrr_update_monspecs)(struct tegra_dc *dc,
		struct list_head *head);
	/* return if hpd asserted or deasserted */
	bool (*hpd_state)(struct tegra_dc *dc);
	/* Configure controller to receive hotplug events */
	int (*hotplug_init)(struct tegra_dc *dc);
	/* Configure controller for avi infoframe update */
	int (*set_avi)(struct tegra_dc *dc, struct tegra_dc_ext_avi *avi);
	/* Configure controller for hdr infoframe update */
	int (*set_hdr)(struct tegra_dc *dc);
	/* shutdown the serial interface */
	void (*shutdown_interface)(struct tegra_dc *dc);
	u32 (*get_crc)(struct tegra_dc *dc);
	void (*toggle_crc)(struct tegra_dc *dc, u32 val);
	/* returns sor ctrl_num, it can be extended to DSI if needed */
	int (*get_connector_instance)(struct tegra_dc *dc);
	/* Accesses the underlying OR's CRC mechanisms */
	int (*crc_en)(struct tegra_dc *dc,
		      struct tegra_dc_ext_crc_or_params *params);
	int (*crc_dis)(struct tegra_dc *dc,
		       struct tegra_dc_ext_crc_or_params *params);
	int (*crc_get)(struct tegra_dc *dc, u32 *crc);
};

struct tegra_dc_shift_clk_div {
	unsigned long mul; /* numerator */
	unsigned long div; /* denominator */
};

enum tegra_dc_cursor_size {
	TEGRA_DC_CURSOR_SIZE_32X32 = 0,
	TEGRA_DC_CURSOR_SIZE_64X64 = 1,
	TEGRA_DC_CURSOR_SIZE_128X128 = 2,
	TEGRA_DC_CURSOR_SIZE_256X256 = 3,
};

enum tegra_dc_cursor_blend_format {
	TEGRA_DC_CURSOR_FORMAT_2BIT_LEGACY = 0,
	TEGRA_DC_CURSOR_FORMAT_RGBA_NON_PREMULT_ALPHA = 1,
	TEGRA_DC_CURSOR_FORMAT_RGBA_PREMULT_ALPHA = 3,
	TEGRA_DC_CURSOR_FORMAT_RGBA_XOR = 4,
};

enum tegra_dc_cursor_color_format {
	TEGRA_DC_CURSOR_COLORFMT_LEGACY,   /* 2bpp   */
	TEGRA_DC_CURSOR_COLORFMT_R8G8B8A8, /* normal */
	TEGRA_DC_CURSOR_COLORFMT_A1R5G5B5,
	TEGRA_DC_CURSOR_COLORFMT_A8R8G8B8,
};

struct tegra_nvdisp_mempool_req {
	bool program_cursor;
	u32 cursor_entry;

	u32 *win_entries;
	u8 *win_ids;
	u8 num_wins;

	u8 ctrl_num;
};

struct tegra_nvdisp_imp_head_settings {
	struct tegra_dc_ext_nvdisp_imp_head_entries entries;

	struct tegra_dc_ext_nvdisp_imp_win_entries *win_entries;
	u8 num_wins;
};

/*
 * This struct is the internal representation that kernel will use to store the
 * relevant IMP settings. Both the v1 and v2 dc_ext IMP structs will be
 * converted to this internal format.
 */
struct tegra_nvdisp_imp_settings {
	struct tegra_dc_ext_nvdisp_imp_global_entries global_entries;

	struct tegra_nvdisp_imp_head_settings *head_settings;
	u8 num_heads;

	struct tegra_nvdisp_mempool_req *decreasing_pool_reqs;
	struct tegra_nvdisp_mempool_req *increasing_pool_reqs;
	u8 num_decreasing_pool;
	u8 num_increasing_pool;

	u64 session_id;
	u8 owner_ctrl_num;

	struct list_head imp_node;
};

enum tegra_dc_hw {
	TEGRA_DC_HW_T210,
	TEGRA_DC_HW_T18x,
	TEGRA_DC_HW_T19x,
	TEGRA_DC_HW_MAX
};

struct tegra_dc_pd_clk_info {
	const char	*name;
	struct clk	*clk;
};

struct tegra_dc_pd_info {
	struct of_device_id			of_id[2];
	int					pg_id;

	const u32				head_owner;
	const u32				head_mask;
	const u32				win_mask;

	struct tegra_dc_pd_clk_info		*domain_clks;
	const int				nclks;

	int					ref_cnt;
};

struct tegra_dc_pd_table {
	struct tegra_dc_pd_info *pd_entries;
	int npd;
	struct mutex pd_lock;
};

struct tegra_dc_sor_info {
	bool hdcp_supported;
};

struct tegra_dc_hw_data {
	bool valid;
	int nheads;
	int nwins;
	int nsors;
	struct tegra_dc_sor_info *sor_info; /* per SOR info */
	struct tegra_dc_pd_table *pd_table;
	enum tegra_dc_hw version;
};

struct tegra_dc_topology {
	bool valid;	/* check if topology is dangling */
	int disp_id;	/* fb id and display logical instance */
	int protocol;	/* out type */
	int conn_inst;	/* SOR/DSI instance number. */
};

struct tegra_dc_flip_stats {
	atomic64_t flips_skipped;
	atomic64_t flips_queued;
	atomic64_t flips_cmpltd;
};

/*
 * struct tegra_dc_client_data - stores all per client specific data for
 * required for notifying when the requested events occur.
 * @registered : flag to keep track of client's registration status.
 * @usr_ctx : stores any context provided by the client.
 * @callback_fn : array of callback functions requested by the client. The
 * indices are controlled by the corresponding event_type.
 */
struct tegra_dc_client_data {
	bool registered;
	void *usr_ctx;
	void *callback_fn[MAX_EVENT];
};

/*
 * struct tegra_dc_clients_info - Used to store info regarding all the clients
 * registered with a specific head.
 * @client_id_map : a pool of resource for client ids under a specific head.
 * @client_data : array storing all individual client's info.
 */
struct tegra_dc_clients_info {
	unsigned long client_id_map;
	struct tegra_dc_client_data client_data[MAX_NO_DC_CLIENTS];
};

struct tegra_dc_cached_settings {
	u32 csc2_control;
};

/* Data structures related to CRC IOCTLs */

#define TEGRA_DC_MAX_CRC_REGIONS 9

enum tegra_dc_flip_state {
	TEGRA_DC_FLIP_STATE_QUEUED,
	TEGRA_DC_FLIP_STATE_DEQUEUED,
	TEGRA_DC_FLIP_STATE_FLIPPED,
	TEGRA_DC_FLIP_STATE_SKIPPED,
	TEGRA_DC_FLIP_STATE_MAX,
};

/*
 * tegra_dc_flip_buf_ele - Single element of the circular buffer holding flips
 * @id  - The unique ID of the flip, assigned by FLIP4 IOCTL
 * @state    - The stage the flip is in, during its lifecycle
 *             TEGRA_DC_FLIP_STATE_QUEUED - The FLIP4 IOCTL has queued the
 *                                          flip, but is yet to be worked upon
 *                                          by the flip worker
 *             TEGRA_DC_FLIP_STATE_DEQUEUED - The flip worker has started
 *                                            working on this flip
 *             TEGRA_DC_FLIP_STATE_FLIPPED - The flip worker is done waiting
 *                                           for the frame end interrupt
 *                                           servicing this flip
 *             TEGRA_DC_FLIP_STATE_SKIPPED - The flip will not be processed
 */
struct tegra_dc_flip_buf_ele {
	u64 id;
	enum tegra_dc_flip_state state;
};

/*
 * tegra_dc_crc_buf_ele - Single element of the circular buffer holding CRCs
 * @flip      - The flip ID of the flip corresponding to the frame for
 *              which CRC was generated. "id" member stores the ID whereas
 *              "valid" member denotes whether the match between the CRC
 *              and FLIP queue data structures has happened yet
 *              The array size equals total number of DC Windows, since that
 *              is the maximum concurrent active flip workers for a given
 *              frame. Please note that the array index is not meant to be used
 *              as a per window index
 * @rg/sor/comp/regional - HW generated CRC queued by FRAME_END_INT interrupt.
 *             "crc" member holds the value, while "valid" member says whether
 *             the crc field is valid
 */
struct tegra_dc_crc_buf_ele {
	struct {
		u64 id;
		bool valid;
	} matching_flips[DC_N_WINDOWS];
	struct {
		u32 crc;
		bool valid;
	} rg, sor, comp, regional[TEGRA_DC_MAX_CRC_REGIONS];
};

enum tegra_dc_ring_buf_type {
	TEGRA_DC_RING_BUF_FLIP,
	TEGRA_DC_RING_BUF_CRC
};

/*
 * tegra_dc_ring_buf - Circular buffer for storing flips and CRCs
 * @head     - Array index where next item will be buffered
 * @tail     - Array index pointing to least recently buffered item
 * @size     - The number of valid items in the buffer
 * @capacity - The maximum number of valid items in the buffer. Set this value
 *             during initialization
 * @data     - The actual memory for the buffer
 * @lock     - Mutex to serialize accesses across various contexts, namely the
 *             flip IOCTL, flip worker thread and frame end interrupt service
 *             routine
 */
struct tegra_dc_ring_buf {
	enum tegra_dc_ring_buf_type type;
	u16 head;
	u16 tail;
	u16 size;
	u16 capacity;
	char *data;
	struct mutex lock;
};

/*
 * tegra_dc_crc_ref_count - Reference counts for various CRC features
 *                ### Note ###
 *                o Legacy mechanism for collecting CRC via sysfs interface
 *                  can not be simultaneously activated along with the IOCTL
 *                  interface
 *                o If required expand the ref counts per block (RG/SOR/COMP)
 *                  or per region
 * @global      - Track CRC as a client of the Frame End Interrupt
 * @rg          - Track if RG CRC is enabled
 * @comp        - Track if COMP CRC is enabled
 * @out         - Track if CRC of an Output Resource (OR) is enabled
 * @region      - Track if CRC for a region is enabled
 * @regional    - Track if regional CRCs are enabled
 *                Should always be a sum of reference counts of each region
 * @legacy      - Keep account of whether legacy sysfs API is activated
 */
struct tegra_dc_crc_ref_cnt {
	atomic_t global;
	atomic_t rg;
	atomic_t comp;
	atomic_t out;
	atomic_t region[TEGRA_DC_MAX_CRC_REGIONS];
	atomic_t regional;
	bool legacy;
};

/*
 * struct tegra_dc_latency_measurement_data - data structure used for
 *		latency measurement purpose.
 * @buf_handle : dma_buff handle used for storing currently scanned
 *		framebuffer by dc.
 * @offset : offset for the first pixel in the current fb.
 * @enabled : stores the status that indicates whether measurement info
 *		has to be collected or not.
 * @line_num : the scanline at which the latency value is being read.
 * @lock : used to sequentialize operations on
 *		tegra_dc_latency_measurement_data.
 */
struct tegra_dc_latency_measurement_data {
	struct dma_buf *buf_handle;
	u32 offset;
	bool enabled;
	u16 line_num;
	struct mutex lock;
};

/* Add specific variable related to each output type.
 * Save and reuse on changing the output type
 */
#if defined(CONFIG_TEGRA_DC_FAKE_PANEL_SUPPORT)
struct tegra_dc_out_info {
	struct tegra_dc_out_ops *out_ops;
	void *out_data;
	struct tegra_dc_out out;
	struct tegra_dc_mode mode;
	int fblistindex;
	struct tegra_edid *edid;
};
#endif

/*
 * struct tegra_dc_win_detach_state - Caches the relevant window register state
 * when disabling and restoring windows during SOR detach
 *
 * This struct is only used on nvdisplay.
 *
 * @win_set_control_reg - Cached win_set_control_r() value before disable
 * @win_scaler_usage_reg - Cached win_scaler_usage_r() value before disable
 * @win_options_reg - Cached win_options_r() value before disable
 */
struct tegra_dc_win_detach_state {
	u32 win_set_control_reg;
	u32 win_scaler_usage_reg;
	u32 win_options_reg;
};

struct tegra_dc {
	struct platform_device		*ndev;
	struct tegra_dc_platform_data	*pdata;
	void __iomem			*base;
	int				irq;

	struct clk			*clk;
	struct clk			*parent_clk_safe;
	struct reset_control		*rst;

	/* Nvdisplay and Isomgr */
	/* Reference to a single instance */
	struct nvdisp_isoclient_bw_info	*ihub_bw_info;
	bool				la_dirty;

	/* T21x and Isomgr */
	tegra_isomgr_handle		isomgr_handle;
	u32				reserved_bw;
	u32				available_bw;

	/* Used when Isomgr is not defined */
	struct clk			*emc_clk;

	struct tegra_bwmgr_client	*emc_la_handle;
	long				bw_kbps; /* bandwidth in KBps */
	long				new_bw_kbps;
	struct tegra_dc_shift_clk_div	shift_clk_div;

	u32				powergate_id;

	bool				connected;
	bool				enabled;
	bool				suspended;
	bool				blanked;
	bool				shutdown;
	bool				reenable_on_resume;

	/* Some of the setup code could reset display even if
	 * DC is already by bootloader.  This one-time mark is
	 * used to suppress such code blocks during system boot,
	 * a.k.a the call stack above tegra_dc_probe().
	 */
	bool				initialized;
#ifdef CONFIG_TEGRA_HDMI2FPD
	struct tegra_dc_hdmi2fpd_data   *fpddata;
#endif
	struct tegra_dc_out		*out;
	struct tegra_dc_out_ops		*out_ops;
	void				*out_data;

	struct tegra_dc_mode		mode;
	struct tegra_dc_mode		cached_mode;
	bool				use_cached_mode;
	s64				frametime_ns;
	struct tegra_dc_mode_metadata	mode_metadata;

	/* Used only on T21x */
	struct tegra_dc_win		windows[DC_N_WINDOWS];

	struct tegra_dc_win		shadow_windows[DC_N_WINDOWS];

	struct tegra_dc_blend		blend;
	int				n_windows;
	struct tegra_dc_hdr		hdr;

	/* Used only on Nvdisplay */
	bool					common_channel_reserved;
	bool					common_channel_pending;
	bool					common_channel_intr_enabled;
	bool					comp_clk_inuse;

	bool					imp_dirty;
	u64					imp_session_id_cntr;

	struct tegra_dc_cmu		cmu;
	struct tegra_dc_nvdisp_lut	nvdisp_postcomp_lut;

	/* either unity or panel specific */
	struct tegra_dc_nvdisp_win_csc	default_csc;

	struct tegra_dc_cmu		cmu_shadow;
	bool				cmu_dirty;
	/* Is CMU set by bootloader */
	bool				is_cmu_set_bl;
	bool				cmu_shadow_dirty;
	bool				cmu_shadow_force_update;
	bool				cmu_enabled;

	struct tegra_dc_lut		fb_lut;
	struct tegra_dc_nvdisp_lut	fb_nvdisp_lut;
	wait_queue_head_t		wq;
	wait_queue_head_t		timestamp_wq;

	struct mutex			lp_lock;
	struct mutex			lock;
	struct mutex			one_shot_lock;

	struct resource			*fb_mem;
	struct tegra_fb_info		*fb;

	u32				vblank_syncpt;
	u32				vpulse3_syncpt;

	unsigned long int		valid_windows;

	unsigned long			underflow_mask;
	struct work_struct		reset_work;

#ifdef CONFIG_SWITCH
	struct switch_dev		modeset_switch;
#endif

	struct completion		frame_end_complete;
	struct completion		crc_complete;
	struct completion		hpd_complete;
	bool				crc_pending;

	struct work_struct		vblank_work;
	long				vblank_ref_count;
	struct work_struct		frame_end_work;
	struct work_struct		vpulse2_work;
	long				vpulse2_ref_count;

	struct {
		u64			underflows;
		u64			underflows_a;
		u64			underflows_b;
		u64			underflows_c;
		u64			underflows_d;
		u64			underflows_h;
		u64			underflows_t;
		u64			underflow_frames;
	} stats;

	struct tegra_dc_ext		*ext;

	struct tegra_dc_cached_settings cached_settings;

	struct tegra_dc_feature		*feature;
	int				gen1_blend_num;

#ifdef CONFIG_DEBUG_FS
	struct dentry			*debugdir;
	struct dentry			*sor_link;  /*symbolic link to SOR debugfs*/

	/* Used only on Nvdisplay */
	struct dentry			*debug_common_dir;
#endif
	struct tegra_dc_topology		boot_topology;
	struct tegra_dc_topology		current_topology;
	struct delayed_work		underflow_work;
	u32				one_shot_delay_ms;
	struct delayed_work		one_shot_work;
	s64				frame_end_timestamp;
	atomic_t			frame_end_ref;

	/* Nvdisplay specific */
	struct delayed_work		vrr_work;

	bool				mode_dirty;
	bool				yuv_bypass;
	bool				yuv_bypass_dirty;
	atomic_t			holding;

	struct tegra_dc_win		tmp_wins[DC_N_WINDOWS];

	struct tegra_edid		*edid;

	bool	disp_active_dirty;

	struct tegra_dc_cursor {
		bool dirty;
		bool enabled;
		dma_addr_t phys_addr;
		u32 fg;
		u32 bg;
		unsigned clip_win;
		int x;
		int y;
		enum tegra_dc_cursor_size size;
		enum tegra_dc_cursor_blend_format blendfmt;
		enum tegra_dc_cursor_color_format colorfmt;
		u32 alpha;
	} cursor;

	int	ctrl_num;
	bool	switchdev_registered;

	struct notifier_block slgc_notifier;
	bool	vedid;
	u8	*vedid_data;
	atomic_t	enable_count;
	bool	hdr_cache_dirty;
	/* hotplug_supported refers to the capability of the protocol
	 * to support hotplug and should not be used to ascertain the
	 * HW support for hotplug on the device
	 * e.g. DP/HDMI: hotplug supported | EDP/DSI: hotplug not supported
	 * for fake output types:
	 * FAKE_DSI: hotplug not supported
	 * FAKE_DP: hotplug supported if ext_dp_panel DT flag is set
	 * */
	bool    hotplug_supported;

	/* user call-back for shared ISR */
	int  (*isr_usr_cb)(int dcid, unsigned long irq_sts, void *usr_pdt);
	void  *isr_usr_pdt;

	u32 dbg_fe_count;
	struct frame_lock_info frm_lck_info;
	unsigned long act_req_mask;
	struct tegra_dc_clients_info clients_info;
	struct tegra_dc_flip_stats flip_stats;

	struct tegra_dc_ring_buf flip_buf; /* Buffer to save flip requests */
	struct tegra_dc_ring_buf crc_buf; /* Buffer to save HW generated CRCs */
	struct tegra_dc_crc_ref_cnt crc_ref_cnt;
	bool crc_initialized;
	struct tegra_dc_latency_measurement_data msrmnt_info;

#if defined(CONFIG_TEGRA_DC_FAKE_PANEL_SUPPORT)
	struct tegra_dc_out_info dbg_dc_out_info[TEGRA_DC_OUT_MAX];
#endif
};
#endif
