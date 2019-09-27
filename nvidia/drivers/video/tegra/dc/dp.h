/*
 * dp.h: tegra dp driver.
 *
 * Copyright (c) 2011-2018, NVIDIA CORPORATION, All rights reserved.
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

#ifndef __DRIVER_VIDEO_TEGRA_DC_DP_H__
#define __DRIVER_VIDEO_TEGRA_DC_DP_H__

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/reset.h>
#include <linux/tegra_prod.h>

#include "sor.h"
#include "dc_priv.h"
#include "dpaux_regs.h"
#include "hpd.h"
#include "../../../../arch/arm/mach-tegra/iomap.h"
#include "dp_lt.h"

#ifdef CONFIG_DEBUG_FS
#include "dp_debug.h"
extern struct tegra_dp_test_settings default_dp_test_settings;
#endif

#define TEGRA_NVHDCP_MAX_DEVS 127
#define DP_POWER_ON_MAX_TRIES 3
#define DP_CLOCK_RECOVERY_MAX_TRIES 7
#define DP_CLOCK_RECOVERY_TOT_TRIES 15

/*
 * After hpd irq event, source must start
 * reading dpdc offset 200h-205h within
 * 100ms of rising edge hpd
 */
#define HPD_IRQ_EVENT_TIMEOUT_MS 70

/* the +10ms is the time for power rail going up from 10-90% or
   90%-10% on powerdown */
/* Time from power-rail is turned on and aux/12c-over-aux is available */
#define EDP_PWR_ON_TO_AUX_TIME_MS	    (200+10)
/* Time from power-rail is turned on and MainLink is available for LT */
#define EDP_PWR_ON_TO_ML_TIME_MS	    (200+10)
/* Time from turning off power to turn-it on again (does not include post
   poweron time) */
#define EDP_PWR_OFF_TO_ON_TIME_MS	    (500+10)

/*
 * Receiver capability fields extend from 0 - 0x11fh.
 * By default we read only more useful fields(offsets 0 - 0xb) as
 * required by CTS.
 */
#define DP_DPCD_SINK_CAP_SIZE (0xc)

struct tegra_dc_dp_data {
	/*
	 * The following "dpaux" and "sor" fields need to stay at the top of
	 * this struct. The placement of these fields needs to align with the
	 * tegra_hdmi struct definition in order to support dynamic SOR
	 * re-assignment with fakeDP. This is something that needs to be fixed,
	 * though.
	 */
	struct tegra_dc_dpaux_data *dpaux;
	struct tegra_dc_sor_data *sor;
	void *out_data;

	void *hda_handle;
	struct tegra_dc *dc;
	u32 irq;

	struct clk *parent_clk; /* pll_dp clock */

	u8 revision;

	struct tegra_dc_mode *mode;
	struct tegra_dc_dp_link_config link_cfg;
	struct tegra_dc_dp_link_config max_link_cfg;

	u8 typec_lane_count; /* # of lanes reported by extcon (Type-C) */

	/*
	 * In certain cases, the CCG4 USB-C controller might have already
	 * negotiated Alt Mode before the ucsi_ccg kernel driver is initialized.
	 * These cases can typically occur when the downstream USB-C partner is
	 * connected before ucsi_ccg init (e.g., seamless display). If Alt Mode
	 * has been established before ucsi_ccg init, ucsi_ccg has no way to
	 * determine what the current lane configuration is since the relevant
	 * VDMs (Vendor Defined Messages) have already been sent and received,
	 * and ucsi_ccg will not trigger any extcon notifications until the next
	 * USB-C configuration change is requested.
	 *
	 * In order to alleviate the above cases, we'll introduce two flags:
	 * - "typec_notified_once" indicates whether the DP driver has received
	 *   at least one extcon notification from ucsi_ccg so far. Once this
	 *   flag is set, it will remain set from that point on.
	 * - "typec_timed_out_once" indicates whether the DP driver has timed
	 *   out at least once while waiting for an extcon notification during
	 *   HPD_PLUG processing. Once this flag is set, it will remain set from
	 *   that point on.
	 *
	 * If "typec_notified_once" is FALSE and "typec_timed_out_once" is TRUE,
	 * the DP driver will skip all extcon waits during subsequent HPD_PLUG
	 * events until it receives at least one extcon notification. This logic
	 * is hacky, but is specifically meant to prevent the DP driver from
	 * continually timing out during each subsequent HPD_PLUG event before
	 * the next configuration change.
	 */
	bool typec_notified_once;
	bool typec_timed_out_once;

	struct tegra_dp_lt_data lt_data;

	bool enabled; /* Controller ready. LT not yet initiated. */
	bool suspended;

	int test_max_lanes; /* Test maximum cfg settings */
	int test_max_link_bw;

	u8 edid_src;
	struct tegra_hpd_data hpd_data;
#ifdef CONFIG_SWITCH
	struct switch_dev audio_switch;
#endif
	char *hpd_switch_name;
	char *audio_switch_name;
	struct delayed_work irq_evt_dwork;

	struct tegra_dphdcp *dphdcp;

	struct completion hpd_plug;

	struct tegra_dp_out_ops *out_ops;
	struct tegra_dp_out *pdata;

	struct tegra_prod *prod_list;

#ifdef CONFIG_DEBUG_FS
	struct dentry *debugdir;
#endif
	u8 sink_cap[DP_DPCD_SINK_CAP_SIZE];
	bool sink_cap_valid;
	u8 sink_cnt_cp_ready;

	u16 dpaux_i2c_dbg_addr;
	u32 dpaux_i2c_dbg_num_bytes;
	u16 dpaux_dpcd_dbg_addr;
	u32 dpaux_dpcd_dbg_num_bytes;

	bool early_enable;

#ifdef CONFIG_DEBUG_FS
	struct tegra_dp_test_settings test_settings;
};

extern const struct file_operations test_settings_fops;
#else
};
#endif

struct tegra_dp_out_ops {
	/* initialize output */
	int (*init)(struct tegra_dc_dp_data *);
	/* destroy output */
	void (*destroy)(struct tegra_dc_dp_data *);
	/* enable output */
	int (*enable)(struct tegra_dc_dp_data *);
	/* disable output */
	void (*disable)(struct tegra_dc_dp_data *);
	/* suspend output */
	void (*suspend)(struct tegra_dc_dp_data *);
	/* resume output */
	void (*resume)(struct tegra_dc_dp_data *);
	/* output postpoweron, called at the end of dc out_ops postpoweron.
	 * usually used for bridge devices.
	 */
	void (*postpoweron)(struct tegra_dc_dp_data *);
};

#if defined(CONFIG_TEGRA_EDP2LVDS_PS8625)
extern struct tegra_dp_out_ops tegra_edp2lvds_ops;
#else
#define tegra_edp2lvds_ops (*(struct tegra_dp_out_ops *)NULL)
#endif

enum {
	VSC_RGB = 0,
	VSC_YUV444 = 1,
	VSC_YUV422 = 2,
	VSC_YUV420 = 3,
	VSC_YONLY = 4,
	VSC_RAW = 5,
};

enum {
	VSC_VESA_RANGE = 0,
	VSC_CEA_RANGE = 1,
};

enum {
	VSC_6BPC = 0,
	VSC_8BPC = 1,
	VSC_10BPC = 2,
	VSC_12BPC = 3,
	VSC_16BPC = 4,
};

enum {
	VSC_RGB_SRGB = 0,
	VSC_RGB_ADOBERGB = 3,
};

enum {
	VSC_YUV_ITU_R_BT601 = 0,
	VSC_YUV_ITU_R_BT709 = 1,
	VSC_YUV_XVYCC601 = 2,
	VSC_YUV_XVYCC709 = 3,
	VSC_YUV_SYCC601 = 4,
	VSC_YUV_ADOBEYCC601 = 5,
};

enum {
	VSC_CONTENT_TYPE_DEFAULT = 0,
	VSC_CONTENT_TYPE_GRAPHICS = 1,
	VSC_CONTENT_TYPE_PHOTO = 2,
	VSC_CONTENT_TYPE_VIDEO = 3,
	VSC_CONTENT_TYPE_GAME = 4,
};

int tegra_dp_dpcd_write_field(struct tegra_dc_dp_data *dp, u32 cmd,
	u8 mask, u8 data);
void tegra_dc_dp_pre_disable_link(struct tegra_dc_dp_data *dp);
void tegra_dc_dp_disable_link(struct tegra_dc_dp_data *dp, bool powerdown);
void tegra_dc_dp_enable_link(struct tegra_dc_dp_data *dp);
void tegra_dp_update_link_config(struct tegra_dc_dp_data *dp);
int tegra_dc_dp_dpcd_read(struct tegra_dc_dp_data *dp, u32 cmd, u8 *data_ptr);
int tegra_dc_dp_dpcd_write(struct tegra_dc_dp_data *dp, u32 cmd, u8 data);
void tegra_dp_tpg(struct tegra_dc_dp_data *dp, u32 tp, u32 n_lanes);
bool tegra_dc_dp_calc_config(struct tegra_dc_dp_data *dp,
				const struct tegra_dc_mode *mode,
				struct tegra_dc_dp_link_config *cfg);
int tegra_dc_dp_read_ext_dpcd_caps(struct tegra_dc_dp_data *dp,
				struct tegra_dc_dp_ext_dpcd_caps *ext_caps);

static inline void *tegra_dp_get_outdata(struct tegra_dc_dp_data *dp)
{
	return dp->out_data;
}

static inline void tegra_dp_set_outdata(struct tegra_dc_dp_data *dp,
								void *data)
{
	dp->out_data = data;
}

/* DPCD definitions */
#define NV_DPCD_REV					(0x00000000)
#define NV_DPCD_REV_MAJOR_SHIFT				(4)
#define NV_DPCD_REV_MAJOR_MASK				(0xf << 4)
#define NV_DPCD_REV_MINOR_SHIFT				(0)
#define NV_DPCD_REV_MINOR_MASK				(0xf)
#define NV_DPCD_MAX_LINK_BANDWIDTH			(0x00000001)
#define NV_DPCD_MAX_LINK_BANDWIDTH_VAL_1_62_GBPS	(0x00000006)
#define NV_DPCD_MAX_LINK_BANDWIDTH_VAL_2_70_GBPS	(0x0000000a)
#define NV_DPCD_MAX_LINK_BANDWIDTH_VAL_5_40_GBPS	(0x00000014)
#define NV_DPCD_MAX_LINK_BANDWIDTH_VAL_8_10_GBPS	(0x0000001e)
#define NV_DPCD_MAX_LANE_COUNT				(0x00000002)
#define NV_DPCD_MAX_LANE_COUNT_MASK			(0x1f)
#define NV_DPCD_MAX_LANE_COUNT_LANE_1			(0x00000001)
#define NV_DPCD_MAX_LANE_COUNT_LANE_2			(0x00000002)
#define NV_DPCD_MAX_LANE_COUNT_LANE_4			(0x00000004)
#define NV_DPCD_MAX_LANE_COUNT_TPS3_SUPPORTED_YES	(0x00000001 << 6)
#define NV_DPCD_MAX_LANE_COUNT_ENHANCED_FRAMING_NO	(0x00000000 << 7)
#define NV_DPCD_MAX_LANE_COUNT_ENHANCED_FRAMING_YES	(0x00000001 << 7)
#define NV_DPCD_MAX_DOWNSPREAD				(0x00000003)
#define NV_DPCD_MAX_DOWNSPREAD_VAL_NONE			(0x00000000)
#define NV_DPCD_MAX_DOWNSPREAD_VAL_0_5_PCT		(0x00000001)
#define NV_DPCD_MAX_DOWNSPREAD_NO_AUX_HANDSHAKE_LT_F	(0x00000000 << 6)
#define NV_DPCD_MAX_DOWNSPREAD_NO_AUX_HANDSHAKE_LT_T	(0x00000001 << 6)
#define NV_DPCD_MAX_DOWNSPREAD_TPS4_SUPPORTED_YES	(0x00000001 << 7)
#define NV_DPCD_EDP_CONFIG_CAP				(0x0000000D)
#define NV_DPCD_EDP_CONFIG_CAP_ASC_RESET_NO		(0x00000000)
#define NV_DPCD_EDP_CONFIG_CAP_ASC_RESET_YES		(0x00000001)
#define NV_DPCD_EDP_CONFIG_CAP_FRAMING_CHANGE_NO	(0x00000000 << 1)
#define NV_DPCD_EDP_CONFIG_CAP_FRAMING_CHANGE_YES	(0x00000001 << 1)
#define NV_DPCD_EDP_CONFIG_CAP_DISPLAY_CONTROL_CAP_YES	(0x00000001 << 3)
#define NV_DPCD_TRAINING_AUX_RD_INTERVAL		(0x0000000E)
#define NV_DPCD_TRAINING_AUX_RD_INTERVAL_MASK		(0x3f)
#define NV_DPCD_EXT_RECEIVER_CAP_FIELD_PRESENT_SHIFT	(6)
#define NV_DPCD_LINK_BANDWIDTH_SET			(0x00000100)
#define NV_DPCD_LANE_COUNT_SET				(0x00000101)
#define NV_DPCD_LANE_COUNT_SET_MASK			(0x1f)
#define NV_DPCD_LANE_COUNT_SET_ENHANCEDFRAMING_F	(0x00000000 << 7)
#define NV_DPCD_LANE_COUNT_SET_ENHANCEDFRAMING_T	(0x00000001 << 7)
#define NV_DPCD_TRAINING_PATTERN_SET			(0x00000102)
#define NV_DPCD_TRAINING_PATTERN_SET_TPS_MASK		0x3
#define NV_DPCD_TRAINING_PATTERN_SET_TPS_NONE		(0x00000000)
#define NV_DPCD_TRAINING_PATTERN_SET_TPS_TP1		(0x00000001)
#define NV_DPCD_TRAINING_PATTERN_SET_TPS_TP2		(0x00000002)
#define NV_DPCD_TRAINING_PATTERN_SET_TPS_TP3		(0x00000003)
#define NV_DPCD_TRAINING_PATTERN_SET_TPS_TP4		(0x00000007)
#define NV_DPCD_TRAINING_PATTERN_SET_SC_DISABLED_F	(0x00000000 << 5)
#define NV_DPCD_TRAINING_PATTERN_SET_SC_DISABLED_T	(0x00000001 << 5)
#define NV_DPCD_TRAINING_LANE0_SET			(0x00000103)
#define NV_DPCD_TRAINING_LANE1_SET			(0x00000104)
#define NV_DPCD_TRAINING_LANE2_SET			(0x00000105)
#define NV_DPCD_TRAINING_LANE3_SET			(0x00000106)
#define NV_DPCD_TRAINING_LANEX_SET_DC_SHIFT		0
#define NV_DPCD_TRAINING_LANEX_SET_DC_MAX_REACHED_T	(0x00000001 << 2)
#define NV_DPCD_TRAINING_LANEX_SET_DC_MAX_REACHED_F	(0x00000000 << 2)
#define NV_DPCD_TRAINING_LANEX_SET_PE_SHIFT		3
#define NV_DPCD_TRAINING_LANEX_SET_PE_MAX_REACHED_T	(0x00000001 << 5)
#define NV_DPCD_TRAINING_LANEX_SET_PE_MAX_REACHED_F	(0x00000000 << 5)
#define NV_DPCD_DOWNSPREAD_CTRL				(0x00000107)
#define NV_DPCD_DOWNSPREAD_CTRL_SPREAD_AMP_NONE		(0x00000000 << 4)
#define NV_DPCD_DOWNSPREAD_CTRL_SPREAD_AMP_LT_0_5	(0x00000001 << 4)
#define NV_DPCD_MAIN_LINK_CHANNEL_CODING_SET		(0x00000108)
#define NV_DPCD_MAIN_LINK_CHANNEL_CODING_SET_ANSI_8B10B	1
#define NV_DPCD_EDP_CONFIG_SET				(0x0000010A)
#define NV_DPCD_EDP_CONFIG_SET_ASC_RESET_DISABLE	(0x00000000)
#define NV_DPCD_EDP_CONFIG_SET_ASC_RESET_ENABLE		(0x00000001)
#define NV_DPCD_EDP_CONFIG_SET_FRAMING_CHANGE_DISABLE	(0x00000000 << 1)
#define NV_DPCD_EDP_CONFIG_SET_FRAMING_CHANGE_ENABLE	(0x00000001 << 1)
#define NV_DPCD_TRAINING_LANE0_1_SET2			(0x0000010F)
#define NV_DPCD_TRAINING_LANE2_3_SET2			(0x00000110)
#define NV_DPCD_LANEX_SET2_PC2_SHIFT			0
#define NV_DPCD_LANEX_SET2_PC2_MAX_REACHED_T		(0x00000001 << 2)
#define NV_DPCD_LANEX_SET2_PC2_MAX_REACHED_F		(0x00000000 << 2)
#define NV_DPCD_LANEXPLUS1_SET2_PC2_SHIFT		4
#define NV_DPCD_LANEXPLUS1_SET2_PC2_MAX_REACHED_T	(0x00000001 << 6)
#define NV_DPCD_LANEXPLUS1_SET2_PC2_MAX_REACHED_F	(0x00000000 << 6)
#define NV_DPCD_SINK_COUNT				(0x00000200)
#define NV_DPCD_DEVICE_SERVICE_IRQ_VECTOR		(0x00000201)
#define NV_DPCD_DEVICE_SERVICE_IRQ_VECTOR_AUTO_TEST_NO	(0x00000000 << 1)
#define NV_DPCD_DEVICE_SERVICE_IRQ_VECTOR_AUTO_TEST_YES	(0x00000001 << 1)
#define NV_DPCD_DEVICE_SERVICE_IRQ_VECTOR_CP_NO		(0x00000000 << 2)
#define NV_DPCD_DEVICE_SERVICE_IRQ_VECTOR_CP_YES	(0x00000001 << 2)
#define NV_DPCD_LANE0_1_STATUS				(0x00000202)
#define NV_DPCD_LANE2_3_STATUS				(0x00000203)
#define NV_DPCD_STATUS_LANEX_CR_DONE_SHIFT		0
#define NV_DPCD_STATUS_LANEX_CR_DONE_NO			(0x00000000)
#define NV_DPCD_STATUS_LANEX_CR_DONE_YES		(0x00000001)
#define NV_DPCD_STATUS_LANEX_CHN_EQ_DONE_SHIFT		1
#define NV_DPCD_STATUS_LANEX_CHN_EQ_DONE_NO		(0x00000000 << 1)
#define NV_DPCD_STATUS_LANEX_CHN_EQ_DONE_YES		(0x00000001 << 1)
#define NV_DPCD_STATUS_LANEX_SYMBOL_LOCKED_SHFIT	2
#define NV_DPCD_STATUS_LANEX_SYMBOL_LOCKED_NO		(0x00000000 << 2)
#define NV_DPCD_STATUS_LANEX_SYMBOL_LOCKED_YES		(0x00000001 << 2)
#define NV_DPCD_STATUS_LANEXPLUS1_CR_DONE_SHIFT		4
#define NV_DPCD_STATUS_LANEXPLUS1_CR_DONE_NO		(0x00000000 << 4)
#define NV_DPCD_STATUS_LANEXPLUS1_CR_DONE_YES		(0x00000001 << 4)
#define NV_DPCD_STATUS_LANEXPLUS1_CHN_EQ_DONE_SHIFT	5
#define NV_DPCD_STATUS_LANEXPLUS1_CHN_EQ_DONE_NO	(0x00000000 << 5)
#define NV_DPCD_STATUS_LANEXPLUS1_CHN_EQ_DONE_YES	(0x00000001 << 5)
#define NV_DPCD_STATUS_LANEXPLUS1_SYMBOL_LOCKED_SHIFT	6
#define NV_DPCD_STATUS_LANEXPLUS1_SYMBOL_LOCKED_NO	(0x00000000 << 6)
#define NV_DPCD_STATUS_LANEXPLUS1_SYMBOL_LOCKED_YES	(0x00000001 << 6)
#define NV_DPCD_LANE_ALIGN_STATUS_UPDATED		(0x00000204)
#define NV_DPCD_LANE_ALIGN_STATUS_INTERLANE_ALIGN_DONE_NO	(0x00000000)
#define NV_DPCD_LANE_ALIGN_STATUS_INTERLANE_ALIGN_DONE_YES	(0x00000001)
#define NV_DPCD_LANE0_1_ADJUST_REQ			(0x00000206)
#define NV_DPCD_LANE2_3_ADJUST_REQ			(0x00000207)
#define NV_DPCD_ADJUST_REQ_LANEX_DC_SHIFT		0
#define NV_DPCD_ADJUST_REQ_LANEX_DC_MASK		0x3
#define NV_DPCD_ADJUST_REQ_LANEX_PE_SHIFT		2
#define NV_DPCD_ADJUST_REQ_LANEX_PE_MASK		(0x3 << 2)
#define NV_DPCD_ADJUST_REQ_LANEXPLUS1_DC_SHIFT		4
#define NV_DPCD_ADJUST_REQ_LANEXPLUS1_DC_MASK		(0x3 << 4)
#define NV_DPCD_ADJUST_REQ_LANEXPLUS1_PE_SHIFT		6
#define NV_DPCD_ADJUST_REQ_LANEXPLUS1_PE_MASK		(0x3 << 6)
#define NV_DPCD_ADJUST_REQ_POST_CURSOR2			(0x0000020C)
#define NV_DPCD_ADJUST_REQ_POST_CURSOR2_LANE_MASK	0x3
#define NV_DPCD_ADJUST_REQ_POST_CURSOR2_LANE_SHIFT(i)	(i*2)
#define NV_DPCD_TEST_REQUEST				(0x00000218)
#define NV_DPCD_TEST_REQUEST_TEST_LT			(1 << 0)
#define NV_DPCD_TEST_REQUEST_TEST_PATTERN		(1 << 1)
#define NV_DPCD_TEST_REQUEST_TEST_EDID_READ		(1 << 2)
#define NV_DPCD_TEST_RESPONSE				(0x00000260)
#define NV_DPCD_TEST_RESPONSE_ACK			(1 << 0)
#define NV_DPCD_TEST_RESPONSE_NACK			(1 << 1)
#define NV_DPCD_TEST_EDID_CHECKSUM_WR		(1 << 2)
#define NV_DPCD_TEST_EDID_CHECKSUM			(0x00000261)
#define NV_DPCD_SOURCE_IEEE_OUI				(0x00000300)
#define NV_IEEE_OUI					(0x00044b)
#define NV_DPCD_SINK_IEEE_OUI				(0x00000400)
#define NV_DPCD_BRANCH_IEEE_OUI				(0x00000500)
#define NV_DPCD_SET_POWER				(0x00000600)
#define NV_DPCD_SET_POWER_VAL_RESERVED			(0x00000000)
#define NV_DPCD_SET_POWER_VAL_D0_NORMAL			(0x00000001)
#define NV_DPCD_SET_POWER_VAL_D3_PWRDWN			(0x00000002)
#define NV_DPCD_REV_EXT_CAP				(0x00002200)
#define NV_DPCD_MAX_LINK_BANDWIDTH_EXT_CAP		(0x00002201)
#define NV_DPCD_FEATURE_ENUM_LIST			(0x00002210)
#define NV_DPCD_FEATURE_ENUM_LIST_VSC_EXT_COLORIMETRY	(1 << 3)
#define NV_DPCD_HDCP_BKSV_OFFSET			(0x00068000)
#define NV_DPCD_HDCP_RPRIME_OFFSET			(0x00068005)
#define NV_DPCD_HDCP_AKSV_OFFSET			(0x00068007)
#define NV_DPCD_HDCP_AN_OFFSET				(0x0006800C)
#define NV_DPCD_HDCP_VPRIME_OFFSET			(0x00068014)
#define NV_DPCD_HDCP_BCAPS_OFFSET			(0x00068028)
#define NV_DPCD_HDCP_BSTATUS_OFFSET			(0x00068029)
#define NV_DPCD_HDCP_BINFO_OFFSET			(0x0006802A)
#define NV_DPCD_HDCP_KSV_FIFO_OFFSET			(0x0006802C)
#define NV_DPCD_HDCP_AINFO_OFFSET			(0x0006803B)
#define NV_DPCP_HDCP_SHA_H0_OFFSET                      (0x00068014)
#define NV_DPCP_HDCP_SHA_H1_OFFSET                      (0x00068018)
#define NV_DPCP_HDCP_SHA_H2_OFFSET                      (0x0006801C)
#define NV_DPCP_HDCP_SHA_H3_OFFSET                      (0x00068020)
#define NV_DPCP_HDCP_SHA_H4_OFFSET                      (0x00068024)
/* DP 2.2 specific registers */
#define NV_DPCD_HDCP_RTX_OFFSET                         (0x00069000)
#define NV_DPCD_HDCP_TXCAPS_OFFSET                      (0x00069008)
#define NV_DPCD_HDCP_CERT_RX_OFFSET                     (0x0006900B)
#define NV_DPCD_HDCP_CERT_RRX_OFFSET                    (0x00069215)
#define NV_DPCD_HDCP_CERT_RXCAPS_OFFSET                 (0x0006921D)
#define NV_DPCD_HDCP_EKM_NOSTORED                       (0x69220)
#define NV_DPCD_HDCP_EKM_STORED                         (0x692A0)
#define NV_DPCD_HDCP_M                                  (0x692B0)
#define NV_DPCD_HDCP_HPRIME                             (0x000692C0)
#define NV_DPCD_HDCP_EKM_PAIRING                        (0x0000692E0)
#define NV_DPCD_HDCP_RN                                 (0x000692F0)
#define NV_DPCD_HDCP_LPRIME                             (0x000692F8)
#define NV_DPCD_HDCP_EKS                                (0x00069318)
#define NV_DPCD_HDCP_RIV                                (0x00069328)
#define NV_DPCD_HDCP_RXINFO                             (0x00069330)
#define NV_DPCD_HDCP_SEQNUM_V                           (0x00069332)
#define NV_DPCD_HDCP_VPRIME                             (0x00069335)
#define NV_DPCD_HDCP_RX_ID_LIST                         (0x00069345)
#define NV_DPCD_HDCP_V                                  (0x000693E0)
#define NV_DPCD_HDCP_SEQ_NUM_M                          (0x000693F0)
#define NV_DPCD_HDCP_K                                  (0x000693F3)
#define NV_DPCD_HDCP_STRMID_TYPE                        (0x000693F5)
#define NV_DPCD_HDCP_MPRIME                             (0x00069473)
#define NV_DPCD_HDCP_RXSTATUS                           (0x00069493)
#define NV_DPCD_HDCP_RSVD                               (0x00069494)
#define NV_DPCD_HDCP_DBG                                (0x00069518)

void tegra_dp_set_max_link_bw(struct tegra_dc_sor_data *sor,
			      struct tegra_dc_dp_link_config *cfg);
int tegra_dc_dp_get_max_link_bw(struct tegra_dc_dp_data *dp);
int tegra_dc_dp_get_max_lane_count(struct tegra_dc_dp_data *dp, u8 *dpcd_data);
int tegra_dp_set_enhanced_framing(struct tegra_dc_dp_data *dp, bool enable);
#endif
