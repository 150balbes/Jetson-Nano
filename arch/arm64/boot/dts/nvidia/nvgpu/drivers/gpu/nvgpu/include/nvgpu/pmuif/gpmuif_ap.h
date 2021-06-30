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
#ifndef NVGPU_PMUIF_GPMUIF_AP_H
#define NVGPU_PMUIF_GPMUIF_AP_H

/* PMU Command/Message Interfaces for Adaptive Power */
/* Macro to get Histogram index */
#define PMU_AP_HISTOGRAM(idx)		(idx)
#define PMU_AP_HISTOGRAM_CONT		(4)

/* Total number of histogram bins */
#define PMU_AP_CFG_HISTOGRAM_BIN_N	(16)

/* Mapping between Idle counters and histograms */
#define PMU_AP_IDLE_MASK_HIST_IDX_0		(2)
#define PMU_AP_IDLE_MASK_HIST_IDX_1		(3)
#define PMU_AP_IDLE_MASK_HIST_IDX_2		(5)
#define PMU_AP_IDLE_MASK_HIST_IDX_3		(6)


/* Mapping between AP_CTRLs and Histograms */
#define PMU_AP_HISTOGRAM_IDX_GRAPHICS	(PMU_AP_HISTOGRAM(1))

/* Mapping between AP_CTRLs and Idle counters */
#define PMU_AP_IDLE_MASK_GRAPHICS	(PMU_AP_IDLE_MASK_HIST_IDX_1)

/* Adaptive Power Controls (AP_CTRL) */
enum {
	PMU_AP_CTRL_ID_GRAPHICS = 0x0,
	PMU_AP_CTRL_ID_MAX,
};

/* AP_CTRL Statistics */
struct pmu_ap_ctrl_stat {
	/*
	 * Represents whether AP is active or not
	 */
	u8	b_active;

	/* Idle filter represented by histogram bin index */
	u8	idle_filter_x;
	u8	rsvd[2];

	/* Total predicted power saving cycles. */
	s32	power_saving_h_cycles;

	/* Counts how many times AP gave us -ve power benefits. */
	u32	bad_decision_count;

	/*
	 * Number of times ap structure needs to skip AP iterations
	 * KICK_CTRL from kernel updates this parameter.
	 */
	u32	skip_count;
	u8	bin[PMU_AP_CFG_HISTOGRAM_BIN_N];
};

/* Parameters initialized by INITn APCTRL command */
struct pmu_ap_ctrl_init_params {
	/* Minimum idle filter value in Us */
	u32	min_idle_filter_us;

	/*
	 * Minimum Targeted Saving in Us. AP will update idle thresholds only
	 * if power saving achieved by updating idle thresholds is greater than
	 * Minimum targeted saving.
	 */
	u32	min_target_saving_us;

	/* Minimum targeted residency of power feature in Us */
	u32	power_break_even_us;

	/*
	 * Maximum number of allowed power feature cycles per sample.
	 *
	 * We are allowing at max "pgPerSampleMax" cycles in one iteration of AP
	 * AKA pgPerSampleMax in original algorithm.
	 */
	u32	cycles_per_sample_max;
};

/* AP Commands/Message structures */

/*
 * Structure for Generic AP Commands
 */
struct pmu_ap_cmd_common {
	u8	cmd_type;
	u16	cmd_id;
};

/*
 * Structure for INIT AP command
 */
struct pmu_ap_cmd_init {
	u8	cmd_type;
	u16	cmd_id;
	u8	rsvd;
	u32	pg_sampling_period_us;
};

/*
 * Structure for Enable/Disable ApCtrl Commands
 */
struct pmu_ap_cmd_enable_ctrl {
	u8	cmd_type;
	u16	cmd_id;

	u8	ctrl_id;
};

struct pmu_ap_cmd_disable_ctrl {
	u8	cmd_type;
	u16	cmd_id;

	u8	ctrl_id;
};

/*
 * Structure for INIT command
 */
struct pmu_ap_cmd_init_ctrl {
	u8				cmd_type;
	u16				cmd_id;
	u8				ctrl_id;
	struct pmu_ap_ctrl_init_params	params;
};

struct pmu_ap_cmd_init_and_enable_ctrl {
	u8				cmd_type;
	u16				cmd_id;
	u8				ctrl_id;
	struct pmu_ap_ctrl_init_params	params;
};

/*
 * Structure for KICK_CTRL command
 */
struct pmu_ap_cmd_kick_ctrl {
	u8	cmd_type;
	u16	cmd_id;
	u8	ctrl_id;

	u32	skip_count;
};

/*
 * Structure for PARAM command
 */
struct pmu_ap_cmd_param {
	u8	cmd_type;
	u16	cmd_id;
	u8	ctrl_id;

	u32	data;
};

/*
 * Defines for AP commands
 */
enum {
	PMU_AP_CMD_ID_INIT = 0x0,
	PMU_AP_CMD_ID_INIT_AND_ENABLE_CTRL,
	PMU_AP_CMD_ID_ENABLE_CTRL,
	PMU_AP_CMD_ID_DISABLE_CTRL,
	PMU_AP_CMD_ID_KICK_CTRL,
};

/*
 * AP Command
 */
union pmu_ap_cmd {
	u8					cmd_type;
	struct pmu_ap_cmd_common		cmn;
	struct pmu_ap_cmd_init			init;
	struct pmu_ap_cmd_init_and_enable_ctrl	init_and_enable_ctrl;
	struct pmu_ap_cmd_enable_ctrl		enable_ctrl;
	struct pmu_ap_cmd_disable_ctrl		disable_ctrl;
	struct pmu_ap_cmd_kick_ctrl		kick_ctrl;
};

/*
 * Structure for generic AP Message
 */
struct pmu_ap_msg_common {
	u8	msg_type;
	u16	msg_id;
};

/*
 * Structure for INIT_ACK Message
 */
struct pmu_ap_msg_init_ack {
	u8	msg_type;
	u16	msg_id;
	u8	ctrl_id;
	u32	stats_dmem_offset;
};

/*
 * Defines for AP messages
 */
enum {
	PMU_AP_MSG_ID_INIT_ACK = 0x0,
};

/*
 * AP Message
 */
union pmu_ap_msg {
	u8				msg_type;
	struct pmu_ap_msg_common	cmn;
	struct pmu_ap_msg_init_ack	init_ack;
};

/*
 * Adaptive Power Controller
 */
struct ap_ctrl {
	u32			stats_dmem_offset;
	u32			disable_reason_mask;
	struct pmu_ap_ctrl_stat	stat_cache;
	u8			b_ready;
};

/*
 * Adaptive Power structure
 *
 * ap structure provides generic infrastructure to make any power feature
 * adaptive.
 */
struct pmu_ap {
	u32			supported_mask;
	struct ap_ctrl		ap_ctrl[PMU_AP_CTRL_ID_MAX];
};

#endif /* NVGPU_PMUIF_GPMUIF_AP_H*/
