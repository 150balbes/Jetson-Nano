/*
 * drivers/video/tegra/dc/dc_common.h
 *
 * Copyright (c) 2017-2019, NVIDIA CORPORATION, All rights reserved.
 * Author: Arun Swain <arswain@nvidia.com>
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

#ifndef __DRIVERS_VIDEO_TEGRA_DC_DC_COMMON_H__
#define __DRIVERS_VIDEO_TEGRA_DC_DC_COMMON_H__

#include <uapi/video/tegra_dc_ext.h>

/* Flags for tegra_dc_common */

/**
 * @DC_COMMON_JOB_SUBMITTED - Used by dc_common to keep track
 * of a submitted job. For frame/flip locks, in the presence
 * of multiple heads, the host1x job needs to be submitted
 * only once. This flag is use track the same.
 */
#define DC_COMMON_JOB_SUBMITTED		0x00

/**
 * @DC_COMMON_ERROR_CHECK_REQ- Used by dc_common to keep track
 * of checking errors. For flip lock, mulitple frame_end
 * interrupts are received but the check for error is only when
 * this flag is set.
 */
#define DC_COMMON_ERROR_CHECK_REQ	0x01

/**
 * @LOCK_TYPE_FRAME - Used by tegradc heads to request for
 * FRAME LOCK.
 */
#define LOCK_TYPE_FRAME	(1<<0)
/**
 * @LOCK_TYPE_FLIP - Used by tegradc heads to request for
 * FLIP LOCK.
 */
#define LOCK_TYPE_FLIP	(1<<1)
/**
 * struct bit_mapped_data - Used by dc_common to store head-specific
 * data in a bit-mapped format.
 */
struct bit_mapped_data {
	/**
	 * @gen_act_read_result: Used to keep track of the result of
	 * general_act_request promotion at frame_end for multiple
	 * heads.
	 */
	ulong gen_act_read_result;
	/**
	 * @fr_lck_req_rcvd
	 */
	ulong fr_lck_req_rcvd;
	/**
	 * @fl_lck_req_rcvd: Used to keep track of received request for
	 * flip lock from mulitple heads.
	 */
	ulong fl_lck_req_rcvd;
	/**
	 * @fl_lck_req_completed: Used to keep track of completed request for
	 * flip lock from mulitple heads.
	 */
	ulong fl_lck_req_completed;
};

/**
 * struct tegra_dc_common- The main struct used by dc_common platform
 * driver. Stored as the private_data in dc_common platform device.
 */
struct tegra_dc_common {
	/**
	 * @flags: Used by dc_comon to track of the various results and
	 * requests received for multiple heads. Please see above for
	 * description of each of them.
	 */
	ulong	flags;
	/**
	 * @base: Used by dc_common to store the base address of display
	 * controller/Nvdisplay. Read from device tree.
	 */
	void __iomem	*base;
	/**
	 * @cpuvaddr: Used by dc_common to store the virtual address of
	 * memory chunk allocated for dma. This is used to write cmds
	 * later on to be accessed by host1x h/w via dma.
	 */
	u32	*cpuvaddr;
	/**
	 * @syncpt_id: Used for host-managed syncpt for notifying dc_common
	 * after job submission. Immediate syncpt incr is used for this
	 * purpose.
	 */
	int	syncpt_id;
	/**
	 * @valid_heads: A bit-mapped variable used to store all the
	 * heads that are being used for frame-lock. Default value is
	 * being taken from device tree. Each bit set corresponds to
	 * the head_id being used for frame-lock.
	 */
	ulong	valid_heads;
	/**
	 * @lock: Used by dc_common for operating on shared variables
	 * amongst multiple heads to avoid multiple heads running
	 * through the critical section simultaneously.
	 */
	struct mutex	lock;
	/**
	 * @heads_locked: Used by dc_common to make sure that the
	 * participating heads are frame-locked before taking in
	 * requests for flip-lock. Required as a safety back-up for
	 * cases where we can't guarantee the probe order(tegradc
	 * and dc_common).
	 */
	bool	heads_locked;
	/**
	 * @dev: Points to device struct in platform_device.
	 */
	struct device	*dev;
	/**
	 * @dma_handle: Stores the dma-address to be used by nvhost for
	 * accessing the commands/opcodes programmed by dc_common.
	 */
	dma_addr_t	dma_handle;
	/**
	 * @job: Pointer to the job-handle allocated by nvhost which is
	 * used to submit jobs from dc_common.
	 */
	struct nvhost_job	*job;
	/**
	 * @pdev: Points the platform_device for dc_common.
	 */
	struct platform_device	*pdev;
	/**
	 * @channel: Points to the channel mapped by nvhost for job
	 * submission.
	 */
	struct nvhost_channel	*channel;
	/**
	 * @head_data: Stores data related to multiple heads. Please
	 * see above for detailed description.
	 */
	struct bit_mapped_data	head_data;
	/**
	 * @upd_val: Stores value to be programmed to DC_CMD_STATE_CONTROL
	 * register during job submission for flip lock. Mulitple heads can
	 * have different values to be written to the regsiter. Could have
	 * reused dsp_cmd_reg_val here but there can be flip requests while
	 * the frame lock job submisssion is pending.
	 */
	ulong	*upd_val;
	/**
	 * @dsp_cmd_reg_val: Stores value to be programmed for frame lock
	 * to DC_CMD_STATE_CONTROL register during job submission.
	 * Mulitple heads can have different values to be written to the
	 * regsiter.
	 */
	ulong	*dsp_cmd_reg_val;
	/**
	 * @prgrm_reg_reqs_wq: wait queue for all the requests received
	 * to program DC_CMD_STATE_CONTROL register.
	 */
	wait_queue_head_t	prgrm_reg_reqs_wq;
	/**
	 * @imp_table: Holds platform data of IHUB settings for all
	 * windows and heads.
	 */
	struct nvdisp_imp_table *imp_table;

#ifdef CONFIG_DEBUG_FS
	struct dentry	*debugdir;
#endif
};

/**
 * struct tegra_dc_common_platform_data- This is used to store dc_common
 * specific data from the device tree.
 * @valid_heads: Same usage as valid_heads in tegra_dc_common.
 * @imp_table: IMP platform data.
 * Please see above for details.
 */
struct tegra_dc_common_platform_data {
	u8 valid_heads;
	struct nvdisp_imp_table *imp_table;
};

/*
 * Functions prototype here. Descriptions could be found in dc_common.c
 * file.
 */
struct tegra_dc_common_platform_data
	*of_dc_common_parse_platform_data(struct platform_device *pdev);

int tegra_dc_common_sync_flips(struct tegra_dc *dc, ulong val, ulong reg);

int tegra_dc_common_sync_frames(struct tegra_dc *dc, ulong dsp_cmd_reg,
		ulong dsp_cmd_state_access_reg, ulong dsp_cmd_reg_val);

int tegra_dc_common_handle_flip_lock_error(struct tegra_dc *dc);

int tegra_dc_common_get_frm_lock_params(
		struct tegra_dc_ext_control_frm_lck_params *params);

int tegra_dc_common_set_frm_lock_params(
		struct tegra_dc_ext_control_frm_lck_params *params);

struct nvdisp_imp_table
	*tegra_dc_common_get_imp_table(void);

bool tegra_dc_common_probe_status(void);
#endif
