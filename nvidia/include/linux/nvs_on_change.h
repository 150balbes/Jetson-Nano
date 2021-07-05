/* Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */


#ifndef _NVS_ON_CHANGE_H_
#define _NVS_ON_CHANGE_H_

#include <linux/of.h>
#include <linux/nvs.h>

#define RET_POLL_NEXT			(-1)
#define RET_NO_CHANGE			(0)
#define RET_HW_UPDATE			(1)
#define OC_CALIBRATION_DISABLE		(-1)
#define OC_CALIBRATION_DONE		(0)
#define OC_CALIBRATION_ENABLE		(1)
#define OC_THRESHOLDS_DISABLE		(-1)
#define OC_THRESHOLDS_RELATIVE		(0)
#define OC_THRESHOLDS_ABSOLUTE		(1)

/*
 * struct nvs_on_change - the common structure between the
 * on_change hardware DRIVER and the NVS common module for
 * Android defined on_change sensors.
 * @timestamp: DRIVER writes the timestamp.
 * @timestamp_report: NVS writes the timestamp reported.
 * @on_change: NVS writes the on_change value reported.
 * @hw: DRIVER writes the value read from HW.
 * @hw_min: DRIVER sets the minimum HW value possible.
 * @hw_max: DRIVER sets the maximum HW value possible.
 * @hw_thresh_lo: NVS sets the low threshold value for DRIVER
 *                to write to HW.
 * @hw_thresh_hi: NVS sets the high threshold value for DRIVER
 *                to write to HW.
 * @hw_limit_lo: NVS determines if the HW value will no longer
 *               trigger below the low threshold.
 * @hw_limit_hi: NVS determines if the HW value will no longer
 *               trigger above the high threshold.
 * @thresh_valid_lo: NVS determines if cfg.thresh_lo valid.
 * @thresh_valid_hi: NVS determines if cfg.thresh_hi valid.
 * Note: If thresh_valid_x == true then DRIVER should write to HW
 *       the corresponding hw_thresh_x value.
 * @reverse_range_en: DRIVER sets this if the on_change range is
 *                    reversed.
 * Note: If an on_change HW value increases when it represents a
 *       unit value that should be decreasing and vice versa, NVS
 *       can reverse this by subtracting the HW value from the
 *       maximum possible HW value (hw_max).
 * @scale_float_en: DRIVER can set this to true if using the
 *                  scale to convert to float.
 * Note: scale_float_en allows floating point to be calculated
 *       in the kernel by shifting up to integer the floating
 *       point significant amount. This allows real-time resolution
 *       changes without the NVS HAL having to synchronize to the
 *       actual resolution per data value.
 *       The scale.fval must be a 10 base value, e.g. 0.1, 0.01,
 *       ... 0.000001, etc. as the significant amount.  The NVS
 *       HAL will then convert the integer float-data to a float
 *       value by multiplying it with scale.
 * @binary_report_en: NVS determines binary reporting is enabled
 *                    if range and resolution == 1.
 * @threshold: NVS determines threshold mode.
 * Note: NVS determines thresholds are disabled when both
 *       oc->cfg->thresh_lo and oc->cfg->thresh_hi == 0.
 *       NVS uses device tree to determine:
 *       OC_THRESHOLDS_RELATIVE
 *       OC_THRESHOLDS_ABSOLUTE
 *       The default is OC_THRESHOLDS_RELATIVE if not device tree.
 * @calibration: NVS determines if calibration is enabled.
 * @poll_period_ms: NVS writes the polling period needed if polling.
 * @period_us: DRIVER writes the requested sampling period.
 * @report: NVS writes the report count.
 * @cfg: DRIVER writes the sensor_cfg structure pointer.
 * @nvs_data: DRIVER writes the private pointer for handler.
 * @handler: DRIVER writes the handler pointer.
 */
struct nvs_on_change {
	s64 timestamp;			/* sample timestamp */
	s64 timestamp_report;		/* last reported timestamp */
	s32 on_change;			/* on_change value */
	s32 hw;				/* HW on_change value */
	s32 hw_min;			/* minimum HW value */
	s32 hw_max;			/* maximum HW value */
	s32 hw_thresh_lo;		/* HW low threshold value */
	s32 hw_thresh_hi;		/* HW high threshold value */
	bool hw_limit_lo;		/* hw <= hw_thresh_lo */
	bool hw_limit_hi;		/* hw > hw_thresh_hi or hw = hw_mask */
	bool thresh_valid_lo;		/* valid cfg.thresh_lo */
	bool thresh_valid_hi;		/* valid cfg.thresh_hi */
	bool reverse_range_en;		/* if on_change range reversed */
	bool scale_float_en;		/* enable internal float calculation */
	bool binary_report_en;		/* report either 0 or 1 */
	int threshold;			/* threshold mode */
	int calibration;		/* calibration mode */
	unsigned int poll_period_ms;	/* HW polling period (ms) */
	unsigned int period_us;		/* OS requested sampling period */
	unsigned int report_n;		/* report count */
	struct sensor_cfg *cfg;		/* pointer to sensor configuration */
	void *nvs_st;			/* NVS state data for NVS handler */
	int (*handler)(void *handle, void *buffer, s64 ts);
};

int nvs_on_change_enable(struct nvs_on_change *oc);
int nvs_on_change_read(struct nvs_on_change *oc);
int nvs_on_change_of_dt(struct nvs_on_change *oc, const struct device_node *dn,
			const char *dev_name);
int nvs_on_change_threshold_lo(struct nvs_on_change *oc, int lo);
int nvs_on_change_threshold_hi(struct nvs_on_change *oc, int hi);
ssize_t nvs_on_change_dbg_dt(struct nvs_on_change *oc, char *buf, size_t size,
			     const char *dev_name);
ssize_t nvs_on_change_dbg(struct nvs_on_change *oc, char *buf, size_t size);

#endif /* _NVS_ON_CHANGE_H_ */

