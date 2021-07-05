/*
 * drivers/video/tegra/dc/nvdisp/nvdisp_crc.c
 *
 * Copyright (c) 2017-2019, NVIDIA CORPORATION, All rights reserved.
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

#include <linux/errno.h>
#include <linux/delay.h>

#include <uapi/video/tegra_dc_ext.h>
#include "dc_priv.h"
#include "dc_priv_defs.h"
#include "hw_nvdisp_nvdisp.h"
#include "nvdisp_priv.h"

static struct tegra_nvdisp_rg_region_regs {
	u32 point;
	u32 size;
	u32 golden_crc;
} regs[TEGRA_DC_EXT_MAX_REGIONS];

/* Masks for different functional bits across control/status registers */
static struct tegra_nvdisp_rg_region_masks {
	u32 pending;
	u32 error;
	u32 enable;
} masks[TEGRA_DC_EXT_MAX_REGIONS];

/* Unlike G++, GCC does not allow using static inlines as member initializers
 * of static variables.
 */
void tegra_nvdisp_crc_region_init(void)
{
	regs[0].point = nvdisp_rg_region_0_point_r();
	regs[0].size = nvdisp_rg_region_0_size_r();
	regs[0].golden_crc = nvdisp_rg_region_0_golden_crc_r();

	regs[1].point = nvdisp_rg_region_1_point_r();
	regs[1].size = nvdisp_rg_region_1_size_r();
	regs[1].golden_crc = nvdisp_rg_region_1_golden_crc_r();

	regs[2].point = nvdisp_rg_region_2_point_r();
	regs[2].size = nvdisp_rg_region_2_size_r();
	regs[2].golden_crc = nvdisp_rg_region_2_golden_crc_r();

	regs[3].point = nvdisp_rg_region_3_point_r();
	regs[3].size = nvdisp_rg_region_3_size_r();
	regs[3].golden_crc = nvdisp_rg_region_3_golden_crc_r();

	regs[4].point = nvdisp_rg_region_4_point_r();
	regs[4].size = nvdisp_rg_region_4_size_r();
	regs[4].golden_crc = nvdisp_rg_region_4_golden_crc_r();

	regs[5].point = nvdisp_rg_region_5_point_r();
	regs[5].size = nvdisp_rg_region_5_size_r();
	regs[5].golden_crc = nvdisp_rg_region_5_golden_crc_r();

	regs[6].point = nvdisp_rg_region_6_point_r();
	regs[6].size = nvdisp_rg_region_6_size_r();
	regs[6].golden_crc = nvdisp_rg_region_6_golden_crc_r();

	regs[7].point = nvdisp_rg_region_7_point_r();
	regs[7].size = nvdisp_rg_region_7_size_r();
	regs[7].golden_crc = nvdisp_rg_region_7_golden_crc_r();

	regs[8].point = nvdisp_rg_region_8_point_r();
	regs[8].size = nvdisp_rg_region_8_size_r();
	regs[8].golden_crc = nvdisp_rg_region_8_golden_crc_r();

	masks[0].pending = nvdisp_rg_region_crc_region0_pending_yes_f();
	masks[0].error = nvdisp_rg_region_crc_region0_error_yes_f();
	masks[0].enable = nvdisp_rg_region_crc_control_region0_enable_f();

	masks[1].pending = nvdisp_rg_region_crc_region1_pending_yes_f();
	masks[1].error = nvdisp_rg_region_crc_region1_error_yes_f();
	masks[1].enable = nvdisp_rg_region_crc_control_region1_enable_f();

	masks[2].pending = nvdisp_rg_region_crc_region2_pending_yes_f();
	masks[2].error = nvdisp_rg_region_crc_region2_error_yes_f();
	masks[2].enable = nvdisp_rg_region_crc_control_region2_enable_f();

	masks[3].pending = nvdisp_rg_region_crc_region3_pending_yes_f();
	masks[3].error = nvdisp_rg_region_crc_region3_error_yes_f();
	masks[3].enable = nvdisp_rg_region_crc_control_region3_enable_f();

	masks[4].pending = nvdisp_rg_region_crc_region4_pending_yes_f();
	masks[4].error = nvdisp_rg_region_crc_region4_error_yes_f();
	masks[4].enable = nvdisp_rg_region_crc_control_region4_enable_f();

	masks[5].pending = nvdisp_rg_region_crc_region5_pending_yes_f();
	masks[5].error = nvdisp_rg_region_crc_region5_error_yes_f();
	masks[5].enable = nvdisp_rg_region_crc_control_region5_enable_f();

	masks[6].pending = nvdisp_rg_region_crc_region6_pending_yes_f();
	masks[6].error = nvdisp_rg_region_crc_region6_error_yes_f();
	masks[6].enable = nvdisp_rg_region_crc_control_region6_enable_f();

	masks[7].pending = nvdisp_rg_region_crc_region7_pending_yes_f();
	masks[7].error = nvdisp_rg_region_crc_region7_error_yes_f();
	masks[7].enable = nvdisp_rg_region_crc_control_region7_enable_f();

	masks[8].pending = nvdisp_rg_region_crc_region8_pending_yes_f();
	masks[8].error = nvdisp_rg_region_crc_region8_error_yes_f();
	masks[8].enable = nvdisp_rg_region_crc_control_region8_enable_f();
}

static int tegra_nvdisp_crc_region_enable(struct tegra_dc *dc,
					  struct tegra_dc_ext_crc_conf *conf)
{
	struct tegra_dc_ext_crc_region *region = &conf->region;
	u32 point, size, control, status;
	u32 crc_readback_location_mask =
	nvdisp_rg_region_crc_control_readback_location_readback_golden_f();

	if (conf->crc.valid) {
		dev_err(&dc->ndev->dev,
			"Programming golden CRCs is yet to be supported\n");
		return -ENOTSUPP;
	}

	status = tegra_dc_readl(dc, nvdisp_rg_region_crc_r());
	status |= masks[region->id].pending;
	tegra_dc_writel(dc, status, nvdisp_rg_region_crc_r());

	/*
	 * HW Bug 2038907: In DP mode, vertical calculations are off by
	 *                 1 line. Adjust them to match exactly with HDMI mode.
	 */
	if (dc->out->type == TEGRA_DC_OUT_FAKE_DP ||
	    dc->out->type == TEGRA_DC_OUT_DP) {
		if (region->y == 0)
			region->h += 1;
		else
			region->y += 1;
	}

	point = nvdisp_rg_region_0_point_x_f(region->x) |
		nvdisp_rg_region_0_point_y_f(region->y);
	tegra_dc_writel(dc, point, regs[region->id].point);

	size = nvdisp_rg_region_0_size_width_f(region->w) |
	       nvdisp_rg_region_0_size_height_f(region->h);
	tegra_dc_writel(dc, size, regs[region->id].size);

	control = tegra_dc_readl(dc, nvdisp_rg_region_crc_control_r());
	control |= masks[region->id].enable;

	/* TODO: This is yet to be supported. We return -ENOTSUPP at the
	 * begnning of the function for this case
	 */
	if (conf->crc.valid) {
		control |= crc_readback_location_mask;

		tegra_dc_writel(dc, conf->crc.val,
				regs[region->id].golden_crc);
	}

	tegra_dc_writel(dc, control, nvdisp_rg_region_crc_control_r());

	return 0;
}

static void tegra_nvdisp_crc_rg_comp_enable(struct tegra_dc *dc,
					    struct tegra_dc_ext_crc_conf *conf)
{
	u32 ctl = nvdisp_crc_control_enable_enable_f();
	enum tegra_dc_ext_crc_input_data input_data = conf->input_data;

	if (input_data == TEGRA_DC_EXT_CRC_INPUT_DATA_FULL_FRAME)
		ctl |= nvdisp_crc_control_input_data_full_frame_f();
	else
		ctl |= nvdisp_crc_control_input_data_active_data_f();

	tegra_dc_writel(dc, ctl, nvdisp_crc_control_r());
}

int tegra_nvdisp_crc_enable(struct tegra_dc *dc,
			    struct tegra_dc_ext_crc_conf *conf)
{
	int ret = 0;
	enum tegra_dc_ext_crc_type type = conf->type;
	u8 id; /* region id */

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);

	switch (type) {
	case TEGRA_DC_EXT_CRC_TYPE_RG_REGIONAL:
		id = conf->region.id;
		if (id >= TEGRA_DC_EXT_MAX_REGIONS) {
			ret = -EINVAL;
			goto error_out;
		}

		ret = tegra_nvdisp_crc_region_enable(dc, conf);
		if (ret)
			goto error_out;

		atomic_inc(&dc->crc_ref_cnt.region[id]);
		atomic_inc(&dc->crc_ref_cnt.regional);
		break;
	case TEGRA_DC_EXT_CRC_TYPE_RG:
		tegra_nvdisp_crc_rg_comp_enable(dc, conf);
		atomic_inc(&dc->crc_ref_cnt.rg);
		break;
	case TEGRA_DC_EXT_CRC_TYPE_COMP:
		tegra_nvdisp_crc_rg_comp_enable(dc, conf);
		atomic_inc(&dc->crc_ref_cnt.comp);
		break;
	case TEGRA_DC_EXT_CRC_TYPE_OR:
		if (!dc->out_ops->crc_en) {
			ret = -ENOTSUPP;
			goto error_out;
		}

		ret = dc->out_ops->crc_en(dc, &conf->or_params);
		if (ret)
			goto error_out;

		atomic_inc(&dc->crc_ref_cnt.out);
		break;
	default:
		ret = -ENOTSUPP;
		goto error_out;
	}

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	if (atomic_inc_return(&dc->crc_ref_cnt.global) == 1) {
		ret = tegra_dc_config_frame_end_intr(dc, true);
		if (ret) {
			atomic_dec(&dc->crc_ref_cnt.global);

			switch (type) {
			case TEGRA_DC_EXT_CRC_TYPE_RG_REGIONAL:
				id = conf->region.id;
				atomic_dec(&dc->crc_ref_cnt.regional);
				atomic_dec(&dc->crc_ref_cnt.region[id]);
				break;
			case TEGRA_DC_EXT_CRC_TYPE_RG:
				atomic_dec(&dc->crc_ref_cnt.rg);
				break;
			case TEGRA_DC_EXT_CRC_TYPE_COMP:
				atomic_dec(&dc->crc_ref_cnt.comp);
				break;
			case TEGRA_DC_EXT_CRC_TYPE_OR:
				atomic_dec(&dc->crc_ref_cnt.out);
				break;
			default:
				return -ENOTSUPP;
			}

			return ret;
		}
	}

	return 0;

error_out:
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
	return ret;
}

static void tegra_nvdisp_crc_region_disable(struct tegra_dc *dc,
					    struct tegra_dc_ext_crc_conf *conf)
{
	struct tegra_dc_ext_crc_region *region = &conf->region;
	u32 control;

	tegra_dc_writel(dc, 0x0, regs[region->id].point);
	tegra_dc_writel(dc, 0x0, regs[region->id].size);
	tegra_dc_writel(dc, 0x0, regs[region->id].golden_crc);

	control = tegra_dc_readl(dc, nvdisp_rg_region_crc_control_r());
	control ^= masks[region->id].enable;

	tegra_dc_writel(dc, control, nvdisp_rg_region_crc_control_r());
}

int tegra_nvdisp_crc_disable(struct tegra_dc *dc,
			     struct tegra_dc_ext_crc_conf *conf)
{
	int ret = 0;
	enum tegra_dc_ext_crc_type type = conf->type;
	u8 id; /* region id */

	mutex_lock(&dc->lock);
	tegra_dc_get(dc);

	switch (type) {
	case TEGRA_DC_EXT_CRC_TYPE_RG_REGIONAL:
		id = conf->region.id;
		if (id >= TEGRA_DC_EXT_MAX_REGIONS) {
			ret = -EINVAL;
			goto error_out;
		}

		if (!atomic_dec_return(&dc->crc_ref_cnt.region[id]))
			tegra_nvdisp_crc_region_disable(dc, conf);

		atomic_dec(&dc->crc_ref_cnt.regional);
		break;
	case TEGRA_DC_EXT_CRC_TYPE_RG:
		if (atomic_dec_return(&dc->crc_ref_cnt.rg) == 0 &&
		    atomic_read(&dc->crc_ref_cnt.comp) == 0)
			tegra_dc_writel(dc, 0x0, nvdisp_crc_control_r());
		break;
	case TEGRA_DC_EXT_CRC_TYPE_COMP:
		if (atomic_dec_return(&dc->crc_ref_cnt.comp) == 0 &&
		    atomic_read(&dc->crc_ref_cnt.rg) == 0)
			tegra_dc_writel(dc, 0x0, nvdisp_crc_control_r());
		break;
	case TEGRA_DC_EXT_CRC_TYPE_OR:
		if (atomic_dec_return(&dc->crc_ref_cnt.out) == 0) {
			if (!dc->out_ops->crc_dis) {
				ret = -ENOTSUPP;
				goto error_out;
			}
			ret = dc->out_ops->crc_dis(dc, &conf->or_params);
			if (ret) {
				atomic_inc(&dc->crc_ref_cnt.out);
				goto error_out;
			}
		}
		break;
	default:
		ret = -ENOTSUPP;
		goto error_out;
	}

	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);

	if (atomic_dec_return(&dc->crc_ref_cnt.global) == 0) {
		ret = tegra_dc_config_frame_end_intr(dc, false);
		if (ret) {
			switch (type) {
			case TEGRA_DC_EXT_CRC_TYPE_RG_REGIONAL:
				id = conf->region.id;
				atomic_inc(&dc->crc_ref_cnt.region[id]);
				atomic_inc(&dc->crc_ref_cnt.regional);
				break;
			case TEGRA_DC_EXT_CRC_TYPE_RG:
				atomic_inc(&dc->crc_ref_cnt.rg);
				break;
			case TEGRA_DC_EXT_CRC_TYPE_COMP:
				atomic_inc(&dc->crc_ref_cnt.comp);
				break;
			case TEGRA_DC_EXT_CRC_TYPE_OR:
				atomic_inc(&dc->crc_ref_cnt.out);
				break;
			default:
				return -ENOTSUPP;
			}

			return ret;
		}
	}

	return 0;

error_out:
	tegra_dc_put(dc);
	mutex_unlock(&dc->lock);
	return ret;
}

static inline bool is_region_enabled(u32 ctl, int id)
{
	return ctl & masks[id].enable;
}

static inline bool is_region_crc_ready(u32 status, int id)
{
	return status & masks[id].pending;
}

static int tegra_nvdisp_crc_rg_regional_get(struct tegra_dc *dc,
					    struct tegra_dc_crc_buf_ele *e)
{
	int id, ret = 0;
	u32 ctl, status;
	u32 frame_cnt;
	u32 crc_readback_location_mask =
	nvdisp_rg_region_crc_control_readback_location_readback_golden_f();

	status = tegra_dc_readl(dc, nvdisp_rg_region_crc_r());

	ctl = tegra_dc_readl(dc, nvdisp_rg_region_crc_control_r());
	if (ctl & crc_readback_location_mask) {
		dev_err(&dc->ndev->dev,
			"Golden CRCs are already programmed. Ignore reading "
			"status = %#x, ctl = %#x\n", status, ctl);
		return -EPERM;
	}

	frame_cnt = tegra_dc_get_frame_cnt(dc);

	for (id = 0; id < TEGRA_DC_MAX_CRC_REGIONS; id++) {
		if (!is_region_enabled(ctl, id))
			continue;

		if (!is_region_crc_ready(status, id)) {
			trace_display_crc_rgnl_stat(dc->ctrl_num, frame_cnt,
				"RGN", id, status);
			continue;
		}

		/* Clearing out the error bit is a functional no-op, since the
		 * error bit simply signifies that the HW calculated CRC value
		 * is different from the value programmed in the golden
		 * register, or lack thereof. However, we clear it to avoid
		 * confusion when reading register dumps
		 */
		if (status & masks[id].error)
			status |= masks[id].error;

		e->regional[id].crc = tegra_dc_readl(dc, regs[id].golden_crc);
		e->regional[id].valid = true;
		status |= masks[id].pending;

		trace_display_crc_rgnl(dc->ctrl_num, frame_cnt, "RGN",
			id, status, e->regional[id].crc);
	}

	tegra_dc_writel(dc, status, nvdisp_rg_region_crc_r());

	return ret;
}

static int tegra_nvdisp_crc_rg_get(struct tegra_dc *dc,
				   struct tegra_dc_crc_buf_ele *e)
{
	u32 status;
	u32 frame_cnt;

	status = tegra_dc_readl(dc, nvdisp_rg_crca_r());
	frame_cnt = tegra_dc_get_frame_cnt(dc);

	if (status & nvdisp_rg_crca_valid_true_f()) {
		e->rg.crc = tegra_dc_readl(dc, nvdisp_rg_crcb_r());
		e->rg.valid = true;
		trace_display_crc_read(dc->ctrl_num, frame_cnt, "RG  ",
			status, e->rg.crc);
	}

	if (status & nvdisp_rg_crca_error_true_f()) {
		trace_display_crc_late(dc->ctrl_num, frame_cnt, "RG  ",
			e->rg.valid, e->rg.crc);
		if (!e->rg.valid)
			e->rg.crc = 0;
		status |= nvdisp_rg_crca_error_true_f();
		tegra_dc_writel(dc, status, nvdisp_rg_crca_r());
	}

	return 0;
}

static int tegra_nvdisp_crc_comp_get(struct tegra_dc *dc,
				     struct tegra_dc_crc_buf_ele *e)
{
	u32 status;
	u32 frame_cnt;

	status = tegra_dc_readl(dc, nvdisp_comp_crca_r());
	frame_cnt = tegra_dc_get_frame_cnt(dc);

	if (status & nvdisp_comp_crca_valid_true_f()) {
		e->comp.crc = tegra_dc_readl(dc, nvdisp_comp_crcb_r());
		e->comp.valid = true;
		trace_display_crc_read(dc->ctrl_num, frame_cnt, "COMP",
			status, e->comp.crc);
	}

	if (status & nvdisp_comp_crca_error_true_f()) {
		trace_display_crc_late(dc->ctrl_num, frame_cnt, "COMP",
			e->comp.valid, e->comp.crc);
		if (!e->comp.valid)
			e->comp.crc = 0;
		status |= nvdisp_comp_crca_error_true_f();
		tegra_dc_writel(dc, status, nvdisp_comp_crca_r());
	}

	return 0;
}

static int tegra_nvdisp_crc_or_get(struct tegra_dc *dc,
				   struct tegra_dc_crc_buf_ele *e)
{
	int ret;

	if (dc->out_ops->crc_get) {
		ret = dc->out_ops->crc_get(dc, &e->sor.crc);
		if (ret)
			return ret;
	}

	e->sor.valid = true;

	return 0;
}

int tegra_nvdisp_crc_collect(struct tegra_dc *dc,
			     struct tegra_dc_crc_buf_ele *e)
{
#define NEXT_FRAME_POLL_DELAY_US 1
#define FRAME_BOUNDARY_CROSSING_RETRIES 5
	int ret = 0, iter;
	int max_retries = FRAME_BOUNDARY_CROSSING_RETRIES;
	bool valids = false; /* Logical OR of valid fields of individual CRCs */
	u32 frame_cnt_start;
	u32 frame_cnt_end;

retry:
	frame_cnt_start = tegra_dc_get_frame_cnt(dc);

	if (atomic_read(&dc->crc_ref_cnt.rg))
		tegra_nvdisp_crc_rg_get(dc, e);

	if (atomic_read(&dc->crc_ref_cnt.comp))
		tegra_nvdisp_crc_comp_get(dc, e);

	if (atomic_read(&dc->crc_ref_cnt.out))
		tegra_nvdisp_crc_or_get(dc, e);

	if (atomic_read(&dc->crc_ref_cnt.regional))
		ret = tegra_nvdisp_crc_rg_regional_get(dc, e);

	frame_cnt_end = tegra_dc_get_frame_cnt(dc);

	if (frame_cnt_end != frame_cnt_start) {
		unsigned long timeout_jf;
		u32 frame_cnt_retry;

		trace_display_crc_boundary_crossed(dc->ctrl_num,
			frame_cnt_start, frame_cnt_end,
			"wait for next frame");

		timeout_jf = jiffies + msecs_to_jiffies(MSEC_PER_SEC << 1);

		do {
			usleep_range(NEXT_FRAME_POLL_DELAY_US,
				NEXT_FRAME_POLL_DELAY_US << 1);
			frame_cnt_retry = tegra_dc_get_frame_cnt(dc);
		} while ((frame_cnt_end == frame_cnt_retry) &&
			time_after(timeout_jf, jiffies));

		if (frame_cnt_end == frame_cnt_retry) {
			dev_err(&dc->ndev->dev,
				"frame=%u NEXT FRAME POLL TIMEOUT\n",
				frame_cnt_end);
			trace_display_crc_boundary_crossed(dc->ctrl_num,
				frame_cnt_end, frame_cnt_retry,
				"NEXT FRAME POLL TIMEOUT");
			return -EBUSY;
		}
		if (--max_retries <= 0) {
			trace_display_crc_boundary_crossed(dc->ctrl_num,
				frame_cnt_end, frame_cnt_retry,
				"retries exhausted");
			return -EBUSY;
		}
		trace_display_crc_boundary_crossed(dc->ctrl_num,
			frame_cnt_end, frame_cnt_retry,
			"rereading CRCs");
		goto retry;
	}

	valids = e->rg.valid || e->comp.valid || e->sor.valid;

	for (iter = 0; iter < TEGRA_DC_EXT_MAX_REGIONS && !valids; iter++)
		valids = e->regional[iter].valid;

	return valids ? ret : -EINVAL;
#undef FRAME_BOUNDARY_CROSSING_RETRIES
#undef NEXT_FRAME_POLL_DELAY_US
}

void tegra_nvdisp_crc_reset(struct tegra_dc *dc)
{
	tegra_dc_writel(dc, 0x0, nvdisp_rg_region_crc_control_r());
	tegra_dc_writel(dc, 0x0, nvdisp_crc_control_r());
}
