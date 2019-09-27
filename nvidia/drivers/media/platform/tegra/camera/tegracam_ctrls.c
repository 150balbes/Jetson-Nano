/*
 * tegracam_ctrls - control framework for tegra camera drivers
 *
 * Copyright (c) 2017-2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <linux/types.h>
#include <media/tegra-v4l2-camera.h>
#include <media/camera_common.h>
#include <media/tegracam_utils.h>

#define CTRL_U32_MIN 0
#define CTRL_U32_MAX 0x7FFFFFFF
#define CTRL_U64_MIN 0
#define CTRL_U64_MAX 0x7FFFFFFFFFFFFFFFLL
#define CTRL_S32_MIN 0x80000000
#define CTRL_S32_MAX 0x7FFFFFFF
#define CTRL_S64_MIN 0x8000000000000000LL
#define CTRL_S64_MAX 0x7FFFFFFFFFFFFFFFLL
#define CTRL_MAX_STR_SIZE 4096

#define TEGRACAM_DEF_CTRLS 1

static int tegracam_s_ctrl(struct v4l2_ctrl *ctrl);
static const struct v4l2_ctrl_ops tegracam_ctrl_ops = {
	.s_ctrl = tegracam_s_ctrl,
};

static const u32 tegracam_def_cids[] = {
	TEGRA_CAMERA_CID_GROUP_HOLD,
};

/*
 * For auto control, the states of the previous controls must
 * be applied to get optimal quality faster. List all the controls
 * which must be overriden
 */
static const u32 tegracam_override_cids[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
};
#define NUM_OVERRIDE_CTRLS ARRAY_SIZE(tegracam_override_cids)

static struct v4l2_ctrl_config ctrl_cfg_list[] = {
/* Do not change the name field for the controls! */
	{
		.ops = &tegracam_ctrl_ops,
		.id = TEGRA_CAMERA_CID_GAIN,
		.name = "Gain",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = CTRL_U64_MIN,
		.max = CTRL_U64_MAX,
		.def = CTRL_U64_MIN,
		.step = 1,
	},
	{
		.ops = &tegracam_ctrl_ops,
		.id = TEGRA_CAMERA_CID_EXPOSURE,
		.name = "Exposure",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = CTRL_U64_MIN,
		.max = CTRL_U64_MAX,
		.def = CTRL_U64_MIN,
		.step = 1,
	},
	{
		.ops = &tegracam_ctrl_ops,
		.id = TEGRA_CAMERA_CID_EXPOSURE_SHORT,
		.name = "Exposure Short",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = CTRL_U64_MIN,
		.max = CTRL_U64_MAX,
		.def = CTRL_U64_MIN,
		.step = 1,
	},
	{
		.ops = &tegracam_ctrl_ops,
		.id = TEGRA_CAMERA_CID_FRAME_RATE,
		.name = "Frame Rate",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = CTRL_U64_MIN,
		.max = CTRL_U64_MAX,
		.def = CTRL_U64_MIN,
		.step = 1,
	},
	{
		.ops = &tegracam_ctrl_ops,
		.id = TEGRA_CAMERA_CID_GROUP_HOLD,
		.name = "Group Hold",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.flags = V4L2_CTRL_FLAG_EXECUTE_ON_WRITE,
		.min = 0,
		.max = 1,
		.def = 0,
		.step = 1,
	},
	{
		.ops = &tegracam_ctrl_ops,
		.id = TEGRA_CAMERA_CID_EEPROM_DATA,
		.name = "EEPROM Data",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.min = 0,
		.max = CTRL_MAX_STR_SIZE,
		.step = 2,
	},
	{
		.ops = &tegracam_ctrl_ops,
		.id = TEGRA_CAMERA_CID_FUSE_ID,
		.name = "Fuse ID",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.min = 0,
		.max = CTRL_MAX_STR_SIZE,
		.step = 2,
	},
	{
		.ops = &tegracam_ctrl_ops,
		.id = TEGRA_CAMERA_CID_SENSOR_MODE_ID,
		.name = "Sensor Mode",
		.type = V4L2_CTRL_TYPE_INTEGER64,
		.flags = V4L2_CTRL_FLAG_SLIDER,
		.min = CTRL_U32_MIN,
		.max = CTRL_U32_MAX,
		.def = CTRL_U32_MIN,
		.step = 1,
	},
	{
		.ops = &tegracam_ctrl_ops,
		.id = TEGRA_CAMERA_CID_HDR_EN,
		.name = "HDR enable",
		.type = V4L2_CTRL_TYPE_INTEGER_MENU,
		.min = 0,
		.max = ARRAY_SIZE(switch_ctrl_qmenu) - 1,
		.menu_skip_mask = 0,
		.def = 0,
		.qmenu_int = switch_ctrl_qmenu,
	},
	{
		.ops = &tegracam_ctrl_ops,
		.id = TEGRA_CAMERA_CID_OTP_DATA,
		.name = "OTP Data",
		.type = V4L2_CTRL_TYPE_STRING,
		.flags = V4L2_CTRL_FLAG_READ_ONLY,
		.min = 0,
		.max = CTRL_MAX_STR_SIZE,
		.step = 2,
	},
};

static int tegracam_get_ctrl_index(u32 cid)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ctrl_cfg_list); i++) {
		if (ctrl_cfg_list[i].id == cid)
			return i;
	}

	return -EINVAL;
}

static int tegracam_get_string_ctrl_size(u32 cid,
		const struct tegracam_ctrl_ops *ops)
{
	u32 index = 0;

	switch (cid) {
	case TEGRA_CAMERA_CID_EEPROM_DATA:
		index = TEGRA_CAM_STRING_CTRL_EEPROM_INDEX;
		break;
	case TEGRA_CAMERA_CID_FUSE_ID:
		index = TEGRA_CAM_STRING_CTRL_FUSEID_INDEX;
		break;
	case TEGRA_CAMERA_CID_OTP_DATA:
		index = TEGRA_CAM_STRING_CTRL_OTP_INDEX;
		break;
	default:
		return -EINVAL;
	}

	return ops->string_ctrl_size[index];
}

static int tegracam_setup_string_ctrls(struct tegracam_device *tc_dev,
				struct tegracam_ctrl_handler *handler)
{
	const struct tegracam_ctrl_ops *ops = handler->ctrl_ops;
	u32 numctrls = ops->numctrls;
	int i;
	int err = 0;

	for (i = 0; i < numctrls; i++) {
		struct v4l2_ctrl *ctrl = handler->ctrls[i];

		if (ctrl->type == V4L2_CTRL_TYPE_STRING) {
			err = ops->fill_string_ctrl(tc_dev, ctrl);
			if (err)
				return err;
		}
	}

	return 0;
}

static int tegracam_set_ctrls(struct tegracam_ctrl_handler *handler,
			struct v4l2_ctrl *ctrl)
{
	const struct tegracam_ctrl_ops *ops = handler->ctrl_ops;
	struct tegracam_device *tc_dev = handler->tc_dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	int err = 0;
	u32 status = 0;

	/* For controls that are independent of power state */
	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_SENSOR_MODE_ID:
		s_data->sensor_mode_id = (int) (*ctrl->p_new.p_s64);
		return 0;
	case TEGRA_CAMERA_CID_HDR_EN:
		return 0;
	}

	if (v4l2_subdev_call(&s_data->subdev, video,
				g_input_status, &status)) {
		dev_err(s_data->dev, "power status query unsupported\n");
		return -ENOTTY;
	}

	/* power state is turned off, do not program sensor now */
	if (!status)
		return 0;

	/* For controls that require sensor to be on */
	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_GAIN:
		err = ops->set_gain(tc_dev, *ctrl->p_new.p_s64);
		break;
	case TEGRA_CAMERA_CID_FRAME_RATE:
		err = ops->set_frame_rate(tc_dev, *ctrl->p_new.p_s64);
		break;
	case TEGRA_CAMERA_CID_EXPOSURE:
		err = ops->set_exposure(tc_dev, *ctrl->p_new.p_s64);
		break;
	case TEGRA_CAMERA_CID_EXPOSURE_SHORT:
		err = ops->set_exposure_short(tc_dev, *ctrl->p_new.p_s64);
		break;
	case TEGRA_CAMERA_CID_GROUP_HOLD:
		err = ops->set_group_hold(tc_dev, ctrl->val);
		break;
	default:
		pr_err("%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	return err;
}

static int tegracam_set_grouphold_ex(struct tegracam_device *tc_dev,
				struct sensor_blob *blob,
				bool status)
{
	const struct tegracam_ctrl_ops *ops = tc_dev->tcctrl_ops;
	struct camera_common_data *s_data = tc_dev->s_data;
	int err = 0;

	/*
	 * when grouphold is set, reset control blob
	 * set grouphold register using set API
	 * start packetize commands for delivering the blob
	 * when grouphold is unset, unset grouphold register
	 * and write the blob only if sensor is streaming.
	 */
	if (status) {
		memset(blob, 0, sizeof(struct sensor_blob));
		err = ops->set_group_hold_ex(tc_dev, blob, status);
		if (err)
			return err;
	} else {
		err = ops->set_group_hold_ex(tc_dev, blob, status);
		if (err)
			return err;

		/* TODO: block this write selectively from VI5 */
		if (tc_dev->is_streaming) {
			err = write_sensor_blob(s_data->regmap, blob);
			if (err)
				return err;
		}
	}

	return 0;
}

static int tegracam_set_ctrls_ex(struct tegracam_ctrl_handler *handler,
				struct v4l2_ctrl *ctrl)
{
	const struct tegracam_ctrl_ops *ops = handler->ctrl_ops;
	struct tegracam_device *tc_dev = handler->tc_dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct tegracam_sensor_data *sensor_data = &handler->sensor_data;
	struct sensor_blob *blob = &sensor_data->ctrls_blob;
	int err = 0;

	switch (ctrl->id) {
	case TEGRA_CAMERA_CID_GAIN:
		err = ops->set_gain_ex(tc_dev, blob, *ctrl->p_new.p_s64);
		break;
	case TEGRA_CAMERA_CID_FRAME_RATE:
		err = ops->set_frame_rate_ex(tc_dev, blob, *ctrl->p_new.p_s64);
		break;
	case TEGRA_CAMERA_CID_EXPOSURE:
		err = ops->set_exposure_ex(tc_dev, blob, *ctrl->p_new.p_s64);
		break;
	case TEGRA_CAMERA_CID_GROUP_HOLD:
		err = tegracam_set_grouphold_ex(tc_dev, blob, ctrl->val);
		break;
	case TEGRA_CAMERA_CID_SENSOR_MODE_ID:
		s_data->sensor_mode_id = (int) (*ctrl->p_new.p_s64);
		break;
	case TEGRA_CAMERA_CID_HDR_EN:
		break;
	default:
		pr_err("%s: unknown ctrl id.\n", __func__);
		return -EINVAL;
	}

	return err;
}


static int tegracam_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct tegracam_ctrl_handler *handler =
		container_of(ctrl->handler,
			struct tegracam_ctrl_handler, ctrl_handler);
	const struct tegracam_ctrl_ops *ops = handler->ctrl_ops;

	if (ops->is_blob_supported)
		return tegracam_set_ctrls_ex(handler, ctrl);
	else
		return tegracam_set_ctrls(handler, ctrl);

	return 0;
}

int tegracam_ctrl_set_overrides(struct tegracam_ctrl_handler *hdl)
{
	struct v4l2_ext_controls ctrls;
	struct v4l2_ext_control control;
	struct tegracam_device *tc_dev = hdl->tc_dev;
	struct device *dev = tc_dev->dev;
	const struct tegracam_ctrl_ops *ops = hdl->ctrl_ops;
	struct tegracam_sensor_data *sensor_data = &hdl->sensor_data;
	struct sensor_blob *blob = &sensor_data->ctrls_blob;
	bool is_blob_supported = ops->is_blob_supported;
	int err, result = 0;
	int i;

	/*
	 * write list of override regs for the asking frame length,
	 * coarse integration time, and gain. Failures to write
	 * overrides are non-fatal
	 */
	memset(&ctrls, 0, sizeof(ctrls));
#if LINUX_VERSION_CODE > KERNEL_VERSION(4, 9, 0)
	ctrls.which = V4L2_CTRL_ID2WHICH(TEGRA_CAMERA_CID_BASE);
#else
	ctrls.ctrl_class = V4L2_CTRL_ID2CLASS(TEGRA_CAMERA_CID_BASE);
#endif
	ctrls.count = 1;
	ctrls.controls = &control;

	for (i = 0; i < NUM_OVERRIDE_CTRLS; i++) {
		s64 val = 0;

		control.id = tegracam_override_cids[i];
		result = v4l2_g_ext_ctrls(&hdl->ctrl_handler, &ctrls);
		if (result == 0) {
			val = control.value64;
			switch (control.id) {
			case TEGRA_CAMERA_CID_GAIN:
				if (is_blob_supported)
					err = ops->set_gain_ex(tc_dev,
								blob, val);
				else
					err = ops->set_gain(tc_dev, val);
				break;
			case TEGRA_CAMERA_CID_EXPOSURE:
				if (is_blob_supported)
					err = ops->set_exposure_ex(tc_dev,
								blob, val);
				else
					err = ops->set_exposure(tc_dev, val);
				break;
			case TEGRA_CAMERA_CID_FRAME_RATE:
				if (is_blob_supported)
					err = ops->set_frame_rate_ex(tc_dev,
								blob, val);
				else
					err = ops->set_frame_rate(tc_dev, val);
				break;
			default:
				dev_err(dev, "%s: unsupported override %x\n",
						__func__, control.id);
				return -EINVAL;
			}

			if (err) {
				dev_err(dev, "%s: error to set %d override\n",
						__func__, control.id);
				return err;
			}
		}
	}

	return 0;
}

int tegracam_init_ctrl_ranges_by_mode(
		struct tegracam_ctrl_handler *handler,
		u32 modeidx)
{
	struct tegracam_device *tc_dev = handler->tc_dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct sensor_control_properties *ctrlprops = NULL;
	s64 min_short_exp_time = 0;
	s64 max_short_exp_time = 0;
	s64 default_short_exp_time = 0;
	int i;

	if (modeidx >= s_data->sensor_props.num_modes)
		return -EINVAL;

	ctrlprops =
		&s_data->sensor_props.sensor_modes[modeidx].control_properties;

	for (i = 0; i < handler->numctrls; i++) {
		struct v4l2_ctrl *ctrl = handler->ctrls[i];
		int err = 0;

		switch (ctrl->id) {
		case TEGRA_CAMERA_CID_GAIN:
			err = v4l2_ctrl_modify_range(ctrl,
				ctrlprops->min_gain_val,
				ctrlprops->max_gain_val,
				ctrlprops->step_gain_val,
				ctrlprops->default_gain);
			break;
		case TEGRA_CAMERA_CID_FRAME_RATE:
			err = v4l2_ctrl_modify_range(ctrl,
				ctrlprops->min_framerate,
				ctrlprops->max_framerate,
				ctrlprops->step_framerate,
				ctrlprops->default_framerate);
			break;
		case TEGRA_CAMERA_CID_EXPOSURE:
			err = v4l2_ctrl_modify_range(ctrl,
				ctrlprops->min_exp_time.val,
				ctrlprops->max_exp_time.val,
				ctrlprops->step_exp_time.val,
				ctrlprops->default_exp_time.val);
			break;
		case TEGRA_CAMERA_CID_EXPOSURE_SHORT:
			/*
			 * min_hdr_ratio should be equal to max_hdr_ratio.
			 * This will ensure consistent short exposure
			 * limit calculations.
			 */
			min_short_exp_time =
				ctrlprops->min_exp_time.val /
				ctrlprops->min_hdr_ratio;
			max_short_exp_time =
				ctrlprops->max_exp_time.val /
				ctrlprops->min_hdr_ratio;
			default_short_exp_time =
				ctrlprops->default_exp_time.val /
				ctrlprops->min_hdr_ratio;
			err = v4l2_ctrl_modify_range(ctrl,
				min_short_exp_time,
				max_short_exp_time,
				ctrlprops->step_exp_time.val,
				default_short_exp_time);
			dev_dbg(s_data->dev,
				"%s:short_exp_limits[%lld,%lld], default_short_exp_time=%lld\n",
				__func__,
				min_short_exp_time,
				max_short_exp_time,
				default_short_exp_time);
			break;
		default:
			/* Not required to modify these control ranges */
			break;
		}

		if (err) {
			dev_err(s_data->dev,
				"ctrl %s range update failed\n", ctrl->name);
			return err;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegracam_init_ctrl_ranges_by_mode);

int tegracam_init_ctrl_ranges(struct tegracam_ctrl_handler *handler)
{
	struct tegracam_device *tc_dev = handler->tc_dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct device *dev = tc_dev->dev;
	int i, err = 0;

	/* Updating static control ranges */
	for (i = 0; i < handler->numctrls; i++) {
		struct v4l2_ctrl *ctrl = handler->ctrls[i];

		switch (ctrl->id) {
		case TEGRA_CAMERA_CID_SENSOR_MODE_ID:
			err = v4l2_ctrl_modify_range(ctrl,
				CTRL_U32_MIN,
				(s64) s_data->sensor_props.num_modes,
				1,
				CTRL_U32_MIN);
			break;
		default:
			/* Not required to modify these control ranges */
			break;
		}

		if (err) {
			dev_err(s_data->dev,
				"ctrl %s range update failed\n", ctrl->name);
			return err;
		}
	}

	/* Use mode 0 control ranges as default */
	if (s_data->sensor_props.num_modes > 0)	{
		err = tegracam_init_ctrl_ranges_by_mode(handler, 0);
		if (err) {
			dev_err(dev,
				"Error %d updating mode specific control ranges\n",
				err);
			return err;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(tegracam_init_ctrl_ranges);

int tegracam_ctrl_handler_init(struct tegracam_ctrl_handler *handler)
{
	struct tegracam_device *tc_dev = handler->tc_dev;
	struct v4l2_ctrl *ctrl;
	struct v4l2_ctrl_config *ctrl_cfg;
	struct device *dev = tc_dev->dev;
	const struct tegracam_ctrl_ops *ops = handler->ctrl_ops;
	const u32 *cids = ops->ctrl_cid_list;
	u32 numctrls = ops->numctrls + TEGRACAM_DEF_CTRLS;
	int i, j;
	int err = 0;

	err = v4l2_ctrl_handler_init(&handler->ctrl_handler, numctrls);

	for (i = 0, j = 0; i < numctrls; i++) {
		u32 cid = i < ops->numctrls ? cids[i] : tegracam_def_cids[j++];
		int index = tegracam_get_ctrl_index(cid);
		int size = 0;

		if (index >= ARRAY_SIZE(ctrl_cfg_list)) {
			dev_err(dev, "unsupported control in the list\n");
			return -ENOTTY;
		}

		ctrl_cfg = &ctrl_cfg_list[index];
		if (ctrl_cfg->type == V4L2_CTRL_TYPE_STRING) {
			size = tegracam_get_string_ctrl_size(ctrl_cfg->id, ops);
			if (size < 0) {
				dev_err(dev, "Invalid string ctrl size\n");
				return -EINVAL;
			}
			ctrl_cfg->max = size;
		}
		ctrl = v4l2_ctrl_new_custom(&handler->ctrl_handler,
			ctrl_cfg, NULL);
		if (ctrl == NULL) {
			dev_err(dev, "Failed to init %s ctrl\n",
				ctrl_cfg->name);
			return -EINVAL;
		}

		if (ctrl_cfg->type == V4L2_CTRL_TYPE_STRING &&
			ctrl_cfg->flags & V4L2_CTRL_FLAG_READ_ONLY) {
			ctrl->p_new.p_char = devm_kzalloc(tc_dev->dev,
				size + 1, GFP_KERNEL);
		}
		handler->ctrls[i] = ctrl;
	};

	handler->numctrls = numctrls;
	err = v4l2_ctrl_handler_setup(&handler->ctrl_handler);
	if (err) {
		dev_err(dev, "Error %d in control hdl setup\n", err);
		goto error;
	}

	err = handler->ctrl_handler.error;
	if (err) {
		dev_err(dev, "Error %d adding controls\n", err);
		goto error;
	}

	err = tegracam_setup_string_ctrls(tc_dev, handler);
	if (err) {
		dev_err(dev, "setup string controls failed\n");
		goto error;
	}

	err = tegracam_init_ctrl_ranges(handler);
	if (err) {
		dev_err(dev, "Error %d updating control ranges\n", err);
		goto error;
	}
	return 0;
error:
	v4l2_ctrl_handler_free(&handler->ctrl_handler);
	return err;
}
EXPORT_SYMBOL_GPL(tegracam_ctrl_handler_init);
