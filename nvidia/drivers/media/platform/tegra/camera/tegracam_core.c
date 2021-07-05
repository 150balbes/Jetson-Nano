/*
 * tegracam_core - tegra camera framework initialization
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
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/types.h>
#include <media/tegra-v4l2-camera.h>
#include <media/tegracam_core.h>

struct tegracam_device_entry {
	struct tegracam_device *tc_dev;
	struct list_head list;
};

static struct list_head tc_device_list_head =
	LIST_HEAD_INIT(tc_device_list_head);
static DEFINE_MUTEX(tc_device_list_mutex);

/* use semantic versioning convention */
#define TEGRACAM_MAJOR_VERSION 2
#define TEGRACAM_MINOR_VERSION 0
#define TEGRACAM_PATCH_VERSION 6

u32 tegracam_version(u8 major, u8 minor, u8 patch)
{
	return ((major << 16) | (minor << 8) | patch);
}
EXPORT_SYMBOL_GPL(tegracam_version);

u32 tegracam_query_version(const char *of_dev_name)
{
	struct tegracam_device_entry *entry = NULL;
	struct device_node *node;
	u32 version = 0;

	if (of_dev_name == NULL)
		return 0;

	mutex_lock(&tc_device_list_mutex);
	list_for_each_entry(entry, &tc_device_list_head, list) {
		node = entry->tc_dev->dev->of_node;
		if (strcmp(of_dev_name, node->name) == 0) {
			version = entry->tc_dev->version;
			break;
		}
	}
	mutex_unlock(&tc_device_list_mutex);

	return version;
}
EXPORT_SYMBOL_GPL(tegracam_query_version);

struct tegracam_device *to_tegracam_device(struct camera_common_data *data)
{
	/* fix this by moving subdev to base struct */
	return (struct tegracam_device *)data->tegracam_ctrl_hdl->tc_dev;
}
EXPORT_SYMBOL_GPL(to_tegracam_device);

void tegracam_set_privdata(struct tegracam_device * tc_dev, void *priv)
{
	tc_dev->priv = priv;

	/* TODO: cleanup needed for priv once sensors adapt this driver */
	tc_dev->s_data->priv = priv;
}
EXPORT_SYMBOL_GPL(tegracam_set_privdata);

void *tegracam_get_privdata(struct tegracam_device *tc_dev)
{
	return tc_dev->priv;
}
EXPORT_SYMBOL_GPL(tegracam_get_privdata);

int tegracam_device_register(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct tegracam_ctrl_handler *ctrl_hdl = NULL;
	struct tegracam_device_entry *tc_dev_entry = NULL;
	struct camera_common_power_rail *pw_rail = NULL;
	struct camera_common_data *s_data = NULL;
	struct sensor_mode_properties *sensor_mode = NULL;
	struct sensor_signal_properties *signal_props = NULL;
	struct sensor_image_properties *image_props = NULL;
	u32 mode_idx = 0;
	int err = 0;

	s_data = devm_kzalloc(dev,
		sizeof(struct camera_common_data), GFP_KERNEL);
	s_data->dev = dev;

	ctrl_hdl = devm_kzalloc(dev,
		sizeof(struct tegracam_ctrl_handler), GFP_KERNEL);
	ctrl_hdl->tc_dev = tc_dev;
	s_data->tegracam_ctrl_hdl = ctrl_hdl;

	pw_rail = devm_kzalloc(dev,
		sizeof(struct camera_common_power_rail), GFP_KERNEL);
	s_data->power = pw_rail;

	s_data->regmap = devm_regmap_init_i2c(tc_dev->client,
					tc_dev->dev_regmap_config);
	if (IS_ERR(s_data->regmap)) {
		dev_err(dev,
			"regmap init failed: %ld\n", PTR_ERR(s_data->regmap));
		return -ENODEV;
	}

	if (!tc_dev->sensor_ops) {
		dev_err(dev, "sensor ops not initialized\n");
		return -EINVAL;
	}
	s_data->ops = tc_dev->sensor_ops;

	s_data->pdata = tc_dev->sensor_ops->parse_dt(tc_dev);
	if (PTR_ERR(s_data->pdata) == -EPROBE_DEFER)
		return -EPROBE_DEFER;
	if (!s_data->pdata) {
		dev_err(dev, "unable to get platform data\n");
		return -EFAULT;
	}
	tc_dev->s_data = s_data;
	err = tc_dev->sensor_ops->power_get(tc_dev);
	if (err) {
		dev_err(dev, "unable to power get\n");
		return -EFAULT;
	}

	err = camera_common_initialize(s_data, tc_dev->name);
	if (err) {
		dev_err(dev, "Failed to initialize %s\n", tc_dev->name);
		return err;
	}

	/* TODO: updated default mode from DT ?? */
	mode_idx = s_data->mode_prop_idx = 0;
	/* init format context */
	/*TODO: compile frmfmt array from DT */
	s_data->frmfmt = tc_dev->sensor_ops->frmfmt_table;
	s_data->numfmts = tc_dev->sensor_ops->numfrmfmts;
	sensor_mode = &s_data->sensor_props.sensor_modes[mode_idx];
	signal_props = &sensor_mode->signal_properties;
	image_props = &sensor_mode->image_properties;

	s_data->def_mode = s_data->frmfmt[mode_idx].mode;
	s_data->colorfmt =
		camera_common_find_pixelfmt(image_props->pixel_format);
	s_data->def_width = s_data->fmt_width =
		s_data->frmfmt[mode_idx].size.width;
	s_data->def_height = s_data->fmt_height =
		s_data->frmfmt[mode_idx].size.height;
	s_data->def_clk_freq = signal_props->mclk_freq * 1000;

	/* add version info to identify the right feature set */
	tc_dev->version = tegracam_version(TEGRACAM_MAJOR_VERSION,
			TEGRACAM_MINOR_VERSION, TEGRACAM_PATCH_VERSION);
	s_data->version = tc_dev->version;

	/* Add tc_dev to list of registered devices */
	tc_dev_entry = devm_kzalloc(dev,
			sizeof(*tc_dev_entry), GFP_KERNEL);
	tc_dev_entry->tc_dev = tc_dev;
	INIT_LIST_HEAD(&tc_dev_entry->list);
	mutex_lock(&tc_device_list_mutex);
	list_add(&tc_dev_entry->list, &tc_device_list_head);
	mutex_unlock(&tc_device_list_mutex);

	dev_info(dev, "tegracam sensor driver:%s_v%d.%d.%d\n",
			tc_dev->name, TEGRACAM_MAJOR_VERSION,
			TEGRACAM_MINOR_VERSION, TEGRACAM_PATCH_VERSION);

	return 0;
}
EXPORT_SYMBOL_GPL(tegracam_device_register);

void tegracam_device_unregister(struct tegracam_device *tc_dev)
{
	struct tegracam_device_entry *entry;
	struct tegracam_device_entry *temp;
	struct camera_common_data *s_data = tc_dev->s_data;

	tc_dev->sensor_ops->power_put(tc_dev);
	camera_common_cleanup(s_data);

	/* Remove tc_dev from list of registered devices */
	mutex_lock(&tc_device_list_mutex);
	list_for_each_entry_safe(entry, temp, &tc_device_list_head, list) {
		if (entry->tc_dev == tc_dev) {
			list_del(&entry->list);
			break;
		}
	}
	mutex_unlock(&tc_device_list_mutex);
	devm_kfree(tc_dev->dev, entry);
	devm_kfree(tc_dev->dev, tc_dev->s_data->tegracam_ctrl_hdl);
	devm_kfree(tc_dev->dev, tc_dev->s_data->power);
	devm_kfree(tc_dev->dev, tc_dev->s_data);
	tc_dev->s_data = NULL;
}
EXPORT_SYMBOL_GPL(tegracam_device_unregister);
