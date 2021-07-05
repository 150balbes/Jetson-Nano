/*
 * drivers/video/tegra/host/tpg/tpg.c
 *
 * Tegra VI test pattern generator driver
 *
 * Copyright (c) 2017-2019, NVIDIA CORPORATION. All rights reserved.
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
#include <linux/init.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/debugfs.h>

#include <media/mc_common.h>
#include <media/csi.h>

#include "nvcsi/nvcsi.h"
#include "host1x/host1x.h"

static struct tpg_frmfmt *frmfmt_table;

static bool override_frmfmt;
module_param(override_frmfmt, bool, 0444);
MODULE_PARM_DESC(override_frmfmt, "override existing format table");

static int framerate = 30;
module_param(framerate, int, 0444);

/* PG generate 32 bit per nvcsi_clk:
 * clks_per_line = width * bits_per_pixel / 32
 * ((clks_per_line + hblank) * height + vblank) * fps * lanes = nvcsi_clk_freq
 *
 */
const struct tpg_frmfmt tegra19x_csi_tpg_frmfmt[] = {
	{{1280, 720}, V4L2_PIX_FMT_SRGGB10, 30, 0, 0},
	{{1920, 1080}, V4L2_PIX_FMT_SRGGB10, 30, 0, 0},
	{{3840, 2160}, V4L2_PIX_FMT_SRGGB10, 30,  0, 0},
	{{1280, 720}, V4L2_PIX_FMT_RGB32, 30, 0, 0},
	{{1920, 1080}, V4L2_PIX_FMT_RGB32, 30, 0, 0},
	{{3840, 2160}, V4L2_PIX_FMT_RGB32, 30, 0, 0},
};

#define TPG_PORT_IDX	0

static int tpg_debugfs_height_show(void *data, u64 *val)
{
	struct tegra_csi_channel *chan = data;
	struct tegra_csi_port *port = &chan->ports[TPG_PORT_IDX];

	mutex_lock(&chan->format_lock);
	*val = port->format.height;
	mutex_unlock(&chan->format_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(tpg_debugfs_height_fops,
			tpg_debugfs_height_show,
			NULL,
			"%lld\n");

static int tpg_debugfs_width_show(void *data, u64 *val)
{
	struct tegra_csi_channel *chan = data;
	struct tegra_csi_port *port = &chan->ports[TPG_PORT_IDX];

	mutex_lock(&chan->format_lock);
	*val = port->format.width;
	mutex_unlock(&chan->format_lock);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(tpg_debugfs_width_fops,
			tpg_debugfs_width_show,
			NULL,
			"%lld\n");


static void tpg_remove_debugfs(struct tegra_csi_device *csi)
{
	debugfs_remove_recursive(csi->debugdir);
	csi->debugdir = NULL;
}

static int tpg_create_debugfs(struct tegra_csi_device *csi)
{
	struct dentry *dir;
	struct tegra_csi_channel *chan;

	csi->debugdir = dir = debugfs_create_dir("tpg", NULL);
	if (dir == NULL)
		return -ENOMEM;

	chan = csi->tpg_start;
	list_for_each_entry_from(chan, &csi->csi_chans, list) {
		const struct tegra_channel *vi_chan =
				v4l2_get_subdev_hostdata(&chan->subdev);
		if (vi_chan->pg_mode) {
			const char *name = vi_chan->video->name;

			dev_dbg(csi->dev, "debugfs node installed %s\n", name);
			dir = debugfs_create_dir(name, csi->debugdir);
			if (!dir)
				goto error;

			if (!debugfs_create_file("height", 0444,
						dir, chan,
						&tpg_debugfs_height_fops))
				goto error;
			if (!debugfs_create_file("width", 0444,
						dir, chan,
						&tpg_debugfs_width_fops))
				goto error;
		}
	}

	return 0;
error:
	tpg_remove_debugfs(csi);
	return -ENOMEM;
}

static int __init tpg_probe_t19x(void)
{
	struct tegra_csi_device *mc_csi = tegra_get_mc_csi();
	struct tegra_mc_vi *mc_vi = tegra_get_mc_vi();
	int err = 0;
	int i = 0;
	unsigned int table_size = ARRAY_SIZE(tegra19x_csi_tpg_frmfmt);

	if (!mc_vi || !mc_csi)
		return -EINVAL;

	dev_info(mc_csi->dev, "%s\n", __func__);
	mc_vi->csi = mc_csi;
	/* Init CSI related media controller interface */
	frmfmt_table = devm_kzalloc(mc_csi->dev,
			table_size * sizeof(struct tpg_frmfmt), GFP_KERNEL);
	if (!frmfmt_table)
		return -ENOMEM;

	mc_csi->tpg_frmfmt_table_size = table_size;
	memcpy(frmfmt_table, tegra19x_csi_tpg_frmfmt,
		table_size * sizeof(struct tpg_frmfmt));

	if (override_frmfmt) {
		for (i = 0; i < table_size; i++)
			frmfmt_table[i].framerate = framerate;
	}
	mc_csi->tpg_frmfmt_table = frmfmt_table;

	err = tpg_csi_media_controller_init(mc_csi, TEGRA_VI_PG_PATCH);
	if (err)
		return -EINVAL;
	err = tpg_vi_media_controller_init(mc_vi, TEGRA_VI_PG_PATCH);
	if (err)
		goto vi_init_err;

	err = tpg_create_debugfs(mc_csi);
	if (err)
		goto debugfs_init_err;

	return err;
debugfs_init_err:
	tpg_remove_debugfs(mc_csi);
vi_init_err:
	tpg_csi_media_controller_cleanup(mc_csi);
	dev_err(mc_csi->dev, "%s error\n", __func__);
	return err;
}
static void __exit tpg_remove_t19x(void)
{
	struct tegra_csi_device *mc_csi = tegra_get_mc_csi();
	struct tegra_mc_vi *mc_vi = tegra_get_mc_vi();

	if (!mc_vi || !mc_csi)
		return;

	dev_info(mc_csi->dev, "%s\n", __func__);
	tpg_remove_debugfs(mc_csi);
	tpg_csi_media_controller_cleanup(mc_csi);
	tpg_vi_media_controller_cleanup(mc_vi);

	mc_csi->tpg_frmfmt_table = NULL;
	mc_csi->tpg_frmfmt_table_size = 0;
	devm_kfree(mc_csi->dev, frmfmt_table);
}

module_init(tpg_probe_t19x);
module_exit(tpg_remove_t19x);
MODULE_LICENSE("GPL v2");
