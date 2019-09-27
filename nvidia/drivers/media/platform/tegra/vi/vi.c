/*
 * drivers/video/tegra/host/vi/vi.c
 *
 * Tegra Graphics Host VI
 *
 * Copyright (c) 2012-2018, NVIDIA CORPORATION. All rights reserved.
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
#include <linux/resource.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/clk/tegra.h>
#include <soc/tegra/chip-id.h>
#include <linux/tegra_pm_domains.h>
#include <linux/debugfs.h>
#include <linux/slab.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegra_camera_dev_mfi.h>
#include <media/vi.h>
#include <media/tegra_camera_platform.h>

#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "t210/t210.h"
#include "vi/vi_irq.h"
#include "camera/vi/vi2_fops.h"
#include "camera/csi/csi2_fops.h"

#define MAX_DEVID_LENGTH	16
#define TEGRA_VI_NAME		"tegra_vi"

static struct vi *tegra_vi;

struct vi *tegra_vi_get(void)
{
	return tegra_vi;
}
EXPORT_SYMBOL(tegra_vi_get);

static struct tegra_t210_vi_data t21_vi_data = {
	.info = (struct nvhost_device_data *)&t21_vi_info,
	.vi_fops = &vi2_fops,
	.csi_fops = &csi2_fops,
};

static struct of_device_id tegra_vi_of_match[] = {
	{ .compatible = "nvidia,tegra210-vi",
		.data = (struct tegra_t210_vi_data *)&t21_vi_data },
	{ },
};

static struct i2c_camera_ctrl *i2c_ctrl;

static void (*mfi_callback)(void *);
static struct mfi_cb_arg *mfi_callback_arg;
static DEFINE_MUTEX(vi_isr_lock);

int tegra_vi_register_mfi_cb(callback cb, void *cb_arg)
{
	mutex_lock(&vi_isr_lock);
	if (mfi_callback || mfi_callback_arg) {
		pr_err("cb already registered\n");
		mutex_unlock(&vi_isr_lock);
		return -1;
	}

	mfi_callback = cb;
	mfi_callback_arg = (struct  mfi_cb_arg *)cb_arg;
	mutex_unlock(&vi_isr_lock);

	return 0;
}
EXPORT_SYMBOL(tegra_vi_register_mfi_cb);

int tegra_vi_unregister_mfi_cb(void)
{
	mutex_lock(&vi_isr_lock);
	mfi_callback = NULL;
	mfi_callback_arg = NULL;
	mutex_unlock(&vi_isr_lock);

	return 0;
}
EXPORT_SYMBOL(tegra_vi_unregister_mfi_cb);

struct tegra_mfi_chan {
	struct work_struct mfi_cb_work;
	struct rcu_head rcu;
	u8 channel;
	void *priv;
};

struct tegra_vi_mfi_ctx {
	u8 num_channels;
	struct workqueue_struct *mfi_workqueue;
	struct tegra_mfi_chan __rcu *mfi_chans;
};

static void vi_mfi_worker(struct work_struct *vi_work)
{
	struct tegra_mfi_chan *mfi_chan =
		container_of(vi_work, struct tegra_mfi_chan, mfi_cb_work);
	struct tegra_mc_vi *vi = tegra_get_mc_vi();

	mutex_lock(&vi_isr_lock);
	tegra_vi_mfi_work(vi, mfi_chan->channel);
	mutex_unlock(&vi_isr_lock);
}

int tegra_vi_mfi_event_notify(struct tegra_vi_mfi_ctx *mfi_ctx, u8 channel)
{
	struct tegra_mfi_chan *chan;

	if (!mfi_ctx) {
		pr_err("Invalid mfi_ctx\n");
		return -EINVAL;
	}

	if (channel > mfi_ctx->num_channels-1) {
		pr_err("Invalid mfi channel\n");
		return -EINVAL;
	}

	rcu_read_lock();
	chan = rcu_dereference(mfi_ctx->mfi_chans);
	queue_work(mfi_ctx->mfi_workqueue, &chan[channel].mfi_cb_work);
	rcu_read_unlock();

	return 0;
}
EXPORT_SYMBOL(tegra_vi_mfi_event_notify);

bool tegra_vi_has_mfi_callback(void)
{
	bool ret;

	mutex_lock(&vi_isr_lock);
	ret = mfi_callback ? true : false;
	mutex_unlock(&vi_isr_lock);

	return ret;
}
EXPORT_SYMBOL(tegra_vi_has_mfi_callback);

int tegra_vi_init_mfi(struct tegra_vi_mfi_ctx **pmfi_ctx, u8 num_channels)
{
	u8 i;
	struct tegra_mfi_chan *chan;
	struct tegra_vi_mfi_ctx *mfi_ctx;

	if (!pmfi_ctx) {
		pr_err("mfi_ctx is invalid\n");
		return -EINVAL;
	}

	mfi_ctx = kzalloc(sizeof(*mfi_ctx), GFP_KERNEL);
	if (unlikely(mfi_ctx == NULL))
		return -ENOMEM;

	/* create workqueue for mfi callback */
	mfi_ctx->mfi_workqueue = alloc_workqueue("mfi_workqueue",
					WQ_HIGHPRI | WQ_UNBOUND, 1);
	if (!mfi_ctx->mfi_workqueue) {
		pr_err("Failed to allocated mfi_workqueue");
		tegra_vi_deinit_mfi(&mfi_ctx);
		return -ENOMEM;
	}

	chan = kcalloc(num_channels, sizeof(*chan), GFP_KERNEL);
	if (unlikely(chan == NULL)) {
		tegra_vi_deinit_mfi(&mfi_ctx);
		return -ENOMEM;
	}

	mfi_ctx->num_channels = num_channels;

	/* Init mfi callback work */
	for (i = 0; i < num_channels; i++) {
		INIT_WORK(&chan[i].mfi_cb_work, vi_mfi_worker);
		chan[i].channel = i;
	}

	rcu_assign_pointer(mfi_ctx->mfi_chans, chan);

	*pmfi_ctx = mfi_ctx;

	return 0;
}
EXPORT_SYMBOL(tegra_vi_init_mfi);

void tegra_vi_deinit_mfi(struct tegra_vi_mfi_ctx **pmfi_ctx)
{
	struct tegra_vi_mfi_ctx *mfi_ctx;
	struct tegra_mfi_chan *chan;

	if (!pmfi_ctx || !*pmfi_ctx)
		return;

	mfi_ctx = *pmfi_ctx;

	flush_workqueue(mfi_ctx->mfi_workqueue);
	destroy_workqueue(mfi_ctx->mfi_workqueue);

	rcu_read_lock();
	chan = rcu_dereference(mfi_ctx->mfi_chans);
	rcu_read_unlock();
	RCU_INIT_POINTER(mfi_ctx->mfi_chans, NULL);
	kfree_rcu(chan, rcu);

	kfree(mfi_ctx);
	mfi_ctx = NULL;
}
EXPORT_SYMBOL(tegra_vi_deinit_mfi);

static int vi_out_show(struct seq_file *s, void *unused)
{
	struct vi *vi = s->private;

	seq_printf(s, "vi overflow: %u\n",
		atomic_read(&(vi->vi_out.overflow)));

	return 0;
}

static int vi_out_open(struct inode *inode, struct file *file)
{
	return single_open(file, vi_out_show, inode->i_private);
}

static const struct file_operations vi_out_fops = {
	.open		= vi_out_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static void vi_remove_debugfs(struct vi *vi)
{
	debugfs_remove_recursive(vi->debugdir);
	vi->debugdir = NULL;
}

static void vi_create_debugfs(struct vi *vi)
{
	struct dentry *ret;
	char tegra_vi_name[20];
	char debugfs_file_name[20];


	snprintf(tegra_vi_name, sizeof(tegra_vi_name), "%s", TEGRA_VI_NAME);

	vi->debugdir = debugfs_create_dir(tegra_vi_name, NULL);
	if (!vi->debugdir) {
		dev_err(&vi->ndev->dev,
			"%s: failed to create %s directory",
			__func__, tegra_vi_name);
		goto create_debugfs_fail;
	}

	snprintf(debugfs_file_name, sizeof(debugfs_file_name), "%s", "vi_out");

	ret = debugfs_create_file(debugfs_file_name, S_IRUGO,
			vi->debugdir, vi, &vi_out_fops);
	if (!ret) {
		dev_err(&vi->ndev->dev,
		"%s: failed to create %s", __func__, debugfs_file_name);
		goto create_debugfs_fail;
	}

	return;

create_debugfs_fail:
	dev_err(&vi->ndev->dev, "%s: could not create debugfs", __func__);
	vi_remove_debugfs(vi);
}

static int nvhost_vi_slcg_handler(struct notifier_block *nb,
		unsigned long action, void *data)
{
	int ret = 0;

	struct nvhost_device_data *pdata =
		container_of(nb, struct nvhost_device_data,
			toggle_slcg_notifier);
	struct vi *tegra_vi = (struct vi *)pdata->private_data;
	struct tegra_csi_device *csi = &tegra_vi->csi;

	if (tegra_vi->mc_vi.pg_mode)
		return NOTIFY_OK;

	/* Make CSI sourced from PLL_D */
	csi_source_from_plld();
	ret = clk_prepare_enable(csi->plld);
	if (ret) {
		dev_err(tegra_vi->dev, "pll_d enable failed");
		return ret;
	}

	ret = clk_prepare_enable(csi->plld_dsi);
	if (ret) {
		dev_err(tegra_vi->dev, "pll_d enable failed");
		goto plld_dsi_err;
	}

	udelay(1);

	/* Disable PLL_D */
	clk_disable_unprepare(csi->plld_dsi);
	clk_disable_unprepare(csi->plld);

	/* Restore CSI source */
	csi_source_from_brick();
	return NOTIFY_OK;

plld_dsi_err:
	clk_disable_unprepare(csi->plld);
	csi_source_from_brick();

	return NOTIFY_OK;
}

static int vi_probe(struct platform_device *dev)
{
	int err = 0;
	struct nvhost_device_data *pdata = NULL;
	struct tegra_t210_vi_data *data = NULL;
	u8 num_channels;
	struct tegra_camera_dev_info vi_info;

	if (dev->dev.of_node) {
		const struct of_device_id *match;

		match = of_match_device(tegra_vi_of_match, &dev->dev);
		if (match) {
			data = (struct tegra_t210_vi_data *)match->data;
			pdata = data->info;
			dev->dev.platform_data = pdata;
		}
		/* DT initializes it to -1, use below WAR to set correct value.
		 * TODO: Once proper fix for dev-id goes in, remove it.
		 */
		dev->id = dev->dev.id;
	} else
		pdata = (struct nvhost_device_data *)dev->dev.platform_data;

	WARN_ON(!pdata);
	if (!pdata) {
		dev_info(&dev->dev, "no platform data\n");
		return -ENODATA;
	}

	pdata->pdev = dev;
	mutex_init(&pdata->lock);
	platform_set_drvdata(dev, pdata);

	dev_info(&dev->dev, "%s: ++\n", __func__);

	tegra_vi = devm_kzalloc(&dev->dev, sizeof(struct vi), GFP_KERNEL);
	if (!tegra_vi)
		return -ENOMEM;

	tegra_vi->ndev = dev;
	tegra_vi->dev = &dev->dev;
	err = nvhost_client_device_get_resources(dev);
	if (err)
		goto vi_probe_fail;

	num_channels = 6;

	err = tegra_vi_init_mfi(&tegra_vi->mfi_ctx, num_channels);
	if (err)
		goto vi_probe_fail;

	if (!pdata->aperture[0]) {
		dev_err(&dev->dev, "%s: failed to map register base\n",
				__func__);
		return -ENXIO;
	}

	/* call vi_intr_init and stats_work */
	INIT_WORK(&tegra_vi->stats_work, vi_stats_worker);

	err = vi_intr_init(tegra_vi);
	if (err)
		goto vi_mfi_init_fail;

	vi_create_debugfs(tegra_vi);

	i2c_ctrl = pdata->private_data;
	pdata->private_data = tegra_vi;

	/* Create I2C Devices according to settings from board file */
	if (i2c_ctrl && i2c_ctrl->new_devices)
		i2c_ctrl->new_devices(dev);

	tegra_vi->reg = regulator_get(&tegra_vi->ndev->dev, "avdd_dsi_csi");
	if (IS_ERR(tegra_vi->reg)) {
		err = PTR_ERR(tegra_vi->reg);
		if (err == -ENODEV)
			dev_info(&tegra_vi->ndev->dev,
				"%s: no regulator device\n", __func__);
		else
			dev_err(&tegra_vi->ndev->dev,
				"%s: couldn't get regulator\n", __func__);
		tegra_vi->reg = NULL;
		if (tegra_platform_is_silicon())
			goto camera_i2c_unregister;
	}

	tegra_vi->csi.plld = devm_clk_get(&dev->dev, "pll_d");
	if (IS_ERR(tegra_vi->csi.plld)) {
		dev_err(&dev->dev, "Fail to get pll_d\n");
		return PTR_ERR(tegra_vi->csi.plld);
	}
	tegra_vi->csi.plld_dsi = devm_clk_get(&dev->dev, "pll_d_dsi_out");
	if (IS_ERR(tegra_vi->csi.plld_dsi)) {
		dev_err(&dev->dev, "Fail to get pll_d_dsi_out\n");
		return PTR_ERR(tegra_vi->csi.plld_dsi);
	}
#ifdef CONFIG_TEGRA_CAMERA
	tegra_vi->camera = tegra_camera_register(dev);
	if (!tegra_vi->camera) {
		dev_err(&dev->dev, "%s: can't register tegra_camera\n",
				__func__);
		goto vi_regulator_put;
	}
#endif

	if (pdata->slcg_notifier_enable) {
		pdata->toggle_slcg_notifier.notifier_call =
			&nvhost_vi_slcg_handler;
	}

	nvhost_module_init(dev);

	err = nvhost_client_device_init(dev);
	if (err)
		goto camera_unregister;

	tegra_vi->mc_vi.vi = tegra_vi;
	tegra_vi->mc_vi.csi = &tegra_vi->csi;
	tegra_vi->mc_vi.reg = tegra_vi->reg;
	tegra_vi->mc_vi.fops = data->vi_fops;
	tegra_vi->csi.fops = data->csi_fops;
	err = tegra_csi_media_controller_init(&tegra_vi->csi, dev);
	if (err)
		goto vi_mc_init_error;

	err = tegra_vi_media_controller_init(&tegra_vi->mc_vi, dev);
	if (err)
		goto vi_mc_init_error;

	memset(&vi_info, 0, sizeof(vi_info));
	vi_info.pdev = dev;
	vi_info.hw_type = HWTYPE_VI;
	/* 4 uS latency allowed for memory freq switch */
	vi_info.memory_latency = 4;
	err = tegra_camera_device_register(&vi_info, tegra_vi);
	if (err)
		goto vi_probe_fail;

	return 0;

vi_mc_init_error:
	nvhost_client_device_release(dev);
camera_unregister:
#ifdef CONFIG_TEGRA_CAMERA
	tegra_camera_unregister(tegra_vi->camera);
vi_regulator_put:
#endif

	regulator_put(tegra_vi->reg);
	tegra_vi->reg = NULL;

camera_i2c_unregister:
	if (i2c_ctrl && i2c_ctrl->remove_devices)
		i2c_ctrl->remove_devices(dev);
	pdata->private_data = i2c_ctrl;
vi_mfi_init_fail:
	tegra_vi_deinit_mfi(&tegra_vi->mfi_ctx);
vi_probe_fail:
	dev_err(&dev->dev, "%s: failed\n", __func__);
	return err;
}

static int __exit vi_remove(struct platform_device *dev)
{
#ifdef CONFIG_TEGRA_CAMERA
	int err = 0;
#endif
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	struct vi *tegra_vi = (struct vi *)pdata->private_data;

#ifdef CONFIG_PM
	if (atomic_read(&dev->dev.power.usage_count) > 0)
		return -EBUSY;
#endif

	dev_info(&dev->dev, "%s: ++\n", __func__);

	tegra_vi_deinit_mfi(&tegra_vi->mfi_ctx);

	vi_remove_debugfs(tegra_vi);

	tegra_vi_media_controller_cleanup(&tegra_vi->mc_vi);

	nvhost_client_device_release(dev);

	vi_intr_free(tegra_vi);

	pdata->aperture[0] = NULL;
#ifdef CONFIG_TEGRA_CAMERA
	err = tegra_camera_unregister(tegra_vi->camera);
	if (err)
		return err;
#endif

#ifdef CONFIG_PM_GENERIC_DOMAINS
	tegra_pd_remove_device(&dev->dev);
#endif

	regulator_put(tegra_vi->reg);
	tegra_vi->reg = NULL;

	/* Remove I2C Devices according to settings from board file */
	if (i2c_ctrl && i2c_ctrl->remove_devices)
		i2c_ctrl->remove_devices(dev);

	pdata->private_data = i2c_ctrl;

	return 0;
}

static struct platform_driver vi_driver = {
	.probe = vi_probe,
	.remove = __exit_p(vi_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "vi",
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
#ifdef CONFIG_OF
		.of_match_table = tegra_vi_of_match,
#endif
	}
};

static struct of_device_id tegra_vi_domain_match[] = {
	{.compatible = "nvidia,tegra210-ve-pd",
	 .data = (struct nvhost_device_data *)&t21_vi_info},
	{},
};

static int __init vi_init(void)
{
	int ret;

	ret = nvhost_domain_init(tegra_vi_domain_match);
	if (ret)
		return ret;

	return platform_driver_register(&vi_driver);
}

static void __exit vi_exit(void)
{
	platform_driver_unregister(&vi_driver);
}

late_initcall(vi_init);
module_exit(vi_exit);
MODULE_LICENSE("GPL v2");
