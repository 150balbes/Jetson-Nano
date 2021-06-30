/*
 * tegra-hv-xhci-debug: Tegra Hypervisor XHCI server debug
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <soc/tegra/chip-id.h>
#include <linux/wait.h>

#define DRV_NAME "tegra_hv_xhci_debug"

#include <linux/tegra-ivc.h>

/* frame format is
 * 0000: <size>
 * 0004: <flags>
 * 0008: data
 */

struct msg_header {
	uint32_t size;
	uint32_t flag;
};
#define HDR_SIZE			(sizeof(struct msg_header))
#define FW_LOG_FILE_OPENED		(1)

struct tegra_hv_xhci_debug {
	struct platform_device *pdev;
	struct tegra_hv_ivc_cookie *ivck;
	struct dentry *debugfs_dir;
	struct dentry *log_file;
	unsigned long flags;

	wait_queue_head_t read_wait;

	spinlock_t lock;

	void *rx_frame;
	u8 *rx_buf;
	void *tx_frame;
	u8 *tx_buf;
	void *buf;
	int rx_cnt;
};

static ssize_t fw_log_file_read(struct file *file, char __user *buf,
		size_t count, loff_t *offp)
{
	struct tegra_hv_xhci_debug *tegra = file->private_data;
	struct device *dev = &tegra->pdev->dev;
	struct msg_header *header;
	int rc;
	size_t n = 0;
	int s;

	dev_dbg(dev, "%s can_read %d rx_cnt %d\n",
		__func__, tegra_hv_ivc_can_read(tegra->ivck), tegra->rx_cnt);

	while (!tegra_hv_ivc_can_read(tegra->ivck) && !tegra->rx_cnt) {
		dev_dbg(dev, "%s can_read %d rx_cnt %d\n",
			__func__, tegra_hv_ivc_can_read(tegra->ivck),
			tegra->rx_cnt);

		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN; /* non-blocking read */

		dev_dbg(dev, "%s: nothing to read\n", __func__);

		if (wait_event_interruptible(tegra->read_wait,
			tegra_hv_ivc_can_read(tegra->ivck)))
			return -ERESTARTSYS;
	}

	while (count > 0) {
		if (!tegra->rx_cnt) {
			if (!tegra_hv_ivc_can_read(tegra->ivck))
				break;

			rc = tegra_hv_ivc_read(tegra->ivck, tegra->rx_frame,
					tegra->ivck->frame_size);

			if (rc < 0) {
				dev_err(dev, "ivc_read failied %d\n", rc);
				return rc;
			}
			header = tegra->rx_frame;

			dev_dbg(dev, "%s received %u bytes\n", __func__,
				header->size);
			tegra->rx_cnt = header->size;
		}

		s = min_t(int, count, tegra->rx_cnt);
		if (s > 0) {
			if (copy_to_user(&buf[n], tegra->rx_buf, s)) {
				dev_warn(dev, "copy_to_user failed\n");
				return -EFAULT;
			}

			tegra->rx_cnt -= s;
			tegra->rx_buf += s;
			if (tegra->rx_cnt == 0)
				tegra->rx_buf = tegra->rx_frame + HDR_SIZE;

			count -= s;
			n += s;
		} else
			break;
	}

	dev_dbg(dev, "%s: %zu bytes\n", __func__, n);

	return n;
}

static int fw_log_file_open(struct inode *inode, struct file *file)
{
	struct tegra_hv_xhci_debug *tegra;

	file->private_data = inode->i_private;
	tegra = file->private_data;

	if (test_and_set_bit(FW_LOG_FILE_OPENED, &tegra->flags)) {
		dev_info(&tegra->pdev->dev, "%s: already opened\n", __func__);
		return -EBUSY;
	}

	return 0;
}

static int fw_log_file_close(struct inode *inode, struct file *file)
{

	struct tegra_hv_xhci_debug *tegra = file->private_data;

	clear_bit(FW_LOG_FILE_OPENED, &tegra->flags);

	return 0;
}

static const struct file_operations firmware_log_fops = {
		.open		= fw_log_file_open,
		.release	= fw_log_file_close,
		.read		= fw_log_file_read,
		.owner		= THIS_MODULE,
};

static void tegra_hv_xhci_debug_debugfs_init(struct tegra_hv_xhci_debug *tegra)
{
	struct device *dev = &tegra->pdev->dev;

	tegra->debugfs_dir = debugfs_create_dir("tegra_hv_xhci_debug", NULL);
	if (IS_ERR_OR_NULL(tegra->debugfs_dir)) {
		tegra->debugfs_dir = NULL;
		dev_warn(dev, "debugfs_create_dir() for tegra_xhci failed\n");
		return;
	}

	tegra->log_file = debugfs_create_file("firmware_log", 0444,
		tegra->debugfs_dir, tegra, &firmware_log_fops);

	if ((!tegra->log_file) || (tegra->log_file == ERR_PTR(-ENODEV))) {
		dev_warn(dev, "debugfs_create_file() failed\n");
		tegra->log_file = NULL;
	}
}

static
void tegra_hv_xhci_debug_debugfs_deinit(struct tegra_hv_xhci_debug *tegra)
{
	debugfs_remove(tegra->debugfs_dir);
	tegra->debugfs_dir = NULL;
}

static irqreturn_t tegra_hv_xhci_debug_irq(int irq, void *data)
{
	struct tegra_hv_xhci_debug *tegra = data;
	struct device *dev = &tegra->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);

	spin_lock(&tegra->lock);

	/* until this function returns 0, the channel is unusable */
	if (tegra_hv_ivc_channel_notified(tegra->ivck) != 0)
		dev_info(dev, "tegra_hv_ivc_channel_notified failed\n");


	wake_up_interruptible(&tegra->read_wait);

	spin_unlock(&tegra->lock);

	return IRQ_HANDLED;
}

static int tegra_hv_xhci_debug_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np, *hv_np;
	struct tegra_hv_xhci_debug *tegra;
	int ret;
	u32 id;

	if (!is_tegra_hypervisor_mode()) {
		dev_info(dev, "Hypervisor is not present\n");
		return -ENODEV;
	}

	np = dev->of_node;
	if (!np) {
		dev_err(dev, "No OF data\n");
		return -EINVAL;
	}

	hv_np = of_parse_phandle(np, "ivc", 0);
	if (!hv_np) {
		dev_err(dev, "Failed to parse phandle of ivc prop\n");
		return -EINVAL;
	}

	ret = of_property_read_u32_index(np, "ivc", 1, &id);
	if (ret) {
		dev_err(dev, "Failed to read IVC property ID\n");
		of_node_put(hv_np);
		return ret;
	}

	tegra = devm_kzalloc(&pdev->dev, sizeof(*tegra), GFP_KERNEL);
	if (!tegra)
		return -ENOMEM;

	tegra->ivck = tegra_hv_ivc_reserve(hv_np, id, NULL);

	of_node_put(hv_np);

	if (IS_ERR_OR_NULL(tegra->ivck)) {
		dev_err(dev, "Failed to reserve IVC channel %d\n", id);
		ret = PTR_ERR(tegra->ivck);
		tegra->ivck = NULL;
		return ret;
	}

	/* make sure the frame size is sufficient */
	if (tegra->ivck->frame_size <= HDR_SIZE) {
		dev_err(dev, "frame size too small to support COMM\n");
		ret = -EINVAL;
		goto out_unreserve;
	}

	dev_info(dev, "Reserved IVC channel #%d - frame_size=%d irq %d\n",
			id, tegra->ivck->frame_size, tegra->ivck->irq);

	/* allocate temporary frames */
	tegra->buf = devm_kzalloc(dev, tegra->ivck->frame_size * 2, GFP_KERNEL);
	if (!tegra->buf) {
		ret = -ENOMEM;
		goto out_unreserve;
	}
	tegra->rx_frame = tegra->buf;
	tegra->rx_buf = tegra->rx_frame + HDR_SIZE;
	tegra->tx_frame = tegra->rx_frame + tegra->ivck->frame_size;
	tegra->tx_buf = tegra->tx_frame + HDR_SIZE;

	init_waitqueue_head(&tegra->read_wait);

	tegra->pdev = pdev;
	platform_set_drvdata(pdev, tegra);

	/*
	 * start the channel reset process asynchronously. until the reset
	 * process completes, any attempt to use the ivc channel will return
	 * an error (e.g., all transmits will fail.)
	 */
	tegra_hv_ivc_channel_reset(tegra->ivck);

	spin_lock_init(&tegra->lock);
	ret = devm_request_irq(dev, tegra->ivck->irq,
			tegra_hv_xhci_debug_irq, 0,
			dev_name(dev), tegra);
	if (ret) {
		dev_err(dev, "unable to request irq=%d\n", tegra->ivck->irq);
		return ret;
	}

	tegra_hv_xhci_debug_debugfs_init(tegra);

	dev_info(dev, "ready\n");

	return 0;

out_unreserve:
	tegra_hv_ivc_unreserve(tegra->ivck);
	return ret;
}

static int tegra_hv_xhci_debug_remove(struct platform_device *pdev)
{
	struct tegra_hv_xhci_debug *tegra = platform_get_drvdata(pdev);

	tegra_hv_ivc_unreserve(tegra->ivck);

	tegra_hv_xhci_debug_debugfs_deinit(tegra);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id tegra_hv_xhci_debug_match[] = {
	{ .compatible = "nvidia,tegra-hv-xhci-debug", },
	{},
};
MODULE_DEVICE_TABLE(of, tegra_hv_xhci_debug_match);
#endif /* CONFIG_OF */

static struct platform_driver tegra_hv_xhci_debug_platform_driver = {
	.probe	= tegra_hv_xhci_debug_probe,
	.remove	= tegra_hv_xhci_debug_remove,
	.driver	= {
		.name		= DRV_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(tegra_hv_xhci_debug_match),
	},
};

static int tegra_hv_xhci_debug_init(void)
{
	return platform_driver_register(&tegra_hv_xhci_debug_platform_driver);
}

static void tegra_hv_xhci_debug_exit(void)
{
	platform_driver_unregister(&tegra_hv_xhci_debug_platform_driver);
}

module_init(tegra_hv_xhci_debug_init);
module_exit(tegra_hv_xhci_debug_exit);

MODULE_AUTHOR("JC Kuo <jckuo@nvidia.com>");
MODULE_DESCRIPTION("Tegra Hyperisor XHCI Server Debug");
MODULE_LICENSE("GPL");
