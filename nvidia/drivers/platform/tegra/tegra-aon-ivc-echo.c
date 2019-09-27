/*
 * Copyright (c) 2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/device.h>
#include <linux/mailbox_client.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/tegra-aon.h>

#define TX_BLOCK_PERIOD	100

struct tegra_aon_ivc_echo_data {
	struct mbox_client cl;
	struct mbox_chan *mbox;
	u32 frame_size;
	char *rx_data;
};

static ssize_t tegra_aon_ivc_echo_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct tegra_aon_ivc_echo_data *drvdata = dev_get_drvdata(dev);

	memcpy(buf, drvdata->rx_data, drvdata->frame_size);

	return drvdata->frame_size;
}

static ssize_t tegra_aon_ivc_echo_tx(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct tegra_aon_ivc_echo_data *drvdata = dev_get_drvdata(dev);
	struct tegra_aon_mbox_msg msg;
	int ret;

	if (count > drvdata->frame_size) {
		dev_err(dev, "Message is greater than the frame size %d\n",
							drvdata->frame_size);
		return -EINVAL;
	}

	msg.length = count;
	msg.data = (void *)buf;
	ret = mbox_send_message(drvdata->mbox, (void *)&msg);
	if (ret < 0)
		dev_err(dev, "mbox_send_message failed %d\n", ret);

	return count;
}

static const DEVICE_ATTR(data_channel, S_IRUGO | S_IWUSR,
				tegra_aon_ivc_echo_show, tegra_aon_ivc_echo_tx);

static void tegra_aon_ivc_echo_rx(struct mbox_client *cl, void *data)
{
	struct tegra_aon_mbox_msg *msg = data;
	struct tegra_aon_ivc_echo_data *drvdata = container_of(cl,
					struct tegra_aon_ivc_echo_data,
					cl);
	memcpy(drvdata->rx_data, msg->data, drvdata->frame_size);
}

static int tegra_aon_ivc_echo_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct tegra_aon_ivc_echo_data *drvdata;

	drvdata = devm_kzalloc(dev, sizeof(*drvdata), GFP_KERNEL);
	if (!drvdata)
		return -ENOMEM;
	dev_set_drvdata(dev, drvdata);
	drvdata->cl.dev = dev;
	drvdata->cl.tx_block = true;
	drvdata->cl.tx_tout = TX_BLOCK_PERIOD;
	drvdata->cl.knows_txdone = false;
	drvdata->cl.rx_callback = tegra_aon_ivc_echo_rx;
	drvdata->mbox = mbox_request_channel(&drvdata->cl, 0);
	if (IS_ERR(drvdata->mbox)) {
		ret = PTR_ERR(drvdata->mbox);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "mbox_request_channel failed. Error %d\n",
									ret);
		return ret;
	}

	drvdata->frame_size = mbox_get_max_txsize(drvdata->mbox);
	if (drvdata->frame_size > PAGE_SIZE) {
		dev_err(dev, "Faled to get a a valid frame_size\n");
          	ret = -EINVAL;
		goto out;
	}

	drvdata->rx_data = devm_kzalloc(dev, drvdata->frame_size, GFP_KERNEL);
	if (!drvdata->rx_data) {
		ret = -ENOMEM;
		goto out;
	}

	ret = device_create_file(dev, &dev_attr_data_channel);
	if (ret) {
		dev_err(dev, "Failed to create device file. Error %d\n", ret);
		goto out;
	}

	return ret;
out:
	mbox_free_channel(drvdata->mbox);
	return ret;
}

static int tegra_aon_ivc_echo_remove(struct platform_device *pdev)
{
	struct tegra_aon_ivc_echo_data *drvdata = dev_get_drvdata(&pdev->dev);

	device_remove_file(&pdev->dev, &dev_attr_data_channel);
	mbox_free_channel(drvdata->mbox);

	return 0;
}

static const struct of_device_id tegra_aon_ivc_echo_match[] = {
	{ .compatible = "nvidia,tegra186-aon-ivc-echo", },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_aon_ivc_echo_match);

static struct platform_driver tegra_aon_ivc_echo_driver = {
	.probe = tegra_aon_ivc_echo_probe,
	.remove = tegra_aon_ivc_echo_remove,
	.driver = {
		.name = "tegra-aon-ivc-echo",
		.of_match_table = tegra_aon_ivc_echo_match,
	},
};
module_platform_driver(tegra_aon_ivc_echo_driver);
