/*
 * vmipi.c
 *
 * Copyright (c) 2017, NVIDIA CORPORATION, All rights reserved.
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
#include <linux/device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/semaphore.h>
#include <linux/wait.h>
#include <linux/tegra-ivc.h>

#include "../mipi_cal.h"
#include "vmipi.h"

static const char *tegra_vmipi_cmd[] = {
	"TEGRA_VMIPI_CMD_CALIBRATE",
	"TEGRA_VMIPI_CMD_BIAS_PAD_ENABLE",
	"TEGRA_VMIPI_CMD_BIAS_PAD_DISABLE",
};

struct tegra_vmipi_ivc_context {
	struct device *dev;
	struct tegra_hv_ivc_cookie *cookie;
	wait_queue_head_t tx_wq;
	wait_queue_head_t rx_wq;
	struct mutex lock;
};
static struct tegra_vmipi_ivc_context *ctx;

static int tegra_vmipi_ivc_send(struct tegra_vmipi_cmd_msg *msg, int size)
{
	int ret = 0;

	if (!tegra_hv_ivc_can_write(ctx->cookie)) {
		ret = wait_event_timeout(ctx->tx_wq,
			tegra_hv_ivc_can_write(ctx->cookie),
			msecs_to_jiffies(500));
		if (!ret) {
			dev_err(ctx->dev,
				"%s timeout waiting for tx buffer for cmd %s\n",
				__func__, tegra_vmipi_cmd[msg->cmd]);
			return -ETIMEDOUT;
		}
	}

	ret = tegra_hv_ivc_write(ctx->cookie, msg, size);
	if (ret != size) {
		dev_err(ctx->dev,
			"%s: ivc send incorrect msg size %d for cmd %s\n",
			__func__, ret, tegra_vmipi_cmd[msg->cmd]);
		return -EIO;
	}

	return 0;
}

static int tegra_vmipi_ivc_rcv(struct tegra_vmipi_cmd_msg *msg, int size)
{
	int ret = 0, len;

	if (!tegra_hv_ivc_can_read(ctx->cookie)) {
		ret = wait_event_timeout(ctx->rx_wq,
			tegra_hv_ivc_can_read(ctx->cookie),
			msecs_to_jiffies(500));
		if (!ret)  {
			dev_err(ctx->dev,
				"%s timeout waiting for rx buffer for cmd %s\n",
				__func__, tegra_vmipi_cmd[msg->cmd]);
			ret = -ETIMEDOUT;
			goto out;
		}
		ret = 0;
	}

	len = tegra_hv_ivc_read(ctx->cookie, msg, size);
	if (len != size) {
		dev_err(ctx->dev,
			"%s: ivc read incorrect msg size %d for cmd %s\n",
			__func__, len, tegra_vmipi_cmd[msg->cmd]);
		ret = -EIO;
	}

out:
	if (tegra_hv_ivc_can_write(ctx->cookie))
		wake_up(&ctx->tx_wq);

	return ret ? ret : msg->ret;
}

static irqreturn_t tegra_vmipi_ivc_isr(int irq, void *data)
{
	struct tegra_vmipi_ivc_context *ictx =
		(struct tegra_vmipi_ivc_context *)data;

	if (tegra_hv_ivc_channel_notified(ictx->cookie))
		return IRQ_HANDLED;

	wake_up(&ictx->rx_wq);
	return IRQ_HANDLED;
}

static int tegra_vmipi_ivc_send_rcv(struct tegra_vmipi_cmd_msg *msg, int size)
{
	int err;

	if (!msg || !size)
		return -EINVAL;

	mutex_lock(&ctx->lock);

	err = tegra_vmipi_ivc_send(msg, size);
	if (err)
		goto out;

	err = tegra_vmipi_ivc_rcv(msg, size);

out:
	mutex_unlock(&ctx->lock);

	return err;
}

int tegra_vmipi_calibration(struct tegra_mipi *mipi, int lanes)
{
	struct tegra_vmipi_cmd_msg msg;
	int size = sizeof(msg);
	struct tegra_vmipi_calibrate_params *p;

	if (!ctx)
		return -EINVAL;

	msg.cmd = TEGRA_VMIPI_CMD_CALIBRATE;
	p = &msg.params.calibrate;
	p->lanes = lanes;
	return tegra_vmipi_ivc_send_rcv(&msg, size);
}

int tegra_vmipi_bias_pad_enable(struct tegra_mipi *mipi)
{
	struct tegra_vmipi_cmd_msg msg;
	int size = sizeof(msg);

	if (!ctx)
		return -EINVAL;

	msg.cmd = TEGRA_VMIPI_CMD_BIAS_PAD_ENABLE;
	return tegra_vmipi_ivc_send_rcv(&msg, size);
}

int tegra_vmipi_bias_pad_disable(struct tegra_mipi *mipi)
{
	struct tegra_vmipi_cmd_msg msg;
	int size = sizeof(msg);

	if (!ctx)
		return -EINVAL;

	msg.cmd = TEGRA_VMIPI_CMD_BIAS_PAD_DISABLE;
	return tegra_vmipi_ivc_send_rcv(&msg, size);
}

int tegra_vmipi_init(struct platform_device *pdev)
{
	static struct tegra_vmipi_ivc_context *context;
	struct device_node *np, *ivcq_np;
	struct tegra_hv_ivc_cookie *cookie;
	int err, ivcq;

	if (!pdev)
		return -ENODEV;

	context = devm_kzalloc(&pdev->dev, sizeof(*context), GFP_KERNEL);
	if (!context)
		return -ENOMEM;

	context->dev = &pdev->dev;

	np = pdev->dev.of_node;
	ivcq_np = of_parse_phandle(np, "ivc_queue", 0);
	if (!ivcq_np) {
		dev_err(&pdev->dev, "Fail to find vmipi ivc queue\n");
		return -ENOMEM;
	}

	err = of_property_read_u32_index(np, "ivc_queue", 1, &ivcq);
	if (err) {
		dev_err(&pdev->dev, "Fail to read vmipi ivc queue value\n");
		of_node_put(ivcq_np);
		return -EINVAL;
	}
	of_node_put(ivcq_np);

	cookie = tegra_hv_ivc_reserve(ivcq_np, ivcq, NULL);
	if (IS_ERR_OR_NULL(cookie)) {
		if (cookie == ERR_PTR(-EPROBE_DEFER))
			return -EPROBE_DEFER;

		dev_err(&pdev->dev,
			"Fail to reserve vmipi ivc queue %d\n", ivcq);
		return PTR_ERR(cookie);
	}
	context->cookie = cookie;

	init_waitqueue_head(&context->tx_wq);
	init_waitqueue_head(&context->rx_wq);

	mutex_init(&context->lock);

	tegra_hv_ivc_channel_reset(context->cookie);

	err = devm_request_irq(&pdev->dev, cookie->irq, tegra_vmipi_ivc_isr, 0,
		"mipi-virt", context);
	if (err) {
		dev_err(&pdev->dev, "Fail to request vmipi ivc irq %d\n",
			cookie->irq);
		goto fail;
	}

	ctx = context;
	return 0;

fail:
	tegra_hv_ivc_unreserve(context->cookie);
	return err;
}

void tegra_vmipi_deinit(void)
{
	if (ctx)
		tegra_hv_ivc_unreserve(ctx->cookie);
}
