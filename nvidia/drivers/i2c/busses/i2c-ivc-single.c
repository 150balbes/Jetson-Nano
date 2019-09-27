/*
 * drivers/i2c/busses/i2c-ivc-single.c
 *
 * Copyright (C) 2017 NVIDIA Corporation. All rights reserved.
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

#include "i2c-ivc-single.h"

#include <linux/debugfs.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>

#include <soc/tegra/tegra-ivc-rpc.h>

#include "soc/tegra/camrtc-i2c-common.h"
#include "i2c-rtcpu-common.h"

/*
 * I2C IVC Single driver internal data structure
 */

#define MAX_DEVS    4

#define I2C_CAMRTC_RPC_IVC_SINGLE_TIMEOUT_MS   250

struct tegra_i2c_ivc_single_dev {
	/* IVC RPC */
	struct tegra_ivc_channel *chan;
	struct mutex inuse;

	bool is_added;
	bool is_online;

	/* I2C device behind this IVC */
	u32 bus_id;
	u32 reg_base;
	u32 bus_clk_rate;

	/* statistics */
	struct {
		unsigned int xfer_requests;
		unsigned int total_bytes;
		unsigned int reads, read_bytes;
		unsigned int writes, write_bytes;
		unsigned int errors;
	} stat;

	/* RPC call for I2C_REQUEST_SINGLE */
	struct tegra_ivc_rpc_call_param rpc_i2c_req;
	u8 rpc_i2c_req_buf[CAMRTC_I2C_REQUEST_MAX_LEN];
	struct camrtc_rpc_i2c_response rpc_i2c_rsp;
};

static DEFINE_MUTEX(g_i2c_ivc_lock);
static struct tegra_i2c_ivc_single_dev *g_i2c_ivc_devs[MAX_DEVS];

u32 tegra_i2c_get_clk_freq(struct device_node *np)
{
	int ret;
	u32 bus_clk_rate;

	ret = of_property_read_u32(np, "clock-frequency",
			&bus_clk_rate);
	if (ret)
		bus_clk_rate = 100000; /* default clock rate */

	return bus_clk_rate;
}
EXPORT_SYMBOL(tegra_i2c_get_clk_freq);

u32 tegra_i2c_get_reg_base(struct device_node *np)
{
	int ret;
	u32 reg_base;

	ret = of_property_read_u32_index(np, "reg", 1, &reg_base);

	BUG_ON(ret < 0);

	return reg_base;
}
EXPORT_SYMBOL(tegra_i2c_get_reg_base);

/*
 * I2C interface
 */
static struct tegra_i2c_ivc_single_dev *tegra_i2c_ivc_get_dev(u32 reg_base)
{
	size_t i;
	struct tegra_i2c_ivc_single_dev *sdev = NULL;

	mutex_lock(&g_i2c_ivc_lock);

	/* Find an IVC channel corresponding to base */
	for (i = 0; i < ARRAY_SIZE(g_i2c_ivc_devs); ++i) {
		if (!g_i2c_ivc_devs[i])
			continue;

		if (g_i2c_ivc_devs[i]->reg_base == reg_base) {
			sdev = g_i2c_ivc_devs[i];
			mutex_lock(&sdev->inuse);
			break;
		}
	}

	mutex_unlock(&g_i2c_ivc_lock);

	return sdev;
}

static void tegra_i2c_ivc_put_dev(struct tegra_i2c_ivc_single_dev *sdev)
{
	if (sdev)
		mutex_unlock(&sdev->inuse);
}

static int tegra_i2c_ivc_register(struct tegra_i2c_ivc_single_dev *sdev)
{
	size_t i;
	int ret = -EBUSY;

	mutex_lock(&g_i2c_ivc_lock);

	for (i = 0; i < ARRAY_SIZE(g_i2c_ivc_devs); ++i) {
		if (!g_i2c_ivc_devs[i]) {
			g_i2c_ivc_devs[i] = sdev;
			ret = 0;
			break;
		}
	}

	mutex_unlock(&g_i2c_ivc_lock);

	return ret;
}

static void tegra_i2c_ivc_unregister(struct tegra_i2c_ivc_single_dev *sdev)
{
	size_t i;

	mutex_lock(&g_i2c_ivc_lock);

	for (i = 0; i < ARRAY_SIZE(g_i2c_ivc_devs); ++i) {
		if (g_i2c_ivc_devs[i] == sdev) {
			g_i2c_ivc_devs[i] = NULL;
			break;
		}
	}

	mutex_unlock(&g_i2c_ivc_lock);
}

static int tegra_i2c_ivc_add_single(struct tegra_ivc_channel *chan);

static int tegra_i2c_ivc_do_single_xfer(
	struct tegra_i2c_ivc_single_dev *i2c_ivc_dev,
	const struct i2c_msg *reqs, int num)
{
	u8 *pbuf = NULL, *pprev_len = NULL;
	const struct i2c_msg *preq, *preq_end;
	int ret = 0, len = 0;
	u8 *read_ptr = NULL;
	int read_len = 0;

	if (num == 0)
		return 0;

	++i2c_ivc_dev->stat.xfer_requests;

	/* First byte is bus ID */

	pbuf = i2c_ivc_dev->rpc_i2c_req_buf + 1;
	len = 1;

	preq = reqs;
	preq_end = reqs + num;

	for (;;) {
		bool is_read = (preq->flags & I2C_M_RD);
		u32 bytes = 0;

		if ((preq == preq_end) || (is_read && read_ptr != NULL) ||
			((len + 4 + preq->len) > CAMRTC_I2C_REQUEST_MAX_LEN)) {
			struct camrtc_rpc_i2c_response *rpc_rsp;

			rpc_rsp = &i2c_ivc_dev->rpc_i2c_rsp;

			i2c_ivc_dev->rpc_i2c_req.request_len = len;
			i2c_ivc_dev->rpc_i2c_req.callback = NULL;
			i2c_ivc_dev->rpc_i2c_req.callback_param =
				&i2c_ivc_dev->chan->dev;
			ret = tegra_ivc_rpc_call(i2c_ivc_dev->chan,
				&i2c_ivc_dev->rpc_i2c_req);

			if (ret < 0) {
				++i2c_ivc_dev->stat.errors;
				dev_err(&i2c_ivc_dev->chan->dev,
					"I2C transaction to 0x%x failed: %d\n",
					reqs[0].addr, ret);
				return -EIO;
			}

			if (rpc_rsp->result != 0) {
				dev_err(&i2c_ivc_dev->chan->dev,
					"I2C transaction response at addr 0x%x failed: %d\n",
					reqs[0].addr, rpc_rsp->result);
				return -EIO;
			}
			if (read_ptr) {
				memcpy(read_ptr, rpc_rsp->read_data, read_len);
				read_ptr = NULL;
			}

			if (preq == preq_end)
				return num;

			pbuf = i2c_ivc_dev->rpc_i2c_req_buf + 1;
			len = 1;
		}

		if (!is_read) {
			pbuf[0] = 0;
			++i2c_ivc_dev->stat.writes;
			i2c_ivc_dev->stat.write_bytes += preq->len;
		} else {
			read_ptr = preq->buf;
			read_len = preq->len;
			pbuf[0] = CAMRTC_I2C_REQUEST_FLAG_READ;
			++i2c_ivc_dev->stat.reads;
			i2c_ivc_dev->stat.read_bytes += preq->len;
		}

		if ((preq->flags & I2C_M_NOSTART) == 0) {
			pbuf[1] = preq->addr & 0xff;
			if (WARN_ON(preq->flags & I2C_M_TEN)) {
				/* This is not yet implemented */
				pbuf[0] |= CAMRTC_I2C_REQUEST_FLAG_TEN;
				pbuf[2] = (preq->addr >> 8) & 0xff;
				bytes = 2;
			} else {
				pbuf[2] = 0;
				bytes = 1;
			}
		} else {
			pbuf[0] |= CAMRTC_I2C_REQUEST_FLAG_NOSTART;
			/* slave address is don't care */
			pbuf[1] = 0;
			pbuf[2] = 0;
		}

		i2c_ivc_dev->stat.total_bytes += bytes + preq->len;

		pbuf[3] = preq->len;
		pprev_len = pbuf + 3;

		pbuf += 4;
		len += 4;

		if (is_read) {
			preq++;
			continue;
		}

		memcpy(pbuf, preq->buf, preq->len);
		pbuf += preq->len;
		len += preq->len;

		++preq;

		/* Merge write requests with NOSTART */
		while (preq != preq_end && (preq->flags & I2C_M_NOSTART)) {
			u8 *psrc = preq->buf;

			if ((len + preq->len) > CAMRTC_I2C_REQUEST_MAX_LEN)
				break;

			memcpy(pbuf, psrc, preq->len);

			pbuf += preq->len;
			*pprev_len += preq->len;
			len += preq->len;

			i2c_ivc_dev->stat.write_bytes += preq->len;
			i2c_ivc_dev->stat.total_bytes += preq->len;

			++preq;
		}
	}
}

int tegra_i2c_ivc_single_xfer(u32 i2c_base,
	const struct i2c_msg *reqs, int num)
{
	struct tegra_i2c_ivc_single_dev *i2c_ivc_dev;
	int ret = 0;

	i2c_ivc_dev = tegra_i2c_ivc_get_dev(i2c_base);
	if (i2c_ivc_dev == NULL)
		return -ENODEV;

	if (num <= 0)
		goto unlock;

	ret = tegra_ivc_channel_runtime_get(i2c_ivc_dev->chan);
	if (ret < 0)
		goto unlock;

	if (tegra_ivc_rpc_channel_is_suspended(i2c_ivc_dev->chan)) {
		ret = -EBUSY;
		goto error;
	}

	if (!i2c_ivc_dev->is_online) {
		ret = -ECONNRESET;
		goto error;
	}

	if (!i2c_ivc_dev->is_added) {
		ret = tegra_i2c_ivc_add_single(i2c_ivc_dev->chan);
		if (ret != 0) {
			dev_err(&i2c_ivc_dev->chan->dev,
				"I2C device not ready\n");
			ret = -EIO;
			goto error;
		}
	}

	ret = tegra_i2c_ivc_do_single_xfer(i2c_ivc_dev, reqs, num);

error:
	tegra_ivc_channel_runtime_put(i2c_ivc_dev->chan);
unlock:
	tegra_i2c_ivc_put_dev(i2c_ivc_dev);

	return ret;
}
EXPORT_SYMBOL(tegra_i2c_ivc_single_xfer);

/*
 * IVC channel Debugfs
 */

#define DEFINE_SEQ_FOPS(_fops_, _show_) \
	static int _fops_ ## _open(struct inode *inode, struct file *file) \
	{ \
		return single_open(file, _show_, inode->i_private); \
	} \
	static const struct file_operations _fops_ = { \
		.open = _fops_ ## _open, \
		.read = seq_read, \
		.llseek = seq_lseek, \
		.release = single_release }

static int tegra_i2c_ivc_single_stat_show(
	struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *chan = file->private;
	struct tegra_i2c_ivc_single_dev *i2c_ivc_dev =
		tegra_ivc_channel_get_drvdata(chan);

	seq_printf(file, "Xfer requests: %u\n",
		i2c_ivc_dev->stat.xfer_requests);
	seq_printf(file, "Total bytes: %u\n", i2c_ivc_dev->stat.total_bytes);
	seq_printf(file, "Read requests: %u\n", i2c_ivc_dev->stat.reads);
	seq_printf(file, "Read bytes: %u\n", i2c_ivc_dev->stat.read_bytes);
	seq_printf(file, "Write requests: %u\n", i2c_ivc_dev->stat.writes);
	seq_printf(file, "Write bytes: %u\n", i2c_ivc_dev->stat.write_bytes);
	seq_printf(file, "Errors: %u\n", i2c_ivc_dev->stat.errors);

	return 0;
}

DEFINE_SEQ_FOPS(tegra_i2c_ivc_debugfs_stats,
	tegra_i2c_ivc_single_stat_show);

static void tegra_i2c_ivc_single_create_debugfs(
	struct tegra_ivc_channel *chan,
	struct dentry *debugfs_root)
{
	debugfs_create_file("stats", S_IRUGO,
		debugfs_root, chan,
		&tegra_i2c_ivc_debugfs_stats);
}

/*
 * IVC channel driver interface
 */

static int tegra_i2c_ivc_add_single(struct tegra_ivc_channel *chan)
{
	struct tegra_i2c_ivc_single_dev *i2c_ivc_dev =
		tegra_ivc_channel_get_drvdata(chan);
	struct camrtc_rpc_i2c_add_single rpc_add_single;
	u32 bus_id;
	int ret;

	if (i2c_ivc_dev->reg_base == 0) {
		dev_err(&chan->dev,
			"Invalid I2C device at 0x%08x\n",
			i2c_ivc_dev->reg_base);
		return -ENODEV;
	}

	rpc_add_single.reg_base = i2c_ivc_dev->reg_base;
	rpc_add_single.bus_clk_rate = i2c_ivc_dev->bus_clk_rate;

	/* Register an I2C device to CamRTC */
	ret = tegra_ivc_rpc_call_pl(chan,
		CAMRTC_RPC_REQ_I2C_ADD_SINGLE_DEV,
		sizeof(rpc_add_single), &rpc_add_single,
		TEGRA_IVC_RPC_RSP_RET_CODE,
		sizeof(bus_id), &bus_id,
		NULL, NULL, 0);
	if (ret < 0) {
		dev_err(&chan->dev,
			"Failed to register an I2C device at 0x%08x: %d\n",
			i2c_ivc_dev->reg_base, ret);
		return -EIO;
	}

	dev_info(&chan->dev,
		"Registered an I2C single device at 0x%08x to bus %d\n",
		i2c_ivc_dev->reg_base, bus_id);

	i2c_ivc_dev->bus_id = bus_id;
	i2c_ivc_dev->rpc_i2c_req_buf[0] = bus_id;
	i2c_ivc_dev->rpc_i2c_req.request_id =
		CAMRTC_RPC_REQ_I2C_REQUEST_SINGLE;
	i2c_ivc_dev->rpc_i2c_req.request = i2c_ivc_dev->rpc_i2c_req_buf;
	i2c_ivc_dev->rpc_i2c_req.response_id = CAMRTC_RPC_RSP_I2C_RESPONSE;
	i2c_ivc_dev->rpc_i2c_req.response_len =
		sizeof(i2c_ivc_dev->rpc_i2c_rsp);
	i2c_ivc_dev->rpc_i2c_req.response = &i2c_ivc_dev->rpc_i2c_rsp;
	i2c_ivc_dev->rpc_i2c_req.callback = NULL;
	i2c_ivc_dev->rpc_i2c_req.callback_param = &i2c_ivc_dev->chan->dev;
	i2c_ivc_dev->rpc_i2c_req.timeout_ms =
		I2C_CAMRTC_RPC_IVC_SINGLE_TIMEOUT_MS;
	i2c_ivc_dev->is_added = true;

	return 0;
}

static void tegra_i2c_ivc_single_ready(
	struct tegra_ivc_channel *chan, bool online)
{
	struct tegra_i2c_ivc_single_dev *i2c_ivc_dev =
		tegra_ivc_channel_get_drvdata(chan);

	i2c_ivc_dev->is_online = online;

	if (!online)
		i2c_ivc_dev->is_added = false;
}

static struct tegra_ivc_rpc_ops tegra_ivc_rpc_user_ops = {
	.create_debugfs = tegra_i2c_ivc_single_create_debugfs,
	.ready = tegra_i2c_ivc_single_ready,
};

/* Platform device */
static int tegra_ivc_rpc_i2c_single_probe(struct tegra_ivc_channel *chan)
{
	int ret;
	struct tegra_i2c_ivc_single_dev *i2c_ivc_dev;
	struct device_node *i2c_node;
	u32 reg_base;

	/* Register IVC/RPC channel */
	ret = tegra_ivc_rpc_channel_probe(chan, &tegra_ivc_rpc_user_ops);
	if (ret < 0) {
		dev_err(&chan->dev, "Cannot start IVC/RPC interface");
		return ret;
	}

	i2c_node = of_parse_phandle(chan->dev.of_node, "device", 0);
	if (i2c_node == NULL) {
		dev_err(&chan->dev, "Cannot get i2c device node");
		ret = -ENODEV;
		goto remove;
	}

	reg_base = tegra_i2c_get_reg_base(i2c_node);
	if (reg_base == 0) {
		ret = -ENODEV;
		goto put;
	}

	i2c_ivc_dev = devm_kzalloc(&chan->dev, sizeof(*i2c_ivc_dev),
			GFP_KERNEL);
	if (i2c_ivc_dev == NULL) {
		ret = -ENOMEM;
		goto put;
	}

	mutex_init(&i2c_ivc_dev->inuse);

	i2c_ivc_dev->is_added = false;
	i2c_ivc_dev->is_online = false;
	i2c_ivc_dev->reg_base = reg_base;
	i2c_ivc_dev->bus_clk_rate = tegra_i2c_get_clk_freq(i2c_node);
	i2c_ivc_dev->chan = chan;

	tegra_ivc_channel_set_drvdata(chan, i2c_ivc_dev);

	ret = tegra_i2c_ivc_register(i2c_ivc_dev);
	if (ret < 0)
		goto put;

	return 0;

put:
	of_node_put(i2c_node);
remove:
	tegra_ivc_rpc_channel_remove(chan);
	return ret;
}

static void tegra_ivc_rpc_i2c_single_remove(struct tegra_ivc_channel *chan)
{
	struct tegra_i2c_ivc_single_dev *i2c_ivc_dev;

	i2c_ivc_dev = tegra_ivc_channel_get_drvdata(chan);
	tegra_i2c_ivc_unregister(i2c_ivc_dev);
	tegra_ivc_rpc_channel_remove(chan);
}

static const struct of_device_id tegra_ivc_rpc_i2c_single_of_match[] = {
	{ .compatible = "nvidia,tegra186-camera-ivc-rpc-i2c-single", },
	{ },
};
TEGRA_IVC_RPC_DRIVER_DEFINE(i2c_single, "tegra-ivc-rpc-i2c-single")

MODULE_AUTHOR("Ashish Singh <assingh@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra CAMRTC I2C IVC driver");
MODULE_LICENSE("GPL");
