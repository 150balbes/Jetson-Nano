/*
 * Copyright (C) 2017-2020 NVIDIA Corporation.  All rights reserved.
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

#include <linux/debugfs.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/slab.h>

#include "soc/tegra/tegra-ivc-rpc.h"
#include "soc/tegra/tegra-i2c-rtcpu.h"
#include "soc/tegra/camrtc-i2c-common.h"
#include "i2c-rtcpu-common.h"

/*
 * I2C IVC Multi driver internal data structure
 */

#define TEGRA_I2C_MULTI_MAX_DEV	4

#define I2C_CAMRTC_RPC_IVC_MULTI_TIMEOUT_MS	250

struct tegra_i2c_ivc_multi_dev {
	/* IVC RPC */
	struct tegra_ivc_channel *chan;
	bool is_valid;
	bool is_failed;
	bool is_added;
	bool is_online;

	/* I2C */
	u32 bus_id;
	/* parameters from DT */
	u32 reg_base;
	u32 bus_clk_rate;

	struct list_head sensors;
	/* statistics */
	struct {
		unsigned int xfer_requests;
		unsigned int total_bytes;
		unsigned int reads, read_bytes;
		unsigned int writes, write_bytes;
		unsigned int errors;
	} stat;
};

static struct tegra_i2c_ivc_multi_dev
	g_i2c_i2c_ivc_devs[TEGRA_I2C_MULTI_MAX_DEV];

/*
 * Per sensor data structure
 */

struct tegra_i2c_rtcpu_sensor {
	struct list_head node;
	struct tegra_i2c_ivc_multi_dev *i2c_ivc_dev;
	bool is_registered;
	unsigned int sensor_id;
	/* config */
	struct tegra_i2c_rtcpu_config config;
	/* from device tree */
	unsigned int addr;
	unsigned int flag;
	unsigned int mp_type;
	unsigned int mp_addr;
	unsigned int mp_channel;
	/* runtime information */
	bool in_agg; /* in the middle of aggregation */
	unsigned int last_addr; /* is_valid if in_agg. last register address */
	int frame_id; /* is_valid if in_agg. frame ID */
	/* RPC call for I2C_REQUEST_MULTI */
	unsigned int req_len;
	u8 *req_cur, *req_last_len;
	struct tegra_ivc_rpc_call_param rpc_i2c_req;
	u8 rpc_i2c_req_buf[CAMRTC_I2C_REQUEST_MAX_LEN];
	struct camrtc_rpc_i2c_response rpc_i2c_rsp;
};

/*
 * Sensor APIs
 */
static int tegra_i2c_ivc_register_sensor(struct tegra_ivc_channel *chan,
	struct tegra_i2c_rtcpu_sensor *sensor);

struct tegra_i2c_rtcpu_sensor *tegra_i2c_rtcpu_register_sensor(
	struct i2c_client *client,
	const struct tegra_i2c_rtcpu_config *config)
{
	int i, ret;
	u32 data_u32;
	struct tegra_i2c_rtcpu_sensor *sensor;
	struct device_node *node = client->dev.of_node;
	struct device_node *np_mux, *np_i2c;
	struct tegra_i2c_ivc_multi_dev *i2c_ivc_dev;

	if (config->reg_bytes <= 0 || config->reg_bytes > 2)
		return NULL;

	sensor = kzalloc(sizeof(*sensor), GFP_KERNEL);
	if (sensor == NULL)
		return NULL;

	sensor->config = *config;

	/* Example of a Sensor node
	 * behind an I2C multiplexer: /i2c@3180000/tca9548@77/i2c@0/ov5693_a@36
	 * without an I2C multiplexer: /i2c@3180000/ov5693_c@36
	 */

	/* Sensor node */
	ret = of_property_read_u32(node, "reg", &data_u32);
	if (ret)
		goto fail;
	sensor->addr = data_u32;

	if (of_property_read_bool(node, "i2c-10bits"))
		sensor->flag |= CAMRTC_I2C_SENSOR_FLAG_TEN;
	if (of_property_read_bool(node, "i2c-fast-mode-plus"))
		sensor->flag |= CAMRTC_I2C_SENSOR_FLAG_FM_PLUS;
	if (of_property_read_bool(node, "i2c-high-speed-mode"))
		sensor->flag |= CAMRTC_I2C_SENSOR_FLAG_HS;

	/* Parent of sensor is either I2C bus, or a channel under multiplexer.
	 * Detect a multiplexer.
	 */
	np_i2c = of_get_parent(node);
	np_mux = of_get_parent(np_i2c);

	sensor->mp_type = CAMRTC_I2C_MP_NONE;
	ret = of_property_read_u32(np_mux, "nvidia,camrtc-mux-type",
		&data_u32);

	if (ret == 0) {
		sensor->mp_type = data_u32;

		/* Information about the multiplexer */
		if (sensor->mp_type != CAMRTC_I2C_MP_NONE) {
			ret = of_property_read_u32(np_i2c, "reg",
				&data_u32);
			if (ret)
				goto fail;
			sensor->mp_channel = data_u32;

			ret = of_property_read_u32(np_mux, "reg",
					&data_u32);
			if (ret)
				goto fail;

			sensor->mp_addr = data_u32;
			np_i2c = of_get_parent(np_mux);
		}
	} else
		np_mux = NULL;

	/* Detect whether to enable I2C multi */
	if (!of_property_read_bool(np_i2c, "nvidia,camrtc-use-multi"))
		goto fail;

	/* I2C controller address */
	ret = of_property_read_u32_index(np_i2c, "reg", 1, &data_u32);
	if (ret)
		goto fail;

	/* Find an IVC channel */
	for (i = 0; i < TEGRA_I2C_MULTI_MAX_DEV; ++i) {
		if (g_i2c_i2c_ivc_devs[i].is_valid) {
			if (g_i2c_i2c_ivc_devs[i].reg_base == (u32) data_u32) {
				if (g_i2c_i2c_ivc_devs[i].is_failed)
					goto fail;
				break;
			}
		} else
			goto fail;
	}

	if (i == TEGRA_I2C_MULTI_MAX_DEV)
		goto fail;

	i2c_ivc_dev = g_i2c_i2c_ivc_devs + i;
	sensor->i2c_ivc_dev = i2c_ivc_dev;

	/* Add i2c device */
	ret = tegra_i2c_ivc_register_sensor(i2c_ivc_dev->chan, sensor);
	if (ret != 0)
		goto fail;

	/* Sensor information */
	if (sensor->mp_type != CAMRTC_I2C_MP_NONE) {
		dev_info(&i2c_ivc_dev->chan->dev,
			"Bus: %u, Multiplexer type: %u, Address: 0x%x:%u:0x%x\n",
			i2c_ivc_dev->bus_id,
			sensor->mp_type,
			sensor->mp_addr, sensor->mp_channel, sensor->addr);
	} else {
		dev_info(&i2c_ivc_dev->chan->dev,
			"Bus: %u, Address: 0x%x\n",
			i2c_ivc_dev->bus_id,
			sensor->addr);
	}

	list_add_tail(&sensor->node, &i2c_ivc_dev->sensors);
	return sensor;

fail:
	kfree(sensor);
	return NULL;
}
EXPORT_SYMBOL(tegra_i2c_rtcpu_register_sensor);

/*
 * I2C transfer
 */

static int tegra_i2c_ivc_multi_xfer(
	struct tegra_i2c_rtcpu_sensor *sensor)
{
	int ret = 0;

	tegra_ivc_channel_runtime_get(sensor->i2c_ivc_dev->chan);

	if (sensor->req_len == CAMRTC_I2C_MULTI_HEADER_SIZE) {
		ret = 0;
		goto exit;
	}

	if (!sensor->is_registered) {
		unsigned int req_len = sensor->req_len;

		ret = tegra_i2c_ivc_register_sensor(sensor->i2c_ivc_dev->chan,
			sensor);

		if (ret != 0)
			goto exit;

		/* restore len since register sensor inits to a differnet
		 * value, which is used during first init at boot
		 */
		sensor->req_len = req_len;
	}

	sensor->rpc_i2c_req_buf[1] = (sensor->frame_id > 0) ?
		CAMRTC_I2C_REQUEST_MULTI_FLAG_FRAMEID : 0;
	sensor->rpc_i2c_req_buf[2] = (sensor->frame_id >> 0) & 0xff;
	sensor->rpc_i2c_req_buf[3] = (sensor->frame_id >> 8) & 0xff;

	sensor->rpc_i2c_req.request_len = sensor->req_len;
	ret = tegra_ivc_rpc_call(sensor->i2c_ivc_dev->chan,
		&sensor->rpc_i2c_req);

	/* reset request buffer pointers */
	sensor->last_addr = (unsigned int) -1;
	sensor->req_len = CAMRTC_I2C_MULTI_HEADER_SIZE;
	sensor->req_cur = sensor->rpc_i2c_req_buf +
		CAMRTC_I2C_MULTI_HEADER_SIZE;
	sensor->req_last_len = NULL;

	if (ret < 0) {
		++sensor->i2c_ivc_dev->stat.errors;
		dev_err(&sensor->i2c_ivc_dev->chan->dev,
			"I2C transaction to sensor %u failed: %d\n",
			sensor->sensor_id, ret);
		ret = -EIO;
		goto exit;
	}

exit:
	tegra_ivc_channel_runtime_put(sensor->i2c_ivc_dev->chan);
	return ret;
}

/*
 * I2C APIs
 */

int tegra_i2c_rtcpu_aggregate(
	struct tegra_i2c_rtcpu_sensor *sensor,
	bool start)
{
	if (sensor->in_agg) {
		if (!start) {
			sensor->in_agg = false;
			return tegra_i2c_ivc_multi_xfer(sensor);
		}
	} else {
		sensor->in_agg = start;
		sensor->last_addr = (unsigned int) -1;
		sensor->frame_id = -1;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_i2c_rtcpu_aggregate);

int tegra_i2c_rtcpu_set_frame_id(
	struct tegra_i2c_rtcpu_sensor *sensor,
	int frame_id)
{
	if (!sensor->in_agg)
		return -EINVAL;

	sensor->frame_id = frame_id;

	return 0;
}
EXPORT_SYMBOL(tegra_i2c_rtcpu_set_frame_id);

int tegra_i2c_rtcpu_read_reg8(
	struct tegra_i2c_rtcpu_sensor *sensor,
	unsigned int addr,
	u8 *data,
	unsigned int count)
{
	int ret;
	int this_len;
	u8 *req;

	/* Read requires writing address first */
	this_len = (CAMRTC_I2C_MULTI_DATA_OFFSET * 2) +
		sensor->config.reg_bytes;

	/* If there is no room, flush current transfer */
	if (sensor->req_len + this_len > CAMRTC_I2C_REQUEST_MAX_LEN) {
		ret = tegra_i2c_ivc_multi_xfer(sensor);
		if (ret != 0)
			return ret;
	}

	/* Write register address */
	req = sensor->req_cur;

	if (!sensor->in_agg || sensor->last_addr != addr) {
		*req++ = 0;
		*req++ = (u8) sensor->config.reg_bytes;
		if (sensor->config.reg_bytes == 2)
			*req++ = (u8) (addr >> 8);
		*req++ = (u8) addr;
		sensor->req_len += CAMRTC_I2C_MULTI_DATA_OFFSET +
			sensor->config.reg_bytes;
	}

	/* Read register */
	*req++ = CAMRTC_I2C_REQUEST_FLAG_READ;
	*req = (u8) count;
	sensor->req_len += CAMRTC_I2C_MULTI_DATA_OFFSET;

	/* Read transaction always start a transaction */
	ret = tegra_i2c_ivc_multi_xfer(sensor);
	if (ret == 0)
		memcpy(data, sensor->rpc_i2c_rsp.read_data, count);

	return ret;
}
EXPORT_SYMBOL(tegra_i2c_rtcpu_read_reg8);

int tegra_i2c_rtcpu_write_reg8(
	struct tegra_i2c_rtcpu_sensor *sensor,
	unsigned int addr,
	const u8 *data,
	unsigned int count)
{
	int ret;
	int this_len;
	u8 *req;

	this_len = CAMRTC_I2C_MULTI_DATA_OFFSET +
		sensor->config.reg_bytes + count;

	/* If there is no room, flush current transfer */
	if (sensor->req_len + this_len > CAMRTC_I2C_REQUEST_MAX_LEN) {
		ret = tegra_i2c_ivc_multi_xfer(sensor);
		if (ret != 0)
			return ret;
	}

	/* Write transfer */
	req = sensor->req_cur;

	if (!sensor->in_agg || sensor->last_addr != addr) {
		*req++ = 0;
		sensor->req_last_len = req;
		*req++ = (u8) (sensor->config.reg_bytes + count);
		if (sensor->config.reg_bytes == 2)
			*req++ = (u8) (addr >> 8);
		*req++ = (u8) addr;
		sensor->req_len += CAMRTC_I2C_MULTI_DATA_OFFSET +
			sensor->config.reg_bytes;
	} else {
		/* append to previous transfer */
		*sensor->req_last_len += (u8) count;
	}

	switch (count) {
	case 4:
		*req++ = *data++;
		/* fallthrough */
	case 3:
		*req++ = *data++;
		/* fallthrough */
	case 2:
		*req++ = *data++;
		/* fallthrough */
	case 1:
		*req++ = *data++;
		/* fallthrough */
	case 0:
		break;
	default:
		memcpy(req, data, count);
		req += count;
		break;
	}

	sensor->req_cur = req;
	sensor->req_len += count;
	sensor->last_addr = addr + count;

	if (!sensor->in_agg)
		return tegra_i2c_ivc_multi_xfer(sensor);
	else
		return 0;
}
EXPORT_SYMBOL(tegra_i2c_rtcpu_write_reg8);

int tegra_i2c_rtcpu_write_table_8(
	struct tegra_i2c_rtcpu_sensor *sensor,
	const struct reg_8 table[],
	const struct reg_8 override_list[],
	int num_override_regs, u16 wait_ms_addr, u16 end_addr)
{
	const struct reg_8 *next;
	int i;

	tegra_i2c_rtcpu_aggregate(sensor, true);

	for (next = table;; ++next) {
		u8 val;

		if (next->addr == end_addr)
			break;
		if (next->addr == wait_ms_addr) {
			tegra_i2c_rtcpu_aggregate(sensor, false);
			msleep_range(next->val);
			tegra_i2c_rtcpu_aggregate(sensor, true);
			continue;
		}

		val = next->val;

		if (override_list) {
			for (i = 0; i < num_override_regs; ++i) {
				if (next->addr == override_list[i].addr) {
					val = override_list[i].val;
					break;
				}
			}
		}

		tegra_i2c_rtcpu_write_reg8(sensor,
			next->addr, &val, 1);
	}

	return tegra_i2c_rtcpu_aggregate(sensor, false);
}
EXPORT_SYMBOL(tegra_i2c_rtcpu_write_table_8);

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

static int tegra_i2c_ivc_multi_stat_show(
	struct seq_file *file, void *data)
{
	struct tegra_ivc_channel *chan = file->private;
	struct tegra_i2c_ivc_multi_dev *i2c_ivc_dev =
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
	tegra_i2c_ivc_multi_stat_show);

static void tegra_i2c_ivc_multi_create_debugfs(
	struct tegra_ivc_channel *chan,
	struct dentry *debugfs_root)
{
	debugfs_create_file("stats", 0444,
		debugfs_root, chan,
		&tegra_i2c_ivc_debugfs_stats);
}

/*
 * IVC channel driver interface
 */
static int tegra_i2c_ivc_add_multi(struct tegra_ivc_channel *chan)
{

	struct tegra_i2c_ivc_multi_dev *i2c_ivc_dev =
		tegra_ivc_channel_get_drvdata(chan);
	struct camrtc_rpc_i2c_add_multi rpc_add_multi;
	u32 bus_id;
	int ret;

	rpc_add_multi.reg_base = i2c_ivc_dev->reg_base;

	/* Register an I2C device to CamRTC */
	ret = tegra_ivc_rpc_call_pl(chan,
		CAMRTC_RPC_REQ_I2C_ADD_MULTI_DEV,
		sizeof(rpc_add_multi), &rpc_add_multi,
		TEGRA_IVC_RPC_RSP_RET_CODE,
		sizeof(bus_id), &bus_id,
		NULL, NULL, 0);
	if (ret < 0) {
		dev_err(&chan->dev,
			"failed to register an I2C device at 0x%08x: %d\n",
			i2c_ivc_dev->reg_base, ret);
		ret = -EIO;
		goto fail_remove_chan;
	}

	dev_info(&chan->dev,
		"Registered an I2C multi device at 0x%08x to bus %d\n",
		i2c_ivc_dev->reg_base, bus_id);

	i2c_ivc_dev->bus_id = bus_id;
	i2c_ivc_dev->is_added = true;

	return 0;

fail_remove_chan:
	tegra_ivc_rpc_channel_remove(chan);
	i2c_ivc_dev->is_failed = true;
	return ret;
}

static int tegra_i2c_ivc_add_sensor(struct tegra_ivc_channel *chan,
	struct tegra_i2c_rtcpu_sensor *sensor)
{
	int ret = 0;
	u32 sensor_id;
	struct tegra_i2c_ivc_multi_dev *i2c_ivc_dev =
		tegra_ivc_channel_get_drvdata(chan);
	struct camrtc_rpc_i2c_add_sensor rpc_add_sensor = {0};

	WARN_ON(!chan);
	WARN_ON(!i2c_ivc_dev->is_added);

	rpc_add_sensor.bus_id = i2c_ivc_dev->bus_id;
	rpc_add_sensor.addr = sensor->addr;
	rpc_add_sensor.flag = sensor->flag;
	rpc_add_sensor.mp_type = sensor->mp_type;
	rpc_add_sensor.mp_addr = sensor->mp_addr;
	rpc_add_sensor.mp_channel = sensor->mp_channel;

	ret = tegra_ivc_rpc_call_pl(chan,
		CAMRTC_RPC_REQ_I2C_ADD_SENSOR,
		sizeof(rpc_add_sensor), &rpc_add_sensor,
		TEGRA_IVC_RPC_RSP_RET_CODE,
		sizeof(sensor_id), &sensor_id,
		NULL, NULL, 0);
	if (ret < 0) {
		dev_err(&chan->dev,
			"Failed to register a sensor: %d\n", ret);
		ret = -EIO;
		goto fail;
	}

	sensor->sensor_id = sensor_id;
	dev_info(&chan->dev,
		"Registered a sensor as ID: %u\n", sensor_id);

	/* Initialization of runtime field */
	sensor->in_agg = false;
	sensor->last_addr = (unsigned int) -1;
	sensor->req_len = CAMRTC_I2C_MULTI_HEADER_SIZE;
	sensor->req_cur = sensor->rpc_i2c_req_buf +
		CAMRTC_I2C_MULTI_HEADER_SIZE;
	sensor->req_last_len = NULL;

	sensor->rpc_i2c_req_buf[0] = (u8) sensor_id;
	sensor->rpc_i2c_req.request_id = CAMRTC_RPC_REQ_I2C_REQUEST_MULTI;
	sensor->rpc_i2c_req.request = sensor->rpc_i2c_req_buf;
	sensor->rpc_i2c_req.response_id = CAMRTC_RPC_RSP_I2C_RESPONSE;
	sensor->rpc_i2c_req.response_len = sizeof(sensor->rpc_i2c_rsp);
	sensor->rpc_i2c_req.response = &sensor->rpc_i2c_rsp;
	sensor->rpc_i2c_req.callback = NULL;
	sensor->rpc_i2c_req.callback_param = NULL;
	sensor->rpc_i2c_req.timeout_ms = I2C_CAMRTC_RPC_IVC_MULTI_TIMEOUT_MS;
	sensor->is_registered = true;

	return 0;
fail:
	return ret;
}

static int tegra_i2c_ivc_register_sensor(struct tegra_ivc_channel *chan,
	struct tegra_i2c_rtcpu_sensor *sensor)
{
	int ret = 0;
	struct tegra_i2c_ivc_multi_dev *i2c_ivc_dev =
		tegra_ivc_channel_get_drvdata(chan);

	WARN_ON(!i2c_ivc_dev->is_online);

	if (!i2c_ivc_dev->is_added) {
		ret = tegra_i2c_ivc_add_multi(chan);
		if (ret != 0) {
			dev_err(&chan->dev,
				"I2C device not added\n");
			goto fail;
		}
	}

	/* Add a sensor */
	ret = tegra_i2c_ivc_add_sensor(chan,
		sensor);
	if (ret != 0)
		dev_err(&chan->dev,
			"I2C sensor not added\n");

fail:
	return ret;
}

static void tegra_i2c_ivc_multi_ready(
	struct tegra_ivc_channel *chan, bool online)
{
	struct tegra_i2c_rtcpu_sensor *sensor = NULL;
	struct tegra_i2c_ivc_multi_dev *i2c_ivc_dev =
		tegra_ivc_channel_get_drvdata(chan);

	i2c_ivc_dev->is_online = online;

	if (!online) {
		i2c_ivc_dev->is_added = false;

		list_for_each_entry(sensor, &i2c_ivc_dev->sensors, node)
			sensor->is_registered = false;
	}
}

static struct tegra_ivc_rpc_ops tegra_ivc_rpc_user_ops = {
	.create_debugfs = tegra_i2c_ivc_multi_create_debugfs,
	.ready = tegra_i2c_ivc_multi_ready,
};

static int tegra_ivc_rpc_i2c_multi_probe(struct tegra_ivc_channel *chan)
{
	int ret;
	int i;
	struct tegra_i2c_ivc_multi_dev *i2c_ivc_dev;
	struct device_node *i2c_node;

	/* Find an empty slot */
	for (i = 0; i < TEGRA_I2C_MULTI_MAX_DEV; ++i) {
		if (!g_i2c_i2c_ivc_devs[i].is_valid)
			break;
	}

	if (i == TEGRA_I2C_MULTI_MAX_DEV)
		return -ENOMEM;

	i2c_ivc_dev = g_i2c_i2c_ivc_devs + i;

	i2c_node = of_parse_phandle(chan->dev.of_node, "device", 0);
	if (i2c_node == NULL) {
		dev_err(&chan->dev, "Cannot get i2c device node");
		ret = -ENODEV;
		goto fail_free;
	}

	/* Register IVC/RPC channel */
	ret = tegra_ivc_rpc_channel_probe(chan, &tegra_ivc_rpc_user_ops);
	if (ret < 0) {
		dev_err(&chan->dev, "Cannot start IVC/RPC interface");
		goto fail_free;
	}

	i2c_ivc_dev->is_valid = true;
	i2c_ivc_dev->is_added = false;
	i2c_ivc_dev->is_failed = false;
	i2c_ivc_dev->is_online = false;
	i2c_ivc_dev->chan = chan;
	i2c_ivc_dev->reg_base = tegra_i2c_get_reg_base(i2c_node);
	i2c_ivc_dev->bus_clk_rate = tegra_i2c_get_clk_freq(i2c_node);
	INIT_LIST_HEAD(&i2c_ivc_dev->sensors);
	tegra_ivc_channel_set_drvdata(chan, i2c_ivc_dev);

	return 0;

fail_free:
	devm_kfree(&chan->dev, i2c_ivc_dev);

	return ret;
}

static void tegra_ivc_rpc_i2c_multi_remove(struct tegra_ivc_channel *chan)
{
	tegra_ivc_rpc_channel_remove(chan);
}

static const struct of_device_id tegra_ivc_rpc_i2c_multi_of_match[] = {
	{ .compatible = "nvidia,tegra186-camera-ivc-rpc-i2c-multi", },
	{ },
};
TEGRA_IVC_RPC_DRIVER_DEFINE(i2c_multi, "tegra-ivc-rpc-i2c-multi")

MODULE_AUTHOR("Kai Lee <kailee@nvidia.com>");
MODULE_DESCRIPTION("NVIDIA Tegra CAMRTC I2C IVC multi driver");
MODULE_LICENSE("GPL");
