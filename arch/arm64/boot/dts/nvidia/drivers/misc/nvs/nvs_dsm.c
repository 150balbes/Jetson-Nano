/* Copyright (c) 2016-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* The NVS = NVidia Sensor framework */
/* See nvs.h for documentation */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/nvs.h>

#define DSM_DRIVER_VERSION		(3)
#define DSM_VENDOR			"NVIDIA Corporation"
#define DSM_NAME			"nvs_dsm"
#define DSM_DEV_PATH_N			(64)
#define DSM_PUSH_DELAY_MS		(100)
#define DSM_PUSH_DELAY_N		(10)

enum DSM_KIF {
	DSM_KIF_RELAY = 0,
	DSM_KIF_IIO,
	DSM_KIF_INPUT,
	DSM_KIF_N,
};

static const char *dsm_kif_str[DSM_KIF_N] = {
	[DSM_KIF_RELAY]			= "RELAY",
	[DSM_KIF_IIO]			= "IIO",
	[DSM_KIF_INPUT]			= "INPUT",
};

static struct nvs_fn_if *(*nvs_kif[])(void) = {
#ifdef NVS_KIF_RELAY
	nvs_relay,
#else
	NULL,
#endif
#ifdef NVS_KIF_IIO
	nvs_iio,
#else
	NULL,
#endif
#ifdef NVS_KIF_INPUT
	nvs_input,
#else
	NULL,
#endif
};

struct dsm_state {
	struct platform_device *pd;
	struct mutex mutex;
	struct delayed_work dw;
	struct nvs_fn_if *nvs[ARRAY_SIZE(nvs_kif)];
	void *nvs_st[ARRAY_SIZE(nvs_kif)];
	struct sensor_cfg cfg;
	struct nvs_dsm_msg msg;
	s64 msg_ts;
	int msg_err;
	unsigned int sts;
	unsigned int delay_ms;
	unsigned int delay_n;
	unsigned int connect_n;
	unsigned int disconnect_n;
	unsigned int kif;
	bool mutex_locked;
	char dev_path[DSM_DEV_PATH_N];
};

enum DSM_DBG {
	DSM_DBG_STS = 0,
	DSM_DBG_PUSH = 0xC6, /* use 0xD0 on cmd line */
	DSM_DBG_MSG_FLAGS,
	DSM_DBG_MSG_DEV_ID,
	DSM_DBG_MSG_SNSR_ID,
	DSM_DBG_UUID,
	DSM_DBG_PUSH_DELAY_MS,
	DSM_DBG_PUSH_DELAY_N,
	DSM_DBG_N,
};


static struct dsm_state *dsm_state_local;

static void dsm_mutex_lock(struct dsm_state *st)
{
	mutex_lock(&st->mutex);
	st->mutex_locked = true;
}

static void dsm_mutex_unlock(struct dsm_state *st)
{
	if (st->mutex_locked) {
		st->mutex_locked = false;
		mutex_unlock(&st->mutex);
	}
}

static int dsm_push_msg(struct dsm_state *st, unsigned int kif_i,
			char *dev_path)
{
	int ret = 0;

	if (dev_path) {
		ret = 1;
		if (st->sts & (NVS_STS_SPEW_MSG | NVS_STS_SPEW_DATA)) {
			if (ret)
				dev_info(&st->pd->dev, "%s NO %s\n",
					 __func__, dev_path);
			else
				dev_info(&st->pd->dev, "%s %s FOUND\n",
					 __func__, dev_path);
		}
	}
	if (st->msg_err <= 0 || !ret) {
		st->msg_ts = nvs_timestamp();
		ret = st->nvs[kif_i]->handler(st->nvs_st[kif_i],
					      &st->msg, st->msg_ts);
		if (ret > 0)
			ret = 0;
		st->msg_err = ret;
		dsm_mutex_unlock(st);
	} else {
		if (st->sts & (NVS_STS_SPEW_MSG | NVS_STS_SPEW_DATA))
			dev_info(&st->pd->dev,
				 "%s schedule_delayed_work=%ums (n=%u)\n",
				 __func__, st->delay_ms, st->msg_err);
		if (st->msg_err > 0)
			st->msg_err--;
		schedule_delayed_work(&st->dw, msecs_to_jiffies(st->delay_ms));
		ret = 0;
	}
	return ret;
}

static void dsm_work(struct work_struct *ws)
{
	struct dsm_state *st = container_of((struct delayed_work *)ws,
					    struct dsm_state, dw);

	dsm_push_msg(st, st->kif, st->dev_path);
}

static int dsm_push(struct dsm_state *st, int dev_id, bool connect,
		    int snsr_id, unsigned char *uuid, unsigned int kif_i)
{
	int ret = 0;

	dsm_mutex_lock(st);
	st->kif = kif_i;
	memset(&st->msg, 0, sizeof(st->msg));
	st->msg.ver = sizeof(st->msg);
	st->msg.dev_id = dev_id;
	st->msg.snsr_id = snsr_id;
	if (uuid)
		memcpy(&st->msg.uuid, uuid, sizeof(st->msg.uuid));
	if (connect) {
		st->msg.flags = 1 << NVS_DSM_MSG_FLAGS_CONNECT;
		st->connect_n++;
		if (!st->connect_n)
			st->connect_n--;
		if (dev_id >= 0) {
			switch (kif_i) {
			case DSM_KIF_RELAY:
				ret = snprintf(st->dev_path,
					       sizeof(st->dev_path),
					       "/dev/nvs:device%d", dev_id);
				break;

			case DSM_KIF_IIO:
				ret = snprintf(st->dev_path,
					       sizeof(st->dev_path),
					       "/dev/iio:device%d", dev_id);
				break;

			case DSM_KIF_INPUT:
				ret = snprintf(st->dev_path,
					       sizeof(st->dev_path),
					       "/dev/input/event%d", dev_id);
				break;
			}

			if (ret > 0) {
				st->msg_err = st->delay_n;
				return dsm_push_msg(st, kif_i, st->dev_path);
			}
		} else {
			st->msg_err = 0;
			return dsm_push_msg(st, kif_i, NULL);
		}

		dsm_mutex_unlock(st);
		return -ENOMEM;
	}

	/* disconnect */
	st->disconnect_n++;
	if (!st->disconnect_n)
		st->disconnect_n--;
	st->msg_err = 0;
	return dsm_push_msg(st, kif_i, NULL);
}

int nvs_dsm_relay(int dev_id, bool connect, int snsr_id, unsigned char *uuid)
{
	struct dsm_state *st = dsm_state_local;
	int ret = -EPERM;

	if (st == NULL)
		return -EPERM;

	if (st->nvs[DSM_KIF_RELAY] && st->nvs_st[DSM_KIF_RELAY])
		ret = dsm_push(st, dev_id, connect, snsr_id, uuid,
			       DSM_KIF_RELAY);
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&st->pd->dev, "%s err=%d\n", __func__, ret);
	return ret;
}

int nvs_dsm_iio(int dev_id, bool connect, int snsr_id, unsigned char *uuid)
{
	struct dsm_state *st = dsm_state_local;
	int ret = -EPERM;

	if (st == NULL)
		return -EPERM;

	if (st->nvs[DSM_KIF_IIO] && st->nvs_st[DSM_KIF_IIO])
		ret = dsm_push(st, dev_id, connect, snsr_id, uuid,
			       DSM_KIF_IIO);
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&st->pd->dev, "%s err=%d\n", __func__, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(nvs_dsm_iio);

int nvs_dsm_input(int dev_id, bool connect, int snsr_id, unsigned char *uuid)
{
	struct dsm_state *st = dsm_state_local;
	int ret = -EPERM;

	if (st == NULL)
		return -EPERM;

	if (st->nvs[DSM_KIF_INPUT] && st->nvs_st[DSM_KIF_INPUT])
		ret = dsm_push(st, dev_id, connect, snsr_id, uuid,
			       DSM_KIF_INPUT);
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&st->pd->dev, "%s err=%d\n", __func__, ret);
	return ret;
}

static int dsm_nvs_write(void *client, int snsr_id, unsigned int nvs)
{
	struct dsm_state *st = (struct dsm_state *)client;
	unsigned int val;

	switch (nvs & 0xFF) {
	case DSM_DBG_STS:
		return 0;

	case DSM_DBG_PUSH:
		val = (nvs >> 8) & 0xFF;
		if (val)
			/* 0 based index */
			val--;
		else
			/* kif = auto: use last used */
			val = st->kif;
		if (val < ARRAY_SIZE(nvs_kif) && nvs_kif[val])
			return dsm_push_msg(st, val, NULL);

		return -EINVAL;

	case DSM_DBG_MSG_FLAGS:
		st->msg.flags = (nvs >> 8) & 0xFF;
		val = (nvs >> 16) & 0xFF;
		if (val)
			st->msg.ver = val;
		else
			st->msg.ver = sizeof(st->msg);
		return 0;

	case DSM_DBG_MSG_DEV_ID:
		st->msg.dev_id = nvs >> 8;
		return 0;

	case DSM_DBG_MSG_SNSR_ID:
		st->msg.snsr_id = nvs >> 8;
		return 0;

	case DSM_DBG_UUID:
		val = (nvs >> 16) & 0xF;
		st->msg.uuid[val] = (nvs >> 8) & 0xFF;
		return 0;

	case DSM_DBG_PUSH_DELAY_MS:
		st->delay_ms = nvs >> 8;
		return 0;

	case DSM_DBG_PUSH_DELAY_N:
		st->delay_n = nvs >> 8;
		return 0;
	}

	return -EINVAL;
}

static int dsm_nvs_read(void *client, int snsr_id, char *buf)
{
	struct dsm_state *st = (struct dsm_state *)client;
	int i;
	ssize_t t;

	t = snprintf(buf, PAGE_SIZE, "driver v.%u\n", DSM_DRIVER_VERSION);
	t += snprintf(buf + t, PAGE_SIZE - t, "Device tree:\n");
	t += snprintf(buf + t, PAGE_SIZE - t,
		      "   push_delay_ms=%ums\n", st->delay_ms);
	t += snprintf(buf + t, PAGE_SIZE - t,
		      "   push_delay_count=%u\n", st->delay_n);
	t += snprintf(buf + t, PAGE_SIZE - t,
		      "connections=%u\n", st->connect_n);
	t += snprintf(buf + t, PAGE_SIZE - t,
		      "disconnections=%u\n", st->disconnect_n);
	if (st->kif < DSM_KIF_N) {
		t += snprintf(buf + t, PAGE_SIZE - t, "Last DSM message:\n");
		t += snprintf(buf + t, PAGE_SIZE - t,
			      "   msg ver/size: %u\n", st->msg.ver);
		t += snprintf(buf + t, PAGE_SIZE - t,
			      "   flags: 0x%X\n", st->msg.flags);
		t += snprintf(buf + t, PAGE_SIZE - t,
			      "   dev_id: %d\n", st->msg.dev_id);
		t += snprintf(buf + t, PAGE_SIZE - t,
			      "   snsr_id: %d\n", st->msg.snsr_id);
		t += snprintf(buf + t, PAGE_SIZE - t, "   uuid: ");
		for (i = 0; i < 16; i++)
			t += snprintf(buf + t, PAGE_SIZE - t, "%02x ",
				      st->msg.uuid[i]);
		t += snprintf(buf + t, PAGE_SIZE - t,
			      "\nmessage timestamp=%lld\n", st->msg_ts);
		if (st->msg_err > 0)
			t += snprintf(buf + t, PAGE_SIZE - t,
				      "message delay pending=%dms\n",
				      st->msg_err * st->delay_ms);
		else
			t += snprintf(buf + t, PAGE_SIZE - t,
				      "message error=%d\n", st->msg_err);
		t += snprintf(buf + t, PAGE_SIZE - t, "KIF[%u] %s\n",
			      st->kif + 1, dsm_kif_str[st->kif]);
	}
	t += snprintf(buf + t, PAGE_SIZE - t, "Supported KIFs:\n");
	for (i = 0; i < DSM_KIF_N; i++) {
		if (nvs_kif[i])
			t += snprintf(buf + t, PAGE_SIZE - t,
				      "   KIF[%u]=%s \n", i + 1,
				      dsm_kif_str[i]);
	}
	return t;
}

static struct nvs_fn_dev dsm_fn_dev = {
	.nvs_write			= dsm_nvs_write,
	.nvs_read			= dsm_nvs_read,
};

static void dsm_shutdown(struct platform_device *pd)
{
	struct dsm_state *st = (struct dsm_state *)dev_get_drvdata(&pd->dev);
	unsigned int i;

	dsm_state_local = NULL;
	for (i = 0; i < ARRAY_SIZE(nvs_kif); i++) {
		if (st->nvs[i] && st->nvs_st[i])
			st->nvs[i]->shutdown(st->nvs_st[i]);
	}
}

static int dsm_remove(struct platform_device *pd)
{
	struct dsm_state *st = (struct dsm_state *)dev_get_drvdata(&pd->dev);
	unsigned int i;

	if (st != NULL) {
		dsm_shutdown(pd);
		for (i = 0; i < ARRAY_SIZE(nvs_kif); i++) {
			if (st->nvs[i] && st->nvs_st[i])
				st->nvs[i]->remove(st->nvs_st[i]);
		}
	}
	dev_info(&pd->dev, "%s\n", __func__);
	return 0;
}

static struct sensor_cfg dsm_sensor_cfg = {
	.name			= "dynamic_sensor_meta",
	.kbuf_sz		= 32,
	.ch_n			= 1,
	.ch_sz			= sizeof(struct nvs_dsm_msg),
	.part			= DSM_NAME,
	.vendor			= DSM_VENDOR,
	.version		= DSM_DRIVER_VERSION,
	.flags			= SENSOR_FLAG_SPECIAL_REPORTING_MODE,
};

static int dsm_of_dt(struct dsm_state *st)
{
	st->kif = -1;
	st->delay_ms = DSM_PUSH_DELAY_MS;
	st->delay_n = DSM_PUSH_DELAY_N;
	of_property_read_u32(st->pd->dev.of_node, "dsm_push_delay_ms",
			     &st->delay_ms);
	of_property_read_u32(st->pd->dev.of_node, "dsm_push_delay_count",
			     &st->delay_n);
	memcpy(&st->cfg, &dsm_sensor_cfg, sizeof(st->cfg));
	return nvs_of_dt(st->pd->dev.of_node, &st->cfg, NULL);
}

static int dsm_init(struct dsm_state *st)
{
	unsigned int i;
	unsigned int n;
	int ret;

	ret = dsm_of_dt(st);
	if (ret < 0) {
		if (ret == -ENODEV) {
			dev_info(&st->pd->dev, "%s DT disabled\n", __func__);
		} else {
			dev_err(&st->pd->dev, "%s _of_dt ERR\n", __func__);
			ret = -ENODEV;
		}
		return ret;
	}

	n = 0;
	for (i = 0; i < ARRAY_SIZE(nvs_kif); i++) {
		if (nvs_kif[i]) {
			st->nvs[i] = nvs_kif[i]();
			if (!st->nvs[i]) {
				dev_err(&st->pd->dev, "%s nvs_kif[%u] ERR\n",
					__func__, i);
				continue;
			}

			ret = st->nvs[i]->probe(&st->nvs_st[i],
						st, &st->pd->dev,
						&dsm_fn_dev, &st->cfg);
			if (ret) {
				dev_err(&st->pd->dev, "%s nvs_probe ERR\n",
					__func__);
				st->nvs[i] = NULL;
				st->nvs_st[i] = NULL;
				continue;
			}

			n++;
		}
	}

	if (!n)
		return -ENODEV;

	INIT_DELAYED_WORK(&st->dw, dsm_work);
	return 0;
}

static int dsm_probe(struct platform_device *pd)
{
	struct dsm_state *st;
	int ret;

	dev_info(&pd->dev, "%s\n", __func__);
	st = devm_kzalloc(&pd->dev, sizeof(*st), GFP_KERNEL);
	if (!st) {
		dev_err(&pd->dev, "%s devm_kzalloc ERR\n", __func__);
		return -ENOMEM;
	}

	dev_set_drvdata(&pd->dev, st);
	st->pd = pd;
	dsm_fn_dev.sts = &st->sts;
	mutex_init(&st->mutex);
	ret = dsm_init(st);
	if (ret < 0)
		dsm_remove(pd);
	else
		dsm_state_local = st;
	dev_info(&pd->dev, "%s done\n", __func__);
	return ret;
}

static const struct of_device_id dsm_of_match[] = {
	{ .compatible = "nvidia,nvs_dsm", },
	{},
};

MODULE_DEVICE_TABLE(of, dsm_of_match);

static struct platform_driver dsm_driver = {
	.probe		= dsm_probe,
	.remove		= dsm_remove,
	.shutdown	= dsm_shutdown,
	.driver = {
		.name		= DSM_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(dsm_of_match),
	},
};
module_platform_driver(dsm_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("NVidia Sensor Dynamic Sensor Meta driver");
MODULE_AUTHOR("NVIDIA Corporation");

