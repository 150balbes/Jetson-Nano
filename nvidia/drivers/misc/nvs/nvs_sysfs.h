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

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/nvs.h>

#define NVS_ALIGN L1_CACHE_BYTES

struct nvs_buf_ch {
	unsigned int buf_i;
	unsigned int byte_n;
	bool sign;
};

struct nvs_dev_attr {
	struct device_attribute dev_attr;
	unsigned int channel;
};

#define to_nvs_dev_attr(_dev_attr) \
	container_of(_dev_attr, struct nvs_dev_attr, dev_attr)

struct nvs_state {
	void *client;
	struct nvs_kif_fn *kif_fn;
	struct device *dev;
	struct mutex mutex;
	struct nvs_fn_dev *fn_dev;
	struct sensor_cfg *cfg;
	struct nvs_dev_attr *attr_so;
	struct nvs_dev_attr *attr_ch;
	struct attribute **attrs;
	struct attribute_group attr_grp;
	struct nvs_buf_ch *buf_ch;
	unsigned int attr_so_n;
	unsigned int snsr_type;
	unsigned int ch_n;
	unsigned int enabled;
	unsigned int us_period;
	unsigned int us_timeout;
	unsigned int dbg;
	unsigned int fn_dev_sts;
	unsigned int fn_dev_errs;
	bool shutdown;
	bool suspend;
	bool flush;
	bool first_push;
	bool on_change;
	bool one_shot;
	bool special;
	s64 ts_diff;
	s64 ts;
	u64 dbg_data_lock;
	u8 *buf;
};

struct nvs_kif_fn {
	char *name;
	unsigned int driver_version;
	void (*remove)(struct nvs_state *st);
	int (*init)(struct nvs_state *st);
	int (*push)(struct nvs_state *st);
};

int nvs_probe(void **handle, void *dev_client, struct device *dev,
	      struct nvs_fn_dev *fn_dev, struct sensor_cfg *snsr_cfg,
	      struct nvs_kif_fn *kif_fn, unsigned int kif_st_n);
int nvs_remove(void *handle);
void nvs_shutdown(void *handle);
void nvs_mutex_lock(void *handle);
void nvs_mutex_unlock(void *handle);
int nvs_suspend(void *handle);
int nvs_resume(void *handle);
int nvs_handler(void *handle, void *buffer, s64 ts);

static inline void *nvs_st_kif(struct nvs_state *st)
{
	return (char *)st + ALIGN(sizeof(struct nvs_state), NVS_ALIGN);
}

