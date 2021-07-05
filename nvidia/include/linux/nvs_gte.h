/* Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef _NVS_GTE_IRQ_H_
#define _NVS_GTE_IRQ_H_

#include <linux/device.h>

struct nvs_gte_irq {
	void *gte;
	const char *dev_name;
	int gpio;
	int irq;
	u64 irq_ts;
	u64 err_n;
};

struct nvs_gte_sts {
	unsigned int ver;
	char *gte;
	char *err;
	u64 ts_ns;
	u64 ts_raw;
};

int nvs_gte_ts(struct nvs_gte_irq *ngi);
int nvs_gte_exit(struct device *dev, struct nvs_gte_irq *ngi, unsigned int n);
int nvs_gte_init(struct device *dev, struct nvs_gte_irq *ngi, unsigned int n);
int nvs_gte_sts(struct nvs_gte_irq *ngi, struct nvs_gte_sts *ngs);

#endif /* _NVS_GTE_IRQ_H_ */

