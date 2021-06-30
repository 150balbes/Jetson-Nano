/*
 * include/linux/iio/imu/tsfw_icm20628.h
 *
 * Copyright (c) 2016, NVIDIA CORPORATION, All rights reserved.
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

#ifndef __IIO_NVI_TS_MPU_H__
#define __IIO_NVI_TS_MPU_H__

#include <linux/nvs.h>
#include <linux/hid.h>

#define SENSOR_REPORT_ID_COMBINED	5
#define SENSOR_REPORT_ID		6
#define SENSOR_REPORT_ID_SYN		7

#define SNSR_N				3
#define SENSOR_DATA_SET_SIZE		18

struct tsfw_icm20628_snsr {
	void *nvs_st;
	struct sensor_cfg cfg;
};

struct tsfw_icm20628_state {
	struct nvs_fn_if *nvs;
	struct hid_device *hdev; /* keep ptr to hid_device to send HOSTCMDs */
	unsigned int sts;     /* TODO: needed? */
	unsigned int errs;    /* TODO: needed? */
	unsigned int enabled; /* TODO: move to tsfw_icm20628_snsr? */
	struct tsfw_icm20628_snsr snsr[SNSR_N];
	u8 report_counter;
	/* sensor data buffer for fragmented packets
	 * guaranteed to be < SENSOR_DATA_SET_SIZE
	 */
	u8 buffered_data[SENSOR_DATA_SET_SIZE];
	u8 buffered_cnt;
};

struct tsfw_icm20628_fn_dev {
	/* probe
	 * @hdev: hid device associated with sensor
	 * @pst: sensor state data
	 * Returns 0 on success or a negative error code.
	 */
	int (*probe)(struct hid_device *hdev, struct tsfw_icm20628_state **pst);

	/* recv
	 * @st: sensor state data
	 * @type: report type
	 * @data: sensor data from hid report
	 * @len: length of sensor data
	 * Returns 0 on success or a negative error code.
	 */
	int (*recv)(struct tsfw_icm20628_state *st, u8 *data, size_t len);

	/* remove
	 * @st: sensor state data
	 * Returns 0 on success or a negative error code.
	 */
	int (*remove)(struct tsfw_icm20628_state *st);
};

#ifdef CONFIG_TSFW_ICM
struct tsfw_icm20628_fn_dev *tsfw_icm20628_fns(void);
#else
inline struct tsfw_icm20628_fn_dev *tsfw_icm20628_fns(void) { return NULL; }
#endif

#endif
