/*
 * HID driver for the NVIDIA BT Stand
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION, All rights reserved.
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
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/version.h>

#include "hid-ids.h"

/* Time difference tolerance in ms */
#define MAX_PACKET_DIFF_TOLERANCE 100
/*
 * Max time(in ms) below which packet delivery check is done.
 * Once this time passes, time difference checks is ignored as only 2 bytes are
 * used for getting time difference and it rounds off after 65 seconds.
 */
#define MAX_TIME_BETWEEN_PACKETS 65000

enum {
	RAVEN_KB_ID		= 1, /* Keyboard report ID */
	RAVEN_BM_ID		= 2, /* bitMapped report ID */
	RAVEN_TP_ID		= 3, /* TrackPad report ID */
	MAX_DEBUG_REPORTS	= RAVEN_TP_ID
};

struct raven_hid_debug_data {
	u8	seq_num;
	ktime_t	time;
};

struct raven_device {
	struct hid_device		*hdev;
	struct raven_hid_debug_data	debug_info[MAX_DEBUG_REPORTS];
};

static int raven_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	int ret, i;
	struct raven_device *raven_dev;

	raven_dev = devm_kzalloc(&hdev->dev, sizeof(*raven_dev), GFP_KERNEL);
	if (!raven_dev)
		return -ENOMEM;

	raven_dev->hdev = hdev;

	hid_set_drvdata(hdev, raven_dev);

	/* set seq num to 255 since first packet comes with seq number 0 */
	for (i = 0; i < MAX_DEBUG_REPORTS; i++)
		raven_dev->debug_info[i].seq_num = 0xFF;

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "hid parse failed\n");
		return ret;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret) {
		hid_err(hdev, "hw start failed\n");
		return ret;
	}
	return 0;
}

static void raven_remove(struct hid_device *hdev)
{
	hid_hw_stop(hdev);
}

static int raven_raw_event(struct hid_device *hdev, struct hid_report *report,
			   u8 *data, int size)
{
	struct raven_device *raven_dev = hid_get_drvdata(hdev);

	hid_dbg(hdev, "%s: report->id = 0x%x, size = %d\n",
		__func__, report->id, size);

	switch (report->id) {
	/* Fall through intentionally */
	case RAVEN_KB_ID:
	case RAVEN_BM_ID:
	case RAVEN_TP_ID:
	{
		/* Check Seq Number */
		struct raven_hid_debug_data *debug_info =
					&raven_dev->debug_info[report->id - 1];
		/* Last but 2nd is seq number and last 2 bytes are time diff */
		u8 seq = data[size - 3];
		u16 fw_time_diff =  data[size - 2] | (data[size - 1] << 8);
		ktime_t current_time = ktime_get();
		s64 time_diff = ktime_ms_delta(current_time,
							debug_info->time);
		u16 packet_delay = abs(time_diff - fw_time_diff);

		hid_dbg(hdev, "%s:report->id = 0x%x,seq %d diff %d",
			__func__, report->id, seq, fw_time_diff);

		if ((u8)(debug_info->seq_num + 1) != seq)
			hid_err(hdev, "%s: id:%d seq num mis Prev %d curr %d",
				__func__, report->id, debug_info->seq_num, seq);

		/* Ignore first packet time diff */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
		if (debug_info->time &&
		    time_diff < MAX_TIME_BETWEEN_PACKETS &&
		    packet_delay > MAX_PACKET_DIFF_TOLERANCE)
#else
		if (debug_info->time.tv64 &&
		    time_diff < MAX_TIME_BETWEEN_PACKETS &&
		    packet_delay > MAX_PACKET_DIFF_TOLERANCE)
#endif
			hid_err(hdev, "%s: id:%d Packet delay:%d ms at seq %d host diff :%lli ms fw diff %d",
				__func__, report->id, packet_delay, seq,
				time_diff, fw_time_diff);
		debug_info->seq_num = seq;
		debug_info->time = current_time;
	}
		break;
	default:
		break;
	}
	return 0;
}

static const struct hid_device_id raven_devices[] = {
	{HID_BLUETOOTH_DEVICE(USB_VENDOR_ID_NVIDIA,
			      USB_DEVICE_ID_NVIDIA_RAVEN)},
	{ }
};
MODULE_DEVICE_TABLE(hid, raven_devices);

static struct hid_driver raven_driver = {
	.name = "raven",
	.id_table = raven_devices,
	.raw_event = raven_raw_event,
	.probe = raven_probe,
	.remove = raven_remove,
};

module_hid_driver(raven_driver);

MODULE_LICENSE("GPL");
