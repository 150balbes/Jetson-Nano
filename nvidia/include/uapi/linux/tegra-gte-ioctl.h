/*
 * Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef __UAPI_TEGRA_GTE_H
#define __UAPI_TEGRA_GTE_H

#include <linux/ioctl.h>
#include <linux/types.h>

/**
 * GPIO event types
 */
#define TEGRA_GTE_EVENT_RISING_EDGE	0x1
#define TEGRA_GTE_EVENT_FALLING_EDGE	0x2
#define TEGRA_GTE_EVENT_REQ_BOTH_EDGES	(TEGRA_GTE_EVENT_RISING_EDGE | \
					 TEGRA_GTE_EVENT_FALLING_EDGE)

/**
 * Information about a GPIO event request
 * @global_gpio_pin: global gpio pin number to monitor event
 * @eventflags: desired flags for the desired GPIO event line, such as
 * EVENT_RISING_EDGE or EVENT_FALLING_EDGE
 * @fd: if successful this field will contain a valid anonymous file handle
 * after a HTS_CREATE_GPIO_EVENT_IOCTL operation, zero or negative value
 * means error
 */
struct tegra_gte_hts_event_req {
	__u32 global_gpio_pin;
	__u32 eventflags;
	int fd;
};

/**
 * struct hts_event_data - event data
 * @timestamp: hardware timestamp in nanosecond
 * @dir: direction of the event
 */

struct tegra_gte_hts_event_data {
	__u64 timestamp;
	int dir;
};

/**
 * Event request IOCTL command
 */
#define TEGRA_GTE_HTS_CREATE_GPIO_EV_IOCTL \
					_IOWR(0xB5, 0x0, \
					      struct tegra_gte_hts_event_req)

#endif
