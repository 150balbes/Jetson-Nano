/*
 * tegra_gte_mon - monitor GPIO line events from userspace and hardware
 * timestamp.
 *
 * Copyright (C) 2020 Dipen Patel
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * Example Usage:
 *	tegra_gte_mon -d <device> -g <global gpio pin> -r -f
 */

#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <dirent.h>
#include <errno.h>
#include <string.h>
#include <poll.h>
#include <fcntl.h>
#include <getopt.h>
#include <inttypes.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <linux/tegra-gte-ioctl.h>

int monitor_device(const char *device_name,
		   unsigned int gnum,
		   unsigned int eventflags,
		   unsigned int loops)
{
	struct tegra_gte_hts_event_req req = {0};
	struct tegra_gte_hts_event_data event;
	char *chrdev_name;
	int fd;
	int ret;
	int i = 0;

	ret = asprintf(&chrdev_name, "/dev/%s", device_name);
	if (ret < 0)
		return -ENOMEM;

	fd = open(chrdev_name, 0);
	if (fd == -1) {
		ret = -errno;
		perror("Error: ");
		goto exit_close_error;
	}

	req.global_gpio_pin = gnum;
	req.eventflags = eventflags;

	ret = ioctl(fd, TEGRA_GTE_HTS_CREATE_GPIO_EV_IOCTL, &req);
	if (ret == -1) {
		ret = -errno;
		fprintf(stderr, "Failed to issue GET EVENT "
			"IOCTL (%d)\n",
			ret);
		goto exit_close_error;
	}

	fprintf(stdout, "Monitoring line %d on %s\n", gnum, device_name);

	while (1) {
		ret = read(req.fd, &event, sizeof(event));
		if (ret == -1) {
			if (errno == -EAGAIN) {
				fprintf(stderr, "nothing available\n");
				continue;
			} else {
				ret = -errno;
				fprintf(stderr, "Failed to read event (%d)\n",
					ret);
				break;
			}
		}

		if (ret != sizeof(event)) {
			fprintf(stderr, "Reading event failed\n");
			ret = -EIO;
			break;
		}

		fprintf(stdout, "HW timestamp GPIO EVENT %" PRIu64 "\n",
			event.timestamp);

		i++;
		if (i == loops)
			break;
	}

exit_close_error:
	if (close(fd) == -1)
		perror("Failed to close GPIO character device file");
	free(chrdev_name);
	return ret;
}

void print_usage(char *bin_name)
{
	fprintf(stderr, "Usage: %s [options]...\n"
		"Listen to events on GPIO lines, 0->1 1->0\n"
		"  -d <name>  Listen using named HW ts engine device\n"
		"  -g <n>     GPIO global id\n"
		"  -r         Listen for rising edges\n"
		"  -f         Listen for falling edges\n"
		" [-c <n>]    Do <n> loops (optional, infinite loop if not stated)\n"
		"  -h         This helptext\n"
		"\n"
		"Example:\n"
		"%s -d gtechip0 -g 257 -r -f\n"
		"(means GPIO 257 rising and falling edge monitoring)\n",
		bin_name, bin_name
	);
}

int main(int argc, char **argv)
{
	const char *device_name = NULL;
	unsigned int gnum = -1;
	unsigned int loops = 0;
	unsigned int eventflags = 0;
	int c;

	while ((c = getopt(argc, argv, "c:g:d:rfh")) != -1) {
		switch (c) {
		case 'c':
			loops = strtoul(optarg, NULL, 10);
			break;
		case 'd':
			device_name = optarg;
			break;
		case 'g':
			gnum = strtoul(optarg, NULL, 10);
			break;
		case 'r':
			eventflags |= TEGRA_GTE_EVENT_RISING_EDGE;
			break;
		case 'f':
			eventflags |= TEGRA_GTE_EVENT_FALLING_EDGE;
			break;
		case 'h':
			print_usage(argv[0]);
			return 1;
		}
	}

	if (!device_name || gnum == -1) {
		print_usage(argv[0]);
		return 1;
	}

	if (!eventflags) {
		printf("No flags specified, listening on both rising and "
		       "falling edges\n");
		eventflags = TEGRA_GTE_EVENT_REQ_BOTH_EDGES;
	}
	return monitor_device(device_name, gnum, eventflags, loops);
}
