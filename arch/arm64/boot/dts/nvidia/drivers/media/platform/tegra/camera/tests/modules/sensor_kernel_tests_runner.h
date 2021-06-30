/*
 * sensor_kernel_tests_runner - test runner for sensor kernel tests
 *
 * Copyright (c) 2019, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __SENSOR_KERNEL_TESTS_RUNNER_H__
#define __SENSOR_KERNEL_TESTS_RUNNER_H__

#define SKT_RUNNER_STR_LEN (256U + 1U)

struct device_node;

struct skt_test {
	const char *name;
	const char *description;
	int (*run)(struct device_node *node, const u32 tvcf_version);
};

struct skt_runner_ctx {
	char compat[SKT_RUNNER_STR_LEN];
	char name[SKT_RUNNER_STR_LEN];
	char glob[SKT_RUNNER_STR_LEN];
	u32 tvcf_version;
	u32 dest_portid;
};

int skt_runner_num_tests(void);
int skt_runner_query_tests(const char *glob, struct skt_test **tests,
		const int len);
int skt_runner_run_tests(const struct skt_runner_ctx *ctx);

#endif // __SENSOR_KERNEL_TESTS_RUNNER_H__
