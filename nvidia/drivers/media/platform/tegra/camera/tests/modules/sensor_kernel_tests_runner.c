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

#include <linux/glob.h>
#include <linux/of.h>
#include <linux/string.h>
#include <media/tegracam_core.h>
#include <media/tegracam_utils.h>
#include <stdarg.h>

#include "sensor_kernel_tests_runner.h"
#include "sensor_kernel_tests_core.h"
#include "../tegracam_tests.h"
#include "../utils/tegracam_log.h"

#define SKT_TEST_RUN    "[ RUN      ]"
#define SKT_TEST_OK     "[       OK ]"
#define SKT_TEST_FAIL   "[     FAIL ]"
#define SKT_TEST_PASSED "[  PASSED  ]"
#define SKT_TEST_FAILED "[  FAILED  ]"
#define SKT_TEST_SEP    "[==========]"
#define SKT_TEST_BLANK  "[          ]"

#define SKT_TVCF_VERS_BUFF_SIZE (32U)
static char tvcf_vers_buff[SKT_TVCF_VERS_BUFF_SIZE];
static u32 dest_portid;

struct skt_test skt_available_tests[] = {
	{
		.name = "Sensor DT Test",
		.description = "Asserts compliance of sensor DT",
		.run = sensor_verify_dt,
	},
};

int skt_runner_num_tests(void)
{
	return ARRAY_SIZE(skt_available_tests);
}

int skt_runner_query_tests(const char *glob,
		struct skt_test **tests, const int len)
{
	int i;
	int ntests = 0;

	if (tests == NULL)
		return -1;

	for (i = 0; i < ARRAY_SIZE(skt_available_tests); i++) {
		if (ntests > len)
			break;

		if ((glob != NULL) &&
				!glob_match(glob, skt_available_tests[i].name))
			continue;

		tests[ntests++] = &skt_available_tests[i];
	}

	return ntests;
}

static int skt_runner_log(const char *fmt, ...)
{
	va_list args;
	int ret;

	va_start(args, fmt);
	ret = skt_core_vlog_msg(dest_portid, fmt, args);
	va_end(args);

	return ret;
}

static int skt_runner_test_log(const char *fmt, va_list args)
{
	int err;

	err = skt_core_log_msg(dest_portid, "%s ", SKT_TEST_BLANK);
	if (err != 0)
		return err;

	return skt_core_vlog_msg(dest_portid, fmt, args);
}

static int skt_runner_run_single_test(const struct skt_test *test,
		struct device_node *node,
		const u32 tvcf_version)
{
	int res;

	skt_runner_log(SKT_TEST_RUN " %s\n", test->name);

	res = test->run(node, tvcf_version);

	skt_runner_log("%s %s\n",
			(res == 0) ? SKT_TEST_OK : SKT_TEST_FAIL, test->name);

	return res;
}

static u32 skt_runner_query_version(struct device_node *node)
{
	u32 version;

	skt_runner_log("Note: No TVCF version specified - querying TVCF\n");
	version = tegracam_query_version(node->name);
	if (version == 0) {
		version = tegracam_version(1, 0, 0);
		skt_runner_log("Note: Sensor %s not registered with TVCF\n",
				node->name);
		skt_runner_log("      Falling back to TVCF version ");
	} else {
		skt_runner_log("Note: Sensor %s registered with TVCF\n",
				node->name);
		skt_runner_log("      Using queried TVCF version ");
	}

	return version;
}


static int skt_runner_test_node(struct device_node *node,
		const u32 req_version, const char *glob)
{
	u32 active_version;
	int res = 0;
	int ntests_ran = 0;
	int ntests_passed = 0;
	int i;

	/* Register callback to send output to userspace */
	if (camtest_try_acquire_global_log(skt_runner_test_log) != 0) {
		skt_runner_log("Unable to direct tegra camtest output\n");
		return -1;
	}

	if (req_version != 0) {
		active_version = req_version;
		skt_runner_log("Note: Using user-specified TVCF version ");
	} else
		active_version = skt_runner_query_version(node);

	format_tvcf_version(active_version, tvcf_vers_buff,
			SKT_TVCF_VERS_BUFF_SIZE);
	skt_runner_log("%s\n", tvcf_vers_buff);

	if (glob == NULL) {
		skt_runner_log(SKT_TEST_SEP " Starting tests for %s.\n",
				node->name);
	} else {
		skt_runner_log(SKT_TEST_SEP
				" Starting tests for %s with filter \"%s\".\n",
				node->name, glob);
	}

	for (i = 0; i < ARRAY_SIZE(skt_available_tests); i++) {
		if ((glob != NULL) &&
				!glob_match(glob, skt_available_tests[i].name))
			continue;

		res = skt_runner_run_single_test(&skt_available_tests[i],
				node, active_version);
		if (res == 0)
			ntests_passed++;
		ntests_ran++;
	}

	camtest_release_global_log();

	skt_runner_log(SKT_TEST_SEP " %d tests ran.\n", ntests_ran);
	skt_runner_log("%s %d of %d test(s) passed.\n\n",
		(ntests_ran == ntests_passed) ?
			SKT_TEST_PASSED : SKT_TEST_FAILED,
		ntests_passed, ntests_ran);

	return res;
}

int skt_runner_run_tests(const struct skt_runner_ctx *ctx)
{
	int res = 0;
	bool node_found = false;
	const char *glob;
	struct device_node *node;

	if (ctx == NULL)
		return -1;

	dest_portid = ctx->dest_portid;

	if ((strlen(ctx->compat) == 0) && (strlen(ctx->name) == 0)) {
		skt_runner_log("** Sensor compatible or name required! **\n");
		return -1;
	}

	if (strlen(ctx->glob) == 0)
		glob = NULL;
	else
		glob = ctx->glob;

	if (strlen(ctx->compat) != 0) {
		for_each_compatible_node(node, NULL, ctx->compat) {
			if ((strlen(ctx->name) != 0) &&
					!(strcmp(node->name, ctx->name) == 0))
				continue;

			node_found = true;
			res |= skt_runner_test_node(node, ctx->tvcf_version,
					glob);
			of_node_put(node);
		}
	} else if (strlen(ctx->name) != 0) {
		node = of_find_node_by_name(NULL, ctx->name);
		if (node != NULL) {
			node_found = true;
			res = skt_runner_test_node(node, ctx->tvcf_version,
					glob);
			of_node_put(node);
		}
	}

	if (!node_found) {
		skt_runner_log("** No valid sensors found! **\n");
		skt_runner_log("Search Criteria:\n");
		skt_runner_log("  sensor compatible: %s\n",
				(strlen(ctx->compat) == 0) ?
				"N/A" : ctx->compat);
		skt_runner_log("  sensor name: %s\n",
				(strlen(ctx->name) == 0) ?
				"N/A" : ctx->name);

		return -1;
	}

	skt_runner_log("** All tests complete **\n\n");

	return res;
}
