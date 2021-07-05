/*
 * Copyright (c) 2015-2017, NVIDIA CORPORATION. All rights reserved
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files
 * (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <soc/tegra/tegra_powergate.h>
#include <soc/tegra/bpmp_abi.h>
#include <soc/tegra/tegra_bpmp.h>
#include <soc/tegra/tegra-powergate-driver.h>

struct pg_partition_info {
	const char *name;
	int refcount;
	int run_refcount;
	struct mutex pg_mutex;
};

static struct pg_partition_info t186_partition_info[] = {
	[TEGRA186_POWER_DOMAIN_AUD] = { .name = "audio" },
	[TEGRA186_POWER_DOMAIN_DFD] = { .name = "dfd" },
	[TEGRA186_POWER_DOMAIN_DISP] = { .name = "disp" },
	[TEGRA186_POWER_DOMAIN_DISPB] = { .name = "dispb" },
	[TEGRA186_POWER_DOMAIN_DISPC] = { .name = "dispc" },
	[TEGRA186_POWER_DOMAIN_ISPA] = { .name = "ispa" },
	[TEGRA186_POWER_DOMAIN_NVDEC] = { .name = "nvdec" },
	[TEGRA186_POWER_DOMAIN_NVJPG] = { .name = "nvjpg" },
	[TEGRA186_POWER_DOMAIN_MPE] = { .name = "nvenc" },
	[TEGRA186_POWER_DOMAIN_PCX] = { .name = "pcie" },
	[TEGRA186_POWER_DOMAIN_SAX] = { .name = "sata" },
	[TEGRA186_POWER_DOMAIN_VE] = { .name = "ve" },
	[TEGRA186_POWER_DOMAIN_VIC] = { .name = "vic" },
	[TEGRA186_POWER_DOMAIN_XUSBA] = { .name = "xusba" },
	[TEGRA186_POWER_DOMAIN_XUSBB] = { .name = "xusbb" },
	[TEGRA186_POWER_DOMAIN_XUSBC] = { .name = "xusbc" },
	[TEGRA186_POWER_DOMAIN_GPU] = { .name = "gpu" },
};

static int pg_set_state(int id, u32 state)
{
	struct mrq_pg_request req = {
		.cmd = CMD_PG_SET_STATE,
		.id = id,
		.set_state = {
			.state = state,
		}
	};

	return tegra_bpmp_send_receive(MRQ_PG, &req, sizeof(req), NULL, 0);
}

static int tegra186_pg_query_abi(void)
{
	int ret;
	struct mrq_query_abi_request req = { .mrq = MRQ_PG };
	struct mrq_query_abi_response resp;

	ret = tegra_bpmp_send_receive(MRQ_QUERY_ABI, &req, sizeof(req), &resp,
				      sizeof(resp));
	if (ret)
		return ret;

	return resp.status;
}

static int tegra186_pg_powergate_partition(int id)
{
	int ret = 0;
	struct pg_partition_info *partition =
		&t186_partition_info[id];

	mutex_lock(&partition->pg_mutex);
	if (partition->refcount) {
		if (--partition->refcount == 0)
			ret = pg_set_state(id, PG_STATE_OFF);
	} else {
		WARN(1, "partition %s refcount underflow\n",
		     partition->name);
	}
	mutex_unlock(&partition->pg_mutex);

	return ret;
}

static int tegra186_pg_unpowergate_partition(int id)
{
	int ret = 0;
	struct pg_partition_info *partition =
		&t186_partition_info[id];

	mutex_lock(&partition->pg_mutex);
	if (partition->refcount++ == 0)
		ret = pg_set_state(id, PG_STATE_ON);
	mutex_unlock(&partition->pg_mutex);

	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
static int tegra186_pg_powergate_clk_off(int id)
{
	int ret = 0;
	struct pg_partition_info *partition =
		&t186_partition_info[id];

	mutex_lock(&partition->pg_mutex);
	if (partition->refcount) {
		if (--partition->refcount == 0)
			ret = pg_set_state(id, PG_STATE_OFF);
		else if (partition->run_refcount == 1)
			ret = pg_set_state(id, PG_STATE_ON);

		if (partition->run_refcount)
			partition->run_refcount--;
	} else {
		WARN(1, "partition %s refcount underflow\n",
		     partition->name);
	}
	mutex_unlock(&partition->pg_mutex);

	return ret;
}

static int tegra186_pg_unpowergate_clk_on(int id)
{
	int ret = 0;
	struct pg_partition_info *partition =
		&t186_partition_info[id];

	mutex_lock(&partition->pg_mutex);
	if (partition->refcount++ == 0 || partition->run_refcount == 0)
		ret = pg_set_state(id, PG_STATE_RUNNING);
	partition->run_refcount++;
	mutex_unlock(&partition->pg_mutex);

	return ret;
}
#endif

static const char *tegra186_pg_get_name(int id)
{
	return t186_partition_info[id].name;
}

static bool tegra186_pg_is_powered(int id)
{
	int ret;
	struct mrq_pg_request req = {
		.cmd = CMD_PG_GET_STATE,
		.id = id,
	};
	struct mrq_pg_response resp;

	ret = tegra_bpmp_send_receive(MRQ_PG, &req, sizeof(req), &resp, sizeof(resp));
	if (ret)
		return false;

	if (resp.get_state.state == PG_STATE_OFF)
		return false;
	else
		return true;
}

static int tegra186_pg_force_powergate(int id)
{
	int ret;

	ret = pg_set_state(id, PG_STATE_ON);
	if (ret)
		return ret;

	return pg_set_state(id, PG_STATE_OFF);
}

static int tegra186_init_refcount(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(t186_partition_info); ++i)
		mutex_init(&t186_partition_info[i].pg_mutex);

	tegra186_pg_force_powergate(TEGRA186_POWER_DOMAIN_XUSBA);
	tegra186_pg_force_powergate(TEGRA186_POWER_DOMAIN_XUSBB);
	tegra186_pg_force_powergate(TEGRA186_POWER_DOMAIN_XUSBC);
	tegra186_pg_force_powergate(TEGRA186_POWER_DOMAIN_SAX);
	tegra186_pg_force_powergate(TEGRA186_POWER_DOMAIN_PCX);

	return 0;
}

static bool tegra186_powergate_id_is_valid(int id)
{
	if ((id < 0) || (id >= TEGRA186_POWER_DOMAIN_MAX))
		return false;

	return true;
}

static struct tegra_powergate_driver_ops tegra186_pg_ops = {
	.soc_name = "tegra186",

	.num_powerdomains = TEGRA186_POWER_DOMAIN_MAX,
	.powergate_id_is_soc_valid = tegra186_powergate_id_is_valid,

	.get_powergate_domain_name = tegra186_pg_get_name,

	.powergate_partition = tegra186_pg_powergate_partition,
	.unpowergate_partition = tegra186_pg_unpowergate_partition,

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	.powergate_partition_with_clk_off = tegra186_pg_powergate_clk_off,
	.unpowergate_partition_with_clk_on = tegra186_pg_unpowergate_clk_on,
#endif

	.powergate_is_powered = tegra186_pg_is_powered,
	.powergate_init_refcount = tegra186_init_refcount,
};

struct tegra_powergate_driver_ops *tegra186_powergate_init_chip_support(void)
{
	if (tegra186_pg_query_abi()) {
		WARN(1, "Missing BPMP support for MRQ_PG\n");
		return NULL;
	}

	return &tegra186_pg_ops;
}
