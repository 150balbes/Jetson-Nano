/*
 * Copyright (c) 2017, NVIDIA CORPORATION. All rights reserved.
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

#include <dt-bindings/soc/tegra194-powergate.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/tegra-powergate.h>
#include <soc/tegra/bpmp_abi.h>
#include <soc/tegra/tegra_bpmp.h>
#include <soc/tegra/tegra-powergate-driver.h>

struct pg_partition_info {
	const char *name;
	int refcount;
	int run_refcount;
	struct mutex pg_mutex;
};

static struct pg_partition_info t194_partition_info[] = {
	[TEGRA194_POWER_DOMAIN_AUD] = { .name = "aud" },
	[TEGRA194_POWER_DOMAIN_DISP] = { .name = "disp" },
	[TEGRA194_POWER_DOMAIN_DISPB] = { .name = "dispb" },
	[TEGRA194_POWER_DOMAIN_DISPC] = { .name = "dispc" },
	[TEGRA194_POWER_DOMAIN_ISPA] = { .name = "ispa" },
	[TEGRA194_POWER_DOMAIN_NVDECA] = { .name = "nvdeca" },
	[TEGRA194_POWER_DOMAIN_NVJPG] = { .name = "nvjpg" },
	[TEGRA194_POWER_DOMAIN_NVENCA] = { .name = "nvenca" },
	[TEGRA194_POWER_DOMAIN_NVENCB] = { .name = "nvencb" },
	[TEGRA194_POWER_DOMAIN_NVDECB] = { .name = "nvdecb" },
	[TEGRA194_POWER_DOMAIN_SAX] = { .name = "sax" },
	[TEGRA194_POWER_DOMAIN_VE] = { .name = "ve" },
	[TEGRA194_POWER_DOMAIN_VIC] = { .name = "vic" },
	[TEGRA194_POWER_DOMAIN_XUSBA] = { .name = "xusba" },
	[TEGRA194_POWER_DOMAIN_XUSBB] = { .name = "xusbb" },
	[TEGRA194_POWER_DOMAIN_XUSBC] = { .name = "xusbc" },
	[TEGRA194_POWER_DOMAIN_PCIEX8A] = { .name = "pciex8a" },
	[TEGRA194_POWER_DOMAIN_PCIEX4A] = { .name = "pciex4a" },
	[TEGRA194_POWER_DOMAIN_PCIEX1A] = { .name = "pciex1a" },
	[TEGRA194_POWER_DOMAIN_NVL] = { .name = "nvl" },
	[TEGRA194_POWER_DOMAIN_PCIEX8B] = { .name = "pciex8b" },
	[TEGRA194_POWER_DOMAIN_PVAA] = { .name = "pvaa" },
	[TEGRA194_POWER_DOMAIN_PVAB] = { .name = "pvab" },
	[TEGRA194_POWER_DOMAIN_DLAA] = { .name = "dlaa" },
	[TEGRA194_POWER_DOMAIN_DLAB] = { .name = "dlab" },
	[TEGRA194_POWER_DOMAIN_CV] = { .name = "cv" },
	[TEGRA194_POWER_DOMAIN_GPU] = { .name = "gpu" },
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

static int tegra194_pg_query_abi(void)
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

static int tegra194_pg_powergate_partition(int id)
{
	int ret = 0;
	struct pg_partition_info *partition =
		&t194_partition_info[id];

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

static int tegra194_pg_unpowergate_partition(int id)
{
	int ret = 0;
	struct pg_partition_info *partition =
		&t194_partition_info[id];

	mutex_lock(&partition->pg_mutex);
	if (partition->refcount++ == 0)
		ret = pg_set_state(id, PG_STATE_ON);
	mutex_unlock(&partition->pg_mutex);

	return ret;
}

static const char *tegra194_pg_get_name(int id)
{
	return t194_partition_info[id].name;
}

static bool tegra194_pg_is_powered(int id)
{
	int ret;
	struct mrq_pg_request req = {
		.cmd = CMD_PG_GET_STATE,
		.id = id,
	};
	struct mrq_pg_response resp;

	ret = tegra_bpmp_send_receive(MRQ_PG, &req, sizeof(req),
			&resp, sizeof(resp));
	if (ret)
		return false;

	if (resp.get_state.state == PG_STATE_OFF)
		return false;

	return true;
}

static int tegra194_init_refcount(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(t194_partition_info); ++i)
		mutex_init(&t194_partition_info[i].pg_mutex);

	return 0;
}

static bool tegra194_powergate_id_is_valid(int id)
{
	return (id >= 1) && (id <= TEGRA194_POWER_DOMAIN_MAX);
}

static struct tegra_powergate_driver_ops tegra194_pg_ops = {
	.soc_name = "tegra194",
	.num_powerdomains = TEGRA194_POWER_DOMAIN_MAX + 1,
	.get_powergate_domain_name = tegra194_pg_get_name,
	.powergate_id_is_soc_valid = tegra194_powergate_id_is_valid,
	.powergate_init_refcount = tegra194_init_refcount,
	.powergate_is_powered = tegra194_pg_is_powered,
	.powergate_partition = tegra194_pg_powergate_partition,
	.unpowergate_partition = tegra194_pg_unpowergate_partition,
};

struct tegra_powergate_driver_ops *tegra194_powergate_init_chip_support(void)
{
	if (tegra194_pg_query_abi()) {
		WARN(1, "Missing BPMP support for MRQ_PG\n");
		return NULL;
	}

	return &tegra194_pg_ops;
}
