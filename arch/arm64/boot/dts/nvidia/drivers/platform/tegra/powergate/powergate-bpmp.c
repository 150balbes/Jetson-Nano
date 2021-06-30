/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION. All rights reserved
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
#include <linux/of.h>
#include <linux/pm_domain.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/version.h>

#include <soc/tegra/bpmp_abi.h>
#include <soc/tegra/tegra_bpmp.h>
#include <soc/tegra/tegra_powergate.h>

#define to_tegra_bpmp_pd(domain) \
	container_of(domain, struct tegra_bpmp_pd, genpd)

struct tegra_bpmp_pd {
	struct generic_pm_domain genpd;
	int id;
};

struct tegra_bpmp_pg {
	struct device *dev;
	struct genpd_onecell_data genpd_data;
	struct generic_pm_domain *domains[];
};

static int tegra_bpmp_pg_set_state(int id, u32 state)
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

static int tegra_bpmp_pg_get_state(int id)
{
	int err;
	struct mrq_pg_request req = {
		.cmd = CMD_PG_GET_STATE,
		.id = id,
	};
	struct mrq_pg_response resp;

	err = tegra_bpmp_send_receive(MRQ_PG, &req, sizeof(req),
				      &resp, sizeof(resp));
	if (err)
		return PG_STATE_OFF;

	return resp.get_state.state;
}

static int tegra_bpmp_pg_get_max_id(void)
{
	int err;
	struct mrq_pg_request req = { .cmd = CMD_PG_GET_MAX_ID };
	struct mrq_pg_response resp;

	err = tegra_bpmp_send_receive(MRQ_PG, &req, sizeof(req),
				      &resp, sizeof(resp));
	if (err)
		return err;

	return resp.get_max_id.max_id;
}

static char *tegra_bpmp_pg_get_name(int id)
{
	int err;
	struct mrq_pg_request req = {
		.cmd = CMD_PG_GET_NAME,
		.id = id,
	};
	struct mrq_pg_response resp;

	err = tegra_bpmp_send_receive(MRQ_PG, &req, sizeof(req),
				      &resp, sizeof(resp));
	if (err)
		return NULL;

	return kstrdup(resp.get_name.name, GFP_KERNEL);
}

static inline bool tegra_bpmp_pg_is_off(int id)
{
	return tegra_bpmp_pg_get_state(id) == PG_STATE_OFF;
}

static int tegra_bpmp_pg_power_on(struct generic_pm_domain *domain)
{
	struct tegra_bpmp_pd *pd = to_tegra_bpmp_pd(domain);

	return tegra_bpmp_pg_set_state(pd->id, PG_STATE_ON);
}

static int tegra_bpmp_pg_power_off(struct generic_pm_domain *domain)
{
	struct tegra_bpmp_pd *pd = to_tegra_bpmp_pd(domain);

	return tegra_bpmp_pg_set_state(pd->id, PG_STATE_OFF);
}

static int tegra_bpmp_pg_add_domain(struct tegra_bpmp_pg *pg, int id)
{
	int err;
	char *name;
	struct tegra_bpmp_pd *pd;

	name = tegra_bpmp_pg_get_name(id);
	if (!name)
		return -EINVAL;

	pd = devm_kzalloc(pg->dev, sizeof(*pd), GFP_KERNEL);
	if (!pd) {
		err = -ENOMEM;
		goto out;
	}

	pd->id = id;
	pd->genpd.name = name;
	pd->genpd.power_on = tegra_bpmp_pg_power_on;
	pd->genpd.power_off = tegra_bpmp_pg_power_off;
	pm_genpd_init(&pd->genpd, NULL, tegra_bpmp_pg_is_off(id));

	pg->genpd_data.domains[id] = &pd->genpd;

	return 0;

out:
	kfree(name);
	return err;
}

static void tegra_bpmp_pg_remove_domain(struct tegra_bpmp_pg *pg, int id)
{
	struct generic_pm_domain *genpd = pg->genpd_data.domains[id];

	if (!genpd)
		return;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0)
	if (pm_genpd_remove(genpd) < 0)
		dev_err(pg->dev, "failed to remove %s\n", genpd->name);
#endif

	kfree(genpd->name);
}

#if defined(CONFIG_TEGRA_POWERGATE)
/*
 * As long as we live alongside the legacy tegra powergate framework,
 * only add the domains that refers to us from the device tree.
 */
static void tegra_bpmp_pg_add_domains(struct tegra_bpmp_pg *pg)
{
	int err;
	u32 phandle, id;
	struct device_node *np;
	struct device_node *pg_np = pg->dev->of_node;

	for_each_node_with_property(np, "power-domains") {
		err = of_property_read_u32_index(np, "power-domains", 0,
						 &phandle);
		if (err || phandle != pg_np->phandle)
			continue;
		err = of_property_read_u32_index(np, "power-domains", 1, &id);
		if (err)
			continue;

		/* Don't add a domain more than once */
		if (pg->genpd_data.domains[id])
			continue;

		err = tegra_bpmp_pg_add_domain(pg, id);
		if (err)
			dev_warn(pg->dev, "failed to add domain: %d\n", id);
	}
}
#else
static void tegra_bpmp_pg_add_domains(struct tegra_bpmp_pg *pg)
{
	int i, err;

	for (i = 0; i < pg->genpd_data.num_domains; i++) {
		err = tegra_bpmp_pg_add_domain(pg, i);
		/*
		 * All IDs between 0 and max ID are not guaranteed to exist,
		 * tegra_bpmp_pg_add_domain() returns EINVAL on the ones that
		 * do not exist. Thus, avoid printing the warning for that error
		 * code.
		 */
		if (err && err != -EINVAL)
			dev_warn(pg->dev, "failed to add domain: %d\n", i);
	}
}
#endif

int tegra_bpmp_init_powergate(struct platform_device *pdev)
{
	int i, max_id, err;
	struct tegra_bpmp_pg *pg;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	max_id = tegra_bpmp_pg_get_max_id();
	if (max_id < 0)
		return max_id;

	pg = devm_kzalloc(dev,
			  sizeof(*pg) + (max_id + 1) * sizeof(pg->domains[0]),
			  GFP_KERNEL);
	if (!pg)
		return -ENOMEM;

	pg->dev = dev;
	pg->genpd_data.domains = pg->domains;
	pg->genpd_data.num_domains = max_id + 1;

	tegra_bpmp_pg_add_domains(pg);

	err = of_genpd_add_provider_onecell(np, &pg->genpd_data);
	if (err) {
		dev_err(dev, "failed to add provider: %d\n", err);
		goto out;
	}

	return 0;

out:
	for (i = 0; i < pg->genpd_data.num_domains; i++) {
		tegra_bpmp_pg_remove_domain(pg, i);
	}

	return err;
}
