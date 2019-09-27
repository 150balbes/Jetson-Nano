/*
 * drivers/video/tegra/host/nvhost_pd.c
 *
 * Tegra Graphics Host Legacy Power Domain Provider
 *
 * Copyright (c) 2017-2018, NVIDIA Corporation.  All rights reserved.
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

#include <linux/pm_domain.h>
#include <linux/slab.h>
#include <linux/tegra-powergate.h>

#include <soc/tegra/chip-id.h>

#include <trace/events/nvhost.h>

#include "dev.h"
#include "nvhost_pd.h"

static int nvhost_module_power_on(struct generic_pm_domain *domain);
static int nvhost_module_power_off(struct generic_pm_domain *domain);

struct nvhost_pm_domain {
	struct generic_pm_domain domain;
	struct list_head list;
	int powergate_id;
};

static struct nvhost_pm_domain *genpd_to_nvhost_pd(
	struct generic_pm_domain *genpd)
{
	return container_of(genpd, struct nvhost_pm_domain, domain);
}

static int do_powergate_locked(int id)
{
	int ret;

	nvhost_dbg_fn("%d", id);
	if (id == -1 || !tegra_powergate_is_powered(id))
		return 0;

	ret = tegra_powergate_partition(id);
	if (ret && tegra_platform_is_sim()) {
		pr_err("%s: running on simulator, ignoring powergate failure\n",
		       __func__);
		ret = 0;
	}

	return ret;
}

static int do_unpowergate_locked(int id)
{
	int ret;

	if (id == -1)
		return 0;

	ret = tegra_unpowergate_partition(id);
	if (ret) {
		pr_err("%s: unpowergate failed: id = %d\n", __func__, id);
		if (tegra_platform_is_sim()) {
			pr_err("%s: running on simulator, ignoring failure\n",
			       __func__);
			ret = 0;
		}
	}

	return ret;
}

static int nvhost_module_power_on(struct generic_pm_domain *domain)
{
	struct nvhost_pm_domain *pd = genpd_to_nvhost_pd(domain);

	trace_nvhost_module_power_on(pd->domain.name, pd->powergate_id);
	return do_unpowergate_locked(pd->powergate_id);
}

static int nvhost_module_power_off(struct generic_pm_domain *domain)
{
	struct nvhost_pm_domain *pd = genpd_to_nvhost_pd(domain);

	trace_nvhost_module_power_off(pd->domain.name, pd->powergate_id);
	return do_powergate_locked(pd->powergate_id);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0)
static int pm_subdomain_attach(struct device_node *node)
{
	int ret;
	struct of_phandle_args master_phandle, child_phandle;

	child_phandle.np = node;
	child_phandle.args_count = 0;

	ret = of_parse_phandle_with_args(node, "power-domains",
					 "#power-domain-cells", 0,
					 &master_phandle);
	if (ret < 0)
		return ret;

	pr_info("Adding domain %s to PM domain %s\n",
		node->name, master_phandle.np->name);

	return of_genpd_add_subdomain(&master_phandle, &child_phandle);
}
#endif

static LIST_HEAD(pd_list);

static int _nvhost_init_domain(struct device_node *np)
{
	struct nvhost_pm_domain *pd;
	struct list_head *head;

	list_for_each(head, &pd_list) {
		struct nvhost_pm_domain *list_pd;
		list_pd = list_entry(head, struct nvhost_pm_domain, list);
		/* Domain already registered somewhere else */
		if (list_pd->domain.name == np->name)
			return 0;
	}

	pd = kzalloc(sizeof(*pd), GFP_KERNEL);
	if (!pd) {
		nvhost_err(NULL, "failed to allocate pd structure");
		return -ENOMEM;
	}

	if (of_property_read_u32(np, "partition-id", &pd->powergate_id))
		pd->powergate_id = -1;

	pd->domain.name = (char *)np->name;

	do_powergate_locked(pd->powergate_id);

	pm_genpd_init(&pd->domain, NULL, true);

	pd->domain.power_off = nvhost_module_power_off;
	pd->domain.power_on = nvhost_module_power_on;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 18, 0) && LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	pd->domain.flags = GENPD_FLAG_PM_UPSTREAM;
#endif

	of_genpd_add_provider_simple(np, &pd->domain);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	genpd_pm_subdomain_attach(&pd->domain);
#else
	pm_subdomain_attach(np);
#endif

	list_add(&pd->list, &pd_list);

	return 0;
}

int nvhost_domain_init(struct of_device_id *matches)
{
	struct device_node *np;
	int ret = 0;

	for_each_matching_node(np, matches) {
		ret = _nvhost_init_domain(np);
		if (ret)
			break;
	}
	return ret;

}
EXPORT_SYMBOL(nvhost_domain_init);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
void nvhost_pd_slcg_install_workaround(struct nvhost_device_data *pdata,
				       struct generic_pm_domain *genpd)
{
	struct nvhost_pm_domain *pd = genpd_to_nvhost_pd(genpd);

	/* needed to WAR MBIST issue */
	if (pdata->poweron_toggle_slcg || pdata->slcg_notifier_enable) {
		if (pd->powergate_id != -1)
			slcg_register_notifier(pd->powergate_id,
					       &pdata->toggle_slcg_notifier);
	}
}

void nvhost_pd_slcg_remove_workaround(struct nvhost_device_data *pdata,
				      struct generic_pm_domain *genpd)
{
	struct nvhost_pm_domain *pd = genpd_to_nvhost_pd(genpd);

	if ((pdata->poweron_toggle_slcg || pdata->slcg_notifier_enable) &&
	    pd->powergate_id != 1)
	{
		slcg_unregister_notifier(pd->powergate_id,
					 &pdata->toggle_slcg_notifier);
	}
}
#endif
