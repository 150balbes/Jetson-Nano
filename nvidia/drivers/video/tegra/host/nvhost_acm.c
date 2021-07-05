/*
 * drivers/video/tegra/host/nvhost_acm.c
 *
 * Tegra Graphics Host Automatic Clock Management
 *
 * Copyright (c) 2010-2020, NVIDIA Corporation. All rights reserved.
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

#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/reset.h>
#include <soc/tegra/chip-id.h>
#include <trace/events/nvhost.h>
#include <linux/tegra_pm_domains.h>
#include <uapi/linux/nvhost_ioctl.h>
#include <linux/version.h>
#include <linux/clk/tegra.h>
#include <linux/clk-provider.h>
#include <linux/dma-mapping.h>
#include <linux/nospec.h>

#include <linux/platform/tegra/mc.h>
#if defined(CONFIG_TEGRA_BWMGR)
#include <linux/platform/tegra/emc_bwmgr.h>
#endif

#include "vhost/vhost.h"
#include "nvhost_acm.h"
#include "nvhost_pd.h"
#include "nvhost_vm.h"
#include "nvhost_scale.h"
#include "nvhost_channel.h"
#include "dev.h"
#include "bus_client.h"
#include "chip_support.h"

#define ACM_SUSPEND_WAIT_FOR_IDLE_TIMEOUT	(2 * HZ)
#define POWERGATE_DELAY 			10
#define MAX_DEVID_LENGTH			32

static void nvhost_module_load_regs(struct platform_device *pdev, bool prod);
static void nvhost_module_set_actmon_regs(struct platform_device *pdev);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
static int nvhost_module_toggle_slcg(struct notifier_block *nb,
				     unsigned long action, void *data);
#endif

static int nvhost_module_prepare_suspend(struct device *dev);
static void nvhost_module_complete_resume(struct device *dev);
#if IS_ENABLED(CONFIG_PM_SLEEP)
static int nvhost_module_suspend(struct device *dev);
static int nvhost_module_resume(struct device *dev);
#endif
static int nvhost_module_runtime_suspend(struct device *dev);
static int nvhost_module_runtime_resume(struct device *dev);
static int nvhost_module_prepare_poweroff(struct device *dev);
static int nvhost_module_finalize_poweron(struct device *dev);

static DEFINE_MUTEX(client_list_lock);

struct nvhost_module_client {
	struct list_head node;
	unsigned long constraint[NVHOST_MODULE_MAX_CLOCKS];
	unsigned long type[NVHOST_MODULE_MAX_CLOCKS];
	void *priv;
};

static bool nvhost_module_emc_clock(struct nvhost_clock *clock)
{
	return (clock->moduleid ==
		NVHOST_MODULE_ID_EXTERNAL_MEMORY_CONTROLLER) ||
	       (clock->moduleid == NVHOST_MODULE_ID_EMC_SHARED);
}

static bool nvhost_is_bwmgr_clk(struct nvhost_device_data *pdata, int index)
{

	return (nvhost_module_emc_clock(&pdata->clocks[index]) &&
		pdata->bwmgr_handle);
}

static void do_module_reset_locked(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	if (pdata->reset) {
		pdata->reset(dev);
		return;
	}

	if (pdata->reset_control) {
		reset_control_reset(pdata->reset_control);
		return;
	}
}

static unsigned long nvhost_emc_bw_to_freq_req(unsigned long rate)
{
	return tegra_emc_bw_to_freq_req((unsigned long)(rate));
}

void nvhost_module_reset(struct platform_device *dev, bool reboot)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	dev_dbg(&dev->dev,
		"%s: asserting %s module reset\n",
		__func__, dev_name(&dev->dev));

	/* Ensure that the device state is sane (i.e. device specifics
	 * IRQs get disabled */
	if (reboot)
		if (pdata->prepare_poweroff)
			pdata->prepare_poweroff(dev);

	mutex_lock(&pdata->lock);
	do_module_reset_locked(dev);
	mutex_unlock(&pdata->lock);

	if (reboot) {
		/* Load clockgating registers */
		nvhost_module_load_regs(dev, pdata->engine_can_cg);

		/* Set actmon registers */
		nvhost_module_set_actmon_regs(dev);

		/* initialize device vm */
		nvhost_vm_init_device(dev);

		/* ..and execute engine specific operations (i.e. boot) */
		if (pdata->finalize_poweron)
			pdata->finalize_poweron(dev);
	}

	dev_dbg(&dev->dev, "%s: module %s out of reset\n",
		__func__, dev_name(&dev->dev));
}
EXPORT_SYMBOL(nvhost_module_reset);

void nvhost_module_busy_noresume(struct platform_device *dev)
{
	if (dev->dev.parent && (dev->dev.parent != &platform_bus))
		nvhost_module_busy_noresume(nvhost_get_parent(dev));

	pm_runtime_get_noresume(&dev->dev);
}

int nvhost_module_busy(struct platform_device *dev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	int ret = 0;

	/* Explicitly turn on the host1x clocks
	 * - This is needed as host1x driver sets ignore_children = true
	 * to cater the use case of display clock ON but host1x clock OFF
	 * in OS-Idle-Display-ON case
	 * - This was easily done in ACM as it only checked the ref count
	 * of host1x (or any device for that matter) to be zero before
	 * turning off its clock
	 * - However, runtime PM checks to see if *ANY* child of device is
	 * in ACTIVE state and if yes, it doesn't suspend the parent. As a
	 * result of this, display && host1x clocks remains ON during
	 * OS-Idle-Display-ON case
	 * - The code below fixes this use-case
	 */
	if (dev->dev.parent && (dev->dev.parent != &platform_bus))
		ret = nvhost_module_busy(nvhost_get_parent(dev));

	if (ret)
		return ret;

	down_read(&pdata->busy_lock);

	ret = pm_runtime_get_sync(&dev->dev);
	if (ret < 0) {
		pm_runtime_put_noidle(&dev->dev);
		up_read(&pdata->busy_lock);
		if (dev->dev.parent && (dev->dev.parent != &platform_bus))
			nvhost_module_idle(nvhost_get_parent(dev));
		nvhost_err(&dev->dev, "failed to power on, err %d", ret);
		return ret;
	}

	if (pdata->busy)
		pdata->busy(dev);

	up_read(&pdata->busy_lock);

	return 0;
}
EXPORT_SYMBOL(nvhost_module_busy);

inline void nvhost_module_idle(struct platform_device *dev)
{
	nvhost_module_idle_mult(dev, 1);
}
EXPORT_SYMBOL(nvhost_module_idle);

void nvhost_module_idle_mult(struct platform_device *dev, int refs)
{
	int original_refs = refs;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	/* call idle callback only if the device is turned on. */
	if (atomic_read(&dev->dev.power.usage_count) == refs &&
	    pm_runtime_active(&dev->dev)) {
		if (pdata->idle)
			pdata->idle(dev);
	}

	while (refs--) {
		pm_runtime_mark_last_busy(&dev->dev);
		if (pdata->autosuspend_delay)
			pm_runtime_put_autosuspend(&dev->dev);
		else
			pm_runtime_put(&dev->dev);
	}

	/* Explicitly turn off the host1x clocks */
	if (dev->dev.parent && (dev->dev.parent != &platform_bus))
		nvhost_module_idle_mult(nvhost_get_parent(dev), original_refs);
}

int nvhost_module_do_idle(struct device *dev)
{
	struct nvhost_device_data *pdata = dev_get_drvdata(dev);
	int ref_cnt;
	unsigned long end_jiffies;
	int err;

	/* acquire busy lock to block other busy() calls */
	down_write(&pdata->busy_lock);

	/* prevent suspend by incrementing usage counter */
	pm_runtime_get_sync(dev);

	/* check and wait until module is idle (with a timeout) */
	end_jiffies = jiffies + msecs_to_jiffies(2000);
	do {
		msleep(1);
		ref_cnt = atomic_read(&dev->power.usage_count);
	} while (ref_cnt != 1 && time_before(jiffies, end_jiffies));

	if (ref_cnt != 1) {
		nvhost_err(dev, "failed to idle - refcount %d != 1",
			ref_cnt);
		goto fail_drop_usage_count;
	}

	err = nvhost_module_runtime_suspend(dev);
	if (err)
		goto fail_drop_usage_count;

	pdata->forced_idle = true;

	return 0;

fail_drop_usage_count:
	pm_runtime_put_sync(dev);
	up_write(&pdata->busy_lock);

	return -EBUSY;
}

int nvhost_module_do_unidle(struct device *dev)
{
	struct nvhost_device_data *pdata = dev_get_drvdata(dev);
	int err = 0;

	if (!pdata->forced_idle)
		goto done;

	err = nvhost_module_runtime_resume(dev);

	/* balance usage counter */
	pm_runtime_put_sync(dev);
	pdata->forced_idle = false;

done:
	up_write(&pdata->busy_lock);
	return err;
}

static ssize_t force_idle_store(struct device *device,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	int err;
	unsigned long val = 0;

	if (kstrtoul(buf, 0, &val) < 0)
		return -EINVAL;

	if (val)
		err = nvhost_module_do_idle(device);
	else
		err = nvhost_module_do_unidle(device);

	if (err)
		return err;

	return count;
}

static ssize_t force_idle_read(struct device *device,
				struct device_attribute *attr, char *buf)
{
	struct nvhost_device_data *pdata = dev_get_drvdata(device);

	return sprintf(buf, "%d\n", pdata->forced_idle ? 1 : 0);
}

static DEVICE_ATTR(force_idle, 0744, force_idle_read, force_idle_store);

int nvhost_module_get_rate(struct platform_device *dev, unsigned long *rate,
		int index)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	index = array_index_nospec(index, NVHOST_MODULE_MAX_CLOCKS);

#if defined(CONFIG_TEGRA_BWMGR)
	if (nvhost_is_bwmgr_clk(pdata, index)) {
		*rate = tegra_bwmgr_get_emc_rate();
		return 0;
	}
#endif

	if (pdata->clk[index]) {
		/* Terrible and racy, but so is the whole concept of
		 * querying the clock rate from userspace.
		 */
		if (__clk_is_enabled(pdata->clk[index]))
			*rate = clk_get_rate(pdata->clk[index]);
		else
			*rate = 0;
	} else {
		nvhost_err(&dev->dev, "invalid clk index %d", index);
		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL(nvhost_module_get_rate);

static int nvhost_module_update_rate(struct platform_device *dev, int index)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	unsigned long bw_constraint = 0, floor_rate = 0, pixelrate = 0;
	unsigned long rate = 0;
	struct nvhost_module_client *m;
	int ret = -EINVAL;

	if (!nvhost_is_bwmgr_clk(pdata, index) && !pdata->clk[index]) {
		nvhost_err(&dev->dev, "invalid clk index %d", index);
		return -EINVAL;
	}

	/* don't update rate if scaling is disabled for this clock */
	if (pdata->clocks[index].disable_scaling)
		return 0;

	/* aggregate client constraints */
	list_for_each_entry(m, &pdata->client_list, node) {
		unsigned long constraint = m->constraint[index];
		unsigned long type = m->type[index];

		if (!constraint)
			continue;

		/* Note: We need to take max to avoid wrapping issues */
		if (type == NVHOST_BW)
			bw_constraint = max(bw_constraint,
				bw_constraint + (constraint / 1000));
		else if (type == NVHOST_PIXELRATE)
			pixelrate = max(pixelrate,
				pixelrate + constraint);
		else if (type == NVHOST_BW_KHZ)
			bw_constraint = max(bw_constraint,
				bw_constraint + constraint);
		else
			floor_rate = max(floor_rate, constraint);
	}

	/* use client specific aggregation if available */
	if (pdata->aggregate_constraints)
		rate = pdata->aggregate_constraints(dev, index, floor_rate,
						    pixelrate, bw_constraint);

	/* if frequency is not available, use default policy */
	if (!rate) {
		unsigned long bw_freq_khz =
			nvhost_emc_bw_to_freq_req(bw_constraint);
		bw_freq_khz = min(ULONG_MAX / 1000, bw_freq_khz);
		rate = max(floor_rate, bw_freq_khz * 1000);
		if (pdata->num_ppc)
			rate = max(rate, pixelrate / pdata->num_ppc);
	}

	/* take devfreq rate into account */
	rate = max(rate, pdata->clocks[index].devfreq_rate);

	/* if we still don't have any rate, use default */
	if (!rate)
		rate = pdata->clocks[index].default_rate;

	trace_nvhost_module_update_rate(dev->name, pdata->clocks[index].name,
					rate);

#if defined(CONFIG_TEGRA_BWMGR)
	if (nvhost_is_bwmgr_clk(pdata, index))
		ret = tegra_bwmgr_set_emc(pdata->bwmgr_handle, rate,
			pdata->clocks[index].bwmgr_request_type);
	else
#endif
		ret = clk_set_rate(pdata->clk[index], rate);

	return ret;

}

int nvhost_module_set_rate(struct platform_device *dev, void *priv,
		unsigned long constraint, int index, unsigned long type)
{
	struct nvhost_module_client *m;
	int ret = 0;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	nvhost_dbg_fn("%s", dev->name);

	index = array_index_nospec(index, NVHOST_MODULE_MAX_CLOCKS);

	mutex_lock(&client_list_lock);
	list_for_each_entry(m, &pdata->client_list, node) {
		if (m->priv == priv) {
			m->constraint[index] = constraint;
			m->type[index] = type;
		}
	}

	ret = nvhost_module_update_rate(dev, index);
	mutex_unlock(&client_list_lock);
	return ret;
}
EXPORT_SYMBOL(nvhost_module_set_rate);

int nvhost_module_add_client(struct platform_device *dev, void *priv)
{
	struct nvhost_module_client *client;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	nvhost_dbg_fn("%s num_clks=%d priv=%p", dev->name,
		      pdata->num_clks, priv);

	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client) {
		nvhost_err(&dev->dev,
			   "failed to allocate client structure");
		return -ENOMEM;
	}

	INIT_LIST_HEAD(&client->node);
	client->priv = priv;

	mutex_lock(&client_list_lock);
	list_add_tail(&client->node, &pdata->client_list);
	mutex_unlock(&client_list_lock);

	return 0;
}
EXPORT_SYMBOL(nvhost_module_add_client);

void nvhost_module_remove_client(struct platform_device *dev, void *priv)
{
	int i;
	struct nvhost_module_client *m;
	int found = 0;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	nvhost_dbg_fn("%s priv=%p", dev->name, priv);

	mutex_lock(&client_list_lock);
	list_for_each_entry(m, &pdata->client_list, node) {
		if (priv == m->priv) {
			list_del(&m->node);
			found = 1;
			break;
		}
	}
	if (found) {
		kfree(m);
		for (i = 0; i < pdata->num_clks; i++)
			nvhost_module_update_rate(dev, i);
	}
	mutex_unlock(&client_list_lock);
}
EXPORT_SYMBOL(nvhost_module_remove_client);

static ssize_t force_on_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int force_on = 0, ret = 0;
	struct nvhost_device_power_attr *power_attribute =
		container_of(attr, struct nvhost_device_power_attr,
			     power_attr[NVHOST_POWER_SYSFS_ATTRIB_FORCE_ON]);
	struct platform_device *dev = power_attribute->ndev;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	ret = sscanf(buf, "%d", &force_on);
	if (ret != 1)
		return -EINVAL;

	/* should we force the power on or off? */
	if (force_on && !pdata->forced_on) {
		/* force on */
		ret = nvhost_module_busy(dev);
		if (!ret)
			pdata->forced_on = true;
	} else if (!force_on && pdata->forced_on) {
		/* force off */
		nvhost_module_idle(dev);
		pdata->forced_on = false;
	}

	return count;
}

static ssize_t force_on_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct nvhost_device_power_attr *power_attribute =
		container_of(attr, struct nvhost_device_power_attr,
			     power_attr[NVHOST_POWER_SYSFS_ATTRIB_FORCE_ON]);
	struct platform_device *dev = power_attribute->ndev;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", pdata->forced_on);

}

static ssize_t autosuspend_delay_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	int autosuspend_delay = 0, ret = 0;
	struct nvhost_device_power_attr *power_attribute =
		container_of(attr, struct nvhost_device_power_attr, \
			power_attr[NVHOST_POWER_SYSFS_ATTRIB_AUTOSUSPEND_DELAY]);
	struct platform_device *dev = power_attribute->ndev;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	ret = sscanf(buf, "%d", &autosuspend_delay);
	if (ret == 1 && autosuspend_delay >= 0) {
		mutex_lock(&pdata->lock);
		pdata->autosuspend_delay = autosuspend_delay;
		mutex_unlock(&pdata->lock);

		pm_runtime_set_autosuspend_delay(&dev->dev,
				pdata->autosuspend_delay);
	} else {
		dev_err(&dev->dev, "Invalid autosuspend delay\n");
	}

	return count;
}

static ssize_t autosuspend_delay_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	int ret;
	struct nvhost_device_power_attr *power_attribute =
		container_of(attr, struct nvhost_device_power_attr, \
			power_attr[NVHOST_POWER_SYSFS_ATTRIB_AUTOSUSPEND_DELAY]);
	struct platform_device *dev = power_attribute->ndev;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	mutex_lock(&pdata->lock);
	ret = snprintf(buf, PAGE_SIZE, "%d\n", pdata->autosuspend_delay);
	mutex_unlock(&pdata->lock);

	return ret;
}

static ssize_t clk_cap_store(struct kobject *kobj,
	struct kobj_attribute *attr, const char *buf, size_t count)
{
	struct nvhost_device_data *pdata =
		container_of(kobj, struct nvhost_device_data, clk_cap_kobj);
	/* i is indeed 'index' here after type conversion */
	int ret, i = attr - pdata->clk_cap_attrs;
	struct clk *clk = pdata->clk[i];
	unsigned long freq_cap;

	ret = kstrtoul(buf, 0, &freq_cap);
	if (ret)
		return -EINVAL;

	ret = clk_set_max_rate(clk, freq_cap);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t clk_cap_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct nvhost_device_data *pdata =
		container_of(kobj, struct nvhost_device_data, clk_cap_kobj);
	/* i is indeed 'index' here after type conversion */
	int i = attr - pdata->clk_cap_attrs;
	struct clk *clk = pdata->clk[i];
	long max_rate;

	max_rate = clk_round_rate(clk, UINT_MAX);
	if (max_rate < 0)
		return max_rate;

	return snprintf(buf, PAGE_SIZE, "%ld\n", max_rate);
}

static void acm_kobj_release(struct kobject *kobj)
{
	sysfs_remove_dir(kobj);
}

static struct kobj_type acm_kobj_ktype = {
	.release    = acm_kobj_release,
	.sysfs_ops  = &kobj_sysfs_ops,
};

int nvhost_clk_get(struct platform_device *dev, char *name, struct clk **clk)
{
	int i;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

	for (i = 0; i < pdata->num_clks; i++) {
		if (strcmp(pdata->clocks[i].name, name) == 0) {
			*clk = pdata->clk[i];
			return 0;
		}
	}
	nvhost_err(&dev->dev,
		   "could not find clk %s", name);
	return -EINVAL;
}

static int nvhost_module_set_parent(struct platform_device *dev,
			     struct nvhost_clock *clock, struct clk *clk)
{
	struct clk *parent_clk;
	char parent_name[MAX_DEVID_LENGTH];
	int err;

	snprintf(parent_name, sizeof(parent_name), "%s_parent", clock->name);

	/* if parent is not available, assume that
	 * we do not need to change it.
	 */
	parent_clk = devm_clk_get(&dev->dev, parent_name);
	if (IS_ERR(parent_clk))
		return 0;

	err = clk_set_parent(clk, parent_clk);

	devm_clk_put(&dev->dev, parent_clk);

	return err;
}

int nvhost_module_init(struct platform_device *dev)
{
	int i = 0, err = 0;
	struct kobj_attribute *attr = NULL;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);
	struct nvhost_master *master = nvhost_get_host(dev);

	if (!pdata->no_platform_dma_mask) {
		dma_set_mask_and_coherent(&dev->dev, master->info.dma_mask);
	}

	/* initialize clocks to known state (=enabled) */
	pdata->num_clks = 0;
	INIT_LIST_HEAD(&pdata->client_list);

	if (nvhost_dev_is_virtual(dev)) {
		pm_runtime_enable(&dev->dev);

		/* If powergating is disabled, take a reference on the device
		 * without turning it on
		 */
		if (!pdata->can_powergate)
			nvhost_module_busy_noresume(dev);

		return err;
	}

#if defined(CONFIG_TEGRA_BWMGR)
	/* get bandwidth manager handle if needed */
	if (pdata->bwmgr_client_id)
		pdata->bwmgr_handle =
			tegra_bwmgr_register(pdata->bwmgr_client_id);
#endif

	while (i < NVHOST_MODULE_MAX_CLOCKS && pdata->clocks[i].name) {
		char devname[MAX_DEVID_LENGTH];
		long rate = pdata->clocks[i].default_rate;
		struct clk *c;

#if defined(CONFIG_TEGRA_BWMGR)
		if (nvhost_module_emc_clock(&pdata->clocks[i])) {
			if (nvhost_is_bwmgr_clk(pdata, i))
				tegra_bwmgr_set_emc(pdata->bwmgr_handle, 0,
					pdata->clocks[i].bwmgr_request_type);
			pdata->clk[pdata->num_clks++] = NULL;
			i++;
			continue;
		}
#endif

		snprintf(devname, MAX_DEVID_LENGTH,
			 (dev->id <= 0) ? "tegra_%s" : "tegra_%s.%d",
			 dev->name, dev->id);

		c = devm_clk_get(&dev->dev, pdata->clocks[i].name);

		if (IS_ERR(c)) {
			dev_err(&dev->dev, "clk_get failed for i=%d %s:%s",
				i, devname, pdata->clocks[i].name);
			/* arguably we should fail init here instead... */
			i++;
			continue;
		}

		nvhost_dbg_fn("%s->clk[%d] -> %s:%s:%p",
			      dev->name, pdata->num_clks,
			      devname, pdata->clocks[i].name,
			      c);

		/* Update parent */
		err = nvhost_module_set_parent(dev, &pdata->clocks[i], c);
		if (err)
			dev_warn(&dev->dev, "failed to set parent clock %s (err=%d)",
				 pdata->clocks[i].name, err);

		rate = clk_round_rate(c, rate);
		clk_set_rate(c, rate);
		clk_prepare_enable(c);

		pdata->clk[pdata->num_clks++] = c;
		i++;
	}
	pdata->num_clks = i;

	pdata->reset_control = devm_reset_control_get(&dev->dev, NULL);
	if (IS_ERR(pdata->reset_control))
		pdata->reset_control = NULL;

	/* reset the module */
	mutex_lock(&pdata->lock);
	do_module_reset_locked(dev);
	mutex_unlock(&pdata->lock);

	/* disable the clocks */
	for (i = 0; i < pdata->num_clks; ++i) {
		if (pdata->clk[i])
			clk_disable_unprepare(pdata->clk[i]);
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
	if (nvhost_is_210()) {
		struct generic_pm_domain *gpd = pd_to_genpd(dev->dev.pm_domain);

		/* needed to WAR MBIST issue */
		if (pdata->poweron_toggle_slcg) {
			pdata->toggle_slcg_notifier.notifier_call =
				&nvhost_module_toggle_slcg;
		}

		nvhost_pd_slcg_install_workaround(pdata, gpd);
	}
#endif

	/* set pm runtime delays */
	if (pdata->autosuspend_delay) {
		pm_runtime_set_autosuspend_delay(&dev->dev,
			pdata->autosuspend_delay);
		pm_runtime_use_autosuspend(&dev->dev);
	}

	/* initialize no_poweroff_req_mutex */
	mutex_init(&pdata->no_poweroff_req_mutex);
	init_rwsem(&pdata->busy_lock);

	/* turn on pm runtime */
	pm_runtime_enable(&dev->dev);
	if (!pm_runtime_enabled(&dev->dev))
		nvhost_module_enable_clk(&dev->dev);

	/* If powergating is disabled, take a reference on the device without
	 * turning it on
	 */
	if (!pdata->can_powergate)
		nvhost_module_busy_noresume(dev);

	/* Init the power sysfs attributes for this device */
	pdata->power_attrib = devm_kzalloc(&dev->dev,
		sizeof(struct nvhost_device_power_attr), GFP_KERNEL);
	if (!pdata->power_attrib) {
		dev_err(&dev->dev, "Unable to allocate sysfs attributes\n");
		return -ENOMEM;
	}
	pdata->power_attrib->ndev = dev;

	pdata->power_kobj = kobject_create_and_add("acm", &dev->dev.kobj);
	if (!pdata->power_kobj) {
		dev_err(&dev->dev, "Could not add dir 'power'\n");
		err = -EIO;
		goto fail_attrib_alloc;
	}

	attr = &pdata->power_attrib->power_attr[NVHOST_POWER_SYSFS_ATTRIB_AUTOSUSPEND_DELAY];
	attr->attr.name = "autosuspend_delay";
	attr->attr.mode = S_IWUSR | S_IRUGO;
	attr->show = autosuspend_delay_show;
	attr->store = autosuspend_delay_store;
	sysfs_attr_init(&attr->attr);
	if (sysfs_create_file(pdata->power_kobj, &attr->attr)) {
		dev_err(&dev->dev, "Could not create sysfs attribute autosuspend_delay\n");
		err = -EIO;
		goto fail_autosuspenddelay;
	}

	attr = &pdata->power_attrib->power_attr[NVHOST_POWER_SYSFS_ATTRIB_FORCE_ON];
	attr->attr.name = "force_on";
	attr->attr.mode = S_IWUSR | S_IRUGO;
	attr->show = force_on_show;
	attr->store = force_on_store;
	sysfs_attr_init(&attr->attr);
	if (sysfs_create_file(pdata->power_kobj, &attr->attr)) {
		dev_err(&dev->dev, "Could not create sysfs attribute force_on\n");
		err = -EIO;
		goto fail_forceon;
	}

	err = device_create_file(&dev->dev, &dev_attr_force_idle);
	if (err) {
		dev_err(&dev->dev, "Couldn't create device file force_idle\n");
		goto fail_force_idle;
	}

	err = kobject_init_and_add(&pdata->clk_cap_kobj, &acm_kobj_ktype,
		pdata->power_kobj, "%s", "clk_cap");
	if (err) {
		dev_err(&dev->dev, "Could not add dir 'clk_cap'\n");
		goto fail_clk_cap;
	}

	pdata->clk_cap_attrs = devm_kcalloc(&dev->dev, pdata->num_clks,
		sizeof(*attr), GFP_KERNEL);
	if (!pdata->clk_cap_attrs) {
		err = -ENOMEM;
		goto fail_clks;
	}

	for (i = 0; i < pdata->num_clks; ++i) {
		struct clk *c;

		c = pdata->clk[i];
		if (!c)
			continue;

		attr = &pdata->clk_cap_attrs[i];
		attr->attr.name = __clk_get_name(c);
		/* octal permission is preferred nowadays */
		attr->attr.mode = 0644;
		attr->show = clk_cap_show;
		attr->store = clk_cap_store;
		sysfs_attr_init(&attr->attr);
		if (sysfs_create_file(&pdata->clk_cap_kobj, &attr->attr)) {
			dev_err(&dev->dev, "Could not create sysfs attribute %s\n",
				__clk_get_name(c));
			err = -EIO;
			goto fail_clks;
		}
	}

	return 0;

fail_clks:
	/* kobj of acm_kobj_ktype cleans up sysfs entries automatically */
	kobject_put(&pdata->clk_cap_kobj);

fail_clk_cap:
	device_remove_file(&dev->dev, &dev_attr_force_idle);

fail_force_idle:
	attr = &pdata->power_attrib->power_attr[NVHOST_POWER_SYSFS_ATTRIB_FORCE_ON];
	sysfs_remove_file(pdata->power_kobj, &attr->attr);

fail_forceon:
	attr = &pdata->power_attrib->power_attr[NVHOST_POWER_SYSFS_ATTRIB_AUTOSUSPEND_DELAY];
	sysfs_remove_file(pdata->power_kobj, &attr->attr);

fail_autosuspenddelay:
	kobject_put(pdata->power_kobj);

fail_attrib_alloc:
	return err;
}
EXPORT_SYMBOL(nvhost_module_init);

void nvhost_module_deinit(struct platform_device *dev)
{
	int i;
	struct kobj_attribute *attr = NULL;
	struct nvhost_device_data *pdata = platform_get_drvdata(dev);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
	if (nvhost_is_210()) {
		struct generic_pm_domain *gpd = pd_to_genpd(dev->dev.pm_domain);

		nvhost_pd_slcg_remove_workaround(pdata, gpd);
	}
#endif

	devfreq_suspend_device(pdata->power_manager);

	if (pm_runtime_enabled(&dev->dev)) {
		pm_runtime_disable(&dev->dev);
	} else {
		/* call device specific poweroff call */
		if (pdata->booted && pdata->prepare_poweroff)
			pdata->prepare_poweroff(dev);

		/* disable clock.. */
		nvhost_module_disable_clk(&dev->dev);
	}

#if defined(CONFIG_TEGRA_BWMGR)
	if (pdata->bwmgr_handle)
		tegra_bwmgr_unregister(pdata->bwmgr_handle);
#endif

	/* kobj of acm_kobj_ktype cleans up sysfs entries automatically */
	kobject_put(&pdata->clk_cap_kobj);

	if (pdata->power_kobj) {
		for (i = 0; i < NVHOST_POWER_SYSFS_ATTRIB_MAX; i++) {
			attr = &pdata->power_attrib->power_attr[i];
			sysfs_remove_file(pdata->power_kobj, &attr->attr);
		}

		kobject_put(pdata->power_kobj);
	}

	device_remove_file(&dev->dev, &dev_attr_force_idle);
}
EXPORT_SYMBOL(nvhost_module_deinit);

const struct dev_pm_ops nvhost_module_pm_ops = {
	SET_RUNTIME_PM_OPS(nvhost_module_runtime_suspend,
			   nvhost_module_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(nvhost_module_suspend, nvhost_module_resume)
	.prepare = nvhost_module_prepare_suspend,
	.complete = nvhost_module_complete_resume,
};
EXPORT_SYMBOL(nvhost_module_pm_ops);

void nvhost_register_client_domain(struct generic_pm_domain *domain)
{
}
EXPORT_SYMBOL(nvhost_register_client_domain);

void nvhost_unregister_client_domain(struct generic_pm_domain *domain)
{
}
EXPORT_SYMBOL(nvhost_unregister_client_domain);

int nvhost_module_enable_clk(struct device *dev)
{
	int index = 0;
	struct nvhost_device_data *pdata;

	/* enable parent's clock if required */
	if (dev->parent && dev->parent != &platform_bus)
		nvhost_module_enable_clk(dev->parent);

	pdata = dev_get_drvdata(dev);
	if (!pdata) {
		nvhost_err(dev, "pdata not found");
		return -EINVAL;
	}

	for (index = 0; index < pdata->num_clks; index++) {
		int err;

		if (!pdata->clk[index])
			continue;

		err = clk_prepare_enable(pdata->clk[index]);
		if (err) {
			dev_err(dev, "Cannot turn on clock %s",
				pdata->clocks[index].name);
			return -EINVAL;
		}
	}

	trace_nvhost_module_enable_clk(pdata->pdev->name,
					pdata->num_clks);

	return 0;
}

int nvhost_module_disable_clk(struct device *dev)
{
	int index = 0;
	struct nvhost_device_data *pdata;

	pdata = dev_get_drvdata(dev);
	if (!pdata)
		return -EINVAL;

	trace_nvhost_module_disable_clk(pdata->pdev->name,
					pdata->num_clks);

	for (index = 0; index < pdata->num_clks; index++)
		if (pdata->clk[index])
			clk_disable_unprepare(pdata->clk[index]);

	/* disable parent's clock if required */
	if (dev->parent && dev->parent != &platform_bus)
		nvhost_module_disable_clk(dev->parent);

	return 0;
}

static void nvhost_module_load_regs(struct platform_device *pdev, bool prod)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_gating_register *regs = pdata->engine_cg_regs;

	if (!regs)
		return;

	while (regs->addr) {
		if (prod)
			host1x_writel(pdev, regs->addr, regs->prod);
		else
			host1x_writel(pdev, regs->addr, regs->disable);
		regs++;
	}
}

static void nvhost_module_set_actmon_regs(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct nvhost_actmon_register *regs = pdata->actmon_setting_regs;

	if (!regs)
		return;

	if (nvhost_dev_is_virtual(pdev))
		return;

	while (regs->addr) {
		host1x_writel(pdev, regs->addr, regs->val);
		regs++;
	}
}

static int nvhost_module_runtime_suspend(struct device *dev)
{
	int err;

	dev_dbg(dev, "runtime suspending");

	err = nvhost_module_prepare_poweroff(dev);
	if (err)
		return err;

	err = nvhost_module_disable_clk(dev);
	if (err)
		return err;

	return 0;
}

static int nvhost_module_runtime_resume(struct device *dev)
{
	int err;

	dev_dbg(dev, "runtime resuming");

	err = nvhost_module_enable_clk(dev);
	if (err)
		return err;

	err = nvhost_module_finalize_poweron(dev);
	if (err) {
		nvhost_module_disable_clk(dev);
		return err;
	}

	return 0;
}

static bool nvhost_module_has_open_channels(struct platform_device *pdev)
{
	struct nvhost_master *host = nvhost_get_host(pdev);
	struct nvhost_channel *ch = NULL;
	int idx = 0;

	for (idx = 0; idx < nvhost_channel_nb_channels(host); idx++) {
		ch = host->chlist[idx];
		if (ch->dev == pdev && !list_empty(&ch->cdma.sync_queue))
			return true;
	}

	return false;
}

static int nvhost_module_prepare_suspend(struct device *dev)
{
	struct nvhost_device_data *pdata = dev_get_drvdata(dev);
	struct platform_device *pdev = to_platform_device(dev);
	int i;

	for (i = 0; i < 10; i++) {
		if (!nvhost_module_has_open_channels(pdev))
			break;
		msleep(1);
	}

	if (i == 10) {
		nvhost_err(dev, "device has not quiesced within 10ms");
		nvhost_debug_dump_device(pdev);
		return -EBUSY;
	}

	if (!pdata->can_powergate) {
		/* If we took an extra reference, drop it now to prevent
		 * the device from automatically resuming upon system
		 * resume.
		 */
		pm_runtime_put_sync(dev);
	}

	return 0;
}

#if IS_ENABLED(CONFIG_PM_SLEEP)
static int nvhost_module_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	int err;

	err = vhost_suspend(pdev);
	if (err)
		nvhost_err(dev, "vhost suspend has failed %d", err);

	return pm_runtime_force_suspend(dev);
}

static int nvhost_module_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	int err;

	err = vhost_resume(pdev);
	if (err)
		nvhost_err(dev, "vhost resume has failed %d", err);

	return pm_runtime_force_resume(dev);
}
#endif

static void nvhost_module_complete_resume(struct device *dev)
{
	struct nvhost_device_data *pdata = dev_get_drvdata(dev);

	if (!pdata->can_powergate) {
		/* Retake reference dropped above */
		pm_runtime_get_noresume(dev);
	}
}

static int nvhost_module_prepare_poweroff(struct device *dev)
{
	struct nvhost_device_data *pdata;
	struct nvhost_master *host;

	pdata = dev_get_drvdata(dev);
	if (!pdata) {
		nvhost_err(dev, "pdata not found");
		return -EINVAL;
	}

	if (!pdata->power_on) {
		WARN_ON(1);
		return 0;
	}

	host = nvhost_get_host(pdata->pdev);

	devfreq_suspend_device(pdata->power_manager);
	nvhost_scale_hw_deinit(to_platform_device(dev));

	/* disable module interrupt if support available */
	if (pdata->module_irq)
		nvhost_intr_disable_module_intr(&host->intr,
						pdata->module_irq);

#if defined(CONFIG_TEGRA_BWMGR)
	/* set EMC rate to zero */
	if (pdata->bwmgr_handle) {
		int i;

		for (i = 0; i < NVHOST_MODULE_MAX_CLOCKS; i++) {
			if (nvhost_module_emc_clock(&pdata->clocks[i])) {
				tegra_bwmgr_set_emc(pdata->bwmgr_handle, 0,
					pdata->clocks[i].bwmgr_request_type);
				break;
			}
		}
	}
#endif

	if (pdata->prepare_poweroff)
		pdata->prepare_poweroff(to_platform_device(dev));

	pdata->power_on = false;

	return 0;
}

static int nvhost_module_finalize_poweron(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct nvhost_master *host;
	struct nvhost_device_data *pdata;
	int retry_count, ret = 0, i;

	pdata = dev_get_drvdata(dev);
	if (!pdata) {
		nvhost_err(dev, "pdata not found");
		return -EINVAL;
	}

	host = nvhost_get_host(pdata->pdev);
	/* WAR to bug 1588951: Retry booting 3 times */

	for (retry_count = 0; retry_count < 3; retry_count++) {
		if (!pdata->poweron_toggle_slcg) {
			if (pdata->poweron_reset)
				nvhost_module_reset(pdev, false);

			/* Load clockgating registers */
			nvhost_module_load_regs(pdev, pdata->engine_can_cg);
		} else {
			/* If poweron_toggle_slcg is set, following is already
			 * executed once. Skip to avoid doing it twice. */
			if (retry_count > 0) {
				/* First, reset module */
				nvhost_module_reset(pdev, false);

				/* Disable SLCG, wait and re-enable it */
				nvhost_module_load_regs(pdev, false);
				udelay(1);
				if (pdata->engine_can_cg)
					nvhost_module_load_regs(pdev, true);
			}
		}

		/* initialize device vm */
		ret = nvhost_vm_init_device(pdev);
		if (ret)
			continue;

		if (pdata->finalize_poweron)
			ret = pdata->finalize_poweron(to_platform_device(dev));

		/* Exit loop if we pass module specific initialization */
		if (!ret)
			break;
	}

	/* Failed to start the device */
	if (ret) {
		nvhost_err(dev, "failed to start the device");
		goto out;
	}


	/* Set actmon registers */
	nvhost_module_set_actmon_regs(pdev);

	/* set default EMC rate to zero */
	if (pdata->bwmgr_handle) {
		for (i = 0; i < NVHOST_MODULE_MAX_CLOCKS; i++) {
			if (nvhost_module_emc_clock(&pdata->clocks[i])) {
				mutex_lock(&client_list_lock);
				nvhost_module_update_rate(pdev, i);
				mutex_unlock(&client_list_lock);
				break;
			}
		}
	}

	/* enable module interrupt if support available */
	if (pdata->module_irq)
		nvhost_intr_enable_module_intr(&host->intr,
						pdata->module_irq);

	nvhost_scale_hw_init(to_platform_device(dev));
	devfreq_resume_device(pdata->power_manager);

	pdata->power_on = true;

out:
	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 14, 0)
static int nvhost_module_toggle_slcg(struct notifier_block *nb,
				     unsigned long action, void *data)
{
	struct nvhost_device_data *pdata =
		container_of(nb, struct nvhost_device_data,
			     toggle_slcg_notifier);
	struct platform_device *pdev = pdata->pdev;

	/* First, reset the engine */
	do_module_reset_locked(pdev);

	/* Then, disable slcg, wait a while and re-enable it */
	nvhost_module_load_regs(pdev, false);
	udelay(1);
	if (pdata->engine_can_cg)
		nvhost_module_load_regs(pdev, true);

	return 0;
}
#endif

/* public host1x power management APIs */
bool nvhost_module_powered_ext(struct platform_device *dev)
{
	if (dev->dev.parent && dev->dev.parent != &platform_bus)
		dev = to_platform_device(dev->dev.parent);
	return nvhost_module_powered(dev);
}
EXPORT_SYMBOL(nvhost_module_powered_ext);

int nvhost_module_busy_ext(struct platform_device *dev)
{
	if (dev->dev.parent && dev->dev.parent != &platform_bus)
		dev = to_platform_device(dev->dev.parent);
	return nvhost_module_busy(dev);
}
EXPORT_SYMBOL(nvhost_module_busy_ext);

void nvhost_module_idle_ext(struct platform_device *dev)
{
	if (dev->dev.parent && dev->dev.parent != &platform_bus)
		dev = to_platform_device(dev->dev.parent);
	nvhost_module_idle(dev);
}
EXPORT_SYMBOL(nvhost_module_idle_ext);
