/*
 * Virtualized GPU for Linux
 *
 * Copyright (c) 2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/pm_runtime.h>
#include <linux/pm_qos.h>
#include <linux/platform_device.h>
#include <soc/tegra/chip-id.h>

#include <nvgpu/kmem.h>
#include <nvgpu/bug.h>
#include <nvgpu/enabled.h>
#include <nvgpu/debug.h>
#include <nvgpu/soc.h>
#include <nvgpu/ctxsw_trace.h>
#include <nvgpu/defaults.h>
#include <nvgpu/ltc.h>
#include <nvgpu/channel.h>
#include <nvgpu/clk_arb.h>

#include "vgpu_linux.h"
#include "vgpu/fecs_trace_vgpu.h"
#include "vgpu/clk_vgpu.h"
#include "gk20a/regops_gk20a.h"
#include "gm20b/hal_gm20b.h"

#include "os/linux/module.h"
#include "os/linux/os_linux.h"
#include "os/linux/ioctl.h"
#include "os/linux/scale.h"
#include "os/linux/driver_common.h"
#include "os/linux/platform_gk20a.h"
#include "os/linux/vgpu/platform_vgpu_tegra.h"

struct vgpu_priv_data *vgpu_get_priv_data(struct gk20a *g)
{
	struct gk20a_platform *plat = gk20a_get_platform(dev_from_gk20a(g));

	return (struct vgpu_priv_data *)plat->vgpu_priv;
}

static void vgpu_remove_support(struct gk20a *g)
{
	vgpu_remove_support_common(g);
}

static void vgpu_init_vars(struct gk20a *g, struct gk20a_platform *platform)
{
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	struct vgpu_priv_data *priv = vgpu_get_priv_data(g);

	nvgpu_mutex_init(&g->power_lock);
	nvgpu_mutex_init(&g->ctxsw_disable_lock);
	nvgpu_mutex_init(&g->clk_arb_enable_lock);
	nvgpu_mutex_init(&g->cg_pg_lock);

	nvgpu_mutex_init(&priv->vgpu_clk_get_freq_lock);

	nvgpu_mutex_init(&l->ctrl.privs_lock);
	nvgpu_init_list_node(&l->ctrl.privs);

	l->regs_saved = l->regs;
	l->bar1_saved = l->bar1;

	nvgpu_atomic_set(&g->clk_arb_global_nr, 0);

	g->aggressive_sync_destroy = platform->aggressive_sync_destroy;
	g->aggressive_sync_destroy_thresh = platform->aggressive_sync_destroy_thresh;
	__nvgpu_set_enabled(g, NVGPU_HAS_SYNCPOINTS, platform->has_syncpoints);
	g->ptimer_src_freq = platform->ptimer_src_freq;
	__nvgpu_set_enabled(g, NVGPU_CAN_RAILGATE, platform->can_railgate_init);
	g->railgate_delay = platform->railgate_delay_init;

	__nvgpu_set_enabled(g, NVGPU_MM_UNIFY_ADDRESS_SPACES,
			    platform->unify_address_spaces);
}

static int vgpu_init_support(struct platform_device *pdev)
{
	struct resource *r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct gk20a *g = get_gk20a(&pdev->dev);
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	void __iomem *regs;
	int err = 0;

	if (!r) {
		nvgpu_err(g, "failed to get gk20a bar1");
		err = -ENXIO;
		goto fail;
	}

	if (r->name && !strcmp(r->name, "/vgpu")) {
		regs = devm_ioremap_resource(&pdev->dev, r);
		if (IS_ERR(regs)) {
			nvgpu_err(g, "failed to remap gk20a bar1");
			err = PTR_ERR(regs);
			goto fail;
		}
		l->bar1 = regs;
		l->bar1_mem = r;
	}

	nvgpu_mutex_init(&g->dbg_sessions_lock);
	nvgpu_mutex_init(&g->client_lock);

	nvgpu_init_list_node(&g->profiler_objects);

	g->dbg_regops_tmp_buf = nvgpu_kzalloc(g, SZ_4K);
	if (!g->dbg_regops_tmp_buf) {
		nvgpu_err(g, "couldn't allocate regops tmp buf");
		return -ENOMEM;
	}
	g->dbg_regops_tmp_buf_ops =
		SZ_4K / sizeof(g->dbg_regops_tmp_buf[0]);

	g->remove_support = vgpu_remove_support;
	return 0;

 fail:
	vgpu_remove_support(g);
	return err;
}

int vgpu_pm_prepare_poweroff(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	int ret = 0;

	nvgpu_log_fn(g, " ");

	nvgpu_mutex_acquire(&g->power_lock);

	if (!g->power_on)
		goto done;

	if (g->ops.fifo.channel_suspend)
		ret = g->ops.fifo.channel_suspend(g);
	if (ret)
		goto done;

	g->power_on = false;
 done:
	nvgpu_mutex_release(&g->power_lock);

	return ret;
}

int vgpu_pm_finalize_poweron(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	int err = 0;

	nvgpu_log_fn(g, " ");

	nvgpu_mutex_acquire(&g->power_lock);

	if (g->power_on)
		goto done;

	g->power_on = true;

	vgpu_detect_chip(g);
	err = vgpu_init_hal(g);
	if (err)
		goto done;

	if (g->ops.ltc.init_fs_state)
		g->ops.ltc.init_fs_state(g);

	err = nvgpu_init_ltc_support(g);
	if (err) {
		nvgpu_err(g, "failed to init ltc");
		goto done;
	}

	err = vgpu_init_mm_support(g);
	if (err) {
		nvgpu_err(g, "failed to init gk20a mm");
		goto done;
	}

	err = vgpu_init_fifo_support(g);
	if (err) {
		nvgpu_err(g, "failed to init gk20a fifo");
		goto done;
	}

	err = vgpu_init_gr_support(g);
	if (err) {
		nvgpu_err(g, "failed to init gk20a gr");
		goto done;
	}

	err = nvgpu_clk_arb_init_arbiter(g);
	if (err) {
		nvgpu_err(g, "failed to init clk arb");
		goto done;
	}

	err = g->ops.chip_init_gpu_characteristics(g);
	if (err) {
		nvgpu_err(g, "failed to init gk20a gpu characteristics");
		goto done;
	}

	err = nvgpu_finalize_poweron_linux(l);
	if (err)
		goto done;

#ifdef CONFIG_GK20A_CTXSW_TRACE
	gk20a_ctxsw_trace_init(g);
#endif
	gk20a_sched_ctrl_init(g);
	gk20a_channel_resume(g);

	g->sw_ready = true;

done:
	if (err)
		g->power_on = false;

	nvgpu_mutex_release(&g->power_lock);
	return err;
}

static int vgpu_qos_notify(struct notifier_block *nb,
			  unsigned long n, void *data)
{
	struct gk20a_scale_profile *profile =
			container_of(nb, struct gk20a_scale_profile,
			qos_notify_block);
	struct gk20a *g = get_gk20a(profile->dev);
	u32 max_freq;
	int err;

	nvgpu_log_fn(g, " ");

	max_freq = (u32)pm_qos_read_max_bound(PM_QOS_GPU_FREQ_BOUNDS);
	err = vgpu_plat_clk_cap_rate(profile->dev, max_freq);
	if (err)
		nvgpu_err(g, "%s failed, err=%d", __func__, err);

	return NOTIFY_OK; /* need notify call further */
}

static int vgpu_pm_qos_init(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	struct gk20a_scale_profile *profile = g->scale_profile;

	if (IS_ENABLED(CONFIG_GK20A_DEVFREQ)) {
		if (!profile)
			return -EINVAL;
	} else {
		profile = nvgpu_kzalloc(g, sizeof(*profile));
		if (!profile)
			return -ENOMEM;
		g->scale_profile = profile;
	}

	profile->dev = dev;
	profile->qos_notify_block.notifier_call = vgpu_qos_notify;
	pm_qos_add_max_notifier(PM_QOS_GPU_FREQ_BOUNDS,
				&profile->qos_notify_block);
	return 0;
}

static void vgpu_pm_qos_remove(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);

	pm_qos_remove_max_notifier(PM_QOS_GPU_FREQ_BOUNDS,
				&g->scale_profile->qos_notify_block);
	nvgpu_kfree(g, g->scale_profile);
	g->scale_profile = NULL;
}

static int vgpu_pm_init(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);
	unsigned long *freqs;
	int num_freqs;
	int err = 0;

	nvgpu_log_fn(g, " ");

	if (nvgpu_platform_is_simulation(g))
		return 0;

	__pm_runtime_disable(dev, false);

	if (IS_ENABLED(CONFIG_GK20A_DEVFREQ))
		gk20a_scale_init(dev);

	if (l->devfreq) {
		/* set min/max frequency based on frequency table */
		err = platform->get_clk_freqs(dev, &freqs, &num_freqs);
		if (err)
			return err;

		if (num_freqs < 1)
			return -EINVAL;

		l->devfreq->min_freq = freqs[0];
		l->devfreq->max_freq = freqs[num_freqs - 1];
	}

	err = vgpu_pm_qos_init(dev);
	if (err)
		return err;

	return err;
}

int vgpu_probe(struct platform_device *pdev)
{
	struct nvgpu_os_linux *l;
	struct gk20a *gk20a;
	int err;
	struct device *dev = &pdev->dev;
	struct gk20a_platform *platform = gk20a_get_platform(dev);
	struct vgpu_priv_data *priv;

	if (!platform) {
		dev_err(dev, "no platform data\n");
		return -ENODATA;
	}

	l = kzalloc(sizeof(*l), GFP_KERNEL);
	if (!l) {
		dev_err(dev, "couldn't allocate gk20a support");
		return -ENOMEM;
	}
	gk20a = &l->g;

	nvgpu_log_fn(gk20a, " ");

	nvgpu_init_gk20a(gk20a);

	nvgpu_kmem_init(gk20a);

	err = nvgpu_init_enabled_flags(gk20a);
	if (err) {
		kfree(gk20a);
		return err;
	}

	l->dev = dev;
	if (tegra_platform_is_vdk())
		__nvgpu_set_enabled(gk20a, NVGPU_IS_FMODEL, true);

	gk20a->is_virtual = true;

	priv = nvgpu_kzalloc(gk20a, sizeof(*priv));
	if (!priv) {
		kfree(gk20a);
		return -ENOMEM;
	}

	platform->g = gk20a;
	platform->vgpu_priv = priv;

	err = gk20a_user_init(dev, INTERFACE_NAME, &nvgpu_class);
	if (err)
		return err;

	vgpu_init_support(pdev);

	vgpu_init_vars(gk20a, platform);

	init_rwsem(&l->busy_lock);

	nvgpu_spinlock_init(&gk20a->mc_enable_lock);

	gk20a->ch_wdt_timeout_ms = platform->ch_wdt_timeout_ms;

	/* Initialize the platform interface. */
	err = platform->probe(dev);
	if (err) {
		if (err == -EPROBE_DEFER)
			nvgpu_info(gk20a, "platform probe failed");
		else
			nvgpu_err(gk20a, "platform probe failed");
		return err;
	}

	if (platform->late_probe) {
		err = platform->late_probe(dev);
		if (err) {
			nvgpu_err(gk20a, "late probe failed");
			return err;
		}
	}

	err = vgpu_comm_init(gk20a);
	if (err) {
		nvgpu_err(gk20a, "failed to init comm interface");
		return -ENOSYS;
	}

	priv->virt_handle = vgpu_connect();
	if (!priv->virt_handle) {
		nvgpu_err(gk20a, "failed to connect to server node");
		vgpu_comm_deinit();
		return -ENOSYS;
	}

	err = vgpu_get_constants(gk20a);
	if (err) {
		vgpu_comm_deinit();
		return err;
	}

	err = vgpu_pm_init(dev);
	if (err) {
		nvgpu_err(gk20a, "pm init failed");
		return err;
	}

	err = nvgpu_thread_create(&priv->intr_handler, gk20a,
			vgpu_intr_thread, "gk20a");
	if (err)
		return err;

	gk20a_debug_init(gk20a, "gpu.0");

	/* Set DMA parameters to allow larger sgt lists */
	dev->dma_parms = &l->dma_parms;
	dma_set_max_seg_size(dev, UINT_MAX);

	gk20a->gr_idle_timeout_default = NVGPU_DEFAULT_GR_IDLE_TIMEOUT;
	gk20a->timeouts_disabled_by_user = false;
	nvgpu_atomic_set(&gk20a->timeouts_disabled_refcount, 0);

	vgpu_create_sysfs(dev);
	gk20a_init_gr(gk20a);

	nvgpu_log_info(gk20a, "total ram pages : %lu", totalram_pages);
	gk20a->gr.max_comptag_mem = totalram_size_in_mb;

	nvgpu_ref_init(&gk20a->refcount);

	return 0;
}

int vgpu_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gk20a *g = get_gk20a(dev);

	nvgpu_log_fn(g, " ");

	vgpu_pm_qos_remove(dev);
	if (g->remove_support)
		g->remove_support(g);

	vgpu_comm_deinit();
	gk20a_sched_ctrl_cleanup(g);
	gk20a_user_deinit(dev, &nvgpu_class);
	vgpu_remove_sysfs(dev);
	gk20a_get_platform(dev)->g = NULL;
	gk20a_put(g);

	return 0;
}

bool vgpu_is_reduced_bar1(struct gk20a *g)
{
	struct fifo_gk20a *f = &g->fifo;
	struct nvgpu_os_linux *l = nvgpu_os_linux_from_gk20a(g);

	return resource_size(l->bar1_mem) == (resource_size_t)f->userd.size;
}

int vgpu_tegra_suspend(struct device *dev)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct gk20a *g = get_gk20a(dev);
	int err = 0;

	msg.cmd = TEGRA_VGPU_CMD_SUSPEND;
	msg.handle = vgpu_get_handle(g);
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (err)
		nvgpu_err(g, "vGPU suspend failed\n");

	return err;
}

int vgpu_tegra_resume(struct device *dev)
{
	struct tegra_vgpu_cmd_msg msg = {};
	struct gk20a *g = get_gk20a(dev);
	int err = 0;

	msg.cmd = TEGRA_VGPU_CMD_RESUME;
	msg.handle = vgpu_get_handle(g);
	err = vgpu_comm_sendrecv(&msg, sizeof(msg), sizeof(msg));
	err = err ? err : msg.ret;
	if (err)
		nvgpu_err(g, "vGPU resume failed\n");

	return err;
}
