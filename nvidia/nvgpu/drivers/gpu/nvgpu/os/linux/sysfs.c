/*
 * Copyright (c) 2011-2019, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/device.h>
#include <linux/pm_runtime.h>
#include <linux/fb.h>

#include <nvgpu/kmem.h>
#include <nvgpu/nvhost.h>
#include <nvgpu/ptimer.h>
#include <nvgpu/power_features/cg.h>
#include <nvgpu/power_features/pg.h>

#include "os_linux.h"
#include "sysfs.h"
#include "platform_gk20a.h"
#include "gk20a/gr_gk20a.h"
#include "gv11b/gr_gv11b.h"

#define PTIMER_FP_FACTOR			1000000

#define ROOTRW (S_IRWXU|S_IRGRP|S_IROTH)

#define TPC_MASK_FOR_ALL_ACTIVE_TPCs           (u32) 0x0

static ssize_t elcg_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	unsigned long val = 0;
	int err;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	err = gk20a_busy(g);
	if (err)
		return err;

	if (val) {
		nvgpu_cg_elcg_set_elcg_enabled(g, true);
	} else {
		nvgpu_cg_elcg_set_elcg_enabled(g, false);
	}

	gk20a_idle(g);

	nvgpu_info(g, "ELCG is %s.", val ? "enabled" :
			"disabled");

	return count;
}

static ssize_t elcg_enable_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", g->elcg_enabled ? 1 : 0);
}

static DEVICE_ATTR(elcg_enable, ROOTRW, elcg_enable_read, elcg_enable_store);

static ssize_t blcg_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	unsigned long val = 0;
	int err;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	err = gk20a_busy(g);
	if (err)
		return err;

	if (val) {
		nvgpu_cg_blcg_set_blcg_enabled(g, true);
	} else {
		nvgpu_cg_blcg_set_blcg_enabled(g, false);
	}

	gk20a_idle(g);

	nvgpu_info(g, "BLCG is %s.", val ? "enabled" :
			"disabled");

	return count;
}

static ssize_t blcg_enable_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", g->blcg_enabled ? 1 : 0);
}


static DEVICE_ATTR(blcg_enable, ROOTRW, blcg_enable_read, blcg_enable_store);

static ssize_t slcg_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	unsigned long val = 0;
	int err;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	err = gk20a_busy(g);
	if (err) {
		return err;
	}

	if (val) {
		nvgpu_cg_slcg_set_slcg_enabled(g, true);
	} else {
		nvgpu_cg_slcg_set_slcg_enabled(g, false);
	}

	/*
	 * TODO: slcg_therm_load_gating is not enabled anywhere during
	 * init. Therefore, it would be incongruous to add it here. Once
	 * it is added to init, we should add it here too.
	 */
	gk20a_idle(g);

	nvgpu_info(g, "SLCG is %s.", val ? "enabled" :
			"disabled");

	return count;
}

static ssize_t slcg_enable_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", g->slcg_enabled ? 1 : 0);
}

static DEVICE_ATTR(slcg_enable, ROOTRW, slcg_enable_read, slcg_enable_store);

static ssize_t ptimer_scale_factor_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct gk20a *g = get_gk20a(dev);
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	u32 src_freq_hz = platform->ptimer_src_freq;
	u32 scaling_factor_fp;
	ssize_t res;

	if (!src_freq_hz) {
		nvgpu_err(g, "reference clk_m rate is not set correctly");
		return -EINVAL;
	}

	scaling_factor_fp = (u32)(PTIMER_REF_FREQ_HZ) /
				((u32)(src_freq_hz) /
				(u32)(PTIMER_FP_FACTOR));
	res = snprintf(buf,
				PAGE_SIZE,
				"%u.%u\n",
				scaling_factor_fp / PTIMER_FP_FACTOR,
				scaling_factor_fp % PTIMER_FP_FACTOR);

	return res;

}

static DEVICE_ATTR(ptimer_scale_factor,
			S_IRUGO,
			ptimer_scale_factor_show,
			NULL);

static ssize_t ptimer_ref_freq_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct gk20a *g = get_gk20a(dev);
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	u32 src_freq_hz = platform->ptimer_src_freq;
	ssize_t res;

	if (!src_freq_hz) {
		nvgpu_err(g, "reference clk_m rate is not set correctly");
		return -EINVAL;
	}

	res = snprintf(buf, PAGE_SIZE, "%u\n", PTIMER_REF_FREQ_HZ);

	return res;

}

static DEVICE_ATTR(ptimer_ref_freq,
			S_IRUGO,
			ptimer_ref_freq_show,
			NULL);

static ssize_t ptimer_src_freq_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct gk20a *g = get_gk20a(dev);
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	u32 src_freq_hz = platform->ptimer_src_freq;
	ssize_t res;

	if (!src_freq_hz) {
		nvgpu_err(g, "reference clk_m rate is not set correctly");
		return -EINVAL;
	}

	res = snprintf(buf, PAGE_SIZE, "%u\n", src_freq_hz);

	return res;

}

static DEVICE_ATTR(ptimer_src_freq,
			S_IRUGO,
			ptimer_src_freq_show,
			NULL);


static ssize_t gpu_powered_on_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", g->power_on);
}

static DEVICE_ATTR(gpu_powered_on, S_IRUGO, gpu_powered_on_show, NULL);

#if defined(CONFIG_PM)
static ssize_t railgate_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long railgate_enable = 0;
	/* dev is guaranteed to be valid here. Ok to de-reference */
	struct gk20a *g = get_gk20a(dev);
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	bool enabled = nvgpu_is_enabled(g, NVGPU_CAN_RAILGATE);
	int err;

	if (kstrtoul(buf, 10, &railgate_enable) < 0)
		return -EINVAL;

	/* convert to boolean */
	railgate_enable = !!railgate_enable;

	/* writing same value should be treated as nop and successful */
	if (railgate_enable == enabled)
		goto out;

	if (!platform->can_railgate_init) {
		nvgpu_err(g, "Railgating is not supported");
		return -EINVAL;
	}

	if (railgate_enable) {
		__nvgpu_set_enabled(g, NVGPU_CAN_RAILGATE, true);
		pm_runtime_set_autosuspend_delay(dev, g->railgate_delay);
	} else {
		__nvgpu_set_enabled(g, NVGPU_CAN_RAILGATE, false);
		pm_runtime_set_autosuspend_delay(dev, -1);
	}
	/* wake-up system to make rail-gating setting effective */
	err = gk20a_busy(g);
	if (err)
		return err;
	gk20a_idle(g);

out:
	nvgpu_info(g, "railgate is %s.",
			nvgpu_is_enabled(g, NVGPU_CAN_RAILGATE) ?
			"enabled" : "disabled");

	return count;
}

static ssize_t railgate_enable_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			nvgpu_is_enabled(g, NVGPU_CAN_RAILGATE) ? 1 : 0);
}

static DEVICE_ATTR(railgate_enable, ROOTRW, railgate_enable_read,
			railgate_enable_store);
#endif

static ssize_t railgate_delay_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	int railgate_delay = 0, ret = 0;
	struct gk20a *g = get_gk20a(dev);
	int err;

	if (!nvgpu_is_enabled(g, NVGPU_CAN_RAILGATE)) {
		nvgpu_info(g, "does not support power-gating");
		return count;
	}

	ret = sscanf(buf, "%d", &railgate_delay);
	if (ret == 1 && railgate_delay >= 0) {
		g->railgate_delay = railgate_delay;
		pm_runtime_set_autosuspend_delay(dev, g->railgate_delay);
	} else
		nvgpu_err(g, "Invalid powergate delay");

	/* wake-up system to make rail-gating delay effective immediately */
	err = gk20a_busy(g);
	if (err)
		return err;
	gk20a_idle(g);

	return count;
}
static ssize_t railgate_delay_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", g->railgate_delay);
}
static DEVICE_ATTR(railgate_delay, ROOTRW, railgate_delay_show,
		   railgate_delay_store);

static ssize_t is_railgated_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct gk20a_platform *platform = dev_get_drvdata(dev);
	bool is_railgated = 0;

	if (platform->is_railgated)
		is_railgated = platform->is_railgated(dev);

	return snprintf(buf, PAGE_SIZE, "%s\n", is_railgated ? "yes" : "no");
}
static DEVICE_ATTR(is_railgated, S_IRUGO, is_railgated_show, NULL);

static ssize_t counters_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);
	u32 busy_cycles, total_cycles;
	ssize_t res;

	nvgpu_pmu_get_load_counters(g, &busy_cycles, &total_cycles);

	res = snprintf(buf, PAGE_SIZE, "%u %u\n", busy_cycles, total_cycles);

	return res;
}
static DEVICE_ATTR(counters, S_IRUGO, counters_show, NULL);

static ssize_t counters_show_reset(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	ssize_t res = counters_show(dev, attr, buf);
	struct gk20a *g = get_gk20a(dev);

	nvgpu_pmu_reset_load_counters(g);

	return res;
}
static DEVICE_ATTR(counters_reset, S_IRUGO, counters_show_reset, NULL);

static ssize_t gk20a_load_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct gk20a *g = get_gk20a(dev);
	u32 busy_time;
	ssize_t res;
	int err;

	if (!g->power_on) {
		busy_time = 0;
	} else {
		err = gk20a_busy(g);
		if (err)
			return err;

		nvgpu_pmu_load_update(g);
		nvgpu_pmu_load_norm(g, &busy_time);
		gk20a_idle(g);
	}

	res = snprintf(buf, PAGE_SIZE, "%u\n", busy_time);

	return res;
}
static DEVICE_ATTR(load, S_IRUGO, gk20a_load_show, NULL);

static ssize_t elpg_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	unsigned long val = 0;
	int err;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (!g->power_on) {
		return -EINVAL;
	} else {
		err = gk20a_busy(g);
		if (err)
			return -EAGAIN;
		/*
		 * Since elpg is refcounted, we should not unnecessarily call
		 * enable/disable if it is already so.
		 */
		if (val != 0) {
			nvgpu_pg_elpg_set_elpg_enabled(g, true);
		} else {
			nvgpu_pg_elpg_set_elpg_enabled(g, false);
		}
		gk20a_idle(g);
	}
	nvgpu_info(g, "ELPG is %s.", val ? "enabled" :
			"disabled");

	return count;
}

static ssize_t elpg_enable_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n",
			nvgpu_pg_elpg_is_enabled(g) ? 1 : 0);
}

static DEVICE_ATTR(elpg_enable, ROOTRW, elpg_enable_read, elpg_enable_store);

static ssize_t ldiv_slowdown_factor_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	unsigned long val = 0;
	int err;

	if (kstrtoul(buf, 10, &val) < 0) {
		nvgpu_err(g, "parse error for input SLOWDOWN factor\n");
		return -EINVAL;
	}

	if (val >= SLOWDOWN_FACTOR_FPDIV_BYMAX) {
		nvgpu_err(g, "Invalid SLOWDOWN factor\n");
		return -EINVAL;
	}

	if (val == g->ldiv_slowdown_factor)
		return count;

	if (!g->power_on) {
		g->ldiv_slowdown_factor = val;
	} else {
		err = gk20a_busy(g);
		if (err)
			return -EAGAIN;

		g->ldiv_slowdown_factor = val;

		if (g->ops.pmu.pmu_pg_init_param)
			g->ops.pmu.pmu_pg_init_param(g,
				PMU_PG_ELPG_ENGINE_ID_GRAPHICS);

		gk20a_idle(g);
	}

	nvgpu_info(g, "ldiv_slowdown_factor is %x\n", g->ldiv_slowdown_factor);

	return count;
}

static ssize_t ldiv_slowdown_factor_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", g->ldiv_slowdown_factor);
}

static DEVICE_ATTR(ldiv_slowdown_factor, ROOTRW,
			ldiv_slowdown_factor_read, ldiv_slowdown_factor_store);

static ssize_t mscg_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	struct nvgpu_pmu *pmu = &g->pmu;
	unsigned long val = 0;
	int err;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (!g->power_on) {
		g->mscg_enabled = val ? true : false;
	} else {
		err = gk20a_busy(g);
		if (err)
			return -EAGAIN;
		/*
		 * Since elpg is refcounted, we should not unnecessarily call
		 * enable/disable if it is already so.
		 */
		if (val && !g->mscg_enabled) {
			g->mscg_enabled = true;
			if (g->ops.pmu.pmu_is_lpwr_feature_supported(g,
					PMU_PG_LPWR_FEATURE_MSCG)) {
				if (!ACCESS_ONCE(pmu->mscg_stat)) {
					WRITE_ONCE(pmu->mscg_stat,
						PMU_MSCG_ENABLED);
					/* make status visible */
					smp_mb();
				}
			}

		} else if (!val && g->mscg_enabled) {
			if (g->ops.pmu.pmu_is_lpwr_feature_supported(g,
					PMU_PG_LPWR_FEATURE_MSCG)) {
				nvgpu_pmu_pg_global_enable(g, false);
				WRITE_ONCE(pmu->mscg_stat, PMU_MSCG_DISABLED);
				/* make status visible */
				smp_mb();
				g->mscg_enabled = false;
				if (nvgpu_pg_elpg_is_enabled(g)) {
					nvgpu_pg_elpg_enable(g);
				}
			}
			g->mscg_enabled = false;
		}
		gk20a_idle(g);
	}
	nvgpu_info(g, "MSCG is %s.", g->mscg_enabled ? "enabled" :
			"disabled");

	return count;
}

static ssize_t mscg_enable_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", g->mscg_enabled ? 1 : 0);
}

static DEVICE_ATTR(mscg_enable, ROOTRW, mscg_enable_read, mscg_enable_store);

static ssize_t aelpg_param_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	int status = 0;
	union pmu_ap_cmd ap_cmd;
	int *paramlist = (int *)g->pmu.aelpg_param;
	u32 defaultparam[5] = {
			APCTRL_SAMPLING_PERIOD_PG_DEFAULT_US,
			APCTRL_MINIMUM_IDLE_FILTER_DEFAULT_US,
			APCTRL_MINIMUM_TARGET_SAVING_DEFAULT_US,
			APCTRL_POWER_BREAKEVEN_DEFAULT_US,
			APCTRL_CYCLES_PER_SAMPLE_MAX_DEFAULT
	};

	/* Get each parameter value from input string*/
	sscanf(buf, "%d %d %d %d %d", &paramlist[0], &paramlist[1],
				&paramlist[2], &paramlist[3], &paramlist[4]);

	/* If parameter value is 0 then reset to SW default values*/
	if ((paramlist[0] | paramlist[1] | paramlist[2]
		| paramlist[3] | paramlist[4]) == 0x00) {
		memcpy(paramlist, defaultparam, sizeof(defaultparam));
	}

	/* If aelpg is enabled & pmu is ready then post values to
	 * PMU else store then post later
	 */
	if (g->aelpg_enabled && g->pmu.pmu_ready) {
		/* Disable AELPG */
		ap_cmd.disable_ctrl.cmd_id = PMU_AP_CMD_ID_DISABLE_CTRL;
		ap_cmd.disable_ctrl.ctrl_id = PMU_AP_CTRL_ID_GRAPHICS;
		status = nvgpu_pmu_ap_send_command(g, &ap_cmd, false);

		/* Enable AELPG */
		nvgpu_aelpg_init(g);
		nvgpu_aelpg_init_and_enable(g, PMU_AP_CTRL_ID_GRAPHICS);
	}

	return count;
}

static ssize_t aelpg_param_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return snprintf(buf, PAGE_SIZE,
		"%d %d %d %d %d\n", g->pmu.aelpg_param[0],
		g->pmu.aelpg_param[1], g->pmu.aelpg_param[2],
		g->pmu.aelpg_param[3], g->pmu.aelpg_param[4]);
}

static DEVICE_ATTR(aelpg_param, ROOTRW,
		aelpg_param_read, aelpg_param_store);

static ssize_t aelpg_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	unsigned long val = 0;
	int status = 0;
	union pmu_ap_cmd ap_cmd;
	int err;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	err = gk20a_busy(g);
	if (err)
		return err;

	if (g->pmu.pmu_ready) {
		if (val && !g->aelpg_enabled) {
			g->aelpg_enabled = true;
			/* Enable AELPG */
			ap_cmd.enable_ctrl.cmd_id = PMU_AP_CMD_ID_ENABLE_CTRL;
			ap_cmd.enable_ctrl.ctrl_id = PMU_AP_CTRL_ID_GRAPHICS;
			status = nvgpu_pmu_ap_send_command(g, &ap_cmd, false);
		} else if (!val && g->aelpg_enabled) {
			g->aelpg_enabled = false;
			/* Disable AELPG */
			ap_cmd.disable_ctrl.cmd_id = PMU_AP_CMD_ID_DISABLE_CTRL;
			ap_cmd.disable_ctrl.ctrl_id = PMU_AP_CTRL_ID_GRAPHICS;
			status = nvgpu_pmu_ap_send_command(g, &ap_cmd, false);
		}
	} else {
		nvgpu_info(g, "PMU is not ready, AELPG request failed");
	}
	gk20a_idle(g);

	nvgpu_info(g, "AELPG is %s.", g->aelpg_enabled ? "enabled" :
			"disabled");

	return count;
}

static ssize_t aelpg_enable_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", g->aelpg_enabled ? 1 : 0);
}

static DEVICE_ATTR(aelpg_enable, ROOTRW,
		aelpg_enable_read, aelpg_enable_store);


static ssize_t allow_all_enable_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", g->allow_all ? 1 : 0);
}

static ssize_t allow_all_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	unsigned long val = 0;
	int err;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	err = gk20a_busy(g);
	g->allow_all = (val ? true : false);
	gk20a_idle(g);

	return count;
}

static DEVICE_ATTR(allow_all, ROOTRW,
		allow_all_enable_read, allow_all_enable_store);

static ssize_t emc3d_ratio_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	unsigned long val = 0;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	g->emc3d_ratio = val;

	return count;
}

static ssize_t emc3d_ratio_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", g->emc3d_ratio);
}

static DEVICE_ATTR(emc3d_ratio, ROOTRW, emc3d_ratio_read, emc3d_ratio_store);

static ssize_t fmax_at_vmin_safe_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);
	unsigned long gpu_fmax_at_vmin_hz = 0;

	if (g->ops.clk.get_fmax_at_vmin_safe)
		gpu_fmax_at_vmin_hz = g->ops.clk.get_fmax_at_vmin_safe(g);

	return snprintf(buf, PAGE_SIZE, "%d\n", (int)(gpu_fmax_at_vmin_hz));
}

static DEVICE_ATTR(fmax_at_vmin_safe, S_IRUGO, fmax_at_vmin_safe_read, NULL);

#ifdef CONFIG_PM
static ssize_t force_idle_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	unsigned long val = 0;
	int err = 0;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (val) {
		if (g->forced_idle)
			return count; /* do nothing */
		else {
			err = __gk20a_do_idle(g, false);
			if (!err) {
				g->forced_idle = 1;
				nvgpu_info(g, "gpu is idle : %d",
					g->forced_idle);
			}
		}
	} else {
		if (!g->forced_idle)
			return count; /* do nothing */
		else {
			err = __gk20a_do_unidle(g);
			if (!err) {
				g->forced_idle = 0;
				nvgpu_info(g, "gpu is idle : %d",
					g->forced_idle);
			}
		}
	}

	return count;
}

static ssize_t force_idle_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", g->forced_idle ? 1 : 0);
}

static DEVICE_ATTR(force_idle, ROOTRW, force_idle_read, force_idle_store);
#endif

static bool is_tpc_mask_valid(struct gk20a *g, u32 tpc_mask)
{
	u32 i;
	bool valid = false;

	for (i = 0; i < MAX_TPC_PG_CONFIGS; i++) {
		if (tpc_mask == g->valid_tpc_mask[i]) {
			valid = true;
			break;
		}
	}
	return valid;
}

static ssize_t tpc_pg_mask_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", g->tpc_pg_mask);
}

static ssize_t tpc_pg_mask_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	struct gr_gk20a *gr = &g->gr;
	unsigned long val = 0;

	nvgpu_mutex_acquire(&g->tpc_pg_lock);

	if (kstrtoul(buf, 10, &val) < 0) {
		nvgpu_err(g, "invalid value");
		nvgpu_mutex_release(&g->tpc_pg_lock);
		return -EINVAL;
	}

	if (val == g->tpc_pg_mask) {
		nvgpu_info(g, "no value change, same mask already set");
		goto exit;
	}

	if (gr->ctx_vars.golden_image_size) {
		nvgpu_err(g, "golden image size already initialized");
		nvgpu_mutex_release(&g->tpc_pg_lock);
		return -ENODEV;
	}

	/* checking that the value from userspace is within
	 * the  possible valid TPC configurations.
	 */
	if (is_tpc_mask_valid(g, (u32)val)) {
		g->tpc_pg_mask = val;
	} else {
		nvgpu_err(g, "TPC-PG mask is invalid");
		nvgpu_mutex_release(&g->tpc_pg_lock);
		return -EINVAL;
	}
exit:
	nvgpu_mutex_release(&g->tpc_pg_lock);

	return count;
}

static DEVICE_ATTR(tpc_pg_mask, ROOTRW, tpc_pg_mask_read, tpc_pg_mask_store);

static ssize_t tpc_fs_mask_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	unsigned long val = 0;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (!g->gr.gpc_tpc_mask)
		return -ENODEV;

	if (val && val != g->gr.gpc_tpc_mask[0] && g->ops.gr.set_gpc_tpc_mask) {
		g->gr.gpc_tpc_mask[0] = val;
		g->tpc_fs_mask_user = val;

		g->ops.gr.set_gpc_tpc_mask(g, 0);

		nvgpu_vfree(g, g->gr.ctx_vars.local_golden_image);
		g->gr.ctx_vars.local_golden_image = NULL;
		g->gr.ctx_vars.golden_image_initialized = false;
		g->gr.ctx_vars.golden_image_size = 0;
		/* Cause next poweron to reinit just gr */
		g->gr.sw_ready = false;
	}

	return count;
}

static ssize_t tpc_fs_mask_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);
	struct gr_gk20a *gr = &g->gr;
	u32 gpc_index;
	u32 tpc_fs_mask = 0;
	int err = 0;

	err = gk20a_busy(g);
	if (err)
		return err;

	for (gpc_index = 0; gpc_index < gr->gpc_count; gpc_index++) {
		if (g->ops.gr.get_gpc_tpc_mask)
			tpc_fs_mask |=
				g->ops.gr.get_gpc_tpc_mask(g, gpc_index) <<
				(gr->max_tpc_per_gpc_count * gpc_index);
	}

	gk20a_idle(g);

	return snprintf(buf, PAGE_SIZE, "0x%x\n", tpc_fs_mask);
}

static DEVICE_ATTR(tpc_fs_mask, ROOTRW, tpc_fs_mask_read, tpc_fs_mask_store);

static ssize_t min_timeslice_us_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", g->min_timeslice_us);
}

static ssize_t min_timeslice_us_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (val > g->max_timeslice_us)
		return -EINVAL;

	g->min_timeslice_us = val;

	return count;
}

static DEVICE_ATTR(min_timeslice_us, ROOTRW, min_timeslice_us_read,
		   min_timeslice_us_store);

static ssize_t max_timeslice_us_read(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", g->max_timeslice_us);
}

static ssize_t max_timeslice_us_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (val < g->min_timeslice_us)
		return -EINVAL;

	g->max_timeslice_us = val;

	return count;
}

static DEVICE_ATTR(max_timeslice_us, ROOTRW, max_timeslice_us_read,
		   max_timeslice_us_store);

static ssize_t czf_bypass_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (val >= 4)
		return -EINVAL;

	g->gr.czf_bypass = val;

	return count;
}

static ssize_t czf_bypass_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return sprintf(buf, "%d\n", g->gr.czf_bypass);
}

static DEVICE_ATTR(czf_bypass, ROOTRW, czf_bypass_read, czf_bypass_store);

static ssize_t pd_max_batches_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (val > 64)
		return -EINVAL;

	g->gr.pd_max_batches = val;

	return count;
}

static ssize_t pd_max_batches_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return sprintf(buf, "%d\n", g->gr.pd_max_batches);
}

static DEVICE_ATTR(pd_max_batches, ROOTRW, pd_max_batches_read, pd_max_batches_store);

static ssize_t gfxp_wfi_timeout_count_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	struct gr_gk20a *gr = &g->gr;
	unsigned long val = 0;
	int err = -1;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (g->ops.gr.get_max_gfxp_wfi_timeout_count) {
		if (val >= g->ops.gr.get_max_gfxp_wfi_timeout_count(g))
			return -EINVAL;
	}

	gr->gfxp_wfi_timeout_count = val;

	if (g->ops.gr.init_preemption_state && g->power_on) {
		err = gk20a_busy(g);
		if (err)
			return err;

		err = gr_gk20a_elpg_protected_call(g,
			g->ops.gr.init_preemption_state(g));

		gk20a_idle(g);

		if (err)
			return err;
	}
	return count;
}

static ssize_t gfxp_wfi_timeout_unit_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	struct gr_gk20a *gr = &g->gr;
	int err = -1;

	if (count > 0 && buf[0] == 's')
		/* sysclk */
		gr->gfxp_wfi_timeout_unit = GFXP_WFI_TIMEOUT_UNIT_SYSCLK;
	else
		/* usec */
		gr->gfxp_wfi_timeout_unit = GFXP_WFI_TIMEOUT_UNIT_USEC;

	if (g->ops.gr.init_preemption_state && g->power_on) {
		err = gk20a_busy(g);
		if (err)
			return err;

		err = gr_gk20a_elpg_protected_call(g,
			g->ops.gr.init_preemption_state(g));

		gk20a_idle(g);

		if (err)
			return err;
	}

	return count;
}

static ssize_t gfxp_wfi_timeout_count_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);
	struct gr_gk20a *gr = &g->gr;
	u32 val = gr->gfxp_wfi_timeout_count;

	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t gfxp_wfi_timeout_unit_read(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);
	struct gr_gk20a *gr = &g->gr;

	if (gr->gfxp_wfi_timeout_unit == GFXP_WFI_TIMEOUT_UNIT_USEC)
		return snprintf(buf, PAGE_SIZE, "usec\n");
	else
		return snprintf(buf, PAGE_SIZE, "sysclk\n");
}

static DEVICE_ATTR(gfxp_wfi_timeout_count, (S_IRWXU|S_IRGRP|S_IROTH),
		gfxp_wfi_timeout_count_read, gfxp_wfi_timeout_count_store);

static DEVICE_ATTR(gfxp_wfi_timeout_unit, (S_IRWXU|S_IRGRP|S_IROTH),
		gfxp_wfi_timeout_unit_read, gfxp_wfi_timeout_unit_store);

static ssize_t comptag_mem_deduct_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct gk20a *g = get_gk20a(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val) < 0)
		return -EINVAL;

	if (val >= totalram_size_in_mb) {
		dev_err(dev, "comptag_mem_deduct can not be set above %lu",
			totalram_size_in_mb);
		return -EINVAL;
	}

	g->gr.comptag_mem_deduct = val;
	/* Deduct the part taken by the running system */
	g->gr.max_comptag_mem -= val;

	return count;
}

static ssize_t comptag_mem_deduct_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct gk20a *g = get_gk20a(dev);

	return sprintf(buf, "%d\n", g->gr.comptag_mem_deduct);
}

static DEVICE_ATTR(comptag_mem_deduct, ROOTRW,
		   comptag_mem_deduct_show, comptag_mem_deduct_store);

void nvgpu_remove_sysfs(struct device *dev)
{
	device_remove_file(dev, &dev_attr_elcg_enable);
	device_remove_file(dev, &dev_attr_blcg_enable);
	device_remove_file(dev, &dev_attr_slcg_enable);
	device_remove_file(dev, &dev_attr_ptimer_scale_factor);
	device_remove_file(dev, &dev_attr_ptimer_ref_freq);
	device_remove_file(dev, &dev_attr_ptimer_src_freq);
	device_remove_file(dev, &dev_attr_elpg_enable);
	device_remove_file(dev, &dev_attr_mscg_enable);
	device_remove_file(dev, &dev_attr_emc3d_ratio);
	device_remove_file(dev, &dev_attr_ldiv_slowdown_factor);

	device_remove_file(dev, &dev_attr_fmax_at_vmin_safe);

	device_remove_file(dev, &dev_attr_counters);
	device_remove_file(dev, &dev_attr_counters_reset);
	device_remove_file(dev, &dev_attr_load);
	device_remove_file(dev, &dev_attr_railgate_delay);
	device_remove_file(dev, &dev_attr_is_railgated);
#ifdef CONFIG_PM
	device_remove_file(dev, &dev_attr_force_idle);
	device_remove_file(dev, &dev_attr_railgate_enable);
#endif
	device_remove_file(dev, &dev_attr_aelpg_param);
	device_remove_file(dev, &dev_attr_aelpg_enable);
	device_remove_file(dev, &dev_attr_allow_all);
	device_remove_file(dev, &dev_attr_tpc_fs_mask);
	device_remove_file(dev, &dev_attr_tpc_pg_mask);
	device_remove_file(dev, &dev_attr_min_timeslice_us);
	device_remove_file(dev, &dev_attr_max_timeslice_us);

#ifdef CONFIG_TEGRA_GK20A_NVHOST
	nvgpu_nvhost_remove_symlink(get_gk20a(dev));
#endif

	device_remove_file(dev, &dev_attr_czf_bypass);
	device_remove_file(dev, &dev_attr_pd_max_batches);
	device_remove_file(dev, &dev_attr_gfxp_wfi_timeout_count);
	device_remove_file(dev, &dev_attr_gfxp_wfi_timeout_unit);
	device_remove_file(dev, &dev_attr_gpu_powered_on);

	device_remove_file(dev, &dev_attr_comptag_mem_deduct);

	if (strcmp(dev_name(dev), "gpu.0")) {
		struct kobject *kobj = &dev->kobj;
		struct device *parent = container_of((kobj->parent),
				struct device, kobj);
		sysfs_remove_link(&parent->kobj, "gpu.0");
	}
}

int nvgpu_create_sysfs(struct device *dev)
{
	struct gk20a *g = get_gk20a(dev);
	int error = 0;

	error |= device_create_file(dev, &dev_attr_elcg_enable);
	error |= device_create_file(dev, &dev_attr_blcg_enable);
	error |= device_create_file(dev, &dev_attr_slcg_enable);
	error |= device_create_file(dev, &dev_attr_ptimer_scale_factor);
	error |= device_create_file(dev, &dev_attr_ptimer_ref_freq);
	error |= device_create_file(dev, &dev_attr_ptimer_src_freq);
	error |= device_create_file(dev, &dev_attr_elpg_enable);
	error |= device_create_file(dev, &dev_attr_mscg_enable);
	error |= device_create_file(dev, &dev_attr_emc3d_ratio);
	error |= device_create_file(dev, &dev_attr_ldiv_slowdown_factor);

	error |= device_create_file(dev, &dev_attr_fmax_at_vmin_safe);

	error |= device_create_file(dev, &dev_attr_counters);
	error |= device_create_file(dev, &dev_attr_counters_reset);
	error |= device_create_file(dev, &dev_attr_load);
	error |= device_create_file(dev, &dev_attr_railgate_delay);
	error |= device_create_file(dev, &dev_attr_is_railgated);
#ifdef CONFIG_PM
	error |= device_create_file(dev, &dev_attr_force_idle);
	error |= device_create_file(dev, &dev_attr_railgate_enable);
#endif
	error |= device_create_file(dev, &dev_attr_aelpg_param);
	error |= device_create_file(dev, &dev_attr_aelpg_enable);
	error |= device_create_file(dev, &dev_attr_allow_all);
	error |= device_create_file(dev, &dev_attr_tpc_fs_mask);
	error |= device_create_file(dev, &dev_attr_tpc_pg_mask);
	error |= device_create_file(dev, &dev_attr_min_timeslice_us);
	error |= device_create_file(dev, &dev_attr_max_timeslice_us);

#ifdef CONFIG_TEGRA_GK20A_NVHOST
	error |= nvgpu_nvhost_create_symlink(g);
#endif

	error |= device_create_file(dev, &dev_attr_czf_bypass);
	error |= device_create_file(dev, &dev_attr_pd_max_batches);
	error |= device_create_file(dev, &dev_attr_gfxp_wfi_timeout_count);
	error |= device_create_file(dev, &dev_attr_gfxp_wfi_timeout_unit);
	error |= device_create_file(dev, &dev_attr_gpu_powered_on);

	error |= device_create_file(dev, &dev_attr_comptag_mem_deduct);

	if (strcmp(dev_name(dev), "gpu.0")) {
		struct kobject *kobj = &dev->kobj;
		struct device *parent = container_of((kobj->parent),
					struct device, kobj);
		error |= sysfs_create_link(&parent->kobj,
				   &dev->kobj, "gpu.0");
	}

	if (error)
		nvgpu_err(g, "Failed to create sysfs attributes!\n");

	return error;
}
