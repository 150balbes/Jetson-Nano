/*
 * Copyright (c) 2015-2018 NVIDIA CORPORATION. All rights reserved.
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

#include <linux/tegra-camera-rtcpu.h>

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/irqchip/tegra-agic.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/platform/tegra/emc_bwmgr.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/seq_buf.h>
#include <linux/slab.h>
#include <linux/tegra-firmwares.h>
#include <linux/tegra-hsp.h>
#include <linux/tegra-ivc-bus.h>
#include <linux/tegra_pm_domains.h>
#include <linux/tegra-rtcpu-monitor.h>
#include <linux/tegra-rtcpu-trace.h>
#include <linux/version.h>
#include <linux/wait.h>

#include <dt-bindings/memory/tegra-swgroup.h>

#include "rtcpu/clk-group.h"
#include "rtcpu/device-group.h"
#include "rtcpu/reset-group.h"

#include "soc/tegra/camrtc-commands.h"
#include "soc/tegra/camrtc-ctrl-commands.h"
#include <linux/tegra-rtcpu-coverage.h>

#ifndef RTCPU_DRIVER_SM5_VERSION
#define RTCPU_DRIVER_SM5_VERSION U32_C(5)
#endif

#if defined(LINUX_VERSION) && (LINUX_VERSION < 409)
#define DISABLE_APE_RUNTIME_PM 1
#else
#define DISABLE_APE_RUNTIME_PM 0
#endif

enum tegra_cam_rtcpu_id {
	TEGRA_CAM_RTCPU_SCE,
	TEGRA_CAM_RTCPU_APE,
	TEGRA_CAM_RTCPU_RCE,
};

#define CAMRTC_NUM_REGS		2
#define CAMRTC_NUM_RESETS	2
#define CAMRTC_NUM_IRQS		1

struct tegra_cam_rtcpu_pdata {
	const char *name;
	void (*assert_resets)(struct device *);
	int (*deassert_resets)(struct device *);
	int (*suspend_core)(struct device *);
	const char * const *reset_names;
	const char * const *reg_names;
	const char * const *irq_names;
	enum tegra_cam_rtcpu_id id;
};

/* Register specifics */
#define TEGRA_APS_FRSC_SC_CTL_0			0x0
#define TEGRA_APS_FRSC_SC_MODEIN_0		0x14
#define TEGRA_PM_R5_CTRL_0			0x40
#define TEGRA_PM_PWR_STATUS_0			0x20

#define TEGRA_R5R_SC_DISABLE			0x5
#define TEGRA_FN_MODEIN				0x29527
#define TEGRA_PM_FWLOADDONE			0x2
#define TEGRA_PM_WFIPIPESTOPPED			0x200000

#define AMISC_ADSP_STATUS			0x14
#define AMISC_ADSP_L2_IDLE			BIT(31)
#define AMISC_ADSP_L2_CLKSTOPPED		BIT(30)

static int tegra_sce_cam_suspend_core(struct device *dev);
static void tegra_sce_cam_assert_resets(struct device *dev);
static int tegra_sce_cam_deassert_resets(struct device *dev);

static int tegra_ape_cam_suspend_core(struct device *dev);
static irqreturn_t tegra_camrtc_adsp_wfi_handler(int irq, void *data);
static void tegra_ape_cam_assert_resets(struct device *dev);
static int tegra_ape_cam_deassert_resets(struct device *dev);

static void tegra_rce_cam_assert_resets(struct device *dev);
static int tegra_rce_cam_deassert_resets(struct device *dev);

static const char * const sce_reset_names[] = {
	"nvidia,reset-group-1",
	"nvidia,reset-group-2",
	NULL,
};

static const char * const sce_reg_names[] = {
	"sce-pm",
	"sce-cfg",
	NULL
};

static const struct tegra_cam_rtcpu_pdata sce_pdata = {
	.name = "sce",
	.suspend_core = tegra_sce_cam_suspend_core,
	.assert_resets = tegra_sce_cam_assert_resets,
	.deassert_resets = tegra_sce_cam_deassert_resets,
	.id = TEGRA_CAM_RTCPU_SCE,
	.reset_names = sce_reset_names,
	.reg_names = sce_reg_names,
};

static const char * const ape_reg_names[] = {
	"ape-amisc",
	NULL,
};

static const char * const ape_reset_names[] = {
	"reset-names",			/* all named resets */
	NULL,
};

static const char * const ape_irq_names[] = {
	"adsp-wfi",
	NULL
};

static const struct tegra_cam_rtcpu_pdata ape_pdata = {
	.name = "ape",
	.assert_resets = tegra_ape_cam_assert_resets,
	.deassert_resets = tegra_ape_cam_deassert_resets,
	.suspend_core = tegra_ape_cam_suspend_core,
	.id = TEGRA_CAM_RTCPU_APE,
	.reset_names = ape_reset_names,
	.reg_names = ape_reg_names,
	.irq_names = ape_irq_names,
};

static const char * const rce_reset_names[] = {
	"reset-names",			/* all named resets */
	NULL,
};

/* SCE and RCE share the PM regs */
static const char * const rce_reg_names[] = {
	"rce-pm",
	NULL,
};

static const struct tegra_cam_rtcpu_pdata rce_pdata = {
	.name = "rce",
	.suspend_core = tegra_sce_cam_suspend_core,
	.assert_resets = tegra_rce_cam_assert_resets,
	.deassert_resets = tegra_rce_cam_deassert_resets,
	.id = TEGRA_CAM_RTCPU_RCE,
	.reset_names = rce_reset_names,
	.reg_names = rce_reg_names,
};

#define NV(p) "nvidia," #p

struct tegra_cam_rtcpu {
	const char *name;
	struct tegra_ivc_bus *ivc;
	struct device_dma_parameters dma_parms;
	struct device *hsp_device;
	struct tegra_hsp_sm_pair *sm_pair;
	struct tegra_rtcpu_trace *tracer;
	struct tegra_rtcpu_coverage *coverage;
	struct {
		struct mutex mutex;
		wait_queue_head_t response_waitq;
		wait_queue_head_t empty_waitq;
		atomic_t response;
		atomic_t emptied;
		u32 timeout;
	} cmd;
	u32 fw_version;
	u8 fw_hash[RTCPU_FW_HASH_SIZE];
	struct {
		u64 reset_complete;
		u64 boot_handshake;
	} stats;
	union {
		void __iomem *regs[CAMRTC_NUM_REGS];
		struct {
			void __iomem *pm_base;
			void __iomem *cfg_base;
		};
		struct {
			void __iomem *amisc_base;
		};
	};
	struct camrtc_clk_group *clocks;
	struct camrtc_reset_group *resets[CAMRTC_NUM_RESETS];
	union {
		int irqs[CAMRTC_NUM_IRQS];
		struct {
			int adsp_wfi_irq;
		};
	};
	const struct tegra_cam_rtcpu_pdata *pdata;
	struct camrtc_device_group *camera_devices;
	struct tegra_bwmgr_client *bwmgr;
	struct tegra_camrtc_mon *monitor;
	unsigned long full_bw;
	u32 max_reboot_retry;
	bool powered;
	bool boot_sync_done;
	bool online;
};

static int tegra_camrtc_mbox_exchange(struct device *dev,
				u32 command, long *timeout);

static void __iomem *tegra_cam_ioremap(struct device *dev, int index)
{
	struct resource mem;
	int err = of_address_to_resource(dev->of_node, index, &mem);
	if (err)
		return IOMEM_ERR_PTR(err);

	/* NOTE: assumes size is large enough for caller */
	return devm_ioremap_resource(dev, &mem);
}

static void __iomem *tegra_cam_ioremap_byname(struct device *dev,
					const char *name)
{
	int index = of_property_match_string(dev->of_node, "reg-names", name);
	if (index < 0)
		return IOMEM_ERR_PTR(-ENOENT);
	return tegra_cam_ioremap(dev, index);
}

static int tegra_camrtc_get_resources(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	const struct tegra_cam_rtcpu_pdata *pdata = rtcpu->pdata;
	struct camrtc_device_group *devgrp;
	int i, err;

	rtcpu->clocks = camrtc_clk_group_get(dev);
	if (IS_ERR(rtcpu->clocks)) {
		err = PTR_ERR(rtcpu->clocks);
		if (err == -EPROBE_DEFER)
			dev_info(dev, "defer %s probe because of %s\n",
				rtcpu->name, "clocks");
		else
			dev_warn(dev, "clocks not available: %d\n", err);
		return err;
	}

	devgrp = camrtc_device_group_get(dev, "nvidia,camera-devices",
		"nvidia,camera-device-names");
	if (!IS_ERR(devgrp)) {
		rtcpu->camera_devices = devgrp;
	} else {
		err = PTR_ERR(devgrp);
		if (err == -EPROBE_DEFER)
			return err;
		if (err != -ENODATA && err != -ENOENT)
			dev_warn(dev, "get %s: failed: %d\n",
				"nvidia,camera-devices", err);
	}

#define GET_RESOURCES(_res_, _get_, _null_, _toerr)	\
	for (i = 0; i < ARRAY_SIZE(rtcpu->_res_##s); i++) { \
		if (!pdata->_res_##_names[i]) \
			break; \
		rtcpu->_res_##s[i] = _get_(dev, pdata->_res_##_names[i]); \
		err = _toerr(rtcpu->_res_##s[i]); \
		if (err == 0) \
			continue; \
		rtcpu->_res_##s[i] = _null_; \
		if (err == -EPROBE_DEFER) { \
			dev_info(dev, "defer %s probe because %s %s\n", \
				rtcpu->name, #_res_, pdata->_res_##_names[i]); \
			return err; \
		} \
		if (err != -ENODATA && err != -ENOENT) \
			dev_warn(dev, "%s %s not available: %d\n", #_res_, \
				pdata->_res_##_names[i], err); \
	}

#define _PTR2ERR(x) (IS_ERR(x) ? PTR_ERR(x) : 0)

	GET_RESOURCES(reset, camrtc_reset_group_get, NULL, _PTR2ERR);
	GET_RESOURCES(reg, tegra_cam_ioremap_byname, NULL, _PTR2ERR);

#undef _PTR2ERR

	if (rtcpu->resets[0] == NULL) {
		struct camrtc_reset_group *resets;

		resets = camrtc_reset_group_get(dev, NULL);

		if (!IS_ERR(resets))
			rtcpu->resets[0] = resets;
		else if (PTR_ERR(resets) == -EPROBE_DEFER) {
			dev_info(dev, "defer %s probe because of %s\n",
				rtcpu->name, "resets");
			return -EPROBE_DEFER;
		}
	}

	return 0;
}

static int tegra_camrtc_get_irqs(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	const struct tegra_cam_rtcpu_pdata *pdata = rtcpu->pdata;
	int i, err;

	/*
	 * AGIC can be touched only after APE is fully powered on.
	 *
	 * This can be called only after runtime resume.
	 */

	if (pdata->irq_names == NULL)
		return 0;

#define _get_irq(_dev, _name) of_irq_get_byname(_dev->of_node, _name)
#define _int2err(x) ((x) < 0 ? (x) : 0)

	GET_RESOURCES(irq, _get_irq, 0, _int2err);

#undef _get_irq
#undef _int2err

	return 0;
}

static int tegra_camrtc_enable_clks(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	return camrtc_clk_group_enable(rtcpu->clocks);
}

static void tegra_camrtc_disable_clks(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	return camrtc_clk_group_disable(rtcpu->clocks);
}

static void tegra_camrtc_assert_resets(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	if (rtcpu->pdata->assert_resets)
		rtcpu->pdata->assert_resets(dev);
}

static int tegra_camrtc_deassert_resets(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	int ret = 0;

	if (rtcpu->pdata->deassert_resets) {
		ret = rtcpu->pdata->deassert_resets(dev);
		rtcpu->stats.reset_complete = ktime_get_ns();
		rtcpu->stats.boot_handshake = 0;
	}

	return ret;
}

static void tegra_camrtc_init_bwmgr(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	u32 bw;

	if (of_property_read_u32(dev->of_node, NV(memory-bw), &bw) != 0)
		return;

	if (bw == 0xFFFFFFFFU)
		rtcpu->full_bw = tegra_bwmgr_get_max_emc_rate();
	else
		rtcpu->full_bw = tegra_bwmgr_round_rate(bw);

	rtcpu->bwmgr = tegra_bwmgr_register(TEGRA_BWMGR_CLIENT_CAMRTC);

	if (IS_ERR_OR_NULL(rtcpu->bwmgr)) {
		dev_warn(dev, "no memory bw manager\n");
		rtcpu->bwmgr = NULL;
		return;
	}

	dev_dbg(dev, "using emc rate %lu for power-on\n", rtcpu->full_bw);
}

static void tegra_camrtc_full_mem_bw(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	if (rtcpu->bwmgr != NULL) {
		int ret = tegra_bwmgr_set_emc(rtcpu->bwmgr, rtcpu->full_bw,
				TEGRA_BWMGR_SET_EMC_FLOOR);
		if (ret < 0)
			dev_info(dev, "emc request rate %lu failed, %d\n",
				rtcpu->full_bw, ret);
		else
			dev_dbg(dev, "requested emc rate %lu\n",
				rtcpu->full_bw);
	}
}

static void tegra_camrtc_slow_mem_bw(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	if (rtcpu->bwmgr != NULL)
		(void)tegra_bwmgr_set_emc(rtcpu->bwmgr, 0,
				TEGRA_BWMGR_SET_EMC_FLOOR);
}

/*
 * Send the PM_SUSPEND command to remote core FW.
 */
static int tegra_camrtc_cmd_pm_suspend(struct device *dev, long *timeout)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	u32 command = RTCPU_COMMAND(PM_SUSPEND, 0);
	int expect = RTCPU_COMMAND(PM_SUSPEND,
			(CAMRTC_PM_CTRL_STATE_SUSPEND << 8) |
			CAMRTC_PM_CTRL_STATUS_OK);
	int err;

	mutex_lock(&rtcpu->cmd.mutex);
	err = tegra_camrtc_mbox_exchange(dev, command, timeout);
	mutex_unlock(&rtcpu->cmd.mutex);

	if (err == expect)
		return 0;

	dev_WARN(dev, "PM_SUSPEND failed: 0x%08x\n", (unsigned)err);
	if (err >= 0)
		err = -EIO;
	return err;
}

static int tegra_sce_cam_deassert_resets(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	int err;

	err = camrtc_reset_group_deassert(rtcpu->resets[0]);
	if (err)
		return err;

	/* Configure R5 core */
	if (rtcpu->cfg_base != NULL) {
		u32 val = readl(rtcpu->cfg_base + TEGRA_APS_FRSC_SC_CTL_0);

		if (val != TEGRA_R5R_SC_DISABLE) {
			/* Disable R5R and smartcomp in camera mode */
			writel(TEGRA_R5R_SC_DISABLE,
				rtcpu->cfg_base + TEGRA_APS_FRSC_SC_CTL_0);

			/* Enable JTAG/Coresight */
			writel(TEGRA_FN_MODEIN,
				rtcpu->cfg_base + TEGRA_APS_FRSC_SC_MODEIN_0);
		}
	}

	/* Group 2 */
	err = camrtc_reset_group_deassert(rtcpu->resets[1]);
	if (err)
		return err;

	/* Group 3: nCPUHALT controlled by PM, not by CAR. */
	if (rtcpu->pm_base != NULL) {
		u32 val = readl(rtcpu->pm_base + TEGRA_PM_R5_CTRL_0);

		writel(val | TEGRA_PM_FWLOADDONE,
			rtcpu->pm_base + TEGRA_PM_R5_CTRL_0);
	}

	return 0;
}

static void tegra_sce_cam_assert_resets(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	camrtc_reset_group_assert(rtcpu->resets[1]);
	camrtc_reset_group_assert(rtcpu->resets[0]);
}

static int tegra_sce_cam_wait_for_wfi(struct device *dev, long *timeout)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	long delay_stride = HZ / 50;

	/* Poll for WFI assert.*/
	for (;;) {
		u32 val = readl(rtcpu->pm_base + TEGRA_PM_PWR_STATUS_0);

		if ((val & TEGRA_PM_WFIPIPESTOPPED) == 0)
			break;

		if (*timeout < 0) {
			dev_WARN(dev, "timeout waiting for WFI\n");
			return -EBUSY;
		}

		msleep(delay_stride);
		*timeout -= delay_stride;
	}

	return 0;
}

static int tegra_sce_cam_suspend_core(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	long timeout = 2 * rtcpu->cmd.timeout;
	u32 val;
	int err;

	err = tegra_camrtc_cmd_pm_suspend(dev, &timeout);

	if (rtcpu->pm_base != NULL) {
		/* Don't bother to check for WFI if core is unresponsive */
		if (err == 0)
			err = tegra_sce_cam_wait_for_wfi(dev, &timeout);

		val = readl(rtcpu->pm_base + TEGRA_PM_R5_CTRL_0);
		writel(val & ~TEGRA_PM_FWLOADDONE,
			rtcpu->pm_base + TEGRA_PM_R5_CTRL_0);
	}

	return err;
}

static void tegra_ape_cam_assert_resets(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	camrtc_reset_group_assert(rtcpu->resets[0]);
}

static int tegra_ape_cam_deassert_resets(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	return camrtc_reset_group_deassert(rtcpu->resets[0]);
}

static irqreturn_t tegra_camrtc_adsp_wfi_handler(int irq, void *data)
{
	struct completion *entered_wfi = data;

	disable_irq_nosync(irq);

	complete(entered_wfi);

	return IRQ_HANDLED;
}

static int tegra_ape_cam_wait_for_l2_idle(struct device *dev, long *timeout)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	long delay_stride = HZ / 50;

	if (rtcpu->amisc_base == NULL) {
		dev_WARN(dev, "iobase \"ape-amisc\" missing\n");
		return 0;
	}

	/* Poll for L2 idle.*/
	for (;;) {
		u32 val = readl(rtcpu->amisc_base + AMISC_ADSP_STATUS);
		u32 mask = AMISC_ADSP_L2_IDLE;

		if ((val & mask) == mask)
			break;

		if (*timeout <= 0) {
			dev_WARN(dev, "timeout waiting for L2 idle\n");
			return -EBUSY;
		}

		msleep(delay_stride);
		*timeout -= delay_stride;
	}

	return 0;
}

static int tegra_ape_cam_wait_for_wfi(struct device *dev, long *timeout)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	DECLARE_COMPLETION_ONSTACK(entered_wfi);
	unsigned int irq = rtcpu->adsp_wfi_irq;
	int err;

	if (irq <= 0) {
		dev_WARN(dev, "irq \"adsp-wfi\" missing\n");
		return 0;
	}

	err = request_threaded_irq(irq,
			tegra_camrtc_adsp_wfi_handler, NULL,
			IRQF_TRIGGER_HIGH,
			"adsp-wfi", &entered_wfi);
	if (err) {
		dev_WARN(dev, "cannot request for %s interrupt: %d\n",
			"adsp-wfi", err);
		return err;
	}

	*timeout = wait_for_completion_timeout(&entered_wfi, *timeout);

	free_irq(irq, &entered_wfi);

	if (*timeout == 0) {
		dev_WARN(dev, "timeout waiting for WFI\n");
		return -EBUSY;
	}

	return 0;
}

static int tegra_ape_cam_suspend_core(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	long timeout = 2 * rtcpu->cmd.timeout;
	int err;

	err = tegra_camrtc_cmd_pm_suspend(dev, &timeout);
	if (err)
		return err;

	err = tegra_ape_cam_wait_for_wfi(dev, &timeout);
	if (err)
		return err;

	return tegra_ape_cam_wait_for_l2_idle(dev, &timeout);
}

static int tegra_rce_cam_deassert_resets(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	int err;

	err = camrtc_reset_group_deassert(rtcpu->resets[0]);
	if (err)
		return err;

	/* nCPUHALT is a reset controlled by PM, not by CAR. */
	if (rtcpu->pm_base != NULL) {
		u32 val = readl(rtcpu->pm_base + TEGRA_PM_R5_CTRL_0);

		writel(val | TEGRA_PM_FWLOADDONE,
			rtcpu->pm_base + TEGRA_PM_R5_CTRL_0);
	}

	return 0;
}

static void tegra_rce_cam_assert_resets(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	camrtc_reset_group_assert(rtcpu->resets[0]);
}

static int tegra_camrtc_suspend_core(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	if (!rtcpu->boot_sync_done || !rtcpu->sm_pair)
		return 0;

	rtcpu->boot_sync_done = false;

	return rtcpu->pdata->suspend_core(dev);
}

static void tegra_camrtc_set_online(struct device *dev, bool online)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	/* No need to report if there is no bus yet */
	if (IS_ERR_OR_NULL(rtcpu->ivc))
		return;

	if (online == rtcpu->online)
		return;

	if (online) {
		/* Activate the IVC services in firmware */
		int ret = tegra_ivc_bus_boot_sync(rtcpu->ivc);
		if (ret < 0) {
			dev_err(dev, "ivc-bus boot sync failed: %d\n", ret);
			return;
		}
	}

	rtcpu->online = online;
	tegra_ivc_bus_ready(rtcpu->ivc, online);
}

static u32 tegra_camrtc_full_notify(void *data, u32 response)
{
	struct tegra_cam_rtcpu *rtcpu = data;

	atomic_set(&rtcpu->cmd.response, response);
	wake_up(&rtcpu->cmd.response_waitq);

	return 0;
}

static void tegra_camrtc_empty_notify(void *data, u32 empty_value)
{
	struct tegra_cam_rtcpu *rtcpu = data;

	atomic_set(&rtcpu->cmd.emptied, 1);
	wake_up(&rtcpu->cmd.empty_waitq);
}

static long tegra_camrtc_wait_for_empty(struct device *dev,
				long timeout)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	if (timeout == 0)
		timeout = 2 * HZ;

	timeout = wait_event_timeout(
		rtcpu->cmd.empty_waitq,
		/* Make sure IRQ has been handled */
		atomic_read(&rtcpu->cmd.emptied) != 0 &&
		tegra_hsp_sm_pair_is_empty(rtcpu->sm_pair),
		timeout);

	if (timeout > 0)
		atomic_set(&rtcpu->cmd.emptied, 0);

	return timeout;
}

static int tegra_camrtc_mbox_exchange(struct device *dev,
					u32 command, long *timeout)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

#define INVALID_RESPONSE (0x80000000U)

	*timeout = tegra_camrtc_wait_for_empty(dev, *timeout);
	if (*timeout <= 0) {
		dev_err(dev, "command: 0x%08x: empty mailbox%s timeout\n",
			command,
			tegra_hsp_sm_pair_is_empty(rtcpu->sm_pair) ?
			" interrupt" : "");
		return -ETIMEDOUT;
	}

	atomic_set(&rtcpu->cmd.response, INVALID_RESPONSE);

	tegra_hsp_sm_pair_write(rtcpu->sm_pair, command);

	*timeout = wait_event_timeout(
		rtcpu->cmd.response_waitq,
		atomic_read(&rtcpu->cmd.response) != INVALID_RESPONSE,
		*timeout);
	if (*timeout <= 0) {
		dev_err(dev, "command: 0x%08x: response timeout\n", command);
		return -ETIMEDOUT;
	}

	return (int)atomic_read(&rtcpu->cmd.response);
}

int tegra_camrtc_command(struct device *dev, u32 command, long timeout)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	int response;

	if (timeout == 0)
		timeout = rtcpu->cmd.timeout;

	mutex_lock(&rtcpu->cmd.mutex);

	response = tegra_camrtc_mbox_exchange(dev, command, &timeout);

	mutex_unlock(&rtcpu->cmd.mutex);

	return response;
}
EXPORT_SYMBOL(tegra_camrtc_command);

int tegra_camrtc_prefix_command(struct device *dev,
				u32 prefix, u32 command, long timeout)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	int response;

	if (timeout == 0)
		timeout = rtcpu->cmd.timeout;

	mutex_lock(&rtcpu->cmd.mutex);

	prefix = RTCPU_COMMAND(PREFIX, prefix);
	response = tegra_camrtc_mbox_exchange(dev, prefix, &timeout);

	if (RTCPU_GET_COMMAND_ID(response) == RTCPU_CMD_PREFIX)
		response = tegra_camrtc_mbox_exchange(dev, command, &timeout);

	mutex_unlock(&rtcpu->cmd.mutex);

	return response;
}
EXPORT_SYMBOL(tegra_camrtc_prefix_command);

static int tegra_camrtc_poweron(struct device *dev, bool full_speed)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	int ret;

	if (rtcpu->powered) {
		if (full_speed)
			camrtc_clk_group_adjust_fast(rtcpu->clocks);
		return 0;
	}

	/* APE power domain may misbehave and try to resume while probing */
	if (rtcpu->sm_pair == NULL) {
		dev_info(dev, "poweron while probing");
		return 0;
	}

	/* Power on and let core run */
	ret = tegra_camrtc_enable_clks(dev);
	if (ret) {
		dev_err(dev, "failed to turn on %s clocks: %d\n",
			rtcpu->name, ret);
		return ret;
	}

	if (full_speed)
		camrtc_clk_group_adjust_fast(rtcpu->clocks);
	camrtc_device_group_reset(rtcpu->camera_devices);

	ret = tegra_camrtc_deassert_resets(dev);
	if (ret)
		return ret;

	rtcpu->powered = true;

	return 0;
}

static void tegra_camrtc_poweroff(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	if (!rtcpu->powered)
		return;

	rtcpu->powered = false;

	tegra_camrtc_assert_resets(dev);
	tegra_camrtc_disable_clks(dev);
}

static int tegra_camrtc_boot_sync(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	int ret;
	u32 command;

	if (rtcpu->boot_sync_done)
		return 0;

	/*
	 * Handshake FW version before continuing with the boot
	 */
	command = RTCPU_COMMAND(INIT, 0);
	ret = tegra_camrtc_command(dev, command, 0);
	if (ret < 0)
		return ret;
	if (ret != command) {
		dev_err(dev, "RTCPU sync problem (response=0x%08x)\n", ret);
		return -EIO;
	}

	if (rtcpu->stats.boot_handshake == 0) {
		u64 bt;

		rtcpu->stats.boot_handshake = ktime_get_ns();

		bt = rtcpu->stats.boot_handshake -
			rtcpu->stats.reset_complete + 500U;

		dev_dbg(dev, "boot time %llu.%03u ms\n", bt / 1000000U,
			(unsigned int)(bt % 10000000U) / 1000U);
	}

	command = RTCPU_COMMAND(FW_VERSION, RTCPU_DRIVER_SM5_VERSION);
	ret = tegra_camrtc_command(dev, command, 0);
	if (ret < 0)
		return ret;
	if (RTCPU_GET_COMMAND_ID(ret) != RTCPU_CMD_FW_VERSION ||
		RTCPU_GET_COMMAND_VALUE(ret) < RTCPU_FW_SM4_VERSION) {
		dev_err(dev, "RTCPU version mismatch (response=0x%08x)\n", ret);
		return -EIO;
	}

	rtcpu->fw_version = RTCPU_GET_COMMAND_VALUE(ret);
	rtcpu->boot_sync_done = true;

	/*
	 * Enable trace
	 */
	if (rtcpu->tracer) {
		ret = tegra_rtcpu_trace_boot_sync(rtcpu->tracer);
		if (ret < 0) {
			dev_err(dev, "trace boot sync failed: %d\n", ret);
			return ret;
		}
	}

	if (rtcpu->coverage != NULL) {
		ret = tegra_rtcpu_coverage_boot_sync(rtcpu->coverage);
		if (ret < 0) {
			dev_dbg(dev, "coverage boot sync status: %d\n", ret);
			/*
			 * Not a fatal error, don't stop the sync.
			 * But go ahead and remove the coverage debug FS
			 * entries and release the memory.
			 */
			tegra_rtcpu_coverage_destroy(rtcpu->coverage);
			rtcpu->coverage = NULL;
		}
	}

	return 0;
}

/*
 * RTCPU boot sequence
 */
static int tegra_camrtc_boot(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	int retry = 0, max_retries = rtcpu->max_reboot_retry;
	int ret;

	ret = tegra_camrtc_poweron(dev, true);
	if (ret)
		return ret;

	tegra_camrtc_full_mem_bw(dev);

	for (;;) {
		ret = tegra_camrtc_boot_sync(dev);
		if (ret == 0)
			break;
		if (retry++ == max_retries)
			break;
		dev_warn(dev, "%s full reset, retry %u/%u\n",
			rtcpu->name, retry, max_retries);
		tegra_camrtc_assert_resets(dev);
		usleep_range(10, 30);
		tegra_camrtc_deassert_resets(dev);
	}

	if (ret == 0) {
		tegra_camrtc_set_online(dev, true);
	}

	tegra_camrtc_slow_mem_bw(dev);

	return 0;
}

int tegra_camrtc_iovm_setup(struct device *dev, dma_addr_t iova)
{
	u32 command = RTCPU_COMMAND(CH_SETUP, iova >> 8);
	int ret = tegra_camrtc_command(dev, command, 0);

	if (ret < 0)
		return -ret;

	if (RTCPU_GET_COMMAND_ID(ret) == RTCPU_CMD_ERROR) {
		u32 error = RTCPU_GET_COMMAND_VALUE(ret);

		dev_dbg(dev, "IOVM setup error: %u\n", error);

		return (int)error;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_camrtc_iovm_setup);

static int tegra_camrtc_get_fw_hash(struct device *dev,
				u8 hash[RTCPU_FW_HASH_SIZE])
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	int ret, i;
	u32 value;

	if (rtcpu->fw_version < RTCPU_FW_SM2_VERSION) {
		dev_info(dev, "fw version %u has no sha1\n", rtcpu->fw_version);
		return -EIO;
	}

	for (i = 0; i < RTCPU_FW_HASH_SIZE; i++) {
		ret = tegra_camrtc_command(dev, RTCPU_COMMAND(FW_HASH, i), 0);
		value = RTCPU_GET_COMMAND_VALUE(ret);

		if (ret < 0 ||
			RTCPU_GET_COMMAND_ID(ret) != RTCPU_CMD_FW_HASH ||
			value > (u8)~0) {
			dev_warn(dev, "FW_HASH problem (0x%08x)\n", ret);
			return -EIO;
		}

		hash[i] = value;
	}

	return 0;
}

ssize_t tegra_camrtc_print_version(struct device *dev,
					char *buf, size_t size)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	struct seq_buf s;
	int i;

	seq_buf_init(&s, buf, size);
	seq_buf_printf(&s, "version cpu=%s cmd=%u sha1=",
		rtcpu->name, rtcpu->fw_version);

	for (i = 0; i < RTCPU_FW_HASH_SIZE; i++)
		seq_buf_printf(&s, "%02x", rtcpu->fw_hash[i]);

	return seq_buf_used(&s);
}
EXPORT_SYMBOL(tegra_camrtc_print_version);

static void tegra_camrtc_log_fw_version(struct device *dev)
{
	char version[TEGRA_CAMRTC_VERSION_LEN];

	tegra_camrtc_print_version(dev, version, sizeof(version));

	dev_info(dev, "firmware %s\n", version);
}

static int tegra_cam_rtcpu_runtime_suspend(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	int err;

	err = tegra_camrtc_suspend_core(dev);
	/* Try full reset if an error occurred while suspending core. */
	if (WARN(err < 0, "RTCPU suspend failed, resetting it")) {
		/* runtime_resume() powers RTCPU back on */
		tegra_camrtc_poweroff(dev);
		tegra_camrtc_set_online(dev, false);
	}

	camrtc_clk_group_adjust_slow(rtcpu->clocks);

	camrtc_device_group_idle(rtcpu->camera_devices);

	return 0;
}

static int tegra_cam_rtcpu_runtime_resume(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	int ret;

	ret = camrtc_device_group_busy(rtcpu->camera_devices);
	if (ret < 0)
		return ret;

	ret = tegra_camrtc_boot(dev);

	if (ret < 0)
		camrtc_device_group_idle(rtcpu->camera_devices);

	return ret;
}

static struct device *tegra_camrtc_get_hsp_device(struct device_node *hsp_node)
{
	struct device_node *of_node;
	struct platform_device *pdev;

	of_node = of_parse_phandle(hsp_node, "device", 0);
	if (of_node == NULL)
		return NULL;

	pdev = of_find_device_by_node(of_node);
	of_node_put(of_node);

	if (pdev == NULL)
		return ERR_PTR(-EPROBE_DEFER);

	if (&pdev->dev.driver == NULL) {
		platform_device_put(pdev);
		return ERR_PTR(-EPROBE_DEFER);
	}

	return &pdev->dev;
}

static int tegra_camrtc_mbox_init(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);
	struct device_node *hsp_node;

	if (!IS_ERR_OR_NULL(rtcpu->sm_pair))
		return 0;

	mutex_init(&rtcpu->cmd.mutex);
	init_waitqueue_head(&rtcpu->cmd.response_waitq);
	init_waitqueue_head(&rtcpu->cmd.empty_waitq);

	hsp_node = of_get_child_by_name(dev->of_node, "hsp");
	rtcpu->hsp_device = tegra_camrtc_get_hsp_device(hsp_node);
	if (IS_ERR(rtcpu->hsp_device)) {
		of_node_put(hsp_node);
		return PTR_ERR(rtcpu->hsp_device);
	}

	if (rtcpu->hsp_device != NULL) {
		int ret = pm_runtime_get_sync(rtcpu->hsp_device);
		if (ret < 0) {
			dev_warn(rtcpu->hsp_device,
				"power on failure: %d\n", ret);
			put_device(rtcpu->hsp_device);
			rtcpu->hsp_device = NULL;
			return ret;
		}
	}

	rtcpu->sm_pair = of_tegra_hsp_sm_pair_by_name(hsp_node,
					"cmd-pair", tegra_camrtc_full_notify,
					tegra_camrtc_empty_notify, rtcpu);
	of_node_put(hsp_node);
	if (IS_ERR(rtcpu->sm_pair)) {
		int ret = PTR_ERR(rtcpu->sm_pair);
		if (ret != -EPROBE_DEFER)
			dev_err(dev, "failed to obtain %s mbox pair: %d\n",
				rtcpu->name, ret);
		rtcpu->sm_pair = NULL;
		return ret;
	}

	return 0;
}

static int tegra_cam_rtcpu_remove(struct platform_device *pdev)
{
	struct tegra_cam_rtcpu *rtcpu = platform_get_drvdata(pdev);
	bool pm_is_active = pm_runtime_active(&pdev->dev);

	pm_runtime_disable(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);

	tegra_camrtc_set_online(&pdev->dev, false);

	if (rtcpu->sm_pair) {
		if (pm_is_active)
			tegra_cam_rtcpu_runtime_suspend(&pdev->dev);
		tegra_hsp_sm_pair_free(rtcpu->sm_pair);
		rtcpu->sm_pair = NULL;
	}

	if (!IS_ERR_OR_NULL(rtcpu->hsp_device)) {
		pm_runtime_put(rtcpu->hsp_device);
		put_device(rtcpu->hsp_device);
	}

	tegra_rtcpu_trace_destroy(rtcpu->tracer);
	rtcpu->tracer = NULL;
	tegra_rtcpu_coverage_destroy(rtcpu->coverage);
	rtcpu->coverage = NULL;

	tegra_camrtc_poweroff(&pdev->dev);
	if (rtcpu->bwmgr != NULL)
		tegra_bwmgr_unregister(rtcpu->bwmgr);
	rtcpu->bwmgr = NULL;
	tegra_pd_remove_device(&pdev->dev);
	tegra_cam_rtcpu_mon_destroy(rtcpu->monitor);
	tegra_ivc_bus_destroy(rtcpu->ivc);

	pdev->dev.dma_parms = NULL;

	return 0;
}

static struct device *s_dev;

static int tegra_cam_rtcpu_probe(struct platform_device *pdev)
{
	struct tegra_cam_rtcpu *rtcpu;
	const struct tegra_cam_rtcpu_pdata *pdata;
	struct device *dev = &pdev->dev;
	int ret;
	const char *name;
	uint32_t timeout;

	pdata = of_device_get_match_data(dev);
	if (pdata == NULL) {
		dev_err(dev, "no device match\n");
		return -ENODEV;
	}

	name = pdata->name;
	of_property_read_string(dev->of_node, NV(cpu-name), &name);

	dev_dbg(dev, "probing RTCPU on %s\n", name);

	rtcpu = devm_kzalloc(dev, sizeof(*rtcpu), GFP_KERNEL);
	if (rtcpu == NULL)
		return -ENOMEM;

	rtcpu->pdata = pdata;
	rtcpu->name = name;
	platform_set_drvdata(pdev, rtcpu);

	/* Enable runtime power management */
	pm_runtime_enable(dev);

	ret = tegra_camrtc_get_resources(dev);
	if (ret)
		goto fail;

	rtcpu->max_reboot_retry = 2;
	(void)of_property_read_u32(dev->of_node, NV(max-reboot),
			&rtcpu->max_reboot_retry);

	timeout = 2000;
	(void)of_property_read_u32(dev->of_node, NV(cmd-timeout), &timeout);
	rtcpu->cmd.timeout = msecs_to_jiffies(timeout);

	timeout = 60000;
	ret = of_property_read_u32(dev->of_node, NV(autosuspend-delay-ms), &timeout);
	if (ret == 0) {
		pm_runtime_use_autosuspend(dev);
		pm_runtime_set_autosuspend_delay(&pdev->dev, timeout);
	}

	tegra_camrtc_init_bwmgr(dev);

	dev->dma_parms = &rtcpu->dma_parms;
	dma_set_max_seg_size(dev, UINT_MAX);

	rtcpu->tracer = tegra_rtcpu_trace_create(dev, rtcpu->camera_devices);

	rtcpu->coverage = tegra_rtcpu_coverage_create(dev);

	ret = tegra_camrtc_mbox_init(dev);
	if (ret)
		goto fail;

	/* Power on device */
	ret = pm_runtime_get_sync(dev);
	if (ret < 0)
		goto fail;

	/* Clocks are on, resets are deasserted, we can touch the hardware */

	/* Tegra-agic driver routes IRQs when probing, do it when powered */
	ret = tegra_camrtc_get_irqs(dev);
	if (ret)
		goto put_and_fail;

	rtcpu->ivc = tegra_ivc_bus_create(dev);
	if (IS_ERR(rtcpu->ivc)) {
		ret = PTR_ERR(rtcpu->ivc);
		goto put_and_fail;
	}

	rtcpu->monitor = tegra_camrtc_mon_create(dev);
	if (IS_ERR(rtcpu->monitor)) {
		ret = PTR_ERR(rtcpu->monitor);
		goto put_and_fail;
	}

	if (of_property_read_bool(dev->of_node, NV(disable-runtime-pm)) ||
		/* APE power domain powergates APE block when suspending */
		/* This won't do */
		(DISABLE_APE_RUNTIME_PM && pdata->id == TEGRA_CAM_RTCPU_APE)) {
		pm_runtime_get(dev);
	}

	ret = tegra_camrtc_get_fw_hash(dev, rtcpu->fw_hash);
	if (ret == 0)
		devm_tegrafw_register(dev,
			name != pdata->name ? name :  "camrtc",
			TFW_NORMAL, tegra_camrtc_print_version, NULL);

	tegra_camrtc_set_online(dev, true);

	pm_runtime_put(dev);

	/* Print firmware version */
	tegra_camrtc_log_fw_version(dev);

	s_dev = dev;

	dev_dbg(dev, "successfully probed RTCPU on %s\n", name);

	return 0;

put_and_fail:
	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_put_sync_suspend(dev);
fail:
	tegra_cam_rtcpu_remove(pdev);
	return ret;
}

int tegra_camrtc_reboot(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	if (pm_runtime_suspended(dev)) {
		dev_info(dev, "cannot reboot while suspended\n");
		return -EIO;
	}

	if (!rtcpu->powered)
		return -EIO;

	rtcpu->boot_sync_done = false;

	pm_runtime_mark_last_busy(dev);

	tegra_camrtc_set_online(dev, false);

	tegra_camrtc_assert_resets(dev);

	rtcpu->powered = false;

	return tegra_camrtc_boot(dev);
}
EXPORT_SYMBOL(tegra_camrtc_reboot);

int tegra_camrtc_restore(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	if (rtcpu->monitor)
		return tegra_camrtc_mon_restore_rtcpu(rtcpu->monitor);
	else
		return tegra_camrtc_reboot(dev);
}
EXPORT_SYMBOL(tegra_camrtc_restore);

bool tegra_camrtc_is_rtcpu_alive(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	return rtcpu->online;
}
EXPORT_SYMBOL(tegra_camrtc_is_rtcpu_alive);

bool tegra_camrtc_is_rtcpu_powered(void)
{
	struct tegra_cam_rtcpu *rtcpu;

	if (s_dev) {
		rtcpu = dev_get_drvdata(s_dev);
		return rtcpu->powered;
	}

	return false;
}
EXPORT_SYMBOL(tegra_camrtc_is_rtcpu_powered);

void tegra_camrtc_flush_trace(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	tegra_rtcpu_trace_flush(rtcpu->tracer);
}
EXPORT_SYMBOL(tegra_camrtc_flush_trace);

static int tegra_camrtc_halt(struct device *dev)
{
	struct tegra_cam_rtcpu *rtcpu = dev_get_drvdata(dev);

	tegra_camrtc_set_online(dev, false);

	if (!rtcpu->powered)
		return 0;

	if (!pm_runtime_suspended(dev))
		tegra_camrtc_suspend_core(dev);

	tegra_camrtc_poweroff(dev);

	return 0;
}

static int tegra_camrtc_resume(struct device *dev)
{
	return tegra_camrtc_poweron(dev, false);
}

static void tegra_cam_rtcpu_shutdown(struct platform_device *pdev)
{
	tegra_camrtc_halt(&pdev->dev);
}

static const struct of_device_id tegra_cam_rtcpu_of_match[] = {
	{
		.compatible = NV(tegra186-sce-ivc), .data = &sce_pdata
	},
	{
		.compatible = NV(tegra186-ape-ivc), .data = &ape_pdata
	},
	{
		.compatible = NV(tegra194-rce), .data = &rce_pdata
	},
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_cam_rtcpu_of_match);

static const struct dev_pm_ops tegra_cam_rtcpu_pm_ops = {
	.suspend = tegra_camrtc_halt,
	.resume = tegra_camrtc_resume,
	.runtime_suspend = tegra_cam_rtcpu_runtime_suspend,
	.runtime_resume = tegra_cam_rtcpu_runtime_resume,
};

static struct platform_driver tegra_cam_rtcpu_driver = {
	.driver = {
		.name	= "tegra186-cam-rtcpu",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(tegra_cam_rtcpu_of_match),
#ifdef CONFIG_PM
		.pm = &tegra_cam_rtcpu_pm_ops,
#endif
	},
	.probe = tegra_cam_rtcpu_probe,
	.remove = tegra_cam_rtcpu_remove,
	.shutdown = tegra_cam_rtcpu_shutdown,
};
module_platform_driver(tegra_cam_rtcpu_driver);

MODULE_DESCRIPTION("CAMERA RTCPU driver");
MODULE_AUTHOR("NVIDIA");
MODULE_LICENSE("GPL v2");
