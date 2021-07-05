/*
 * SLVS-EC driver for T194
 *
 * Copyright (c) 2017-2019, NVIDIA Corporation.  All rights reserved.
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

#include "slvsec.h"
#include <media/slvs.h>

#include <asm/ioctls.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <soc/tegra/chip-id.h>
#include <media/tegra_camera_platform.h>

#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "t194/t194.h"

#define SLVSEC_CORE_INTR_STATUS			U32_C(0x84)
#define SLVSEC_CORE_INTR_STATUS_SW		BIT(4)
#define SLVSEC_CORE_INTR_STATUS_HOST1X		BIT(3)
#define SLVSEC_CORE_INTR_STATUS_CIL		BIT(2)
#define SLVSEC_CORE_INTR_STATUS_STRM_1		BIT(1)
#define SLVSEC_CORE_INTR_STATUS_STRM_0		BIT(0)

#define SLVSEC_CORE_HOST1X_INTR_STATUS		U32_C(0xb0)
#define SLVSEC_CORE_HOST1X_INTR_MASK		U32_C(0xb4)

#define SLVSEC_CORE_DBG_CNT_CTRL		U32_C(0x8c)
#define SLVSEC_CORE_DBG_CNT_CTRL_EVENT_MASK	U32_C(0x3f)
#define SLVSEC_CORE_DBG_CNT_CTRL_EVENT_SHIFT	U32_C(8)
#define SLVSEC_CORE_DBG_CNT_CTRL_STRM_SEL_MASK	U32_C(0x1)
#define SLVSEC_CORE_DBG_CNT_CTRL_STRM_SEL_SHIFT	U32_C(0)

#define SLVSEC_CORE_DBG_CNT_VALUE		U32_C(0x90)
#define SLVSEC_CORE_DBG_CNT_SIZE		U32_C(0x08)

#define SLVSEC_CORE_NUM_DEBUG_COUNTERS		4U

#define SLVSEC_CORE_SW_DEBUG_INTR_STATUS	U32_C(0xb8)
#define SLVSEC_CORE_SW_DEBUG_INTR_TRIG		U32_C(0xbc)

#define SLVSEC_CORE_STRM0_INTR_STATUS_CH0	U32_C(0x10048)
#define SLVSEC_CORE_STRM0_INTR_STATUS_CH1	U32_C(0x1004c)
#define SLVSEC_CORE_STRM0_INTR_STATUS		U32_C(0x10050)

#define SLVSEC_CORE_STRM1_INTR_STATUS_CH0	U32_C(0x20048)
#define SLVSEC_CORE_STRM1_INTR_STATUS_CH1	U32_C(0x2004c)
#define SLVSEC_CORE_STRM1_INTR_STATUS		U32_C(0x20050)

#define SLVSEC_CIL_STRM0_INTR_STATUS		U32_C(0x30818)
#define SLVSEC_CIL_STRM1_INTR_STATUS		U32_C(0x31018)
#define SLVSEC_CIL_STRM0_INTR_MASK		U32_C(0x3081c)
#define SLVSEC_CIL_STRM1_INTR_MASK		U32_C(0x3181c)

#define SLVSEC_CIL_STRM_INTR_STATUS_CAL_DONE	BIT(3)

/* HW capabilities */
#define BUS_WIDTH	64
#define LANE_NUM	8
/* split in two to satisfy static code analysis tool */
#define LANE_SPEED_MHZ	U64_C(2304)
#define ONE_MILLION	U64_C(1000000)

/* uses 8b10b encoding */
#define ENCODE_NR	8
#define ENCODE_DR	10

struct slvsec {
	struct platform_device *pdev;
	struct tegra_mc_slvs *mc_slvs;
	int irq;
	int vi_irq;

	/* Debugfs */
	struct slvsec_debug {
		struct debugfs_regset32 core;
		struct debugfs_regset32 core_stream0;
		struct debugfs_regset32 core_stream1;
		struct debugfs_regset32 cil;
		struct debugfs_regset32 vi_syncgen0;
		struct debugfs_regset32 vi_syncgen1;
		struct debugfs_regset32 vi_syncgen2;
		struct slvsec_debug_counter {
			struct dentry *dir;
			uint8_t number;
			uint8_t event;
			uint8_t stream;
		} counters[SLVSEC_CORE_NUM_DEBUG_COUNTERS];
	} debug;
};

static int slvsec_init_debugfs(struct slvsec *slvsec);
static void slvsec_remove_debugfs(struct slvsec *slvsec);

#ifdef SLVSEC_ENABLE_IRQ
static irqreturn_t slvsec_isr(int irq, void *arg)
{
	struct platform_device *pdev = arg;
	struct slvsec *slvsec = nvhost_get_private_data(pdev);
	u32 status, val;
	u32 strm0_cil_status = 0;
	u32 strm1_cil_status = 0;

	status = host1x_readl(pdev, SLVSEC_CORE_INTR_STATUS);
	dev_info(&pdev->dev, "INTR: %08x\n", status);

	if (status & SLVSEC_CORE_INTR_STATUS_SW) {
		val = host1x_readl(pdev, SLVSEC_CORE_SW_DEBUG_INTR_STATUS);
		host1x_writel(pdev, SLVSEC_CORE_SW_DEBUG_INTR_STATUS, val);
		dev_info(&pdev->dev, "SW_DEBUG_INTR: %08x\n", val);
	}

	if (status & SLVSEC_CORE_INTR_STATUS_HOST1X) {
		val = host1x_readl(pdev, SLVSEC_CORE_HOST1X_INTR_STATUS);
		host1x_writel(pdev, SLVSEC_CORE_HOST1X_INTR_STATUS, val);
		dev_info(&pdev->dev, "HOST1X_INTR: %08x\n", val);
	}

	if (status & SLVSEC_CORE_INTR_STATUS_CIL) {
		val = host1x_readl(pdev, SLVSEC_CIL_STRM0_INTR_STATUS);
		host1x_writel(pdev, SLVSEC_CIL_STRM0_INTR_STATUS, val);
		dev_info(&pdev->dev, "CIL_STRM0_INTR: %08x\n", val);
		strm0_cil_status = val;

		val = host1x_readl(pdev, SLVSEC_CIL_STRM1_INTR_STATUS);
		host1x_writel(pdev, SLVSEC_CIL_STRM1_INTR_STATUS, val);
		dev_info(&pdev->dev, "CIL_STRM1_INTR: %08x\n", val);
		strm1_cil_status = val;
	}

	if (status & SLVSEC_CORE_INTR_STATUS_STRM_1) {
		val = host1x_readl(pdev, SLVSEC_CORE_STRM1_INTR_STATUS);
		host1x_writel(pdev, SLVSEC_CORE_STRM1_INTR_STATUS, val);
		dev_info(&pdev->dev, "CORE_STRM1_INTR: %08x\n", val);


		val = host1x_readl(pdev, SLVSEC_CORE_STRM1_INTR_STATUS_CH0);
		host1x_writel(pdev, SLVSEC_CORE_STRM1_INTR_STATUS_CH0, val);
		dev_info(&pdev->dev, "CORE_STRM1_CH0_INTR: %08x\n", val);

		val = host1x_readl(pdev, SLVSEC_CORE_STRM1_INTR_STATUS_CH1);
		host1x_writel(pdev, SLVSEC_CORE_STRM1_INTR_STATUS_CH1, val);
		dev_info(&pdev->dev, "CORE_STRM1_CH1_INTR: %08x\n", val);
	}

	if (status & SLVSEC_CORE_INTR_STATUS_STRM_0) {
		val = host1x_readl(pdev, SLVSEC_CORE_STRM0_INTR_STATUS);
		host1x_writel(pdev, SLVSEC_CORE_STRM0_INTR_STATUS, val);
		dev_info(&pdev->dev, "CORE_STRM0_INTR: %08x\n", val);

		val = host1x_readl(pdev, SLVSEC_CORE_STRM0_INTR_STATUS_CH0);
		host1x_writel(pdev, SLVSEC_CORE_STRM0_INTR_STATUS_CH0, val);
		dev_info(&pdev->dev, "CORE_STRM0_CH0_INTR: %08x\n", val);

		val = host1x_readl(pdev, SLVSEC_CORE_STRM0_INTR_STATUS_CH1);
		host1x_writel(pdev, SLVSEC_CORE_STRM0_INTR_STATUS_CH1, val);
		dev_info(&pdev->dev, "CORE_STRM0_CH1_INTR: %08x\n", val);
	}

	if (strm0_cil_status)
		tegra_slvs_media_controller_cil_notify(slvsec->mc_slvs,
						0, strm0_cil_status);

	if (strm1_cil_status)
		tegra_slvs_media_controller_cil_notify(slvsec->mc_slvs,
						1, strm1_cil_status);

	return IRQ_HANDLED;
}

static irqreturn_t slvsec_vi_isr(int irq, void *arg)
{
	struct platform_device *pdev = arg;

	dev_info(&pdev->dev, "%s()\n", __func__);

	return IRQ_HANDLED;
}

static int slvsec_get_irq(struct platform_device *pdev,
			char const *name,
			int *irqslot,
			irq_handler_t thread_fn)
{
	struct device *dev = &pdev->dev;
	int ret = of_irq_get_byname(dev->of_node, name);
	const char *devname;

	if (ret == 0)
		ret = -ENXIO;

	if (ret < 0) {
		dev_err(dev, "No %s irq available\n", name);
		*irqslot = -ENXIO;
		return ret;
	}

	*irqslot = ret;

	devname = devm_kasprintf(dev, GFP_KERNEL, "%s:%s",
				dev_name(dev), name);

	ret = devm_request_threaded_irq(dev, *irqslot,
					NULL, thread_fn, IRQF_ONESHOT,
					devname, pdev);
	if (ret < 0) {
		dev_err(dev, "Cannot setup %s irq\n", name);
		*irqslot = -ENXIO;
	}

	return ret;
}
#endif

int slvsec_finalize_poweron(struct platform_device *pdev)
{
	struct slvsec *slvsec = nvhost_get_private_data(pdev);
	int i;

	for (i = 0; i < ARRAY_SIZE(slvsec->debug.counters); i++) {
		struct slvsec_debug_counter *counter;
		u32 reg, val;

		counter = &slvsec->debug.counters[i];
		reg = SLVSEC_CORE_DBG_CNT_CTRL + SLVSEC_CORE_DBG_CNT_SIZE * i;
		val = (counter->event << SLVSEC_CORE_DBG_CNT_CTRL_EVENT_SHIFT)
			| counter->stream;
		host1x_writel(pdev, reg, val);

		reg = SLVSEC_CORE_DBG_CNT_VALUE + SLVSEC_CORE_DBG_CNT_SIZE * i;
		host1x_writel(pdev, reg, 0);
	}

	if (!tegra_platform_is_silicon()) {
		host1x_writel(pdev, SLVSEC_CORE_HOST1X_INTR_STATUS, 0x1);
		host1x_writel(pdev, SLVSEC_CORE_STRM0_INTR_STATUS, 0x9);
		host1x_writel(pdev, SLVSEC_CIL_STRM0_INTR_MASK, 0x1f);
		host1x_writel(pdev, SLVSEC_CORE_STRM0_INTR_STATUS_CH0, 0xf);
	}

	return 0;
}
EXPORT_SYMBOL(slvsec_finalize_poweron);

int slvsec_prepare_poweroff(struct platform_device *pdev)
{
	return 0;
}
EXPORT_SYMBOL(slvsec_prepare_poweroff);

static int slvsec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvhost_device_data *info;
	struct device_node *vi_np;
	struct slvsec *slvsec;
	struct tegra_camera_dev_info slvsec_info;
	int err = 0;

	info = (void *)of_device_get_match_data(dev);
	if (unlikely(info == NULL)) {
		dev_WARN(dev, "no platform data\n");
		return -ENODATA;
	}

	slvsec = devm_kzalloc(dev, sizeof(*slvsec), GFP_KERNEL);
	if (!slvsec)
		return -ENOMEM;

	vi_np = of_parse_phandle(dev->of_node, "nvidia,vi-device", 0);
	if (vi_np == NULL) {
		dev_WARN(dev, "missing %s handle\n", "nvidia,vi-device");
		return -ENODEV;
	}

	slvsec->pdev = pdev;
	info->pdev = pdev;
	mutex_init(&info->lock);
	platform_set_drvdata(pdev, info);
	info->private_data = slvsec;

#ifdef SLVSEC_ENABLE_IRQ
	if (tegra_platform_is_silicon()) {
		slvsec_get_irq(pdev, "slvs-ec", &slvsec->irq, slvsec_isr);
		slvsec_get_irq(pdev, "syncgen", &slvsec->vi_irq, slvsec_vi_isr);
	}
#endif

	err = nvhost_client_device_get_resources(pdev);
	if (err)
		goto error;

	err = nvhost_module_init(pdev);
	if (err)
		goto error;

	err = nvhost_client_device_init(pdev);
	if (err)
		goto deinit;

	memset(&slvsec_info, 0, sizeof(slvsec_info));
	slvsec_info.pdev = pdev;
	slvsec_info.hw_type = HWTYPE_SLVSEC;
	slvsec_info.use_max = true;
	slvsec_info.lane_speed =
		LANE_SPEED_MHZ * ONE_MILLION * ENCODE_NR / ENCODE_DR;
	slvsec_info.lane_num = LANE_NUM;
	slvsec_info.bus_width = BUS_WIDTH;
	err = tegra_camera_device_register(&slvsec_info, slvsec);
	if (err)
		goto deinit;

	slvsec_init_debugfs(slvsec);

#ifdef SLVSEC_ENABLE_IRQ
	dev_info(dev, "clearing pending interrupts\n");
	if (tegra_platform_is_silicon())
		slvsec_isr(slvsec->irq, pdev);
#endif

	slvsec->mc_slvs	= tegra_slvs_media_controller_init(pdev);
	if (IS_ERR(slvsec->mc_slvs)) {
		err = (int)PTR_ERR(slvsec->mc_slvs);
		dev_info(dev, "failed to init SLVS media controller\n");
		goto deinit;
	}

	dev_info(dev, "probed\n");

	return 0;

deinit:
	nvhost_module_deinit(pdev);
error:
	dev_err(dev, "probe failed: %d\n", err);
	return err;
}

static int __exit slvsec_remove(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct slvsec *slvsec = (struct slvsec *)pdata->private_data;

	tegra_camera_device_unregister(slvsec);
	slvsec_remove_debugfs(slvsec);

	return 0;
}

static const struct of_device_id tegra_slvsec_of_match[] = {
	{
		.compatible = "nvidia,tegra-slvs-ec",
		.data = &t19_slvsec_info,
	},
	{ },
};

static struct platform_driver slvsec_driver = {
	.probe = slvsec_probe,
	.remove = __exit_p(slvsec_remove),
	.driver = {
		.owner = THIS_MODULE,
		.name = "tegra-slvs-ec",
#ifdef CONFIG_OF
		.of_match_table = tegra_slvsec_of_match,
#endif
#ifdef CONFIG_PM
		.pm = &nvhost_module_pm_ops,
#endif
	},
};

module_platform_driver(slvsec_driver);

/* === Debugfs ========================================================== */

static const struct debugfs_reg32 slvsec_core_regs[] = {
	{ .name = "slvsec_id", 0x80 },
	{ .name = "intr_status", 0x84 },
	{ .name = "dbg_cnt0_ctrl", 0x8c },
	{ .name = "dbg_counter0", 0x90 },
	{ .name = "dbg_cnt1_ctrl", 0x94 },
	{ .name = "dbg_counter1", 0x98 },
	{ .name = "dbg_cnt2_ctrl", 0x9C },
	{ .name = "dbg_counter2", 0xA0 },
	{ .name = "dbg_cnt3_ctrl", 0xA4 },
	{ .name = "dbg_counter3", 0xA8 },
};

static const struct debugfs_reg32 slvsec_core_strm_regs[] = {
	{ .name = "ctrl", 0x0 },
	{ .name = "rst_ctrl", 0x4 },
	{ .name = "clk_ctrl", 0x8 },
	{ .name = "ch0_cfg", 0x0c },
	{ .name = "ch1_cfg", 0x10 },
	{ .name = "ch0_embd_cfg", 0x14 },
	{ .name = "ch1_embd_cfg", 0x18 },
	{ .name = "timeout_ctrl", 0x1C },
	{ .name = "ph_rx_crc0", 0x20 },
	{ .name = "ph_rx_crc1", 0x24 },
	{ .name = "ph_rx_crc2", 0x28 },
	{ .name = "pd_rx_crc", 0x2c },
	{ .name = "pd_cal_crc", 0x30 },
	{ .name = "vi_err_mask_ch0", 0x34 },
	{ .name = "vi_err_mask_ch1", 0x38 },
	{ .name = "intr_mask_ch0", 0x3c },
	{ .name = "intr_mask_ch1", 0x40 },
	{ .name = "intr_mask", 0x44 },
	{ .name = "intr_status_ch0", 0x48 },
	{ .name = "intr_status_ch1", 0x4c },
	{ .name = "intr_status", 0x50 },
};

static const struct debugfs_reg32 slvsec_cil_regs[] = {
	{ .name = "ctrl", 0x0 },
	{ .name = "lane_swizzle", 0x4 },
	{ .name = "uphy_ctrl0", 0x8 },
	{ .name = "uphy_ctrl1", 0xC },
	{ .name = "uphy_ctrl2", 0x10 },
	{ .name = "timeout_ctrl", 0x14 },
	{ .name = "padctrl_rst_ctrl", 0x18 },
};

static const struct debugfs_reg32 slvsec_vi_syncgen_regs[] = {
	{ .name = "hclk_div", 0x0 },
	{ .name = "hclk_div_fmt", 0x4 },
	{ .name = "xhs", 0x8 },
	{ .name = "xvs", 0xC },
	{ .name = "xvs_to_xhs_delay", 0x10 },
	{ .name = "int_status", 0x14 },
	{ .name = "int_mask", 0x18 },
	{ .name = "xhs_timer", 0x1c },
	{ .name = "control", 0x20 },
	{ .name = "command", 0x24 },
	{ .name = "status", 0x28 },
	{ .name = "scan_status", 0x2c },
	{ .name = "force_xvs", 0x30 },
};

static int slvsec_sw_debug_intr_show(void *data, u64 *val)
{
	struct platform_device *pdev = data;

	*val = host1x_readl(pdev, SLVSEC_CORE_SW_DEBUG_INTR_STATUS);

	return 0;
}

static int slvsec_sw_debug_intr_store(void *data, u64 val)
{
	struct platform_device *pdev = data;

	if (val > (u32)~U32_C(0))
		return -EINVAL;

	host1x_writel(pdev, SLVSEC_CORE_SW_DEBUG_INTR_TRIG, (u32)val);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(slvsec_sw_debug_intr_fops,
			slvsec_sw_debug_intr_show,
			slvsec_sw_debug_intr_store,
			"%llu\n");

static int slvsec_core_debug_cnt_event_show(void *data, u64 *event)
{
	struct slvsec_debug_counter *counter = data;
	struct slvsec *slvsec = container_of(data, struct slvsec,
					debug.counters[counter->number]);
	struct platform_device *pdev = slvsec->pdev;
	u32 reg = SLVSEC_CORE_DBG_CNT_CTRL +
		SLVSEC_CORE_DBG_CNT_SIZE * counter->number;
	u32 val = host1x_readl(pdev, reg);

	val >>= SLVSEC_CORE_DBG_CNT_CTRL_EVENT_SHIFT;
	val &= SLVSEC_CORE_DBG_CNT_CTRL_EVENT_MASK;

	*event = val;

	return 0;
}

static int slvsec_core_debug_cnt_event_store(void *data, u64 event)
{
	struct slvsec_debug_counter *counter = data;
	struct slvsec *slvsec = container_of(data, struct slvsec,
					debug.counters[counter->number]);
	struct platform_device *pdev = slvsec->pdev;
	u32 control_reg = SLVSEC_CORE_DBG_CNT_CTRL +
		SLVSEC_CORE_DBG_CNT_SIZE * counter->number;
	u32 counter_reg = SLVSEC_CORE_DBG_CNT_VALUE +
		SLVSEC_CORE_DBG_CNT_SIZE * counter->number;
	u32 val = host1x_readl(pdev, control_reg);

	if (event > SLVSEC_CORE_DBG_CNT_CTRL_EVENT_MASK)
		return -EINVAL;

	counter->event = event;
	val &= ~SLVSEC_CORE_DBG_CNT_CTRL_EVENT_MASK;
	val |= event << SLVSEC_CORE_DBG_CNT_CTRL_EVENT_SHIFT;

	host1x_writel(pdev, control_reg, val);
	/* Clear counter */
	host1x_writel(pdev, counter_reg, 0);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(slvsec_core_debug_cnt_event_fops,
			slvsec_core_debug_cnt_event_show,
			slvsec_core_debug_cnt_event_store,
			"%llu\n");

static int slvsec_core_debug_cnt_strm_sel_show(void *data, u64 *strm_sel)
{
	struct slvsec_debug_counter *counter = data;
	struct slvsec *slvsec = container_of(data, struct slvsec,
					debug.counters[counter->number]);
	struct platform_device *pdev = slvsec->pdev;
	u32 reg = SLVSEC_CORE_DBG_CNT_CTRL +
		SLVSEC_CORE_DBG_CNT_SIZE * counter->number;
	u32 val = host1x_readl(pdev, reg);

	val >>= SLVSEC_CORE_DBG_CNT_CTRL_STRM_SEL_SHIFT;
	val &= SLVSEC_CORE_DBG_CNT_CTRL_STRM_SEL_MASK;

	*strm_sel = val;

	return 0;
}

static int slvsec_core_debug_cnt_strm_sel_store(void *data, u64 strm_sel)
{
	struct slvsec_debug_counter *counter = data;
	struct slvsec *slvsec = container_of(data, struct slvsec,
					debug.counters[counter->number]);
	struct platform_device *pdev = slvsec->pdev;
	u32 control_reg = SLVSEC_CORE_DBG_CNT_CTRL +
		SLVSEC_CORE_DBG_CNT_SIZE * counter->number;
	u32 counter_reg = SLVSEC_CORE_DBG_CNT_VALUE +
		SLVSEC_CORE_DBG_CNT_SIZE * counter->number;
	u32 val = host1x_readl(pdev, control_reg);

	if (strm_sel > SLVSEC_CORE_DBG_CNT_CTRL_STRM_SEL_MASK)
		return -EINVAL;

	counter->stream = strm_sel;
	val &= ~SLVSEC_CORE_DBG_CNT_CTRL_STRM_SEL_MASK;
	val |= strm_sel << SLVSEC_CORE_DBG_CNT_CTRL_STRM_SEL_SHIFT;

	host1x_writel(pdev, control_reg, val);
	/* Clear counter */
	host1x_writel(pdev, counter_reg, 0);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(slvsec_core_debug_cnt_strm_sel_fops,
			slvsec_core_debug_cnt_strm_sel_show,
			slvsec_core_debug_cnt_strm_sel_store,
			"%llu\n");

static int slvsec_core_debug_cnt_value_show(void *data, u64 *value)
{
	struct slvsec_debug_counter *counter = data;
	struct slvsec *slvsec = container_of(data, struct slvsec,
					debug.counters[counter->number]);
	struct platform_device *pdev = slvsec->pdev;
	u32 reg = SLVSEC_CORE_DBG_CNT_VALUE +
		SLVSEC_CORE_DBG_CNT_SIZE * counter->number;

	*value = host1x_readl(pdev, reg);

	return 0;
}

static int slvsec_core_debug_cnt_value_store(void *data, u64 value)
{
	struct slvsec_debug_counter *counter = data;
	struct slvsec *slvsec = container_of(data, struct slvsec,
					debug.counters[counter->number]);
	struct platform_device *pdev = slvsec->pdev;
	u32 counter_reg = SLVSEC_CORE_DBG_CNT_VALUE +
		SLVSEC_CORE_DBG_CNT_SIZE * counter->number;

	if (value > ~U32_C(0))
		return -EINVAL;

	host1x_writel(pdev, counter_reg, (u32)value);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(slvsec_core_debug_cnt_value_fops,
			slvsec_core_debug_cnt_value_show,
			slvsec_core_debug_cnt_value_store,
			"%llu\n");

static int slvsec_init_debugfs(struct slvsec *slvsec)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(slvsec->pdev);
	struct dentry *dir = pdata->debugfs;
	struct slvsec_debug *debug = &slvsec->debug;
	int i;

	/* Trigger or clear SW interrupts */
	debugfs_create_file("sw-intr", S_IRUGO | S_IWUSR, dir, slvsec->pdev,
			&slvsec_sw_debug_intr_fops);

	for (i = 0; i < ARRAY_SIZE(debug->counters); i++) {
		struct slvsec_debug_counter *counter = &debug->counters[i];
		char name[16];

		snprintf(name, sizeof(name), "counter%u", i);
		counter->number = i;
		counter->dir = debugfs_create_dir(name, pdata->debugfs);
		debugfs_create_file("event",
				S_IRUGO | S_IWUSR,
				counter->dir, counter,
				&slvsec_core_debug_cnt_event_fops);
		debugfs_create_file("stream",
				S_IRUGO | S_IWUSR,
				counter->dir, counter,
				&slvsec_core_debug_cnt_strm_sel_fops);
		debugfs_create_file("value",
				S_IRUGO | S_IWUSR,
				counter->dir, counter,
				&slvsec_core_debug_cnt_value_fops);
	}

	debug->counters[0].event = 0; /* packet num */
	debug->counters[1].event = 4; /* sof */
	debug->counters[2].event = 5; /* eof */
	debug->counters[3].event = 6; /* valid lines */

	debug->core.base = pdata->aperture[0];
	debug->core.regs = slvsec_core_regs;
	debug->core.nregs = ARRAY_SIZE(slvsec_core_regs);
	debugfs_create_regset32("core", S_IRUGO, dir, &debug->core);

	debug->core_stream0.base = pdata->aperture[0] + 0x10000;
	debug->core_stream0.regs = slvsec_core_strm_regs;
	debug->core_stream0.nregs = ARRAY_SIZE(slvsec_core_strm_regs);
	debugfs_create_regset32("stream0", S_IRUGO, dir, &debug->core_stream0);

	debug->core_stream1.base = pdata->aperture[0] + 0x20000;
	debug->core_stream1.regs = slvsec_core_strm_regs;
	debug->core_stream1.nregs = ARRAY_SIZE(slvsec_core_strm_regs);
	debugfs_create_regset32("stream1", S_IRUGO, dir, &debug->core_stream1);

	debug->cil.base = pdata->aperture[0] + 0x30000;
	debug->cil.regs = slvsec_cil_regs;
	debug->cil.nregs = ARRAY_SIZE(slvsec_cil_regs);
	debugfs_create_regset32("cil", S_IRUGO, dir, &debug->cil);

	debug->vi_syncgen0.base = pdata->aperture[1] + 0x4800;
	debug->vi_syncgen0.regs = slvsec_vi_syncgen_regs;
	debug->vi_syncgen0.nregs = ARRAY_SIZE(slvsec_vi_syncgen_regs);
	debugfs_create_regset32("fw-syncgen0", S_IRUGO, dir,
				&debug->vi_syncgen0);

	debug->vi_syncgen1.base = pdata->aperture[1] + 0x4c00;
	debug->vi_syncgen1.regs = slvsec_vi_syncgen_regs;
	debug->vi_syncgen1.nregs = ARRAY_SIZE(slvsec_vi_syncgen_regs);
	debugfs_create_regset32("fw-syncgen1", S_IRUGO, dir,
				&debug->vi_syncgen1);

	debug->vi_syncgen2.base = pdata->aperture[1] + 0x5000;
	debug->vi_syncgen2.regs = slvsec_vi_syncgen_regs;
	debug->vi_syncgen2.nregs = ARRAY_SIZE(slvsec_vi_syncgen_regs);
	debugfs_create_regset32("fw-syncgen2", S_IRUGO, dir,
				&debug->vi_syncgen2);

	return 0;
}

static void slvsec_remove_debugfs(struct slvsec *slvsec)
{
}
