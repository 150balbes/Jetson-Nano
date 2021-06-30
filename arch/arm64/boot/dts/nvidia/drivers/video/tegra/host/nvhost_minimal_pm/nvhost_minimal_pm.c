/*
 * Nvhost Minimal Powermanangement
 *
 * Copyright (c) 2017, NVIDIA Corporation.  All rights reserved.
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

#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/of.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/io.h>

#include <soc/tegra/chip-id.h>

struct register_info {
	u32 index;
	u32 value;
};

#define CLK_RST_CONTROLLER		0x20000000
#define HOST1X_THOST			0x13e00000

#define CLK_OUT_ENB_HOST1X_SET_0	(CLK_RST_CONTROLLER + 0x001e1004)
#define RST_DEV_HOST1X_CLR_0		(CLK_RST_CONTROLLER + 0x001e0008)
#define RST_DEV_CV_HOST1X_CLR_0		(CLK_RST_CONTROLLER + 0x01190014)
#define COMMON_CV_CLUSTER_CLAMP_0	(HOST1X_THOST + 0xc000)
#define COMMON_MOD_CLAMP_E_CLR_0	(HOST1X_THOST + 0xc00c)
static struct register_info host1x_cv_reg[] = {
		{CLK_OUT_ENB_HOST1X_SET_0, 1},
		{RST_DEV_HOST1X_CLR_0, 1},
		{RST_DEV_CV_HOST1X_CLR_0, 1},
		{COMMON_CV_CLUSTER_CLAMP_0, 0},
		{RST_DEV_CV_HOST1X_CLR_0, 0xffff}
	};

#define RST_DEV_PVA0_CLR_0		(CLK_RST_CONTROLLER + 0x0115006c)
#define RST_DEV_PVA1_CLR_0		(CLK_RST_CONTROLLER + 0x0116006c)
#define RST_DEV_PVA0_CPU_AXI_CLR_0	(CLK_RST_CONTROLLER + 0x011500a8)
#define RST_DEV_PVA1_CPU_AXI_CLR_0	(CLK_RST_CONTROLLER + 0x011600a8)
#define RST_DEV_PVA0_DMA0_CLR_0		(CLK_RST_CONTROLLER + 0x01150078)
#define RST_DEV_PVA1_DMA0_CLR_0		(CLK_RST_CONTROLLER + 0x01160078)
#define RST_DEV_PVA0_DMA1_CLR_0		(CLK_RST_CONTROLLER + 0x01150084)
#define RST_DEV_PVA1_DMA1_CLR_0		(CLK_RST_CONTROLLER + 0x01160084)
#define RST_DEV_PVA0_PROCCFG_CLR_0	(CLK_RST_CONTROLLER + 0x01150090)
#define RST_DEV_PVA1_PROCCFG_CLR_0	(CLK_RST_CONTROLLER + 0x01160090)
#define RST_DEV_PVA0_PM_CLR_0		(CLK_RST_CONTROLLER + 0x0115009c)
#define RST_DEV_PVA1_PM_CLR_0		(CLK_RST_CONTROLLER + 0x0116009c)
#define RST_DEV_PVA0_VPS0_CLR_0		(CLK_RST_CONTROLLER + 0x011500f0)
#define RST_DEV_PVA1_VPS0_CLR_0		(CLK_RST_CONTROLLER + 0x011600f0)
#define RST_DEV_PVA0_VPS1_CLR_0		(CLK_RST_CONTROLLER + 0x011500fc)
#define RST_DEV_PVA1_VPS1_CLR_0		(CLK_RST_CONTROLLER + 0x011600fc)
#define RST_DEV_PVA0_CFG_CLR_0		(CLK_RST_CONTROLLER + 0x01150108)
#define RST_DEV_PVA1_CFG_CLR_0		(CLK_RST_CONTROLLER + 0x01160108)
#define RST_DEV_PVA0_H1X_CLR_0		(CLK_RST_CONTROLLER + 0x01150114)
#define RST_DEV_PVA1_H1X_CLR_0		(CLK_RST_CONTROLLER + 0x01160114)
#define RST_DEV_PVA0_SEC_CLR_0		(CLK_RST_CONTROLLER + 0x01150120)
#define RST_DEV_PVA1_SEC_CLR_0		(CLK_RST_CONTROLLER + 0x01160120)
#define RST_DEV_PVA0_RAM1C_CLR_0	(CLK_RST_CONTROLLER + 0x0115012c)
#define RST_DEV_PVA1_RAM1C_CLR_0	(CLK_RST_CONTROLLER + 0x0116012c)
#define RST_DEV_PVA0_ACTMON_CLR_0	(CLK_RST_CONTROLLER + 0x01150138)
#define RST_DEV_PVA1_ACTMON_CLR_0	(CLK_RST_CONTROLLER + 0x01160138)
#define RST_DEV_PVA0_TKE_CLR_0		(CLK_RST_CONTROLLER + 0x01150144)
#define RST_DEV_PVA1_TKE_CLR_0		(CLK_RST_CONTROLLER + 0x01160144)
#define RST_DEV_PVA0_TSCTN_CLR_0	(CLK_RST_CONTROLLER + 0x01150150)
#define RST_DEV_PVA1_TSCTN_CLR_0	(CLK_RST_CONTROLLER + 0x01160150)
#define RST_DEV_PVA0_GTE_CLR_0		(CLK_RST_CONTROLLER + 0x0115015c)
#define RST_DEV_PVA1_GTE_CLR_0		(CLK_RST_CONTROLLER + 0x0116015c)

static struct register_info pva_reg_reset[] = {
		/* pva resets */
		{RST_DEV_PVA0_CLR_0, 0x7},
		{RST_DEV_PVA1_CLR_0, 0x7},
		{RST_DEV_PVA0_CPU_AXI_CLR_0, 0xb},
		{RST_DEV_PVA1_CPU_AXI_CLR_0, 0xb},
		{RST_DEV_PVA0_DMA0_CLR_0, 0x1},
		{RST_DEV_PVA1_DMA0_CLR_0, 0x1},
		{RST_DEV_PVA0_DMA1_CLR_0, 0x1},
		{RST_DEV_PVA1_DMA1_CLR_0, 0x1},
		{RST_DEV_PVA0_PROCCFG_CLR_0, 0x1},
		{RST_DEV_PVA1_PROCCFG_CLR_0, 0x1},
		{RST_DEV_PVA0_PM_CLR_0, 0x1},
		{RST_DEV_PVA1_PM_CLR_0, 0x1},
		{RST_DEV_PVA0_VPS0_CLR_0, 0x3},
		{RST_DEV_PVA1_VPS0_CLR_0, 0x3},
		{RST_DEV_PVA0_VPS1_CLR_0, 0x3},
		{RST_DEV_PVA1_VPS1_CLR_0, 0x3},
		{RST_DEV_PVA0_CFG_CLR_0, 0x1},
		{RST_DEV_PVA1_CFG_CLR_0, 0x1},
		{RST_DEV_PVA0_H1X_CLR_0, 0x1},
		{RST_DEV_PVA1_H1X_CLR_0, 0x1},
		{RST_DEV_PVA0_SEC_CLR_0, 0x1},
		{RST_DEV_PVA1_SEC_CLR_0, 0x1},
		{RST_DEV_PVA0_RAM1C_CLR_0, 0x1},
		{RST_DEV_PVA1_RAM1C_CLR_0, 0x1},
		{RST_DEV_PVA0_ACTMON_CLR_0, 0x1},
		{RST_DEV_PVA1_ACTMON_CLR_0, 0x1},
		{RST_DEV_PVA0_TKE_CLR_0, 0x1},
		{RST_DEV_PVA1_TKE_CLR_0, 0x1},
		{RST_DEV_PVA0_TSCTN_CLR_0, 0x1},
		{RST_DEV_PVA1_TSCTN_CLR_0, 0x1},
		{RST_DEV_PVA0_GTE_CLR_0, 0x1},
		{RST_DEV_PVA1_GTE_CLR_0, 0x1},
	};

#define CLK_OUT_ENB_PVA0_CPU_AXI_SET_0	(CLK_RST_CONTROLLER + 0x01150008)
#define CLK_OUT_ENB_PVA1_CPU_AXI_SET_0	(CLK_RST_CONTROLLER + 0x01160008)
#define CLK_OUT_ENB_PVA0_VPS_SET_0	(CLK_RST_CONTROLLER + 0x011500b4)
#define CLK_OUT_ENB_PVA1_VPS_SET_0	(CLK_RST_CONTROLLER + 0x011600b4)
#define CLK_OUT_ENB_PVA0_VPS0_SET_0	(CLK_RST_CONTROLLER + 0x011500c0)
#define CLK_OUT_ENB_PVA1_VPS0_SET_0	(CLK_RST_CONTROLLER + 0x011600c0)
#define CLK_OUT_ENB_PVA0_VPS1_SET_0	(CLK_RST_CONTROLLER + 0x011500cc)
#define CLK_OUT_ENB_PVA1_VPS1_SET_0	(CLK_RST_CONTROLLER + 0x011600cc)
#define CLK_OUT_ENB_PVA0_CPU_SET_0	(CLK_RST_CONTROLLER + 0x01150030)
#define CLK_OUT_ENB_PVA1_CPU_SET_0	(CLK_RST_CONTROLLER + 0x01160030)
#define CLK_OUT_ENB_PVA0_APB_SET_0	(CLK_RST_CONTROLLER + 0x0115003c)
#define CLK_OUT_ENB_PVA1_APB_SET_0	(CLK_RST_CONTROLLER + 0x0116003c)
#define CLK_OUT_ENB_PVA0_DMA0_SET_0	(CLK_RST_CONTROLLER + 0x01150018)
#define CLK_OUT_ENB_PVA1_DMA0_SET_0	(CLK_RST_CONTROLLER + 0x01160018)
#define CLK_OUT_ENB_PVA0_DMA1_SET_0	(CLK_RST_CONTROLLER + 0x01150024)
#define CLK_OUT_ENB_PVA1_DMA1_SET_0	(CLK_RST_CONTROLLER + 0x01160024)

static struct register_info pva_reg_clock[] = {
		/* pva clocks */
		{CLK_OUT_ENB_PVA0_CPU_AXI_SET_0, 0x1},
		{CLK_OUT_ENB_PVA1_CPU_AXI_SET_0, 0x1},
		{CLK_OUT_ENB_PVA0_VPS_SET_0, 0x1},
		{CLK_OUT_ENB_PVA1_VPS_SET_0, 0x1},
		{CLK_OUT_ENB_PVA0_VPS0_SET_0, 0x1},
		{CLK_OUT_ENB_PVA1_VPS0_SET_0, 0x1},
		{CLK_OUT_ENB_PVA0_VPS1_SET_0, 0x1},
		{CLK_OUT_ENB_PVA1_VPS1_SET_0, 0x1},
		{CLK_OUT_ENB_PVA0_CPU_SET_0, 0x1},
		{CLK_OUT_ENB_PVA1_CPU_SET_0, 0x1},
		{CLK_OUT_ENB_PVA0_APB_SET_0, 0x1},
		{CLK_OUT_ENB_PVA1_APB_SET_0, 0x1},
		{CLK_OUT_ENB_PVA0_DMA0_SET_0, 0x1},
		{CLK_OUT_ENB_PVA1_DMA0_SET_0, 0x1},
		{CLK_OUT_ENB_PVA0_DMA1_SET_0, 0x1},
		{CLK_OUT_ENB_PVA1_DMA1_SET_0, 0x1},

	};

#define PMC_IMPL_CNTRL_0		0x0c360000
#define PVAA_POWER_GATE_CONTROL_0	(PMC_IMPL_CNTRL_0 + 0x460)
#define INTER_PART_DELAY_EN		(1 << 31)
#define START_DONE			(1 << 8)
#define PVAB_POWER_GATE_CONTROL_0	(PMC_IMPL_CNTRL_0 + 0x46c)
#define PVAA_CLAMP_CONTROL_0		(PMC_IMPL_CNTRL_0 + 0x468)
#define PVAB_CLAMP_CONTROL_0		(PMC_IMPL_CNTRL_0 + 0x474)
#define CV_RG_CNTRL_0			(PMC_IMPL_CNTRL_0 + 0x8b4)

#define MC_CLIENT_HOTRESET_CTRL_1_0	0x02c20970
#define PVA1A_FLUSH_ENABLE		(1 << 30)
#define PVA0A_FLUSH_ENABLE		(1 << 29)

#define MC_CLIENT_HOTRESET_CTRL_2_0	0x02c2097c
#define PVA1C_FLUSH_ENABLE		(1 << 8)
#define PVA1B_FLUSH_ENABLE		(1 << 7)
#define PVA0C_FLUSH_ENABLE		(1 << 6)
#define PVA0B_FLUSH_ENABLE		(1 << 5)

#define DELAY_100MS	100

static void nvhost_minimal_pm_write(u32 reg, u32 val)
{
	writel(val, ioremap(reg, 4));
}

static u32 nvhost_minimal_pm_read(u32 reg)
{
	return readl(ioremap(reg, 4));
}

static void nvhost_pm_init(void)
{
	int i;
	int count = ARRAY_SIZE(host1x_cv_reg);

	for (i = 0; i < count; i++)
		nvhost_minimal_pm_write(host1x_cv_reg[i].index,
				host1x_cv_reg[i].value);
}

static void pva_deassert_reset(void)
{
	int i;
	int count = ARRAY_SIZE(pva_reg_reset);

	for (i = 0; i < count; i++) {
		nvhost_minimal_pm_write(pva_reg_reset[i].index,
					pva_reg_reset[i].value);
		nvhost_minimal_pm_read(pva_reg_reset[i].index);
	}
}

static void pva_clocks_enable(void)
{
	int i;
	int count = ARRAY_SIZE(pva_reg_clock);

	for (i = 0; i < count; i++) {
		nvhost_minimal_pm_write(pva_reg_clock[i].index,
				pva_reg_clock[i].value);
	}
}


static void pva_pm_init(void)
{
	int regvalue = 0;
	/* Unpowergate pva partitions */
	/* PVAA and PVAB POWER_GATE_CONTROL */
	regvalue = (INTER_PART_DELAY_EN | START_DONE);
	nvhost_minimal_pm_write(PVAA_POWER_GATE_CONTROL_0, regvalue);
	nvhost_minimal_pm_write(PVAB_POWER_GATE_CONTROL_0, regvalue);
	mdelay(DELAY_100MS);
	nvhost_minimal_pm_read(PVAA_POWER_GATE_CONTROL_0);
	nvhost_minimal_pm_read(PVAB_POWER_GATE_CONTROL_0);

	/* Enable PVAA and PVAB clocks */
	pva_clocks_enable();

	/* Assert pva resets - not needed as
	 * pva is alreadey in reset mode
	 */

	/* Remove PVAA and PVAB clamps */
	nvhost_minimal_pm_write(PVAA_CLAMP_CONTROL_0, 0);
	nvhost_minimal_pm_write(PVAB_CLAMP_CONTROL_0, 0);
	mdelay(DELAY_100MS);
	nvhost_minimal_pm_read(PVAA_CLAMP_CONTROL_0);
	nvhost_minimal_pm_read(PVAB_CLAMP_CONTROL_0);

	/* Remove the clamp for rail ungating */
	nvhost_minimal_pm_write(CV_RG_CNTRL_0, 0);
	mdelay(DELAY_100MS);

	/* De-assert PVAA and PVAB Resets */
	pva_deassert_reset();

	/* Enable the MC client */
	regvalue = nvhost_minimal_pm_read(MC_CLIENT_HOTRESET_CTRL_1_0);
	regvalue &= ~(PVA1A_FLUSH_ENABLE | PVA0A_FLUSH_ENABLE);
	nvhost_minimal_pm_write(MC_CLIENT_HOTRESET_CTRL_1_0, regvalue);

	regvalue = nvhost_minimal_pm_read(MC_CLIENT_HOTRESET_CTRL_2_0);
	regvalue &= ~(PVA1C_FLUSH_ENABLE | PVA1B_FLUSH_ENABLE |
			PVA0C_FLUSH_ENABLE | PVA0B_FLUSH_ENABLE);
	nvhost_minimal_pm_write(MC_CLIENT_HOTRESET_CTRL_2_0, regvalue);

}

static struct platform_driver nvhost_minimal_pm_dev_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "nvhost_minimal_pm_dev",
	},
};

static int __init nvhost_minimal_pm_dev_init(void)
{
	struct device_node *bpmp_device;
	int err = 0;

	if (!tegra_platform_is_qt()) {
		/* This hack is only intended for VSP */
		return -ENODEV;
	}

	/* Check for bpmp device status in DT */
	bpmp_device = of_find_node_by_name(NULL, "bpmp");
	if (!bpmp_device)
		bpmp_device = of_find_compatible_node(NULL,
						NULL,
						"nvidia,tegra186-bpmp");

	if (bpmp_device && of_device_is_available(bpmp_device)) {
		err = -EFAULT;
		goto err_found_bpmp;
	}

	/* Host1x Initialization */
	nvhost_pm_init();
	/* PVA Initialization */
	pva_pm_init();

	err = platform_driver_register(&nvhost_minimal_pm_dev_driver);
	if (err < 0)
		pr_err("nvhost_minimal_pm failed to register\n");

err_found_bpmp:
	return err;
}

static void __exit nvhost_minimal_pm_dev_exit(void)
{
	platform_driver_unregister(&nvhost_minimal_pm_dev_driver);
}

/* This driver need to be called before pva and
 * dla driver get called.
 */
rootfs_initcall(nvhost_minimal_pm_dev_init);
module_exit(nvhost_minimal_pm_dev_exit);

