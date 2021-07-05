/*
 * dev.h
 *
 * A header file for Host driver for ADSP and APE
 *
 * Copyright (C) 2014-2019, NVIDIA Corporation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __TEGRA_NVADSP_DEV_H
#define __TEGRA_NVADSP_DEV_H

#include <linux/tegra_nvadsp.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/debugfs.h>

#include <linux/platform/tegra/emc_bwmgr.h>

#include "hwmailbox.h"
#include "amc.h"

/*
 * Note: These enums should be aligned to the regs mentioned in the
 * device tree
*/
enum {
	AMC,
	AMISC,
	ABRIDGE,
	UNIT_FPGA_RST,
	AHSP,
	APE_MAX_REG
};

enum {
	ADSP_DRAM1,
	ADSP_DRAM2,
	ADSP_MAX_DRAM_MAP
};

/*
 * Note: These enums should be aligned to the adsp_mem node mentioned in the
 * device tree
*/
enum adsp_mem_dt {
	ADSP_OS_ADDR,
	ADSP_OS_SIZE,
	ADSP_APP_ADDR,
	ADSP_APP_SIZE,
	ARAM_ALIAS_0_ADDR,
	ARAM_ALIAS_0_SIZE,
	ACSR_ADDR, /* ACSR: ADSP CPU SHARED REGION */
	ACSR_SIZE,
	ADSP_MEM_END,
};

enum adsp_evp_dt {
	ADSP_EVP_BASE,
	ADSP_EVP_SIZE,
	ADSP_EVP_END,
};

enum adsp_unit_fpga_reset {
	ADSP_ASSERT,
	ADSP_DEASSERT,
	ADSP_UNIT_FPGA_RESET_END,
};

#define AMISC_REGS	0x2000

#define AMISC_ADSP_L2_REGFILEBASE	0x10
#define AMISC_SHRD_SMP_STA		0x14
#define AMISC_SEM_REG_START		0x1c
#define AMISC_SEM_REG_END		0x44
#define AMISC_TSC			0x48
#define AMISC_ACTMON_AVG_CNT		0x81c

#define AMISC_REG_START_OFFSET		0x0
#define AMISC_REG_MBOX_OFFSET		0x64
#define ADSP_ACTMON_REG_START_OFFSET	0x800
#define ADSP_ACTMON_REG_END_OFFSET	0x828

enum nvadsp_virqs {
	MBOX_SEND_VIRQ,
	MBOX_RECV_VIRQ,
	WDT_VIRQ,
	WFI_VIRQ,
	AMC_ERR_VIRQ,
	ACTMON_VIRQ,
	NVADSP_VIRQ_MAX,
};

struct nvadsp_pm_state {
	u32 aram[AMC_ARAM_WSIZE];
	uint32_t amc_regs[AMC_REGS];
	uint32_t amisc_regs[AMISC_REGS];
	u32 *evp;
	void *evp_ptr;
};

struct nvadsp_hwmb {
	u32 reg_idx;
	u32 hwmbox0_reg;
	u32 hwmbox1_reg;
	u32 hwmbox2_reg;
	u32 hwmbox3_reg;
	u32 hwmbox4_reg;
	u32 hwmbox5_reg;
	u32 hwmbox6_reg;
	u32 hwmbox7_reg;
};


typedef int (*reset_init) (struct platform_device *pdev);
typedef int (*os_init) (struct platform_device *pdev);
#ifdef CONFIG_PM
typedef int (*pm_init) (struct platform_device *pdev);
#endif

struct nvadsp_chipdata {
	struct nvadsp_hwmb	hwmb;
	u32			adsp_state_hwmbox;
	u32			adsp_thread_hwmbox;
	u32			adsp_irq_hwmbox;
	reset_init		reset_init;
	os_init			os_init;
#ifdef CONFIG_PM
	pm_init			pm_init;
#endif
	int			wdt_irq;
	int			start_irq;
	int			end_irq;
};

struct nvadsp_drv_data {
	void __iomem **base_regs;
	void __iomem **base_regs_saved;
	struct platform_device *pdev;
	struct resource *dram_region[ADSP_MAX_DRAM_MAP];
	struct hwmbox_queue hwmbox_send_queue;

	struct nvadsp_mbox **mboxes;
	unsigned long *mbox_ids;
	spinlock_t mbox_lock;

#ifdef CONFIG_DEBUG_FS
	struct dentry *adsp_debugfs_root;
#endif
	struct clk *ape_clk;
	struct clk *apb2ape_clk;
	struct clk *adsp_clk;
	struct clk *aclk_clk;
	struct clk *adsp_cpu_abus_clk;
	struct clk *adsp_neon_clk;
	struct clk *uartape_clk;
	struct clk *ahub_clk;
	unsigned long adsp_freq; /* in KHz*/
	unsigned long adsp_freq_hz; /* in Hz*/
	unsigned long ape_freq; /* in KHz*/
	unsigned long ape_emc_freq; /* in KHz*/

	int (*runtime_suspend)(struct device *dev);
	int (*runtime_resume)(struct device *dev);
	int (*runtime_idle)(struct device *dev);
	int (*assert_adsp)(struct nvadsp_drv_data *drv_data);
	int (*deassert_adsp)(struct nvadsp_drv_data *drv_data);
	struct reset_control *adspall_rst;

	struct nvadsp_pm_state state;
	bool adsp_os_running;
	bool adsp_os_suspended;
	bool adsp_os_secload;

	void *shared_adsp_os_data;

#ifdef CONFIG_TEGRA_ADSP_DFS
	bool dfs_initialized;
#endif

#ifdef CONFIG_TEGRA_ADSP_ACTMON
	bool actmon_initialized;
#endif

#ifdef CONFIG_TEGRA_ADSP_CPUSTAT
	bool cpustat_initialized;
#endif

#if defined(CONFIG_TEGRA_ADSP_FILEIO)
	bool adspff_init;
#endif

#ifdef CONFIG_TEGRA_ADSP_LPTHREAD
	bool lpthread_initialized;
#endif

	wait_queue_head_t adsp_health_waitq;
	bool adsp_crashed;

	u32 adsp_mem[ADSP_MEM_END];
	bool adsp_unit_fpga;
	u32 unit_fpga_reset[ADSP_UNIT_FPGA_RESET_END];
	int agic_irqs[NVADSP_VIRQ_MAX];

	struct tegra_bwmgr_client *bwmgr;
	u32 evp_base[ADSP_EVP_END];

	const struct nvadsp_chipdata *chip_data;
};

#define ADSP_CONFIG	0x04
#define MAXCLKLATENCY	(3 << 29)
#define UART_BAUD_RATE	9600

status_t nvadsp_mbox_init(struct platform_device *pdev);

int nvadsp_setup_amc_interrupts(struct platform_device *pdev);
void nvadsp_free_amc_interrupts(struct platform_device *pdev);

#ifdef CONFIG_TEGRA_ADSP_DFS
void adsp_cpu_set_rate(unsigned long freq);
int adsp_dfs_core_init(struct platform_device *pdev);
int adsp_dfs_core_exit(struct platform_device *pdev);
u32 adsp_to_emc_freq(u32 adspfreq);
#endif

#ifdef CONFIG_TEGRA_ADSP_ACTMON
int ape_actmon_probe(struct platform_device *pdev);
#endif

#ifdef CONFIG_TEGRA_ADSP_CPUSTAT
int adsp_cpustat_init(struct platform_device *pdev);
int adsp_cpustat_exit(struct platform_device *pdev);
#endif

#if defined(CONFIG_TEGRA_ADSP_FILEIO)
int adspff_init(struct platform_device *pdev);
void adspff_exit(void);
#endif

#ifdef CONFIG_TEGRA_EMC_APE_DFS
status_t emc_dfs_init(struct platform_device *pdev);
void emc_dfs_exit(void);
#endif

#ifdef CONFIG_PM
static inline int __init nvadsp_pm_init(struct platform_device *pdev)
{
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(pdev);

	if (drv_data->chip_data->pm_init)
		return drv_data->chip_data->pm_init(pdev);

	return -EINVAL;
}
#endif
static inline int __init nvadsp_reset_init(struct platform_device *pdev)
{
	struct nvadsp_drv_data *drv_data = platform_get_drvdata(pdev);

	if (drv_data->chip_data->reset_init)
		return drv_data->chip_data->reset_init(pdev);

	return -EINVAL;
}

#ifdef CONFIG_TEGRA_ADSP_LPTHREAD
int adsp_lpthread_init(bool is_adsp_suspended);
int adsp_lpthread_resume(void);
int adsp_lpthread_pause(void);
int adsp_lpthread_uninit(void);
int adsp_lpthread_get_state(void);

int adsp_lpthread_entry(struct platform_device *pdev);
int adsp_lpthread_exit(struct platform_device *pdev);
int adsp_lpthread_set_suspend(bool is_suspended);
#endif

#endif /* __TEGRA_NVADSP_DEV_H */
