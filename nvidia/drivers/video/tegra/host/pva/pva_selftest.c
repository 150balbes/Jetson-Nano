/*
 * PVA uCode Self Test
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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


#include <linux/kernel.h>
#include <linux/nvhost.h>
#include <linux/delay.h>
#include <linux/iommu.h>
#include <linux/dma-mapping.h>

#include "dev.h"
#include "pva.h"

/* Defines for self test-mode in ucode */
#define PVA_MBOX_VAL_TESTS_DONE		0x57800000
#define PVA_SELF_TESTMODE_START_ADDR	0x90000000
#define PVA_SELF_TESTMODE_ADDR_SIZE	0x00800000

int pva_run_ucode_selftest(struct platform_device *pdev)
{
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct pva *pva = pdata->private_data;
	int err = 0;
	u32 reg_status;
	u32 ucode_mode;
	void *selftest_cpuaddr;
	dma_addr_t base_iova = PVA_SELF_TESTMODE_START_ADDR;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 9, 0)
	struct dma_attrs attrs;

	init_dma_attrs(&attrs);
#else
	unsigned long attrs = 0;
#endif

	/* Map static memory for self test mode */
	nvhost_dbg_info("uCode TESTMODE Enabled");
	dma_set_attr(DMA_ATTR_SKIP_CPU_SYNC, __DMA_ATTR(attrs));
	dma_set_attr(DMA_ATTR_SKIP_IOVA_GAP, __DMA_ATTR(attrs));

	selftest_cpuaddr = dma_alloc_at_attrs(&pdev->dev,
			PVA_SELF_TESTMODE_ADDR_SIZE, &base_iova,
			GFP_KERNEL|__GFP_ZERO, __DMA_ATTR(attrs));

	if (!selftest_cpuaddr) {
		dev_warn(&pdev->dev, "Failed to get Selftest Static memory\n");
		err = -ENOMEM;
		goto err_selftest;
	}

	pva->mailbox_status = PVA_MBOX_STATUS_WFI;
	host1x_writel(pdev, hsp_ss0_set_r(), PVA_TEST_RUN);

	/* Wait till we get a AISR_ABORT interrupt */
	err = pva_mailbox_wait_event(pva, 60000);
	if (err)
		goto wait_timeout;

	ucode_mode = host1x_readl(pdev, hsp_ss0_state_r());

	/*Check whether ucode halted */
	if ((ucode_mode & PVA_HALTED) == 0) {
		nvhost_dbg_info("uCode SELFTEST Failed to Halt");
		err = -EINVAL;
		goto err_selftest;
	}

	reg_status = pva_read_mailbox(pdev, PVA_MBOX_ISR);

	/* check test passed bit set and test status done*/
	if ((ucode_mode & PVA_TESTS_PASSED) &&
		(reg_status == PVA_MBOX_VAL_TESTS_DONE))
		nvhost_dbg_info("uCode SELFTEST Passed");
	else if (ucode_mode & PVA_TESTS_FAILED)
		nvhost_dbg_info("uCode SELFTEST Failed");
	else
		nvhost_dbg_info("uCode SELFTEST UnKnown State");

	/* Get CCQ8 register value */
	reg_status = host1x_readl(pdev, cfg_ccq_status8_r());
	nvhost_dbg_info("Major 0x%x, Minor 0x%x, Flags 0x%x, Trace Sequence 0x%x \n",
			(reg_status & 0xFF000000) >> 24,
			(reg_status & 0x00FF0000) >> 16,
			(reg_status & 0xFF00) >> 8,
			(reg_status & 0xFF));

wait_timeout:
err_selftest:
	if (selftest_cpuaddr)
		dma_free_attrs(&pdev->dev,
			PVA_SELF_TESTMODE_ADDR_SIZE, selftest_cpuaddr,
			PVA_SELF_TESTMODE_START_ADDR, __DMA_ATTR(attrs));

	return err;

}
