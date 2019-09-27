/*
 * Deskew driver
 *
 * Copyright (c) 2014-2019, NVIDIA Corporation.  All rights reserved.
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

#include "deskew.h"

#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/export.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/fs.h>
#include <asm/ioctls.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <uapi/linux/nvhost_nvcsi_ioctl.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include <media/mc_common.h>

#include "dev.h"
#include "bus_client.h"
#include "nvhost_acm.h"
#include "t186/t186.h"
#include "nvcsi.h"
#include "camera/csi/csi4_fops.h"

static struct tegra_csi_device *mc_csi;
static struct mutex deskew_lock;

static int debugfs_deskew_clk_stats_low[NVCSI_PHY_CIL_NUM_LANE];
static int debugfs_deskew_clk_stats_high[NVCSI_PHY_CIL_NUM_LANE];
static int debugfs_deskew_data_stats_low[NVCSI_PHY_CIL_NUM_LANE];
static int debugfs_deskew_data_stats_high[NVCSI_PHY_CIL_NUM_LANE];

static unsigned int enabled_deskew_lanes;
static unsigned int done_deskew_lanes;

static int nvcsi_deskew_apply_helper(unsigned int active_lanes);

static bool is_t19x_or_greater;
// a regmap for address changes between chips
static uint32_t regs[REGS_COUNT];

static const uint32_t t186_regs[REGS_COUNT] = {
0x10090,	//< NVCSI_STREAM_0_ERROR_STATUS2VI_MASK		regs[0]
0x10890,	//< NVCSI_STREAM_1_ERROR_STATUS2VI_MASK		regs[1]
0x1010101,	//< CFG_ERR_STATUS2VI_MASK_ALL			regs[2]
0x10400,	//< NVCSI_PHY_0_CILA_INTR_STATUS		regs[3]
0x10404,	//< NVCSI_PHY_0_CILA_INTR_MASK			regs[4]
0x10c00,	//< NVCSI_PHY_0_CILB_INTR_STATUS		regs[5]
0x10c04,	//< NVCSI_PHY_0_CILB_INTR_MASK			regs[6]
0x18000,	//< NVCSI_PHY_0_NVCSI_CIL_PHY_CTRL_0		regs[7]
0x18,		//< NVCSI_CIL_A_SW_RESET_0_OFFSET		regs[8]
0x2c,		//< NVCSI_CIL_A_CLK_DESKEW_CTRL_0_OFFSET	regs[9]
0x24,		//< NVCSI_CIL_A_DPHY_INADJ_CTRL_0_OFFSET	regs[10]
0x30,		//< NVCSI_CIL_A_DATA_DESKEW_CTRL_0_OFFSET	regs[11]
0x34,		//< NVCSI_CIL_A_DPHY_DESKEW_STATUS_0_OFFSET	regs[12]
0x38,	//< NVCSI_CIL_A_DPHY_DESKEW_DATA_CALIB_STATUS_LOW_0_0_OFFSET	regs[13]
0x3c,	//< NVCSI_CIL_A_DPHY_DESKEW_DATA_CALIB_STATUS_HIGH_0_0_OFFSET	regs[14]
0x48,	//< NVCSI_CIL_A_DPHY_DESKEW_CLK_CALIB_STATUS_LOW_0_0_OFFSET	regs[15]
0x4c,	//< NVCSI_CIL_A_DPHY_DESKEW_CLK_CALIB_STATUS_HIGH_0_0_OFFSET	regs[16]
0x5c,		//< NVCSI_CIL_A_DESKEW_CONTROL_0_OFFSET		regs[17]
0x5c,		//< NVCSI_CIL_A_CONTROL_0_OFFSET		regs[18]
0xf << 20,	//< DESKEW_COMPARE				regs[19]
20,		//< DESKEW_COMPARE_SHIFT			regs[20]
0xf << 16,	//< DESKEW_SETTLE				regs[21]
16,		//< DESKEW_SETTLE_SHIFT				regs[22]
0x3f << 8,	//< CLK_SETTLE					regs[23]
8,		//< CLK_SETTLE_SHIFT0				regs[24]
0x7f << 0,	//< THS_SETTLE0					regs[25]
0x7f << 0,	//< THS_SETTLE1					regs[26]
0,		//< THS_SETTLE0_SHIFT				regs[27]
0,		//< THS_SETTLE1_SHIFT				regs[28]
0x88,		//< NVCSI_CIL_B_DPHY_INADJ_CTRL_0_OFFSET	regs[29]
0x90,		//< NVCSI_CIL_B_CLK_DESKEW_CTRL_0_OFFSET	regs[30]
0x94,		//< NVCSI_CIL_B_DATA_DESKEW_CTRL_0_OFFSET	regs[31]
0x98,		//< NVCSI_CIL_B_DPHY_DESKEW_STATUS_0_OFFSET	regs[32]
0xc0,		//< NVCSI_CIL_B_DESKEW_CONTROL_0_OFFSET		regs[33]
0xc0,		//< NVCSI_CIL_B_CONTROL_0_OFFSET		regs[34]
0x64,		//< NVCSI_CIL_B_OFFSET				regs[35]
};

static const uint32_t t194_regs[REGS_COUNT] = {
0x101e4,	//< NVCSI_STREAM_0_ERROR_STATUS2VI_MASK		regs[0]
0x181e4,	//< NVCSI_STREAM_1_ERROR_STATUS2VI_MASK		regs[1]
0x0ffff,	//< CFG_ERR_STATUS2VI_MASK_ALL			regs[2]
0x10400,	//< NVCSI_PHY_0_CILA_INTR_STATUS		regs[3]
0x10408,	//< NVCSI_PHY_0_CILA_INTR_MASK			regs[4]
0x10800,	//< NVCSI_PHY_0_CILB_INTR_STATUS		regs[5]
0x10808,	//< NVCSI_PHY_0_CILB_INTR_MASK			regs[6]
0x11000,	//< NVCSI_PHY_0_NVCSI_CIL_PHY_CTRL_0		regs[7]
0x24,		//< NVCSI_CIL_A_SW_RESET_0_OFFSET		regs[8]
0x38,		//< NVCSI_CIL_A_CLK_DESKEW_CTRL_0_OFFSET	regs[9]
0x30,		//< NVCSI_CIL_A_DPHY_INADJ_CTRL_0_OFFSET	regs[10]
0x3c,		//< NVCSI_CIL_A_DATA_DESKEW_CTRL_0_OFFSET	regs[11]
0x40,		//< NVCSI_CIL_A_DPHY_DESKEW_STATUS_0_OFFSET	regs[12]
0x44,	//< NVCSI_CIL_A_DPHY_DESKEW_DATA_CALIB_STATUS_LOW_0_0_OFFSET	regs[13]
0x48,	//< NVCSI_CIL_A_DPHY_DESKEW_DATA_CALIB_STATUS_HIGH_0_0_OFFSET	regs[14]
0x54,	//< NVCSI_CIL_A_DPHY_DESKEW_CLK_CALIB_STATUS_LOW_0_0_OFFSET	regs[15]
0x58,	//< NVCSI_CIL_A_DPHY_DESKEW_CLK_CALIB_STATUS_HIGH_0_0_OFFSET	regs[16]
0x6c,		//< NVCSI_CIL_A_DESKEW_CONTROL_0_OFFSET		regs[17]
0x70,		//< NVCSI_CIL_A_CONTROL_0_OFFSET		regs[18]
0xf << 4,	//< DESKEW_COMPARE				regs[19]
4,		//< DESKEW_COMPARE_SHIFT			regs[20]
0xf << 0,	//< DESKEW_SETTLE				regs[21]
0,		//< DESKEW_SETTLE_SHIFT				regs[22]
0x7f << 17,	//< CLK_SETTLE					regs[23]
17,		//< CLK_SETTLE_SHIFT0				regs[24]
0xff << 1,	//< THS_SETTLE0					regs[25]
0xff << 9,	//< THS_SETTLE1					regs[26]
1,		//< THS_SETTLE0_SHIFT				regs[27]
9,		//< THS_SETTLE1_SHIFT				regs[28]
0xbc,		//< NVCSI_CIL_B_DPHY_INADJ_CTRL_0_OFFSET	regs[29]
0xc4,		//< NVCSI_CIL_B_CLK_DESKEW_CTRL_0_OFFSET	regs[30]
0xc8,		//< NVCSI_CIL_B_DATA_DESKEW_CTRL_0_OFFSET	regs[31]
0xcc,		//< NVCSI_CIL_B_DPHY_DESKEW_STATUS_0_OFFSET	regs[32]
0xf8,		//< NVCSI_CIL_B_DESKEW_CONTROL_0_OFFSET		regs[33]
0xfc,		//< NVCSI_CIL_B_CONTROL_0_OFFSET		regs[34]
0x8c,		//< NVCSI_CIL_B_OFFSET				regs[35]
};

void nvcsi_deskew_platform_setup(struct tegra_csi_device *dev, bool t19x)
{
	int i;

	mc_csi = dev;
	is_t19x_or_greater = t19x;
	mutex_init(&deskew_lock);
	enabled_deskew_lanes = 0;
	done_deskew_lanes = 0;
	if (is_t19x_or_greater)
		for (i = 0; i < REGS_COUNT; ++i)
			regs[i] = t194_regs[i];
	else
		for (i = 0; i < REGS_COUNT; ++i)
			regs[i] = t186_regs[i];
}

static inline void set_enabled_with_lock(unsigned int active_lanes)
{
	mutex_lock(&deskew_lock);
	enabled_deskew_lanes |= active_lanes;
	mutex_unlock(&deskew_lock);
}

static inline void unset_enabled_with_lock(unsigned int active_lanes)
{
	mutex_lock(&deskew_lock);
	enabled_deskew_lanes &= ~active_lanes;
	mutex_unlock(&deskew_lock);
}

static inline void set_done_with_lock(unsigned int done_lanes)
{
	mutex_lock(&deskew_lock);
	done_deskew_lanes |= done_lanes;
	enabled_deskew_lanes &= ~done_lanes;
	mutex_unlock(&deskew_lock);
}


static inline void nvcsi_phy_write(unsigned int phy_num,
				   unsigned int addr_offset, unsigned int val)
{
	unsigned int addr;

	addr = NVCSI_PHY_0_NVCSI_CIL_PHY_CTRL_0 + NVCSI_PHY_OFFSET * phy_num
		+addr_offset;
	dev_dbg(mc_csi->dev, "%s: addr %x val %x\n", __func__, addr,
			val);
	host1x_writel(mc_csi->pdev, addr, val);
}
static inline unsigned int nvcsi_phy_readl(unsigned int phy_num,
					   unsigned int addr_offset)
{
	unsigned int addr;
	int val;

	addr = NVCSI_PHY_0_NVCSI_CIL_PHY_CTRL_0 + NVCSI_PHY_OFFSET * phy_num
		+ addr_offset;
	val = host1x_readl(mc_csi->pdev, addr);
	dev_dbg(mc_csi->dev, "%s: addr %x val %x\n", __func__, addr,
			val);
	return val;
}

static void nvcsi_deskew_setup_start(unsigned int active_lanes)
{
	unsigned int phy_num = 0;
	unsigned int cil_lanes = 0, cila_io_lanes = 0, cilb_io_lanes = 0;
	unsigned int remaining_lanes = active_lanes;
	unsigned int val = 0, newval = 0;

	dev_dbg(mc_csi->dev, "%s: active_lanes: %x\n", __func__, active_lanes);
	while (remaining_lanes) {
		cil_lanes = (active_lanes & (0x000f << (phy_num * 4)))
				>> (phy_num * 4);
		cila_io_lanes =  cil_lanes & (NVCSI_PHY_0_NVCSI_CIL_A_IO0
			| NVCSI_PHY_0_NVCSI_CIL_A_IO1);
		cilb_io_lanes = cil_lanes & (NVCSI_PHY_0_NVCSI_CIL_B_IO0
			| NVCSI_PHY_0_NVCSI_CIL_B_IO1);
		remaining_lanes &= ~(0xf << (phy_num * 4));
		if (cila_io_lanes) {
			/*
			 * Disable single bit err when detecting leader
			 * pattern
			 */
			newval = CFG_ERR_STATUS2VI_MASK_ALL;
			host1x_writel(mc_csi->pdev,
				      NVCSI_STREAM_0_ERROR_STATUS2VI_MASK +
				      NVCSI_PHY_OFFSET * phy_num,
				      newval);
			val = nvcsi_phy_readl(phy_num,
				NVCSI_CIL_A_DESKEW_CONTROL_0_OFFSET);
			val = (val & (~(DESKEW_COMPARE | DESKEW_SETTLE)
					))
				| (0x4 << DESKEW_COMPARE_SHIFT)
				| (0X6 << DESKEW_SETTLE_SHIFT);
			nvcsi_phy_write(phy_num,
					NVCSI_CIL_A_DESKEW_CONTROL_0_OFFSET,
					val);
			val = nvcsi_phy_readl(phy_num,
				NVCSI_CIL_A_CONTROL_0_OFFSET);
			val = (val & (~(CLK_SETTLE | THS_SETTLE0 |
							THS_SETTLE1)))
				| (0x19 << CLK_SETTLE_SHIFT0)
				| (0x16 << THS_SETTLE0_SHIFT)
				| (0x16 << THS_SETTLE1_SHIFT);
			nvcsi_phy_write(phy_num, NVCSI_CIL_A_CONTROL_0_OFFSET,
					val);

			if (is_t19x_or_greater)
				nvcsi_phy_write(phy_num,
				NVCSI_CIL_A_DPHY_DESKEW_RESULT_STATUS_OFFSET,
					0
					);

		}
		if (cilb_io_lanes) {
			/*
			 * Disable single bit err when detecting leader
			 * pattern
			 */
			newval = CFG_ERR_STATUS2VI_MASK_ALL;
			host1x_writel(mc_csi->pdev,
				      NVCSI_STREAM_1_ERROR_STATUS2VI_MASK +
				      NVCSI_PHY_OFFSET * phy_num,
				      newval);
			val = nvcsi_phy_readl(phy_num,
				NVCSI_CIL_B_DESKEW_CONTROL_0_OFFSET);
			val = (val & (~(DESKEW_COMPARE | DESKEW_SETTLE)
					))
				| (0x4 << DESKEW_COMPARE_SHIFT)
				| (0X6 << DESKEW_SETTLE_SHIFT);
			nvcsi_phy_write(phy_num,
					NVCSI_CIL_B_DESKEW_CONTROL_0_OFFSET,
					val);
			val = nvcsi_phy_readl(phy_num,
				NVCSI_CIL_B_CONTROL_0_OFFSET);
			val = (val & (~(CLK_SETTLE | THS_SETTLE0 |
							THS_SETTLE1)))
				| (0x19 << CLK_SETTLE_SHIFT0)
				| (0x16 << THS_SETTLE0_SHIFT)
				| (0x16 << THS_SETTLE1_SHIFT);
			nvcsi_phy_write(phy_num, NVCSI_CIL_B_CONTROL_0_OFFSET,
					val);

			if (is_t19x_or_greater)
				nvcsi_phy_write(phy_num,
				NVCSI_CIL_B_DPHY_DESKEW_RESULT_STATUS_OFFSET,
					0
					);

		}

		// HW sometimes throws an error during the deskew operation
		// without a delay here
		// let the nvcsi writes for deskew settings propagate properly
		// before enabling deskew
		usleep_range(40, 50);

		if (cila_io_lanes) {
			val = CLK_INADJ_LIMIT_HIGH;
			nvcsi_phy_write(phy_num,
					NVCSI_CIL_A_CLK_DESKEW_CTRL_0_OFFSET,
					val | CLK_INADJ_SWEEP_CTRL
					);
			val = nvcsi_phy_readl(phy_num,
					NVCSI_CIL_A_DATA_DESKEW_CTRL_0_OFFSET);
			newval =
			((cila_io_lanes & NVCSI_PHY_0_NVCSI_CIL_A_IO0) != 0 ?
			(DATA_INADJ_SWEEP_CTRL0 | DATA_INADJ_LIMIT_HIGH0) : 0)
			|
			((cila_io_lanes & NVCSI_PHY_0_NVCSI_CIL_A_IO1) != 0 ?
			(DATA_INADJ_SWEEP_CTRL1 | DATA_INADJ_LIMIT_HIGH1) : 0);

			nvcsi_phy_write(phy_num,
					NVCSI_CIL_A_DATA_DESKEW_CTRL_0_OFFSET,
					((val & ~(DATA_INADJ_SWEEP_CTRL0 |
						DATA_INADJ_SWEEP_CTRL1)) |
					newval));
		}

		if (cilb_io_lanes) {
			val = CLK_INADJ_LIMIT_HIGH;
			nvcsi_phy_write(phy_num,
					NVCSI_CIL_B_CLK_DESKEW_CTRL_0_OFFSET,
					val |  CLK_INADJ_SWEEP_CTRL
					);
			val = nvcsi_phy_readl(phy_num,
					NVCSI_CIL_B_DATA_DESKEW_CTRL_0_OFFSET);
			newval =
			((cilb_io_lanes & NVCSI_PHY_0_NVCSI_CIL_B_IO0) != 0 ?
			(DATA_INADJ_SWEEP_CTRL0 | DATA_INADJ_LIMIT_HIGH0) : 0)
			|
			((cilb_io_lanes & NVCSI_PHY_0_NVCSI_CIL_B_IO1) != 0 ?
			(DATA_INADJ_SWEEP_CTRL1 | DATA_INADJ_LIMIT_HIGH1) : 0);

			nvcsi_phy_write(phy_num,
					NVCSI_CIL_B_DATA_DESKEW_CTRL_0_OFFSET,
					((val & ~(DATA_INADJ_SWEEP_CTRL0 |
						DATA_INADJ_SWEEP_CTRL1)) |
					newval));
		}
		phy_num++;
	}
}

static int wait_cila_done(unsigned int phy_num, unsigned int cila_io_lanes,
			unsigned long timeout)
{
	bool done;
	unsigned int val;

	while (time_before(jiffies, timeout)) {
		done = true;
		val = nvcsi_phy_readl(phy_num,
	NVCSI_CIL_A_DPHY_DESKEW_STATUS_0_OFFSET);
		if (cila_io_lanes & NVCSI_PHY_0_NVCSI_CIL_A_IO0)
			done &= !!(val & DPHY_CALIB_DONE_IO0);
		if (cila_io_lanes & NVCSI_PHY_0_NVCSI_CIL_A_IO1)
			done &= !!(val & DPHY_CALIB_DONE_IO1);
		if (val & DPHY_CALIB_ERR_IO1 || val & DPHY_CALIB_ERR_IO0)
			return -EINVAL;
		if (done)
			return 0;
		usleep_range(5, 10);
	}
	return -ETIMEDOUT;
}

static int wait_cilb_done(unsigned int phy_num, unsigned int cilb_io_lanes,
			unsigned long timeout)
{
	bool done;
	unsigned int val;

	while (time_before(jiffies, timeout)) {
		done = true;
		val = nvcsi_phy_readl(phy_num,
	NVCSI_CIL_B_DPHY_DESKEW_STATUS_0_OFFSET);
		if (cilb_io_lanes & NVCSI_PHY_0_NVCSI_CIL_B_IO0)
			done &= !!(val & DPHY_CALIB_DONE_IO0);
		if (cilb_io_lanes & NVCSI_PHY_0_NVCSI_CIL_B_IO1)
			done &= !!(val & DPHY_CALIB_DONE_IO1);
		if (val & DPHY_CALIB_ERR_IO1 || val & DPHY_CALIB_ERR_IO0)
			return -EINVAL;
		if (done)
			return 0;
		usleep_range(5, 10);
	}
	return -ETIMEDOUT;
}

static int nvcsi_deskew_thread(void *data)
{
	int ret = 0;
	unsigned int phy_num = 0;
	unsigned int cil_lanes = 0, cila_io_lanes = 0, cilb_io_lanes = 0;
	struct nvcsi_deskew_context *ctx = data;
	unsigned int remaining_lanes = ctx->deskew_lanes;
	unsigned long timeout = 0;

	timeout = jiffies + msecs_to_jiffies(DESKEW_TIMEOUT_MSEC);

	while (remaining_lanes) {
		cil_lanes = (ctx->deskew_lanes & (0x000f << (phy_num * 4)))
				>> (phy_num * 4);
		cila_io_lanes =  cil_lanes & (NVCSI_PHY_0_NVCSI_CIL_A_IO0
			| NVCSI_PHY_0_NVCSI_CIL_A_IO1);
		cilb_io_lanes = cil_lanes & (NVCSI_PHY_0_NVCSI_CIL_B_IO0
			| NVCSI_PHY_0_NVCSI_CIL_B_IO1);
		remaining_lanes &= ~(0xf << (phy_num * 4));
		if (cila_io_lanes) {
			ret = wait_cila_done(phy_num, cila_io_lanes, timeout);
			if (ret)
				goto err;
		}
		if (cilb_io_lanes) {
			ret = wait_cilb_done(phy_num, cilb_io_lanes, timeout);
			if (ret)
				goto err;
		}
		phy_num++;
	}

	ret = nvcsi_deskew_apply_helper(ctx->deskew_lanes);
	if (!ret) {
		dev_info(mc_csi->dev, "deskew finished for lanes 0x%04x",
							ctx->deskew_lanes);
		set_done_with_lock(ctx->deskew_lanes);
	} else {
		dev_info(mc_csi->dev,
			"deskew apply helper failed for lanes 0x%04x",
							ctx->deskew_lanes);
		goto err;
	}


	complete(&ctx->thread_done);
	return 0;

err:
	if (ret == -ETIMEDOUT)
		dev_info(mc_csi->dev, "deskew timed out for lanes 0x%04x",
					ctx->deskew_lanes);
	else if (ret == -EINVAL)
		dev_info(mc_csi->dev, "deskew calib err for lanes 0x%04x",
					ctx->deskew_lanes);
	unset_enabled_with_lock(ctx->deskew_lanes);
	complete(&ctx->thread_done);

	return ret;
}

int nvcsi_deskew_setup(struct nvcsi_deskew_context *ctx)
{
	int ret = 0;
	unsigned int new_lanes;

	if (!ctx || !ctx->deskew_lanes)
		return -EINVAL;

	if (ctx->deskew_lanes >> NVCSI_PHY_CIL_NUM_LANE) {
		dev_err(mc_csi->dev, "%s Invalid lanes for deskew\n",
								__func__);
		return -EINVAL;
	}

	mutex_lock(&deskew_lock);
	done_deskew_lanes &= ~(ctx->deskew_lanes);
	mutex_unlock(&deskew_lock);

	new_lanes = ctx->deskew_lanes & ~enabled_deskew_lanes;
	if (new_lanes) {
		set_enabled_with_lock(new_lanes);
		nvcsi_deskew_setup_start(new_lanes);
		init_completion(&ctx->thread_done);
		ctx->deskew_kthread = kthread_run(nvcsi_deskew_thread,
							ctx, "deskew");
		if (IS_ERR(ctx->deskew_kthread)) {
			ret = PTR_ERR(ctx->deskew_kthread);
			complete(&ctx->thread_done);
		}
	}
	return ret;
}
EXPORT_SYMBOL(nvcsi_deskew_setup);

static inline unsigned int checkpass(unsigned long long stat)
{
	/* Due to bug 200098288, use
	 * mask101 = 0x5; mask111 = 0x7; mask1010 = 0xa;
	 * to check if current trimmer setting results in passing
	 * Algorithm is explained in NVCSI_CIL_IAS chapter 5.3
	 */

	return ((stat & 0x5) == 0x5 || ((stat & 0x7) == 0x7)
		|| (stat & 0xa) == 0xa);
}

/* compute_boundary:
 * This function find the flipping point when the trimmer settings starts
 * to pass/fail.
 * Each graph represent the 64-bit status,trimmer setting 0~0x3F
 * from Right to Left.
 *
 * pf: pass to fail, fp: fail to pass
 *
 *   pf     fp
 * __|------|__
 *     pf     fp = 0
 * ____|-------
 * pf=0x3f   fp
 * ----------|__
 */
static unsigned int compute_boundary(unsigned long long stat, unsigned int *x,
				     unsigned int *w)
{
	unsigned int was_pass, i = 0;
	int pf = -1, fp = -1;
	unsigned long long last_stat;

	was_pass = checkpass(stat);
	fp = (was_pass == 1 ? 0 : -1);
	last_stat = stat;

	while (i < 64) {
		if ((was_pass == 1) && (!(last_stat & 1))) {
			if (!checkpass(last_stat)) {
				pf = i;
				was_pass = 0;
				dev_dbg(mc_csi->dev, "pf %d\n", pf);
			}
		} else if ((was_pass == 0) && (last_stat & 1)) {
			if (checkpass(last_stat)) {
				fp = i;
				was_pass = 1;
				dev_dbg(mc_csi->dev, "fp %d\n", fp);
			}
		}
		i++;
		last_stat >>= 1;
	}

	dev_dbg(mc_csi->dev, "fp %d pf %d\n", fp, pf);
	if (fp == -1 && pf == -1) {
		dev_dbg(mc_csi->dev, "No passing record found, please retry\n");
		return -EINVAL;
	} else if (pf == -1 && was_pass == 1)
		pf = 0x3f;

	*x = pf;
	*w = fp;
	dev_dbg(mc_csi->dev, "%s: stats %llx, f2p %d, p2f %d",
		__func__, stat, fp, pf);

	return 0;
}

static unsigned int error_boundary(unsigned int phy_num, unsigned int cil_bit,
			    unsigned int *x, unsigned int *w,
			    unsigned int *y, unsigned int *z)
{
	unsigned int stats_low = 0, stats_high = 0, stats_offset = 0;
	unsigned long long result = 0;
	unsigned int is_cilb = 0, is_io1 = 0;

	is_cilb = (cil_bit > 1);
	is_io1 = (cil_bit % 2);
	dev_dbg(mc_csi->dev, "boundary for cilb?:%d io1?:%d\n",
							is_cilb, is_io1);
	stats_offset = is_cilb * NVCSI_CIL_B_OFFSET +
				is_io1 * NVCSI_DPHY_CALIB_STATUS_IO_OFFSET;
	/* step #1 clk lane */
	stats_low = nvcsi_phy_readl(phy_num, stats_offset +
		NVCSI_CIL_A_DPHY_DESKEW_CLK_CALIB_STATUS_LOW_0_0_OFFSET);
	stats_high = nvcsi_phy_readl(phy_num, stats_offset +
		NVCSI_CIL_A_DPHY_DESKEW_CLK_CALIB_STATUS_HIGH_0_0_OFFSET);
	result = ((unsigned long long)stats_high) << 32 | stats_low;

	debugfs_deskew_clk_stats_low[cil_bit + phy_num * 4] = stats_low;
	debugfs_deskew_clk_stats_high[cil_bit + phy_num * 4] = stats_high;

	dev_dbg(mc_csi->dev, "clk boundary: 0x%016llx\n", result);

	if (compute_boundary(result, x, w))
		return -EINVAL;
	/* step #2 data lane */
	stats_low = nvcsi_phy_readl(phy_num, stats_offset +
		NVCSI_CIL_A_DPHY_DESKEW_DATA_CALIB_STATUS_LOW_0_0_OFFSET);
	stats_high = nvcsi_phy_readl(phy_num, stats_offset +
		NVCSI_CIL_A_DPHY_DESKEW_DATA_CALIB_STATUS_HIGH_0_0_OFFSET);
	result = ((unsigned long long)stats_high) << 32 | stats_low;

	debugfs_deskew_data_stats_low[cil_bit + phy_num * 4] = stats_low;
	debugfs_deskew_data_stats_high[cil_bit + phy_num * 4] = stats_high;

	dev_dbg(mc_csi->dev, "data boundary: 0x%016llx\n", result);
	if (compute_boundary(result, y, z))
		return -EINVAL;

	return 0;
}
static void compute_trimmer(unsigned int *x, unsigned int *w,
			    unsigned int *y, unsigned int *z,
			    unsigned int *d, unsigned int *c)
{
	int mid[4], base = 0;
	unsigned int i = 0;


	/* NVCSI_CIL_IAS Chapter 5.3 */
	for (i = 0; i < 4; i++) {
		if (w[i] < z[i]) {
			y[i] = 0;
			z[i] = 0;
		} else if (w[i] > z[i]) {
			x[i] = 0;
			w[i] = 0;
		}
		mid[i] = ((y[i] + z[i]) - (x[i] + w[i])) >> 1;
		base = mid[i] < base ? mid[i] : base;
	}
	*c = -base;
	for (i = 0; i < 4; i++)
		d[i] = mid[i] - base;

	/* debug prints */
	for (i = 0; i < 4; i++)
		dev_dbg(mc_csi->dev, "x %u w %u y %u z %u d %u\n",
				x[i], w[i], y[i], z[i], d[i]);
	dev_dbg(mc_csi->dev, "clk %u\n", *c);
}
static void set_trimmer(unsigned int phy_num, unsigned int cila,
			unsigned int cilb,
			unsigned int *d, unsigned int c)
{
	unsigned int val = 0, val1 = 0;

	if (cila && cilb) {
		/* 4-lane */
		val = SW_SET_DPHY_INADJ_CLK |
		      SW_SET_DPHY_INADJ_IO0 |
		      SW_SET_DPHY_INADJ_IO1 |
		      (c << DPHY_INADJ_CLK_SHIFT) |
		      (d[PHY_0_CIL_A_IO0] << DPHY_INADJ_IO0_SHIFT) |
		      (d[PHY_0_CIL_A_IO1] << DPHY_INADJ_IO1_SHIFT);
		val1 = SW_SET_DPHY_INADJ_IO0 |
		       SW_SET_DPHY_INADJ_IO1 |
		       (d[PHY_0_CIL_B_IO0] << DPHY_INADJ_IO0_SHIFT)|
		       (d[PHY_0_CIL_B_IO1] << DPHY_INADJ_IO1_SHIFT);
		nvcsi_phy_write(phy_num, NVCSI_CIL_A_DPHY_INADJ_CTRL_0_OFFSET,
				val);
		nvcsi_phy_write(phy_num, NVCSI_CIL_B_DPHY_INADJ_CTRL_0_OFFSET,
				val1);
		dev_dbg(mc_csi->dev, "cila %x cilb %x\n", val, val1);
		return;
	}
	/* TODO:
	 * 2-lane and 1-lane cases cannot be verified since there
	 * is no such sensor supported yet
	 */
	if (cila) {
		/* 2-lane and 1-lane*/
		val1 = nvcsi_phy_readl(phy_num,
				NVCSI_CIL_A_DPHY_INADJ_CTRL_0_OFFSET);
		if (cila & NVCSI_PHY_0_NVCSI_CIL_A_IO0) {
			val1 &= ~(SW_SET_DPHY_INADJ_IO0 | DPHY_INADJ_IO0);
			val |= SW_SET_DPHY_INADJ_IO0 |
			      (d[PHY_0_CIL_A_IO0] << DPHY_INADJ_IO0_SHIFT);

		}
		if (cila & NVCSI_PHY_0_NVCSI_CIL_A_IO1) {
			val1 &= ~(SW_SET_DPHY_INADJ_IO1 | DPHY_INADJ_IO1);
			val |= SW_SET_DPHY_INADJ_IO1 |
			      (d[PHY_0_CIL_A_IO1] << DPHY_INADJ_IO1_SHIFT);
		}
		val1 &= ~(SW_SET_DPHY_INADJ_CLK | DPHY_INADJ_CLK);
		val |= SW_SET_DPHY_INADJ_CLK | (c << DPHY_INADJ_CLK_SHIFT);

		nvcsi_phy_write(phy_num, NVCSI_CIL_A_DPHY_INADJ_CTRL_0_OFFSET,
				val | val1);
		dev_dbg(mc_csi->dev, "cila %x\n", val | val1);
	} else {
		/* 2-lane and 1-lane*/
		val1 = nvcsi_phy_readl(phy_num,
				NVCSI_CIL_B_DPHY_INADJ_CTRL_0_OFFSET);
		if (cilb & NVCSI_PHY_0_NVCSI_CIL_B_IO0) {
			val1 &= ~(SW_SET_DPHY_INADJ_IO0 | DPHY_INADJ_IO0);
			val |= SW_SET_DPHY_INADJ_IO0 |
				(d[PHY_0_CIL_B_IO0] << DPHY_INADJ_IO0_SHIFT);
		}
		if (cilb & NVCSI_PHY_0_NVCSI_CIL_B_IO1) {
			val1 &= ~(SW_SET_DPHY_INADJ_IO1 | DPHY_INADJ_IO1);
			val |= SW_SET_DPHY_INADJ_IO1 |
				(d[PHY_0_CIL_B_IO1] << DPHY_INADJ_IO1_SHIFT);
		}
		val1 &= ~(SW_SET_DPHY_INADJ_CLK | DPHY_INADJ_CLK);
		val |= SW_SET_DPHY_INADJ_CLK | (c << DPHY_INADJ_CLK_SHIFT);
		nvcsi_phy_write(phy_num, NVCSI_CIL_B_DPHY_INADJ_CTRL_0_OFFSET,
				val | val1);
		dev_dbg(mc_csi->dev, "cilb %x\n", val | val1);
	}
}

int nvcsi_deskew_apply_check(struct nvcsi_deskew_context *ctx)
{
	unsigned long timeout = 0, timeleft = 1;

	if (!completion_done(&ctx->thread_done)) {
		timeout = msecs_to_jiffies(DESKEW_TIMEOUT_MSEC);
		timeleft = wait_for_completion_timeout(&ctx->thread_done,
								timeout);
	}
	if (!timeleft)
		return -ETIMEDOUT;
	if (ctx->deskew_lanes ==
			(done_deskew_lanes & ctx->deskew_lanes)) {
		// sleep for a frame to make sure deskew result is reflected
		usleep_range(35*1000, 36*1000);
		return 0;
	} else
		return -EINVAL;
}
EXPORT_SYMBOL(nvcsi_deskew_apply_check);

static int nvcsi_deskew_apply_helper(unsigned int active_lanes)
{
	unsigned int phy_num = -1;
	unsigned int cil_lanes = 0, cila_io_lanes = 0, cilb_io_lanes = 0;
	unsigned int remaining_lanes = active_lanes;
	unsigned int i, j;

	dev_dbg(mc_csi->dev, "%s: interrupt lane: %x\n",
		__func__, active_lanes);
	while (remaining_lanes) {
		unsigned int x[4] = {0, 0, 0, 0};
		unsigned int w[4] = {0, 0, 0, 0};
		unsigned int y[4] = {0, 0, 0, 0};
		unsigned int z[4] = {0, 0, 0, 0};
		unsigned int d_trimmer[4] = {0, 0, 0, 0};
		unsigned int clk_trimmer = 0;

		phy_num++;
		cil_lanes = (active_lanes & (0xf << (phy_num * 4)))
				>> (phy_num * 4);
		remaining_lanes &= ~(0xf << (phy_num * 4));
		if (!cil_lanes)
			continue;
		cila_io_lanes =  cil_lanes & (NVCSI_PHY_0_NVCSI_CIL_A_IO0
			| NVCSI_PHY_0_NVCSI_CIL_A_IO1);
		cilb_io_lanes = cil_lanes & (NVCSI_PHY_0_NVCSI_CIL_B_IO0
			| NVCSI_PHY_0_NVCSI_CIL_B_IO1);
		/* Step 1: Read status registers and compute error boundaries */
		for (i = NVCSI_PHY_0_NVCSI_CIL_A_IO0, j = 0;
		     i <= NVCSI_PHY_0_NVCSI_CIL_B_IO1;
		     i <<= 1, j++) {
			if ((cil_lanes & i) == 0)
				continue;
			if (error_boundary(phy_num, j,
						&x[j], &w[j],
						&y[j], &z[j]))
				return -EINVAL;
		}
		/*step 2: compute trimmer value based on error boundaries */
		compute_trimmer(x, w, y, z, d_trimmer, &clk_trimmer);
		/*step 3: Apply trimmer settings */
		set_trimmer(phy_num, cila_io_lanes, cilb_io_lanes,
				d_trimmer, clk_trimmer);
	}
	return 0;
}

void deskew_dbgfs_calc_bound(struct seq_file *s, long long input_stats)
{
	unsigned int x, w;

	seq_printf(s, "input: %llx\n", input_stats);
	compute_boundary(input_stats, &x, &w);
	seq_printf(s, "setting: x %u w %u\n", x, w);
}


void deskew_dbgfs_deskew_stats(struct seq_file *s)
{
	unsigned int i = 0;

	seq_puts(s, "clk stats\n");
	for (i = 0; i < NVCSI_PHY_CIL_NUM_LANE; i++) {
		seq_printf(s, "0x%08x 0x%08x\n",
			debugfs_deskew_clk_stats_high[i],
			debugfs_deskew_clk_stats_low[i]);
	}
	seq_puts(s, "data stats\n");
	for (i = 0; i < NVCSI_PHY_CIL_NUM_LANE; i++) {
		seq_printf(s, "0x%08x 0x%08x\n",
				debugfs_deskew_data_stats_high[i],
				debugfs_deskew_data_stats_low[i]);
	}
}

