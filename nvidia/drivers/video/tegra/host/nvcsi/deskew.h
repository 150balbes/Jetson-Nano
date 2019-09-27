/*
 * drivers/video/tegra/host/nvcsi/nvcsi.h
 *
 * Deskew driver
 *
 * Copyright (c) 2018-2019 NVIDIA Corporation.  All rights reserved.
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


#ifndef __DESKEW_H__
#define __DESKEW_H__

#include <linux/completion.h>
#include <uapi/linux/nvhost_nvcsi_ioctl.h>
#include <media/csi.h>


////////////////////////////////////////////////////////////////
// STREAM REGISTERS
////////////////////////////////////////////////////////////////

#define NVCSI_STREAM_0_ERROR_STATUS2VI_MASK		regs[0]
#define NVCSI_STREAM_1_ERROR_STATUS2VI_MASK		regs[1]
#define CFG_ERR_STATUS2VI_MASK_ALL			regs[2]

////////////////////////////////////////////////////////////////
// PHY REGISTERS
////////////////////////////////////////////////////////////////

// PHY INTERRUPTS REGISTERS
#define NVCSI_PHY_0_CILA_INTR_STATUS			regs[3]
// bits in register NVCSI_PHY_0_CILA_INTR_STATUS
#define intr_dphy_cil_deskew_calib_err_ctrl	(1 << 27)
#define intr_dphy_cil_deskew_calib_err_lane1	(1 << 26)
#define intr_dphy_cil_deskew_calib_err_lane0	(1 << 25)
#define intr_dphy_cil_deskew_calib_done_ctrl	(1 << 24)
#define intr_dphy_cil_deskew_calib_done_lane1	(1 << 23)
#define intr_dphy_cil_deskew_calib_done_lane0	(1 << 22)
#define NVCSI_PHY_0_CILA_INTR_MASK			regs[4]
#define NVCSI_PHY_0_CILB_INTR_STATUS			regs[5]
#define NVCSI_PHY_0_CILB_INTR_MASK			regs[6]
// new registers in T194
#define T194_NVCSI_PHY_0_CILA_INTR_1_STATUS		0x10404
#define T194_NVCSI_PHY_0_CILA_INTR_1_MASK		0x1040c
#define T194_NVCSI_PHY_0_CILB_INTR_1_STATUS		0x10804
#define T194_NVCSI_PHY_0_CILB_INTR_1_MASK		0x1080c

////////////////////////////////////////////////////////////////
// PHY DESKEW REGISTERS
////////////////////////////////////////////////////////////////

// XXX_OFFSET: address offset from NVCSI_CIL_PHY_CTRL_0
#define NVCSI_PHY_0_NVCSI_CIL_PHY_CTRL_0		regs[7]
#define NVCSI_CIL_A_SW_RESET_0_OFFSET			regs[8]
#define NVCSI_CIL_A_CLK_DESKEW_CTRL_0_OFFSET		regs[9]
// bits in register NVCSI_CIL_A_CLK_DESKEW_CTRL_0
#define CLK_INADJ_SWEEP_CTRL			(0x1 << 15)
#define CLK_INADJ_LIMIT_HIGH			(0x3f << 8)
#define CLK_INADJ_LIMIT_LOW			0x3f
#define NVCSI_CIL_A_DPHY_INADJ_CTRL_0_OFFSET		regs[10]
// bits in register NVCSI_CIL_A_DPHY_INADJ_CTRL_0
#define SW_SET_DPHY_INADJ_CLK			(0x1 << 22)
#define DPHY_INADJ_CLK				(0x3f << 16)
#define DPHY_INADJ_CLK_SHIFT			16
#define SW_SET_DPHY_INADJ_IO1			(0x1 << 14)
#define DPHY_INADJ_IO1				(0x3f << 8)
#define DPHY_INADJ_IO1_SHIFT			8
#define SW_SET_DPHY_INADJ_IO0			(0x1 << 6)
#define DPHY_INADJ_IO0				0x3f
#define DPHY_INADJ_IO0_SHIFT			0

#define NVCSI_CIL_A_DATA_DESKEW_CTRL_0_OFFSET		regs[11]
// bits in register NVCSI_CIL_A_DATA_DESKEW_CTRL_0
#define DATA_INADJ_SWEEP_CTRL1			(0x1 << 31)
#define DATA_INADJ_SWEEP_CTRL0			(0x1 << 15)
#define DATA_INADJ_LIMIT_HIGH1			(0x3f << 23)
#define DATA_INADJ_LIMIT_HIGH0			(0x3f << 8)

#define NVCSI_CIL_A_DPHY_DESKEW_STATUS_0_OFFSET		regs[12]
// bits in register NVCSI_CIL_A_DPHY_DESKEW_STATUS_0
#define DPHY_CALIB_ERR_IO1			(0x1 << 15)
#define DPHY_CALIB_DONE_IO1			(0x1 << 14)
#define DPHY_CALIB_ERR_IO0			(0x1 << 7)
#define DPHY_CALIB_DONE_IO0			(0x1 << 6)

#define NVCSI_CIL_A_DPHY_DESKEW_DATA_CALIB_STATUS_LOW_0_0_OFFSET	regs[13]
#define NVCSI_CIL_A_DPHY_DESKEW_DATA_CALIB_STATUS_HIGH_0_0_OFFSET	regs[14]
#define NVCSI_CIL_A_DPHY_DESKEW_CLK_CALIB_STATUS_LOW_0_0_OFFSET		regs[15]
#define NVCSI_CIL_A_DPHY_DESKEW_CLK_CALIB_STATUS_HIGH_0_0_OFFSET	regs[16]

// only for t194+
#define NVCSI_CIL_A_DPHY_DESKEW_RESULT_STATUS_OFFSET	0x64
#define NVCSI_CIL_B_DPHY_DESKEW_RESULT_STATUS_OFFSET	0xf0

/*
 * NVCSI_PHY_0_NVCSI_CIL_A_DESKEW_CONTROL_0 was introduced in T194
 * Use this register for DESKEW_COMPARE and DESKEW_SETTLE
 * On T186, this register doesn't exist, and will be mapped to
 * NVCSI_PHY_0_NVCSI_CIL_A_CONTROL_0 for deskew compare/settle programming
 */
#define NVCSI_CIL_A_DESKEW_CONTROL_0_OFFSET		regs[17]
#define NVCSI_CIL_A_CONTROL_0_OFFSET			regs[18]

/*
 * bits in NVCSI_CIL_A_DESKEW_CONTROL_0/NVCSI_CIL_A_CONTROL_0
 * For T194, the THS_SETTLE control was split into
 * THS_SETTLE0 and THS_SETTLE1 for per-lane control
 * For T186, THS_SETTLE0_SHIFT and THS_SETTLE0_SHIFT1 will be the same.
 */
#define	DESKEW_COMPARE					regs[19]
#define DESKEW_COMPARE_SHIFT				regs[20]
#define DESKEW_SETTLE					regs[21]
#define DESKEW_SETTLE_SHIFT				regs[22]
#define CLK_SETTLE					regs[23]
#define CLK_SETTLE_SHIFT0				regs[24]
#define THS_SETTLE0					regs[25]
#define THS_SETTLE1					regs[26]
#define THS_SETTLE0_SHIFT				regs[27]
#define THS_SETTLE1_SHIFT				regs[28]

#define NVCSI_CIL_B_DPHY_INADJ_CTRL_0_OFFSET		regs[29]
#define NVCSI_CIL_B_CLK_DESKEW_CTRL_0_OFFSET		regs[30]
#define NVCSI_CIL_B_DATA_DESKEW_CTRL_0_OFFSET		regs[31]
#define NVCSI_CIL_B_DPHY_DESKEW_STATUS_0_OFFSET		regs[32]
// same note as above for NVCSI_CIL_A_DESKEW_CONTROL_0
#define NVCSI_CIL_B_DESKEW_CONTROL_0_OFFSET		regs[33]
#define NVCSI_CIL_B_CONTROL_0_OFFSET			regs[34]

#define NVCSI_DPHY_CALIB_STATUS_IO_OFFSET		0x8
#define NVCSI_PHY_OFFSET				0x10000
#define NVCSI_CIL_B_OFFSET				regs[35]

#define REGS_COUNT					36

////////

#define DESKEW_TIMEOUT_MSEC 100

struct nvcsi_deskew_context {
	unsigned int deskew_lanes;
	struct task_struct *deskew_kthread;
	struct completion thread_done;
};

#if IS_ENABLED(CONFIG_TEGRA_GRHOST_NVCSI)
int nvcsi_deskew_apply_check(struct nvcsi_deskew_context *ctx);
int nvcsi_deskew_setup(struct nvcsi_deskew_context *ctx);
#else
static int inline nvcsi_deskew_apply_check(struct nvcsi_deskew_context *ctx)
{
	return 0;
}
static int inline nvcsi_deskew_setup(struct nvcsi_deskew_context *ctx)
{
	return 0;
}
#endif

void nvcsi_deskew_platform_setup(struct tegra_csi_device *dev, bool is_t19x);

void deskew_dbgfs_calc_bound(struct seq_file *s, long long input_stats);
void deskew_dbgfs_deskew_stats(struct seq_file *s);

#endif
