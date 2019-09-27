/*
 * Copyright (c) 2016-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */
#include <nvgpu/bug.h>
#include <nvgpu/xve.h>
#include <nvgpu/io.h>
#include <nvgpu/utils.h>
#include <nvgpu/timers.h>
#include <nvgpu/gk20a.h>

#include "gp106/bios_gp106.h"

#include "xve_gp106.h"

#include <nvgpu/hw/gp106/hw_xp_gp106.h>
#include <nvgpu/hw/gp106/hw_xve_gp106.h>

#define NV_PCFG 0x88000

void xve_xve_writel_gp106(struct gk20a *g, u32 reg, u32 val)
{
	gk20a_writel(g, NV_PCFG + reg, val);
}

u32 xve_xve_readl_gp106(struct gk20a *g, u32 reg)
{
	return gk20a_readl(g, NV_PCFG + reg);
}

/**
 * Resets the GPU (except the XVE/XP).
 */
void xve_reset_gpu_gp106(struct gk20a *g)
{
	u32 reset;

	/*
	 * This resets the GPU except for the XVE/XP (since then we would lose
	 * the dGPU from the bus). t18x has a HW limitation where once that
	 * happens the GPU is gone until the entire system is reset.
	 *
	 * We have to use the auto-deassert register since we won't be able to
	 * access the GPU after the GPU goes into reset. This appears like the
	 * GPU has dropped from the bus and causes nvgpu to reset the entire
	 * system. Whoops!
	 */
	reset = xve_reset_reset_m() |
		xve_reset_gpu_on_sw_reset_m() |
		xve_reset_counter_en_m() |
		xve_reset_counter_val_f(0x7ff) |
		xve_reset_clock_on_sw_reset_m() |
		xve_reset_clock_counter_en_m() |
		xve_reset_clock_counter_val_f(0x7ff);

	g->ops.xve.xve_writel(g, xve_reset_r(), reset | xve_reset_reset_m());

	/*
	 * Don't access GPU until _after_ it's back out of reset!
	 */
	nvgpu_msleep(100);
	g->ops.xve.xve_writel(g, xve_reset_r(), 0);
}

/**
 * Places one of:
 *
 *   %GPU_XVE_SPEED_2P5
 *   %GPU_XVE_SPEED_5P0
 *   %GPU_XVE_SPEED_8P0
 *
 * in the u32 pointed to by @xve_link_speed. If for some reason an unknown PCIe
 * bus speed is detected then *@xve_link_speed is not touched and -ENODEV is
 * returned.
 */
int xve_get_speed_gp106(struct gk20a *g, u32 *xve_link_speed)
{
	u32 status;
	u32 link_speed, real_link_speed = 0;

	status = g->ops.xve.xve_readl(g, xve_link_control_status_r());

	link_speed = xve_link_control_status_link_speed_v(status);

	/*
	 * Can't use a switch statement becuase switch statements dont work with
	 * function calls.
	 */
	if (link_speed == xve_link_control_status_link_speed_link_speed_2p5_v()) {
		real_link_speed = GPU_XVE_SPEED_2P5;
	}
	if (link_speed == xve_link_control_status_link_speed_link_speed_5p0_v()) {
		real_link_speed = GPU_XVE_SPEED_5P0;
	}
	if (link_speed == xve_link_control_status_link_speed_link_speed_8p0_v()) {
		real_link_speed = GPU_XVE_SPEED_8P0;
	}

	if (real_link_speed == 0U) {
		return -ENODEV;
	}

	*xve_link_speed = real_link_speed;
	return 0;
}

/**
 * Set the mask for L0s in the XVE.
 *
 * When @status is non-zero the mask for L0s is set which _disables_ L0s. When
 * @status is zero L0s is no longer masked and may be enabled.
 */
static void set_xve_l0s_mask(struct gk20a *g, bool status)
{
	u32 xve_priv;
	u32 status_bit = status ? 1 : 0;

	xve_priv = g->ops.xve.xve_readl(g, xve_priv_xv_r());

	xve_priv = set_field(xve_priv,
		  xve_priv_xv_cya_l0s_enable_m(),
		  xve_priv_xv_cya_l0s_enable_f(status_bit));

	g->ops.xve.xve_writel(g, xve_priv_xv_r(), xve_priv);
}

/**
 * Set the mask for L1 in the XVE.
 *
 * When @status is non-zero the mask for L1 is set which _disables_ L0s. When
 * @status is zero L1 is no longer masked and may be enabled.
 */
static void set_xve_l1_mask(struct gk20a *g, int status)
{
	u32 xve_priv;
	u32 status_bit = (status != 0) ? 1 : 0;

	xve_priv = g->ops.xve.xve_readl(g, xve_priv_xv_r());

	xve_priv = set_field(xve_priv,
		  xve_priv_xv_cya_l1_enable_m(),
		  xve_priv_xv_cya_l1_enable_f(status_bit));

	g->ops.xve.xve_writel(g, xve_priv_xv_r(), xve_priv);
}

/**
 * Disable ASPM permanently.
 */
void xve_disable_aspm_gp106(struct gk20a *g)
{
	set_xve_l0s_mask(g, true);
	set_xve_l1_mask(g, true);
}

/**
 * When doing the speed change disable power saving features.
 */
static void disable_aspm_gp106(struct gk20a *g)
{
	u32 xve_priv;

	xve_priv = g->ops.xve.xve_readl(g, xve_priv_xv_r());

	/*
	 * Store prior ASPM state so we can restore it later on.
	 */
	g->xve_l0s = xve_priv_xv_cya_l0s_enable_v(xve_priv);
	g->xve_l1  = xve_priv_xv_cya_l1_enable_v(xve_priv);

	set_xve_l0s_mask(g, true);
	set_xve_l1_mask(g, true);
}

/**
 * Restore the state saved by disable_aspm_gp106().
 */
static void enable_aspm_gp106(struct gk20a *g)
{
	set_xve_l0s_mask(g, g->xve_l0s);
	set_xve_l1_mask(g, g->xve_l1);
}

/*
 * Error checking is done in xve_set_speed_gp106.
 */
static int __do_xve_set_speed_gp106(struct gk20a *g, u32 next_link_speed)
{
	u32 current_link_speed, new_link_speed;
	u32 dl_mgr, saved_dl_mgr;
	u32 pl_link_config;
	u32 link_control_status, link_speed_setting, link_width;
	struct nvgpu_timeout timeout;
	int attempts = 10, err_status = 0;

	g->ops.xve.get_speed(g, &current_link_speed);
	xv_sc_dbg(g, PRE_CHANGE, "Executing PCIe link change.");
	xv_sc_dbg(g, PRE_CHANGE, "  Current speed:  %s",
		  xve_speed_to_str(current_link_speed));
	xv_sc_dbg(g, PRE_CHANGE, "  Next speed:     %s",
		  xve_speed_to_str(next_link_speed));
	xv_sc_dbg(g, PRE_CHANGE, "  PL_LINK_CONFIG: 0x%08x",
		  gk20a_readl(g, xp_pl_link_config_r(0)));

	xv_sc_dbg(g, DISABLE_ASPM, "Disabling ASPM...");
	disable_aspm_gp106(g);
	xv_sc_dbg(g, DISABLE_ASPM, "  Done!");

	xv_sc_dbg(g, DL_SAFE_MODE, "Putting DL in safe mode...");
	saved_dl_mgr = gk20a_readl(g, xp_dl_mgr_r(0));

	/*
	 * Put the DL in safe mode.
	 */
	dl_mgr = saved_dl_mgr;
	dl_mgr |= xp_dl_mgr_safe_timing_f(1);
	gk20a_writel(g, xp_dl_mgr_r(0), dl_mgr);
	xv_sc_dbg(g, DL_SAFE_MODE, "  Done!");

	nvgpu_timeout_init(g, &timeout, GPU_XVE_TIMEOUT_MS,
			NVGPU_TIMER_CPU_TIMER);

	xv_sc_dbg(g, CHECK_LINK, "Checking for link idle...");
	do {
		pl_link_config = gk20a_readl(g, xp_pl_link_config_r(0));
		if ((xp_pl_link_config_ltssm_status_f(pl_link_config) ==
		     xp_pl_link_config_ltssm_status_idle_v()) &&
		    (xp_pl_link_config_ltssm_directive_f(pl_link_config) ==
		     xp_pl_link_config_ltssm_directive_normal_operations_v())) {
			break;
		}
	} while (nvgpu_timeout_expired(&timeout) == 0);

	if (nvgpu_timeout_peek_expired(&timeout)) {
		err_status = -ETIMEDOUT;
		goto done;
	}

	xv_sc_dbg(g, CHECK_LINK, "  Done");

	xv_sc_dbg(g, LINK_SETTINGS, "Preparing next link settings");
	pl_link_config &= ~xp_pl_link_config_max_link_rate_m();
	switch (next_link_speed) {
	case GPU_XVE_SPEED_2P5:
		link_speed_setting =
			xve_link_control_status_link_speed_link_speed_2p5_v();
		pl_link_config |= xp_pl_link_config_max_link_rate_f(
			xp_pl_link_config_max_link_rate_2500_mtps_v());
		break;
	case GPU_XVE_SPEED_5P0:
		link_speed_setting =
			xve_link_control_status_link_speed_link_speed_5p0_v();
		pl_link_config |= xp_pl_link_config_max_link_rate_f(
			xp_pl_link_config_max_link_rate_5000_mtps_v());
		break;
	case GPU_XVE_SPEED_8P0:
		link_speed_setting =
			xve_link_control_status_link_speed_link_speed_8p0_v();
		pl_link_config |= xp_pl_link_config_max_link_rate_f(
			xp_pl_link_config_max_link_rate_8000_mtps_v());
		break;
	default:
		BUG(); /* Should never be hit. */
	}

	link_control_status =
		g->ops.xve.xve_readl(g, xve_link_control_status_r());
	link_width = xve_link_control_status_link_width_v(link_control_status);

	pl_link_config &= ~xp_pl_link_config_target_tx_width_m();

	/* Can't use a switch due to oddities in register definitions. */
	if (link_width == xve_link_control_status_link_width_x1_v()) {
		pl_link_config |= xp_pl_link_config_target_tx_width_f(
			xp_pl_link_config_target_tx_width_x1_v());
	} else if (link_width == xve_link_control_status_link_width_x2_v()) {
		pl_link_config |= xp_pl_link_config_target_tx_width_f(
			xp_pl_link_config_target_tx_width_x2_v());
	} else if (link_width == xve_link_control_status_link_width_x4_v()) {
		pl_link_config |= xp_pl_link_config_target_tx_width_f(
			xp_pl_link_config_target_tx_width_x4_v());
	} else if (link_width == xve_link_control_status_link_width_x8_v()) {
		pl_link_config |= xp_pl_link_config_target_tx_width_f(
			xp_pl_link_config_target_tx_width_x8_v());
	} else if (link_width == xve_link_control_status_link_width_x16_v()) {
		pl_link_config |= xp_pl_link_config_target_tx_width_f(
			xp_pl_link_config_target_tx_width_x16_v());
	} else {
		BUG();
	}

	xv_sc_dbg(g, LINK_SETTINGS, "  pl_link_config = 0x%08x", pl_link_config);
	xv_sc_dbg(g, LINK_SETTINGS, "  Done");

	xv_sc_dbg(g, EXEC_CHANGE, "Running link speed change...");

	nvgpu_timeout_init(g, &timeout, GPU_XVE_TIMEOUT_MS,
			NVGPU_TIMER_CPU_TIMER);
	do {
		gk20a_writel(g, xp_pl_link_config_r(0), pl_link_config);
		if (pl_link_config ==
		    gk20a_readl(g, xp_pl_link_config_r(0))) {
			break;
		}
	} while (nvgpu_timeout_expired(&timeout) == 0);

	if (nvgpu_timeout_peek_expired(&timeout)) {
		err_status = -ETIMEDOUT;
		goto done;
	}

	xv_sc_dbg(g, EXEC_CHANGE, "  Wrote PL_LINK_CONFIG.");

	pl_link_config = gk20a_readl(g, xp_pl_link_config_r(0));

	do {
		pl_link_config = set_field(pl_link_config,
			  xp_pl_link_config_ltssm_directive_m(),
			  xp_pl_link_config_ltssm_directive_f(
			  xp_pl_link_config_ltssm_directive_change_speed_v()));

		xv_sc_dbg(g, EXEC_CHANGE, "  Executing change (0x%08x)!",
			  pl_link_config);
		gk20a_writel(g, xp_pl_link_config_r(0), pl_link_config);

		/*
		 * Read NV_XP_PL_LINK_CONFIG until the link has swapped to
		 * the target speed.
		 */
		nvgpu_timeout_init(g, &timeout, GPU_XVE_TIMEOUT_MS,
				NVGPU_TIMER_CPU_TIMER);
		do {
			pl_link_config = gk20a_readl(g, xp_pl_link_config_r(0));
			if (pl_link_config != 0xfffffff &&
			    (xp_pl_link_config_ltssm_status_f(pl_link_config) ==
			     xp_pl_link_config_ltssm_status_idle_v()) &&
			    (xp_pl_link_config_ltssm_directive_f(pl_link_config) ==
			     xp_pl_link_config_ltssm_directive_normal_operations_v())) {
				break;
			}
		} while (nvgpu_timeout_expired(&timeout) == 0);

		if (nvgpu_timeout_peek_expired(&timeout)) {
			err_status = -ETIMEDOUT;
			xv_sc_dbg(g, EXEC_CHANGE, "  timeout; pl_link_config = 0x%x",
				pl_link_config);
		}

		xv_sc_dbg(g, EXEC_CHANGE, "  Change done... Checking status");

		if (pl_link_config == 0xffffffff) {
			WARN(1, "GPU fell of PCI bus!?");

			/*
			 * The rest of the driver is probably about to
			 * explode...
			 */
			BUG();
		}

		link_control_status =
			g->ops.xve.xve_readl(g, xve_link_control_status_r());
		xv_sc_dbg(g, EXEC_CHANGE, "  target %d vs current %d",
			  link_speed_setting,
			  xve_link_control_status_link_speed_v(link_control_status));

		if (err_status == -ETIMEDOUT) {
			xv_sc_dbg(g, EXEC_CHANGE, "  Oops timed out?");
			break;
		}
	} while (attempts-- > 0 &&
		 link_speed_setting !=
		 xve_link_control_status_link_speed_v(link_control_status));

	xv_sc_dbg(g, EXEC_VERIF, "Verifying speed change...");

	/*
	 * Check that the new link speed is actually active. If we failed to
	 * change to the new link speed then return to the link speed setting
	 * pre-speed change.
	 */
	new_link_speed = xve_link_control_status_link_speed_v(
		link_control_status);
	if (link_speed_setting != new_link_speed) {
		u32 link_config = gk20a_readl(g, xp_pl_link_config_r(0));

		xv_sc_dbg(g, EXEC_VERIF, "  Current and target speeds mismatch!");
		xv_sc_dbg(g, EXEC_VERIF, "    LINK_CONTROL_STATUS: 0x%08x",
			  g->ops.xve.xve_readl(g, xve_link_control_status_r()));
		xv_sc_dbg(g, EXEC_VERIF, "    Link speed is %s - should be %s",
			  xve_speed_to_str(new_link_speed),
			  xve_speed_to_str(link_speed_setting));

		link_config &= ~xp_pl_link_config_max_link_rate_m();
		if (new_link_speed ==
		    xve_link_control_status_link_speed_link_speed_2p5_v()) {
			link_config |= xp_pl_link_config_max_link_rate_f(
				xp_pl_link_config_max_link_rate_2500_mtps_v());
		} else if (new_link_speed ==
			 xve_link_control_status_link_speed_link_speed_5p0_v()) {
			link_config |= xp_pl_link_config_max_link_rate_f(
				xp_pl_link_config_max_link_rate_5000_mtps_v());
		} else if (new_link_speed ==
			 xve_link_control_status_link_speed_link_speed_8p0_v()) {
			link_config |= xp_pl_link_config_max_link_rate_f(
				xp_pl_link_config_max_link_rate_8000_mtps_v());
		} else {
			link_config |= xp_pl_link_config_max_link_rate_f(
				xp_pl_link_config_max_link_rate_2500_mtps_v());
		}

		gk20a_writel(g, xp_pl_link_config_r(0), link_config);
		err_status = -ENODEV;
	} else {
		xv_sc_dbg(g, EXEC_VERIF, "  Current and target speeds match!");
		err_status = 0;
	}

done:
	/* Restore safe timings. */
	xv_sc_dbg(g, CLEANUP, "Restoring saved DL settings...");
	gk20a_writel(g, xp_dl_mgr_r(0), saved_dl_mgr);
	xv_sc_dbg(g, CLEANUP, "  Done");

	xv_sc_dbg(g, CLEANUP, "Re-enabling ASPM settings...");
	enable_aspm_gp106(g);
	xv_sc_dbg(g, CLEANUP, "  Done");

	return err_status;
}

/**
 * Sets the PCIe link speed to @xve_link_speed which must be one of:
 *
 *   %GPU_XVE_SPEED_2P5
 *   %GPU_XVE_SPEED_5P0
 *   %GPU_XVE_SPEED_8P0
 *
 * If an error is encountered an appropriate error will be returned.
 */
int xve_set_speed_gp106(struct gk20a *g, u32 next_link_speed)
{
	u32 current_link_speed;
	int err;

	if ((next_link_speed & GPU_XVE_SPEED_MASK) == 0) {
		return -EINVAL;
	}

	err = g->ops.xve.get_speed(g, &current_link_speed);
	if (err) {
		return err;
	}

	/* No-op. */
	if (current_link_speed == next_link_speed) {
		return 0;
	}

	return __do_xve_set_speed_gp106(g, next_link_speed);
}

/**
 * Places a bitmask of available speeds for gp106 in @speed_mask.
 */
void xve_available_speeds_gp106(struct gk20a *g, u32 *speed_mask)
{
	*speed_mask = GPU_XVE_SPEED_2P5 | GPU_XVE_SPEED_5P0;
}

#if defined(CONFIG_PCI_MSI)
void xve_rearm_msi_gp106(struct gk20a *g)
{
	/* We just need to write a dummy val in the CYA_2 offset */
	g->ops.xve.xve_writel(g, xve_cya_2_r(), 0);
}
#endif

void xve_enable_shadow_rom_gp106(struct gk20a *g)
{
	g->ops.xve.xve_writel(g, xve_rom_ctrl_r(),
			xve_rom_ctrl_rom_shadow_enabled_f());
}

void xve_disable_shadow_rom_gp106(struct gk20a *g)
{
	g->ops.xve.xve_writel(g, xve_rom_ctrl_r(),
			xve_rom_ctrl_rom_shadow_disabled_f());
}

u32 xve_get_link_control_status(struct gk20a *g)
{
	return g->ops.xve.xve_readl(g, xve_link_control_status_r());
}
