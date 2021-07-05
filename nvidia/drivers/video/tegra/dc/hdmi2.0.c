/*
 * hdmi2.0.c: hdmi2.0 driver.
 *
 * Copyright (c) 2014-2020, NVIDIA CORPORATION, All rights reserved.
 * Author: Animesh Kishore <ankishore@nvidia.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/clk/tegra.h>
#include <linux/nvhost.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/unistd.h>
#include <linux/extcon/extcon-disp.h>
#include <linux/extcon.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <soc/tegra/tegra_powergate.h>
#include <uapi/video/tegra_dc_ext.h>

#include "dc.h"
#include "dc_reg.h"
#include "dc_priv.h"
#include "sor.h"
#include "sor_regs.h"
#include "edid.h"
#include "hdmi2.0.h"
#include "hdcp/hdmihdcp.h"
#include "dpaux.h"
#include "hda_dc.h"
#include "hdmivrr.h"

#include <linux/tegra_prod.h>
#include "../../../../arch/arm/mach-tegra/iomap.h"

#include "bridge/hdmi2fpd_ds90uh949.h"
#include "bridge/max929x_hdmi2gmsl.h"
#include "bridge/hdmi2dsi_tc358870.h"

static struct tmds_prod_pair tmds_config_modes[] = {
	{ /* 54 MHz */
	.clk = 54000000,
	.name = "prod_c_54M"
	},
	{ /* 75 MHz */
	.clk = 75000000,
	.name = "prod_c_75M"
	},
	{ /* 150 MHz */
	.clk = 150000000,
	.name = "prod_c_150M"
	},
	{ /* 200 MHz */
	.clk = 200000000,
	.name = "prod_c_200M"
	},
	{ /* 300 MHz */
	.clk = 300000000,
	.name = "prod_c_300M"
	},
	{ /* HDMI 2.0 */
	.clk = 600000000,
	.name = "prod_c_600M"
	},
	{ /* end mark */
	.clk = 0,
	}
};

static int hdmi_instance;

static int tegra_hdmi_controller_enable(struct tegra_hdmi *hdmi);
static void tegra_hdmi_config_clk(struct tegra_hdmi *hdmi, u32 clk_type);
static long tegra_dc_hdmi_setup_clk(struct tegra_dc *dc, struct clk *clk);
static void tegra_hdmi_scdc_worker(struct work_struct *work);
static void tegra_hdmi_debugfs_init(struct tegra_hdmi *hdmi);
static void tegra_hdmi_debugfs_remove(struct tegra_hdmi *hdmi);
static void tegra_hdmi_hdr_worker(struct work_struct *work);
static int tegra_hdmi_v2_x_mon_config(struct tegra_hdmi *hdmi, bool enable);
static void tegra_hdmi_v2_x_host_config(struct tegra_hdmi *hdmi, bool enable);

static inline bool tegra_hdmi_is_connected(struct tegra_hdmi *hdmi)
{
	return (hdmi->mon_spec.misc & FB_MISC_HDMI) ||
		(hdmi->mon_spec.misc & FB_MISC_HDMI_FORUM);
}

static inline void __maybe_unused
tegra_hdmi_irq_enable(struct tegra_hdmi *hdmi)
{
	enable_irq(hdmi->irq);
}

static inline void __maybe_unused
tegra_hdmi_irq_disable(struct tegra_hdmi *hdmi)
{
	disable_irq(hdmi->irq);
}

static inline bool __maybe_unused
tegra_hdmi_hpd_asserted(struct tegra_hdmi *hdmi)
{
	return tegra_dc_hpd(hdmi->dc);
}

/*
 * Calculates the effective SOR link rate based on the DC pclk and the selected
 * output mode for native support:
 * - RGB444/YUV444 12bpc requires a 2:3 pclk:orclk ratio
 * - YUV420 8bpc requires a 2:1 pclk:orclk ratio
 */
static inline unsigned long tegra_sor_get_link_rate(struct tegra_dc *dc)
{
	int yuv_flag = dc->mode.vmode & FB_VMODE_YUV_MASK;
	unsigned long rate = dc->mode.pclk;

	if (tegra_dc_is_t21x())
		return rate;

	if (!(dc->mode.vmode & FB_VMODE_BYPASS)) {
		if ((IS_RGB(yuv_flag) && (yuv_flag == FB_VMODE_Y36)) ||
			(yuv_flag == (FB_VMODE_Y444 | FB_VMODE_Y36))) {
			rate = rate >> 1;
			rate = rate * 3;
		} else if (tegra_dc_is_yuv420_8bpc(&dc->mode)) {
			rate = rate >> 1;
		}
	}

	return rate;
}

static inline void _tegra_hdmi_ddc_enable(struct tegra_hdmi *hdmi)
{
	mutex_lock(&hdmi->ddc_refcount_lock);
	if (hdmi->ddc_refcount++)
		goto fail;
	tegra_hdmi_get(hdmi->dc);
	mutex_lock(&hdmi->dpaux->lock);
	tegra_dpaux_get(hdmi->dpaux);
	mutex_unlock(&hdmi->dpaux->lock);
	/*
	 * hdmi uses i2c lane muxed on dpaux1 pad.
	 * Enable dpaux1 pads and configure the mux.
	 */
	tegra_dpaux_config_pad_mode(hdmi->dpaux, TEGRA_DPAUX_PAD_MODE_I2C);

fail:
	mutex_unlock(&hdmi->ddc_refcount_lock);
}

static inline void _tegra_hdmi_ddc_disable(struct tegra_hdmi *hdmi)
{
	mutex_lock(&hdmi->ddc_refcount_lock);

	if (WARN_ONCE(hdmi->ddc_refcount <= 0, "ddc refcount imbalance"))
		goto fail;
	if (--hdmi->ddc_refcount != 0)
		goto fail;

	/*
	 * hdmi uses i2c lane muxed on dpaux1 pad.
	 * Disable dpaux1 pads.
	 */
	tegra_dpaux_pad_power(hdmi->dpaux, false);
	mutex_lock(&hdmi->dpaux->lock);
	tegra_dpaux_put(hdmi->dpaux);
	mutex_unlock(&hdmi->dpaux->lock);
	tegra_hdmi_put(hdmi->dc);

fail:
	mutex_unlock(&hdmi->ddc_refcount_lock);
}

static int tegra_hdmi_ddc_i2c_xfer(struct tegra_dc *dc,
					struct i2c_msg *msgs, int num)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);
	int ret;

	/* No physical panel and/or emulator is attached in simulation. */
	if (tegra_platform_is_sim())
		return -EINVAL;

	_tegra_hdmi_ddc_enable(hdmi);
	ret = i2c_transfer(hdmi->ddc_i2c_client->adapter, msgs, num);
	_tegra_hdmi_ddc_disable(hdmi);
	return ret;
}

static int tegra_hdmi_ddc_init(struct tegra_hdmi *hdmi)
{
	struct tegra_dc *dc = hdmi->dc;
	struct i2c_adapter *i2c_adap;
	int err = 0;
	struct i2c_board_info i2c_dev_info = {
		.type = "tegra_hdmi2.0",
		.addr = 0x50,
	};

	if (hdmi->edid_src == EDID_SRC_PANEL)
		hdmi->edid = tegra_edid_create(dc, tegra_hdmi_ddc_i2c_xfer);
	else if (hdmi->edid_src == EDID_SRC_DT)
		hdmi->edid = tegra_edid_create(dc, tegra_dc_edid_blob);
	if (IS_ERR_OR_NULL(hdmi->edid)) {
		dev_err(&dc->ndev->dev, "hdmi: can't create edid\n");
		return PTR_ERR(hdmi->edid);
	}
	tegra_dc_set_edid(dc, hdmi->edid);

	if (tegra_platform_is_sim())
		return 0;

	i2c_adap = i2c_get_adapter(dc->out->ddc_bus);
	if (i2c_adap) {
		hdmi->ddc_i2c_original_rate =
			i2c_get_adapter_bus_clk_rate(i2c_adap);

		hdmi->ddc_i2c_client = i2c_new_device(i2c_adap, &i2c_dev_info);
		i2c_put_adapter(i2c_adap);
		if (!hdmi->ddc_i2c_client) {
			dev_err(&dc->ndev->dev, "hdmi: can't create new i2c device\n");
			err = -EBUSY;
			goto fail_edid_free;
		}
	} else if (hdmi->edid_src != EDID_SRC_DT) {
		dev_err(&dc->ndev->dev,
			"hdmi: can't get adpater for ddc bus %d\n",
			dc->out->ddc_bus);
		err = -EBUSY;
		goto fail_edid_free;
	}

	return 0;
fail_edid_free:
	tegra_edid_destroy(hdmi->edid);
	return err;
}

static int tegra_hdmi_scdc_i2c_xfer(struct tegra_dc *dc,
					struct i2c_msg *msgs, int num)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);

	return i2c_transfer(hdmi->scdc_i2c_client->adapter, msgs, num);
}

static int tegra_hdmi_scdc_init(struct tegra_hdmi *hdmi)
{
	struct tegra_dc *dc = hdmi->dc;
	struct i2c_adapter *i2c_adap;
	int err = 0;
	struct i2c_board_info i2c_dev_info = {
		.type = "tegra_hdmi_scdc",
		.addr = 0x54,
	};

	if (tegra_platform_is_sim())
		return 0;

	i2c_adap = i2c_get_adapter(dc->out->ddc_bus);
	if (!i2c_adap) {
		dev_err(&dc->ndev->dev,
			"hdmi: can't get adpater for scdc bus %d\n",
			dc->out->ddc_bus);
		err = -EBUSY;
		goto fail;
	}

	hdmi->scdc_i2c_client = i2c_new_device(i2c_adap, &i2c_dev_info);
	i2c_put_adapter(i2c_adap);
	if (!hdmi->scdc_i2c_client) {
		dev_err(&dc->ndev->dev,
			"hdmi: can't create scdc i2c device\n");
		err = -EBUSY;
		goto fail;
	}

	INIT_DELAYED_WORK(&hdmi->scdc_work, tegra_hdmi_scdc_worker);

	return 0;
fail:
	return err;
}

/*  does not return precise tmds character rate */
static u32 tegra_hdmi_mode_min_tmds_rate(const struct fb_videomode *mode)
{
	u32 tmds_csc_8bpc_khz = PICOS2KHZ(mode->pixclock);

	if (mode->vmode & (FB_VMODE_Y420 | FB_VMODE_Y420_ONLY))
		tmds_csc_8bpc_khz /= 2;

	return tmds_csc_8bpc_khz;
}

static bool tegra_hdmi_fb_mode_filter(const struct tegra_dc *dc,
					struct fb_videomode *mode)
{
	struct tegra_hdmi *hdmi = dc->out_data;

	if (!mode->pixclock)
		return false;

	if (mode->xres > 4096 || mode->yres > 2160)
		return false;

#if defined(CONFIG_TEGRA_YUV_BYPASS_MODE_FILTER)
	if (tegra_dc_is_t21x()) {
		/* No support for YUV modes on T210 hardware. Filter them out */
		if (mode->vmode & FB_VMODE_YUV_MASK)
			return false;
	} else {
		/* T186 hardware supports only YUV422.
		 * Filter out YUV420 modes.
		 */
		if ((mode->vmode & FB_VMODE_Y420_ONLY) ||
				(mode->vmode & FB_VMODE_Y420))
			return false;
	}
#endif

	if (mode->vmode & FB_VMODE_INTERLACED)
		return false;

	/* some non-compliant edids list 420vdb modes in vdb */
	if ((mode->vmode & FB_VMODE_Y420) &&
		!(mode->flag & FB_MODE_IS_FROM_VAR) &&
		!(tegra_edid_is_hfvsdb_present(hdmi->edid) &&
		tegra_edid_is_scdc_present(hdmi->edid)) &&
		tegra_edid_is_420db_present(hdmi->edid)) {
		mode->vmode &= ~FB_VMODE_Y420;
		mode->vmode |= FB_VMODE_Y420_ONLY;
	}

	if ((mode->vmode & FB_VMODE_YUV_MASK) &&
		!(mode->flag & FB_MODE_IS_FROM_VAR) &&
		(tegra_edid_get_quirks(hdmi->edid) & TEGRA_EDID_QUIRK_NO_YUV))
		return false;

	if (!(mode->vmode & FB_VMODE_IS_CEA) &&
		!(mode->flag & FB_MODE_IS_FROM_VAR) &&
		(tegra_edid_get_quirks(hdmi->edid) & TEGRA_EDID_QUIRK_ONLY_CEA))
		return false;

	/*
	 * There are currently many TVs in the market that actually do NOT support
	 * 4k@60fps 4:4:4 (594 MHz), (especially on the HDCP 2.2 ports), but
	 * advertise it in the DTD block in their EDIDs. The workaround for this port
	 * is to disable the 594 MHz mode if no HF-VSDB is present or if no SCDC
	 * support is indicated
	 */
	if (((tegra_hdmi_mode_min_tmds_rate(mode) / 1000 >= 340) &&
		!(mode->flag & FB_MODE_IS_FROM_VAR)) &&
		(!tegra_edid_is_hfvsdb_present(hdmi->edid) ||
		!tegra_edid_is_scdc_present(hdmi->edid)))
		return false;

	/* Check if the mode's pixel clock is more than the max rate*/
	if (!tegra_dc_valid_pixclock(dc, mode))
		return false;

	/*
	 * Workaround for modes that fail the constraint:
	 * V_FRONT_PORCH >= V_REF_TO_SYNC + 1
	 *
	 * This constraint does not apply to nvdisplay.
	 */
	if (!tegra_dc_is_nvdisplay() && mode->lower_margin == 1) {
		mode->lower_margin++;
		mode->upper_margin--;
		mode->vmode |= FB_VMODE_ADJUSTED;
	}

	if (!check_fb_videomode_timings(dc, mode)) {
#if defined(CONFIG_TEGRA_DC_TRACE_PRINTK)
		trace_printk("check_fb_videomode_timings: false\n"
			     "%u x %u @ %u Hz\n",
			     mode->xres, mode->yres, mode->pixclock);
#endif
		return false;
	}

	return true;
}

static int tegra_hdmi_get_mon_spec(struct tegra_hdmi *hdmi)
{
#define MAX_RETRY 10
#define MIN_RETRY_DELAY_US 200
#define MAX_RETRY_DELAY_US (MIN_RETRY_DELAY_US + 200)

	size_t attempt_cnt = 0;
	int err = 0;
	struct i2c_adapter *i2c_adap = i2c_get_adapter(hdmi->dc->out->ddc_bus);

	if (IS_ERR_OR_NULL(hdmi->edid)) {
		dev_err(&hdmi->dc->ndev->dev, "hdmi: edid not initialized\n");
		return PTR_ERR(hdmi->edid);
	}

	tegra_edid_i2c_adap_change_rate(i2c_adap, hdmi->ddc_i2c_original_rate);

	hdmi->mon_spec_valid = false;
	if (hdmi->mon_spec_valid)
		fb_destroy_modedb(hdmi->mon_spec.modedb);
	memset(&hdmi->mon_spec, 0, sizeof(hdmi->mon_spec));

	do {
		err = tegra_edid_get_monspecs(hdmi->edid, &hdmi->mon_spec);
		if (err < 0)
			usleep_range(MIN_RETRY_DELAY_US, MAX_RETRY_DELAY_US);
		else
			break;
	} while (++attempt_cnt < MAX_RETRY);

	if (err < 0) {
		dev_err(&hdmi->dc->ndev->dev, "hdmi: edid read failed\n");
		/* Try to load and parse the fallback edid */
		hdmi->edid->errors = EDID_ERRORS_READ_FAILED;
		err = tegra_edid_get_monspecs(hdmi->edid, &hdmi->mon_spec);
		if (err < 0) {
			dev_err(&hdmi->dc->ndev->dev,
				"hdmi: parsing fallback edid failed\n");
			return err;
		}
		dev_info(&hdmi->dc->ndev->dev, "hdmi: using fallback edid\n");
	}

	hdmi->mon_spec_valid = true;
	return 0;

#undef MAX_RETRY_DELAY_US
#undef MIN_RETRY_DELAY_US
#undef MAX_RETRY
}

static inline int tegra_hdmi_edid_read(struct tegra_hdmi *hdmi)
{
	int err;

	err = tegra_hdmi_get_mon_spec(hdmi);

	return err;
}

static int tegra_hdmi_get_eld(struct tegra_hdmi *hdmi)
{
	int err;

	hdmi->eld_valid = false;
	memset(&hdmi->eld, 0, sizeof(hdmi->eld));

	err = tegra_edid_get_eld(hdmi->edid, &hdmi->eld);
	if (err < 0) {
		dev_err(&hdmi->dc->ndev->dev, "hdmi: eld not available\n");
		return err;
	}

	hdmi->eld_valid = true;
	return 0;
}

static inline int tegra_hdmi_eld_read(struct tegra_hdmi *hdmi)
{
	return tegra_hdmi_get_eld(hdmi);
}

static void tegra_hdmi_edid_config(struct tegra_hdmi *hdmi)
{
#define CM_TO_MM(x) (x * 10)

	struct tegra_dc *dc = hdmi->dc;

	if (!hdmi->mon_spec_valid)
		return;

	dc->out->h_size = CM_TO_MM(hdmi->mon_spec.max_x);
	dc->out->v_size = CM_TO_MM(hdmi->mon_spec.max_y);
	hdmi->dvi = !tegra_hdmi_is_connected(hdmi);

#undef CM_TO_MM
}

static void tegra_hdmi_hotplug_notify(struct tegra_hdmi *hdmi,
					bool is_asserted)
{
	struct tegra_dc *dc = hdmi->dc;
	struct fb_monspecs *mon_spec;
	int n_display_timings, idx;

	if (is_asserted)
		mon_spec = &hdmi->mon_spec;
	else
		mon_spec = NULL;

	n_display_timings = 0;
	/*
	 * If display timing with non-zero pclk is specified in DT,
	 * skip parsing EDID from monitor. Except if vedid is active.
	 * In that case force parsing the EDID
	 */
	for (idx = 0; idx < dc->out->n_modes; idx++) {
		if (0 != dc->out->modes->pclk) {
			n_display_timings++;
			break;
		}
	}

	if (dc->vedid) {
		n_display_timings = 0;
	}

	if (dc->fb && 0 == n_display_timings) {
		tegra_fb_update_monspecs(hdmi->dc->fb, mon_spec,
					tegra_hdmi_fb_mode_filter);
		tegra_fb_update_fix(hdmi->dc->fb, mon_spec);
	}

	dc->connected = is_asserted;
	tegra_dc_ext_process_hotplug(dc->ndev->id);

	tegra_dc_extcon_hpd_notify(dc);
#ifdef CONFIG_SWITCH
	switch_set_state(&hdmi->hpd_switch, is_asserted ? 1 : 0);
#endif
}

static int tegra_hdmi_edid_eld_setup(struct tegra_hdmi *hdmi)
{
	int err;

	tegra_unpowergate_partition(hdmi->sor->powergate_id);

	err = tegra_hdmi_edid_read(hdmi);
	if (err < 0)
		goto fail;

	err = tegra_hdmi_eld_read(hdmi);
	if (err < 0)
		goto fail;

	err = tegra_hdmivrr_setup(hdmi);
	if (err && err != -ENODEV)
		dev_err(&hdmi->dc->ndev->dev, "vrr_setup failed\n");

	/*
	 * Try to write ELD data to SOR (needed only for boot
	 * doesn't do anything during hotplug)
	 */
	tegra_hdmi_setup_hda_presence(hdmi->sor->dev_id);

	tegra_powergate_partition(hdmi->sor->powergate_id);

	tegra_hdmi_edid_config(hdmi);

	/*
	 * eld is configured when audio needs it
	 * via tegra_hdmi_edid_config()
	 */

	tegra_hdmi_hotplug_notify(hdmi, true);
	return 0;
fail:
	tegra_powergate_partition(hdmi->sor->powergate_id);
	return err;
}

static int tegra_hdmi_controller_disable(struct tegra_hdmi *hdmi)
{
	struct tegra_dc_sor_data *sor = hdmi->sor;
	struct tegra_dc *dc = hdmi->dc;
	int ret = 0;

	tegra_dc_get(dc);

	/* disable hdcp */
	if (hdmi->edid_src == EDID_SRC_PANEL && !hdmi->dc->vedid)
		tegra_nvhdcp_set_plug(hdmi->nvhdcp, false);

	cancel_delayed_work_sync(&hdmi->scdc_work);
	if (tegra_sor_get_link_rate(dc) > 340000000) {
		tegra_hdmi_v2_x_mon_config(hdmi, false);
		tegra_hdmi_v2_x_host_config(hdmi, false);
	}

	tegra_dc_sor_detach(sor);

	if (dc->out->vrr_hotplug_state == TEGRA_HPD_STATE_FORCE_DEASSERT)
		tegra_dc_disable_disp_ctrl_mode(dc);

	tegra_sor_clk_switch_setup(sor, false);
	tegra_hdmi_config_clk(hdmi, TEGRA_HDMI_SAFE_CLK);
	tegra_sor_power_lanes(sor, 4, false);
	tegra_sor_hdmi_pad_power_down(sor);
	tegra_sor_reset(hdmi->sor);
	tegra_hdmi_put(dc);

	if (tegra_dc_is_nvdisplay()) {
		ret = clk_set_parent(sor->ref_clk, dc->parent_clk_safe);
		if (ret)
			dev_err(&dc->ndev->dev,
				"can't set parent_clk_safe for sor->ref_clk\n");
	}

	cancel_delayed_work_sync(&hdmi->hdr_worker);
	tegra_dc_put(dc);

	return ret;
}

static int tegra_hdmi_disable(struct tegra_hdmi *hdmi)
{
	struct tegra_dc *dc = hdmi->dc;

	if (!hdmi->enabled) {
		dc->connected = false;
		tegra_dc_ext_process_hotplug(dc->ndev->id);
		tegra_dc_extcon_hpd_notify(dc);
#ifdef CONFIG_SWITCH
		switch_set_state(&hdmi->hpd_switch, 0);
#endif
		return 0;
	}

	hdmi->enabled = false;
	hdmi->eld_valid = false;
	hdmi->mon_spec_valid = false;

	tegra_hdmivrr_disable(hdmi);
	tegra_dc_disable(hdmi->dc);

	tegra_hdmi_hotplug_notify(hdmi, false);

	return 0;
}

static int (*tegra_hdmi_state_func[])(struct tegra_hdmi *) = {
	tegra_hdmi_disable,
	tegra_hdmi_edid_eld_setup,
};

enum tegra_hdmi_plug_states {
	TEGRA_HDMI_MONITOR_DISABLE,
	TEGRA_HDMI_MONITOR_ENABLE,
};

static int hdmi_hpd_process_edid_match(struct tegra_hdmi *hdmi, int match)
{
	int ret = 0;

	if (match) {
		if (!tegra_dc_ext_is_userspace_active()) {
			/* No userspace running. Enable DC with cached mode */
			dev_info(&hdmi->dc->ndev->dev,
			"hdmi: No EDID change. No userspace active. Using "
			"cached mode to initialize dc!\n");
			hdmi->dc->use_cached_mode = true;
			hdmi->plug_state = TEGRA_HDMI_MONITOR_ENABLE;
		} else {
			if ((hdmi->edid_src == EDID_SRC_PANEL)
					&& !hdmi->dc->vedid && hdmi->enabled) {
				tegra_nvhdcp_set_plug(hdmi->nvhdcp, false);
				tegra_nvhdcp_set_plug(hdmi->nvhdcp, true);
			}

			/*
			 * Userspace is active. No EDID change. Userspace will
			 * issue unblank call to enable DC later.
			 */
			dev_info(&hdmi->dc->ndev->dev, "hdmi: No EDID change "
			"after HPD bounce, taking no action\n");
			ret = -EINVAL;
		}
	} else {
		dev_info(&hdmi->dc->ndev->dev,
			"hdmi: EDID change after HPD bounce, resetting\n");
		hdmi->plug_state = TEGRA_HDMI_MONITOR_DISABLE;

		/*
		 * In dc resume context DC has to be in disable state if EDID
		 * is changed, setting dc->reenable_on_resume to false makes
		 * sure DC does not get enabled.
		 */
		if (hdmi->dc->suspended)
			hdmi->dc->reenable_on_resume = false;
	}

	return ret;
}

static int read_edid_into_buffer(struct tegra_hdmi *hdmi,
				 u8 *edid_data, size_t edid_data_len)
{
	int err, i;
	int extension_blocks;
	int max_ext_blocks = (edid_data_len / 128) - 1;

	err = tegra_edid_read_block(hdmi->edid, 0, edid_data);
	if (err) {
		dev_info(&hdmi->dc->ndev->dev, "hdmi: tegra_edid_read_block(0) returned err %d\n",
			err);
		return err;
	}
	extension_blocks = edid_data[0x7e];
	dev_info(&hdmi->dc->ndev->dev, "%s: extension_blocks = %d, max_ext_blocks = %d\n",
		__func__, extension_blocks, max_ext_blocks);
	if (extension_blocks > max_ext_blocks)
		extension_blocks = max_ext_blocks;
	for (i = 1; i <= extension_blocks; i++) {
		err = tegra_edid_read_block(hdmi->edid, i, edid_data + i * 128);
		if (err) {
			dev_info(&hdmi->dc->ndev->dev, "hdmi: tegra_edid_read_block(%d) returned err %d\n",
				i, err);
			return err;
		}
	}
	return i * 128;
}

static int hdmi_recheck_edid(struct tegra_hdmi *hdmi, int *match)
{
	int ret;
	u8 tmp[HDMI_EDID_MAX_LENGTH] = {0};

	if (tegra_platform_is_sim())
		return 0;
	if (hdmi->dc->vedid) {
		/* Use virtual EDID if it is present. */
		memcpy(tmp, hdmi->dc->vedid_data, EDID_BYTES_PER_BLOCK);
		ret = EDID_BYTES_PER_BLOCK;
	} else {
		ret = read_edid_into_buffer(hdmi, tmp, sizeof(tmp));
	}
	dev_info(&hdmi->dc->ndev->dev, "%s: read_edid_into_buffer() returned %d\n",
		__func__, ret);
	if (ret > 0) {
		struct tegra_dc_edid *data = tegra_edid_get_data(hdmi->edid);
		dev_info(&hdmi->dc->ndev->dev, "old edid len = %ld\n",
			(long int)data->len);
		*match = ((ret == data->len) &&
			  !memcmp(tmp, data->buf, data->len));
		if (*match == 0) {
			print_hex_dump(KERN_INFO, "tmp :", DUMP_PREFIX_ADDRESS,
				       16, 4, tmp, ret, true);
			print_hex_dump(KERN_INFO, "data:", DUMP_PREFIX_ADDRESS,
				       16, 4, data->buf, data->len, true);
		}
		tegra_edid_put_data(data);
		ret = 0;
	}

	return ret;
}
static void tegra_hdmi_hpd_worker(struct work_struct *work)
{
	struct tegra_hdmi *hdmi = container_of(to_delayed_work(work),
				struct tegra_hdmi, hpd_worker);
	int err;
	bool connected;
	enum tegra_hdmi_plug_states orig_state;
	int match = 0;

	mutex_lock(&hdmi->hpd_lock);

	connected = tegra_dc_hpd(hdmi->dc);
	orig_state = hdmi->plug_state;

	if (hdmi->dc->out->hotplug_state == TEGRA_HPD_STATE_NORMAL &&
		hdmi->dc->out->prev_hotplug_state == TEGRA_HPD_STATE_NORMAL)
			tegra_nvhdcp_clear_fallback(hdmi->nvhdcp);

	if (connected) {
		switch (orig_state) {
		case TEGRA_HDMI_MONITOR_ENABLE:
			if (hdmi_recheck_edid(hdmi, &match)) {
				dev_info(&hdmi->dc->ndev->dev, "hdmi: unable to read EDID\n");
				goto fail;
			} else {
				err = hdmi_hpd_process_edid_match(hdmi, match);
				if (err < 0)
					goto fail;
			}
			break;
		case TEGRA_HDMI_MONITOR_DISABLE:
			hdmi->plug_state = TEGRA_HDMI_MONITOR_ENABLE;
			break;
		default:
			break;
		};
	} else {
		switch (orig_state) {
		case TEGRA_HDMI_MONITOR_ENABLE:
			hdmi->plug_state = TEGRA_HDMI_MONITOR_DISABLE;
			break;
		case TEGRA_HDMI_MONITOR_DISABLE:
			goto fail;
		default:
			break;
		};
	}

	err = tegra_hdmi_state_func[hdmi->plug_state](hdmi);

	if (err < 0) {
			dev_info(&hdmi->dc->ndev->dev,
				"hdmi state %d failed during %splug\n",
				hdmi->plug_state, connected ? "" : "un");
			hdmi->plug_state = orig_state;
			goto fail;
		} else {
			dev_info(&hdmi->dc->ndev->dev, "hdmi: %splugged\n",
					connected ? "" : "un");
		}

	if (connected && hdmi->plug_state == TEGRA_HDMI_MONITOR_DISABLE)
		goto reschedule_worker;

fail:
	mutex_unlock(&hdmi->hpd_lock);
	complete(&hdmi->dc->hpd_complete);
	return;

reschedule_worker:
	mutex_unlock(&hdmi->hpd_lock);
	cancel_delayed_work(&hdmi->hpd_worker);
	schedule_delayed_work(&hdmi->hpd_worker, 0);
	return;

}

static irqreturn_t tegra_hdmi_hpd_irq_handler(int irq, void *ptr)
{
	struct tegra_dc *dc = ptr;
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);
	unsigned int hpd_debounce = HDMI_HPD_DEBOUNCE_DELAY_MS;

	if (dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return IRQ_HANDLED;

	if (atomic_read(&hdmi->suspended))
		return IRQ_HANDLED;

	tegra_nvhdcp_clear_fallback(hdmi->nvhdcp);
	cancel_delayed_work(&hdmi->hpd_worker);

	if (tegra_edid_get_quirks(hdmi->edid) &
			TEGRA_EDID_QUIRK_HPD_BOUNCE) {
		hpd_debounce = HDMI_HPD_DEBOUNCE_WAR_DELAY_MS;
	}

	schedule_delayed_work(&hdmi->hpd_worker,
				msecs_to_jiffies(hpd_debounce));

	return IRQ_HANDLED;
}

static int tegra_dc_hdmi_hpd_init(struct tegra_dc *dc)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);
	struct device_node *sor_np = NULL;
	enum of_gpio_flags flags;
	int hotplug_gpio = dc->out->hotplug_gpio;
	int hotplug_gpio_np;
	int hotplug_state;
	int hotplug_irq;
	int err;

	if (tegra_platform_is_sim())
		goto skip_gpio_irq_settings;

	hotplug_state = hdmi->dc->out->hotplug_state;

	sor_np = tegra_dc_get_conn_np(hdmi->dc);
	if (!sor_np) {
		dev_err(&dc->ndev->dev, "%s: error getting connector np\n",
			__func__);
		return -ENODEV;
	}

	hotplug_gpio_np = of_get_named_gpio_flags(sor_np,
					       "nvidia,hpd-gpio", 0, &flags);
	if (hotplug_gpio_np == -ENOENT &&
		hotplug_state == TEGRA_HPD_STATE_FORCE_ASSERT) {
		dev_info(&dc->ndev->dev,
			"hdmi: No hotplug gpio is assigned\n");
		goto skip_gpio_irq_settings;
	}

	if (!gpio_is_valid(hotplug_gpio)) {
		dev_err(&dc->ndev->dev, "hdmi: invalid hotplug gpio\n");
		return -EINVAL;
	}

	hotplug_irq = gpio_to_irq(hotplug_gpio);
	if (hotplug_irq < 0) {
		dev_err(&dc->ndev->dev,
			"hdmi: hotplug gpio to irq map failed\n");
		return -EINVAL;
	}

	err = gpio_request(hotplug_gpio, "hdmi2.0_hpd");
	if (err < 0)
		dev_err(&dc->ndev->dev,
			"hdmi: hpd gpio_request failed %d\n", err);
	gpio_direction_input(hotplug_gpio);

	err = request_threaded_irq(hotplug_irq,
				NULL, tegra_hdmi_hpd_irq_handler,
				(IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
				IRQF_ONESHOT),
				dev_name(&dc->ndev->dev), dc);
	if (err) {
		dev_err(&dc->ndev->dev,
			"hdmi: request_threaded_irq failed: %d\n", err);
		goto fail;
	}
	hdmi->irq = hotplug_irq;

	if (hotplug_state != TEGRA_HPD_STATE_NORMAL) {
		dev_info(&dc->ndev->dev,
			"hdmi: keeping hotplug irq disabled\n");
		disable_irq(hotplug_irq);
	}

skip_gpio_irq_settings:
	INIT_DELAYED_WORK(&hdmi->hpd_worker, tegra_hdmi_hpd_worker);

	mutex_init(&hdmi->hpd_lock);

	return 0;
fail:
	gpio_free(hotplug_gpio);
	return err;
}


#define  MAX_TMDS_FREQ   600000000
#define  NAME_PROD_LIST_SOC  "prod_list_hdmi_soc"
#define  NAME_PROD_LIST_PKG  "prod_list_hdmi_package"
#define  NAME_PROD_LIST_BOARD  "prod_list_hdmi_board"

struct tmds_range_info  {
	struct list_head  list;
	int  lowerHz;
	int  upperHz;
	const char  *pch_prod;
};


/*
 * Read a prod-setting list from DT
 * o Inputs:
 *  - hdmi: pointer to hdmi info
 *  - np_prod: pointer to prod-settings node
 *  - pch_prod_list: name of the DT property of prod-setting list
 *  - optional: boolean to indicate optional list or not
 *  - hd_range: pointer to linked list head for range info read
 * o Outputs:
 *  - return: pointer to a temporary memory block allocated
 *            caller should free this memory block after use of the linked
 *            list *hd_range
 *            NULL to indicate failure
 *  - *hd_range: linked list of struct tmds_range_info read
 *               the list is sorted with lower boundary value
 */
static void  *tegra_tmds_range_read(struct tegra_hdmi *hdmi,
		struct device_node *np_prod, char *pch_prod_list,
		bool optional, struct list_head *hd_range)
{
	int  err = 0;
	int  num_str;
	int  i, j, k, n;
	int  upper, lower;
	const char  *pch_prod;
	struct device_node  *np;
	LIST_HEAD(head);
	struct tmds_range_info  *ranges = NULL;
	struct tmds_range_info  *pos, *lowest;
	LIST_HEAD(head_sorted);
#define  LEN_NAME  128
	char  buf_name[LEN_NAME];

	/* sanity check */
	if (!np_prod)
		goto fail_sanity;
	num_str = of_property_count_strings(np_prod, pch_prod_list);
	if (num_str <= 0) {
		if (!optional)
			dev_info(&hdmi->dc->ndev->dev,
				"hdmi: invalid prod list %s\n",
				pch_prod_list);
		goto fail_sanity;
	}

	/* allocate single space for info array */
	ranges = kcalloc(num_str, sizeof(*ranges), GFP_KERNEL);
	if (!ranges)
		goto fail_alloc;

	/* read into the linked list */
	for (i = j = 0; i < num_str; i++) {
		err = of_property_read_string_index(np_prod,
				pch_prod_list, i, &pch_prod);
		if (err) {
			dev_warn(&hdmi->dc->ndev->dev,
				"hdmi: %s: string %d read failed, err:%d\n",
				pch_prod_list, i, err);
			continue;
		}
		if ('\0' == *pch_prod)
			continue;
		np = of_get_child_by_name(np_prod, pch_prod);
		if (np) {
			of_node_put(np);
		} else {
			dev_warn(&hdmi->dc->ndev->dev,
				"hdmi: %s: prod-setting %s is not found\n",
				pch_prod_list, pch_prod);
			continue;
		}
		ranges[j].pch_prod = pch_prod;
		strlcpy(buf_name, pch_prod, LEN_NAME);
		for (k = 0; k < strlen(buf_name); k++) {
			if ('A' <= buf_name[k] && buf_name[k] <= 'Z')
				buf_name[k] += 'a' - 'A';
			else if ('-' == buf_name[k])
				buf_name[k] = '_';
		}
		if (2 == sscanf(buf_name, "prod_c_hdmi_%dm_%dm%n",
				&lower, &upper, &n)) {
			if (!(lower < upper) || strlen(pch_prod) != n) {
				dev_warn(&hdmi->dc->ndev->dev,
					"hdmi: %s: invalid range in %s\n",
					pch_prod_list, pch_prod);
				continue;
			}
		} else {
			dev_warn(&hdmi->dc->ndev->dev,
				"hdmi: %s: missing boundary in %s\n",
				pch_prod_list, pch_prod);
			continue;
		}
		ranges[j].lowerHz = lower * 1000000;
		ranges[j].upperHz = upper * 1000000;
		list_add_tail(&ranges[j].list, &head);
		j++;
	}

	/* sort ranges */
	while (!list_empty(&head)) {
		lowest = list_first_entry(&head, typeof(*lowest), list);
		list_for_each_entry(pos, &head, list) {
			if (pos->lowerHz < lowest->lowerHz)
				lowest = pos;
		}
		list_move_tail(&lowest->list, &head_sorted);
	}
	if (!list_empty(&head_sorted))
		list_replace(&head_sorted, hd_range);

fail_alloc:
fail_sanity:
	return ranges;
#undef  LEN_NAME
}


/*
 * Construct the HDMI TMDS range prod-setting table, hdmi->tmds_range, from
 * the range list read from DeviceTree.
 *
 * o inputs:
 *  - hdmi: HDMI info
 *  - phead: head of the range list
 * o outputs:
 *  - return: 0: no error
 *            !0: error return
 *  - hdmi->tmds_range: HDMI TMDS prod-setting range table
 */
static int  tegra_tmds_range_construct(struct tegra_hdmi *hdmi,
		struct list_head *phead)
{
	int  i,  l;
	struct tmds_range_info  *pos;
	struct tmds_prod_pair  *tmds_ranges = NULL;
	char  *pch_name;

	/* allocate space for the table & names */
	i = l = 0;
	list_for_each_entry(pos, phead, list) {
		l += strlen(pos->pch_prod) + 1;
		i++;
	}
	l += sizeof(*tmds_ranges) * (i + 1);
	tmds_ranges = kzalloc(l, GFP_KERNEL);
	if (!tmds_ranges)
		return -ENOMEM;

	/* construct the TMDS range table from the list */
	pch_name = (char *)&tmds_ranges[i + 1];
	i = 0;
	list_for_each_entry(pos, phead, list) {
		tmds_ranges[i].clk = pos->upperHz;
		strcpy(pch_name, pos->pch_prod);
		tmds_ranges[i].name = pch_name;
		pch_name += strlen(pos->pch_prod) + 1;
		dev_dbg(&hdmi->dc->ndev->dev,
			"hdmi: range %dM-%dM:%s\n",
			pos->lowerHz / 1000000,
			pos->upperHz / 1000000,
			pos->pch_prod);
		i++;
	}
	tmds_ranges[i].clk = 0;  /* end mark */

	hdmi->tmds_range = tmds_ranges;
	return 0;
}


/*
 * Combine two HDMI range lists into the first list, and the second list has
 * the priority over the first for an overlapped range. This function returns
 * a temporary memory block allocated for combining. This must be freed by
 * the caller.
 *
 * o inputs:
 *  - hd_f: head of the first range list sorted in range freq.
 *  - hd_s: head of the second range list sorted in range freq.
 * o outputs:
 *  - return: !0:temporary memory block to be freed with successful return
 *            0:error return
 *  - *hd_f: combined result list
 *  - *hd_s: empty list
 */
static void  *tegra_tmds_range_combine(struct tegra_hdmi *hdmi,
		struct list_head *hd_f, struct list_head *hd_s)
{
	int  idx;
	struct tmds_range_info  *pos_f,  *tmp_f;
	struct tmds_range_info  *pos_s,  *tmp_s;
	struct tmds_range_info  *spares;
	int  num_spare;

	if (!hd_f || !hd_s)
		return NULL;

	/* allocate some spares for split ranges */
	num_spare = 0;
	list_for_each_entry(pos_s, hd_s, list)
		num_spare++;
	spares = kcalloc(num_spare, sizeof(*spares), GFP_KERNEL);
	if (!spares)
		return NULL;

	/* combine ranges */
	idx = 0;
	pos_f = list_first_entry(hd_f, typeof(*pos_f), list);
	list_for_each_entry_safe(pos_s, tmp_s, hd_s, list) {
		list_for_each_entry_safe_from(pos_f, tmp_f, hd_f, list) {
			if (pos_s->upperHz <= pos_f->lowerHz) {
				break;
			} else if (pos_s->lowerHz <= pos_f->lowerHz &&
					pos_f->lowerHz < pos_s->upperHz &&
					pos_s->upperHz < pos_f->upperHz) {
				pos_f->lowerHz = pos_s->upperHz;
				break;
			} else if (pos_s->lowerHz <= pos_f->lowerHz &&
					pos_f->upperHz <= pos_s->upperHz) {
				list_del(&pos_f->list);
			} else if (pos_f->lowerHz < pos_s->lowerHz &&
					pos_s->upperHz < pos_f->upperHz) {
				if (idx < num_spare) {
					spares[idx].lowerHz = pos_f->lowerHz;
					spares[idx].upperHz = pos_s->lowerHz;
					spares[idx].pch_prod = pos_f->pch_prod;
					list_add_tail(&spares[idx].list,
							&pos_f->list);
					idx++;
					pos_f->lowerHz = pos_s->upperHz;
				} else {
					dev_err(&hdmi->dc->ndev->dev,
						"hdmi: %s: out of spare!\n",
						__func__);
				}
				break;
			} else if (pos_f->lowerHz < pos_s->lowerHz &&
					pos_s->lowerHz < pos_f->upperHz &&
					pos_f->upperHz <= pos_s->upperHz) {
				pos_f->upperHz = pos_s->lowerHz;
			}
		}
		list_move_tail(&pos_s->list, &pos_f->list);
	}
	return spares;
}


/*
 * Check the prod-setting list covers the full TMDS spectrum or not
 *
 * o inputs:
 *  - phead: head of the prod-setting list
 *  - max_freq: maximum frequency of the TMDS spectrum in Hz
 * o outputs:
 *  - return: true: covers the full spectrum
 *            false: does not cover the full spectrum
 */
static bool  tegra_tmds_range_check_full_spectrum(
			struct list_head *phead, int max_freq)
{
	int  i;
	bool  has_gap;
	struct tmds_range_info  *pos;

	/* check the level covers the full range or not */
	has_gap = false;
	i = 0;
	list_for_each_entry(pos, phead, list) {
		if (i < pos->lowerHz) {
			has_gap = true;
			break;
		}
		i = pos->upperHz;
	}
	if (!has_gap && i < max_freq)
		has_gap = true;

	return has_gap ? false : true;
}


/*
 * Read the HDMI TMDS range list in all levels from the DeviceTree
 * and construct the HDMI TMDS range table, hdmi->tmds_range.
 *
 * o inputs:
 *  - hdmi: HDMI info
 *  - np_prod: valid DT node of this HDMI interface prod-setting
 *  - phead_board: head of the board level list read
 * o outputs:
 *  - return:
 *   . 0: succeeded
 *   . !0: failed
 *  - hdmi->tmds_range: constructed TMDS prod-setting range table
 */
static int  tegra_tmds_range_read_all(struct tegra_hdmi *hdmi,
			struct device_node *np_prod,
			struct list_head *phead_board)
{
	int  err = 0;
	LIST_HEAD(head_range_soc);
	LIST_HEAD(head_range_pkg);
	struct list_head  *phead_range_board;
	void  *pmem_soc = NULL,  *pmem_pkg = NULL;
	void  *pmem_pkg2 = NULL,  *pmem_board2 = NULL;

	/* bring the board level list read */
	phead_range_board = phead_board;

	/* read the SoC level list */
	pmem_soc = tegra_tmds_range_read(hdmi, np_prod,
			NAME_PROD_LIST_SOC, false, &head_range_soc);
	if (!pmem_soc) {
		dev_warn(&hdmi->dc->ndev->dev,
			"hdmi: tegra_hdmi_tmds_range_read(soc) failed\n");
		err = -EINVAL;
		goto fail_range_read_soc;
	}
	/* check the SoC level covers the full spectrum */
	if (!tegra_tmds_range_check_full_spectrum(&head_range_soc,
			MAX_TMDS_FREQ))
		dev_warn(&hdmi->dc->ndev->dev,
			"hdmi: SoC prod-setting covers partial!\n");

	/* read the optional SoC-Package level */
	pmem_pkg = tegra_tmds_range_read(hdmi, np_prod,
			NAME_PROD_LIST_PKG, true, &head_range_pkg);

	/* combine the SoC and the SoC-Package level list */
	if (pmem_pkg) {
		pmem_pkg2 = tegra_tmds_range_combine(hdmi,
				&head_range_soc, &head_range_pkg);
		if (!pmem_pkg2) {
			dev_warn(&hdmi->dc->ndev->dev,
				"hdmi: combining SoC & Pkg level failed\n");
			err = -ENOMEM;
			goto fall_combine_pkg;
		}
	}
	/* combine the board level as well */
	pmem_board2 = tegra_tmds_range_combine(hdmi,
			&head_range_soc, phead_range_board);
	if (!pmem_board2) {
		dev_warn(&hdmi->dc->ndev->dev,
			"hdmi: combining SoC & Board level failed\n");
		err = -ENOMEM;
		goto fall_combine_board;
	}

	/* construct TMDS ranges from the combined list */
	err = tegra_tmds_range_construct(hdmi, &head_range_soc);
	if (err) {
		dev_warn(&hdmi->dc->ndev->dev,
			"hdmi: tegra_hdmi_tmds_range_construct() failed\n");
		goto fail_construct;
	}
	err = 0;

fail_construct:
	kfree(pmem_board2);
fall_combine_board:
	kfree(pmem_pkg2);
fall_combine_pkg:
	kfree(pmem_pkg);
	kfree(pmem_soc);
fail_range_read_soc:
	return err;
}


/*
 * Read the HDMI TMDS range list in the board level from the DeviceTree
 * property, prod_list_hdmi_board, and construct the HDMI TMDS range table,
 * hdmi->tmds_range. If the board level range list does not cover the full
 * spectrum, then return with error to combine with lower levels. And, the
 * read info from DT will be saved for a reuse.
 *
 * o inputs:
 *  - hdmi: HDMI info
 *  - np_prod: valid DT node of this HDMI interface prod-setting
 *  - phead_board: empty list head for board level list
 *  - ppmem_board: double pointer to save the temporary memory allocation
 * o outputs:
 *  - return:
 *   . 0: succeeded
 *   . !0: failed
 *  - hdmi->tmds_range: constructed TMDS prod-setting range table
 *  - *phead_board: head of the board level list read
 *  - *ppmem_board: temporary memory block to be freed
 */
static int  tegra_tmds_range_read_board(struct tegra_hdmi *hdmi,
			struct device_node *np_prod,
			struct list_head *phead_board,
			void **ppmem_board)
{
	int  err = 0;
	void  *ptmp = NULL;

	/* read the board level list */
	ptmp = tegra_tmds_range_read(hdmi, np_prod,
			NAME_PROD_LIST_BOARD, false, phead_board);
	if (!ptmp) {
		dev_info(&hdmi->dc->ndev->dev,
			"hdmi: tegra_hdmi_tmds_range_read(bd) failed\n");
		err = -EINVAL;
		return err;
	}
	*ppmem_board = ptmp;

	/* check the board level covers the full spectrum or not */
	if (!tegra_tmds_range_check_full_spectrum(phead_board,
			MAX_TMDS_FREQ)) {
		/* partial only */
		dev_info(&hdmi->dc->ndev->dev,
			"hdmi: board prod-setting covers partial!\n");
		return -ENODATA;
	}

	/* construct TMDS ranges from the list read */
	err = tegra_tmds_range_construct(hdmi, phead_board);
	if (err) {
		dev_warn(&hdmi->dc->ndev->dev,
			"hdmi: tegra_hdmi_tmds_range_construct(bd) failed\n");
		return err;
	}

	return 0;
}


/*
 * Initialize the HDMI TMDS range from the DeviceTree prod-settings.
 * HDMI has 3 levels of prod-setting list in SoC, SoC-package and board level,
 * while the board level has the highest priority and the SoC-Package level is
 * optional, DeviceTree properties prod_list_hdmi_soc, prod_list_hdmi_package
 * and prod_list_hdmi_board represent each level of prod-setting list. These
 * lists hold DeviceTree node names of prod-setting data for each range. Each
 * range's upper and lower boundary frequencies will be decoded from the
 * prod-setting data node name, prod_c_hdmi_LLm_UUm, in the list.
 * If the board level list covers the whole spectrum then there is no need to
 * combine ranges with lower levels. If the board level covers partial only
 * then the board level info read will be saved for a reuse.
 *
 * o inputs:
 *  - hdmi: HDMI info
 *  - np_prod: DT node of prod-setting for this HDMI interface
 * o outputs:
 *  - return: error code
 *  - hdmi->tmds_range: pointer to TMDS range table constructed
 *                      NULL then the default fall-back table will be used
 */
static int  tegra_hdmi_tmds_range_init(struct tegra_hdmi *hdmi,
			struct device_node *np_prod)
{
	int  err = 0;
	LIST_HEAD(head_board);
	void  *pmem_board = NULL;

	/* sanity check */
	if (!np_prod)
		return -EINVAL;

	/* build ranges from the board level list first
	 * if it fails then build ranges by combining all 3 levels */
	hdmi->tmds_range = NULL;
	err = tegra_tmds_range_read_board(hdmi, np_prod,
			&head_board, &pmem_board);
	if (err)
		err = tegra_tmds_range_read_all(hdmi, np_prod, &head_board);

	/* clean-up */
	kfree(pmem_board);
	return err;
}

#undef  MAX_TMDS_FREQ
#undef  NAME_PROD_LIST_SOC
#undef  NAME_PROD_LIST_PKG
#undef  NAME_PROD_LIST_BOARD


static int tegra_hdmi_tmds_init(struct tegra_hdmi *hdmi)
{
	int retval = 0;
	struct tegra_dc *dc = hdmi->dc;
	struct device_node *np_sor;
	struct device_node *np_ps;

	if (tegra_platform_is_sim())
		return 0;

	np_sor = tegra_dc_get_conn_np(dc);
	if (!np_sor) {
		dev_warn(&hdmi->dc->ndev->dev,
			"hdmi: error getting connector np\n");
		retval = -EINVAL;
		goto fail;
	}

	hdmi->prod_list = devm_tegra_prod_get_from_node(&hdmi->dc->ndev->dev,
							np_sor);
	if (IS_ERR(hdmi->prod_list)) {
		hdmi->prod_list = NULL;
		dev_warn(&hdmi->dc->ndev->dev,
			"hdmi: prod list init failed with error %ld\n",
			PTR_ERR(hdmi->prod_list));
		retval = -EINVAL;
		goto fail;
	}

	/* construct the HDMI TMDS range */
	np_ps = of_get_child_by_name(np_sor, "prod-settings");
	retval = tegra_hdmi_tmds_range_init(hdmi, np_ps);
	if (retval) {
		dev_warn(&hdmi->dc->ndev->dev,
			"hdmi: TMDS range init failed with error %d\n",
			retval);
		if (np_ps)
			of_node_put(np_ps);
		retval = -EINVAL;
		goto fail;
	}
	if (np_ps)
		of_node_put(np_ps);

fail:
	return retval;
}

static int tegra_hdmi_config_tmds(struct tegra_hdmi *hdmi)
{
	int i;
	int err = 0;
	struct tmds_prod_pair  *ranges;
	unsigned long tmds_rate = tegra_sor_get_link_rate(hdmi->dc);

	if (tegra_platform_is_sim())
		return 0;

	/* Apply the range where tmds rate belongs to */
	ranges = hdmi->tmds_range ? : tmds_config_modes;
	for (i = 0; ranges[i].clk; i++) {
		if (ranges[i].clk < tmds_rate)
			continue;

		dev_info(&hdmi->dc->ndev->dev,
			"hdmi: tmds rate:%luK prod-setting:%s\n",
			tmds_rate / 1000, ranges[i].name);
		err = tegra_prod_set_by_name(&hdmi->sor->base,
			ranges[i].name, hdmi->prod_list);
		/* Return if matching range found */
		if (!err)
			return 0;
	}
	/* apply highest range if no covered range found */
	if (i)
		tegra_prod_set_by_name(&hdmi->sor->base,
			ranges[i - 1].name, hdmi->prod_list);

	dev_warn(&hdmi->dc->ndev->dev,
		"hdmi: tmds prod set for tmds rate:%lu failed\n",
		tmds_rate);
	return -EINVAL;
}

static int tegra_hdmi_hdr_init(struct tegra_hdmi *hdmi)
{
	INIT_DELAYED_WORK(&hdmi->hdr_worker, tegra_hdmi_hdr_worker);
	return 0;
}

static int tegra_dc_hdmi_init(struct tegra_dc *dc)
{
	int err;
	struct device_node *sor_np, *panel_np;
	struct tegra_hdmi *hdmi;

	sor_np = tegra_dc_get_conn_np(dc);
	if (!sor_np) {
		dev_err(&dc->ndev->dev, "%s: error getting connector np\n",
			__func__);
		return -ENODEV;
	}

	panel_np = tegra_dc_get_panel_np(dc);
	if (!panel_np) {
		dev_err(&dc->ndev->dev, "%s: error getting panel np\n",
			__func__);
		return -ENODEV;
	}

	hdmi = devm_kzalloc(&dc->ndev->dev, sizeof(*hdmi), GFP_KERNEL);
	if (!hdmi) {
		err = -ENOMEM;
		goto fail_sor_np;
	}

	hdmi->hpd_switch_name = devm_kzalloc(&dc->ndev->dev,
		CHAR_BUF_SIZE_MAX, GFP_KERNEL);
	if (!hdmi->hpd_switch_name) {
		err = -ENOMEM;
		goto fail_hdmi;
	}

	hdmi->audio_switch_name = devm_kzalloc(&dc->ndev->dev,
		CHAR_BUF_SIZE_MAX, GFP_KERNEL);
	if (!hdmi->audio_switch_name) {
		err = -ENOMEM;
		goto fail_hpd_switch;
	}

	hdmi->dc = dc;
	hdmi->edid_src = EDID_SRC_PANEL;
	hdmi->plug_state = TEGRA_HDMI_MONITOR_DISABLE;

	if (of_property_read_bool(panel_np, "nvidia,edid"))
		hdmi->edid_src = EDID_SRC_DT;

	hdmi->dpaux = tegra_dpaux_init_data(dc, sor_np);
	if (IS_ERR_OR_NULL(hdmi->dpaux)) {
		if ((hdmi->dpaux == NULL) &&
			(hdmi->dc->pdata->default_out->ddc_bus < 0) &&
			(hdmi->edid_src == EDID_SRC_DT)) {
			/* This is not an error since there is a case using */
			/* HDMI DDC as generic I2C with these configuration */
			dev_info(&hdmi->dc->ndev->dev,
				"hdmi: DDC is not used for HDMI.\n");
		} else {
			err = PTR_ERR(hdmi->dpaux);
			goto fail_audio_switch;
		}
	}

	hdmi->sor = tegra_dc_sor_init(dc, NULL);
	if (IS_ERR_OR_NULL(hdmi->sor)) {
		err = PTR_ERR(hdmi->sor);
		goto fail_audio_switch;
	}

	hdmi->pdata = dc->pdata->default_out->hdmi_out;
	hdmi->ddc_refcount = 0; /* assumes this is disabled when starting */
	mutex_init(&hdmi->ddc_refcount_lock);
	hdmi->nvhdcp = NULL;
	hdmi->mon_spec_valid = false;
	hdmi->eld_valid = false;
	hdmi->device_shutdown = false;

	if (hdmi_instance) {
		snprintf(hdmi->hpd_switch_name, CHAR_BUF_SIZE_MAX,
			"hdmi%d", hdmi_instance);
		snprintf(hdmi->audio_switch_name, CHAR_BUF_SIZE_MAX,
			"hdmi%d_audio", hdmi_instance);
	} else {
		snprintf(hdmi->hpd_switch_name, CHAR_BUF_SIZE_MAX, "hdmi");
		snprintf(hdmi->audio_switch_name, CHAR_BUF_SIZE_MAX,
			"hdmi_audio");
	}
#ifdef CONFIG_SWITCH
	hdmi->hpd_switch.name = hdmi->hpd_switch_name;
	hdmi->audio_switch.name = hdmi->audio_switch_name;
#endif

	if (0) {
		/* TODO: seamless boot mode needs initialize the state */
	} else {
		hdmi->enabled = false;
		atomic_set(&hdmi->clock_refcount, 0);
	}
	atomic_set(&hdmi->suspended, 0);
	if (!tegra_platform_is_sim()) {
		hdmi->nvhdcp = tegra_nvhdcp_create(hdmi, dc->ndev->id,
			dc->out->ddc_bus);
		if (IS_ERR(hdmi->nvhdcp)) {
			err = PTR_ERR(hdmi->nvhdcp);
			dev_err(&dc->ndev->dev,
				"hdmi hdcp creation failed with err %d\n", err);
		} else {
			tegra_nvhdcp_debugfs_init(hdmi->nvhdcp);
		}
	}

	tegra_hdmi_ddc_init(hdmi);

	tegra_hdmi_scdc_init(hdmi);

	err = tegra_hdmi_vrr_init(hdmi);
	if (err && err != -ENODEV)
		dev_err(&dc->ndev->dev, "vrr_init failed\n");

	tegra_hdmi_debugfs_init(hdmi);

	tegra_hdmi_tmds_init(hdmi);

	tegra_dc_set_outdata(dc, hdmi);

	tegra_hdmi_hdr_init(hdmi);

	if (hdmi->pdata->hdmi2fpd_bridge_enable) {
		hdmi2fpd_init(dc);
		hdmi2fpd_enable(dc);
	}
#ifdef CONFIG_TEGRA_HDMI2GMSL_MAX929x
	if (hdmi->pdata->hdmi2gmsl_bridge_enable) {
		hdmi->out_ops = &tegra_hdmi2gmsl_ops;
		hdmi->out_ops->init(hdmi);
	}
#endif	/* CONFIG_TEGRA_HDMI2GMSL_MAX929x */

	if (hdmi->pdata->hdmi2dsi_bridge_enable) {
		hdmi->out_ops = &tegra_hdmi2dsi_ops;
		if (hdmi->out_ops && hdmi->out_ops->init)
			hdmi->out_ops->init(hdmi);
	}

	/* NOTE: Below code is applicable to L4T or embedded systems and is
	 * protected accordingly. This section chooses a mode to early
	 * enable DC.
	 */
	if ((IS_ENABLED(CONFIG_FRAMEBUFFER_CONSOLE) ||
			((dc->pdata->flags & TEGRA_DC_FLAG_ENABLED) &&
			 (dc->pdata->flags & TEGRA_DC_FLAG_SET_EARLY_MODE))) &&
			dc->out && (dc->out->type == TEGRA_DC_OUT_HDMI)) {
		/* In case of seamless display, dc mode would already be set */
		if (!dc->initialized)
			tegra_dc_set_fbcon_boot_mode(dc, hdmi->edid);
	}

#ifdef CONFIG_SWITCH
	err = switch_dev_register(&hdmi->hpd_switch);
	if (err)
		dev_err(&dc->ndev->dev,
			"%s: failed to register hpd switch, err=%d\n",
			__func__, err);

	err = switch_dev_register(&hdmi->audio_switch);
	if (err)
		dev_err(&dc->ndev->dev,
			"%s: failed to register audio switch, err=%d\n",
			__func__, err);
#endif

#ifdef CONFIG_TEGRA_HDA_DC
	tegra_hda_init(dc, hdmi);
#endif

	hdmi_instance++;
	return 0;

fail_audio_switch:
	devm_kfree(&dc->ndev->dev, hdmi->audio_switch_name);
fail_hpd_switch:
	devm_kfree(&dc->ndev->dev, hdmi->hpd_switch_name);
fail_hdmi:
	kfree(hdmi->tmds_range);
	hdmi->tmds_range = NULL;
	devm_kfree(&dc->ndev->dev, hdmi);
fail_sor_np:
	return err;
}

static void tegra_dc_hdmi_destroy(struct tegra_dc *dc)
{
	struct tegra_hdmi *hdmi = NULL;

	if (!dc->current_topology.valid)
		return;

	hdmi = tegra_dc_get_outdata(dc);

	if (hdmi->pdata->hdmi2fpd_bridge_enable)
		hdmi2fpd_destroy(dc);

	if (hdmi->out_ops && hdmi->out_ops->destroy)
		hdmi->out_ops->destroy(hdmi);

#ifdef CONFIG_TEGRA_HDA_DC
	tegra_hda_destroy(hdmi->hda_handle);
#endif

	tegra_nvhdcp_destroy(hdmi->nvhdcp);

	if (hdmi->dpaux)
		tegra_dpaux_destroy_data(hdmi->dpaux);
	if (hdmi->ddc_i2c_client)
		i2c_unregister_device(hdmi->ddc_i2c_client);
	if (hdmi->scdc_i2c_client)
		i2c_unregister_device(hdmi->scdc_i2c_client);
	if (hdmi->ddcci_i2c_client)
		i2c_unregister_device(hdmi->ddcci_i2c_client);

	tegra_dc_sor_destroy(hdmi->sor);

	tegra_edid_destroy(hdmi->edid);

	kfree(hdmi->tmds_range);
	hdmi->tmds_range = NULL;
	hdmi->prod_list = NULL;

	free_irq(gpio_to_irq(dc->out->hotplug_gpio), dc);
	gpio_free(dc->out->hotplug_gpio);

	tegra_dc_out_destroy(dc);

#ifdef CONFIG_SWITCH
	switch_dev_unregister(&hdmi->hpd_switch);
	switch_dev_unregister(&hdmi->audio_switch);
#endif

	devm_kfree(&dc->ndev->dev, hdmi->hpd_switch_name);
	devm_kfree(&dc->ndev->dev, hdmi->audio_switch_name);
	tegra_hdmi_debugfs_remove(hdmi);
	devm_kfree(&dc->ndev->dev, hdmi);
	dc->current_topology.valid = false;
}

static void tegra_hdmi_config(struct tegra_hdmi *hdmi)
{
	struct tegra_dc_sor_data *sor = hdmi->sor;
	struct tegra_dc *dc = hdmi->dc;
	u32 h_pulse_start, h_pulse_end;
	u32 hblank, max_ac, rekey;
	unsigned long val;
	u32 arm_video_range;

	if ((dc->mode.vmode & FB_VMODE_BYPASS) ||
	    !(dc->mode.vmode & FB_VMODE_LIMITED_RANGE))
		arm_video_range = NV_SOR_INPUT_CONTROL_ARM_VIDEO_RANGE_FULL;
	else
		arm_video_range = NV_SOR_INPUT_CONTROL_ARM_VIDEO_RANGE_LIMITED;

	tegra_sor_write_field(sor, NV_SOR_INPUT_CONTROL,
			NV_SOR_INPUT_CONTROL_ARM_VIDEO_RANGE_LIMITED,
			arm_video_range);

	if (tegra_dc_is_t21x())
		tegra_sor_write_field(sor, NV_SOR_INPUT_CONTROL,
			NV_SOR_INPUT_CONTROL_HDMI_SRC_SELECT_DISPLAYB,
			NV_SOR_INPUT_CONTROL_HDMI_SRC_SELECT_DISPLAYB);

	/*
	 * The rekey register and corresponding eq want to operate
	 * on "-2" of the desired rekey value
	 */
	rekey = NV_SOR_HDMI_CTRL_REKEY_DEFAULT - 2;
	hblank = dc->mode.h_sync_width + dc->mode.h_back_porch +
			dc->mode.h_front_porch;
	max_ac = (hblank - rekey - 18) / 32;

	val = 0;
	/* no DVI when the display mode belongs to HDMI 2.0 */
	val |= (hdmi->dvi && tegra_sor_get_link_rate(hdmi->dc) <= 340000000)
		? 0x0 : NV_SOR_HDMI_CTRL_ENABLE;
	val |= NV_SOR_HDMI_CTRL_REKEY(rekey);
	val |= NV_SOR_HDMI_CTRL_MAX_AC_PACKET(max_ac);
	val |= NV_SOR_HDMI_CTRL_AUDIO_LAYOUT_SELECT;
	tegra_sor_writel(sor, NV_SOR_HDMI_CTRL, val);

	if (tegra_dc_is_t21x()) {
		tegra_dc_writel(dc, 0x180, DC_DISP_H_PULSE2_CONTROL);
		h_pulse_start = dc->mode.h_ref_to_sync +
			dc->mode.h_sync_width +
			dc->mode.h_back_porch - 10;
		h_pulse_end = h_pulse_start + 8;
		tegra_dc_writel(dc, PULSE_START(h_pulse_start) |
				PULSE_END(h_pulse_end),
				DC_DISP_H_PULSE2_POSITION_A);

		val = tegra_dc_readl(dc, DC_DISP_DISP_SIGNAL_OPTIONS0);
		val |= H_PULSE_2_ENABLE;
		tegra_dc_writel(dc, val, DC_DISP_DISP_SIGNAL_OPTIONS0);
	}
}

void tegra_hdmi_infoframe_pkt_write(struct tegra_hdmi *hdmi,
						u32 header_reg, u8 pkt_type,
						u8 pkt_vs, u8 pkt_len,
						void *reg_payload,
						u32 reg_payload_len,
						bool sw_checksum)
{
	struct tegra_dc_sor_data *sor = hdmi->sor;
	u32 val;
	u32 *data = reg_payload;
	u32 data_reg = header_reg + 1;

	val = NV_SOR_HDMI_INFOFRAME_HEADER_TYPE(pkt_type) |
		NV_SOR_HDMI_INFOFRAME_HEADER_VERSION(pkt_vs) |
		NV_SOR_HDMI_INFOFRAME_HEADER_LEN(pkt_len);
	tegra_sor_writel(sor, header_reg, val);

	if (sw_checksum) {
		u8 checksum = pkt_type + pkt_vs + pkt_len;

		for (val = 1; val < reg_payload_len; val++)
			checksum += ((u8 *)reg_payload)[val];

		/* The first byte of the payload must always be the checksum
		 * that we are going to calculate in SW */
		((u8 *)reg_payload)[0] = (256 - checksum);
	}

	for (val = 0; val < reg_payload_len; val += 4, data_reg++, data++)
		tegra_sor_writel(sor, data_reg, *data);
}

u32 tegra_hdmi_get_cea_modedb_size(struct tegra_hdmi *hdmi)
{
	if (!tegra_hdmi_is_connected(hdmi) || !hdmi->mon_spec_valid)
		return 0;

	return (hdmi->mon_spec.misc & FB_MISC_HDMI_FORUM) ?
		CEA_861_F_MODEDB_SIZE : CEA_861_D_MODEDB_SIZE;
}

static void tegra_hdmi_get_cea_fb_videomode(struct fb_videomode *m,
						struct tegra_hdmi *hdmi)
{
	struct tegra_dc *dc = hdmi->dc;
	struct tegra_dc_mode dc_mode;
	bool yuv_bypass_vmode = false;

	memcpy(&dc_mode, &dc->mode, sizeof(dc->mode));

	/* get CEA video timings */
	yuv_bypass_vmode = (dc_mode.vmode & FB_VMODE_YUV_MASK) &&
				(dc_mode.vmode & FB_VMODE_BYPASS);

	/*
	 * On T21x, the bypass flag is exclusively sent as part of the flip, and
	 * not as part of the modeset.
	 */
	if (yuv_bypass_vmode || tegra_dc_is_t21x()) {
		if (tegra_dc_is_yuv420_8bpc(&dc_mode)) {
			dc_mode.h_back_porch *= 2;
			dc_mode.h_front_porch *= 2;
			dc_mode.h_sync_width *= 2;
			dc_mode.h_active *= 2;
			dc_mode.pclk *= 2;
		} else if (tegra_dc_is_yuv420_10bpc(&dc_mode)) {
			dc_mode.h_back_porch = (dc_mode.h_back_porch * 8) / 5;
			dc_mode.h_front_porch = (dc_mode.h_front_porch * 8) / 5;
			dc_mode.h_sync_width = (dc_mode.h_sync_width * 8) / 5;
			dc_mode.h_active = (dc_mode.h_active * 8) / 5;
			dc_mode.pclk = (dc_mode.pclk / 5) * 8;
		}
	}

	tegra_dc_to_fb_videomode(m, &dc_mode);

	/* only interlaced required for VIC identification */
	m->vmode &= FB_VMODE_INTERLACED;
}

__maybe_unused
static int tegra_hdmi_find_cea_vic(struct tegra_hdmi *hdmi)
{
	struct fb_videomode m;
	struct tegra_dc *dc = hdmi->dc;
	unsigned i;
	unsigned best = 0;
	u32 modedb_size = tegra_hdmi_get_cea_modedb_size(hdmi);

	if (dc->initialized) {
		u32 vic = tegra_sor_readl(hdmi->sor,
			NV_SOR_HDMI_AVI_INFOFRAME_SUBPACK0_HIGH) & 0xff;
		if (!vic)
			dev_warn(&dc->ndev->dev, "hdmi: BL set VIC 0\n");
		return vic;
	}

	tegra_hdmi_get_cea_fb_videomode(&m, hdmi);

	m.flag &= ~FB_FLAG_RATIO_MASK;
	m.flag |= tegra_dc_get_aspect_ratio(dc);

	for (i = 1; i < modedb_size; i++) {
		const struct fb_videomode *curr = &cea_modes[i];

		if (!fb_mode_is_equal_tolerance(curr, &m,
			FB_MODE_TOLERANCE_DEFAULT))
			continue;

		if (!best)
			best = i;
		/* if either flag is set, then match is required */
		if (curr->flag &
			(FB_FLAG_RATIO_4_3 | FB_FLAG_RATIO_16_9 |
			FB_FLAG_RATIO_64_27 | FB_FLAG_RATIO_256_135)) {
			if (m.flag & curr->flag & FB_FLAG_RATIO_4_3)
				best = i;
			else if (m.flag & curr->flag & FB_FLAG_RATIO_16_9)
				best = i;
			else if (m.flag & curr->flag & FB_FLAG_RATIO_64_27)
				best = i;
			else if (m.flag & curr->flag & FB_FLAG_RATIO_256_135)
				best = i;
		} else {
			best = i;
		}
	}
	return best;
}

static u32 tegra_hdmi_get_aspect_ratio(struct tegra_hdmi *hdmi)
{
	u32 aspect_ratio;

	switch (hdmi->dc->mode.avi_m) {
	case TEGRA_DC_MODE_AVI_M_4_3:
		aspect_ratio = HDMI_AVI_ASPECT_RATIO_4_3;
		break;
	case TEGRA_DC_MODE_AVI_M_16_9:
		aspect_ratio = HDMI_AVI_ASPECT_RATIO_16_9;
		break;
	/*
	 * no avi_m field for picture aspect ratio 64:27 and 256:135.
	 * sink detects via VIC, avi_m is 0.
	 */
	case TEGRA_DC_MODE_AVI_M_64_27: /* fall through */
	case TEGRA_DC_MODE_AVI_M_256_135: /* fall through */
	default:
		aspect_ratio = HDMI_AVI_ASPECT_RATIO_NO_DATA;
	}

	/* For seamless HDMI, read aspect ratio parameters from bootloader
	 * set AVI Infoframe parameters
	 */
	if ((aspect_ratio == HDMI_AVI_ASPECT_RATIO_NO_DATA) &&
					(hdmi->dc->initialized)) {
		u32 temp = 0;
		temp = tegra_sor_readl(hdmi->sor,
			NV_SOR_HDMI_AVI_INFOFRAME_SUBPACK0_LOW);
		temp = (temp >> 20) & 0x3;
		switch (temp) {
		case 1:
		aspect_ratio = HDMI_AVI_ASPECT_RATIO_4_3;
		break;
		case 2:
		aspect_ratio = HDMI_AVI_ASPECT_RATIO_16_9;
		break;
		default:
		aspect_ratio = HDMI_AVI_ASPECT_RATIO_NO_DATA;
		}
	}
	return aspect_ratio;
}

static u32 tegra_hdmi_get_rgb_ycc(struct tegra_hdmi *hdmi)
{
	int yuv_flag = hdmi->dc->mode.vmode & FB_VMODE_YUV_MASK;

	/*
	 * For seamless HDMI, read YUV flag parameters from bootloader
	 * set AVI Infoframe parameters
	 */
	if (hdmi->dc->initialized) {
		u32 temp = 0;
		temp = tegra_sor_readl(hdmi->sor,
			NV_SOR_HDMI_AVI_INFOFRAME_SUBPACK0_LOW);
		temp = (temp >> 13) & 0x3;
		switch (temp) {
		case HDMI_AVI_RGB:
			return HDMI_AVI_RGB;
		case HDMI_AVI_YCC_420:
			return HDMI_AVI_YCC_420;
		default:
			dev_warn(&hdmi->dc->ndev->dev, "hdmi: BL didn't set RGB/YUV indicator flag\n");
			break;
		}
	}

	if (yuv_flag & (FB_VMODE_Y420 | FB_VMODE_Y420_ONLY))
		return HDMI_AVI_YCC_420;
	else if (yuv_flag & FB_VMODE_Y422)
		return HDMI_AVI_YCC_422;
	else if (yuv_flag & FB_VMODE_Y444)
		return HDMI_AVI_YCC_444;

	return HDMI_AVI_RGB;
}

static bool tegra_hdmi_is_ex_colorimetry(struct tegra_hdmi *hdmi)
{
	return !!(hdmi->dc->mode.vmode & FB_VMODE_EC_ENABLE);
}

static u32 tegra_hdmi_get_ex_colorimetry(struct tegra_hdmi *hdmi)
{
	u32 vmode = hdmi->dc->mode.vmode;

	return tegra_hdmi_is_ex_colorimetry(hdmi) ?
		((vmode & FB_VMODE_EC_MASK) >> FB_VMODE_EC_SHIFT) :
		HDMI_AVI_EXT_COLORIMETRY_INVALID;
}

static u32 tegra_hdmi_get_rgb_quant(struct tegra_hdmi *hdmi)
{
	u32 vmode = hdmi->dc->mode.vmode;

	/*
	 * For seamless HDMI, read Q0/Q1 from bootloader
	 * set AVI Infoframe packet contents of the HDMI SPEC
	 */
	if (hdmi->dc->initialized) {
		u32 temp = 0;

		dev_info(&hdmi->dc->ndev->dev, "hdmi: get RGB quant from REG programmed by BL.\n");
		temp = tegra_sor_readl(hdmi->sor,
			NV_SOR_HDMI_AVI_INFOFRAME_SUBPACK0_LOW);
		temp = (temp >> 26) & 0x3;
		switch (temp) {
		case HDMI_AVI_RGB_QUANT_DEFAULT:
		case HDMI_AVI_RGB_QUANT_LIMITED:
		case HDMI_AVI_RGB_QUANT_FULL:
			return temp;
		default:
			dev_warn(&hdmi->dc->ndev->dev,
				"hdmi: BL didn't set quantization range\n");
			return HDMI_AVI_RGB_QUANT_DEFAULT;
		}
	}

	return (vmode & FB_VMODE_LIMITED_RANGE) ? HDMI_AVI_RGB_QUANT_LIMITED :
		HDMI_AVI_RGB_QUANT_FULL;
}

static u32 tegra_hdmi_get_ycc_quant(struct tegra_hdmi *hdmi)
{
	u32 vmode = hdmi->dc->mode.vmode;
	u32 hdmi_quant = HDMI_AVI_YCC_QUANT_NONE;

	/*
	 * For seamless HDMI, read YQ1/YQ1 from bootloader
	 * set AVI Infoframe packet contents of the HDMI SPEC
	 */
	if (hdmi->dc->initialized) {
		u32 temp = 0;

		dev_info(&hdmi->dc->ndev->dev, "hdmi: get YCC quant from REG programmed by BL.\n");
		temp = tegra_sor_readl(hdmi->sor,
			NV_SOR_HDMI_AVI_INFOFRAME_SUBPACK0_HIGH);
		temp = (temp >> 14) & 0x3;
		switch (temp) {
		case HDMI_AVI_YCC_QUANT_LIMITED:
		case HDMI_AVI_YCC_QUANT_FULL:
			return temp;
		default:
			dev_warn(&hdmi->dc->ndev->dev,
				"hdmi: BL didn't set ycc quantization range\n");
			return HDMI_AVI_YCC_QUANT_NONE;
		}
	}

	dev_info(&hdmi->dc->ndev->dev, "hdmi: get YCC quant from EDID.\n");
	if (tegra_edid_is_yuv_quantization_selectable(hdmi->edid)) {
		if (vmode & FB_VMODE_LIMITED_RANGE)
			hdmi_quant = HDMI_AVI_YCC_QUANT_LIMITED;
		else
			hdmi_quant = HDMI_AVI_YCC_QUANT_FULL;
	}
	return hdmi_quant;
}

static void tegra_hdmi_avi_infoframe_update(struct tegra_hdmi *hdmi)
{
	struct hdmi_avi_infoframe *avi = &hdmi->avi;

	memset(&hdmi->avi, 0, sizeof(hdmi->avi));

	avi->scan = HDMI_AVI_UNDERSCAN;
	avi->bar_valid = HDMI_AVI_BAR_INVALID;
	avi->act_fmt_valid = HDMI_AVI_ACTIVE_FORMAT_INVALID;
	avi->rgb_ycc = tegra_hdmi_get_rgb_ycc(hdmi);

	avi->act_format = HDMI_AVI_ACTIVE_FORMAT_SAME;
	avi->aspect_ratio = tegra_hdmi_get_aspect_ratio(hdmi);
	avi->colorimetry = tegra_hdmi_is_ex_colorimetry(hdmi) ?
			HDMI_AVI_COLORIMETRY_EXTENDED_VALID :
			HDMI_AVI_COLORIMETRY_DEFAULT;

	avi->scaling = HDMI_AVI_SCALING_UNKNOWN;
	avi->rgb_quant = tegra_hdmi_get_rgb_quant(hdmi);
	avi->ext_colorimetry = tegra_hdmi_get_ex_colorimetry(hdmi);

	switch (hdmi->avi_colorimetry) {
	case TEGRA_DC_EXT_AVI_COLORIMETRY_BT2020_YCC_RGB:
		avi->colorimetry = HDMI_AVI_COLORIMETRY_EXTENDED_VALID;
		avi->ext_colorimetry = HDMI_AVI_EXT_COLORIMETRY_BT2020_YCC_RGB;
		break;
	case TEGRA_DC_EXT_AVI_COLORIMETRY_xvYCC709:
		avi->colorimetry = HDMI_AVI_COLORIMETRY_EXTENDED_VALID;
		avi->ext_colorimetry = HDMI_AVI_EXT_COLORIMETRY_xvYCC709;
		break;
	default:
		/* Let default value as it is.*/
		break;
	}

	avi->it_content = HDMI_AVI_IT_CONTENT_FALSE;

	/* set correct vic if video format is cea defined else set 0 */
	avi->video_format = tegra_hdmi_find_cea_vic(hdmi);

	/* set aspect ratio to match CEA VIC */
	if (avi->video_format) {
		/* Only 4:3 & 16:9 are valid picture aspect ratios for avi_m */
		if (cea_modes[avi->video_format].flag & FB_FLAG_RATIO_4_3)
			avi->aspect_ratio = HDMI_AVI_ASPECT_RATIO_4_3;
		else if (cea_modes[avi->video_format].flag & FB_FLAG_RATIO_16_9)
			avi->aspect_ratio = HDMI_AVI_ASPECT_RATIO_16_9;
	}

	avi->pix_rep = HDMI_AVI_NO_PIX_REPEAT;
	avi->it_content_type = HDMI_AVI_IT_CONTENT_NONE;
	avi->ycc_quant = tegra_hdmi_get_ycc_quant(hdmi);

	avi->top_bar_end_line_low_byte = 0;
	avi->top_bar_end_line_high_byte = 0;

	avi->bot_bar_start_line_low_byte = 0;
	avi->bot_bar_start_line_high_byte = 0;

	avi->left_bar_end_pixel_low_byte = 0;
	avi->left_bar_end_pixel_high_byte = 0;

	avi->right_bar_start_pixel_low_byte = 0;
	avi->right_bar_start_pixel_high_byte = 0;
}

static void tegra_hdmi_avi_infoframe(struct tegra_hdmi *hdmi)
{
	struct tegra_dc_sor_data *sor = hdmi->sor;

	if (hdmi->dvi)
		return;

	/* disable avi infoframe before configuring except for seamless case */
	if (!hdmi->dc->initialized)
		tegra_sor_writel(sor, NV_SOR_HDMI_AVI_INFOFRAME_CTRL, 0);

	tegra_hdmi_avi_infoframe_update(hdmi);

	tegra_hdmi_infoframe_pkt_write(hdmi, NV_SOR_HDMI_AVI_INFOFRAME_HEADER,
					HDMI_INFOFRAME_TYPE_AVI,
					HDMI_INFOFRAME_VS_AVI,
					HDMI_INFOFRAME_LEN_AVI,
					&hdmi->avi, sizeof(hdmi->avi),
					false);

	/* Send infoframe every frame, checksum hw generated */
	tegra_sor_writel(sor, NV_SOR_HDMI_AVI_INFOFRAME_CTRL,
		NV_SOR_HDMI_AVI_INFOFRAME_CTRL_ENABLE_YES |
		NV_SOR_HDMI_AVI_INFOFRAME_CTRL_OTHER_DISABLE |
		NV_SOR_HDMI_AVI_INFOFRAME_CTRL_SINGLE_DISABLE |
		NV_SOR_HDMI_AVI_INFOFRAME_CTRL_CHECKSUM_ENABLE);
}

static int tegra_dc_hdmi_set_avi(struct tegra_dc *dc, struct tegra_dc_ext_avi *avi)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);

	hdmi->avi_colorimetry = avi->avi_colorimetry;
	/* Setting AVI infoframe externally */
	tegra_hdmi_avi_infoframe(hdmi);

	return 0;
}

static int tegra_hdmi_get_extended_vic(const struct tegra_dc_mode *mode)
{
	struct fb_videomode m;
	struct tegra_dc_mode mode_fixed;
	unsigned i;

	mode_fixed = *mode;

	if (mode_fixed.vmode & FB_VMODE_1000DIV1001) {
		mode_fixed.pclk = (u64)mode_fixed.pclk * 1001 / 1000;
		mode_fixed.vmode &= ~FB_VMODE_1000DIV1001;
	}

	tegra_dc_to_fb_videomode(&m, &mode_fixed);

	/* only interlaced required for VIC identification */
	m.vmode &= FB_VMODE_INTERLACED;

	for (i = 1; i < HDMI_EXT_MODEDB_SIZE; i++) {
		const struct fb_videomode *curr = &hdmi_ext_modes[i];

		if (fb_mode_is_equal_tolerance(curr, &m,
			FB_MODE_TOLERANCE_DEFAULT))
			return i;
	}
	return 0;
}

static void tegra_hdmi_vendor_infoframe_update(struct tegra_hdmi *hdmi)
{
	struct hdmi_vendor_infoframe *vsi = &hdmi->vsi;
	u8 extended_vic;

	memset(&hdmi->vsi, 0, sizeof(hdmi->vsi));

	vsi->oui = HDMI_LICENSING_LLC_OUI;

	extended_vic = tegra_hdmi_get_extended_vic(&hdmi->dc->mode);
	if (extended_vic) {
		vsi->video_format =
			HDMI_VENDOR_VIDEO_FORMAT_EXTENDED;
		vsi->extended_vic = extended_vic;
	}
}

static void tegra_hdmi_vendor_infoframe(struct tegra_hdmi *hdmi)
{
/* hdmi licensing, LLC vsi playload len as per hdmi1.4b  */
#define HDMI_INFOFRAME_LEN_VENDOR_LLC	(6)

	struct tegra_dc_sor_data *sor = hdmi->sor;

	if (hdmi->dvi)
		return;

	/* disable vsi infoframe before configuring */
	tegra_sor_writel(sor, NV_SOR_HDMI_VSI_INFOFRAME_CTRL, 0);

	tegra_hdmi_vendor_infoframe_update(hdmi);

	tegra_hdmi_infoframe_pkt_write(hdmi, NV_SOR_HDMI_VSI_INFOFRAME_HEADER,
					HDMI_INFOFRAME_TYPE_VENDOR,
					HDMI_INFOFRAME_VS_VENDOR,
					HDMI_INFOFRAME_LEN_VENDOR_LLC,
					&hdmi->vsi, sizeof(hdmi->vsi),
					false);

	/* Send infoframe every frame, checksum hw generated */
	tegra_sor_writel(sor, NV_SOR_HDMI_VSI_INFOFRAME_CTRL,
		NV_SOR_HDMI_VSI_INFOFRAME_CTRL_ENABLE_YES |
		NV_SOR_HDMI_VSI_INFOFRAME_CTRL_OTHER_DISABLE |
		NV_SOR_HDMI_VSI_INFOFRAME_CTRL_SINGLE_DISABLE |
		NV_SOR_HDMI_VSI_INFOFRAME_CTRL_CHECKSUM_ENABLE);

#undef HDMI_INFOFRAME_LEN_VENDOR_LLC
}

static void tegra_hdmi_hdr_infoframe_update(struct tegra_hdmi *hdmi)
{
	struct hdmi_hdr_infoframe *hdr = &hdmi->hdr;

	memset(&hdmi->hdr, 0, sizeof(hdmi->hdr));

	hdr->eotf = hdmi->dc->hdr.eotf;
	hdr->static_metadata_id = hdmi->dc->hdr.static_metadata_id;

	/* PB3-14 : Group 1 : Static Metadata*/
	hdr->display_primaries_x_0_lsb = hdmi->dc->hdr.static_metadata[0];
	hdr->display_primaries_x_0_msb = hdmi->dc->hdr.static_metadata[1];
	hdr->display_primaries_y_0_lsb = hdmi->dc->hdr.static_metadata[2];
	hdr->display_primaries_y_0_msb = hdmi->dc->hdr.static_metadata[3];
	hdr->display_primaries_x_1_lsb = hdmi->dc->hdr.static_metadata[4];
	hdr->display_primaries_x_1_msb = hdmi->dc->hdr.static_metadata[5];
	hdr->display_primaries_y_1_lsb = hdmi->dc->hdr.static_metadata[6];
	hdr->display_primaries_y_1_msb = hdmi->dc->hdr.static_metadata[7];
	hdr->display_primaries_x_2_lsb = hdmi->dc->hdr.static_metadata[8];
	hdr->display_primaries_x_2_msb = hdmi->dc->hdr.static_metadata[9];
	hdr->display_primaries_y_2_lsb = hdmi->dc->hdr.static_metadata[10];
	hdr->display_primaries_y_2_msb = hdmi->dc->hdr.static_metadata[11];

	/* PB15-18 : Group 2 : Static Metadata*/
	hdr->white_point_x_lsb = hdmi->dc->hdr.static_metadata[12];
	hdr->white_point_x_msb = hdmi->dc->hdr.static_metadata[13];
	hdr->white_point_y_lsb = hdmi->dc->hdr.static_metadata[14];
	hdr->white_point_y_msb = hdmi->dc->hdr.static_metadata[15];

	/* PB19-20 : Group 3 : Static Metadata*/
	hdr->max_display_mastering_luminance_lsb =
					hdmi->dc->hdr.static_metadata[16];
	hdr->max_display_mastering_luminance_msb =
					hdmi->dc->hdr.static_metadata[17];

	/* PB21-22 : Group 4 : Static Metadata*/
	hdr->min_display_mastering_luminance_lsb =
					hdmi->dc->hdr.static_metadata[18];
	hdr->min_display_mastering_luminance_msb =
					hdmi->dc->hdr.static_metadata[19];

	/* PB23-24 : Group 5 : Static Metadata*/
	hdr->max_content_light_level_lsb = hdmi->dc->hdr.static_metadata[20];
	hdr->max_content_light_level_msb = hdmi->dc->hdr.static_metadata[21];

	/* PB25-26 : Group 6 : Static Metadata*/
	hdr->max_frame_avg_light_level_lsb = hdmi->dc->hdr.static_metadata[22];
	hdr->min_frame_avg_light_level_msb = hdmi->dc->hdr.static_metadata[23];

	return;
}

static void tegra_hdmi_hdr_infoframe(struct tegra_hdmi *hdmi)
{
	struct tegra_dc_sor_data *sor = hdmi->sor;

	/* set_bits = contains all the bits to be set
	 * for NV_SOR_HDMI_GENERIC_CTRL reg */
	u32 set_bits = (NV_SOR_HDMI_GENERIC_CTRL_ENABLE_YES |
			NV_SOR_HDMI_GENERIC_CTRL_AUDIO_ENABLE);

	/* disable generic infoframe before configuring */
	tegra_sor_writel(sor, NV_SOR_HDMI_GENERIC_CTRL, 0);

	tegra_hdmi_hdr_infoframe_update(hdmi);

	tegra_hdmi_infoframe_pkt_write(hdmi, NV_SOR_HDMI_GENERIC_HEADER,
					HDMI_INFOFRAME_TYPE_HDR,
					HDMI_INFOFRAME_VS_HDR,
					HDMI_INFOFRAME_LEN_HDR,
					&hdmi->hdr, sizeof(hdmi->hdr),
					true);

	/* set the required bits in NV_SOR_HDMI_GENERIC_CTRL*/
	tegra_sor_writel(sor, NV_SOR_HDMI_GENERIC_CTRL, set_bits);

	return;
}

static void tegra_hdmi_spd_infoframe_update(struct tegra_hdmi *hdmi)
{
	struct hdmi_spd_infoframe *spd = &hdmi->spd;
	struct spd_infoframe *spd_infoframe =
		hdmi->dc->pdata->default_out->hdmi_out->spd_infoframe;

	memset(&hdmi->spd, 0, sizeof(hdmi->spd));

	if (!spd_infoframe)
		return;

	/* PB1-8 : Vendor Name */
	spd->vendor_name_char_1 = spd_infoframe->vendor_name[0];
	spd->vendor_name_char_2 = spd_infoframe->vendor_name[1];
	spd->vendor_name_char_3 = spd_infoframe->vendor_name[2];
	spd->vendor_name_char_4 = spd_infoframe->vendor_name[3];
	spd->vendor_name_char_5 = spd_infoframe->vendor_name[4];
	spd->vendor_name_char_6 = spd_infoframe->vendor_name[5];
	spd->vendor_name_char_7 = spd_infoframe->vendor_name[6];
	spd->vendor_name_char_8 = spd_infoframe->vendor_name[7];

	/* PB9-24 : Product Description */
	spd->prod_desc_char_1 = spd_infoframe->prod_desc[0];
	spd->prod_desc_char_2 = spd_infoframe->prod_desc[1];
	spd->prod_desc_char_3 = spd_infoframe->prod_desc[2];
	spd->prod_desc_char_4 = spd_infoframe->prod_desc[3];
	spd->prod_desc_char_5 = spd_infoframe->prod_desc[4];
	spd->prod_desc_char_6 = spd_infoframe->prod_desc[5];
	spd->prod_desc_char_7 = spd_infoframe->prod_desc[6];
	spd->prod_desc_char_8 = spd_infoframe->prod_desc[7];
	spd->prod_desc_char_9 = spd_infoframe->prod_desc[8];
	spd->prod_desc_char_10 = spd_infoframe->prod_desc[9];
	spd->prod_desc_char_11 = spd_infoframe->prod_desc[10];
	spd->prod_desc_char_12 = spd_infoframe->prod_desc[11];
	spd->prod_desc_char_13 = spd_infoframe->prod_desc[12];
	spd->prod_desc_char_14 = spd_infoframe->prod_desc[13];
	spd->prod_desc_char_15 = spd_infoframe->prod_desc[14];
	spd->prod_desc_char_16 = spd_infoframe->prod_desc[15];

	/* PB25 : Source Information */
	spd->source_information = spd_infoframe->source_information;
}

static void tegra_hdmi_spd_infoframe(struct tegra_hdmi *hdmi)
{
	struct tegra_dc_sor_data *sor = hdmi->sor;
	u32 val, set_bits;

	/* If the generic infoframe is not for SPD, return */
	if (hdmi->pdata->generic_infoframe_type != HDMI_INFOFRAME_TYPE_SPD)
		return;

	/* set_bits = contains all the bits to be set
	 * for NV_SOR_HDMI_GENERIC_CTRL reg
	 */
	set_bits = (NV_SOR_HDMI_GENERIC_CTRL_ENABLE_YES |
			NV_SOR_HDMI_GENERIC_CTRL_OTHER_DISABLE |
			NV_SOR_HDMI_GENERIC_CTRL_SINGLE_DISABLE);

	/* read the current value to restore some bit values */
	val = (tegra_sor_readl(sor, NV_SOR_HDMI_GENERIC_CTRL)
				& ~set_bits);

	/* disable generic infoframe before configuring */
	tegra_sor_writel(sor, NV_SOR_HDMI_GENERIC_CTRL, 0);

	tegra_hdmi_spd_infoframe_update(hdmi);

	tegra_hdmi_infoframe_pkt_write(hdmi, NV_SOR_HDMI_GENERIC_HEADER,
					HDMI_INFOFRAME_TYPE_SPD,
					HDMI_INFOFRAME_VS_SPD,
					HDMI_INFOFRAME_LEN_SPD,
					&hdmi->spd, sizeof(hdmi->spd),
					true);

	/* set the required bits in NV_SOR_HDMI_GENERIC_CTRL*/
	val = val | set_bits;

	tegra_sor_writel(sor, NV_SOR_HDMI_GENERIC_CTRL, val);
}

__maybe_unused
static int tegra_hdmi_scdc_read(struct tegra_hdmi *hdmi,
					u8 offset_data[][2], u32 n_entries)
{
	u32 i;
	int ret = 0;
	struct i2c_adapter *i2c_adap = i2c_get_adapter(hdmi->dc->out->ddc_bus);
	struct i2c_msg msg[] = {
		{
			.addr = 0x54,
			.len = 1,
			.buf = NULL,
		},
		{
			.addr = 0x54,
			.flags = I2C_M_RD,
			.len = 1,
			.buf = NULL,
		},
	};

	_tegra_hdmi_ddc_enable(hdmi);

	for (i = 0; i < n_entries; i++) {
		msg[0].buf = offset_data[i];
		msg[1].buf = &offset_data[i][1];
		do {
			ret = tegra_hdmi_scdc_i2c_xfer(hdmi->dc, msg,
							ARRAY_SIZE(msg));
		} while ((ret < 0)  && !tegra_edid_i2c_divide_rate(hdmi->edid));
	}

	/* Reset ddc i2c clock rate to original rate */
	ret = tegra_edid_i2c_adap_change_rate(i2c_adap,
				hdmi->ddc_i2c_original_rate);
	if (ret < 0)
		dev_info(&hdmi->dc->ndev->dev,
			"Failed to reset DDC i2c clock rate for scdc read\n");

	_tegra_hdmi_ddc_disable(hdmi);

	return 0;
}

static int tegra_hdmi_scdc_write(struct tegra_hdmi *hdmi,
					u8 offset_data[][2], u32 n_entries)
{
	u32 i;
	struct i2c_msg msg[] = {
		{
			.addr = 0x54,
			.len = 2,
			.buf = NULL,
		},
	};

	_tegra_hdmi_ddc_enable(hdmi);

	for (i = 0; i < n_entries; i++) {
		msg[0].buf = offset_data[i];
		tegra_hdmi_scdc_i2c_xfer(hdmi->dc, msg, ARRAY_SIZE(msg));
	}

	_tegra_hdmi_ddc_disable(hdmi);

	return 0;
}

static int tegra_hdmi_v2_x_mon_config(struct tegra_hdmi *hdmi, bool enable)
{
	u8 tmds_config_en[][2] = {
		{
			HDMI_SCDC_TMDS_CONFIG_OFFSET,
			(HDMI_SCDC_TMDS_CONFIG_BIT_CLK_RATIO_40 |
			HDMI_SCDC_TMDS_CONFIG_SCRAMBLING_EN)
		},
	};
	u8 tmds_config_dis[][2] = {
		{
			HDMI_SCDC_TMDS_CONFIG_OFFSET,
			0
		},
	};

	if (hdmi->dc->vedid)
		goto skip_scdc_i2c;

	tegra_hdmi_scdc_write(hdmi,
			enable ? tmds_config_en : tmds_config_dis,
			ARRAY_SIZE(tmds_config_en));

skip_scdc_i2c:
	return 0;
}

static void tegra_hdmi_v2_x_host_config(struct tegra_hdmi *hdmi, bool enable)
{
	u32 val = NV_SOR_HDMI2_CTRL_SCRAMBLE_ENABLE |
		NV_SOR_HDMI2_CTRL_SSCP_START |
		NV_SOR_HDMI2_CTRL_CLK_MODE_DIV_BY_4;

	tegra_sor_write_field(hdmi->sor, NV_SOR_HDMI2_CTRL,
			NV_SOR_HDMI2_CTRL_SCRAMBLE_ENABLE |
			NV_SOR_HDMI2_CTRL_SSCP_START_MASK |
			NV_SOR_HDMI2_CTRL_CLK_MODE_DIV_BY_4,
			enable ? val : 0);
}

static int _tegra_hdmi_v2_x_config(struct tegra_hdmi *hdmi)
{
#define SCDC_STABILIZATION_DELAY_MS (20)

	/* disable hdmi2.x config on host and monitor only
	 * if bootloader didn't initialize hdmi
	 */
	if (!hdmi->dc->initialized) {
		tegra_hdmi_v2_x_mon_config(hdmi, false);
		tegra_hdmi_v2_x_host_config(hdmi, false);
	}

	/* enable hdmi2.x config on host and monitor */
	tegra_hdmi_v2_x_mon_config(hdmi, true);
	msleep(SCDC_STABILIZATION_DELAY_MS);

	tegra_hdmi_v2_x_host_config(hdmi, true);

	return 0;
#undef SCDC_STABILIZATION_DELAY_MS
}

static int tegra_hdmi_v2_x_config(struct tegra_hdmi *hdmi)
{
	_tegra_hdmi_v2_x_config(hdmi);

	return 0;
}

static void tegra_hdmi_scdc_worker(struct work_struct *work)
{
	struct tegra_hdmi *hdmi = container_of(to_delayed_work(work),
				struct tegra_hdmi, scdc_work);
	u8 rd_status_flags[][2] = {
		{HDMI_SCDC_STATUS_FLAGS, 0x0}
	};
	unsigned long tmds_rate = tegra_sor_get_link_rate(hdmi->dc);

	if (!hdmi->enabled || tmds_rate <= 340000000)
		return;

	if (hdmi->dc->vedid)
		goto skip_scdc_i2c;

	if (!tegra_edid_is_scdc_present(hdmi->dc->edid))
		return;

	tegra_hdmi_scdc_read(hdmi, rd_status_flags, ARRAY_SIZE(rd_status_flags));
	if (!rd_status_flags[0][1]  && (tmds_rate > 340000000)) {
		dev_info(&hdmi->dc->ndev->dev, "hdmi: scdc scrambling status is reset, "
						"trying to reconfigure.\n");
		_tegra_hdmi_v2_x_config(hdmi);
	}

skip_scdc_i2c:
	/* reschedule the worker */
	cancel_delayed_work(&hdmi->scdc_work);
	schedule_delayed_work(&hdmi->scdc_work,
			msecs_to_jiffies(HDMI_SCDC_MONITOR_TIMEOUT_MS));
}

static void _tegra_hdmi_clock_enable(struct tegra_hdmi *hdmi)
{
	struct tegra_dc_sor_data *sor = hdmi->sor;
	tegra_disp_clk_prepare_enable(sor->safe_clk);

	tegra_hdmi_config_clk(hdmi, TEGRA_HDMI_SAFE_CLK);

	/* Setting various clock to figure out whether bpmp is able to
	 * handle this
	 */
	/* Set PLLD2 to preset clock value*/
	/* Set PLLD2 to SOR_REF_CLK*/
	tegra_sor_clk_enable(sor);
}

static void _tegra_hdmi_clock_disable(struct tegra_hdmi *hdmi)
{
	struct tegra_dc_sor_data *sor = hdmi->sor;
	tegra_sor_clk_disable(sor);
	tegra_disp_clk_disable_unprepare(sor->safe_clk);
}

void tegra_hdmi_get(struct tegra_dc *dc)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);

	if (atomic_inc_return(&hdmi->clock_refcount) == 1)
		_tegra_hdmi_clock_enable(hdmi);
}

void tegra_hdmi_put(struct tegra_dc *dc)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);

	if (WARN_ONCE(atomic_read(&hdmi->clock_refcount) <= 0,
		"hdmi: clock refcount imbalance"))
		return;
	if (atomic_dec_return(&hdmi->clock_refcount) == 0)
		_tegra_hdmi_clock_disable(hdmi);
}

static inline u32 tegra_hdmi_get_bpp(struct tegra_hdmi *hdmi)
{
	int yuv_flag = hdmi->dc->mode.vmode & FB_VMODE_YUV_MASK;

	/* Special case for YUV422 modes. For all  pixel depths, YUV422
	 * is packed as 24 BPP HDMI video stream
	 */
	if (yuv_flag & FB_VMODE_Y422)
		return 24;
	else if (yuv_flag & FB_VMODE_Y24)
		return 24;
	else if (yuv_flag & FB_VMODE_Y30)
		return 30;
	else if (yuv_flag & FB_VMODE_Y36)
		return 36;
	else if (yuv_flag & FB_VMODE_Y48)
		return 48;
	else
		return 0;
}

static u32 tegra_hdmi_gcp_color_depth(struct tegra_hdmi *hdmi)
{
	u32 gcp_cd = 0;

	switch (tegra_hdmi_get_bpp(hdmi)) {
	case 0:
		gcp_cd = TEGRA_HDMI_BPP_UNKNOWN;
		break;
	case 24:
		gcp_cd = TEGRA_HDMI_BPP_24;
		break;
	case 30:
		gcp_cd = TEGRA_HDMI_BPP_30;
		break;
	case 36:
		gcp_cd = TEGRA_HDMI_BPP_36;
		break;
	case 48:
		gcp_cd = TEGRA_HDMI_BPP_48;
		break;
	default:
		dev_WARN(&hdmi->dc->ndev->dev,
			"hdmi: unknown gcp color depth\n");
	};

	return gcp_cd;
}

/* return packing phase of last pixel in preceding video data period */
static u32 tegra_hdmi_gcp_packing_phase(struct tegra_hdmi *hdmi)
{
	int yuv_flag = hdmi->dc->mode.vmode & FB_VMODE_YUV_MASK;

	if (!tegra_hdmi_gcp_color_depth(hdmi))
		return 0;

	if ((IS_RGB(yuv_flag) && (yuv_flag == FB_VMODE_Y36)) ||
			(yuv_flag == (FB_VMODE_Y444 | FB_VMODE_Y36)))
		return 2;
	else
		return 0;
}

static bool tegra_hdmi_gcp_default_phase_en(struct tegra_hdmi *hdmi)
{
	int yuv_flag = hdmi->dc->mode.vmode & FB_VMODE_YUV_MASK;

	if (!tegra_hdmi_gcp_color_depth(hdmi))
		return false;

	if (tegra_dc_is_yuv420_10bpc(&hdmi->dc->mode) ||
			(yuv_flag == (FB_VMODE_Y444 | FB_VMODE_Y36)) ||
			(IS_RGB(yuv_flag) && (yuv_flag == FB_VMODE_Y36)))
		return true;
	else
		return false;
}

/* general control packet */
static void tegra_hdmi_gcp(struct tegra_hdmi *hdmi)
{
#define GCP_SB1_PP_SHIFT 4

	struct tegra_dc_sor_data *sor = hdmi->sor;
	u8 sb1, sb2;

	/* disable gcp before configuring */
	tegra_sor_writel(sor, NV_SOR_HDMI_GCP_CTRL, 0);

	sb1 = tegra_hdmi_gcp_packing_phase(hdmi) << GCP_SB1_PP_SHIFT |
		tegra_hdmi_gcp_color_depth(hdmi);
	sb2 = !!tegra_hdmi_gcp_default_phase_en(hdmi);
	tegra_sor_writel(sor, NV_SOR_HDMI_GCP_SUBPACK(0),
			sb1 << NV_SOR_HDMI_GCP_SUBPACK_SB1_SHIFT |
			sb2 << NV_SOR_HDMI_GCP_SUBPACK_SB2_SHIFT);

	/* Send gcp every frame */
	tegra_sor_writel(sor, NV_SOR_HDMI_GCP_CTRL,
			NV_SOR_HDMI_GCP_CTRL_ENABLE |
			NV_SOR_HDMI_GCP_CTRL_OTHER_DIS |
			NV_SOR_HDMI_GCP_CTRL_SINGLE_DIS);

#undef GCP_SB1_PP_SHIFT
}

static int tegra_hdmi_controller_enable(struct tegra_hdmi *hdmi)
{
	struct tegra_dc *dc = hdmi->dc;
	struct tegra_dc_sor_data *sor = hdmi->sor;
	u32 dispclk_div_8_2 = 0;
	int val = NV_SOR_CLK_CNTRL_DP_LINK_SPEED_G2_7;

	tegra_dc_get(dc);
	tegra_hdmi_get(dc);

	if (tegra_platform_is_fpga())
		tegra_sor_program_fpga_clk_mux(sor);

	if (tegra_dc_is_t21x()) {
		dispclk_div_8_2 = (dc->mode.pclk) / 1000000 * 4;
		tegra_sor_writel(sor, NV_SOR_REFCLK,
				NV_SOR_REFCLK_DIV_INT(dispclk_div_8_2 >> 2) |
				NV_SOR_REFCLK_DIV_FRAC(dispclk_div_8_2));
	}

	/* half rate and double vco */
	if (tegra_sor_get_link_rate(dc) > 340000000)
		val = NV_SOR_CLK_CNTRL_DP_LINK_SPEED_G5_4;

	val |= NV_SOR_CLK_CNTRL_DP_CLK_SEL_SINGLE_PCLK;
	tegra_sor_writel(hdmi->sor, NV_SOR_CLK_CNTRL, val);

	tegra_sor_cal(sor);

	tegra_hdmi_config_tmds(hdmi);

	tegra_sor_hdmi_pad_power_up(sor);

	tegra_sor_power_lanes(sor, 4, true);

	tegra_sor_config_xbar(hdmi->sor);

	tegra_sor_clk_switch_setup(sor, true);
	tegra_hdmi_config_clk(hdmi, TEGRA_HDMI_BRICK_CLK);

	tegra_dc_sor_set_internal_panel(sor, false);

	tegra_hdmi_config(hdmi);

	/* For simulator we still have use cases where
	 * we can have hotplug without a valid edid. This
	 * check ensures we don't reference a null edid
	 * */
	if (hdmi->edid) {
		tegra_hdmi_avi_infoframe(hdmi);
		tegra_hdmi_vendor_infoframe(hdmi);
		tegra_hdmi_spd_infoframe(hdmi);
	}

	if (dc->out->vrr_hotplug_state == TEGRA_HPD_STATE_NORMAL)
		tegra_dc_sor_attach(sor);
	else
		tegra_dc_enable_disp_ctrl_mode(dc);

	/* enable hdcp only if valid edid */
	if (hdmi->edid_src == EDID_SRC_PANEL && !hdmi->dc->vedid &&
		(tegra_edid_get_monspecs(hdmi->edid, &hdmi->mon_spec) == 0))
		tegra_nvhdcp_set_plug(hdmi->nvhdcp, true);

	if (hdmi->dpaux) {
		mutex_lock(&hdmi->dpaux->lock);
		tegra_dpaux_prod_set(hdmi->dpaux);
		mutex_unlock(&hdmi->dpaux->lock);
	}

	if (tegra_dc_is_t21x()) {
		tegra_dc_setup_clk(dc, dc->clk);
		tegra_dc_hdmi_setup_clk(dc, hdmi->sor->sor_clk);
	}

	/* IS THE POWER ENABLE AFTER ATTACH IS VALID*/
	/* TODO: Confirm sequence with HW */
	tegra_sor_writel(sor,  NV_SOR_SEQ_INST(0), 0x8080);
	tegra_sor_writel(sor,  NV_SOR_PWR, 0x80000001);

	if (tegra_sor_get_link_rate(hdmi->dc) > 340000000) {
		tegra_hdmi_v2_x_config(hdmi);
		schedule_delayed_work(&hdmi->scdc_work,
			msecs_to_jiffies(HDMI_SCDC_MONITOR_TIMEOUT_MS));
	}

	tegra_hdmi_gcp(hdmi);

	/* check SOR pad PLL lock status */
	/*  for newer SOC than T210 */
	if (!tegra_dc_is_t21x() &&
		!(tegra_sor_readl(sor, nv_sor_pll4()) &
			NV_SOR_PLL4_LOCKDET_MASK))
		dev_err(&hdmi->dc->ndev->dev,
			"hdmi: pad PLL is not locked!\n");

	tegra_dc_put(dc);
	return 0;
}

static void tegra_dc_hdmi_enable(struct tegra_dc *dc)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);

	if (hdmi->enabled)
		return;

	if (NULL != hdmi->out_ops && NULL != hdmi->out_ops->enable)
		hdmi->out_ops->enable(hdmi);

	tegra_hdmi_controller_enable(hdmi);

	hdmi->enabled = true;

#ifdef CONFIG_TEGRA_HDA_DC
	tegra_hda_enable(hdmi->hda_handle);
#endif

	if (!hdmi->dvi) {
		disp_state_extcon_aux_report(hdmi->sor->ctrl_num,
			EXTCON_DISP_AUX_STATE_ENABLED);
		pr_info("Extcon AUX%d(HDMI) enable\n", hdmi->sor->ctrl_num);
	}

#ifdef CONFIG_SWITCH
	if (!hdmi->dvi)
		switch_set_state(&hdmi->audio_switch, 1);
#endif
}

static inline u32 __maybe_unused
tegra_hdmi_get_shift_clk_div(struct tegra_hdmi *hdmi)
{
	/*
	 * HW does not support deep color yet
	 * always return 0
	 */

	return 0;
}

static void tegra_hdmi_config_clk_nvdisplay(struct tegra_hdmi *hdmi,
					    u32 clk_type)
{
	if (clk_type == hdmi->clk_type)
		return;

	if (clk_type == TEGRA_HDMI_BRICK_CLK) {

		struct tegra_dc_sor_data *sor = hdmi->sor;
		long rate = clk_get_rate(sor->ref_clk);

		if (tegra_sor_get_link_rate(hdmi->dc) > 340000000)
			clk_set_rate(sor->ref_clk, (rate >> 1));

		clk_set_rate(sor->pad_clk, rate);
		clk_set_parent(sor->sor_clk, sor->pad_clk);
		hdmi->clk_type = TEGRA_HDMI_BRICK_CLK;
	} else if (clk_type == TEGRA_HDMI_SAFE_CLK) {
		if (!hdmi->dc->initialized) {
			clk_set_parent(hdmi->sor->sor_clk, hdmi->sor->safe_clk);
			hdmi->clk_type = TEGRA_HDMI_SAFE_CLK;
		}
	} else {
		dev_err(&hdmi->dc->ndev->dev,
				"hdmi: incorrect clk type configured\n");
	}
}

static void tegra_hdmi_config_clk_t21x(struct tegra_hdmi *hdmi, u32 clk_type)
{
	if (clk_type == hdmi->clk_type)
		return;

	if (clk_type == TEGRA_HDMI_BRICK_CLK) {
		u32 val;
		struct tegra_dc_sor_data *sor = hdmi->sor;
		int div = (tegra_sor_get_link_rate(hdmi->dc) < 340000000) ?
			1 : 2;
		unsigned long rate = clk_get_rate(sor->ref_clk);
		unsigned long parent_rate =
			clk_get_rate(clk_get_parent(sor->ref_clk));

		/* Set sor divider */
		if (rate != DIV_ROUND_UP(parent_rate, div)) {
			rate = DIV_ROUND_UP(parent_rate, div);
			clk_set_rate(sor->ref_clk, rate);
		}

		val = (tegra_sor_get_link_rate(hdmi->dc) < 340000000) ?
			NV_SOR_CLK_CNTRL_DP_LINK_SPEED_G2_7 :
			NV_SOR_CLK_CNTRL_DP_LINK_SPEED_G5_4;

		val |= NV_SOR_CLK_CNTRL_DP_CLK_SEL_SINGLE_PCLK;

		/*
		 * Report brick configuration and rate, so that SOR clock tree
		 * is properly updated. No h/w changes by clock api calls below,
		 * just sync s/w state with brick h/w.
		 */
		rate = rate/NV_SOR_HDMI_BRICK_DIV*NV_SOR_HDMI_BRICK_MUL(val);
		if (clk_get_parent(sor->pad_clk) != sor->ref_clk)
			clk_set_parent(sor->pad_clk, sor->ref_clk);
		clk_set_rate(sor->pad_clk, rate);

#ifdef CONFIG_TEGRA_CORE_DVFS
		/*
		 * Select primary -- HDMI -- DVFS table for SOR clock (if SOR
		 * clock has single DVFS table for all modes, nothing changes).
		 */
		tegra_dvfs_use_alt_freqs_on_clk(sor->sor_clk, false);
#endif

		/* Select sor clock muxes */
#ifdef CONFIG_TEGRA_CLK_FRAMEWORK
		tegra_clk_cfg_ex(sor->sor_clk, TEGRA_CLK_SOR_CLK_SEL, 3);
#else
		clk_set_parent(sor->sor_clk, sor->pad_clk);
#endif

		tegra_dc_writel(hdmi->dc, PIXEL_CLK_DIVIDER_PCD1 |
			SHIFT_CLK_DIVIDER(tegra_hdmi_get_shift_clk_div(hdmi)),
			DC_DISP_DISP_CLOCK_CONTROL);

		hdmi->clk_type = TEGRA_HDMI_BRICK_CLK;
	} else if (clk_type == TEGRA_HDMI_SAFE_CLK) {
		if (!hdmi->dc->initialized) {
			/* Select sor clock muxes */
#ifdef CONFIG_TEGRA_CLK_FRAMEWORK
			tegra_clk_cfg_ex(hdmi->sor->sor_clk,
				TEGRA_CLK_SOR_CLK_SEL, 0);
#else
			clk_set_parent(hdmi->sor->sor_clk, hdmi->sor->safe_clk);
#endif
			hdmi->clk_type = TEGRA_HDMI_SAFE_CLK;
		}
	} else {
		dev_err(&hdmi->dc->ndev->dev, "hdmi: incorrect clk type configured\n");
	}
}

static void tegra_hdmi_config_clk(struct tegra_hdmi *hdmi, u32 clk_type)
{
	if (tegra_dc_is_nvdisplay())
		return tegra_hdmi_config_clk_nvdisplay(hdmi, clk_type);
	else
		return tegra_hdmi_config_clk_t21x(hdmi, clk_type);
}

/* returns exact pixel clock in Hz */
static long __maybe_unused tegra_hdmi_get_pclk(struct tegra_dc_mode *mode)
{
	long h_total, v_total;
	long refresh, pclk;
	h_total = mode->h_active + mode->h_front_porch + mode->h_back_porch +
		mode->h_sync_width;
	v_total = mode->v_active + mode->v_front_porch + mode->v_back_porch +
		mode->v_sync_width;
	refresh = tegra_dc_calc_refresh(mode);
	refresh = DIV_ROUND_CLOSEST(refresh, 1000);

	pclk = h_total * v_total * refresh;

	if (mode->vmode & FB_VMODE_1000DIV1001)
		pclk = pclk * 1000 / 1001;

	return pclk;
}

static inline void tegra_sor_set_ref_clk_rate(struct tegra_dc_sor_data *sor)
{
	unsigned long rate = tegra_sor_get_link_rate(sor->dc);

	clk_set_rate(sor->ref_clk, rate);
}

static long tegra_dc_hdmi_setup_clk_nvdisplay(struct tegra_dc *dc,
					      struct clk *clk)
{
#define MIN_PARENT_CLK	(27000000)
#define DIVIDER_CAP	(128)

	struct clk *parent_clk;
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);
	struct tegra_dc_sor_data *sor = hdmi->sor;
	long parent_clk_rate;
	int yuv_flag = hdmi->dc->mode.vmode & FB_VMODE_YUV_MASK;

	if (!dc->out->parent_clk) {
		dev_err(&dc->ndev->dev,
				"hdmi: failed, no clock name for this node.\n");
		return -EINVAL;
	}

	/* GET THE PARENT  */
	parent_clk = tegra_disp_clk_get(&dc->ndev->dev, dc->out->parent_clk);
	if (IS_ERR_OR_NULL(parent_clk)) {
		dev_err(&dc->ndev->dev, "hdmi: failed to get clock %s\n",
				dc->out->parent_clk);
		return -EINVAL;
	}

	if (!dc->mode.pclk_hz_used)
		dc->mode.pclk = tegra_hdmi_get_pclk(&dc->mode);

	/* Set rate on PARENT */
	if (!dc->initialized) {
		/*
		 * For RGB/YUV444 12bpc, the pclk:orclk ratio should be 2:3.
		 * The SOR refclk vs. pclk dividers should be in a 2:3 ratio if
		 * TMDS <= 340, and in a 4:3 ratio if TMDS > 340. Use a PCLK_DIV
		 * of 3 to comply with these constraints.
		 */
		parent_clk_rate = dc->mode.pclk;
		if ((IS_RGB(yuv_flag) && (yuv_flag == FB_VMODE_Y36)) ||
				(yuv_flag == (FB_VMODE_Y444 | FB_VMODE_Y36)))
			parent_clk_rate *= 3;

		/*
		 * For t18x plldx cannot go below 27MHz.
		 * Real HW limit is lesser though.
		 * 27Mz is chosen to have a safe margin.
		 */
		if (parent_clk_rate < (MIN_PARENT_CLK/DIVIDER_CAP)) {
			dev_err(&dc->ndev->dev, "hdmi: unsupported parent clock rate (%ld).\n",
					parent_clk_rate);
			return -EINVAL;
		}

		while (parent_clk_rate < MIN_PARENT_CLK)
			parent_clk_rate += parent_clk_rate;

		clk_set_rate(parent_clk, parent_clk_rate);

		if (clk == dc->clk)
			clk_set_rate(clk, dc->mode.pclk);
	}

	tegra_sor_safe_clk_enable(sor);
	clk_set_parent(sor->ref_clk, parent_clk);

	if (!dc->initialized)
		tegra_sor_set_ref_clk_rate(sor);

	if (atomic_inc_return(&hdmi->clock_refcount) == 1)
		tegra_sor_clk_enable(sor);

	if (!dc->initialized)
		clk_set_parent(sor->sor_clk, sor->safe_clk);

	tegra_disp_clk_prepare_enable(sor->pad_clk);
	tegra_disp_clk_prepare_enable(sor->sor_clk);

	return tegra_dc_pclk_round_rate(dc, dc->mode.pclk);
}

static void tegra_dc_hdmi_configure_ss(struct tegra_dc *dc, struct clk *clk)
{
#if defined(CONFIG_ARCH_TEGRA_210_SOC)
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);

	if (!dc->pdata->plld2_ss_enable)
		return;

	if (dc->mode.pclk == 148500000 &&
		dc->mode.h_active == 1920 &&
		dc->mode.v_active == 1080 &&
		(tegra_hdmi_find_cea_vic(hdmi) == 31)) {
		tegra210_plld2_configure_ss(true);
	} else {
		tegra210_plld2_configure_ss(false);
	}
#endif
}

static long tegra_dc_hdmi_setup_clk_t21x(struct tegra_dc *dc, struct clk *clk)
{
	struct clk *parent_clk;

	parent_clk = clk_get(NULL, "pll_d2_out0");

	if (!dc->mode.pclk_hz_used)
		dc->mode.pclk = tegra_hdmi_get_pclk(&dc->mode);

	if (IS_ERR_OR_NULL(parent_clk)) {
		dev_err(&dc->ndev->dev, "hdmi: parent clk get failed\n");
		return 0;
	}

	if (!tegra_platform_is_silicon())
		return dc->mode.pclk;

	if (clk == dc->clk) {
		if (clk_get_parent(clk) != parent_clk) {
			if (clk_set_parent(clk, parent_clk)) {
				dev_err(&dc->ndev->dev,
					"hdmi: set dc parent failed\n");
				return 0;
			}
		}
	} else {
		struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);
		struct tegra_dc_sor_data *sor = hdmi->sor;

		if (clk_get_parent(sor->ref_clk) != parent_clk) {
			if (clk_set_parent(sor->ref_clk, parent_clk)) {
				dev_err(&dc->ndev->dev,
					"hdmi: set src switch parent failed\n");
				return 0;
			}
		}
		tegra_dc_hdmi_configure_ss(dc, clk);
	}

	if (dc->initialized)
		goto skip_setup;
	if (clk_get_rate(parent_clk) != dc->mode.pclk)
		clk_set_rate(parent_clk, dc->mode.pclk);
skip_setup:
#ifdef CONFIG_TEGRA_CORE_DVFS
	/*
	 * DC clock divider is controlled by DC driver transparently to clock
	 * framework -- hence, direct call to DVFS with target mode rate. SOR
	 * clock rate in clock tree is properly updated, and can be used for
	 * DVFS update.
	 *
	 * TODO: tegra_hdmi_controller_enable() procedure 1st configures SOR
	 * clock via tegra_hdmi_config_clk(), and then calls this function
	 * that may re-lock parent PLL. That needs to be double-checked:
	 * in general re-locking PLL while the downstream module is already
	 * sourced from it is not recommended. If/when the order of enabling
	 * HDMI controller is changed, we can remove direct DVFS call for SOR
	 * (but for DC it should be kept, anyway).
	 */
	if (clk == dc->clk)
		tegra_dvfs_set_rate(clk, dc->mode.pclk);
	else
		tegra_dvfs_set_rate(clk, clk_get_rate(clk));
#endif

	return tegra_dc_pclk_round_rate(dc, dc->mode.pclk);
}

static long tegra_dc_hdmi_setup_clk(struct tegra_dc *dc, struct clk *clk)
{
	if (tegra_dc_is_nvdisplay())
		return tegra_dc_hdmi_setup_clk_nvdisplay(dc, clk);
	else
		return tegra_dc_hdmi_setup_clk_t21x(dc, clk);
}

static void tegra_dc_hdmi_shutdown(struct tegra_dc *dc)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);

	_tegra_hdmivrr_activate(hdmi, false);
	hdmi->device_shutdown = true;
	tegra_nvhdcp_shutdown(hdmi->nvhdcp);

	return;
}

static void tegra_dc_hdmi_disable(struct tegra_dc *dc)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);

	_tegra_hdmivrr_activate(hdmi, false);

	if (NULL != hdmi->out_ops && NULL != hdmi->out_ops->disable)
		hdmi->out_ops->disable(hdmi);

	hdmi->enabled = false;

	disp_state_extcon_aux_report(hdmi->sor->ctrl_num,
		EXTCON_DISP_AUX_STATE_DISABLED);
	pr_info("Extcon AUX%d(HDMI) disable\n", hdmi->sor->ctrl_num);

#ifdef CONFIG_SWITCH
	switch_set_state(&hdmi->audio_switch, 0);
#endif

	tegra_hdmi_controller_disable(hdmi);
#ifdef CONFIG_TEGRA_HDA_DC
	tegra_hda_disable(hdmi->hda_handle);
#endif

	return;
}

static bool tegra_dc_hdmi_detect(struct tegra_dc *dc)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);
	unsigned long delay = msecs_to_jiffies(HDMI_HPD_DEBOUNCE_DELAY_MS);

	if (dc->out->hotplug_state != TEGRA_HPD_STATE_NORMAL)
		delay = 0;

	if ((tegra_platform_is_sim() || tegra_platform_is_fpga()) &&
		(dc->out->hotplug_state == TEGRA_HPD_STATE_NORMAL)) {
		complete(&dc->hpd_complete);
		return true;
	}

	cancel_delayed_work(&hdmi->hpd_worker);
	schedule_delayed_work(&hdmi->hpd_worker, delay);
	return tegra_dc_hpd(dc);
}

static void tegra_dc_hdmi_suspend(struct tegra_dc *dc)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);

	_tegra_hdmivrr_activate(hdmi, false);

	if (hdmi->pdata->hdmi2fpd_bridge_enable)
		hdmi2fpd_suspend(dc);

	if (hdmi->out_ops && hdmi->out_ops->suspend)
		hdmi->out_ops->suspend(hdmi);

	if (dc->out->flags & TEGRA_DC_OUT_HOTPLUG_WAKE_LP0) {
		int wake_irq = gpio_to_irq(dc->out->hotplug_gpio);
		int ret;

		ret = enable_irq_wake(wake_irq);
		if (ret < 0) {
			dev_err(&dc->ndev->dev,
			"%s: Couldn't enable HDMI wakeup, irq=%d, error=%d\n",
			__func__, wake_irq, ret);
		}
	}

	atomic_set(&hdmi->suspended, 1);
}

static void tegra_dc_hdmi_resume(struct tegra_dc *dc)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);

	atomic_set(&hdmi->suspended, 0);

	if (dc->out->flags & TEGRA_DC_OUT_HOTPLUG_WAKE_LP0)
		disable_irq_wake(gpio_to_irq(dc->out->hotplug_gpio));

	if (hdmi->pdata->hdmi2fpd_bridge_enable)
		hdmi2fpd_resume(dc);

	if (hdmi->out_ops && hdmi->out_ops->resume)
		hdmi->out_ops->resume(hdmi);

	if (tegra_platform_is_sim() &&
		(dc->out->hotplug_state == TEGRA_HPD_STATE_NORMAL))
		return;

	cancel_delayed_work(&hdmi->hpd_worker);

	reinit_completion(&dc->hpd_complete);

	/* If resume happens with a non-VRR monitor, the HPD
	   worker will correct the mode based on the new EDID */
	_tegra_hdmivrr_activate(hdmi, true);

	schedule_delayed_work(&hdmi->hpd_worker,
				msecs_to_jiffies(HDMI_HPD_DEBOUNCE_DELAY_MS));

	wait_for_completion(&dc->hpd_complete);
}

static int tegra_dc_hdmi_set_hdr(struct tegra_dc *dc)
{
	u16 ret = 0;
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);

	/* If the generic infoframe is not for HDR, return */
	if (hdmi->pdata->generic_infoframe_type != HDMI_INFOFRAME_TYPE_HDR)
		return 0;

	/* If the sink isn't HDR capable, return */
	ret = tegra_edid_get_ex_hdr_cap(hdmi->edid);
	if (!ret)
		return 0;

	/* Cancel any pending hdr-exit work */
	if (dc->hdr.enabled)
		cancel_delayed_work_sync(&hdmi->hdr_worker);

	tegra_hdmi_hdr_infoframe(hdmi);

	/*
	 *If hdr is disabled then send HDR infoframe for
	 *2 secs with SDR eotf and then stop
	 */
	if (dc->hdr.enabled)
		return 0;

	schedule_delayed_work(&hdmi->hdr_worker,
		msecs_to_jiffies(HDMI_HDR_INFOFRAME_STOP_TIMEOUT_MS));

	return 0;
}

static void tegra_hdmi_hdr_worker(struct work_struct *work)
{
	u32 val = 0;
	struct tegra_hdmi *hdmi = container_of(to_delayed_work(work),
				struct tegra_hdmi, hdr_worker);

	if (hdmi && hdmi->enabled) {
		/* If hdr re-enabled within 2s, return.
		 * Note this an extra safety check since
		 * we should have already cancelled this work */
		if (hdmi->dc->hdr.enabled)
			return;
		/* Read the current regsiter value to restore the bits */
		val = tegra_sor_readl(hdmi->sor, NV_SOR_HDMI_GENERIC_CTRL);

		/* Set val to disable generic infoframe */
		val &= ~NV_SOR_HDMI_GENERIC_CTRL_ENABLE_YES;

		tegra_sor_writel(hdmi->sor, NV_SOR_HDMI_GENERIC_CTRL, val);
	}
	return;
}

static int tegra_dc_hdmi_ddc_enable(struct tegra_dc *dc)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);
	_tegra_hdmi_ddc_enable(hdmi);
	return 0;
}

static int tegra_dc_hdmi_ddc_disable(struct tegra_dc *dc)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);
	_tegra_hdmi_ddc_disable(hdmi);
	return 0;
}

static void tegra_dc_hdmi_modeset_notifier(struct tegra_dc *dc)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);

	/* In case of seamless display, kernel carries forward BL config */
	if (dc->initialized)
		return;

	tegra_hdmi_get(dc);
	tegra_dc_io_start(dc);

	/* disable hdmi2.x config on host and monitor */
	if (tegra_sor_get_link_rate(dc) > 340000000) {
		if (tegra_edid_is_scdc_present(dc->edid))
			tegra_hdmi_v2_x_mon_config(hdmi, true);
		tegra_hdmi_v2_x_host_config(hdmi, true);
	} else {
		if (tegra_edid_is_scdc_present(dc->edid))
			tegra_hdmi_v2_x_mon_config(hdmi, false);
		tegra_hdmi_v2_x_host_config(hdmi, false);
	}
	if (tegra_dc_is_t21x()) {
		if (!(dc->mode.vmode & FB_VMODE_SET_YUV_MASK))
			_tegra_dc_cmu_enable(dc,
				dc->mode.vmode & FB_VMODE_LIMITED_RANGE);
	}
	tegra_dc_io_end(dc);
	tegra_hdmi_put(dc);
}

int tegra_hdmi_get_hotplug_state(struct tegra_hdmi *hdmi)
{
	rmb();
	return hdmi->dc->out->hotplug_state;
}

void tegra_hdmi_set_hotplug_state(struct tegra_hdmi *hdmi, int new_hpd_state)
{
	struct tegra_dc *dc = hdmi->dc;
	int hotplug_state;

	rmb();
	hotplug_state = dc->out->hotplug_state;
	dc->out->prev_hotplug_state = hotplug_state;

	if (hotplug_state == TEGRA_HPD_STATE_NORMAL &&
			new_hpd_state != TEGRA_HPD_STATE_NORMAL &&
			dc->hotplug_supported) {
		disable_irq(gpio_to_irq(dc->out->hotplug_gpio));
	} else if (hotplug_state != TEGRA_HPD_STATE_NORMAL &&
			new_hpd_state == TEGRA_HPD_STATE_NORMAL &&
			dc->hotplug_supported) {
		enable_irq(gpio_to_irq(dc->out->hotplug_gpio));
	}

	dc->out->hotplug_state = new_hpd_state;
	wmb();
	/*
	 * sw controlled plug/unplug.
	 * wait for any already executing hpd worker thread.
	 * No debounce delay, schedule immedately
	 */
	reinit_completion(&dc->hpd_complete);
	cancel_delayed_work_sync(&hdmi->hpd_worker);
	schedule_delayed_work(&hdmi->hpd_worker, 0);
	wait_for_completion(&dc->hpd_complete);
}


#ifdef CONFIG_DEBUG_FS
/* show current hpd state */
static int tegra_hdmi_hotplug_dbg_show(struct seq_file *m, void *unused)
{
	struct tegra_hdmi *hdmi = m->private;
	struct tegra_dc *dc = hdmi->dc;

	if (WARN_ON(!hdmi || !dc || !dc->out))
		return -EINVAL;
	/* make sure we see updated hotplug_state value */
	rmb();
	seq_printf(m, "hdmi hpd state: %d\n", dc->out->hotplug_state);

	return 0;
}

/*
 * sw control for hpd.
 * 0 is normal state, hw drives hpd.
 * -1 is force deassert, sw drives hpd.
 * 1 is force assert, sw drives hpd.
 * before releasing to hw, sw must ensure hpd state is normal i.e. 0
 */
static ssize_t tegra_hdmi_hotplug_dbg_write(struct file *file,
					const char __user *addr,
					size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data;
	struct tegra_hdmi *hdmi = m->private;
	struct tegra_dc *dc = hdmi->dc;
	long new_hpd_state;
	int ret;

	if (WARN_ON(!hdmi || !dc || !dc->out))
		return -EINVAL;

	ret = kstrtol_from_user(addr, len, 10, &new_hpd_state);
	if (ret < 0)
		return ret;

	tegra_hdmi_set_hotplug_state(hdmi, new_hpd_state);
	return len;
}

static int tegra_hdmi_hotplug_dbg_open(struct inode *inode, struct file *file)
{
	return single_open(file, tegra_hdmi_hotplug_dbg_show, inode->i_private);
}

/* show current hpd/vhotplug state */
static int tegra_hdmi_vhotplug_dbg_show(struct seq_file *m, void *unused)
{
	struct tegra_hdmi *hdmi = m->private;
	struct tegra_dc *dc;

	if (WARN_ON(!hdmi))
		return -EINVAL;

	dc = hdmi->dc;

	if (WARN_ON(!dc || !dc->out))
		return -EINVAL;

	/* make sure we see updated hotplug_state value */
	rmb();
	seq_printf(m, "hdmi vhotplug state: %d\n", dc->out->vrr_hotplug_state);

	return 0;
}

/*
 * sw control for hpd/vhotplug.
 * 0 is normal state, hw drives hpd/vhotplug.
 * -1 is force deassert, sw drives hpd/vhotplug.
 * 1 is force assert, sw drives hpd/vhotplug.
 * before releasing to hw, sw must ensure hpd/vhotplug state is normal i.e. 0
 */
static ssize_t tegra_hdmi_vhotplug_dbg_write(struct file *file,
					const char __user *addr,
					size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data;
	struct tegra_hdmi *hdmi = m->private;
	struct tegra_dc *dc;
	long new_hpd_state;
	int ret;
	static bool free_vrr;

	if (WARN_ON(!hdmi))
		return -EINVAL;

	dc = hdmi->dc;

	if (WARN_ON(!dc || !dc->out))
		return -EINVAL;

	ret = kstrtol_from_user(addr, len, 10, &new_hpd_state);
	if (ret < 0)
		return ret;

	if (new_hpd_state == TEGRA_HPD_STATE_FORCE_ASSERT) {

		if (!dc->out->vrr) {
			dc->out->vrr = devm_kzalloc(&dc->ndev->dev,
						    sizeof(struct tegra_vrr),
						    GFP_KERNEL);
			if (!dc->out->vrr) {
				dev_err(&dc->ndev->dev, "not enough memory\n");
				return -ENOMEM;
			}

			free_vrr = true;
		}

		dc->out->vrr->capability = 1;

	} else if (new_hpd_state == TEGRA_HPD_STATE_FORCE_DEASSERT) {

		if (free_vrr && dc->out->vrr) {
			devm_kfree(&dc->ndev->dev, dc->out->vrr);
			dc->out->vrr = NULL;
		}

		if (dc->out->vrr)
			dc->out->vrr->capability = 0;
	}

	dc->out->vrr_hotplug_state = new_hpd_state;

	tegra_hdmi_set_hotplug_state(hdmi, new_hpd_state);

	return len;
}

static int tegra_hdmi_vhotplug_dbg_open(struct inode *inode,
					  struct file *file)
{
	return single_open(file, tegra_hdmi_vhotplug_dbg_show,
			   inode->i_private);
}

static const struct file_operations tegra_hdmi_hotplug_dbg_ops = {
	.open = tegra_hdmi_hotplug_dbg_open,
	.read = seq_read,
	.write = tegra_hdmi_hotplug_dbg_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations tegra_hdmi_vhotplug_dbg_ops = {
	.open = tegra_hdmi_vhotplug_dbg_open,
	.read = seq_read,
	.write = tegra_hdmi_vhotplug_dbg_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static void tegra_hdmi_ddc_power_toggle(struct tegra_hdmi *dc_hdmi, int value)
{
	if (!dc_hdmi || !dc_hdmi->sor)
		return;

	if (value == 0)
		_tegra_hdmi_ddc_disable(dc_hdmi);
	else if (value == 1)
		_tegra_hdmi_ddc_enable(dc_hdmi);
}

static int tegra_hdmi_ddc_power_toggle_dbg_show(struct seq_file *m,
					void *unused)
{
	struct tegra_hdmi *hdmi = m->private;
	struct tegra_dc *dc = hdmi->dc;

	if (WARN_ON(!hdmi || !dc || !dc->out))
		return -EINVAL;
	/* make sure we see updated ddc_refcount value */
	rmb();
	seq_printf(m, "ddc_refcount: %d\n", hdmi->ddc_refcount);

	return 0;
}

static ssize_t tegra_hdmi_ddc_power_toggle_dbg_write(struct file *file,
					const char __user *addr,
					size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data;
	struct tegra_hdmi *hdmi = m->private;
	struct tegra_dc *dc = hdmi->dc;
	long value;
	int ret;

	if (WARN_ON(!hdmi || !dc || !dc->out))
		return -EINVAL;

	ret = kstrtol_from_user(addr, len, 10, &value);
	if (ret < 0)
		return ret;

	tegra_hdmi_ddc_power_toggle(hdmi, value);

	return len;
}

static int tegra_hdmi_ddc_power_toggle_dbg_open(struct inode *inode,
					struct file *file)
{
	return single_open(file, tegra_hdmi_ddc_power_toggle_dbg_show,
		inode->i_private);
}

static const struct file_operations tegra_hdmi_ddc_power_toggle_dbg_ops = {
	.open = tegra_hdmi_ddc_power_toggle_dbg_open,
	.read = seq_read,
	.write = tegra_hdmi_ddc_power_toggle_dbg_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int tegra_hdmi_status_dbg_show(struct seq_file *m, void *unused)
{
	struct tegra_hdmi *hdmi = m->private;
	struct tegra_dc *dc;

	if (WARN_ON(!hdmi))
		return -EINVAL;

	dc = hdmi->dc;

	if (WARN_ON(!dc || !dc->out))
		return -EINVAL;

	seq_printf(m, "hotplug state: %d\n", tegra_dc_hpd(dc));
	seq_printf(m, "SCDC present: %d\n",
		tegra_edid_is_scdc_present(hdmi->edid));
	seq_printf(m, "Forum VSDB present: %d\n",
		tegra_edid_is_hfvsdb_present(hdmi->edid));
	seq_printf(m, "YCbCr4:2:0 VDB present: %d\n",
		tegra_edid_is_420db_present(hdmi->edid));

	return 0;
}

static int tegra_hdmi_status_dbg_open(struct inode *inode, struct file *file)
{
	return single_open(file, tegra_hdmi_status_dbg_show,
		inode->i_private);
}

static const struct file_operations tegra_hdmi_status_dbg_ops = {
	.open = tegra_hdmi_status_dbg_open,
	.read = seq_read,
	.release = single_release,
};

static void tegra_hdmi_debugfs_init(struct tegra_hdmi *hdmi)
{
	struct dentry *ret;
	char debug_dirname[CHAR_BUF_SIZE_MAX] = "tegra_hdmi";

	if (hdmi_instance) {
		snprintf(debug_dirname, sizeof(debug_dirname),
			"tegra_hdmi%d", hdmi_instance);
	}

	hdmi->debugdir = debugfs_create_dir(debug_dirname, NULL);
	if (IS_ERR_OR_NULL(hdmi->debugdir)) {
		dev_err(&hdmi->dc->ndev->dev, "could not create hdmi%d debugfs\n",
			hdmi_instance);
		return;
	}
	ret = debugfs_create_file("hotplug", 0444, hdmi->debugdir,
				hdmi, &tegra_hdmi_hotplug_dbg_ops);
	if (IS_ERR_OR_NULL(ret))
		goto fail;
	ret = debugfs_create_file("hdmi_status", 0444, hdmi->debugdir,
				hdmi, &tegra_hdmi_status_dbg_ops);
	if (IS_ERR_OR_NULL(ret))
		goto fail;
	ret = debugfs_create_file("ddc_power_toggle", 0444, hdmi->debugdir,
				hdmi, &tegra_hdmi_ddc_power_toggle_dbg_ops);
	if (IS_ERR_OR_NULL(ret))
		goto fail;
	ret = debugfs_create_file("vhotplug", 0444, hdmi->debugdir,
				hdmi, &tegra_hdmi_vhotplug_dbg_ops);
	if (IS_ERR_OR_NULL(ret))
		goto fail;
	return;
fail:
	dev_err(&hdmi->dc->ndev->dev, "could not create hdmi%d debugfs\n",
		hdmi_instance);
	tegra_hdmi_debugfs_remove(hdmi);
	return;
}

static void tegra_hdmi_debugfs_remove(struct tegra_hdmi *hdmi)
{
	debugfs_remove_recursive(hdmi->debugdir);
	hdmi->debugdir = NULL;
}

#else
static void tegra_hdmi_debugfs_init(struct tegra_hdmi *hdmi)
{
	return;
}
static void tegra_hdmi_debugfs_remove(struct tegra_hdmi *hdmi)
{
	return;
}
#endif

static bool tegra_dc_hdmi_hpd_state(struct tegra_dc *dc)
{
	int sense;
	int level;
	bool hpd;

	if (WARN_ON(!dc || !dc->out))
		return false;

	if (tegra_platform_is_sim())
		return true;

	level = gpio_get_value_cansleep(dc->out->hotplug_gpio);

	sense = dc->out->flags & TEGRA_DC_OUT_HOTPLUG_MASK;

	hpd = (sense == TEGRA_DC_OUT_HOTPLUG_HIGH && level) ||
		(sense == TEGRA_DC_OUT_HOTPLUG_LOW && !level);

	return hpd;
}

static void tegra_dc_hdmi_vrr_enable(struct tegra_dc *dc, bool enable)
{
	struct tegra_vrr *vrr  = dc->out->vrr;

	if (!vrr || !vrr->capability)
		return;

	if (!(dc->mode.vmode & FB_VMODE_VRR)) {
		WARN(enable, "VRR enable request in non-VRR mode\n");
		return;
	}

	vrr->enable = enable;
}

static void tegra_dc_hdmi_postpoweron(struct tegra_dc *dc)
{
	_tegra_hdmivrr_activate(tegra_dc_get_outdata(dc), true);
}

static void tegra_dc_hdmi_sor_sleep(struct tegra_dc *dc)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);

	if (hdmi->sor->sor_state == SOR_ATTACHED)
		tegra_dc_sor_sleep(hdmi->sor);
}

static u32 tegra_dc_hdmi_sor_crc_check(struct tegra_dc *dc)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);

	return tegra_dc_sor_debugfs_get_crc(hdmi->sor, NULL);
}

static void tegra_dc_hdmi_sor_crc_toggle(struct tegra_dc *dc,
	u32 val)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);

	tegra_dc_sor_toggle_crc(hdmi->sor, val);
}

static int tegra_dc_hdmi_get_sor_ctrl_num(struct tegra_dc *dc)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);

	return (!hdmi) ? -ENODEV : tegra_sor_get_ctrl_num(hdmi->sor);
}

static int tegra_dc_hdmi_sor_crc_en_dis(struct tegra_dc *dc,
				    struct tegra_dc_ext_crc_or_params *params,
				    bool en)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);

	if (params->out_type != TEGRA_DC_EXT_HDMI)
		return -EINVAL;

	tegra_dc_sor_crc_en_dis(hdmi->sor, params->sor_params, en);

	return 0;
}

static int tegra_dc_hdmi_sor_crc_en(struct tegra_dc *dc,
				    struct tegra_dc_ext_crc_or_params *params)
{
	return tegra_dc_hdmi_sor_crc_en_dis(dc, params, true);
}

static int tegra_dc_hdmi_sor_crc_dis(struct tegra_dc *dc,
				     struct tegra_dc_ext_crc_or_params *params)
{
	return tegra_dc_hdmi_sor_crc_en_dis(dc, params, false);
}

static int tegra_dc_hdmi_sor_crc_get(struct tegra_dc *dc, u32 *crc)
{
	struct tegra_hdmi *hdmi = tegra_dc_get_outdata(dc);

	return tegra_dc_sor_crc_get(hdmi->sor, crc);
}

struct tegra_dc_out_ops tegra_dc_hdmi2_0_ops = {
	.init = tegra_dc_hdmi_init,
	.hotplug_init = tegra_dc_hdmi_hpd_init,
	.destroy = tegra_dc_hdmi_destroy,
	.enable = tegra_dc_hdmi_enable,
	.disable = tegra_dc_hdmi_disable,
	.setup_clk = tegra_dc_hdmi_setup_clk,
	.detect = tegra_dc_hdmi_detect,
	.shutdown = tegra_dc_hdmi_shutdown,
	.suspend = tegra_dc_hdmi_suspend,
	.resume = tegra_dc_hdmi_resume,
	.ddc_enable = tegra_dc_hdmi_ddc_enable,
	.ddc_disable = tegra_dc_hdmi_ddc_disable,
	.modeset_notifier = tegra_dc_hdmi_modeset_notifier,
	.mode_filter = tegra_hdmi_fb_mode_filter,
	.hpd_state = tegra_dc_hdmi_hpd_state,
	.vrr_enable = tegra_dc_hdmi_vrr_enable,
	.vrr_update_monspecs = tegra_hdmivrr_update_monspecs,
	.set_hdr = tegra_dc_hdmi_set_hdr,
	.set_avi = tegra_dc_hdmi_set_avi,
	.postpoweron = tegra_dc_hdmi_postpoweron,
	.shutdown_interface = tegra_dc_hdmi_sor_sleep,
	.get_crc = tegra_dc_hdmi_sor_crc_check,
	.toggle_crc = tegra_dc_hdmi_sor_crc_toggle,
	.get_connector_instance = tegra_dc_hdmi_get_sor_ctrl_num,
	.crc_en = tegra_dc_hdmi_sor_crc_en,
	.crc_dis = tegra_dc_hdmi_sor_crc_dis,
	.crc_get = tegra_dc_hdmi_sor_crc_get,
};
