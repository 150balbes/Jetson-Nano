/*
 * drivers/video/tegra/dc/nvdisp/nvdisp_bandwidth.c
 *
 * Copyright (c) 2016-2018, NVIDIA CORPORATION, All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/platform/tegra/bwmgr_mc.h>
#include <linux/platform/tegra/emc_bwmgr.h>
#include <linux/platform/tegra/isomgr.h>
#include <linux/platform/tegra/latency_allowance.h>

#include "dc.h"
#include "dc_priv.h"
#include "nvdisp.h"
#include "dc_common.h"

#ifdef CONFIG_TEGRA_ISOMGR

/* Output id that we pass to tegra_dc_ext_process_bandwidth_negotiate */
#define NVDISP_BW_OUTPUT_ID		0

/* Global bw info shared across all heads */
static struct nvdisp_isoclient_bw_info ihub_bw_info;

static int tegra_nvdisp_set_latency_allowance(u32 bw, u32 emc_freq)
{
	struct dc_to_la_params disp_params;
	int ret = 0;

	if (bw == 0)
		return ret;

	/* Zero out this struct since it's ignored by the LA/PTSA driver. */
	memset(&disp_params, 0, sizeof(disp_params));

	/* Our bw is in KB/s, but LA takes MB/s. Round up to the next MB/s. */
	if (bw != U32_MAX)
		bw = bw / 1000 + 1;

	return tegra_set_disp_latency_allowance(TEGRA_LA_NVDISPLAYR,
					emc_freq,
					bw,
					disp_params);
}

static int tegra_nvdisp_program_final_bw_settings(
				struct nvdisp_bandwidth_config *cur_config,
				u32 final_iso_bw,
				u32 final_total_bw,
				u32 final_emc,
				u32 final_hubclk,
				bool before_win_update)
{
	bool update_la_ptsa = false;
	int ret = 0;

	if (!before_win_update && final_hubclk != cur_config->hubclk) {
		ret = clk_set_rate(hubclk, final_hubclk);
		if (ret) {
			pr_err("%s: failed to set hubclk=%u Hz\n", __func__,
				final_hubclk);
			return ret;
		}

		cur_config->hubclk = final_hubclk;
	}

	if (final_iso_bw != cur_config->iso_bw) {
		if (!tegra_isomgr_realize(ihub_bw_info.isomgr_handle)) {
			pr_err("%s: failed to realize %u KB/s\n", __func__,
				final_iso_bw);
			return -EINVAL;
		}

		cur_config->iso_bw = final_iso_bw;
		update_la_ptsa = true;
	}

	if (final_emc != cur_config->emc_la_floor) {
		/*
		 * tegra_bwmgr_set_emc() takes in the DRAM frequency. We need
		 * to use the conversion factor to properly convert the required
		 * EMC frequency to its corresponding DRAM frequency.
		 */
		int freq_factor = bwmgr_get_emc_to_dram_freq_factor();

		ret = tegra_bwmgr_set_emc(ihub_bw_info.bwmgr_handle,
					final_emc * freq_factor,
					TEGRA_BWMGR_SET_EMC_FLOOR);
		if (ret) {
			pr_err("%s: failed to set EMC floor=%u Hz\n", __func__,
				final_emc);
			return ret;
		}

		cur_config->emc_la_floor = final_emc;
		update_la_ptsa = true;
	}

	if (update_la_ptsa) {
		/*
		 * If either out ISO bw requirement or the EMC floor has
		 * changed, we need to update LA/PTSA. The bw value that we pass
		 * to the LA/PTSA driver should not include the catchup factor.
		 */
		ret = tegra_nvdisp_set_latency_allowance(final_total_bw,
								final_emc);
		if (ret) {
			pr_err("%s: LA/PTSA failed w/ bw=%u KB/s,freq=%u Hz\n",
				__func__, final_total_bw, final_emc);
			return ret;
		}
	}

	if (before_win_update && final_hubclk != cur_config->hubclk) {
		ret = clk_set_rate(hubclk, final_hubclk);
		if (ret) {
			pr_err("%s: failed to set hubclk=%u Hz\n", __func__,
				final_hubclk);
			return ret;
		}

		cur_config->hubclk = final_hubclk;
	}

	return ret;
}

int tegra_nvdisp_program_bandwidth(struct tegra_dc *dc,
				u32 new_iso_bw,
				u32 new_total_bw,
				u32 new_emc,
				u32 new_hubclk,
				bool before_win_update)
{
	/*
	 * This function is responsible for updating the ISO bw, EMC floor,
	 * LA/PTSA, and hubclk values both before and after the current window
	 * update. In the two below cases, let X represent the current frame
	 * configuration and X' represent the new proposed frame configuration.
	 *
	 * A) Before the window update actually occurs, display needs to ensure
	 *    that the four aforementioned values are compatible with both X and
	 *    X'. This is due to the fact that any updates to these values take
	 *    effect immediately and aren't latched to any kind of frame
	 *    boundary. In order to update these values appropriately, display
	 *    will simply program the max across both X and X'.
	 * B) After the window update occurs and the new state has promoted,
	 *    display can program all the new values associated with X' as long
	 *    as they don't violate the ISO bw requirements that are currently
	 *    in place.
	 */

	struct nvdisp_bandwidth_config *cur_config = &ihub_bw_info.cur_config;
	u32 final_iso_bw = 0;
	u32 final_total_bw = 0;
	u32 final_emc = 0;
	u32 final_hubclk = 0;
	int ret = 0;

	if (IS_ERR_OR_NULL(ihub_bw_info.isomgr_handle) ||
				IS_ERR_OR_NULL(ihub_bw_info.bwmgr_handle))
		return -EINVAL;

	final_iso_bw = cur_config->iso_bw;
	final_total_bw = cur_config->total_bw;
	final_emc = cur_config->emc_la_floor;
	final_hubclk = cur_config->hubclk;

	if (before_win_update) { /* Case A */
		bool update_bw = false;

		/*
		 * ISO clients can only realize exactly what they have already
		 * reserved. The ISO bw that display has currently reserved is
		 * always guaranteed to be at least the bw needed for the
		 * proposed configuration since we aggregate bw reservations
		 * during PROPOSE.
		 */
		if (new_iso_bw > cur_config->iso_bw) {
			final_iso_bw = ihub_bw_info.reserved_bw;
			update_bw = true;
		}

		final_emc = max(final_emc, new_emc);
		if (update_bw)
			final_emc = max(final_emc, ihub_bw_info.emc_at_res_bw);

		final_hubclk = max(final_hubclk, new_hubclk);
		if (update_bw)
			final_hubclk =
			max(final_hubclk, ihub_bw_info.hubclk_at_res_bw);
	} else { /* Case B */
		u32 max_bw = tegra_nvdisp_get_max_pending_bw(dc);
		if (new_iso_bw >= max_bw &&
					new_iso_bw < cur_config->iso_bw) {
			/*
			 * Client's latency tolerance is ignored by isomgr. Pass
			 * in a dummy value of 1000 usec.
			 */
			if (!tegra_isomgr_reserve(ihub_bw_info.isomgr_handle,
						new_iso_bw,
						1000)) {
				pr_err("%s: failed to reserve %u KB/s\n",
					__func__, new_iso_bw);
				return -EINVAL;
			}

			ihub_bw_info.reserved_bw = new_iso_bw;
			ihub_bw_info.emc_at_res_bw = new_emc;
			ihub_bw_info.hubclk_at_res_bw = new_hubclk;
			cur_config->total_bw = new_total_bw;

			final_iso_bw = new_iso_bw;
			final_total_bw = new_total_bw;
			final_emc = new_emc;
			final_hubclk = new_hubclk;
		}
	}

	ret = tegra_nvdisp_program_final_bw_settings(cur_config,
						final_iso_bw,
						final_total_bw,
						final_emc,
						final_hubclk,
						before_win_update);

	trace_display_imp_bw_programmed(dc->ctrl_num, final_iso_bw,
					final_total_bw, final_emc,
					final_hubclk);

	return ret;
}

void tegra_nvdisp_init_bandwidth(struct tegra_dc *dc)
{
	/*
	 * Use the max config settings. These values will eventually be adjusted
	 * through IMP, if the client supports it.
	 */
	struct nvdisp_bandwidth_config *max_bw_config;
	u32 new_iso_bw = 0;
	u32 new_total_bw = 0;
	u32 new_emc = 0;
	u32 new_hubclk = 0;
	bool before_win_update = true;

	if (!tegra_platform_is_silicon())
		return;

	if (IS_ERR_OR_NULL(ihub_bw_info.isomgr_handle) ||
		IS_ERR_OR_NULL(ihub_bw_info.bwmgr_handle))
		return;

	max_bw_config = &ihub_bw_info.max_config;
	new_iso_bw = max_bw_config->iso_bw;
	new_total_bw = max_bw_config->total_bw;
	new_emc = max_bw_config->emc_la_floor;
	new_hubclk = max_bw_config->hubclk;

	memset(&ihub_bw_info.cur_config, 0, sizeof(ihub_bw_info.cur_config));
	ihub_bw_info.reserved_bw = 0;

	tegra_nvdisp_negotiate_reserved_bw(dc,
				new_iso_bw,
				new_total_bw,
				new_emc,
				new_hubclk);
	tegra_nvdisp_program_bandwidth(dc,
				new_iso_bw,
				new_total_bw,
				new_emc,
				new_hubclk,
				before_win_update);
}

void tegra_nvdisp_clear_bandwidth(struct tegra_dc *dc)
{
	u32 new_iso_bw = 0;
	u32 new_total_bw = 0;
	u32 new_emc = 0;
	u32 new_hubclk = 0;
	bool before_win_update = false;

	if (!tegra_platform_is_silicon())
		return;

	tegra_nvdisp_program_bandwidth(dc,
				new_iso_bw,
				new_total_bw,
				new_emc,
				new_hubclk,
				before_win_update);
}

/*
 * tegra_nvdisp_calc_min_bandwidth - returns the minimum dedicated ISO bw
 *
 * @dc		dc instance
 *
 * @retval	minimum dedicated ISO bw
 */
long tegra_nvdisp_calc_min_bandwidth(struct tegra_dc *dc)
{
	return ihub_bw_info.max_config.iso_bw;
}

int tegra_nvdisp_negotiate_reserved_bw(struct tegra_dc *dc,
				u32 new_iso_bw,
				u32 new_total_bw,
				u32 new_emc,
				u32 new_hubclk)
{
	/*
	 * There are two possible cases:
	 * A) If the proposed ISO bw is greater than the available bw, return an
	 *    error.
	 * B) If the proposed ISO bw is greater than the bw that is currently
	 *    reserved, try to reserve the proposed bw. Else, the current
	 *    reserved bw is already sufficient.
	 *
	 * This function is only responsible for reserving bw, NOT realizing it.
	 */

	int ret = 0;

	if (IS_ERR_OR_NULL(ihub_bw_info.isomgr_handle) ||
				IS_ERR_OR_NULL(ihub_bw_info.bwmgr_handle)) {
		ret = -EINVAL;
		goto exit;
	}

	if (new_iso_bw > ihub_bw_info.available_bw) { /* Case A */
		pr_err("%s: requested %u KB/s > available %u KB/s",
			__func__, new_iso_bw, ihub_bw_info.available_bw);
		ret = -E2BIG;
		goto exit;
	}

	if (new_iso_bw > ihub_bw_info.reserved_bw) { /* Case B */
		/*
		 * Client's latency tolerance is ignored by isomgr. Pass in a
		 * dummy value of 1000 usec.
		 */
		if (!tegra_isomgr_reserve(ihub_bw_info.isomgr_handle,
						new_iso_bw,
						1000)) {
			pr_err("%s: failed to reserve %u KB/s\n", __func__,
				new_iso_bw);
			ret = -EINVAL;
			goto exit;
		}

		ihub_bw_info.reserved_bw = new_iso_bw;
		ihub_bw_info.emc_at_res_bw = new_emc;
		ihub_bw_info.hubclk_at_res_bw = new_hubclk;
		ihub_bw_info.cur_config.total_bw = new_total_bw;

		trace_display_imp_bw_reserved(dc->ctrl_num, new_iso_bw,
						new_total_bw, new_emc,
						new_hubclk);
	}

exit:
	return ret;
}

static void tegra_nvdisp_bandwidth_renegotiate(void *p, u32 avail_bw)
{
	struct tegra_dc_bw_data data;
	struct nvdisp_isoclient_bw_info *bw_info = p;

	if (!bw_info) {
		pr_err("%s: bw_info is NULL!\n", __func__);
		return;
	}

	if (IS_ERR_OR_NULL(bw_info->isomgr_handle) ||
					IS_ERR_OR_NULL(bw_info->bwmgr_handle))
		return;

	mutex_lock(&tegra_nvdisp_lock);

	if (bw_info->available_bw == avail_bw) {
		mutex_unlock(&tegra_nvdisp_lock);
		return;
	}

	data.total_bw = tegra_isomgr_get_total_iso_bw(TEGRA_ISO_CLIENT_DISP_0);
	data.avail_bw = avail_bw;
	data.resvd_bw = bw_info->reserved_bw;

	tegra_dc_ext_process_bandwidth_renegotiate(NVDISP_BW_OUTPUT_ID, &data);

	bw_info->available_bw = avail_bw;

	mutex_unlock(&tegra_nvdisp_lock);
}

/*
 * tegra_nvdisp_bandwidth_attach - save reference to ihub_bw_info
 *
 * @dc	dc instance
 */
void tegra_nvdisp_bandwidth_attach(struct tegra_dc *dc)
{
	if (!tegra_platform_is_silicon())
		return;

	if (!dc) {
		pr_err("%s: dc is NULL!\n", __func__);
		return;
	}

	dc->ihub_bw_info = &ihub_bw_info;
}

void tegra_nvdisp_get_max_bw_cfg(struct nvdisp_bandwidth_config *max_cfg)
{
	if (!max_cfg)
		return;

	*max_cfg = ihub_bw_info.max_config;
}

static u64 tegra_nvdisp_bw_get_emc(u64 requested_emc_floor)
{
	u64 final_emc_floor = requested_emc_floor;

	if (final_emc_floor == U64_MAX)
		final_emc_floor = tegra_bwmgr_get_max_emc_rate() /
				bwmgr_get_emc_to_dram_freq_factor();

	return final_emc_floor;
}

static u64 tegra_nvdisp_bw_get_hubclk(u64 requested_hubclk)
{
	u64 final_hubclk = requested_hubclk;

	if (final_hubclk == U64_MAX)
		final_hubclk = (u64)clk_round_rate(hubclk, ULONG_MAX);

	return final_hubclk;
}

/*
 * Registers the max possible bandwidth configuration from platform data
 * Return 0 on success and -E2BIG/ENOENT/ENOMEM on failure
 */
static int tegra_nvdisp_bandwidth_register_max_config(
					enum tegra_iso_client iso_client)
{
	struct nvdisp_bandwidth_config *bw_cfg_table;
	struct nvdisp_imp_table *imp_table;
	tegra_isomgr_handle isomgr_handle = NULL;
	u32 max_emc_rate, emc_to_dram_factor;
	u32 total_iso_bw;
	bool found_max_cfg = false;
	int ret = 0, i;

	/* DRAM frequency (Hz) */
	max_emc_rate = tegra_bwmgr_get_max_emc_rate();

	/* conversion factor */
	emc_to_dram_factor = bwmgr_get_emc_to_dram_freq_factor();

	total_iso_bw = tegra_isomgr_get_total_iso_bw(iso_client);

	imp_table = tegra_dc_common_get_imp_table();
	if (!imp_table || imp_table->num_settings <= 0)
		return -ENOENT;

	/*
	 * WAR: Some platforms, such as tegrasim, expose themselves as silicon,
	 * but do not correctly model MC with respect to the max EMC rate and
	 * total available ISO bw. For these platforms, pick the first entry in
	 * the IMP table by default, and get out.
	 */
	if (max_emc_rate == 0 || total_iso_bw == 0) {
		imp_table->boot_setting = imp_table->settings;
		return 0;
	}

	bw_cfg_table = kcalloc(imp_table->num_settings, sizeof(*bw_cfg_table),
				GFP_KERNEL);
	if (!bw_cfg_table)
		return -ENOMEM;

	for (i = 0; i < imp_table->num_settings; i++) {
		struct nvdisp_bandwidth_config *cfg;
		struct tegra_nvdisp_imp_settings *imp_settings;
		struct tegra_dc_ext_nvdisp_imp_global_entries *g_ents;

		cfg = &bw_cfg_table[i];
		imp_settings = &imp_table->settings[i];
		g_ents = &imp_settings->global_entries;

		cfg->iso_bw = g_ents->total_iso_bw_with_catchup_kBps;
		cfg->total_bw = g_ents->total_iso_bw_without_catchup_kBps;

		g_ents->min_hubclk_hz =
			tegra_nvdisp_bw_get_hubclk(g_ents->min_hubclk_hz);
		cfg->hubclk = g_ents->min_hubclk_hz;

		g_ents->emc_floor_hz =
			tegra_nvdisp_bw_get_emc(g_ents->emc_floor_hz);
		cfg->emc_la_floor = g_ents->emc_floor_hz;
	}

	/*
	 * Start with the highest-bw config and continue to fallback until we
	 * find one that works.
	 */
	for (i = 0; i < imp_table->num_settings; i++) {
		struct nvdisp_bandwidth_config *cfg = &bw_cfg_table[i];
		u32 cfg_dram_freq = 0;

		/*
		 * Check that our dedicated request doesn't exceed the total ISO
		 * bw. Both operands are in KB/s.
		 */
		if (cfg->iso_bw > total_iso_bw) {
			ret = -E2BIG;
			continue;
		}

		/* Make sure isomgr registration succeeds. */
		isomgr_handle = tegra_isomgr_register(iso_client,
				cfg->iso_bw,
				tegra_nvdisp_bandwidth_renegotiate,
				&ihub_bw_info);
		if (IS_ERR_OR_NULL(isomgr_handle)) {
			ret = -ENOENT;
			continue;
		}

		/*
		 * Check that the required EMC floor doesn't exceed the max EMC
		 * rate allowed.
		 */
		cfg_dram_freq = cfg->emc_la_floor * emc_to_dram_factor;
		if (tegra_bwmgr_round_rate(cfg_dram_freq) > max_emc_rate) {
			tegra_isomgr_unregister(isomgr_handle);
			ret = -E2BIG;
			continue;
		}

		ihub_bw_info.isomgr_handle = isomgr_handle;
		ihub_bw_info.max_config = *cfg;
		found_max_cfg = true;

		pr_info("%s: max config iso bw = %u KB/s\n",
			__func__, cfg->iso_bw);
		pr_info("%s: max config EMC floor = %u Hz\n",
			__func__, cfg->emc_la_floor);
		pr_info("%s: max config hubclk = %u Hz\n",
			__func__, cfg->hubclk);

		/* If we registered one of the platform settings, cache it. */
		imp_table->boot_setting = &imp_table->settings[i];
		break;
	}

	if (imp_table->boot_setting)
		ret = 0;
	else
		pr_err("%s: couldn't find valid max config!\n", __func__);

	kfree(bw_cfg_table);
	return ret;
}

/*
 * tegra_nvdisp_bandwidth_register - register the IHUB BW client
 *
 * @iso_client		ISO client id
 * @bwmgr_client	BWMGR client id
 * @udedi_bw		minimum dedicated ISO bw (KB/s) needed by client
 *
 * @retval		0 on success
 * @retval		-ENOENT on registration failures
 */
int tegra_nvdisp_bandwidth_register(enum tegra_iso_client iso_client,
				enum tegra_bwmgr_client_id bwmgr_client)
{
	int ret = 0;

	if (!tegra_platform_is_silicon())
		return ret;

	memset(&ihub_bw_info, 0, sizeof(ihub_bw_info));

	ihub_bw_info.bwmgr_handle = tegra_bwmgr_register(bwmgr_client);
	if (IS_ERR_OR_NULL(ihub_bw_info.bwmgr_handle)) {
		ret = -ENOENT;
		return ret;
	}

	ret = tegra_nvdisp_bandwidth_register_max_config(iso_client);
	if (ret)
		return ret;

	/*
	 * Assume that all the bandwidth is currently available to us so that we
	 * can reserve as much as needed until we are told by isomgr to backoff.
	 */
	ihub_bw_info.available_bw = UINT_MAX;

	return ret;
}

/*
 * tegra_nvdisp_bandwidth_unregister - unregister the IHUB BW client
 */
void _tegra_nvdisp_bandwidth_unregister(void)
{
	if (!tegra_platform_is_silicon())
		return;

	if (!IS_ERR_OR_NULL(ihub_bw_info.isomgr_handle))
		tegra_isomgr_unregister(ihub_bw_info.isomgr_handle);
	if (!IS_ERR_OR_NULL(ihub_bw_info.bwmgr_handle))
		tegra_bwmgr_unregister(ihub_bw_info.bwmgr_handle);
}

void tegra_nvdisp_bandwidth_unregister(void)
{
	if (!tegra_platform_is_silicon())
		return;

	mutex_lock(&tegra_nvdisp_lock);
	_tegra_nvdisp_bandwidth_unregister();
	mutex_unlock(&tegra_nvdisp_lock);
}
#else
int tegra_nvdisp_program_bandwidth(struct tegra_dc *dc, u32 new_iso_bw,
	u32 new_total_bw, u32 new_emc, u32 new_hubclk,
	bool before_win_update)
{
	return -ENOSYS;
}
int tegra_nvdisp_negotiate_reserved_bw(struct tegra_dc *dc, u32 new_iso_bw,
	u32 new_total_bw, u32 new_emc, u32 new_hubclk)
{
	return -ENOSYS;
}
void tegra_nvdisp_init_bandwidth(struct tegra_dc *dc) {}
void tegra_nvdisp_clear_bandwidth(struct tegra_dc *dc) {}
long tegra_nvdisp_calc_min_bandwidth(struct tegra_dc *dc)
{
	return -ENOSYS;
}
#endif /* CONFIG_TEGRA_ISOMGR */

