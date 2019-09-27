/*
 * dp_debug.c: dp debug interface.
 *
 * Copyright (c) 2015-2018 NVIDIA CORPORATION, All rights reserved.
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

#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/export.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/delay.h>
#include <linux/version.h>
#include "dc_reg.h"
#include "dc_priv.h"
#include "dp.h"
#include "dp_lt.h"
#include "sor_regs.h"
#include "sor.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <linux/uaccess.h>
#else
#include <asm/uaccess.h>
#endif

#ifdef CONFIG_DEBUG_FS

struct tegra_dp_test_settings default_dp_test_settings = {
	DRIVE_CURRENT_L0,
	PRE_EMPHASIS_L0,
	4,
	NV_DPCD_MAX_LINK_BANDWIDTH_VAL_1_62_GBPS,
	TEGRA_DC_DP_TRAINING_PATTERN_DISABLE,
	0,
	"rbr",
	"none",
	false,
	false,
};

static inline int parse_bitrate_setting(char *bitrate_name,
	struct tegra_dp_test_settings *test_settings)
{
	u8 bitrate;

	if (!strcmp(bitrate_name, "rbr"))
		bitrate = NV_DPCD_MAX_LINK_BANDWIDTH_VAL_1_62_GBPS;
	else if (!strcmp(bitrate_name, "hbr"))
		bitrate = NV_DPCD_MAX_LINK_BANDWIDTH_VAL_2_70_GBPS;
	else if (!strcmp(bitrate_name, "hbr2"))
		bitrate = NV_DPCD_MAX_LINK_BANDWIDTH_VAL_5_40_GBPS;
	else if (!strcmp(bitrate_name, "hbr3"))
		bitrate = NV_DPCD_MAX_LINK_BANDWIDTH_VAL_8_10_GBPS;
	else
		return -EINVAL;

	test_settings->bitrate = bitrate;
	test_settings->bitrate_name = bitrate_name;

	return 0;
}

static inline int parse_patt_setting(char *patt,
	struct tegra_dp_test_settings *test_settings)
{
	u8 tpg;

	if (!strcmp(patt, "none"))
		tpg = TEGRA_DC_DP_TRAINING_PATTERN_DISABLE;
	else if (!strcmp(patt, "t1"))
		tpg = TEGRA_DC_DP_TRAINING_PATTERN_1;
	else if (!strcmp(patt, "t2"))
		tpg = TEGRA_DC_DP_TRAINING_PATTERN_2;
	else if (!strcmp(patt, "t3"))
		tpg = TEGRA_DC_DP_TRAINING_PATTERN_3;
	else if (!strcmp(patt, "d102"))
		tpg = TEGRA_DC_DP_TRAINING_PATTERN_D102;
	else if (!strcmp(patt, "sblerrrate"))
		tpg = TEGRA_DC_DP_TRAINING_PATTERN_SBLERRRATE;
	else if (!strcmp(patt, "prbs7"))
		tpg = TEGRA_DC_DP_TRAINING_PATTERN_PRBS7;
	else if (!strcmp(patt, "pltpat") || !strcmp(patt, "pctpat"))
		tpg = TEGRA_DC_DP_TRAINING_PATTERN_CSTM;
	else if (!strcmp(patt, "hbr2compliance"))
		tpg = TEGRA_DC_DP_TRAINING_PATTERN_HBR2_COMPLIANCE;
	else if (!strcmp(patt, "cp2520_pat1"))
		tpg = TEGRA_DC_DP_TRAINING_PATTERN_CP2520_PAT1;
	else if (!strcmp(patt, "cp2520_pat3"))
		tpg = TEGRA_DC_DP_TRAINING_PATTERN_CP2520_PAT3;
	else if (!strcmp(patt, "t4"))
		tpg = TEGRA_DC_DP_TRAINING_PATTERN_4;
	else
		return -EINVAL;

	test_settings->tpg = tpg;
	test_settings->patt = patt;

	return 0;
}

static int parse_test_settings(const char __user *user_buf, size_t count,
	struct tegra_dp_test_settings *test_settings)
{
#define IN_RANGE(val, low, high) ((val) >= (low) && (val) <= (high))

	char *buf, *token, *settings[8];
	int i = 0, j = 0;
	u8 u8_val;

	buf = kzalloc(count + 1, GFP_KERNEL);
	if (!buf) {
		pr_err("dp_debug: Not enough memory for buffer\n");
		return -ENOMEM;
	}

	if (copy_from_user(buf, user_buf, count)) {
		pr_err("dp_debug: Copy from user failed\n");
		goto fail;
	}

	while ((token = strsep(&buf, ",\n")) && *token != '\0')
		settings[i++] = token;

	for (; j < i; j++) {
		char *name = strsep(&settings[j], "=");
		char *val = strsep(&settings[j], "=");

		if (!name || !val)
			goto parse_fail;

		if (!strcmp(name, "br")) {
			if (parse_bitrate_setting(val, test_settings))
				goto parse_fail;
			continue;
		}

		if (!strcmp(name, "patt")) {
			if (parse_patt_setting(val, test_settings))
				goto parse_fail;
			continue;
		}

		if (kstrtou8(val, 10, &u8_val))
			goto parse_fail;

		if (!strcmp(name, "sw") && IN_RANGE(u8_val,
					DRIVE_CURRENT_L0, DRIVE_CURRENT_L3))
			test_settings->drive_strength = u8_val;
		else if (!strcmp(name, "pre") && IN_RANGE(u8_val,
					PRE_EMPHASIS_L0, PRE_EMPHASIS_L3))
			test_settings->preemphasis = u8_val;
		else if (!strcmp(name, "lanes") && (u8_val == 1 || u8_val == 2
						|| u8_val == 4))
			test_settings->lanes = u8_val;
		else if (!strcmp(name, "ssc"))
			pr_info("dp_debug: SSC is fixed, ignoring request\n");
		else if (!strcmp(name, "tx_pu_disable"))
			test_settings->disable_tx_pu = u8_val;
		else if (!strcmp(name, "panel"))
			test_settings->panel_type = u8_val;
		else
			goto parse_fail;
	}

	kfree(buf);
	return 0;

parse_fail:
	pr_err("dp_debug: Invalid arg or value given\n");
fail:
	kfree(buf);
	return -EINVAL;
}

static int test_settings_show(struct seq_file *s, void *unused)
{
	struct tegra_dc_dp_data *dp = s->private;
	struct tegra_dp_test_settings *test_settings = &dp->test_settings;

	seq_printf(s, "\tDrive strength: %d\n", test_settings->drive_strength);
	seq_printf(s, "\tPreemphasis level: %d\n", test_settings->preemphasis);
	seq_printf(s, "\tLanes: %d\n", test_settings->lanes);
	seq_printf(s, "\tBitrate: %s\n", test_settings->bitrate_name);
	seq_printf(s, "\tTest pattern: %s\n", test_settings->patt);
	seq_printf(s, "\tSSC %sabled\n", test_settings->disable_ssc ?
							"dis" : "en");
	seq_printf(s, "\tTX_PU %sabled\n", test_settings->disable_tx_pu ?
							"dis" : "en");
	seq_printf(s, "\tPanel type : %s\n", test_settings->panel_type ?
						"Internal" : "External");

	return 0;
}

static ssize_t test_settings_set(struct file *file, const char __user *buf,
					size_t count, loff_t *off)
{
	struct seq_file *s = file->private_data;
	struct tegra_dc_dp_data *dp = s->private;
	struct tegra_dc *dc = dp->dc;
	struct tegra_dc_sor_data *sor = dp->sor;
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;
	struct tegra_dp_test_settings *test_settings = &dp->test_settings;
	int dc_out_type = dc->out->type;
	u32 vs_reg, pe_reg;
	u32 max_tx_pu;

	if (parse_test_settings(buf, count, test_settings))
		return -EINVAL;

	/* Switch DC out type as TEGRA_DC_OUT_FAKE_DP so that SOR will be
	 * enabled as if it was interacting with a fake panel.
	 */
	dc->out->type = TEGRA_DC_OUT_FAKE_DP;
	if (!dc->enabled) {
		/* Set default mode to VGA. tegra_dc_enable needs some mode
		 * to be set.
		 */
		tegra_dc_set_default_videomode(dc);
		tegra_dc_enable(dc);
	}

	/* detach SOR and precharge lanes */
	tegra_dc_sor_detach(sor);
	tegra_sor_precharge_lanes(sor);

	tegra_dc_io_start(dc);

	/* Set panel type */
	if (test_settings->panel_type)
		tegra_sor_write_field(sor, NV_SOR_DP_SPARE(sor->portnum),
					NV_SOR_DP_SPARE_PANEL_MASK,
					NV_SOR_DP_SPARE_PANEL_INTERNAL);
	else
		tegra_sor_write_field(sor, NV_SOR_DP_SPARE(sor->portnum),
					NV_SOR_DP_SPARE_PANEL_MASK,
					NV_SOR_DP_SPARE_PANEL_EXTERNAL);

	/* set lane count and bitrate */
	cfg->lane_count = test_settings->lanes;
	cfg->link_bw = test_settings->bitrate;
	tegra_dp_update_link_config(dp);

	/* set drive strength, preemphasis, and postcursor */
	vs_reg = dp->pdata->lt_data[DP_VS].data[POST_CURSOR2_L0]
		[test_settings->drive_strength][test_settings->preemphasis];
	pe_reg = dp->pdata->lt_data[DP_PE].data[POST_CURSOR2_L0]
		[test_settings->drive_strength][test_settings->preemphasis];
	tegra_sor_writel(sor, NV_SOR_DC(sor->portnum), vs_reg | (vs_reg << 8) |
				(vs_reg << 16) | (vs_reg << 24));
	tegra_sor_writel(sor, NV_SOR_PR(sor->portnum), pe_reg | (pe_reg << 8) |
				(pe_reg << 16) | (pe_reg << 24));
	tegra_sor_writel(sor, NV_SOR_POSTCURSOR(sor->portnum), 0);

	/* set TX_PU */
	if (test_settings->disable_tx_pu) {
		tegra_sor_write_field(sor,
				nv_sor_dp_padctl(sor->portnum),
				NV_SOR_DP_PADCTL_TX_PU_ENABLE |
				NV_SOR_DP_PADCTL_TX_PU_VALUE_DEFAULT_MASK,
				NV_SOR_DP_PADCTL_TX_PU_DISABLE);
	} else {
		max_tx_pu = dp->pdata->lt_data[DP_TX_PU].data[POST_CURSOR2_L0]
		[test_settings->drive_strength][test_settings->preemphasis];
		tegra_sor_write_field(sor, nv_sor_dp_padctl(sor->portnum),
				NV_SOR_DP_PADCTL_TX_PU_VALUE_DEFAULT_MASK,
				(max_tx_pu <<
				NV_SOR_DP_PADCTL_TX_PU_VALUE_SHIFT |
				NV_SOR_DP_PADCTL_TX_PU_ENABLE));
	}

	/* send training pattern */
	if (test_settings->tpg == TEGRA_DC_DP_TRAINING_PATTERN_CSTM) {
		if (!strcmp(test_settings->patt, "pltpat")) {
			tegra_sor_writel(sor, NV_SOR_DP_LQ_CSTM_0, 0x3e0f83e0);
			tegra_sor_writel(sor, NV_SOR_DP_LQ_CSTM_1, 0x0f83e0f8);
			tegra_sor_writel(sor, NV_SOR_DP_LQ_CSTM_2, 0x0000f83e);
		} else {
			tegra_sor_writel(sor, NV_SOR_DP_LQ_CSTM_0, 0xfe0f83e0);
			tegra_sor_writel(sor, NV_SOR_DP_LQ_CSTM_1, 0xaccccccc);
			tegra_sor_writel(sor, NV_SOR_DP_LQ_CSTM_2, 0x0000aa8a);
		}
	} else {
		tegra_sor_writel(sor, NV_SOR_DP_LQ_CSTM_0, 0);
		tegra_sor_writel(sor, NV_SOR_DP_LQ_CSTM_1, 0);
		tegra_sor_writel(sor, NV_SOR_DP_LQ_CSTM_2, 0);
	}

	if (test_settings->tpg == TEGRA_DC_DP_TRAINING_PATTERN_HBR2_COMPLIANCE)
		tegra_sor_writel(sor, NV_SOR_DP_TPG_CONFIG, 0x00FC);
	else
		tegra_sor_writel(sor, NV_SOR_DP_TPG_CONFIG, 0);

	if (test_settings->tpg == TEGRA_DC_DP_TRAINING_PATTERN_CP2520_PAT1 ||
		test_settings->tpg == TEGRA_DC_DP_TRAINING_PATTERN_CP2520_PAT3 ||
		test_settings->tpg == TEGRA_DC_DP_TRAINING_PATTERN_4)
		tegra_dp_set_enhanced_framing(dp, false);
	else
		tegra_dp_set_enhanced_framing(dp, cfg->enhanced_framing);

	tegra_sor_tpg(sor, test_settings->tpg, test_settings->lanes);

	/* restore DC out type */
	dc->out->type = dc_out_type;

	tegra_dc_io_end(dc);

	return count;
}

static int test_settings_open(struct inode *inode, struct file *file)
{
	return single_open(file, test_settings_show, inode->i_private);
}

const struct file_operations test_settings_fops = {
	.open		= test_settings_open,
	.read		= seq_read,
	.write		= test_settings_set,
	.llseek		= seq_lseek,
	.release	= single_release,
};
EXPORT_SYMBOL(test_settings_fops);

#endif
