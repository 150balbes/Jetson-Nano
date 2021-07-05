/*
 * hda_dc.c: tegra dc hda dc driver.
 *
 * Copyright (c) 2015-2020, NVIDIA CORPORATION, All rights reserved.
 * Author: Animesh Kishore <ankishore@nvidia.com>
 * Author: Rahul Mittal <rmittal@nvidia.com>
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

#include <linux/kernel.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif

#include <video/tegra_hdmi_audio.h>

#include "dc.h"
#include "dc_priv.h"
#include "sor.h"
#include "sor_regs.h"
#include "edid.h"
#include "hdmi2.0.h"
#include "dp.h"
#include "hda_dc.h"

static struct tegra_hda_inst *hda_inst;
static DEFINE_MUTEX(global_hda_lock);

#define to_hdmi(DATA)	((struct tegra_hdmi *)DATA)
#define to_dp(DATA)	((struct tegra_dc_dp_data *)DATA)

int tegra_hda_get_dev_id(struct tegra_dc_sor_data *sor)
{
	int dev_id;

	if (!sor)
		return -ENODEV;

	tegra_unpowergate_partition(sor->powergate_id);
	tegra_sor_safe_clk_enable(sor);
	if (!sor->dc->initialized)
		tegra_sor_clk_enable(sor);
	tegra_dc_io_start(sor->dc);
	dev_id = tegra_sor_readl_ext(sor, NV_SOR_AUDIO_GEN_CTRL);
	dev_id = (dev_id >> NV_SOR_AUDIO_GEN_CTRL_DEV_ID_SHIFT) &
			NV_SOR_AUDIO_GEN_CTRL_DEV_ID_MASK;
	tegra_dc_io_end(sor->dc);
	if (!sor->dc->initialized)
		tegra_sor_clk_disable(sor);
	tegra_sor_safe_clk_disable(sor);
	tegra_powergate_partition(sor->powergate_id);
	return dev_id;
}
EXPORT_SYMBOL(tegra_hda_get_dev_id);

static int tegra_hda_get_sor_num(int dev_id)
{
	int i;
	int val = -EINVAL;

	if (!hda_inst)
		goto err;

	for (i = 0; i < tegra_dc_get_numof_dispsors(); i++) {
		struct tegra_dc_hda_data *hda;

		hda = hda_inst[i].hda;
		if (!hda)
			continue;

		if ((dev_id == hda->dev_id) && hda->sor) {
			val = hda->sor->ctrl_num;
			if (val >= tegra_dc_get_numof_dispsors())
				val = -EINVAL;
			break;
		}
	}
err:
	return val;
}

static void tegra_hda_get_eld_header(u8 *eld_mem_block,
					struct tegra_dc_hda_data *hda)
{
	struct tegra_edid_hdmi_eld *eld = hda->eld;

	eld->baseline_len = HDMI_ELD_MONITOR_NAME_STR + eld->mnl +
				eld->sad_count * 3 - HDMI_ELD_CEA_EDID_VER_MNL;

	eld_mem_block[HDMI_ELD_VER] = eld->eld_ver << 3;
	eld_mem_block[HDMI_ELD_BASELINE_ELD_LEN] =
			DIV_ROUND_UP(eld->baseline_len, 4);
}

static void tegra_hda_get_eld_baseline(u8 *eld_mem_block,
					struct tegra_dc_hda_data *hda)
{
	struct tegra_edid_hdmi_eld *eld = hda->eld;
	u8 tmp;

	tmp = eld->mnl | (eld->cea_edid_ver << 5);
	eld_mem_block[HDMI_ELD_CEA_EDID_VER_MNL] = tmp;

	tmp = eld->support_hdcp | (eld->support_ai << 1) |
		(eld->conn_type << 2) | (eld->sad_count << 4);
	eld_mem_block[HDMI_ELD_SAD_CNT_CON_TYPE_S_AI_S_HDCP] = tmp;

	eld_mem_block[HDMI_ELD_AUDIO_SYNC_DELAY] = eld->aud_synch_delay;

	eld_mem_block[HDMI_ELD_RLRC_FLRC_RC_RLR_FC_LFE_FLR] = eld->spk_alloc;

	memcpy(&eld_mem_block[HDMI_ELD_PORT_ID], eld->port_id, 8);

	memcpy(&eld_mem_block[HDMI_ELD_MANUFACTURER_NAME],
					eld->manufacture_id, 2);

	memcpy(&eld_mem_block[HDMI_ELD_PRODUCT_CODE], eld->product_id, 2);

	memcpy(&eld_mem_block[HDMI_ELD_MONITOR_NAME_STR],
				eld->monitor_name, eld->mnl);

	memcpy(&eld_mem_block[HDMI_ELD_MONITOR_NAME_STR + eld->mnl],
						eld->sad, eld->sad_count * 3);
}

static void tegra_hda_get_eld_vendor(u8 *eld_mem_block,
					struct tegra_dc_hda_data *hda)
{
	struct tegra_edid_hdmi_eld *eld = hda->eld;
	u32 vendor_block_index = 4 + eld->baseline_len; /* 4 byte header */

	if (!eld->baseline_len)
		dev_err(&hda->dc->ndev->dev,
			"hdm: eld baseline length not populated\n");

	memset(&eld_mem_block[vendor_block_index], 0,
		HDMI_ELD_BUF - vendor_block_index + 1);
}

static int tegra_hda_eld_config(struct tegra_dc_hda_data *hda)
{
	u8 *eld_mem;
	int cnt;

	eld_mem = devm_kzalloc(&hda->dc->ndev->dev,
				HDMI_ELD_BUF, GFP_KERNEL);
	if (!eld_mem) {
		dev_warn(&hda->dc->ndev->dev,
			"hdmi: eld memory allocation failed\n");
		return -ENOMEM;
	}

	tegra_hda_get_eld_header(eld_mem, hda);
	tegra_hda_get_eld_baseline(eld_mem, hda);
	tegra_hda_get_eld_vendor(eld_mem, hda);

	for (cnt = 0; cnt < HDMI_ELD_BUF; cnt++)
		tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_HDA_ELD_BUFWR,
				NV_SOR_AUDIO_HDA_ELD_BUFWR_INDEX(cnt) |
				NV_SOR_AUDIO_HDA_ELD_BUFWR_DATA(eld_mem[cnt]));

	devm_kfree(&hda->dc->ndev->dev, eld_mem);
	return 0;
}

int tegra_hda_get_switch_name(int dev_id, char *name)
{
	int sor_num;
	struct tegra_dc_hda_data *hda;

	mutex_lock(&global_hda_lock);
	sor_num = tegra_hda_get_sor_num(dev_id);
	if (sor_num < 0) {
		mutex_unlock(&global_hda_lock);
		return -EINVAL;
	}
	mutex_lock(&hda_inst[sor_num].hda_inst_lock);
	mutex_unlock(&global_hda_lock);
	hda = hda_inst[sor_num].hda;
	if (hda) {
		snprintf(name, CHAR_BUF_SIZE_MAX, "%s",
			hda->audio_switch_name);
		mutex_unlock(&hda_inst[sor_num].hda_inst_lock);
		return 0;
	}
	mutex_unlock(&hda_inst[sor_num].hda_inst_lock);
	return -EINVAL;
}
EXPORT_SYMBOL(tegra_hda_get_switch_name);

/* Applicable for dp too, func name still uses hdmi as per generic hda driver */
int tegra_hdmi_setup_hda_presence(int dev_id)
{
	struct tegra_dc_hda_data *hda;
	int sor_num, val = -EINVAL;

	mutex_lock(&global_hda_lock);
	sor_num = tegra_hda_get_sor_num(dev_id);
	if (sor_num < 0) {
		mutex_unlock(&global_hda_lock);
		return sor_num;
	}

	mutex_lock(&hda_inst[sor_num].hda_inst_lock);
	mutex_unlock(&global_hda_lock);

	if (hda_inst[sor_num].hda_state != HDA_ENABLED)
		goto err;

	hda = hda_inst[sor_num].hda;
	if ((hda->sink == TEGRA_DC_OUT_HDMI) &&
			to_hdmi(hda->client_data)->dvi)
		goto err;

	if (*(hda->enabled) && *(hda->eld_valid)) {
		tegra_unpowergate_partition(hda->sor->powergate_id);
		tegra_sor_safe_clk_enable(hda->sor);
		tegra_sor_clk_enable(hda->sor);
		tegra_dc_io_start(hda->dc);

		/* remove hda presence while setting up eld */
		tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_HDA_PRESENCE, 0);

		tegra_hda_eld_config(hda);
		tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_HDA_PRESENCE,
				NV_SOR_AUDIO_HDA_PRESENCE_ELDV(1) |
				NV_SOR_AUDIO_HDA_PRESENCE_PD(1));

		tegra_dc_io_end(hda->dc);
		tegra_sor_clk_disable(hda->sor);
		tegra_sor_safe_clk_disable(hda->sor);
		tegra_powergate_partition(hda->sor->powergate_id);
		val = 0;
	}

err:
	mutex_unlock(&hda_inst[sor_num].hda_inst_lock);
	return val;
}
EXPORT_SYMBOL(tegra_hdmi_setup_hda_presence);

static void tegra_hdmi_audio_infoframe(struct tegra_dc_hda_data *hda)
{
	if ((hda->sink == TEGRA_DC_OUT_HDMI) &&
					to_hdmi(hda->client_data)->dvi)
		return;

	/* disable audio infoframe before configuring */
	tegra_sor_writel_ext(hda->sor, NV_SOR_HDMI_AUDIO_INFOFRAME_CTRL, 0);

	if (hda->sink == TEGRA_DC_OUT_HDMI) {
		to_hdmi(hda->client_data)->audio.channel_cnt =
			HDMI_AUDIO_CHANNEL_CNT_2;
		tegra_hdmi_infoframe_pkt_write(to_hdmi(hda->client_data),
				NV_SOR_HDMI_AUDIO_INFOFRAME_HEADER,
				HDMI_INFOFRAME_TYPE_AUDIO,
				HDMI_INFOFRAME_VS_AUDIO,
				HDMI_INFOFRAME_LEN_AUDIO,
				&to_hdmi(hda->client_data)->audio,
				sizeof(to_hdmi(hda->client_data)->audio),
				false);
	}

	/* Send infoframe every frame, checksum hw generated */
	tegra_sor_writel_ext(hda->sor, NV_SOR_HDMI_AUDIO_INFOFRAME_CTRL,
		NV_SOR_HDMI_AUDIO_INFOFRAME_CTRL_ENABLE_YES |
		NV_SOR_HDMI_AUDIO_INFOFRAME_CTRL_OTHER_DISABLE |
		NV_SOR_HDMI_AUDIO_INFOFRAME_CTRL_SINGLE_DISABLE |
		NV_SOR_HDMI_AUDIO_INFOFRAME_CTRL_CHECKSUM_ENABLE);
}

/* HW generated CTS and N */
static void tegra_hdmi_audio_acr(u32 audio_freq, struct tegra_dc_hda_data *hda)
{
#define GET_AVAL(n, fs_hz) ((24000 * n) / (128 * fs_hz / 1000))
	u32 val;

	tegra_sor_writel_ext(hda->sor, NV_SOR_HDMI_ACR_CTRL, 0x0);

	val = NV_SOR_HDMI_SPARE_HW_CTS_ENABLE |
		NV_SOR_HDMI_SPARE_CTS_RESET_VAL(1) |
		NV_SOR_HDMI_SPARE_ACR_PRIORITY_HIGH;
	tegra_sor_writel_ext(hda->sor, NV_SOR_HDMI_SPARE, val);

	tegra_sor_writel_ext(hda->sor, NV_SOR_HDMI_ACR_0441_SUBPACK_LOW,
			NV_SOR_HDMI_ACR_SUBPACK_USE_HW_CTS);
	tegra_sor_writel_ext(hda->sor, NV_SOR_HDMI_ACR_0441_SUBPACK_HIGH,
			NV_SOR_HDMI_ACR_SUBPACK_ENABLE);

	val = NV_SOR_HDMI_AUDIO_N_RESET_ASSERT |
		NV_SOR_HDMI_AUDIO_N_LOOKUP_ENABLE;
	tegra_sor_writel_ext(hda->sor, NV_SOR_HDMI_AUDIO_N, val);

	/* N from table 7.1, 7.2, 7.3 hdmi spec v1.4 */
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_NVAL_0320, 4096);
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_AVAL_0320,
			GET_AVAL(4096, audio_freq));
	/*For Multiple of 44.1khz source some receiver cannot handle CTS value
	  which is a little far away with golden value,So for the
	  TMDS_clk=148.5Mhz case, we should keep AVAL as default value(20000),
	  and set N= 4704*2 and 4704*4 for the 88.2 and 176.4khz audio case */
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_NVAL_0441, 4704);
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_AVAL_0441, 20000);
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_NVAL_0882, 9408);
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_AVAL_0882, 20000);
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_NVAL_1764, 18816);
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_AVAL_1764, 20000);
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_NVAL_0480, 6144);
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_AVAL_0480,
			GET_AVAL(6144, audio_freq));
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_NVAL_0960, 12288);
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_AVAL_0960,
			GET_AVAL(12288, audio_freq));
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_NVAL_1920, 24576);
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_AVAL_1920,
			GET_AVAL(24576, audio_freq));

	tegra_sor_write_field_ext(hda->sor, NV_SOR_HDMI_AUDIO_N,
				NV_SOR_HDMI_AUDIO_N_RESET_ASSERT,
				NV_SOR_HDMI_AUDIO_N_RESET_DEASSERT);
#undef GET_AVAL
}

static void tegra_hda_audio_config(u32 audio_freq, u32 audio_src,
					struct tegra_dc_hda_data *hda)
{
	u32 val;
	struct tegra_dc_dp_link_config *cfg = NULL;

	if (hda->sink == TEGRA_DC_OUT_DP)
		cfg = &to_dp(hda->client_data)->link_cfg;

	if ((hda->sink == TEGRA_DC_OUT_HDMI) && to_hdmi(hda->client_data)->dvi)
		return;

	/* hda is the only audio source */
	val = NV_SOR_AUDIO_CTRL_AFIFO_FLUSH |
		NV_SOR_AUDIO_CTRL_SRC_HDA;
	if (hda->null_sample_inject)
		val |= NV_SOR_AUDIO_CTRL_NULL_SAMPLE_EN;
	tegra_sor_writel_ext(hda->sor, NV_SOR_AUDIO_CTRL, val);

	/* override to advertise HBR capability */
	tegra_sor_writel_ext(hda->sor, NV_PDISP_SOR_AUDIO_SPARE0_0,
		(1 << HDMI_AUDIO_HBR_ENABLE_SHIFT) |
		tegra_sor_readl_ext(hda->sor, NV_PDISP_SOR_AUDIO_SPARE0_0));

	if (hda->sink == TEGRA_DC_OUT_DP) {
		/* program h/vblank sym */
		tegra_sor_write_field_ext(hda->sor,
			NV_SOR_DP_AUDIO_HBLANK_SYMBOLS,
			NV_SOR_DP_AUDIO_HBLANK_SYMBOLS_MASK, cfg->hblank_sym);

		tegra_sor_write_field_ext(hda->sor,
			NV_SOR_DP_AUDIO_VBLANK_SYMBOLS,
			NV_SOR_DP_AUDIO_VBLANK_SYMBOLS_MASK, cfg->vblank_sym);

		val = NV_SOR_DP_AUDIO_CTRL_ENABLE |
			NV_SOR_DP_AUDIO_CTRL_NEW_SETTINGS_TRIGGER |
			NV_SOR_DP_AUDIO_CTRL_CA_SELECT_HW |
			NV_SOR_DP_AUDIO_CTRL_SS_SELECT_HW |
			NV_SOR_DP_AUDIO_CTRL_SF_SELECT_HW |
			NV_SOR_DP_AUDIO_CTRL_CC_SELECT_HW |
			NV_SOR_DP_AUDIO_CTRL_CT_SELECT_HW;
		tegra_sor_writel_ext(hda->sor, NV_SOR_DP_AUDIO_CTRL, val);

		/* make sure to disable overriding channel data */
		tegra_sor_write_field_ext(hda->sor,
			NV_SOR_DP_OUTPUT_CHANNEL_STATUS2,
			NV_SOR_DP_OUTPUT_CHANNEL_STATUS2_OVERRIDE_EN,
			NV_SOR_DP_OUTPUT_CHANNEL_STATUS2_OVERRIDE_DIS);
	}

	if (hda->sink == TEGRA_DC_OUT_HDMI) {
		tegra_hdmi_audio_acr(audio_freq, hda);
		tegra_hdmi_audio_infoframe(hda);
	}
}

/* Applicable for dp too, func name still uses hdmi as per generic hda driver */
int tegra_hdmi_setup_audio_freq_source(unsigned audio_freq,
					unsigned audio_source,
					int dev_id)
{
	bool valid_freq;
	struct tegra_dc_hda_data *hda;
	int sor_num, val = -EINVAL;

	mutex_lock(&global_hda_lock);
	sor_num = tegra_hda_get_sor_num(dev_id);
	if (sor_num < 0) {
		mutex_unlock(&global_hda_lock);
		return sor_num;
	}

	mutex_lock(&hda_inst[sor_num].hda_inst_lock);
	mutex_unlock(&global_hda_lock);

	hda = hda_inst[sor_num].hda;
	valid_freq = AUDIO_FREQ_32K == audio_freq ||
			AUDIO_FREQ_44_1K == audio_freq ||
			AUDIO_FREQ_48K == audio_freq ||
			AUDIO_FREQ_88_2K == audio_freq ||
			AUDIO_FREQ_96K == audio_freq ||
			AUDIO_FREQ_176_4K == audio_freq ||
			AUDIO_FREQ_192K == audio_freq;

	if (hda_inst[sor_num].hda_state != HDA_ENABLED)
		goto err;

	if ((hda->sink == TEGRA_DC_OUT_HDMI) && to_hdmi(hda->client_data)->dvi)
		goto err;

	if (valid_freq) {
		tegra_unpowergate_partition(hda->sor->powergate_id);
		tegra_sor_clk_enable(hda->sor);
		tegra_dc_io_start(hda->dc);

		tegra_hda_audio_config(audio_freq, audio_source, hda);

		tegra_dc_io_end(hda->dc);
		tegra_sor_clk_disable(hda->sor);
		tegra_powergate_partition(hda->sor->powergate_id);
		val = 0;
	}
err:
	if (valid_freq)
		hda->audio_freq = audio_freq;
	mutex_unlock(&hda_inst[sor_num].hda_inst_lock);
	return val;
}
EXPORT_SYMBOL(tegra_hdmi_setup_audio_freq_source);

/* Applicable for dp too, func name still uses hdmi as per generic hda driver */
int tegra_hdmi_audio_null_sample_inject(bool on, int dev_id)
{
	struct tegra_dc_hda_data *hda;
	int sor_num;
	bool null_sample_flag;

	mutex_lock(&global_hda_lock);
	sor_num = tegra_hda_get_sor_num(dev_id);
	if (sor_num < 0) {
		mutex_unlock(&global_hda_lock);
		return sor_num;
	}

	mutex_lock(&hda_inst[sor_num].hda_inst_lock);
	mutex_unlock(&global_hda_lock);

	hda = hda_inst[sor_num].hda;
	null_sample_flag = hda->null_sample_inject;
	if (hda_inst[sor_num].hda_state != HDA_ENABLED)
		goto err;

	tegra_unpowergate_partition(hda->sor->powergate_id);
	tegra_sor_clk_enable(hda->sor);
	tegra_dc_io_start(hda->dc);

	if (on && !hda->null_sample_inject)
		tegra_sor_write_field_ext(hda->sor,
					NV_SOR_AUDIO_CTRL,
					NV_SOR_AUDIO_CTRL_NULL_SAMPLE_EN,
					NV_SOR_AUDIO_CTRL_NULL_SAMPLE_EN);
	else if (!on && hda->null_sample_inject)
		tegra_sor_write_field_ext(hda->sor,
					NV_SOR_AUDIO_CTRL,
					NV_SOR_AUDIO_CTRL_NULL_SAMPLE_EN,
					NV_SOR_AUDIO_CTRL_NULL_SAMPLE_DIS);

	null_sample_flag = on;
	tegra_dc_io_end(hda->dc);
	tegra_sor_clk_disable(hda->sor);
	tegra_powergate_partition(hda->sor->powergate_id);
err:
	hda->null_sample_inject = null_sample_flag;
	mutex_unlock(&hda_inst[sor_num].hda_inst_lock);
	return 0;
}
EXPORT_SYMBOL(tegra_hdmi_audio_null_sample_inject);

static void tegra_dc_hda_put_clocks(struct tegra_dc_hda_data *hda)
{
	if (!IS_ERR_OR_NULL(hda->hda_clk))
		clk_put(hda->hda_clk);
	if (!IS_ERR_OR_NULL(hda->hda2codec_clk))
		clk_put(hda->hda2codec_clk);
	if (!IS_ERR_OR_NULL(hda->hda2hdmi_clk))
		clk_put(hda->hda2hdmi_clk);
	if (hda->sink == TEGRA_DC_OUT_DP) {
		if (!IS_ERR_OR_NULL(hda->pll_p_clk))
			clk_put(hda->pll_p_clk);
		if (!IS_ERR_OR_NULL(hda->maud_clk))
			clk_put(hda->maud_clk);
	}
}

static void tegra_dc_hda_get_clocks(struct tegra_dc *dc,
					struct tegra_dc_hda_data *hda)
{
	struct device_node *np_sor = tegra_dc_get_conn_np(dc);

	if (!np_sor) {
		dev_err(&dc->ndev->dev, "%s: error getting connector np\n",
			__func__);
		return;
	}

	hda->hda_clk = tegra_disp_of_clk_get_by_name(np_sor, "hda");
	if (IS_ERR_OR_NULL(hda->hda_clk)) {
		dev_err(&dc->ndev->dev, "hda: can't get hda clock\n");
		goto err_get_clk;
	}
	hda->hda2codec_clk = tegra_disp_of_clk_get_by_name(np_sor,
							"hda2codec_2x");
	if (IS_ERR_OR_NULL(hda->hda2codec_clk)) {
		dev_err(&dc->ndev->dev,
			"hda: can't get hda2codec clock\n");
		goto err_get_clk;
	}
	hda->hda2hdmi_clk = tegra_disp_of_clk_get_by_name(np_sor, "hda2hdmi");
	if (IS_ERR_OR_NULL(hda->hda2hdmi_clk)) {
		dev_err(&dc->ndev->dev, "hda: can't get hda2hdmi clock\n");
		goto err_get_clk;
	}

	if (hda->sink == TEGRA_DC_OUT_DP) {
		if (tegra_dc_is_nvdisplay()) {
			hda->pll_p_clk = tegra_disp_of_clk_get_by_name(np_sor,
					"pllp_out0");
			if (IS_ERR_OR_NULL(hda->pll_p_clk)) {
				dev_err(&dc->ndev->dev,
						"hda: can't get pllp_out0 clock\n");
				goto err_get_clk;
			}
		} else {
			hda->pll_p_clk = tegra_disp_of_clk_get_by_name(np_sor,
					"pll_p");
			if (IS_ERR_OR_NULL(hda->pll_p_clk)) {
				dev_err(&dc->ndev->dev,
						"hda: can't get pll_p clock\n");
				goto err_get_clk;
			}
		}
		hda->maud_clk = tegra_disp_of_clk_get_by_name(np_sor, "maud");
		if (IS_ERR_OR_NULL(hda->maud_clk)) {
			dev_err(&hda->dc->ndev->dev,
				"hda: can't get maud clock\n");
			goto err_get_clk;
		}
		clk_set_parent(hda->maud_clk, hda->pll_p_clk);
	}
	return;

err_get_clk:
	tegra_dc_hda_put_clocks(hda);
	return;
}

static void tegra_dc_hda_enable_clocks(struct tegra_dc_hda_data *hda)
{
	clk_prepare_enable(hda->hda_clk);
	clk_prepare_enable(hda->hda2codec_clk);
	clk_prepare_enable(hda->hda2hdmi_clk);

	if (hda->sink == TEGRA_DC_OUT_DP) {
		clk_set_rate(hda->maud_clk, 102000000);
		clk_prepare_enable(hda->maud_clk);
	}
}

static void tegra_dc_hda_disable_clocks(struct tegra_dc_hda_data *hda)
{
	if (hda->sink == TEGRA_DC_OUT_DP)
		clk_disable_unprepare(hda->maud_clk);

	clk_disable_unprepare(hda->hda2hdmi_clk);
	clk_disable_unprepare(hda->hda2codec_clk);
	clk_disable_unprepare(hda->hda_clk);
}

void tegra_hda_enable(void *hda_handle)
{
	unsigned int sor_num;
	struct tegra_dc_hda_data *hda = hda_handle;

	if (!hda_inst || !hda)
		return;

	sor_num = hda->sor->ctrl_num;
	if (sor_num >= tegra_dc_get_numof_dispsors())
		return;

	tegra_dc_hda_enable_clocks(hda);

	if (hda->sink == TEGRA_DC_OUT_DP)
		hda->eld->conn_type = 1; /* For DP, conn_type = 1 */

	mutex_lock(&hda_inst[sor_num].hda_inst_lock);
	hda_inst[sor_num].hda_state = HDA_ENABLED;
	mutex_unlock(&hda_inst[sor_num].hda_inst_lock);

	tegra_hdmi_setup_hda_presence(hda->dev_id);
	tegra_hdmi_audio_null_sample_inject(hda->null_sample_inject, hda->dev_id);
	tegra_hdmi_setup_audio_freq_source(hda->audio_freq, HDA, hda->dev_id);
}

void tegra_hda_disable(void *hda_handle)
{
	int sor_num;
	struct tegra_dc_hda_data *hda = (struct tegra_dc_hda_data *)hda_handle;

	if (!hda || !hda_inst)
		return;

	tegra_dc_hda_disable_clocks(hda);

	mutex_lock(&global_hda_lock);
	sor_num = tegra_hda_get_sor_num(hda->dev_id);
	if (sor_num < 0) {
		mutex_unlock(&global_hda_lock);
		return;
	}

	mutex_unlock(&global_hda_lock);
	mutex_lock(&hda_inst[sor_num].hda_inst_lock);
	hda_inst[sor_num].hda_state &= HDA_INITIALIZED;
	mutex_unlock(&hda_inst[sor_num].hda_inst_lock);
}

void tegra_hda_init(struct tegra_dc *dc, void *data)
{
	int i, size;
	struct tegra_dc_hda_data *hda = NULL;

	mutex_lock(&global_hda_lock);
	if (!hda_inst) {
		size = tegra_dc_get_numof_dispsors() * sizeof(*hda_inst);

		hda_inst = kzalloc(size, GFP_KERNEL);
		if (!hda_inst) {
			mutex_unlock(&global_hda_lock);
			goto err;
		}

		for (i = 0; i < tegra_dc_get_numof_dispsors(); i++)
			mutex_init(&hda_inst[i].hda_inst_lock);
	}
	mutex_unlock(&global_hda_lock);

	if ((dc->out->type == TEGRA_DC_OUT_HDMI) ||
			(dc->out->type == TEGRA_DC_OUT_DP)) {
		hda = kzalloc(sizeof(*hda), GFP_KERNEL);
		if (!hda)
			goto err;

		hda->audio_switch_name = kzalloc(sizeof(char) *
			CHAR_BUF_SIZE_MAX, GFP_KERNEL);
		if (!hda->audio_switch_name)
			goto err;

		hda->sink = dc->out->type;
		hda->client_data = data;
		hda->sink = dc->out->type;
		hda->null_sample_inject = false;
		tegra_dc_hda_get_clocks(dc, hda);

		if (dc->out->type == TEGRA_DC_OUT_HDMI) {
			struct tegra_hdmi *hdmi = data;

			hdmi->hda_handle = hda;
			hda->sor = to_hdmi(hda->client_data)->sor;
			hda->dc = to_hdmi(hda->client_data)->dc;
			hda->eld = &to_hdmi(hda->client_data)->eld;
			hda->enabled = &to_hdmi(hda->client_data)->enabled;
			hda->eld_valid = &to_hdmi(hda->client_data)->eld_valid;
			snprintf(hda->audio_switch_name, CHAR_BUF_SIZE_MAX,
				"aux%d_audio", hdmi->sor->ctrl_num);
		} else if (dc->out->type == TEGRA_DC_OUT_DP) {
			struct tegra_dc_dp_data *dp = data;

			dp->hda_handle = hda;
			hda->sor = to_dp(hda->client_data)->sor;
			hda->dc = to_dp(hda->client_data)->dc;
			hda->eld = &to_dp(hda->client_data)->hpd_data.eld;
			hda->enabled = &to_dp(hda->client_data)->enabled;
			hda->eld_valid =
			&to_dp(hda->client_data)->hpd_data.eld_retrieved;
			snprintf(hda->audio_switch_name, CHAR_BUF_SIZE_MAX,
				"aux%d_audio", dp->sor->ctrl_num);
		}

		if (hda->sor) {
			int sor_num = hda->sor->ctrl_num;

			mutex_lock(&global_hda_lock);
			if ((sor_num >= 0) &&
				(sor_num < tegra_dc_get_numof_dispsors()) &&
				hda_inst) {
				mutex_lock(&hda_inst[sor_num].hda_inst_lock);
				hda->dev_id = tegra_hda_get_dev_id(hda->sor);
				hda->sor->dev_id = hda->dev_id;
				hda_inst[sor_num].hda = hda;
				hda_inst[sor_num].hda_state = HDA_INITIALIZED;
				mutex_unlock(&hda_inst[sor_num].hda_inst_lock);
			}
			mutex_unlock(&global_hda_lock);
		}
	}
	return;
err:
	if (hda) {
		if (hda->audio_switch_name)
			/* this should not happen, but left for future */
			kfree(hda->audio_switch_name);
		kfree(hda);
	}
	dev_err(&dc->ndev->dev,
		"Failed to allocate hda handle memory");
}

void tegra_hda_destroy(void *hda_handle)
{
	int i, sor_num;
	bool free_hda_mem = true;
	struct tegra_dc_hda_data *hda = (struct tegra_dc_hda_data *)hda_handle;

	mutex_lock(&global_hda_lock);
	if (!hda_inst)
		goto err;

	if (hda) {
		sor_num = hda->sor->ctrl_num;
		if (sor_num < 0)
			goto err;

		mutex_lock(&hda_inst[sor_num].hda_inst_lock);
		tegra_dc_hda_put_clocks(hda);
		kfree(hda->audio_switch_name);
		kfree(hda);
		hda = NULL;
		hda_inst[sor_num].hda = NULL;
		hda_inst[sor_num].hda_state = HDA_UNINITIALIZED;
		mutex_unlock(&hda_inst[sor_num].hda_inst_lock);
	}

	for (i = 0; i < tegra_dc_get_numof_dispsors(); i++) {
		if (hda_inst[i].hda_state & HDA_INITIALIZED) {
			free_hda_mem = false;
			break;
		}
	}

	if (free_hda_mem) {
		for (i = 0; i < tegra_dc_get_numof_dispsors(); i++)
			mutex_destroy(&hda_inst[i].hda_inst_lock);

		kfree(hda_inst);
		hda_inst = NULL;
	}
err:
	mutex_unlock(&global_hda_lock);
}
