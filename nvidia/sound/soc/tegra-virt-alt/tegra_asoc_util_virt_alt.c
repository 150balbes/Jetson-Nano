/*
 * tegra_asoc_util_virt_alt.c - Tegra xbar dai link for machine drivers
 *
 * Copyright (c) 2017-2020, NVIDIA CORPORATION.  All rights reserved.
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
#include <linux/module.h>
#include <linux/platform_device.h>

#include "tegra_virt_alt_ivc.h"
#include "tegra_asoc_util_virt_alt.h"

static struct tegra_audio_metadata_cntx *metadata;

const int tegra186_arad_mux_value[] = {
	-1, /* None */
	0, 1, 2, 3, 4, 5,	/* I2S1~6 */
	28, 29, 30, 31,	/* SPDIF_RX1,2 & SPDIF_TX1,2 */
};

const char * const tegra186_arad_mux_text[] = {
	"None",
	"I2S1",
	"I2S2",
	"I2S3",
	"I2S4",
	"I2S5",
	"I2S6",
	"SPDIF1_RX1",
	"SPDIF1_RX2",
	"SPDIF1_TX1",
	"SPDIF1_TX2",
};

const char * const tegra186_asrc_ratio_source_text[] = {
	"ARAD",
	"SW",
};

const char * const tegra210_mvc_curve_type_text[] = {
	"Poly",
	"Linear",
};

int tegra_virt_t210mixer_get_gain(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AMIXER_GET_RX_GAIN;
	msg.params.amixer_info.id = 0;
	msg.params.amixer_info.rx_idx = (int) reg;

	err = nvaudio_ivc_send_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("%s: error on ivc_send_receive\n", __func__);

	ucontrol->value.integer.value[0] =
		msg.params.amixer_info.gain;

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mixer_get_gain);

int tegra_virt_t210mixer_set_gain(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AMIXER_SET_RX_GAIN;
	msg.params.amixer_info.id = 0;
	msg.params.amixer_info.rx_idx = (int) reg;
	msg.params.amixer_info.gain =
		ucontrol->value.integer.value[0];
	msg.params.amixer_info.is_instant_gain = 0;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mixer_set_gain);

int tegra_virt_t210mixer_set_gain_instant(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AMIXER_SET_RX_GAIN;
	msg.params.amixer_info.id = 0;
	msg.params.amixer_info.rx_idx = (int) reg;
	msg.params.amixer_info.gain =
		ucontrol->value.integer.value[0];
	msg.params.amixer_info.is_instant_gain = 1;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mixer_set_gain_instant);

int tegra_virt_t210mixer_get_duration(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AMIXER_GET_RX_DURATION;
	msg.params.amixer_info.id = 0;
	msg.params.amixer_info.rx_idx = (int) reg;

	err = nvaudio_ivc_send_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("%s: error on ivc_send_receive\n", __func__);

	ucontrol->value.integer.value[0] =
		msg.params.amixer_info.duration_n3;

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mixer_get_duration);

int tegra_virt_t210mixer_set_duration(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AMIXER_SET_RX_DURATION;
	msg.params.amixer_info.id = 0;
	msg.params.amixer_info.rx_idx = (int) reg;
	msg.params.amixer_info.duration_n3 =
		ucontrol->value.integer.value[0];
	msg.params.amixer_info.is_instant_gain = 0;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mixer_set_duration);

int tegra_virt_t210mixer_get_adder_config(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AMIXER_GET_TX_ADDER_CONFIG;
	msg.params.amixer_info.id = 0;
	msg.params.amixer_info.adder_idx = (((int) reg) >>
				MIXER_CONFIG_SHIFT_VALUE) & 0xFFFF;
	msg.params.amixer_info.adder_rx_idx = ((int) reg) & 0xFFFF;

	err = nvaudio_ivc_send_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("%s: error on ivc_send_receive\n", __func__);

	ucontrol->value.integer.value[0] =
		msg.params.amixer_info.adder_rx_idx_enable;

	if (err < 0)
		return err;

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mixer_get_adder_config);

int tegra_virt_t210mixer_set_adder_config(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm =
		snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err, connect;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AMIXER_SET_TX_ADDER_CONFIG;
	msg.params.amixer_info.id = 0;
	msg.params.amixer_info.adder_idx = (((int) reg) >>
				MIXER_CONFIG_SHIFT_VALUE) & 0xFFFF;
	msg.params.amixer_info.adder_rx_idx = ((int) reg) & 0xFFFF;
	msg.params.amixer_info.adder_rx_idx_enable =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	connect = !!ucontrol->value.integer.value[0];
	snd_soc_dapm_mixer_update_power(dapm, kcontrol, connect, NULL);

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mixer_set_adder_config);

int tegra_virt_t210mixer_get_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AMIXER_GET_ENABLE;
	msg.params.amixer_info.id = 0;

	err = nvaudio_ivc_send_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("%s: error on ivc_send_receive\n", __func__);

	ucontrol->value.integer.value[0] = msg.params.amixer_info.enable;

	if (err < 0)
		return err;

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mixer_get_enable);
int tegra_virt_t210mixer_set_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AMIXER_SET_ENABLE;
	msg.params.amixer_info.id = 0;
	msg.params.amixer_info.enable =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mixer_set_enable);

int tegra_virt_t210sfc_get_in_freq(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_SFC_GET_IN_FREQ;
	msg.params.sfc_info.id = reg;

	err = nvaudio_ivc_send_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	ucontrol->value.integer.value[0] = msg.params.sfc_info.in_freq;

	if (err < 0) {
		pr_err("%s: error on ivc_send_receive\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210sfc_get_in_freq);
int tegra_virt_t210sfc_set_in_freq(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_SFC_SET_IN_FREQ;
	msg.params.sfc_info.id = reg;
	msg.params.sfc_info.in_freq =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210sfc_set_in_freq);

int tegra_virt_t210sfc_get_out_freq(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_SFC_GET_OUT_FREQ;
	msg.params.sfc_info.id = reg;

	err = nvaudio_ivc_send_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	ucontrol->value.integer.value[0] = msg.params.sfc_info.out_freq;

	if (err < 0) {
		pr_err("%s: error on ivc_send_receive\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210sfc_get_out_freq);
int tegra_virt_t210sfc_set_out_freq(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_SFC_SET_OUT_FREQ;
	msg.params.sfc_info.id = reg;
	msg.params.sfc_info.out_freq =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210sfc_set_out_freq);

int tegra_virt_t210mvc_get_curve_type(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_MVC_GET_CURVETYPE;
	msg.params.mvc_info.id = reg;

	err = nvaudio_ivc_send_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: error on ivc_send_receive\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] = msg.params.mvc_info.curve_type;

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mvc_get_curve_type);

int tegra_virt_t210mvc_set_curve_type(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	long int reg = (long int)kcontrol->tlv.p;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_MVC_SET_CURVETYPE;
	msg.params.mvc_info.id = reg;
	msg.params.mvc_info.curve_type =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mvc_set_curve_type);

int tegra_virt_t210mvc_get_tar_vol(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
			nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_MVC_GET_TAR_VOL;
	msg.params.mvc_info.id = reg;

	err = nvaudio_ivc_send_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: error on ivc_send_receive\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] = msg.params.mvc_info.tar_vol;

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mvc_get_tar_vol);

int tegra_virt_t210mvc_set_tar_vol(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_MVC_SET_TAR_VOL;
	msg.params.mvc_info.id = reg;
	msg.params.mvc_info.tar_vol =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}
	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mvc_set_tar_vol);

int tegra_virt_t210mvc_get_mute(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_MVC_GET_MUTE;
	msg.params.mvc_info.id = reg;

	err = nvaudio_ivc_send_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	if (err < 0) {
		pr_err("%s: error on ivc_send_receive\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] = msg.params.mvc_info.mute;

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mvc_get_mute);

int tegra_virt_t210mvc_set_mute(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_MVC_SET_MUTE;
	msg.params.mvc_info.id = reg;
	msg.params.mvc_info.mute =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}
	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mvc_set_mute);

int tegra186_virt_asrc_get_ratio(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mreg_control *mc =
		(struct soc_mreg_control *)kcontrol->private_value;
	unsigned int reg = mc->regbase;
	int err;
	uint64_t val;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_GET_RATIO;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;

	err = nvaudio_ivc_send_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: error on ivc_send_receive\n", __func__);
		return err;
	}

	val = (uint64_t) msg.params.asrc_info.int_ratio << 32;
	val &= 0xffffffff00000000ULL;
	val |= msg.params.asrc_info.frac_ratio;
	ucontrol->value.integer64.value[0] = val;

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_get_ratio);

int tegra186_virt_asrc_set_ratio(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mreg_control *mc =
		(struct soc_mreg_control *)kcontrol->private_value;
	unsigned int reg = mc->regbase;
	int err;
	uint64_t val;
	struct nvaudio_ivc_msg msg;

	val = ucontrol->value.integer64.value[0];

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_SET_RATIO;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;
	msg.params.asrc_info.int_ratio =
		(val >> 32) & 0xffffffffULL;
	msg.params.asrc_info.frac_ratio =
		(val & 0xffffffffULL);

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_set_ratio);

int tegra186_virt_asrc_get_ratio_source(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	uint64_t reg = (uint64_t)kcontrol->tlv.p;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_GET_RATIO_SOURCE;
	msg.params.asrc_info.stream_num = reg;

	err = nvaudio_ivc_send_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: error on ivc_send_receive\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] = msg.params.asrc_info.ratio_source;

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_get_ratio_source);

int tegra186_virt_asrc_set_ratio_source(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	uint64_t reg = (uint64_t)kcontrol->tlv.p;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_SET_RATIO_SOURCE;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;
	msg.params.asrc_info.ratio_source =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_set_ratio_source);

int tegra186_virt_asrc_get_stream_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_GET_STREAM_ENABLE;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;

	err = nvaudio_ivc_send_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: error on ivc_send_receive\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] = msg.params.asrc_info.stream_enable;

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_get_stream_enable);

int tegra186_virt_asrc_set_stream_enable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_SET_STREAM_ENABLE;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;
	msg.params.asrc_info.stream_enable =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_set_stream_enable);

int tegra186_virt_asrc_get_hwcomp_disable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_GET_HWCOMP_DISABLE;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;

	err = nvaudio_ivc_send_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] = msg.params.asrc_info.hwcomp_disable;

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_get_hwcomp_disable);

int tegra186_virt_asrc_set_hwcomp_disable(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_SET_HWCOMP_DISABLE;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;
	msg.params.asrc_info.hwcomp_disable =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_set_hwcomp_disable);

int tegra186_virt_asrc_get_input_threshold(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_GET_INPUT_THRESHOLD;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;

	err = nvaudio_ivc_send_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));

	ucontrol->value.integer.value[0] = msg.params.sfc_info.out_freq;

	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] = msg.params.asrc_info.input_threshold;

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_get_input_threshold);

int tegra186_virt_asrc_set_input_threshold(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_SET_INPUT_THRESHOLD;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;
	msg.params.asrc_info.input_threshold =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_set_input_threshold);

int tegra186_virt_asrc_get_output_threshold(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_GET_OUTPUT_THRESHOLD;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;

	err = nvaudio_ivc_send_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] =
			msg.params.asrc_info.output_threshold;

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_get_output_threshold);

int tegra186_virt_asrc_set_output_threshold(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ASRC_SET_OUTPUT_THRESHOLD;
	msg.params.asrc_info.id = 0;
	msg.params.asrc_info.stream_num = reg;
	msg.params.asrc_info.output_threshold =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_asrc_set_output_threshold);

int tegra_virt_t210_amx_get_input_stream_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210_amx_get_input_stream_enable);

int tegra_virt_t210_amx_set_input_stream_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AMX_SET_INPUT_STREAM_ENABLE;
	msg.params.amx_info.amx_id = (((int) reg) >>
				MIXER_CONFIG_SHIFT_VALUE) & 0xFFFF;
	msg.params.amx_info.amx_stream_id = ((int) reg) & 0xFFFF;
	msg.params.amx_info.amx_stream_enable =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210_amx_set_input_stream_enable);

int tegra186_virt_arad_get_lane_source(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	uint64_t reg = (uint64_t)kcontrol->tlv.p;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err, i;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ARAD_GET_LANE_SRC;
	msg.params.arad_info.id = 0;
	msg.params.arad_info.lane_id = reg % NUM_ARAD_LANES;

	err = nvaudio_ivc_send_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	/* numerator reg 0 to 5, denominator reg 6 to 11 */
	if (reg/NUM_ARAD_LANES) {
		for (i = 0; i < NUM_ARAD_SOURCES; i++) {
			if (e->values[i] ==
					msg.params.arad_info.den_source)
				break;
		}
		ucontrol->value.integer.value[0] = i;
	} else {
		for (i = 0; i < NUM_ARAD_SOURCES; i++) {
			if (e->values[i] ==
					msg.params.arad_info.num_source)
				break;
		}
		ucontrol->value.integer.value[0] = i;
	}

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_arad_get_lane_source);

int tegra186_virt_arad_set_lane_source(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	uint64_t reg = (uint64_t)kcontrol->tlv.p;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;
	int source;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ARAD_SET_LANE_SRC;
	msg.params.arad_info.id = 0;
	msg.params.arad_info.lane_id = reg % NUM_ARAD_LANES;

	/* numerator reg 0 to 5, denominator reg 6 to 11 */
	source = e->values[ucontrol->value.integer.value[0]];
	if (reg/NUM_ARAD_LANES) {
		msg.params.arad_info.num_source = -1;
		msg.params.arad_info.den_source = source;
	} else {
		msg.params.arad_info.num_source = source;
		msg.params.arad_info.den_source = -1;
	}

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_arad_set_lane_source);

int tegra186_virt_arad_get_lane_prescalar(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ARAD_GET_PRESCALAR;
	msg.params.arad_info.id = 0;
	msg.params.arad_info.lane_id = reg % NUM_ARAD_LANES;

	err = nvaudio_ivc_send_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	/* numerator reg 0 to 5, denominator reg 6 to 11 */
	if (reg/NUM_ARAD_LANES)
		ucontrol->value.integer.value[0] =
			msg.params.arad_info.den_prescalar;
	else
		ucontrol->value.integer.value[0] =
			msg.params.arad_info.num_prescalar;

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_arad_get_lane_prescalar);

int tegra186_virt_arad_set_lane_prescalar(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ARAD_SET_PRESCALAR;
	msg.params.arad_info.id = 0;
	msg.params.arad_info.lane_id = reg % NUM_ARAD_LANES;

	/* numerator reg 0 to 5, denominator reg 6 to 11 */
	if (reg/NUM_ARAD_LANES) {
		msg.params.arad_info.num_prescalar = -1;
		msg.params.arad_info.den_prescalar =
			ucontrol->value.integer.value[0];
	} else {
		msg.params.arad_info.num_prescalar =
			ucontrol->value.integer.value[0];
		msg.params.arad_info.den_prescalar = -1;
	}

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_arad_set_lane_prescalar);

int tegra186_virt_arad_get_lane_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ARAD_GET_LANE_ENABLE;
	msg.params.arad_info.id = 0;
	msg.params.arad_info.lane_id = reg;

	err = nvaudio_ivc_send_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] =
			msg.params.arad_info.lane_enable;

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_arad_get_lane_enable);

int tegra186_virt_arad_set_lane_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ARAD_SET_LANE_ENABLE;
	msg.params.arad_info.id = 0;
	msg.params.arad_info.lane_id = reg;
	msg.params.arad_info.lane_enable =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_arad_set_lane_enable);

int tegra186_virt_arad_get_lane_ratio(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	uint64_t val;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ARAD_GET_LANE_RATIO;
	msg.params.arad_info.id = 0;
	msg.params.arad_info.lane_id = reg;

	err = nvaudio_ivc_send_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	val = (uint64_t)msg.params.arad_info.int_ratio << 32;
	val &= 0xffffffff00000000ULL;
	val |= (uint64_t)msg.params.arad_info.frac_ratio;
	ucontrol->value.integer64.value[0] = val;

	return 0;
}
EXPORT_SYMBOL(tegra186_virt_arad_get_lane_ratio);

int tegra_virt_i2s_get_loopback_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_I2S_GET_LOOPBACK_ENABLE;
	msg.params.i2s_info.i2s_id = reg;

	err = nvaudio_ivc_send_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] =
			msg.params.i2s_info.i2s_loopback_enable;

	return 0;
}
EXPORT_SYMBOL(tegra_virt_i2s_get_loopback_enable);

int tegra_virt_i2s_set_loopback_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_I2S_SET_LOOPBACK_ENABLE;
	msg.params.i2s_info.i2s_id = reg;
	msg.params.i2s_info.i2s_loopback_enable =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_i2s_set_loopback_enable);

int tegra_virt_i2s_get_rate(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_I2S_GET_RATE;
	msg.params.i2s_info.i2s_id = reg;

	err = nvaudio_ivc_send_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	ucontrol->value.integer.value[0] =
			msg.params.i2s_info.i2s_rate;

	return 0;
}
EXPORT_SYMBOL(tegra_virt_i2s_get_rate);

int tegra_virt_i2s_set_rate(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_I2S_SET_RATE;
	msg.params.i2s_info.i2s_id = reg;
	msg.params.i2s_info.i2s_rate =
		ucontrol->value.integer.value[0];

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_i2s_set_rate);

int tegra_virt_get_metadata(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	uint8_t *m = ucontrol->value.bytes.data;

	tegra_metadata_flood_get(m);

	return 0;
}
EXPORT_SYMBOL(tegra_virt_get_metadata);

int tegra_virt_set_metadata(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	uint8_t *m = ucontrol->value.bytes.data;

	tegra_metadata_flood_update(m);

	return 0;
}
EXPORT_SYMBOL(tegra_virt_set_metadata);

int tegra_metadata_get_init(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =
			metadata->init_metadata_flood;

	return 0;
}
EXPORT_SYMBOL(tegra_metadata_get_init);

int tegra_metadata_set_init(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);

	if (metadata->init_metadata_flood == ucontrol->value.integer.value[0])
		return 0;

	if (ucontrol->value.integer.value[0]) {
		if (tegra_metadata_flood_init(metadata, card->dev)) {
			dev_err(card->dev, "failed to initialize metadata\n");
			return -1;
		}
	} else {
		if (metadata->enable_metadata_flood) {
			dev_err(card->dev, "META flood is enabled, disable first\n");
			return -1;
		}
		tegra_metadata_flood_deinit(metadata, card->dev);
	}

	return 0;
}
EXPORT_SYMBOL(tegra_metadata_set_init);

int tegra_metadata_get_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] =
			metadata->enable_metadata_flood;

	return 0;
}
EXPORT_SYMBOL(tegra_metadata_get_enable);

int tegra_metadata_set_enable(
	struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);

	if (!metadata->init_metadata_flood) {
		dev_err(card->dev, "META flood has not been initialized yet\n");
		return -1;
	}

	tegra_metadata_flood_enable(metadata, ucontrol->value.integer.value[0],
								card->dev);

	return 0;
}
EXPORT_SYMBOL(tegra_metadata_set_enable);


int tegra_metadata_setup(struct platform_device *pdev,
	struct tegra_audio_metadata_cntx *psad, struct snd_soc_card *card)
{

	metadata = psad;
	if (of_property_read_u32_index(pdev->dev.of_node, "nvidia,adma_ch_page",
			0, &metadata->dma_ch_page)) {
		dev_info(&pdev->dev, "META dma channel page address dt entry not found\n");
		goto lb;
	}
	if (of_property_read_u32_index(pdev->dev.of_node,
				"nvidia,sad_admaif_id", 0,
					&metadata->admaif_id)) {
		dev_info(&pdev->dev, "META admaif id dt entry not found\n");
		goto lb;
	}
	if (of_property_read_u32_index(pdev->dev.of_node, "nvidia,sad_dma_id",
			0, &metadata->dma_id)) {
		dev_info(&pdev->dev, "META dma id dt entry not found\n");
		goto lb;
	}
	if (of_property_read_u32_index(pdev->dev.of_node,
			"nvidia,sad_header_mode", 0,
					&metadata->metadata_mode)) {
		dev_info(&pdev->dev, "META header mode dt entry not found\n");
		goto lb;
	}

	if (metadata->metadata_mode == SUBFRAME_MODE) {
		if (tegra_metadata_flood_init(metadata, card->dev)) {
			dev_err(&pdev->dev, "failed to initialize metadata\n");
			goto lb;
		}

		if (tegra_metadata_flood_enable(metadata, 1, card->dev)) {
			dev_err(&pdev->dev, "failed to initialize\n");
			goto lb;
		}
	}
lb:
	return 0;
}
EXPORT_SYMBOL(tegra_metadata_setup);

int tegra_virt_t210ahub_get_regdump(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210ahub_get_regdump);

int tegra_virt_t210ahub_set_regdump(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AHUB_BLOCK_REGDUMP;
	msg.params.ahub_block_info.block_id = ((int) reg) & 0xFFFF;
	msg.params.ahub_block_info.stream_id = (((int) reg) >>
				STREAM_ID_SHIFT_VALUE) & 0xFF;
	msg.params.ahub_block_info.dump_cmd = (((int) reg) >>
				REGDUMP_CMD_SHIFT_VALUE) & 0xFF;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210ahub_set_regdump);


int tegra_virt_t210adma_get_regdump(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210adma_get_regdump);

int tegra_virt_t210adma_set_regdump(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	unsigned int reg = mc->reg;
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_ADMA_BLOCK_REGDUMP;
	msg.params.adma_info.channel_num = (uint32_t)reg;

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210adma_set_regdump);

//Set mixer fade
int tegra_virt_t210mixer_set_fade(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{

	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);
	int err, id;
	uint32_t rx_id, rx_gain, rx_dur;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AMIXER_SET_FADE;
	msg.params.fade_info.id = 0;

	for (id = 0; id < TEGRA210_MIXER_AXBAR_RX_MAX; id++) {

		rx_id = ucontrol->value.integer.value[3 * id];
		rx_gain = ucontrol->value.integer.value[(3 * id) + 1];
		rx_dur = ucontrol->value.integer.value[(3 * id) + 2];

		// Checking for end of input data
		if (rx_id == 0 && rx_gain == 0 && rx_dur == 0)
			break;

		// Checking for valid rx id
		if (rx_id <= 0 || rx_id > TEGRA210_MIXER_AXBAR_RX_MAX) {
			pr_err("Mixer id is out of range\n");
			return -EINVAL;
		}

		// Making rx id zero-indexed for audio server
		rx_id = rx_id - 1;

		msg.params.fade_info.rx_idx |= (1 << rx_id);
		msg.params.fade_info.gain_level[rx_id] = rx_gain;
		msg.params.fade_info.duration_n3[rx_id] = rx_dur;

	}

	err = nvaudio_ivc_send_retry(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0) {
		pr_err("%s: Timedout on ivc_send_retry\n", __func__);
		return err;
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mixer_set_fade);

// Dummy get fade
int tegra_virt_t210mixer_get_fade(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mixer_get_fade);

//Get fade status
int tegra_virt_t210mixer_get_fade_status(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct nvaudio_ivc_ctxt *hivc_client =
		nvaudio_ivc_alloc_ctxt(card->dev);

	int err, id;
	struct nvaudio_ivc_msg msg;

	memset(&msg, 0, sizeof(struct nvaudio_ivc_msg));
	msg.cmd = NVAUDIO_AMIXER_GET_FADE_STATUS;
	msg.params.fade_status.id = 0;

	err = nvaudio_ivc_send_receive(hivc_client,
			&msg,
			sizeof(struct nvaudio_ivc_msg));
	if (err < 0)
		pr_err("%s: error on ivc_send_receive\n", __func__);

	for (id = 0; id < TEGRA210_MIXER_AXBAR_RX_MAX; id++) {
		ucontrol->value.integer.value[id] =
			msg.params.fade_status.status[id];
	}

	return 0;
}
EXPORT_SYMBOL(tegra_virt_t210mixer_get_fade_status);

//Fade param info
int tegra_virt_t210mixer_param_info(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_info *uinfo)
{
	struct soc_bytes *params = (void *)kcontrol->private_value;

	if (params->mask == SNDRV_CTL_ELEM_TYPE_INTEGER) {
		params->num_regs = 30;
		uinfo->value.integer.min = 0;
		uinfo->value.integer.max = 0xffffffff;
	}
	uinfo->type = params->mask;
	uinfo->count = params->num_regs;

	return 0;
}

MODULE_AUTHOR("Dipesh Gandhi <dipeshg@nvidia.com>");
MODULE_DESCRIPTION("Tegra Virt ASoC utility code");
MODULE_LICENSE("GPL");
