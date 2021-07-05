/******************************************************************************
 *
 * Copyright(c) 2007 - 2018 Realtek Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 *****************************************************************************/
#define _RTW_CHPLAN_C_

#include <drv_types.h>

#define RTW_DOMAIN_MAP_VER	"41e"
#define RTW_COUNTRY_MAP_VER	"24"
#define RTW_HEXFILE_LEN 2001

#define RTW_RD_2G_MAX 13
#define RTW_RD_5G_MAX 73

struct ch_list_t {
	u8 *len_ch;
};

#define CH_LIST_ENT(_len, arg...) \
	{.len_ch = (u8[_len + 1]) {_len, ##arg}, }

#define CH_LIST_LEN(_ch_list) (_ch_list.len_ch[0])
#define CH_LIST_CH(_ch_list, _i) (_ch_list.len_ch[_i + 1])

struct chplan_ent_t {
	u8 rd_2g;
#ifdef CONFIG_IEEE80211_BAND_5GHZ
	u8 rd_5g;
#endif
	u8 regd; /* value of REGULATION_TXPWR_LMT */
};

#ifdef CONFIG_IEEE80211_BAND_5GHZ
#define CHPLAN_ENT(i2g, i5g, regd) {i2g, i5g, regd}
#else
#define CHPLAN_ENT(i2g, i5g, regd) {i2g, regd}
#endif

static u8 rtw_hex_setting_buf[RTW_HEXFILE_LEN];
static struct ch_list_t RTW_ChannelPlan2G[RTW_RD_2G_MAX];

#ifdef CONFIG_IEEE80211_BAND_5GHZ
static struct ch_list_t RTW_ChannelPlan5G[RTW_RD_5G_MAX]; 
#endif /* CONFIG_IEEE80211_BAND_5GHZ */

static struct chplan_ent_t RTW_ChannelPlanMap[RTW_CHPLAN_MAX];
static struct country_chplan country_chplan_map[238];
int rtw_get_channel_plan_from_file(const char *path)
{
	u32 len, idx;
	u32 i = 0, j = 0;
	u8 sec_1, sec_2;
	u16 sec_3, chksum;

	len = rtw_retrieve_from_file(path, rtw_hex_setting_buf, RTW_HEXFILE_LEN);
	if (len > RTW_HEXFILE_LEN) {
		RTW_WARN("channel plan hexfile oversize\n");
		return -1;
	}
	if (len == 0) {
		RTW_WARN("read channel plan hexfile fail\n");
		return -1;
	}
	chksum = *(u16 *)(rtw_hex_setting_buf);
	if (rtw_calc_crc(rtw_hex_setting_buf + 2, len - 2) != chksum) {
		RTW_WARN("read channel plan fail(tainted)\n");
		return -1;
	}
	sec_1 = rtw_hex_setting_buf[2];
	sec_2 = sec_1 + rtw_hex_setting_buf[3];
	sec_3 = sec_2 + rtw_hex_setting_buf[4];
	i += 5;
	while (i < len) {
		if (j < sec_1) {
			RTW_ChannelPlan2G[j].len_ch = rtw_hex_setting_buf + i;
			i += (rtw_hex_setting_buf[i] + 1);
		} else if (j >= sec_1 && j < sec_2) {
			idx = j - sec_1;
			RTW_ChannelPlan5G[idx].len_ch = rtw_hex_setting_buf + i;
			i += (rtw_hex_setting_buf[i] + 1);
		} else if (j >= sec_2 && j < sec_3) {
			if (i + 2 >= len)
				return -1;
			idx = j - sec_2;
			RTW_ChannelPlanMap[idx].rd_2g =
						*(rtw_hex_setting_buf + i);
			RTW_ChannelPlanMap[idx].rd_5g =
						*(rtw_hex_setting_buf + i + 1);
			RTW_ChannelPlanMap[idx].regd =
						*(rtw_hex_setting_buf + i + 2);
			i += 3;
		} else {
			break;
		}
		j++;
	}
	return 0;
}
 
u8 rtw_chplan_get_default_regd(u8 id)
{
	u8 regd;

	regd = RTW_ChannelPlanMap[id].regd;

	return regd;
}

bool rtw_chplan_is_empty(u8 id)
{
	struct chplan_ent_t *chplan_map;

	chplan_map = &RTW_ChannelPlanMap[id];

	if (chplan_map->rd_2g == 8
		#ifdef CONFIG_IEEE80211_BAND_5GHZ
		&& chplan_map->rd_5g == 0
		#endif
	)
		return _TRUE;

	return _FALSE;
}

bool rtw_regsty_is_excl_chs(struct registry_priv *regsty, u8 ch)
{
	int i;

	for (i = 0; i < MAX_CHANNEL_NUM; i++) {
		if (regsty->excl_chs[i] == 0)
			break;
		if (regsty->excl_chs[i] == ch)
			return _TRUE;
	}
	return _FALSE;
}

inline static u8 rtw_rd_5g_band1_passive(u8 rtw_rd_5g)
{
	u8 passive = 0;

	switch (rtw_rd_5g) {
	case 41:
	case 44:
	case 45:
	case 46:
	case 52:
	case 55:
	case 59:
	case 60:
	case 63:
		passive = 1;
	};

	return passive;
}

inline static u8 rtw_rd_5g_band4_passive(u8 rtw_rd_5g)
{
	u8 passive = 0;

	switch (rtw_rd_5g) {
	case 37:
	case 38:
	case 45:
	case 46:
	case 52:
	case 61:
	case 63:
		passive = 1;
	};

	return passive;
}

u8 init_channel_set(_adapter *padapter, u8 ChannelPlan, RT_CHANNEL_INFO *channel_set)
{
	struct registry_priv *regsty = adapter_to_regsty(padapter);
	u8	index, chanset_size = 0;
	u8	b5GBand = _FALSE, b2_4GBand = _FALSE;
	u8	rd_2g = 0, rd_5g = 0;
#ifdef CONFIG_DFS_MASTER
	int i;
#endif

	if (!rtw_is_channel_plan_valid(ChannelPlan)) {
		RTW_ERR("ChannelPlan ID 0x%02X error !!!!!\n", ChannelPlan);
		return chanset_size;
	}

	_rtw_memset(channel_set, 0, sizeof(RT_CHANNEL_INFO) * MAX_CHANNEL_NUM);

	if (IsSupported24G(regsty->wireless_mode) && hal_chk_band_cap(padapter, BAND_CAP_2G))
		b2_4GBand = _TRUE;

	if (is_supported_5g(regsty->wireless_mode) && hal_chk_band_cap(padapter, BAND_CAP_5G))
		b5GBand = _TRUE;

	if (b2_4GBand == _FALSE && b5GBand == _FALSE) {
		RTW_WARN("HW band_cap has no intersection with SW wireless_mode setting\n");
		return chanset_size;
	}

	if (b2_4GBand) {
		rd_2g = RTW_ChannelPlanMap[ChannelPlan].rd_2g;

		for (index = 0; index < CH_LIST_LEN(RTW_ChannelPlan2G[rd_2g]); index++) {
			if (rtw_regsty_is_excl_chs(regsty, CH_LIST_CH(RTW_ChannelPlan2G[rd_2g], index)) == _TRUE)
				continue;

			if (chanset_size >= MAX_CHANNEL_NUM) {
				RTW_WARN("chset size can't exceed MAX_CHANNEL_NUM(%u)\n", MAX_CHANNEL_NUM);
				break;
			}

			channel_set[chanset_size].ChannelNum = CH_LIST_CH(RTW_ChannelPlan2G[rd_2g], index);

			if (ChannelPlan == RTW_CHPLAN_GLOBAL_DOAMIN
				|| rd_2g == 5
			) {
				/* Channel 1~11 is active, and 12~14 is passive */
				if (channel_set[chanset_size].ChannelNum >= 1 && channel_set[chanset_size].ChannelNum <= 11)
					channel_set[chanset_size].ScanType = SCAN_ACTIVE;
				else if ((channel_set[chanset_size].ChannelNum  >= 12 && channel_set[chanset_size].ChannelNum  <= 14))
					channel_set[chanset_size].ScanType  = SCAN_PASSIVE;
			} else if (ChannelPlan == RTW_CHPLAN_WORLD_WIDE_13
				|| ChannelPlan == RTW_CHPLAN_WORLD_WIDE_5G
				|| rd_2g == 0
			) {
				/* channel 12~13, passive scan */
				if (channel_set[chanset_size].ChannelNum <= 11)
					channel_set[chanset_size].ScanType = SCAN_ACTIVE;
				else
					channel_set[chanset_size].ScanType = SCAN_PASSIVE;
			} else
				channel_set[chanset_size].ScanType = SCAN_ACTIVE;

			chanset_size++;
		}
	}

#ifdef CONFIG_IEEE80211_BAND_5GHZ
	if (b5GBand) {
		rd_5g = RTW_ChannelPlanMap[ChannelPlan].rd_5g;

		for (index = 0; index < CH_LIST_LEN(RTW_ChannelPlan5G[rd_5g]); index++) {
			if (rtw_regsty_is_excl_chs(regsty, CH_LIST_CH(RTW_ChannelPlan5G[rd_5g], index)) == _TRUE)
				continue;
			#ifndef CONFIG_DFS
			if (rtw_is_dfs_ch(CH_LIST_CH(RTW_ChannelPlan5G[rd_5g], index)))
				continue;
			#endif

			if (chanset_size >= MAX_CHANNEL_NUM) {
				RTW_WARN("chset size can't exceed MAX_CHANNEL_NUM(%u)\n", MAX_CHANNEL_NUM);
				break;
			}

			channel_set[chanset_size].ChannelNum = CH_LIST_CH(RTW_ChannelPlan5G[rd_5g], index);

			if ((ChannelPlan == RTW_CHPLAN_WORLD_WIDE_5G) /* all channels passive */
				|| (rtw_is_5g_band1(channel_set[chanset_size].ChannelNum)
					&& rtw_rd_5g_band1_passive(rd_5g)) /* band1 passive */
				|| (rtw_is_5g_band4(channel_set[chanset_size].ChannelNum)
					&& rtw_rd_5g_band4_passive(rd_5g)) /* band4 passive */
				|| (rtw_is_dfs_ch(channel_set[chanset_size].ChannelNum)) /* DFS channel(band2, 3) passive */
			)
				channel_set[chanset_size].ScanType = SCAN_PASSIVE;
			else
				channel_set[chanset_size].ScanType = SCAN_ACTIVE;

			chanset_size++;
		}
	}

	#ifdef CONFIG_DFS_MASTER
	for (i = 0; i < chanset_size; i++)
		channel_set[i].non_ocp_end_time = rtw_get_current_time();
	#endif
#endif /* CONFIG_IEEE80211_BAND_5GHZ */

	if (chanset_size)
		RTW_INFO(FUNC_ADPT_FMT" ChannelPlan ID:0x%02x, ch num:%d\n"
			, FUNC_ADPT_ARG(padapter), ChannelPlan, chanset_size);
	else
		RTW_WARN(FUNC_ADPT_FMT" ChannelPlan ID:0x%02x, final chset has no channel\n"
			, FUNC_ADPT_ARG(padapter), ChannelPlan);

	return chanset_size;
}

#ifdef CONFIG_80211AC_VHT
#define COUNTRY_CHPLAN_ASSIGN_EN_11AC(_val) , .en_11ac = (_val)
#else
#define COUNTRY_CHPLAN_ASSIGN_EN_11AC(_val)
#endif

#if RTW_DEF_MODULE_REGULATORY_CERT
#define COUNTRY_CHPLAN_ASSIGN_DEF_MODULE_FLAGS(_val) , .def_module_flags = (_val)
#else
#define COUNTRY_CHPLAN_ASSIGN_DEF_MODULE_FLAGS(_val)
#endif

/* has def_module_flags specified, used by common map and HAL dfference map */
#define COUNTRY_CHPLAN_ENT(_alpha2, _chplan, _en_11ac, _def_module_flags) \
	{.alpha2 = (_alpha2), .chplan = (_chplan) \
		COUNTRY_CHPLAN_ASSIGN_EN_11AC(_en_11ac) \
		COUNTRY_CHPLAN_ASSIGN_DEF_MODULE_FLAGS(_def_module_flags) \
	}

#ifdef CONFIG_CUSTOMIZED_COUNTRY_CHPLAN_MAP

#include "../platform/custom_country_chplan.h"

#elif RTW_DEF_MODULE_REGULATORY_CERT

/**
 * rtw_def_module_get_chplan_from_country -
 * @country_code: string of country code
 * @return:
 * Return NULL for case referring to common map
 */
static const struct country_chplan *rtw_def_module_get_chplan_from_country(const char *country_code)
{
	const struct country_chplan *ent = NULL;
	const struct country_chplan *hal_map = NULL;
	u16 hal_map_sz = 0;
	int i;

	/* TODO: runtime selection for multi driver */
#if (RTW_DEF_MODULE_REGULATORY_CERT == RTW_MODULE_RTL8821AE_HMC_M2)
	hal_map = RTL8821AE_HMC_M2_country_chplan_exc_map;
	hal_map_sz = sizeof(RTL8821AE_HMC_M2_country_chplan_exc_map) / sizeof(struct country_chplan);
#elif (RTW_DEF_MODULE_REGULATORY_CERT == RTW_MODULE_RTL8821AU)
	hal_map = RTL8821AU_country_chplan_exc_map;
	hal_map_sz = sizeof(RTL8821AU_country_chplan_exc_map) / sizeof(struct country_chplan);
#elif (RTW_DEF_MODULE_REGULATORY_CERT == RTW_MODULE_RTL8812AENF_NGFF)
	hal_map = RTL8812AENF_NGFF_country_chplan_exc_map;
	hal_map_sz = sizeof(RTL8812AENF_NGFF_country_chplan_exc_map) / sizeof(struct country_chplan);
#elif (RTW_DEF_MODULE_REGULATORY_CERT == RTW_MODULE_RTL8812AEBT_HMC)
	hal_map = RTL8812AEBT_HMC_country_chplan_exc_map;
	hal_map_sz = sizeof(RTL8812AEBT_HMC_country_chplan_exc_map) / sizeof(struct country_chplan);
#elif (RTW_DEF_MODULE_REGULATORY_CERT == RTW_MODULE_RTL8188EE_HMC_M2)
	hal_map = RTL8188EE_HMC_M2_country_chplan_exc_map;
	hal_map_sz = sizeof(RTL8188EE_HMC_M2_country_chplan_exc_map) / sizeof(struct country_chplan);
#elif (RTW_DEF_MODULE_REGULATORY_CERT == RTW_MODULE_RTL8723BE_HMC_M2)
	hal_map = RTL8723BE_HMC_M2_country_chplan_exc_map;
	hal_map_sz = sizeof(RTL8723BE_HMC_M2_country_chplan_exc_map) / sizeof(struct country_chplan);
#elif (RTW_DEF_MODULE_REGULATORY_CERT == RTW_MODULE_RTL8723BS_NGFF1216)
	hal_map = RTL8723BS_NGFF1216_country_chplan_exc_map;
	hal_map_sz = sizeof(RTL8723BS_NGFF1216_country_chplan_exc_map) / sizeof(struct country_chplan);
#elif (RTW_DEF_MODULE_REGULATORY_CERT == RTW_MODULE_RTL8192EEBT_HMC_M2)
	hal_map = RTL8192EEBT_HMC_M2_country_chplan_exc_map;
	hal_map_sz = sizeof(RTL8192EEBT_HMC_M2_country_chplan_exc_map) / sizeof(struct country_chplan);
#elif (RTW_DEF_MODULE_REGULATORY_CERT == RTW_MODULE_RTL8723DE_NGFF1630)
	hal_map = RTL8723DE_NGFF1630_country_chplan_exc_map;
	hal_map_sz = sizeof(RTL8723DE_NGFF1630_country_chplan_exc_map) / sizeof(struct country_chplan);
#elif (RTW_DEF_MODULE_REGULATORY_CERT == RTW_MODULE_RTL8822BE)
	hal_map = RTL8822BE_country_chplan_exc_map;
	hal_map_sz = sizeof(RTL8822BE_country_chplan_exc_map) / sizeof(struct country_chplan);
#elif (RTW_DEF_MODULE_REGULATORY_CERT == RTW_MODULE_RTL8821CE)
	hal_map = RTL8821CE_country_chplan_exc_map;
	hal_map_sz = sizeof(RTL8821CE_country_chplan_exc_map) / sizeof(struct country_chplan);
#endif

	if (hal_map == NULL || hal_map_sz == 0)
		goto exit;

	for (i = 0; i < hal_map_sz; i++) {
		if (strncmp(country_code, hal_map[i].alpha2, 2) == 0) {
			ent = &hal_map[i];
			break;
		}
	}

exit:
	return ent;
}
#endif /* CONFIG_CUSTOMIZED_COUNTRY_CHPLAN_MAP or RTW_DEF_MODULE_REGULATORY_CERT */

/*
* rtw_get_chplan_from_country -
* @country_code: string of country code
*
* Return pointer of struct country_chplan entry or NULL when unsupported country_code is given
*/
const struct country_chplan *rtw_get_chplan_from_country(const char *country_code)
{
#if RTW_DEF_MODULE_REGULATORY_CERT
	const struct country_chplan *exc_ent = NULL;
#endif
	const struct country_chplan *ent = NULL;
	const struct country_chplan *map = NULL;
	u16 map_sz = 0;
	char code[2];
	int i;

	code[0] = alpha_to_upper(country_code[0]);
	code[1] = alpha_to_upper(country_code[1]);

#ifdef CONFIG_CUSTOMIZED_COUNTRY_CHPLAN_MAP
	map = CUSTOMIZED_country_chplan_map;
	map_sz = sizeof(CUSTOMIZED_country_chplan_map) / sizeof(struct country_chplan);
#else
	#if RTW_DEF_MODULE_REGULATORY_CERT
	exc_ent = rtw_def_module_get_chplan_from_country(code);
	#endif
	map = country_chplan_map;
	map_sz = sizeof(country_chplan_map) / sizeof(struct country_chplan);
#endif

	for (i = 0; i < map_sz; i++) {
		if (strncmp(code, map[i].alpha2, 2) == 0) {
			ent = &map[i];
			break;
		}
	}

	#if RTW_DEF_MODULE_REGULATORY_CERT
	if (!ent || !(COUNTRY_CHPLAN_DEF_MODULE_FALGS(ent) & RTW_DEF_MODULE_REGULATORY_CERT))
		exc_ent = ent = NULL;
	if (exc_ent)
		ent = exc_ent;
	#endif

	return ent;
}

void dump_country_chplan(void *sel, const struct country_chplan *ent)
{
	RTW_PRINT_SEL(sel, "\"%c%c\", 0x%02X%s\n"
		, ent->alpha2[0], ent->alpha2[1], ent->chplan
		, COUNTRY_CHPLAN_EN_11AC(ent) ? " ac" : ""
	);
}

void dump_country_chplan_map(void *sel)
{
	const struct country_chplan *ent;
	u8 code[2];

#if RTW_DEF_MODULE_REGULATORY_CERT
	RTW_PRINT_SEL(sel, "RTW_DEF_MODULE_REGULATORY_CERT:0x%x\n", RTW_DEF_MODULE_REGULATORY_CERT);
#endif
#ifdef CONFIG_CUSTOMIZED_COUNTRY_CHPLAN_MAP
	RTW_PRINT_SEL(sel, "CONFIG_CUSTOMIZED_COUNTRY_CHPLAN_MAP\n");
#endif

	for (code[0] = 'A'; code[0] <= 'Z'; code[0]++) {
		for (code[1] = 'A'; code[1] <= 'Z'; code[1]++) {
			ent = rtw_get_chplan_from_country(code);
			if (!ent)
				continue;

			dump_country_chplan(sel, ent);
		}
	}
}

void dump_chplan_id_list(void *sel)
{
	u8 first = 1;
	int i;

	for (i = 0; i < RTW_CHPLAN_MAX; i++) {
		if (!rtw_is_channel_plan_valid(i))
			continue;

		if (first) {
			RTW_PRINT_SEL(sel, "0x%02X ", i);
			first = 0;
		} else
			_RTW_PRINT_SEL(sel, "0x%02X ", i);
	}

	_RTW_PRINT_SEL(sel, "0x7F\n");
}

void dump_chplan_test(void *sel)
{
	int i, j;

	/* check invalid channel */
	for (i = 0; i < RTW_RD_2G_MAX; i++) {
		for (j = 0; j < CH_LIST_LEN(RTW_ChannelPlan2G[i]); j++) {
			if (rtw_ch2freq(CH_LIST_CH(RTW_ChannelPlan2G[i], j)) == 0)
				RTW_PRINT_SEL(sel, "invalid ch:%u at (%d,%d)\n", CH_LIST_CH(RTW_ChannelPlan2G[i], j), i, j);
		}
	}

#ifdef CONFIG_IEEE80211_BAND_5GHZ
	for (i = 0; i < RTW_RD_5G_MAX; i++) {
		for (j = 0; j < CH_LIST_LEN(RTW_ChannelPlan5G[i]); j++) {
			if (rtw_ch2freq(CH_LIST_CH(RTW_ChannelPlan5G[i], j)) == 0)
				RTW_PRINT_SEL(sel, "invalid ch:%u at (%d,%d)\n", CH_LIST_CH(RTW_ChannelPlan5G[i], j), i, j);
		}
	}
#endif
}

void dump_chplan_ver(void *sel)
{
	RTW_PRINT_SEL(sel, "%s-%s\n", RTW_DOMAIN_MAP_VER, RTW_COUNTRY_MAP_VER);
}
