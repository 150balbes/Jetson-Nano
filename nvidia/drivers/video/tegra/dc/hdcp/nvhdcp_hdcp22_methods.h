/*
 * nvhdcp_hdcp22_methods.h: hdcp 2.2 driver definitions.
 *
 * Copyright (c) 2014-2018, NVIDIA CORPORATION, All rights reserved.
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

/* for 0x50 HDCP 2.2 support */
#define HDCP_22_SUPPORTED               (1 << 2)
#define HDCP_22_REPEATER                (0x0100)
#define HDCP_STATUS_READY               (0x200)
#define HDCP_NONCE_SIZE			16
#define HDCP_CMAC_SIZE			16
#define HDCP_22				1
#define HDCP_1x				0
#define TEGRA_NVHDCP_PORT_DP		2
#define TEGRA_NVHDCP_PORT_HDMI		3

#define DISPLAY_TYPE_HDMI		1
#define DISPLAY_TYPE_DP			2

#define HDCP_CMD_GEN_CMAC		0xB
#define HDCP_CMAC_OFFSET		6
#define HDCP_TSEC_ADDR_OFFSET		22
#define RCVR_ID_LIST_SIZE               635
#define TSEC_SRM_REVOCATION_CHECK	(1)
#define PKT_SIZE			256

/* FUSE details for t19x */
#define FUSE_START_SFK                  0x5

int tsec_hdcp_readcaps(struct hdcp_context_t *hdcp_context);
int tsec_hdcp_init(struct hdcp_context_t *hdcp_context);
int tsec_hdcp_create_session(struct hdcp_context_t *hdcp_context,
		int display_type, int sor_num);
int tsec_hdcp_context_creation(struct hdcp_context_t *hdcp_context,
		int display_type, int sor);
int tsec_hdcp_verify_cert(struct hdcp_context_t *hdcp_context);
int tsec_hdcp_generate_ekm(struct hdcp_context_t *hdcp_context);
int tsec_hdcp_verify_hprime(struct hdcp_context_t *hdcp_context);
int tsec_hdcp_encrypt_pairing_info(struct hdcp_context_t *hdcp_context);
int tsec_hdcp_generate_lc_init(struct hdcp_context_t *hdcp_context);
int tsec_hdcp_verify_lprime(struct hdcp_context_t *hdcp_context);
int tsec_hdcp_ske_init(struct hdcp_context_t *hdcp_context);
int tsec_hdcp_verify_vprime(struct hdcp_context_t *hdcp_context,
	unsigned char *cmac, unsigned int tsec_address, unsigned int port);
int tsec_hdcp_exchange_info(struct hdcp_context_t *hdcp_context,
		u32 method_flag,
		u8 *version,
		u16 *caps);
int tsec_hdcp_update_rrx(struct hdcp_context_t *hdcp_context);
int tsec_hdcp_rptr_stream_ready(struct hdcp_context_t *hdcp_context);
int tsec_hdcp_session_ctrl(struct hdcp_context_t *hdcp_context, int flag);
int tsec_hdcp_revocation_check(struct hdcp_context_t *hdcp_context,
		unsigned char *cmac, unsigned int tsec_address,
		unsigned int port, unsigned int hdcp_version);
int tsec_hdcp_rptr_stream_manage(struct hdcp_context_t *hdcp_context);
int tsec_hdcp_generate_nonce(struct hdcp_context_t *hdcp_context,
		unsigned char *nonce);
int tsec_hdcp_srm_read(struct hdcp_context_t *hdcp_context, unsigned int);
int tsec_hdcp1x_verify_vprime(struct hdcp_verify_vprime_param
	verify_vprime_param,
	struct hdcp_context_t *hdcp_context,
	u8 *buf, u8 num_bksv_list, u64 *pkt);
