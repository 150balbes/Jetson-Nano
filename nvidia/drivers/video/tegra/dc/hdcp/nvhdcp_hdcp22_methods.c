/*
 * nvhdcp_hdcp22_methods.c: hdcp 2.2 interface.
 *
 * Copyright (c) 2014-2019, NVIDIA CORPORATION, All rights reserved.
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
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/tsec.h>

#include "tsec/tsec_methods.h"
#include "nvhdcp_hdcp22_methods.h"
#include "tsec_drv.h"

#define hdcp_debug(...)       \
		pr_debug("hdcp: " __VA_ARGS__)
#define hdcp_err(...) \
		pr_err("hdcp: Error: " __VA_ARGS__)

#if defined(CONFIG_ANDROID)
#define HDCP22_SRM_PATH "vendor/etc/hdcpsrm/hdcp2x.srm"
#define HDCP11_SRM_PATH "vendor/etc/hdcpsrm/hdcp1x.srm"
#else
#define HDCP22_SRM_PATH "/etc/hdcpsrm/hdcp2x.srm"
#define HDCP11_SRM_PATH "/etc/hdcpsrm/hdcp1x.srm"
#endif

static u8 g_seq_num_init;

int tsec_hdcp_readcaps(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_read_caps_param read_caps_param;
	memset(&read_caps_param, 0, sizeof(struct hdcp_read_caps_param));
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&read_caps_param,
		sizeof(struct hdcp_read_caps_param));
	tsec_send_method(hdcp_context, HDCP_READ_CAPS, 0);
	memcpy(&read_caps_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_read_caps_param));
	if (read_caps_param.ret_code) {
		hdcp_err("tsec_hdcp_readcaps: failed with error %d\n",
			read_caps_param.ret_code);
	}

	return err;
}

int tsec_hdcp_create_session(struct hdcp_context_t *hdcp_context,
		int display_type, int sor_num)
{
	int err = 0;
	struct hdcp_create_session_param create_session_param;
	g_seq_num_init = 0;

	memset(&create_session_param, 0,
			sizeof(struct hdcp_create_session_param));
	create_session_param.no_of_streams = 1;
	create_session_param.session_type = 0;
	create_session_param.display_type = display_type;
	create_session_param.sor_num = sor_num;
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&create_session_param,
		sizeof(struct hdcp_create_session_param));

	tsec_send_method(hdcp_context, HDCP_CREATE_SESSION, HDCP_MTHD_FLAGS_SB);
	memcpy(&create_session_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_create_session_param));
	if (create_session_param.ret_code) {
		hdcp_err("tsec_hdcp_create_session: failed with error %d\n",
			create_session_param.ret_code);
	}
	hdcp_context->session_id = create_session_param.session_id;
	hdcp_context->msg.rtx = create_session_param.rtx;
	err = create_session_param.ret_code;

	return err;
}

int tsec_hdcp_init(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_init_param init_param;

	memset(&init_param, 0, sizeof(struct hdcp_init_param));

	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	init_param.flags = 0;
	init_param.chip_id = 0;

	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&init_param,
		sizeof(struct hdcp_init_param));

	tsec_send_method(hdcp_context, HDCP_INIT, HDCP_MTHD_FLAGS_SB);

	memcpy(&init_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_init_param));

	if (init_param.ret_code) {
		hdcp_err("tsec_hdcp_init: failed with error %d\n",
			init_param.ret_code);
	}
	err = init_param.ret_code;
	return err;
}

int tsec_hdcp_context_creation(struct hdcp_context_t *hdcp_context,
			int display_type, int sor_instance)
{
	int e = 0;

	e = tsec_hdcp_create_context(hdcp_context);
	if (e) {
		hdcp_err("Error creating hdcp context\n");
		goto exit;
	}
	e = tsec_hdcp_init(hdcp_context);
	if (e) {
		hdcp_err("error in tsec init\n");
		goto exit;
	}
	e =  tsec_hdcp_create_session(hdcp_context, display_type,
				sor_instance);
	if (e)
		hdcp_err("error in session creation\n");
exit:
	return e;
}

/* vprime verification for repeater/srm revocation check for rx */
int tsec_hdcp1x_verify_vprime(struct hdcp_verify_vprime_param
		verify_vprime_param,
		struct hdcp_context_t *hdcp_context,
		u8 *buf, u8 num_bksv_list, u64 *pkt)
{
	unsigned int *tsec_addr;
	int e = 0;

	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	verify_vprime_param.srm_size = tsec_hdcp_srm_read(hdcp_context,
							HDCP_1x);

	if (!verify_vprime_param.srm_size) {
		hdcp_err("Error reading SRM file!\n");
		goto exit;
	}

	memcpy(hdcp_context->cpuvaddr_rcvr_id_list, buf,
			num_bksv_list*5);
	verify_vprime_param.trans_id.session_id = hdcp_context->session_id;
	verify_vprime_param.is_ver_hdcp2x = 0; /* hdcp 1.x */
	verify_vprime_param.depth = 0; /* depth not used */
	verify_vprime_param.device_count = num_bksv_list;
	verify_vprime_param.has_hdcp2_repeater = 0;
	tsec_addr = (unsigned int *)(pkt + HDCP_TSEC_ADDR_OFFSET);
	verify_vprime_param.tsec_gsc_address = *tsec_addr;
	memcpy(verify_vprime_param.srm_cmac,
		(unsigned char *)(pkt + HDCP_CMAC_OFFSET),
		HDCP_CMAC_SIZE);
	verify_vprime_param.has_hdcp1_device = 0;
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&verify_vprime_param,
		sizeof(struct hdcp_verify_vprime_param));
	tsec_send_method(hdcp_context,
	HDCP_VERIFY_VPRIME,
	HDCP_MTHD_FLAGS_SB|HDCP_MTHD_FLAGS_RECV_ID_LIST|HDCP_MTHD_FLAGS_SRM);
	memcpy(&verify_vprime_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_verify_vprime_param));
	if (verify_vprime_param.ret_code) {
		hdcp_err("tsec_hdcp_verify_vprime: failed with error:%x\n",
		verify_vprime_param.ret_code);
	}
	e = verify_vprime_param.ret_code;
exit:
	return e;
}

int tsec_hdcp_verify_cert(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_verify_cert_rx_param verify_cert_rx_param;
	memset(&verify_cert_rx_param, 0,
		sizeof(struct hdcp_verify_cert_rx_param));
	memset(hdcp_context->cpuvaddr_cert, 0, HDCP_CERT_SIZE_ALIGNED);
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	verify_cert_rx_param.session_id = hdcp_context->session_id;
	memcpy((u8 *)hdcp_context->cpuvaddr_cert_aligned,
		hdcp_context->msg.cert_rx,
		HDCP_CERT_SIZE);
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&verify_cert_rx_param,
		sizeof(struct hdcp_verify_cert_rx_param));
	tsec_send_method(hdcp_context,
		HDCP_VERIFY_CERT_RX,
		HDCP_MTHD_FLAGS_SB|HDCP_MTHD_FLAGS_CERT);
	memcpy(&verify_cert_rx_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_verify_cert_rx_param));

	if (verify_cert_rx_param.ret_code) {
		hdcp_err("tsec_hdcp_verify_cert: failed with error %d\n",
			verify_cert_rx_param.ret_code);
	}
	err = verify_cert_rx_param.ret_code;
	return err;

}

int tsec_hdcp_generate_ekm(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_generate_ekm_param generate_ekm_param;
	memset(&generate_ekm_param, 0, sizeof(struct hdcp_generate_ekm_param));
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	generate_ekm_param.session_id = hdcp_context->session_id;
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&generate_ekm_param,
		sizeof(struct hdcp_generate_ekm_param));
	tsec_send_method(hdcp_context, HDCP_GENERATE_EKM, HDCP_MTHD_FLAGS_SB);
	memcpy(&generate_ekm_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_generate_ekm_param));
	if (generate_ekm_param.ret_code) {
		hdcp_err("tsec_hdcp_generate_ekm: failed with error %d\n",
			generate_ekm_param.ret_code);
		goto exit;
	}
	memcpy(hdcp_context->msg.ekm, generate_ekm_param.ekm, HDCP_SIZE_E_KM_8);
exit:
	err = generate_ekm_param.ret_code;
	return err;
}

int tsec_hdcp_verify_hprime(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_verify_hprime_param verify_hprime_param;
	memset(&verify_hprime_param, 0,
			sizeof(struct hdcp_verify_hprime_param));
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	verify_hprime_param.session_id = hdcp_context->session_id;
	memcpy(verify_hprime_param.hprime,
		hdcp_context->msg.hprime,
		HDCP_SIZE_HPRIME_8);
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&verify_hprime_param,
		sizeof(struct hdcp_verify_hprime_param));
	tsec_send_method(hdcp_context, HDCP_VERIFY_HPRIME, HDCP_MTHD_FLAGS_SB);
	memcpy(&verify_hprime_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_verify_hprime_param));
	if (verify_hprime_param.ret_code) {
		hdcp_err("tsec_hdcp_verify_hprime: failed with error %d\n",
		verify_hprime_param.ret_code);
	}
	err = verify_hprime_param.ret_code;
	return err;
}

int tsec_hdcp_encrypt_pairing_info(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_encrypt_pairing_info_param encrypt_pairing_info_param;
	memset(&encrypt_pairing_info_param, 0,
			sizeof(struct hdcp_encrypt_pairing_info_param));
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	encrypt_pairing_info_param.session_id = hdcp_context->session_id;
	memcpy(encrypt_pairing_info_param.ekhkm,
		hdcp_context->msg.ekhkm,
		HDCP_SIZE_EKH_KM_8);
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&encrypt_pairing_info_param,
		sizeof(struct hdcp_encrypt_pairing_info_param));
	tsec_send_method(hdcp_context,
		HDCP_ENCRYPT_PAIRING_INFO,
		HDCP_MTHD_FLAGS_SB);
	memcpy(&encrypt_pairing_info_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_encrypt_pairing_info_param));
	if (encrypt_pairing_info_param.ret_code) {
		hdcp_err("tsec_hdcp_encrypt_pairing_info: failed with error %d\n",
			encrypt_pairing_info_param.ret_code);
	}
	err = encrypt_pairing_info_param.ret_code;
	return err;
}

int tsec_hdcp_generate_lc_init(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_generate_lc_init_param generate_lc_init_param;
	memset(&generate_lc_init_param, 0,
			sizeof(struct hdcp_generate_lc_init_param));
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	generate_lc_init_param.session_id = hdcp_context->session_id;
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&generate_lc_init_param,
		sizeof(struct hdcp_generate_lc_init_param));
	tsec_send_method(hdcp_context,
		HDCP_GENERATE_LC_INIT,
		HDCP_MTHD_FLAGS_SB);
	memcpy(&generate_lc_init_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_generate_lc_init_param));
	if (generate_lc_init_param.ret_code) {
		hdcp_err("tsec_hdcp_generate_lc_init: failed with error %d\n",
			generate_lc_init_param.ret_code);
		goto exit;
	}
	memcpy(&hdcp_context->msg.rn,
		&generate_lc_init_param.rn,
		HDCP_SIZE_RN_8);
exit:
	err = generate_lc_init_param.ret_code;
	return err;
}

int tsec_hdcp_verify_lprime(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_verify_lprime_param verify_lprime_param;
	memset(&verify_lprime_param, 0,
			sizeof(struct hdcp_verify_lprime_param));
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	verify_lprime_param.session_id = hdcp_context->session_id;
	memcpy(verify_lprime_param.lprime,
		hdcp_context->msg.lprime,
		HDCP_SIZE_LPRIME_8);
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&verify_lprime_param,
		sizeof(struct hdcp_verify_lprime_param));
	tsec_send_method(hdcp_context, HDCP_VERIFY_LPRIME, HDCP_MTHD_FLAGS_SB);
	memcpy(&verify_lprime_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_verify_lprime_param));
	if (verify_lprime_param.ret_code) {
		hdcp_err("tsec_hdcp_verify_lprime: failed with error %d\n",
			verify_lprime_param.ret_code);
	}
	err = verify_lprime_param.ret_code;
	return err;
}

int tsec_hdcp_ske_init(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_generate_ske_init_param generate_ske_init_param;
	memset(&generate_ske_init_param, 0,
			sizeof(struct hdcp_generate_ske_init_param));
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	generate_ske_init_param.session_id = hdcp_context->session_id;
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&generate_ske_init_param,
		sizeof(struct hdcp_verify_lprime_param));
	tsec_send_method(hdcp_context,
		HDCP_GENERATE_SKE_INIT,
		HDCP_MTHD_FLAGS_SB);
	memcpy(&generate_ske_init_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_generate_ske_init_param));
	if (generate_ske_init_param.ret_code) {
		hdcp_err("tsec_hdcp_ske_init: failed with error %d\n",
			generate_ske_init_param.ret_code);
		goto exit;
	}
	memcpy(hdcp_context->msg.eks,
		(u8 *)generate_ske_init_param.eks,
		HDCP_SIZE_E_KS_8);
	memcpy(&hdcp_context->msg.riv,
		&generate_ske_init_param.riv,
		HDCP_SIZE_RIV_8);
exit:
	err = generate_ske_init_param.ret_code;
	return err;
}

int tsec_hdcp_session_ctrl(struct hdcp_context_t *hdcp_context, int flag)
{
	int err = 0;
	struct hdcp_session_ctrl_param session_ctrl_param;
	memset(&session_ctrl_param, 0, sizeof(struct hdcp_session_ctrl_param));
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	session_ctrl_param.session_id = hdcp_context->session_id;
	session_ctrl_param.ctrl_flag = flag;
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&session_ctrl_param,
		sizeof(struct hdcp_session_ctrl_param));
	tsec_send_method(hdcp_context,
		HDCP_SESSION_CTRL,
		HDCP_MTHD_FLAGS_SB);
	memcpy(&session_ctrl_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_session_ctrl_param));
	if (session_ctrl_param.ret_code) {
		hdcp_err("tsec_hdcp_session_ctrl: failed with error %d\n",
			session_ctrl_param.ret_code);
		goto exit;
	}
exit:
	err = session_ctrl_param.ret_code;
	return err;
}

int tsec_hdcp_generate_nonce(struct hdcp_context_t *hdcp_context,
				unsigned char *nonce)
{
	int err = 0;
	struct hdcp_get_current_nonce_param nonce_param;

	if (!hdcp_context || !nonce) {
		hdcp_err("tsec_hdcp_generate_nonce: null params sent!");
		return -EINVAL;
	}
	memset(&nonce_param, 0,
			sizeof(struct hdcp_get_current_nonce_param));
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	nonce_param.session_id = hdcp_context->session_id;
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&nonce_param,
		sizeof(struct hdcp_get_current_nonce_param));
	tsec_send_method(hdcp_context,
		HDCP_GET_CURRENT_NONCE,
		HDCP_MTHD_FLAGS_SB);
	memcpy(&nonce_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_get_current_nonce_param));
	if (nonce_param.ret_code) {
		hdcp_err("tsec_hdcp_get_current_nonce: failed with error %d\n",
			nonce_param.ret_code);
		goto exit;
	}
	memcpy(nonce, (u8 *)nonce_param.nonce, HDCP_NONCE_SIZE);
exit:
	err = nonce_param.ret_code;
	return err;
}

int tsec_hdcp_srm_read(struct hdcp_context_t *hdcp_context,
			uint32_t hdcp_version)
{
	struct file *fp = NULL;
	unsigned int size = 0;
	mm_segment_t seg;

	if (!hdcp_context) {
		hdcp_err("tsec_hdcp_srm_read: null params sent!");
		return -EINVAL;
	}

	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);

	if (hdcp_version == HDCP_22)
		fp = filp_open(HDCP22_SRM_PATH, O_RDONLY, 0);
	else if (hdcp_version == HDCP_1x)
		fp = filp_open(HDCP11_SRM_PATH, O_RDONLY, 0);
	else {
		hdcp_err("Invalid HDCP version sent!\n");
		return -EINVAL;
	}

	if (IS_ERR(fp) || !fp) {
		hdcp_err("Opening SRM file failed!\n");
		return -ENOENT;
	}
	seg = get_fs();
	set_fs(get_ds());
	/* copy SRM to buffer */
	vfs_read(fp, (u8 *)hdcp_context->cpuvaddr_srm,
		HDCP_SRM_SIZE, &fp->f_pos);
	set_fs(seg);
	size = fp->f_pos;
	filp_close(fp, NULL);
	return size;
}

int tsec_hdcp_revocation_check(struct hdcp_context_t *hdcp_context,
			unsigned char *cmac, unsigned int tsec_address,
			unsigned int port, unsigned int hdcp_version)
{
	int err = 0;
	struct hdcp_revocation_check_param revocation_check_param;

	if (!hdcp_context || !cmac || !tsec_address) {
		hdcp_err("tsec_hdcp_revocation_check: null params sent!");
		return -EINVAL;
	}
	memset(&revocation_check_param, 0,
			sizeof(struct hdcp_revocation_check_param));
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	revocation_check_param.srm_size = tsec_hdcp_srm_read(hdcp_context,
							hdcp_version);
	revocation_check_param.trans_id.session_id = hdcp_context->session_id;
	revocation_check_param.is_ver_hdcp2x = hdcp_version;
	revocation_check_param.tsec_gsc_address = tsec_address;
	revocation_check_param.port = port;

	/* send the cmac generated from secure service */
	memcpy(revocation_check_param.srm_cmac, cmac, HDCP_CMAC_SIZE);
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&revocation_check_param,
		sizeof(struct hdcp_revocation_check_param));
	tsec_send_method(hdcp_context,
		HDCP_REVOCATION_CHECK,
		HDCP_MTHD_FLAGS_SB | HDCP_MTHD_FLAGS_SRM);
	memcpy(&revocation_check_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_revocation_check_param));
	if (revocation_check_param.ret_code) {
		hdcp_err("tsec_hdcp_revocation_check: failed with error %d\n",
			revocation_check_param.ret_code);
		goto exit;
	}
exit:
	err = revocation_check_param.ret_code;
	return err;
}

int tsec_hdcp_verify_vprime(struct hdcp_context_t *hdcp_context,
			unsigned char *cmac, unsigned int tsec_addr,
			unsigned int port)
{
	int err = 0;
	u16 rxinfo;
	u8 seq_num_start[HDCP_SIZE_SEQ_NUM_V_8] = {0x00, 0x00, 0x00};
	struct hdcp_verify_vprime_param verify_vprime_param;
	if (!cmac || !hdcp_context || !tsec_addr) {
		hdcp_err("tsec_hdcp_verify_vprime: null params sent!");
		return -EINVAL;
	}
	memset(&verify_vprime_param, 0,
			sizeof(struct hdcp_verify_vprime_param));
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	/* SRM is already copied into the buffer during revocation check */
	/* Copy receiver id list into buf */
	memcpy(hdcp_context->cpuvaddr_rcvr_id_list,
		hdcp_context->msg.rcvr_id_list,
		HDCP_RCVR_ID_LIST_SIZE);
	memset(&verify_vprime_param, 0x0,
		sizeof(struct hdcp_verify_vprime_param));
	memcpy(verify_vprime_param.vprime,
		hdcp_context->msg.vprime,
		HDCP_SIZE_VPRIME_2X_8/2);
	verify_vprime_param.tsec_gsc_address = tsec_addr;
	memcpy(verify_vprime_param.srm_cmac, cmac, HDCP_CMAC_SIZE);
	verify_vprime_param.rxinfo = hdcp_context->msg.rxinfo;

	verify_vprime_param.trans_id.session_id = hdcp_context->session_id;
	/* Get the device count and depth from rx */
	rxinfo = cpu_to_be16(hdcp_context->msg.rxinfo);
	verify_vprime_param.device_count = (rxinfo & 0x01F0)>>4;
	verify_vprime_param.depth = (rxinfo & 0x0E00)>>9;
	verify_vprime_param.has_hdcp2_repeater = (rxinfo & 0x0002)>>1;
	verify_vprime_param.has_hdcp1_device = (rxinfo & 0x0001);
	verify_vprime_param.bstatus = 0;
	verify_vprime_param.is_ver_hdcp2x = 1;
	verify_vprime_param.port = port;

	if (!g_seq_num_init) {
		if (memcmp(hdcp_context->msg.seq_num,
			seq_num_start,
			HDCP_SIZE_SEQ_NUM_V_8)) {
			hdcp_err("tsec_hdcp_verify_vprime: Invalid seq_num_V\n");
			return -EINVAL;
		}
		g_seq_num_init = 1;
	}

	memcpy(verify_vprime_param.seqnum,
		hdcp_context->msg.seq_num,
		HDCP_SIZE_SEQ_NUM_V_8);
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&verify_vprime_param,
		sizeof(struct hdcp_verify_vprime_param));
	tsec_send_method(hdcp_context,
	HDCP_VERIFY_VPRIME,
	HDCP_MTHD_FLAGS_SB|HDCP_MTHD_FLAGS_RECV_ID_LIST|HDCP_MTHD_FLAGS_SRM);
	memcpy(&verify_vprime_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_verify_vprime_param));
	if (verify_vprime_param.ret_code) {
		hdcp_err("tsec_hdcp_verify_vprime: failed with error %x\n",
			verify_vprime_param.ret_code);
	}
	err = verify_vprime_param.ret_code;
	if (!err)
		memcpy(hdcp_context->msg.v,
			verify_vprime_param.v128l,
			HDCP_SIZE_VPRIME_2X_8/2);
	return err;
}

int tsec_hdcp_exchange_info(struct hdcp_context_t *hdcp_context,
				u32 method_flag,
				u8 *version,
				u16 *caps)
{
	int err = 0;
	struct hdcp_exchange_info_param exchange_info;
	memset(&exchange_info, 0, sizeof(struct hdcp_exchange_info_param));
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	exchange_info.session_id = hdcp_context->session_id;
	exchange_info.method_flag = method_flag;
	if (method_flag == HDCP_EXCHANGE_INFO_SET_RCVR_INFO) {
		exchange_info.info.set_rx_info.version = *version;
		exchange_info.info.set_rx_info.rcvr_caps_mask = *caps;
	} else if (method_flag == HDCP_EXCHANGE_INFO_SET_TMTR_INFO) {
		exchange_info.info.set_tx_info.version = *version;
		exchange_info.info.set_tx_info.tmtr_caps_mask = *caps;
	}
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&exchange_info,
		sizeof(struct hdcp_exchange_info_param));
	tsec_send_method(hdcp_context, HDCP_EXCHANGE_INFO, HDCP_MTHD_FLAGS_SB);
	memcpy(&exchange_info,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_exchange_info_param));
	if (exchange_info.ret_code) {
		hdcp_err("tsec_hdcp_exchange_info: failed with error %d\n",
			exchange_info.ret_code);
	}
	err = exchange_info.ret_code;
	if (!err) {
		if (method_flag == HDCP_EXCHANGE_INFO_GET_RCVR_INFO) {
			*version = exchange_info.info.get_rx_info.version;
			*caps = exchange_info.info.get_rx_info.rcvr_caps_mask;
		} else if (method_flag == HDCP_EXCHANGE_INFO_GET_TMTR_INFO) {
			*version = exchange_info.info.get_tx_info.version;
			*caps = exchange_info.info.get_tx_info.tmtr_caps_mask;
		}
	}
	return err;
}

int tsec_hdcp_update_rrx(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_update_session_param update_session_param;
	memset(&update_session_param, 0,
			sizeof(struct hdcp_update_session_param));
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	update_session_param.session_id = hdcp_context->session_id;
	update_session_param.updmask   =
		(1 << HDCP_UPDATE_SESSION_MASK_RRX_PRESENT);
	memcpy(&update_session_param.rrx,
		&hdcp_context->msg.rrx,
		HDCP_SIZE_RRX_8);
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&update_session_param,
		sizeof(struct hdcp_update_session_param));
	tsec_send_method(hdcp_context,
		HDCP_UPDATE_SESSION,
		HDCP_MTHD_FLAGS_SB);
	memcpy(&update_session_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_update_session_param));
	if (update_session_param.ret_code) {
		hdcp_err("tsec_hdcp_update_rrx: failed with error %d\n",
			update_session_param.ret_code);
	}
	err = update_session_param.ret_code;
	return err;
}

int tsec_hdcp_rptr_stream_manage(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_stream_manage_param stream_manage_param;

	memset(&stream_manage_param, 0, sizeof(stream_manage_param));
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	stream_manage_param.session_id = hdcp_context->session_id;
	stream_manage_param.manage_flag = HDCP_STREAM_MANAGE_FLAG_MANAGE;
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&stream_manage_param,
		sizeof(struct hdcp_stream_manage_param));
	tsec_send_method(hdcp_context, HDCP_STREAM_MANAGE,
		HDCP_MTHD_FLAGS_SB);
	memcpy(&stream_manage_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_stream_manage_param));
	/* Need to fix the ucode for this */
	if (stream_manage_param.ret_code) {
		hdcp_err("tsec_hdcp_rptr_stream_manage:failed with error %d\n",
		stream_manage_param.ret_code);
	}
	memcpy(hdcp_context->msg.seq_num_m, stream_manage_param.seqnum_m,
		HDCP_SIZE_SEQ_NUM_M_8);
	err = stream_manage_param.ret_code;
	return err;
}

int tsec_hdcp_rptr_stream_ready(struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	struct hdcp_stream_manage_param    stream_manage_param;
	memset(&stream_manage_param, 0,
			sizeof(struct hdcp_stream_manage_param));
	memset(hdcp_context->cpuvaddr_mthd_buf_aligned, 0,
		HDCP_MTHD_RPLY_BUF_SIZE);
	stream_manage_param.session_id = hdcp_context->session_id;
	stream_manage_param.manage_flag = HDCP_STREAM_MANAGE_FLAG_READY;
	stream_manage_param.streamid_type = 0x0000;
	stream_manage_param.content_id[0][0] = 0x0000;
	stream_manage_param.str_type[0][0] = 0x0;
	memcpy(stream_manage_param.mprime,
		hdcp_context->msg.mprime,
		HDCP_SIZE_MPRIME_8);
	memcpy(hdcp_context->cpuvaddr_mthd_buf_aligned,
		&stream_manage_param,
		sizeof(struct hdcp_stream_manage_param));
	tsec_send_method(hdcp_context, HDCP_STREAM_MANAGE, HDCP_MTHD_FLAGS_SB);
	memcpy(&stream_manage_param,
		hdcp_context->cpuvaddr_mthd_buf_aligned,
		sizeof(struct hdcp_stream_manage_param));
	if (stream_manage_param.ret_code) {
		hdcp_err("tsec_hdcp_rptr_auth_stream_ready: failed with error %d\n",
		stream_manage_param.ret_code);
	}
	err = stream_manage_param.ret_code;
	return err;
}
