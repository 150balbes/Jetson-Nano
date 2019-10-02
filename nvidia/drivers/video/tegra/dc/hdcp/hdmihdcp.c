/*
 * hdmihdcp.c: hdmi hdcp functions.
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
#include <asm/atomic.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/tsec.h>

#include <soc/tegra/kfuse.h>
#include <soc/tegra/fuse.h>
#include <linux/trusty/trusty_ipc.h>
#include <linux/ote_protocol.h>

#include <video/nvhdcp.h>

#include "dc.h"
#include "dc_reg.h"
#include "dc_priv.h"
#include "edid.h"
#include "hdmi2.0.h"
#include "sor.h"
#include "sor_regs.h"
#include "hdmihdcp.h"
#include "hdmi_reg.h"
#include "host1x/host1x01_hardware.h"
#include "tsec/tsec.h"
#include "class_ids.h"
#include "tsec_drv.h"
#include "tsec/tsec_methods.h"
#include "nvhdcp_hdcp22_methods.h"

static DECLARE_WAIT_QUEUE_HEAD(wq_worker);

/* for 0x40 Bcaps */
#define BCAPS_REPEATER (1 << 6)
#define BCAPS_READY (1 << 5)
#define BCAPS_11 (1 << 1) /* used for both Bcaps and Ainfo */

/* for 0x41 Bstatus */
#define BSTATUS_MAX_DEVS_EXCEEDED	(1 << 7)
#define BSTATUS_MAX_CASCADE_EXCEEDED	(1 << 11)

/* for HDCP2.2 */
#define HDCP_READY			1
#define HDCP_REAUTH			2
#define HDCP_REAUTH_MASK		(1 << 11)
#define HDCP_MSG_SIZE_MASK		0x03FF
#define HDCP22_PROTOCOL			1
#define HDCP1X_PROTOCOL			0
#define HDCP_INFINITE_RETRIES		-1
#define HDCP_DEBUG                      0
#define SEQ_NUM_M_MAX_RETRIES		1
#define RX_CERT_POLL_TIME		1000

#define HDCP_FALLBACK_1X                0xdeadbeef
#define HDCP_NON_22_RX                  0x0300

#define HDCP_EESS_ENABLE		(0x1)
#define HDCP22_EESS_START		(0x201)
#define HDCP22_EESS_END			(0x211)
#define HDCP1X_EESS_START		(0x200)
#define HDCP1X_EESS_END			(0x210)
#define HDMI_VSYNC_WINDOW		(0xc2)

#define HDCP_TA_CMD_CTRL		0
#define HDCP_TA_CMD_AKSV		1
#define HDCP_TA_CMD_ENC			2
#define HDCP_TA_CMD_REP			3
#define HDCP_TA_CMD_BKSV		4

#define PKT_SIZE			256

#define HDCP_TA_CTRL_ENABLE		1
#define HDCP_TA_CTRL_DISABLE		0

#define HDCP_CMD_OFFSET			1
#define HDCP_CMD_BYTE_OFFSET		8
#define HDCP_AUTH_CMD			0x5
#define MAX_CERT_RETRY			10

#ifdef VERBOSE_DEBUG
#define nvhdcp_vdbg(...)	\
		pr_debug("nvhdcp: " __VA_ARGS__)
#else
#define nvhdcp_vdbg(...)		\
({						\
	if (0)					\
		pr_debug("nvhdcp: " __VA_ARGS__); \
	0;					\
})
#endif
#define nvhdcp_debug(...)	\
		pr_debug("nvhdcp: " __VA_ARGS__)
#define nvhdcp_err(...)	\
		pr_err("nvhdcp: Error: " __VA_ARGS__)
#define nvhdcp_info(...)	\
		pr_info("nvhdcp: " __VA_ARGS__)
#define HDCP_PORT_NAME	"com.nvidia.tos.13f616f9-8572-4a6f-a1f104aa9b05f9ff"
#define HDCP_SERVICE_UUID		{0x13F616F9, 0x4A6F8572,\
				 0xAA04F1A1, 0xFFF9059B}
static u8 g_seq_num_m_retries;
static u8 g_fallback;

#ifdef CONFIG_TRUSTED_LITTLE_KERNEL
static uint32_t hdcp_uuid[4] = HDCP_SERVICE_UUID;
static uint32_t session_id;
#endif

static struct tegra_dc *tegra_dc_hdmi_get_dc(struct tegra_hdmi *hdmi)
{
	return hdmi ? hdmi->dc : NULL;
}

static inline bool nvhdcp_is_plugged(struct tegra_nvhdcp *nvhdcp)
{
	rmb();
	return nvhdcp->plugged;
}

static inline bool nvhdcp_set_plugged(struct tegra_nvhdcp *nvhdcp, bool plugged)
{
	nvhdcp->plugged = plugged;
	wmb();
	return plugged;
}

static int nvhdcp_i2c_read(struct tegra_nvhdcp *nvhdcp, u8 reg,
					size_t len, void *data)
{
	int status;
	int retries = 11;
	struct i2c_msg msg[] = {
		{
			.addr = 0x74 >> 1, /* primary link */
			.flags = 0,
			.len = 1,
			.buf = &reg,
		},
		{
			.addr = 0x74 >> 1, /* primary link */
			.flags = I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};
	struct tegra_dc *dc = tegra_dc_hdmi_get_dc(nvhdcp->hdmi);

	if (dc->vedid)
		goto skip_hdcp_i2c;

	tegra_dc_ddc_enable(dc, true);
	do {
		mutex_lock(&nvhdcp->lock);
		if (!nvhdcp_is_plugged(nvhdcp)) {
			nvhdcp_err("disconnect during i2c xfer\n");
			mutex_unlock(&nvhdcp->lock);
			tegra_dc_ddc_enable(dc, false);
			return -EIO;
		}
		mutex_unlock(&nvhdcp->lock);
		status = i2c_transfer(nvhdcp->client->adapter,
			msg, ARRAY_SIZE(msg));
		if ((status < 0) && (retries > 1))
			msleep(250);
	} while ((status < 0) && retries--);
	tegra_dc_ddc_enable(dc, false);

	if (status < 0) {
		nvhdcp_err("i2c xfer error %d\n", status);
		return status;
	}

	return 0;

skip_hdcp_i2c:
	nvhdcp_err("vedid active\n");
	return -EIO;
}

static int nvhdcp_i2c_write(struct tegra_nvhdcp *nvhdcp, u8 reg,
					size_t len, const void *data)
{
	int status;
	u8 buf[len + 1];
	struct i2c_msg msg[] = {
		{
			.addr = 0x74 >> 1, /* primary link */
			.flags = 0,
			.len = len + 1,
			.buf = buf,
		},
	};
	int retries = 15;
	struct tegra_dc *dc = tegra_dc_hdmi_get_dc(nvhdcp->hdmi);

	if (dc->vedid)
		goto skip_hdcp_i2c;

	buf[0] = reg;
	memcpy(buf + 1, data, len);

	tegra_dc_ddc_enable(dc, true);
	do {
		mutex_lock(&nvhdcp->lock);
		if (!nvhdcp_is_plugged(nvhdcp)) {
			nvhdcp_err("disconnect during i2c xfer\n");
			mutex_unlock(&nvhdcp->lock);
			tegra_dc_ddc_enable(dc, false);
			return -EIO;
		}
		mutex_unlock(&nvhdcp->lock);
		status = i2c_transfer(nvhdcp->client->adapter,
			msg, ARRAY_SIZE(msg));
		if ((status < 0) && (retries > 1))
			msleep(250);
	} while ((status < 0) && retries--);
	tegra_dc_ddc_enable(dc, false);

	if (status < 0) {
		nvhdcp_err("i2c xfer error %d\n", status);
		return status;
	}

	return 0;

skip_hdcp_i2c:
	nvhdcp_err("vedid active\n");
	return -EIO;
}

static inline int nvhdcp_i2c_read8(struct tegra_nvhdcp *nvhdcp, u8 reg, u8 *val)
{
	return nvhdcp_i2c_read(nvhdcp, reg, 1, val);
}

static inline int nvhdcp_i2c_write8(struct tegra_nvhdcp *nvhdcp, u8 reg, u8 val)
{
	return nvhdcp_i2c_write(nvhdcp, reg, 1, &val);
}

static inline int nvhdcp_i2c_read16(struct tegra_nvhdcp *nvhdcp,
					u8 reg, u16 *val)
{
	u8 buf[2];
	int e;

	e = nvhdcp_i2c_read(nvhdcp, reg, sizeof(buf), buf);
	if (e)
		return e;

	if (val)
		*val = buf[0] | (u16)buf[1] << 8;

	return 0;
}

static int nvhdcp_i2c_read40(struct tegra_nvhdcp *nvhdcp, u8 reg, u64 *val)
{
	u8 buf[5];
	int e, i;
	u64 n;

	e = nvhdcp_i2c_read(nvhdcp, reg, sizeof(buf), buf);
	if (e)
		return e;

	for (i = 0, n = 0; i < 5; i++) {
		n <<= 8;
		n |= buf[4 - i];
	}

	if (val)
		*val = n;

	return 0;
}

static int nvhdcp_i2c_write40(struct tegra_nvhdcp *nvhdcp, u8 reg, u64 val)
{
	char buf[5];
	int i;
	for (i = 0; i < 5; i++) {
		buf[i] = val;
		val >>= 8;
	}
	return nvhdcp_i2c_write(nvhdcp, reg, sizeof(buf), buf);
}

static int nvhdcp_i2c_write64(struct tegra_nvhdcp *nvhdcp, u8 reg, u64 val)
{
	char buf[8];
	int i;
	for (i = 0; i < 8; i++) {
		buf[i] = val;
		val >>= 8;
	}
	return nvhdcp_i2c_write(nvhdcp, reg, sizeof(buf), buf);
}

/* 64-bit link encryption session random number */
static inline u64 __maybe_unused get_an(struct tegra_hdmi *hdmi)
{
	u64 r;
	r = (u64)tegra_sor_readl_ext(hdmi->sor, NV_SOR_TMDS_HDCP_AN_MSB) << 32;
	r |= tegra_sor_readl_ext(hdmi->sor, NV_SOR_TMDS_HDCP_AN_LSB);
	return r;
}

/* 64-bit upstream exchange random number */
static inline void __maybe_unused set_cn(struct tegra_hdmi *hdmi,
					 u64 c_n)
{
	tegra_sor_writel_ext(hdmi->sor, NV_SOR_TMDS_HDCP_CN_LSB, (u32)c_n);
	tegra_sor_writel_ext(hdmi->sor, NV_SOR_TMDS_HDCP_CN_MSB, c_n >> 32);
}


/* 40-bit transmitter's key selection vector */
static inline u64 __maybe_unused get_aksv(struct tegra_hdmi *hdmi)
{
	u64 r;
	r = (u64)tegra_sor_readl_ext(hdmi->sor,
			NV_SOR_TMDS_HDCP_AKSV_MSB) << 32;
	r |= tegra_sor_readl_ext(hdmi->sor, NV_SOR_TMDS_HDCP_AKSV_LSB);
	return r;
}

/* 40-bit receiver's key selection vector */
static inline void __maybe_unused set_bksv(struct tegra_hdmi *hdmi,
						u64 b_ksv, bool repeater)
{
	if (repeater)
		b_ksv |= (u64)REPEATER << 32;
	tegra_sor_writel_ext(hdmi->sor, NV_SOR_TMDS_HDCP_BKSV_LSB, (u32)b_ksv);
	tegra_sor_writel_ext(hdmi->sor, NV_SOR_TMDS_HDCP_BKSV_MSB, b_ksv >> 32);
}


/* 40-bit software's key selection vector */
static inline void set_cksv(struct tegra_hdmi *hdmi, u64 c_ksv)
{
	tegra_sor_writel_ext(hdmi->sor, NV_SOR_TMDS_HDCP_CKSV_LSB, (u32)c_ksv);
	tegra_sor_writel_ext(hdmi->sor, NV_SOR_TMDS_HDCP_CKSV_MSB, c_ksv >> 32);
}

/* 40-bit connection state */
static inline u64 get_cs(struct tegra_hdmi *hdmi)
{
	u64 r;
	r = (u64)tegra_sor_readl_ext(hdmi->sor, NV_SOR_TMDS_HDCP_CS_MSB) << 32;
	r |= tegra_sor_readl_ext(hdmi->sor, NV_SOR_TMDS_HDCP_CS_LSB);
	return r;
}

/* 40-bit upstream key selection vector */
static inline u64 get_dksv(struct tegra_hdmi *hdmi)
{
	u64 r;
	r = (u64)tegra_sor_readl_ext(hdmi->sor,
			NV_SOR_TMDS_HDCP_DKSV_MSB) << 32;
	r |= tegra_sor_readl_ext(hdmi->sor, NV_SOR_TMDS_HDCP_DKSV_LSB);
	return r;
}

/* 64-bit encrypted M0 value */
static inline u64 get_mprime(struct tegra_hdmi *hdmi)
{
	u64 r;
	r = (u64)tegra_sor_readl_ext(hdmi->sor,
			NV_SOR_TMDS_HDCP_MPRIME_MSB) << 32;
	r |= tegra_sor_readl_ext(hdmi->sor, NV_SOR_TMDS_HDCP_MPRIME_LSB);
	return r;
}

static inline u16 get_transmitter_ri(struct tegra_hdmi *hdmi)
{
	return tegra_sor_readl_ext(hdmi->sor, NV_SOR_TMDS_HDCP_RI);
}

static inline int get_receiver_ri(struct tegra_nvhdcp *nvhdcp, u16 *r)
{
	return nvhdcp_i2c_read16(nvhdcp, 0x8, r); /* long read */
}

static int get_bcaps(struct tegra_nvhdcp *nvhdcp, u8 *b_caps)
{
	return nvhdcp_i2c_read8(nvhdcp, 0x40, b_caps);
}

static int get_ksvfifo(struct tegra_nvhdcp *nvhdcp,
					unsigned num_bksv_list, u64 *ksv_list)
{
	u8 *buf, *p;
	int e;
	unsigned i;
	size_t buf_len = num_bksv_list * 5;

	if (!ksv_list || num_bksv_list > TEGRA_NVHDCP_MAX_DEVS)
		return -EINVAL;

	if (num_bksv_list == 0)
		return 0;

	buf = kmalloc(buf_len, GFP_KERNEL);
	if (IS_ERR_OR_NULL(buf))
		return -ENOMEM;

	e = nvhdcp_i2c_read(nvhdcp, 0x43, buf_len, buf);
	if (e) {
		kfree(buf);
		return e;
	}

	/* load 40-bit keys from repeater into array of u64 */
	p = buf;
	for (i = 0; i < num_bksv_list; i++) {
		ksv_list[i] = p[0] | ((u64)p[1] << 8) | ((u64)p[2] << 16)
				| ((u64)p[3] << 24) | ((u64)p[4] << 32);
		p += 5;
	}

	kfree(buf);
	return 0;
}

/* get V' 160-bit SHA-1 hash from repeater */
static int get_vprime(struct tegra_nvhdcp *nvhdcp, u8 *v_prime)
{
	int e, i;

	for (i = 0; i < 20; i += 4) {
		e = nvhdcp_i2c_read(nvhdcp, 0x20 + i, 4, v_prime + i);
		if (e)
			return e;
	}
	return 0;
}

/* set or clear RUN_YES */
static void hdcp_ctrl_run(struct tegra_hdmi *hdmi, bool v)
{
	u32 ctrl;

	if (v) {
		ctrl = tegra_sor_readl_ext(hdmi->sor, NV_SOR_TMDS_HDCP_CTRL);
		ctrl |= HDCP_RUN_YES;
	} else {
		ctrl = 0;
	}

	tegra_sor_writel_ext(hdmi->sor, NV_SOR_TMDS_HDCP_CTRL, ctrl);
}

/* wait for any bits in mask to be set in NV_SOR_TMDS_HDCP_CTRL
 * sleeps up to 120mS */
static int wait_hdcp_ctrl(struct tegra_hdmi *hdmi, u32 mask, u32 *v)
{
	int retries = 13;
	u32 ctrl;

	do {
		ctrl = tegra_sor_readl_ext(hdmi->sor, NV_SOR_TMDS_HDCP_CTRL);
		if ((ctrl & mask)) {
			if (v)
				*v = ctrl;
			break;
		}
		if (retries > 1)
			usleep_range(10, 15);
	} while (--retries);
	if (!retries) {
		nvhdcp_err("ctrl read timeout (mask=0x%x)\n", mask);
		return -EIO;
	}
	return 0;
}

/* wait for bits in mask to be set to value in NV_SOR_KEY_CTRL
 * waits up to 100mS */
static int wait_key_ctrl(struct tegra_hdmi *hdmi, u32 mask, u32 value)
{
	int retries = 101;
	u32 ctrl;

	do {
		usleep_range(1, 2);
		ctrl = tegra_sor_readl_ext(hdmi->sor, NV_SOR_KEY_CTRL);
		if (((ctrl ^ value) & mask) == 0)
			break;
	} while (--retries);
	if (!retries) {
		nvhdcp_err("key ctrl read timeout (mask=0x%x)\n", mask);
		return -EIO;
	}
	return 0;
}

/* check that key selection vector is well formed.
 * NOTE: this function assumes KSV has already been checked against
 * revocation list.
 */
static int verify_ksv(u64 k)
{
	unsigned i;

	/* count set bits, must be exactly 20 set to be valid */
	for (i = 0; k; i++)
		k ^= k & -k;

	return  (i != 20) ? -EINVAL : 0;
}

static int get_nvhdcp_state(struct tegra_nvhdcp *nvhdcp,
			struct tegra_nvhdcp_packet *pkt)
{
	int	i;

	mutex_lock(&nvhdcp->lock);
	if (nvhdcp->state != STATE_LINK_VERIFY) {
		memset(pkt, 0, sizeof(*pkt));
		pkt->packet_results = TEGRA_NVHDCP_RESULT_LINK_FAILED;
	} else {
		pkt->num_bksv_list = nvhdcp->num_bksv_list;
		for (i = 0; i < pkt->num_bksv_list; i++)
			pkt->bksv_list[i] = nvhdcp->bksv_list[i];
		pkt->b_status = nvhdcp->b_status;
		pkt->b_ksv = nvhdcp->b_ksv;
		memcpy(pkt->v_prime, nvhdcp->v_prime, sizeof(nvhdcp->v_prime));
		pkt->packet_results = TEGRA_NVHDCP_RESULT_SUCCESS;
		pkt->hdcp22 = nvhdcp->hdcp22;
		pkt->port = TEGRA_NVHDCP_PORT_HDMI;
	}
	mutex_unlock(&nvhdcp->lock);
	return 0;
}

/* get Status and Kprime signature - READ_S on TMDS0_LINK0 only */
static int get_s_prime(struct tegra_nvhdcp *nvhdcp,
			struct tegra_nvhdcp_packet *pkt)
{
	struct tegra_hdmi *hdmi = nvhdcp->hdmi;
	u32 sp_msb, sp_lsb1, sp_lsb2;
	int e;

	/* if connection isn't authenticated ... */
	mutex_lock(&nvhdcp->lock);
	if (nvhdcp->state != STATE_LINK_VERIFY) {
		memset(pkt, 0, sizeof(*pkt));
		pkt->packet_results = TEGRA_NVHDCP_RESULT_LINK_FAILED;
		e = 0;
		goto err;
	}

	pkt->packet_results = TEGRA_NVHDCP_RESULT_UNSUCCESSFUL;

	/* we will be taking c_n, c_ksv as input */
	if (!(pkt->value_flags & TEGRA_NVHDCP_FLAG_CN)
			|| !(pkt->value_flags & TEGRA_NVHDCP_FLAG_CKSV)) {
		nvhdcp_err("missing value_flags (0x%x)\n", pkt->value_flags);
		e = -EINVAL;
		goto err;
	}

	pkt->value_flags = 0;

	pkt->a_ksv = nvhdcp->a_ksv;
	pkt->a_n = nvhdcp->a_n;
	pkt->value_flags = TEGRA_NVHDCP_FLAG_AKSV | TEGRA_NVHDCP_FLAG_AN;

	nvhdcp_vdbg("%s():cn %llx cksv %llx\n", __func__, pkt->c_n, pkt->c_ksv);

	set_cn(hdmi, pkt->c_n);

	tegra_sor_writel_ext(hdmi->sor,	NV_SOR_TMDS_HDCP_CMODE,
					TMDS0_LINK0 | READ_S);

	set_cksv(hdmi, pkt->c_ksv);

	e = wait_hdcp_ctrl(hdmi, SPRIME_VALID, NULL);
	if (e) {
		nvhdcp_err("Sprime read timeout\n");
		pkt->packet_results = TEGRA_NVHDCP_RESULT_UNSUCCESSFUL;
		e = -EIO;
		goto err;
	}

	msleep(50);

	/* read 56-bit Sprime plus 16 status bits */
	sp_msb = tegra_sor_readl_ext(hdmi->sor, NV_SOR_TMDS_HDCP_SPRIME_MSB);
	sp_lsb1 = tegra_sor_readl_ext(hdmi->sor, NV_SOR_TMDS_HDCP_SPRIME_LSB1);
	sp_lsb2 = tegra_sor_readl_ext(hdmi->sor, NV_SOR_TMDS_HDCP_SPRIME_LSB2);

	/* top 8 bits of LSB2 and bottom 8 bits of MSB hold status bits. */
	pkt->hdcp_status = (sp_msb << 8) | (sp_lsb2 >> 24);
	pkt->value_flags |= TEGRA_NVHDCP_FLAG_S;

	/* 56-bit Kprime */
	pkt->k_prime = ((u64)(sp_lsb2 & 0xffffff) << 32) | sp_lsb1;
	pkt->value_flags |= TEGRA_NVHDCP_FLAG_KP;

	/* is connection state supported? */
	if (sp_msb & STATUS_CS) {
		pkt->cs = get_cs(hdmi);
		pkt->value_flags |= TEGRA_NVHDCP_FLAG_CS;
	}

	/* load Dksv */
	pkt->d_ksv = get_dksv(hdmi);
	if (verify_ksv(pkt->d_ksv)) {
		nvhdcp_err("Dksv invalid!\n");
		pkt->packet_results = TEGRA_NVHDCP_RESULT_UNSUCCESSFUL;
		e = -EIO; /* treat bad Dksv as I/O error */
	}
	pkt->value_flags |= TEGRA_NVHDCP_FLAG_DKSV;

	/* copy current Bksv */
	pkt->b_ksv = nvhdcp->b_ksv;
	pkt->value_flags |= TEGRA_NVHDCP_FLAG_BKSV;

	pkt->packet_results = TEGRA_NVHDCP_RESULT_SUCCESS;
	mutex_unlock(&nvhdcp->lock);
	return 0;

err:
	mutex_unlock(&nvhdcp->lock);
	return e;
}

/* get M prime - READ_M on TMDS0_LINK0 only */
static inline int get_m_prime(struct tegra_nvhdcp *nvhdcp,
				struct tegra_nvhdcp_packet *pkt)
{
	struct tegra_hdmi *hdmi = nvhdcp->hdmi;
	int e;

	pkt->packet_results = TEGRA_NVHDCP_RESULT_UNSUCCESSFUL;

	/* if connection isn't authenticated ... */
	mutex_lock(&nvhdcp->lock);
	if (nvhdcp->state != STATE_LINK_VERIFY) {
		memset(pkt, 0, sizeof(*pkt));
		pkt->packet_results = TEGRA_NVHDCP_RESULT_LINK_FAILED;
		e = 0;
		goto err;
	}

	pkt->a_ksv = nvhdcp->a_ksv;
	pkt->a_n = nvhdcp->a_n;
	pkt->value_flags = TEGRA_NVHDCP_FLAG_AKSV | TEGRA_NVHDCP_FLAG_AN;

	set_cn(hdmi, pkt->c_n);

	tegra_sor_writel_ext(hdmi->sor, NV_SOR_TMDS_HDCP_CMODE,
					TMDS0_LINK0 | READ_M);

	/* Cksv write triggers Mprime update */
	set_cksv(hdmi, pkt->c_ksv);

	e = wait_hdcp_ctrl(hdmi, MPRIME_VALID, NULL);
	if (e) {
		nvhdcp_err("Mprime read timeout\n");
		e = -EIO;
		goto err;
	}
	msleep(50);

	/* load Mprime */
	pkt->m_prime = get_mprime(hdmi);
	pkt->value_flags |= TEGRA_NVHDCP_FLAG_MP;

	pkt->b_status = nvhdcp->b_status;
	pkt->value_flags |= TEGRA_NVHDCP_FLAG_BSTATUS;

	/* copy most recent KSVFIFO, if it is non-zero */
	pkt->num_bksv_list = nvhdcp->num_bksv_list;
	if (nvhdcp->num_bksv_list) {
		BUILD_BUG_ON(sizeof(pkt->bksv_list) !=
				sizeof(nvhdcp->bksv_list));
		memcpy(pkt->bksv_list, nvhdcp->bksv_list,
			nvhdcp->num_bksv_list * sizeof(*pkt->bksv_list));
		pkt->value_flags |= TEGRA_NVHDCP_FLAG_BKSVLIST;
	}

	/* copy v_prime */
	BUILD_BUG_ON(sizeof(pkt->v_prime) != sizeof(nvhdcp->v_prime));
	memcpy(pkt->v_prime, nvhdcp->v_prime, sizeof(nvhdcp->v_prime));
	pkt->value_flags |= TEGRA_NVHDCP_FLAG_V;

	/* load Dksv */
	pkt->d_ksv = get_dksv(hdmi);
	if (verify_ksv(pkt->d_ksv)) {
		nvhdcp_err("Dksv invalid!\n");
		e = -EIO;
		goto err;
	}
	pkt->value_flags |= TEGRA_NVHDCP_FLAG_DKSV;

	/* copy current Bksv */
	pkt->b_ksv = nvhdcp->b_ksv;
	pkt->value_flags |= TEGRA_NVHDCP_FLAG_BKSV;

	pkt->packet_results = TEGRA_NVHDCP_RESULT_SUCCESS;
	mutex_unlock(&nvhdcp->lock);
	return 0;

err:
	mutex_unlock(&nvhdcp->lock);
	return e;
}

static int load_kfuse(struct tegra_hdmi *hdmi)
{
	unsigned buf[KFUSE_DATA_SZ / 4];
	int e, i;
	u32 ctrl;
	u32 tmp;
	int retries;

	/* copy load kfuse into buffer - only needed for early Tegra parts */
	e = tegra_kfuse_read(buf, sizeof(buf));
	if (e) {
		nvhdcp_err("Kfuse read failure\n");
		return e;
	}

	/* write the kfuse to HDMI SRAM */

	tegra_sor_writel_ext(hdmi->sor, NV_SOR_KEY_CTRL, 1); /* LOAD_KEYS */

	/* issue a reload */
	ctrl = tegra_sor_readl_ext(hdmi->sor, NV_SOR_KEY_CTRL);
	tegra_sor_writel_ext(hdmi->sor, NV_SOR_KEY_CTRL,
				ctrl | PKEY_RELOAD_TRIGGER | LOCAL_KEYS);

	e = wait_key_ctrl(hdmi, PKEY_LOADED, PKEY_LOADED);
	if (e) {
		nvhdcp_err("key reload timeout\n");
		return -EIO;
	}

	tegra_sor_writel_ext(hdmi->sor, NV_SOR_KEY_SKEY_INDEX, 0);

	/* wait for SRAM to be cleared */
	retries = 6;
	do {
		tmp = tegra_sor_readl_ext(hdmi->sor, NV_SOR_KEY_DEBUG0);
		if ((tmp & 1) == 0)
			break;
		if (retries > 1)
			mdelay(1);
	} while (--retries);
	if (!retries) {
		nvhdcp_err("key SRAM clear timeout\n");
		return -EIO;
	}

	for (i = 0; i < KFUSE_DATA_SZ / 4; i += 4) {

		/* load 128-bits*/
		tegra_sor_writel_ext(hdmi->sor,
			NV_SOR_KEY_HDCP_KEY_0, buf[i]);
		tegra_sor_writel_ext(hdmi->sor,
			NV_SOR_KEY_HDCP_KEY_1, buf[i+1]);
		tegra_sor_writel_ext(hdmi->sor,
			NV_SOR_KEY_HDCP_KEY_2, buf[i+2]);
		tegra_sor_writel_ext(hdmi->sor,
			NV_SOR_KEY_HDCP_KEY_3, buf[i+3]);

		/* trigger LOAD_HDCP_KEY */
		tegra_sor_writel_ext(hdmi->sor,
			NV_SOR_KEY_HDCP_KEY_TRIG, 0x100);

		tmp = LOCAL_KEYS | WRITE16;
		if (i)
			tmp |= AUTOINC;
		tegra_sor_writel_ext(hdmi->sor, NV_SOR_KEY_CTRL, tmp);

		/* wait for WRITE16 to complete */
		e = wait_key_ctrl(hdmi, 0x10, 0); /* WRITE16 */
		if (e) {
			nvhdcp_err("key write timeout\n");
			return -EIO;
		}
	}

	return 0;
}

/* validate srm signature for hdcp 2.2 */
static int get_srm_signature(struct hdcp_context_t *hdcp_context,
		char *nonce, uint64_t *pkt, void *ta_ctx, uint8_t version)
{
	int err = 0;
	/* generate nonce in the ucode */
	err = tsec_hdcp_generate_nonce(hdcp_context, nonce);
	if (err) {
		nvhdcp_err("Error generating nonce!\n");
		return err;
	}

	/* pass the nonce to hdcp TA and get the signature back */
	memcpy(pkt, nonce, HDCP_NONCE_SIZE);
	*(pkt + HDCP_NONCE_SIZE) = version;

#ifdef CONFIG_TRUSTED_LITTLE_KERNEL
	if (te_is_secos_dev_enabled())
		err = te_launch_trusted_oper_tlk(pkt, PKT_SIZE, session_id,
				hdcp_uuid, HDCP_CMD_GEN_CMAC, sizeof(hdcp_uuid));
	else
		err = te_launch_trusted_oper(pkt, PKT_SIZE,
				HDCP_CMD_GEN_CMAC, ta_ctx);
#else
	err = te_launch_trusted_oper(pkt, PKT_SIZE, HDCP_CMD_GEN_CMAC, ta_ctx);
#endif

	if (err)
		nvhdcp_err("te launch operation failed with error %d\n", err);
	return err;
}

static int verify_link(struct tegra_nvhdcp *nvhdcp, bool wait_ri)
{
	struct tegra_hdmi *hdmi = nvhdcp->hdmi;
	int retries = 3;
	u16 old, rx, tx;
	int e;

	old = 0;
	rx = 0;
	tx = 0;
	/* retry 3 times to deal with I2C link issues */
	do {
		if (wait_ri)
			old = get_transmitter_ri(hdmi);

		e = get_receiver_ri(nvhdcp, &rx);
		if (!e) {
			if (!rx) {
				nvhdcp_err("Ri is 0!\n");
				return -EINVAL;
			}

			tx = get_transmitter_ri(hdmi);
		} else {
			rx = ~tx;
			msleep(50);
		}

	} while (wait_ri && --retries && old != tx);

	nvhdcp_debug("R0 Ri poll:rx=0x%04x tx=0x%04x\n", rx, tx);

	mutex_lock(&nvhdcp->lock);
	if (!nvhdcp_is_plugged(nvhdcp)) {
		nvhdcp_err("aborting verify links - lost hdmi connection\n");
		mutex_unlock(&nvhdcp->lock);
		return -EIO;
	}
	mutex_unlock(&nvhdcp->lock);

	if (rx != tx)
		return -EINVAL;

	return 0;
}

static int verify_vprime(struct tegra_nvhdcp *nvhdcp, u8 repeater)
{
	int i;
	u8 *p;
	u8 buf[RCVR_ID_LIST_SIZE];
	unsigned char nonce[HDCP_NONCE_SIZE];
	struct hdcp_verify_vprime_param verify_vprime_param;
	int e = 0;
	uint64_t *pkt = NULL;
	struct hdcp_context_t *hdcp_context =
		kmalloc(sizeof(struct hdcp_context_t), GFP_KERNEL);

	e = tsec_hdcp_context_creation(hdcp_context,
			DISPLAY_TYPE_HDMI, nvhdcp->hdmi->sor->ctrl_num);
	if (e) {
		nvhdcp_err("hdcp context create/init failed\n");
		goto exit;
	}
	pkt = kzalloc(PKT_SIZE, GFP_KERNEL);

	if (!pkt || !hdcp_context)
		goto exit;

	if (tegra_dc_is_nvdisplay()) {
		e = get_srm_signature(hdcp_context, nonce, pkt, nvhdcp->ta_ctx,
					HDCP_1x);
		if (e) {
			nvhdcp_err("Error getting srm signature!\n");
			goto exit;
		}
	}

	memset(&verify_vprime_param, 0x0,
		sizeof(struct hdcp_verify_vprime_param));
	if (!repeater) {
		nvhdcp->num_bksv_list = 1;
		nvhdcp->bksv_list[0] = nvhdcp->b_ksv;
		verify_vprime_param.reserved1[0] = TSEC_SRM_REVOCATION_CHECK;
	}
	p = buf;
	for (i = 0; i < nvhdcp->num_bksv_list; i++) {
		p[0] = (u8)(nvhdcp->bksv_list[i] & 0xff);
		p[1] = (u8)((nvhdcp->bksv_list[i]>>8) & 0xff);
		p[2] = (u8)((nvhdcp->bksv_list[i]>>16) & 0xff);
		p[3] = (u8)((nvhdcp->bksv_list[i]>>24) & 0xff);
		p[4] = (u8)((nvhdcp->bksv_list[i]>>32) & 0xff);
		p += 5;
	}
	memcpy((void *)verify_vprime_param.vprime, nvhdcp->v_prime,
			HDCP_SIZE_VPRIME_1X_8);
	verify_vprime_param.bstatus = nvhdcp->b_status;
	verify_vprime_param.port = TEGRA_NVHDCP_PORT_HDMI; /* hdcp 1.x */
	e = tsec_hdcp1x_verify_vprime(verify_vprime_param, hdcp_context,
			buf, nvhdcp->num_bksv_list, pkt);
exit:
	tsec_hdcp_free_context(hdcp_context);
	kfree(pkt);
	return e;
}

static int get_repeater_info(struct tegra_nvhdcp *nvhdcp)
{
	int e, retries;
	u8 b_caps;
	u16 b_status;

	nvhdcp_vdbg("repeater found:fetching repeater info\n");

	/* wait up to 5 seconds for READY on repeater */
	retries = 51;
	do {
		mutex_lock(&nvhdcp->lock);
		if (!nvhdcp_is_plugged(nvhdcp)) {
			nvhdcp_err("disconnect while waiting for repeater\n");
			mutex_unlock(&nvhdcp->lock);
			return -EIO;
		}
		mutex_unlock(&nvhdcp->lock);

		e = get_bcaps(nvhdcp, &b_caps);
		if (!e && (b_caps & BCAPS_READY)) {
			nvhdcp_debug("Bcaps READY from repeater\n");
			break;
		}
		if (retries > 1)
			msleep(100);
	} while (--retries);
	if (!retries) {
		nvhdcp_err("repeater Bcaps read timeout\n");
		return -ETIMEDOUT;
	}

	memset(nvhdcp->v_prime, 0, sizeof(nvhdcp->v_prime));
	e = get_vprime(nvhdcp, nvhdcp->v_prime);
	if (e) {
		nvhdcp_err("repeater Vprime read failure!\n");
		return e;
	}

	e = nvhdcp_i2c_read16(nvhdcp, 0x41, &b_status);
	if (e) {
		nvhdcp_err("Bstatus read failure!\n");
		return e;
	}

	if (b_status & BSTATUS_MAX_DEVS_EXCEEDED) {
		nvhdcp_err("repeater:max devices (0x%04x)\n", b_status);
		return -EINVAL;
	}

	if (b_status & BSTATUS_MAX_CASCADE_EXCEEDED) {
		nvhdcp_err("repeater:max cascade (0x%04x)\n", b_status);
		return -EINVAL;
	}

	nvhdcp->b_status = b_status;
	nvhdcp->num_bksv_list = b_status & 0x7f;
	nvhdcp_vdbg("Bstatus 0x%x (devices: %d)\n",
				b_status, nvhdcp->num_bksv_list);

	memset(nvhdcp->bksv_list, 0, sizeof(nvhdcp->bksv_list));
	e = get_ksvfifo(nvhdcp, nvhdcp->num_bksv_list, nvhdcp->bksv_list);
	if (e) {
		nvhdcp_err("repeater:could not read KSVFIFO (err %d)\n", e);
		return e;
	}

	return 0;
}

static int nvhdcp_ake_init_send(struct tegra_nvhdcp *nvhdcp, u8 *buf)
{
	int e;
	e = nvhdcp_i2c_write(nvhdcp, 0x60, SIZE_AKE_INIT, buf);
	return e;
}

static int nvhdcp_ake_cert_receive(struct tegra_nvhdcp *nvhdcp, u8 *buf)
{
	int e;
	e = nvhdcp_i2c_read(nvhdcp, 0x80, SIZE_AKE_SEND_CERT, buf);
	return e;
}

static int nvhdcp_ake_no_stored_km_send(struct tegra_nvhdcp *nvhdcp, u8 *buf)
{
	int e;
	e = nvhdcp_i2c_write(nvhdcp, 0x60, SIZE_AKE_NO_STORED_KM, buf);
	return e;
}

static int nvhdcp_ake_hprime_receive(struct tegra_nvhdcp *nvhdcp, u8 *buf)
{
	int e;
	e = nvhdcp_i2c_read(nvhdcp, 0x80, SIZE_AKE_SEND_HPRIME, buf);
	return e;
}

static int nvhdcp_ake_pairing_info_receive(struct tegra_nvhdcp *nvhdcp,
	u8 *buf)
{
	int e;
	e = nvhdcp_i2c_read(nvhdcp, 0x80, SIZE_AKE_SEND_PAIRING_INFO, buf);
	return e;
}

static int nvhdcp_lc_init_send(struct tegra_nvhdcp *nvhdcp, u8 *buf)
{
	int e;
	e = nvhdcp_i2c_write(nvhdcp, 0x60, SIZE_LC_INIT, buf);
	return e;
}

static int nvhdcp_lc_lprime_receive(struct tegra_nvhdcp *nvhdcp, u8 *buf)
{
	int e;
	e = nvhdcp_i2c_read(nvhdcp, 0x80, SIZE_LC_SEND_LPRIME, buf);
	return e;
}

static int nvhdcp_ske_eks_send(struct tegra_nvhdcp *nvhdcp, u8 *buf)
{
	int e;
	e = nvhdcp_i2c_write(nvhdcp, 0x60, SIZE_SKE_SEND_EKS, buf);
	return e;
}

static int nvhdcp_receiverid_list_receive(struct tegra_nvhdcp *nvhdcp, u8 *buf)
{
	int e;
	e = nvhdcp_i2c_read(nvhdcp, 0x80, SIZE_SEND_RCVR_ID_LIST, buf);
	return e;
}

static int nvhdcp_rptr_ack_send(struct tegra_nvhdcp *nvhdcp, u8 *buf)
{
	int e;
	e = nvhdcp_i2c_write(nvhdcp, 0x60, SIZE_SEND_RPTR_ACK, buf);
	return e;
}

static int nvhdcp_rptr_stream_manage_send(struct tegra_nvhdcp *nvhdcp,
	u8 *buf)
{
	int e;
	e = nvhdcp_i2c_write(nvhdcp, 0x60, SIZE_SEND_RPTR_STREAM_MANAGE, buf);
	return e;
}

static int nvhdcp_rptr_stream_ready_receive(struct tegra_nvhdcp *nvhdcp,
	u8 *buf)
{
	int e;
	e = nvhdcp_i2c_read(nvhdcp, 0x80, SIZE_SEND_RPTR_STREAM_READY, buf);
	return e;
}

static int nvhdcp_poll(struct tegra_nvhdcp *nvhdcp, int timeout, int status)
{
	int e;
	u16 val;
	s64 start_time;
	s64 end_time;
	struct timespec tm;
	ktime_get_ts(&tm);
	start_time = timespec_to_ns(&tm);
	while (1) {
		ktime_get_ts(&tm);
		end_time = timespec_to_ns(&tm);
		if ((end_time - start_time)/1000 >= (s64)timeout*1000)
			return -ETIMEDOUT;
		else {
			e = nvhdcp_i2c_read(nvhdcp, 0x70, 2, &val);
			if (e) {
				nvhdcp_err("nvhdcp_poll_ready failed\n");
				goto exit;
			}
			if (status == HDCP_READY) {
				if (val & HDCP_MSG_SIZE_MASK)
					break;
			} else if (status == HDCP_REAUTH) {
				if (cpu_to_be16(val) & HDCP_REAUTH_MASK)
					break;
			}
		}
	}
	e = 0;
exit:
	return e;
}

static int nvhdcp_poll_ready(struct tegra_nvhdcp *nvhdcp, int timeout)
{
	int e;
	e = nvhdcp_poll(nvhdcp, timeout, HDCP_READY);
	return e;
}


static int tsec_hdcp_authentication(struct tegra_nvhdcp *nvhdcp,
				struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	u8 version = 2;
	u16 caps = 0;
	u16 txcaps = 0x0;
	uint64_t *pkt = NULL;
	unsigned char nonce[HDCP_NONCE_SIZE];
	int cert_retry_count;

	pkt = kzalloc(PKT_SIZE, GFP_KERNEL);

	if (!pkt)
		goto exit;

	err =  tsec_hdcp_readcaps(hdcp_context);
	if (err)
		goto exit;
	err =  tsec_hdcp_init(hdcp_context);
	if (err)
		goto exit;
	down_read(&nvhdcp->hdmi->sor->reset_lock);
	err =  tsec_hdcp_create_session(hdcp_context, DISPLAY_TYPE_HDMI,
				nvhdcp->hdmi->sor->ctrl_num);
	up_read(&nvhdcp->hdmi->sor->reset_lock);
	if (err)
		goto exit;
	err =  tsec_hdcp_exchange_info(hdcp_context,
			HDCP_EXCHANGE_INFO_GET_TMTR_INFO,
			&version,
			&caps);
	if (err)
		goto exit;

	msleep(50);

	hdcp_context->msg.txcaps_version = version;
	hdcp_context->msg.txcaps_capmask = txcaps;
	hdcp_context->msg.ake_init_msg_id = ID_AKE_INIT;

	err = nvhdcp_ake_init_send(nvhdcp,
		(u8 *)&hdcp_context->msg.ake_init_msg_id);
	if (err)
		goto exit;

	nvhdcp_vdbg("ake init sent is %x %llx %x %x\n",
	hdcp_context->msg.ake_init_msg_id, hdcp_context->msg.rtx,
	hdcp_context->msg.txcaps_version, hdcp_context->msg.txcaps_capmask);

	/* Certain receivers show a latency between asserting the ready bit on
	 * rxstatus register and writing the certificate on the i2c channel. Since
	 * the number of such receivers is quite high in the field, we do not want
	 * to regress by not letting those TVs authenticate HDCP. The fallback
	 * option is to give some time to the TVs to be ready to write on the
	 * channel
	 */
	cert_retry_count = MAX_CERT_RETRY;
	while (cert_retry_count) {

		err = nvhdcp_poll_ready(nvhdcp, RX_CERT_POLL_TIME);
		if (err) {
			/* RX should be ready for ceritificate within the stipulated period of time */
			nvhdcp_err("Rx not ready yet, bailing out!\n");
			goto exit;
		} else {
			/* else if rx is claiming to be ready within the retry count; read the certificate */
			/* clear old messages */
			memset(&hdcp_context->msg, 0, sizeof(hdcp_context->msg));
			err = nvhdcp_ake_cert_receive(nvhdcp,
					&hdcp_context->msg.ake_send_cert_msg_id);
			if (err)
				goto exit;
			if (hdcp_context->msg.ake_send_cert_msg_id != ID_AKE_SEND_CERT) {
				nvhdcp_err("Not ID_AKE_SEND_CERT but %d instead\n",
				hdcp_context->msg.ake_send_cert_msg_id);

				/* for error, print the first few bytes for debugging;
				 * the first few bytes will give us some clarity on whether
				 * the RX is sending the same buffer as what was sent by TX
				 * during ake_init
				 */
				nvhdcp_vdbg("first few bytes of cert %x %x %x %x\n",
				hdcp_context->msg.cert_rx[0], hdcp_context->msg.cert_rx[1],
				hdcp_context->msg.cert_rx[2], hdcp_context->msg.cert_rx[3]);
			} else
				/* received the cert, move ahead */
				break;
		}
		cert_retry_count--;
		/* sleep because there is nothing better to do ! :) */
		msleep(1);
	}
	if (hdcp_context->msg.ake_send_cert_msg_id != ID_AKE_SEND_CERT) {
		nvhdcp_err("Not ID_AKE_SEND_CERT but %d instead\n",
			hdcp_context->msg.ake_send_cert_msg_id);
		err = -EINVAL;
		goto exit;
	}
	err =  tsec_hdcp_verify_cert(hdcp_context);
	if (err)
		goto exit;
	err =  tsec_hdcp_update_rrx(hdcp_context);
	if (err)
		goto exit;
	err =  tsec_hdcp_generate_ekm(hdcp_context);
	if (err)
		goto exit;
	hdcp_context->msg.ake_no_stored_km_msg_id = ID_AKE_NO_STORED_KM;
	err = nvhdcp_ake_no_stored_km_send(nvhdcp,
		&hdcp_context->msg.ake_no_stored_km_msg_id);
	if (err)
		goto exit;
	err =  tsec_hdcp_exchange_info(hdcp_context,
		HDCP_EXCHANGE_INFO_SET_RCVR_INFO,
		&hdcp_context->msg.rxcaps_version,
		&hdcp_context->msg.rxcaps_capmask);
	if (err)
		goto exit;

#ifdef CONFIG_TRUSTED_LITTLE_KERNEL
	/* differentiate between TLK and trusty */
	if (te_is_secos_dev_enabled()) {
		err = te_open_trusted_session_tlk(hdcp_uuid, sizeof(hdcp_uuid),
					&session_id);
	} else {
		nvhdcp->ta_ctx = NULL;
		/* Open a trusted sesion with HDCP TA */
		err = te_open_trusted_session(HDCP_PORT_NAME, &nvhdcp->ta_ctx);
	}
#else
	nvhdcp->ta_ctx = NULL;
	/* Open a trusted sesion with HDCP TA */
	err = te_open_trusted_session(HDCP_PORT_NAME, &nvhdcp->ta_ctx);
#endif

	if (err) {
		nvhdcp_err("Error opening trusted session\n");
		goto exit;
	}
	err = get_srm_signature(hdcp_context, nonce, pkt, nvhdcp->ta_ctx,
			HDCP_22);
	if (err) {
		nvhdcp_err("Error getting srm signature!\n");
		goto exit;
	}
	err =  tsec_hdcp_revocation_check(hdcp_context,
		(unsigned char *)(pkt + HDCP_CMAC_OFFSET),
		*((unsigned int *)(pkt + HDCP_TSEC_ADDR_OFFSET)),
		TEGRA_NVHDCP_PORT_HDMI, HDCP_22);
	if (err)
		goto exit;
	err = nvhdcp_poll_ready(nvhdcp, 1000);
	if (err)
		goto exit;
	err = nvhdcp_ake_hprime_receive(nvhdcp,
		&hdcp_context->msg.ake_send_hprime_msg_id);
	if (err)
		goto exit;
	if (hdcp_context->msg.ake_send_hprime_msg_id != ID_AKE_SEND_HPRIME) {
		nvhdcp_err("Not ID_AKE_SEND_HPRIME but %d instead\n",
			hdcp_context->msg.ake_send_hprime_msg_id);
		err = -EINVAL;
		goto exit;
	}
	err =  tsec_hdcp_verify_hprime(hdcp_context);
	if (err)
		goto exit;
	err = nvhdcp_poll_ready(nvhdcp, 200);
	if (err)
		goto exit;
	err = nvhdcp_ake_pairing_info_receive(nvhdcp,
		&hdcp_context->msg.ake_send_pairing_info_msg_id);
	if (err)
		goto exit;
	if (hdcp_context->msg.ake_send_pairing_info_msg_id !=
	ID_AKE_SEND_PAIRING_INFO) {
		nvhdcp_err("Not ID_AKE_SEND_PAIRING_INFO but %d instead\n",
			hdcp_context->msg.ake_send_hprime_msg_id);
		err = -EINVAL;
		goto exit;
	}
	err =  tsec_hdcp_encrypt_pairing_info(hdcp_context);
	if (err)
		goto exit;
	err =  tsec_hdcp_generate_lc_init(hdcp_context);
	if (err)
		goto exit;
	hdcp_context->msg.lc_init_msg_id = ID_LC_INIT;
	err = nvhdcp_lc_init_send(nvhdcp, &hdcp_context->msg.lc_init_msg_id);
	if (err)
		goto exit;
	err = nvhdcp_poll_ready(nvhdcp, 20);
	if (err)
		goto exit;
	err = nvhdcp_lc_lprime_receive(nvhdcp,
		&hdcp_context->msg.lc_send_lprime_msg_id);
	if (err)
		goto exit;
	if (hdcp_context->msg.lc_send_lprime_msg_id != ID_LC_SEND_LPRIME) {
		nvhdcp_err("Not ID_LC_SEND_LPRIME but %d instead\n",
			hdcp_context->msg.lc_send_lprime_msg_id);
		err = -EINVAL;
		goto exit;
	}
	err =  tsec_hdcp_verify_lprime(hdcp_context);
	if (err)
		goto exit;
	err =  tsec_hdcp_ske_init(hdcp_context);
	if (err)
		goto exit;
	hdcp_context->msg.ske_send_eks_msg_id = ID_SKE_SEND_EKS;
	err = nvhdcp_ske_eks_send(nvhdcp,
		&hdcp_context->msg.ske_send_eks_msg_id);
	if (err)
		goto exit;
	if (hdcp_context->msg.rxcaps_capmask & HDCP_22_REPEATER) {
		nvhdcp->repeater = 1;
		err = nvhdcp_poll_ready(nvhdcp, 3000);
		if (err)
			goto exit;
		err = nvhdcp_receiverid_list_receive(nvhdcp,
				&hdcp_context->msg.send_receiverid_list_msg_id);
		if (err)
			goto exit;
		if (hdcp_context->msg.send_receiverid_list_msg_id !=
		ID_SEND_RCVR_ID_LIST) {
			nvhdcp_err("Not ID_SEND_RCVR_ID_LIST but %d instead\n",
				hdcp_context->msg.send_receiverid_list_msg_id);
			err = -EINVAL;
			goto exit;
		}

		if (hdcp_context->msg.rxinfo & HDCP_NON_22_RX) {
			err = HDCP_FALLBACK_1X;
			g_fallback = 1;
			goto exit;
		}
		err = get_srm_signature(hdcp_context, nonce, pkt,
				nvhdcp->ta_ctx, HDCP_22);
		if (err) {
			nvhdcp_err("Error getting srm signature!\n");
			goto exit;
		}
		err =  tsec_hdcp_verify_vprime(hdcp_context,
		(char *)(pkt + HDCP_CMAC_OFFSET),
		*((unsigned int *)(pkt + HDCP_TSEC_ADDR_OFFSET)),
		TEGRA_NVHDCP_PORT_HDMI);

		if (err)
			goto exit;
		hdcp_context->msg.rptr_send_ack_msg_id = ID_SEND_RPTR_ACK;
		err = nvhdcp_rptr_ack_send(nvhdcp,
			&hdcp_context->msg.rptr_send_ack_msg_id);
		if (err)
			goto exit;
stream_manage_send:
		err =  tsec_hdcp_rptr_stream_manage(hdcp_context);
		if (err)
			goto exit;
		hdcp_context->msg.rptr_auth_stream_manage_msg_id =
			ID_SEND_RPTR_STREAM_MANAGE;
		/* Num of streams = 1, only video, big endian */
		hdcp_context->msg.k = 0x0100;
		/* STREAM_ID = 0 and Type = 0  */
		hdcp_context->msg.streamid_type[0] = 0x0100;

		err = nvhdcp_rptr_stream_manage_send(nvhdcp,
			&hdcp_context->msg.rptr_auth_stream_manage_msg_id);
		if (err)
			goto exit;
		err = nvhdcp_poll_ready(nvhdcp, 100);
		if (err) {
			/* HDCP 2.2 analyzer expects to retry atleast once */
			if (g_seq_num_m_retries >= SEQ_NUM_M_MAX_RETRIES)
				goto exit;
			else {
				g_seq_num_m_retries++;
				goto stream_manage_send;
			}
		}
		err = nvhdcp_rptr_stream_ready_receive(nvhdcp,
			&hdcp_context->msg.rptr_auth_stream_ready_msg_id);
		if (err)
			goto exit;
		if (hdcp_context->msg.rptr_auth_stream_ready_msg_id !=
		ID_SEND_RPTR_STREAM_READY) {
			nvhdcp_err("Not ID_SEND_RPTR_STREAM_READY but %d\n",
			hdcp_context->msg.rptr_auth_stream_ready_msg_id);
			err = -EINVAL;
			goto exit;
		}
		err =  tsec_hdcp_rptr_stream_ready(hdcp_context);
		if (err) {
			/* HDCP 2.2 analyzer expects to retry atleast once */
			if (g_seq_num_m_retries >= SEQ_NUM_M_MAX_RETRIES)
				goto exit;
			else {
				g_seq_num_m_retries++;
				goto stream_manage_send;
			}
		}
	}

	nvhdcp_info("HDCP Authentication successful!\n");

exit:
	if (err)
		nvhdcp_err("HDCP authentication failed with err %d\n", err);
	kfree(pkt);

#ifdef CONFIG_TRUSTED_LITTLE_KERNEL
	if (te_is_secos_dev_enabled()) {
		if (session_id) {
			te_close_trusted_session_tlk(session_id, hdcp_uuid,
			sizeof(hdcp_uuid));
			session_id = 0;
		}
	} else {
		if (nvhdcp->ta_ctx) {
			te_close_trusted_session(nvhdcp->ta_ctx);
			nvhdcp->ta_ctx = NULL;
		}
	}
#else
	if (nvhdcp->ta_ctx) {
		te_close_trusted_session(nvhdcp->ta_ctx);
		nvhdcp->ta_ctx = NULL;
	}
#endif

	return err;
}

void tegra_nvhdcp_clear_fallback(struct tegra_nvhdcp *nvhdcp)
{
	g_fallback = 0;
}

static void nvhdcp_fallback_worker(struct work_struct *work)
{
	struct tegra_nvhdcp *nvhdcp =
		container_of(to_delayed_work(work), struct tegra_nvhdcp, fallback_work);
	struct tegra_hdmi *hdmi = nvhdcp->hdmi;
	struct tegra_dc *dc = tegra_dc_hdmi_get_dc(hdmi);
	int hotplug_state;
	bool dc_enabled;

	hotplug_state = tegra_hdmi_get_hotplug_state(hdmi);

	mutex_lock(&dc->lock);
	dc_enabled = dc->enabled;
	mutex_unlock(&dc->lock);

	if (hotplug_state == TEGRA_HPD_STATE_NORMAL) {
		tegra_hdmi_set_hotplug_state(hdmi, TEGRA_HPD_STATE_FORCE_DEASSERT);
		cancel_delayed_work(&nvhdcp->fallback_work);
		queue_delayed_work(nvhdcp->fallback_wq, &nvhdcp->fallback_work,
			msecs_to_jiffies(1000));
	} else if (hotplug_state == TEGRA_HPD_STATE_FORCE_DEASSERT && dc_enabled) {
		cancel_delayed_work(&nvhdcp->fallback_work);
		queue_delayed_work(nvhdcp->fallback_wq, &nvhdcp->fallback_work,
			msecs_to_jiffies(1000));
	} else if (hotplug_state == TEGRA_HPD_STATE_FORCE_DEASSERT && !dc_enabled) {
		tegra_hdmi_set_hotplug_state(hdmi, TEGRA_HPD_STATE_NORMAL);
	}
}

static void nvhdcp_downstream_worker(struct work_struct *work)
{
	struct tegra_nvhdcp *nvhdcp =
		container_of(to_delayed_work(work), struct tegra_nvhdcp, work);
	struct tegra_hdmi *hdmi = nvhdcp->hdmi;
	struct tegra_dc *dc = tegra_dc_hdmi_get_dc(hdmi);
	int e;
	int alloc_err = 0;
	u8 b_caps;
	int hdcp_ta_ret; /* track returns from TA */
	uint32_t ta_cmd = HDCP_AUTH_CMD;
	bool enc = false;
	u32 tmp;
	u32 res;
	uint64_t *pkt = NULL;

	if (tegra_dc_is_nvdisplay()) {
		pkt = kzalloc(PKT_SIZE, GFP_KERNEL);

		if (!pkt) {
			nvhdcp_err("Memory allocation failed\n");
			alloc_err = -ENOMEM;
			goto failure;
		}
	}


	nvhdcp_vdbg("%s():started thread %s for sor: %x\n", __func__,
			nvhdcp->name, nvhdcp->hdmi->sor->ctrl_num);
	tegra_dc_io_start(dc);

	mutex_lock(&nvhdcp->lock);
	if (nvhdcp->state == STATE_OFF) {
		nvhdcp_err("nvhdcp failure - giving up\n");
		goto err;
	}
	nvhdcp->state = STATE_UNAUTHENTICATED;

	/* check plug state to terminate early in case flush_workqueue() */
	if (!nvhdcp_is_plugged(nvhdcp)) {
		nvhdcp_err("worker started while unplugged!\n");
		goto lost_hdmi;
	}
	nvhdcp_vdbg("%s():hpd=%d\n", __func__, nvhdcp->plugged);

	nvhdcp->a_ksv = 0;
	nvhdcp->b_ksv = 0;
	nvhdcp->a_n = 0;
	mutex_unlock(&nvhdcp->lock);

	e = get_bcaps(nvhdcp, &b_caps);
	mutex_lock(&nvhdcp->lock);
	if (e) {
		nvhdcp_err("Bcaps read failure\n");
		goto failure;
	}

	nvhdcp_vdbg("read Bcaps = 0x%02x\n", b_caps);

	if (tegra_dc_is_nvdisplay()) {
		nvhdcp->ta_ctx = NULL;
		e = te_open_trusted_session(HDCP_PORT_NAME, &nvhdcp->ta_ctx);
		if (e) {
			nvhdcp_err("Error opening trusted session\n");
			goto failure;
		}

		/* if session successfully opened, launch operations
		 * repeater flag in Bskv must be configured before loading fuses
		 */
		*pkt = HDCP_TA_CMD_REP;
		*(pkt + 1*HDCP_CMD_OFFSET) = TEGRA_NVHDCP_PORT_HDMI;
		*(pkt + 2*HDCP_CMD_OFFSET) = 0;
		*(pkt + 3*HDCP_CMD_OFFSET) = b_caps & BCAPS_REPEATER;
		*(pkt + 4*HDCP_CMD_OFFSET) = false;
		*(pkt + 5*HDCP_CMD_OFFSET) = nvhdcp->hdmi->sor->ctrl_num;
		e = te_launch_trusted_oper(pkt, PKT_SIZE, ta_cmd, nvhdcp->ta_ctx);
		if (e) {
			nvhdcp_err("te launch operation failed with error %d\n",
				    e);
			goto failure;
		} else {
			nvhdcp_vdbg("Loading kfuse\n");
			e = load_kfuse(hdmi);
			if (e) {
				nvhdcp_err("kfuse could not be loaded\n");
				goto failure;
			}
		}
		usleep_range(20000, 25000);
		*pkt = HDCP_TA_CMD_CTRL;
		*(pkt + 1*HDCP_CMD_OFFSET) = TEGRA_NVHDCP_PORT_HDMI;
		*(pkt + 2*HDCP_CMD_OFFSET) = HDCP_TA_CTRL_ENABLE;
		*(pkt + 3*HDCP_CMD_OFFSET) = false;
		*(pkt + 4*HDCP_CMD_OFFSET) = nvhdcp->hdmi->sor->ctrl_num;
		e = te_launch_trusted_oper(pkt, PKT_SIZE, ta_cmd, nvhdcp->ta_ctx);
		if (e) {
			nvhdcp_err("te launch operation failed with error %d\n",
				    e);
			goto failure;
		} else {
			nvhdcp_vdbg("wait AN_VALID ...\n");

			hdcp_ta_ret = *pkt;
			nvhdcp_vdbg("An returned %x\n", e);
			if (hdcp_ta_ret) {
				nvhdcp_err("An key generation timeout\n");
				goto failure;
			}

			/* check SROM return */
			hdcp_ta_ret = *(pkt + HDCP_CMD_BYTE_OFFSET);
			if (hdcp_ta_ret) {
				nvhdcp_err("SROM error\n");
				goto failure;
			}
		}

		msleep(25);
		*pkt = HDCP_TA_CMD_AKSV;
		*(pkt + 1) = TEGRA_NVHDCP_PORT_HDMI;
		*(pkt + 2) = false;
		*(pkt + 3) = nvhdcp->hdmi->sor->ctrl_num;
		e = te_launch_trusted_oper(pkt, PKT_SIZE, ta_cmd, nvhdcp->ta_ctx);
		if (e) {
			nvhdcp_err("te launch operation failed with error %d\n",
				    e);
			goto failure;
		} else {
			hdcp_ta_ret = (u64)*pkt;
			nvhdcp->a_ksv = (u64)*(pkt + 1*HDCP_CMD_BYTE_OFFSET);
			nvhdcp->a_n = (u64)*(pkt + 2*HDCP_CMD_BYTE_OFFSET);
			nvhdcp_vdbg("Aksv is 0x%016llx\n", nvhdcp->a_ksv);
			nvhdcp_vdbg("An is 0x%016llx\n", nvhdcp->a_n);
			/* check if verification of Aksv failed */
			if (hdcp_ta_ret) {
				nvhdcp_err("Aksv verify failure\n");
				goto disable;
			}
		}
	} else {
		set_bksv(hdmi, 0, (b_caps & BCAPS_REPEATER));
		e = load_kfuse(hdmi);
		if (e) {
			nvhdcp_err("kfuse could not be loaded\n");
			goto failure;
		}

		usleep_range(20000, 25000);
		hdcp_ctrl_run(hdmi, 1);

		nvhdcp_vdbg("wait AN_VALID ...\n");

		/* wait for hardware to generate HDCP values */
		e = wait_hdcp_ctrl(hdmi, AN_VALID | SROM_ERR, &res);
		if (e) {
			nvhdcp_err("An key generation timeout\n");
			goto failure;
		}
		if (res & SROM_ERR) {
			nvhdcp_err("SROM error\n");
			goto failure;
		}

		msleep(25);

		nvhdcp->a_ksv = get_aksv(hdmi);
		nvhdcp->a_n = get_an(hdmi);
		nvhdcp_vdbg("Aksv is %016llx\n", nvhdcp->a_ksv);
		nvhdcp_vdbg("An is 0x%016llx\n", nvhdcp->a_n);

		if (verify_ksv(nvhdcp->a_ksv)) {
			nvhdcp_err("Aksv verify failure! (0x%016llx)\n",
				    nvhdcp->a_ksv);
			goto disable;
		}
	}
	mutex_unlock(&nvhdcp->lock);

	/* write Ainfo to receiver - set 1.1 only if b_caps supports it */
	e = nvhdcp_i2c_write8(nvhdcp, 0x15, b_caps & BCAPS_11);
	if (e) {
		nvhdcp_err("Ainfo write failure\n");
		mutex_lock(&nvhdcp->lock);
		goto failure;
	}

	/* write An to receiver */
	e = nvhdcp_i2c_write64(nvhdcp, 0x18, nvhdcp->a_n);
	if (e) {
		nvhdcp_err("An write failure\n");
		mutex_lock(&nvhdcp->lock);
		goto failure;
	}

	nvhdcp_vdbg("wrote An = 0x%016llx\n", nvhdcp->a_n);

	/* write Aksv to receiver - triggers auth sequence */
	e = nvhdcp_i2c_write40(nvhdcp, 0x10, nvhdcp->a_ksv);
	if (e) {
		nvhdcp_err("Aksv write failure\n");
		mutex_lock(&nvhdcp->lock);
		goto failure;
	}

	nvhdcp_vdbg("wrote Aksv = 0x%010llx\n", nvhdcp->a_ksv);

	mutex_lock(&nvhdcp->lock);
	/* bail out if unplugged in the middle of negotiation */
	if (!nvhdcp_is_plugged(nvhdcp))
		goto lost_hdmi;
	mutex_unlock(&nvhdcp->lock);

	/* get Bksv from receiver */
	e = nvhdcp_i2c_read40(nvhdcp, 0x00, &nvhdcp->b_ksv);
	mutex_lock(&nvhdcp->lock);
	if (e) {
		nvhdcp_err("Bksv read failure\n");
		goto failure;
	}
	nvhdcp_vdbg("Bksv is 0x%016llx\n", nvhdcp->b_ksv);

	if (tegra_dc_is_nvdisplay()) {
		*pkt = HDCP_TA_CMD_BKSV;
		*(pkt + 1*HDCP_CMD_OFFSET) = TEGRA_NVHDCP_PORT_HDMI;
		*(pkt + 2*HDCP_CMD_OFFSET) = nvhdcp->b_ksv;
		*(pkt + 3*HDCP_CMD_OFFSET) = b_caps & BCAPS_REPEATER;
		*(pkt + 4*HDCP_CMD_OFFSET) = false;
		*(pkt + 5*HDCP_CMD_OFFSET) =  nvhdcp->hdmi->sor->ctrl_num;
		e = te_launch_trusted_oper(pkt, PKT_SIZE, ta_cmd, nvhdcp->ta_ctx);
		if (e) {
			nvhdcp_err("te launch operation failed with error %d\n",
					e);
		} else {
			/* check if Bksv verification was successful */
			hdcp_ta_ret = (int)*pkt;
			if (hdcp_ta_ret) {
				nvhdcp_err("Bksv verify failure\n");
				goto failure;
			} else {
				nvhdcp_vdbg("loaded Bksv into controller\n");
				/* check if R0 read was successful */
				hdcp_ta_ret = (int)*pkt;
				if (hdcp_ta_ret) {
					nvhdcp_err("R0 read failure\n");
					goto failure;
				}
			}
		}
	} else {
		if (verify_ksv(nvhdcp->b_ksv)) {
			nvhdcp_err("Bksv verify failure!\n");
			goto failure;
		}

		set_bksv(hdmi, nvhdcp->b_ksv, (b_caps & BCAPS_REPEATER));
		nvhdcp_vdbg("loaded Bksv into controller\n");
		e = wait_hdcp_ctrl(hdmi, R0_VALID, NULL);
		if (e) {
			nvhdcp_err("R0 read failure!\n");
			goto failure;
		}
		nvhdcp_vdbg("R0 valid\n");
	}

	msleep(100); /* can't read R0' within 100ms of writing Aksv */

	nvhdcp_vdbg("verifying links ...\n");

	mutex_unlock(&nvhdcp->lock);
	e = verify_link(nvhdcp, false);
	if (e) {
		nvhdcp_err("link verification failed err %d\n", e);
		mutex_lock(&nvhdcp->lock);
		goto failure;
	}

	mutex_lock(&nvhdcp->lock);

	if (tegra_dc_is_nvdisplay()) {
		*pkt = HDCP_TA_CMD_ENC;
		*(pkt + HDCP_CMD_OFFSET) = TEGRA_NVHDCP_PORT_HDMI;
		*(pkt + 2*HDCP_CMD_OFFSET) = b_caps;
		*(pkt + 3*HDCP_CMD_OFFSET) = nvhdcp->hdmi->sor->ctrl_num;
		e = te_launch_trusted_oper(pkt, PKT_SIZE/4, ta_cmd, nvhdcp->ta_ctx);
		if (e) {
			nvhdcp_err("te launch operation failed with error %d\n",
					e);
			goto failure;
		}
		enc = true;
	} else {
		tmp = tegra_sor_readl_ext(hdmi->sor, NV_SOR_TMDS_HDCP_CTRL);
		tmp |= CRYPT_ENABLED;
		if (b_caps & BCAPS_11) /* HDCP 1.1 ? */
			tmp |= ONEONE_ENABLED;
		tegra_sor_writel_ext(hdmi->sor, NV_SOR_TMDS_HDCP_CTRL, tmp);
	}

	nvhdcp_vdbg("CRYPT enabled\n");
	mutex_unlock(&nvhdcp->lock);

	/* if repeater then get repeater info */
	if (b_caps & BCAPS_REPEATER) {
		e = get_repeater_info(nvhdcp);
		if (e) {
			nvhdcp_err("get repeater info failed\n");
			mutex_lock(&nvhdcp->lock);
			goto failure;
		}
	}
	/* T210/T210B01 vprime verification is handled in the upstream lib */
	if (tegra_dc_is_nvdisplay()) {
		/* perform vprime verification for repeater or SRM
		 * revocation check for receiver
		 */
		e = verify_vprime(nvhdcp, b_caps & BCAPS_REPEATER);
		if (e) {
			nvhdcp_err("get vprime params failed\n");
			goto failure;
		} else
			nvhdcp_vdbg("vprime verification passed\n");
	}

	mutex_lock(&nvhdcp->lock);
	nvhdcp->state = STATE_LINK_VERIFY;
	nvhdcp_info("link verified!\n");

	while (1) {
		if (!nvhdcp_is_plugged(nvhdcp))
			goto lost_hdmi;

		if (nvhdcp->state != STATE_LINK_VERIFY)
			goto failure;

		mutex_unlock(&nvhdcp->lock);
		e = verify_link(nvhdcp, true);
		if (e) {
			nvhdcp_err("link verification failed err %d\n", e);
			mutex_lock(&nvhdcp->lock);
			goto failure;
		}
		tegra_dc_io_end(dc);
		wait_event_interruptible_timeout(wq_worker,
				!nvhdcp_is_plugged(nvhdcp), msecs_to_jiffies(1500));
		tegra_dc_io_start(dc);
		mutex_lock(&nvhdcp->lock);
	}
failure:
	nvhdcp->fail_count++;
	if (nvhdcp->max_retries <= HDCP_INFINITE_RETRIES ||
		nvhdcp->fail_count < nvhdcp->max_retries) {
		nvhdcp_err("nvhdcp failure - renegotiating in 1 second\n");
		if (!nvhdcp_is_plugged(nvhdcp))
			goto lost_hdmi;
		queue_delayed_work(nvhdcp->downstream_wq, &nvhdcp->work,
				msecs_to_jiffies(1000));
	} else
		nvhdcp_err("nvhdcp failure - too many failures, giving up!\n");

	/* Failed because of lack of memory */
	if (alloc_err == -ENOMEM) {
		g_fallback = 0;
		/* No need to unlock the mutex, memory failure
		 * only happens when the mutex is not held.
		 */
		return;
	}

lost_hdmi:
	nvhdcp->state = STATE_UNAUTHENTICATED;
	if (tegra_dc_is_nvdisplay()) {
		*pkt = HDCP_TA_CMD_CTRL;
		*(pkt + 1*HDCP_CMD_OFFSET) = TEGRA_NVHDCP_PORT_HDMI;
		*(pkt + 2*HDCP_CMD_OFFSET) = HDCP_TA_CTRL_DISABLE;
		*(pkt + 3*HDCP_CMD_OFFSET) = false;
		*(pkt + 4*HDCP_CMD_OFFSET) = nvhdcp->hdmi->sor->ctrl_num;
		if (enc) {
			e = te_launch_trusted_oper(pkt, PKT_SIZE, ta_cmd,
						   nvhdcp->ta_ctx);
			if (e) {
				nvhdcp_err(
				  "te launch operation failed with error: %d\n",
				   e);
				goto failure;
			}
		}
	} else {
		hdcp_ctrl_run(hdmi, 0);
	}
err:
	mutex_unlock(&nvhdcp->lock);
	if (tegra_dc_is_nvdisplay()) {
		kfree(pkt);
		if (nvhdcp->ta_ctx) {
			te_close_trusted_session(nvhdcp->ta_ctx);
			nvhdcp->ta_ctx = NULL;
		}
	}
	tegra_dc_io_end(dc);
	return;
disable:
	nvhdcp->state = STATE_OFF;
	if (tegra_dc_is_nvdisplay()) {
		kfree(pkt);
		if (nvhdcp->ta_ctx) {
			te_close_trusted_session(nvhdcp->ta_ctx);
			nvhdcp->ta_ctx = NULL;
		}
	}
	nvhdcp_set_plugged(nvhdcp, false);
	mutex_unlock(&nvhdcp->lock);
	tegra_dc_io_end(dc);
	return;
}

static int link_integrity_check(struct tegra_nvhdcp *nvhdcp,
			struct hdcp_context_t *hdcp_context)
{
	u16 rx_status = 0;
	uint64_t *pkt = NULL;
	int err = 0;
	unsigned char nonce[HDCP_NONCE_SIZE];

	pkt = kzalloc(PKT_SIZE, GFP_KERNEL);

	if (!pkt)
		goto exit;

	nvhdcp_i2c_read16(nvhdcp, HDCP_RX_STATUS, &rx_status);
	if (nvhdcp->repeater && (rx_status & HDCP_RX_STATUS_MSG_READY_YES)) {
		err = nvhdcp_poll_ready(nvhdcp, 1000);
		if (err) {
			nvhdcp_err("Failed to get RX list\n");
			goto exit;
		}
		err = nvhdcp_receiverid_list_receive(nvhdcp,
				&hdcp_context->msg.send_receiverid_list_msg_id);
		if (err)
			goto exit;
		if (hdcp_context->msg.send_receiverid_list_msg_id !=
		ID_SEND_RCVR_ID_LIST) {
			nvhdcp_err("Not ID_SEND_RCVR_ID_LIST but %d instead\n",
				hdcp_context->msg.send_receiverid_list_msg_id);
			err = -EINVAL;
			goto exit;
		}
		if (hdcp_context->msg.rxinfo & HDCP_NON_22_RX) {
			g_fallback = 1;
			cancel_delayed_work(&nvhdcp->fallback_work);
			queue_delayed_work(nvhdcp->fallback_wq, &nvhdcp->fallback_work,
							msecs_to_jiffies(10));
			goto exit;
		}
/* If CONFIG_TUSTED_LITTLE_KERNEL flag is disabled, TSEC wil be sent a garbage SRM signature
 * resulting in HDCP authentication failure
 */
#ifdef CONFIG_TRUSTED_LITTLE_KERNEL
		/* differentiate between TLK and trusty */
		if (te_is_secos_dev_enabled()) {
			err = te_open_trusted_session_tlk(hdcp_uuid, sizeof(hdcp_uuid),
					&session_id);
		} else {
			nvhdcp->ta_ctx = NULL;
			/* Open a trusted sesion with HDCP TA */
			err = te_open_trusted_session(HDCP_PORT_NAME, &nvhdcp->ta_ctx);
		}
		if (err) {
			nvhdcp_err("Error opening trusted session\n");
			goto exit;
		}
		err = get_srm_signature(hdcp_context, nonce, pkt,
					nvhdcp->ta_ctx, HDCP_22);
		if (err) {
			nvhdcp_err("Error getting srm signature!\n");
			goto exit;
		}
#endif
		err =  tsec_hdcp_verify_vprime(hdcp_context,
				(char *)(pkt + HDCP_CMAC_OFFSET),
				*((unsigned int *)(pkt + HDCP_TSEC_ADDR_OFFSET)),
				TEGRA_NVHDCP_PORT_HDMI);
		if (err)
			goto exit;
		hdcp_context->msg.rptr_send_ack_msg_id = ID_SEND_RPTR_ACK;
		err = nvhdcp_rptr_ack_send(nvhdcp,
				&hdcp_context->msg.rptr_send_ack_msg_id);
		if (err)
			goto exit;
	} else
		err = (rx_status & HDCP_RX_STATUS_MSG_REAUTH_REQ);
exit:
	kfree(pkt);
	if (nvhdcp->ta_ctx) {
		te_close_trusted_session(nvhdcp->ta_ctx);
		nvhdcp->ta_ctx = NULL;
	}
	return err;
}

static void nvhdcp2_downstream_worker(struct work_struct *work)
{
	struct tegra_nvhdcp *nvhdcp =
		container_of(to_delayed_work(work), struct tegra_nvhdcp, work);
	struct tegra_hdmi *hdmi = nvhdcp->hdmi;
	struct tegra_dc *dc = tegra_dc_hdmi_get_dc(hdmi);
	int e;
	int ret;
	struct hdcp_context_t *hdcp_context;

	hdcp_context = kmalloc(sizeof(struct hdcp_context_t), GFP_KERNEL);
	if (!hdcp_context) {
		e = -ENOMEM;
		goto failure;
	}

	g_seq_num_m_retries = 0;

	e = tsec_hdcp_create_context(hdcp_context);
	if (e) {
		mutex_lock(&nvhdcp->lock);
		goto err;
	}

	nvhdcp_vdbg("%s():started thread for sor:%s\n", __func__, nvhdcp->name);
	tegra_dc_io_start(dc);

	mutex_lock(&nvhdcp->lock);
	if (nvhdcp->state == STATE_OFF) {
		nvhdcp_err("nvhdcp failure - giving up\n");
		goto err;
	}
	nvhdcp->state = STATE_UNAUTHENTICATED;

	/* check plug state to terminate early in case flush_workqueue() */
	if (!nvhdcp_is_plugged(nvhdcp)) {
		nvhdcp_err("worker started while unplugged!\n");
		goto lost_hdmi;
	}
	nvhdcp_vdbg("%s():hpd=%d\n", __func__, nvhdcp->plugged);
	mutex_unlock(&nvhdcp->lock);

	ret = tsec_hdcp_authentication(nvhdcp, hdcp_context);
	if (ret == HDCP_FALLBACK_1X) {
		cancel_delayed_work(&nvhdcp->fallback_work);
		queue_delayed_work(nvhdcp->fallback_wq, &nvhdcp->fallback_work,
						msecs_to_jiffies(10));
		mutex_lock(&nvhdcp->lock);
		goto lost_hdmi;
	} else if (ret) {
		mutex_lock(&nvhdcp->lock);
		goto failure;
	}

	mdelay(350);
	nvhdcp_vdbg("link integrity check ...\n");
	e = link_integrity_check(nvhdcp, hdcp_context);
	if (e) {
		nvhdcp_err("link integrity check failed err %d\n", e);
		mutex_lock(&nvhdcp->lock);
		goto failure;
	}

	mutex_lock(&nvhdcp->lock);
	nvhdcp->state = STATE_LINK_VERIFY;
	nvhdcp_info("link integrity check passed!\n");
	mutex_unlock(&nvhdcp->lock);

	down_read(&hdmi->sor->reset_lock);
	e = tsec_hdcp_session_ctrl(hdcp_context,
		HDCP_SESSION_CTRL_FLAG_ACTIVATE);
	up_read(&hdmi->sor->reset_lock);
	if (e) {
		nvhdcp_info("tsec_hdcp_session_ctrl failed\n");
		mutex_lock(&nvhdcp->lock);
		goto failure;
	}
	nvhdcp_info("HDCP 2.2 crypt enabled!\n");


	mutex_lock(&nvhdcp->lock);
	while (1) {
		if (!nvhdcp_is_plugged(nvhdcp))
			goto lost_hdmi;

		if (nvhdcp->state != STATE_LINK_VERIFY)
			goto failure;

		mutex_unlock(&nvhdcp->lock);
		e = link_integrity_check(nvhdcp, hdcp_context);
		if (e) {
			nvhdcp_err("link integrity check failed err %d\n", e);
			mutex_lock(&nvhdcp->lock);
			goto failure;
		}
		tegra_dc_io_end(dc);
		/* HDCP 2.2 Analyzer expects to check rx status more */
		/* frequently for repeaters */
		if (nvhdcp->repeater)
			wait_event_interruptible_timeout(wq_worker,
				!nvhdcp_is_plugged(nvhdcp),
				msecs_to_jiffies(300));
		else
			wait_event_interruptible_timeout(wq_worker,
				!nvhdcp_is_plugged(nvhdcp),
				msecs_to_jiffies(500));
		tegra_dc_io_start(dc);
		mutex_lock(&nvhdcp->lock);

	}

failure:
	nvhdcp->fail_count++;
	if (nvhdcp->max_retries <= HDCP_INFINITE_RETRIES ||
		nvhdcp->fail_count < nvhdcp->max_retries) {
		nvhdcp_err("nvhdcp failure - renegotiating in 1 second\n");
		if (!nvhdcp_is_plugged(nvhdcp))
			goto lost_hdmi;
		queue_delayed_work(nvhdcp->downstream_wq, &nvhdcp->work,
						msecs_to_jiffies(1000));
	} else
		nvhdcp_err("nvhdcp failure - too many failures, giving up!\n");

lost_hdmi:
	nvhdcp->state = STATE_UNAUTHENTICATED;

err:
	nvhdcp->repeater = 0;
	mutex_unlock(&nvhdcp->lock);
	tegra_dc_io_end(dc);
	e = tsec_hdcp_free_context(hdcp_context);
	kfree(hdcp_context);
	return;
}

static int tegra_nvhdcp_on(struct tegra_nvhdcp *nvhdcp)
{
	u8 hdcp2version = 0;
	int e;
	int val;
	int delay = tegra_edid_get_quirks(nvhdcp->hdmi->edid) &
			TEGRA_EDID_QUIRK_DELAY_HDCP ? 5000 : 100;
	nvhdcp->state = STATE_UNAUTHENTICATED;
	if (nvhdcp_is_plugged(nvhdcp) &&
		atomic_read(&nvhdcp->policy) !=
		TEGRA_DC_HDCP_POLICY_ALWAYS_OFF &&
		!(tegra_edid_get_quirks(nvhdcp->hdmi->edid) &
		  TEGRA_EDID_QUIRK_NO_HDCP)) {
		nvhdcp->fail_count = 0;
		e = nvhdcp_i2c_read8(nvhdcp, HDCP_HDCP2_VERSION, &hdcp2version);
		if (e)
			nvhdcp_err("nvhdcp i2c HDCP22 version read failed\n");
		/* Do not stop nauthentication if i2c version reads fail as  */
		/* HDCP 1.x test 1A-04 expects reading HDCP regs */
		if (hdcp2version & HDCP_HDCP2_VERSION_HDCP22_YES) {
			if (g_fallback) {
				val = HDCP_EESS_ENABLE<<31|
					HDCP1X_EESS_START<<16|
					HDCP1X_EESS_END;
				tegra_sor_writel_ext(nvhdcp->hdmi->sor,
					HDMI_VSYNC_WINDOW, val);
				INIT_DELAYED_WORK(&nvhdcp->work,
				nvhdcp_downstream_worker);
				nvhdcp->hdcp22 = HDCP1X_PROTOCOL;
			} else {
				val = HDCP_EESS_ENABLE<<31|
					HDCP22_EESS_START<<16|
					HDCP22_EESS_END;
				tegra_sor_writel_ext(nvhdcp->hdmi->sor,
					HDMI_VSYNC_WINDOW, val);
				INIT_DELAYED_WORK(&nvhdcp->work,
					nvhdcp2_downstream_worker);
				nvhdcp->hdcp22 = HDCP22_PROTOCOL;
			}
		} else {
			val = HDCP_EESS_ENABLE<<31|
				HDCP1X_EESS_START<<16|
				HDCP1X_EESS_END;
			tegra_sor_writel_ext(nvhdcp->hdmi->sor, HDMI_VSYNC_WINDOW,
						val);
			INIT_DELAYED_WORK(&nvhdcp->work,
				nvhdcp_downstream_worker);
			nvhdcp->hdcp22 = HDCP1X_PROTOCOL;
		}
		queue_delayed_work(nvhdcp->downstream_wq, &nvhdcp->work,
				msecs_to_jiffies(delay));
	}
	return 0;
}

static int tegra_nvhdcp_off(struct tegra_nvhdcp *nvhdcp)
{
	mutex_lock(&nvhdcp->lock);
	nvhdcp->state = STATE_OFF;
	nvhdcp_set_plugged(nvhdcp, false);
	mutex_unlock(&nvhdcp->lock);
	wake_up_interruptible(&wq_worker);
	cancel_delayed_work_sync(&nvhdcp->work);
	return 0;
}

void tegra_nvhdcp_set_plug(struct tegra_nvhdcp *nvhdcp, bool hpd)
{
	if (tegra_platform_is_sim())
		return;

	if (tegra_dc_is_t19x()) {
		uint32_t ft_info;
		/* enable HDCP only if board has SFK */
		tegra_fuse_readl(FUSE_OPT_FT_REV_0, &ft_info);
		/* only fuses with revision id greater than or equal to 0x5 have SFK */
		if (ft_info < FUSE_START_SFK) {
			nvhdcp_err("device does not have SFK fused\n");
			return;
		}
	}

	nvhdcp_debug("hdmi hotplug detected (hpd = %d)\n", hpd);

	if (hpd) {
		nvhdcp_set_plugged(nvhdcp, true);
		tegra_nvhdcp_on(nvhdcp);
	} else {
		tegra_nvhdcp_off(nvhdcp);
	}
}

int tegra_nvhdcp_set_policy(struct tegra_nvhdcp *nvhdcp, int pol)
{
	if (pol == TEGRA_DC_HDCP_POLICY_ALWAYS_ON) {
		nvhdcp_info("using \"always on\" policy.\n");
		if (atomic_xchg(&nvhdcp->policy, pol) != pol) {
			/* policy changed, start working */
			tegra_nvhdcp_on(nvhdcp);
		}
	} else if (pol == TEGRA_DC_HDCP_POLICY_ALWAYS_OFF) {
		nvhdcp_info("using \"always off\" policy.\n");
		if (atomic_xchg(&nvhdcp->policy, pol) != pol) {
			/* policy changed, stop working */
			tegra_nvhdcp_off(nvhdcp);
		}
	} else {
		/* unsupported policy */
		return -EINVAL;
	}

	return 0;
}

static int tegra_nvhdcp_renegotiate(struct tegra_nvhdcp *nvhdcp)
{
	mutex_lock(&nvhdcp->lock);
	nvhdcp->state = STATE_RENEGOTIATE;
	mutex_unlock(&nvhdcp->lock);
	tegra_nvhdcp_on(nvhdcp);
	return 0;
}

void tegra_nvhdcp_suspend(struct tegra_nvhdcp *nvhdcp)
{
	if (!nvhdcp)
		return;
	tegra_nvhdcp_off(nvhdcp);
}

void tegra_nvhdcp_resume(struct tegra_nvhdcp *nvhdcp)
{
	if (!nvhdcp)
		return;
	tegra_nvhdcp_renegotiate(nvhdcp);
}

void tegra_nvhdcp_shutdown(struct tegra_nvhdcp *nvhdcp)
{
	if (!nvhdcp)
		return;
	tegra_nvhdcp_off(nvhdcp);
}

static int tegra_nvhdcp_recv_capable(struct tegra_nvhdcp *nvhdcp)
{
	if (!nvhdcp)
		return 0;

	if (nvhdcp->state == STATE_LINK_VERIFY)
		return 1;
	else {
		__u64 b_ksv;
		/* get Bksv from receiver */
		if (!nvhdcp_i2c_read40(nvhdcp, 0x00, &b_ksv))
			return !verify_ksv(b_ksv);
	}
	return 0;
}

static long nvhdcp_dev_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	struct tegra_nvhdcp *nvhdcp = filp->private_data;
	struct tegra_nvhdcp_packet *pkt;
	int e = -ENOTTY;

	switch (cmd) {
	case TEGRAIO_NVHDCP_ON:
		return tegra_nvhdcp_on(nvhdcp);

	case TEGRAIO_NVHDCP_OFF:
		return tegra_nvhdcp_off(nvhdcp);

	case TEGRAIO_NVHDCP_SET_POLICY:
		return tegra_nvhdcp_set_policy(nvhdcp, arg);

	case TEGRAIO_NVHDCP_READ_M:
		pkt = kmalloc(sizeof(*pkt), GFP_KERNEL);
		if (!pkt)
			return -ENOMEM;
		if (copy_from_user(pkt, (void __user *)arg, sizeof(*pkt))) {
			e = -EFAULT;
			goto kfree_pkt;
		}
		e = get_m_prime(nvhdcp, pkt);
		if (copy_to_user((void __user *)arg, pkt, sizeof(*pkt))) {
			e = -EFAULT;
			goto kfree_pkt;
		}
		kfree(pkt);
		return e;

	case TEGRAIO_NVHDCP_READ_S:
		pkt = kmalloc(sizeof(*pkt), GFP_KERNEL);
		if (!pkt)
			return -ENOMEM;
		if (copy_from_user(pkt, (void __user *)arg, sizeof(*pkt))) {
			e = -EFAULT;
			goto kfree_pkt;
		}
		e = get_s_prime(nvhdcp, pkt);
		if (copy_to_user((void __user *)arg, pkt, sizeof(*pkt))) {
			e = -EFAULT;
			goto kfree_pkt;
		}
		kfree(pkt);
		return e;

	case TEGRAIO_NVHDCP_RENEGOTIATE:
		e = tegra_nvhdcp_renegotiate(nvhdcp);
		break;

	case TEGRAIO_NVHDCP_HDCP_STATE:
		pkt = kmalloc(sizeof(*pkt), GFP_KERNEL);
		if (!pkt)
			return -ENOMEM;
		e = get_nvhdcp_state(nvhdcp, pkt);
		if (copy_to_user((void __user *)arg, pkt, sizeof(*pkt))) {
			e = -EFAULT;
			goto kfree_pkt;
		}
		kfree(pkt);
		return e;

	case TEGRAIO_NVHDCP_RECV_CAPABLE:
		{
			__u32 recv_capable = tegra_nvhdcp_recv_capable(nvhdcp);
			if (copy_to_user((void __user *)arg, &recv_capable,
				sizeof(recv_capable)))
				return -EFAULT;
			return 0;
		}
	}

	return e;
kfree_pkt:
	kfree(pkt);
	return e;
}

static int nvhdcp_dev_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *miscdev = filp->private_data;
	struct tegra_nvhdcp *nvhdcp =
		container_of(miscdev, struct tegra_nvhdcp, miscdev);
	filp->private_data = nvhdcp;
	return 0;
}

static int nvhdcp_dev_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;
	return 0;
}

static const struct file_operations nvhdcp_fops = {
	.owner          = THIS_MODULE,
	.llseek         = no_llseek,
	.unlocked_ioctl = nvhdcp_dev_ioctl,
	.open           = nvhdcp_dev_open,
	.release        = nvhdcp_dev_release,
#ifdef CONFIG_COMPAT
	.compat_ioctl   = nvhdcp_dev_ioctl,
#endif
};

static struct tegra_nvhdcp **nvhdcp_head;

/* we only support one AP right now, so should only call this once. */
struct tegra_nvhdcp *tegra_nvhdcp_create(struct tegra_hdmi *hdmi,
			int id, int bus)
{
	struct tegra_nvhdcp *nvhdcp;
	struct i2c_adapter *adapter;
	int e;
	int num_heads;

	num_heads = tegra_dc_get_numof_dispheads();

	if (id >= num_heads) {
		nvhdcp_err("head id greater than what's available!");
		return ERR_PTR(-EMFILE);
	}

	if (!nvhdcp_head) {
		nvhdcp_head =
		kzalloc(sizeof(void *) * num_heads, GFP_KERNEL);
		if (!nvhdcp_head)
			return ERR_PTR(-ENOMEM);
	}

	nvhdcp = nvhdcp_head[id];
	/* do not allow duplicate node creation */
	if (nvhdcp)
		return ERR_PTR(-EMFILE);

	nvhdcp = kzalloc(sizeof(*nvhdcp), GFP_KERNEL);
	if (!nvhdcp)
		return ERR_PTR(-ENOMEM);

	nvhdcp->id = id;
	snprintf(nvhdcp->name, sizeof(nvhdcp->name), "nvhdcp%u", id);
	nvhdcp->hdmi = hdmi;
	mutex_init(&nvhdcp->lock);

	strlcpy(nvhdcp->info.type, nvhdcp->name, sizeof(nvhdcp->info.type));
	nvhdcp->bus = bus;
	nvhdcp->info.addr = 0x74 >> 1;
	nvhdcp->info.platform_data = nvhdcp;
	nvhdcp->fail_count = 0;
	nvhdcp->max_retries = HDCP_INFINITE_RETRIES;
	atomic_set(&nvhdcp->policy, hdmi->dc->pdata->default_out->hdcp_policy);

	adapter = i2c_get_adapter(bus);
	if (!adapter) {
		nvhdcp_err("can't get adapter for bus %d\n", bus);
		e = -EBUSY;
		goto free_nvhdcp;
	}

	nvhdcp->client = i2c_new_device(adapter, &nvhdcp->info);
	i2c_put_adapter(adapter);

	if (!nvhdcp->client) {
		nvhdcp_err("can't create new device\n");
		e = -EBUSY;
		goto free_nvhdcp;
	}

	nvhdcp->state = STATE_UNAUTHENTICATED;

	nvhdcp->downstream_wq = create_singlethread_workqueue(nvhdcp->name);
	nvhdcp->fallback_wq = create_singlethread_workqueue(nvhdcp->name);

	INIT_DELAYED_WORK(&nvhdcp->fallback_work, nvhdcp_fallback_worker);

	nvhdcp->miscdev.minor = MISC_DYNAMIC_MINOR;
	nvhdcp->miscdev.name = nvhdcp->name;
	nvhdcp->miscdev.fops = &nvhdcp_fops;

	e = misc_register(&nvhdcp->miscdev);
	if (e)
		goto free_workqueue;

	nvhdcp_head[id] = nvhdcp;
	nvhdcp_vdbg("%s(): created misc device %s\n", __func__, nvhdcp->name);

	return nvhdcp;
free_workqueue:
	destroy_workqueue(nvhdcp->downstream_wq);
	destroy_workqueue(nvhdcp->fallback_wq);
	i2c_release_client(nvhdcp->client);
free_nvhdcp:
	kfree(nvhdcp);
	nvhdcp_err("unable to create device.\n");
	return ERR_PTR(e);
}

void tegra_nvhdcp_destroy(struct tegra_nvhdcp *nvhdcp)
{
	misc_deregister(&nvhdcp->miscdev);
	tegra_nvhdcp_off(nvhdcp);
	destroy_workqueue(nvhdcp->downstream_wq);
	destroy_workqueue(nvhdcp->fallback_wq);
	i2c_unregister_device(nvhdcp->client);
	nvhdcp_head[nvhdcp->id] = NULL;
	kfree(nvhdcp);
}

#ifdef CONFIG_TEGRA_DEBUG_HDCP
/* show current maximum number of retries for HDCP authentication */
static int tegra_nvhdcp_max_retries_dbg_show(struct seq_file *m, void *unused)
{
	struct tegra_nvhdcp *hdcp = m->private;

	if (WARN_ON(!hdcp))
		return -EINVAL;

	seq_printf(m, "hdcp max_retries value: %d\n", hdcp->max_retries);

	return 0;
}

/*
 * sw control for hdcp max retries.
 * -1 is the default value which indicates infinite retries
 * If a non-infinite value is desired, set max_retries to a non-negative
 *  integer
 * 0 is the minimum number of retries.
 */
static ssize_t tegra_nvhdcp_max_retries_dbg_write(struct file *file,
						const char __user *addr,
						size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data;
	struct tegra_nvhdcp *hdcp = m->private;
	u8 new_max_retries;
	int ret;

	if (WARN_ON(!hdcp))
		return -EINVAL;

	ret = kstrtou8_from_user(addr, len, 6, &new_max_retries);
	if (ret < 0)
		return ret;

	hdcp->max_retries = new_max_retries;

	return len;
}

static int tegra_nvhdcp_max_retries_dbg_open(struct inode *inode,
		struct file *file)
{
	return single_open(file, tegra_nvhdcp_max_retries_dbg_show,
			inode->i_private);
}

static const struct file_operations tegra_nvhdcp_max_retries_dbg_ops = {
	.open = tegra_nvhdcp_max_retries_dbg_open,
	.read = seq_read,
	.write = tegra_nvhdcp_max_retries_dbg_write,
	.llseek = seq_lseek,
	.release = single_release,
};

void tegra_nvhdcp_debugfs_init(struct tegra_nvhdcp *nvhdcp)
{
	struct dentry *dir, *ret;

	dir = debugfs_create_dir("tegra_nvhdcp",  NULL);
	if (IS_ERR_OR_NULL(dir))
		return;

	ret = debugfs_create_file("max_retries", 0444, dir,
				nvhdcp, &tegra_nvhdcp_max_retries_dbg_ops);
	if (IS_ERR_OR_NULL(ret))
		goto fail;

	return;
fail:
	debugfs_remove_recursive(dir);
	return;
}
#else
void tegra_nvhdcp_debugfs_init(struct tegra_nvhdcp *nvhdcp)
{
	return;
}
#endif
