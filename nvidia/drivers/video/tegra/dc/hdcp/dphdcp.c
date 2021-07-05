/*
 * dphdcp.c: dp hdcp driver.
 *
 * Copyright (c) 2015-2020, NVIDIA CORPORATION, All rights reserved.
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
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/sched.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>
#include <linux/atomic.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/tsec.h>
#include <soc/tegra/kfuse.h>
#include <soc/tegra/fuse.h>

#include "dc.h"
#include "dphdcp.h"
#include "dp.h"
#include "dpaux.h"
#include "edid.h"
#include "sor.h"
#include "sor_regs.h"
#include "dpaux_regs.h"
#include "tsec_drv.h"
#include "tsec/tsec_methods.h"
#include "nvhdcp_hdcp22_methods.h"
#include "tsec/tsec.h"

#if (defined(CONFIG_TRUSTY))
#include <linux/trusty/trusty_ipc.h>
#endif

static DECLARE_WAIT_QUEUE_HEAD(wq_worker);

/* Bcaps register bits */
#define BCAPS_REPEATER			(1 << 1)
#define BCAPS_HDCP_CAPABLE		(1 << 0)

/* Bstatus register bits */
#define BSTATUS_REAUTH_REQ		(1 << 3)
#define BSTATUS_LINK_INTEG_FAIL		(1 << 2)
#define BSTATUS_R0_PRIME_SET		(1 << 1)
#define BSTATUS_READY			(1 << 0)

/* Binfo register bits */
#define BINFO_MAX_DEVS_EXCEEDED		(1 << 7)
#define BINFO_MAX_CASCADE_EXCEEDED	(1 << 11)

/* for hdcp 2.2 */
#define HDCP22_PROTOCOL			1
#define HDCP1X_PROTOCOL			0
#define HDCP_DEBUG			1
#define HDCP_READY			1
#define HDCP_REAUTH			2
#define HDCP_READY_SET			(1 << 0)
#define HDCP_HPRIME_AVAIL		(1 << 1)
#define HDCP_PAIRING_AVAIL		(1 << 2)
#define HDCP_REAUTH_MASK		(1 << 3)
#define HDCP_LINK_INTEG_FAIL		(1 << 4)

#define HDCP_TA_CMD_CTRL		0
#define HDCP_TA_CMD_AKSV		1
#define HDCP_TA_CMD_ENC			2
#define HDCP_TA_CMD_REP			3
#define HDCP_TA_CMD_BKSV		4

#define PKT_SIZE			256
#define HDCP_AUTH_CMD			0x5

#define HDCP_TA_CTRL_ENABLE		1
#define HDCP_TA_CTRL_DISABLE		0

#define HDCP_CMD_OFFSET			1
#define HDCP_CMD_BYTE_OFFSET		8


#define MAX_AUX_SIZE			15

#define SIZE_ONE_BYTE			1
#define SIZE_TWO_BYTES			2
#define SIZE_FIVE_BYTES			5
#define SIZE_EIGHT_BYTES		8

#define KEY_CTRL_RETRIES		101
#define HDCP_CTRL_RETRIES		13
#define SRAM_CLR_RETRIES		6
#define RX_VALIDATE_RETRIES		3
#define VPRIME_RETRIES			3
#define KSV_RETRIES			10
#define MAX_BYTES_READ			15
#define REPEATER_READY_RETRY		51

#define HDCP_KEY_LOAD			0x100
#define KFUSE_MASK			0x10

#define HDCP11_SRM_PATH			"vendor/etc/hdcpsrm/hdcp1x.srm"

#define CP_IRQ_OFFSET			(1 << 2)
#define CP_IRQ_RESET			0x4

/* logging */
#ifdef VERBOSE_DEBUG
#define dphdcp_vdbg(...)	\
		pr_debug("dphdcp: " __VA_ARGS__)
#else
#define dphdcp_vdbg(...)		\
({						\
	if (0)					\
		pr_debug("dphdcp: " __VA_ARGS__); \
	0;					\
})
#endif
#define dphdcp_debug(...)	\
		pr_debug("dphdcp: " __VA_ARGS__)
#define	dphdcp_err(...)	\
		pr_err("dphdcp: Error: " __VA_ARGS__)
#define dphdcp_info(...)	\
		pr_info("dphdcp: " __VA_ARGS__)

#define HDCP_PORT_NAME	"com.nvidia.tos.13f616f9-8572-4a6f-a1f104aa9b05f9ff"

static bool repeater_flag;
static bool vprime_check_done;
static struct tegra_dphdcp **dphdcp_head;

static int tegra_dphdcp_read(struct tegra_dc_dp_data *dp, u32 cmd,
	u8 *data_ptr, u32 size, u32 *aux_status)
{
	u32 status = 0;
	u32 cursize = 0;
	int ret = 0;
	struct tegra_dc_dpaux_data *dpaux = NULL;

	if (!dp || !data_ptr || !aux_status) {
		dphdcp_err("Null params sent\n");
		return -EINVAL;
	}

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return -EIO;

	dpaux = dp->dpaux;
	cursize = size;
	mutex_lock(&dpaux->lock);
	tegra_dpaux_get(dp->dpaux);
	ret = tegra_dc_dpaux_read_chunk_locked(dpaux, DPAUX_DP_AUXCTL_CMD_AUXRD,
		cmd, data_ptr, &cursize, &status);
	tegra_dpaux_put(dp->dpaux);
	mutex_unlock(&dpaux->lock);
	if (ret)
		dev_err(&dp->dc->ndev->dev,
			"dp: Failed to read data. CMD 0x%x, Status 0x%x\n",
			cmd, status);
	*aux_status = status;
	return ret;
}

static int tegra_dphdcp_write(struct tegra_dc_dp_data *dp, u32 cmd,
	u8 *data, u32 size)
{
	u32 status = 0;
	u32 cursize = 0;
	int ret;
	struct tegra_dc_dpaux_data *dpaux = NULL;

	if (!dp || !data) {
		dphdcp_err("Null params sent\n");
		return -EINVAL;
	}

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return -EIO;

	dpaux = dp->dpaux;
	cursize = size;
	mutex_lock(&dpaux->lock);
	tegra_dpaux_get(dp->dpaux);
	ret = tegra_dc_dpaux_write_chunk_locked(dpaux,
			DPAUX_DP_AUXCTL_CMD_AUXWR, cmd, data,
			&cursize, &status);
	tegra_dpaux_put(dp->dpaux);
	mutex_unlock(&dpaux->lock);
	if (ret)
		dev_err(&dp->dc->ndev->dev,
			"dp: Failed to write data. CMD 0x%x, Status 0x%x\n",
			cmd, status);
	return ret;
}

/* read 5 bytes of data */
static int tegra_dphdcp_read40(struct tegra_dc_dp_data *dp, u32 cmd,
			u64 *data)
{
	u8 buf[SIZE_FIVE_BYTES];
	int i;
	u64 n;
	int e;
	u32 status;

	if (!dp || !data) {
		dphdcp_err("Null params sent\n");
		return -EINVAL;
	}

	e = tegra_dphdcp_read(dp, cmd, buf, sizeof(buf), &status);
	if (e)
		return e;

	/* assign the value read from aux to data */
	for (i = 0, n = 0; i < 5; i++) {
		n <<= 8;
		n |= buf[4 - i];
	}
	if (data)
		*data = n;
	return 0;
}

/* read 2 bytes of data */
static int tegra_dphdcp_read16(struct tegra_dc_dp_data *dp, u32 cmd,
				u64 *data)
{
	u8 buf[SIZE_TWO_BYTES];
	int e;
	u32 status;

	if (!dp || !data) {
		dphdcp_err("Null params sent\n");
		return -EINVAL;
	}

	e = tegra_dphdcp_read(dp, cmd, buf, sizeof(buf), &status);
	if (e)
		return e;

	if (data)
		*data = buf[0] | (u16)buf[1] << 8;
	return 0;
}

/* write 8 bytes of data */
static int tegra_dphdcp_write64(struct tegra_dc_dp_data *dp, u32 reg,
	u64 *data)
{
	char buf[SIZE_EIGHT_BYTES];

	if (!dp || !data) {
		dphdcp_err("Null params sent\n");
		return -EINVAL;
	}

	memcpy(buf, (char *)data, sizeof(buf));
	return tegra_dphdcp_write(dp, reg, buf, sizeof(buf));
}

/* write 1 byte of data */
static int tegra_dphdcp_write8(struct tegra_dc_dp_data *dp, u32 reg,
	u8 data)
{
	char buf[SIZE_ONE_BYTE];
	u8 cur_data;

	if (!dp) {
		dphdcp_err("Null params sent\n");
		return -EINVAL;
	}

	cur_data = data;

	memcpy(buf, (char *)&cur_data, sizeof(buf));
	return tegra_dphdcp_write(dp, reg, buf, sizeof(buf));
}

/* write 5 bytes of data */
static int tegra_dphdcp_write40(struct tegra_dc_dp_data *dp, u32 reg,
	u64 *data)
{
	char buf[SIZE_FIVE_BYTES];

	if (!dp || !data) {
		dphdcp_err("Null params sent\n");
		return -EINVAL;
	}

	memcpy(buf, data, sizeof(buf));
	return tegra_dphdcp_write(dp, reg, buf, sizeof(buf));
}

/*
 * wait for bits in mask to be set to value in NV_SOR_KEY_CTRL
 * waits upto 100 ms
 */
static int wait_key_ctrl(struct tegra_dc_sor_data *sor, u32 mask, u32 value)
{
	int retries = KEY_CTRL_RETRIES;
	u32 ctrl;

	if (!sor) {
		dphdcp_err("Null params sent\n");
		return -EINVAL;
	}

	do {
		usleep_range(1, 2);
		ctrl = tegra_sor_readl_ext(sor, NV_SOR_KEY_CTRL);
		if (((ctrl ^ value) & mask) == 0)
			break;
	} while (--retries);
	if (!retries) {
		dphdcp_err("key ctrl read timeout (mask=0x%x)\n", mask);
		return -EIO;
	}
	return 0;
}

/* set or clear RUN_YES */
static void hdcp_ctrl_run(struct tegra_dc_sor_data *sor, bool v)
{
	u32 ctrl;

	if (!sor) {
		dphdcp_err("Null params sent\n");
		return;
	}

	if (v) {
		ctrl = tegra_sor_readl_ext(sor, NV_SOR_DP_HDCP_CTRL);
		ctrl |= HDCP_RUN_YES;
	} else {
		ctrl = 0;
	}

	tegra_sor_writel_ext(sor, NV_SOR_DP_HDCP_CTRL, ctrl);
}

/*
 * wait for any bits in mask to be set in NV_SOR_DP_HDCP_CTRL
 * sleeps up to 120 ms
 */
static int wait_hdcp_ctrl(struct tegra_dc_sor_data *sor, u32 mask, u32 *v)
{
	int retries = HDCP_CTRL_RETRIES;
	u32 ctrl;

	if (!sor) {
		dphdcp_err("Null params sent\n");
		return -EINVAL;
	}

	do {
		ctrl = tegra_sor_readl_ext(sor, NV_SOR_DP_HDCP_CTRL);
		if ((ctrl & mask)) {
			if (v)
				*v = ctrl;
			break;
		}
		if (retries > 1)
			usleep_range(10, 15);
	} while (--retries);
	if (!retries) {
		dphdcp_err("ctrl read timeout (mask=0x%x)\n", mask);
		return -EIO;
	}
	return 0;
}

/*
 * check if the KSV values returned are valid,
 * i.e a combination of 20 ones and 20 zeroes
 */
static int verify_ksv(u64 k)
{
	unsigned i;
	/* count set bits, must be exactly 20 set to be valid */
	for (i = 0; k; i++)
		k ^= k & -k;

	return  (i != 20) ? -EINVAL : 0;
}

/* 64-bit link encryption session random number */
static inline u64 get_an(struct tegra_dc_sor_data *sor)
{
	u64 r;

	if (!sor) {
		dphdcp_err("Null params sent!\n");
		return -EINVAL;
	}

	r = (u64)tegra_sor_readl_ext(sor, NV_SOR_DP_HDCP_AN_MSB) << 32;
	r |= tegra_sor_readl_ext(sor, NV_SOR_DP_HDCP_AN_LSB);
	return r;
}

/* 40-bit transmitter's key selection vector */
static inline u64 get_aksv(struct tegra_dc_sor_data *sor)
{
	u64 r;

	if (!sor) {
		dphdcp_err("Null params sent!\n");
		return -EINVAL;
	}

	r = (u64)tegra_sor_readl_ext(sor, NV_SOR_DP_HDCP_AKSV_MSB) << 32;
	r |= tegra_sor_readl_ext(sor, NV_SOR_DP_HDCP_AKSV_LSB);
	return r;
}

/* 40-bit receiver's key selection vector */
static inline void set_bksv(struct tegra_dc_sor_data *sor, u64 b_ksv,
							bool repeater)
{
	if (sor) {
		if (repeater)
			b_ksv |= (u64)REPEATER << 32;
		tegra_sor_writel_ext(sor, NV_SOR_DP_HDCP_BKSV_LSB, (u32)b_ksv);
		tegra_sor_writel_ext(sor, NV_SOR_DP_HDCP_BKSV_MSB, b_ksv >> 32);
	}
}

static int get_bcaps(struct tegra_dc_dp_data *dp, u8 *b_caps)
{
	u32 status;

	if (!dp || !b_caps) {
		dphdcp_err("Null params sent!\n");
		return -EINVAL;
	}

	return tegra_dphdcp_read(dp, NV_DPCD_HDCP_BCAPS_OFFSET,
						b_caps, 1, &status);
}

static int get_bstatus(struct tegra_dc_dp_data *dp, u8 *bstatus)
{
	u32 status;

	if (!dp || !bstatus) {
		dphdcp_err("Null params sent!\n");
		return -EINVAL;
	}

	return tegra_dphdcp_read(dp, NV_DPCD_HDCP_BSTATUS_OFFSET,
						bstatus, 1, &status);
}

static int get_irq_status(struct tegra_dc_dp_data *dp, u8 *irq_status)
{
	u32 status;

	if (!dp || !irq_status) {
		dphdcp_err("Null params sent!\n");
		return -EINVAL;
	}

	return tegra_dphdcp_read(dp, NV_DPCD_DEVICE_SERVICE_IRQ_VECTOR,
				irq_status, 1, &status);
}

static inline bool dphdcp_is_plugged(struct tegra_dphdcp *dphdcp)
{
	rmb();
	if (dphdcp)
		return dphdcp->plugged;
	return false;
}

static inline bool dphdcp_set_plugged(struct tegra_dphdcp *dphdcp,
						bool plugged)
{
	if (dphdcp) {
		dphdcp->plugged = plugged;
		wmb();
	}
	return plugged;
}

static int load_kfuse(struct tegra_dc_dp_data *dp)
{
	u32 ctrl;
	u32 tmp;
	int retries;
	int e;
	int i;
	unsigned buf[KFUSE_DATA_SZ/4];
	struct tegra_dc_sor_data *sor;

	if (!dp) {
		dphdcp_err("Null params sent!\n");
		return -EINVAL;
	}

	sor = dp->sor;

	memset(buf, 0, sizeof(buf));

	/* load kfuse */
	dphdcp_vdbg("loading kfuse\n");
	/* copy load kfuse into buffer - only needed for early Tegra parts */
	e = tegra_kfuse_read(buf, sizeof(buf));
	if (e) {
		dphdcp_err("Kfuse read failure\n");
		return e;
	}

	/* write the kfuse to the DP SRAM */
	tegra_sor_writel_ext(sor, NV_SOR_KEY_CTRL, 1);

	/* issue a reload */
	ctrl = tegra_sor_readl_ext(sor, NV_SOR_KEY_CTRL);
	tegra_sor_writel_ext(sor, NV_SOR_KEY_CTRL, ctrl | PKEY_RELOAD_TRIGGER
					| LOCAL_KEYS);
	e = wait_key_ctrl(sor, PKEY_LOADED, PKEY_LOADED);
	if (e) {
		dphdcp_err("key reload timeout\n");
		return e;
	}

	tegra_sor_writel_ext(sor, NV_SOR_KEY_SKEY_INDEX, 0);

	/* wait for SRAM to be cleared */
	retries = SRAM_CLR_RETRIES;
	do {
		tmp = tegra_sor_readl_ext(sor, NV_SOR_KEY_DEBUG0);
		if ((tmp & 1) == 0)
			break;
		if (retries > 1)
			mdelay(1);
	} while (--retries);
	if (!retries) {
		dphdcp_err("key SRAM clear timeout\n");
		return -EIO;
	}

	for (i = 0; i < KFUSE_DATA_SZ / 4; i += 4) {

		/* load 128-bits*/
		tegra_sor_writel_ext(sor, NV_SOR_KEY_HDCP_KEY_0, buf[i]);
		tegra_sor_writel_ext(sor, NV_SOR_KEY_HDCP_KEY_1, buf[i+1]);
		tegra_sor_writel_ext(sor, NV_SOR_KEY_HDCP_KEY_2, buf[i+2]);
		tegra_sor_writel_ext(sor, NV_SOR_KEY_HDCP_KEY_3, buf[i+3]);

		/* trigger LOAD_HDCP_KEY */
		tegra_sor_writel_ext(sor,
			NV_SOR_KEY_HDCP_KEY_TRIG, HDCP_KEY_LOAD);
		tmp = LOCAL_KEYS | WRITE16;
		if (i)
			tmp |= AUTOINC;
		tegra_sor_writel_ext(sor, NV_SOR_KEY_CTRL, tmp);

		/* wait for WRITE16 to complete */
		e = wait_key_ctrl(sor, KFUSE_MASK, 0); /* WRITE16 */
		if (e) {
			dphdcp_err("key write timeout\n");
			return -EIO;
		}
	}
	return 0;
}

/* check the 16 bit link integrity value */
static inline u64 get_transmitter_ro_prime(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc_sor_data *sor;

	if (!dp) {
		dphdcp_err("Null params sent!\n");
		return -EINVAL;
	}

	sor = dp->sor;
	return tegra_sor_readl_ext(sor, NV_SOR_DP_HDCP_RI);
}

/* R0' prime value generated from the receiver */
static inline int get_receiver_ro_prime(struct tegra_dc_dp_data *dp, u64 *r)
{
	if (!dp || !r) {
		dphdcp_err("Null params sent!\n");
		return -EINVAL;
	}
	return tegra_dphdcp_read16(dp, NV_DPCD_HDCP_RPRIME_OFFSET, r);
}

static int validate_rx(struct tegra_dphdcp *dphdcp)
{
	int retries = RX_VALIDATE_RETRIES;
	u64 rx = 0, tx = 0;
	int e;
	struct tegra_dc_dp_data *dp;

	if (!dphdcp) {
		dphdcp_err("Null params sent!\n");
		return -EINVAL;
	}

	dp = dphdcp->dp;

	/* try 3 times for possible link errors */
	do {
		tx = get_transmitter_ro_prime(dp);
		e = get_receiver_ro_prime(dp, &rx);
	} while (--retries && rx != tx);
	dphdcp_vdbg("rx=0x%016llx tx=0x%016llx\n", rx, tx);

	if (rx != tx)
		return -EINVAL;
	return 0;
}

/* get V' 160-bit SHA-1 hash from repeater */
static int get_vprime(struct tegra_dc_dp_data *dp, u8 *v_prime)
{
	int e, i;
	u32 status;

	if (!dp || !v_prime) {
		dphdcp_err("Null params sent!\n");
		return -EINVAL;
	}

	for (i = 0; i < 20; i += 4) {
		e = tegra_dphdcp_read(dp,
			NV_DPCP_HDCP_SHA_H0_OFFSET+i, v_prime+i, 4, &status);
		if (e) {
			dphdcp_err("Error reading V'\n");
			return e;
		}
	}
	return 0;
}

static int get_ksvfifo(struct tegra_dc_dp_data *dp,
			unsigned num_bksv_list, u64 *ksv_list)
{
	u8 *buf = NULL;
	u8 *orig_buf = NULL;
	int e;
	unsigned int dp_retries = KSV_RETRIES;
	u32 status;
	size_t buf_len = num_bksv_list * SIZE_FIVE_BYTES;


	if (!ksv_list || !dp || num_bksv_list > TEGRA_NVHDCP_MAX_DEVS)
		return -EINVAL;

	if (num_bksv_list == 0)
		return 0;

	buf = kmalloc(buf_len, GFP_KERNEL);
	if (IS_ERR_OR_NULL(buf))
		return -ENOMEM;

	orig_buf = buf;
	while (buf_len > MAX_BYTES_READ) {
aux_read:
		e = tegra_dphdcp_read(dp,
		NV_DPCD_HDCP_KSV_FIFO_OFFSET, buf, MAX_BYTES_READ, &status);
		if (e) {
			dphdcp_err("Error reading KSV\n");
			kfree(orig_buf);
			return e;
		}
		if ((status & DPAUX_DP_AUXSTAT_REPLYTYPE_I2CDEFER) ||
		(status & DPAUX_DP_AUXSTAT_REPLYTYPE_DEFER)) {
			if (--dp_retries)
				goto aux_read;
		}
		buf_len -= MAX_BYTES_READ;
		buf += MAX_BYTES_READ;
	}

	if (buf_len)
		e = tegra_dphdcp_read(dp,
		NV_DPCD_HDCP_KSV_FIFO_OFFSET, buf, buf_len, &status);
	if (e) {
		dphdcp_err("Error reading KSV\n");
		kfree(orig_buf);
		return e;
	}
	memcpy(ksv_list, orig_buf, num_bksv_list*5);
	kfree(orig_buf);
	return 0;
}

/* validate srm signature */
static int get_srm_signature(struct hdcp_context_t *hdcp_context,
			char *nonce, uint64_t *pkt, void *ta_ctx)
{
	int err = 0;

	if (!hdcp_context || !nonce || !pkt || !ta_ctx) {
		dphdcp_err("Null params sent!\n");
		return err;
	}
	/* generate nonce in the ucode */
	err = tsec_hdcp_generate_nonce(hdcp_context, nonce);
	if (err) {
		dphdcp_err("Error generating nonce!\n");
		return err;
	}
	/* pass the nonce to hdcp TA and get the signature back */
	memcpy(pkt, nonce, HDCP_NONCE_SIZE);
	*(pkt + HDCP_NONCE_SIZE) = HDCP_1x;
	err = te_launch_trusted_oper(pkt, PKT_SIZE, HDCP_CMD_GEN_CMAC, ta_ctx);
	if (err)
		dphdcp_err("te launch operation failed with error %d\n", err);
	return err;
}

/* SRM revocation check for receiver */
static int srm_revocation_check(struct tegra_dphdcp *dphdcp)
{
	struct hdcp_context_t *hdcp_context =
		kmalloc(sizeof(struct hdcp_context_t), GFP_KERNEL);
	int e = 0;
	unsigned char nonce[HDCP_NONCE_SIZE];

	uint64_t *pkt = kzalloc(PKT_SIZE, GFP_KERNEL);

	if (!pkt || !hdcp_context)
		goto exit;

	e = tsec_hdcp_context_creation(hdcp_context, DISPLAY_TYPE_DP,
			dphdcp->dp->sor->ctrl_num);
	if (e) {
		dphdcp_err("hdcp context create/init failed\n");
		goto exit;
	}

	e =  tsec_hdcp_create_session(hdcp_context, DISPLAY_TYPE_DP,
				dphdcp->dp->sor->ctrl_num);
	if (e) {
		dphdcp_err("error in session creation\n");
		goto exit;
	}
	e = get_srm_signature(hdcp_context, nonce, pkt, dphdcp->ta_ctx);
	if (e) {
		dphdcp_err("Error getting srm signature!\n");
		goto exit;
	}
	e = tsec_hdcp_revocation_check(hdcp_context,
			(unsigned char *)(pkt + HDCP_CMAC_OFFSET),
			*((unsigned int *)(pkt + HDCP_TSEC_ADDR_OFFSET)),
			TEGRA_NVHDCP_PORT_DP, HDCP_1x);

	if (e)
		dphdcp_err("hdcp revocation check failed with err: %x\n", e);
exit:
	tsec_hdcp_free_context(hdcp_context);
	kfree(hdcp_context);
	kfree(pkt);
	return e;
}

/* vprime verification for repeater */
static int tsec_hdcp_dp_verify_vprime(struct tegra_dphdcp *dphdcp)
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

	e = tsec_hdcp_context_creation(hdcp_context, DISPLAY_TYPE_DP,
			dphdcp->dp->sor->ctrl_num);
	if (e) {
		dphdcp_err("hdcp context create/init failed\n");
		goto exit;
	}
	pkt = kzalloc(PKT_SIZE, GFP_KERNEL);

	if (!pkt || !hdcp_context)
		goto exit;
	e = get_srm_signature(hdcp_context, nonce, pkt, dphdcp->ta_ctx);
	if (e) {
		dphdcp_err("Error getting srm signature!\n");
		goto exit;
	}

	memset(&verify_vprime_param, 0x0,
		sizeof(struct hdcp_verify_vprime_param));

	/* convert 64 bit values to 40 bit */
	p = buf;
	for (i = 0; i < dphdcp->num_bksv_list; i++) {
		p[0] = (u8)(dphdcp->bksv_list[i] & 0xff);
		p[1] = (u8)((dphdcp->bksv_list[i]>>8) & 0xff);
		p[2] = (u8)((dphdcp->bksv_list[i]>>16) & 0xff);
		p[3] = (u8)((dphdcp->bksv_list[i]>>24) & 0xff);
		p[4] = (u8)((dphdcp->bksv_list[i]>>32) & 0xff);
		p += 5;
	}

	memcpy((void *)verify_vprime_param.vprime, dphdcp->v_prime,
			HDCP_SIZE_VPRIME_1X_8);
	verify_vprime_param.port = TEGRA_NVHDCP_PORT_DP; /* hdcp 1.x */
	verify_vprime_param.bstatus = dphdcp->binfo;
	e = tsec_hdcp1x_verify_vprime(verify_vprime_param, hdcp_context,
		buf, dphdcp->num_bksv_list, pkt);

exit:
	tsec_hdcp_free_context(hdcp_context);
	kfree(pkt);
	kfree(hdcp_context);
	return e;
}

static int get_repeater_info(struct tegra_dphdcp *dphdcp)
{
	int e = 0;
	unsigned int retries;
	int vcheck_tries = VPRIME_RETRIES;
	u8 bstatus;
	u64 binfo;
	u8 irq;
	int err = 0;
	struct tegra_dc_dp_data *dp = dphdcp->dp;

	dphdcp_vdbg("repeater found:fetching repeater info\n");
	/* wait up to 5 seconds for READY on repeater */
	retries = REPEATER_READY_RETRY;
	do {
		mutex_lock(&dphdcp->lock);
		e = dphdcp_is_plugged(dphdcp);
		mutex_unlock(&dphdcp->lock);
		if (!e) {
			dphdcp_err("disconnect while waiting for repeater\n");
			return -EIO;
		}
		/* wait till receiver computes V' */
		e = get_bstatus(dp, &bstatus);
		if (!e && (bstatus & BSTATUS_READY)) {
			dphdcp_vdbg("Bstatus READY from repeater\n");
			break;
		}
		if (retries > 1)
			msleep(100);
	} while (--retries);
	if (!retries) {
		dphdcp_err("repeater Bstatus read timeout\n");
		return -ETIMEDOUT;
	}
	/* READY is set so the CP_IRQ interrupt should go high */
	e = get_irq_status(dp, &irq);
	if (e) {
		dphdcp_err("irq register read failure!\n");
		return e;
	}
	if (irq & CP_IRQ_OFFSET)
		dphdcp_vdbg("CP_IRQ interrupt set high\n");

	/* verify V' thrice to check for link failures */
	do {
		memset(dphdcp->bksv_list, 0, sizeof(dphdcp->bksv_list));
		memset(dphdcp->v_prime, 0, sizeof(dphdcp->v_prime));
		/* read Binfo register */
		e = tegra_dphdcp_read16(dp, NV_DPCD_HDCP_BINFO_OFFSET,
				&binfo);
		if (e) {
			dphdcp_err("Binfo read failure!\n");
			return e;
		}
		msleep(100);

		/* clear the irq register, this will be needed to find out
		 * if a spurious interrupt is generated. The DEVICE_SERVICE_IRQ
		 * register is clearable read only and can be cleared by writing
		 * 1 to the respective bit
		 */
		e = tegra_dphdcp_write8(dp, NV_DPCD_DEVICE_SERVICE_IRQ_VECTOR,
								CP_IRQ_RESET);
		/* wait for the CP_IRQ bit to be cleared */
		msleep(100);

		e = get_irq_status(dp, &irq);
		if (e) {
			dphdcp_err("irq register read failure!\n");
			return e;
		}
		dphdcp_vdbg("read irq after clearing: %x\n", irq);

		if (binfo & BINFO_MAX_DEVS_EXCEEDED) {
			dphdcp_err("repeater:max devices (0x%016llx)\n", binfo);
			return -EINVAL;
		}

		if (binfo & BINFO_MAX_CASCADE_EXCEEDED) {
			dphdcp_err("repeater:max cascade (0x%16llx)\n", binfo);
			return -EINVAL;
		}
		dphdcp->binfo = binfo;
		dphdcp->num_bksv_list = binfo & 0x7f;
		dphdcp_vdbg("Binfo 0x%16llx (devices: %d)\n",
				binfo, dphdcp->num_bksv_list);

		/* do not read KSV FIFO when device count is zero */
		if (dphdcp->num_bksv_list != 0)	 {
			e = get_ksvfifo(dp, dphdcp->num_bksv_list,
						dphdcp->bksv_list);
			if (e) {
				dphdcp_err("KSVFIFO read (err %d)\n", e);
				return e;
			}
		}
		/* verify V' three times to check for link failures */
		e = get_vprime(dp, dphdcp->v_prime);
		if (e)
			dphdcp_err("repeater Vprime read failure!\n");
		err = tsec_hdcp_dp_verify_vprime(dphdcp);
		if (err)
			dphdcp_err("vprime verification failed\n");

		/* read CP_IRQ interrupt to check for spurious interrupts.
		 * This needs to be done only when we are authenticating
		 * on the DP engine. On the HDMI engine, we will not take
		 * any action for a spurious interrupt
		 */
		if (!repeater_flag) {
			e = get_irq_status(dp, &irq);
			if (e) {
				dphdcp_err("irq register read failure!\n");
				return e;
			}
			get_bstatus(dp, &bstatus);
			/* For a spurious CP_IRQ interrupt, the CP_IRQ bit
			 * will be high and the bstatus register bits will be
			 * de-asserted. Check for both these conditions
			 * to ensure if the interrupt generated is a
			 * spurious one
			 */
			if ((irq & CP_IRQ_OFFSET) && (bstatus == 0)) {
				dphdcp_vdbg("Spurious interrupt set\n");
				return -EINVAL;
			}
		}
	} while (--vcheck_tries && err);
	if (err)
		return -EINVAL;
	vprime_check_done = true;
	return 0;
}

static void dphdcp_downstream_worker(struct work_struct *work)
{
	int e = 0;
	u8 b_caps = 0;
	u8 bstatus = 0;
	u32 tmp = 0;
	u32 res = 0;

	struct tegra_dphdcp *dphdcp =
		container_of(to_delayed_work(work), struct tegra_dphdcp, work);
	struct tegra_dc_dp_data *dp = dphdcp->dp;
	struct tegra_dc *dc = dp->dc;
	struct tegra_dc_sor_data *sor = dp->sor;
	int hdcp_ta_ret; /* track returns from TA */
	uint32_t ta_cmd = HDCP_AUTH_CMD;
	bool enc = false;

	uint64_t *pkt = kmalloc(PKT_SIZE, GFP_KERNEL);

	/* If memory unavailable */
	if (!pkt) {
		dphdcp_err("Memory allocation failed\n");
		e = -ENOMEM;
		goto failure;
	}
	dphdcp_vdbg("%s():started thread %s\n", __func__, dphdcp->name);
	tegra_dc_io_start(dc);
	mutex_lock(&dphdcp->lock);
	if (dphdcp->state == STATE_OFF) {
		dphdcp_err("dphdcp failure - giving up\n");
		goto err;
	}
	dphdcp->state = STATE_UNAUTHENTICATED;

	/* check plug state to terminate early in case flush_workqueue() */
	if (!dphdcp_is_plugged(dphdcp)) {
		dphdcp_err("worker started while unplugged!\n");
		goto lost_dp;
	}
	dphdcp_vdbg("%s():hpd=%d\n", __func__, dphdcp->plugged);

	dphdcp->a_ksv = 0;
	dphdcp->b_ksv = 0;
	dphdcp->a_n = 0;
	mutex_unlock(&dphdcp->lock);

	/* read bcaps from receiver */
	e = get_bcaps(dp, &b_caps);
	mutex_lock(&dphdcp->lock);
	if (e) {
		dphdcp_err("Error reading bcaps\n");
		goto failure;
	}

	dphdcp_vdbg("read Bcaps = 0x%02x\n", b_caps);
	 /* check if receiver is hdcp capable */
	if (b_caps & BCAPS_HDCP_CAPABLE)
		dphdcp_vdbg("receiver is hdcp capable\n");
	else {
		dphdcp_err("receiver is not hdcp capable\n");
		goto failure;
	}
	dphdcp->ta_ctx = NULL;
	e = te_open_trusted_session(HDCP_PORT_NAME, &dphdcp->ta_ctx);
	if (e) {
		dphdcp_err("open session failed\n");
		goto failure;
	}
repeater_auth:
	if (tegra_dc_is_nvdisplay()) {
		/* if session successfully opened, launch operations */
		/* repeater flag in Bskv must be configured before
		 * loading fuses
		 */
		*pkt = HDCP_TA_CMD_REP;
		*(pkt + 1*HDCP_CMD_OFFSET) = TEGRA_NVHDCP_PORT_DP;
		*(pkt + 2*HDCP_CMD_OFFSET) = 0;
		*(pkt + 3*HDCP_CMD_OFFSET) = b_caps & BCAPS_REPEATER;
		*(pkt + 4*HDCP_CMD_OFFSET) = repeater_flag;
		*(pkt + 5*HDCP_CMD_OFFSET) = dphdcp->dp->sor->ctrl_num;
		e = te_launch_trusted_oper(pkt, PKT_SIZE, ta_cmd,
				dphdcp->ta_ctx);
		if (e) {
			dphdcp_err("te_launch_op failed with error %d\n", e);
			goto failure;
		} else {
			dphdcp_vdbg("Loading kfuse\n");
			e = load_kfuse(dp);
			if (e) {
				dphdcp_err("te_launch_op failed with error %d\n", e);
				goto failure;
			} else {
				dphdcp_vdbg("Loading kfuse\n");
				e = load_kfuse(dp);
				if (e) {
					dphdcp_err("kfuse could not be loaded\n");
					goto failure;
				}
			}

			usleep_range(20000, 25000);
			*pkt = HDCP_TA_CMD_CTRL;
			*(pkt + 1*HDCP_CMD_OFFSET) = TEGRA_NVHDCP_PORT_DP;
			*(pkt + 2*HDCP_CMD_OFFSET) = HDCP_TA_CTRL_ENABLE;
			*(pkt + 3*HDCP_CMD_OFFSET) = repeater_flag;
			*(pkt + 4*HDCP_CMD_OFFSET) = dphdcp->dp->sor->ctrl_num;
			e = te_launch_trusted_oper(pkt, PKT_SIZE, ta_cmd,
					dphdcp->ta_ctx);
			if (e) {
				dphdcp_err("te_launch_op failed with error %d\n", e);
				goto failure;
			} else {
				dphdcp_vdbg("wait AN_VALID ...\n");
				hdcp_ta_ret = *pkt;
				dphdcp_vdbg("An returned %x\n", e);
				if (hdcp_ta_ret) {
					dphdcp_err("An key generation timeout\n");
					goto failure;
				}
				/* check SROM return */
				hdcp_ta_ret = *(pkt + HDCP_CMD_BYTE_OFFSET);
				if (hdcp_ta_ret) {
					dphdcp_err("SROM error\n");
					goto failure;
				}
			}

			msleep(25);
			*pkt = HDCP_TA_CMD_AKSV;
			*(pkt + 1*HDCP_CMD_OFFSET) = TEGRA_NVHDCP_PORT_DP;
			*(pkt + 2*HDCP_CMD_OFFSET) = repeater_flag;
			*(pkt + 3*HDCP_CMD_OFFSET) = dphdcp->dp->sor->ctrl_num;
			e = te_launch_trusted_oper(pkt, PKT_SIZE, ta_cmd,
					dphdcp->ta_ctx);
			if (e) {
				dphdcp_err("te_launch_op failed with error %d\n", e);
				goto failure;

			} else {
				hdcp_ta_ret = (u64)*pkt;
				dphdcp->a_ksv = (u64)*(pkt + 1*HDCP_CMD_BYTE_OFFSET);
				dphdcp->a_n = (u64)*(pkt + 2*HDCP_CMD_BYTE_OFFSET);
				dphdcp_vdbg("Aksv is 0x%016llx\n", dphdcp->a_ksv);
				dphdcp_vdbg("An is 0x%016llx\n", dphdcp->a_n);
				/* check if verification of Aksv failed */
			if (hdcp_ta_ret) {
				dphdcp_err("Aksv verify failure\n");
				goto disable;
			}
		} }
	} else {
		set_bksv(sor, 0, (b_caps & BCAPS_REPEATER));
		e = load_kfuse(dp);
		if (e) {
			dphdcp_err("error loading kfuse\n");
			goto failure;
		}

		usleep_range(20000, 25000);
		hdcp_ctrl_run(sor, 1);

		dphdcp_vdbg("waiting for An_valid\n");

		/* wait for hardware to generate HDCP values */
		e = wait_hdcp_ctrl(sor, AN_VALID | SROM_ERR, &res);
		if (e) {
			dphdcp_err("An key generation timeout\n");
			goto failure;
		}
		if (res & SROM_ERR) {
			dphdcp_err("SROM error\n");
			goto failure;
		}

		msleep(25);

		dphdcp->a_ksv = get_aksv(sor);
		dphdcp->a_n = get_an(sor);

		dphdcp_vdbg("aksv is 0x%016llx\n", dphdcp->a_ksv);
		dphdcp_vdbg("an is 0x%016llx\n", dphdcp->a_n);

		if (verify_ksv(dphdcp->a_ksv)) {
			dphdcp_err("Aksv verify failure! (0x%016llx)\n",
					dphdcp->a_ksv);
			goto disable;
		}
	}
	mutex_unlock(&dphdcp->lock);

	/* write An to receiver */
	e = tegra_dphdcp_write64(dp, NV_DPCD_HDCP_AN_OFFSET, &dphdcp->a_n);
	if (e) {
		dphdcp_err("An write failure\n");
		mutex_lock(&dphdcp->lock);
		goto failure;
	}
	dphdcp_vdbg("wrote An = 0x%016llx\n", dphdcp->a_n);

	/* write Aksv to receiver */
	e = tegra_dphdcp_write40(dp, NV_DPCD_HDCP_AKSV_OFFSET,
						&dphdcp->a_ksv);
	if (e) {
		dphdcp_err("Aksv write failure\n");
		mutex_lock(&dphdcp->lock);
		goto failure;
	}
	dphdcp_vdbg("wrote Aksv = 0x%010llx\n", dphdcp->a_ksv);

	mutex_lock(&dphdcp->lock);
	/* handle if connection lost in the middle of authentication */
	if (!dphdcp_is_plugged(dphdcp))
		goto lost_dp;

	mutex_unlock(&dphdcp->lock);

	/* get bksv from receiver */
	e = tegra_dphdcp_read40(dp, NV_DPCD_HDCP_BKSV_OFFSET,
					&dphdcp->b_ksv);
	mutex_lock(&dphdcp->lock);
	if (e) {
		dphdcp_err("Bksv read failure\n");
		goto failure;
	}
	dphdcp_vdbg("read Bksv from device: 0x%016llx\n", dphdcp->b_ksv);

	if (tegra_dc_is_nvdisplay()) {
		*pkt = HDCP_TA_CMD_BKSV;
		*(pkt + 1*HDCP_CMD_OFFSET) = TEGRA_NVHDCP_PORT_DP;
		*(pkt + 2*HDCP_CMD_OFFSET) = dphdcp->b_ksv;
		*(pkt + 3*HDCP_CMD_OFFSET) = b_caps & BCAPS_REPEATER;
		*(pkt + 4*HDCP_CMD_OFFSET) = repeater_flag;
		*(pkt + 5*HDCP_CMD_OFFSET) = dphdcp->dp->sor->ctrl_num;
		e = te_launch_trusted_oper(pkt, PKT_SIZE, ta_cmd, dphdcp->ta_ctx);
		if (e) {
			dphdcp_err("te launch operation failed with error: %d\n", e);
			goto failure;
		} else {
			/* check if Bksv verification was successful */
			hdcp_ta_ret = (int)*pkt;
			if (hdcp_ta_ret) {
				dphdcp_err("Bksv verify failure\n");
				goto failure;
			} else {
				dphdcp_vdbg("loaded Bksv into controller\n");
				/* check if R0 read was successful */
				hdcp_ta_ret = (int)*(pkt);
				if (hdcp_ta_ret) {
					dphdcp_err("R0 read failure\n");
					goto failure;
				}
			}
		}
	} else {
		/*
		 * verify the bksv to be if it has a correct combination of
		 * 1's and 0's
		 */
		if (verify_ksv(dphdcp->b_ksv)) {
			dphdcp_err("Bksv verify failure! (0x%016llx)\n",
					dphdcp->b_ksv);
			goto failure;
		}

		set_bksv(sor, dphdcp->b_ksv, (b_caps & BCAPS_REPEATER));
		dphdcp_vdbg("Loaded Bksv into controller\n");

		/* check if the computations of Km, Ks, M0 and R0 are over */
		e = wait_hdcp_ctrl(sor, R0_VALID, NULL);
		if (e) {
			dphdcp_err("R0 read failure\n");
			goto failure;
		}

		dphdcp_vdbg("R0 valid\n");
	}
	mutex_unlock(&dphdcp->lock);

	msleep(100); /* cannot read R0' within 100 ms of writing AKSV */

	/*
	 * after part 1 of authentication protocol, check for
	 * link integrity
	 * TODO: add support for both single and multi stream mode
	 */
	if (!repeater_flag) {
		e = validate_rx(dphdcp);
		if (e) {
			dphdcp_err("could not validate receiver\n");
			mutex_lock(&dphdcp->lock);
			goto failure;
		}
		if (tegra_dc_is_nvdisplay()) {
			*pkt = HDCP_TA_CMD_ENC;
			*(pkt + 1*HDCP_CMD_OFFSET) = TEGRA_NVHDCP_PORT_DP;
			*(pkt + 2*HDCP_CMD_OFFSET) = b_caps;
			*(pkt + 3*HDCP_CMD_OFFSET) = dphdcp->dp->sor->ctrl_num;
			e = te_launch_trusted_oper(pkt, PKT_SIZE/4, ta_cmd,
					dphdcp->ta_ctx);
			if (e) {
				dphdcp_err("launch oper failed with error: %d\n", e);
				goto failure;
			}
			enc = true;
		} else {
			tmp = tegra_sor_readl_ext(sor, NV_SOR_DP_HDCP_CTRL);
			tmp |= CRYPT_ENABLED;
			tegra_sor_writel_ext(sor, NV_SOR_DP_HDCP_CTRL, tmp);
		}
		dphdcp_vdbg("CRYPT enabled\n");
	}
	msleep(100);
	e = get_bstatus(dp, &bstatus);
	if (!e && (bstatus & BSTATUS_LINK_INTEG_FAIL)) {
		dphdcp_err("link integrity failure\n");
		mutex_lock(&dphdcp->lock);
		goto failure;
	}
	/* revocation check for receiver. For repeater, is it
	 * handled in verify V'
	 */
	if (!(b_caps & BCAPS_REPEATER)) {
		e = srm_revocation_check(dphdcp);
		if (e) {
			dphdcp_err("SRM revocation check failed\n");
			goto failure;
		}
	}
	/*
	 * part 2 of authentication protocol, if receiver is
	 * a repeater
	 */
	if ((b_caps & BCAPS_REPEATER) && !vprime_check_done) {
		e = get_repeater_info(dphdcp);
		if (e) {
			dphdcp_err("get repeater info failed\n");
			/* some latency before we transition to the
			 * HDMI engine
			 */
			msleep(100);
			repeater_flag = true;
			mutex_lock(&dphdcp->lock);
			goto failure;
		}
		/* continue last part of authentication as
		 * a DP receiver, ie. second stage of
		 * authentication will not be performed
		 */
		repeater_flag = false;
		mutex_lock(&dphdcp->lock);
		if (tegra_dc_is_nvdisplay()) {
			*pkt = HDCP_TA_CMD_CTRL;
			*(pkt + 1*HDCP_CMD_OFFSET) = TEGRA_NVHDCP_PORT_DP;
			*(pkt + 2*HDCP_CMD_OFFSET) = HDCP_TA_CTRL_DISABLE;
			*(pkt + 3*HDCP_CMD_OFFSET) = repeater_flag;
			*(pkt + 4*HDCP_CMD_OFFSET) = dphdcp->dp->sor->ctrl_num;
			e = te_launch_trusted_oper(pkt, PKT_SIZE, ta_cmd, dphdcp->ta_ctx);
			if (e) {
				dphdcp_err("te_launch_oper failed with err: %d\n", e);
				goto failure;
			}
		} else {
			tmp = tegra_sor_readl_ext(sor, NV_SOR_DP_HDCP_CTRL);
			tmp |= CRYPT_ENABLED;
			tegra_sor_writel_ext(sor, NV_SOR_DP_HDCP_CTRL, tmp);
		}
		goto repeater_auth;
	}

	mutex_lock(&dphdcp->lock);
	dphdcp->state = STATE_LINK_VERIFY;
	dphdcp_info("link verified!\n");

	while (1) {
		if (!dphdcp_is_plugged(dphdcp))
			goto lost_dp;

		if (dphdcp->state != STATE_LINK_VERIFY)
			goto failure;

		mutex_unlock(&dphdcp->lock);
		/* check for link integrity failure */
		e = get_bstatus(dp, &bstatus);
		if (!e && (bstatus & BSTATUS_LINK_INTEG_FAIL)) {
			dphdcp_err("link integrity failure\n");
			mutex_lock(&dphdcp->lock);
			goto failure;
		}
		tegra_dc_io_end(dc);
		wait_event_interruptible_timeout(wq_worker,
				!dphdcp_is_plugged(dphdcp), msecs_to_jiffies(200));
		tegra_dc_io_start(dc);
		mutex_lock(&dphdcp->lock);

	}

failure:
	dphdcp->fail_count++;
	if (dphdcp->fail_count > dphdcp->max_retries)
		dphdcp_err("dphdcp failure- too many failures, giving up\n");
	else {
		dphdcp_err("dphdcp failure- renegotiating in 1 second\n");
		if (!dphdcp_is_plugged(dphdcp))
			goto lost_dp;

		queue_delayed_work(dphdcp->downstream_wq, &dphdcp->work,
						msecs_to_jiffies(1000));
	}

	/* Failed because of lack of memory */
	if (e == -ENOMEM) {
		kfree(pkt);
		return;
	}
lost_dp:
	dphdcp_info("lost dp connection\n");
	dphdcp->state = STATE_UNAUTHENTICATED;
	if (tegra_dc_is_nvdisplay()) {
		if (pkt) {
			*pkt = HDCP_TA_CMD_CTRL;
			*(pkt + HDCP_CMD_OFFSET) = TEGRA_NVHDCP_PORT_DP;
			*(pkt + 2*HDCP_CMD_OFFSET) = HDCP_TA_CTRL_DISABLE;
			*(pkt + 3*HDCP_CMD_OFFSET) = repeater_flag;
			*(pkt + 4*HDCP_CMD_OFFSET) = dphdcp->dp->sor->ctrl_num;
		}
		/* a launch operation makes sense only if a valid context exists
		 * already
		 */
		if (dphdcp->ta_ctx) {
			e = te_launch_trusted_oper(pkt, PKT_SIZE, ta_cmd,
					dphdcp->ta_ctx);
			if (e) {
				dphdcp_err("te_launch_oper failed with error:"
						"%d\n", e);
				goto failure;
			}
		}
	} else {
		hdcp_ctrl_run(sor, 0);
	}

err:
	mutex_unlock(&dphdcp->lock);
	kfree(pkt);
	if (dphdcp->ta_ctx) {
		te_close_trusted_session(dphdcp->ta_ctx);
		dphdcp->ta_ctx = NULL;
	}
	tegra_dc_io_end(dc);
	return;
disable:
	dphdcp->state = STATE_OFF;
	kfree(pkt);
	if (dphdcp->ta_ctx) {
		te_close_trusted_session(dphdcp->ta_ctx);
		dphdcp->ta_ctx = NULL;
	}
	dphdcp_set_plugged(dphdcp, false);
	mutex_unlock(&dphdcp->lock);
	tegra_dc_io_end(dc);
}

#ifdef DPHDCP22
/* HDCP 2.2 over display port */

/* write N bytes of data over AUX channel */
static int tegra_dphdcp_write_n(struct tegra_dc_dp_data *dp, u32 reg,
						u64 *data, u8 size)
{
	u8 *buf = NULL;
	u8 *orig_buf = NULL;
	int e;

	buf = kmalloc(size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	orig_buf = buf;
	memcpy(buf, data, size);
	if (size <= MAX_AUX_SIZE) {
		e = tegra_dphdcp_write(dp, reg, buf, size);
		if (e) {
			dphdcp_err("Error writing over AUX\n");
			goto error;
		}
	} else {
		while (size > MAX_AUX_SIZE) {
			e = tegra_dphdcp_write(dp, reg, buf, MAX_AUX_SIZE);
			if (e) {
				dphdcp_err("Error writing over AUX\n");
				goto error;
			}
			size -= MAX_AUX_SIZE;
			buf += MAX_AUX_SIZE;
		}
		if (size) {
			e = tegra_dphdcp_write(dp, reg, buf, size);
			if (e) {
				dphdcp_err("Error writing over AUX\n");
				goto error;
			}
		}
	}

error:
	kfree(orig_buf);
	return e;
}

/* read N bytes of data over AUX channel */
static int tegra_dphdcp_read_n(struct tegra_dc_dp_data *dp, u32 cmd,
						u64 *data, int size)
{
	u8 *buf;
	int e;
	u8 *orig_buf;
	u32 status;

	buf = kmalloc(size, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	orig_buf = buf;
	if (size <= MAX_AUX_SIZE) {
		e = tegra_dphdcp_read(dp, cmd, buf, size, &status);
		if (e) {
			dphdcp_err("Error reading over AUX\n");
			goto error;
		}
	} else {
		while (size > MAX_AUX_SIZE) {
			e = tegra_dphdcp_read(dp, cmd, buf,
					MAX_AUX_SIZE, &status);
			if (e) {
				dphdcp_err("Error reading over AUX\n");
				goto error;
			}
			size -= MAX_AUX_SIZE;
			buf += MAX_AUX_SIZE;
		}
		if (size) {
			e = tegra_dphdcp_write(dp, cmd, buf, MAX_AUX_SIZE);
			if (e) {
				dphdcp_err("Error reading over AUX\n");
				goto error;
			}
		}
	}
	/* copy the content to data */
	memcpy(data, orig_buf, size);

error:
	kfree(orig_buf);
	return e;
}

static int get_rxstatus(struct tegra_dc_dp_data *dp, u8 *rxstatus)
{
	u32 status;

	return tegra_dphdcp_read(dp, NV_DPCD_HDCP_RXSTATUS,
					rxstatus, 1, &status);
}

static int dphdcp_ake_init_rtx_send(struct tegra_dc_dp_data *dp, u64 *buf)
{
	/* write 8 bytes of rtx */
	return tegra_dphdcp_write_n(dp, NV_DPCD_HDCP_RTX_OFFSET, buf, 8);
}

static int dphdcp_ake_init_txcaps_send(struct tegra_dc_dp_data *dp, u64 *buf)
{
	/* write 3 bytes of txcaps */
	return tegra_dphdcp_write_n(dp, NV_DPCD_HDCP_TXCAPS_OFFSET, buf, 3);
}

static int dphdcp_ake_cert_recv_rx(struct tegra_dc_dp_data *dp, u8 *buf)
{
	/* read 522 bytes of receiver certificate */
	return tegra_dphdcp_read_n(dp, NV_DPCD_HDCP_CERT_RX_OFFSET,
						(u64 *)buf, 522);
}

static int dphdcp_ake_cert_recv_rrx(struct tegra_dc_dp_data *dp, u64 *buf)
{
	/* write 8 bytes of pseudo random number */
	return tegra_dphdcp_read_n(dp, NV_DPCD_HDCP_CERT_RRX_OFFSET,
							buf, 8);
}

static int dphdcp_ake_cert_recv_rxcaps(struct tegra_dc_dp_data *dp, u16 *buf)
{
	/* write 3 bytes of receiver capability */
	return tegra_dphdcp_read_n(dp, NV_DPCD_HDCP_CERT_RXCAPS_OFFSET,
							(u64 *)buf, 3);
}

static int dphdcp_ake_no_stored_km_send(struct tegra_dc_dp_data *dp, u64 *buf)
{
	return tegra_dphdcp_write_n(dp, NV_DPCD_HDCP_EKM_NOSTORED, buf, 128);
}

static int dphdcp_ake_hprime_receive(struct tegra_dc_dp_data *dp, u32 *buf)
{
	return tegra_dphdcp_read_n(dp, NV_DPCD_HDCP_HPRIME,
					(u64 *)buf, 32);
}

static int dphdcp_lc_init_send(struct tegra_dc_dp_data *dp, u64 *buf)
{
	return tegra_dphdcp_write_n(dp, NV_DPCD_HDCP_RN, buf, 8);
}

static int dphdcp_ake_pairing_info_receive(struct tegra_dc_dp_data *dp,
							u64 *buf)
{
	return tegra_dphdcp_read_n(dp, NV_DPCD_HDCP_LPRIME,
						buf, 16);
}

static int dphdcp_lc_lprime_receive(struct tegra_dc_dp_data *dp, u64 *buf)
{
	return tegra_dphdcp_read_n(dp, NV_DPCD_HDCP_LPRIME,
						buf, 32);
}

static int dphdcp_ske_eks_send(struct tegra_dc_dp_data *dp, u64 *buf)
{
	return tegra_dphdcp_write_n(dp, NV_DPCD_HDCP_EKS, buf, 16);
}

static int dphdcp_ske_riv_send(struct tegra_dc_dp_data *dp, u64 *buf)
{
	return tegra_dphdcp_write_n(dp, NV_DPCD_HDCP_RIV, buf, 8);
}

static int dphdcp_rxinfo_recv(struct tegra_dc_dp_data *dp, u16 *buf)
{
	return tegra_dphdcp_read_n(dp, NV_DPCD_HDCP_RXINFO, (u64 *)buf, 2);
}

static int dphdcp_seqnum_recv(struct tegra_dc_dp_data *dp, u64 *buf)
{
	return tegra_dphdcp_read_n(dp, NV_DPCD_HDCP_SEQNUM_V, buf, 3);
}

static int dphdcp_vprime_recv(struct tegra_dc_dp_data *dp, u16 *buf)
{
	return tegra_dphdcp_read_n(dp, NV_DPCD_HDCP_VPRIME, (u64 *)buf, 16);
}

static int dphdcp_recvrid_list_recv(struct tegra_dc_dp_data *dp, u64 *buf)
{
	return tegra_dphdcp_read_n(dp, NV_DPCD_HDCP_RX_ID_LIST, buf, 635);
}

static int dphdcp_rptr_ack_send(struct tegra_dc_dp_data *dp, u64 *buf)
{
	return tegra_dphdcp_read_n(dp, NV_DPCD_HDCP_V, buf, 16);
}

static int dphdcp_rptr_seqnum_send(struct tegra_dc_dp_data *dp, u8 *buf)
{
	return tegra_dphdcp_write_n(dp, NV_DPCD_HDCP_SEQ_NUM_M,
						(u64 *)buf, 3);
}

static int dphdcp_rptr_k_send(struct tegra_dc_dp_data *dp, u16 *buf)
{
	return tegra_dphdcp_write_n(dp, NV_DPCD_HDCP_K,
					(u64 *)&buf, 2);
}

static int dphdcp_rptr_strmid_type_send(struct tegra_dc_dp_data *dp, u16 *buf)
{
	/* max streams: 1 */
	return tegra_dphdcp_write_n(dp, NV_DPCD_HDCP_STRMID_TYPE,
					(u64 *)buf, 2);
}

static int dphdcp_rptr_stream_ready_recv(struct tegra_dc_dp_data *dp, u8 *buf)
{
	return tegra_dphdcp_read_n(dp, NV_DPCD_HDCP_MPRIME, (u64 *)buf, 32);
}

/* poll for status until timeout */
static int dphdcp_poll(struct tegra_dc_dp_data *dp, int timeout, int status)
{
	int e;
	s64 start_time;
	s64 end_time;
	struct timespec tm;
	u8 val;
	u32 aux_stat;

	ktime_get_ts(&tm);
	start_time = timespec_to_ns(&tm);

	while (1) {
		ktime_get_ts(&tm);
		end_time = timespec_to_ns(&tm);
		if ((end_time - start_time)/1000 >= timeout*1000)
			return -ETIMEDOUT;
		e = tegra_dphdcp_read(dp,
		NV_DPCD_HDCP_RXSTATUS, &val, 1, &aux_stat);
		if (e) {
			dphdcp_err("dphdcp_poll_ready failed\n");
			goto exit;
		}
		if (status == HDCP_READY) {
			if (val)
				break;
		} else if (status == HDCP_REAUTH) {
			if (cpu_to_be16(val) & HDCP_REAUTH_MASK)
				break;
		}
	}
	e = 0;
exit:
	return e;
}

static int dphdcp_poll_ready(struct tegra_dc_dp_data *dp,
					int timeout)
{
	int e;

	e = dphdcp_poll(dp, timeout, HDCP_READY);
	return e;
}

static int tsec_hdcp_authentication(struct tegra_dc_dp_data *dp,
				struct hdcp_context_t *hdcp_context)
{
	int err = 0;
	u8 version = 0x02;
	u16 caps = 0;
	u16 txcaps = 0x0;
	u64 txval;
	u32 *buf;

	err = tsec_hdcp_readcaps(hdcp_context);
	if (err)
		goto exit;
	err = tsec_hdcp_init(hdcp_context);
	if (err)
		goto exit;
	/* rtx populated in hdcp create session */
	err = tsec_hdcp_create_session(hdcp_context, DISPLAY_TYPE_DP,
					dphdcp->dp->sor->ctrl_num);
	if (err)
		goto exit;
	err = tsec_hdcp_exchange_info(hdcp_context,
		HDCP_EXCHANGE_INFO_GET_TMTR_INFO, &version, &caps);
	if (err)
		goto exit;
	hdcp_context->msg.txcaps_version = version;
	hdcp_context->msg.txcaps_capmask = txcaps;
	/* initiate authentication with 64 bit pseudo random # */
	err = dphdcp_ake_init_rtx_send(dp, &hdcp_context->msg.rtx);
	if (err)
		goto exit;
	txval = (version << 16) | txcaps;
	/* write the txcaps register */
	err = dphdcp_ake_init_txcaps_send(dp, &txval);
	if (err)
		goto exit;

	/* wait for 100 ms before reading the RX */
	err = dphdcp_poll_ready(dp, 100);
	if (err)
		goto exit;

	/* process upon CP_IRQ interrupt */
	/* read the certificate from the rx */
	err = dphdcp_ake_cert_recv_rx(dp, hdcp_context->msg.cert_rx);
	if (err)
		goto exit;

	err = dphdcp_ake_cert_recv_rrx(dp, &hdcp_context->msg.rrx);
	if (err)
		goto exit;

	err = dphdcp_ake_cert_recv_rxcaps(dp,
		&hdcp_context->msg.rxcaps_capmask);
	if (err)
		goto exit;

	/* verify received certificate */
	err = tsec_hdcp_verify_cert(hdcp_context);
	if (err)
		goto exit;

	err = tsec_hdcp_update_rrx(hdcp_context);
	if (err)
		goto exit;

	err = tsec_hdcp_generate_ekm(hdcp_context);
	if (err)
		goto exit;

	err = dphdcp_ake_no_stored_km_send(dp, (u64 *)hdcp_context->msg.ekm);
	if (err)
		goto exit;

	err = tsec_hdcp_exchange_info(hdcp_context,
		HDCP_EXCHANGE_INFO_SET_RCVR_INFO,
		&hdcp_context->msg.rxcaps_version,
		&hdcp_context->msg.rxcaps_capmask);

	if (err)
		goto exit;

	/* device in revocation list ? */
	err = tsec_hdcp_revocation_check(hdcp_context);
	if (err)
		goto exit;
	/* H' should be ready within 1 sec */
	err = dphdcp_poll_ready(dp, 1000);
	if (err)
		goto exit;
	buf = (u32 *)hdcp_context->msg.hprime;
	err = dphdcp_ake_hprime_receive(dp, buf);
	if (err)
		goto exit;

	err = tsec_hdcp_verify_hprime(hdcp_context);
	if (err)
		goto exit;
	/* wait for AKE pairing message to be ready */
	err = dphdcp_poll_ready(dp, 200);
	if (err)
		goto exit;

	err = dphdcp_ake_pairing_info_receive(dp,
		(u64 *)hdcp_context->msg.ekhkm);
	if (err)
		goto exit;

	err = tsec_hdcp_encrypt_pairing_info(hdcp_context);
	if (err)
		goto exit;

	err = tsec_hdcp_encrypt_pairing_info(hdcp_context);
	if (err)
		goto exit;

	err = tsec_hdcp_generate_lc_init(hdcp_context);
	if (err)
		goto exit;

	err = dphdcp_lc_init_send(dp, &hdcp_context->msg.rn);
	if (err)
		goto exit;

	err = dphdcp_poll_ready(dp, 7);
	if (err)
		goto exit;

	err = dphdcp_lc_lprime_receive(dp, (u64 *)hdcp_context->msg.lprime);
	if (err)
		goto exit;

	err = tsec_hdcp_verify_lprime(hdcp_context);
	if (err)
		goto exit;

	err = tsec_hdcp_ske_init(hdcp_context);
	if (err)
		goto exit;

	err = dphdcp_ske_eks_send(dp, (u64 *)hdcp_context->msg.eks);
	if (err)
		goto exit;

	err = dphdcp_ske_riv_send(dp, (u64 *)hdcp_context->msg.riv);
	if (err)
		goto exit;

	/* check if the receiver is a repeater */
	if (hdcp_context->msg.rxcaps_capmask & HDCP_22_REPEATER) {
		dp->dphdcp->repeater = 1;
		err = dphdcp_poll_ready(dp, 3000);
		if (err)
			goto exit;

		/* read receiver id list */
		err = dphdcp_rxinfo_recv(dp, &hdcp_context->msg.rxinfo);
		if (err)
			goto exit;

		err = dphdcp_seqnum_recv(dp, (u64 *)hdcp_context->msg.seq_num);
		if (err)
			goto exit;

		err = dphdcp_vprime_recv(dp, &hdcp_context->msg.rxinfo);
		if (err)
			goto exit;

		err = dphdcp_recvrid_list_recv(dp,
			(u64 *)&hdcp_context->msg.rxinfo);
		if (err)
			goto exit;

		err = tsec_hdcp_verify_vprime(hdcp_context);
		if (err)
			goto exit;

		/* send ack for repeater auth */
		err = dphdcp_rptr_ack_send(dp, (u64 *)hdcp_context->msg.v);
		if (err)
			goto exit;

		err = tsec_hdcp_rptr_stream_manage(hdcp_context);
		if (err)
			goto exit;
		/* One stream */
		hdcp_context->msg.k = 0x0100;
		/* STREAM_ID and Type are 0 */
		hdcp_context->msg.streamid_type[0] = 0x0000;
		/* stream management information */
		err = dphdcp_rptr_seqnum_send(dp,
			hdcp_context->msg.seq_num_m);
		if (err)
			goto exit;

		err = dphdcp_rptr_k_send(dp,
			&hdcp_context->msg.k);
		if (err)
			goto exit;

		err = dphdcp_rptr_strmid_type_send(dp,
			hdcp_context->msg.streamid_type);
		if (err)
			goto exit;

		/* repeater auth stream ready information */
		err = dphdcp_rptr_stream_ready_recv(dp,
				hdcp_context->msg.mprime);
		if (err)
			goto exit;

	}
	dphdcp_info("HDCP authentication successful\n");
exit:
	if (err)
		dphdcp_err("HDCP authentication failed with err %d\n", err);
	return err;
}

static void dphdcp2_downstream_worker(struct work_struct *work)
{
	struct tegra_dphdcp *dphdcp =
		container_of(to_delayed_work(work),
		 struct tegra_dphdcp, work);
	struct tegra_dc_dp_data *dp = dphdcp->dp;
	struct hdcp_context_t hdcp_context;
	struct tegra_dc *dc = dp->dc;
	int e;
	u8 rxstatus;

	e = tsec_hdcp_create_context(&hdcp_context);
	if (e)
		goto err;

	dphdcp_vdbg("%s():started thread %s\n", __func__, dphdcp->name);
	tegra_dc_io_start(dc);

	mutex_lock(&dphdcp->lock);
	if (dphdcp->state == STATE_OFF) {
		dphdcp_err("dphdcp failure: giving up\n");
		goto err;
	}
	dphdcp->state = STATE_UNAUTHENTICATED;

	/* check plug state to terminate early in case of flush_workqueue */
	if (!dphdcp_is_plugged(dphdcp)) {
		dphdcp_err("worker started in unplugged state\n");
		goto lost_dp;
	}
	dphdcp_vdbg("%s():hpd=%d\n", __func__, dphdcp->plugged);
	mutex_unlock(&dphdcp->lock);

	if (tsec_hdcp_authentication(dp, &hdcp_context)) {
		mutex_lock(&dphdcp->lock);
		goto failure;
	}

	mutex_lock(&dphdcp->lock);
	dphdcp->state = STATE_LINK_VERIFY;
	mutex_unlock(&dphdcp->lock);

	e = tsec_hdcp_session_ctrl(&hdcp_context,
			HDCP_SESSION_CTRL_FLAG_ACTIVATE);
	if (e) {
		dphdcp_err("tsec hdcp session ctrl failed\n");
		mutex_lock(&dphdcp->lock);
		goto failure;
	}
	dphdcp_info("HDCP 2.2 CRYPT enabled\n");
	mutex_lock(&dphdcp->lock);
	while (1) {
		if (!dphdcp_is_plugged(dphdcp))
			goto lost_dp;

		if (dphdcp->state != STATE_LINK_VERIFY)
			goto failure;

		mutex_unlock(&dphdcp->lock);

		/* link integrity check */
		e = get_rxstatus(dp, &rxstatus);
		if (!e && (rxstatus & HDCP_LINK_INTEG_FAIL)) {
			dphdcp_err("link integrity failure\n");
			mutex_lock(&dphdcp->lock);
			goto failure;
		}
		tegra_dc_io_end(dc);
		wait_event_interruptible_timeout(wq_worker,
		!dphdcp_is_plugged(dphdcp), msecs_to_jiffies(200));
		tegra_dc_io_start(dc);
		mutex_lock(&dphdcp->lock);
	}

failure:
	dphdcp->fail_count++;
	if (dphdcp->fail_count > dphdcp->max_retries) {
		dphdcp_err("dphdcp failure - too many failures, giving up\n");
	} else {
		dphdcp_err("dphdcp failure - renegotiating in 1 sec\n");
		if (!dphdcp_is_plugged(dphdcp))
			goto lost_dp;

		queue_delayed_work(dphdcp->downstream_wq, &dphdcp->work,
						msecs_to_jiffies(1000));
	}

lost_dp:
	dphdcp->state = STATE_UNAUTHENTICATED;

err:
	mutex_unlock(&dphdcp->lock);
	tegra_dc_io_end(dc);
	e = tsec_hdcp_free_context(&hdcp_context);
}
#endif

static int tegra_dphdcp_on(struct tegra_dphdcp *dphdcp)
{
	u8 hdcp2version = 0;
	int e;
	u32 status;
	struct tegra_dc_dp_data *dp = dphdcp->dp;

	dphdcp->state = STATE_UNAUTHENTICATED;
	if (dphdcp_is_plugged(dphdcp) &&
		atomic_read(&dphdcp->policy) !=
		TEGRA_DC_HDCP_POLICY_ALWAYS_OFF) {
		dphdcp->fail_count = 0;
		e = tegra_dphdcp_read(dp, HDCP_HDCP2_VERSION,
				&hdcp2version, 1, &status);
		if (e)
			return -EIO;

		dphdcp_vdbg("read back version:%x\n", hdcp2version);
		if (hdcp2version & HDCP_HDCP2_VERSION_HDCP22_YES) {
#ifdef DPHDCP22
			INIT_DELAYED_WORK(&dphdcp->work,
					dphdcp2_downstream_worker);
#endif
			dphdcp->hdcp22 = HDCP22_PROTOCOL;
		} else {
			INIT_DELAYED_WORK(&dphdcp->work,
					dphdcp_downstream_worker);
			dphdcp->hdcp22 = HDCP1X_PROTOCOL;
		}

		queue_delayed_work(dphdcp->downstream_wq, &dphdcp->work,
						msecs_to_jiffies(100));
	}

	return 0;
}

static int tegra_dphdcp_off(struct tegra_dphdcp *dphdcp)
{
	mutex_lock(&dphdcp->lock);
	dphdcp->state = STATE_OFF;
	dphdcp_set_plugged(dphdcp, false);
	mutex_unlock(&dphdcp->lock);
	wake_up_interruptible(&wq_worker);
	cancel_delayed_work_sync(&dphdcp->work);
	return 0;
}

static int get_dphdcp_state(struct tegra_dphdcp *dphdcp,
			struct tegra_nvhdcp_packet *pkt)
{
	int i;

	mutex_lock(&dphdcp->lock);
	if (dphdcp->state != STATE_LINK_VERIFY) {
		memset(pkt, 0, sizeof(struct tegra_nvhdcp_packet));
		pkt->packet_results = TEGRA_NVHDCP_RESULT_LINK_FAILED;
	} else {
		pkt->num_bksv_list = dphdcp->num_bksv_list;
		for (i = 0; i < pkt->num_bksv_list; i++)
			pkt->bksv_list[i] = dphdcp->bksv_list[i];
		pkt->binfo = dphdcp->binfo;
		pkt->b_ksv = dphdcp->b_ksv;
		memcpy(pkt->v_prime, dphdcp->v_prime, sizeof(dphdcp->v_prime));
		pkt->packet_results = TEGRA_NVHDCP_RESULT_SUCCESS;
		pkt->hdcp22 = dphdcp->hdcp22;
		pkt->port = TEGRA_NVHDCP_PORT_DP;
	}
	pkt->sor = dphdcp->dp->sor->ctrl_num;
	mutex_unlock(&dphdcp->lock);
	return 0;
}

int tegra_dphdcp_set_policy(struct tegra_dphdcp *dphdcp, int pol)
{
	if (pol == TEGRA_DC_HDCP_POLICY_ALWAYS_ON) {
		dphdcp_info("using \"always on\" policy.\n");
		if (atomic_xchg(&dphdcp->policy, pol) != pol) {
			/* policy changed, start working */
			tegra_dphdcp_on(dphdcp);
		}
	} else if (pol == TEGRA_DC_HDCP_POLICY_ALWAYS_OFF) {
		dphdcp_info("using \"always off\" policy.\n");
		if (atomic_xchg(&dphdcp->policy, pol) != pol) {
			/* policy changed, stop working */
			tegra_dphdcp_off(dphdcp);
		}
	} else {
		/* unsupported policy */
		return -EINVAL;
	}

	return 0;
}

static int tegra_dphdcp_renegotiate(struct tegra_dphdcp *dphdcp)
{
	mutex_lock(&dphdcp->lock);
	dphdcp->state = STATE_RENEGOTIATE;
	mutex_unlock(&dphdcp->lock);
	tegra_dphdcp_on(dphdcp);
	return 0;
}

void tegra_dphdcp_set_plug(struct tegra_dphdcp *dphdcp, bool hpd)
{
	if (tegra_dc_is_t19x()) {
		uint32_t ft_info;
		/* enable HDCP only if board has SFK */
		tegra_fuse_readl(FUSE_OPT_FT_REV_0, &ft_info);
		/* only fuses with revision id >= 0x5 have SFK */
		if (ft_info < FUSE_START_SFK) {
			dphdcp_err("Device does not have SFK!");
			return;
		}
	}
	/* ensure all previous values are reset on hotplug */
	vprime_check_done = false;
	repeater_flag = false;
	dphdcp_debug("DP hotplug detected (hpd = %d)\n", hpd);
	if (hpd) {
		dphdcp_set_plugged(dphdcp, true);
		tegra_dphdcp_on(dphdcp);
	} else {
		tegra_dphdcp_off(dphdcp);
	}
}

static long dphdcp_dev_ioctl(struct file *filp,
			unsigned int cmd, unsigned long arg)
{
	struct tegra_dphdcp *dphdcp;
	struct tegra_nvhdcp_packet *pkt;
	struct tegra_dc_dp_data *dp;
	int e = -ENOTTY;

	if (!filp)
		return -EINVAL;

	dphdcp = filp->private_data;
	dp = dphdcp->dp;

	switch (cmd) {
	case TEGRAIO_NVHDCP_ON:
		mutex_lock(&dphdcp->lock);
		dphdcp_set_plugged(dphdcp, dp->enabled);
		mutex_unlock(&dphdcp->lock);
		return tegra_dphdcp_on(dphdcp);

	case TEGRAIO_NVHDCP_OFF:
		return tegra_dphdcp_off(dphdcp);

	case TEGRAIO_NVHDCP_SET_POLICY:
		mutex_lock(&dphdcp->lock);
		dphdcp_set_plugged(dphdcp, dp->enabled);
		mutex_unlock(&dphdcp->lock);
		return tegra_dphdcp_set_policy(dphdcp, arg);

	case TEGRAIO_NVHDCP_RENEGOTIATE:
		mutex_lock(&dphdcp->lock);
		dphdcp_set_plugged(dphdcp, dp->enabled);
		mutex_unlock(&dphdcp->lock);
		e = tegra_dphdcp_renegotiate(dphdcp);
		break;

	case TEGRAIO_NVHDCP_HDCP_STATE:
		pkt = kmalloc(sizeof(*pkt), GFP_KERNEL);
		if (!pkt) {
			kfree(pkt);
			return -ENOMEM;
		}
		e = get_dphdcp_state(dphdcp, pkt);
		if (copy_to_user((void __user *)arg, pkt, sizeof(*pkt))) {
			kfree(pkt);
			return -EFAULT;
		}
		kfree(pkt);
		return e;

	}
	return e;
}

static int dphdcp_dev_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *miscdev = filp->private_data;
	struct tegra_dphdcp *dphdcp =
		container_of(miscdev, struct tegra_dphdcp, miscdev);
	filp->private_data = dphdcp;
	return 0;
}

static int dphdcp_dev_release(struct inode *inode, struct file *filp)
{
	filp->private_data = NULL;
	return 0;
}

static const struct file_operations dphdcp_fops = {
	.owner			= THIS_MODULE,
	.llseek			= no_llseek,
	.unlocked_ioctl = dphdcp_dev_ioctl,
	.open			= dphdcp_dev_open,
	.release        = dphdcp_dev_release,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= dphdcp_dev_ioctl,
#endif
};
/* we only support one AP right now, so should only call this once. */
struct tegra_dphdcp *tegra_dphdcp_create(struct tegra_dc_dp_data *dp,
			int id, int bus)
{
	struct tegra_dphdcp *dphdcp;
	int e;
	int num_heads;

	num_heads = tegra_dc_get_numof_dispheads();

	if (id >= num_heads) {
		dphdcp_err("head id greater than what's available!");
		return ERR_PTR(-EMFILE);
	}

	/* ensure memory allocated once */
	if (!dphdcp_head) {
		dphdcp_head =
		kzalloc(sizeof(void *) * num_heads, GFP_KERNEL);
		if (!dphdcp_head)
			return ERR_PTR(-ENOMEM);
	}

	dphdcp = dphdcp_head[id];
	/* do not allow multiple node creation */
	if (dphdcp)
		return ERR_PTR(-EMFILE);

	dphdcp = kzalloc(sizeof(*dphdcp), GFP_KERNEL);
	if (!dphdcp)
		return ERR_PTR(-ENOMEM);

	dphdcp->id = id;
	snprintf(dphdcp->name, sizeof(dphdcp->name), "nvhdcp%u", id);
	dphdcp->dp = dp;
	mutex_init(&dphdcp->lock);

	strlcpy(dphdcp->info.type, dphdcp->name, sizeof(dphdcp->info.type));
	dphdcp->bus = bus;
	dphdcp->fail_count = 0;
	dphdcp->max_retries = HDCP_MAX_RETRIES;
	dphdcp->hpd = 0;
	atomic_set(&dphdcp->policy, dp->dc->pdata->default_out->hdcp_policy);

	dphdcp->state = STATE_UNAUTHENTICATED;

	dphdcp->downstream_wq = create_singlethread_workqueue(dphdcp->name);

	dphdcp->miscdev.minor = MISC_DYNAMIC_MINOR;
	dphdcp->miscdev.name = dphdcp->name;
	dphdcp->miscdev.fops = &dphdcp_fops;
	/* register miscellaneous device */
	e = misc_register(&dphdcp->miscdev);
	if (e) {
		dphdcp_err("cannot register\n");
		goto free_workqueue;
	}

	dphdcp_head[id] = dphdcp;
	dphdcp_vdbg("%s(): created misc device %s\n", __func__, dphdcp->name);

	return dphdcp;

free_workqueue:
	destroy_workqueue(dphdcp->downstream_wq);
	return ERR_PTR(e);
}

void tegra_dphdcp_destroy(struct tegra_dphdcp *dphdcp)
{
	misc_deregister(&dphdcp->miscdev);
	tegra_dphdcp_off(dphdcp);
	destroy_workqueue(dphdcp->downstream_wq);
	dphdcp_head[dphdcp->id] = NULL;
	kfree(dphdcp);
}

#ifdef CONFIG_TEGRA_DEBUG_DP_HDCP
/* show current maximum number of retries for HDCP DP authentication */
static int tegra_dp_max_retries_dbg_show(struct seq_file *m, void *unused)
{
	struct tegra_dphdcp *dphdcp = m->private;

	if (WARN_ON(!dphdcp))
		return -EINVAL;

	seq_printf(m, "hdcp max_retries value: %d\n", dphdcp->max_retries);

	return 0;
}

/* show current hotplug state */
static int tegra_dp_hotplug_dbg_show(struct seq_file *m, void *unused)
{
	struct tegra_dphdcp *dphdcp = m->private;

	if (WARN_ON(!dphdcp))
		return -EINVAL;

	seq_printf(m, "hotplug value set to: %d\n", dphdcp->hpd);

	return 0;
}

/*
 * sw control for hdcp max retries.
 * 5 is the normal number of max retries.
 * 1 is the minimum number of retries.
 * 5 is the maximum number of retries.
 * sw should keep the number of retries to 5 until unless a change is required
 */
static ssize_t tegra_dp_max_retries_dbg_write(struct file *file,
						const char __user *addr,
						size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data;
	struct tegra_dphdcp *dphdcp = m->private;
	u8 new_max_retries;
	int ret;

	if (WARN_ON(!dphdcp))
		return -EINVAL;

	ret = kstrtou8_from_user(addr, len, 6, &new_max_retries);
	if (ret < 0)
		return ret;

	if (new_max_retries >= HDCP_MIN_RETRIES &&
		new_max_retries <= HDCP_MAX_RETRIES)
		dphdcp->max_retries = new_max_retries;

	return len;
}

/*
 * sw control for hotplug on and off
 * 1: turn hotplug on
 * 0: turn hotplug off
 */
static ssize_t tegra_dp_hotplug_dbg_write(struct file *file,
					const char __user *addr,
					size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data;
	struct tegra_dphdcp *dphdcp = m->private;
	u8 new_hpd;
	int ret;

	if (WARN_ON(!dphdcp))
		return -EINVAL;

	ret = kstrtou8_from_user(addr, len, 6, &new_hpd);
	if (ret < 0)
		return ret;

	if (new_hpd > 0) {
		/* start downstream_worker */
		dphdcp_set_plugged(dphdcp, true);
		tegra_dphdcp_on(dphdcp);
		dphdcp->hpd = new_hpd;
	}
	return len;
}

static int tegra_dp_max_retries_dbg_open(struct inode *inode,
		struct file *file)
{
	if (!inode || !file)
		return -EINVAL;

	return single_open(file, tegra_dp_max_retries_dbg_show,
			inode->i_private);
}

static int tegra_dp_hotplug_dbg_open(struct inode *inode,
		struct file *file)
{
	if (!inode || !file)
		return -EINVAL;

	return single_open(file, tegra_dp_hotplug_dbg_show,
			inode->i_private);
}

static const struct file_operations tegra_dp_max_retries_dbg_ops = {
	.open = tegra_dp_max_retries_dbg_open,
	.read = seq_read,
	.write = tegra_dp_max_retries_dbg_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations tegra_dp_hotplug_dbg_ops = {
	.open = tegra_dp_hotplug_dbg_open,
	.read = seq_read,
	.write = tegra_dp_hotplug_dbg_write,
	.llseek = seq_lseek,
	.release = single_release,
};

void tegra_dphdcp_debugfs_init(struct tegra_dphdcp *dphdcp)
{
	struct dentry *dir, *ret;

	if (!dphdcp)
		goto fail;

	dir = debugfs_create_dir("tegra_dphdcp",  NULL);

	ret = debugfs_create_file("max_retries", 0444, dir,
				dphdcp, &tegra_dp_max_retries_dbg_ops);
	if (IS_ERR_OR_NULL(ret))
		goto fail;

	ret = debugfs_create_file("hotplug", 0444, dir,
				dphdcp, &tegra_dp_hotplug_dbg_ops);
	if (IS_ERR_OR_NULL(ret))
		goto fail;

fail:
	debugfs_remove_recursive(dir);
}
#else
void tegra_dphdcp_debugfs_init(struct tegra_dphdcp *dphdcp)
{
}
#endif
