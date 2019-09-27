/*
 * arch/arm/mach-tegra/isomgr.c
 *
 * Copyright (c) 2012-2018, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#define pr_fmt(fmt)	"%s(): " fmt, __func__

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/compiler.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/printk.h>
#include <linux/err.h>
#include <linux/kref.h>
#include <linux/sched.h>
#include <linux/version.h>
#include <soc/tegra/chip-id.h>
#include <asm/processor.h>
#include <asm/current.h>

#include <linux/platform/tegra/isomgr.h>

#include <linux/platform/tegra/emc_bwmgr.h>
#ifdef CONFIG_COMMON_CLK
#include <linux/platform/tegra/bwmgr_mc.h>
#else
#include <linux/platform/tegra/mc.h>
#include <linux/clk.h>
#include <linux/platform/tegra/clock.h>
#endif

#define CREATE_TRACE_POINTS
#include <trace/events/isomgr.h>

#define ISOMGR_SYSFS_VERSION 0	/* increment on change */

#define VALIDATE_HANDLE() \
do { \
	if (unlikely(!cp || !is_client_valid(client) || \
		     cp->magic != ISOMGR_MAGIC)) { \
		pr_err("bad handle %p\n", handle); \
		goto validation_fail; \
	} \
} while (0)

#define VALIDATE_CLIENT() \
do { \
	if (unlikely(!is_client_valid(client))) { \
		pr_err("invalid client %d\n", client); \
		goto validation_fail; \
	} \
} while (0)


#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#define OBJ_REF_SET		refcount_set
#define OBJ_REF_READ		refcount_read
#define OBJ_REF_INC_NOT_ZERO	refcount_inc_not_zero
#else
#define OBJ_REF_SET		atomic_set
#define OBJ_REF_READ		atomic_read
#define OBJ_REF_INC_NOT_ZERO	atomic_inc_not_zero
#endif

/* To allow test code take over control */
static bool test_mode;

char *cname[] = {
	"disp_0",
	"disp_1",
	"disp_2",
	"vi_0",
	"vi_1",
	"isp_a",
	"isp_b",
	"bbc_0",
	"tegra_camera_ctrl",
	"ape_adma",
	"eqos",
	"unknown"
};

struct isoclient_info *isoclient_info;
/*platform specific flag for requesting max emc floor req for camera client*/
u8 isomgr_camera_max_floor_req;
int isoclients;
bool client_valid[TEGRA_ISO_CLIENT_COUNT];
struct isomgr_client isomgr_clients[TEGRA_ISO_CLIENT_COUNT];
struct isomgr isomgr = {
	.max_iso_bw = CONFIG_TEGRA_ISOMGR_POOL_KB_PER_SEC,
	.avail_bw = CONFIG_TEGRA_ISOMGR_POOL_KB_PER_SEC,
};

/* get minimum MC frequency for client that can support this BW and LT */
static inline u32 mc_min_freq(u32 ubw, u32 ult) /* in KB/sec and usec */
{
	unsigned int min_freq = 0;

	/* ult==0 means ignore LT (effectively infinite) */
	if (ubw == 0)
		goto out;

#ifdef CONFIG_COMMON_CLK
	min_freq = bwmgr_bw_to_freq(ubw);
#else
	min_freq = tegra_emc_bw_to_freq_req(ubw);
#endif

out:
	return min_freq; /* return value in KHz*/
}

/* get dvfs switch latency for client requiring this frequency */
static inline u32 mc_dvfs_latency(u32 ufreq)
{
#ifdef CONFIG_COMMON_CLK
	return bwmgr_dvfs_latency(ufreq);
#else
	return tegra_emc_dvfs_latency(ufreq); /* return value in usec */
#endif
}

static inline bool isomgr_lock(void)
{
	/* disallow rentrance, avoid deadlock */
	if (unlikely(isomgr.task == current)) {
		pr_err("isomgr: lock deadlock ?\n");
		dump_stack();
		return false;
	}
	mutex_lock(&isomgr.lock);
	isomgr.task = current;
	return true;
}

static inline bool isomgr_unlock(void)
{
	/* detect mismatched calls */
	if (unlikely(isomgr.task != current)) {
		pr_err("isomgr: unlock mismatch ?\n");
		dump_stack();
		return false;
	}
	isomgr.task = NULL;
	mutex_unlock(&isomgr.lock);
	return true;
}

/* call with isomgr_lock held. */
static void update_mc_clock(void)
{
	int i;
	u64 floor_freq;
	unsigned long emc_max_rate = tegra_bwmgr_get_max_emc_rate();

	/*If we get the lock, it means lock was not taken and hence we return*/
	if (unlikely(mutex_trylock(&isomgr.lock))) {
		pr_err("isomgr: %s called without lock\n", __func__);
		WARN_ON(true);
		return;
	}
	/* determine worst case freq to satisfy LT */
	isomgr.lt_mf = 0;
	for (i = 0; i < TEGRA_ISO_CLIENT_COUNT; i++)
		isomgr.lt_mf = max(isomgr.lt_mf, isomgr_clients[i].real_mf);

	/* request the floor freq to satisfy LT */
	if (isomgr.lt_mf_rq != isomgr.lt_mf) {
#ifdef CONFIG_COMMON_CLK
		if (!tegra_bwmgr_set_emc(isomgr.bwmgr_handle,
				isomgr.lt_mf * 1000,
				TEGRA_BWMGR_SET_EMC_FLOOR))
			isomgr.lt_mf_rq = isomgr.lt_mf;
#else
		if (!clk_set_rate(isomgr.emc_clk, isomgr.lt_mf * 1000)) {

			if (isomgr.lt_mf_rq == 0)
				clk_enable(isomgr.emc_clk);
			isomgr.lt_mf_rq = isomgr.lt_mf;
			if (isomgr.lt_mf_rq == 0)
				clk_disable(isomgr.emc_clk);
		}
#endif
	}

	for (i = 0; i < TEGRA_ISO_CLIENT_COUNT; i++) {

		/*max emc floor req when camera is active for t194 */
		if (i == TEGRA_ISO_CLIENT_TEGRA_CAMERA) {
			if (isomgr_camera_max_floor_req) {
				if (isomgr_clients[i].real_mf)
					tegra_bwmgr_set_emc(
						isomgr_clients[i].bwmgr_handle,
						emc_max_rate,
						TEGRA_BWMGR_SET_EMC_FLOOR);
				else
					tegra_bwmgr_set_emc(
						isomgr_clients[i].bwmgr_handle,
						0,
						TEGRA_BWMGR_SET_EMC_FLOOR);
			}
		}

		if (isomgr_clients[i].real_mf != isomgr_clients[i].real_mf_rq) {
			/* Ignore clocks for clients that are non-existent. */
#ifdef CONFIG_COMMON_CLK
			if (!isomgr_clients[i].bwmgr_handle)
				continue;
			/* Each client's request is limited to
			 * limit_bw_percentage% of current DRAM BW. A floor
			 * request is made by each client to hold DRAM freq
			 * high enough such that
			 * DRAM_freq > client's req_bw/limit_bw_percentage
			 */
			if (isomgr_clients[i].limit_bw_percentage != 100) {
				floor_freq = (u64)isomgr_clients[i].real_mf
					* 1000 * 100;
				floor_freq = floor_freq /
				(u64)isomgr_clients[i].limit_bw_percentage;
				tegra_bwmgr_set_emc(
					isomgr_clients[i].bwmgr_handle,
					floor_freq,
					TEGRA_BWMGR_SET_EMC_FLOOR);
			}
			if (!tegra_bwmgr_set_emc(
					isomgr_clients[i].bwmgr_handle,
					isomgr_clients[i].real_mf * 1000,
					TEGRA_BWMGR_SET_EMC_SHARED_BW_ISO))
				isomgr_clients[i].real_mf_rq =
					isomgr_clients[i].real_mf;
#else
			if (!isomgr_clients[i].emc_clk)
				continue;

			if (clk_set_rate(isomgr_clients[i].emc_clk,
					 isomgr_clients[i].real_mf * 1000))
				continue;

			if (isomgr_clients[i].real_mf_rq == 0)
				clk_enable(isomgr_clients[i].emc_clk);
			isomgr_clients[i].real_mf_rq =
				isomgr_clients[i].real_mf;
			if (isomgr_clients[i].real_mf_rq == 0)
				clk_disable(isomgr_clients[i].emc_clk);
#endif
		}
	}
}

static void purge_isomgr_client(struct isomgr_client *cp)
{
	cp->magic = 0;
	OBJ_REF_SET(&cp->kref.refcount, 0);
	cp->dedi_bw = 0;
	cp->rsvd_bw = 0;
	cp->real_bw = 0;
	cp->rsvd_mf = 0;
	cp->real_mf = 0;
	cp->renegotiate = NULL;
	cp->realize = false;
	cp->priv = NULL;
	cp->sleep_bw = 0;
	cp->margin_bw = 0;
}

/* This function should be called with isomgr lock */
static void unregister_iso_client(struct kref *kref)
{
	struct isomgr_client *cp = container_of(kref,
					struct isomgr_client, kref);
	int client = cp - &isomgr_clients[0];

	/*If we get the lock, it means lock was not taken and hence we return*/
	if (unlikely(mutex_trylock(&isomgr.lock))) {
		pr_err("isomgr: unregister_iso_client called without lock\n");
		WARN_ON(true);
		return;
	}

	trace_tegra_isomgr_unregister_iso_client(cname[client], "enter");
	if (unlikely(cp->realize)) {
		pr_err
		("isomgr: %s called while realize in progress\n", __func__);
		goto fail;
	}

	if (isomgr.ops->isomgr_plat_unregister)
		isomgr.ops->isomgr_plat_unregister(cp);

	isomgr.dedi_bw -= cp->dedi_bw;
	purge_isomgr_client(cp);
	update_mc_clock();

	trace_tegra_isomgr_unregister_iso_client(cname[client], "exit");
	return;

fail:
	trace_tegra_isomgr_unregister_iso_client(cname[client], "exit fail");
}

static bool is_client_valid(enum tegra_iso_client client)
{
	if (unlikely(client < 0 ||
			client >= TEGRA_ISO_CLIENT_COUNT ||
			!client_valid[client] ||
#ifdef CONFIG_COMMON_CLK
			!isomgr_clients[client].bwmgr_handle))
#else
			!isomgr_clients[client].emc_clk))
#endif
		return false;
	return true;
}

static tegra_isomgr_handle __tegra_isomgr_register(
			enum tegra_iso_client client, u32 udedi_bw,
			tegra_isomgr_renegotiate renegotiate, void *priv)
{
	s32 dedi_bw = udedi_bw;
	bool ret = 0;
	struct isomgr_client *cp = NULL;

	VALIDATE_CLIENT();
	trace_tegra_isomgr_register(client, dedi_bw, renegotiate,
		priv, cname[client], "enter");

	if (unlikely(!udedi_bw && !renegotiate))
		goto validation_fail;

	if (!isomgr_lock()) {
		pr_err("isomgr: %s failed for %s\n",
			__func__, cname[client]);
		goto validation_fail;
	}
	cp = &isomgr_clients[client];

	if (unlikely(OBJ_REF_READ(&cp->kref.refcount)))
		goto fail_unlock;

	if (isomgr.ops->isomgr_plat_register) {
		ret = isomgr.ops->isomgr_plat_register(dedi_bw, client);
		if (!ret)
			goto fail_unlock;
	}

	purge_isomgr_client(cp);
	cp->magic = ISOMGR_MAGIC;
	kref_init(&cp->kref);
	cp->dedi_bw = dedi_bw;
	cp->renegotiate = renegotiate;
	cp->priv = priv;
	isomgr.dedi_bw += dedi_bw;

	if (!isomgr_unlock()) {
		pr_err("isomgr: %s failed for %s\n",
			__func__, cname[client]);
		goto validation_fail;
	}
	trace_tegra_isomgr_register(client, dedi_bw, renegotiate,
		priv, cname[client], "exit");
	return (tegra_isomgr_handle)cp;

fail_unlock:
	if (!isomgr_unlock()) {
		pr_err("isomgr: %s failed for %s\n",
			__func__, cname[client]);
	}
validation_fail:
	trace_tegra_isomgr_register(client, dedi_bw, renegotiate,
		priv, cname[client], "inv_args_exit");
	return ERR_PTR(-EINVAL);
}

/**
 * tegra_isomgr_register - register an ISO BW client.
 *
 * @client	client to register as an ISO client.
 * @udedi_bw	minimum bw client can work at. This bw is guarnteed to be
 *		available for client when ever client need it. Client can
 *		always request for more bw and client can get it based on
 *		availability of bw in the system. udedi_bw is specified in KB.
 * @renegotiate	callback function to be called to renegotiate for bw.
 *		client with no renegotiate callback provided can't allocate
 *		bw more than udedi_bw.
 *		Client with renegotiate callback can allocate more than
 *		udedi_bw and release it during renegotiate callback, when
 *		other clients in the system need their bw back.
 *		renegotiate callback is called in two cases. 1. The isomgr
 *		has excess bw, checking client to see if they need more bw.
 *		2. The isomgr is out of bw and other clients need their udedi_bw
 *		back. In this case, the client, which is using higher bw need to
 *		release the bw and fallback to low(udedi_bw) bw use case.
 * @priv	pointer to renegotiate callback function.
 *
 * @return	returns valid handle on successful registration.
 * @retval	-EINVAL invalid arguments passed.
 */
tegra_isomgr_handle tegra_isomgr_register(enum tegra_iso_client client,
					  u32 udedi_bw,
					  tegra_isomgr_renegotiate renegotiate,
					  void *priv)
{
	if (test_mode)
		return (tegra_isomgr_handle)1;
	return __tegra_isomgr_register(client, udedi_bw, renegotiate, priv);
}
EXPORT_SYMBOL(tegra_isomgr_register);

static void __tegra_isomgr_unregister(tegra_isomgr_handle handle)
{
	struct isomgr_client *cp = (struct isomgr_client *)handle;
	int client = cp - &isomgr_clients[0];

	VALIDATE_HANDLE();
	if (!isomgr_lock()) {
		pr_err("isomgr: %s failed for %s\n",
			__func__, cname[client]);
		goto validation_fail;
	}
	trace_tegra_isomgr_unregister(handle, cname[client]);
	kref_put(&cp->kref, unregister_iso_client);
	if (!isomgr_unlock()) {
		pr_err("isomgr: %s failed for %s\n",
			__func__, cname[client]);
	}
validation_fail:
	return;
}

/**
 * tegra_isomgr_unregister - unregister an ISO BW client.
 *
 * @handle	handle acquired during tegra_isomgr_register.
 */
void tegra_isomgr_unregister(tegra_isomgr_handle handle)
{
	if (test_mode)
		return;
	__tegra_isomgr_unregister(handle);
}
EXPORT_SYMBOL(tegra_isomgr_unregister);

static u32 __tegra_isomgr_reserve(tegra_isomgr_handle handle,
			 u32 ubw, u32 ult)
{
	s32 bw = ubw;
	bool ret = 0;
	u32 mf, dvfs_latency = 0;
	struct isomgr_client *cp = (struct isomgr_client *) handle;
	int client = cp - &isomgr_clients[0];

	VALIDATE_HANDLE();

	if (!isomgr_lock()) {
		pr_err("isomgr: %s failed for %s\n",
			__func__, cname[client]);
		goto validation_fail;
	}
	if (unlikely(!OBJ_REF_INC_NOT_ZERO(&cp->kref.refcount)))
		goto handle_unregistered;

	if (cp->rsvd_bw == ubw && cp->lti == ult) {
		kref_put(&cp->kref, unregister_iso_client);
		if (!isomgr_unlock()) {
			pr_err("isomgr: %s failed for %s\n",
				__func__, cname[client]);
			goto validation_fail;
		}
		return cp->lto;
	}

	trace_tegra_isomgr_reserve(handle, ubw, ult, cname[client], "enter");

	if (unlikely(cp->realize))
		goto out;

	if (unlikely(!cp->renegotiate && bw > cp->dedi_bw))
		goto out;

	if (isomgr.ops->isomgr_plat_reserve) {
		ret = isomgr.ops->isomgr_plat_reserve(cp, bw,
				(enum tegra_iso_client)client);
		if (!ret)
			goto out;
	}

	/* Look up MC's min freq that could satisfy requested BW and LT */
	mf = mc_min_freq(ubw, ult);
	/* Look up MC's dvfs latency at min freq */
	dvfs_latency = mc_dvfs_latency(mf);

	cp->lti = ult;		/* remember client spec'd LT (usec) */
	cp->lto = dvfs_latency;	/* remember MC calculated LT (usec) */
	cp->rsvd_mf = mf;	/* remember associated min freq */
	cp->rsvd_bw = bw;
out:
	kref_put(&cp->kref, unregister_iso_client);
	if (!isomgr_unlock()) {
		pr_err("isomgr: %s failed for %s\n",
			__func__, cname[client]);
		goto validation_fail;
	}
	trace_tegra_isomgr_reserve(handle, ubw, ult, cname[client],
		dvfs_latency ? "exit" : "rsrv_fail_exit");
	return dvfs_latency;
handle_unregistered:
	if (!isomgr_unlock()) {
		pr_err("isomgr: %s failed for %s\n",
			__func__, cname[client]);
		goto validation_fail;
	}
	trace_tegra_isomgr_reserve(handle, ubw, ult,
		cname[client], "inv_handle_exit");
	return dvfs_latency;
validation_fail:
	trace_tegra_isomgr_reserve(handle, ubw, ult, "unk", "inv_handle_exit");
	return dvfs_latency;
}

/**
 * tegra_isomgr_reserve - reserve bw for the ISO client.
 *
 * @handle	handle acquired during tegra_isomgr_register.
 * @ubw		bandwidth in KBps.
 * @ult		latency that can be tolerated by client in usec.
 *
 * returns dvfs latency thresh in usec.
 * return 0 indicates that reserve failed.
 */
u32 tegra_isomgr_reserve(tegra_isomgr_handle handle,
			 u32 ubw, u32 ult)
{
	if (test_mode)
		return 1;
	return __tegra_isomgr_reserve(handle, ubw, ult);
}
EXPORT_SYMBOL(tegra_isomgr_reserve);

static u32 __tegra_isomgr_realize(tegra_isomgr_handle handle)
{
	u32 dvfs_latency = 0;
	bool ret = 0;
	struct isomgr_client *cp = (struct isomgr_client *) handle;
	int client = cp - &isomgr_clients[0];

	VALIDATE_HANDLE();

	if (!isomgr_lock()) {
		pr_err("isomgr: %s failed for %s\n",
			__func__, cname[client]);
		goto validation_fail;
	}
	if (unlikely(!OBJ_REF_INC_NOT_ZERO(&cp->kref.refcount)))
		goto handle_unregistered;

	if (cp->rsvd_bw == cp->real_bw && cp->rsvd_mf == cp->real_mf) {
		kref_put(&cp->kref, unregister_iso_client);
		if (!isomgr_unlock()) {
			pr_err("isomgr: %s failed for %s\n",
				__func__, cname[client]);
			goto validation_fail;
		}
		return cp->lto;
	}

	trace_tegra_isomgr_realize(handle, cname[client], "enter");

	if (isomgr.ops->isomgr_plat_realize) {
		ret = isomgr.ops->isomgr_plat_realize(cp);
		if (!ret)
			goto out;
	}

	dvfs_latency = (u32)cp->lto;
	cp->realize = false;
	update_mc_clock();

out:
	kref_put(&cp->kref, unregister_iso_client);
	if (!isomgr_unlock()) {
		pr_err("isomgr: %s failed for %s\n",
			__func__, cname[client]);
		goto validation_fail;
	}
	trace_tegra_isomgr_realize(handle, cname[client],
		dvfs_latency ? "exit" : "real_fail_exit");
	return dvfs_latency;
handle_unregistered:
	if (!isomgr_unlock()) {
		pr_err("isomgr: %s failed for %s\n",
			__func__, cname[client]);
		goto validation_fail;
	}
	trace_tegra_isomgr_realize(handle, cname[client], "inv_handle_exit");
	return dvfs_latency;
validation_fail:
	trace_tegra_isomgr_realize(handle, "unk", "inv_handle_exit");
	return dvfs_latency;
}

/**
 * tegra_isomgr_realize - realize the bw reserved by client.
 *
 * @handle	handle acquired during tegra_isomgr_register.
 *
 * returns dvfs latency thresh in usec.
 * return 0 indicates that realize failed.
 */
u32 tegra_isomgr_realize(tegra_isomgr_handle handle)
{
	if (test_mode)
		return 1;
	return __tegra_isomgr_realize(handle);
}
EXPORT_SYMBOL(tegra_isomgr_realize);

static int __tegra_isomgr_set_margin(enum tegra_iso_client client,
					u32 bw, bool wait)
{
	int ret = -EINVAL;
	s32 high_bw;
	struct isomgr_client *cp = NULL;

	trace_tegra_isomgr_set_margin(client, bw, wait, "enter");
	VALIDATE_CLIENT();

	if (!isomgr_lock()) {
		pr_err("isomgr: %s failed for %s\n",
			__func__, cname[client]);
		goto validation_fail;
	}
	cp = &isomgr_clients[client];
	if (unlikely(!OBJ_REF_INC_NOT_ZERO(&cp->kref.refcount)))
		goto handle_unregistered;

	if (bw > cp->dedi_bw)
		goto out;

	if (bw <= cp->real_bw) {
		if (cp->margin_bw > cp->real_bw)
			isomgr.avail_bw += cp->margin_bw - cp->real_bw;
		cp->margin_bw = bw;
	} else if (bw <= cp->margin_bw) {
		if (unlikely(cp->margin_bw > cp->real_bw)) {
			pr_err("isomgr: set_margin: margin_bw > real_bw\n");
			ret = -EINVAL;
			goto out;
		}
		isomgr.avail_bw += cp->margin_bw - bw;
		cp->margin_bw = bw;
		if (unlikely(cp->margin_bw > cp->real_bw)) {
			pr_err("isomgr: set_margin: margin_bw > real_bw\n");
			ret = -EINVAL;
			goto out;
		}
	} else if (bw > cp->margin_bw) {
		high_bw = (cp->margin_bw > cp->real_bw) ?
				cp->margin_bw : cp->real_bw;
		if (bw - high_bw <= isomgr.avail_bw - isomgr.sleep_bw) {
			isomgr.avail_bw -= bw - high_bw;
			cp->margin_bw = bw;
		} else {
			ret = -EINVAL;
			goto out;
		}
	}
	ret = 0;
out:
	kref_put(&cp->kref, unregister_iso_client);
	if (!isomgr_unlock()) {
		pr_err("isomgr: %s failed for %s\n",
			__func__, cname[client]);
		goto validation_fail;
	}
	trace_tegra_isomgr_set_margin(client, bw, wait,
					ret ? "fail_exit" : "exit");
	return ret;
handle_unregistered:
	if (!isomgr_unlock()) {
		pr_err("isomgr: %s failed for %s\n",
			__func__, cname[client]);
	}
validation_fail:
	trace_tegra_isomgr_set_margin(client, bw, wait, "inv_arg_fail");
	return ret;
}

/**
 * This sets bw aside for the client specified.
 * This bw can never be used for other clients needs.
 * margin bw, if not zero, should always be greater than equal to
 * reserved/realized bw.
 *
 * @client client
 * @bw bw margin KB.
 * @wait if true and bw is not available, it would wait till bw is available.
 *	  if false, it would return immediately with success or failure.
 *
 * @retval  0 operation is successful.
 * @retval -ENOMEM Iso Bw requested is not avialable.
 * @retval -EINVAL Invalid arguments, bw is less than reserved/realized bw.
 */
int tegra_isomgr_set_margin(enum tegra_iso_client client, u32 bw, bool wait)
{
	if (test_mode)
		return 0;
	return __tegra_isomgr_set_margin(client, bw, wait);
}
EXPORT_SYMBOL(tegra_isomgr_set_margin);

static int __tegra_isomgr_get_imp_time(enum tegra_iso_client client, u32 bw)
{
	int ret = -EINVAL;

	if (unlikely(!is_client_valid(client)))
		return ret;

	/* FIXME: get this from renegotiable clients(display driver). */
	ret = 100;
	if (isomgr.avail_bw >= bw)
		ret = 0;
	trace_tegra_isomgr_get_imp_time(client, bw, ret, cname[client]);
	return ret;
}

/**
 * Returns the imp time required to realize the bw request.
 * The time returned is an approximate. It is possible that imp
 * time is returned as zero and still realize would be blocked for
 * non-zero time in realize call.
 *
 * @client	client id
 * @bw		bw in KB/sec
 *
 * @returns	time in milliseconds.
 * @retval	-EINVAL, client id is invalid.
 */
int tegra_isomgr_get_imp_time(enum tegra_iso_client client, u32 bw)
{
	if (test_mode)
		return 0;
	return __tegra_isomgr_get_imp_time(client, bw);
}
EXPORT_SYMBOL(tegra_isomgr_get_imp_time);

static u32 __tegra_isomgr_get_available_iso_bw(void)
{
	trace_tegra_isomgr_get_available_iso_bw(isomgr.avail_bw);
	return isomgr.avail_bw;
}

/**
 * Returns available iso bw at the time of calling this API.
 *
 * @returns	available bw in KB/sec.
 */
u32 tegra_isomgr_get_available_iso_bw(void)
{
	if (test_mode)
		return UINT_MAX;
	return __tegra_isomgr_get_available_iso_bw();
}
EXPORT_SYMBOL(tegra_isomgr_get_available_iso_bw);

static u32 __tegra_isomgr_get_total_iso_bw(enum tegra_iso_client client)
{
	u32 bw;

	if (isomgr.ops->isomgr_max_iso_bw) /*t19x and later*/
		bw = isomgr.ops->isomgr_max_iso_bw(client);
	else /*t18x and before*/
		bw = isomgr.max_iso_bw;

	trace_tegra_isomgr_get_total_iso_bw(bw);
	return bw;
}

/**
 * Returns total iso bw in the system.
 *
 * @returns	total bw in KB/sec.
 */
u32 tegra_isomgr_get_total_iso_bw(enum tegra_iso_client client)
{
	if (test_mode)
		return UINT_MAX;
	return __tegra_isomgr_get_total_iso_bw(client);
}
EXPORT_SYMBOL(tegra_isomgr_get_total_iso_bw);

#ifdef CONFIG_TEGRA_ISOMGR_SYSFS
static ssize_t isomgr_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf);

static const struct kobj_attribute lt_mf_attr =
	__ATTR(lt_mf, 0444, isomgr_show, NULL);
static const struct kobj_attribute avail_bw_attr =
	__ATTR(avail_bw, 0444, isomgr_show, NULL);
static const struct kobj_attribute max_iso_bw_attr =
	__ATTR(max_iso_bw, 0444, isomgr_show, NULL);
static const struct kobj_attribute version_attr =
	__ATTR(version, 0444, isomgr_show, NULL);

static const struct attribute *isomgr_attrs[] = {
	&lt_mf_attr.attr,
	&avail_bw_attr.attr,
	&max_iso_bw_attr.attr,
	&version_attr.attr,
	NULL
};

static ssize_t isomgr_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	ssize_t rval = 0;

	if (attr == &lt_mf_attr)
		rval = sprintf(buf, "%dKHz\n", isomgr.lt_mf);
	else if (attr == &avail_bw_attr)
		rval = sprintf(buf, "%dKB\n", isomgr.avail_bw);
	else if (attr == &max_iso_bw_attr)
		rval = sprintf(buf, "%dKB\n", isomgr.max_iso_bw);
	else if (attr == &version_attr)
		rval = sprintf(buf, "%d\n", ISOMGR_SYSFS_VERSION);
	return rval;
}

static ssize_t isomgr_client_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	int client = ((char *)attr - (char *)isomgr_clients) /
			sizeof(struct isomgr_client);
	struct isomgr_client *cp =
			(struct isomgr_client *)&isomgr_clients[client];
	ssize_t rval = 0;

	if (attr == &cp->client_attrs.dedi_bw)
		rval = sprintf(buf, "%dKB\n", cp->dedi_bw);
	else if (attr == &cp->client_attrs.rsvd_bw)
		rval = sprintf(buf, "%dKB\n", cp->rsvd_bw);
	else if (attr == &cp->client_attrs.real_bw)
		rval = sprintf(buf, "%dKB\n", cp->real_bw);
	else if (attr == &cp->client_attrs.lti)
		rval = sprintf(buf, "%dus\n", cp->lti);
	else if (attr == &cp->client_attrs.lto)
		rval = sprintf(buf, "%dus\n", cp->lto);
	else if (attr == &cp->client_attrs.rsvd_mf)
		rval = sprintf(buf, "%dKHz\n", cp->rsvd_mf);
	else if (attr == &cp->client_attrs.real_mf)
		rval = sprintf(buf, "%dKHz\n", cp->real_mf);
	else if (attr == &cp->client_attrs.sleep_bw)
		rval = sprintf(buf, "%dKB\n", cp->sleep_bw);
	else if (attr == &cp->client_attrs.margin_bw)
		rval = sprintf(buf, "%dKB\n", cp->margin_bw);
	return rval;
}

static const struct isomgr_client_attrs client_attrs = {
	__ATTR(dedi_bw, 0444, isomgr_client_show, NULL),
	__ATTR(rsvd_bw, 0444, isomgr_client_show, NULL),
	__ATTR(real_bw, 0444, isomgr_client_show, NULL),
	__ATTR(lti,     0444, isomgr_client_show, NULL),
	__ATTR(lto,     0444, isomgr_client_show, NULL),
	__ATTR(rsvd_mf, 0444, isomgr_client_show, NULL),
	__ATTR(real_mf, 0444, isomgr_client_show, NULL),
	__ATTR(sleep_bw, 0444, isomgr_client_show, NULL),
	__ATTR(margin_bw, 0444, isomgr_client_show, NULL),
};

#define NCATTRS (sizeof(client_attrs) / sizeof(struct kobj_attribute))
static const struct attribute *client_attr_list[][NCATTRS+1] = {
#define CLIENT_ATTR(i)\
	{\
		&isomgr_clients[i].client_attrs.dedi_bw.attr,\
		&isomgr_clients[i].client_attrs.rsvd_bw.attr,\
		&isomgr_clients[i].client_attrs.real_bw.attr,\
		&isomgr_clients[i].client_attrs.lti.attr,\
		&isomgr_clients[i].client_attrs.lto.attr,\
		&isomgr_clients[i].client_attrs.rsvd_mf.attr,\
		&isomgr_clients[i].client_attrs.real_mf.attr,\
		&isomgr_clients[i].client_attrs.sleep_bw.attr,\
		&isomgr_clients[i].client_attrs.margin_bw.attr,\
		NULL\
	},
	CLIENT_ATTR(0)
	CLIENT_ATTR(1)
	CLIENT_ATTR(2)
	CLIENT_ATTR(3)
	CLIENT_ATTR(4)
	CLIENT_ATTR(5)
	CLIENT_ATTR(6)
	CLIENT_ATTR(7)
	CLIENT_ATTR(8)
	CLIENT_ATTR(9)
	CLIENT_ATTR(10)
	CLIENT_ATTR(11)
};

static void isomgr_create_client(int client, const char *name)
{
	struct isomgr_client *cp = &isomgr_clients[client];

	/* If this error hits, more CLIENT_ATTR(x) need to be added
	 * in the above array client_attr_list.
	 */
	BUILD_BUG_ON(TEGRA_ISO_CLIENT_COUNT > 11);
	if (unlikely(!isomgr.kobj)) {
		pr_err("isomgr: create_client failed, isomgr.kobj is null\n");
		return;
	}
	if (unlikely(cp->client_kobj)) {
		pr_err("isomgr: create_client failed, client_kobj is null\n");
		return;
	}
	cp->client_kobj = kobject_create_and_add(name, isomgr.kobj);
	if (!cp->client_kobj) {
		pr_err("failed to create sysfs client dir\n");
		return;
	}
	cp->client_attrs = client_attrs;
	if (sysfs_create_files(cp->client_kobj, &client_attr_list[client][0])) {
		pr_err("failed to create sysfs client files\n");
		kobject_del(cp->client_kobj);
		return;
	}
}

static void isomgr_create_sysfs(void)
{
	int i;

	if (unlikely(isomgr.kobj)) {
		pr_err("isomgr: create_sysfs failed, isomgr.kobj exists\n");
		return;
	}
	isomgr.kobj = kobject_create_and_add("isomgr", kernel_kobj);
	if (!isomgr.kobj) {
		pr_err("failed to create kobject\n");
		return;
	}
	if (sysfs_create_files(isomgr.kobj, isomgr_attrs)) {
		pr_err("failed to create sysfs files\n");
		kobject_del(isomgr.kobj);
		isomgr.kobj = NULL;
		return;
	}

	for (i = 0; i < isoclients; i++) {
		if (isoclient_info[i].name)
			isomgr_create_client(isoclient_info[i].client,
					     isoclient_info[i].name);
	}
}
#else
static inline void isomgr_create_sysfs(void) {};
#endif /* CONFIG_TEGRA_ISOMGR_SYSFS */

int __init isomgr_init(void)
{
	int i;

	mutex_init(&isomgr.lock);

	if (tegra_get_chip_id() == TEGRA194)
		isomgr.ops = t19x_isomgr_init();
	else
		isomgr.ops = pre_t19x_isomgr_init();

	for (i = 0; ; i++) {
		if (isoclient_info[i].name)
			client_valid[isoclient_info[i].client] = true;
		else
			break;
	}

#ifdef CONFIG_COMMON_CLK
	isomgr.bwmgr_handle = tegra_bwmgr_register(TEGRA_BWMGR_CLIENT_ISOMGR);
	if (IS_ERR_OR_NULL(isomgr.bwmgr_handle)) {
		pr_err("couldn't get handle from bwmgr. disabling isomgr.\n");
#else
	isomgr.emc_clk = clk_get_sys("iso", "emc");
	if (IS_ERR_OR_NULL(isomgr.emc_clk)) {
		pr_err("couldn't find iso emc clock. disabling isomgr.\n");
#endif
		test_mode = 1;
		return 0;
	}

	isomgr.ops->isomgr_plat_init();

	for (i = 0; i < isoclients; i++) {
		if (isoclient_info[i].name) {
			enum tegra_iso_client c = isoclient_info[i].client;

			OBJ_REF_SET(&isomgr_clients[c].kref.refcount, 0);
			init_completion(&isomgr_clients[c].cmpl);
#ifdef CONFIG_COMMON_CLK
			isomgr_clients[c].bwmgr_handle = tegra_bwmgr_register(
					isoclient_info[i].bwmgr_id);

			if (IS_ERR_OR_NULL(isomgr_clients[c].bwmgr_handle)) {
				pr_err("couldn't get %s's bwmgr handle\n",
						isoclient_info[i].name);
				isomgr_clients[c].bwmgr_handle = NULL;
#else
			isomgr_clients[c].emc_clk = clk_get_sys(
					isoclient_info[i].dev_name,
					isoclient_info[i].emc_clk_name);

			if (IS_ERR_OR_NULL(isomgr_clients[c].emc_clk)) {
				pr_err("couldn't find %s %s clock",
					isoclient_info[i].dev_name,
					isoclient_info[i].emc_clk_name);

				isomgr_clients[c].emc_clk = NULL;
#endif
				return 0;
			}
		}
	}

	isomgr_create_sysfs();
	return 0;
}
#ifdef CONFIG_COMMON_CLK
fs_initcall(isomgr_init);
#endif

int tegra_isomgr_enable_test_mode(void)
{
	int i;
	struct isomgr_client *cp = NULL;

	if (!isomgr_lock()) {
		pr_err("isomgr: enable_test_mode lock deadlock\n");
		return -EINVAL;
	}
	test_mode = 1;
	if (!isomgr_unlock()) {
		pr_err("isomgr: enable_test_mode unlock mismatch\n");
		return -EINVAL;
	}
	for (i = 0; i < TEGRA_ISO_CLIENT_COUNT; i++) {
		if (!client_valid[i])
			continue;
		cp = &isomgr_clients[i];
retry:
		__tegra_isomgr_unregister(cp);
		if (OBJ_REF_READ(&cp->kref.refcount))
			goto retry;
	}
	return 0;
}
EXPORT_SYMBOL(tegra_isomgr_enable_test_mode);

tegra_isomgr_handle test_tegra_isomgr_register(enum tegra_iso_client client,
					  u32 dedicated_bw,	/* KB/sec */
					  tegra_isomgr_renegotiate renegotiate,
					  void *priv)
{
	return __tegra_isomgr_register(client, dedicated_bw, renegotiate, priv);
}
EXPORT_SYMBOL(test_tegra_isomgr_register);

void test_tegra_isomgr_unregister(tegra_isomgr_handle handle)
{
	return __tegra_isomgr_unregister(handle);
}
EXPORT_SYMBOL(test_tegra_isomgr_unregister);

/* bw in KB/sec and lt in usec*/
u32 test_tegra_isomgr_reserve(tegra_isomgr_handle handle,
			 u32 bw, u32 lt)
{
	return __tegra_isomgr_reserve(handle, bw, lt);
}
EXPORT_SYMBOL(test_tegra_isomgr_reserve);

u32 test_tegra_isomgr_realize(tegra_isomgr_handle handle)
{
	return __tegra_isomgr_realize(handle);
}
EXPORT_SYMBOL(test_tegra_isomgr_realize);

int test_tegra_isomgr_set_margin(enum tegra_iso_client client,
				u32 bw, bool wait)
{
	return __tegra_isomgr_set_margin(client, bw, wait);
}
EXPORT_SYMBOL(test_tegra_isomgr_set_margin);

int test_tegra_isomgr_get_imp_time(enum tegra_iso_client client, u32 bw)
{
	return __tegra_isomgr_get_imp_time(client, bw);
}
EXPORT_SYMBOL(test_tegra_isomgr_get_imp_time);

u32 test_tegra_isomgr_get_available_iso_bw(void)
{
	return __tegra_isomgr_get_available_iso_bw();
}
EXPORT_SYMBOL(test_tegra_isomgr_get_available_iso_bw);

u32 test_tegra_isomgr_get_total_iso_bw(enum tegra_iso_client client)
{
	return __tegra_isomgr_get_total_iso_bw(client);
}
EXPORT_SYMBOL(test_tegra_isomgr_get_total_iso_bw);
