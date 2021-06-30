/*
 * include/mach/isomgr.h
 *
 * Copyright (c) 2012-2018, NVIDIA CORPORATION.  All rights reserved.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef _INCLUDE_MACH_ISOMGR_H
#define _INCLUDE_MACH_ISOMGR_H

#include <linux/platform/tegra/emc_bwmgr.h>
#include <linux/platform/tegra/iso_client.h>

#define ISOMGR_MAGIC  0x150A1C

/* handle to identify registered client */
#define tegra_isomgr_handle void *

/* callback to client to renegotiate ISO BW allocation */
typedef void (*tegra_isomgr_renegotiate)(void *priv,
					 u32 avail_bw); /* KB/sec */

struct isoclient_info {
	enum tegra_iso_client client;
	char *name;
	char *dev_name;
	char *emc_clk_name;
	enum tegra_bwmgr_client_id bwmgr_id;
};

struct isomgr_client {
	u32 magic;              /* magic to identify handle */
	struct kref kref;       /* ref counting */
	s32 dedi_bw;            /* BW dedicated to this client  (KB/sec) */
	s32 rsvd_bw;            /* BW reserved for this client  (KB/sec) */
	s32 real_bw;            /* BW realized for this client  (KB/sec) */
	s32 lti;                /* Client spec'd Latency Tolerance (usec) */
	s32 lto;                /* MC calculated Latency Tolerance (usec) */
	s32 rsvd_mf;            /* reserved minimum freq in support of LT */
	s32 real_mf;            /* realized minimum freq in support of LT */
	s32 real_mf_rq;         /* real_mf requested */
	tegra_isomgr_renegotiate renegotiate;   /* ask client to renegotiate */
	bool realize;           /* bw realization in progress */
	s32 sleep_bw;           /* sleeping for realize */
	s32 margin_bw;          /* BW set aside for this client (KB/sec) */
	u8 limit_bw_percentage; /* Insufficient HW buffers cause BW to be
				 * limited to this percentage of DRAM BW
				 */
	void *priv;             /* client driver's private data */
	struct completion cmpl; /* so we can sleep waiting for delta BW */

#ifdef CONFIG_COMMON_CLK
	struct tegra_bwmgr_client *bwmgr_handle;
#else
	struct clk *emc_clk;    /* client emc clk for bw */
#endif

#ifdef CONFIG_TEGRA_ISOMGR_SYSFS
	struct kobject *client_kobj;
	struct isomgr_client_attrs {
		struct kobj_attribute dedi_bw;
		struct kobj_attribute rsvd_bw;
		struct kobj_attribute real_bw;
		struct kobj_attribute lti;
		struct kobj_attribute lto;
		struct kobj_attribute rsvd_mf;
		struct kobj_attribute real_mf;
		struct kobj_attribute sleep_bw;
		struct kobj_attribute margin_bw;
	} client_attrs;
#endif /* CONFIG_TEGRA_ISOMGR_SYSFS */
};

struct isomgr {
	struct mutex lock;              /* to lock ALL isomgr state */
	struct task_struct *task;       /* check reentrant/mismatched locks */

#ifdef CONFIG_COMMON_CLK
	struct tegra_bwmgr_client *bwmgr_handle;
#else
	struct clk *emc_clk;            /* isomgr emc clock for floor freq */
#endif

	s32 lt_mf;                      /* min freq to support worst LT */
	s32 lt_mf_rq;                   /* requested lt_mf */
	s32 avail_bw;                   /* globally available MC BW */
	s32 dedi_bw;                    /* total BW 'dedicated' to clients */
	s32 sleep_bw;                   /* pending bw requirement */
	u32 max_iso_bw;                 /* max ISO BW MC can accommodate */
	struct kobject *kobj;           /* for sysfs linkage */
	struct isomgr_ops *ops;         /* ops structure for isomgr*/
};

extern struct isoclient_info *isoclient_info;
/*platform specific flag for requesting max emc floor req for camera client*/
extern u8 isomgr_camera_max_floor_req;
extern int isoclients;
extern bool client_valid[TEGRA_ISO_CLIENT_COUNT];
extern struct isomgr_client isomgr_clients[TEGRA_ISO_CLIENT_COUNT];
extern struct isomgr isomgr;
extern char *cname[];

struct isomgr_ops {
	void (*isomgr_plat_init)(void);
	bool (*isomgr_plat_register)(u32 dedi_bw,
			enum tegra_iso_client client);
	void (*isomgr_plat_unregister)(struct isomgr_client *cp);
	bool (*isomgr_plat_reserve)(struct isomgr_client *cp,
			u32 bw, enum tegra_iso_client client);
	bool (*isomgr_plat_realize)(struct isomgr_client *cp);
	u32 (*isomgr_max_iso_bw)(enum tegra_iso_client client);
};

struct isomgr_ops *pre_t19x_isomgr_init(void);
struct isomgr_ops *t19x_isomgr_init(void);

#if defined(CONFIG_TEGRA_ISOMGR)
/* Register an ISO BW client */
tegra_isomgr_handle tegra_isomgr_register(enum tegra_iso_client client,
					  u32 dedicated_bw,	/* KB/sec */
					  tegra_isomgr_renegotiate renegotiate,
					  void *priv);

/* Unregister an ISO BW client */
void tegra_isomgr_unregister(tegra_isomgr_handle handle);

/* Reserve ISO BW on behalf of client - don't apply, rval is dvfs thresh usec */
u32 tegra_isomgr_reserve(tegra_isomgr_handle handle,
			 u32 bw,	/* KB/sec */
			 u32 lt);	/* usec */

/* Realize client reservation - apply settings, rval is dvfs thresh usec */
u32 tegra_isomgr_realize(tegra_isomgr_handle handle);

/* This sets bw aside for the client specified. */
int tegra_isomgr_set_margin(enum tegra_iso_client client, u32 bw, bool wait);

int tegra_isomgr_get_imp_time(enum tegra_iso_client, u32 bw);

/* returns available in iso bw in KB/sec */
u32 tegra_isomgr_get_available_iso_bw(void);

/* returns total iso bw in KB/sec */
u32 tegra_isomgr_get_total_iso_bw(enum tegra_iso_client client);

/* Initialize isomgr.
 * This api would be called by .init_machine during boot.
 * isomgr clients, don't call this api.
 */
int __init isomgr_init(void);
#else
static inline tegra_isomgr_handle tegra_isomgr_register(
					  enum tegra_iso_client client,
					  u32 dedicated_bw,
					  tegra_isomgr_renegotiate renegotiate,
					  void *priv)
{
	/* return a dummy handle to allow client function
	 * as if isomgr were enabled.
	 */
	return (tegra_isomgr_handle)1;
}

static inline void tegra_isomgr_unregister(tegra_isomgr_handle handle) {}

static inline u32 tegra_isomgr_reserve(tegra_isomgr_handle handle,
			 u32 bw, u32 lt)
{
	return 1;
}

static inline u32 tegra_isomgr_realize(tegra_isomgr_handle handle)
{
	return 1;
}

static inline int tegra_isomgr_set_margin(enum tegra_iso_client client, u32 bw)
{
	return 0;
}

static inline int tegra_isomgr_get_imp_time(enum tegra_iso_client client,
	u32 bw)
{
	return 0;
}

static inline u32 tegra_isomgr_get_available_iso_bw(void)
{
	return UINT_MAX;
}

static inline u32 tegra_isomgr_get_total_iso_bw(enum tegra_iso_client client)
{
	return UINT_MAX;
}

static inline int isomgr_init(void)
{
	return 0;
}
#endif
#endif /* _INCLUDE_MACH_ISOMGR_H */
