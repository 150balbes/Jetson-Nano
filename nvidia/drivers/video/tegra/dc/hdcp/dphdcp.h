/*
 * dphdcp.h: dp hdcp driver.
 *
 * Copyright (c) 2015-2019, NVIDIA CORPORATION, All rights reserved.
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

#include <uapi/video/nvhdcp.h>
#include <linux/version.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <linux/miscdevice.h>
#endif

#define HDCP_HDCP2_VERSION		0x50
#define HDCP_HDCP2_VERSION_HDCP22_YES	0x04 /* DP 2.2 */
#define HDCP_HDCP2_VERSION_HDCP22_NO	0x00 /* DP 1.x */

#define HDCP_WRITE_MSG			0x60

#define HDCP_RX_STATUS			0x70
#define HDCP_RX_STATUS_MSG_SIZE_MASK	0x03ff
#define HDCP_RX_STATUS_MSG_READY_YES	0x0400
#define HDCP_RX_STATUS_MSG_READY_NO	0x0000
#define HDCP_RX_STATUS_MSG_REAUTH_REQ	0x0800

#define HDCP_READ_MSG			0x80
#define HDCP_MIN_RETRIES		0
#define HDCP_MAX_RETRIES		5
#define TEGRA_DPHDCP_MAX_DEVS		127

/* for dphdcp state */
enum tegra_dphdcp_state {
	STATE_OFF,
	STATE_UNAUTHENTICATED,
	STATE_LINK_VERIFY,
	STATE_RENEGOTIATE,
};

struct tegra_dphdcp {
	struct delayed_work		work;
	struct tegra_dc_dp_data		*dp;
	struct workqueue_struct		*downstream_wq;
	struct mutex			lock;
	struct miscdevice		miscdev;
	char				name[12];
	unsigned			id;
	bool				plugged; /* true if hotplug detected */
	atomic_t			policy;
	enum tegra_dphdcp_state		state; /* un/authenticated, etc */
	struct i2c_board_info		info;
	int				bus;
	u32				binfo;
	u64				a_n;
	u64				c_n;
	u64				a_ksv;
	u64				b_ksv;
	u64				c_ksv;
	u64				d_ksv;
	u8				v_prime[20];
	u64				m_prime;
	u32				num_bksv_list;
	u64				bksv_list[TEGRA_DPHDCP_MAX_DEVS];
	int				fail_count;
	char				hdcp22;
	u8				max_retries;
	u8				hpd;
	u8				repeater;
	void				*ta_ctx;
};

#ifdef CONFIG_DPHDCP
void tegra_dphdcp_set_plug(struct tegra_dphdcp *dphdcp, bool hpd);
int tegra_dphdcp_set_policy(struct tegra_dphdcp *nvhdcp, int pol);
struct tegra_dphdcp *tegra_dphdcp_create(struct tegra_dc_dp_data *dp,
						int id, int bus);
void tegra_dphdcp_debugfs_init(struct tegra_dphdcp *dphdcp);
void tegra_dphdcp_destroy(struct tegra_dphdcp *dphdcp);
#else
static inline void tegra_dphdcp_set_plug(struct tegra_dphdcp *dphdcp, bool hpd)
{
}

static inline int tegra_dphdcp_set_policy(struct tegra_dphdcp *dphdcp,
							int pol)
{
	return 0;
}

static inline struct tegra_dphdcp *tegra_dphdcp_create(
		struct tegra_dc_dp_data *dp, int id, int bus)
{
	return NULL;
}
static inline void tegra_dphdcp_destroy(struct tegra_dphdcp *dphdcp)
{
}
#endif
