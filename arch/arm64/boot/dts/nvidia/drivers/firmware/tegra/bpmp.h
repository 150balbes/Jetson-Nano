/*
 * Copyright (c) 2013-2018, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _DRIVERS_BPMP_H
#define _DRIVERS_BPMP_H

#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <soc/tegra/bpmp_abi.h>
#include <soc/tegra/tegra_bpmp.h>

#define NR_MAX_CHANNELS		14

#define DO_ACK			(1 << 0)
#define RING_DOORBELL		(1 << 1)

struct channel_cfg {
	unsigned int channel_mask;
	unsigned int per_cpu_ch_0;
	unsigned int per_cpu_ch_cnt;
	unsigned int thread_ch_0;
	unsigned int thread_ch_cnt;
	unsigned int ib_ch_0;
	unsigned int ib_ch_cnt;
};

struct fops_entry {
	char *name;
	const struct file_operations *fops;
	mode_t mode;
};

struct mb_data {
	int32_t code;
	int32_t flags;
	u8 data[MSG_DATA_MIN_SZ];
} __packed;

struct channel_data {
	struct mb_data *ib;
	struct mb_data *ob;
};

struct mail_ops {
	int (*init_prepare)(void);
	int (*init_irq)(unsigned int cnt);
	int (*connect)(const struct channel_cfg *cfg,
			const struct mail_ops *ops,
			struct device_node *of_node);
	void (*resume)(void);
	void (*suspend)(void);

	struct ivc *(*ivc_obj)(int ch);

	bool (*master_free)(const struct mail_ops *ops, int ch);
	void (*free_master)(const struct mail_ops *ops, int ch);
	bool (*master_acked)(const struct mail_ops *ops, int ch);
	void (*signal_slave)(const struct mail_ops *ops, int ch);
	bool (*slave_signalled)(const struct mail_ops *ops, int ch);
	void (*ring_doorbell)(int ch);
	void (*return_data)(const struct mail_ops *ops, int ch,
			int code, void *data, int sz);
};

extern const struct mail_ops t210_mail_ops;
extern const struct mail_ops t186_native_mail_ops;
extern const struct mail_ops t186_hv_mail_ops;

extern struct channel_data channel_area[NR_MAX_CHANNELS];
extern char firmware_tag[sizeof(struct mrq_query_fw_tag_response)];

struct dentry *bpmp_init_debug(struct platform_device *pdev);
int bpmp_init_cpuidle_debug(struct dentry *root);
int bpmp_mail_init(const struct channel_cfg *cfg,
		const struct mail_ops *ops, struct device_node *of_node);
int __bpmp_do_ping(void);
int bpmp_create_attrs(const struct fops_entry *fent, struct dentry *parent,
		void *data);
int bpmp_mailman_init(void);
void bpmp_handle_mail(int mrq, int ch);
void tegra_bpmp_resume(void);
int tegra_bpmp_suspend(void);
void bpmp_handle_irq(unsigned int chidx);

#endif
