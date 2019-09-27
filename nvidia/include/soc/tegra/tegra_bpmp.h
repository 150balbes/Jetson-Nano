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

#ifndef _SOC_TEGRA_TEGRA_BPMP_H
#define _SOC_TEGRA_TEGRA_BPMP_H

#include <linux/kernel.h>

typedef void (*bpmp_mrq_handler)(int mrq, void *data, int ch);

#ifdef CONFIG_NV_TEGRA_BPMP
int tegra_bpmp_running(void);
int tegra_bpmp_send_receive_atomic(int mrq, void *ob_data, int ob_sz,
		void *ib_data, int ib_sz);
int tegra_bpmp_send_receive(int mrq, void *ob_data, int ob_sz,
		void *ib_data, int ib_sz);
int tegra_bpmp_request_mrq(int mrq, bpmp_mrq_handler handler, void *data);
int tegra_bpmp_cancel_mrq(int mrq);
int tegra_bpmp_request_module_mrq(uint32_t module_base,
		bpmp_mrq_handler handler, void *data);
void tegra_bpmp_cancel_module_mrq(uint32_t module_base);
uint32_t tegra_bpmp_mail_readl(int ch, int offset);
int tegra_bpmp_read_data(unsigned int ch, void *data, size_t sz);
void tegra_bpmp_mail_return(int ch, int code, int v);
void tegra_bpmp_mail_return_data(int ch, int code, void *data, int sz);
void *tegra_bpmp_alloc_coherent(size_t size, dma_addr_t *phys,
		gfp_t flags);
void tegra_bpmp_free_coherent(size_t size, void *vaddr,
		dma_addr_t phys);
#ifdef CONFIG_DEBUG_FS
struct dentry *tegra_bpmp_debugfs_add_file(char *name,
	umode_t mode, void *data, const struct file_operations *fops);
#endif
#else
static inline int tegra_bpmp_running(void) { return 0; }
static inline int tegra_bpmp_send_receive_atomic(int mrq, void *ob_data,
		int ob_sz, void *ib_data, int ib_sz) { return -ENODEV; }
static inline int tegra_bpmp_send_receive(int mrq, void *ob_data, int ob_sz,
		void *ib_data, int ib_sz) { return -ENODEV; }
static inline int tegra_bpmp_request_mrq(int mrq, bpmp_mrq_handler handler,
		void *data) { return -ENODEV; }
static inline int tegra_bpmp_cancel_mrq(int mrq) { return -ENODEV; }
static inline int tegra_bpmp_request_module_mrq(uint32_t module_base,
		bpmp_mrq_handler handler, void *data) { return -ENODEV; }
static inline void tegra_bpmp_cancel_module_mrq(uint32_t module_base) {}
static inline uint32_t tegra_bpmp_mail_readl(int ch, int offset) { return 0; }
static inline int tegra_bpmp_read_data(unsigned int ch, void *data, size_t sz)
{ return -ENODEV; }
static inline void tegra_bpmp_mail_return(int ch, int code, int v) {}
static inline void tegra_bpmp_mail_return_data(int ch, int code,
		void *data, int sz) { }
static inline void *tegra_bpmp_alloc_coherent(size_t size, dma_addr_t *phys,
		gfp_t flags) { return NULL; }
static inline void tegra_bpmp_free_coherent(size_t size, void *vaddr,
		dma_addr_t phys) { }
#ifdef CONFIG_DEBUG_FS
struct dentry *tegra_bpmp_debugfs_add_file(char *name,
	umode_t mode, void *data, const struct file_operations *fops)
{ return NULL; }
#endif
#endif

#endif
