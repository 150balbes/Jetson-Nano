/*
 * drivers/virt/tegra/hvc_sysfs.c
 *
 * Copyright (c) 2016-2017, NVIDIA CORPORATION. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/errno.h>
#include <soc/tegra/chip-id.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/mm.h>
#include <linux/bug.h>
#include <linux/io.h>
#include <soc/tegra/virt/syscalls.h>

#define TEGRA_HV_ERR(...) pr_err("hvc_sysfs: " __VA_ARGS__)
#define TEGRA_HV_INFO(...) pr_info("hvc_sysfs: " __VA_ARGS__)


/*
 * This file implements a hypervisor control driver that can be accessed
 * from user-space via the sysfs interface. Currently, the only supported
 * use case is retrieval of the HV trace log when it is available.
 */

struct hyp_shared_memory_info {
	const char *node_name;
	struct bin_attribute attr;
	uint64_t ipa;
	unsigned long size;
};

enum HYP_SHM_ID {
	HYP_SHM_ID_LOG,
	HYP_SHM_ID_PCT,

	HYP_SHM_ID_NUM
};

static struct hyp_shared_memory_info hyp_shared_memory_attrs[HYP_SHM_ID_NUM];



/* Map the HV trace buffer to the calling user process */
static int hvc_sysfs_mmap(struct file *fp, struct kobject *ko,
	struct bin_attribute *attr, struct vm_area_struct *vma)
{
	struct hyp_shared_memory_info *hyp_shm_info =
		container_of(attr, struct hyp_shared_memory_info, attr);

	if ((hyp_shm_info->ipa == 0) || (hyp_shm_info->size == 0))
		return -EINVAL;


	if ((vma->vm_end - vma->vm_start) != attr->size)
		return -EINVAL;

	return remap_pfn_range(
		vma,
		vma->vm_start,
		hyp_shm_info->ipa >> PAGE_SHIFT,
		hyp_shm_info->size,
		vma->vm_page_prot);
}

/* Discover availability and placement of the HV trace buffer */
static int hvc_create_sysfs(
	struct kobject *kobj,
	struct hyp_shared_memory_info *hyp_shm_info)
{
	sysfs_bin_attr_init((struct bin_attribute *)&hyp_shm_info->attr);

	hyp_shm_info->attr.attr.name = hyp_shm_info->node_name;
	hyp_shm_info->attr.attr.mode = S_IRUSR | S_IRGRP | S_IROTH;
	hyp_shm_info->attr.mmap = hvc_sysfs_mmap;
	hyp_shm_info->attr.size = (size_t)hyp_shm_info->size;

	if ((hyp_shm_info->ipa == 0) || (hyp_shm_info->size == 0))
		return -EINVAL;

	return sysfs_create_bin_file(kobj, &hyp_shm_info->attr);
}

/* Set up all relevant hypervisor control nodes */
static int __init hvc_sysfs_register(void)
{
	struct kobject *kobj;
	int ret;
	uint64_t ipa;
	struct hyp_info_page *info;

	if (is_tegra_hypervisor_mode() == false) {
		TEGRA_HV_INFO("hypervisor is not present\n");
		return -EPERM;
	}

	kobj = kobject_create_and_add("hvc", NULL);
	if (kobj == NULL) {
		TEGRA_HV_INFO("failed to add kobject\n");
		return -ENOMEM;
	}

	if (hyp_read_hyp_info(&ipa) != 0)
		return -EINVAL;

	info = (struct hyp_info_page *)ioremap(ipa, sizeof(*info));
	if (info == NULL)
		return -EFAULT;

	hyp_shared_memory_attrs[HYP_SHM_ID_LOG].ipa = info->log_ipa;
	hyp_shared_memory_attrs[HYP_SHM_ID_LOG].size = (size_t)info->log_size;
	hyp_shared_memory_attrs[HYP_SHM_ID_LOG].node_name = "log";

	ret = hvc_create_sysfs(kobj, &hyp_shared_memory_attrs[HYP_SHM_ID_LOG]);
	if (ret == 0)
		TEGRA_HV_INFO("log is available\n");
	else
		TEGRA_HV_INFO("log is unavailable\n");

	hyp_shared_memory_attrs[HYP_SHM_ID_PCT].ipa = info->pct_ipa;
	hyp_shared_memory_attrs[HYP_SHM_ID_PCT].size = (size_t)info->pct_size;
	hyp_shared_memory_attrs[HYP_SHM_ID_PCT].node_name = "pct";

	ret = hvc_create_sysfs(kobj, &hyp_shared_memory_attrs[HYP_SHM_ID_PCT]);
	if (ret == 0)
		TEGRA_HV_INFO("pct is available\n");
	else
		TEGRA_HV_INFO("pct is unavailable\n");

	iounmap(info);

	return 0;
}

late_initcall(hvc_sysfs_register);
