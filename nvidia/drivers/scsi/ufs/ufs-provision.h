/*
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * Author:
 *      Abhinav Site	<asite@nvidia.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _UFS_PROVISION_H
#define _UFS_PROVISION_H

#include <linux/kernel.h>
#include <linux/module.h>
#include "ufshcd.h"

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
void debugfs_provision_init(struct ufs_hba *hba, struct dentry *device_root);
void debugfs_provision_exit(struct ufs_hba *hba);
#endif

#endif
