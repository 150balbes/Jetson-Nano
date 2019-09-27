/*
 * PVA Debug Information file
 *
 * Copyright (c) 2017-2018, NVIDIA Corporation.  All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/platform_device.h>
#include "dev.h"
#include "pva.h"

static void pva_read_crashdump(struct seq_file *s, struct pva_seg_info *seg_info)
{
	int i = 0;
	u32 *seg_addr = (u32 *) seg_info->addr;

	if (!seg_addr)
		return;

	for (i = 0; i < (seg_info->size >> 4);) {
		seq_printf(s, "0x%x 0x%x 0x%x 0x%x\n",
			seg_addr[i], seg_addr[i+1],
			seg_addr[i+2], seg_addr[i+3]);
		i = i + 4;
	}
}

static int pva_crashdump(struct seq_file *s, void *data)
{
	int err = 0;
	struct pva_crashdump_debugfs_entry *entry =
			(struct pva_crashdump_debugfs_entry *)s->private;
	struct pva *pva = entry->pva;

	err = nvhost_module_busy(pva->pdev);
	if (err) {
		nvhost_dbg_info("err in powering up pva\n");
		goto err_poweron;
	}

	pva_read_crashdump(s, &entry->seg_info);

	nvhost_module_idle(pva->pdev);

err_poweron:
	return err;
}

static int crashdump_open(struct inode *inode, struct file *file)
{
	return single_open(file, pva_crashdump, inode->i_private);
}

static const struct file_operations pva_crashdump_fops = {
	.open = crashdump_open,
	.read = seq_read,
	.release = single_release,
};

static int pva_print_function_table(struct seq_file *s, void *data)
{
	struct pva_func_table fn_table;
	struct pva *pva = s->private;
	struct vpu_func *fn;
	uint32_t entries;
	int ret = 0;
	int i;

	ret = nvhost_module_busy(pva->pdev);
	if (ret) {
		nvhost_dbg_info("error in powering up pva\n");
		goto err_poweron;
	}

	ret = pva_alloc_and_populate_function_table(pva, &fn_table);
	if (ret) {
		nvhost_dbg_info("unable to populate function table\n");
		goto err_vpu_alloc;
	}

	fn = fn_table.addr;
	entries = fn_table.entries;

	seq_puts(s, "NAME  ID\n");
	for (i = 0; i < entries; i++) {
		char *name = (char *)&fn->name;

		seq_printf(s, "%s  %d\n",
			   name, fn->id);
		fn = (struct vpu_func *)(((u8 *)fn) + fn->next);
	}

	pva_dealloc_vpu_function_table(pva, &fn_table);

	nvhost_module_idle(pva->pdev);
	return 0;

err_vpu_alloc:
	nvhost_module_idle(pva->pdev);
err_poweron:
	return ret;
}

static int vpu_func_table_open(struct inode *inode, struct file *file)
{
	return single_open(file, pva_print_function_table, inode->i_private);
}

static const struct file_operations pva_vpu_function_table_fops = {
	.open = vpu_func_table_open,
	.read = seq_read,
	.release = single_release,
};

static inline void print_version(struct seq_file *s,
				 const char *version_str,
				 const u32 version)
{
	const char type = PVA_EXTRACT(version, 31, 24, u8);
	const u32 major = PVA_EXTRACT(version, 23, 16, u32);
	const u32 minor = PVA_EXTRACT(version, 15, 8, u32);
	const u32 subminor = PVA_EXTRACT(version, 7, 0, u32);

	seq_printf(s, "%s: %c.%02u.%02u.%02u\n", version_str,
		   type, major, minor, subminor);
}

static int print_firmware_versions(struct seq_file *s, void *data)
{
	struct pva *pva = s->private;
	struct pva_version_info info;
	int ret = 0;

	ret = nvhost_module_busy(pva->pdev);
	if (ret < 0)
		goto err_poweron;

	ret = pva_get_firmware_version(pva, &info);
	if (ret < 0)
		goto err_get_firmware_version;

	nvhost_module_idle(pva->pdev);

	print_version(s, "pva_r5_version", info.pva_r5_version);
	print_version(s, "pva_compat_version", info.pva_compat_version);
	seq_printf(s, "pva_revision: %x\n", info.pva_revision);
	seq_printf(s, "pva_built_on: %u\n", info.pva_built_on);

	return 0;

err_get_firmware_version:
	nvhost_module_idle(pva->pdev);
err_poweron:
	return ret;
}

static int print_version_open(struct inode *inode, struct file *file)
{
	return single_open(file, print_firmware_versions, inode->i_private);
}

static const struct file_operations print_version_fops = {
	.open = print_version_open,
	.read = seq_read,
	.release = single_release,
};

static int get_log_level(void *data, u64 *val)
{
	struct pva *pva = (struct pva *) data;

	*val = pva->log_level;
	return 0;
}

static int set_log_level(void *data, u64 val)
{
	struct pva *pva = (struct pva *) data;

	pva->log_level = val;
	if (pva->booted)
		return pva_set_log_level(pva, val);
	else
		return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(log_level_fops, get_log_level, set_log_level, "%llu");

void pva_debugfs_init(struct platform_device *pdev)
{
	struct dentry *ret;
	struct nvhost_device_data *pdata = platform_get_drvdata(pdev);
	struct pva *pva = pdata->private_data;
	struct dentry *de = pdata->debugfs;

	if (!de)
		return;

	pva->debugfs_entry_r5.pva = pva;
	pva->debugfs_entry_vpu0.pva = pva;
	pva->debugfs_entry_vpu1.pva = pva;

	ret = debugfs_create_file("r5_crashdump", S_IRUGO, de,
				&pva->debugfs_entry_r5, &pva_crashdump_fops);
	if (!ret)
		nvhost_dbg_info("Failed R5_crashdump file creation");

	ret = debugfs_create_file("vpu0_crashdump", S_IRUGO, de,
				&pva->debugfs_entry_vpu0, &pva_crashdump_fops);
	if (!ret)
		nvhost_dbg_info("Failed VPU0_crashdump file creation");


	ret = debugfs_create_file("vpu1_crashdump", S_IRUGO, de,
				&pva->debugfs_entry_vpu1, &pva_crashdump_fops);
	if (!ret)
		nvhost_dbg_info("Failed VPU1_crashdump file creation");

	ret = debugfs_create_u32("submit_mode", S_IRUGO | S_IWUSR, de,
				 &pva->submit_mode);
	if (!ret)
		nvhost_dbg_info("Failed to create submit mode selection file");

	ret = debugfs_create_file("vpu_function_table", S_IRUGO, de,
				pva, &pva_vpu_function_table_fops);
	if (!ret)
		nvhost_dbg_info("Failed to create vpu function table file");

	ret = debugfs_create_u32("vpu_app_id", S_IRUGO | S_IWUSR, de,
				 &pva->dbg_vpu_app_id);
	if (!ret)
		nvhost_dbg_info("Failed to create vpu_app id debug file");

	ret = debugfs_create_u32("r5_dbg_wait", 0644, de,
				 &pva->r5_dbg_wait);
	if (!ret)
		nvhost_dbg_info("Failed to create r5_dbg_wait file");

	ret = debugfs_create_bool("r5_timeout_enable", 0644, de,
				 &pva->timeout_enabled);
	if (!ret)
		nvhost_dbg_info("Failed to create r5_timeout_enable node");

	ret = debugfs_create_file("firmware_version", S_IRUGO, de,
				pva, &print_version_fops);
	if (!ret)
		nvhost_dbg_info("Failed to create PVA version file");

	ret = debugfs_create_u32("cg_disable", 0644, de,
				 &pva->slcg_disable);
	if (!ret)
		nvhost_dbg_info("Failed to create cg_disable node");

	ret = debugfs_create_bool("vpu_perf_counters_enable", 0644, de,
				  &pva->vpu_perf_counters_enable);
	if (!ret)
		nvhost_dbg_info("Failed to create vpu_perf_counters_enable node");

	ret = debugfs_create_file("log_level", 0644, de,
				  pva, &log_level_fops);
	if (!ret)
		nvhost_dbg_info("Failed to create log_lovel node");
}
