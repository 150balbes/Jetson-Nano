/*
 * drivers/virt/tegra/hvc_sysfs.c
 *
 * Copyright (c) 2016-2019, NVIDIA CORPORATION. All rights reserved.
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

#define pr_fmt(fmt) "hvc_sysfs: " fmt

#include <linux/module.h>
#include <linux/mm.h>
#include <linux/io.h>
#include <soc/tegra/virt/syscalls.h>
#include <soc/tegra/chip-id.h>

#define ATTRS_MAX 10U

static struct kobject *kobj;

/*
 * This file implements a hypervisor control driver that can be accessed
 * from user-space via the sysfs interface. Currently, the only supported
 * use case is retrieval of the HV trace log when it is available.
 */

struct uart_relay_info_t {
	uint32_t num_channels;
	uint32_t max_msg_size;
};

struct nvlog_reader_info_t {
	uint32_t num_vms;
};

static struct uart_relay_info_t uart_relay_info;
static struct nvlog_reader_info_t nvlog_reader_info;

struct hyp_shared_memory_info {
	const char *node_name;
	struct bin_attribute attr;
	umode_t mode;
	uint64_t ipa;
	unsigned long size;
	ssize_t (*read)(struct file *, struct kobject *, struct bin_attribute *,
			char *, loff_t, size_t);
	ssize_t (*write)(struct file *, struct kobject *,
			struct bin_attribute *, char *, loff_t, size_t);
	bool available;
};

static struct hyp_shared_memory_info hyp_shared_memory_attrs[ATTRS_MAX];

static ssize_t uart_relay_read(struct file *filp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf, loff_t off,
			size_t count)
{
	ssize_t num_bytes;
	size_t last_byte_index = min_t(size_t,
		sizeof(struct uart_relay_info_t), count + off);

	if (off >= last_byte_index)
		return -ESPIPE;

	num_bytes = last_byte_index - off;
	memcpy(buf, attr->private + off, num_bytes);
	return num_bytes;
}

ssize_t trace_mask_read(struct file *filp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf, loff_t off,
			size_t count)
{
	/* Mask must be read all 8 bytes at once. Consecutive 8 byte reads for
	 * the same open fd yield new mask reads.
	 */
	if (count < sizeof(uint64_t))
		return -ESPIPE;

	if ((count % sizeof(uint64_t) != 0) || (off % sizeof(uint64_t) != 0))
		return -ESPIPE;

	hyp_trace_get_mask((uint64_t *)buf);

	return sizeof(uint64_t);
}

ssize_t trace_mask_write(struct file *filp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf, loff_t off,
			size_t count)
{
	// Mask must be supplied all 8 bytes at once.
	if (count != sizeof(uint64_t))
		return -ESPIPE;

	hyp_trace_set_mask(*(uint64_t *)buf);

	return count;
}

static ssize_t nvlog_reader_read(struct file *filp, struct kobject *kobj,
			struct bin_attribute *attr, char *buf,
			loff_t off, size_t count)
{
	ssize_t num_bytes;
	size_t last_byte_index = min_t(size_t,
		sizeof(struct nvlog_reader_info_t), count + off);

	if (off >= last_byte_index)
		return -ESPIPE;

	num_bytes = last_byte_index - off;
	memcpy(buf, attr->private + off, num_bytes);
	return num_bytes;
}

/* Map the HV trace buffer to the calling user process */
static int hvc_sysfs_mmap(struct file *fp, struct kobject *ko,
	struct bin_attribute *attr, struct vm_area_struct *vma)
{
	struct hyp_shared_memory_info *hyp_shm_info =
		container_of(attr, struct hyp_shared_memory_info, attr);

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
	sysfs_bin_attr_init(&hyp_shm_info->attr);

	hyp_shm_info->attr.attr.name = hyp_shm_info->node_name;
	hyp_shm_info->attr.attr.mode = hyp_shm_info->mode;
	hyp_shm_info->attr.size = (size_t)hyp_shm_info->size;

	if (hyp_shm_info->read != NULL)
		hyp_shm_info->attr.read = hyp_shm_info->read;

	if (hyp_shm_info->write != NULL)
		hyp_shm_info->attr.write = hyp_shm_info->write;

	if (hyp_shm_info->ipa > 0 && hyp_shm_info->size > 0)
		hyp_shm_info->attr.mmap = hvc_sysfs_mmap;

	return sysfs_create_bin_file(kobj, &hyp_shm_info->attr);
}

static int __init hvc_sysfs_register_log(struct kobject *kobj, int index)
{
	int ret;
	uint64_t ipa;
	struct hyp_info_page *info;

	ret = hyp_read_hyp_info(&ipa);
	if (ret)
		return ret;

	info = (struct hyp_info_page *)ioremap(ipa, sizeof(*info));
	if (info == NULL)
		return -ENOMEM;

	if (index < ATTRS_MAX) {
		hyp_shared_memory_attrs[index].mode = 0444;
		hyp_shared_memory_attrs[index].ipa = info->log_ipa;
		hyp_shared_memory_attrs[index].size = (size_t)info->log_size;
		hyp_shared_memory_attrs[index].node_name = "log";
		hyp_shared_memory_attrs[index].available = true;
		index = index + 1;
	}

	if (index < ATTRS_MAX) {
		hyp_shared_memory_attrs[index].mode = 0444;
		hyp_shared_memory_attrs[index].ipa = info->pct_ipa;
		hyp_shared_memory_attrs[index].size = (size_t)info->pct_size;
		hyp_shared_memory_attrs[index].node_name = "pct";
		hyp_shared_memory_attrs[index].available = true;
		index = index + 1;
	}

	iounmap(info);
	return index;
}

static int __init hvc_sysfs_register_uartrelay(struct kobject *kobj, int index)
{
	uint64_t ipa;
	uint64_t size, nc, maxsize;

	if (hyp_read_uart_relay_info(&ipa, &size, &nc, &maxsize)) {
		pr_info("uart_relay: feature deactivated\n");
		return -ENXIO;
	}

	uart_relay_info.num_channels = (uint32_t)nc;
	uart_relay_info.max_msg_size = (uint32_t)maxsize;

	if (index < ATTRS_MAX) {
		hyp_shared_memory_attrs[index].mode = 0444;
		hyp_shared_memory_attrs[index].ipa = ipa;
		hyp_shared_memory_attrs[index].size = size;
		hyp_shared_memory_attrs[index].node_name = "uart_relay";
		hyp_shared_memory_attrs[index].attr.private = &uart_relay_info;
		hyp_shared_memory_attrs[index].read = uart_relay_read;
		hyp_shared_memory_attrs[index].available = true;
		index = index + 1;
	}

	return index;
}

static int __init hvc_sysfs_register_tracemask(struct kobject *kobj, int index)
{
	uint64_t dummy;

	if (hyp_trace_get_mask(&dummy)) {
		pr_info("trace_mask: Not allowed to get/set trace mask\n");
		return -ENXIO;
	}

	if (index < ATTRS_MAX) {
		hyp_shared_memory_attrs[index].mode = 0600;
		hyp_shared_memory_attrs[index].size = sizeof(uint64_t);
		hyp_shared_memory_attrs[index].node_name = "trace_mask";
		hyp_shared_memory_attrs[index].read = trace_mask_read;
		hyp_shared_memory_attrs[index].write = trace_mask_write;
		hyp_shared_memory_attrs[index].available = true;
		index = index + 1;
	}

	return index;
}

static int __init hvc_sysfs_register_nvlog_reader(struct kobject *kobj,
								int index)
{
	uint64_t ipa;
	uint64_t size, vms;

	if (hyp_read_nvlog_reader_info(&ipa, &size, &vms)) {
		pr_info("nvlog: Reader feature not enabled for this VM\n");
		return -ENXIO;
	}

	nvlog_reader_info.num_vms = (uint32_t)vms;

	if (index < ATTRS_MAX) {
		hyp_shared_memory_attrs[index].mode = 0444;
		hyp_shared_memory_attrs[index].ipa = ipa;
		hyp_shared_memory_attrs[index].size = size;
		hyp_shared_memory_attrs[index].node_name = "nvlog_reader";
		hyp_shared_memory_attrs[index].attr.private =
							&nvlog_reader_info;
		hyp_shared_memory_attrs[index].read = nvlog_reader_read;
		hyp_shared_memory_attrs[index].available = true;
		index = index + 1;
	}

	return index;
}

static int __init hvc_sysfs_register_nvlog_writer(struct kobject *kobj,
								int index)
{
	uint64_t ipa;
	uint64_t size;

	if (hyp_read_nvlog_writer_info(&ipa, &size)) {
		pr_info("nvlog: Writer feature not enabled for this VM\n");
		return -ENXIO;
	}
	if (index < ATTRS_MAX) {
		hyp_shared_memory_attrs[index].mode = 0666;
		hyp_shared_memory_attrs[index].ipa = ipa;
		hyp_shared_memory_attrs[index].size = size;
		hyp_shared_memory_attrs[index].node_name = "nvlog_writer";
		hyp_shared_memory_attrs[index].attr.private = NULL;
		hyp_shared_memory_attrs[index].read = NULL;
		hyp_shared_memory_attrs[index].available = true;
		index = index + 1;
	}

	return index;
}

/* Set up all relevant hypervisor control nodes */
static int __init hvc_sysfs_register(void)
{
	int ret;
	int index;

	if (!is_tegra_hypervisor_mode()) {
		pr_info("hypervisor is not present\n");
		return -EPERM;
	}

	kobj = kobject_create_and_add("hvc", NULL);
	if (kobj == NULL) {
		pr_err("failed to add kobject\n");
		return -ENOMEM;
	}

	index = 0;
	ret = hvc_sysfs_register_log(kobj, index);
	if (ret >= 0)
		index = ret;
	ret = hvc_sysfs_register_uartrelay(kobj, index);
	if (ret >= 0)
		index = ret;
	ret = hvc_sysfs_register_tracemask(kobj, index);
	if (ret >= 0)
		index = ret;
	ret = hvc_sysfs_register_nvlog_reader(kobj, index);
	if (ret >= 0)
		index = ret;
	ret = hvc_sysfs_register_nvlog_writer(kobj, index);
	if (ret >= 0)
		index = ret;

	for (index = 0; index < ARRAY_SIZE(hyp_shared_memory_attrs); index++) {
		if (!hyp_shared_memory_attrs[index].node_name)
			continue;

		if (hyp_shared_memory_attrs[index].available)
			ret = hvc_create_sysfs(kobj,
					&hyp_shared_memory_attrs[index]);
		else
			ret = -ENXIO;

		if (ret) {
			hyp_shared_memory_attrs[index].available = false;
			pr_info("%s is not available (%d)\n",
				hyp_shared_memory_attrs[index].node_name, ret);
			continue;
		}
		pr_info("%s is available\n",
			hyp_shared_memory_attrs[index].node_name);
	}
	return 0;
}

static void hvc_sysfs_unregister(void)
{
	int index;

	for (index = 0; index < ARRAY_SIZE(hyp_shared_memory_attrs); index++) {
		if (!hyp_shared_memory_attrs[index].node_name)
			continue;
		if (!hyp_shared_memory_attrs[index].available)
			continue;
		sysfs_remove_bin_file(kobj,
				&hyp_shared_memory_attrs[index].attr);
	}
	kobject_put(kobj);
}

module_init(hvc_sysfs_register);
module_exit(hvc_sysfs_unregister);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("NVIDIA Corporation");
