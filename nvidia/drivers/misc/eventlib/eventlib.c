/*
 * eventlib.c
 *
 * Copyright (c) 2017-2018, NVIDIA CORPORATION.  All rights reserved.
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
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/crc32.h>

#include <linux/keventlib.h>

#include "eventlib.h"

#define KEVENTLIB_VERSION		"0.2"

#define EVENTLIB_SYSFS_DIR_NAME		"eventlib"
#define EVENTLIB_SYSFS_TEST_FILE_NAME	"test"
#define EVENTLIB_SYSFS_EVENTS_FILE_NAME	"events"
#define EVENTLIB_SYSFS_SCHEMA_FILE_NAME	"schema"

#define EVENTLIB_TEST_SHM_SIZE		(PAGE_SIZE)

#define EVENTLIB_MAX_PROVIDERS		256
#define EVENTLIB_TEST_DATA_SIZE		0x10

struct eventlib_provider_info {
	struct kobject *kobj;

	struct bin_attribute attr;
	struct bin_attribute attr_schema;

	void *data;
	size_t data_size;

	struct eventlib_ctx el_ctx;

	void *w2r;
	size_t w2r_size;

	int id;
	struct list_head list;

	char *schema;
	size_t schema_size;
};

static struct eventlib_module {
	struct kobject *kobj_root;

	struct list_head providers;
	atomic_t nr_providers;

	spinlock_t lock;

	int test_id;
} ctx;

struct eventlib_work_data {
	struct work_struct work;
	struct eventlib_provider_info *provider;
};

#define EVENTLIB_TEST_SAMPLE_MAGIC	0x11223344
struct eventlib_test_sample {
	uint32_t magic;
	uint32_t size;
	uint32_t crc;
} __attribute__((__packed__));

static int is_initialized;

static int keventlib_init(struct eventlib_provider_info *info)
{
	int ret;
	struct eventlib_ctx *el_ctx = &info->el_ctx;

	info->w2r = info->data;
	info->w2r_size = info->data_size;

	pr_debug("w2r: %p, size: %#zx\n", info->w2r, info->w2r_size);

	memset(el_ctx, 0, sizeof(*el_ctx));

	el_ctx->direction = EVENTLIB_DIRECTION_WRITER;
	el_ctx->w2r_shm = info->w2r;
	el_ctx->w2r_shm_size = (uint32_t)info->w2r_size;
	el_ctx->r2w_shm = NULL;
	el_ctx->r2w_shm_size = 0;
	el_ctx->flags = 0;

	ret = eventlib_init(el_ctx);
	if (ret)
		return ret;

	return 0;
}

static int
sysfs_mmap(struct file *filp, struct kobject *kobj,
	   struct bin_attribute *attr, struct vm_area_struct *vma)
{
	unsigned long vm_size, pfn;

	struct eventlib_provider_info *info =
		container_of(attr, struct eventlib_provider_info, attr);

	vm_size = vma->vm_end - vma->vm_start;
	vma->vm_private_data = filp->private_data;

	pr_debug("%s: vma: %#lx - %#lx (%#lx)\n",
		 __func__, vma->vm_start, vma->vm_end, vm_size);

	if (vm_size != attr->size)
		return -EINVAL;

	if (!info->data)
		return -ENOMEM;

	pfn = virt_to_phys(info->data) >> PAGE_SHIFT;

	if (remap_pfn_range(vma, vma->vm_start, pfn,
			    vma->vm_end - vma->vm_start,
			    vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static ssize_t
sysfs_schema_read(struct file *filp, struct kobject *kobj,
		  struct bin_attribute *attr,
		  char *buf, loff_t off, size_t len)
{
	struct eventlib_provider_info *info =
		container_of(attr, struct eventlib_provider_info, attr_schema);

	if (info->schema == NULL)
		return -ENOENT;

	if (len > info->schema_size - off)
		len = info->schema_size - off;

	memcpy(buf, info->schema + off, len);

	return len;
}

static int
create_sysfs_entry(struct eventlib_provider_info *info,
		   const char *name)
{
	int ret;
	struct bin_attribute *attr = &info->attr;

	info->kobj = kobject_create_and_add(name, ctx.kobj_root);
	if (info->kobj == NULL) {
		pr_err("Unable to create sysfs directory: %s\n", name);
		return -ENOMEM;
	}

	sysfs_bin_attr_init(attr);

	attr->attr.name = EVENTLIB_SYSFS_EVENTS_FILE_NAME;
	attr->attr.mode = 0444;
	attr->mmap = sysfs_mmap;
	attr->read = NULL;
	attr->size = info->data_size;

	ret = sysfs_create_bin_file(info->kobj, attr);
	if (ret) {
		pr_err("Unable to create sysfs file: %s\n",
		       attr->attr.name);
		kobject_put(info->kobj);
		return ret;
	}

	if (info->schema) {
		struct bin_attribute *attr_schema = &info->attr_schema;

		sysfs_bin_attr_init(attr_schema);

		attr_schema->attr.name = EVENTLIB_SYSFS_SCHEMA_FILE_NAME;
		attr_schema->attr.mode = 0444;
		attr_schema->mmap = NULL;
		attr_schema->read = sysfs_schema_read;
		attr_schema->write = NULL;
		attr_schema->size = info->schema_size;

		ret = sysfs_create_bin_file(info->kobj, attr_schema);
		if (ret) {
			pr_err("Unable to create sysfs file: %s\n",
			       attr_schema->attr.name);
			kobject_put(info->kobj);
			return ret;
		}
	}

	return 0;
}

static void remove_sysfs_entry(struct eventlib_provider_info *info)
{
	sysfs_remove_bin_file(info->kobj, &info->attr);
	if (info->schema)
		sysfs_remove_bin_file(info->kobj, &info->attr_schema);

	kobject_put(info->kobj);

}

static int is_id_free(int id)
{
	struct eventlib_provider_info *info;

	list_for_each_entry(info, &ctx.providers, list) {
		if (id == info->id)
			return 0;
	}

	return 1;
}

static int get_free_id(void)
{
	int id;

	for (id = 0; id < EVENTLIB_MAX_PROVIDERS; id++) {
		if (is_id_free(id))
			return id;
	}

	return -EMFILE;
}

static int
provider_init(struct eventlib_provider_info *info,
	      size_t size, const char *name,
	      const char *schema, size_t schema_size)
{
	int ret = 0, id;

	info->data = NULL;
	info->data_size = 0;

	info->w2r = NULL;
	info->w2r_size = 0;

	if (size == 0 || !is_power_of_2(size))
		return -EINVAL;

	info->data = (void *)__get_free_pages(GFP_KERNEL, get_order(size));
	if (!info->data)
		return -ENOMEM;

	memset(info->data, 0, size);
	info->data_size = size;

	if (schema && schema_size > 0) {
		info->schema_size = schema_size;

		info->schema = kmalloc(info->schema_size, GFP_KERNEL);
		if (!info->schema)
			return -ENOMEM;

		memcpy(info->schema, schema, schema_size);
	} else {
		info->schema = NULL;
		info->schema_size = 0;
	}

	ret = create_sysfs_entry(info, name);
	if (ret < 0)
		goto err_free;

	ret = keventlib_init(info);
	if (ret < 0)
		goto err_sysfs;

	INIT_LIST_HEAD(&info->list);

	spin_lock(&ctx.lock);

	id = get_free_id();
	if (id < 0) {
		pr_err("Too many providers: > %d\n", EVENTLIB_MAX_PROVIDERS);
		ret = -EMFILE;
		goto err_get_id;
	}

	info->id = id;

	list_add_tail(&info->list, &ctx.providers);
	atomic_inc(&ctx.nr_providers);

	spin_unlock(&ctx.lock);

	return 0;

err_get_id:
	spin_unlock(&ctx.lock);
	eventlib_close(&info->el_ctx);

err_sysfs:
	remove_sysfs_entry(info);

err_free:
	if (info->schema) {
		kfree(info->schema);
		info->schema = NULL;
	}

	free_pages((unsigned long)info->data, get_order(size));

	return ret;
}

static struct eventlib_provider_info *
find_provider_info(int id)
{
	struct eventlib_provider_info *info;

	list_for_each_entry(info, &ctx.providers, list) {
		if (id == info->id)
			return info;
	}

	return NULL;
}

static void
__free_provider(struct work_struct *work)
{
	struct eventlib_work_data *wd =
		container_of(work, struct eventlib_work_data, work);

	struct eventlib_provider_info *info = wd->provider;

	remove_sysfs_entry(info);

	if (info->schema)
		kfree(info->schema);

	kfree(info);
	kfree(wd);

	if (atomic_dec_and_test(&ctx.nr_providers))
		kobject_put(ctx.kobj_root);
}

static void free_provider(struct eventlib_provider_info *info)
{
	struct eventlib_work_data *wd;

	eventlib_close(&info->el_ctx);

	free_pages((unsigned long)info->data,
		   get_order(info->data_size));

	list_del(&info->list);

	wd = kmalloc(sizeof(*wd), GFP_ATOMIC);
	if (!wd)
		return;

	wd->provider = info;
	INIT_WORK(&wd->work, __free_provider);
	schedule_work(&wd->work);
}

static void unregister_all_providers(void)
{
	struct eventlib_provider_info *info, *next;

	spin_lock(&ctx.lock);
	list_for_each_entry_safe(info, next, &ctx.providers, list)
		free_provider(info);
	spin_unlock(&ctx.lock);
}

int keventlib_write(int id, void *data, size_t size, uint32_t type, uint64_t ts)
{
	int err = 0;
	struct eventlib_provider_info *info;

	pr_debug("%s: size: %#zx\n", __func__, size);

	spin_lock(&ctx.lock);

	info = find_provider_info(id);
	if (!info) {
		err = -ENOENT;
		goto err_out;
	}

	if (!info->data) {
		err = -ENOMEM;
		goto err_out;
	}

	eventlib_write(&info->el_ctx, 0, type, ts, data, size);

err_out:
	spin_unlock(&ctx.lock);
	return err;
}
EXPORT_SYMBOL(keventlib_write);

int keventlib_register(size_t size, const char *name,
		       const char *schema, size_t schema_size)
{
	int ret;
	struct eventlib_provider_info *info;

	if (!is_initialized) {
		pr_warn("keventlib is not initialized\n");
		return -EACCES;
	}

	info = kmalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	ret = provider_init(info, size, name, schema, schema_size);
	if (ret < 0) {
		kfree(info);
		return ret;
	}

	return info->id;
}
EXPORT_SYMBOL(keventlib_register);

void keventlib_unregister(int id)
{
	struct eventlib_provider_info *info;

	if (!is_initialized) {
		pr_warn("keventlib is not initialized\n");
		return;
	}

	spin_lock(&ctx.lock);

	info = find_provider_info(id);
	if (!info) {
		pr_err("Unregistered provider: %d\n", id);
		spin_unlock(&ctx.lock);
		return;
	}

	free_provider(info);

	spin_unlock(&ctx.lock);
}
EXPORT_SYMBOL(keventlib_unregister);

static void put_test_data(int id)
{
	int i;
	uint8_t value = 0;
	uint64_t ts = 0;
	uint8_t *data;
	struct eventlib_test_sample *s;
	uint8_t buffer[sizeof(struct eventlib_test_sample) +
		       EVENTLIB_TEST_DATA_SIZE];

	for (i = 0; i < 32; i++) {
		s = (struct eventlib_test_sample *)buffer;
		data = (uint8_t *)(s + 1);

		s->magic = EVENTLIB_TEST_SAMPLE_MAGIC;
		s->size = EVENTLIB_TEST_DATA_SIZE;
		memset(data, value++, s->size);
		s->crc = crc32(~0U, data, s->size) ^ 0xffffffff;

		keventlib_write(ctx.test_id, buffer,
				sizeof(buffer), 10 + id, ts++);
	}
}

static int __init
eventlib_module_init(void)
{
	int ret;

	if (is_initialized)
		return -ENOMEM;

	atomic_set(&ctx.nr_providers, 0);

	INIT_LIST_HEAD(&ctx.providers);
	spin_lock_init(&ctx.lock);

	ctx.kobj_root = kobject_create_and_add(EVENTLIB_SYSFS_DIR_NAME,
					       kernel_kobj);
	if (ctx.kobj_root == NULL) {
		pr_err("Unable to create sysfs directory: %s\n",
		       EVENTLIB_SYSFS_DIR_NAME);
		return -ENOMEM;
	}

	is_initialized = 1;

	ret = keventlib_register(EVENTLIB_TEST_SHM_SIZE,
				 EVENTLIB_SYSFS_TEST_FILE_NAME,
				 NULL, 0);
	if (ret < 0) {
		kobject_put(ctx.kobj_root);
		is_initialized = 0;
		return ret;
	}

	ctx.test_id = ret;
	put_test_data(ctx.test_id);

	pr_info("keventlib is initialized, test id: %d\n", ctx.test_id);

	return 0;
}

static void __exit
eventlib_module_exit(void)
{
	unregister_all_providers();
	pr_info("keventlib is uninitialized\n");
}

subsys_initcall(eventlib_module_init);
module_exit(eventlib_module_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Nvidia Ltd");
MODULE_DESCRIPTION("Kernel Eventlib");
MODULE_VERSION(KEVENTLIB_VERSION);
