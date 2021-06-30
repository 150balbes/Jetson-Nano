/*
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
 */

#define pr_fmt(fmt)	"%s(): " fmt, __func__

#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/debugfs.h>
#include <linux/tegra-mce.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <soc/tegra/chip-id.h>
#include <uapi/linux/tegra_l3_cache.h>
#include <soc/tegra/chip-id.h>
#include <linux/delay.h>

#define MASK GENMASK(15, 12)
#define T19x_CACHE_STR	"l3_cache"

#define CCPLEX_CC_GPU_ONLY_BITS_SHIFT	8

#define L3_WAYS_MASK			GENMASK(4, 0)

#define PROFILE_DCACHE_FLUSH		0

#ifdef CONFIG_DEBUG_FS
static struct dentry *t19x_l3cache_root;
#endif

struct cache_drv_data {
	struct platform_device *pdev;
	struct miscdevice dev_user;
	struct tegra_l3_ioctl_data ioctl_data;
};

static struct cache_drv_data *cache_data;


int t19x_flush_cache_all(void)
{
	u64 id_afr0;
	u64 ret;
	u64 retry = 10;

	asm volatile ("mrs %0, ID_AFR0_EL1" : "=r"(id_afr0));
	/* check if cache flush through mts is supported */
	if (!likely(id_afr0 & MASK)) {
		pr_warn("SCF cache flush all is not supported in MTS\n");
		return -ENOTSUPP;
	}

	do {
		asm volatile ("mrs %0, s3_0_c15_c3_7" : "=r" (ret));
	} while (!ret && retry--);
	asm volatile ("dsb sy");

	if (!ret) {
		WARN_ONCE(!ret, "%s failed\n", __func__);
		pr_err("SCF cache flush all: instruction error\n");
		return -EINVAL;
	}

	return 0;
}

#if PROFILE_DCACHE_FLUSH
struct flush_error {
	u64 run_time;
	u64 ret;
};

static int __t19x_flush_dcache_all(void)
{
#define FLUSH_TO_IN_MS 1000
#define RETRY 10
	static struct flush_error fe[RETRY] = { { 0 } };
	ktime_t kt, tt = { 0 };
	u64 id_afr0;
	u64 i = 0;
	u64 ret;

	asm volatile ("mrs %0, ID_AFR0_EL1" : "=r"(id_afr0));
	/* check if dcache flush through mts is supported */
	if (!likely(id_afr0 & MASK)) {
		pr_warn("SCF dcache flush is not supported in MTS\n");
		return -ENOTSUPP;
	}

	do {
		kt = ktime_get();
		asm volatile ("mrs %0, s3_0_c15_c3_6" : "=r" (ret));
		kt = ktime_sub(ktime_get(), kt);

		fe[i].run_time = ktime_to_ns(kt);
		fe[i].ret = ret;
		i++;

		tt = ktime_add(tt, kt);
		if (ktime_to_ms(tt) >= FLUSH_TO_IN_MS)
			break;

		udelay(1);
	} while (!ret);
	asm volatile ("dsb sy");

	i = 0;

	if (!ret) {
		pr_err("SCF dcache flush: instruction error\n");
		while (fe[i].run_time) {
			pr_err("flush_dcache_all_exec_time[%llu]:%lluns\n",
				 i,  fe[i].run_time);
			pr_err("flush_dcache_all_return[%llu]:%llu\n",
				 i,  fe[i].ret);
			i++;
		}
		WARN_ONCE(!ret, "%s failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}
#endif

int t19x_flush_dcache_all(void)
{
#if PROFILE_DCACHE_FLUSH
	return __t19x_flush_dcache_all();
#else
	u64 id_afr0;
	u64 ret;
	u64 retry = 10;

	asm volatile ("mrs %0, ID_AFR0_EL1" : "=r"(id_afr0));
	/* check if dcache flush through mts is supported */
	if (!likely(id_afr0 & MASK)) {
		pr_warn("SCF dcache flush is not supported in MTS\n");
		return -ENOTSUPP;
	}

	do {
		asm volatile ("mrs %0, s3_0_c15_c3_6" : "=r" (ret));
		udelay(1);
	} while (!ret && retry--);
	asm volatile ("dsb sy");

	if (!ret) {
		WARN_ONCE(!ret, "%s failed\n", __func__);
		pr_err("SCF dcache flush: instruction error\n");
		return -EINVAL;
	}

	return 0;
#endif
}

int t19x_clean_dcache_all(void)
{
	u64 id_afr0;
	u64 ret;
	u64 retry = 10;

	asm volatile ("mrs %0, ID_AFR0_EL1" : "=r"(id_afr0));
	/* check if dcache clean through mts is supported */
	if (!likely(id_afr0 & MASK)) {
		pr_err("SCF dcache clean is not supported in MTS\n");
		return -ENOTSUPP;
	}

	do {
		asm volatile ("mrs %0, s3_0_c15_c3_5" : "=r" (ret));
	} while (!ret && retry--);
	asm volatile ("dsb sy");

	if (!ret) {
		WARN_ONCE(!ret, "%s failed\n", __func__);
		pr_err("SCF dcache clean: instruction error\n");
		return -EINVAL;
	}

	return 0;
}

static int t19x_extract_l3_cache_ways(struct device *dev)
{
	u64 nvg_data;
	u32 *gpu_cpu_ways = &cache_data->ioctl_data.igpu_cpu_ways;
	u32 *gpu_only_ways = &cache_data->ioctl_data.igpu_only_ways;
	u32 dt_gpu_cpu_ways = *gpu_cpu_ways;
	u32 dt_gpu_only_ways = *gpu_only_ways;

	if (tegra_mce_read_l3_cache_ways(&nvg_data)) {
		dev_err(dev, "Reading L3 cache ways is not supported in this platform\n");
		return -ENOTSUPP;
	}

	*gpu_cpu_ways = nvg_data & L3_WAYS_MASK;
	*gpu_only_ways = (nvg_data >> CCPLEX_CC_GPU_ONLY_BITS_SHIFT) & L3_WAYS_MASK;

	if ((dt_gpu_cpu_ways != *gpu_cpu_ways) ||
	    (dt_gpu_only_ways != *gpu_only_ways)) {
		dev_info(dev, "L3 ways information mismatch "
			 "between HW and DT!\n"
			 "DT: %d  %d HW: %d %d\n",
			 dt_gpu_cpu_ways, dt_gpu_only_ways,
			 *gpu_cpu_ways, *gpu_only_ways);
	}

	return 0;
}

static int t19x_set_l3_cache_ways(u32 gpu_cpu_ways, u32 gpu_only_ways)
{
	struct device *dev = &cache_data->pdev->dev;
	u64 nvg_data;
	u64 ret;

	if ((gpu_only_ways > cache_data->ioctl_data.total_ways) ||
		(gpu_cpu_ways > cache_data->ioctl_data.total_ways)) {
		dev_err(dev, "gpu_cpu_ways:%u or gpu_only_ways:%u"
			"exceeds total_ways:%u!!\n",
			gpu_cpu_ways, gpu_only_ways,
			cache_data->ioctl_data.total_ways);
		return -EINVAL;
	}

	if (gpu_cpu_ways < gpu_only_ways) {
		dev_err(dev, "gpu_cpu_ways:%u is smaller than gpu_only_ways:%u\n",
			gpu_cpu_ways, gpu_only_ways);
		return -EINVAL;
	}

	gpu_only_ways <<= CCPLEX_CC_GPU_ONLY_BITS_SHIFT;

	nvg_data = gpu_cpu_ways | gpu_only_ways;

	if (tegra_mce_write_l3_cache_ways(nvg_data, &ret)) {
		dev_err(dev, "Writing L3 cache ways is not supported in this platform\n");
		return -ENOTSUPP;
	}
	if (ret != nvg_data) {
		dev_err(dev, "CCPLEX_CACHE_CONTROL contents are not updated!!\n");
		return -ENODEV;
	}

	return 0;
}

static ssize_t gpu_only_ways_show(struct device *device,
				struct device_attribute *attr, char *buf)
{
	struct cache_drv_data *pdata;

	t19x_extract_l3_cache_ways(device);

	pdata = dev_get_drvdata(device);

	return sprintf(buf, "%u\n", pdata->ioctl_data.igpu_only_ways);
}

static ssize_t gpu_only_ways_store(struct device *device,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct cache_drv_data *pdata;
	u32 new_gpu_only_ways;
	u32 gpu_cpu_ways;

	if (is_tegra_hypervisor_mode())
		return -EPERM;

	if (kstrtou32(buf, 0, &new_gpu_only_ways) < 0)
		return -EINVAL;

	t19x_extract_l3_cache_ways(device);
	pdata = dev_get_drvdata(device);
	gpu_cpu_ways = pdata->ioctl_data.igpu_cpu_ways;
	t19x_set_l3_cache_ways(gpu_cpu_ways, new_gpu_only_ways);

	return count;
}

static ssize_t gpu_cpu_ways_show(struct device *device,
				struct device_attribute *attr, char *buf)
{
	struct cache_drv_data *pdata;

	t19x_extract_l3_cache_ways(device);

	pdata = dev_get_drvdata(device);

	return sprintf(buf, "%u\n", pdata->ioctl_data.igpu_cpu_ways);
}

static ssize_t gpu_cpu_ways_store(struct device *device,
				struct device_attribute *attr, const char *buf,
				size_t count)
{
	struct cache_drv_data *pdata;
	u32 new_gpu_cpu_ways;
	u32 gpu_only_ways;

	if (is_tegra_hypervisor_mode())
		return -EPERM;

	if (kstrtou32(buf, 0, &new_gpu_cpu_ways) < 0)
		return -EINVAL;

	t19x_extract_l3_cache_ways(device);
	pdata = dev_get_drvdata(device);
	gpu_only_ways = pdata->ioctl_data.igpu_only_ways;
	t19x_set_l3_cache_ways(new_gpu_cpu_ways, gpu_only_ways);

	return count;
}

static DEVICE_ATTR(l3_gpu_only_ways, 0644,
	gpu_only_ways_show, gpu_only_ways_store);
static DEVICE_ATTR(l3_gpu_cpu_ways, 0644,
	gpu_cpu_ways_show, gpu_cpu_ways_store);

static int t19x_cache_sysfs_create(struct device *dev)
{
	int err;

	err = device_create_file(dev, &dev_attr_l3_gpu_only_ways);
	if (err) {
		dev_warn(dev, "Couldn't create gpu_only_ways attribute err%d\n",
			err);
		return err;
	}

	err = device_create_file(dev, &dev_attr_l3_gpu_cpu_ways);
	if (err) {
		dev_warn(dev, "Couldn't create gpu_cpu_ways attribute err%d\n",
			err);
		device_remove_file(dev, &dev_attr_l3_gpu_only_ways);
		return err;
	}

	return 0;
}
static void t19x_cache_sysfs_remove(struct device *dev)
{
	device_remove_file(dev, &dev_attr_l3_gpu_only_ways);
	device_remove_file(dev, &dev_attr_l3_gpu_cpu_ways);
}

static int t19x_parse_dt(void)
{
	struct device *dev = &cache_data->pdev->dev;
	struct device_node *np = dev->of_node;
	struct tegra_l3_ioctl_data *idata = &cache_data->ioctl_data;
	int ret = 0;


	if (!np) {
		dev_err(dev, "Did not find device node\n");
		return -EINVAL;
	}

	if (!of_device_is_available(np)) {
		dev_err(dev, "device is disabled\n");
		return -ENODEV;
	}

	ret =  of_property_read_u32_array(np, "l3-gpu-cpu-ways",
				&idata->igpu_cpu_ways, 1);
	if (ret) {
		dev_err(dev, "Did not find l3-gpu-cpu-ways property\n");
		return ret;
	}

	ret =  of_property_read_u32_array(np, "l3-gpu-only-ways",
				&idata->igpu_only_ways, 1);
	if (ret) {
		dev_err(dev, "Did not find l3-gpu-only-ways property\n");
		return ret;
	}

	ret =  of_property_read_u32_array(np, "l3-total-ways",
				&idata->total_ways, 1);
	if (ret) {
		dev_err(dev, "Did not find l3-total-ways property\n");
		return ret;
	}

	ret =  of_property_read_u64(np, "l3-size",
				&idata->size);
	if (ret) {
		dev_err(dev, "Did not find l3-size property\n");
		return ret;
	}

	return ret;
}

static int t19x_cache_open(struct inode *inode, struct file *filp)
{
	int err = 0;

	err = nonseekable_open(inode, filp);
	if (unlikely(err))
		return err;

	return err;
}

static int t19x_cache_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static int get_l3_data(void *arg)
{
	struct tegra_l3_ioctl_data *in_data =
			(struct tegra_l3_ioctl_data *)arg;

	memcpy(in_data, &cache_data->ioctl_data,
		sizeof(struct tegra_l3_ioctl_data));

	return 0;
}

static long t19x_cache_ioctl(struct file *filp, unsigned int cmd,
		unsigned long arg)
{
	u8 buf[TEGRA_L3_CACHE_IOCTL_MAX_ARG_SIZE] __aligned(sizeof(u64));
	struct device *dev = &cache_data->pdev->dev;
	void __user *uarg = (void __user *)arg;
	int err = 0;

	/* check for valid IOCTL cmd */
	if ((_IOC_TYPE(cmd) != TEGRA_L3_CACHE_IOC_MAGIC) ||
		(_IOC_NR(cmd) == _IOC_NR(0)) ||
		(_IOC_NR(cmd) > TEGRA_L3_CACHE_IOCTL_IOC_MAXNR) ||
		(_IOC_SIZE(cmd) > TEGRA_L3_CACHE_IOCTL_MAX_ARG_SIZE)) {
		return -ENOIOCTLCMD;
	}

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, uarg, _IOC_SIZE(cmd));
	if (!err && (_IOC_DIR(cmd) & _IOC_WRITE))
		err = !access_ok(VERIFY_READ, uarg, _IOC_SIZE(cmd));
	if (err)
		return -EFAULT;

	if (_IOC_DIR(cmd) & _IOC_WRITE) {
		if (copy_from_user(buf, (void __user *)arg, _IOC_SIZE(cmd)))
			return -EFAULT;
	}

	switch (cmd) {
	case TEGRA_L3_CACHE_GET_IOCTL_DATA:
		err = get_l3_data(buf);
		break;
	default:
		dev_err(dev, "Unknown NVMAP_IOC = 0x%x\n", cmd);
		return -ENOIOCTLCMD;
	}

	if ((err == 0) && (_IOC_DIR(cmd) & _IOC_READ))
		err = copy_to_user((void __user *)arg, buf, _IOC_SIZE(cmd));

	return err;
}

#ifdef CONFIG_DEBUG_FS
#define RW_MODE			(S_IWUSR | S_IRUGO)
#define RO_MODE			(S_IRUGO)

static int get_gpu_cpu_ways(void *data, u64 *val)
{
	*val = cache_data->ioctl_data.igpu_cpu_ways;
	return 0;
}

static int set_gpu_cpu_ways(void *data, u64 val)
{
	u32 igpu_cpu_ways = (u32)val;
	int ret = 0;

	if (is_tegra_hypervisor_mode())
		return -EPERM;

	if (igpu_cpu_ways != cache_data->ioctl_data.igpu_cpu_ways) {
		ret = t19x_set_l3_cache_ways(igpu_cpu_ways,
				cache_data->ioctl_data.igpu_only_ways);
		if (!ret)
			cache_data->ioctl_data.igpu_cpu_ways = igpu_cpu_ways;
	}

	return ret;
}
DEFINE_SIMPLE_ATTRIBUTE(gpu_cpu_ways_ops, get_gpu_cpu_ways, set_gpu_cpu_ways,
	"%llu\n");

static int get_gpu_only_ways(void *data, u64 *val)
{
	*val = cache_data->ioctl_data.igpu_only_ways;
	return 0;
}

static int set_gpu_only_ways(void *data, u64 val)
{
	u32 igpu_only_ways = (u32)val;
	int ret = 0;

	if (is_tegra_hypervisor_mode())
		return -EPERM;

	if (igpu_only_ways != cache_data->ioctl_data.igpu_only_ways) {
		ret = t19x_set_l3_cache_ways(
			cache_data->ioctl_data.igpu_cpu_ways, igpu_only_ways);
		if (!ret)
			cache_data->ioctl_data.igpu_only_ways = igpu_only_ways;
	}

	return ret;
}
DEFINE_SIMPLE_ATTRIBUTE(gpu_only_ways_ops, get_gpu_only_ways, set_gpu_only_ways,
	"%llu\n");

static int get_total_ways(void *data, u64 *val)
{
	*val = cache_data->ioctl_data.total_ways;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(total_ways_ops, get_total_ways, NULL,
	"%llu\n");

static int get_l3_cache_size(void *data, u64 *val)
{
	*val = cache_data->ioctl_data.size;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(size_ops, get_l3_cache_size, NULL,
	"%llu\n");

static int __init t19x_cache_debug_init(void)
{
	struct dentry *dir;
	uint8_t buff[15];

	snprintf(buff, sizeof(buff), T19x_CACHE_STR);
	dir = debugfs_create_dir(buff, t19x_l3cache_root);
	if (!dir)
		goto err_out;

	if (!debugfs_create_file("gpu_cpu_ways", RW_MODE, dir, NULL,
			&gpu_cpu_ways_ops))
		goto err_out;
	if (!debugfs_create_file("gpu_only_ways", RW_MODE, dir, NULL,
			&gpu_only_ways_ops))
		goto err_out;
	if (!debugfs_create_file("total_ways", RO_MODE, dir, NULL,
			&total_ways_ops))
		goto err_out;
	if (!debugfs_create_file("size", RO_MODE, dir, NULL,
			&size_ops))
		goto err_out;
	return 0;

err_out:
	debugfs_remove_recursive(t19x_l3cache_root);
	return -EINVAL;
}

static void t19x_cache_debug_exit(void)
{
	debugfs_remove_recursive(t19x_l3cache_root);
}
#endif /* End of debug fs */

static const struct file_operations t19x_cache_user_fops = {
	.owner		= THIS_MODULE,
	.open		= t19x_cache_open,
	.release	= t19x_cache_release,
	.unlocked_ioctl	= t19x_cache_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = t19x_cache_ioctl,
#endif
};

static int __init t19x_cache_probe(struct platform_device *pdev)
{
	struct cache_drv_data *drv_data;
	struct device *dev = &pdev->dev;
	int ret = 0;

	if (!tegra_platform_is_silicon()) {
		dev_err(dev, "is not supported on this platform\n");
		return -EPERM;
	}

	dev_dbg(dev, "in probe()...\n");

	drv_data = devm_kzalloc(dev, sizeof(*drv_data),
				GFP_KERNEL);
	if (!drv_data)
		return -ENOMEM;

	platform_set_drvdata(pdev, drv_data);

	drv_data->pdev = pdev;
	drv_data->dev_user.minor = MISC_DYNAMIC_MINOR;
	drv_data->dev_user.name = "l3cache";
	drv_data->dev_user.fops = &t19x_cache_user_fops;
	drv_data->dev_user.parent = &pdev->dev;
	cache_data = drv_data;

	ret = misc_register(&drv_data->dev_user);
	if (ret) {
		dev_err(dev, "unable to register miscdevice %s\n",
			drv_data->dev_user.name);
		goto err_out;
	}

	ret = t19x_parse_dt();
	if (ret)
		goto err_out;

	if (is_tegra_hypervisor_mode()) {
		ret = t19x_extract_l3_cache_ways(dev);
		if (ret) {
			dev_err(dev, "Could not extract gpu_cpu_ways from L3\n");
			goto err_out;
		}
	} else {
		ret = t19x_set_l3_cache_ways(cache_data->ioctl_data.igpu_cpu_ways,
					     cache_data->ioctl_data.igpu_only_ways);
		if (ret) {
			dev_err(dev, "Could not set gpu_cpu_ways in L3\n");
			goto err_out;
		}
	}

	ret = t19x_cache_sysfs_create(dev);
	if (ret) {
		dev_err(dev, "Failed to create sysfs attributes!\n");
		goto err_out;
	}

#ifdef CONFIG_DEBUG_FS
	ret = t19x_cache_debug_init();
	if (ret) {
		dev_err(dev, "Could not init debugfs\n");
		goto err_out;
	}
#endif

	dev_notice(dev, "probed\n");

	return 0;

err_out:
	dev_info(dev, "failed\n");
	if (drv_data->dev_user.minor != MISC_DYNAMIC_MINOR)
		misc_deregister(&drv_data->dev_user);
	devm_kfree(dev, drv_data);
	return ret;
}

static int __exit t19x_cache_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	t19x_cache_sysfs_remove(dev);

	misc_deregister(&cache_data->dev_user);
	devm_kfree(dev, cache_data);

#ifdef CONFIG_DEBUG_FS
	t19x_cache_debug_exit();
#endif

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id cache_of_match[] = {
	{ .compatible = "nvidia,t19x-cache", .data = NULL, },
	{},
};
#endif

static struct platform_driver t19x_cache_driver __refdata = {
	.driver	= {
		.name	= "t19x_cache",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(cache_of_match),
	},
	.probe		= t19x_cache_probe,
	.remove		= t19x_cache_remove,
};
module_platform_driver(t19x_cache_driver);

MODULE_DESCRIPTION("T19x Cache operations registration");
MODULE_AUTHOR("Sri Krishna chowdary K <schowdary@nvidia.com>");
MODULE_AUTHOR("Puneet Saxena <puneets@nvidia.com>");
MODULE_LICENSE("GPL v2");
