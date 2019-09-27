/*
 * drivers/platform/tegra/cvnas.c
 *
 * Copyright (C) 2017-2018, NVIDIA Corporation.  All rights reserved.
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

#ifdef pr_fmt
#undef pr_fmt
#endif

#define pr_fmt(fmt) "cvnas: %s,%d" fmt, __func__, __LINE__

#include <linux/compiler.h>
#include <linux/reset.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/nvmap_t19x.h>
#include <soc/tegra/chip-id.h>
#include <soc/tegra/fuse.h>
#include <linux/clk-provider.h>

static int cvnas_debug;
module_param(cvnas_debug, int, 0644);

#define CVSRAM_MEM_INIT_OFFSET		0x00
#define CVSRAM_MEM_INIT_START		BIT(0)
#define CVSRAM_MEM_INIT_STATUS		BIT(1)

#define CVSRAM_RD_COUNT_OFFSET		0x008
#define CVSRAM_WR_COUNT_OFFSET		0x00C
#define CVSRAM_STALLED_RD_COUNT_OFFSET	0x010
#define CVSRAM_STALLED_WR_COUNT_OFFSET	0x014

#define CVSRAM_PWR_CTRL_OFFSET		0x018

#define CVSRAM_EC_MERR_FORCE_OFFSET	0x83C
#define CVSRAM_EC_MERR_ECC_INJECT	0xFFFFFF

#define ERRCOLLATOR_MISSIONERR_STATUS	0x840

#define CVNAS_EC_MERR_FORCE_OFFSET	0xF134
#define CVNAS_EC_MERR_ECC_INJECT	0x1FE

#define MEM_INIT_FCM			0x1
#define DEV_CVNAS_CLR_RST		0x2

#define HSM_CVSRAM_ECC_CORRECT_OFFSET	0x1A8
#define HSM_CVSRAM_ECC_DED_OFFSET_0	0x180
#define HSM_CVSRAM_ECC_DED_OFFSET_1	0x184

#define HSM_CVSRAM_ECC_CORRECT_MASK	0x0F000000
#define HSM_CVSRAM_ECC_DED_MASK_0	0x80000000
#define HSM_CVSRAM_ECC_DED_MASK_1	0x00000007

struct cvnas_device {
	struct dentry *debugfs_root;

	void __iomem *cvsram_iobase;
	void __iomem *cvreg_iobase;
	void __iomem *hsm_iobase;

	struct device dma_dev;

	int nslices;
	int slice_size;
	phys_addr_t cvsram_base;
	size_t cvsram_size;

	struct clk *clk;

	struct reset_control *rst;
	struct reset_control *rst_fcm;

	bool virt;

	int (*pmops_busy)(void);
	int (*pmops_idle)(void);
};

static struct platform_device *cvnas_plat_dev;

static u32 nvcvsram_readl(struct cvnas_device *dev, int sid, u32 reg)
{
	return readl(dev->cvsram_iobase + dev->slice_size * sid + reg);
}

static void nvcvsram_writel(struct cvnas_device *dev, int sid, u32 val, u32 reg)
{
	writel(val, dev->cvsram_iobase + dev->slice_size * sid + reg);
}

static u32 nvhsm_readl(struct cvnas_device *dev, u32 reg)
{
	return readl(dev->hsm_iobase + reg);
}

static int cvsram_perf_counters_show(struct seq_file *s, void *data)
{
	struct cvnas_device *dev = s->private;
	int i;
	u32 val;

	if (!dev) {
		seq_printf(s, "Invalid cvnas device!\n");
		return -EINVAL;
	}

	seq_printf(s, "RD:  ");
	for (i = 0; i < dev->nslices; i++) {
		val = nvcvsram_readl(dev, i, CVSRAM_RD_COUNT_OFFSET);
		seq_printf(s, "%x ", val);
	}
	seq_printf(s, "\nWR:  ");
	for (i = 0; i < dev->nslices; i++) {
		val = nvcvsram_readl(dev, i, CVSRAM_WR_COUNT_OFFSET);
		seq_printf(s, "%x ", val);
	}
	seq_printf(s, "\nSRD: ");
	for (i = 0; i < dev->nslices; i++) {
		val = nvcvsram_readl(dev, i, CVSRAM_STALLED_RD_COUNT_OFFSET);
		seq_printf(s, "%x ", val);
	}
	seq_printf(s, "\nSWR: ");
	for (i = 0; i < dev->nslices; i++) {
		val = nvcvsram_readl(dev, i, CVSRAM_STALLED_WR_COUNT_OFFSET);
		seq_printf(s, "%x ", val);
	}
	seq_printf(s, "\n");
	return 0;
}

static int cvsram_perf_counter_open(struct inode *inode, struct file *file)
{
	return single_open(file, cvsram_perf_counters_show,
				inode->i_private);
}

static const struct file_operations cvsram_perf_fops = {
	.open = cvsram_perf_counter_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int cvsram_ecc_err_inject(struct seq_file *s, void *data)
{
	struct cvnas_device *dev = (struct cvnas_device *)s->private;
	int i;
	u32 val;

	if (!dev) {
		seq_printf(s, "Invalid cvnas device!\n");
		return -EINVAL;
	}

	for (i = 0; i < dev->nslices; i++) {
		nvcvsram_writel(dev, i, CVSRAM_EC_MERR_ECC_INJECT,
					CVSRAM_EC_MERR_FORCE_OFFSET);
		if (cvnas_debug) {
			val = nvcvsram_readl(dev, i,
					 CVSRAM_EC_MERR_FORCE_OFFSET);
			seq_printf(s, "CVSRAM_EC_MERR_FORCE_OFFSET_SLICE%d: 0x%x : 0x%x\n",
					i, CVSRAM_EC_MERR_FORCE_OFFSET, val);
		}
	}

	for (i = 0; i < dev->nslices; i++) {
		if (cvnas_debug) {
			val = nvcvsram_readl(dev, i,
					 ERRCOLLATOR_MISSIONERR_STATUS);
			seq_printf(s, "ERRCOLLATOR_SLICE0_ERRSLICE0_MISSIONERR_STATUS_SLICE%d: 0x%x : 0x%x\n",
					i, ERRCOLLATOR_MISSIONERR_STATUS, val);
		}
	}

	val = nvhsm_readl(dev, HSM_CVSRAM_ECC_CORRECT_OFFSET);
	if (val & HSM_CVSRAM_ECC_CORRECT_MASK) {
		seq_printf(s, "HSM received ECC corrected SEC error\n");
	}
	val = nvhsm_readl(dev, HSM_CVSRAM_ECC_DED_OFFSET_0);
	if (val & HSM_CVSRAM_ECC_DED_MASK_0) {
		seq_printf(s, "HSM received ECC DED_0 error\n");
	}
	val = nvhsm_readl(dev, HSM_CVSRAM_ECC_DED_OFFSET_1);
	if (val & HSM_CVSRAM_ECC_DED_MASK_1) {
		seq_printf(s, "HSM received ECC DED_1 error\n");
	}

	return 0;
}

static int cvsram_ecc_err_open(struct inode *inode, struct file *file)
{
	return single_open(file, cvsram_ecc_err_inject,
				inode->i_private);
}

static const struct file_operations cvsram_ecc_err_fops = {
	.open = cvsram_ecc_err_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int nvcvnas_debugfs_init(struct cvnas_device *dev)
{
	struct dentry *root;

	root = debugfs_create_dir("cvnas", NULL);
	if (!root)
		return PTR_ERR(root);

	debugfs_create_x64("cvsram_base", S_IRUGO, root, &dev->cvsram_base);
	debugfs_create_size_t("cvsram_size", S_IRUGO, root, &dev->cvsram_size);
	debugfs_create_file("cvsram_perf_counters", S_IRUGO, root, dev, &cvsram_perf_fops);
	debugfs_create_file("inject_cvsram_ecc_error", S_IRUGO, root, dev, &cvsram_ecc_err_fops);
	dev->debugfs_root = root;
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int nvcvsram_ecc_setup(struct cvnas_device *dev)
{
	u32 mem_init = 0;
	int i;

	/* enable clock if disabled */

	for (i = 0; i < dev->nslices; i++) {
		mem_init = nvcvsram_readl(dev, i, CVSRAM_MEM_INIT_OFFSET);
		if (mem_init & CVSRAM_MEM_INIT_STATUS)
			return 0;
		nvcvsram_writel(dev, i, MEM_INIT_FCM, CVSRAM_MEM_INIT_OFFSET);
	}

	for (i = 0; i < dev->nslices; i++) {
		while (1) {
			usleep_range(100, 200);
			mem_init = nvcvsram_readl(dev, i,
					CVSRAM_MEM_INIT_OFFSET);
			/* FIXME: Use CCF to make sure clock runs
			 * at fixed frequency and wait for just
			 * that much time.
			 */
			if (((mem_init & CVSRAM_MEM_INIT_STATUS) >> 1) & 1)
				break;
		}
	}

	if (mem_init & CVSRAM_MEM_INIT_STATUS)
		return 0;
	return -EBUSY;
}

static int nvcvnas_power_on(struct cvnas_device *cvnas_dev)
{
	u32 fcm_upg_seq[] =
		{0xFE, 0xFC, 0xF8, 0xF0, 0xE0, 0xC0, 0x80, 0x00};

	int i, j;
	int err;

	if (!tegra_platform_is_silicon()) {
		pr_err("is not supported on this platform\n");
		return 0;
	}

	if (cvnas_dev->virt)
		return 0;

	err = clk_prepare_enable(cvnas_dev->clk);
	if (err < 0)
		goto err_enable_clk;

	err = reset_control_deassert(cvnas_dev->rst);
	if (err < 0)
		goto err_deassert_reset;

	for (i = 0; i < ARRAY_SIZE(fcm_upg_seq); i++) {
		for (j = 0; j < cvnas_dev->nslices; j++) {
			nvcvsram_writel(cvnas_dev, j, fcm_upg_seq[i],
					CVSRAM_PWR_CTRL_OFFSET);
			if (cvnas_debug) {
				u32 val = nvcvsram_readl(cvnas_dev, j,
						CVSRAM_PWR_CTRL_OFFSET);
				pr_info("Set SRAM%d_CVSRAM_PWR_CTRL %x to %x\n",
					j, CVSRAM_PWR_CTRL_OFFSET, val);
			}
		}
	}

	err = reset_control_deassert(cvnas_dev->rst_fcm);
	if (err < 0)
		goto err_deassert_fcm_reset;

	err = nvcvsram_ecc_setup(cvnas_dev);
	if (err < 0) {
		pr_err("ECC init failed\n");
		goto err_init_ecc;
	}

	return 0;

err_init_ecc:
	reset_control_assert(cvnas_dev->rst_fcm);
err_deassert_fcm_reset:
	reset_control_assert(cvnas_dev->rst);
err_deassert_reset:
	clk_disable_unprepare(cvnas_dev->clk);
err_enable_clk:
	return err;
}
#endif /* CONFIG_PM_SLEEP */

static int nvcvnas_power_off(struct cvnas_device *cvnas_dev)
{
	int val, i, j;
	u32 fcm_pg_seq[] =
		{0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF};

	if (!tegra_platform_is_silicon()) {
		pr_err("is not supported on this platform\n");
		return 0;
	}

	if (cvnas_dev->virt)
		return 0;

	reset_control_assert(cvnas_dev->rst_fcm);

	/* FCM low power mode */
	for (i = 0; i < ARRAY_SIZE(fcm_pg_seq); i++) {
		for (j = 0; j < cvnas_dev->nslices; j++) {
			nvcvsram_writel(cvnas_dev, j, fcm_pg_seq[i],
					CVSRAM_PWR_CTRL_OFFSET);
			if (cvnas_debug) {
				val = nvcvsram_readl(cvnas_dev, j,
						CVSRAM_PWR_CTRL_OFFSET);
				pr_info("Set SRAM%d_CVSRAM_PWR_CTRL %x to %x\n",
					j, CVSRAM_PWR_CTRL_OFFSET, val);
			}
		}
	}

	reset_control_assert(cvnas_dev->rst);
	clk_disable_unprepare(cvnas_dev->clk);

	return 0;
}

/* Call at the time we allocate something from CVNAS */
int nvcvnas_busy(void)
{
	if (!cvnas_plat_dev) {
		pr_err("CVNAS Platform Device not found\n");
		return -ENODEV;
	}

	return pm_runtime_get_sync(&cvnas_plat_dev->dev);
}
EXPORT_SYMBOL(nvcvnas_busy);

/* Call after we release a buffer */
int nvcvnas_idle(void)
{
	if (!cvnas_plat_dev) {
		pr_err("CVNAS Platform Device not found\n");
		return -ENODEV;
	}

	return pm_runtime_put(&cvnas_plat_dev->dev);
}
EXPORT_SYMBOL(nvcvnas_idle);

phys_addr_t nvcvnas_get_cvsram_base(void)
{
	struct cvnas_device *cvnas_dev = dev_get_drvdata(&cvnas_plat_dev->dev);

	return cvnas_dev->cvsram_base;
}
EXPORT_SYMBOL(nvcvnas_get_cvsram_base);

size_t nvcvnas_get_cvsram_size(void)
{
	struct cvnas_device *cvnas_dev = dev_get_drvdata(&cvnas_plat_dev->dev);

	return cvnas_dev->cvsram_size;
}
EXPORT_SYMBOL(nvcvnas_get_cvsram_size);

int is_nvcvnas_probed(void)
{
	if (cvnas_plat_dev && dev_get_drvdata(&cvnas_plat_dev->dev))
		return 1;
	else
		return 0;
}

int is_nvcvnas_clk_enabled(void)
{
	struct cvnas_device *cvnas_dev = dev_get_drvdata(&cvnas_plat_dev->dev);

	if (cvnas_plat_dev && cvnas_dev)
		return  __clk_is_enabled(cvnas_dev->clk);
	else
		return 0;
}
EXPORT_SYMBOL(is_nvcvnas_clk_enabled);

static const struct of_device_id nvcvnas_of_ids[] = {
	{ .compatible = "nvidia,tegra-cvnas", .data = (void *)false, },
	{ .compatible = "nvidia,tegra-cvnas-hv", .data = (void *)true, },
	{ }
};

static int nvcvnas_probe(struct platform_device *pdev)
{
	struct cvnas_device *cvnas_dev;
	int ret;
	u32 cvsram_slice_data[2];
	u32 cvsram_reg_data[4];
	const struct of_device_id *match;

	if (tegra_get_chipid() == TEGRA_CHIPID_TEGRA19 &&
		tegra_get_sku_id() == 0x9E) {
		dev_err(&pdev->dev, "CVNAS IP is disabled in SKU.\n");
		return -ENODEV;
	}

	cvnas_plat_dev = pdev;

	cvnas_dev = (struct cvnas_device *)kzalloc(
			sizeof(*cvnas_dev), GFP_KERNEL);
	if (!cvnas_dev)
		return -ENOMEM;

	match = of_match_device(nvcvnas_of_ids, &pdev->dev);
	if (match)
		cvnas_dev->virt = (bool)match->data;

	cvnas_dev->cvreg_iobase = of_iomap(pdev->dev.of_node, 0);
	if (!cvnas_dev->cvreg_iobase) {
		dev_err(&pdev->dev, "No cvnas reg property found\n");
		ret = PTR_ERR(cvnas_dev->cvreg_iobase);
		goto err_of_iomap;
	}

	cvnas_dev->cvsram_iobase = of_iomap(pdev->dev.of_node, 1);
	if (!cvnas_dev->cvsram_iobase) {
		dev_err(&pdev->dev, "No cvsram reg property found\n");
		ret = PTR_ERR(cvnas_dev->cvsram_iobase);
		goto err_cvsram_of_iomap;
	}

	cvnas_dev->hsm_iobase = of_iomap(pdev->dev.of_node, 2);
	if (!cvnas_dev->hsm_iobase) {
		dev_err(&pdev->dev, "No hsm reg property found\n");
		ret = PTR_ERR(cvnas_dev->hsm_iobase);
		goto err_hsm_of_iomap;
	}

	ret = of_property_read_u32_array(pdev->dev.of_node,
		"cvsramslice", cvsram_slice_data,
		ARRAY_SIZE(cvsram_slice_data));
	if (ret) {
		dev_err(&pdev->dev, "no cvsramslice property found\n");
		goto err_cvsram_get_slice_data;
	}
	cvnas_dev->nslices = cvsram_slice_data[0];
	cvnas_dev->slice_size = cvsram_slice_data[1];

	ret = of_property_read_u32_array(pdev->dev.of_node,
		"cvsram-reg", cvsram_reg_data,
		ARRAY_SIZE(cvsram_reg_data));
	if (ret) {
		dev_err(&pdev->dev, "no cvsram-reg property found\n");
		goto err_cvsram_get_reg_data;
	}

	cvnas_dev->cvsram_base = ((u64)cvsram_reg_data[0]) << 32;
	cvnas_dev->cvsram_base |= cvsram_reg_data[1];
	cvnas_dev->cvsram_size = ((u64)cvsram_reg_data[2]) << 32;
	cvnas_dev->cvsram_size |= cvsram_reg_data[3];

	cvnas_dev->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(cvnas_dev->clk)) {
		ret = PTR_ERR(cvnas_dev->clk);
		goto err_get_clk;
	}

	cvnas_dev->rst = devm_reset_control_get(&pdev->dev, "rst");
	if (IS_ERR(cvnas_dev->rst)) {
		ret = PTR_ERR(cvnas_dev->rst);
		goto err_get_reset;
	}

	cvnas_dev->rst_fcm = devm_reset_control_get(&pdev->dev, "rst_fcm");
	if (IS_ERR(cvnas_dev->rst_fcm)) {
		ret = PTR_ERR(cvnas_dev->rst_fcm);
		goto err_get_reset_fcm;
	}

	pm_runtime_enable(&pdev->dev);

	ret = nvcvnas_debugfs_init(cvnas_dev);
	if (ret) {
		dev_err(&pdev->dev, "debugfs init failed. ret=%d\n", ret);
		goto err_cvnas_debugfs_init;
	}

	cvnas_dev->pmops_busy = nvcvnas_busy;
	cvnas_dev->pmops_idle = nvcvnas_idle;

	ret = nvmap_register_cvsram_carveout(&cvnas_dev->dma_dev,
			cvnas_dev->cvsram_base, cvnas_dev->cvsram_size,
			cvnas_dev->pmops_busy, cvnas_dev->pmops_idle);
	if (ret) {
		dev_err(&pdev->dev,
			"nvmap cvsram register failed. ret=%d\n", ret);
		goto err_cvsram_nvmap_heap_register;
	}

	dev_set_drvdata(&pdev->dev, cvnas_dev);

	/* TODO: Add interrupt handler */

	return 0;
err_cvsram_nvmap_heap_register:
	debugfs_remove(cvnas_dev->debugfs_root);
err_cvnas_debugfs_init:
err_get_reset_fcm:
err_get_reset:
err_get_clk:
err_cvsram_get_reg_data:
err_cvsram_get_slice_data:
	iounmap(cvnas_dev->hsm_iobase);
err_hsm_of_iomap:
	iounmap(cvnas_dev->cvsram_iobase);
err_cvsram_of_iomap:
	iounmap(cvnas_dev->cvreg_iobase);
err_of_iomap:
	kfree(cvnas_dev);
	return ret;
}

static int nvcvnas_remove(struct platform_device *pdev)
{
	struct cvnas_device *cvnas_dev;

	cvnas_dev = dev_get_drvdata(&pdev->dev);
	if (!cvnas_dev)
		return -ENODEV;

	debugfs_remove(cvnas_dev->debugfs_root);
	of_reserved_mem_device_release(&pdev->dev);
	iounmap(cvnas_dev->cvsram_iobase);
	iounmap(cvnas_dev->cvreg_iobase);
	kfree(cvnas_dev);
	return 0;
}

static void nvcvnas_shutdown(struct platform_device *pdev)
{
	struct cvnas_device *cvnas_dev;
	int ret;

	if (pm_runtime_suspended(&pdev->dev))
		return;

	cvnas_dev = dev_get_drvdata(&pdev->dev);
	if (!cvnas_dev) {
		dev_err(&pdev->dev, "shutdown fail\n");
		return;
	}

	ret = nvcvnas_power_off(cvnas_dev);
	if (ret)
		dev_err(&pdev->dev, "power off fail\n");
}

/* TODO: Add runtime power management */
#ifdef CONFIG_PM_SLEEP
static int nvcvnas_suspend(struct device *dev)
{
	struct cvnas_device *cvnas_dev;

	cvnas_dev = dev_get_drvdata(dev);
	if (!cvnas_dev)
		return -ENODEV;

	return nvcvnas_power_off(cvnas_dev);
}

static int nvcvnas_resume(struct device *dev)
{
	struct cvnas_device *cvnas_dev;
	int ret;

	cvnas_dev = dev_get_drvdata(dev);
	if (!cvnas_dev) {
		dev_err(dev, "empty drvdata!\n");
		return -ENODEV;
	}

	ret = nvcvnas_power_on(cvnas_dev);
	if (ret) {
		dev_err(dev, "cvnas power on failed\n");
		return ret;
	}
	return 0;
}

static int nvcvnas_runtime_suspend(struct device *dev)
{
	nvcvnas_suspend(dev);
	return 0;
}

static int nvcvnas_runtime_resume(struct device *dev)
{
	nvcvnas_resume(dev);
	return 0;
}

static const struct dev_pm_ops nvcvnas_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	SET_RUNTIME_PM_OPS(nvcvnas_runtime_suspend,
			nvcvnas_runtime_resume, NULL)
};

#define NVCVNAS_PM_OPS (&nvcvnas_pm_ops)
#else
#define NVCVNAS_PM_OPS NULL
#endif

/* Function to resume CV without using runtime pm.
 * CVNOC register setting is required by CBB driver during resume to
 * enable reporting CVNOC errors for illegal register accesses.
 */
int nvcvnas_busy_no_rpm(void)
{
#ifdef CONFIG_PM_SLEEP
	if (cvnas_plat_dev && dev_get_drvdata(&cvnas_plat_dev->dev))
		return nvcvnas_resume(&cvnas_plat_dev->dev);
	else
#endif
		return 0;
}
EXPORT_SYMBOL(nvcvnas_busy_no_rpm);

/*
 * Function to suspend CV without using runtime pm.
 */
int nvcvnas_idle_no_rpm(struct device *dev)
{
#ifdef CONFIG_PM_SLEEP
	if (cvnas_plat_dev && dev_get_drvdata(&cvnas_plat_dev->dev))
		return nvcvnas_suspend(&cvnas_plat_dev->dev);
	else
#endif
		return 0;
}
EXPORT_SYMBOL(nvcvnas_idle_no_rpm);

static struct platform_driver nvcvnas_driver = {
	.driver = {
		.name	= "tegra-cvnas",
		.owner	= THIS_MODULE,
		.of_match_table = nvcvnas_of_ids,
#ifdef CONFIG_PM
		.pm = NVCVNAS_PM_OPS,
#endif
	},

	.probe		= nvcvnas_probe,
	.remove		= nvcvnas_remove,
	.shutdown	= nvcvnas_shutdown,
};

static int __init nvcvnas_init(void)
{
	int ret;

	ret = platform_driver_register(&nvcvnas_driver);
	if (ret)
		return ret;

	return 0;
}
module_init(nvcvnas_init);

static void __exit nvcvnas_exit(void)
{
}
module_exit(nvcvnas_exit);
