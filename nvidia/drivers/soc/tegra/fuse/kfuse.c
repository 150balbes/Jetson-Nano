/*
 * Copyright (c) 2010-2017, NVIDIA CORPORATION. All rights reserved.
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

/* The kfuse block stores downstream and upstream HDCP keys for use by HDMI
 * module.
 */

#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <soc/tegra/chip-id.h>
#include <soc/tegra/kfuse.h>

/* SOC specific Tegra kfuse information */
struct tegra_kfuse_soc {
	bool sensing_support;
};

struct tegra_kfuse {
	struct device *dev;
	struct clk *clk;
	void __iomem *aperture;
	const struct tegra_kfuse_soc *soc;
	unsigned int cg_refcount;

	/* Mutex for handling clockgating reference count */
	struct mutex cg_refcount_mutex;
};

/* Public API does not provide kfuse structure or device */
static struct tegra_kfuse *global_kfuse;

/* register definition */
#define KFUSE_PD		0x24
#define KFUSE_PD_PU		0u
#define KFUSE_PD_PD		BIT(0)

#define KFUSE_STATE		0x80
#define KFUSE_STATE_DONE	BIT(16)
#define KFUSE_STATE_CRCPASS	BIT(17)

#define KFUSE_KEYADDR		0x88
#define KFUSE_KEYADDR_AUTOINC	BIT(16)
#define KFUSE_KEYS		0x8c
#define KFUSE_CG1_0		0x90

static u32 tegra_kfuse_readl(struct tegra_kfuse *kfuse, unsigned long offset)
{
	return readl(kfuse->aperture + offset);
}

static void tegra_kfuse_writel(struct tegra_kfuse *kfuse, u32 value,
			       unsigned long offset)
{
	writel(value, kfuse->aperture + offset);
}

static struct tegra_kfuse *tegra_kfuse_get(void)
{
	return global_kfuse;
}

static int tegra_kfuse_wait_for_done(struct tegra_kfuse *kfuse)
{
	u32 reg;
	int retries = 50;

	do {
		reg = tegra_kfuse_readl(kfuse, KFUSE_STATE);
		if (reg & KFUSE_STATE_DONE)
			return 0;
		msleep(10);
	} while (--retries);
	return -ETIMEDOUT;
}

int tegra_kfuse_enable_sensing(void)
{
	struct tegra_kfuse *kfuse = tegra_kfuse_get();
	int err = 0;

	/* check that kfuse driver is available.. */
	if (!kfuse)
		return -ENODEV;

	mutex_lock(&kfuse->cg_refcount_mutex);

	/* increment refcount */
	kfuse->cg_refcount++;

	/* if clock was already up, quit */
	if (kfuse->cg_refcount > 1)
		goto exit_unlock;

	/* enable kfuse clock */
	err = clk_prepare_enable(kfuse->clk);
	if (err) {
		kfuse->cg_refcount--;
		goto exit_unlock;
	}

	/* enable kfuse sensing */
	tegra_kfuse_writel(kfuse, 1, KFUSE_CG1_0);

exit_unlock:
	mutex_unlock(&kfuse->cg_refcount_mutex);

	return err;
}
EXPORT_SYMBOL(tegra_kfuse_enable_sensing);

void tegra_kfuse_disable_sensing(void)
{
	struct tegra_kfuse *kfuse = tegra_kfuse_get();

	/* check that kfuse driver is available.. */
	if (!kfuse)
		return;

	mutex_lock(&kfuse->cg_refcount_mutex);

	if (WARN_ON(kfuse->cg_refcount == 0))
		goto exit_unlock;

	/* decrement refcount */
	kfuse->cg_refcount--;

	/* if there are still users, quit */
	if (kfuse->cg_refcount > 0)
		goto exit_unlock;

	/* disable kfuse sensing */
	tegra_kfuse_writel(kfuse, 0, KFUSE_CG1_0);

	/* ..and disable kfuse clock */
	clk_disable_unprepare(kfuse->clk);

exit_unlock:
	mutex_unlock(&kfuse->cg_refcount_mutex);
}
EXPORT_SYMBOL(tegra_kfuse_disable_sensing);

/* read up to KFUSE_DATA_SZ bytes into dest.
 * always starts at the first kfuse.
 */
int tegra_kfuse_read(void *dest, size_t len)
{
	struct tegra_kfuse *kfuse = tegra_kfuse_get();
	int err;
	u32 v;
	unsigned cnt;

	if (!kfuse)
		return -ENODEV;

	if (len > KFUSE_DATA_SZ)
		return -EINVAL;

	if (kfuse->soc->sensing_support) {
		err = tegra_kfuse_enable_sensing();
		if (err)
			return err;
	}

	err = clk_prepare_enable(kfuse->clk);
	if (err)
		return err;

	tegra_kfuse_writel(kfuse, KFUSE_PD_PU, KFUSE_PD);
	udelay(2);

	tegra_kfuse_writel(kfuse, KFUSE_KEYADDR_AUTOINC, KFUSE_KEYADDR);

	err = tegra_kfuse_wait_for_done(kfuse);
	if (err) {
		dev_err(kfuse->dev, "kfuse: read timeout\n");
		clk_disable_unprepare(kfuse->clk);
		return err;
	}

	if ((tegra_kfuse_readl(kfuse, KFUSE_STATE) &
			       KFUSE_STATE_CRCPASS) == 0) {
		dev_err(kfuse->dev, "kfuse: crc failed\n");
		clk_disable_unprepare(kfuse->clk);
		return -EIO;
	}

	for (cnt = 0; cnt < len; cnt += 4) {
		v = tegra_kfuse_readl(kfuse, KFUSE_KEYS);
		memcpy(dest + cnt, &v, sizeof(v));
	}

	tegra_kfuse_writel(kfuse, KFUSE_PD_PD, KFUSE_PD);

	clk_disable_unprepare(kfuse->clk);

	if (kfuse->soc->sensing_support)
		tegra_kfuse_disable_sensing();

	return 0;
}
EXPORT_SYMBOL(tegra_kfuse_read);

static int tegra_kfuse_probe(struct platform_device *pdev)
{
	struct tegra_kfuse *kfuse;
	struct resource *resource;
	int err;

	if (global_kfuse)
		return -EBUSY;

	kfuse = devm_kzalloc(&pdev->dev, sizeof(*kfuse), GFP_KERNEL);
	if (!kfuse)
		return -ENOMEM;

	kfuse->soc = of_device_get_match_data(&pdev->dev);
	kfuse->dev = &pdev->dev;

	kfuse->clk = devm_clk_get(&pdev->dev, "kfuse");

	if (IS_ERR(kfuse->clk)) {
		err = PTR_ERR(kfuse->clk);
		dev_err(&pdev->dev, "Failed to get kfuse clk: %d\n", err);
		return err;
	}

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!resource) {
		dev_err(&pdev->dev, "Failed to get MEM resource\n");
		return -EINVAL;
	}

	kfuse->aperture = devm_ioremap_resource(&pdev->dev, resource);
	if (IS_ERR(kfuse->aperture)) {
		err = PTR_ERR(kfuse->aperture);
		dev_err(&pdev->dev, "Failed to ioremap: %d\n", err);
		return err;
	}

	mutex_init(&kfuse->cg_refcount_mutex);

	platform_set_drvdata(pdev, kfuse);

	/* for public API */
	global_kfuse = kfuse;

	dev_info(&pdev->dev, "initialized\n");

	return 0;
}

static int tegra_kfuse_remove(struct platform_device *pdev)
{
	struct tegra_kfuse *kfuse = platform_get_drvdata(pdev);
	int ret = 0;

	/* ensure that no-one is using sensing now */
	mutex_lock(&kfuse->cg_refcount_mutex);
	if (kfuse->cg_refcount)
		ret = -EBUSY;
	mutex_unlock(&kfuse->cg_refcount_mutex);

	if (ret < 0)
		return ret;

	global_kfuse = NULL;

	dev_info(&pdev->dev, "removed\n");

	return 0;
}

static const struct tegra_kfuse_soc tegra124_kfuse_soc = {
	.sensing_support = false,
};

static const struct tegra_kfuse_soc tegra210_kfuse_soc = {
	.sensing_support = false,
};

static const struct tegra_kfuse_soc tegra186_kfuse_soc = {
	.sensing_support = true,
};

static const struct tegra_kfuse_soc tegra194_kfuse_soc = {
	.sensing_support = true,
};

static const struct of_device_id tegra_kfuse_of_match[] = {
	{ .compatible = "nvidia,tegra124-kfuse", .data = &tegra124_kfuse_soc, },
	{ .compatible = "nvidia,tegra210-kfuse", .data = &tegra210_kfuse_soc, },
	{ .compatible = "nvidia,tegra186-kfuse", .data = &tegra186_kfuse_soc, },
	{ .compatible = "nvidia,tegra194-kfuse", .data = &tegra194_kfuse_soc, },
	{ },
};

static struct platform_driver kfuse_driver = {
	.probe = tegra_kfuse_probe,
	.remove = tegra_kfuse_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "kfuse",
		.of_match_table = tegra_kfuse_of_match,
	},
};

module_platform_driver(kfuse_driver);
