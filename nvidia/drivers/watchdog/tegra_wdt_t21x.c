/*
 * Copyright (c) 2014-2020, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>
#include <linux/delay.h>

/* minimum and maximum watchdog trigger timeout, in seconds */
#define MIN_WDT_TIMEOUT			1
#define MAX_WDT_TIMEOUT			255

/* WDT registers */
#define WDT_CFG				0x0
#define WDT_CFG_PERIOD_SHIFT		4
#define WDT_CFG_PERIOD_MASK		0xff
#define WDT_CFG_PMC2CAR_RST_EN		(1 << 15)
#define WDT_STS				0x4
#define WDT_STS_COUNT_SHIFT		4
#define WDT_STS_COUNT_MASK		0xff
#define WDT_STS_EXP_SHIFT		12
#define WDT_STS_EXP_MASK		0x3
#define WDT_STS_COUNTER_ACTIVE		0x01
#define WDT_CMD				0x8
#define WDT_CMD_START_COUNTER		(1 << 0)
#define WDT_CMD_DISABLE_COUNTER		(1 << 1)
#define WDT_UNLOCK			(0xc)
#define WDT_UNLOCK_PATTERN		(0xc45a << 0)

/* Timer registers */
#define TIMER_PTV			0x0
#define TIMER_EN			(1 << 31)
#define TIMER_PERIODIC			(1 << 30)

struct tegra_wdt {
	struct watchdog_device	wdd;
	struct device		*dev;
	void __iomem		*wdt_regs;
	void __iomem		*tmr_regs;
	u8			timer_id;
};

/*
 * The total expiry count of Tegra WDTs is limited to HW design and depends
 * on skip configuration if supported. To be safe, we set the default expiry
 * count to 1. It should be updated later with value specified in device tree.
 */
static int expiry_count = 1;

/*
 * Period value: Trigger Value is the time period and period value determines
 * number of periods for watchdog expiration i.e. effectively after time:
 * "period value" * "Trigger Vaue"
 */
#define WDT_TRG_PERIOD	1
static int trigger_period = WDT_TRG_PERIOD;

#define WDT_HEARTBEAT 120
static int heartbeat = WDT_HEARTBEAT;
module_param(heartbeat, int, 0);
MODULE_PARM_DESC(heartbeat,
	"Watchdog heartbeats in seconds. (default = "
	__MODULE_STRING(WDT_HEARTBEAT) ")");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout,
	"Watchdog cannot be stopped once started (default="
	__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

/*
 * Timer ID is required to be programmed to WDT_CFG register
 * As timer address and timer ID mapping is non-linear,
 * hence these function are defined to do the conversion.
 */
static resource_size_t tegra_tmr_addr(int timer_id, struct resource *tmr_res)
{
	int i;
	resource_size_t tmr_addr;

	if (timer_id == 0)
		i = 10;
	else if (timer_id < 3)
		i = timer_id - 8;
	else if (timer_id > 9)
		i = timer_id + 1;
	else
		i = timer_id;

	tmr_addr = tmr_res->start - (10 - i) * resource_size(tmr_res);

	return tmr_addr;
}

static int tegra_tmr_index(struct resource *tmr_res)
{
	int timer_id;

	/* Select Timer ID using Timer base address provided in DT:
	 *   [Base Addr:Timer#]
	 *   [0x00 :  1], [0x08 :  2], [0x50 :  3], [0x58 : 4], [0x60 : 5],
	 *   [0x68 :  6], [0x70 :  7], [0x78 :  8], [0x80 : 9], [0x88 : 0],
	 *   [0x90 : 10], [0x98 : 11], [0xA0 : 12], [0xA8 : 13]
	 */
	timer_id = ((3 + ((tmr_res->start & 0xff) - 0x50) / 8)) % 10;
	if ((tmr_res->start & 0xff) < 0x50)
		timer_id -= 4;
	if ((tmr_res->start & 0xff) > 0x88)
		timer_id += 9;

	return timer_id;
}

static int tegra_wdt_start(struct watchdog_device *wdd)
{
	struct tegra_wdt *wdt = watchdog_get_drvdata(wdd);
	u32 val;

	if (readl(wdt->wdt_regs + WDT_STS) & WDT_STS_COUNTER_ACTIVE) {
		dev_info(wdt->dev, "tegra watchdog is already running\n");
		return 0;
	}

	/*
	 * The timeout needs to be divided by expiry_count here so as to
	 * keep the ultimate watchdog reset timeout the same as the program
	 * timeout requested by application.
	 */
	val = (wdd->timeout * USEC_PER_SEC) / expiry_count;
	val |= (TIMER_EN | TIMER_PERIODIC);
	writel(val, wdt->tmr_regs + TIMER_PTV);

	/*
	 * Set number of periods and start counter.
	 */
	val = (wdt->timer_id) |
	      (trigger_period << WDT_CFG_PERIOD_SHIFT) |
		WDT_CFG_PMC2CAR_RST_EN;
	writel(val, wdt->wdt_regs + WDT_CFG);

	writel(WDT_CMD_START_COUNTER, wdt->wdt_regs + WDT_CMD);

	return 0;


}

static int tegra_wdt_stop(struct watchdog_device *wdd)
{
	struct tegra_wdt *wdt = watchdog_get_drvdata(wdd);

	writel(WDT_UNLOCK_PATTERN, wdt->wdt_regs + WDT_UNLOCK);
	writel(WDT_CMD_DISABLE_COUNTER, wdt->wdt_regs + WDT_CMD);
	writel(0, wdt->tmr_regs + TIMER_PTV);

	return 0;
}

static int tegra_wdt_ping(struct watchdog_device *wdd)
{
	struct tegra_wdt *wdt = watchdog_get_drvdata(wdd);

	if (readl(wdt->wdt_regs + WDT_STS) & WDT_STS_COUNTER_ACTIVE)
		writel(WDT_CMD_START_COUNTER, wdt->wdt_regs + WDT_CMD);

	return 0;
}

static int tegra_wdt_set_timeout(struct watchdog_device *wdd,
				 unsigned int timeout)
{
	struct tegra_wdt *wdt = watchdog_get_drvdata(wdd);

	wdd->timeout = timeout;

	if (readl(wdt->wdt_regs + WDT_STS) & WDT_STS_COUNTER_ACTIVE) {
		tegra_wdt_stop(wdd);
		/* Sleep to give time for stop */
		usleep_range(10, 100);
	}

	tegra_wdt_start(wdd);

	dev_info(wdt->dev, "tegra wdt timeout %u is set\n", timeout);

	return 0;
}

static unsigned int tegra_wdt_get_timeleft(struct watchdog_device *wdd)
{
	struct tegra_wdt *wdt = watchdog_get_drvdata(wdd);
	u32 val;
	int count;
	int exp;

	val = readl(wdt->wdt_regs + WDT_STS);

	/* Current countdown (from timeout) */
	count = (val >> WDT_STS_COUNT_SHIFT) & WDT_STS_COUNT_MASK;

	/* Number of expirations */
	exp = (val >> WDT_STS_EXP_SHIFT) & WDT_STS_EXP_MASK;
	/*
	 * The entire thing is divided by expiry_count because we are ticking
	 * down expiry_count times  faster due to needing to wait for the
	 * expiry_count'th expiration.
	 */
	return (((3 - exp) * wdd->timeout) + count) / expiry_count;
}

unsigned int tegra_wdt_status(struct watchdog_device *wdd)
{
	struct tegra_wdt *wdt = watchdog_get_drvdata(wdd);

	return readl(wdt->wdt_regs + WDT_STS);
}

static const struct watchdog_info tegra_wdt_info = {
	.options	= WDIOF_SETTIMEOUT |
			  WDIOF_MAGICCLOSE |
			  WDIOF_KEEPALIVEPING,
	.firmware_version = 0,
	.identity	= "Tegra Watchdog",
};

static struct watchdog_ops tegra_wdt_ops = {
	.owner = THIS_MODULE,
	.start = tegra_wdt_start,
	.stop = tegra_wdt_stop,
	.status = tegra_wdt_status,
	.ping = tegra_wdt_ping,
	.set_timeout = tegra_wdt_set_timeout,
	.get_timeleft = tegra_wdt_get_timeleft,
};

static int tegra_wdt_probe(struct platform_device *pdev)
{
	struct watchdog_device *wdd;
	struct tegra_wdt *wdt;
	struct resource *wdt_res, *tmr_res;
	struct device_node *np = pdev->dev.of_node;
	resource_size_t tmr_addr;
	u32 pval = 0;
	int ret = 0;

	wdt_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	tmr_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);

	if (!wdt_res || !tmr_res) {
		dev_err(&pdev->dev, "incorrect wdt resources\n");
		return -ENOENT;
	}

	ret = of_property_read_u32(np, "nvidia,expiry-count", &pval);
	if (!ret)
		expiry_count = pval;

	ret = of_property_read_u32(np, "nvidia,heartbeat-init", &pval);
	if (!ret)
		heartbeat = pval;

	/*
	 * Allocate our watchdog driver data, which has the
	 * struct watchdog_device nested within it.
	 */
	wdt = devm_kzalloc(&pdev->dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->dev = &pdev->dev;

	/*
	 * Get Timer index from (in decending priority):
	 *      from "nvidia,timer-index" DT property
	 *      from Timer Source Address in DT
	 */
	wdt->timer_id = tegra_tmr_index(tmr_res);

	ret = of_property_read_u32(np, "nvidia,timer-index", &pval);
	if (!ret) {
		/*
		 * If timer-index is provided then either corresponding
		 * timer source address or timer base address should be
		 * provided.
		 */
		if (wdt->timer_id && (wdt->timer_id != pval)) {
			dev_err(&pdev->dev, "Invalid Timer base address\n");
			return -EINVAL;
		}

		/* Skip adjust resource if timer source address is provided */
		if (!wdt->timer_id) {
			/* Timer address corresponding to timer ID */
			tmr_addr = tegra_tmr_addr(pval, tmr_res);
			ret = adjust_resource(tmr_res, tmr_addr,
					resource_size(tmr_res));
			if (ret < 0) {
				dev_err(&pdev->dev,
					"Failed to adjust resource:%d\n", ret);
				return ret;
			}
		}
		wdt->timer_id = pval;
	}

	wdt->wdt_regs = devm_ioremap_resource(&pdev->dev, wdt_res);
	if (IS_ERR(wdt->wdt_regs)) {
		dev_err(&pdev->dev, "failed ioremap wdt resource\n");
		return PTR_ERR(wdt->wdt_regs);
	}

	wdt->tmr_regs = devm_ioremap_resource(&pdev->dev, tmr_res);
	if (IS_ERR(wdt->tmr_regs)) {
		dev_err(&pdev->dev, "failed ioremap tmr resource\n");
		return PTR_ERR(wdt->tmr_regs);
	}

	/* Initialize struct watchdog_device. */
	wdd = &wdt->wdd;
	wdd->timeout = heartbeat;
	wdd->info = &tegra_wdt_info;
	wdd->ops = &tegra_wdt_ops;
	wdd->min_timeout = MIN_WDT_TIMEOUT * expiry_count;
	wdd->max_timeout = MAX_WDT_TIMEOUT * expiry_count;
	wdd->parent = &pdev->dev;

	watchdog_set_drvdata(wdd, wdt);

	if (readl(wdt->wdt_regs + WDT_STS) & WDT_STS_COUNTER_ACTIVE) {
		dev_info(&pdev->dev, "Tegra WDT is already runnig\n");
		set_bit(WDOG_HW_RUNNING, &wdd->status);
	} else if (of_property_read_bool(np, "nvidia,enable-on-init")) {
		/* start watchdog when "enable-on-init" flag is set */
		tegra_wdt_start(wdd);
		set_bit(WDOG_HW_RUNNING, &wdd->status);
	}

	watchdog_set_nowayout(wdd, nowayout);

	ret = watchdog_register_device(wdd);
	if (ret) {
		dev_err(&pdev->dev,
			"failed to register watchdog device\n");
		return ret;
	}

	platform_set_drvdata(pdev, wdt);

	dev_info(&pdev->dev,
		 "initialized (timeout = %d sec, nowayout = %d)\n",
		 wdt->wdd.timeout, nowayout);

	return 0;
}

static int tegra_wdt_remove(struct platform_device *pdev)
{
	struct tegra_wdt *wdt = platform_get_drvdata(pdev);

	tegra_wdt_stop(&wdt->wdd);

	watchdog_unregister_device(&wdt->wdd);

	dev_info(&pdev->dev, "removed wdt\n");

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tegra_wdt_runtime_suspend(struct device *dev)
{
	struct tegra_wdt *wdt = dev_get_drvdata(dev);

	if (watchdog_active(&wdt->wdd) ||
		test_bit(WDOG_HW_RUNNING, &wdt->wdd.status))
		tegra_wdt_stop(&wdt->wdd);

	return 0;
}

static int tegra_wdt_runtime_resume(struct device *dev)
{
	struct tegra_wdt *wdt = dev_get_drvdata(dev);

	if (watchdog_active(&wdt->wdd) ||
		test_bit(WDOG_HW_RUNNING, &wdt->wdd.status))
		tegra_wdt_start(&wdt->wdd);

	return 0;
}
#endif

static const struct of_device_id tegra_wdt_of_match[] = {
	{ .compatible = "nvidia,tegra-wdt-t21x", },
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_wdt_of_match);

static const struct dev_pm_ops tegra_wdt_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(tegra_wdt_runtime_suspend,
				tegra_wdt_runtime_resume)
};

static struct platform_driver tegra_wdt_driver = {
	.probe		= tegra_wdt_probe,
	.remove		= tegra_wdt_remove,
	.driver		= {
		.name	= "tegra-wdt",
		.pm	= &tegra_wdt_pm_ops,
		.of_match_table = tegra_wdt_of_match,
	},
};
module_platform_driver(tegra_wdt_driver);

MODULE_AUTHOR("NVIDIA Corporation");
MODULE_DESCRIPTION("Tegra Watchdog Driver");
MODULE_LICENSE("GPL v2");
