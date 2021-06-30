/*
 * mods_clock.c - This file is part of NVIDIA MODS kernel driver.
 *
 * Copyright (c) 2011-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA MODS kernel driver is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * NVIDIA MODS kernel driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with NVIDIA MODS kernel driver.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "mods_internal.h"
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <linux/reset.h>

#define ARBITRARY_MAX_CLK_FREQ  3500000000

static struct list_head mods_clock_handles;
static spinlock_t mods_clock_lock;
static u32 last_handle;

struct clock_entry {
	struct clk *pclk;
	u32 handle;
	struct list_head list;
};

static struct device_node *find_clocks_node(const char *name)
{
	const char *node_name = "mods-simple-bus";
	struct device_node *pp = NULL, *np = NULL;

	pp = of_find_node_by_name(NULL, node_name);

	if (!pp) {
		mods_error_printk("'mods-simple-bus' node not found in device tree\n");
		return pp;
	}

	np = of_get_child_by_name(pp, name);
	return np;
}

void mods_init_clock_api(void)
{
	const char *okay_value = "okay";
	struct device_node *mods_np = 0;
	struct property *pp = 0;
	int size_value = 0;

	mods_np = find_clocks_node("mods-clocks");
	if (!mods_np) {
		mods_error_printk("'mods-clocks' node not found in device tree\n");
		goto err;
	}

	pp = of_find_property(mods_np, "status", NULL);
	if (IS_ERR(pp)) {
		mods_error_printk("'status' prop not found in 'mods-clocks' node.");
		goto err;
	}

	/* if status is 'okay', then skip updating property */
	if (of_device_is_available(mods_np))
		goto err;

	size_value = strlen(okay_value) + 1;
	pp->value = kmalloc(size_value, GFP_KERNEL);
	strncpy(pp->value, okay_value, size_value);
	pp->length = size_value;

err:
	of_node_put(mods_np);

	spin_lock_init(&mods_clock_lock);
	INIT_LIST_HEAD(&mods_clock_handles);
	last_handle = 0;
}

void mods_shutdown_clock_api(void)
{
	struct list_head *head = &mods_clock_handles;
	struct list_head *iter;
	struct list_head *tmp;

	spin_lock(&mods_clock_lock);

	list_for_each_safe(iter, tmp, head) {
		struct clock_entry *entry
			= list_entry(iter, struct clock_entry, list);
		list_del(iter);
		kfree(entry);
	}

	spin_unlock(&mods_clock_lock);
}

static u32 mods_get_clock_handle(struct clk *pclk)
{
	struct list_head *head = &mods_clock_handles;
	struct list_head *iter;
	struct clock_entry *entry = 0;
	u32 handle = 0;

	spin_lock(&mods_clock_lock);

	list_for_each(iter, head) {
		struct clock_entry *cur
			= list_entry(iter, struct clock_entry, list);
		if (cur->pclk == pclk) {
			entry = cur;
			handle = cur->handle;
			break;
		}
	}

	if (!entry) {
		entry = kmalloc(sizeof(*entry), GFP_ATOMIC);
		if (!unlikely(!entry)) {
			entry->pclk = pclk;
			entry->handle = ++last_handle;
			handle = entry->handle;
			list_add(&entry->list, &mods_clock_handles);
		}
	}

	spin_unlock(&mods_clock_lock);

	return handle;
}

static struct clk *mods_get_clock(u32 handle)
{
	struct list_head *head = &mods_clock_handles;
	struct list_head *iter;
	struct clk *pclk = 0;

	spin_lock(&mods_clock_lock);

	list_for_each(iter, head) {
		struct clock_entry *entry
			= list_entry(iter, struct clock_entry, list);
		if (entry->handle == handle) {
			pclk = entry->pclk;
			break;
		}
	}

	spin_unlock(&mods_clock_lock);

	return pclk;
}

int esc_mods_get_clock_handle(struct file *pfile,
			      struct MODS_GET_CLOCK_HANDLE *p)
{
	struct clk *pclk = 0;
	int ret = -EINVAL;

	struct device_node *mods_np = 0;
	struct property *pp = 0;

	LOG_ENT();

	mods_np = find_clocks_node("mods-clocks");
	if (!mods_np || !of_device_is_available(mods_np)) {
		mods_error_printk("'mods-clocks' node not found in device tree\n");
		goto err;
	}
	pp = of_find_property(mods_np, "clock-names", NULL);
	if (IS_ERR(pp)) {
		mods_error_printk(
		    "No 'clock-names' prop in 'mods-clocks' node for dev %s\n",
				  p->controller_name);
		goto err;
	}

	pclk = of_clk_get_by_name(mods_np, p->controller_name);

	if (IS_ERR(pclk))
		mods_error_printk("clk (%s) not found\n", p->controller_name);
	else {
		p->clock_handle = mods_get_clock_handle(pclk);
		ret = OK;
	}
err:
	of_node_put(mods_np);
	LOG_EXT();
	return ret;
}

int esc_mods_set_clock_rate(struct file *pfile, struct MODS_CLOCK_RATE *p)
{
	struct clk *pclk = 0;
	int ret = -EINVAL;

	LOG_ENT();

	pclk = mods_get_clock(p->clock_handle);

	if (!pclk) {
		mods_error_printk("unrecognized clock handle: 0x%x\n",
				  p->clock_handle);
	} else {
		ret = clk_set_rate(pclk, p->clock_rate_hz);
		if (ret) {
			mods_error_printk(
				"unable to set rate %lluHz on clock 0x%x\n",
				p->clock_rate_hz, p->clock_handle);
		} else {
			mods_debug_printk(DEBUG_CLOCK,
				  "successfuly set rate %lluHz on clock 0x%x\n",
				  p->clock_rate_hz, p->clock_handle);
		}
	}

	LOG_EXT();
	return ret;
}

int esc_mods_get_clock_rate(struct file *pfile, struct MODS_CLOCK_RATE *p)
{
	struct clk *pclk = 0;
	int ret = -EINVAL;

	LOG_ENT();

	pclk = mods_get_clock(p->clock_handle);

	if (!pclk) {
		mods_error_printk("unrecognized clock handle: 0x%x\n",
				  p->clock_handle);
	} else {
		p->clock_rate_hz = clk_get_rate(pclk);
		mods_debug_printk(DEBUG_CLOCK, "clock 0x%x has rate %lluHz\n",
				  p->clock_handle, p->clock_rate_hz);
		ret = OK;
	}

	LOG_EXT();
	return ret;
}

int esc_mods_get_clock_max_rate(struct file *pfile, struct MODS_CLOCK_RATE *p)
{
	struct clk *pclk = 0;
	int ret = -EINVAL;

	LOG_ENT();
	pclk = mods_get_clock(p->clock_handle);

	if (!pclk) {
		mods_error_printk("unrecognized clock handle: 0x%x\n",
				  p->clock_handle);
	} else {
		long rate = clk_round_rate(pclk, ARBITRARY_MAX_CLK_FREQ);

		p->clock_rate_hz = rate < 0 ? ARBITRARY_MAX_CLK_FREQ
			: (unsigned long)rate;
		mods_debug_printk(DEBUG_CLOCK,
				  "clock 0x%x has max rate %lluHz\n",
				  p->clock_handle, p->clock_rate_hz);
		ret = OK;
	}

	LOG_EXT();
	return ret;
}

int esc_mods_set_clock_max_rate(struct file *pfile, struct MODS_CLOCK_RATE *p)
{
	struct clk *pclk = 0;
	int ret = -EINVAL;

	LOG_ENT();

	pclk = mods_get_clock(p->clock_handle);

	if (!pclk) {
		mods_error_printk("unrecognized clock handle: 0x%x\n",
				  p->clock_handle);
	} else {
#if defined(CONFIG_TEGRA_CLOCK_DEBUG_FUNC)
		ret = tegra_clk_set_max(pclk, p->clock_rate_hz);
		if (ret) {
			mods_error_printk(
		"unable to override max clock rate %lluHz on clock 0x%x\n",
					  p->clock_rate_hz, p->clock_handle);
		} else {
			mods_debug_printk(DEBUG_CLOCK,
			  "successfuly set max rate %lluHz on clock 0x%x\n",
					  p->clock_rate_hz, p->clock_handle);
		}
#else
		mods_error_printk("unable to override max clock rate\n");
		mods_error_printk(
		"reconfigure kernel with CONFIG_TEGRA_CLOCK_DEBUG_FUNC=y\n");
		ret = -EINVAL;
#endif
	}

	LOG_EXT();
	return ret;
}

int esc_mods_set_clock_parent(struct file *pfile, struct MODS_CLOCK_PARENT *p)
{
	struct clk *pclk = 0;
	struct clk *pparent = 0;
	int ret = -EINVAL;

	LOG_ENT();

	pclk = mods_get_clock(p->clock_handle);
	pparent = mods_get_clock(p->clock_parent_handle);

	if (!pclk) {
		mods_error_printk("unrecognized clock handle: 0x%x\n",
				  p->clock_handle);
	} else if (!pparent) {
		mods_error_printk("unrecognized parent clock handle: 0x%x\n",
				  p->clock_parent_handle);
	} else {
		ret = clk_set_parent(pclk, pparent);
		if (ret) {
			mods_error_printk(
			    "unable to make clock 0x%x parent of clock 0x%x\n",
			    p->clock_parent_handle, p->clock_handle);
		} else {
			mods_debug_printk(DEBUG_CLOCK,
			  "successfuly made clock 0x%x parent of clock 0x%x\n",
			  p->clock_parent_handle, p->clock_handle);
		}
	}

	LOG_EXT();
	return ret;
}

int esc_mods_get_clock_parent(struct file *pfile, struct MODS_CLOCK_PARENT *p)
{
	struct clk *pclk = 0;
	int ret = -EINVAL;

	LOG_ENT();

	pclk = mods_get_clock(p->clock_handle);

	if (!pclk) {
		mods_error_printk("unrecognized clock handle: 0x%x\n",
				  p->clock_handle);
	} else {
		struct clk *pparent = clk_get_parent(pclk);

		p->clock_parent_handle = mods_get_clock_handle(pparent);
		mods_debug_printk(DEBUG_CLOCK,
				  "clock 0x%x is parent of clock 0x%x\n",
				  p->clock_parent_handle, p->clock_handle);
		ret = OK;
	}

	LOG_EXT();
	return ret;
}

int esc_mods_enable_clock(struct file *pfile, struct MODS_CLOCK_HANDLE *p)
{
	struct clk *pclk = 0;
	int ret = -EINVAL;

	LOG_ENT();

	pclk = mods_get_clock(p->clock_handle);

	if (!pclk) {
		mods_error_printk("unrecognized clock handle: 0x%x\n",
				  p->clock_handle);
	} else {
		ret = clk_prepare(pclk);
		if (ret) {
			mods_error_printk(
			    "unable to prepare clock 0x%x before enabling\n",
					  p->clock_handle);
		}
		ret = clk_enable(pclk);
		if (ret) {
			mods_error_printk("failed to enable clock 0x%x\n",
					  p->clock_handle);
		} else {
			mods_debug_printk(DEBUG_CLOCK, "clock 0x%x enabled\n",
					  p->clock_handle);
		}
	}

	LOG_EXT();
	return ret;
}

int esc_mods_disable_clock(struct file *pfile, struct MODS_CLOCK_HANDLE *p)
{
	struct clk *pclk = 0;
	int ret = -EINVAL;

	LOG_ENT();

	pclk = mods_get_clock(p->clock_handle);

	if (!pclk) {
		mods_error_printk("unrecognized clock handle: 0x%x\n",
				  p->clock_handle);
	} else {
		clk_disable(pclk);
		clk_unprepare(pclk);
		mods_debug_printk(DEBUG_CLOCK, "clock 0x%x disabled\n",
				  p->clock_handle);
		ret = OK;
	}

	LOG_EXT();
	return ret;
}

int esc_mods_is_clock_enabled(struct file *pfile, struct MODS_CLOCK_ENABLED *p)
{
	struct clk *pclk = 0;
	int ret = -EINVAL;

	LOG_ENT();

	pclk = mods_get_clock(p->clock_handle);

	if (!pclk) {
		mods_error_printk("unrecognized clock handle: 0x%x\n",
				  p->clock_handle);
	} else {
		p->enable_count = (u32)__clk_is_enabled(pclk);
		mods_debug_printk(DEBUG_CLOCK, "clock 0x%x enable count is %u\n",
				  p->clock_handle, p->enable_count);
		ret = OK;
	}

	LOG_EXT();
	return ret;
}

int esc_mods_clock_reset_assert(struct file *pfile,
				struct MODS_CLOCK_HANDLE *p)
{
	struct clk *pclk = 0;
	int ret = -EINVAL;

	LOG_ENT();

	pclk = mods_get_clock(p->clock_handle);

	if (!pclk) {
		mods_error_printk("unrecognized clock handle: 0x%x\n",
				  p->clock_handle);
	} else {
		const char *clk_name = 0;
		struct reset_control *prst = 0;
		struct device_node *mods_np = 0;
		struct property *pp = 0;

		mods_np = find_clocks_node("mods-clocks");
		if (!mods_np || !of_device_is_available(mods_np)) {
			mods_error_printk("'mods-clocks' node not found in DTB\n");
			goto err;
		}
		pp = of_find_property(mods_np, "reset-names", NULL);
		if (IS_ERR(pp)) {
			mods_error_printk(
		    "No 'reset-names' prop in 'mods-clocks' node for dev %s\n",
					  __clk_get_name(pclk));
			goto err;
		}

		clk_name = __clk_get_name(pclk);

		prst = of_reset_control_get(mods_np, clk_name);
		if (IS_ERR(prst)) {
			mods_error_printk("reset device %s not found\n",
					  clk_name);
			goto err;
		}
		ret = reset_control_assert(prst);
		if (ret) {
			mods_error_printk("failed to assert reset on '%s'\n",
					  clk_name);
		} else {
			mods_debug_printk(DEBUG_CLOCK, "asserted reset on '%s'",
					  clk_name);
		}

err:
		of_node_put(mods_np);
	}
	LOG_EXT();
	return ret;
}

int esc_mods_clock_reset_deassert(struct file *pfile,
				  struct MODS_CLOCK_HANDLE *p)
{
	struct clk *pclk = 0;
	int ret = -EINVAL;

	LOG_ENT();

	pclk = mods_get_clock(p->clock_handle);

	if (!pclk) {
		mods_error_printk("unrecognized clock handle: 0x%x\n",
				  p->clock_handle);
	} else {
		const char *clk_name = 0;
		struct reset_control *prst = 0;
		struct device_node *mods_np = 0;
		struct property *pp = 0;

		mods_np = find_clocks_node("mods-clocks");
		if (!mods_np || !of_device_is_available(mods_np)) {
			mods_error_printk("'mods-clocks' node not found in DTB\n");
			goto err;
		}
		pp = of_find_property(mods_np, "reset-names", NULL);
		if (IS_ERR(pp)) {
			mods_error_printk(
		    "No 'reset-names' prop in 'mods-clocks' node for dev %s\n",
					  __clk_get_name(pclk));
			goto err;
		}

		clk_name = __clk_get_name(pclk);

		prst = of_reset_control_get(mods_np, clk_name);
		if (IS_ERR(prst)) {
			mods_error_printk(
				"reset device %s not found\n", clk_name);
			goto err;
		}
		ret = reset_control_deassert(prst);
		if (ret) {
			mods_error_printk("failed to assert reset on '%s'\n",
					  clk_name);
		} else {
			mods_debug_printk(DEBUG_CLOCK, "deasserted reset on '%s'",
					  clk_name);
		}

err:
		of_node_put(mods_np);
	}

	LOG_EXT();
	return ret;
}
