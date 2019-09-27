/*
 * Copyright (c) 2014-2018, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include <linux/export.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/version.h>
#include <soc/tegra/tegra_bpmp.h>
#include <soc/tegra/bpmp_abi.h>

#include "clk-tegra-bpmp.h"
#include "clk-mrq.h"

/**
 * struct tegra_bpmp_clk
 *
 * @hw:		handle between common and hardware-specific interfaces
 * @clk_num:	bpmp clk identifier
 */
struct tegra_clk_bpmp {
	struct clk_hw	hw;
	int		clk_num;
	int		num_parents;
	int		parent;
	int		parent_ids[0];
};

#define to_clk_bpmp(_hw) container_of(_hw, struct tegra_clk_bpmp, hw)

/**
 * Mutex to prevent concurrent invocations
 * to register a clock.
 */
static DEFINE_MUTEX(clk_reg_lock);

struct bpmp_clk_req {
	u32	cmd;
	u8	args[0];
};

#define BPMP_CLK_CMD(cmd, id) ((id) | ((cmd) << 24))

struct possible_parents {
	u8	num_of_parents;
	s32	clk_ids[MRQ_CLK_MAX_PARENTS];
};

static const char **clk_names;

struct clk_data {
	struct clk_onecell_data cell;
	int staged;
};

static struct clk_data clk_data;

static int bpmp_send_clk_message_atomic(struct bpmp_clk_req *req, int size,
				 u8 *reply, int reply_size)
{
	unsigned long flags;
	int err;

	local_irq_save(flags);
	err = tegra_bpmp_send_receive_atomic(MRQ_CLK, req, size, reply,
			reply_size);
	local_irq_restore(flags);

	return err;
}

static int bpmp_send_clk_message(struct bpmp_clk_req *req, int size,
				 u8 *reply, int reply_size)
{
	int err;

	err = tegra_bpmp_send_receive(MRQ_CLK, req, size, reply, reply_size);
	if (err != -EAGAIN)
		return err;

	/*
	 * in case the mail systems worker threads haven't been started yet,
	 * use the atomic send/receive interface. This happens because the
	 * clocks are initialized before the IPC mechanism.
	 */
	return bpmp_send_clk_message_atomic(req, size, reply, reply_size);
}

static int clk_bpmp_enable(struct clk_hw *hw)
{
	struct tegra_clk_bpmp *bpmp_clk = to_clk_bpmp(hw);
	struct bpmp_clk_req req;

	req.cmd = BPMP_CLK_CMD(MRQ_CLK_ENABLE, bpmp_clk->clk_num);

	return bpmp_send_clk_message(&req, sizeof(req), NULL, 0);
}

static void clk_bpmp_disable(struct clk_hw *hw)
{
	struct tegra_clk_bpmp *bpmp_clk = to_clk_bpmp(hw);
	struct bpmp_clk_req req;

	req.cmd = BPMP_CLK_CMD(MRQ_CLK_DISABLE, bpmp_clk->clk_num);

	bpmp_send_clk_message(&req, sizeof(req), NULL, 0);
}

static int clk_bpmp_is_enabled(struct clk_hw *hw)
{
	struct tegra_clk_bpmp *bpmp_clk = to_clk_bpmp(hw);
	struct bpmp_clk_req req;
	int err;
	u8 reply[4];

	req.cmd = BPMP_CLK_CMD(MRQ_CLK_IS_ENABLED, bpmp_clk->clk_num);

	err = bpmp_send_clk_message_atomic(&req, sizeof(req),
			reply, sizeof(reply));
	if (err < 0)
		return err;

	return ((s32 *)reply)[0];
}

static u8 clk_bpmp_get_parent(struct clk_hw *hw)
{
	struct tegra_clk_bpmp *bpmp_clk = to_clk_bpmp(hw);
	int parent_id, i;

	parent_id = bpmp_clk->parent;

	if (parent_id < 0)
		goto err_out;

	for (i = 0; i < bpmp_clk->num_parents; i++) {
		if (bpmp_clk->parent_ids[i] == parent_id)
			return i;
	}

err_out:
	pr_err("clk_bpmp_get_parent for %s parent_id: %d, num_parents: %d\n",
		__clk_get_name(hw->clk), parent_id, bpmp_clk->num_parents);
	WARN_ON(1);

	return 0;
}

static int clk_bpmp_set_parent(struct clk_hw *hw, u8 index)
{
	struct tegra_clk_bpmp *bpmp_clk = to_clk_bpmp(hw);
	u8 req_d[12], reply[4];
	struct bpmp_clk_req *req  = (struct bpmp_clk_req *)&req_d[0];
	int err;

	if (index > bpmp_clk->num_parents - 1)
		return -EINVAL;

	req->cmd = BPMP_CLK_CMD(MRQ_CLK_SET_PARENT, bpmp_clk->clk_num);
	*((u32 *)&req->args[0]) = bpmp_clk->parent_ids[index];

	err = bpmp_send_clk_message(req, sizeof(req_d), reply, sizeof(reply));
	if (!err)
		bpmp_clk->parent = bpmp_clk->parent_ids[index];

	return err;
}

static int clk_bpmp_set_rate(struct clk_hw *hw, unsigned long rate,
			      unsigned long parent_rate)
{
	struct tegra_clk_bpmp *bpmp_clk = to_clk_bpmp(hw);
	u8 req_d[16], reply[8];
	struct bpmp_clk_req *req = (struct bpmp_clk_req *)&req_d[0];

	pr_debug("%s: %s(%d): %lu\n",
		 __func__, clk_hw_get_name(hw), bpmp_clk->clk_num, rate);

	req->cmd = BPMP_CLK_CMD(MRQ_CLK_SET_RATE, bpmp_clk->clk_num);
	if (rate > S64_MAX)
		rate = S64_MAX;

	*((s64 *)&req->args[4]) = rate;

	return bpmp_send_clk_message(req, sizeof(req_d), reply, sizeof(reply));
}

static int clk_bpmp_determine_rate(struct clk_hw *hw,
				    struct clk_rate_request *rate_req)
{
	struct tegra_clk_bpmp *bpmp_clk = to_clk_bpmp(hw);
	int err;
	s64 reply_val;
	unsigned long rate;
	u8 req_d[16], reply[8];
	struct bpmp_clk_req *req = (struct bpmp_clk_req *)&req_d[0];

	rate = min(max(rate_req->rate, rate_req->min_rate), rate_req->max_rate);

	req->cmd = BPMP_CLK_CMD(MRQ_CLK_ROUND_RATE, bpmp_clk->clk_num);
	if (rate > S64_MAX)
		rate = S64_MAX;

	*((s64 *)&req->args[4]) = rate;
	err = bpmp_send_clk_message(req, sizeof(req_d), reply, sizeof(reply));

	if (err < 0)
		return err;

	reply_val = ((s64 *)reply)[0];

	if (reply_val < 0)
		return (int)reply_val;

	rate_req->rate = (unsigned long)reply_val;

	return 0;
}

static unsigned long clk_bpmp_get_rate_clk_num(int clk_num)
{
	u8 reply[8];
	struct bpmp_clk_req req;
	int err;

	req.cmd = BPMP_CLK_CMD(MRQ_CLK_GET_RATE, clk_num);
	err = bpmp_send_clk_message(&req, sizeof(req), reply, sizeof(reply));
	if (err < 0)
		return err;

	return ((s64 *)reply)[0];
}

static unsigned long clk_bpmp_get_rate(struct clk_hw *hw,
				       unsigned long parent_rate)
{
	struct tegra_clk_bpmp *bpmp_clk = to_clk_bpmp(hw);

	return clk_bpmp_get_rate_clk_num(bpmp_clk->clk_num);
}

static int clk_bpmp_get_max_clk_id(u32 *max_id)
{
	struct bpmp_clk_req req;
	u8 reply[4];
	int err;

	req.cmd = BPMP_CLK_CMD(CMD_CLK_GET_MAX_CLK_ID, 0);
	err = bpmp_send_clk_message(&req, sizeof(req), reply, sizeof(reply));
	if (err < 0)
		return err;

	*max_id = ((u32 *)reply)[0];
	return 0;
}

static int clk_bpmp_get_all_info(int clk_num,
				 u32 *flags,
				 u32 *parent,
				 struct possible_parents *parents,
				 char *name)
{
	struct bpmp_clk_req req;
	struct cmd_clk_get_all_info_response resp;
	int i, err;

	req.cmd = BPMP_CLK_CMD(CMD_CLK_GET_ALL_INFO, clk_num);
	err = bpmp_send_clk_message((void *)&req, sizeof(req), (void *)&resp,
				    sizeof(resp));
	if (err < 0)
		return err;
	*flags = resp.flags;
	*parent = resp.parent;
	parents->num_of_parents = resp.num_parents;
	for (i = 0; i < resp.num_parents; ++i)
		parents->clk_ids[i] = resp.parents[i];
	strncpy(name, resp.name, MRQ_CLK_NAME_MAXLEN);
	name[MRQ_CLK_NAME_MAXLEN-1] = 0;

	return 0;
}

const struct clk_ops tegra_clk_bpmp_gate_ops = {
	.is_enabled = clk_bpmp_is_enabled,
	.prepare = clk_bpmp_enable,
	.unprepare = clk_bpmp_disable,
};

const struct clk_ops tegra_clk_bpmp_mux_rate_ops = {
	.is_enabled = clk_bpmp_is_enabled,
	.prepare = clk_bpmp_enable,
	.unprepare = clk_bpmp_disable,
	.get_parent = clk_bpmp_get_parent,
	.set_parent = clk_bpmp_set_parent,
	.set_rate = clk_bpmp_set_rate,
	.determine_rate = clk_bpmp_determine_rate,
	.recalc_rate = clk_bpmp_get_rate,
};

const struct clk_ops tegra_clk_bpmp_rate_ops = {
	.is_enabled = clk_bpmp_is_enabled,
	.prepare = clk_bpmp_enable,
	.unprepare = clk_bpmp_disable,
	.set_rate = clk_bpmp_set_rate,
	.determine_rate = clk_bpmp_determine_rate,
	.recalc_rate = clk_bpmp_get_rate,
};

const struct clk_ops  tegra_clk_bpmp_mux_ops = {
	.get_parent = clk_bpmp_get_parent,
	.set_parent = clk_bpmp_set_parent,
	.is_enabled = clk_bpmp_is_enabled,
	.prepare = clk_bpmp_enable,
	.unprepare = clk_bpmp_disable,
};

static struct clk *tegra_clk_register_bpmp(const char *name, int parent,
		const char **parent_names, int *parent_ids,
		uint32_t num_parents, int clk_num, uint32_t flags)
{
	const uint32_t mux_div_flags = BPMP_CLK_HAS_MUX | BPMP_CLK_HAS_SET_RATE;
	struct tegra_clk_bpmp *bpmp_clk;
	struct clk *clk;
	struct clk_init_data init;

	bpmp_clk = kzalloc(sizeof(*bpmp_clk) + num_parents * sizeof(int),
			GFP_KERNEL);
	if (!bpmp_clk) {
		pr_err("%s: unable to allocate clock %s\n", __func__, name);
		return ERR_PTR(-ENOMEM);
	}

	init.name = name;
	init.flags = 0;
	init.parent_names = parent_names;
	init.num_parents = num_parents;

	if ((flags & mux_div_flags) == mux_div_flags) {
		init.flags |= CLK_SET_RATE_NOCACHE;
		init.ops = &tegra_clk_bpmp_mux_rate_ops;
	} else if (flags & BPMP_CLK_HAS_SET_RATE) {
		init.flags |= CLK_SET_RATE_NOCACHE;
		init.ops = &tegra_clk_bpmp_rate_ops;
	}
	else if (flags & BPMP_CLK_HAS_MUX)
		init.ops = &tegra_clk_bpmp_mux_ops;
	else
		init.ops = &tegra_clk_bpmp_gate_ops;

	/* Data in .init is copied by clk_register(), so stack variable OK */
	bpmp_clk->clk_num = clk_num;
	bpmp_clk->hw.init = &init;
	bpmp_clk->num_parents = num_parents;
	bpmp_clk->parent = parent;

	if (num_parents > 1)
		memcpy(&bpmp_clk->parent_ids[0], parent_ids,
		       num_parents * sizeof(int));

	clk = clk_register(NULL, &bpmp_clk->hw);
	if (IS_ERR(clk)) {
		pr_err("registration failed for clock %s (%d)\n", name,
		       clk_num);
		kfree(bpmp_clk);
	}

	return clk;
}

static int clk_bpmp_init(uint32_t clk_num);

static int clk_init_parents(uint32_t clk_num, uint32_t flags,
		struct possible_parents *parents, const char **parent_names)
{
	uint32_t num_parents;
	uint32_t j;
	int p_id;
	int err;

	num_parents = parents->num_of_parents;

	if (num_parents > 1 && !(flags & BPMP_CLK_HAS_MUX)) {
		pr_err("clk-bpmp: inconsistent data from BPMP."
		       " Clock %d has more than one parent but no mux.\n",
			clk_num);
		return -EINVAL;
	}

	if (num_parents > 0 && (flags & BPMP_CLK_IS_ROOT)) {
		pr_err("clk-bpmp: inconsistent data from BPMP."
		       " Clock %d has parents but it's declared as root.\n",
			clk_num);
		return -EINVAL;
	}

	if (num_parents > MRQ_CLK_MAX_PARENTS) {
		pr_err("clk-bpmp: inconsistent data from BPMP."
		       " Clock %d has too many parents.\n",
		       clk_num);
		return -EINVAL;
	}

	for (j = 0; j < num_parents; j++) {
		p_id = parents->clk_ids[j];
		if (p_id < 0 || p_id >= clk_data.cell.clk_num) {
			pr_err("%s() bad clk num %d\n", __func__, p_id);
			parent_names[j] = "ERR!";
			continue;
		}

		err = clk_bpmp_init(p_id);
		if (err) {
			pr_err("clk-bpmp: unable to initialize clk %d\n",
			       p_id);
			parent_names[j] = "ERR!";
			continue;
		}

		if (IS_ERR_OR_NULL(clk_data.cell.clks[p_id])) {
			pr_err("clk-bpmp: clk %d not initialized."
			       " How did this happen?\n",
			       p_id);
			WARN_ON(1);
			parent_names[j] = "ERR!";
			continue;
		}

		parent_names[j] = clk_names[p_id];
	}

	return 0;
}

static int clk_bpmp_init(uint32_t clk_num)
{
	struct clk *clk;
	const char *parent_names[MRQ_CLK_MAX_PARENTS];
	struct possible_parents parents;
	u32 flags, parent;
	int err;
	char name[MRQ_CLK_NAME_MAXLEN];

	if (!IS_ERR_OR_NULL(clk_data.cell.clks[clk_num]))
		return 0;

	err = clk_bpmp_get_all_info(clk_num, &flags, &parent, &parents, name);
	if (!err)
		pr_debug("%s: %s: num = %u flags = %u parents = %u\n", __func__,
			 name, clk_num, flags, parents.num_of_parents);

	/**
	 * If the real clk is unavailable and if we are using the
	 * staged clk provider, allocate and use a dummy clk
	 */
	if (err && clk_data.staged) {
		clk = tegra_fclk_init(clk_num, name, sizeof(name));
		if (clk) {
			pr_warn("clock %d is dummy\n", clk_num);
			goto out;
		}
	}

	if (err)
		return err;

	err = clk_init_parents(clk_num, flags, &parents, parent_names);
	if (err)
		return err;

	if (flags & BPMP_CLK_IS_ROOT && !(flags & BPMP_CLK_HAS_SET_RATE)) {
		int64_t rate;
		rate = clk_bpmp_get_rate_clk_num(clk_num);
		clk = clk_register_fixed_rate(NULL, name, NULL, 0, rate);
	} else {
		clk = tegra_clk_register_bpmp(name, parent, parent_names,
				parents.clk_ids,
				parents.num_of_parents,
				clk_num,
				flags);
	}

	err = clk_register_clkdev(clk, name, "tegra-clk-debug");
	if (err)
		pr_err("clk_register_clkdev() returned %d for clk %s\n",
		       err, name);

out:
	clk_data.cell.clks[clk_num] = clk;

	clk_names[clk_num] = kstrdup(name, GFP_KERNEL);

	return 0;
}

static struct clk *tegra_of_clk_src_onecell_get(struct of_phandle_args *clkspec,
	void *data)
{
	struct clk_data *clk_data = data;
	uint32_t idx = clkspec->args[0];
	int err;

	if (idx >= clk_data->cell.clk_num) {
		pr_err("%s: invalid clock index %d\n", __func__, idx);
		return ERR_PTR(-EINVAL);
	}

	mutex_lock(&clk_reg_lock);

	if (!clk_data->cell.clks[idx]) {
		err = clk_bpmp_init(idx);
		if (err < 0) {
			pr_err("clk-bpmp: failed to initialize clk %d\n", idx);
			clk_data->cell.clks[idx] = ERR_PTR(-EINVAL);
		}
	}

	mutex_unlock(&clk_reg_lock);

	return clk_data->cell.clks[idx];
}

int tegra_bpmp_clk_init(struct device_node *np, int staged)
{
	struct clk **clks;
	int max_clk_id = 0;
	int r;

	pr_info("Registering BPMP clocks...\n");

	r = clk_bpmp_get_max_clk_id(&max_clk_id);
	if (r || max_clk_id < 0) {
		pr_err("failed to retrieve clk data (r %d, max_clk_id %d)\n",
				max_clk_id, r);
		return -ENODEV;
	}

	clks = kzalloc((max_clk_id + 1) * sizeof(struct clk *), GFP_KERNEL);
	if (!clks) {
		WARN_ON(1);
		return -ENOMEM;
	}

	clk_names = kzalloc((max_clk_id + 1) * sizeof(char *), GFP_KERNEL);
	if (!clk_names) {
		WARN_ON(1);
		kfree(clks);
		return -ENOMEM;
	}

	clk_data.staged = staged;
	clk_data.cell.clks = clks;
	clk_data.cell.clk_num = max_clk_id + 1;

	r = of_clk_add_provider(np, tegra_of_clk_src_onecell_get, &clk_data);

	pr_info("%s: clock init %s (%d clks)\n", __func__, r ? "failed" : "ok",
		max_clk_id + 1);

	return r;
}

#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include "clk.h"

static int fmon_clamp_read(void *data, u64 *val)
{
	struct mrq_fmon_request req;
	struct mrq_fmon_response rsp;
	int ret;
	struct clk_hw *hw = data;
	struct tegra_clk_bpmp *bpmp_clk = to_clk_bpmp(hw);

	req.cmd_and_id = BPMP_CLK_CMD(CMD_FMON_GEAR_GET, bpmp_clk->clk_num);

	ret = tegra_bpmp_send_receive(MRQ_FMON, &req, sizeof(req),
				      &rsp, sizeof(rsp));
	*val = ret < 0 ? ret : rsp.fmon_gear_get.rate;

	return 0;
}

static int fmov_clamp_write(void *data, u64 val)
{
	struct mrq_fmon_request req;
	struct clk_hw *hw = data;
	struct tegra_clk_bpmp *bpmp_clk = to_clk_bpmp(hw);

	if (val) {
		req.cmd_and_id = BPMP_CLK_CMD(CMD_FMON_GEAR_CLAMP,
					      bpmp_clk->clk_num);
		if (val > S64_MAX)
			val = S64_MAX;
		req.fmon_gear_clamp.rate = val;
	} else {
		req.cmd_and_id = BPMP_CLK_CMD(CMD_FMON_GEAR_FREE,
					      bpmp_clk->clk_num);
	}
	return tegra_bpmp_send_receive(MRQ_FMON, &req, sizeof(req), NULL, 0);
}
DEFINE_SIMPLE_ATTRIBUTE(fmon_clamp_fops, fmon_clamp_read, fmov_clamp_write,
			"%lld\n");

static void tegra_clk_bpmp_debugfs_add(struct clk *c)
{
	struct clk_hw *hw = __clk_get_hw(c);

	if (IS_ERR(clk_debugfs_add_file(hw, "fmon_clamp_rate",
					0644, hw, &fmon_clamp_fops)))
		pr_err("debugfs fmon_clamp failed for %s\n", __clk_get_name(c));
}

static int clk_init_set(void *data, u64 val)
{
	int i;
	static struct clk *c;
	struct of_phandle_args clkspec;
	struct clk_data *clk_data = data;

	if (!val)
		return 0;

	for (i = 0; i < clk_data->cell.clk_num; i++) {
		clkspec.args[0] = i;

		c = tegra_of_clk_src_onecell_get(&clkspec, data);
		if (IS_ERR_OR_NULL(c))
			continue;

		tegra_clk_debugfs_add(c);
		tegra_clk_bpmp_debugfs_add(c);
	}
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(clk_init_fops, NULL, clk_init_set, "%llu\n");

static int __init bpmp_clk_debug_init(void)
{
	if (!clk_data.staged && clk_data.cell.clks)
		tegra_bpmp_debugfs_add_file("clk_init", S_IWUSR, &clk_data,
					    &clk_init_fops);
	return 0;
}
late_initcall(bpmp_clk_debug_init);

#endif
