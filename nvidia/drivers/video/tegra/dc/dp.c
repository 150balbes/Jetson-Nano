/*
 * dp.c: tegra dp driver.
 *
 * Copyright (c) 2011-2020, NVIDIA CORPORATION, All rights reserved.
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

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/seq_file.h>
#include <linux/debugfs.h>
#include <linux/version.h>
#include <soc/tegra/chip-id.h>
#include <linux/clk/tegra.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/ctype.h>
#include <linux/extcon/extcon-disp.h>
#include <linux/extcon.h>
#ifdef CONFIG_SWITCH
#include <linux/switch.h>
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 14, 0)
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#else
#include <asm/uaccess.h>
#endif

#include "dc.h"
#include "dp.h"
#include "sor.h"
#include "sor_regs.h"
#include "dpaux_regs.h"
#include "dpaux.h"
#include "dc_priv.h"
#include "edid.h"
#include "hdcp/dphdcp.h"
#include "dp_lt.h"
#include "dp_auto.h"

#include "hda_dc.h"

#include "fake_panel.h"
#include <linux/tegra_prod.h>

#include "bridge/hdmi2fpd_ds90uh949.h"
static bool tegra_dp_debug = true;
module_param(tegra_dp_debug, bool, 0644);
MODULE_PARM_DESC(tegra_dp_debug, "Enable to print all link configs");

/*
 * WAR for DPR-120 firmware v1.9[r6] limitation for CTS 400.3.2.*
 * The analyzer issues IRQ_EVENT while we are still link training.
 * Not expected but analyzer limitation.
 * Ongoing link training confuses the analyzer leading to false failure.
 * The WAR eludes link training during unblank. This keeps the purpose
 * of CTS intact within analyzer limitation.
 */
static bool no_lt_at_unblank = false;
module_param(no_lt_at_unblank, bool, 0644);
MODULE_PARM_DESC(no_lt_at_unblank, "DP enabled but link not trained");

static struct tegra_hpd_ops hpd_ops;

static int dp_instance;

static void tegra_dc_dp_debugfs_create(struct tegra_dc_dp_data *dp);
static void tegra_dc_dp_debugfs_remove(struct tegra_dc_dp_data *dp);
static int tegra_dp_init_max_link_cfg(struct tegra_dc_dp_data *dp,
					struct tegra_dc_dp_link_config *cfg);
static inline void tegra_dp_default_int(struct tegra_dc_dp_data *dp,
					bool enable);
__maybe_unused
static void tegra_dp_hpd_config(struct tegra_dc_dp_data *dp);

static inline void tegra_dp_clk_enable(struct tegra_dc_dp_data *dp)
{
	tegra_disp_clk_prepare_enable(dp->parent_clk);
}

static inline void tegra_dp_clk_disable(struct tegra_dc_dp_data *dp)
{
	tegra_disp_clk_disable_unprepare(dp->parent_clk);
}

static inline void tegra_dp_enable_irq(u32 irq)
{
	enable_irq(irq);
}

static inline void tegra_dp_disable_irq(u32 irq)
{
	disable_irq(irq);
}

static inline void tegra_dp_pending_hpd(struct tegra_dc_dp_data *dp)
{
	if (!is_hotplug_supported(dp))
		return;

	tegra_hpd_set_pending_evt(&dp->hpd_data);
}

static inline void tegra_dp_hpd_suspend(struct tegra_dc_dp_data *dp)
{
	if (!is_hotplug_supported(dp))
		return;

	tegra_hpd_suspend(&dp->hpd_data);
}

int tegra_dc_dp_dpcd_read(struct tegra_dc_dp_data *dp, u32 cmd,
	u8 *data_ptr)
{
	u32 size = 1;
	u32 status = 0;
	int ret = 0;

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return ret;

	mutex_lock(&dp->dpaux->lock);
	tegra_dpaux_get(dp->dpaux);
	ret = tegra_dc_dpaux_read_chunk_locked(dp->dpaux,
			DPAUX_DP_AUXCTL_CMD_AUXRD, cmd, data_ptr,
			&size, &status);
	tegra_dpaux_put(dp->dpaux);
	mutex_unlock(&dp->dpaux->lock);
	if (ret)
		dev_err(&dp->dc->ndev->dev,
			"dp: Failed to read DPCD data. CMD 0x%x, Status 0x%x\n",
			cmd, status);

	return ret;
}

static int tegra_dc_dp_i2c_xfer(struct tegra_dc *dc, struct i2c_msg *msgs,
	int num)
{
	struct i2c_msg *pmsg;
	int i;
	u32 aux_stat;
	int status = 0;
	u32 len = 0;
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);

	/* No physical panel and/or emulator is attached in simulation. */
	if (tegra_platform_is_sim())
		return -EINVAL;

	for (i = 0; i < num; ++i) {
		pmsg = &msgs[i];

		if (!pmsg->flags) {
			len = pmsg->len;

			status = tegra_dc_dpaux_i2c_write(dp->dpaux,
					DPAUX_DP_AUXCTL_CMD_MOTWR,
					pmsg->addr, pmsg->buf, &len, &aux_stat);
			if (status) {
				dev_err(&dp->dc->ndev->dev,
					"dp: Failed for I2C write"
					" addr:%d, size:%d, stat:0x%x\n",
					pmsg->addr, len, aux_stat);
				return status;
			}
		} else if (pmsg->flags & I2C_M_RD) {
			len = pmsg->len;

			status = tegra_dc_dpaux_i2c_read(dp->dpaux, pmsg->addr,
						pmsg->buf, &len, &aux_stat);
			if (status) {
				dev_err(&dp->dc->ndev->dev,
					"dp: Failed for I2C read"
					" addr:%d, size:%d, stat:0x%x\n",
					pmsg->addr, len, aux_stat);
				return status;
			}
		} else {
			dev_err(&dp->dc->ndev->dev,
				"dp: i2x_xfer: Invalid i2c flag 0x%x\n",
				pmsg->flags);
			return -EINVAL;
		}
	}

	return i;
}

static i2c_transfer_func_t tegra_dp_hpd_op_edid_read(void *drv_data)
{
	struct tegra_dc_dp_data *dp = drv_data;

	return (dp->edid_src == EDID_SRC_DT) ?
		tegra_dc_edid_blob : tegra_dc_dp_i2c_xfer;
}

int tegra_dc_dp_dpcd_write(struct tegra_dc_dp_data *dp, u32 cmd,
	u8 data)
{
	u32 size = 1;
	u32 status = 0;
	int ret;

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return 0;

	mutex_lock(&dp->dpaux->lock);
	tegra_dpaux_get(dp->dpaux);
	ret = tegra_dc_dpaux_write_chunk_locked(dp->dpaux,
			DPAUX_DP_AUXCTL_CMD_AUXWR, cmd, &data, &size, &status);
	tegra_dpaux_put(dp->dpaux);
	mutex_unlock(&dp->dpaux->lock);
	if (ret)
		dev_err(&dp->dc->ndev->dev,
			"dp: Failed to write DPCD data. CMD 0x%x, Status 0x%x\n",
			cmd, status);
	return ret;
}

int tegra_dp_dpcd_write_field(struct tegra_dc_dp_data *dp,
					u32 cmd, u8 mask, u8 data)
{
	u8 dpcd_data;
	int ret;

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return 0;

	might_sleep();

	ret = tegra_dc_dp_dpcd_read(dp, cmd, &dpcd_data);
	if (ret)
		return ret;

	dpcd_data &= ~mask;
	dpcd_data |= data;

	ret = tegra_dc_dp_dpcd_write(dp, cmd, dpcd_data);
	if (ret)
		return ret;

	return 0;
}

static inline u64 tegra_div64(u64 dividend, u32 divisor)
{
	do_div(dividend, divisor);
	return dividend;
}

static inline bool tegra_dp_is_audio_supported(struct tegra_dc_dp_data *dp)
{
	if (tegra_edid_audio_supported(dp->hpd_data.edid)
		&& tegra_dc_is_ext_panel(dp->dc) &&
		dp->dc->out->type != TEGRA_DC_OUT_FAKE_DP)
		return true;
	else
		return false;
}

#ifdef CONFIG_DEBUG_FS
static int dbg_dp_dpaux_show(struct seq_file *s, void *unused)
{
#define DUMP_REG(a) seq_printf(s, "%-32s  %03x	%08x\n",	\
		#a, a, tegra_dpaux_readl(dpaux, a))

	struct tegra_dc_dp_data *dp = s->private;
	struct tegra_dc_dpaux_data *dpaux = dp->dpaux;

	tegra_dpaux_get(dpaux);

	DUMP_REG(DPAUX_INTR_EN_AUX);
	DUMP_REG(DPAUX_INTR_AUX);
	DUMP_REG(DPAUX_DP_AUXADDR);
	DUMP_REG(DPAUX_DP_AUXCTL);
	DUMP_REG(DPAUX_DP_AUXSTAT);
	DUMP_REG(DPAUX_HPD_CONFIG);
	DUMP_REG(DPAUX_HPD_IRQ_CONFIG);
	DUMP_REG(DPAUX_DP_AUX_CONFIG);
	DUMP_REG(DPAUX_HYBRID_PADCTL);
	DUMP_REG(DPAUX_HYBRID_SPARE);

	tegra_dpaux_put(dpaux);
	return 0;
}

static int dbg_dp_dpaux_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_dp_dpaux_show, inode->i_private);
}

static const struct file_operations dbg_fops = {
	.open		= dbg_dp_dpaux_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int lane_count_show(struct seq_file *s, void *unused)
{
	struct tegra_dc_dp_data *dp = s->private;
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;

	seq_puts(s, "\n");
	seq_printf(s,
		"DP Lane_Count: \t%d\n",
		cfg->lane_count);
	return 0;
}

static ssize_t lane_count_set(struct file *file, const char __user *buf,
						size_t count, loff_t *off)
{
	struct seq_file *s = file->private_data;
	struct tegra_dc_dp_data *dp = s->private;
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;
	long lane_count = 0;
	int ret = 0;

	ret = kstrtol_from_user(buf, count, 10, &lane_count);
	if (ret < 0)
		return ret;

	if (cfg->lane_count == lane_count)
		return count;

	dp->test_max_lanes = lane_count;
	cfg->is_valid = false;
	tegra_dp_init_max_link_cfg(dp, cfg);

	return count;
}

static int lane_count_open(struct inode *inode, struct file *file)
{
	return single_open(file, lane_count_show, inode->i_private);
}

static const struct file_operations lane_count_fops = {
	.open		= lane_count_open,
	.read		= seq_read,
	.write		= lane_count_set,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int link_speed_show(struct seq_file *s, void *unused)
{
	struct tegra_dc_dp_data *dp = s->private;
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;

	seq_puts(s, "\n");
	seq_printf(s,
		"DP Link Speed: \t%d\n",
		cfg->link_bw);
	return 0;
}

static ssize_t link_speed_set(struct file *file, const char __user *buf,
						size_t count, loff_t *off)
{
	struct seq_file *s = file->private_data;
	struct tegra_dc_dp_data *dp = s->private;
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;
	long link_speed = 0;
	int ret = 0;

	ret = kstrtol_from_user(buf, count, 10, &link_speed);
	if (ret < 0)
		return ret;

	if (cfg->link_bw == link_speed)
		return count;

	dp->test_max_link_bw = link_speed;
	cfg->is_valid = false;
	tegra_dp_init_max_link_cfg(dp, cfg);

	return count;
}

static int link_speed_open(struct inode *inode, struct file *file)
{
	return single_open(file, link_speed_show, inode->i_private);
}

static const struct file_operations link_speed_fops = {
	.open		= link_speed_open,
	.read		= seq_read,
	.write		= link_speed_set,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int dbg_hotplug_show(struct seq_file *m, void *unused)
{
	struct tegra_dc_dp_data *dp = m->private;
	struct tegra_dc *dc = dp->dc;

	if (WARN_ON(!dp || !dc || !dc->out))
		return -EINVAL;

	seq_printf(m, "dp hpd state: %d\n", dc->out->hotplug_state);
	return 0;
}

static int dbg_hotplug_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_hotplug_show, inode->i_private);
}

/*
 * sw control for hpd.
 * 0 is normal state, hw drives hpd.
 * -1 is force deassert, sw drives hpd.
 * 1 is force assert, sw drives hpd.
 * before releasing to hw, sw must ensure hpd state is normal i.e. 0
 */
static ssize_t dbg_hotplug_write(struct file *file, const char __user *addr,
	size_t len, loff_t *pos)
{
	struct seq_file *m = file->private_data; /* single_open() initialized */
	struct tegra_dc_dp_data *dp = m->private;
	struct tegra_dc_dpaux_data *dpaux = dp->dpaux;
	struct tegra_dc *dc = dp->dc;
	int ret;
	long new_state;

	if (WARN_ON(!dp || !dc || !dpaux || !dc->out))
		return -EINVAL;

	ret = kstrtol_from_user(addr, len, 10, &new_state);
	if (ret < 0)
		return ret;

	if (dc->out->hotplug_state == TEGRA_HPD_STATE_NORMAL
		&& new_state != TEGRA_HPD_STATE_NORMAL
		&& dc->hotplug_supported) {
		/* SW controlled hotplug. Ignore hpd HW interrupts. */
		tegra_dpaux_int_toggle(dpaux, DPAUX_INTR_EN_AUX_PLUG_EVENT |
				DPAUX_INTR_EN_AUX_UNPLUG_EVENT |
				DPAUX_INTR_EN_AUX_PLUG_EVENT,
				false);
	} else if (dc->out->hotplug_state != TEGRA_HPD_STATE_NORMAL
		&& new_state == TEGRA_HPD_STATE_NORMAL
		&& dc->hotplug_supported) {
		/* Enable hpd HW interrupts */
		tegra_dpaux_int_toggle(dpaux, DPAUX_INTR_EN_AUX_PLUG_EVENT |
				DPAUX_INTR_EN_AUX_UNPLUG_EVENT |
				DPAUX_INTR_EN_AUX_PLUG_EVENT,
				true);
	}

	dc->out->hotplug_state = new_state;

	reinit_completion(&dc->hpd_complete);
	tegra_dp_pending_hpd(dp);
	wait_for_completion(&dc->hpd_complete);
	return len;
}

static const struct file_operations dbg_hotplug_fops = {
	.open = dbg_hotplug_open,
	.read = seq_read,
	.write = dbg_hotplug_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int bits_per_pixel_show(struct seq_file *s, void *unused)
{
	struct tegra_dc_dp_data *dp = s->private;
	struct tegra_dc_dp_link_config *cfg = NULL;

	if (WARN_ON(!dp || !dp->dc || !dp->dc->out))
		return -EINVAL;
	cfg = &dp->link_cfg;

	if (WARN_ON(!cfg))
		return -EINVAL;

	seq_puts(s, "\n");
	seq_printf(s, "DP Bits Per Pixel: %u\n", cfg->bits_per_pixel);
	return 0;
}

static ssize_t bits_per_pixel_set(struct file *file, const char __user *buf,
						size_t count, loff_t *off)
{
	struct seq_file *s = file->private_data;
	struct tegra_dc_dp_data *dp = s->private;
	struct tegra_dc_dpaux_data *dpaux = dp->dpaux;
	struct tegra_dc_dp_link_config *cfg = NULL;
	u32 bits_per_pixel = 0;
	int ret = 0;

	if (WARN_ON(!dp || !dp->dc || !dp->dc->out))
		return -EINVAL;

	ret = kstrtouint_from_user(buf, count, 10, &bits_per_pixel);
	if (ret < 0)
		return ret;

	cfg = &dp->link_cfg;

	if (WARN_ON(!cfg))
		return -EINVAL;

	if (cfg->bits_per_pixel == bits_per_pixel)
		return count;

	if ((bits_per_pixel == 18) || (bits_per_pixel == 24))
		dev_info(&dp->dc->ndev->dev, "Setting the bits per pixel from %u to %u\n",
			cfg->bits_per_pixel, bits_per_pixel);
	else {
		dev_info(&dp->dc->ndev->dev, "%ubpp is not supported. Restoring to %ubpp\n",
		bits_per_pixel, cfg->bits_per_pixel);

		return count;
	}

	tegra_dpaux_int_toggle(dpaux, DPAUX_INTR_EN_AUX_PLUG_EVENT |
			DPAUX_INTR_EN_AUX_UNPLUG_EVENT |
			DPAUX_INTR_EN_AUX_PLUG_EVENT,
			false);
	dp->dc->out->hotplug_state = TEGRA_HPD_STATE_FORCE_DEASSERT;
	tegra_dp_pending_hpd(dp);

	/* wait till HPD state machine has reached disable state */
	msleep(HPD_DROP_TIMEOUT_MS + 500);

	dp->dc->out->depth = bits_per_pixel;

	tegra_dpaux_int_toggle(dpaux, DPAUX_INTR_EN_AUX_PLUG_EVENT |
			DPAUX_INTR_EN_AUX_UNPLUG_EVENT |
			DPAUX_INTR_EN_AUX_PLUG_EVENT,
			true);
	dp->dc->out->hotplug_state = TEGRA_HPD_STATE_NORMAL;
	tegra_dp_pending_hpd(dp);

	if (tegra_dp_is_audio_supported(dp)) {
		disp_state_extcon_aux_report(dp->sor->ctrl_num,
			EXTCON_DISP_AUX_STATE_DISABLED);
		pr_info("Extcon AUX%d(DP): disable\n", dp->sor->ctrl_num);

		usleep_range(1000, 1020);

		disp_state_extcon_aux_report(dp->sor->ctrl_num,
			EXTCON_DISP_AUX_STATE_ENABLED);
		pr_info("Extcon AUX%d(DP): enable\n", dp->sor->ctrl_num);
	}

#ifdef CONFIG_SWITCH
	if (tegra_edid_audio_supported(dp->hpd_data.edid) &&
		tegra_dc_is_ext_panel(dp->dc) &&
		dp->dc->out->type != TEGRA_DC_OUT_FAKE_DP) {
		switch_set_state(&dp->audio_switch, 0);
		msleep(1);
		pr_info("audio_switch toggle 0\n");
		switch_set_state(&dp->audio_switch, 1);
		pr_info("audio_switch toggle 1\n");
	}
#endif

	return count;
}

static int bits_per_pixel_open(struct inode *inode, struct file *file)
{
	return single_open(file, bits_per_pixel_show, inode->i_private);
}

static const struct file_operations bits_per_pixel_fops = {
	.open		= bits_per_pixel_open,
	.read		= seq_read,
	.write		= bits_per_pixel_set,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static inline void dpaux_print_data(struct seq_file *s, u8 *data, u32 size)
{
	u8 row_size = 16;
	u32 i, j;

	for (i = 0; i < size; i += row_size) {
		for (j = i; j < i + row_size && j < size; j++)
			seq_printf(s, "%02x ", data[j]);
		seq_puts(s, "\n");
	}
}

static int dpaux_i2c_data_show(struct seq_file *s, void *unused)
{
	struct tegra_dc_dp_data *dp = s->private;
	u32 addr = dp->dpaux_i2c_dbg_addr;
	u32 size = dp->dpaux_i2c_dbg_num_bytes;
	u32 aux_stat;
	u8 *data;
	int ret;

	data = kzalloc(size, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	ret = tegra_dc_dpaux_i2c_read(dp->dpaux, addr, data, &size, &aux_stat);
	if (ret) {
		seq_printf(s, "Error reading %d bytes from I2C reg %x",
			dp->dpaux_i2c_dbg_num_bytes,
			dp->dpaux_i2c_dbg_addr);
		goto free_mem;
	}

	dpaux_print_data(s, data, size);

free_mem:
	kfree(data);
	return ret;
}

static int dpaux_dpcd_data_show(struct seq_file *s, void *unused)
{
	struct tegra_dc_dp_data *dp = s->private;
	u32 addr = dp->dpaux_dpcd_dbg_addr;
	u32 size = dp->dpaux_dpcd_dbg_num_bytes;
	u32 i;
	u8 *data;
	int ret;

	data = kzalloc(size, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	tegra_dpaux_get(dp->dpaux);
	for (i = 0; i < size; i++) {
		ret = tegra_dc_dp_dpcd_read(dp, addr+i, data+i);
		if (ret) {
			seq_printf(s, "Reading %d bytes from reg %x; "
				   "Error at DPCD reg offset %x\n",
				dp->dpaux_dpcd_dbg_num_bytes,
				dp->dpaux_dpcd_dbg_addr,
				addr+i);
			goto free_mem;
		}
	}

	dpaux_print_data(s, data, size);

free_mem:
	tegra_dpaux_put(dp->dpaux);
	kfree(data);
	return ret;
}

static inline int dpaux_parse_input(const char __user *user_buf,
					u8 *data, size_t count)
{
	int size = 0;
	u32 i = 0;
	char tmp[3];
	char *buf;

	buf = kzalloc(count, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	if (copy_from_user(buf, user_buf, count)) {
		size = -EINVAL;
		goto free_mem;
	}

	/*
	 * Assumes each line of input is of the form: XX XX XX XX ...,
	 * where X represents one hex digit. You can have an arbitrary
	 * amount of whitespace between each XX.
	 */
	while (i + 1 < count) {
		if (buf[i] == ' ') {
			i += 1;
			continue;
		}

		tmp[0] = buf[i]; tmp[1] = buf[i + 1]; tmp[2] = '\0';
		if (kstrtou8(tmp, 16, data + size)) {
			size = -EINVAL;
			goto free_mem;
		}

		size += 1;
		i += 2;
	}

free_mem:
	kfree(buf);
	return size;
}

static ssize_t dpaux_i2c_data_set(struct file *file,
	const char __user *user_buf, size_t count, loff_t *off)
{
	struct seq_file *s = file->private_data;
	struct tegra_dc_dp_data *dp = s->private;
	int size = 0;
	u32 aux_stat;
	u32 addr = dp->dpaux_i2c_dbg_addr;
	u8 *data;
	int ret = count;

	data = kzalloc(count, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	size = dpaux_parse_input(user_buf, data, count);
	if (size <= 0) {
		ret = -EINVAL;
		goto free_mem;
	}

	ret = tegra_dc_dpaux_i2c_write(dp->dpaux, DPAUX_DP_AUXCTL_CMD_I2CWR,
					addr, data, &size, &aux_stat);
	if (!ret)
		ret = count;
	else
		ret = -EIO;

free_mem:
	kfree(data);

	return ret;
}

static ssize_t dpaux_dpcd_data_set(struct file *file,
	const char __user *user_buf, size_t count, loff_t *off)
{
	struct seq_file *s = file->private_data;
	struct tegra_dc_dp_data *dp = s->private;
	int size = 0;
	u32 aux_stat;
	u32 addr = dp->dpaux_dpcd_dbg_addr;
	u8 *data;
	int ret = count;

	data = kzalloc(count, GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	size = dpaux_parse_input(user_buf, data, count);
	if (size <= 0) {
		ret = -EINVAL;
		goto free_mem;
	}

	ret = tegra_dc_dpaux_write(dp->dpaux, DPAUX_DP_AUXCTL_CMD_AUXWR,
				addr, data, &size, &aux_stat);
	if (!ret)
		ret = count;
	else
		ret = -EIO;

free_mem:
	kfree(data);

	return ret;
}

static int dpaux_i2c_data_open(struct inode *inode, struct file *file)
{
	return single_open(file, dpaux_i2c_data_show, inode->i_private);
}

static int dpaux_dpcd_data_open(struct inode *inode, struct file *file)
{
	return single_open(file, dpaux_dpcd_data_show, inode->i_private);
}

static const struct file_operations dpaux_i2c_data_fops = {
	.open		= dpaux_i2c_data_open,
	.read		= seq_read,
	.write		= dpaux_i2c_data_set,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations dpaux_dpcd_data_fops = {
	.open		= dpaux_dpcd_data_open,
	.read		= seq_read,
	.write		= dpaux_dpcd_data_set,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static struct dentry *tegra_dpaux_i2c_dir_create(struct tegra_dc_dp_data *dp,
	struct dentry *parent)
{
	struct dentry *dpaux_i2c_dir;
	struct dentry *retval = NULL;

	dpaux_i2c_dir = debugfs_create_dir("dpaux_i2c", parent);
	if (!dpaux_i2c_dir)
		return retval;
	retval = debugfs_create_u16("addr", 0644, dpaux_i2c_dir,
			&dp->dpaux_i2c_dbg_addr);
	if (!retval)
		goto free_out;
	retval = debugfs_create_u32("num_bytes", 0644,
			dpaux_i2c_dir, &dp->dpaux_i2c_dbg_num_bytes);
	if (!retval)
		goto free_out;
	retval = debugfs_create_file("data", 0444, dpaux_i2c_dir, dp,
			&dpaux_i2c_data_fops);
	if (!retval)
		goto free_out;

	return retval;
free_out:
	debugfs_remove_recursive(dpaux_i2c_dir);
	return retval;
}

static struct dentry *tegra_dpaux_dpcd_dir_create(struct tegra_dc_dp_data *dp,
	struct dentry *parent)
{
	struct dentry *dpaux_dir;
	struct dentry *retval = NULL;

	dpaux_dir = debugfs_create_dir("dpaux_dpcd", parent);
	if (!dpaux_dir)
		return retval;
	retval = debugfs_create_u16("addr", 0644, dpaux_dir,
			&dp->dpaux_dpcd_dbg_addr);
	if (!retval)
		goto free_out;
	retval = debugfs_create_u32("num_bytes", 0644,
			dpaux_dir, &dp->dpaux_dpcd_dbg_num_bytes);
	if (!retval)
		goto free_out;
	retval = debugfs_create_file("data", 0444, dpaux_dir, dp,
			&dpaux_dpcd_data_fops);
	if (!retval)
		goto free_out;

	return retval;
free_out:
	debugfs_remove_recursive(dpaux_dir);
	return retval;
}

static void tegra_dc_dp_debugfs_create(struct tegra_dc_dp_data *dp)
{
	struct dentry *retval;
	char debug_dirname[CHAR_BUF_SIZE_MAX];

	snprintf(debug_dirname, sizeof(debug_dirname),
		"tegra_dp%d", dp->dc->ndev->id);

	dp->debugdir = debugfs_create_dir(debug_dirname, NULL);
	if (!dp->debugdir) {
		dev_err(&dp->dc->ndev->dev, "could not create %s debugfs\n",
			debug_dirname);
		return;
	}
	retval = debugfs_create_file("dpaux_regs", 0444, dp->debugdir, dp,
		&dbg_fops);
	if (!retval)
		goto free_out;
	retval = debugfs_create_file("lanes", 0444, dp->debugdir, dp,
		&lane_count_fops);
	if (!retval)
		goto free_out;
	retval = debugfs_create_file("linkspeed", 0444, dp->debugdir, dp,
		&link_speed_fops);
	if (!retval)
		goto free_out;
	retval = debugfs_create_file("bitsperpixel", 0444, dp->debugdir, dp,
		&bits_per_pixel_fops);
	if (!retval)
		goto free_out;
	retval = debugfs_create_file("test_settings", 0444, dp->debugdir, dp,
		&test_settings_fops);
	if (!retval)
		goto free_out;
	retval = tegra_dpaux_i2c_dir_create(dp, dp->debugdir);
	if (!retval)
		goto free_out;
	retval = tegra_dpaux_dpcd_dir_create(dp, dp->debugdir);
	if (!retval)
		goto free_out;

	/* hotplug not allowed for eDP */
	if (is_hotplug_supported(dp)) {
		retval = debugfs_create_file("hotplug", 0444, dp->debugdir,
			dp, &dbg_hotplug_fops);
		if (!retval)
			goto free_out;
	}

	return;
free_out:
	dev_err(&dp->dc->ndev->dev, "could not create %s debugfs\n",
		debug_dirname);
	tegra_dc_dp_debugfs_remove(dp);
}

static void tegra_dc_dp_debugfs_remove(struct tegra_dc_dp_data *dp)
{
	debugfs_remove_recursive(dp->debugdir);
	dp->debugdir = NULL;
}
#else
static void tegra_dc_dp_debugfs_create(struct tegra_dc_dp_data *dp)
{ }

static void tegra_dc_dp_debugfs_remove(struct tegra_dc_dp_data *dp)
{ }
#endif

static void tegra_dp_dpaux_enable(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc_dpaux_data *dpaux = dp->dpaux;

	tegra_dpaux_config_pad_mode(dpaux, TEGRA_DPAUX_PAD_MODE_AUX);
	if (dp->dc->out->type != TEGRA_DC_OUT_FAKE_DP) {
		tegra_dp_enable_irq(dp->irq);
		tegra_dp_hpd_config(dp);
		tegra_dp_default_int(dp, true);
	}
}

static void tegra_dp_dpaux_disable(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc_dpaux_data *dpaux = dp->dpaux;

	tegra_dpaux_pad_power(dpaux, false);
	tegra_dpaux_put(dpaux);

	if (dp->sor->safe_clk)
		tegra_sor_safe_clk_disable(dp->sor);
}

static int tegra_dp_panel_power_state(struct tegra_dc_dp_data *dp, u8 state)
{
	u32 retry = 0;
	int ret;

	do {
		ret = tegra_dc_dp_dpcd_write(dp, NV_DPCD_SET_POWER, state);
	} while ((state != NV_DPCD_SET_POWER_VAL_D3_PWRDWN) &&
		(retry++ < DP_POWER_ON_MAX_TRIES) && ret);

	return ret;
}

/* Calcuate if given cfg can meet the mode request. */
/* Return true if mode is possible, false otherwise. */
bool tegra_dc_dp_calc_config(struct tegra_dc_dp_data *dp,
	const struct tegra_dc_mode *mode,
	struct tegra_dc_dp_link_config *cfg)
{
	const u32	link_rate = 27 * cfg->link_bw * 1000 * 1000;
	const u64	f	  = 100000;	/* precision factor */

	u32	num_linkclk_line; /* Number of link clocks per line */
	u64	ratio_f; /* Ratio of incoming to outgoing data rate */
	u64	frac_f;
	u64	activesym_f;	/* Activesym per TU */
	u64	activecount_f;
	u32	activecount;
	u32	activepolarity;
	u64	approx_value_f;
	u32	activefrac		  = 0;
	u64	accumulated_error_f	  = 0;
	u32	lowest_neg_activecount	  = 0;
	u32	lowest_neg_activepolarity = 0;
	u32	lowest_neg_tusize	  = 64;
	u32	num_symbols_per_line;
	u64	lowest_neg_activefrac	  = 0;
	u64	lowest_neg_error_f	  = 64 * f;
	u64	watermark_f;

	int	i;
	bool	neg;
	unsigned long rate;

	cfg->is_valid = false;

	/* The pclk rate is fixed at 27 MHz on FPGA. */
	if (tegra_dc_is_t19x() && tegra_platform_is_fpga())
		rate = 27000000;
	else
		rate = tegra_dc_clk_get_rate(dp->dc);

	if (!link_rate || !cfg->lane_count || !rate ||
		!cfg->bits_per_pixel)
		return false;

	if ((u64)rate * cfg->bits_per_pixel >=
		(u64)link_rate * 8 * cfg->lane_count) {
		dev_dbg(&dp->dc->ndev->dev,
			"Requested rate calc > link_rate calc\n");
		return false;
	}

	num_linkclk_line = (u32)tegra_div64(
		(u64)link_rate * mode->h_active, rate);

	ratio_f = (u64)rate * cfg->bits_per_pixel * f;
	ratio_f /= 8;
	ratio_f = tegra_div64(ratio_f, link_rate * cfg->lane_count);

	for (i = 64; i >= 32; --i) {
		activesym_f	= ratio_f * i;
		activecount_f	= tegra_div64(activesym_f, (u32)f) * f;
		frac_f		= activesym_f - activecount_f;
		activecount	= (u32)tegra_div64(activecount_f, (u32)f);

		if (frac_f < (f / 2)) /* fraction < 0.5 */
			activepolarity = 0;
		else {
			activepolarity = 1;
			frac_f = f - frac_f;
		}

		if (frac_f != 0) {
			frac_f = tegra_div64((f * f),  frac_f); /* 1/fraction */
			if (frac_f > (15 * f))
				activefrac = activepolarity ? 1 : 15;
			else
				activefrac = activepolarity ?
					(u32)tegra_div64(frac_f, (u32)f) + 1 :
					(u32)tegra_div64(frac_f, (u32)f);
		}

		if (activefrac == 1)
			activepolarity = 0;

		if (activepolarity == 1)
			approx_value_f = activefrac ? tegra_div64(
				activecount_f + (activefrac * f - f) * f,
				(activefrac * f)) :
				activecount_f + f;
		else
			approx_value_f = activefrac ?
				activecount_f + tegra_div64(f, activefrac) :
				activecount_f;

		if (activesym_f < approx_value_f) {
			accumulated_error_f = num_linkclk_line *
				tegra_div64(approx_value_f - activesym_f, i);
			neg = true;
		} else {
			accumulated_error_f = num_linkclk_line *
				tegra_div64(activesym_f - approx_value_f, i);
			neg = false;
		}

		if ((neg && (lowest_neg_error_f > accumulated_error_f)) ||
			(accumulated_error_f == 0)) {
			lowest_neg_error_f = accumulated_error_f;
			lowest_neg_tusize = i;
			lowest_neg_activecount = activecount;
			lowest_neg_activepolarity = activepolarity;
			lowest_neg_activefrac = activefrac;

			if (accumulated_error_f == 0)
				break;
		}
	}

	if (lowest_neg_activefrac == 0) {
		cfg->activepolarity = 0;
		cfg->active_count   = lowest_neg_activepolarity ?
			lowest_neg_activecount : lowest_neg_activecount - 1;
		cfg->tu_size	      = lowest_neg_tusize;
		cfg->active_frac    = 1;
	} else {
		cfg->activepolarity = lowest_neg_activepolarity;
		cfg->active_count   = (u32)lowest_neg_activecount;
		cfg->tu_size	      = lowest_neg_tusize;
		cfg->active_frac    = (u32)lowest_neg_activefrac;
	}

	dev_dbg(&dp->dc->ndev->dev,
		"dp: sor configuration: polarity: %d active count: %d "
		"tu size: %d, active frac: %d\n",
		cfg->activepolarity, cfg->active_count, cfg->tu_size,
		cfg->active_frac);

	watermark_f = tegra_div64(ratio_f * cfg->tu_size * (f - ratio_f), f);
	cfg->watermark = (u32)tegra_div64(watermark_f + lowest_neg_error_f,
		f) + cfg->bits_per_pixel / 4 - 1;
	num_symbols_per_line = (mode->h_active * cfg->bits_per_pixel) /
		(8 * cfg->lane_count);
	if (cfg->watermark > 30) {
		dev_dbg(&dp->dc->ndev->dev,
			"dp: sor setting: unable to get a good tusize, "
			"force watermark to 30.\n");
		cfg->watermark = 30;
		return false;
	} else if (cfg->watermark > num_symbols_per_line) {
		dev_dbg(&dp->dc->ndev->dev,
			"dp: sor setting: force watermark to the number "
			"of symbols in the line.\n");
		cfg->watermark = num_symbols_per_line;
		return false;
	}

	/* Refer to dev_disp.ref for more information. */
	/* # symbols/hblank = ((SetRasterBlankEnd.X + SetRasterSize.Width - */
	/*                      SetRasterBlankStart.X - 7) * link_clk / pclk) */
	/*                      - 3 * enhanced_framing - Y */
	/* where Y = (# lanes == 4) 3 : (# lanes == 2) ? 6 : 12 */
	cfg->hblank_sym = (int)tegra_div64((u64)(mode->h_back_porch +
			mode->h_front_porch + mode->h_sync_width - 7)
		* link_rate, rate)
		- 3 * cfg->enhanced_framing - (12 / cfg->lane_count);

	if (cfg->hblank_sym < 0)
		cfg->hblank_sym = 0;


	/* Refer to dev_disp.ref for more information. */
	/* # symbols/vblank = ((SetRasterBlankStart.X - */
	/*                      SetRasterBlankEen.X - 25) * link_clk / pclk) */
	/*                      - Y - 1; */
	/* where Y = (# lanes == 4) 12 : (# lanes == 2) ? 21 : 39 */
	cfg->vblank_sym = (int)tegra_div64((u64)(mode->h_active - 25)
		* link_rate, rate) - (36 / cfg->lane_count) - 4;

	if (cfg->vblank_sym < 0)
		cfg->vblank_sym = 0;

	cfg->is_valid = true;

	return true;
}

int tegra_dc_dp_read_ext_dpcd_caps(struct tegra_dc_dp_data *dp,
				struct tegra_dc_dp_ext_dpcd_caps *ext_caps)
{
	u8 dpcd_data = 0;
	int ret = 0;

	ext_caps->valid = false;

	ret = tegra_dc_dp_dpcd_read(dp, NV_DPCD_TRAINING_AUX_RD_INTERVAL,
				&dpcd_data);
	if (ret || !(dpcd_data >> NV_DPCD_EXT_RECEIVER_CAP_FIELD_PRESENT_SHIFT))
		return ret;

	ret = tegra_dc_dp_dpcd_read(dp, NV_DPCD_REV_EXT_CAP,
				&ext_caps->revision);
	if (ret)
		return ret;

	ret = tegra_dc_dp_dpcd_read(dp, NV_DPCD_MAX_LINK_BANDWIDTH_EXT_CAP,
				&ext_caps->max_link_bw);
	if (ret)
		return ret;

	ext_caps->valid = true;
	return ret;
}

int tegra_dc_dp_get_max_link_bw(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;
	u8 max_link_bw = 0;

	/*
	 * The max link bw supported is a product of everything below:
	 * 1) The max link bw supported by the DPRX.
	 * 2) If the DPRX advertises an additional max link bw value as part of
	 *    the Extended Receiver Capability field, this value should override
	 *    the value in #1.
	 * 3) The max link bw supported by the DPTX.
	 * 4) If test configuration parameters are set on the DPTX side, these
	 *    parameters need to be accounted for as well.
	 */

	/* Constraint #1 */
	if (dp->sink_cap_valid) {
		max_link_bw = dp->sink_cap[NV_DPCD_MAX_LINK_BANDWIDTH];
	} else if (tegra_dc_dp_dpcd_read(dp, NV_DPCD_MAX_LINK_BANDWIDTH,
					&max_link_bw)) {
		dev_err(&dp->dc->ndev->dev,
			"dp: NV_DPCD_MAX_LINK_BANDWIDTH read failed\n");
		return 0;
	}

	/* Constraint #2 */
	if (tegra_dc_is_ext_panel(dp->dc) && !cfg->ext_dpcd_caps.valid) {
		/* DPCD caps are already read in hpd worker, use them if they
		 * are valid. Also, use cached values for internal panels as
		 * they don't change during runtime */
		if (tegra_dc_dp_read_ext_dpcd_caps(dp, &cfg->ext_dpcd_caps)) {
			dev_err(&dp->dc->ndev->dev,
			"dp: Failed to read ext DPCD caps\n");
			return 0;
		}
	}

	if (cfg->ext_dpcd_caps.valid)
		max_link_bw = cfg->ext_dpcd_caps.max_link_bw;

	/* Constraint #3 */
	if (dp->pdata && dp->pdata->link_bw > 0)
		max_link_bw = min(max_link_bw, (u8)dp->pdata->link_bw);

	/* Constraint #4 */
	if (dp->test_max_link_bw > 0)
		max_link_bw = min(max_link_bw, (u8)dp->test_max_link_bw);

	if (max_link_bw >= NV_DPCD_MAX_LINK_BANDWIDTH_VAL_8_10_GBPS)
		max_link_bw = NV_DPCD_MAX_LINK_BANDWIDTH_VAL_8_10_GBPS;
	else if (max_link_bw >= NV_DPCD_MAX_LINK_BANDWIDTH_VAL_5_40_GBPS)
		max_link_bw = NV_DPCD_MAX_LINK_BANDWIDTH_VAL_5_40_GBPS;
	else if (max_link_bw >= NV_DPCD_MAX_LINK_BANDWIDTH_VAL_2_70_GBPS)
		max_link_bw = NV_DPCD_MAX_LINK_BANDWIDTH_VAL_2_70_GBPS;
	else
		max_link_bw = NV_DPCD_MAX_LINK_BANDWIDTH_VAL_1_62_GBPS;

	return max_link_bw;
}

int tegra_dc_dp_get_max_lane_count(struct tegra_dc_dp_data *dp, u8 *dpcd_data)
{
	u8 max_lane_count = 0;

	/*
	 * The max lane count supported is a product of everything below:
	 * 1) The max lane count supported by the DPRX.
	 * 2) The # of connected lanes reported by the extcon provider (Type-C).
	 * 3) The max lane count supported by the DPTX.
	 * 4) If test configuration parameters are set on the DPTX side, these
	 *    parameters need to be accounted for as well.
	 */

	/* Constraint #1 */
	if (dp->sink_cap_valid) {
		max_lane_count = dp->sink_cap[NV_DPCD_MAX_LANE_COUNT];
	} else if (tegra_dc_dp_dpcd_read(dp, NV_DPCD_MAX_LANE_COUNT,
				&max_lane_count)) {
		dev_err(&dp->dc->ndev->dev,
			"dp: NV_DPCD_MAX_LANE_COUNT read failed\n");
		return 0;
	}
	*dpcd_data = max_lane_count;
	max_lane_count = max_lane_count & NV_DPCD_MAX_LANE_COUNT_MASK;

	/* Constraint #2 */
	max_lane_count = min(max_lane_count, dp->typec_lane_count);

	/* Constraint #3 */
	if (dp->pdata && dp->pdata->lanes > 0)
		max_lane_count = min(max_lane_count, (u8)dp->pdata->lanes);

	/* Constraint #4 */
	if (dp->test_max_lanes > 0)
		max_lane_count = min(max_lane_count, (u8)dp->test_max_lanes);

	if (max_lane_count >= 4)
		max_lane_count = 4;
	else if (max_lane_count >= 2)
		max_lane_count = 2;
	else
		max_lane_count = 1;

	return max_lane_count;
}

static inline u32 tegra_dp_get_bpp(struct tegra_dc_dp_data *dp, u32 vmode)
{
	int yuv_flag = vmode & FB_VMODE_YUV_MASK;

	if (yuv_flag == (FB_VMODE_Y422 | FB_VMODE_Y24)) {
		return 16;
	} else if (yuv_flag & (FB_VMODE_Y422 | FB_VMODE_Y420)) {
		/* YUV 422 non 8bpc and YUV 420 modes are not supported in hw */
		dev_err(&dp->dc->ndev->dev, "%s: Unsupported mode with vmode: 0x%x for DP\n",
				__func__, vmode);
		return 0;
	} else if (yuv_flag & FB_VMODE_Y24) {
		return 24;
	} else if (yuv_flag & FB_VMODE_Y30) {
		return 30;
	} else if (yuv_flag & FB_VMODE_Y36) {
		return 36;
	} else {
		dev_info(&dp->dc->ndev->dev, "%s: vmode=0x%x did not specify bpp\n",
				__func__, vmode);
		return dp->dc->out->depth ? dp->dc->out->depth : 24;
	}
}

static int tegra_dc_init_default_panel_link_cfg(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;

	if (!cfg->is_valid) {
		if (dp->test_max_lanes > 0)
			cfg->max_lane_count = dp->test_max_lanes;
		else
			cfg->max_lane_count = dp->pdata->lanes;

		if (dp->test_max_link_bw > 0)
			cfg->max_link_bw = dp->test_max_link_bw;
		else
			cfg->max_link_bw = dp->pdata->link_bw;
		cfg->tps = TEGRA_DC_DP_TRAINING_PATTERN_2;
		cfg->support_enhanced_framing = true;
		cfg->downspread = true;
		cfg->support_fast_lt = true;
		cfg->aux_rd_interval = 0;
		cfg->alt_scramber_reset_cap = true;
		cfg->only_enhanced_framing = true;
		cfg->edp_cap = true;
		cfg->scramble_ena = 0;
		cfg->lt_data_valid = 0;
	}

	return 0;
}

void tegra_dp_set_max_link_bw(struct tegra_dc_sor_data *sor,
			      struct tegra_dc_dp_link_config *cfg)
{
	unsigned int key; /* Index into the link speed table */

	for (key = sor->num_link_speeds - 1; key; key--)
		if (cfg->max_link_bw >= sor->link_speeds[key].link_rate)
			break;

	cfg->max_link_bw = sor->link_speeds[key].link_rate;
}

static int tegra_dp_init_sink_link_cfg(struct tegra_dc_dp_data *dp,
					struct tegra_dc_dp_link_config *cfg)
{
	u8 dpcd_data = 0;
	int ret = 0;

	cfg->max_lane_count = tegra_dc_dp_get_max_lane_count(dp, &dpcd_data);
	if (cfg->max_lane_count == 0) {
		dev_err(&dp->dc->ndev->dev,
		"dp: Invalid max lane count: %u\n", cfg->max_lane_count);
		return -EINVAL;
	}

	if (dpcd_data & NV_DPCD_MAX_LANE_COUNT_TPS3_SUPPORTED_YES)
		cfg->tps = TEGRA_DC_DP_TRAINING_PATTERN_3;
	else
		cfg->tps = TEGRA_DC_DP_TRAINING_PATTERN_2;

	cfg->support_enhanced_framing =
	(dpcd_data & NV_DPCD_MAX_LANE_COUNT_ENHANCED_FRAMING_YES) ?
	true : false;

	if (dp->sink_cap_valid) {
		dpcd_data = dp->sink_cap[NV_DPCD_MAX_DOWNSPREAD];
	} else {
		ret = tegra_dc_dp_dpcd_read(dp, NV_DPCD_MAX_DOWNSPREAD,
					    &dpcd_data);
		if (ret)
			return ret;
	}

	/*
	 * The check for TPS4 should be after the check for TPS3. That helps
	 * assign a higher priority to TPS4.
	 */
	if (tegra_dc_is_t19x() &&
		(dpcd_data & NV_DPCD_MAX_DOWNSPREAD_TPS4_SUPPORTED_YES))
		cfg->tps = TEGRA_DC_DP_TRAINING_PATTERN_4;

	cfg->downspread =
		(dpcd_data & NV_DPCD_MAX_DOWNSPREAD_VAL_0_5_PCT) ?
		true : false;
	cfg->support_fast_lt = (dpcd_data &
		NV_DPCD_MAX_DOWNSPREAD_NO_AUX_HANDSHAKE_LT_T) ?
		true : false;

	ret = tegra_dc_dp_dpcd_read(dp, NV_DPCD_TRAINING_AUX_RD_INTERVAL,
				    &dpcd_data);
	if (ret)
		return ret;

	cfg->aux_rd_interval = dpcd_data &
				NV_DPCD_TRAINING_AUX_RD_INTERVAL_MASK;

	cfg->max_link_bw = tegra_dc_dp_get_max_link_bw(dp);
	if (cfg->max_link_bw == 0) {
		dev_err(&dp->dc->ndev->dev,
			"dp: Invalid max link bw: %u\n", cfg->max_link_bw);
		return -EINVAL;
	}
	tegra_dp_set_max_link_bw(dp->sor, cfg);

	ret = tegra_dc_dp_dpcd_read(dp, NV_DPCD_EDP_CONFIG_CAP, &dpcd_data);
	if (ret)
		return ret;

	cfg->alt_scramber_reset_cap =
		(dpcd_data & NV_DPCD_EDP_CONFIG_CAP_ASC_RESET_YES) ?
		true : false;
	cfg->only_enhanced_framing = (dpcd_data &
		NV_DPCD_EDP_CONFIG_CAP_FRAMING_CHANGE_YES) ?
		true : false;
	cfg->edp_cap = (dpcd_data &
		NV_DPCD_EDP_CONFIG_CAP_DISPLAY_CONTROL_CAP_YES) ?
		true : false;

	ret = tegra_dc_dp_dpcd_read(dp, NV_DPCD_FEATURE_ENUM_LIST, &dpcd_data);
	if (ret)
		return ret;

	cfg->support_vsc_ext_colorimetry = (dpcd_data &
		NV_DPCD_FEATURE_ENUM_LIST_VSC_EXT_COLORIMETRY) ?
		true : false;

	return 0;
}

static int tegra_dp_init_max_link_cfg(struct tegra_dc_dp_data *dp,
					struct tegra_dc_dp_link_config *cfg)
{
	int ret = 0;

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		tegra_dc_init_default_panel_link_cfg(dp);
	else
		ret = tegra_dp_init_sink_link_cfg(dp, cfg);

	if (ret)
		return ret;

	cfg->bits_per_pixel = tegra_dp_get_bpp(dp, dp->dc->mode.vmode);

	cfg->lane_count = cfg->max_lane_count;

	cfg->link_bw = cfg->max_link_bw;

	cfg->enhanced_framing = cfg->only_enhanced_framing ?
				cfg->support_enhanced_framing :
				(dp->pdata->enhanced_framing_disable ?
				false : cfg->support_enhanced_framing);

	tegra_dc_dp_calc_config(dp, dp->mode, cfg);

	dp->max_link_cfg = *cfg;

	return 0;
}

static int tegra_dc_dp_set_assr(struct tegra_dc_dp_data *dp, bool ena)
{
	int ret;

	u8 dpcd_data = ena ?
		NV_DPCD_EDP_CONFIG_SET_ASC_RESET_ENABLE :
		NV_DPCD_EDP_CONFIG_SET_ASC_RESET_DISABLE;

	ret = tegra_dc_dp_dpcd_write(dp, NV_DPCD_EDP_CONFIG_SET, dpcd_data);
	if (ret)
		return ret;

	/* Also reset the scrambler to 0xfffe */
	tegra_dc_sor_set_internal_panel(dp->sor, ena);
	return 0;
}

static int tegra_dp_set_link_bandwidth(struct tegra_dc_dp_data *dp, u8 link_bw)
{
	tegra_dc_sor_set_link_bandwidth(dp->sor, link_bw);

	/* Sink side */
	return tegra_dc_dp_dpcd_write(dp, NV_DPCD_LINK_BANDWIDTH_SET, link_bw);
}

int tegra_dp_set_enhanced_framing(struct tegra_dc_dp_data *dp, bool enable)
{
	int ret;

	tegra_sor_write_field(dp->sor,
		NV_SOR_DP_LINKCTL(dp->sor->portnum),
		NV_SOR_DP_LINKCTL_ENHANCEDFRAME_ENABLE,
		(enable ? NV_SOR_DP_LINKCTL_ENHANCEDFRAME_ENABLE :
		NV_SOR_DP_LINKCTL_ENHANCEDFRAME_DISABLE));

	ret = tegra_dp_dpcd_write_field(dp, NV_DPCD_LANE_COUNT_SET,
			NV_DPCD_LANE_COUNT_SET_ENHANCEDFRAMING_T,
			(enable ? NV_DPCD_LANE_COUNT_SET_ENHANCEDFRAMING_T :
			NV_DPCD_LANE_COUNT_SET_ENHANCEDFRAMING_F));
	if (ret)
		return ret;

	return 0;
}

static int tegra_dp_set_lane_count(struct tegra_dc_dp_data *dp, u8 lane_cnt)
{
	int ret;

	tegra_sor_power_lanes(dp->sor, lane_cnt, true);

	ret = tegra_dp_dpcd_write_field(dp, NV_DPCD_LANE_COUNT_SET,
					NV_DPCD_LANE_COUNT_SET_MASK,
					lane_cnt);
	if (ret)
		return ret;

	return 0;
}

/*
 * Get the index of a certain link speed in the link speed table that has a
 * given link rate
 */
static inline unsigned int tegra_dp_link_speed_get(struct tegra_dc_dp_data *dp,
						   u8 link_rate)
{
	unsigned int key;

	for (key = 0; key < dp->sor->num_link_speeds; key++)
		if (dp->sor->link_speeds[key].link_rate == link_rate)
			break;

	return key;
}

static void tegra_dp_config_common_prods(struct tegra_dc_dp_data *dp)
{
	if (!IS_ERR_OR_NULL(dp->prod_list)) {
		int err = 0;

		err = tegra_prod_set_by_name(&dp->sor->base, "prod_c_dp",
					dp->prod_list);
		if (err)
			dev_warn(&dp->dc->ndev->dev,
				"dp: prod set failed\n");
	}
}

static void tegra_dp_link_cal(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;
	int err = 0;
	char *prop = NULL;
	unsigned int key; /* Index into the link speed table */

	if (IS_ERR_OR_NULL(dp->prod_list))
		return;

	key = tegra_dp_link_speed_get(dp, cfg->link_bw);
	if (WARN_ON(key == dp->sor->num_link_speeds))
		return;

	prop = dp->sor->link_speeds[key].prod_prop;

	err = tegra_prod_set_by_name(&dp->sor->base, prop, dp->prod_list);
	if (err == -ENODEV) {
		dev_info(&dp->dc->ndev->dev,
			"DP: no %s prod settings node in device tree\n", prop);
	} else if (err) {
		dev_warn(&dp->dc->ndev->dev, "DP : Prod set failed\n");
	}
}

static void tegra_dp_irq_evt_worker(struct work_struct *work)
{
#define LANE0_1_CR_CE_SL_MASK (0x7 | (0x7 << 4))
#define LANE0_CR_CE_SL_MASK (0x7)
#define INTERLANE_ALIGN_MASK (0x1)
#define DPCD_LINK_SINK_STATUS_REGS 6

	struct tegra_dc_dp_data *dp = container_of(to_delayed_work(work),
					struct tegra_dc_dp_data,
					irq_evt_dwork);
	struct tegra_dc *dc = dp->dc;
	struct tegra_dc_dpaux_data *dpaux = dp->dpaux;
	u32 aux_stat;
	bool link_stable = !!true;
	u8 dpcd_200h_205h[DPCD_LINK_SINK_STATUS_REGS] = {0, 0, 0, 0, 0, 0};
	u32 n_lanes = dp->lt_data.n_lanes;

	tegra_dpaux_get(dpaux);
	aux_stat = tegra_dpaux_readl(dpaux, DPAUX_DP_AUXSTAT);
	if (aux_stat & DPAUX_DP_AUXSTAT_SINKSTAT_ERROR_PENDING) {
		int cnt;

		/*
		 * HW failed to automatically read DPCD
		 * offsets 0x200-0x205. Initiate SW transaction.
		 */
		for (cnt = 0; cnt < DPCD_LINK_SINK_STATUS_REGS; cnt++) {
			tegra_dc_dp_dpcd_read(dp, NV_DPCD_SINK_COUNT + cnt,
						&dpcd_200h_205h[cnt]);
		}
	} else  {
		u32 aux_sinkstat_lo = tegra_dpaux_readl(dpaux,
					DPAUX_DP_AUX_SINKSTATLO);
		u32 aux_sinkstat_hi = tegra_dpaux_readl(dpaux,
					DPAUX_DP_AUX_SINKSTATHI);

		dpcd_200h_205h[0] = aux_sinkstat_lo & 0xff;
		dpcd_200h_205h[1] = (aux_sinkstat_lo >> 8) & 0xff;
		dpcd_200h_205h[2] = (aux_sinkstat_lo >> 16) & 0xff;
		dpcd_200h_205h[3] = (aux_sinkstat_lo >> 24) & 0xff;
		dpcd_200h_205h[4] = aux_sinkstat_hi & 0xff;
		dpcd_200h_205h[5] = (aux_sinkstat_hi >> 8) & 0xff;
	}

	switch (n_lanes) {
	case 4:
		link_stable &= !!((dpcd_200h_205h[3] &
				LANE0_1_CR_CE_SL_MASK) ==
				LANE0_1_CR_CE_SL_MASK);
		/* fall through */
	case 2:
		link_stable &= !!((dpcd_200h_205h[2] &
				LANE0_1_CR_CE_SL_MASK) ==
				LANE0_1_CR_CE_SL_MASK);
		/* fall through */
	case 1:
		link_stable &= !!((dpcd_200h_205h[2] &
				LANE0_CR_CE_SL_MASK) ==
				LANE0_CR_CE_SL_MASK);
		/* fall through */
	default:
		link_stable &= !!(dpcd_200h_205h[4] &
				INTERLANE_ALIGN_MASK);
	}

	if (dpcd_200h_205h[1] &
		NV_DPCD_DEVICE_SERVICE_IRQ_VECTOR_AUTO_TEST_YES) {
		enum auto_test_requests test_rq;

		test_rq = tegra_dp_auto_get_test_rq(dp);

		tegra_dp_auto_is_test_supported(test_rq) ?
			tegra_dp_auto_ack_test_rq(dp) :
			tegra_dp_auto_nack_test_rq(dp);

		if (test_rq == TEST_LINK_TRAINING) {
			dp->lt_data.force_trigger = true;
			link_stable = false;
		}
	}

	if (!link_stable) {
		int ret = 0;

		ret = tegra_dc_reserve_common_channel(dc);
		if (ret) {
			dev_err(&dc->ndev->dev,
				"%s: DC %d reserve failed during DP IRQ\n",
				__func__, dc->ctrl_num);

			goto done;
		}
		mutex_lock(&dc->lock);

		tegra_dp_lt_set_pending_evt(&dp->lt_data);
		ret = tegra_dp_lt_wait_for_completion(&dp->lt_data,
						STATE_DONE_PASS, LT_TIMEOUT_MS);

		mutex_unlock(&dc->lock);
		tegra_dc_release_common_channel(dc);

		if (!ret)
			dev_err(&dc->ndev->dev,
				"dp: link training after IRQ failed\n");
	} else {
		dev_info(&dc->ndev->dev,
			"dp: link stable, ignore irq event\n");
	}
done:
	tegra_dpaux_put(dpaux);

#undef LANE0_1_CR_CE_SL_MASK
#undef LANE0_CR_CE_SL_MASK
#undef INTERLANE_ALIGN_MASK
#undef DPCD_LINK_SINK_STATUS_REGS

}

static inline struct tegra_dc_extcon_cable
	*tegra_dp_get_typec_ecable(struct tegra_dc *dc)
{
	if (!dc || !dc->out || !dc->out->dp_out)
		return NULL;

	return &dc->out->dp_out->typec_ecable;
}

static void tegra_dp_wait_for_typec_connect(struct tegra_dc_dp_data *dp)
{
#if KERNEL_VERSION(4, 9, 0) <= LINUX_VERSION_CODE
	struct tegra_dc_extcon_cable *typec_ecable;
	struct tegra_dc *dc;
	union extcon_property_value lane_count = {0};
	int ret;
	bool ecable_connected;

	if (!dp || !dp->dc) {
		pr_err("%s: all arguments must be non-NULL!\n", __func__);
		return;
	}
	dc = dp->dc;

	if (dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return;

	typec_ecable = tegra_dp_get_typec_ecable(dc);
	if (!typec_ecable || !typec_ecable->edev)
		return;

	mutex_lock(&typec_ecable->lock);

	ecable_connected = typec_ecable->connected;
	if (!ecable_connected)
		reinit_completion(&typec_ecable->comp);

	mutex_unlock(&typec_ecable->lock);

	if (!ecable_connected) {
		/*
		 * See the comment above these fields in dp.h for why this is
		 * required.
		 */
		if (unlikely(!dp->typec_notified_once &&
			dp->typec_timed_out_once))
			return;

		if (!wait_for_completion_timeout(&typec_ecable->comp,
						msecs_to_jiffies(1000))) {
			dev_info(&dc->ndev->dev,
				"dp: typec extcon wait timeout\n");
			dp->typec_timed_out_once = true;

			goto typec_lane_count_err;
		}
	}

	ret = extcon_get_property(typec_ecable->edev, EXTCON_DISP_DP,
				EXTCON_PROP_DISP_DP_LANE, &lane_count);
	if (ret) {
		dev_err(&dc->ndev->dev,
			"dp: extcon get lane prop error - ret=%d\n", ret);

		goto typec_lane_count_err;
	}

	if (lane_count.intval > 0)
		dp->typec_lane_count = lane_count.intval;

	return;
typec_lane_count_err:
	dp->typec_lane_count = 4;
#endif
}

static int tegra_dp_typec_ecable_notifier(struct notifier_block *nb,
					unsigned long event, void *data)
{
	struct tegra_dc_extcon_cable *typec_ecable =
			container_of(nb, struct tegra_dc_extcon_cable, nb);
	struct tegra_dc_dp_data *dp =
			(struct tegra_dc_dp_data *)typec_ecable->drv_data;

	/* See the comment above this field in dp.h for why this is required. */
	dp->typec_notified_once = true;

	mutex_lock(&typec_ecable->lock);

	typec_ecable->connected = !!event;
	if (event) /* connected */
		complete(&typec_ecable->comp);

	mutex_unlock(&typec_ecable->lock);

	return NOTIFY_DONE;
}

static int tegra_dp_register_typec_ecable(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc_extcon_cable *typec_ecable;
	int ret;

#if KERNEL_VERSION(4, 9, 0) <= LINUX_VERSION_CODE
	union extcon_property_value lane_count = {0};
	int init_cable_state;
#endif

	if (!dp || !dp->dc) {
		pr_err("%s: all arguments must be non-NULL!\n", __func__);
		return -EINVAL;
	}

	/* Assume that all TX lanes are dedicated for DP by default. */
	dp->typec_lane_count = 4;

	dp->typec_notified_once = false;
	dp->typec_timed_out_once = false;

	typec_ecable = tegra_dp_get_typec_ecable(dp->dc);
	if (!typec_ecable || !typec_ecable->edev)
		return 0;

	mutex_init(&typec_ecable->lock);
	init_completion(&typec_ecable->comp);

	typec_ecable->drv_data = dp;
	typec_ecable->connected = false;

	typec_ecable->nb.notifier_call = tegra_dp_typec_ecable_notifier;
	ret = extcon_register_notifier(typec_ecable->edev, EXTCON_DISP_DP,
					&typec_ecable->nb);
	if (ret < 0) {
		dev_err(&dp->dc->ndev->dev,
			"dp: typec extcon notifier registration failed\n");
		return ret;
	}

#if KERNEL_VERSION(4, 9, 0) <= LINUX_VERSION_CODE
	/*
	 * Query the initial Type-C cable state here in case ucsi_ccg updated it
	 * before we were able to register the extcon notifier.
	 */

	mutex_lock(&typec_ecable->lock);

	init_cable_state = extcon_get_state(typec_ecable->edev, EXTCON_DISP_DP);
	if (init_cable_state < 0) {
		dev_err(&dp->dc->ndev->dev,
			"dp: failed to get initial cable state\n");
	} else if (init_cable_state) { /* connected */
		ret = extcon_get_property(typec_ecable->edev, EXTCON_DISP_DP,
					EXTCON_PROP_DISP_DP_LANE, &lane_count);
		if (ret) {
			dev_err(&dp->dc->ndev->dev,
				"dp: failed to get initial lane count\n");
		} else if (lane_count.intval > 0) {
			typec_ecable->connected = true;
			dp->typec_lane_count = lane_count.intval;
		}
	}

	mutex_unlock(&typec_ecable->lock);
#endif

	return 0;
}

static void tegra_dp_unregister_typec_ecable(struct tegra_dc *dc)
{
	struct tegra_dc_extcon_cable *typec_ecable;
	int ret;

	if (!dc) {
		pr_err("%s: all arguments must be non-NULL!\n", __func__);
		return;
	}

	typec_ecable = tegra_dp_get_typec_ecable(dc);
	if (!typec_ecable || !typec_ecable->edev)
		return;

	ret = extcon_unregister_notifier(typec_ecable->edev, EXTCON_DISP_DP,
					&typec_ecable->nb);
	if (ret < 0)
		dev_err(&dc->ndev->dev,
			"dp: typec extcon notifier unregistration failed\n");

	typec_ecable->drv_data = NULL;
	typec_ecable->connected = false;
}

static irqreturn_t tegra_dp_irq(int irq, void *ptr)
{
	struct tegra_dc_dp_data *dp = ptr;
	struct tegra_dc_dpaux_data *dpaux = dp->dpaux;
	struct tegra_dc *dc = dp->dc;
	u32 status;

	if (!dpaux) {
		dev_err(&dc->ndev->dev,
			"%s: must be non-NULL\n", __func__);
		return IRQ_HANDLED;
	}

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return IRQ_HANDLED;

	if (dp->suspended) {
		dev_info(&dc->ndev->dev,
			"dp: irq received while suspended, ignoring\n");
		return IRQ_HANDLED;
	}

	tegra_dpaux_get(dpaux);

	/* clear pending bits */
	status = tegra_dpaux_readl(dpaux, DPAUX_INTR_AUX);
	tegra_dpaux_writel(dpaux, DPAUX_INTR_AUX, status);

	tegra_dpaux_put(dpaux);

	if (status & (DPAUX_INTR_AUX_PLUG_EVENT_PENDING |
		DPAUX_INTR_AUX_UNPLUG_EVENT_PENDING)) {
		if (status & DPAUX_INTR_AUX_PLUG_EVENT_PENDING) {
			dev_info(&dp->dc->ndev->dev,
				"dp: plug event received\n");
			complete_all(&dp->hpd_plug);
		} else {
			dev_info(&dp->dc->ndev->dev,
				"dp: unplug event received\n");
			reinit_completion(&dp->hpd_plug);
		}
		tegra_dp_pending_hpd(dp);
	} else if (status & DPAUX_INTR_AUX_IRQ_EVENT_PENDING) {
		dev_info(&dp->dc->ndev->dev, "dp: irq event received%s\n",
			dp->enabled ? "" : ", ignoring");
		if (dp->enabled) {
			cancel_delayed_work(&dp->irq_evt_dwork);
			schedule_delayed_work(&dp->irq_evt_dwork,
						msecs_to_jiffies(
						HPD_IRQ_EVENT_TIMEOUT_MS));
		}
	}

	return IRQ_HANDLED;
}

static void tegra_dp_dpaux_init(struct tegra_dc_dp_data *dp)
{
	WARN_ON(!dp || !dp->dc || !dp->dpaux);

	if (dp->sor->safe_clk)
		tegra_sor_safe_clk_enable(dp->sor);

	tegra_dpaux_get(dp->dpaux);

	/* do not enable interrupt for now. */
	tegra_dpaux_writel(dp->dpaux, DPAUX_INTR_EN_AUX, 0x0);

	/* clear interrupt */
	tegra_dpaux_writel(dp->dpaux, DPAUX_INTR_AUX, 0xffffffff);
}

static int tegra_dc_dp_hotplug_init(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);

	/*
	 * dp interrupts are received by dpaux.
	 * Initialize dpaux to receive hotplug events.
	 */
	tegra_dp_dpaux_init(dp);
	tegra_dp_dpaux_enable(dp);

	return 0;
}

static void _tegra_dc_dp_init(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);

	if (dp->pdata->edp2lvds_bridge_enable)
		dp->out_ops = &tegra_edp2lvds_ops;
	else
		dp->out_ops = NULL;

	if (dp->out_ops && dp->out_ops->init)
		dp->out_ops->init(dp);

	if (dp->out_ops && dp->out_ops->enable)
		dp->out_ops->enable(dp);
}

static int tegra_dc_dp_init(struct tegra_dc *dc)
{
	u32 irq;
	int err;
	struct clk *parent_clk;
	struct device_node *sor_np, *panel_np;
	struct tegra_dc_dp_data *dp;

	sor_np = tegra_dc_get_conn_np(dc);
	if (!sor_np) {
		dev_err(&dc->ndev->dev, "%s: error getting connector np\n",
			__func__);
		return -ENODEV;
	}

	panel_np = tegra_dc_get_panel_np(dc);
	if (!panel_np) {
		dev_err(&dc->ndev->dev, "%s: error getting panel np\n",
			__func__);
		return -ENODEV;
	}

	dp = devm_kzalloc(&dc->ndev->dev, sizeof(*dp), GFP_KERNEL);
	if (!dp) {
		err = -ENOMEM;
		goto err_dp_alloc;
	}

	dp->hpd_switch_name = devm_kzalloc(&dc->ndev->dev,
		CHAR_BUF_SIZE_MAX, GFP_KERNEL);
	if (!dp->hpd_switch_name) {
		err = -ENOMEM;
		goto err_free_dp;
	}

	dp->audio_switch_name = devm_kzalloc(&dc->ndev->dev,
		CHAR_BUF_SIZE_MAX, GFP_KERNEL);
	if (!dp->audio_switch_name) {
		err = -ENOMEM;
		goto err_hpd_switch;
	}

	if ((
			((dc->pdata->flags & TEGRA_DC_FLAG_ENABLED) &&
			(dc->pdata->flags & TEGRA_DC_FLAG_SET_EARLY_MODE))
			|| (tegra_fb_is_console_enabled(dc->pdata) &&
			tegra_dc_is_ext_panel(dc))
		) &&
		dc->out->type != TEGRA_DC_OUT_FAKE_DP
	) {
		dp->early_enable = true;
	} else {
		dp->early_enable = false;
	}

	dp->edid_src = EDID_SRC_PANEL;

	if (of_property_read_bool(panel_np, "nvidia,edid"))
		dp->edid_src = EDID_SRC_DT;

	/*
	 * If the new output type is fakeDP and an DPAUX instance from a
	 * previous output type exists, re-use it.
	 */
	if (dc->out->type == TEGRA_DC_OUT_FAKE_DP && dc->out_data
			&& !tegra_dc_is_nvdisplay()) {
		struct tegra_dc_dp_data *dp_copy =
					(struct tegra_dc_dp_data *)dc->out_data;

		if (dp_copy->dpaux)
			dp->dpaux = dp_copy->dpaux;
	}

	if (!dp->dpaux)
		dp->dpaux = tegra_dpaux_init_data(dc, sor_np);

	if (IS_ERR_OR_NULL(dp->dpaux)) {
		err = PTR_ERR(dp->dpaux);
		dev_err(&dc->ndev->dev, "dpaux registers can't be mapped\n");
		dp->dpaux = NULL;
		goto err_audio_switch;
	}

	irq = tegra_dpaux_get_irq(dp->dpaux);
	if (!irq) {
		dev_err(&dc->ndev->dev, "%s: error getting irq\n", __func__);
		err = -ENOENT;
		goto err_audio_switch;
	}

	parent_clk = tegra_disp_of_clk_get_by_name(sor_np, "pll_dp");
	if (IS_ERR_OR_NULL(parent_clk)) {
		dev_err(&dc->ndev->dev, "dp: clock pll_dp unavailable\n");
		err = -EFAULT;
		goto err_audio_switch;
	}
	if (request_threaded_irq(irq, NULL, tegra_dp_irq,
				IRQF_ONESHOT, "tegra_dp", dp)) {
		dev_err(&dc->ndev->dev,
			"dp: request_irq %u failed\n", irq);
		err = -EBUSY;
		goto err_audio_switch;
	}

	if (dc->out->type != TEGRA_DC_OUT_FAKE_DP)
		tegra_dp_disable_irq(irq);

	dp->dc = dc;
	dp->parent_clk = parent_clk;
	dp->mode = &dc->mode;

	err = tegra_dp_register_typec_ecable(dp);
	if (err) {
		dev_err(&dc->ndev->dev, "dp: typec ecable register failed\n");
		goto err_audio_switch;
	}

	if (dp_instance) {
		snprintf(dp->hpd_switch_name, CHAR_BUF_SIZE_MAX,
			"dp%d", dp_instance);
		snprintf(dp->audio_switch_name, CHAR_BUF_SIZE_MAX,
			"dp%d_audio", dp_instance);
	} else {
		snprintf(dp->hpd_switch_name, CHAR_BUF_SIZE_MAX, "dp");
		snprintf(dp->audio_switch_name, CHAR_BUF_SIZE_MAX, "dp_audio");
	}
#ifdef CONFIG_SWITCH
	dp->hpd_data.hpd_switch.name = dp->hpd_switch_name;
	dp->audio_switch.name = dp->audio_switch_name;
#endif

	/*
	 * If the new output type is fakeDP and an SOR instance
	 * from a previous output type exists, re-use it.
	 */
	if (dc->out->type == TEGRA_DC_OUT_FAKE_DP &&
		!tegra_dc_is_nvdisplay() && dc->out_data &&
		((struct tegra_dc_dp_data *)dc->out_data)->sor) {
		dp->sor = ((struct tegra_dc_dp_data *)dc->out_data)->sor;
	} else {
		dp->sor = tegra_dc_sor_init(dc, &dp->link_cfg);
		if (dc->initialized)
			dp->sor->clk_type = TEGRA_SOR_MACRO_CLK;
	}
	dp->irq = irq;
	dp->pdata = dc->pdata->default_out->dp_out;
	dp->suspended = false;

	if (IS_ERR_OR_NULL(dp->sor)) {
		err = PTR_ERR(dp->sor);
		dp->sor = NULL;
		dev_err(&dc->ndev->dev, "%s: error getting sor,%d\n",
				__func__, err);
		goto err_audio_switch;
	}

#ifdef CONFIG_DPHDCP
	if (dp->sor->hdcp_support) {
		dp->dphdcp = tegra_dphdcp_create(dp, dc->ndev->id,
				dc->out->ddc_bus);
		if (IS_ERR_OR_NULL(dp->dphdcp)) {
			err = PTR_ERR(dp->dphdcp);
			dev_err(&dc->ndev->dev,
				"dp hdcp creation failed with err %d\n", err);
		} else {
			/* create a /d entry to change the max retries */
			tegra_dphdcp_debugfs_init(dp->dphdcp);
		}
	}
#endif

	if (!tegra_platform_is_sim()) {
		dp->prod_list = devm_tegra_prod_get_from_node(
				&dc->ndev->dev, sor_np);
		if (IS_ERR(dp->prod_list)) {
			dev_warn(&dc->ndev->dev, "%s: error getting prod-list\n",
					__func__);
			dp->prod_list = NULL;
		}
	}

	init_completion(&dp->hpd_plug);

	tegra_dc_set_outdata(dc, dp);

	_tegra_dc_dp_init(dc);

	if (dp->pdata->hdmi2fpd_bridge_enable) {
		hdmi2fpd_init(dc);
		hdmi2fpd_enable(dc);
	}

	/*
	 * Adding default link configuration at init. Since
	 * we check for max link bandwidth during modeset,
	 * this addresses usecases where modeset happens
	 * before unblank without preset default configuration
	 */
	tegra_dc_init_default_panel_link_cfg(dp);

	/*
	 * We don't really need hpd driver for eDP.
	 * Nevertheless, go ahead and init hpd driver.
	 * eDP uses some of its fields to interact with panel.
	 */
	tegra_hpd_init(&dp->hpd_data, dc, dp, &hpd_ops);

	tegra_dp_lt_init(&dp->lt_data, dp);

	INIT_DELAYED_WORK(&dp->irq_evt_dwork, tegra_dp_irq_evt_worker);
#ifdef CONFIG_DEBUG_FS
	dp->test_settings = default_dp_test_settings;
#endif

#ifdef CONFIG_SWITCH
	if (tegra_dc_is_ext_panel(dc)) {
		err = switch_dev_register(&dp->hpd_data.hpd_switch);
		if (err)
			dev_err(&dc->ndev->dev,
				"%s: failed to register hpd switch, err=%d\n",
				__func__, err);
	}

	if (tegra_dc_is_ext_panel(dc) &&
		dc->out->type != TEGRA_DC_OUT_FAKE_DP) {
		err = switch_dev_register(&dp->audio_switch);
		if (err)
			dev_err(&dc->ndev->dev,
				"%s: failed to register audio switch, err=%d\n",
				__func__, err);
	}
#endif

#ifdef CONFIG_TEGRA_HDA_DC
	if (tegra_dc_is_ext_panel(dc) && dp->sor->audio_support)
		tegra_hda_init(dc, dp);
#endif

	if (!(dc->mode.pclk) && IS_ENABLED(CONFIG_FRAMEBUFFER_CONSOLE)) {
		if (dc->out->fbcon_default_mode)
			tegra_dc_set_fb_mode(dc,
					dc->out->fbcon_default_mode, false);
		else
			tegra_dc_set_fb_mode(dc, &tegra_dc_vga_mode, false);
	}

	tegra_dc_dp_debugfs_create(dp);
	dp_instance++;

	return 0;

err_audio_switch:
	devm_kfree(&dc->ndev->dev, dp->audio_switch_name);
err_hpd_switch:
	devm_kfree(&dc->ndev->dev, dp->hpd_switch_name);
err_free_dp:
	devm_kfree(&dc->ndev->dev, dp);
err_dp_alloc:
	return err;
}

static void tegra_dp_hpd_config(struct tegra_dc_dp_data *dp)
{
#define TEGRA_DP_HPD_UNPLUG_MIN_US	2000
#define TEGRA_DP_HPD_PLUG_MIN_US	250
#define TEGRA_DP_HPD_IRQ_MIN_US		250

	struct tegra_dc_dpaux_data *dpaux = dp->dpaux;
	u32 val;

	val = TEGRA_DP_HPD_PLUG_MIN_US |
		(TEGRA_DP_HPD_UNPLUG_MIN_US <<
		DPAUX_HPD_CONFIG_UNPLUG_MIN_TIME_SHIFT);
	tegra_dpaux_writel(dpaux, DPAUX_HPD_CONFIG, val);

	tegra_dpaux_writel(dpaux, DPAUX_HPD_IRQ_CONFIG,
						TEGRA_DP_HPD_IRQ_MIN_US);

#undef TEGRA_DP_HPD_IRQ_MIN_US
#undef TEGRA_DP_HPD_PLUG_MIN_US
#undef TEGRA_DP_HPD_UNPLUG_MIN_US
}

static int tegra_dp_dpcd_init(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;
	int ret;
	u32 size_ieee_oui = 3, auxstat;
	u8 data_ieee_oui_be[3] = {(NV_IEEE_OUI >> 16) & 0xff,
		(NV_IEEE_OUI >> 8) & 0xff,
		NV_IEEE_OUI & 0xff};

	/* Check DP version */
	if (tegra_dc_dp_dpcd_read(dp, NV_DPCD_REV, &dp->revision))
		dev_err(&dp->dc->ndev->dev,
			"dp: failed to read the revision number from sink\n");

	ret = tegra_dp_init_max_link_cfg(dp, cfg);
	if (ret) {
		dev_err(&dp->dc->ndev->dev, "dp: failed to init link cfg\n");
		return ret;
	}

	tegra_dc_dpaux_write(dp->dpaux, DPAUX_DP_AUXCTL_CMD_AUXWR,
		NV_DPCD_SOURCE_IEEE_OUI, data_ieee_oui_be, &size_ieee_oui,
		&auxstat);

	return 0;
}

void tegra_dp_tpg(struct tegra_dc_dp_data *dp, u32 tp, u32 n_lanes)
{
	tegra_sor_tpg(dp->sor, tp, n_lanes);
	tegra_dc_dp_dpcd_write(dp, NV_DPCD_TRAINING_PATTERN_SET,
		dp->sor->training_patterns[tp].dpcd_val);
}

static void tegra_dp_tu_config(struct tegra_dc_dp_data *dp,
				const struct tegra_dc_dp_link_config *cfg)
{
	struct tegra_dc_sor_data *sor = dp->sor;
	u32 reg_val;

	tegra_sor_write_field(sor, NV_SOR_DP_LINKCTL(sor->portnum),
			NV_SOR_DP_LINKCTL_TUSIZE_MASK,
			(cfg->tu_size << NV_SOR_DP_LINKCTL_TUSIZE_SHIFT));

	tegra_sor_write_field(sor, NV_SOR_DP_CONFIG(sor->portnum),
				NV_SOR_DP_CONFIG_WATERMARK_MASK,
				cfg->watermark);

	tegra_sor_write_field(sor, NV_SOR_DP_CONFIG(sor->portnum),
				NV_SOR_DP_CONFIG_ACTIVESYM_COUNT_MASK,
				(cfg->active_count <<
				NV_SOR_DP_CONFIG_ACTIVESYM_COUNT_SHIFT));

	tegra_sor_write_field(sor, NV_SOR_DP_CONFIG(sor->portnum),
				NV_SOR_DP_CONFIG_ACTIVESYM_FRAC_MASK,
				(cfg->active_frac <<
				NV_SOR_DP_CONFIG_ACTIVESYM_FRAC_SHIFT));

	reg_val = cfg->activepolarity ?
		NV_SOR_DP_CONFIG_ACTIVESYM_POLARITY_POSITIVE :
		NV_SOR_DP_CONFIG_ACTIVESYM_POLARITY_NEGATIVE;
	tegra_sor_write_field(sor, NV_SOR_DP_CONFIG(sor->portnum),
				NV_SOR_DP_CONFIG_ACTIVESYM_POLARITY_POSITIVE,
				reg_val);

	tegra_sor_write_field(sor, NV_SOR_DP_CONFIG(sor->portnum),
				NV_SOR_DP_CONFIG_ACTIVESYM_CNTL_ENABLE,
				NV_SOR_DP_CONFIG_ACTIVESYM_CNTL_ENABLE);

	tegra_sor_write_field(sor, NV_SOR_DP_CONFIG(sor->portnum),
				NV_SOR_DP_CONFIG_RD_RESET_VAL_NEGATIVE,
				NV_SOR_DP_CONFIG_RD_RESET_VAL_NEGATIVE);
}

void tegra_dp_update_link_config(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;

	tegra_dp_set_link_bandwidth(dp, cfg->link_bw);
	tegra_dp_set_lane_count(dp, cfg->lane_count);
	tegra_dp_link_cal(dp);
	tegra_dp_tu_config(dp, cfg);
}

static void tegra_dp_read_sink_cap(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc *dc = dp->dc;
	u32 sink_cap_rd_size = DP_DPCD_SINK_CAP_SIZE;
	u32 aux_stat = 0;
	u8 start_offset = 0;
	int err;

	tegra_dc_io_start(dc);

	dp->sink_cap_valid = false;

	err = tegra_dc_dpaux_read(dp->dpaux, DPAUX_DP_AUXCTL_CMD_AUXRD,
				start_offset, dp->sink_cap, &sink_cap_rd_size,
				&aux_stat);
	if (!err)
		dp->sink_cap_valid = true;

	tegra_dc_io_end(dc);
}

static void tegra_dp_hpd_op_edid_ready(void *drv_data)
{
	struct tegra_dc_dp_data *dp = drv_data;
	struct tegra_dc *dc = dp->dc;

	/*
	 * we have a new panel connected.
	 * Forget old LT config data.
	 */
	tegra_dp_lt_invalidate(&dp->lt_data);

	/* in mm */
	dc->out->h_size = dc->out->h_size ? : dp->hpd_data.mon_spec.max_x * 10;
	dc->out->v_size = dc->out->v_size ? : dp->hpd_data.mon_spec.max_y * 10;

	/*
	 * EDID specifies either the acutal screen sizes or
	 * the aspect ratios. The panel file can choose to
	 * trust the value as the actual sizes by leaving
	 * width/height to 0s
	 */
	dc->out->width = dc->out->width ? : dc->out->h_size;
	dc->out->height = dc->out->height ? : dc->out->v_size;

	tegra_dp_read_sink_cap(dp);
	tegra_dc_dp_read_ext_dpcd_caps(dp, &dp->link_cfg.ext_dpcd_caps);

	tegra_dc_io_start(dc);
	tegra_dc_dp_dpcd_read(dp, NV_DPCD_SINK_COUNT,
				&dp->sink_cnt_cp_ready);

	if (tegra_dp_auto_is_rq(dp)) {
		enum auto_test_requests test_rq;

		test_rq = tegra_dp_auto_get_test_rq(dp);

		tegra_dp_auto_is_test_supported(test_rq) ?
			tegra_dp_auto_ack_test_rq(dp) :
			tegra_dp_auto_nack_test_rq(dp);

		if (test_rq == TEST_EDID_READ)
			tegra_dp_auto_set_edid_checksum(dp);
	}

	/*
	 * For Type-C, wait until ucsi_ccg notifies us that an extcon CONNECT
	 * event has occurred. This ensures that we're in DP configuration, and
	 * that the EXTCON_PROP_DISP_DP_LANE property has been populated by
	 * ucsi_ccg before we proceed with our subsequent mode filtering.
	 */
	tegra_dp_wait_for_typec_connect(dp);

	/* Early enables DC with first mode from the monitor specs */
	if (dp->early_enable) {
		struct tegra_hpd_data *data = &dp->hpd_data;
		struct fb_videomode *target_videomode;
		struct fb_var_screeninfo var;

		/* This function is called only when EDID is read
		 * successfully. target_videomode should never set
		 * to default VGA mode unless unexpected issue
		 * happens and first mode was a null pointer.
		 */
		target_videomode = (data->mon_spec.modedb) ?
			data->mon_spec.modedb : &tegra_dc_vga_mode;
		memset(&var, 0x0, sizeof(var));
		fb_videomode_to_var(&var, target_videomode);
		var.bits_per_pixel = dc->pdata->fb->bits_per_pixel;
		tegra_fb_set_var(dc, &var);
		if (!dp->dc->enabled)
			tegra_dc_enable(dp->dc);
		dp->early_enable = false;
		if (tegra_fb_is_console_enabled(dc->pdata)) {
			tegra_fb_update_monspecs(dc->fb,
				&dp->hpd_data.mon_spec,
				tegra_dc_dp_ops.mode_filter);
		}
	}
	tegra_dc_io_end(dc);
}

static void tegra_dp_hpd_op_edid_recheck(void *drv_data)
{
	struct tegra_dc_dp_data __maybe_unused *dp = drv_data;

	/*
	 * If we ever encounter panel which sends unplug event
	 * to indicate synchronization loss, this is the placeholder.
	 * As per specification, panel is expected to send irq_event to
	 * indicate synchronization loss to host.
	 */
}

static inline void tegra_dp_default_int(struct tegra_dc_dp_data *dp,
					bool enable)
{
	struct tegra_dc_dpaux_data *dpaux = dp->dpaux;

	if (dp->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return;

	if (enable)
		tegra_dpaux_int_toggle(dpaux, DPAUX_INTR_EN_AUX_IRQ_EVENT |
				DPAUX_INTR_EN_AUX_PLUG_EVENT |
				DPAUX_INTR_EN_AUX_UNPLUG_EVENT,
				true);
	else
		tegra_dpaux_int_toggle(dpaux, DPAUX_INTR_EN_AUX_IRQ_EVENT,
				false);
}

static int tegra_edp_edid_read(struct tegra_dc_dp_data *dp)
{
	struct tegra_hpd_data *data = &dp->hpd_data;

	BUG_ON(!data);

	memset(&data->mon_spec, 0, sizeof(data->mon_spec));

	return tegra_edid_get_monspecs(data->edid, &data->mon_spec);
}

static void tegra_edp_mode_set(struct tegra_dc_dp_data *dp)
{
	struct fb_videomode *best_edp_fbmode = dp->hpd_data.mon_spec.modedb;

	if (best_edp_fbmode)
		tegra_dc_set_fb_mode(dp->dc, best_edp_fbmode, false);
	else
		tegra_dc_set_default_videomode(dp->dc);
}

static int tegra_edp_wait_plug_hpd(struct tegra_dc_dp_data *dp)
{
#define TEGRA_DP_HPD_PLUG_TIMEOUT_MS	1000

	u32 val;
	int err = 0;

	might_sleep();

	if (!tegra_platform_is_silicon()) {
		msleep(TEGRA_DP_HPD_PLUG_TIMEOUT_MS);
		return 0;
	}

	val = tegra_dpaux_readl(dp->dpaux, DPAUX_DP_AUXSTAT);
	if (likely(val & DPAUX_DP_AUXSTAT_HPD_STATUS_PLUGGED))
		err = 0;
	else if (!wait_for_completion_timeout(&dp->hpd_plug,
		msecs_to_jiffies(TEGRA_DP_HPD_PLUG_TIMEOUT_MS)))
		err = -ENODEV;

	return err;

#undef TEGRA_DP_HPD_PLUG_TIMEOUT_MS
}

#define VSC_PKT_ID (0x07)
#define VSC_REV (0x05)
#define VSC_N_VALID_DATA_BYTES (0x13)
static void tegra_dp_vsc_col_ext_header(struct tegra_dc_dp_data *dp)
{
	u32 val = (VSC_N_VALID_DATA_BYTES << 24) |
		(VSC_REV << 16) | (VSC_PKT_ID << 8);

	tegra_sor_writel(dp->sor, NV_SOR_DP_GENERIC_INFOFRAME_HEADER, val);
}
#undef VSC_N_VALID_DATA_BYTES
#undef VSC_REV
#undef VSC_PKT_ID

static void tegra_dp_vsc_col_ext_payload(struct tegra_dc_dp_data *dp,
					u8 vsc_pix_encoding, u8 colorimetry,
					u8 dynamic_range, u8 bpc,
					u8 content_type)
{
	u8 db16 = 0;
	u8 db17 = 0;
	u8 db18 = 0;
	struct tegra_dc_sor_data *sor = dp->sor;

	db16 = (vsc_pix_encoding << 4) | colorimetry;
	db17 = (dynamic_range << 7) | bpc;
	db18 = content_type;

	tegra_sor_writel(sor, NV_SOR_DP_GENERIC_INFOFRAME_SUBPACK(0), 0);
	tegra_sor_writel(sor, NV_SOR_DP_GENERIC_INFOFRAME_SUBPACK(1), 0);
	tegra_sor_writel(sor, NV_SOR_DP_GENERIC_INFOFRAME_SUBPACK(2), 0);
	tegra_sor_writel(sor, NV_SOR_DP_GENERIC_INFOFRAME_SUBPACK(3), 0);
	tegra_sor_writel(sor, NV_SOR_DP_GENERIC_INFOFRAME_SUBPACK(4),
			(db18 << 16) | (db17 << 8) | db16);
	tegra_sor_writel(sor, NV_SOR_DP_GENERIC_INFOFRAME_SUBPACK(5), 0);
	tegra_sor_writel(sor, NV_SOR_DP_GENERIC_INFOFRAME_SUBPACK(6), 0);
}

static int tegra_dp_vsc_col_ext_enable(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc_sor_data *sor = dp->sor;
	unsigned long ret;
	u32 nv_sor_dp_misc1_override_reg = nv_sor_dp_misc1_override();

	ret = tegra_dc_sor_poll_register(sor, nv_sor_dp_misc1_override_reg,
					NV_SOR_DP_MISC1_OVERRIDE_CNTL_TRIGGER,
					NV_SOR_DP_MISC1_OVERRIDE_CNTL_DONE,
					100, TEGRA_SOR_TIMEOUT_MS);
	if (!ret) {
		tegra_sor_writel(sor, nv_sor_dp_misc1_bit6(),
				NV_SOR_DP_MISC1_BIT6_0_SET);
		tegra_sor_writel(sor, nv_sor_dp_misc1_override_reg,
				NV_SOR_DP_MISC1_OVERRIDE_CNTL_TRIGGER |
				NV_SOR_DP_MISC1_OVERRIDE_ENABLE);

		tegra_sor_write_field(sor, NV_SOR_DP_AUDIO_CTRL,
			NV_SOR_DP_AUDIO_CTRL_GENERIC_INFOFRAME_ENABLE |
			NV_SOR_DP_AUDIO_CTRL_NEW_SETTINGS_TRIGGER,
			NV_SOR_DP_AUDIO_CTRL_GENERIC_INFOFRAME_ENABLE |
			NV_SOR_DP_AUDIO_CTRL_NEW_SETTINGS_TRIGGER);
	}

	return !ret ? 0 : -ETIMEDOUT;
}

static int tegra_dp_vsc_col_ext_disable(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc_sor_data *sor = dp->sor;
	unsigned long ret;
	u32 nv_sor_dp_misc1_override_reg = nv_sor_dp_misc1_override();

	ret = tegra_dc_sor_poll_register(sor, nv_sor_dp_misc1_override_reg,
					NV_SOR_DP_MISC1_OVERRIDE_CNTL_TRIGGER,
					NV_SOR_DP_MISC1_OVERRIDE_CNTL_DONE,
					100, TEGRA_SOR_TIMEOUT_MS);
	if (!ret) {
		tegra_sor_writel(sor, nv_sor_dp_misc1_override_reg,
				NV_SOR_DP_MISC1_OVERRIDE_CNTL_TRIGGER |
				NV_SOR_DP_MISC1_OVERRIDE_DISABLE);

		tegra_sor_write_field(sor, NV_SOR_DP_AUDIO_CTRL,
			NV_SOR_DP_AUDIO_CTRL_GENERIC_INFOFRAME_ENABLE |
			NV_SOR_DP_AUDIO_CTRL_NEW_SETTINGS_TRIGGER,
			NV_SOR_DP_AUDIO_CTRL_GENERIC_INFOFRAME_DISABLE |
			NV_SOR_DP_AUDIO_CTRL_NEW_SETTINGS_TRIGGER);
	}

	return !ret ? 0 : -ETIMEDOUT;
}

static inline u8 tegra_dp_vsc_get_bpc(struct tegra_dc_dp_data *dp)
{
	int yuv_flag = dp->dc->mode.vmode & FB_VMODE_YUV_MASK;
	u8 bpc = VSC_8BPC;

	if (yuv_flag & FB_VMODE_Y24) {
		bpc = VSC_8BPC;
	} else if (yuv_flag & FB_VMODE_Y30) {
		bpc = VSC_10BPC;
	} else if (yuv_flag & FB_VMODE_Y36) {
		bpc = VSC_12BPC;
	} else if (yuv_flag & FB_VMODE_Y48) {
		bpc = VSC_16BPC;
	} else {
		switch (dp->dc->out->depth) {
		case 18:
			bpc = VSC_6BPC;
			break;
		case 30:
			bpc = VSC_10BPC;
			break;
		case 36:
			bpc = VSC_12BPC;
			break;
		case 48:
			bpc = VSC_16BPC;
			break;
		case 24:
		default:
			bpc = VSC_8BPC;
			break;
		}
	};

	return bpc;
}

static inline u8 tegra_dp_vsc_get_pixel_encoding(struct tegra_dc_dp_data *dp)
{
	int yuv_flag = dp->dc->mode.vmode & FB_VMODE_YUV_MASK;

	if (yuv_flag & FB_VMODE_Y422)
		return VSC_YUV422;
	else if (yuv_flag & FB_VMODE_Y444)
		return VSC_YUV444;
	else if (IS_RGB(yuv_flag))
		return VSC_RGB;

	return VSC_RGB;
}

static inline u8 tegra_dp_vsc_get_dynamic_range(struct tegra_dc_dp_data *dp)
{
	if ((dp->dc->mode.vmode & FB_VMODE_BYPASS) ||
			!(dp->dc->mode.vmode & FB_VMODE_LIMITED_RANGE))
		return VSC_VESA_RANGE;

	return VSC_CEA_RANGE;
}

static inline u8 tegra_dp_vsc_get_colorimetry(struct tegra_dc_dp_data *dp)
{
	u32 vmode_flag = dp->dc->mode.vmode;
	u8 colorimetry = VSC_RGB_SRGB;

	if (vmode_flag & FB_VMODE_EC_ENABLE) {
		u32 ec = vmode_flag & FB_VMODE_EC_MASK;

		switch (ec) {
		case FB_VMODE_EC_ADOBE_RGB:
			colorimetry = VSC_RGB_ADOBERGB;
			break;
		case FB_VMODE_EC_ADOBE_YCC601:
			colorimetry = VSC_YUV_ADOBEYCC601;
			break;
		case FB_VMODE_EC_SYCC601:
			colorimetry = VSC_YUV_SYCC601;
			break;
		case FB_VMODE_EC_XVYCC601:
			colorimetry = VSC_YUV_XVYCC709;
			break;
		case FB_VMODE_EC_XVYCC709:
			colorimetry = VSC_YUV_XVYCC709;
			break;
		default:
			colorimetry = VSC_RGB_SRGB;
			break;
		}
	}

	return colorimetry;
}

static void tegra_dp_vsc_col_ext(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;
	u8 vsc_pix_encoding = 0, colorimetry = 0, dynamic_range = 0,
			bpc = 0, content_type = 0;
	u32 vmode_flag = dp->dc->mode.vmode;
	u32 ec = vmode_flag & FB_VMODE_EC_MASK;

	if (!tegra_dc_is_nvdisplay() || !cfg->support_vsc_ext_colorimetry)
		return;

	if (!(vmode_flag & FB_VMODE_Y420) &&
		!(ec & (FB_VMODE_EC_BT2020_CYCC | FB_VMODE_EC_BT2020_YCC_RGB)))
		return;

	tegra_dp_vsc_col_ext_disable(dp);
	vsc_pix_encoding = tegra_dp_vsc_get_pixel_encoding(dp);
	colorimetry = tegra_dp_vsc_get_colorimetry(dp);
	dynamic_range = tegra_dp_vsc_get_dynamic_range(dp);
	bpc = tegra_dp_vsc_get_bpc(dp);
	content_type = VSC_CONTENT_TYPE_DEFAULT;

	tegra_dp_vsc_col_ext_header(dp);

	tegra_dp_vsc_col_ext_payload(dp, vsc_pix_encoding,
				colorimetry, dynamic_range,
				bpc, content_type);

	tegra_dp_vsc_col_ext_enable(dp);
}

static inline void tegra_dp_set_sor_clk_src(struct tegra_dc_dp_data *dp,
					struct clk *src)
{
	struct tegra_dc_sor_data *sor = dp->sor;

	/*
	 * Disable and re-enable the sor_clk while switching the source to avoid
	 * any momentary glitches. This shouldn't really matter since the SOR
	 * wouldn't be actively sending any data at this point in time, but
	 * we're doing this to be safe.
	 */
	tegra_disp_clk_disable_unprepare(sor->sor_clk);
	clk_set_parent(sor->sor_clk, src);
	tegra_disp_clk_prepare_enable(sor->sor_clk);
}

static void tegra_dp_prepare_pad(struct tegra_dc_dp_data *dp)
{
	struct tegra_dc *dc = dp->dc;
	struct tegra_dc_sor_data *sor = dp->sor;

	if (!dc->initialized) {
		tegra_sor_reset(sor);

		/*
		 * Enable PLLDP and the PLLD* pixel clock reference.
		 * Select the SOR safe clock before powering up the pads.
		 *
		 * For nvdisplay, the above steps are performed as part of
		 * out_ops->setup_clk(), which is invoked during the head enable
		 * sequence prior to this point.
		 */
		if (!tegra_dc_is_nvdisplay()) {
			tegra_dp_clk_enable(dp);
			tegra_dc_setup_clk(dc, dc->clk);
			tegra_sor_config_safe_clk(sor);
		}
	}

	tegra_sor_clk_enable(sor);

	if (!dc->initialized) {
		tegra_sor_write_field(sor, NV_SOR_CLK_CNTRL,
			NV_SOR_CLK_CNTRL_DP_CLK_SEL_MASK,
			NV_SOR_CLK_CNTRL_DP_CLK_SEL_DIFF_DPCLK);
		tegra_dc_sor_set_link_bandwidth(sor, dp->link_cfg.link_bw);

		/* Program common and linkspeed-specific PROD settings. */
		tegra_dp_config_common_prods(dp);
		tegra_dp_link_cal(dp);
	}
}

static void tegra_dc_dp_enable(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);
	struct tegra_dc_dp_link_config *cfg = &dp->link_cfg;
	struct tegra_dc_sor_data *sor = dp->sor;
	int ret;

	if (dp->enabled)
		return;

	tegra_dc_io_start(dc);

	if (tegra_platform_is_fpga())
		tegra_sor_program_fpga_clk_mux(sor);

	/* Change for seamless */
	if (!dc->initialized) {
		ret = tegra_dp_panel_power_state(dp,
					NV_DPCD_SET_POWER_VAL_D0_NORMAL);
		if (ret < 0) {
			dev_err(&dp->dc->ndev->dev,
			"dp: failed to exit panel power save mode (0x%x)\n",
			ret);
			tegra_dc_io_end(dp->dc);
			return;
		}
	}

	/* For eDP, driver gets to decide the best mode. */
	if (!tegra_dc_is_ext_panel(dc) &&
		dc->out->type != TEGRA_DC_OUT_FAKE_DP) {
		int err;

		/*
		 * Hotplug for internal panels is not supported.
		 * Wait till the panel asserts hpd
		 */
		err = tegra_edp_wait_plug_hpd(dp);
		if (err < 0) {
			tegra_dc_io_end(dc);
			dc->connected = false;
			dev_err(&dc->ndev->dev,
				"edp: plug hpd wait timeout\n");
			return;
		}

		err = tegra_edp_edid_read(dp);
		if (err < 0)
			dev_warn(&dc->ndev->dev, "edp: edid read failed\n");
		else
			tegra_dp_hpd_op_edid_ready(dp);
		tegra_edp_mode_set(dp);
		tegra_dc_setup_clk(dc, dc->clk);
	}

	if (tegra_dp_dpcd_init(dp)) {
		dev_err(&dp->dc->ndev->dev, "dp: failed dpcd init\n");
		return;
	}

	tegra_dp_prepare_pad(dp);
	tegra_dc_sor_enable_dp(dp->sor);

	if (cfg->alt_scramber_reset_cap)
		tegra_dc_dp_set_assr(dp, true);
	else
		tegra_dc_sor_set_internal_panel(dp->sor, false);

	tegra_dc_dp_dpcd_write(dp, NV_DPCD_MAIN_LINK_CHANNEL_CODING_SET,
			NV_DPCD_MAIN_LINK_CHANNEL_CODING_SET_ANSI_8B10B);

	if (!dc->initialized) {
		tegra_sor_write_field(sor, NV_SOR_DP_CONFIG(sor->portnum),
				NV_SOR_DP_CONFIG_IDLE_BEFORE_ATTACH_ENABLE,
				NV_SOR_DP_CONFIG_IDLE_BEFORE_ATTACH_ENABLE);

		tegra_dc_dp_dpcd_write(dp, NV_DPCD_DOWNSPREAD_CTRL,
				NV_DPCD_DOWNSPREAD_CTRL_SPREAD_AMP_LT_0_5);

		tegra_dc_dp_dpcd_write(dp, NV_DPCD_LINK_BANDWIDTH_SET,
				cfg->link_bw);

		/*
		 * enhanced framing enable field shares DPCD offset
		 * with lane count set field. Make sure lane count is set
		 * before enhanced framing enable. CTS waits on first
		 * write to this offset to check for lane count set.
		 */
		tegra_dp_dpcd_write_field(dp, NV_DPCD_LANE_COUNT_SET,
					NV_DPCD_LANE_COUNT_SET_MASK,
					cfg->lane_count);

		tegra_dp_set_enhanced_framing(dp, cfg->enhanced_framing);

		tegra_dp_tu_config(dp, cfg);
	}

	tegra_sor_port_enable(sor, true);
	tegra_sor_config_xbar(dp->sor);

	/* Select the macro feedback clock. */
	if (!dc->initialized) {
		if (tegra_dc_is_nvdisplay()) {
			tegra_sor_clk_switch_setup(sor, true);
			tegra_dp_set_sor_clk_src(dp, sor->pad_clk);
		} else {
			tegra_sor_config_dp_clk_t21x(sor);
		}
	}

	/* Host is ready. Start link training. */
	dp->enabled = true;

#ifdef CONFIG_TEGRA_HDA_DC
	if (tegra_dc_is_ext_panel(dc) && sor->audio_support)
		tegra_hda_enable(dp->hda_handle);
#endif

	tegra_dp_vsc_col_ext(dp);

	if (likely(dc->out->type != TEGRA_DC_OUT_FAKE_DP) &&
		!no_lt_at_unblank) {
		if (!dc->initialized) {
			tegra_dp_lt_set_pending_evt(&dp->lt_data);
			ret = tegra_dp_lt_wait_for_completion(&dp->lt_data,
						STATE_DONE_PASS, LT_TIMEOUT_MS);
			if (!ret)
				dev_err(&dp->dc->ndev->dev,
					"dp: link training failed\n");
		} else {
			/* Perform SOR attach here */
			tegra_dc_sor_attach(dp->sor);
		}
	} else {
		/* Just enable host. */
		tegra_dp_tpg(dp, TEGRA_DC_DP_TRAINING_PATTERN_DISABLE,
			      dp->link_cfg.lane_count);
		tegra_dc_sor_attach(dp->sor);
	}
#ifdef CONFIG_DPHDCP
	if (tegra_dc_is_ext_panel(dc) && dp->dphdcp &&
		dc->out->type != TEGRA_DC_OUT_FAKE_DP) {
		tegra_dphdcp_set_plug(dp->dphdcp, true);
	}
#endif
	dc->connected = true;
	tegra_dc_io_end(dc);

	if (tegra_dp_is_audio_supported(dp)) {
		disp_state_extcon_aux_report(dp->sor->ctrl_num,
			EXTCON_DISP_AUX_STATE_ENABLED);
		pr_info("Extcon AUX%d(DP): enable\n", dp->sor->ctrl_num);
	}

#ifdef CONFIG_SWITCH
	if (tegra_edid_audio_supported(dp->hpd_data.edid)
				&& tegra_dc_is_ext_panel(dc) &&
				dc->out->type != TEGRA_DC_OUT_FAKE_DP) {
		pr_info("dp_audio switch 1\n");
		switch_set_state(&dp->audio_switch, 1);
	}
#endif
}

void tegra_dc_dp_enable_link(struct tegra_dc_dp_data *dp)
{
	if (!dp->enabled)
		tegra_dc_dp_enable(dp->dc);
	else
		tegra_dc_sor_attach(dp->sor);
}

static void tegra_dc_dp_destroy(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = NULL;

	if (!dc->current_topology.valid)
		return;

	dp = tegra_dc_get_outdata(dc);

	if (dp->dc->out->type != TEGRA_DC_OUT_FAKE_DP) {
		tegra_dp_disable_irq(dp->irq);
		tegra_dp_default_int(dp, false);
	}

	if (dp->pdata->hdmi2fpd_bridge_enable)
		hdmi2fpd_destroy(dc);

#ifdef CONFIG_TEGRA_HDA_DC
	if (tegra_dc_is_ext_panel(dc) && dp->sor->audio_support)
		tegra_hda_destroy(dp->hda_handle);
#endif

	if (dp->dphdcp)
		tegra_dphdcp_destroy(dp->dphdcp);

	tegra_dp_dpaux_disable(dp);
	if (dp->dpaux)
		tegra_dpaux_destroy_data(dp->dpaux);

	if (dp->sor)
		tegra_dc_sor_destroy(dp->sor);

	tegra_hpd_shutdown(&dp->hpd_data);

	clk_put(dp->parent_clk);

	dp->prod_list = NULL;

	tegra_dp_unregister_typec_ecable(dc);

	tegra_dc_out_destroy(dc);

	tegra_dc_dp_debugfs_remove(dp);

#ifdef CONFIG_SWITCH
	if (tegra_dc_is_ext_panel(dc) &&
			dc->out->type != TEGRA_DC_OUT_FAKE_DP) {
		switch_dev_unregister(&dp->audio_switch);
	}
#endif

	devm_kfree(&dc->ndev->dev, dp->hpd_switch_name);
	devm_kfree(&dc->ndev->dev, dp->audio_switch_name);
	free_irq(dp->irq, dp);
	devm_kfree(&dc->ndev->dev, dp);
	dc->current_topology.valid = false;
}

static void tegra_dc_dp_disable(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);
	int ret;

	if (!dp->enabled)
		return;

	dp->enabled = false;

	tegra_dc_io_start(dc);

#ifdef CONFIG_DPHDCP
	if (tegra_dc_is_ext_panel(dc) && dp->dphdcp &&
		dc->out->type != TEGRA_DC_OUT_FAKE_DP)
		tegra_dphdcp_set_plug(dp->dphdcp, false);
#endif

	if (dc->out->type != TEGRA_DC_OUT_FAKE_DP) {
		cancel_delayed_work_sync(&dp->irq_evt_dwork);
		tegra_dp_lt_force_disable(&dp->lt_data);
		ret = tegra_dp_lt_wait_for_completion(&dp->lt_data,
					STATE_DONE_FAIL, LT_TIMEOUT_MS);
		WARN_ON(!ret);
	}

	if (tegra_dc_hpd(dc)) {
		ret = tegra_dp_panel_power_state(dp,
			NV_DPCD_SET_POWER_VAL_D3_PWRDWN);
		if (ret < 0)
			dev_info(&dp->dc->ndev->dev,
				"dp: failed to enter panel power save mode\n");
	}

	tegra_dc_sor_detach(dp->sor);

	if (tegra_dc_is_nvdisplay()) {
		tegra_sor_clk_switch_setup(dp->sor, false);
		tegra_dp_set_sor_clk_src(dp, dp->sor->safe_clk);
	}

	tegra_dc_sor_disable(dp->sor);

	tegra_dp_clk_disable(dp);

	tegra_dc_io_end(dc);

#ifdef CONFIG_TEGRA_HDA_DC
	if (tegra_dc_is_ext_panel(dc) && dp->sor->audio_support)
		tegra_hda_disable(dp->hda_handle);
#endif

	if (tegra_dp_is_audio_supported(dp)) {
		disp_state_extcon_aux_report(dp->sor->ctrl_num,
			EXTCON_DISP_AUX_STATE_DISABLED);
		pr_info("Extcon AUX%d(DP) disable\n", dp->sor->ctrl_num);
	}

#ifdef CONFIG_SWITCH
	if (tegra_edid_audio_supported(dp->hpd_data.edid)
				&& tegra_dc_is_ext_panel(dc) &&
				dc->out->type != TEGRA_DC_OUT_FAKE_DP) {
		pr_info("dp_audio switch 0\n");
		switch_set_state(&dp->audio_switch, 0);
	}
#endif
}

void tegra_dc_dp_pre_disable_link(struct tegra_dc_dp_data *dp)
{
	tegra_dc_sor_pre_detach(dp->sor);
}

void tegra_dc_dp_disable_link(struct tegra_dc_dp_data *dp, bool powerdown)
{
	tegra_dc_sor_detach(dp->sor);

	if (powerdown)
		tegra_dc_dp_disable(dp->dc);
}

static long tegra_dc_dp_setup_clk(struct tegra_dc *dc, struct clk *clk)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);
	struct clk *dc_parent_clk;
	struct tegra_dc_sor_data *sor = NULL;

	if (!tegra_platform_is_silicon())
		return tegra_dc_pclk_round_rate(dc, dc->mode.pclk);

	if (clk == dc->clk) {
		if (tegra_dc_is_nvdisplay()) {
			dc_parent_clk = tegra_disp_clk_get(&dc->ndev->dev,
					dc->out->parent_clk);
			if (IS_ERR_OR_NULL(dc_parent_clk)) {
				dev_err(&dc->ndev->dev,
						"dp: failed to get clock %s\n",
						dc->out->parent_clk);
				return -EINVAL;
			}
		} else {
			if (dc->out->type == TEGRA_DC_OUT_FAKE_DP)
				dc_parent_clk = clk_get_sys(NULL,
						"pll_d2_out0");
			else
				dc_parent_clk = clk_get_sys(NULL,
						dc->out->parent_clk);
		}
		clk_set_parent(dc->clk, dc_parent_clk);
	}

	/* set pll_d2 to pclk rate */
	tegra_sor_setup_clk(dp->sor, clk, false);

	if (tegra_dc_is_nvdisplay()) {
		sor = dp->sor;

		/* enable pll_dp */
		tegra_dp_clk_enable(dp);

		tegra_sor_safe_clk_enable(sor);

		/* Change for seamless */
		if (!dc->initialized)
			clk_set_parent(sor->sor_clk, sor->safe_clk);

		tegra_disp_clk_prepare_enable(sor->pad_clk);
		tegra_disp_clk_prepare_enable(sor->sor_clk);
	}

	return tegra_dc_pclk_round_rate(dc, dc->mode.pclk);
}

static bool tegra_dc_dp_hpd_state(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);
	struct tegra_dc_dpaux_data *dpaux = dp->dpaux;
	u32 val;

	if (dp->suspended)
		return false;

	if (WARN_ON(!dc || !dc->out))
		return false;

	if (dc->out->type == TEGRA_DC_OUT_FAKE_DP || tegra_platform_is_vdk())
		return true;

	tegra_dpaux_get(dpaux);
	val = tegra_dpaux_readl(dpaux, DPAUX_DP_AUXSTAT);
	tegra_dpaux_put(dpaux);
	return !!(val & DPAUX_DP_AUXSTAT_HPD_STATUS_PLUGGED);
}

/* used by tegra_dc_probe() to detect connection(HPD) status at boot */
static bool tegra_dc_dp_detect(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);

	if ((tegra_platform_is_sim() || tegra_platform_is_fpga()) &&
		(dc->out->hotplug_state == TEGRA_HPD_STATE_NORMAL)) {
		complete(&dc->hpd_complete);
		return true;
	}

	if (dc->out->type == TEGRA_DC_OUT_FAKE_DP && !dc->vedid &&
		dp->edid_src != EDID_SRC_DT) {
		complete(&dc->hpd_complete);
		return false;
	}

	if (tegra_fb_is_console_enabled(dc->pdata) &&
		!tegra_dc_is_ext_panel(dc) &&
		dc->out->type != TEGRA_DC_OUT_FAKE_DP) {
		if (dp->hpd_data.mon_spec.modedb_len > 0) {
			tegra_fb_update_monspecs(dc->fb, &dp->hpd_data.mon_spec,
					tegra_dc_dp_ops.mode_filter);
			tegra_fb_update_fix(dc->fb, &dp->hpd_data.mon_spec);
		}
	}

	tegra_dp_pending_hpd(dp);

	return tegra_dc_hpd(dc);
}

static void tegra_dc_dp_suspend(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);

	if (dp->pdata->hdmi2fpd_bridge_enable)
		hdmi2fpd_suspend(dc);

	if (dp->suspended)
		return;

	dp->suspended = true;

	tegra_dp_lt_invalidate(&dp->lt_data);

	if (dp->dc->out->type != TEGRA_DC_OUT_FAKE_DP) {
		tegra_dp_disable_irq(dp->irq);
		tegra_dp_default_int(dp, false);
	}

	tegra_dp_hpd_suspend(dp);
	tegra_dp_dpaux_disable(dp);
}

static void tegra_dc_dp_resume(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);

	if (!dp->suspended)
		return;

	if (dp->pdata->hdmi2fpd_bridge_enable)
		hdmi2fpd_resume(dc);

	/* Get ready to receive any hpd event */
	tegra_dc_dp_hotplug_init(dc);

	dp->suspended = false;

	if (tegra_platform_is_sim() &&
		(dc->out->hotplug_state == TEGRA_HPD_STATE_NORMAL))
		return;

	if (is_hotplug_supported(dp))
		reinit_completion(&dc->hpd_complete);

	dp->hpd_data.hpd_resuming = true;
	tegra_dp_pending_hpd(dp);

	if (is_hotplug_supported(dp))
		wait_for_completion(&dc->hpd_complete);
}

static void tegra_dc_dp_modeset_notifier(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);
	struct tegra_dc_dpaux_data *dpaux = dp->dpaux;

	/* In case of seamless display, kernel carries forward BL config */
	if (dc->initialized)
		return;

	tegra_dc_io_start(dc);
	tegra_dpaux_clk_en(dpaux);

	tegra_dc_sor_modeset_notifier(dp->sor, false);

	if (!(tegra_platform_is_vdk()))
		tegra_dc_dp_calc_config(dp, dp->mode, &dp->link_cfg);

	tegra_dpaux_clk_dis(dpaux);
	tegra_dc_io_end(dc);
}

static bool tegra_dp_check_dc_constraint(const struct fb_videomode *mode)
{
	return (mode->hsync_len >= 1) && (mode->vsync_len >= 1) &&
		(mode->lower_margin + mode->vsync_len +
		mode->upper_margin > 1) &&
		(mode->xres >= 16) && (mode->yres >= 16);
}

static bool tegra_dp_mode_filter(const struct tegra_dc *dc,
				struct fb_videomode *mode)
{
	struct tegra_dc_dp_data *dp = dc->out_data;
	u8 link_rate = 0, lane_count = 0;
	unsigned int key; /* Index into the link speed table */
	int capability = 1;
	struct tegra_vrr *vrr;

	if (!tegra_dc_hpd((struct tegra_dc *)dc))
		return false;

	if (!mode->pixclock)
		return false;

	if (tegra_dc_is_nvdisplay()) {
		if (mode->xres > 8192)
			return false;
	} else {
		if (mode->xres > 4096)
			return false;
	}

	/* Check if the mode's pixel clock is more than the max rate*/
	if (!tegra_dc_valid_pixclock(dc, mode))
		return false;

	/*
	 * Workaround for modes that fail the constraint:
	 * V_FRONT_PORCH >= V_REF_TO_SYNC + 1
	 *
	 * This constraint does not apply to nvdisplay.
	 */
	if (!tegra_dc_is_nvdisplay() && mode->lower_margin == 1) {
		mode->lower_margin++;
		mode->upper_margin--;
		mode->vmode |= FB_VMODE_ADJUSTED;
	}

	if (tegra_dc_is_t21x()) {
		/* No support for YUV modes on T21x hardware. */
		if (mode->vmode & (YUV_MASK))
			return false;
	}

	if (mode->vmode & FB_VMODE_INTERLACED)
		return false;

	if ((mode->vmode & FB_VMODE_Y420_ONLY) ||
			(mode->vmode & FB_VMODE_Y420))
		return false;

	if ((mode->vmode & FB_VMODE_Y422) &&
			!(mode->vmode & FB_VMODE_Y24))
		return false;

	if (!tegra_dp_check_dc_constraint(mode))
		return false;

	/*
	 * CTS mandates that if edid is corrupted
	 * use fail-safe mode i.e. VGA 640x480@60
	 */
	if (dc->edid->errors)
		return (mode->xres == 640 && mode->yres == 480)
			 ? true : false;

	if (dc->out->vrr) {
		vrr = dc->out->vrr;

		/* FIXME Bug: 1740464 */
		if (tegra_dc_is_vrr_authentication_enabled())
			capability = vrr->capability;

		if (capability) {
			mode->upper_margin += 2;
			if (mode->lower_margin >= 4)
				mode->lower_margin -= 2;
		}
	}

	/* Note: The multiplier when multiplied to 270MHz gives us the link
	 * bandwidth. In other words, one is derived from the other, and the
	 * spec ends up using the two terms interchangeably
	 */
	if (dc->out->type == TEGRA_DC_OUT_FAKE_DP) {
		link_rate = dp->link_cfg.max_link_bw;
		lane_count = dp->link_cfg.max_lane_count;
	} else {
		u8 dpcd_data = 0;

		link_rate = tegra_dc_dp_get_max_link_bw(dp);
		lane_count = tegra_dc_dp_get_max_lane_count(dp, &dpcd_data);
	}

	if (link_rate == 0 || lane_count == 0) {
		dev_err(&dc->ndev->dev,
			"dp: Invalid link rate (%u) or lane count (%u)\n",
			link_rate, lane_count);
		return false;
	}

	if (dc->out->dp_out != NULL) {
		u32 bits_per_pixel;
		u32 max_link_bw_rate;
		u64 total_max_link_bw;
		u64 mode_bw;

		bits_per_pixel = tegra_dp_get_bpp(dp, mode->vmode);
		key = tegra_dp_link_speed_get(dp, link_rate);
		if (WARN_ON(key == dp->sor->num_link_speeds)) {
			dev_info(&dc->ndev->dev, "invalid link bw\n");
			return false;
		}

		max_link_bw_rate = dp->sor->link_speeds[key].max_link_bw;

		/* max link bandwidth = lane_freq * lanes * 8 / 10 */
		total_max_link_bw = (u64)max_link_bw_rate
				* 1000 * 1000 * 8 / 10 * lane_count;
		mode_bw = (u64)PICOS2KHZ(mode->pixclock) * 1000
				* bits_per_pixel;

		if (total_max_link_bw < mode_bw) {
			dev_info(&dc->ndev->dev,
				"mode bw=%llu > link bw=%llu\n",
				mode_bw, total_max_link_bw);
			return false;
		}
	}

	return true;
}

static void tegra_dp_init_hpd_timer_data(struct tegra_dc_dp_data *dp)
{
	struct tegra_hpd_timer_data *timer_data = &dp->hpd_data.timer_data;

	timer_data->plug_stabilize_delay_us = 100000; /* spec-recommended */

	/*
	 * For DP, PLUG and UNPLUG interrupts are generated by the AUX logic
	 * only after the minimum DPAUX_HPD_CONFIG_0.PLUG_MIN_TIME and
	 * DPAUX_HPD_CONFIG_0.UNPLUG_MIN_TIME detection thresholds have been
	 * satisfied. DPAUX_HPD_CONFIG_0.UNPLUG_MIN_TIME is always set at 2ms
	 * per the spec. If HPD has dropped LOW for 2ms, this should always be
	 * considered as a legitimate UNPLUG event - no additional stabilization
	 * delay is needed.
	 */
	timer_data->unplug_stabilize_delay_us = 0;

	timer_data->reassert_delay_us = 100000; /* spec-recommended */
	timer_data->check_edid_delay_us = 400;

	/*
	 * DP sinks can generate pairs of UNPLUG + PLUG events that occur within
	 * (2ms, 100ms). To address these cases, reset the HPD state machine if
	 * HPD drops LOW and then comes back up within reassert_delay_us.
	 */
	timer_data->reset_on_reassert = true;

	/*
	 * For DP, a PLUG interrupt is only generated when HPD transitions from
	 * LOW to HIGH, and stays HIGH for at least
	 * DPAUX_HPD_CONFIG_0.PLUG_MIN_TIME. If the state machine is currently
	 * enabled, and receives a PLUG event, this indicates SW was not able to
	 * schedule the HPD worker for the previous UNPLUG before it got
	 * canceled. In this scenario, we should still treat the current PLUG
	 * event as a legitimate event, and reset.
	 */
	timer_data->reset_on_plug_bounce = true;
}

static void tegra_dp_hpd_op_init(void *drv_data)
{
	struct tegra_dc_dp_data *dp = drv_data;

	tegra_dp_init_hpd_timer_data(dp);
}

static bool (*tegra_dp_op_get_mode_filter(void *drv_data))
	(const struct tegra_dc *dc, struct fb_videomode *mode) {
	return tegra_dp_mode_filter;
}

static bool tegra_dp_hpd_op_get_hpd_state(void *drv_data)
{
	struct tegra_dc_dp_data *dp = drv_data;

	return tegra_dc_hpd(dp->dc);
}

static bool tegra_dp_hpd_op_edid_read_prepare(void *drv_data)
{
	struct tegra_dc_dp_data *dp = drv_data;
	int ret;

	tegra_dc_io_start(dp->dc);

	ret = tegra_dp_panel_power_state(dp, NV_DPCD_SET_POWER_VAL_D0_NORMAL);
	if (ret < 0) {
		dev_err(&dp->dc->ndev->dev,
		"dp: failed to exit panel power save mode (0x%x)\n", ret);
		tegra_dc_io_end(dp->dc);
		return false;
	}

	tegra_dc_io_end(dp->dc);

	return true;
}

static void tegra_dc_dp_sor_sleep(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);

	if (dp->sor->sor_state == SOR_ATTACHED)
		tegra_dc_sor_sleep(dp->sor);
}

static u32 tegra_dc_dp_sor_crc_check(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);

	return tegra_dc_sor_debugfs_get_crc(dp->sor, NULL);
}

static void tegra_dc_dp_sor_crc_toggle(struct tegra_dc *dc,
	u32 val)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);

	tegra_dc_sor_toggle_crc(dp->sor, val);
}

static int tegra_dc_dp_sor_crc_en_dis(struct tegra_dc *dc,
				      struct tegra_dc_ext_crc_or_params *params,
				      bool en)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);

	if (params->out_type != TEGRA_DC_EXT_DP)
		return -EINVAL;

	tegra_dc_sor_crc_en_dis(dp->sor, params->sor_params, en);

	return 0;
}

static int tegra_dc_dp_sor_crc_en(struct tegra_dc *dc,
				  struct tegra_dc_ext_crc_or_params *params)
{
	return tegra_dc_dp_sor_crc_en_dis(dc, params, true);
}

static int tegra_dc_dp_sor_crc_dis(struct tegra_dc *dc,
				   struct tegra_dc_ext_crc_or_params *params)
{
	return tegra_dc_dp_sor_crc_en_dis(dc, params, false);
}

static int tegra_dc_dp_sor_crc_get(struct tegra_dc *dc, u32 *crc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);

	return tegra_dc_sor_crc_get(dp->sor, crc);
}

static struct tegra_hpd_ops hpd_ops = {
	.init = tegra_dp_hpd_op_init,
	.edid_read = tegra_dp_hpd_op_edid_read,
	.edid_ready = tegra_dp_hpd_op_edid_ready,
	.edid_recheck = tegra_dp_hpd_op_edid_recheck,
	.get_mode_filter = tegra_dp_op_get_mode_filter,
	.get_hpd_state = tegra_dp_hpd_op_get_hpd_state,
	.edid_read_prepare = tegra_dp_hpd_op_edid_read_prepare,
};

static int tegra_dc_dp_get_sor_ctrl_num(struct tegra_dc *dc)
{
	struct tegra_dc_dp_data *dp = tegra_dc_get_outdata(dc);

	return (!dp) ? -ENODEV : tegra_sor_get_ctrl_num(dp->sor);
}

struct tegra_dc_out_ops tegra_dc_dp_ops = {
	.init	   = tegra_dc_dp_init,
	.destroy   = tegra_dc_dp_destroy,
	.enable	   = tegra_dc_dp_enable,
	.disable   = tegra_dc_dp_disable,
	.detect    = tegra_dc_dp_detect,
	.setup_clk = tegra_dc_dp_setup_clk,
	.modeset_notifier = tegra_dc_dp_modeset_notifier,
	.mode_filter = tegra_dp_mode_filter,
	.hpd_state = tegra_dc_dp_hpd_state,
	.suspend = tegra_dc_dp_suspend,
	.resume = tegra_dc_dp_resume,
	.hotplug_init = tegra_dc_dp_hotplug_init,
	.shutdown_interface = tegra_dc_dp_sor_sleep,
	.get_crc = tegra_dc_dp_sor_crc_check,
	.toggle_crc = tegra_dc_dp_sor_crc_toggle,
	.get_connector_instance = tegra_dc_dp_get_sor_ctrl_num,
	.crc_en = tegra_dc_dp_sor_crc_en,
	.crc_dis = tegra_dc_dp_sor_crc_dis,
	.crc_get = tegra_dc_dp_sor_crc_get,
};
