/*
 * dpaux.c: dpaux function definitions.
 *
 * Copyright (c) 2014-2019, NVIDIA CORPORATION, All rights reserved.
 * Author: Animesh Kishore <ankishore@nvidia.com>
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

#include <linux/mutex.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/tegra_prod.h>
#include <linux/of_irq.h>
#include <linux/tegra_pm_domains.h>
#include <linux/delay.h>
#include <soc/tegra/chip-id.h>

#include "dpaux_regs.h"
#include "dc_priv.h"
#include "dpaux.h"

static struct of_device_id tegra_dpaux_pd[] = {
	{ .compatible = "nvidia,tegra210-sor-pd", },
	{ .compatible = "nvidia,tegra186-disa-pd", },
	{ .compatible = "nvidia,tegra194-disa-pd", },
	{},
};

static inline unsigned long
tegra_dpaux_poll_register(struct tegra_dc_dpaux_data *dpaux,
			u32 reg, u32 mask, u32 exp_val,
			u32 poll_interval_us, u32 timeout_ms)
{
	unsigned long timeout_jf = jiffies + msecs_to_jiffies(timeout_ms);
	u32 reg_val = 0;

	do {
		usleep_range(poll_interval_us, poll_interval_us << 1);
		reg_val = tegra_dpaux_readl(dpaux, reg);
	} while (((reg_val & mask) != exp_val) &&
		time_after(timeout_jf, jiffies));

	if ((reg_val & mask) == exp_val)
		return 0;

	dev_dbg(&dpaux->dc->ndev->dev,
		"dpaux_poll_register 0x%x: timeout\n", reg);
	return jiffies - timeout_jf + 1;
}

static void tegra_dpaux_reset(struct tegra_dc_dpaux_data *dpaux)
{
	if (tegra_platform_is_sim())
		return;

	if (!dpaux)
		return;

	/* Seamless prevent reset */
	if (dpaux->dc->initialized)
		return;

	if (dpaux->rst) {
		reset_control_assert(dpaux->rst);
		mdelay(2);
		reset_control_deassert(dpaux->rst);
		mdelay(1);
	}
}

void tegra_dpaux_get(struct tegra_dc_dpaux_data *dpaux)
{
	int enable_count = atomic_inc_return(&dpaux->enable_count);

	WARN_ON(enable_count < 1);
	if (enable_count == 1) {
		tegra_dc_io_start(dpaux->dc);
		tegra_unpowergate_partition(dpaux->powergate_id);
		tegra_dpaux_clk_en(dpaux);
		tegra_dpaux_reset(dpaux);
	}
}

void tegra_dpaux_put(struct tegra_dc_dpaux_data *dpaux)
{
	WARN_ON(atomic_read(&dpaux->enable_count) == 0);
	if (atomic_dec_return(&dpaux->enable_count) == 0) {
		tegra_dpaux_clk_dis(dpaux);
		tegra_powergate_partition(dpaux->powergate_id);
		tegra_dc_io_end(dpaux->dc);
	}
}

static inline void tegra_dpaux_get_name(char *buf, size_t buf_len, int ctrl_num)
{
	if (ctrl_num > 0)
		snprintf(buf, buf_len, "dpaux%d", ctrl_num);
	else
		snprintf(buf, buf_len, "dpaux");
}

int tegra_dpaux_readl(struct tegra_dc_dpaux_data *dpaux, u32 reg)
{
	return readl(dpaux->base + reg * 4);
}

void tegra_dpaux_writel(struct tegra_dc_dpaux_data *dpaux, u32 reg, u32 val)
{
	writel(val, dpaux->base + reg * 4);
}

void tegra_dpaux_write_field(struct tegra_dc_dpaux_data *dpaux, u32 reg,
				u32 mask, u32 val)
{
	u32 reg_val = tegra_dpaux_readl(dpaux, reg);

	reg_val = (reg_val & ~mask) | (val & mask);
	tegra_dpaux_writel(dpaux, reg, reg_val);
}

int tegra_dpaux_clk_en(struct tegra_dc_dpaux_data *dpaux)
{
	return tegra_disp_clk_prepare_enable(dpaux->clk);
}

void tegra_dpaux_clk_dis(struct tegra_dc_dpaux_data *dpaux)
{
	tegra_disp_clk_disable_unprepare(dpaux->clk);
}

void tegra_dpaux_int_toggle(struct tegra_dc_dpaux_data *dpaux, u32 intr,
				bool enable)
{
	u32 reg_val = tegra_dpaux_readl(dpaux, DPAUX_INTR_EN_AUX);

	if (enable)
		reg_val |= intr;
	else
		reg_val &= ~intr;

	tegra_dpaux_writel(dpaux, DPAUX_INTR_EN_AUX, reg_val);
}

static inline int tegra_dpaux_wait_transaction(
					struct tegra_dc_dpaux_data *dpaux)
{
	int err = 0;

	if (likely(tegra_platform_is_silicon()) ||
		unlikely(tegra_platform_is_fpga())) {
		if (tegra_dpaux_poll_register(dpaux, DPAUX_DP_AUXCTL,
			DPAUX_DP_AUXCTL_TRANSACTREQ_MASK,
			DPAUX_DP_AUXCTL_TRANSACTREQ_DONE,
			100, DP_AUX_TIMEOUT_MS) != 0)
			err = -EFAULT;
	}

	if (err)
		dev_err(&dpaux->dc->ndev->dev, "dp: aux tx timeout\n");

	return err;
}

/*
 * To config DPAUX Transaction Control
 * o Inputs
 *  - dpaux    : pointer to DPAUX information
 *  - cmd   : transaction command DPAUX_DP_AUXCTL_CMD_xxx
 *  - addr  : transaction address (20 bit sink device AUX reg addr space)
 *  - p_wrdt: pointer to the write data buffer / NULL:no write data
 *  - size  : 1-16: number of byte to read/write
 *            0   : address only transaction
 * o Outputs
 *  - return: error status; 0:no error / !0:error
 */
static int tegra_dp_aux_tx_config(struct tegra_dc_dpaux_data *dpaux,
				u32 cmd, u32 addr, u8 *p_wrdt, u32 size)
{
	int i;
	union {
		u32 d32[DP_AUX_MAX_BYTES / sizeof(u32)];
		u8  d8[DP_AUX_MAX_BYTES];
	} __packed wdata = {{0}};

	if (size > DP_AUX_MAX_BYTES)
		goto fail;

	switch (cmd) {
	case DPAUX_DP_AUXCTL_CMD_I2CWR:
	case DPAUX_DP_AUXCTL_CMD_I2CRD:
	case DPAUX_DP_AUXCTL_CMD_I2CREQWSTAT:
	case DPAUX_DP_AUXCTL_CMD_MOTWR:
	case DPAUX_DP_AUXCTL_CMD_MOTRD:
	case DPAUX_DP_AUXCTL_CMD_MOTREQWSTAT:
	case DPAUX_DP_AUXCTL_CMD_AUXWR:
	case DPAUX_DP_AUXCTL_CMD_AUXRD:
		tegra_dpaux_write_field(dpaux, DPAUX_DP_AUXCTL,
					DPAUX_DP_AUXCTL_CMD_MASK, cmd);
		break;
	default:
		goto fail;
	};
	tegra_dpaux_write_field(dpaux, DPAUX_DP_AUXCTL,
				DPAUX_DP_AUXCTL_CMDLEN_MASK,
				size ? size - 1 : 0);
	tegra_dpaux_write_field(dpaux, DPAUX_DP_AUXCTL,
			DPAUX_DP_AUXCTL_ADDRESS_ONLY_MASK,
			(size == 0) ? DPAUX_DP_AUXCTL_ADDRESS_ONLY_TRUE :
				DPAUX_DP_AUXCTL_ADDRESS_ONLY_FALSE);

	tegra_dpaux_writel(dpaux, DPAUX_DP_AUXADDR, addr);
	if ((p_wrdt) && (size)) {
		for (i = 0; i < size; i++)
			wdata.d8[i] = *(p_wrdt+i);

		for (i = 0; i < (DP_AUX_MAX_BYTES / sizeof(u32)); ++i)
			tegra_dpaux_writel(dpaux, DPAUX_DP_AUXDATA_WRITE_W(i),
				wdata.d32[i]);
	}
	return 0;
fail:
	return -EINVAL;
}

int tegra_dc_dpaux_i2c_read(struct tegra_dc_dpaux_data *dpaux, u32 i2c_addr,
				u8 *data, u32 *size, u32 *aux_stat)
{
	u32 finished = 0;
	u32 cur_size;
	int ret = 0;

	if (!dpaux) {
		pr_err("%s: dpaux must be non-NULL", __func__);
		return -ENODEV;
	}

	if (*size == 0) {
		dev_err(&dpaux->dc->ndev->dev,
			"dp: i2c read size can't be 0\n");
		return -EINVAL;
	}

	if (dpaux->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return ret;

	mutex_lock(&dpaux->lock);
	tegra_dpaux_get(dpaux);
	do {
		cur_size = *size - finished;

		if (cur_size > DP_AUX_MAX_BYTES)
			cur_size = DP_AUX_MAX_BYTES;

		ret = tegra_dc_dpaux_read_chunk_locked(dpaux,
			DPAUX_DP_AUXCTL_CMD_MOTRD,
			i2c_addr, data, &cur_size, aux_stat);
		if (ret)
			break;

		data += cur_size;
		finished += cur_size;
	} while (*size > finished);

	cur_size = 0;
	tegra_dc_dpaux_read_chunk_locked(dpaux,
			DPAUX_DP_AUXCTL_CMD_I2CRD,
			i2c_addr, data, &cur_size, aux_stat);

	tegra_dpaux_put(dpaux);
	mutex_unlock(&dpaux->lock);

	*size = finished;

	return ret;
}

/* TODO: Handle update status scenario and size > 16 bytes*/
int tegra_dc_dpaux_i2c_write(struct tegra_dc_dpaux_data *dpaux, u32 cmd,
			u32 i2c_addr, u8 *data, u32 *size, u32 *aux_stat)
{
	int ret = 0;

	if (!dpaux) {
		pr_err("%s: dpaux must be non-NULL", __func__);
		return -ENODEV;
	}

	if (*size == 0) {
		dev_err(&dpaux->dc->ndev->dev,
			"dp: i2c write size can't be 0\n");
		return -EINVAL;
	}

	if (dpaux->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return ret;

	mutex_lock(&dpaux->lock);
	tegra_dpaux_get(dpaux);

	ret = tegra_dc_dpaux_write_chunk_locked(dpaux, cmd, i2c_addr, data,
						size, aux_stat);

	tegra_dpaux_put(dpaux);
	mutex_unlock(&dpaux->lock);

	return ret;
}

int tegra_dc_dpaux_read_chunk_locked(struct tegra_dc_dpaux_data *dpaux,
	u32 cmd, u32 addr, u8 *data, u32 *size, u32 *aux_stat)
{
	int err = 0;
	u32 timeout_retries = DP_AUX_TIMEOUT_MAX_TRIES;
	u32 defer_retries	= DP_AUX_DEFER_MAX_TRIES;

	if (!dpaux) {
		pr_err("%s: dpaux must be non-NULL", __func__);
		return -ENODEV;
	}

	WARN_ON(!mutex_is_locked(&dpaux->lock));

	switch (cmd) {
	case DPAUX_DP_AUXCTL_CMD_I2CRD:
	case DPAUX_DP_AUXCTL_CMD_I2CREQWSTAT:
	case DPAUX_DP_AUXCTL_CMD_MOTREQWSTAT:
	case DPAUX_DP_AUXCTL_CMD_MOTRD:
	case DPAUX_DP_AUXCTL_CMD_AUXRD:
		break;
	default:
		dev_err(&dpaux->dc->ndev->dev,
			"dp: invalid aux read cmd: 0x%x\n", cmd);
		return -EINVAL;
	};

	err = tegra_dp_aux_tx_config(dpaux, cmd, addr, NULL, *size);
	if (err < 0) {
		dev_err(&dpaux->dc->ndev->dev, "dp: incorrect aux tx params\n");
		return err;
	}

	while (1) {
		if ((timeout_retries != DP_AUX_TIMEOUT_MAX_TRIES) ||
		    (defer_retries != DP_AUX_DEFER_MAX_TRIES))
			usleep_range(DP_DPCP_RETRY_SLEEP_US,
				DP_DPCP_RETRY_SLEEP_US << 1);

		if (tegra_platform_is_silicon()) {
			*aux_stat = tegra_dpaux_readl(dpaux, DPAUX_DP_AUXSTAT);
			if (!(*aux_stat &
				DPAUX_DP_AUXSTAT_HPD_STATUS_PLUGGED)) {
				dev_err(&dpaux->dc->ndev->dev,
					"dp: HPD is not detected\n");
				return -EFAULT;
			}
		}

		tegra_dpaux_write_field(dpaux, DPAUX_DP_AUXCTL,
					DPAUX_DP_AUXCTL_TRANSACTREQ_MASK,
					DPAUX_DP_AUXCTL_TRANSACTREQ_PENDING);

		if (tegra_dpaux_wait_transaction(dpaux))
			dev_err(&dpaux->dc->ndev->dev,
				"dp: aux read transaction timeout\n");

		*aux_stat = tegra_dpaux_readl(dpaux, DPAUX_DP_AUXSTAT);

		if ((*aux_stat & DPAUX_DP_AUXSTAT_TIMEOUT_ERROR_PENDING) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_RX_ERROR_PENDING) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_SINKSTAT_ERROR_PENDING) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_NO_STOP_ERROR_PENDING)) {
			if (timeout_retries-- > 0) {
				dev_info(&dpaux->dc->ndev->dev,
					"dp: aux read retry (0x%x) -- %d\n",
					*aux_stat, timeout_retries);
				/* clear the error bits */
				tegra_dpaux_writel(dpaux, DPAUX_DP_AUXSTAT,
					*aux_stat);
				continue; /* retry */
			} else {
				dev_err(&dpaux->dc->ndev->dev,
					"dp: aux read got error (0x%x)\n",
					*aux_stat);
				return -EFAULT;
			}
		}

		if ((*aux_stat & DPAUX_DP_AUXSTAT_REPLYTYPE_I2CDEFER) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_REPLYTYPE_DEFER)) {
			if (defer_retries-- > 0) {
				dev_info(&dpaux->dc->ndev->dev,
					"dp: aux read defer (0x%x) -- %d\n",
					*aux_stat, defer_retries);
				/* clear the error bits */
				tegra_dpaux_writel(dpaux, DPAUX_DP_AUXSTAT,
					*aux_stat);
				continue;
			} else {
				dev_err(&dpaux->dc->ndev->dev,
					"dp: aux read defer exceeds max retries (0x%x)\n",
					*aux_stat);
				return -EFAULT;
			}
		}

		if ((*aux_stat & DPAUX_DP_AUXSTAT_REPLYTYPE_MASK) ==
			DPAUX_DP_AUXSTAT_REPLYTYPE_ACK) {
			int i;
			u32 temp_data[4];

			for (i = 0; i < DP_AUX_MAX_BYTES/4; ++i)
				temp_data[i] = tegra_dpaux_readl(dpaux,
					DPAUX_DP_AUXDATA_READ_W(i));

			*size = ((*aux_stat) & DPAUX_DP_AUXSTAT_REPLY_M_MASK);
			memcpy(data, temp_data, *size);

			return 0;
		} else {
			dev_err(&dpaux->dc->ndev->dev,
				"dp: aux read failed (0x%x\n", *aux_stat);
			return -EFAULT;
		}
	}
	/* Should never come to here */
	return -EFAULT;
}

int tegra_dc_dpaux_write_chunk_locked(struct tegra_dc_dpaux_data *dpaux,
	u32 cmd, u32 addr, u8 *data, u32 *size, u32 *aux_stat)
{
	int err = 0;
	u32 timeout_retries = DP_AUX_TIMEOUT_MAX_TRIES;
	u32 defer_retries	= DP_AUX_DEFER_MAX_TRIES;

	if (!dpaux) {
		pr_err("%s: dpaux must be non-NULL", __func__);
		return -ENODEV;
	}

	WARN_ON(!mutex_is_locked(&dpaux->lock));

	switch (cmd) {
	case DPAUX_DP_AUXCTL_CMD_I2CWR:
	case DPAUX_DP_AUXCTL_CMD_MOTWR:
	case DPAUX_DP_AUXCTL_CMD_AUXWR:
		break;
	default:
		dev_err(&dpaux->dc->ndev->dev,
			"dp: invalid aux write cmd: 0x%x\n", cmd);
		return -EINVAL;
	};

	err = tegra_dp_aux_tx_config(dpaux, cmd, addr, data, *size);
	if (err < 0) {
		dev_err(&dpaux->dc->ndev->dev, "dp: incorrect aux tx params\n");
		return err;
	}

	while (1) {
		if ((timeout_retries != DP_AUX_TIMEOUT_MAX_TRIES) ||
		    (defer_retries != DP_AUX_DEFER_MAX_TRIES))
			usleep_range(DP_DPCP_RETRY_SLEEP_US,
				DP_DPCP_RETRY_SLEEP_US << 1);

		if (tegra_platform_is_silicon()) {
			*aux_stat = tegra_dpaux_readl(dpaux, DPAUX_DP_AUXSTAT);
			if (!(*aux_stat &
				DPAUX_DP_AUXSTAT_HPD_STATUS_PLUGGED)) {
				dev_err(&dpaux->dc->ndev->dev,
					"dp: HPD is not detected\n");
				return -EFAULT;
			}
		}

		tegra_dpaux_write_field(dpaux, DPAUX_DP_AUXCTL,
					DPAUX_DP_AUXCTL_TRANSACTREQ_MASK,
					DPAUX_DP_AUXCTL_TRANSACTREQ_PENDING);

		if (tegra_dpaux_wait_transaction(dpaux))
			dev_err(&dpaux->dc->ndev->dev,
				"dp: aux write transaction timeout\n");

		*aux_stat = tegra_dpaux_readl(dpaux, DPAUX_DP_AUXSTAT);

		if ((*aux_stat & DPAUX_DP_AUXSTAT_TIMEOUT_ERROR_PENDING) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_RX_ERROR_PENDING) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_SINKSTAT_ERROR_PENDING) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_NO_STOP_ERROR_PENDING) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_REPLYTYPE_NACK) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_REPLYTYPE_I2CNACK)) {
			if (timeout_retries-- > 0) {
				dev_info(&dpaux->dc->ndev->dev,
					"dp: aux write retry (0x%x) -- %d\n",
					*aux_stat, timeout_retries);
				/* clear the error bits */
				tegra_dpaux_writel(dpaux, DPAUX_DP_AUXSTAT,
					*aux_stat);
				continue;
			} else {
				dev_err(&dpaux->dc->ndev->dev,
					"dp: aux write got error (0x%x)\n",
					*aux_stat);
				return -EFAULT;
			}
		}

		if ((*aux_stat & DPAUX_DP_AUXSTAT_REPLYTYPE_I2CDEFER) ||
			(*aux_stat & DPAUX_DP_AUXSTAT_REPLYTYPE_DEFER)) {
			if (defer_retries-- > 0) {
				dev_info(&dpaux->dc->ndev->dev,
					"dp: aux write defer (0x%x) -- %d\n",
					*aux_stat, defer_retries);
				/* clear the error bits */
				tegra_dpaux_writel(dpaux, DPAUX_DP_AUXSTAT,
					*aux_stat);
				continue;
			} else {
				dev_err(&dpaux->dc->ndev->dev,
					"dp: aux write defer exceeds max retries (0x%x)\n",
					*aux_stat);
				return -EFAULT;
			}
		}

		if ((*aux_stat & DPAUX_DP_AUXSTAT_REPLYTYPE_MASK) ==
			DPAUX_DP_AUXSTAT_REPLYTYPE_ACK) {
			(*size)++;
			return 0;
		} else {
			dev_err(&dpaux->dc->ndev->dev,
				"dp: aux write failed (0x%x)\n", *aux_stat);
			return -EFAULT;
		}
	}
	/* Should never come to here */
	return -EFAULT;
}

int tegra_dc_dpaux_read(struct tegra_dc_dpaux_data *dpaux, u32 cmd, u32 addr,
	u8 *data, u32 *size, u32 *aux_stat)
{
	u32	finished = 0;
	u32	cur_size;
	int	ret	 = 0;

	if (*size == 0) {
		dev_err(&dpaux->dc->ndev->dev,
			"dp: aux read size can't be 0\n");
		return -EINVAL;
	}

	if (dpaux->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return  ret;

	mutex_lock(&dpaux->lock);
	tegra_dpaux_get(dpaux);
	do {
		cur_size = *size - finished;
		if (cur_size > DP_AUX_MAX_BYTES)
			cur_size = DP_AUX_MAX_BYTES;

		ret = tegra_dc_dpaux_read_chunk_locked(dpaux, cmd, addr,
			data, &cur_size, aux_stat);

		if (ret)
			break;

		/* cur_size should be the real size returned */
		addr += cur_size;
		data += cur_size;
		finished += cur_size;

	} while (*size > finished);
	tegra_dpaux_put(dpaux);
	mutex_unlock(&dpaux->lock);

	*size = finished;
	return ret;
}

int tegra_dc_dpaux_write(struct tegra_dc_dpaux_data *dpaux, u32 cmd, u32 addr,
	u8 *data, u32 *size, u32 *aux_stat)
{
	u32	cur_size = 0;
	u32	finished = 0;
	int	ret	 = 0;

	if (*size == 0) {
		dev_err(&dpaux->dc->ndev->dev,
			"dp: aux write size can't be 0\n");
		return -EINVAL;
	}

	if (dpaux->dc->out->type == TEGRA_DC_OUT_FAKE_DP)
		return ret;

	mutex_lock(&dpaux->lock);
	tegra_dpaux_get(dpaux);
	do {
		cur_size = *size - finished;
		if (cur_size > DP_AUX_MAX_BYTES)
			cur_size = DP_AUX_MAX_BYTES;

		ret = tegra_dc_dpaux_write_chunk_locked(dpaux, cmd, addr,
			data, &cur_size, aux_stat);

		finished += cur_size;
		addr += cur_size;
		data += cur_size;

		if (ret)
			break;
	} while (*size > finished);
	tegra_dpaux_put(dpaux);
	mutex_unlock(&dpaux->lock);

	*size = finished;
	return ret;
}

static inline void _tegra_dpaux_pad_power(struct tegra_dc_dpaux_data *dpaux,
					bool on)
{
	tegra_dpaux_writel(dpaux,
			DPAUX_HYBRID_SPARE,
			(on ? DPAUX_HYBRID_SPARE_PAD_PWR_POWERUP :
			DPAUX_HYBRID_SPARE_PAD_PWR_POWERDOWN));
}

void tegra_dpaux_pad_power(struct tegra_dc_dpaux_data *dpaux, bool on)
{
	struct tegra_dc *dc;

	if (!dpaux || !dpaux->dc) {
		pr_err("%s: dpaux and dc must both be non-NULL", __func__);
		return;
	}

	dc = dpaux->dc;
	mutex_lock(&dpaux->lock);
	tegra_dpaux_get(dpaux);
	_tegra_dpaux_pad_power(dpaux, on);
	tegra_dpaux_put(dpaux);
	mutex_unlock(&dpaux->lock);
}

static inline void _tegra_dpaux_config_pad_mode(
					struct tegra_dc_dpaux_data *dpaux,
					enum tegra_dpaux_pad_mode mode)
{
	u32 val = 0;

	val = tegra_dpaux_readl(dpaux, DPAUX_HYBRID_PADCTL);

	val &= ~(DPAUX_HYBRID_PADCTL_I2C_SDA_INPUT_RCV_ENABLE |
		DPAUX_HYBRID_PADCTL_I2C_SCL_INPUT_RCV_ENABLE |
		DPAUX_HYBRID_PADCTL_MODE_I2C);
	val |= mode ? (DPAUX_HYBRID_PADCTL_I2C_SDA_INPUT_RCV_ENABLE |
		DPAUX_HYBRID_PADCTL_I2C_SCL_INPUT_RCV_ENABLE |
		mode) : 0;

	tegra_dpaux_writel(dpaux, DPAUX_HYBRID_PADCTL, val);
}

void tegra_dpaux_config_pad_mode(struct tegra_dc_dpaux_data *dpaux,
			enum tegra_dpaux_pad_mode mode)
{
	struct tegra_dc *dc;

	if (!dpaux || !dpaux->dc) {
		pr_err("%s: dpaux and dc must both be non-NULL", __func__);
		return;
	}

	dc = dpaux->dc;
	mutex_lock(&dpaux->lock);
	tegra_dpaux_get(dpaux);
	tegra_dpaux_prod_set(dpaux);
	/*
	 * Make sure to configure the pad mode before we power it on. If not
	 * done in this order, there is a chance that the pad will run in the
	 * default mode for a while before switching to the requested mode. This
	 * could cause intermittent glitches on the physical lines.
	 */
	_tegra_dpaux_config_pad_mode(dpaux, mode);
	_tegra_dpaux_pad_power(dpaux, true);
	tegra_dpaux_put(dpaux);
	mutex_unlock(&dpaux->lock);
}

void tegra_dpaux_prod_set(struct tegra_dc_dpaux_data *dpaux)
{
	struct tegra_dc *dc;

	if (!dpaux || !dpaux->dc) {
		pr_err("%s: dpaux and dc must both be non-NULL", __func__);
		return;
	}

	/* Only HDMI, DP, and fakeDP use DPAUX. */
	dc = dpaux->dc;
	if (dc->out->type != TEGRA_DC_OUT_HDMI &&
		dc->out->type != TEGRA_DC_OUT_DP &&
		dc->out->type != TEGRA_DC_OUT_FAKE_DP) {
		pr_err("%s: dc output type must be HDMI, DP, or fakeDP\n",
			__func__);
		return;
	}

	tegra_dpaux_get(dpaux);

	if (!IS_ERR_OR_NULL(dpaux->prod_list)) {
		char *prod_string = NULL;

		prod_string = dc->out->type == TEGRA_DC_OUT_HDMI ?
				"prod_c_dpaux_hdmi" : "prod_c_dpaux_dp";

		if (tegra_prod_set_by_name(&dpaux->base, prod_string,
							dpaux->prod_list)) {
			dev_warn(&dc->ndev->dev, "%s: dpaux prod set failed\n",
				__func__);
		}
	}

	tegra_dpaux_put(dpaux);
}

struct tegra_dc_dpaux_data *tegra_dpaux_init_data(struct tegra_dc *dc,
		struct device_node *sor_np)
{
	u32 temp;
	int err = 0;
	char dpaux_name[CHAR_BUF_SIZE_MAX] = {0};
	void __iomem *base = NULL;
	struct clk *clk = NULL;
	struct device_node *dpaux_np = NULL;
	struct reset_control *rst = NULL;
	struct tegra_prod *prod_list = NULL;
	struct tegra_dc_dpaux_data *dpaux = NULL;

	if (!dc || !sor_np) {
		pr_err("%s: err: %s cannot be NULL\n", __func__,
				!dc ? "dc" : "sor_np");
		return NULL;
	}

	dpaux_np = of_parse_phandle(sor_np, "nvidia,dpaux", 0);
	if (IS_ERR_OR_NULL(dpaux_np)) {
		dev_err(&dc->ndev->dev, "%s: could not find %s property for %s\n",
			__func__, "nvidia,dpaux", of_node_full_name(sor_np));
		return NULL;
	}

	if (!of_device_is_available(dpaux_np)) {
		dev_err(&dc->ndev->dev, "%s: %s present but disabled\n",
				__func__, of_node_full_name(dpaux_np));
		err = -ENODEV;
		goto exit;
	}

	/* Allocate memory for the dpaux struct. */
	dpaux = devm_kzalloc(&dc->ndev->dev, sizeof(*dpaux), GFP_KERNEL);
	if (!dpaux) {
		err = -ENOMEM;
		goto exit;
	}

	if (!of_property_read_u32(dpaux_np, "nvidia,dpaux-ctrlnum", &temp)) {
		dpaux->ctrl_num = (unsigned long)temp;
	} else {
		dev_err(&dc->ndev->dev, "mandatory property %s for %s not found\n",
				"nvidia,dpaux-ctrlnum",
				of_node_full_name(dpaux_np));
		goto release_mem;
	}

	tegra_dpaux_get_name(dpaux_name, CHAR_BUF_SIZE_MAX, dpaux->ctrl_num);

	/* ioremap the memory region for the DPAUX registers. */
	base = of_iomap(dpaux_np, 0);
	if (!base) {
		dev_err(&dc->ndev->dev, "%s: %s regs can't be mapped\n",
				__func__, of_node_full_name(dpaux_np));
		err = -ENOENT;
		goto release_mem;
	}

	/* Query the DPAUX clock. */
	clk = tegra_disp_of_clk_get_by_name(dpaux_np, dpaux_name);
	if (IS_ERR_OR_NULL(clk)) {
		dev_err(&dc->ndev->dev, "%s: %s clk unavailable\n", __func__,
			dpaux_name);
		err = -ENOENT;

		goto err_unmap_region;
	}

	/* Extract the reset entry from the DT node. */
	rst = of_reset_control_get(dpaux_np, dpaux_name);
	if (IS_ERR_OR_NULL(rst)) {
		dev_err(&dc->ndev->dev,
			"%s: Unable to get %s reset control\n",
			__func__, dpaux_name);
		err = -ENOENT;
		goto err_put_clk;
	}
	reset_control_deassert(rst);

	if (!tegra_platform_is_sim()) {
		prod_list = devm_tegra_prod_get_from_node(
				&dc->ndev->dev, dpaux_np);
		if (IS_ERR_OR_NULL(prod_list)) {
			dev_err(&dc->ndev->dev,
				"%s: prod list init failed for dpaux with error %ld\n",
				__func__, PTR_ERR(prod_list));
			err = -EINVAL;
			goto err_put_rst;
		}
	}

	mutex_init(&dpaux->lock);
	dpaux->powergate_id = tegra_pd_get_powergate_id(tegra_dpaux_pd);
	dpaux->dc = dc;
	dpaux->base = base;
	dpaux->clk = clk;
	dpaux->rst = rst;
	dpaux->prod_list = prod_list;
	dpaux->np = dpaux_np;

	return dpaux;

err_put_rst:
	if (rst)
		reset_control_put(rst);
err_put_clk:
	if (tegra_dc_is_t21x())
		clk_put(clk);
err_unmap_region:
	iounmap(base);
release_mem:
	devm_kfree(&dc->ndev->dev, dpaux);
exit:
	of_node_put(dpaux_np);
	return ERR_PTR(err);
}

int tegra_dpaux_get_irq(struct tegra_dc_dpaux_data *dpaux)
{
	int irq;

	if (!dpaux)
		return 0; /* return 0 for an error */

	irq = of_irq_to_resource(dpaux->np, 0, NULL);
	if (!irq)
		pr_err("%s: error getting irq\n", __func__);

	return irq;
}

struct clk *tegra_dpaux_get_clk(struct tegra_dc_dpaux_data *dpaux,
		const char *clk_name)
{
	if (!dpaux || !clk_name)
		return NULL;

	if (tegra_dc_is_nvdisplay())
		return tegra_disp_of_clk_get_by_name(dpaux->np, clk_name);
	else
		return clk_get_sys(NULL, clk_name);
}

void tegra_dpaux_destroy_data(struct tegra_dc_dpaux_data *dpaux)
{
	struct tegra_dc *dc;

	if (!dpaux || !dpaux->dc) {
		pr_err("%s: %s must be non-NULL\n", __func__,
				!dpaux ? "dpaux" : "dc");
		return;
	}

	dc = dpaux->dc;
	if (dpaux->rst)
		reset_control_put(dpaux->rst);

	if (tegra_dc_is_t21x())
		clk_put(dpaux->clk);

	iounmap(dpaux->base);

	devm_kfree(&dc->ndev->dev, dpaux);
}
