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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/phy/phy.h>
#include <linux/of.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of_platform.h>
#include <soc/tegra/bpmp_abi.h>
#include <soc/tegra/tegra_bpmp.h>

#define P2U_PERIODIC_EQ_CTRL_GEN3	0xc0
#define P2U_PERIODIC_EQ_CTRL_GEN3_PERIODIC_EQ_EN		BIT(0)
#define P2U_PERIODIC_EQ_CTRL_GEN3_INIT_PRESET_EQ_TRAIN_EN	BIT(1)
#define P2U_PERIODIC_EQ_CTRL_GEN4	0xc4
#define P2U_PERIODIC_EQ_CTRL_GEN4_INIT_PRESET_EQ_TRAIN_EN	BIT(1)

#define P2U_CONTROL_GEN1					0x78
#define P2U_CONTROL_GEN1_ENABLE_RXIDLE_ENTRY_ON_LINK_STATUS	BIT(2)
#define P2U_CONTROL_GEN1_ENABLE_RXIDLE_ENTRY_ON_EIOS		BIT(3)

#define P2U_RX_DEBOUNCE_TIME				0xa4
#define P2U_RX_DEBOUNCE_TIME_DEBOUNCE_TIMER_MASK	0xFFFF
#define P2U_RX_DEBOUNCE_TIME_DEBOUNCE_TIMER_VAL		160

#define P2U_RX_MARGIN_SW_INT_EN				0xe0
#define P2U_RX_MARGIN_SW_INT_EN_READINESS		BIT(0)
#define P2U_RX_MARGIN_SW_INT_EN_MARGIN_START		BIT(1)
#define P2U_RX_MARGIN_SW_INT_EN_MARGIN_CHANGE		BIT(2)
#define P2U_RX_MARGIN_SW_INT_EN_MARGIN_STOP		BIT(3)

#define P2U_RX_MARGIN_SW_INT				0xe4
#define P2U_RX_MARGIN_SW_INT_MASK			0xf
#define P2U_RX_MARGIN_SW_INT_READINESS			BIT(0)
#define P2U_RX_MARGIN_SW_INT_MARGIN_START		BIT(1)
#define P2U_RX_MARGIN_SW_INT_MARGIN_CHANGE		BIT(2)
#define P2U_RX_MARGIN_SW_INT_MARGIN_STOP		BIT(3)

#define P2U_RX_MARGIN_SW_STATUS				0xe8
#define P2U_RX_MARGIN_SW_STATUS_MARGIN_SW_READY		BIT(0)
#define P2U_RX_MARGIN_SW_STATUS_MARGIN_READY		BIT(1)
#define P2U_RX_MARGIN_SW_STATUS_PHY_MARGIN_STATUS	BIT(2)
#define P2U_RX_MARGIN_SW_STATUS_PHY_MARGIN_ERROR_STATUS	BIT(3)

#define P2U_RX_MARGIN_CTRL				0xec
#define P2U_RX_MARGIN_CTRL_EN				BIT(0)
#define P2U_RX_MARGIN_CTRL_N_BLKS_MASK			GENMASK(21, 14)
#define P2U_RX_MARGIN_CTRL_N_BLKS_SHIFT			14

/* Any value between {0x80, 0xFF}, randomly selected 0x81 */
#define N_BLKS_COUNT	0x81

#define P2U_RX_MARGIN_STATUS				0xf0
#define P2U_RX_MARGIN_STATUS_ERRORS_MASK		GENMASK(15, 0)

#define P2U_RX_MARGIN_CONTROL				0xf0
#define P2U_RX_MARGIN_CONTROL_START			BIT(0)

#define RX_MARGIN_START_CHANGE	(1)
#define RX_MARGIN_STOP		(2)
#define RX_MARGIN_GET_MARGIN	(3)

struct tegra_p2u {
	void __iomem		*base;
	struct device		*dev;
	u32			id;
	struct work_struct	rx_margin_work;
	u32			next_state;
	spinlock_t		next_state_lock; /* lock for next_state */
	bool			enable_lm;
	bool			disable_uphy_rx_idle;
};

struct margin_ctrl {
	u32 en:1;
	u32 clr:1;
	u32 x:6;
	u32 y:6;
	u32 n_blks:8;
};

static int tegra_p2u_power_off(struct phy *x)
{
	return 0;
}

static int tegra_p2u_power_on(struct phy *x)
{
	u32 val;
	struct tegra_p2u *phy = phy_get_drvdata(x);

	if (phy->enable_lm) {
		val = P2U_RX_MARGIN_SW_INT_EN_READINESS |
		      P2U_RX_MARGIN_SW_INT_EN_MARGIN_START |
		      P2U_RX_MARGIN_SW_INT_EN_MARGIN_CHANGE |
		      P2U_RX_MARGIN_SW_INT_EN_MARGIN_STOP;
		writel(val, phy->base + P2U_RX_MARGIN_SW_INT_EN);
	}

	if (!phy->disable_uphy_rx_idle) {
		val = readl(phy->base + P2U_CONTROL_GEN1);
		val &= ~P2U_CONTROL_GEN1_ENABLE_RXIDLE_ENTRY_ON_EIOS;
		val |= P2U_CONTROL_GEN1_ENABLE_RXIDLE_ENTRY_ON_LINK_STATUS;
		writel(val, phy->base + P2U_CONTROL_GEN1);
	}

	val = readl(phy->base + P2U_PERIODIC_EQ_CTRL_GEN3);
	val &= ~P2U_PERIODIC_EQ_CTRL_GEN3_PERIODIC_EQ_EN;
	val |= P2U_PERIODIC_EQ_CTRL_GEN3_INIT_PRESET_EQ_TRAIN_EN;
	writel(val, phy->base + P2U_PERIODIC_EQ_CTRL_GEN3);

	val = readl(phy->base + P2U_PERIODIC_EQ_CTRL_GEN4);
	val |= P2U_PERIODIC_EQ_CTRL_GEN4_INIT_PRESET_EQ_TRAIN_EN;
	writel(val, phy->base + P2U_PERIODIC_EQ_CTRL_GEN4);

	val = readl(phy->base + P2U_RX_DEBOUNCE_TIME);
	val &= ~P2U_RX_DEBOUNCE_TIME_DEBOUNCE_TIMER_MASK;
	val |= P2U_RX_DEBOUNCE_TIME_DEBOUNCE_TIMER_VAL;
	writel(val, phy->base + P2U_RX_DEBOUNCE_TIME);

	return 0;
}

static int tegra_p2u_init(struct phy *x)
{
	return 0;
}

static int tegra_p2u_exit(struct phy *x)
{
	return 0;
}

static const struct phy_ops ops = {
	.init		= tegra_p2u_init,
	.exit		= tegra_p2u_exit,
	.power_on	= tegra_p2u_power_on,
	.power_off	= tegra_p2u_power_off,
	.owner		= THIS_MODULE,
};

static int set_margin_control(u32 id, u32 ctrl_data)
{
	struct mrq_uphy_request req;
	struct mrq_uphy_response resp;
	struct margin_ctrl ctrl;

	memcpy(&ctrl, &ctrl_data, sizeof(ctrl_data));

	req.lane = id;
	req.cmd = CMD_UPHY_PCIE_LANE_MARGIN_CONTROL;
	req.uphy_set_margin_control.en = ctrl.en;
	req.uphy_set_margin_control.clr = ctrl.clr;
	req.uphy_set_margin_control.x = ctrl.x;
	req.uphy_set_margin_control.y = ctrl.y;
	req.uphy_set_margin_control.nblks = ctrl.n_blks;

	return tegra_bpmp_send_receive(MRQ_UPHY, &req, sizeof(req),
				       &resp, sizeof(resp));
}

static int get_margin_status(u32 id, u32 *val)
{
	struct mrq_uphy_request req;
	struct mrq_uphy_response resp;
	int rc;

	req.lane = id;
	req.cmd = CMD_UPHY_PCIE_LANE_MARGIN_STATUS;

	rc = tegra_bpmp_send_receive(MRQ_UPHY, &req, sizeof(req), &resp,
				     sizeof(resp));
	*val = resp.uphy_get_margin_status.status;
	return (rc < 0) ? rc : 0;
}

void rx_margin_work_fn(struct work_struct *work)
{
	struct tegra_p2u *phy =
	    container_of(work, struct tegra_p2u, rx_margin_work);
	unsigned long flags;
	u32 val;
	int ret;
	u8 state;

	do {
		spin_lock_irqsave(&phy->next_state_lock, flags);
		state = phy->next_state;
		spin_unlock_irqrestore(&phy->next_state_lock, flags);
		switch (state) {
		case RX_MARGIN_START_CHANGE:
		case RX_MARGIN_STOP:
			val = readl(phy->base + P2U_RX_MARGIN_CTRL);
			ret = set_margin_control(phy->id, val);
			if (ret) {
				dev_err(phy->dev,
					"MARGIN_SET BPMP-FW SEND ERR\n");
				break;
			}
			val = readl(phy->base + P2U_RX_MARGIN_SW_STATUS);
			val |= P2U_RX_MARGIN_SW_STATUS_MARGIN_SW_READY;
			val |= P2U_RX_MARGIN_SW_STATUS_MARGIN_READY;
			val |= P2U_RX_MARGIN_SW_STATUS_PHY_MARGIN_STATUS;
			val |= P2U_RX_MARGIN_SW_STATUS_PHY_MARGIN_ERROR_STATUS;
			writel(val, phy->base + P2U_RX_MARGIN_SW_STATUS);

			udelay(10);

			if (state != RX_MARGIN_STOP) {
				spin_lock_irqsave(&phy->next_state_lock, flags);
				phy->next_state = RX_MARGIN_GET_MARGIN;
				spin_unlock_irqrestore(&phy->next_state_lock,
						       flags);
				continue;
			}

		case RX_MARGIN_GET_MARGIN:
			if (state != RX_MARGIN_STOP) {
				ret = get_margin_status(phy->id, &val);
				if (ret) {
					dev_err(phy->dev,
						"MARGIN_GET BPMP-FW RCV ERR\n");
					break;
				}
				writel(val & P2U_RX_MARGIN_STATUS_ERRORS_MASK,
				       phy->base + P2U_RX_MARGIN_STATUS);
			}
			val = readl(phy->base + P2U_RX_MARGIN_SW_STATUS);
			val |= P2U_RX_MARGIN_SW_STATUS_MARGIN_SW_READY;
			val |= P2U_RX_MARGIN_SW_STATUS_MARGIN_READY;
			val &= ~P2U_RX_MARGIN_SW_STATUS_PHY_MARGIN_STATUS;
			val |= P2U_RX_MARGIN_SW_STATUS_PHY_MARGIN_ERROR_STATUS;
			writel(val, phy->base + P2U_RX_MARGIN_SW_STATUS);

			if (state != RX_MARGIN_STOP) {
				msleep(20);
				continue;
			} else {
				return;
			}
			break;

		default:
			dev_err(phy->dev, "MARGIN INVALID STATE\n");
			return;
		};
	} while (1);
}

static irqreturn_t tegra_p2u_irq_handler(int irq, void *arg)
{
	struct tegra_p2u *phy = (struct tegra_p2u *)arg;
	unsigned long flags;
	u32 val = 0;

	val = readl(phy->base + P2U_RX_MARGIN_SW_INT);
	writel(val, phy->base + P2U_RX_MARGIN_SW_INT);
	switch (val & P2U_RX_MARGIN_SW_INT_MASK) {
	case P2U_RX_MARGIN_SW_INT_READINESS:
		dev_dbg(phy->dev, "Rx_Margin_intr : READINESS\n");
		val = readl(phy->base + P2U_RX_MARGIN_SW_STATUS);
		val |= P2U_RX_MARGIN_SW_STATUS_MARGIN_SW_READY;
		val |= P2U_RX_MARGIN_SW_STATUS_MARGIN_READY;
		writel(val, phy->base + P2U_RX_MARGIN_SW_STATUS);
		/* Write N_BLKS with any value between {0x80, 0xFF} */
		val = readl(phy->base + P2U_RX_MARGIN_CTRL);
		val &= P2U_RX_MARGIN_CTRL_N_BLKS_MASK;
		val |= (N_BLKS_COUNT << P2U_RX_MARGIN_CTRL_N_BLKS_SHIFT);
		writel(val, phy->base + P2U_RX_MARGIN_CTRL);
		break;

	case P2U_RX_MARGIN_SW_INT_MARGIN_STOP:
		dev_dbg(phy->dev, "Rx_Margin_intr : MARGIN_STOP\n");
		spin_lock_irqsave(&phy->next_state_lock, flags);
		phy->next_state = RX_MARGIN_STOP;
		spin_unlock_irqrestore(&phy->next_state_lock, flags);
		break;

	case P2U_RX_MARGIN_SW_INT_MARGIN_CHANGE:
	case (P2U_RX_MARGIN_SW_INT_MARGIN_CHANGE |
	      P2U_RX_MARGIN_SW_INT_MARGIN_START):
		dev_dbg(phy->dev, "Rx_Margin_intr : MARGIN_CHANGE\n");
		spin_lock_irqsave(&phy->next_state_lock, flags);
		phy->next_state = RX_MARGIN_START_CHANGE;
		spin_unlock_irqrestore(&phy->next_state_lock, flags);
		/* fallthrough */

	case P2U_RX_MARGIN_SW_INT_MARGIN_START:
		dev_dbg(phy->dev, "Rx_Margin_intr : MARGIN_START\n");
		schedule_work(&phy->rx_margin_work);
		break;

	default:
		dev_err(phy->dev, "INVALID Rx_Margin_intr : 0x%x\n", val);
		break;
	}

	return IRQ_HANDLED;
}

static int tegra_p2u_probe(struct platform_device *pdev)
{
	struct tegra_p2u *phy;
	struct phy *generic_phy;
	struct phy_provider *phy_provider;
	struct device *dev = &pdev->dev;
	struct resource *res;
	u32 val;
	int irq, ret = 0;

	phy = devm_kzalloc(dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;

	phy->dev = dev;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "base");
	phy->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(phy->base))
		return PTR_ERR(phy->base);

	platform_set_drvdata(pdev, phy);

	generic_phy = devm_phy_create(dev, NULL, &ops);
	if (IS_ERR(generic_phy))
		return PTR_ERR(generic_phy);

	phy_set_drvdata(generic_phy, phy);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider))
		return PTR_ERR(phy_provider);

	ret = of_property_read_u32(dev->of_node, "nvidia,uphy-id", &val);
	if (ret) {
		dev_err(dev, "uphy-id is missing\n");
		return ret;
	}
	phy->id = val;

	phy->enable_lm = of_property_read_bool(dev->of_node,
					       "nvidia,enable-lm");

	phy->disable_uphy_rx_idle =
		of_property_read_bool(dev->of_node,
				      "nvidia,disable-uphy-rx-idle");

	spin_lock_init(&phy->next_state_lock);
	INIT_WORK(&phy->rx_margin_work, rx_margin_work_fn);

	irq = platform_get_irq_byname(pdev, "intr");
	if (!irq) {
		dev_err(dev, "failed to get intr interrupt\n");
		return irq;
	}

	ret = devm_request_irq(&pdev->dev, irq, tegra_p2u_irq_handler, 0,
			       "tegra-p2u-intr", phy);
	if (ret) {
		dev_err(dev, "failed to request \"intr\" irq\n");
		return ret;
	}

	return 0;
}

static int tegra_p2u_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id tegra_p2u_id_table[] = {
	{
		.compatible = "nvidia,phy-p2u",
	},
	{}
};
MODULE_DEVICE_TABLE(of, tegra_p2u_id_table);

static struct platform_driver tegra_p2u_driver = {
	.probe		= tegra_p2u_probe,
	.remove		= tegra_p2u_remove,
	.driver		= {
		.name	= "tegra-p2u",
		.of_match_table = tegra_p2u_id_table,
	},
};

module_platform_driver(tegra_p2u_driver);

MODULE_AUTHOR("Vidya Sagar <vidyas@nvidia.com>");
MODULE_DESCRIPTION("Nvidia Tegra PIPE_To_UPHY phy driver");
MODULE_LICENSE("GPL v2");
