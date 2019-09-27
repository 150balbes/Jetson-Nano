/*
 * drivers/thermal/tegra_aotag.c
 *
 * TEGRA AOTAG (Always-On Thermal Alert Generator) driver.
 *
 * Copyright (c) 2014 - 2017, NVIDIA CORPORATION.  All rights reserved.
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

#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/thermal.h>
#include <linux/tsensor-fuse.h>
#include <soc/tegra/fuse.h>
#include <soc/tegra/pmc.h>

#define PDEV2DEVICE(pdev) (&(pdev->dev))
#define PDEV2SENSOR_INFO(pdev) \
	((struct aotag_sensor_info_t *)pdev->dev.platform_data)

#define REG_SET(r, mask, value)	\
	((r & ~(mask##_MASK)) | ((value<<(mask##_POS_START)) & mask##_MASK))

#define REG_GET(r, mask) \
	((r & mask##_MASK) >> mask##_POS_START)

#define MASK(start, end)	\
	(((0xFFFFFFFF)<<start) & (u32)(((u64)1<<(end+1))-1))

#define MAX_THRESHOLD_TEMP	(127000)
#define MIN_THRESHOLD_TEMP	(-127000)

/*
 * Register definitions
 */
#define TSENSOR_COMMON_FUSE_ADDR	(0x280)
#define AOTAG_FUSE_ADDR			(0x1D4)

#define PMC_R_OBS_AOTAG			(0x017)
#define PMC_R_OBS_AOTAG_CAPTURE		(0x017)
#define PMC_RST_STATUS			(0x1B4)
#define PMC_DIRECT_THERMTRIP_CFG	(0x474)

#define PMC_AOTAG_CFG			(0x484)
#define CFG_DISABLE_CLK_POS		(0)
#define CFG_SW_TRIGGER_POS		(2)
#define CFG_USE_EXT_TRIGGER_POS		(3)
#define CFG_ENABLE_SW_DEBUG_POS		(4)
#define CFG_TAG_EN_POS			(5)
#define CFG_THERMTRIP_EN_POS		(6)

#define PMC_AOTAG_THRESH1_CFG		(0x488)
#define PMC_AOTAG_THRESH2_CFG		(0x48C)
#define PMC_AOTAG_THRESH3_CFG		(0x490)
#define THRESH3_CFG_POS_START		(0)
#define THRESH3_CFG_POS_END		(14)
#define THRESH3_CFG_MASK		(MASK(THRESH3_CFG_POS_START,\
						THRESH3_CFG_POS_END))
#define PMC_AOTAG_STATUS		(0x494)
#define PMC_AOTAG_SECURITY		(0x498)

#define PMC_TSENSOR_CONFIG0		(0x49C)
#define CONFIG0_STOP_POS		(0)
#define CONFIG0_RO_SEL_POS		(1)
#define CONFIG0_STATUS_CLR_POS		(5)
#define CONFIG0_TALL_POS_START		(8)
#define CONFIG0_TALL_POS_END		(27)
#define CONFIG0_TALL_SIZE		(CONFIG0_TALL_POS_END - \
					CONFIG0_TALL_POS + 1)
#define CONFIG0_TALL_MASK		(MASK(CONFIG0_TALL_POS_START, \
						CONFIG0_TALL_POS_END))

#define PMC_TSENSOR_CONFIG1		(0x4A0)
#define CONFIG1_TEMP_ENABLE_POS		(31)
#define CONFIG1_TEN_COUNT_POS_START	(24)
#define CONFIG1_TEN_COUNT_POS_END	(29)
#define CONFIG1_TEN_COUNT_MASK		(MASK(CONFIG1_TEN_COUNT_POS_START, \
						CONFIG1_TEN_COUNT_POS_END))
#define CONFIG1_TIDDQ_EN_POS_START	(15)
#define CONFIG1_TIDDQ_EN_POS_END	(20)
#define CONFIG1_TIDDQ_EN_MASK		(MASK(CONFIG1_TIDDQ_EN_POS_START, \
						CONFIG1_TIDDQ_EN_POS_END))
#define CONFIG1_TSAMPLE_POS_START	(0)
#define CONFIG1_TSAMPLE_POS_END		(9)
#define CONFIG1_TSAMPLE_MASK		(MASK(CONFIG1_TSAMPLE_POS_START, \
						CONFIG1_TSAMPLE_POS_END))

#define PMC_TSENSOR_CONFIG2		(0x4A4)
#define CONFIG2_THERM_A_POS_START	(16)
#define CONFIG2_THERM_A_POS_END		(31)
#define CONFIG2_THERM_A_MASK		(MASK(CONFIG2_THERM_A_POS_START, \
						CONFIG2_THERM_A_POS_END))

#define CONFIG2_THERM_B_POS_START	(0)
#define CONFIG2_THERM_B_POS_END		(15)
#define CONFIG2_THERM_B_MASK		(MASK(CONFIG2_THERM_B_POS_START, \
						CONFIG2_THERM_B_POS_END))

#define PMC_TSENSOR_STATUS0		(0x4A8)
#define STATUS0_CAPTURE_VALID_POS	(31)
#define STATUS0_CAPTURE_POS_START	(0)
#define STATUS0_CAPTURE_POS_END		(15)
#define STATUS0_CAPTURE_MASK		(MASK(STATUS0_CAPTURE_POS_START, \
					STATUS0_CAPTURE_POS_END))

#define PMC_TSENSOR_STATUS1		(0x4AC)
#define STATUS1_TEMP_POS		(31)
#define STATUS1_TEMP_POS_START		(0)
#define STATUS1_TEMP_POS_END		(15)
#define STATUS1_TEMP_MASK		(MASK(STATUS1_TEMP_POS_START, \
					STATUS1_TEMP_POS_END))

#define STATUS1_TEMP_VALID_POS_START	(31)
#define STATUS1_TEMP_VALID_POS_END	(31)
#define STATUS1_TEMP_VALID_MASK		(MASK(STATUS1_TEMP_VALID_POS_START, \
					STATUS1_TEMP_VALID_POS_END))

#define STATUS1_TEMP_ABS_POS_START	(8)
#define STATUS1_TEMP_ABS_POS_END	(15)
#define STATUS1_TEMP_ABS_MASK		(MASK(STATUS1_TEMP_ABS_POS_START, \
					STATUS1_TEMP_ABS_POS_END))

#define STATUS1_TEMP_FRAC_POS_START	(7)
#define STATUS1_TEMP_FRAC_POS_END	(7)
#define STATUS1_TEMP_FRAC_MASK		(MASK(STATUS1_TEMP_FRAC_POS_START, \
					STATUS1_TEMP_FRAC_POS_END))

#define STATUS1_TEMP_SIGN_POS_START	(0)
#define STATUS1_TEMP_SIGN_POS_END	(0)
#define STATUS1_TEMP_SIGN_MASK		(MASK(STATUS1_TEMP_SIGN_POS_START, \
					STATUS1_TEMP_SIGN_POS_END))

#define PMC_TSENSOR_STATUS2		(0x4B0)
#define PMC_TSENSOR_PDIV0		(0x4B4)
#define TSENSOR_PDIV_POS_START		(12)
#define TSENSOR_PDIV_POS_END		(15)
#define TSENSOR_PDIV_MASK		(MASK(TSENSOR_PDIV_POS_START, \
						TSENSOR_PDIV_POS_END))

#define PMC_AOTAG_INTR_EN		(0x4B8)
#define PMC_AOTAG_INTR_DIS		(0x4BC)

#define AOTAG_FUSE_CALIB_CP_POS_START	(0)
#define AOTAG_FUSE_CALIB_CP_POS_END	(12)
#define AOTAG_FUSE_CALIB_CP_MASK	(MASK(AOTAG_FUSE_CALIB_CP_POS_START, \
						AOTAG_FUSE_CALIB_CP_POS_END))

#define AOTAG_FUSE_CALIB_FT_POS_START	(13)
#define AOTAG_FUSE_CALIB_FT_POS_END	(25)
#define AOTAG_FUSE_CALIB_FT_MASK	(MASK(AOTAG_FUSE_CALIB_FT_POS_START, \
						AOTAG_FUSE_CALIB_FT_POS_END))
/*
 * Data definitions
 */

/*
 * Mask/shift bits in FUSE_TSENSOR_COMMON and
 * FUSE_TSENSOR_COMMON, which are described in
 * tsensor-fuse.c
 */
static const struct tegra_tsensor_fuse tegra_aotag_fuse = {
	.fuse_base_cp_mask = 0x3ff << 11,
	.fuse_base_cp_shift = 11,
	.fuse_base_ft_mask = 0x7ff << 21,
	.fuse_base_ft_shift = 21,
	.fuse_shift_ft_mask = 0x1f << 6,
	.fuse_shift_ft_shift = 6,
	.fuse_spare_realignment = 0,
};

static struct tegra_tsensor_configuration tegra_aotag_config = {
	.tall = 76,
	.tiddq_en = 1,
	.ten_count = 16,
	.tsample = 9,
	.tsample_ate = 39,
	.pdiv = 8,
	.pdiv_ate = 8,
};

static struct tegra_tsensor_configuration tegra210b01_aotag_config = {
	.tall = 76,
	.tiddq_en = 1,
	.ten_count = 16,
	.tsample = 19,
	.tsample_ate = 39,
	.pdiv = 12,
	.pdiv_ate = 6,
};

static const struct fuse_corr_coeff tegra_aotag_coeff = {
	.alpha = 1063200,
	.beta = -6749000,
};

static const struct fuse_corr_coeff tegra210b01_aotag_coeff = {
	.alpha = 991100,
	.beta = 1096200,
};

struct aotag_platform_data {
	struct tegra_tsensor_configuration *config;
	const struct fuse_corr_coeff *coeff;
};

static struct aotag_platform_data tegra210_plat_data = {
	.config = &tegra_aotag_config,
	.coeff = &tegra_aotag_coeff,
};

static struct aotag_platform_data tegra210b01_plat_data = {
	.config = &tegra210b01_aotag_config,
	.coeff = &tegra210b01_aotag_coeff,
};

static const struct of_device_id tegra_aotag_of_match[] = {
	{	.compatible = "nvidia,tegra21x-aotag",
		.data = &tegra210_plat_data,
	},
	{	.compatible = "nvidia,tegra210b01-aotag",
		.data = &tegra210b01_plat_data,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, tegra_aotag_of_match);

struct aotag_sensor_info_t {
	u32 id;
	const char *name;
	struct tegra_tsensor_configuration *config;
	const struct tegra_tsensor_fuse *fuse;
	const struct fuse_corr_coeff *coeff;
	struct thermal_zone_device *tzd;
	s32 therm_a;
	s32 therm_b;
};

static void aotag_thermtrip_enable(struct aotag_sensor_info_t *info, int temp)
{
	unsigned long r = 0;

	if (temp > MAX_THRESHOLD_TEMP)
		temp = MAX_THRESHOLD_TEMP;
	else if (temp < MIN_THRESHOLD_TEMP)
		temp = MIN_THRESHOLD_TEMP;

	r = REG_SET(r, THRESH3_CFG, temp/1000);
	tegra_pmc_writel(r, PMC_AOTAG_THRESH3_CFG);

	r = tegra_pmc_readl(PMC_AOTAG_CFG);
	set_bit(CFG_THERMTRIP_EN_POS, &r);
	tegra_pmc_writel(r, PMC_AOTAG_CFG);
}

static int aotag_set_trip_temp(void *data, int trip, int trip_temp)
{
	int ret = 0;
	enum thermal_trip_type type;
	struct platform_device *pdev = (struct platform_device *)data;
	struct aotag_sensor_info_t *ps_info = PDEV2SENSOR_INFO(pdev);
	struct thermal_zone_device *tz = ps_info->tzd;

	ret = tz->ops->get_trip_type(tz, trip, &type);
	if (ret)
		return ret;

	if (type != THERMAL_TRIP_CRITICAL)
		return 0;

	aotag_thermtrip_enable(ps_info, trip_temp);

	return 0;
}

static int aotag_get_temp_generic(void *data, int *temp)
{
	int ret = 0;
	u32 regval = 0, abs = 0, fraction = 0, valid = 0, sign = 0;
	struct platform_device *pdev = (struct platform_device *) data;

	if (unlikely(!data)) {
		pr_err(" Invalid data pointer\n");
		*temp = (-66);
		ret = -EINVAL;
		goto out;
	}

	regval = tegra_pmc_readl(PMC_TSENSOR_STATUS1);
	valid = REG_GET(regval, STATUS1_TEMP_VALID);
	if (!valid) {
		*temp = -125;
		dev_info(&pdev->dev, "Invalid temp readout\n");
		goto out;
	}
	abs = REG_GET(regval, STATUS1_TEMP_ABS);
	fraction = REG_GET(regval, STATUS1_TEMP_FRAC);
	sign = REG_GET(regval, STATUS1_TEMP_SIGN);
	*temp = (abs*1000) + (fraction*500);
	if (sign)
		*temp = (-1) * (*temp);
out:
	return ret;
}

static int aotag_init(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node *pmc_np = NULL;
	struct aotag_sensor_info_t *info = NULL;
	const struct of_device_id *match;
	struct aotag_platform_data *pdata = NULL;

	match = of_match_node(tegra_aotag_of_match, pdev->dev.of_node);
	if (!match)
		return -ENODEV;

	pdata = (struct aotag_platform_data *)match->data;
	pmc_np = of_parse_phandle(pdev->dev.of_node, "parent-block", 0);
	if (unlikely(!pmc_np)) {
		dev_err(&pdev->dev, "PMC handle not found.\n");
		return -EINVAL;
	}

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (unlikely(!info)) {
		dev_err(&pdev->dev, "Unable to allocate platform memory\n");
		return -ENOMEM;
	}

	pdev->dev.platform_data = info;

	info->config = pdata->config;
	/* HW WAR for early parts. Account for incorrect ATE fusing during the
	 * early parts. */
	if (tegra_chip_get_revision() == TEGRA210B01_REVISION_A01) {
		u32 major, minor, rev;

		tegra_fuse_readl(FUSE_CP_REV, &rev);
		minor = rev & 0x1f;
		major = (rev >> 5) & 0x3f;
		if (major == 0 && minor < TEGRA_FUSE_CP_REV_0_3) {
			dev_info(&pdev->dev, "Applying WAR for CP rev:%d.%d\n",
				major, minor);
			info->config->tsample_ate -= 1;
		}
	}

	info->fuse = &tegra_aotag_fuse;
	info->coeff = pdata->coeff;

	return ret;
}

static int aotag_parse_sensor_params(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node *np =  pdev->dev.of_node;
	struct aotag_sensor_info_t *psensor_info = PDEV2SENSOR_INFO(pdev);

	ret = of_property_read_u32(np, "sensor-id", &psensor_info->id);
	if (ret)
		pr_err("aotag: no property sensor-id");

	ret = of_property_read_string(np, "sensor-name", &psensor_info->name);
	if (ret)
		pr_err("aotag: no property sensor-name");

	return ret;
}

static const struct thermal_zone_of_device_ops aotag_of_thermal_ops = {
	.get_temp = aotag_get_temp_generic,
	.set_trip_temp = aotag_set_trip_temp,
};

static int aotag_register_sensors(struct platform_device *pdev)
{
	int ret = 0, thermtrip;
	struct aotag_sensor_info_t *ps_info = PDEV2SENSOR_INFO(pdev);
	struct thermal_zone_device *ptz = NULL;

	dev_info(&pdev->dev, "Registering sensor %d\n", ps_info->id);
	ptz = thermal_zone_of_sensor_register(
			&pdev->dev,
			ps_info->id,
			pdev,
			&aotag_of_thermal_ops);
	if (IS_ERR(ptz)) {
		dev_err(&pdev->dev, "Failed to register aotag sensor\n");
		ret = PTR_ERR(ptz);
		return ret;
	}

	ps_info->tzd = ptz;
	dev_info(&pdev->dev, "Bound to TZ : ID %d\n", ps_info->tzd->id);

	ret = ptz->ops->get_crit_temp(ptz, &thermtrip);
	if (!ret)
		aotag_thermtrip_enable(ps_info, thermtrip);

	return ret;
}


static int aotag_calib_init(struct platform_device *pdev)
{
	int ret = 0;
	struct aotag_sensor_info_t *ps_info = PDEV2SENSOR_INFO(pdev);
	struct tsensor_shared_calib shared_fuses;
	u32 therm_ab;

	ret = tegra_calc_shared_calib(ps_info->fuse, &shared_fuses);
	if (ret)
		return ret;

	ret = tegra_calc_tsensor_calib(ps_info->config, &shared_fuses,
			ps_info->coeff, &therm_ab, AOTAG_FUSE_ADDR);
	if (ret)
		return ret;

	ps_info->therm_a = REG_GET(therm_ab, CONFIG2_THERM_A);
	ps_info->therm_b = REG_GET(therm_ab, CONFIG2_THERM_B);

	dev_dbg(&pdev->dev, "thermA:%d, thermB:%d\n",
			ps_info->therm_a, ps_info->therm_b);
	tegra_pmc_writel(therm_ab, PMC_TSENSOR_CONFIG2);

	return 0;
}

static int aotag_hw_init(struct platform_device *pdev)
{
	int ret = 0;
	unsigned long r = 0;
	struct aotag_sensor_info_t *i = PDEV2SENSOR_INFO(pdev);

	/* init CONFIG_0 registers */
	/* clear CONFIG0_STOP, CONFIG0_RO_SEL, CONFIG0_STATUS_CLR */
	r = 0;
	r = REG_SET(r, CONFIG0_TALL, i->config->tall);
	tegra_pmc_writel(r, PMC_TSENSOR_CONFIG0);

	/* init CONFIG_1 registers */
	r = 0;
	r = REG_SET(r, CONFIG1_TEN_COUNT, i->config->ten_count);
	r = REG_SET(r, CONFIG1_TIDDQ_EN, i->config->tiddq_en);
	/* tsample needs to be 1 less while programming the sensor. This
	 * accounts for how ATE counts the pulses.
	 */
	r = REG_SET(r, CONFIG1_TSAMPLE, (i->config->tsample - 1));
	set_bit(CONFIG1_TEMP_ENABLE_POS, &r);
	tegra_pmc_writel(r, PMC_TSENSOR_CONFIG1);

	/* init CONFIG_2 registers */
	r = 0;
	r = REG_SET(r, TSENSOR_PDIV, i->config->pdiv);
	tegra_pmc_writel(r, PMC_TSENSOR_PDIV0);

	/* Enable AOTAG*/
	r = 0;
	set_bit(CFG_TAG_EN_POS, &r);
	clear_bit(CFG_DISABLE_CLK_POS, &r);
	tegra_pmc_writel(r, PMC_AOTAG_CFG);

	return ret;
}

/*
 * Driver Initialization code begins here
 */

static int __init tegra_aotag_probe(struct platform_device *pdev)
{
	int ret = 0;

	if (unlikely(!pdev)) {
		pr_err("Probe failed\n");
		return -EINVAL;
	}

	if (unlikely(!(pdev->dev.of_node))) {
		ret = -EINVAL;
		dev_err(&pdev->dev, "OF node not found\n");
		goto out;
	}

	ret = aotag_init(pdev);
	if (unlikely(ret)) {
		dev_err(&pdev->dev, "failed aotag init\n");
		goto out;
	}

	ret = aotag_parse_sensor_params(pdev);
	if (unlikely(ret)) {
		dev_err(&pdev->dev, "failed sensor params read\n");
		goto out;
	}

	ret = aotag_calib_init(pdev);
	if (unlikely(ret)) {
		dev_err(&pdev->dev, "failed fuse init\n");
		goto out;
	}

	ret = aotag_hw_init(pdev);
	if (unlikely(ret)) {
		dev_err(&pdev->dev, "failed sensor registration\n");
		goto out;
	}

	ret = aotag_register_sensors(pdev);
	if (unlikely(ret)) {
		dev_err(&pdev->dev, "failed sensor registration\n");
		goto out;
	}

out:
	dev_info(&pdev->dev, "Probe done %s:%d\n",
			(ret ? "[FAILURE]" : "[SUCCESS]"), ret);
	return ret;
}

static struct platform_driver tegra_aotag_driver = {
	/*
	 * not using the .probe here to allow __init usage.
	 */
	.driver = {
		.name = "tegra_aotag",
		.owner = THIS_MODULE,
		.of_match_table = tegra_aotag_of_match,
	},
};

static int __init tegra_aotag_driver_init(void)
{
	return platform_driver_probe(&tegra_aotag_driver, tegra_aotag_probe);
}
module_init(tegra_aotag_driver_init);

MODULE_AUTHOR("NVIDIA Corp.");
MODULE_DESCRIPTION("Tegra AOTAG driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:tegra-aotag");
