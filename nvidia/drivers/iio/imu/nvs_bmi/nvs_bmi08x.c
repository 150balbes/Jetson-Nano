/* Copyright (c) 2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* NVS = NVidia Sensor framework */
/* Uses one of the NVS kernel modules: nvs_iio, nvs_input, nvs_relay, nvs_nv */
/* See nvs.h for documentation */
/* See nvs_on_change.c and nvs_on_change.h for documentation */

/* This driver supports these Bosch IMUs:
 * - BMI085
 * - BMI088
 */

/* This driver operates in one of two modes depending on if an interrupt is
 * defined:
 * 1. Without an interrupt the sensors are all set at the fastest enabled
 *    rate and polled at that rate using the register map.
 * 2. The interrupts defined allows the driver to use the FIFO and its
 *    features, e.g. support for independent ODRs. If both the accelerometer
 *    and gyroscope have the same interrupt defined, then they will be synced
 *    together with the shared interrupt.
 */

/* Device tree example:
 * IMPORTANT: If not using the device auto-detection mechanism,
 *            'compatible = "bmi,bmi08x";',
 *            then use the gyroscope I2C address in the I2C device structures
 *            and define the accelerometer I2C address via the device tree
 *            entry: accelerometer_i2c_addr.
 *
 *            Use the <sensor>_irq_gpio entries to define all the interrupts
 *            by defining the corresponding GPIO. Don't use the I2C structure.
 *
 * bmi088@69 {
 *   compatible = "bmi,bmi088";
 *   reg = <0x69>; // <-- gyroscope I2C address
 *   accelerometer_i2c_addr = <0x19>;
 *   accelerometer_irq_gpio = <&tegra_gpio TEGRA_GPIO(AA, 2) GPIO_ACTIVE_HIGH>;
 *   gyroscope_irq_gpio = <&tegra_gpio TEGRA_GPIO(I, 4) GPIO_ACTIVE_HIGH>;
 *   accelerometer_matrix    = [01 00 00 00 01 00 00 00 01];
 *   gyroscope_matrix        = [01 00 00 00 01 00 00 00 01];
 *   vdd-supply = <&spmic_sd3>;
 *   vdd_IO-supply = <&spmic_sd3>;
 * };
 */

#include <linux/device.h>
#include <linux/version.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of.h>
#include <linux/nvs.h>
#include <linux/nvs_on_change.h>
#include <linux/nvs_gte.h>

#define BMI_DRIVER_VERSION		(4)

#define BMI_VENDOR			"Bosch"
#define BMI_NAME_BMI085			"bmi085"
#define BMI_NAME_BMI088			"bmi088"
#define BMI_NAME			"bmi08x"
#define BMI_ACC_VERSION			(1)
#define BMI_GYR_VERSION			(1)
#define BMI_TMP_VERSION			(1)
#define BMI_ACC_BUF_SZ			(64)
#define BMI_GYR_BUF_SZ			(600) /* 6 (frame sz) * 100 (buf sz) */
#define BMI_ACC_SELF_TEST_LIMIT_X	(1000)
#define BMI_ACC_SELF_TEST_LIMIT_Y	(1000)
#define BMI_ACC_SELF_TEST_LIMIT_Z	(500)
#define BMI_ACC_SELFTEST_DELAY_MS	(50)
#define BMI_GYR_SELFTEST_DELAY_MS	(20)
#define BMI_GYR_SELFTEST_RD_LOOP_N	(10)
#define BMI_ACC_SOFTRESET_DELAY_MS	(1)
#define BMI_GYR_SOFTRESET_DELAY_MS	(30)
#define BMI_ACC_PM_DELAY_MS		(5)
#define BMI_GYR_PM_DELAY_MS		(30)
#define BMI_HW_DELAY_POR_MS		(10)
#define BMI_HW_DELAY_DEV_ON_US		(2)
#define BMI_HW_DELAY_DEV_OFF_US		(1000)
/* HW registers accelerometer */
#define BMI_REG_ACC_CHIP_ID		(0x00)
#define BMI_REG_ACC_CHIP_ID_POR		(0x1E)
#define BMI_REG_ACC_ERR_REG		(0x02)
#define BMI_REG_ACC_STATUS		(0x03)
#define BMI_REG_ACC_DATA		(0x12)
#define BMI_REG_ACC_X_LSB		(0x12)
#define BMI_REG_ACC_X_MSB		(0x13)
#define BMI_REG_ACC_Y_LSB		(0x14)
#define BMI_REG_ACC_Y_MSB		(0x15)
#define BMI_REG_ACC_Z_LSB		(0x16)
#define BMI_REG_ACC_Z_MSB		(0x17)
#define BMI_REG_ACC_DATA_N		(6)
#define BMI_REG_SENSORTIME_0		(0x18)
#define BMI_REG_SENSORTIME_1		(0x19)
#define BMI_REG_SENSORTIME_2		(0x1A)
#define BMI_REG_ACC_INT_STAT_1		(0x1D)
#define BMI_REG_TEMP_MSB		(0x22)
#define BMI_REG_TEMP_LSB		(0x23)
#define BMI_REG_FIFO_LENGTH_0		(0x24)
#define BMI_REG_FIFO_LENGTH_1		(0x25)
#define BMI_REG_ACC_FIFO_DATA		(0x26)
#define BMI_REG_ACC_CONF		(0x40)
#define BMI_REG_ACC_CONF_BWP_POR	(0xA0)
#define BMI_REG_ACC_CONF_BWP_MSK	(0xF0)
#define BMI_REG_ACC_RANGE		(0x41)
#define BMI_REG_FIFO_DOWNS		(0x45)
#define BMI_REG_FIFO_WTM_0		(0x46)
#define BMI_REG_FIFO_WTM_1		(0x47)
#define BMI_FIFO_FRAME_MAX		(BMI_REG_ACC_DATA_N + 1) /* +header */
#define BMI_REG_FIFO_WTM_MAX		(0x400 - (BMI_FIFO_FRAME_MAX * 2))
#define BMI_REG_ACC_FIFO_CFG_0		(0x48)
#define BMI_REG_ACC_FIFO_CFG_0_MSK	(0x01)
#define BMI_REG_ACC_FIFO_CFG_0_DFLT	(0x02)
#define BMI_REG_ACC_FIFO_CFG_1		(0x49)
#define BMI_REG_ACC_FIFO_CFG_1_MSK	(0x06)
#define BMI_REG_ACC_FIFO_CFG_1_DFLT	(0x50)
#define BMI_REG_INT1_IO_CTRL		(0x53)
#define BMI_REG_INT2_IO_CTRL		(0x54)
#define BMI_REG_INTX_IO_CTRL_OUT_EN	(0x08)
#define BMI_REG_INTX_IO_CTRL_OPEN_DRAIN	(0x04)
#define BMI_REG_INTX_IO_CTRL_ACTV_HI	(0x02)
#define BMI_REG_INT_MAP_DATA		(0x58)
#define BMI_REG_ACC_SELF_TEST		(0x6D)
#define BMI_REG_ACC_SELF_TEST_OFF	(0x00)
#define BMI_REG_ACC_SELF_TEST_POS	(0x0D)
#define BMI_REG_ACC_SELF_TEST_NEG	(0x09)
#define BMI_REG_ACC_PWR_CONF		(0x7C)
#define BMI_REG_ACC_PWR_CONF_ACTV	(0x00)
#define BMI_REG_ACC_PWR_CONF_SUSP	(0x03)
#define BMI_REG_ACC_PWR_CTRL		(0x7D)
#define BMI_REG_ACC_PWR_CTRL_OFF	(0x00)
#define BMI_REG_ACC_PWR_CTRL_ON		(0x04)
#define BMI_REG_ACC_SOFTRESET		(0x7E)
#define BMI_REG_ACC_SOFTRESET_FIFO	(0xB0)
#define BMI_REG_ACC_SOFTRESET_EXE	(0xB6)
/* HW registers gyroscope */
#define BMI_REG_GYR_CHIP_ID		(0x00)
#define BMI_REG_GYR_CHIP_ID_POR		(0x0F)
#define BMI_REG_GYR_DATA		(0x02)
#define BMI_REG_GYR_X_LSB		(0x02)
#define BMI_REG_GYR_X_MSB		(0x03)
#define BMI_REG_GYR_Y_LSB		(0x04)
#define BMI_REG_GYR_Y_MSB		(0x05)
#define BMI_REG_GYR_Z_LSB		(0x06)
#define BMI_REG_GYR_Z_MSB		(0x07)
#define BMI_REG_GYR_DATA_N		(6)
#define BMI_REG_GYR_INT_STAT_1		(0x0A)
#define BMI_REG_FIFO_STATUS		(0x0E)
#define BMI_REG_FIFO_STATUS_N_MSK	(0x7F)
#define BMI_REG_FIFO_STATUS_OVRRN_MSK	(0x80)
#define BMI_REG_GYR_RANGE		(0x0F)
#define BMI_REG_GYR_BW			(0x10)
#define BMI_REG_GYR_LPM1		(0x11)
#define BMI_REG_GYR_LPM1_NORM		(0x00)
#define BMI_REG_GYR_LPM1_SUSP		(0x80)
#define BMI_REG_GYR_LPM1_DEEP		(0x20)
#define BMI_REG_GYR_SOFTRESET		(0x14)
#define BMI_REG_GYR_SOFTRESET_EXE	(0xB6)
#define BMI_REG_GYR_INT_CTRL		(0x15)
#define BMI_REG_GYR_INT_CTRL_DIS	(0x00)
#define BMI_REG_GYR_INT_CTRL_DATA_EN	(0x80)
#define BMI_REG_GYR_INT_CTRL_FIFO_EN	(0x40)
#define BMI_REG_INT_3_4_IO_CONF		(0x16)
#define BMI_REG_INT_3_4_IO_CONF_3_HI	(0x01)
#define BMI_REG_INT_3_4_IO_CONF_3_OD	(0x02)
#define BMI_REG_INT_3_4_IO_CONF_4_HI	(0x04)
#define BMI_REG_INT_3_4_IO_CONF_4_OD	(0x08)
#define BMI_REG_INT_3_4_IO_MAP		(0x18)
#define BMI_REG_INT_3_4_IO_MAP_INT3	(0x05)
#define BMI_REG_INT_3_4_IO_MAP_INT4	(0xA0)
#define BMI_REG_FIFO_WM_ENABLE		(0x1E)
#define BMI_REG_FIFO_WM_ENABLE_DIS	(0x08)
#define BMI_REG_FIFO_WM_ENABLE_EN	(0x88)
#define BMI_REG_FIFO_EXT_INT_S		(0x34)
#define BMI_REG_GYR_SELF_TEST		(0x3C)
#define BMI_REG_GYR_SELF_TEST_EXE	(1 << 0)
#define BMI_REG_GYR_SELF_TEST_RDY	(1 << 1)
#define BMI_REG_GYR_SELF_TEST_FAIL	(1 << 2)
#define BMI_REG_GYR_SELF_TEST_OK	(1 << 3)
#define BMI_REG_GYR_FIFO_CFG_0		(0x3D)
#define BMI_REG_GYR_FIFO_CFG_0_WM_N	(98)
#define BMI_REG_GYR_FIFO_CFG_1		(0x3E)
#define BMI_REG_GYR_FIFO_CFG_1_FIFO	(0x40)
#define BMI_REG_GYR_FIFO_CFG_1_STREAM	(0x80)
#define BMI_REG_GYR_FIFO_DATA		(0x3F)
/* operational modes */
#define BMI_OP_MODE_POLL		(0)
#define BMI_OP_MODE_IRQ_SHARED		(1)
#define BMI_OP_MODE_IRQ			(2)
/* fastest speed with shared resources */
#define BMI_OP_MODE_SHARED_DELAY_US_MIN (2500)
/* degrees of freedom */
#define BMI_AXIS_X			(0)
#define BMI_AXIS_Y			(1)
#define BMI_AXIS_Z			(2)
#define BMI_AXIS_N			(3)
/* hardware devices */
#define BMI_HW_ACC			(0)
#define BMI_HW_GYR			(1)
#define BMI_HW_TMP			(2)
#define BMI_HW_N			(3)
/* switch toggles */
#define BMI_STS_SPEW_FIFO		(NVS_STS_EXT_N)
#define BMI_STS_SPEW_FIFO_ACC		(1 << (NVS_STS_EXT_N + BMI_HW_ACC))
#define BMI_STS_SPEW_FIFO_GYR		(1 << (NVS_STS_EXT_N + BMI_HW_GYR))
#define BMI_STS_SPEW_TS			(BMI_STS_SPEW_FIFO + BMI_HW_N)
#define BMI_STS_SPEW_TS_ACC		(1 << (BMI_STS_SPEW_TS + BMI_HW_ACC))
#define BMI_STS_SPEW_TS_GYR		(1 << (BMI_STS_SPEW_TS + BMI_HW_GYR))
#define BMI_STS_SPEW_ST			(BMI_STS_SPEW_TS + BMI_HW_N)
#define BMI_STS_SPEW_ST_ACC		(1 << (BMI_STS_SPEW_ST + BMI_HW_ACC))
#define BMI_STS_SPEW_ST_GYR		(1 << (BMI_STS_SPEW_ST + BMI_HW_GYR))

enum BMI_INF {
	BMI_INF_VER = 0,
	BMI_INF_DBG,
	BMI_INF_SPEW_FIFO,
	BMI_INF_SPEW_TS,
	BMI_INF_SPEW_ST,
	BMI_INF_REG_WR = 0xC6, /* use 0xD0 on cmd line */
	BMI_INF_REG_RD,
};

enum BMI_PART {
	BMI_PART_BMI085 = 0,
	BMI_PART_BMI088,
	BMI_PART_AUTO,
};
#define BMI_PART_N			(BMI_PART_AUTO)

static const struct i2c_device_id bmi_i2c_device_ids[] = {
	{ BMI_NAME_BMI085, BMI_PART_BMI085 },
	{ BMI_NAME_BMI088, BMI_PART_BMI088 },
	{ BMI_NAME, BMI_PART_AUTO },
	{},
};

/* regulator names in order of powering on */
static char *bmi_vregs[] = {
	"vdd",
	"vdd_IO",
};

static u16 bmi_i2c_addrs_acc[] = {
	0x18,
	0x19,
};

static u16 bmi_i2c_addrs_gyr[] = {
	0x68,
	0x69,
};

/* bmi_chip_ids_? must have the same number of entries as BMI_PART_N */
static u8 bmi_chip_ids_acc[] = {
	0x1F,
	0x1E,
};

static u8 bmi_chip_ids_gyr[] = {
	0x0F,
	0x0F,
};

static unsigned int bmi_hw_dpnd_msks[] = {
	[BMI_HW_ACC] = (1 << BMI_HW_ACC) | (1 << BMI_HW_TMP),
	[BMI_HW_GYR] = (1 << BMI_HW_GYR),
	[BMI_HW_TMP] = (1 << BMI_HW_ACC) | (1 << BMI_HW_TMP),
};

struct bmi_reg_rd {
	u8 reg_lo;
	u8 reg_hi;
};

static struct bmi_reg_rd bmi_reg_rds_acc[] = {
	{
		.reg_lo			= BMI_REG_ACC_CHIP_ID,
		.reg_hi			= BMI_REG_ACC_STATUS,
	},
	{
		.reg_lo			= BMI_REG_ACC_DATA,
		.reg_hi			= BMI_REG_SENSORTIME_2,
	},
	{
		.reg_lo			= BMI_REG_ACC_INT_STAT_1,
		.reg_hi			= BMI_REG_ACC_INT_STAT_1,
	},
	{
		.reg_lo			= BMI_REG_TEMP_LSB,
		.reg_hi			= BMI_REG_FIFO_LENGTH_1,
	},
	{
		.reg_lo			= BMI_REG_ACC_CONF,
		.reg_hi			= BMI_REG_ACC_RANGE,
	},
	{
		.reg_lo			= BMI_REG_FIFO_DOWNS,
		.reg_hi			= BMI_REG_ACC_FIFO_CFG_1,
	},
	{
		.reg_lo			= BMI_REG_INT1_IO_CTRL,
		.reg_hi			= BMI_REG_INT2_IO_CTRL,
	},
	{
		.reg_lo			= BMI_REG_INT_MAP_DATA,
		.reg_hi			= BMI_REG_INT_MAP_DATA,
	},
	{
		.reg_lo			= BMI_REG_ACC_SELF_TEST,
		.reg_hi			= BMI_REG_ACC_SELF_TEST,
	},
	{
		.reg_lo			= BMI_REG_ACC_PWR_CONF,
		.reg_hi			= BMI_REG_ACC_SOFTRESET,
	},
};

static struct bmi_reg_rd bmi_reg_rds_gyr[] = {
	{
		.reg_lo			= BMI_REG_GYR_CHIP_ID,
		.reg_hi			= BMI_REG_GYR_Z_MSB,
	},
	{
		.reg_lo			= BMI_REG_GYR_INT_STAT_1,
		.reg_hi			= BMI_REG_GYR_INT_STAT_1,
	},
	{
		.reg_lo			= BMI_REG_FIFO_STATUS,
		.reg_hi			= BMI_REG_GYR_LPM1,
	},
	{
		.reg_lo			= BMI_REG_GYR_SOFTRESET,
		.reg_hi			= BMI_REG_INT_3_4_IO_CONF,
	},
	{
		.reg_lo			= BMI_REG_INT_3_4_IO_MAP,
		.reg_hi			= BMI_REG_INT_3_4_IO_MAP,
	},
	{
		.reg_lo			= BMI_REG_FIFO_WM_ENABLE,
		.reg_hi			= BMI_REG_FIFO_WM_ENABLE,
	},
	{
		.reg_lo			= BMI_REG_FIFO_EXT_INT_S,
		.reg_hi			= BMI_REG_FIFO_EXT_INT_S,
	},
	{
		.reg_lo			= BMI_REG_GYR_SELF_TEST,
		.reg_hi			= BMI_REG_GYR_FIFO_CFG_1,
	},
};

static struct sensor_cfg bmi_snsr_cfgs[] = {
	{
		.name			= "accelerometer",
		.snsr_id		= BMI_HW_ACC,
		.kbuf_sz		= BMI_ACC_BUF_SZ,
		.ch_n			= BMI_AXIS_N,
		.ch_sz			= -2,
		.part			= BMI_NAME,
		.vendor			= BMI_VENDOR,
		.version		= BMI_ACC_VERSION,
		.max_range		= {
			.ival		= 0, /* default = +/-3g */
		},
		.milliamp		= {
			.ival		= 0,
			.fval		= 150000,
		},
		.delay_us_min		= 625,
		.delay_us_max		= 80000,
		.fifo_rsrv_evnt_cnt	= 0,
		.fifo_max_evnt_cnt	= 146,
		/* default matrix to get the attribute */
		.matrix[0]		= 1,
		.matrix[4]		= 1,
		.matrix[8]		= 1,
		.float_significance	= NVS_FLOAT_NANO,
	},
	{
		.name			= "gyroscope",
		.snsr_id		= BMI_HW_GYR,
		.kbuf_sz		= BMI_GYR_BUF_SZ,
		.ch_n			= BMI_AXIS_N,
		.ch_sz			= -2,
		.part			= BMI_NAME,
		.vendor			= BMI_VENDOR,
		.version		= BMI_GYR_VERSION,
		.max_range		= {
			.ival		= 0, /* default = +/-2000dps */
		},
		.milliamp		= {
			.ival		= 0,
			.fval		= 5000000,
		},
		.delay_us_min		= 500,
		.delay_us_max		= 10000,
		.fifo_rsrv_evnt_cnt	= 100,
		.fifo_max_evnt_cnt	= 100,
		/* default matrix to get the attribute */
		.matrix[0]		= 1,
		.matrix[4]		= 1,
		.matrix[8]		= 1,
		.float_significance	= NVS_FLOAT_NANO,
	},
	{
		.name			= "temperature",
		.snsr_id		= BMI_HW_TMP,
		.ch_n			= 1,
		.ch_sz			= -2,
		.part			= BMI_NAME,
		.vendor			= BMI_VENDOR,
		.version		= BMI_TMP_VERSION,
		.max_range		= {
			.ival		= 150,
			.fval		= 0,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 125000,
		},
		.milliamp		= {
			.ival		= 0,
			.fval		= 150000,
		},
		.delay_us_min		= 0,
		.delay_us_max		= 1280000,
		.flags			= SENSOR_FLAG_ON_CHANGE_MODE,
		.thresh_lo		= 1,
		.thresh_hi		= 1,
		.float_significance	= NVS_FLOAT_MICRO,
		.scale			= {
			.ival		= 0,
			.fval		= 125000,
		},
		.offset			= {
			.ival		= 23,
			.fval		= 0,
		},
	},
};

struct bmi_rr {
	struct nvs_float max_range;
	struct nvs_float resolution;
};

static struct bmi_rr bmi_rr_acc_bmi085[] = {
/* all accelerometer values are in g's (9.80665 m/s2) fval = NVS_FLOAT_NANO */
	{
		.max_range		= {
			.ival		= 19,
			.fval		= 613300000,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 598550,
		},
	},
	{
		.max_range		= {
			.ival		= 39,
			.fval		= 226600000,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 1197101,
		},
	},
	{
		.max_range		= {
			.ival		= 78,
			.fval		= 453200000,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 2394202,
		},
	},
	{
		.max_range		= {
			.ival		= 156,
			.fval		= 906400000,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 4788403,
		},
	},
};

static struct bmi_rr bmi_rr_acc_bmi088[] = {
/* all accelerometer values are in g's (9.80665 m/s2) fval = NVS_FLOAT_NANO */
	{
		.max_range		= {
			.ival		= 29,
			.fval		= 419950000,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 897826,
		},
	},
	{
		.max_range		= {
			.ival		= 58,
			.fval		= 839900000,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 1795651,
		},
	},
	{
		.max_range		= {
			.ival		= 117,
			.fval		= 679800000,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 3591302,
		},
	},
	{
		.max_range		= {
			.ival		= 235,
			.fval		= 359600000,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 3591302,
		},
	},
};

static struct bmi_rr bmi_rr_gyr[] = {
/* rad / sec  fval = NVS_FLOAT_NANO */
	{
		.max_range		= {
			.ival		= 34,
			.fval		= 906585040,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 1064225,
		},
	},
	{
		.max_range		= {
			.ival		= 17,
			.fval		= 453292520,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 532113,
		},
	},
	{
		.max_range		= {
			.ival		= 8,
			.fval		= 726646260,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 266462,
		},
	},
	{
		.max_range		= {
			.ival		= 4,
			.fval		= 363323130,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 133231,
		},
	},
	{
		.max_range		= {
			.ival		= 2,
			.fval		= 181661565,
		},
		.resolution		= {
			.ival		= 0,
			.fval		= 066615,
		},
	},
};

struct bmi_rrs {
	struct bmi_rr *rr;
	unsigned int rr_0n;
};

static struct bmi_rrs bmi_rrs_acc[] = {
	{
		.rr			= bmi_rr_acc_bmi085,
		.rr_0n			= ARRAY_SIZE(bmi_rr_acc_bmi085) - 1,
	},
	{
		.rr			= bmi_rr_acc_bmi088,
		.rr_0n			= ARRAY_SIZE(bmi_rr_acc_bmi088) - 1,
	},
};

static struct bmi_rrs bmi_rrs_gyr[] = {
	{
		.rr			= bmi_rr_gyr,
		.rr_0n			= ARRAY_SIZE(bmi_rr_gyr) - 1,
	},
	{
		.rr			= bmi_rr_gyr,
		.rr_0n			= ARRAY_SIZE(bmi_rr_gyr) - 1,
	},
};

struct bmi_state;
static int bmi_acc_able(struct bmi_state *st, int en);
static int bmi_acc_batch(struct bmi_state *st, unsigned int *odr_us,
			 unsigned int period_us, unsigned int timeout_us);
static int bmi_acc_softreset(struct bmi_state *st, unsigned int hw);
static int bmi_acc_selftest(struct bmi_state *st);
static int bmi_acc_pm(struct bmi_state *st, unsigned int hw, int able);
static unsigned long bmi_acc_irqflags(struct bmi_state *st);
static int bmi_gyr_able(struct bmi_state *st, int en);
static int bmi_gyr_batch(struct bmi_state *st, unsigned int *odr_us,
			 unsigned int period_us, unsigned int timeout_us);
static int bmi_gyr_softreset(struct bmi_state *st, unsigned int hw);
static int bmi_gyr_selftest(struct bmi_state *st);
static int bmi_gyr_pm(struct bmi_state *st, unsigned int hw, int able);
static unsigned long bmi_gyr_irqflags(struct bmi_state *st);
static int bmi_tmp_able(struct bmi_state *st, int en);
static int bmi_tmp_batch(struct bmi_state *st, unsigned int *odr_us,
			 unsigned int period_us, unsigned int timeout_us);

struct bmi_hw {
	struct bmi_reg_rd *reg_rds;
	struct bmi_rrs *rrs;
	u16 *i2c_addrs;
	u8 *chip_ids;
	u8 chip_id_reg;
	unsigned int reg_rds_n;
	unsigned int rrs_0n;
	unsigned int i2c_addrs_n;
	int (*fn_able)(struct bmi_state *st, int en);
	int (*fn_batch)(struct bmi_state *st, unsigned int *odr_us,
			unsigned int period_us, unsigned int timeout_us);
	int (*fn_softreset)(struct bmi_state *st, unsigned int hw);
	int (*fn_selftest)(struct bmi_state *st);
	int (*fn_pm)(struct bmi_state *st, unsigned int hw, int able);
	unsigned long (*fn_irqflags)(struct bmi_state *st);
};

static struct bmi_hw bmi_hws[] = {
	{
		.reg_rds		= bmi_reg_rds_acc,
		.rrs			= bmi_rrs_acc,
		.i2c_addrs		= bmi_i2c_addrs_acc,
		.chip_ids		= bmi_chip_ids_acc,
		.chip_id_reg		= BMI_REG_ACC_CHIP_ID,
		.reg_rds_n		= ARRAY_SIZE(bmi_reg_rds_acc),
		.rrs_0n			= ARRAY_SIZE(bmi_rrs_acc) - 1,
		.i2c_addrs_n		= ARRAY_SIZE(bmi_i2c_addrs_acc),
		.fn_able		= &bmi_acc_able,
		.fn_batch		= &bmi_acc_batch,
		.fn_softreset		= &bmi_acc_softreset,
		.fn_selftest		= &bmi_acc_selftest,
		.fn_pm			= &bmi_acc_pm,
		.fn_irqflags		= &bmi_acc_irqflags,
	},
	{
		.reg_rds		= bmi_reg_rds_gyr,
		.rrs			= bmi_rrs_gyr,
		.i2c_addrs		= bmi_i2c_addrs_gyr,
		.chip_ids		= bmi_chip_ids_gyr,
		.chip_id_reg		= BMI_REG_GYR_CHIP_ID,
		.reg_rds_n		= ARRAY_SIZE(bmi_reg_rds_gyr),
		.rrs_0n			= ARRAY_SIZE(bmi_rrs_gyr) - 1,
		.i2c_addrs_n		= ARRAY_SIZE(bmi_i2c_addrs_gyr),
		.fn_able		= &bmi_gyr_able,
		.fn_batch		= &bmi_gyr_batch,
		.fn_softreset		= &bmi_gyr_softreset,
		.fn_selftest		= &bmi_gyr_selftest,
		.fn_pm			= &bmi_gyr_pm,
		.fn_irqflags		= &bmi_gyr_irqflags,
	},
	{
		.reg_rds		= bmi_reg_rds_acc,
		.rrs			= NULL,
		.i2c_addrs		= bmi_i2c_addrs_acc,
		.chip_ids		= bmi_chip_ids_acc,
		.chip_id_reg		= BMI_REG_ACC_CHIP_ID,
		.reg_rds_n		= ARRAY_SIZE(bmi_reg_rds_acc),
		.rrs_0n			= 0,
		.i2c_addrs_n		= ARRAY_SIZE(bmi_i2c_addrs_acc),
		.fn_able		= &bmi_tmp_able,
		.fn_batch		= &bmi_tmp_batch,
		.fn_softreset		= &bmi_acc_softreset,
		.fn_selftest		= NULL,
		.fn_pm			= &bmi_acc_pm,
		.fn_irqflags		= NULL,
	},
};

struct bmi_snsr {
	void *nvs_st;
	struct bmi_rrs *rrs;
	struct sensor_cfg cfg;
	unsigned int hw;
	unsigned int usr_cfg;
	unsigned int odr_us;
	unsigned int period_us;
	unsigned int timeout_us;
	bool timeout_en;
};

struct bmi_state {
	struct i2c_client *i2c;
	struct nvs_fn_if *nvs;
	struct bmi_snsr snsrs[BMI_HW_N];
	struct nvs_gte_irq gis[BMI_HW_N];
	struct regulator_bulk_data vreg[ARRAY_SIZE(bmi_vregs)];
	struct workqueue_struct *wq;
	struct work_struct ws;
	struct nvs_on_change oc;	/* on-change temperature sensor*/
	bool irq_setup_done;		/* irq probe status: true=setup done */
	unsigned int part;		/* part index */
	unsigned int sts;		/* status flags */
	unsigned int errs;		/* error count */
	unsigned int inf;		/* NVS rd/wr */
	unsigned int enabled;		/* enable status */
	unsigned int period_us;		/* global period */
	unsigned int period_us_max;	/* maximum global period */
	unsigned int snsr_t;		/* HW sensor time */
	unsigned int frame_n;		/* sensor time frame count */
	unsigned int lost_frame_n;	/* frames lost to FIFO overflow */
	unsigned int hw_n;		/* sensor count */
	unsigned int hw2ids[BMI_HW_N];	/* sensor ids */
	unsigned int hw_en;		/* for HW access tracking */
	unsigned int buf_acc_n;		/* acc buffer size */
	unsigned int buf_gyr_n;		/* gyr buffer size */
	unsigned int op_mode;		/* operational mode */
	atomic64_t ts_irq[BMI_HW_N];	/* interrupt timestamp */
	s64 ts_st_irq;			/* sensor time IRQ timestamp */
	s64 ts_lo;			/* timestamp threshold low */
	s64 ts_hi;			/* timestamp threshold high */
	s64 ts[BMI_HW_N];		/* timestamp */
	s64 ts_odr[BMI_HW_N];		/* timestamp ODR */
	s64 ts_hw[BMI_HW_N];		/* timestamp HW access */
	u8 ra_0x40;			/* user cfg */
	u8 ra_0x48;			/* FIFO cfg */
	u8 ra_0x49;			/* " */
	u8 ra_0x53;			/* user interrupt cfg */
	u8 ra_0x54;			/* " */
	u8 ra_0x58;			/* " */
	u8 rg_0x16;			/* " */
	u8 rg_0x18;			/* " */
	u8 rg_0x34;			/* " */
	u16 i2c_addrs[BMI_HW_N];	/* I2C addresses */
	u16 buf_acc_i;			/* ACC data buffer index */
	u8 *buf_acc;			/* ACC data buffer */
	u8 *buf_gyr;			/* GYR data buffer */
};


static int bmi_mutex_lock(struct bmi_state *st)
{
	unsigned int i;

	if (st->nvs) {
		for (i = 0; i < st->hw_n; i++)
			st->nvs->nvs_mutex_lock(st->snsrs[i].nvs_st);
	}

	return 0;
}

static int bmi_mutex_unlock(struct bmi_state *st)
{
	unsigned int i;

	if (st->nvs) {
		for (i = 0; i < st->hw_n; i++)
			st->nvs->nvs_mutex_unlock(st->snsrs[i].nvs_st);
	}

	return 0;
}

static int bmi_err(struct bmi_state *st)
{
	st->errs++;
	if (!st->errs)
		st->errs--;
	return 0;
}

static int bmi_i2c_rd(struct bmi_state *st, unsigned int hw,
		      u8 reg, u16 len, u8 *buf)
{
	struct i2c_msg msg[2];
	int ret = -ENODEV;
	s64 ts;

	if (st->i2c_addrs[hw]) {
		msg[0].addr = st->i2c_addrs[hw];
		msg[0].flags = 0;
		msg[0].len = 1;
		msg[0].buf = &reg;
		msg[1].addr = st->i2c_addrs[hw];
		msg[1].flags = I2C_M_RD;
		msg[1].len = len;
		msg[1].buf = buf;
		ts = st->ts_hw[hw];
		if (st->hw_en & bmi_hw_dpnd_msks[hw])
			ts += (BMI_HW_DELAY_DEV_ON_US * 1000);
		else
			ts += (BMI_HW_DELAY_DEV_OFF_US * 1000);
		ts -= nvs_timestamp();
		if (ts > 0) {
			ts /= 1000;
			ts++;
			udelay(ts);
		}
		ret = i2c_transfer(st->i2c->adapter, msg, 2);
		st->ts_hw[hw] = nvs_timestamp();
		if (ret != 2) {
			bmi_err(st);
			if (st->sts & NVS_STS_SPEW_MSG)
				dev_err(&st->i2c->dev, "%s ERR: reg 0x%02X\n",
					__func__, reg);
			ret = -EIO;
		} else {
			ret = 0;
		}
	}

	return ret;
}

static int bmi_i2c_w(struct bmi_state *st, unsigned int hw,
		     u16 len, u8 *buf)
{
	struct i2c_msg msg;
	int ret;
	s64 ts;

	if (st->i2c_addrs[hw]) {
		msg.addr = st->i2c_addrs[hw];
		msg.flags = 0;
		msg.len = len;
		msg.buf = buf;
		ts = st->ts_hw[hw];
		if (st->hw_en & bmi_hw_dpnd_msks[hw])
			ts += (BMI_HW_DELAY_DEV_ON_US * 1000);
		else
			ts += (BMI_HW_DELAY_DEV_OFF_US * 1000);
		ts -= nvs_timestamp();
		if (ts > 0) {
			ts /= 1000; /* ns => us */
			ts++;
			if (st->sts & NVS_STS_SPEW_MSG)
				dev_info(&st->i2c->dev,
					 "%s %s HW access delay=%lldus\n",
					 __func__, bmi_snsr_cfgs[hw].name, ts);
			udelay(ts);
		}
		ret = i2c_transfer(st->i2c->adapter, &msg, 1);
		st->ts_hw[hw] = nvs_timestamp();
		if (ret != 1) {
			bmi_err(st);
			ret = -EIO;
		} else {
			ret = 0;
		}
	} else {
		ret = -ENODEV;
	}
	return ret;
}

static int bmi_i2c_wr(struct bmi_state *st, unsigned int hw, u8 reg, u8 val)
{
	int ret;
	u8 buf[2];

	buf[0] = reg;
	buf[1] = val;
	ret = bmi_i2c_w(st, hw, sizeof(buf), buf);
	if (st->sts & NVS_STS_SPEW_MSG) {
		if (ret)
			dev_err(&st->i2c->dev, "%s ERR: 0x%02X=>0x%02X\n",
				__func__, val, reg);
		else
			dev_info(&st->i2c->dev, "%s 0x%02X=>0x%02X\n",
				 __func__, val, reg);
	}
	return ret;
}

static int bmi_i2c_wr16(struct bmi_state *st, unsigned int hw, u8 reg, u16 val)
{
	int ret;
	u8 buf[3];

	buf[0] = reg;
	buf[1] = val & 0xFF;
	buf[2] = val >> 8;
	ret = bmi_i2c_w(st, hw, sizeof(buf), buf);
	if (st->sts & NVS_STS_SPEW_MSG) {
		if (ret)
			dev_err(&st->i2c->dev, "%s ERR: 0x%04X=>0x%02X\n",
				__func__, val, reg);
		else
			dev_info(&st->i2c->dev, "%s 0x%04X=>0x%02X\n",
				 __func__, val, reg);
	}
	return ret;
}

static int bmi_pm(struct bmi_state *st, int snsr_id, bool en)
{
	unsigned int hw_en = st->hw_en;
	unsigned int hw;
	unsigned int i;
	int ret = 0;

	if (en) {
		if (!hw_en) {
			ret = nvs_vregs_enable(&st->i2c->dev, st->vreg,
					       ARRAY_SIZE(bmi_vregs));
			if (ret > 0)
				mdelay(BMI_HW_DELAY_POR_MS);
		}
		if (snsr_id < 0) {
			ret = 0;
			for (i = 0; i < st->hw_n; i++) {
				hw = st->snsrs[i].hw;
				if (st->hw_en & bmi_hw_dpnd_msks[hw]) {
					st->hw_en |= (1 << hw);
				} else  {
					ret |= bmi_hws[hw].fn_softreset(st,
									hw);
					ret |= bmi_hws[hw].fn_pm(st, hw, 1);
				}
			}
		} else {
			hw = st->snsrs[snsr_id].hw;
			if (hw_en & (bmi_hw_dpnd_msks[hw] & ~(1 << hw))) {
				st->hw_en |= (1 << hw);
			} else  {
				ret = bmi_hws[hw].fn_softreset(st, hw);
				ret |= bmi_hws[hw].fn_pm(st, hw, 1);
			}
		}
	} else {
		ret = nvs_vregs_sts(st->vreg, ARRAY_SIZE(bmi_vregs));
		if ((ret < 0) || (ret == ARRAY_SIZE(bmi_vregs))) {
			/* we're fully powered */
			if (snsr_id < 0) {
				ret = 0;
				for (i = 0; i < st->hw_n; i++) {
					hw = st->snsrs[i].hw;
					ret |= bmi_hws[hw].fn_pm(st, hw, 0);
				}
			} else {
				hw = st->snsrs[snsr_id].hw;
				if ((hw_en &
				     bmi_hw_dpnd_msks[hw]) & ~(1 << hw)) {
					st->enabled &= ~(1 << snsr_id);
					st->hw_en &= ~(1 << hw);
				} else {
					ret = bmi_hws[hw].fn_pm(st, hw, 0);
				}
			}
		} else if (ret > 0) {
			/* partially powered so go to full before disables */
			ret = nvs_vregs_enable(&st->i2c->dev, st->vreg,
					       ARRAY_SIZE(bmi_vregs));
			mdelay(BMI_HW_DELAY_POR_MS);
			if (snsr_id < 0) {
				for (i = 0; i < st->hw_n; i++) {
					hw = st->snsrs[i].hw;
					ret |= bmi_hws[hw].fn_pm(st, hw, 0);
				}
			} else {
				hw = st->snsrs[snsr_id].hw;
				if ((hw_en &
				     bmi_hw_dpnd_msks[hw]) & ~(1 << hw)) {
					st->enabled &= ~(1 << snsr_id);
					st->hw_en &= ~(1 << hw);
				} else {
					ret |= bmi_hws[hw].fn_pm(st, hw, 0);
				}
			}
		}
		/* disables put us in low power sleep state in case no vregs */
		if (!st->hw_en) /* pull plug if all devices are low power */
			ret |= nvs_vregs_disable(&st->i2c->dev, st->vreg,
						 ARRAY_SIZE(bmi_vregs));
	}
	if (ret > 0)
		ret = 0;
	if (ret) {
		if (snsr_id < 0)
			dev_err(&st->i2c->dev, "%s ALL pm_en=%x  ERR=%d\n",
				__func__, en, ret);
		else
			dev_err(&st->i2c->dev, "%s %s pm_en=%x  ERR=%d\n",
				__func__, st->snsrs[snsr_id].cfg.name,
				en, ret);
	} else if (st->sts & NVS_STS_SPEW_MSG && hw_en != st->hw_en) {
		dev_info(&st->i2c->dev, "%s hw_en: 0x%X=>0x%XS\n",
			 __func__, hw_en, st->hw_en);
	}
	return ret;
}

static int bmi_pm_exit(struct bmi_state *st)
{
	bmi_pm(st, -1, false);
	nvs_vregs_exit(&st->i2c->dev, st->vreg, ARRAY_SIZE(bmi_vregs));
	return 0;
}

static int bmi_pm_init(struct bmi_state *st)
{
	int ret;

	nvs_vregs_init(&st->i2c->dev,
		       st->vreg, ARRAY_SIZE(bmi_vregs), (char **)bmi_vregs);
	ret = bmi_pm(st, -1, true);
	return ret;
}

static int bmi_hw_off(struct bmi_state *st, unsigned int hw)
{
	unsigned int snsr_id;
	unsigned int hw_msk;
	unsigned int i;

	hw_msk = 1;
	for (i = 0; i < BMI_HW_N; i++) {
		if (bmi_hw_dpnd_msks[hw] & hw_msk) {
			st->hw_en &= ~hw_msk;
			snsr_id = st->hw2ids[i];
			if (snsr_id < BMI_HW_N)
				st->enabled &= ~(1 << snsr_id);
		}
		hw_msk <<= 1;
	}

	return 0;
}

static unsigned int bmi_buf2sensortime(u8 *buf)
{
	unsigned int snsr_t = 0;

	snsr_t |= buf[2];
	snsr_t <<= 8;
	snsr_t |= buf[1];
	snsr_t <<= 8;
	snsr_t |= buf[0];
	return snsr_t;
}

static unsigned int bmi_sensortime_rd(struct bmi_state *st,
				      unsigned int *snsr_t)
{
	u8 buf[3];
	int ret;

	ret = bmi_i2c_rd(st, BMI_HW_ACC, BMI_REG_SENSORTIME_0,
			 sizeof(buf), buf);
	if (!ret)
		*snsr_t = bmi_buf2sensortime(buf);
	return ret;
}

struct bmi_odr {
	unsigned int period_us;
	u8 hw;
};

static unsigned int bmi_odr_i(struct bmi_odr *odrs, unsigned int odrs_n,
			      unsigned int period_us)
{
	unsigned int i;
	unsigned int n;

	n = odrs_n;
	n--;
	for (i = 0; i < n; i++) {
		if (period_us >= odrs[i].period_us)
			break;
	}

	return i;
}

static struct bmi_odr bmi_odrs_acc[] = {
	{ 80000, 0x05 },
	{ 40000, 0x06 },
	{ 20000, 0x07 },
	{ 10000, 0x08 },
	{ 5000,  0x09 },
	{ 2500,  0x0A },
	{ 1250,  0x0B },
	{ 625,   0x0C },
};

static int bmi_acc_batch(struct bmi_state *st, unsigned int *odr_us,
			 unsigned int period_us, unsigned int timeout_us)
{
	u8 val;
	u16 val16;
	unsigned int i;
	unsigned int odr_i;
	int ret = 0;

	odr_i = bmi_odr_i(bmi_odrs_acc, ARRAY_SIZE(bmi_odrs_acc), period_us);
	if (odr_us) {
		*odr_us = bmi_odrs_acc[odr_i].period_us;
	} else if (st->hw_en & bmi_hw_dpnd_msks[BMI_HW_ACC]) {
		/* power is up */
		val = bmi_odrs_acc[odr_i].hw;
		val |= st->ra_0x40;
		ret = bmi_i2c_wr(st, BMI_HW_ACC, BMI_REG_ACC_CONF, val);
		i = st->hw2ids[BMI_HW_ACC];
		if ((i < BMI_HW_N) && !ret) {
			st->snsrs[i].timeout_en = false;
			st->snsrs[i].odr_us = bmi_odrs_acc[odr_i].period_us;
			st->ts_odr[BMI_HW_ACC] = (s64)st->snsrs[i].odr_us *
									  1000;
			if (timeout_us) {
				/* calculate batch timeout */
				timeout_us /= bmi_odrs_acc[odr_i].period_us;
				timeout_us *= BMI_FIFO_FRAME_MAX;
				if (timeout_us > BMI_REG_FIFO_WTM_MAX)
					timeout_us = BMI_REG_FIFO_WTM_MAX;
				val16 = timeout_us;
				ret = bmi_i2c_wr16(st, BMI_HW_ACC,
						   BMI_REG_FIFO_WTM_0, val16);
				if (!ret)
					st->snsrs[i].timeout_en = true;
			}
		}
	}
	return ret;
}

static int bmi_acc_able(struct bmi_state *st, int en)
{
	u8 val;
	unsigned int i;
	int ret;

	if (en) {
		i = st->hw2ids[BMI_HW_ACC];
		if (i < BMI_HW_N)
			val = st->snsrs[i].usr_cfg;
		else
			val = 3; /* selftest value if ACC not populated */
		ret = bmi_i2c_wr(st, BMI_HW_ACC, BMI_REG_ACC_RANGE, val);
		st->frame_n = 0;
		st->ts_hi = 0;
		ret |= bmi_i2c_wr(st, BMI_HW_ACC, BMI_REG_ACC_PWR_CTRL,
				  BMI_REG_ACC_PWR_CTRL_ON);
		ret |= bmi_sensortime_rd(st, &st->snsr_t);
	} else {
		ret = bmi_i2c_wr(st, BMI_HW_ACC, BMI_REG_ACC_PWR_CTRL,
				 BMI_REG_ACC_PWR_CTRL_OFF);
	}
	return ret;
}

static int bmi_acc_softreset(struct bmi_state *st, unsigned int hw)
{
	int ret;

	ret = bmi_i2c_wr(st, BMI_HW_ACC, BMI_REG_ACC_SOFTRESET,
			 BMI_REG_ACC_SOFTRESET_EXE);
	mdelay(BMI_ACC_SOFTRESET_DELAY_MS);
	bmi_hw_off(st, hw);
	return ret;
}

static int bmi_acc_selftest(struct bmi_state *st)
{
	int16_t x_pos;
	int16_t y_pos;
	int16_t z_pos;
	int16_t x_neg;
	int16_t y_neg;
	int16_t z_neg;
	unsigned int i;
	unsigned int usr_cfg;
	int ret;

	i = st->hw2ids[BMI_HW_ACC];
	usr_cfg = st->snsrs[i].usr_cfg;
	ret = bmi_pm(st, i, true);
	ret |= bmi_acc_batch(st, NULL, 625, 0); /* 1600Hz */
	st->snsrs[i].usr_cfg = 3; /* 24g */
	ret |= bmi_acc_able(st, 1);
	mdelay(2);
	/* Enable accel self-test with positive excitation */
	ret |= bmi_i2c_wr(st, BMI_HW_ACC, BMI_REG_ACC_SELF_TEST,
			  BMI_REG_ACC_SELF_TEST_POS);
	mdelay(BMI_ACC_SELFTEST_DELAY_MS);
	ret |= bmi_i2c_rd(st, BMI_HW_ACC, BMI_REG_ACC_DATA,
			  BMI_REG_ACC_DATA_N, st->buf_acc);
	x_pos = (st->buf_acc[1] << 8) | (st->buf_acc[0]);
	y_pos = (st->buf_acc[3] << 8) | (st->buf_acc[2]);
	z_pos = (st->buf_acc[5] << 8) | (st->buf_acc[4]);
	/* Enable accel self-test with negative excitation */
	ret |= bmi_i2c_wr(st, BMI_HW_ACC, BMI_REG_ACC_SELF_TEST,
			  BMI_REG_ACC_SELF_TEST_NEG);
	mdelay(BMI_ACC_SELFTEST_DELAY_MS);
	ret |= bmi_i2c_rd(st, BMI_HW_ACC, BMI_REG_ACC_DATA,
			  BMI_REG_ACC_DATA_N, st->buf_acc);
	ret |= bmi_i2c_wr(st, BMI_HW_ACC, BMI_REG_ACC_SELF_TEST,
			  BMI_REG_ACC_SELF_TEST_OFF);
	mdelay(BMI_ACC_SELFTEST_DELAY_MS);
	st->buf_acc_i = 0; /* self test corrupts buffer - reset buffer index */
	st->snsrs[i].usr_cfg = usr_cfg; /* restore user configuration */
	bmi_acc_softreset(st, BMI_HW_ACC);
	if (!ret) {
		x_neg = (st->buf_acc[1] << 8) | (st->buf_acc[0]);
		y_neg = (st->buf_acc[3] << 8) | (st->buf_acc[2]);
		z_neg = (st->buf_acc[5] << 8) | (st->buf_acc[4]);
		if (!((abs(x_neg - x_pos) > BMI_ACC_SELF_TEST_LIMIT_X)
			&& (abs(y_neg - y_pos) > BMI_ACC_SELF_TEST_LIMIT_Y)
			&& (abs(z_neg - z_pos) > BMI_ACC_SELF_TEST_LIMIT_Z)))
			ret = 1; /* failure */
	}
	return ret;
}

static int bmi_acc_pm(struct bmi_state *st, unsigned int hw, int able)
{
	int ret;

	if (able) {
		if (st->buf_acc == NULL)
			st->buf_acc = devm_kzalloc(&st->i2c->dev,
						   (size_t)st->buf_acc_n,
						   GFP_KERNEL);
		st->buf_acc_i = 0;
		if (st->buf_acc) {
			/* when we have buffer we can power on */
			ret = bmi_i2c_wr(st, BMI_HW_ACC,
					 BMI_REG_ACC_PWR_CONF,
					 BMI_REG_ACC_PWR_CONF_ACTV);
			if (ret) {
				st->hw_en &= ~(1 << hw);
			} else {
				st->hw_en |= (1 << hw);
				mdelay(BMI_ACC_PM_DELAY_MS);
				ret = bmi_i2c_wr(st, BMI_HW_ACC,
						 BMI_REG_INT1_IO_CTRL,
						 st->ra_0x53);
				ret |= bmi_i2c_wr(st, BMI_HW_ACC,
						  BMI_REG_INT2_IO_CTRL,
						  st->ra_0x54);
				ret |= bmi_i2c_wr(st, BMI_HW_ACC,
						  BMI_REG_INT_MAP_DATA,
						  st->ra_0x58);
/*				ret |= bmi_i2c_wr(st, BMI_HW_ACC,
 *						  BMI_REG_ACC_FIFO_CFG_0,
 *						  st->ra_0x48);
 */
				ret |= bmi_i2c_wr(st, BMI_HW_ACC,
						  BMI_REG_ACC_FIFO_CFG_1,
						  st->ra_0x49);
			}
		} else {
			ret = -ENOMEM;
		}
	} else {
		ret = bmi_acc_able(st, 0);
		ret |= bmi_i2c_wr(st, BMI_HW_ACC, BMI_REG_ACC_PWR_CONF,
				  BMI_REG_ACC_PWR_CONF_SUSP);
		bmi_hw_off(st, hw);
	}
	return ret;
}

static unsigned long bmi_acc_irqflags(struct bmi_state *st)
{
	unsigned long irqflags;
	u8 int_io_conf;

	if (st->ra_0x53 & BMI_REG_INTX_IO_CTRL_OUT_EN)
		int_io_conf = st->ra_0x53;
	else
		int_io_conf = st->ra_0x54;
	if (int_io_conf & BMI_REG_INTX_IO_CTRL_ACTV_HI)
		irqflags = IRQF_TRIGGER_RISING;
	else
		irqflags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	return irqflags;
}

static struct bmi_odr bmi_odrs_gyr[] = {
	{ 10000, 0x05 },
	{ 5000,  0x04 },
	{ 2500,  0x03 },
	{ 1000,  0x02 },
	{ 500,   0x01 },
};

static int bmi_gyr_batch(struct bmi_state *st, unsigned int *odr_us,
			 unsigned int period_us, unsigned int timeout_us)
{
	u8 val;
	unsigned int i;
	unsigned int n;
	unsigned int odr_i;
	int ret = 0;

	odr_i = bmi_odr_i(bmi_odrs_gyr, ARRAY_SIZE(bmi_odrs_gyr), period_us);
	if (odr_us) {
		*odr_us = bmi_odrs_gyr[odr_i].period_us;
	} else if (st->hw_en & bmi_hw_dpnd_msks[BMI_HW_GYR]) {
		/* power is up */
		val = bmi_odrs_gyr[odr_i].hw;
		ret = bmi_i2c_wr(st, BMI_HW_GYR, BMI_REG_GYR_BW, val);
		i = st->hw2ids[BMI_HW_GYR];
		if ((i < BMI_HW_N) && !ret) {
			st->snsrs[i].timeout_en = false;
			st->snsrs[i].odr_us = bmi_odrs_gyr[odr_i].period_us;
			st->ts_odr[BMI_HW_GYR] = (s64)st->snsrs[i].odr_us *
									  1000;
			if (timeout_us) {
				/* calculate batch timeout */
				n = timeout_us;
				n /= bmi_odrs_gyr[odr_i].period_us;
				if (n > BMI_REG_GYR_FIFO_CFG_0_WM_N)
					n = BMI_REG_GYR_FIFO_CFG_0_WM_N;
				val = n;
				ret = bmi_i2c_wr(st, BMI_HW_GYR,
						 BMI_REG_GYR_FIFO_CFG_0, val);
				if (!ret)
					st->snsrs[i].timeout_en = true;
			}
		}
	}
	return ret;
}

static int bmi_gyr_able(struct bmi_state *st, int en)
{
	u8 val;
	unsigned int i;
	int ret;

	if (en) {
		i = st->hw2ids[BMI_HW_GYR]; /* st->hw2ids only valid on en */
		val = st->snsrs[i].usr_cfg;
		ret = bmi_i2c_wr(st, BMI_HW_GYR, BMI_REG_GYR_RANGE, val);
		if (st->snsrs[i].timeout_en) {
			ret = bmi_i2c_wr(st, BMI_HW_GYR,
					 BMI_REG_FIFO_WM_ENABLE,
					 BMI_REG_FIFO_WM_ENABLE_EN);
			val = BMI_REG_GYR_INT_CTRL_FIFO_EN;
		} else {
			val = BMI_REG_GYR_INT_CTRL_DATA_EN;
		}
		st->ts[BMI_HW_GYR] = nvs_timestamp();
		ret = bmi_i2c_wr(st, BMI_HW_GYR, BMI_REG_GYR_INT_CTRL, val);
	} else {
		ret = bmi_i2c_wr(st, BMI_HW_GYR, BMI_REG_FIFO_WM_ENABLE,
				 BMI_REG_FIFO_WM_ENABLE_DIS);
		ret |= bmi_i2c_wr(st, BMI_HW_GYR, BMI_REG_GYR_INT_CTRL,
				  BMI_REG_GYR_INT_CTRL_DIS);
	}
	return ret;
}

static int bmi_gyr_softreset(struct bmi_state *st, unsigned int hw)
{
	int ret;

	ret = bmi_i2c_wr(st, BMI_HW_GYR, BMI_REG_GYR_SOFTRESET,
			 BMI_REG_GYR_SOFTRESET_EXE);
	mdelay(BMI_GYR_SOFTRESET_DELAY_MS);
	bmi_hw_off(st, hw);
	return ret;
}

static int bmi_gyr_selftest(struct bmi_state *st)
{
	uint8_t val;
	unsigned int i;
	int ret;

	ret = bmi_i2c_wr(st, BMI_HW_GYR, BMI_REG_GYR_SELF_TEST,
			 BMI_REG_GYR_SELF_TEST_EXE);
	if (!ret) {
		for (i = 0; i < BMI_GYR_SELFTEST_RD_LOOP_N; i++) {
			mdelay(BMI_GYR_SELFTEST_DELAY_MS);
			ret = bmi_i2c_rd(st, BMI_HW_GYR, BMI_REG_GYR_SELF_TEST,
					 1, &val);
			if ((val & BMI_REG_GYR_SELF_TEST_RDY) && !ret)
				break;
		}
		if (!ret) {
			if (val & BMI_REG_GYR_SELF_TEST_FAIL)
				/* failure */
				ret = 1;
		}
	}
	bmi_gyr_softreset(st, BMI_HW_GYR);
	return ret;
}

static int bmi_gyr_pm(struct bmi_state *st, unsigned int hw, int able)
{
	size_t n;
	int ret;

	if (able) {
		if (st->buf_gyr == NULL) {
			n = st->buf_gyr_n * BMI_REG_GYR_DATA_N;
			st->buf_gyr = devm_kzalloc(&st->i2c->dev, n,
						   GFP_KERNEL);
		}
		if (st->buf_gyr) {
			/* when we have buffer we can power on */
			ret = bmi_i2c_wr(st, BMI_HW_GYR, BMI_REG_GYR_LPM1,
					 BMI_REG_GYR_LPM1_NORM);
			if (ret) {
				st->hw_en &= ~(1 << hw);
			} else {
				st->hw_en |= (1 << hw);
				mdelay(BMI_GYR_PM_DELAY_MS);
				ret = bmi_i2c_wr(st, BMI_HW_GYR,
						 BMI_REG_INT_3_4_IO_CONF,
						 st->rg_0x16);
				ret |= bmi_i2c_wr(st, BMI_HW_GYR,
						  BMI_REG_INT_3_4_IO_MAP,
						  st->rg_0x18);
				ret |= bmi_i2c_wr(st, BMI_HW_GYR,
						  BMI_REG_FIFO_EXT_INT_S,
						  st->rg_0x34);
				ret |= bmi_i2c_wr(st, BMI_HW_GYR,
						  BMI_REG_GYR_FIFO_CFG_1,
						BMI_REG_GYR_FIFO_CFG_1_STREAM);
			}
		} else {
			ret = -ENOMEM;
		}
	} else {
		ret = bmi_gyr_able(st, 0);
		ret |= bmi_i2c_wr(st, BMI_HW_GYR, BMI_REG_GYR_LPM1,
				  BMI_REG_GYR_LPM1_DEEP);
		bmi_hw_off(st, hw);
	}
	return ret;
}

static unsigned long bmi_gyr_irqflags(struct bmi_state *st)
{
	unsigned long irqflags;

	if (st->rg_0x18 & BMI_REG_INT_3_4_IO_MAP_INT3) {
		if (st->rg_0x16 & BMI_REG_INT_3_4_IO_CONF_3_HI)
			irqflags = IRQF_TRIGGER_RISING;
		else
			irqflags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	} else { /* BMI_REG_INT_3_4_IO_MAP_INT4 */
		if (st->rg_0x16 & BMI_REG_INT_3_4_IO_CONF_4_HI)
			irqflags = IRQF_TRIGGER_RISING;
		else
			irqflags = IRQF_TRIGGER_FALLING | IRQF_ONESHOT;
	}
	return irqflags;
}

static int bmi_tmp_able(struct bmi_state *st, int en)
{
	unsigned int i;
	int ret = 0;

	if (en) {
		nvs_on_change_enable(&st->oc);
		st->ts_hw[BMI_HW_TMP] = 0;
		i = st->hw2ids[BMI_HW_ACC];
		if (!(st->enabled & (1 << i)))
			/* TMP needs ACC to be on */
			ret = bmi_acc_able(st, 1);
	}
	return ret;
}

static int bmi_tmp_batch(struct bmi_state *st, unsigned int *odr_us,
			 unsigned int period_us, unsigned int timeout_us)
{
	unsigned int i;
	int ret = 0;

	i = st->hw2ids[BMI_HW_TMP];
	if (odr_us) {
		*odr_us = st->oc.period_us;
	} else {
		st->oc.period_us = period_us;
		st->snsrs[i].odr_us = period_us;
		st->ts_odr[BMI_HW_TMP] = (s64)st->snsrs[i].odr_us * 1000;
	}
	return ret;
}

static int bmi_read_tmp(struct bmi_state *st)
{
	u8 buf[2];
	u16 hw;
	unsigned int i;
	int ret = 0;

	i = st->hw2ids[BMI_HW_TMP];
	if ((i < BMI_HW_N) && (st->enabled & (1 << i)) &&
			       (st->ts_hw[BMI_HW_TMP] <= st->ts[BMI_HW_TMP])) {
		ret = bmi_i2c_rd(st, BMI_HW_ACC, BMI_REG_TEMP_MSB,
				 sizeof(buf), buf);
		if ((buf[0] != 0x80) && !ret) { /* 0x80 == invalid */
			hw = buf[0];
			hw *= 8; /* TEMP_MSB * 8 */
			buf[1] >>= 5; /* TEMP_LSB / 32*/
			hw += buf[1];
			if (hw > 1023)
				hw -= 2048;
			st->oc.hw = hw;
			st->oc.timestamp = st->ts[BMI_HW_TMP];
			ret = nvs_on_change_read(&st->oc);
			st->ts_hw[BMI_HW_TMP] = (st->ts[BMI_HW_TMP] +
						 st->ts_odr[BMI_HW_TMP]);
		}
	}
	return ret;
}

static void bmi_work(struct work_struct *ws)
{
	struct bmi_state *st = container_of((struct work_struct *)ws,
					    struct bmi_state, ws);
	s64 ts1;
	s64 ts2;
	s64 ts3;
	u64 ts_diff;
	unsigned long sleep_us;
	unsigned int i;
	int ret;

	while (st->enabled) {
		bmi_mutex_lock(st);
		ts1 = nvs_timestamp();
		i = st->hw2ids[BMI_HW_GYR];
		if ((i < BMI_HW_N) && (st->enabled & (1 << i))) {
			ret = bmi_i2c_rd(st, BMI_HW_GYR, BMI_REG_GYR_DATA,
					 BMI_REG_GYR_DATA_N, st->buf_gyr);
			if (!ret)
				st->nvs->handler(st->snsrs[i].nvs_st,
						 st->buf_gyr, ts1);
		}
		ts2 = nvs_timestamp();
		i = st->hw2ids[BMI_HW_ACC];
		if ((i < BMI_HW_N) && (st->enabled & (1 << i))) {
			ret = bmi_i2c_rd(st, BMI_HW_ACC, BMI_REG_ACC_DATA,
					 BMI_REG_ACC_DATA_N, st->buf_acc);
			if (!ret)
				st->nvs->handler(st->snsrs[i].nvs_st,
						 st->buf_acc, ts2);
		}
		st->ts_hw[BMI_HW_TMP] = ts2;
		bmi_read_tmp(st);
		ts3 = nvs_timestamp();
		bmi_mutex_unlock(st);
		ts_diff = (ts3 - ts1) / 1000; /* ns => us */
		if (st->period_us > (unsigned int)ts_diff)
			sleep_us = st->period_us - (unsigned int)ts_diff;
		else
			sleep_us = 0;
		if (sleep_us > 10000) /* SLEEPING FOR LARGER MSECS ( 10ms+ ) */
			msleep(sleep_us / 1000);
		else if (sleep_us)
			/* SLEEPING FOR ~USECS OR SMALL MSECS ( 10us - 20ms) */
			usleep_range(sleep_us, st->period_us);
	}
}

static int bmi_push_acc(struct bmi_state *st, s64 ts)
{
	unsigned int i;
	unsigned int n;

	i = st->hw2ids[BMI_HW_ACC];
	st->buf_acc_i++;
	if (i < BMI_HW_N)
		st->nvs->handler(st->snsrs[i].nvs_st,
				 &st->buf_acc[st->buf_acc_i], ts);
	if ((i < BMI_HW_N) && (st->sts & BMI_STS_SPEW_FIFO_ACC)) {
		for (n = 0; n < BMI_REG_ACC_DATA_N; n++) {
			dev_info(&st->i2c->dev,
				 "%s: buf[%u]=0x%02X\n",
				 st->snsrs[i].cfg.name,
				 st->buf_acc_i, st->buf_acc[st->buf_acc_i]);
			st->buf_acc_i++;
		}
	} else {
		n = BMI_REG_ACC_DATA_N;
		st->buf_acc_i += n;
	}
	return n;
}

static int bmi_ts_thr_dbg(struct bmi_state *st, s64 ts_irq)
{
	if (ts_irq > 1) {
		dev_info(&st->i2c->dev,
			 "%s CALC ts=%lld irq=%lld->%lld (%lld) odr=%lld\n",
			 __func__, st->ts[BMI_HW_ACC], st->ts_st_irq, ts_irq,
			 ts_irq - st->ts_st_irq, st->ts_odr[BMI_HW_ACC]);
	} else if (!ts_irq) {
		dev_info(&st->i2c->dev,
			 "%s IRQ SYNC ts=%lld=>%lld (%lld)\n",
			 __func__, st->ts[BMI_HW_ACC], st->ts_st_irq,
			 st->ts[BMI_HW_ACC] - st->ts_st_irq);
	} else {
		dev_info(&st->i2c->dev,
			 "%s START ts=%lld lo=%lld hi=%lld odr=%lld\n",
			 __func__, st->ts[BMI_HW_ACC], st->ts_lo, st->ts_hi,
			 st->ts_odr[BMI_HW_ACC]);
	}
	return 0;
}

static int bmi_ts_thr(struct bmi_state *st)
{
	s64 ns;
	s64 ts_irq;

	if (st->ts_hi) {
		if (st->ts[BMI_HW_ACC] > st->ts_hi) {
			/* missed this TS sync - calculate new thresholds */
			ts_irq = atomic64_read(&st->ts_irq[BMI_HW_ACC]);
			ns = st->ts_odr[BMI_HW_ACC];
			ns >>= 1;
			st->ts_lo = ts_irq - ns;
			st->ts_hi = ts_irq + ns;
			if (st->sts & BMI_STS_SPEW_TS_ACC)
				bmi_ts_thr_dbg(st, ts_irq);
			st->ts_st_irq = ts_irq;
		} else if (st->ts[BMI_HW_ACC] >= st->ts_lo) {
			/* IRQ TS within range */
			if (st->sts & BMI_STS_SPEW_TS_ACC)
				bmi_ts_thr_dbg(st, 0);
			st->ts[BMI_HW_ACC] = st->ts_st_irq;
		}
	} else {
		/* first timestamp and event */
		st->ts_st_irq = st->ts[BMI_HW_ACC] + st->ts_odr[BMI_HW_ACC];
		st->ts_lo = st->ts_odr[BMI_HW_ACC];
		st->ts_lo >>= 1;
		st->ts_lo += st->ts[BMI_HW_ACC];
		st->ts_hi = st->ts_lo + st->ts_odr[BMI_HW_ACC];
		if (st->sts & BMI_STS_SPEW_TS_ACC)
			bmi_ts_thr_dbg(st, -1);
	}
	if (st->ts[BMI_HW_ACC] > st->ts_hw[BMI_HW_ACC])
		/* st->ts_hw can be used as ts_now since it's the timestamp of
		 * the last time the HW was accessed. It's used here to confirm
		 * that we never go forward in time.
		 */
		st->ts[BMI_HW_ACC] = st->ts_hw[BMI_HW_ACC];
	return 0;
}

static int bmi_frame_drop(struct bmi_state *st)
{
	s64 ns;
	unsigned int i;
	unsigned int n;

	st->buf_acc_i++;
	n = st->buf_acc[st->buf_acc_i]; /* drop frame count */
	st->frame_n += n; /* for ODR calculation */
	i = st->lost_frame_n; /* save old */
	st->lost_frame_n += n; /* debug FYI */
	if (st->lost_frame_n < i)
		/* rollover */
		st->lost_frame_n = -1;
	/* update timestamp to include lost frames */
	ns = st->ts_odr[BMI_HW_ACC];
	ns *= n;
	ns += st->ts[BMI_HW_ACC];
	if (st->sts & BMI_STS_SPEW_FIFO_ACC)
		dev_info(&st->i2c->dev,
			 "SKIP FRAME: buf[%u]=0x%02X\n",
			 st->buf_acc_i, n);
	if (st->sts & BMI_STS_SPEW_TS_ACC)
		dev_info(&st->i2c->dev,
			 "SKIP FRAME: n=%u odr=%lld TS: %lld->%lld\n",
			 n, st->ts_odr[BMI_HW_ACC], st->ts[BMI_HW_ACC], ns);
	st->ts[BMI_HW_ACC] = ns;
	st->buf_acc_i++;
	return 0;
}

static int bmi_frame_time(struct bmi_state *st)
{
	s64 ns;
	unsigned int snsr_t;
	unsigned int i;
	unsigned int n = 0;

	st->buf_acc_i++;
	snsr_t = bmi_buf2sensortime(&st->buf_acc[st->buf_acc_i]);
	if (st->frame_n) {
		if (snsr_t > st->snsr_t) {
			n = snsr_t - st->snsr_t;
		} else {
			/* counter rolled over */
			n = ~0xFFFFFFU; /* 24->32 */
			n |= st->snsr_t;
			n = ~n;
			n++;
			n += snsr_t;
			if (st->sts & BMI_STS_SPEW_TS_ACC)
				dev_info(&st->i2c->dev,
					"%s st: %u->%u n=%u\n",
					 __func__, st->snsr_t,
					 snsr_t, n);
		}
		/* n is the number of sensortime ticks since
		 * last time.  Each tick is 39062.5ns.
		 */
		ns = n;
		ns *= 390625;
		n = st->frame_n;
		n *= 10;
		st->ts_odr[BMI_HW_ACC] = ns / n;
	}
	if (st->sts & BMI_STS_SPEW_ST_ACC) {
		for (i = 0; i < 3; i++) {
			dev_info(&st->i2c->dev,
				 "SENSORTIME: buf[%u]=0x%02X\n",
				 st->buf_acc_i, st->buf_acc[st->buf_acc_i]);
			st->buf_acc_i++;
		}

		dev_info(&st->i2c->dev,
			 "snsr_t: %u->%u ticks=%u frame_n=%u odr=%lld\n",
			 st->snsr_t, snsr_t, n, st->frame_n,
			 st->ts_odr[BMI_HW_ACC]);
	}
	st->snsr_t = snsr_t;
	st->frame_n = 0;
	st->buf_acc_i = 0; /* break out of outer loop */
	return 0;
}

static int bmi_read_acc(struct bmi_state *st)
{
	u8 hdr;
	u16 buf_n;
	u16 fifo_n;
	unsigned int n;
	int ret;

	ret = bmi_i2c_rd(st, BMI_HW_ACC, BMI_REG_FIFO_LENGTH_0,
			 sizeof(fifo_n), (u8 *)&fifo_n);
	if (ret)
		return ret;

	if (st->sts & BMI_STS_SPEW_FIFO_ACC)
		dev_info(&st->i2c->dev, "%s fifo_n=%u\n", __func__, fifo_n);
	fifo_n &= 0x03FF;
	/* to get the sensor time apparently we have to +25 */
	fifo_n += 25;
	while (fifo_n) {
		if (fifo_n > st->buf_acc_n)
			buf_n = st->buf_acc_n;
		else
			buf_n = fifo_n;
		ret = bmi_i2c_rd(st, BMI_HW_ACC, BMI_REG_ACC_FIFO_DATA,
				 buf_n, st->buf_acc);
		if (ret)
			return ret;

		st->buf_acc_i = 0;
		while (st->buf_acc_i < buf_n) {
			hdr = st->buf_acc[st->buf_acc_i];
			if (st->sts & BMI_STS_SPEW_FIFO_ACC)
				dev_info(&st->i2c->dev,
					 "HDR: buf_acc[%u]=0x%02X fifo_n=%u\n",
					 st->buf_acc_i, hdr, fifo_n);

			hdr &= 0xFC;
			n = buf_n - st->buf_acc_i;
			if (hdr == 0x84) {
				/* regular frame */
				if (n < (BMI_REG_ACC_DATA_N + 1))
					/* not enough data to process */
					break; /* exit inner loop */

				st->frame_n++;
				bmi_ts_thr(st);
				bmi_push_acc(st, st->ts[BMI_HW_ACC]);
				st->ts[BMI_HW_TMP] = st->ts[BMI_HW_ACC];
				st->ts[BMI_HW_ACC] += st->ts_odr[BMI_HW_ACC];
			} else if (hdr == 0x40) {
				/* drop frame */
				if (n > 1)
					bmi_frame_drop(st);
			} else if (hdr == 0x44) {
				/* sensortime */
				if (n > 3)
					bmi_frame_time(st);
				break; /* exit inner loop */
			} else if (hdr == 0x48) {
				/* config change */
				st->buf_acc_i++;
				st->buf_acc_i++;
			} else if (hdr == 0x50) {
				/* config change drop */
				st->buf_acc_i++;
				st->buf_acc_i++;
			} else {
				/* error - but possible when disabling */
				st->buf_acc_i = 0; /* exit fifo_n loop */
				break; /* exit (st->buf_acc_i < buf_n) loop */
			}
		}

		if (!st->buf_acc_i)
			/* not enough data to process - exit fifo_n loop */
			break;

		fifo_n -= st->buf_acc_i;
	}

	ret = bmi_read_tmp(st);
	return ret;
}

static int bmi_read_gyr(struct bmi_state *st)
{
	s64 ts1;
	s64 ts2;
	s64 ts3;
	u8 fifo_n;
	unsigned int buf_i;
	unsigned int buf_n;
	unsigned int i;
	unsigned int n;
	unsigned int snsr_id;
	int ret;

	ts1 = atomic64_read(&st->ts_irq[BMI_HW_GYR]);
	ret = bmi_i2c_rd(st, BMI_HW_GYR, BMI_REG_FIFO_STATUS,
			 sizeof(fifo_n), (u8 *)&fifo_n);
	ts2 = atomic64_read(&st->ts_irq[BMI_HW_GYR]);
	n = fifo_n & BMI_REG_FIFO_STATUS_N_MSK;
	if ((!ret) && n) {
		if (ts1 == ts2) {
			/* nth read == ts_irq */
			ts3 = n;
			ts3 *= st->ts_odr[BMI_HW_GYR];
			ts3 = ts1 - ts3;
			ts2 = st->ts_odr[BMI_HW_GYR];
			if ((ts3 + ts2) <= st->ts[BMI_HW_GYR]) {
				ts2 = ts1 - st->ts[BMI_HW_GYR];
				ts2 /= n;
			} else {
				st->ts[BMI_HW_GYR] = ts3;
			}
			if (st->sts & BMI_STS_SPEW_TS_GYR)
				dev_info(&st->i2c->dev,
					 "%s S irq=%lld 1st=%lld n=%u @%lld\n",
					 __func__, ts1, ts3 + ts2, n, ts2);
		} else if (ts2 > ts1) {
			ts3 = n;
			ts3 *= st->ts_odr[BMI_HW_GYR];
			ts3 += st->ts[BMI_HW_GYR];
			if (ts3 > ts2) {
				/* out of range - need to adjust max limit */
				ts2 = ts2 - st->ts[BMI_HW_GYR];
				ts2 /= n;
			} else if (ts3 < ts1) {
				/* out of range - need to adjust min limit */
				ts2 = st->ts_odr[BMI_HW_GYR];
				ts3 = n * ts2;
				st->ts[BMI_HW_GYR] = ts1 - ts3;
			} else { /* ts3 >= ts1 && ts3 <= ts2 */
				ts2 = st->ts_odr[BMI_HW_GYR];
			}
			if (st->sts & BMI_STS_SPEW_TS_GYR)
				dev_info(&st->i2c->dev,
					 "%s R irq=%lld 1st=%lld n=%u @%lld\n",
					 __func__, ts1,
					 st->ts[BMI_HW_GYR] + ts2, n, ts2);
		} else { /* ts2 < ts1 */
			/* internal error */
			ts2 = st->ts_odr[BMI_HW_GYR];
			if (st->sts & BMI_STS_SPEW_TS_GYR)
				dev_info(&st->i2c->dev,
					 "%s E irq=%lld 1st=%lld n=%u @%lld\n",
					 __func__, ts1,
					 st->ts[BMI_HW_GYR] + ts2, n, ts2);
			bmi_err(st);
		}
		if (st->sts & BMI_STS_SPEW_FIFO_GYR)
			dev_info(&st->i2c->dev, "%s fifo_n=%u\n", __func__, n);
		snsr_id = st->hw2ids[BMI_HW_GYR];
		do {
			if (n > st->buf_gyr_n)
				buf_n = st->buf_gyr_n;
			else
				buf_n = n;
			ret = bmi_i2c_rd(st, BMI_HW_GYR, BMI_REG_GYR_FIFO_DATA,
					 buf_n * BMI_REG_GYR_DATA_N,
					 st->buf_gyr);
			if (ret)
				break; /* do while (n) */

			for (i = 0, buf_i = 0; i < buf_n; i++) {
				st->ts[BMI_HW_GYR] += ts2;
				st->nvs->handler(st->snsrs[snsr_id].nvs_st,
						 &st->buf_gyr[buf_i],
						 st->ts[BMI_HW_GYR]);
				buf_i += BMI_REG_GYR_DATA_N;
			}

			n -= buf_n;
		} while (n);
	}

	return ret;
}

static irqreturn_t bmi_irq_thread(int irq, void *dev_id)
{
	struct bmi_state *st = (struct bmi_state *)dev_id;

	bmi_mutex_lock(st);
	if (irq == st->gis[BMI_HW_ACC].irq)
		bmi_read_acc(st);
	if (irq == st->gis[BMI_HW_GYR].irq)
		bmi_read_gyr(st);
	bmi_mutex_unlock(st);
	return IRQ_HANDLED;
}

static irqreturn_t bmi_irq_handler(int irq, void *dev_id)
{
	struct bmi_state *st = (struct bmi_state *)dev_id;
	s64 ts;
	s64 ts_old;
	s64 ts_diff;
	unsigned int hw;
	int ret;

	if (irq == st->gis[BMI_HW_GYR].irq)
		hw = BMI_HW_GYR;
	else
		hw = BMI_HW_ACC;
	ret = nvs_gte_ts(&st->gis[hw]);
	if (ret)
		st->gis[hw].irq_ts = nvs_timestamp();
	if (irq == st->gis[BMI_HW_ACC].irq) {
		ts = st->gis[hw].irq_ts;
		ts_old = atomic64_xchg(&st->ts_irq[hw], ts);
		if (st->sts & BMI_STS_SPEW_TS_ACC) {
			ts_diff = ts - ts_old;
			dev_info(&st->i2c->dev,
				 "%s  %s ts=%lld  ts_diff=%lld\n", __func__,
				 bmi_snsr_cfgs[hw].name, ts, ts_diff);
		}
	}
	if (irq == st->gis[BMI_HW_GYR].irq) {
		ts = st->gis[hw].irq_ts;
		ts_old = atomic64_xchg(&st->ts_irq[hw], ts);
		if (st->sts & BMI_STS_SPEW_TS_GYR) {
			ts_diff = ts - ts_old;
			dev_info(&st->i2c->dev,
				 "%s  %s ts=%lld  ts_diff=%lld\n", __func__,
				 bmi_snsr_cfgs[hw].name, ts, ts_diff);
		}
	}
	return IRQ_WAKE_THREAD;
}

static int bmi_read(struct bmi_state *st, int snsr_id)
{
	int ret;

	if (st->snsrs[snsr_id].hw == BMI_HW_ACC)
		ret = bmi_read_acc(st);
	else if (st->snsrs[snsr_id].hw == BMI_HW_GYR)
		ret = bmi_read_gyr(st);
	else
		ret = -ENODEV;
	return ret;
}

static int bmi_period(struct bmi_state *st, unsigned int msk_en, int snsr_id)
{
	unsigned int hw;
	unsigned int i;
	unsigned int to;
	unsigned int us;
	int ret = 0;

	if (st->op_mode == BMI_OP_MODE_IRQ) {
		hw = st->snsrs[snsr_id].hw;
		us = st->snsrs[snsr_id].period_us;
		to = st->snsrs[snsr_id].timeout_us;
		if (st->enabled & (1 << snsr_id)) {
			/* already enabled */
			ret = bmi_hws[hw].fn_able(st, 0);
			bmi_read(st, snsr_id); /* empty FIFO */
			ret |= bmi_hws[hw].fn_batch(st, NULL, us, to);
			ret |= bmi_hws[hw].fn_able(st, 1); /* reenable */
		} else {
			ret = bmi_hws[hw].fn_batch(st, NULL, us, to);
		}
	} else if (msk_en) {
		/* find the fastest poll time of enabled devices */
		us = st->period_us_max;
		for (i = 0; i < st->hw_n; i++) {
			if (msk_en & (1 << i)) {
				if (st->snsrs[i].period_us) {
					if (st->snsrs[i].period_us < us)
						us = st->snsrs[i].period_us;
				}
			}
		}

		st->period_us = us;
		ret = 0;
		if (st->op_mode == BMI_OP_MODE_POLL) {
			/* put devices in fastest speed */
			for (i = 0; i < st->hw_n; i++) {
				hw = st->snsrs[i].hw;
				ret |= bmi_hws[hw].fn_batch(st, NULL, 0, 0);
			}
		} else { /* st->op_mode == BMI_OP_MODE_IRQ_SHARED */
			/* set the same IRQ time of all enabled devices */
			ret = 0;
			/* disable enabled sensors */
			for (i = 0; i < st->hw_n; i++) {
				if (st->enabled & (1 << i)) {
					hw = st->snsrs[i].hw;
					ret = bmi_hws[hw].fn_able(st, 0);
				}
			}

			/* set common period */
			for (i = 0; i < st->hw_n; i++) {
				if (msk_en & (1 << i)) {
					hw = st->snsrs[i].hw;
					ret |= bmi_hws[hw].fn_batch(st, NULL,
								    us, 0);
				}
			}

			/* reenable */
			for (i = 0; i < st->hw_n; i++) {
				if (msk_en & (1 << i)) {
					hw = st->snsrs[i].hw;
					ret |= bmi_hws[hw].fn_able(st, 1);
				}
			}
		}
	}
	return ret;
}

static int bmi_disable(struct bmi_state *st, int snsr_id)
{
	int ret;

	ret = bmi_pm(st, snsr_id, false);
	if (st->op_mode != BMI_OP_MODE_IRQ)
		bmi_period(st, st->enabled, snsr_id);
	return ret;
}

static int bmi_enable(void *client, int snsr_id, int enable)
{
	struct bmi_state *st = (struct bmi_state *)client;
	unsigned int hw;
	int ret;

	if (enable < 0)
		return (st->enabled & (1 << snsr_id));

	if (enable) {
		enable = st->enabled | (1 << snsr_id);
		ret = bmi_pm(st, snsr_id, true);
		if (ret < 0)
			return ret;

		ret = bmi_period(st, enable, snsr_id);
		if (st->op_mode != BMI_OP_MODE_IRQ_SHARED) {
			hw = st->snsrs[snsr_id].hw;
			ret |= bmi_hws[hw].fn_able(st, 1);
		}
		if (st->op_mode == BMI_OP_MODE_POLL) {
			if (!ret) {
				if (st->enabled) {
					/* already enabled */
					st->enabled = enable;
				} else {
					st->enabled = enable;
					cancel_work_sync(&st->ws);
					queue_work(st->wq, &st->ws);
				}
			}
		} else {
			if (!ret)
				st->enabled = enable;
		}
		if (ret)
			bmi_disable(st, snsr_id);
	} else {
		ret = bmi_disable(st, snsr_id);
	}

	return ret;
}

static int bmi_batch(void *client, int snsr_id, int flags,
		     unsigned int period_us, unsigned int timeout_us)
{
	struct bmi_state *st = (struct bmi_state *)client;
	int ret = 0;

	if (timeout_us && !st->snsrs[snsr_id].cfg.fifo_max_evnt_cnt)
		/* timeout not supported */
		return -EINVAL;

	st->snsrs[snsr_id].period_us = period_us;
	st->snsrs[snsr_id].timeout_us = timeout_us;
	if (st->enabled & (1 << snsr_id))
		ret = bmi_period(st, st->enabled, snsr_id);
	return ret;
}

static int bmi_batch_read(void *client, int snsr_id,
			  unsigned int *period_us, unsigned int *timeout_us)
{
	struct bmi_state *st = (struct bmi_state *)client;
	unsigned int hw;
	int ret = 0;

	if (period_us) {
		if (st->enabled & (1 << snsr_id)) {
			*period_us = st->snsrs[snsr_id].odr_us;
		} else {
			hw = st->snsrs[snsr_id].hw;
			ret = bmi_hws[hw].fn_batch(st, period_us,
						  st->snsrs[snsr_id].period_us,
						st->snsrs[snsr_id].timeout_us);
		}
	}
	if (timeout_us)
		*timeout_us = st->snsrs[snsr_id].timeout_us;
	return ret;
}

static int bmi_flush(void *client, int snsr_id)
{
	struct bmi_state *st = (struct bmi_state *)client;
	int ret = -EINVAL;

	if (st->enabled & (1 << snsr_id)) {
		bmi_read(st, snsr_id);
		ret = st->nvs->handler(st->snsrs[snsr_id].nvs_st, NULL, 0LL);
	}
	return ret;
}

static int bmi_max_range(void *client, int snsr_id, int max_range)
{
	struct bmi_state *st = (struct bmi_state *)client;
	unsigned int i = max_range;

	if (st->enabled & (1 << snsr_id))
		/* can't change settings on the fly (disable device first) */
		return -EBUSY;

	if (st->snsrs[snsr_id].rrs) {
		if (i > st->snsrs[snsr_id].rrs->rr_0n)
			/* clamp to highest setting */
			i = st->snsrs[snsr_id].rrs->rr_0n;
		st->snsrs[snsr_id].usr_cfg = i;
		st->snsrs[snsr_id].cfg.max_range.ival =
				  st->snsrs[snsr_id].rrs->rr[i].max_range.ival;
		st->snsrs[snsr_id].cfg.max_range.fval =
				  st->snsrs[snsr_id].rrs->rr[i].max_range.fval;
		st->snsrs[snsr_id].cfg.resolution.ival =
				 st->snsrs[snsr_id].rrs->rr[i].resolution.ival;
		st->snsrs[snsr_id].cfg.resolution.fval =
				 st->snsrs[snsr_id].rrs->rr[i].resolution.fval;
		st->snsrs[snsr_id].cfg.scale.ival =
				 st->snsrs[snsr_id].rrs->rr[i].resolution.ival;
		st->snsrs[snsr_id].cfg.scale.fval =
				 st->snsrs[snsr_id].rrs->rr[i].resolution.fval;
	}
	return 0;
}

static int bmi_thresh_lo(void *client, int snsr_id, int thresh_lo)
{
	struct bmi_state *st = (struct bmi_state *)client;

	return nvs_on_change_threshold_lo(&st->oc, thresh_lo);
}

static int bmi_thresh_hi(void *client, int snsr_id, int thresh_hi)
{
	struct bmi_state *st = (struct bmi_state *)client;

	return nvs_on_change_threshold_hi(&st->oc, thresh_hi);
}

static int bmi_reset(void *client, int snsr_id)
{
	struct bmi_state *st = (struct bmi_state *)client;
	unsigned int hw = st->snsrs[snsr_id].hw;
	int ret = -EPERM;

	if (st->hw_en & (1 << hw))
		/* device is powered on */
		ret = bmi_hws[hw].fn_softreset(st, hw);
		/* leave the device in the softreset state */
	return ret;
}

static int bmi_selftest(void *client, int snsr_id, char *buf)
{
	struct bmi_state *st = (struct bmi_state *)client;
	unsigned int hw;
	ssize_t t;
	int ret = 0;

	hw = st->snsrs[snsr_id].hw;
	if (bmi_hws[hw].fn_selftest)
		ret = bmi_hws[hw].fn_selftest(st);
	if (buf) {
		if (ret < 0) {
			t = snprintf(buf, PAGE_SIZE,
				     "%s=ERR: %d\n", __func__, ret);
		} else {
			if (ret > 0)
				t = snprintf(buf, PAGE_SIZE,
					     "%s=FAIL\n", __func__);
			else
				t = snprintf(buf, PAGE_SIZE,
					     "%s=PASS\n", __func__);
		}
		ret = t;
	}

	return ret;
}

static int bmi_regs(void *client, int snsr_id, char *buf)
{
	struct bmi_state *st = (struct bmi_state *)client;
	struct bmi_reg_rd *reg_rd;
	ssize_t t;
	u8 reg;
	u8 val;
	unsigned int hw;
	unsigned int i;
	int ret;

	t = snprintf(buf, PAGE_SIZE, "registers:\n");
	hw = st->snsrs[snsr_id].hw;
	reg_rd = bmi_hws[hw].reg_rds;
	for (i = 0; i < bmi_hws[hw].reg_rds_n; i++) {
		for (reg = reg_rd[i].reg_lo; reg <= reg_rd[i].reg_hi; reg++) {
			ret = bmi_i2c_rd(st, hw, reg, 1, &val);
			if (ret)
				t += snprintf(buf + t, PAGE_SIZE - t,
					      "0x%02X=ERR\n", i);
			else
				t += snprintf(buf + t, PAGE_SIZE - t,
					      "0x%02X=0x%02X\n", reg, val);
		}
	}

	return t;
}

static int bmi_nvs_write(void *client, int snsr_id, unsigned int nvs)
{
	struct bmi_state *st = (struct bmi_state *)client;
	s64 ts_irq;
	unsigned int hw;
	int ret = 0;

	hw = st->snsrs[snsr_id].hw;
	switch (nvs & 0xFF) {
	case BMI_INF_VER:
	case BMI_INF_DBG:
	case BMI_INF_REG_WR:
	case BMI_INF_REG_RD:
		break;

	case BMI_INF_SPEW_FIFO:
		st->sts ^= (1 << (BMI_STS_SPEW_FIFO + hw));
		break;

	case BMI_INF_SPEW_TS:
		ts_irq = atomic64_read(&st->ts_irq[hw]);
		dev_info(&st->i2c->dev,
			"ts=%lld irq=%lld sti=%lld odr=%lld\n",
			 st->ts[hw], ts_irq, st->ts_st_irq, st->ts_odr[hw]);
		st->sts ^= (1 << (BMI_STS_SPEW_TS + hw));
		break;

	case BMI_INF_SPEW_ST:
		st->sts ^= (1 << (BMI_STS_SPEW_TS + hw));
		break;

	default:
		return -EINVAL;
	}

	st->inf = nvs;
	return ret;
}

static int bmi_nvs_read(void *client, int snsr_id, char *buf)
{
	struct bmi_state *st = (struct bmi_state *)client;
	struct nvs_gte_sts ngs;
	unsigned int hw;
	unsigned int i;
	unsigned int inf;
	int ret;
	ssize_t t;
	u8 val;

	hw = st->snsrs[snsr_id].hw;
	inf = st->inf;
	switch (inf & 0xFF) {
	case BMI_INF_VER:
		t = snprintf(buf, PAGE_SIZE, "driver v.%u\n",
			     BMI_DRIVER_VERSION);
		ret = nvs_gte_sts(&st->gis[hw], &ngs);
		if (ret) {
			t += snprintf(buf + t, PAGE_SIZE - t,
				      "GTE status ERR %d\n", ret);
		} else {
			t += snprintf(buf + t, PAGE_SIZE - t,
				      "GTE v.%u\n", ngs.ver);
			if (ngs.gte)
				t += snprintf(buf + t, PAGE_SIZE - t,
					      "GTE: %s\n", ngs.gte);
			if (ngs.ts_raw || ngs.ts_ns) {
				t += snprintf(buf + t, PAGE_SIZE - t,
					      "  ts=%llu\n", ngs.ts_ns);
				t += snprintf(buf + t, PAGE_SIZE - t,
					      "  raw=%llu\n", ngs.ts_raw);
			}
			if (st->gis[hw].err_n) {
				t += snprintf(buf + t, PAGE_SIZE - t,
					      "GTE error count=%llu\n",
					      st->gis[hw].err_n);
				st->gis[hw].err_n = 0;
			}
			if (ngs.err)
				t += snprintf(buf + t, PAGE_SIZE - t,
					      "GTE ERR: %s\n", ngs.err);
		}
		t += snprintf(buf + t, PAGE_SIZE - t, "DEVICE TREE:\n");
		t += snprintf(buf + t, PAGE_SIZE - t,
			      "device IRQ=%d (should be <= 0)\n",
			      st->i2c->irq);
		for (i = 0; i < BMI_HW_N; i++)
			t += snprintf(buf + t, PAGE_SIZE - t,
				      "%s_irq_gpio=%d  (IRQ=%d)\n",
				      bmi_snsr_cfgs[i].name,
				      st->gis[i].gpio, st->gis[i].irq);
		t += snprintf(buf + t, PAGE_SIZE - t,
			      "%s_buffer_size=%u\n",
			      bmi_snsr_cfgs[BMI_HW_ACC].name,
			      st->buf_acc_n);
		t += snprintf(buf + t, PAGE_SIZE - t,
			      "%s_register_0x40=0x%02X\n",
			      bmi_snsr_cfgs[BMI_HW_ACC].name,
			      st->ra_0x40);
/*		t += snprintf(buf + t, PAGE_SIZE - t,
 *			      "%s_register_0x48=0x%02X\n",
 *			      bmi_snsr_cfgs[BMI_HW_ACC].name,
 *			      st->ra_0x48);
 */
		t += snprintf(buf + t, PAGE_SIZE - t,
			      "%s_register_0x49=0x%02X\n",
			      bmi_snsr_cfgs[BMI_HW_ACC].name,
			      st->ra_0x49);
		t += snprintf(buf + t, PAGE_SIZE - t,
			      "%s_register_0x53=0x%02X\n",
			      bmi_snsr_cfgs[BMI_HW_ACC].name,
			      st->ra_0x53);
		t += snprintf(buf + t, PAGE_SIZE - t,
			      "%s_register_0x54=0x%02X\n",
			      bmi_snsr_cfgs[BMI_HW_ACC].name,
			      st->ra_0x54);
		t += snprintf(buf + t, PAGE_SIZE - t,
			      "%s_register_0x58=0x%02X\n",
			      bmi_snsr_cfgs[BMI_HW_ACC].name,
			      st->ra_0x58);
		t += snprintf(buf + t, PAGE_SIZE - t,
			      "%s_buffer_size=%u\n",
			      bmi_snsr_cfgs[BMI_HW_GYR].name,
			      st->buf_gyr_n * BMI_REG_GYR_DATA_N);
		t += snprintf(buf + t, PAGE_SIZE - t,
			      "%s_register_0x16=0x%02X\n",
			      bmi_snsr_cfgs[BMI_HW_GYR].name,
			      st->rg_0x16);
		t += snprintf(buf + t, PAGE_SIZE - t,
			      "%s_register_0x18=0x%02X\n",
			      bmi_snsr_cfgs[BMI_HW_GYR].name,
			      st->rg_0x18);
		t += snprintf(buf + t, PAGE_SIZE - t,
			      "%s_register_0x34=0x%02X\n",
			      bmi_snsr_cfgs[BMI_HW_GYR].name,
			      st->rg_0x34);
		t += nvs_on_change_dbg_dt(&st->oc, buf + t, PAGE_SIZE - t,
					  NULL);
		break;

	case BMI_INF_DBG:
		st->inf = BMI_INF_VER;
		t = snprintf(buf, PAGE_SIZE, "msk_en=%X\n", st->enabled);
		t += snprintf(buf + t, PAGE_SIZE - t, "op_mode=%u\n",
			      st->op_mode);
		if (hw == BMI_HW_ACC) {
			t += snprintf(buf + t, PAGE_SIZE - t,
				      "lost_frame_n=%u\n", st->lost_frame_n);
			st->lost_frame_n = 0;
		}
		if (st->op_mode != BMI_OP_MODE_IRQ) {
			t += snprintf(buf + t, PAGE_SIZE - t,
				      "period_us_max=%u\n", st->period_us_max);
			t += snprintf(buf + t, PAGE_SIZE - t, "period_us=%u\n",
				      st->period_us);
		}
		if (snsr_id == BMI_HW_TMP)
			t += nvs_on_change_dbg(&st->oc, buf + t,
					       PAGE_SIZE - t);
		i = snsr_id;
		t += snprintf(buf + t, PAGE_SIZE - t, "i2c_addr=0x%X\n",
			      st->i2c_addrs[hw]);
		t += snprintf(buf + t, PAGE_SIZE - t, "usr_cfg=%u\n",
			      st->snsrs[i].usr_cfg);
		t += snprintf(buf + t, PAGE_SIZE - t, "period_us_odr=%u\n",
			      st->snsrs[i].odr_us);
		t += snprintf(buf + t, PAGE_SIZE - t, "period_us_req=%u\n",
			      st->snsrs[i].period_us);
		t += snprintf(buf + t, PAGE_SIZE - t, "timeout_us=%u\n",
			      st->snsrs[i].timeout_us);
		t += snprintf(buf + t, PAGE_SIZE - t, "timeout_en=%X\n",
			      st->snsrs[i].timeout_en);
		break;

	case BMI_INF_SPEW_FIFO:
		i = (1 << (BMI_STS_SPEW_FIFO + hw));
		t = snprintf(buf, PAGE_SIZE, "%s FIFO spew=%x\n",
			     st->snsrs[snsr_id].cfg.name, !!(st->sts & i));
		break;

	case BMI_INF_SPEW_TS:
		i = (1 << (BMI_STS_SPEW_TS + hw));
		t = snprintf(buf, PAGE_SIZE, "%s TS spew=%x\n",
			     st->snsrs[snsr_id].cfg.name, !!(st->sts & i));
		break;

	case BMI_INF_SPEW_ST:
		i = (1 << (BMI_STS_SPEW_ST + hw));
		t = snprintf(buf, PAGE_SIZE, "%s sensortime spew=%x\n",
			     st->snsrs[snsr_id].cfg.name, !!(st->sts & i));
		break;

	case BMI_INF_REG_WR:
		ret = bmi_i2c_wr(st, hw, (u8)((inf >> 8) & 0xFF),
				 (u8)((inf >> 16) & 0xFF));
		if (ret) {
			t = snprintf(buf, PAGE_SIZE,
				     "REG WR (v=>r): ERR=%d\n", ret);
			break;
		}

		t = snprintf(buf, PAGE_SIZE, "REG WR (v=>r): 0x%02X=>0x%02X\n",
			     (inf >> 16) & 0xFF, (inf >> 8) & 0xFF);
		break;

	case BMI_INF_REG_RD:
		ret = bmi_i2c_rd(st, hw, (u8)((inf >> 8) & 0xFF), 1, &val);
		if (ret) {
			t = snprintf(buf, PAGE_SIZE, "REG RD: ERR=%d\n", ret);
			break;
		}

		t = snprintf(buf, PAGE_SIZE, "REG RD: 0x%02X=0x%02X\n",
			     (inf >> 8) & 0xFF, val);
		break;

	default:
		t = -EINVAL;
		break;
	}

	return t;
}

static struct nvs_fn_dev bmi_fn_dev = {
	.enable				= bmi_enable,
	.batch				= bmi_batch,
	.batch_read			= bmi_batch_read,
	.flush				= bmi_flush,
	.max_range			= bmi_max_range,
	.thresh_lo			= bmi_thresh_lo,
	.thresh_hi			= bmi_thresh_hi,
	.reset				= bmi_reset,
	.self_test			= bmi_selftest,
	.regs				= bmi_regs,
	.nvs_write			= bmi_nvs_write,
	.nvs_read			= bmi_nvs_read,
};

static int bmi_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bmi_state *st = i2c_get_clientdata(client);
	unsigned int i;
	int ret = 0;

	st->sts |= NVS_STS_SUSPEND;
	if (st->nvs) {
		for (i = 0; i < st->hw_n; i++)
			ret |= st->nvs->suspend(st->snsrs[i].nvs_st);
	}

	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
	return ret;
}

static int bmi_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bmi_state *st = i2c_get_clientdata(client);
	unsigned int i;
	int ret = 0;

	if (st->nvs) {
		for (i = 0; i < st->hw_n; i++)
			ret |= st->nvs->resume(st->snsrs[i].nvs_st);
	}

	st->sts &= ~NVS_STS_SUSPEND;
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
	return ret;
}

static SIMPLE_DEV_PM_OPS(bmi_pm_ops, bmi_suspend, bmi_resume);

static void bmi_shutdown(struct i2c_client *client)
{
	struct bmi_state *st = i2c_get_clientdata(client);
	unsigned int i;

	st->sts |= NVS_STS_SHUTDOWN;
	if (st->nvs) {
		for (i = 0; i < st->hw_n; i++)
			st->nvs->shutdown(st->snsrs[i].nvs_st);
	}
	if (st->sts & NVS_STS_SPEW_MSG)
		dev_info(&client->dev, "%s\n", __func__);
}

static int bmi_remove(struct i2c_client *client)
{
	struct bmi_state *st = i2c_get_clientdata(client);
	unsigned int i;

	if (st != NULL) {
		bmi_shutdown(client);
		if (st->irq_setup_done) {
			nvs_gte_exit(&st->i2c->dev, st->gis, BMI_HW_N);
			st->irq_setup_done = false; /* clear IRQ setup flag */
		}

		if (st->nvs) {
			for (i = 0; i < st->hw_n; i++)
				st->nvs->remove(st->snsrs[i].nvs_st);
		}

		if (st->wq) {
			destroy_workqueue(st->wq);
			st->wq = NULL;
		}
		bmi_pm_exit(st);
		/* TODO: st->buf_? should move to fn_pm disable */
		if (st->buf_acc) {
			devm_kfree(&st->i2c->dev, st->buf_acc);
			st->buf_acc = NULL;
		}
		if (st->buf_gyr) {
			devm_kfree(&st->i2c->dev, st->buf_gyr);
			st->buf_gyr = NULL;
		}
	}
	dev_info(&client->dev, "%s\n", __func__);
	return 0;
}

static int bmi_id_dev(struct bmi_state *st, unsigned int hw)
{
	u8 val;
	unsigned int i;
	int ret;

	ret = bmi_i2c_rd(st, hw, bmi_hws[hw].chip_id_reg, 1, &val);
	if (!ret) {
		for (i = 0; i < BMI_PART_N; i++) {
			if (val == bmi_hws[hw].chip_ids[i])
				break;
		}
		if (i < BMI_PART_N) {
			/* found device */
			if (i > st->part)
				/* only store if can confirm unique part */
				st->part = i;
			else
				i = BMI_PART_AUTO;
			dev_info(&st->i2c->dev, "%s %s found in %s\n",
				 __func__, bmi_snsr_cfgs[hw].name,
				 bmi_i2c_device_ids[i].name);
		} else {
			ret = -ENODEV;
		}
	}
	return ret;
}

static int bmi_id_i2c(struct bmi_state *st, unsigned int hw)
{
	unsigned int i;
	unsigned int n;
	int ret = -ENODEV;

	for (i = 0; i < hw; i++) {
		if (bmi_hws[i].i2c_addrs == bmi_hws[hw].i2c_addrs) {
			/* we've already searched for these I2C addresses */
			if (st->i2c_addrs[i]) {
				/* and already found this I2C device */
				st->i2c_addrs[hw] = st->i2c_addrs[i];
				dev_info(&st->i2c->dev, "%s %s found in %s\n",
					 __func__, bmi_snsr_cfgs[hw].name,
					 bmi_i2c_device_ids[st->part].name);
				return 0;
			}

			return -ENODEV;
		}
	}

	n = bmi_hws[hw].i2c_addrs_n;
	for (i = 0; i < n; i++) {
		if (st->i2c->addr == bmi_hws[hw].i2c_addrs[i])
			break;
	}
	if (i < n) {
		st->i2c_addrs[hw] = st->i2c->addr;
		ret = bmi_id_dev(st, hw);
	}
	if (ret) {
		for (i = 0; i < n; i++) {
			st->i2c_addrs[hw] = bmi_hws[hw].i2c_addrs[i];
			ret = bmi_id_dev(st, hw);
			if (!ret)
				break;
		}

		if (ret)
			st->i2c_addrs[hw] = 0;
	}
	return ret;
}

static int bmi_of_dt(struct bmi_state *st, struct device_node *dn)
{
	u8 val8;
	u32 val32;

	if (dn) {
		if (!st->i2c_addrs[BMI_HW_ACC]) {
			if (!of_property_read_u32(dn, "accelerometer_i2c_addr",
						  &val32))
				st->i2c_addrs[BMI_HW_ACC] = val32;
		}
		/* driver specific device tree parameters */
		st->gis[BMI_HW_ACC].gpio =
			    of_get_named_gpio(dn, "accelerometer_irq_gpio", 0);
		if (!of_property_read_u32(dn, "accelerometer_buffer_size",
					  &val32))
			st->buf_acc_n = val32;
		if (!of_property_read_u32(dn, "accelerometer_register_0x40",
					  &val32)) {
			val8 = val32 & BMI_REG_ACC_CONF_BWP_MSK;
			st->ra_0x40 = val8;
		}
/* not needed - but note that it could be optional
 *		if (!of_property_read_u32(dn, "accelerometer_register_0x48",
 *					  &val32)) {
 *			val8 = val32 & BMI_REG_ACC_FIFO_CFG_0_MSK;
 *			val8 |= BMI_REG_ACC_FIFO_CFG_0_DFLT;
 *			st->ra_0x48 = val8;
 *		}
 */
		if (!of_property_read_u32(dn, "accelerometer_register_0x49",
					  &val32)) {
			val8 = val32 & BMI_REG_ACC_FIFO_CFG_0_MSK;
			val8 |= BMI_REG_ACC_FIFO_CFG_0_DFLT;
			st->ra_0x49 = val8;
		}
		if (!of_property_read_u32(dn, "accelerometer_register_0x53",
					  &val32)) {
			val8 = val32;
			st->ra_0x53 = val8;
		}
		if (!of_property_read_u32(dn, "accelerometer_register_0x54",
					  &val32)) {
			val8 = val32;
			st->ra_0x54 = val8;
		}
		if (!of_property_read_u32(dn, "accelerometer_register_0x58",
					  &val32)) {
			val8 = val32;
			st->ra_0x58 = val8;
		}
		st->gis[BMI_HW_GYR].gpio =
				of_get_named_gpio(dn, "gyroscope_irq_gpio", 0);
		if (!of_property_read_u32(dn, "gyroscope_buffer_size",
					  &val32)) {
			/* needs to be divisible by 6 */
			val32 /= BMI_REG_GYR_DATA_N;
			/* buf_gyr_n == frame count (event is 6 bytes) */
			st->buf_gyr_n = val32;
		}
		if (!of_property_read_u32(dn, "gyroscope_register_0x16",
					  &val32)) {
			val8 = val32;
			st->rg_0x16 = val8;
		}
		if (!of_property_read_u32(dn, "gyroscope_register_0x18",
					  &val32)) {
			val8 = 0;
			if (val32 & BMI_REG_INT_3_4_IO_MAP_INT3)
				val8 |= BMI_REG_INT_3_4_IO_MAP_INT3;
			if (val32 & BMI_REG_INT_3_4_IO_MAP_INT4)
				val8 |= BMI_REG_INT_3_4_IO_MAP_INT4;
			st->rg_0x18 = val8;
		}
		if (!of_property_read_u32(dn, "gyroscope_register_0x34",
					  &val32)) {
			val8 = val32;
			st->rg_0x34 = val8;
		}
	}
	return 0;
}

static int bmi_init(struct bmi_state *st, const struct i2c_device_id *id)
{
	unsigned long irqflags;
	unsigned int i;
	unsigned int n;
	int lo;
	int hi;
	int ret;

	/* driver specific defaults */
	for (i = 0; i < BMI_HW_N; i++) {
		st->hw2ids[i] = -1;
		st->gis[i].gpio = -1;
	}

	st->buf_acc_n = BMI_ACC_BUF_SZ;
	st->buf_gyr_n = BMI_GYR_BUF_SZ / BMI_REG_GYR_DATA_N;
	st->ra_0x40 = BMI_REG_ACC_CONF_BWP_POR;
	st->ra_0x48 = BMI_REG_ACC_FIFO_CFG_0_DFLT;
	st->ra_0x49 = BMI_REG_ACC_FIFO_CFG_1_DFLT;
	st->ra_0x53 = 0x0A;
	st->ra_0x54 = 0x00;
	st->ra_0x58 = 0x04;
	st->rg_0x16 = 0x01;
	st->rg_0x18 = BMI_REG_INT_3_4_IO_MAP_INT3;
	bmi_pm_init(st); /* power up */
	if (id->driver_data >= BMI_PART_AUTO) {
		n = 0;
		for (i = 0; i < BMI_HW_N; i++) {
			ret = bmi_id_i2c(st, i);
			if (!ret)
				n++;
		}
		if (!n) {
			/* no devices were found */
			dev_err(&st->i2c->dev, "%s _id_i2c ERR\n", __func__);
			return -ENODEV;
		}
	} else {
		st->part = id->driver_data;
		/* The gyroscope is in full power after the IMU POR. Therefore
		 * we need to know the I2C address of the gyroscope so we can
		 * put it in suspend mode now in case the device is "disabled".
		 */
		st->i2c_addrs[BMI_HW_GYR] = st->i2c->addr;
	}

	bmi_disable(st, -1); /* disable all devices and power down */
	bmi_fn_dev.errs = &st->errs;
	bmi_fn_dev.sts = &st->sts;
	st->nvs = nvs_auto(NVS_CFG_KIF);
	if (st->nvs == NULL) {
		dev_err(&st->i2c->dev, "%s nvs_ ERR\n", __func__);
		return -ENODEV;
	}

	n = 0;
	for (i = 0; i < BMI_HW_N; i++) {
		memcpy(&st->snsrs[n].cfg, &bmi_snsr_cfgs[i],
		       sizeof(st->snsrs[n].cfg));
		nvs_of_dt(st->i2c->dev.of_node, &st->snsrs[n].cfg, NULL);
		ret = st->nvs->probe(&st->snsrs[n].nvs_st, st, &st->i2c->dev,
				     &bmi_fn_dev, &st->snsrs[n].cfg);
		if (!ret) {
			st->hw2ids[i] = n;
			st->snsrs[n].cfg.snsr_id = n;
			st->snsrs[n].cfg.part =
					     bmi_i2c_device_ids[st->part].name;
			st->snsrs[n].hw = i;
			if (bmi_hws[i].rrs) {
				st->snsrs[n].rrs = &bmi_hws[i].rrs[st->part];
				bmi_max_range(st, n,
					      st->snsrs[n].cfg.max_range.ival);
			} else {
				st->snsrs[n].rrs = NULL;
			}
			n++;
		}
	}
	if (!n)
		return -ENODEV;

	st->hw_n = n;
	i = st->hw2ids[BMI_HW_TMP];
	if (i < BMI_HW_N) {
		st->oc.cfg = &st->snsrs[i].cfg;
		st->oc.nvs_st = st->snsrs[i].nvs_st;
		st->oc.handler = st->nvs->handler;
		nvs_on_change_of_dt(&st->oc, st->i2c->dev.of_node, NULL);
		st->oc.hw_min = 0;
		st->oc.hw_max = 0xFFFF;
		st->oc.reverse_range_en = false;
		st->oc.scale_float_en = false;
		st->oc.binary_report_en = false;
	}
	ret = bmi_of_dt(st, st->i2c->dev.of_node);
	if (ret) {
		dev_err(&st->i2c->dev, "%s _of_dt ERR\n", __func__);
		return ret;
	}

	/* determine operating mode */
	for (i = 0; i < st->hw_n; i++) {
		if (!bmi_hws[st->snsrs[i].hw].fn_irqflags)
			/* sensor doesn't take an IRQ */
			continue;

		if (st->gis[st->snsrs[i].hw].gpio < 0) {
			/* device has no IRQ defined putting us in poll mode */
			st->op_mode = BMI_OP_MODE_POLL;
			break;
		}

		for (n = 0; n < i; n++) {
			if (st->gis[st->snsrs[n].hw].gpio ==
					       st->gis[st->snsrs[i].hw].gpio) {
				/* a previous device has same IRQ (shared) */
				st->op_mode = BMI_OP_MODE_IRQ_SHARED;
				break;
			}
		}
		if (n < i)
			/* exit outside loop */
			break;
	}
	if (i >= st->hw_n)
		/* devices have their own IRQ */
		st->op_mode = BMI_OP_MODE_IRQ;

	if (st->op_mode == BMI_OP_MODE_POLL ||
				       st->op_mode == BMI_OP_MODE_IRQ_SHARED) {
		/* devices must be run at the same speed so we adjust all
		 * the devices to have common timing period capabilities.
		 */
		lo = ((int)(~0U >> 1));
		hi = BMI_OP_MODE_SHARED_DELAY_US_MIN;
		for (i = 0; i < st->hw_n; i++) {
			if (st->snsrs[i].cfg.delay_us_min > hi)
				/* slowest of the fastest */
				hi = st->snsrs[i].cfg.delay_us_min;
			if (st->snsrs[i].cfg.delay_us_max < lo)
				/* fastest of the slowest */
				lo = st->snsrs[i].cfg.delay_us_max;
		}

		st->period_us_max = lo;
		st->period_us = lo;
		for (i = 0; i < st->hw_n; i++) {
			st->snsrs[i].cfg.delay_us_min = hi;
			st->snsrs[i].cfg.delay_us_max = lo;
			/* disable batching (it requires separate IRQs) */
			st->snsrs[i].cfg.fifo_max_evnt_cnt = 0;
			st->snsrs[i].cfg.fifo_rsrv_evnt_cnt = 0;
		}

		if (st->op_mode == BMI_OP_MODE_POLL) {
			st->buf_gyr_n = 1; /* one GYR event (6 bytes) */
			st->buf_acc_n = BMI_REG_ACC_DATA_N;
			i = st->part;
			st->wq = create_workqueue(bmi_i2c_device_ids[i].name);
			if (!st->wq) {
				dev_err(&st->i2c->dev,
					"%s create_workqueue ERR\n", __func__);
				return -ENODEV;
			}

			INIT_WORK(&st->ws, bmi_work);
		}
	}

	if (st->op_mode == BMI_OP_MODE_IRQ_SHARED ||
					      st->op_mode == BMI_OP_MODE_IRQ) {
		i = st->part;
		for (n = 0; n < BMI_HW_N; n++)
			/* assume IRQ shared or disabled devices (no NULLs) */
			st->gis[n].dev_name = bmi_i2c_device_ids[i].name;

		if (st->op_mode == BMI_OP_MODE_IRQ) {
			for (n = 0; n < BMI_HW_N; n++) {
				i = st->snsrs[n].hw;
				if (i < BMI_HW_N)
					st->gis[n].dev_name =
							 st->snsrs[i].cfg.name;
			}
		}

		ret = nvs_gte_init(&st->i2c->dev, st->gis, BMI_HW_N);
		if (ret < 0)
			return ret;

		if (st->op_mode == BMI_OP_MODE_IRQ_SHARED)
			n = st->hw_n - 1; /* single loop iteration */
		else /* st->op_mode == BMI_OP_MODE_IRQ */
			n = 0;
		for (; n < st->hw_n; n++) {
			i = st->snsrs[n].hw;
			if (bmi_hws[i].fn_irqflags) {
				irqflags = bmi_hws[i].fn_irqflags(st);
				ret = request_threaded_irq(st->gis[i].irq,
							   bmi_irq_handler,
							   bmi_irq_thread,
							   irqflags,
							   st->gis[i].dev_name,
							   st);
				if (ret) {
					dev_err(&st->i2c->dev,
						"%s req_threaded_irq ERR %d\n",
						__func__, ret);
					return -ENOMEM;
				}
			}
		}

		st->irq_setup_done = true;
	}

	/* default rate to slowest speed in case enabled without rate */
	for (i = 0; i < st->hw_n; i++)
		st->snsrs[i].period_us = st->snsrs[i].cfg.delay_us_max;

	/* Disabling batching due to BMI088 FIFO watermark interrupt... um...
	 * let's say, "issues". Watermark interrupt triggers the first time but
	 * not subsequently without a register reconfiguration which resets the
	 * FIFO causing loss of events. TODO: figure out a WAR.
	 */
	for (i = 0; i < st->hw_n; i++) {
		st->snsrs[i].cfg.fifo_max_evnt_cnt = 0;
		st->snsrs[i].cfg.fifo_rsrv_evnt_cnt = 0;
	}

	return ret;
}

static int bmi_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct bmi_state *st;
	int ret;

	/* just test if global disable */
	ret = nvs_of_dt(client->dev.of_node, NULL, NULL);
	if (ret == -ENODEV) {
		dev_info(&client->dev, "%s DT disabled\n", __func__);
		return ret;
	}

	st = devm_kzalloc(&client->dev, sizeof(*st), GFP_KERNEL);
	if (st == NULL)
		return -ENOMEM;

	i2c_set_clientdata(client, st);
	st->i2c = client;
	ret = bmi_init(st, id);
	if (ret)
		bmi_remove(client);
	dev_info(&client->dev, "%s done\n", __func__);
	device_unblock_probing();
	return ret;
}

MODULE_DEVICE_TABLE(i2c, bmi_i2c_device_ids);

static const struct of_device_id bmi_of_match[] = {
	{ .compatible = "bmi,bmi085", },
	{ .compatible = "bmi,bmi088", },
	{ .compatible = "bmi,bmi08x", },
	{}
};

MODULE_DEVICE_TABLE(of, bmi_of_match);

static struct i2c_driver bmi_driver = {
	.class				= I2C_CLASS_HWMON,
	.probe				= bmi_probe,
	.remove				= bmi_remove,
	.shutdown			= bmi_shutdown,
	.driver				= {
		.name			= BMI_NAME,
		.owner			= THIS_MODULE,
		.of_match_table		= of_match_ptr(bmi_of_match),
		.pm			= &bmi_pm_ops,
	},
	.id_table			= bmi_i2c_device_ids,
};

module_i2c_driver(bmi_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("BMI08x driver");
MODULE_AUTHOR("NVIDIA Corporation");

