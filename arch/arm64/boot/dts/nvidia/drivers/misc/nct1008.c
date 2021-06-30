/*
 * drivers/misc/nct1008.c
 *
 * Driver for NCT1008, temperature monitoring device from ON Semiconductors
 *
 * Copyright (c) 2010-2018, NVIDIA CORPORATION.  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */


#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/nct1008.h>
#include <linux/delay.h>
#include <linux/thermal.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/version.h>
#include <soc/tegra/fuse.h>
#include <dt-bindings/misc/nvidia,nct1008.h>

/* Register Addresses used in this module. */
#define LOC_TEMP_RD                  0x00
#define EXT_TEMP_HI_RD               0x01
#define STATUS_RD                    0x02
#define CONFIG_RD                    0x03
#define CONV_RATE_RD                 0x04
#define LOC_TEMP_HI_LIMIT_RD         0x05
#define LOC_TEMP_LO_LIMIT_RD         0x06
#define EXT_TEMP_HI_LIMIT_HI_BYTE_RD 0x07
#define EXT_TEMP_LO_LIMIT_HI_BYTE_RD 0x08
#define CONFIG_WR                    0x09
#define CONV_RATE_WR                 0x0A
#define LOC_TEMP_HI_LIMIT_WR         0x0B
#define LOC_TEMP_LO_LIMIT_WR         0x0C
#define EXT_TEMP_HI_LIMIT_HI_BYTE_WR 0x0D
#define EXT_TEMP_LO_LIMIT_HI_BYTE_WR 0x0E
#define ONE_SHOT                     0x0F
#define EXT_TEMP_LO_RD               0x10
#define OFFSET_WR                    0x11
#define OFFSET_QUARTER_WR            0x12
#define EXT_TEMP_HI_LIMIT_LO_BYTE    0x13
#define EXT_TEMP_LO_LIMIT_LO_BYTE    0x14
/* NOT USED                          0x15 */
/* NOT USED                          0x16 */
/* NOT USED                          0x17 */
/* NOT USED                          0x18 */
#define EXT_THERM_LIMIT_WR           0x19
/* NOT USED                          0x1A */
/* NOT USED                          0x1B */
/* NOT USED                          0x1C */
/* NOT USED                          0x1D */
/* NOT USED                          0x1E */
/* NOT USED                          0x1F */
#define LOC_THERM_LIMIT              0x20
#define THERM_HYSTERESIS             0x21
#define COSECUTIVE_ALERT             0x22
#define NFACTOR_CORRECTION           0x23
#define MANUFACTURER_ID              0xFE
#define MAX6649_LOC_TEMP_LO_RD       0x11

/* Tdiode Offset fuse is stored in usigned Q5.2 Fixed Point format */
#define CP_INT		(5)
#define CP_FRAC		(2)
#define MASK_CP1	(0x7F)
#define SHIFT_CP1	(0)
#define SHIFT_CP2	(14)
#define MASK_CP2	(0x7F << SHIFT_CP2)
#define FIXED_TO_MILLI_C(val, cp) \
	(((((val) & MASK_##cp) >> SHIFT_##cp) * 1000) / (1 << CP_FRAC))

#define OFFSET_FRAC_BITS	(4)
#define OFFSET_FRAC_MULT	(1 << OFFSET_FRAC_BITS)
#define OFFSET_FRAC_MASK	(0xf)

/* Temperatures at which offsets are captured in K*/
#define TC1 (25000)
#define TC2 (105000)
#define TK1 (273150 + TC1)
#define TK2 (273150 + TC2)
/*
 * optimized nfactor for an ideal TMP451 sensor specified in datasheet is
 * 1.008, but since it changes in increments of 1/2088 steps we need 5 fraction
 * bits to maintain precision and hence is multipled by 10^5.
 */
#define TMP451_NFACTOR (100800)
/* step size in which nfactor can increment is 1/2088 specified in datasheet */
#define TMP451_NFACTOR_STEP (2088)

/* Configuration register bits. */
#define EXTENDED_RANGE_BIT BIT(2)
#define THERM2_BIT         BIT(5)
#define STANDBY_BIT        BIT(6)
#define ALERT_BIT          BIT(7)

/* Status register trip point bits. */
#define EXT_LO_BIT BIT(3) /* External Sensor has tripped 'temp <= LOW' */
#define EXT_HI_BIT BIT(4) /* External Sensor has tripped 'temp > HIGH' */
#define LOC_LO_BIT BIT(5) /* Local Sensor has tripped 'temp <= LOW' */
#define LOC_HI_BIT BIT(6) /* Local Sensor has tripped 'temp > HIGH' */

/* Constants.  */
#define EXTENDED_RANGE_OFFSET 64U
#define STANDARD_RANGE_MAX    127U
#define EXTENDED_RANGE_MAX   (150U + EXTENDED_RANGE_OFFSET)

#define NCT1008_MIN_TEMP       (-64L)
#define NCT1008_MAX_TEMP         191L
#define NCT1008_MAX_TEMP_MILLI   191750

#define MAX_STR_PRINT            50
#define NCT_CONV_TIME_ONESHOT_US	52000
#define TMP451_CONV_TIME_ONESHOT_US	31000

#define CELSIUS_TO_MILLICELSIUS(x) ((x)*1000)
#define MILLICELSIUS_TO_CELSIUS(x) ((x)/1000)

#define LOC TEGRA_NCT_SENSOR_LOC
#define EXT TEGRA_NCT_SENSOR_EXT
#define CNT TEGRA_NCT_SENSOR_MAX

struct nct1008_sensor_data {
	struct thermal_zone_device *thz;
	long current_hi_limit;
	long current_lo_limit;
	int shutdown_limit;
	int temp;
};

struct nct1008_data {
	struct workqueue_struct *workqueue;
	struct work_struct work;
	struct i2c_client *client;
	struct nct1008_platform_data plat_data;
	struct mutex mutex;
	u8 config;
	enum nct1008_chip chip;
	char chip_name[I2C_NAME_SIZE];
	struct regulator *nct_reg;
	int oneshot_conv_period_ns;
	int nct_disabled;
	int stop_workqueue;
	struct nct1008_sensor_data sensors[CNT];
};

static const unsigned long THERM_WARN_RANGE_HIGH_OFFSET = 3000;
static unsigned long nct1008_shutdown_warning_cur_state;
static long shutdown_warn_saved_temp;

static inline s16 value_to_temperature(bool extended, u8 value)
{
	return extended ? (s16)(value - EXTENDED_RANGE_OFFSET) : (s16)value;
}

static inline u8 temperature_to_value(bool extended, s16 temp)
{
	return extended ? (u8)(temp + EXTENDED_RANGE_OFFSET) : (u8)temp;
}

static int nct1008_write_reg(struct nct1008_data *data, u8 reg, u16 value)
{
	int ret = -ENODEV;

	if (!data)
		return ret;

	mutex_lock(&data->mutex);
	if (data->nct_disabled) {
		mutex_unlock(&data->mutex);
		goto err;
	}

	ret = i2c_smbus_write_byte_data(data->client, reg, value);
	mutex_unlock(&data->mutex);

err:
	if (ret < 0)
		dev_err(&data->client->dev, "write reg err %d\n", ret);

	return ret;
}

static int nct1008_read_reg(struct nct1008_data *data, u8 reg)
{
	int ret = -ENODEV;

	if (!data)
		return ret;

	mutex_lock(&data->mutex);
	if (data->nct_disabled) {
		mutex_unlock(&data->mutex);
		goto err;
	}

	ret = i2c_smbus_read_byte_data(data->client, reg);
	mutex_unlock(&data->mutex);

err:
	if (ret < 0)
		dev_err(&data->client->dev, "read reg err %d\n", ret);

	return ret;
}

static int nct1008_get_temp_common(int sensor, struct nct1008_data *data,
					int *temp)
{
	struct nct1008_platform_data *pdata = &data->plat_data;
	s16 temp_hi;
	s16 temp_lo = 0;
	long temp_milli = 0;
	u8 value;
	int ret;

	if (!((sensor == EXT) || (sensor == LOC)))
		return -1;

	/* Read External Temp */
	if (sensor == EXT) {
		ret = nct1008_read_reg(data, EXT_TEMP_LO_RD);
		if (ret < 0)
			return -1;
		else
			value = ret;

		temp_lo = (value >> 6);

		ret = nct1008_read_reg(data, EXT_TEMP_HI_RD);
		if (ret < 0)
			return -1;
		else
			value = ret;

		temp_hi = value_to_temperature(pdata->extended_range, value);
		temp_milli = CELSIUS_TO_MILLICELSIUS(temp_hi) + temp_lo * 250;

	} else if (sensor == LOC) {
		ret = nct1008_read_reg(data, LOC_TEMP_RD);
		if (ret < 0)
			return -1;
		else
			value = ret;
		temp_hi = value_to_temperature(pdata->extended_range, value);

		if (data->chip == MAX6649)
		{
			ret = nct1008_read_reg(data, MAX6649_LOC_TEMP_LO_RD);
			if(ret < 0)
				return -1;
			else
				value = ret;
			temp_lo = (value >> 6);
		}
		temp_milli = CELSIUS_TO_MILLICELSIUS(temp_hi) + temp_lo * 250;
	}

	if (temp_milli > NCT1008_MAX_TEMP_MILLI)
		return -1;

	*temp = temp_milli;
	data->sensors[sensor].temp = temp_milli;

	return 0;
}

static ssize_t nct1008_show_temp(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct1008_data *data = i2c_get_clientdata(client);
	struct nct1008_platform_data *pdata = &data->plat_data;

	s16 temp1 = 0;
	s16 temp = 0;
	u8 temp2 = 0;
	int value = 0;

	if (!dev || !buf || !attr)
		return -EINVAL;

	value = nct1008_read_reg(data, LOC_TEMP_RD);
	if (value < 0)
		goto error;

	temp1 = value_to_temperature(pdata->extended_range, value);
	if(data->chip == MAX6649)
	{
		value = nct1008_read_reg(data, MAX6649_LOC_TEMP_LO_RD);
		if(value < 0)
			goto error;

		temp2 = (value >> 6);
		return snprintf(buf, MAX_STR_PRINT, "%d.%d\n",
			temp1, temp2 * 25);
	}

	value = nct1008_read_reg(data, EXT_TEMP_LO_RD);
	if (value < 0)
		goto error;

	temp2 = (value >> 6);
	value = nct1008_read_reg(data, EXT_TEMP_HI_RD);
	if (value < 0)
		goto error;

	temp = value_to_temperature(pdata->extended_range, value);

	return snprintf(buf, MAX_STR_PRINT, "%d %d.%d\n",
		temp1, temp, temp2 * 25);

error:
	return snprintf(buf, MAX_STR_PRINT,
		"Error read local/ext temperature\n");
}

static ssize_t nct1008_show_temp_overheat(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct1008_data *data = i2c_get_clientdata(client);
	struct nct1008_platform_data *pdata = &data->plat_data;
	int value;
	s16 temp, temp2;

	/* Local temperature h/w shutdown limit */
	value = nct1008_read_reg(data, LOC_THERM_LIMIT);
	if (value < 0)
		goto error;

	temp = value_to_temperature(pdata->extended_range, value);
	/* External temperature h/w shutdown limit */
	value = nct1008_read_reg(data, EXT_THERM_LIMIT_WR);
	if (value < 0)
		goto error;

	temp2 = value_to_temperature(pdata->extended_range, value);

	return snprintf(buf, MAX_STR_PRINT, "%d %d\n", temp, temp2);
error:
	return snprintf(buf, MAX_STR_PRINT, " Rd overheat Error\n");
}

static ssize_t nct1008_set_temp_overheat(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int num;
	int err;
	int temp;
	struct i2c_client *client = to_i2c_client(dev);
	struct nct1008_data *data = i2c_get_clientdata(client);

	if (kstrtoint(buf, 0, &num))
		return -EINVAL;

	if ((num < NCT1008_MIN_TEMP) || (num >= NCT1008_MAX_TEMP)) {
		dev_err(dev, "Out of bounds temperature limit\n");
		return -EINVAL;
	}

	/* check for system power down */
	err = nct1008_get_temp_common(EXT, data, &temp);
	if (err)
		goto error;

	temp = MILLICELSIUS_TO_CELSIUS(temp);
	if (temp >= num)
		dev_warn(dev, "new overheat temp=%d exceeds curr temp=%dn\n",
				num, temp);

	/* External temperature h/w shutdown limit */
	temp = temperature_to_value(data->plat_data.extended_range, (s16)num);
	err = nct1008_write_reg(data, EXT_THERM_LIMIT_WR, temp);
	if (err < 0)
		goto error;

	/* Local temperature h/w shutdown limit */
	temp = temperature_to_value(data->plat_data.extended_range, (s16)num);
	err = nct1008_write_reg(data, LOC_THERM_LIMIT, temp);
	if (err < 0)
		goto error;

	data->sensors[EXT].shutdown_limit = num;
	return count;
error:
	dev_err(dev, "failed to set temperature-overheat\n");
	return err;
}

static ssize_t nct1008_show_temp_alert(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct1008_data *data = i2c_get_clientdata(client);
	struct nct1008_platform_data *pdata = &data->plat_data;
	int value;
	s16 temp_hi, temp_lo;

	/* External Temperature Throttling hi-limit */
	value = nct1008_read_reg(data, EXT_TEMP_HI_LIMIT_HI_BYTE_RD);
	if (value < 0)
		goto error;

	temp_hi = value_to_temperature(pdata->extended_range, value);
	/* External Temperature Throttling lo-limit */
	value = nct1008_read_reg(data, EXT_TEMP_LO_LIMIT_HI_BYTE_RD);
	if (value < 0)
		goto error;

	temp_lo = value_to_temperature(pdata->extended_range, value);

	return snprintf(buf, MAX_STR_PRINT, "lo:%d hi:%d\n", temp_lo, temp_hi);
error:
	dev_err(dev, "%s: failed to read temperature-alert\n", __func__);
	return snprintf(buf, MAX_STR_PRINT, " Rd alert Error\n");
}

static ssize_t nct1008_set_temp_alert(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	int val;
	int err;
	struct i2c_client *client = to_i2c_client(dev);
	struct nct1008_data *data = i2c_get_clientdata(client);
	struct nct1008_platform_data *pdata = &data->plat_data;

	if (kstrtoint(buf, 0, &val))
		return -EINVAL;

	if ((val < NCT1008_MIN_TEMP) || (val >= NCT1008_MAX_TEMP)) {
		dev_err(dev, "Out of bounds temperature limit\n");
		return -EINVAL;
	}

	/* External Temperature Throttling limit */
	val = temperature_to_value(pdata->extended_range, (s16)val);
	err = nct1008_write_reg(data, EXT_TEMP_HI_LIMIT_HI_BYTE_WR, val);
	if (err < 0)
		goto error;

	/* Local Temperature Throttling limit */
	err = nct1008_write_reg(data, LOC_TEMP_HI_LIMIT_WR, val);
	if (err < 0)
		goto error;

	return count;
error:
	dev_err(dev, "%s: failed to set temperature-alert\n", __func__);
	return err;
}

static ssize_t nct1008_show_ext_temp(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct1008_data *data = i2c_get_clientdata(client);
	struct nct1008_platform_data *pdata = &data->plat_data;
	s16 temp_value;
	int val = 0;
	int data_lo;

	if (!dev || !buf || !attr)
		return -EINVAL;

	/* When reading the full external temperature value, read the
	 * LSB first. This causes the MSB to be locked (that is, the
	 * ADC does not write to it) until it is read */
	data_lo = nct1008_read_reg(data, EXT_TEMP_LO_RD);
	if (data_lo < 0)
		goto error;

	val = nct1008_read_reg(data, EXT_TEMP_HI_RD);
	if (val < 0)
		goto error;

	temp_value = value_to_temperature(pdata->extended_range, val);

	return snprintf(buf, MAX_STR_PRINT, "%d.%d\n", temp_value,
			(25 * (data_lo >> 6)));
error:
	return snprintf(buf, MAX_STR_PRINT, "Error read ext temperature\n");
}

static ssize_t pr_reg(struct nct1008_data *nct, char *buf, int max_s,
			const char *reg_name, int offset)
{
	int ret, sz = 0;

	ret = nct1008_read_reg(nct, offset);
	if (ret >= 0)
		sz += snprintf(buf + sz, PAGE_SIZE - sz,
				"%20s  0x%02x  0x%02x  0x%02x\n",
				reg_name, nct->client->addr, offset, ret);
	else
		sz += snprintf(buf + sz, PAGE_SIZE - sz,
				"%s: line=%d, i2c ** read error=%d **\n",
				__func__, __LINE__, ret);
	return sz;
}

static ssize_t nct1008_show_regs(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct1008_data *nct = i2c_get_clientdata(client);
	int sz = 0;

	sz += snprintf(buf + sz, PAGE_SIZE - sz,
		"%s Registers\n", nct->chip_name);
	sz += snprintf(buf + sz, PAGE_SIZE - sz,
		"---------------------------------------\n");
	sz += snprintf(buf + sz, PAGE_SIZE - sz, "%20s  %4s  %4s  %s\n",
		"Register Name       ", "Addr", "Reg", "Value");
	sz += snprintf(buf + sz, PAGE_SIZE - sz, "%20s  %4s  %4s  %s\n",
		"--------------------", "----", "----", "-----");
	sz += pr_reg(nct, buf+sz, PAGE_SIZE-sz,
		"Status              ", STATUS_RD);
	sz += pr_reg(nct, buf+sz, PAGE_SIZE-sz,
		"Configuration       ", CONFIG_RD);
	sz += pr_reg(nct, buf+sz, PAGE_SIZE-sz,
		"Conversion Rate     ", CONV_RATE_RD);
	sz += pr_reg(nct, buf+sz, PAGE_SIZE-sz,
		"Hysteresis          ", THERM_HYSTERESIS);
	sz += pr_reg(nct, buf+sz, PAGE_SIZE-sz,
		"Consecutive Alert   ", COSECUTIVE_ALERT);
	sz += pr_reg(nct, buf+sz, PAGE_SIZE-sz,
		"Local Temp Value    ", LOC_TEMP_RD);
	sz += pr_reg(nct, buf+sz, PAGE_SIZE-sz,
		"Local Temp Hi Limit ", LOC_TEMP_HI_LIMIT_RD);
	sz += pr_reg(nct, buf+sz, PAGE_SIZE-sz,
		"Local Temp Lo Limit ", LOC_TEMP_LO_LIMIT_RD);
	sz += pr_reg(nct, buf+sz, PAGE_SIZE-sz,
		"Local Therm Limit   ", LOC_THERM_LIMIT);
	sz += pr_reg(nct, buf+sz, PAGE_SIZE-sz,
		"Ext Temp Value Hi   ", EXT_TEMP_HI_RD);
	sz += pr_reg(nct, buf+sz, PAGE_SIZE-sz,
		"Ext Temp Value Lo   ", EXT_TEMP_LO_RD);
	sz += pr_reg(nct, buf+sz, PAGE_SIZE-sz,
		"Ext Temp Hi Limit Hi", EXT_TEMP_HI_LIMIT_HI_BYTE_RD);
	sz += pr_reg(nct, buf+sz, PAGE_SIZE-sz,
		"Ext Temp Lo Limit Hi", EXT_TEMP_LO_LIMIT_HI_BYTE_RD);
	if(nct->chip == MAX6649) {
		sz += pr_reg(nct, buf+sz, PAGE_SIZE-sz,
			"Local Temp Value lo ", MAX6649_LOC_TEMP_LO_RD);
	}
	else {
		sz += pr_reg(nct, buf+sz, PAGE_SIZE-sz,
			"Ext Temp Offset Hi  ", OFFSET_WR);
		sz += pr_reg(nct, buf+sz, PAGE_SIZE-sz,
			"Ext Temp Offset Lo  ", OFFSET_QUARTER_WR);
		sz += pr_reg(nct, buf+sz, PAGE_SIZE-sz,
			"Ext Temp Hi Limit Lo", EXT_TEMP_HI_LIMIT_LO_BYTE);
		sz += pr_reg(nct, buf+sz, PAGE_SIZE-sz,
			"Ext Temp Lo Limit Lo", EXT_TEMP_LO_LIMIT_LO_BYTE);
	}
	sz += pr_reg(nct, buf+sz, PAGE_SIZE-sz,
		"Ext Therm Limit     ", EXT_THERM_LIMIT_WR);
	sz += pr_reg(nct, buf+sz, PAGE_SIZE-sz,
		"ManufacturerID      ", MANUFACTURER_ID);

	return sz;
}

static ssize_t nct1008_set_nadjust(struct device *dev,
			struct device_attribute *attr, const char *buf,
			size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct1008_data *data = i2c_get_clientdata(client);
	int r, nadj;

	sscanf(buf, "%d", &nadj);
	r = nct1008_write_reg(data, NFACTOR_CORRECTION, nadj);
	if (r)
		return r;

	return count;
}

static ssize_t nct1008_show_nadjust(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct1008_data *data = i2c_get_clientdata(client);
	int nadj, nf, sz = 0;

	nadj = nct1008_read_reg(data, NFACTOR_CORRECTION);
	nf = (TMP451_NFACTOR * TMP451_NFACTOR_STEP) /
			(TMP451_NFACTOR_STEP + nadj);

	sz += snprintf(buf + sz, PAGE_SIZE - sz,
				"nadj: %d, nf: %d\n", nadj, nf);
	return sz;
}

static ssize_t nct1008_set_offset(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct1008_data *data = i2c_get_clientdata(client);
	int r = count, hi_b, lo_b;

	sscanf(buf, "%d %d", &hi_b, &lo_b);
	r = nct1008_write_reg(data, OFFSET_WR, hi_b);
	r = r ? r : nct1008_write_reg(data, OFFSET_QUARTER_WR, lo_b << 4);
	if (r)
		return r;

	return count;
}

static ssize_t nct1008_show_offset(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct nct1008_data *data = i2c_get_clientdata(client);
	int hi_b, lo_b, sz = 0;

	hi_b = nct1008_read_reg(data, OFFSET_WR);
	lo_b = nct1008_read_reg(data, OFFSET_QUARTER_WR);
	sz += snprintf(buf + sz, PAGE_SIZE - sz,
				"offset: %d, %d\n", hi_b, lo_b);
	return sz;
}

static DEVICE_ATTR(temperature, S_IRUGO, nct1008_show_temp, NULL);
static DEVICE_ATTR(temperature_overheat, (S_IRUGO | (S_IWUSR | S_IWGRP)),
		nct1008_show_temp_overheat, nct1008_set_temp_overheat);
static DEVICE_ATTR(temperature_alert, (S_IRUGO | (S_IWUSR | S_IWGRP)),
		nct1008_show_temp_alert, nct1008_set_temp_alert);
static DEVICE_ATTR(ext_temperature, S_IRUGO, nct1008_show_ext_temp, NULL);
static DEVICE_ATTR(registers, S_IRUGO, nct1008_show_regs, NULL);
static DEVICE_ATTR(offset, (S_IRUGO | (S_IWUSR | S_IWGRP)),
		nct1008_show_offset, nct1008_set_offset);
static DEVICE_ATTR(nadj, (S_IRUGO | (S_IWUSR | S_IWGRP)),
		nct1008_show_nadjust, nct1008_set_nadjust);

static struct attribute *nct1008_attributes[] = {
	&dev_attr_temperature.attr,
	&dev_attr_temperature_overheat.attr,
	&dev_attr_temperature_alert.attr,
	&dev_attr_ext_temperature.attr,
	&dev_attr_registers.attr,
	&dev_attr_offset.attr,
	&dev_attr_nadj.attr,
	NULL
};

static const struct attribute_group nct1008_attr_group = {
	.attrs = nct1008_attributes,
};
static int nct1008_shutdown_warning_get_max_state(
					struct thermal_cooling_device *cdev,
					unsigned long *max_state)
{
	/* A state for every 250mC */
	*max_state = THERM_WARN_RANGE_HIGH_OFFSET / 250;
	return 0;
}

static int nct1008_shutdown_warning_get_cur_state(
					struct thermal_cooling_device *cdev,
					unsigned long *cur_state)
{
	struct nct1008_data *data = cdev->devdata;
	long limit = data->sensors[EXT].shutdown_limit * 1000;
	int temp;

	if (nct1008_get_temp_common(EXT, data, &temp))
		return -1;

	if (temp >= (limit - THERM_WARN_RANGE_HIGH_OFFSET))
		*cur_state = nct1008_shutdown_warning_cur_state;
	else
		*cur_state = 0;

	return 0;
}

static int nct1008_shutdown_warning_set_cur_state(
					struct thermal_cooling_device *cdev,
					unsigned long cur_state)
{
	struct nct1008_data *data = cdev->devdata;
	long limit = data->sensors[EXT].shutdown_limit * 1000;
	int temp;

	if (nct1008_get_temp_common(EXT, data, &temp))
		return -1;
	else if (temp < 0)
		goto ret;

	if ((temp >= (limit - THERM_WARN_RANGE_HIGH_OFFSET)) &&
		(temp != shutdown_warn_saved_temp)) {
		pr_warn("%s: Warning: chip temperature (%d.%02dC) is %s SHUTDOWN limit (%c%ldC).\n",
			data->chip_name,
			temp / 1000, (temp % 1000) / 10,
			temp > limit ? "above" :
			temp == limit ? "at" : "near",
			temp > limit ? '>' : '<', limit / 1000);
		shutdown_warn_saved_temp = temp;
	}

 ret:
	nct1008_shutdown_warning_cur_state = cur_state;
	return 0;
}

static struct thermal_cooling_device_ops nct1008_shutdown_warning_ops = {
	.get_max_state = nct1008_shutdown_warning_get_max_state,
	.get_cur_state = nct1008_shutdown_warning_get_cur_state,
	.set_cur_state = nct1008_shutdown_warning_set_cur_state,
};

static int nct1008_thermal_set_limits(int sensor,
				      struct nct1008_data *data,
				      long lo_limit_mC,
				      long hi_limit_mC)
{
	int err;
	u8 value;
	struct i2c_client *client = data->client;
	bool extended_range = data->plat_data.extended_range;
	long lo_limit;
	long hi_limit;
	u8 reg;

	lo_limit = max(NCT1008_MIN_TEMP, MILLICELSIUS_TO_CELSIUS(lo_limit_mC));
	hi_limit = min(NCT1008_MAX_TEMP, MILLICELSIUS_TO_CELSIUS(hi_limit_mC));

	if (lo_limit >= hi_limit)
		return -EINVAL;

	if (data->sensors[sensor].current_lo_limit != lo_limit) {
		value = temperature_to_value(extended_range, lo_limit);
		reg = (sensor == LOC) ? LOC_TEMP_LO_LIMIT_WR :
					EXT_TEMP_LO_LIMIT_HI_BYTE_WR;
		err = nct1008_write_reg(data, reg, value);
		if (err)
			return err;

		data->sensors[sensor].current_lo_limit = lo_limit;
	}

	if (data->sensors[sensor].current_hi_limit != hi_limit) {
		value = temperature_to_value(extended_range, hi_limit);
		reg = (sensor == LOC) ? LOC_TEMP_HI_LIMIT_WR :
					EXT_TEMP_HI_LIMIT_HI_BYTE_WR;
		err = nct1008_write_reg(data, reg, value);
		if (err)
			return err;

		data->sensors[sensor].current_hi_limit = hi_limit;
	}

	dev_dbg(&client->dev, "limits set to lo:%ld hi:%ld\n", lo_limit,
			hi_limit);

	return 0;
}


static int nct1008_ext_get_temp_as_sensor(void *data, int *temp)
{
	return nct1008_get_temp_common(EXT, (struct nct1008_data *) data, temp);
}

static int nct1008_get_trend_as_sensor(int sensor, void *data, int trip,
		enum thermal_trend *trend)
{
	int ret, temp, trip_temp, last_temp;
	struct nct1008_data *nct_data = (struct nct1008_data *)data;
	struct thermal_zone_device *thz = nct_data->sensors[sensor].thz;
	*trend = THERMAL_TREND_STABLE;

	if (!thz)
		return 0;

	ret = thz->ops->get_trip_temp(thz, trip, &trip_temp);
	if (ret)
		return ret;

	mutex_lock(&thz->lock);
	temp = thz->temperature;
	last_temp = thz->last_temperature;
	mutex_unlock(&thz->lock);

	if (temp > trip_temp) {
		if (temp >= last_temp)
			*trend = THERMAL_TREND_RAISING;
		else
			*trend = THERMAL_TREND_STABLE;
	} else if (temp < trip_temp) {
		*trend = THERMAL_TREND_DROPPING;
	} else {
		*trend = THERMAL_TREND_STABLE;
	}

	return 0;
}

/* Helper function to get trend for the local sensor. */
static inline int nct1008_loc_get_trend_as_sensor(void *data, int trip,
		enum thermal_trend *trend)
{
	return nct1008_get_trend_as_sensor(LOC, data, trip, trend);
}

static inline int nct1008_ext_get_trend_as_sensor(void *data, int trip,
		enum thermal_trend *trend)
{
	return nct1008_get_trend_as_sensor(EXT, data, trip, trend);
}

static int nct1008_loc_get_temp_as_sensor(void *data, int *temp)
{
	return nct1008_get_temp_common(LOC, (struct nct1008_data *) data, temp);
}

static int nct1008_loc_set_trips(void *of_data, int low, int high)
{
	struct nct1008_data *data = (struct nct1008_data *)of_data;

	nct1008_thermal_set_limits(LOC, data, low, high);

	return 0;
}

static int nct1008_ext_set_trips(void *of_data, int low, int high)
{
	struct nct1008_data *data = (struct nct1008_data *)of_data;

	nct1008_thermal_set_limits(EXT, data, low, high);

	return 0;
}

static int nct1008_enable(struct i2c_client *client)
{
	struct nct1008_data *data = i2c_get_clientdata(client);
	return nct1008_write_reg(data, CONFIG_WR, data->config);
}

static int nct1008_disable(struct i2c_client *client)
{
	struct nct1008_data *data = i2c_get_clientdata(client);
	return nct1008_write_reg(data, CONFIG_WR, data->config | STANDBY_BIT);
}

static void nct1008_work_func(struct work_struct *work)
{
	struct nct1008_data *data = container_of(work, struct nct1008_data,
						work);
	struct i2c_client *client = data->client;
	int err;
	int st;

	mutex_lock(&data->mutex);
	if (data->stop_workqueue) {
		mutex_unlock(&data->mutex);
		return;
	}
	mutex_unlock(&data->mutex);

	err = nct1008_disable(data->client);
	if (err == -ENODEV)
		return;

	st = nct1008_read_reg(data, STATUS_RD);
	dev_dbg(&client->dev, "%s: interrupt (0x%08x)\n", data->chip_name, st);
	if ((st & (LOC_LO_BIT | LOC_HI_BIT)) && data->sensors[LOC].thz)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0)
		thermal_zone_device_update(data->sensors[LOC].thz,
					   THERMAL_EVENT_UNSPECIFIED);
#else
		thermal_zone_device_update(data->sensors[LOC].thz);
#endif

	if ((st & (EXT_LO_BIT | EXT_HI_BIT)) && data->sensors[EXT].thz)
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 9, 0)
		thermal_zone_device_update(data->sensors[EXT].thz,
					   THERMAL_EVENT_UNSPECIFIED);
#else
		thermal_zone_device_update(data->sensors[EXT].thz);
#endif

	/* Initiate one-shot conversion */
	err = nct1008_write_reg(data, ONE_SHOT, 0x1);
	if (err < 0)
		return;

	/* Give hardware necessary time to finish conversion */
	usleep_range(data->oneshot_conv_period_ns,
			data->oneshot_conv_period_ns + 1000);

	err = nct1008_read_reg(data, STATUS_RD);
	if (err < 0)
		return;

	nct1008_enable(data->client);

	enable_irq(data->client->irq);
}

static irqreturn_t nct1008_irq(int irq, void *dev_id)
{
	struct nct1008_data *data = dev_id;

	disable_irq_nosync(irq);
	queue_work(data->workqueue, &data->work);

	return IRQ_HANDLED;
}

static void nct1008_power_control(struct nct1008_data *data, bool is_enable)
{
	int ret;

	mutex_lock(&data->mutex);
	if (!data->nct_reg) {
		mutex_unlock(&data->mutex);
		return;
	}

	if (is_enable)
		ret = regulator_enable(data->nct_reg);
	else
		ret = regulator_disable(data->nct_reg);

	if (ret < 0)
		dev_err(&data->client->dev, "Error in %s %s VDD rail, "
			"error %d\n", (is_enable) ? "enabling" : "disabling",
			data->chip_name,
			ret);
	else
		dev_info(&data->client->dev, "success in %s %s VDD rail\n",
			(is_enable) ? "enabling" : "disabling",
			data->chip_name);
	data->nct_disabled = !is_enable;
	mutex_unlock(&data->mutex);
}

/*
 * CP2 = Tdiode - Tchuck at 105C in Q5.2 format
 * CP1 = Tdiode - Tchuck at 25C in Q5.2 format
 * TC1, TC2 are true physical temperatures 25C & 105C.
 * Treported2 = TC2 + CP2
 * to account for part to part variation use alpha beta and apply
 * linear correction to the actual temperature
 * Tr2 = alpha * (TC2 + CP2) + beta
 * Terr = Treported - Tphysical
 * Terr @ TC2 = TE2 = Tr2 - TC2
 * Terr @ TC1 = TE1 = Tr1 - TC1
 * slope = (TE2 - TE1)/(TC2 - TC1)
 *       = alpha * ((cp2 - cp1)/(TC2 - T1)) + alpha  - 1
 *
 * per TMP451 datasheet:
 * Terr = (nf - 1.008)/1.008 * Tk
 * So, Terr_slope = (nf - 1.008)/1.008 ...(1)
 *
 * Emperically TMP451 reports higher temperature when nFactor is reduced. This
 * means the Terr in (2) is negative and is being subtracted when the sensor
 * overestimates the temperature. Slope therefore has to be negative:
 * slope = - alpha * ((cp2 - cp1)/(TC2 - T1)) - alpha  + 1...(2)
 *
 * solving for nf from (1) & (2):
 * nf = -(1.008 * slope) + 1.008 ...(4)
 *
 * quantize and map nf to an signed int using the nfactor step size
 * nadj = (1.008 * 2088 / nFactor) - 2088
 *
 * After applying the nFactor and Offset, Tdiode's reported temperature should
 * be closer to the physical temperature. The adjusted temp is calculated as:
 * Tadjusted = Offset - slope * Tk + Treported
 * Assuming adjusted temp same as physical temp, calculate offset @ T2:
 * Offset = Tadjusted - Treported + slope * Tk
 *        = Tphysical - Treported + slope * Tk
 *        = -TE2 + slope * TK2
 *        = -alpha * (TC2 + CP2) - beta + TC2 + slope * TK1
*/
static int nct1008_offsets_program(struct nct1008_data *data)
{
	struct i2c_client *client = data->client;
	struct nct1008_platform_data *p = &data->plat_data;
	/* use DT based offset unless fuse_offset is present */
	int64_t off = p->offset, slope, nf = 0, nadj = 0, cp2, cp1;
	int r = 0, val, lo_b, hi_b;

	if (p->fuse_offset) {
		r = tegra_fuse_readl(FUSE_TDIODE_CALIB, &val);
		if (r)
			return r;

		cp2 = FIXED_TO_MILLI_C(val, CP2);
		cp1 = FIXED_TO_MILLI_C(val, CP1);

		/*
		 * alpha and beta have 4 fraction digits and is scaled by 10^4.
		 * nf is therefore scaled back by 10^4 in the end.
		 */

		slope = ((p->alpha * (cp2 - cp1)) / (TC2 - TC1)) + p->alpha -
				10000;
		nf = -((TMP451_NFACTOR * slope) / 10000) + TMP451_NFACTOR;
		nadj = (TMP451_NFACTOR * TMP451_NFACTOR_STEP / nf) -
				TMP451_NFACTOR_STEP;
		off = -p->alpha * (TC2 + cp2) - p->beta * 1000 + TC2 * 10000 -
				slope * TK2;
		off = off * OFFSET_FRAC_MULT / 10000000;
		r = nct1008_write_reg(data, NFACTOR_CORRECTION, nadj);
		dev_info(&client->dev,
			"nf:%lld, nadj:%lld, off:%lld\n", nf, nadj, off);
	}

	lo_b = ((off & OFFSET_FRAC_MASK) << OFFSET_FRAC_BITS);
	hi_b = off >> OFFSET_FRAC_BITS;
	dev_info(&client->dev, "hi_b:%d, lo_b:%d\n", hi_b, lo_b);
	r = r ? r : nct1008_write_reg(data, OFFSET_WR, hi_b);
	r = r ? r : nct1008_write_reg(data, OFFSET_QUARTER_WR, lo_b);

	return r;
}

static int nct1008_ext_sensor_init(struct nct1008_data *data)
{
	int ret, val;
	struct nct1008_platform_data *pdata = &data->plat_data;

	ret = nct1008_read_reg(data, STATUS_RD);
	if (ret & BIT(2)) {
		/* skip configuration of EXT sensor */
		dev_err(&data->client->dev, "EXT sensor circuit is open\n");
		return -ENODEV;
	}

	/* External temperature h/w shutdown limit. */
	if (data->sensors[EXT].shutdown_limit != INT_MIN) {
		val = temperature_to_value(pdata->extended_range,
					data->sensors[EXT].shutdown_limit);
		ret = nct1008_write_reg(data, EXT_THERM_LIMIT_WR, val);
		if (ret)
			goto err;

		dev_info(&data->client->dev, "EXT shutdown limit %d",
				data->sensors[EXT].shutdown_limit);
	}

	/* Setup external hi and lo limits */
	ret = nct1008_write_reg(data, EXT_TEMP_LO_LIMIT_HI_BYTE_WR, 0);
	if (ret)
		goto err;

	ret = nct1008_write_reg(data, EXT_TEMP_HI_LIMIT_HI_BYTE_WR,
				NCT1008_MAX_TEMP);
err:
	if (ret < 0)
		dev_err(&data->client->dev, "EXT sensor init failed 0x%x", ret);

	return ret;

}

static int nct1008_loc_sensor_init(struct nct1008_data *data)
{
	int ret = 0, val;
	struct nct1008_platform_data *pdata = &data->plat_data;

	/* Local temperature h/w shutdown limit */
	if (data->sensors[LOC].shutdown_limit != INT_MIN) {
		val = temperature_to_value(pdata->extended_range,
					data->sensors[LOC].shutdown_limit);
		ret = nct1008_write_reg(data, LOC_THERM_LIMIT, val);
		if (ret)
			goto err;

		dev_info(&data->client->dev, "LOC shutdown limit %d",
				data->sensors[LOC].shutdown_limit);
	}

	/* Setup local hi and lo limits. */
	ret = nct1008_write_reg(data, LOC_TEMP_HI_LIMIT_WR, NCT1008_MAX_TEMP);
	if (ret)
		goto err;

	ret = nct1008_write_reg(data, LOC_TEMP_LO_LIMIT_WR, 0);

err:
	if (ret < 0)
		dev_err(&data->client->dev, "LOC sensor init failed 0x%x", ret);

	return ret;
}

static int nct1008_sensors_init(struct nct1008_data *data)
{
	int ret = -ENODEV;
	int ext_err = 0;
	int temp;
	struct nct1008_platform_data *pdata = &data->plat_data;

	if (!pdata)
		goto err;

	/* Configure sensor to trigger alerts clear THERM2_BIT and ALERT_BIT*/
	data->config = 0;

	/* Initially place in Standby */
	ret = nct1008_write_reg(data, CONFIG_WR, STANDBY_BIT);
	if (ret)
		goto err;

	/* Add a delay to make sure it enters into standby mode */
	usleep_range(data->oneshot_conv_period_ns, data->oneshot_conv_period_ns
			+ 1000);

	ret = nct1008_loc_sensor_init(data);
	if (ret < 0)
		goto err;

	ext_err = nct1008_ext_sensor_init(data);

	if (pdata->extended_range)
		data->config |= EXTENDED_RANGE_BIT;

	ret = nct1008_write_reg(data, CONFIG_WR, data->config | STANDBY_BIT);
	if (ret)
		goto err;

	/* Temperature conversion rate */
	if (pdata->conv_rate >= 0) {
		ret = nct1008_write_reg(data, CONV_RATE_WR, pdata->conv_rate);
		if (ret)
			goto err;
	}

	/* Initiate one-shot conversion  */
	ret = nct1008_write_reg(data, ONE_SHOT, 0x1);
	if (ret)
		goto err;

	/* Give hardware necessary time to finish conversion */
	usleep_range(data->oneshot_conv_period_ns, data->oneshot_conv_period_ns
			+ 1000);

	/* read initial local temperature */
	ret = nct1008_get_temp_common(LOC, data, &temp);
	if (ret < 0)
		goto err;

	dev_info(&data->client->dev, "initial LOC temp: %d ", temp);
	/* read initial ext temperature */
	if (ext_err == 0) {
		ret = nct1008_get_temp_common(EXT, data, &temp);
		if (ret < 0)
			goto err;

		dev_info(&data->client->dev, "initial EXT temp: %d ", temp);
	}

err:
	if (ret < 0)
		dev_err(&data->client->dev, "sensor init failed 0x%x", ret);

	return ret;
}

static int nct1008_limits_store(struct nct1008_data *data)
{
	int val;
	struct nct1008_platform_data *pdata = &data->plat_data;

	/* Reset current hi/lo limit values with register values */
	val = nct1008_read_reg(data, EXT_TEMP_LO_LIMIT_HI_BYTE_RD);
	if (val < 0)
		goto err;

	data->sensors[EXT].current_lo_limit =
		value_to_temperature(pdata->extended_range, val);
	val = nct1008_read_reg(data, EXT_TEMP_HI_LIMIT_HI_BYTE_RD);
	if (val < 0)
		goto err;

	data->sensors[EXT].current_hi_limit =
		value_to_temperature(pdata->extended_range, val);
	val = nct1008_read_reg(data, LOC_TEMP_LO_LIMIT_RD);
	if (val < 0)
		goto err;

	data->sensors[LOC].current_lo_limit =
		value_to_temperature(pdata->extended_range, val);

	val = nct1008_read_reg(data, LOC_TEMP_HI_LIMIT_RD);
	if (val < 0)
		goto err;

	data->sensors[LOC].current_hi_limit =
		value_to_temperature(pdata->extended_range, val);

err:
	if (val < 0)
		dev_err(&data->client->dev, "limit reg read failed 0x%x", val);

	return val;
}

static int nct1008_configure_sensor(struct nct1008_data *data)
{
	int ret;

	ret = nct1008_sensors_init(data);
	if (ret < 0)
		goto err;

	ret = nct1008_limits_store(data);
	if (ret < 0)
		goto err;

	if (data->chip != MAX6649)
		nct1008_offsets_program(data);

err:
	return ret;
}

static int nct1008_configure_irq(struct nct1008_data *data)
{
	if (data->client->irq < 0)
		return -EINVAL;

	data->workqueue = create_singlethread_workqueue(data->chip_name);
	INIT_WORK(&data->work, nct1008_work_func);
	return request_irq(data->client->irq, nct1008_irq, IRQF_TRIGGER_NONE,
				data->chip_name, data);

}

static int nct1008_dt_parse(struct i2c_client *client,
			struct nct1008_data *data)
{
	struct device_node *np = client->dev.of_node;
	struct device_node *child;
	struct nct1008_platform_data *pdata = &data->plat_data;
	struct gpio_desc *nct1008_gpio = NULL;
	unsigned int proc, index = 0;

	pdata->conv_rate = INT_MIN;
	data->sensors[LOC].shutdown_limit = INT_MIN;
	data->sensors[EXT].shutdown_limit = INT_MIN;
	if (!np) {
		dev_err(&client->dev, "Cannot find the DT node\n");
		return -ENODEV;
	}

	dev_info(&client->dev, "starting parse dt\n");
	if (client->irq == 0) {
		client->irq = -1;
		dev_info(&client->dev, "Missing interrupt prop\n");
	}

	if (of_property_read_u32(np, "conv-rate", &pdata->conv_rate))
		dev_info(&client->dev, "Missing conv-rate prop\n");

	nct1008_gpio = gpiod_get(&client->dev, "temp-alert",  GPIOD_IN);
	if (IS_ERR(nct1008_gpio))
		dev_info(&client->dev, "Missing temp-alert-gpio prop\n");

	dev_dbg(&client->dev, "gpio:%d irq:%d\n", desc_to_gpio(nct1008_gpio),
			gpiod_to_irq(nct1008_gpio));

	/* optional properties */
	/* Keep property with typo for backward compatibility */
	if (!of_property_read_u32(np, "extended-rage", &proc) ||
		!of_property_read_u32(np, "extended-range", &proc))
		pdata->extended_range = (proc) ? true : false;

	pdata->alpha = 10000;
	pdata->beta = 0;
	pdata->offset = 0;
	pdata->fuse_offset = false;
	if (!of_property_read_u32(np, "alpha", &proc))
		pdata->alpha = proc;

	if (!of_property_read_u32(np, "beta", &proc))
		pdata->beta = proc;

	if (!of_property_read_u32(np, "offset", &proc))
		/* offset resolution is 0.25C resolution, TMP451 uses 0.0625C*/
		pdata->offset = proc * 4;
	else if (!of_property_read_u32(np, "offset-hi-res", &proc))
		/* high resolution offset 0.0625C*/
		pdata->offset = proc;
	else if (of_property_read_bool(np, "support-fuse-offset"))
		/* offset present in fuses */
		pdata->fuse_offset = true;
	else
		dev_info(&client->dev, "programming offset of 0C\n");

	/*
	 * This a legacy property that is incorrect because it assumes
	 * the first subnode will be the LOC sensor. This has been
	 * deprecated in favor of the new *-shutdown-limit properties
	 * but is being kept to maintain backward compatibility.
	 */
	for_each_child_of_node(np, child) {
		if (!of_property_read_u32(child, "shutdown-limit", &proc))
			data->sensors[index].shutdown_limit = proc;

		index++;
	}

	if (index)
		dev_info(&client->dev, "!!!Found deprecated property!!!\n");

	if (!of_property_read_u32(np, "loc-shutdown-limit", &proc))
		data->sensors[LOC].shutdown_limit = proc;

	if (!of_property_read_u32(np, "ext-shutdown-limit", &proc))
		data->sensors[EXT].shutdown_limit = proc;

	dev_info(&client->dev, "success parsing dt\n");
	client->dev.platform_data = pdata;
	return 0;
}

static struct thermal_zone_of_device_ops loc_sops = {
	.get_temp = nct1008_loc_get_temp_as_sensor,
	.get_trend = nct1008_loc_get_trend_as_sensor,
	.set_trips = nct1008_loc_set_trips,
};

static struct thermal_zone_of_device_ops ext_sops = {
	.get_temp = nct1008_ext_get_temp_as_sensor,
	.get_trend = nct1008_ext_get_trend_as_sensor,
	.set_trips = nct1008_ext_set_trips,
};

static void nct1008_thermal_init(struct nct1008_data *data)
{
	struct thermal_zone_device *tzd;
	struct thermal_cooling_device *cdev;
	struct device *dev = &data->client->dev;

	/* Config for the Local sensor. */
	tzd = thermal_zone_of_sensor_register(dev, LOC, data, &loc_sops);
	if (!IS_ERR_OR_NULL(tzd))
		data->sensors[LOC].thz = tzd;

	/* register External sensor if connection is good  */
	tzd = thermal_zone_of_sensor_register(dev, EXT, data, &ext_sops);
	if (!IS_ERR_OR_NULL(tzd))
		data->sensors[EXT].thz = tzd;

	cdev = thermal_of_cooling_device_register(dev->of_node,
					"shutdown_warning", data,
					&nct1008_shutdown_warning_ops);
	if (IS_ERR_OR_NULL(cdev))
		dev_err(dev, "cdev registration failed %ld\n", PTR_ERR(cdev));
}

/*
 * Manufacturer(OnSemi) recommended sequence for
 * Extended Range mode is as follows
 * nct1008_configure_sensor:
 * 1. Place in Standby
 * 2. Scale the THERM and ALERT limits
 *	appropriately(for Extended Range mode).
 * 3. Enable Extended Range mode.
 *	ALERT mask/THERM2 mode may be done here
 *	as these are not critical
 * 4. Set Conversion Rate as required
 * nct1008_enable:
 * 5. Take device out of Standby
 */

/*
 * function nct1008_probe takes care of initial configuration
 */
static int nct1008_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	int err;
	struct nct1008_data *data;
	if (!client->dev.of_node) {
		dev_err(&client->dev, "missing device tree node\n");
		return -EINVAL;
	}

	dev_dbg(&client->dev, "find device tree node, parsing dt\n");
	data = devm_kzalloc(&client->dev, sizeof(struct nct1008_data),
				GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	err = nct1008_dt_parse(client, data);
	if (err)
		return err;

	data->client = client;
	data->chip = id->driver_data;
	strlcpy(data->chip_name, id->name, I2C_NAME_SIZE);
	i2c_set_clientdata(client, data);
	mutex_init(&data->mutex);
	data->nct_reg = regulator_get(&client->dev, "vdd");
	if (IS_ERR(data->nct_reg)) {
		dev_err(&client->dev, "vdd regulator get failed: 0x%x\n", err);
		mutex_destroy(&data->mutex);
		return -ENODEV;
	}

	/* oneshot conversion time */
	if (data->chip == TMP451)
		data->oneshot_conv_period_ns = TMP451_CONV_TIME_ONESHOT_US;
	else
		data->oneshot_conv_period_ns = NCT_CONV_TIME_ONESHOT_US;

	nct1008_power_control(data, true);
	/* sensor is in standby */
	err = nct1008_configure_sensor(data);
	if (err < 0)
		goto error;

	err = nct1008_configure_irq(data);
	if (err < 0)
		dev_info(&client->dev, "irq config failed: 0x%x ", err);

	/* sensor is running */
	err = nct1008_enable(client);
	if (err < 0)
		goto error;

	/* register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &nct1008_attr_group);
	if (err < 0) {
		dev_err(&client->dev, "\n sysfs create err=%d ", err);
		goto error;
	}

	dev_info(&client->dev, "%s: initialized\n", __func__);
	nct1008_thermal_init(data);
	return 0;
error:
	dev_err(&client->dev, "\n exit %s, err=%d ", __func__, err);
	mutex_destroy(&data->mutex);
	nct1008_power_control(data, false);
	return err;
}

static int nct1008_remove(struct i2c_client *client)
{
	struct nct1008_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->mutex);
	data->stop_workqueue = 1;
	mutex_unlock(&data->mutex);

	cancel_work_sync(&data->work);
	free_irq(data->client->irq, data);
	sysfs_remove_group(&client->dev.kobj, &nct1008_attr_group);
	nct1008_power_control(data, false);

	if (data->nct_reg)
		regulator_put(data->nct_reg);

	mutex_destroy(&data->mutex);

	return 0;
}

static void nct1008_shutdown(struct i2c_client *client)
{
	struct nct1008_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->mutex);
	data->stop_workqueue = 1;
	mutex_unlock(&data->mutex);

	cancel_work_sync(&data->work);

	if (client->irq)
		disable_irq(client->irq);

	if (data->sensors[LOC].thz) {
		if (client->dev.of_node)
			thermal_zone_of_sensor_unregister
				(&(client->dev), data->sensors[LOC].thz);
		else
			thermal_zone_device_unregister(data->sensors[LOC].thz);
		data->sensors[LOC].thz = NULL;
	}
	if (data->sensors[EXT].thz) {
		if (client->dev.of_node)
			thermal_zone_of_sensor_unregister
				(&(client->dev), data->sensors[EXT].thz);
		else
			thermal_zone_device_unregister(data->sensors[EXT].thz);
		data->sensors[EXT].thz = NULL;
	}

	mutex_lock(&data->mutex);
	data->nct_disabled = 1;
	mutex_unlock(&data->mutex);
}

static int nct1008_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	int err;
	struct nct1008_data *data = i2c_get_clientdata(client);

	mutex_lock(&data->mutex);
	data->stop_workqueue = 1;
	mutex_unlock(&data->mutex);
	cancel_work_sync(&data->work);
	disable_irq(client->irq);
	err = nct1008_disable(client);
	nct1008_power_control(data, false);

	return err;
}

static int nct1008_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	int err;
	struct nct1008_data *data = i2c_get_clientdata(client);

	nct1008_power_control(data, true);
	err = nct1008_configure_sensor(data);
	if (err < 0)
		return err;

	err = nct1008_enable(client);
	if (err < 0)
		return err;

	mutex_lock(&data->mutex);
	data->stop_workqueue = 0;
	mutex_unlock(&data->mutex);
	enable_irq(client->irq);

	return 0;
}

static const struct dev_pm_ops nct1008_pm_ops = {
	.suspend	= nct1008_suspend,
	.resume		= nct1008_resume,
};

static const struct i2c_device_id nct1008_id[] = {
	{ "nct1008", NCT1008 },
	{ "nct72", NCT72 },
	{ "tmp451", TMP451 },
	{ "max6649", MAX6649 },
	{}
};
MODULE_DEVICE_TABLE(i2c, nct1008_id);

static const struct of_device_id nct1008_of_match[] = {
	{.compatible = "onsemi,nct72", },
	{.compatible = "ti,tmp451", },
	{.compatible = "maxim,max6649", },
	{ }
};

static struct i2c_driver nct1008_driver = {
	.driver = {
		.name	= "nct1008_nct72",
		.pm = &nct1008_pm_ops,
		.of_match_table = nct1008_of_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe		= nct1008_probe,
	.remove		= nct1008_remove,
	.id_table	= nct1008_id,
	.shutdown	= nct1008_shutdown,
};

module_i2c_driver(nct1008_driver);
MODULE_AUTHOR("Srikar Srimath Tirumala <srikars@nvidia.com>");
MODULE_DESCRIPTION("Temperature sensor driver for NCT1008/NCT72/TMP451");
MODULE_LICENSE("GPL");
