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
/* The NVS = NVidia Sensor framework */
/* This common NVS on-change module allows, along with the NVS KIF (Kernel
 * InterFace) common module, an on-change driver to offload the code
 * interacting with the kernel and handling the on-change reporting specifics,
 * and just have code that interacts with the HW.
 * An on-change sensor is defined by the Android sensor specification. This
 * module adheres to that specification with regard to operation and behavior,
 * but also adds a superset of features.
 * The commonality between this module and the NVS HW driver is the
 * nvs_on_change structure.  It is expected that the NVS on-change driver will:
 * - call nvs_on_change_enable when the device is enabled for initialization.
 * - read the HW and place the value in nvs_on_change.hw
 * - call nvs_on_change_read
 * - depending on the nvs_on_change_read return value:
 *     - -1 = poll HW using nvs_on_change.poll_period_ms period.
 *     - 0 = if interrupt driven, do nothing or resume regular polling
 *     - 1 = set new thresholds using the nvs_on_change.hw_thresh_lo/hi
 * Reporting the on-change value is handled within this module.
 * See nvs_on_change.h for nvs_on_change structure details.
 */
/* NVS on-change drivers can be configured for binary output. If the max_range
 * and resolution settings in the device tree is set for 1.0, the driver
 * will configure the rest of the settings for binary reporting. For example,
 * the Android specification allows proximity to be reported as 1 for
 * "far away" and 0 for "near".
 * If on-change binary output is disabled, the driver can use a number of NVS
 * mechanisms to send real-world values:
 * - scale_float_en
 * - calibration
 * - and just plain 'ol raw data
 * See nvs_on_change.h for nvs_on_change features.
 */
/* When using the threshold sysfs attributes, keep in mind that all threshold
 * values are in HW units. This allows the thresholds to automatically scale
 * due to any parameter modifications such as resolution.
 */
/* If the NVS on-change driver is not configured for binary output, then
 * there are few mechanisms for reporting float data:
 * Method 1
 * This is required if the driver is using dynamic resolution since the
 * resolution cannot be read by the NVS HAL on every data value read due to
 * buffering.  Instead, this mechanism allows floating point to be calculated
 * here in the kernel by multiplying up to an integer the floating point's
 * significant amount. The scale.fval must be a 10 base value,
 * e.g. 0.1, 0.01, ... 0.000001, etc. as the significant amount. The NVS HAL
 * will then convert the value to float by multiplying the integer float-data
 * with scale. To enable this feature, set the scale_float_en member in the
 * nvs_on_change structure. The following will be needed for calibration.
 * Method 2
 * The NVS HAL will use the scale and offset sysfs attributes to modify the
 * data using the following formula: (data * scale) + offset
 * A scale value of 0 disables scale.
 * A scale value of 1 puts the NVS HAL into calibration mode where the scale
 * and offset are read everytime the data is read to allow realtime calibration
 * of the scale and offset values to be used in the device tree parameters.
 * Keep in mind the data is buffered but the NVS HAL will display the data and
 * scale/offset parameters in the log.  See calibration steps below if needed.
 * Method 3
 * This method uses interpolation and requires a low and high uncalibrated
 * value along with the corresponding low and high calibrated values.  The
 * uncalibrated values are what is read from the sensor in the steps below.
 * The corresponding calibrated values are what the correct value should be.
 * All values are programmed into the device tree settings.
 * Calibration steps:
 * 1. Read scale sysfs attribute.  This value will need to be written back.
 * 2. Disable device.
 * 3. Write 1 to the scale sysfs attribute.
 * 4. Enable device.
 * 5. The NVS HAL will announce in the log that calibration mode is enabled and
 *    display the data along with the scale and offset parameters applied.
 * 6. Write the scale value read in step 1 back to the scale sysfs attribute.
 * 7. Put the device into a state where the data read is a low value.
 * 8. Note the values displayed in the log.  Separately measure the actual
 *    value.  The value from the sensor will be the uncalibrated value and the
 *    separately measured value will be the calibrated value for the current
 *    state (low or high values).
 * 9. Put the device into a state where the data read is a high value.
 * 10. Repeat step 8.
 * 11. Enter the values in the device tree settings for the device.  Both
 *     calibrated and uncalibrated values will be the values before scale and
 *     offset are applied.
 *     The on-change sensor has the following device tree parameters for this:
 *     on_change_uncalibrated_lo
 *     on_change_calibrated_lo
 *     on_change_uncalibrated_hi
 *     on_change_calibrated_hi
 *
 * An NVS on-change driver may support a simplified version of method 2 that
 * can be used in realtime:
 * At step 8, write the calibrated value to the in_on_change_threshold_low
 * attribute.  When in calibration mode this value will be written to the
 * on_change_calibrated_lo and the current on_change written to
 * on_change_uncalibrated_lo internal to the driver.
 * If this is after step 9, then use the in_on_change_threshold_high
 * attribute.
 * Note that the calibrated value must be the value before the scale and offset
 * is applied.  For example, if the calibrated on_change reading is 123.4 cm,
 * and the in_on_change_scale is normally 0.01, then the value entered is 12340
 * which will be 123.4 cm when the scale is applied at the HAL layer.
 * To confirm the realtime values and see what the driver used for uncalibrated
 * values, do the following at the adb prompt in the driver space:
 * # echo 5 > nvs
 * # cat nvs
 * This will be a partial dump of the sensor's configuration structure that
 * will show the calibrated and uncalibrated values.  For example:
 * ...
 * uncal_lo=1
 * uncal_hi=96346
 * cal_lo=230
 * cal_hi=1888000
 * thresh_lo=10
 * thresh_hi=10
 * ...
 * If the thresholds have changed instead of the calibration settings, then
 * the driver doesn't support this feature.
 * In order to display raw values, interpolation, that uses the calibration
 * values, is not executed by the driver when in calibration mode, so to test,
 * disable and reenable the device to exit calibration mode and test the new
 * calibration values.
 *
 * Method 2 can only be used if dynamic resolution is not used by the HW
 * driver.  The data passed up to the HAL is the HW value read so that the HAL
 * can multiply the HW value with the scale (resolution).
 * As a baseline, scale would be the same value as the static resolution.
 * Method 2:
 * 1. Disable device.
 * 2. Write 1 to the scale sysfs attribute.
 * 3. Enable device.
 * 4. The NVS HAL will announce in the log that calibration mode is enabled and
 *    display the data along with the scale and offset parameters applied.
 * 5. Write to scale and offset sysfs attributes as needed to get the data
 *    modified as desired.
 * 6. Disabling the device disables calibration mode.
 * 7. Set the new scale and offset parameters in the device tree:
 *    on_change_scale_ival = the integer value of the scale.
 *    on_change_scale_fval = the floating value of the scale.
 *    on_change_offset_ival = the integer value of the offset.
 *    on_change_offset_fval = the floating value of the offset.
 *    The values are in the NVS_FLOAT_SIGNIFICANCE_ format (see nvs.h).
 */
/* If the NVS on-change driver is configured for binary output, then
 * interpolation is not used and the thresholds are used to trigger either the
 * 0 or 1 output. The following is an example of how to calibrate a proximity
 * sensor using the thresholds:
 * 1. Disable device.
 * 2. Write 1 to the scale sysfs attribute.
 * 3. Enable device.
 * 4. The NVS HAL will announce in the log that calibration mode is enabled and
 *    display the HW on-change data.
 * 5. Move an object (your hand) through the on-change range.  Note the HW
 *    value when the object is at a point that the output should be 0.  This
 *    will be the high threshold value.  Move the object away from the sensor
 *    and note the HW value where the output should change to 1.  This will be
 *    the low threshold value.
 * NOTE: Proximity typically works by reading the reflected IR light from an
 *       LED.  The more light reflected, the higher the HW value and the closer
 *       the object is.  Because of this, the thresholds appear to be reversed
 *       to the output, but keep in mind, the thresholds are HW based, so low
 *       threshold means low HW value regardless of the actual output.
 * NOTE: If greater range is needed, modify the LED output strength if the
 *       on-change HW supports it.  This will be a DT configuration option that
 *       is specific to the driver and HW.
 * 6. Enter the threshold values in the device tree settings for the device.
 *     The on-change sensor has the following device tree parameters for this:
 *     on_change_threshold_lo
 *     on_change_threshold_hi
 * 7. Be sure to add the device tree attribute:
 *    <sensor_name>_thresholds_absolute_en = <1>;
 *    This will configure the thresholds to use absolute values,
 *    (OC_THRESHOLDS_ABSOLUTE).
 */
/* If the NVS on-change driver is not configured for OC_THRESHOLDS_ABSOLUTE,
 * then the thresholds are used for hysterysis.  The threshold settings are HW
 * based and allow a window around the last reported on_change HW value.  For
 * example, if the low threshold is set to 10 and the high threshold set to 20,
 * if the on_change HW value is 100, the on_change won't be reported again
 * until the on_change HW value is either < 90 or > than 120.
 * The low/high threshold values are typically the same, but they can be
 * configured so that on_change changes at a different rate based on the
 * direction of change.
 * Use the calibration methods for a steady output of data to get an idea of
 * the debounce desired.
 * NOTE: If both configuration thresholds are 0, then thresholds are disabled.
 * NOTE: An NVS feature is the use of the report_count configuration variable,
 *       <sensor_name>_report_count in DT (see nvs.h).  This allows additional
 *       reporting of on_change a set amount of times while still within the
 *       threshold window.
 */
/* If the NVS on-change driver is configured for binary output, then the
 * thresholds are absolute HW values.  If not configured for binary output,
 * then the thresholds are relative HW values to set a trigger window around
 * the last read on_change HW value.
 */


#include <linux/module.h>
#include <linux/of.h>
#include <linux/version.h>
#include <linux/nvs_on_change.h>

#define NVS_ON_CHANGE_VERSION		(1)
#define NVS_FS_NANO			NVS_FLOAT_SIGNIFICANCE_NANO
#define NVS_FS_MICRO			NVS_FLOAT_SIGNIFICANCE_MICRO


ssize_t nvs_on_change_dbg(struct nvs_on_change *oc, char *buf, size_t size)
{
	ssize_t t;

	t = snprintf(buf, size, "NVS ON-CHANGE v.%u\n", NVS_ON_CHANGE_VERSION);
	t += snprintf(buf + t, size - t, "timestamp=%lld\n",
		      oc->timestamp);
	t += snprintf(buf + t, size - t, "timestamp_report=%lld\n",
		      oc->timestamp_report);
	t += snprintf(buf + t, size - t, "on_change=%u\n", oc->on_change);
	t += snprintf(buf + t, size - t, "hw=%u\n", oc->hw);
	t += snprintf(buf + t, size - t, "hw_min=%x\n", oc->hw_min);
	t += snprintf(buf + t, size - t, "hw_max=%x\n", oc->hw_max);
	t += snprintf(buf + t, size - t, "hw_thresh_lo=%u\n",
		      oc->hw_thresh_lo);
	t += snprintf(buf + t, size - t, "hw_thresh_hi=%u\n",
		      oc->hw_thresh_hi);
	t += snprintf(buf + t, size - t, "hw_limit_lo=%x\n",
		      oc->hw_limit_lo);
	t += snprintf(buf + t, size - t, "hw_limit_hi=%x\n",
		      oc->hw_limit_hi);
	t += snprintf(buf + t, size - t, "thresh_valid_lo=%x\n",
		      oc->thresh_valid_lo);
	t += snprintf(buf + t, size - t, "thresh_valid_hi=%x\n",
		      oc->thresh_valid_hi);
	t += snprintf(buf + t, size - t, "reverse_range_en=%x\n",
		      oc->reverse_range_en);
	t += snprintf(buf + t, size - t, "scale_float_en=%x\n",
		      oc->scale_float_en);
	t += snprintf(buf + t, size - t, "binary_report_en=%x\n",
		      oc->binary_report_en);
	if (oc->threshold < 0)
		t += snprintf(buf + t, size - t, "OC_THRESHOLDS_DISABLE\n");
	else if (oc->threshold > 0)
		t += snprintf(buf + t, size - t, "OC_THRESHOLDS_ABSOLUTE\n");
	else
		t += snprintf(buf + t, size - t, "OC_THRESHOLDS_RELATIVE\n");
	if (oc->calibration < 0)
		t += snprintf(buf + t, size - t, "OC_CALIBRATION_DISABLE\n");
	else if (oc->calibration > 0)
		t += snprintf(buf + t, size - t, "OC_CALIBRATION_ENABLE\n");
	else
		t += snprintf(buf + t, size - t, "OC_CALIBRATION_DONE\n");
	t += snprintf(buf + t, size - t, "poll_period_ms=%u\n",
		      oc->poll_period_ms);
	t += snprintf(buf + t, size - t, "period_us=%u\n", oc->period_us);
	t += snprintf(buf + t, size - t, "report_n=%u\n", oc->report_n);
	return t;
}
EXPORT_SYMBOL_GPL(nvs_on_change_dbg);

ssize_t nvs_on_change_dbg_dt(struct nvs_on_change *oc, char *buf, size_t size,
			     const char *dev_name)
{
	char str[256];
	int ret;
	ssize_t t = 0;

	if (buf == NULL)
		return -EINVAL;

	if (dev_name == NULL)
		dev_name = oc->cfg->name;
	ret = snprintf(str, sizeof(str), "%s_thresholds_absolute_en",
		       dev_name);
	if (ret > 0) {
		t += snprintf(buf + t, size - t, "%s=%d\n",
			      str, oc->threshold);
	}
	return t;
}
EXPORT_SYMBOL_GPL(nvs_on_change_dbg_dt);

/**
 * nvs_on_change_of_dt - called during system boot for
 * configuration from device tree.
 * @oc: the common structure between driver and common module.
 * @dn: device node pointer.
 * @dev_name: device name string.  Typically a string to the
 *            Android sensor name.
 *
 * Returns 0 on success or a negative error code.
 *
 * Driver must initialize variables if no success.
 */
int nvs_on_change_of_dt(struct nvs_on_change *oc, const struct device_node *dn,
			const char *dev_name)
{
	s32 val;
	char str[256];
	int ret;

	if (oc->cfg)
		oc->cfg->flags |= SENSOR_FLAG_ON_CHANGE_MODE;
	if (dn == NULL)
		return -EINVAL;

	val = oc->cfg->thresh_lo;
	val |= oc->cfg->thresh_hi;
	/* val > 0  ==> 0   (OC_THRESHOLDS_RELATIVE)
	 * val == 0 ==> -1  (OC_THRESHOLDS_DISABLE)
	 * val < 0  ==> 0   (OC_THRESHOLDS_RELATIVE)
	 */
	val = !!!val;
	val = 0 - val;
	if (dev_name == NULL)
		dev_name = oc->cfg->name;
	ret = snprintf(str, sizeof(str), "%s_thresholds_absolute_en", dev_name);
	if (ret > 0)
		of_property_read_s32(dn, str, &val);
	if (val < 0)
		oc->threshold = OC_THRESHOLDS_DISABLE;
	else if (val > 0)
		oc->threshold = OC_THRESHOLDS_ABSOLUTE;
	else
		oc->threshold = OC_THRESHOLDS_RELATIVE;
	return 0;
}
EXPORT_SYMBOL_GPL(nvs_on_change_of_dt);

static void nvs_on_change_interpolate(int x1, s64 x2, int x3,
				      int y1, u32 *y2, int y3)
{
	s64 dividend;
	s64 divisor;

	/* y2 = ((x2 - x1)(y3 - y1)/(x3 - x1)) + y1 */
	divisor = (x3 - x1);
	if (!divisor) {
		*y2 = (u32)x2;
		return;
	}

	dividend = (x2 - x1) * (y3 - y1);
	if (dividend < 0) {
		dividend = abs(dividend);
		do_div(dividend, divisor);
		dividend = 0 - dividend;
	} else {
		do_div(dividend, divisor);
	}
	dividend += y1;
	if (dividend < 0)
		dividend = 0;
	*y2 = (u32)dividend;
}

static int nvs_on_change_poll_period(struct nvs_on_change *oc, int ret,
				     unsigned int poll_period,
				     bool report_period_min)
{
	if (report_period_min)
		poll_period = oc->period_us;
	if ((poll_period < oc->cfg->delay_us_min) || (oc->calibration ==
						      OC_CALIBRATION_ENABLE))
		poll_period = oc->cfg->delay_us_min;
	oc->poll_period_ms = poll_period / 1000;
	if (oc->report_n || oc->calibration == OC_CALIBRATION_ENABLE)
		ret = RET_POLL_NEXT; /* poll for next sample */
	return ret;
}

static int nvs_on_change_push(struct nvs_on_change *oc, s32 on_change,
			      unsigned int poll_period, bool report_period_min)
{
	int ret = RET_NO_CHANGE;

	if (oc->report_n && report_period_min) {
		oc->report_n--;
		oc->timestamp_report = oc->timestamp;
		oc->on_change = on_change;
		oc->handler(oc->nvs_st, &oc->on_change,
			    oc->timestamp_report);
		ret = RET_HW_UPDATE;
	}
	return nvs_on_change_poll_period(oc, ret, poll_period,
					 report_period_min);
}

static int nvs_on_change_calc(struct nvs_on_change *oc, s64 *on_change)
{
	s64 calc;
	u64 calc_i;
	u64 calc_f;

	if (oc->reverse_range_en)
		/* reverse the value in the range */
		calc = oc->hw_max - oc->hw;
	else
		calc = oc->hw;
	if (oc->scale_float_en && oc->cfg->scale.fval) {
		/* The mechanism below allows floating point to
		 * be calculated here in the kernel by shifting
		 * up to integer the floating point significant
		 * amount.
		 * The oc->cfg->scale.fval must be a 10 base
		 * value, e.g. 0.1, 0.01, ... 0.000001, etc.
		 * The significance is calculated as:
		 * s = (NVS_FLOAT_SIGNIFICANCE_* / scale.fval)
		 * so that on_change = HW * resolution * s
		 * The NVS HAL will then convert the value to
		 * float by multiplying the data with scale.
		 *
		 * calc = HW * (resolution * NVS_FLOAT_SIGNIFICANCE_) / scale
		 */
		calc_i = calc;
		calc_f = 0;
		if (oc->cfg->resolution.fval) {
			calc_f = calc * oc->cfg->resolution.fval;
			do_div(calc_f, oc->cfg->scale.fval);
		}
		if (oc->cfg->resolution.ival) {
			if (oc->cfg->float_significance)
				calc_i = NVS_FS_NANO;
			else
				calc_i = NVS_FS_MICRO;
			do_div(calc_i, oc->cfg->scale.fval);
			calc_i *= calc * oc->cfg->resolution.ival;
		}
		calc = (s64)(calc_i + calc_f);
	}
	if (oc->calibration != OC_CALIBRATION_DISABLE) {
		if (oc->calibration == OC_CALIBRATION_DONE)
			/* get calibrated value */
			nvs_on_change_interpolate(oc->cfg->uncal_lo,
						  calc,
						  oc->cfg->uncal_hi,
						  oc->cfg->cal_lo,
						  &oc->on_change,
						  oc->cfg->cal_hi);
		else /* (oc->calibration == OC_CALIBRATION_ENABLE) */
			/* when in calibration mode just return calc */
			oc->on_change = (s32)calc;
	}
	*on_change = calc;
	return 0;
}


/**
 * nvs_on_change_read - called after HW is read and written to oc.
 * @oc: the common structure between driver and common module.
 *
 * This will handle the conversion of HW to on-change value,
 * reporting, calculation of thresholds and poll time.
 *
 * Returns: -1 = Error and/or polling is required for next
 *               sample regardless of being interrupt driven.
 *          0 = Do nothing.  Value has not changed for reporting
 *              and same threshold values if interrupt driven.
 *              If not interrupt driven use poll_period_ms.
 *          1 = New HW thresholds are needed.
 *              If not interrupt driven use poll_period_ms.
 */
int nvs_on_change_read(struct nvs_on_change *oc)
{
	s32 thresh;
	s64 on_change;
	s64 timestamp_diff;
	s64 period;
	bool report_period_min = true;
	unsigned int poll_period = 0;
	int ret;

	if (oc->calibration == OC_CALIBRATION_ENABLE)
		/* always report without report_period_min */
		oc->report_n = oc->cfg->report_n;
	if (oc->report_n < oc->cfg->report_n) { /* always report 1st sample */
		/* calculate elapsed time for allowed report rate */
		timestamp_diff = oc->timestamp - oc->timestamp_report;
		period = (s64)oc->period_us * 1000;
		if (timestamp_diff < period) {
			/* data changes are happening faster than allowed to
			 * report so we poll for the next data at an allowed
			 * rate with interrupts disabled.
			 */
			period -= timestamp_diff;
			do_div(period, 1000); /* ns => us */
			poll_period = period;
			report_period_min = false;
		}
	}
	if (oc->threshold == OC_THRESHOLDS_DISABLE) {
		nvs_on_change_calc(oc, &on_change);
		if (on_change != oc->on_change)
			oc->report_n = oc->cfg->report_n;
		ret = nvs_on_change_push(oc, on_change, poll_period,
					 report_period_min);
		return ret;
	}

	if (oc->threshold == OC_THRESHOLDS_ABSOLUTE) {
		if (oc->thresh_valid_lo && (oc->hw <= oc->hw_thresh_lo)) {
			on_change = 0; /* assume binary_report_en */
			oc->report_n = oc->cfg->report_n;
			/* disable lower threshold */
			oc->thresh_valid_lo = false;
			oc->hw_limit_lo = true;
			/* enable upper threshold */
			oc->thresh_valid_hi = true;
			oc->hw_limit_hi = false;
		} else if (oc->thresh_valid_hi && (oc->hw >=
						   oc->hw_thresh_hi)) {
			on_change = 1; /* assume binary_report_en */
			oc->report_n = oc->cfg->report_n;
			/* enable lower threshold */
			oc->thresh_valid_lo = true;
			oc->hw_limit_lo = false;
			/* disable upper threshold */
			oc->thresh_valid_hi = false;
			oc->hw_limit_hi = true;
		}
		if (oc->binary_report_en) {
			if (oc->reverse_range_en)
				/* reverse the value in the range */
				on_change = !on_change;
			if (oc->calibration == OC_CALIBRATION_ENABLE)
				on_change = oc->hw;
		} else if (oc->report_n) {
			nvs_on_change_calc(oc, &on_change);
		}
		ret = nvs_on_change_push(oc, on_change, poll_period,
					 report_period_min);
		return ret;
	}

	/* (oc->threshold == OC_THRESHOLDS_RELATIVE) */
	ret = RET_NO_CHANGE;
	/* thresholds */
	if (oc->thresh_valid_lo && (oc->hw <= oc->hw_thresh_lo))
		oc->report_n = oc->cfg->report_n;
	else if (oc->thresh_valid_hi && (oc->hw > oc->hw_thresh_hi))
		oc->report_n = oc->cfg->report_n;
	/* reporting */
	if (oc->report_n && report_period_min) {
		oc->report_n--;
		oc->timestamp_report = oc->timestamp;
		nvs_on_change_calc(oc, &on_change);
		oc->on_change = on_change;
		/* report on_change */
		oc->handler(oc->nvs_st, &oc->on_change, oc->timestamp_report);
		if (!oc->report_n) {
			/* calculate low threshold */
			thresh = oc->hw - oc->cfg->thresh_lo;
			if (thresh < oc->hw_min) {
				/* low threshold is disabled */
				oc->hw_thresh_lo = oc->hw_min;
				oc->thresh_valid_lo = false;
				oc->hw_limit_lo = true;
			} else {
				oc->hw_thresh_lo = thresh;
				oc->thresh_valid_lo = true;
				oc->hw_limit_lo = false;
			}
			/* calculate high threshold */
			thresh = oc->hw + oc->cfg->thresh_hi;
			if (thresh > oc->hw_max) {
				/* high threshold is disabled */
				oc->hw_thresh_hi = oc->hw_max;
				oc->thresh_valid_hi = false;
				oc->hw_limit_hi = true;
			} else {
				oc->hw_thresh_hi = thresh;
				oc->thresh_valid_hi = true;
				oc->hw_limit_hi = false;
			}
			ret = RET_HW_UPDATE;
		}
	}
	ret = nvs_on_change_poll_period(oc, ret, poll_period,
					report_period_min);
	return ret;
}
EXPORT_SYMBOL_GPL(nvs_on_change_read);

/**
 * nvs_on_change_enable - called when the on_change sensor is enabled.
 * @oc: the common structure between driver and common module.
 *
 * This inititializes the oc NVS variables.
 *
 * Returns 0 on success or a negative error code.
 */
int nvs_on_change_enable(struct nvs_on_change *oc)
{
	int val;
	int ret = 0;

	if (!oc->cfg->report_n)
		oc->cfg->report_n = 1;
	oc->report_n = oc->cfg->report_n;
	oc->timestamp_report = 0;
	oc->on_change = 1;
	/* determine calibration mode */
	if ((oc->cfg->scale.ival == 1) && (oc->cfg->scale.fval == 0)) {
		oc->calibration = OC_CALIBRATION_ENABLE;
	} else {
		val = oc->cfg->uncal_lo;
		val |= oc->cfg->uncal_hi;
		val |= oc->cfg->cal_lo;
		val |= oc->cfg->cal_hi;
		if (val)
			oc->calibration = OC_CALIBRATION_DONE;
		else
			oc->calibration = OC_CALIBRATION_DISABLE;
	}
	/* determine binary_report_en */
	if ((oc->cfg->resolution.ival == 1) &&
					     (oc->cfg->resolution.fval == 0) &&
					      (oc->cfg->max_range.ival == 1) &&
					      (oc->cfg->max_range.fval == 0)) {
		oc->binary_report_en = true;
		oc->threshold = OC_THRESHOLDS_ABSOLUTE;
		oc->hw_thresh_hi = oc->cfg->thresh_hi;
		oc->hw_thresh_lo = oc->cfg->thresh_lo;
		oc->thresh_valid_lo = true;
		oc->thresh_valid_hi = true;
	} else {
		oc->binary_report_en = false;
		/* determine thresholds */
		oc->hw_limit_lo = false;
		oc->hw_limit_hi = false;
		val = oc->cfg->thresh_lo;
		val |= oc->cfg->thresh_hi;
		if (val) {
			if (oc->threshold == OC_THRESHOLDS_ABSOLUTE) {
				oc->hw_thresh_hi = oc->cfg->thresh_hi;
				oc->hw_thresh_lo = oc->cfg->thresh_lo;
			} else { /* oc->threshold == OC_THRESHOLDS_RELATIVE */
				oc->threshold = OC_THRESHOLDS_RELATIVE;
				oc->hw_thresh_hi = oc->hw_min;
				oc->hw_thresh_lo = oc->hw_max;
			}
			oc->thresh_valid_lo = true;
			oc->thresh_valid_hi = true;
		} else { /* oc->threshold == OC_THRESHOLDS_DISABLE */
			oc->threshold = OC_THRESHOLDS_DISABLE;
			oc->thresh_valid_lo = false;
			oc->thresh_valid_hi = false;
		}
	}
	if (oc->period_us)
		oc->poll_period_ms = oc->period_us * 1000;
	else
		oc->poll_period_ms = oc->cfg->delay_us_min * 1000;
	if (oc->hw_max <= oc->hw_min)
		ret = -EINVAL;
	return ret;
}
EXPORT_SYMBOL_GPL(nvs_on_change_enable);

/**
 * nvs_on_change_threshold_lo - runtime mechanism to modify low
 * threshold value.
 * @oc: the common structure between driver and common module.
 *
 * NOTE: If in calibration mode then calibrated/uncalibrated low
 * values are modified instead.
 */
int nvs_on_change_threshold_lo(struct nvs_on_change *oc, int lo)
{
	s32 lo_max;

	if (oc->calibration == OC_CALIBRATION_ENABLE) {
		oc->cfg->uncal_lo = oc->on_change;
		oc->cfg->cal_lo = lo;
	} else if (oc->threshold == OC_THRESHOLDS_ABSOLUTE) {
		if (lo > oc->hw_max)
			lo = oc->hw_max;
		else if (lo < oc->hw_min)
			lo = oc->hw_min;
		oc->cfg->thresh_lo = lo;
	} else if (oc->threshold == OC_THRESHOLDS_RELATIVE) {
		lo_max = oc->hw_max - oc->hw_min;
		if (lo > lo_max)
			lo = lo_max;
		else if (lo < 0)
			lo = 0;
		oc->cfg->thresh_lo = lo;
	} else {
		oc->cfg->thresh_lo = lo;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(nvs_on_change_threshold_lo);

/**
 * nvs_on_change_threshold_hi - runtime mechanism to modify high
 * threshold value.
 * @oc: the common structure between driver and common module.
 *
 * NOTE: If in calibration mode then calibrated/uncalibrated
 * high values are modified instead.
 */
int nvs_on_change_threshold_hi(struct nvs_on_change *oc, int hi)
{
	s32 hi_max;

	if (oc->calibration == OC_CALIBRATION_ENABLE) {
		oc->cfg->uncal_hi = oc->on_change;
		oc->cfg->cal_hi = hi;
	} else if (oc->threshold == OC_THRESHOLDS_ABSOLUTE) {
		if (hi > oc->hw_max)
			hi = oc->hw_max;
		else if (hi < oc->hw_min)
			hi = oc->hw_min;
		oc->cfg->thresh_hi = hi;
	} else if (oc->threshold == OC_THRESHOLDS_RELATIVE) {
		hi_max = oc->hw_max - oc->hw_min;
		if (hi > hi_max)
			hi = hi_max;
		else if (hi < 0)
			hi = 0;
		oc->cfg->thresh_hi = hi;
	} else {
		oc->cfg->thresh_hi = hi;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(nvs_on_change_threshold_hi);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("NVidia Sensor on-change module");
MODULE_AUTHOR("NVIDIA Corporation");

