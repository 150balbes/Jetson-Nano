/*
 * include/linux/therm_est.h
 *
 * Copyright (c) 2010-2019, NVIDIA CORPORATION.  All rights reserved.
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

#ifndef _LINUX_THERM_EST_H
#define _LINUX_THERM_EST_H

#include <linux/workqueue.h>
#include <linux/thermal.h>
#include <linux/pwm.h>

#define HIST_LEN (20)

#define MAX_ACTIVE_STATES	10
#define MAX_TIMER_TRIPS		10

#define MAX_SUBDEVICE_GROUP	2

struct therm_est_sub_thz {
	struct thermal_zone_device *thz;
	long hist[HIST_LEN];
};

struct therm_est_coeffs {
	long toffset;
	s32 (*coeffs)[HIST_LEN];
};

struct therm_est_subdevice {
	struct therm_est_sub_thz *sub_thz;
	struct therm_est_coeffs *coeffs_set;
	int num_devs;
	int num_coeffs;
	int active_coeffs;
	int ntemp;
};

/*
 * Timer trip provides a way to change trip temp dynamically based on timestamp
 * when the trip is enabled.
 * - Timer trip can be various numbers on a trip.
 * - If the trip is enabled, then timer will be started with time_after delay
 *   in the corresponding timer trip. After the timer expires, trip_temp and
 *   hysteresis in the corresponding timer trip will be used to trip_temp for
 *   the trip.
 * - When the timer has expired, index of timer trip will be increased a step
 *   and then start the timer with time_after delay in newly indexed timer trip.
 * - When temp is below trip temp, index of timer trip will be decreased a step
 *   and then stop the timer and start the timer with time_after delay in newly
 *   indexed timer trip.
 * - The timer will be stopped if there is no more next timer trip on the trip,
 *   or the trip is disabled.
 */
struct therm_est_timer_trip {
	long time_after; /* in msec */
	long trip_temp;
	long hysteresis;
};

struct therm_est_timer_trip_info {
	int trip; /* trip point on thermal zone to apply timer trip */
	int num_timers;
	struct therm_est_timer_trip timers[MAX_TIMER_TRIPS];
	int cur; /* index of current timer trip */
	s64 last_tripped; /* timestamp when the trip is enabled, in msec */
};

struct therm_est_data {
	/* trip point info */
	int num_trips;
	struct thermal_trip_info *trips;

	/* timer trip info */
	int num_timer_trips;
	struct therm_est_timer_trip_info *timer_trips;

	/* zone parameters */
	struct thermal_zone_params *tzp;
	long polling_period;
	int passive_delay;
	int tc1;
	int tc2;
	struct therm_est_subdevice subdevice;
	int use_activator;
};

struct therm_fan_est_subdevice {
	const char *dev_data;
	int (*get_temp)(const char *, int *);
	int group;
	/*
	 * as we read coeffs array from the device tree,
	 * specify type that has known width - 32 bits
	 *
	 * 'long' does not work, as its size if arch-dependent
	 */
	s32 coeffs[HIST_LEN];
	int hist[HIST_LEN];
};

struct therm_fan_est_data {
	long toffset;
	long polling_period;
	int ndevs;
	char *cdev_type;
	int active_trip_temps[MAX_ACTIVE_STATES];
	int active_hysteresis[MAX_ACTIVE_STATES];
	struct thermal_zone_params *tzp;
	struct therm_fan_est_subdevice devs[];
};

struct fan_dev_data {
	int next_state;
	int active_steps;
	int *fan_rpm;
	int *fan_pwm;
	int *fan_rru;
	int *fan_rrd;
	int *fan_state_cap_lookup;
	int num_profiles;
	int current_profile;
	const char **fan_profile_names;
	int **fan_profile_pwms;
	int *fan_profile_caps;
	struct workqueue_struct *workqueue;
	int fan_temp_control_flag;
	struct pwm_device *pwm_dev;
	bool pwm_legacy_api;
	int fan_cap_pwm;
	int fan_cur_pwm;
	int next_target_pwm;
	struct thermal_cooling_device *cdev;
	struct delayed_work fan_ramp_work;
	struct delayed_work fan_hyst_work;
	int step_time;
	long long precision_multiplier;
	struct mutex fan_state_lock;
	int pwm_period;
	int fan_pwm_max;
	struct device *dev;
	int tach_gpio;
	int tach_irq;
	atomic_t tach_enabled;
	int fan_state_cap;
	int pwm_gpio;
	int pwm_id;
	int fan_startup_pwm;
	int fan_startup_time;
	bool fan_kickstart; /*flag to check if fan is kickstart*/
	bool kickstart_en;  /*flag to check if kickstart feature is enabled*/
	enum pwm_polarity fan_pwm_polarity;
	int suspend_state;
	const char *name;
	struct regulator *fan_reg;
	bool is_fan_reg_enabled;
	struct dentry *debugfs_root;
	/* for tach feedback */
	atomic64_t rpm_measured;
	struct delayed_work fan_tach_work;
	struct workqueue_struct *tach_workqueue;
	int tach_period;
	spinlock_t irq_lock;
	int irq_count;
	u64 first_irq;
	u64 last_irq;
	u64 old_irq;

	bool   continuous_gov;
};

#define DEBUG 0
#define MULTIQP (100)
#define DEFERRED_RESUME_TIME 3000
#define THERMAL_GOV_PID "pid_thermal_gov"
#define THERMAL_CONTINUOUS_GOV "continuous_therm_gov"

struct therm_fan_estimator {
	long cur_temp;
	long pre_temp;
#if DEBUG
	long cur_temp_debug;
#endif
	long polling_period;
	struct workqueue_struct *workqueue;
	struct delayed_work therm_fan_est_work;
	long toffset;
	int ntemp;
	int ndevs;
	struct therm_fan_est_subdevice *devs;
	struct thermal_zone_device *thz;
	int current_trip_level;
	const char *cdev_type;
	rwlock_t state_lock;
	int num_profiles;
	int current_profile;
	const char **fan_profile_names;
	s32 **fan_profile_trip_temps;
	s32 **fan_profile_trip_hysteresis;
	s32 active_trip_temps[MAX_ACTIVE_STATES];
	s32 active_hysteresis[MAX_ACTIVE_STATES];
	struct thermal_zone_params *tzp;
	int num_resources;
	int trip_length;
	const char *name;
	bool is_pid_gov;
	bool reset_trip_index;
	/* allow cooling device to turn off at higher temperature if sleep */
	bool sleep_mode;
	int nonsleep_hyst;

	bool is_continuous_gov;
};
#endif /* _LINUX_THERM_EST_H */
