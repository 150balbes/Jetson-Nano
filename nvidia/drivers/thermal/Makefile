ccflags-y += -I$(srctree.nvidia)/include
ccflags-y += -I$(srctree.nvidia)/drivers/thermal
ccflags-y += -I$(srctree)/drivers/thermal

thermal_sys-$(CONFIG_THERMAL_GOV_CONTINUOUS) += continuous_thermal_gov.o

obj-$(CONFIG_NV_TEGRA_BPMP) += tegra_bpmp_thermal.o
obj-$(CONFIG_TEGRA_THERMAL_THROTTLE) += tegra_throttle.o
obj-$(CONFIG_TEGRA_THERMAL_THROTTLE) += tegra_thermal_throttle.o
obj-$(CONFIG_PWM_FAN) += pwm_fan.o
obj-$(CONFIG_TEGRA_THERMAL_THROTTLE) += tegra_oc_event.o

# userspace cooling
obj-$(CONFIG_USERSPACE_THERM_ALERT)    += userspace_alert.o

thermal_sys-$(CONFIG_THERMAL_GOV_PID) += pid_thermal_gov.o
