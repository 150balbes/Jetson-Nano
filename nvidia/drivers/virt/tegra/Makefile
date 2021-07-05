#
# Makefile for Hypervisor interface
#

obj-$(CONFIG_TEGRA_HV_PM_CTL)		+= tegra_hv_pm_ctl.o

obj-$(CONFIG_TEGRA_HV_MANAGER)		+= tegra_hv.o ivc-cdev.o
obj-$(CONFIG_TEGRA_HV_MANAGER)		+= userspace_ivc_mempool.o
obj-$(CONFIG_TEGRA_HV_SYSFS)		+= hvc_sysfs.o
obj-$(CONFIG_TEGRA_HV_WDT_HANDLER)	+= tegra_hv_wdt_handler.o

obj-$(CONFIG_TEGRA_VM_ERR_HANDLER)	+= vm_err.o
obj-$(CONFIG_TEGRA_VM_ERR_HANDLER)	+= vm_err_sample_handler.o
