KBUILD_CFLAGS += -I$(srctree)/arch/arm/mach-tegra \
		 -I$(srctree)/arch/arm/mach-tegra/include/ \
		 -I$(srctree)/drivers/platform/tegra/ \
		 -I$(srctree.nvidia)/drivers/platform/tegra/rtcpu \
		 -I$(srctree.nvidia)/drivers/video/tegra/dc/ \
		 -I$(srctree.nvidia)/drivers/video/tegra/dc/panel/

ccflags-y += -Werror
obj-y += tegra-mce.o
obj-$(CONFIG_TEGRA_MC) += mc/
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra18_speedo.o
obj-$(CONFIG_TEGRA_DC) += panel-simulator.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra18x-mce.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra-roc.o
obj-$(CONFIG_TEGRA_PMC_AO_WAKE) += tegra186-aowake.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra186-system-config.o
obj-$(CONFIG_TEGRA_CAMERA_RTCPU) += tegra-camera-rtcpu.o
obj-$(CONFIG_TEGRA_CAMERA_RTCPU) += rtcpu/
obj-$(CONFIG_TEGRA_PM_IRQ) += pm-irq-t18x.o
ifeq ($(CONFIG_PM_SLEEP),y)
obj-$(CONFIG_TEGRA_WAKEUP) += wakeups-t18x.o
endif
obj-$(CONFIG_TEGRA_CENTRAL_ACTMON) += central_actmon/
obj-y += powergate/
obj-$(CONFIG_TEGRA_AON) += tegra-aon.o
obj-$(CONFIG_TEGRA_AON) += tegra-aon-ivc-echo.o
obj-$(CONFIG_TEGRA_CPU_TOPOLOGY_DEBUGFS) += tegra18_cpu_debugfs.o
obj-$(CONFIG_TEGRA_CPU_TOPOLOGY_SYSFS) += tegra_cpu_sysfs.o
obj-$(CONFIG_PM) += pm.o
obj-$(CONFIG_PM) += pm_debugfs.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += reboot-t18x.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += pm-tegra186.o
obj-$(CONFIG_TEGRA_AON) += tegra186_aondbg.o
obj-$(CONFIG_TEGRA_CAMERA_RTCPU) += rtcpu/ivc-bus.o
obj-$(CONFIG_TEGRA_CAMERA_RTCPU) += rtcpu/hsp-doorbell.o
obj-$(CONFIG_TEGRA_ARI_MCA) += ari_mca.o
obj-$(CONFIG_TEGRA_18X_SERROR) += tegra18_serr.o
obj-$(CONFIG_TEGRA_BRIDGE_MCA) += bridge_mca.o
obj-$(CONFIG_TEGRA_18X_SERROR) += tegra18_a57_serr.o
obj-$(CONFIG_TEGRA_CENTRAL_ACTMON) += cactmon_t18x.o
obj-$(CONFIG_TEGRA_CENTRAL_ACTMON) += cactmon_t21x.o
ifeq ($(CONFIG_HW_PERF_EVENTS), y)
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += tegra18_perf_uncore.o
endif
obj-$(CONFIG_TEGRA_PTP_NOTIFIER) += ptp-notifier.o
obj-$(CONFIG_DENVER_CPU) += denver-knobs.o
obj-$(CONFIG_DENVER_MCA) += denver_mca.o
obj-$(CONFIG_TEGRA_SAFETY_SCE) += tegra-safety.o
obj-$(CONFIG_TEGRA_HV_XHCI_DEBUG) += tegra-hv-xhci-debug.o
tegra-safety-objs += tegra-safety-ivc.o tegra-safety-dev.o

obj-$(CONFIG_PSTORE_RAM) += tegra_ramoops.o

obj-$(CONFIG_TEGRA_NVADSP) += nvadsp/

obj-y += tegra_vpr.o
obj-y += smc-calls.o

ifeq ($(CONFIG_CPU_FREQ),y)
obj-$(CONFIG_ARCH_TEGRA_210_SOC) += cpu_emc.o
endif

ifdef CONFIG_ARCH_TEGRA_19x_SOC

obj-$(CONFIG_TEGRA_CENTRAL_ACTMON) += cactmon_t19x.o
obj-y += tegra19x-mce.o
obj-y += t19x_cache.o
obj-y += cvnas.o
obj-$(CONFIG_ARCH_TEGRA_210_SOC) += mselect.o
obj-$(CONFIG_TEGRA_19X_RAS) += carmel_ras.o
obj-$(CONFIG_TEGRA_CBB_NOC) += tegra_cbb.o
ifeq ($(CONFIG_HW_PERF_EVENTS), y)
obj-$(CONFIG_ARCH_TEGRA_19x_SOC) += tegra19_perf_uncore.o
endif

endif

obj-$(CONFIG_TEGRA_HSP) += tegra-hsp.o
obj-$(CONFIG_TEGRA_HSP) += tegra186-hsp.o
obj-$(CONFIG_PM_SLEEP) += tegra-wakeups.o

obj-$(CONFIG_TEGRA_FIRMWARES_CLASS)    += firmwares.o
obj-$(CONFIG_TEGRA_FIRMWARES_INVENTORY)    += firmwares-all.o
obj-$(CONFIG_NV_TEGRA_IVC)		+= tegra-ivc.o
obj-$(CONFIG_TEGRA_FIQ_DEBUGGER)        += tegra_fiq_debugger.o

obj-$(CONFIG_TEGRA_BOOTLOADER_DEBUG)    += tegra_bootloader_debug.o
obj-$(CONFIG_TEGRA_BOOTLOADER_DEBUG_INIT) += tegra_bootloader_debug_init.o
obj-$(CONFIG_TEGRA_BOOTLOADER_BOOT_CFG)	+= bcp.o

obj-$(CONFIG_TEGRA_WDT_RECOVERY)        += wdt-recovery.o
obj-y					+= tegra_prod.o
obj-m 					+= bad.o

obj-$(CONFIG_TEGRA_CLOCKS_CONFIGURE)	+= clocks-config.o
