#
# Memory controller code.
#

GCOV_PROFILE := y

ccflags-y += -I$(srctree)/arch/arm/mach-tegra/include \
             -I$(srctree)/arch/arm/mach-tegra \
             -I$(srctree)/drivers/platform/tegra/include

obj-y                                   += mc.o

obj-y                                   += mcerr.o
obj-y                                   += mcerr-t21.o

obj-y                                   += latency_allowance.o
obj-y                                   += tegra21x_la.o

obj-$(CONFIG_ARCH_TEGRA_19x_SOC)        += tegra19x_la_ptsa_core.o tegra19x_la_ptsa.o
obj-y                                   += fixed_point.o
obj-$(CONFIG_TEGRA_ISOMGR)              += isomgr.o isomgr-pre_t19x.o isomgr-t19x.o
obj-$(CONFIG_TEGRA_BWMGR)               += emc_bwmgr.o pmqos_bwmgr_client.o
obj-$(CONFIG_TEGRA_BWMGR)               += emc_bwmgr-t21x.o emc_bwmgr-t18x.o emc_bwmgr-t19x.o

obj-y                                   += tegra-mc-sid.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC)        += mcerr_ecc_t18x.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC)        += mc_addr_translate.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC)        += mcerr-t18x.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC)        += tegra18_emc.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC)        += tegra18x_la.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC)        += tegra186-mc-sid.o

obj-y					+= tegra_fd.o

ifdef CONFIG_ARCH_TEGRA_19x_SOC

ccflags-y += -Werror

obj-y += tegra194-mc-sid.o
obj-y += mc-t19x.o
obj-y += mcerr-t19x.o

endif
