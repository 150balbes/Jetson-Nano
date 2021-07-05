GCOV_PROFILE := y

subdir-ccflags-y += -Werror

################################################################################
# tegra platform drivers
obj-$(CONFIG_ARCH_TEGRA_18x_SOC)		+= tegra_cpufreq.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC)		+= tegra_cpufreq_hv.o
obj-$(CONFIG_ARCH_TEGRA_19x_SOC)		+= tegra194_cpufreq.o
obj-$(CONFIG_ARCH_TEGRA_210_SOC)		+= tegra210-cpufreq.o
################################################################################
obj-y += cpufreq_cpu_emc_table.o
