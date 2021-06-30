old-dtb := $(dtb-y)
dtb-y :=
makefile-path := platform/t210/jetson/kernel-dts

dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-jetson-cv-base-p2597-2180-a00.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-jetson-cv-p2597-2180-a00.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-jetson-cv-p2597-2180-imx274-hdmi.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-jetson-cv-p2597-2180-a00-auo-1080p-edp.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-jetson-tx1-p2597-2180-a01-android-devkit.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-jetson-tx1-p2597-2180-a01-devkit.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-jetson-e-base-p2595-0000-a00.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-jetson-e-p2595-0000-a00-00.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-jetson-tx1-p2597-2180-a02-devkit-24x7.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-jetson-tx1-p2597-2180-a01-devkit_as_TM660M.dtb

ifneq ($(dtb-y),)
dtb-y := $(addprefix $(makefile-path)/,$(dtb-y))
endif

dtb-y += $(old-dtb)
