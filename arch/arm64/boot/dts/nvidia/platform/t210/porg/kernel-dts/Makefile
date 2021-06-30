old-dtb := $(dtb-y)
dtb-y :=
makefile-path := platform/t210/porg/kernel-dts

dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0000-p3449-0000-a00.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0000-p3449-0000-a01.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0000-p3449-0000-a02.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0002-p3449-0000-a02.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0000-p3449-0000-b00.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0002-p3449-0000-b00.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0000-p3449-0000-b00-hdmi-dsi.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0000-p3449-0000-a02-hdmi-dsi.dtb

ifneq ($(dtb-y),)
dtb-y := $(addprefix $(makefile-path)/,$(dtb-y))
endif

dtb-y += $(old-dtb)
