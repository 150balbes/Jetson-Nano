old-dtb := $(dtb-y)
dtb-y :=
makefile-path := platform/t19x/galen/kernel-dts

dtb-$(CONFIG_ARCH_TEGRA_19x_SOC) += tegra194-p2888-0001-p2822-0000.dtb
dtb-$(CONFIG_ARCH_TEGRA_19x_SOC) += tegra194-p2888-0001-p2822-0000-imx274-hdmi.dtb
dtb-$(CONFIG_ARCH_TEGRA_19x_SOC) += tegra194-p2888-0001-p2822-0000-imx185_v1.dtb
dtb-$(CONFIG_ARCH_TEGRA_19x_SOC) += tegra194-p2888-0001-p2822-0000-maxn.dtb
dtb-$(CONFIG_ARCH_TEGRA_19x_SOC) += tegra194-p2888-0001-e3366-1199.dtb
dtb-$(CONFIG_ARCH_TEGRA_19x_SOC) += tegra194-p2888-0006-p2822-0000.dtb
dtb-$(CONFIG_ARCH_TEGRA_19x_SOC) += tegra194-p2888-as-0006-p2822-0000.dtb

ifneq ($(dtb-y),)
dtb-y := $(addprefix $(makefile-path)/,$(dtb-y))
endif

dtb-y += $(old-dtb)
