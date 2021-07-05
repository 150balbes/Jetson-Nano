old-dtb := $(dtb-y)
old-dtbo := $(dtbo-y)
dtb-y :=
dtbo-y :=
makefile-path := platform/t210/porg/kernel-dts

dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0000-p3449-0000-a00.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0000-p3449-0000-a01.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0000-p3449-0000-a02.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0002-p3449-0000-a02.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0000-p3449-0000-b00.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0002-p3449-0000-b00.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0000-as-p3448-0003.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0000-p3449-0000-b00-hdmi-dsi.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0000-p3449-0000-a02-hdmi-dsi.dtb
dtbo-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0000-p3449-0000-a01-adafruit-sph0645lm4h.dtbo
dtbo-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0000-p3449-0000-a02-adafruit-sph0645lm4h.dtbo
dtbo-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0000-p3449-0000-a01-fe-pi-audio.dtbo
dtbo-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0000-p3449-0000-a02-fe-pi-audio.dtbo
dtbo-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0000-p3449-0000-a01-hdr40.dtbo
dtbo-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0000-p3449-0000-a02-hdr40.dtbo
dtbo-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0000-p3449-0000-a01-respeaker-4-mic-array.dtbo
dtbo-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0000-p3449-0000-a02-respeaker-4-mic-array.dtbo
dtbo-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0000-p3449-0000-a01-mcp251x.dtbo
dtbo-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0000-p3449-0000-a02-mcp251x.dtbo

ifneq ($(dtb-y),)
dtb-y := $(addprefix $(makefile-path)/,$(dtb-y))
endif
ifneq ($(dtbo-y),)
dtbo-y := $(addprefix $(makefile-path)/,$(dtbo-y))
endif

dtb-y += $(old-dtb)
dtbo-y += $(old-dtbo)
