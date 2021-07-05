old-dtb := $(dtb-y)
old-dtbo := $(dtbo-y)
dtb-y :=
dtbo-y :=
makefile-path := platform/t210/batuu/kernel-dts

dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0003-p3542-0000.dtb
dtb-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0003-p3542-0000-hdmi-dsi.dtb
dtbo-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0003-p3542-0000-adafruit-sph0645lm4h.dtbo
dtbo-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0003-p3542-0000-fe-pi-audio.dtbo
dtbo-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0003-p3542-0000-hdr40.dtbo
dtbo-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0003-p3542-0000-respeaker-4-mic-array.dtbo
dtbo-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-0003-p3542-0000-mcp251x.dtbo

ifneq ($(dtb-y),)
dtb-y := $(addprefix $(makefile-path)/,$(dtb-y))
endif
ifneq ($(dtbo-y),)
dtbo-y := $(addprefix $(makefile-path)/,$(dtbo-y))
endif

dtb-y += $(old-dtb)
dtbo-y += $(old-dtbo)
