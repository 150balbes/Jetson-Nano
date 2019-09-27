GCOV_PROFILE := y
subdir-ccflags-y := -Werror
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/host
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/dc
ccflags-y += -I$(srctree.nvidia)/drivers/platform/tegra
ccflags-y += -I$(srctree)/arch/arm/mach-tegra

obj-$(CONFIG_TEGRA_HDMI2FPD_DS90UH949) += hdmi2fpd_ds90uh949.o
obj-$(CONFIG_TEGRA_HDMI2DSI_TC358870) += hdmi2dsi_tc358870.o
obj-$(CONFIG_TEGRA_DSI2EDP_TC358767) += tc358767_dsi2edp.o
obj-$(CONFIG_TEGRA_DSI2EDP_SN65DSI86) += sn65dsi86_dsi2edp.o
obj-$(CONFIG_TEGRA_DSI2LVDS_SN65DSI85) += sn65dsi85_dsi2lvds.o
obj-$(CONFIG_TEGRA_LVDS2FPDL_DS90UB947) += ds90ub947-q1.o
obj-$(CONFIG_TEGRA_DS90UH948Q_DESER)    += ds90uh948q_deser.o
obj-$(CONFIG_TEGRA_EDP2LVDS_PS8625) += ps8625_edp2lvds.o
obj-$(CONFIG_TEGRA_HDMI2GMSL_MAX929x) += max929x_hdmi2gmsl.o
