GCOV_PROFILE := y
subdir-ccflags-y := -Werror
ccflags-y += -I$(overlay)/drivers/video/tegra/host
ccflags-y += -I$(overlay)/drivers/video/tegra/dc
ccflags-y += -I$(overlay)/drivers/platform/tegra
ccflags-$(CONFIG_TRUSTED_LITTLE_KERNEL) += -I$(overlay)/security/tlk_driver
ccflags-y += -I$(srctree)/arch/arm/mach-tegra
obj-y += dc_common.o dc.o mode.o clock.o window.o dc_client.o
obj-y += t21x/bandwidth.o
obj-y += t21x/csc.o
obj-y += t21x/lut.o
obj-y += cursor.o
obj-y += of_dc.o
ccflags-y +=-DCOMMON_MIPICAL_SUPPORTED\
	-I$(overlay)/drivers/media/platform/tegra

#obj-y += hdcp/

ifneq ($(filter y,$(CONFIG_TRUSTY) $(CONFIG_TRUSTED_LITTLE_KERNEL)),)
obj-$(CONFIG_TEGRA_HDMIHDCP) += hdcp/hdmihdcp.o
obj-$(CONFIG_TEGRA_HDMIHDCP) += hdcp/nvhdcp_hdcp22_methods.o
obj-$(CONFIG_TEGRA_DPHDCP) += hdcp/dphdcp.o
ccflags-$(CONFIG_TEGRA_HDMIHDCP) += -DCONFIG_HDCP
ccflags-$(CONFIG_TEGRA_DPHDCP) += -DCONFIG_DPHDCP
endif

obj-y += edid.o edid_disp_id_ext.o edid_quirks.o
obj-y += dsi.o
obj-$(CONFIG_DEBUG_FS) += dsi_debug.o
obj-$(CONFIG_DEBUG_FS) += tegra_dsi_csi_test.o
obj-y += dc_sysfs.o
obj-y += dc_config.o
obj-y += sor.o
obj-y += dpaux.o
obj-$(CONFIG_DEBUG_FS) += dp_debug.o
obj-$(CONFIG_TEGRA_DP) += dp.o
obj-$(CONFIG_TEGRA_DP) += dp_lt.o
obj-$(CONFIG_TEGRA_DP) += dp_auto.o
obj-$(CONFIG_TEGRA_DP) += hpd.o
obj-$(CONFIG_TEGRA_DC) += ext/
obj-$(CONFIG_TEGRA_HDMI2_0) += hdmi2.0.o
obj-$(CONFIG_TEGRA_HDA_DC) += hda_dc.o
obj-$(CONFIG_TEGRA_DC_FAKE_PANEL_SUPPORT) += fake_panel.o
obj-$(CONFIG_TEGRA_VRR) += vrr.o
obj-y += dsi_padctrl.o
obj-$(CONFIG_TEGRA_HDMIVRR) += hdmivrr.o
obj-y += panel/
obj-y += bridge/
obj-$(CONFIG_TEGRA_CEC_SUPPORT) += ../../../misc/tegra-cec/
obj-y += crc.o

GCOV_PROFILE := y
subdir-ccflags-y := -Werror
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/dc
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/dc/nvdisp

obj-y += sor_t19x.o
obj-y += nvdisp/

