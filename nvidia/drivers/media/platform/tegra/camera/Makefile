GCOV_PROFILE := y
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/host
ccflags-y += -I$(srctree)/drivers/video/tegra/host
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/camera
ccflags-y += -I$(srctree.nvidia)/drivers/media/platform/tegra
ccflags-y += -Werror

obj-y += vi/
obj-y += csi/
obj-y += camera_common.o camera_gpio.o sensor_common.o camera_version_utils.o \
	tegracam_ctrls.o tegracam_core.o tegracam_v4l2.o tegracam_utils.o
obj-$(CONFIG_TEGRA_CAMERA_RTCPU) += capture_common.o
obj-$(CONFIG_TEGRA_CAMERA_RTCPU) += isp/isp_channel.o
obj-$(CONFIG_TEGRA_CAMERA_RTCPU) += isp/capture_isp.o

obj-y += tests/

ifdef CONFIG_ARCH_TEGRA_19x_SOC

obj-y += slvs/
obj-y += nvcsi/

endif
