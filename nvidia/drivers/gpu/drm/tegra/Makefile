ccflags-y += -Idrivers/gpu/drm/tegra
ccflags-y += -I$(srctree.nvidia)/drivers/gpu/drm/tegra

tegra-drm-t186-y = \
	vic_t186.o \
	nvenc_t186.o \
	nvdec_t186.o \
	nvjpg_t186.o \
	tsec_t186.o

ifeq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
obj-$(CONFIG_TEGRA_HOST1X) += tegra-drm-t186.o
endif
