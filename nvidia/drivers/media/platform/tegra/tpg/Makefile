GCOV_PROFILE := y
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/host
ccflags-y += -I$(srctree.nvidia)/drivers/media/platform/tegra
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/host
ccflags-y += -Werror

ifeq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
nvhost-vi-tpg-objs += tpg_t18x.o
else
nvhost-vi-tpg-objs += tpg_t21x.o
endif

ifeq ($(CONFIG_ARCH_TEGRA_19x_SOC),y)
nvhost-vi-tpg-t19x-objs += tpg_t19x.o
endif

obj-$(CONFIG_VIDEO_TEGRA_VI_TPG) += nvhost-vi-tpg.o
ifeq ($(CONFIG_ARCH_TEGRA_19x_SOC),y)
obj-$(CONFIG_VIDEO_TEGRA_VI_TPG) += nvhost-vi-tpg-t19x.o
endif
