ifdef CONFIG_TEGRA_T19X_GRHOST

GCOV_PROFILE := y

ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/host
ccflags-y += -I$(srctree.nvidia)/drivers/media/platform/tegra
ccflags-y += -I$(srctree.nvidia)/include
ccflags-y += -Werror

nvhost-slvsec-objs = slvsec.o

obj-$(CONFIG_TEGRA_GRHOST_SLVSEC) += nvhost-slvsec.o

endif
