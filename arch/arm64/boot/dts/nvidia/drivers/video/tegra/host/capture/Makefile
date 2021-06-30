GCOV_PROFILE := y

ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/host
ccflags-y += -I$(srctree.nvidia)/drivers/media/platform/tegra
ccflags-y += -I$(srctree.nvidia)/include
ccflags-y += -Werror

nvhost-capture-objs = capture-support.o

obj-$(CONFIG_TEGRA_GRHOST) += nvhost-capture.o
