GCOV_PROFILE := y
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/host
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -Werror

nvhost-host1x-objs  = \
	host1x.o

obj-$(CONFIG_TEGRA_GRHOST) += nvhost-host1x.o
