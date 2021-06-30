GCOV_PROFILE := y
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/host
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -I$(srctree.nvidia)/include
ccflags-y += -Wno-multichar
ccflags-y += -Werror

nvhost-t124-objs  = \
	t124.o

obj-$(CONFIG_TEGRA_GRHOST) += nvhost-t124.o
