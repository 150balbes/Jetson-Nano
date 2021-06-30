GCOV_PROFILE := y
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/host
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -Idrivers/video/tegra/host/host1x
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/host/t186
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/host
ccflags-y += -Idrivers/video/tegra/host/t186
ccflags-y += -I$(srctree.nvidia)/include
ccflags-y += -Wno-multichar
ccflags-y += -Werror

nvhost-t186-objs  = \
        t186.o

obj-$(CONFIG_TEGRA_GRHOST) += nvhost-t186.o
