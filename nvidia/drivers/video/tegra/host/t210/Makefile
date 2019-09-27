GCOV_PROFILE := y
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/host
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/host/t124
ccflags-y += -Idrivers/video/tegra/host/t124
ccflags-y += -I$(srctree.nvidia)/include
ccflags-y += -Wno-multichar
ccflags-y += -Werror

nvhost-t210-objs  = \
        t210.o

obj-$(CONFIG_TEGRA_GRHOST) += nvhost-t210.o
