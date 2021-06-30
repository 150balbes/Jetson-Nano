GCOV_PROFILE := y
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/host
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -Werror

nvhost-vhost-objs  = \
	vhost.o \
	vhost_intr.o \
	vhost_syncpt.o \
	vhost_channel.o \
	vhost_cdma.o \
	vhost_client.o

obj-$(CONFIG_TEGRA_GRHOST) += nvhost-vhost.o
