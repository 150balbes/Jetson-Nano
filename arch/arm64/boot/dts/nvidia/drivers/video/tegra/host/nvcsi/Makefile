GCOV_PROFILE := y
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/host
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -I$(srctree.nvidia)/drivers/media/platform/tegra
ccflags-y += -I$(srctree.nvidia)/include
ccflags-y += -Werror

nvhost-nvcsi-objs  = \
		nvcsi.o \
		deskew.o

obj-$(CONFIG_TEGRA_GRHOST) += nvhost-nvcsi.o

ifdef CONFIG_TEGRA_T19X_GRHOST

nvhost-nvcsi-t194-objs = nvcsi-t194.o

obj-$(CONFIG_TEGRA_GRHOST_NVCSI) += nvhost-nvcsi-t194.o

endif
