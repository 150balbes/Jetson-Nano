ifdef CONFIG_TEGRA_T19X_GRHOST

GCOV_PROFILE := y
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/host
ccflags-y += -I$(srctree.nvidia)/include
ccflags-y += -Werror

nvhost-pva-objs  = \
		pva.o \
		pva_ioctl.o \
		pva_mailbox.o \
		pva_isr.o \
		pva_queue.o \
		pva_debug.o \
		pva_trace.o \
		pva_abort.o \
		pva_ccq.o \
		pva_selftest.o

obj-$(CONFIG_TEGRA_GRHOST_PVA) += nvhost-pva.o

endif
