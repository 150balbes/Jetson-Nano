GCOV_PROFILE := y
subdir-ccflags-y := -Werror
ccflags-y += -Idrivers/video/tegra
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/dc
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/dc/panel

obj-y += nvdisp.o
obj-y += nvdisp_win.o
obj-y += nvdisp_stub.o
obj-y += nvdisp_cursor.o
obj-y += nvdisp_config.o
obj-y += nvdisp_bandwidth.o
obj-y += nvdisp_crc.o
obj-y += nvdisp_lut.o
obj-y += nvdisp_csc.o

ifdef CONFIG_ARCH_TEGRA_19x_SOC

ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/dc
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/dc/nvdisp

obj-y += nvdisp_t19x.o

endif
