ccflags-y += -I$(srctree)/include

tegra-vivid-objs := vivid-core.o vivid-ctrls.o vivid-vid-common.o vivid-vbi-gen.o \
		vivid-vid-cap.o vivid-vid-out.o vivid-kthread-cap.o vivid-kthread-out.o \
		vivid-radio-rx.o vivid-radio-tx.o vivid-radio-common.o \
		vivid-rds-gen.o vivid-sdr-cap.o vivid-vbi-cap.o vivid-vbi-out.o \
		vivid-osd.o vivid-tpg.o vivid-tpg-colors.o vivid-trace.o
obj-$(CONFIG_VIDEO_TEGRA_VIVID) += tegra-vivid.o
