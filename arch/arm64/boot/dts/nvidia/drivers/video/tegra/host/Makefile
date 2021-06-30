GCOV_PROFILE := y
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/host
ccflags-y += -I$(srctree.nvidia)/include
ccflags-y += -Idrivers/video/tegra/host
ccflags-y += -Idrivers/devfreq
ccflags-y += -Wno-multichar

subdir-ccflags-y := -Werror

nvhost-objs = \
	nvhost_acm.o \
	nvhost_syncpt.o \
	nvhost_cdma.o \
	nvhost_intr.o \
	nvhost_channel.o \
	nvhost_job.o \
	dev.o \
	debug.o \
	bus_client.o \
	chip_support.o \
	nvhost_vm.o \
	nvhost_pd.o \
	interrupt_syncpt.o

obj-$(CONFIG_TEGRA_GRHOST) += nvhost.o
obj-$(CONFIG_TEGRA_GRHOST) += host1x/
obj-$(CONFIG_TEGRA_GRHOST_TSEC) += tsec/
obj-$(CONFIG_TEGRA_GRHOST_NVDEC) += nvdec/
obj-$(CONFIG_TEGRA_GRHOST_ISP) += isp/
obj-$(CONFIG_TEGRA_GRHOST) += t124/
obj-$(CONFIG_TEGRA_GRHOST) += flcn/
obj-$(CONFIG_TEGRA_GRHOST) += t210/
obj-$(CONFIG_TEGRA_GRHOST_SCALE) += scale_emc.o nvhost_scale.o

ifeq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
obj-$(CONFIG_TEGRA_GRHOST) += iommu_context_dev.o

obj-$(CONFIG_TEGRA_GRHOST) += t186/
endif

obj-$(CONFIG_TEGRA_GRHOST_NVCSI) += nvcsi/
obj-$(CONFIG_TEGRA_GRHOST_SYNC) += nvhost_sync.o
obj-$(CONFIG_ARCH_TEGRA_18x_SOC) += vi/
obj-$(CONFIG_TEGRA_GRHOST_VHOST) += vhost/

ifdef CONFIG_TEGRA_T19X_GRHOST

obj-y  += nvhost_queue.o \
	  nvhost_buffer.o \
	  nvhost_syncpt_unit_interface.o

obj-$(CONFIG_TEGRA_GRHOST) += t194/
obj-$(CONFIG_TEGRA_GRHOST_CAPTURE_SUPPORT) += capture/
obj-$(CONFIG_TEGRA_GRHOST_PVA) += pva/
obj-$(CONFIG_TEGRA_GRHOST_NVDLA) += nvdla/
obj-$(CONFIG_TEGRA_GRHOST_SLVSEC) += slvsec/
obj-$(CONFIG_TEGRA_GRHOST) += nvhost_minimal_pm/

endif

ifdef CONFIG_EVENTLIB

$(obj)/bus_client.o: $(obj)/nvhost_events_json.h

$(obj)/nvhost_events_json.h: $(src)/nvhost_events.json
	od -tx1 -v $< | sed -r -e's,^[0-9a-f]+, ,' -e's! ([0-9a-f][0-9a-f])! 0x\1,!g' | head -c -4 > $@

clean-files := nvhost_events_json.h

endif
