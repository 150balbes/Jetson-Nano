GCOV_PROFILE := y


subdir-ccflags-y := -Werror

USE_NV2 := n

ifeq ($(CONFIG_TEGRA_NVMAP_V2),y)

obj-y += nv2/

else

ifeq ($(CONFIG_ARM64),y)
ccflags-y += -Iarch/arm/mach-tegra/include
endif

ccflags-y += -Iarch/arm/mach-tegra
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/nvmap/
ccflags-y += -I$(srctree.nvidia)/include/

obj-y += nvmap.o
obj-y += nvmap_alloc.o
obj-y += nvmap_cache.o
obj-y += nvmap_dev.o
obj-y += nvmap_dmabuf.o
obj-y += nvmap_fault.o
obj-y += nvmap_handle.o
obj-y += nvmap_heap.o
obj-y += nvmap_ioctl.o
obj-y += nvmap_init.o
obj-y += nvmap_tag.o
obj-y += nvmap_mm.o
obj-y += nvmap_stats.o
obj-y += nvmap_carveout.o

obj-$(CONFIG_NVMAP_PAGE_POOLS) += nvmap_pp.o

ifeq ($(CONFIG_ARCH_TEGRA_18x_SOC),y)
obj-y += nvmap_cache_nvmap_t18x.o
endif

ifdef CONFIG_ARCH_TEGRA_19x_SOC

obj-y += nvmap_init_t19x.o
obj-y += nvmap_dmabuf_t19x.o
obj-y += nvmap_cache_t19x.o

endif

endif
