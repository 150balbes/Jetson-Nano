ifdef CONFIG_ARCH_TEGRA_19x_SOC

ccflags-y += -Werror
ccflags-y += -I$(srctree)/drivers/iommu

obj-$(CONFIG_TEGRA_ARM_SMMU_T19x) += arm-smmu-t19x.o
obj-$(CONFIG_ARM_SMMU) += arm-smmu-regs.o
obj-$(CONFIG_ARM_SMMU) += of_tegra-smmu.o
obj-y += dma-override.o

endif

obj-$(CONFIG_ARM_SMMU_SUSPEND) += arm-smmu-suspend.o
obj-$(CONFIG_OF_IOMMU) += of_tegra-smmu.o
