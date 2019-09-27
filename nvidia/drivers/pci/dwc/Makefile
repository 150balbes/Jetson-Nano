ifdef CONFIG_ARCH_TEGRA_19x_SOC

ccflags-y += -I$(srctree)/drivers/pci/dwc

ifneq ($(filter y,$(CONFIG_PCIE_TEGRA_HOST) $(CONFIG_PCIE_TEGRA_EP)),)
        obj-$(CONFIG_PCIE_TEGRA) += pcie-tegra.o
endif

endif
