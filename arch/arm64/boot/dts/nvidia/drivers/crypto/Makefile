GCOV_PROFILE := y

ccflags-y += -I$(srctree)/drivers/video/tegra/host
ccflags-y += -I$(srctree.nvidia)/drivers/video/tegra/host
ccflags-y += -I$(srctree.nvidia)/include
ccflags-y += -I$(srctree)/include
obj-$(CONFIG_TEGRA_CRYPTO_DEV) += tegra-cryptodev.o
obj-$(CONFIG_CRYPTO_DEV_TEGRA_SE) += tegra-se.o
obj-$(CONFIG_CRYPTO_DEV_TEGRA_ELLIPTIC_SE) += tegra-se-elp.o
obj-$(CONFIG_CRYPTO_DEV_TEGRA_SE_USE_HOST1X_INTERFACE) += tegra-se-nvhost.o
obj-$(CONFIG_CRYPTO_DEV_TEGRA_VIRTUAL_SE_INTERFACE) += tegra-hv-vse.o
