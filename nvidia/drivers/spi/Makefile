GCOV_PROFILE := y

ccflags-y += -Werror
ccflags-$(CONFIG_SPI_DEBUG) := -DDEBUG

obj-$(CONFIG_SPI_TEGRA186_AON)		+= spi-tegra-aon.o
obj-$(CONFIG_SPI_AURIX_TEGRA)		+= spi-aurix-tegra.o

