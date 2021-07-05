GCOV_PROFILE := y
ccflags-y += -Werror

obj-$(CONFIG_TEGRA_NVADSP) := nvadsp.o
nvadsp-objs += dev.o os.o app.o app_loader_linker.o\
	 amc.o nvadsp_shared_sema.o \
	 hwmailbox.o mailbox.o msgq.o \
	 mem_manager.o aram_manager.o dram_app_mem_manager.o \
	 dev-t21x.o os-t21x.o dev-t18x.o os-t18x.o acast.o


ifeq ($(CONFIG_TEGRA_ADSP_DFS),y)
nvadsp-objs += adsp_dfs.o
endif

ifeq ($(CONFIG_TEGRA_ADSP_ACTMON),y)
nvadsp-objs += ape_actmon.o
endif

ifeq ($(CONFIG_TEGRA_EMC_APE_DFS),y)
nvadsp-objs += emc_dfs.o
endif

ifeq ($(CONFIG_TEGRA_ADSP_CONSOLE),y)
nvadsp-objs += adsp_console_dbfs.o
endif

ifeq ($(CONFIG_TEGRA_ADSP_CPUSTAT),y)
nvadsp-objs += adsp_cpustat.o
endif

ifeq ($(CONFIG_TEGRA_ADSP_FILEIO),y)
nvadsp-objs += adspff.o
endif

ifeq ($(CONFIG_TEGRA_ADSP_LPTHREAD),y)
nvadsp-objs += adsp_lpthread.o
endif
