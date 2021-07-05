GCOV_PROFILE := y

subdir-ccflags-y := -Werror

subdir-ccflags-y += -I$(srctree)
subdir-ccflags-y += -I$(srctree)/sound/soc/codecs/
subdir-ccflags-y += -I$(srctree.nvidia)
subdir-ccflags-y += -I$(srctree.nvidia)/sound/soc/tegra-virt-alt/
subdir-ccflags-y += -I$(srctree.nvidia)/sound/soc/tegra-alt/include/

#----------------------------- utility drivers --------------------------------------
CFLAGS_tegra_pcm_alt.o 				+= -Wno-shift-count-overflow

snd-soc-tegra-alt-utils-objs 			:= utils/tegra_asoc_utils_alt.o		\
							utils/tegra_pcm_alt.o		\
							utils/tegra_asoc_machine_alt.o	\
							utils/tegra_isomgr_bw_alt.o	\
							utils/ahub_unit_fpga_clock.o	\
							utils/tegra210_xbar_utils_alt.o

obj-$(CONFIG_SND_SOC_TEGRA_ALT)			+= snd-soc-tegra-alt-utils.o

#----------------------------- platform drivers -------------------------------------
snd-soc-tegra210-alt-admaif-objs 		:= tegra210_admaif_alt.o
snd-soc-tegra210-alt-xbar-objs 			:= tegra210_xbar_alt.o
snd-soc-tegra210-alt-i2s-objs 			:= tegra210_i2s_alt.o
snd-soc-tegra210-alt-dmic-objs 			:= tegra210_dmic_alt.o
snd-soc-tegra210-alt-amx-objs 			:= tegra210_amx_alt.o
snd-soc-tegra210-alt-adx-objs 			:= tegra210_adx_alt.o
snd-soc-tegra210-alt-mixer-objs 		:= tegra210_mixer_alt.o
snd-soc-tegra210-alt-sfc-objs 			:= tegra210_sfc_alt.o
snd-soc-tegra210-alt-afc-objs 			:= tegra210_afc_alt.o
snd-soc-tegra210-alt-mvc-objs 			:= tegra210_mvc_alt.o
snd-soc-tegra210-alt-iqc-objs 			:= tegra210_iqc_alt.o
snd-soc-tegra210-alt-adsp-objs			:= tegra210_adsp_alt.o
snd-soc-tegra186-alt-asrc-objs			:= tegra186_asrc_alt.o
snd-soc-tegra186-alt-arad-objs			:= tegra186_arad_alt.o
snd-soc-tegra186-alt-dspk-objs			:= tegra186_dspk_alt.o
snd-soc-tegra210-alt-ope-objs 			:= tegra210_ope_alt.o		\
							tegra210_peq_alt.o	\
							tegra210_mbdrc_alt.o

obj-$(CONFIG_SND_SOC_TEGRA210_ADMAIF_ALT)	+= snd-soc-tegra210-alt-admaif.o
obj-$(CONFIG_SND_SOC_TEGRA210_XBAR_ALT)		+= snd-soc-tegra210-alt-xbar.o
obj-$(CONFIG_SND_SOC_TEGRA210_I2S_ALT)		+= snd-soc-tegra210-alt-i2s.o
obj-$(CONFIG_SND_SOC_TEGRA210_DMIC_ALT)		+= snd-soc-tegra210-alt-dmic.o
obj-$(CONFIG_SND_SOC_TEGRA210_AMX_ALT)		+= snd-soc-tegra210-alt-amx.o
obj-$(CONFIG_SND_SOC_TEGRA210_ADX_ALT)		+= snd-soc-tegra210-alt-adx.o
obj-$(CONFIG_SND_SOC_TEGRA210_MIXER_ALT)	+= snd-soc-tegra210-alt-mixer.o
obj-$(CONFIG_SND_SOC_TEGRA210_SFC_ALT)		+= snd-soc-tegra210-alt-sfc.o
obj-$(CONFIG_SND_SOC_TEGRA210_AFC_ALT)		+= snd-soc-tegra210-alt-afc.o
obj-$(CONFIG_SND_SOC_TEGRA210_MVC_ALT)		+= snd-soc-tegra210-alt-mvc.o
obj-$(CONFIG_SND_SOC_TEGRA210_IQC_ALT)		+= snd-soc-tegra210-alt-iqc.o
obj-$(CONFIG_SND_SOC_TEGRA210_OPE_ALT)		+= snd-soc-tegra210-alt-ope.o
obj-$(CONFIG_SND_SOC_TEGRA210_ADSP_ALT)		+= snd-soc-tegra210-alt-adsp.o
obj-$(CONFIG_SND_SOC_TEGRA186_ARAD_ALT)		+= snd-soc-tegra186-alt-arad.o
obj-$(CONFIG_SND_SOC_TEGRA186_ASRC_ALT)		+= snd-soc-tegra186-alt-asrc.o
obj-$(CONFIG_SND_SOC_TEGRA186_DSPK_ALT)		+= snd-soc-tegra186-alt-dspk.o

#------------------------------------ machine drivers ---------------------------------------
snd-soc-tegra-alt-p1889ref-objs			:= machine_drivers/tegra_p1889ref_alt.o
snd-soc-tegra-alt-t210ref-objs			:= machine_drivers/tegra_t210ref_alt.o
snd-soc-tegra-alt-t186ref-objs			:= machine_drivers/tegra_t186ref_alt.o
snd-soc-tegra-machine-driver-mobile-objs	:= machine_drivers/tegra_machine_driver_mobile.o
snd-soc-tegra-alt-t186ref-p2382-objs		:= machine_drivers/tegra_t186ref_p2382_alt.o
snd-soc-tegra-alt-t186ref-bali-objs		:= machine_drivers/tegra_t186ref_bali_alt.o
snd-soc-tegra-alt-t186ref-boden-objs		:= machine_drivers/tegra_t186ref_boden_alt.o
snd-soc-tegra-alt-t186ref-p4573-objs		:= machine_drivers/tegra_t186ref_p4573_alt.o
snd-soc-tegra-alt-t210ref-p2382-objs		:= machine_drivers/tegra_t210ref_p2382_alt.o \
                                                	machine_drivers/tegra_maui_alt.o

obj-$(CONFIG_SND_SOC_TEGRA_P1889REF_ALT)	+= snd-soc-tegra-alt-p1889ref.o
obj-$(CONFIG_SND_SOC_TEGRA_T210REF_ALT)		+= snd-soc-tegra-alt-t210ref.o
obj-$(CONFIG_SND_SOC_TEGRA_T210REF_P2382_ALT)	+= snd-soc-tegra-alt-t210ref-p2382.o
obj-$(CONFIG_SND_SOC_TEGRA_T186REF_FPGA_ALT)	+= snd-soc-tegra-alt-t186ref.o
obj-$(CONFIG_SND_SOC_TEGRA210_AUDIO_ALT)          += snd-soc-tegra-machine-driver-mobile.o
obj-$(CONFIG_SND_SOC_TEGRA_T186REF_AUTO_ALT)	+= snd-soc-tegra-alt-t186ref-p2382.o
obj-$(CONFIG_SND_SOC_TEGRA_T186REF_AUTO_ALT)	+= snd-soc-tegra-alt-t186ref-bali.o
obj-$(CONFIG_SND_SOC_TEGRA_T186REF_AUTO_ALT)	+= snd-soc-tegra-alt-t186ref-boden.o
obj-$(CONFIG_SND_SOC_TEGRA_T186REF_P4573_ALT)	+= snd-soc-tegra-alt-t186ref-p4573.o
