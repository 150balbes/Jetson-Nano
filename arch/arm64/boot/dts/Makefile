dtb-$(CONFIG_ARM64) += gxbb_p200.dtb gxbb_p200_2G.dtb \
				gxl_p212_1g.dtb gxl_p212_2g.dtb \
				gxbb_p201.dtb odroidc2.dtb \
				gxbb_p200_1G_wetek_hub.dtb gxbb_p200_2G_wetek_play_2.dtb \
				gxl_p230_2g.dtb meson64_odroidc2.dtb \
				gxl_p231_1g.dtb gxl_p231_2g.dtb \
				gxbb_p200_2G_vegas95.dtb kvim.dtb \
				gxm_q200_2g.dtb gxm_q200_3g.dtb \
				gxm_q201_1g.dtb gxm_q201_2g.dtb \
				vega_s96.dtb beelink_gt1_g2.dtb

dtb-$(CONFIG_ARCH_MESON64_ODROIDC2) += meson64_odroidc2.dtb

targets += dtbs

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif
targets += $(DTB_LIST)

dtbs: $(addprefix $(obj)/, $(DTB_LIST))

clean-files := dts/*.dtb *.dtb
