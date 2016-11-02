dtb-$(CONFIG_ARCH_VEXPRESS) += rtsm_ve-aemv8a.dtb foundation-v8.dtb \
				fvp-base-gicv2-psci.dtb
dtb-$(CONFIG_ARCH_XGENE) += apm-mustang.dtb
dtb-$(CONFIG_ARCH_VEXPRESS) += juno.dtb
dtb-$(CONFIG_ARCH_MESON64_ODROIDC2) += meson64_odroidc2.dtb \
				vega_s95_telos.dtb vega_s95_meta.dtb vega_s95_pro.dtb \
				spot_i7.dtb acemax_kiii.dtb ki_plus.dtb \
				iconbit.dtb mini_m8s.dtb mk808b_pro.dtb mxv.dtb \
				gxl_p212_1g.dtb gxl_p212_2g.dtb \
				gxl_p230_2g.dtb gxl_p231_1g.dtb gxl_p231_2g.dtb \
				mxq_pro_4k.dtb mxq_pro_plus.dtb mini_mx.dtb no_emmc.dtb

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
