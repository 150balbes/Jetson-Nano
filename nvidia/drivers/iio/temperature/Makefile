obj-$(CONFIG_NVS_TMP1X2_IIO)	+= nvs_tmp1x2.o
obj-$(CONFIG_NVS_TMP1X2_INPUT)	+= nvs_tmp1x2.o
obj-$(CONFIG_NVS_TMP1X2_RELAY)	+= nvs_tmp1x2.o

ifneq (,$(filter $(CONFIG_NVS_TMP1X2_IIO),y m))
CFLAGS_nvs_tmp1x2.o += -DNVS_CFG_KIF=NVS_KIF_IIO
endif

ifneq (,$(filter $(CONFIG_NVS_TMP1X2_INPUT),y m))
CFLAGS_nvs_tmp1x2.o += -DNVS_CFG_KIF=NVS_KIF_INPUT
endif

ifneq (,$(filter $(CONFIG_NVS_TMP1X2_RELAY),y m))
CFLAGS_nvs_tmp1x2.o += -DNVS_CFG_KIF=NVS_KIF_RELAY
endif

