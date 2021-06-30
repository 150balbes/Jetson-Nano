obj-$(CONFIG_NVS_BMI160_IIO)	+= nvs_bmi160.o
obj-$(CONFIG_NVS_BMI160_INPUT)	+= nvs_bmi160.o
obj-$(CONFIG_NVS_BMI160_RELAY)	+= nvs_bmi160.o

ifneq (,$(filter $(CONFIG_NVS_BMI160_IIO),y m))
CFLAGS_nvs_bmi160.o += -DNVS_CFG_KIF=NVS_KIF_IIO
endif

ifneq (,$(filter $(CONFIG_NVS_BMI160_INPUT),y m))
CFLAGS_nvs_bmi160.o += -DNVS_CFG_KIF=NVS_KIF_INPUT
endif

ifneq (,$(filter $(CONFIG_NVS_BMI160_RELAY),y m))
CFLAGS_nvs_bmi160.o += -DNVS_CFG_KIF=NVS_KIF_RELAY
endif

