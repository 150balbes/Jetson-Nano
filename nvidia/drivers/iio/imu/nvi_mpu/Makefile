#
# Makefile for NVidia Invensense nvi-mpu device.
#

obj-$(CONFIG_NVI_MPU_IIO)	+= nvi-mpu.o
obj-$(CONFIG_NVI_MPU_INPUT)	+= nvi-mpu.o
obj-$(CONFIG_NVI_MPU_RELAY)	+= nvi-mpu.o

ifneq (,$(filter $(CONFIG_NVI_MPU_IIO),y m))
CFLAGS_nvi.o			+= -DNVS_CFG_KIF=NVS_KIF_IIO
CFLAGS_nvi.o			+= -Idrivers/iio
CFLAGS_nvi_icm.o		+= -Idrivers/iio
CFLAGS_nvi_mpu.o		+= -Idrivers/iio
CFLAGS_nvi_dmp_icm.o		+= -Idrivers/iio
CFLAGS_nvi_dmp_mpu.o		+= -Idrivers/iio
endif

ifneq (,$(filter $(CONFIG_NVI_MPU_INPUT),y m))
CFLAGS_nvi.o			+= -DNVS_CFG_KIF=NVS_KIF_INPUT
endif

ifneq (,$(filter $(CONFIG_NVI_MPU_RELAY),y m))
CFLAGS_nvi.o			+= -DNVS_CFG_KIF=NVS_KIF_RELAY
endif

nvi-mpu-objs := nvi.o
nvi-mpu-objs += nvi_icm.o
nvi-mpu-objs += nvi_mpu.o
nvi-mpu-objs += nvi_dmp_icm.o
nvi-mpu-objs += nvi_dmp_mpu.o

# Uncomment next line to enable TRACING
# CFLAGS_nvi.o			+= -DENABLE_TRACE -DCREATE_TRACE_POINTS
#
