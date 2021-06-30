#
# Makefile for industrial I/O Magnetometer sensor drivers
#

# When adding new entries keep the list in alphabetical order

obj-$(CONFIG_NVI_AK89XX) += nvi-ak89xx.o
# Uncomment next line to enable TRACING
# CFLAGS_nvi_ak89xx.o     += -DENABLE_TRACE
#
ifneq (,$(filter $(CONFIG_NVI_AK89XX),m))
# Uncomment next line to enable TRACING
# CFLAGS_nvi_ak89xx.o     += -DCREATE_TRACE_POINTS
#
endif

ifneq (,$(filter $(CONFIG_NVI_MPU_IIO),y m))
	CFLAGS_nvi_ak89xx.o += -DAKM_NVI_MPU_SUPPORT=1
endif
ifneq (,$(filter $(CONFIG_NVI_MPU_INPUT),y m))
	CFLAGS_nvi_ak89xx.o += -DAKM_NVI_MPU_SUPPORT=1
endif
ifneq (,$(filter $(CONFIG_NVI_MPU_RELAY),y m))
	CFLAGS_nvi_ak89xx.o += -DAKM_NVI_MPU_SUPPORT=1
endif
nvi-ak89xx-objs += nvi_ak89xx.o
