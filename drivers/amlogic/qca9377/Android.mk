# Android makefile for the WLAN Module

# Build/Package only in case of supported target
ifneq ($(filter imx6 imx7,$(TARGET_BOARD_PLATFORM)),)

LOCAL_PATH := $(call my-dir)
LOCAL_PATH_BACKUP := $(ANDROID_BUILD_TOP)/$(LOCAL_PATH)

CROSS_COMPILE=$(ANDROID_BUILD_TOP)/prebuilts/gcc/linux-x86/arm/arm-eabi-4.8/bin/arm-eabi-

MAKE_OPTIONS := ARCH=arm
MAKE_OPTIONS += CROSS_COMPILE=$(CROSS_COMPILE)
MAKE_OPTIONS += WLAN_ROOT=$(LOCAL_PATH_BACKUP)
MAKE_OPTIONS += MODNAME=wlan
MAKE_OPTIONS += WLAN_OPEN_SOURCE=1
MAKE_OPTIONS += CONFIG_CLD_HL_SDIO_CORE=y
MAKE_OPTIONS += CONFIG_QCA_WIFI_ISOC=0
MAKE_OPTIONS += CONFIG_QCA_WIFI_2_0=1
MAKE_OPTIONS += CONFIG_QCA_CLD_WLAN=m

KERNEL_SRC=$(ANDROID_BUILD_TOP)/kernel_imx

include $(CLEAR_VARS)
LOCAL_MODULE       := qcacld_wlan.ko
LOCAL_MODULE_PATH  := $(TARGET_OUT)/lib/modules/
LOCAL_MODULE_CLASS := ETC
LOCAL_MODULE_TAGS  := optional
include $(BUILD_PREBUILT)

QCACLD_INTERMEDIATES := $(TARGET_OUT_INTERMEDIATES)/$(LOCAL_MODULE_CLASS)/$(LOCAL_MODULE)_intermediates

# Override the default build target in order to issue our own custom command.
# Note that the module name is wlan.ko by default, we then change it to
# qcacld_wlan.ko in order to be more explicit.
$(LOCAL_BUILT_MODULE): $(TARGET_PREBUILT_KERNEL)
	$(MAKE) -C $(KERNEL_SRC) M=$(LOCAL_PATH_BACKUP) $(MAKE_OPTIONS) modules
	$(hide) $(CROSS_COMPILE)strip --strip-debug $(LOCAL_PATH_BACKUP)/wlan.ko
	$(hide) mkdir -p $(QCACLD_INTERMEDIATES)
	$(hide) $(ACP) $(LOCAL_PATH_BACKUP)/wlan.ko $(QCACLD_INTERMEDIATES)/qcacld_wlan.ko

endif
