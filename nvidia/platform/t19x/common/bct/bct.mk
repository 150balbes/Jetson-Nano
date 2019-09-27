BCT_FILES_PATH := hardware/nvidia/platform/t19x/common/bct

PRODUCT_COPY_FILES += \
    $(BCT_FILES_PATH)/device/tegra19x-mb1-bct-device-sdmmc.cfg:tegra19x-mb1-bct-device-sdmmc.cfg \
    $(BCT_FILES_PATH)/device/tegra19x-mb1-bct-device-qspi.cfg:tegra19x-mb1-bct-device-qspi.cfg \
    $(BCT_FILES_PATH)/device/tegra19x-mb1-bct-device-ufs.cfg:tegra19x-mb1-bct-device-ufs.cfg \
    $(BCT_FILES_PATH)/device/tegra19x-mb1-bct-device-sata.cfg:tegra19x-mb1-bct-device-sata.cfg \
    $(BCT_FILES_PATH)/gpio-intmap/tegra194-mb1-bct-gpio-int-to-all-int0.cfg:tegra194-mb1-bct-gpio-int-to-all-int0.cfg	\
    $(BCT_FILES_PATH)/uphy-lane/tegra194-mb1-bct-uphy-lane-ufs-x1-lane10.cfg:tegra194-mb1-bct-uphy-lane-ufs-x1-lane10.cfg    \
    $(BCT_FILES_PATH)/uphy-lane/tegra194-mb1-bct-uphy-lane-ufs-x1-lane11.cfg:tegra194-mb1-bct-uphy-lane-ufs-x1-lane11.cfg    \
    $(BCT_FILES_PATH)/uphy-lane/tegra194-mb1-bct-uphy-lane-ufs-x2.cfg:tegra194-mb1-bct-uphy-lane-ufs-x2.cfg  \
    $(BCT_FILES_PATH)/uphy-lane/tegra194-mb1-bct-uphy-lane-sata.cfg:tegra194-mb1-bct-uphy-lane-sata.cfg      \
    $(BCT_FILES_PATH)/uphy-lane/tegra194-mb1-bct-uphy-lane-ufs-x1-sata.cfg:tegra194-mb1-bct-uphy-lane-ufs-x1-sata.cfg \
    $(BCT_FILES_PATH)/scr/tegra194-mb1-bct-scr-cbb-mini.cfg:tegra194-mb1-bct-scr-cbb-mini.cfg \
    $(BCT_FILES_PATH)/scr/tegra194-mb1-bct-scr-cbb-mods.cfg:tegra194-mb1-bct-scr-cbb-mods.cfg
