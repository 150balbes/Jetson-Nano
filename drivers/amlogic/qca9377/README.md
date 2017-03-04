qcacld-2.0
==========

Boundary Devices changes to [CodeAurora qcacld-2.0 repository][codeaurora].

This code has only been tested againt the following Boundary Devices kernel branches:
* [boundary-imx\_3.14.52\_1.1.0\_ga kernel branch][branch-3.14.x]
* [boundary-imx\_4.1.15\_1.0.0\_ga kernel branch][branch-4.1.x]

Build instructions
------------------

**1. Download the source code**
```
$ cd
$ git clone https://github.com/boundarydevices/qcacld-2.0 -b boundary-LNX.LEH.4.2.2.2-4.5.20.034
$ cd qcacld-2.0/
```

**2. Setup the environment**
* Assuming you are using `gcc-arm-linux-gnueabihf` toolchain available for Debian/Ubuntu
* If not, please make sure to specify the proper toolchain
```
$ export ARCH=arm
$ export CROSS_COMPILE=arm-linux-gnueabihf-
```

**3. Build the module**
* `<kernel_path>` must be replaced with the actual path of kernel source code
```
$ KERNEL_SRC=<kernel_path> CONFIG_CLD_HL_SDIO_CORE=y make
```

**4. Install the module**
* `<rootfs_path>` must be replaced with the actual path of the target root file-system
* It can either be on your drive (NFS) or directly on an SD card
```
$ KERNEL_SRC=<kernel_path> INSTALL_MOD_PATH=<rootfs_path> make modules_install
```

[codeaurora]: https://source.codeaurora.org/quic/la/platform/vendor/qcom-opensource/wlan/qcacld-2.0/ "CodeAurora qcacld-2.0"
[branch-3.14.x]: https://github.com/boundarydevices/linux-imx6/tree/boundary-imx_3.14.52_1.1.0_ga "boundary-imx_3.14.52_1.1.0_ga kernel branch"
[branch-4.1.x]: https://github.com/boundarydevices/linux-imx6/tree/boundary-imx_4.1.15_1.0.0_ga "boundary-imx_4.1.15_1.0.0_ga kernel branch"
