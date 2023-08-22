This repository contains the source code for the UWB stack on Android

# Repository organisation

The following layout is used:

- kernel: all the Linux specific code
  - kernel/drivers: the DW3000 driver
  - kernel/net/mcps802154: the Linux specific code for the mcps and symbolic
                           links to the general purpose MAC code

- mac: the general purpose MAC layer code, to be used in various targets
       including the Linux kernel

# HOWTO

## SETUP your target

:warning: IMPORTANT Please note that the following section describes how to setup a build for the
db845c Dragonboard target and must be adapted to your own system.

1. Copy config/sdm845-db845c-uwb.dts in arch/arm64/boot/dts/qcom/sdm845-db845c-uwb.dts to support dw3000 device in your Device Tree (DT)

**Please note** the following key points:

- the SPI controller driver is now an external module named
spi-geni-qcom-lowlatency. (for DB)
- the dw3000@0 is put inside the &spi2 node.
- interrupts must be changed to match the GPIO where the dw3000 IRQ pin is
connected.
- uwbhal,reset-gpio = <&tlmm 50 GPIO_ACTIVE_LOW> must be change to match the
GPIO where the dw3000 reset pin is connected.
- uwbhal,affinity must be set to the cpumask where to put our own threaded
softirq and SPI controller thread to achieve good performance.
- dw3000_default_state must be change according required GPIO configuration.
- compatible = "decawave,dw3000" is the module alias used by our SPI driver
so the ko is automatically loaded

2. Copy db845c-uwb_defconfig in arch/arm64/configs/db845c-uwb_defconfig

3. Once your target device tree is configured, create a new build.config to
include our requirements for the kernel configuration.
Like config/build.config.db845c

For example on the Dragonboard db845c's:

```
KERNEL_DIR=common
. ${ROOT_DIR}/common/build.config.common
. ${ROOT_DIR}/common/build.config.aarch64

DEFCONFIG=db845c-uwb_defconfig

MAKE_GOALS="${MAKE_GOALS}
qcom/sdm845-db845c-uwb.dtb
Image.gz
"

FILES="
arch/arm64/boot/Image.gz
arch/arm64/boot/dts/qcom/sdm845-db845c-uwb.dtb
"
```

4. Ensure that you kernel is built with the following options:

- Enable GPIOLIB support

```
CONFIG_GPIOLIB=y
```

- Enable IEEEE802154 support

```
CONFIG_6LOWPAN=m
CONFIG_IEEE802154=m
CONFIG_IEEE802154_NL802154_EXPERIMENTAL=y
CONFIG_IEEE802154_6LOWPAN=m
CONFIG_MAC802154=m
```

## BUILD

### Overview

Qorvo UWB driver can be built as an external module using the
common-android-mainline kernel manifest.

It must be passed as prebuilt kernel to the AOSP build script to generate a boot
image ready to be flashed.

### Build kernel and modules

If the uwb-release project is installed in <UWB_RELEASE_PATH>, run from your
kernel source build:

```bash
EXT_MODULES=${UWB_RELEASE_PATH}/kernel ./build/build.sh
```

For the dragonboard db845c, using the android 5.9 stable kernel,
`out/android-mainline/dist/` contains the kernel and its modules.

### Build images for the dragonboard db845c

1. Install the content of the previously built kernel folder into your device
kernel prebuilt location.

For the dragonboard db845c, using the android 5.9 stable kernel, prebuit kernel
is located in the target repository inside AOSP sources:

`device/linaro/dragonboard-kernel/android-5.9`

2. Enable Permissive SELinux

Our SELinux rules have not yet been written for this release,
for now enable the permissive mode:

In BoardConfig.mk:

```
BOARD_KERNEL_CMDLINE += androidboot.selinux=permissive
```

3. Add Qorvo vendor modules

Copy all the content of platform_vendor_qorvo in `vendor/qorvo`.

In your device.mk, add the vendor extension:
```
$(call inherit-product-if-exists, vendor/qorvo/device.mk)
```

4. Then recompile the AOSP sources.

:warning: IMPORTANT Please ensure that the dw3000 module is loaded on startup
after the following dependencies:

- spi-geni-qcom-lowlatency (or your board driver)
- mac802154
- mcps802154

```bash
. build/envsetup.sh
lunch db845c-userdebug
make
```

## Testing

once you flashed yout android boards, you can launch a fira ranging with the following commands on the boards : 

~~~
service call uwb 10 i32 0
haluwbctl start_fira_twr_session -a 1 -d 2
~~~

~~~
service call uwb 10 i32 0
haluwbctl start_fira_twr_session -a 1 -d 2 --responder --controlee
~~~

To find out more : `haluwbctl --help`


