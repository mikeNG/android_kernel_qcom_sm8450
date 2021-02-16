ifeq ($(CONFIG_BUILD_ARM64_DT_OVERLAY), y)
dtbo-$(CONFIG_ARCH_WAIPIO)		+= waipio-camera.dtbo
dtbo-$(CONFIG_ARCH_WAIPIO)		+= waipio-camera-sensor-mtp.dtbo \
										waipio-camera-sensor-cdp.dtbo \
										waipio-camera-sensor-qrd.dtbo
else
$(error CONFIG_BUILD_ARM64_DT_OVERLAY is: $(CONFIG_BUILD_ARM64_DT_OVERLAY))
endif

always-y	:= $(dtbo-y) $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb *.dtbo