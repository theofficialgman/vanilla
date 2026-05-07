#!/bin/bash

set -e

# Main build
make -C /buildroot O=/build

# Copy initramfs
mkdir -p /install/bootloader/ini
mkdir -p /install/switchroot/vanilla/data
mv /build/images/rootfs.cpio.uboot /install/switchroot/vanilla/initramfs

# Copy icons
cp /vanilla/gui/res/switch/bootlogo_vanilla.bmp /install/switchroot/vanilla/
cp /vanilla/gui/res/switch/icon_vanilla.bmp /install/switchroot/vanilla/

# Copy bootloader ini
cp /vanilla/gui/res/switch/vanilla.ini /install/bootloader/ini/

# Create U-Boot image of kernel
/build/host/bin/mkimage -A arm64 -O linux -T kernel -C none -a 0x80200000 -e 0x80200000 -d /build/images/Image /install/switchroot/vanilla/uImage

# Copy pre-made config
cp /vanilla/gui/res/switch/config.xml /install/switchroot/vanilla/data/

# Copy support files
cp /vanilla/gui/res/switch/bl31.bin /install/switchroot/vanilla/
cp /vanilla/gui/res/switch/bl33.bin /install/switchroot/vanilla/
cp /vanilla/gui/res/switch/boot.scr /install/switchroot/vanilla/
cp /vanilla/gui/res/switch/nx-plat.dtimg /install/switchroot/vanilla/
