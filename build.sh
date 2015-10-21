#!/bin/sh

DEFCONFIG=studio5_defconfig
KERNEL=zImage-dtb

if [ "$CROSS_COMPILE" = "" ]; then
echo "Define absolute path for your toolchain"
read input
for input
do export CROSS_COMPILE=$input
done
fi

make ARCH=arm $DEFCONFIG

if [ "$CROSS_COMPILE" = "" ]; then
make -j4 $CROSS_COMPILE $KERNEL
else
make -j4 $KERNEL
fi
