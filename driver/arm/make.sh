cp ../sv_driver.c ../sv_driver.h ../support_funcs.c ./
#make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- 
make ARCH=arm CROSS_COMPILE=arm-xilinx-linux-gnueabi-
