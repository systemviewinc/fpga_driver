obj-m += vsi_driver.o
vsi_driver-y +=  xdma/sv_xdma.o xdma/xdma-core.o xdma/xdma-sgm.o support_funcs.o sv_streaming_funcs.o  sv_driver.o

CFLAGS_xdma-core.o := -DDEBUG
CFLAGS_xdma-sgm.o := -DDEBUG
CFLAGS_sv_xdma.o := -DDEBUG
CFLAGS_support_funcs.o := -DDEBUG

#KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build

all:
	make -C ${KERNEL_SRC} M=$(PWD)
modules_install:
	make -C ${KERNEL_SRC} M=$(PWD) modules_install
clean:
	make -C ${KERNEL_SRC} M=$(PWD) clean
