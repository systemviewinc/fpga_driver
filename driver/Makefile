obj-m += vsi_driver.o
vsi_driver-y += support_funcs.o sv_driver.o libxdma.o

KERNEL_SOURCES ?= /lib/modules/$(shell uname -r)/build

all:
	make -C ${KERNEL_SOURCES} M=$(PWD) modules

clean:
	make -C ${KERNEL_SOURCES} M=$(PWD) clean
