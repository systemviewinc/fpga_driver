obj-m += vsi_driver.o
vsi_driver-y += sv_driver.o support_funcs.o
#KDIR := ../../linux-xlnx/
#KDIR := ~/Desktop/repos/linux-xlnx-debug/
#KDIR := ~/Desktop/repos/linux-xlnx/
#KDIR := ~/DF_vivado/zynq_linux/linux-xlnx
KDIR := ~/petalinux/MPSoc_Boot/build/linux/kernel/xlnx-4.0
PWD := $(shell pwd)

all:
	make -C $(KDIR) M=$(PWD) modules
clean:
	make -C $(KDIR) M=$(PWD) clean

