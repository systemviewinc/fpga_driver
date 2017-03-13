obj-m += vsi_driver.o
vsi_driver-y += sv_driver.o support_funcs.o
#KDIR := ~/DF_vivado/zynq_linux/linux-xlnx
#KDIR := ~/petalinux/Xilinx-ZCU102-2016.2/build/linux/kernel/xlnx-4.4
#KDIR := ~/petalinux/project_1/build/linux/kernel/xlnx-4.4/
KDIR := /lib/modules/`uname -r`/build

PWD := $(shell pwd)

all:
	make -C $(KDIR) M=$(PWD) modules
clean:
	make -C $(KDIR) M=$(PWD) clean

