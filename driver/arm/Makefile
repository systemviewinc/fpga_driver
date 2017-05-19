obj-m += vsi_driver.o
vsi_driver-y += sv_driver.o support_funcs.o libxdma.o
#KDIR := ~/DF_vivado/zynq_linux/linux-xlnx
#KDIR := ~/petalinux/Xilinx-ZCU102-2016.2/build/linux/kernel/xlnx-4.4
#KDIR := ~/petalinux/mz_7020_2016_2/build/linux/kernel/xlnx-4.6/
KDIR := ~/enclustra/bsp-xilinx/sources/xilinx-linux/
PWD := $(shell pwd)

all:
	make -C $(KDIR) M=$(PWD) modules
clean:
	make -C $(KDIR) M=$(PWD) clean

