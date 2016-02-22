obj-m += vsi_driver.o
vsi_driver-y += sv_driver.o support_funcs.o
#KDIR := ../../linux-xlnx/
#KDIR := ~/Desktop/repos/linux-xlnx-debug/
KDIR := ~/Desktop/repos/linux-xlnx/
PWD := $(shell pwd)

all:
	make -C $(KDIR) M=$(PWD) modules
clean:
	make -C $(KDIR) M=$(PWD) clean

