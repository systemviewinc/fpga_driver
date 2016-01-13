obj-m += sv_driver.o
sv_driver-y += vsi_driver_bin.o


KDIR := ~/Desktop/repos/linux-xlnx/
PWD := $(shell pwd)

all:
	make -C $(KDIR) M=$(PWD) modules
clean:
	make -C $(KDIR) M=$(PWD) clean

