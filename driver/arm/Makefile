obj-m += sv_driver.o
KDIR := ~/Desktop/repos/linux-xlnx/
PWD := $(shell pwd)

all:
	make -C $(KDIR) M=$(PWD) modules
clean:
	make -C $(KDIR) M=$(PWD) clean

