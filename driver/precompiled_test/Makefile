obj-m += vsi_driver.o
vsi_driver-y += vsi_driver_bin.o

all: 
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
