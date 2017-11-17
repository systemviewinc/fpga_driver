obj-m += vsi_driver.o
vsi_driver-y += support_funcs.o sv_driver.o libxdma.o

all: 
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
