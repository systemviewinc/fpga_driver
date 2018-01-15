obj-m += vsi_driver.o
vsi_driver-y += support_funcs.o sv_xdma.o sv_driver.o  xdma-core.o xdma-sgm.o

#CFLAGS_xdma-core.o := -DDEBUG
#CFLAGS_xdma-sgm.o := -DDEBUG
#CFLAGS_sv_xdma.o := -DDEBUG


all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
