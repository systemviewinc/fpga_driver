obj-m += vsi_driver.o
vsi_driver-y += sv_driver.o support_funcs.o libxdma.o

all:
	make -C $(KERNEL_SRC) M=$(CURDIR)
modules_install:
	make -C $(KERNEL_SRC) M=$(CURDIR) modules_install
clean:
	make -C $(KERNEL_SRC) M=$(CURDIR) clean
