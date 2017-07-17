ifndef ARCH
	#assume being built for x86 machine if ARCH is not defined
	ARCH=x86
endif
obj-m += vsi_driver.o
vsi_driver-y += sv_driver.o support_funcs.o

KDIR := ~/petalinux/project_1/build/linux/kernel/xlnx-4.4/
#KDIR := /lib/modules/`uname -r`/build

PWD += $(shell pwd)
export ARCH

all:
	@make -C $(KDIR) M=$(PWD) modules; \
	if [ $(ARCH) = x86 ]; then \
		cp vsi_driver.ko vsi_driver_1.ko; \
		mv vsi_driver.ko vsi_driver_2.ko; \
	fi
clean:
	@make -C $(KDIR) M=$(PWD) clean
