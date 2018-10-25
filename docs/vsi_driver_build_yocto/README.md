## Instructions to build the driver for petalinux /yocto:

### Petalinux Flow
*These instructions assume you have already built a petalinux project for your system*

Navigate to the petalinux project and source the petalinux settings.

##### Create a template module for your petalinux project.

		$ petalinux-create --type modules --name vsidriver

**The name must be lowercase and must not contain special characters, we suggest you call is vsidriver**

Within the created module template make a symbolic link to driver files

		$ rm <name>.c
		$ rm Makefile
		$ mkdir xdma
		$ ln -s ~/<<VSI_FPGA_DIR>>/driver/*.c .
		$ ln -s ~/<<VSI_FPGA_DIR>>/driver/*.h .
		$ ln -s ~/<<VSI_FPGA_DIR>>/driver/Makefile .
		$ ln -s ~/<<VSI_FPGA_DIR>>/xdma/*.c xdma/.
		$ ln -s ~/<<VSI_FPGA_DIR>>r/xdma/*.h xdma/.


Modify the *recipe_name.bb* as shown below (or use [vsidriver.bb](vsi_driver.bb) as and example):

		SRC_URI = "file://Makefile \
				file://*.c \
				file://*.h \
				file://xdma/*.c \
				file://xdma/*.h \
				file://COPYING \
			"

And append the following to the end:

		inherit module
		EXTRA_OEMAKE = 'KERNEL_SRC="${STAGING_KERNEL_DIR}" \
					O=${STAGING_KERNEL_BUILDDIR} \
					'


Clean, build, install the module and build petalinux:

		$ petalinux-build -c vsidriver -x do_clean
		$ petalinux-build -c vsidriver -x do_build
		$ petalinux-build -c vsidriver -x do_install

The driver will now be in the filesystem at:

		/lib/modules/<version>-xilinx/extra/vsi_driver.ko


Petalinux 2016.4 module build reference - https://www.xilinx.com/support/answers/68441.html
