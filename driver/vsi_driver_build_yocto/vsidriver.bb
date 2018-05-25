SUMMARY = "Recipe for  build an external vsidriver Linux kernel module"
SECTION = "PETALINUX/modules"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=12f884d2ae1ff87c09e5b7ccc2c4ca7e"

inherit module

SRC_URI = "file://Makefile \
	   file://COPYING \
           file://support_funcs.c \
           file://sv_driver.c \
           file://sv_driver.h \
           file://sv_streaming_funcs.c \
           file://xdma/perfmon_parameters.h \
           file://xdma/sv_xdma.c \
           file://xdma/sv_xdma.h \
           file://xdma/version.h \
           file://xdma/xbar_sys_parameters.h \
           file://xdma/xdma-core.c \
           file://xdma/xdma-core.h \
           file://xdma/xdma-ioctl.c \
           file://xdma/xdma-ioctl.h \
           file://xdma/xdma-sgm.c \
           file://xdma/xdma-sgm.h \
          "

S = "${WORKDIR}"

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.
