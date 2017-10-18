SUMMARY = "VSI Linux kernel module"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=12f884d2ae1ff87c09e5b7ccc2c4ca7e"

inherit module

SRC_URI = "file://Makefile \
           file://sv_driver.c \
           file://libxdma_api.h  \
           file://libxdma.c \
           file://libxdma.h \
           file://support_funcs.c \
           file://sv_driver.h \
           file://COPYING \
          "

S = "${WORKDIR}"

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.
