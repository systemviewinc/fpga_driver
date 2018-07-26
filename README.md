# Visual System Integrator FPGA Driver

## Table of Contents
1. [VSI Overview](#overview)
2. [Getting Started](#gettingstarted)
    - [Building the driver](#build)

<a name="overview"></a>
## VSI Overview

* [Visual System Integrator (VSI)](https://systemviewinc.com/) is a development tool for creating complete heterogeneous systems intuitively easy to use graphical environment.  Built on top of Vivado VSI has complete support for all FPGAs supported by Vivado, multiple different processors and other peripheral processing units such as the R5. At the heat of FPGA to/from CPU data transfer lies our FPGA driver.

<a name="gettingstarted"></a>
## Getting Started

<a name="build"></a>
#### Building the driver

The VSI FPGA driver repo can be cloned to your host or target computer, depending on the application.

    $ git clone https://github.com/systemviewinc/fpga_driver $VSI_FPGA_REPO_DIR
    $ cd $VSI_FPGA_REPO_DIR/driver
    $ make

###### For cross compiling the driver.

    $ make ARCH=arm CROSS_COMPILE=<<your cross compiler>> KERNEL_SRC=<<directory to kernel headers>>

###### For Petalinux / Yocto built kernels

[Building kernel for Petalinux / Yocto](docs/vsi_driver_build_yocto/README.md)
