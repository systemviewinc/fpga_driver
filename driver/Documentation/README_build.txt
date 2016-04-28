To build for PCIe : 

	for single  PCIe card: 

	- on host machine,  cd into ../sv_driver/driver/  and run make.sh
		- rename either vsi_driver_1.ko or vsi_driver_2.ko to match your insmod script.

	for dual PCIe cards (two driver builds)

	- on host machine,  cd into ../sv_driver/driver/  and run make.sh
		- This will create two instances of the driver, vsi_driver_1 and vsi_driver_2

To build for ARM : 

	- source /opt/Xilinx/Vivado/....

	- cd into ../sv_driver/driver/arm and modify Makefile to point to your Xilinx Linux repo
	- cd into ../sv_driver/driver/arm and run make.sh
		
