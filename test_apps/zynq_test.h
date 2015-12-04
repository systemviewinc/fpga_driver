
#include <sys/wait.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>

//#define DEV_NAME "/dev/pci_skel"
//#define DEV_NAME_2 "/dev/pci_skel_2"
//#define DEV_NAME_3 "/dev/pci_skel_3"
//#define DEV_NAME_4 "/dev/pci_skel_4"
//#define DEV_NAME_5 "/dev/pci_skel_5"

#define DEV_NAME "/dev/bram"
#define DEV_NAME_2 "/dev/vsi_system_vc709_packet_p_arg_1_seq_i"

#ifndef __SPCICOMMON_H_
#define __SPCICOMMON_H_

/*enum for ioctl*/
/*enum{	
	SET_AXI_DEVICE,
	SET_AXI_CDMA,
	RESERVED,
	SET_AXI_PCIE_CTL,
	SET_AXI_PCIE_M,
	SET_AXI_INT_CTRL,
	SET_AXI_DEV_SI,
	SET_AXI_DEV_M,
	CLEAR_AXI_INTERRUPT_CTLR,
	SET_CDMA_KEYHOLE_WRITE,
	SET_CDMA_KEYHOLE_READ
};*/

#endif
