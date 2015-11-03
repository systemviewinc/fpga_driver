/*************************************
 *
 *************************************/

#ifndef __SPCICOMMON_H_
#define __SPCICOMMON_H_

/*register address size*/
//#define REG_SIZE (1024*1024)
/*MAX length of DMA buffer*/
//#define BUF_SIZE (1024*4096)

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

/****************** struct for pio **********************/
//struct register_struct{
//	int reg;
//	unsigned int value;
//};

/****************** struct for zero copy dma **********************/
//struct zero_copy_dma_req_struct{
//	int offset;
//	unsigned int count;
//};

#endif
