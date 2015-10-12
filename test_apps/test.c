#include <stdio.h>
#include "test.h"
#include <pthread.h>
#include <sys/poll.h>

#define AXI_STREAM_FIFO 1

void *rxfifo_read(void *read_buf);

char devname[] = DEV_NAME;
char devname_2[] = DEV_NAME_2;
char devname_3[] = DEV_NAME_3;

int bram = -1;
int hls_write = -1;
int hls_read = -1;

char * devfilename = devname;
char * devfilename_2 = devname_2;
char * devfilename_3 = devname_3;

//unsigned int bram_axi_addr = 0x00000000;
//unsigned int hls_write_ctl_axi_addr = 0x00005000;
//unsigned int hls_read_ctl_axi_addr = 0x00007000;
//unsigned int hls_write_axi_addr = 0x00020000;
//unsigned int hls_read_axi_addr = 0x00010000;

unsigned int bram_axi_addr = 0x50000000;
unsigned int hls_write_ctl_axi_addr = 0x50010000;
unsigned int hls_read_ctl_axi_addr = 0x50030000;
unsigned int hls_write_axi_addr = 0x50020000;
unsigned int hls_read_axi_addr = 0x50040000;


unsigned int cdma_addr = 0x00001000;
unsigned int pcie_ctl_addr = 0x00002000;
unsigned int pcie_m_addr = 0x00003000;
unsigned int axi_int_addr = 0x00004000;
unsigned int in[8];

int hls_fifo_mode = AXI_STREAM_FIFO;

unsigned int SET_AXI_DEVICE = 50;
unsigned int SET_AXI_CDMA = 51;
unsigned int SET_AXI_PCIE_CTL = 52;
unsigned int SET_AXI_PCIE_M = 53;
unsigned int SET_AXI_INT_CTRL = 54;
unsigned int SET_AXI_DEV_SI = 55;
unsigned int SET_AXI_DEV_M = 56;
unsigned int CLEAR_AXI_INTERRUPT_CTLR = 60;
unsigned int SET_CDMA_KEYHOLE_WRITE = 58;
unsigned int SET_CDMA_KEYHOLE_READ = 59;
unsigned int SET_MODE = 62;
unsigned int SET_INTERRUPT = 61;
unsigned int SET_AXI_CTL_DEVICE = 63;


/*these are the register offsets of the AXI-Stream FIFO*/
const unsigned int ISR = 0x0;  //Interrupt Status Regiser  R/WC
const unsigned int IER = 0x4;  //Interrupt Enable Register R/W
const unsigned int TDFR = 0x8; //Transmit Data FIFO Reset W
const unsigned int TDFV = 0xC; //Transmit Data FIFO Vacancy R
const unsigned int TDFD = 0x0; //Transmit Data FIFO 32-bit Data Write Port   W
//const unsigned int TDFD = 0x10; //Transmit Data FIFO 32-bit Data Write Port   W
const unsigned int TLR = 0x14;  //Transmit Length Register  W
const unsigned int RDFR = 0x18; //Receive Data FIFO Reset W
const unsigned int RDFO = 0x1C; //Receive Data FIFO Occupancy  R
//const unsigned int RDFD = 0x2000; //Receive Data FIFO 32-bit Data Read Port R
const unsigned int RDFD = 0x1000; //Receive Data FIFO 32-bit Data Read Port R
//const unsigned int RDFD = 0x20; //Receive Data FIFO 32-bit Data Read Port R
const unsigned int RLR = 0x24;  //Receive LEngth Register R
const unsigned int SRR = 0x28;  //AXI4-S Reset W
const unsigned int TDR = 0x2C;  //Transmit Destination Register W
const unsigned int RDR = 0x30;  //Receive Destination Register  R
const unsigned int TXID = 0x34; // Transmit ID Register W
const unsigned int TXUSER = 0x38; //Transmit USER Register W
const unsigned int RXID = 0x3C; //Receive ID Register  R
const unsigned int RXUESR = 0x40; // Receive USER REgister R

int main()
{

	/* Open device file and initialize */
	bram = open(devfilename, O_RDWR);
	if(bram < 0){
		return -1;
	}

	hls_write = open(devfilename_2, O_RDWR);
	if(hls_write < 0){
		return -1;
	}

	hls_read = open(devfilename_3, O_RDWR);
	if(hls_read < 0){
		return -1;
	}
	printf("Opened files\n");

	/*******************Set AXI addresses of Peripherals********************/

	if(ioctl(bram, SET_AXI_DEVICE, &bram_axi_addr) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("set peripheral to axi base address %x\n", bram_axi_addr);

	if(ioctl(hls_write, SET_AXI_CTL_DEVICE, &hls_write_ctl_axi_addr) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("set peripheral to axi base address %x\n", hls_write_ctl_axi_addr);


	if(ioctl(hls_read, SET_AXI_CTL_DEVICE, &hls_read_ctl_axi_addr) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("set peripheral to axi base address %x\n", hls_read_ctl_axi_addr);


	if(ioctl(hls_write, SET_AXI_DEVICE, &hls_write_axi_addr) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("set peripheral to axi base address %x\n", hls_write_axi_addr);


	if(ioctl(hls_read, SET_AXI_DEVICE, &hls_read_axi_addr) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("set peripheral to axi base address %x\n", hls_read_axi_addr);

	/* set mode of AXI_FIFO*/
	if(ioctl(hls_read, SET_MODE, &hls_fifo_mode) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("set axi fifo to mode: %d\n", hls_fifo_mode);

	/* set mode of AXI_FIFO*/
	if(ioctl(hls_write, SET_MODE, &hls_fifo_mode) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("set axi fifo to mode: %d\n", hls_fifo_mode);

	/**** Set AXI Addresses of CDMA, PCIe, and INTERRUPT CONTROLLER ****/
	/* Any open device File can perform these configurations */

	if(ioctl(bram, SET_AXI_PCIE_CTL, &pcie_ctl_addr) < 0){
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("set PCIE CTL to axi address %x\n", pcie_ctl_addr);

	if(ioctl(bram, SET_AXI_CDMA, &cdma_addr) < 0){
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("set CDMA to axi address %x\n", cdma_addr);

	if(ioctl(bram, SET_AXI_PCIE_M, &pcie_m_addr) < 0){ 
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("set PCIE_M to axi address %x\n", pcie_m_addr);

	if(ioctl(bram, SET_AXI_INT_CTRL, &axi_int_addr) < 0){ 
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("set AXI interrupt Controller to axi address %x\n", axi_int_addr);

	/****** Set the mode of hls_read to be "Slave with interrupt" ***********/
	unsigned int interrupt_vector = 0x2;
	ioctl(hls_read, SET_INTERRUPT, &interrupt_vector); 
	printf("set peripheral as slave with interrupt at vector:%x\n", interrupt_vector);

	/**************** Initialize test buffer ********************************/
	in[0] = 0xAAAAAAAA;
	in[1] = 0xBBBBBBBB;
	in[2] = 0xCCCCCCCC;
	in[3] = 0xDDDDDDDD;
	in[4] = 0xEEEEEEEE;
	in[5] = 0xFFFFFFFF;
	in[6] = 0x00000000;
	in[7] = 0x11111111;

	int ret_val;

	/*use lseek to move offset in file*/
	lseek(bram, 0x4, 0);

	ret_val = write(bram, in, sizeof(in));
	printf("wrote a value from address:('%p')\n", in);

	/***************** Allocate a Receive Buffer ***************************/

	unsigned int buff[sizeof(in)/4];

	ret_val = read(bram, (void*)buff, sizeof(in));

	printf("buffer address is:%p\n", buff);

	int error = 0;
	int i;
	for(i=0;i<(sizeof(buff)/4);i++)
	{
		printf("value read: %x\n", buff[i]);
		if (buff[i] != in[i])
		{
			error = 1;
		}
	}

	close(bram);

	if (error == 0)
		printf("BRAM TEST PASSED\n");

	else
		printf("BRAM TEST FAILED\n");

	/*Start up the RX FIFO Thread*/
	//This thread will perform a blocking read until an interrupt
	//has occured signifying the data is availble
	//first initialize a receive buffer
	unsigned int rxbuff[sizeof(in)];

	pthread_t rxfifo;

	if(pthread_create(&rxfifo, NULL, rxfifo_read, rxbuff))
	{
		printf("Error creating thread\n");
	}
	printf("Other Thread\n");

	/*******Send data to the TX FIFO (front end) **********/

	ret_val = write(hls_write, in, sizeof(in));  

	printf("TX FIFO: Transmitted data\n");

	//Should have written.... wait for RX FIFO interrupt.


	/*wait for the RX FIFO Thread to return data*/
	if(pthread_join(rxfifo, NULL)) 
	{
		printf("Error joining threads\n");
	}

	printf("Threads joined!\n");
	
//	ioctl(hls_read, SET_CDMA_KEYHOLE_READ, 0); 

	/*Close files*/	
	close(bram);

	close(hls_write);

	close(hls_read);

	printf("closed files\n");

	return 0;
}

/**************RX FIFO Thread**********/

void *rxfifo_read(void *read_buf)
{	
	int return_val;
	unsigned int txregbuf;
	struct pollfd pollfds;
	int timeout = 10000;    //in ms
	int result;
	unsigned int buff2[1];
	unsigned int buff[8];  //8 32b data words
	int i;

	/*initialize pollfds*/
	pollfds.fd = hls_read;
	pollfds.events = POLLIN;  //wait for data

	printf("entered rx fifo thread\n");

	/*This should perform a blocking poll until an interrupt is detected
	 * to this device*/
	printf("just before poll().....\n");

	result = poll(&pollfds, 1, timeout);
	switch (result) {
		case 0: 
			printf ("timeout occured, no interrupt detected\n");
		break;

		case -1:
			printf("error occured in poll\n");
			return 0;
		
		default:
			printf("RX FIFO INTERRUPT DETECTED!\n");
		
			/* Read from peripheral */
			return_val = read(hls_read, (void*)buff, sizeof(buff));  
			
			printf("Read the values from axi fifo....\n");
			for(i=0;i<(sizeof(buff)/4);i++)
			{
				printf("value read: %x\n", buff[i]);
			}
		
	}

	return NULL;
}
