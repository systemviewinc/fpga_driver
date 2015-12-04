#include <stdio.h>
#include "test.h"
#include <pthread.h>
#include <sys/poll.h>

#define AXI_STREAM_FIFO 1

void *rxfifo_read(void *read_buf);

char devname[] = DEV_NAME;
char devname_2[] = DEV_NAME_2;
//char devname_3[] = DEV_NAME_3;

int bram = -1;
int hls_write = -1;
int hls_read = -1;

char * devfilename = devname;
char * devfilename_2 = devname_2;

/* System View Core AXI Addresses */
//unsigned int pcie_ctl_addr = 0x40002000;
//unsigned int cdma_addr = 0x40001000;
//unsigned int pcie_m_addr = 0x40010000;
//unsigned int axi_int_addr = 0x40004000;
unsigned int in[64];

/* User Peripheral AXI Addresses */
//unsigned int bram_axi_addr = 0x80000000;
//unsigned int hls_write_ctl_axi_addr = 0x80001000;
//unsigned int hls_write_axi_addr = 0x80010000;
//unsigned int hls_read_ctl_axi_addr = 0x80002000;
//unsigned int hls_read_axi_addr = 0x80020000;

unsigned long hls_write_ctl_axi_addr = 0x80010000;
unsigned long hls_write_axi_addr = 0x80000000;
unsigned long hls_read_ctl_axi_addr = 0xC0010000;
unsigned long hls_read_axi_addr = 0xC0000000;

unsigned long hls_fifo_mode = AXI_STREAM_FIFO;

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


//	hls_write = open(devfilename, O_RDWR);
//	if(hls_write < 0){
//		return -1;
//	}

//	hls_read = open(devfilename_2, O_RDWR);
//	if(hls_read < 0){
//		return -1;
//	}
//	printf("Opened files\n");

//	if(ioctl(hls_write, SET_AXI_CTL_DEVICE, &hls_write_ctl_axi_addr) < 0) {
//		printf("ERROR doing ioctl\n");
//		return -1;
//	}
//	printf("set peripheral to axi base address %x\n", hls_write_ctl_axi_addr);


//	if(ioctl(hls_read, SET_AXI_CTL_DEVICE, &hls_read_ctl_axi_addr) < 0) {
//		printf("ERROR doing ioctl\n");
//		return -1;
//	}
//	printf("set peripheral to axi base address %x\n", hls_read_ctl_axi_addr);


//	if(ioctl(hls_write, SET_AXI_DEVICE, &hls_write_axi_addr) < 0) {
//		printf("ERROR doing ioctl\n");
//		return -1;
//	}
//	printf("set peripheral to axi base address %x\n", hls_write_axi_addr);


//	if(ioctl(hls_read, SET_AXI_DEVICE, &hls_read_axi_addr) < 0) {
//		printf("ERROR doing ioctl\n");
//		return -1;
//	}
//	printf("set peripheral to axi base address %x\n", hls_read_axi_addr);

	/* set mode of AXI_FIFO*/
//	if(ioctl(hls_read, SET_MODE, &hls_fifo_mode) < 0) {
//		printf("ERROR doing ioctl\n");
//		return -1;
//	}
//	printf("set axi fifo to mode: %d\n", hls_fifo_mode);

	/* set mode of AXI_FIFO*/
//	if(ioctl(hls_write, SET_MODE, &hls_fifo_mode) < 0) {
//		printf("ERROR doing ioctl\n");
//		return -1;
//	}
//	printf("set axi fifo to mode: %d\n", hls_fifo_mode);

	/****** Set the mode of hls_read to be "Slave with interrupt" ***********/
//	unsigned int interrupt_vector = 0x10;  /*2^5 - NOTE - THIS IS POWER OF 2!!!*/
//	ioctl(hls_read, SET_INTERRUPT, &interrupt_vector); 
//	printf("set peripheral as slave with interrupt at vector:%x\n", interrupt_vector);
/********************************* BRAM TEST  ********************************************/

//int p=0;
//while(p<63)
//{
//	in[p++] = 0xAAAAAAA0;
//	in[p++] = 0xBBBBBBB0;
//	in[p++] = 0xCCCCCCC0;
//	in[p++] = 0xDDDDDDD0;
//	in[p++] = 0xEEEEEEE0;
//	in[p++] = 0xFFFFFFF0;
//	in[p++] = 0x11111110;
//	in[p++] = 0x22222220;
//}
unsigned char packet_binary[] = {
	  0x00, 0x1c, 0x42, 0x00, 0x00, 0x18, 0x08, 0x00, 0x27, 0xeb, 0x0b, 0x97,
	    0x08, 0x00, 0x45, 0x00, 0x00, 0x8e, 0x15, 0x5a, 0x40, 0x00, 0x40, 0x06,
		  0xd4, 0x8e, 0x0a, 0xd3, 0x37, 0x04, 0x0a, 0x0c, 0x04, 0x9f, 0x87, 0x59,
		    0x0c, 0x31, 0xd4, 0x04, 0xd0, 0x74, 0x47, 0xee, 0x26, 0x47, 0x50, 0x18,
			  0x77, 0xc4, 0x55, 0xb8, 0x00, 0x00, 0x43, 0x00, 0x39, 0x31, 0x31, 0x36,
			    0x38, 0x03, 0x03, 0xd9, 0x00, 0x00, 0x78, 0x73, 0x64, 0x62, 0x00, 0x73,
				  0x65, 0x71, 0x75, 0x65, 0x6e, 0x63, 0x65, 0x00, 0x22, 0x6a, 0x73, 0x6e,
				    0x2d, 0x4a, 0x54, 0x41, 0x47, 0x2d, 0x53, 0x4d, 0x54, 0x31, 0x2d, 0x32,
					  0x31, 0x30, 0x32, 0x30, 0x33, 0x38, 0x35, 0x39, 0x34, 0x32, 0x34, 0x41,
					    0x2d, 0x33, 0x33, 0x36, 0x39, 0x31, 0x30, 0x39, 0x33, 0x2d, 0x30, 0x2d,
						  0x30, 0x32, 0x30, 0x31, 0x30, 0x30, 0x53, 0x4c, 0x56, 0x22, 0x00, 0x5b,
						    0x5b, 0x22, 0x72, 0x65, 0x61, 0x64, 0x22, 0x2c, 0x38, 0x2c, 0x31, 0x2c,
							  0x32, 0x31, 0x38, 0x32, 0x5d, 0x5d, 0x00, 0x22, 0x22, 0x00, 0x03, 0x01,
							    0x22, 0x2c, 0x38, 0x2c, 0x31, 0x2c, 0x00, 0x90, 0x32, 0x31, 0x38, 0x32,
								  0x5d, 0x5d, 0x00, 0x22, 0x22, 0x00, 0x03, 0x01
};

//	int	packet_file = open("./packet_hex", O_RDONLY);
//	
//	if(packet_file < 0){
//		printf("error reading packet file\n");
//		return -1;
//	}

//	#define BUFFER_SIZE 156

//	unsigned char buffer[BUFFER_SIZE];

	/* Read in 256 8-bit numbers into the buffer */
//	size_t bytes_read = 0;
  //  bytes_read = fread((void*)buffer, sizeof(unsigned char), BUFFER_SIZE, packet_file);
	// Note: sizeof(unsigned char) is for emphasis

	int i;

	for(i=0;i<(sizeof(packet_binary));i++)
	{
		printf("value read: %x\n", packet_binary[i]);
	}
	
	int ret_val;

	return 0;

	/*use lseek to move offset in file*/
	//lseek(bram, 0x4, 0);

/*	ret_val = write(bram, in, sizeof(in));
	printf("wrote a value from address:('%p')\n", in);


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


/************************* AXI STREAMING FIFO TEST ***************************************/

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
//	ret_val = write(hls_write, in, sizeof(in));  
//	sleep(5);
	ret_val = write(hls_write, in, sizeof(in));   
	if (ret_val == 0)
		printf("WRITE ERROR\n");

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
	unsigned int buff[64];  //50 32b data words
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
	//		return_val = read(hls_read, (void*)buff, sizeof(buff));  

			return_val = read(hls_read, (void*)buff, (sizeof(buff)));  
			if (return_val == 0)
				printf("READ ERROR\n");
			
			printf("Number of bytes read:%x\n", return_val);

			for(i=0;i<(return_val/4);i++)
			{
				printf("value read: %x\n", buff[i]);
			}
		
//			return_val = read(hls_read, (void*)buff, (sizeof(buff)-2));  
//			if (return_val == 0)
//				printf("READ ERROR\n");
			
//			printf("Number of bytes read:%x\n", return_val);
//
//			for(i=0;i<(return_val/4);i++)
//			{
//				printf("value read: %x\n", buff[i]);
//			}
	}

	return NULL;
}
