#include <stdio.h>
#include "echo_test.h"
#include <pthread.h>
#include <sys/poll.h>
#include <unistd.h>
#include <sched.h>

#define AXI_STREAM_FIFO 1

void *rxfifo_read(void *read_buf);
void *rxfifo_read2(void *read_buf);
void *rxfifo_read3(void * read_buf);
void *rxfifo_read4(void * read_buf);

char devname[] = DEV_NAME;
char devname_2[] = DEV_NAME_2;
char devname_3[] = DEV_NAME_3;
char devname_4[] = DEV_NAME_4;

int hls_write = -1;
int hls_read = -1;
int hls_write_2 = -1;
int hls_read_2 = -1;
//int trace_read = -1;
//int trace_control = -1;

char * devfilename = devname;
char * devfilename_2 = devname_2;
char * devfilename_3 = devname_3;
char * devfilename_4 = devname_4;

/* System View Core AXI Addresses */
//unsigned int pcie_ctl_addr = 0x40002000;
//unsigned int cdma_addr = 0x40001000;
//unsigned int pcie_m_addr = 0x40010000;
//unsigned int axi_int_addr = 0x40004000;
unsigned int in[2048];

/* User Peripheral AXI Addresses */
//unsigned int bram_axi_addr = 0x80000000;
//unsigned int hls_write_ctl_axi_addr = 0x80001000;
//unsigned int hls_write_axi_addr = 0x80010000;
//unsigned int hls_read_ctl_axi_addr = 0x80002000;
//unsigned int hls_read_axi_addr = 0x80020000;

unsigned long hls_read_ctl_axi_addr = 0xC0010000;
unsigned long hls_read_axi_addr = 0xC0000000;
unsigned long hls_write_ctl_axi_addr = 0x80010000;
unsigned long hls_write_axi_addr = 0x80000000;
unsigned long hls_read_2_ctl_axi_addr = 0xC0030000;
unsigned long hls_read_2_axi_addr = 0xC0020000;
unsigned long hls_write_2_ctl_axi_addr = 0x80030000;
unsigned long hls_write_2_axi_addr = 0x80020000;
//unsigned long trace_read_ctl_axi_addr = 0x80020000;
//unsigned long trace_read_axi_addr = 0x80010000;
//unsigned long trace_control_axi_addr = 0x80000000;

unsigned long hls_fifo_mode = AXI_STREAM_FIFO;
unsigned long dma_size = 4096;

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
unsigned int SET_DMA_SIZE = 64;
unsigned int RESET_DMA_ALLOC = 65;


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
	//	bram = open(devfilename, O_RDWR);
	//	if(bram < 0){
	//		return -1;
	//	}

	hls_write = open(devfilename, O_RDWR);
	if(hls_write < 0){
		printf("ERROR doing ioctl\n");
		return -1;
	}

	hls_read = open(devfilename_2, O_RDWR);
	if(hls_read < 0){
		printf("ERROR doing ioctl\n");
		return -1;
	}

	hls_write_2 = open(devfilename_3, O_RDWR);
	if(hls_write < 0){
		printf("ERROR doing ioctl\n");
		return -1;
	}

	hls_read_2 = open(devfilename_4, O_RDWR);
	if(hls_read < 0){
		printf("ERROR doing ioctl\n");
		return -1;
	}

	//	trace_read = open(devfilename_3, O_RDWR);
	//	if(trace_read < 0){
	//		printf("ERROR doing ioctl\n");
	//		return -1;
	//	}

	//	trace_control = open(devfilename_4, O_RDWR);
	//	if(trace_control < 0){
	//		printf("ERROR doing ioctl\n");
	//		return -1;
	//	}
	//	printf("Opened files\n");

	/*******************Set AXI addresses of Peripherals********************/

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

	if(ioctl(hls_write_2, SET_AXI_CTL_DEVICE, &hls_write_2_ctl_axi_addr) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("set peripheral to axi base address %x\n", hls_write_2_ctl_axi_addr);


	if(ioctl(hls_read_2, SET_AXI_CTL_DEVICE, &hls_read_2_ctl_axi_addr) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("set peripheral to axi base address %x\n", hls_read_2_ctl_axi_addr);

	//	if(ioctl(trace_read, SET_AXI_CTL_DEVICE, &trace_read_ctl_axi_addr) < 0) {
	//		printf("ERROR doing ioctl\n");
	//		return -1;
	//	}
	//	printf("set peripheral to axi base address %x\n", trace_read_ctl_axi_addr);

	if(ioctl(hls_write, SET_AXI_DEVICE, &hls_write_axi_addr) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("set peripheral to axi base address %x\n", hls_write_axi_addr);

	if(ioctl(hls_write_2, SET_AXI_DEVICE, &hls_write_2_axi_addr) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("set peripheral to axi base address %x\n", hls_write_2_axi_addr);

	//	if(ioctl(trace_control, SET_AXI_DEVICE, &trace_control_axi_addr) < 0) {
	//		printf("ERROR doing ioctl\n");
	//		return -1;
	//	}
	//	printf("set peripheral to axi base address %x\n", trace_control_axi_addr);

	if(ioctl(hls_read, SET_AXI_DEVICE, &hls_read_axi_addr) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("set peripheral to axi base address %x\n", hls_read_axi_addr);

	if(ioctl(hls_read_2, SET_AXI_DEVICE, &hls_read_2_axi_addr) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("set peripheral to axi base address %x\n", hls_read_2_axi_addr);

	//	if(ioctl(trace_read, SET_AXI_DEVICE, &trace_read_axi_addr) < 0) {
	//		printf("ERROR doing ioctl\n");
	//		return -1;
	//	}
	//	printf("set peripheral to axi base address %x\n", trace_read_axi_addr);

	/* Reset DMA allocation*/
	if(ioctl(hls_read, RESET_DMA_ALLOC, 0) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("DMA Allocation Reset\n");


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

	if(ioctl(hls_read_2, SET_MODE, &hls_fifo_mode) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("set axi fifo to mode: %d\n", hls_fifo_mode);

	/* set mode of AXI_FIFO*/
	if(ioctl(hls_write_2, SET_MODE, &hls_fifo_mode) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("set axi fifo to mode: %d\n", hls_fifo_mode);

	/* set mode of AXI_FIFO*/
	//	if(ioctl(trace_read, SET_MODE, &hls_fifo_mode) < 0) {
	//		printf("ERROR doing ioctl\n");
	//		return -1;
	//	}
	//	printf("set axi fifo to mode: %d\n", hls_fifo_mode);

	/****** Set the mode of hls_read to be "Slave with interrupt" ***********/
	unsigned int interrupt_vector = 0x10;  /*2^5 - NOTE - THIS IS POWER OF 2!!!*/
	ioctl(hls_read, SET_INTERRUPT, &interrupt_vector); 
	printf("set peripheral as slave with interrupt at vector:%x\n", interrupt_vector);

	interrupt_vector = 0x20;  /*2^5 - NOTE - THIS IS POWER OF 2!!!*/
	ioctl(hls_read_2, SET_INTERRUPT, &interrupt_vector); 
	printf("set peripheral as slave with interrupt at vector:%x\n", interrupt_vector);

	/****** Set the mode of trace_read to be "Slave with interrupt" ***********/
	//	interrupt_vector = 0x10;  /*2^5 - NOTE - THIS IS POWER OF 2!!!*/
	//	ioctl(trace_read, SET_INTERRUPT, &interrupt_vector); 
	//	printf("set peripheral as slave with interrupt at vector:%x\n", interrupt_vector);

	/*initialize and reset the trace module counter*/
	//	unsigned int activate = 3;
	int ret_val;
	//	ret_val = write(trace_control, &activate, sizeof(activate));   
	//	if (ret_val == 0)
	//		printf("WRITE ERROR\n");

	//	/*initialize the trace module counter*/
	//	unsigned int activate = 1;
	//	int ret_val;
	//	ret_val = write(trace_control, &activate, sizeof(activate));   
	//	if (ret_val == 0)
	//		printf("WRITE ERROR\n");
	/********************************* BRAM TEST  ********************************************/
	int p=0;
	while(p<63)
	{
		in[p++] = 0xAAAAAAA0;
		in[p++] = 0xBBBBBBB0;
		in[p++] = 0xCCCCCCC0;
		in[p++] = 0xDDDDDDD0;
		in[p++] = 0xEEEEEEE0;
		in[p++] = 0xFFFFFFF0;
		in[p++] = 0x11111110;
		in[p++] = 0x22222220;
		//	in[p++] = 0x33333330;
		//	in[p++] = 0x44444440;
	}


	/************************* AXI STREAMING FIFO TEST ***************************************/

	/*Start up the RX FIFO Thread*/
	//This thread will perform a blocking read until an interrupt
	//has occured signifying the data is availble
	//first initialize a receive buffer
	unsigned int rxbuff[sizeof(in)];
	unsigned int rxbuff2[sizeof(in)];

	pthread_t rxfifo, rxfifo2, rxfifo3, rxfifo4;

	if(pthread_create(&rxfifo2, NULL, rxfifo_read2, rxbuff2))
	{
		printf("Error creating thread\n");
	}
	printf("Other Thread 2\n");

	if(pthread_create(&rxfifo, NULL, rxfifo_read, rxbuff))
	{
		printf("Error creating thread\n");
	}
	printf("Other Thread\n");


	/*the first write thread*/
	if(pthread_create(&rxfifo3, NULL, rxfifo_read3, in))
	{
		printf("Error creating thread\n");
	}
	printf("Other Thread 3\n");

	/*the second write thread*/
	if(pthread_create(&rxfifo4, NULL, rxfifo_read4, in))
	{
		printf("Error creating thread\n");
	}
	printf("Other Thread 4\n");

	/*******Send data to the TX FIFO (front end) **********/

	printf("waiting for threads to finish....\n");
	//Should have written.... wait for RX FIFO interrupt.

	//sleep(10);

	pthread_exit(0);

	if(pthread_join(rxfifo3, NULL)) 
	{
		printf("Error joining threads\n");
	}
	printf("thread done\n");

	if(pthread_join(rxfifo4, NULL)) 
	{
		printf("Error joining threads\n");
	}
	printf("thread done\n");

	/*wait for the RX FIFO Thread to return data*/
	if(pthread_join(rxfifo, NULL)) 
	{
		printf("Error joining threads\n");
	}
	printf("thread done\n");

	if(pthread_join(rxfifo2, NULL)) 
	{
		printf("Error joining threads\n");
	}
	printf("thread done\n");

	printf("Threads joined!\n");
	/*Close files*/	
	//close(bram);

	close(hls_write);

	close(hls_read);

	close(hls_write_2);

	close(hls_read_2);
	//	close(trace_read);
	//	close(trace_control);

	printf("closed files\n");

	return 0;
}

/*TX Thread*/
void *rxfifo_read3(void * tx_buf)
{	
	int ret_val;
	unsigned int txregbuf;
	struct pollfd pollfds;
	int timeout = 100;    //in ms
	int result;
	unsigned int buff2[1];
	unsigned int buff[1024];  //50 32b data words
	unsigned long long trace_buff[256];  //long long is 64bit on ARM
	int i;


	printf("entered tx fifo thread\n");

	unsigned int tx[2048];
	int counter = 0;

	while(counter < 200)
	{
		ret_val = write(hls_write, tx_buf, sizeof(tx));   
		if (ret_val == -1)
		{
			printf("WRITE ERROR\n");
			break;
		}

		if (ret_val == 0)
		{
			sched_yield();
		}
		else
		{
		printf("file 1 write!\n");
		counter++;
		}
		}

	printf("Finished file 1 writing!!!!!\n");
	pthread_exit(0);
	return NULL;
}

/*TX Thread*/
void *rxfifo_read4(void * tx_buf)
{	
	int ret_val;
	unsigned int txregbuf;
	struct pollfd pollfds;
	int timeout = 100;    //in ms
	int result;
	unsigned int buff2[1];
	unsigned int buff[1024];  //50 32b data words
	unsigned long long trace_buff[256];  //long long is 64bit on ARM
	int i;
	int counter = 0;


	printf("entered tx fifo thread\n");

	unsigned int tx[2048];

	while(counter < 200)
	{
		ret_val = write(hls_write_2, tx_buf, sizeof(tx));   
		if (ret_val == -1)
		{
			printf("WRITE ERROR\n");
			break;
		}
		
		if (ret_val == 0)
		{
			sched_yield();
		}
		else
		{
		printf("file 2 write!\n");
		counter++;
		}
		}
	printf("Finished file 2 writing!!!!!\n");
	pthread_exit(0);
	return NULL;
}

/**************RX FIFO Thread**********/

void *rxfifo_read(void *read_buf)
{	
	int return_val;
	unsigned int txregbuf;
	struct pollfd pollfds;
	int timeout = 1000;    //in ms
	int result;
	unsigned int buff2[1];
	unsigned int buff[1024];  //50 32b data words
	unsigned long long trace_buff[256];  //long long is 64bit on ARM
	int i;

	/*initialize pollfds*/
	pollfds.fd = hls_read;
	pollfds.events = POLLIN;  //wait for data

	printf("entered rx fifo 1 thread\n");

	int policy, s;
	struct sched_param param;

	//policy = SCHED_BATCH;
	//pthread_setschedparam(pthread_self(), 2, &param); //1 = FIFO

	s = pthread_getschedparam(pthread_self(), &policy, &param);
	printf("Thread Priority for read 1: %x\n", param.sched_priority);		   
	printf("Thread Policy for read 1: %d\n", policy);		   
	
	//For the sake of running again and not being deadlocked.
		return_val = 1;
	while (return_val != 0)
					//	while (1)
				{
					return_val = read(hls_read, (void*)buff, (sizeof(buff)));  
					if (return_val == -1)
					{
						printf("READ ERROR DATA\n");
						break;
					}
					else if (return_val > 0)
					{		
						printf("Number of bytes read from file 1:%d\n", return_val);

					}
					else if (return_val ==  0)
					{		
						printf("No initial values to read from read 1\n");

					}
				}

	/*This should perform a blocking poll until an interrupt is detected
	 * to this device*/
	printf("just before poll() 1.....\n");

	while(1)
	{
		return_val = 1;
		result = poll(&pollfds, 1, timeout);
		switch (result) {
			case 0: 
				printf ("timeout occured, no interrupt detected\n");
				return NULL;
			case -1:
				printf("error occured in poll\n");
				while(1);

			default:
				//			printf("RX FIFO INTERRUPT DETECTED!\n");

				/* Read from peripheral */

				//	sleep(1);
				//	usleep(2000);
				while (return_val != 0)
					//	while (1)
				{
					return_val = read(hls_read, (void*)buff, (sizeof(buff)));  
					if (return_val == -1)
					{
						printf("READ ERROR DATA\n");
						break;
					}
					else if (return_val > 0)
					{		
						printf("Number of bytes read from file 1:%d\n", return_val);

					}
				}

		}
		//sched_yield();
		//	sleep(10);
	}
	printf ("File 1 read thread closing!\n");
	pthread_exit(0);
	return NULL;
}

/**************RX FIFO Thread**********/

void *rxfifo_read2(void *read_buf)
{	
	int return_val;
	unsigned int txregbuf;
	struct pollfd pollfds;
	int timeout = 1000;    //in ms
	int result;
	unsigned int buff2[1];
	unsigned int buff[1024];  //50 32b data words
	unsigned long long trace_buff[256];  //long long is 64bit on ARM
	int i;

	int policy, s;
	struct sched_param param;

	//pthread_setschedparam(pthread_self(), 2, &param);

	s = pthread_getschedparam(pthread_self(), &policy, &param);
	printf("Thread Priority for read 2: %x\n", param.sched_priority);		   
	printf("Thread Policy for read 2: %d\n", policy);		   

	/*initialize pollfds*/
	pollfds.fd = hls_read_2;
	pollfds.events = POLLIN;  //wait for data

	printf("entered rx fifo 2 thread\n");

	//For the sake of running again and not being deadlocked.
		return_val = 1;
	while (return_val != 0)
					//	while (1)
				{
					return_val = read(hls_read_2, (void*)buff, (sizeof(buff)));  
					if (return_val == -1)
					{
						printf("READ ERROR DATA\n");
						break;
					}
					else if (return_val > 0)
					{		
						printf("Number of bytes read from file 2:%d\n", return_val);

					}
					else if (return_val ==  0)
					{		
						printf("No initial values to read from read 2\n");

					}
				}

	/*This should perform a blocking poll until an interrupt is detected
	 * to this device*/
	printf("just before poll() 2.....\n");

	while(1)
	{
		return_val = 1;
		result = poll(&pollfds, 1, timeout);
		switch (result) {
			case 0: 
				printf ("timeout occured, no interrupt detected\n");
				//	sleep(1);
				return NULL;

			case -1:
				printf("error occured in poll\n");
				while(1);

			default:
				//			printf("RX FIFO INTERRUPT DETECTED!\n");

				/* Read from peripheral */

				//	sleep(1);
				//	usleep(2000);
				while (return_val != 0)
					//	while (1)
				{
					return_val = read(hls_read_2, (void*)buff, (sizeof(buff)));  
					if (return_val == -1)
					{
						printf("READ ERROR DATA\n");
						break;
					}
					else if (return_val > 0)
					{		
						printf("Number of bytes read from file 2:%d\n", return_val);

					}
				}

				/*Read the trace module FIFO*/
				//			return_val = 256;
				//			while (return_val == 256)
				//			{
				//			return_val = read(trace_read, (void*)trace_buff, (sizeof(trace_buff)));  

				//			printf("number of TRACE bytes read: %x\n", return_val);

				//			for(i=0;i<(return_val/8);i++)
				//			{
				//				printf("trace value read: %llx\n", trace_buff[i]);
				//			}
				//			}
		}
		//sched_yield();
	}
	printf ("File 2 read thread closing!\n");
	pthread_exit(0);
	return NULL;
}
