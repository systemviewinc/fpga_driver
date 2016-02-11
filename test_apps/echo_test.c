#include <stdio.h>
#include "echo_test.h"
#include <pthread.h>
#include <sys/poll.h>
#include <unistd.h>
#include <sched.h>

#define AXI_STREAM_FIFO 1

#define FILE_SIZE_1 65536
#define FILE_SIZE_2 65536
#define FILE_SIZE_3 65536
#define FILE_SIZE_4 65536

#define TRANSFER_SIZE_1 1024
#define TRANSFER_SIZE_2 1024
#define TRANSFER_SIZE_3 1024
#define TRANSFER_SIZE_4 1024

int xfer_size_1, xfer_size_2, xfer_size_3, xfer_size_4;


struct statistics 
{
	int tx_bytes;
	int rx_bytes;
	unsigned long seconds;
	unsigned long ns;
	int cdma_attempt;
	int ip_not_ready;
	int cdma_usage_cnt;
};

//void *rxfifo_read(void *read_buf);
//void *rxfifo_read2(void *read_buf);
//void *rxfifo_read3(void * read_buf);
void *tx(void * file_handle);
void *rx(void * file_handle);
double calc_BW(double bytes, double secs, double ns);
int file_init(int file_h, unsigned long axi_addr, unsigned long axi_ctl_addr, unsigned int mode, unsigned int interrupt_vector, unsigned int file_size);
double spawn_threads(int file_1, int file_2, int file_3, int file_4);
void create_csv(char *filename, double a[4][5][10],int n,int m, int x);

char devname[] = DEV_NAME;
char devname_2[] = DEV_NAME_2;
char devname_3[] = DEV_NAME_3;
char devname_4[] = DEV_NAME_4;
char devname_5[] = DEV_NAME_5;
char devname_6[] = DEV_NAME_6;
char devname_7[] = DEV_NAME_7;
char devname_8[] = DEV_NAME_8;

int hls_write = -1;
int hls_read = -1;
int hls_write_2 = -1;
int hls_read_2 = -1;
int hls_write_3 = -1;
int hls_read_3 = -1;
int hls_write_4 = -1;
int hls_read_4 = -1;
//int trace_read = -1;
//int trace_control = -1;

struct thread_struct 
{
	int file_handle;
	int transfer_size;
};

char * devfilename = devname;
char * devfilename_2 = devname_2;
char * devfilename_3 = devname_3;
char * devfilename_4 = devname_4;
char * devfilename_5 = devname_5;
char * devfilename_6 = devname_6;
char * devfilename_7 = devname_7;
char * devfilename_8 = devname_8;

/* System View Core AXI Addresses */
//unsigned int pcie_ctl_addr = 0x40002000;
//unsigned int cdma_addr = 0x40001000;
//unsigned int pcie_m_addr = 0x40010000;
//unsigned int axi_int_addr = 0x40004000;
unsigned int in[FILE_SIZE_4/4];

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
unsigned long hls_read_3_ctl_axi_addr = 0xC0050000;
unsigned long hls_read_3_axi_addr = 0xC0040000;
unsigned long hls_write_3_ctl_axi_addr = 0x80050000;
unsigned long hls_write_3_axi_addr = 0x80040000;
unsigned long hls_read_4_ctl_axi_addr = 0xC0070000;
unsigned long hls_read_4_axi_addr = 0xC0060000;
unsigned long hls_write_4_ctl_axi_addr = 0x80070000;
unsigned long hls_write_4_axi_addr = 0x80060000;
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
unsigned int SET_FILE_SIZE = 66;
unsigned int GET_FILE_STATISTICS = 67;
unsigned int START_FILE_TIMER = 68;
unsigned int STOP_FILE_TIMER = 69;
unsigned int GET_DRIVER_STATISTICS = 70;
unsigned int START_DRIVER_TIMER = 71;
unsigned int STOP_DRIVER_TIMER = 72;

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

pthread_t tx1, tx2, rx1, rx2, tx3, rx3, tx4, rx4;

struct statistics * tx_statistics_1;
struct statistics * tx_statistics_2;
struct statistics * tx_statistics_3;
struct statistics * tx_statistics_4;
struct statistics * rx_statistics_1;
struct statistics * rx_statistics_2;
struct statistics * rx_statistics_3;
struct statistics * rx_statistics_4;

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
	printf("hls_write:%d\n", hls_write);

	hls_read = open(devfilename_2, O_RDWR);
	if(hls_read < 0){
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("hls_read:%d\n", hls_read);

	hls_write_2 = open(devfilename_3, O_RDWR);
	if(hls_write < 0){
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("hls_write_2:%d\n", hls_write_2);

	hls_read_2 = open(devfilename_4, O_RDWR);
	if(hls_read < 0){
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("hls_read_2:%d\n", hls_read_2);

	hls_write_3 = open(devfilename_5, O_RDWR);
	if(hls_write_3 < 0){
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("hls_write_3:%d\n", hls_write_3);

	hls_read_3 = open(devfilename_6, O_RDWR);
	if(hls_read_3 < 0){
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("hls_read_3:%d\n", hls_read_3);

	hls_write_4 = open(devfilename_7, O_RDWR);
	if(hls_write_4 < 0){
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("hls_write_4:%d\n", hls_write_4);

	hls_read_4 = open(devfilename_8, O_RDWR);
	if(hls_read_4 < 0){
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("hls_read_4:%d\n", hls_read_4);

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

	/* Reset DMA allocation*/
	if(ioctl(hls_read, RESET_DMA_ALLOC, 0) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("DMA Allocation Reset\n");

	int ret;

	ret = file_init(hls_write, hls_write_axi_addr, hls_write_ctl_axi_addr, hls_fifo_mode, 0, FILE_SIZE_1);
	if(ret < 0){
		printf("ERROR doing file init\n");
		return -1;
	}
	ret = file_init(hls_read, hls_read_axi_addr, hls_read_ctl_axi_addr, hls_fifo_mode, 0x010, FILE_SIZE_1);
	if(ret < 0){
		printf("ERROR doing file init\n");
		return -1;
	}

	ret = file_init(hls_write_2, hls_write_2_axi_addr, hls_write_2_ctl_axi_addr, hls_fifo_mode, 0, FILE_SIZE_2);
	if(ret < 0){
		printf("ERROR doing file init\n");
		return -1;
	}
	ret = file_init(hls_read_2, hls_read_2_axi_addr, hls_read_2_ctl_axi_addr, hls_fifo_mode, 0x020, FILE_SIZE_2);
	if(ret < 0){
		printf("ERROR doing file init\n");
		return -1;
	}

	ret = file_init(hls_write_3, hls_write_3_axi_addr, hls_write_3_ctl_axi_addr, hls_fifo_mode, 0, FILE_SIZE_3);
	if(ret < 0){
		printf("ERROR doing file init\n");
		return -1;
	}
	ret = file_init(hls_read_3, hls_read_3_axi_addr, hls_read_3_ctl_axi_addr, hls_fifo_mode, 0x040, FILE_SIZE_3);
	if(ret < 0){
		printf("ERROR doing file init\n");
		return -1;
	}

	ret = file_init(hls_write_4, hls_write_4_axi_addr, hls_write_4_ctl_axi_addr, hls_fifo_mode, 0, FILE_SIZE_4);
	if(ret < 0){
		printf("ERROR doing file init\n");
		return -1;
	}
	ret = file_init(hls_read_4, hls_read_4_axi_addr, hls_read_4_ctl_axi_addr, hls_fifo_mode, 0x080, FILE_SIZE_4);
	if(ret < 0){
		printf("ERROR doing file init\n");
		return -1;
	}

	/*initialize and reset the trace module counter*/
	//	unsigned int activate = 3;
	//	ret_val = write(trace_control, &activate, sizeof(activate));   
	//	if (ret_val == 0)
	//		printf("WRITE ERROR\n");

	//	/*initialize the trace module counter*/
	//	unsigned int activate = 1;
	//	int ret_val;
	//	ret_val = write(trace_control, &activate, sizeof(activate));   
	//	if (ret_val == 0)
	//		printf("WRITE ERROR\n");
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


	double bw, avg;
	double bw_arr[4][5][10];
	bw = 0.0;
	int i, j, k, x;

	for(x = 0; x<4; x++)
	{
		printf("Number of Threads: %d \n", x+1);

		k = 0;

		for(j = 4; j<=64; j=j*2)
		{
			xfer_size_1 = TRANSFER_SIZE_1*j;
			xfer_size_2 = TRANSFER_SIZE_1*j;
			xfer_size_3 = TRANSFER_SIZE_1*j;
			xfer_size_4 = TRANSFER_SIZE_1*j;

			for(i = 0; i<10; i++)
			{
				switch(x){
					case 0:
						bw_arr[x][k][i] = spawn_threads(1, 0, 0, 0);
						break;
					case 1:
						bw_arr[x][k][i] = spawn_threads(1, 1, 0, 0);
						break;
					case 2:
						bw_arr[x][k][i] = spawn_threads(1, 1, 1, 0);
						break;
					case 3:
						bw_arr[x][k][i] = spawn_threads(1, 1, 1, 1);
						break;
				}

				bw += bw_arr[x][k][i];
			}

			avg = bw/10;
			bw = 0;
			printf("TRANSFER SIZE OF %d B\n", 1024*j);

			for(i = 0; i<10; i++)
			{
				printf("%f ", bw_arr[x][k][i]);
			}

			printf("\nAverage Bandwidth: %f MB/s\n", avg);

			k++;
		}
	}
	/*write to file*/
	char* filename; 
	filename = "statistics.csv";
	create_csv(filename, bw_arr , 10, 5, 4);

	//	bw = spawn_threads(1, 0, 0, 0);
	//	bw = spawn_threads(0, 1, 0, 0);
	//	bw = spawn_threads(0, 0, 1, 0);
	//	bw = spawn_threads(0, 0, 0, 1);
	//	bw = spawn_threads(0, 1, 1, 1);


	close(hls_write);
	close(hls_read);
	close(hls_write_2);
	close(hls_read_2);
	close(hls_write_3);
	close(hls_read_3);
	close(hls_write_4);
	close(hls_read_4);
	//	close(trace_read);
	//	close(trace_control);

	printf("closed files\n");

	return 0;
}


/*TX Thread*/
void * tx(void * file_desc)
{	
	int ret_val;
	unsigned int txregbuf;
	struct pollfd pollfds;
	int timeout = 500;    //in ms
	int result;
	unsigned int buff2[1];
	unsigned int buff[FILE_SIZE_4/4];  //50 32b data words
	unsigned long long trace_buff[256];  //long long is 64bit on ARM
	int i;
	int counter = 0;
	int tx_write_bytes;
	void * statistics_buf;
	int fd;
	int iter;
	struct statistics * statistics;
	struct thread_struct * thread_struct_loc;

	thread_struct_loc = (struct thread_struct*)(file_desc);

	//	fd = (int *)file_desc;
	fd = thread_struct_loc->file_handle;

	statistics_buf = malloc(sizeof(struct statistics));
	statistics = (struct statistics *)statistics_buf;

	//	printf("entered tx thread with fd: %d\n", *fd);
	//	pthread_exit(0);

	unsigned int tx[2048];

	tx_write_bytes = 0;

	if(ioctl(fd, START_FILE_TIMER, NULL) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}

	if (fd == hls_write)
		iter = 500;
	else
		iter = 500;

	//	while(counter < iter)  //this holds number of transfers constant
	while(tx_write_bytes < (FILE_SIZE_1*512/(2*2)))  //this holds amount of data constant
	{
		//		ret_val = write(hls_write_2, tx_buf, sizeof(tx));   
		ret_val = write(fd, in, thread_struct_loc->transfer_size);   
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
			//		printf("file 2 write!\n");
			counter++;
			tx_write_bytes = tx_write_bytes + ret_val;
			//usleep(20);
			sched_yield();
		}
	}
	//	printf("Finished file writing!!!!!\n");
	//	printf("Total bytes written to file: %d\n", tx_write_bytes);
	//	struct statistics statistics;

	if(ioctl(fd, STOP_FILE_TIMER, NULL) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}

	if(ioctl(fd, GET_FILE_STATISTICS, statistics) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	//	printf("TX Statstics of file from the driver %d\n", statistics->tx_bytes);
	//	printf("TX Time Elapsed:%lu sec :  %lu ns\n\n\n", statistics->seconds, statistics->ns);
	//	pthread_exit(0);
	return statistics;
}

/**************RX FIFO Thread**********/

void *rx(void * file_desc)
{	
	int return_val;
	unsigned int txregbuf;
	struct pollfd pollfds;
	int timeout = 1000;    //in ms
	int result;
	unsigned int buff2[1];
	unsigned int buff[FILE_SIZE_4/3];  //50 32b data words
	unsigned long long trace_buff[256];  //long long is 64bit on ARM
	int i;
	int total_bytes;
	void * statistics_buf;
	//	int * fd;
	int fd;
	struct statistics * statistics;
	int zero_count; 
	struct thread_struct * thread_struct_loc;

	nice(-5);

	thread_struct_loc = (struct thread_struct*)(file_desc);

	//	fd = (int *)file_desc;
	fd = thread_struct_loc->file_handle;

	statistics_buf = malloc(sizeof(struct statistics));
	statistics = (struct statistics *)statistics_buf;

	/*initialize pollfds*/
	//	pollfds.fd = *fd;
	pollfds.fd = fd;
	pollfds.events = POLLIN;  //wait for data

	//	printf("entered rx thread with file desc: %d\n", *fd);


	int policy, s;
	struct sched_param param;

	//policy = SCHED_BATCH;
	//pthread_setschedparam(pthread_self(), 2, &param); //1 = FIFO

	//	s = pthread_getschedparam(pthread_self(), &policy, &param);
	//	printf("Thread Priority for read 1: %x\n", param.sched_priority);		   
	//	printf("Thread Policy for read 1: %d\n", policy);		   

	//	if(ioctl(*fd, START_FILE_TIMER, NULL) < 0) {
	if(ioctl(fd, START_FILE_TIMER, NULL) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	//For the sake of running again and not being deadlocked.
	/*	return_val = 1;
		while (return_val != 0)
	//	while (1)
	{
	return_val = read(*fd, (void*)buff, (sizeof(buff)));  
	if (return_val == -1)
	{
	printf("READ ERROR DATA\n");
	break;
	}
	else if (return_val > 0)
	{		
	printf("Number of bytes read from file:%d\n", return_val);

	}
	else if (return_val ==  0)
	{		
	printf("No initial values to read from read\n");
	}
	}
	*/	

	/*This should perform a blocking poll until an interrupt is detected
	 * to this device*/
	//	printf("just before poll() 1.....\n");

	total_bytes = 0;

	//	while(1)
	while(total_bytes < (FILE_SIZE_1*512/(2*2)))  //this holds amount of data constant
	{
		return_val = 1;
		result = poll(&pollfds, 1, timeout);
		switch (result) {
			case 0: 
				printf ("timeout occured, no interrupt detected\n");
				//				printf ("Total bytes read from File: %d\n", total_bytes);
				//Checking if any remaining data is in the FIFO
				//	return_val = 1;
				//	while (return_val != 0)
				//	{
				//		return_val = read(fd, (void*)buff, (thread_struct_loc->transfer_size));  
				//		if (return_val == -1)
				//		{
				//			printf("READ ERROR DATA\n");
				//			break;
				//		}
				//		else if (return_val > 0)
				//		{
				//			total_bytes = total_bytes + return_val;		
				//			printf("Yikes more data found\n");
				//			printf("Number of bytes read from file:%d\n", return_val);
				//			printf ("NEW Total bytes read from File: %d\n", total_bytes);

				//		}
				//	}

				//	if(ioctl(fd, STOP_FILE_TIMER, NULL) < 0) {
				//		printf("ERROR doing ioctl\n");
				//		return -1;
				//	}

				//	if(ioctl(fd, GET_FILE_STATISTICS, statistics) < 0) {
				//		printf("ERROR doing ioctl\n");
				//		return -1;
				//	}
				//	return statistics;
				break;

			case -1:
				printf("error occured in poll\n");
				while(1);

			default:
				//			printf("RX FIFO INTERRUPT DETECTED!\n");

				/* Read from peripheral */

				zero_count = 0;
				while (return_val != 0)
				{
					return_val = read(fd, (void*)buff, thread_struct_loc->transfer_size);  
					if (return_val == -1)
					{
						printf("READ ERROR DATA\n");
						break;
					}
					else if (return_val > 0)
					{
						total_bytes = total_bytes + return_val;		
						//						printf("Number of bytes read from file 1:%d\n", return_val);

					}
					else if (return_val == 0)
					{
						zero_count = zero_count + 1;
					}
				}

		}
		//sched_yield();
		//	sleep(10);
	}
	if(ioctl(fd, STOP_FILE_TIMER, NULL) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}

	if(ioctl(fd, GET_FILE_STATISTICS, statistics) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	//		printf("RX Statstics of file from the driver %d\n", statistics->rx_bytes);
	//		printf("RX Time Elapsed:%lu sec :  %lu ns\n\n\n", statistics->seconds, statistics->ns);
	return statistics;
	//	printf ("Total bytes read from File: %d\n", total_bytes);
	//	printf ("File read thread closing!\n");
}

double calc_BW(double bytes, double secs, double ns)
{
	double calc;
	calc = (bytes/1000000)/(secs+(ns/1000000000));
	return calc;
}

int file_init(int file_h, unsigned long axi_addr, unsigned long axi_ctl_addr, unsigned int mode, unsigned int interrupt_vector, unsigned int file_size)
{
	if(ioctl(file_h, SET_AXI_CTL_DEVICE, &axi_ctl_addr) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("set peripheral to axi base address %x\n", axi_ctl_addr);


	if(ioctl(file_h, SET_AXI_DEVICE, &axi_addr) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("set peripheral to axi base address %x\n", axi_addr);


	/* set mode of AXI_FIFO*/
	if(ioctl(file_h, SET_MODE, &mode) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("set axi fifo to mode: %d\n", mode);

	if(ioctl(file_h, SET_FILE_SIZE, &file_size) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}
	printf("set file size to: %d\n", file_size);

	if(interrupt_vector > 0)
	{
		/****** Set the mode of hls_read to be "Slave with interrupt" ***********/
		ioctl(file_h, SET_INTERRUPT, &interrupt_vector); 
		printf("set peripheral as slave with interrupt at vector:%x\n", interrupt_vector);
	}

	return 0;

}

double spawn_threads(int file_1, int file_2, int file_3, int file_4)
{
	struct thread_struct thread_struct_1_tx;
	struct thread_struct thread_struct_1_rx;
	struct thread_struct thread_struct_2_tx;
	struct thread_struct thread_struct_2_rx;
	struct thread_struct thread_struct_3_tx;
	struct thread_struct thread_struct_3_rx;
	struct thread_struct thread_struct_4_tx;
	struct thread_struct thread_struct_4_rx;

	struct statistics driver_statistics;

	int driver_bytes;
	double rx_bandwidth, tx_bandwidth, total_bandwidth, driver_bandwidth;

	/*reset the counters*/
	//	if(ioctl(hls_read, GET_DRIVER_STATISTICS, NULL) < 0) {
	//		printf("ERROR doing ioctl\n");
	//		return -1;
	//	}

	if(ioctl(hls_read, START_DRIVER_TIMER, NULL) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}

	driver_bandwidth = 0;

	if (file_1)
	{
		thread_struct_1_rx.file_handle = hls_read;
		thread_struct_1_rx.transfer_size = xfer_size_1;

		if(pthread_create(&rx1, NULL, rx, (void *)(&thread_struct_1_rx)))
		{
			printf("Error creating thread\n");
		}
	}

	if (file_2)
	{
		thread_struct_2_rx.file_handle = hls_read_2;
		thread_struct_2_rx.transfer_size = xfer_size_2;

		if(pthread_create(&rx2, NULL, rx, (void *)(&thread_struct_2_rx)))
		{
			printf("Error creating thread\n");
		}
	}
	if (file_3)
	{
		thread_struct_3_rx.file_handle = hls_read_3;
		thread_struct_3_rx.transfer_size = xfer_size_3;

		if(pthread_create(&rx3, NULL, rx, (void *)(&thread_struct_3_rx)))
		{
			printf("Error creating thread\n");
		}
	}
	if (file_4)
	{
		thread_struct_4_rx.file_handle = hls_read_4;
		thread_struct_4_rx.transfer_size = xfer_size_4;

		if(pthread_create(&rx4, NULL, rx, (void *)(&thread_struct_4_rx)))
		{
			printf("Error creating thread\n");
		}
	}
	if (file_1)
	{
		thread_struct_1_tx.file_handle = hls_write;
		thread_struct_1_tx.transfer_size = xfer_size_1;

		/*the first write thread*/
		if(pthread_create(&tx1, NULL, tx, (void *)(&thread_struct_1_tx)))
		{
			printf("Error creating thread\n");
		}
	}
	if (file_2)
	{
		thread_struct_2_tx.file_handle = hls_write_2;
		thread_struct_2_tx.transfer_size = xfer_size_2;
		/*the second write thread*/
		if(pthread_create(&tx2, NULL, tx, (void *)(&thread_struct_2_tx)))
		{
			printf("Error creating thread\n");
		}
	}
	if (file_3)
	{
		thread_struct_3_tx.file_handle = hls_write_3;
		thread_struct_3_tx.transfer_size = xfer_size_3;
		if(pthread_create(&tx3, NULL, tx, (void *)(&thread_struct_3_tx)))
		{
			printf("Error creating thread\n");
		}
	}
	if (file_4)
	{
		thread_struct_4_tx.file_handle = hls_write_4;
		thread_struct_4_tx.transfer_size = xfer_size_4;
		if(pthread_create(&tx4, NULL, tx, (void *)(&thread_struct_4_tx)))
		{
			printf("Error creating thread\n");
		}
	}
	/*******Send data to the TX FIFO (front end) **********/

	//	printf("waiting for threads to finish....\n");
	//Should have written.... wait for RX FIFO interrupt.

	//sleep(10);

	//	pthread_exit(0);

	if (file_1)
	{
		if(pthread_join(tx1, (void**)&tx_statistics_1)) 
		{
			printf("Error joining threads\n");
		}
		//	printf("thread done\n");
	}
	if (file_2)
	{
		if(pthread_join(tx2, (void**)&tx_statistics_2)) 
		{
			printf("Error joining threads\n");
		}
		//	printf("thread done\n");
	}
	if (file_3)
	{
		if(pthread_join(tx3, (void**)&tx_statistics_3)) 
		{
			printf("Error joining threads\n");
		}
		//	printf("thread done\n");
	}
	if (file_4)
	{
		if(pthread_join(tx4, (void**)&tx_statistics_4)) 
		{
			printf("Error joining threads\n");
		}
		//	printf("thread done\n");
	}
	/*wait for the RX FIFO Thread to return data*/
	if (file_1)
	{
		if(pthread_join(rx1, (void**)&rx_statistics_1)) 
		{
			printf("Error joining threads\n");
		}
		//	printf("thread done\n");
	}
	if (file_2)
	{
		if(pthread_join(rx2, (void**)&rx_statistics_2)) 
		{
			printf("Error joining threads\n");
		}
		//	printf("thread done\n");
	}
	if (file_3)
	{
		if(pthread_join(rx3, (void**)&rx_statistics_3)) 
		{
			printf("Error joining threads\n");
		}
		//	printf("thread done\n");
	}
	if (file_4)
	{
		if(pthread_join(rx4, (void**)&rx_statistics_4)) 
		{
			printf("Error joining threads\n");
		}
		//	printf("thread done\n");
	}
	//	printf("Threads joined!\n\n\n");
	/*Close files*/	
	//close(bram);

	if(ioctl(hls_read, STOP_DRIVER_TIMER, NULL) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}

	if(ioctl(hls_read, GET_DRIVER_STATISTICS, &driver_statistics) < 0) {
		printf("ERROR doing ioctl\n");
		return -1;
	}

	printf("****************************************************************************\n");
	printf("                        TRANSFER STATISTICS                                 \n");
	printf("****************************************************************************\n\n");

	//	printf("CDMA Usage Count: %d\n", tx_statistics_1->cdma_usage_cnt);
	total_bandwidth = 0;
	if (file_1)
	{
		tx_bandwidth = calc_BW((double)(tx_statistics_1->tx_bytes), (double)(tx_statistics_1->seconds), (double)(tx_statistics_1->ns));
		rx_bandwidth = calc_BW((double)(rx_statistics_1->rx_bytes), (double)(rx_statistics_1->seconds), (double)(rx_statistics_1->ns));
		printf("TX Byte Count of file 1 from the driver %d Bytes\n", tx_statistics_1->tx_bytes);
		//		printf("TX Time Elapsed:%lu sec :  %lu ns\n", tx_statistics_1->seconds, tx_statistics_1->ns);
		//		printf("TX Numer of times the IP was not ready: %d\n", tx_statistics_1->ip_not_ready);
		//		printf("TX Bandwidth of File 1: %f MB/s\n\n\n", tx_bandwidth);
		printf("RX Byte Count of file 1 from the driver %d Bytes\n", rx_statistics_1->rx_bytes);
		//		printf("RX Time Elapsed:%lu sec :  %lu ns\n", rx_statistics_1->seconds, rx_statistics_1->ns);
		//		printf("RX Bandwidth of File 1: %f MB/s\n\n\n", rx_bandwidth);

		total_bandwidth = total_bandwidth+(tx_bandwidth + rx_bandwidth);
	}
	if (file_2)
	{
		tx_bandwidth = calc_BW((double)(tx_statistics_2->tx_bytes), (double)(tx_statistics_2->seconds), (double)(tx_statistics_2->ns));
		rx_bandwidth = calc_BW((double)(rx_statistics_2->rx_bytes), (double)(rx_statistics_2->seconds), (double)(rx_statistics_2->ns));
		printf("TX Byte Count of file 2 from the driver %d Bytes\n", tx_statistics_2->tx_bytes);
		//		printf("TX Time Elapsed:%lu sec :  %lu ns\n", tx_statistics_2->seconds, tx_statistics_2->ns);
		//		printf("TX Numer of times the IP was not ready: %d\n", tx_statistics_2->ip_not_ready);
		//		printf("TX Bandwidth of File 2: %f MB/s\n\n\n", calc_BW((double)(tx_statistics_2->tx_bytes), (double)(tx_statistics_2->seconds), (double)(tx_statistics_2->ns)));
		printf("RX Byte Count of file 2 from the driver %d Bytes\n", rx_statistics_2->rx_bytes);
		//		printf("RX Time Elapsed:%lu sec :  %lu ns\n", rx_statistics_2->seconds, rx_statistics_2->ns);
		//		printf("RX Bandwidth of File 2: %f MB/s\n\n\n", calc_BW((double)(rx_statistics_2->rx_bytes), (double)(rx_statistics_2->seconds), (double)(rx_statistics_2->ns)));

		total_bandwidth = total_bandwidth+(tx_bandwidth + rx_bandwidth);
	}
	if (file_3)
	{
		tx_bandwidth = calc_BW((double)(tx_statistics_3->tx_bytes), (double)(tx_statistics_3->seconds), (double)(tx_statistics_3->ns));
		rx_bandwidth = calc_BW((double)(rx_statistics_3->rx_bytes), (double)(rx_statistics_3->seconds), (double)(rx_statistics_3->ns));
		printf("TX Byte Count of file 3 from the driver %d Bytes\n", tx_statistics_3->tx_bytes);
		//		printf("TX Time Elapsed:%lu sec :  %lu ns\n", tx_statistics_3->seconds, tx_statistics_3->ns);
		//		printf("TX Numer of times the IP was not ready: %d\n", tx_statistics_3->ip_not_ready);
		//		printf("TX Bandwidth of File 3: %f MB/s\n\n\n", calc_BW((double)(tx_statistics_3->tx_bytes), (double)(tx_statistics_3->seconds), (double)(tx_statistics_3->ns)));
		printf("RX Byte Count of file 3 from the driver %d Bytes\n", rx_statistics_3->rx_bytes);
		//		printf("RX Time Elapsed:%lu sec :  %lu ns\n", rx_statistics_3->seconds, rx_statistics_3->ns);
		//		printf("RX Bandwidth of File 3: %f MB/s\n\n\n", calc_BW((double)(rx_statistics_3->rx_bytes), (double)(rx_statistics_3->seconds), (double)(rx_statistics_3->ns)));
		total_bandwidth = total_bandwidth+(tx_bandwidth + rx_bandwidth);
	}
	if (file_4)
	{
		tx_bandwidth = calc_BW((double)(tx_statistics_4->tx_bytes), (double)(tx_statistics_4->seconds), (double)(tx_statistics_4->ns));
		rx_bandwidth = calc_BW((double)(rx_statistics_4->rx_bytes), (double)(rx_statistics_4->seconds), (double)(rx_statistics_4->ns));
		printf("TX Byte Count of file 4 from the driver %d Bytes\n", tx_statistics_4->tx_bytes);
		//		printf("TX Time Elapsed:%lu sec :  %lu ns\n", tx_statistics_4->seconds, tx_statistics_4->ns);
		//		printf("TX Numer of times the IP was not ready: %d\n", tx_statistics_4->ip_not_ready);
		//		printf("TX Bandwidth of File 4: %f MB/s\n\n\n", calc_BW((double)(tx_statistics_4->tx_bytes), (double)(tx_statistics_4->seconds), (double)(tx_statistics_4->ns)));
		printf("RX Byte Count of file 4 from the driver %d Bytes\n", rx_statistics_4->rx_bytes);
		//		printf("RX Time Elapsed:%lu sec :  %lu ns\n", rx_statistics_4->seconds, rx_statistics_4->ns);
		//		printf("RX Bandwidth of File 4: %f MB/s\n\n\n", calc_BW((double)(rx_statistics_4->rx_bytes), (double)(rx_statistics_4->seconds), (double)(rx_statistics_4->ns)));
		total_bandwidth = total_bandwidth+(tx_bandwidth + rx_bandwidth);
	}
	//	printf("\n\nCalculated Total Bandiwdth of Transaction is: %f MB/s\n", total_bandwidth);

	driver_bytes = (driver_statistics.tx_bytes) + (driver_statistics.rx_bytes);
	driver_bandwidth = calc_BW((double)(driver_bytes), (double)(driver_statistics.seconds), (double)(driver_statistics.ns));

	printf("Total Byte Count of Transaction:  %d Bytes\n", driver_bytes);
	//	printf("\n\nMeasured Total Bandiwdth of Transaction is: %f MB/s\n", driver_bandwidth);

	//	printf("****************************************************************************\n");
	//	printf("                           END STATISTICS                                   \n");
	//	printf("****************************************************************************\n\n");

	return driver_bandwidth;
}

void create_csv(char *filename, double a[4][5][10],int n,int m, int x){

	printf("\n Creating %s file",filename);

	FILE *fp;
	int amt;
	int i,j,k;

	//	filename=strcat(filename,".csv");
	fp=fopen(filename,"w+");

	for(k=0;k<x;k++)
	{
		fprintf(fp,"\n\n\n Number of Threads %d -------------------------------------------\n",k+1);

		for(i=0;i<m;i++){

			switch(i){
				case 0: amt = 4;
						break;
				case 1: amt = 8;
						break;
				case 2: amt = 16;
						break;
				case 3: amt = 32;
						break;
				case 4: amt = 64;
			}

			fprintf(fp,"\n\n%dK Transfer size\n",amt);

			for(j=0;j<n;j++)

				fprintf(fp," %f, ",a[k][i][j]);

		}
	}
	fclose(fp);

	printf("\n %s file created",filename);

}
