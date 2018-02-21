/**
 * System View Device Driver

 * @date		11/10/2015

 * @author	 System View Inc.

 * @file		 support_funcs.c

 * @brief			This file contains all the non char driver specific code. It contains all
 *				 the data movement code and xilinx specific IP controllers such as CDMA.
 ***********************************************************
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/version.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/sched.h>

#include <linux/slab.h>
#include <linux/msi.h>
#include <linux/poll.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/spinlock.h>
//#include <stdint.h>
#include "sv_driver.h"
//debug
#include <linux/time.h>


/******************************** Xilinx Register Offsets **********************************/

const u32 AXI_STREAM_ISR	 = 0x00;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
const u32 AXI_STREAM_IER	 = 0x04;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
const u32 AXI_STREAM_TDFR	= 0x08;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
const u32 AXI_STREAM_TDFV	= 0x0c;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
const u32 AXI_STREAM_TDFD	= 0x00;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
const u32 AXI_STREAM_TLR	 = 0x14;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
const u32 AXI_STREAM_RDFR	= 0x18;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
const u32 AXI_STREAM_RDFO	= 0x1C;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
const u32 AXI_STREAM_RDFD	= 0x1000; /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
const u32 AXI_STREAM_RLR	 = 0x24;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
const u32 AXI_STREAM_SRR	 = 0x28;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
const u32 AXI_STREAM_TDR	 = 0x2C;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
const u32 AXI_STREAM_RDR	 = 0x30;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
const u32 AXI_STREAM_TXID	= 0x34;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
const u32 AXI_STREAM_TXUSER = 0x38;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
const u32 AXI_STREAM_RXID	= 0x3C;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */

const u32 CDMA_CR			 = 0x00;	 /**< CDMA Register Offset (See Xilinx Doc) */
const u32 CDMA_SR			 = 0x04;	 /**< CDMA Register Offset (See Xilinx Doc) */
const u32 CDMA_CU_PTR		 = 0x08;	 /**< CDMA Register Offset (See Xilinx Doc) */
const u32 CDMA_CU_PTR_MSB	= 0x0C;	 /**< CDMA Register Offset (See Xilinx Doc) */
const u32 CDMA_TA_PTR		 = 0x10;	 /**< CDMA Register Offset (See Xilinx Doc) */
const u32 CDMA_TA_PTR_MSB	= 0x14;	 /**< CDMA Register Offset (See Xilinx Doc) */
const u32 CDMA_SA			 = 0x18;	 /**< CDMA Register Offset (See Xilinx Doc) */
const u32 CDMA_SA_MSB		 = 0x1C;	 /**< CDMA Register Offset (See Xilinx Doc) */
const u32 CDMA_DA			 = 0x20;	 /**< CDMA Register Offset (See Xilinx Doc) */
const u32 CDMA_DA_MSB		 = 0x24;	 /**< CDMA Register Offset (See Xilinx Doc) */
const u32 CDMA_BTT			 = 0x28;	 /**< CDMA Register Offset (See Xilinx Doc) */


const u32 PCIE_BRIDGE_INFO = 0x134;	 /**< AXI PCIe Subsystem Offset (See Xilinx Doc) */
const u32 PCIE_ISR_INFO	 = 0x138;	 /**< AXI PCIe Subsystem Offset (See Xilinx Doc) */
const u32 PCIE_PHY_STATUS = 0x144;	 /**< AXI PCIe Subsystem Offset (See Xilinx Doc) */

const u32 AXIBAR2PCIEBAR_0U = 0x208;	 /**< AXI PCIe Subsystem Offset (See Xilinx Doc) */
const u32 AXIBAR2PCIEBAR_0L = 0x20c;	 /**< AXI PCIe Subsystem Offset (See Xilinx Doc) */


const u32 AXIBAR2PCIEBAR_1L = 0x214;	 /**< AXI PCIe Subsystem Offset (See Xilinx Doc) */

/******************************** Local Scope Globals ***************************************/

static struct mutex xdma_h2c_sem[XDMA_CHANNEL_NUM_MAX];
static struct mutex xdma_c2h_sem[XDMA_CHANNEL_NUM_MAX];
static int xdma_query(int);
static int cdma_query(void);
static dma_addr_t xdma_h2c_buff[XDMA_CHANNEL_NUM_MAX];
static dma_addr_t xdma_c2h_buff[XDMA_CHANNEL_NUM_MAX];
static void * xdma_h2c_da[XDMA_CHANNEL_NUM_MAX];
static void * xdma_c2h_da[XDMA_CHANNEL_NUM_MAX];
static int cdma_use_count[CDMA_MAX_NUM];
/******************************** Support functions ***************************************/

/**
 * This function is will update a ring pointer given the amount of bytes written or read.
 * bytes_written The number of bytes to advance the ring pointer
 * ring_pointer_offset The current ring pointer offset.
 * file_size The size of the ring buffer to handle wrap around cases.
 */
int get_new_ring_pointer(int bytes_written, int ring_pointer_offset, int file_size)
{
	/* This function is common between wtk and WTH*/
	/*	This updates the pointer to be pointing to location
	 *		to take action on next*/

	//find new pointer and take wrap around into account
	if(ring_pointer_offset+bytes_written < file_size)
		return ring_pointer_offset + bytes_written;
	else
		return ring_pointer_offset + bytes_written - file_size;

}

/**
 * This function returns the amount of data/space available in the ring buffer
 * mod_desc The struct containing all the file variables.
 * tail The ring pointer offset at the tail of the ring buffer (starting point)
 * head The ring pointer offset at the head of the ring buffer (ending point)
 * priorty setting this to 1 gives full if head==tail.
 */
int room_in_buffer(int head, int tail, int full, size_t dma_size)
{

	if(tail == head) {
		if(full) // this is a full variable to solve which side can write (1 = wtk, 0 = wth)
			return 0;
		else
			return dma_size;

	} else if(head > tail) {
		return dma_size-head+tail;
	}
	//else tail > head
	return tail-head;

}

/**
 * This function returns the amount of data/space available in the ring buffer
 * mod_desc The struct containing all the file variables.
 * tail The ring pointer offset at the tail of the ring buffer (starting point)
 * head The ring pointer offset at the head of the ring buffer (ending point)
 * priorty setting this to 1 gives full if head==tail.
 */
int data_in_buffer(int head, int tail, int full, size_t dma_size)
{

	if(tail == head) {
		if(full) // this is a full variable to solve which side can write (1 = wtk, 0 = wth)
			return dma_size;
		else
			return 0;

	} else if(head > tail) {
		return head-tail;
	} else if(head < tail)	 //wrap around case, we will do another read call to come back and get wrap around data
		return dma_size-tail+head;

	return 0;
}


/**
 * This function is called by the read thread to read data from the FPGA.
 * read_size: the number of bytes to read from the fifo
 */
int read_data(struct mod_desc * mod_desc, int read_size)
{
	int read_count;
	int drop_count = 0;
	int drop_count_inc;
	int max_can_read = 0;
	int rfu, rfh, full;

	rfh = atomic_read(mod_desc->rfh);
	rfu = atomic_read(mod_desc->rfu);
	full = atomic_read(mod_desc->read_ring_buf_full);

	max_can_read = room_in_buffer(rfh, rfu, full, mod_desc->dma_size);

	//make sure there is room in our buffer for the read
	if(( read_size + dma_byte_width + 4*sizeof(size_t) ) > max_can_read) {
		//ring buffer is full, cannot fit in our read
		if(back_pressure) {
			//if this is a stream packet we cannot read out the whole packet return 1
			if(mod_desc->mode == AXI_STREAM_PACKET){
				verbose_printk(KERN_INFO"[read_data]: Packet size is larger than room in ring buffer, max_can_read: %d < read_size: %d \n", max_can_read, read_size);
				return 1;
			}
			//if this is not a stream packet read out what we can
			else {
				verbose_read_thread_printk(KERN_INFO"[read_data]: maximum read amount: %d\n", max_can_read);
				read_size = max_can_read - 4*sizeof(size_t) - dma_byte_width;
				verbose_read_thread_printk(KERN_INFO"[read_data]: read size: %d\n", read_size);
				if(read_size > 0){
					read_count = axi_stream_fifo_read((size_t)read_size, mod_desc->dma_read_addr, axi_pcie_m + mod_desc->dma_offset_read, mod_desc, rfh, mod_desc->dma_size);
					if(read_count < 0) {
						printk(KERN_INFO"[read_data]: !!!!!!!!ERROR reading data from axi stream fifo\n");
						//break;
						return ERROR;
					}
				}
				else {
					printk(KERN_INFO"[read_data]: No room in ring buffer to read data.\n");
					//break;
					return ERROR;
				}
			}

		} else {
			//----------------------------------------------drop data----------------------------------------------
			verbose_read_thread_printk(KERN_INFO"[read_data]: Needing to garbage data, max_can_read: %d < read_size: %d \n", max_can_read, read_size);

			while (drop_count < read_size) { //also garbage the trailer
				if(( read_size - drop_count + 2*sizeof(size_t) ) > dma_garbage_size){//garbage buffer cannot fit in our read
					drop_count_inc = axi_stream_fifo_read_direct((size_t)(dma_garbage_size - 2*sizeof(size_t)),
											 dma_buffer_base+dma_garbage_offset, axi_pcie_m + dma_garbage_offset, mod_desc, (size_t)dma_garbage_size);
				} else {	//read out the data
					drop_count_inc = axi_stream_fifo_read_direct((size_t)(read_size-drop_count),
											 dma_buffer_base + dma_garbage_offset, axi_pcie_m + dma_garbage_offset, mod_desc, (size_t)dma_garbage_size);
				}
				if(drop_count_inc == ERROR) {
					//This seems to happen when removing the driver, return error so we don't get stuck in the loop
					//look into the release code to fix
					printk(KERN_INFO"[read_data]: !!!!!!!!ERROR DROP_COUNT READ.....\n");
					return ERROR;
				}
				// extra debug message for drops
				verbose_read_thread_printk(KERN_INFO"[read_data]: drop loop DROP_COUNT_INC %d DROP COUNT: %d > READ_SIZE: %d.....\n", drop_count_inc, drop_count, read_size);
				drop_count += drop_count_inc;
			}

			if(drop_count > read_size) {
				printk(KERN_INFO"[read_data]: !!!!!!!!ERROR DROP COUNT: %d > READ_SIZE: %d.....\n", drop_count, read_size);
			}

			mod_desc->axi_fifo_rlr = mod_desc->axi_fifo_rdfo = 0;

			if(drop_count > 0) {
				verbose_read_thread_printk(KERN_INFO"[read_data]: DROP_COUNT: Just dropped %d bytes from HW\n", drop_count);
			} else {
				//this shouldn't happen
				printk(KERN_INFO"[read_data]: !!!!!!!!ERROR DROP_COUNT: < 1.....\n");
				//break;
				return ERROR;
			}
		}
	} else {
		//the ring buffer has room!
		verbose_read_thread_printk(KERN_INFO"[read_data]: maximum read amount: %d\n", max_can_read);
		verbose_read_thread_printk(KERN_INFO"[read_data]: read size: %d\n", read_size);

		read_count = axi_stream_fifo_read((size_t)read_size, mod_desc->dma_read_addr,
						  axi_pcie_m + mod_desc->dma_offset_read, mod_desc, rfh, mod_desc->dma_size);

		if(read_count < 0) {
			printk(KERN_INFO"[read_data]: !!!!!!!!ERROR reading data from axi stream fifo\n");
			//break;
			return ERROR;
		}

	}
	//send signal to userspace poll()

	atomic_set(mod_desc->atomic_poll, 1);
	verbose_read_thread_printk(KERN_INFO"[read_data]: waking up the poll.....%d\n", mod_desc->minor);
	//wake_up(&wq_periph);
	wake_up(&mod_desc->poll_wq);

	return 0;
}



/**
 * This function operates on it's own thread after insmod. It is used to handle all AXI-Streaming
 * read transfers. It reads from the hardware and stores data in the Read Ring Buffer. It wakes up the
 * poll method for the file upon receiving new data.
 * read_fifo the kernel FIFO data structure for the global read FIFO.
 */
int read_thread(void *in_param) {
	struct kfifo* read_fifo = (struct kfifo*)in_param;
	struct mod_desc * mod_desc;
	int read_incomplete;
	int d2r;
	unsigned int sz;

	verbose_read_thread_printk(KERN_INFO"[read_thread]: started !!\n");
	while(!kthread_should_stop()) {
		verbose_read_thread_printk(KERN_INFO"[read_thread]: waiting in wait_event_interruptible for data!!\n");
		//waits until thread_q_read is true or we should stop the thread (previous methods of exitings weren't working -MM)
		if( wait_event_interruptible(thread_q_head_read, ( atomic_read(&thread_q_read) == 1 || kthread_should_stop() )) ){

				}
		atomic_set(&thread_q_read, 0); // the threaded way
		verbose_read_thread_printk(KERN_INFO"[read_thread]: woke up the read thread have data!!\n");

		// process reads as long as there is something in the read fifo
		while (!kfifo_is_empty(read_fifo) && ! kthread_should_stop()) {
			//while the kfifo has data
			verbose_read_thread_printk(KERN_INFO"[read_thread]: read_fifo size:%d\n", kfifo_size(read_fifo));
			//read a mod_desc out of the fifo
			sz = kfifo_out(read_fifo, &mod_desc, 1);

			if(sz == 1 && mod_desc->file_open) {
				verbose_read_thread_printk(KERN_INFO"[read_thread]: kfifo_out returned non-zero\n");
				atomic_dec(mod_desc->in_read_fifo_count);


				//Check if there is any data to be read
				if(mod_desc->axi_fifo_rdfo > 0) {
					d2r = mod_desc->axi_fifo_rlr;
				} else {
					//else read it from the register
					d2r = axi_stream_fifo_d2r(mod_desc);
				}

				verbose_read_thread_printk(KERN_INFO"[read_thread]: mod_desc minor: %d d2r: %d\n", mod_desc->minor, d2r);
				while(d2r != 0 && !kthread_should_stop()) {
					read_incomplete = read_data(mod_desc, d2r);
					//if read_data needs back_pressure
					if(read_incomplete == 1 || read_incomplete == ERROR) {
						//write the mod_desc back in if the fifo is not full and it isn't already in the fifo
						if(!kfifo_is_full(read_fifo)) {
							atomic_inc(mod_desc->in_read_fifo_count);
							kfifo_in_spinlocked(read_fifo, &mod_desc, 1, &fifo_lock_read);
						} else {
							printk(KERN_INFO"[read_thread]: kfifo is full, not writing mod desc\n");
						}

						//if we needed backpressure wait here for a read to take data out of the buffer so we have room -MM
						verbose_read_thread_printk(KERN_INFO"[read_thread]: ring buffer is full waiting in wait_event_interruptible!!\n");
										if( wait_event_interruptible(thread_q_head_read, ( atomic_read(&thread_q_read) == 1 || kthread_should_stop() )) ){

										}
						//waits until thread_q_read is true or we should stop the thread (previous methods of exitings weren't working -MM)
						atomic_set(&thread_q_read, 0);	// the threaded way

						schedule();
						//set d2r to zero to get out of while loop
										//if(mod_desc->mode == AXI_STREAM_PACKET){
										d2r = 0;
										//}
					} else if(mod_desc->file_open){
						//read more data if the file is open
						d2r = axi_stream_fifo_d2r(mod_desc);
						verbose_read_thread_printk(KERN_INFO"[read_thread]: read more! mod_desc minor: %d d2r: %d\n", mod_desc->minor, d2r);
					} else {
						d2r = 0;
						verbose_read_thread_printk(KERN_INFO"[read_thread]: file is closed: %d d2r: %d\n", mod_desc->minor, d2r);
					}
				}
				verbose_read_thread_printk(KERN_INFO"[read_thread]: No more data to read.....\n");
			} else if(sz == 1 && !mod_desc->file_open) {
				verbose_read_thread_printk(KERN_INFO"[read_thread]: file is not open\n");
				atomic_dec(mod_desc->in_read_fifo_count);
			} else {
				printk(KERN_INFO"[read_thread]: !!!!!!!!ERROR kfifo_out returned zeron\n");
			}
		}
		verbose_read_thread_printk(KERN_INFO"[read_thread]: read_fifo is empty!\n");
	}
	verbose_printk(KERN_INFO"[read_thread]: Leaving thread\n");
	return 0;
}

/**
 * This function operates on it's own thread after insmod. It is used to handle all AXI-Streaming
 * write transfers. It reads from the write ring buffers and writes the data to hardware.
 * write_fifo the kernel FIFO data structure for the global write FIFO.
 */
int write_thread(void *in_param) {
	struct kfifo* write_fifo = (struct kfifo*)in_param;
	struct mod_desc * mod_desc;
	int write_incomplete;
	int d2w;
	int wth, wtk, full;
	unsigned int sz;


	verbose_write_thread_printk(KERN_INFO"[write_thread]: started !!\n");
	while(!kthread_should_stop()){
		verbose_write_thread_printk(KERN_INFO"[write_thread]: waiting in wait_event_interruptible, write buffer empty!!\n");
		if( wait_event_interruptible(thread_q_head_write, ( atomic_read(&thread_q_write) == 1 || kthread_should_stop() )) ) {

		}
		atomic_set(&thread_q_write, 0); // the threaded way
		verbose_write_thread_printk(KERN_INFO"[write_thread]: woke up the write thread!!\n");

		// process write as long as there is something in the write fifo
		while (!kfifo_is_empty(write_fifo) && ! kthread_should_stop()) {
			verbose_write_thread_printk(KERN_INFO"[write_thread]: write_fifo size:%d\n", kfifo_size(write_fifo));
			//read a mod_desc out of the fifo
			sz = kfifo_out(write_fifo, &mod_desc, 1);

			if(sz == 1 && mod_desc->file_open) {
				atomic_dec(mod_desc->in_write_fifo_count);
				verbose_write_thread_printk(KERN_INFO"[write_thread]: kfifo_out returned non-zero\n");

				//Check if there is any data to be written
				wth = atomic_read(mod_desc->wth);
				wtk = atomic_read(mod_desc->wtk);
				full = atomic_read(mod_desc->write_ring_buf_full);

				//d2w = data_to_transfer(mod_desc, wth, wtk, priority);
				d2w = data_in_buffer(wtk, wth, full, mod_desc->dma_size);

				verbose_write_thread_printk(KERN_INFO"[write_thread]: mod_desc minor: %d d2w: %d\n", mod_desc->minor, d2w);
				while(d2w != 0 && !kthread_should_stop()) {
					write_incomplete = write_data(mod_desc);
					//if read_data needs back_pressure
					if((write_incomplete == 1 || write_incomplete == ERROR) && mod_desc->file_open) {
						//write the mod_desc back in if the fifo is not full and it isn't already in the fifo
						if(!kfifo_is_full(write_fifo)) {
							atomic_inc(mod_desc->in_write_fifo_count);
							kfifo_in_spinlocked(write_fifo, &mod_desc, 1, &fifo_lock_write);
						}
						else {
							printk(KERN_INFO"[write_thread]: kfifo is full, not writing mod desc\n");
						}

						//if we needed backpressure wait here for a write to take data out of the buffer so we have room -MM
						verbose_write_thread_printk(KERN_INFO"[write_thread]: waiting in wait_event_interruptible write buffer full!!\n");
						wait_event_interruptible_timeout(thread_q_head_write,
										 ( atomic_read(&thread_q_write) == 1 || kthread_should_stop() ),
										 msecs_to_jiffies(1));

						//waits until thread_q_write is true or we should stop the thread (previous methods of exitings weren't working -MM)
						atomic_set(&thread_q_write, 0);	// the threaded way

						schedule();
						//set d2w to zero to get out of while loop
						d2w = 0;
					} else if(mod_desc->file_open) {
						//if write was complete and file is open write more data!
						//Check if there is any data to be written
						wth = atomic_read(mod_desc->wth);
						wtk = atomic_read(mod_desc->wtk);
						full = atomic_read(mod_desc->write_ring_buf_full);

						d2w = data_in_buffer(wtk, wth, full, mod_desc->dma_size);
						verbose_write_thread_printk(KERN_INFO"[write_thread]: write more! mod_desc minor: %d d2w: %d\n", mod_desc->minor, d2w);
					} else {
						d2w = 0;
						verbose_write_thread_printk(KERN_INFO"[write_thread]: file is closed: %d d2w: %d\n", mod_desc->minor, d2w);
					}
				}
				verbose_write_thread_printk(KERN_INFO"[write_thread]: No more data to write.....\n");
			} else if(sz == 1 && !mod_desc->file_open) {
				verbose_write_thread_printk(KERN_INFO"[write_thread]: file is not open!\n");
				atomic_dec(mod_desc->in_write_fifo_count);
			} else {
				printk(KERN_INFO"[write_thread]: !!!!!!!!ERROR kfifo_out returned zero\n");
			}
		}
		verbose_write_thread_printk(KERN_INFO"[write_thread]: write_fifo is empty!\n");
	}
	verbose_printk(KERN_INFO"[write_thread]: Leaving write thread\n");
	return 0;
}

/**
 * @brief This function is used to initiate a CDMA Transfer. It requires
 * a locked CDMA resource an AXI Start Address, AXI Destination Address,
 * and a Keyhole setting. It breaks up the transfer into multiple transfers


 * if the max_read / write sizes are non zero
 * @param SA 64bit Starting Address of DMA transfer.
 * @param DA 64bit Destination Address of DMA transfer.
 * @param BTT Number of bytes to transfer.
 * @param keyhole_en Instructs the CDMA to to a keyhole transaction or not
*/
static int dma_transfer(u64 l_sa, u64 l_da, u32 l_btt, int keyhole_en, u32 xfer_type)
{
	u32 max_xfer_size;
	int ret = 0, cdma_num = -1, xdma_channel = -1, attempts = 10000;
	struct sg_table sg_table;
	struct scatterlist sg;

	verbose_dma_printk(KERN_INFO"\t\t[dma_transfer]: **** Starting XDMA transfer ****\n");

	// if xdma to be used
	if(pcie_use_xdma) {
		verbose_dma_printk(KERN_INFO"\t\t[dma_transfer]: Using XDMA \n");

		if(xfer_type & HOST_READ) { // host to device
			// get a free channel
			while (xdma_channel == -1 && --attempts != 0) {
				xdma_channel = xdma_query(DMA_TO_DEVICE);
				if(xdma_channel == -1) schedule();
			}

			// break up transfers to size
			do {
				int xfer_size = l_btt > dma_max_read_size ? dma_max_read_size : l_btt;
				// create the scatter gather list
				sg_init_table(&sg, 1);
				sg_dma_len(&sg) = xfer_size;
				sg_table.sgl = &sg;
				sg_table.nents = 1;
				//xfer_mem = (char *)dma_buffer_base + ((char *)l_sa - (char *)dma_addr_base);
				//memcpy(xdma_h2c_da[xdma_channel],xfer_mem,xfer_size);
				sg_dma_address(&sg) = l_sa;//(dma_addr_t)xdma_h2c_buff[xdma_channel];

				verbose_dma_printk(KERN_INFO"\t\t[dma_transfer] DMA_TO_DEVICE l_sa: 0x%p, l_da: 0x%p, xdma_b: 0x%p, xfer_size: %d, BTT: %d\n",
								(void*)l_sa, (void*)l_da, (void*)xdma_h2c_buff[xdma_channel], xfer_size, l_btt);

				ret = xdma_xfer_submit(xdma_channel_list[xdma_channel].h2c,
						       DMA_TO_DEVICE,
						       l_da,
						       &sg_table,
						       true,
						       XDMA_TIMEOUT_IN_MSEC);
				if (ret < 0) {
					mutex_unlock(&xdma_h2c_sem[xdma_channel]);
					return ret;
				}
				if(xfer_type & INC_SA) l_sa += xfer_size;
				if(xfer_type & INC_DA) l_da += xfer_size;
				l_btt -= xfer_size;
				verbose_dma_printk(KERN_INFO"\t\t[dma_transfer]: tranfer complete DMA_TO_DEVICE remain %d\n", l_btt);
			} while (l_btt);
			mutex_unlock(&xdma_h2c_sem[xdma_channel]);
			verbose_dmaq_printk(KERN_INFO"\t\t\t[dma_queue]: unlock xdma_channel = %d \n", xdma_channel);
		} else if(xfer_type & HOST_WRITE) { // device to host
			while (xdma_channel == -1 && --attempts != 0) {
				xdma_channel = xdma_query(DMA_FROM_DEVICE);
				if(xdma_channel == -1) schedule();
			}
			// break up transfers to size
			do {
				int xfer_size = l_btt > dma_max_read_size ? dma_max_read_size : l_btt;
				// create the scatter gather list
				sg_init_table(&sg, 1);
				sg_dma_len(&sg) = l_btt;
				sg_table.sgl = &sg;
				sg_table.nents = 1;
				sg_dma_address(&sg) = (dma_addr_t)xdma_c2h_buff;
				verbose_dma_printk(KERN_INFO"\t\t[dma_transfer]: DMA_FROM_DEVICE l_sa 0x%p l_da 0x%p xfer_size %d\n", (void*)l_sa, (void*)l_da, xfer_size);
				ret = xdma_xfer_submit(xdma_channel_list[xdma_channel].c2h, DMA_FROM_DEVICE, l_sa, &sg_table, true, XDMA_TIMEOUT_IN_MSEC);
				if(ret < 0) {
					mutex_unlock(&xdma_c2h_sem[xdma_channel]);
					return ret;
				}
				//xfer_mem = (char *)dma_buffer_base + ((char *)l_da - (char *)dma_addr_base);
				//memcpy(xfer_mem, xdma_c2h_da[xdma_channel],xfer_size);
				if (xfer_type & INC_SA) l_sa += xfer_size;
				if (xfer_type & INC_DA) l_da += xfer_size;
				l_btt -= xfer_size;
				verbose_dma_printk(KERN_INFO"\t\t[dma_transfer]: tranfer complete DMA_FROM_DEVICE\n");
			} while (l_btt);
			mutex_unlock(&xdma_c2h_sem[xdma_channel]);
			verbose_dma_printk(KERN_INFO"\t\t\t[dma_transfer]: unlock xdma_channel = %d \n", xdma_channel);
		}
		return (ret < 0 ? ret : 0);
	}
	else if(cdma_capable != 0) {
		verbose_dma_printk(KERN_INFO"\t\t[dma_transfer]: Using CDMA \n");
		/* Find an available CDMA to use and wait if both are in use */
	cdma_retry:
		while(cdma_num == -1 && --attempts != 0) {
			cdma_num = cdma_query();
			schedule();
		}
		if (cdma_num == -1) {
			printk(KERN_INFO"\t\t[dma_transfer]: could not get cdma will sleep 10 msecs cdma_use(%d,%d,%d,%d)\n",
			       cdma_use_count[0],cdma_use_count[1],cdma_use_count[2],cdma_use_count[3]);
			if (xfer_type & HOST_WRITE)
				wait_event_interruptible_timeout(thread_q_head_write,1,msecs_to_jiffies(10));
			else
				wait_event_interruptible_timeout(thread_q_head_read,1,msecs_to_jiffies(10));
			attempts = 1000;
			goto cdma_retry;
		}
		// if write check required and dma max write is set
		if((xfer_type & HOST_WRITE) && dma_max_write_size && l_btt > dma_max_write_size) {
			max_xfer_size = dma_max_write_size;
		} else if((xfer_type & HOST_READ) && dma_max_read_size && l_btt > dma_max_read_size) {
			max_xfer_size = dma_max_read_size;
		} else {
			max_xfer_size = l_btt;
		}
		do {
			int xfer_size = l_btt > max_xfer_size ? max_xfer_size : l_btt;
			verbose_dma_printk(KERN_INFO"\t\t[dma_transfer]: l_sa 0x%p l_da 0x%p xfer_size %d\n", (void*)l_sa, (void*)l_da, xfer_size);
			ret = cdma_transfer(l_sa, l_da, xfer_size, keyhole_en, cdma_num);
			if(ret != 0) return ret;
			if(xfer_type & INC_SA) l_sa += xfer_size;
			if(xfer_type & INC_DA) l_da += xfer_size;
			l_btt -= xfer_size;
		} while (l_btt);
		/*Release Mutex on CDMA*/

		verbose_dma_printk(KERN_INFO"\t\t\t[dma_transfer]: Releasing Mutex on CDMA %d \n", cdma_num);
		mutex_unlock(&cdma_sem[cdma_num]);

		return ret;
	}
	else{
		verbose_printk(KERN_INFO"\t\t[dma_transfer]: !!!!!!!!ERROR: unknown transfer type.\n");
		return ERROR;
	}
}

/**
 * @brief This function is called by the write thread to write data to the FPGA that is in the ring buffer.
 * @param mod_desc The struct containing all the file variables.
 */
int write_data(struct mod_desc * mod_desc)
{
	int wtk, wth, full;
	int d2w;
	//int cdma_num;
	size_t room_till_end;
	size_t remaining;
	size_t write_header_size;
	u64 axi_dest;
	u32 buf, read_reg;
	u32 data_in_fifo;
	int attempts = 100;

	wth = atomic_read(mod_desc->wth);
	wtk = atomic_read(mod_desc->wtk);
	full = atomic_read(mod_desc->write_ring_buf_full);

	d2w = data_in_buffer(wtk, wth, full, mod_desc->dma_size);

	if( d2w > sizeof(write_header_size)) {

		room_till_end = mod_desc->dma_size - wth;
		//If the header will not fit in 1 chuck of the ring buffer (in the room till end) then just get it at the start
		if(sizeof(write_header_size) > room_till_end) {
			wth = 0;
		}

		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: header memcpy (0x%p, 0x%p, 0x%zx)\n", &write_header_size, mod_desc->dma_write_addr+wth, sizeof(write_header_size));
		memcpy(&write_header_size, mod_desc->dma_write_addr+wth, sizeof(write_header_size));
		wth = get_new_ring_pointer(sizeof(write_header_size), wth, (int)(mod_desc->dma_size));

		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: header: (%zx)\n", write_header_size);
		d2w -= sizeof(write_header_size);

	} else {
		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: no data to read\n");
		return 1;
	}

	//check the tx vacancy
	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_TDFV; // the transmit FIFO vacancy
	buf = 0x0;
	if( direct_read(axi_dest, (void*)(&buf), 4, NORMAL_READ) ) {
		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: !!!!!!!!ERROR reading from AXI Streaming FIFO control interface\n");
		return ERROR;
	}

	read_reg = (buf+4)*dma_byte_width;
	data_in_fifo = 32768 - read_reg;
	verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: vacancy: (0x%x)(bytes)\n", (u32)read_reg);
	verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: data_in_fifo: (0x%x)(bytes)\n", (u32)data_in_fifo);

	if(write_header_size > d2w) {
		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: header is bigger(%zd) than data to read(%d)! \n", write_header_size, d2w);
		return 1;
	} else if(write_header_size > read_reg) {
		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: header is bigger(%zd) than vacancy(%u)! \n", write_header_size, (u32)read_reg);
		return 1;
	}


	verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: vacancy: (0x%x)(bytes)\n", (u32)read_reg);
	verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: d2w: (0x%x)(bytes)\n", (u32)d2w);

	/*write to TDR register*/

	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_TDR;
	buf = mod_desc->tx_dest;
	verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: Wrote 0x%x to TDR!!.\n", mod_desc->tx_dest);
	if( direct_write(axi_dest, (void*)(&buf), 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[write_data]: !!!!!!!!ERROR writing to AXI Streaming FIFO Control Interface\n");
		return ERROR;
		}

	axi_dest = mod_desc->axi_addr + AXI_STREAM_TDFD;
	//now we want to cdma write_header_size amount of data

	room_till_end = ((mod_desc->dma_size - wth) & ~(dma_byte_width-1));					 //make it divisible by dma_byte_width
	if(write_header_size > room_till_end) {			//needs to be 2 copy
		remaining = write_header_size-room_till_end;

		verbose_axi_fifo_write_printk(KERN_INFO"[write_%d_data]: dma_transfer 1 ring_buff address: 0x%llx + 0x%x, AXI Address: %llx, len: 0x%zx)\n",
							mod_desc->minor, axi_pcie_m+mod_desc->dma_offset_write, wth, axi_dest, room_till_end);
		if(dma_transfer(axi_pcie_m+mod_desc->dma_offset_write+wth, axi_dest, room_till_end, KEYHOLE_WRITE, (HOST_READ|INC_SA))) { //unsuccessful CDMA transmission
			verbose_printk(KERN_INFO"[write_data]: !!!!!!!!ERROR on CDMA READ!!!.\n");
		}
		//extra verbose debug message
		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: first 4 bytes 0x%08x\n", *((u32*)(mod_desc->dma_write_addr+wth)));



		//extra verbose debug message
		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: first 4 bytes 0x%08x\n", *((u32*)(mod_desc->dma_write_addr+wth)));
		wth = 0;

		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: dma_transfer 2 ring_buff address: 0x%llx + 0x%x, AXI Address: %llx, len: 0x%zx)\n",
							axi_pcie_m+mod_desc->dma_offset_write, wth, axi_dest, remaining);
		if(dma_transfer(axi_pcie_m+mod_desc->dma_offset_write+wth, axi_dest, remaining, KEYHOLE_WRITE, (HOST_READ|INC_SA))) { //unsuccessful CDMA transmission
			verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: !!!!!!!!ERROR on CDMA READ!!!.\n");
		}
		wth = get_new_ring_pointer(remaining, wth, (int)(mod_desc->dma_size));

	} else {														//only 1 copy
		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: dma_transfer ring_buff address: 0x%llx + 0x%x, AXI Address: %llx, len: 0x%zx)\n",
							axi_pcie_m+mod_desc->dma_offset_write, wth, axi_dest, write_header_size);
		if(dma_transfer(axi_pcie_m+mod_desc->dma_offset_write+wth, axi_dest, write_header_size, KEYHOLE_WRITE, (HOST_READ|INC_SA))) { //unsuccessful CDMA transmission
			verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: !!!!!!!!ERROR on CDMA READ!!!.\n");
		}

		//extra verbose debug message
		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: first 4 bytes 0x%08x\n", *((u32*)(mod_desc->dma_write_addr+wth)));
		wth = get_new_ring_pointer(write_header_size, wth, (int)(mod_desc->dma_size));
	}


	// Make sure there is enough data in the fifo to transmit
	do {

		//check the tx vacancy
		axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_TDFV; // the transmit FIFO vacancy
		buf = 0x0;
		if( direct_read(axi_dest, (void*)(&buf), 4, NORMAL_READ) ) {
			verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: !!!!!!!!ERROR reading from AXI Streaming FIFO control interface\n");
			return ERROR;
		}

		read_reg = (buf+4)*dma_byte_width;
		data_in_fifo = 32768 - read_reg;
		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: vacancy: (0x%x)(bytes)\n", (u32)read_reg);
		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: data_in_fifo: (0x%x)(bytes)\n", (u32)data_in_fifo);

		attempts--;

	} while(data_in_fifo < d2w && attempts > 0);

	if(data_in_fifo < d2w) {
		printk(KERN_INFO"[write_data]: ERROR not enough data in the fifo to write!\n");
		return 1;
	}


	/*write to ctl interface*/
	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_TLR;
	buf = (u32)write_header_size;
	verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: Wrote 0x%x to TLR!!.\n", (u32)write_header_size);
	if( direct_write(axi_dest, (void*)(&buf), 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[write_data]: !!!!!!!!ERROR writing to AXI Streaming FIFO Control Interface\n");
		return ERROR;
	}

	//Report status after transfer
	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_ISR;
	if( direct_read(axi_dest, (void*)(&buf), 4, NORMAL_READ) ) {
		printk(KERN_INFO"[write_data]: !!!!!!!!ERROR reading AXI_STREAM_ISR register.\n");
		return ERROR;
	}
	verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: AXI_STREAM_ISR register: ('0x%08x')\n", buf);

	//update wth pointer
	atomic_set(mod_desc->wth, wth);
	//if the ring buffer was full it is no longer
	if(full) {
		atomic_set(mod_desc->write_ring_buf_full, 0);
		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: ring_point_%d : write full: %d\n", mod_desc->minor, 0);
	}

	verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: write ring_buffer: WTH: %d WTK: %d\n", wth, atomic_read(mod_desc->wtk));

	atomic_set(mod_desc->pci_write_q, 1);
	wake_up_interruptible(&pci_write_head);

	verbose_axi_fifo_write_printk(KERN_INFO"[axi_stream_fifo_write]: Leaving write_data\n");

	return 0;

}


/*
 *
 *
 *
 */
struct task_struct* create_thread_write(struct kfifo * write_fifo) {

	struct task_struct * kthread_heap;
	kthread_heap = kthread_create(write_thread, write_fifo, "vsi_write_thread");

	if((kthread_heap)) {
		printk(KERN_INFO"[vsi_init]: Write Thread Created\n");
		wake_up_process(kthread_heap);
	}
	return kthread_heap;
}

struct task_struct* create_thread_read(struct kfifo * read_fifo) {

	struct task_struct * kthread_heap;
	kthread_heap = kthread_create(read_thread, read_fifo, "vsi_read_thread");

	if((kthread_heap)) {
		printk(KERN_INFO"[vsi_init]: Read Thread Created\n");
		wake_up_process(kthread_heap);
	}
	return kthread_heap;
}


/**
 * This function allocates the DMA regions for the peripheral.
 * mod_desc the struct containing all the file variables.
 * dma_buffer_base the current DMA allocated region offset to be assigned
 * dma_buffer_size The size of the DMA buffer.
 */
int dma_file_init(struct mod_desc *mod_desc, char *dma_buffer_base, u64 dma_buffer_size) {

	int dma_file_size;
	dma_file_size = (int)(mod_desc->dma_size);//*RING_BUFF_SIZE_MULTIPLIER; //we want the ring buffer to be atleast 2 times the size of the file size (aka fifo size)

	printk(KERN_INFO"[dma_file_init]: Setting Peripheral DMA size:%d, current offset %lld\n", dma_file_size, (u64)dma_current_offset);
	mod_desc->dma_size = (size_t)dma_file_size;

	//if(((u64)dma_current_offset + dma_file_size) > (u64)((char*)dma_buffer_base + dma_buffer_size))
	if(((u64)dma_current_offset + (2*dma_file_size)) > (u64)dma_buffer_size) {
		printk(KERN_INFO"[dma_file_init]: !!!!!!!!ERROR! DMA Buffer out of memory!\n");
		printk(KERN_INFO"[dma_file_init]: Decrease file sizes or increase dma size insmod parameter\n");
		return ERROR;
	} else {
		verbose_printk(KERN_INFO"[dma_file_init]: The current system memory dma offset:0x%x\n", dma_current_offset);
		mod_desc->dma_offset_read = dma_current_offset;				//set the dma start address for the peripheral read
		mod_desc->dma_offset_write = dma_current_offset + (u32)dma_file_size; //set the dma start address for the peripheral write

		mod_desc->dma_read_addr = (dma_buffer_base + (u64)dma_current_offset);	//actual pointer to kernel buffer
		verbose_printk(KERN_INFO"[dma_file_init]: DMA kernel read address set to:0x%p\n", mod_desc->dma_read_addr);
		mod_desc->dma_write_addr = (dma_buffer_base + (u64)dma_current_offset + dma_file_size);				//actual pointer to kernel buffer
		verbose_printk(KERN_INFO"[dma_file_init]: DMA kernel write address set to:0x%p\n", mod_desc->dma_write_addr);

		dma_current_offset += (u32)(2*dma_file_size);				//update the current dma allocation pointer, 2 buffers (R/W)
		//dma_current_offset = dma_current_offset + (u32)(dma_file_size);				//update the current dma allocation pointer, 1 Buffer)
		verbose_printk(KERN_INFO"[dma_file_init]: Success setting peripheral DMA\n");
	}
	mod_desc->set_dma_flag = 1;
	printk(KERN_INFO"[dma_file_init]: Success setting peripheral DMA for file minor: %d size %d\n", mod_desc->minor, dma_file_size);

	return 0;
}

/**
 * @brief This function initialized the interrupt controller in the FPGA.
 * @param axi_address The 64b AXI address of the Interrupt Controller (set through insmod).
 */
void int_ctlr_init(u64 axi_address)
{
	u32 status;
	u64 axi_dest;

	printk(KERN_INFO"[int_ctlr_init]: Setting Interrupt Controller Axi Address\n");
	axi_interr_ctrl = axi_address;

	/*Write to Interrupt Enable Register (IER)*/
	/* Write to enable all possible interrupt inputs */
	status = 0xFFFFFFFF;
	axi_dest = axi_interr_ctrl + INT_CTRL_IER;
	if( direct_write(axi_dest, (void *)&status, 4, NORMAL_WRITE) ) {
				printk(KERN_INFO"[int_ctlr_init]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n");
				return;
		}

	/*Write to the Master Enable Register (MER) */
	/* Write to enable the hardware interrupts */
	status = 0x3;
	axi_dest = axi_interr_ctrl + INT_CTRL_MER;
	if( direct_write(axi_dest, (void *)&status, 4, NORMAL_WRITE) ) {
				printk(KERN_INFO"[int_ctlr_init]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n");
				return;
		}

	if( direct_read(axi_dest, (void *)&status, 4, NORMAL_READ) ) {
				printk(KERN_INFO"[int_ctlr_init]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n");
				return;
		}
	printk(KERN_INFO"[int_ctlr_init]: read: ('0x%08x') from MER register\n", status);

	/*Here we need to clear the service interrupt in the interrupt acknowledge register*/
	status = 0xFFFFFFFF;
	axi_dest = axi_interr_ctrl + INT_CTRL_IAR;
	if( direct_write(axi_dest, (void *)&status, 4, NORMAL_WRITE) ) {
				printk(KERN_INFO"[int_ctlr_init]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n");
				return;
		}
}

/*
 *
 *
 *
 */
int pcie_ctl_init(u64 axi_pcie_ctl, u64 dma_addr_base)
{
	u32 dma_addr_loc;
	u64 axi_dest;
	u32 status;

	verbose_printk(KERN_INFO"[pcie_ctl_init]: Setting PCIe Control Axi Address\n");

	if(cdma_set[0] == 1) {
		//update pcie_ctl_virt address with register offset
		axi_dest = axi_pcie_ctl + AXIBAR2PCIEBAR_0U;

		/*convert to upper u32 to send to CDMA*/
		dma_addr_loc = (u32)(dma_addr_base>>32);

		//write DMA addr to PCIe CTL upper for address translation
		if( direct_write(axi_dest, (void *)(&dma_addr_loc), 4, NORMAL_WRITE) ) {
			printk(KERN_INFO"[pcie_ctl_init]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n");
			return ERROR;
		}
		printk(KERN_INFO"[pcie_ctl_init]: writing dma address ('0x%08x') to pcie_ctl upper at AXI address:%llx\n", dma_addr_loc, axi_dest);


		//check the pcie-ctl got the translation address
		if( direct_read(axi_dest, (void *)&status, 4, NORMAL_READ) ) {
			printk(KERN_INFO"[pcie_ctl_init]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n");
			return ERROR;
		}
		if(status == dma_addr_loc)
			verbose_printk(KERN_INFO"[pcie_ctl_init]: PCIe CTL upper register set SUCCESS: ('0x%08x')\n", status);
		else {
			printk(KERN_INFO"[pcie_ctl_init]: \t!!!!!!!!!!! PCIe CTL upper register FAILURE !!!!!!!!!!!!!!!: ('0x%08x')\n", status);
			printk(KERN_INFO"[pcie_ctl_init]: Most likely due to incorrect setting in PCIe IP for AXI to BAR translation HIGH address\n");
			printk(KERN_INFO"[pcie_ctl_init]: Must set this value to be the DMA buffer size allocation.\n");
			return ERROR;
		}


		//update pcie_ctl_virt lower address with register offset
		axi_dest = axi_pcie_ctl + AXIBAR2PCIEBAR_0L;

		/*convert to lower u32 to send to CDMA*/
		dma_addr_loc = (u32)dma_addr_base;

		//write DMA addr to PCIe CTL for address translation
		if( direct_write(axi_dest, (void *)(&dma_addr_loc), 4, NORMAL_WRITE) ) {
					printk(KERN_INFO"[pcie_ctl_init]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n");
					return ERROR;
				}
		printk(KERN_INFO"[pcie_ctl_init]: writing dma address ('0x%08x') to pcie_ctl lower at AXI address:%llx\n", dma_addr_loc, axi_dest);

		//check the pcie-ctl got the translation address
		if( direct_read(axi_dest, (void *)&status, 4, NORMAL_READ) ) {
					printk(KERN_INFO"[pcie_ctl_init]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n");
					return ERROR;
				}
		if(status == dma_addr_loc)
			verbose_printk(KERN_INFO"[pci_ioctl_cdma_set]: PCIe CTL register lower set SUCCESS: ('0x%08x')\n", status);
		else {
			printk(KERN_INFO"[pcie_ctl_init]: \t!!!!!!!!!!! PCIe CTL lower register FAILURE !!!!!!!!!!!!!!!: ('0x%08x')\n", status);
			printk(KERN_INFO"[pcie_ctl_init]: Most likely due to incorrect setting in PCIe IP for AXI to BAR translation HIGH address\n");
			printk(KERN_INFO"[pcie_ctl_init]: Must set this value to be the DMA buffer size allocation.\n");
			return ERROR;
		}

		//update pcie_ctl_virt address with register offset
		axi_dest = axi_pcie_ctl + AXIBAR2PCIEBAR_0U;

		/*convert to upper u32 to send to CDMA*/
		dma_addr_loc = (u32)(dma_addr_base>>32);

		//write DMA addr to PCIe CTL upper for address translation
		if( direct_write(axi_dest, (void *)(&dma_addr_loc), 4, NORMAL_WRITE) ) {
					printk(KERN_INFO"[pcie_ctl_init]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n");
					return ERROR;
				}





	}
	return 0;
}

/**
 * This function will intiialize the XDMA channels
 */
int xdma_init_sv(int num_channels)
{
	int i;
	for (i = 0 ; i < num_channels; i++) {
		mutex_init(&xdma_h2c_sem[i]);
		mutex_init(&xdma_c2h_sem[i]);
		xdma_h2c_da[i] = dma_alloc_coherent(NULL, (size_t)4096, &xdma_h2c_buff[i], GFP_KERNEL);
		xdma_c2h_da[i] = dma_alloc_coherent(NULL, (size_t)4096, &xdma_c2h_buff[i], GFP_KERNEL);
		if (!xdma_c2h_da[i] || !xdma_h2c_da[i]) {
			verbose_printk(KERN_INFO"vsi_driver:[xdma_init] cannot allocated xdma buffer 0x%p 0x%p\n",
				       (void *)xdma_c2h_da[i], (void *)xdma_h2c_da[i]);
		} else {
			verbose_printk(KERN_INFO"vsi_driver:[xdma_init] callocated xdma buffer 0x%p 0x%p\n",
				       (void *)xdma_c2h_da[i],(void *)xdma_h2c_da[i]);
		}
	}
	return 0;
}


/**
 * This function initializes the CDMA.
 * cdma_num Instructs which CDMA to use (Assumes it has been locked)
 * cdma_address the AXI address of the CDMA
 * dma_addr_base Used to set the dma translation address to the PCIe control reg.
 */
int cdma_init(int cdma_num, uint cdma_addr)
{
	u64 axi_dest;
	u32 cdma_status;

	if( (cdma_num < 0) | (cdma_num > CDMA_MAX_NUM) ) {
		printk(KERN_INFO"\t\t[cdma_0x%x_init]: \t!!!!!!!!ERROR: incorrect CDMA number detected!!!!!!!\n", cdma_num);
		return ERROR;
	}

	cdma_address[cdma_num] = cdma_addr;
	mutex_init(&cdma_sem[cdma_num]);

	printk(KERN_INFO"\t\t[cdma_0x%x_init]: *******************Setting CDMA AXI Address:%x ******************************************\n", cdma_num, cdma_address[cdma_num]);


	//wait for reset to finish	axi_dest = cdma_addr + CDMA_CR;
	axi_dest = cdma_addr + CDMA_CR;
	if( direct_read(axi_dest, (void *)&cdma_status, 4, NORMAL_READ) ) {
		printk(KERN_INFO"\t\t[cdma_idle_%x_init]: \t!!!!!!!!ERROR: in direct_read 0x%p !!!!!!!\n", cdma_num, axi_dest);
		return ERROR;
	}
	printk(KERN_INFO"\t\t[cdma_0x%x_init]: CDMA configuration before reset: ('0x%08x')\n", cdma_num, cdma_status);


	/*Issue a Soft Reset*/
	axi_dest = cdma_addr + CDMA_CR;
	cdma_status = 0x00000004;
	printk(KERN_INFO"\t\t[cdma_0x%x_init]: sending a soft reset to the CDMA\n", cdma_num);
	if( direct_write(axi_dest, (void *)&cdma_status, 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"\t\t[cdma_idle_%x_init]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n", cdma_num);
		return ERROR;
	}

	//wait for reset to finish
	if( direct_read(axi_dest, (void *)&cdma_status, 4, NORMAL_READ) ) {
		printk(KERN_INFO"\t\t[cdma_idle_%x_init]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n", cdma_num);
		return ERROR;
	}
	printk(KERN_INFO"\t\t[cdma_0x%x_init]: CDMA configuration after reset: ('0x%08x')\n", cdma_num, cdma_status);



	/*Check the current status*/
	axi_dest = cdma_addr + CDMA_SR;
	if( direct_read(axi_dest, (void *)&cdma_status, 4, NORMAL_READ) ) {
		printk(KERN_INFO"\t\t[cdma_idle_%x_init]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n", cdma_num);
		return ERROR;
	}

	printk(KERN_INFO"\t\t[cdma_0x%x_init]: CDMA status before configuring: ('0x%08x')\n", cdma_num, cdma_status);

	/*Check the current config*/
	axi_dest = cdma_addr + CDMA_CR;
	if( direct_read(axi_dest, (void *)&cdma_status, 4, NORMAL_READ) ) {
		printk(KERN_INFO"\t\t[cdma_idle_%x_init]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n", cdma_num);
		return ERROR;
	}
	printk(KERN_INFO"\t\t[cdma_0x%x_init]: CDMA config before configuring: ('0x%08x')\n", cdma_num, cdma_status);

	/*clear any pre existing interrupt*/
	axi_dest = cdma_addr + CDMA_SR;
	cdma_status = 0x00001000;
	printk(KERN_INFO"\t\t[cdma_0x%x_init]: attempting to write: ('0x%08x') to cdma status reg\n", cdma_num, cdma_status);
	if( direct_write(axi_dest, (void *)&cdma_status, 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"\t\t[cdma_idle_%x_init]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n", cdma_num);
		return ERROR;
	}

	/*set the interrupt on complete bit*/
	//	cdma_status = 0x00001000;
	//	axi_dest = cdma_addr + CDMA_CR;
	//	if( direct_write(axi_dest, (void *)&cdma_status, 4, NORMAL_WRITE) ) {
		//			printk(KERN_INFO"\t\t[cdma_idle_%x_init]: \t!!!!!!!!ERROR: in data_transfer!!!!!!!\n", cdma_num);
		//			return ERROR;
		//	}

	/*Check the current status*/
	axi_dest = cdma_addr + CDMA_SR;
	if( direct_read(axi_dest, (void *)&cdma_status, 4, NORMAL_READ) ) {
		printk(KERN_INFO"\t\t[cdma_idle_%x_init]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n", cdma_num);
		return ERROR;
	}
	printk(KERN_INFO"\t\t[cdma_0x%x_init]: CDMA status after configuring: ('0x%08x')\n", cdma_num, cdma_status);

	/*Check the current status*/
	axi_dest = cdma_addr + CDMA_SR;
	if( data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_READ, 0) ) {
		printk(KERN_INFO"\t\t[cdma_idle_%x_init]: \t!!!!!!!!ERROR: in data_transfer!!!!!!!\n", cdma_num);
		return ERROR;
	}
	printk(KERN_INFO"\t\t[cdma_0x%x_init]: CDMA status after configuring(data_transfer): ('0x%08x')\n", cdma_num, cdma_status);

	/*Check the current config*/
	axi_dest = cdma_addr + CDMA_CR;
	if( direct_read(axi_dest, (void *)&cdma_status, 4, NORMAL_READ) ) {
		printk(KERN_INFO"\t\t[cdma_idle_%x_init]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n", cdma_num);
		return ERROR;
	}
	printk(KERN_INFO"\t\t[cdma_0x%x_init]: CDMA config after configuring: ('0x%08x')\n", cdma_num, cdma_status);
	printk(KERN_INFO"\t\t[cdma_0x%x_init]: ****************Setting CDMA AXI Address:%x Complete*************************************\n", cdma_num, cdma_addr);

	cdma_set[cdma_num] = 1;

	return 0;
}

/*
 *
 *
 *
 */
int vec2num(u32 vec)
{
	int count = 0;

	if(vec == 0) {
		return 0;
	}

	while((vec & 0x1) == 0) {
		count = count + 1;
		vec = vec>>1;
	}

	return count;
}

/*
 *
 *
 *
 */
u32 num2vec(int num)
{
	u32 vec = 1;
	vec = vec << num;

	return vec;
}

/**
 * This function determines whether the axi peripheral can be read from/written to
 * directly or if it needs to use the DMAs. Once it decides it calls the apporpiate functions
 * to transfer data.
 * axi_address 64b AXI address to act on.
 * buf the system memory address to act on if direct R/W is selected.
 * count The amount of data to R/W
 * transfer_type determines R/W or R/W with keyhole.
 * dma_offset The DMA offset of memory region if DMA transfer is selected.
 */
int data_transfer(u64 axi_address, void *buf, size_t count, int transfer_type, u64 dma_off)
{
	//	u32 test;
	int in_range = 0;
	int status = 0;
	u64 dma_axi_address = 0;
	int i = 0;

	switch (transfer_type) {
		case KEYHOLE_WRITE:
			verbose_data_xfer_printk(KERN_INFO"[data_transfer]: keyhole write to base address 0x%llx %zd\n", axi_address, count);
			break;
		case KEYHOLE_READ:
			verbose_data_xfer_printk(KERN_INFO"[data_transfer]: keyhole read to base address 0x%llx %zd\n", axi_address, count);
			break;
		case NORMAL_WRITE:
			verbose_data_xfer_printk(KERN_INFO"[data_transfer]: writing to base address 0x%llx %zd\n", axi_address, count);
			break;
		case NORMAL_READ:
			verbose_data_xfer_printk(KERN_INFO"[data_transfer]: reading to base address 0x%llx %zd\n", axi_address, count);
			break;
		default:
			verbose_data_xfer_printk(KERN_INFO"[data_transfer]: Unknown transfer type\n");
	}

	/* Also does a final check to make sure you are writing in range */
	while (in_range == 0 && i < num_bars){
		if((axi_address + count) < pci_bar_end[i] && (axi_address >= pci_bar_addr[i])) {
			verbose_data_xfer_printk(KERN_INFO"[data_transfer]: address 0x%llx in range of bar %d going direct\n", axi_address, i);
			in_range = 1;
		}
		else{
			verbose_data_xfer_printk(KERN_INFO"[data_transfer]: address 0x%llx is not in range of bar %d, rage: 0x%lx to 0x%lx \n", axi_address, i, pci_bar_addr[i], pci_bar_end[i]);
		}
		i++;
	}
	if(!in_range){
		verbose_data_xfer_printk(KERN_INFO"[data_transfer]: address 0x%llx out of range using DMA %d\n", axi_address, cdma_capable);
	}

	//if data is small or the cdma is not initialized and in range
	if(in_range == 1) {
		if((transfer_type == NORMAL_READ) | (transfer_type == KEYHOLE_READ)) {
			if( direct_read(axi_address, buf, count, transfer_type) ) { //unsuccessful CDMA transmission
					printk(KERN_INFO"[data_transfer]: !!!!!!!!ERROR on direct_read!!!.\n");
				}
		} else if((transfer_type == NORMAL_WRITE) | (transfer_type == KEYHOLE_WRITE)) {
			if( direct_write(axi_address, buf, count, transfer_type) ) { //unsuccessful CDMA transmission
					printk(KERN_INFO"[data_transfer]: !!!!!!!!ERROR on direct_write!!!.\n");
				}
		} else {
			verbose_printk(KERN_INFO"[data_transfer]: !!!!!!!!ERROR no transfer type specified\n");
		}


	} else if(cdma_capable > 0 || pcie_use_xdma) {
		dma_axi_address = axi_pcie_m + dma_off; //the AXI address written to the CDMA

		if((transfer_type == NORMAL_READ) || (transfer_type == KEYHOLE_READ)) {
			u32 xfer_type = (HOST_WRITE|(transfer_type == KEYHOLE_READ ? INC_DA : INC_BOTH));
			status = dma_transfer(axi_address, dma_axi_address, (u32)count, transfer_type, xfer_type);
			if(status != 0) { //unsuccessful CDMA transmission
				printk(KERN_INFO"[data_transfer]: !!!!!!!!ERROR on CDMA READ!!!.\n");
			}
		} else if((transfer_type == NORMAL_WRITE) || (transfer_type == KEYHOLE_WRITE)) {
			u32 xfer_type = (HOST_READ|(transfer_type == KEYHOLE_WRITE ? INC_SA : INC_BOTH));
			//Transfer data from user space to kernal space at the allocated DMA region
			status = dma_transfer(dma_axi_address, axi_address, (u32)count, transfer_type, xfer_type);
			if(status != 0) { //unsuccessful CDMA transmission
				printk(KERN_INFO"[data_transfer]: !!!!!!!!ERROR on CDMA WRITE!!!.\n");
			}
		}
		else {
			verbose_printk(KERN_INFO"[data_transfer]: !!!!!!!!ERROR no transfer type specified\n");
		}
	} else {
		verbose_printk(KERN_INFO"[data_transfer]: !!!!!!!!ERROR: Address 0x%llx out of range and CDMA is not initialized\n", axi_address);
	}
	return status;
}


/*
 *
 *
 *
 */
int direct_write(u64 axi_address, void *buf, size_t count, int transfer_type)
{
	char * kern_buf;
	char * virt_addr;
	int offset;
	int len;
	u32 * newaddr_src;
	u32 * newaddr_dest;
	int i = 0;
	u32 write_value;
	int in_range = 0;

	kern_buf = buf;

	/*determine which BAR to write to*/
	/* Also does a final check to make sure you are writing in range */
	while (in_range == 0 && i < num_bars){
		if((axi_address + count) < pci_bar_end[i] && (axi_address >= pci_bar_addr[i])) {
			verbose_direct_write_printk(KERN_INFO"\t[direct_write]: Entering Direct write: address: 0x%llx count: ('0x%08x') \n", axi_address, (u32)count);
			verbose_direct_write_printk(KERN_INFO"\t[direct_write]: Direct write to BAR %d\n", i);
			virt_addr = (axi_address - pci_bar_addr[i]) + pci_bar_vir_addr[i];
			in_range = 1;
		}
		i++;
	}
	if(!in_range){
		verbose_direct_write_printk(KERN_INFO"\t[direct_write]: Entering Direct write: address: 0x%llx count: ('0x%08x') \n", axi_address, (u32)count);
		verbose_direct_write_printk(KERN_INFO"\t[direct_write]: !!!!!!!!ERROR trying to Direct write out of memory range!\n");
		return 1;
	}

	verbose_direct_write_printk(KERN_INFO"\t[direct_write]: writing:to 0x%llx value ('0x%08x') \n", axi_address, *(u32*)buf);

	offset = 0;
	len = count/4;
	for(i = 0; i<len; i++) {
		newaddr_src = (u32 *)(kern_buf + offset);
		write_value = *newaddr_src;
		newaddr_dest = (u32 *)(virt_addr + offset);
		iowrite32(write_value, newaddr_dest);
		verbose_direct_write_printk(KERN_INFO"\t[direct_write]: wrote: ('0x%08x') to virtual address:('0x%p')\n", write_value, newaddr_dest);
		if(transfer_type != KEYHOLE_WRITE)
			offset += 4;
	}

	return 0;

}


/*
 *
 *
 *
 */
int direct_read(u64 axi_address, void *buf, size_t count, int transfer_type)
{
	int len;
	char * virt_addr;
	int offset = 0;
	u32 kern_buf[count/4];
	u32 * newaddr_src;
	int i = 0;
	int in_range = 0;

	len = count/4; //how many 32b transferis

	/*determine which BAR to read from*/
	/* Also does a final check to make sure you are writing in range */
	while (in_range == 0 && i < num_bars){
		if((axi_address + count) < pci_bar_end[i] && (axi_address >= pci_bar_addr[i])) {
			verbose_direct_read_printk(KERN_INFO"\t[direct_read]: Entering Direct Read: address: 0x%llx count: ('0x%08x') \n", axi_address, (u32)count);
			verbose_direct_read_printk(KERN_INFO"\t[direct_read]: Direct reading from BAR %d\n", i);
			virt_addr = (axi_address - pci_bar_addr[i]) + pci_bar_vir_addr[i];
			in_range = 1;
		}
		i++;
	}
	if(!in_range){
		verbose_direct_read_printk(KERN_INFO"\t[direct_read]: Entering Direct Read: address: 0x%llx count: ('0x%08x') \n", axi_address, (u32)count);
		verbose_direct_read_printk(KERN_INFO"\t[direct_read]: !!!!!!!!ERROR trying to Direct read out of memory range!\n");
		return 1;
	}

	for(i = 0; i<len; i++) {
		newaddr_src = (u32 *)(virt_addr + offset);
		kern_buf[i] = ioread32(newaddr_src);
		//		kern_buf[i] = *newaddr_src; *this works too*
		if(transfer_type != KEYHOLE_READ)
			offset += 4;
		verbose_direct_read_printk(KERN_INFO"\t[direct_read]: read: ('0x%08x') from kernel address 0x%p.\n", kern_buf[i], newaddr_src);
	}

	memcpy(buf, (const void*)kern_buf, count);
	verbose_direct_read_printk(KERN_INFO"\t[direct_read]: Leaving Direct Read\n");

	return 0;
}

/**
 * This function will return the xdma channel that is available
 */
static int xdma_query(int direction)
{
	int i;
	if(direction == DMA_TO_DEVICE) {
		for (i = 0 ; i < xdma_num_channels; i++)
			if(!mutex_is_locked(&xdma_h2c_sem[i])) {
				if(!mutex_trylock(&xdma_h2c_sem[i]) ){
					verbose_dmaq_printk(KERN_INFO"\t\t\t[dma_queue]: Mutex trylock failed.\n");
					return -1;
				}
				verbose_dmaq_printk(KERN_INFO"\t\t\t[dma_queue]: xdma_channel = %d \n", i);
				return i;
			}
	} else if(direction == DMA_FROM_DEVICE) {
		for (i = 0 ; i < xdma_num_channels; i++)
			if(!mutex_is_locked(&xdma_c2h_sem[i])) {
				if(!mutex_trylock(&xdma_c2h_sem[i]) ) {
					verbose_dmaq_printk(KERN_INFO"\t\t\t[dma_queue]: Mutex trylock failed.\n");
					return -1;
				}
				verbose_dmaq_printk(KERN_INFO"\t\t\t[dma_queue]: xdma_channel = %d \n", i);
				return i;
			}
	}
	verbose_dmaq_printk(KERN_INFO"\t\t\t[dma_queue]: no xdma queue\n");
	return -1;
}

/**
 * @brief This function determines if a CDMA is available. if it is, it locks the semaphore and returns
 * the CDMA number.
*/
static int cdma_query(void)
{
	int i;
	/*Check the CDMA Semaphore*/
	for( i = 0; i < cdma_count; i++) {
		if( (cdma_set[i] == 1) && (!mutex_is_locked(&cdma_sem[i])) ) {
			if(!mutex_trylock(&cdma_sem[i]) ){
				verbose_dmaq_printk(KERN_INFO"\t\t\t[cdma_query]: Mutex trylock failed on %d.\n", i);
				return -1;
			}
			verbose_dmaq_printk(KERN_INFO"\t\t\t[cdma_query]: cdma_channel = %d \n", i);
			cdma_use_count[i]++;
			return i;
		}
	}
	verbose_cdmaq_printk(KERN_INFO "\t\t\t[cdma_query]: \t!!!!!!!!! all CDMAs in use !!!!!!!!\n");
	return -1;
}


/**
 * This function is used to initiate a CDMA Transfer. It requires
 * a locked CDMA resource an AXI Start Address, AXI Destination Address,
 * and a Keyhole setting.
 * SA 64bit Starting Address of DMA transfer.
 * DA 64bit Destination Address of DMA transfer.
 * BTT Number of bytes to transfer.
 * keyhole_en Instructs the CDMA to to a keyhole transaction or not
 * cdma_num Instructs which CDMA to use (Assumes it has been locked)
 *
 * These basic steps describe how to set up and initiate a CDMA transfer in simple operation
 * mode.
 * 1. Verify CDMASR.IDLE = 1.
 * 2. Program the CDMACR.IOC_IrqEn bit to the desired state for interrupt generation on
 * transfer completion. Also set the error interrupt enable (CDMACR.ERR_IrqEn), if so desired.
 * 3. Write the desired transfer source address to the Source Address (SA) register. The
 * transfer data at the source address must be valid and ready for transfer. If the address
 * space selected is more than 32, write the SA_MSB register also.
 * 4. Write the desired transfer destination address to the Destination Address (DA) register.
 * If the address space selected is more than 32, then write the DA_MSB register also.
 * 5. Write the number of bytes to transfer to the CDMA Bytes to Transfer (BTT) register. Up
 * to 8, 388, 607 bytes can be specified for a single transfer (unless DataMover Lite is being
 * used). Writing to the BTT register also starts the transfer.
 * 6. Either poll the CDMASR.IDLE bit for assertion (CDMASR.IDLE = 1) or wait for the CDMA
 * to generate an output interrupt (assumes CDMACR.IOC_IrqEn = 1).
 * 7. If interrupt based, determine the interrupt source (transfer completed or an error has occurred).
 * 8. Clear the CDMASR.IOC_Irq bit by writing a 1 to the DMASR.IOC_Irq bit position.
 * 9. Ready for another transfer. Go back to step 1.
 *
 */
int cdma_transfer(u64 SA, u64 DA, u32 BTT, int keyhole_en, int cdma_num)
{
	u32 bit_vec;
	u32 axi_dest;
	u32 SA_MSB;
	u32 SA_LSB;
	u32 DA_MSB;
	u32 DA_LSB;

	verbose_cdma_printk(KERN_INFO"\t\t[cdma_transfer]: **** Starting CDMA %d transfer ****\n", cdma_usage_cnt);

	SA_MSB = (u32)(SA>>32);
	SA_LSB = (u32)SA;

	DA_MSB = (u32)(DA>>32);
	DA_LSB = (u32)DA;

	//Step 1. Verify CDMASR.IDLE = 1.
	cdma_idle_poll(cdma_num);


	// Step 2. Program the CDMACR.IOC_IrqEn bit to the desired state for interrupt generation on
	// transfer completion. Also set the error interrupt enable (CDMACR.ERR_IrqEn), if so desired.

	switch(keyhole_en){

		case KEYHOLE_READ:
			bit_vec = 0x00000010;	//the bit for KEYHOLE READ
			verbose_cdma_printk(KERN_INFO"\t\t[cdma_transfer]: Setting the CDMA Keyhole READ as ENABLED\n");
			if( cdma_config_set(bit_vec, 1, cdma_num) ) {	//value of one means we want to SET the register
				printk(KERN_INFO"[cdma_transfer]: !!!!!!!!ERROR on CDMA WRITE!!!.\n");
			}
			break;

		case KEYHOLE_WRITE:
			bit_vec = 0x00000020;	//the bit for KEYHOLE WRITE
			verbose_cdma_printk(KERN_INFO"\t\t[cdma_transfer]: Setting the CDMA Keyhole WRITE as ENABLED\n");
			if( cdma_config_set(bit_vec, 1, cdma_num) ) {	//value of one means we want to SET the register
				printk(KERN_INFO"[data_transfer]: !!!!!!!!ERROR on CDMA WRITE!!!.\n");
			}	//value of one means we want to SET the register
			break;

		default:verbose_cdma_printk(KERN_INFO"\t\t[cdma_transfer]: no keyhole setting will be used.\n");
	}

	verbose_cdma_printk(KERN_INFO"\t\t[cdma_transfer]: Using CDMA %d\n", cdma_num);
	verbose_cdma_printk(KERN_INFO"\t\t[cdma_transfer]: ********* CDMA TRANSFER %d INITIALIZATION *************\n", cdma_usage_cnt);

	// Step 3. Write the desired transfer source address to the Source Address (SA) register. The
	// transfer data at the source address must be valid and ready for transfer. If the address
	// space selected is more than 32, write the SA_MSB register also.
	axi_dest = cdma_address[cdma_num] + CDMA_SA;
	verbose_cdma_printk(KERN_INFO"\t\t[cdma_transfer]: writing dma SA address ('0x%012llx') to CDMA at axi address:0x%x\n", SA, axi_dest);
	//extra debug messages
	verbose_cdma_printk(KERN_INFO"\t\t[cdma_transfer]: writing dma SA_LSB address ('0x%08x') to CDMA at axi address:0x%x\n", SA_LSB, axi_dest);
	verbose_cdma_printk(KERN_INFO"\t\t[cdma_transfer]: writing dma SA_MSB address ('0x%08x') to CDMA at axi address:0x%x\n", SA_MSB, axi_dest+4);

	direct_write(axi_dest , (void*)&SA_LSB, 4, NORMAL_WRITE);				//todo add error check
	direct_write(axi_dest + 4, (void*)&SA_MSB, 4, NORMAL_WRITE);			//todo add error check

	// Step 4. Write the desired transfer destination address to the Destination Address (DA) register.
	// If the address space selected is more than 32, then write the DA_MSB register also.
	axi_dest = cdma_address[cdma_num] + CDMA_DA;

	verbose_cdma_printk(KERN_INFO"\t\t[cdma_transfer]: writing DA address ('0x%012llx') to CDMA at axi address:0x%x\n", DA, axi_dest);
	//extra debug messages
	verbose_cdma_printk(KERN_INFO"\t\t[cdma_transfer]: writing DA_LSB address ('0x%08x') to CDMA at axi address:0x%x\n", DA_LSB, axi_dest);
	verbose_cdma_printk(KERN_INFO"\t\t[cdma_transfer]: writing DA_MSB address ('0x%08x') to CDMA at axi address:0x%x\n", DA_MSB, axi_dest+4);

	direct_write(axi_dest,	(void*)&DA_LSB, 4, NORMAL_WRITE);			//todo add error check
	direct_write(axi_dest+4, (void*)&DA_MSB, 4, NORMAL_WRITE);			//todo add error check
	//read the status register

	// Step 5. Write the number of bytes to transfer to the CDMA Bytes to Transfer (BTT) register. Up
	// to 8, 388, 607 bytes can be specified for a single transfer (unless DataMover Lite is being
	// used). Writing to the BTT register also starts the transfer.
	axi_dest = cdma_address[cdma_num] + CDMA_BTT;
	verbose_cdma_printk(KERN_INFO"\t\t[cdma_transfer]: writing bytes to transfer ('0x%x') to CDMA at axi address:0x%x\n", BTT, axi_dest);
	direct_write(axi_dest, (void*)&BTT, 4, NORMAL_WRITE);			//todo add error chceck


	// Step 6. Either poll the CDMASR.IDLE bit for assertion (CDMASR.IDLE = 1) or wait for the CDMA
	// to generate an output interrupt (assumes CDMACR.IOC_IrqEn = 1).
	//--------------------------------------------Poll ready next CDMA tansfer---------------------------------------------
	//cdma_idle_poll(cdma_num);

	// Step 7. If interrupt based, determine the interrupt source (transfer completed or an error has occurred).
	//--------------------------------------------NA---------------------------------------------

	// Step 8. Clear the CDMASR.IOC_Irq bit by writing a 1 to the DMASR.IOC_Irq bit position.
	//--------------------------------------------NA---------------------------------------------

	// Step 9. Ready for another transfer. Go back to step 1.

	// Acknowledge the CDMA and check for error status
	cdma_idle_poll(cdma_num);													//verify idle before checking status
	if( cdma_ack(cdma_num, SA, DA, BTT) ) {	//value of one means we want to SET the register
			printk(KERN_INFO"[data_transfer]: !!!!!!!!ERROR on CDMA WRITE!!!.\n");
	}

	if((keyhole_en == KEYHOLE_READ) | (keyhole_en == KEYHOLE_WRITE)) {
		bit_vec = 0x20 | 0x10;		// "0x30" unset both CDMA keyhole read and write
		if( cdma_config_set(bit_vec, 0, cdma_num) ) {	//unsets the keyhole configuration
			printk(KERN_INFO"[data_transfer]: !!!!!!!!ERROR on CDMA WRITE!!!.\n");
		}
	}

	verbose_cdma_printk(KERN_INFO"\t\t[cdma_transfer]: ********* CDMA TRANSFER %d FINISHED *************\n", cdma_usage_cnt);

	cdma_usage_cnt = cdma_usage_cnt + 1;

	return 0;
}

/**
 * This function continuously polls the CDMA until the completion bit is read.
 * cdma_num Instructs which CDMA to use (Assumes it has been locked)
 */
void cdma_idle_poll(int cdma_num)
{
	u32 status;
	u64 axi_dest;

	status = 0x0;
	axi_dest = cdma_address[cdma_num] + CDMA_SR;
	if( direct_read(axi_dest, (void *)&status, 4, NORMAL_READ) ) {
		printk(KERN_INFO"\t\t[cdma_idle_%d_poll]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n", cdma_num);
		return;
	}
	verbose_cdma_printk(KERN_INFO"\t\t[cdma_idle_%d_poll]: CDMA status: ('0x%08x')\n", cdma_num, status);


	while(((u32)status & 0x02) != 0x02) { //this means while CDMA is NOT idle
		schedule();
		//Check the status of the CDMA to see if successful now
				if( direct_read(axi_dest, (void *)&status, 4, NORMAL_READ) ) {
					printk(KERN_INFO"\t\t[cdma_idle_%d_poll]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n", cdma_num);
					return;
				}
	}
	if(status == 0xFFFFFFFF) {
		printk(KERN_INFO"\t\t[cdma_idle_%d_poll]: BAD CDMA status: ('0x%08x')\n", cdma_num, status);
		printk(KERN_INFO"\t\t[cdma_idle_%d_poll]: If using PCIe, add a buffer to axi interconnect in front of PCIe Slave input\n", cdma_num);
	}
	verbose_cdma_printk(KERN_INFO"\t\t[cdma_idle_%d_poll]: EXIT CDMA status: ('0x%08x')\n", cdma_num, status);
}


/**
 * This asserts or deasserts hte keyhole setting in the CDMA register.
 * cdma_num Instructs which CDMA to use (Assumes it has been locked)
 * bit_vec 32b bit vector to set the CDMA register
 * set (1) or unset (0) the bits.
 */
int cdma_config_set(u32 bit_vec, int set_unset, int cdma_num)
{
	u32 current_CR;
	u32 new_CR;
	u32 set_vec;
	u64 axi_dest;

	axi_dest = cdma_address[cdma_num] + CDMA_CR;
	if( direct_read(axi_dest, (void *)&current_CR, 4, NORMAL_READ) ) {
		printk(KERN_INFO"\t\t[cdma_idle_%d_poll]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n", cdma_num);
		return ERROR;
	}

	if(set_unset == 1) { //we are setting the CR
		verbose_cdma_printk(KERN_INFO"\t\t[cdma_0x%x_config_set]: setting the keyhole config on CDMA.\n", cdma_num);
		set_vec = (current_CR | bit_vec);
		if( direct_write(axi_dest, (void *)&set_vec, 4, NORMAL_WRITE) ) {
			printk(KERN_INFO"\t\t[cdma_idle_%d_poll]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n", cdma_num);
			return ERROR;
		}
	} else {	 //We are unsetting bits in the CR
		set_vec = (current_CR & (~bit_vec));
		verbose_cdma_printk(KERN_INFO"\t\t[cdma_0x%x_config_set]: UNsetting the keyhole config on CDMA.\n", cdma_num);
		if( direct_write(axi_dest, (void *)&set_vec, 4, NORMAL_WRITE) ) {
			printk(KERN_INFO"\t\t[cdma_idle_%d_poll]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n", cdma_num);
			return ERROR;
		}
	}

	if( direct_read(axi_dest, (void *)&new_CR, 4, NORMAL_READ) ) {
		printk(KERN_INFO"\t\t[cdma_idle_%d_poll]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n", cdma_num);
		return ERROR;
	}
	verbose_cdma_printk(KERN_INFO"\t\t[cdma_0x%x_config_set]: new CDMA Config Register is: ('0x%08x')\n", cdma_num, new_CR);
	return 0;
}


/**
 * This function is used to acknowledge a CDMA transaction. It will check
 * for any failures.
 * cdma_num Instructs which CDMA to use (Assumes it has been locked)
 */
int cdma_ack(int cdma_num, u64 sa, u64 da, u32 btt)
{
	u32 cdma_status;
	u32 status;
	u64 axi_dest;

	/*acknowledge the CDMA interrupt (to reset it)*/
	cdma_status = 0x00001000;
	axi_dest = cdma_address[cdma_num] + CDMA_SR;
	if( direct_write(axi_dest, (void *)&cdma_status, 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"\t\t[cdma_0x%x_ack]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n", cdma_num);
		return ERROR;
	}
	verbose_cdma_printk(KERN_INFO"\t\t[cdma_0x%x_ack]: writing SR config ('0x%08x') to CDMA at AXI address:%llx\n", cdma_num, cdma_status, axi_dest);

	status = 0x0;

	/* Check the status of the CDMA to see if successful */
	if( direct_read(axi_dest, (void *)&status, 4, NORMAL_READ) ) {
		printk(KERN_INFO"\t\t[cdma_0x%x_ack]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n", cdma_num);
		return ERROR;
	}
	verbose_cdma_printk(KERN_INFO"\t\t[cdma_0x%x_ack]: CDMA status: ('0x%08x')\n", cdma_num, status);
	//if this is not the expected status report
	if((u32)status != 0x2) {
		printk(KERN_INFO"\t\t[cdma_0x%x_ack]: !!!!!!!!ERROR CDMA status: ('0x%08x') SA:0x%llx DA:0x%llx btt:%d\n", cdma_num, status, sa,da,btt);
		if(status == 0x4042) {
			printk(KERN_INFO"\t\t[cdma_0x%x_ack]: This is an AXI slave response error SA:0x%llx DA:0x%llx Bytes:%d\n", cdma_num, sa, da, btt);
			printk(KERN_INFO"\t\t\t\t[cdma_0x%x_ack]: commonly due to incorrect setting in pcie ip for axi to bar translation high address\n", cdma_num);
			printk(KERN_INFO"\t\t\t\t[cdma_0x%x_ack]: Must set this value to be the DMA buffer size allocation.\n", cdma_num);
		}
		verbose_printk(KERN_INFO"\t\t[cdma_0x%x_ack]: Issuing soft reset to CDMA.\n", cdma_num);

		/*Issue a Soft Reset*/
		axi_dest = cdma_address[cdma_num] + CDMA_CR;
		cdma_status = 0x00000004;
		verbose_cdma_printk(KERN_INFO"\t\t[cdma_0x%x_ack]: sending a soft reset to the CDMA\n", cdma_num);
		if( direct_write(axi_dest, (void *)&cdma_status, 4, NORMAL_WRITE) ) {
			printk(KERN_INFO"\t\t[cdma_0x%x_ack]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n", cdma_num);
			return ERROR;
		}
		/*Check the current status*/
		axi_dest = cdma_address[cdma_num] + CDMA_SR;
		if( direct_read(axi_dest, (void *)&cdma_status, 4, NORMAL_READ) ) {
			printk(KERN_INFO"\t\t[cdma_0x%x_ack]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n", cdma_num);
			return ERROR;
		}
		verbose_cdma_printk(KERN_INFO"\t\t[cdma_0x%x_ack]: CDMA status before configuring: ('0x%08x')\n", cdma_num, cdma_status);
		/*Check the current config*/
		axi_dest = cdma_address[cdma_num] + CDMA_CR;
		if( direct_read(axi_dest, (void *)&cdma_status, 4, NORMAL_READ) ) {
			printk(KERN_INFO"\t\t[cdma_0x%x_ack]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n", cdma_num);
			return ERROR;
		}
		verbose_cdma_printk(KERN_INFO"\t\t[cdma_0x%x_ack]: CDMA config before configuring: ('0x%08x')\n", cdma_num, cdma_status);
		/*clear any pre existing interrupt*/
		axi_dest = cdma_address[cdma_num] + CDMA_SR;
		cdma_status = 0x00001000;
		verbose_cdma_printk(KERN_INFO"\t\t[cdma_0x%x_ack]: attempting to write: ('0x%08x') to cdma status reg\n", cdma_num, cdma_status);
		if( direct_write(axi_dest, (void *)&cdma_status, 4, NORMAL_WRITE) ) {
			printk(KERN_INFO"\t\t[cdma_0x%x_ack]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n", cdma_num);
			return ERROR;
		}

		/*Check the current status*/
		axi_dest = cdma_address[cdma_num] + CDMA_SR;
		if( direct_read(axi_dest, (void *)&cdma_status, 4, NORMAL_READ) ) {
			printk(KERN_INFO"\t\t[cdma_0x%x_ack]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n", cdma_num);
			return ERROR;
		}
		verbose_cdma_printk(KERN_INFO"\t\t[cdma_0x%x_ack]: CDMA status after configuring: ('0x%08x')\n", cdma_num, cdma_status);
		/*Check the current config*/
		axi_dest = cdma_address[cdma_num] + CDMA_CR;
		if( direct_read(axi_dest, (void *)&cdma_status, 4, NORMAL_READ) ) {
			printk(KERN_INFO"\t\t[cdma_0x%x_ack]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n", cdma_num);
			return ERROR;
		}
		verbose_cdma_printk(KERN_INFO"\t\t[cdma_0x%x_ack]: CDMA config after configuring: ('0x%08x')\n", cdma_num, cdma_status);

		return status;
	}
	else {
		return 0; // successful CDMA transmission
	}
}


/**
 * This function checks if there is any available data to be read from an axi stream fifo.
 * mod_desc The struct containing all the file variables.
 *		returns 0 if the occupancy is zero, otherwise returns read fifo length
 */
size_t axi_stream_fifo_d2r(struct mod_desc * mod_desc)
{
	u32 buf;
	u64 axi_dest;

	if(mod_desc->mode == AXI_STREAM_PACKET) {

		/*Read FIFO Fill level RDFO */
		axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_RDFO;
		if( direct_read(axi_dest, (void*)(&buf), 4, NORMAL_READ) ) {
			printk(KERN_INFO"[axi_stream_fifo_d2r]: !!!!!!!!ERROR reading Read FIFO fill level (RDFO)\n");
			return ERROR;
		}
		verbose_axi_fifo_d2r_printk(KERN_INFO"[axi_stream_fifo_d2r]: RDFO value = 0x%08x\n", buf);
		mod_desc->axi_fifo_rdfo = buf;

		//verify that the occupancy is not zero	MM
		if(buf == 0){
			verbose_axi_fifo_d2r_printk(KERN_INFO"[axi_stream_fifo_d2r]: Receive occupancy is zero, nothing to read\n");
			return 0;
		}



		/*Read FIFO Fill level RDR (tdest)*/
		axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_RDR;

		if( direct_read(axi_dest, (void*)(&buf), 4, NORMAL_READ) ) {
			printk(KERN_INFO"[axi_stream_fifo_d2r]: !!!!!!!!ERROR reading Read FIFO dest (RDR)\n");
			return ERROR;
		}
		verbose_axi_fifo_d2r_printk(KERN_INFO"[axi_stream_fifo_d2r]: RDR value = 0x%08x\n", buf);
		mod_desc->rx_dest = buf;

		/*Read FIFO Fill level RLR (byte count)*/
		axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_RLR;

		if( direct_read(axi_dest, (void*)(&buf), 4, NORMAL_READ) ) {
			printk(KERN_INFO"[axi_stream_fifo_d2r]: !!!!!!!!ERROR reading Read FIFO fill level (RLR)\n");
			return ERROR;
		}
		verbose_axi_fifo_d2r_printk(KERN_INFO"[axi_stream_fifo_d2r]: RLR value = 0x%08x\n", buf);
		mod_desc->axi_fifo_rlr = buf; //not sure why you would and here...... & 0x7fffffff; // remember the read

		return buf;

	}
	else if(mod_desc->mode == AXI_STREAM_FIFO){

		/*Read FIFO Fill level RLR (byte count)*/
		axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_RLR;

		if( direct_read(axi_dest, (void*)(&buf), 4, NORMAL_READ) ) {
			printk(KERN_INFO"[axi_stream_fifo_d2r]: !!!!!!!!ERROR reading Read FIFO fill level (RLR)\n");
			return ERROR;
		}
		verbose_axi_fifo_d2r_printk(KERN_INFO"[axi_stream_fifo_d2r]: RLR value = 0x%08x\n", buf);

		buf = buf & 0x7FFFFFFF;								//mask out the highest bit (that indicates partial packet)

		mod_desc->axi_fifo_rlr = buf; //not sure why you would and here...... & 0x7fffffff; // remember the read

		return buf;

	}
	else {
		printk(KERN_INFO"[axi_stream_fifo_d2r]: Unknown mode!\n");
		return ERROR;
	}

}

/**
 * This function performs calls appropriate functions for Reading from the AXI Streaming FIFO.
 * count: The number of bytes to read from the AXI streaming FIFO.
 * mod_desc: The struct containing all the file variables.
 * ring_pointer_offset: The current offset of the ring pointer in memory to store data.
 * returns the new ring_pointer_offset
 */
size_t axi_stream_fifo_read(size_t count, char * buf_base_addr, u64 hw_base_addr, struct mod_desc * mod_desc, int ring_pointer_offset, size_t buf_size) {
	size_t room_till_end;
	size_t remaining;
	u64 axi_dest;
	size_t zero = 0;

	//clear values in mod_desc
	mod_desc->axi_fifo_rlr = mod_desc->axi_fifo_rdfo = 0;

	if(count == 0) {
		verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read]: There is either no data to read, or less than 8 bytes.\n");
		return 0; // nothing to read
	} else {
		verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read]: count = %zd.\n", count);
	}

	//copy the count to the ring buffer to act as the "header"
	room_till_end = buf_size - ring_pointer_offset;
	if(sizeof(count) > room_till_end) {		//If the header will not fit in 1 chuck of the ring buffer (in the room till end) then just put it at the start
		ring_pointer_offset = 0;
	}

	verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read]: header copy_from_user(0x%p + 0x%x, 0x%p, 0x%zx)\n", buf_base_addr, ring_pointer_offset, (void * )&count, sizeof(count));

	memcpy(buf_base_addr+ring_pointer_offset, (void * )&count, sizeof(count));
	ring_pointer_offset = get_new_ring_pointer(sizeof(count), ring_pointer_offset, (int)buf_size);

	//}
	room_till_end = ( (buf_size - ring_pointer_offset) & ~(dma_byte_width-1));				 //make it divisible by dma_byte_width
	axi_dest = mod_desc->axi_addr + AXI_STREAM_RDFD;

	if(count > room_till_end) {			//needs to be 2 cdma transfers
		remaining = count-room_till_end;

		verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read]: dma_transfer 1 AXI Address: 0x%llx, ring_buff address: 0x%llx + 0x%x, len: 0x%zx\n", axi_dest, hw_base_addr, ring_pointer_offset, room_till_end);
		if(dma_transfer(axi_dest, hw_base_addr+(u64)ring_pointer_offset, room_till_end, KEYHOLE_READ, (HOST_WRITE|INC_DA))) { //unsuccessful CDMA transmission
			printk(KERN_INFO"[axi_stream_fifo_read]: !!!!!!!!ERROR on CDMA READ!!!.\n");
		}


		//extra verbose debug message
		verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read]: first 4 bytes 0x%08x\n", *((u32*)(mod_desc->dma_read_addr+ring_pointer_offset)));

		ring_pointer_offset = 0;
		//end of buffer reached
		verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read]: dma_transfer 2 AXI Address: 0x%llx, ring_buff address: 0x%llx + 0x%x, len: 0x%zx\n", axi_dest, hw_base_addr, ring_pointer_offset, remaining);
		if(dma_transfer(axi_dest, hw_base_addr+(u64)ring_pointer_offset, remaining, KEYHOLE_READ, (HOST_WRITE|INC_DA)) ) { //unsuccessful CDMA transmission
			printk(KERN_INFO"[axi_stream_fifo_read]: !!!!!!!!ERROR on CDMA READ!!!.\n");
		}
		ring_pointer_offset = get_new_ring_pointer(remaining, ring_pointer_offset, (int)buf_size);

	} else {														//only 1 cdma transfer
		verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read]: dma_transfer AXI Address: 0x%llx, ring_buff address: 0x%llx + 0x%x, len: 0x%zx\n", axi_dest, hw_base_addr, ring_pointer_offset , count);
		if(dma_transfer(axi_dest, ((u64)hw_base_addr+ring_pointer_offset), count, KEYHOLE_READ, (HOST_WRITE|INC_DA)) ) { //unsuccessful CDMA transmission
			printk(KERN_INFO"[axi_stream_fifo_read]: !!!!!!!!ERROR on CDMA READ!!!.\n");
		}


		//extra verbose debug message
		verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read]: first 4 bytes 0x%08x\n", *((u32*)(mod_desc->dma_read_addr+ring_pointer_offset)));


		ring_pointer_offset = get_new_ring_pointer(count, ring_pointer_offset, (int)buf_size);
	}

	//copy the 0 to the ring buffer to act as a "0 header" to mark inbetween packets
	room_till_end = buf_size - ring_pointer_offset;
	if(sizeof(zero) > room_till_end) {		//If the trailer will not fit in 1 chuck of the ring buffer (in the room till end) then just put it at the start
		ring_pointer_offset = 0;
	}

	verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read]: trailer copy_from_user(0x%p + 0x%x, 0x%p, 0x%zx)\n", buf_base_addr, ring_pointer_offset, (void * )&zero, sizeof(zero));
	memcpy(buf_base_addr+ring_pointer_offset, (void * )&zero, sizeof(zero));
	ring_pointer_offset = get_new_ring_pointer(sizeof(zero), ring_pointer_offset, (int)buf_size);

	/*update rfh pointer*/
	atomic_set(mod_desc->rfh, ring_pointer_offset);
	//We increased the rfh (head) if it is now equal to the rfu (tail) then the buffer is full
	if(atomic_read(mod_desc->rfu) == ring_pointer_offset) {
		atomic_set(mod_desc->read_ring_buf_full, 1);
		verbose_printk(KERN_INFO"[axi_stream_fifo_read]: ring_point: Read Full: %d\n", atomic_read(mod_desc->read_ring_buf_full));
	}
	verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read]: read ring_buffer: RFU : %d RFH %d\n", atomic_read(mod_desc->rfu), ring_pointer_offset);


	verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read]: Leaving the READ AXI Stream FIFO routine %zd\n", count);
	return ring_pointer_offset;
}




/**
 * This function performs calls appropriate functions for Reading from the AXI Streaming FIFO.
 * count: The number of bytes to read from the AXI streaming FIFO.
 * mod_desc: The struct containing all the file variables.
 * ring_pointer_offset: The current offset of the ring pointer in memory to store data.
 * returns the number of bytes read
 */
size_t axi_stream_fifo_read_direct(size_t count, char * buf_base_addr, u64 hw_base_addr, struct mod_desc * mod_desc, size_t buf_size) {
	u64 axi_dest;

	//clear values in mod_desc
	mod_desc->axi_fifo_rlr = mod_desc->axi_fifo_rdfo = 0;

	if(count == 0) {
		verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read_direct]: There is either no data to read, or less than 8 bytes.\n");
		return 0; // nothing to read
	} else {
		verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read_direct]: count = %zd.\n", count);
	}

	axi_dest = mod_desc->axi_addr + AXI_STREAM_RDFD;

	if(count > buf_size) {			//needs to be 2 cdma transfers
		//there is not enough room in the buffer
		printk(KERN_INFO"[axi_stream_fifo_read_direct]: not enough room in the buffer!!!.\n");
		return 0;

	}
	//only 1 cdma transfer
	verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read_direct]: dma_transfer AXI Address: 0x%llx, ring_buff address: 0x%llx, len: 0x%zx\n", axi_dest, hw_base_addr , count);

	if(dma_transfer(axi_dest, ((u64)hw_base_addr), count, KEYHOLE_READ, (HOST_WRITE|INC_DA))) { //unsuccessful CDMA transmission
		printk(KERN_INFO"[axi_stream_fifo_read_direct]: !!!!!!!!ERROR on CDMA READ!!!.\n");
	}

	verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read_direct]: Leaving the READ AXI Stream FIFO routine %zd\n", count);
	return count;
}


/**
 * This function performs calls appropriate functions for writing to the AXI Streaming FIFO.
 * count The number of bytes to write to the AXI streaming FIFO.
 * mod_desc The struct containing all the file variables.
 * ring_pointer_offset The current offset of the ring pointer in memory for data to write.
 * return ERROR for error, 0 otherwise
 */
int axi_stream_fifo_init(struct mod_desc * mod_desc)
{
	u64 axi_dest;
	u32 buf, read_reg;
	u32 fifo_empty_level;

	//reset The transmit side
	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_TDFR;
	buf = 0x000000A5;
	if( direct_write(axi_dest, (void*)(&buf), 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[axi_stream_fifo_%x_init]: !!!!!!!!ERROR resetting TX fifo\n", mod_desc->minor);
		return ERROR;
	}
	verbose_printk(KERN_INFO"[axi_stream_fifo_%x_init]: Reset the TX axi fifo\n", mod_desc->minor);

	//reset The receive side
	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_RDFR;
	buf = 0x000000A5;
	if( direct_write(axi_dest, (void*)(&buf), 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[axi_stream_fifo_%x_init]: !!!!!!!!ERROR resetting RX fifo\n", mod_desc->minor);
		return ERROR;
	}
	verbose_printk(KERN_INFO"[axi_stream_fifo_%x_init]: Reset the RX axi fifo\n", mod_desc->minor);

	//Read CTL ISR interface
	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_ISR;
	buf = 0;
	if( direct_read(axi_dest, (void*)(&buf), 4, NORMAL_READ) ) {
		printk(KERN_INFO"[axi_stream_fifo_%x_init]: !!!!!!!!ERROR reading AXI_STREAM_ISR register\n", mod_desc->minor);
		return ERROR;
	}
	verbose_printk(KERN_INFO"[axi_stream_fifo_%x_init]: AXI_STREAM_ISR register: ('0x%08x')\n", mod_desc->minor, buf);

	printk(KERN_INFO"[axi_stream_fifo_%x_init]: Checkpoint 2\n", mod_desc->minor);

	//reset interrupts on CTL interface
	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_ISR;
	buf = 0xFFFFFFFF;
	if( direct_write(axi_dest, (void*)(&buf), 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[axi_stream_fifo_%x_init]: !!!!!!!!ERROR resetting AXI_STREAM_ISR register.\n", mod_desc->minor);
		return ERROR;
	}
	verbose_printk(KERN_INFO"[axi_stream_fifo_%x_init]: Reset the interrupts on the axi fifo\n", mod_desc->minor);

	//Read CTL interface
	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_ISR;
	if( direct_read(axi_dest, (void*)(&buf), 4, NORMAL_READ) ) {
		printk(KERN_INFO"[axi_stream_fifo_%x_init]: !!!!!!!!ERROR reading AXI_STREAM_ISR register.\n", mod_desc->minor);
		return ERROR;
	}
	verbose_printk(KERN_INFO"[axi_stream_fifo_%x_init]: AXI_STREAM_ISR register: ('0x%08x')\n", mod_desc->minor, buf);
	//-----------------------------------------------------------------mm-----------------------------------------

	//enable interupt from axi_stream_fifo if we are using the axi_stream_packet
	if(mod_desc->mode == AXI_STREAM_PACKET) {
		verbose_printk(KERN_INFO"[axi_stream_fifo_%x_init]: Setting AXI_STREAM_IER register for AXI_STREAM_PACKET.\n", mod_desc->minor);
		//Set IER Register for interrupt on read -MM
		axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_IER;
		buf = 0x04000000;							//bit 26 RX Complete
		if( direct_write(axi_dest, (void*)(&buf), 4, NORMAL_WRITE) ) {
			printk(KERN_INFO"[axi_stream_fifo_%x_init]: !!!!!!!!ERROR setting AXI_STREAM_IER register.\n", mod_desc->minor);
			return ERROR;
		}
	}

	buf = 0x0;

	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_TDFV; // the transmit FIFO vacancy
	if( direct_read(axi_dest, (void*)(&buf), 4, NORMAL_READ) ) {
		printk(KERN_INFO"[axi_stream_fifo_%x_init]: !!!!!!!!ERROR reading tx side vacancy\n", mod_desc->minor);
		return ERROR;
	}

	//this is a hack until I fix the data_transfer function to only use 1 buffer. regardless of cdma use
	read_reg = buf;

	/*Check to see if the calculated fifo empty level via the DMA data byte width (aka the axi-s fifo byte width)
	 * and file size (aka the fifo size) is equal to the actual fifo empty level when read */
	fifo_empty_level = (((u32)(mod_desc->file_size))/8)-4; //(Byte size / 8 bytes per word) - 4)	this value is the empty fill level of the tx fifo.
	printk(KERN_INFO"[axi_stream_fifo_%x_init]: file size: ('0x%08x')\n", mod_desc->minor, (u32)mod_desc->file_size);
	printk(KERN_INFO"[axi_stream_fifo_%x_init]: calculated size: ('0x%08x')\n", mod_desc->minor, fifo_empty_level);
	printk(KERN_INFO"[axi_stream_fifo_%x_init]: register read size: ('0x%08x')\n", mod_desc->minor, read_reg);
	if(read_reg > 0) {
		fifo_empty_level = (((u32)(mod_desc->file_size))/8)-4; //(Byte size / 8 bytes per word) - 4)	this value is the empty fill level of the tx fifo.
		if(read_reg != fifo_empty_level) {
			printk(KERN_INFO"[axi_stream_fifo_%x_init]: !!!!!!!!ERROR during initialization, the calculated axi-s fifo size is not equivalent to the actual size, Fix parameters\n", mod_desc->minor);
			printk(KERN_INFO"[axi_stream_fifo_%x_init]: calculated size: ('0x%08x')\n", mod_desc->minor, fifo_empty_level);
			printk(KERN_INFO"[axi_stream_fifo_%x_init]: register read size: ('0x%08x')\n", mod_desc->minor, read_reg);
			//return ERROR;
		}
		printk(KERN_INFO"[axi_stream_fifo_%x_init]: Transmit axi-s fifo initialized\n", mod_desc->minor);
	} else
		printk(KERN_INFO"[axi_stream_fifo_%x_init]: Receive axi-s fifo initialized\n", mod_desc->minor);

	return 0;
}

/**
 * This function performs calls appropriate functions for writing to the AXI Streaming FIFO.
 * count The number of bytes to write to the AXI streaming FIFO.
 * mod_desc The struct containing all the file variables.
 * ring_pointer_offset The current offset of the ring pointer in memory for data to write.
 * return ERROR for error, 0 otherwise
 */
int axi_stream_fifo_deinit(struct mod_desc * mod_desc)
{
	u64 axi_dest;
	u32 buf;

	//reset The transmit side
	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_TDFR;
	buf = 0x000000A5;
	if( direct_write(axi_dest, (void*)(&buf), 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[axi_stream_fifo_%x_deinit]: !!!!!!!!ERROR resetting TX fifo\n", mod_desc->minor);
		return ERROR;
	}
	verbose_printk(KERN_INFO"[axi_stream_fifo_%x_deinit]: Reset the the axi fifo\n", mod_desc->minor);

	//reset The receive side
	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_RDFR;
	buf = 0x000000A5;
	if( direct_write(axi_dest, (void*)(&buf), 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[axi_stream_fifo_%x_deinit]: !!!!!!!!ERROR resetting RX fifo\n", mod_desc->minor);
		return ERROR;
	}
	verbose_printk(KERN_INFO"[axi_stream_fifo_%x_deinit]: Reset the the axi fifo\n", mod_desc->minor);

	//Set IER Register for no interupts on read -MM
	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_IER;
	buf = 0x00000000;
	if( direct_write(axi_dest, (void*)(&buf), 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[axi_stream_fifo_%x_deinit]: !!!!!!!!ERROR setting AXI_STREAM_IER register.\n", mod_desc->minor);
		return ERROR;
	}

	printk(KERN_INFO"[axi_stream_fifo_%x_deinit]: Fifo interupt disabled\n", mod_desc->minor);

	return 0;
}
