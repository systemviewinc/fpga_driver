/**
 * System View Device Driver

 * @date		11/10/2015

 * @author	 System View Inc.

 * @file		 sv_streaming_funcs.c

 * @brief			This file contains all the non char driver specific code. It contains all
 *				 the steaming (stream fifo) data movement code and xilinx specific IP controllers
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

 #if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 12, 0)
 #include <linux/sched/task.h>
 #endif

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
 #include "sv_driver.h"
 #include "xdma/sv_xdma.h"

 //debug
 #include <linux/time.h>


 /**
  * This function operates on it's own thread after insmod. It is used to handle all AXI-Streaming
  * read transfers. It reads from the hardware and stores data in the Read Ring Buffer. It wakes up the
  * poll method for the file upon receiving new data.
  * read_fifo the kernel FIFO data structure for the global read FIFO.
  */
 int read_thread(void *in_param) {
 	struct sv_mod_dev* svd = (struct sv_mod_dev*)in_param;
 	struct kfifo* read_fifo = (struct kfifo*) &svd->read_fifo;

 	struct file_desc * file_desc;
 	int read_incomplete;
 	int d2r;
 	unsigned int sz;
 	void * buffer;

 	verbose_read_thread_printk(KERN_INFO"[read_thread]: started !!\n");
 	while(!kthread_should_stop()) {
 		verbose_read_thread_printk(KERN_INFO"[read_thread]: waiting in wait_event_interruptible for data!!\n");
 		//waits until thread_q_read is true or we should stop the thread (previous methods of exitings weren't working -MM)
 		if( wait_event_interruptible(svd->thread_q_head_read, ( atomic_read(&svd->thread_q_read) == 1 || kthread_should_stop() )) ){

 				}
 		atomic_set(&svd->thread_q_read, 0); // the threaded way
 		verbose_read_thread_printk(KERN_INFO"[read_thread]: woke up the read thread have data!!\n");
 		// process reads as long as there is something in the read fifo
 		while (!kfifo_is_empty(read_fifo) && ! kthread_should_stop()) {
 			//while the kfifo has data
 			verbose_read_thread_printk(KERN_INFO"[read_thread]: read_fifo size:%d\n", kfifo_size(read_fifo));
 			//read a file_desc out of the fifo
 			sz = kfifo_out(read_fifo, &file_desc, 1);

 			if(sz == 1 && file_desc->file_activate) {
 				verbose_read_thread_printk(KERN_INFO"[read_thread]: kfifo_out returned non-zero\n");
 				atomic_dec(file_desc->in_read_fifo_count);

 				//Check if there is any data to be read
 				if(file_desc->axi_fifo_rdfo > 0) {
 					d2r = file_desc->axi_fifo_rlr;
 				} else {
 					//else read it from the register
 					d2r = axi_stream_fifo_d2r(file_desc);
					 //if we have read all the data from the fifo (only true if d2r is zero from d2r function)
 				}

 				verbose_read_thread_printk(KERN_INFO"[read_thread]: file_desc minor: %d d2r: %d\n", file_desc->minor, d2r);
 				while(d2r != 0 && !kthread_should_stop()) {

 					if(pcie_use_xdma == 1){
 						buffer = file_desc->read_buffer;
 					}
 					else{
 						buffer = file_desc->dma_read_addr;
 					}

 					read_incomplete = read_data(file_desc, d2r, buffer);
 					//if read_data needs back_pressure
 					if(read_incomplete == 1 || read_incomplete == ERROR) {
 						//write the file_desc back in if the fifo is not full and it isn't already in the fifo
						 verbose_read_thread_printk(KERN_INFO"[read_thread]: read incomplete: %d\n", read_incomplete);

 						if(!kfifo_is_full(read_fifo) && file_desc->file_activate) {
 							atomic_inc(file_desc->in_read_fifo_count);
 							kfifo_in_spinlocked(read_fifo, &file_desc, 1, &file_desc->svd->fifo_lock_read);
 						} else {
 							printk(KERN_INFO"[read_thread]: kfifo is full, not writing mod desc\n");
 						}

						wait_event_interruptible_timeout(svd->thread_q_head_read, atomic_read(&svd->thread_q_read) == 1 || kthread_should_stop(), msecs_to_jiffies(10000) );
						atomic_set(&svd->thread_q_read, 0); // the threaded way
 						d2r = 0;
 					} else if(file_desc->file_activate && is_packet_full_interrupt(file_desc)){
						//clear ISR if it was from packet full
						clear_fifo_isr(file_desc);
						d2r = axi_stream_fifo_d2r(file_desc);
 						verbose_read_thread_printk(KERN_INFO"[read_thread]: file is closed: %d d2r: %d\n", file_desc->minor, d2r);
					}
					else if(file_desc->file_activate){			//read more data if the file is open
 						d2r = axi_stream_fifo_d2r(file_desc);
						//if we have read all the data from the fifo (only true if d2r is zero from d2r function)
						if(d2r == 0){
						}
 						verbose_read_thread_printk(KERN_INFO"[read_thread]: read more! file_desc minor: %d d2r: %d\n", file_desc->minor, d2r);
 					} else {
 						d2r = 0;
 						verbose_read_thread_printk(KERN_INFO"[read_thread]: file is closed: %d d2r: %d\n", file_desc->minor, d2r);
 					}
 				}
				clear_fifo_isr(file_desc);
 				verbose_read_thread_printk(KERN_INFO"[read_thread]: No more data to read.....\n");
 			} else if(sz == 1 && !file_desc->file_activate) {
 				verbose_read_thread_printk(KERN_INFO"[read_thread]: file is not open\n");
 				atomic_dec(file_desc->in_read_fifo_count);
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
 	struct sv_mod_dev* svd = (struct sv_mod_dev*)in_param;
 	struct kfifo* write_fifo = (struct kfifo*) &svd->write_fifo;

 	struct file_desc * file_desc;
 	int write_incomplete;
 	int d2w;
 	int wth, wtk, full;
 	unsigned int sz;
 	void * buffer;

 	verbose_write_thread_printk(KERN_INFO"[write_thread]: started !!\n");
 	while(!kthread_should_stop()){
 		verbose_write_thread_printk(KERN_INFO"[write_thread]: waiting in wait_event_interruptible, write buffer empty!!\n");

 		wait_event_interruptible(svd->thread_q_head_write, ( atomic_read(&svd->thread_q_write) == 1 || kthread_should_stop() ));
 		atomic_set(&svd->thread_q_write, 0); // the threaded way
 		verbose_write_thread_printk(KERN_INFO"[write_thread]: woke up the write thread!!\n");

 		// process write as long as there is something in the write fifo
 		while (!kfifo_is_empty(write_fifo) && ! kthread_should_stop()) {
 			verbose_write_thread_printk(KERN_INFO"[write_thread]: write_fifo size:%d\n", kfifo_size(write_fifo));
 			//read a file_desc out of the fifo
 			sz = kfifo_out(write_fifo, &file_desc, 1);

 			if(sz == 1 && file_desc->file_activate) {
 				atomic_dec(file_desc->in_write_fifo_count);
 				verbose_write_thread_printk(KERN_INFO"[write_thread]: kfifo_out returned non-zero\n");

 				//Check if there is any data to be written
 				wth = atomic_read(file_desc->wth);
 				wtk = atomic_read(file_desc->wtk);
 				full = atomic_read(file_desc->write_ring_buf_full);

 				//d2w = data_to_transfer(file_desc, wth, wtk, priority);
 				d2w = data_in_buffer(wtk, wth, full, file_desc->dma_size);

 				verbose_write_thread_printk(KERN_INFO"[write_thread]: file_desc minor: %d d2w: %d\n", file_desc->minor, d2w);
 				while(d2w != 0 && !kthread_should_stop()) {

 					if(pcie_use_xdma == 1) {
 						buffer = file_desc->write_buffer;
 					}
 					else {
 						buffer = file_desc->dma_write_addr;
 					}

 					write_incomplete = write_data(file_desc, buffer);
 					//if read_data needs back_pressure
 					if((write_incomplete == 1 || write_incomplete == ERROR)) {
						 verbose_write_thread_printk(KERN_INFO"[write_thread]: write incomplete: %d\n", write_incomplete);

 						//write the file_desc back in if the fifo is not full and it isn't already in the fifo
 						if(!kfifo_is_full(write_fifo) && file_desc->file_activate) {
 							atomic_inc(file_desc->in_write_fifo_count);
 							kfifo_in_spinlocked(write_fifo, &file_desc, 1, &svd->fifo_lock_write);
 						}
 						else {
 							printk(KERN_INFO"[write_thread]: kfifo is full, not writing mod desc\n");
 						}
						wait_event_interruptible_timeout(svd->thread_q_head_write, atomic_read(&svd->thread_q_write) == 1 || kthread_should_stop(), msecs_to_jiffies(1000) );
						atomic_set(&svd->thread_q_write, 0); // the threaded way
						d2w = 0;
 					} else if(file_desc->file_activate) {
 						//if write was complete and file is open write more data!
 						//Check if there is any data to be written
 						wth = atomic_read(file_desc->wth);
 						wtk = atomic_read(file_desc->wtk);
 						full = atomic_read(file_desc->write_ring_buf_full);

 						d2w = data_in_buffer(wtk, wth, full, file_desc->dma_size);
 						verbose_write_thread_printk(KERN_INFO"[write_thread]: write more! file_desc minor: %d d2w: %d\n", file_desc->minor, d2w);
 					} else {
 						d2w = 0;
 						verbose_write_thread_printk(KERN_INFO"[write_thread]: file is closed: %d d2w: %d\n", file_desc->minor, d2w);
 					}
 				}
 				verbose_write_thread_printk(KERN_INFO"[write_thread]: No more data to write.....\n");
 			} else if(sz == 1 && !file_desc->file_activate) {
 				verbose_write_thread_printk(KERN_INFO"[write_thread]: file is not open!\n");
 				atomic_dec(file_desc->in_write_fifo_count);
 			} else {
 				printk(KERN_INFO"[write_thread]: !!!!!!!!ERROR kfifo_out returned zero\n");
 			}
 		}
 		verbose_write_thread_printk(KERN_INFO"[write_thread]: write_fifo is empty!\n");
 	}			//end  	while!kthread_should_stop()
 	verbose_printk(KERN_INFO"[write_thread]: Leaving write thread\n");
 	return 0;
 }

 bool is_packet_full_interrupt(struct file_desc * file_desc)
 {
	u32 status;
 	u64 axi_dest;

	 /*Read the axi fifo ISR*/
	 axi_dest = file_desc->axi_addr_ctl + AXI_STREAM_ISR;
	 if(direct_read(axi_dest, (void *)&status, 4, NORMAL_READ) ) {
	 	printk(KERN_INFO"[is_packet_full_interrupt]: !!!!!!!!ERROR direct_read\n");
	 	return ERROR;
	 }
	 verbose_isr_printk(KERN_INFO"[is_packet_full_interrupt]: Stream FIFO ISR status: 0x%08x\n", status);

	 if((status&AXI_INTR_RFPF_MASK) == AXI_INTR_RFPF_MASK){
		 return true;
	 }
	 return false;
 }

 int clear_fifo_isr(struct file_desc * file_desc)
 {
	u32 status;
 	u64 axi_dest;

	 /*Read the axi fifo ISR*/
	 axi_dest = file_desc->axi_addr_ctl + AXI_STREAM_ISR;
	 if(direct_read(axi_dest, (void *)&status, 4, NORMAL_READ) ) {
	 	printk(KERN_INFO"[clear_fifo_isr]: !!!!!!!!ERROR direct_read\n");
	 	return ERROR;
	 }

	 verbose_isr_printk(KERN_INFO"[clear_fifo_isr]: Stream FIFO ISR status: 0x%08x\n", status);
	 /*clear the axi fifo ISR*/
	 if(direct_write(axi_dest, (void *)&status, 4, NORMAL_WRITE) ) {
	 	printk(KERN_INFO"[clear_fifo_isr]: !!!!!!!!ERROR direct_write\n");
	 	return ERROR;
	 }

	 verbose_isr_printk(KERN_INFO"[clear_fifo_isr]: Clear fifo ISR sucessful!\n");
	 return 0;

 }


/**
 * This function is called by the read thread to read data from the FPGA.
 * read_size: the number of bytes to read from the fifo
 */
int read_data(struct file_desc * file_desc, int read_size, void * buffer_addr)
{
	int read_count;
	int max_can_read = 0;
	int rfu, rfh, full;

	rfh = atomic_read(file_desc->rfh);
	rfu = atomic_read(file_desc->rfu);
	full = atomic_read(file_desc->read_ring_buf_full);

	max_can_read = room_in_buffer(rfh, rfu, full, file_desc->dma_size);

	//make sure there is room in our buffer for the read
	if(( read_size + dma_byte_width + 4*sizeof(size_t) ) > max_can_read) {
		//ring buffer is full, cannot fit in our read
		if(back_pressure) {
			//if this is a stream packet we cannot read out the whole packet return 1
			if(file_desc->mode == AXI_STREAM_PACKET){
				verbose_printk(KERN_INFO"[read_data]: Packet size is larger than room in ring buffer, max_can_read: %d < read_size: %d \n", max_can_read, read_size);
				return 1;
			}
			//if this is not a stream packet read out what we can
			else {
				verbose_read_thread_printk(KERN_INFO"[read_data]: maximum read amount: %d\n", max_can_read);
				read_size = max_can_read - 4*sizeof(size_t) - dma_byte_width;
				verbose_read_thread_printk(KERN_INFO"[read_data]: read size: %d\n", read_size);
				if(read_size > 0){
					read_count = axi_stream_fifo_read(file_desc, (size_t)read_size, buffer_addr, file_desc->svd->dma_addr_base + file_desc->dma_offset_read, rfh, file_desc->dma_size);

					if(read_count < 0) {
						printk(KERN_INFO"[read_data]: !!!!!!!!ERROR reading data from axi stream fifo\n");
						//break;
						return ERROR;
					}
				}
				else {
					printk(KERN_INFO"[read_data]: No room in ring buffer to read data, setting read ring buffer locked.\n");
					atomic_set(file_desc->read_ring_buf_locked, 1);
					//break;
					return ERROR;
				}
			}

		} else {
			//----------------------------------------------drop data----------------------------------------------
			verbose_read_thread_printk(KERN_INFO"[read_data]: Needing to garbage data, max_can_read: %d < read_size: %d \n", max_can_read, read_size);

			//move user pointer to hw pointer to clear out the data in the ring buffer
			atomic_set(file_desc->rfu, rfh);
			atomic_set(file_desc->read_ring_buf_full, 0);
		}
	} else {
		//the ring buffer has room!
		verbose_read_thread_printk(KERN_INFO"[read_data]: maximum read amount: %d\n", max_can_read);
		verbose_read_thread_printk(KERN_INFO"[read_data]: read size: %d\n", read_size);

		read_count = axi_stream_fifo_read(file_desc, (size_t)read_size, buffer_addr, file_desc->svd->dma_addr_base + file_desc->dma_offset_read, rfh, file_desc->dma_size);

		if(read_count < 0) {
			printk(KERN_INFO"[read_data]: !!!!!!!!ERROR reading data from axi stream fifo\n");
			//break;
			return ERROR;
		}

	}
	//send signal to userspace poll()

	atomic_set(file_desc->atomic_poll, 1);
	verbose_read_thread_printk(KERN_INFO"[read_data]: waking up the poll.....%d\n", file_desc->minor);
	wake_up(&file_desc->poll_wq);

	return 0;
}


/**
 * @brief This function is called by the write thread to write data to the FPGA that is in the ring buffer.
 * @param file_desc The struct containing all the file variables.
 */
int write_data(struct file_desc * file_desc, void * buffer_addr)
{
	int wtk, wth, full;
	int d2w;
	//int cdma_num;
	size_t room_till_end;
	size_t remaining;
	size_t write_header_size;
	u64 axi_dest;
	u32 buf, read_reg;

	wth = atomic_read(file_desc->wth);
	wtk = atomic_read(file_desc->wtk);
	full = atomic_read(file_desc->write_ring_buf_full);

	d2w = data_in_buffer(wtk, wth, full, file_desc->dma_size);

	if( d2w > sizeof(write_header_size)) {

		room_till_end = file_desc->dma_size - wth;
		//If the header will not fit in 1 chuck of the ring buffer (in the room till end) then just get it at the start
		if(sizeof(write_header_size) > room_till_end) {
			wth = 0;
		}

		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: header memcpy (0x%p, 0x%p + 0x%x, 0x%zx)\n", &write_header_size, buffer_addr, wth, sizeof(write_header_size));
		memcpy(&write_header_size, buffer_addr+wth, sizeof(write_header_size));
		wth = get_new_ring_pointer(sizeof(write_header_size), wth, (int)(file_desc->dma_size), dma_byte_width);

		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: header: (0x%zx)\n", write_header_size);
		d2w -= sizeof(write_header_size);

	} else {
		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: no data to write\n");
		return 1;
	}

	//check the tx vacancy
	axi_dest = file_desc->axi_addr_ctl + AXI_STREAM_TDFV; // the transmit FIFO vacancy
	if( direct_read(axi_dest, (void*)(&buf), 4, NORMAL_READ) ) {
		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: !!!!!!!!ERROR reading from AXI Streaming FIFO control interface\n");
		return ERROR;
	}

	read_reg = buf*dma_byte_width;
	verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: vacancy: (0x%x)(bytes)\n", (u32)read_reg);
	verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: d2w: (0x%x)(bytes)\n", (u32)d2w);


	if(write_header_size > d2w) {
		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: header is bigger(%zd) than data to write(%d)! \n", write_header_size, d2w);
		return 1;
	} else if(write_header_size > read_reg) {
		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: header is bigger(%zd) than vacancy(%u)! \n", write_header_size, (u32)read_reg);
		return 1;
	}

	/*write to TDR register*/

	axi_dest = file_desc->axi_addr_ctl + AXI_STREAM_TDR;
	buf = file_desc->tx_dest;
	verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: Wrote 0x%x to TDR!!.\n", file_desc->tx_dest);
	if( direct_write(axi_dest, (void*)(&buf), 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[write_data]: !!!!!!!!ERROR writing to AXI Streaming FIFO Control Interface\n");
		return ERROR;
		}

	axi_dest = file_desc->axi_addr + AXI_STREAM_TDFD;
	//now we want to cdma write_header_size amount of data

	room_till_end = ((file_desc->dma_size - wth) & ~(dma_byte_width-1));					 //make it divisible by dma_byte_width
	if(write_header_size > room_till_end) {			//needs to be 2 copy
		remaining = write_header_size-room_till_end;

		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: dma_transfer 1 ring_buff address: 0x%p + 0x%x, AXI Address: %llx, len: 0x%zx)\n",
							buffer_addr, wth, axi_dest, room_till_end);

		if(dma_transfer(file_desc, axi_dest, buffer_addr, room_till_end, KEYHOLE_WRITE, wth) ) { //unsuccessful DMA transmission
			verbose_printk(KERN_INFO"[write_data]: !!!!!!!!ERROR on CDMA READ!!!.\n");
		}
		//extra verbose debug message
		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: first 4 bytes 0x%08x\n", *((u32*)(buffer_addr+wth)));
		wth = 0;

		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: dma_transfer 2 ring_buff address: 0x%p + 0x%x, AXI Address: %llx, len: 0x%zx)\n",
							buffer_addr, wth, axi_dest, remaining);
		if(dma_transfer(file_desc, axi_dest, buffer_addr, remaining, KEYHOLE_WRITE, wth) ) { //unsuccessful DMA transmission
			verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: !!!!!!!!ERROR on CDMA READ!!!.\n");
		}
		wth = get_new_ring_pointer(remaining, wth, (int)(file_desc->dma_size), dma_byte_width);

	} else {														//only 1 copy
		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: dma_transfer ring_buff address: 0x%p + 0x%x, AXI Address: %llx, len: 0x%zx)\n",
							buffer_addr, wth, axi_dest, write_header_size);
		if(dma_transfer(file_desc, axi_dest, buffer_addr, write_header_size, KEYHOLE_WRITE, wth) ) { //unsuccessful CDMA transmission
			verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: !!!!!!!!ERROR on CDMA READ!!!.\n");
		}

		//extra verbose debug message
		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: first 4 bytes 0x%08x\n", *((u32*)(buffer_addr+wth)));
		wth = get_new_ring_pointer(write_header_size, wth, (int)(file_desc->dma_size), dma_byte_width);
	}

	/*write to ctl interface*/
	axi_dest = file_desc->axi_addr_ctl + AXI_STREAM_TLR;
	buf = (u32)write_header_size;
	verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: Wrote 0x%x to TLR!!.\n", (u32)write_header_size);
	if( direct_write(axi_dest, (void*)(&buf), 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[write_data]: !!!!!!!!ERROR writing to AXI Streaming FIFO Control Interface\n");
		return ERROR;
	}

	//Report status after transfer
	axi_dest = file_desc->axi_addr_ctl + AXI_STREAM_ISR;
	if( direct_read(axi_dest, (void*)(&buf), 4, NORMAL_READ) ) {
		printk(KERN_INFO"[write_data]: !!!!!!!!ERROR reading AXI_STREAM_ISR register.\n");
		return ERROR;
	}
	verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: AXI_STREAM_ISR register: ('0x%08x')\n", buf);

	//update wth pointer
	atomic_set(file_desc->wth, wth);
	//if the ring buffer was full it is no longer
	if(full) {
		atomic_set(file_desc->write_ring_buf_full, 0);
		verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: ring_point_%d : write full: %d\n", file_desc->minor, 0);
	}

	verbose_axi_fifo_write_printk(KERN_INFO"[write_data]: write ring_buffer: WTH: %d WTK: %d\n", wth, atomic_read(file_desc->wtk));

	atomic_set(file_desc->pci_write_q, 1);
	wake_up_interruptible(&file_desc->svd->pci_write_head);

	verbose_axi_fifo_write_printk(KERN_INFO"[axi_stream_fifo_write]: Leaving write_data\n");

	return 0;

}



/**
 * This function checks if there is any available data to be read from an axi stream fifo.
 * file_desc The struct containing all the file variables.
 *		returns 0 if the occupancy is zero, otherwise returns read fifo length
 */
size_t axi_stream_fifo_d2r(struct file_desc * file_desc)
{
	u32 buf;
	u64 axi_dest;

	if(file_desc->mode == AXI_STREAM_PACKET) {

		/*Read FIFO Fill level RDFO */
		axi_dest = file_desc->axi_addr_ctl + AXI_STREAM_RDFO;
		if( direct_read(axi_dest, (void*)(&buf), 4, NORMAL_READ) ) {
			printk(KERN_INFO"[axi_stream_fifo_d2r]: !!!!!!!!ERROR reading Read FIFO fill level (RDFO)\n");
			return ERROR;
		}
		verbose_axi_fifo_d2r_printk(KERN_INFO"[axi_stream_fifo_d2r]: RDFO value = 0x%08x\n", buf);
		file_desc->axi_fifo_rdfo = buf;

		//verify that the occupancy is not zero	MM
		if(buf == 0){
			verbose_axi_fifo_d2r_printk(KERN_INFO"[axi_stream_fifo_d2r]: Receive occupancy is zero, nothing to read\n");
			return 0;
		}



		/*Read FIFO Fill level RDR (tdest)*/
		axi_dest = file_desc->axi_addr_ctl + AXI_STREAM_RDR;

		if( direct_read(axi_dest, (void*)(&buf), 4, NORMAL_READ) ) {
			printk(KERN_INFO"[axi_stream_fifo_d2r]: !!!!!!!!ERROR reading Read FIFO dest (RDR)\n");
			return ERROR;
		}
		verbose_axi_fifo_d2r_printk(KERN_INFO"[axi_stream_fifo_d2r]: RDR value = 0x%08x\n", buf);
		file_desc->rx_dest = buf;

		/*Read FIFO Fill level RLR (byte count)*/
		axi_dest = file_desc->axi_addr_ctl + AXI_STREAM_RLR;

		if( direct_read(axi_dest, (void*)(&buf), 4, NORMAL_READ) ) {
			printk(KERN_INFO"[axi_stream_fifo_d2r]: !!!!!!!!ERROR reading Read FIFO fill level (RLR)\n");
			return ERROR;
		}
		verbose_axi_fifo_d2r_printk(KERN_INFO"[axi_stream_fifo_d2r]: RLR value = 0x%08x\n", buf);
		file_desc->axi_fifo_rlr = buf; //not sure why you would and here...... & 0x7fffffff; // remember the read

		return buf;

	}
	else if(file_desc->mode == AXI_STREAM_FIFO){

		/*Read FIFO Fill level RLR (byte count)*/
		axi_dest = file_desc->axi_addr_ctl + AXI_STREAM_RLR;

		if( direct_read(axi_dest, (void*)(&buf), 4, NORMAL_READ) ) {
			printk(KERN_INFO"[axi_stream_fifo_d2r]: !!!!!!!!ERROR reading Read FIFO fill level (RLR)\n");
			return ERROR;
		}
		verbose_axi_fifo_d2r_printk(KERN_INFO"[axi_stream_fifo_d2r]: RLR value = 0x%08x\n", buf);

		buf = buf & 0x7FFFFFFF;								//mask out the highest bit (that indicates partial packet)

		file_desc->axi_fifo_rlr = buf; //not sure why you would and here...... & 0x7fffffff; // remember the read

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
 * file_desc: The struct containing all the file variables.
 * count: number of bytes to read from the fifo
 * buffer_addr: the base address of the ring buffer to copy data into
 * hw_base_addr the hw address of the same buffer from above (not needed for xdma transfer)
 * ring_pointer_offset: The current offset of the ring pointer in memory to store data.
 * returns the new ring_pointer_offset
 */
size_t axi_stream_fifo_read(struct file_desc * file_desc, size_t count, void * buffer_addr, u64 hw_base_addr, int ring_pointer_offset, size_t buf_size) {
	size_t room_till_end;
	size_t remaining;
	u64 axi_dest;
	size_t zero = 0;

	//clear values in file_desc
	file_desc->axi_fifo_rlr = file_desc->axi_fifo_rdfo = 0;

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

	verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read]: header copy_from_user(0x%p + 0x%x, 0x%p, 0x%zx)\n", buffer_addr, ring_pointer_offset, (void * )&count, sizeof(count));

	memcpy(buffer_addr+ring_pointer_offset, (void * )&count, sizeof(count));
	ring_pointer_offset = get_new_ring_pointer(sizeof(count), ring_pointer_offset, (int)buf_size, dma_byte_width);

	//}
	room_till_end = ( (buf_size - ring_pointer_offset) & ~(dma_byte_width-1));				 //make it divisible by dma_byte_width
	axi_dest = file_desc->axi_addr + AXI_STREAM_RDFD;

	if(count > room_till_end) {			//needs to be 2 cdma transfers
		remaining = count-room_till_end;

		verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read]: dma_transfer 1 AXI Address: 0x%llx, ring_buff address: 0x%llx + 0x%x, len: 0x%zx\n", axi_dest, hw_base_addr, ring_pointer_offset, room_till_end);
		if(dma_transfer(file_desc, axi_dest, buffer_addr, room_till_end, KEYHOLE_READ, ring_pointer_offset) ) { //unsuccessful CDMA transmission
			printk(KERN_INFO"[axi_stream_fifo_read]: !!!!!!!!ERROR on CDMA READ!!!.\n");
		}

		//extra verbose debug message
		verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read]: first 4 bytes 0x%08x\n", *((u32*)(buffer_addr+ring_pointer_offset)));

		ring_pointer_offset = 0;
		//end of buffer reached
		verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read]: dma_transfer 2 AXI Address: 0x%llx, ring_buff address: 0x%llx + 0x%x, len: 0x%zx\n", axi_dest, hw_base_addr, ring_pointer_offset, remaining);
		if(dma_transfer(file_desc, axi_dest, buffer_addr, remaining, KEYHOLE_READ, ring_pointer_offset) ) { //unsuccessful CDMA transmission
			printk(KERN_INFO"[axi_stream_fifo_read]: !!!!!!!!ERROR on CDMA READ!!!.\n");
		}
		ring_pointer_offset = get_new_ring_pointer(remaining, ring_pointer_offset, (int)buf_size, dma_byte_width);

	} else {														//only 1 cdma transfer
		verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read]: dma_transfer AXI Address: 0x%llx, ring_buff address: 0x%llx + 0x%x, len: 0x%zx\n", axi_dest, hw_base_addr, ring_pointer_offset , count);
		if(dma_transfer(file_desc, axi_dest, buffer_addr, count, KEYHOLE_READ, ring_pointer_offset ) ) { //unsuccessful CDMA transmission
			printk(KERN_INFO"[axi_stream_fifo_read]: !!!!!!!!ERROR on CDMA READ!!!.\n");
		}


		//extra verbose debug message
		verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read]: first 4 bytes 0x%08x\n", *((u32*)(buffer_addr+ring_pointer_offset)));


		ring_pointer_offset = get_new_ring_pointer(count, ring_pointer_offset, (int)buf_size, dma_byte_width);
	}

	//copy the 0 to the ring buffer to act as a "0 header" to mark inbetween packets
	room_till_end = buf_size - ring_pointer_offset;
	if(sizeof(zero) > room_till_end) {		//If the trailer will not fit in 1 chuck of the ring buffer (in the room till end) then just put it at the start
		ring_pointer_offset = 0;
	}

	verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read]: trailer copy_from_user(0x%p + 0x%x, 0x%p, 0x%zx)\n", buffer_addr, ring_pointer_offset, (void * )&zero, sizeof(zero));
	memcpy(buffer_addr+ring_pointer_offset, (void * )&zero, sizeof(zero));
	ring_pointer_offset = get_new_ring_pointer(sizeof(zero), ring_pointer_offset, (int)buf_size, dma_byte_width);

	/*update rfh pointer*/
	atomic_set(file_desc->rfh, ring_pointer_offset);
	//We increased the rfh (head) if it is now equal to the rfu (tail) then the buffer is full
	if(atomic_read(file_desc->rfu) == ring_pointer_offset) {
		atomic_set(file_desc->read_ring_buf_full, 1);
		verbose_printk(KERN_INFO"[axi_stream_fifo_read]: ring_point: Read Full: %d\n", atomic_read(file_desc->read_ring_buf_full));
	}
	verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read]: read ring_buffer: RFU : %d RFH %d\n", atomic_read(file_desc->rfu), ring_pointer_offset);


	verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read]: Leaving the READ AXI Stream FIFO routine %zd\n", count);
	return ring_pointer_offset;
}




/**
 * This function performs calls appropriate functions for Reading from the AXI Streaming FIFO.
 * count: The number of bytes to read from the AXI streaming FIFO.
 * file_desc: The struct containing all the file variables.
 * ring_pointer_offset: The current offset of the ring pointer in memory to store data.
 * returns the number of bytes read
 */
size_t axi_stream_fifo_read_direct(struct file_desc * file_desc, size_t count, char * buf_base_addr, size_t buf_size) {
	u64 axi_dest;

	//clear values in file_desc
	file_desc->axi_fifo_rlr = file_desc->axi_fifo_rdfo = 0;

	if(count == 0) {
		verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read_direct]: There is either no data to read, or less than 8 bytes.\n");
		return 0; // nothing to read
	} else {
		verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read_direct]: count = %zd.\n", count);
	}

	axi_dest = file_desc->axi_addr + AXI_STREAM_RDFD;

	if(count > buf_size) {			//needs to be 2 cdma transfers
		//there is not enough room in the buffer
		printk(KERN_INFO"[axi_stream_fifo_read_direct]: not enough room in the buffer!!!.\n");
		return 0;

	}
	//only 1 cdma transfer
	verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read_direct]: dma_transfer AXI Address: 0x%llx, ring_buff address: 0x%p, len: 0x%zx\n", axi_dest, (void *)buf_base_addr, count);

	if(dma_transfer(file_desc, axi_dest, (void *)buf_base_addr, count, KEYHOLE_READ, 0) ) { //unsuccessful CDMA transmission
		printk(KERN_INFO"[axi_stream_fifo_read_direct]: !!!!!!!!ERROR on CDMA READ!!!.\n");
	}

	verbose_axi_fifo_read_printk(KERN_INFO"[axi_stream_fifo_read_direct]: Leaving the READ AXI Stream FIFO routine %zd\n", count);
	return count;
}

/**
 * This function sets the initial values for the ring buffer
 *
 */
void ring_buffer_init(struct file_desc * file_desc)
{
	verbose_printk(KERN_INFO"[pci_%x_ioctl]: Initializing the FIFO and setting registers\n", file_desc->minor);
	/*set the ring buff full*/
	atomic_set(file_desc->write_ring_buf_full, 0);
	atomic_set(file_desc->read_ring_buf_full, 0);
	atomic_set(file_desc->write_ring_buf_locked, 0);
	atomic_set(file_desc->read_ring_buf_locked, 0);
	/*set the pointer defaults*/
	verbose_printk(KERN_INFO"[pci_%x_ioctl]: read ring_buffer: RFU : %d RFH %d\n", file_desc->minor, 0, 0);
	atomic_set(file_desc->wtk, 0);
	atomic_set(file_desc->wth, 0);
	verbose_printk(KERN_INFO"[pci_%x_ioctl]: write ring_buffer: WTH: %d WTK: %d\n", file_desc->minor, 0, 0);
	atomic_set(file_desc->rfh, 0);
	atomic_set(file_desc->rfu, 0);
}

/**
 * This function performs calls appropriate functions for writing to the AXI Streaming FIFO.
 * count The number of bytes to write to the AXI streaming FIFO.
 * file_desc The struct containing all the file variables.
 * ring_pointer_offset The current offset of the ring pointer in memory for data to write.
 * return ERROR for error, 0 otherwise
 */
int axi_stream_fifo_init(struct file_desc * file_desc)
{
	u64 axi_dest;
	u32 buf;

	if((file_desc->axi_addr == -1) | (file_desc->axi_addr_ctl == -1)) {
		printk(KERN_INFO"[pci_%x_ioctl]: !!!!!!!!ERROR: axi addresses of AXI STREAM FIFO not set\n", file_desc->minor);
		printk(KERN_INFO"[pci_%x_ioctl]: \tset the AXI addresses then set mode again\n", file_desc->minor);
		return ERROR;
	}

	//reset The transmit side
	axi_dest = file_desc->axi_addr_ctl + AXI_STREAM_TDFR;
	buf = XLLF_TDFR_RESET_MASK;
	if( direct_write(axi_dest, (void*)(&buf), 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[axi_stream_fifo_%x_init]: !!!!!!!!ERROR resetting TX fifo\n", file_desc->minor);
		return ERROR;
	}
	verbose_printk(KERN_INFO"[axi_stream_fifo_%x_init]: Reset the TX axi fifo\n", file_desc->minor);

	//reset The receive side
	axi_dest = file_desc->axi_addr_ctl + AXI_STREAM_RDFR;
	buf = XLLF_RDFR_RESET_MASK;
	if( direct_write(axi_dest, (void*)(&buf), 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[axi_stream_fifo_%x_init]: !!!!!!!!ERROR resetting RX fifo\n", file_desc->minor);
		return ERROR;
	}
	verbose_printk(KERN_INFO"[axi_stream_fifo_%x_init]: Reset the RX axi fifo\n", file_desc->minor);

	//Read CTL ISR interface
	axi_dest = file_desc->axi_addr_ctl + AXI_STREAM_ISR;
	if( direct_read(axi_dest, (void*)(&buf), 4, NORMAL_READ) ) {
		printk(KERN_INFO"[axi_stream_fifo_%x_init]: !!!!!!!!ERROR reading AXI_STREAM_ISR register\n", file_desc->minor);
		return ERROR;
	}
	verbose_printk(KERN_INFO"[axi_stream_fifo_%x_init]: AXI_STREAM_ISR register: ('0x%08x')\n", file_desc->minor, buf);

	//reset interrupts on CTL interface
	axi_dest = file_desc->axi_addr_ctl + AXI_STREAM_ISR;
	//buf = 0xFFFFFFFF;
	if( direct_write(axi_dest, (void*)(&buf), 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[axi_stream_fifo_%x_init]: !!!!!!!!ERROR resetting AXI_STREAM_ISR register.\n", file_desc->minor);
		return ERROR;
	}
	verbose_printk(KERN_INFO"[axi_stream_fifo_%x_init]: Reset the interrupts on the axi fifo\n", file_desc->minor);

	//Read CTL interface
	axi_dest = file_desc->axi_addr_ctl + AXI_STREAM_ISR;
	if( direct_read(axi_dest, (void*)(&buf), 4, NORMAL_READ) ) {
		printk(KERN_INFO"[axi_stream_fifo_%x_init]: !!!!!!!!ERROR reading AXI_STREAM_ISR register.\n", file_desc->minor);
		return ERROR;
	}
	verbose_printk(KERN_INFO"[axi_stream_fifo_%x_init]: AXI_STREAM_ISR register: ('0x%08x')\n", file_desc->minor, buf);
	//-----------------------------------------------------------------mm-----------------------------------------

	//enable interupt from axi_stream_fifo if we are using the axi_stream_packet
	if(file_desc->mode == AXI_STREAM_PACKET) {
		verbose_printk(KERN_INFO"[axi_stream_fifo_%x_init]: Setting AXI_STREAM_IER register for AXI_STREAM_PACKET.\n", file_desc->minor);
		//Set IER Register for interrupt on read
		axi_dest = file_desc->axi_addr_ctl + AXI_STREAM_IER;
		buf = AXI_INTR_RC_MASK;
		if( direct_write(axi_dest, (void*)(&buf), 4, NORMAL_WRITE) ) {
			printk(KERN_INFO"[axi_stream_fifo_%x_init]: !!!!!!!!ERROR setting AXI_STREAM_IER register.\n", file_desc->minor);
			return ERROR;
		}
	}
	//enable interupt from axi_stream_fifo if we are using the axi_stream_packet
	else {
		verbose_printk(KERN_INFO"[axi_stream_fifo_%x_init]: Setting AXI_STREAM_IER register for AXI_STREAM.\n", file_desc->minor);
		//Set IER Register for interrupt on read
		axi_dest = file_desc->axi_addr_ctl + AXI_STREAM_IER;
		buf = AXI_INTR_RFPF_MASK;
		if( direct_write(axi_dest, (void*)(&buf), 4, NORMAL_WRITE) ) {
			printk(KERN_INFO"[axi_stream_fifo_%x_init]: !!!!!!!!ERROR setting AXI_STREAM_IER register.\n", file_desc->minor);
			return ERROR;
		}
	}

	axi_dest = file_desc->axi_addr_ctl + AXI_STREAM_TDFV; // the transmit FIFO vacancy
	if( direct_read(axi_dest, (void*)(&buf), 4, NORMAL_READ) ) {
		printk(KERN_INFO"[axi_stream_fifo_%x_init]: !!!!!!!!ERROR reading tx side vacancy\n", file_desc->minor);
		return ERROR;
	}
	file_desc->tx_fifo_size = buf*dma_byte_width;
	/*Check to see if the calculated fifo empty level via the DMA data byte width (aka the axi-s fifo byte width)
	 * and file size (aka the fifo size) is equal to the actual fifo empty level when read */
	printk(KERN_INFO"[axi_stream_fifo_%x_init]: file size: ('0x%08x')\n", file_desc->minor, (u32)file_desc->file_size);
	printk(KERN_INFO"[axi_stream_fifo_%x_init]: register read size: ('0x%08x')\n", file_desc->minor, file_desc->tx_fifo_size);
	printk(KERN_INFO"[axi_stream_fifo_%x_init]: Receive axi-s fifo initialized\n", file_desc->minor);

	return 0;
}

/**
 * This function performs calls appropriate functions for writing to the AXI Streaming FIFO.
 * count The number of bytes to write to the AXI streaming FIFO.
 * file_desc The struct containing all the file variables.
 * ring_pointer_offset The current offset of the ring pointer in memory for data to write.
 * return ERROR for error, 0 otherwise
 */
int axi_stream_fifo_deinit(struct file_desc * file_desc)
{
	u64 axi_dest;
	u32 buf;
	//READ ISR
	axi_dest = file_desc->axi_addr_ctl + AXI_STREAM_ISR;
	if( direct_read(axi_dest, (void*)(&buf), 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[axi_stream_fifo_%x_deinit]: !!!!!!!!ERROR setting AXI_STREAM_ISR register.\n", file_desc->minor);
		return ERROR;
	}
	printk(KERN_INFO"[axi_stream_fifo_%x_deinit]: ISR 0x%08x\n", file_desc->minor, buf);


	//end debugging




	//Set IER Register for no interrupts
	axi_dest = file_desc->axi_addr_ctl + AXI_STREAM_IER;
	buf = 0x00000000;
	if( direct_write(axi_dest, (void*)(&buf), 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[axi_stream_fifo_%x_deinit]: !!!!!!!!ERROR setting AXI_STREAM_IER register.\n", file_desc->minor);
		return ERROR;
	}
	printk(KERN_INFO"[axi_stream_fifo_%x_deinit]: Fifo interupt disabled\n", file_desc->minor);

	return 0;
}


int copy_from_ring_buffer(struct file_desc * file_desc, void* buf, size_t count, void * buffer_addr)
{
	int rfh, rfu, full, d2r;
	unsigned long bytes_to_copy, rc, bytes_copied = 0;
	size_t read_header_size, room_till_end, remaining;

	/* -----------------------------------------------------------------------------------
	 *	 Ring buffer Code Start */
	/*for the ring buffer case, we just need to determine a pointer location and a size
	 * to give to the copy_to_user function */
	rfh = atomic_read(file_desc->rfh);
	rfu = atomic_read(file_desc->rfu);
	full = atomic_read(file_desc->read_ring_buf_full); //The thread has full when the atomic variable is 0

	d2r = data_in_buffer(rfh, rfu, full, file_desc->dma_size);		   //d2r is data in the buffer

	verbose_pci_read_printk(KERN_INFO"\t[copy_from_ring_buffer_%x]: the amount of bytes requested: 0x%zx\n", file_desc->minor, count);
	verbose_pci_read_printk(KERN_INFO"\t[copy_from_ring_buffer_%x]: the amount of data in the ring buffer: 0x%x\n", file_desc->minor, d2r);


	//if there is data in the ring buffer
	if(d2r > 0) {
		// if there is no left over read_header_size, then read the packet header read_header_size from the ring buffer
		if(file_desc->read_header_size == 0) {
			room_till_end = file_desc->dma_size - rfu;
			//If the header will not fit in 1 chuck of the ring buffer (in the room till end) then just put it at the start
			if(sizeof(read_header_size) > room_till_end) {
				rfu = 0;
			}
			verbose_pci_read_printk(KERN_INFO"\t[copy_from_ring_buffer_%x]: header copy(0x%p, 0x%p + 0x%x, 0x%zx)\n", file_desc->minor, &read_header_size, buffer_addr, rfu, sizeof(read_header_size));
			memcpy(&read_header_size, buffer_addr+rfu, sizeof(read_header_size));
			rfu = get_new_ring_pointer((int)sizeof(read_header_size), rfu, (int)(file_desc->dma_size), dma_byte_width);
		} else {
			read_header_size = file_desc->read_header_size;
		}
		verbose_pci_read_printk(KERN_INFO"\t[copy_from_ring_buffer_%x]: header: (0x%zx)\n" , file_desc->minor, read_header_size);

		//if the header is  bigger than data in the buffer
		if(read_header_size > d2r) {
			printk(KERN_INFO"\t[copy_from_ring_buffer_%x]: !!!!!!!!ERROR read_header_size: 0x%zx is larger than data to read: 0x%x\n" , file_desc->minor, read_header_size, d2r);
			bytes_to_copy = 0;
		} else if(read_header_size > count) {   //can't read more than requested data
			verbose_pci_read_printk(KERN_INFO"\t[copy_from_ring_buffer_%x]: read_header_size: 0x%zx is larger than requested data: 0x%lx\n" , file_desc->minor, read_header_size, count);
			bytes_to_copy = count;
		} else {		//read_header_size <= count
			verbose_pci_read_printk(KERN_INFO"\t[copy_from_ring_buffer_%x]: setting read count to 0x%zx\n" , file_desc->minor, read_header_size);
			bytes_to_copy = read_header_size;
		}

		//if there is a packet to read
		if(bytes_to_copy != 0) {
			room_till_end = ( (file_desc->dma_size - rfu) & ~(dma_byte_width-1));				 //make it divisible by dma_byte_width

			if(bytes_to_copy > room_till_end) { //need to do two read since we are at the edge
				remaining = bytes_to_copy - room_till_end;
				bytes_to_copy = room_till_end;
				verbose_pci_read_printk(KERN_INFO"\t[copy_from_ring_buffer_%x]: copy_to_user 1(0x%p, 0x%p + 0x%x, 0x%lx)\n", file_desc->minor, &buf, buffer_addr, rfu, bytes_to_copy);
				rc = copy_to_user(buf, buffer_addr + rfu, bytes_to_copy);
				bytes_copied = bytes_copied + bytes_to_copy - rc;
				bytes_to_copy = rc;
				while(rc){
					verbose_pci_read_printk(KERN_INFO"\t[copy_from_ring_buffer_%x]: copy_to_user 1(0x%p, 0x%p + 0x%lx, 0x%lx)\n", file_desc->minor, &buf + bytes_copied, buffer_addr, rfu + bytes_copied, bytes_to_copy);
					rc = copy_to_user(buf + bytes_copied, buffer_addr + rfu + bytes_copied, bytes_to_copy);
					bytes_copied = bytes_copied + bytes_to_copy - rc;
					bytes_to_copy = rc;
				}

				//extra verbose debug message
				verbose_pci_read_printk(KERN_INFO"\t[copy_from_ring_buffer_%x]: first 4 bytes 0x%08x\n", file_desc->minor, *((u32*)(buffer_addr+rfu)));

				rfu = 0;
				bytes_to_copy = remaining;
				//end of buffer reached
				verbose_pci_read_printk(KERN_INFO"\t[copy_from_ring_buffer_%x]: copy_to_user 2(0x%p, 0x%p + 0x%x, 0x%lx)\n", file_desc->minor, &buf+room_till_end, buffer_addr, rfu, bytes_to_copy);
				rc = copy_to_user(buf+room_till_end, buffer_addr + rfu, bytes_to_copy);
				bytes_copied = bytes_copied + bytes_to_copy - rc;
				bytes_to_copy = rc;

				while(rc){
					verbose_pci_read_printk(KERN_INFO"\t[copy_from_ring_buffer_%x]: copy_to_user 1(0x%p, 0x%p + 0x%lx, 0x%lx)\n", file_desc->minor, &buf + bytes_copied, buffer_addr, rfu + bytes_copied, bytes_to_copy);
					rc = copy_to_user(buf + bytes_copied, buffer_addr + rfu + bytes_copied, room_till_end);
					bytes_copied = bytes_copied + bytes_to_copy - rc;
					bytes_to_copy = rc;
				}

				rfu = get_new_ring_pointer((int)remaining, rfu, (int)file_desc->dma_size, dma_byte_width);

			} else { 		//else we can do it in 1 read
				verbose_pci_read_printk(KERN_INFO"\t[copy_from_ring_buffer_%x]: copy_to_user(0x%p, 0x%p + 0x%x, 0x%zx)\n", file_desc->minor, &buf, buffer_addr, rfu, bytes_to_copy);
				rc = copy_to_user(buf, buffer_addr + rfu, bytes_to_copy);
				bytes_copied = bytes_copied + bytes_to_copy - rc;
				bytes_to_copy = rc;

				while(rc){
					bytes_to_copy -= rc;
					verbose_pci_read_printk(KERN_INFO"\t[copy_from_ring_buffer_%x]: copy_to_user(0x%p, 0x%p + 0x%lx, 0x%lx)\n", file_desc->minor, &buf + bytes_copied, buffer_addr, rfu + bytes_copied, bytes_to_copy);
					rc = copy_to_user(buf + bytes_copied, buffer_addr + rfu + bytes_copied, bytes_to_copy);
					bytes_copied = bytes_copied + bytes_to_copy - rc;
					bytes_to_copy = rc;
				}

				//extra verbose debug message
				verbose_pci_read_printk(KERN_INFO"\t[copy_from_ring_buffer_%x]: first 4 bytes 0x%08x\n", file_desc->minor, *((u32*)(buffer_addr+rfu)));
				rfu = get_new_ring_pointer((int)bytes_copied, rfu, (int)file_desc->dma_size, dma_byte_width);
			}

			verbose_pci_read_printk(KERN_INFO"\t[copy_from_ring_buffer_%x]: bytes copied to user 0x%zx\n", file_desc->minor, bytes_copied);

			//if we didn't read the whole packet out then write a header back in for the next read
			//we need to write the header before the data read out, so rfu will need to be decreased
			//because we did not write the rfu yet, we do not have to worry about data being put there yet
			if(bytes_copied < read_header_size) {
				read_header_size -= bytes_copied;
				file_desc->read_header_size = read_header_size;
				verbose_pci_read_printk(KERN_INFO"\t[copy_from_ring_buffer_%x]: set file_desc->read_header_size to (0x%zx)\n", file_desc->minor, read_header_size);
			} else {
				file_desc->read_header_size = 0;
			}
		} else {
			verbose_pci_read_printk(KERN_INFO"\t[copy_from_ring_buffer_%x]: 0 size packet buffer\n" , file_desc->minor);
			bytes_copied = 0;
		}

		atomic_set(file_desc->rfu, rfu);

		//if we were full, we just read data out of the ring buffer so we are no longer full
		if(full && bytes_copied != 0) {
			atomic_set(file_desc->read_ring_buf_full, 0);
			verbose_pci_read_printk(KERN_INFO"\t[copy_from_ring_buffer_%x]: ring_point : Read full: %d\n", file_desc->minor, 0);
		}
		verbose_pci_read_printk(KERN_INFO"\t[copy_from_ring_buffer_%x]: copied 0x%lx from the ring buffer\n", file_desc->minor, bytes_copied);
		verbose_pci_read_printk(KERN_INFO"\t[copy_from_ring_buffer_%x]: read ring_buffer: RFU : %d RFH %d\n", file_desc->minor, rfu, atomic_read(file_desc->rfh));

	} else {
		verbose_pci_read_printk(KERN_INFO"\t[copy_from_ring_buffer_%x]: max can read is 0.\n", file_desc->minor);
		bytes_copied = 0;
	}

	return bytes_copied;
}


/*
*	Copies data at buf to our ring buffer at buffer_addr
* 	returns the number of bytes copied, else returns 0 if could not copy
*
*
*/


int copy_to_ring_buffer(struct file_desc * file_desc, void* buf, size_t count, void * buffer_addr)
{
	int wtk, wth, full, rc, bytes = 0;
	size_t room_till_end, remaining;

	verbose_copy_ring_buf_printk(KERN_INFO"\t[copy_to_ring_buffer_%x]: the amount of bytes being copied to the ring buffer: %zu\n", file_desc->minor, count);

	wtk = atomic_read(file_desc->wtk);
	wth = atomic_read(file_desc->wth);
	full = atomic_read(file_desc->write_ring_buf_full);
	verbose_copy_ring_buf_printk(KERN_INFO"\t[copy_to_ring_buffer_%x]: wtk %d wth %d full %d dma_byte_size %d room_in_buffer %d active %d\n",file_desc->minor,wtk,wth,full,dma_byte_width,room_in_buffer(wtk, wth, full, file_desc->dma_size),file_desc->file_activate);
	//we are going to write the whole count + header
	if( (count + dma_byte_width + 2*sizeof(count)) > room_in_buffer(wtk, wth, full, file_desc->dma_size) && file_desc->file_activate ) {
		 return 0;
	}

	//if we need to wrap around the ring buffer....
	//copy the count to the ring buffer to act as the "header"
	room_till_end = (int)file_desc->dma_size - wtk;
	if(sizeof(count) > room_till_end) {		//If the header will not fit in 1 chuck of the ring buffer (in the room till end) then just get it at the start
		wtk = 0;
	}

	verbose_copy_ring_buf_printk(KERN_INFO"\t[copy_to_ring_buffer_%x]: header memcpy(0x%p + 0x%x, 0x%p, 0x%zx)\n", file_desc->minor, buffer_addr, wtk, (void * )&count, sizeof(count));
	memcpy(buffer_addr+wtk, (void * )&count, sizeof(count));
	wtk = get_new_ring_pointer((int)sizeof(count), wtk, (int)file_desc->dma_size, dma_byte_width);

	room_till_end = (int)( (file_desc->dma_size - wtk) & ~(dma_byte_width-1));				 //make it divisible by dma_byte_width
	if(count > room_till_end) {		//if header size is larger than room till the end, write in two steps
		remaining = count-room_till_end;
		//write the room_till_end
		verbose_copy_ring_buf_printk(KERN_INFO"\t[copy_to_ring_buffer_%x]: copy_from_user 1(0x%p + 0x%x, 0x%p, 0x%zx)\n", file_desc->minor, buffer_addr, wtk, buf, room_till_end);
		rc = copy_from_user(buffer_addr+wtk, buf, room_till_end);
		if(rc) {
			printk(KERN_INFO"[copy_to_ring_buffer_%x]: !!!!!!!!ERROR copy_from_user rc: 0x%x\n", file_desc->minor, rc);
			return ERROR;
		}
		bytes += room_till_end;
		wtk = 0;																																										//end of buffer reached

		verbose_copy_ring_buf_printk(KERN_INFO"\t[copy_to_ring_buffer_%x]: copy_from_user 2(0x%p + 0x%x, 0x%p, 0x%zx)\n", file_desc->minor, buffer_addr, wtk, buf+room_till_end, remaining);
		rc = copy_from_user(buffer_addr+wtk, buf+room_till_end, remaining);
		if(rc) {
			printk(KERN_INFO"[copy_to_ring_buffer_%x]: !!!!!!!!ERROR copy_from_user rc: 0x%x\n", file_desc->minor, rc);
			return ERROR;
		}
		bytes += remaining;
		wtk = get_new_ring_pointer((int)remaining, wtk, (int)file_desc->dma_size, dma_byte_width);
	} else {
		verbose_copy_ring_buf_printk(KERN_INFO"\t[copy_to_ring_buffer_%x]: copy_from_user(0x%p + 0x%x, 0x%p, 0x%zx)\n", file_desc->minor, buffer_addr, wtk, buf, count);
		rc = copy_from_user(buffer_addr+wtk, buf, count);
		if(rc) {
			printk(KERN_INFO"[copy_to_ring_buffer_%x]: !!!!!!!!ERROR copy_from_user rc: 0x%x\n", file_desc->minor, rc);
			return ERROR;
		}
		bytes = count;
		wtk = get_new_ring_pointer((int)count, wtk, (int)file_desc->dma_size, dma_byte_width);
	}

	atomic_set(file_desc->wtk, wtk);

	/*This says that if the wtk pointer has caught up to the WTH pointer, give priority to the WTH*/
	if(atomic_read(file_desc->wth) == wtk) {
		atomic_set(file_desc->write_ring_buf_full, 1);
		verbose_copy_ring_buf_printk(KERN_INFO"\t[copy_to_ring_buffer_%x]: ring_point_%d: ring buff priority: %d\n", file_desc->minor, file_desc->minor, atomic_read(file_desc->write_ring_buf_full));
	}

	verbose_copy_ring_buf_printk(KERN_INFO"\t[copy_to_ring_buffer_%x]: copied 0x%x to the ring buffer\n", file_desc->minor, bytes);
	verbose_copy_ring_buf_printk(KERN_INFO"\t[copy_to_ring_buffer_%x]: write ring_buffer: WTH: %d WTK: %d\n", file_desc->minor, atomic_read(file_desc->wth), wtk);

	return bytes;
}


int align_dma(int addr, int dma_byte_width)
{
	return (addr + dma_byte_width - 1) & ~(dma_byte_width - 1);
}


/**
 * This function is will update a ring pointer given the amount of bytes written or read.
 * bytes_written The number of bytes to advance the ring pointer
 * ring_pointer_offset The current ring pointer offset.
 * file_size The size of the ring buffer to handle wrap around cases.
 */
int get_new_ring_pointer(int bytes_written, int ring_pointer_offset, int file_size, int dma_byte_width)
{
	/* This function is common between wtk and WTH*/
	/*	This updates the pointer to be pointing to location
	 *		to take action on next*/

	//find new pointer and take wrap around into account
	if(align_dma(ring_pointer_offset+bytes_written, dma_byte_width)  < file_size)
		return align_dma(ring_pointer_offset + bytes_written, dma_byte_width);
	else
		return align_dma(ring_pointer_offset + bytes_written - file_size, dma_byte_width);

}

/**
 * This function returns the amount of data/space available in the ring buffer
 * file_desc The struct containing all the file variables.
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
 * file_desc The struct containing all the file variables.
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
