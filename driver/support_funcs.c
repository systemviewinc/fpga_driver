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
 //#include <stdint.h>
 #include "sv_driver.h"
 #include "xdma/sv_xdma.h"

 //debug
 #include <linux/time.h>

/******************************** Local Scope Globals ***************************************/

static struct mutex xdma_h2c_sem[XDMA_CHANNEL_NUM_MAX];
static struct mutex xdma_c2h_sem[XDMA_CHANNEL_NUM_MAX];
static int xdma_query(int);
static int cdma_query(void);
static int cdma_use_count[CDMA_MAX_NUM];
/******************************** Support functions ***************************************/

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
int dma_transfer(struct file_desc * file_desc, u64 axi_address, void *buf, size_t count, int transfer_type, u64 dma_offset)
{
	u32 max_xfer_size;
	int ret = 0, cdma_num = -1, xdma_channel = -1, attempts = 10000;
	u32 xfer_type, l_btt = count;
	u64 l_sa, l_da;


	if((transfer_type == NORMAL_READ) || (transfer_type == KEYHOLE_READ)) {
		l_sa = axi_address;				//source address is axi_address when we read
		l_da = file_desc->svd->axi_pcie_m + file_desc->dma_offset_read + dma_offset; //the AXI address written to the CDMA

		xfer_type = (HOST_WRITE|(transfer_type == KEYHOLE_READ ? INC_DA : INC_BOTH));		//DMA writes to the host when we read from the card --fix
	}
	else if((transfer_type == NORMAL_WRITE) || (transfer_type == KEYHOLE_WRITE)) {
		l_sa = file_desc->svd->axi_pcie_m + file_desc->dma_offset_write + dma_offset; //the AXI address written to the CDMA
		l_da = axi_address;				//destination address is axi_address when we write

		xfer_type = (HOST_READ|(transfer_type == KEYHOLE_WRITE ? INC_SA : INC_BOTH));		//DMA reads from the host when we write from the card --fix
	}
	else{
		verbose_printk(KERN_INFO"\t\t[dma_transfer]: !!!!!!!!ERROR: unknown transfer type.\n");
		return ERROR;
	}

	if(file_desc->svd->cdma_capable > 0){
		//----------------------------------------------------------------------
		//--------------------------do a CDMA Transfer--------------------------
		//----------------------------------------------------------------------
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
				wait_event_interruptible_timeout(file_desc->svd->thread_q_head_write,1,msecs_to_jiffies(10));
			else
				wait_event_interruptible_timeout(file_desc->svd->thread_q_head_read,1,msecs_to_jiffies(10));
			attempts = 1000;
			goto cdma_retry;
		}
		// if write check required and dma max write is set
		if((xfer_type & HOST_WRITE) && file_desc->svd->dma_max_write_size && l_btt > file_desc->svd->dma_max_write_size) {
			max_xfer_size = file_desc->svd->dma_max_write_size;
		} else if((xfer_type & HOST_READ) && file_desc->svd->dma_max_read_size && l_btt > file_desc->svd->dma_max_read_size) {
			max_xfer_size = file_desc->svd->dma_max_read_size;
		} else {
			max_xfer_size = l_btt;
		}
		do {
			int xfer_size = l_btt > max_xfer_size ? max_xfer_size : l_btt;
			verbose_dma_printk(KERN_INFO"\t\t[dma_transfer]: l_sa 0x%p l_da 0x%p xfer_size %d\n", (void*)l_sa, (void*)l_da, xfer_size);
			ret = cdma_transfer(file_desc, l_sa, l_da, xfer_size, transfer_type, cdma_num);
			if(ret != 0) return ret;
			if(xfer_type & INC_SA) l_sa += xfer_size;
			if(xfer_type & INC_DA) l_da += xfer_size;
			l_btt -= xfer_size;
		} while (l_btt);
		/*Release Mutex on CDMA*/

		verbose_dma_printk(KERN_INFO"\t\t\t[dma_transfer]: Releasing Mutex on CDMA %d \n", cdma_num);
		mutex_unlock(&svd_global->cdma_sem[cdma_num]);

		return ret;
		//----------------------------------------------------------------------
		//--------------------------CDMA Transfer done--------------------------
		//----------------------------------------------------------------------
	}
	else if(pcie_use_xdma) {
		//----------------------------------------------------------------------
		//--------------------------do a XDMA Transfer--------------------------
		//----------------------------------------------------------------------
		loff_t pos = axi_address;
		int rc = 0;

		verbose_dma_printk(KERN_INFO"\t\t[dma_transfer]: Using XDMA \n");

		if(xfer_type & HOST_READ) { // host to card AKA WRITE
			while(1){
				while(xdma_channel == -1 && --attempts != 0) {
					xdma_channel = xdma_query(DMA_TO_DEVICE);
					if(xdma_channel == -1) schedule();
				}
				if(xdma_channel == -1){
					printk(KERN_INFO"\t\t[dma_transfer]: Could not get xdma (hr), sleep 1 ms. \n");
					wait_event_interruptible_timeout(file_desc->svd->thread_q_head_read,0,msecs_to_jiffies(1));
					attempts = 10000;
				}
				else{
					break;
				}
			}
			//
			// //if keyhole read/write set address mode to non_incr_mode
			// if(transfer_type == KEYHOLE_WRITE) || (transfer_type == KEYHOLE_READ) {
			//	 if( sv_do_addrmode_set(file_desc->xdma_dev->sgdma_char_dev[xdma_channel][0]->engine, 1) ) {
			//		 printk(KERN_INFO"\t\t[dma_transfer]: ERROR xdma failed to set address mode\n");
			//		 return ERROR;
			//	 }
			//
			//	 if(l_btt < (int)file_desc->xdma_dev->sgdma_char_dev[xdma_channel][0]->engine->len_granularity){
			//		 verbose_dma_printk(KERN_INFO"\t\t[dma_transfer]: setting BTT to len_granularity\n");
			//		 l_btt = (int)file_desc->xdma_dev->sgdma_char_dev[xdma_channel][0]->engine->len_granularity;
			//
			//	 }
			//
			// }
			// //else set it to normal (incrementing) mode
			// else {
			//	 if( sv_do_addrmode_set(file_desc->xdma_dev->sgdma_char_dev[xdma_channel][0]->engine, 0) ) {
			//		 printk(KERN_INFO"\t\t[dma_transfer]: ERROR xdma failed to se address mode\n");
			//		 return ERROR;
			//	 }
			// }

			verbose_dma_printk(KERN_INFO"\t\t[dma_transfer]: xdma xfer write address 0x%p offset 0x%x\n", buf+dma_offset, (u32)pos);
			rc = sv_char_sgdma_read_write(file_desc, file_desc->xdma_dev->sgdma_char_dev[xdma_channel][0], buf+dma_offset, l_btt, &pos, 1);

			if(rc == l_btt) {	//successfully transfered all bytes
				verbose_dma_printk(KERN_INFO"\t\t[dma_transfer]: unlock h2c %i\n", xdma_channel);
				mutex_unlock(&xdma_h2c_sem[xdma_channel]);
				return 0;
			}
			//else we did not transfer the l_btt amount of data
			printk(KERN_INFO"\t\t[dma_transfer]: ERROR xdma transfed 0x%x bytes, should have transfered 0x%x bytes \n",rc, l_btt);
			mutex_unlock(&xdma_h2c_sem[xdma_channel]);
			return ERROR;


		} else if(xfer_type & HOST_WRITE) { // card to host AKA READ
			while(1){
				while(xdma_channel == -1 && --attempts != 0) {
					xdma_channel = xdma_query(DMA_FROM_DEVICE);
					if(xdma_channel == -1) schedule();
				}
				if(xdma_channel == -1){
					printk(KERN_INFO"\t\t[dma_transfer]: Could not get xdma (hw), sleep 1 ms. \n");
					wait_event_interruptible_timeout(file_desc->svd->thread_q_head_write,0,msecs_to_jiffies(1));
					attempts = 10000;
				}
				else{
					break;
				}
			}

			// if( sv_do_addrmode_set(file_desc->xdma_dev->sgdma_char_dev[xdma_channel][1]->engine, 1) ) {
			//	 printk(KERN_INFO"\t\t[dma_transfer]: ERROR xdma failed to se address mode\n");
			//	 return ERROR;
			// }
			//
			verbose_dma_printk(KERN_INFO"\t\t[dma_transfer]: xdma xfer write address 0x%p offset 0x%x\n", buf+dma_offset, (u32)pos);
			rc = sv_char_sgdma_read_write(file_desc, file_desc->xdma_dev->sgdma_char_dev[xdma_channel][1], buf+dma_offset, l_btt, &pos, 0);
			//
			// if( sv_do_addrmode_set(file_desc->xdma_dev->sgdma_char_dev[xdma_channel][1]->engine, 0) ) {
			//	 printk(KERN_INFO"\t\t[dma_transfer]: ERROR xdma failed to se address mode\n");
			//	 return ERROR;
			// }

			if(rc == l_btt) {	//successfully transfered all bytes
				verbose_dma_printk(KERN_INFO"\t\t[dma_transfer]: unlock c2h %i\n", xdma_channel);
				mutex_unlock(&xdma_c2h_sem[xdma_channel]);
				return 0;
			}
			//else we did not transfer the l_btt amount of data
			printk(KERN_INFO"\t\t[dma_transfer]: ERROR xdma transfed 0x%x bytes, should have transfered 0x%x bytes \n",rc, l_btt);
			verbose_dma_printk(KERN_INFO"\t\t[dma_transfer]: unlock c2h %i\n", xdma_channel);
			mutex_unlock(&xdma_c2h_sem[xdma_channel]);
			return ERROR;
		}
		if (ret < 0) {
			mutex_unlock(&xdma_h2c_sem[xdma_channel]);
			return ret;
		}
		return rc;
		//----------------------------------------------------------------------
		//--------------------------XDMA Transfer done--------------------------
		//----------------------------------------------------------------------
	}
	else{
		verbose_printk(KERN_INFO"\t\t[dma_transfer]: !!!!!!!!ERROR: unknown transfer type.\n");
		return ERROR;
	}
}


/*
 *
 *
 *
 */
struct task_struct* create_thread_write(struct sv_mod_dev * svd) {

	struct task_struct * kthread_heap;
	kthread_heap = kthread_create(write_thread, svd, "vsi_write_thread");

	if((kthread_heap)) {
		printk(KERN_INFO"[Create_thread_write]: Write Thread Created\n");
		wake_up_process(kthread_heap);
	}
	get_task_struct(kthread_heap);		//hold struct incase the thread ends early
	return kthread_heap;
}

struct task_struct* create_thread_read(struct sv_mod_dev * svd) {

	struct task_struct * kthread_heap;
	kthread_heap = kthread_create(read_thread, svd, "vsi_read_thread");

	if((kthread_heap)) {
		printk(KERN_INFO"[Create_thread_read]: Read Thread Created\n");
		wake_up_process(kthread_heap);
	}
	get_task_struct(kthread_heap);		//hold struct incase the thread ends early
	return kthread_heap;
}

/**
 * This function allocates the DMA regions for the peripheral.
 * file_desc the struct containing all the file variables.
 * dma_buffer_base the current DMA allocated region offset to be assigned
 * dma_buffer_size The size of the DMA buffer.
 */
int dma_file_init(struct file_desc *file_desc, char *dma_buffer_base, int xdma) {
	int size;
	unsigned long offset = 0;

	//in case we have streaming make the DMA big enough to contain everything we need
	if(file_desc->mode == AXI_STREAM_FIFO || file_desc->mode == AXI_STREAM_PACKET){
	file_desc->dma_size = size = (size_t)PAGE_ALIGN(file_desc->dma_size + 2 * sizeof(size_t) + dma_byte_width);
	file_desc->max_dma_read_write = file_desc->dma_size - 2 * sizeof(size_t) - dma_byte_width;		//the max you can read/write in stream mode is the dma size - 2 headers (sizeof size_t)
															//use this in pci read/write to determine if we can read/write and resize
	}
	else{
	file_desc->dma_size = size = (size_t)PAGE_ALIGN(file_desc->dma_size);
	file_desc->max_dma_read_write = file_desc->dma_size;
	}
	printk(KERN_INFO"[dma_%x_init]: Setting Peripheral DMA size:0x%zx\n", file_desc->minor, file_desc->dma_size);
	printk(KERN_INFO"[dma_%x_init]: Max DMA read/write size:0x%zx\n", file_desc->minor, file_desc->max_dma_read_write);

	//for xdma buffers also initialize a vmalloc_32 for the sg
	if(xdma) {
		if((file_desc->f_flags & O_ACCMODE) == O_RDONLY){
			verbose_printk(KERN_INFO"[dma_%x_init]: vmalloc for xdma read buffer.\n", file_desc->minor);
			file_desc->read_buffer = vmalloc_32(file_desc->dma_size);
			if (!file_desc->read_buffer) {
				printk(KERN_INFO"[dma_%x_init]: ERROR vmalloc for xdma read failed.\n", file_desc->minor);
				return ERROR;
			}

			while (size > 0) {
				SetPageReserved(vmalloc_to_page((void *)file_desc->read_buffer + offset));
				offset += PAGE_SIZE;
				size -= PAGE_SIZE;
			}

			verbose_printk(KERN_INFO"[dma_%x_init]: XDMA read address set to:0x%p\n", file_desc->minor, file_desc->read_buffer);
			verbose_printk(KERN_INFO"[dma_%x_init]: XDMA write address set to:0x%p\n", file_desc->minor, file_desc->write_buffer);
		}
		else if((file_desc->f_flags & O_ACCMODE) == O_WRONLY){
			verbose_printk(KERN_INFO"[dma_%x_init]: vmalloc for xdma write buffer.\n", file_desc->minor);
			file_desc->write_buffer = vmalloc_32(file_desc->dma_size);
			if (!file_desc->write_buffer){
				printk(KERN_INFO"[dma_%x_init]: ERROR vmalloc for xdma write failed.\n", file_desc->minor);
				return ERROR;
			}

			while (size > 0) {
				SetPageReserved(vmalloc_to_page((void *)file_desc->write_buffer + offset));
				offset += PAGE_SIZE;
				size -= PAGE_SIZE;
			}

			verbose_printk(KERN_INFO"[dma_%x_init]: XDMA read address set to:0x%p\n", file_desc->minor, file_desc->read_buffer);
			verbose_printk(KERN_INFO"[dma_%x_init]: XDMA write address set to:0x%p\n", file_desc->minor, file_desc->write_buffer);
		}
		else if((file_desc->f_flags & O_ACCMODE) == O_RDWR){

			verbose_printk(KERN_INFO"[dma_%x_init]: vmalloc for xdma read and write buffer.\n", file_desc->minor);
			file_desc->read_buffer = vmalloc_32(file_desc->dma_size);
			if (!file_desc->read_buffer) {
				printk(KERN_INFO"[dma_%x_init]: ERROR vmalloc for xdma read failed.\n", file_desc->minor);
				return ERROR;
			}
			file_desc->write_buffer = vmalloc_32(file_desc->dma_size);
			if (!file_desc->write_buffer){
				printk(KERN_INFO"[dma_%x_init]: ERROR vmalloc for xdma write failed.\n", file_desc->minor);
				return ERROR;
			}

			while (size > 0) {
				SetPageReserved(vmalloc_to_page((void *)file_desc->read_buffer + offset));
				SetPageReserved(vmalloc_to_page((void *)file_desc->write_buffer + offset));
				offset += PAGE_SIZE;
				size -= PAGE_SIZE;
			}

			verbose_printk(KERN_INFO"[dma_%x_init]: XDMA read address set to:0x%p\n", file_desc->minor, file_desc->read_buffer);
			verbose_printk(KERN_INFO"[dma_%x_init]: XDMA write address set to:0x%p\n", file_desc->minor, file_desc->write_buffer);
		}
		else{
			printk(KERN_INFO"[dma_%x_init]: ERROR unknown flags\n", file_desc->minor);
			return ERROR;
		}
	}

	if((file_desc->f_flags & O_ACCMODE) == O_RDONLY){
		verbose_printk(KERN_INFO"[dma_%x_init]: The current system memory dma offset:0x%x\n", file_desc->minor, file_desc->svd->dma_current_offset);
		file_desc->dma_offset_read = file_desc->svd->dma_current_offset;				//set the dma start address for the peripheral read
		file_desc->dma_read_addr = dma_buffer_base + file_desc->dma_offset_read;				//actual pointer to kernel buffer
		verbose_printk(KERN_INFO"[dma_%x_init]: DMA kernel write address set to:0x%p\n", file_desc->minor, file_desc->dma_write_addr);
		file_desc->svd->dma_current_offset += (u32)file_desc->dma_size;				//update the current dma allocation pointer, 2 buffers (R/W)
	}
	else if((file_desc->f_flags & O_ACCMODE) == O_WRONLY){
		verbose_printk(KERN_INFO"[dma_%x_init]: The current system memory dma offset:0x%x\n", file_desc->minor, file_desc->svd->dma_current_offset);
		file_desc->dma_offset_write = file_desc->svd->dma_current_offset + file_desc->dma_size; //set the dma start address for the peripheral write
		file_desc->dma_write_addr = dma_buffer_base + file_desc->dma_offset_write;				//actual pointer to kernel buffer
		verbose_printk(KERN_INFO"[dma_%x_init]: DMA kernel write address set to:0x%p\n", file_desc->minor, file_desc->dma_write_addr);
		file_desc->svd->dma_current_offset += (u32)file_desc->dma_size;				//update the current dma allocation pointer, 2 buffers (R/W)
	}
	else if((file_desc->f_flags & O_ACCMODE) == O_RDWR){
		verbose_printk(KERN_INFO"[dma_%x_init]: The current system memory dma offset:0x%x\n", file_desc->minor, file_desc->svd->dma_current_offset);
		file_desc->dma_offset_read = file_desc->svd->dma_current_offset;				//set the dma start address for the peripheral read
		file_desc->dma_offset_write = file_desc->svd->dma_current_offset + file_desc->dma_size; //set the dma start address for the peripheral write
		file_desc->dma_read_addr = dma_buffer_base + file_desc->dma_offset_read;				//actual pointer to kernel buffer
		file_desc->dma_write_addr = dma_buffer_base + file_desc->dma_offset_write;				//actual pointer to kernel buffer
		verbose_printk(KERN_INFO"[dma_%x_init]: DMA kernel read address set to:0x%p\n", file_desc->minor, file_desc->dma_read_addr);
		verbose_printk(KERN_INFO"[dma_%x_init]: DMA kernel write address set to:0x%p\n", file_desc->minor, file_desc->dma_write_addr);
		file_desc->svd->dma_current_offset += (u32)(2*file_desc->dma_size);				//update the current dma allocation pointer, 2 buffers (R/W)
	}
	else{
		printk(KERN_INFO"[dma_%x_init]: ERROR unknown flags\n", file_desc->minor);
		return ERROR;
	}
	if(file_desc->svd->dma_current_offset > file_desc->svd->dma_buffer_size) { //used too much memory
		printk(KERN_INFO"[dma_%x_init]: ERROR File needs more memory than system allocated\n", file_desc->minor);
		printk(KERN_INFO"[dma_%x_init]: Total system Memory: %08llx, Memory Used: %08x\n", file_desc->minor, file_desc->svd->dma_buffer_size, file_desc->svd->dma_current_offset);
		printk(KERN_INFO"[dma_%x_init]: Increase memory allocated by the driver DMA_SYSTEM_SIZE or adjust file DMA sizes\n", file_desc->minor);
		return ERROR;
	}
	file_desc->set_dma_flag = 1;
	printk(KERN_INFO"[dma_%x_init]: Success setting peripheral DMA size %zu\n", file_desc->minor, file_desc->dma_size);
	return 0;
}


int dma_file_deinit(struct file_desc *file_desc) {
	unsigned long addr, size;

	file_desc->set_dma_flag = 0;

	if(file_desc->read_buffer) {
		verbose_printk(KERN_INFO"[dma_%x_deinit]: XDMA freeing read_buffer :0x%p\n", file_desc->minor, file_desc->read_buffer);
		size = (unsigned long)file_desc->dma_size;
		addr = (unsigned long)file_desc->read_buffer;
		while ((long) size > 0) {
			ClearPageReserved(vmalloc_to_page((void *)addr));
			addr += PAGE_SIZE;
			size -= PAGE_SIZE;
		}
		vfree(file_desc->read_buffer);

	}

	if(file_desc->write_buffer) {
		verbose_printk(KERN_INFO"[dma_%x_deinit]: XDMA freeing write_buffer :0x%p\n", file_desc->minor, file_desc->write_buffer);
		size = (unsigned long)file_desc->dma_size;
		addr = (unsigned long)file_desc->write_buffer;
		while ((long) size > 0) {
			ClearPageReserved(vmalloc_to_page((void *)addr));
			addr += PAGE_SIZE;
			size -= PAGE_SIZE;
		}
		vfree(file_desc->write_buffer);

	}

	verbose_printk(KERN_INFO"[dma_%x_deinit]: Deinit done!\n", file_desc->minor);

	return 0;
}


/**
 * @brief This function initialized the interrupt controller in the FPGA.
 * @param axi_address The 64b AXI address of the Interrupt Controller (set through insmod).
 */
 void axi_intc_init(struct sv_mod_dev *svd, uint axi_address)
{
	u32 status;
	u64 axi_dest;
	wait_queue_head_t sw_interrupt_wq;
	init_waitqueue_head(&sw_interrupt_wq);


	printk(KERN_INFO"[axi_intc_init]: Setting Interrupt Controller Axi Address to 0x%08x\n", axi_address);
	svd->axi_intc_addr = axi_address;

	/*Here we need to clear the service interrupt in the interrupt acknowledge register*/
	status = 0xFFFFFFFF;
	axi_dest = svd->axi_intc_addr + INT_CTRL_IAR;
	if( direct_write(axi_dest, (void *)&status, 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[axi_intc_init]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n");
		return;
	}


	/*Write to Interrupt Enable Register (IER)*/
	/* Write to enable all possible interrupt inputs */
	status = 0xFFFFFFFF;
	axi_dest = svd->axi_intc_addr + INT_CTRL_IER;
	if( direct_write(axi_dest, (void *)&status, 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[axi_intc_init]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n");
		return;
	}


	/*Write to the Master Enable Register (MER) */
	/* Write to enable the master IRQ enable (ME) */
	status = 0x1;
	axi_dest = svd->axi_intc_addr + INT_CTRL_MER;
	if( direct_write(axi_dest, (void *)&status, 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[axi_intc_init]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n");
		return;
	}

	/*Write to Interrupt STATUS Register (ISR)*/
	/* Write to generate a software interrupt */
	status = 0x1000;
	axi_dest = svd->axi_intc_addr + INT_CTRL_ISR;
	if( direct_write(axi_dest, (void *)&status, 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[axi_intc_init]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n");
		return;
	}


	//wait for the software intterupt
	if(wait_event_interruptible_timeout(sw_interrupt_wq, atomic_read(&svd->sw_interrupt_rx) == 1 , msecs_to_jiffies(1))) {
		printk(KERN_INFO"[axi_intc_init]: Received SW interrupt: Passed!\n");
	}
	else{
		printk(KERN_INFO"[axi_intc_init]: ERROR no SW interrupt: FAILED!\n");
	}

	/*Write to the Master Enable Register (MER) */
	/* Write to enable the hardware interrupts */
	status = 0x3;
	axi_dest = svd->axi_intc_addr + INT_CTRL_MER;
	if( direct_write(axi_dest, (void *)&status, 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[axi_intc_init]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n");
		return;
	}
	printk(KERN_INFO"[axi_intc_init]: wrote: ('0x%08x') from MER register\n", status);

	svd->interrupt_set = true;

}

/**
 * @brief This function initialized the interrupt controller in the FPGA.
 * @param axi_address The 64b AXI address of the Interrupt Controller (set through insmod).
 */
void axi_lodc_init(struct sv_mod_dev *svd, uint axi_address)
{
	u32 status;
	u64 axi_dest;

	printk(KERN_INFO"[axi_lodc_init]: Setting Load on demand Controller Axi Address to 0x%08x\n", axi_address);
	svd->axi_lodc_addr = axi_address;

	//write all 1s to the output to deactiave all regions
	status = 0xFFFFFFFF;///to-do add multiple sections
	axi_dest = svd->axi_lodc_addr + 0;
	if( direct_write(axi_dest, (void *)&status, 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[axi_lodc_init]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n");
		return;
	}
	svd->lod_set = true;
}

/**
 * This function performs calls appropriate functions for writing to the AXI Streaming FIFO.
 * count The number of bytes to write to the AXI streaming FIFO.
 * file_desc The struct containing all the file variables.
 * ring_pointer_offset The current offset of the ring pointer in memory for data to write.
 * return ERROR for error, 0 otherwise
 */
int hls_block_init(struct file_desc * file_desc)
{
	u32 slave_address = file_desc->svd->dma_addr_base + file_desc->dma_offset_read;		//address of the hw space buffer we are writing to
	printk(KERN_INFO"[hls_block_init]: Setting HLS block slave address to to 0x%08x\n", slave_address);
	if(direct_write(file_desc->axi_addr_ctl + HLS_SLAVE_ADDR, (void *)&slave_address, 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[hls_block_init]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n");
	return ERROR;
	}
	return 0;
}




void axi_lodc_activate(struct file_desc * file_desc)
{
	u32 status;
	u64 axi_dest;

	printk(KERN_INFO"[axi_lodc_activate]: Setting Load on demand Controller to activate\n");
	axi_dest = svd_global->axi_lodc_addr + 0;
	//write all 1s to the output to deactiave all regions
	if( direct_read(axi_dest, (void *)&status, 4, NORMAL_READ) ) {
		printk(KERN_INFO"[axi_lodc_activate]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n");
		return;
	}

	printk(KERN_INFO"[axi_lodc_activate]: \tCurrent decoupler controller status: 0x%08X\n", status);
	status &= !(file_desc->decoupler_vec);
	printk(KERN_INFO"[axi_lodc_activate]: \tTry to write new decoupler status: 0x%08X\n", status);
	if( direct_write(axi_dest, (void *)&status, 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[axi_lodc_activate]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n");
		return;
	}
	file_desc->file_activate = true;
}

void axi_lodc_deactivate(struct file_desc * file_desc)
{
	u32 status;
	u64 axi_dest;

	printk(KERN_INFO"[axi_lodc_deactivate]: Setting Load on demand Controller to deactivate\n");
	axi_dest = svd_global->axi_lodc_addr + 0;
	//write all 1s to the output to deactiave all regions
	if( direct_read(axi_dest, (void *)&status, 4, NORMAL_READ) ) {
		printk(KERN_INFO"[axi_lodc_deactivate]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n");
		return;
	}

	printk(KERN_INFO"[axi_lodc_deactivate]: \tCurrent decoupler controller status: 0x%08X\n", status);
	status |= file_desc->decoupler_vec;
	printk(KERN_INFO"[axi_lodc_deactivate]: \tTry to write new decoupler status: 0x%08X\n", status);
	if( direct_write(axi_dest, (void *)&status, 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[axi_lodc_deactivate]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n");
		return;
	}
	file_desc->file_activate = false;
}


void axi_intc_deinit(struct sv_mod_dev *svd)
{
	u32 status;
	u64 axi_dest;

	printk(KERN_INFO"[axi_intc_deinit]: Disabling Interrupt Controller Axi Address to 0x%08x\n", svd->axi_intc_addr);
	axi_dest = svd->axi_intc_addr + INT_CTRL_ISR;

	if( direct_read(axi_dest, (void *)&status, 4, NORMAL_READ) ) {
		printk(KERN_INFO"[axi_intc_deinit]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n");
		return;
	}
	printk(KERN_INFO"[axi_intc_deinit]: read: ('0x%08x') from ISR register\n", status);

	/*Write to the Master Enable Register (MER) */
	/* Write to enable the hardware interrupts */
	status = 0x0;
	axi_dest = svd->axi_intc_addr + INT_CTRL_MER;
	if( direct_write(axi_dest, (void *)&status, 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[axi_intc_deinit]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n");
		return;
	}

	printk(KERN_INFO"[axi_intc_deinit]: wrote: ('0x%08x') from MER register\n", status);
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

	if(svd_global->cdma_set[0] == 1) {
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
int xdma_init_sv(struct xdma_dev *lro)
{
	int i;
	for (i = 0 ; i < XDMA_CHANNEL_NUM_MAX; i++) {
		if(lro->sgdma_char_dev[i][0] != NULL){
			printk(KERN_INFO"[xdma_init_sv]:Found h2c engine %i\n", i);
			mutex_init(&xdma_h2c_sem[i]);
			svd_global->xdma_h2c_num_channels++;
		}
		else{
			printk(KERN_INFO"[xdma_init_sv]:Did not find h2c engine %i\n", i);
		}
		if(lro->sgdma_char_dev[i][1] != NULL){
			printk(KERN_INFO"[xdma_init_sv]:Found c2h engine %i\n", i);
			mutex_init(&xdma_c2h_sem[i]);
			svd_global->xdma_c2h_num_channels++;
		}
		else {
			printk(KERN_INFO"[xdma_init_sv]:Did not find c2h engine %i\n", i);
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
int cdma_init(struct sv_mod_dev *svd, int cdma_num, uint cdma_addr)
{
	u64 axi_dest;
	u32 cdma_status;

	if( (cdma_num < 0) | (cdma_num > CDMA_MAX_NUM) ) {
		printk(KERN_INFO"\t\t[cdma_0x%x_init]: \t!!!!!!!!ERROR: incorrect CDMA number detected!!!!!!!\n", cdma_num);
		return ERROR;
	}

	cdma_address[cdma_num] = cdma_addr;
	mutex_init(&svd->cdma_sem[cdma_num]);

	verbose_cdma_printk(KERN_INFO"\t\t[cdma_0x%x_init]: *******************Setting CDMA AXI Address:%x ******************************************\n", cdma_num, cdma_address[cdma_num]);


	//wait for reset to finish	axi_dest = cdma_addr + CDMA_CR;
	axi_dest = cdma_addr + CDMA_CR;
	if( direct_read(axi_dest, (void *)&cdma_status, 4, NORMAL_READ) ) {
		printk(KERN_INFO"\t\t[cdma_idle_%x_init]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n", cdma_num);
		return ERROR;
	}
	verbose_cdma_printk(KERN_INFO"\t\t[cdma_0x%x_init]: CDMA configuration before reset: ('0x%08x')\n", cdma_num, cdma_status);


	/*Issue a Soft Reset*/
	axi_dest = cdma_addr + CDMA_CR;
	cdma_status = 0x00000004;
	verbose_cdma_printk(KERN_INFO"\t\t[cdma_0x%x_init]: sending a soft reset to the CDMA\n", cdma_num);
	if( direct_write(axi_dest, (void *)&cdma_status, 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"\t\t[cdma_idle_%x_init]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n", cdma_num);
		return ERROR;
	}

	//wait for reset to finish
	if( direct_read(axi_dest, (void *)&cdma_status, 4, NORMAL_READ) ) {
		printk(KERN_INFO"\t\t[cdma_idle_%x_init]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n", cdma_num);
		return ERROR;
	}
	verbose_cdma_printk(KERN_INFO"\t\t[cdma_0x%x_init]: CDMA configuration after reset: ('0x%08x')\n", cdma_num, cdma_status);



	/*Check the current status*/
	axi_dest = cdma_addr + CDMA_SR;
	if( direct_read(axi_dest, (void *)&cdma_status, 4, NORMAL_READ) ) {
		printk(KERN_INFO"\t\t[cdma_idle_%x_init]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n", cdma_num);
		return ERROR;
	}

	verbose_cdma_printk(KERN_INFO"\t\t[cdma_0x%x_init]: CDMA status before configuring: ('0x%08x')\n", cdma_num, cdma_status);

	/*Check the current config*/
	axi_dest = cdma_addr + CDMA_CR;
	if( direct_read(axi_dest, (void *)&cdma_status, 4, NORMAL_READ) ) {
		printk(KERN_INFO"\t\t[cdma_idle_%x_init]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n", cdma_num);
		return ERROR;
	}
	verbose_cdma_printk(KERN_INFO"\t\t[cdma_0x%x_init]: CDMA config before configuring: ('0x%08x')\n", cdma_num, cdma_status);

	/*clear any pre existing interrupt*/
	axi_dest = cdma_addr + CDMA_SR;
	cdma_status = 0x00001000;
	verbose_cdma_printk(KERN_INFO"\t\t[cdma_0x%x_init]: attempting to write: ('0x%08x') to cdma status reg\n", cdma_num, cdma_status);
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
	verbose_cdma_printk(KERN_INFO"\t\t[cdma_0x%x_init]: CDMA status after configuring: ('0x%08x')\n", cdma_num, cdma_status);

	/*Check the current status*/
	axi_dest = cdma_addr + CDMA_SR;
	if( direct_read(axi_dest, (void *)&cdma_status, 4, NORMAL_READ) ) {
		printk(KERN_INFO"\t\t[cdma_idle_%x_init]: \t!!!!!!!!ERROR: in data_transfer!!!!!!!\n", cdma_num);
		return ERROR;
	}
	verbose_cdma_printk(KERN_INFO"\t\t[cdma_0x%x_init]: CDMA status after configuring(data_transfer): ('0x%08x')\n", cdma_num, cdma_status);

	/*Check the current config*/
	axi_dest = cdma_addr + CDMA_CR;
	if( direct_read(axi_dest, (void *)&cdma_status, 4, NORMAL_READ) ) {
		printk(KERN_INFO"\t\t[cdma_idle_%x_init]: \t!!!!!!!!ERROR: in direct_read!!!!!!!\n", cdma_num);
		return ERROR;
	}
	svd->cdma_set[cdma_num] = 1;
	verbose_cdma_printk(KERN_INFO"\t\t[cdma_0x%x_init]: CDMA config after configuring: ('0x%08x')\n", cdma_num, cdma_status);
	verbose_cdma_printk(KERN_INFO"\t\t[cdma_0x%x_init]: ****************Setting CDMA AXI Address:%x Complete*************************************\n", cdma_num, cdma_addr);

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
	//u32 write_value;
	int in_range = 0;

	kern_buf = buf;

	/*determine which BAR to write to*/
	/* Also does a final check to make sure you are writing in range */
	while (in_range == 0 && i < svd_global->bars->num_bars){
		verbose_direct_write_printk(KERN_INFO"\t\t[direct_write]: pci bar %d addr is:0x%p \n", i, svd_global->bars->pci_bar_vir_addr[i]);
		verbose_direct_write_printk(KERN_INFO"\t\t[direct_write]: pci bar %d start is:0x%lx end is:%lx\n", i, svd_global->bars->pci_bar_addr[i], svd_global->bars->pci_bar_end[i]);

		if((axi_address + count) < svd_global->bars->pci_bar_end[i] && (axi_address >= svd_global->bars->pci_bar_addr[i])) {
			verbose_direct_write_printk(KERN_INFO"\t\t[direct_write]: Entering Direct write: address: 0x%llx count: ('0x%08x') \n", axi_address, (u32)count);
			verbose_direct_write_printk(KERN_INFO"\t\t[direct_write]: Direct write to BAR %d\n", i);
			virt_addr = (axi_address - svd_global->bars->pci_bar_addr[i]) + svd_global->bars->pci_bar_vir_addr[i];
			in_range = 1;
		}
		i++;
	}
	if(!in_range){
		verbose_direct_write_printk(KERN_INFO"\t\t[direct_write]: Entering Direct write: address: 0x%llx count: ('0x%08x') \n", axi_address, (u32)count);
		verbose_direct_write_printk(KERN_INFO"\t\t[direct_write]: !!!!!!!!ERROR trying to Direct write out of memory range!\n");
		return 1;
	}

	verbose_direct_write_printk(KERN_INFO"\t\t[direct_write]: writing:to 0x%llx value ('0x%08x') \n", axi_address, *(u32*)buf);

	offset = 0;
	len = count/4;
	for(i = 0; i<len; i++) {
		newaddr_src = (u32 *)(kern_buf + offset);
		//write_value = *newaddr_src;
		newaddr_dest = (u32 *)(virt_addr + offset);
		iowrite32(*newaddr_src, newaddr_dest);
		verbose_direct_write_printk(KERN_INFO"\t\t[direct_write]: wrote: ('0x%08x') to virtual address:('0x%p')\n", *newaddr_src, newaddr_dest);
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
	char __iomem* newaddr_src;
	u32 * newaddr_dest;
	int i = 0;
	int in_range = 0;

	len = count/4; //how many 32b transferis


	/*determine which BAR to read from*/
	/* Also does a final check to make sure you are writing in range */
	while (in_range == 0 && i < svd_global->bars->num_bars){
		verbose_direct_read_printk(KERN_INFO"\t\t[direct_read]: pci bar %d addr is:0x%p \n", i, svd_global->bars->pci_bar_vir_addr[i]);
		verbose_direct_read_printk(KERN_INFO"\t\t[direct_read]: pci bar %d start is:0x%lx end is:%lx\n", i, svd_global->bars->pci_bar_addr[i], svd_global->bars->pci_bar_end[i]);


		if((axi_address + count) < svd_global->bars->pci_bar_end[i] && (axi_address >= svd_global->bars->pci_bar_addr[i])) {
			verbose_direct_read_printk(KERN_INFO"\t\t[direct_read]: Entering Direct Read: address: 0x%llx count: ('0x%08x') \n", axi_address, (u32)count);
			verbose_direct_read_printk(KERN_INFO"\t\t[direct_read]: Direct reading from BAR %d\n", i);
			virt_addr = (axi_address - svd_global->bars->pci_bar_addr[i]) + svd_global->bars->pci_bar_vir_addr[i];
			in_range = 1;
		}
		i++;
	}
	if(!in_range){
		verbose_direct_read_printk(KERN_INFO"\t\t[direct_read]: Entering Direct Read: address: 0x%llx count: ('0x%08x') \n", axi_address, (u32)count);
		verbose_direct_read_printk(KERN_INFO"\t\t[direct_read]: !!!!!!!!ERROR trying to Direct read out of memory range!\n");
		return 1;
	}

	verbose_direct_read_printk(KERN_INFO"\t\t[direct_read]: reading from 0x%llx \n", axi_address);


	for(i = 0; i<len; i++) {
		newaddr_dest = buf + offset;
		newaddr_src = virt_addr + offset;
		*newaddr_dest = ioread32(newaddr_src);
		verbose_direct_read_printk(KERN_INFO"\t\t[direct_read]: read: ('0x%08x')@0x%p from kernel address 0x%p.\n", *newaddr_dest, newaddr_dest, newaddr_src);
		if(transfer_type != KEYHOLE_WRITE)
			offset += 4;
	}

	//memcpy(buf, (const void*)kern_buf, count);
	verbose_direct_read_printk(KERN_INFO"\t\t[direct_read]: Leaving Direct Read\n");

	return 0;
}


/**
 * This function will return the xdma channel that is available
 */
static int xdma_query(int direction)
{
	int i;
	if(direction == DMA_TO_DEVICE) {
		for (i = 0 ; i < svd_global->xdma_h2c_num_channels; i++)
			if(!mutex_is_locked(&xdma_h2c_sem[i])) {
				if(!mutex_trylock(&xdma_h2c_sem[i]) ){
					verbose_dmaq_printk(KERN_INFO"\t\t\t[dma_queue]: Mutex trylock failed.\n");
					return -1;
				}
				verbose_dmaq_printk(KERN_INFO"\t\t\t[dma_queue]: xdma_channel = %d \n", i);
				return i;
			}
	} else if(direction == DMA_FROM_DEVICE) {
		for (i = 0 ; i < svd_global->xdma_c2h_num_channels; i++)
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
		if( (svd_global->cdma_set[i] == 1) && (!mutex_is_locked(&svd_global->cdma_sem[i])) ) {
			if(!mutex_trylock(&svd_global->cdma_sem[i]) ){
				verbose_dmaq_printk(KERN_INFO"\t\t\t[cdma_query]: Mutex trylock failed on %d.\n", i);
				return -1;
			}
			verbose_dmaq_printk(KERN_INFO"\t\t\t[cdma_query]: cdma_channel = %d \n", i);
			cdma_use_count[i]++;
			return i;
		}
	}
	verbose_dmaq_printk(KERN_INFO "\t\t\t[cdma_query]: \t!!!!!!!!! all CDMAs in use !!!!!!!!\n");
	return -1;
}

/**
 * set_cdma_keyhole() - Enable keyhole for CDMA procedure.
 * @arg1: keyhole_en - procedure type.
 * @arg2: cdma_num - CDMA number.
 */
void set_cdma_keyhole(int keyhole_en, int cdma_num) {
	u32 bit_vec;
	switch(keyhole_en) {
		case KEYHOLE_READ:
			bit_vec = 0x00000010;	//the bit for KEYHOLE READ
			verbose_cdma_printk(KERN_INFO"\t\t[%s]: Setting the CDMA Keyhole READ as ENABLED\n", __func__);
			if( cdma_config_set(bit_vec, 1, cdma_num) ) {	//value of one means we want to SET the register
				printk(KERN_INFO"[%s]: !!!!!!!!ERROR on CDMA WRITE!!!.\n", __func__);
			}
			break;

		case KEYHOLE_WRITE:
			bit_vec = 0x00000020;	//the bit for KEYHOLE WRITE
			verbose_cdma_printk(KERN_INFO"\t\t[%s]: Setting the CDMA Keyhole WRITE as ENABLED\n", __func__);
			if( cdma_config_set(bit_vec, 1, cdma_num) ) {	//value of one means we want to SET the register
				printk(KERN_INFO"[%s]: !!!!!!!!ERROR on CDMA WRITE!!!.\n", __func__);
			}	//value of one means we want to SET the register
			break;

		default:
			verbose_cdma_printk(KERN_INFO"\t\t[%s]: no keyhole setting will be used.\n", __func__);
	}
}

/**
 * unset_cdma_keyhole() - Unset keyhole for CDMA procedure.
 * @arg1: keyhole_en - procedure type.
 * @arg2: cdma_num - CDMA number.
 */
void unset_cdma_keyhole(int keyhole_en, int cdma_num) {
	if((keyhole_en == KEYHOLE_READ) | (keyhole_en == KEYHOLE_WRITE)) {
		u32 bit_vec;
		bit_vec = 0x20 | 0x10;	// "0x30" unset both CDMA keyhole read and write
		if( cdma_config_set(bit_vec, 0, cdma_num) ) {	//unsets the keyhole configuration
			printk(KERN_INFO"[%s]: !!!!!!!!ERROR on CDMA WRITE!!!.\n", __func__);
		}
	}
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
 * Returns 0 on successful transfer
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
int cdma_transfer(struct file_desc * file_desc, u64 SA, u64 DA, u32 BTT, int keyhole_en, int cdma_num)
{
	u32 axi_dest;
	u32 SA_MSB;
	u32 SA_LSB;
	u32 DA_MSB;
	u32 DA_LSB;
	int dma_usage_cnt = file_desc->svd->dma_usage_cnt++;

	verbose_cdma_printk(KERN_INFO"\t\t[%s]: **** Starting CDMA %d transfer ****\n", __func__, dma_usage_cnt);

	SA_MSB = (u32)(SA>>32);
	SA_LSB = (u32)SA;

	DA_MSB = (u32)(DA>>32);
	DA_LSB = (u32)DA;

	//Step 1. Verify CDMASR.IDLE = 1.
	cdma_idle_poll(cdma_num);

	// Step 2. Program the CDMACR.IOC_IrqEn bit to the desired state for interrupt generation on
	// transfer completion. Also set the error interrupt enable (CDMACR.ERR_IrqEn), if so desired.
	if ( !file_desc->svd->keyhole_prohibited ) {
		set_cdma_keyhole(keyhole_en, cdma_num);
	}

	verbose_cdma_printk(KERN_INFO"\t\t[%s]: Using CDMA %d\n", __func__, cdma_num);
	verbose_cdma_printk(KERN_INFO"\t\t[%s]: ********* CDMA TRANSFER %d INITIALIZATION *************\n", __func__, dma_usage_cnt);

	// Step 3. Write the desired transfer source address to the Source Address (SA) register. The
	// transfer data at the source address must be valid and ready for transfer. If the address
	// space selected is more than 32, write the SA_MSB register also.
	axi_dest = cdma_address[cdma_num] + CDMA_SA;
	verbose_cdma_printk(KERN_INFO"\t\t[%s]: writing dma SA address ('0x%012llx') to CDMA at axi address:0x%x\n", __func__, SA, axi_dest);
	//extra debug messages
	verbose_cdma_printk(KERN_INFO"\t\t[%s]: writing dma SA_LSB address ('0x%08x') to CDMA at axi address:0x%x\n", __func__, SA_LSB, axi_dest);
	verbose_cdma_printk(KERN_INFO"\t\t[%s]: writing dma SA_MSB address ('0x%08x') to CDMA at axi address:0x%x\n", __func__, SA_MSB, axi_dest+4);

	direct_write(axi_dest , (void*)&SA_LSB, 4, NORMAL_WRITE);				//todo add error check
	direct_write(axi_dest + 4, (void*)&SA_MSB, 4, NORMAL_WRITE);			//todo add error check

	// Step 4. Write the desired transfer destination address to the Destination Address (DA) register.
	// If the address space selected is more than 32, then write the DA_MSB register also.
	axi_dest = cdma_address[cdma_num] + CDMA_DA;

	verbose_cdma_printk(KERN_INFO"\t\t[%s]: writing DA address ('0x%012llx') to CDMA at axi address:0x%x\n", __func__, DA, axi_dest);
	//extra debug messages
	verbose_cdma_printk(KERN_INFO"\t\t[%s]: writing DA_LSB address ('0x%08x') to CDMA at axi address:0x%x\n", __func__, DA_LSB, axi_dest);
	verbose_cdma_printk(KERN_INFO"\t\t[%s]: writing DA_MSB address ('0x%08x') to CDMA at axi address:0x%x\n", __func__, DA_MSB, axi_dest+4);

	direct_write(axi_dest, 	(void*)&DA_LSB, 4, NORMAL_WRITE);			//todo add error check
	direct_write(axi_dest + 4, (void*)&DA_MSB, 4, NORMAL_WRITE);			//todo add error check
	//read the status register

	// Step 5. Write the number of bytes to transfer to the CDMA Bytes to Transfer (BTT) register. Up
	// to 8, 388, 607 bytes can be specified for a single transfer (unless DataMover Lite is being
	// used). Writing to the BTT register also starts the transfer.
	axi_dest = cdma_address[cdma_num] + CDMA_BTT;
	verbose_cdma_printk(KERN_INFO"\t\t[%s]: writing bytes to transfer ('0x%x') to CDMA at axi address:0x%x\n", __func__, BTT, axi_dest);
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
			printk(KERN_INFO"[%s]: !!!!!!!!ERROR on CDMA WRITE!!!.\n", __func__);
	}

	if ( !file_desc->svd->keyhole_prohibited ) {
		unset_cdma_keyhole(keyhole_en, cdma_num);
	}

	verbose_cdma_printk(KERN_INFO"\t\t[%s]: ********* CDMA TRANSFER %d FINISHED *************\n", __func__, dma_usage_cnt);

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


/* map_bars() -- map device regions into kernel virtual address space
 *
 * Map the device memory regions into kernel virtual address space after
 * verifying their sizes respect the minimum sizes needed
 */
int sv_map_bars(struct bar_info *bars, struct pci_dev *dev)
{
	int rc;
	int i;
	int bar_id_list[XDMA_BAR_NUM];
	int bar_id_idx = 0;

	/* iterate through all the BARs */
	for (i = 0; i < XDMA_BAR_NUM; i++) {
		int bar_len;

		bar_len = sv_map_single_bar(bars, NULL, dev, i);
		if (bar_len == 0) {
			continue;
		} else if (bar_len < 0) {
			rc = -1;
			goto fail;
		}

		bar_id_list[bar_id_idx] = i;
		bar_id_idx++;
	}

	/* successfully mapped all required BAR regions */
	rc = 0;
	goto success;
fail:
	/* unwind; unmap any BARs that we did map */
	sv_unmap_bars(bars, dev);
success:
	return rc;
}

int sv_map_single_bar(struct bar_info *bars, struct xdma_dev *lro, struct pci_dev *dev, int idx)
{
	resource_size_t bar_start;
	resource_size_t bar_len;
	resource_size_t map_len;

	bar_start = pci_resource_start(dev, idx);
	bar_len = pci_resource_len(dev, idx);
	map_len = bar_len;

	bars->pci_bar_vir_addr[idx] = NULL;

	/* do not map BARs with length 0. Note that start MAY be 0! */
	if (!bar_len) {
		dbg_bar("BAR #%d is not present - skipping\n", idx);
		return 0;
	}

	/* BAR size exceeds maximum desired mapping? */
	if (bar_len > INT_MAX) {
		dbg_bar("Limit BAR %d mapping from %llu to %d bytes\n", idx, (u64)bar_len, INT_MAX);
		map_len = (resource_size_t)INT_MAX;
	}
	/*
	 * map the full device memory or IO region into kernel virtual
	 * address space
	 */
	dbg_bar("BAR%d: %llu bytes to be mapped.\n", idx, (u64)map_len);
	bars->pci_bar_vir_addr[idx] = pci_iomap(dev, idx, map_len);

	if(lro != NULL){
		lro->bar[idx] = bars->pci_bar_vir_addr[idx];
	}

	if (!bars->pci_bar_vir_addr[idx]) {
		dbg_bar("Could not map BAR %d", idx);
		return -1;
	}

	dbg_bar("BAR%d at 0x%llx mapped at 0x%p, length=%llu(/%llu)\n", idx,
		(u64)bar_start, bars->pci_bar_vir_addr[idx], (u64)map_len, (u64)bar_len);

	bars->pci_bar_end[idx] = bars->pci_bar_addr[idx]+map_len;
	bars->num_bars++;

	dbg_bar("[sv_map_single_bar]: pci bar %d addr is:0x%p \n", idx, bars->pci_bar_vir_addr[idx]);
	dbg_bar("[sv_map_single_bar]: pci bar %d start is:0x%lx end is:0x%lx\n", idx, bars->pci_bar_addr[idx], bars->pci_bar_end[idx]);
	dbg_bar("[sv_map_single_bar]: pci bar %d size is:0x%lx\n", idx, (unsigned long)map_len);

	return (int)map_len;
}

/*
 * Unmap the BAR regions that had been mapped earlier using map_bars()
 */
void sv_unmap_bars(struct bar_info *bars, struct pci_dev *dev)
{
	int i;

	for (i = 0; i < XDMA_BAR_NUM; i++) {
		/* is this BAR mapped? */
		if (bars->pci_bar_vir_addr[i]) {
			/* unmap BAR */
			pci_iounmap(dev, bars->pci_bar_vir_addr[i]);
			/* mark as unmapped */
			bars->pci_bar_vir_addr[i] = NULL;
		}
	}
}

struct sv_mod_dev *alloc_sv_dev_instance(u64 dma_size)
{
	struct sv_mod_dev *sv_dev;

	//BUG_ON(!pdev);

	/* allocate zeroed device book keeping structure */
	sv_dev = kzalloc(sizeof(struct sv_mod_dev), GFP_KERNEL);
	if (!sv_dev) {
		printk(KERN_INFO"Could not kzalloc(sv_mod_dev).\n");
		return NULL;
	}

	sv_dev->dma_addr_base = -1;
	sv_dev->dma_buffer_base = NULL;
	sv_dev->dma_current_offset = -1;
	sv_dev->dma_buffer_size = dma_size;

	sv_dev->axi_intc_addr = -1;
	sv_dev->axi_pcie_m = -1;

	sv_dev->cdma_capable = 0;
	sv_dev->dma_usage_cnt = 0;

	sv_dev->dma_max_write_size = 0;
	sv_dev->dma_max_read_size = 0;

	sv_dev->xdma_c2h_num_channels = 0;
	sv_dev->xdma_h2c_num_channels = 0;

	sv_dev->bars = NULL;
	atomic_set(&sv_dev->sw_interrupt_rx, 0);
	sv_dev->interrupt_set = false;
	sv_dev->lod_set = false;

	init_waitqueue_head(&sv_dev->thread_q_head_write);
	init_waitqueue_head(&sv_dev->thread_q_head_read);
	init_waitqueue_head(&sv_dev->pci_write_head);

	sv_dev->thread_struct_write = NULL;
	sv_dev->thread_struct_read = NULL;

	atomic_set(&sv_dev->thread_q_write, 0);
	atomic_set(&sv_dev->thread_q_read, 0);

	INIT_KFIFO(sv_dev->read_fifo);
	INIT_KFIFO(sv_dev->write_fifo);

	printk(KERN_INFO"probe() read_fifo = 0x%p\n", &sv_dev->read_fifo);
	printk(KERN_INFO"probe() write_fifo = 0x%p\n", &sv_dev->write_fifo);

	/*FIFO init stuff*/
	spin_lock_init(&sv_dev->fifo_lock_read);
	spin_lock_init(&sv_dev->fifo_lock_write);

	atomic_set(&sv_dev->driver_tx_bytes, 0); /**< Global Atomic Variable for Driver Statistics */
	atomic_set(&sv_dev->driver_rx_bytes, 0);/**< Global Atomic Variable for Driver Statistics */
	atomic_set(&sv_dev->driver_start_flag, 0);/**< Global Atomic Variable for Driver Statistics */
	atomic_set(&sv_dev->driver_stop_flag, 0);/**< Global Atomic Variable for Driver Statistics */
	//struct timespec64 driver_start_time;/**< Global Struct for Driver Statistics */
	//struct timespec64 driver_stop_time;/**< Global Struct Variable for Driver Statistics */

	printk(KERN_INFO"probe() sv_dev = 0x%p\n", sv_dev);

	return sv_dev;
}
