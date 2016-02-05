
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
#include "sv_driver.h"

/******************************** Xilinx Register Offsets **********************************/

const u32 AXI_STREAM_ISR    = 0x00;
const u32 AXI_STREAM_IER    = 0x04;
const u32 AXI_STREAM_TDFR   = 0x08;
const u32 AXI_STREAM_TDFV   = 0x0c;
const u32 AXI_STREAM_TDFD   = 0x00;
const u32 AXI_STREAM_TLR    = 0x14;
const u32 AXI_STREAM_RDFR   = 0x18;
const u32 AXI_STREAM_RDFO   = 0x1C;
const u32 AXI_STREAM_RDFD   = 0x1000;
const u32 AXI_STREAM_RLR    = 0x24;
const u32 AXI_STREAM_SRR    = 0x28;
const u32 AXI_STREAM_TDR    = 0x2C;
const u32 AXI_STREAM_RDR    = 0x30;
const u32 AXI_STREAM_TXID   = 0x34;
const u32 AXI_STREAM_TXUSER = 0x38;
const u32 AXI_STREAM_RXID   = 0x3C;

const u32 CDMA_CR           = 0x00;
const u32 CDMA_SR           = 0x04;
const u32 CDMA_DA           = 0x20;
const u32 CDMA_DA_MSB       = 0x24;
const u32 CDMA_SA           = 0x18;
const u32 CDMA_SA_MSB       = 0x1C;
const u32 CDMA_BTT          = 0x28;

const u32 AXIBAR2PCIEBAR_0L = 0x20c;
const u32 AXIBAR2PCIEBAR_1L = 0x214;

/******************************** Local Scope Globals ***************************************/

u64 axi_cdma;
u64 axi_cdma_2;

/******************************** Support functions ***************************************/
int dma_file_init(struct mod_desc *mod_desc, int dma_file_size, void *dma_buffer_base, u64 dma_buffer_size)
{

	verbose_printk(KERN_INFO"<dma_file_init>: Setting Peripheral DMA size:%d\n", dma_file_size);
	mod_desc->dma_size = (size_t)dma_file_size;

	if (((u64)dma_current_offset + dma_file_size) > (u64)((char*)dma_buffer_base + dma_buffer_size))
	{
		printk(KERN_INFO"<dma_file_init>: ERROR! DMA Buffer out of memory!\n");
		return -1;
	}
	else
	{
		verbose_printk(KERN_INFO"<dma_file_init>: The current system memory dma offset:%x\n", dma_current_offset);
		mod_desc->dma_offset_read = dma_current_offset;            //set the dma start address for the peripheral read
		mod_desc->dma_offset_write = dma_current_offset + (u32)dma_file_size; //set the dma start address for the peripheral write
		mod_desc->dma_write = (void*)((char*)dma_buffer_base + (u64)dma_current_offset + dma_file_size);            //actual pointer to kernel buffer
		verbose_printk(KERN_INFO"<dma_file_init>: DMA kernel write address set to:%llx\n", (u64)mod_desc->dma_write);
		mod_desc->dma_read = (void*)((char*)dma_buffer_base + (u64)dma_current_offset);            //actual pointer to kernel buffer

		dma_current_offset = dma_current_offset + (u32)(2*dma_file_size);            //update the current dma allocation pointer, 2 buffers (R/W)
		verbose_printk(KERN_INFO"<dma_file_init>: Success setting peripheral DMA\n");
	}
	return 0;
}

void int_ctlr_init(u64 axi_address)
{
	u32 status;
	u64 axi_dest;
	int ret;

	verbose_printk(KERN_INFO"<int_ctlr_init>: Setting Interrupt Controller Axi Address\n");
	axi_interr_ctrl = axi_address;

	/*Write to Interrupt Enable Register (IER)*/
	/* Write to enable all possible interrupt inputs */
	status = 0xFFFFFFFF;
	axi_dest = axi_interr_ctrl + INT_CTRL_IER;
	ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_WRITE, 0);

	/*Write to the Master Enable Register (MER) */
	/* Write to enable the hardware interrupts */
	status = 0x3;
	axi_dest = axi_interr_ctrl + INT_CTRL_MER;
	ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_WRITE, 0);

	//			status = ioread32((u32 *)int_ctrl_virt_addr_loc);  //binary "11"
	ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_READ, 0);
	verbose_printk(KERN_INFO"<int_ctlr_init>: read: ('%x') from MER register\n", status);

	/*Here we need to clear the service interrupt in the interrupt acknowledge register*/ 
	status = 0xFFFFFFFF;
	axi_dest = axi_interr_ctrl + INT_CTRL_IAR;
	ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_WRITE, 0);
}

void pcie_m_init(cdma_num)
{
	verbose_printk(KERN_INFO"<pcie_m_init>: Setting PCIe Master Axi Address\n");
	axi_pcie_m = pcie_m_address;
}

int pcie_ctl_init(u64 axi_address, u32 dma_addr_base)
{
	u32 dma_addr_loc;
	u64 axi_dest;
	u32 status;
	int ret;

	verbose_printk(KERN_INFO"<pcie_ctl_init>: Setting PCIe Control Axi Address\n");
	axi_pcie_ctl = axi_address;

	if(cdma_set[1] == 1)
	{
		/*convert to u32 to send to CDMA*/
		dma_addr_loc = (u32)dma_addr_base;

		//update pcie_ctl_virt address with register offset
		axi_dest = axi_pcie_ctl + AXIBAR2PCIEBAR_0L;

		//write DMA addr to PCIe CTL for address translation
		ret = data_transfer(axi_dest, (void *)(&dma_addr_loc), 4, NORMAL_WRITE, 0);
		verbose_printk(KERN_INFO"<pci_ioctl_cdma_set>: writing dma address ('%x') to pcie_ctl at AXI address:%llx\n", dma_addr_loc, axi_dest);

		//check the pcie-ctl got the translation address
		ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_READ, 0);
		verbose_printk(KERN_INFO"<pci_ioctl_cdma_set>: PCIe CTL register:%x\n", status);

		if (status == dma_addr_loc)
			verbose_printk(KERN_INFO"<pci_ioctl_cdma_set>: PCIe CTL register set SUCCESS:%x\n", status);
		else
		{
			printk(KERN_INFO"<pci_ioctl_cdma_set>:!!!!!!!!!!! PCIe CTL register FAILURE !!!!!!!!!!!!!!!:%x\n", status);
			printk(KERN_INFO"<pci_ioctl_cdma_set>: Most likely due to incorrect setting in PCIe IP for AXI to BAR translation HIGH address\n");
			printk(KERN_INFO"<pci_ioctl_cdma_set>: Must set this value to be the DMA buffer size allocation.\n");
			return -1;
				}
	}
	return 0;
}

int cdma_init(int cdma_num, int cdma_addr, u32 dma_addr_base)
{
	//	u32 axi_cdma;
	u64 axi_dest;
	u32 cdma_status;
	u32 dma_addr_loc;
	int * mode;
	int ret;
	int index;
	u64 axi_cdma_loc;

	switch(cdma_num)
	{
		case 1:
			axi_cdma = cdma_addr;
			axi_cdma_loc = axi_cdma;
			mutex_init(&CDMA_sem);
			break;
		case 2:
			axi_cdma_2 = cdma_addr;
			axi_cdma_loc = axi_cdma_2;
			mutex_init(&CDMA_sem_2);
			break;
		default:
			printk(KERN_INFO"	!!!!!!!!ERROR: incorrect CDMA number detected!!!!!!!\n");
			return -1;
	}

	verbose_printk(KERN_INFO"<cdma_%x_init>: *******************Setting CDMA AXI Address:%llx ******************************************\n", cdma_num, axi_cdma_loc);
	cdma_set[cdma_num] = 1;

	/*Issue a Soft Reset*/
	axi_dest = axi_cdma_loc + CDMA_CR;
	cdma_status = 0x00000004;
	verbose_printk(KERN_INFO"<cdma_%x_init>: sending a soft reset to the CDMA\n", cdma_num);
	ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_WRITE, 0);

	/*Check the current status*/
	axi_dest = axi_cdma_loc + CDMA_SR;
	//			direct_read(axi_dest, (void*)&cdma_status, 4, NORMAL_READ);
	ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_READ, 0);
	verbose_printk(KERN_INFO"<cdma_%x_init>: CDMA status before configuring:%x\n", cdma_num, cdma_status);	

	/*Check the current config*/
	axi_dest = axi_cdma_loc + CDMA_CR;
	//			direct_read(axi_dest, (void*)&cdma_status, 4, NORMAL_READ);
	ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_READ, 0);
	verbose_printk(KERN_INFO"<cdma_%x_init>: CDMA config before configuring:%x\n", cdma_num, cdma_status);	

	/*clear any pre existing interrupt*/
	axi_dest = axi_cdma_loc + CDMA_SR;
	cdma_status = 0x00001000;
	verbose_printk(KERN_INFO"<cdma_%x_init>: attempting to write:%x to cdma status reg\n", cdma_num, cdma_status);
	ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_WRITE, 0);

	cdma_status = 0x00001000;
	axi_dest = axi_cdma_loc + CDMA_CR;
	ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_WRITE, 0);

	/*Check the current status*/
	axi_dest = axi_cdma_loc + CDMA_SR;
	ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_READ, 0);
	verbose_printk(KERN_INFO"<cdma_%x_init>: CDMA status after configuring:%x\n", cdma_num, cdma_status);	

	/*Check the current config*/
	axi_dest = axi_cdma_loc + CDMA_CR;
	ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_READ, 0);
	verbose_printk(KERN_INFO"<cdma_%x_init>: CDMA config after configuring:%x\n", cdma_num, cdma_status);	


	/*This checks to see if the user has set the pcie_ctl base address
	 * if so, we can go ahead and write the dma_addr to the pcie translation
	 * register.*/
	if(pcie_ctl_set == 1)
	{
		/*convert to u32 to send to CDMA*/
		dma_addr_loc = (u32)dma_addr_base;
		axi_dest = axi_pcie_ctl + AXIBAR2PCIEBAR_0L;

		//write DMA addr to PCIe CTL for address translation
		ret = data_transfer(axi_dest, (void *)(&dma_addr_loc), 4, NORMAL_WRITE, 0);
		verbose_printk(KERN_INFO"<cdma_init>: writing dma address ('%x') to pcie_ctl at AXI address:%llx\n", dma_addr_loc, axi_dest);
		//check the pcie-ctl got the translation address
		ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_READ, 0);
		verbose_printk(KERN_INFO"<cdma_init>: PCIe CTL register:%x\n", cdma_status);

	}

	/*update the interr_dict with interrupt number and mode*/
	mode = (u32*)kmalloc(sizeof(int), GFP_KERNEL);
	*mode = CDMA;
	index = cdma_num - 1;
	interr_dict[index].mode = mode;

	return 0;
}

int vec2num(u32 vec)
{
	int count = 0;

	while((vec & 0x1) == 0)
	{
		count = count + 1;	
		vec = vec>>1;
	}

	return count;
}

u32 num2vec(int num)
{
	u32 vec = 1;
	vec = vec << num;

	return vec;
}


int data_transfer(u64 axi_address, void *buf, size_t count, int transfer_type, u64 dma_off)
{
	//	u32 test;
	int in_range = 0;
	int status = 0;
	int cdma_num = 0;
	u64 dma_axi_address = 0;
	//	void * dma_p;

	/*determine if the axi range is in direct accessible memory space*/
	if ((axi_address + (u64)count) < (bar_0_axi_offset + pci_bar_size))
		in_range = 1;
	else if ((axi_address + (u64)count) < (peripheral_space_offset + pci_bar_1_size))
	{
		if (axi_address >= peripheral_space_offset)
			in_range = 1;
	}

	//if  data is small or the cdma is not initialized and in range
	if (((count < 16) | (cdma_capable == 0)) & (in_range == 1))
	{
		if ((transfer_type == NORMAL_READ) | (transfer_type == KEYHOLE_READ))
			status = direct_read(axi_address, buf, count, transfer_type);
		else if ((transfer_type == NORMAL_WRITE) | (transfer_type == KEYHOLE_WRITE))
			status = direct_write(axi_address, buf, count, transfer_type);
		else	
			verbose_printk(KERN_INFO"<data_transfer>: error no transfer type specified\n");
	}

	else if (cdma_capable == 1)
	{
		/* Find an available CDMA to use and wait if both are in use */
		cdma_num = 0;
		while (cdma_num == 0)
		{
			cdma_num = cdma_query();
			if (cdma_num == 0)
			schedule();
		}

	//	if (cdma_num == 0)
	//	{
			/*default to CDMA 1 and wait on it*/
	//		if (mutex_lock_interruptible(&CDMA_sem))
	//		{
	//			verbose_printk(KERN_INFO"User interrupted while waiting for CDMA semaphore.\n");
	//			return -ERESTARTSYS;
	//		}
	//		cdma_num = 1;
			//		atomic_set(&mutex_free, 0);  //wait variable for mutex lock
			//		wait_event_interruptible(mutexq, atomic_read(&mutex_free) != 0);
	//	}

		dma_axi_address = axi_pcie_m + dma_off;  //the AXI address written to the CDMA

		if ((transfer_type == NORMAL_READ) | (transfer_type == KEYHOLE_READ))
		{
			status = cdma_transfer(axi_address, dma_axi_address, (u32)count, transfer_type, cdma_num);
			if (status != 0)   //unsuccessful CDMA transmission
				verbose_printk(KERN_INFO"ERROR on CDMA READ!!!.\n");
			//			memcpy(buf, (const void*)dma_p, count);

		}
		else if ((transfer_type == NORMAL_WRITE) | (transfer_type == KEYHOLE_WRITE))
		{
			//Transfer data from user space to kernal space at the allocated DMA region
			//		memcpy(dma_p, (const void*)buf, count);
			status = cdma_transfer(dma_axi_address, axi_address, (u32)count, transfer_type, cdma_num);
			//			test = 0xDEADBEEF;
			//			memcpy(dma_p, (const void*)&test, 4);	
		}
		else
			verbose_printk(KERN_INFO"<data_transfer>: error no transfer type specified\n");

		/*Release Mutex on CDMA*/
		switch(cdma_num)
		{
			case 1:
				verbose_printk(KERN_INFO"												<data_transfer>: Releasing Mutex on CDMA 1\n");
				mutex_unlock(&CDMA_sem);
				break;

			case 2:
				verbose_printk(KERN_INFO"												<data_transfer>: Releasing Mutex on CDMA 2\n");
				mutex_unlock(&CDMA_sem_2);
				break;
			default : verbose_printk(KERN_INFO"<data_transfer>: ERROR: unknown cdma number detected.\n");
					  return(0);
		}

		//	atomic_set(&mutex_free, 1);  //wait variable for mutex lock
		//	wake_up_interruptible(&mutexq);

	}

	else
		verbose_printk(KERN_INFO"<data_transfer>: ERROR: Address is out of range and CDMA is not initialized\n");

	return status;
}

int direct_write(u64 axi_address, void *buf, size_t count, int transfer_type)
{
	void * kern_buf;
	void * virt_addr;
	int offset;
	int len;
	u32 * newaddr_src;
	u32 * newaddr_dest;
	int i;
	u32 write_value;

	kern_buf = buf;

	/*determine which BAR to write to*/
	/* Also does a final check to make sure you are writing in range */
	if ((axi_address >= peripheral_space_offset) & ((axi_address + count) < (peripheral_space_offset + pci_bar_1_size)))
	{
		verbose_printk(KERN_INFO"		<direct_write>: Direct writing to BAR 1\n");
		verbose_printk(KERN_INFO"		<direct_write>: Direct writing to BAR 1 with axi address:%llx\n", axi_address);
		virt_addr = (axi_address - peripheral_space_offset) + pci_bar_1_vir_addr;
	}
	else if ((axi_address + count) < (bar_0_axi_offset + pci_bar_size))
	{
		virt_addr = (axi_address - bar_0_axi_offset) + pci_bar_vir_addr;
		verbose_printk(KERN_INFO"		<direct_write>: Direct writing to BAR 0\n");
	}
	else
	{
		verbose_printk(KERN_INFO"		<direct_write>: ERROR trying to Direct write out of memory range!\n");
		return 1;
	}

	verbose_printk(KERN_INFO"		<direct_write>: writing:%x \n", *(u32*)buf);

	offset = 0;
	len = count/4;
	for(i = 0; i<len; i++)
	{
		newaddr_src = (u32 *)(kern_buf + offset);
		write_value = *newaddr_src;
		newaddr_dest = (u32 *)(virt_addr + offset);
		iowrite32(write_value, newaddr_dest);
		//		*newaddr_dest = write_value;   *this works too*
		verbose_printk(KERN_INFO"		<direct_write>: wrote:%x to virtual address:('%p')\n", write_value, newaddr_dest);
		if (transfer_type != KEYHOLE_WRITE)
			offset += 4;
	}

	return 0;

}

int direct_read(u64 axi_address, void *buf, size_t count, int transfer_type)
{
	int len;
	void * virt_addr;
	int offset;
	u32 kern_buf[count/4];
	u32 * newaddr_src;
	int i;

	len = count/4;  //how many 32b transferis

	/*determine which BAR to read from*/
	/* Also does a final check to make sure you are writing in range */
	if ((axi_address >= peripheral_space_offset) & ((axi_address + count) < (peripheral_space_offset + pci_bar_1_size)))
	{
		verbose_printk(KERN_INFO"		<direct_read>: Direct reading from BAR 1 with axi address:%llx\n", axi_address);
		virt_addr = (axi_address - peripheral_space_offset) + pci_bar_1_vir_addr;
		verbose_printk(KERN_INFO"		<direct_read>: Direct reading from virtual address:%p\n", virt_addr);
	}
	else if ((axi_address + count) < (bar_0_axi_offset + pci_bar_size))
	{
		verbose_printk(KERN_INFO"		<direct_read>: Direct reading from BAR 0\n");
		virt_addr = (axi_address - bar_0_axi_offset) + pci_bar_vir_addr;
	}
	else
	{
		verbose_printk(KERN_INFO"		<direct_read>: ERROR trying to Direct read out of memory range!\n");
		return 1;
	}

	offset = 0;

	for(i = 0; i<len; i++)
	{
		newaddr_src = (u32 *)(virt_addr + offset);
		kern_buf[i] = ioread32(newaddr_src);
		//		kern_buf[i] = *newaddr_src;  *this works too*
		if (transfer_type != KEYHOLE_READ)
			offset += 4;
		verbose_printk(KERN_INFO"		<direct_read>: read: %x from kernel address %p.\n", kern_buf[i], newaddr_src);
	}

	memcpy(buf, (const void*)kern_buf, count);

	return 0;
}

int cdma_query(void)
{
	/*Check the CDMA Semaphore*/

	if (mutex_is_locked(&CDMA_sem))
	{
		if (mutex_is_locked(&CDMA_sem_2) | (cdma_set[2] == 0))
			verbose_printk(KERN_INFO "!!!!!!!!! all CDMAs in use !!!!!!!!\n");
		else
		{
			if (mutex_lock_interruptible(&CDMA_sem_2))
			{
				verbose_printk(KERN_INFO"User interrupted while waiting for CDMA semaphore.\n");
				return -ERESTARTSYS;
			}
			verbose_printk(KERN_INFO"										<cdma_transfer>: CDMA Resource 2 is now locked!\n");
			verbose_printk(KERN_INFO"										<cdma_transfer>: cdma_set[2] = %x\n", cdma_set[2]);
			return 2;
		}	
	}
	else
	{	
		if (mutex_lock_interruptible(&CDMA_sem))
		{
			verbose_printk(KERN_INFO"User interrupted while waiting for CDMA semaphore.\n");
			return -ERESTARTSYS;
		}

		verbose_printk(KERN_INFO"											<cdma_transfer>: CDMA Resource 1 is now locked!\n");
		return 1;
	}

	return 0;   //no CDMA's are available
}

int cdma_transfer(u64 SA, u64 DA, u32 BTT, int keyhole_en, int cdma_num)
{
	u32 bit_vec;
	u64 axi_dest;
	u32 status;
	u32 SA_MSB;
	u32 SA_LSB;
	u32 DA_MSB;
	u32 DA_LSB;
	int ack_status; 
	u64 axi_cdma_loc;
	int ret;

	verbose_printk(KERN_INFO"	<cdma_transfer>: **** Starting a CDMA transfer ****\n");

	SA_MSB = (u32)(SA>>32);
	SA_LSB = (u32)SA;

	DA_MSB = (u32)(DA>>32);
	DA_LSB = (u32)DA;

	switch(keyhole_en){

		case KEYHOLE_READ:
			bit_vec = 0x00000010;   //the bit for KEYHOLE READ		
			verbose_printk(KERN_INFO"	<keyhole_read_set>: Setting the CDMA Keyhole READ as ENABLED\n");
			ret = cdma_config_set(bit_vec, 1, cdma_num);   //value of one means we want to SET the register
			break;

		case KEYHOLE_WRITE:
			bit_vec = 0x00000020;   //the bit for KEYHOLE WRITE
			verbose_printk(KERN_INFO"	<keyhole_write_set>: Setting the CDMA Keyhole WRITE as ENABLED\n");
			ret = cdma_config_set(bit_vec, 1, cdma_num);   //value of one means we want to SET the register
			break;

		default:verbose_printk(KERN_INFO"	<keyhole_setting> no keyhole swtting will be used.\n");
	}

	switch(cdma_num)
	{
		case 1:
			axi_cdma_loc = axi_cdma;
			verbose_printk(KERN_INFO"	<pci_dma_transfer>:Using CDMA 1\n");	
			break;
		case 2:
			axi_cdma_loc = axi_cdma_2;
			verbose_printk(KERN_INFO"	<pci_dma_transfer>:Using CDMA 2\n");	
			break;
		default:verbose_printk(KERN_INFO"	!!!!!!!!ERROR: incorrect CDMA number detected!!!!!!!\n");
				return(0);
	}

	//read the config register
	axi_dest = axi_cdma_loc + CDMA_CR;
	direct_read(axi_dest, (void*)&status, 4, NORMAL_READ);
	verbose_printk(KERN_INFO"	<pci_dma_transfer>: CDMA Configuration before transmission:%x\n", status);	

	//Writing SA_MSB	
	//	direct_write(axi_dest, (void*)&SA_MSB, 4, NORMAL_WRITE);
	verbose_printk(KERN_INFO"	<pci_dma_transfer>: ********* CDMA TRANSFER INITIALIZATION *************\n\n");	
	//	verbose_printk(KERN_INFO"	<pci_dma_transfer>: writing dma SA_MSB address ('%x') to CDMA at axi address:%llx\n", SA_MSB, axi_dest);
	//Writing SA_LSB	
	axi_dest = axi_cdma_loc + CDMA_SA;
	direct_write(axi_dest, (void*)&SA_LSB, 4, NORMAL_WRITE);
	verbose_printk(KERN_INFO"	<pci_dma_transfer>: writing dma SA_LSB address ('%x') to CDMA at axi address:%llx\n", SA_LSB, axi_dest);
	//read the status register
	axi_dest = axi_cdma_loc + CDMA_SR;
	direct_read(axi_dest, (void*)&status, 4, NORMAL_READ);
	if (status != 0x02)
		printk(KERN_INFO"<pci_dma_transfer>: ERROR! CDMA Status after writing SA:%x\n", status);	

	//Writing DA_MSB	
	//	axi_dest = axi_cdma + CDMA_DA_MSB;
	//	direct_write(axi_dest, (void*)&DA_MSB, 4, NORMAL_WRITE);
	//	verbose_printk(KERN_INFO"	<pci_dma_transfer>: writing DA_MSB address ('%x') to CDMA at axi address:%llx\n", DA_MSB, axi_dest);
	//Writing DA_LSB
	axi_dest = axi_cdma_loc + CDMA_DA;
	direct_write(axi_dest, (void*)&DA_LSB, 4, NORMAL_WRITE);
	verbose_printk(KERN_INFO"	<pci_dma_transfer>: writing DA_LSB address ('%x') to CDMA at axi address:%llx\n", DA_LSB, axi_dest);
	//read the status register
	axi_dest = axi_cdma_loc + CDMA_SR;
	direct_read(axi_dest, (void*)&status, 4, NORMAL_READ);
	if (status != 0x02)
		printk(KERN_INFO"<pci_dma_transfer>: ERROR CDMA Status after writing DA:%x\n", status);	

	//Writing BTT	
	axi_dest = axi_cdma_loc + CDMA_BTT;
	verbose_printk(KERN_INFO"	<pci_dma_transfer>: writing bytes to transfer ('%d') to CDMA at axi address:%llx\n", BTT, axi_dest);	
	direct_write(axi_dest, (void*)&BTT, 4, NORMAL_WRITE);

	verbose_printk(KERN_INFO"	<pci_dma_transfer>: ********* CDMA TRANSFER INITIALIZED *************\n");	

	/*Go to sleep and wait for interrupt*/
	verbose_printk(KERN_INFO"	<pci_dma_transfer>: waiting on isr to set cdma num:%x cdma_comp:%x to 1\n", cdma_num, cdma_comp[cdma_num]);	
	
//	cdma_wait_sleep(cdma_num);

	cdma_idle_poll(cdma_num);

//	iret = wait_event_interruptible(wq, cdma_comp[cdma_num] != 0);
//	ret = wait_event_interruptible(wq, atomic_read(&cdma_atom[cdma_num]) != 0);
	
//	if (ret != 0)
//		printk(KERN_INFO"	<pci_dma_transfer>: WAIT EVENT FORCED FROM USER SPACE, CDMA %x Never interrupted.\n", cdma_num);

	cdma_comp[cdma_num] = 0;
//	atomic_set(&cdma_atom[cdma_num], 0);
	verbose_printk(KERN_INFO"	<pci_dma_transfer>: returned from ISR.\n");
	verbose_printk(KERN_INFO"	<pci_dma_transfer>: reset the cdma num:%x wait variable cdma_comp:%x to 0\n", cdma_num, cdma_comp[cdma_num]);	

	// Acknowledge the CDMA and check for error status
	ack_status = cdma_ack(cdma_num);

	if ((keyhole_en == KEYHOLE_READ) | (keyhole_en == KEYHOLE_WRITE))
	{
		bit_vec = 0x20 | 0x10;      // "0x30" unset both CDMA keyhole read and write	
		ret = cdma_config_set(bit_vec, 0, cdma_num);	//unsets the keyhole configuration
	}

	verbose_printk(KERN_INFO"	<pci_dma_transfer>: ********* CDMA TRANSFER FINISHED *************\n");	

	return ack_status;
}

void cdma_wait_sleep(int cdma_num)
{
	int ret;

	ret = wait_event_interruptible(wq, cdma_comp[cdma_num] != 0);
//	ret = wait_event_interruptible(wq, atomic_read(&cdma_atom[cdma_num]) != 0);
	
	if (ret != 0)
		printk(KERN_INFO"	<pci_dma_transfer>: WAIT EVENT FORCED FROM USER SPACE, CDMA %x Never interrupted.\n", cdma_num);

}

void cdma_idle_poll(int cdma_num)
{
	u32 cdma_status;
	u32 status;
	u64 axi_dest;
	int ret;
	u64 axi_cdma_loc;

	switch(cdma_num)
	{
		case 1:
			axi_cdma_loc = axi_cdma;
			break;
		case 2:
			axi_cdma_loc = axi_cdma_2;
			break;
		default:
			verbose_printk(KERN_INFO"	!!!!!!!!ERROR: incorrect CDMA number detected!!!!!!!\n");
			return 1;	
	}

	status = 0x0;
	axi_dest = axi_cdma_loc + CDMA_SR;

	while((status & 0x2) != 0x2)  //this means while CDMA is NOT idle
	{
	/* Check the status of the CDMA to see if successful */
	ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_READ, 0);
	verbose_printk(KERN_INFO"	<cdma_ack>: CDMA status:%x\n", status);
	}
}

int cdma_config_set(u32 bit_vec, int set_unset, int cdma_num)
{
	u32 current_CR;
	u32 new_CR;
	u32 set_vec;
	u64 axi_dest;
	int status;
	u64 axi_cdma_loc;

	switch(cdma_num)
	{
		case 1:
			axi_cdma_loc = axi_cdma;
			break;
		case 2:
			axi_cdma_loc = axi_cdma_2;
			break;
		default:
			printk(KERN_INFO"	!!!!!!!!ERROR: incorrect CDMA number detected!!!!!!!\n");
			return -1;
	}

	axi_dest = axi_cdma_loc + CDMA_CR;
	status = data_transfer(axi_dest, (void *)&current_CR, 4, NORMAL_READ, 0);

	if (set_unset == 1)   //we are setting the CR
	{
		verbose_printk(KERN_INFO"<cdma_config_set>: setting the keyhole config on CDMA.\n");
		set_vec = (current_CR | bit_vec);
		status = data_transfer(axi_dest, (void *)&set_vec, 4, NORMAL_WRITE, 0);
	}
	else    //We are unsetting bits in the CR
	{
		set_vec = (current_CR & (~bit_vec));
		verbose_printk(KERN_INFO"<cdma_config_set>: UNsetting the keyhole config on CDMA.\n");
		status = data_transfer(axi_dest, (void *)&set_vec, 4, NORMAL_WRITE, 0);
	}

	status = data_transfer(axi_dest, (void *)&new_CR, 4, NORMAL_READ, 0);
	verbose_printk(KERN_INFO"<pci_cdma_config_set>: new CDMA Config Register is:%x\n", new_CR);	
	return 0;
}

int cdma_ack(cdma_num)
{
	u32 cdma_status;
	u32 status;
	u64 axi_dest;
	int ret;
	u64 axi_cdma_loc;

	switch(cdma_num)
	{
		case 1:
			axi_cdma_loc = axi_cdma;
			break;
		case 2:
			axi_cdma_loc = axi_cdma_2;
			break;
		default:
			verbose_printk(KERN_INFO"	!!!!!!!!ERROR: incorrect CDMA number detected!!!!!!!\n");
			return 1;	
	}

	/*acknowledge the CDMA interrupt (to reset it)*/
	cdma_status = 0x00001000;
	axi_dest = axi_cdma_loc + CDMA_SR;
	ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_WRITE, 0);
	verbose_printk(KERN_INFO"	<cdma_ack>: writing SR config ('%x') to CDMA at AXI address:%llx\n", cdma_status, axi_dest);

	/* Check the status of the CDMA to see if successful */
	ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_READ, 0);
	verbose_printk(KERN_INFO"	<cdma_ack>: CDMA status:%x\n", status);
	if (status != 0x2)    //this is the expected status report  
	{
		printk(KERN_INFO"	<cdma_ack>: CDMA %d status ERROR\n", cdma_num);
		printk(KERN_INFO"	<cdma_ack>: CDMA %d status:%x\n", cdma_num, status);
		verbose_printk(KERN_INFO"	<cdma_ack>: Issuing soft reset to CDMA %d\n", cdma_num);

		axi_dest = axi_cdma_loc;

		/*Issue a Soft Reset*/
		axi_dest = axi_cdma_loc + CDMA_CR;
		cdma_status = 0x00000004;
		verbose_printk(KERN_INFO"<cdma_%x_init>: sending a soft reset to the CDMA\n", cdma_num);
		ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_WRITE, 0);
		/*Check the current status*/
		axi_dest = axi_cdma_loc + CDMA_SR;
		//			direct_read(axi_dest, (void*)&cdma_status, 4, NORMAL_READ);
		ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_READ, 0);
		verbose_printk(KERN_INFO"<cdma_%x_init>: CDMA status before configuring:%x\n", cdma_num, cdma_status);	
		/*Check the current config*/
		axi_dest = axi_cdma_loc + CDMA_CR;
		//			direct_read(axi_dest, (void*)&cdma_status, 4, NORMAL_READ);
		ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_READ, 0);
		verbose_printk(KERN_INFO"<cdma_%x_init>: CDMA config before configuring:%x\n", cdma_num, cdma_status);	
		/*clear any pre existing interrupt*/
		axi_dest = axi_cdma_loc + CDMA_SR;
		cdma_status = 0x00001000;
		verbose_printk(KERN_INFO"<cdma_%x_init>: attempting to write:%x to cdma status reg\n", cdma_num, cdma_status);
		ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_WRITE, 0);

		cdma_status = 0x00001000;
		axi_dest = axi_cdma_loc + CDMA_CR;
		ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_WRITE, 0);

		/*Check the current status*/
		axi_dest = axi_cdma_loc + CDMA_SR;
		ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_READ, 0);
		verbose_printk(KERN_INFO"<cdma_%x_init>: CDMA status after configuring:%x\n", cdma_num, cdma_status);	
		/*Check the current config*/
		axi_dest = axi_cdma_loc + CDMA_CR;
		ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_READ, 0);
		verbose_printk(KERN_INFO"<cdma_%x_init>: CDMA config after configuring:%x\n", cdma_num, cdma_status);	

		return status;
	}
	else {
		return 0;  // successful CDMA transmission
	}
}

size_t axi_stream_fifo_write(size_t count, struct mod_desc * mod_desc)
{
	u64 axi_dest;
	u32 init_write;
	u64 dma_offset_write;
	u64 dma_offset_internal_read;
	u64 dma_offset_internal_write;
	int ret;
	int keyhole_en;

	verbose_printk(KERN_INFO"<axi_stream_fifo_write>: writing to the AXI Stream FIFO\n");
	verbose_printk(KERN_INFO"<axi_stream_fifo_write>: number of bytes to write:%zu\n", count);

	init_write = *((u32*)(mod_desc->dma_write));
	dma_offset_write = (u64)mod_desc->dma_offset_write;
	dma_offset_internal_read = (u64)mod_desc->dma_offset_internal_read;
	dma_offset_internal_write = (u64)mod_desc->dma_offset_internal_write;

	verbose_printk(KERN_INFO"<pci_write>: writing peripheral with starting value: %x\n", init_write);
	verbose_printk(KERN_INFO"<pci_write>: DMA offset write value: %llx\n", dma_offset_write);
	verbose_printk(KERN_INFO"<pci_write>: DMA internal offset write value: %llx\n", dma_offset_internal_write);
	verbose_printk(KERN_INFO"<pci_write>: DMA internal offset read value: %llx\n", dma_offset_internal_read);

	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_TDFV;  // the transmit FIFO vacancy

	*(mod_desc->kernel_reg_read) = 0x0;   //set to zero
	
	ret = data_transfer(axi_dest, 0, 4, NORMAL_READ, dma_offset_internal_read);
	if (ret > 0)
	{
		printk(KERN_INFO"<pci_write>: ERROR reading from AXI Streaming FIFO control interface\n");
		return -1;
	}

	verbose_printk(KERN_INFO"										<pci_write>: Initial Transmit Data FIFO Fill Level:%x\n", *(mod_desc->kernel_reg_read));
	/*This While loop will continuously loop until the axi streaming fifo is empty
	 * if there is a holdup in data, we are stuck here....  */

	while (*(mod_desc->kernel_reg_read) != 0x01fc)  //1fc is an empty 512 depth fifo
//	while (*(mod_desc->kernel_reg_read) != 0x07fc)  //7fc is an empty 2048 depth fifo
	{
		ret = data_transfer(axi_dest, 0, 4, NORMAL_READ, dma_offset_internal_read);
		if (ret > 0)
		{
			printk(KERN_INFO"<pci_write>: ERROR reading from AXI Streaming FIFO control interface\n");
			return -1;
		}
//		printk(KERN_INFO"										<pci_write>: Transmit Data FIFO Fill Level:%x\n", *(mod_desc->kernel_reg_read));
		if (*(mod_desc->kernel_reg_read) != 0x01fc)
//		if (*(mod_desc->kernel_reg_read) != 0x07fc)
		{
			//msleep(100);
			mod_desc->ip_not_ready = mod_desc->ip_not_ready + 1;
		//	atomic_set(interr_dict[4].atomic_poll, 1);
		//	atomic_set(interr_dict[5].atomic_poll, 1);
		//	wake_up(&wq_periph);
			schedule();
			//return 0;
		}
	}

	/*Set keyhole*/
	keyhole_en = KEYHOLE_WRITE;

	axi_dest = mod_desc->axi_addr + AXI_STREAM_TDFD;  //The data transfer to the FIFO
	ret = data_transfer(axi_dest, 0, count, KEYHOLE_WRITE, dma_offset_write);
	if (ret > 0)
	{
		printk(KERN_INFO"<pci_write>: ERROR writing to AXI Streaming FIFO\n");
		return -1;
	}

	/*write to ctl interface*/
	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_TLR;

	*(mod_desc->kernel_reg_write) = (u32)count;
	ret = data_transfer(axi_dest, 0, 4, NORMAL_WRITE, dma_offset_internal_write);
	if (ret > 0)
	{
		printk(KERN_INFO"<pci_write>: ERROR writing to AXI Streaming FIFO Control Interface\n");
		return -1;
	}

	return count;
}

size_t axi_stream_fifo_read(size_t count, struct mod_desc * mod_desc)
{
	int ret;
	int keyhole_en;
	u32 read_bytes;
	u64 axi_dest;

	u64 dma_offset_write;
	u64 dma_offset_read;
	u64 dma_offset_internal_read;
	u64 dma_offset_internal_write;

	dma_offset_write = (u64)mod_desc->dma_offset_write;
	dma_offset_read = (u64)mod_desc->dma_offset_read;
	dma_offset_internal_read = (u64)mod_desc->dma_offset_internal_read;
	dma_offset_internal_write = (u64)mod_desc->dma_offset_internal_write;

	/*debug stuff*/
	//Read CTL interface
	verbose_printk(KERN_INFO"<axi_stream_fifo_read>: Reading the AXI Stream FIFO\n");
	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_ISR;

	//reset interrupts on CTL interface
	*(mod_desc->kernel_reg_write) = 0xFFFFFFFF;
	ret = data_transfer(axi_dest, 0, 4, NORMAL_WRITE, dma_offset_internal_write);
	if (ret > 0)
	{
		printk(KERN_INFO"<axi_stream_fifo_read>: ERROR writing to reset interrupts on AXI Stream FIFO\n");
		return -1;
	}


	/*Read FIFO Fill level*/
	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_RLR;
	ret = data_transfer(axi_dest, 0, 4, NORMAL_READ, dma_offset_internal_read);
	if (ret > 0)
	{
		printk(KERN_INFO"<axi_stream_fifo_read>: ERROR reading Read FIFO fill level\n");
		return -1;
	}

	/*we are masking off the 32nd bit because the FIFO is in cut through mode
	 *and sets the LSB to 1 to indicate a partial packet.*/
	read_bytes = (0x7fff & *(mod_desc->kernel_reg_read));

	verbose_printk(KERN_INFO"<axi_stream_fifo_read> Read FIFO fill level:0x%x bytes\n", read_bytes);

	/*So we don't read more data than is available*/
	if (read_bytes < (u32)count)
		count = read_bytes;

	/*if more data is available than what is requested from user space*/
	else if (read_bytes > (u32)count)
	{
		verbose_printk(KERN_INFO"<axi_stream_fifo_read> There is more data to be read than requested\n");
		verbose_printk(KERN_INFO"<axi_stream_fifo_read> This could lead to overflow of data in the FPGA\n");
	//	printk(KERN_INFO"<axi_stream_fifo_read> Lots of data in the read FIFO: 0x%x\n", read_bytes);
		/*something needs to be done here.....*/
	}

	/*Check to make sure we are doing an 8 byte aligned read*/
	if ((count % 8) > 0)
	{

		count = count - (count % 8);			
		verbose_printk(KERN_INFO"<axi_stream_fifo_read> Read value changed to: 0x%x for alignment.\n", (u32)count);
	}

	if (count == 0)
	{	
		verbose_printk(KERN_INFO"<axi_stream_fifo_read> There is either no data to read, or less than 8 bytes.\n");
		return 0;
	}

	//set CDMA KEYHOLE
	keyhole_en = KEYHOLE_READ;

	axi_dest = mod_desc->axi_addr + AXI_STREAM_RDFD;

	verbose_printk(KERN_INFO"<axi_stream_fifo_read> AXI Address of FIFO:%llx\n", mod_desc->axi_addr);

	ret = data_transfer(axi_dest, 0, count, keyhole_en, dma_offset_read);
	if (ret > 0)
	{
		printk(KERN_INFO"<axi_stream_fifo_read>: ERROR reading Data from Read FIFO\n");
		return -1;
	}

	/*Reset the AXI Streaming FIFO*/
//	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_RDFR;
//	*(mod_desc->kernel_reg_write) = 0x000000A5;
//	ret = data_transfer(axi_dest, 0, 4, NORMAL_WRITE, mod_desc->dma_offset_internal_write);
//	if (ret > 0)
//	{
//		verbose_printk(KERN_INFO"<axi_stream_fifo_read>: Resetting the interrupt IER\n");
//		return -1;
//	}

//	axi_stream_fifo_init(mod_desc);

	verbose_printk(KERN_INFO"<axi_stream_fifo_read>: Leaving the READ AXI Stream FIFO routine\n");
	return count;
}

void axi_stream_fifo_init(struct mod_desc * mod_desc)
{
	int ret;
	u64 axi_dest;

	//reset The transmit side
	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_TDFR;
	*(mod_desc->kernel_reg_write) = 0x000000A5;
	ret = data_transfer(axi_dest, 0, 4, NORMAL_WRITE, mod_desc->dma_offset_internal_write);
	verbose_printk(KERN_INFO"<axi_fifo_isr_reg>: Reset the  the axi fifo\n");

	//reset The receive side
	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_RDFR;
	*(mod_desc->kernel_reg_write) = 0x000000A5;
	ret = data_transfer(axi_dest, 0, 4, NORMAL_WRITE, mod_desc->dma_offset_internal_write);
	verbose_printk(KERN_INFO"<axi_fifo_isr_reg>: Reset the  the axi fifo\n");

	//Read CTL interface
	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_ISR;
	ret = data_transfer(axi_dest, 0, 4, NORMAL_READ, mod_desc->dma_offset_internal_read);
	verbose_printk(KERN_INFO"<axi_fifo_isr_reg>:%x\n", *(mod_desc->kernel_reg_read));

	//reset interrupts on CTL interface
	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_ISR;
	*(mod_desc->kernel_reg_write) = 0xFFFFFFFF;
	ret = data_transfer(axi_dest, 0, 4, NORMAL_WRITE, mod_desc->dma_offset_internal_write);
	verbose_printk(KERN_INFO"<axi_fifo_isr_reg>: Reset the interrupts on the axi fifo\n");

	//Read CTL interface
	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_ISR;
	ret = data_transfer(axi_dest, 0, 4, NORMAL_READ, mod_desc->dma_offset_internal_read);
	verbose_printk(KERN_INFO"<axi_fifo_isr_reg>:%x\n", *(mod_desc->kernel_reg_read));

	/*Set IER Register for interrupt on read*/
	axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_IER;
	*(mod_desc->kernel_reg_write) = 0x04000000;
	ret = data_transfer(axi_dest, 0, 4, NORMAL_WRITE, mod_desc->dma_offset_internal_write);
}
