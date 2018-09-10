/**
 * System View Device Driver

 * @date	 11/10/2015

 * @author	System View Inc.

 * @file	 sv_driver.c

 * @brief			This file contains all the kernel char driver initialization/
 *				 exit routines as well as file operations such as read, write, ioctl
 *				 and interrupt service routine
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
 #include <linux/time.h>
 #include <linux/kthread.h>
 #include <linux/kfifo.h>
 #include <linux/spinlock.h>
 #include "sv_driver.h"
 #include "xdma/sv_xdma.h"


/***********Set default values for insmod parameters***************************/
int vendor_id = PCI_VENDOR_ID_XILINX;	 /**< Insmod Patameter - PCIe vendor	*/
int device_id = 100;	/**< Insmod Parameter - PCIe specific */
int major = 0;/**< Insmod Parameter - Major number of Driver*/
uint cdma_address[CDMA_MAX_NUM] = {-1}; /**< Insmod Parameter - the AXI Address of CDMA 1*/
int cdma_count;	/**< The number of elements in cdma_address (or max value)*/
int pcie_bar_num; /**< The number of elements in pcie_ctl_address (or max value)*/
ulong pcie_ctl_address = -1; /**< The number of elements in pcie_ctl_address (or max value)*/
ulong pcie_bar_address[BAR_MAX_NUM] = {-1};/**< Insmod Parameter - The AXI Address of the PCIe Control regs*/
uint int_ctlr_address = -1;/**< Insmod Parameter - AXI Address of Interrupt Controller*/
int driver_type = PCI;/**< Insmod Parameter - Driver typem either PCIe or Platform*/
int dma_system_size = 4194304;/**< Insmod Parameter - Size of DMA Allocation, max and default is 4MB*/
int dma_file_size = 4096;/**< Insmod Parameter - Currently not used, the HW buffer size is now set on a file by file basis.*/
int dma_byte_width = 8;	/**< Insmod Parameter - This parameter is the data width of the CDMA. It is used to calculate the FIFO empty value.*/
int back_pressure = 0;/**< Insmod Parameter - This parameter sets whether the READ Ring Buffer should overwrite data or backpressure to HW.*/
uint axi2pcie_bar0_size = -1;/**< Insmod Parameter - This parameter sets the PCIE2AXI bar size for address translation parameters.*/
uint vsi_reg_intf_addr = -1;/**< Insmod Parameter - This parameter sets the PCIE2AXI bar size for address translation parameters.*/
uint interface_crc = 0;/**< Insmod Parameter - This parameter sets whether the READ Ring Buffer should overwrite data or backpressure to HW.*/
uint interface_crc_check = 0;/**< Insmod Parameter - This parameter sets whether the READ Ring Buffer should overwrite data or backpressure to HW.*/
uint pcie_use_xdma = 1; /**< Insmod Parameter : will use XDMA instead of CDMA to move data */
module_param(vendor_id, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(vendor_id, "Vendor ID");/**< Insmod Parameter */

module_param(device_id, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(device_id, "DeviceID");/**< Insmod Parameter */

module_param(major, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(major, "MajorNumber");/**< Insmod Parameter */

module_param_array(cdma_address, uint, &cdma_count, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(cdma_address, "CDMAAddress");/**< Insmod Parameter */

module_param_array(pcie_bar_address, ulong, &pcie_bar_num, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(pcie_bar_address, "PCIeBarAddress");/**< Insmod Parameter */

module_param(pcie_ctl_address, ulong, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(pcie_ctl_address, "PCIeCTLAddress");/**< Insmod Parameter */

module_param(int_ctlr_address, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(int_ctlr_address, "IntCtlrAddress");/**< Insmod Parameter */

module_param(driver_type, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(driver_type, "DriverType");/**< Insmod Parameter */

module_param(dma_system_size, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(dma_system_size, "DMASystemSize");/**< Insmod Parameter */

module_param(dma_file_size, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(dma_file_size, "DMAFileSize");/**< Insmod Parameter */

module_param(dma_byte_width, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(dma_byte_width, "DMAByteWidth");/**< Insmod Parameter */

module_param(back_pressure, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(back_pressure, "BackPressure");/**< Insmod Parameter */

module_param(axi2pcie_bar0_size, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(axi2pcie_bar0_size, "AXI2PCIE bar size");/**< Insmod Parameter */

module_param(vsi_reg_intf_addr, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(vsi_reg_intf_addr, "vsi_reg_intf_addr");/**< Insmod Parameter */

module_param(interface_crc, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(interface_crc, "interface_crc value");/**< Insmod Parameter */

module_param(interface_crc_check, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(interface_crc_check, "interface_crc_check bool");/**< Insmod Parameter */

module_param(pcie_use_xdma, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(pcie_use_xdma, "USE XDMA Instead of CDMA");/**< Insmod Parameter */

/*****************************************************************************/

struct pci_dev * pci_dev_struct = NULL; /**<pci device struct */
struct platform_device * platform_dev_struct = NULL; /**< Platform device struct (for zynq) */
struct device *	dev_struct = NULL;

char pci_driver_name[20]; //name of the device


/*This is an array of interrupt structures to hold up to 8 peripherals*/
struct file_desc * file_desc_arr[MAX_FILE_DESC] = { 0 }; /**< This is an array of Module Descriptions that is used to be indexed by interrupt number
						 * This is how the pci_isr knows which peripheral has sent the interrupt. */


/* ************************* file operations *************************** */
/**
 * @brief File Operation Function for ioctl system calls
*/
long pci_unlocked_ioctl(struct file * filep, unsigned int cmd, unsigned long arg);
/**
 * @brief File Operation Function for open system calls
*/
int pci_open(struct inode *inode, struct file *filep);
/**
 * @brief File Operation Function for close system calls
*/
int pci_release(struct inode *inode, struct file *filep);
/**
 * @brief File Operation Function for seek system calls
*/
loff_t pci_llseek( struct file *filep, loff_t off, int whence);
/**
 * @brief File Operation Function for write system calls
*/
ssize_t pci_write(struct file *filep, const char __user *buf, size_t count, loff_t *f_pos);
/**
 * @brief File Operation Function for read system calls
*/
ssize_t pci_read(struct file *filep, char __user *buf, size_t count, loff_t *f_pos);
/**
 * @brief File Operation Function for mmap
*/
int pci_mmap(struct file *filep, struct vm_area_struct *vma);
/**
 * @brief File Operation Function for poll system calls
*/
unsigned int pci_poll(struct file *filep, poll_table * pwait);


/* ************************* VM operations *************************** */
/**
 * @brief File Operation Function for ioctl system calls
*/
 void mmap_open(struct vm_area_struct *vma);
/**
 * @brief File Operation Function for open system calls
*/
void mmap_close(struct vm_area_struct *vma);
/**
 * @brief File Operation Function for close system calls
*/

/**
 * @brief File Operations Struct map
*/
struct file_operations pci_fops = {
		read:			 pci_read,
		write:		 	pci_write,
		unlocked_ioctl: pci_unlocked_ioctl,
		open:			 pci_open,
		release:		 pci_release,
		llseek:			pci_llseek,
		poll:			 pci_poll,
		mmap:			 pci_mmap,
};


/**
 * @brief VM Operations Struct map: functions that the kernel will invoke
 *						to operate in VMA
*/
struct vm_operations_struct mmap_vm_ops = {
		open:			 	mmap_open,
		close:			mmap_close,
};

/* ************************device init and exit *********************** */
static int __init sv_driver_init(void);
static void sv_driver_exit(void);
static int sv_pci_probe(struct pci_dev * dev, const struct pci_device_id * id);
static int sv_plat_probe(struct platform_device *pdev);
static void sv_pci_remove(struct pci_dev * dev);
static int sv_plat_remove(struct platform_device * dev);
static unsigned char skel_get_revision(struct pci_dev * dev);
/************************** ISR functions **************************** */
static irqreturn_t pci_isr(int irq, void *dev_id);

static struct pci_device_id ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_XILINX, 0x7022), },
	//{ PCI_DEVICE(PCI_VENDOR_ID_XILINX, 0x7021), },
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, ids);

//xdma file driver infomation struct
	struct xdma_dev *lro_global = NULL;
//sv_mod_dev driver infomation struct
	struct sv_mod_dev *svd_global = NULL;


static const struct of_device_id sv_driver_match[] = {
	{ .compatible = "xlnx,sv_driver_plat", },
	{},
};

static struct platform_driver sv_plat_driver = {
	.probe	= sv_plat_probe,
	.remove = sv_plat_remove,
	.driver = {
		.name = "sv_driver_plat",
		.of_match_table = of_match_ptr(sv_driver_match),
	},
};

static struct pci_driver sv_pci_driver = {
	//.name = "vsi_driver",
	.name = "vsi_driver",
	.id_table = ids,
	.probe = sv_pci_probe,
	.remove = sv_pci_remove,
};

MODULE_DEVICE_TABLE(of, sv_driver_match);

static unsigned char skel_get_revision(struct pci_dev *dev)
{
	u8 revision = 0;

	pci_read_config_byte(dev, PCI_REVISION_ID, &revision);
	return revision;
}

/**
 * @brief This is the Probe function called for PCIe Devices
*/
static int sv_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int int_ctrl_set;
	int rc = 0, i;
	const char * pci_name = dev->driver->name;


	verbose_printk(KERN_INFO"[probe:%s]: ******************************** PROBE PARAMETERS *****************************************\n", pci_name);
	verbose_printk(KERN_INFO"[probe:%s]: device_id: 0x%x \n", pci_name, device_id);
	verbose_printk(KERN_INFO"[probe:%s]: major: %d \n", pci_name, major);
	if(cdma_count > CDMA_MAX_NUM) {
		verbose_printk(KERN_INFO"[probe:%s]: given more CDMA address than max allowed, setting cdma_count to %d\n", pci_name, CDMA_MAX_NUM);
		cdma_count = CDMA_MAX_NUM;
	}
	for( i = 0; i < cdma_count; i++) {
		verbose_printk(KERN_INFO"[probe:%s]: cdma_ctrl_%d_address: 0x%x \n", pci_name, i, cdma_address[i]);
	}
	for( i = 0; i < pcie_bar_num; i++) {
		verbose_printk(KERN_INFO"[probe:%s]: pcie_bar_%d_address: 0x%lx \n", pci_name, i, pcie_bar_address[i]);
	}
	verbose_printk(KERN_INFO"[probe:%s]: pcie_ctl_address: 0x%lx \n", pci_name, pcie_ctl_address);
	verbose_printk(KERN_INFO"[probe:%s]: int_ctlr_address: 0x%x \n", pci_name, int_ctlr_address);
	verbose_printk(KERN_INFO"[probe:%s]: driver_type: 0x%x \n", pci_name, driver_type);
	verbose_printk(KERN_INFO"[probe:%s]: dma_system_size: %d \n", pci_name, dma_system_size);
	verbose_printk(KERN_INFO"[probe:%s]: dma_file_size: %d \n", pci_name, dma_file_size);
	verbose_printk(KERN_INFO"[probe:%s]: dma_byte_width: %d \n", pci_name, dma_byte_width);
	verbose_printk(KERN_INFO"[probe:%s]: back_pressure: %d \n", pci_name, back_pressure);
	verbose_printk(KERN_INFO"[probe:%s]: axi2pcie_bar0_size : 0x%x \n", pci_name, axi2pcie_bar0_size);
	verbose_printk(KERN_INFO"[probe:%s]: interface_crc : 0x%x \n", pci_name, interface_crc);
	verbose_printk(KERN_INFO"[probe:%s]: interface_crc_check : 0x%x \n", pci_name, interface_crc_check);
	verbose_printk(KERN_INFO"[probe:%s]: pcie_use_xdma : 0x%x \n", pci_name, pcie_use_xdma);
	verbose_printk(KERN_INFO"[probe:%s]: ******************************** BEGIN PROBE ROUTINE *****************************************\n", pci_name);

	pci_dev_struct = dev;
	dev_struct = &dev->dev;

	if(NULL == pci_dev_struct){
		printk(KERN_INFO"[probe:%s]: struct pci_dev_struct is NULL\n", pci_name);
		goto error;
	}

	svd_global = alloc_sv_dev_instance(dma_system_size);
	if (!svd_global) {
		verbose_printk("%s: OOM.\n", __func__);
		goto error;
	}
	printk(KERN_INFO"[probe:%s]: svd_global: %p\n", pci_name, svd_global);

	//bar struct to hold bar data
	svd_global->bars = kzalloc(sizeof(struct bar_info), GFP_KERNEL);
	if (!svd_global->bars) {
		pr_info("%s: OOM.\n", __func__);
		goto error;
	}
	svd_global->bars->num_bars = 0;

	//alloc xdma device struct
	if(pcie_use_xdma) {
		lro_global = alloc_dev_instance(pci_dev_struct);
		if (!lro_global) {
			verbose_printk("%s: OOM.\n", __func__);
			goto error;
		}
		printk(KERN_INFO"[probe:%s]: lro_global: %p\n", pci_name, lro_global);
	}

	/*Create Read Thread*/
	svd_global->thread_struct_read = create_thread_read(svd_global);
	/*Create Write Thread*/
	svd_global->thread_struct_write = create_thread_write(svd_global);
	for( i = 0; i < pcie_bar_num; i++) {
		svd_global->bars->pci_bar_addr[i] = pcie_bar_address[i];
	}

	//request memory region
	if(pci_request_regions(pci_dev_struct, "vsi_driver")) {
		printk(KERN_INFO"[probe:%s]: Failed to pci_request_selected_regions\n", pci_name);
		goto rel_region;
	}

	//map bars
	if(pcie_use_xdma) {
		rc = sv_xdma_map_bars(lro_global, svd_global->bars, pci_dev_struct);
		if (rc)
			goto unmap_bar;
	} else {
		rc = sv_map_bars(svd_global->bars, pci_dev_struct);
		if (rc)
			goto unmap_bar;
	}

	//enable the device
	rc = pci_enable_device(dev);
	if( rc ) {
		printk(KERN_INFO"[probe:%s]: !!!!!!!!ERROR pci_enable_device\n", pci_name);
		goto disable_device;
	}

	//set DMA mask
	if(0 != dma_set_coherent_mask(&dev->dev, 0x00000000FFFFFFFF)){
		printk(KERN_INFO"[probe:%s]: set DMA mask error\n", pci_name);
		goto disable_device;
	}
	verbose_printk(KERN_INFO"[probe:%s]: dma mask set\n", pci_name);

	//enable bus mastering
	pci_set_master(dev);
	verbose_printk(KERN_INFO"[probe:%s]: pci set as master\n", pci_name);
//-----------------------------------------------NEW-----------------------------------------------
	if((driver_type == PCI || driver_type == AWS) && pcie_use_xdma) {
		//MSIX
		if(msi_msix_capable(pci_dev_struct, PCI_CAP_ID_MSIX)) {
			int req_nvec = MAX_NUM_ENGINES + MAX_USER_IRQ;

			verbose_printk(KERN_INFO"[probe:%s]: MSIX capable XDMA\n", pci_name);

			for (i = 0; i < req_nvec; i++)
				svd_global->sv_msix_entry[i].entry = i;

			verbose_printk("[probe:%s]: pci_enable_msix()\n", pci_name);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 0, 0)
			rc = pci_enable_msix_range(pci_dev_struct, svd_global->sv_msix_entry, req_nvec, req_nvec);
#else
			rc = pci_enable_msix(pci_dev_struct, svd_global->sv_msix_entry, req_nvec);
#endif
			if (rc < 0) {
				verbose_printk("[probe:%s]: ERROR Couldn't enable MSI-X mode: rc = %d\n", pci_name, rc);
				goto disable_device;
			}
			lro_global->msix_enabled = 0;

			// take over the user interrupts, the rest will be
			// handles by xdma engine if enabled
			for (i = 0 ; i < MAX_USER_IRQ ; i++) {
				rc = request_irq(svd_global->sv_msix_entry[i].vector, pci_isr, 0 , pci_name, pci_dev_struct);
				if (rc) {
					printk(KERN_INFO"[probe:%s]: MSI-X Couldn't use IRQ#%d, rc=%d\n", pci_name, svd_global->sv_msix_entry[i].vector, rc);
					break;
				}
				verbose_printk(KERN_INFO"[probe:%s]: Using IRQ#%d\n", pci_name, svd_global->sv_msix_entry[i].vector);

			}
			if (rc) {
				while (--i >= 0)
					free_irq(svd_global->sv_msix_entry[i].vector, pci_dev_struct);
				goto disable_device;
			}
			lro_global->irq_user_count = MAX_USER_IRQ;


		}
		//MSI
		else if (msi_msix_capable(pci_dev_struct, PCI_CAP_ID_MSI)) {
			verbose_printk(KERN_INFO"[probe:%s]: MSI capable XDMA\n", pci_name);
			//fix bar mapping?

			verbose_printk("[probe:%s]: pci_enable_msi()\n", pci_name);
			rc = pci_enable_msi(pci_dev_struct);
			if (rc) {
				verbose_printk("[probe:%s]: ERROR Couldn't enable MSI mode: rc = %d\n", pci_name, rc);
				goto disable_device;

			}
			lro_global->msi_enabled = 1;

			rc = request_irq(pci_dev_struct->irq, &pci_isr, IRQF_TRIGGER_RISING | IRQF_SHARED, pci_name, pci_dev_struct);
			if(rc){
				printk(KERN_INFO"%s:[probe]request IRQ error\n", pci_name);
				goto disable_device;
			}
            verbose_printk(KERN_INFO"[probe:%s]: Using IRQ#%d\n", pci_name, pci_dev_struct->irq);

		} else {
			//LEGACY
			verbose_printk(KERN_INFO"[probe:%s]: Legacy interrupts XDMA\n", pci_name);
		}

		verbose_printk("[probe:%s]: probe_engines()\n", pci_name);

		rc = probe_engines(lro_global);
		if (rc) {
			verbose_printk(KERN_INFO"[probe:%s]: ERROR probe_engines\n", pci_name);
			goto disable_device;
		}

		verbose_printk("[probe:%s]: create_interfaces()\n", pci_name);
		rc = create_interfaces(lro_global);
		if (rc) {
			verbose_printk(KERN_INFO"[probe:%s]: ERROR create_interfaces\n", pci_name);
			goto disable_device;
		}

		xdma_init_sv(lro_global);

		/* enable user interrupts */
		//printk(KERN_INFO"[probe:%s]: Using IRQ#%d with 0x%p\n", pci_name, pci_dev_struct->irq, &pci_dev_struct);
		user_interrupts_enable(lro_global, ~0);

	}
	//-----------------------------------------------END NEW-----------------------------------------------
	else if(driver_type == PCI) {
		verbose_printk(KERN_INFO"[probe:%s]: MSI capable PCI\n", pci_name);

		// allocate vectors
		if(0 != pci_enable_msi(pci_dev_struct)) {
			printk(KERN_INFO"[probe:%s]: Alloc MSI failed\n", pci_name);
			goto disable_device;
		}
		// take over the user interrupts, the rest will be
		// handles by xdma engine if enabled

		if(0 > request_irq(pci_dev_struct->irq, &pci_isr, IRQF_TRIGGER_RISING | IRQF_SHARED, pci_name, pci_dev_struct)){
			printk(KERN_INFO"%s:[probe]request IRQ error\n", pci_name);
			goto disable_device;
		}

		printk(KERN_INFO"[probe:%s]: Using IRQ#%d with 0x%p\n", pci_name, pci_dev_struct->irq, &pci_dev_struct);
	}
	else{
		verbose_printk(KERN_INFO"[probe:%s]: ERROR Unknown setup\n", pci_name);
		goto disable_device;
	}

	verbose_printk(KERN_INFO"[probe:%s]: pci enabled msi interrupt\n", pci_name);

	//register the char device
	rc = register_chrdev(major, pci_name, &pci_fops);
	if(0 > rc){
		//	dynamic_major = register_chrdev(0, pci_name, &pci_fops);
		printk(KERN_INFO"[probe:%s]: char driver not registered rc: 0x%x\n", pci_name, rc);
		printk(KERN_INFO"[probe:%s]: char driver major number: %d\n", pci_name, major);
		goto rmv_cdev;
	}
    else if(major == 0){
        major = rc;
        printk(KERN_INFO"[probe:%s]: char driver major number: %d\n", pci_name, major);
    }

	if(skel_get_revision(dev) == 0x42)
		goto rmv_cdev;

	/*allocate the DMA buffer*/
	svd_global->dma_buffer_base = dma_alloc_coherent(dev_struct, (size_t)svd_global->dma_buffer_size, &svd_global->dma_addr_base, GFP_KERNEL);

	if(NULL == svd_global->dma_buffer_base) {
		printk(KERN_INFO"[probe:%s]: DMA buffer base allocation ERROR\n", pci_name);
		printk(KERN_INFO"[probe:%s]: typical max DMA size is 4M, check your linux settings\n", pci_name);
		goto free_alloc;
	} else {
		verbose_printk(KERN_INFO"[probe:%s]: dma kernel buffer base address is:0x%p\n", pci_name, svd_global->dma_buffer_base);
		verbose_printk(KERN_INFO"[probe:%s]: dma system memory buffer base address is:0x%p\n", pci_name, (void *)svd_global->dma_addr_base);
		svd_global->dma_current_offset = 0;	//we want to leave the first 4k for the kernel to use internally.
	}

	//set defaults
	for(i = 0; i < cdma_count; i++){
		svd_global->cdma_set[i] = 0;
	}

	int_ctrl_set = 0;

	for( i = 0; i < cdma_count; i++){
		if(cdma_address[i] != -1 && !pcie_use_xdma) {
			if( cdma_init(svd_global, i, cdma_address[i]) ) { //cdma_num = 1
				printk(KERN_INFO"[probe:%s]: !!!!!!!!ERROR cdma_init(%d, %08x)\n", pci_name, i, cdma_address[i]);
					goto free_alloc;
			}
		}
	}

	if(axi2pcie_bar0_size != -1 && pcie_ctl_address != -1) {
		svd_global->axi_pcie_m = svd_global->dma_addr_base % axi2pcie_bar0_size;
		if(pcie_ctl_init((u64)pcie_ctl_address, (u64)(svd_global->dma_addr_base - svd_global->axi_pcie_m)) ) {
			printk(KERN_INFO"[probe:%s]: !!!!!!!!ERROR pcie_ctl_init(0x%llx, 0x%llx)\n", pci_name, (u64)pcie_ctl_address, (u64)(svd_global->dma_addr_base - svd_global->axi_pcie_m));
			goto free_alloc;
		}
	} else if(pcie_ctl_address != -1) {
		printk(KERN_INFO"[probe:%s]: axi2pcie_bar0_size not set\n", pci_name);
		printk(KERN_INFO"[probe:%s]: If this is a gen2 PCIE design address translation will fail\n", pci_name);
		svd_global->axi_pcie_m = 0;
		if(pcie_ctl_init((u64)pcie_ctl_address, (u64)(svd_global->dma_addr_base)) ) {
			printk(KERN_INFO"[probe:%s]: !!!!!!!!ERROR pcie_ctl_init(0x%llx, 0x%llx)\n", pci_name, (u64)pcie_ctl_address, (u64)svd_global->dma_addr_base);
			goto free_alloc;
		}
		//return ERROR;
	} else {
		printk(KERN_INFO"[probe:%s]: !!!!!!!!ERROR axi2pcie_bar0_size and pcie_ctl_address was not set\n", pci_name);
		svd_global->axi_pcie_m = 0;
	}

	printk(KERN_INFO"[probe:%s]: DMA hardware offset(axi_pcie_m) = 0x%08llx\n", pci_name, svd_global->axi_pcie_m);

	if(int_ctlr_address != -1) {
		axi_intc_init(svd_global, int_ctlr_address);
		int_ctrl_set = 1;
	}

	for(i = 0; i < cdma_count; i++){
		verbose_printk(KERN_INFO"[probe:%s]: cdma_set[%d] = 0x%x\n", pci_name, i, svd_global->cdma_set[i]);
		svd_global->cdma_capable += (svd_global->cdma_set[i] & int_ctrl_set);
	}

	printk(KERN_INFO"[probe:%s]: cdma_capable = 0x%x\n", pci_name, svd_global->cdma_capable);
	printk(KERN_INFO"[probe:%s]: ***********************PROBE FINISHED SUCCESSFULLY**************************************\n", pci_name);
	return 0;

	free_alloc:
		printk(KERN_INFO"[probe:%s]: Cleanup: free dma\n", pci_name);
		dma_free_coherent(dev_struct, (size_t)svd_global->dma_buffer_size, svd_global->dma_buffer_base, svd_global->dma_addr_base);

	rmv_cdev:
		printk(KERN_INFO"[probe:%s]: Cleanup: unregister char driver\n", pci_name);
		unregister_chrdev(major, pci_name);

	disable_device:
		printk(KERN_INFO"[probe:%s]: Cleanup: disable device \n", pci_name);
		pci_disable_device(dev);

	unmap_bar:
		printk(KERN_INFO"[probe:%s]: Cleanup: unmap bars \n", pci_name);
		if(pcie_use_xdma)
			sv_xdma_unmap_bars(lro_global, svd_global->bars, dev);
		else
			sv_unmap_bars(svd_global->bars, dev);

	rel_region:
		printk(KERN_INFO"[probe:%s]: Cleanup: release region \n", pci_name);
		pci_release_regions(dev);

	error:
		printk("[probe:%s]: probe returning %d\n", pci_name, rc);
		return -1;

}

/**
 * @brief This is the Probe function called for Platform Devices (ie Zynq)
*/
static int sv_plat_probe(struct platform_device *pdev)
{
	struct resource * resource;
	int int_ctrl_set;
	int i;
	int rc = 0;
	const char * plat_name = sv_plat_driver.driver.name;

	verbose_printk(KERN_INFO"[probe:%s]: ******************************** PROBE PARAMETERS *****************************************\n", plat_name);
	verbose_printk(KERN_INFO"[probe:%s]: device_id: 0x%x \n", plat_name, device_id);
    if(major == 0) {
        verbose_printk(KERN_INFO"[probe:%s]: major: dynamic \n", plat_name);
    }
    else {
    	verbose_printk(KERN_INFO"[probe:%s]: major: %d \n", plat_name, major);
    }
	if(cdma_count > CDMA_MAX_NUM) {
		verbose_printk(KERN_INFO"[probe:%s]: given more CDMA address than max allowed, setting cdma_count to %d\n", plat_name, CDMA_MAX_NUM);
		cdma_count = CDMA_MAX_NUM;
	}
	for( i = 0; i < cdma_count; i++) {
		verbose_printk(KERN_INFO"[probe:%s]: cdma_ctrl_%d_address: 0x%x \n", plat_name, i, cdma_address[i]);
	}
	for( i = 0; i < pcie_bar_num; i++) {
		verbose_printk(KERN_INFO"[probe:%s]: pcie_bar_%d_address: 0x%lx \n", plat_name, i, pcie_bar_address[i]);
	}
	verbose_printk(KERN_INFO"[probe:%s]: pcie_ctl_address: 0x%lx \n", plat_name, pcie_ctl_address);
	verbose_printk(KERN_INFO"[probe:%s]: int_ctlr_address: 0x%x \n", plat_name, int_ctlr_address);
	verbose_printk(KERN_INFO"[probe:%s]: driver_type: 0x%x \n", plat_name, driver_type);
	verbose_printk(KERN_INFO"[probe:%s]: dma_system_size: %d \n", plat_name, dma_system_size);
	verbose_printk(KERN_INFO"[probe:%s]: dma_file_size: %d \n", plat_name, dma_file_size);
	verbose_printk(KERN_INFO"[probe:%s]: dma_byte_width: %d \n", plat_name, dma_byte_width);
	verbose_printk(KERN_INFO"[probe:%s]: back_pressure: %d \n", plat_name, back_pressure);
	verbose_printk(KERN_INFO"[probe:%s]: axi2pcie_bar0_size : 0x%x \n", plat_name, axi2pcie_bar0_size);
	verbose_printk(KERN_INFO"[probe:%s]: interface_crc : 0x%x \n", plat_name, interface_crc);
	verbose_printk(KERN_INFO"[probe:%s]: interface_crc_check : 0x%x \n", plat_name, interface_crc_check);
	if(pcie_use_xdma){
		verbose_printk(KERN_INFO"[probe:%s]: pcie_use_xdma is not 0 on platform\n", plat_name);
		verbose_printk(KERN_INFO"[probe:%s]: setting pcie_use_xdma to 0\n", plat_name);
		pcie_use_xdma = 0;
	}
	verbose_printk(KERN_INFO"[probe:%s]: pcie_use_xdma : 0x%x \n", plat_name, pcie_use_xdma);
	verbose_printk(KERN_INFO"[probe:%s]: ******************************** BEGIN PROBE ROUTINE *****************************************\n", plat_name);

	platform_dev_struct = pdev;
	dev_struct = &pdev->dev;

	svd_global = alloc_sv_dev_instance(dma_system_size);
	if (!svd_global) {
		verbose_printk("%s: OOM.\n", __func__);
		goto error;
	}

	printk(KERN_INFO"[probe:%s]: svd_global: %p\n", plat_name, svd_global);

	//bar struct to hold bar data
	svd_global->bars = kzalloc(sizeof(struct bar_info), GFP_KERNEL);
	if (!svd_global->bars) {
		pr_info("%s: OOM.\n", __func__);
		goto error;
	}
	svd_global->bars->num_bars = 0;

	/*Create Read Thread*/
	svd_global->thread_struct_read = create_thread_read(svd_global);
	/*Create Write Thread*/
	svd_global->thread_struct_write = create_thread_write(svd_global);


	for(i = 0; i < BAR_MAX_NUM; i++){

		//look for a second resource
		resource = platform_get_resource(pdev, IORESOURCE_MEM, i);

		if(!resource)	 //if null
		{
			printk(KERN_INFO"[probe:%s]: platform_get_resource for bar %d not found\n", plat_name, i);
			svd_global->bars->pci_bar_addr[i] = 0;
			svd_global->bars->pci_bar_end[i] = 0;
		} else {
			//get the control memory size
			svd_global->bars->pci_bar_addr[i] = resource->start;
			svd_global->bars->pci_bar_end[i] = resource->end;
			printk(KERN_INFO"[probe:%s]: platform name is: %s\n", plat_name, resource->name);
			printk(KERN_INFO"[probe:%s]: platform bar %d size is:0x%lx\n", plat_name, i, (unsigned long)resource_size(resource));
			printk(KERN_INFO"[probe:%s]: platform bar %d start is:%lx end is:%lx\n", plat_name, i, svd_global->bars->pci_bar_addr[i], svd_global->bars->pci_bar_end[i]);
			//map the hardware space to virtual space
			svd_global->bars->pci_bar_vir_addr[i] = devm_ioremap_resource(&pdev->dev, resource);
			if(IS_ERR(svd_global->bars->pci_bar_vir_addr[i])){
				printk(KERN_INFO"[probe:%s]: ioremap error when mapping to virtual address %d \n", plat_name, i);
				goto error;
			}
			printk(KERN_INFO"[probe:%s]: pci bar %d virtual address base is:0x%p\n", plat_name, i, svd_global->bars->pci_bar_vir_addr[i]);
			svd_global->bars->num_bars++;
		}

	}
	verbose_printk(KERN_INFO"[probe:%s]: num_bars : 0x%x \n", plat_name, svd_global->bars->num_bars);


	//set DMA mask
	if(0 != dma_set_mask(&pdev->dev, 0x00000000ffffffff)){
		printk(KERN_INFO"[probe:%s]: set DMA mask error\n", plat_name);
		goto error;
	}
	printk(KERN_INFO"[probe:%s]: dma mask set\n", plat_name);

	svd_global->irq_num = platform_get_irq(pdev, 0);
	printk(KERN_INFO"[probe:%s]: IRQ number is:%d\n", plat_name, svd_global->irq_num);

	//register the char device
	rc = register_chrdev(major, "sv_driver", &pci_fops);

	if(0 > rc){
		printk(KERN_INFO"[probe:%s]: char driver not registered, 0x%x\n", plat_name, rc);
		goto error;
	}

	printk(KERN_INFO"[probe:%s]: register device complete going to alloc %llx byte for dma\n", plat_name, svd_global->dma_buffer_size);

	/*allocate the DMA buffer*/
	svd_global->dma_buffer_base = dma_alloc_coherent(dev_struct, (size_t)svd_global->dma_buffer_size, &svd_global->dma_addr_base, GFP_KERNEL);
	if(NULL == svd_global->dma_buffer_base) {
		printk(KERN_INFO"[probe:%s]: DMA buffer base allocation ERROR\n", plat_name);
		goto rmv_cdev;
	} else {
		printk(KERN_INFO"[probe:%s]: dma buffer base address is:0x%p\n", plat_name, svd_global->dma_buffer_base);
		printk(KERN_INFO"[probe:%s]: dma system memory HW base address is:%llx\n", plat_name, (u64)svd_global->dma_addr_base);
		printk(KERN_INFO"[probe:%s]: dma system memory buffer size is:%llx\n", plat_name, (u64)svd_global->dma_buffer_size);
		svd_global->dma_current_offset = 0;
	}
	printk(KERN_INFO"[probe:%s]: alloc coherent complete\n", plat_name);

	//set defaults
	for(i = 0; i < cdma_count; i++){
		svd_global->cdma_set[i] = 0;
	}

	int_ctrl_set = 0;

	// these addresses should alayws be in the 32 bit range
	//int_ctlr_address &= 0xFFFFFFFF;

	for( i = 0; i < cdma_count; i++){
		cdma_address[i]	&= 0xFFFFFFFF;
		if(cdma_address[i] != -1) {
			if( cdma_init(svd_global, i, cdma_address[i]) ) { //cdma_num = 1
				printk(KERN_INFO"[probe:%s]: !!!!!!!!ERROR cdma_init(%d, %08x)\n", plat_name, i, cdma_address[i]);
				goto free_alloc;
			}
		}
	}

	/*ARM works a little differently for DMA than PCIe in that that translation
	* is not handled by the core by writing to a register. For Zynq, the axi to DDR address
	* mapping is 1-1 and should be written directly to the returned DMA handle */

	//request IRQ last
	if(0 > request_irq(svd_global->irq_num, &pci_isr, IRQF_TRIGGER_RISING | IRQF_SHARED, plat_name, pdev)){
		printk(KERN_INFO"[probe:%s]: request IRQ error\n", plat_name);
		goto free_alloc;
	}
	printk(KERN_INFO"[test]: post request IRQ \n");

	printk(KERN_INFO"[test]: axi_intc_init 0x%08x\n", int_ctlr_address);
	if(int_ctlr_address != -1) {
		axi_intc_init(svd_global, int_ctlr_address);
		int_ctrl_set = 1;
	}

	printk(KERN_INFO"[probe:%s]: Using IRQ#%d with 0x%p\n", plat_name, svd_global->irq_num, &platform_dev_struct);

	svd_global->axi_pcie_m = (u64)svd_global->dma_addr_base;	//cdma 1

	for(i = 0; i < cdma_count; i++){
		verbose_printk(KERN_INFO"[probe:%s]: cdma_set[%d] = 0x%x\n", plat_name, i, svd_global->cdma_set[i]);
		svd_global->cdma_capable += ( svd_global->cdma_set[i] & int_ctrl_set ) ;
	}


	printk(KERN_INFO"[probe:%s]: cdma_capable = 0x%x\n", plat_name, svd_global->cdma_capable);

	printk(KERN_INFO"[probe:%s]: ***********************PROBE FINISHED SUCCESSFULLY**************************************\n", plat_name);
	return 0;


	free_alloc:
		dma_free_coherent(dev_struct, (size_t)svd_global->dma_buffer_size, svd_global->dma_buffer_base, svd_global->dma_addr_base);

	rmv_cdev:
		unregister_chrdev(major, plat_name);

	error:
		printk("[probe:%s]: probe returning %d\n", plat_name, rc);
		return -1;

}

static void sv_pci_remove(struct pci_dev *dev)
{
	const char * pci_name = dev->driver->name;
	int i;
	// clean up any allocated resources and stuff here.
	printk(KERN_INFO"%s[sv_pci_remove]: PCIE remove\n", pci_name);


	//disable interrupts from the axi interrupt controller
	if(svd_global->axi_intc_addr != -1) {
		axi_intc_deinit(svd_global);
	}

	// free and disable IRQ

	if(msi_msix_capable(dev, PCI_CAP_ID_MSIX)) {
		printk(KERN_INFO"[probe:%s]: MSIX capable PCI remove\n", pci_name);
		for (i = 0 ; i < MAX_INTERRUPTS; i++)
			free_irq(svd_global->sv_msix_entry[i].vector, dev);


		printk(KERN_INFO"%s[sv_pci_remove]: free_irq done\n", pci_name);
		pci_disable_msix(dev);
		printk(KERN_INFO"%s[sv_pci_remove]: MSI-X disabled\n", pci_name);
	} else if(msi_msix_capable(pci_dev_struct, PCI_CAP_ID_MSI)) {
		printk(KERN_INFO"[probe:%s]: MSIX capable PCI remove\n", pci_name);
		free_irq(dev->irq, dev);
		printk(KERN_INFO"%s[sv_pci_remove]: free_irq done\n", pci_name);
		pci_disable_msi(dev);
		printk(KERN_INFO"%s[sv_pci_remove]: MSI disabled\n", pci_name);
	} else {
		//LEGACY
		printk(KERN_INFO"[probe:%s]: Legacy PCI remove\n", pci_name);
		free_irq(dev->irq, dev);
		printk(KERN_INFO"%s[sv_pci_remove]: free_irq done\n", pci_name);
		pci_disable_msi(dev);
		printk(KERN_INFO"%s[sv_pci_remove]: MSI disabled\n", pci_name);
	}







	/*Destroy the read thread*/
	printk(KERN_INFO"[sv_pci_remove]: Stopping read thead\n");
	atomic_set(&svd_global->thread_q_read, 1);
	if(svd_global->thread_struct_read) {
		while(kthread_stop(svd_global->thread_struct_read)<0) {
			wake_up_interruptible(&svd_global->thread_q_head_read);
			printk(KERN_INFO"[sv_pci_remove]: Read thread failed to stop, attemping again\n");
			atomic_set(&svd_global->thread_q_read, 1);
			wake_up_interruptible(&svd_global->thread_q_head_read);
		}
		put_task_struct(svd_global->thread_struct_read);
	}
	printk(KERN_INFO"[sv_pci_remove]: Read Thread Destroyed\n");

	/*Destroy the write thread*/
	printk(KERN_INFO"[sv_pci_remove]: Stopping write thead\n");
	atomic_set(&svd_global->thread_q_write, 1);
	if(svd_global->thread_struct_write) {
		wake_up_interruptible(&svd_global->thread_q_head_write);
		while(kthread_stop(svd_global->thread_struct_write)<0) {
			printk(KERN_INFO"[sv_pci_remove]: Write thread failed to stop, attemping again\n");
			atomic_set(&svd_global->thread_q_write, 1);
			wake_up_interruptible(&svd_global->thread_q_head_write);
		}
		put_task_struct(svd_global->thread_struct_write);
	}
	printk(KERN_INFO"[sv_pci_remove]: Write Thread Destroyed\n");

	unregister_chrdev(major, pci_name);

	printk(KERN_INFO"[sv_pci_remove]: unmapping pci bars\n");
	if(pcie_use_xdma) {
		sv_xdma_unmap_bars(lro_global, svd_global->bars, dev);
	} else {
		sv_unmap_bars(svd_global->bars, dev);
	}

	pci_release_regions(dev);
	dma_free_coherent(dev_struct, (size_t)svd_global->dma_buffer_size, svd_global->dma_buffer_base, svd_global->dma_addr_base);

	printk(KERN_INFO"[sv_pci_remove]: ***********************PCIE DEVICE REMOVED**************************************\n");

}

static int sv_plat_remove(struct platform_device *pdev)
{
	int i;
	const char * plat_name = sv_plat_driver.driver.name;

	printk(KERN_INFO"[sv_plat_remove]: Platform remove\n");

	if(svd_global->axi_intc_addr != -1) {
		axi_intc_deinit(svd_global);
	}

	free_irq(svd_global->irq_num, platform_dev_struct);

	/*Destroy the read thread*/
	printk(KERN_INFO"[sv_plat_remove]: Stopping read thead\n");
	atomic_set(&svd_global->thread_q_read, 1);
	wake_up_interruptible(&svd_global->thread_q_head_read);
	if(svd_global->thread_struct_read) {
		while(kthread_stop(svd_global->thread_struct_read)<0)
		{
			printk(KERN_INFO"[sv_plat_remove]: Read thread failed to stop, attemping again\n");
			atomic_set(&svd_global->thread_q_read, 1);
			wake_up_interruptible(&svd_global->thread_q_head_read);
		}
	}
	printk(KERN_INFO"[sv_plat_remove]: Read Thread Destroyed\n");

	/*Destroy the write thread*/
	printk(KERN_INFO"[sv_plat_remove]: Stopping write thead\n");
	atomic_set(&svd_global->thread_q_write, 1);
	wake_up_interruptible(&svd_global->thread_q_head_write);
	if(svd_global->thread_struct_write) {
		while(kthread_stop(svd_global->thread_struct_write)<0) {
			printk(KERN_INFO"[sv_plat_remove]: Write thread failed to stop, attemping again\n");
			atomic_set(&svd_global->thread_q_write, 1);
			wake_up_interruptible(&svd_global->thread_q_head_write);
		}
	}
	printk(KERN_INFO"[sv_plat_remove]: Write Thread Destroyed\n");

	unregister_chrdev(major, plat_name);
	for(i = 0; i < svd_global->bars->num_bars; i++){
		iounmap(svd_global->bars->pci_bar_vir_addr[i]);
	}
	dma_free_coherent(dev_struct, (size_t)svd_global->dma_buffer_size, svd_global->dma_buffer_base, svd_global->dma_addr_base);

	printk(KERN_INFO"[sv_plat_remove]: ***********************PLATFORM DEVICE REMOVED**************************************\n");

	return 0;
}

static int __init sv_driver_init(void)
{
	sprintf(pci_driver_name, "vsi_driver_0x%x", device_id);
	memset(file_desc_arr, 0, sizeof(file_desc_arr));

	printk(KERN_INFO"[sv_driver_init]: Initializing driver type %d\n", driver_type);
	switch(driver_type){
		case PCI:
		case AWS:
			printk(KERN_INFO"[pci_init]: Device ID: ('0x%x')\n", device_id);
			printk(KERN_INFO"[pci_init]: Major Number: ('%d')\n", major);
			printk(KERN_INFO"[pci_init]: Name: ('%s')\n", pci_driver_name);

			ids[0].vendor = (u32)vendor_id;			  //PCI_VENDOR_ID_XILINX;
			ids[0].device = (u32)device_id;
			sv_pci_driver.name = pci_driver_name;


			return pci_register_driver(&sv_pci_driver);
			break;

		case PLATFORM:

			printk(KERN_INFO"[platform_init]: Device ID: ('0x%x')\n", device_id);
			printk(KERN_INFO"[platform_init]: Major Number: ('%d')\n", major);
			return platform_driver_register(&sv_plat_driver);
			break;

		default:;
	}

	printk(KERN_INFO"[platform_init]: \t!!!!ERROR!!!!! No correct driver type detected!\n");
	return 0;
}

static void __exit sv_driver_exit(void)
{
	switch(driver_type){
		case PCI:
		case AWS:
			printk(KERN_INFO"[sv_driver_exit]: pci_driver_unregister started \n");
			pci_unregister_driver(&sv_pci_driver);
			printk(KERN_INFO"[sv_driver_exit]: pci_driver_unregister done \n");

			break;

		case PLATFORM:
			printk(KERN_INFO"[sv_driver_exit]: platform_driver_unregister started \n");
			platform_driver_unregister(&sv_plat_driver);
			break;

		default:;
	}

}

MODULE_LICENSE("GPL");

module_init(sv_driver_init);
module_exit(sv_driver_exit);

/********************** ISR Stuff ***************************/
static irqreturn_t pci_isr(int irq, void *dev_id)
{
	u32 status, isr_status;
	u64 axi_dest;
	int interrupt_num;
	u32 device_mode;
	u32 vec_serviced;
	struct file_desc* irq_file;

	verbose_isr_printk(KERN_INFO"[pci_isr]: Entered the ISR (0x%x)\n", svd_global->axi_intc_addr);
	//	tasklet_schedule(&isr_tasklet);
	if(svd_global-> interrupt_set == 0){
		if(svd_global->axi_intc_addr == -1) {
			printk(KERN_INFO"[pci_isr]: ERROR: returning early ISR (%x)\n", svd_global->axi_intc_addr);
			// controller not intialized yet
			return IRQ_NONE;
		}
		else{
			verbose_isr_printk(KERN_INFO"[pci_isr]: Software Interrupt!\n");
			atomic_set(&svd_global->sw_interrupt_rx, 1);
			//recieved the sw interrupt test

			/*Here we need to clear the service interrupt in the interrupt acknowledge register*/
			status = 0xFFFFFFFF;
			axi_dest = svd_global->axi_intc_addr + INT_CTRL_IAR;
			if( direct_write(axi_dest, (void *)&status, 4, NORMAL_WRITE) ) {
				printk(KERN_INFO"[axi_intc_init]: \t!!!!!!!!ERROR: in direct_write!!!!!!!\n");
				return IRQ_NONE;
			}
			return IRQ_HANDLED;

		}
	}
	vec_serviced = 0;
	/*Here we need to find out who triggered the interrupt*
	 *Since we only allow one MSI vector, we need to query the
	 *Interrupt controller to find out. */

	/*This is the interrupt status register*/
	axi_dest = svd_global->axi_intc_addr + INT_CTRL_IPR;
	if( direct_read(axi_dest, (void *)&status, 4, NORMAL_READ) ) {
		printk(KERN_INFO"[pci_isr]: !!!!!!!!ERROR direct_read\n");
		return IRQ_NONE;
	}

	verbose_isr_printk(KERN_INFO"[pci_isr]: interrupt status register vector is: ('0x%08x')\n", status);
	/*Here we need to clear the service interrupt in the interrupt acknowledge register*/
	axi_dest = svd_global->axi_intc_addr + INT_CTRL_IAR;
	isr_status = status;
	if( direct_write(axi_dest, (void *)&status, 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[pci_isr]: !!!!!!!!ERROR direct_write\n");
		return IRQ_NONE;
	}

	while(isr_status > 0) {

		interrupt_num = vec2num(isr_status);
		verbose_isr_printk(KERN_INFO"[pci_isr]: interrupt status interrupt number is: ('%i')\n", interrupt_num);
		irq_file = file_desc_arr[interrupt_num];

		//check to file_desc is not null for this interrupt_num and that it doesn't step over our array
		if(interrupt_num >= MAX_FILE_DESC || !irq_file) {
			//do nothing
			verbose_isr_printk(KERN_INFO"[pci_isr]: WARNING: Interrupt not set for this driver or interrupt over max. \n");
		}
		else {
			verbose_isr_printk(KERN_INFO"[pci_isr]: interrupt number is: ('%d'-->' minor: %x')\n", interrupt_num, irq_file->minor);
			device_mode = irq_file->mode;

			if(device_mode == AXI_STREAM_FIFO || device_mode == AXI_STREAM_PACKET) {
				verbose_isr_printk(KERN_INFO"[pci_isr]: this interrupt is from a streaming peripheral\n");
				clear_fifo_isr(irq_file);
				//Put streaming types into the kfifo for read thread
				if(atomic_read(irq_file->in_read_fifo_count) == 0 && irq_file->file_open) {
					//debug message
					if(kfifo_len(&irq_file->svd->read_fifo) > 4) {
						verbose_isr_printk(KERN_INFO"[pci_isr]: kfifo stored elements: %d\n", kfifo_len(&irq_file->svd->read_fifo));
					}

					//if the read fifo isn't full and the file is open, write the file_desc in
					if(!kfifo_is_full(&irq_file->svd->read_fifo)) {
						atomic_inc(irq_file->in_read_fifo_count);
						kfifo_in_spinlocked(&irq_file->svd->read_fifo, &irq_file, 1, &irq_file->svd->fifo_lock_read);
					}
					else {
						verbose_isr_printk(KERN_INFO"[pci_isr]: kfifo is full, not writing mod desc\n");
					}
				}
				else if(!irq_file->file_open) {
					verbose_isr_printk(KERN_INFO"[pci_isr]: file is closed, not writing file_desc\n");
				}
				else {
					verbose_isr_printk(KERN_INFO"[pci_isr]: kfifo already contains axi_steam_fifo mod desc\n");
				}
				verbose_isr_printk(KERN_INFO"[pci_isr]: Waking up the read thread\n");

				//for ring buffer peripherals (axi streaming) there is now data so we should read
				atomic_set(&svd_global->thread_q_read, 1); // wake up if empty
				wake_up_interruptible(&svd_global->thread_q_head_read);
			}
			else {
				atomic_set(irq_file->atomic_poll, 1); //non threaded way
				//for non ring buffer peripherals (memory)
				wake_up(&irq_file->poll_wq);
				verbose_isr_printk(KERN_INFO"[pci_isr]: Waking up the Poll(normal)\n");
			}

			vec_serviced = vec_serviced | num2vec(interrupt_num);
		}

		//get next interrupt number
		isr_status = isr_status & ~num2vec(interrupt_num);
		verbose_isr_printk(KERN_INFO"[pci_isr]: vectors serviced is: ('0x%08x')\n", vec_serviced);
		verbose_isr_printk(KERN_INFO"[pci_isr]: end interrupt status register vector is: ('0x%08x')\n", isr_status);
	}

	verbose_isr_printk(KERN_INFO"[pci_isr]: All interrupts serviced. The following Vector is acknowledged: 0x%x\n", vec_serviced);
	verbose_isr_printk(KERN_INFO"[pci_isr]: Exiting ISR\n");

	return IRQ_HANDLED;

}

/* -----------------File Operations-------------------------*/
int pci_open(struct inode *inode, struct file *filep)
{
	//void * interrupt_count;
	struct file_desc * s;
	struct timespec * start_time;
	struct timespec * stop_time;

	atomic_t * atomic_poll;
	atomic_t * wth;
	atomic_t * wtk;
	atomic_t * write_ring_buf_full;
	atomic_t * write_ring_buf_locked;

	atomic_t * rfh;
	atomic_t * rfu;
	atomic_t * read_ring_buf_full;
	atomic_t * read_ring_buf_locked;

	atomic_t * pci_write_q;
	atomic_t * in_read_fifo_count;
	atomic_t * in_write_fifo_count;
	atomic_t * mmap_count;


	spinlock_t * in_fifo_read;
	spinlock_t * in_fifo_write;
	spinlock_t * ring_pointer_write;
	spinlock_t * ring_pointer_read;

	printk(KERN_INFO"[pci_open]: gogo\n");

	in_fifo_read = (spinlock_t *)kmalloc(sizeof(spinlock_t), GFP_KERNEL);
	spin_lock_init(in_fifo_read);

	in_fifo_write = (spinlock_t *)kmalloc(sizeof(spinlock_t), GFP_KERNEL);
	spin_lock_init(in_fifo_write);

	ring_pointer_write = (spinlock_t *)kmalloc(sizeof(spinlock_t), GFP_KERNEL);
	spin_lock_init(ring_pointer_write);

	ring_pointer_read = (spinlock_t *)kmalloc(sizeof(spinlock_t), GFP_KERNEL);
	spin_lock_init(ring_pointer_read);

	start_time = (struct timespec *)kmalloc(sizeof(struct timespec), GFP_KERNEL);
	stop_time = (struct timespec *)kmalloc(sizeof(struct timespec), GFP_KERNEL);

	atomic_poll = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);
	wtk = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);
	wth = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);
	write_ring_buf_full = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);
	write_ring_buf_locked = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);

	rfu = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);
	rfh = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);
	read_ring_buf_full = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);
	read_ring_buf_locked = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);

	pci_write_q = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);
	in_read_fifo_count = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);
	in_write_fifo_count = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);
	mmap_count = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);


	atomic_set(atomic_poll, 0);
	atomic_set(wth, 0);
	atomic_set(wtk, 0);
	atomic_set(write_ring_buf_full, 0);
	atomic_set(write_ring_buf_locked, 0);

	atomic_set(rfh, 0);
	atomic_set(rfu, 0);
	atomic_set(read_ring_buf_full, 0);
	atomic_set(read_ring_buf_locked, 0);

	atomic_set(pci_write_q, 0);
	atomic_set(in_read_fifo_count, 0);
	atomic_set(in_write_fifo_count, 0);
	atomic_set(mmap_count, 0);

	//interrupt_count = kmalloc(sizeof(int), GFP_KERNEL);

	s = (struct file_desc *)kmalloc(sizeof(struct file_desc), GFP_KERNEL);

	s->svd = svd_global;

	s->minor = MINOR(inode->i_rdev);
	s->f_flags = filep->f_flags;
	printk(KERN_INFO"[pci_open]: f_flags 0x%x\n", filep->f_flags);

	s->axi_addr = -1;
	s->axi_addr_ctl = -1;
	s->mode = 0;
	s->int_num = 100;
	s->master_num = 0;
	s->interrupt_vec = 0;
	s->has_interrupt_vec = 0;
	s->axi_fifo_rlr = 0;
	s->axi_fifo_rdfo = 0;
	s->read_header_size = 0;
	s->keyhole_config = 0;
	s->dma_offset_read = -1;
	s->dma_offset_write = -1;
	s->dma_read_addr = NULL;
	s->dma_write_addr = NULL;
	s->dma_size = dma_file_size;
	s->file_size = 4096;	//default to 4K
	s->tx_bytes = 0;
	s->rx_bytes = 0;
	s->start_time = start_time;
	s->stop_time = stop_time;
	s->start_flag = 0;
	s->stop_flag = 0;
	s->cdma_attempt = 0;
	s->ip_not_ready = 0;
	s->atomic_poll = atomic_poll;
	s->set_dma_flag = 0;
	s->wth = wth;
	s->wtk = wtk;
	s->rfh = rfh;
	s->rfu = rfu;
	s->write_ring_buf_full = write_ring_buf_full;
	s->write_ring_buf_locked = write_ring_buf_locked;

	s->read_ring_buf_full = read_ring_buf_full;
	s->read_ring_buf_locked = read_ring_buf_locked;

	s->ring_pointer_write = ring_pointer_write;
	s->ring_pointer_read = ring_pointer_read;
	s->in_fifo_read = in_fifo_read;
	s->in_fifo_read_flag = 0;
	s->in_fifo_write = in_fifo_write;
	s->in_fifo_write_flag = 0;
	s->pci_write_q = pci_write_q;

	s->read_buffer = NULL;
	s->write_buffer = NULL;

	s->in_write_fifo_count = in_write_fifo_count;
	s->in_read_fifo_count = in_read_fifo_count;
	s->mmap_count = mmap_count;

	s->tx_dest = 0x2;
	s->rx_dest = 0x0;

	s->file_open = true;

	s->xdma_dev = lro_global;


	init_waitqueue_head(&s->poll_wq);
	verbose_printk(KERN_INFO"[pci_%x_open]: minor number %x detected\n", s->minor, s->minor);

	/*set the private data field of the file pointer for file op use*/
	filep->private_data = s;
	verbose_printk(KERN_INFO"[pci_%x_open]: file open complete.\n", s->minor);
	return SUCCESS;
}

int pci_release(struct inode *inode, struct file *filep)
{
	/*release is only called on the final close() of a file
	 * This is so that the file can be open multiple times.
	 * Might need to find a way to kfree() the other open
	 * instances of the file. Or restrict that each file
	 * only be open once. */
	int in_read_fifo_count, in_write_fifo_count, attempts = 1000;
	struct file_desc* file_desc;

	//printk(KERN_INFO"[pci_release]: Attempting to close file minor number: %d\n", file_desc->minor);

	file_desc = filep->private_data;

	if(file_desc->mode == AXI_STREAM_FIFO || file_desc->mode == AXI_STREAM_PACKET)	{
		//turn off interupts from fifo
		axi_stream_fifo_deinit(file_desc);
	}


	file_desc->file_open = false;


	//reset ring_buffers incase anything was help up on them being too full
	atomic_set(file_desc->rfh, 0);
	atomic_set(file_desc->rfu, 0);
	atomic_set(file_desc->wtk, 0);
	atomic_set(file_desc->wth, 0);
	atomic_set(file_desc->write_ring_buf_full, 0);
	atomic_set(file_desc->read_ring_buf_full, 0);
	atomic_set(file_desc->write_ring_buf_locked, 0);
	atomic_set(file_desc->read_ring_buf_locked, 0);

	in_read_fifo_count = atomic_read(file_desc->in_read_fifo_count);
	//wake up the read threads until we clear our file_desc out of them so we can free the memory assiociated with them.
	while(attempts-- > 0 && in_read_fifo_count != 0){
		verbose_printk(KERN_INFO"[pci_%x_release]: in_read_fifo_count(%d) is not zero! waking thread\n", file_desc->minor, in_read_fifo_count);
		atomic_set(&svd_global->thread_q_read, 1);
		wake_up_interruptible(&file_desc->svd->thread_q_head_read);
		schedule();					//think we want to schedule here to give thread time to do work
		in_read_fifo_count = atomic_read(file_desc->in_read_fifo_count);
	}
	//remove items from the in_read_fifo
	while(atomic_read(file_desc->in_read_fifo_count) > 0) {
		atomic_dec(file_desc->in_read_fifo_count);
	}


	in_write_fifo_count = atomic_read(file_desc->in_write_fifo_count);
	attempts = 1000;
	//wake up the write threads until we clear our file_desc out of them so we can free the memory assiociated with them.
	while(attempts-- > 0 && in_write_fifo_count != 0){
		verbose_printk(KERN_INFO"[pci_%x_release]: in_write_fifo_count(%d) is not zero! waking thread\n", file_desc->minor, in_write_fifo_count);
		atomic_set(&svd_global->thread_q_write, 1);
		wake_up_interruptible(&file_desc->svd->thread_q_head_write);
		schedule();					//think we want to schedule here to give thread time to do work
		in_write_fifo_count = atomic_read(file_desc->in_write_fifo_count);
	}
	//remove items from the in_read_fifo
	while(atomic_read(file_desc->in_write_fifo_count) > 0) {
		atomic_dec(file_desc->in_write_fifo_count);
	}

	dma_file_deinit(file_desc, file_desc->dma_size);




	kfree((const void*)file_desc->start_time);
	kfree((const void*)file_desc->stop_time);

	kfree((const void*)file_desc->in_fifo_read);
	kfree((const void*)file_desc->in_fifo_write);
	kfree((const void*)file_desc->ring_pointer_write);
	kfree((const void*)file_desc->ring_pointer_read);
	kfree((const void*)file_desc->atomic_poll);
	kfree((const void*)file_desc->wtk);
	kfree((const void*)file_desc->wth);
	kfree((const void*)file_desc->write_ring_buf_full);
	kfree((const void*)file_desc->write_ring_buf_locked);

	kfree((const void*)file_desc->rfu);
	kfree((const void*)file_desc->rfh);
	kfree((const void*)file_desc->read_ring_buf_full);
	kfree((const void*)file_desc->read_ring_buf_locked);

	kfree((const void*)file_desc->pci_write_q);
	kfree((const void*)file_desc->in_read_fifo_count);
	kfree((const void*)file_desc->in_write_fifo_count);
	kfree((const void*)file_desc->mmap_count);

	verbose_printk(KERN_INFO"[pci_%x_release]: ************************************************************************\n", file_desc->minor);
	verbose_printk(KERN_INFO"[pci_%x_release]: *************************** File Close *********************************\n", file_desc->minor);
	verbose_printk(KERN_INFO"[pci_%x_release]: \t\tTotal Bytes read : %d\n", file_desc->minor, file_desc->rx_bytes);
	verbose_printk(KERN_INFO"[pci_%x_release]: \t\tTotal Bytes wrote : %d\n", file_desc->minor, file_desc->tx_bytes);
	verbose_printk(KERN_INFO"[pci_%x_release]: ************************************************************************\n", file_desc->minor);

	kfree((const void*)file_desc);

	return SUCCESS;
}


/*item for file_operations: unlocked ioctl*/
long pci_unlocked_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct file_desc *file_desc;
	static int master_count = 0;
	u32 bit_vec;
	u64 arg_loc;
	//	u32 kern_reg;
	int interrupt_num;
	struct statistics * stats; //= kmalloc(sizeof(struct statistics), GFP_KERNEL);
	struct timespec diff;
	int minor;
	void * buffer;

	file_desc = filep->private_data;
	minor = file_desc->minor;

	if(arg == 0){
		arg_loc = 0;
	}
	else if( copy_from_user(&arg_loc, argp, sizeof(u64)) ) {
		printk(KERN_INFO"[pci_%x_ioctl]: !!!!!!!!ERROR copy_to_user\n", minor);
		return ERROR;
	}

	if(pcie_use_xdma == 1){
		buffer = file_desc->read_buffer;
	}
	else{
		buffer = file_desc->dma_read_addr;
	}

	verbose_printk(KERN_INFO"[pci_%x_ioctl]: Entering IOCTL with command: %d and arg %llx\n", minor, cmd, arg_loc);


	switch(cmd){

		case WRITE_REG:
			verbose_printk(KERN_INFO"[pci_%x_ioctl]: Writing data at offset:%08llx\n", minor, arg_loc);
			if( direct_write(file_desc->axi_addr + arg_loc, buffer+arg_loc, 4, NORMAL_WRITE) ) {
				printk(KERN_INFO"[pci_%x_ioctl]: !!!!!!!!ERROR direct_read\n", minor);
				return ERROR;
			}

		break;

		case READ_REG:
			verbose_printk(KERN_INFO"[pci_%x_ioctl]: Reading data at offset:%08llx\n", minor, arg_loc);
			if( direct_read(file_desc->axi_addr + arg_loc, buffer+arg_loc, 4, NORMAL_READ) ) {
				printk(KERN_INFO"[pci_%x_ioctl]: !!!!!!!!ERROR direct_read\n", minor);
				return ERROR;
			}

		break;

		case SET_AXI_DEVICE:
			verbose_printk(KERN_INFO"[pci_%x_ioctl]: Setting Peripheral AXI Address: 0x%llx\n", minor, arg_loc);
			file_desc->axi_addr = arg_loc&(0xFFFFFFFFFFFFFFFF);
			break;

		case SET_AXI_CTL_DEVICE:
			verbose_printk(KERN_INFO"[pci_%x_ioctl]: Setting Peripheral control AXI Address: 0x%llx\n", minor, arg_loc);
			file_desc->axi_addr_ctl = arg_loc&(0xFFFFFFFFFFFFFFFF);
			break;

		case SET_AXI_CDMA:
			//			cdma_init(arg_loc);
			break;


		case SET_AXI_PCIE_CTL:
			//			pcie_ctl_init(arg_loc);
			break;

		case SET_AXI_PCIE_M:
			//			pcie_m_init(arg_loc);
			break;

			/*This is the Interrupt controller that will MUX in all interrupts from axi masters and produce
			 *one interrupt output to the the PCIe controller. This is because we want to use one msi vector. */
		case SET_AXI_INT_CTRL:
			//			axi_intc_init(arg_loc);
			break;

		case SET_DMA_SIZE:
			if( dma_file_init(file_desc, svd_global->dma_buffer_base, arg_loc, file_desc->f_flags, pcie_use_xdma) ) {
				printk(KERN_INFO"[pci_%x_ioctl]: \t!!!!set dma FAILURE!!!!.\n", minor);
				return ERROR;
			}

			break;

		case RESET_DMA_ALLOC:
			svd_global->dma_current_offset = 0;	//we want to leave the first 4k for the kernel to use internally.
			verbose_printk(KERN_INFO"[pci_%x_ioctl]: Reset the DMA Allocation\n", minor);
			break;

		case SET_INTERRUPT:
			verbose_printk(KERN_INFO"[pci_%x_ioctl]: Setting device as an Interrupt source with vector:%llx\n", minor, arg_loc);

			/*Store the Interrupt Vector*/
			file_desc->interrupt_vec = (u32)arg_loc;
			file_desc->has_interrupt_vec = 1;
			interrupt_num = vec2num((u32)arg_loc);
			file_desc->int_num = interrupt_num;
			verbose_printk(KERN_INFO"[pci_%x_ioctl]: Interrupt Number:%d\n", minor, interrupt_num);

			file_desc_arr[interrupt_num] = file_desc;

			break;

		case SET_FILE_SIZE:
			file_desc->file_size = ((loff_t)arg_loc & 0xffffffffffffffff);
			verbose_printk(KERN_INFO"[pci_%x_ioctl]: Setting device file size:%llu\n", minor, file_desc->file_size);

			/*initialize the DMA for the file*/
			if(file_desc->set_dma_flag == 0) {
				if( dma_file_init(file_desc, svd_global->dma_buffer_base, file_desc->dma_size, file_desc->f_flags, pcie_use_xdma) ) {
					printk(KERN_INFO"[pci_%x_ioctl]: \t!!!! DMA init FAILURE!!!!\n", minor);
					return ERROR;
				 }
			}

			verbose_printk(KERN_INFO"[pci_%x_ioctl]: Returning dma size:%zu\n", minor, file_desc->dma_size);

			if( copy_to_user((void *)arg, &(file_desc->dma_size), sizeof(size_t)) ) {
				verbose_printk(KERN_INFO"[pci_%x_ioctl]: !!!!!!!!ERROR copy_to_user\n", minor);
				return ERROR;
			}
			break;


		case SET_CDMA_KEYHOLE_WRITE:
			bit_vec = 0x00000020;	//the bit for KEYHOLE WRITE

			if(arg_loc>0) {	//We are ENABLING keyhole write
				verbose_printk(KERN_INFO"[pci_%x_ioctl]: Setting the CDMA Keyhole WRITE as ENABLED\n", minor);
				if( cdma_config_set(bit_vec, 1, 1) ) {	//value of one means we want to SET the register
					printk(KERN_INFO"[pci_%x_ioctl]: !!!!!!!!ERROR reading from AXI Streaming FIFO control interface\n", minor);
					return ERROR;
				}
			} else { //We are disabling keyhole write
				verbose_printk(KERN_INFO"[pci_%x_ioctl]: Setting the CDMA Keyhole WRITE as DISABLED\n", minor);
				if( cdma_config_set(bit_vec, 0, 1) ) {	//value of 0 means we want to UNSET the register
					printk(KERN_INFO"[pci_%x_ioctl]: !!!!!!!!ERROR reading from AXI Streaming FIFO control interface\n", minor);
					return ERROR;
				}
			}
			break;

		case SET_CDMA_KEYHOLE_READ:
			bit_vec = 0x00000010;	//the bit for KEYHOLE READ

			if(arg_loc>0) { //We are ENABLING keyhole read
				verbose_printk(KERN_INFO"[pci_%x_ioctl]: Setting the CDMA Keyhole READ as ENABLED\n", minor);
				if( cdma_config_set(bit_vec, 1, 1) ) {	//value of one means we want to SET the register
					printk(KERN_INFO"[pci_%x_ioctl]: !!!!!!!!ERROR cdma_config_set\n", minor);
					return ERROR;
				}
			} else {//We are disabling keyhole read
				verbose_printk(KERN_INFO"[pci_%x_ioctl]: Setting the CDMA Keyhole READ as DISABLED\n", minor);
				if( cdma_config_set(bit_vec, 0, 1) ) {	//value of 0 means we want to UNSET the register
					printk(KERN_INFO"[pci_%x_ioctl]: !!!!!!!!ERROR cdma_config_set\n", minor);
					return ERROR;
				}
			}
			break;

		case GET_FILE_STATISTICS:
			stats = (struct statistics *)kmalloc(sizeof(struct statistics), GFP_KERNEL);
			if( copy_from_user(stats, (void *)arg, sizeof(struct statistics)) ) {	//value of 0 means we want to UNSET the register
				printk(KERN_INFO"[pci_%x_ioctl]: !!!!!!!!ERROR copy_from_user\n", minor);
				return ERROR;
			}

			//statistics = (struct statistics *)arg;
			stats->tx_bytes = file_desc->tx_bytes;
			stats->rx_bytes = file_desc->rx_bytes;
			stats->cdma_attempt = file_desc->cdma_attempt;
			stats->ip_not_ready = file_desc->ip_not_ready;
			stats->dma_usage_cnt = file_desc->svd->dma_usage_cnt;
			file_desc->svd->dma_usage_cnt = 0;

			if(file_desc->stop_flag == 1) {
				file_desc->stop_flag = 0;
				diff = timespec_sub(*(file_desc->stop_time), *(file_desc->start_time));
				stats->seconds = (unsigned long)diff.tv_sec;
				stats->ns = (unsigned long)diff.tv_nsec;
			}

			/*reset the counters*/
			file_desc->tx_bytes = 0;
			file_desc->rx_bytes = 0;
			file_desc->ip_not_ready = 0;

			if( copy_to_user((void *)arg, stats, sizeof(struct statistics)) ) {
				printk(KERN_INFO"[pci_%x_ioctl]: !!!!!!!!ERROR copy_to_user\n", minor);
				return ERROR;
			}

			break;

		case GET_DRIVER_STATISTICS:
			stats = (struct statistics *)kmalloc(sizeof(struct statistics), GFP_KERNEL);
			if( copy_from_user(stats, (void *)arg, sizeof(struct statistics)) ) {
				printk(KERN_INFO"[pci_%x_ioctl]: !!!!!!!!ERROR copy_from_user\n", minor);
				return ERROR;
			}
			//	statistics = (struct statistics *)arg;
			stats->tx_bytes = atomic_read(&svd_global->driver_tx_bytes);
			stats->rx_bytes = atomic_read(&svd_global->driver_rx_bytes);

			if(atomic_read(&svd_global->driver_stop_flag) == 1) {
				atomic_set(&svd_global->driver_stop_flag, 0);
				diff = timespec_sub((svd_global->driver_stop_time), (svd_global->driver_start_time));
				stats->seconds = (unsigned long)diff.tv_sec;
				stats->ns = (unsigned long)diff.tv_nsec;
			}

			/*reset the counters*/
			atomic_set(&svd_global->driver_tx_bytes, 0);
			atomic_set(&svd_global->driver_rx_bytes, 0);
			//driver_ip_not_ready = 0;
			if( copy_to_user((void *)arg, stats, sizeof(struct statistics)) ) {
				printk(KERN_INFO"[pci_%x_ioctl]: !!!!!!!!ERROR copy_to_user\n", minor);
				return ERROR;
			}
			break;

		case START_FILE_TIMER:
			file_desc->start_flag = 1;
			break;

		case STOP_FILE_TIMER:
			file_desc->stop_flag = 1;
			break;

		case START_DRIVER_TIMER:
			atomic_set(&svd_global->driver_start_flag, 1);
			break;

		case STOP_DRIVER_TIMER:
			atomic_set(&svd_global->driver_stop_flag, 1);
			break;

		case SET_MODE:
			verbose_printk(KERN_INFO"[pci_%x_ioctl]: Setting the mode of the peripheral\n", minor);
			file_desc->mode = (u32)arg_loc;

			switch(file_desc->mode){

				case SLAVE:
					printk(KERN_INFO"[pci_%x_ioctl]: Setting the mode of the peripheral to SLAVE\n", minor);
					break;

				case MASTER:
					printk(KERN_INFO"[pci_%x_ioctl]: Setting the mode of the peripheral to MASTER\n", minor);
					/*write master_count as master_num to file descriptor to use in referencing its memory address*/
					file_desc->master_num = master_count;
					break;
				case CONTROL:
					printk(KERN_INFO"[pci_%x_ioctl]: Setting the mode of the peripheral to CONTROL\n", minor);
					/*write master_count as master_num to file descriptor to use in referencing its memory address*/
					break;

				case AXI_STREAM_PACKET :
				case AXI_STREAM_FIFO :
					printk(KERN_INFO"[pci_%x_ioctl]: Setting the mode of the peripheral to AXI_STREAM_FIFO\n", minor);

					/*Initialize the AXI_STREAM_FIFO*/
					/*for now we will initialize all as interrupting for read*/
					//					/*check to see if the AXI addresses have been set*/
					if((file_desc->axi_addr == -1) | (file_desc->axi_addr_ctl == -1)) {
						printk(KERN_INFO"[pci_%x_ioctl]: !!!!!!!!ERROR: axi addresses of AXI STREAM FIFO not set\n", minor);
						printk(KERN_INFO"[pci_%x_ioctl]: \tset the AXI addresses then set mode again\n", minor);
						return ERROR;
					} else {
						verbose_printk(KERN_INFO"[pci_%x_ioctl]: Initializing the FIFO and setting registers\n", minor);
						/*set the ring buff full*/
						atomic_set(file_desc->write_ring_buf_full, 0);
						atomic_set(file_desc->read_ring_buf_full, 0);
						atomic_set(file_desc->write_ring_buf_locked, 0);
						atomic_set(file_desc->read_ring_buf_locked, 0);
						/*set the pointer defaults*/
						verbose_printk(KERN_INFO"[pci_%x_ioctl]: read ring_buffer: RFU : %d RFH %d\n", minor, 0, 0);
						atomic_set(file_desc->wtk, 0);
						atomic_set(file_desc->wtk, 0);
						verbose_printk(KERN_INFO"[pci_%x_ioctl]: write ring_buffer: WTH: %d WTK: %d\n", minor, 0, 0);
						atomic_set(file_desc->rfh, 0);
						atomic_set(file_desc->rfu, 0);

						if( axi_stream_fifo_init(file_desc) ) {
							return ERROR;
						}

					}
					break;

					default:printk(KERN_INFO"[pci_%x_ioctl]: !!!!!!!!ERROR, improper mode type!\n", minor);
				}
			break;

		case FILE_ACTIVATE:
			verbose_printk(KERN_INFO"[pci_%x_ioctl]: Activate file!\n", minor);
			break;

		case FILE_DEACTIVATE:
			verbose_printk(KERN_INFO"[pci_%x_ioctl]: Deactivate file!\n", minor);
			break;

		default:printk(KERN_INFO"[pci_%x_ioctl]: !!!!!!!!ERROR, no command found.\n", minor);
	}
	return 0;
}

loff_t pci_llseek( struct file *filep, loff_t off, int whence)
{
	struct file_desc * file_desc;
	loff_t newpos;

	file_desc = filep->private_data;

	switch(whence) {
		case 0: /*SEEK SET*/
			newpos = off;
			break;

		case 1: /*SEEK_CUR*/
			newpos = filep->f_pos + off;
			break;

			//		case 2: /*SEEK_END */
			//			newpos = dev->size + off;
			//			break;

		default: /* cant happen */
			return -EINVAL;
	}
	verbose_llseek_printk(KERN_INFO"[pci_%x_llseek]: attempted seek %lld\n", file_desc->minor, newpos);
	if(newpos < 0) return -EINVAL;
	filep->f_pos = newpos;
	return newpos;
}

unsigned pci_poll(struct file *filep, poll_table * pwait)
{
	struct file_desc * file_desc;
	int rfh;
	int rfu;
	int full;
	int d2r;
	unsigned int mask = 0;

	file_desc = filep->private_data;

	rfh = atomic_read(file_desc->rfh);
	rfu = atomic_read(file_desc->rfu);
	full = atomic_read(file_desc->write_ring_buf_full);


	very_verbose_poll_printk(KERN_INFO"[pci_%x_poll]: Poll() has been entered!\n", file_desc->minor);
	/*Register the poll table with the peripheral wait queue
	 *so that every wake_up on the wait queue will see if the
	 *peripheral has data*/

	//if data in the ring buffer
	//if(data_in_buffer == 0) {		//if there is no data to read


	poll_wait(filep, &file_desc->poll_wq, pwait);
	very_verbose_poll_printk(KERN_INFO"[pci_%x_poll]: poll_wait done!\n", file_desc->minor);
	d2r = data_in_buffer(rfh, rfu, full, file_desc->dma_size);

	if(atomic_read(file_desc->atomic_poll) || (d2r != 0) ) {
		verbose_poll_printk(KERN_INFO"[pci_%x_poll]: Interrupting Peripheral Matched or Data in ring buffer(d2r:%x)!\n", file_desc->minor, d2r);
		/*reset the has_data flag*/
		atomic_set(file_desc->atomic_poll, 0);
		mask |= POLLIN;
	}

	very_verbose_poll_printk(KERN_INFO"[pci_%x_poll]: Leaving Poll(0x%08x)\n", file_desc->minor, mask);
	return mask;

}

ssize_t pci_write(struct file *filep, const char __user *buf, size_t count, loff_t *f_pos)
{
	u64 axi_dest;
	struct file_desc * file_desc;
	size_t bytes_written = 0;
	size_t partial_count;
	int transfer_type;
	size_t mmap_addr_offset;						//offset of the buffer from the begining of the mmap
	int minor;
	void* buffer;
	int mmap_count;

	//this gets the minor number of the calling file so we can map the correct AXI address to write to
	file_desc = filep->private_data;
	minor = file_desc->minor;

	bytes_written = 0;

	if(pcie_use_xdma == 1){
		buffer = file_desc->write_buffer;
	}
	else{
		buffer = file_desc->dma_write_addr;
	}


	verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: ************************************************************************\n", minor);
	verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: ******************** WRITE TRANSACTION BEGIN **************************\n", minor);
	switch(file_desc->mode){
		case AXI_STREAM_FIFO :
			verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: \tAttempting to transfer %zu bytes : mode AXI_STREAM_FIFO \n", minor, count);
			break;

		case AXI_STREAM_PACKET :
			verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: \tAttempting to transfer %zu bytes : mode AXI_STREAM_PACKET \n", minor, count);
			break;

		case MASTER :
		case SLAVE :
			verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: \tAttempting to transfer %zu bytes : mode DIRECT \n", minor, count);
			break;

		case CONTROL :
			verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: \tAttempting to transfer %zu bytes : mode CONTROL \n", minor, count);
			break;

		default :
			verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: \tAttempting to transfer %zu bytes : mode UNKNOWN \n", minor, count);

	}
	verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: ************************************************************************\n", minor);
	verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: WRITE file struct offset: %llx\n", minor, filep->f_pos);
	verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: WRITE offset param: %llx\n", minor, *f_pos);

	if(count == 0) {
		printk(KERN_INFO"[pci_%x_write]: !!!!!!!!ERROR count == 0 !\n", minor);
		return ERROR;
	}

	if(file_desc->set_dma_flag == 0) {
		if( dma_file_init(file_desc, svd_global->dma_buffer_base, file_desc->dma_size, file_desc->f_flags, pcie_use_xdma) ) {
			printk(KERN_INFO"[pci_%x_write]: \t!!!! DMA init FAILURE!!!!\n", minor);
			return ERROR;
			 }
		printk(KERN_INFO"[pci_%x_write]: Warning - Set the DMA file size to default value %d. IOCTL SET_FILE_SIZE was never called.\n", minor, (int)file_desc->file_size);
		file_desc->set_dma_flag = 1;
	}

	/*Start timers for statistics*/
	if(file_desc->start_flag == 1) {
		getnstimeofday(file_desc->start_time);
		file_desc->start_flag = 0;
	}

	if(atomic_read(&svd_global->driver_start_flag) == 1) {
		getnstimeofday(&svd_global->driver_start_time);
		atomic_set(&svd_global->driver_start_flag, 0);
	}

	switch(file_desc->mode) {
		case AXI_STREAM_FIFO :
		case AXI_STREAM_PACKET :
			/*Stay until requested transmission is complete*/
			while (bytes_written < count && file_desc->file_open) {

				if(count > file_desc->dma_size) {
					printk(KERN_INFO"[pci_%x_write]: count > dma_size \n", minor);
					return ERROR;
				}

				while(copy_to_ring_buffer(file_desc, (void *)buf, count, buffer) == 0 ) {
					//wake up write thread
					atomic_set(&svd_global->thread_q_write, 1);
					wake_up_interruptible(&file_desc->svd->thread_q_head_write);
					verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: not enough room in the the write ring buffer, waking up write thread a\n", file_desc->minor);

					//verbose_pci_write_printk(KERN_INFO"[pci_%x_write]:: pci_write is going to sleep ZzZzZzZzZzZzZz, room in buffer: %d\n", file_desc->minor, room_in_buffer(wtk, wth, full, file_desc->dma_size));
					if( wait_event_interruptible(file_desc->svd->pci_write_head, atomic_read(file_desc->pci_write_q) == 1) ) {
						printk(KERN_INFO"[pci_%x_write]: !!!!!!!!ERROR wait_event_interruptible\n", file_desc->minor);
						return ERROR;
					}
					atomic_set(file_desc->pci_write_q, 0);
					verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: woke up the sleeping pci_write function!!\n", file_desc->minor);
				}

				//write the file_desc to the write FIFO
				if(atomic_read(file_desc->in_write_fifo_count) == 0) {
					//debug message
					if(kfifo_len(&file_desc->svd->write_fifo) > 4) {
						verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: kfifo write stored elements: %d\n", minor, kfifo_len(&file_desc->svd->write_fifo));
					}

					if(!kfifo_is_full(&file_desc->svd->write_fifo) && file_desc->file_open) {
						atomic_inc(file_desc->in_write_fifo_count);
						kfifo_in_spinlocked(&file_desc->svd->write_fifo, &file_desc, 1, &file_desc->svd->fifo_lock_write);
					}
					else {
						verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: write kfifo is full, not writing mod desc\n", minor);
					}
				}

				//wake up write thread
				atomic_set(&svd_global->thread_q_write, 1);
				wake_up_interruptible(&file_desc->svd->thread_q_head_write);
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: waking up write thread b\n", minor);

				partial_count = count;

				bytes_written = bytes_written + partial_count;
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: Wrote %zu bytes in this pass.\n", minor, partial_count);
			}
			break;

		case CONTROL:
		case SLAVE:
		case MASTER:						//should master be supported?

			mmap_count = atomic_read(file_desc->mmap_count);
			axi_dest = file_desc->axi_addr + *f_pos;
			mmap_addr_offset = ((size_t)buf - file_desc->mmap_start_addr);		//(difference between vitrual write address and given buffer)

			if(file_desc->keyhole_config & 0x1)
				transfer_type = KEYHOLE_WRITE;
			else
				transfer_type = NORMAL_WRITE;

			//check to make sure count is not larger than file_size
			if(transfer_type == NORMAL_WRITE && (count + *f_pos) > file_desc->file_size) {
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: File size is: 0x%llx is large than transfer size: 0x%zx bytes + offset: 0x%llxx\n", minor, file_desc->file_size, count, *f_pos);
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: Setting Transfer size: 0x%llx bytes\n", minor, file_desc->file_size);
				count = (size_t)(file_desc->file_size - *f_pos);

			}
			//if we have mmap and the buf address is in the mmap range
			if(mmap_count > 0 && mmap_addr_offset >= 0 && mmap_addr_offset <= file_desc->dma_size){			//todo make sure it is less than the size of memory map instead of dma size
				//don't do copy from user copy directly from buffer given because it is mmap'd
				buffer = file_desc->dma_read_addr;		 //use this buffer for MMAP

				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: writing peripheral with starting value(mmap): 0x%p\n", minor, (void *)(buffer));
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: dma buffer offset write value: %llx\n", minor, (u64)file_desc->dma_offset_write);
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: writing peripheral using a transfer_type: 0x%x, offset: %llx\n", minor, transfer_type, *f_pos);

				if(file_desc->mode == CONTROL){
					verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: direct_write AXI Address(mmap): 0x%llx, buf: 0x%p + 0x%zx, len: 0x%zx\n", minor, axi_dest, buffer, mmap_addr_offset, count);
					//for mmap the data is really in the read buffer only, so use read_addr and offset_read for now
					if( direct_write(axi_dest, buffer + mmap_addr_offset, count, transfer_type) ) { //unsuccessful CDMA transmission
						printk(KERN_INFO"[pci_%x_write]: !!!!!!!!ERROR writing to User Peripheral\n", minor);
						return ERROR;
					}
				}
				else {
					verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: dma_transfer AXI Address(mmap): 0x%llx, buf: 0x%p + 0x%zx, len: 0x%zx\n", minor, axi_dest, buffer, mmap_addr_offset, count);
					//for mmap the data is really in the read buffer only, so use read_addr and offset_read for now
					if( dma_transfer(file_desc, axi_dest, buffer + mmap_addr_offset, count, transfer_type, 0) ) {
						printk(KERN_INFO"[pci_%x_write]: !!!!!!!!ERROR writing to User Peripheral\n", minor);
						return ERROR;
					}
				}

				if(transfer_type == NORMAL_WRITE) {
					//*f_pos = *f_pos + count;
					if(*f_pos + count == file_desc->file_size) {
						//*f_pos = 0;
						verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: Resetting file pointer back to zero...\n", minor);
					} else if(*f_pos + count > file_desc->file_size) {
						printk(KERN_INFO"[pci_%x_write]: !!!!!!!!ERROR! Wrote past the file size. This should not have happened...\n", minor);
						printk(KERN_INFO"[pci_%x_write]: Resetting file pointer back to zero...\n", minor);
						//*f_pos = 0;
						return ERROR;
					}
					verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: updated file offset is: %llx\n", minor, *f_pos);
				}
			}

			else{
				//not mmap
				if(count > file_desc->dma_size) {
					verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: DMA Buffer size of: 0x%zx is not large enough for *remaining* transfer size: 0x%zx bytes\n", minor, file_desc->dma_size, count);
					count = file_desc->dma_size;
				}

				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: the amount of bytes being copied to kernel: %zu\n", minor, count);
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: copy_from_user (0x%p, 0x%p + 0x%zx, 0x%zx)\n" , minor, buffer, &buf, bytes_written, count);

				if( copy_from_user(buffer, (buf + bytes_written), count) ) {
					printk(KERN_INFO"[pci_%x_write]: !!!!!!!!ERROR copy_from_user\n", minor);
					return ERROR;
				}

				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: writing peripheral with starting value: 0x%p\n", minor, (void *)(buffer));
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: dma buffer offset write value: %llx\n", minor, (u64)file_desc->dma_offset_write);
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: writing peripheral using a transfer_type: 0x%x, offset: %llx\n", minor, transfer_type, *f_pos);

				if(file_desc->mode == CONTROL){
					verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: direct_write AXI Address: 0x%llx, buf: 0x%p, len: 0x%zx\n",	minor, axi_dest, buffer, count);
					if(direct_write(axi_dest, buffer, count, transfer_type)) {
						printk(KERN_INFO"[pci_%x_write]: !!!!!!!!ERROR writing to User Peripheral\n", minor);
						return ERROR;
					}
				}
				else {
					verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: dma_transfer AXI Address: 0x%llx, buf: 0x%p, len: 0x%zx\n",	minor, axi_dest, buffer, count);
					if(dma_transfer(file_desc, axi_dest, buffer, count, transfer_type, 0)) {
						printk(KERN_INFO"[pci_%x_write]: !!!!!!!!ERROR writing to User Peripheral\n", minor);
						return ERROR;
					}
				}

				if(transfer_type == NORMAL_WRITE) {
					//*f_pos = *f_pos + count;

					if(*f_pos + count == file_desc->file_size) {
						//*f_pos = 0;
						verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: Resetting file pointer back to zero...\n", minor);
					} else if(*f_pos + count > file_desc->file_size) {
						printk(KERN_INFO"[pci_%x_write]: !!!!!!!!ERROR! Wrote past the file size. This should not have happened...\n", minor);
						printk(KERN_INFO"[pci_%x_write]: Resetting file pointer back to zero...\n", minor);
						//*f_pos = 0;
						return ERROR;
					}
					verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: updated file offset is: %llx\n", minor, *f_pos);
				}
			}
			bytes_written = count;

			break;
		default:
			verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: mode not detected on write\n", minor);
	}


	//file statistics
	file_desc->tx_bytes = file_desc->tx_bytes + bytes_written;
	atomic_add(bytes_written, &svd_global->driver_tx_bytes);

	verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: total file tx byes: %d \n", minor, file_desc->tx_bytes);

	/*always update the stop_timer*/
	getnstimeofday(file_desc->stop_time);
	getnstimeofday(&svd_global->driver_stop_time);

	verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: ************************************************************************\n", minor);
	verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: ******************** WRITE TRANSACTION END **************************\n", minor);
	verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: \t\tWrote a total of %zu bytes in write call.\n", minor, bytes_written);
	verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: ************************************************************************\n", minor);
	return bytes_written;

}

/*
 * This function is called whenever a process which has already opened the
 * device file attempts to read from it.
 *
 * struct file *filep	 see include/linux/fs.h
 * char __user * buf		buffer holding the data to be written or the empty buffer
 * size_t count					size of the requested data transfer
 * loff_t * offset 			pointer to a "long offset type" object that indicates the file position the user is accessing
 *
 */
ssize_t pci_read(struct file *filep, char __user *buf, size_t count, loff_t *f_pos)
{
	u64 axi_dest;
	struct file_desc *file_desc;
	size_t bytes = 0;
	int transfer_type;
	int minor;
	void* buffer;
	int mmap_count;
	size_t mmap_addr_offset;

	file_desc = filep->private_data;
	minor = file_desc->minor;

	if(pcie_use_xdma == 1){
		buffer = file_desc->read_buffer;
	}
	else{
		buffer = file_desc->dma_read_addr;
	}


	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: ************************************************************************\n", minor);
	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: ******************** READ TRANSACTION BEGIN **************************\n", minor);
	switch(file_desc->mode){
		case AXI_STREAM_FIFO :
			verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: \tAttempting to transfer %zu bytes : mode AXI_STREAM_FIFO \n", minor, count);
			break;

		case AXI_STREAM_PACKET :
			verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: \tAttempting to transfer %zu bytes : mode AXI_STREAM_PACKET \n", minor, count);
			break;

		case MASTER :
		case SLAVE :
			verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: \tAttempting to transfer %zu bytes : mode DIRECT \n", minor, count);
			break;

		case CONTROL :
			verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: \tAttempting to transfer %zu bytes : mode CONTROL \n", minor, count);
			break;

		default :
			verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: \tAttempting to transfer %zu bytes : mode UNKNOWN \n", minor, count);

	}
	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: ************************************************************************\n", minor);

	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: Read file struct offset: %llx\n", minor, filep->f_pos);
	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: Read offset param: %llx , count requested %zd\n", minor, *f_pos, count);

	if(count == 0) {
	 printk(KERN_INFO"[pci_%x_read]: !!!!!!!!ERROR got invalid read count %zu\n", minor, count);
		return EINVAL;
	}

	if(file_desc->set_dma_flag == 0) {
	 if( dma_file_init(file_desc, svd_global->dma_buffer_base, file_desc->dma_size, file_desc->f_flags, pcie_use_xdma) ) {
			printk(KERN_INFO"[pci_%x_read]: \t!!!! DMA init FAILURE!!!!.\n", minor);
			return ERROR;
		}
		printk(KERN_INFO"[pci_%x_read]: Warning - Set the DMA file size to default value %lld. IOCTL SET_FILE_SIZE was never called.\n", minor, file_desc->file_size);
		file_desc->set_dma_flag = 1;
	}

	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: Attempting to read %zu bytes\n", minor, count);

	if(file_desc->start_flag == 1) {
		getnstimeofday(file_desc->start_time);
		file_desc->start_flag = 0;
	}

	if(atomic_read(&svd_global->driver_start_flag) == 1) {
		getnstimeofday(&svd_global->driver_start_time);
		atomic_set(&svd_global->driver_start_flag, 0);
	}

	if(count > file_desc->dma_size) {
		verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: Attempting to read more than the allocated DMA size of:%zu\n", minor, file_desc->dma_size);
		verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: readjusting the read amount to:%zu\n", minor, file_desc->dma_size);
		count = (size_t)file_desc->dma_size;
	}


	switch(file_desc->mode){

		case AXI_STREAM_FIFO:
		case AXI_STREAM_PACKET:
			// if AXI streaming fifo set with no interrupt then just read
			if(!file_desc->has_interrupt_vec) {
				if (count == 0) {
					printk(KERN_INFO"[pci_%x_read] illegal to read 0 bytes\n",minor);
					break;
				}
				bytes = axi_stream_fifo_d2r(file_desc);

				if(bytes == 0) {
					verbose_axi_fifo_write_printk(KERN_INFO"[pci_%x_read]: No data to read from axi stream fifo\n" , minor);
				} else {

					if(bytes <= count){
						bytes = axi_stream_fifo_read_direct(file_desc, (size_t)bytes, buffer, file_desc->svd->axi_pcie_m + file_desc->dma_offset_read, file_desc->dma_size);
						count = bytes;
					}
					else if(count < bytes) {
						bytes = axi_stream_fifo_read_direct(file_desc, (size_t)count, buffer, file_desc->svd->axi_pcie_m + file_desc->dma_offset_read, file_desc->dma_size);
					}

					verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: copy_to_user (0x%p, 0x%p, 0x%zx)\n" , minor, &buf, buffer, count);
					if( copy_to_user(buf, buffer, count) ) {
						printk(KERN_INFO"[pci_%x_read]: !!!!!!!!ERROR copy_to_user\n" , minor);
						return ERROR;
					}
					verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: Read %zd bytes from AXI FIFO\n" , minor , count);
				}
				break;
			} else {
				count = copy_from_ring_buffer(file_desc, buf, count, buffer);

				if(back_pressure && atomic_read(file_desc->read_ring_buf_locked)) {
					verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: freed up the locked ring buffer, waking up read thread.\n", file_desc->minor);

					// took data out, if full we can read now
					atomic_set(file_desc->read_ring_buf_locked, 0);
					atomic_set(&file_desc->svd->thread_q_read, 1);
					wake_up_interruptible(&file_desc->svd->thread_q_head_read);
				}
			}
			bytes = count;
			break;

		case CONTROL:
		case SLAVE:
			// Here we will decide whether to do a zero copy DMA, or to read directly from the peripheral
			axi_dest = file_desc->axi_addr + *f_pos;

			mmap_addr_offset = ((size_t)buf - file_desc->mmap_start_addr);		//(difference between vitrual write address and given buffer)

			if(file_desc->keyhole_config & 0x1)
				transfer_type = KEYHOLE_READ;
			else
				transfer_type = NORMAL_READ;

			if(count > file_desc->dma_size) {
				verbose_printk(KERN_INFO"[pci_%x_read]: DMA Buffer size of: 0x%zx is not large enough for *remaining* transfer size: 0x%zx bytes\n", minor, file_desc->dma_size, count);
				count = file_desc->dma_size;
			}

			/*Check to see if read will go past the boundary*/
			if(count + *f_pos > (size_t)file_desc->file_size) {
				verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: Read will overrun the file size because \n", minor);
				verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: (the current file offset + amount to read)->(%llu) > (%llu)->file_size\n", minor, count + *f_pos, file_desc->file_size);
				count = (size_t)(file_desc->file_size - *f_pos);
				verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: Adjusting to only reading %zu bytes to end of file!\n", minor, count);
			}

			mmap_count = atomic_read(file_desc->mmap_count);

			if(mmap_count > 0 && mmap_addr_offset >= 0 && mmap_addr_offset <= file_desc->dma_size){			//todo make sure it is less than the size of memory map instead of dma size
				//don't do copy to user copy directly to buffer given

				buffer = file_desc->dma_read_addr;		 //use this buffer for MMAP

				verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: reading peripheral using a transfer_type(mmap): 0x%x \n", minor, transfer_type);
				verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: current file offset is: %llu \n", minor, *f_pos);

				if(file_desc->mode == CONTROL) {
					verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: direct_read AXI Address(mmap): 0x%llx, buf: 0x%p + 0x%zx, len: 0x%zx\n", minor, axi_dest, buffer, mmap_addr_offset, count);
					if( direct_read(axi_dest, buffer + mmap_addr_offset, count, transfer_type) ) {
						printk(KERN_INFO"[pci_%x_read]: !!!!!!!!ERROR reading data from User Peripheral\n", minor);
						return ERROR;
					}
				}
				else {
					verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: dma_transfer AXI Address(mmap): 0x%llx, buf: 0x%p + 0x%zx, len: 0x%zx\n", minor, axi_dest, buffer, mmap_addr_offset, count);
					if(dma_transfer(file_desc, axi_dest, buffer + mmap_addr_offset, count, transfer_type, 0)) {
						printk(KERN_INFO"[pci_%x_read]: !!!!!!!!ERROR reading data from User Peripheral\n", minor);
						return ERROR;
					}
				}

				verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: first 4 bytes 0x%08x\n", minor,*(unsigned int *)(buffer + mmap_addr_offset));
			}
			else{
				//else read data and copy it to the user

				verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: reading peripheral using a transfer_type: 0x%x \n", minor, transfer_type);
				verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: current file offset is: %llu\n", minor, *f_pos);

				if(file_desc->mode == CONTROL) {
					verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: direct_read AXI Address: 0x%llx, buf: 0x%p, len: 0x%zx\n", minor, axi_dest, buffer, count);
					if( direct_read(axi_dest, buffer, count, transfer_type) ) {
						printk(KERN_INFO"[pci_%x_read]: !!!!!!!!ERROR reading data from User Peripheral\n", minor);
						return ERROR;
					}

				}
				else {
					verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: dma_transfer AXI Address: 0x%llx, buf: 0x%p, len: 0x%zx\n", minor, axi_dest, buffer, count);
					if(dma_transfer(file_desc, axi_dest, buffer, count, transfer_type, 0)) {
						printk(KERN_INFO"[pci_%x_read]: !!!!!!!!ERROR reading data from User Peripheral\n", minor);
						return ERROR;
					}
				}


				verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: first 4 bytes 0x%08x\n", minor,*(unsigned int *)buffer);
				verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: copy_to_user (0x%p, 0x%p, 0x%zx)\n" , minor, &buf, buffer, count);
				if( copy_to_user(buf, buffer, count) ) {
					printk(KERN_INFO"[pci_%x_read]: !!!!!!!!ERROR copy to user\n", minor);
					return ERROR;
				}
			}
			bytes = count;
			break;

		case MASTER:
			//Transfer buffer from kernel space to user space at the allocated DMA region
			verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: Transferred a Master write from kernel space to user space\n" , minor);
			// if( copy_to_user(buf, dma_master_buf[file_desc->master_num], count) ) {
			// 	printk(KERN_INFO"[pci_%x_read]: !!!!!!!!ERROR copy_to_user\n" , minor);
			// 	return ERROR;
			// }
			break;

		default:
			verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: mode not detected on read\n", minor);
	}

	/*always update the stop_timer*/
	if(bytes > 0) {
		getnstimeofday(file_desc->stop_time);
		getnstimeofday(&svd_global->driver_stop_time);
		file_desc->rx_bytes = file_desc->rx_bytes + bytes;
		atomic_add(bytes, &svd_global->driver_rx_bytes);
	}

	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: ************************************************************************\n", minor);
	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: ******************** READ TRANSACTION END **************************\n", minor);
	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: \t\tBytes read : %zd\n", minor, bytes);
	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: \t\tTotal Bytes read : %d\n", minor, file_desc->rx_bytes);
	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: ************************************************************************\n", minor);

	return bytes;
}



/*
 * This function is called whenever a process which has already opened the
 * device file attempts to read from it.
 *
 * struct file *filep	 see include/linux/fs.h
 * char __user * buf		buffer holding the data to be written or the empty buffer
 * size_t count					size of the requested data transfer
 * loff_t * offset 			pointer to a "long offset type" object that indicates the file position the user is accessing
 *
 */
int pci_mmap(struct file *filep, struct vm_area_struct *vma) {

	struct file_desc *file_desc;
	int minor;
	void* buffer;
	long length;
	int ret= 0;

	file_desc = filep->private_data;
	minor = file_desc->minor;
	length = vma->vm_end - vma->vm_start;

	// if(pcie_use_xdma == 1){
	//	 buffer = file_desc->read_buffer;
	// }
	// else{
		 buffer = file_desc->dma_read_addr;
	// }



	verbose_mmap_printk(KERN_INFO"[pci_%x_mmap]: ************************************************************************\n", minor);
	verbose_mmap_printk(KERN_INFO"[pci_%x_mmap]: ****************************** MMAP BEGIN ******************************\n", minor);


	switch(file_desc->mode){
		case AXI_STREAM_FIFO :
			verbose_mmap_printk(KERN_INFO"[pci_%x_mmap]: \tAttempting to mmap %zu bytes : mode AXI_STREAM_FIFO \n", minor, length);
			verbose_mmap_printk(KERN_INFO"[pci_%x_mmap]: ************************************************************************\n", minor);
			printk(KERN_INFO"[pci_%x_mmap]: ERROR Cannot MMAP AXI_STREAM_PACKET \n", minor);
			return ERROR;
			break;

		case AXI_STREAM_PACKET :
			verbose_mmap_printk(KERN_INFO"[pci_%x_mmap]: \tAttempting to mmap %zu bytes : mode AXI_STREAM_PACKET \n", minor, length);
			verbose_mmap_printk(KERN_INFO"[pci_%x_mmap]: ************************************************************************\n", minor);
			printk(KERN_INFO"[pci_%x_mmap]: ERROR Cannot MMAP AXI_STREAM_PACKET \n", minor);
			return ERROR;
			break;

		case MASTER :
		case SLAVE :
			verbose_mmap_printk(KERN_INFO"[pci_%x_mmap]: \tAttempting to mmap %zu bytes : mode DIRECT \n", minor, length);
			break;

		case CONTROL :
			verbose_mmap_printk(KERN_INFO"[pci_%x_mmap]: \tAttempting to mmap %zu bytes : mode CONTROL \n", minor, length);
			break;

		default :
			verbose_mmap_printk(KERN_INFO"[pci_%x_mmap]: \tAttempting to mmap %zu bytes : mode UNKNOWN \n", minor, length);

	}
	verbose_mmap_printk(KERN_INFO"[pci_%x_mmap]: ************************************************************************\n", minor);


	/* check length - do not allow larger mappings than the number of	pages allocated */
	if (length > file_desc->dma_size) {
		printk(KERN_INFO "[pci_%x_mmap]: ERROR requested length is larger than dma_size\n", minor);
		ret = -EIO;
	}

	if (vma->vm_pgoff != 0) {
		printk(KERN_INFO "[pci_%x_mmap]: WARNING: vm_pgoff != 0, vm_pg_off ignored \n", minor);
	}


// if(pcie_use_xdma == 1){
//	 /*
//	  * pages must not be cached as this would result in cache line sized
//	  * accesses to the end point
//	  */
//	 vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
// 	/*
// 	 * prevent touching the pages (byte access) for swap-in,
// 	 * and prevent the pages from being swapped out
// 	 */
// 	vma->vm_flags |= VMEM_FLAGS;
// 	/* make MMIO accessible to user space */
// 	rc = io_remap_pfn_range(vma, vma->vm_start, phys >> PAGE_SHIFT,
// 			vsize, vma->vm_page_prot);
// 	dbg_sg("vma=0x%p, vma->vm_start=0x%lx, phys=0x%lx, size=%lu = %d\n",
// 		vma, vma->vm_start, phys >> PAGE_SHIFT, vsize, rc);
//
// 	if (rc)
// 		return -EAGAIN;
// 	return 0;
//
// } else {
	//fix me, using the "read" buffer for everything, change to 1 buffer
		if((vma->vm_flags & VM_READ) == VM_READ || (vma->vm_flags & VM_WRITE) == VM_WRITE){
			verbose_mmap_printk(KERN_INFO "[pci_%x_mmap]: Using dma_mmap_coherent for read/write space\n", minor);
			ret = dma_mmap_coherent(NULL, vma, buffer, svd_global->dma_addr_base+file_desc->dma_offset_read, length);
			file_desc->mmap_start_addr = vma->vm_start;

		}
		else {
			printk(KERN_INFO "[pci_%x_mmap]: ERROR unknown flags\n", minor);
			return ERROR;
		}
	//}
	/* map the whole physically contiguous area in one piece */
	if (ret < 0) {
		printk(KERN_ERR "[pci_%x_mmap]: mmap_alloc: remap failed (%d)\n", minor, ret);
		return ERROR;
	}

	vma->vm_ops = &mmap_vm_ops;
	vma->vm_private_data = filep->private_data;
	mmap_open(vma);

	verbose_mmap_printk(KERN_INFO"[pci_%x_mmap]: ************************************************************************\n", minor);
	verbose_mmap_printk(KERN_INFO"[pci_%x_mmap]: ******************************* MMAP END *******************************\n", minor);
	verbose_mmap_printk(KERN_INFO"[pci_%x_mmap]: \t\tmmap length : 0x%zx\n", minor, length);
	verbose_mmap_printk(KERN_INFO"[pci_%x_mmap]: \t\tmmap address : 0x%lx\n", minor, vma->vm_start);
	verbose_mmap_printk(KERN_INFO"[pci_%x_mmap]: \t\tmmap end address : 0x%lx\n", minor, vma->vm_end);

	verbose_mmap_printk(KERN_INFO"[pci_%x_mmap]: ************************************************************************\n", minor);

	return ret;
}


/* keep track of how many times it is mmapped */
void mmap_open(struct vm_area_struct *vma)
{
	struct file_desc *file_desc;
	int minor;
	int mmap_count;

	file_desc = vma->vm_private_data;
	minor = file_desc->minor;

	atomic_inc(file_desc->mmap_count);
	mmap_count = atomic_read(file_desc->mmap_count);

	verbose_mmap_printk(KERN_INFO"[mmap_open[%x]]: mmap_open: %x \n", minor, mmap_count);
}

/* decrement reference cout */
void mmap_close(struct vm_area_struct *vma)
{
	struct file_desc *file_desc;
	int minor;
	int mmap_count;

	file_desc = vma->vm_private_data;
	minor = file_desc->minor;

	atomic_dec(file_desc->mmap_count);
	mmap_count = atomic_read(file_desc->mmap_count);

	verbose_mmap_printk(KERN_INFO"[mmap[%x]_close]: mmap_close: %x \n", minor, mmap_count);

}
