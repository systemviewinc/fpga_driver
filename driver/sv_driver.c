/**
 * System View Device Driver

 * @date     11/10/2015

 * @author   System View Inc.

 * @file     sv_driver.c

 * @brief         This file contains all the kernel char driver initialization/
 *				  exit routines as well as file operations such as read,write,ioctl
 *				  and interrupt service routine
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
#include <linux/time.h>
#include <linux/kthread.h>
#include <linux/kfifo.h>
#include <linux/spinlock.h>
#include "sv_driver.h"


/***********Set default values for insmod parameters***************************/
int device_id = 100;   /**< Insmod Parameter - PCIe specific */
int major = 241;/**< Insmod Parameter - Major number of Driver*/
uint cdma_address = 0xFFFFFFFF;/**< Insmod Parameter - the AXI Address of CDMA 1*/
uint cdma_2_address = 0xFFFFFFFF;/**< Insmod Parameter - the AXI Address of CDMA 2*/
int enable_cdma_2 = 0;/**< Insmod Parameter - The Enable to use CDMA 2*/
uint pcie_ctl_address = 0xFFFFFFFF;/**< Insmod Parameter - The AXI Address of the PCIe Control regs*/
uint pcie_m_address = 0xFFFFFFFF;/**< Insmod Parameter - AXI address of data transport slave address as viewed from CDMA, set to 0 (currently not used) */
uint int_ctlr_address = 0xFFFFFFFF;/**< Insmod Parameter - AXI Address of Interrupt Controller*/
int driver_type = PCI;/**< Insmod Parameter - Driver typem either PCIe or Platform*/
int dma_system_size = 4194304;/**< Insmod Parameter - Size of DMA Allocation, max and default is 4MB*/
int dma_file_size = 4096;/**< Insmod Parameter - Currently not used, the HW buffer size is now set on a file by file basis.*/
int dma_byte_width = 8;   /**< Insmod Parameter - This parameter is the data width of the CDMA. It is used to calculate the FIFO empty value.*/
int back_pressure = 0;/**< Insmod Parameter - This parameter sets whether the READ Ring Buffer should overwrite data or backpressure to HW.*/
uint axi2pcie_bar0_size = 0xFFFFFFFF;/**< Insmod Parameter - This parameter sets the PCIE2AXI bar size for address translation parameters.*/
uint vsi_reg_intf_addr = 0xFFFFFFFF;/**< Insmod Parameter - This parameter sets the PCIE2AXI bar size for address translation parameters.*/
uint interface_crc = 0;/**< Insmod Parameter - This parameter sets whether the READ Ring Buffer should overwrite data or backpressure to HW.*/
uint interface_crc_check = 0;/**< Insmod Parameter - This parameter sets whether the READ Ring Buffer should overwrite data or backpressure to HW.*/

//static char buffer[128];/**< Used to store the PCIe Device Name*/
//static char  *pci_devName = &buffer[0];/**< Insmod Parameter - the PCIe Device Name.*/
//const char * pci_devName_const;
const char pci_devName[] = "vsi_driver"; //name of the device

module_param(device_id, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(device_id, "DeviceID");/**< Insmod Parameter */

//module_param(pci_devName, charp, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
//MODULE_PARM_DESC(pci_devName, "DeviceName");/**< Insmod Parameter */

module_param(major, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(major, "MajorNumber");/**< Insmod Parameter */

module_param(enable_cdma_2, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(enable_cdma_2, "EnableCDMA2");/**< Insmod Parameter */

module_param(cdma_address, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(cdma_address, "CDMAAddress");/**< Insmod Parameter */

module_param(cdma_2_address, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(cdma_2_address, "CDMAAddress2");/**< Insmod Parameter */

module_param(pcie_ctl_address, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(pcie_ctl_address, "PCIeCTLAddress");/**< Insmod Parameter */

module_param(pcie_m_address, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(pcie_m_address, "PCIeMAddress");/**< Insmod Parameter */

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
/*****************************************************************************/

//char pci_devName_const[128] ;
unsigned long pci_bar_hw_addr;         /**< hardware base address of BAR 0 */
unsigned long pci_bar_size;            /**< hardware bar memory size of BAR 0 */
unsigned long pci_bar_1_addr;         /**< hardware base address of the device of BAR 1 (Not currently used) */
unsigned long pci_bar_1_size;         /**< hardware bar memory size of BAR 1 (not currently used) */
unsigned long pci_bar_2_addr;         /** < hardware base address of BAR 2 (not currently used) */
unsigned long pci_bar_2_size;         /** < Hardware bar memory size of BAR 2 (not currently used) */
struct pci_dev * pci_dev_struct = NULL;  /**<pci device struct */
struct platform_device * platform_dev_struct = NULL; /**< Platform device struct (for zynq) */
struct device *	dev_struct = NULL;
void * pci_bar_vir_addr = NULL;        /**< hardware base virtual address for BAR 0 */
void * pci_bar_1_vir_addr = NULL;        /**< hardware base virtual address for BAR 1 (not currently used) */
void * pci_bar_2_vir_addr = NULL;        /**< hardware base virtual address for BAR 2 (not currently used) */

/*this is the user peripheral address offset*/
u64 bar_0_axi_offset = 0x80000000;         /**< The AXI  address of BAR 0 (ie common interface IP) */

u64 axi_interr_ctrl = 0; /**< Global Variable that stores the Interrupt Controller AXI Address */
u64 axi_pcie_m;  /**< Global Variable that stores the data transport IP Slave AXI Address as seen from the CDMA*/

u8 cdma_set[5];  /**< Global variable that stores which CDMAs have been initialized. (currently only using 2 CDMAs) */
u8 pcie_ctl_set; /**< Global variable that is a flag to tell if the PCIe Controll address has been set */

//int cdma_status;
int cdma_capable = 0; /**< Global variable that is a flag to tell if the driver has been initialized properly to use the CDMA(s) */
unsigned int irq_num; /**< Global variable that stores the IRQ number that is probed from the device */
int cdma_usage_cnt = 0; /**< Global variable to count the number of CDMA uses. Used for statistics gathering */

/*CDMA Semaphores*/
struct mutex CDMA_sem;		/**< the Semaphore to lock CDMA 1 */
struct mutex CDMA_sem_2;	/**< THe Semaphore to lock CDMA 2 */
wait_queue_head_t cdma_q_head;   /**< The Sleep wait queue for CDMAs (if not polling) */
atomic_t cdma_q = ATOMIC_INIT(0);  /**< The atomic conditional variable for CDMA (if not polling) */


dma_addr_t dma_addr_base;  /**< The hardware DMA Allocation Address */
void * dma_buffer_base;   /**< This is the start of the DMA region virtual address */
u32 dma_current_offset;   /**< This variable holds the current offset of the DMA Allocation */
u64 dma_buffer_size = 1048576; /**< Default value for size of DMA Allocation, Max is 4MB, this is set through insmod */
u32 dma_garbage_offset;   /**< This offset memory region is used for dumping data when back pressure is not enabled */
u32 dma_garbage_size = 4096;   /**< This size of memory region is used for dumping data when back pressure is not enabled */
u32 dma_internal_offset; /**< The current offset of the internal DMA regions. The driver uses these for register R/W */
u32 dma_internal_size= 4096; /**< The size of the internal DMA regions. The driver uses these for register R/W */


dma_addr_t dma_m_addr[MAX_NUM_MASTERS]; /**< Used for Master DMA Allocations (currently not used) */
void * dma_master_buf[MAX_NUM_MASTERS]; /**< Used for Master DMA Allocations (currently not used) */

/* *************************************************  */

//size_t dma_size;
wait_queue_head_t wq;  /**< This is the wait queue for the CDMAs, (if not polling) (currently not used) */
wait_queue_head_t wq_periph; /**< This is the wait queue for the peripherals being polled. */
//wait_queue_head_t mutexq;

// Write Thread data
wait_queue_head_t thread_q_head_write; /**< The Wait queue for the WRITE Thread */
atomic_t thread_q_write = ATOMIC_INIT(0); /**< The Wait variable for the WRITE Thread */
struct task_struct * thread_struct_write; /**< task_struct used in creation of WRITE Thread */

// Read Thread data
wait_queue_head_t thread_q_head_read; /**< The Wait Queue for the READ Thread */
atomic_t thread_q_read = ATOMIC_INIT(0); /**< The Wait variable for the READ Thread */

struct task_struct * thread_struct_read; /**< task_struct used in creation of READ Thread */

wait_queue_head_t pci_write_head; /**< The Wait Queue for the blocking/sleeping pci_write function */

atomic_t cdma_atom[5]; /**< CDMA_x wait variable (if not polling) (currently not used) */
//int num_int;

//atomic_t mutex_free = ATOMIC_INIT(0);


DEFINE_KFIFO(read_fifo, struct mod_desc*, 8192); /**< sets up the global READ FIFO */
spinlock_t fifo_lock_read; /**< The Spinlock Variable for writing to the READ FIFO */

DEFINE_KFIFO(write_fifo, struct mod_desc*, 8192); /**< sets up the global WRITE FIFO */
spinlock_t fifo_lock_write; /**< The Spinlock Variable for writing to the WRITE FIFO */

/*Driver Statistics*/
atomic_t driver_tx_bytes = ATOMIC_INIT(0);  /**< Global Atomic Variable for Driver Statistics */
atomic_t driver_rx_bytes = ATOMIC_INIT(0);/**< Global Atomic Variable for Driver Statistics */
atomic_t driver_start_flag = ATOMIC_INIT(0);/**< Global Atomic Variable for Driver Statistics */
atomic_t driver_stop_flag = ATOMIC_INIT(0);/**< Global Atomic Variable for Driver Statistics */
struct timespec driver_start_time;/**< Global Struct for Driver Statistics */
struct timespec driver_stop_time;/**< Global Struct Variable for Driver Statistics */

//spinlock_t mLock;

/******************************** Xilinx Register Offsets **********************************/
const u32 INT_CTRL_IER      = 0x08;  /**< Interrupt Controller Register Offset, see Xilinx doc. */
const u32 INT_CTRL_MER      = 0x1c;	 /**< Interrupt Controller Register Offset, see Xilinx doc. */
const u32 INT_CTRL_ISR      = 0x00;	 /**< Interrupt Controller Register Offset, see Xilinx doc. */
const u32 INT_CTRL_IAR      = 0x0C;	 /**< Interrupt Controller Register Offset, see Xilinx doc. */


/*This is an array of interrupt structures to hold up to 8 peripherals*/
struct mod_desc * mod_desc_arr[12] = { 0 }; /**< This is an array of Module Descriptions that is used to be indexed by interrupt number
					     * This is how the pci_isr knows which peripheral has sent the interrupt. */

/*ISR Tasklet */
//void do_isr_tasklet(unsigned long);
//DECLARE_TASKLET(isr_tasklet, do_isr_tasklet, 0);



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
 * @brief File Operation Function for poll system calls
*/
unsigned int pci_poll(struct file *filep, poll_table * pwait);

/**
 * @brief File Operations Struct map
*/
struct file_operations pci_fops = {
		read:           pci_read,
		write:        	pci_write,
		unlocked_ioctl: pci_unlocked_ioctl,
		open:           pci_open,
		release:        pci_release,
		llseek:         pci_llseek,
		poll:           pci_poll,
		//	mmap:           pci_map,
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

static const struct of_device_id sv_driver_match[] = {
	{ .compatible = "xlnx,sv_driver_plat", },
	{},
};

static struct platform_driver sv_plat_driver = {
	.probe   = sv_plat_probe,
	.remove  = sv_plat_remove,
	.driver  = {
		.name = "sv_driver_plat",
		.of_match_table = of_match_ptr(sv_driver_match),
	},
};

static struct pci_driver pci_driver = {
	//.name = "vsi_driver",
	.name = pci_devName,
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
	int ret;
	int int_ctrl_set;
	/* Do probing type stuff here.
	 * 	 * Like calling request_region();
	 * 	 	 */


    verbose_printk(KERN_INFO"%s:[probe]******************************** PROBE PARAMETERS *****************************************\n", pci_devName);

    verbose_printk(KERN_INFO"%s:[probe] device_id: %d \n", pci_devName, device_id);
    verbose_printk(KERN_INFO"%s:[probe] major: %d \n", pci_devName, major);
    verbose_printk(KERN_INFO"%s:[probe] cdma_address: 0x%x \n", pci_devName, cdma_address);
    verbose_printk(KERN_INFO"%s:[probe] cdma_2_address: 0x%x \n", pci_devName, cdma_2_address);
    verbose_printk(KERN_INFO"%s:[probe] enable_cdma_2: %d \n", pci_devName, enable_cdma_2);
    verbose_printk(KERN_INFO"%s:[probe] pcie_ctl_address: 0x%x \n", pci_devName, pcie_ctl_address);
    verbose_printk(KERN_INFO"%s:[probe] pcie_m_address: 0x%x \n", pci_devName, pcie_m_address);
    verbose_printk(KERN_INFO"%s:[probe] int_ctlr_address: 0x%x \n", pci_devName, int_ctlr_address);
    verbose_printk(KERN_INFO"%s:[probe] driver_type: 0x%x \n", pci_devName, driver_type);
    verbose_printk(KERN_INFO"%s:[probe] dma_system_size: %d \n", pci_devName, dma_system_size);
    verbose_printk(KERN_INFO"%s:[probe] dma_file_size: %d \n", pci_devName, dma_file_size);
    verbose_printk(KERN_INFO"%s:[probe] dma_byte_width: %d \n", pci_devName, dma_byte_width);
    verbose_printk(KERN_INFO"%s:[probe] back_pressure: %d \n", pci_devName, back_pressure);
    verbose_printk(KERN_INFO"%s:[probe] axi2pcie_bar0_size : 0x%x \n", pci_devName, axi2pcie_bar0_size);
   verbose_printk(KERN_INFO"%s:[probe] interface_crc : 0x%x \n", pci_devName, interface_crc);
   verbose_printk(KERN_INFO"%s:[probe] interface_crc_check : 0x%x \n", pci_devName, interface_crc_check);


	verbose_printk(KERN_INFO"%s:[probe]******************************** BEGIN PROBE ROUTINE *****************************************\n", pci_devName);

	pci_dev_struct = dev;
	dev_struct = &dev->dev;

	if(NULL == pci_dev_struct){
		printk(KERN_INFO"%s:[probe]struct pci_dev_struct is NULL\n", pci_devName);
		return ERROR;
	}
	/****************** BAR 0 Mapping *******************************************/
	//get the base hardware address
	pci_bar_hw_addr = pci_resource_start(pci_dev_struct, 0);
	if (0 > pci_bar_hw_addr){
		printk(KERN_INFO"%s[probe]base hardware address is not set\n", pci_devName);
		return ERROR;
	}
	//get the base memory size
	pci_bar_size = pci_resource_len(pci_dev_struct, 0);
	printk(KERN_INFO"[probe]pci bar size is:%lu\n", pci_bar_size);

	//map the hardware space to virtual space
	pci_bar_vir_addr = ioremap(pci_bar_hw_addr, pci_bar_size);
	if(0 == pci_bar_vir_addr){
		printk(KERN_INFO"%s:[probe]ioremap error when mapping to vritaul address\n", pci_devName);
		return ERROR;
	}
	printk(KERN_INFO"[probe]pci bar virtual address base is:0x%p\n", pci_bar_vir_addr);


	//enable the device
	ret = pci_enable_device(dev);
	verbose_printk(KERN_INFO"[probe]device enabled\n");

	//set DMA mask
	if(0 != dma_set_coherent_mask(&dev->dev, 0x00000000FFFFFFFF)){
		printk(KERN_INFO"%s:[probe]set DMA mask error\n", pci_devName);
		return ERROR;
	}
	verbose_printk(KERN_INFO"[probe]dma mask set\n");

	//enable bus mastering
	pci_set_master(dev);
	verbose_printk(KERN_INFO"[probe]pci set as master\n");

	//enable MSI interrupts
#if defined(CONFIG_PCI_MSI)
	if(0 > pci_enable_msi(pci_dev_struct)){
		printk(KERN_INFO"%s:[probe]MSI enable error\n", pci_devName);
		return ERROR;
	}
	verbose_printk(KERN_INFO"[probe]pci enabled msi interrupt\n");
#endif
	//request IRQ
	if(0 > request_irq(pci_dev_struct->irq, &pci_isr, IRQF_TRIGGER_RISING | IRQF_SHARED, pci_devName, pci_dev_struct)){
		printk(KERN_INFO"%s:[probe]request IRQ error\n", pci_devName);
		return ERROR;
	}

	//register the char device
	if(0 > register_chrdev(major, pci_devName, &pci_fops)){
		//	dynamic_major = register_chrdev(0, pci_devName, &pci_fops);
		printk(KERN_INFO"%s:[probe]char driver not registered\n", pci_devName);
		printk(KERN_INFO"%s:[probe]char driver major number: 0x%x\n", pci_devName,major);
		return ERROR;
	}

	if (skel_get_revision(dev) == 0x42)
		return -ENODEV;

	/*allocate the DMA buffer*/
	dma_buffer_base = dma_alloc_coherent(dev_struct, (size_t)dma_buffer_size, &dma_addr_base, GFP_KERNEL);

	if(NULL == dma_buffer_base) {
		printk(KERN_INFO"%s:[sv_driver_init]DMA buffer base allocation ERROR\n", pci_devName);
		printk(KERN_INFO"[sv_driver_init] typical max DMA size is 4M, check your linux settings\n");
		return ERROR;
	} else {
		verbose_printk(KERN_INFO"[sv_driver_init]: dma kernel buffer base address is:0x%p\n", dma_buffer_base);
		verbose_printk(KERN_INFO"[sv_driver_init]: dma system memory buffer base address is:0x%p\n", (void *)dma_addr_base);
		dma_current_offset = 4096;   //we want to leave the first 4k for the kernel to use internally.
		dma_internal_offset = 0;
	}

	//set defaults
	cdma_set[1] = 0;
	cdma_set[2] = 0;
	pcie_ctl_set = 0;
	int_ctrl_set = 0;



	if (cdma_address != 0xFFFFFFFF) {
		ret = cdma_init(1, cdma_address);  //cdma_num = 1
	}

	//	if (cdma_address_2 != 0xFFFFFFFF)
	if (enable_cdma_2 != 0) {
		ret = cdma_init(2, cdma_2_address);  //cdma_num = 2
	}

	if (axi2pcie_bar0_size != 0xFFFFFFFF && pcie_ctl_address != 0xFFFFFFFF) {
      axi_pcie_m = dma_addr_base % axi2pcie_bar0_size;

		ret = pcie_ctl_init((u64)pcie_ctl_address, (u64)(dma_addr_base - axi_pcie_m));
		if (ret < 0)
			return ERROR;
		else {
			pcie_ctl_set = 1;
      }
	} else if (pcie_ctl_address != 0xFFFFFFFF) {
      printk(KERN_INFO"[sv_driver_init] axi2pcie_bar0_size not set\n");
      printk(KERN_INFO"[sv_driver_init] If this is a gen2 PCIE design address translation will fail\n");
      axi_pcie_m = 0;
      //return ERROR;
   }
   else {
      printk(KERN_INFO"[sv_driver_init] ERROR axi2pcie_bar0_size and pcie_ctl_address was not set\n");
      axi_pcie_m = 0;
   }

	if (int_ctlr_address != 0xFFFFFFFF) {
		int_ctlr_init((u64)int_ctlr_address);
		int_ctrl_set = 1;
	}

	cdma_capable = (cdma_set[1] == 1) & (int_ctrl_set == 1) & (pcie_ctl_set == 1);

	printk(KERN_INFO"[probe] cdma_capable = 0x%x\n", cdma_capable);
	printk(KERN_INFO"[probe] cdma_set[2] = 0x%x\n", cdma_set[2]);

	printk(KERN_INFO"[probe]***********************PROBE FINISHED SUCCESSFULLY**************************************\n");
	return 0;
}

/**
 * @brief This is the Probe function called for Platform Devices (ie Zynq)
*/
static int sv_plat_probe(struct platform_device *pdev)
{
	struct resource * resource_1;
	int ret;
	int int_ctrl_set;

   verbose_printk(KERN_INFO"%s:[probe]******************************** PROBE PARAMETERS *****************************************\n", pci_devName);

	verbose_printk(KERN_INFO"%s:[probe] device_id: %d \n", pci_devName, device_id);
	verbose_printk(KERN_INFO"%s:[probe] major: %d \n", pci_devName, major);
	verbose_printk(KERN_INFO"%s:[probe] cdma_address: 0x%x \n", pci_devName, cdma_address);
	verbose_printk(KERN_INFO"%s:[probe] cdma_2_address: 0x%x \n", pci_devName, cdma_2_address);
	verbose_printk(KERN_INFO"%s:[probe] enable_cdma_2: %d \n", pci_devName, enable_cdma_2);
	verbose_printk(KERN_INFO"%s:[probe] pcie_ctl_address: 0x%x \n", pci_devName, pcie_ctl_address);
	verbose_printk(KERN_INFO"%s:[probe] pcie_m_address: 0x%x \n", pci_devName, pcie_m_address);
	verbose_printk(KERN_INFO"%s:[probe] int_ctlr_address: 0x%x \n", pci_devName, int_ctlr_address);
	verbose_printk(KERN_INFO"%s:[probe] driver_type: 0x%x \n", pci_devName, driver_type);
	verbose_printk(KERN_INFO"%s:[probe] dma_system_size: %d \n", pci_devName, dma_system_size);
	verbose_printk(KERN_INFO"%s:[probe] dma_file_size: %d \n", pci_devName, dma_file_size);
	verbose_printk(KERN_INFO"%s:[probe] dma_byte_width: %d \n", pci_devName, dma_byte_width);
	verbose_printk(KERN_INFO"%s:[probe] back_pressure: %d \n", pci_devName, back_pressure);
	verbose_printk(KERN_INFO"%s:[probe] axi2pcie_bar0_size : 0x%x \n", pci_devName, axi2pcie_bar0_size);
   	verbose_printk(KERN_INFO"%s:[probe] interface_crc : 0x%x \n", pci_devName, interface_crc);
   	verbose_printk(KERN_INFO"%s:[probe] interface_crc_check : 0x%x \n", pci_devName, interface_crc_check);


	verbose_printk(KERN_INFO"%s:[probe]******************************** BEGIN PROBE ROUTINE *****************************************\n", pci_devName);

	platform_dev_struct = pdev;
	dev_struct = &pdev->dev;

	resource_1 = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	//get the base memory size
	pci_bar_size = resource_1->end - resource_1->start;
	printk(KERN_INFO"[probe]platform name is: %s\n", resource_1->name);
	printk(KERN_INFO"[probe]platform bar size is:%lu\n", pci_bar_size);
	printk(KERN_INFO"[probe]platform bar start is:0x%p end is:0x%p\n", &resource_1->start, &resource_1->end);
	//map the hardware space to virtual space
	pci_bar_vir_addr = devm_ioremap_resource(&pdev->dev, resource_1);

	if(0 == pci_bar_vir_addr){
		printk(KERN_INFO"%s:[probe]ioremap error when mapping to virtual address\n", pci_devName);
		return ERROR;
	}
	printk(KERN_INFO"[probe]pci bar virtual address base is:0x%p\n", pci_bar_vir_addr);

	/****************** BAR 1 Mapping *******************************************/

	//	resource_2 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	//get the base memory size
	//	pci_bar_1_size = resource_2->end - resource_2->start;
	//	verbose_printk(KERN_INFO"[probe]pci bar 1 size is:%d\n", pci_bar_1_size);

	//map the hardware space to virtual space
	//	pci_bar_1_vir_addr = devm_ioremap_resource(&pdev->dev, resource_2);
	//	if(0 == pci_bar_1_vir_addr){
	//		verbose_printk(KERN_INFO"%s:[probe]ioremap error when mapping to virtual address\n", pci_devName);
	//		return ERROR;
	//	}
	//	verbose_printk(KERN_INFO"[probe]pci bar 1 virtual address base is:0x%p\n", pci_bar_1_vir_addr);
	//peripheral_space_offset = 0;
	pci_bar_1_size = 0;

	/*****************************************************************************/


	//set DMA mask
	if(0 != dma_set_mask(&pdev->dev, 0x00000000ffffffff)){
		printk(KERN_INFO"%s:[probe]set DMA mask error\n", pci_devName);
		return ERROR;
	}
	printk(KERN_INFO"[probe]dma mask set\n");

	printk(KERN_INFO"[probe]interrupt controller initialized\n");

	irq_num = platform_get_irq(pdev, 0);
	printk(KERN_INFO"[probe]IRQ number is:%d\n", irq_num);

	printk(KERN_INFO"[probe]IRQ request complete\n");

	//register the char device
	if(0 > register_chrdev(major, "sv_driver", &pci_fops)){
		printk(KERN_INFO"%s:[probe]char driver not registered\n", "sv_driver");
		return ERROR;
	}

	printk(KERN_INFO"[probe]register device complete going to alloc %llx byte for dma\n",dma_buffer_size);

	/*allocate the DMA buffer*/
	dma_buffer_base = dma_alloc_coherent(dev_struct, (size_t)dma_buffer_size, &dma_addr_base, GFP_KERNEL);
	if(NULL == dma_buffer_base) {
		printk(KERN_INFO"%s:[sv_driver_init]DMA buffer base allocation ERROR\n", pci_devName);
		return ERROR;
	} else {
		printk(KERN_INFO"[sv_driver_init]: dma buffer base address is:0x%p\n", dma_buffer_base);
		printk(KERN_INFO"[sv_driver_init]: dma system memory buffer base address is:%llx\n", (u64)dma_addr_base);
		printk(KERN_INFO"[sv_driver_init]: dma system memory buffer size is:%llx\n", (u64)dma_buffer_size);
		//we want to leave the first 4k for the kernel to use internally on file registers.
		//the second 4k is used as a throw away buffer for data drops.
		//dma_current_offset = 4096;
		dma_internal_offset = 0;
		dma_garbage_offset = dma_internal_size;
		dma_current_offset = dma_internal_size + dma_garbage_size;
	}
	printk(KERN_INFO"[probe]alloc coherent complete\n");

	//set defaults
	cdma_set[1] = 0;
	cdma_set[2] = 0;
	pcie_ctl_set = 0;

	int_ctrl_set = 0;
	printk(KERN_INFO"[sv_driver_init] cdma_address    size (%d) value 0x%x\n", (int)sizeof(cdma_address),(unsigned int)cdma_address);
	printk(KERN_INFO"[sv_driver_init] pcie_m_address  size (%d) value 0x%x\n", (int)sizeof(pcie_m_address),(unsigned int)pcie_m_address);
	printk(KERN_INFO"[sv_driver_init] cdma_2_address  size (%d) value 0x%x\n", (int)sizeof(cdma_2_address),(unsigned int)cdma_2_address);
	printk(KERN_INFO"[sv_driver_init] int_ctlr_address  size (%d) value 0x%x\n", (int)sizeof(int_ctlr_address),(unsigned int)int_ctlr_address);
	// these addresses should alayws be in the 32 bit range
	cdma_address     &= 0xFFFFFFFF;
	pcie_m_address   &= 0xFFFFFFFF;
	cdma_2_address   &= 0xFFFFFFFF;
	int_ctlr_address &= 0xFFFFFFFF;

	if (cdma_address != 0xFFFFFFFF) {
		ret = cdma_init(1, cdma_address);
	}

	if (enable_cdma_2 != 0) {
		ret = cdma_init(2, cdma_2_address);
	}

	if (int_ctlr_address != 0xFFFFFFFF) {
		int_ctlr_init((u64)int_ctlr_address);
		int_ctrl_set = 1;
	}

	/*ARM works a little differently for DMA than PCIe in that that translation
	 * is not handled by the core by writing to a register. For Zynq, the axi to DDR address
	 * mapping is 1-1 and should be written directly to the returned DMA handle */

	//request IRQ last
	if(0 > request_irq(irq_num, &pci_isr, IRQF_TRIGGER_RISING  | IRQF_SHARED, pci_devName, pdev)){
		printk(KERN_INFO"%s:[probe]request IRQ error\n", pci_devName);
		return ERROR;
	}

	printk(KERN_INFO"[probe]IRQ request complete\n");

	if (cdma_address != 0xFFFFFFFF) {
		ret = cdma_init(1, cdma_address);
	}

	if (enable_cdma_2 != 0) {
		ret = cdma_init(2, cdma_2_address);
	}

	if (int_ctlr_address != 0xFFFFFFFF) {
		int_ctlr_init((u64)int_ctlr_address);
		int_ctrl_set = 1;
	}

	axi_pcie_m = (u64)dma_addr_base;   //cdma 1

	cdma_capable = (cdma_set[1] == 1) & (int_ctrl_set == 1);


	printk(KERN_INFO"[probe] cdma_capable = 0x%x\n", cdma_capable);
	printk(KERN_INFO"[probe] cdma_set[2] = 0x%x\n", cdma_set[2]);

	printk(KERN_INFO"[probe]***********************PROBE FINISHED SUCCESSFULLY**************************************\n");
	return 0;
}

static void sv_pci_remove(struct pci_dev *dev)
{
	/* clean up any allocated resources and stuff here.
	 * 	 * like call release_region();
	 * 	 	 */
	//      release_mem_region(pci_bar_hw_addr, REG_SIZE);
   printk(KERN_INFO"[sv_pci_remove]: PCIE remove\n");

   free_irq(pci_dev_struct->irq, pci_dev_struct);

	/*Destroy the read thread*/
   printk(KERN_INFO"[sv_pci_remove]: Stopping read thead\n");
	atomic_set(&thread_q_read, 1);
	wake_up_interruptible(&thread_q_head_read);
   if(thread_struct_read) {
   	while(kthread_stop(thread_struct_read)<0)
   	{
         printk(KERN_INFO"[sv_pci_remove]: Read thread failed to stop, attemping again\n");
   		atomic_set(&thread_q_read, 1);
   		wake_up_interruptible(&thread_q_head_read);
   	}
   }
	printk(KERN_INFO"[sv_pci_remove]: Read Thread Destroyed\n");

	/*Destroy the write thread*/
   printk(KERN_INFO"[sv_pci_remove]: Stopping read thead\n");
	atomic_set(&thread_q_write, 1);
	wake_up_interruptible(&thread_q_head_write);
   if(thread_struct_write) {
   	while(kthread_stop(thread_struct_write)<0)
   	{
         printk(KERN_INFO"[sv_pci_remove]: Write thread failed to stop, attemping again\n");
   		atomic_set(&thread_q_write, 1);
   		wake_up_interruptible(&thread_q_head_write);
   	}
   }
	printk(KERN_INFO"[sv_pci_remove]: Write Thread Destroyed\n");

#if defined(CONFIG_PCI_MSI)
	pci_disable_msi(pci_dev_struct);
#endif

	unregister_chrdev(major, pci_devName);
   iounmap(pci_bar_vir_addr);
   dma_free_coherent(dev_struct, (size_t)dma_buffer_size, dma_buffer_base, dma_addr_base);

	printk(KERN_INFO"[sv_pci_remove]: ***********************PCIE DEVICE REMOVED**************************************\n");

}

static int sv_plat_remove(struct platform_device *pdev)
{
   printk(KERN_INFO"[sv_plat_remove]: Platform remove\n");
	free_irq(irq_num, platform_dev_struct);

	/*Destroy the read thread*/
	printk(KERN_INFO"[sv_plat_remove]: Stopping read thead\n");
	atomic_set(&thread_q_read, 1);
	wake_up_interruptible(&thread_q_head_read);
	if(thread_struct_read) {
		while(kthread_stop(thread_struct_read)<0)
		{
			printk(KERN_INFO"[sv_plat_remove]: Read thread failed to stop, attemping again\n");
			atomic_set(&thread_q_read, 1);
			wake_up_interruptible(&thread_q_head_read);
		}
	}
	printk(KERN_INFO"[sv_plat_remove]: Read Thread Destroyed\n");

	/*Destroy the write thread*/
	printk(KERN_INFO"[sv_plat_remove]: Stopping write thead\n");
	atomic_set(&thread_q_write, 1);
	wake_up_interruptible(&thread_q_head_write);
	if(thread_struct_write) {
		while(kthread_stop(thread_struct_write)<0) {
			printk(KERN_INFO"[sv_plat_remove]: Write thread failed to stop, attemping again\n");
			atomic_set(&thread_q_write, 1);
			wake_up_interruptible(&thread_q_head_write);
		}
	}
	printk(KERN_INFO"[sv_plat_remove]: Write Thread Destroyed\n");

	unregister_chrdev(major, pci_devName);
	iounmap(pci_bar_vir_addr);
	dma_free_coherent(dev_struct, (size_t)dma_buffer_size, dma_buffer_base, dma_addr_base);

	printk(KERN_INFO"[sv_plat_remove]: ***********************PLATFORM DEVICE REMOVED**************************************\n");

	return 0;
}

static int __init sv_driver_init(void)
{

	dma_buffer_size = (u64)dma_system_size;
	//void * dummy;

	memset(mod_desc_arr, 0, sizeof(mod_desc_arr));

	printk(KERN_INFO"[sv_driver_init]: Initializing driver type %d\n",driver_type);
	switch(driver_type){
		case PCI:
			printk(KERN_INFO"[pci_init]: Device ID: ('%d')\n", device_id);
			printk(KERN_INFO"[pci_init]: Major Number: ('%d')\n", major);

			init_waitqueue_head(&wq);
			init_waitqueue_head(&wq_periph);
			init_waitqueue_head(&thread_q_head_write);
			init_waitqueue_head(&thread_q_head_read);
			init_waitqueue_head(&cdma_q_head);
			init_waitqueue_head(&pci_write_head);

			ids[0].vendor =  PCI_VENDOR_ID_XILINX;
			ids[0].device =  (u32)device_id;

			//strcpy(pci_devName_const, pci_devName);
			printk(KERN_INFO"using driver name: %s\n", pci_devName);
			//pci_devName_const = pci_devName;

			/*Create Read Thread*/
			thread_struct_read = create_thread_read((struct kfifo *)&read_fifo);

			/*Create Write Thread*/
			thread_struct_write = create_thread_write((struct kfifo *)&write_fifo);

			/*FIFO init stuff*/
			spin_lock_init(&fifo_lock_read);
			spin_lock_init(&fifo_lock_write);

			return pci_register_driver(&pci_driver);
			break;

		case PLATFORM:

			printk(KERN_INFO"[platform_init]: Device ID: ('%d')\n", device_id);
			printk(KERN_INFO"[platform_init]: Major Number: ('%d')\n", major);

			init_waitqueue_head(&wq);
			init_waitqueue_head(&wq_periph);
			init_waitqueue_head(&thread_q_head_write);
			init_waitqueue_head(&thread_q_head_read);
			init_waitqueue_head(&pci_write_head);

			/*Create Read Thread*/
			thread_struct_read = create_thread_read((struct kfifo *)&read_fifo);

			/*Create Write Thread*/
			thread_struct_write = create_thread_write((struct kfifo *)&write_fifo);

			/*FIFO init stuff*/
			spin_lock_init(&fifo_lock_read);
			spin_lock_init(&fifo_lock_write);
			printk(KERN_INFO"[platform_init]: platform_driver_register started \n");
			return platform_driver_register(&sv_plat_driver);

			break;

		default:;
	}

	verbose_printk(KERN_INFO"[platform_init]: !!!!ERROR!!!!! No correct driver type detected!\n");
	return 0;
}

static void __exit sv_driver_exit(void)
{
	switch(driver_type){
		case PCI:
			printk(KERN_INFO"[sv_driver_exit]: pci_driver_unregister started \n");
			pci_unregister_driver(&pci_driver);
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
	int ret;
	u32 vec_serviced;
	int i;

	verbose_isr_printk(KERN_INFO"[pci_isr]: 	Entered the ISR (%llx)\n",axi_interr_ctrl);
	//	tasklet_schedule(&isr_tasklet);
	if (axi_interr_ctrl == 0) {
		verbose_isr_printk(KERN_INFO"[pci_isr]: 	returning early ISR (%llx)\n",axi_interr_ctrl);
		return IRQ_HANDLED; // controller not intialized yet
	}
	vec_serviced = 0;
	i = 0;
	/*Here we need to find out who triggered the interrupt*
	 *Since we only allow one MSI vector, we need to query the
	 *Interrupt controller to find out. */

	/*This is the interrupt status register*/
	axi_dest = axi_interr_ctrl + INT_CTRL_ISR;
	ret = data_transfer(axi_dest, (void *)&isr_status, 4, NORMAL_READ, 0);
	verbose_isr_printk(KERN_INFO"[pci_isr]: interrupt status register vector is: ('0x%08x')\n", isr_status);

	/*Here we need to clear the service interrupt in the interrupt acknowledge register*/
	axi_dest = axi_interr_ctrl + INT_CTRL_IAR;
	status = isr_status;
	ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_WRITE, 0);

	interrupt_num = vec2num(isr_status);

	while(isr_status > 0) {

		//interrupt_num = vec2num(isr_status);
		//	tasklet_schedule(&isr_tasklet);

		//check to see if axi stream fifo is set_interrupt
				//todo should be only for steam fifo interupt
		if (!mod_desc_arr[interrupt_num]) {
			isr_status = isr_status && ~num2vec(interrupt_num);					//don't service this one, clear it from isr_status
			interrupt_num = 0;																					//don't service this one
			//vec_serviced = vec_serviced | num2vec(interrupt_num);
			verbose_isr_printk(KERN_INFO"[pci_isr]: 	returning early ISR null axi stream fifo \n");
			continue; // controller not intialized yet
														//todo make it so we handle other interupts instead of just failing
		}

		verbose_isr_printk(KERN_INFO"[pci_isr]: interrupt number is: ('%d'-->' minor: %x')\n", interrupt_num, mod_desc_arr[interrupt_num]->minor);
		device_mode = mod_desc_arr[interrupt_num]->mode;


		if (device_mode == CDMA) {
			verbose_isr_printk(KERN_INFO"[pci_isr]: this interrupt is from the CDMA\n");
			vec_serviced = vec_serviced | num2vec(interrupt_num);
		}
		else if (device_mode == AXI_STREAM_FIFO || device_mode == AXI_STREAM_PACKET) {
			verbose_isr_printk(KERN_INFO"[pci_isr]: this interrupt is from a user peripheral\n");

			/*Read the axi fifo ISR*/
			axi_dest = mod_desc_arr[interrupt_num]->axi_addr_ctl + AXI_STREAM_ISR;
			ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_READ, 0);
			verbose_isr_printk(KERN_INFO"[pci_isr]: Stream FIFO ISR status: 0x%08x\n", status);

			/*clear the axi fifo ISR*/
			status = status & 0x04000000;																																					//clear the status observed masked wtih RX complete
			ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_WRITE, 0);

			if(atomic_read(mod_desc_arr[interrupt_num]->in_read_fifo_count) == 0 && mod_desc_arr[interrupt_num]->file_open) {

				//debug message
				if (kfifo_len(&read_fifo) > 4) {
					verbose_isr_printk(KERN_INFO"[pci_isr]: kfifo stored elements: %d\n", kfifo_len(&read_fifo));
				}

				//if the read fifo isn't full and the file is open, write the mod_desc in
				if(!kfifo_is_full(&read_fifo)) {
					atomic_inc(mod_desc_arr[interrupt_num]->in_read_fifo_count);
					kfifo_in_spinlocked(&read_fifo, &mod_desc_arr[interrupt_num], 1, &fifo_lock_read);
				}
				else {
					verbose_isr_printk(KERN_INFO"[pci_isr]: kfifo is full, not writing mod desc\n");
				}
			}
			else if (!mod_desc_arr[interrupt_num]->file_open) {
				verbose_isr_printk(KERN_INFO"[pci_isr]: file is closed, not writing mod_desc\n");
			}
			else {
				verbose_isr_printk(KERN_INFO"[pci_isr]: kfifo already contains axi_steam_fifo mod desc\n");
			}

			//for ring buffer peripherals (axi streaming) there is now data so we should read
			atomic_set(&thread_q_read, 1); // wake up if empty
			wake_up_interruptible(&thread_q_head_read);

			verbose_isr_printk(KERN_INFO"[pci_isr]: Waking up the read thread\n");
		}
		else {
			atomic_set(mod_desc_arr[interrupt_num]->atomic_poll, 1);  //non threaded way
			//for non ring buffer peripherals (memory)
			//wake_up(&wq_periph);
			wake_up(&mod_desc_arr[interrupt_num]->poll_wq);
			verbose_isr_printk(KERN_INFO"[pci_isr]: Waking up the Poll(normal)\n");
		}

		vec_serviced = vec_serviced | num2vec(interrupt_num);

		//get next interrupt number
		isr_status = isr_status & ~num2vec(interrupt_num);
      verbose_isr_printk(KERN_INFO"[pci_isr]: vectors serviced is: ('0x%08x')\n", vec_serviced);
      verbose_isr_printk(KERN_INFO"[pci_isr]: interrupt status register vector is: ('0x%08x')\n", isr_status);
		interrupt_num = vec2num(isr_status);
	}


	verbose_isr_printk(KERN_INFO"[pci_isr]: All interrupts serviced. The following Vector is acknowledged: 0x%x\n", vec_serviced);

	if (vec_serviced > 0) {
		/* The CDMA vectors (1 and 2) */

		/*The CDMA vectors are currently not being used unless we go back to the CDMAs sending interrupts.
		 * Right now we are polling the IDLE bit and it works much faster than waiting for the interrupt
		 */

		if ((vec_serviced & 0x01) == 0x01) {
			//	cdma_comp[1] = 1;      //condition for wake_up
			atomic_set(&cdma_atom[1], 1);
			verbose_isr_printk(KERN_INFO"[pci_isr]: Waking up CDMA 1\n");
			wake_up_interruptible(&wq);
		}

		if ((vec_serviced & 0x02) == 0x02) {
			//	cdma_comp[2] = 1;      //condition for wake_up
			atomic_set(&cdma_atom[2], 1);
			verbose_isr_printk(KERN_INFO"[pci_isr]: Waking up CDMA 2\n");
			wake_up_interruptible(&wq);
		}

		if (vec_serviced >= 0x10) {
			//for non ring buffer peripherals (memory)
		//	wake_up(&wq_periph);
		//	verbose_printk(KERN_INFO"[soft_isr]: Waking up the Poll()\n");

			//for ring buffer peripherals (axi streaming)
		//	atomic_set(&thread_q_read, 1); // the threaded way
		//	wake_up_interruptible(&thread_q_head_read);
		//	verbose_printk(KERN_INFO"[soft_isr]: Waking up the read thread\n");

		}
	}

	verbose_isr_printk(KERN_INFO"[pci_isr]: 						Exiting ISR\n");

	return IRQ_HANDLED;

}

/* -----------------File Operations-------------------------*/
int pci_open(struct inode *inode, struct file *filep)
{
	//void * interrupt_count;
	struct mod_desc * s;
	struct timespec * start_time;
	struct timespec * stop_time;

	atomic_t * atomic_poll;
	atomic_t * wth;
	atomic_t * wtk;
	atomic_t * write_ring_buf_full;
	atomic_t * rfh;
	atomic_t * rfu;
	atomic_t * read_ring_buf_full;
	atomic_t * pci_write_q;
	atomic_t * in_read_fifo_count;
	atomic_t * in_write_fifo_count;


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
	rfu = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);
	rfh = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);
	read_ring_buf_full = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);
	pci_write_q = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);
	in_read_fifo_count = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);
	in_write_fifo_count = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);



	atomic_set(atomic_poll, 0);
	atomic_set(wth, 0);
	atomic_set(wtk, 0);
	atomic_set(write_ring_buf_full, 0);
	atomic_set(rfh, 0);
	atomic_set(rfu, 0);
	atomic_set(read_ring_buf_full, 0);
	atomic_set(pci_write_q, 0);
	atomic_set(in_read_fifo_count, 0);
	atomic_set(in_write_fifo_count, 0);


	//interrupt_count = kmalloc(sizeof(int), GFP_KERNEL);

	s = (struct mod_desc *)kmalloc(sizeof(struct mod_desc), GFP_KERNEL);
	s->minor = MINOR(inode->i_rdev);
	s->axi_addr = 0;
	s->axi_addr_ctl = 0;
	s->mode = 0;
	s->int_num = 100;
	s->master_num = 0;
	s->interrupt_vec = 0;
	s->has_interrupt_vec = 0;
	s->axi_fifo_rlr  = 0;
	s->axi_fifo_rdfo = 0;
   s->read_header_size = 0;
	s->keyhole_config = 0;
	s->dma_offset_read = 0;
	s->dma_offset_write = 0;
	s->dma_size = dma_file_size;
	s->file_size = 4096;   //default to 4K
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
	s->read_ring_buf_full = read_ring_buf_full;
	s->ring_pointer_write = ring_pointer_write;
	s->ring_pointer_read = ring_pointer_read;
	s->in_fifo_read = in_fifo_read;
	s->in_fifo_read_flag = 0;
	s->in_fifo_write = in_fifo_write;
	s->in_fifo_write_flag = 0;
	s->pci_write_q = pci_write_q;

	s->in_write_fifo_count = in_write_fifo_count;
	s->in_read_fifo_count = in_read_fifo_count;

	s->file_open = true;

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
	int in_read_fifo_count, in_write_fifo_count;
	struct mod_desc* mod_desc;

	//printk(KERN_INFO"[pci_release]: Attempting to close file minor number: %d\n", mod_desc->minor);

	mod_desc = filep->private_data;

	/*Query private data to see if it allocated DMA as a Master*/
	if (mod_desc->mode == MASTER)	{
		//unallocate DMA
		pci_free_consistent(pci_dev_struct, 131702, dma_master_buf[mod_desc->master_num], dma_m_addr[mod_desc->master_num]);
	}
	else if (mod_desc->mode == AXI_STREAM_FIFO || mod_desc->mode == AXI_STREAM_PACKET)	{
		//turn off interupts from fifo
		axi_stream_fifo_deinit(mod_desc);
	}


	mod_desc->file_open = false;


	//reset ring_buffers incase anything was help up on them being too full
	atomic_set(mod_desc->rfh, 0);
	atomic_set(mod_desc->rfu, 0);
	atomic_set(mod_desc->wtk, 0);
	atomic_set(mod_desc->wth, 0);
	atomic_set(mod_desc->write_ring_buf_full, 0);
	atomic_set(mod_desc->read_ring_buf_full, 0);

	in_read_fifo_count = atomic_read(mod_desc->in_read_fifo_count);
	//wake up the read threads until we clear our mod_desc out of them so we can free the memory assiociated with them.
	while(in_read_fifo_count != 0){
		verbose_printk(KERN_INFO"[pci_%x_release]: in_read_fifo_count(%d) is not zero! waking thread\n", mod_desc->minor, in_read_fifo_count);
		atomic_set(&thread_q_read, 1);
		wake_up_interruptible(&thread_q_head_read);
		schedule();					//think we want to schedule here to give thread time to do work
		in_read_fifo_count = atomic_read(mod_desc->in_read_fifo_count);
	}

	in_write_fifo_count = atomic_read(mod_desc->in_write_fifo_count);
	//wake up the write threads until we clear our mod_desc out of them so we can free the memory assiociated with them.
	while(in_write_fifo_count != 0){
		verbose_printk(KERN_INFO"[pci_%x_release]: in_write_fifo_count(%d) is not zero! waking thread\n", mod_desc->minor, in_write_fifo_count);
		atomic_set(&thread_q_write, 1);
		wake_up_interruptible(&thread_q_head_write);
		schedule();					//think we want to schedule here to give thread time to do work
		in_write_fifo_count = atomic_read(mod_desc->in_write_fifo_count);
	}

	kfree((const void*)mod_desc->start_time);
	kfree((const void*)mod_desc->stop_time);

	kfree((const void*)mod_desc->in_fifo_read);
	kfree((const void*)mod_desc->in_fifo_write);
	kfree((const void*)mod_desc->ring_pointer_write);
	kfree((const void*)mod_desc->ring_pointer_read);
	kfree((const void*)mod_desc->atomic_poll);
	kfree((const void*)mod_desc->wtk);
	kfree((const void*)mod_desc->wth);
	kfree((const void*)mod_desc->write_ring_buf_full);
	kfree((const void*)mod_desc->rfu);
	kfree((const void*)mod_desc->rfh);
	kfree((const void*)mod_desc->read_ring_buf_full);
	kfree((const void*)mod_desc->pci_write_q);
	kfree((const void*)mod_desc->in_read_fifo_count);
	kfree((const void*)mod_desc->in_write_fifo_count);




	//	kfree((const void*)filep->private_data);

	verbose_printk(KERN_INFO"[pci_%x_release]: Successfully closed file.\n", mod_desc->minor);
	kfree((const void*)mod_desc);

	return SUCCESS;
}


/*item for file_operations: unlocked ioctl*/
long pci_unlocked_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct mod_desc *mod_desc;
	static int master_count = 0;
	u32 bit_vec;
	u64 arg_loc;
	//	u32 kern_reg;
	int interrupt_num;
	int ret;
	struct statistics * stats; //= kmalloc(sizeof(struct statistics), GFP_KERNEL);
	struct timespec diff;
	int minor;
	ret = copy_from_user(&arg_loc, argp, sizeof(u64));


	mod_desc = filep->private_data;
	minor = mod_desc->minor;

	verbose_printk(KERN_INFO"[pci_%x_ioctl]: Entering IOCTL with command: %d and arg %llx\n", minor, cmd, arg_loc);


	switch(cmd){

		case SET_AXI_DEVICE:
			printk(KERN_INFO"[pci_%x_ioctl]: Setting Peripheral AXI Address: 0x%llx\n", minor, arg_loc);
			mod_desc->axi_addr = arg_loc&(0xFFFFFFFFFFFFFFFF);
			break;

		case SET_AXI_CTL_DEVICE:
			printk(KERN_INFO"[pci_%x_ioctl]: Setting Peripheral CTL AXI Address: 0x%llx\n", minor, arg_loc);
			mod_desc->axi_addr_ctl = arg_loc&(0xFFFFFFFFFFFFFFFF);
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
			 *one interrupt output to the the PCIe controller.  This is because we want to use one msi vector. */
		case SET_AXI_INT_CTRL:
			//			int_ctlr_init(arg_loc);
			break;

		case SET_DMA_SIZE:
			printk(KERN_INFO"[pci_%x_ioctl]: Setting Peripheral DMA size:%llx\n", minor, arg_loc);
			mod_desc->dma_size = (size_t)arg_loc;
			//if (((u64)dma_current_offset + arg_loc) > (u64)((char*)dma_buffer_base + dma_buffer_size)) {
			if (((u64)dma_current_offset + arg_loc*2) > dma_buffer_size) {
				printk(KERN_INFO"[pci_%x_ioctl]: ERROR! DMA Buffer z memory!\n", minor);
				return ERROR;
			}
			else {
				mod_desc->dma_size = (size_t)arg_loc;
				printk(KERN_INFO"[[pci_%x_ioctl]: The current system memory dma offset:0x%x\n", minor, dma_current_offset);
				mod_desc->dma_offset_read = dma_current_offset;            //set the dma start address for the peripheral read
				mod_desc->dma_offset_write = dma_current_offset + mod_desc->dma_size; //set the dma start address for the peripheral write
				mod_desc->dma_write_addr = dma_buffer_base + (u64)dma_current_offset + mod_desc->dma_size;            //actual pointer to kernel buffer
				printk(KERN_INFO"[pci_%x_ioctl]: DMA kernel write address set to:0x%p\n", minor, mod_desc->dma_write_addr);
				mod_desc->dma_read_addr = dma_buffer_base + (u64)dma_current_offset;            //actual pointer to kernel buffer
				printk(KERN_INFO"[pci_%x_ioctl]: DMA kernel read address set to:0x%p\n", minor, mod_desc->dma_read_addr);

				dma_current_offset = dma_current_offset + (u32)(2*mod_desc->dma_size);            //update the current dma allocation pointer, 2 buffers (R/W)
				verbose_printk(KERN_INFO"[pci_%x_ioctl]: Success setting peripheral DMA\n", minor);
			}

			break;

		case RESET_DMA_ALLOC:
			dma_current_offset = 4096;   //we want to leave the first 4k for the kernel to use internally.
			dma_internal_offset = 0;
			verbose_printk(KERN_INFO"[pci_%x_ioctl]: Reset the DMA Allocation\n", minor);
			break;

		case SET_INTERRUPT:
			verbose_printk(KERN_INFO"[pci_%x_ioctl]: Setting device as an Interrupt source with vector:%llx\n", minor, arg_loc);

			//	init_waitqueue_head(mod_desc->iwq);

			/*Store the Interrupt Vector*/
			mod_desc->interrupt_vec = (u32)arg_loc;
			mod_desc->has_interrupt_vec = 1;
			interrupt_num = vec2num((u32)arg_loc);
			mod_desc->int_num = interrupt_num;
			printk(KERN_INFO"[pci_%x_ioctl]: Interrupt Number:%d\n", minor, interrupt_num);

			mod_desc_arr[interrupt_num] = mod_desc;

			//MM
			/*
			ret = axi_stream_fifo_init(mod_desc);
			if (ret < 0)
				return ERROR;
				*/
			break;

		case SET_FILE_SIZE:
			mod_desc->file_size = ((loff_t)arg_loc & 0xffffffff);
			verbose_printk(KERN_INFO"[pci_%x_ioctl]: Setting device file size:%llu\n", minor, mod_desc->file_size);

			/*initialize the DMA for the file*/
			ret = dma_file_init(mod_desc, dma_buffer_base, dma_buffer_size);
			if (ret < 0) {
				printk(KERN_INFO"[pci_%x_ioctl]: !!!!file open FAILURE!!!!.\n", minor);
				return ERROR;
			}
			break;


		case SET_CDMA_KEYHOLE_WRITE:
			bit_vec = 0x00000020;   //the bit for KEYHOLE WRITE

			if(arg_loc>0) {   //We are ENABLING keyhole write
				verbose_printk(KERN_INFO"[pci_%x_ioctl]: Setting the CDMA Keyhole WRITE as ENABLED\n", minor);
				ret = cdma_config_set(bit_vec, 1, 1);   //value of one means we want to SET the register
			} else { //We are disabling keyhole write
				verbose_printk(KERN_INFO"[pci_%x_ioctl]: Setting the CDMA Keyhole WRITE as DISABLED\n", minor);
				ret = cdma_config_set(bit_vec, 0, 1);   //value of 0 means we want to UNSET the register
			}
			break;

		case SET_CDMA_KEYHOLE_READ:
			bit_vec = 0x00000010;   //the bit for KEYHOLE READ

			if(arg_loc>0)  {  //We are ENABLING keyhole read
				verbose_printk(KERN_INFO"[pci_%x_ioctl]: Setting the CDMA Keyhole READ as ENABLED\n", minor);
				ret = cdma_config_set(bit_vec, 1, 1);   //value of one means we want to SET the register
			} else {//We are disabling keyhole read
				verbose_printk(KERN_INFO"[pci_%x_ioctl]: Setting the CDMA Keyhole READ as DISABLED\n", minor);
				ret = cdma_config_set(bit_vec, 0, 1);   //value of 0 means we want to UNSET the register
			}
			break;

		case GET_FILE_STATISTICS:
			stats = (struct statistics *)kmalloc(sizeof(struct statistics), GFP_KERNEL);
			ret = copy_from_user(stats, (void *)arg, sizeof(struct statistics));

			//statistics = (struct statistics *)arg;
			stats->tx_bytes = mod_desc->tx_bytes;
			stats->rx_bytes = mod_desc->rx_bytes;
			stats->cdma_attempt = mod_desc->cdma_attempt;
			stats->ip_not_ready = mod_desc->ip_not_ready;
			stats->cdma_usage_cnt = cdma_usage_cnt;
			cdma_usage_cnt = 0;

			if (mod_desc->stop_flag == 1) {
				mod_desc->stop_flag = 0;
				diff = timespec_sub(*(mod_desc->stop_time), *(mod_desc->start_time));
				stats->seconds = (unsigned long)diff.tv_sec;
				stats->ns = (unsigned long)diff.tv_nsec;
			}

			/*reset the counters*/
			mod_desc->tx_bytes = 0;
			mod_desc->rx_bytes = 0;
			mod_desc->ip_not_ready = 0;

			ret = copy_to_user((void *)arg, stats, sizeof(struct statistics));

			break;

		case GET_DRIVER_STATISTICS:
         stats = (struct statistics *)kmalloc(sizeof(struct statistics), GFP_KERNEL);
			ret = copy_from_user(stats, (void *)arg, sizeof(struct statistics));
			//	statistics = (struct statistics *)arg;
			stats->tx_bytes = atomic_read(&driver_tx_bytes);
			stats->rx_bytes = atomic_read(&driver_rx_bytes);

			if (atomic_read(&driver_stop_flag) == 1) {
				atomic_set(&driver_stop_flag, 0);
				diff = timespec_sub((driver_stop_time), (driver_start_time));
				stats->seconds = (unsigned long)diff.tv_sec;
				stats->ns = (unsigned long)diff.tv_nsec;
			}

			/*reset the counters*/
			atomic_set(&driver_tx_bytes, 0);
			atomic_set(&driver_rx_bytes, 0);
			//driver_ip_not_ready = 0;
			ret = copy_to_user((void *)arg, stats, sizeof(struct statistics));
			break;

		case START_FILE_TIMER:
			mod_desc->start_flag = 1;
			break;

		case STOP_FILE_TIMER:
			mod_desc->stop_flag = 1;
			break;

		case START_DRIVER_TIMER:
			atomic_set(&driver_start_flag, 1);
			break;

		case STOP_DRIVER_TIMER:
			atomic_set(&driver_stop_flag, 1);
			break;

		case SET_MODE:
			verbose_printk(KERN_INFO"[pci_%x_ioctl]: Setting the mode of the peripheral\n", minor);
			mod_desc->mode = (u32)arg_loc;

			switch(mod_desc->mode){

				case SLAVE:
					printk(KERN_INFO"[pci_%x_ioctl]: Setting the mode of the peripheral to SLAVE\n", minor);
					break;

				case MASTER:
					printk(KERN_INFO"[pci_%x_ioctl]: Setting the mode of the peripheral to MASTER\n", minor);

					/*DMA Allocation*/
					/*This sets the DMA read and write kernel buffers and obtains the
					 * dma_addr's to be sent to the CDMA core upon R/W transactions*/
					dma_master_buf[master_count] = pci_alloc_consistent(pci_dev_struct, 4096, &dma_m_addr[master_count]);
					if(NULL == dma_master_buf[master_count]) {
						verbose_printk(KERN_INFO"[pci_%x_ioctl]: DMA AXI Master allocation ERROR: \"%s\" \n", minor, pci_devName);
						return ERROR;
					}

					/*write address to AXI to PCIe-M Translation Register*/

					/*write master_count as master_num to file descriptor to use in referencing its memory address*/
					mod_desc->master_num = master_count;
					break;
            case AXI_STREAM_PACKET :
				case AXI_STREAM_FIFO :
					printk(KERN_INFO"[pci_%x_ioctl]: Setting the mode of the peripheral to AXI_STREAM_FIFO\n", minor);

					/*Initialize the AXI_STREAM_FIFO*/
					/*for now we will initialize all as interrupting for read*/
					//					/*check to see if the AXI addresses have been set*/
					if ((mod_desc->axi_addr == 0) | (mod_desc->axi_addr_ctl == 0)) {
						printk(KERN_INFO"[pci_%x_ioctl]: ERROR: axi addresses of AXI STREAM FIFO not set\n", minor);
						printk(KERN_INFO"[pci_%x_ioctl]:         set the AXI addresses then set mode again\n", minor);
						return ERROR;
					} else {
						verbose_printk(KERN_INFO"[pci_%x_ioctl]: Initializing the FIFO and setting registers\n", minor);

						/*allocate a small buffer of DMA for kernel to use*/
						//mod_desc->dma_read_offset = dma_internal_offset;
						//mod_desc->dma_write_offset = dma_internal_offset + 0x04;
						//mod_desc->kernel_reg_read = (u32*)((u64)dma_internal_offset + dma_buffer_base);   //pointer to kernel read register
						//mod_desc->kernel_reg_write = (u32*)((u64)(dma_internal_offset + 0x04) + dma_buffer_base); //pointer to kernel write register
						//verbose_printk(KERN_INFO"[ioctl_axi_stream_fifo]: current dma kernel offset:0x%x\n", minor, dma_internal_offset);
						//verbose_printk(KERN_INFO"[ioctl_axi_stream_fifo]: kernel R/W register kernel addresses:0x%p / 0x%p\n", minor, mod_desc->kernel_reg_read, mod_desc->kernel_reg_write );

						//dma_internal_offset = dma_internal_offset + 0x08;   // update the current dma buffer status (just added two 32b buffers)

						/*set the ring buff full*/
						atomic_set(mod_desc->write_ring_buf_full, 0);
						atomic_set(mod_desc->read_ring_buf_full, 0);
						/*set the pointer defaults*/
						verbose_printk(KERN_INFO"[pci_%x_ioctl]: read ring_buffer: RFU : %d RFH %d\n", minor, 0, 0);
						atomic_set(mod_desc->wtk, 0);
						atomic_set(mod_desc->wtk, 0);
						verbose_printk(KERN_INFO"[pci_%x_ioctl]: write ring_buffer: WTH: %d  WTK: %d\n", minor, 0, 0);
						atomic_set(mod_desc->rfh, 0);
						atomic_set(mod_desc->rfu, 0);

						ret = axi_stream_fifo_init(mod_desc);
						if (ret < 0)
							return ERROR;

						/*Create Threads */
						//mod_desc->thread_struct_write = create_thread_write(mod_desc);
						//mod_desc->thread_struct_read = create_thread_read(mod_desc);
					}

					break;

				default:verbose_printk(KERN_INFO"[pci_%x_ioctl]: ERROR, improper mode type!\n", minor);
			}
			break;
		default:printk(KERN_INFO"[pci_%x_ioctl]: ERROR, no command found.\n", minor);
	}
	return 0;
}




loff_t pci_llseek( struct file *filep, loff_t off, int whence)
{
	struct mod_desc * mod_desc;
	loff_t newpos;

	mod_desc = filep->private_data;

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
	verbose_llseek_printk(KERN_INFO"[pci_%x_llseek]: attempted seek %lld\n", mod_desc->minor, newpos);
	if (newpos < 0) return -EINVAL;
	filep->f_pos = newpos;
	return newpos;
}

unsigned pci_poll(struct file *filep, poll_table * pwait)
{
	struct mod_desc * mod_desc;
	int rfh;
	int rfu;
	int full;
	int d2r;
	unsigned int mask = 0;

	mod_desc = filep->private_data;

	rfh = atomic_read(mod_desc->rfh);
	rfu = atomic_read(mod_desc->rfu);
	full = atomic_read(mod_desc->write_ring_buf_full);

	d2r = data_in_buffer(rfh, rfu, full, mod_desc->dma_size);

	very_verbose_poll_printk(KERN_INFO"[pci_%x_poll]: Poll() has been entered!\n",mod_desc->minor);
	/*Register the poll table with the peripheral wait queue
	 *so that every wake_up on the wait queue will see if the
	 *peripheral has data*/

	//poll_wait(filep, &wq_periph, pwait);
	//if data in the ring buffer
	//if(data_in_buffer == 0) {		//if there is no data to read


	poll_wait(filep,&mod_desc->poll_wq,pwait);
	very_verbose_poll_printk(KERN_INFO"[pci_%x_poll]: poll_wait done!\n",mod_desc->minor);

	if (atomic_read(mod_desc->atomic_poll) || (d2r != 0) ) {
		verbose_poll_printk(KERN_INFO"[pci_%x_poll]: Interrupting Peripheral Matched or Data in ring buffer(d2r:%x)!\n",mod_desc->minor, d2r);
		/*reset the has_data flag*/
		atomic_set(mod_desc->atomic_poll, 0);
		mask |= POLLIN;
	}

	very_verbose_poll_printk(KERN_INFO"[pci_%x_poll]: Leaving Poll(0x%08x)\n",mod_desc->minor, mask);
	return mask;

}

ssize_t pci_write(struct file *filep, const char __user *buf, size_t count, loff_t *f_pos)
{
	u64 axi_dest;
   	u64 internal_offset = 0;
	struct mod_desc * mod_desc;
	size_t bytes;
	size_t bytes_written = 0;
	size_t remaining_size;
	size_t partial_count;
	int transfer_type;
	int ret;
	size_t room_till_end;
	size_t remaining;
	u32 init_write;
	u64 dma_offset_write;
	int minor;
	void* buffer;

	int wtk, wth, full;
	bytes = 0;

	//this gets the minor number of the calling file so we can map the correct AXI address to write to
	mod_desc = filep->private_data;
	minor = mod_desc->minor;

	bytes = 0;

	verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: ************************************************************************\n", minor);
	verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: ******************** WRITE TRANSACTION BEGIN  **************************\n", minor);
   switch(mod_desc->mode){
      case AXI_STREAM_FIFO :
         verbose_pci_write_printk(KERN_INFO"[pci_%x_write]:                   Attempting to transfer %zu bytes : mode AXI_STREAM_FIFO \n", minor, count);
         break;

      case AXI_STREAM_PACKET :
         verbose_pci_write_printk(KERN_INFO"[pci_%x_write]:                   Attempting to transfer %zu bytes : mode AXI_STREAM_PACKET \n", minor, count);
         break;

      case MASTER :
      case SLAVE :
         verbose_pci_write_printk(KERN_INFO"[pci_%x_write]:                   Attempting to transfer %zu bytes : mode DIRECT \n", minor, count);
         break;

      default :
         verbose_pci_write_printk(KERN_INFO"[pci_%x_write]:                   Attempting to transfer %zu bytes : mode UNKNOWN \n", minor, count);

   }
	verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: ************************************************************************\n", minor);
	verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: WRITE file struct offset: %llx\n", minor, filep->f_pos);
	verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: WRITE offset param: %llx\n", minor, *f_pos);

	if (count <= 0) {
		printk(KERN_INFO"[pci_%x_write]: ERROR count <= 0 !\n", minor);
		return ERROR;
	}

	if(mod_desc->set_dma_flag == 0) {
		ret = dma_file_init(mod_desc, dma_buffer_base, dma_buffer_size);
		if (ret < 0) {
			printk(KERN_INFO"[pci_%x_write]: !!!! DMA init FAILURE!!!!\n", minor);
			return ERROR;
		}
		printk(KERN_INFO"[pci_%x_write]: Warning - Set the DMA file size to default value %d. IOCTL SET_FILE_SIZE was never called.\n", minor, (int)mod_desc->file_size);
		mod_desc->set_dma_flag = 1;
	}

	/*Start timers for statistics*/
	if (mod_desc->start_flag == 1) {
		getnstimeofday(mod_desc->start_time);
		mod_desc->start_flag = 0;
	}

	if (atomic_read(&driver_start_flag) == 1) {
		getnstimeofday(&driver_start_time);
		atomic_set(&driver_start_flag, 0);
	}

	/*Stay until requested transmission is complete*/
	while (bytes_written < count && mod_desc->file_open) {

		if (mod_desc->mode == AXI_STREAM_FIFO || mod_desc->mode == AXI_STREAM_PACKET) {

			if (count > mod_desc->dma_size) {
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: DMA Buffer size of: 0x%zx is not large enough for transfer size:%zu bytes\n", minor, mod_desc->dma_size, count);
				return ERROR;
			}

			verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: the amount of bytes being copied to kernel: %zu\n", minor, count);

			wtk = atomic_read(mod_desc->wtk);
			wth = atomic_read(mod_desc->wth);
			full = atomic_read(mod_desc->write_ring_buf_full);

			//we are going to write the whole count + header
			while( (count + dma_byte_width + 2*sizeof(count)) > room_in_buffer(wtk, wth, full, mod_desc->dma_size) && mod_desc->file_open ) {
				/*sleep until the write thread signals priority to pci_write
				 *  (ie there is no more data for the write thread to write)
				 *  (ie there MUST be room in the ring buffer now)*/

				//wake up write thread
				atomic_set(&thread_q_write, 1);
				wake_up_interruptible(&thread_q_head_write);
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: waking up write thread a\n", minor);

				//return bytes_written;

				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: pci_write is going to sleep ZzZzZzZzZzZzZz, room in buffer: %d\n", minor, room_in_buffer(wtk, wth, full, mod_desc->dma_size));
				ret = wait_event_interruptible(pci_write_head, atomic_read(mod_desc->pci_write_q) == 1);
				atomic_set(mod_desc->pci_write_q, 0);
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: woke up the sleeping pci_write function!!\n", minor);

				wtk = atomic_read(mod_desc->wtk);
				wth = atomic_read(mod_desc->wth);
				full = atomic_read(mod_desc->write_ring_buf_full);
			}

			//if we need to wrap around the ring buffer....


			//getnstimeofday(&start_time);

			//copy the count to the ring buffer to act as the "header"
			room_till_end = (int)mod_desc->dma_size - wtk;
			if (sizeof(count) > room_till_end) {		//If the header will not fit in 1 chuck of the ring buffer (in the room till end) then just get it at the start
            wtk = 0;
         }

			verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: header memcpy(0x%p + 0x%x, 0x%p, 0x%zx)\n", minor, mod_desc->dma_write_addr, wtk, (void * )&count, sizeof(count));
			memcpy(mod_desc->dma_write_addr+wtk, (void * )&count, sizeof(count));
			wtk = get_new_ring_pointer((int)sizeof(count), wtk, (int)mod_desc->dma_size);

			room_till_end = (int)( (mod_desc->dma_size - wtk) & ~(dma_byte_width-1));              //make it divisible by dma_byte_width
			if (count > room_till_end) {		//if header size is larger than room till the end, write in two steps
				remaining = count-room_till_end;
				//write the room_till_end
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: copy_from_user 1(0x%p + 0x%x, 0x%p, 0x%zx)\n", minor, mod_desc->dma_write_addr, wtk, buf, room_till_end);
				ret = copy_from_user(mod_desc->dma_write_addr+wtk, buf, room_till_end);
				wtk = 0;																																										//end of buffer reached

				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: copy_from_user 2(0x%p + 0x%x, 0x%p, 0x%zx)\n", minor, mod_desc->dma_write_addr, wtk, buf+room_till_end, remaining);
				ret = copy_from_user(mod_desc->dma_write_addr+wtk,  buf+room_till_end, remaining);
				wtk = get_new_ring_pointer((int)remaining, wtk, (int)mod_desc->dma_size);
			} else {
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: copy_from_user(0x%p + 0x%x, 0x%p, 0x%zx)\n", minor, mod_desc->dma_write_addr, wtk, buf, count);
				ret = copy_from_user(mod_desc->dma_write_addr+wtk, buf, count);
				wtk = get_new_ring_pointer((int)count, wtk, (int)mod_desc->dma_size);
			}

			//getnstimeofday(&stop_time);
			//diff = timespec_sub((stop_time), (start_time));
			//if(diff.tv_nsec > 1000000)
			//	printk(KERN_INFO"[pci_%x_write] Copy_from_user time: %lunS\n", diff.tv_nsec);

			atomic_set(mod_desc->wtk, wtk);

			/*This says that if the wtk pointer has caught up to the WTH pointer, give priority to the WTH*/
			if(atomic_read(mod_desc->wth) == wtk) {
				atomic_set(mod_desc->write_ring_buf_full, 1);
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: ring_point_%d: ring buff priority: %d\n", minor, minor, atomic_read(mod_desc->write_ring_buf_full));
			}

			verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: write ring_buffer: WTH: %d  WTK: %d\n",minor, atomic_read(mod_desc->wth), wtk);

			/*write the mod_desc to the write FIFO*/
			if(atomic_read(mod_desc->in_write_fifo_count) == 0) {
				//debug message
				if (kfifo_len(&write_fifo) > 4) {
					verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: kfifo write stored elements: %d\n", minor, kfifo_len(&write_fifo));
				}

				if(!kfifo_is_full(&write_fifo) && mod_desc->file_open) {
					atomic_inc(mod_desc->in_write_fifo_count);
					kfifo_in_spinlocked(&write_fifo, &mod_desc, 1, &fifo_lock_write);
				}
				else {
					verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: write kfifo is full, not writing mod desc\n", minor);
				}
			}

			//wake up write thread
			atomic_set(&thread_q_write, 1);
			wake_up_interruptible(&thread_q_head_write);
			verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: waking up write thread b\n", minor);

			partial_count = count;


		} else { // Not stream

			partial_count = count - bytes_written;

			if (partial_count > mod_desc->dma_size) {
				remaining_size = count - bytes_written;
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: DMA Buffer size of: 0x%zx is not large enough for *remaining* transfer size: 0x%zx bytes\n", minor, mod_desc->dma_size, remaining_size);
				partial_count = mod_desc->dma_size;
			}

			verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: the amount of bytes being copied to kernel: %zu\n", minor, partial_count);

			ret = copy_from_user(mod_desc->dma_write_addr, (buf + bytes_written), partial_count);
			buffer = mod_desc->dma_write_addr;

			init_write = *((u32*)(mod_desc->dma_write_addr));
			dma_offset_write = (u64)mod_desc->dma_offset_write;

			verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: writing peripheral with starting value: 0x%x\n", minor, init_write);
			verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: DMA offset write value: %llx\n", minor, dma_offset_write);

			axi_dest = mod_desc->axi_addr + *f_pos + internal_offset;

			if (mod_desc->keyhole_config & 0x1)
				transfer_type = KEYHOLE_WRITE;
			else
				transfer_type = NORMAL_WRITE;

			verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: writing peripheral using a transfer_type: 0x%x,offset: %llx\n", minor, transfer_type, *f_pos);

			if (transfer_type == NORMAL_WRITE) {
				/*Check to see if write will go past the boundary*/
				if((partial_count + *f_pos) > mod_desc->file_size) {
					verbose_pci_write_printk(KERN_INFO"[user_peripheral_%x_write]: End of file overrun!\n", minor);
					partial_count = (size_t)(mod_desc->file_size - *f_pos);
					verbose_pci_write_printk(KERN_INFO"[user_peripheral_%x_write]: Only writing %zu bytes to end of file!\n", minor, partial_count);
				}
			}

			ret = data_transfer(axi_dest, buffer, partial_count, transfer_type, dma_offset_write);
			if (ret > 0) {
				printk(KERN_INFO"[pci_%x_write]: ERROR writing to User Peripheral\n", minor);
				return ERROR;
			}

			if (transfer_type == NORMAL_WRITE) {
				//*f_pos = *f_pos + partial_count;

				if (*f_pos + partial_count == mod_desc->file_size) {
					//*f_pos = 0;
					verbose_pci_write_printk(KERN_INFO"[user_peripheral_%x_write]: Resetting file pointer back to zero...\n", minor);
				} else if (*f_pos +  partial_count > mod_desc->file_size) {
					printk(KERN_INFO"[user_peripheral_%x_write]: ERROR! Wrote past the file size. This should not have happened...\n", minor);
					printk(KERN_INFO"[user_peripheral_%x_write]: Resetting file pointer back to zero...\n", minor);
					//*f_pos = 0;
					return ERROR;
				}

				verbose_pci_write_printk(KERN_INFO"[user_peripheral_%x_write]: updated file offset is: %llx\n", minor, *f_pos);
			}

		}

		bytes_written = bytes_written + partial_count;
		verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: Wrote %zu bytes in this pass.\n", minor, partial_count);

      internal_offset += partial_count;
      verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: Internal offset updated: %llu.\n", minor, internal_offset);

	}


	//file statistics
	mod_desc->tx_bytes = mod_desc->tx_bytes + bytes_written;
	atomic_add(bytes_written, &driver_tx_bytes);
	verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: total file tx byes: %d \n", minor, mod_desc->tx_bytes);



	/*always update the stop_timer*/
	getnstimeofday(mod_desc->stop_time);
	getnstimeofday(&driver_stop_time);

	verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: ************************************************************************\n", minor);
	verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: ******************** WRITE TRANSACTION END  **************************\n", minor);
	verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: Wrote a total of %zu bytes in write call.\n", minor, bytes_written);
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
   u64 internal_offset = 0;
	struct mod_desc *mod_desc;
	size_t bytes = 0;
	int transfer_type;
	int ret;
	size_t temp;
	u64 dma_offset_write;
	u64 dma_offset_read;
	int rfh, rfu, full;
	u64 d2r;
	size_t room_till_end;
	size_t remaining;
	int wake_up_flag = 0;
	int minor;
	void* buffer;
	size_t read_header_size;

	mod_desc = filep->private_data;
	minor = mod_desc->minor;
	buffer = mod_desc->dma_read_addr;

	temp = 0;

	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: ************************************************************************\n", minor);
	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: ******************** READ TRANSACTION BEGIN  **************************\n", minor);
   switch(mod_desc->mode){
      case AXI_STREAM_FIFO :
         verbose_pci_read_printk(KERN_INFO"[pci_%x_read]:                   Attempting to transfer %zu bytes : mode AXI_STREAM_FIFO \n", minor, count);
         break;

      case AXI_STREAM_PACKET :
         verbose_pci_read_printk(KERN_INFO"[pci_%x_read]:                   Attempting to transfer %zu bytes : mode AXI_STREAM_PACKET \n", minor, count);
         break;

      case MASTER :
      case SLAVE :
         verbose_pci_read_printk(KERN_INFO"[pci_%x_read]:                   Attempting to transfer %zu bytes : mode DIRECT \n", minor, count);
         break;

      default :
         verbose_pci_read_printk(KERN_INFO"[pci_%x_read]:                   Attempting to transfer %zu bytes : mode UNKNOWN \n", minor, count);

   }
	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: ************************************************************************\n", minor);

	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: Read file struct offset: %llx\n", minor, filep->f_pos);
	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: Read offset param: %llx , count requested %zd\n", minor, *f_pos, count);

	if (count <= 0) {
	  printk(KERN_INFO"[pci_%x_read]: ERROR got invalid read count %zu\n", minor,count);
		return EINVAL;
	}

	if(mod_desc->set_dma_flag == 0) {
		ret = dma_file_init(mod_desc, dma_buffer_base, dma_buffer_size);
		if (ret < 0) {
			printk(KERN_INFO"[pci_%x_read]: !!!! DMA init FAILURE!!!!.\n", minor);
			return ERROR;
		}
		printk(KERN_INFO"[pci_%x_read]: Warning - Set the DMA file size to default value %lld. IOCTL SET_FILE_SIZE was never called.\n", minor, mod_desc->file_size);
		mod_desc->set_dma_flag = 1;
	}

	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: Attempting to read %zu bytes\n", minor, count);

	if (mod_desc->start_flag == 1) {
		getnstimeofday(mod_desc->start_time);
		mod_desc->start_flag = 0;
	}

	if (atomic_read(&driver_start_flag) == 1) {
		getnstimeofday(&driver_start_time);
		atomic_set(&driver_start_flag, 0);
	}

	if (count > mod_desc->dma_size) {
		verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: Attempting to read more than the allocated DMA size of:%zu\n", minor, mod_desc->dma_size);
		verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: readjusting the read amount to:%zu\n", minor, mod_desc->dma_size);
		count = (size_t)mod_desc->dma_size;
	}


	switch(mod_desc->mode){

		case MASTER:
			//Transfer buffer from kernel space to user space at the allocated DMA region
			verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: Transferred a Master write from kernel space to user space\n" ,minor);
			ret = copy_to_user(buf, dma_master_buf[mod_desc->master_num], count);
			break;

		case AXI_STREAM_FIFO:
      case AXI_STREAM_PACKET:
			// if AXI streaming fifo set with  no interrupt then just read
			if (!mod_desc->has_interrupt_vec) {

            bytes = axi_stream_fifo_d2r(mod_desc);

				if (bytes <= 0) {
					if (bytes < 0) {
						printk(KERN_INFO"[pci_%x_read]: ERROR reading data from axi stream fifo\n" ,minor);
					}
					ret = bytes;
			} else {
					if (bytes <= count){
                  bytes = axi_stream_fifo_read_direct((size_t)bytes, mod_desc->dma_read_addr, axi_pcie_m + mod_desc->dma_offset_read, mod_desc, mod_desc->dma_size);
                  count = bytes;
					}
               else if(count < bytes) {
                  bytes = axi_stream_fifo_read_direct((size_t)count, mod_desc->dma_read_addr, axi_pcie_m+mod_desc->dma_offset_read, mod_desc, mod_desc->dma_size);
               }
               verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: copy_to_user (0x%p, 0x%p, 0x%zx)\n" ,minor, &buf, mod_desc->dma_read_addr, count);
               ret = copy_to_user(buf, mod_desc->dma_read_addr, count);
					verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: Read %zd bytes from AXI FIFO\n" ,minor ,count);
				}
				break;
			}

			/* -----------------------------------------------------------------------------------
			 *    Ring buffer Code Start  */
			/*for the ring buffer case, we just need to determine a pointer location and a size
			 * to give to the copy_to_user function */
	 	 	else {
				rfh = atomic_read(mod_desc->rfh);
				rfu = atomic_read(mod_desc->rfu);
				full = atomic_read(mod_desc->read_ring_buf_full); //The thread has full when the atomic variable is 0

				d2r = data_in_buffer(rfh, rfu, full, mod_desc->dma_size);


				//if there is data in the ring buffer
				if (d2r > 0) {
					// if there is no left over raed_header size, read the packet header  read_header_size from the ring buffer
               if(mod_desc->read_header_size == 0) {
   					room_till_end = mod_desc->dma_size - rfu;
					//If the header will not fit in 1 chuck of the ring buffer (in the room till end) then just put it at the start
   					if (sizeof(read_header_size) > room_till_end) {
                     rfu = 0;
                  }

   					verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: header copy_to_user(0x%p, 0x%p + 0x%x, 0x%zx)\n", minor, &read_header_size, mod_desc->dma_read_addr, rfu, sizeof(read_header_size));
   					memcpy(&read_header_size, mod_desc->dma_read_addr+rfu, sizeof(read_header_size));
   					rfu = get_new_ring_pointer((int)sizeof(read_header_size), rfu, (int)(mod_desc->dma_size));
				} else{
                  read_header_size = mod_desc->read_header_size;
               }

					verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: header: (0x%zx)\n" ,minor, read_header_size);

					if (read_header_size > d2r) {
						verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: ERROR read_header_size: 0x%zx is larger than data to read: 0x%llx\n" ,minor, read_header_size, d2r);
						count = 0;
						bytes = 0;
				} else if(read_header_size < count){
						count = read_header_size;
						verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: setting read count to 0x%zx\n" ,minor, read_header_size);
				} else {
                  verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: buffer size 0x%zx <  read header size 0x%zx\n" ,minor, count, read_header_size);
               }

					//if there is a packet to read
					if (count > 0) {

						room_till_end = ( (mod_desc->dma_size - rfu) & ~(dma_byte_width-1));              //make it divisible by dma_byte_width

						if(count > room_till_end) { //need to do two read since we are at the edge
							remaining = count - room_till_end;
							verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: copy_to_user 1(0x%p, 0x%p + 0x%x, 0x%zx)\n", minor, &buf, mod_desc->dma_read_addr, rfu, room_till_end);
							ret = copy_to_user(buf, mod_desc->dma_read_addr + rfu, room_till_end);

                     //extra verbose debug message
                     verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: first 4 bytes 0x%x\n", minor, *((u32*)(mod_desc->dma_read_addr+rfu)));

							rfu = 0;
																																									//end of buffer reached
							verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: copy_to_user 2(0x%p, 0x%p + 0x%x, 0x%zx)\n", minor, &buf+room_till_end, mod_desc->dma_read_addr, rfu, remaining);
							ret = copy_to_user(buf+room_till_end, mod_desc->dma_read_addr + rfu, remaining);
							rfu = get_new_ring_pointer((int)remaining, rfu, (int)mod_desc->dma_size);

					} else { 		//else we can do it in 1 read
							verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: copy_to_user(0x%p, 0x%p + 0x%x, 0x%zx)\n", minor, &buf, mod_desc->dma_read_addr, rfu, count);
							ret = copy_to_user(buf, mod_desc->dma_read_addr + rfu, count);

                     //extra verbose debug message
                     verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: first 4 bytes 0x%x\n", minor, *((u32*)(mod_desc->dma_read_addr+rfu)));

							rfu = get_new_ring_pointer((int)count, rfu, (int)mod_desc->dma_size);

						}

					//if ring buffer is more than half way full wake up the read thread after the rfu is updated.
					if (( d2r > (mod_desc->dma_size > 1) ) & back_pressure) {
						wake_up_flag = 1;
					}
					verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: bytes copied to user 0x%zx\n", minor, count);
					bytes = count;

               //if we didn't read the whole packet out then write a header back in for the next read
                  //we need to write the header before the data read out, so rfu will need to be decreased
                  //because we did not write the rfu yet, we do not have to worry about data being put there yet
               if (count < read_header_size) {
                  read_header_size -= count;
                  mod_desc->read_header_size = read_header_size;
                  verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: set mod_desc->read_header_size to (0x%zx)\n", minor, read_header_size);
					} else {
                  mod_desc->read_header_size = 0;
               }
				} else {
					verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: 0 size packet buffer\n" ,minor);
					bytes = 0;
				}

				atomic_set(mod_desc->rfu, rfu);

				//if we were full, we just read data out of the ring buffer so we are no longer full
				if(full && count > 0) {
					atomic_set(mod_desc->read_ring_buf_full, 0);
					verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: ring_point : Read full: %d\n", minor, 0);
				}
				//------------------------------------------------------------------//
				verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: read ring_buffer: RFU : %d RFH %d\n", minor, rfu, atomic_read(mod_desc->rfh));

				if (wake_up_flag == 1) {
					verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: freed up the locked ring buffer, waking up read thread.\n", minor);

					// took data out, if full we can read now
					atomic_set(&thread_q_read, 1);
					wake_up_interruptible(&thread_q_head_read);
				}

			} else {
				verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: max can read is 0.\n", minor);
				bytes = 0;
			}
			/*    Ring buffer Code End
			 * ------------------------------------------------------------------------------------*/
		}
		break;

		case SLAVE:
			// Here we will decide whether to do a zero copy DMA, or to read directly from the peripheral

			dma_offset_write = (u64)mod_desc->dma_offset_write;
			dma_offset_read = (u64)mod_desc->dma_offset_read;

			axi_dest = mod_desc->axi_addr + *f_pos + internal_offset;

			if (mod_desc->keyhole_config & 0x1)
				transfer_type = KEYHOLE_READ;
			else
				transfer_type = NORMAL_READ;

			verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: reading peripheral using a transfer_type: 0x%x \n", minor, transfer_type);
			verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: current file offset is: %llu \n", minor, *f_pos);

			temp = count + *f_pos;

			/*Check to see if read will go past the boundary*/
			if(temp > (size_t)mod_desc->file_size) {
				verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: Read will overrun the file size because \n", minor);
				verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: (the current file offset + amount to read)->(%zu) > (%llu)->file_size\n", minor, temp, mod_desc->file_size);
				count = (size_t)(mod_desc->file_size - *f_pos);
				verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: Adjusting to only reading %zu bytes to end of file!\n", minor, count);

			}
            //int data_transfer(u64 axi_address, void *buf, size_t count, int transfer_type, u64 dma_offset);
			verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: data_transfer  AXI Address: 0x%llx, buf: 0x%p + 0x%llx, len: 0x%zx\n",
						minor, axi_dest, mod_desc->dma_read_addr, (u64)mod_desc->dma_offset_read, count);
			ret = data_transfer(axi_dest, mod_desc->dma_read_addr, count, transfer_type, (u64)mod_desc->dma_offset_read);

			if (ret > 0) {
				printk(KERN_INFO"[user_peripheral_%x_read]: ERROR reading data from User Peripheral\n", minor);
				return ERROR;
			}

			if (transfer_type == NORMAL_READ) {
				//*f_pos = (loff_t)(*f_pos + (loff_t)count);
				verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: updated file offset after adding count(%zu) is: %llu\n", minor, count, *f_pos);
				verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: first 4 bytes 0x%x\n", minor,*(unsigned int *)mod_desc->dma_read_addr);

            /*
				if (*f_pos + (loff_t)(*f_pos + (loff_t)count) == mod_desc->file_size) {
					*f_pos = 0;
					verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: End of file reached.\n", minor);
					verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: Resetting file pointer back to zero...\n", minor);
					verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: updated file offset is: %llu\n", minor, *f_pos);
				} else if (*f_pos + (loff_t)(*f_pos + (loff_t)count) > mod_desc->file_size) {
					printk(KERN_INFO"[user_peripheral_%x_read]: ERROR! Read past the file size. This should not have happened...\n", minor);
					printk(KERN_INFO"[user_peripheral_%x_read]: the offset position is:%llu\n", minor, *f_pos);
					printk(KERN_INFO"[user_peripheral_%x_read]: Resetting file pointer back to zero...\n", minor);
					*f_pos = 0;
					printk(KERN_INFO"[user_peripheral_%x_read]: updated file offset is: %llu\n", minor, *f_pos);
					return ERROR;
				}
            */
			}
         verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: copy_to_user (0x%p, 0x%p, 0x%zx)\n" ,minor, &buf, mod_desc->dma_read_addr, count);

			ret = copy_to_user(buf, mod_desc->dma_read_addr, count);
			bytes = count;
			break;

		default:
			verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: mode not detected on read\n", minor);
	}


	/*eventually this will go away once we add mmap
	 * for now we copy to the appropriate file buffer*/
	//ret = copy_to_user(buf, mod_desc->dma_read_addr + ring_buf_ptr, count);

	//printk(KERN_INFO"total file rx byes: %d \n", mod_desc->rx_bytes);

	/*always update the stop_timer*/
	if (bytes > 0) {
		getnstimeofday(mod_desc->stop_time);
		getnstimeofday(&driver_stop_time);
		mod_desc->rx_bytes = mod_desc->rx_bytes + bytes;
		atomic_add(bytes, &driver_rx_bytes);
	}

	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: ************************************************************************\n", minor);
	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: ******************** READ TRANSACTION END  **************************\n", minor);
	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]:              					Bytes read : %zd\n", minor, bytes);
	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]:                        Total Bytes read : %d\n", minor, mod_desc->rx_bytes);
	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: ************************************************************************\n", minor);

	return bytes;
}
