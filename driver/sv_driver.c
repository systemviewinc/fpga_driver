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


/***********Set default values for insmod parameters***************************/
int vendor_id = PCI_VENDOR_ID_XILINX;	 /**< Insmod Patameter - PCIe vendor	*/
int device_id = 100;	/**< Insmod Parameter - PCIe specific */
int major = 241;/**< Insmod Parameter - Major number of Driver*/
uint cdma_address[CDMA_MAX_NUM] = {0xFFFFFFFF}; /**< Insmod Parameter - the AXI Address of CDMA 1*/
int cdma_count;	/**< The number of elements in cdma_address (or max value)*/
int pcie_bar_num; /**< The number of elements in pcie_ctl_address (or max value)*/
ulong pcie_ctl_address = 0xFFFFFFFF; /**< The number of elements in pcie_ctl_address (or max value)*/
ulong pcie_bar_address[BAR_MAX_NUM] = {0xFFFFFFFF};/**< Insmod Parameter - The AXI Address of the PCIe Control regs*/
ulong pcie_m_address = 0xFFFFFFFF;/**< Insmod Parameter - AXI address of data transport slave address as viewed from CDMA */
uint int_ctlr_address = 0xFFFFFFFF;/**< Insmod Parameter - AXI Address of Interrupt Controller*/
int driver_type = PCI;/**< Insmod Parameter - Driver typem either PCIe or Platform*/
int dma_system_size = 4194304;/**< Insmod Parameter - Size of DMA Allocation, max and default is 4MB*/
int dma_file_size = 4096;/**< Insmod Parameter - Currently not used, the HW buffer size is now set on a file by file basis.*/
int dma_byte_width = 8;	/**< Insmod Parameter - This parameter is the data width of the CDMA. It is used to calculate the FIFO empty value.*/
int back_pressure = 0;/**< Insmod Parameter - This parameter sets whether the READ Ring Buffer should overwrite data or backpressure to HW.*/
uint axi2pcie_bar0_size = 0xFFFFFFFF;/**< Insmod Parameter - This parameter sets the PCIE2AXI bar size for address translation parameters.*/
uint vsi_reg_intf_addr = 0xFFFFFFFF;/**< Insmod Parameter - This parameter sets the PCIE2AXI bar size for address translation parameters.*/
uint interface_crc = 0;/**< Insmod Parameter - This parameter sets whether the READ Ring Buffer should overwrite data or backpressure to HW.*/
uint interface_crc_check = 0;/**< Insmod Parameter - This parameter sets whether the READ Ring Buffer should overwrite data or backpressure to HW.*/
uint pcie_use_xdma = 1; /**< Insmod Parameter : will use XDMA instead of CDMA to move data */
//static char *pci_devName = &buffer[0];/**< Insmod Parameter - the PCIe Device Name.*/
//const char * pci_devName_const;
const char pci_devName[] = "vsi_driver"; //name of the device
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

module_param(pcie_m_address, ulong, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
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

module_param(pcie_use_xdma, uint, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);/**< Insmod Parameter */
MODULE_PARM_DESC(pcie_use_xdma, "USE XDMA Instead of CDMA");/**< Insmod Parameter */

/*****************************************************************************/

struct pci_dev * pci_dev_struct = NULL; /**<pci device struct */
struct platform_device * platform_dev_struct = NULL; /**< Platform device struct (for zynq) */
struct device *	dev_struct = NULL;

u64 axi_interr_ctrl = 0; /**< Global Variable that stores the Interrupt Controller AXI Address */
u64 axi_pcie_m; /**< Global Variable that stores the data transport IP Slave AXI Address as seen from the CDMA*/

u8 cdma_set[CDMA_MAX_NUM]; /**< Global variable that stores which CDMAs have been initialized. (currently only using 2 CDMAs) */

//int cdma_status;
int cdma_capable = 0; /**< Global variable that is a flag to tell if the driver has been initialized properly to use the CDMA(s), holds the number of cdmas init'd */
unsigned int irq_num; /**< Global variable that stores the IRQ number that is probed from the device */
int cdma_usage_cnt = 0; /**< Global variable to count the number of CDMA uses. Used for statistics gathering */

/*CDMA Semaphores*/
struct mutex cdma_sem[CDMA_MAX_NUM];

wait_queue_head_t cdma_q_head;	/**< The Sleep wait queue for CDMAs (if not polling) */
atomic_t cdma_q = ATOMIC_INIT(0); /**< The atomic conditional variable for CDMA (if not polling) */


dma_addr_t dma_addr_base; /**< The hardware DMA Allocation Address */
char * dma_buffer_base;	/**< This is the start of the DMA region virtual address */
u32 dma_current_offset;	/**< This variable holds the current offset of the DMA Allocation */
u64 dma_buffer_size = 1048576; /**< Default value for size of DMA Allocation, Max is 4MB, this is set through insmod */
u32 dma_garbage_offset;	/**< This offset memory region is used for dumping data when back pressure is not enabled */
u32 dma_garbage_size = 4096;	/**< This size of memory region is used for dumping data when back pressure is not enabled */
u32 dma_internal_offset; /**< The current offset of the internal DMA regions. The driver uses these for register R/W */
u32 dma_internal_size= 4096; /**< The size of the internal DMA regions. The driver uses these for register R/W */

int aws_config_idx = -1 ;	/**< AWS XDMA configuration bar */
int dma_max_write_size = 0;	 /**< AWS PCI/e max write size	*/
int dma_max_read_size = 0;	 /**< AWS PCI/e max read size	*/
dma_addr_t dma_m_addr[MAX_NUM_MASTERS]; /**< Used for Master DMA Allocations (currently not used) */
void * dma_master_buf[MAX_NUM_MASTERS]; /**< Used for Master DMA Allocations (currently not used) */

/* ************************************************* */

//size_t dma_size;
wait_queue_head_t wq; /**< This is the wait queue for the CDMAs, (if not polling) (currently not used) */
wait_queue_head_t wq_periph; /**< This is the wait queue for the peripherals being polled. */

// Write Thread data
wait_queue_head_t thread_q_head_write; /**< The Wait queue for the WRITE Thread */
atomic_t thread_q_write = ATOMIC_INIT(0); /**< The Wait variable for the WRITE Thread */
struct task_struct * thread_struct_write; /**< task_struct used in creation of WRITE Thread */

// Read Thread data
wait_queue_head_t thread_q_head_read; /**< The Wait Queue for the READ Thread */
atomic_t thread_q_read = ATOMIC_INIT(0); /**< The Wait variable for the READ Thread */

struct task_struct * thread_struct_read; /**< task_struct used in creation of READ Thread */

wait_queue_head_t pci_write_head; /**< The Wait Queue for the blocking/sleeping pci_write function */

#if defined(CONFIG_PCI_MSI)
#define MAX_INTERRUPTS MAX_USER_IRQ
static struct msix_entry sv_msix_entry[32];
#endif
struct bar_mapping bar_map[16];

DEFINE_KFIFO(read_fifo, struct mod_desc*, 8192); /**< sets up the global READ FIFO */
spinlock_t fifo_lock_read; /**< The Spinlock Variable for writing to the READ FIFO */

DEFINE_KFIFO(write_fifo, struct mod_desc*, 8192); /**< sets up the global WRITE FIFO */
spinlock_t fifo_lock_write; /**< The Spinlock Variable for writing to the WRITE FIFO */

/*Driver Statistics*/
atomic_t driver_tx_bytes = ATOMIC_INIT(0); /**< Global Atomic Variable for Driver Statistics */
atomic_t driver_rx_bytes = ATOMIC_INIT(0);/**< Global Atomic Variable for Driver Statistics */
atomic_t driver_start_flag = ATOMIC_INIT(0);/**< Global Atomic Variable for Driver Statistics */
atomic_t driver_stop_flag = ATOMIC_INIT(0);/**< Global Atomic Variable for Driver Statistics */
struct timespec driver_start_time;/**< Global Struct for Driver Statistics */
struct timespec driver_stop_time;/**< Global Struct Variable for Driver Statistics */

//spinlock_t mLock;

/******************************** Xilinx Register Offsets **********************************/
const u32 INT_CTRL_IER		= 0x08; /**< Interrupt Controller Register Offset, see Xilinx doc. */
const u32 INT_CTRL_MER		= 0x1c;	 /**< Interrupt Controller Register Offset, see Xilinx doc. */
const u32 INT_CTRL_ISR		= 0x00;	 /**< Interrupt Controller Register Offset, see Xilinx doc. */
const u32 INT_CTRL_IAR		= 0x0C;	 /**< Interrupt Controller Register Offset, see Xilinx doc. */


/*This is an array of interrupt structures to hold up to 8 peripherals*/
struct mod_desc * mod_desc_arr[MAX_FILE_DESC] = { 0 }; /**< This is an array of Module Descriptions that is used to be indexed by interrupt number
						 * This is how the pci_isr knows which peripheral has sent the interrupt. */

struct bar_info * bars = NULL;


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

/* functions & data used from libxdma.c */
#if (XDMA_AWS == 1)
	/******************************** AWS XDMA related **********************************/
	struct xdma_dev *xdma_dev_s;
    int xdma_c2h_num_channels = 0;
    int xdma_h2c_num_channels = 0;
	xdma_channel_tuple* xdma_channel_list = NULL;

	u32 build_vector_reg(u32 a, u32 b, u32 c, u32 d) ;
	int msi_msix_capable(struct pci_dev *dev, int type) ;
	struct xdma_dev *sv_alloc_dev_instance(struct pci_dev *pdev);
	int sv_xdma_device_open (struct pci_dev *pdev, struct xdma_dev *lro, xdma_channel_tuple **tuple_p);
	void sv_xdma_device_close(struct pci_dev *pdev, struct xdma_dev *lro, xdma_channel_tuple *tuple);

#else
/******************************** NON XDMA related **********************************/
    struct xdma_dev *lro_global = NULL;
    int xdma_c2h_num_channels = 0;
    int xdma_h2c_num_channels = 0;


#endif


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


static void aws_write_msix_vectors(char *bar)
{
	struct interrupt_regs *int_regs;
	u32 reg_val;

	int_regs = (struct interrupt_regs *) (bar + XDMA_OFS_INT_CTRL);

	/* user irq MSI-X vectors */
	reg_val = build_vector_reg(0, 1, 2, 3);
	iowrite32(reg_val, &int_regs->user_msi_vector[0]);

	reg_val = build_vector_reg(4, 5, 6, 7);
	iowrite32(reg_val, &int_regs->user_msi_vector[1]);

	reg_val = build_vector_reg(8, 9, 10, 11);
	iowrite32(reg_val, &int_regs->user_msi_vector[2]);

	reg_val = build_vector_reg(12, 13, 14, 15);
	iowrite32(reg_val, &int_regs->user_msi_vector[3]);
	iowrite32(~0, &int_regs->user_int_enable_w1s);

	/* channel irq MSI-X vectors */
	if(pcie_use_xdma) {
		reg_val = build_vector_reg(16, 17, 18, 19);
		write_register(reg_val, &int_regs->channel_msi_vector[0]);

		reg_val = build_vector_reg(20, 21, 22, 23);
		write_register(reg_val, &int_regs->channel_msi_vector[1]);
	}
}


/**
 * @brief will return 1 if this is the XDMA Config bar
 *
 * @param bar virtual address
 *
 * @return
 */
static int aws_is_config_bar(char *bar)
{
	u32 irq_id = 0;
	u32 cfg_id = 0;
	int flag = 0;
	u32 mask = 0xffff0000; /* Compare only XDMA ID's not Version number */
	struct interrupt_regs *irq_regs =
		(struct interrupt_regs *) (bar + XDMA_OFS_INT_CTRL);
	struct config_regs *cfg_regs =
		(struct config_regs *)(bar + XDMA_OFS_CONFIG);

	irq_id = ioread32(&irq_regs->identifier);
	cfg_id = ioread32(&cfg_regs->identifier);

	if(((irq_id & mask)== IRQ_BLOCK_ID) && ((cfg_id & mask)== CONFIG_BLOCK_ID)) {
		verbose_printk(KERN_INFO"[aws_is_config_bar]: BAR %p is the XDMA config BAR\n", bar);
		flag = 1;
	} else {
		verbose_printk(KERN_INFO"[aws_is_config_bar]: BAR %p is not XDMA config BAR\n", bar);
		verbose_printk(KERN_INFO"[aws_is_config_bar]: BAR %p is NOT the XDMA config BAR: 0x%x, 0x%x.\n", bar, irq_id, cfg_id);
		flag = 0;
	}

	return flag;
}

/**
 * @brief enable interrupts in the XDMA core for AWS F1 instances
 *
 * @param dev pci_dev
 */
static void aws_enable_interrupts(struct pci_dev *dev)
{
	int i;
	for (i = 0 ; i < 6; i++) {
		bar_map[i].bar_start = pci_resource_start(dev, i);
		bar_map[i].bar_len	= pci_resource_len(dev, i);
		if(!bar_map[i].bar_len) continue;
		bar_map[i].bar = pci_iomap(dev, i, bar_map[i].bar_len);
		verbose_printk(KERN_INFO"[aws_enable_interrupts:%s]: bar %d, len %d, start 0x%p, mapped @ 0x%p\n", pci_devName,
					 i, (int)bar_map[i].bar_len, (void *)bar_map[i].bar_start, bar_map[i].bar);
		if(aws_config_idx == -1 && aws_is_config_bar(bar_map[i].bar)) {
			aws_config_idx = i;
			aws_write_msix_vectors(bar_map[i].bar);
		} else {
			pci_iounmap(dev, bar_map[i].bar);
		}
	}
}

static void aws_disable_interrupts(struct pci_dev *dev)
{
	struct interrupt_regs *reg ;
	if(aws_config_idx == -1) return;
	reg = (struct interrupt_regs *) (bar_map[aws_config_idx].bar + XDMA_OFS_INT_CTRL);

	iowrite32(~0, &reg->user_int_enable_w1c);
	pci_iounmap(dev, bar_map[aws_config_idx].bar);
}

#if (XDMA_AWS == 1)

static void aws_get_max_sizes(void)
{
	char *c_bar;
	struct config_regs *cfg_regs;
	if(aws_config_idx == -1) return;
	c_bar = bar_map[aws_config_idx].bar;
	cfg_regs = (struct config_regs *)(c_bar + XDMA_OFS_CONFIG);
	// write size
	switch (cfg_regs->max_payload_size & 0x7) {
		case 0: dma_max_write_size = 128 ; break;
		case 1: dma_max_write_size = 256 ; break;
		case 2: dma_max_write_size = 512 ; break;
		case 3: dma_max_write_size = 1024; break;
		case 4: dma_max_write_size = 2048; break;
		case 5: dma_max_write_size = 4096; break;
		default: dma_max_write_size = 0;
	}
	// read size
	switch (cfg_regs->max_read_size & 0x7) {
		case 0: dma_max_read_size = 128 ; break;
		case 1: dma_max_read_size = 256 ; break;
		case 2: dma_max_read_size = 512 ; break;
		case 3: dma_max_read_size = 1024; break;
		case 4: dma_max_read_size = 2048; break;
		case 5: dma_max_read_size = 4096; break;
		default: dma_max_read_size = 0;
	}
	if(pcie_use_xdma)
		dma_max_read_size = dma_max_write_size = 4096;
	verbose_printk(KERN_INFO"[aws_disable_interrupts:%s]: dma_max_read_size %d, dma_max_write_size = %d\n", pci_devName, dma_max_read_size, dma_max_write_size);
}

#endif

/**
 * @brief This is the Probe function called for PCIe Devices
*/
#if (XDMA_AWS == 0)

static int sv_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int int_ctrl_set;
	int rc = 0, i;
	int bar_id_list[XDMA_BAR_NUM];
	//struct bar_info *bars = NULL;


	verbose_printk(KERN_INFO"[probe:%s]: ******************************** PROBE PARAMETERS *****************************************\n", pci_devName);

	verbose_printk(KERN_INFO"[probe:%s]: device_id: %d \n", pci_devName, device_id);
	verbose_printk(KERN_INFO"[probe:%s]: major: %d \n", pci_devName, major);
	if(cdma_count > CDMA_MAX_NUM) {
		verbose_printk(KERN_INFO"[probe:%s]: given more CDMA address than max allowed, setting cdma_count to %d\n", pci_devName, CDMA_MAX_NUM);
		cdma_count = CDMA_MAX_NUM;
	}
	for( i = 0; i < cdma_count; i++) {
		verbose_printk(KERN_INFO"[probe:%s]: cdma_ctrl_%d_address: 0x%x \n", pci_devName, i, cdma_address[i]);
	}
	for( i = 0; i < pcie_bar_num; i++) {
		verbose_printk(KERN_INFO"[probe:%s]: pcie_bar_%d_address: 0x%lx \n", pci_devName, i, pcie_bar_address[i]);
	}

	verbose_printk(KERN_INFO"[probe:%s]: pcie_ctl_address: 0x%lx \n", pci_devName, pcie_ctl_address);
	verbose_printk(KERN_INFO"[probe:%s]: pcie_m_address: 0x%lx \n", pci_devName, pcie_m_address);
	verbose_printk(KERN_INFO"[probe:%s]: int_ctlr_address: 0x%x \n", pci_devName, int_ctlr_address);
	verbose_printk(KERN_INFO"[probe:%s]: driver_type: 0x%x \n", pci_devName, driver_type);
	verbose_printk(KERN_INFO"[probe:%s]: dma_system_size: %d \n", pci_devName, dma_system_size);
	verbose_printk(KERN_INFO"[probe:%s]: dma_file_size: %d \n", pci_devName, dma_file_size);
	verbose_printk(KERN_INFO"[probe:%s]: dma_byte_width: %d \n", pci_devName, dma_byte_width);
	verbose_printk(KERN_INFO"[probe:%s]: back_pressure: %d \n", pci_devName, back_pressure);
	verbose_printk(KERN_INFO"[probe:%s]: axi2pcie_bar0_size : 0x%x \n", pci_devName, axi2pcie_bar0_size);
	verbose_printk(KERN_INFO"[probe:%s]: interface_crc : 0x%x \n", pci_devName, interface_crc);
	verbose_printk(KERN_INFO"[probe:%s]: interface_crc_check : 0x%x \n", pci_devName, interface_crc_check);
	verbose_printk(KERN_INFO"[probe:%s]: pcie_use_xdma : 0x%x \n", pci_devName, pcie_use_xdma);

	verbose_printk(KERN_INFO"[probe:%s]: ******************************** BEGIN PROBE ROUTINE *****************************************\n", pci_devName);

	pci_dev_struct = dev;
	dev_struct = &dev->dev;

	lro_global = alloc_dev_instance(pci_dev_struct);
	if (!lro_global) {
		verbose_printk("%s: OOM.\n", __func__);
		goto disable_device;
	}
    printk(KERN_INFO"[probe:%s]: lro_global: %p\n", pci_devName, lro_global);



	bars = kzalloc(sizeof(struct bar_info), GFP_KERNEL);
	if (!lro_global) {
		pr_info("%s: OOM.\n", __func__);
		return -ENOMEM;
	}
	bars->num_bars = 0;
	for( i = 0; i < pcie_bar_num; i++) {
		bars->pci_bar_addr[i] = pcie_bar_address[i];
	}

	if(NULL == pci_dev_struct){
		printk(KERN_INFO"[probe:%s]: struct pci_dev_struct is NULL\n", pci_devName);
		goto disable_device;
	}

	//request memory region
	if(pci_request_regions(pci_dev_struct, "vsi_driver")) {
		printk(KERN_INFO"[probe:%s]: Failed to pci_request_selected_regions\n", pci_devName);
		lro_global->regions_in_use = 1;
		goto rel_region;
	}
	lro_global->got_regions = 1;



	//map bars
	//TODO GET BAR INFO TO OUR DRIVER
	rc = sv_xdma_map_bars(lro_global, bars, pci_dev_struct);
	if (rc)
		goto unmap_bar;

	//identify_bars(lro_global, bar_id_list, num_bars, config_bar_pos);



	//enable the device
    rc = pci_enable_device(dev);
	if( rc ) {
		printk(KERN_INFO"[probe:%s]: !!!!!!!!ERROR pci_enable_device\n", pci_devName);
		goto disable_device;
	}




	//set DMA mask
	if(0 != dma_set_coherent_mask(&dev->dev, 0x00000000FFFFFFFF)){
		printk(KERN_INFO"[probe:%s]: set DMA mask error\n", pci_devName);
		goto disable_device;
	}
	verbose_printk(KERN_INFO"[probe:%s]: dma mask set\n", pci_devName);

	//enable bus mastering
	pci_set_master(dev);
	verbose_printk(KERN_INFO"[probe:%s]: pci set as master\n", pci_devName);

	//-----------------------------------------------NEW-----------------------------------------------
	if(driver_type == PCI && pcie_use_xdma) {

		//MSIX
		if(msi_msix_capable(pci_dev_struct, PCI_CAP_ID_MSIX)) {
			verbose_printk(KERN_INFO"[probe:%s]: MSIX capable XDMA\n", pci_devName);

		}
		//MSI
		else if (msi_msix_capable(pci_dev_struct, PCI_CAP_ID_MSI)) {
			verbose_printk(KERN_INFO"[probe:%s]: MSI capable XDMA\n", pci_devName);
			//fix bar mapping?

			verbose_printk("pci_enable_msi()\n");
			rc = pci_enable_msi(pci_dev_struct);
			if (rc < 0)
				verbose_printk("Couldn't enable MSI mode: rc = %d\n", rc);
			lro_global->msi_enabled = 1;

			verbose_printk("probe_engines()\n");

			rc = probe_engines(lro_global);
			if (rc) {
				verbose_printk(KERN_INFO"[probe:%s]: ERROR probe_engines\n", pci_devName);
				goto disable_device;
			}

			verbose_printk("create_interfaces()\n");

			rc = create_interfaces(lro_global);
			if (rc) {
				verbose_printk(KERN_INFO"[probe:%s]: ERROR create_interfaces\n", pci_devName);
				goto disable_device;
			}


            if(0 > request_irq(pci_dev_struct->irq, &pci_isr, 0, pci_devName, pci_dev_struct)){
    			printk(KERN_INFO"%s:[probe]request IRQ error\n", pci_devName);
    			goto disable_device;
    		}

            xdma_init_sv(lro_global);



            printk(KERN_INFO"[probe:%s]: Using IRQ#%d with 0x%p\n", pci_devName, pci_dev_struct->irq, &pci_dev_struct);

            /* enable user interrupts */
        	user_interrupts_enable(lro_global, ~0);

		}
		//LEGACY
		else {
			verbose_printk(KERN_INFO"[probe:%s]: Legacy interupts XDMA\n", pci_devName);

		}
	}
	//-----------------------------------------------END NEW-----------------------------------------------

	else if(driver_type == PCI) {
		verbose_printk(KERN_INFO"[probe:%s]: MSI capable PCI\n", pci_devName);

		// allocate vectors
		if(0 != pci_enable_msi(pci_dev_struct)) {
			printk(KERN_INFO"[probe:%s]: Alloc MSI failed\n", pci_devName);
			goto disable_device;
		}
		// take over the user interrupts, the rest will be
		// handles by xdma engine if enabled

		if(0 > request_irq(pci_dev_struct->irq, &pci_isr, IRQF_TRIGGER_RISING | IRQF_SHARED, pci_devName, pci_dev_struct)){
			printk(KERN_INFO"%s:[probe]request IRQ error\n", pci_devName);
			goto disable_device;
		}

        printk(KERN_INFO"[probe:%s]: Using IRQ#%d with 0x%p\n", pci_devName, pci_dev_struct->irq, &pci_dev_struct);


	}
	//USING AWS
	else if(driver_type == AWS) {
		printk(KERN_INFO"[probe:%s]: ERROR: driver built for not AWS\n", pci_devName);
		goto disable_device;
	}
	else{
		verbose_printk(KERN_INFO"[probe:%s]: ERROR Unknown setup\n", pci_devName);
		goto disable_device;
	}

	verbose_printk(KERN_INFO"[probe:%s]: pci enabled msi interrupt\n", pci_devName);

	//register the char device
	if(0 > register_chrdev(major, pci_devName, &pci_fops)){
		//	dynamic_major = register_chrdev(0, pci_devName, &pci_fops);
		printk(KERN_INFO"[probe:%s]: char driver not registered\n", pci_devName);
		printk(KERN_INFO"[probe:%s]: char driver major number: 0x%x\n", pci_devName, major);
		goto rmv_cdev;
	}

	if(skel_get_revision(dev) == 0x42)
		goto rmv_cdev;

	/*allocate the DMA buffer*/
	dma_buffer_base = dma_alloc_coherent(dev_struct, (size_t)dma_buffer_size, &dma_addr_base, GFP_KERNEL);

	if(NULL == dma_buffer_base) {
		printk(KERN_INFO"[probe:%s]: DMA buffer base allocation ERROR\n", pci_devName);
		printk(KERN_INFO"[probe:%s]: typical max DMA size is 4M, check your linux settings\n", pci_devName);
		goto free_alloc;
	} else {
		verbose_printk(KERN_INFO"[probe:%s]: dma kernel buffer base address is:0x%p\n", pci_devName, dma_buffer_base);
		verbose_printk(KERN_INFO"[probe:%s]: dma system memory buffer base address is:0x%p\n", pci_devName, (void *)dma_addr_base);
		dma_current_offset = PAGE_ALIGN(4096);	//we want to leave the first 4k for the kernel to use internally.
		dma_internal_offset = 0;
	}

	//set defaults
	for(i = 0; i < cdma_count; i++){
		cdma_set[i] = 0;
	}

	int_ctrl_set = 0;

	for( i = 0; i < cdma_count; i++){
		if(cdma_address[i] != 0xFFFFFFFF && !pcie_use_xdma) {
			if( cdma_init(i, cdma_address[i]) ) { //cdma_num = 1
				printk(KERN_INFO"[probe:%s]: !!!!!!!!ERROR cdma_init(%d, %08x)\n", pci_devName, i, cdma_address[i]);
					goto free_alloc;
			}
		}
	}

	if(driver_type == AWS && pcie_m_address != 0) {
		dma_addr_base = pci_map_single(pci_dev_struct, dma_buffer_base, dma_buffer_size, PCI_DMA_BIDIRECTIONAL);
		verbose_printk(KERN_INFO"[probe:%s]: pci_map_single physical 0x%p, virtual %p\n", pci_devName, (void *)dma_addr_base, (void *)dma_buffer_base);
		bars->pci_bar_addr[0] = 0x82000000;			/**< The AXI address of BAR 0 (ie common interface IP) */
		bars->pci_bar_end[0] = 0x8fffffff;		//test!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

		axi_pcie_m = dma_addr_base;
	} else if(axi2pcie_bar0_size != 0xFFFFFFFF && pcie_ctl_address != 0xFFFFFFFF) {
		axi_pcie_m = dma_addr_base % axi2pcie_bar0_size;
		if(pcie_ctl_init((u64)pcie_ctl_address, (u64)(dma_addr_base - axi_pcie_m)) ) {
			printk(KERN_INFO"[probe:%s]: !!!!!!!!ERROR pcie_ctl_init(0x%llx, 0x%llx)\n", pci_devName, (u64)pcie_ctl_address, (u64)(dma_addr_base - axi_pcie_m));
			goto free_alloc;
		}
	} else if(pcie_ctl_address != 0xFFFFFFFF) {
		printk(KERN_INFO"[probe:%s]: axi2pcie_bar0_size not set\n", pci_devName);
		printk(KERN_INFO"[probe:%s]: If this is a gen2 PCIE design address translation will fail\n", pci_devName);
		axi_pcie_m = 0;
		//return ERROR;
	} else {
		printk(KERN_INFO"[probe:%s]: !!!!!!!!ERROR axi2pcie_bar0_size and pcie_ctl_address was not set\n", pci_devName);
		axi_pcie_m = 0;
	}

	printk(KERN_INFO"[probe:%s]: DMA hardware offset(axi_pcie_m) = 0x08%llx\n", pci_devName, axi_pcie_m);

	if(int_ctlr_address != 0xFFFFFFFF) {
		int_ctlr_init((u64)int_ctlr_address);
		int_ctrl_set = 1;
	}

	for(i = 0; i < cdma_count; i++){
		verbose_printk(KERN_INFO"[probe:%s]: cdma_set[%d] = 0x%x\n", pci_devName, i, cdma_set[i]);
		cdma_capable += (cdma_set[i] & int_ctrl_set);
	}

	printk(KERN_INFO"[probe:%s]: cdma_capable = 0x%x\n", pci_devName, cdma_capable);
	printk(KERN_INFO"[probe:%s]: ***********************PROBE FINISHED SUCCESSFULLY**************************************\n", pci_devName);
	return 0;


	// rmv_cdev:
	// 	dev_present[lro->instance] = 0;
	// 	device_remove_file(&pdev->dev, &dev_attr_xdma_dev_instance);
    //
	// rmv_interface:
	// 	destroy_interfaces(lro);
	// 	rmv_engine:
	// 	remove_engines(lro);
    //
	// disable_irq:
	// 	irq_teardown(lro);
    //
    //
	// disable_msi:
	// 	if (lro->msix_enabled) {
	// 		pci_disable_msix(pdev);
	// 		lro->msix_enabled = 0;
	// 	} else if (lro->msi_enabled) {
	// 		pci_disable_msi(pdev);
	// 		lro->msi_enabled = 0;
	// 	}
	// 	if (!lro->regions_in_use)
	// 		 pci_disable_device(pci_devName);
    //

	free_alloc:
        dma_free_coherent(dev_struct, (size_t)dma_buffer_size, dma_buffer_base, dma_addr_base);

    rmv_cdev:
        unregister_chrdev(major, pci_devName);


    disable_device:
        pci_disable_device(dev);

	unmap_bar:
        unmap_bars(lro_global, dev);


	rel_region:
		pci_release_regions(dev);


		printk("probe() returning %d\n", rc);
		return -1;



}

#else

static int sv_pci_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int int_ctrl_set;
	int rc = 0, i;

	verbose_printk(KERN_INFO"[probe:%s]: ******************************** PROBE PARAMETERS *****************************************\n", pci_devName);

	verbose_printk(KERN_INFO"[probe:%s]: device_id: %d \n", pci_devName, device_id);
	verbose_printk(KERN_INFO"[probe:%s]: major: %d \n", pci_devName, major);
	if(cdma_count > CDMA_MAX_NUM) {
		verbose_printk(KERN_INFO"[probe:%s]: given more CDMA address than max allowed, setting cdma_count to %d\n", pci_devName, CDMA_MAX_NUM);
		cdma_count = CDMA_MAX_NUM;
	}
	for( i = 0; i < cdma_count; i++) {
		verbose_printk(KERN_INFO"[probe:%s]: cdma_ctrl_%d_address: 0x%x \n", pci_devName, i, cdma_address[i]);
	}
	for( i = 0; i < pcie_bar_num; i++) {
		verbose_printk(KERN_INFO"[probe:%s]: pcie_bar_%d_address: 0x%lx \n", pci_devName, i, pcie_bar_address[i]);
	}

	verbose_printk(KERN_INFO"[probe:%s]: pcie_ctl_address: 0x%lx \n", pci_devName, pcie_ctl_address);
	verbose_printk(KERN_INFO"[probe:%s]: pcie_m_address: 0x%lx \n", pci_devName, pcie_m_address);
	verbose_printk(KERN_INFO"[probe:%s]: int_ctlr_address: 0x%x \n", pci_devName, int_ctlr_address);
	verbose_printk(KERN_INFO"[probe:%s]: driver_type: 0x%x \n", pci_devName, driver_type);
	verbose_printk(KERN_INFO"[probe:%s]: dma_system_size: %d \n", pci_devName, dma_system_size);
	verbose_printk(KERN_INFO"[probe:%s]: dma_file_size: %d \n", pci_devName, dma_file_size);
	verbose_printk(KERN_INFO"[probe:%s]: dma_byte_width: %d \n", pci_devName, dma_byte_width);
	verbose_printk(KERN_INFO"[probe:%s]: back_pressure: %d \n", pci_devName, back_pressure);
	verbose_printk(KERN_INFO"[probe:%s]: axi2pcie_bar0_size : 0x%x \n", pci_devName, axi2pcie_bar0_size);
	verbose_printk(KERN_INFO"[probe:%s]: interface_crc : 0x%x \n", pci_devName, interface_crc);
	verbose_printk(KERN_INFO"[probe:%s]: interface_crc_check : 0x%x \n", pci_devName, interface_crc_check);
	verbose_printk(KERN_INFO"[probe:%s]: pcie_use_xdma : 0x%x \n", pci_devName, pcie_use_xdma);

	verbose_printk(KERN_INFO"[probe:%s]: ******************************** BEGIN PROBE ROUTINE *****************************************\n", pci_devName);

	pci_dev_struct = dev;
	dev_struct = &dev->dev;

	lro_global = alloc_dev_instance(pci_dev_struct);
	if (!lro_global) {
		verbose_printk("%s: OOM.\n", __func__);
		goto disable_device;
	}

	if(NULL == pci_dev_struct){
		printk(KERN_INFO"[probe:%s]: struct pci_dev_struct is NULL\n", pci_devName);
		goto rel_region;
	}

	//request memory region
	if(pci_request_regions(pci_dev_struct, "vsi_driver")) {
		printk(KERN_INFO"[probe:%s]: Failed to pci_request_selected_regions\n", pci_devName);
		return ERROR;
	}


	//map bars
    //
	// num_bars = sv_map_bars(pci_dev_struct, pcie_bar_address, pcie_bar_num);
    //
	// if(num_bars <= 0) {
	// 	printk(KERN_INFO"[probe:%s]: Failed to map bars\n", pci_devName);
	// 	goto unmap_bar;
	// }

	//enable the device
    rc = pci_enable_device(dev);
	if( rc ) {
		printk(KERN_INFO"[probe:%s]: !!!!!!!!ERROR pci_enable_device\n", pci_devName);
		goto disable_device;
	}

	//set DMA mask
	if(0 != dma_set_coherent_mask(&dev->dev, 0x00000000FFFFFFFF)){
		printk(KERN_INFO"[probe:%s]: set DMA mask error\n", pci_devName);
		goto disable_device;
	}
	verbose_printk(KERN_INFO"[probe:%s]: dma mask set\n", pci_devName);

	//enable bus mastering
	pci_set_master(dev);
	verbose_printk(KERN_INFO"[probe:%s]: pci set as master\n", pci_devName);

	//-----------------------------------------------NEW-----------------------------------------------
	if(driver_type == PCI && pcie_use_xdma) {

		printk(KERN_INFO"[probe:%s]: ERROR: driver built for AWS\n", pci_devName);
		goto disable_device;

	else if(driver_type == PCI) {
		verbose_printk(KERN_INFO"[probe:%s]: MSI capable PCI\n", pci_devName);

		// allocate vectors
		if(0 != pci_enable_msi(pci_dev_struct)) {
			printk(KERN_INFO"[probe:%s]: Alloc MSI failed\n", pci_devName);
			goto disable_device;
		}
		// take over the user interrupts, the rest will be
		// handles by xdma engine if enabled

		if(0 > request_irq(pci_dev_struct->irq, &pci_isr, IRQF_TRIGGER_RISING | IRQF_SHARED, pci_devName, pci_dev_struct)){
			printk(KERN_INFO"%s:[probe]request IRQ error\n", pci_devName);
			goto disable_device;
		}

        printk(KERN_INFO"[probe:%s]: Using IRQ#%d with 0x%p\n", pci_devName, pci_dev_struct->irq, &pci_dev_struct);


	}
	//USING AWS
	else if(driver_type == AWS) {
		if(msi_msix_capable(pci_dev_struct, PCI_CAP_ID_MSIX)) {
			// do AWS Specific stuff
			int i;
			int req_nvec = MAX_NUM_ENGINES + MAX_USER_IRQ;
			verbose_printk(KERN_INFO"[probe:%s]: MSI-X capable AWS\n", pci_devName);
			for (i = 0 ; i < req_nvec ; i++)
				sv_msix_entry[i].entry = i;
			// request all vectors

            #if LINUX_VERSION_CODE < KERNEL_VERSION(4, 12, 0)
                if (pci_enable_msix(pci_dev_struct, &sv_msix_entry, req_nvec))
            #else
                if (pci_enable_msix_exact(pci_dev_struct, &sv_msix_entry, req_nvec))
            #endif

				printk(KERN_INFO"[probe:%s]: Enable MSI-X failed\n", pci_devName);
				goto disable_device;
			}
			// take over the user interrupts, the rest will be
			// handles by xdma engine if enabled
			for (i = 0 ; i < MAX_INTERRUPTS ; i++) {
				printk(KERN_INFO"[probe:%s]: MSI-X requesting IRQ #%d for entry %d\n", pci_devName, sv_msix_entry[i].vector, sv_msix_entry[i].entry);
				if(0 > request_irq(sv_msix_entry[i].vector, pci_isr, 0 , pci_devName, pci_dev_struct)) {
					printk(KERN_INFO"[probe:%s]: MSI-X request_irq failed\n", pci_devName);
					goto disable_device;
				}
                printk(KERN_INFO"[probe:%s]: Using IRQ#%d with 0x%p\n", pci_devName, pci_dev_struct->irq, &pci_dev_struct);

			}
			aws_enable_interrupts(pci_dev_struct);
			aws_get_max_sizes();
			//if(pcie_use_xdma) {
			if(!(xdma_dev_s = sv_alloc_dev_instance(pci_dev_struct))) {
				printk(KERN_INFO"[probe:%s]: cannot allocate xdma\n", pci_devName);
				goto disable_device;
			}
			xdma_dev_s->sv_driver = 1; /* tell xdma that SV driver is in control */
			for (i = 0 ; i < req_nvec ; i++) {
				memcpy(&xdma_dev_s->entry[i], &sv_msix_entry[i], sizeof(xdma_dev_s->entry[i]));
				printk(KERN_INFO"[probe:%s]: entry %d, vector IRQ#%d\n", pci_devName, i, xdma_dev_s->entry[i].vector);
			}

			xdma_c2h_num_channels = xdma_h2c_num_channels = sv_xdma_device_open(pci_dev_struct, xdma_dev_s, &xdma_channel_list);
			if(xdma_num_xdma_c2h_num_channelschannels < 0) {
				printk(KERN_INFO"[probe:%s]: sv_xdma_device_open failed\n", pci_devName);
				goto disable_device;
			}

			xdma_init_sv(xdma_c2h_num_channels);
			verbose_printk(KERN_INFO"[probe:%s]: xdma initialized with %d channels\n", pci_devName, xdma_c2h_num_channels);
			//}
		} else if (msi_msix_capable(pci_dev_struct, PCI_CAP_ID_MSI)) {

			//TODO update this to AWS code

			verbose_printk(KERN_INFO"[probe:%s]: MSI capable\n", pci_devName);
			if(0 > pci_enable_msi(pci_dev_struct)){
				printk(KERN_INFO"[probe:%s]: MSI enable error\n", pci_devName);
				goto disable_device;
			}

			if(0 > request_irq(pci_dev_struct->irq, &pci_isr, 0, pci_devName, pci_dev_struct)){
				printk(KERN_INFO"[probe:%s]: MSI request_irq failed\n", pci_devName);
				goto disable_device;
			}
            printk(KERN_INFO"[probe:%s]: Using IRQ#%d with 0x%p\n", pci_devName, pci_dev_struct->irq, &pci_dev_struct);


			aws_enable_interrupts(pci_dev_struct);
			aws_get_max_sizes();
			//if(pcie_use_xdma) {
			if(!(xdma_dev_s = sv_alloc_dev_instance(pci_dev_struct))) {
				printk(KERN_INFO"[probe:%s]: cannot allocate xdma\n", pci_devName);
				goto disable_device;
			}
			xdma_dev_s->sv_driver = 1; /* tell xdma that SV driver is in control */
			// for (i = 0 ; i < req_nvec ; i++) {
			// 	memcpy(&xdma_dev_s->entry[i], &sv_msix_entry[i], sizeof(xdma_dev_s->entry[i]));
			// 	printk(KERN_INFO"[probe:%s]: entry %d, vector IRQ#%d\n", pci_devName, i, xdma_dev_s->entry[i].vector);
			// }

			xdma_c2h_num_channels = xdma_h2c_num_channels = sv_xdma_device_open(pci_dev_struct, xdma_dev_s, &xdma_channel_list);
			if(xdma_c2h_num_channels < 0) {
				printk(KERN_INFO"[probe:%s]: sv_xdma_device_open failed\n", pci_devName);
				goto disable_device;
			}

			xdma_init_sv(xdma_c2h_num_channels);
			verbose_printk(KERN_INFO"[probe:%s]: xdma initialized with %d channels\n", pci_devName, xdma_c2h_num_channels);

		} else {
			printk(KERN_INFO"[probe:%s]: MSI/MSI-X not detected - using legacy interrupts\n", pci_devName);
			if(0 > request_irq(pci_dev_struct->irq, &pci_isr, IRQF_TRIGGER_RISING | IRQF_SHARED, pci_devName, pci_dev_struct)){
				printk(KERN_INFO"[probe:%s]: request IRQ error\n", pci_devName);
				goto disable_device;
			}
            printk(KERN_INFO"[probe:%s]: Using IRQ#%d with 0x%p\n", pci_devName, pci_dev_struct->irq, &pci_dev_struct);

		}
	}
	else{
		verbose_printk(KERN_INFO"[probe:%s]: ERROR Unknown setup\n", pci_devName);
		goto disable_device;
	}

	verbose_printk(KERN_INFO"[probe:%s]: pci enabled msi interrupt\n", pci_devName);

	//register the char device
	if(0 > register_chrdev(major, pci_devName, &pci_fops)){
		//	dynamic_major = register_chrdev(0, pci_devName, &pci_fops);
		printk(KERN_INFO"[probe:%s]: char driver not registered\n", pci_devName);
		printk(KERN_INFO"[probe:%s]: char driver major number: 0x%x\n", pci_devName, major);
		goto rmv_cdev;
	}

	if(skel_get_revision(dev) == 0x42)
		goto rmv_cdev;

	/*allocate the DMA buffer*/
	dma_buffer_base = dma_alloc_coherent(dev_struct, (size_t)dma_buffer_size, &dma_addr_base, GFP_KERNEL);

	if(NULL == dma_buffer_base) {
		printk(KERN_INFO"[probe:%s]: DMA buffer base allocation ERROR\n", pci_devName);
		printk(KERN_INFO"[probe:%s]: typical max DMA size is 4M, check your linux settings\n", pci_devName);
		goto free_alloc;
	} else {
		verbose_printk(KERN_INFO"[probe:%s]: dma kernel buffer base address is:0x%p\n", pci_devName, dma_buffer_base);
		verbose_printk(KERN_INFO"[probe:%s]: dma system memory buffer base address is:0x%p\n", pci_devName, (void *)dma_addr_base);
		dma_current_offset = 4096;	//we want to leave the first 4k for the kernel to use internally.
		dma_internal_offset = 0;
	}

	//set defaults
	for(i = 0; i < cdma_count; i++){
		cdma_set[i] = 0;
	}

	int_ctrl_set = 0;

	for( i = 0; i < cdma_count; i++){
		if(cdma_address[i] != 0xFFFFFFFF && !pcie_use_xdma) {
			if( cdma_init(i, cdma_address[i]) ) { //cdma_num = 1
				printk(KERN_INFO"[probe:%s]: !!!!!!!!ERROR cdma_init(%d, %08x)\n", pci_devName, i, cdma_address[i]);
					goto free_alloc;
			}
		}
	}

	if(driver_type == AWS && pcie_m_address != 0) {
		dma_addr_base = pci_map_single(pci_dev_struct, dma_buffer_base, dma_buffer_size, PCI_DMA_BIDIRECTIONAL);
		verbose_printk(KERN_INFO"[probe:%s]: pci_map_single physical 0x%p, virtual %p\n", pci_devName, (void *)dma_addr_base, (void *)dma_buffer_base);
		bars->pci_bar_addr[0] = 0x82000000;			/**< The AXI address of BAR 0 (ie common interface IP) */
		bars->pci_bar_end[0] = 0x8fffffff;		//test!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

		axi_pcie_m = dma_addr_base;
	} else if(axi2pcie_bar0_size != 0xFFFFFFFF && pcie_ctl_address != 0xFFFFFFFF) {
		axi_pcie_m = dma_addr_base % axi2pcie_bar0_size;
		if(pcie_ctl_init((u64)pcie_ctl_address, (u64)(dma_addr_base - axi_pcie_m)) ) {
			printk(KERN_INFO"[probe:%s]: !!!!!!!!ERROR pcie_ctl_init(0x%llx, 0x%llx)\n", pci_devName, (u64)pcie_ctl_address, (u64)(dma_addr_base - axi_pcie_m));
			goto free_alloc;
		}
	} else if(pcie_ctl_address != 0xFFFFFFFF) {
		printk(KERN_INFO"[probe:%s]: axi2pcie_bar0_size not set\n", pci_devName);
		printk(KERN_INFO"[probe:%s]: If this is a gen2 PCIE design address translation will fail\n", pci_devName);
		axi_pcie_m = 0;
		//return ERROR;
	} else {
		printk(KERN_INFO"[probe:%s]: !!!!!!!!ERROR axi2pcie_bar0_size and pcie_ctl_address was not set\n", pci_devName);
		axi_pcie_m = 0;
	}

	printk(KERN_INFO"[probe:%s]: DMA hardware offset(axi_pcie_m) = 0x08%llx\n", pci_devName, axi_pcie_m);

	if(int_ctlr_address != 0xFFFFFFFF) {
		int_ctlr_init((u64)int_ctlr_address);
		int_ctrl_set = 1;
	}

	for(i = 0; i < cdma_count; i++){
		verbose_printk(KERN_INFO"[probe:%s]: cdma_set[%d] = 0x%x\n", pci_devName, i, cdma_set[i]);
		cdma_capable += (cdma_set[i] & int_ctrl_set);
	}

	printk(KERN_INFO"[probe:%s]: cdma_capable = 0x%x\n", pci_devName, cdma_capable);
	printk(KERN_INFO"[probe:%s]: ***********************PROBE FINISHED SUCCESSFULLY**************************************\n", pci_devName);
	return 0;


	// rmv_cdev:
	// 	dev_present[lro->instance] = 0;
	// 	device_remove_file(&pdev->dev, &dev_attr_xdma_dev_instance);
    //
	// rmv_interface:
	// 	destroy_interfaces(lro);
	// 	rmv_engine:
	// 	remove_engines(lro);
    //
	// disable_irq:
	// 	irq_teardown(lro);
    //
    //
	// disable_msi:
	// 	if (lro->msix_enabled) {
	// 		pci_disable_msix(pdev);
	// 		lro->msix_enabled = 0;
	// 	} else if (lro->msi_enabled) {
	// 		pci_disable_msi(pdev);
	// 		lro->msi_enabled = 0;
	// 	}
	// 	if (!lro->regions_in_use)
	// 		 pci_disable_device(pci_devName);
    //

	free_alloc:
        dma_free_coherent(dev_struct, (size_t)dma_buffer_size, dma_buffer_base, dma_addr_base);

    rmv_cdev:
        unregister_chrdev(major, pci_devName);


    disable_device:
        pci_disable_device(dev);

	unmap_bar:
		unmap_bars(lro_global, pdev);
	rel_region:
		pci_release_regions(dev);


		printk("probe() returning %d\n", rc);
		return -1;



}

#endif


/**
 * @brief This is the Probe function called for Platform Devices (ie Zynq)
*/
static int sv_plat_probe(struct platform_device *pdev)
{
	struct resource * resource;
	int int_ctrl_set;
	int i;

	verbose_printk(KERN_INFO"[probe:%s]: ******************************** PROBE PARAMETERS *****************************************\n", pci_devName);

	verbose_printk(KERN_INFO"[probe:%s]: device_id: %d \n", pci_devName, device_id);
	verbose_printk(KERN_INFO"[probe:%s]: major: %d \n", pci_devName, major);
	if(cdma_count > CDMA_MAX_NUM) {
		verbose_printk(KERN_INFO"[probe:%s]: given more CDMA address than max allowed, setting cdma_count to %d\n", pci_devName, CDMA_MAX_NUM);
		cdma_count = CDMA_MAX_NUM;
	}
	for( i = 0; i < cdma_count; i++) {
		verbose_printk(KERN_INFO"[probe:%s]: cdma_ctrl_%d_address: 0x%x \n", pci_devName, i, cdma_address[i]);
	}
	for( i = 0; i < pcie_bar_num; i++) {
		verbose_printk(KERN_INFO"[probe:%s]: pcie_bar_%d_address: 0x%lx \n", pci_devName, i, pcie_bar_address[i]);
	}
	verbose_printk(KERN_INFO"[probe:%s]: pcie_ctl_address: 0x%lx \n", pci_devName, pcie_ctl_address);
	verbose_printk(KERN_INFO"[probe:%s]: pcie_m_address: 0x%lx \n", pci_devName, pcie_m_address);
	verbose_printk(KERN_INFO"[probe:%s]: int_ctlr_address: 0x%x \n", pci_devName, int_ctlr_address);
	verbose_printk(KERN_INFO"[probe:%s]: driver_type: 0x%x \n", pci_devName, driver_type);
	verbose_printk(KERN_INFO"[probe:%s]: dma_system_size: %d \n", pci_devName, dma_system_size);
	verbose_printk(KERN_INFO"[probe:%s]: dma_file_size: %d \n", pci_devName, dma_file_size);
	verbose_printk(KERN_INFO"[probe:%s]: dma_byte_width: %d \n", pci_devName, dma_byte_width);
	verbose_printk(KERN_INFO"[probe:%s]: back_pressure: %d \n", pci_devName, back_pressure);
	verbose_printk(KERN_INFO"[probe:%s]: axi2pcie_bar0_size : 0x%x \n", pci_devName, axi2pcie_bar0_size);
	verbose_printk(KERN_INFO"[probe:%s]: interface_crc : 0x%x \n", pci_devName, interface_crc);
	verbose_printk(KERN_INFO"[probe:%s]: interface_crc_check : 0x%x \n", pci_devName, interface_crc_check);
	verbose_printk(KERN_INFO"[probe:%s]: pcie_use_xdma : 0x%x \n", pci_devName, pcie_use_xdma);

	verbose_printk(KERN_INFO"[probe:%s]: ******************************** BEGIN PROBE ROUTINE *****************************************\n", pci_devName);

	platform_dev_struct = pdev;
	dev_struct = &pdev->dev;

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if(!resource)	 //if null
	{
		printk(KERN_INFO"[probe:%s]: platform_get_resource error\n", pci_devName);
		return ERROR;
	}


	for(i = 0; i < BAR_MAX_NUM; i++){

		//look for a second resource
		resource = platform_get_resource(pdev, IORESOURCE_MEM, i);

		if(!resource)	 //if null
		{
			printk(KERN_INFO"[probe:%s]: platform_get_resource for bar %d not found\n", pci_devName, i);
			bars->pci_bar_addr[i] = 0;
			bars->pci_bar_end[i] = 0;
		} else {
			//get the control memory size
			bars->pci_bar_addr[i] = resource->start;
			bars->pci_bar_end[i] = resource->end;
			printk(KERN_INFO"[probe:%s]: platform name is: %s\n", pci_devName, resource->name);
			printk(KERN_INFO"[probe:%s]: platform bar %d size is:0x%lx\n", pci_devName, i, (unsigned long)resource_size(resource));
			printk(KERN_INFO"[probe:%s]: platform bar %d start is:%lx end is:%lx\n", pci_devName, i, bars->pci_bar_addr[i], bars->pci_bar_end[i]);
			//map the hardware space to virtual space
			bars->pci_bar_vir_addr[i] = devm_ioremap_resource(&pdev->dev, resource);
			if(IS_ERR(bars->pci_bar_vir_addr[i])){
				printk(KERN_INFO"[probe:%s]: ioremap error when mapping to virtual address %d \n", pci_devName, i);
				return ERROR;
			}
			printk(KERN_INFO"[probe:%s]: pci bar %d virtual address base is:0x%p\n", pci_devName, i, bars->pci_bar_vir_addr[i]);
			bars->num_bars++;
		}

	}
	verbose_printk(KERN_INFO"[probe:%s]: num_bars : 0x%x \n", pci_devName, bars->num_bars);


	//set DMA mask
	if(0 != dma_set_mask(&pdev->dev, 0x00000000ffffffff)){
		printk(KERN_INFO"[probe:%s]: set DMA mask error\n", pci_devName);
		return ERROR;
	}
	printk(KERN_INFO"[probe:%s]: dma mask set\n", pci_devName);

	printk(KERN_INFO"[probe:%s]: interrupt controller initialized\n", pci_devName);

	irq_num = platform_get_irq(pdev, 0);
	printk(KERN_INFO"[probe:%s]: IRQ number is:%d\n", pci_devName, irq_num);

	printk(KERN_INFO"[probe:%s]: IRQ request complete\n", pci_devName);

	//register the char device
	if(0 > register_chrdev(major, "sv_driver", &pci_fops)){
		printk(KERN_INFO"[probe:%s]: char driver not registered\n", pci_devName);
		return ERROR;
	}

	printk(KERN_INFO"[probe:%s]: register device complete going to alloc %llx byte for dma\n", pci_devName, dma_buffer_size);

	/*allocate the DMA buffer*/
	dma_buffer_base = dma_alloc_coherent(dev_struct, (size_t)dma_buffer_size, &dma_addr_base, GFP_KERNEL);
	if(NULL == dma_buffer_base) {
		printk(KERN_INFO"[probe:%s]: DMA buffer base allocation ERROR\n", pci_devName);
		return ERROR;
	} else {
		printk(KERN_INFO"[probe:%s]: dma buffer base address is:0x%p\n", pci_devName, dma_buffer_base);
		printk(KERN_INFO"[probe:%s]: dma system memory buffer base address is:%llx\n", pci_devName, (u64)dma_addr_base);
		printk(KERN_INFO"[probe:%s]: dma system memory buffer size is:%llx\n", pci_devName, (u64)dma_buffer_size);
		//we want to leave the first 4k for the kernel to use internally on file registers.
		//the second 4k is used as a throw away buffer for data drops.
		//dma_current_offset = 4096;
		dma_internal_offset = 0;
		dma_garbage_offset = dma_internal_size;
		dma_current_offset = dma_internal_size + dma_garbage_size;
	}
	printk(KERN_INFO"[probe:%s]: alloc coherent complete\n", pci_devName);

	//set defaults
	for(i = 0; i < cdma_count; i++){
		cdma_set[i] = 0;
	}

	int_ctrl_set = 0;
	printk(KERN_INFO"[probe:%s]: cdma_address	 size (%d) value 0x%x\n", pci_devName, (int)sizeof(cdma_address[0]), (unsigned int)cdma_address[0]);
	printk(KERN_INFO"[probe:%s]: pcie_m_address size (%d) value 0x%llx\n", pci_devName, (int)sizeof(pcie_m_address), (u64)pcie_m_address);
	printk(KERN_INFO"[probe:%s]: cdma_2_address size (%d) value 0x%x\n", pci_devName, (int)sizeof(cdma_address[1]), (unsigned int)cdma_address[1]);
	printk(KERN_INFO"[probe:%s]: int_ctlr_address size (%d) value 0x%x\n", pci_devName, (int)sizeof(int_ctlr_address), (unsigned int)int_ctlr_address);
	// these addresses should alayws be in the 32 bit range
	cdma_address[0]	 &= 0xFFFFFFFF;
	pcie_m_address	&= 0xFFFFFFFF;
	cdma_address[1]	&= 0xFFFFFFFF;
	int_ctlr_address &= 0xFFFFFFFF;

	for( i = 0; i < cdma_count; i++){
		if(cdma_address[i] != 0xFFFFFFFF) {
			if( cdma_init(i, cdma_address[i]) ) { //cdma_num = 1
				printk(KERN_INFO"[probe:%s]: !!!!!!!!ERROR cdma_init(%d, %08x)\n", pci_devName, i, cdma_address[i]);
				return ERROR;
			}
		}
	}

	if(int_ctlr_address != 0xFFFFFFFF) {
		int_ctlr_init((u64)int_ctlr_address);
		int_ctrl_set = 1;
	}

	/*ARM works a little differently for DMA than PCIe in that that translation
	 * is not handled by the core by writing to a register. For Zynq, the axi to DDR address
	 * mapping is 1-1 and should be written directly to the returned DMA handle */

	//request IRQ last
	if(0 > request_irq(irq_num, &pci_isr, IRQF_TRIGGER_RISING | IRQF_SHARED, pci_devName, pdev)){
		printk(KERN_INFO"[probe:%s]: request IRQ error\n", pci_devName);
		return ERROR;
	}
    printk(KERN_INFO"[probe:%s]: Using IRQ#%d with 0x%p\n", pci_devName, pci_dev_struct->irq, &pci_dev_struct);

	if(pcie_use_xdma){
		verbose_axi_fifo_write_printk(KERN_INFO"[probe:%s]: pcie_use_xdma is not 0 on platform\n", pci_devName);
		verbose_axi_fifo_write_printk(KERN_INFO"[probe:%s]: setting pcie_use_xdma to 0\n", pci_devName);
		pcie_use_xdma = 0;
	}

	axi_pcie_m = (u64)dma_addr_base;	//cdma 1

	for(i = 0; i < cdma_count; i++){
		verbose_printk(KERN_INFO"[probe:%s]: cdma_set[%d] = 0x%x\n", pci_devName, i, cdma_set[i]);
		cdma_capable += ( cdma_set[i] & int_ctrl_set ) ;
	}


	printk(KERN_INFO"[probe:%s]: cdma_capable = 0x%x\n", pci_devName, cdma_capable);

	printk(KERN_INFO"[probe:%s]: ***********************PROBE FINISHED SUCCESSFULLY**************************************\n", pci_devName);
	return 0;
}

static void sv_pci_remove(struct pci_dev *dev)
{

	int i;
	// clean up any allocated resources and stuff here.
	printk(KERN_INFO"%s[sv_pci_remove]: PCIE remove\n", pci_devName);
	// free and disable IRQ
#if defined(CONFIG_PCI_MSI)

	if(driver_type == PCI) {
		printk(KERN_INFO"[probe:%s]: MSI capable PCI remove\n", pci_devName);
		free_irq(dev->irq, dev);
		printk(KERN_INFO"%s[sv_pci_remove]: free_irq done\n", pci_devName);
		pci_disable_msi(dev);
		printk(KERN_INFO"%s[sv_pci_remove]: MSI disabled\n", pci_devName);

	}
	else {
		if(msi_msix_capable(dev, PCI_CAP_ID_MSIX)) {
			int i ;
			for (i = 0 ; i < MAX_INTERRUPTS; i++)
				free_irq (sv_msix_entry[i].vector, dev);
			printk(KERN_INFO"%s[sv_pci_remove]: free_irq done\n", pci_devName);
			// do AWS Specific interrupts
			if(driver_type == AWS) {
				aws_disable_interrupts(pci_dev_struct);
				if(pcie_use_xdma) {
#if (XDMA_AWS == 1)
					sv_xdma_device_close(pci_dev_struct, xdma_dev_s, xdma_channel_list);
#endif
				}
			}
			pci_disable_msix(dev);
			printk(KERN_INFO"%s[sv_pci_remove]: MSI-X disabled\n", pci_devName);
		} else {
			free_irq(dev->irq, dev);
			printk(KERN_INFO"%s[sv_pci_remove]: free_irq done\n", pci_devName);
			pci_disable_msi(dev);
			printk(KERN_INFO"%s[sv_pci_remove]: MSI disabled\n", pci_devName);
		}
	}
#else
	free_irq(pci_dev_struct->irq, pci_dev_struct);
	printk(KERN_INFO"%s[sv_pci_remove]: free_irq_done\n", pci_devName);
#endif

	/*Destroy the read thread*/
	printk(KERN_INFO"[sv_pci_remove]: Stopping read thead\n");
	atomic_set(&thread_q_read, 1);
	if(thread_struct_read) {
		while(kthread_stop(thread_struct_read)<0) {
            wake_up_interruptible(&thread_q_head_read);
			printk(KERN_INFO"[sv_pci_remove]: Read thread failed to stop, attemping again\n");
			atomic_set(&thread_q_read, 1);
			wake_up_interruptible(&thread_q_head_read);
		}
        put_task_struct(thread_struct_read);
	}
	printk(KERN_INFO"[sv_pci_remove]: Read Thread Destroyed\n");

	/*Destroy the write thread*/
	printk(KERN_INFO"[sv_pci_remove]: Stopping write thead\n");
	atomic_set(&thread_q_write, 1);
	if(thread_struct_write) {
        wake_up_interruptible(&thread_q_head_write);
		while(kthread_stop(thread_struct_write)<0) {
			printk(KERN_INFO"[sv_pci_remove]: Write thread failed to stop, attemping again\n");
			atomic_set(&thread_q_write, 1);
			wake_up_interruptible(&thread_q_head_write);
		}
        put_task_struct(thread_struct_write);
	}
	printk(KERN_INFO"[sv_pci_remove]: Write Thread Destroyed\n");

	unregister_chrdev(major, pci_devName);
	for(i = 0; i < bars->num_bars; i++){
			printk(KERN_INFO"[sv_pci_remove]: unmapping pci bar %d\n", i);
			pci_iounmap(dev, bars->pci_bar_vir_addr[i]);
	}
	pci_release_regions(dev);
	dma_free_coherent(dev_struct, (size_t)dma_buffer_size, dma_buffer_base, dma_addr_base);

	printk(KERN_INFO"[sv_pci_remove]: ***********************PCIE DEVICE REMOVED**************************************\n");

}

static int sv_plat_remove(struct platform_device *pdev)
{
	int i;
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
	for(i = 0; i < bars->num_bars; i++){
		iounmap(bars->pci_bar_vir_addr[i]);
	}
	dma_free_coherent(dev_struct, (size_t)dma_buffer_size, dma_buffer_base, dma_addr_base);

	printk(KERN_INFO"[sv_plat_remove]: ***********************PLATFORM DEVICE REMOVED**************************************\n");

	return 0;
}

static int __init sv_driver_init(void)
{

	dma_buffer_size = (u64)dma_system_size;
	//void * dummy;

	memset(mod_desc_arr, 0, sizeof(mod_desc_arr));

	printk(KERN_INFO"[sv_driver_init]: Initializing driver type %d\n", driver_type);
	switch(driver_type){
		case PCI:
		case AWS:
			printk(KERN_INFO"[pci_init]: Device ID: ('%d')\n", device_id);
			printk(KERN_INFO"[pci_init]: Major Number: ('%d')\n", major);

			init_waitqueue_head(&wq);
			init_waitqueue_head(&wq_periph);
			init_waitqueue_head(&thread_q_head_write);
			init_waitqueue_head(&thread_q_head_read);
			init_waitqueue_head(&cdma_q_head);
			init_waitqueue_head(&pci_write_head);

			ids[0].vendor = (u32)vendor_id;//PCI_VENDOR_ID_XILINX;
			ids[0].device = (u32)device_id;

			//strcpy(pci_devName_const, pci_devName);
			printk(KERN_INFO"[pci_init]: using driver name: %s\n", pci_devName);
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

	printk(KERN_INFO"[platform_init]: \t!!!!ERROR!!!!! No correct driver type detected!\n");
	return 0;
}

static void __exit sv_driver_exit(void)
{
	switch(driver_type){
		case PCI:
		case AWS:
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
	u32 vec_serviced;

    int test_attempts = 10;

	verbose_isr_printk(KERN_INFO"[pci_isr]: Entered the ISR (%llx)\n", axi_interr_ctrl);
	//	tasklet_schedule(&isr_tasklet);
	if(axi_interr_ctrl == 0) {
		verbose_isr_printk(KERN_INFO"[pci_isr]: returning early ISR (%llx)\n", axi_interr_ctrl);
		return IRQ_NONE; // controller not intialized yet
	}
	vec_serviced = 0;
	/*Here we need to find out who triggered the interrupt*
	 *Since we only allow one MSI vector, we need to query the
	 *Interrupt controller to find out. */

	/*This is the interrupt status register*/
	axi_dest = axi_interr_ctrl + INT_CTRL_ISR;

    /*This is the interrupt status register*/
    axi_dest = axi_interr_ctrl + INT_CTRL_ISR;
    if( direct_read(axi_dest, (void *)&status, 4, NORMAL_READ) ) {
        printk(KERN_INFO"[pci_isr]: !!!!!!!!ERROR direct_read\n");
        return IRQ_NONE;
    }

	verbose_isr_printk(KERN_INFO"[pci_isr]: interrupt status register vector is: ('0x%08x')\n", status);
	/*Here we need to clear the service interrupt in the interrupt acknowledge register*/
	axi_dest = axi_interr_ctrl + INT_CTRL_IAR;
	isr_status = status;
	if( direct_write(axi_dest, (void *)&status, 4, NORMAL_WRITE) ) {
		printk(KERN_INFO"[pci_isr]: !!!!!!!!ERROR direct_write\n");
		return IRQ_NONE;
	}

	while(isr_status > 0 && test_attempts-- > 0) {

		interrupt_num = vec2num(isr_status);
        verbose_isr_printk(KERN_INFO"[pci_isr]: interrupt status interrupt number is: ('%i')\n", interrupt_num);

		//	tasklet_schedule(&isr_tasklet);

		//check to mod_desc is not null for this interrupt_num and that it doesn't step over our array
		if(interrupt_num >= MAX_FILE_DESC || !mod_desc_arr[interrupt_num]) {
            //do nothing
			verbose_isr_printk(KERN_INFO"[pci_isr]: WARNING: Interrupt not set for this driver or interrupt over max. \n");
        }
        else {
    		verbose_isr_printk(KERN_INFO"[pci_isr]: interrupt number is: ('%d'-->' minor: %x')\n", interrupt_num, mod_desc_arr[interrupt_num]->minor);
    		device_mode = mod_desc_arr[interrupt_num]->mode;


    		if(device_mode == CDMA) {
    			verbose_isr_printk(KERN_INFO"[pci_isr]: this interrupt is from the CDMA\n");
    			vec_serviced = vec_serviced | num2vec(interrupt_num);
    		}
    		else if(device_mode == AXI_STREAM_FIFO || device_mode == AXI_STREAM_PACKET) {
    			verbose_isr_printk(KERN_INFO"[pci_isr]: this interrupt is from a user peripheral\n");

    			/*Read the axi fifo ISR*/
    			axi_dest = mod_desc_arr[interrupt_num]->axi_addr_ctl + AXI_STREAM_ISR;
    			if(direct_read(axi_dest, (void *)&status, 4, NORMAL_READ) ) {
    				printk(KERN_INFO"[pci_isr]: !!!!!!!!ERROR direct_read\n");
    				return ERROR;
    			}
    			verbose_isr_printk(KERN_INFO"[pci_isr]: Stream FIFO ISR status: 0x%08x\n", status);

    			/*clear the axi fifo ISR*/
    			status = status & 0x04000000;																																					//clear the status observed masked wtih RX complete
    			if(direct_write(axi_dest, (void *)&status, 4, NORMAL_WRITE) ) {
    				printk(KERN_INFO"[pci_isr]: !!!!!!!!ERROR direct_write\n");
    				return ERROR;
    			}

    			if(atomic_read(mod_desc_arr[interrupt_num]->in_read_fifo_count) == 0 && mod_desc_arr[interrupt_num]->file_open) {

    				//debug message
    				if(kfifo_len(&read_fifo) > 4) {
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
    			else if(!mod_desc_arr[interrupt_num]->file_open) {
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
    			atomic_set(mod_desc_arr[interrupt_num]->atomic_poll, 1); //non threaded way
    			//for non ring buffer peripherals (memory)
    			//wake_up(&wq_periph);
    			wake_up(&mod_desc_arr[interrupt_num]->poll_wq);
    			verbose_isr_printk(KERN_INFO"[pci_isr]: Waking up the Poll(normal)\n");
    		}

    		vec_serviced = vec_serviced | num2vec(interrupt_num);
        }

		//get next interrupt number
		isr_status = isr_status & ~num2vec(interrupt_num);
		verbose_isr_printk(KERN_INFO"[pci_isr]: vectors serviced is: ('0x%08x')\n", vec_serviced);
		verbose_isr_printk(KERN_INFO"[pci_isr]: interrupt status register vector is: ('0x%08x')\n", isr_status);
	}


	verbose_isr_printk(KERN_INFO"[pci_isr]: All interrupts serviced. The following Vector is acknowledged: 0x%x\n", vec_serviced);

	if(vec_serviced > 0) {
		/* The CDMA vectors (1 and 2) */

		/*The CDMA vectors are currently not being used unless we go back to the CDMAs sending interrupts.
		 * Right now we are polling the IDLE bit and it works much faster than waiting for the interrupt
		 */

		if((vec_serviced & 0x01) == 0x01) {
			verbose_isr_printk(KERN_INFO"[pci_isr]: Waking up CDMA 1\n");
			wake_up_interruptible(&wq);
		}

		if((vec_serviced & 0x02) == 0x02) {
			verbose_isr_printk(KERN_INFO"[pci_isr]: Waking up CDMA 2\n");
			wake_up_interruptible(&wq);
		}

		if(vec_serviced >= 0x10) {
			//for non ring buffer peripherals (memory)
		//	wake_up(&wq_periph);
		//	verbose_printk(KERN_INFO"[soft_isr]: Waking up the Poll()\n");

			//for ring buffer peripherals (axi streaming)
		//	atomic_set(&thread_q_read, 1); // the threaded way
		//	wake_up_interruptible(&thread_q_head_read);
		//	verbose_printk(KERN_INFO"[soft_isr]: Waking up the read thread\n");

		}
	}

	verbose_isr_printk(KERN_INFO"[pci_isr]: Exiting ISR\n");

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
	rfu = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);
	rfh = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);
	read_ring_buf_full = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);
	pci_write_q = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);
	in_read_fifo_count = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);
	in_write_fifo_count = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);
	mmap_count = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);


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
	atomic_set(mmap_count, 0);

	//interrupt_count = kmalloc(sizeof(int), GFP_KERNEL);

	s = (struct mod_desc *)kmalloc(sizeof(struct mod_desc), GFP_KERNEL);
	s->minor = MINOR(inode->i_rdev);
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
	s->dma_offset_read = 0;
	s->dma_offset_write = 0;
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
	s->mmap_count = mmap_count;

	s->tx_dest = 0x2;
	s->rx_dest = 0x0;

	s->file_open = true;
	#if (XDMA_AWS == 0)

	s->xdma_dev = lro_global;

	#endif

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
	struct mod_desc* mod_desc;

	//printk(KERN_INFO"[pci_release]: Attempting to close file minor number: %d\n", mod_desc->minor);

	mod_desc = filep->private_data;

	/*Query private data to see if it allocated DMA as a Master*/
	if(mod_desc->mode == MASTER)	{
		//unallocate DMA
		pci_free_consistent(pci_dev_struct, 131702, dma_master_buf[mod_desc->master_num], dma_m_addr[mod_desc->master_num]);
	} else if(mod_desc->mode == AXI_STREAM_FIFO || mod_desc->mode == AXI_STREAM_PACKET)	{
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
	while(attempts-- > 0 && in_read_fifo_count != 0){
		verbose_printk(KERN_INFO"[pci_%x_release]: in_read_fifo_count(%d) is not zero! waking thread\n", mod_desc->minor, in_read_fifo_count);
		atomic_set(&thread_q_read, 1);
		wake_up_interruptible(&thread_q_head_read);
		schedule();					//think we want to schedule here to give thread time to do work
		in_read_fifo_count = atomic_read(mod_desc->in_read_fifo_count);
	}
    //remove items from the in_read_fifo
    while(atomic_read(mod_desc->in_read_fifo_count) > 0) {
        atomic_dec(mod_desc->in_read_fifo_count);
    }


	in_write_fifo_count = atomic_read(mod_desc->in_write_fifo_count);
	attempts = 1000;
	//wake up the write threads until we clear our mod_desc out of them so we can free the memory assiociated with them.
	while(attempts-- > 0 && in_write_fifo_count != 0){
		verbose_printk(KERN_INFO"[pci_%x_release]: in_write_fifo_count(%d) is not zero! waking thread\n", mod_desc->minor, in_write_fifo_count);
		atomic_set(&thread_q_write, 1);
		wake_up_interruptible(&thread_q_head_write);
		schedule();					//think we want to schedule here to give thread time to do work
		in_write_fifo_count = atomic_read(mod_desc->in_write_fifo_count);
	}
    //remove items from the in_read_fifo
    while(atomic_read(mod_desc->in_write_fifo_count) > 0) {
        atomic_dec(mod_desc->in_write_fifo_count);
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
	kfree((const void*)mod_desc->mmap_count);

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
	struct statistics * stats; //= kmalloc(sizeof(struct statistics), GFP_KERNEL);
	struct timespec diff;
	int minor;
    int i;

	mod_desc = filep->private_data;
	minor = mod_desc->minor;

	if(arg == 0){
		arg_loc = 0;
	}
	else if( copy_from_user(&arg_loc, argp, sizeof(u64)) ) {
		printk(KERN_INFO"[pci_%x_ioctl]: !!!!!!!!ERROR copy_to_user\n", minor);
		return ERROR;
	}


	verbose_printk(KERN_INFO"[pci_%x_ioctl]: Entering IOCTL with command: %d and arg %llx\n", minor, cmd, arg_loc);


	switch(cmd){

		case WRITE_REG:
			verbose_printk(KERN_INFO"[pci_%x_ioctl]: Writing data at offset:%08llx\n", minor, arg_loc);
			if( direct_write(mod_desc->axi_addr + arg_loc, mod_desc->dma_write_addr+arg_loc, 4, NORMAL_WRITE) ) {
				printk(KERN_INFO"[pci_isr]: !!!!!!!!ERROR direct_read\n");
				return ERROR;
			}

		break;

		case READ_REG:
			verbose_printk(KERN_INFO"[pci_%x_ioctl]: Reading data at offset:%08llx\n", minor, arg_loc);
			if( direct_read(mod_desc->axi_addr + arg_loc, mod_desc->dma_write_addr+arg_loc, 4, NORMAL_READ) ) {
				printk(KERN_INFO"[pci_isr]: !!!!!!!!ERROR direct_read\n");
				return ERROR;
			}

		break;

		case SET_AXI_DEVICE:
			verbose_printk(KERN_INFO"[pci_%x_ioctl]: Setting Peripheral AXI Address: 0x%llx\n", minor, arg_loc);
			mod_desc->axi_addr = arg_loc&(0xFFFFFFFFFFFFFFFF);
			break;

		case SET_AXI_CTL_DEVICE:
			verbose_printk(KERN_INFO"[pci_%x_ioctl]: Setting Peripheral control AXI Address: 0x%llx\n", minor, arg_loc);
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
			 *one interrupt output to the the PCIe controller. This is because we want to use one msi vector. */
		case SET_AXI_INT_CTRL:
			//			int_ctlr_init(arg_loc);
			break;

		case SET_DMA_SIZE:
            if( dma_file_init(mod_desc, dma_buffer_base, dma_buffer_size, arg_loc) ) {
                printk(KERN_INFO"[pci_%x_ioctl]: \t!!!!set dma FAILURE!!!!.\n", minor);
                return ERROR;
            }

			break;

		case RESET_DMA_ALLOC:
			dma_current_offset = 4096;	//we want to leave the first 4k for the kernel to use internally.
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
			verbose_printk(KERN_INFO"[pci_%x_ioctl]: Interrupt Number:%d\n", minor, interrupt_num);

			mod_desc_arr[interrupt_num] = mod_desc;

			break;

		case SET_FILE_SIZE:
			mod_desc->file_size = ((loff_t)arg_loc & 0xffffffffffffffff);
			verbose_printk(KERN_INFO"[pci_%x_ioctl]: Setting device file size:%llu\n", minor, mod_desc->file_size);

			/*initialize the DMA for the file*/
            if(mod_desc->set_dma_flag == 0) {
        		if( dma_file_init(mod_desc, dma_buffer_base, dma_buffer_size, mod_desc->dma_size) ) {
        			printk(KERN_INFO"[pci_%x_ioctl]: \t!!!! DMA init FAILURE!!!!\n", minor);
        			return ERROR;
    			 }
        	}

			verbose_printk(KERN_INFO"[pci_%x_ioctl]: Returning dma size:%zu\n", minor, mod_desc->dma_size);

			if( copy_to_user((void *)arg, &(mod_desc->dma_size), sizeof(size_t)) ) {
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
			stats->tx_bytes = mod_desc->tx_bytes;
			stats->rx_bytes = mod_desc->rx_bytes;
			stats->cdma_attempt = mod_desc->cdma_attempt;
			stats->ip_not_ready = mod_desc->ip_not_ready;
			stats->cdma_usage_cnt = cdma_usage_cnt;
			cdma_usage_cnt = 0;

			if(mod_desc->stop_flag == 1) {
				mod_desc->stop_flag = 0;
				diff = timespec_sub(*(mod_desc->stop_time), *(mod_desc->start_time));
				stats->seconds = (unsigned long)diff.tv_sec;
				stats->ns = (unsigned long)diff.tv_nsec;
			}

			/*reset the counters*/
			mod_desc->tx_bytes = 0;
			mod_desc->rx_bytes = 0;
			mod_desc->ip_not_ready = 0;

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
			stats->tx_bytes = atomic_read(&driver_tx_bytes);
			stats->rx_bytes = atomic_read(&driver_rx_bytes);

			if(atomic_read(&driver_stop_flag) == 1) {
				atomic_set(&driver_stop_flag, 0);
				diff = timespec_sub((driver_stop_time), (driver_start_time));
				stats->seconds = (unsigned long)diff.tv_sec;
				stats->ns = (unsigned long)diff.tv_nsec;
			}

			/*reset the counters*/
			atomic_set(&driver_tx_bytes, 0);
			atomic_set(&driver_rx_bytes, 0);
			//driver_ip_not_ready = 0;
			if( copy_to_user((void *)arg, stats, sizeof(struct statistics)) ) {
				printk(KERN_INFO"[pci_%x_ioctl]: !!!!!!!!ERROR copy_to_user\n", minor);
				return ERROR;
			}
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
						printk(KERN_INFO"[pci_%x_ioctl]: DMA AXI Master allocation ERROR: \"%s\" \n", minor, pci_devName);
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
					if((mod_desc->axi_addr == -1) | (mod_desc->axi_addr_ctl == -1)) {
						printk(KERN_INFO"[pci_%x_ioctl]: !!!!!!!!ERROR: axi addresses of AXI STREAM FIFO not set\n", minor);
						printk(KERN_INFO"[pci_%x_ioctl]: \tset the AXI addresses then set mode again\n", minor);
						return ERROR;
					} else {
						verbose_printk(KERN_INFO"[pci_%x_ioctl]: Initializing the FIFO and setting registers\n", minor);
						/*set the ring buff full*/
						atomic_set(mod_desc->write_ring_buf_full, 0);
						atomic_set(mod_desc->read_ring_buf_full, 0);
						/*set the pointer defaults*/
						verbose_printk(KERN_INFO"[pci_%x_ioctl]: read ring_buffer: RFU : %d RFH %d\n", minor, 0, 0);
						atomic_set(mod_desc->wtk, 0);
						atomic_set(mod_desc->wtk, 0);
						verbose_printk(KERN_INFO"[pci_%x_ioctl]: write ring_buffer: WTH: %d WTK: %d\n", minor, 0, 0);
						atomic_set(mod_desc->rfh, 0);
						atomic_set(mod_desc->rfu, 0);

						if( axi_stream_fifo_init(mod_desc) ) {
							return ERROR;
						}

                        //testing setting first to be no incr address mode
                        // if( sv_do_addrmode_set(mod_desc->xdma_dev->sgdma_char_dev[0][0]->engine, 1) ) {
                        //     return ERROR;
                        // }
                        // if( sv_do_addrmode_set(mod_desc->xdma_dev->sgdma_char_dev[0][1]->engine, 1) ) {
                        //     return ERROR;
                        // }
					}
					break;

				default:printk(KERN_INFO"[pci_%x_ioctl]: !!!!!!!!ERROR, improper mode type!\n", minor);
			}
			break;

		default:printk(KERN_INFO"[pci_%x_ioctl]: !!!!!!!!ERROR, no command found.\n", minor);
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
	if(newpos < 0) return -EINVAL;
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

	very_verbose_poll_printk(KERN_INFO"[pci_%x_poll]: Poll() has been entered!\n", mod_desc->minor);
	/*Register the poll table with the peripheral wait queue
	 *so that every wake_up on the wait queue will see if the
	 *peripheral has data*/

	//poll_wait(filep, &wq_periph, pwait);
	//if data in the ring buffer
	//if(data_in_buffer == 0) {		//if there is no data to read


	poll_wait(filep, &mod_desc->poll_wq, pwait);
	very_verbose_poll_printk(KERN_INFO"[pci_%x_poll]: poll_wait done!\n", mod_desc->minor);

	if(atomic_read(mod_desc->atomic_poll) || (d2r != 0) ) {
		verbose_poll_printk(KERN_INFO"[pci_%x_poll]: Interrupting Peripheral Matched or Data in ring buffer(d2r:%x)!\n", mod_desc->minor, d2r);
		/*reset the has_data flag*/
		atomic_set(mod_desc->atomic_poll, 0);
		mask |= POLLIN;
	}

	very_verbose_poll_printk(KERN_INFO"[pci_%x_poll]: Leaving Poll(0x%08x)\n", mod_desc->minor, mask);
	return mask;

}

ssize_t pci_write(struct file *filep, const char __user *buf, size_t count, loff_t *f_pos)
{
	u64 axi_dest;
	u64 internal_offset = 0;
	struct mod_desc * mod_desc;
	size_t bytes_written = 0;
	size_t partial_count;
	int transfer_type;
	size_t room_till_end;
	size_t remaining;
	size_t mmap_addr_offset;						//offset of the buffer from the begining of the mmap
	int minor;
	void* buffer;
	int mmap_count;


	int wtk, wth, full;

	//this gets the minor number of the calling file so we can map the correct AXI address to write to
	mod_desc = filep->private_data;
	minor = mod_desc->minor;

	bytes_written = 0;

	verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: ************************************************************************\n", minor);
	verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: ******************** WRITE TRANSACTION BEGIN **************************\n", minor);
	switch(mod_desc->mode){
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

	if(mod_desc->set_dma_flag == 0) {
		if( dma_file_init(mod_desc, dma_buffer_base, dma_buffer_size, mod_desc->dma_size) ) {
			printk(KERN_INFO"[pci_%x_write]: \t!!!! DMA init FAILURE!!!!\n", minor);
			return ERROR;
			 }
		printk(KERN_INFO"[pci_%x_write]: Warning - Set the DMA file size to default value %d. IOCTL SET_FILE_SIZE was never called.\n", minor, (int)mod_desc->file_size);
		mod_desc->set_dma_flag = 1;
	}

	/*Start timers for statistics*/
	if(mod_desc->start_flag == 1) {
		getnstimeofday(mod_desc->start_time);
		mod_desc->start_flag = 0;
	}

	if(atomic_read(&driver_start_flag) == 1) {
		getnstimeofday(&driver_start_time);
		atomic_set(&driver_start_flag, 0);
	}

	switch(mod_desc->mode) {
		case AXI_STREAM_FIFO :
		case AXI_STREAM_PACKET :
			/*Stay until requested transmission is complete*/
			while (bytes_written < count && mod_desc->file_open) {

				if(count > mod_desc->dma_size) {
					printk(KERN_INFO"[pci_%x_write]: count > dma_size \n", minor);
					return ERROR;
				}

				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: the amount of bytes being copied to kernel: %zu\n", minor, count);

				wtk = atomic_read(mod_desc->wtk);
				wth = atomic_read(mod_desc->wth);
				full = atomic_read(mod_desc->write_ring_buf_full);

				//we are going to write the whole count + header
				while( (count + dma_byte_width + 2*sizeof(count)) > room_in_buffer(wtk, wth, full, mod_desc->dma_size) && mod_desc->file_open ) {
					/*sleep until the write thread signals priority to pci_write
					 * (ie there is no more data for the write thread to write)
					 * (ie there MUST be room in the ring buffer now)*/

					//wake up write thread
					atomic_set(&thread_q_write, 1);
					wake_up_interruptible(&thread_q_head_write);
					verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: not enough room in the the write buffer, waking up write thread a\n", minor);

					verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: pci_write is going to sleep ZzZzZzZzZzZzZz, room in buffer: %d\n", minor, room_in_buffer(wtk, wth, full, mod_desc->dma_size));
					if( wait_event_interruptible(pci_write_head, atomic_read(mod_desc->pci_write_q) == 1) ) {
						printk(KERN_INFO"[pci_%x_write]: !!!!!!!!ERROR wait_event_interruptible\n", minor);
						return ERROR;
					}
					atomic_set(mod_desc->pci_write_q, 0);
					verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: woke up the sleeping pci_write function!!\n", minor);

					wtk = atomic_read(mod_desc->wtk);
					wth = atomic_read(mod_desc->wth);
					full = atomic_read(mod_desc->write_ring_buf_full);
				}

				//if we need to wrap around the ring buffer....
				//copy the count to the ring buffer to act as the "header"
				room_till_end = (int)mod_desc->dma_size - wtk;
				if(sizeof(count) > room_till_end) {		//If the header will not fit in 1 chuck of the ring buffer (in the room till end) then just get it at the start
					wtk = 0;
				}

				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: header memcpy(0x%p + 0x%x, 0x%p, 0x%zx)\n", minor, mod_desc->dma_write_addr, wtk, (void * )&count, sizeof(count));
				memcpy(mod_desc->dma_write_addr+wtk, (void * )&count, sizeof(count));
				wtk = get_new_ring_pointer((int)sizeof(count), wtk, (int)mod_desc->dma_size);

				room_till_end = (int)( (mod_desc->dma_size - wtk) & ~(dma_byte_width-1));				 //make it divisible by dma_byte_width
				if(count > room_till_end) {		//if header size is larger than room till the end, write in two steps
					remaining = count-room_till_end;
					//write the room_till_end
					verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: copy_from_user 1(0x%p + 0x%x, 0x%p, 0x%zx)\n", minor, mod_desc->dma_write_addr, wtk, buf, room_till_end);
					if( copy_from_user(mod_desc->dma_write_addr+wtk, buf, room_till_end) ) {
						printk(KERN_INFO"[write_%x_data]: !!!!!!!!ERROR copy_from_user\n", minor);
						return ERROR;
					}
					wtk = 0;																																										//end of buffer reached

					verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: copy_from_user 2(0x%p + 0x%x, 0x%p, 0x%zx)\n", minor, mod_desc->dma_write_addr, wtk, buf+room_till_end, remaining);
					if( copy_from_user(mod_desc->dma_write_addr+wtk, buf+room_till_end, remaining) ) {
						printk(KERN_INFO"[write_%x_data]: !!!!!!!!ERROR copy_from_user\n", minor);
						return ERROR;
					}
					wtk = get_new_ring_pointer((int)remaining, wtk, (int)mod_desc->dma_size);
				} else {
					verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: copy_from_user(0x%p + 0x%x, 0x%p, 0x%zx)\n", minor, mod_desc->dma_write_addr, wtk, buf, count);
					if( copy_from_user(mod_desc->dma_write_addr+wtk, buf, count) ) {
						printk(KERN_INFO"[write_%x_data]: !!!!!!!!ERROR copy_from_user\n", minor);
						return ERROR;
					}
					wtk = get_new_ring_pointer((int)count, wtk, (int)mod_desc->dma_size);
				}

				//getnstimeofday(&stop_time);
				//diff = timespec_sub((stop_time), (start_time));
				//if(diff.tv_nsec > 1000000)
				//	printk(KERN_INFO"[pci_%x_write]: Copy_from_user time: %lunS\n", minor, diff.tv_nsec);

				atomic_set(mod_desc->wtk, wtk);

				/*This says that if the wtk pointer has caught up to the WTH pointer, give priority to the WTH*/
				if(atomic_read(mod_desc->wth) == wtk) {
					atomic_set(mod_desc->write_ring_buf_full, 1);
					verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: ring_point_%d: ring buff priority: %d\n", minor, minor, atomic_read(mod_desc->write_ring_buf_full));
				}

				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: write ring_buffer: WTH: %d WTK: %d\n", minor, atomic_read(mod_desc->wth), wtk);

				/*write the mod_desc to the write FIFO*/
				if(atomic_read(mod_desc->in_write_fifo_count) == 0) {
					//debug message
					if(kfifo_len(&write_fifo) > 4) {
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

				bytes_written = bytes_written + partial_count;
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: Wrote %zu bytes in this pass.\n", minor, partial_count);

				internal_offset += partial_count;
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: Internal offset updated: %llu.\n", minor, internal_offset);
			}
			break;

		case SLAVE:
		case MASTER:						//should master be supported?

			mmap_count = atomic_read(mod_desc->mmap_count);

			axi_dest = mod_desc->axi_addr + *f_pos;

			mmap_addr_offset = ((size_t)buf - mod_desc->mmap_start_addr);		//(difference between vitrual write address and given buffer)

			if(mod_desc->keyhole_config & 0x1)
				transfer_type = KEYHOLE_WRITE;
			else
				transfer_type = NORMAL_WRITE;

			//check to make sure count is not larger than file_size
			if(transfer_type == NORMAL_WRITE && (count + *f_pos) > mod_desc->file_size) {
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: File size is: 0x%llx is large than transfer size: 0x%zx bytes + offset: 0x%llxx\n", minor, mod_desc->file_size, count, *f_pos);
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: Setting Transfer size: 0x%llx bytes\n", minor, mod_desc->file_size);
				count = (size_t)(mod_desc->file_size - *f_pos);

			}
			//if we have mmap and the buf address is in the mmap range
			if(mmap_count > 0 && mmap_addr_offset >= 0 && mmap_addr_offset <= mod_desc->dma_size){			//todo make sure it is less than the size of memory map instead of dma size
				//don't do copy from user copy directly from buffer given because it is mmap'd

				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: writing peripheral with starting value(mmap): 0x%p\n", minor, (void *)(mod_desc->dma_write_addr));
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: dma buffer offset write value: %llx\n", minor, (u64)mod_desc->dma_offset_write);
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: writing peripheral using a transfer_type: 0x%x, offset: %llx\n", minor, transfer_type, *f_pos);

				verbose_pci_write_printk(KERN_INFO"[user_peripheral_%x_write]: data_transfer AXI Address(mmap): 0x%llx, buf: 0x%p, len: 0x%zx\n",
							minor, axi_dest, buf, count);

				//for mmap the data is really in the read buffer only, so use read_addr and offset_read for now
				if(data_transfer(mod_desc, axi_dest, mod_desc->dma_read_addr + mmap_addr_offset, count, transfer_type, (u64)mod_desc->dma_read_addr) ) {
					printk(KERN_INFO"[pci_%x_write]: !!!!!!!!ERROR writing to User Peripheral\n", minor);
					return ERROR;
				}

				if(transfer_type == NORMAL_WRITE) {
					//*f_pos = *f_pos + count;
					if(*f_pos + count == mod_desc->file_size) {
						//*f_pos = 0;
						verbose_pci_write_printk(KERN_INFO"[user_peripheral_%x_write]: Resetting file pointer back to zero...\n", minor);
					} else if(*f_pos + count > mod_desc->file_size) {
						printk(KERN_INFO"[user_peripheral_%x_write]: !!!!!!!!ERROR! Wrote past the file size. This should not have happened...\n", minor);
						printk(KERN_INFO"[user_peripheral_%x_write]: Resetting file pointer back to zero...\n", minor);
						//*f_pos = 0;
						return ERROR;
					}
					verbose_pci_write_printk(KERN_INFO"[user_peripheral_%x_write]: updated file offset is: %llx\n", minor, *f_pos);
				}
			}

			else{
				//not mmap
				if(count > mod_desc->dma_size) {
					verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: DMA Buffer size of: 0x%zx is not large enough for *remaining* transfer size: 0x%zx bytes\n", minor, mod_desc->dma_size, count);
					count = mod_desc->dma_size;
				}

				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: the amount of bytes being copied to kernel: %zu\n", minor, count);

				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: copy_from_user (0x%p, 0x%p + 0x%zx, 0x%zx)\n" , minor, mod_desc->dma_write_addr, &buf, bytes_written, count);

				if( copy_from_user(mod_desc->dma_write_addr, (buf + bytes_written), count) ) {
					printk(KERN_INFO"[pci_%x_write]: !!!!!!!!ERROR copy_from_user\n", minor);
					return ERROR;
				}
				buffer = mod_desc->dma_write_addr;

				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: writing peripheral with starting value: 0x%p\n", minor, (void *)(mod_desc->dma_write_addr));
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: dma buffer offset write value: %llx\n", minor, (u64)mod_desc->dma_offset_write);
				verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: writing peripheral using a transfer_type: 0x%x, offset: %llx\n", minor, transfer_type, *f_pos);

				verbose_pci_write_printk(KERN_INFO"[user_peripheral_%x_write]: data_transfer AXI Address: 0x%llx, buf: 0x%p, len: 0x%zx\n",
							minor, axi_dest, mod_desc->dma_write_addr, count);

				if(data_transfer(mod_desc, axi_dest, mod_desc->dma_write_addr, count, transfer_type, (u64)mod_desc->dma_offset_write) ) {
					printk(KERN_INFO"[pci_%x_write]: !!!!!!!!ERROR writing to User Peripheral\n", minor);
					return ERROR;
				}

				if(transfer_type == NORMAL_WRITE) {
					//*f_pos = *f_pos + count;

					if(*f_pos + count == mod_desc->file_size) {
						//*f_pos = 0;
						verbose_pci_write_printk(KERN_INFO"[user_peripheral_%x_write]: Resetting file pointer back to zero...\n", minor);
					} else if(*f_pos + count > mod_desc->file_size) {
						printk(KERN_INFO"[user_peripheral_%x_write]: !!!!!!!!ERROR! Wrote past the file size. This should not have happened...\n", minor);
						printk(KERN_INFO"[user_peripheral_%x_write]: Resetting file pointer back to zero...\n", minor);
						//*f_pos = 0;
						return ERROR;
					}
					verbose_pci_write_printk(KERN_INFO"[user_peripheral_%x_write]: updated file offset is: %llx\n", minor, *f_pos);
				}
			}
			bytes_written = count;

			break;
		default:
			verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: mode not detected on write\n", minor);
	}


	//file statistics
	mod_desc->tx_bytes = mod_desc->tx_bytes + bytes_written;
	atomic_add(bytes_written, &driver_tx_bytes);

	verbose_pci_write_printk(KERN_INFO"[pci_%x_write]: total file tx byes: %d \n", minor, mod_desc->tx_bytes);

	/*always update the stop_timer*/
	getnstimeofday(mod_desc->stop_time);
	getnstimeofday(&driver_stop_time);

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
	u64 internal_offset = 0;
	struct mod_desc *mod_desc;
	size_t bytes = 0;
	int transfer_type;
	int rfh, rfu, full;
	u64 d2r;
	size_t room_till_end;
	size_t remaining;
	int wake_up_flag = 0;
	int minor;
	void* buffer;
	size_t read_header_size;
	int mmap_count;
	size_t mmap_addr_offset;

	mod_desc = filep->private_data;
	minor = mod_desc->minor;
	buffer = mod_desc->dma_read_addr;

	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: ************************************************************************\n", minor);
	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: ******************** READ TRANSACTION BEGIN **************************\n", minor);
	switch(mod_desc->mode){
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

	if(mod_desc->set_dma_flag == 0) {
	 if( dma_file_init(mod_desc, dma_buffer_base, dma_buffer_size, mod_desc->dma_size) ) {
			printk(KERN_INFO"[pci_%x_read]: \t!!!! DMA init FAILURE!!!!.\n", minor);
			return ERROR;
		}
		printk(KERN_INFO"[pci_%x_read]: Warning - Set the DMA file size to default value %lld. IOCTL SET_FILE_SIZE was never called.\n", minor, mod_desc->file_size);
		mod_desc->set_dma_flag = 1;
	}

	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: Attempting to read %zu bytes\n", minor, count);

	if(mod_desc->start_flag == 1) {
		getnstimeofday(mod_desc->start_time);
		mod_desc->start_flag = 0;
	}

	if(atomic_read(&driver_start_flag) == 1) {
		getnstimeofday(&driver_start_time);
		atomic_set(&driver_start_flag, 0);
	}

	if(count > mod_desc->dma_size) {
		verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: Attempting to read more than the allocated DMA size of:%zu\n", minor, mod_desc->dma_size);
		verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: readjusting the read amount to:%zu\n", minor, mod_desc->dma_size);
		count = (size_t)mod_desc->dma_size;
	}


	switch(mod_desc->mode){

		case AXI_STREAM_FIFO:
		case AXI_STREAM_PACKET:
			// if AXI streaming fifo set with no interrupt then just read
			if(!mod_desc->has_interrupt_vec) {
				if (count == 0) {
					printk(KERN_INFO"[pci_%x_read] illegal to read 0 bytes\n",minor);
					break;
				}
				bytes = axi_stream_fifo_d2r(mod_desc);

				if(bytes == 0) {
					verbose_axi_fifo_write_printk(KERN_INFO"[pci_%x_read]: No data to read from axi stream fifo\n" , minor);
				} else {
					if(bytes <= count){
						bytes = axi_stream_fifo_read_direct((size_t)bytes, mod_desc->dma_read_addr, axi_pcie_m + mod_desc->dma_offset_read, mod_desc, mod_desc->dma_size);
						count = bytes;
					}
					else if(count < bytes) {
						bytes = axi_stream_fifo_read_direct((size_t)count, mod_desc->dma_read_addr, axi_pcie_m+mod_desc->dma_offset_read, mod_desc, mod_desc->dma_size);
					}

					verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: copy_to_user (0x%p, 0x%p, 0x%zx)\n" , minor, &buf, mod_desc->dma_read_addr, count);
					if( copy_to_user(buf, mod_desc->dma_read_addr, count) ) {
						printk(KERN_INFO"[pci_%x_read]: !!!!!!!!ERROR copy_to_user\n" , minor);
						return ERROR;
					}
					verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: Read %zd bytes from AXI FIFO\n" , minor , count);
				}
				break;
			} else {

				/* -----------------------------------------------------------------------------------
				 *	 Ring buffer Code Start */
				/*for the ring buffer case, we just need to determine a pointer location and a size
				 * to give to the copy_to_user function */
				rfh = atomic_read(mod_desc->rfh);
				rfu = atomic_read(mod_desc->rfu);
				full = atomic_read(mod_desc->read_ring_buf_full); //The thread has full when the atomic variable is 0

				d2r = data_in_buffer(rfh, rfu, full, mod_desc->dma_size);


				//if there is data in the ring buffer
				if(d2r > 0) {
					// if there is no left over raed_header size, read the packet header read_header_size from the ring buffer
					if(mod_desc->read_header_size == 0) {
							room_till_end = mod_desc->dma_size - rfu;
						//If the header will not fit in 1 chuck of the ring buffer (in the room till end) then just put it at the start
							if(sizeof(read_header_size) > room_till_end) {
							rfu = 0;
						}

						verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: header copy_to_user(0x%p, 0x%p + 0x%x, 0x%zx)\n", minor, &read_header_size, mod_desc->dma_read_addr, rfu, sizeof(read_header_size));
						memcpy(&read_header_size, mod_desc->dma_read_addr+rfu, sizeof(read_header_size));
						rfu = get_new_ring_pointer((int)sizeof(read_header_size), rfu, (int)(mod_desc->dma_size));
					} else{
						read_header_size = mod_desc->read_header_size;
					}

					verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: header: (0x%zx)\n" , minor, read_header_size);

					if(read_header_size > d2r) {
						printk(KERN_INFO"[pci_%x_read]: !!!!!!!!ERROR read_header_size: 0x%zx is larger than data to read: 0x%llx\n" , minor, read_header_size, d2r);
						count = 0;
						bytes = 0;
					} else if(read_header_size < count){
						count = read_header_size;
						verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: setting read count to 0x%zx\n" , minor, read_header_size);
					} else {
						verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: buffer size 0x%zx < read header size 0x%zx\n" , minor, count, read_header_size);
					}

					//if there is a packet to read
					if(count != 0) {

						room_till_end = ( (mod_desc->dma_size - rfu) & ~(dma_byte_width-1));				 //make it divisible by dma_byte_width

						if(count > room_till_end) { //need to do two read since we are at the edge
							remaining = count - room_till_end;
							verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: copy_to_user 1(0x%p, 0x%p + 0x%x, 0x%zx)\n", minor, &buf, mod_desc->dma_read_addr, rfu, room_till_end);
							if( copy_to_user(buf, mod_desc->dma_read_addr + rfu, room_till_end) ) {
								printk(KERN_INFO"[pci_%x_read]: !!!!!!!!ERROR copy_to_user\n" , minor);
								return ERROR;
							}

							//extra verbose debug message
							verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: first 4 bytes 0x%08x\n", minor, *((u32*)(mod_desc->dma_read_addr+rfu)));

							rfu = 0;
							//end of buffer reached
							verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: copy_to_user 2(0x%p, 0x%p + 0x%x, 0x%zx)\n", minor, &buf+room_till_end, mod_desc->dma_read_addr, rfu, remaining);
							if( copy_to_user(buf+room_till_end, mod_desc->dma_read_addr + rfu, remaining) ) {
								printk(KERN_INFO"[pci_%x_read]: !!!!!!!!ERROR copy_to_user\n" , minor);
								return ERROR;
							}
							rfu = get_new_ring_pointer((int)remaining, rfu, (int)mod_desc->dma_size);

						} else { 		//else we can do it in 1 read
							verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: copy_to_user(0x%p, 0x%p + 0x%x, 0x%zx)\n", minor, &buf, mod_desc->dma_read_addr, rfu, count);
							if( copy_to_user(buf, mod_desc->dma_read_addr + rfu, count) ) {
								printk(KERN_INFO"[pci_%x_read]: !!!!!!!!ERROR copy_to_user\n" , minor);
								return ERROR;
							}

							//extra verbose debug message
							verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: first 4 bytes 0x%08x\n", minor, *((u32*)(mod_desc->dma_read_addr+rfu)));
							rfu = get_new_ring_pointer((int)count, rfu, (int)mod_desc->dma_size);

						}

						//if ring buffer is more than half way full wake up the read thread after the rfu is updated.
						if(( d2r > (mod_desc->dma_size > 1) ) & back_pressure) {
							wake_up_flag = 1;
						}
						verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: bytes copied to user 0x%zx\n", minor, count);
						bytes = count;

						//if we didn't read the whole packet out then write a header back in for the next read
						//we need to write the header before the data read out, so rfu will need to be decreased
						//because we did not write the rfu yet, we do not have to worry about data being put there yet
						if(count < read_header_size) {
							read_header_size -= count;
							mod_desc->read_header_size = read_header_size;
							verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: set mod_desc->read_header_size to (0x%zx)\n", minor, read_header_size);
						} else {
							mod_desc->read_header_size = 0;
						}
					} else {
						verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: 0 size packet buffer\n" , minor);
						bytes = 0;
					}

					atomic_set(mod_desc->rfu, rfu);

					//if we were full, we just read data out of the ring buffer so we are no longer full
					if(full && count != 0) {
						atomic_set(mod_desc->read_ring_buf_full, 0);
						verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: ring_point : Read full: %d\n", minor, 0);
					}
					//------------------------------------------------------------------//
					verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: read ring_buffer: RFU : %d RFH %d\n", minor, rfu, atomic_read(mod_desc->rfh));

					if(wake_up_flag == 1) {
						verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: freed up the locked ring buffer, waking up read thread.\n", minor);

						// took data out, if full we can read now
						atomic_set(&thread_q_read, 1);
						wake_up_interruptible(&thread_q_head_read);
					}

				} else {
					verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: max can read is 0.\n", minor);
					bytes = 0;
				}
				/*	 Ring buffer Code End
				 * ------------------------------------------------------------------------------------*/
			}
			break;

		case SLAVE:
			// Here we will decide whether to do a zero copy DMA, or to read directly from the peripheral
			axi_dest = mod_desc->axi_addr + *f_pos + internal_offset;

			mmap_addr_offset = ((size_t)buf - mod_desc->mmap_start_addr);		//(difference between vitrual write address and given buffer)

			if(mod_desc->keyhole_config & 0x1)
				transfer_type = KEYHOLE_READ;
			else
				transfer_type = NORMAL_READ;

			if(count > mod_desc->dma_size) {
				verbose_printk(KERN_INFO"[pci_%x_read]: DMA Buffer size of: 0x%zx is not large enough for *remaining* transfer size: 0x%zx bytes\n", minor, mod_desc->dma_size, count);
				count = mod_desc->dma_size;
			}

			/*Check to see if read will go past the boundary*/
			if(count + *f_pos > (size_t)mod_desc->file_size) {
				verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: Read will overrun the file size because \n", minor);
				verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: (the current file offset + amount to read)->(%llu) > (%llu)->file_size\n", minor, count + *f_pos, mod_desc->file_size);
				count = (size_t)(mod_desc->file_size - *f_pos);
				verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: Adjusting to only reading %zu bytes to end of file!\n", minor, count);
			}

			mmap_count = atomic_read(mod_desc->mmap_count);

			if(mmap_count > 0 && mmap_addr_offset >= 0 && mmap_addr_offset <= mod_desc->dma_size){			//todo make sure it is less than the size of memory map instead of dma size
				//don't do copy to user copy directly to buffer given

				verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: reading peripheral using a transfer_type(mmap): 0x%x \n", minor, transfer_type);
				verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: current file offset is: %llu + %llu \n", minor, *f_pos, internal_offset);

				verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: data_transfer AXI Address(mmap): 0x%llx, buf: 0x%p, len: 0x%zx\n",	minor, axi_dest, buf, count);
				if(data_transfer(mod_desc, axi_dest, mod_desc->dma_read_addr + mmap_addr_offset, count, transfer_type, (u64)mod_desc->dma_offset_read) ) {
					printk(KERN_INFO"[user_peripheral_%x_read]: !!!!!!!!ERROR reading data from User Peripheral\n", minor);
					return ERROR;
				}

				verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: first 4 bytes 0x%08x\n", minor,*(unsigned int *)(mod_desc->dma_read_addr + mmap_addr_offset));
			}
			else{
				//else read data and copy it to the user

				verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: reading peripheral using a transfer_type: 0x%x \n", minor, transfer_type);
				verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: current file offset is: %llu + %llu \n", minor, *f_pos, internal_offset);

				verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: data_transfer AXI Address: 0x%llx, buf: 0x%p + 0x%llx, len: 0x%zx\n",
							minor, axi_dest, mod_desc->dma_read_addr, (u64)mod_desc->dma_offset_read, count);
				if(data_transfer(mod_desc, axi_dest, mod_desc->dma_read_addr, count, transfer_type, (u64)mod_desc->dma_offset_read) ) {
					printk(KERN_INFO"[user_peripheral_%x_read]: !!!!!!!!ERROR reading data from User Peripheral\n", minor);
					return ERROR;
				}

				verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: first 4 bytes 0x%08x\n", minor,*(unsigned int *)mod_desc->dma_read_addr);
				verbose_pci_read_printk(KERN_INFO"[user_peripheral_%x_read]: copy_to_user (0x%p, 0x%p, 0x%zx)\n" , minor, &buf, mod_desc->dma_read_addr, count);
				if( copy_to_user(buf, mod_desc->dma_read_addr, count) ) {
					printk(KERN_INFO"[pci_%x_read]: !!!!!!!!ERROR copy to user\n", minor);
					return ERROR;
				}
			}
			bytes = count;
			break;

		case MASTER:
			//Transfer buffer from kernel space to user space at the allocated DMA region
			verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: Transferred a Master write from kernel space to user space\n" , minor);
			if( copy_to_user(buf, dma_master_buf[mod_desc->master_num], count) ) {
				printk(KERN_INFO"[pci_%x_read]: !!!!!!!!ERROR copy_to_user\n" , minor);
				return ERROR;
			}
			break;

		default:
			verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: mode not detected on read\n", minor);
	}

	/*always update the stop_timer*/
	if(bytes > 0) {
		getnstimeofday(mod_desc->stop_time);
		getnstimeofday(&driver_stop_time);
		mod_desc->rx_bytes = mod_desc->rx_bytes + bytes;
		atomic_add(bytes, &driver_rx_bytes);
	}

	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: ************************************************************************\n", minor);
	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: ******************** READ TRANSACTION END **************************\n", minor);
	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: \t\tBytes read : %zd\n", minor, bytes);
	verbose_pci_read_printk(KERN_INFO"[pci_%x_read]: \t\tTotal Bytes read : %d\n", minor, mod_desc->rx_bytes);
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

	struct mod_desc *mod_desc;
	int minor;
	void* buffer;
	long length;
	int ret= 0;

	mod_desc = filep->private_data;
	minor = mod_desc->minor;
	buffer = mod_desc->dma_read_addr;
	length = vma->vm_end - vma->vm_start;


	verbose_mmap_printk(KERN_INFO"[pci_%x_mmap]: ************************************************************************\n", minor);
	verbose_mmap_printk(KERN_INFO"[pci_%x_mmap]: ****************************** MMAP BEGIN ******************************\n", minor);


	switch(mod_desc->mode){
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

		default :
			verbose_mmap_printk(KERN_INFO"[pci_%x_mmap]: \tAttempting to mmap %zu bytes : mode UNKNOWN \n", minor, length);

	}
	verbose_mmap_printk(KERN_INFO"[pci_%x_mmap]: ************************************************************************\n", minor);


	/* check length - do not allow larger mappings than the number of	pages allocated */
	if (length > mod_desc->dma_size) {
		printk(KERN_INFO "[pci_%x_mmap]: ERROR requested length is larger than dma_size\n", minor);
		ret = -EIO;
	}

	if (vma->vm_pgoff != 0) {
		printk(KERN_INFO "[pci_%x_mmap]: WARNING: vm_pgoff != 0, vm_pg_off ignored \n", minor);
	}

//fix me, using the "read" buffer for everything, change to 1 buffer
	if((vma->vm_flags & VM_READ) == VM_READ || (vma->vm_flags & VM_WRITE) == VM_WRITE){
		verbose_mmap_printk(KERN_INFO "[pci_%x_mmap]: Using dma_mmap_coherent for read/write space\n", minor);
		ret = dma_mmap_coherent(NULL, vma, mod_desc->dma_read_addr,	dma_addr_base+mod_desc->dma_offset_read, length);
		mod_desc->mmap_start_addr = vma->vm_start;

	}
	else {
		printk(KERN_INFO "[pci_%x_mmap]: ERROR unknown flags\n", minor);
		return ERROR;
	}
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
	struct mod_desc *mod_desc;
	int minor;
	int mmap_count;

	mod_desc = vma->vm_private_data;
	minor = mod_desc->minor;

	atomic_inc(mod_desc->mmap_count);
	mmap_count = atomic_read(mod_desc->mmap_count);

	verbose_mmap_printk(KERN_INFO"[mmap_open[%x]]: mmap_open: %x \n", minor, mmap_count);
}

/* decrement reference cout */
void mmap_close(struct vm_area_struct *vma)
{
	struct mod_desc *mod_desc;
	int minor;
	int mmap_count;

	mod_desc = vma->vm_private_data;
	minor = mod_desc->minor;

	atomic_dec(mod_desc->mmap_count);
	mmap_count = atomic_read(mod_desc->mmap_count);

	verbose_mmap_printk(KERN_INFO"[mmap[%x]_close]: mmap_close: %x \n", minor, mmap_count);

}
