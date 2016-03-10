/*************************************************************
 * System View Device Driver

 * Date: 11/10/2015

 * Author:   System View Inc.

 * Description:   This driver is to be used to control the
 *                Xilinx interface IP created by System View
 *                Inc. This driver can be set to operate as
 *                a PCIe Device Driver or as a Platform Device
 *                Driver depending on the input parameter.
 ************************************************************/

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
#include "sv_driver.h"

/*IOCTLS */
#define SET_AXI_DEVICE 50
#define SET_AXI_CDMA  51
#define SET_AXI_PCIE_CTL 52
#define SET_AXI_PCIE_M 53
#define SET_AXI_INT_CTRL 54
#define SET_AXI_DEV_SI 55
#define SET_AXI_DEV_M 56
#define CLEAR_AXI_INTERRUPT_CTLR 60
#define SET_CDMA_KEYHOLE_WRITE 58
#define SET_CDMA_KEYHOLE_READ 59
#define SET_MODE 62
#define SET_INTERRUPT 61
#define SET_AXI_CTL_DEVICE 63
#define SET_DMA_SIZE 64
#define RESET_DMA_ALLOC 65
#define SET_FILE_SIZE 66
#define GET_FILE_STATISTICS 67
#define GET_DRIVER_STATISTICS 70
#define START_FILE_TIMER 68
#define STOP_FILE_TIMER 69
#define START_DRIVER_TIMER 71
#define STOP_DRIVER_TIMER 72

#define ERROR   -1
#define SUCCESS 0

/* MODE Types */
#define SLAVE 0
#define AXI_STREAM_FIFO 1
#define MASTER 2
#define CDMA 3



#define MAX_NUM_MASTERS 2
#define MAX_NUM_SLI 4  // Max number of slaves with interrupts
#define MAX_NUM_INT MAX_NUM_MASTERS + MAX_NUM_SLI

/*These are the Driver types that are matched through insmod parameter "driver_type" */
#define PCI 1
#define PLATFORM 2

/***********Set default values for insmod parameters***************************/
int device_id = 100;
int major = 241;
int cdma_address = 0xFFFFFFFF;
int cdma_2_address = 0xFFFFFFFF;
int enable_cdma_2 = 0;
int pcie_ctl_address = 0xFFFFFFFF;
int pcie_m_address = 0xFFFFFFFF;
int int_ctlr_address = 0xFFFFFFFF;
int driver_type = PCI;
int dma_system_size = 1048576;
int dma_file_size = 4096;
int dma_byte_width = 8;   //64b data width

module_param(device_id, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(device_id, "DeviceID");

module_param(major, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(major, "MajorNumber");

module_param(enable_cdma_2, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(enable_cdma_2, "EnableCDMA2");

module_param(cdma_address, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(cdma_address, "CDMAAddress");

module_param(cdma_2_address, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(cdma_2_address, "CDMAAddress2");

module_param(pcie_ctl_address, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(pcie_ctl_address, "PCIeCTLAddress");

module_param(pcie_m_address, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(pcie_m_address, "PCIeMAddress");

module_param(int_ctlr_address, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(int_ctlr_address, "IntCtlrAddress");

module_param(driver_type, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(driver_type, "DriverType");

module_param(dma_system_size, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(dma_system_size, "DMASystemSize");

module_param(dma_file_size, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(dma_file_size, "DMAFileSize");

module_param(dma_byte_width, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(dma_byte_width, "DMAByteWidth");
/*****************************************************************************/

const char pci_devName[] = "vsi_driver"; //name of the device
unsigned long pci_bar_hw_addr;         //hardware base address of the device
unsigned long pci_bar_size;            //hardware bar memory size
unsigned long pci_bar_1_addr;         //hardware base address of the device
unsigned long pci_bar_1_size;            //hardware bar memory size
unsigned long pci_bar_2_addr;         //hardware base address of the device
unsigned long pci_bar_2_size;            //hardware bar memory size
struct pci_dev * pci_dev_struct = NULL; //pci device struct
struct platform_device * platform_dev_struct = NULL;
struct device *	dev_struct = NULL;
void * pci_bar_vir_addr = NULL;        //hardware base virtual address
void * pci_bar_1_vir_addr = NULL;        //hardware base virtual address
void * pci_bar_2_vir_addr = NULL;        //hardware base virtual address

/*this is the user peripheral address offset*/
u64 peripheral_space_offset = 0x80000000;
u64 bar_0_axi_offset = 0x40000000;
u64 peripheral_space_1_offset = 0xC00000000;

u64 axi_pcie_ctl;
u64 axi_interr_ctrl;
u64 axi_pcie_m;

u8 cdma_set[5];
u8 pcie_ctl_set;

int cdma_status;
int cdma_capable = 0;
unsigned int irq_num;
int cdma_usage_cnt = 0;

/*CDMA Semaphores*/
struct mutex CDMA_sem;
struct mutex CDMA_sem_2;

/*Other Semaphores*/

dma_addr_t dma_addr_base;
void * dma_buffer_base;
u32 dma_current_offset;
u64 dma_buffer_size = 1048576;
u32 current_dma_offset_internal;

dma_addr_t dma_m_addr[MAX_NUM_MASTERS];

void * dma_master_buf[MAX_NUM_MASTERS];

/* *************************************************  */

size_t dma_size;
wait_queue_head_t wq;
wait_queue_head_t wq_periph;
wait_queue_head_t mutexq;
int cdma_comp[5];
atomic_t cdma_atom[5];
int num_int;

atomic_t mutex_free = ATOMIC_INIT(0);

/*Driver Statistics*/
atomic_t driver_tx_bytes = ATOMIC_INIT(0);
atomic_t driver_rx_bytes = ATOMIC_INIT(0);
atomic_t driver_start_flag = ATOMIC_INIT(0);
atomic_t driver_stop_flag = ATOMIC_INIT(0);
struct timespec driver_start_time;
struct timespec driver_stop_time;

spinlock_t mLock;

const u32 INT_CTRL_IER      = 0x08;
const u32 INT_CTRL_MER      = 0x1c;
const u32 INT_CTRL_ISR      = 0x00;
const u32 INT_CTRL_IAR      = 0x0C;


/*This is an array of interrupt structures to hold up to 8 peripherals*/
struct interr_struct interr_dict[12] = {{ 0 }};

/*ISR Tasklet */
void do_isr_tasklet(unsigned long);
DECLARE_TASKLET(isr_tasklet, do_isr_tasklet, 0);

/* ************************* file operations *************************** */
long pci_unlocked_ioctl(struct file * filep, unsigned int cmd, unsigned long arg);
int pci_open(struct inode *inode, struct file *filep);
int pci_release(struct inode *inode, struct file *filep);
loff_t pci_llseek( struct file *filp, loff_t off, int whence);
ssize_t pci_write(struct file *filep, const char __user *buf, size_t count, loff_t *f_pos);
ssize_t pci_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos);
int pci_poll(struct file *filep, poll_table * pwait);

struct file_operations pci_fops = {
read:           pci_read,
		write:          pci_write,
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
static int probe(struct pci_dev * dev, const struct pci_device_id * id);
static int sv_plat_probe(struct platform_device *pdev);
static void remove(struct pci_dev * dev);
static int sv_plat_remove(struct platform_device * dev);
static unsigned char skel_get_revision(struct pci_dev * dev);

/************************** ISR functions **************************** */
static irqreturn_t pci_isr(int irq, void *dev_id);

static struct pci_device_id ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_XILINX, 0x7022), },
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

MODULE_DEVICE_TABLE(of, sv_driver_match);

static unsigned char skel_get_revision(struct pci_dev *dev)
{
	u8 revision;

	pci_read_config_byte(dev, PCI_REVISION_ID, &revision);
	return revision;
}

static int probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int ret;
	int pcie_m_set;
	int int_ctrl_set;
	/* Do probing type stuff here.
	 * 	 * Like calling request_region();
	 * 	 	 */

	verbose_printk(KERN_INFO"%s:<probe>******************************** BEGIN PROBE ROUTINE *****************************************\n", pci_devName);

	pci_dev_struct = dev;
	dev_struct = &dev->dev;

	if(NULL == pci_dev_struct){
		printk(KERN_INFO"%s:<probe>struct pci_dev_struct is NULL\n", pci_devName);
		return ERROR;
	}
	/****************** BAR 0 Mapping *******************************************/
	//get the base hardware address
	pci_bar_hw_addr = pci_resource_start(pci_dev_struct, 0);
	if (0 > pci_bar_hw_addr){
		printk(KERN_INFO"%s<probe>base hardware address is not set\n", pci_devName);
		return ERROR;
	}
	//get the base memory size
	pci_bar_size = pci_resource_len(pci_dev_struct, 0);
	printk(KERN_INFO"<probe>pci bar size is:%lu\n", pci_bar_size);

	//map the hardware space to virtual space
	pci_bar_vir_addr = ioremap(pci_bar_hw_addr, pci_bar_size);
	if(0 == pci_bar_vir_addr){
		printk(KERN_INFO"%s:<probe>ioremap error when mapping to vritaul address\n", pci_devName);
		return ERROR;
	}
	printk(KERN_INFO"<probe>pci bar virtual address base is:%p\n", pci_bar_vir_addr);

	/****************** BAR 1 Mapping *******************************************/
	//get the base hardware address
	pci_bar_1_addr = pci_resource_start(pci_dev_struct, 1);
	if (0 > pci_bar_1_addr){
		printk(KERN_INFO"%s<probe>BAR 1 base hardware address is not set\n", pci_devName);
		return ERROR;
	}
	//get the base memory size
	pci_bar_1_size = pci_resource_len(pci_dev_struct, 1);
	verbose_printk(KERN_INFO"<probe>pci bar 1 size is:%lu\n", pci_bar_1_size);

	//map the hardware space to virtual space
	if (pci_bar_1_size > 0)
	{
		pci_bar_1_vir_addr = ioremap(pci_bar_1_addr, pci_bar_1_size);
		if(0 == pci_bar_1_vir_addr){
			printk(KERN_INFO"%s:<probe>ioremap error when mapping to virtual address\n", pci_devName);
			return ERROR;
		}
		verbose_printk(KERN_INFO"<probe>pci bar 1 virtual address base is:%p\n", pci_bar_1_vir_addr);
	}
	else
	{
		peripheral_space_offset = 0;
		verbose_printk(KERN_INFO"%s<probe>BAR 1 is not in use\n", pci_devName);
	}
	/*****************************************************************************/


	/****************** BAR 2 Mapping *******************************************/
	//get the base hardware address
	pci_bar_2_addr = pci_resource_start(pci_dev_struct, 2);
	if (0 > pci_bar_2_addr){
		printk(KERN_INFO"%s<probe>BAR 2 base hardware address is not set\n", pci_devName);
		return ERROR;
	}
	//get the base memory size
	pci_bar_2_size = pci_resource_len(pci_dev_struct, 2);
	verbose_printk(KERN_INFO"<probe>pci bar 2 size is:%lu\n", pci_bar_2_size);

	if (pci_bar_2_size > 0)
	{
		//map the hardware space to virtual space
		pci_bar_2_vir_addr = ioremap(pci_bar_2_addr, pci_bar_2_size);
		if(0 == pci_bar_2_vir_addr){
			printk(KERN_INFO"%s:<probe>ioremap error when mapping to virtual address\n", pci_devName);
			return ERROR;
		}
		verbose_printk(KERN_INFO"<probe>pci bar 2 virtual address base is:%p\n", pci_bar_2_vir_addr);
	}
	else
	{
		peripheral_space_1_offset = 0;
		verbose_printk(KERN_INFO"%s<probe>BAR 2 is not in use\n", pci_devName);
	}
	/*****************************************************************************/

	//enable the device
	ret = pci_enable_device(dev);
	verbose_printk(KERN_INFO"<probe>device enabled\n");

	//set DMA mask
	if(0 != dma_set_coherent_mask(&dev->dev, 0x00000000FFFFFFFF)){
		printk(KERN_INFO"%s:<probe>set DMA mask error\n", pci_devName);
		return ERROR;
	}
	verbose_printk(KERN_INFO"<probe>dma mask set\n");

	//enable bus mastering
	pci_set_master(dev);
	verbose_printk(KERN_INFO"<probe>pci set as master\n");

	//enable MSI interrupts
	if(0 > pci_enable_msi(pci_dev_struct)){
		printk(KERN_INFO"%s:<probe>MSI enable error\n", pci_devName);
		return ERROR;
	}
	verbose_printk(KERN_INFO"<probe>pci enabled msi interrupt\n");

	//request IRQ
	if(0 > request_irq(pci_dev_struct->irq, &pci_isr, IRQF_SHARED, pci_devName, pci_dev_struct)){
		printk(KERN_INFO"%s:<probe>request IRQ error\n", pci_devName);
		return ERROR;
	}


	//register the char device
	if(0 > register_chrdev(major, pci_devName, &pci_fops)){
		printk(KERN_INFO"%s:<probe>char driver not registered\n", pci_devName);
		return ERROR;
	}

	if (skel_get_revision(dev) == 0x42)
		return -ENODEV;

	/*allocate the DMA buffer*/
	dma_buffer_base = dma_alloc_coherent(dev_struct, (size_t)dma_buffer_size, &dma_addr_base, GFP_KERNEL);
	if(NULL == dma_buffer_base)
	{
		printk("%s:<sv_driver_init>DMA buffer base allocation ERROR\n", pci_devName);
		printk("<sv_driver_init> typical max DMA size is 4M, check your linux settings\n");
		return -1;
	}
	else
	{
		verbose_printk(KERN_INFO"<sv_driver_init>: dma kernel buffer base address is:%llx\n", (u64)dma_buffer_base);
		verbose_printk(KERN_INFO"<sv_driver_init>: dma system memory buffer base address is:%llx\n", (u64)dma_addr_base);
		dma_current_offset = 4096;   //we want to leave the first 4k for the kernel to use internally.
		current_dma_offset_internal = 0;
	}

	//set defaults
	cdma_set[1] = 0;
	cdma_set[2] = 0;
	pcie_ctl_set = 0;
	pcie_m_set = 0;
	int_ctrl_set = 0;

	if (cdma_address != 0xFFFFFFFF)
	{
		ret = cdma_init(1, cdma_address, (u32)dma_addr_base);  //cdma_num = 1
	}

	//	if (cdma_address_2 != 0xFFFFFFFF)
	if (enable_cdma_2 != 0)
	{
		ret = cdma_init(2, cdma_2_address, (u32)dma_addr_base);  //cdma_num = 2
	}

	if (pcie_ctl_address != 0xFFFFFFFF)
	{
		ret = pcie_ctl_init((u64)pcie_ctl_address, (u32)dma_addr_base);
		if (ret < 0)
			return -1;
		else
			pcie_ctl_set = 1;
	}

	if (pcie_m_address != 0xFFFFFFFF)
	{
		pcie_m_init(1);
		pcie_m_set = 1;
	}

	//	if (pcie_m_address_2 != 0xFFFFFFFF)
	//	{
	//		pcie_m_init(2);
	//	}

	if (int_ctlr_address != 0xFFFFFFFF)
	{
		int_ctlr_init((u64)int_ctlr_address);
		int_ctrl_set = 1;
	}

	cdma_capable = (cdma_set[1] == 1) & (pcie_m_set == 1) & (int_ctrl_set == 1) & (pcie_ctl_set == 1);
	printk(KERN_INFO"<probe> cdma_capable = %x\n", cdma_capable);
	printk(KERN_INFO"<probe> cdma_set[2] = %x\n", cdma_set[2]);

	printk(KERN_INFO"<probe>***********************PROBE FINISHED SUCCESSFULLY**************************************\n");
	return 0;
}

static int sv_plat_probe(struct platform_device *pdev)
{
	struct resource * resource_1;
	int ret;
	int pcie_m_set;
	int int_ctrl_set;

	verbose_printk(KERN_INFO"%s:<probe>******************************** BEGIN PROBE ROUTINE *****************************************\n", pci_devName);

	platform_dev_struct = pdev;
	dev_struct = &pdev->dev;

	resource_1 = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	//get the base memory size
	pci_bar_size = resource_1->end - resource_1->start;
	verbose_printk(KERN_INFO"<probe>pci bar size is:%lu\n", pci_bar_size);

	//map the hardware space to virtual space
	pci_bar_vir_addr = devm_ioremap_resource(&pdev->dev, resource_1);

	if(0 == pci_bar_vir_addr){
		printk(KERN_INFO"%s:<probe>ioremap error when mapping to virtual address\n", pci_devName);
		return ERROR;
	}
	verbose_printk(KERN_INFO"<probe>pci bar virtual address base is:%p\n", pci_bar_vir_addr);

	/****************** BAR 1 Mapping *******************************************/

	//	resource_2 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	//get the base memory size
	//	pci_bar_1_size = resource_2->end - resource_2->start;
	//	verbose_printk(KERN_INFO"<probe>pci bar 1 size is:%d\n", pci_bar_1_size);

	//map the hardware space to virtual space
	//	pci_bar_1_vir_addr = devm_ioremap_resource(&pdev->dev, resource_2);
	//	if(0 == pci_bar_1_vir_addr){
	//		verbose_printk(KERN_INFO"%s:<probe>ioremap error when mapping to virtual address\n", pci_devName);
	//		return ERROR;
	//	}
	//	verbose_printk(KERN_INFO"<probe>pci bar 1 virtual address base is:%p\n", pci_bar_1_vir_addr);
	peripheral_space_offset = 0;
	pci_bar_1_size = 0;

	/*****************************************************************************/


	//set DMA mask
	if(0 != dma_set_mask(&pdev->dev, 0x00000000FFFFFFFF)){
		printk(KERN_INFO"%s:<probe>set DMA mask error\n", pci_devName);
		return ERROR;
	}
	verbose_printk(KERN_INFO"<probe>dma mask set\n");

	irq_num = platform_get_irq(pdev, 0);
	verbose_printk(KERN_INFO"<probe>IRQ number is:%x\n", irq_num);

	//request IRQ
	if(0 > request_irq(irq_num, &pci_isr, IRQF_SHARED, pci_devName, pdev)){
		printk(KERN_INFO"%s:<probe>request IRQ error\n", pci_devName);
		return ERROR;
	}

	//register the char device
	if(0 > register_chrdev(major, "sv_driver", &pci_fops)){
		printk(KERN_INFO"%s:<probe>char driver not registered\n", "sv_driver");
		return ERROR;
	}

	/*allocate the DMA buffer*/
	dma_buffer_base = dma_alloc_coherent(dev_struct, (size_t)dma_buffer_size, &dma_addr_base, GFP_KERNEL);
	if(NULL == dma_buffer_base)
	{
		printk("%s:<sv_driver_init>DMA buffer base allocation ERROR\n", pci_devName);
		return -1;
	}
	else
	{
		verbose_printk(KERN_INFO"<sv_driver_init>: dma buffer base address is:%llx\n", (u64)dma_buffer_base);
		verbose_printk(KERN_INFO"<sv_driver_init>: dma system memory buffer base address is:%llx\n", (u64)dma_addr_base);
		dma_current_offset = 4096;   //we want to leave the first 4k for the kernel to use internally.
		current_dma_offset_internal = 0;
	}

	//set defaults
	cdma_set[1] = 0;
	cdma_set[2] = 0;
	pcie_ctl_set = 0;
	pcie_m_set = 0;
	int_ctrl_set = 0;

	if (cdma_address != 0xFFFFFFFF)
	{
		ret = cdma_init(1, cdma_address, (u32)dma_addr_base);
	}

	if (enable_cdma_2 != 0)
	{
		ret = cdma_init(2, cdma_2_address, (u32)dma_addr_base);
	}

	if (pcie_m_address != 0xFFFFFFFF)
	{
		pcie_m_init(1);
		pcie_m_set = 1;
	}

	if (int_ctlr_address != 0xFFFFFFFF)
	{
		int_ctlr_init((u64)int_ctlr_address);
		int_ctrl_set = 1;
	}

	/*ARM works a little differently for DMA than PCIe in that that translation
	 * is not handled by the core by writing to a register. For Zynq, the axi to DDR address
	 * mapping is 1-1 and should be written directly to the returned DMA handle */

	axi_pcie_m = axi_pcie_m + (u64)dma_addr_base;   //cdma 1

	cdma_capable = (cdma_set[1] == 1) & (pcie_m_set == 1) & (int_ctrl_set == 1);

	printk(KERN_INFO"<probe> cdma_capable = %x\n", cdma_capable);
	printk(KERN_INFO"<probe> cdma_set[2] = %x\n", cdma_set[2]);

	printk(KERN_INFO"<probe>***********************PROBE FINISHED SUCCESSFULLY**************************************\n");
	return 0;
}

static void remove(struct pci_dev *dev)
{
	/* clean up any allocated resources and stuff here.
	 * 	 * like call release_region();
	 * 	 	 */
	//      release_mem_region(pci_bar_hw_addr, REG_SIZE);

	iounmap(pci_bar_vir_addr);

	free_irq(pci_dev_struct->irq, pci_dev_struct);

	pci_disable_msi(pci_dev_struct);

	dma_free_coherent(dev_struct, (size_t)dma_buffer_size, dma_buffer_base, dma_addr_base);

	unregister_chrdev(major, pci_devName);

}

static int sv_plat_remove(struct platform_device *pdev)
{
	iounmap(pci_bar_vir_addr);

	free_irq(irq_num, platform_dev_struct);

	dma_free_coherent(dev_struct, (size_t)dma_buffer_size, dma_buffer_base, dma_addr_base);

	unregister_chrdev(major, pci_devName);

	verbose_printk(KERN_INFO"<remove>***********************PLATFORM DEVICE REMOVED**************************************\n");

	return 0;
}

static struct pci_driver pci_driver = {
	.name = "vsi_driver",
	//	.name = pci_devName,
	.id_table = ids,
	.probe = probe,
	.remove = remove,
};


static int __init sv_driver_init(void)
{

	dma_buffer_size = (u64)dma_system_size;

	switch(driver_type){
		case PCI:
			verbose_printk(KERN_INFO"<pci_init>: Device ID: ('%d')\n", device_id);
			verbose_printk(KERN_INFO"<pci_init>: Major Number: ('%d')\n", major);

			init_waitqueue_head(&wq);
			init_waitqueue_head(&wq_periph);
			init_waitqueue_head(&mutexq);

			ids[0].vendor =  PCI_VENDOR_ID_XILINX;
			ids[0].device =  (u32)device_id;

			return pci_register_driver(&pci_driver);
			break;

		case PLATFORM:

			verbose_printk(KERN_INFO"<platform_init>: Device ID: ('%d')\n", device_id);
			verbose_printk(KERN_INFO"<platform_init>: Major Number: ('%d')\n", major);

			init_waitqueue_head(&wq);
			init_waitqueue_head(&wq_periph);
			init_waitqueue_head(&mutexq);

			bar_0_axi_offset = 0x40000000;

			return platform_driver_register(&sv_plat_driver);

			break;

		default:;
	}

	verbose_printk(KERN_INFO"<platform_init>: !!!!ERROR!!!!! No correct driver type detected!\n");
	return 0;
}

static void __exit sv_driver_exit(void)
{
	switch(driver_type){
		case PCI:
			pci_unregister_driver(&pci_driver);
			break;

		case PLATFORM:
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
	u32 status;
	u64 axi_dest;
	int int_num;
	u32 device_mode;
	int ret;
	u32 vec_serviced;
	int i;
	// int interr[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	verbose_printk(KERN_INFO"<pci_isr>:						Entered the ISR");

	//	tasklet_schedule(&isr_tasklet);
	verbose_printk(KERN_INFO"		Entered the Tasklet");

	vec_serviced = 0;
	i = 0;
	/*Here we need to find out who triggered the interrupt*
	 *Since we only allow one MSI vector, we need to query the
	 *Interrupt controller to find out. */

	/*This is the interrupt status register*/
	axi_dest = axi_interr_ctrl + INT_CTRL_ISR;
	ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_READ, 0);
	verbose_printk(KERN_INFO"<soft_isr>: interrupt status register vector is: ('%x')\n", status);


	while(status > 0)
	{
		int_num = vec2num(status);
		verbose_printk(KERN_INFO"<soft_isr>: interrupt number is: ('%d')\n", int_num);
		device_mode = *(interr_dict[int_num].mode);

		if (device_mode == CDMA)
		{
			verbose_printk(KERN_INFO"<soft_isr>: this interrupt is from the CDMA\n");
			vec_serviced = vec_serviced | num2vec(int_num);
		}

		else
		{
			verbose_printk(KERN_INFO"<soft_isr>: this interrupt is from a user peripheral\n");

			//	if (*(interr_dict[int_num].int_count) < 10)   //set a max here....
			//	*(interr_dict[int_num].int_count) = (*(interr_dict[int_num].int_count)) + 1;
			*(interr_dict[int_num].int_count) = 1;
			atomic_set(interr_dict[int_num].atomic_poll, 1);

			verbose_printk(KERN_INFO"<soft_isr>: this is after the int count add\n");

			vec_serviced = vec_serviced | num2vec(int_num);
		}

		/*Here we need to clear the service interrupt in the interrupt acknowledge register*/
		axi_dest = axi_interr_ctrl + INT_CTRL_IAR;
		status = num2vec(int_num);
		ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_WRITE, 0);

		/*This is the interrupt status register*/
		axi_dest = axi_interr_ctrl + INT_CTRL_ISR;
		verbose_printk(KERN_INFO"<soft_isr>: Checking the ISR vector for any additional vectors....\n");
		ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_READ, 0);


	}


	verbose_printk(KERN_INFO"<soft_isr>: All interrupts serviced. The following Vector is acknowledged: %x\n", vec_serviced);

	if (vec_serviced > 0)
	{
		/* The CDMA vectors (1 and 2) */
		if ((vec_serviced & 0x01) == 0x01)
		{
			//	cdma_comp[1] = 1;      //condition for wake_up
			atomic_set(&cdma_atom[1], 1);
			verbose_printk(KERN_INFO"<soft_isr>: Waking up CDMA 1\n");
			wake_up_interruptible(&wq);
		}

		if ((vec_serviced & 0x02) == 0x02)
		{
			//	cdma_comp[2] = 1;      //condition for wake_up
			atomic_set(&cdma_atom[2], 1);
			verbose_printk(KERN_INFO"<soft_isr>: Waking up CDMA 2\n");
			wake_up_interruptible(&wq);
		}

		//	wake_up_interruptible(&wq);
		//		*(interr_dict[4].int_count) = (*(interr_dict[4].int_count)) + 1;
		//		*(interr_dict[5].int_count) = (*(interr_dict[5].int_count)) + 1;

		if (vec_serviced >= 0x10)
		{
			wake_up(&wq_periph);

			//	for(i = 4; i<=7; i++)
			//	{
			//	mutex_lock_interruptible(interr_dict[i].int_count_sem);
			//		if (interr_dict[i].int_count > 0)
			//	if (interr[i] > 0)
			//		{
			//	    	verbose_printk(KERN_INFO"<soft_isr>: Waking up User Peripheral:%x\n", i);
			//		mutex_unlock(interr_dict[i].int_count_sem);
			//	interr_dict[i].int_count = 0;
			//	interr[i] = 0;
			//		wake_up(interr_dict[i].iwq);
			//		}
			//	}
			//		verbose_printk(KERN_INFO"<soft_isr>:			Waking up User Peripherals\n");
		}
	}

	verbose_printk(KERN_INFO"<soft_isr>:						Exiting ISR\n");
	verbose_printk(KERN_INFO"		Exited the Tasklet");

	verbose_printk(KERN_INFO"<pci_isr>:						Exiting the ISR");

	return IRQ_HANDLED;

}

void do_isr_tasklet (unsigned long unused)
{
	u32 status;
	u64 axi_dest;
	int int_num;
	u32 device_mode;
	int ret;
	u32 vec_serviced;
	int i;
	// int interr[8] = {0, 0, 0, 0, 0, 0, 0, 0};

	verbose_printk(KERN_INFO"		Entered the Tasklet");

	vec_serviced = 0;
	i = 0;
	/*Here we need to find out who triggered the interrupt*
	 *Since we only allow one MSI vector, we need to query the
	 *Interrupt controller to find out. */

	/*This is the interrupt status register*/
	axi_dest = axi_interr_ctrl + INT_CTRL_ISR;
	ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_READ, 0);
	verbose_printk(KERN_INFO"<soft_isr>: interrupt status register vector is: ('%x')\n", status);


	while(status > 0)
	{
		int_num = vec2num(status);
		verbose_printk(KERN_INFO"<soft_isr>: interrupt number is: ('%d')\n", int_num);
		device_mode = *(interr_dict[int_num].mode);

		if (device_mode == CDMA)
		{
			verbose_printk(KERN_INFO"<soft_isr>: this interrupt is from the CDMA\n");
			vec_serviced = vec_serviced | num2vec(int_num);
		}

		else
		{
			verbose_printk(KERN_INFO"<soft_isr>: this interrupt is from a user peripheral\n");

			//	if (*(interr_dict[int_num].int_count) < 10)   //set a max here....
			//	*(interr_dict[int_num].int_count) = (*(interr_dict[int_num].int_count)) + 1;
			*(interr_dict[int_num].int_count) = 1;
			atomic_set(interr_dict[int_num].atomic_poll, 1);

			verbose_printk(KERN_INFO"<soft_isr>: this is after the int count add\n");

			vec_serviced = vec_serviced | num2vec(int_num);
		}

		/*Here we need to clear the service interrupt in the interrupt acknowledge register*/
		axi_dest = axi_interr_ctrl + INT_CTRL_IAR;
		status = num2vec(int_num);
		ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_WRITE, 0);

		/*This is the interrupt status register*/
		axi_dest = axi_interr_ctrl + INT_CTRL_ISR;
		verbose_printk(KERN_INFO"<soft_isr>: Checking the ISR vector for any additional vectors....\n");
		ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_READ, 0);


	}


	verbose_printk(KERN_INFO"<soft_isr>: All interrupts serviced. The following Vector is acknowledged: %x\n", vec_serviced);

	if (vec_serviced > 0)
	{
		/* The CDMA vectors (1 and 2) */
		if ((vec_serviced & 0x01) == 0x01)
		{
			cdma_comp[1] = 1;      //condition for wake_up
			//	atomic_set(&cdma_atom[1], 1);
			verbose_printk(KERN_INFO"<soft_isr>: Waking up CDMA 1\n");
			wake_up_interruptible(&wq);
		}

		if ((vec_serviced & 0x02) == 0x02)
		{
			cdma_comp[2] = 1;      //condition for wake_up
			//	atomic_set(&cdma_atom[2], 1);
			verbose_printk(KERN_INFO"<soft_isr>: Waking up CDMA 2\n");
			wake_up_interruptible(&wq);
		}

		//	wake_up_interruptible(&wq);
		//		*(interr_dict[4].int_count) = (*(interr_dict[4].int_count)) + 1;
		//		*(interr_dict[5].int_count) = (*(interr_dict[5].int_count)) + 1;

		if (vec_serviced >= 0x10)
		{
			wake_up(&wq_periph);

			//	for(i = 4; i<=7; i++)
			//	{
			//	mutex_lock_interruptible(interr_dict[i].int_count_sem);
			//		if (interr_dict[i].int_count > 0)
			//	if (interr[i] > 0)
			//		{
			//	    	verbose_printk(KERN_INFO"<soft_isr>: Waking up User Peripheral:%x\n", i);
			//		mutex_unlock(interr_dict[i].int_count_sem);
			//	interr_dict[i].int_count = 0;
			//	interr[i] = 0;
			//		wake_up(interr_dict[i].iwq);
			//		}
			//	}
			//		verbose_printk(KERN_INFO"<soft_isr>:			Waking up User Peripherals\n");
		}
	}

	verbose_printk(KERN_INFO"<soft_isr>:						Exiting ISR\n");
	verbose_printk(KERN_INFO"		Exited the Tasklet");

}

/* -----------------File Operations-------------------------*/
int pci_open(struct inode *inode, struct file *filep)
{
	void * mode_address;
	void * interrupt_count;
	//	void * iwq;
	//	void * int_count_sem;

	//	void * kern_reg_write;
	//	void * kern_reg_read;
	struct mod_desc * s;
	struct timespec * start_time;
	struct timespec * stop_time;

	atomic_t * atomic_poll;

	start_time = (struct timespec *)kmalloc(sizeof(struct timespec), GFP_KERNEL);
	stop_time = (struct timespec *)kmalloc(sizeof(struct timespec), GFP_KERNEL);

	atomic_poll = (atomic_t *)kmalloc(sizeof(atomic_t), GFP_KERNEL);

	atomic_set(atomic_poll, 0);
	//wait_queue_head_t * iwq;    //the interrupt wait queue for device

	//	iwq = kmalloc(sizeof(wait_queue_head_t), GFP_KERNEL);
	//	int_count_sem = kmalloc(sizeof(struct mutex), GFP_KERNEL);

	mode_address = kmalloc(sizeof(int), GFP_KERNEL);
	interrupt_count = kmalloc(sizeof(int), GFP_KERNEL);

	s = (struct mod_desc *)kmalloc(sizeof(struct mod_desc), GFP_KERNEL);
	s->minor = MINOR(inode->i_rdev);
	s->axi_addr = 0;
	s->axi_addr_ctl = 0;
	s->mode = (u32*)mode_address;   //defaults as slave only
	*(s->mode) = 0;
	s->int_count = (int*)interrupt_count;
	*(s->int_count) = 0;
	s->int_num = 100;
	s->master_num = 0;
	s->interrupt_vec = 0;
	s->keyhole_config = 0;
	s->dma_offset_read = 0;
	s->dma_offset_write = 0;
	s->dma_size = 4096;
	s->dma_offset_internal_read = 0;
	s->dma_offset_internal_write = 0;
	s->kernel_reg_write = 0;
	s->kernel_reg_read = 0;
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
	//	s->iwq = (wait_queue_head_t*)iwq;
	//	s->int_count_sem = (struct mutex*)int_count_sem;

	verbose_printk(KERN_INFO"<pci_open>: minor number %d detected\n", s->minor);

	/*set the private data field of the file pointer for file op use*/
	filep->private_data = s;

	/*initialize the DMA for the file*/
	//	ret = dma_file_init(s, dma_file_size, dma_buffer_base, dma_buffer_size);
	//	if (ret < 0)
	//	{
	//		printk(KERN_INFO"<pci_open>:!!!!file open FAILURE!!!!.\n");
	//		return ERROR;
	//	}

	verbose_printk(KERN_INFO"<pci_open>: file open complete.\n");
	return SUCCESS;

}

int pci_release(struct inode *inode, struct file *filep)
{
	/*release is only called on the final close() of a file
	 * This is so that the file can be open multiple times.
	 * Might need to find a way to kfree() the other open
	 * instances of the file. Or restrict that each file
	 * only be open once. */

	struct mod_desc* mod_desc;

	mod_desc = filep->private_data;

	/*Query private data to see if it allocated DMA as a Master*/
	if (*(mod_desc->mode) == MASTER)
	{
		//unallocate DMA
		pci_free_consistent(pci_dev_struct, 131702, dma_master_buf[mod_desc->master_num], dma_m_addr[mod_desc->master_num]);
	}

	kfree((const void*)mod_desc->mode);

	kfree((const void*)mod_desc->int_count);
	kfree((const void*)mod_desc->start_time);
	kfree((const void*)mod_desc->stop_time);

	kfree((const void*)filep->private_data);

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
	int int_num;
	int ret;
	struct statistics * statistics = kmalloc(sizeof(struct statistics), GFP_KERNEL);
	struct timespec diff;

	ret = copy_from_user(&arg_loc, argp, sizeof(u64));

	verbose_printk("<ioctl>Entering IOCTL with command: %d and arg %llx\n", cmd, arg_loc);

	mod_desc = filep->private_data;

	switch(cmd){

		case SET_AXI_DEVICE:
			verbose_printk(KERN_INFO"<ioctl>: Setting Peripheral Axi Address:%llx\n", arg_loc);
			mod_desc->axi_addr = arg_loc;
			break;

		case SET_AXI_CTL_DEVICE:
			verbose_printk(KERN_INFO"<ioctl>: Setting Peripheral CTL AXI Address:%llx\n", arg_loc);
			mod_desc->axi_addr_ctl = arg_loc;
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
			verbose_printk(KERN_INFO"<ioctl>: Setting Peripheral DMA size:%llx\n", arg_loc);
			mod_desc->dma_size = (size_t)arg_loc;
			if (((u64)dma_current_offset + arg_loc) > (u64)((char*)dma_buffer_base + dma_buffer_size))
			{
				printk(KERN_INFO"<ioctl>: ERROR! DMA Buffer out of memory!\n");
				return ERROR;
			}
			else
			{
				mod_desc->dma_size = (size_t)arg_loc;
				verbose_printk(KERN_INFO"<set_dma_size>: The current system memory dma offset:%x\n", dma_current_offset);
				mod_desc->dma_offset_read = dma_current_offset;            //set the dma start address for the peripheral read
				mod_desc->dma_offset_write = dma_current_offset + (u32)arg_loc; //set the dma start address for the peripheral write
				mod_desc->dma_write = (void*)((char*)dma_buffer_base + (u64)dma_current_offset + arg_loc);            //actual pointer to kernel buffer
				verbose_printk(KERN_INFO"<ioctl>: DMA kernel write address set to:%llx\n", (u64)mod_desc->dma_write);
				mod_desc->dma_read = (void*)((char*)dma_buffer_base + (u64)dma_current_offset);            //actual pointer to kernel buffer

				dma_current_offset = dma_current_offset + (u32)(2*arg_loc);            //update the current dma allocation pointer, 2 buffers (R/W)
				verbose_printk(KERN_INFO"<ioctl>: Success setting peripheral DMA\n");
			}
			break;

		case RESET_DMA_ALLOC:
			dma_current_offset = 4096;   //we want to leave the first 4k for the kernel to use internally.
			current_dma_offset_internal = 0;
			verbose_printk(KERN_INFO"<ioctl>: Reset the DMA Allocation\n");
			break;

		case SET_INTERRUPT:
			verbose_printk(KERN_INFO"<ioctl>: Setting device as an Interrupt source with vector:%llx\n", arg_loc);

			//	init_waitqueue_head(mod_desc->iwq);

			/*Store the Interrupt Vector*/
			mod_desc->interrupt_vec = (u32)arg_loc;

			int_num = vec2num((u32)arg_loc);
			mod_desc->int_num = int_num;
			verbose_printk(KERN_INFO"<ioctl>: Interrupt Number:%d\n", int_num);

			interr_dict[int_num].int_count = mod_desc->int_count;
			interr_dict[int_num].mode = mod_desc->mode;

			interr_dict[int_num].atomic_poll = mod_desc->atomic_poll;
			//	interr_dict[int_num].iwq = mod_desc->iwq;
			//	interr_dict[int_num].int_count_sem = mod_desc->int_count_sem;
			//	mutex_init(mod_desc->int_count_sem);

			break;

		case SET_FILE_SIZE:
			mod_desc->file_size = ((loff_t)arg_loc & 0xffffffff);
			verbose_printk(KERN_INFO"<ioctl>: Setting device file size:%llu\n", mod_desc->file_size);

			/*initialize the DMA for the file*/
			ret = dma_file_init(mod_desc, dma_buffer_base, dma_buffer_size);
			if (ret < 0)
			{
				printk(KERN_INFO"<pci_open>:!!!!file open FAILURE!!!!.\n");
				return ERROR;
			}
			break;


		case SET_CDMA_KEYHOLE_WRITE:
			bit_vec = 0x00000020;   //the bit for KEYHOLE WRITE

			if(arg_loc>0)    //We are ENABLING keyhole write
			{
				verbose_printk(KERN_INFO"<ioctl_keyhole_write_set>: Setting the CDMA Keyhole WRITE as ENABLED\n");
				ret = cdma_config_set(bit_vec, 1, 1);   //value of one means we want to SET the register
			}
			else    //We are disabling keyhole write
			{
				verbose_printk(KERN_INFO"<ioctl_keyhole_write_disable>: Setting the CDMA Keyhole WRITE as DISABLED\n");
				ret = cdma_config_set(bit_vec, 0, 1);   //value of 0 means we want to UNSET the register
			}
			break;

		case SET_CDMA_KEYHOLE_READ:
			bit_vec = 0x00000010;   //the bit for KEYHOLE READ

			if(arg_loc>0)    //We are ENABLING keyhole write
			{
				verbose_printk(KERN_INFO"<ioctl_keyhole_read_set>: Setting the CDMA Keyhole READ as ENABLED\n");
				ret = cdma_config_set(bit_vec, 1, 1);   //value of one means we want to SET the register
			}
			else    //We are disabling keyhole write
			{
				verbose_printk(KERN_INFO"<ioctl_keyhole_read_disable>: Setting the CDMA Keyhole READ as DISABLED\n");
				ret = cdma_config_set(bit_vec, 0, 1);   //value of 0 means we want to UNSET the register
			}
			break;

		case GET_FILE_STATISTICS:

			ret = copy_from_user(statistics, (void *)arg, sizeof(struct statistics));

			//statistics = (struct statistics *)arg;
			statistics->tx_bytes = mod_desc->tx_bytes;
			statistics->rx_bytes = mod_desc->rx_bytes;
			statistics->cdma_attempt = mod_desc->cdma_attempt;
			statistics->ip_not_ready = mod_desc->ip_not_ready;
			statistics->cdma_usage_cnt = cdma_usage_cnt;
			cdma_usage_cnt = 0;

			if (mod_desc->stop_flag == 1)
			{
				mod_desc->stop_flag = 0;
				diff = timespec_sub(*(mod_desc->stop_time), *(mod_desc->start_time));
				statistics->seconds = (unsigned long)diff.tv_sec;
				statistics->ns = (unsigned long)diff.tv_nsec;
			}

			/*reset the counters*/
			mod_desc->tx_bytes = 0;
			mod_desc->rx_bytes = 0;
			mod_desc->ip_not_ready = 0;

			ret = copy_to_user((void *)arg, statistics, sizeof(struct statistics));

			break;

		case GET_DRIVER_STATISTICS:

			ret = copy_from_user(statistics, (void *)arg, sizeof(struct statistics));
			//	statistics = (struct statistics *)arg;
			statistics->tx_bytes = atomic_read(&driver_tx_bytes);
			statistics->rx_bytes = atomic_read(&driver_rx_bytes);

			if (atomic_read(&driver_stop_flag) == 1)
			{
				atomic_set(&driver_stop_flag, 0);
				diff = timespec_sub((driver_stop_time), (driver_start_time));
				statistics->seconds = (unsigned long)diff.tv_sec;
				statistics->ns = (unsigned long)diff.tv_nsec;
			}

			/*reset the counters*/
			atomic_set(&driver_tx_bytes, 0);
			atomic_set(&driver_rx_bytes, 0);
			//driver_ip_not_ready = 0;
			ret = copy_to_user((void *)arg, statistics, sizeof(struct statistics));
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
			verbose_printk(KERN_INFO"<ioctl_set_mode>: Setting the mode of the peripheral\n");
			*(mod_desc->mode) = (u32)arg_loc;

			switch((u32)arg_loc){

				case SLAVE:
					verbose_printk(KERN_INFO"<ioctl_set_mode>: Setting the mode of the peripheral to SLAVE\n");
					break;

				case MASTER:
					verbose_printk(KERN_INFO"<ioctl_set_mode>: Setting the mode of the peripheral to MASTER\n");

					/*DMA Allocation*/
					/*This sets the DMA read and write kernel buffers and obtains the
					 * dma_addr's to be sent to the CDMA core upon R/W transactions*/
					dma_master_buf[master_count] = pci_alloc_consistent(pci_dev_struct, 4096, &dma_m_addr[master_count]);
					if(NULL == dma_master_buf[master_count])
					{
						verbose_printk("%s:<probe>DMA AXI Master allocation ERROR\n", pci_devName);
						return ERROR;
					}

					/*write address to AXI to PCIe-M Translation Register*/

					/*write master_count as master_num to file descriptor to use in referencing its memory address*/
					mod_desc->master_num = master_count;
					break;

				case AXI_STREAM_FIFO:
					verbose_printk(KERN_INFO"<ioctl_set_mode>: Setting the mode of the peripheral to AXI_STREAM_FIFO\n");

					/*Initialize the AXI_STREAM_FIFO*/
					/*for now we will initialize all as interrupting for read*/
					//					/*check to see if the AXI addresses have been set*/
					if ((mod_desc->axi_addr == 0) | (mod_desc->axi_addr_ctl == 0))
					{
						printk(KERN_INFO"<ioctl_axi_stream_fifo>: ERROR: axi addresses of AXI STREAM FIFO not set\n");
						printk(KERN_INFO"<ioctl_axi_stream_fifo>:        set the AXI addresses then set mode again\n");
						return ERROR;
					}
					else
					{
						verbose_printk(KERN_INFO"<ioctl_axi_stream_fifo>: Initializing the FIFO and setting registers\n");

						/*allocate a small buffer of DMA for kernel to use*/
						mod_desc->dma_offset_internal_read = current_dma_offset_internal;
						mod_desc->dma_offset_internal_write = current_dma_offset_internal + 0x04;
						mod_desc->kernel_reg_read = (u32*)((u64)current_dma_offset_internal + (char*)dma_buffer_base);   //pointer to kernel read register
						mod_desc->kernel_reg_write = (u32*)((u64)(current_dma_offset_internal + 0x04) + (char*)dma_buffer_base); //pointer to kernel write register
						verbose_printk(KERN_INFO"<ioctl_axi_stream_fifo>: current dma kernel offset:%x\n", current_dma_offset_internal);
						verbose_printk(KERN_INFO"<ioctl_axi_stream_fifo>: kernel R/W register kernel addresses:%llx / %llx\n", (u64)mod_desc->kernel_reg_read, (u64)mod_desc->kernel_reg_write );
						current_dma_offset_internal = current_dma_offset_internal + 0x08;   // update the current dma buffer status (just added two 32b buffers)

						ret = axi_stream_fifo_init(mod_desc);
						if (ret < 0)
							return ERROR;
					}

					break;

				default:verbose_printk(KERN_INFO"<ioctl_set_mode> ERROR, improper mode type!\n");
			}
			break;
		default:printk(KERN_INFO"<pci_ioctl> ERROR, no command found.\n");
	}
	return 0;
}

loff_t pci_llseek( struct file *filp, loff_t off, int whence)
{
	loff_t newpos;

	switch(whence) {
		case 0: /*SEEK SET*/
			newpos = off;
			break;

		case 1: /*SEEK_CUR*/
			newpos = filp->f_pos + off;
			break;

			//		case 2: /*SEEK_END */
			//			newpos = dev->size + off;
			//			break;

		default: /* cant happen */
			return -EINVAL;
	}
	verbose_printk(KERN_INFO"<pci_read>: attempted seek\n");
	if (newpos < 0) return -EINVAL;
	filp->f_pos = newpos;
	return newpos;
}

int pci_poll(struct file *filep, poll_table * pwait)
{
	struct mod_desc * mod_desc;
	int has_data;
	unsigned int mask = 0;

	has_data = 0;
	verbose_printk(KERN_INFO"<pci_poll>:User Space has entered Poll()\n");
	mod_desc = filep->private_data;

	/*Register the poll table with the peripheral wait queue
	 *so that every wake_up on the wait queue will see if the
	 *peripheral has data*/

	poll_wait(filep, &wq_periph, pwait);
	//	poll_wait(filep, mod_desc->iwq, pwait);

	verbose_printk(KERN_INFO"<pci_poll>: Peripheral (' %x ') Interrupt Detected!!\n", mod_desc->int_num);

	/*see if the module has triggered an interrupt*/
	has_data = *(mod_desc->int_count);
	//	verbose_printk(KERN_INFO"<pci_poll>: Interrupt count of polling peripheral:%x\n", has_data);

	//	if (has_data > 0)
	if (atomic_read(mod_desc->atomic_poll))
	{
		verbose_printk(KERN_INFO"<pci_poll>: Interrupting Peripheral Matched!\n");
		/*reset the has_data flag*/

		atomic_set(mod_desc->atomic_poll, 0);
		*(mod_desc->int_count) = 0;
		//*(mod_desc->int_count) = *(mod_desc->int_count) - 1;

		mask |= POLLIN;
	}

	return mask;


}

ssize_t pci_write(struct file *filep, const char __user *buf, size_t count, loff_t *f_pos)
{
	//	void * kern_buf = NULL;
	u64 axi_dest;
	struct mod_desc * mod_desc;
	size_t bytes;
	size_t partial_count;
	size_t bytes_written;
	size_t remaining_size;
	int transfer_type;
	//	u32 kern_reg;
	int ret;
	u32 init_write;
	u64 dma_offset_write;
	u64 dma_offset_internal_read;
	u64 dma_offset_internal_write;

	bytes = 0;

	/*this gets the minor number of the calling file so we can map the correct AXI address
	 *to write to*/
	mod_desc = filep->private_data;


	bytes = 0;
	bytes_written = 0;

	verbose_printk(KERN_INFO"\n\n");
	verbose_printk(KERN_INFO"<pci_write>: ************************************************************************\n");
	verbose_printk(KERN_INFO"<pci_write>: ******************** WRITE TRANSACTION BEGIN  **************************\n");
	verbose_printk(KERN_INFO"<pci_write>:                  Attempting to transfer %zu bytes\n", count);
	verbose_printk(KERN_INFO"<pci_write>: ************************************************************************\n\n");

	verbose_printk(KERN_INFO"WRITE file struct offset: %x", filep->f_pos);
	verbose_printk(KERN_INFO"WRITE offset param: %x", *f_pos);

	if(mod_desc->set_dma_flag == 0)
	{
		ret = dma_file_init(mod_desc, dma_buffer_base, dma_buffer_size);
		if (ret < 0)
		{
			printk(KERN_INFO"<pci_write>:!!!! DMA init FAILURE!!!!.\n");
			return ERROR;
		}
		printk(KERN_INFO"<pci_write>: Warning - Set the DMA file size to default value %d. IOCTL SET_FILE_SIZE was never called for file minor: %d.\n", (int)mod_desc->file_size, (int)mod_desc->minor);
		mod_desc->set_dma_flag = 1;
	}

	/*Start timers for statistics*/
	if (mod_desc->start_flag == 1)
	{
		getnstimeofday(mod_desc->start_time);
		mod_desc->start_flag = 0;
	}

	if (atomic_read(&driver_start_flag) == 1)
	{
		getnstimeofday(&driver_start_time);
		atomic_set(&driver_start_flag, 0);
	}

	/*Stay until requested transmission is complete*/
	while (bytes_written < count)
	{
		partial_count = count - bytes_written;

		if (partial_count > mod_desc->dma_size)
		{
			remaining_size = count - bytes_written;
			verbose_printk(KERN_INFO"<pci_write>: the current DMA Buffer size of: 0x%x is not large enough for *remaining* transfer size:%zu bytes\n", mod_desc->dma_size, remaining_size);
			partial_count = mod_desc->dma_size;
		}

		verbose_printk(KERN_INFO"<pci_write>: the amount of bytes being copied to kernel: %zu\n", partial_count);

		/*eventually this will go away once we add mmap
		 * for now we copy to the appropriate file buffer*/
		verbose_printk(KERN_INFO"<pci_write>: copying data from user space...\n");

		/*the pointer of data to be transferred is incremented by the amout of bytes
		 * already written.  This handles the case when a chunk of data larger than
		 * the dma buffer size is attempted to be written*/
		ret = copy_from_user(mod_desc->dma_write, (buf + bytes_written), partial_count);
		//	copy_from_user(mod_desc->dma_write, buf, partial_count);



		if (*(mod_desc->mode) == AXI_STREAM_FIFO)
		{

			bytes = axi_stream_fifo_write(partial_count, mod_desc);
			if (bytes < 0)
			{
				printk(KERN_INFO"<pci_write>: Write Error, exiting write routine...\n\n\n");
				return -1;
			}
			if (bytes == 0)
			{
				return bytes_written;
			}

		}

		else
		{
			/* Here we will decide whether to do a zero copy DMA, or to write */
			/* directly to the peripheral */
			init_write = *((u32*)(mod_desc->dma_write));
			dma_offset_write = (u64)mod_desc->dma_offset_write;
			dma_offset_internal_read = (u64)mod_desc->dma_offset_internal_read;
			dma_offset_internal_write = (u64)mod_desc->dma_offset_internal_write;

			verbose_printk(KERN_INFO"<pci_write>: writing peripheral with starting value: %x\n", init_write);
			verbose_printk(KERN_INFO"<pci_write>: DMA offset write value: %llx\n", dma_offset_write);
			verbose_printk(KERN_INFO"<pci_write>: DMA internal offset write value: %llx\n", dma_offset_internal_write);
			verbose_printk(KERN_INFO"<pci_write>: DMA internal offset read value: %llx\n", dma_offset_internal_read);

			axi_dest = mod_desc->axi_addr + *f_pos;

			if (mod_desc->keyhole_config & 0x1)
				transfer_type = KEYHOLE_WRITE;
			else
				transfer_type = NORMAL_WRITE;

			verbose_printk(KERN_INFO"<pci_write>: writing peripheral using a transfer_type: %x\n", transfer_type);

			verbose_printk(KERN_INFO"<user_peripheral_write>: current file offset is: %llx\n", *f_pos);

			if (transfer_type == NORMAL_WRITE)
			{
				/*Check to see if write will go past the boundary*/
				if((partial_count + *f_pos) > mod_desc->file_size)
				{
					verbose_printk(KERN_INFO"<user_peripheral_write>: End of file overrun!\n");
					partial_count = (size_t)(mod_desc->file_size - *f_pos);
					verbose_printk(KERN_INFO"<user_peripheral_write>: Only writing %zu bytes to end of file!\n", partial_count);
					//return bytes_written + partial_count;
				}
			}

			ret = data_transfer(axi_dest, 0, partial_count, transfer_type, dma_offset_write);
			if (ret > 0)
			{
				printk(KERN_INFO"<pci_write>: ERROR writing to User Peripheral\n");
				return -1;
			}

			if (transfer_type == NORMAL_WRITE)
			{
				*f_pos = *f_pos + partial_count;

				if (*f_pos == mod_desc->file_size)
				{
					*f_pos = 0;
					verbose_printk(KERN_INFO"<user_peripheral_write>: Resetting file pointer back to zero...\n");
				}
				else if (*f_pos > mod_desc->file_size)
				{
					printk(KERN_INFO"<user_peripheral_write>: ERROR! Wrote past the file size. This should not have happened...\n");
					printk(KERN_INFO"<user_peripheral_write>: Resetting file pointer back to zero...\n");
					*f_pos = 0;
					return -1;
				}
				verbose_printk(KERN_INFO"<user_peripheral_write>: updated file offset is: %llx\n", *f_pos);
			}

		}

		bytes_written = bytes_written + partial_count;
		verbose_printk(KERN_INFO"<pci_write>: Wrote %zu bytes in this pass.\n", partial_count);
	}

	//bytes = bytes_written;

	//msleep(5000);   //experiment....

	//file statistics
	mod_desc->tx_bytes = mod_desc->tx_bytes + bytes_written;
	atomic_add(bytes_written, &driver_tx_bytes);
	//printk(KERN_INFO"total file tx byes: %d \n", mod_desc->tx_bytes);

	/*always update the stop_timer*/
	getnstimeofday(mod_desc->stop_time);
	getnstimeofday(&driver_stop_time);

	verbose_printk(KERN_INFO"\n");
	verbose_printk(KERN_INFO"<pci_write>: ************************************************************************\n");
	verbose_printk(KERN_INFO"<pci_write>: ******************** WRITE TRANSACTION END  **************************\n");
	verbose_printk(KERN_INFO"<pci_write>: Wrote a total of %zu bytes in write call.\n", bytes_written);
	verbose_printk(KERN_INFO"<pci_write>: ************************************************************************\n");
	verbose_printk(KERN_INFO"\n");
	return bytes_written;

}

ssize_t pci_read(struct file *filep, char __user *buf, size_t count, loff_t *f_pos)
{
	//	u32 kern_reg;
	u64 axi_dest;
	struct mod_desc *mod_desc;
	int bytes = 0;
	int transfer_type;
	//	void * read_buf = NULL;
	int ret;
	size_t temp;
	u64 dma_offset_write;
	u64 dma_offset_read;
	u64 dma_offset_internal_read;
	u64 dma_offset_internal_write;

	mod_desc = filep->private_data;

	temp = 0;

	verbose_printk(KERN_INFO"\n\n");
	verbose_printk(KERN_INFO"<pci_read>: ************************************************************************\n");
	verbose_printk(KERN_INFO"<pci_read>: ******************** READ TRANSACTION BEGIN  **************************\n");
	verbose_printk(KERN_INFO"<pci_read>: ************************************************************************\n");

	verbose_printk(KERN_INFO"READ file struct offset: %x", filep->f_pos);
	verbose_printk(KERN_INFO"READ offset param: %x", *f_pos);

	if(mod_desc->set_dma_flag == 0)
	{
		ret = dma_file_init(mod_desc, dma_buffer_base, dma_buffer_size);
		if (ret < 0)
		{
			printk(KERN_INFO"<pci_read>:!!!! DMA init FAILURE!!!!.\n");
			return ERROR;
		}
		printk(KERN_INFO"<pci_write>: Warning - Set the DMA file size to default value %d. IOCTL SET_FILE_SIZE was never called for file minor: %d.\n", (int)mod_desc->file_size, (int)mod_desc->minor);
		mod_desc->set_dma_flag = 1;
	}

	verbose_printk(KERN_INFO"<pci_read>: Attempting to read %zu bytes\n", count);

	if (mod_desc->start_flag == 1)
	{
		getnstimeofday(mod_desc->start_time);
		mod_desc->start_flag = 0;
	}

	if (atomic_read(&driver_start_flag) == 1)
	{
		getnstimeofday(&driver_start_time);
		atomic_set(&driver_start_flag, 0);
	}

	if (count > mod_desc->dma_size)
	{
		verbose_printk(KERN_INFO"<pci_read>: Attempting to read more than the allocated DMA size of:%zu\n", mod_desc->dma_size);
		verbose_printk(KERN_INFO"<pci_read>: readjusting the read amount to:%zu\n", mod_desc->dma_size);
		count = (size_t)mod_desc->dma_size;
	}

	switch(*(mod_desc->mode)){

		case MASTER:
			//Transfer buffer from kernel space to user space at the allocated DMA region
			verbose_printk(KERN_INFO"<pci_read>: Transferred a Master write from kernel space to user space\n");
			ret = copy_to_user(buf, dma_master_buf[mod_desc->master_num], count);
			break;

		case AXI_STREAM_FIFO:

			bytes = axi_stream_fifo_read(count, mod_desc);
			if (bytes < 0)
			{
				verbose_printk(KERN_INFO"<user_peripheral_read>: ERROR reading data from axi stream fifo\n");
			}

			break;

		case SLAVE:
			/* Here we will decide whether to do a zero copy DMA, or to read */
			/* directly from the peripheral */

			dma_offset_write = (u64)mod_desc->dma_offset_write;
			dma_offset_read = (u64)mod_desc->dma_offset_read;
			dma_offset_internal_read = (u64)mod_desc->dma_offset_internal_read;
			dma_offset_internal_write = (u64)mod_desc->dma_offset_internal_write;

			axi_dest = mod_desc->axi_addr + *f_pos;

			if (mod_desc->keyhole_config & 0x1)
				transfer_type = KEYHOLE_READ;
			else
				transfer_type = NORMAL_READ;


			verbose_printk(KERN_INFO"<user_peripheral_read>: reading peripheral using a transfer_type: %x\n", transfer_type);

			verbose_printk(KERN_INFO"<user_peripheral_write>: current file offset is: %llu\n", *f_pos);

			temp = count + *f_pos;

			/*Check to see if read will go past the boundary*/
			if(temp > (size_t)mod_desc->file_size)
			{
				verbose_printk(KERN_INFO"<user_peripheral_read>: Read will overrun the file size because \n");
				verbose_printk(KERN_INFO"<user_peripheral_read>: (the current file offset + amount to read)->(%llu) > (%llu)->file_size\n", temp, mod_desc->file_size);
				count = (size_t)(mod_desc->file_size - *f_pos);
				verbose_printk(KERN_INFO"<user_peripheral_read>: Adjusting to only reading %zu bytes to end of file!\n", count);

			}

			ret = data_transfer(axi_dest, 0, count, transfer_type, dma_offset_read);
			if (ret > 0)
			{
				printk(KERN_INFO"<user_peripheral_read>: ERROR reading data from User Peripheral\n");
				return -1;
			}

			if (transfer_type == NORMAL_READ)
			{

				*f_pos = (loff_t)(*f_pos + (loff_t)count);
				verbose_printk(KERN_INFO"<user_peripheral_write>: updated file offset after adding count(%zu) is: %llu\n", count, *f_pos);

				if (*f_pos == mod_desc->file_size)
				{
					*f_pos = 0;
					verbose_printk(KERN_INFO"<user_peripheral_read>: End of file reached.\n");
					verbose_printk(KERN_INFO"<user_peripheral_read>: Resetting file pointer back to zero...\n");
					verbose_printk(KERN_INFO"<user_peripheral_write>: updated file offset is: %llu\n", *f_pos);
				}
				else if (*f_pos > mod_desc->file_size)
				{
					printk(KERN_INFO"<user_peripheral_read>: ERROR! Read past the file size. This should not have happened...\n");
					printk(KERN_INFO"<user_peripheral_read>: the offset position is:%llu\n", *f_pos);
					printk(KERN_INFO"<user_peripheral_read>: Resetting file pointer back to zero...\n");
					*f_pos = 0;
					printk(KERN_INFO"<user_peripheral_write>: updated file offset is: %llu\n", *f_pos);
					return -1;
				}

			}

			bytes = count;

			break;

		default:verbose_printk(KERN_INFO"<pci_read>: mode not detected on read\n");
	}


	/*eventually this will go away once we add mmap
	 * for now we copy to the appropriate file buffer*/
	ret = copy_to_user(buf, mod_desc->dma_read, count);

	//printk(KERN_INFO"total file rx byes: %d \n", mod_desc->rx_bytes);

	/*always update the stop_timer*/
	if (bytes > 0)
	{
		getnstimeofday(mod_desc->stop_time);
		getnstimeofday(&driver_stop_time);
		mod_desc->rx_bytes = mod_desc->rx_bytes + bytes;
		atomic_add(bytes, &driver_rx_bytes);
	}

	verbose_printk(KERN_INFO"\n");
	verbose_printk(KERN_INFO"<pci_read>: ************************************************************************\n");
	verbose_printk(KERN_INFO"<pci_read>: ******************** READ TRANSACTION END  **************************\n");
	verbose_printk(KERN_INFO"                                    Bytes read : %d\n", bytes);
	verbose_printk(KERN_INFO"<pci_read>: ************************************************************************\n");
	verbose_printk(KERN_INFO"\n");

	return bytes;
}
