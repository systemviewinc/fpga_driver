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
#define printk(...)


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
#include "Common.h"
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/atomic.h>

#define ERROR   -1
#define SUCCESS 0

/*MODES*/
#define SLAVE 0 
#define AXI_STREAM_FIFO 1
#define MASTER 2
#define CDMA 3


#define MAX_NUM_MASTERS 2
#define MAX_NUM_SLI 4

#define MAX_NUM_INT MAX_NUM_MASTERS + MAX_NUM_SLI

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

/*Other Constants*/
#define KEYHOLE_WRITE 2 
#define NORMAL_WRITE 0
#define KEYHOLE_READ 3
#define NORMAL_READ 1

#define PCI 1
#define PLATFORM 2

/***********Set default values for insmod parameters***************************/
static int device_id = 100;
static int major = 241;
static int cdma_address = 0xFFFFFFFF;
static int cdma_address_2 = 0xFFFFFFFF;
//static bool enable_cdma_2 = false;
static int pcie_ctl_address = 0xFFFFFFFF;
static int pcie_m_address = 0xFFFFFFFF;
static int pcie_m_address_2 = 0xFFFFFFFF;
static int int_ctlr_address = 0xFFFFFFFF;
static int driver_type = PCI;

module_param(device_id, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(device_id, "DeviceID");

module_param(major, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(major, "MajorNumber");

//module_param(enable_cdma_2, bool, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
//MODULE_PARM_DESC(enable_cdma_2, "EnableCDMA2");

module_param(cdma_address, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(cdma_address, "CDMAAddress");

module_param(cdma_address_2, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(cdma_address_2, "CDMAAddress2");

module_param(pcie_ctl_address, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(pcie_ctl_address, "PCIeCTLAddress");

module_param(pcie_m_address, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(pcie_m_address, "PCIeMAddress");

module_param(pcie_m_address_2, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(pcie_m_address_2, "PCIeMAddress2");

module_param(int_ctlr_address, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(int_ctlr_address, "IntCtlrAddress");

module_param(driver_type, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(driver_type, "DriverType");
/*****************************************************************************/

const char pci_devName[] = "pci_skel"; //name of the device
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
static u64 peripheral_space_offset = 0x80000000;
static u64 bar_0_axi_offset = 0x40000000;
static u64 peripheral_space_1_offset = 0xC00000000;

u64 axi_cdma;
u64 axi_cdma_2;
u64 axi_pcie_ctl;
u64 axi_interr_ctrl;
u64 axi_pcie_m[5];

//u32 axi_dev0;                 // base address of axi slave mapped to minor 0
//u32 axi_dev1;                 // base address of axi slave mapped to minor 1
//u32 axi_dev2;                 // base address of axi slave mapped to minor 2
u8 cdma_set[5];
u8 pcie_ctl_set;
static u8 zero_buf_set[5];
u8 pcie_m_set[5];
u8 int_ctrl_set;
void * zero_copy_buf[5];
int cdma_status;
static int cdma_capable = 0;
unsigned int irq_num;

/*CDMA Semaphore*/
struct mutex CDMA_sem;
struct mutex CDMA_sem_2;

//const int pci_major = 241;             //major number not dynamic

dma_addr_t dma_addr[5];

dma_addr_t dma_addr_base;
void * dma_buffer_base;
u32 dma_current_offset;
u64 dma_buffer_size = 1048576;
//u64 dma_buffer_size = 131072;
u32 current_dma_offset_internal;

dma_addr_t dma_m_addr[MAX_NUM_MASTERS];

void * dma_master_buf[MAX_NUM_MASTERS];

/* *************************************************  */

size_t dma_size;              //this is a global variable to interrupt to recognize
static wait_queue_head_t wq;
static wait_queue_head_t wq_periph;
static wait_queue_head_t mutexq;
static int cdma_comp[5];
int num_int;
int interrupt_vect_dict[1 << (MAX_NUM_INT)];    //2^x

atomic_t mutex_free = ATOMIC_INIT(0); 

/*These are register offsets of Xilinx peripherals*/
const u32 AXIBAR2PCIEBAR_0L = 0x20c;
const u32 AXIBAR2PCIEBAR_1L = 0x214;
const u32 CDMA_CR           = 0x00;
const u32 CDMA_SR           = 0x04;
const u32 CDMA_DA           = 0x20;
const u32 CDMA_DA_MSB       = 0x24;
const u32 CDMA_SA           = 0x18;
const u32 CDMA_SA_MSB       = 0x1C;
const u32 CDMA_BTT          = 0x28;

const u32 INT_CTRL_IER      = 0x08;
const u32 INT_CTRL_MER      = 0x1c;
const u32 INT_CTRL_ISR      = 0x00;
const u32 INT_CTRL_IAR      = 0x0C;

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


/*this is the module description struct*/

struct mod_desc 
{
	int minor;
	u64 axi_addr;
	u64 axi_addr_ctl;
	int * mode;
	int * int_count;
	int int_num;
	int master_num;
	int keyhole_config;
	u32 interrupt_vec;
	u32 dma_offset_read;
	u32 dma_offset_write;
	u64 dma_size;
	void * dma_write;
	void * dma_read;
	u32 * kernel_reg_write;
	u32 * kernel_reg_read;
	u32 dma_offset_internal_read;
	u32 dma_offset_internal_write;
};

/*this is the interrupt structure*/
struct interr_struct
{
	int * mode;
	int * int_count; 
};

/*This is an array of interrupt structures to hold up to 8 peripherals*/
struct interr_struct interr_dict[8];

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
/* ********************** other functions **************************8 */
int cdma_transfer(u64 SA, u64 DA, u32 BTT, int keyhole_en, int cdma_num);
int cdma_ack(int cdma_num);
void cdma_config_set(u32 bit_vec, int set_unset, int cdma_num);
int direct_read(u64 axi_address, void *buf, size_t count, int transfer_type);
int direct_write(u64 axi_address, void *buf, size_t count, int transfer_type);
int data_transfer(u64 axi_address, void *buf, size_t count, int transfer_type, u64 dma_offset);
int vec2num(u32 vec);
int cdma_query(void);
u32 num2vec(int num);
void cdma_init(int cdma_num);
void pcie_ctl_init(u64 axi_address);
void pcie_m_init(int cdma_num);
void int_ctlr_init(u64 axi_address);
/* ****************************************************************** */

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
	/* Do probing type stuff here.  
	 * 	 * Like calling request_region();
	 * 	 	 */

	printk(KERN_INFO"%s:<probe>******************************** BEGIN PROBE ROUTINE *****************************************\n", pci_devName);

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
	printk(KERN_INFO"<probe>pci bar 1 size is:%lu\n", pci_bar_1_size);

	//map the hardware space to virtual space
	if (pci_bar_1_size > 0)
	{
		pci_bar_1_vir_addr = ioremap(pci_bar_1_addr, pci_bar_1_size);
		if(0 == pci_bar_1_vir_addr){
			printk(KERN_INFO"%s:<probe>ioremap error when mapping to virtual address\n", pci_devName);
			return ERROR;
		}
		printk(KERN_INFO"<probe>pci bar 1 virtual address base is:%p\n", pci_bar_1_vir_addr);
	}
	else
	{
		peripheral_space_offset = 0;
		printk(KERN_INFO"%s<probe>BAR 1 is not in use\n", pci_devName);
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
	printk(KERN_INFO"<probe>pci bar 2 size is:%lu\n", pci_bar_2_size);

	if (pci_bar_2_size > 0)
	{
		//map the hardware space to virtual space
		pci_bar_2_vir_addr = ioremap(pci_bar_2_addr, pci_bar_2_size);
		if(0 == pci_bar_2_vir_addr){
			printk(KERN_INFO"%s:<probe>ioremap error when mapping to virtual address\n", pci_devName);
			return ERROR;
		}
		printk(KERN_INFO"<probe>pci bar 2 virtual address base is:%p\n", pci_bar_2_vir_addr);
	}
	else
	{
		peripheral_space_1_offset = 0;
		printk(KERN_INFO"%s<probe>BAR 2 is not in use\n", pci_devName);
	}
	/*****************************************************************************/

	//enable the device
	pci_enable_device(dev);
	printk(KERN_INFO"<probe>device enabled\n");

	//set DMA mask
	if(0 != dma_set_coherent_mask(&dev->dev, 0x00000000FFFFFFFF)){
		printk(KERN_INFO"%s:<probe>set DMA mask error\n", pci_devName);
		return ERROR;
	}
	printk(KERN_INFO"<probe>dma mask set\n");

	//enable bus mastering
	pci_set_master(dev);
	printk(KERN_INFO"<probe>pci set as master\n");

	//enable MSI interrupts
	if(0 > pci_enable_msi(pci_dev_struct)){
		printk(KERN_INFO"%s:<probe>MSI enable error\n", pci_devName);
		return ERROR;
	}
	printk(KERN_INFO"<probe>pci enabled msi interrupt\n");

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
		return -1;
	}
	else
	{
		printk(KERN_INFO"<sv_driver_init>: dma kernel buffer base address is:%lx\n", (u64)dma_buffer_base);
		printk(KERN_INFO"<sv_driver_init>: dma system memory buffer base address is:%lx\n", (u64)dma_addr_base);
		dma_current_offset = 4096;   //we want to leave the first 4k for the kernel to use internally.
		current_dma_offset_internal = 0;
	}

	//set defaults
	cdma_set[0] = 0;
	cdma_set[1] = 0;
	pcie_ctl_set = 0;
	pcie_m_set[0] = 0;
	pcie_m_set[1] = 0;
	int_ctrl_set = 0;

	if (cdma_address != 0xFFFFFFFF)
	{
		cdma_init(1);  //cdma_num = 1
	}

	if (cdma_address_2 != 0xFFFFFFFF)
	{
		cdma_init(2);  //cdma_num = 2
	}

	if (pcie_ctl_address != 0xFFFFFFFF)
	{
		pcie_ctl_init((u64)pcie_ctl_address);
	}

	if (pcie_m_address != 0xFFFFFFFF)
	{
		pcie_m_init(1);
	}

	if (pcie_m_address_2 != 0xFFFFFFFF)
	{
		pcie_m_init(2);
	}

	if (int_ctlr_address != 0xFFFFFFFF)
	{
		int_ctlr_init((u64)int_ctlr_address);
	}

	cdma_capable = (cdma_set[1] == 1) & (pcie_m_set[1] == 1) & (int_ctrl_set == 1) & (pcie_ctl_set == 1);
	printk(KERN_INFO"<probe> cdma_capable = %x\n", cdma_capable);

	/*Since we isolated the AXI bus, this is the new offest for the slave interface of the PCIe*/
//	axi_pcie_m = 0x00000;

	printk(KERN_INFO"<probe>***********************PROBE FINISHED SUCCESSFULLY**************************************\n");
	return 0;
}

static int sv_plat_probe(struct platform_device *pdev)
{
	struct resource * resource_1;
//	struct resource * resource_2;

	printk(KERN_INFO"%s:<probe>******************************** BEGIN PROBE ROUTINE *****************************************\n", pci_devName);

	platform_dev_struct = pdev;
	dev_struct = &pdev->dev;

	resource_1 = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	//get the base memory size
	//	pci_bar_size = pci_resource_len(pci_dev_struct, 0);
	pci_bar_size = resource_1->end - resource_1->start;
	printk(KERN_INFO"<probe>pci bar size is:%lu\n", pci_bar_size);

	//map the hardware space to virtual space
	pci_bar_vir_addr = devm_ioremap_resource(&pdev->dev, resource_1);

	if(0 == pci_bar_vir_addr){
		printk(KERN_INFO"%s:<probe>ioremap error when mapping to vritaul address\n", pci_devName);
		return ERROR;
	}
	printk(KERN_INFO"<probe>pci bar virtual address base is:%p\n", pci_bar_vir_addr);

	/****************** BAR 1 Mapping *******************************************/

	//	resource_2 = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	//get the base memory size
	//	pci_bar_1_size = resource_2->end - resource_2->start;
	//	printk(KERN_INFO"<probe>pci bar 1 size is:%d\n", pci_bar_1_size);

	//map the hardware space to virtual space
	//	pci_bar_1_vir_addr = devm_ioremap_resource(&pdev->dev, resource_2);
	//	if(0 == pci_bar_1_vir_addr){
	//		printk(KERN_INFO"%s:<probe>ioremap error when mapping to virtual address\n", pci_devName);
	//		return ERROR;
	//	}
	//	printk(KERN_INFO"<probe>pci bar 1 virtual address base is:%p\n", pci_bar_1_vir_addr);
	peripheral_space_offset = 0;
        pci_bar_1_size = 0;

	/*****************************************************************************/


	//set DMA mask
	if(0 != dma_set_mask(&pdev->dev, 0x00000000FFFFFFFF)){
		printk(KERN_INFO"%s:<probe>set DMA mask error\n", pci_devName);
		return ERROR;
	}
	printk(KERN_INFO"<probe>dma mask set\n");

	irq_num = platform_get_irq(pdev, 0);
	printk(KERN_INFO"<probe>IRQ number is:%x\n", irq_num);

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
	dma_buffer_base = dma_alloc_coherent(dev_struct, (size_t)dma_buffer_size, dma_addr_base, GFP_KERNEL);
	if(NULL == dma_buffer_base)
	{
		printk("%s:<sv_driver_init>DMA buffer base allocation ERROR\n", pci_devName);
		return -1;
	}
	else
	{
		printk(KERN_INFO"<sv_driver_init>: dma buffer base address is:%lx\n", (u64)dma_buffer_base);
		printk(KERN_INFO"<sv_driver_init>: dma system memory buffer base address is:%lx\n", (u64)dma_addr_base);
		dma_current_offset = 4096;   //we want to leave the first 4k for the kernel to use internally.
		current_dma_offset_internal = 0;
	}

	//set defaults
	cdma_set[1] = 0;
	cdma_set[2] = 0;
	pcie_ctl_set = 0;
	pcie_m_set[1] = 0; 
	pcie_m_set[2] = 0; 

	if (cdma_address != 0xFFFFFFFF)
	{
		cdma_init(1);
	}

	if (cdma_address_2 != 0xFFFFFFFF)
	{
		cdma_init(2);
	}

	if (pcie_m_address != 0xFFFFFFFF)
	{
		pcie_m_init(1);
	}

	if (pcie_m_address_2 != 0xFFFFFFFF)
	{
		pcie_m_init(2);
	}

	if (int_ctlr_address != 0xFFFFFFFF)
	{
		int_ctlr_init((u64)int_ctlr_address);
	}

	/*ARM works a little differently for DMA than PCIe in that that translation
	 * is not handled by the core by writing to a register. For Zynq, the axi to DDR address
	 * mapping is 1-1 and should be written directly to the returned DMA handle */

//	axi_pcie_m[1] = axi_pcie_m[1] + (u64)dma_addr[1];   //cdma 1
//	axi_pcie_m[2] = axi_pcie_m[2] + (u64)dma_addr[2];   //cdma 2

	axi_pcie_m[1] = axi_pcie_m[1] + (u64)dma_addr_base;   //cdma 1
	axi_pcie_m[2] = axi_pcie_m[2] + (u64)dma_addr_base;   //cdma 2

	cdma_capable = (cdma_set[1] == 1) & (pcie_m_set[1] == 1) & (int_ctrl_set == 1);

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

	if(zero_buf_set[1] == 1) 
	{
		//		pci_free_consistent(pci_dev_struct, 131072, zero_copy_buf, dma_addr);
		dma_free_coherent(dev_struct, 131072, zero_copy_buf[1], dma_addr[1]);
	}

	if(zero_buf_set[2] == 1) 
	{
		//		pci_free_consistent(pci_dev_struct, 131072, zero_copy_buf, dma_addr);
		dma_free_coherent(dev_struct, 131072, zero_copy_buf[2], dma_addr[2]);
	}

	dma_free_coherent(dev_struct, (size_t)dma_buffer_size, dma_buffer_base, dma_addr_base);
	
	unregister_chrdev(major, pci_devName);

}

static int sv_plat_remove(struct platform_device *pdev)
{
	iounmap(pci_bar_vir_addr);

	free_irq(irq_num, platform_dev_struct);

	if(zero_buf_set[1] == 1) 
	{
		//		pci_free_consistent(pci_dev_struct, 131072, zero_copy_buf, dma_addr);
		dma_free_coherent(dev_struct, 131072, zero_copy_buf[1], dma_addr[1]);
	}

	if(zero_buf_set[2] == 1) 
	{
		//		pci_free_consistent(pci_dev_struct, 131072, zero_copy_buf, dma_addr);
		dma_free_coherent(dev_struct, 131072, zero_copy_buf[2], dma_addr[2]);
	}

	dma_free_coherent(dev_struct, (size_t)dma_buffer_size, dma_buffer_base, dma_addr_base);

	unregister_chrdev(major, pci_devName);

	printk(KERN_INFO"<remove>***********************PLATFORM DEVICE REMOVED**************************************\n");

	return 0;
}

static struct pci_driver pci_driver = {
	.name = "pci_skel",
	//	.name = pci_devName,
	.id_table = ids,
	.probe = probe,
	.remove = remove,
};


static int __init sv_driver_init(void)
{
	
	switch(driver_type){
		case PCI:
			printk(KERN_INFO"<pci_init>: Device ID: ('%d')\n", device_id);
			printk(KERN_INFO"<pci_init>: Major Number: ('%d')\n", major);
		
			init_waitqueue_head(&wq);
			init_waitqueue_head(&wq_periph);
			init_waitqueue_head(&mutexq);

			ids[0].vendor =  PCI_VENDOR_ID_XILINX;
			ids[0].device =  (u32)device_id;

			return pci_register_driver(&pci_driver);
			break;

		case PLATFORM:

			printk(KERN_INFO"<platform_init>: Device ID: ('%d')\n", device_id);
			printk(KERN_INFO"<platform_init>: Major Number: ('%d')\n", major);

			init_waitqueue_head(&wq);
			init_waitqueue_head(&wq_periph);
			init_waitqueue_head(&mutexq);

			bar_0_axi_offset = 0x40000000;

			return platform_driver_register(&sv_plat_driver);

			break;

		default:;
	}

	printk(KERN_INFO"<platform_init>: !!!!ERROR!!!!! No correct driver type detected!\n");
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
	int device_mode;
	int ret;
	u32 vec_serviced;

	vec_serviced = 0;

	printk(KERN_INFO"<pci_isr>: Entered the pci ISR");

	/*Here we need to find out who triggered the interrupt*
	 *Since we only allow one MSI vector, we need to query the
	 *Interrupt controller to find out. */

	/*This is the interrupt status register*/
	axi_dest = axi_interr_ctrl + INT_CTRL_ISR;
	ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_READ, 0);
	printk(KERN_INFO"<pci_isr>: interrupt status register vector is: ('%x')\n", status);


	while(status > 0)
	{
		int_num = vec2num(status);
		printk(KERN_INFO"<pci_isr>: interrupt number is: ('%d')\n", int_num);
		device_mode = *(interr_dict[int_num].mode);
	
		if (device_mode == CDMA)
		{
			printk(KERN_INFO"<pci_isr>: this interrupt is from the CDMA\n");
			//cdma_status = cdma_ack();
			vec_serviced = vec_serviced | num2vec(int_num);
			/*Here we need to clear the service interrupt in the interrupt acknowledge register*/
		//	axi_dest = axi_interr_ctrl + INT_CTRL_IAR;
		//	status = num2vec(int_num);
		//	ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_WRITE);

		//	cdma_comp = 1;      //condition for wake_up
		//	wake_up_interruptible(&wq);
		}

		else
		{
			printk(KERN_INFO"<pci_isr>: this interrupt is from a user peripheral\n");
			*(interr_dict[int_num].int_count) = (*(interr_dict[int_num].int_count)) + 1;

			vec_serviced = vec_serviced | num2vec(int_num);
			/*Here we need to clear the service interrupt in the interrupt acknowledge register*/
		//	axi_dest = axi_interr_ctrl + INT_CTRL_IAR;
		//	status = num2vec(int_num);
		//	ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_WRITE);

			/*since we don't know exactly how to handle interrupt clearing of this device
			 *we push the interrupt acknowleding of the device to the user space */

		//	wake_up(&wq_periph);
		}

	/*Here we need to clear the service interrupt in the interrupt acknowledge register*/
	axi_dest = axi_interr_ctrl + INT_CTRL_IAR;
	status = num2vec(int_num);
	ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_WRITE, 0);

	/*This is the interrupt status register*/
	axi_dest = axi_interr_ctrl + INT_CTRL_ISR;
	printk(KERN_INFO"<pci_isr>: Checking the ISR vector for any additional vectors....\n");
	ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_READ, 0);

	/*Here we mask off any vector we already know about*/
//	status = status & (~vec_serviced); 

//	printk(KERN_INFO"<pci_isr>: the new interrupt status register vector is: ('%x')\n", status);

	}

	/*Here we need to clear the service interrupt in the interrupt acknowledge register*/
//	axi_dest = axi_interr_ctrl + INT_CTRL_IAR;
//	ret = data_transfer(axi_dest, (void *)&vec_serviced, 4, NORMAL_WRITE);

	printk(KERN_INFO"<pci_isr>: All interrupts serviced. The following Vector is acknowledged: %x\n", vec_serviced);

	/* The CDMA vectors (1 and 2) */
	if ((vec_serviced & 0x01) == 0x01) 
	{	
		cdma_comp[1] = 1;      //condition for wake_up
		printk(KERN_INFO"<pci_isr>: Waking up CDMA 1\n");
		wake_up_interruptible(&wq);
	}

	if ((vec_serviced & 0x02) == 0x02)
	{	
		cdma_comp[2] = 1;      //condition for wake_up
		printk(KERN_INFO"<pci_isr>: Waking up CDMA 2\n");
		wake_up_interruptible(&wq);
	}

//	if (vec_serviced & 0x10)
	if (vec_serviced >= 0x10)
		wake_up(&wq_periph);

	printk(KERN_INFO"<pci_isr>: Exiting ISR\n");
	return IRQ_HANDLED;
}

/* -----------------File Operations-------------------------*/
int pci_open(struct inode *inode, struct file *filep)
{
	void * mode_address;
	void * interrupt_count;
	void * kern_reg_write;
	void * kern_reg_read;

	mode_address = kmalloc(sizeof(int), GFP_KERNEL);
	interrupt_count = kmalloc(sizeof(int), GFP_KERNEL);

	struct mod_desc * s;

	s = (struct mod_desc *)kmalloc(sizeof(struct mod_desc), GFP_KERNEL);
	s->minor = MINOR(inode->i_rdev);
	s->axi_addr = 0;
	s->axi_addr_ctl = 0;
	s->mode = (int*)mode_address;   //defaults as slave only
	*(s->mode) = 0;
	//	s->wait_var = 0;
	s->int_count = (int*)interrupt_count;
	*(s->int_count) = 0;
	s->int_num = 100;
	s->master_num = 0;
	s->interrupt_vec = 0;
	s->keyhole_config = 0;
	s->dma_offset_read = 0;
	s->dma_offset_write = 0;
	s->dma_size = 0;
	s->dma_offset_internal_read = 0;
	s->dma_offset_internal_write = 0;
	s->kernel_reg_write = 0;
	s->kernel_reg_read = 0;
	//	int minor = MINOR(inode->i_rdev);
	printk(KERN_INFO"<pci_open>: minor number %d detected\n", s->minor);

	/*set the private data field of the file pointer for file op use*/
	filep->private_data = s;   


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

	kfree((const void*)filep->private_data);

	return SUCCESS;
}


/*item for file_operations: unlocked ioctl*/
long pci_unlocked_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	u64 axi_dest;
//	u32 cdma_status;
//	u32 status;
	void __user *argp = (void __user *)arg;
//	u32 dma_addr_loc;
	struct mod_desc *mod_desc;
	static int master_count = 0;
	u32 bit_vec;
	u64 arg_loc; 
	u32 kern_reg;
	int int_num;
	int ret;
//	int * mode;

	copy_from_user(&arg_loc, argp, sizeof(u64));

	printk("<ioctl>Entering IOCTL with command: %d and arg %llx\n", cmd, arg_loc);

	mod_desc = filep->private_data;   

	switch(cmd){

		case SET_AXI_DEVICE:
			printk(KERN_INFO"<ioctl>: Setting Peripheral Axi Address:%llx\n", arg_loc);
			mod_desc->axi_addr = arg_loc;
			break;

		case SET_AXI_CTL_DEVICE:
			printk(KERN_INFO"<ioctl>: Setting Peripheral CTL AXI Address:%llx\n", arg_loc);
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
			printk(KERN_INFO"<ioctl>: Setting Peripheral DMA size:%llx\n", arg_loc);
			mod_desc->dma_size = arg_loc;
			if (((u64)dma_current_offset + arg_loc) > (u64)((char*)dma_buffer_base + dma_buffer_size))
			{
				printk(KERN_INFO"<ioctl>: ERROR! DMA Buffer out of memory!\n");
				return ERROR;
			}
			else
			{
				mod_desc->dma_size = arg_loc;
				printk(KERN_INFO"<set_dma_size>: The current system memory dma offset:%x\n", dma_current_offset);
				mod_desc->dma_offset_read = dma_current_offset;            //set the dma start address for the peripheral read
				mod_desc->dma_offset_write = dma_current_offset + (u32)arg_loc; //set the dma start address for the peripheral write
				mod_desc->dma_write = (void*)((char*)dma_buffer_base + (u64)dma_current_offset + arg_loc);            //actual pointer to kernel buffer
				printk(KERN_INFO"<ioctl>: DMA kernel write address set to:%lx\n", (u64)mod_desc->dma_write);
				mod_desc->dma_read = (void*)((char*)dma_buffer_base + (u64)dma_current_offset);            //actual pointer to kernel buffer
				
				dma_current_offset = dma_current_offset + (u32)(2*arg_loc);            //update the current dma allocation pointer, 2 buffers (R/W)
				printk(KERN_INFO"<ioctl>: Success setting peripheral DMA\n");
			}
				break;

                case RESET_DMA_ALLOC:
			dma_current_offset = 4096;   //we want to leave the first 4k for the kernel to use internally.
			current_dma_offset_internal = 0;
			break;
			
		case SET_INTERRUPT:
			printk(KERN_INFO"<ioctl>: Setting device as an Interrupt source with vector:%llx\n", arg_loc);
			/*Store the Interrupt Vector*/
			mod_desc->interrupt_vec = (u32)arg_loc;
			/*initialize the interrupt vector dictionary to 0*/
			//interrupt_vect_dict[arg_loc] = 0; 

			int_num = vec2num((u32)arg_loc);
			mod_desc->int_num = int_num;
			printk(KERN_INFO"<ioctl>: Interrupt Number:%d\n", int_num);

			interr_dict[int_num].int_count = mod_desc->int_count;	
			interr_dict[int_num].mode = mod_desc->mode;

			break;

		case SET_CDMA_KEYHOLE_WRITE:
			bit_vec = 0x00000020;   //the bit for KEYHOLE WRITE

			if(arg_loc>0)    //We are ENABLING keyhole write
			{
				printk(KERN_INFO"<ioctl_keyhole_write_set>: Setting the CDMA Keyhole WRITE as ENABLED\n");
				cdma_config_set(bit_vec, 1, 1);   //value of one means we want to SET the register
			}
			else    //We are disabling keyhole write
			{
				printk(KERN_INFO"<ioctl_keyhole_write_disable>: Setting the CDMA Keyhole WRITE as DISABLED\n");
				cdma_config_set(bit_vec, 0, 1);   //value of 0 means we want to UNSET the register
			}
			break;

		case SET_CDMA_KEYHOLE_READ:
			bit_vec = 0x00000010;   //the bit for KEYHOLE READ

			if(arg_loc>0)    //We are ENABLING keyhole write
			{
				printk(KERN_INFO"<ioctl_keyhole_read_set>: Setting the CDMA Keyhole READ as ENABLED\n");
				cdma_config_set(bit_vec, 1, 1);   //value of one means we want to SET the register
			}
			else    //We are disabling keyhole write
			{
				printk(KERN_INFO"<ioctl_keyhole_read_disable>: Setting the CDMA Keyhole READ as DISABLED\n");
				cdma_config_set(bit_vec, 0, 1);   //value of 0 means we want to UNSET the register
			}
			break;

		case SET_MODE:
			printk(KERN_INFO"<ioctl_set_mode>: Setting the mode of the peripheral\n");
			*(mod_desc->mode) = arg_loc;

			switch(arg_loc){

				case SLAVE:
					printk(KERN_INFO"<ioctl_set_mode>: Setting the mode of the peripheral to SLAVE\n");
					break;

				case MASTER:
					printk(KERN_INFO"<ioctl_set_mode>: Setting the mode of the peripheral to MASTER\n");

					/*DMA Allocation*/
					/*This sets the DMA read and write kernel buffers and obtains the
					 * dma_addr's to be sent to the CDMA core upon R/W transactions*/
					dma_master_buf[master_count] = pci_alloc_consistent(pci_dev_struct, 4096, &dma_m_addr[master_count]);
					if(NULL == dma_master_buf[master_count])
					{
						printk("%s:<probe>DMA AXI Master allocation ERROR\n", pci_devName);
						return ERROR;
					}

					/*write address to AXI to PCIe-M Translation Register*/

					/*write master_count as master_num to file descriptor to use in referencing its memory address*/
					mod_desc->master_num = master_count;
					break;

				case AXI_STREAM_FIFO:
					printk(KERN_INFO"<ioctl_set_mode>: Setting the mode of the peripheral to AXI_STREAM_FIFO\n");

					/*Initialize the AXI_STREAM_FIFO*/
					/*for now we will initialize all as interrupting for read*/
					//					/*check to see if the AXI addresses have been set*/
					if ((mod_desc->axi_addr == 0) | (mod_desc->axi_addr_ctl == 0))
					{
						printk(KERN_INFO"<ioctl_axi_stream_fifo>: ERROR: axi addresses of AXI STREAM FIFO not set\n");
						printk(KERN_INFO"<ioctl_axi_stream_fifo>:        set the AXI addresses then set mode again\n");
					}
					else
					{
						printk(KERN_INFO"<ioctl_axi_stream_fifo>: Initializing the FIFO and setting registers\n");
					
						/*allocate a small buffer of DMA for kernel to use*/
						mod_desc->dma_offset_internal_read = current_dma_offset_internal;
						mod_desc->dma_offset_internal_write = current_dma_offset_internal + 0x04;
						mod_desc->kernel_reg_read = (u32*)((u64)current_dma_offset_internal + (char*)dma_buffer_base);   //pointer to kernel read register
						mod_desc->kernel_reg_write = (u32*)((u64)(current_dma_offset_internal + 0x04) + (char*)dma_buffer_base); //pointer to kernel write register
						printk(KERN_INFO"<ioctl_axi_stream_fifo>: current dma kernel offset:%x\n", current_dma_offset_internal);
						printk(KERN_INFO"<ioctl_axi_stream_fifo>: kernel R/W register kernel addresses:%lx / %lx\n", (u64)mod_desc->kernel_reg_read, (u64)mod_desc->kernel_reg_write );
						current_dma_offset_internal = current_dma_offset_internal + 0x08;   // update the current dma buffer status (just added two 32b buffers)
						//Read CTL interface
						axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_ISR;
				//		ret = data_transfer(axi_dest, (void *)&kern_reg, 4, NORMAL_READ);
						ret = data_transfer(axi_dest, 0, 4, NORMAL_READ, mod_desc->dma_offset_internal_read);
						printk(KERN_INFO"<axi_fifo_isr_reg>:%x\n", *(mod_desc->kernel_reg_read));

						//reset interrupts on CTL interface
				//		kern_reg = 0xFFFFFFFF;
						axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_ISR;
						*(mod_desc->kernel_reg_write) = 0xFFFFFFFF;
						ret = data_transfer(axi_dest, 0, 4, NORMAL_WRITE, mod_desc->dma_offset_internal_write);
						printk(KERN_INFO"<axi_fifo_isr_reg>: Reset the interrupts on the axi fifo\n");

						//Read CTL interface
						axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_ISR;
				//		ret = data_transfer(axi_dest, (void *)&kern_reg, 4, NORMAL_READ);
						ret = data_transfer(axi_dest, 0, 4, NORMAL_READ, mod_desc->dma_offset_internal_read);
						printk(KERN_INFO"<axi_fifo_isr_reg>:%x\n", *(mod_desc->kernel_reg_read));
					
							/*Set IER Register for interrupt on read*/
						axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_IER;
						*(mod_desc->kernel_reg_write) = 0x04000000;
					//	kern_reg = 0x04000000;
					//	ret = data_transfer(axi_dest, (void *)&kern_reg, 4, NORMAL_WRITE);
						ret = data_transfer(axi_dest, 0, 4, NORMAL_WRITE, mod_desc->dma_offset_internal_write);

					}

					break;

				default:printk(KERN_INFO"<ioctl_set_mode> ERROR, improper mode type!\n");
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
	printk(KERN_INFO"<pci_read>: attempted seek\n");
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
	printk(KERN_INFO"<pci_poll>: Entered Poll()\n");
	mod_desc = filep->private_data;   

	/*Register the poll table with the peripheral wait queue
	 *so that every wake_up on the wait queue will see if the 
	 *peripheral has data*/
	poll_wait(filep, &wq_periph, pwait);

	/*see if the module has triggered an interrupt*/
	has_data = *(mod_desc->int_count);
	//	has_data = interrupt_vect_dict[mod_desc->interrupt_vec];
	//	if (has_data == 1)
	if (has_data > 0)
	{
		printk(KERN_INFO"<pci_poll>: wait event detected!!\n");
		/*reset the has_data flag*/
	//	*(mod_desc->int_count) = (*(mod_desc->int_count)) - 1;
		*(mod_desc->int_count) = 0;
		mask |= POLLIN;
	}

	return mask;


}

ssize_t pci_write(struct file *filep, const char __user *buf, size_t count, loff_t *f_pos)
{
	void * kern_buf = NULL;
	u64 axi_dest;
	struct mod_desc * mod_desc;
	size_t bytes;
//	int cdma_capable;
	int keyhole_en;
	int transfer_type;
	u32 kern_reg;
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

	/*Allocate a Buffer in kernel space*/
//	kern_buf = kmalloc(count, GFP_KERNEL);
//	if (!kern_buf)
//	{
//		printk(KERN_INFO"<pci_write>: ERROR Initializing kmalloc\n");
//		return 0;
//	}
	/*Transfer the write data to the kernel space buffer*/
//	copy_from_user(kern_buf, buf, count);

	/*eventually this will go away once we add mmap
	 * for now we copy to the appropriate file buffer*/
	copy_from_user(mod_desc->dma_write, buf, count);

	printk(KERN_INFO"\n\n");	
	printk(KERN_INFO"<pci_write>: ************************************************************************\n");	
	printk(KERN_INFO"<pci_write>: ******************** WRITE TRANSACTION BEGIN  **************************\n");	
	printk(KERN_INFO"<pci_write>: ************************************************************************\n\n");	

	init_write = *((u32*)(mod_desc->dma_write));
	dma_offset_write = (u64)mod_desc->dma_offset_write;
	dma_offset_internal_read = (u64)mod_desc->dma_offset_internal_read;
	dma_offset_internal_write = (u64)mod_desc->dma_offset_internal_write;

	printk(KERN_INFO"<pci_write>: writing peripheral with starting value: %x\n", init_write);
	printk(KERN_INFO"<pci_write>: DMA offset write value: %lx\n", dma_offset_write);
	printk(KERN_INFO"<pci_write>: DMA internal offset write value: %lx\n", dma_offset_internal_write);
	printk(KERN_INFO"<pci_write>: DMA internal offset read value: %lx\n", dma_offset_internal_read);

	if (*(mod_desc->mode) == AXI_STREAM_FIFO)
	{

		printk(KERN_INFO"<axi_stream_fifo_write>: writing to the AXI Stream FIFO\n");
		/*This is the function to move data from user space to kernel space*/
		//		copy_from_user(zero_copy_buf, buf, count);

	
		axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_TDFV;
//		ret = data_transfer(axi_dest, (void *)&kern_reg, 4, NORMAL_READ);
//		printk(KERN_INFO"<pci_write>: Transmit Data FIFO Fill Level:%x\n", kern_reg);
		*(mod_desc->kernel_reg_read) = 0x0;   //set to zero
		while (*(mod_desc->kernel_reg_read) != 0x01fc)  //1fe is an empty 512 depth fifo
		{
			ret = data_transfer(axi_dest, 0, 4, NORMAL_READ, dma_offset_internal_read);
			if (ret > 0)
			{
				printk(KERN_INFO"<pci_write>: ERROR reading from AXI Streaming FIFO control interface\n");
	//			kfree((const void*)kern_buf);
				return 0;
			}
			printk(KERN_INFO"										<pci_write>: Transmit Data FIFO Fill Level:%x\n", *(mod_desc->kernel_reg_read));
			if (*(mod_desc->kernel_reg_read) != 0x01fc)
				msleep(1);
		}
	
		/*Set keyhole*/
		keyhole_en = KEYHOLE_WRITE;

		/*read TDFD*/
		axi_dest = mod_desc->axi_addr + AXI_STREAM_TDFD;
//		ret = data_transfer(axi_dest, kern_buf, count, keyhole_en);
		ret = data_transfer(axi_dest, 0, count, KEYHOLE_WRITE, dma_offset_write);
		if (ret > 0)
		{
			printk(KERN_INFO"<pci_write>: ERROR writing to AXI Streaming FIFO\n");
	//		kfree((const void*)kern_buf);
			return 0;
		}

		/*write to ctl interface*/
		axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_TLR;

//		kern_reg = (u32)count;
//		ret = data_transfer(axi_dest, (void*)&kern_reg, 4, NORMAL_WRITE);
		*(mod_desc->kernel_reg_write) = (u32)count;
		ret = data_transfer(axi_dest, 0, 4, NORMAL_WRITE, dma_offset_internal_write);
		if (ret > 0)
		{
			printk(KERN_INFO"<pci_write>: ERROR writing to AXI Streaming FIFO Control Interface\n");
//			kfree((const void*)kern_buf);
			return 0;
		}
		
		bytes = count;
	}
	else
	{
		/* Here we will decide whether to do a zero copy DMA, or to write */
		/* directly to the peripheral */
		axi_dest = mod_desc->axi_addr + filep->f_pos;

		if (mod_desc->keyhole_config & 0x1)
			transfer_type = KEYHOLE_WRITE;
		else
			transfer_type = NORMAL_WRITE;

		printk(KERN_INFO"<pci_write>: writing peripheral using a transfer_type: %x\n", transfer_type);
	
	//	ret = data_transfer(axi_dest, kern_buf, count, transfer_type);
		ret = data_transfer(axi_dest, 0, count, transfer_type, dma_offset_write);
		if (ret > 0)
		{
			printk(KERN_INFO"<pci_write>: ERROR writing to User Peripheral\n");
//			kfree((const void*)kern_buf);
			return 0;
		}

		bytes = count;
	}

//	kfree((const void*)kern_buf);
	
	printk(KERN_INFO"\n");	
	printk(KERN_INFO"<pci_write>: ************************************************************************\n");	
	printk(KERN_INFO"<pci_write>: ******************** WRITE TRANSACTION END  **************************\n");	
	printk(KERN_INFO"<pci_write>: ************************************************************************\n");	
	printk(KERN_INFO"\n");	
	return bytes;

}

ssize_t pci_read(struct file *filep, char __user *buf, size_t count, loff_t *f_pos)
{
	u32 kern_reg;
	u64 axi_dest;
	struct mod_desc *mod_desc;
	int bytes;
//	int cdma_capable;
	int keyhole_en;
	int transfer_type;
	void * read_buf = NULL;
	int ret;
	u32 read_bytes;
	u64 dma_offset_write;
	u64 dma_offset_read;
	u64 dma_offset_internal_read;
	u64 dma_offset_internal_write;

	mod_desc = filep->private_data;

	printk(KERN_INFO"\n\n");	
	printk(KERN_INFO"<pci_read>: ************************************************************************\n");	
	printk(KERN_INFO"<pci_read>: ******************** READ TRANSACTION BEGIN  **************************\n");	
	printk(KERN_INFO"<pci_read>: ************************************************************************\n");	

//	read_buf = kmalloc(count, GFP_KERNEL);
//	if (!read_buf)
//	{
//		printk(KERN_INFO"<pci_read>: ERROR Initializing kmalloc\n");
//		return 0;
//	}

	dma_offset_write = (u64)mod_desc->dma_offset_write;
	dma_offset_read = (u64)mod_desc->dma_offset_read;
	dma_offset_internal_read = (u64)mod_desc->dma_offset_internal_read;
	dma_offset_internal_write = (u64)mod_desc->dma_offset_internal_write;

	switch(*(mod_desc->mode)){

		case MASTER:
			//Transfer buffer from kernel space to user space at the allocated DMA region
			printk(KERN_INFO"<pci_read>: Transferred a Master write from kernel space to user space\n");
			copy_to_user(buf, dma_master_buf[mod_desc->master_num], count);
			break;
			//			return count;

		case AXI_STREAM_FIFO:

			/*debug stuff*/
			//Read CTL interface
			printk(KERN_INFO"<axi_stream_fifo_read>: Reading the AXI Stream FIFO\n");
			axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_ISR;
//			data_transfer(axi_dest, (void *)&kern_reg, 4, NORMAL_READ);
//			printk(KERN_INFO"<axi_fifo_isr_reg>:%x\n", kern_reg);

			//reset interrupts on CTL interface
	//		kern_reg = 0xFFFFFFFF;
	//		ret = data_transfer(axi_dest, (void *)&kern_reg, 4, NORMAL_WRITE);
			*(mod_desc->kernel_reg_write) = 0xFFFFFFFF;
			ret = data_transfer(axi_dest, 0, 4, NORMAL_WRITE, dma_offset_internal_write);
			if (ret > 0)
			{
				printk(KERN_INFO"<axi_stream_fifo_read>: ERROR writing to reset interrupts on AXI Stream FIFO\n");
			//	kfree((const void*)read_buf);
				return 0;
			}

			/*ISR Read for Debug*/
//			data_transfer(axi_dest, (void *)&kern_reg, 4, NORMAL_READ);
//			printk(KERN_INFO"<axi_fifo_isr_reg>:%x\n", kern_reg);

			/*Read FIFO Fill level*/
			axi_dest = mod_desc->axi_addr_ctl + AXI_STREAM_RLR;
		//	ret = data_transfer(axi_dest, (void *)&kern_reg, 4, NORMAL_READ);
			ret = data_transfer(axi_dest, 0, 4, NORMAL_READ, dma_offset_internal_read);
			if (ret > 0)
			{
				printk(KERN_INFO"<axi_stream_fifo_read>: ERROR reading Read FIFO fill level\n");
			//	kfree((const void*)read_buf);
				return 0;
			}

			/*we are masking off the 32nd bit because the FIFO is in cut through mode
			 *and sets the LSB to 1 to indicate a partial packet.*/
	 	        // read_bytes = (0x7fff & kern_reg);
	 	         read_bytes = (0x7fff & *(mod_desc->kernel_reg_read));
		
			printk(KERN_INFO"<axi_stream_fifo_read> Read FIFO fill level:0x%x bytes\n", read_bytes);

			/*So we don't read more data than is available*/
			if (read_bytes < (u32)count)
				count = read_bytes;

			/*Check to make sure we are doing an 8 byte aligned read*/
			if ((count % 8) > 0)
			{
		
				count = count - (count % 8);			
				printk(KERN_INFO"<axi_stream_fifo_read> Read value changed to: 0x%x for alignment.\n", count);
			}

			if (count == 0)
			{	
				printk(KERN_INFO"<axi_stream_fifo_read> There is either no data to read, or less than 8 bytes.\n");
				return 0;
			}

			//set CDMA KEYHOLE
			keyhole_en = KEYHOLE_READ;

			axi_dest = mod_desc->axi_addr + AXI_STREAM_RDFD;
		
			printk(KERN_INFO"<axi_stream_fifo_read> AXI Address of FIFO:%lx\n", mod_desc->axi_addr);
		
	//		ret = data_transfer(axi_dest, read_buf, count, keyhole_en);
			ret = data_transfer(axi_dest, 0, count, keyhole_en, dma_offset_read);
			if (ret > 0)
			{
				printk(KERN_INFO"<axi_stream_fifo_read>: ERROR reading Data from Read FIFO\n");
	//			kfree((const void*)read_buf);
				return 0;
			}

			//Transfer buffer from kernel space to user space at the allocated DMA region
			//				copy_to_user(buf, zero_copy_buf, count);

			bytes = count;
			printk(KERN_INFO"<axi_stream_fifo_read>: Leaving the READ AXI Stream FIFO routine\n");
			break;

		case SLAVE:
			/* Here we will decide whether to do a zero copy DMA, or to read */
			/* directly from the peripheral */

			axi_dest = mod_desc->axi_addr + filep->f_pos;

			if (mod_desc->keyhole_config & 0x1)
				transfer_type = KEYHOLE_READ;
			else
				transfer_type = NORMAL_READ;

			//				read_buf = kmalloc(count, GFP_KERNEL);

			printk(KERN_INFO"<user_peripheral_read>: reading peripheral using a transfer_type: %x\n", transfer_type);

	//		ret = data_transfer(axi_dest, read_buf, count, transfer_type);
			ret = data_transfer(axi_dest, 0, count, transfer_type, dma_offset_read);
			if (ret > 0)
			{
				printk(KERN_INFO"<user_peripheral_read>: ERROR reading data from User Peripheral\n");
			//	kfree((const void*)read_buf);
				return 0;
			}

			//				copy_to_user(buf, read_buf, count);

			//				kfree((const void*)read_buf);

			//				return count;

			bytes = count;


			break;

		default:printk(KERN_INFO"<pci_read>: mode not detected on read\n");
	}

//	copy_to_user(buf, read_buf, count);

	/*eventually this will go away once we add mmap
	 * for now we copy to the appropriate file buffer*/
	copy_to_user(buf, mod_desc->dma_read, count);

//	kfree((const void*)read_buf);

	printk(KERN_INFO"\n");	
	printk(KERN_INFO"<pci_read>: ************************************************************************\n");	
	printk(KERN_INFO"<pci_read>: ******************** READ TRANSACTION END  **************************\n");	
	printk(KERN_INFO"<pci_read>: ************************************************************************\n");	
	printk(KERN_INFO"\n");	

	return bytes;
}
/******************************** Support functions ***************************************/
void int_ctlr_init(u64 axi_address)
{
	u32 status;
	u64 axi_dest;
	int ret;

	printk(KERN_INFO"<int_ctlr_init>: Setting Interrupt Controller Axi Address\n");
	axi_interr_ctrl = axi_address;
	int_ctrl_set = 1;

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
	printk(KERN_INFO"<int_ctlr_init>: read: ('%x') from MER register\n", status);

	/*Here we need to clear the service interrupt in the interrupt acknowledge register*/ 
	status = 0xFFFFFFFF;
	axi_dest = axi_interr_ctrl + INT_CTRL_IAR;
	ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_WRITE, 0);
}

void pcie_m_init(cdma_num)
{
	printk(KERN_INFO"<pcie_m_init>: Setting PCIe Master Axi Address\n");
	switch(cdma_num)
	{
		case 1:
			axi_pcie_m[1] = pcie_m_address;
			break;
		case 2:
			axi_pcie_m[2] = pcie_m_address_2;
			break;
		default:printk(KERN_INFO"<pcie_m_init>:	!!!!!!!!ERROR: incorrect CDMA number detected!!!!!!!\n");
	}
	pcie_m_set[cdma_num] = 1;
}

void pcie_ctl_init(u64 axi_address)
{
	u32 dma_addr_loc;
	u64 axi_dest;
	u32 status;
	int ret;

	printk(KERN_INFO"<pcie_ctl_init>: Setting PCIe Control Axi Address\n");
	axi_pcie_ctl = axi_address;
	pcie_ctl_set = 1;

	if(cdma_set[1] == 1)
	{
		/*convert to u32 to send to CDMA*/
	//	dma_addr_loc = (u32)dma_addr[1];
		dma_addr_loc = (u32)dma_addr_base;

		//update pcie_ctl_virt address with register offset
		axi_dest = axi_pcie_ctl + AXIBAR2PCIEBAR_0L;

		//write DMA addr to PCIe CTL for address translation
		ret = data_transfer(axi_dest, (void *)(&dma_addr_loc), 4, NORMAL_WRITE, 0);
		printk(KERN_INFO"<pci_ioctl_cdma_set>: writing dma address ('%x') to pcie_ctl at AXI address:%llx\n", dma_addr_loc, axi_dest);

		//check the pcie-ctl got the translation address
		ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_READ, 0);
		printk(KERN_INFO"<pci_ioctl_cdma_set>: PCIe CTL register:%x\n", status);

		if (status == dma_addr_loc)
			printk(KERN_INFO"<pci_ioctl_cdma_set>: PCIe CTL register set SUCCESS:%x\n", status);
		else
			printk(KERN_INFO"<pci_ioctl_cdma_set>:!!!!!!!!!!! PCIe CTL register FAILURE !!!!!!!!!!!!!!!:%x\n", status);
	}

	if(cdma_set[2] == 1)
	{
		/*convert to u32 to send to CDMA*/
	//	dma_addr_loc = (u32)dma_addr[2];
		dma_addr_loc = (u32)dma_addr_base;

		//update pcie_ctl_virt address with register offset
		axi_dest = axi_pcie_ctl + AXIBAR2PCIEBAR_1L;

		//write DMA addr to PCIe CTL for address translation
		ret = data_transfer(axi_dest, (void *)(&dma_addr_loc), 4, NORMAL_WRITE, 0);
		printk(KERN_INFO"<pci_ioctl_cdma_set>: writing dma address ('%x') to pcie_ctl at AXI address:%llx\n", dma_addr_loc, axi_dest);

		//check the pcie-ctl got the translation address
		ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_READ, 0);
		printk(KERN_INFO"<pci_ioctl_cdma_set>: PCIe CTL register:%x\n", status);

		if (status == dma_addr_loc)
			printk(KERN_INFO"<pci_ioctl_cdma_set>: PCIe CTL register set SUCCESS:%x\n", status);
		else
			printk(KERN_INFO"<pci_ioctl_cdma_set>:!!!!!!!!!!! PCIe CTL register FAILURE !!!!!!!!!!!!!!!:%x\n", status);
	}
}

void cdma_init(int cdma_num)
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
			axi_cdma = cdma_address;
			axi_cdma_loc = axi_cdma;
			mutex_init(&CDMA_sem);
			break;
		case 2:
			axi_cdma_2 = cdma_address_2;
			axi_cdma_loc = axi_cdma_2;
			mutex_init(&CDMA_sem_2);
			break;
		default:printk(KERN_INFO"	!!!!!!!!ERROR: incorrect CDMA number detected!!!!!!!\n");
	}
	
	printk(KERN_INFO"<cdma_%x_init>: *******************Setting CDMA AXI Address:%llx ******************************************\n", cdma_num, axi_cdma_loc);
	cdma_set[cdma_num] = 1;

	/*Initialize the semaphore for this CDMA*/
//	mutex_init(&CDMA_sem);

	/*Issue a Soft Reset*/
	axi_dest = axi_cdma_loc + CDMA_CR;
	cdma_status = 0x00000004;
	printk(KERN_INFO"<cdma_%x_init>: sending a soft reset to the CDMA\n", cdma_num);
	ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_WRITE, 0);
	/*Remove Soft Reset*/
//	axi_dest = axi_cdma + CDMA_CR;
//	cdma_status = 0x00000000;
//	data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_WRITE);
	/*Check the current status*/
	axi_dest = axi_cdma_loc + CDMA_SR;
	//			direct_read(axi_dest, (void*)&cdma_status, 4, NORMAL_READ);
	ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_READ, 0);
	printk(KERN_INFO"<cdma_%x_init>: CDMA status before configuring:%x\n", cdma_num, cdma_status);	
	/*Check the current config*/
	axi_dest = axi_cdma_loc + CDMA_CR;
	//			direct_read(axi_dest, (void*)&cdma_status, 4, NORMAL_READ);
	ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_READ, 0);
	printk(KERN_INFO"<cdma_%x_init>: CDMA config before configuring:%x\n", cdma_num, cdma_status);	
	/*clear any pre existing interrupt*/
	axi_dest = axi_cdma_loc + CDMA_SR;
	cdma_status = 0x00001000;
	printk(KERN_INFO"<cdma_%x_init>: attempting to write:%x to cdma status reg\n", cdma_num, cdma_status);
	ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_WRITE, 0);

	cdma_status = 0x00001000;
	axi_dest = axi_cdma_loc + CDMA_CR;
	ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_WRITE, 0);

	/*Check the current status*/
	axi_dest = axi_cdma_loc + CDMA_SR;
	ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_READ, 0);
	printk(KERN_INFO"<cdma_%x_init>: CDMA status after configuring:%x\n", cdma_num, cdma_status);	
	/*Check the current config*/
	axi_dest = axi_cdma_loc + CDMA_CR;
	ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_READ, 0);
	printk(KERN_INFO"<cdma_%x_init>: CDMA config after configuring:%x\n", cdma_num, cdma_status);	

	/*allocate the DMA buffer*/

	if (zero_buf_set[cdma_num] == 0)
	{
		/*DMA Allocation*/
		/*This sets the DMA read and write kernel buffers and obtains the
		 * dma_addr's to be sent to the CDMA core upon R/W transactions*/

		zero_copy_buf[cdma_num] = dma_alloc_coherent(dev_struct, 131072, &dma_addr[cdma_num], GFP_KERNEL);
		if(NULL == zero_copy_buf[cdma_num])
		{
			printk("%s:<cdma_%x_init>DMA zero copy buff allocation ERROR\n", pci_devName, cdma_num);
			zero_buf_set[cdma_num] = 0;
		}
		else
		{
			zero_buf_set[cdma_num] = 1;
			printk(KERN_INFO"<cdma_%x_init>: dma address is:%llx\n", cdma_num, (u64)dma_addr[cdma_num]);
		}

	}

	/*This checks to see if the user has set the pcie_ctl base address
	 * if so, we can go ahead and write the dma_addr to the pcie translation
	 * register.*/
	if(pcie_ctl_set == 1)
	{
		/*convert to u32 to send to CDMA*/
	//	dma_addr_loc = (u32)dma_addr[cdma_num];
		dma_addr_loc = (u32)dma_addr_base;
		//update pcie_ctl_virt address with register offset
		switch(cdma_num)
		{
			case 1:
				axi_dest = axi_pcie_ctl + AXIBAR2PCIEBAR_0L;
				break;
			case 2:
				axi_dest = axi_pcie_ctl + AXIBAR2PCIEBAR_1L;
				break;
			default:printk(KERN_INFO"	!!!!!!!!ERROR: incorrect CDMA number detected!!!!!!!\n");
				return(0);
		}

		//write DMA addr to PCIe CTL for address translation
		ret = data_transfer(axi_dest, (void *)(&dma_addr_loc), 4, NORMAL_WRITE, 0);
		printk(KERN_INFO"<cdma_init>: writing dma address ('%x') to pcie_ctl at AXI address:%llx\n", dma_addr_loc, axi_dest);
		//check the pcie-ctl got the translation address
		ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_READ, 0);
		printk(KERN_INFO"<cdma_init>: PCIe CTL register:%x\n", cdma_status);

	}

	/*update the interr_dict with interrupt number and mode*/
	mode = (int*)kmalloc(sizeof(int), GFP_KERNEL);
	*mode = CDMA;
	index = cdma_num - 1;
	interr_dict[index].mode = mode;
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

/*Data Transfer*/
/*This function should be used to funnel all data traffic between the
 *Kernel and the FPGA. It is used to determine if the CDMA(s) should
 *be and can be used. It is also used to call the functions for direct
 *reads and writes if the CDMA is either unavailable or inefficient to
 *be used*/

int data_transfer(u64 axi_address, void *buf, size_t count, int transfer_type, u64 dma_off)
{
	//	int cdma_capable;
	u32 test;
	int in_range = 0;
	int status;
	int cdma_num = 0;
	u64 dma_axi_address = 0;
	void * dma_p;

	//	cdma_capable = (cdma_set == 1) & (pcie_ctl_set == 1) & (pcie_m_set == 1);

	/*determine if the axi range is in direct accessible memory space*/
	if ((axi_address + count) < (bar_0_axi_offset + pci_bar_size))
		in_range = 1;
	else if ((axi_address + count) < (peripheral_space_offset + pci_bar_1_size))
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
			printk(KERN_INFO"<data_transfer>: error no transfer type specified\n");
	}

	else if (cdma_capable == 1)
	{
		/* Find an available CDMA to use and wait if both are in use */
	//	while(cdma_num == 0)
	//	{
			cdma_num = cdma_query();
			if (cdma_num == 0)
			{
				/*default to CDMA 1 and wait on it*/
				if (mutex_lock_interruptible(&CDMA_sem))
				{
					printk(KERN_INFO"User interrupted while waiting for CDMA semaphore.\n");
					return -ERESTARTSYS;
				}
				cdma_num = 1;
		//		atomic_set(&mutex_free, 0);  //wait variable for mutex lock
		//		wait_event_interruptible(mutexq, atomic_read(&mutex_free) != 0);
			}
	//	}

		/*set the dma_axi_address to be for whichever cdma has been locked*/
		/*the dma_axi_address is the axi address where the HP port is for zynq
		 * or for where the PCIe AXI Slave port is for x86 */ 
	//	dma_axi_address = axi_pcie_m[cdma_num];
	//	dma_p = zero_copy_buf[cdma_num];

		dma_axi_address = axi_pcie_m[cdma_num] + dma_off;  //the AXI address written to the CDMA
	
		if ((transfer_type == NORMAL_READ) | (transfer_type == KEYHOLE_READ))
		{
			status = cdma_transfer(axi_address, dma_axi_address, (u32)count, transfer_type, cdma_num);
			if (status != 0)   //unsuccessful CDMA transmission
				printk(KERN_INFO"ERROR on CDMA READ!!!.\n");
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
			printk(KERN_INFO"<data_transfer>: error no transfer type specified\n");

		/*Release Mutex on CDMA*/
		switch(cdma_num)
		{
			case 1:
				printk(KERN_INFO"												<data_transfer>: Releasing Mutex on CDMA 1\n");
				mutex_unlock(&CDMA_sem);
				break;

			case 2:
				printk(KERN_INFO"												<data_transfer>: Releasing Mutex on CDMA 2\n");
				mutex_unlock(&CDMA_sem_2);
				break;
			default : printk(KERN_INFO"<data_transfer>: ERROR: unknown cdma number detected.\n");
				  return(0);
		}

	//	atomic_set(&mutex_free, 1);  //wait variable for mutex lock
	//	wake_up_interruptible(&mutexq);

	}

	else
		printk(KERN_INFO"<data_transfer>: ERROR: Address is out of range and CDMA is not initialized\n");

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
		printk(KERN_INFO"		<direct_write>: Direct writing to BAR 1\n");
		printk(KERN_INFO"		<direct_write>: Direct writing to BAR 1 with axi address:%llx\n", axi_address);
		virt_addr = (axi_address - peripheral_space_offset) + pci_bar_1_vir_addr;
	}
	else if ((axi_address + count) < (bar_0_axi_offset + pci_bar_size))
	{
		virt_addr = (axi_address - bar_0_axi_offset) + pci_bar_vir_addr;
		printk(KERN_INFO"		<direct_write>: Direct writing to BAR 0\n");
	}
	else
	{
		printk(KERN_INFO"		<direct_write>: ERROR trying to Direct write out of memory range!\n");
		return 1;
	}

	printk(KERN_INFO"		<direct_write>: writing:%x \n", *(u32*)buf);

	offset = 0;
	len = count/4;
	for(i = 0; i<len; i++)
	{
		newaddr_src = (u32 *)(kern_buf + offset);
		write_value = *newaddr_src;
		newaddr_dest = (u32 *)(virt_addr + offset);
		iowrite32(write_value, newaddr_dest);
		//		*newaddr_dest = write_value;   *this works too*
		printk(KERN_INFO"		<direct_write>: wrote:%x to virtual address:('%p')\n", write_value, newaddr_dest);
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
		printk(KERN_INFO"		<direct_read>: Direct reading from BAR 1 with axi address:%llx\n", axi_address);
		virt_addr = (axi_address - peripheral_space_offset) + pci_bar_1_vir_addr;
		printk(KERN_INFO"		<direct_read>: Direct reading from virtual address:%p\n", virt_addr);
	}
	else if ((axi_address + count) < (bar_0_axi_offset + pci_bar_size))
	{
		printk(KERN_INFO"		<direct_read>: Direct reading from BAR 0\n");
		virt_addr = (axi_address - bar_0_axi_offset) + pci_bar_vir_addr;
	}
	else
	{
		printk(KERN_INFO"		<direct_read>: ERROR trying to Direct read out of memory range!\n");
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
		printk(KERN_INFO"		<direct_read>: read: %x from kernel address %p.\n", kern_buf[i], newaddr_src);
	}

	memcpy(buf, (const void*)kern_buf, count);

	return 0;
}

int cdma_query(void)
{
	/*Check the CDMA Semaphore*/

	if (mutex_is_locked(&CDMA_sem))
	{
		if (mutex_is_locked(&CDMA_sem_2) | !cdma_set[2])
			printk(KERN_INFO "!!!!!!!!! all CDMAs in use !!!!!!!!\n");
		else
		{
			if (mutex_lock_interruptible(&CDMA_sem_2))
			{
				printk(KERN_INFO"User interrupted while waiting for CDMA semaphore.\n");
				return -ERESTARTSYS;
			}
			printk(KERN_INFO"										<cdma_transfer>: CDMA Resource 2 is now locked!\n");
			return 2;
		}	
	}
	else
	{	
		if (mutex_lock_interruptible(&CDMA_sem))
		{
			printk(KERN_INFO"User interrupted while waiting for CDMA semaphore.\n");
			return -ERESTARTSYS;
		}

		printk(KERN_INFO"											<cdma_transfer>: CDMA Resource 1 is now locked!\n");
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

	printk(KERN_INFO"	<cdma_transfer>: **** Starting a CDMA transfer ****\n");

	/*Check the CDMA Semaphore*/
//	if (mutex_lock_interruptible(&CDMA_sem))
//	{
//		printk(KERN_INFO"User interrupted while waiting for CDMA semaphore.\n");
//		return -ERESTARTSYS;
//	}

//	printk(KERN_INFO"	<cdma_transfer>: CDMA Resource is now locked!\n");

	SA_MSB = (u32)(SA>>32);
	SA_LSB = (u32)SA;

	DA_MSB = (u32)(DA>>32);
	DA_LSB = (u32)DA;
	
	switch(keyhole_en){

		case KEYHOLE_READ:
			bit_vec = 0x00000010;   //the bit for KEYHOLE READ		
			printk(KERN_INFO"	<keyhole_read_set>: Setting the CDMA Keyhole READ as ENABLED\n");
			cdma_config_set(bit_vec, 1, cdma_num);   //value of one means we want to SET the register
			break;

		case KEYHOLE_WRITE:
			bit_vec = 0x00000020;   //the bit for KEYHOLE WRITE
			printk(KERN_INFO"	<keyhole_write_set>: Setting the CDMA Keyhole WRITE as ENABLED\n");
			cdma_config_set(bit_vec, 1, cdma_num);   //value of one means we want to SET the register
			break;

		default:printk(KERN_INFO"	<keyhole_setting> no keyhole swtting will be used.\n");
	}

	switch(cdma_num)
	{
		case 1:
			axi_cdma_loc = axi_cdma;
			printk(KERN_INFO"	<pci_dma_transfer>:Using CDMA 1\n");	
			break;
		case 2:
			axi_cdma_loc = axi_cdma_2;
			printk(KERN_INFO"	<pci_dma_transfer>:Using CDMA 2\n");	
			break;
		default:printk(KERN_INFO"	!!!!!!!!ERROR: incorrect CDMA number detected!!!!!!!\n");
			return(0);
	}

	//read the config register
	axi_dest = axi_cdma_loc + CDMA_CR;
	direct_read(axi_dest, (void*)&status, 4, NORMAL_READ);
	printk(KERN_INFO"	<pci_dma_transfer>: CDMA Configuration before transmission:%x\n", status);	
	//	if ((status & 0x4) == 0x4)
	//	{
	//	printk(KERN_INFO"<pci_dma_transfer>: CDMA is still in reset, now waiting......\n");	
	//	while ((status & 0x4) == 0x4)
	//	{
	//		msleep(500);	
	//		direct_read(axi_dest, (void*)&status, 4, NORMAL_READ);
	//	}
	//	printk(KERN_INFO"<pci_dma_transfer>: CDMA is now out of reset, resuming......\n");	
	//	}

	//Writing SA_MSB	
//	axi_dest = axi_cdma + CDMA_SA_MSB;
//	direct_write(axi_dest, (void*)&SA_MSB, 4, NORMAL_WRITE);
	printk(KERN_INFO"	<pci_dma_transfer>: ********* CDMA TRANSFER INITIALIZATION *************\n\n");	
//	printk(KERN_INFO"	<pci_dma_transfer>: writing dma SA_MSB address ('%x') to CDMA at axi address:%llx\n", SA_MSB, axi_dest);
	//Writing SA_LSB	
	axi_dest = axi_cdma_loc + CDMA_SA;
	direct_write(axi_dest, (void*)&SA_LSB, 4, NORMAL_WRITE);
	printk(KERN_INFO"	<pci_dma_transfer>: writing dma SA_LSB address ('%x') to CDMA at axi address:%llx\n", SA_LSB, axi_dest);
	//read the status register
	axi_dest = axi_cdma_loc + CDMA_SR;
	direct_read(axi_dest, (void*)&status, 4, NORMAL_READ);
	printk(KERN_INFO"<pci_dma_transfer>: CDMA Status after writing SA:%x\n", status);	

	//Writing DA_MSB	
//	axi_dest = axi_cdma + CDMA_DA_MSB;
//	direct_write(axi_dest, (void*)&DA_MSB, 4, NORMAL_WRITE);
//	printk(KERN_INFO"	<pci_dma_transfer>: writing DA_MSB address ('%x') to CDMA at axi address:%llx\n", DA_MSB, axi_dest);
	//Writing DA_LSB
	axi_dest = axi_cdma_loc + CDMA_DA;
	direct_write(axi_dest, (void*)&DA_LSB, 4, NORMAL_WRITE);
	printk(KERN_INFO"	<pci_dma_transfer>: writing DA_LSB address ('%x') to CDMA at axi address:%llx\n", DA_LSB, axi_dest);
	//read the status register
	axi_dest = axi_cdma_loc + CDMA_SR;
	direct_read(axi_dest, (void*)&status, 4, NORMAL_READ);
	printk(KERN_INFO"<pci_dma_transfer>: CDMA Status after writing DA:%x\n", status);	

	//readback the SA
//	axi_dest = axi_cdma + CDMA_SA;
//	direct_read(axi_dest, (void*)&status, 4, NORMAL_READ);
//	printk(KERN_INFO"<pci_dma_transfer>: CDMA SA readback:%x\n", status);	
	//readback the DA
//	axi_dest = axi_cdma + CDMA_DA;
//	direct_read(axi_dest, (void*)&status, 4, NORMAL_READ);
//	printk(KERN_INFO"<pci_dma_transfer>: CDMA DA readback:%x\n", status);	

	//Writing BTT	
	axi_dest = axi_cdma_loc + CDMA_BTT;
	printk(KERN_INFO"	<pci_dma_transfer>: writing bytes to transfer ('%d') to CDMA at axi address:%llx\n", BTT, axi_dest);	
	direct_write(axi_dest, (void*)&BTT, 4, NORMAL_WRITE);

	printk(KERN_INFO"	<pci_dma_transfer>: ********* CDMA TRANSFER INITIALIZED *************\n");	
	//readback the BTT
	//	direct_read(axi_dest, (void*)&status, 4, NORMAL_READ);
	//	printk(KERN_INFO"<pci_dma_transfer>: CDMA BTT readback:%x\n", status);	
	//read the status register
	//	axi_dest = axi_cdma + CDMA_SR;
	//	direct_read(axi_dest, (void*)&status, 4, NORMAL_READ);
	//	printk(KERN_INFO"<pci_dma_transfer>: CDMA Status after writing BTT::%x\n", status);	

	/*Go to sleep and wait for interrupt*/
	wait_event_interruptible(wq, cdma_comp[cdma_num] != 0);
	cdma_comp[cdma_num] = 0;
	printk(KERN_INFO"	<pci_dma_transfer>: returned from ISR.\n");
	
	// Acknowledge the CDMA and check for error status
	ack_status = cdma_ack(cdma_num);

	if ((keyhole_en == KEYHOLE_READ) | (keyhole_en == KEYHOLE_WRITE))
	{
		bit_vec = 0x20 | 0x10;      // "0x30" unset both CDMA keyhole read and write	
		cdma_config_set(bit_vec, 0, cdma_num);	//unsets the keyhole configuration
	}

//	printk(KERN_INFO"	<pci_dma_transfer>: Unlocking the CDMA Resource\n");	
//	mutex_unlock(&CDMA_sem);

	printk(KERN_INFO"	<pci_dma_transfer>: ********* CDMA TRANSFER FINISHED *************\n");	

	return ack_status;
}

void cdma_config_set(u32 bit_vec, int set_unset, int cdma_num)
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
		default:printk(KERN_INFO"	!!!!!!!!ERROR: incorrect CDMA number detected!!!!!!!\n");
	}

	axi_dest = axi_cdma_loc + CDMA_CR;
	status = data_transfer(axi_dest, (void *)&current_CR, 4, NORMAL_READ, 0);

	if (set_unset == 1)   //we are setting the CR
	{
		printk(KERN_INFO"<cdma_config_set>: setting the keyhole config on CDMA.\n");
		set_vec = (current_CR | bit_vec);
		status = data_transfer(axi_dest, (void *)&set_vec, 4, NORMAL_WRITE, 0);
	}
	else    //We are unsetting bits in the CR
	{
		set_vec = (current_CR & (~bit_vec));
		printk(KERN_INFO"<cdma_config_set>: UNsetting the keyhole config on CDMA.\n");
		status = data_transfer(axi_dest, (void *)&set_vec, 4, NORMAL_WRITE, 0);
	}

	status = data_transfer(axi_dest, (void *)&new_CR, 4, NORMAL_READ, 0);
	printk(KERN_INFO"<pci_cdma_config_set>: new CDMA Config Register is:%x\n", new_CR);	
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
		default:printk(KERN_INFO"	!!!!!!!!ERROR: incorrect CDMA number detected!!!!!!!\n");
	}

	/*acknowledge the CDMA interrupt (to reset it)*/
	cdma_status = 0x00001000;
	axi_dest = axi_cdma_loc + CDMA_SR;
	ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_WRITE, 0);
	printk(KERN_INFO"	<cdma_ack>: writing SR config ('%x') to CDMA at AXI address:%llx\n", cdma_status, axi_dest);

	/* Check the status of the CDMA to see if successful */
	ret = data_transfer(axi_dest, (void *)&status, 4, NORMAL_READ, 0);
	printk(KERN_INFO"	<cdma_ack>: CDMA status:%x\n", status);
	if (status != 0x2)    //this is the expected status report  
	{
		printk(KERN_INFO"	<cdma_ack>: CDMA status ERROR\n");
		printk(KERN_INFO"	<cdma_ack>: Issuing soft reset to CDMA\n");
//		axi_dest = axi_cdma + CDMA_CR;
//		cdma_status = 0x4;
//		data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_WRITE);
		
		axi_dest = axi_cdma_loc;
	//	cdma_init(cdma_num);
	
	/*Issue a Soft Reset*/
	axi_dest = axi_cdma_loc + CDMA_CR;
	cdma_status = 0x00000004;
	printk(KERN_INFO"<cdma_%x_init>: sending a soft reset to the CDMA\n", cdma_num);
	ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_WRITE, 0);
	/*Check the current status*/
	axi_dest = axi_cdma_loc + CDMA_SR;
	//			direct_read(axi_dest, (void*)&cdma_status, 4, NORMAL_READ);
	ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_READ, 0);
	printk(KERN_INFO"<cdma_%x_init>: CDMA status before configuring:%x\n", cdma_num, cdma_status);	
	/*Check the current config*/
	axi_dest = axi_cdma_loc + CDMA_CR;
	//			direct_read(axi_dest, (void*)&cdma_status, 4, NORMAL_READ);
	ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_READ, 0);
	printk(KERN_INFO"<cdma_%x_init>: CDMA config before configuring:%x\n", cdma_num, cdma_status);	
	/*clear any pre existing interrupt*/
	axi_dest = axi_cdma_loc + CDMA_SR;
	cdma_status = 0x00001000;
	printk(KERN_INFO"<cdma_%x_init>: attempting to write:%x to cdma status reg\n", cdma_num, cdma_status);
	ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_WRITE, 0);

	cdma_status = 0x00001000;
	axi_dest = axi_cdma_loc + CDMA_CR;
	ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_WRITE, 0);

	/*Check the current status*/
	axi_dest = axi_cdma_loc + CDMA_SR;
	ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_READ, 0);
	printk(KERN_INFO"<cdma_%x_init>: CDMA status after configuring:%x\n", cdma_num, cdma_status);	
	/*Check the current config*/
	axi_dest = axi_cdma_loc + CDMA_CR;
	ret = data_transfer(axi_dest, (void *)&cdma_status, 4, NORMAL_READ, 0);
	printk(KERN_INFO"<cdma_%x_init>: CDMA config after configuring:%x\n", cdma_num, cdma_status);	
		
		return status;
	}
	else {
		return 0;  // successful CDMA transmission
	}
}


