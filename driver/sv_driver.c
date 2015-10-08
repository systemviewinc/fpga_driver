//#include <linux/config.h>
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
#include <linux/moduleparam.h>

static int device_id = 100;
static int major = 241;
//static char pci_devName[] = "System View Default";
//static char *name = "noname";

module_param(device_id, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(device_id, "DeviceID");

module_param(major, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(major, "MajorNumber");

//module_param(pci_devName, charp, 0000);
//MODULE_PARM_DESC(pci_devName, "DeviceName");

//#define PCI_VENDOR_ID_XILINX  0x10EE
#define ERROR   -1
#define SUCCESS 0
#define SLAVE 0 
#define SLAVE_W_INTERRUPT 1
#define MASTER 2

#define SYS_REG_OFFSET(REG_NUM) ((REG_NUM) * 4)
#define USR_REG_OFFSET(REG_NUM) ((REG_NUM) * 4 + (REG_SIZE / 2))

#define MAX_NUM_MASTERS 2
#define MAX_NUM_SLI 2

#define MAX_NUM_INT MAX_NUM_MASTERS + MAX_NUM_SLI
//const int MAX_NUM_SLI = 2;
//const int MAX_NUM_MASTERS = 2;

const char pci_devName[] = "pci_skel"; //name of the device
//const char pci_devName[] = name;       //name of the device
unsigned long pci_bar_hw_addr;         //hardware base address of the device
unsigned long pci_bar_size;            //hardware bar memory size
struct pci_dev * pci_dev_struct = NULL; //pci device struct
void * pci_bar_vir_addr = NULL;        //hardware base virtual address
void * pcie_ctl_virt_addr = NULL;      // virtual address of pcie CTL interface
void * cdma_virt_addr = NULL;          // virtual address of CDMA
void * int_ctrl_virt_addr = NULL;      // virtual address of Interrupt Controller
u32 axi_dev0;                 // base address of axi slave mapped to minor 0
u32 axi_dev1;                 // base address of axi slave mapped to minor 1
u32 axi_dev2;                 // base address of axi slave mapped to minor 2
u8 cdma_set;
u8 pcie_ctl_set;
static u8 zero_buf_set = 0;
u8 pcie_m_set;
u8 int_ctrl_set;
void * zero_copy_buf;
int cdma_status;

//const int pci_major = 241;             //major number not dynamic
u32 axi_pcie_m;

dma_addr_t dma_addr;

dma_addr_t dma_m_addr[MAX_NUM_MASTERS];

void * dma_master_buf[MAX_NUM_MASTERS];

/* *************************************************  */

size_t dma_size;              //this is a global variable to interrupt to recognize
static wait_queue_head_t wq;
static wait_queue_head_t wq_periph;
static int cdma_comp = 0;
int num_int;
int interrupt_vect_dict[1 << (MAX_NUM_INT)];    //2^x

/*These are register offsets of Xilinx peripherals*/
const u32 AXIBAR2PCIEBAR_0L = 0x20c;
const u32 CDMA_CR = 0x00;
const u32 CDMA_SR = 0x04;
const u32 CDMA_DA = 0x20;
const u32 CDMA_SA = 0x18;
const u32 CDMA_BTT = 0x28;
const u32 INT_CTRL_IER = 0x08;
const u32 INT_CTRL_MER = 0x1c;
const u32 INT_CTRL_ISR = 0x00;
const u32 INT_CTRL_IAR = 0x0C;

/*IOCTLS */
#define SET_AXI_DEVICE 50
#define SET_AXI_CDMA  51
//#define RESERVED 2
//#define RESERVED2 2
#define SET_AXI_PCIE_CTL 52
#define SET_AXI_PCIE_M 53
#define SET_AXI_INT_CTRL 54
#define SET_AXI_DEV_SI 55
#define SET_AXI_DEV_M 56
#define CLEAR_AXI_INTERRUPT_CTLR 60
#define SET_CDMA_KEYHOLE_WRITE 58
#define SET_CDMA_KEYHOLE_READ 59





/*this is the module description struct*/
struct mod_desc 
{
	int minor;
	u32 axi_addr;
	int mode;
	int * wait_var;
	int master_num;
	u32 interrupt_vec;
};

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
static int __init pci_skel_init(void);
static void pci_skel_exit(void);
static int probe(struct pci_dev * dev, const struct pci_device_id * id);
static void remove(struct pci_dev * dev);
static unsigned char skel_get_revision(struct pci_dev * dev);

/************************** ISR functions **************************** */
static irqreturn_t pci_isr(int irq, void *dev_id);
/* ********************** other functions **************************8 */
void cdma_transfer(u32 SA, u32 DA, u32 BTT);
int cdma_ack(void);
void cdma_config_set(u32 bit_vec, int set_unset);


static struct pci_device_id ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_XILINX, 0x7022), },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, ids);

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

	pci_dev_struct = dev;
	if(NULL == pci_dev_struct){
		printk(KERN_INFO"%s:<probe>struct pci_dev_struct is NULL\n", pci_devName);
		return ERROR;
	}

	//get the base hardware address
	pci_bar_hw_addr = pci_resource_start(pci_dev_struct, 0);
	if (0 > pci_bar_hw_addr){
		printk(KERN_INFO"%s<probe>base hardware address is not set\n", pci_devName);
		return ERROR;
	}
	//get the base memory size
	pci_bar_size = pci_resource_len(pci_dev_struct, 0);

	//map the hardware space to virtual space
	pci_bar_vir_addr = ioremap(pci_bar_hw_addr, pci_bar_size);
	if(0 == pci_bar_vir_addr){
		printk(KERN_INFO"%s:<probe>ioremap error when mapping to vritaul address\n", pci_devName);
		return ERROR;
	}

	// check kernel memory region
	//	if (0 > check_mem_region(pci_bar_hw_addr, REG_SIZE)){
	//		printk(KERN_INFO"%s:<probe>kernel memory in use\n", pci_devName);
	//		return ERROR;
	//	}

	//	request_mem_region(pci_bar_hw_addr, REG_SIZE, "xilinx driver");
	//	pci_state_flag = pci_state_flag | HAVE_REGION;

	//enable the device
	pci_enable_device(dev);

	//set DMA mask
	if(0 != dma_set_mask(&dev->dev, 0xFFFFFFFF)){
		printk(KERN_INFO"%s:<probe>set DMA mask error\n", pci_devName);
		return ERROR;
	}

	//enable bus mastering
	pci_set_master(dev);

	//enable MSI interrupts
	if(0 > pci_enable_msi(pci_dev_struct)){
		printk(KERN_INFO"%s:<probe>MSI enable error\n", pci_devName);
		return ERROR;
	}

	//request IRQ
	if(0 > request_irq(pci_dev_struct->irq, &pci_isr, IRQF_SHARED, pci_devName, pci_dev_struct)){
		printk(KERN_INFO"%s:<probe>request IRQ error\n", pci_devName);
		return ERROR;
	}

	/*DMA Allocation*/
	//	zero_copy_buf = pci_alloc_consistent(pci_dev_struct, 4096, &dma_addr);
	//	if(NULL == zero_copy_buf)
	//	{
	//		printk("%s:<probe>DMA zero copy buff allocation ERROR\n", pci_devName);
	//		return ERROR;
	//	}


	//register the char device
	if(0 > register_chrdev(major, pci_devName, &pci_fops)){
		printk(KERN_INFO"%s:<probe>char driver not registered\n", pci_devName);
		return ERROR;
	}
	//	pci_state_flag = pci_state_flag | HAVE_REG_DEV;

	if (skel_get_revision(dev) == 0x42)
		return -ENODEV;

	//set defaults
	cdma_set = 0;
	pcie_ctl_set = 0;
	pcie_m_set = 0;
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

	if(zero_buf_set == 1) 
	{
		pci_free_consistent(pci_dev_struct, 4096, zero_copy_buf, dma_addr);
	}

	unregister_chrdev(major, pci_devName);

}

static struct pci_driver pci_driver = {
	.name = "pci_skel",
//	.name = pci_devName,
	.id_table = ids,
	.probe = probe,
	.remove = remove,
};

static int __init pci_skel_init(void)
{
	printk(KERN_INFO"<pci_init>: Device ID: ('%d')\n", device_id);
	printk(KERN_INFO"<pci_init>: Major Number: ('%d')\n", major);

	init_waitqueue_head(&wq);
	init_waitqueue_head(&wq_periph);

	ids[0].vendor =  PCI_VENDOR_ID_XILINX;
	ids[0].device =  (u32)device_id;
        //ids[1] =  PCI_DEVICE(PCI_VENDOR_ID_XILINX, 0x7028);
	//MODULE_DEVICE_TABLE(pci, ids);

	return pci_register_driver(&pci_driver);
}

static void __exit pci_skel_exit(void)
{
	pci_unregister_driver(&pci_driver);
}

MODULE_LICENSE("GPL");

module_init(pci_skel_init);
module_exit(pci_skel_exit);

/********************** ISR Stuff ***************************/
static irqreturn_t pci_isr(int irq, void *dev_id)
{
	//void* cdma_virt_addr_loc;
	void* int_ctrl_addr_loc;
	//u32 cdma_sr;
	u32 status;
//	int * wait_condition;

	printk(KERN_INFO"<pci_isr>: Entered the pci ISR");

	/*Here we need to find out who triggered the interrupt*
	 *Since we only allow one MSI vector, we need to query the
	 *Interrupt controller to find out. */

	/*This is the interrupt status register*/
	int_ctrl_addr_loc = int_ctrl_virt_addr + INT_CTRL_ISR;
	status = ioread32((u32 *) int_ctrl_addr_loc);
	printk(KERN_INFO"<pci_isr>: interrupt status register vector is: ('%x')\n", status);

	/*bit mask and find out who interrupted*/
	if (status & 0x1)
	{
		/*function to reset the CDMA and check status
		 *returns status register read if error*/
		cdma_status = cdma_ack();

		/*Here we need to clear the service interrupt in the interrupt acknowledge register*/
		int_ctrl_addr_loc = int_ctrl_virt_addr + INT_CTRL_IAR;
		iowrite32(0x01, (u32 *)int_ctrl_addr_loc);

		cdma_comp = 1;      //condition for wake_up
		wake_up_interruptible(&wq);
	}

	else if (status & 0x2)
	{
		interrupt_vect_dict[0x2] = 1;
	
		/*since we don't know exactly how to handle interrupt clearing of this device
		 *we push the interrupt acknowleding to the user space */
	
		wake_up(&wq_periph);
	
	}

	else
	{
		
	}
	
	return IRQ_HANDLED;
}

/* -----------------File Operations-------------------------*/
int pci_open(struct inode *inode, struct file *filep)
{

	struct mod_desc * s;
	s = (struct mod_desc *)kmalloc(sizeof(struct mod_desc), GFP_KERNEL);
	s->minor = MINOR(inode->i_rdev);
	s->axi_addr = 0;
	s->mode = 0;   //defaults as slave only
	s->wait_var = 0;
	s->master_num = 0;
	s->interrupt_vec = 0;
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
	if (mod_desc->mode == MASTER)
	{
		//unallocate DMA
		pci_free_consistent(pci_dev_struct, 4096, dma_master_buf[mod_desc->master_num], dma_m_addr[mod_desc->master_num]);
	}

	kfree((const void*)filep->private_data);

	return SUCCESS;
}


/*item for file_operations: unlocked ioctl*/
long pci_unlocked_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	u32 axi_pcie_ctl;
	u32 axi_cdma;
	u32 axi_int_ctrl;	
	u32 cdma_sr;
	u32 status;
	void __user *argp = (void __user *)arg;

	void * cdma_virt_addr_loc;
	void * pcie_ctl_virt_addr_loc;
	void * int_ctrl_virt_addr_loc;
	u32 dma_addr_loc;
	//struct register_struct * user_reg;
	//int ret_value;
	struct mod_desc *mod_desc;
//	int * w_var;
	static int master_count = 0;
	u32 bit_vec;
	u32 arg_loc; 

	copy_from_user(&arg_loc, argp, sizeof(u32));
	
	printk("<ioctl>Entering IOCTL with command: %d\n", cmd);

	mod_desc = filep->private_data;   

	switch(cmd){
	
		case SET_AXI_DEVICE:
			printk(KERN_INFO"<ioctl>: Setting Peripheral Axi Address:%x\n", arg_loc);
			mod_desc->axi_addr = arg_loc;
			break;
	
		
		case SET_AXI_CDMA:
			printk(KERN_INFO"<ioctl>: Setting CDMA AXI Address:%x\n", arg_loc);
			axi_cdma = arg_loc;
			cdma_virt_addr = axi_cdma + pci_bar_vir_addr;
			cdma_set = 1;
			/*Clear any pre existing interrupt*/
			cdma_virt_addr_loc = cdma_virt_addr + CDMA_SR;
			cdma_sr = 0x00001000;
			iowrite32(cdma_sr, (u32 *)cdma_virt_addr_loc);


			if (zero_buf_set == 0)
			{
				/*DMA Allocation*/
				/*This sets the DMA read and write kernel buffers and obtains the
				 * dma_addr's to be sent to the CDMA core upon R/W transactions*/

				zero_copy_buf = pci_alloc_consistent(pci_dev_struct, 4096, &dma_addr);
				if(NULL == zero_copy_buf)
				{
					printk("%s:<probe>DMA zero copy buff allocation ERROR\n", pci_devName);
					return ERROR;
				}
				zero_buf_set = 1;
				printk(KERN_INFO"<ioctl_CDMA_Set>: dma address is::%x\n", dma_addr);
			}

			/*This checks to see if the user has set the pcie_ctl base address
			 * if so, we can go ahead and write the dma_addr to the pcie translation
			 * register.*/
			if(pcie_ctl_set == 1)
			{
				/*convert to u32 to send to CDMA*/
				dma_addr_loc = (u32)dma_addr;

				//update pcie_ctl_virt address with register offset
				pcie_ctl_virt_addr_loc = pcie_ctl_virt_addr + AXIBAR2PCIEBAR_0L;

				//write DMA addr to PCIe CTL for address translation
				iowrite32(dma_addr_loc, (u32 *)pcie_ctl_virt_addr_loc);
				printk(KERN_INFO"<pci_ioctl_cdma_set>: writing dma address ('%x') to pcie_ctl at virtual address:%p\n", dma_addr_loc, pcie_ctl_virt_addr_loc);

				//check the pcie-ctl got the translation address
				status = ioread32((u32 *)pcie_ctl_virt_addr_loc);
				printk(KERN_INFO"<pci_ioctl_cdma_set>: PCIe CTL register:%x\n", status);

				//Setting the virtual CDMA 
				//write to CDMA
				//
				//Writing SR, which is to enable the interrupt on complete
				cdma_virt_addr_loc = cdma_virt_addr + CDMA_CR;
				cdma_sr = 0x00001000;
				iowrite32(cdma_sr, (u32 *)cdma_virt_addr_loc); 
				printk(KERN_INFO"<pci_ioctl_cdma_set>: writing SR config ('%x') to CDMA at virtual address:%p\n", cdma_sr, cdma_virt_addr_loc);

			}
			break;

			//	case GET_AXI_CDMA:
			//		user_reg = (struct register_struct *)arg;
			//		user_reg->value = ioread32((u32 *)(cdma_virt_addr + (u32)user_reg->reg));

		case SET_AXI_PCIE_CTL:
			printk(KERN_INFO"<ioctl>: Setting PCIe Control Axi Address\n");
			axi_pcie_ctl = arg_loc;
			pcie_ctl_virt_addr = axi_pcie_ctl + pci_bar_vir_addr;
			pcie_ctl_set = 1;

			if(cdma_set == 1)
			{
				/*convert to u32 to send to CDMA*/
				dma_addr_loc = (u32)dma_addr;

					//update pcie_ctl_virt address with register offset
				pcie_ctl_virt_addr_loc = pcie_ctl_virt_addr + AXIBAR2PCIEBAR_0L;

				//write DMA addr to PCIe CTL for address translation
				iowrite32(dma_addr_loc, (u32 *)pcie_ctl_virt_addr_loc);
				printk(KERN_INFO"<pci_ioctl_pcie_set>: writing dma address ('%x') to pcie_ctl at virtual address:%p\n", dma_addr_loc, pcie_ctl_virt_addr_loc);

				//check the pcie-ctl got the translation address
				status = ioread32((u32 *)pcie_ctl_virt_addr_loc);
				printk(KERN_INFO"<pci_ioctl_pcie_set>: PCIe CTL register:%x\n", status);

				//Setting the virtual CDMA 
				//write to CDMA
				//
				//Writing SR, which is to enable the interrupt on complete
				cdma_virt_addr_loc = cdma_virt_addr + CDMA_CR;
				cdma_sr = 0x00001000;
				iowrite32(cdma_sr, (u32 *)cdma_virt_addr_loc); 
				printk(KERN_INFO"<pci_ioctl_pcie_set>: writing SR config ('%x') to CDMA at virtual address:%p\n", cdma_sr, cdma_virt_addr_loc);
			}
			break;

		case SET_AXI_PCIE_M:
			printk(KERN_INFO"<ioctl>: Setting PCIe Master Axi Address\n");
			axi_pcie_m = arg_loc;
			pcie_m_set = 1;

			/*Here we need to set more AXI addresses for each AXI->PCIe BAR*/
			/* ....... */

			break;

			/*This is the Interrupt controller that will MUX in all interrupts from axi masters and produce
			 *one interrupt output to the the PCIe controller.  This is because we want to use one msi vector. */
		case SET_AXI_INT_CTRL:
			printk(KERN_INFO"<ioctl>: Setting Interrupt Controller Axi Address\n");
			axi_int_ctrl = arg_loc;
			int_ctrl_virt_addr = axi_int_ctrl + pci_bar_vir_addr;
			int_ctrl_set = 1;

			/*Write to Interrupt Enable Register (IER)*/
			/* Write to enable all possible interrupt inputs */
			int_ctrl_virt_addr_loc = int_ctrl_virt_addr + INT_CTRL_IER;
			iowrite32(0xFFFFFFFF, (u32 *)int_ctrl_virt_addr_loc);

			/*Write to the Master Enable Register (MER) */
			/* Write to enable the hardware interrupts */
			int_ctrl_virt_addr_loc = int_ctrl_virt_addr + INT_CTRL_MER;
			iowrite32(0x3, (u32 *)int_ctrl_virt_addr_loc);  //binary "11"
			status = ioread32((u32 *)int_ctrl_virt_addr_loc);  //binary "11"
			printk(KERN_INFO"<pci_axi_interrupt_control_set>: read: ('%x') from MER register\n", status);

			/*Here we need to clear the service interrupt in the interrupt acknowledge register*/ 
			int_ctrl_virt_addr_loc = int_ctrl_virt_addr + INT_CTRL_IAR;
			iowrite32(0xFFFFFFFF, (u32 *)int_ctrl_virt_addr_loc);

			break;

		case SET_AXI_DEV_SI:
			printk(KERN_INFO"<ioctl>: Setting device as an AXI Slave with Interrupt\n");
			/*set the slave with interrupt indicator*/
			mod_desc->mode = SLAVE_W_INTERRUPT;
			/*allocate a pointer for the wait variable*/
			//w_var = (int *)kmalloc(sizeof(int), GFP_KERNEL);
			/*initialize wait variable to 0*/
			//*w_var = 0;    
			/*Store the wait variable in the file descriptor struct*/
			//mod_desc->wait_var = w_var;
			/*Store the Interrupt Vector*/
			mod_desc->interrupt_vec = arg_loc;
			/*initialize the interrupt vector dictionary to 0*/
			interrupt_vect_dict[arg_loc] = 0; 
			break;
	
		case SET_AXI_DEV_M:
			printk(KERN_INFO"<ioctl>: Setting device as an AXI Master\n");

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
		
			/*Store the Interrupt Vector*/
			mod_desc->interrupt_vec = arg_loc;
			 
		
			master_count++;
		break;

		case CLEAR_AXI_INTERRUPT_CTLR:
			
			printk(KERN_INFO"<ioctl>: Clearing the axi interrupt vector:%x\n", mod_desc->interrupt_vec);
			
			/*Here we need to clear the service interrupt in the interrupt acknowledge register*/ 
			int_ctrl_virt_addr_loc = int_ctrl_virt_addr + INT_CTRL_IAR;
			iowrite32(mod_desc->interrupt_vec, (u32 *)int_ctrl_virt_addr_loc);
		break;

		case SET_CDMA_KEYHOLE_WRITE:
			bit_vec = 0x00000020;   //the bit for KEYHOLE WRITE
			
			if(arg_loc>0)    //We are ENABLING keyhole write
			{
				printk(KERN_INFO"<ioctl_keyhole_write_set>: Setting the CDMA Keyhole WRITE as ENABLED\n");
				cdma_config_set(bit_vec, 1);   //value of one means we want to SET the register
			}
			else    //We are disabling keyhole write
			{
				printk(KERN_INFO"<ioctl_keyhole_write_disable>: Setting the CDMA Keyhole WRITE as DISABLED\n");
				cdma_config_set(bit_vec, 0);   //value of 0 means we want to UNSET the register
			}
		break;

		case SET_CDMA_KEYHOLE_READ:
			bit_vec = 0x00000010;   //the bit for KEYHOLE READ
			
			if(arg_loc>0)    //We are ENABLING keyhole write
			{
				printk(KERN_INFO"<ioctl_keyhole_read_set>: Setting the CDMA Keyhole READ as ENABLED\n");
				cdma_config_set(bit_vec, 1);   //value of one means we want to SET the register
			}
			else    //We are disabling keyhole write
			{
				printk(KERN_INFO"<ioctl_keyhole_read_disable>: Setting the CDMA Keyhole READ as DISABLED\n");
				cdma_config_set(bit_vec, 0);   //value of 0 means we want to UNSET the register
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
	has_data = interrupt_vect_dict[mod_desc->interrupt_vec];
	if (has_data == 1)
	{
		printk(KERN_INFO"<pci_poll>: wait event detected!!\n");
		/*reset the has_data flag*/
		interrupt_vect_dict[mod_desc->interrupt_vec] = 0;
		mask |= POLLIN;
	}
	
	return mask;
	

}

ssize_t pci_write(struct file *filep, const char __user *buf, size_t count, loff_t *f_pos)
{
	void * virt_addr;
	u32 axi_dest;
	int dbg;
	void * kern_buf;
	u32 * newaddr_src;
	u32 * newaddr_dest;
	int i;
	int len;
	u32 offset;
	struct mod_desc * mod_desc;
	size_t bytes;
	//u32 ret;
	//void * int_ctrl_addr_loc;

	bytes = 0;

	/*this gets the minor number of the calling file so we can map the correct AXI address
	 *to write to*/
	//dev_num = filep->private_data;
	mod_desc = filep->private_data;

	/* Here we will decide whether to do a zero copy DMA, or to write */
	/* directly to the peripheral */

	if ((cdma_set == 1) & (pcie_ctl_set == 1) & (pcie_m_set == 1) & ((u32)count > 28)) {

		//using CDMA
		printk(KERN_INFO"<pci_write>: writing peripheral using a zero copy DMA\n");

		//Transfer buffer from user space to kernel space at the allocated DMA region
		copy_from_user(zero_copy_buf, buf, count);

		axi_dest = mod_desc->axi_addr + filep->f_pos;
		//cdma_transfer(axi_pcie_m, mod_desc->axi_addr, (u32)count);
		cdma_transfer(axi_pcie_m, axi_dest, (u32)count);

		/*Go to sleep and wait for interrupt*/
		wait_event_interruptible(wq, cdma_comp != 0);
		cdma_comp = 0;
		printk(KERN_INFO"<pci_cdma_write>: process awoke.\n");

		/*function to reset the CDMA and check status
		 *returns status register read if error*/
//		ret = cdma_ack();
		if(cdma_status > 0)
		{
			bytes = (size_t)cdma_status;
		}
		else
		{
			bytes = count;
		}

	}
	else {     

		printk(KERN_INFO"<pci_write>: writing peripheral without DMA\n");

		/*assign buffer in the kernel space to get user space data */
		kern_buf = kmalloc(count, GFP_KERNEL);

		/*This is the function to move data from user space to kernel space*/
		//copy_from_user(virt_addr, buf, count);
		copy_from_user(kern_buf, buf, count);

		/*this is the final virtual address*/
		virt_addr = mod_desc->axi_addr + pci_bar_vir_addr + filep->f_pos;
		printk(KERN_INFO"<pci_write>: writing to virtual address:('%p')\n", virt_addr);

		offset = 0;
		len = count/4;
		printk(KERN_INFO"<pci_write>: writing %d 32bit words.\n", len);
		for(i = 0; i<len; i++)
		{
			newaddr_src = (u32 *)(kern_buf + offset);
			newaddr_dest = (u32 *)(virt_addr + offset);
			iowrite32(*newaddr_src, newaddr_dest);
			offset += 4;
			printk(KERN_INFO"<pci_write>: wrote to kernel address %p.\n", newaddr_dest);
		}

		/*Free up allocated kernel memory*/
		kfree((const void*)kern_buf);

		/*this is just for debugging*/
		dbg = (int)*buf;
		printk(KERN_INFO"<pci_write>: writing value ('%x') from user space address:%p\n",dbg,buf);
		bytes = count;      //need to add some sort of error checking
	}

	return bytes;

}

ssize_t pci_read(struct file *filep, char __user *buf, size_t count, loff_t *f_pos)
{
	void * virt_addr;
	//u32 axi_addr;
	int dbg;
	u32 * newaddr_src;
	int i;
	int len;
	u32 offset;
	unsigned int kern_buf[count/4];
	u32 axi_dest;
	struct mod_desc *mod_desc;
	int bytes;
//	int * isr_m_comp;
	//u32 ret;
	//void * int_ctrl_addr_loc;

	mod_desc = filep->private_data;

	bytes = 0;

	/* Here we need to look at the file descriptor to see if this is 
	 * an interrupt designated device.  If it is, we just to block
	 * until an interrupt has occured from the master, indicating
	 * that it has sent data to system memory or is ready to 
	 * to be read from */
//	if  (mod_desc->mode > 0) //  Device with interrupt
//	{
//		printk(KERN_INFO"<pci_read>: performing a blocking read\n");

		/*We need to find out which wait variable to wait on*/
//		isr_m_comp = mod_desc->wait_var;
//		wait_event_interruptible(wq_m, *isr_m_comp != 0);
//		printk(KERN_INFO"<pci_read>: process awoke on a blocking read\n");

		/*Reset the wait variable back to zero*/
//		*isr_m_comp = 0;   

		/*We leave the interrupt clearing and actions to the user space application*/

		//Transfer buffer from kernel space to user space at the allocated DMA region
		if (mod_desc->mode == MASTER)
		{
			printk(KERN_INFO"<pci_read>: Transferred a Master write from kernel space to user space\n");
			copy_to_user(buf, dma_master_buf[mod_desc->master_num], count);
			return count;
		}

//		return count;
//	}

//	else   //slave only device without interrupt
//	{
		/* Here we will decide whether to do a zero copy DMA, or to read */
		/* directly from the peripheral */

		if ((cdma_set == 1) & (pcie_ctl_set == 1) & (pcie_m_set == 1) & ((u32)count > 28)) {

			//using CDMA
			printk(KERN_INFO"<pci_cdma_read>: reading peripheral using a zero copy DMA\n");

			axi_dest = mod_desc->axi_addr + filep->f_pos;
			//	cdma_transfer(mod_desc->axi_addr, axi_pcie_m, (u32)count);
			cdma_transfer(axi_dest, axi_pcie_m, (u32)count);

			/*Go to sleep and wait for interrupt*/
			wait_event_interruptible(wq, cdma_comp != 0);
			cdma_comp = 0;
			printk(KERN_INFO"<pci_cdma_read>: process awoke.\n");

			//Transfer buffer from kernel space to user space at the allocated DMA region
			copy_to_user(buf, zero_copy_buf, count);

			//ret = cdma_ack();

			if(cdma_status > 0)  //Condition if CDMA detects an error
			{            // This will return the error vector. Refer to Xilinx Documentation.
				bytes = (size_t)cdma_status;
			}
			else
			{
				bytes = count;
			}
		
		}

		else {     
			printk(KERN_INFO"<pci_read>: reading peripheral without DMA\n");

			virt_addr = mod_desc->axi_addr + pci_bar_vir_addr + filep->f_pos;
			printk(KERN_INFO"<pci_read>: reading from virtual address:('%p')\n", virt_addr);

			offset = 0;
			len = count/4;

			printk(KERN_INFO"<pci_read>: reading %d 32bit words.\n", len);
			for(i = 0; i<len; i++)
			{
				newaddr_src = (u32 *)(virt_addr + offset);
				kern_buf[i] = ioread32(newaddr_src);
				offset += 4;
				printk(KERN_INFO"<pci_read>: read from kernel address %p.\n", newaddr_src);
			}

			/*This is the function to move data from user space to kernel space*/
			copy_to_user(buf, (void *)kern_buf, count);
			dbg = (int)*buf;
			printk(KERN_INFO"<pci_read>: read value:%x and placed in user space address:%p\n", dbg, buf);
			bytes = count;    //needs some sort of error checking....
		}

		return bytes;
//	}
}

/******************************** Support functions ***************************************/


void cdma_transfer(u32 SA, u32 DA, u32 BTT)
{
	void * cdma_virt_addr_loc;

	//Writing SA, which is the PCIe_Master AXI address	
	cdma_virt_addr_loc = cdma_virt_addr + CDMA_SA;
	iowrite32(SA, (u32 *)cdma_virt_addr_loc); 
	printk(KERN_INFO"<pci_dma_transfer>: writing dma SA address ('%x') to CDMA at virtual address:%p\n", SA, cdma_virt_addr_loc);

	//Writing DA, which is the target peripheral AXI address	
	cdma_virt_addr_loc = cdma_virt_addr + CDMA_DA;
	iowrite32(DA, (u32 *)cdma_virt_addr_loc); 
	printk(KERN_INFO"<pci_dma_transfer>: writing DA address ('%x') to CDMA at virtual address:%p\n", DA, cdma_virt_addr_loc);

	//Writing BTT	
	cdma_virt_addr_loc = cdma_virt_addr + CDMA_BTT;
	iowrite32(BTT, (u32 *)cdma_virt_addr_loc); 
	printk(KERN_INFO"<pci_dma_transfer>: writing bytes to transfer ('%d') to CDMA at virtual address:%p\n", BTT, cdma_virt_addr_loc);	
}

void cdma_config_set(u32 bit_vec, int set_unset)
{
	u32 current_CR;
	u32 new_CR;
	void * cdma_virt_addr_loc;
	u32 set_vec;

	cdma_virt_addr_loc = cdma_virt_addr + CDMA_CR;

	current_CR = ioread32((u32 *)cdma_virt_addr_loc);

	if (set_unset == 1)   //we are setting the CR
	{
		set_vec = (current_CR | bit_vec);
		iowrite32(set_vec, (u32 *)cdma_virt_addr_loc); 
	}
	else    //We are unsetting bits in the CR
	{
		set_vec = (current_CR & (~bit_vec));
		iowrite32(set_vec, (u32 *)cdma_virt_addr_loc); 
	}

	new_CR = ioread32((u32 *)cdma_virt_addr_loc);
	printk(KERN_INFO"<pci_cdma_config_set>: new CDMA Config Register is:%x\n", new_CR);	
}

int cdma_ack(void)
{
	void * cdma_virt_addr_loc;
	u32 cdma_sr;
	u32 status;

	/*acknowledge the CDMA interrupt (to reset it)*/
	cdma_virt_addr_loc = cdma_virt_addr + CDMA_SR;
	cdma_sr = 0x00001000;
	iowrite32(cdma_sr, (u32 *)cdma_virt_addr_loc); 
	printk(KERN_INFO"<pci_isr>: writing SR config ('%x') to CDMA at virtual address:%p\n", cdma_sr, cdma_virt_addr_loc);

	/* Check the status of the CDMA to see if successful */
	status = ioread32((u32 *)cdma_virt_addr_loc);
	printk(KERN_INFO"<pci_dma_write>: CDMA status:%x\n", status);
	if (status != 0x2)    //this is the expected status report  
	{
		printk(KERN_INFO"<pci_dma_write>: CDMA status ERROR\n");
		printk(KERN_INFO"<pci_dma_write>: Issuing soft reset to CDMA\n");
		cdma_virt_addr_loc = cdma_virt_addr + CDMA_CR;
		iowrite32(0x4, (u32 *)cdma_virt_addr_loc);   //sets the reset bit

		return status;
	}
	else {
		return 0;  // the number of successful byte written
	}
}


