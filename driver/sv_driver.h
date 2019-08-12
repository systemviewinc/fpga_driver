/**
 * System View Device Driver

 * @date 11/10/2015

 * @author	System View Inc.

 * @file sv_driver.h

 * @brief			This driver is to be used to control the
 *					 Xilinx interface IP created by System View Inc.
 ***********************************************************
 */

#ifndef SV_DRIVER_H
#define SV_DRIVER_H

#undef CONFIG_PCI_MSI
#undef _PCIE_ENABLED_

#include <linux/kthread.h>
#include <linux/kfifo.h>

#ifdef _PCIE_ENABLED_
#include "xdma/xdma-core.h"
#include "xdma/sv_xdma.h"
#endif

#ifndef XDMA_BAR_NUM
#define XDMA_BAR_NUM (6)
#endif

#ifndef XDMA_CHANNEL_NUM_MAX
#define XDMA_CHANNEL_NUM_MAX (4)
#endif

/********* printk statements *********/
#define verbose_printk printk
// #define verbose_cdma_printk printk
// #define verbose_dma_printk printk
// #define verbose_dmaq_printk printk
// #define verbose_axi_fifo_read_printk printk
// #define verbose_axi_fifo_write_printk printk
// #define verbose_isr_printk printk
// #define verbose_poll_printk printk
// #define very_verbose_poll_printk printk
// #define verbose_axi_fifo_d2r_printk printk
// #define verbose_direct_write_printk printk
// #define verbose_direct_read_printk printk
// #define verbose_llseek_printk printk
// #define verbose_pci_read_printk printk
// #define verbose_pci_write_printk printk
// #define verbose_mmap_printk printk
// #define verbose_read_thread_printk printk
// #define verbose_write_thread_printk printk
#define pr_bar 1

#ifndef verbose_llseek_printk
#define verbose_llseek_printk(...)
#endif
#ifndef verbose_direct_read_printk
#define verbose_direct_read_printk(...)
#endif
#ifndef verbose_direct_write_printk
#define verbose_direct_write_printk(...)
#endif
#ifndef verbose_cdma_printk
#define verbose_cdma_printk(...)
#endif
#ifndef verbose_dma_printk
#define verbose_dma_printk(...)
#endif
#ifndef verbose_dmaq_printk
#define verbose_dmaq_printk(...)
#endif
#ifndef verbose_axi_fifo_read_printk
#define verbose_axi_fifo_read_printk(...)
#endif
#ifndef verbose_axi_fifo_write_printk
#define verbose_axi_fifo_write_printk(...)
#endif
#ifndef verbose_axi_fifo_d2r_printk
#define verbose_axi_fifo_d2r_printk(...)
#endif
#ifndef verbose_isr_printk
#define verbose_isr_printk(...)
#endif
#ifndef verbose_poll_printk
#define verbose_poll_printk(...)
#endif
#ifndef very_verbose_poll_printk
#define very_verbose_poll_printk(...)
#endif
#ifndef verbose_printk
#define verbose_printk(...)
#endif
#ifndef verbose_pci_read_printk
#define verbose_pci_read_printk(...)
#endif
#ifndef verbose_pci_write_printk
#define verbose_pci_write_printk(...)
#endif
#ifndef verbose_mmap_printk
#define verbose_mmap_printk(...)
#endif
#ifndef verbose_read_thread_printk
#define verbose_read_thread_printk(...)
#endif
#ifndef verbose_write_thread_printk
#define verbose_write_thread_printk(...)
#endif
#ifndef verbose_copy_ring_buf_printk
#define verbose_copy_ring_buf_printk(...)
#endif

#ifndef dbg_bar
#define dbg_bar(fmt, ...) pr_debug("%s():" fmt, __func__, ##__VA_ARGS__)
#endif

#ifndef HEAD_COMMIT
#define HEAD_COMMIT "Undefine."
#endif

/******************************/

/*IOCTLS */
#define SET_AXI_DEVICE 50	/**< IOCTL Magic Number */
#define SET_AXI_CDMA 51	/**< IOCTL Magic Number */
#define SET_AXI_PCIE_CTL 52	/**< IOCTL Magic Number */
#define SET_AXI_PCIE_M 53	/**< IOCTL Magic Number */
#define SET_AXI_INT_CTRL 54 /**< IOCTL Magic Number */
#define SET_AXI_DEV_SI 55	/**< IOCTL Magic Number */
#define SET_AXI_DEV_M 56	/**< IOCTL Magic Number */
#define GET_DMA_SIZE 57	/**< IOCTL Magic Number */
#define SET_CDMA_KEYHOLE_WRITE 58	/**< IOCTL Magic Number */
#define SET_CDMA_KEYHOLE_READ 59	/**< IOCTL Magic Number */
#define CLEAR_AXI_INTERRUPT_CTLR 60	/**< IOCTL Magic Number */
#define SET_INTERRUPT 61	/**< IOCTL Magic Number */
#define SET_MODE 62			/**< IOCTL Magic Number */
#define SET_AXI_CTL_DEVICE 63		/**< IOCTL Magic Number */
#define SET_DMA_SIZE 64		/**< IOCTL Magic Number */
#define RESET_DMA_ALLOC 65	/**< IOCTL Magic Number */
#define SET_FILE_SIZE 66	/**< IOCTL Magic Number */
#define GET_FILE_STATISTICS 67		/**< IOCTL Magic Number */
#define START_FILE_TIMER 68	/**< IOCTL Magic Number */
#define STOP_FILE_TIMER 69	/**< IOCTL Magic Number */
#define GET_DRIVER_STATISTICS 70	/**< IOCTL Magic Number */
#define START_DRIVER_TIMER 71		/**< IOCTL Magic Number */
#define STOP_DRIVER_TIMER 72		/**< IOCTL Magic Number */

#define WRITE_REG 73		/**< IOCTL Magic Number */
#define READ_REG 74		/**< IOCTL Magic Number */

#define FILE_ACTIVATE 75		/**< IOCTL Magic Number */
#define FILE_DEACTIVATE 76		/**< IOCTL Magic Number */
#define SET_DECOUPLER 77		/**< IOCTL Magic Number */

#define ERROR	-1
#define SUCCESS 0

/* MODE Types */
#define SLAVE 0									/**< Mode Type : This is standard memory interface */
#define AXI_STREAM_FIFO 1				/**< Mode Type : This is for axi streaming peripherals with no last singal */
#define MASTER 2								/**< Mode Type : This is for AXI Master devices (currently not supported) */
#define CONTROL 3 		/**< Mode Type : This is for axi streaming peripherals with a last signal */
#define AXI_STREAM_PACKET 4 		/**< Mode Type : This is for axi streaming peripherals with a last signal */

//max number of CDMAs
#define CDMA_MAX_NUM 5
#define BAR_MAX_NUM 5
#define PCIE_RESOURCES_MAX_NUM 6

#define MAX_FILE_DESC 12


#define MAX_NUM_MASTERS 2
#define MAX_NUM_SLI 4 // Max number of slaves with interrupts
#define MAX_NUM_INT MAX_NUM_MASTERS + MAX_NUM_SLI

/*These are the Driver types that are matched through insmod parameter "driver_type" */
#define PCI 1	 /**< Driver Type */
#define PLATFORM 2 /**< Driver Type */
#define AWS 5		/**< Driver Type */

/*These are the CDMA R/W types */
#define NORMAL_WRITE 0
#define NORMAL_READ 1
#define KEYHOLE_WRITE 2
#define KEYHOLE_READ 3

enum xfer_type {
	HOST_READ = 1,
	HOST_WRITE = 2,
	INC_SA = 4,
	INC_DA = 8,
	INC_BOTH = INC_DA|INC_SA
};

/******************************** NON XDMA related **********************************/
extern uint pcie_use_xdma;
extern struct xdma_dev *xdma_dev_s;

#define XDMA_TIMEOUT_IN_MSEC				(3 * 1000)
/******************************** Xilinx interrupt controller Register Offsets **********************************/
#define AXI_STREAM_ISR  	0x00000000  /**< Interrupt Status */
#define AXI_STREAM_IER  	0x00000004  /**< Interrupt Enable */
#define AXI_STREAM_TDFR 	0x00000008  /**< Transmit Reset */
#define AXI_STREAM_TDFV 	0x0000000c  /**< Transmit Vacancy */
#define AXI_STREAM_TLR  	0x00000014  /**< Transmit Length */
#define AXI_STREAM_RDFR 	0x00000018  /**< Receive Reset */
#define AXI_STREAM_RDFO 	0x0000001c  /**< Receive Occupancy */
#define AXI_STREAM_RLR  	0x00000024  /**< Receive Length */
#define AXI_STREAM_SRR  	0x00000028  /**< Local Link Reset */
#define AXI_STREAM_TDR  	0x0000002C  /**< Transmit Destination  */
#define AXI_STREAM_RDR  	0x00000030  /**< Receive Destination  */
#define AXI_STREAM_TXID  	0x00000034  /**< Transmit ID  */
#define AXI_STREAM_TXUSER  	0x00000038  /**< Transmit User  */
#define AXI_STREAM_RXID  	0x0000003C  /**< Receive ID  */
#define AXI_STREAM_RXUSER  	0x00000040  /**< Receive User  */
#define AXI_STREAM_TDFD		0x00000000	 /**< Transmit Data */
#define AXI_STREAM_RDFD		0x00001000 	 /**< Receive Data */


 /******************************** Xilinx interrupt controller bits **********************************/
#define AXI_INTR_RPURE_MASK	   0x80000000 /**< Receive under-read */
#define AXI_INTR_RPORE_MASK	   0x40000000 /**< Receive over-read */
#define AXI_INTR_RPUE_MASK		0x20000000 /**< Receive underrun (empty) */
#define AXI_INTR_TPOE_MASK		0x10000000 /**< Transmit overrun */
#define AXI_INTR_TC_MASK		  0x08000000 /**< Transmit complete */
#define AXI_INTR_RC_MASK		  0x04000000 /**< Receive complete */
#define AXI_INTR_TSE_MASK		 0x02000000 /**< Transmit length mismatch */
#define AXI_INTR_TRC_MASK		 0x01000000 /**< Transmit reset complete */
#define AXI_INTR_RRC_MASK		 0x00800000 /**< Receive reset complete */
#define AXI_INTR_TFPF_MASK		0x00400000 /**< Tx FIFO Programmable Full AXI FIFO MM2S Only */
#define AXI_INTR_TFPE_MASK		0x00200000 /**< Tx FIFO Programmable Empty AXI FIFO MM2S Only */
#define AXI_INTR_RFPF_MASK		0x00100000 /**< Rx FIFO Programmable Full AXI FIFO MM2S Only */
#define AXI_INTR_RFPE_MASK		0x00080000 /**< Rx FIFO Programmable Empty AXI FIFO MM2S Only */

/******************************** Xilinx CDMA status bits **********************************/
#define XAXICDMA_SR_IDLE_MASK		 0x00000002  /**< DMA channel idle */
#define XAXICDMA_SR_SGINCLD_MASK	  0x00000008  /**< Hybrid build */
#define XAXICDMA_SR_ERR_INTERNAL_MASK 0x00000010  /**< Datamover internal err */
#define XAXICDMA_SR_ERR_SLAVE_MASK	0x00000020  /**< Datamover slave err */
#define XAXICDMA_SR_ERR_DECODE_MASK   0x00000040  /**< Datamover decode err */
#define XAXICDMA_SR_ERR_SG_INT_MASK   0x00000100  /**< SG internal err */
#define XAXICDMA_SR_ERR_SG_SLV_MASK   0x00000200  /**< SG slave err */
#define XAXICDMA_SR_ERR_SG_DEC_MASK   0x00000400  /**< SG decode err */

/******************************** Xilinx interrupt reset bits **********************************/
#define XLLF_RDFR_RESET_MASK		0x000000a5 /**< receive reset value */
#define XLLF_TDFR_RESET_MASK		0x000000a5 /**< Transmit reset value */
#define XLLF_LLR_RESET_MASK		 0x000000a5 /**< Local Link reset value */

/******************************** Xilinx CDMA Register Offsets **********************************/
#define CDMA_CR			  	0x00	 /**< CDMA Register Offset (See Xilinx Doc) */
#define CDMA_SR			 	0x04	 /**< CDMA Register Offset (See Xilinx Doc) */
#define CDMA_CU_PTR		 	0x08	 /**< CDMA Register Offset (See Xilinx Doc) */
#define CDMA_CU_PTR_MSB		0x0C	 /**< CDMA Register Offset (See Xilinx Doc) */
#define CDMA_TA_PTR		 	0x10	 /**< CDMA Register Offset (See Xilinx Doc) */
#define CDMA_TA_PTR_MSB		0x14	 /**< CDMA Register Offset (See Xilinx Doc) */
#define CDMA_SA			 	0x18	 /**< CDMA Register Offset (See Xilinx Doc) */
#define CDMA_SA_MSB		 	0x1C	 /**< CDMA Register Offset (See Xilinx Doc) */
#define CDMA_DA			 	0x20	 /**< CDMA Register Offset (See Xilinx Doc) */
#define CDMA_DA_MSB		 	0x24	 /**< CDMA Register Offset (See Xilinx Doc) */
#define CDMA_BTT			0x28	 /**< CDMA Register Offset (See Xilinx Doc) */

#define AXIBAR2PCIEBAR_0U	0x208	 /**< AXI PCIe Subsystem Offset (See Xilinx Doc) */
#define AXIBAR2PCIEBAR_0L	0x20c	 /**< AXI PCIe Subsystem Offset (See Xilinx Doc) */

#define PCIE_BRIDGE_INFO	0x134	 /**< AXI PCIe Subsystem Offset (See Xilinx Doc) */
#define PCIE_ISR_INFO		0x138	 /**< AXI PCIe Subsystem Offset (See Xilinx Doc) */
#define PCIE_PHY_STATUS		0x144	 /**< AXI PCIe Subsystem Offset (See Xilinx Doc) */

#define AXIBAR2PCIEBAR_1L = 0x214	 /**< AXI PCIe Subsystem Offset (See Xilinx Doc) */

/******************************** Xilinx Register Offsets **********************************/
#define INT_CTRL_ISR		0x00	 /**< Interrupt Controller Register Offset, see Xilinx doc. */
#define INT_CTRL_IPR		0x04	 /**< Interrupt Controller Register Offset, see Xilinx doc. */
#define INT_CTRL_IER		0x08 	/**< Interrupt Controller Register Offset, see Xilinx doc. */
#define INT_CTRL_IAR		0x0C	 /**< Interrupt Controller Register Offset, see Xilinx doc. */
#define INT_CTRL_MER		0x1c	 /**< Interrupt Controller Register Offset, see Xilinx doc. */
#define INT_CTRL_ILR		0x24	 /**< Interrupt Controller Register Offset, see Xilinx doc. */

/********************************* HLS Register Offsets ***********************************/
#define HLS_SLAVE_ADDR		0x10	 /**< SLAVE address offset. */

/********************************************************************************************/



#if defined(CONFIG_PCI_MSI)
#define MAX_INTERRUPTS MAX_USER_IRQ
#endif



/* Shared Global Variables */
typedef unsigned int uint;

extern int dma_byte_width;

extern int back_pressure;	/**< This variable is set at insmod that tells whether the read ring buffers should backpressure to HW or overwrite */

extern uint cdma_address[CDMA_MAX_NUM]; 		/**< Holds AXI Base address of CDMA 1 */
extern int cdma_count;

extern struct sv_mod_dev *svd_global;


/** Module Description Struct
 *	@brief This is the data structure that is stored inside the private section of each file
 *	struct that is created for this driver. Since each file pertains to an IP within the FPGA
 *	design, it contains characteristics of the IP suxk as AXI address, mode types, interrupt
 *	numbers, etc.
 */
struct sv_mod_dev {
	dma_addr_t dma_addr_base; /**< The hardware DMA Allocation Address */
	char * dma_buffer_base;	/**< This is the start of the DMA region virtual address */
	char * dma_buffer_end;  /**< End of the buffer */
	u32 dma_current_offset;	/**< This variable holds the current offset of the DMA Allocation */
	u64 dma_buffer_size; /**< Default value for size of DMA Allocation, Max is 4MB, this is set through insmod */

	uint axi_intc_addr; /**< Global Variable that stores the Interrupt Controller AXI Address */
	uint axi_lodc_addr; /**< Global Variable that stores the LOD Controller AXI Address */

	u64 axi_pcie_m; /**< Global Variable that stores the data transport IP Slave AXI Address as seen from the CDMA*/
	u8 cdma_set[CDMA_MAX_NUM]; /**< Global variable that stores which CDMAs have been initialized. (currently only using 2 CDMAs) */

	unsigned int irq_num; /**< Global variable that stores the IRQ number that is probed from the device */

	int cdma_capable; /**< Global variable that is a flag to tell if the driver has been initialized properly to use the CDMA(s), holds the number of cdmas init'd */
	int dma_usage_cnt; /**< Global variable to count the number of CDMA uses. Used for statistics gathering */

	int dma_max_write_size ;	 /**< AWS PCI/e max write size	*/
	int dma_max_read_size ;	 /**< AWS PCI/e max read size	*/

	/*CDMA Semaphores*/
	struct mutex cdma_sem[CDMA_MAX_NUM];

	int xdma_c2h_num_channels;
	int xdma_h2c_num_channels;

	struct bar_info * bars;

	atomic_t sw_interrupt_rx; /**< Global Atomic Variable for Driver Statistics */
	bool interrupt_set;
	bool lod_set;

	/*These are the interrupt and mutex wait variables */
	wait_queue_head_t pci_write_head; /**< The Wait Queue for the blocking/sleeping pci_write function */

	// Write Thread data
	spinlock_t fifo_lock_write; /**< The Spinlock Variable for writing to the WRITE FIFO */
	struct task_struct * thread_struct_write; /**< task_struct used in creation of WRITE Thread */
	atomic_t thread_q_write; /**< The Wait variable for the WRITE Thread */
	wait_queue_head_t  thread_q_head_write; /**< waitq for events */

	// Read Thread data
	spinlock_t fifo_lock_read; /**< The Spinlock Variable for writing to the READ FIFO */
	struct task_struct * thread_struct_read; /**< task_struct used in creation of READ Thread */
	atomic_t thread_q_read; /**< The Wait variable for the READ Thread */
	wait_queue_head_t thread_q_head_read; /**< waitq for events */

	#if defined(CONFIG_PCI_MSI)
	struct msix_entry sv_msix_entry[32];
	#endif

	DECLARE_KFIFO(write_fifo, struct file_desc*, 8192); /**< sets up the struct WRITE FIFO */
	DECLARE_KFIFO(read_fifo, struct file_desc*, 8192); /**< sets up the struct READ FIFO */

	/*Driver Statistics*/
	atomic_t driver_tx_bytes; /**< Global Atomic Variable for Driver Statistics */
	atomic_t driver_rx_bytes;/**< Global Atomic Variable for Driver Statistics */
	atomic_t driver_start_flag;/**< Global Atomic Variable for Driver Statistics */
	atomic_t driver_stop_flag;/**< Global Atomic Variable for Driver Statistics */
	struct timespec driver_start_time;/**< Global Struct for Driver Statistics */
	struct timespec driver_stop_time;/**< Global Struct Variable for Driver Statistics */


};

/** Module Description Struct
 *	@brief This is the data structure that is stored inside the private section of each file
 *	struct that is created for this driver. Since each file pertains to an IP within the FPGA
 *	design, it contains characteristics of the IP suxk as AXI address, mode types, interrupt
 *	numbers, etc.
 */
struct file_desc {
	struct sv_mod_dev *svd;		/**< System View Device/driver (svd) struct */
	int minor;					 /**< The minor number of file node (mainly used for debug msgs */
	int f_flags;				/**< The file flags passed by user when opening the file */
	u64 axi_addr;				/**< The axi address of the file node's associated IP */
	u64 axi_addr_ctl;			/**< The axi control address for axi-streaming IPs */
	u32 mode;						 /**< The mode of the IP which defines it as axi-streaming or memory interface */
	int int_num;					 /**< The Interrupt Number of the associated IP */
	int master_num;				 /**< For future master peripherals, currently UNUSED */
	int keyhole_config;			/**< The Keyhole R/W configuration for associated IP */
	u32 interrupt_vec;			 /**< The Interrupt Vector */
	u32 decoupler_vec;			 /**< The Decoupler Vector */

	atomic_t * in_read_fifo_count;		/**< The number of times the file_desc is in the read fifo, once it reaches 0 we can kfree it */
	atomic_t * in_write_fifo_count;		/**< The number of times the file_desc is in the write fifo, once it reaches 0 we can kfree it */
	atomic_t * mmap_count;						/**< The number of current mmaps if it is zero we will read/write the using the mmap section of memory */

	unsigned long mmap_start_addr;
	unsigned long mmap_end_addr;

	bool file_activate;							/**< True if file is open, used to process the file_desc (or throw it away) in read/write threads */

	size_t dma_size;				/**< The size of allocated DMA buffer */
	size_t max_dma_read_write;				/**< The the largest possible DMA (normally dma_size, but not for steaming) */
	loff_t file_size;	/**< This is the size of the axi_streaming FIFO for streaming peripherals or size of ram for memory peripherals */


	u32 dma_offset_read;		 /**< The Offset byte of the allocated DMA buffer for READ from dma_buffer_base */
	u32 dma_offset_write;				/**< The Offset byte of the allocated DMA buffer for WRITE from dma_buffer_base */
	u32 dma_offset_read_write;				/**< The Offset byte of the allocated DMA buffer for WRITE from dma_buffer_base */

	char __iomem* dma_write_addr;			/**< This is the pointer to the start of DMA buffer for WRITE in virtual address space */
	char __iomem* dma_read_addr;			/**< This is the pointer to the start of DMA buffer for READ in virtual address space */
	char __iomem* dma_read_write_addr;			/**< This is the pointer to the start of DMA buffer for READ in virtual address space */

	void *read_buffer;				//used for xdma only
	void *write_buffer;				//used for xdma only

	int tx_bytes;				/**< This is the TX byte count for statistics generation */
	int rx_bytes;				/**< This is the RX byte count for statistics generation */

	struct timespec * start_time;	/**< This holds the start time for statistics generation */
	struct timespec * stop_time;	 /**< This holds the stop time for statistics generation */
	int start_flag;				 /**< This is used internally by the driver for starting the timer */
	int stop_flag;							/**< This is used internally by the driver for stopping the timer */
	int cdma_attempt;			 		/**< This is a statictic collected to count number of failed cdma semaphore lock attempts */
	int ip_not_ready;			 /**< This is a statistic collected to count the number of times the IP was not ready to accept new data */
	atomic_t * atomic_poll;	 /**< this atomic variable is used in the Poll() function to note if the peripheral is ready to be read */
	int set_dma_flag;			/**< This flag is set by the driver to indicate that a DMA region has been allocated */

	atomic_t * wth;				/**< Write to Hardware pointer for WRITE Ring Buffer */
	atomic_t * wtk;				/**< Write to Kernel pointer for WRITE Ring Buffer */
	atomic_t * write_ring_buf_full;	 /**< WRITE Ring Buffer is full and wth == wtk */
	atomic_t * write_ring_buf_locked;	 /**< WRITE Ring Buffer is full and wth == wtk */


	atomic_t * rfh;				/**< Read from Hardware pointer for READ Ring Buffer - HW DMAs data in read_data_thread*/
	atomic_t * rfu;				/**< Read from User pointer for READ Ring Buffer - user copies data into pci_read */
	atomic_t * read_ring_buf_full;	 /**< READ Ring Buffer is full, and rfu == rfh */
	atomic_t * read_ring_buf_locked;	 /**< READ Ring Buffer is full, and rfu == rfh */


	spinlock_t * ring_pointer_write; /**< Spinlock variable to lock code section for updating ring pointer variables */
	spinlock_t * ring_pointer_read;	/**< Spinlock variable to lock code section for updating ring pointer variables */

	atomic_t file_wait_q_write; /**< The Wait variable for the WRITE Thread */
	atomic_t file_wait_q_read; /**< The Wait variable for the READ Thread */

	atomic_t * pci_write_q;	 /**< Used as a wait variable to wake up a sleeping pci_write function */
	spinlock_t * in_fifo_read;		/**< Spinlock variable to proect code for writing in_fifo flag*/
	spinlock_t * in_fifo_write; /**< Spinlock variable to protect code for writing in_fifo flag */
	int in_fifo_read_flag;			/**< Flag variable to tell if it already exists in the READ FIFO */
	int in_fifo_write_flag;	 /**< Flag variable to tell if it already exists in the WRITE FIFO */
	int has_interrupt_vec;	/**< This file has an interrupt associated with it */
	int has_decoupler_vec;	/**< This file has an decoupler associated with it */
	int axi_fifo_rlr;	/**< Last read RLR value if non-zero this has to be used */
	int axi_fifo_rdfo;	/**< Last read RDFO value if non-zero this has to be used */
	size_t read_header_size; 	/**< Last read_header_size value if non-zero this has to be used */
	wait_queue_head_t poll_wq; /**< waitq for events */

	u32 tx_dest;	/**< Last wrote RLR value if non-zero this has to be used */
	u32 rx_dest;	/**< Last read RLR value if non-zero this has to be used */

	u32 tx_fifo_size;	/**< size of the tx fifo */

	struct xdma_dev *xdma_dev;

};

struct mmap_info {
	char *data; 					/**< the data **/
	int reference;					/**< how many times it is mmapped **/
};

//todo use this
struct bar_info {
	unsigned long pci_bar_addr[BAR_MAX_NUM];
	unsigned long pci_bar_end[BAR_MAX_NUM];
	uint num_bars;
	char * pci_bar_vir_addr[BAR_MAX_NUM];		 //hardware base virtual address
};

extern struct bar_info * bars;

extern struct file_desc * file_desc_arr[MAX_FILE_DESC];

/** File Statistics Struct
 *	@brief This is a data structure that contains fields for calculating bandwidth of R/W to a
 *	particular file (IP on the FPGA).
 */

struct statistics
{
	int tx_bytes;
	int rx_bytes;
	unsigned long seconds;
	unsigned long ns;
	int cdma_attempt;
	int ip_not_ready;
	int dma_usage_cnt;
};

// ********************** support functions **************************

int xdma_init_sv(struct xdma_dev *lro);


/**
 * @brief This function is used to initiate a CDMA Transfer. It requires
 * a locked CDMA resource an AXI Start Address, AXI Destination Address,
 * and a Keyhole setting.
 * @param SA 64bit Starting Address of DMA transfer.
 * @param DA 64bit Destination Address of DMA transfer.
 * @param BTT Number of bytes to transfer.
 * @param keyhole_en Instructs the CDMA to to a keyhole transaction or not
 * @param cdma_num Instructs which CDMA to use (Assumes it has been locked)
*/
int cdma_transfer(struct file_desc * file_desc, u64 l_sa, u64 l_da, u32 l_btt, int keyhole_en, int cdma_num);

/**
 * @brief This function determines whether the axi peripheral can be read from/written to
 * directly or if it needs to use the DMAs. Once it decides it calls the apporpiate functions
 * to transfer data.
 * @param axi_address 64b AXI address to act on.
 * @param buf the system memory address to act on if direct R/W is selected.
 * @param count The amount of data to R/W
 * @param transfer_type determines R/W or R/W with keyhole.
 * @param dma_offset The DMA offset of memory region if DMA transfer is selected.
*/
int dma_transfer(struct file_desc * file_desc, u64 axi_address, void *buf, size_t count, int transfer_type, u64 dma_offset);

/**
 * @brief This function is used to acknowledge a CDMA transaction. It will check
 * for any failures.
 * @param cdma_num Instructs which CDMA to use (Assumes it has been locked)
*/
int cdma_ack(int cdma_num, u64 sa, u64 da, u32 btt);
/**
 * @brief This asserts or deasserts hte keyhole setting in the CDMA register.
 * @param cdma_num Instructs which CDMA to use (Assumes it has been locked)
 * @param bit_vec 32b bit vector to set the CDMA register
 * @param set (1) or unset (0) the bits.
 * @param cdma_num Instructs which CDMA to use (Assumes it has been locked)
*/
int cdma_config_set(u32 bit_vec, int set_unset, int cdma_num);
/**
 * @brief This function issues a read from the host to the desired axi address.
 * @param axi_address 64b AXI address to read from.
 * @param buf the address of store the read data
 * @param count the number of bytes to read
 * @param transfer_type determines keyhole read or regular read
*/
int direct_read(u64 axi_address, void *buf, size_t count, int transfer_type);
/**
 * @brief This function issues a write from the host to the desired axi address.
 * @param axi_address 64b AXI address to write to.
 * @param buf the address to write the data from
 * @param count the number of bytes to write
 * @param transfer_type determines keyhole write or regular write
*/
int direct_write(u64 axi_address, void *buf, size_t count, int transfer_type);

/**
 * @brief This function is used for interrupt processing. it returns the integer value of the
 * least significant set bit position.
 * @param vec The 32b vector to convert to an integer (converts to integer of least sig set bit)
*/
int vec2num(u32 vec);
/**
 * @brief This function is used for interrupt processing. it returns the one-hot vector value
 * of the input num as a position.
 * @param num converts this number to one hot vector
*/
u32 num2vec(int num);
/**
 * @brief This function initializes the CDMA.
 * @param cdma_num Instructs which CDMA to use (Assumes it has been locked)
 * @param cdma_address the AXI address of the CDMA
 * @param dma_addr_base Used to set the dma translation address to the PCIe control reg.
*/
int cdma_init(struct sv_mod_dev *svd, int cdma_num, uint cdma_address);
/**
 * @brief This function writes the translation address (The DMA Hardware base address) to the
 * PCIe control register.
 * @param axi_address The 64b AXI address of the PCIe Control interface (set through insmod).
*/
int pcie_ctl_init(u64 axi_address, u64 dma_addr_base);
/**
 * @brief This function initialized the interrupt controller in the FPGA.
 * @param axi_address The 64b AXI address of the Interrupt Controller (set through insmod).
*/
void axi_intc_init(struct sv_mod_dev *svd, uint axi_address);

/**
 * @brief This function initialized the load on demand controller in the FPGA.
 * @param axi_address The 64b AXI address of the LOD Controller (set through insmod).
*/
void axi_lodc_init(struct sv_mod_dev *svd, uint axi_address);

/**
* @brief This function initialized a HLS block setup for NODMA
* @param file_desc the struct containing all the file variables.
*/
int hls_block_init(struct file_desc * file_desc);

/**
* @brief This function activates the load on demand controller for a secion in the FPGA.
* @param file_desc the struct containing all the file variables.
*/
void axi_lodc_activate(struct file_desc * file_desc);

/**
* @brief This function deactivates the load on demand controller for a secion in the FPGA.
* @param file_desc the struct containing all the file variables.
*/
void axi_lodc_deactivate(struct file_desc * file_desc);


/**
* @brief This function deinitialized the interrupt controller in the FPGA.
* @param axi_address The 64b AXI address of the Interrupt Controller (set through insmod).
*/
void axi_intc_deinit(struct sv_mod_dev *svd);

/**
 * @brief This function allocates the DMA regions for the peripheral.
 * @param file_desc the struct containing all the file variables.
 * @param dma_buffer_base the current DMA allocated region offset to be assigned
 * @param dma_buffer_size The size of the DMA buffer.
*/
int dma_file_init(struct file_desc *file_desc, char *dma_buffer_base, int xdma);

/**
 * @brief This function performs calls appropriate functions for Reading from the AXI Streaming FIFO and puts the data into the ring buffer.
 * @param count The number of bytes to read from the AXI streaming FIFO.
 * @param file_desc The struct containing all the file variables.
 * @param ring_pointer_offset The current offset of the ring pointer in memory to store data.
*/
size_t axi_stream_fifo_read(struct file_desc * file_desc, size_t count, void * buffer_addr, u64 hw_base_addr, int ring_pointer_offset, size_t buf_size);
/**
 * @brief This function performs calls appropriate functions for Reading from the AXI Streaming FIFO and copies it into the dma area to be copied to user.
 * It does not use the dma area as a ring buffer
 * @param count The number of bytes to read from the AXI streaming FIFO.
 * @param file_desc The struct containing all the file variables.
 * @param ring_pointer_offset The current offset of the ring pointer in memory to store data.
*/
size_t axi_stream_fifo_read_direct(struct file_desc * file_desc, size_t count, char * buf_base_addr, size_t buf_size);

/**
 * @brief This function initializes the AXI Streaming FIFO.
 * @param file_desc The struct containing all the file variables.
*/

/**
 * This function sets the initial values for the ring buffer
 *
 */
void ring_buffer_init(struct file_desc * file_desc);

int axi_stream_fifo_init(struct file_desc * file_desc);
/**
 * @brief This function deinitializes the AXI Streaming FIFO.
 * @param file_desc The struct containing all the file variables.
*/
int axi_stream_fifo_deinit(struct file_desc * file_desc);

/**
 * @brief This function continuously polls the CDMA until the completion bit is read.
 * @param cdma_num Instructs which CDMA to use (Assumes it has been locked)
*/
void cdma_idle_poll(int cdma_num);
/**
 * @brief This function operates on it's own thread after insmod. It is used to handle all AXI-Streaming
 * write transfers. It reads from the write ring buffers and writes the data to hardware.
 * @param write_fifo the kernel FIFO data structure for the global write FIFO.
*/
int write_thread(void *in_param);
/**
 * @brief This function operates on it's own thread after insmod. It is used to handle all AXI-Streaming
 * read transfers. It reads from the hardware and stores data in the Read Ring Buffer. It wakes up the
 * poll method for the file upon receiving new data.
 * @param read_fifo the kernel FIFO data structure for the global read FIFO.
*/
int read_thread(void *in_param);
/**
 * @brief This function is called by the driver to create the write thread.
 * @param file_desc The struct containing all the file variables.
*/
struct task_struct* create_thread_write(struct sv_mod_dev * svd);
/**
 * @brief This function is called by the driver to create the read thread.
 * @param read_fifo the kernel FIFO data structure for the global read FIFO.
*/
struct task_struct* create_thread_read(struct sv_mod_dev * svd);
/**
 * @brief This function is called by the write thread to write data to the FPGA.
 * This function performs calls appropriate functions for writing to the AXI Streaming FIFO.
 * @param file_desc The struct containing all the file variables.
*/
int write_data(struct file_desc* file_desc, void * buffer_addr);
/**
 * @brief This function is will update a ring pointer given the amount of bytes written or read.
 * @param bytes_written The number of bytes to advance the ring pointer
 * @param ring_pointer_offset The current ring pointer offset.
 * @param file_size The size of the ring buffer to handle wrap around cases.
*/
int get_new_ring_pointer(int bytes_written, int ring_pointer_offset, int file_size, int dma_byte_width);
/**
 * @brief This function returns the amount of space available in the ring buffer
 * @param file_desc The struct containing all the file variables.
 * @param tail The ring pointer offset at the tail of the ring buffer (starting point)
 * @param head The ring pointer offset at the head of the ring buffer (ending point)
 * @param priorty setting this to 1 gives priority if head==tail.
*/
int room_in_buffer(int head, int tail, int full, size_t dma_size);
/**
 * @brief This function returns the amount of data available in the ring buffer
 * @param file_desc The struct containing all the file variables.
 * @param tail The ring pointer offset at the tail of the ring buffer (starting point)
 * @param head The ring pointer offset at the head of the ring buffer (ending point)
 * @param priorty setting this to 1 gives priority if head==tail.
*/
int data_in_buffer(int head, int tail, int full, size_t dma_size);
/**
 * @brief This function checks if there is any available data to be read from an axi stream fifo.
 * @param file_desc The struct containing all the file variables.
*/
size_t axi_stream_fifo_d2r(struct file_desc * file_desc);
/**
 * @brief This function is called by the read thread to read data from the FPGA.
*/
//int read_data(struct file_desc * file_desc);
int read_data(struct file_desc * file_desc, int read_size, void * buffer_addr);


int clear_fifo_isr(struct file_desc * file_desc);

bool is_packet_full_interrupt(struct file_desc * file_desc);



struct sv_mod_dev *alloc_sv_dev_instance(u64 dma_size);

int copy_to_ring_buffer(struct file_desc * file_desc, void* buf, size_t count, void * buffer_addr);

int copy_from_ring_buffer(struct file_desc * file_desc, void* buf, size_t count, void * buffer_addr);

int dma_file_deinit(struct file_desc *file_desc);

int align_dma(int addr, int dma_byte_width);

int sv_map_single_bar(struct bar_info *bars, struct xdma_dev *lro, struct pci_dev *dev, int idx);

int sv_map_bars(struct bar_info *bars, struct pci_dev *dev);

void sv_unmap_bars(struct bar_info *bars, struct pci_dev *dev);
// ******************************************************************
#endif //SV_DRIVER_H
