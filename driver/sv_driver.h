/**
 * System View Device Driver

 * @date 11/10/2015

 * @author	System View Inc.

 * @file sv_driver.h

 * @brief			This driver is to be used to control the
 *					 Xilinx interface IP created by System View Inc.
 ***********************************************************
 */

#include <linux/kthread.h>
#include <linux/kfifo.h>

/*These are the CDMA R/W types */
#ifndef KEYHOLE_WRITE
#define KEYHOLE_WRITE 2
#endif

#ifndef NORMAL_WRITE
#define NORMAL_WRITE 0
#endif

#ifndef KEYHOLE_READ
#define KEYHOLE_READ 3
#endif

#ifndef NORMAL_READ
#define NORMAL_READ 1
#endif

#ifndef CDMA
#define CDMA 3
#endif

//#define BACK_PRESSURE 1
#define RING_BUFF_SIZE_MULTIPLIER 2
/********* printk statements *********/
#define verbose_printk printk
//#define verbose_data_xfer_printk printk
//#define verbose_cdma_printk printk
//#define verbose_dma_printk printk
//#define verbose_cdmaq_printk printk
//#define verbose_dmaq_printk printk
//#define verbose_axi_fifo_read_printk printk
//#define verbose_axi_fifo_write_printk printk
//#define verbose_isr_printk printk
//#define verbose_poll_printk printk
//#define very_verbose_poll_printk printk
//#define verbose_axi_fifo_d2r_printk printk
//#define verbose_direct_write_printk printk
//#define verbose_direct_read_printk printk
//#define verbose_llseek_printk printk
//#define verbose_pci_read_printk printk
//#define verbose_pci_write_printk printk
//#define verbose_read_thread_printk printk
//#define verbose_write_thread_printk printk


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
#ifndef verbose_cdmaq_printk
#define verbose_cdmaq_printk(...)
#endif
#ifndef verbose_dmaq_printk
#define verbose_dmaq_printk(...)
#endif
#ifndef verbose_data_xfer_printk
#define verbose_data_xfer_printk(...)
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
#ifndef verbose_read_thread_printk
#define verbose_read_thread_printk(...)
#endif
#ifndef verbose_write_thread_printk
#define verbose_write_thread_printk(...)
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
#define CLEAR_AXI_INTERRUPT_CTLR 60	/**< IOCTL Magic Number */
#define SET_CDMA_KEYHOLE_WRITE 58	/**< IOCTL Magic Number */
#define SET_CDMA_KEYHOLE_READ 59	/**< IOCTL Magic Number */
#define SET_MODE 62			/**< IOCTL Magic Number */
#define SET_INTERRUPT 61	/**< IOCTL Magic Number */
#define SET_AXI_CTL_DEVICE 63		/**< IOCTL Magic Number */
#define SET_DMA_SIZE 64		/**< IOCTL Magic Number */
#define RESET_DMA_ALLOC 65	/**< IOCTL Magic Number */
#define SET_FILE_SIZE 66	/**< IOCTL Magic Number */
#define GET_FILE_STATISTICS 67		/**< IOCTL Magic Number */
#define GET_DRIVER_STATISTICS 70	/**< IOCTL Magic Number */
#define START_FILE_TIMER 68	/**< IOCTL Magic Number */
#define STOP_FILE_TIMER 69	/**< IOCTL Magic Number */
#define START_DRIVER_TIMER 71		/**< IOCTL Magic Number */
#define STOP_DRIVER_TIMER 72		/**< IOCTL Magic Number */

#define ERROR	-1
#define SUCCESS 0

/* MODE Types */
#define SLAVE 0				/**< Mode Type : This is standard memory interface */
#define AXI_STREAM_FIFO 1	/**< Mode Type : This is for axi streaming peripherals with no last singal */
#define MASTER 2			/**< Mode Type : This is for AXI Master devices (currently not supported) */
#define AXI_STREAM_PACKET 4 /**< Mode Type : This is for axi streaming peripherals with a last signal */
//#define CDMA 3

//max number of CDMAs
#define CDMA_MAX_NUM			5
#define BAR_MAX_NUM			5
#define PCIE_RESOURCES_MAX_NUM			6




#define MAX_NUM_MASTERS 2
#define MAX_NUM_SLI 4 // Max number of slaves with interrupts
#define MAX_NUM_INT MAX_NUM_MASTERS + MAX_NUM_SLI

/*These are the Driver types that are matched through insmod parameter "driver_type" */
#define PCI 1	 /**< Driver Type */
#define PLATFORM 2 /**< Driver Type */
#define AWS 5		/**< Driver Type */

enum xfer_type {
	HOST_READ = 1,
	HOST_WRITE= 2*HOST_READ,
	INC_SA	 = 2*HOST_WRITE,
	INC_DA	 = 2*INC_SA,
	INC_BOTH = INC_DA|INC_SA
};

/******************************** AWS XDMA related **********************************/
#include "libxdma.h"
#include "libxdma_api.h"
extern uint pcie_use_xdma;
extern struct xdma_dev *xdma_dev_s;
extern int xdma_num_channels;
extern xdma_channel_tuple* xdma_channel_list;
#define XDMA_TIMEOUT_IN_MSEC				(3 * 1000)
extern dma_addr_t dma_addr_base; /**< The hardware DMA Allocation Address */

/******************************** Xilinx Register Offsets **********************************/

extern const u32 AXI_STREAM_ISR;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
extern const u32 AXI_STREAM_IER;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
extern const u32 AXI_STREAM_TDFR;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
extern const u32 AXI_STREAM_TDFV;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
extern const u32 AXI_STREAM_TDFD;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
extern const u32 AXI_STREAM_TLR;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
extern const u32 AXI_STREAM_RDFR;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
extern const u32 AXI_STREAM_RDFO;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
extern const u32 AXI_STREAM_RDFD;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
extern const u32 AXI_STREAM_RLR;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
extern const u32 AXI_STREAM_SRR;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
extern const u32 AXI_STREAM_TDR;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
extern const u32 AXI_STREAM_RDR;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
extern const u32 AXI_STREAM_TXID;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
extern const u32 AXI_STREAM_TXUSER;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */
extern const u32 AXI_STREAM_RXID;	 /**< AXI Streaming FIFO Register Offset (See Xilinx Doc) */

extern const u32 CDMA_CR;	 /**< CDMA Register Offset (See Xilinx Doc) */
extern const u32 CDMA_SR;	 /**< CDMA Register Offset (See Xilinx Doc) */
extern const u32 CDMA_DA;	 /**< CDMA Register Offset (See Xilinx Doc) */
extern const u32 CDMA_DA_MSB;	 /**< CDMA Register Offset (See Xilinx Doc) */
extern const u32 CDMA_SA;	 /**< CDMA Register Offset (See Xilinx Doc) */
extern const u32 CDMA_SA_MSB;	 /**< CDMA Register Offset (See Xilinx Doc) */
extern const u32 CDMA_BTT;	 /**< CDMA Register Offset (See Xilinx Doc) */

extern const u32 AXIBAR2PCIEBAR_0L;	 /**< AXI PCIe Subsystem Offset (See Xilinx Doc) */
extern const u32 AXIBAR2PCIEBAR_1L;	 /**< AXI PCIe Subsystem Offset (See Xilinx Doc) */

extern const u32 INT_CTRL_IER; /**< Interrupt Controller Register Offset, see Xilinx doc. */
extern const u32 INT_CTRL_MER;	 /**< Interrupt Controller Register Offset, see Xilinx doc. */
extern const u32 INT_CTRL_ISR;	 /**< Interrupt Controller Register Offset, see Xilinx doc. */
extern const u32 INT_CTRL_IAR;	 /**< Interrupt Controller Register Offset, see Xilinx doc. */

/********************************************************************************************/


extern int dma_max_write_size ;	 /**< AWS PCI/e max write size	*/
extern int dma_max_read_size ;	 /**< AWS PCI/e max read size	*/


/* Shared Global Variables */
typedef unsigned int uint;

extern u64 axi_pcie_ctl;	/**< This variable is set at insmod to hold the PCIe control port AXI Address */
extern u64 axi_interr_ctrl; /**< This variable is set at insmod to hold the Interrupt Controller AXI Address */
extern u64 axi_pcie_m;
extern int dma_byte_width;

extern u8 cdma_set[CDMA_MAX_NUM];
extern u8 pcie_ctl_set;

extern int cdma_capable;	 /**< This variable is set by the driver if a CDMA is initialized and available for use */
extern int back_pressure;	/**< This variable is set at insmod that tells whether the read ring buffers should backpressure to HW or overwrite */

/*CDMA Semaphore*/
extern struct mutex cdma_sem[CDMA_MAX_NUM];
extern uint cdma_address[CDMA_MAX_NUM]; 		/**< Holds AXI Base address of CDMA 1 */
extern int cdma_count;
extern wait_queue_head_t cdma_q_head;
extern atomic_t cdma_q;

extern u32 dma_current_offset;
extern u32 dma_garbage_offset;
extern u32 dma_garbage_size;
/*these are used in the data_transfer function to check for out of range memory r/w */
extern unsigned long pci_bar_addr[BAR_MAX_NUM];
extern unsigned long pci_bar_end[BAR_MAX_NUM];
extern unsigned long pci_bar_size[BAR_MAX_NUM];
extern uint num_bars;

extern char * pci_bar_vir_addr[BAR_MAX_NUM];		 //hardware base virtual address

/*this is the user peripheral address offset*/
//extern u64 bar_0_axi_offset;
//extern u64 bar_1_axi_offset;

extern ulong pcie_m_address;

/*These are the interrupt and mutex wait variables */
extern wait_queue_head_t wq;
extern wait_queue_head_t wq_periph;
extern wait_queue_head_t thread_q_head_write;
extern wait_queue_head_t thread_q_head_read;
extern wait_queue_head_t pci_write_head;
/*this is the CDMA wait condition variable*/
extern int cdma_comp[CDMA_MAX_NUM];
extern atomic_t cdma_atom[CDMA_MAX_NUM];

extern atomic_t thread_q_read;
extern atomic_t thread_q_write;
extern spinlock_t fifo_lock_read;
extern spinlock_t fifo_lock_write;

extern int cdma_usage_cnt;

extern atomic_t driver_tx_bytes;
extern atomic_t driver_rx_bytes;
extern atomic_t driver_start_flag;
extern atomic_t driver_stop_flag;
extern struct timespec driver_start_time;
extern struct timespec driver_stop_time;

extern char * dma_buffer_base;
extern u32 dma_current_offset;
extern u64 dma_buffer_size;

/** Module Description Struct
 *	@brief This is the data structure that is stored inside the private section of each file
 *	struct that is created for this driver. Since each file pertains to an IP within the FPGA
 *	design, it contains characteristics of the IP suxk as AXI address, mode types, interrupt
 *	numbers, etc.
 */
struct mod_desc
{
	int minor;					 /**< The minor number of file node (mainly used for debug msgs */
	u64 axi_addr;				/**< The axi address of the file node's associated IP */
	u64 axi_addr_ctl;			/**< The axi control address for axi-streaming IPs */
	u32 mode;						 /**< The mode of the IP which defines it as axi-streaming or memory interface */
	int int_num;					 /**< The Interrupt Number of the associated IP */
	int master_num;				 /**< For future master peripherals, currently UNUSED */
	int keyhole_config;			/**< The Keyhole R/W configuration for associated IP */
	u32 interrupt_vec;			 /**< The Interrupt Vector */

	atomic_t * in_read_fifo_count;		/**< The number of times the mod_desc is in the read fifo, once it reaches 0 we can kfree it */
	atomic_t * in_write_fifo_count;		/**< The number of times the mod_desc is in the write fifo, once it reaches 0 we can kfree it */

	bool file_open;							/**< True if file is open, used to process the mod_desc (or throw it away) in read/write threads */

	size_t dma_size;				/**< The size of allocated DMA buffer */

	u32 dma_offset_read;		 /**< The Offset byte of the allocated DMA buffer for READ from dma_buffer_base */
	u32 dma_offset_write;				/**< The Offset byte of the allocated DMA buffer for WRITE from dma_buffer_base */
	char * dma_write_addr;			/**< This is the pointer to the start of DMA buffer for WRITE in virtual address space */
	char * dma_read_addr;			/**< This is the pointer to the start of DMA buffer for READ in virtual address space */

	loff_t file_size;	/**< This is the size of the axi_streaming FIFO for streaming peripherals or size of ram for memory peripherals */
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

	atomic_t * rfh;				/**< Read from Hardware pointer for READ Ring Buffer - HW DMAs data in read_data_thread*/
	atomic_t * rfu;				/**< Read from User pointer for READ Ring Buffer - user copies data into pci_read */
	atomic_t * read_ring_buf_full;	 /**< READ Ring Buffer is full, and rfu == rfh */

	spinlock_t * ring_pointer_write; /**< Spinlock variable to lock code section for updating ring pointer variables */
	spinlock_t * ring_pointer_read;	/**< Spinlock variable to lock code section for updating ring pointer variables */

	atomic_t * pci_write_q;	 /**< Used as a wait variable to wake up a sleeping pci_write function */
	spinlock_t * in_fifo_read;		/**< Spinlock variable to proect code for writing in_fifo flag*/
	spinlock_t * in_fifo_write; /**< Spinlock variable to protect code for writing in_fifo flag */
	int in_fifo_read_flag;			/**< Flag variable to tell if it already exists in the READ FIFO */
	int in_fifo_write_flag;	 /**< Flag variable to tell if it already exists in the WRITE FIFO */
	int has_interrupt_vec;	/**< This file has an interrupt associated with it */
	int axi_fifo_rlr;	/**< Last read RLR value if non-zero this has to be used */
	int axi_fifo_rdfo;	/**< Last read RDFO value if non-zero this has to be used */
	size_t read_header_size; 	/**< Last read_header_size value if non-zero this has to be used */
	wait_queue_head_t poll_wq; /**< waitq for events */

	u32 tx_dest;	/**< Last wrote RLR value if non-zero this has to be used */
	u32 rx_dest;	/**< Last read RLR value if non-zero this has to be used */

};

//DECLARE_KFIFO(read_fifo, struct mod_desc*, 4096);


extern struct mod_desc * mod_desc_arr[12];

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
	int cdma_usage_cnt;
};

/** Bar mapping structure
 */
struct bar_mapping {
	resource_size_t bar_start;
	resource_size_t bar_len;
	resource_size_t map_len;
	int is_config_bar;
	void *__iomem	bar;
};

// ********************** support functions **************************

int xdma_init_sv(int num_channels);
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
int cdma_transfer(u64 SA, u64 DA, u32 BTT, int keyhole_en, int cdma_num);
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
 * @brief This function determines whether the axi peripheral can be read from/written to
 * directly or if it needs to use the DMAs. Once it decides it calls the apporpiate functions
 * to transfer data.
 * @param axi_address 64b AXI address to act on.
 * @param buf the system memory address to act on if direct R/W is selected.
 * @param count The amount of data to R/W
 * @param transfer_type determines R/W or R/W with keyhole.
 * @param dma_offset The DMA offset of memory region if DMA transfer is selected.
*/
int data_transfer(u64 axi_address, void *buf, size_t count, int transfer_type, u64 dma_offset);
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
 * @brief This function determines if a CDMA is available. if it is, it locks the semaphore and returns
 * the CDMA number.
*/
int cdma_query(void);
/**
 * @brief This function initializes the CDMA.
 * @param cdma_num Instructs which CDMA to use (Assumes it has been locked)
 * @param cdma_address the AXI address of the CDMA
 * @param dma_addr_base Used to set the dma translation address to the PCIe control reg.
*/
int cdma_init(int cdma_num, uint cdma_address);
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
void int_ctlr_init(u64 axi_address);
/**
 * @brief This function allocates the DMA regions for the peripheral.
 * @param mod_desc the struct containing all the file variables.
 * @param dma_buffer_base the current DMA allocated region offset to be assigned
 * @param dma_buffer_size The size of the DMA buffer.
*/
int dma_file_init(struct mod_desc *mod_desc, char * dma_buffer_base, u64 dma_buffer_size);
/**
 * @brief This function performs calls appropriate functions for Reading from the AXI Streaming FIFO and puts the data into the ring buffer.
 * @param count The number of bytes to read from the AXI streaming FIFO.
 * @param mod_desc The struct containing all the file variables.
 * @param ring_pointer_offset The current offset of the ring pointer in memory to store data.
*/
size_t axi_stream_fifo_read(size_t count, char * buf_base_addr, u64 hw_base_addr, struct mod_desc * mod_desc, int ring_pointer_offset, size_t buf_size);
/**
 * @brief This function performs calls appropriate functions for Reading from the AXI Streaming FIFO and copies it into the dma area to be copied to user.
 * It does not use the dma area as a ring buffer
 * @param count The number of bytes to read from the AXI streaming FIFO.
 * @param mod_desc The struct containing all the file variables.
 * @param ring_pointer_offset The current offset of the ring pointer in memory to store data.
*/
size_t axi_stream_fifo_read_direct(size_t count, char * buf_base_addr, u64 hw_base_addr, struct mod_desc * mod_desc, size_t buf_size);

/**
 * @brief This function initializes the AXI Streaming FIFO.
 * @param mod_desc The struct containing all the file variables.
*/
int axi_stream_fifo_init(struct mod_desc * mod_desc);
/**
 * @brief This function deinitializes the AXI Streaming FIFO.
 * @param mod_desc The struct containing all the file variables.
*/
int axi_stream_fifo_deinit(struct mod_desc * mod_desc);

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
 * @param mod_desc The struct containing all the file variables.
*/
struct task_struct* create_thread_write(struct kfifo * write_fifo);
/**
 * @brief This function is called by the driver to create the read thread.
 * @param read_fifo the kernel FIFO data structure for the global read FIFO.
*/
struct task_struct* create_thread_read(struct kfifo * read_fifo);
/**
 * @brief This function is called by the write thread to write data to the FPGA.
 * This function performs calls appropriate functions for writing to the AXI Streaming FIFO.
 * @param mod_desc The struct containing all the file variables.
*/
int write_data(struct mod_desc* mod_desc);
/**
 * @brief This function is will update a ring pointer given the amount of bytes written or read.
 * @param bytes_written The number of bytes to advance the ring pointer
 * @param ring_pointer_offset The current ring pointer offset.
 * @param file_size The size of the ring buffer to handle wrap around cases.
*/
int get_new_ring_pointer(int bytes_written, int ring_pointer_offset, int file_size);
/**
 * @brief This function returns the amount of space available in the ring buffer
 * @param mod_desc The struct containing all the file variables.
 * @param tail The ring pointer offset at the tail of the ring buffer (starting point)
 * @param head The ring pointer offset at the head of the ring buffer (ending point)
 * @param priorty setting this to 1 gives priority if head==tail.
*/
int room_in_buffer(int head, int tail, int full, size_t dma_size);
/**
 * @brief This function returns the amount of data available in the ring buffer
 * @param mod_desc The struct containing all the file variables.
 * @param tail The ring pointer offset at the tail of the ring buffer (starting point)
 * @param head The ring pointer offset at the head of the ring buffer (ending point)
 * @param priorty setting this to 1 gives priority if head==tail.
*/
int data_in_buffer(int head, int tail, int full, size_t dma_size);
/**
 * @brief This function checks if there is any available data to be read from an axi stream fifo.
 * @param mod_desc The struct containing all the file variables.
*/
size_t axi_stream_fifo_d2r(struct mod_desc * mod_desc);
/**
 * @brief This function is called by the read thread to read data from the FPGA.
*/
//int read_data(struct mod_desc * mod_desc);
int read_data(struct mod_desc * mod_desc, int read_size);
// ******************************************************************
