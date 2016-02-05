
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

/********* printk statements *********/
#ifndef verbose_printk(...)
#define verbose_printk(...)
#endif
/******************************/


/* Shared Global Variables */

extern u64 axi_pcie_ctl;
extern u64 axi_interr_ctrl;
extern u64 axi_pcie_m;

extern u8 cdma_set[5];
extern u8 pcie_ctl_set;

extern int cdma_capable;

/*CDMA Semaphore*/
extern struct mutex CDMA_sem;
extern struct mutex CDMA_sem_2;

extern u32 dma_current_offset;

/*these are used in the data_transfer function to check for out of range memory r/w */
extern unsigned long pci_bar_size;          
extern unsigned long pci_bar_1_size;        
extern unsigned long pci_bar_2_size;        

extern void * pci_bar_vir_addr;        //hardware base virtual address
extern void * pci_bar_1_vir_addr;        //hardware base virtual address
extern void * pci_bar_2_vir_addr;        //hardware base virtual address

/*this is the user peripheral address offset*/
extern u64 peripheral_space_offset;
extern u64 bar_0_axi_offset;
extern u64 peripheral_space_1_offset;

extern int pcie_m_address;

/*These are the interrupt and mutex wait variables */
extern wait_queue_head_t wq;
extern wait_queue_head_t wq_periph;
extern wait_queue_head_t mutexq;

/*this is the CDMA wait condition variable*/
extern int cdma_comp[5];
extern atomic_t cdma_atom[5];

extern const u32 INT_CTRL_IER;
extern const u32 INT_CTRL_MER;
extern const u32 INT_CTRL_ISR;
extern const u32 INT_CTRL_IAR;

/*this is the module description struct*/

struct mod_desc 
{
	int minor;
	u64 axi_addr;
	u64 axi_addr_ctl;
	u32 * mode;
	int * int_count;
	int int_num;
	int master_num;
	int keyhole_config;
	u32 interrupt_vec;
	u32 dma_offset_read;
	u32 dma_offset_write;
	size_t dma_size;
	void * dma_write;
	void * dma_read;
	u32 * kernel_reg_write;
	u32 * kernel_reg_read;
	u32 dma_offset_internal_read;
	u32 dma_offset_internal_write;
	loff_t file_size;
//	wait_queue_head_t * iwq;
	struct mutex * int_count_sem;
	int tx_bytes;
	int rx_bytes;
	struct timespec * start_time;
	struct timespec * stop_time;
	int start_flag;
	int stop_flag;
	int cdma_attempt;
	int ip_not_ready;
	atomic_t * atomic_poll;
};

/*this is the interrupt structure*/
struct interr_struct
{
	u32 * mode;
	int * int_count;
//	wait_queue_head_t * iwq;
	struct mutex * int_count_sem;
	atomic_t * atomic_poll;
    	
};

extern struct interr_struct interr_dict[8];

struct statistics
{
	int tx_bytes;
	int rx_bytes;
	unsigned long seconds;
	unsigned long ns;
	int cdma_attempt;
	int ip_not_ready;
};


// ********************** support functions **************************
int cdma_transfer(u64 SA, u64 DA, u32 BTT, int keyhole_en, int cdma_num);
int cdma_ack(int cdma_num);
int cdma_config_set(u32 bit_vec, int set_unset, int cdma_num);
int direct_read(u64 axi_address, void *buf, size_t count, int transfer_type);
int direct_write(u64 axi_address, void *buf, size_t count, int transfer_type);
int data_transfer(u64 axi_address, void *buf, size_t count, int transfer_type, u64 dma_offset);
int vec2num(u32 vec);
int cdma_query(void);
u32 num2vec(int num);
int cdma_init(int cdma_num, int cdma_address, u32 dma_addr_base);
int pcie_ctl_init(u64 axi_address, u32 dma_addr_base);
void pcie_m_init(int cdma_num);
void int_ctlr_init(u64 axi_address);
int dma_file_init(struct mod_desc *mod_desc, int dma_file_size, void *dma_buffer_base, u64 dma_buffer_size);
size_t axi_stream_fifo_write(size_t count, struct mod_desc * mod_desc);
size_t axi_stream_fifo_read(size_t count, struct mod_desc * mod_desc);
void axi_stream_fifo_init(struct mod_desc * mod_desc);
void cdma_wait_sleep(int cdma_num);
void cdma_idle_poll(int cdma_num);
// ******************************************************************
