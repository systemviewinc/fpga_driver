
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

#define BACK_PRESSURE 1
#define RING_BUFF_SIZE_MULTIPLIER 2
/********* printk statements *********/
#ifndef verbose_printk
#define verbose_printk(...)
#endif
/******************************/

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

/* Shared Global Variables */

extern u64 axi_pcie_ctl;
extern u64 axi_interr_ctrl;
extern u64 axi_pcie_m;
extern int dma_byte_width;

extern u8 cdma_set[5];
extern u8 pcie_ctl_set;

extern int cdma_capable;

/*CDMA Semaphore*/
extern struct mutex CDMA_sem;
extern struct mutex CDMA_sem_2;
extern wait_queue_head_t cdma_q_head;
extern atomic_t cdma_q;

extern u32 dma_current_offset;
extern u32 dma_garbage_offset;
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
extern wait_queue_head_t thread_q_head;
extern wait_queue_head_t thread_q_head_read;
extern wait_queue_head_t pci_write_head;
/*this is the CDMA wait condition variable*/
extern int cdma_comp[5];
extern atomic_t cdma_atom[5];

extern atomic_t thread_q_read;
extern atomic_t thread_q;
//extern struct kfifo read_fifo;
//DECLARE_KFIFO(read_fifo, struct mod_desc*, 4096);
extern spinlock_t fifo_lock;
extern spinlock_t fifo_lock_write;

extern int cdma_usage_cnt;

extern const u32 INT_CTRL_IER;
extern const u32 INT_CTRL_MER;
extern const u32 INT_CTRL_ISR;
extern const u32 INT_CTRL_IAR;

extern atomic_t driver_tx_bytes; 
extern atomic_t driver_rx_bytes; 
extern atomic_t driver_start_flag;
extern atomic_t driver_stop_flag;
extern struct timespec driver_start_time;
extern struct timespec driver_stop_time;

extern void * dma_buffer_base;
extern u32 dma_current_offset;
extern u64 dma_buffer_size;
/*this is the module description struct*/

struct mod_desc
{
	int minor;
	u64 axi_addr;
	u64 axi_addr_ctl;
	u32 mode;
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
	int set_dma_flag;
	//struct task_struct * thread_struct_write;
	//struct task_struct * thread_struct_read;
	int thread_q;
	atomic_t * thread_q_read;
	atomic_t * wth;    //write to hardware pointer
	atomic_t * wtk;    //write to kernel pointer
	atomic_t * ring_buf_pri;    //handshake variable
	atomic_t * rfh;    //write to hardware pointer
	atomic_t * rfu;    //write to kernel pointer
	atomic_t * ring_buf_pri_read;    //handshake variable
	spinlock_t * ring_pointer_write;
	spinlock_t * ring_pointer_read;
	atomic_t * pci_write_q;    //handshake variable
	spinlock_t * in_fifo;
	spinlock_t * in_fifo_write;
	int in_fifo_flag;
	int in_fifo_write_flag;
};

//DECLARE_KFIFO(read_fifo, struct mod_desc*, 4096);


extern struct mod_desc * mod_desc_arr[12];

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
//int dma_file_init(struct mod_desc *mod_desc, int dma_file_size, void *dma_buffer_base, u64 dma_buffer_size);
int dma_file_init(struct mod_desc *mod_desc, void *dma_buffer_base, u64 dma_buffer_size);
size_t axi_stream_fifo_write(size_t count, struct mod_desc * mod_desc, u64 ring_pointer_offset);
size_t axi_stream_fifo_read(size_t count, struct mod_desc * mod_desc, u64 ring_pointer_offset);
int axi_stream_fifo_init(struct mod_desc * mod_desc);
void cdma_wait_sleep(int cdma_num);
void cdma_idle_poll(int cdma_num);
void write_thread(struct kfifo* write_fifo);
void read_thread(struct kfifo* read_fifo);
struct task_struct* create_thread(struct mod_desc *mod_desc);
struct task_struct* create_thread_read(struct kfifo* read_fifo);
//int data_to_write(struct mod_desc *mod_desc);
int write_data(struct mod_desc* mod_desc);
int get_new_ring_pointer(int bytes_written, int ring_pointer_offset, int file_size);
int query_ring_buff(struct mod_desc* mod_desc, size_t size); 
int data_to_transfer(struct mod_desc *mod_desc, int tail, int head, int priority);
size_t axi_stream_fifo_d2r(struct mod_desc * mod_desc);
int read_data(struct mod_desc * mod_desc);
int write_fifo_ready(struct mod_desc* mod_desc);
// ******************************************************************
