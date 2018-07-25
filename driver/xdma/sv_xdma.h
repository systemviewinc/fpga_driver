#ifndef SV_XDMA_H
#define SV_XDMA_H
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
#include <linux/ioctl.h>
#include <linux/types.h>

#include "xdma-core.h"
#include "../sv_driver.h"



//forward declations
struct bar_info;
struct mod_desc;
struct file_desc;


int sv_xdma_map_single_bar(struct xdma_dev *lro, struct bar_info *bars, struct pci_dev *dev, int idx);
int sv_xdma_map_bars(struct xdma_dev *lro, struct bar_info *bars, struct pci_dev *dev);
void sv_xdma_unmap_bars(struct xdma_dev *lro, struct bar_info *bars, struct pci_dev *dev);
ssize_t sv_char_sgdma_read_write(struct file_desc * file_desc, struct xdma_char * lro_char, char __user *buf, size_t count, loff_t *pos, int dir_to_dev);
int sv_do_addrmode_set(struct xdma_engine *engine, unsigned long non_incr_mode);
void sv_identify_bars(struct xdma_dev *lro, int *bar_id_list, int num_bars, int config_bar_pos);

// ******************************************************************

#endif /* SV_XDMA_H */
