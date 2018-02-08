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


struct bar_info;
struct mod_desc;


int sv_xdma_map_bars(struct xdma_dev *lro, struct bar_info *bars, struct pci_dev *dev);
int sv_map_single_bar(struct xdma_dev *lro, struct bar_info *bars, struct pci_dev *dev, int idx);
ssize_t sv_char_sgdma_read_write(struct xdma_char * lro_char, char __user *buf, size_t count, loff_t *pos, int dir_to_dev);
int sv_do_addrmode_set(struct xdma_engine *engine, unsigned long dst);

// ******************************************************************

#endif /* SV_XDMA_H */
