#include <linux/ioctl.h>
#include <linux/types.h>
/* include early, to verify it depends only on the headers above */
#include "xdma-ioctl.h"
#include "xdma-core.h"
#include "xdma-sgm.h"
#include "xbar_sys_parameters.h"
#include "version.h"

#include "sv_driver.h"


/* map_bars() -- map device regions into kernel virtual address space
 *
 * Map the device memory regions into kernel virtual address space after
 * verifying their sizes respect the minimum sizes needed
 */
int sv_xdma_map_bars(struct xdma_dev *lro, struct bar_info *bars, struct pci_dev *dev)
{
	int rc;
	int i;
	int bar_id_list[XDMA_BAR_NUM];
	int bar_id_idx = 0;
	int config_bar_pos = 0;

	/* iterate through all the BARs */
	for (i = 0; i < XDMA_BAR_NUM; i++) {
		int bar_len;

		bar_len = sv_map_single_bar(lro, bars, dev, i);
		if (bar_len == 0) {
			continue;
		} else if (bar_len < 0) {
			rc = -1;
			goto fail;
		}

		/* Try to identify BAR as XDMA control BAR */
		if ((bar_len >= XDMA_BAR_SIZE) && (lro->config_bar_idx < 0)) {

			if (is_config_bar(lro, i)) {
				lro->config_bar_idx = i;
				config_bar_pos = bar_id_idx;

				bars->pci_bar_addr[i] = bars->pci_bar_end[i];

			}
		}

		bar_id_list[bar_id_idx] = i;
		bar_id_idx++;
	}

	/* The XDMA config BAR must always be present */
	if (lro->config_bar_idx < 0) {
		dbg_init("Failed to detect XDMA config BAR\n");
		rc = -1;
		goto fail;
	}

	identify_bars(lro, bar_id_list, bar_id_idx, config_bar_pos);

	/* successfully mapped all required BAR regions */
	rc = 0;
	goto success;
fail:
	/* unwind; unmap any BARs that we did map */
	unmap_bars(lro, dev);
success:
	return rc;
}

int sv_map_single_bar(struct xdma_dev *lro, struct bar_info *bars, struct pci_dev *dev, int idx)
{
	resource_size_t bar_start;
	resource_size_t bar_len;
	resource_size_t map_len;

	bar_start = pci_resource_start(dev, idx);
	bar_len = pci_resource_len(dev, idx);
	map_len = bar_len;

	lro->bar[idx] = NULL;

	/* do not map BARs with length 0. Note that start MAY be 0! */
	if (!bar_len) {
		dbg_init("BAR #%d is not present - skipping\n", idx);
		return 0;
	}

	/* BAR size exceeds maximum desired mapping? */
	if (bar_len > INT_MAX) {
		dbg_init("Limit BAR %d mapping from %llu to %d bytes\n", idx, (u64)bar_len, INT_MAX);
		map_len = (resource_size_t)INT_MAX;
	}
	/*
	 * map the full device memory or IO region into kernel virtual
	 * address space
	 */
	dbg_init("BAR%d: %llu bytes to be mapped.\n", idx, (u64)map_len);
	lro->bar[idx] = pci_iomap(dev, idx, map_len);

	if (!lro->bar[idx]) {
		dbg_init("Could not map BAR %d", idx);
		return -1;
	}

	dbg_init("BAR%d at 0x%llx mapped at 0x%p, length=%llu(/%llu)\n", idx,
		(u64)bar_start, lro->bar[idx], (u64)map_len, (u64)bar_len);

		bars->pci_bar_end[idx] = bars->pci_bar_addr[idx]+map_len;
		bars->pci_bar_vir_addr[idx] = lro->bar[idx];		 //hardware base virtual address
		bars->num_bars++;

		dbg_init("[sv_map_single_bar]: pci bar %d addr is:0x%lx \n", idx, bars->pci_bar_vir_addr[idx]);
		dbg_init("[sv_map_single_bar]: pci bar %d start is:0x%lx end is:0x%lx\n", idx, bars->pci_bar_addr[idx], bars->pci_bar_end[idx]);
		dbg_init("[sv_map_single_bar]: pci bar %d size is:0x%lx\n", idx, map_len);

	return (int)map_len;
}


/* char_sgdma_read_write() -- Read from or write to the device
 *
 * @buf userspace buffer
 * @count number of bytes in the userspace buffer
 * @pos byte-address in device
 * @dir_to_device If !0, a write to the device is performed
 *
 * Iterate over the userspace buffer, taking at most 255 * PAGE_SIZE bytes for
 * each DMA transfer.
 *
 * For each transfer, get the user pages, build a sglist, map, build a
 * descriptor table. submit the transfer. wait for the interrupt handler
 * to wake us on completion.
 */


ssize_t sv_char_sgdma_read_write(struct xdma_char * lro_char, char __user *buf, size_t count, loff_t *pos, int dir_to_dev)
{
	int rc;
	ssize_t res = 0;
	int counter;
	int seq = counter++;

	struct xdma_dev *lro;
	struct xdma_engine *engine;


	/* fetch device specific data stored earlier during open */
	//lro_char = (struct xdma_char *)file->private_data;

	BUG_ON(!lro_char);
	BUG_ON(lro_char->magic != MAGIC_CHAR);



	dbg_tfr("\t\t[sv_char_sgdma_read_write]: buf %p \n", buf);
	dbg_tfr("\t\t[sv_char_sgdma_read_write]: count %x \n", lro_char);
	dbg_tfr("\t\t[sv_char_sgdma_read_write]: pos %p \n", pos);
	dbg_tfr("\t\t[sv_char_sgdma_read_write]: lro_char %x \n", dir_to_dev);


	dbg_tfr("\t\t[sv_char_sgdma_read_write]: lro_char %p \n", lro_char);


	lro = lro_char->lro;
	 BUG_ON(!lro);
	BUG_ON(lro->magic != MAGIC_DEVICE);

	dbg_tfr("\t\t[sv_char_sgdma_read_write]: lro %p \n", lro);


	engine = lro_char->engine;
	dbg_tfr("\t\t[sv_char_sgdma_read_write]: engine %p \n", engine);


	/* XXX detect non-supported directions XXX */
	BUG_ON(!engine);
	BUG_ON(engine->magic != MAGIC_ENGINE);

	dbg_tfr("seq:%d buf=0x%p, count=%lld, pos=%llu\n", seq,
		buf, (s64)count, (u64)*pos);

	dbg_tfr("seq:%d dir_to_dev=%d %s request\n", seq, dir_to_dev,
		dir_to_dev ? "write" : "read");

	dbg_tfr("%s engine channel %d (engine num %d)= 0x%p\n", engine->name,
		engine->channel, engine->number_in_channel, engine);
	dbg_tfr("lro = 0x%p\n", lro);

	/* data direction does not match engine? */
	if (dir_to_dev != engine->dir_to_dev) {
		if (dir_to_dev)
			dbg_tfr("FAILURE: Cannot write to C2H engine.\n");
		else
			dbg_tfr("FAILURE: Cannot read from H2C engine.\n");

		return -EINVAL;
	}

	rc = check_transfer_align(engine, buf, count, *pos, 1);
	if (rc) {
		dbg_tfr("Invalid transfer alignment detected\n");
		return rc;
	}

	dbg_tfr("res = %ld, remaining = %ld\n", res, count);

	res = transfer_data(engine, (char *)buf, count, pos, seq);
	dbg_tfr("seq:%d char_sgdma_read_write() return=%lld.\n", seq, (s64)res);

	interrupt_status(lro);

	return res;
}


int sv_do_addrmode_set(struct xdma_engine *engine, unsigned long dst)
{
	int rc;
	u32 w = XDMA_CTRL_NON_INCR_ADDR;

	dbg_perf("IOCTL_XDMA_ADDRMODE_SET\n");

	engine->non_incr_addr = !!dst;
	if (engine->non_incr_addr)
		write_register(w, &engine->regs->control_w1s);
	else
		write_register(w, &engine->regs->control_w1c);


	engine_alignments(engine);

	return rc;
}




//things to copy
//
// int msix_irq_setup(struct xdma_dev *lro)
// {
// 	int i;
// 	int rc;
//
// 	BUG_ON(!lro);
// 	write_msix_vectors(lro);
//
// 	for (i = 0; i < MAX_USER_IRQ; i++) {
// 		rc = request_irq(lro->entry[i].vector, xdma_user_irq, 0,
// 			DRV_NAME, &lro->user_irq[i]);
//
// 		if (rc) {
// 			dbg_init("Couldn't use IRQ#%d, rc=%d\n",
// 				lro->entry[i].vector, rc);
// 			break;
// 		}
//
// 		dbg_init("Using IRQ#%d with 0x%p\n", lro->entry[i].vector,
// 			&lro->user_irq[i]);
// 	}
//
// 	/* If any errors occur, free IRQs that were successfully requested */
// 	if (rc) {
// 		while (--i >= 0)
// 			free_irq(lro->entry[i].vector, &lro->user_irq[i]);
// 		return -1;
// 	}
//
// 	lro->irq_user_count = MAX_USER_IRQ;
//
// 	return 0;
// }
//
// int irq_setup(struct xdma_dev *lro, struct pci_dev *pdev)
// {
// 	int rc = 0;
// 	u32 irq_flag;
// 	u8 val;
// 	void *reg;
// 	u32 w;
//
// 	BUG_ON(!lro);
//
// 	if (lro->msix_enabled) {
// 		rc = msix_irq_setup(lro);
// 	} else {
// 		if (!lro->msi_enabled){
// 			pci_read_config_byte(pdev, PCI_INTERRUPT_PIN, &val);
// 			dbg_init("Legacy Interrupt register value = %d\n", val);
// 			if (val > 1) {
// 				val--;
// 				w = (val<<24) | (val<<16) | (val<<8)| val;
// 				// Program IRQ Block Channel vactor and IRQ Block User vector with Legacy interrupt value
// 				reg = lro->bar[lro->config_bar_idx] + 0x2080;   // IRQ user
// 				write_register(w, reg);
// 				write_register(w, reg+0x4);
// 				write_register(w, reg+0x8);
// 				write_register(w, reg+0xC);
// 				reg = lro->bar[lro->config_bar_idx] + 0x20A0;   // IRQ Block
// 				write_register(w, reg);
// 				write_register(w, reg+0x4);
// 			}
// 		}
// 		irq_flag = lro->msi_enabled ? 0 : IRQF_SHARED;
// 		lro->irq_line = (int)pdev->irq;
// 		rc = request_irq(pdev->irq, xdma_isr, irq_flag, DRV_NAME, lro);
// 		if (rc)
// 			dbg_init("Couldn't use IRQ#%d, rc=%d\n", pdev->irq, rc);
// 		else
// 			dbg_init("Using IRQ#%d with 0x%p\n", pdev->irq, lro);
// 	}
//
// 	return rc;
// }
//
// int engine_msix_setup(struct xdma_engine *engine)
// {
// 	int rc = 0;
// 	u32 vector;
// 	struct xdma_dev *lro;
//
// 	BUG_ON(!engine);
// 	lro = engine->lro;
// 	BUG_ON(!lro);
//
// 	vector = lro->entry[lro->engines_num + MAX_USER_IRQ].vector;
//
// 	dbg_init("Requesting IRQ#%d for engine %p\n", vector, engine);
// 	rc = request_irq(vector, xdma_channel_irq, 0, DRV_NAME, engine);
// 	if (rc) {
// 		dbg_init("Unable to request_irq for engine %d\n",
// 			lro->engines_num);
// 	} else {
// 		dbg_init("Requested IRQ#%d for engine %d\n", vector,
// 			lro->engines_num);
// 		engine->msix_irq_line = vector;
// 	}
//
// 	return rc;
// }
