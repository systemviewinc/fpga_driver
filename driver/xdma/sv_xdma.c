#include <linux/ioctl.h>
#include <linux/types.h>
/* include early, to verify it depends only on the headers above */
#include "xdma-ioctl.h"
#include "xdma-core.h"
#include "xdma-sgm.h"
#include "xbar_sys_parameters.h"
#include "version.h"

#include "../sv_driver.h"
#include "sv_xdma.h"

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

		bar_len = sv_map_single_bar(bars, lro, dev, i);
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
	sv_xdma_unmap_bars(lro, bars, dev);
success:
	return rc;
}


/*
 * Unmap the BAR regions that had been mapped earlier using map_bars()
 */
void sv_xdma_unmap_bars(struct xdma_dev *lro, struct bar_info *bars, struct pci_dev *dev)
{
	int i;

	for (i = 0; i < XDMA_BAR_NUM; i++) {
		/* is this BAR mapped? */
		if (lro->bar[i]) {
			/* unmap BAR */
			pci_iounmap(dev, lro->bar[i]);
			/* mark as unmapped */
			lro->bar[i] = bars->pci_bar_vir_addr[i] = NULL;
		}
	}
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
ssize_t sv_char_sgdma_read_write(struct file_desc * file_desc, struct xdma_char * lro_char, char __user *buf, size_t count, loff_t *pos, int dir_to_dev)
{
	int rc;
	ssize_t res = 0;
	int seq = file_desc->svd->dma_usage_cnt++;

	struct xdma_dev *lro;
	struct xdma_engine *engine;


	/* fetch device specific data stored earlier during open */
	//lro_char = (struct xdma_char *)file->private_data;

	BUG_ON(!lro_char);
	BUG_ON(lro_char->magic != MAGIC_CHAR);



	// dbg_tfr("\t\t[sv_char_sgdma_read_write]: buf %p \n", buf);
	// dbg_tfr("\t\t[sv_char_sgdma_read_write]: count %x \n", count);
	// dbg_tfr("\t\t[sv_char_sgdma_read_write]: pos %p \n", pos);
	// dbg_tfr("\t\t[sv_char_sgdma_read_write]: lro_char %x \n", dir_to_dev);
	// dbg_tfr("\t\t[sv_char_sgdma_read_write]: lro_char %p \n", lro_char);


	lro = lro_char->lro;
	 BUG_ON(!lro);
	BUG_ON(lro->magic != MAGIC_DEVICE);

	// dbg_tfr("\t\t[sv_char_sgdma_read_write]: lro %p \n", lro);


	engine = lro_char->engine;
	// dbg_tfr("\t\t[sv_char_sgdma_read_write]: engine %p \n", engine);


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


int sv_do_addrmode_set(struct xdma_engine *engine, unsigned long non_incr_mode)
{
	u32 w = XDMA_CTRL_NON_INCR_ADDR;

	dbg_perf("IOCTL_XDMA_ADDRMODE_SET\n");

	engine->non_incr_addr = non_incr_mode;
	if (engine->non_incr_addr)
		write_register(w, &engine->regs->control_w1s);
	else
		write_register(w, &engine->regs->control_w1c);

	engine_alignments(engine);

	return 0;
}
