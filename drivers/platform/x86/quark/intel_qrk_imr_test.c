/*
 * Copyright(c) 2013 Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
/*
 * Intel Quark IMR Test module
 *
 */

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/intel_qrk_sb.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/slab.h>
#include "intel_qrk_imr.h"

#define DRIVER_NAME			"intel_qrk_imr_test"

/**
 * XXX intel_qrk_sb.h needs to be updated with SB_ID_PUNIT and change
 * propagated. This is a workaround to make it look less ugly. */
#define SB_ID_PUNIT			SB_ID_THERMAL

/* Memory-mapped SPI Flash address */
#define ILB_SPIFLASH_BASEADDR			0xFF800000
/* PUnit DMA block transfer size, in bytes */
#define SPI_DMA_BLOCK_SIZE			512

/**************************** Exported to LISA *******************************/

/*
 * Internally-used ioctl code. At the moment it is not reserved by any mainline
 * driver.
 */
#define IMR_TEST_IOCTL_CODE			0xE1

/*
 * Integers for ioctl operation.
 */
#define IOCTL_QRK_SANITY_CHECK_PUNIT_DMA	_IO(IMR_TEST_IOCTL_CODE, 0x00)
#define IOCTL_QRK_IMR_1				_IO(IMR_TEST_IOCTL_CODE, 0x01)

/*****************************************************************************/

/**
 * struct intel_qrk_imr_dev
 *
 * Structure to represent module state/data/etc
 */
struct intel_qrk_imr_test_dev {
	unsigned int opened;
	struct platform_device *pldev;	/* Platform device */
	struct cdev cdev;
	struct mutex open_lock;
};

static struct intel_qrk_imr_test_dev imr_test_dev;
static struct class *imr_test_class;
static DEFINE_MUTEX(imr_test_mutex);
static int imr_test_major;

/* PUnit DMA registers over side-band */
#define PUNIT_SPI_DMA_COUNT_REG		0x60
#define PUNIT_SPI_DMA_DEST_REG		0x61
#define PUNIT_SPI_DMA_SRC_REG		0x62

/**
 * ilb_spi_dma_read 
 *
 * @param src: physical address in Legacy SPI Flash
 * @param dst: physical address of destination
 * @param dma_block_count: number of 512B SPI Flash blocks to be transferred
 *
 * Read-access iLB SPI via PUnit DMA engine.
 * 
 */
static void ilb_spi_dma_read(u32 *src, u32 *dst, u32 dma_block_count)
{
	pr_info("%s: src=%p, dst=%p, count=%u\n", __func__, src, dst,
		dma_block_count);

	/* Setup source and destination addresses. */
	intel_qrk_sb_write_reg(SB_ID_PUNIT, CFG_WRITE_OPCODE,
		PUNIT_SPI_DMA_SRC_REG, (u32) src, 0);
	intel_qrk_sb_write_reg(SB_ID_PUNIT, CFG_WRITE_OPCODE,
		PUNIT_SPI_DMA_DEST_REG, (u32) dst, 0);

	pr_info("%s: starting transaction\n", __func__);

	/*
	 * Setup the number of block to be copied over. Transaction will start
	 * as soon as the register is filled with value.
	 */
	intel_qrk_sb_write_reg(SB_ID_PUNIT, CFG_WRITE_OPCODE,
		PUNIT_SPI_DMA_COUNT_REG, dma_block_count, 0);

	/* Poll for completion. */
	while (dma_block_count > 0) {
		intel_qrk_sb_read_reg(SB_ID_PUNIT, CFG_READ_OPCODE,
			PUNIT_SPI_DMA_COUNT_REG, &dma_block_count, 0); 
	}

	pr_info("%s: transaction completed\n", __func__);
}

/**
 * punit_dma_sanity_check 
 *
 * @return 0 if success, 1 if failure
 *
 * Perform a basic sanity check for PUnit DMA engine. Copy over a 512B SPI
 * Flash block. 
 */
static int punit_dma_sanity_check(void)
{
	int err = 0;
	u32 *buffer = NULL;
	u32 buf_ph_addr = 0;

	/* Allocate 512B buffer for 1 SPI Flash block */
	buffer = kzalloc(SPI_DMA_BLOCK_SIZE, GFP_KERNEL);
        if (!buffer) {
		err = -ENOMEM;
		goto end;
	}

	/* DMA first SPI Flash block into buffer */
	buf_ph_addr = (u32)virt_to_phys(buffer);
	ilb_spi_dma_read((u32 *)ILB_SPIFLASH_BASEADDR, (u32 *)buf_ph_addr, 1);

	kfree(buffer);
end:
	return err;
}

/**
 * imr_violate_kernel_punit_dma 
 *
 * @return always 0
 *
 * PUnit-DMA access to the Uncompressed Kernel IMR.
 * This is based on set_imr_kernel_data() in intel_qrk_imr.c. Find the physical
 * address of .text section and copy a 512B chunk of legacy SPI via PuUnit DMA.
 * 
 */
static int imr_violate_kernel_punit_dma(void)
{
	extern unsigned long _text;
	u32 kernel_text = (u32)virt_to_phys(&_text);

	/* We expect this to trigger an IMR violation reset */
	ilb_spi_dma_read((u32 *)ILB_SPIFLASH_BASEADDR, (u32 *)kernel_text, 1);

	/*
	 * If we're still alive, we have a serious bug:
	 * - we didn't appropriately target the IMR?
	 * - if we have, weren't we prevented from accessing?
	 * - if we weren't prevented, it's unlikely we're alive with a dirty
	 *   text section
	 */
	pr_err("%s: BUG: still running after DMAing into kernel text!?\n",
		__func__);

	return 0;
}

/*
 * File ops
 */
static long imr_test_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int ret = -EINVAL;

	switch (cmd) {
		case IOCTL_QRK_SANITY_CHECK_PUNIT_DMA:
			/* Check PUnit DMA actually works */
			ret = punit_dma_sanity_check();
			break;
		case IOCTL_QRK_IMR_1:
			/* Kernel IMR violation: PUnit DMA access */
			ret = imr_violate_kernel_punit_dma();
			break;
		default:
			break;
	}

	return ret;
}

static int imr_test_open(struct inode *inode, struct file *file)
{
	mutex_lock(&imr_test_mutex);
	nonseekable_open(inode, file);

	if (mutex_lock_interruptible(&imr_test_dev.open_lock)) {
		mutex_unlock(&imr_test_mutex);
		return -ERESTARTSYS;
	}

	if (imr_test_dev.opened) {
		mutex_unlock(&imr_test_dev.open_lock);
		mutex_unlock(&imr_test_mutex);
		return -EINVAL;
	}

	imr_test_dev.opened++;
	mutex_unlock(&imr_test_dev.open_lock);
	mutex_unlock(&imr_test_mutex);
	return 0;
}

static int imr_test_release(struct inode *inode, struct file *file)
{
	mutex_lock(&imr_test_dev.open_lock);
	imr_test_dev.opened = 0;
	mutex_unlock(&imr_test_dev.open_lock);

	return 0;
}

static const struct file_operations imr_test_file_ops = {
	.open = imr_test_open,
	.release = imr_test_release,
	.unlocked_ioctl = imr_test_ioctl,
	.llseek = no_llseek,
};

/**
 * intel_qrk_imr_test_probe
 *
 * @param pdev: Platform device
 * @return 0 success < 0 failure
 *
 * Callback from platform sub-system to probe
 */
static int intel_qrk_imr_test_probe(struct platform_device * pdev)
{
	int retval = 0;
	unsigned int minor = 0;

	mutex_init(&imr_test_dev.open_lock);
	cdev_init(&imr_test_dev.cdev, &imr_test_file_ops);
	imr_test_dev.cdev.owner = THIS_MODULE;

	retval = cdev_add(&imr_test_dev.cdev, MKDEV(imr_test_major, minor), 1);
	if (retval) {
		printk(KERN_ERR "chardev registration failed\n");
		return -EINVAL;
	}
	if (IS_ERR(device_create(imr_test_class, NULL,
				 MKDEV(imr_test_major, minor), NULL,
				 "imrtest%u", minor))){
		dev_err(&pdev->dev, "can't create device\n");
		return -EINVAL;
	}

	return 0;

}

static int intel_qrk_imr_test_remove(struct platform_device * pdev)
{
	unsigned int minor = MINOR(imr_test_dev.cdev.dev);

	device_destroy(imr_test_class, MKDEV(imr_test_major, minor));
	cdev_del(&imr_test_dev.cdev);

	class_destroy(imr_test_class);

	return 0;
}

/*
 * Platform structures useful for interface to PM subsystem
 */
static struct platform_driver intel_qrk_imr_test_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.remove = intel_qrk_imr_test_remove,
};

/**
 * intel_qrk_imr_test_init
 *
 * Load module.
 */
static int __init intel_qrk_imr_test_init(void)
{
	int retval = 0;
	dev_t dev;

	imr_test_class = class_create(THIS_MODULE,"qrk_imr_test");
	if (IS_ERR(imr_test_class)) {
		retval = PTR_ERR(imr_test_class);
		printk(KERN_ERR "imr_test: can't register imr_test class\n");
		goto err;
	}

	retval = alloc_chrdev_region(&dev, 0, 1, "imr_test");
	if (retval) {
		printk(KERN_ERR "earam_test: can't register character device\n");
		goto err_class;
	}
	imr_test_major = MAJOR(dev);

	memset(&imr_test_dev, 0x00, sizeof(imr_test_dev));
	imr_test_dev.pldev = platform_create_bundle(
		&intel_qrk_imr_test_driver, intel_qrk_imr_test_probe, NULL, 0, NULL, 0);

	if(IS_ERR(imr_test_dev.pldev)){
		printk(KERN_ERR "platform_create_bundle fail!\n"); 
		retval = PTR_ERR(imr_test_dev.pldev);
		goto err_class;
	}

	return 0;

err_class:
	class_destroy(imr_test_class);
err:
	return retval;
}

static void __exit intel_qrk_imr_test_exit(void)
{
	platform_device_unregister(imr_test_dev.pldev);
	platform_driver_unregister(&intel_qrk_imr_test_driver);
}

module_init(intel_qrk_imr_test_init);
module_exit(intel_qrk_imr_test_exit);

MODULE_AUTHOR("Josef Ahmad <josef.ahmad@intel.com>");
MODULE_DESCRIPTION("Quark IMR test module");
MODULE_LICENSE("Dual BSD/GPL");

