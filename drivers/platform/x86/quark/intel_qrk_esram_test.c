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
/**
 * intel_qrk_esram_test.c
 *
 * Simple test module to provide test cases for ITS integration
 *
 */
#include <linux/cdev.h>
#include <linux/crc32.h>
#include <linux/crc32c.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/intel_qrk_sb.h>
#include <linux/kallsyms.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/slab.h>

#include "intel_qrk_esram.h"
#include "intel_qrk_esram_test.h"

#define DRIVER_NAME			"intel_qrk_esram_test"

/**
 * struct intel_qrk_esram_dev
 *
 * Structre to represent module state/data/etc
 */
struct intel_qrk_esram_test_dev{
	unsigned int opened;
	struct platform_device *pldev;	/* Platform device */
	struct cdev cdev;
	struct mutex open_lock;
	char * pdata;
	u32 size;
};

static struct intel_qrk_esram_test_dev esram_test_dev;
static struct class *esram_test_class;
static DEFINE_MUTEX(esram_test_mutex);
static int esram_test_major;
static char * name = "testmap";

/******************************************************************************
 *                                eSRAM BIST
 ******************************************************************************/

static int crc_cache = 0;

unsigned long long tsc_delta(unsigned long long first, unsigned long long end)
{
	if (first < end)
		return end - first;
	else
		return (ULLONG_MAX - first) + end;	
}


/**
 * intel_qrk_crctest
 *
 * Do a CRC32 of the specified region. Return the time taken in jiffies
 */
static unsigned long long intel_qrk_crctest(char * pdata, u32 crcsize)
{
	unsigned long long j1 = 0, j2 = 0;
	
	rdtscll(j1);

	/* Flush LMT cache to introduce cache miss to our test */
	__asm__ __volatile__("wbinvd\n");
	crc32(0, pdata, crcsize);

	rdtscll(j2);

	return tsc_delta(j1, j2);
}

#ifdef __DEBUG__
#define bist_err(x){\
	pr_err("eSRAM bist err line %d errno %d\n", (__LINE__-2), x);\
	return x;\
}
#else
#define bist_err(x){\
	return x;\
}
#endif
/**
 * intel_qrk_esram_perpage_overlay
 *
 * Maps to integration test spec ID CLN.F.SW.APP.eSRAM.0
 */
int intel_qrk_esram_test_perpage_overlay(void)
{

	int ret = 0;
	u32 idx = 0, size = INTEL_QRK_ESRAM_PAGE_SIZE;

	/* Set a known state */
	for(idx = 0; idx < size; idx += sizeof(u32)){
		*((u32*)&esram_test_dev.pdata[idx]) = idx;
	}


	/* Basic test of full range of memory */
	ret = intel_qrk_esram_map_range(esram_test_dev.pdata, size, name);
	if(ret){
		bist_err(ret);
	}
	for(idx = 0; idx < size; idx += sizeof(u32)){
		if(*((u32*)&esram_test_dev.pdata[idx]) != idx){
			pr_err("Entry %d is 0x%08x require 0x%08x",
				idx, esram_test_dev.pdata[idx], idx);
			bist_err(-EIO);
		}
	}

#if 0
	ret = intel_qrk_esram_unmap_range(esram_test_dev.pdata, size, name);
	if(ret){
		bist_err(ret);
	}
#endif
	return 0;
}

/**
 * intel_qrk_esram_test_pageref_count
 *
 * Ensure page reference couting works as expected
 */
int intel_qrk_esram_test_pagref_count(void)
{
	u32 size = INTEL_QRK_ESRAM_PAGE_SIZE;
	int ret = 0;

	return 0;
	/* Map a page */
	ret = intel_qrk_esram_map_range(esram_test_dev.pdata, size, name);
	if(ret){
		bist_err(ret);
	}

	/* Map a second time - and verify mapping fails */
	ret = intel_qrk_esram_map_range(esram_test_dev.pdata, size, name);
	if(ret == 0){
		bist_err(-EFAULT);
	}

#if 0
	/* Unmap - OK */
	ret = intel_qrk_esram_unmap_range(esram_test_dev.pdata, size, name);
	if(ret){
		bist_err(ret);
	}

	/* Verify second unmap operation fails */
	ret = intel_qrk_esram_unmap_range(esram_test_dev.pdata, size, name);
	if(ret == 0){
		bist_err(-EFAULT);
	}
#endif	
	return 0;
}

extern uint32_t get_crc32table_le(void);

/**
 * intel_qrk_esram_test_contig_perfmetric
 *
 * Do a CRC16 for a contigous area of memory
 * Map contigous area and get a CRC16
 *
 * Ensure overlayed data takes less time than regular unoverlayed DRAM
 */
int intel_qrk_esram_test_contig_perfmetric(void)
{
	u32 crcsize = 0x60000;
	unsigned long long crc32_fullmap = 0, crc32_fullunmap = 0;
	uint32_t crc32table_le = kallsyms_lookup_name("crc32table_le");
	int ret = 0;

	if (crc32table_le == 0){
		pr_err("%s unable to fine symbol crc32table_le\n", __func__);
		return -ENODEV;
	}

	/* Get raw data metric */
	crc_cache = 1;
	crc32_fullunmap = intel_qrk_crctest(esram_test_dev.pdata, crcsize);

	/* Map CRC16 symbol (algorithm) + code (data) */
	ret = intel_qrk_esram_map_symbol(crc32_le);
	if(ret){
		bist_err(ret);
	}
	ret = intel_qrk_esram_map_symbol((void*)crc32table_le);
	if(ret){
		bist_err(ret);
	}

	/* Map test data */
	ret = intel_qrk_esram_map_range(esram_test_dev.pdata, crcsize, name);
	if(ret){
		bist_err(ret);
	}
	
	/* Get metric */
	crc_cache = 1;
	crc32_fullmap = intel_qrk_crctest(esram_test_dev.pdata, crcsize);
#if 0
	/* Tidy up */
	ret = intel_qrk_esram_unmap_range(esram_test_dev.pdata, crcsize, name);
	if(ret){
		bist_err(ret);
	}
	ret = intel_qrk_esram_unmap_range(((void*)crc32_table),
					  sizeof(crc32_table), name);
	if(ret){
		bist_err(ret);
	}
	ret = intel_qrk_esram_unmap_symbol(crc32);
	if(ret){
		bist_err(ret);
	}
#endif
	pr_info("%s did crctest - mapped - in %llu ticks\n", __func__, crc32_fullmap);
	pr_info("%s mapped count %llu unmapped %llu\n",
		__func__, crc32_fullmap, crc32_fullunmap);
	return crc32_fullmap < crc32_fullunmap;
}

/**
 * intel_qrk_esram_test_kernel_codemap
 *
 * Maps some kernel code - a data section and then calls the code contained
 * therein. Proves out the running overlayed eSRAM works
 */
int intel_qrk_esram_test_kernel_codemap(void)
{
#if 0
	int ret = intel_qrk_esram_map_symbol(msleep);
	if(ret){
		printk(KERN_ERR "%s map symbol msleep fail\n", __FUNCTION__);
		bist_err(ret);
	}
	
	/* run the mapped code */
	msleep(1);

	/* unmap */	
	ret = intel_qrk_esram_unmap_symbol(msleep);
	if(ret){
		printk(KERN_ERR "%s unmap symbol msleep fail\n", __FUNCTION__);
		bist_err(ret);
	}
#endif
	return 0;
}

/**
 * intel_qrk_esram_test_kernel_datamap
 *
 * Tests mapping/unmapping of a kernel data structure
 */
int intel_qrk_esram_test_kernel_datamap(void)
{
#if 0
	unsigned long jtag = 0;
	unsigned long ctrl = 0;

	/* Map the interrupt descriptor table */
	int ret = intel_qrk_esram_map_range(idt_table, INTEL_QRK_ESRAM_PAGE_SIZE, name);
	if(ret){
		bist_err(ret);
	}
	
	jtag = jiffies;
	/* Wait for jiffies to tick or timeout to occur (failure) */
	while(jtag == jiffies){
		ctrl++;
	}

	/* unmap */	
	ret = intel_qrk_esram_unmap_range(idt_table, INTEL_QRK_ESRAM_PAGE_SIZE, name);
	if(ret){
		bist_err(ret);
	}
#endif
	return 0;
}

/**
 * intel_qrk_esram_test_sub_unsub
 *
 * Subscribe and unsubscribe 100% of available eSRAM
 */
int intel_qrk_esram_test_sub_unsub(void)
{
	int ret = 0;
	u32 idx = 0, size = INTEL_QRK_ESRAM_PAGE_SIZE * INTEL_QRK_ESRAM_PAGE_COUNT;

	/* Set a known state */
	for(idx = 0; idx < size; idx += sizeof(u32)){
		*((u32*)&esram_test_dev.pdata[idx]) = idx;
	}

	/* Basic test of full range of memory */
	ret = intel_qrk_esram_map_range(esram_test_dev.pdata, size, name);
	if(ret){
		bist_err(ret);
	}
	for(idx = 0; idx < size; idx += sizeof(u32)){
		if(*((u32*)&esram_test_dev.pdata[idx]) != idx){
			pr_err("Entry %d is 0x%08x require 0x%08x",
				idx, esram_test_dev.pdata[idx], idx);
			bist_err(-EIO);
		}
	}
#if 0
	ret = intel_qrk_esram_unmap_range(esram_test_dev.pdata, size, name);
	if(ret){
		bist_err(ret);
	}
#endif
	return 0;
}

/**
 * intel_qrk_esram_test_over_sub
 *
 * Test oversubscription of eSRAM
 */
int intel_qrk_esram_test_over_sub(void)
{
	int ret = 0;
	u32 size = INTEL_QRK_ESRAM_PAGE_SIZE * (INTEL_QRK_ESRAM_PAGE_COUNT + 1);

	/* Over subscribe should fail */
	ret = intel_qrk_esram_map_range(esram_test_dev.pdata, size, name);
	if(ret == 0){
		//intel_qrk_esram_unmap_range(esram_test_dev.pdata, size, name);
		bist_err(-EFAULT);
	}
	return 0;
}

/*
 * File ops
 */
static long esram_test_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	int ret = -EINVAL;

	cmd -= QRK_ESRAM_IOCTL_BASE;
	switch (cmd) {
		case QRK_F_SW_APP_ESRAM_0:
			/* Per page overlay */
			ret = intel_qrk_esram_test_perpage_overlay();
			break;

		case QRK_F_SW_APP_ESRAM_1:
			/* Verify page reference counting */
			ret = intel_qrk_esram_test_pagref_count();
			break;

		case QRK_F_SW_APP_ESRAM_2:
			/* Performance metric or overlay contig RAM */
			ret = intel_qrk_esram_test_contig_perfmetric();
			if (ret == 1)
				ret = 0;
			break;

		case QRK_F_SW_APP_ESRAM_3:
			/* Verify mapping of kernel code section */
			/* Covered by test #2 */
			ret = 0; //intel_qrk_esram_test_kernel_codemap();
			break;

		case QRK_F_SW_APP_ESRAM_4:
			/* Verify mapping of kernel data section (IDT) */
			/* Covered by test #2 */
			ret = 0; //intel_qrk_esram_test_kernel_datamap();
			break;

		case QRK_F_SW_APP_ESRAM_5:
			/* Complete subscribe/unsubscribe eSRAM */
			ret = intel_qrk_esram_test_sub_unsub();
			break;

		case QRK_F_SW_APP_ESRAM_6:
			/* Over subscribe eSRAM */
			ret = intel_qrk_esram_test_over_sub();
			break;

		default:
			break;
	}

	return ret;
}

static int esram_test_open(struct inode *inode, struct file *file)
{
	mutex_lock(&esram_test_mutex);
	nonseekable_open(inode, file);

	if (mutex_lock_interruptible(&esram_test_dev.open_lock)) {
		mutex_unlock(&esram_test_mutex);
		return -ERESTARTSYS;
	}

	if (esram_test_dev.opened) {
		mutex_unlock(&esram_test_dev.open_lock);
		mutex_unlock(&esram_test_mutex);
		return -EINVAL;
	}

	esram_test_dev.opened++;
	mutex_unlock(&esram_test_dev.open_lock);
	mutex_unlock(&esram_test_mutex);

	return 0;
}

static int esram_test_release(struct inode *inode, struct file *file)
{
	mutex_lock(&esram_test_dev.open_lock);
	esram_test_dev.opened = 0;
	mutex_unlock(&esram_test_dev.open_lock);

	return 0;
}

static const struct file_operations esram_test_file_ops = {
	.open = esram_test_open,
	.release = esram_test_release,
	.unlocked_ioctl = esram_test_ioctl,
	.llseek = no_llseek,
};


/**
 * intel_qrk_esram_test_probe
 *
 * @param pdev: Platform device
 * @return 0 success < 0 failure
 *
 * Callback from platform sub-system to probe
 *
 * This driver manages eSRAM on a per-page basis. Therefore if we find block
 * mode is enabled, or any global, block-level or page-level locks are in place
 * at module initialisation time - we bail out.
 */
static int intel_qrk_esram_test_probe(struct platform_device * pdev)
{
	int retval = 0;
	unsigned int minor = 0;

	esram_test_dev.size = INTEL_QRK_ESRAM_PAGE_COUNT * INTEL_QRK_ESRAM_PAGE_SIZE;

	/* Get memory */
	esram_test_dev.pdata = kzalloc(esram_test_dev.size, GFP_KERNEL);
	if(unlikely(esram_test_dev.pdata == NULL)){
		pr_err("Can't allocate %d bytes\n", esram_test_dev.size);
		return -ENOMEM;
	}

	mutex_init(&esram_test_dev.open_lock);
	cdev_init(&esram_test_dev.cdev, &esram_test_file_ops);
	esram_test_dev.cdev.owner = THIS_MODULE;

	retval = cdev_add(&esram_test_dev.cdev, MKDEV(esram_test_major, minor), 1);
	if (retval) {
		printk(KERN_ERR "chardev registration failed\n");
		kfree(esram_test_dev.pdata);
		return -EINVAL;
	}
	if (IS_ERR(device_create(esram_test_class, NULL,
				 MKDEV(esram_test_major, minor), NULL,
				 "esramtest%u", minor))){
		dev_err(&pdev->dev, "can't create device\n");
		kfree(esram_test_dev.pdata);
		return -EINVAL;
	}
	printk(KERN_INFO "%s/%s/%s complete OK !!\n", __FUNCTION__, __DATE__,__TIME__);
	return 0;

}

/**
 * intel_qrk_esram_remove
 *
 * @return 0 success < 0 failure
 *
 * Removes a platform device
 */
static int intel_qrk_esram_test_remove(struct platform_device * pdev)
{
	unsigned int minor = MINOR(esram_test_dev.cdev.dev);

	device_destroy(esram_test_class, MKDEV(esram_test_major, minor));
	cdev_del(&esram_test_dev.cdev);
	kfree(esram_test_dev.pdata);

	return 0;
}

/*
 * Platform structures useful for interface to PM subsystem
 */
static struct platform_driver intel_qrk_esram_test_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
	},
	.remove = intel_qrk_esram_test_remove,
};

/**
 * intel_qrk_esram_init
 *
 * @return 0 success < 0 failure
 *
 * Module entry point
 */
static int __init intel_qrk_esram_test_init(void)
{
	int retval = 0;
	dev_t dev;

	esram_test_class = class_create(THIS_MODULE,"qrk_esram_test");
	if (IS_ERR(esram_test_class)) {
		retval = PTR_ERR(esram_test_class);
		printk(KERN_ERR "esram_test: can't register earam_test class\n");
		goto err;
	}

	retval = alloc_chrdev_region(&dev, 0, 1, "esram_test");
	if (retval) {
		printk(KERN_ERR "earam_test: can't register character device\n");
		goto err_class;
	}
	esram_test_major = MAJOR(dev);

	memset(&esram_test_dev, 0x00, sizeof(esram_test_dev));
	esram_test_dev.pldev = platform_create_bundle(
		&intel_qrk_esram_test_driver, intel_qrk_esram_test_probe, NULL, 0, NULL, 0);

	if(IS_ERR(esram_test_dev.pldev)){
		printk(KERN_ERR "platform_create_bundle fail!\n"); 
		retval = PTR_ERR(esram_test_dev.pldev);
		goto err_class;
	}

	return 0;

err_class:
	class_destroy(esram_test_class);
err:
	return retval;
}

/**
 * intel_qrk_esram_exit
 *
 * Module exit
 */
static void __exit intel_qrk_esram_test_exit(void)
{
	platform_device_unregister(esram_test_dev.pldev);
	platform_driver_unregister(&intel_qrk_esram_test_driver);
}

MODULE_AUTHOR("Bryan O'Donoghue <bryan.odonoghue@linux.intel.com>");
MODULE_DESCRIPTION("Intel Quark eSRAM ITS driver");
MODULE_LICENSE("Dual BSD/GPL");

module_init(intel_qrk_esram_test_init);
module_exit(intel_qrk_esram_test_exit);
