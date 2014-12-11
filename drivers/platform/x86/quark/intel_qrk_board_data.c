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
 * Intel Quark Legacy Platform Data accessor layer
 *
 * Simple Legacy SPI flash access layer
 *
 * Author : Bryan O'Donoghue <bryan.odonoghue@linux.intel.com> 2013
 */

#include <asm/io.h>
#include <linux/dmi.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/printk.h>

#define DRIVER_NAME				"board_data"
#define PFX					"MFH: "
#define SPIFLASH_BASEADDR			0xFFF00000
#define MFH_OFFSET				0x00008000
#define PLATFORM_DATA_OFFSET			0x00010000
#define MTD_PART_OFFSET				0x00050000
#define MTD_PART_LEN				0x00040000
#define MFH_PADDING				0x1E8
#define MFH_MAGIC				0x5F4D4648
#define FLASH_SIZE				0x00400000

/* MFH types supported @ version #1 */
#define MFH_ITEM_FW_STAGE1			0x00000000
#define MFH_ITEM_FW_STAGE1_SIGNED		0x00000001
#define MFH_ITEM_FW_STAGE2			0x00000003
#define MFH_ITEM_FW_STAGE2_SIGNED		0x00000004
#define MFH_ITEM_FW_STAGE2_CONFIG		0x00000005
#define MFH_ITEM_FW_STAGE2_CONFIG_SIGNED	0x00000006
#define MFH_ITEM_FW_PARAMS			0x00000007
#define MFH_ITEM_FW_RECOVERY			0x00000008
#define MFH_ITEM_FW_RECOVERY_SIGNED		0x00000009
#define MFH_ITEM_BOOTLOADER			0x0000000B
#define MFH_ITEM_BOOTLOADER_SIGNED		0x0000000C
#define MFH_ITEM_BOOTLOADER_CONFIG		0x0000000D
#define MFH_ITEM_BOOTLOADER_CONFIG_SIGNED	0x0000000E
#define MFH_ITEM_KERNEL				0x00000010
#define MFH_ITEM_KERNEL_SIGNED			0x00000011
#define MFH_ITEM_RAMDISK			0x00000012
#define MFH_ITEM_RAMDISK_SIGNED			0x00000013
#define MFH_ITEM_LOADABLE_PROGRAM		0x00000015
#define MFH_ITEM_LOADABLE_PROGRAM_SIGNED	0x00000016
#define MFH_ITEM_BUILD_INFO			0x00000018
#define MFH_ITEM_VERSION			0x00000019

struct intel_qrk_mfh {
	u32	id;
	u32	ver;
	u32	flags;
	u32	next_block;
	u32	item_count;
	u32	boot_priority_list;
	u8	padding[MFH_PADDING];
};

struct intel_qrk_mfh_item {
	u32	type;
	u32	addr;
	u32	len;
	u32	res0;
};

struct kobject * board_data_kobj;
EXPORT_SYMBOL_GPL(board_data_kobj);

static long unsigned int flash_version_data;
static ssize_t flash_version_show(struct kobject *kobj,
                                struct kobj_attribute *attr, char *buf)
{
        return snprintf(buf, 12, "%#010lx\n", flash_version_data);
}

static struct kobj_attribute flash_version_attr =
        __ATTR(flash_version, 0644, flash_version_show, NULL);

extern int intel_qrk_plat_probe(struct resource * pres);

#define DEFAULT_BOARD "Galileo"

static struct platform_device bsp_data [] = {
	{
		.name	= "QuarkEmulation",
		.id	= -1,
	},
	{
		.name	= "ClantonPeakSVP",
		.id	= -1,
	},
	{
		.name	= "KipsBay",
		.id	= -1,
	},
	{
		.name	= "CrossHill",
		.id	= -1,
	},
	{
		.name	= "ClantonHill",
		.id	= -1,
	},
	{
		.name	= "Galileo",
		.id	= -1,
	},
	{
		.name	= "GalileoGen2",
		.id	= -1,
	},

};

/**
 * add_firmware_sysfs_entry
 *
 * Add an entry in sysfs consistent with Galileo IDE's expected location
 * covers current software versions and legacy code < Intel Galileo BIOS 0.9.0
 *
 */
static int add_firmware_sysfs_entry(const char * board_name)
{
	extern struct kobject * firmware_kobj;

	pr_info("Intel Quark Board %s Firmware Version %#010lx\n",
				board_name, flash_version_data);

	/* board_data_kobj subordinate of firmware @ /sys/firmware/board_data */
	board_data_kobj = kobject_create_and_add("board_data", firmware_kobj);
	if (!board_data_kobj) {
		pr_err(PFX"kset create error\n");
		return -ENODEV;
	}
	return sysfs_create_file(board_data_kobj, &flash_version_attr.attr);
}

/**
 * intel_qrk_board_data_init_legacy
 *
 * Module entry point for older BIOS versions
 * Allows more recent kernels to boot on Galileo boards with BIOS before release
 * 0.9.0
 */
static int __init intel_qrk_board_data_init_legacy(void)
{
	struct intel_qrk_mfh __iomem * mfh;
	struct intel_qrk_mfh_item __iomem * item;
	struct platform_device * pdev;
	u32 i;
	char * board_name = NULL;
	void __iomem * spi_data;
	int ret = 0;

	spi_data = ioremap(SPIFLASH_BASEADDR, FLASH_SIZE);
	if (!spi_data)
		return -ENODEV;

	/* get mfh and first item pointer */	
	mfh = spi_data + MFH_OFFSET;
	if (mfh->id != MFH_MAGIC){
		pr_err(PFX"Bad MFH magic want 0x%08x found 0x%08x @ 0x%p\n",
		MFH_MAGIC, mfh->id, &mfh->id);
		return -ENODEV;
	}

	pr_info(PFX"Booting on an old BIOS assuming %s board\n", DEFAULT_BOARD);
	pr_info(PFX"mfh @ 0x%p: id 0x%08lx ver 0x%08lx entries 0x%08lx\n",
		mfh, (unsigned long)mfh->id, (unsigned long)mfh->ver,
		(unsigned long)mfh->item_count);
	item = (struct intel_qrk_mfh_item __iomem *)
		&mfh->padding [sizeof(u32) * mfh->boot_priority_list];

	/* Register a default board */
	for (i = 0; i < sizeof(bsp_data)/sizeof(struct platform_device); i++){
		if (!strcmp(bsp_data[i].name, DEFAULT_BOARD)){
			board_name = (char*)bsp_data[i].name;
			platform_device_register(&bsp_data[i]);
		}
	}

	/* Register flash regions as seperate platform devices */
	for (i = 0; i < mfh->item_count; i++, item++){
		pdev = NULL;

		switch (item->type){
		case MFH_ITEM_VERSION:
			flash_version_data = item->res0;
			ret = add_firmware_sysfs_entry(board_name);
			break;
		default:
			break;
		}
	}
	iounmap(spi_data);
	return ret;
}

/**
 * intel_qrk_board_data_init_legacy
 *
 * Module entry point for older BIOS versions
 */
static int __init intel_qrk_board_data_init(void)
{
	bool found = false;
	const char * bios_version = dmi_get_system_info(DMI_BIOS_VERSION);
	const char * board_name = dmi_get_system_info(DMI_BOARD_NAME);
	int ret = 0;
	u32 i;

	/* BIOS later than version 0.9.0 contains the right DMI data */
	for (i = 0; board_name != NULL && bios_version != NULL && 
		i < sizeof(bsp_data)/sizeof(struct platform_device); i++){
	
		if (!strcmp(bsp_data[i].name, board_name)){

			/* Register board */
			platform_device_register(&bsp_data[i]);
			found = true;

			/* Galileo IDE expects this entry */
			flash_version_data = simple_strtoul(bios_version, NULL, 16);
			ret = add_firmware_sysfs_entry(bsp_data[i].name);

			break;
		}
	}

	/* For older BIOS without DMI data we read the data directly from flash */
	if (found == false){
		ret = intel_qrk_board_data_init_legacy();
	}

	return ret;
}

MODULE_AUTHOR("Bryan O'Donoghue <bryan.odonoghue@intel.com>");
MODULE_DESCRIPTION("Intel Quark SPI Data API");
MODULE_LICENSE("Dual BSD/GPL");
subsys_initcall(intel_qrk_board_data_init);

