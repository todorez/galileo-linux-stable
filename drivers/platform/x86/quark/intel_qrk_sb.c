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
 * Intel Quark side-band driver
 *
 * Thread-safe sideband read/write routine.
 *
 * Author : Bryan O'Donoghue <bryan.odonoghue@linux.intel.com> 2012
 */

#include <linux/errno.h>
#include <linux/intel_qrk_sb.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/spinlock.h>
#include <linux/pci.h>
#include "intel_qrk_imr.h"

#define INTEL_QRK_SB_CMD_ADDR	(0x000000D0)
#define INTEL_QRK_SB_DATA_ADDR	(0x000000D4)

#define INTEL_QRK_SB_MCR_SHIFT	(24)
#define INTEL_QRK_SB_PORT_SHIFT	(16)
#define INTEL_QRK_SB_REG_SHIFT	(8)
#define INTEL_QRK_SB_BYTEEN	(0xF0)	/* enable all 32 bits */

/* Simple structure for module */
struct intel_qrk_sb_dev{
	struct pci_dev * pdev;
	spinlock_t slock;
	u8 initialized;
};

static struct intel_qrk_sb_dev sb_dev = {
	.initialized = 0
};

/* Dependant drivers */
static struct platform_device pdev [] = {
	{
		.name = "intel-qrk-esram",
	},
	{
		.name = "intel-qrk-ecc",
	},
	{
		.name = "intel-qrk-thrm",
	},
};

/**
 * intel_qrk_sb_read_reg
 *
 * @param qrk_sb_id: Sideband identifier
 * @param command: Command to send to destination identifier
 * @param reg: Target register w/r to qrk_sb_id
 * @return nothing
 *
 * Utility function to allow thread-safe read of side-band
 * command - can be different read op-code types - which is why we don't
 * hard-code this value directly into msg
 */
void intel_qrk_sb_read_reg(qrk_sb_id id, u8 cmd, u8 reg, u32 *data, u8 lock)
{
	u32 msg = (cmd << INTEL_QRK_SB_MCR_SHIFT) | 
		  ((id << INTEL_QRK_SB_PORT_SHIFT) & 0xFF0000)| 
		  ((reg << INTEL_QRK_SB_REG_SHIFT) & 0xFF00)|
		  INTEL_QRK_SB_BYTEEN;

	if(data == NULL)
		return;

	if (likely(lock == 1)) {
		spin_lock(&sb_dev.slock);
	}

	pci_write_config_dword(sb_dev.pdev, INTEL_QRK_SB_CMD_ADDR, msg);
	pci_read_config_dword(sb_dev.pdev, INTEL_QRK_SB_DATA_ADDR, data);

	if(likely(lock == 1)){
		spin_unlock(&sb_dev.slock);
	}

}
EXPORT_SYMBOL(intel_qrk_sb_read_reg);

/**
 * intel_qrk_sb_write_reg
 *
 * @param qrk_sb_id: Sideband identifier
 * @param command: Command to send to destination identifier
 * @param reg: Target register w/r to qrk_sb_id
 * @return nothing
 *
 * Utility function to allow thread-safe write of side-band
 */
void intel_qrk_sb_write_reg(qrk_sb_id id, u8 cmd, u8 reg, u32 data, u8 lock)
{
	u32 msg = (cmd << INTEL_QRK_SB_MCR_SHIFT) | 
		  ((id << INTEL_QRK_SB_PORT_SHIFT) & 0xFF0000)| 
		  ((reg << INTEL_QRK_SB_REG_SHIFT) & 0xFF00)|
		  INTEL_QRK_SB_BYTEEN;

	if(likely(lock == 1)){
		spin_lock(&sb_dev.slock);
	}

	pci_write_config_dword(sb_dev.pdev, INTEL_QRK_SB_DATA_ADDR, data);
	pci_write_config_dword(sb_dev.pdev, INTEL_QRK_SB_CMD_ADDR, msg);
	
	if(likely(lock == 1)){
		spin_unlock(&sb_dev.slock);
	}
}
EXPORT_SYMBOL(intel_qrk_sb_write_reg);

/**
 * intel_qrk_sb_runfn_lock
 *
 * @param fn: Callback function - which requires side-band spinlock and !irq
 * @param arg: Callback argument
 * @return 0 on success < 0 on failure
 *
 * Runs the given function pointer inside of a call to the local spinlock using
 * spin_lock_irqsave/spin_unlock_irqrestore. Needed for the eSRAMv1 driver to
 * guarantee atomicity, but, available to any other user of sideband provided
 * rules are respected.
 * Rules:
 *	fn may not sleep
 *	fn may not change the state of irqs	
 */
int intel_qrk_sb_runfn_lock(int (*fn)( void * arg ), void * arg)
{
	unsigned long flags = 0;
	int ret = 0;

	if(unlikely(fn == NULL)){
		return -EINVAL;
	}
	
	/* Get spinlock with IRQs off */
	spin_lock_irqsave(&sb_dev.slock, flags);

	/* Run function atomically */
	ret = fn(arg);

	/* Release lock */
	spin_unlock_irqrestore(&sb_dev.slock, flags);

	return ret;
}
EXPORT_SYMBOL(intel_qrk_sb_runfn_lock);

/**
 * sb_probe
 *
 * @param dev: the PCI device matching
 * @param id: entry in the match table
 * @return 0
 *
 * Callback from PCI layer when dev/vendor ids match.
 * Sets up necessary resources
 */
static int intel_qrk_sb_probe(struct pci_dev *dev, const struct pci_device_id *id)
{
	int i = 0;

	/* Init struct */
	memset(&sb_dev, 0x00, sizeof(sb_dev));

	/* Hook device */
	sb_dev.pdev = dev;

	/* Init locking structures */
	spin_lock_init(&sb_dev.slock);

	/* Set state */
	sb_dev.initialized = 1;

	/* Register side-band sub-ordinate drivers */
	for (i = 0; i < sizeof(pdev)/sizeof(struct platform_device); i++){
		/* Register side-band sub-ordinate drivers */
		platform_device_register(&pdev[i]);
	}
	pr_info("Intel Quark side-band driver registered\n");

	/* Switch off boot-time IMRs nice and early */
	return intel_qrk_imr_init(dev->device);
}

/**
 * sb_remove
 *
 * @param pdev: PCI device
 * @return nothing
 *
 * Callback from PCI sub-system upon PCI dev removal
 */
static void intel_qrk_sb_remove(struct pci_dev *pdev)
{
}

/* Quark hardware */
struct pci_device_id intel_qrk_sb_ids[] = {
        { PCI_VDEVICE(INTEL, PCI_DEVICE_ID_X1000_HOST_BRIDGE), 0},
        { PCI_VDEVICE(INTEL, 0x12C0), 0},
        { 0 }
};

MODULE_DEVICE_TABLE(pci, intel_qrk_sb_ids);

/* PCI callbacks */
static struct pci_driver intel_qrk_sb_driver = {
	.name = "intel_qrk_sb",
	.id_table = intel_qrk_sb_ids,
	.probe = intel_qrk_sb_probe,
	.remove = intel_qrk_sb_remove,
};

/**
 * intel_qrk_sb_init
 *
 * Module entry point
 */
static int __init intel_qrk_sb_init(void)
{
	return pci_register_driver(&intel_qrk_sb_driver);
}

MODULE_AUTHOR("Bryan O'Donoghue <bryan.odonoghue@linux.intel.com>");
MODULE_DESCRIPTION("Intel Quark SOC side-band driver");
MODULE_LICENSE("Dual BSD/GPL");

/* Initialise early since other drivers eSRAM, DRAM ECC and thermal depend */
subsys_initcall(intel_qrk_sb_init);
