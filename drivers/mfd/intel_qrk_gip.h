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
 * Intel Quark GIP (GPIO/I2C) driver
 */

#ifndef __INTEL_QRKGIP_H__
#define __INTEL_QRKGIP_H__

#include <linux/i2c.h>
#include <linux/mfd/intel_qrk_gip_pdata.h>
#include <linux/pci.h>
#include "../i2c/busses/i2c-designware-core.h"

/* PCI BAR for register base address */
#define GIP_I2C_BAR		0
#define GIP_GPIO_BAR		1

/**
 * intel_qrk_gpio_probe
 *
 * @param pdev: Pointer to GIP PCI device
 * @return 0 success < 0 failure
 *
 * Perform GPIO-specific probing on behalf of the top-level GIP driver.
 */
int intel_qrk_gpio_probe(struct pci_dev *pdev);

/**
 * intel_qrk_gpio_remove
 *
 * @param pdev: Pointer to GIP PCI device
 *
 * Perform GPIO-specific resource release on behalf of the top-level GIP driver.
 */
void intel_qrk_gpio_remove(struct pci_dev *pdev);

/**
 * intel_qrk_gpio_isr
 *
 * @param irq: IRQ number to be served
 * @param dev_id: used to distinguish the device (for shared interrupts)
 *
 * Perform GPIO-specific ISR of the top-level GIP driver.
 */
irqreturn_t intel_qrk_gpio_isr(int irq, void *dev_id);

/**
 * intel_qrk_gpio_save_state
 *
 * Save GPIO register state for system-wide suspend events and mask out
 * interrupts.
 */
void intel_qrk_gpio_save_state(void);

/**
 * intel_qrk_gpio_restore_state
 *
 * Restore GPIO register state for system-wide resume events and clear out
 * spurious interrupts.
 */
void intel_qrk_gpio_restore_state(void);

/**
 * intel_qrk_i2c_probe
 * @param pdev: Pointer to GIP PCI device
 * @param drvdata: private driver data
 * @param drvdata: GIP platform-specific settings
 * @return 0 success < 0 failure
 *
 * Perform I2C-specific probing on behalf of the top-level GIP driver.
 */
int intel_qrk_i2c_probe(struct pci_dev *pdev, struct dw_i2c_dev **drvdata,
	struct intel_qrk_gip_pdata *pdata);

/**
 * intel_qrk_i2c_remove
 * @param pdev: Pointer to GIP PCI device
 * @param dev: Pointer to I2C private data
 *
 * Perform I2C-specific resource release on behalf of the top-level GIP driver.
 */
void intel_qrk_i2c_remove(struct pci_dev *pdev, struct dw_i2c_dev *dev);

#endif /* __INTEL_QRKGIP_H__ */
