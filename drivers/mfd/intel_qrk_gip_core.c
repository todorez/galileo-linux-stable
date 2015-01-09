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
 * Intel Quark GIP (GPIO/I2C) PCI driver
 *
 *  PCI glue logic for Quark GIP driver.
 *  Quark GIP is a single PCI function exporting a GPIO and an I2C device.
 *  The PCI driver performs the bus-dependent probe/release operations, and
 *  call into GPIO/I2C specific modules for handling the two diffrerent
 *  functionalities.
 */

#include <asm/qrk.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/mfd/intel_qrk_gip_pdata.h>
#include <linux/module.h>
#include <linux/pci.h>
#include "intel_qrk_gip.h"

static unsigned int enable_msi = 1;
module_param(enable_msi, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(enable_msi, "Enable PCI MSI mode");

static unsigned int i2c = 1;
module_param(i2c, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(i2c, "Register I2C adapter");

static unsigned int gpio = 1;
module_param(gpio, uint, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(gpio, "Register GPIO chip");

/* GIP private data, supporting only a single instance of the device. */
struct intel_qrk_gip_data {
	struct pci_dev *pci_device;
	struct dw_i2c_dev *i2c_drvdata;
	struct intel_qrk_gip_pdata *pdata;
};

/**
 * intel_qrk_gip_handler
 *
 * @param irq: IRQ number to be served
 * @param dev_id: device private data
 *
 * Top-level interrupt handler for GIP driver.
 * It calls into the appropriate sub-routines and gathers the return values.
 */
static irqreturn_t intel_qrk_gip_handler(int irq, void *dev_id)
{
	irqreturn_t ret_i2c = IRQ_NONE;
	irqreturn_t ret_gpio = IRQ_NONE;
	struct intel_qrk_gip_data *data = (struct intel_qrk_gip_data *)dev_id;

	mask_pvm(data->pci_device);

	if (likely(i2c)) {
		/* Only I2C gets platform data */
		ret_i2c = i2c_dw_isr(irq, data->i2c_drvdata);
	}

	if (likely(gpio))
		ret_gpio = intel_qrk_gpio_isr(irq, NULL);

	unmask_pvm(data->pci_device);

	if (likely(IRQ_HANDLED == ret_i2c || IRQ_HANDLED == ret_gpio))
		return IRQ_HANDLED;

	/* Each sub-ISR routine returns either IRQ_HANDLED or IRQ_NONE. */
	return IRQ_NONE;
}

static struct pci_device_id intel_qrk_gip_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x0934) },
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, intel_qrk_gip_ids);

#ifdef CONFIG_PM

/**
 * qrk_gip_suspend
 *
 * @param device: Pointer to device
 * @return 0 success < 0 failure
 *
 * Prepare GIP for system-wide transition to sleep state.
 * Save context, disable GPIO chip and I2C adapter, transition PCI device into
 * low-power state.
 */
static int qrk_gip_suspend(struct device *dev)
{
	int err = 0;
	struct intel_qrk_gip_data *data = NULL;
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);

	data = (struct intel_qrk_gip_data *)pci_get_drvdata(pdev);

	i2c_dw_disable(data->i2c_drvdata);
	intel_qrk_gpio_save_state();

	err = pci_save_state(pdev);
	if (err) {
		dev_err(&pdev->dev, "pci_save_state failed\n");
		return err;
	}

	err = pci_set_power_state(pdev, PCI_D3hot);
	if (err) {
		dev_err(&pdev->dev, "pci_set_power_state failed\n");
		return err;
	}

	return 0;
}

/**
 * qrk_gip_resume
 *
 * @param device: Pointer to device
 * @return 0 success < 0 failure
 *
 * Prepare GIP for system-wide transition to fully active state.
 * Set PCI device into full-power state, restore context, enable I2C adapter
 * and GPIO chip.
 */
static int qrk_gip_resume(struct device *dev)
{
	int err = 0;
	struct intel_qrk_gip_data *data = NULL;
	struct pci_dev *pdev = container_of(dev, struct pci_dev, dev);

	data = (struct intel_qrk_gip_data *)pci_get_drvdata(pdev);

	err = pci_set_power_state(pdev, PCI_D0);
	if (err) {
		dev_err(&pdev->dev, "pci_set_power_state() failed\n");
		return err;
	}

	pci_restore_state(pdev);

	intel_qrk_gpio_restore_state();
	i2c_dw_init(data->i2c_drvdata);
	return 0;
}

#else
#define qrk_gip_suspend		NULL
#define qrk_gip_resume		NULL
#endif

static const struct dev_pm_ops qrk_gip_pm_ops = {
	.resume         = qrk_gip_resume,
	.suspend        = qrk_gip_suspend,
};

/**
 * intel_qrk_gip_probe
 *
 * @param pdev: Pointer to GIP PCI device
 * @param id: GIP PCI Device ID
 * @return 0 success < 0 failure
 *
 * GIP PCI driver probing. Calls into the appropriate probe routines for GPIO
 * and I2C too.
 */
static int intel_qrk_gip_probe(struct pci_dev *pdev,
				const struct pci_device_id *id)
{
	int retval = 0;
	struct intel_qrk_gip_data *gip_drvdata = NULL;

	retval = pci_enable_device(pdev);
	if (retval)
		return retval;

	retval = pci_request_regions(pdev, "qrk-gip");
	if (retval) {
		dev_err(&pdev->dev, "error requesting PCI region\n");
		goto err_pcidev_disable;
	}

	gip_drvdata = kzalloc(sizeof(struct intel_qrk_gip_data), GFP_KERNEL);
	if (NULL == gip_drvdata) {
		retval = -ENOMEM;
		goto err_pciregions_release;
	}
	pci_set_drvdata(pdev, gip_drvdata);

	gip_drvdata->pci_device = pdev;

	/* Retrieve platform data if there is any */
	if (*intel_qrk_gip_get_pdata)
		gip_drvdata->pdata = intel_qrk_gip_get_pdata();

	if (gpio) {
		retval = intel_qrk_gpio_probe(pdev);
		if (retval)
			goto err_release_drvdata;
	}


	if (i2c) {
		retval = intel_qrk_i2c_probe(pdev,
			(struct dw_i2c_dev **)&gip_drvdata->i2c_drvdata,
			gip_drvdata->pdata);
		if (retval)
			goto err_release_drvdata;
	}

	if (enable_msi) {
		pci_set_master(pdev);
		retval = pci_enable_msi(pdev);
		if (retval)
			goto err_release_drvdata;
	}

	/*
	 * Request a shared IRQ.
	 * Since the dev_id cannot be NULL, it points to PCI device descriptor
	 * if I2C is not registered.
	 */
	retval = request_irq(pdev->irq, intel_qrk_gip_handler, IRQF_SHARED,
			"intel_qrk_gip", gip_drvdata);
	if (retval) {
		dev_err(&pdev->dev, "error requesting IRQ\n");
		goto err;
	}

	return 0;

err_release_drvdata:
	pci_set_drvdata(pdev, NULL);
	kfree(gip_drvdata);
err:
	if (enable_msi)
		pci_disable_msi(pdev);
err_pciregions_release:
	pci_release_regions(pdev);
err_pcidev_disable:
	pci_disable_device(pdev);

	return retval;
}

/**
 * intel_qrk_gip_remove
 *
 * @param pdev: Pointer to GIP PCI device
 *
 * Release resources. Calls into GPIO/I2C dedicate routines too.
 */
static void intel_qrk_gip_remove(struct pci_dev *pdev)
{
	struct intel_qrk_gip_data *data = NULL;

	data = (struct intel_qrk_gip_data *)pci_get_drvdata(pdev);

	if (NULL == data) {
		dev_err(&pdev->dev, "%s: failure getting driver data\n",
			__func__);
		return;
	}

	free_irq(pdev->irq, data);

	if (enable_msi) {
		pci_clear_master(pdev);
		if (pci_dev_msi_enabled(pdev))
			pci_disable_msi(pdev);
	}

	if (i2c)
		intel_qrk_i2c_remove(pdev, data->i2c_drvdata);

	if (gpio)
		intel_qrk_gpio_remove(pdev);

	pci_set_drvdata(pdev, NULL);
	kfree(data);

	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

static struct pci_driver intel_qrk_gip_driver = {
	.name =		"intel_qrk_gip",
	.id_table	= intel_qrk_gip_ids,
	.probe		= intel_qrk_gip_probe,
	.remove		= intel_qrk_gip_remove,
	.driver         = {
		.pm     = &qrk_gip_pm_ops,
	},
};

static int intel_qrk_gip_init(void)
{
	return pci_register_driver(&intel_qrk_gip_driver);
}

static void intel_qrk_gip_exit(void)
{
	pci_unregister_driver(&intel_qrk_gip_driver);
}

module_init(intel_qrk_gip_init);
module_exit(intel_qrk_gip_exit);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Quark GIP driver");
MODULE_LICENSE("Dual BSD/GPL");
