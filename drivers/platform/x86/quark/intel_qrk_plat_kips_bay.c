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
 * Intel Quark Legacy Platform Data Layout.conf accessor
 *
 * Simple Legacy SPI flash access layer
 *
 * Author : Bryan O'Donoghue <bryan.odonoghue@linux.intel.com> 2013
 */

#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/spi/spi.h>

#define DRIVER_NAME		"KipsBay"
#define GPIO_RESTRICT_NAME	"qrk-gpio-restrict-sc"

static int gpio_cs = 1;

module_param(gpio_cs, int, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(gpio_cs, "Enable GPIO chip-select for SPI channel 1");


/******************************************************************************
 *             Analog Devices AD7298 SPI Device Platform Data
 ******************************************************************************/
#include "linux/platform_data/ad7298.h"

/* Maximum input voltage allowed for each ADC input, in milliVolts */
#define AD7298_MAX_EXT_VIN 5000

static const struct ad7298_platform_data ad7298_platform_data = {
	.ext_ref = false,
	.ext_vin_max = { AD7298_MAX_EXT_VIN, AD7298_MAX_EXT_VIN,
		AD7298_MAX_EXT_VIN, AD7298_MAX_EXT_VIN,
		AD7298_MAX_EXT_VIN, AD7298_MAX_EXT_VIN,
		AD7298_MAX_EXT_VIN, AD7298_MAX_EXT_VIN }
};

/******************************************************************************
 *                 Intel Quark SPI Controller Data
 ******************************************************************************/
static struct pxa2xx_spi_chip qrk_ffrd_spi_0_cs_0 = {
	.gpio_cs = 8,
};

static struct pxa2xx_spi_chip qrk_ffrd_spi_1_cs_0 = {
	.gpio_cs = 10,
};

static struct spi_board_info spi0_onboard_devs[] = {
	{
		.modalias = "ad7298",
		.max_speed_hz = 5000000,
		.platform_data = &ad7298_platform_data,
		.mode = SPI_MODE_2,
		.bus_num = 0,
		.chip_select = 0,
		.controller_data = &qrk_ffrd_spi_0_cs_0,
	}
};

static struct spi_board_info spi1_onboard_devs_gpiocs[] = {
	{
		.modalias = "spidev",
		.chip_select = 0,
		.controller_data = NULL,
		.max_speed_hz = 50000000,
		.bus_num = 1,
		.controller_data = &qrk_ffrd_spi_1_cs_0,
	},
};

static struct spi_board_info spi1_onboard_devs[] = {
	{
		.modalias = "spidev",
		.chip_select = 0,
		.controller_data = NULL,
		.max_speed_hz = 50000000,
		.bus_num = 1,
	},
};

/**
 * intel_qrk_spi_add_onboard_devs
 *
 * @return 0 on success or standard errnos on failure
 *
 * Registers onboard SPI device(s) present on the Kips Bay platform
 */
static int intel_qrk_spi_add_onboard_devs(void)
{
	int ret = 0;

	ret = spi_register_board_info(spi0_onboard_devs,
				      ARRAY_SIZE(spi0_onboard_devs));
	if (ret)
		return ret;

	if (gpio_cs)
		return spi_register_board_info(spi1_onboard_devs_gpiocs,
					ARRAY_SIZE(spi1_onboard_devs_gpiocs));
	else
		return spi_register_board_info(spi1_onboard_devs,
					ARRAY_SIZE(spi1_onboard_devs));
}


/**
 * intel_qrk_gpio_restrict_probe
 *
 * Make GPIOs pertaining to Firmware inaccessible by requesting them.  The
 * GPIOs are never released nor accessed by this driver.
 */
static int intel_qrk_gpio_restrict_probe(struct platform_device *pdev)
{
	return intel_qrk_spi_add_onboard_devs();
}

static struct platform_driver gpio_restrict_pdriver = {
	.driver		= {
		.name	= GPIO_RESTRICT_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= intel_qrk_gpio_restrict_probe,
};

static int intel_qrk_plat_kips_bay_probe(struct platform_device *pdev)
{
	return platform_driver_register(&gpio_restrict_pdriver);
}

static int intel_qrk_plat_kips_bay_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver qrk_kips_bay_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= intel_qrk_plat_kips_bay_probe,
	.remove		= intel_qrk_plat_kips_bay_remove,
};

module_platform_driver(qrk_kips_bay_driver);

MODULE_AUTHOR("Bryan O'Donoghue <bryan.odonoghue@intel.com>");
MODULE_DESCRIPTION("Kips Bay BSP Data");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:"DRIVER_NAME);

