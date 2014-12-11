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
 */

#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/spi/spi.h>

#define DRIVER_NAME		"ClantonHill"
#define GPIO_RESTRICT_NAME	"qrk-gpio-restrict-nc"

/******************************************************************************
 *             Analog Devices AD7298 SPI Device Platform Data
 ******************************************************************************/
#include "linux/platform_data/ad7298.h"

/* Maximum input voltage allowed for each ADC input, in milliVolts */
#define AD7298_MAX_EXT_VIN 5000
#define AD7298_MAX_EXT_VIN_EXT_BATT 30000
#define AD7298_MAX_EXT_VIN_INT_BATT 9200

static const struct ad7298_platform_data ad7298_platform_data = {
	.ext_ref = false,
	.ext_vin_max = { AD7298_MAX_EXT_VIN, AD7298_MAX_EXT_VIN,
		AD7298_MAX_EXT_VIN, AD7298_MAX_EXT_VIN,
		AD7298_MAX_EXT_VIN, AD7298_MAX_EXT_VIN,
		AD7298_MAX_EXT_VIN_EXT_BATT, AD7298_MAX_EXT_VIN_INT_BATT }
};

/******************************************************************************
 *                 Intel Quark SPI Controller Data
 ******************************************************************************/
static struct pxa2xx_spi_chip qrk_ffrd_spi_0_cs_0 = {
	.gpio_cs = 8,
};

static struct spi_board_info spi_onboard_devs[] = {
	{
		.modalias = "ad7298",
		.max_speed_hz = 5000000,
		.platform_data = &ad7298_platform_data,
		.mode = SPI_MODE_2,
		.bus_num = 0,
		.chip_select = 0,
		.controller_data = &qrk_ffrd_spi_0_cs_0,
	},
};

/******************************************************************************
 *             ST Microelectronics LIS331DLH I2C Device Platform Data
 ******************************************************************************/
#include <linux/platform_data/lis331dlh_intel_qrk.h>

/* GPIO interrupt pins connected to the LIS331DLH */
#define ST_ACCEL_INT1_GPIO 15
#define ST_ACCEL_INT2_GPIO 4

static struct lis331dlh_intel_qrk_platform_data lis331dlh_i2c_platform_data = {
	.irq1_pin = ST_ACCEL_INT1_GPIO,
};

static struct gpio reserved_gpios[] = {
	{
		ST_ACCEL_INT1_GPIO,
		GPIOF_IN,
		"st_accel_i2c-int1"
	},
	{
		ST_ACCEL_INT2_GPIO,
		GPIOF_IN,
		"st_accel_i2c-int2"
	},
};

/* I2C device addresses */
#define MAX9867_ADDR				0x18
#define LIS331DLH_ADDR				0x19

static struct i2c_adapter *i2c_adap;
static struct i2c_board_info probed_i2c_lis331dlh = {
	.platform_data = &lis331dlh_i2c_platform_data,
};
static struct i2c_board_info probed_i2c_max9867;
static const unsigned short max9867_i2c_addr[] =
	{ MAX9867_ADDR, I2C_CLIENT_END };
static const unsigned short lis331dlh_i2c_addr[] =
	{ LIS331DLH_ADDR, I2C_CLIENT_END };

static int i2c_probe(struct i2c_adapter *adap, unsigned short addr)
{
	/* Always return success: the I2C clients are already known.  */
	return 1;
}

/**
 * intel_qrk_spi_add_onboard_devs
 *
 * @return 0 on success or standard errnos on failure
 *
 * Registers onboard SPI device(s) present on the Clanton Hill platform
 */
static int intel_qrk_spi_add_onboard_devs(void)
{
	return spi_register_board_info(spi_onboard_devs,
				       ARRAY_SIZE(spi_onboard_devs));
}

/**
 * intel_qrk_gpio_restrict_probe
 *
 * Make GPIOs pertaining to Firmware inaccessible by requesting them.  The
 * GPIOs are never released nor accessed by this driver.
 */
static int intel_qrk_gpio_restrict_probe(struct platform_device *pdev)
{
	int ret = 0;
	static int gpio_done, spi_done;
	struct i2c_client *max9867 = NULL, *lis331dlh = NULL;

	if (gpio_done)
		goto spi;
	ret = gpio_request_array(reserved_gpios, ARRAY_SIZE(reserved_gpios));
	if (ret)
		goto end;
	gpio_done = 1;

spi:
	if (spi_done)
		goto i2c;
	ret = intel_qrk_spi_add_onboard_devs();
	if (ret)
		goto end;
	spi_done = 1;
i2c:
	i2c_adap = i2c_get_adapter(0);
	if (NULL == i2c_adap) {
		pr_info("%s: i2c adapter not ready yet. Deferring..\n",
			__func__);
		ret = -EPROBE_DEFER;
		goto end;
	}
	strlcpy(probed_i2c_max9867.type, "intel-qrk-max9867", I2C_NAME_SIZE);
	max9867 = i2c_new_probed_device(i2c_adap, &probed_i2c_max9867,
					max9867_i2c_addr, i2c_probe);
	strlcpy(probed_i2c_lis331dlh.type, "lis331dlh_qrk", I2C_NAME_SIZE);
	lis331dlh = i2c_new_probed_device(i2c_adap, &probed_i2c_lis331dlh,
					lis331dlh_i2c_addr, i2c_probe);
	i2c_put_adapter(i2c_adap);

	if (NULL == max9867 || NULL == lis331dlh) {
		pr_err("%s: can't probe I2C devices\n", __func__);
		ret = -ENODEV;
		goto end;
	}

end:
	return ret;
}

static struct platform_driver gpio_restrict_pdriver = {
	.driver		= {
		.name	= GPIO_RESTRICT_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= intel_qrk_gpio_restrict_probe,
};

static int intel_qrk_plat_clanton_hill_probe(struct platform_device *pdev)
{
	int ret = 0;

	ret = platform_driver_register(&gpio_restrict_pdriver);

	return ret;
}

static int intel_qrk_plat_clanton_hill_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver qrk_clanton_hill_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= intel_qrk_plat_clanton_hill_probe,
	.remove		= intel_qrk_plat_clanton_hill_remove,
};

module_platform_driver(qrk_clanton_hill_driver);

MODULE_AUTHOR("Bryan O'Donoghue <bryan.odonoghue@intel.com>");
MODULE_DESCRIPTION("Clanton Hill BSP Data");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:"DRIVER_NAME);

