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
 * CrossHill board entry point
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

#define DRIVER_NAME		"CrossHill"
#define GPIO_RESTRICT_NAME_NC	"qrk-gpio-restrict-nc"
#define GPIO_RESTRICT_NAME_SC	"qrk-gpio-restrict-sc"

/*
 * GPIO numbers to use for reading 4-bit Blackburn Peak SPI daughterboard ID
 */
#define SPI_BPEAK_RESET_GPIO 4
#define SPI_BPEAK_ID0_GPIO   3
#define SPI_BPEAK_ID1_GPIO   2
#define SPI_BPEAK_ID2_GPIO   15
#define SPI_BPEAK_ID3_GPIO   14

/*
 * GPIO number for eADC interrupt (MAX78M6610+LMU)
 */
#define GPIO_78M6610_INT	2

static int nc_gpio_reg;
static int sc_gpio_reg;

static int cross_hill_probe;
/*
 * GPIOs used as interrupts by MAX78M6610+LMU eADC
 * 
 * Extend this array adding new elements at the end.
 */
static struct gpio crh_eadc_int_gpios[] = {
	{
		GPIO_78M6610_INT,
		GPIOF_IN,
		"max78m6610-int"
	},
};


/*
 * Blackburn Peak SPI daughterboard ID values
 */
enum {
	QRK_SPI_BPEAK_ID_ZB_TI = 0xA,
	QRK_SPI_BPEAK_ID_ZB_DIGI,
	QRK_SPI_BPEAK_ID_ZB_INFR_NXP,
	QRK_SPI_BPEAK_ID_ZB_EXEGIN_ATMEL,
	QRK_SPI_BPEAK_ID_ADC_MAXIM,
	QRK_SPI_BPEAK_ID_NONE
};


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
 *             Maxim 78M6610+LMU SPI Device Platform Data
 ******************************************************************************/
#include "linux/platform_data/max78m6610_lmu.h"

static const struct max78m6610_lmu_platform_data max78m6610_lmu_pdata = {
	.reset_gpio = SPI_BPEAK_RESET_GPIO,
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

static struct pxa2xx_spi_chip qrk_ffrd_spi_1_cs_1 = {
	.gpio_cs = 11,
};

static struct spi_board_info spi_generic_devs[] = {
	{
		.modalias = "spidev",
		.max_speed_hz = 50000000,
		.platform_data = NULL,
		.mode = SPI_MODE_0,
		.bus_num = 1,
		.chip_select = 0,
		.controller_data = &qrk_ffrd_spi_1_cs_0,
	},

	{
		.modalias = "spidev",
		.max_speed_hz = 50000000,
		.platform_data = NULL,
		.mode = SPI_MODE_0,
		.bus_num = 1,
		.chip_select = 1,
		.controller_data = &qrk_ffrd_spi_1_cs_1,
	},

};

/* For compatibility reason, new SPI energy modules must be added at the end */
static struct spi_board_info spi_energy_adc_devs[] = {
	{
		.modalias = "max78m6610_lmu",
		.max_speed_hz = 2000000,
		.platform_data = &max78m6610_lmu_pdata,
		.mode = SPI_MODE_3,
		.bus_num = 1,
		.chip_select = 0,
		.controller_data = &qrk_ffrd_spi_1_cs_0,
		.irq = -1,
	},
};



/**
 * intel_qrk_spi_add_onboard_devs
 *
 * @return 0 on success or standard errnos on failure
 *
 * Registers onboard SPI device(s) present on the Cross Hill platform
 */
static int intel_qrk_spi_add_onboard_devs(void)
{
	struct spi_board_info spi_onboard_devs[] = {
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

	return spi_register_board_info(spi_onboard_devs,
			ARRAY_SIZE(spi_onboard_devs));
}


/**
 * intel_qrk_spi_get_bpeak_id
 *
 * @param bpeak_id: The Blackburn Peak SPI ID obtained from the daughterboard
 * @return 0 on success or standard errnos on failure
 *
 * Reads an ID from GPIO-connected pins on Blackburn peak SPI daughterboard
 */
static int intel_qrk_spi_get_bpeak_id(u8 *bpeak_id)
{
	int ret = 0;
	struct gpio spi_bpeak_id_gpios[] = {
		{
			SPI_BPEAK_RESET_GPIO,
			GPIOF_OUT_INIT_HIGH,
			"spi_bpeak_reset"
		},
		{
			SPI_BPEAK_ID0_GPIO,
			GPIOF_IN,
			"spi_bpeak_id0"
		},
		{
			SPI_BPEAK_ID1_GPIO,
			GPIOF_IN,
			"spi_bpeak_id1"
		},
		{
			SPI_BPEAK_ID2_GPIO,
			GPIOF_IN,
			"spi_bpeak_id2"
		},
		{
			SPI_BPEAK_ID3_GPIO,
			GPIOF_IN,
			"spi_bpeak_id3"
		}
	};

	/*
	 * Read a 4-bit ID value from ID GPIO inputs, which are only valid
	 * while a RESET GPIO output is asserted (active-low)
	 */
	ret = gpio_request_array(spi_bpeak_id_gpios,
			ARRAY_SIZE(spi_bpeak_id_gpios));
	if (ret) {
		pr_err("%s: Failed to allocate Blackburn Peak ID GPIO pins\n",
				__func__);
		return ret;
	}

	gpio_set_value(SPI_BPEAK_RESET_GPIO, 0);
	*bpeak_id =
		(gpio_get_value(SPI_BPEAK_ID3_GPIO) ? 1 << 3 : 0) |
		(gpio_get_value(SPI_BPEAK_ID2_GPIO) ? 1 << 2 : 0) |
		(gpio_get_value(SPI_BPEAK_ID1_GPIO) ? 1 << 1 : 0) |
		(gpio_get_value(SPI_BPEAK_ID0_GPIO) ? 1      : 0);
	gpio_set_value(SPI_BPEAK_RESET_GPIO, 1);

	gpio_free_array(spi_bpeak_id_gpios,
			ARRAY_SIZE(spi_bpeak_id_gpios));

	return 0;
}

/**
 * intel_qrk_spi_add_bpeak_devs
 *
 * @return 0 on success or standard errnos on failure
 *
 * Registers SPI device(s) indicated by the ID value obtained from a
 * Blackburn Peak SPI daughterboard
 */
static int intel_qrk_spi_add_bpeak_devs(void)
{
	u8 spi_bpeak_id = 0;
	int ret = 0;

	ret = intel_qrk_spi_get_bpeak_id(&spi_bpeak_id);
	if (ret) {
		pr_err("%s: failed to obtain Blackburn Peak ID\n",
				__func__);
		return ret;
	}

	switch (spi_bpeak_id) {

	case QRK_SPI_BPEAK_ID_NONE:
		break;

	case QRK_SPI_BPEAK_ID_ADC_MAXIM:
		{
			ret = gpio_request_array(crh_eadc_int_gpios,
					ARRAY_SIZE(crh_eadc_int_gpios));
			if (ret) {
				pr_err("%s: Failed to allocate eADC interrupt GPIO pins!\n",
						__func__);
				return ret;
			}
			ret = gpio_to_irq(GPIO_78M6610_INT);
			if (ret < 0) {
				pr_err("%s: Failed to request IRQ for GPIO %u!\n",
						__func__, GPIO_78M6610_INT);
				goto error_gpio_free;
			}
			spi_energy_adc_devs[0].irq = ret;
			ret = spi_register_board_info(spi_energy_adc_devs,
					ARRAY_SIZE(spi_energy_adc_devs));
			if (ret) {
				pr_err("%s: Failed to register eADC module!\n",
						__func__);
				goto error_gpio_free;
			}
			return 0;
error_gpio_free:
			gpio_free_array(crh_eadc_int_gpios,
					ARRAY_SIZE(crh_eadc_int_gpios));
			spi_energy_adc_devs[0].irq = -1;
			return ret;
		}
	case QRK_SPI_BPEAK_ID_ZB_EXEGIN_ATMEL:
		{
			pr_debug("QRK_SPI_BPEAK_ID_ZB_EXEGIN_ATMEL.\n");
			return spi_register_board_info(spi_generic_devs,
					ARRAY_SIZE(spi_generic_devs));
		}
	case QRK_SPI_BPEAK_ID_ZB_DIGI:
		{
			pr_debug("QRK_SPI_BPEAK_ID_ZB_DIGI load.\n");
			return spi_register_board_info(spi_generic_devs,
					ARRAY_SIZE(spi_generic_devs));

		}
	default:
			pr_err("%s: Unsupported Blackburn Peak SPI ID %u\n",
					__func__, spi_bpeak_id);
			ret = -EINVAL;
	}

	return ret;
}

/** intel_qrk_spi_devs_addon
 *
 * addon spi device when gpio support in place
 */
static int intel_qrk_spi_devs_addon(void)
{
	int ret = 0;

	if (cross_hill_probe != 1) {

		ret = intel_qrk_spi_add_onboard_devs();
		if (ret)
			return ret;

		ret = intel_qrk_spi_add_bpeak_devs();

		cross_hill_probe = 1;
	}

	return ret;
}

/**
 * intel_qrk_gpio_restrict_probe_nc
 *
 * Make GPIOs pertaining to Firmware inaccessible by requesting them.  The
 * GPIOs are never released nor accessed by this driver.
 */
static int intel_qrk_gpio_restrict_probe_nc(struct platform_device *pdev)
{
	int ret;
	nc_gpio_reg = 1;

	if (nc_gpio_reg == 1 && sc_gpio_reg == 1) {
		ret = intel_qrk_spi_devs_addon();
		if (ret)
			return ret;
	}
	return 0;
}

/**
 * intel_qrk_gpio_restrict_probe_sc
 *
 * Make GPIOs pertaining to Firmware inaccessible by requesting them.  The
 * GPIOs are never released nor accessed by this driver.
 */
static int intel_qrk_gpio_restrict_probe_sc(struct platform_device *pdev)
{
	int ret;
	sc_gpio_reg = 1;

	if (nc_gpio_reg == 1 && sc_gpio_reg == 1) {
		ret = intel_qrk_spi_devs_addon();
		if (ret)
			return ret;
	}
	return 0;
}

static struct platform_driver gpio_restrict_pdriver_nc = {
	.driver		= {
		.name	= GPIO_RESTRICT_NAME_NC,
		.owner	= THIS_MODULE,
	},
	.probe		= intel_qrk_gpio_restrict_probe_nc,
};

static struct platform_driver gpio_restrict_pdriver_sc = {
	.driver		= {
		.name	= GPIO_RESTRICT_NAME_SC,
		.owner	= THIS_MODULE,
	},
	.probe		= intel_qrk_gpio_restrict_probe_sc,
};

static int intel_qrk_plat_cross_hill_probe(struct platform_device *pdev)
{
	int ret = 0;

	ret = platform_driver_register(&gpio_restrict_pdriver_nc);
	if (ret)
		return ret;

	return platform_driver_register(&gpio_restrict_pdriver_sc);
}

static int intel_qrk_plat_cross_hill_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver qrk_cross_hill_driver = {
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
	},
	.probe		= intel_qrk_plat_cross_hill_probe,
	.remove		= intel_qrk_plat_cross_hill_remove,
};

module_platform_driver(qrk_cross_hill_driver);

MODULE_AUTHOR("Bryan O'Donoghue <bryan.odonoghue@intel.com>");
MODULE_DESCRIPTION("Cross Hill BSP Data");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_ALIAS("platform:"DRIVER_NAME);

