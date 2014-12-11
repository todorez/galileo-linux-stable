/*
 * Platform data for max78m6610+lmu SPI protocol driver
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

#ifndef __LINUX_PLATFORM_DATA_MAX78M6610_LMU_H__
#define __LINUX_PLATFORM_DATA_MAX78M6610_LMU_H__

/**
 * struct max78m6610_lmu_platform_data - Platform data for the max78m6610_lmu
 *                                       ADC driver
 * @reset_gpio: GPIO pin number reserved for MAX78M6610+LMU device reset
 **/
struct max78m6610_lmu_platform_data {
	int reset_gpio;
};

#endif /* IIO_ADC_MAX78M6610_LMU_H_ */
