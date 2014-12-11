/*
 * AD7298 SPI ADC driver
 *
 * Copyright 2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef __LINUX_PLATFORM_DATA_AD7298_H__
#define __LINUX_PLATFORM_DATA_AD7298_H__

#define AD7298_MAX_CHAN		8

/**
 * struct ad7298_platform_data - Platform data for the ad7298 ADC driver
 * @ext_ref: Whether to use an external reference voltage.
 * @ext_vin_max: External input voltage range for each voltage input channel
 *               (set to non-zero if platform uses external voltage dividers)
 **/
struct ad7298_platform_data {
	bool ext_ref;
	u16  ext_vin_max[AD7298_MAX_CHAN];
};

#endif /* IIO_ADC_AD7298_H_ */
