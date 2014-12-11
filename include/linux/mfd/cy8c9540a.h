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

#ifndef LINUX_CY8C9540A_PDATA_H
#define LINUX_CY8C9540A_PDATA_H

#include <linux/types.h>

#define CY8C9540A_POR_SETTINGS_LEN		147
#define CY8C9540A_NPWM				8
#define CY8C9540A_PWM_UNUSED			-1

struct cy8c9540a_pdata {
	u8 por_default[CY8C9540A_POR_SETTINGS_LEN];
	int pwm2gpio_mapping[CY8C9540A_NPWM];
	int gpio_base;
	int pwm_base;
	int irq_base;
};

#endif
