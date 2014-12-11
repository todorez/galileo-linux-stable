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
 * Intel Quark platform data definition
 */

#ifndef _PDATA_QUARK_H
#define _PDATA_QUARK_H

typedef enum  {
	QUARK_PLAT_UNDEFINED = 0,
	QUARK_EMULATION = 1,
	QUARK_PEAK = 2,
	KIPS_BAY = 3,
	CROSS_HILL = 4,
	QUARK_HILL = 5,
	GALILEO = 6,
}qrk_plat_id_t;

typedef enum {
	PLAT_DATA_ID = 1,
	PLAT_DATA_SN = 2,
	PLAT_DATA_MAC0 = 3,
	PLAT_DATA_MAC1 = 4,
}plat_dataid_t;

#endif /* _PDATA_QUARK_H */
