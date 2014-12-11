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

#ifndef LINUX_INTEL_QRK_GIP_DATA_H
#define LINUX_INTEL_QRK_GIP_DATA_H

struct pci_dev;

struct intel_qrk_gip_pdata {
	int		i2c_std_mode;
};

extern struct intel_qrk_gip_pdata *(*intel_qrk_gip_get_pdata)(void);

#endif
