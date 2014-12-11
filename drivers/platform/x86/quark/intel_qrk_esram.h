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
 * Intel Quark eSRAM overlay driver
 *
 * eSRAM is an on-chip fast access SRAM.
 *
 * This driver provides the ability to map a kallsyms derived symbol of
 * arbitrary length or a struct page entitiy.
 * A proc interface is provided to allow map/unmap of kernel structures, without
 * having to use the API from your code directly.
 *
 * Example:
 * echo ehci_irq on > /proc/driver/esram/map
 * echo ehci_irq off > /proc/driver/esram/map
 *
 * An API is provided to allow for mapping of a) kernel symbols or b) pages.
 * eSRAM requires 4k physically aligned addresses to work - so a struct page
 * fits neatly into this.
 *
 * To populte eSRAM we must copy data to a temporary buffer, overlay and
 * then copy data back to the eSRAM region.
 * 
 * When entering S3 - we must save eSRAM state to DRAM, and similarly on restore
 * to S0 we must repopulate eSRAM
 * 
 * Author : Bryan O'Donoghue <bryan.odonoghue@linux.intel.com>
 */
#ifndef __INTEL_QRK_ESRAM_H__
#define __INTEL_QRK_ESRAM_H__

#include <linux/module.h>

/* Basic size of an eSRAM page */
#define	INTEL_QRK_ESRAM_PAGE_SIZE	(0x1000)
#define INTEL_QRK_ESRAM_PAGE_COUNT	(0x80)
/**
 * intel_qrk_esram_map_range
 *
 * @param vaddr: Virtual address to start mapping (must be 4k aligned)
 * @param size: Size to map from
 * @param mapname: Mapping name
 * @return 0 success < 0 failure
 *
 * Map 4k increments at given address to eSRAM.
 */
int intel_qrk_esram_map_range(void * vaddr, u32 size, char * mapname);

/**
 * intel_qrk_esram_unmap_range
 *
 * @param vaddr: The virtual address to unmap
 * @return 0 success < 0 failure
 *
 * Logical corollary of esram_map_page
 */
int intel_qrk_esram_unmap_range(void * vaddr, u32 size, char * mapname);

/**
 * intel_qrk_esram_map_symbol
 *
 * @param vaddr: Virtual address of the symbol
 * @return 0 success < 0 failure
 *
 * Maps a series of 4k chunks starting at vaddr&0xFFFFF000. vaddr shall be a
 * kernel text section symbol (kernel or loaded module)
 *
 * We get the size of the symbol from kallsyms. We guarantee to map the entire
 * size of the symbol - plus whatever padding is necessary to get alignment to
 * eSRAM_PAGE_SIZE 
 * Other stuff inside the mapped pages will get a performance boost 'for free'.
 * If this free boost is not what you want then 
 *	1. Align to 4k
 *	2. Pad to 4k
 *	3. Call intel_qrk_esram_map_range()
 */
int intel_qrk_esram_map_symbol(void * vaddr);

/**
 * intel_qrk_esram_unmap_symbol
 *
 * @param vaddr: Virtual address of the symbol
 * @return 0 success < 0 failure
 *
 * Logical corollary to intel_qrk_esram_map_symbol
 * Undoes any mapping of pages starting at sym for sym's size
 */
int intel_qrk_esram_unmap_symbol(void * vaddr);

#endif /* __INTEL_QRK_ESRAM_H__ */
