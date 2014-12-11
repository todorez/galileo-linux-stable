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
 * Intel Quark IMR driver
 *
 * IMR stand for Insolate Memory Region, supported by Quark SoC.
 *
 * A total number of 8 IMRs have implemented by Quark SoC,
 * some IMRs might be already occupied by BIOS or Linux booting time.
 *
 * Input addresses parameter required the actual Physical address.
 *
 * The IMR alloc API will locate the next available IMR slot set up
 * with input memory region, and apply with the default access right
 * (CPU & CPU_snoop enable).
 *
 * The alloc_mask API takes input read & write masks values to set up
 * IMR with customized access right.
 *
 * User can free IMR with pre-alloc specified addresses.
 */

#ifndef __INTEL_QRK_IMR_H__
#define __INTEL_QRK_IMR_H__

#include <linux/intel_qrk_sb.h>
#include "asm/io.h"

#define CFG_READ_OPCODE         0x10    /* BUnit Control Read */
#define CFG_WRITE_OPCODE        0x11    /* BUnit control write */

/* DRAM IMR register addresses */
#define IMR0L			0x40
#define IMR0H			0x41
#define IMR0RM			0x42
#define IMR0WM			0x43
#define IMR1L			0x44
#define IMR1H			0x45
#define IMR1RM			0x46
#define IMR1WM			0x47
#define IMR2L			0x48
#define IMR2H			0x49
#define IMR2RM			0x4A
#define IMR2WM			0x4B
#define IMR3L			0x4C
#define IMR3H			0x4D
#define IMR3RM			0x4E
#define IMR3WM			0x4F
#define IMR4L			0x50
#define IMR4H			0x51
#define IMR4RM			0x52
#define IMR4WM			0x53
#define IMR5L			0x54
#define IMR5H			0x55
#define IMR5RM			0x56
#define IMR5WM			0x57
#define IMR6L			0x58
#define IMR6H			0x59
#define IMR6RM			0x5A
#define IMR6WM			0x5B
#define IMR7L			0x5C
#define IMR7H			0x5D
#define IMR7RM			0x5E
#define IMR7WM			0x5F

#define IMR_EN			0x40000000
#define IMR_LOCK_BIT            0x80000000
#define IMR_WRITE_ENABLE_ALL    0xFFFFFFFF
#define IMR_READ_ENABLE_ALL     0xBFFFFFFF
#define IMR_REG_MASK            0xFFFFFC

#define IMR_ESRAM_FLUSH_INIT	0x80000000  /* esram flush */
#define IMR_SNOOP_ENABLE	0x40000000  /* core snoops */
#define IMR_PUNIT_ENABLE	0x20000000
#define IMR_SMM_ENABLE		0x02        /* core SMM access */
#define IMR_NON_SMM_ENABLE	0x01        /* core non-SMM access */
#define IMR_BASE_ADDR           0x00
#define IMR_MEM_ALIGN           0x400

#define MAX_INFO_SIZE           64
#define IMR_MAXID		8

/* snoop + Non SMM write mask */
#define IMR_DEFAULT_MASK	(IMR_SNOOP_ENABLE \
				+ IMR_ESRAM_FLUSH_INIT \
				+ IMR_NON_SMM_ENABLE)

/* debug printk */
#ifdef DEBUG
#define DBG(args...) pr_info(args)
#else
#define DBG(args...)
#endif

extern unsigned long  _text;
extern char __init_begin[] ;

/**
 * intel_qrk_imr_alloc
 *
 * @param high: the end of physical memory address
 * @param low: the start of physical memory address
 * @param read: IMR read mask value
 * @param write: IMR write maks value
 * @param info: imr information
 * @param lock: imr lock
 *
 * Setup imr with customised read/ write masks
 */
int intel_qrk_imr_alloc(u32 high, u32 low, u32 read, u32 write,
			unsigned char *info, bool lock);

/**
 * intel_qrk_imr_free
 *
 * @param high: high boundary of memory address
 * @param low: low boundary of memorry address
 *
 * remove the imr based on input memory region
 */
int intel_qrk_imr_free(u32 high, u32 low);

/**
 * intel_qrk_remove_imr_entry
 *
 * @param id: internal imr data struct id
 *
 * Remove imr based on input imr data structure id
 */
void intel_qrk_remove_imr_entry(int id);

/**
 * intel_qrk_imr_init
 *
 * @param dev_id: Host Bridge's PCI device ID
 * Initialise IMRs
 */
int intel_qrk_imr_init(unsigned short dev_id);

#endif
