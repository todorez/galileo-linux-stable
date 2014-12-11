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
 * Intel Quark side-band driver
 *
 * Thread-safe sideband read/write routine.
 *
 * Author : Bryan O'Donoghue <bryan.odonoghue@linux.intel.com> 2012
 */

#ifndef __INTEL_QRK_SB_H__
#define __INTEL_QRK_SB_H__

#include <linux/types.h>

#define PCI_DEVICE_ID_X1000_HOST_BRIDGE			0x0958

typedef enum {
	SB_ID_HUNIT = 0x03,
	SB_ID_THERMAL = 0x04,
	SB_ID_ESRAM = 0x05,
	SB_ID_SOC = 0x31,
}qrk_sb_id;

/**
 * intel_qrk_sb_read_reg
 *
 * @param qrk_sb_id: Sideband identifier
 * @param command: Command to send to destination identifier
 * @param reg: Target register w/r to qrk_sb_id
 * @return nothing
 *
 * Utility function to allow thread-safe read of side-band
 * command - can be different read op-code types - which is why we don't
 * hard-code this value directly into msg
 */
void intel_qrk_sb_read_reg(qrk_sb_id id, u8 cmd, u8 reg, u32 *data, u8 lock);

/**
 * intel_qrk_sb_write_reg
 *
 * @param qrk_sb_id: Sideband identifier
 * @param command: Command to send to destination identifier
 * @param reg: Target register w/r to qrk_sb_id
 * @return nothing
 *
 * Utility function to allow thread-safe write of side-band
 */
void intel_qrk_sb_write_reg(qrk_sb_id id, u8 cmd, u8 reg, u32 data, u8 lock);

/**
 * intel_qrk_sb_runfn_lock
 *
 * @param fn: Callback function - which requires side-band spinlock and !irq
 * @param arg: Callback argument
 * @return 0 on success < 0 on failure
 *
 * Runs the given function pointer inside of a call to the local spinlock using
 * spin_lock_irqsave/spin_unlock_irqrestore. Needed for the eSRAMv1 driver to
 * guarantee atomicity, but, available to any other user of sideband provided
 * rules are respected.
 * Rules:
 *	fn may not sleep
 *	fn may not change the state of irqs	
 */
int intel_qrk_sb_runfn_lock(int (*fn)( void * arg ), void * arg);

/**
 * intel_qrk_sb_initialized
 *
 * False if sideband running on non-Quark system
 */
int intel_qrk_sb_initialized(void);

#endif /* __INTEL_QRK_SB_H__ */
