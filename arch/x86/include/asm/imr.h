/*
 * imr.h: Intel Quark platform imr setup code
 *
 * (C) Copyright 2012 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _ASM_X86_IMR_H
#define _ASM_X86_IMR_H

#if defined(CONFIG_INTEL_QUARK_X1000_SOC)
	extern int intel_qrk_imr_runt_setparams(void);
	extern int intel_qrk_imr_lockall(void);
#else
	static void intel_qrk_imr_runt_setparams(void){}
	static void intel_qrk_imr_lockall(void){}
#endif

#endif /* _ASM_X86_IMR_H */
