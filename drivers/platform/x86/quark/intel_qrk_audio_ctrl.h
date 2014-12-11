/*
 * Intel Quark platform audio control driver
 *
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
 *
 * See intel_qrk_audio_ctrl.c for a detailed description
 *
 */

#ifndef __INTEL_QRK_AUDIO_CTRL_H__
#define __INTEL_QRK_AUDIO_CTRL_H__

#include <linux/module.h>

#define INTEL_QRK_AUDIO_MODE_GSM_ONLY       0x0
#define INTEL_QRK_AUDIO_MODE_SPKR_ONLY      0x1
#define INTEL_QRK_AUDIO_MODE_SPKR_MIC       0x3
#define INTEL_QRK_AUDIO_MODE_GSM_SPKR_MIC   0x5

#define INTEL_QRK_AUDIO_MODE_IOC_GSM_ONLY \
	_IO('x', INTEL_QRK_AUDIO_MODE_GSM_ONLY)
#define INTEL_QRK_AUDIO_MODE_IOC_SPKR_ONLY \
	_IO('x', INTEL_QRK_AUDIO_MODE_SPKR_ONLY)
#define INTEL_QRK_AUDIO_MODE_IOC_SPKR_MIC \
	_IO('x', INTEL_QRK_AUDIO_MODE_SPKR_MIC)
#define INTEL_QRK_AUDIO_MODE_IOC_GSM_SPKR_MIC \
	_IO('x', INTEL_QRK_AUDIO_MODE_GSM_SPKR_MIC)

#endif /* __INTEL_QRK_AUDIO_CTRL_H__ */
