/*
 * Copyright (c) 2016 Google, Inc
 *
 * SPDX-License-Identifier:	GPL-2.0
 */

#ifndef __asm_intel_regs_h
#define __asm_intel_regs_h

/* Access the memory-controller hub */
#define MCH_BASE_ADDRESS	0xfed10000
#define MCH_BASE_SIZE		0x8000
#define MCHBAR_REG(reg)		(MCH_BASE_ADDRESS + (reg))

#endif
