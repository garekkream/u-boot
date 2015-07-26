/*
 * Copyright (C) 2015 Google, Inc
 *
 * SPDX-License-Identifier:	GPL-2.0+
 *
 * From linux dma.h: Defines for using and allocating dma channels.
 * Written by Hennus Bergman, 1992.
 * High DMA channel support & info by Hannu Savolainen
 * and John Boyd, Nov. 1992.
 */

#include <common.h>
#include <asm/io.h>
#include <asm/isa_dma.h>

/* DMA controller registers */
#define DMA1_CMD_REG            0x08    /* command register (w) */
#define DMA1_STAT_REG           0x08    /* status register (r) */
#define DMA1_REQ_REG            0x09    /* request register (w) */
#define DMA1_MASK_REG           0x0A    /* single-channel mask (w) */
#define DMA1_MODE_REG           0x0B    /* mode register (w) */
#define DMA1_CLEAR_FF_REG       0x0C    /* clear pointer flip-flop (w) */
#define DMA1_TEMP_REG           0x0D    /* Temporary Register (r) */
#define DMA1_RESET_REG          0x0D    /* Master Clear (w) */
#define DMA1_CLR_MASK_REG       0x0E    /* Clear Mask */
#define DMA1_MASK_ALL_REG       0x0F    /* all-channels mask (w) */

#define DMA2_CMD_REG            0xD0    /* command register (w) */
#define DMA2_STAT_REG           0xD0    /* status register (r) */
#define DMA2_REQ_REG            0xD2    /* request register (w) */
#define DMA2_MASK_REG           0xD4    /* single-channel mask (w) */
#define DMA2_MODE_REG           0xD6    /* mode register (w) */
#define DMA2_CLEAR_FF_REG       0xD8    /* clear pointer flip-flop (w) */
#define DMA2_TEMP_REG           0xDA    /* Temporary Register (r) */
#define DMA2_RESET_REG          0xDA    /* Master Clear (w) */
#define DMA2_CLR_MASK_REG       0xDC    /* Clear Mask */
#define DMA2_MASK_ALL_REG       0xDE    /* all-channels mask (w) */

/* I/O to memory, no autoinit, increment, single mode */
#define DMA_MODE_READ           0x44
/* memory to I/O, no autoinit, increment, single mode */
#define DMA_MODE_WRITE          0x48
/* pass thru DREQ->HRQ, DACK<-HLDA only */
#define DMA_MODE_CASCADE        0xC0

#define DMA_AUTOINIT            0x10

void isa_dma_init(void)
{
	outb(0, DMA1_RESET_REG);
	outb(0, DMA2_RESET_REG);
	outb(DMA_MODE_CASCADE, DMA2_MODE_REG);
	outb(0, DMA2_MASK_REG);
}
