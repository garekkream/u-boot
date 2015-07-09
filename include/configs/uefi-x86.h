/*
 * Copyright (c) 2011 The Chromium OS Authors.
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include "x86-common.h"

#undef CONFIG_CMD_SF
#undef CONFIG_CMD_SF_TEST

#define CONFIG_ENV_SECT_SIZE            0x1000
#define CONFIG_ENV_OFFSET               0x003f8000

#undef CONFIG_SCSI_AHCI
#undef CONFIG_LIBATA
#undef CONFIG_LBA48
#undef CONFIG_SYS_64BIT_LBA

#undef CONFIG_SYS_SCSI_MAX_SCSI_ID
#undef CONFIG_SYS_SCSI_MAX_LUN
#undef CONFIG_SYS_SCSI_MAX_DEVICE
#undef CONFIG_SCSI_AHCI_PLAT

#undef CONFIG_TPM
#undef CONFIG_TPM_TIS_LPC
#undef CONFIG_TPM_TIS_BASE_ADDRESS


#endif
