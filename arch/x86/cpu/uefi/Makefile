#
# Copyright (c) 2011 The Chromium OS Authors.
#
# (C) Copyright 2008
# Graeme Russ, graeme.russ@gmail.com.
#
# (C) Copyright 2006
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# (C) Copyright 2002
# Daniel Engström, Omicron Ceti AB, daniel@omicron.se.
#
# SPDX-License-Identifier:	GPL-2.0+
#

obj-y += uefi.o
obj-y += tables.o
obj-y += sdram.o
obj-$(CONFIG_PCI) += pci.o


# XXX makefile-wide CFLAGS?
CFLAGS_uefi.o += $(CFLAGS_EFI)
CFLAGS_tables.o += $(CFLAGS_EFI)
CFLAGS_sdram.o += $(CFLAGS_EFI)

head-$(CONFIG_SYS_UEFI) += $(EFI_CRT_OBJS)
extra-$(CONFIG_SYS_UEFI) += $(EFI_LIBS)
