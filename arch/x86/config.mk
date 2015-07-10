#
# (C) Copyright 2000-2002
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# SPDX-License-Identifier:	GPL-2.0+
#

CONFIG_STANDALONE_LOAD_ADDR ?= 0x40000

PLATFORM_CPPFLAGS += -fno-strict-aliasing
PLATFORM_CPPFLAGS += -mregparm=3
PLATFORM_CPPFLAGS += -fomit-frame-pointer
PF_CPPFLAGS_X86   := $(call cc-option, -fno-toplevel-reorder, \
		       $(call cc-option, -fno-unit-at-a-time)) \
		     $(call cc-option, -mpreferred-stack-boundary=2)
PLATFORM_CPPFLAGS += $(PF_CPPFLAGS_X86)
PLATFORM_CPPFLAGS += -fno-dwarf2-cfi-asm
PLATFORM_CPPFLAGS += -march=i386 -m32

PLATFORM_RELFLAGS += -ffunction-sections -fvisibility=hidden

LDFLAGS_FINAL += --gc-sections -pie
LDFLAGS_FINAL += --wrap=__divdi3 --wrap=__udivdi3
LDFLAGS_FINAL += --wrap=__moddi3 --wrap=__umoddi3

ifeq ($(CONFIG_SYS_UEFI),y)

ifeq ($(ARCH),x86)
EFILIB=/usr/lib32
EFIARCH=ia32
else
EFILIB=/usr/lib
EFIARCH=x86_64
endif

EFI_LIBS := $(EFILIB)/libefi.a $(EFILIB)/libgnuefi.a
EFI_CRT_OBJS := $(EFILIB)/crt0-efi-$(EFIARCH).o
U_BOOT_LDS := $(EFILIB)/elf_$(EFIARCH)_efi.lds
EFIINC=/usr/include/efi
CFLAGS_EFI =  -I$(EFIINC)/$(EFIARCH) -I$(EFIINC)/protocol -DEFI_FUNCTION_WRAPPER

LDFLAGS_FINAL := -nostdlib -znocombreloc -shared -Bsymbolic

PLATFORM_LDFLAGS += -m elf_i386

else

PLATFORM_LDFLAGS += --emit-relocs -Bsymbolic -Bsymbolic-functions

endif
