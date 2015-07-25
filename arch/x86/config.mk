#
# (C) Copyright 2000-2002
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.
#
# SPDX-License-Identifier:	GPL-2.0+
#

CONFIG_STANDALONE_LOAD_ADDR ?= 0x40000

PLATFORM_CPPFLAGS += -fno-strict-aliasing
PLATFORM_CPPFLAGS += -fomit-frame-pointer
PF_CPPFLAGS_X86   := $(call cc-option, -fno-toplevel-reorder, \
		       $(call cc-option, -fno-unit-at-a-time)) \
		     $(call cc-option, -mpreferred-stack-boundary=2)

PLATFORM_CPPFLAGS += $(PF_CPPFLAGS_X86)
PLATFORM_CPPFLAGS += -fno-dwarf2-cfi-asm
PLATFORM_CPPFLAGS += -march=i386 -m32

PLATFORM_RELFLAGS += -ffunction-sections -fvisibility=hidden

PLATFORM_LDFLAGS += -Bsymbolic -Bsymbolic-functions

LDFLAGS_FINAL += --wrap=__divdi3 --wrap=__udivdi3
LDFLAGS_FINAL += --wrap=__moddi3 --wrap=__umoddi3

LDFLAGS_EFI := -Bsymbolic -Bsymbolic-functions -znocombreloc -shared \
	--no-undefined

OBJCOPYFLAGS_EFI := -j .text -j .sdata -j .data -j .dynamic -j .dynsym \
	-j .rel -j .rela -j .reloc

CFLAGS_NON_EFI := -mregparm=3
CFLAGS_EFI := -fpic -fshort-wchar $(call cc-option, -mno-red-zone)

EFIARCH=ia32

LDSCRIPT_EFI := $(srctree)/$(CPUDIR)/efi/elf_$(EFIARCH)_efi.lds
OBJCOPYFLAGS_EFI += --target=efi-app-$(EFIARCH)

ifeq ($(CONFIG_ARCH_EFI),y)

PLATFORM_CPPFLAGS += $(CFLAGS_EFI)
PLATFORM_LDFLAGS += -m elf_i386
LDFLAGS_FINAL += -znocombreloc -shared
OBJCOPYFLAGS_EFI += --target=efi-app-$(EFIARCH)
LDSCRIPT := $(LDSCRIPT_EFI)

else

PLATFORM_CPPFLAGS += $(CFLAGS_NON_EFI)
PLATFORM_LDFLAGS += --emit-relocs -m elf_i386
LDFLAGS_FINAL += --gc-sections -pie

endif
