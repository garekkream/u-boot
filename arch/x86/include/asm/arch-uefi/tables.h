/*
 * This file is part of the libpayload project.
 *
 * Copyright (C) 2008 Advanced Micro Devices, Inc.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 */

// TODO(stoltz) copied from coreboot, adapt to UEFI
#ifndef _UEFI_TABLES_H
#define _UEFI_TABLES_H

#include <linux/compiler.h>

struct uefiuint64 {
	u32 lo;
	u32 hi;
};

struct uefi_header {
	u8 signature[4];
	u32 header_bytes;
	u32 header_checksum;
	u32 table_bytes;
	u32 table_checksum;
	u32 table_entries;
};

struct uefi_record {
	u32 tag;
	u32 size;
};

#define UEFI_TAG_UNUSED     0x0000
#define UEFI_TAG_MEMORY     0x0001

struct uefi_memory_range {
	struct uefiuint64 start;
	struct uefiuint64 size;
	u32 type;
};

#define UEFI_MEM_RAM          1
#define UEFI_MEM_RESERVED     2
#define UEFI_MEM_ACPI         3
#define UEFI_MEM_NVS          4
#define UEFI_MEM_UNUSABLE     5
#define UEFI_MEM_VENDOR_RSVD  6
#define UEFI_MEM_TABLE       16

struct uefi_memory {
	u32 tag;
	u32 size;
	struct uefi_memory_range map[0];
};

#define UEFI_TAG_HWRPB      0x0002

struct uefi_hwrpb {
	u32 tag;
	u32 size;
	u64 hwrpb;
};

#define UEFI_TAG_MAINBOARD  0x0003

struct uefi_mainboard {
	u32 tag;
	u32 size;
	u8 vendor_idx;
	u8 part_number_idx;
	u8 strings[0];
};

#define UEFI_TAG_VERSION        0x0004
#define UEFI_TAG_EXTRA_VERSION  0x0005
#define UEFI_TAG_BUILD          0x0006
#define UEFI_TAG_COMPILE_TIME   0x0007
#define UEFI_TAG_COMPILE_BY     0x0008
#define UEFI_TAG_COMPILE_HOST   0x0009
#define UEFI_TAG_COMPILE_DOMAIN 0x000a
#define UEFI_TAG_COMPILER       0x000b
#define UEFI_TAG_LINKER         0x000c
#define UEFI_TAG_ASSEMBLER      0x000d

struct uefi_string {
	u32 tag;
	u32 size;
	u8 string[0];
};

#define UEFI_TAG_SERIAL         0x000f

struct uefi_serial {
	u32 tag;
	u32 size;
#define UEFI_SERIAL_TYPE_IO_MAPPED     1
#define UEFI_SERIAL_TYPE_MEMORY_MAPPED 2
	u32 type;
	u32 baseaddr;
	u32 baud;
};

#define UEFI_TAG_CONSOLE       0x00010

struct uefi_console {
	u32 tag;
	u32 size;
	u16 type;
};

#define UEFI_TAG_CONSOLE_SERIAL8250 0
#define UEFI_TAG_CONSOLE_VGA        1 /* OBSOLETE */
#define UEFI_TAG_CONSOLE_BTEXT      2 /* OBSOLETE */
#define UEFI_TAG_CONSOLE_LOGBUF     3
#define UEFI_TAG_CONSOLE_SROM       4 /* OBSOLETE */
#define UEFI_TAG_CONSOLE_EHCI       5

#define UEFI_TAG_FORWARD       0x00011

struct uefi_forward {
	u32 tag;
	u32 size;
	u64 forward;
};

#define UEFI_TAG_FRAMEBUFFER      0x0012
struct uefi_framebuffer {
	u32 tag;
	u32 size;

	u64 physical_address;
	u32 x_resolution;
	u32 y_resolution;
	u32 bytes_per_line;
	u8 bits_per_pixel;
	u8 red_mask_pos;
	u8 red_mask_size;
	u8 green_mask_pos;
	u8 green_mask_size;
	u8 blue_mask_pos;
	u8 blue_mask_size;
	u8 reserved_mask_pos;
	u8 reserved_mask_size;
};

#define UEFI_TAG_GPIO 0x0013
#define GPIO_MAX_NAME_LENGTH 16
struct uefi_gpio {
	u32 port;
	u32 polarity;
	u32 value;
	u8 name[GPIO_MAX_NAME_LENGTH];
};

struct uefi_gpios {
	u32 tag;
	u32 size;

	u32 count;
	struct uefi_gpio gpios[0];
};

#define UEFI_TAG_FDT	0x0014
struct uefi_fdt {
	uint32_t tag;
	uint32_t size;	/* size of the entire entry */
	/* the actual FDT gets placed here */
};

#define UEFI_TAG_VDAT	0x0015
struct uefi_vdat {
	uint32_t tag;
	uint32_t size;	/* size of the entire entry */
	void	 *vdat_addr;
	uint32_t vdat_size;
};

#define UEFI_TAG_TIMESTAMPS	0x0016
#define UEFI_TAG_UEFIMEM_CONSOLE	0x0017
#define UEFI_TAG_MRC_CACHE	0x0018
struct uefi_uefimem_tab {
	uint32_t tag;
	uint32_t size;
	void   *uefimem_tab;
};

#define UEFI_TAG_VBNV		0x0019
struct uefi_vbnv {
	uint32_t tag;
	uint32_t size;
	uint32_t vbnv_start;
	uint32_t vbnv_size;
};

#define UEFI_TAG_CMOS_OPTION_TABLE 0x00c8
struct uefi_cmos_option_table {
	u32 tag;
	u32 size;
	u32 header_length;
};

#define UEFI_TAG_OPTION         0x00c9
#define CMOS_MAX_NAME_LENGTH    32
struct uefi_cmos_entries {
	u32 tag;
	u32 size;
	u32 bit;
	u32 length;
	u32 config;
	u32 config_id;
	u8 name[CMOS_MAX_NAME_LENGTH];
};


#define UEFI_TAG_OPTION_ENUM    0x00ca
#define CMOS_MAX_TEXT_LENGTH 32
struct uefi_cmos_enums {
	u32 tag;
	u32 size;
	u32 config_id;
	u32 value;
	u8 text[CMOS_MAX_TEXT_LENGTH];
};

#define UEFI_TAG_OPTION_DEFAULTS 0x00cb
#define CMOS_IMAGE_BUFFER_SIZE 128
struct uefi_cmos_defaults {
	u32 tag;
	u32 size;
	u32 name_length;
	u8 name[CMOS_MAX_NAME_LENGTH];
	u8 default_set[CMOS_IMAGE_BUFFER_SIZE];
};

#define UEFI_TAG_OPTION_CHECKSUM 0x00cc
#define CHECKSUM_NONE	0
#define CHECKSUM_PCBIOS	1
struct	uefi_cmos_checksum {
	u32 tag;
	u32 size;
	u32 range_start;
	u32 range_end;
	u32 location;
	u32 type;
};

/* Helpful macros */

#define MEM_RANGE_COUNT(_rec) \
	(((_rec)->size - sizeof(*(_rec))) / sizeof((_rec)->map[0]))

#define MEM_RANGE_PTR(_rec, _idx) \
	(((u8 *) (_rec)) + sizeof(*(_rec)) \
	+ (sizeof((_rec)->map[0]) * (_idx)))

#define MB_VENDOR_STRING(_mb) \
	(((unsigned char *) ((_mb)->strings)) + (_mb)->vendor_idx)

#define MB_PART_STRING(_mb) \
	(((unsigned char *) ((_mb)->strings)) + (_mb)->part_number_idx)

#define UNPACK_UEFI64(_in) \
	((((u64) _in.hi) << 32) | _in.lo)

struct sysinfo_t;

int get_uefi_info(struct sysinfo_t *info);

#define UEFIMEM_TOC_RESERVED      512
#define MAX_UEFIMEM_ENTRIES       16
#define UEFIMEM_MAGIC             0x434f5245

struct uefimem_entry {
	u32 magic;
	u32 id;
	u64 base;
	u64 size;
} __packed;

#define UEFIMEM_ID_FREESPACE      0x46524545
#define UEFIMEM_ID_GDT            0x4c474454
#define UEFIMEM_ID_ACPI           0x41435049
#define UEFIMEM_ID_UEFITABLE        0x43425442
#define UEFIMEM_ID_PIRQ           0x49525154
#define UEFIMEM_ID_MPTABLE        0x534d5054
#define UEFIMEM_ID_RESUME         0x5245534d
#define UEFIMEM_ID_RESUME_SCRATCH 0x52455343
#define UEFIMEM_ID_SMBIOS         0x534d4254
#define UEFIMEM_ID_TIMESTAMP      0x54494d45
#define UEFIMEM_ID_MRCDATA        0x4d524344
#define UEFIMEM_ID_CONSOLE        0x434f4e53
#define UEFIMEM_ID_NONE           0x00000000

#endif
