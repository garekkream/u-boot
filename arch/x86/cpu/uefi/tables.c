/*
 * TODO(stoltz) copied from coreboot, adapt to UEFI
 * This file is part of the libpayload project.
 *
 * Copyright (C) 2008 Advanced Micro Devices, Inc.
 * Copyright (C) 2009 coresystems GmbH
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 */

#include <common.h>
#include <net.h>
#include <asm/arch/sysinfo.h>
#include <asm/arch/tables.h>

/*
 * This needs to be in the .data section so that it's copied over during
 * relocation. By default it's put in the .bss section which is simply filled
 * with zeroes when transitioning from "ROM", which is really RAM, to other
 * RAM.
 */
struct sysinfo_t lib_sysinfo __attribute__((section(".data")));

/*
 * Some of this is x86 specific, and the rest of it is generic. Right now,
 * since we only support x86, we'll avoid trying to make lots of infrastructure
 * we don't need. If in the future, we want to use UEFI on some other
 * architecture, then take out the generic parsing code and move it elsewhere.
 */

/* === Parsing code === */
/* This is the generic parsing code. */

static void uefi_parse_memory(unsigned char *ptr, struct sysinfo_t *info)
{
	struct uefi_memory *mem = (struct uefi_memory *)ptr;
	int count = MEM_RANGE_COUNT(mem);
	int i;

	if (count > SYSINFO_MAX_MEM_RANGES)
		count = SYSINFO_MAX_MEM_RANGES;

	info->n_memranges = 0;

	for (i = 0; i < count; i++) {
		struct uefi_memory_range *range =
		    (struct uefi_memory_range *)MEM_RANGE_PTR(mem, i);

		info->memrange[info->n_memranges].base =
		    UNPACK_UEFI64(range->start);

		info->memrange[info->n_memranges].size =
		    UNPACK_UEFI64(range->size);

		info->memrange[info->n_memranges].type = range->type;

		info->n_memranges++;
	}
}

static void uefi_parse_serial(unsigned char *ptr, struct sysinfo_t *info)
{
	struct uefi_serial *ser = (struct uefi_serial *)ptr;
	info->serial = ser;
}

static void uefi_parse_vbnv(unsigned char *ptr, struct sysinfo_t *info)
{
	struct uefi_vbnv *vbnv = (struct uefi_vbnv *)ptr;

	info->vbnv_start = vbnv->vbnv_start;
	info->vbnv_size = vbnv->vbnv_size;
}

static void uefi_parse_gpios(unsigned char *ptr, struct sysinfo_t *info)
{
	int i;
	struct uefi_gpios *gpios = (struct uefi_gpios *)ptr;

	info->num_gpios = (gpios->count < SYSINFO_MAX_GPIOS) ?
				(gpios->count) : SYSINFO_MAX_GPIOS;

	for (i = 0; i < info->num_gpios; i++)
		info->gpios[i] = gpios->gpios[i];
}

static void uefi_parse_vdat(unsigned char *ptr, struct sysinfo_t *info)
{
	struct uefi_vdat *vdat = (struct uefi_vdat *) ptr;

	info->vdat_addr = vdat->vdat_addr;
	info->vdat_size = vdat->vdat_size;
}

static void uefi_parse_tstamp(unsigned char *ptr, struct sysinfo_t *info)
{
	info->tstamp_table = ((struct uefi_uefimem_tab *)ptr)->uefimem_tab;
}

static void uefi_parse_uefimem_cons(unsigned char *ptr, struct sysinfo_t *info)
{
	info->uefimem_cons = ((struct uefi_uefimem_tab *)ptr)->uefimem_tab;
}

static void uefi_parse_framebuffer(unsigned char *ptr, struct sysinfo_t *info)
{
	info->framebuffer = (struct uefi_framebuffer *)ptr;
}

static void uefi_parse_string(unsigned char *ptr, char **info)
{
	*info = (char *)((struct uefi_string *)ptr)->string;
}

static int uefi_parse_header(void *addr, int len, struct sysinfo_t *info)
{
	struct uefi_header *header;
	unsigned char *ptr = (unsigned char *)addr;
	int i;

	for (i = 0; i < len; i += 16, ptr += 16) {
		header = (struct uefi_header *)ptr;
		if (!strncmp((const char *)header->signature, "LBIO", 4))
			break;
	}

	/* We walked the entire space and didn't find anything. */
	if (i >= len)
		return -1;

	if (!header->table_bytes)
		return 0;

	/* Make sure the checksums match. */
	if (!ip_checksum_ok(header, sizeof(*header)))
		return -1;

	if (compute_ip_checksum(ptr + sizeof(*header), header->table_bytes) !=
	    header->table_checksum)
		return -1;

	/* Now, walk the tables. */
	ptr += header->header_bytes;

	/* Inintialize some fields to sentinel values. */
	info->vbnv_start = info->vbnv_size = (uint32_t)(-1);

	for (i = 0; i < header->table_entries; i++) {
		struct uefi_record *rec = (struct uefi_record *)ptr;

		/* We only care about a few tags here (maybe more later). */
		switch (rec->tag) {
		case UEFI_TAG_FORWARD:
			return uefi_parse_header(
				(void *)(unsigned long)
				((struct uefi_forward *)rec)->forward,
				len, info);
			continue;
		case UEFI_TAG_MEMORY:
			uefi_parse_memory(ptr, info);
			break;
		case UEFI_TAG_SERIAL:
			uefi_parse_serial(ptr, info);
			break;
		case UEFI_TAG_VERSION:
			uefi_parse_string(ptr, &info->version);
			break;
		case UEFI_TAG_EXTRA_VERSION:
			uefi_parse_string(ptr, &info->extra_version);
			break;
		case UEFI_TAG_BUILD:
			uefi_parse_string(ptr, &info->build);
			break;
		case UEFI_TAG_COMPILE_TIME:
			uefi_parse_string(ptr, &info->compile_time);
			break;
		case UEFI_TAG_COMPILE_BY:
			uefi_parse_string(ptr, &info->compile_by);
			break;
		case UEFI_TAG_COMPILE_HOST:
			uefi_parse_string(ptr, &info->compile_host);
			break;
		case UEFI_TAG_COMPILE_DOMAIN:
			uefi_parse_string(ptr, &info->compile_domain);
			break;
		case UEFI_TAG_COMPILER:
			uefi_parse_string(ptr, &info->compiler);
			break;
		case UEFI_TAG_LINKER:
			uefi_parse_string(ptr, &info->linker);
			break;
		case UEFI_TAG_ASSEMBLER:
			uefi_parse_string(ptr, &info->assembler);
			break;
		/*
		 * FIXME we should warn on serial if UEFI set up a
		 * framebuffer buf the payload does not know about it.
		 */
		case UEFI_TAG_FRAMEBUFFER:
			uefi_parse_framebuffer(ptr, info);
			break;
		case UEFI_TAG_GPIO:
			uefi_parse_gpios(ptr, info);
			break;
		case UEFI_TAG_VDAT:
			uefi_parse_vdat(ptr, info);
			break;
		case UEFI_TAG_TIMESTAMPS:
			uefi_parse_tstamp(ptr, info);
			break;
		case UEFI_TAG_UEFIMEM_CONSOLE:
			uefi_parse_uefimem_cons(ptr, info);
			break;
		case UEFI_TAG_VBNV:
			uefi_parse_vbnv(ptr, info);
			break;
		}

		ptr += rec->size;
	}

	return 1;
}

/* == Architecture specific == */
/* This is the x86 specific stuff. */

int get_uefi_info(struct sysinfo_t *info)
{
	int ret = uefi_parse_header((void *)0x00000000, 0x1000, info);

	if (ret != 1)
		ret = uefi_parse_header((void *)0x000f0000, 0x1000, info);

	return (ret == 1) ? 0 : -1;
}
