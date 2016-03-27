/*
 * (C) Copyright 2000-2004
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef BLK_H
#define BLK_H

#ifdef CONFIG_SYS_64BIT_LBA
typedef uint64_t lbaint_t;
#define LBAFlength "ll"
#else
typedef ulong lbaint_t;
#define LBAFlength "l"
#endif
#define LBAF "%" LBAFlength "x"
#define LBAFU "%" LBAFlength "u"

/* Interface types: */
enum if_type {
	IF_TYPE_UNKNOWN = 0,
	IF_TYPE_IDE,
	IF_TYPE_SCSI,
	IF_TYPE_ATAPI,
	IF_TYPE_USB,
	IF_TYPE_DOC,
	IF_TYPE_MMC,
	IF_TYPE_SD,
	IF_TYPE_SATA,
	IF_TYPE_HOST,
	IF_TYPE_SYSTEMACE,

	IF_TYPE_COUNT,			/* Number of interface types */
};

/*
 * With driver model (CONFIG_BLK) this is uclass platform data, accessible
 * with dev_get_uclass_platdata(dev)
 */
struct blk_desc {
	/*
	 * TODO: With driver model we should be able to use the parent
	 * device's uclass instead.
	 */
	enum if_type	if_type;	/* type of the interface */
	int		devnum;		/* device number */
	unsigned char	part_type;	/* partition type */
	unsigned char	target;		/* target SCSI ID */
	unsigned char	lun;		/* target LUN */
	unsigned char	hwpart;		/* HW partition, e.g. for eMMC */
	unsigned char	type;		/* device type */
	unsigned char	removable;	/* removable device */
#ifdef CONFIG_LBA48
	/* device can use 48bit addr (ATA/ATAPI v7) */
	unsigned char	lba48;
#endif
	lbaint_t	lba;		/* number of blocks */
	unsigned long	blksz;		/* block size */
	int		log2blksz;	/* for convenience: log2(blksz) */
	char		vendor[40+1];	/* IDE model, SCSI Vendor */
	char		product[20+1];	/* IDE Serial no, SCSI product */
	char		revision[8+1];	/* firmware revision */
#ifdef CONFIG_BLK
	struct udevice *bdev;
#else
	unsigned long	(*block_read)(struct blk_desc *block_dev,
				      lbaint_t start,
				      lbaint_t blkcnt,
				      void *buffer);
	unsigned long	(*block_write)(struct blk_desc *block_dev,
				       lbaint_t start,
				       lbaint_t blkcnt,
				       const void *buffer);
	unsigned long	(*block_erase)(struct blk_desc *block_dev,
				       lbaint_t start,
				       lbaint_t blkcnt);
	void		*priv;		/* driver private struct pointer */
#endif
};

#define BLOCK_CNT(size, blk_desc) (PAD_COUNT(size, blk_desc->blksz))
#define PAD_TO_BLOCKSIZE(size, blk_desc) \
	(PAD_SIZE(size, blk_desc->blksz))

#ifdef CONFIG_BLK
struct udevice;

/* Operations on block devices */
struct blk_ops {
	/**
	 * read() - read from a block device
	 *
	 * @dev:	Device to read from
	 * @start:	Start block number to read (0=first)
	 * @blkcnt:	Number of blocks to read
	 * @buffer:	Destination buffer for data read
	 * @return number of blocks read, or -ve error number (see the
	 * IS_ERR_VALUE() macro
	 */
	unsigned long (*read)(struct udevice *dev, lbaint_t start,
			      lbaint_t blkcnt, void *buffer);

	/**
	 * write() - write to a block device
	 *
	 * @dev:	Device to write to
	 * @start:	Start block number to write (0=first)
	 * @blkcnt:	Number of blocks to write
	 * @buffer:	Source buffer for data to write
	 * @return number of blocks written, or -ve error number (see the
	 * IS_ERR_VALUE() macro
	 */
	unsigned long (*write)(struct udevice *dev, lbaint_t start,
			       lbaint_t blkcnt, const void *buffer);

	/**
	 * erase() - erase a section of a block device
	 *
	 * @dev:	Device to (partially) erase
	 * @start:	Start block number to erase (0=first)
	 * @blkcnt:	Number of blocks to erase
	 * @return number of blocks erased, or -ve error number (see the
	 * IS_ERR_VALUE() macro
	 */
	unsigned long (*erase)(struct udevice *dev, lbaint_t start,
			       lbaint_t blkcnt);
};

#define blk_get_ops(dev)	((struct blk_ops *)(dev)->driver->ops)

/*
 * These functions should take struct udevice instead of struct blk_desc,
 * but this is convenient for migration to driver model. Add a 'd' prefix
 * to the function operations, so that blk_read(), etc. can be reserved for
 * functions with the correct arguments.
 */
unsigned long blk_dread(struct blk_desc *block_dev, lbaint_t start,
			lbaint_t blkcnt, void *buffer);
unsigned long blk_dwrite(struct blk_desc *block_dev, lbaint_t start,
			 lbaint_t blkcnt, const void *buffer);
unsigned long blk_derase(struct blk_desc *block_dev, lbaint_t start,
			 lbaint_t blkcnt);

/**
 * blk_get_device() - Find and probe a block device ready for use
 *
 * @if_type:	Interface type (enum if_type_t)
 * @devnum:	Device number (specific to each interface type)
 * @devp:	the device, if found
 * @return - if found, -ENODEV if no device found, or other -ve error value
 */
int blk_get_device(int if_type, int devnum, struct udevice **devp);

/**
 * blk_first_device() - Find the first device for a given interface
 *
 * The device is probed ready for use
 *
 * @devnum:	Device number (specific to each interface type)
 * @devp:	the device, if found
 * @return 0 if found, -ENODEV if no device, or other -ve error value
 */
int blk_first_device(int if_type, struct udevice **devp);

/**
 * blk_next_device() - Find the next device for a given interface
 *
 * This can be called repeatedly after blk_first_device() to iterate through
 * all devices of the given interface type.
 *
 * The device is probed ready for use
 *
 * @devp:	On entry, the previous device returned. On exit, the next
 *		device, if found
 * @return 0 if found, -ENODEV if no device, or other -ve error value
 */
int blk_next_device(struct udevice **devp);

/**
 * blk_create_device() - Create a new block device
 *
 * @parent:	Parent of the new device
 * @drv_name:	Driver name to use for the block device
 * @name:	Name for the device
 * @if_type:	Interface type (enum if_type_t)
 * @devnum:	Device number, specific to the interface type
 * @blksz:	Block size of the device in bytes (typically 512)
 * @size:	Total size of the device in bytes
 * @devp:	the new device (which has not been probed)
 */
int blk_create_device(struct udevice *parent, const char *drv_name,
		      const char *name, int if_type, int devnum, int blksz,
		      lbaint_t size, struct udevice **devp);

/**
 * blk_prepare_device() - Prepare a block device for use
 *
 * This reads partition information from the device if supported.
 *
 * @dev:	Device to prepare
 * @return 0 if ok, -ve on error
 */
int blk_prepare_device(struct udevice *dev);

/**
 * blk_unbind_all() - Unbind all device of the given interface type
 *
 * The devices are removed and then unbound.
 *
 * @if_type:	Interface type to unbind
 * @return 0 if OK, -ve on error
 */
int blk_unbind_all(int if_type);

#else
#include <errno.h>
/*
 * These functions should take struct udevice instead of struct blk_desc,
 * but this is convenient for migration to driver model. Add a 'd' prefix
 * to the function operations, so that blk_read(), etc. can be reserved for
 * functions with the correct arguments.
 */
static inline ulong blk_dread(struct blk_desc *block_dev, lbaint_t start,
			      lbaint_t blkcnt, void *buffer)
{
	/*
	 * We could check if block_read is NULL and return -ENOSYS. But this
	 * bloats the code slightly (cause some board to fail to build), and
	 * it would be an error to try an operation that does not exist.
	 */
	return block_dev->block_read(block_dev, start, blkcnt, buffer);
}

static inline ulong blk_dwrite(struct blk_desc *block_dev, lbaint_t start,
			       lbaint_t blkcnt, const void *buffer)
{
	return block_dev->block_write(block_dev, start, blkcnt, buffer);
}

static inline ulong blk_derase(struct blk_desc *block_dev, lbaint_t start,
			       lbaint_t blkcnt)
{
	return block_dev->block_erase(block_dev, start, blkcnt);
}

int blk_list_part(enum if_type if_type);
void blk_list_devices(enum if_type if_type);
int blk_show_device(enum if_type if_type, int devnum);
int blk_print_device_num(enum if_type if_type, int devnum);
int blk_print_part_devnum(enum if_type if_type, int devnum);

ulong blk_read_devnum(enum if_type if_type, int devnum, lbaint_t start,
		      lbaint_t blkcnt, void *buffer);
ulong blk_write_devnum(enum if_type if_type, int devnum, lbaint_t start,
		       lbaint_t blkcnt, const void *buffer);

/**
 * struct blk_driver - Driver for block interface types
 *
 * This provides access to the block devices for each interface type. One
 * driver should be provided using U_BOOT_LEGACY_BLK() for each interface
 * type that is to be supported.
 *
 * @if_typename:	Interface type name
 * @if_type:		Interface type
 * @max_devs:		Maximum number of devices supported
 * @desc:		Pointer to list of devices for this interface type,
 *			or NULL to use @get_dev() instead
 */
struct blk_driver {
	const char *if_typename;
	enum if_type if_type;
	int max_devs;
	struct blk_desc *desc;
	/**
	 * get_dev() - get a pointer to a block device given its number
	 *
	 * Each interface allocates its own devices and typically
	 * struct blk_desc is contained with the interface's data structure.
	 * There is no global numbering for block devices. This method allows
	 * the device for an interface type to be obtained when @desc is NULL.
	 *
	 * @devnum:	Device number (0 for first device on that interface,
	 *		1 for second, etc.
	 * @descp:	Returns pointer to the block device on success
	 * @return 0 if OK, -ve on error
	 */
	int (*get_dev)(int devnum, struct blk_desc **descp);

	/**
	 * select_hwpart() - Select a hardware partition
	 *
	 * Some devices (e.g. MMC) can support partitioning at the hardware
	 * level. This is quite separate from the normal idea of
	 * software-based partitions. MMC hardware partitions must be
	 * explicitly selected. Once selected only the region of the device
	 * covered by that partition is accessible.
	 *
	 * The MMC standard provides for two boot partitions (numbered 1 and 2),
	 * rpmb (3), and up to 4 addition general-purpose partitions (4-7).
	 *
	 * @desc:	Block device descriptor
	 * @hwpart:	Hardware partition number to select. 0 means the raw
	 *		device, 1 is the first partition, 2 is the second, etc.
	 * @return 0 if OK, other value for an error
	 */
	int (*select_hwpart)(struct blk_desc *desc, int hwpart);
};

/*
 * Declare a new U-Boot legacy block driver. New drivers should use driver
 * model (UCLASS_BLK).
 */
#define U_BOOT_LEGACY_BLK(__name)					\
	ll_entry_declare(struct blk_driver, __name, part_driver)

struct blk_desc *blk_get_devnum_by_type(enum if_type if_type, int devnum);
struct blk_desc *blk_get_devnum_by_typename(const char *if_typename,
					    int devnum);
int blk_select_hwpart(struct blk_desc *desc, int hwpart);

#endif /* !CONFIG_BLK */

#endif
