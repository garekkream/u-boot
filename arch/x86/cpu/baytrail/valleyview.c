/*
 * Copyright (C) 2014, Bin Meng <bmeng.cn@gmail.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#define DEBUG
#include <common.h>
#include <dm.h>
#include <mmc.h>
#include <pci_ids.h>
#include <asm/interrupt.h>
#include <asm/irq.h>
#include <asm/isa_dma.h>
#include <asm/ioapic.h>
#include <asm/pirq_routing.h>
#include <asm/post.h>

static struct pci_device_id mmc_supported[] = {
	{ PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_VALLEYVIEW_SDIO },
	{ PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_VALLEYVIEW_SDCARD },
};

int cpu_mmc_init(bd_t *bis)
{
	return pci_mmc_init("ValleyView SDHCI", mmc_supported,
			    ARRAY_SIZE(mmc_supported));
}

#ifndef CONFIG_ARCH_EFI
int arch_cpu_init(void)
{
	int ret;

	post_code(POST_CPU_INIT);
#ifdef CONFIG_SYS_X86_TSC_TIMER
	timer_set_base(rdtsc());
#endif

	ret = x86_cpu_init_f();
	if (ret)
		return ret;

	return 0;
}
#endif

/* PIC IRQ settings. */
#define PIRQ_PIC_IRQ3			0x3
#define PIRQ_PIC_IRQ4			0x4
#define PIRQ_PIC_IRQ5			0x5
#define PIRQ_PIC_IRQ6			0x6
#define PIRQ_PIC_IRQ7			0x7
#define PIRQ_PIC_IRQ9			0x9
#define PIRQ_PIC_IRQ10			0xa
#define PIRQ_PIC_IRQ11			0xb
#define PIRQ_PIC_IRQ12			0xc
#define PIRQ_PIC_IRQ14			0xe
#define PIRQ_PIC_IRQ15			0xf
#define PIRQ_PIC_IRQDISABLE		0x80
#define PIRQ_PIC_UNKNOWN_UNUSED		0xff

#define PCI_DEV_PIRQ_ROUTE(dev_, a_, b_, c_, d_) \
	[dev_] = ((PIRQ ## d_) << 12) | ((PIRQ ## c_) << 8) | \
	         ((PIRQ ## b_) <<  4) | ((PIRQ ## a_) << 0)

#define PIRQ_PIC(pirq_, pic_irq_) \
	[PIRQ ## pirq_] = PIRQ_PIC_IRQ ## pic_irq_

#define NUM_OF_PCI_DEVS 32
#define NUM_PIRQS   8

struct baytrail_irq_route {
	/* Per device configuration. */
	uint16_t pcidev[NUM_OF_PCI_DEVS];
	/* Route path for each internal PIRQx in PIC mode. */
	uint8_t  pic[NUM_PIRQS];
};

static struct baytrail_irq_route irq_info = {
	{
		PCI_DEV_PIRQ_ROUTE(0x02, A, A, A, A),
		PCI_DEV_PIRQ_ROUTE(0x10, D, E, F, G),
		PCI_DEV_PIRQ_ROUTE(0x11, B, A, A, A),
		PCI_DEV_PIRQ_ROUTE(0x12, C, A, A, A),
		PCI_DEV_PIRQ_ROUTE(0x13, D, A, A, A),
		PCI_DEV_PIRQ_ROUTE(0x14, E, A, A, A),
		PCI_DEV_PIRQ_ROUTE(0x15, F, A, A, A),
		PCI_DEV_PIRQ_ROUTE(0x17, F, A, A, A),
		PCI_DEV_PIRQ_ROUTE(0x18, B, A, D, C),
		PCI_DEV_PIRQ_ROUTE(0x1a, F, A, A, A),
		PCI_DEV_PIRQ_ROUTE(0x1b, G, A, A, A),
		PCI_DEV_PIRQ_ROUTE(0x1c, E, F, G, H),
		PCI_DEV_PIRQ_ROUTE(0x1d, D, A, A, A),
		PCI_DEV_PIRQ_ROUTE(0x1e, B, D, E, F),
		PCI_DEV_PIRQ_ROUTE(0x1f, H, G, B, C),
	},
	{
	/*
	* Route each PIRQ[A-H] to a PIC IRQ[0-15]
	* Reserved: 0, 1, 2, 8, 13
	* PS2 keyboard: 12
	* ACPI/SCI: 9
	* Floppy: 6
	*/
		PIRQ_PIC(A,  4),
		PIRQ_PIC(B,  5),
		PIRQ_PIC(C,  7),
		PIRQ_PIC(D, 10),
		PIRQ_PIC(E, 11),
		PIRQ_PIC(F, 12),
		PIRQ_PIC(G, 14),
		PIRQ_PIC(H, 15),
	}
};


#define ILB_BASE_ADDRESS		0xfed08000

/* These registers live behind the ILB_BASE_ADDRESS */
#define ACTL				0x00
# define SCIS_MASK				0x07
# define SCIS_IRQ9				0x00
# define SCIS_IRQ10				0x01
# define SCIS_IRQ11				0x02
# define SCIS_IRQ20				0x04
# define SCIS_IRQ21				0x05
# define SCIS_IRQ22				0x06
# define SCIS_IRQ23				0x07

/* Default IO range claimed by the LPC device. The upper bound is exclusive. */
#define LPC_DEFAULT_IO_RANGE_LOWER 0
#define LPC_DEFAULT_IO_RANGE_UPPER 0x1000

#define ILB_ACTL	0
#define ILB_MC			0x4
#define ILB_PIRQA_ROUT	0x8
#define ILB_PIRQB_ROUT	0x9
#define ILB_PIRQC_ROUT	0xA
#define ILB_PIRQD_ROUT	0xB
#define ILB_PIRQE_ROUT	0xC
#define ILB_PIRQF_ROUT	0xD
#define ILB_PIRQG_ROUT	0xE
#define ILB_PIRQH_ROUT	0xF
#define ILB_SERIRQ_CNTL	0x10
#define  SCNT_CONTINUOUS_MODE	(1 << 7)
#define  SCNT_QUIET_MODE		0
#define ILB_IR00 0x20
#define ILB_IR01 0x22
#define ILB_IR02 0x24
#define ILB_IR03 0x26
#define ILB_IR04 0x28
#define ILB_IR05 0x2A
#define ILB_IR06 0x2C
#define ILB_IR07 0x2E
#define ILB_IR08 0x30
#define ILB_IR09 0x32
#define ILB_IR10 0x34
#define ILB_IR11 0x36
#define ILB_IR12 0x38
#define ILB_IR13 0x3A
#define ILB_IR14 0x3C
#define ILB_IR15 0x3E
#define ILB_IR16 0x40
#define ILB_IR17 0x42
#define ILB_IR18 0x44
#define ILB_IR19 0x46
#define ILB_IR20 0x48
#define ILB_IR21 0x4A
#define ILB_IR22 0x4C
#define ILB_IR23 0x4E
#define ILB_IR24 0x50
#define ILB_IR25 0x52
#define ILB_IR26 0x54
#define ILB_IR27 0x56
#define ILB_IR28 0x58
#define ILB_IR29 0x5A
#define ILB_IR30 0x5C
#define ILB_IR31 0x5E
#define ILB_OIC	0x60
#define  SIRQEN (1 << 12)
#define  AEN    (1 << 8)

#define IBASE	0x50

static int sc_enable_ioapic(struct udevice *dev, u8 *ibase)
{
	int i;
	u32 reg32;
	volatile u32 *ioapic_index = (u32 *)(IO_APIC_ADDR);
	volatile u32 *ioapic_data = (u32 *)(IO_APIC_ADDR + 0x10);

	/*
	 * Enable ACPI I/O and power management.
	 * Set SCI IRQ to IRQ9
	 */
	writel(0x100, ibase + ILB_OIC);  /* AEN */
	reg32 = readl(ibase + ILB_OIC); /* Read back per BWG */
	writel(0, ibase + ILB_ACTL);  /* ACTL bit 2:0 SCIS IRQ9 */

	*ioapic_index = 0;
	*ioapic_data = (1 << 25);

	/* affirm full set of redirection table entries ("write once") */
	*ioapic_index = 1;
	reg32 = *ioapic_data;
	*ioapic_index = 1;
	*ioapic_data = reg32;

	*ioapic_index = 0;
	reg32 = *ioapic_data;
	debug("Southbridge APIC ID = %x\n", (reg32 >> 24) & 0x0f);
	if (reg32 != (1 << 25)) {
		debug("APIC Error\n");
		return -EINVAL;
	}

	debug("Dumping IOAPIC registers\n");
	for (i=0; i<3; i++) {
		*ioapic_index = i;
		debug("  reg 0x%04x:", i);
		reg32 = *ioapic_data;
		debug(" 0x%08x\n", reg32);
	}

	*ioapic_index = 3; /* Select Boot Configuration register. */
	*ioapic_data = 1; /* Use Processor System Bus to deliver interrupts. */

	return 0;
}

static void sc_enable_serial_irqs(struct udevice *dev, u8 *ibase)
{
#ifdef SETUPSERIQ /* NOT defined. Remove when the TODO is done. */
	/*
	 * TODO: SERIRQ seems to have a number of problems on baytrail.
	 * With it enabled, we get some spurious interrupts (ps2)
	 * in seabios. It also caused IOCHK# NMIs. Remove it
	 * until we understand how it needs to be configured.
	 */
	u8 reg8;

	/*
	 * Disable the IOCHK# NMI. Let the NMI handler enable it if it needs.
	 */
	reg8 = inb(0x61);
	reg8 &= 0x0f; /* Higher Nibble must be 0 */
	reg8 |= (1 << 3); /* IOCHK# NMI  Disable for now */
	outb(reg8, 0x61);

	setbits_le32(ibase + ILB_OIC, SIRQEN);
	writeb(SCNT_CONTINUOUS_MODE, ibase + ILB_SERIRQ_CNTL);

#if !IS_ENABLED(CONFIG_SERIRQ_CONTINUOUS_MODE)
	/*
	 * SoC requires that the System BIOS first set the SERIRQ logic to
	 * continuous mode operation for at least one frame before switching
	 *  it into quiet mode operation.
	 */
	outb(0x00, 0xED); /* I/O Delay to get the 1 frame */
	writeb(SCNT_QUIET_MODE, ibase + ILB_SERIRQ_CNTL);
#endif
#endif  /* DON'T SET UP IRQS */
}

/**
 * Take an INT_PIN number (0, 1 - 4) and convert
 * it to a string ("NO PIN", "PIN A" - "PIN D")
 *
 * @param pin PCI Interrupt Pin number (0, 1 - 4)
 * @return A string corresponding to the pin number or "Invalid"
 */
const char *pin_to_str(int pin)
{
	const char *str[5] = {
		"NO PIN",
		"PIN A",
		"PIN B",
		"PIN C",
		"PIN D",
	};

	if (pin >= 0 && pin <= 4)
		return str[pin];
	else
		return "Invalid PIN, not 0 - 4";
}

/**
 * Get the PCI INT_PIN swizzle for a device defined as:
 *   pin_parent = (pin_child + devn_child) % 4 + 1
 *   where PIN A = 1 ... PIN_D = 4
 *
 * Given a PCI device structure 'dev', find the interrupt pin
 * that will be triggered on its parent bridge device when
 * generating an interrupt.  For example: Device 1:3.2 may
 * use INT_PIN A but will trigger PIN D on its parent bridge
 * device.  In this case, this function will return 4 (PIN D).
 *
 * @param dev A PCI device structure to swizzle interrupt pins for
 * @param *parent_bridge The PCI device structure for the bridge
 *        device 'dev' is attached to
 * @return The interrupt pin number (1 - 4) that 'dev' will
 *         trigger when generating an interrupt
 */
static int swizzle_irq_pins(struct udevice *dev,
			    struct udevice **parent_bridgep)
{
	struct udevice *parent;		/* Our current device's parent device */
	struct udevice *child;		/* The child device of the parent */
	uint8_t swizzled_pin = 0;	/* Pin swizzled across a bridge */

	/* Start with PIN A = 0 ... D = 3 */
	dm_pci_read_config8(dev, PCI_INTERRUPT_PIN, &swizzled_pin);
	swizzled_pin -= 1;

	/* While our current device has parent devices */
	child = dev;
	for (parent = child->parent; parent; parent = parent->parent) {
		struct pci_child_platdata *pplat, *cplat;

		cplat = dev_get_parent_platdata(child);
		pplat = dev_get_parent_platdata(parent);

		/* Swizzle the INT_PIN for any bridges not on root bus */
		swizzled_pin = (PCI_DEV(cplat->devfn) + swizzled_pin) % 4;
		debug("\tWith INT_PIN swizzled to %s\n"
			"\tAttached to bridge device %01X:%02Xh.%02Xh\n",
			pin_to_str(swizzled_pin + 1), parent->seq,
			PCI_DEV(pplat->devfn), PCI_FUNC(pplat->devfn));

		/* Continue until we find the root bus */
		if (device_get_uclass_id(parent->parent->parent) == UCLASS_PCI) {
			/*
			 * We will go on to the next parent so this parent
			 * becomes the child
			 */
			child = parent;
			continue;
		} else {
			/*
			 *  Found the root bridge device,
			 *  fill in the structure and exit
			 */
			*parent_bridgep = parent;
			break;
		}
	}

	/* End with PIN A = 1 ... D = 4 */
	return swizzled_pin + 1;
}

/**
 * Given a device structure 'dev', find its interrupt pin
 * and its parent bridge 'parent_bdg' device structure.
 * If it is behind a bridge, it will return the interrupt
 * pin number (1 - 4) of the parent bridge that the device
 * interrupt pin has been swizzled to, otherwise it will
 * return the interrupt pin that is programmed into the
 * PCI config space of the target device.  If 'dev' is
 * behind a bridge, it will fill in 'parent_bdg' with the
 * device structure of the bridge it is behind, otherwise
 * it will copy 'dev' into 'parent_bdg'.
 *
 * @param dev A PCI device structure to get interrupt pins for.
 * @param *parent_bdg The PCI device structure for the bridge
 *        device 'dev' is attached to.
 * @return The interrupt pin number (1 - 4) that 'dev' will
 *         trigger when generating an interrupt.
 *         Errors: -1 is returned if the device is not enabled
 *                 -2 is returned if a parent bridge could not be found.
 */
int get_pci_irq_pins(struct udevice *dev, struct udevice **parent_bdgp)
{
	uint8_t bus = 0;	/* The bus this device is on */
	uint16_t devfn = 0;	/* This device's device and function numbers */
	uint8_t int_pin = 0;	/* Interrupt pin used by the device */
	uint8_t target_pin = 0;	/* Interrupt pin we want to assign an IRQ to */

	bus = dev->parent->seq;
	devfn = pci_get_bdf(dev);

	/* Get and validate the interrupt pin used. Only 1-4 are allowed */
	dm_pci_read_config8(dev, PCI_INTERRUPT_PIN, &int_pin);
	if (int_pin < 1 || int_pin > 4)
		return -EINVAL;

	debug("PCI IRQ: Found device %01X:%02X.%02X using %s\n",
		bus, PCI_DEV(devfn), PCI_FUNC(devfn), pin_to_str(int_pin));

	/* If this device is on a bridge, swizzle its INT_PIN */
	if (bus) {
		/* Swizzle its INT_PINs */
		target_pin = swizzle_irq_pins(dev, parent_bdgp);

		/* Make sure the swizzle returned valid structures */
		if (*parent_bdgp == NULL) {
			debug("Warning: Could not find parent bridge for this device!\n");
			return -ENOENT;
		}
	} else {	/* Device is not behind a bridge */
		target_pin = int_pin;	/* Return its own interrupt pin */
		*parent_bdgp = dev;		/* Return its own structure */
	}

	/* Target pin is the interrupt pin we want to assign an IRQ to */
	return target_pin;
}

/*
 * Write PCI config space IRQ assignments.  PCI devices have the INT_LINE
 * (0x3C) and INT_PIN (0x3D) registers which report interrupt routing
 * information to operating systems and drivers.  The INT_PIN register is
 * generally read only and reports which interrupt pin A - D it uses.  The
 * INT_LINE register is configurable and reports which IRQ (generally the
 * PIC IRQs 1 - 15) it will use.  This needs to take interrupt pin swizzling
 * on devices that are downstream on a PCI bridge into account.
 *
 * This function will loop through all enabled PCI devices and program the
 * INT_LINE register with the correct PIC IRQ number for the INT_PIN that it
 * uses.  It then configures each interrupt in the pic to be level triggered.
 */
static void write_pci_config_irqs(const struct baytrail_irq_route *ir)
{
	struct udevice *dev;
	struct udevice *targ_dev;
	uint8_t int_line = 0;
	uint8_t original_int_pin = 0;
	uint8_t new_int_pin = 0;
	uint16_t current_bdf = 0;
	uint16_t parent_bdf = 0;
	uint8_t pirq = 0;
	uint8_t device_num = 0;

	/*
	 * Loop through all enabled devices and program their
	 * INT_LINE, INT_PIN registers from values taken from
	 * the Interrupt Route registers in the ILB
	 */
	debug("PCI_CFG IRQ: Write PCI config space IRQ assignments\n");
	for (pci_find_first_device(&dev); dev; pci_find_next_device(&dev)) {

		current_bdf = pci_get_bdf(dev);

		/*
		 * Step 1: Get the INT_PIN and device structure to look for
		 * in the pirq_data table defined in the mainboard directory.
		 */
		targ_dev = NULL;
		new_int_pin = get_pci_irq_pins(dev, &targ_dev);
		if (targ_dev == NULL || new_int_pin < 1)
			continue;

		/* Get the original INT_PIN for record keeping */
		dm_pci_read_config8(dev, PCI_INTERRUPT_PIN, &original_int_pin);

		parent_bdf = pci_get_bdf(targ_dev);
		device_num = PCI_DEV(parent_bdf);

		if (ir->pcidev[device_num] == 0) {
			debug("Warning: PCI Device %d does not have an IRQ entry, skipping it\n",
			      device_num);
			continue;
		}

		/* Find the PIRQ that is attached to the INT_PIN this device uses */
		pirq = (ir->pcidev[device_num] >> ((new_int_pin - 1) * 4)) & 0xF;

		/* Get the INT_LINE this device/function will use */
		int_line = ir->pic[pirq];

		if (int_line != PIRQ_PIC_IRQDISABLE) {
			/* Set this IRQ to level triggered since it is used by a PCI device */
			configure_irq_trigger(int_line, true);
			/* Set the Interrupt Line register in PCI config space */
			dm_pci_write_config8(dev, PCI_INTERRUPT_LINE, int_line);
		} else {
			/* Set the Interrupt line register as "unknown or unused" */
			dm_pci_write_config8(dev, PCI_INTERRUPT_LINE,
					     PIRQ_PIC_UNKNOWN_UNUSED);
		}

		debug("\tINT_PIN\t\t: %d (%s)\n",
			original_int_pin, pin_to_str(original_int_pin));
		if (parent_bdf != current_bdf)
			debug("\tSwizzled to\t: %d (%s)\n",
							new_int_pin, pin_to_str(new_int_pin));
		debug("\tPIRQ\t\t: %c\n"
						"\tINT_LINE\t: 0x%X (IRQ %d)\n",
						'A' + pirq, int_line, int_line);
	}
	debug("PCI_CFG IRQ: Finished writing PCI config space IRQ assignments\n");
}

static void sc_pirq_init(const struct baytrail_irq_route *ir, ulong ibase)
{
	u8 *pr_base = (u8 *)(ibase + 0x08);
	u32 *actl = (u32 *)(ibase + ACTL);
	u16 *ir_base = (u16 *)(ibase + 0x20);
	int i;

	/* Set up the PIRQ PIC routing based on static config. */
	debug("Start writing IRQ assignments\n"
			"PIRQ\tA \tB \tC \tD \tE \tF \tG \tH\n"
			"IRQ ");
	for (i = 0; i < NUM_PIRQS; i++) {
		writeb(ir->pic[i], pr_base + i);
		debug("\t%d", ir->pic[i]);
	}
	debug("\n\n");

	/* Set up the per device PIRQ routing based on static config. */
	debug("\t\t\tPIRQ[A-H] routed to each INT_PIN[A-D]\n"
		"Dev\tINTA (IRQ)\tINTB (IRQ)\tINTC (IRQ)\tINTD (IRQ)\n");

	for (i = 0; i < NUM_OF_PCI_DEVS; i++) {
		writew(ir->pcidev[i], ir_base + i);

		/* If the entry is more than just 0, print it out */
		if (ir->pcidev[i]) {
			int j, pirq;

			debug(" %d: ", i);
			for (j = 0; j < 4; j++) {
				pirq = (ir->pcidev[i] >> (j * 4)) & 0xF;
				debug("\t%-4c (%d)", 'A' + pirq, ir->pic[pirq]);
			}
			debug("\n");
		}
	}

	/* Route SCI to IRQ9 */
	clrsetbits_le32(actl, SCIS_MASK, SCIS_IRQ9);
	debug("Finished writing IRQ assignments\n");

	/* Write IRQ assignments to PCI config space */
	write_pci_config_irqs(ir);
}

static const uint8_t lpc_pads[12] = {
	70, 68, 67, 66, 69, 71, 65, 72, 86, 90, 88, 92,
};

#define IO_BASE_ADDRESS			0xfed0c000

#define  SET_BAR_ENABLE	0x02

static void set_up_lpc_pads(struct udevice *dev)
{
	int i;

	for (i = 0; i < 12; i++)
		score_select_func(lpc_pads[i], 1);
}

static int sc_init(struct udevice *dev)
{
	int ret;
	u32 addr;
	u8 *ibase;

	debug("soc: southcluster_init\n");

	dm_pci_read_config32(dev, IBASE, &addr);
	ibase = (u8 *)(addr & ~0xF);
	debug("ibase=%p\n", ibase);

	set_up_lpc_pads(dev);

	writeb(0, ibase + ILB_MC);

	/* Set the value for PCI command register */
	dm_pci_write_config16(dev, PCI_COMMAND, PCI_COMMAND_IO |
		PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER | PCI_COMMAND_SPECIAL);

	/* IO APIC initialisation */
	ret = sc_enable_ioapic(dev, ibase);
	if (ret)
		return ret;

	sc_enable_serial_irqs(dev, ibase);

	/* Setup the PIRQ */
	sc_pirq_init(&irq_info, addr);

	/* Initialize ISA DMA */
	isa_dma_init();

	i8259_init();
// 	setup_i8259();

// 	setup_i8254();
        board_pci_post_scan(NULL);

	return 0;
}

int arch_misc_init(void)
{
	struct udevice *dev;
	int ret;

	ret = pci_bus_find_bdf(PCI_BDF(0, 0x1f, 0), &dev);
	if (ret) {
		debug("Cannot find Intel Legacy Block\n");
		return ret;
	}

	return sc_init(dev);
}
