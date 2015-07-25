/*
 * Copyright (C) 2014, Bin Meng <bmeng.cn@gmail.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */
#define DEBUG
#include <common.h>
#include <mmc.h>
#include <pci_ids.h>
#include <asm/irq.h>
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

#define PCI_DEV_PIRQ_ROUTE(dev_, a_, b_, c_, d_) \
	[dev_] = ((PIRQ ## d_) << 12) | ((PIRQ ## c_) << 8) | \
	         ((PIRQ ## b_) <<  4) | ((PIRQ ## a_) << 0)

static uint16_t pcidev[32] = {
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
	PCI_DEV_PIRQ_ROUTE(0x1f, H, G, B, C)
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

static int sc_enable_ioapic(pci_dev_t dev)
{
	int i;
	u32 reg32;
	volatile u32 *ioapic_index = (u32 *)(IO_APIC_ADDR);
	volatile u32 *ioapic_data = (u32 *)(IO_APIC_ADDR + 0x10);
	uint32_t ibase;
	u8 *ilb_base;

	pci_read_config32(dev, IBASE, &ibase);
	ilb_base = (u8 *)(ibase & ~0x0f);

	/*
	 * Enable ACPI I/O and power management.
	 * Set SCI IRQ to IRQ9
	 */
	writel(0x100, ilb_base + ILB_OIC);  /* AEN */
	reg32 = readl(ilb_base + ILB_OIC); /* Read back per BWG */
	writel(0, ilb_base + ILB_ACTL);  /* ACTL bit 2:0 SCIS IRQ9 */

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
}

static void sc_enable_serial_irqs(pci_dev_t dev)
{
#ifdef SETUPSERIQ /* NOT defined. Remove when the TODO is done. */
	/*
	 * TODO: SERIRQ seems to have a number of problems on baytrail.
	 * With it enabled, we get some spurious interrupts (ps2)
	 * in seabios. It also caused IOCHK# NMIs. Remove it
	 * until we understand how it needs to be configured.
	 */
	u8 reg8;
	u8 *ibase = (u8 *)(pci_read_config32(dev, IBASE) & ~0xF);

	/*
	 * Disable the IOCHK# NMI. Let the NMI handler enable it if it needs.
	 */
	reg8 = inb(0x61);
	reg8 &= 0x0f; /* Higher Nibble must be 0 */
	reg8 |= (1 << 3); /* IOCHK# NMI  Disable for now */
	outb(reg8, 0x61);

	write32(ibase + ILB_OIC, read32(ibase + ILB_OIC) | SIRQEN);
	write8(ibase + ILB_SERIRQ_CNTL, SCNT_CONTINUOUS_MODE);

#if !IS_ENABLED(CONFIG_SERIRQ_CONTINUOUS_MODE)
	/*
	 * SoC requires that the System BIOS first set the SERIRQ logic to
	 * continuous mode operation for at least one frame before switching
	 *  it into quiet mode operation.
	 */
	outb(0x00, 0xED); /* I/O Delay to get the 1 frame */
	write8(ibase + ILB_SERIRQ_CNTL, SCNT_QUIET_MODE);
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
	device_t parent;	/* Our current device's parent device */
	device_t child;		/* The child device of the parent */
	uint8_t parent_bus = 0;		/* Parent Bus number */
	uint16_t parent_devfn = 0;	/* Parent Device and Function number */
	uint16_t child_devfn = 0;	/* Child Device and Function number */
	uint8_t swizzled_pin = 0;	/* Pin swizzled across a bridge */

	/* Start with PIN A = 0 ... D = 3 */
	swizzled_pin = pci_read_config8(dev, PCI_INTERRUPT_PIN) - 1;

	/* While our current device has parent devices */
	child = dev;
	for (parent = child->bus->dev; parent; parent = parent->bus->dev) {
		parent_bus = parent->bus->secondary;
		parent_devfn = parent->path.pci.devfn;
		child_devfn = child->path.pci.devfn;

		/* Swizzle the INT_PIN for any bridges not on root bus */
		swizzled_pin = (PCI_SLOT(child_devfn) + swizzled_pin) % 4;
		printk(BIOS_SPEW, "\tWith INT_PIN swizzled to %s\n"
			"\tAttached to bridge device %01X:%02Xh.%02Xh\n",
			pin_to_str(swizzled_pin + 1), parent_bus,
			PCI_SLOT(parent_devfn), PCI_FUNC(parent_devfn));

		/* Continue until we find the root bus */
		if (parent_bus > 0) {
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
			*parent_bridge = parent;
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
int get_pci_irq_pins(pci_dev_t dev, device_t *parent_bdg)
{
	uint8_t bus = 0;	/* The bus this device is on */
	uint16_t devfn = 0;	/* This device's device and function numbers */
	uint8_t int_pin = 0;	/* Interrupt pin used by the device */
	uint8_t target_pin = 0;	/* Interrupt pin we want to assign an IRQ to */

	/* Make sure this device is enabled */
	if (!(dev->enabled && (dev->path.type == DEVICE_PATH_PCI)))
		return -1;

	bus = dev->bus->secondary;
	devfn = dev->path.pci.devfn;

	/* Get and validate the interrupt pin used. Only 1-4 are allowed */
	int_pin = pci_read_config8(dev, PCI_INTERRUPT_PIN);
	if (int_pin < 1 || int_pin > 4)
		return -1;

	printk(BIOS_SPEW, "PCI IRQ: Found device %01X:%02X.%02X using %s\n",
		bus, PCI_SLOT(devfn), PCI_FUNC(devfn), pin_to_str(int_pin));

	/* If this device is on a bridge, swizzle its INT_PIN */
	if (bus) {
		/* Swizzle its INT_PINs */
		target_pin = swizzle_irq_pins(dev, parent_bdg);

		/* Make sure the swizzle returned valid structures */
		if (parent_bdg == NULL) {
			printk(BIOS_WARNING,
				"Warning: Could not find parent bridge for this device!\n");
			return -2;
		}
	} else {	/* Device is not behind a bridge */
		target_pin = int_pin;	/* Return its own interrupt pin */
		*parent_bdg = dev;		/* Return its own structure */
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
static void write_pci_config_irqs(void)
{
	device_t irq_dev;
	device_t targ_dev;
	uint8_t int_line = 0;
	uint8_t original_int_pin = 0;
	uint8_t new_int_pin = 0;
	uint16_t current_bdf = 0;
	uint16_t parent_bdf = 0;
	uint8_t pirq = 0;
	uint8_t device_num = 0;
	const struct baytrail_irq_route *ir = &global_baytrail_irq_route;

	if (ir == NULL) {
		printk(BIOS_WARNING, "Warning: Can't write PCI IRQ assignments because"
				" 'global_baytrail_irq_route' structure does not exist\n");
		return;
	}

	/*
	 * Loop through all enabled devices and program their
	 * INT_LINE, INT_PIN registers from values taken from
	 * the Interrupt Route registers in the ILB
	 */
	debug("PCI_CFG IRQ: Write PCI config space IRQ assignments\n");
	for (irq_dev = all_devices; irq_dev; irq_dev = irq_dev->next) {

		if ((irq_dev->path.type != DEVICE_PATH_PCI) ||
			(!irq_dev->enabled))
			continue;

		current_bdf = irq_dev->path.pci.devfn |
			irq_dev->bus->secondary << 8;

		/*
		 * Step 1: Get the INT_PIN and device structure to look for
		 * in the pirq_data table defined in the mainboard directory.
		 */
		targ_dev = NULL;
		new_int_pin = get_pci_irq_pins(irq_dev, &targ_dev);
		if (targ_dev == NULL || new_int_pin < 1)
			continue;

		/* Get the original INT_PIN for record keeping */
		original_int_pin = pci_read_config8(irq_dev, PCI_INTERRUPT_PIN);

		parent_bdf = targ_dev->path.pci.devfn
			| targ_dev->bus->secondary << 8;
		device_num = PCI_SLOT(parent_bdf);

		if (ir->pcidev[device_num] == 0) {
			printk(BIOS_WARNING,
				"Warning: PCI Device %d does not have an IRQ entry, skipping it\n",
				device_num);
			continue;
		}

		/* Find the PIRQ that is attached to the INT_PIN this device uses */
		pirq = (ir->pcidev[device_num] >> ((new_int_pin - 1) * 4)) & 0xF;

		/* Get the INT_LINE this device/function will use */
		int_line = ir->pic[pirq];

		if (int_line != PIRQ_PIC_IRQDISABLE) {
			/* Set this IRQ to level triggered since it is used by a PCI device */
			i8259_configure_irq_trigger(int_line, IRQ_LEVEL_TRIGGERED);
			/* Set the Interrupt Line register in PCI config space */
			pci_write_config8(irq_dev, PCI_INTERRUPT_LINE, int_line);
		} else {
			/* Set the Interrupt line register as "unknown or unused" */
			pci_write_config8(irq_dev, PCI_INTERRUPT_LINE,
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

static void sc_pirq_init(pci_dev_t dev)
{
	u8 *pr_base = (u8 *)(ILB_BASE_ADDRESS + 0x08);
	u32 *actl = (u32 *)(ILB_BASE_ADDRESS + ACTL);
	u16 *ir_base = (u16 *)(ILB_BASE_ADDRESS + 0x20);
	int i;

	/* Set up the per device PIRQ routing based on static config. */
	debug("\t\t\tPIRQ[A-H] routed to each INT_PIN[A-D]\n"
		"Dev\tINTA (IRQ)\tINTB (IRQ)\tINTC (IRQ)\tINTD (IRQ)\n");
	for (i = 0; i < NUM_PIRQS; i++) {
		write8(pr_base + i, ir->pic[i]);
		debug("\t%d", ir->pic[i]);
	}
	debug("\n\n");

	for (i = 0; i < ARRAY_SIZE(pcidev); i++) {
		writew(pcidev[i], ir_base + i);

		/* If the entry is more than just 0, print it out */
		if (pcidev[i]) {
			int j, pirq;

			debug(" %d: ", i);
			for (j = 0; j < 4; j++) {
				pirq = (pcidev[i] >> (j * 4)) & 0xF;
				debug("\t%-4c (%d)", 'A' + pirq, -1/*pic[pirq]*/);
			}
			debug("\n");
		}
	}

	/* Route SCI to IRQ9 */
	clrsetbits_le32(actl, SCIS_MASK, SCIS_IRQ9);
	debug("Finished writing IRQ assignments\n");

	/* Write IRQ assignments to PCI config space */
	write_pci_config_irqs();
}

static void sc_init(pci_dev_t dev)
{
	u8 *ibase;

	debug("soc: southcluster_init\n");

	ibase = (u8 *)(pci_read_config32(dev, IBASE) & ~0xF);

	write8(ibase + ILB_MC, 0);

	/* Set the value for PCI command register. */
	pci_write_config16(dev, PCI_COMMAND,
		PCI_COMMAND_IO | PCI_COMMAND_MEMORY |
		PCI_COMMAND_MASTER | PCI_COMMAND_SPECIAL);

	/* IO APIC initialization. */
	sc_enable_ioapic(dev);

	sc_enable_serial_irqs(dev);

	/* Setup the PIRQ. */
	sc_pirq_init(dev);

	/* Initialize the High Precision Event Timers, if present. */
	enable_hpet();

	/* Initialize ISA DMA. */
	isa_dma_init();

	setup_i8259();

	setup_i8254();
}

int arch_misc_init(void)
{
	sc_init(PCI_BDF(0, 0x1f, 0));

	return 0;
}
