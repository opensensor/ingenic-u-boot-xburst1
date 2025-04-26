// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2010
 * Texas Instruments, <www.ti.com>
 *
 * Aneesh V <aneesh@ti.com>
 */
#include <common.h>
#include <spl.h>
#include <asm/u-boot.h>
#include <nand.h>
#include <fat.h>
#include <version.h>
#include <i2c.h>
#include <image.h>
#include <malloc.h>
#include <linux/compiler.h>
#include <regulator.h>

DECLARE_GLOBAL_DATA_PTR;

#ifndef CONFIG_SYS_UBOOT_START
#define CONFIG_SYS_UBOOT_START	CONFIG_SYS_TEXT_BASE
#endif
#ifndef CONFIG_SYS_MONITOR_LEN
#define CONFIG_SYS_MONITOR_LEN	(200 * 1024)
#endif

u32 *boot_params_ptr = NULL;
struct spl_image_info spl_image;

/* Define board data structure */
static bd_t bdata __attribute__ ((section(".data")));

/*
 * Default function to determine if u-boot or the OS should
 * be started. This implementation always returns 1.
 *
 * Please implement your own board specific funcion to do this.
 *
 * RETURN
 * 0 to not start u-boot
 * positive if u-boot should start
 */
#ifdef CONFIG_SPL_OS_BOOT
__weak int spl_start_uboot(void)
{
	puts("SPL: Please implement spl_start_uboot() for your board\n");
	puts("SPL: Direct Linux boot not active!\n");
	return 1;
}
#endif

/*
 * Weak default function for board specific cleanup/preparation before
 * Linux boot. Some boards/platforms might not need it, so just provide
 * an empty stub here.
 */
__weak void spl_board_prepare_for_linux(void)
{
	/* Nothing to do! */
}

void spl_parse_image_header(const struct image_header *header)
{
	u32 header_size = sizeof(struct image_header);

	if (image_get_magic(header) == IH_MAGIC) {
		if (spl_image.flags & SPL_COPY_PAYLOAD_ONLY) {
			/*
			 * On some system (e.g. powerpc), the load-address and
			 * entry-point is located at address 0. We can't load
			 * to 0-0x40. So skip header in this case.
			 */
			spl_image.load_addr = image_get_load(header);
			spl_image.entry_point = image_get_ep(header);
			spl_image.size = image_get_data_size(header);
		} else {
			spl_image.entry_point = image_get_load(header);
			/* Load including the header */
			spl_image.load_addr = spl_image.entry_point -
				header_size;
			spl_image.size = image_get_data_size(header) +
				header_size;
		}
		spl_image.os = image_get_os(header);
		spl_image.name = image_get_name(header);
		debug("spl: payload image: %s load addr: 0x%x size: %d\n",
			spl_image.name, spl_image.load_addr, spl_image.size);
	} else {
		/* Signature not found - assume u-boot.bin */
		debug("mkimage signature not found - ih_magic = %x\n",
			header->ih_magic);
		/* Let's assume U-Boot will not be more than 200 KB */
		spl_image.size = CONFIG_SYS_MONITOR_LEN;
		spl_image.entry_point = CONFIG_SYS_UBOOT_START;
		spl_image.load_addr = CONFIG_SYS_TEXT_BASE;
		spl_image.os = IH_OS_U_BOOT;
		spl_image.name = "U-Boot";
	}
}

__weak void __noreturn jump_to_image_no_args(struct spl_image_info *spl_image)
{
	typedef void __noreturn (*image_entry_noargs_t)(void);

	image_entry_noargs_t image_entry =
			(image_entry_noargs_t) spl_image->entry_point;

	printf("image entry point: 0x%X\n", spl_image->entry_point);
	image_entry();
}

/*
 * Print the SOC ID information based on the chip ID registers
 * This is a simplified version of the detection in cmd_socinfo.c
 */
static void print_hex(unsigned int value)
{
	static const char hex_chars[] = "0123456789ABCDEF";
	char hex_str[9];
	int i;

	for (i = 0; i < 8; i++) {
		hex_str[7-i] = hex_chars[value & 0xF];
		value >>= 4;
	}
	hex_str[8] = '\0';
	puts(hex_str);
}

static void print_soc_id(void)
{
	unsigned int soc_id, subsoctype1, subsoctype2;
	const char *soc_name = "Unknown";

	/* Read all the registers needed for SOC detection */
	soc_id = *((volatile unsigned int *)(0xb300002C));
	subsoctype1 = *((volatile unsigned int *)(0xb3540238));
	subsoctype2 = *((volatile unsigned int *)(0xb3540250));

	/* Extract the relevant bits from each register */
	unsigned int cpu_id = (soc_id >> 12) & 0xFFFF;
	unsigned int subsoctype1_shifted = (subsoctype1 >> 16) & 0xFFFF;
	unsigned int subsoctype2_shifted = (subsoctype2 >> 16) & 0xFFFF;

	/* Match against the SOC info table from cmd_socinfo.c */
	if (cpu_id == 0x0005 && subsoctype1_shifted == 0x0000) {
		soc_name = "T10L";
	} else if (cpu_id == 0x2000 && subsoctype1_shifted == 0x2222) {
		soc_name = "T20X";
	} else if (cpu_id == 0x2000 && subsoctype1_shifted == 0x3333) {
		soc_name = "T20L";
	} else if (cpu_id == 0x0021 && subsoctype1_shifted == 0x1111) {
		soc_name = "T21N";
	} else if (cpu_id == 0x0023 && subsoctype1_shifted == 0x1111) {
		soc_name = "T23N";
	} else if (cpu_id == 0x0023 && subsoctype1_shifted == 0x7777) {
		soc_name = "T23ZN";
	} else if (cpu_id == 0x0023 && subsoctype1_shifted == 0x2222) {
		soc_name = "T23X";
	} else if (cpu_id == 0x0030 && subsoctype1_shifted == 0x3333) {
		soc_name = "T30L";
	} else if (cpu_id == 0x0030 && subsoctype1_shifted == 0x2222) {
		soc_name = "T30X";
	} else if (cpu_id == 0x0031 && subsoctype1_shifted == 0x4444) {
		soc_name = "T31A";
	} else if (cpu_id == 0x0031 && subsoctype1_shifted == 0xCCCC) {
		soc_name = "T31AL";
	} else if (cpu_id == 0x0031 && subsoctype1_shifted == 0x3333) {
		soc_name = "T31L";
	} else if (cpu_id == 0x0031 && subsoctype1_shifted == 0xEEEE &&
	           subsoctype2_shifted == 0x300F) {
		soc_name = "T31LC";
	} else if (cpu_id == 0x0031 && subsoctype1_shifted == 0x1111) {
		soc_name = "T31N";
	} else if (cpu_id == 0x0031 && subsoctype1_shifted == 0x2222) {
		soc_name = "T31X";
	} else if (cpu_id == 0x0031 && subsoctype1_shifted == 0xDDDD) {
		soc_name = "T31ZC";
	} else if (cpu_id == 0x0031 && subsoctype1_shifted == 0x5555) {
		soc_name = "T31ZL";
	} else if (cpu_id == 0x0031 && subsoctype1_shifted == 0x6666) {
		soc_name = "T31ZX";
	}

	if (strcmp(soc_name, "Unknown") == 0) {
		/* Print debug information if SOC is unknown */
		puts("SoC: Unknown\n");
		puts("SoC Debug Info: cpu_id=0x");
		print_hex(cpu_id);
		puts(" subsoctype1=0x");
		print_hex(subsoctype1_shifted);
		puts(" subsoctype2=0x");
		print_hex(subsoctype2_shifted);
		puts("\n");
	} else {
		puts("SoC: ");
		puts(soc_name);
		puts("\n");
	}
}

#ifdef CONFIG_SPL_RAM_DEVICE
static void spl_ram_load_image(void)
{
	const struct image_header *header;

	/*
	 * Get the header.  It will point to an address defined by handoff
	 * which will tell where the image located inside the flash. For
	 * now, it will temporary fixed to address pointed by U-Boot.
	 */
	header = (struct image_header *)
		(CONFIG_SYS_TEXT_BASE -	sizeof(struct image_header));

	spl_parse_image_header(header);
}
#endif

void board_init_r(gd_t *dummy1, ulong dummy2)
{
	u32 boot_device;
	debug(">>spl:board_init_r()\n");

#ifdef CONFIG_SYS_SPL_MALLOC_START
	mem_malloc_init(CONFIG_SYS_SPL_MALLOC_START,
			CONFIG_SYS_SPL_MALLOC_SIZE);
#endif

#ifndef CONFIG_PPC
	/*
	 * timer_init() does not exist on PPC systems. The timer is initialized
	 * and enabled (decrementer) in interrupt_init() here.
	 */
	timer_init();
#endif

#ifdef CONFIG_SPL_BOARD_INIT
	spl_board_init();
#endif

	boot_device = spl_boot_device();
	debug("boot device - %d\n", boot_device);
#ifdef CONFIG_PALLADIUM
	spl_board_prepare_for_linux();
#endif
	switch (boot_device) {
#ifdef CONFIG_SPL_RAM_DEVICE
	case BOOT_DEVICE_RAM:
		spl_ram_load_image();
		break;
#endif
#ifdef CONFIG_SPL_MMC_SUPPORT
	case BOOT_DEVICE_MMC1:
	case BOOT_DEVICE_MMC2:
	case BOOT_DEVICE_MMC2_2:
		spl_mmc_load_image();
		break;
#endif
#ifdef CONFIG_SPL_SFC_SUPPORT
#ifdef CONFIG_SFC_NOR
	case BOOT_DEVICE_SFC_NOR:
		spl_sfc_nor_load_image();
		break;
#endif
#ifdef CONFIG_SFC_NAND
	case BOOT_DEVICE_SFC_NAND:
		spl_sfc_nand_load_image();
		break;
#endif
#endif
#if defined(CONFIG_SPL_NAND_SUPPORT) || defined(CONFIG_JZ_NAND_MGR)
	case BOOT_DEVICE_NAND:
		spl_nand_load_image();
		break;
#endif
#ifdef CONFIG_SPL_ONENAND_SUPPORT
	case BOOT_DEVICE_ONENAND:
		spl_onenand_load_image();
		break;
#endif
#ifdef CONFIG_SPL_NOR_SUPPORT
	case BOOT_DEVICE_NOR:
		spl_nor_load_image();
		break;
#endif
#ifdef CONFIG_SPL_YMODEM_SUPPORT
	case BOOT_DEVICE_UART:
		spl_ymodem_load_image();
		break;
#endif
#ifdef CONFIG_SPL_SPI_SUPPORT
	case BOOT_DEVICE_SPI:
		spl_spi_load_image();
		break;
#endif
#ifdef CONFIG_SPL_ETH_SUPPORT
	case BOOT_DEVICE_CPGMAC:
#ifdef CONFIG_SPL_ETH_DEVICE
		spl_net_load_image(CONFIG_SPL_ETH_DEVICE);
#else
		spl_net_load_image(NULL);
#endif
		break;
#endif
#ifdef CONFIG_SPL_USBETH_SUPPORT
	case BOOT_DEVICE_USBETH:
		spl_net_load_image("usb_ether");
		break;
#endif
	default:
		debug("SPL: Un-supported Boot Device\n");
		hang();
	}

	switch (spl_image.os) {
	case IH_OS_U_BOOT:
		debug("Jumping to U-Boot\n");
		break;
#ifdef CONFIG_SPL_OS_BOOT
	case IH_OS_LINUX:
		debug("Jumping to Linux\n");
		spl_board_prepare_for_linux();
		jump_to_image_linux((void *)CONFIG_SYS_SPL_ARGS_ADDR);
#endif
	default:
		debug("Unsupported OS image.. Jumping nevertheless..\n");
	}
	jump_to_image_no_args(&spl_image);
}

/*
 * This requires UART clocks to be enabled. In order for this to work, the
 * caller must ensure that the gd pointer is valid.
 */
void preloader_console_init(void)
{
	gd->bd = &bdata;
#ifndef CONFIG_BURNER
	gd->baudrate = CONFIG_BAUDRATE;
#else
	gd->baudrate = gd->arch.gi->baud_rate;
#endif
#ifdef CONFIG_PALLADIUM
	gd->baudrate = 3750000;
#endif
	serial_init();		/* serial communications setup */

	gd->have_console = 1;

#if !defined(CONFIG_FAST_BOOT) && !defined (CONFIG_SIMULATION) && !defined(CONFIG_YMODEM_NO_PRINTF)
	puts("\n\nThingino U-Boot for Ingenic " SOC_VAR " SPL " PLAIN_VERSION " (" U_BOOT_DATE " - " \
			U_BOOT_TIME ")\n");
	print_soc_id();
#endif
#ifdef CONFIG_SPL_DISPLAY_PRINT
	spl_display_print();
#endif
}

void spl_regulator_set(void)
{
#ifdef CONFIG_SPL_CORE_VOLTAGE
	spl_regulator_init();
	debug("Set core voltage:%dmv\n", CONFIG_SPL_CORE_VOLTAGE);
	spl_regulator_set_voltage(REGULATOR_CORE, CONFIG_SPL_CORE_VOLTAGE);
#endif
#ifdef CONFIG_SPL_MEM_VOLTAGE
	debug("Set mem voltage:%dmv\n", CONFIG_SPL_MEM_VOLTAGE);
	spl_regulator_set_voltage(REGULATOR_MEM, CONFIG_SPL_MEM_VOLTAGE);
#endif
}
