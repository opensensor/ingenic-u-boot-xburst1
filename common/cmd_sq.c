#include <common.h>
#include <div64.h>
#include <malloc.h>
#include <spi_flash.h>

extern struct spi_flash *get_flash(void);

#define ALIGNMENT_BLOCK_SIZE        0x10000  // 64KB
#define KERNEL_MAGIC_NUMBER         0x56190527
#define KERNEL_MAGIC_OFFSET         0
#define SQUASHFS_BYTES_USED_OFFSET  40
#define SQUASHFS_MAGIC              0x73717368
#define SQUASHFS_MAGIC_OFFSET       0

// Function to align using the erase block size
static uint64_t align_to_erase_block(uint64_t size, uint64_t erase_block_size) {
	if (size % erase_block_size == 0) {
		return size; // Already aligned
	}
	return ((size / erase_block_size) + 1) * erase_block_size;
}

// Function to find the start of the kernel
static uint64_t find_kernel_start(struct spi_flash *flash, unsigned int start_addr, unsigned int end_addr) {
	uint32_t magic_number;
	char buf[256];  // Buffer size as needed based on expected header size
	unsigned int addr;

	for (addr = start_addr; addr < end_addr; addr += ALIGNMENT_BLOCK_SIZE) {
		if (spi_flash_read(flash, addr, sizeof(buf), buf)) {
			printf("SQ:    Failed to read from SPI flash at address 0x%X\n", addr);
			continue;  // Skip to the next block
		}

		memcpy(&magic_number, buf + KERNEL_MAGIC_OFFSET, sizeof(magic_number));
		if (magic_number == KERNEL_MAGIC_NUMBER) {
			printf("SQ:    Kernel start detected at address 0x%X\n", addr);
			return addr;  // Return the address where the kernel is found
		}
	}

	return 0; // Kernel not found
}

// Function to find the start of the SquashFS
static uint64_t find_squashfs_start(struct spi_flash *flash, uint64_t start_search_addr) {
	uint32_t magic_number;
	char buf[4];  // Buffer to read potential magic numbers
	uint64_t addr;

	for (addr = start_search_addr; addr < flash->size; addr += ALIGNMENT_BLOCK_SIZE) {
		if (spi_flash_read(flash, addr, sizeof(buf), buf) != 0) {
			printf("SQ:    Failed to read from SPI flash at address 0x%llX\n", addr);
			continue;
		}

		memcpy(&magic_number, buf, sizeof(magic_number));
		if (magic_number == SQUASHFS_MAGIC) {
			printf("SQ:    SquashFS start detected at address 0x%llX\n", addr);
			return addr;
		}
	}

	return 0; // SquashFS not found
}

// Function to update kernel environment variables
static int update_kernel_env(struct spi_flash *flash, uint64_t *kernel_start_addr, uint64_t *squashfs_start_addr) {
	uint64_t k_start = find_kernel_start(flash, 0x40000, 0xB0000);
	if (k_start == 0) {
		printf("SQ:    Kernel not found in specified range.\n");
		return -1; // Indicate failure
	}

	*kernel_start_addr = k_start;  // Set the kernel start address

	uint64_t s_start = find_squashfs_start(flash, k_start + ALIGNMENT_BLOCK_SIZE);
	*squashfs_start_addr = s_start;  // Set the SquashFS start address
	uint64_t kernel_size = s_start - k_start;
	// No alignment for the environment variables, use raw size
	char k_start_str[32];
	sprintf(k_start_str, "%llx", k_start);
	setenv("kern_addr", k_start_str);
	printf("SQ:    kern_addr env set to: %s\n", k_start_str);

	char kern_size_str[32];
	sprintf(kern_size_str, "%lluk", kernel_size / 1024); // Report raw size in kilobytes
	setenv("kern_size", kern_size_str);
	printf("SQ:    kern_size env set to: %llu kB\n", kernel_size / 1024);

	char kern_length_str[32];
	sprintf(kern_length_str, "%llx", kernel_size); // Report raw size in hex
	setenv("kern_len", kern_length_str);
	printf("SQ:    kernel_len env set to: %s\n", kern_length_str);
	return 0;  // Success
}

static uint64_t update_squashfs_env(struct spi_flash *flash, uint64_t squashfs_start_addr) {
	char buf[64];
	if (spi_flash_read(flash, squashfs_start_addr, sizeof(buf), buf)) {
		printf("SQ:    Failed to read from SPI flash at 0x%llX\n", squashfs_start_addr);
		return 0; // Handle error appropriately and return 0
	}

	// Extract bytes used from the SquashFS header
	uint32_t bytes_used_low, bytes_used_high;
	memcpy(&bytes_used_low, buf + SQUASHFS_BYTES_USED_OFFSET, sizeof(uint32_t));
	memcpy(&bytes_used_high, buf + SQUASHFS_BYTES_USED_OFFSET + sizeof(uint32_t), sizeof(uint32_t));
	uint64_t bytes_used = ((uint64_t)bytes_used_high << 32) | bytes_used_low;
	// Align the size using the erase block size
	uint64_t aligned_bytes_used = align_to_erase_block(bytes_used, ALIGNMENT_BLOCK_SIZE);

	// Set the rootfs size environment variable using the aligned size
	char size_str[32];
	sprintf(size_str, "%lluk", aligned_bytes_used / 1024);
	setenv("rootfs_size", size_str);
	printf("SQ:    rootfs size env set to: %llu kB\n", aligned_bytes_used / 1024);

	// Return the aligned size for further calculations
	return aligned_bytes_used;
}

// Function to update MTD partition information
static void update_mtdparts_env(struct spi_flash *flash, uint64_t kernel_start_addr) {
	uint64_t flashsize = flash->size;
	uint64_t upgrade_offset = kernel_start_addr;
	char flashlen_str[32], flashsize_str[32], upgrade_offset_str[32], upgrade_size_str[32];

	sprintf(flashlen_str, "%llx", flashsize);
	setenv("flash_len", flashlen_str);
	sprintf(flashsize_str, "%lluk", flashsize / 1024); // Convert to kilobytes
	setenv("flash_size", flashsize_str);
	printf("SQ:    flash_size env to to %s\n", flashsize_str);

	uint64_t upgrade_size = flashsize - upgrade_offset;
	// Convert the flash size to kilobytes and format
	sprintf(upgrade_offset_str, "0x%llX", upgrade_offset);
	sprintf(upgrade_size_str, "%lluk", upgrade_size / 1024);

	const char *enable_update_str = getenv("enable_updates");
	if (enable_update_str != NULL && strcmp(enable_update_str, "true") == 0) {
		char update_str[256];
		if (enable_update_str != NULL && strcmp(enable_update_str, "true") == 0) {
			sprintf(update_str, ",%s@%s(upgrade),%s@0(all)", upgrade_size_str, upgrade_offset_str, flashsize_str);
		} else {
			strcpy(update_str, "");
		}
		setenv("update", update_str);
		printf("SQ:    Update ENV updated with: %s\n", update_str);
	}
}

// Function to update overlay partition
static void update_overlay_env(uint64_t overlay_addr) {
	char overlay_str[32];
	sprintf(overlay_str, "%llX", overlay_addr);
	setenv("overlay", overlay_str);
	printf("SQ:    Overlay start set to address 0x%s\n", overlay_str);
}

// Function to process SPI flash data and update environment variables
int process_spi_flash_data(struct spi_flash *flash) {
	printf("SQ:    Starting process_spi_flash_data\n");
	printf("SQ:    Alignent block size: 0x%X bytes\n", ALIGNMENT_BLOCK_SIZE);
	printf("SQ:    Erase sector size: 0x%X bytes\n", flash->sector_size);

	uint64_t kernel_start_addr = 0, squashfs_start_addr = 0;
	if (update_kernel_env(flash, &kernel_start_addr, &squashfs_start_addr) != 0) {
		printf("SQ:    Failed to find kernel or SquashFS start.\n");
		return 1;  // Indicate error
	}

	// Call update_squashfs_env and get the aligned size returned
	uint64_t aligned_bytes_used = update_squashfs_env(flash, squashfs_start_addr);
	if (aligned_bytes_used == 0) {
		printf("SQ:    Failed to calculate aligned SquashFS size.\n");
		return 1;  // Handle error if SquashFS size calculation fails
	}

	update_mtdparts_env(flash, kernel_start_addr);  // Use the actual kernel start address

	// Calculate the overlay start address without over-aligning
	uint64_t overlay_start_addr = squashfs_start_addr + aligned_bytes_used;

	// Correct alignment: ensure you are not adding extra alignment
	overlay_start_addr = overlay_start_addr & ~(ALIGNMENT_BLOCK_SIZE - 1); // Align overlay start
	update_overlay_env(overlay_start_addr);

	return 0;  // Success
}

/* env functions which do not use the flash api */

#define KERNEL_FLASH_OFFSET 0x50000
#define KERNEL_READ_SIZE    0x100000
#define ROOTFS_READ_SIZE    0x150000

static int search_for_magic(const char *buf, size_t size, uint32_t magic)
{
	size_t i;
	for (i = 0; i < size; i += 4) {
		uint32_t val;
		memcpy(&val, buf + i, sizeof(uint32_t));
		if (val == magic)
			return i;
	}
	return -1;
}

//find kernel start addr and save startaddr to ENV
uint32_t update_kernel_address_nor_noapi(void)
{
	int ret;
	char read_cmd[128];

	sprintf(read_cmd, "sf read ${baseaddr} 0x%X 0x%X", KERNEL_FLASH_OFFSET, KERNEL_READ_SIZE);

	const char *baseaddr_str;
	unsigned long base_addr;
	char *buf;
	int offset_found;
	uint32_t kernel_flash_addr;

	/* Execute the flash read command to load the block into RAM */
	ret = run_command(read_cmd, 0);
	if (ret != 0) {
		printf("SQ:    Error: 'sf read' command failed with return code %d\n", ret);
		return 0;
	}

	/* Retrieve the RAM base address from the environment variable "baseaddr" */
	baseaddr_str = getenv("baseaddr");
	if (!baseaddr_str) {
		printf("SQ:    Error: Environment variable 'baseaddr' not set.\n");
		return 0;
	}
	base_addr = simple_strtoul(baseaddr_str, NULL, 16);
	buf = (char *)base_addr;

	/* Search through the loaded block for the kernel magic value */
	offset_found = search_for_magic(buf, KERNEL_READ_SIZE, KERNEL_MAGIC_NUMBER);
	if (offset_found < 0) {
		printf("SQ:    Error: Kernel magic (0x%x) not found in flash block\n", KERNEL_MAGIC_NUMBER);
		return 0;
	}

	/* Calculate the kernel start flash address */
	kernel_flash_addr = KERNEL_FLASH_OFFSET + offset_found;
	{
		char kern_addr_str[32];
		sprintf(kern_addr_str, "0x%x", kernel_flash_addr);
		setenv("kern_addr", kern_addr_str);
		printf("SQ:    Kernel start address found: %s\n", kern_addr_str);
	}

	return kernel_flash_addr;
}

uint32_t update_rootfs_address_nor_noapi(void)
{
	int ret;
	char read_cmd[128];
	const char *baseaddr_str;
	unsigned long base_addr;
	char *buf;
	int offset_found;
	uint32_t rootfs_flash_addr;
	unsigned long kernel_flash_addr;  // retrieve this from env

	/* Retrieve the kernel start address from the environment variable "kern_addr" */
	const char *kern_addr_str = getenv("kern_addr");
	if (!kern_addr_str) {
		printf("SQ:    Error: Environment variable 'kern_addr' not set.\n");
		return 0;
	}
	kernel_flash_addr = simple_strtoul(kern_addr_str, NULL, 16);

	/* Kernel will not be smaller than 1.2mb, so start search there... */
	sprintf(read_cmd, "sf read ${baseaddr} 0x%X 0x%X", (unsigned int)(kernel_flash_addr + 0x135000), ROOTFS_READ_SIZE);

	/* Execute the flash read command to load the block into RAM */
	ret = run_command(read_cmd, 0);
	if (ret != 0) {
		printf("SQ:    Error: 'sf read' command failed with return code %d\n", ret);
		return 0;
	}

	/* Retrieve the RAM base address from the environment variable "baseaddr" */
	baseaddr_str = getenv("baseaddr");
	if (!baseaddr_str) {
		printf("SQ:    Error: Environment variable 'baseaddr' not set.\n");
		return 0;
	}
	base_addr = simple_strtoul(baseaddr_str, NULL, 16);
	buf = (char *)base_addr;

	/* Search through the loaded block for the rootfs magic value */
	offset_found = search_for_magic(buf, ROOTFS_READ_SIZE, SQUASHFS_MAGIC);
	if (offset_found < 0) {
		printf("SQ:    Error: rootfs magic (0x%x) not found in flash block\n", SQUASHFS_MAGIC);
		return 0;
	}

	/* Calculate the rootfs flash address */
	rootfs_flash_addr = kernel_flash_addr + 0x135000 + offset_found;
	{
		char rootfs_addr_str[32];
		sprintf(rootfs_addr_str, "0x%x", rootfs_flash_addr);
		setenv("rootfs_addr", rootfs_addr_str);
		printf("SQ:    rootfs start address found: %s\n", rootfs_addr_str);
	}

	return rootfs_flash_addr;
}

uint64_t compute_rootfs_partition_size_nor_noapi(void)
{
	int ret;
	const char *read_cmd = "sf read ${baseaddr} ${rootfs_addr} 64";
	uint64_t bytes_used, aligned_bytes_used;
	const char *baseaddr_str;
	unsigned long base_addr;
	char *header;
	uint32_t bytes_used_low, bytes_used_high;
	char size_str[32];

	/* Execute the command to load 64 bytes from flash into RAM */
	ret = run_command(read_cmd, 0);
	if (ret != 0) {
		printf("SQ:    Error: 'sf read' command failed with return code: %d\n", ret);
		return 0;
	}

	/* Retrieve the RAM base address from the environment variable "baseaddr" */
	baseaddr_str = getenv("baseaddr");
	if (!baseaddr_str) {
		printf("SQ:    Error: Environment variable 'baseaddr' not set.\n");
		return 0;
	}
	base_addr = simple_strtoul(baseaddr_str, NULL, 16);
	header = (char *)base_addr;

	/* Extract the 32-bit low part from the header at offset SQUASHFS_BYTES_USED_OFFSET */
	memcpy(&bytes_used_low, header + SQUASHFS_BYTES_USED_OFFSET, sizeof(uint32_t));
	/* Extract the next 32-bit high part (4 bytes after the low part) */
	memcpy(&bytes_used_high, header + SQUASHFS_BYTES_USED_OFFSET + 4, sizeof(uint32_t));

	/* Combine the two 32-bit parts into a 64-bit value */
	bytes_used = ((uint64_t)bytes_used_high << 32) | bytes_used_low;
	printf("SQ:    Bytes used from SquashFS header: 0x%llx\n", bytes_used);

	/* Align the bytes_used value upward to the nearest multiple of ALIGNMENT_BLOCK_SIZE */
	aligned_bytes_used = (bytes_used + ALIGNMENT_BLOCK_SIZE - 1) & ~(ALIGNMENT_BLOCK_SIZE - 1);
	printf("SQ:    Aligned bytes used: 0x%llx\n", aligned_bytes_used);

	/* Convert the aligned size to kilobytes and save it in the environment variable "rootfs_size" */
	sprintf(size_str, "%lluk", aligned_bytes_used / 1024);
	setenv("rootfs_size", size_str);
	printf("SQ:    rootfs_size env set to: %s\n", size_str);

	return aligned_bytes_used;
}

static void compute_kernel_partition_size(void)
{
	const char *kern_addr_str = getenv("kern_addr");
	const char *rootfs_str    = getenv("rootfs_addr");
	unsigned long kern_addr, rootfs, kern_size, kern_size_k;
	char buf[32];
	char buf_dec[32];

	if (!kern_addr_str || !rootfs_str) {
		printf("SQ:    Error: kern_addr or rootfs not set in env\n");
		return;
	}

	/* Convert the hex strings */
	kern_addr = simple_strtoul(kern_addr_str, NULL, 16);
	rootfs    = simple_strtoul(rootfs_str, NULL, 16);

	if (rootfs < kern_addr) {
		printf("SQ:    Error: rootfs (0x%lx) is less than kern_addr (0x%lx)\n", rootfs, kern_addr);
		return;
	}

	kern_size = rootfs - kern_addr;
	snprintf(buf, sizeof(buf), "0x%lx", kern_size);
	setenv("kern_size", buf);
	printf("SQ:    Computed kernel partition size (hex): %s\n", buf);

	/* Compute the size in kilobytes and format it as a decimal string with "k" */
	kern_size_k = kern_size / 1024;
	snprintf(buf_dec, sizeof(buf_dec), "%luk", kern_size_k);
	setenv("kern_size_dec", buf_dec);
	printf("SQ:    Computed kernel partition size (decimal): %s\n", buf_dec);
}

static void update_mtdparts_env_nor_noapi(void)
{
	const char *flash_len_str = getenv("flash_len");
	if (!flash_len_str) {
		printf("SQ:    Error: Environment variable 'flash_len' not set.\n");
		return;
	}
	/* Convert flash_len (assumed to be a hex string) to a number */
	uint64_t flashsize = simple_strtoull(flash_len_str, NULL, 16);

	char flashsize_str[32];
	sprintf(flashsize_str, "%lluk", flashsize / 1024);

	/* Retrieve kernel start address from environment variable "kern_addr" */
	const char *kern_addr_str = getenv("kern_addr");
	if (!kern_addr_str) {
		printf("SQ:    Error: Environment variable 'kern_addr' not set.\n");
		return;
	}
	uint64_t kernel_start_addr = simple_strtoull(kern_addr_str, NULL, 16);
	uint64_t upgrade_offset = kernel_start_addr;
	uint64_t upgrade_size = flashsize - upgrade_offset;

	char upgrade_offset_str[32], upgrade_size_str[32];
	sprintf(upgrade_offset_str, "0x%llX", upgrade_offset);
	sprintf(upgrade_size_str, "%lluk", upgrade_size / 1024);

	const char *enable_update_str = getenv("enable_updates");
	if (enable_update_str != NULL && strcmp(enable_update_str, "true") == 0) {
		char update_str[256];
		sprintf(update_str, ",%s@%s(upgrade),%s@0(all)", upgrade_size_str, upgrade_offset_str, flashsize_str);
		setenv("update", update_str);
		printf("SQ:    Update ENV updated with: %s\n", update_str);
	}
}

uint64_t update_overlay_start_addr_nor_noapi(void)
{
    const char *rootfs_addr_str = getenv("rootfs_addr");
    if (!rootfs_addr_str) {
        printf("SQ: Error: Environment variable 'rootfs_addr' not set.\n");
        return 0;
    }
    /* Convert the rootfs start address (flash address) from a string to a number */
    uint64_t rootfs_addr = simple_strtoull(rootfs_addr_str, NULL, 16);

    /* Compute the rootfs partition size by reading the SquashFS header.
     * This function should read 64 bytes from flash at 'rootfs_addr', process the header,
     * align the value, update the env variable "rootfs_size", and return the size in bytes.
     */
    uint64_t rootfs_size = compute_rootfs_partition_size_nor_noapi();
    if (rootfs_size == 0) {
        printf("SQ: Error: Unable to compute rootfs partition size.\n");
        return 0;
    }

    /* The overlay partition starts immediately after the rootfs partition */
    uint64_t overlay_start = rootfs_addr + rootfs_size;

    /* Format the overlay start address as a hex string (e.g., "0x80000") */
    char overlay_str[32];
    sprintf(overlay_str, "0x%llX", overlay_start);
    setenv("overlay", overlay_str);
    printf("SQ:    overlay start address set to: %s\n", overlay_str);

    return overlay_start;
}


static int do_sq(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	if (argc != 2 ||
	(strcmp(argv[1], "probe") != 0 && strcmp(argv[1], "probe-alt") != 0)) {
		puts("Usage: sq probe|probe-alt\n");
		return CMD_RET_USAGE;
	}

	struct spi_flash *flash = get_flash();
	if (!flash) {
		printf("SQ:    No SPI flash device available.\n");
		return CMD_RET_FAILURE;
	}

	if (strcmp(argv[1], "probe") == 0) {
		if (process_spi_flash_data(flash) == 0) {
			printf("SQ:    SquashFS processing complete.\n");
		} else {
			printf("SQ:    SquashFS processing failed.\n");
		}
	} else if (strcmp(argv[1], "probe-alt") == 0) {
		update_kernel_address_nor_noapi();
		update_rootfs_address_nor_noapi();
		compute_rootfs_partition_size_nor_noapi();
		compute_kernel_partition_size();
		update_mtdparts_env_nor_noapi();
		update_overlay_start_addr_nor_noapi();
	}

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	sq, 2, 1, do_sq,
	"Probe SquashFS data in SPI flash",
	"probe|probe-alt - Probe SquashFS data in SPI flash and update ENV"
);
