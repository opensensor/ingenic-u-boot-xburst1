#include <common.h>
#include <div64.h>
#include <malloc.h>
#include <configs/isvp_common.h>

#define ALIGNMENT_BLOCK_SIZE        CONFIG_SFC_MIN_ALIGN
#define KERNEL_MAGIC_NUMBER         0x56190527
#define SQUASHFS_BYTES_USED_OFFSET  40
#define SQUASHFS_MAGIC              0x73717368
#define KERNEL_FLASH_OFFSET         0x50000  /* Start reading for the kernel at this offset */
#define KERNEL_READ_SIZE            0x100000 /* Read this much data to search for the kernel starting from the kernel offset */
#define ROOTFS_READ_SIZE            0x150000 /* Read this much data from the kernel offset to search for the rootfs */
#define ROOTFS_READ_OFFSET          0x135000 /* Start reading for the rootfs at this offset */

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

/* Find kernel start addr and save startaddr to ENV */
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
	unsigned long kernel_flash_addr;  /* retrieve saved value from env */

	/* Retrieve the kernel start address from the environment variable "kern_addr" */
	const char *kern_addr_str = getenv("kern_addr");
	if (!kern_addr_str) {
		printf("SQ:    Error: Environment variable 'kern_addr' not set.\n");
		return 0;
	}
	kernel_flash_addr = simple_strtoul(kern_addr_str, NULL, 16);

	/* Kernel will not be smaller than 1.2mb, so start search there... */
	sprintf(read_cmd, "sf read ${baseaddr} 0x%X 0x%X", (unsigned int)(kernel_flash_addr + ROOTFS_READ_OFFSET), ROOTFS_READ_SIZE);

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
	rootfs_flash_addr = kernel_flash_addr + ROOTFS_READ_OFFSET + offset_found;
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
        printf("SQ:    Error: Environment variable 'rootfs_addr' not set.\n");
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
        printf("SQ:    Error: Unable to compute rootfs partition size.\n");
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

	if (strcmp(argv[1], "probe") == 0 || strcmp(argv[1], "probe-alt") == 0) {
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
	"Probe Kernel / SquashFS data in SPI flash",
	"probe|probe-alt - Probe Kernel / SquashFS data in SPI flash and update ENV"
);
