/*
* SD update support using U-Boot commands
*/
#include <common.h>
#include <command.h>
#include <environment.h>

#define UBOOT_SIZE          0x40000   /* 256KB - this one we keep fixed */

#define AU_UBOOT    "autoupdate-uboot.bin"
#define AU_FW       "autoupdate-full.bin"

int autoupdate_status = -1;

static int check_env_vars(void)
{
	if (!getenv("flash_len")) {
		printf("SDU:   Error: Environment variable 'flash_len' not set\n");
		return -1;
	}
	if (!getenv("baseaddr")) {
		printf("SDU:   Error: Environment variable 'baseaddr' not set\n");
		return -1;
	}
	return 0;
}

static int do_flash_uboot(void)
{
	char cmd[128];
	int ret;
	const char *baseaddr = getenv("baseaddr");

	printf("SDU:   Checking for u-boot update flag file...\n");
	sprintf(cmd, "fatload mmc 0:1 %s autoupdate-uboot.done 1", baseaddr);
	debug("SDU:   Running command: %s\n", cmd);
	if (run_command(cmd, 0) == 0) {
		printf("SDU:   Flag file autoupdate-uboot.done exists, skipping uboot update\n");
		return 0;
	}

	printf("SDU:   Checking for u-boot update file...\n");
	sprintf(cmd, "fatload mmc 0:1 %s %s 1", baseaddr, AU_UBOOT);
	debug("SDU:   Running command: %s\n", cmd);
	ret = run_command(cmd, 0);
	if (ret != 0) {
		printf("SDU:   U-boot update file not found\n");
		return 0;
	}

	printf("SDU:   Loading u-boot update file...\n");
	sprintf(cmd, "fatload mmc 0:1 %s %s", baseaddr, AU_UBOOT);
	debug("SDU:   Running command: %s\n", cmd);
	ret = run_command(cmd, 0);
	if (ret != 0) {
		printf("SDU:   Error: Failed to load %s\n", AU_UBOOT);
		return -1;
	}

	printf("SDU:   Erasing u-boot section in flash...\n");
	sprintf(cmd, "sf erase 0x0 0x%x", UBOOT_SIZE);
	debug("SDU:   Running command: %s\n", cmd);
	ret = run_command(cmd, 0);
	if (ret != 0) {
		printf("SDU:   Error: Flash erase failed\n");
		return -1;
	}

	printf("SDU:   Writing new u-boot to flash...\n");
	sprintf(cmd, "sf write %s 0x0 ${filesize}", baseaddr);
	debug("SDU:   Running command: %s\n", cmd);
	ret = run_command(cmd, 0);
	if (ret != 0) {
		printf("SDU:   Error: Flash write failed\n");
		return -1;
	}

	printf("SDU:   Creating u-boot update flag file...\n");
	sprintf(cmd, "fatwrite mmc 0:1 %s autoupdate-uboot.done 1", baseaddr);
	debug("SDU:   Running command: %s\n", cmd);
	if (run_command(cmd, 0) != 0) {
		printf("SDU:   Warning: Failed to create u-boot .done flag file\n");
	}

	printf("SDU:   Successfully updated u-boot\n");
	return 1;
}

static int do_flash_full(void)
{
	char cmd[128];
	int ret;
	const char *baseaddr = getenv("baseaddr");
	const char *flash_len = getenv("flash_len");
	unsigned long flash_size;

	/* Convert flash_len from hex string to number */
	flash_size = simple_strtoul(flash_len, NULL, 16);

	printf("SDU:   Checking for full update flag file...\n");
	sprintf(cmd, "fatload mmc 0:1 %s autoupdate-full.done 1", baseaddr);
	debug("SDU:   Running command: %s\n", cmd);
	if (run_command(cmd, 0) == 0) {
		printf("SDU:   Flag file autoupdate-full.done exists, skipping full update\n");
		return 0;
	}

	printf("SDU:   Checking for full update file...\n");
	sprintf(cmd, "fatload mmc 0:1 %s %s 1", baseaddr, AU_FW);
	debug("SDU:   Running command: %s\n", cmd);
	ret = run_command(cmd, 0);
	if (ret != 0) {
		printf("SDU:   Full update file not found\n");
		return 0;
	}

	printf("SDU:   Loading full update file...\n");
	sprintf(cmd, "fatload mmc 0:1 %s %s", baseaddr, AU_FW);
	debug("SDU:   Running command: %s\n", cmd);
	ret = run_command(cmd, 0);
	if (ret != 0) {
		printf("SDU:   Error: Failed to load %s\n", AU_FW);
		return -1;
	}

	printf("SDU:   Erasing flash chip...\n");
	sprintf(cmd, "sf erase 0x0 0x%lx", flash_size);
	debug("SDU:   Running command: %s\n", cmd);
	ret = run_command(cmd, 0);
	if (ret != 0) {
		printf("SDU:   Error: Flash chip erase failed\n");
		return -1;
	}

	printf("SDU:   Writing full image to flash...\n");
	sprintf(cmd, "sf write %s 0x0 ${filesize}", baseaddr);
	debug("SDU:   Running command: %s\n", cmd);
	ret = run_command(cmd, 0);
	if (ret != 0) {
		printf("SDU:   Error: Flash write failed\n");
		return -1;
	}

	printf("SDU:   Creating full update flag file...\n");
	sprintf(cmd, "fatwrite mmc 0:1 %s autoupdate-full.done 1", baseaddr);
	debug("SDU:   Running command: %s\n", cmd);
	if (run_command(cmd, 0) != 0) {
		printf("SDU:   Warning: Failed to create full update flag file\n");
	}

	printf("SDU:   Successfully updated full image\n");
	autoupdate_status = 3;
	return 1;
}

static int do_flash_custom(const char *filename)
{
	char cmd[128];
	int ret;
	const char *baseaddr = getenv("baseaddr");
	const char *flash_len = getenv("flash_len");
	unsigned long flash_size;

	/* Convert flash_len from hex string to number */
	flash_size = simple_strtoul(flash_len, NULL, 16);

	printf("SDU:   Checking for custom update file %s...\n", filename);
	sprintf(cmd, "fatload mmc 0:1 %s %s 1", baseaddr, filename);
	debug("SDU:   Running command: %s\n", cmd);
	ret = run_command(cmd, 0);
	if (ret != 0) {
		printf("SDU:   Custom update file not found\n");
		return 0;
	}

	printf("SDU:   Loading custom update file...\n");
	sprintf(cmd, "fatload mmc 0:1 %s %s", baseaddr, filename);
	debug("SDU:   Running command: %s\n", cmd);
	ret = run_command(cmd, 0);
	if (ret != 0) {
		printf("SDU:   Error: Failed to load %s\n", filename);
		return -1;
	}

	printf("SDU:   Erasing entire flash...\n");
	sprintf(cmd, "sf erase 0x0 0x%lx", flash_size);
	debug("SDU:   Running command: %s\n", cmd);
	ret = run_command(cmd, 0);
	if (ret != 0) {
		printf("SDU:   Error: Flash erase failed\n");
		return -1;
	}

	printf("SDU:   Writing image to flash...\n");
	sprintf(cmd, "sf write %s 0x0 ${filesize}", baseaddr);
	debug("SDU:   Running command: %s\n", cmd);
	ret = run_command(cmd, 0);
	if (ret != 0) {
		printf("SDU:   Error: Flash write failed\n");
		return -1;
	}

	printf("SDU:   Successfully flashed custom image\n");
	return 1;
}

static int do_sdupdate(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret = -1;

	/* Check required environment variables */
	if (check_env_vars() != 0) {
		return CMD_RET_FAILURE;
	}

	/* Initialize MMC and FAT */
	if (run_command("mmc rescan", 0) != 0) {
		printf("MMC:   Error: MMC rescan failed\n");
		return CMD_RET_FAILURE;
	}

	if (argc > 1) {
		/* If argument is a number, treat as type */
		if (argv[1][0] >= '0' && argv[1][0] <= '9') {
			int update_type = simple_strtoul(argv[1], NULL, 16);
			printf("SDU:   Checking for update file...\n");

			if (update_type == 0) {
				return do_flash_uboot();  /* Only try u-boot */
			} else if (update_type == 1) {
				return do_flash_full();   /* Only try full */
			} else {
				return CMD_RET_USAGE;
			}
		} else {
			/* Treat argument as filename */
			printf("SDU:   Custom flash requested...\n");
			ret = do_flash_custom(argv[1]);
		}
	} else {
		/* No arguments - try both auto updates */
		printf("SDU:   Checking for autoupdate files...\n");
		ret = do_flash_uboot();
		if (ret >= 0) {
			int full_ret = do_flash_full();
			if (full_ret > 0) {
				ret = 1; /* Success if either update worked */
			}
		}
	}

	return (ret >= 0) ? CMD_RET_SUCCESS : CMD_RET_FAILURE;
}

U_BOOT_CMD(
	sdupdate, 2, 0, do_sdupdate,
	"auto upgrade file from SD card to flash",
	"[type] OR [file] - accepts ONE argument only, not both\n"
	"Usage:\n"
	"  sdupdate         - Try both autoupdate-uboot.bin and autoupdate-full.bin\n"
	"  sdupdate 0       - Use autoupdate-uboot.bin only\n"
	"  sdupdate 1       - Use autoupdate-full.bin only\n"
	"  sdupdate file.bin - Flash custom image file.bin"
);
