#include <common.h>
#include <command.h>

#define ALIGNMENT_BLOCK_SIZE        0x10000  // 64KB

static int factory_reset(void)
{
	int ret_status = CMD_RET_SUCCESS;  /* Assume success if reset operations are performed */
	char *overlay_str, *flashsize_str;
	unsigned long overlay, flashsize, length_to_erase, erase_block_size;
	char cmd[64]; /* Buffer for command */

	char *gpio_button_str = getenv("gpio_button");
	if (gpio_button_str) {
		unsigned gpio_number = (unsigned)simple_strtoul(gpio_button_str, NULL, 10);
		handle_gpio_settings("gpio_button");
		int value = gpio_get_value(gpio_number); /* Get the GPIO value */
		debug("GPIO %u value: %d\n", gpio_number, value);

		if (value == 0) {
			/* Button pressed: perform reset operations */
			printf("KEY:   Reset button pressed during boot, erasing ENV and Overlay...\n");

			if (run_command("env default -f -a", 0) != 0) {
				printf("RST:   Error: Failed to reset ENV defaults.\n");
				ret_status = CMD_RET_FAILURE;
			}
			saveenv();

			if (run_command("sf probe-alt", 0) != 0) {
				printf("RST:   Error: SPI flash probe failed.\n");
				ret_status = CMD_RET_FAILURE;
			}

			/* Use the defined block size */
			erase_block_size = ALIGNMENT_BLOCK_SIZE;

			if (run_command("sq probe-alt", 0) != 0) {
				printf("RST:   Error: Flash probing (sq probe-alt) failed.\n");
				ret_status = CMD_RET_FAILURE;
			}

			overlay_str = getenv("overlay");
			flashsize_str = getenv("flash_len");
			if (overlay_str && flashsize_str) {
				overlay = simple_strtoul(overlay_str, NULL, 16);
				flashsize = simple_strtoul(flashsize_str, NULL, 16);

				/* Align overlay address to the erase block size */
				unsigned long aligned_overlay = (overlay + erase_block_size - 1) & ~(erase_block_size - 1);

				/* Calculate the length to erase */
				length_to_erase = flashsize - aligned_overlay;
				unsigned long aligned_length = (length_to_erase + erase_block_size - 1) & ~(erase_block_size - 1);
				if (aligned_overlay + aligned_length > flashsize) {
					aligned_length = flashsize - aligned_overlay;
					aligned_length = aligned_length & ~(erase_block_size - 1);
				}

				if (aligned_overlay + aligned_length <= flashsize) {
					debug("RST:   Aligned Overlay: 0x%lX, Flash size: 0x%lX, Length to erase: 0x%lX\n",
						aligned_overlay, flashsize, aligned_length);
					sprintf(cmd, "sf erase 0x%lX 0x%lX", aligned_overlay, aligned_length);
					debug("Executing command: %s\n", cmd);
					if (run_command(cmd, 0) != 0) {
						printf("RST:   Error: Failed to execute erase command.\n");
						ret_status = CMD_RET_FAILURE;
					} else {
						debug("RST:   Successfully executed: %s\n", cmd);
					}
				} else {
					printf("RST:   Error: Erase range still exceeds flash size after adjustment.\n");
					ret_status = CMD_RET_FAILURE;
				}
			} else {
				printf("RST:   Error: overlay or flash_len environment variable is not set.\n");
				ret_status = CMD_RET_FAILURE;
			}
		} else {
			/* Button not pressed: do not perform reset */
			printf("KEY:   Reset button not pressed; skipping factory reset.\n");
			ret_status = CMD_RET_FAILURE;
		}
	} else {
		printf("KEY:   Reset button undefined\n");
		ret_status = CMD_RET_FAILURE;
	}

	/* Return the status (CMD_RET_SUCCESS if reset operations occurred, CMD_RET_FAILURE otherwise) */
	return ret_status;
}

static int do_factory_reset(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	if (argc != 2 || strcmp(argv[1], "reset") != 0) {
		puts("Usage: factory reset\n");
		return CMD_RET_USAGE;
	}

	int ret = factory_reset();

	if (ret == CMD_RET_SUCCESS) {
		printf("RST:   Factory reset successful, resetting system...\n");
		run_command("reset", 0);
		return CMD_RET_SUCCESS;
	} else {
		printf("RST:   Factory reset not performed (or failed).\n");
		return CMD_RET_FAILURE;
	}
}

U_BOOT_CMD(
	factory, 2, 1, do_factory_reset,
	"Erase overlay partition and reset ENV",
	"reset - Erase overlay partition and reset ENV"
);
