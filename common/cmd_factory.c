#include <common.h>
#include <command.h>

#define ALIGNMENT_BLOCK_SIZE        0x10000  // 64KB

/*
* factory_reset_internal() - Perform factory reset operations.
* @force: if nonzero, bypass the GPIO button check.
*
* Returns CMD_RET_SUCCESS if reset operations succeed,
* or CMD_RET_FAILURE otherwise.
*/
static int factory_reset_internal(int force)
{
	int ret_status = CMD_RET_SUCCESS;  /* Assume success */
	char *overlay_str, *flashsize_str;
	unsigned long overlay, flashsize, length_to_erase, erase_block_size;
	char cmd[64]; /* Buffer for command */

	if (!force) {
		char *gpio_button_str = getenv("gpio_button");
		if (gpio_button_str) {
			unsigned gpio_number = (unsigned)simple_strtoul(gpio_button_str, NULL, 10);
			handle_gpio_settings("gpio_button");
			int value = gpio_get_value(gpio_number); /* Get the GPIO value */
			debug("GPIO %u value: %d\n", gpio_number, value);
			if (value != 0) {
				printf("KEY:   Reset button not pressed; skipping factory reset.\n");
				return CMD_RET_FAILURE;
			}
		} else {
			printf("KEY:   Reset button undefined; skipping factory reset.\n");
			return CMD_RET_FAILURE;
		}
	}

	/* Proceed with reset operations (either forced or button checked) */
	printf("KEY:   Factory reset initiated%s.\n", force ? " (forced)" : "");

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

	return ret_status;
}

/*
* do_factory_reset() - Command handler for factory reset.
*
* Accepts two forms:
*   factory reset      -> Force reset (ignores button)
*   factory reset-boot -> Reset only if reset button is pressed
*/
static int do_factory_reset(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	if (argc != 2 ||
		(strcmp(argv[1], "reset") != 0 && strcmp(argv[1], "reset-boot") != 0)) {
		puts("Usage: factory reset\n");
		return CMD_RET_USAGE;
	}

	int ret;
	if (strcmp(argv[1], "reset") == 0) {
		/* "reset" mode: do not check button, always perform reset */
		ret = factory_reset_internal(1);
	} else { /* "reset-boot" mode: check for button */
		ret = factory_reset_internal(0);
	}

	if (ret == CMD_RET_SUCCESS) {
		printf("RST:   Factory reset successful, resetting system...\n");
		run_command("reset", 0);
		return CMD_RET_SUCCESS;
	} else {
		debug("RST:   Factory reset not performed (or failed).\n");
		return CMD_RET_FAILURE;
	}
}

U_BOOT_CMD(
	factory, 2, 1, do_factory_reset,
	"Erase overlay partition and reset ENV",
	"reset - Erase overlay partition and reset ENV"
);
