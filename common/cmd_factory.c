#include <common.h>
#include <command.h>

#define ALIGNMENT_BLOCK_SIZE        0x10000  // 64KB

void factory_reset(void) {
	/* Reset */
	char *overlay_str, *flashsize_str;
	unsigned long overlay, flashsize, length_to_erase, erase_block_size;
	char cmd[64]; // Buffer for command

	char* gpio_button_str = getenv("gpio_button");
	if (gpio_button_str) {
		unsigned gpio_number = (unsigned)simple_strtoul(gpio_button_str, NULL, 10);
		handle_gpio_settings("gpio_button");
		int value = gpio_get_value(gpio_number); // Get the GPIO value
		debug("GPIO %u value: %d\n", gpio_number, value); // Print the value

		if (value == 0) {
			printf("KEY:   Reset button pressed during boot, erasing ENV and Overlay...\n");
			run_command("env default -f -a", 0);
			/* Carry over the gpio between resets if desired */
			/* setenv("gpio_button", gpio_number); */
			saveenv();

			run_command("sf probe-alt", 0); // Initialize SPI flash

			// switch to define
			erase_block_size = ALIGNMENT_BLOCK_SIZE;

			run_command("sq probe-alt", 0); // Probe flash to set ENV variables

			overlay_str = getenv("overlay");
			flashsize_str = getenv("flash_len");
			if (overlay_str && flashsize_str) {
				overlay = simple_strtoul(overlay_str, NULL, 16);
				flashsize = simple_strtoul(flashsize_str, NULL, 16);

				// Align overlay address to the sector size
				unsigned long aligned_overlay = (overlay + erase_block_size - 1) & ~(erase_block_size - 1);

				// Calculate the initial length to erase before alignment
				length_to_erase = flashsize - aligned_overlay;

				// Align length to the next block size without exceeding flash size
				unsigned long aligned_length = (length_to_erase + erase_block_size - 1) & ~(erase_block_size - 1);
				if (aligned_overlay + aligned_length > flashsize) {
					// If alignment exceeds flash size, adjust length to not exceed flash
					aligned_length = flashsize - aligned_overlay;
					// Ensure adjusted length is also aligned to block size
					aligned_length = aligned_length & ~(erase_block_size - 1);
				}

				if (aligned_overlay + aligned_length <= flashsize) {
					debug("RST:   Aligned Overlay: 0x%lX, Flash size: 0x%lX, Length to erase: 0x%lX\n", aligned_overlay, flashsize, aligned_length);
					sprintf(cmd, "sf erase 0x%lX 0x%lX", aligned_overlay, aligned_length);
					debug("Executing command: %s\n", cmd);
					if (run_command(cmd, 0) != 0) {
						printf("RST:   Error: Failed to execute erase command.\n");
					} else {
						debug("RST:   Successfully executed: %s\n", cmd);
					}
				} else {
					printf("RST:   Error: Erase range still exceeds flash size after adjustment.\n");
				}
			} else {
				printf("RST:   Error: overlay or flash_len environment variable is not set.\n");
			}
		}
	} else {
		printf("KEY:   Reset button undefined\n");
	}
}

static int do_factory_reset(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
    if (argc != 2 || strcmp(argv[1], "reset") != 0) {
        puts("Usage: factory reset\n");
        return CMD_RET_USAGE;
    }

    factory_reset();

    return CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	factory, 2, 1, do_factory_reset,
	"Erase overlay partition and reset ENV",
	"reset - Erase overlay partition and reset ENV"
);
