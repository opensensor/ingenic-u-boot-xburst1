#include <common.h>
#include <command.h>

#ifdef CONFIG_BITBANGMII
#include <miiphy.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

void jznet_init(void) {
#ifdef CONFIG_BITBANGMII
	bb_miiphy_init();
#endif

#if defined(CONFIG_CMD_NET)
	int ret = 0;
	char* disable_eth = getenv("disable_eth");

#ifdef CONFIG_USB_ETHER_ASIX
	char* ethact = getenv("ethact");
	if (ethact && strncmp(ethact, "asx", 3) == 0) {
		if (run_command("usb start", 0) != 0) {
			printf("USB:   USB start failed\n");
		}
	}
#endif
	int networkInitializationAttempted = 0;

	/* Check if disable_eth is set to "true" */
	if (disable_eth && strcmp(disable_eth, "true") == 0) {
		/* disable_eth is true, so skip network initialization */
		printf("Net:   Networking disabled (U-Boot)\n");
		/* Handle GPIO settings since network init is skipped */
		handle_gpio_settings("gpio_default_net");
	} else {
		/* Attempt network initialization */
		networkInitializationAttempted = 1;
		ret = jz_net_initialize(gd->bd);
		if (ret < 0) {
			debug("Net:   Network initialization failed.\n");
			// Network initialization failed, handle GPIO settings here
			handle_gpio_settings("gpio_default_net");
		}
	}
#endif
}

static int do_jznet(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
    if (argc != 2 || strcmp(argv[1], "init") != 0) {
        puts("Usage: jznet init\n");
        return CMD_RET_USAGE;
    }

    jznet_init();

    return CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	jznet, 2, 1, do_jznet,
	"Probe and Initialize ingenic PHY devices.",
	"init - Probe and Initialize ingenic PHY devices"
);
