#include <common.h>
#include <command.h>
#include <serial.h>

#define SERIAL_NUM_ADDR1 0x13540200
#define SERIAL_NUM_ADDR2 0x13540204
#define SERIAL_NUM_ADDR3 0x13540208
#define SERIAL_NUM_ADDR4 0x13540238 // T10/T20/T30

static void increment_mac_address(uint8_t *mac) {
	int i; // Declare loop variable outside of the for loop
	// Increment the last byte of the MAC address
	// If it rolls over, increment the next byte, and so on
	for (i = 5; i >= 0; i--) {
		mac[i]++;
		if (mac[i] != 0) {
			break; // No rollover, so stop
		}
	}
}

static void generate_mac_from_serial(uint8_t *mac, uint32_t base1, uint32_t base2, uint32_t base3, uint32_t base4) {
	// Use base4 if it is not all zeros, otherwise use a combination of other bases
	if (base4 != 0) {
		mac[0] = 0x02; // Locally administered and unicast
		mac[1] = (base4 >> 24) & 0xFF;
		mac[2] = (base4 >> 16) & 0xFF;
		mac[3] = (base4 >> 8) & 0xFF;
		mac[4] = base4 & 0xFF;
	} else {
		mac[0] = 0x02; // Locally administered and unicast
		mac[1] = (base1 >> 24) & 0xFF;
		mac[2] = (base1 >> 16) & 0xFF;
		mac[3] = (base2 >> 24) & 0xFF;
		mac[4] = (base2 >> 16) & 0xFF;
	}
	// Use a combination of serial parts for the last byte
	mac[5] = ((base1 ^ base2 ^ base3 ^ base4) & 0xFF) | 0x01; // Ensure the last bit is 1 for unicast
}

static void generate_or_set_mac_address(const char *iface_name, const char *iface_type, bool increment) {
	uint8_t addr[6];
	uint32_t serial_part1 = readl(SERIAL_NUM_ADDR1);
	uint32_t serial_part2 = readl(SERIAL_NUM_ADDR2);
	uint32_t serial_part3 = readl(SERIAL_NUM_ADDR3);
	uint32_t serial_part4 = readl(SERIAL_NUM_ADDR4 + 4);

	if (!eth_getenv_enetaddr((char*)iface_name, addr)) {
		// Check if all serial parts are zero
		if (serial_part1 == 0 && serial_part2 == 0 && serial_part3 == 0 && serial_part4 == 0) {
			// Generate a random MAC address
			eth_random_enetaddr(addr);
			printf("Net:   Random MAC address for %s generated\n", iface_type);
		} else {
			// Generate MAC from serial
			generate_mac_from_serial(addr, serial_part1, serial_part2, serial_part3, serial_part4);
			if (increment) {
				increment_mac_address(addr); // For WLAN, make MAC +1 compared to ETH
			}
			printf("Net:   MAC address for %s based on device serial set\n", iface_type);
		}

		if (eth_setenv_enetaddr((char*)iface_name, addr)) {
			printf("Net:   Failed to set address for %s\n", iface_type);
		} else {
			saveenv();
		}
	} else {
		// A valid MAC address is already set
		printf("Net:   HW address for %s: %02X:%02X:%02X:%02X:%02X:%02X\n",
			iface_type,
			addr[0], addr[1], addr[2],
			addr[3], addr[4], addr[5]);
	}
}

/* Generate/set MAC Address */
void ethaddr_init(void) {
#ifdef CONFIG_RANDOM_MACADDR
	generate_or_set_mac_address("ethaddr", "ETH", false);
	generate_or_set_mac_address("wlanmac", "WLAN", true);
#endif
}

static int do_ethaddr(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
    if (argc != 2 || strcmp(argv[1], "init") != 0) {
        puts("Usage: ethaddr init\n");
        return CMD_RET_USAGE;
    }

    ethaddr_init();

    return CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	ethaddr, 2, 1, do_ethaddr,
	"Read SoC S/N and set to ENV",
	"init - Read SoC S/N and set to ENV"
);
