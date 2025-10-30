/*
 * Ingenic isvp setup code
 *
 * Copyright (c) 2017 Ingenic Semiconductor Co.,Ltd
 * Author: Zoro <ykli@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <nand.h>
#include <net.h>
#include <netdev.h>
#include <mmc.h>
#include <asm/gpio.h>
#include <asm/arch/cpm.h>
#include <asm/arch/nand.h>
#include <asm/arch/mmc.h>
#include <asm/arch/clk.h>
#include <power/d2041_core.h>

#include <watchdog.h>

#if defined(CONFIG_JZ_MMC_MSC1)
/* ATBM6441 WiFi periodic keepalive support */
extern void atbm_init_keepalive(void);
#endif

extern int jz_net_initialize(bd_t *bis);
struct cgu_clk_src cgu_clk_src[] = {
	{AVPU, MPLL},
	{MACPHY, CONFIG_MACPHY_SEL_PLL},
	{MSC, APLL},
	{SSI, MPLL},
	{CIM, MPLL},
	{ISP, MPLL},
	{I2S_SPK, APLL}, //i2s使用APLL
	{I2S_MIC, APLL}, //i2s使用APLL
	{SRC_EOF,SRC_EOF}
};

int board_early_init_f(void)
{
	return 0;
}

#ifdef CONFIG_USB_GADGET
int jz_udc_probe(void);
void board_usb_init(void)
{
	printf("USB_udc_probe\n");
	jz_udc_probe();
}
#endif /* CONFIG_USB_GADGET */

int misc_init_r(void)
{
	int i, a, attempts, ret;

#if 0 /* TO DO */
	uint8_t mac[6] = { 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc };

	/* set MAC address */
	eth_setenv_enetaddr("ethaddr", mac);
#endif
	/* used for usb_dete */
	gpio_enable_pull_up(GPIO_PB(24));//UART1 rx

#ifdef CONFIG_JZ_MMC_MSC1
	/* Power cycle ATBM6441 WiFi chip (like Linux SDK does) */
	printf("\nATBM: Power cycling WiFi chip...\n");

	/* Step 1: Power OFF (reset the chip) */
	printf("ATBM: Setting WL_REG_EN (GPIO_PC9) LOW (power off)...\n");
	gpio_port_direction_output(GPIO_PORT_C, 9, 0);
	udelay(500000);  /* Wait 500ms (power-off hold, increased) */

	/* Step 2: Power ON */
	printf("ATBM: Setting WL_REG_EN (GPIO_PC9) HIGH (power on)...\n");
	gpio_port_direction_output(GPIO_PORT_C, 9, 1);

	/* Step 3: Toggle WL_WAKE_HOST to wake the chip's MCU */
	printf("ATBM: Toggling WL_WAKE_HOST (GPIO_PC8) LOW->HIGH before SDIO init...\n");
	gpio_port_direction_output(GPIO_PORT_C, 8, 0);
	udelay(10000);  /* 10ms low pulse */
	gpio_port_direction_output(GPIO_PORT_C, 8, 1);
	udelay(50000);  /* 50ms after high */

	/* Step 4: Wait for chip to stabilize (feed WDT while waiting) */
	printf("ATBM: Waiting 2500ms for chip to stabilize...\n");
	for (i = 0; i < 25; ++i) {
		WATCHDOG_RESET();
		udelay(100000); /* 100ms x 25 */
	}
	printf("ATBM: WiFi chip power cycle complete!\n\n");

	/* Step 5: Initialize MSC0 (SDIO) controller - target WiFi on MSC0 */
	{
		struct mmc *mmc = find_mmc_device(0);
		if (mmc) {
			printf("ATBM: Initializing MSC0 (mmc device 0) for SDIO...\n");
			ret = -1;
			attempts = 1;  /* Only try once - don't block boot */
			for (a = 1; a <= attempts; ++a) {
				WATCHDOG_RESET();
				ret = mmc_init(mmc);
				printf("ATBM: MSC0 init attempt %d/%d returned %d\n", a, attempts, ret);
				if (ret == 0)
					break;
			}
			printf("ATBM: MSC0 final init status %d\n", ret);

			if (ret == 0 && mmc->is_sdio) {
				printf("\n========================================\n");
				printf("ATBM: SDIO device enumerated successfully!\n");
				printf("ATBM: Starting periodic keepalive...\n");
				printf("========================================\n\n");

				/* Start periodic keepalive to satisfy MCU watchdog */
				atbm_init_keepalive();
			} else {
				printf("\n========================================\n");
				printf("ATBM: SDIO initialization failed (ret=%d)\n", ret);
				printf("ATBM: Continuing to U-Boot prompt anyway...\n");
				printf("========================================\n\n");
			}
		} else {
			printf("ATBM: ERROR - Could not find mmc device 0 (MSC0)!\n");
		}
	}
#endif

	return 0;
}

#ifdef CONFIG_BOARD_LATE_INIT
int board_late_init(void)
{
	printf("\n========================================\n");
	printf("BOARD_LATE_INIT: Starting...\n");
	printf("========================================\n\n");

	/* Keepalive is already started in misc_init_r after SDIO enumeration */
	printf("ATBM: Periodic keepalive already running from misc_init_r\n");

	return 0;
}
#endif



#ifdef CONFIG_MMC
int board_mmc_init(bd_t *bd)
{
	jz_mmc_init();
	return 0;
}
#endif

#ifdef CONFIG_SYS_NAND_SELF_INIT
void board_nand_init(void)
{
	return;
}
#endif

int board_eth_init(bd_t *bis)
{
	int ret = 0;
#ifdef CONFIG_USB_ETHER_ASIX
	if (0 == strncmp(getenv("ethact"), "asx", 3)) {
		run_command("usb start", 0);
	}
#endif
	ret += jz_net_initialize(bis);
	return ret;
}

#ifdef CONFIG_SPL_NOR_SUPPORT
int spl_start_uboot(void)
{
	return 1;
}
#endif

#ifdef CONFIG_SPL_BUILD

void spl_board_init(void)
{
}

#endif /* CONFIG_SPL_BUILD */
