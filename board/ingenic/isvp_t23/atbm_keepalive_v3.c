/*
 * ATBM6441 WiFi Keepalive for U-Boot - PROPER SDIO IMPLEMENTATION
 *
 * This version actually sends SDIO commands to the ATBM6441 chip.
 *
 * Copyright (c) 2025
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/gpio.h>
#include <mmc.h>
#include <ingenic_soft_i2c.h>

#ifdef CONFIG_JZ_MMC_MSC1

#define ATBM_WSM_CMD_KEEPALIVE 0x003c
#define ATBM_SDIO_BLOCK_SIZE   256

/* I2C GPIO pins for D2041 PMIC
 * T23 typically uses Port C pins for I2C0
 * PC26 = SCL, PC27 = SDA (common configuration)
 * GPIO_PC(n) = (2*32 + n) = 64 + n
 */
#define I2C_SCL_GPIO  GPIO_PC(26)  /* PC26 = GPIO 90 */
#define I2C_SDA_GPIO  GPIO_PC(27)  /* PC27 = GPIO 91 */

/* MSC register offsets */
#define MSC_CTRL		0x00
#define MSC_STAT		0x04
#define MSC_CLKRT		0x08
#define MSC_CMDAT		0x0C
#define MSC_RESTO		0x10
#define MSC_RDTO		0x14
#define MSC_BLKLEN		0x18
#define MSC_NOB			0x1C
#define MSC_SNOB		0x20
#define MSC_IMASK		0x24
#define MSC_IFLG		0x28
#define MSC_CMD			0x2C
#define MSC_ARG			0x30
#define MSC_RES			0x34
#define MSC_RXFIFO		0x38
#define MSC_TXFIFO		0x3C
#define MSC_LPM			0x40

/* MSC_CTRL bits */
#define MSC_CTRL_START_OP	(1 << 7)
#define MSC_CTRL_RESET		(1 << 3)
#define MSC_CTRL_CLOCK_START	(1 << 1)

/* MSC_STAT bits */
#define MSC_STAT_PRG_DONE	(1 << 13)
#define MSC_STAT_DATA_TRAN_DONE	(1 << 12)
#define MSC_STAT_DATA_FIFO_EMPTY (1 << 2)

/* MSC_IFLG bits */
#define MSC_IREG_END_CMD_RES	(1 << 2)
#define MSC_IREG_TIME_OUT_RES	(1 << 9)
#define MSC_IREG_TXFIFO_WR_REQ	(1 << 7)
#define MSC_IREG_PRG_DONE	(1 << 13)

/* MSC_CMDAT bits */
#define MSC_CMDAT_DATA_EN	(1 << 0)
#define MSC_CMDAT_WRITE		(1 << 1)
#define MSC_CMDAT_RESPONSE_R1	(0x1 << 4)
#define MSC_CMDAT_RESPONSE_R5	(0x1 << 4)  /* Same as R1 for SDIO */
#define MSC_CMDAT_BUS_WIDTH_4	(0x2 << 9)

/* SDIO CMD53 - IO_RW_EXTENDED */
#define SD_IO_RW_EXTENDED	53

/* CMD53 argument bits */
#define SDIO_CMD53_WRITE	(1 << 31)
#define SDIO_CMD53_FUNC(x)	(((x) & 0x7) << 28)
#define SDIO_CMD53_BLOCK_MODE	(1 << 27)
#define SDIO_CMD53_OP_CODE	(1 << 26)  /* 1=increment address */
#define SDIO_CMD53_ADDR(x)	(((x) & 0x1FFFF) << 9)
#define SDIO_CMD53_COUNT(x)	((x) & 0x1FF)

static inline void atbm_write_le16(unsigned char *p, unsigned short v)
{
	p[0] = v & 0xff;
	p[1] = (v >> 8) & 0xff;
}

static inline void atbm_write_le32(unsigned char *p, unsigned int v)
{
	p[0] = v & 0xff;
	p[1] = (v >> 8) & 0xff;
	p[2] = (v >> 16) & 0xff;
	p[3] = (v >> 24) & 0xff;
}

static void msc1_writel(unsigned int val, unsigned int off)
{
	writel(val, MSC1_BASE + off);
}

static unsigned int msc1_readl(unsigned int off)
{
	return readl(MSC1_BASE + off);
}

/* Forward declarations */
static int atbm_sdio_init_sequence(void);
static int atbm_sdio_quick_test(void);

/*
 * Try to disable PMIC watchdog
 *
 * The D2041 PMIC has a watchdog control bit in CONTROLD register (0x11).
 * Bit 7 (D2041_CONTROLD_WATCHDOG) controls the watchdog.
 *
 * This might be controlling the ATBM6441's watchdog!
 */
static int atbm_try_disable_pmic_watchdog(void)
{
	/* D2041 PMIC I2C address is 0x49 */
	struct i2c pmic_i2c;
	unsigned char reg_val;
	int ret;
	int i;

	printf("ATBM: Attempting to disable D2041 PMIC watchdog via I2C...\n");

	/* Setup I2C GPIO pins */
	pmic_i2c.scl = I2C_SCL_GPIO;
	pmic_i2c.sda = I2C_SDA_GPIO;

	printf("ATBM: I2C using GPIO SCL=%d, SDA=%d\n", pmic_i2c.scl, pmic_i2c.sda);

	/* Initialize I2C */
	i2c_init(&pmic_i2c);

	printf("ATBM: I2C initialized, probing PMIC at address 0x49...\n");

	/* Probe the PMIC */
	ret = i2c_probe(&pmic_i2c, 0x49);
	if (ret != 0) {
		printf("ATBM: PMIC not found at I2C address 0x49 (ret=%d)\n", ret);
		printf("ATBM: This might mean wrong GPIO pins or PMIC not powered\n");
		return -1;
	}

	printf("ATBM: PMIC found! Dumping control registers...\n");

	/* Dump all control registers to understand the PMIC state */
	printf("ATBM: PMIC Register Dump:\n");
	for (i = 0x0e; i <= 0x14; i++) {
		ret = i2c_read(&pmic_i2c, 0x49, i, 1, &reg_val, 1);
		if (ret == 0) {
			printf("ATBM:   Reg 0x%02x = 0x%02x", i, reg_val);
			if (i == 0x0e) printf(" (CONTROLA)");
			if (i == 0x0f) printf(" (CONTROLB)");
			if (i == 0x10) printf(" (CONTROLC)");
			if (i == 0x11) printf(" (CONTROLD - bit7=WDT)");
			if (i == 0x12) printf(" (PDDIS)");
			if (i == 0x13) printf(" (INTERFACE)");
			if (i == 0x14) printf(" (RESET)");
			printf("\n");
		}
	}

	/* Read CONTROLD register (0x11) */
	ret = i2c_read(&pmic_i2c, 0x49, 0x11, 1, &reg_val, 1);
	if (ret != 0) {
		printf("ATBM: Failed to read PMIC CONTROLD register (ret=%d)\n", ret);
		return -1;
	}

	printf("\nATBM: CONTROLD register analysis:\n");
	printf("ATBM:   Raw value: 0x%02x\n", reg_val);
	printf("ATBM:   Bit 7 (WATCHDOG): %d\n", (reg_val >> 7) & 1);
	printf("ATBM:   Bit 5 (ONKEYAUTOBOOTEN): %d\n", (reg_val >> 5) & 1);
	printf("ATBM:   Bit 4 (ONKEYSD): %d\n", (reg_val >> 4) & 1);
	printf("ATBM:   Bit 3 (KEEPACTEN): %d\n", (reg_val >> 3) & 1);

	/* Try setting bit 7 to 1 (enable watchdog) then back to 0 */
	printf("\nATBM: Testing watchdog control...\n");

	/* First, try to SET the watchdog bit */
	unsigned char test_val = reg_val | (1 << 7);
	printf("ATBM: Setting watchdog bit (0x%02x -> 0x%02x)...\n", reg_val, test_val);
	ret = i2c_write(&pmic_i2c, 0x49, 0x11, 1, &test_val, 1);
	if (ret != 0) {
		printf("ATBM: Failed to write CONTROLD (ret=%d)\n", ret);
		return -1;
	}

	/* Read back */
	ret = i2c_read(&pmic_i2c, 0x49, 0x11, 1, &reg_val, 1);
	printf("ATBM: After setting bit 7: 0x%02x\n", reg_val);

	/* Now clear it */
	unsigned char new_val = reg_val & ~(1 << 7);
	printf("ATBM: Clearing watchdog bit (0x%02x -> 0x%02x)...\n", reg_val, new_val);
	ret = i2c_write(&pmic_i2c, 0x49, 0x11, 1, &new_val, 1);
	if (ret != 0) {
		printf("ATBM: Failed to write CONTROLD (ret=%d)\n", ret);
		return -1;
	}

	/* Verify */
	ret = i2c_read(&pmic_i2c, 0x49, 0x11, 1, &reg_val, 1);
	printf("ATBM: After clearing bit 7: 0x%02x\n", reg_val);

	if ((reg_val & (1 << 7)) == 0) {
		printf("\nATBM: PMIC watchdog bit is now 0\n");
	} else {
		printf("\nATBM: WARNING: Could not clear watchdog bit!\n");
	}

	/* Also check RESET register (0x14) which has RESETTIMER field */
	printf("\nATBM: Checking RESET register (0x14)...\n");
	ret = i2c_read(&pmic_i2c, 0x49, 0x14, 1, &reg_val, 1);
	if (ret == 0) {
		printf("ATBM: RESET register: 0x%02x\n", reg_val);
		printf("ATBM:   RESETEVENT (bits 6-7): %d\n", (reg_val >> 6) & 3);
		printf("ATBM:   RESETTIMER (bits 0-5): %d\n", reg_val & 0x3f);

		/* Try to set RESETTIMER to maximum (63) to delay any reset */
		unsigned char reset_val = (reg_val & 0xc0) | 0x3f;
		if (reset_val != reg_val) {
			printf("ATBM: Setting RESETTIMER to max (0x%02x -> 0x%02x)...\n", reg_val, reset_val);
			ret = i2c_write(&pmic_i2c, 0x49, 0x14, 1, &reset_val, 1);
			if (ret == 0) {
				ret = i2c_read(&pmic_i2c, 0x49, 0x14, 1, &reg_val, 1);
				printf("ATBM: RESET register after write: 0x%02x\n", reg_val);
			}
		}
	}

	printf("\nATBM: Waiting to see if system stays alive...\n");
	return 0;
}

/* Brute-force try all GPIOs to find WiFi power pin */
static int atbm_brute_force_gpio_power(void)
{
	int gpio;
	int ret;

	/*
	 * Skip known GPIOs that are used for other functions:
	 * - GPIO 0-23: Port A (UART, SPI, I2C, etc.)
	 * - GPIO 24: UART1 RX (has pull-up in board.c)
	 * - GPIO 32-47: Port B lower (MMC0 data/cmd/clk)
	 * - GPIO 48: Skip (might be critical)
	 * - GPIO 49-58: IR-CUT and other camera controls
	 *
	 * Try GPIOs 25-31 (Port A upper) and 59-95 (Port B/C)
	 */

	printf("ATBM: BRUTE FORCE GPIO SCAN - Finding WiFi power pin\n");
	printf("ATBM: Will try each GPIO, test SDIO, then move to next\n");
	printf("ATBM: This may take a while...\n\n");

	/* Try Port A upper (GPIO 25-31) - FAST MODE */
	for (gpio = 25; gpio <= 31; gpio++) {
		printf("ATBM: Testing GPIO %d... ", gpio);

		/* Set GPIO as output HIGH */
		gpio_direction_output(gpio, 1);

		/* Wait for power to stabilize - REDUCED to 100ms */
		udelay(100000);

		/* Reset MSC1 controller */
		msc1_writel(MSC_CTRL_RESET, MSC_CTRL);
		udelay(2000);
		msc1_writel(MSC_CTRL_CLOCK_START, MSC_CTRL);
		udelay(2000);

		/* Quick test - just try CMD5 */
		ret = atbm_sdio_quick_test();
		if (ret == 0) {
			printf("SUCCESS!\n");
			printf("\n*** FOUND IT! GPIO %d is the WiFi power pin! ***\n\n", gpio);
			/* Do full init sequence to confirm */
			ret = atbm_sdio_init_sequence();
			if (ret == 0) {
				return gpio;
			}
		}

		/* Didn't work, turn off and try next */
		printf("no response\n");
		gpio_direction_output(gpio, 0);
		udelay(20000);  /* 20ms - REDUCED */
	}

	/* Try Port B upper (GPIO 59-63) */
	for (gpio = 59; gpio <= 63; gpio++) {
		printf("\n========================================\n");
		printf("ATBM: Testing GPIO %d (Port B)\n", gpio);
		printf("========================================\n");

		gpio_direction_output(gpio, 1);
		printf("ATBM: GPIO %d set to HIGH\n", gpio);

		udelay(200000);  /* 200ms - REDUCED */

		msc1_writel(MSC_CTRL_RESET, MSC_CTRL);
		udelay(5000);
		msc1_writel(MSC_CTRL_CLOCK_START, MSC_CTRL);
		udelay(5000);

		ret = atbm_sdio_init_sequence();
		if (ret == 0) {
			printf("\n*** SUCCESS! GPIO %d is the WiFi power pin! ***\n\n", gpio);
			return gpio;
		}

		printf("ATBM: GPIO %d - no response\n", gpio);
		gpio_direction_output(gpio, 0);
		udelay(50000);  /* 50ms - REDUCED */
	}

	/* Try Port C (GPIO 64-95) */
	for (gpio = 64; gpio <= 95; gpio++) {
		printf("\n========================================\n");
		printf("ATBM: Testing GPIO %d (Port C)\n", gpio);
		printf("========================================\n");

		gpio_direction_output(gpio, 1);
		printf("ATBM: GPIO %d set to HIGH\n", gpio);

		udelay(200000);  /* 200ms - REDUCED */

		msc1_writel(MSC_CTRL_RESET, MSC_CTRL);
		udelay(5000);
		msc1_writel(MSC_CTRL_CLOCK_START, MSC_CTRL);
		udelay(5000);

		ret = atbm_sdio_init_sequence();
		if (ret == 0) {
			printf("\n*** SUCCESS! GPIO %d is the WiFi power pin! ***\n\n", gpio);
			return gpio;
		}

		printf("ATBM: GPIO %d - no response\n", gpio);
		gpio_direction_output(gpio, 0);
		udelay(50000);  /* 50ms - REDUCED */
	}

	printf("\n========================================\n");
	printf("ATBM: BRUTE FORCE FAILED - No GPIO found!\n");
	printf("========================================\n");
	return -1;
}

/* Send a simple SDIO command (no data) and optionally read response */
static int atbm_send_simple_cmd(unsigned int cmd, unsigned int arg, unsigned int resp_type, unsigned int *resp_out)
{
	unsigned int stat;
	int timeout = 10000;  /* REDUCED from 100000 to 10000 (100ms) */

	/* Set command and argument */
	msc1_writel(cmd, MSC_CMD);
	msc1_writel(arg, MSC_ARG);

	/* Clear interrupts */
	msc1_writel(0xffffffff, MSC_IFLG);

	/* Set command data register (no data, just response) */
	msc1_writel(resp_type, MSC_CMDAT);

	/* Start the operation */
	msc1_writel(MSC_CTRL_CLOCK_START | MSC_CTRL_START_OP, MSC_CTRL);

	/* Wait for command response */
	while (!(stat = (msc1_readl(MSC_IFLG) & (MSC_IREG_END_CMD_RES | MSC_IREG_TIME_OUT_RES)))) {
		udelay(10);
		if (--timeout <= 0) {
			return -1;  /* Timeout */
		}
	}
	msc1_writel(stat, MSC_IFLG);

	if (stat & MSC_IREG_TIME_OUT_RES) {
		return -1;  /* Timeout */
	}

	/* Read response if requested */
	if (resp_out != NULL && resp_type != 0) {
		*resp_out = msc1_readl(MSC_RES);  /* Read first response register */
	}

	return 0;  /* Success */
}

/*
 * Send CMD52 - SDIO direct I/O (single byte read/write)
 * This works WITHOUT full SDIO enumeration!
 *
 * arg format:
 *   bit 31: R/W flag (0=read, 1=write)
 *   bit 28: RAW flag
 *   bits 25-9: Register address
 *   bits 7-0: Write data (if R/W=1)
 *
 * NOTE: Non-static so it can be used by atbm_firmware_loader.c
 */
int atbm_send_cmd52(unsigned int func, unsigned int addr, unsigned int write, unsigned char data, unsigned char *resp_data)
{
	unsigned int arg = 0;
	unsigned int resp;
	int ret;

	/* Build CMD52 argument */
	arg |= (write & 1) << 31;        /* R/W flag */
	arg |= (func & 7) << 28;         /* Function number */
	arg |= (addr & 0x1FFFF) << 9;    /* Register address */
	arg |= (data & 0xFF);            /* Write data */

	ret = atbm_send_simple_cmd(52, arg, MSC_CMDAT_RESPONSE_R1, &resp);

	if (ret == 0 && resp_data != NULL) {
		*resp_data = resp & 0xFF;  /* Response data is in bits 7:0 */
	}

	return ret;
}

/* Quick SDIO test - just try CMD5 */
static int atbm_sdio_quick_test(void)
{
	int ret;

	/* Just try CMD5 - IO_SEND_OP_COND (R4 response) */
	ret = atbm_send_simple_cmd(5, 0x00000000, MSC_CMDAT_RESPONSE_R1, NULL);
	return ret;
}

/*
 * Initialize SDIO device with proper enumeration sequence
 * Based on SDIO 3.0 spec and Linux mmc_sdio_init_card()
 */
static int atbm_sdio_init_sequence(void)
{
	int ret;
	unsigned int resp;
	unsigned int ocr;
	unsigned int rca;
	int i;

	printf("ATBM: Sending SDIO init sequence...\n");

	/* CMD0 - GO_IDLE_STATE (no response) */
	printf("ATBM: CMD0 (GO_IDLE_STATE)...\n");
	ret = atbm_send_simple_cmd(0, 0, 0, NULL);
	udelay(10000);  /* 10ms */

	/* CMD5 with arg=0 to query OCR (Operating Conditions Register) */
	printf("ATBM: CMD5 (IO_SEND_OP_COND) - query OCR...\n");
	ret = atbm_send_simple_cmd(5, 0x00000000, MSC_CMDAT_RESPONSE_R1, &resp);
	if (ret != 0) {
		printf("ATBM: CMD5 query failed - device not responding\n");
		return -1;
	}

	ocr = resp & 0x00FFFFFF;  /* Bits 23:0 contain voltage ranges */
	printf("ATBM: Device OCR: 0x%06x\n", ocr);

	/* CMD5 with voltage to initialize - retry until ready */
	printf("ATBM: CMD5 (IO_SEND_OP_COND) - initialize...\n");
	for (i = 0; i < 100; i++) {
		ret = atbm_send_simple_cmd(5, ocr, MSC_CMDAT_RESPONSE_R1, &resp);
		if (ret != 0) {
			printf("ATBM: CMD5 init failed\n");
			return -1;
		}

		/* Check if card is ready (bit 31 = 1) */
		if (resp & 0x80000000) {
			printf("ATBM: Device ready! OCR=0x%08x\n", resp);
			break;
		}

		udelay(10000);  /* 10ms between retries */
	}

	if (i >= 100) {
		printf("ATBM: Device did not become ready\n");
		return -1;
	}

	/* CMD3 - SEND_RELATIVE_ADDR (R6 response) */
	printf("ATBM: CMD3 (SEND_RELATIVE_ADDR)...\n");
	ret = atbm_send_simple_cmd(3, 0, MSC_CMDAT_RESPONSE_R1, &resp);
	if (ret != 0) {
		printf("ATBM: CMD3 failed\n");
		return -1;
	}

	rca = (resp >> 16) & 0xFFFF;
	printf("ATBM: RCA: 0x%04x\n", rca);

	/* CMD7 - SELECT_CARD (R1b response) */
	printf("ATBM: CMD7 (SELECT_CARD)...\n");
	ret = atbm_send_simple_cmd(7, rca << 16, MSC_CMDAT_RESPONSE_R1, NULL);
	if (ret != 0) {
		printf("ATBM: CMD7 failed\n");
		return -1;
	}

	printf("ATBM: SDIO device successfully initialized!\n");
	return 0;  /* Success! */
}

static int atbm_sdio_cmd53_write(unsigned int addr, const unsigned char *buf, unsigned int len)
{
	unsigned int cmd_arg;
	unsigned int stat;
	unsigned int cmdat;
	int i;
	
	/* Build CMD53 argument for write */
	cmd_arg = SDIO_CMD53_WRITE |        /* Write operation */
	          SDIO_CMD53_FUNC(1) |      /* Function 1 (WiFi data) */
	          SDIO_CMD53_BLOCK_MODE |   /* Block mode */
	          SDIO_CMD53_OP_CODE |      /* Increment address */
	          SDIO_CMD53_ADDR(addr) |   /* Start address */
	          SDIO_CMD53_COUNT(1);      /* 1 block */
	
	/* Set block length and number of blocks */
	msc1_writel(ATBM_SDIO_BLOCK_SIZE, MSC_BLKLEN);
	msc1_writel(1, MSC_NOB);
	
	/* Set command and argument */
	msc1_writel(SD_IO_RW_EXTENDED, MSC_CMD);
	msc1_writel(cmd_arg, MSC_ARG);
	
	/* Clear interrupts */
	msc1_writel(0xffffffff, MSC_IFLG);
	
	/* Set command data register: data enable, write, R5 response, 4-bit bus */
	cmdat = MSC_CMDAT_DATA_EN | MSC_CMDAT_WRITE | MSC_CMDAT_RESPONSE_R5 | MSC_CMDAT_BUS_WIDTH_4;
	msc1_writel(cmdat, MSC_CMDAT);
	
	/* Start the operation */
	msc1_writel(MSC_CTRL_CLOCK_START | MSC_CTRL_START_OP, MSC_CTRL);
	
	/* Wait for command response with timeout */
	{
		int timeout = 100000;  /* 1 second timeout */
		while (!(stat = (msc1_readl(MSC_IFLG) & (MSC_IREG_END_CMD_RES | MSC_IREG_TIME_OUT_RES)))) {
			udelay(10);
			if (--timeout <= 0) {
				printf("ATBM: CMD53 wait timeout (no response)\n");
				return -1;
			}
		}
		msc1_writel(stat, MSC_IFLG);
	}
	
	if (stat & MSC_IREG_TIME_OUT_RES) {
		printf("ATBM: CMD53 timeout\n");
		return -1;
	}
	
	/* Write data to TX FIFO */
	for (i = 0; i < len; i += 4) {
		unsigned int word;
		int timeout = 100000;  /* 1 second timeout */

		/* Wait for TX FIFO ready with timeout */
		while (!(msc1_readl(MSC_IFLG) & MSC_IREG_TXFIFO_WR_REQ)) {
			udelay(10);
			if (--timeout <= 0) {
				printf("ATBM: TX FIFO timeout at byte %d\n", i);
				return -1;
			}
		}
		
		/* Pack 4 bytes into word (little-endian) */
		if (i + 4 <= len) {
			word = buf[i] | (buf[i+1] << 8) | (buf[i+2] << 16) | (buf[i+3] << 24);
		} else {
			/* Handle partial word at end */
			word = 0;
			if (i < len) word |= buf[i];
			if (i+1 < len) word |= buf[i+1] << 8;
			if (i+2 < len) word |= buf[i+2] << 16;
		}
		
		msc1_writel(word, MSC_TXFIFO);
	}
	
	/* Wait for programming done with timeout */
	{
		int timeout = 100000;  /* 1 second timeout */
		while (!(msc1_readl(MSC_STAT) & MSC_STAT_PRG_DONE)) {
			udelay(10);
			if (--timeout <= 0) {
				printf("ATBM: Programming done timeout\n");
				return -1;
			}
		}
		msc1_writel(MSC_IREG_PRG_DONE, MSC_IFLG);
	}
	
	return 0;
}

static int atbm_send_keepalive_real(void)
{
	unsigned char buf[256];  /* Full SDIO block */
	unsigned int addr;
	
	/* Build WSM keepalive command */
	memset(buf, 0, sizeof(buf));
	
	/* WSM header: len=16, cmd=0x003c */
	atbm_write_le16(buf + 4, 16);
	atbm_write_le16(buf + 6, ATBM_WSM_CMD_KEEPALIVE);
	
	/* Payload: 0x00000000, 0x000004BC, interval_s=3, max_fails=1 */
	atbm_write_le32(buf + 8, 0);
	atbm_write_le32(buf + 12, 0x000004bc);
	atbm_write_le32(buf + 16, 3);  /* 3 second interval */
	atbm_write_le32(buf + 20, 1);  /* max_fails */
	
	/* 
	 * Write to SDIO data address
	 * Address calculation: (base << 5) | (slot << 6)
	 * For first keepalive: base=0, slot=0, addr=0x00000000
	 */
	addr = 0x00000000;
	
	printf("ATBM: Sending keepalive via SDIO CMD53 to addr 0x%08x\n", addr);
	
	/* Send via SDIO CMD53 */
	return atbm_sdio_cmd53_write(addr, buf, ATBM_SDIO_BLOCK_SIZE);
}

static void msc1_basic_init(void)
{
	/* Basic MSC1 controller initialization */
	
	/* Reset controller */
	msc1_writel(MSC_CTRL_RESET, MSC_CTRL);
	udelay(1000);
	
	/* Set clock divider (divide by 2 for safety) */
	msc1_writel(1, MSC_CLKRT);
	
	/* Set timeout values */
	msc1_writel(0xffff, MSC_RESTO);
	msc1_writel(0xffffffff, MSC_RDTO);
	
	/* Mask all interrupts */
	msc1_writel(0xffffffff, MSC_IMASK);
	
	/* Clear all interrupt flags */
	msc1_writel(0xffffffff, MSC_IFLG);
	
	/* Start clock */
	msc1_writel(MSC_CTRL_CLOCK_START, MSC_CTRL);
	udelay(1000);
	
	printf("ATBM: MSC1 controller initialized\n");
}

int atbm_init_keepalive(void)
{
	int ret, count = 0;
	int gpio_found = -1;
	struct mmc *mmc;

	printf("\n");
	printf("========================================\n");
	printf("ATBM6441 WiFi Keepalive - FIRMWARE LOAD\n");
	printf("========================================\n");
	printf("\n");

	/* GPIO power already enabled in board.c, skip it here */

	/* Check if MSC0 (SDIO) is already initialized */
	printf("ATBM: Checking MMC subsystem status...\n");
	mmc = find_mmc_device(0);  /* Device 0 = MSC0 (ATBM6441 is here!) */
	if (!mmc) {
		printf("ATBM: ERROR - MMC device 0 not found!\n");
		return -1;
	}

	printf("ATBM: MMC device 0 found!\n");
	printf("ATBM:   - Name: %s\n", mmc->name);
	printf("ATBM:   - Clock: %u Hz\n", mmc->clock);
	printf("ATBM:   - Bus width: %u\n", mmc->bus_width);
	printf("ATBM:   - Initialized: %s\n", mmc->has_init ? "YES" : "NO");
	printf("ATBM:   - Is SDIO: %s\n", mmc->is_sdio ? "YES" : "NO");

	if (!mmc->has_init) {
		printf("ATBM: ERROR - MMC device not initialized!\n");
		return -1;
	}

	if (!mmc->is_sdio) {
		printf("ATBM: ERROR - Device is not SDIO!\n");
		return -1;
	}

	printf("\n");
	printf("ATBM: SDIO device ready - testing CMD52 communication...\n");
	printf("\n");

	/* Test CMD52 - Read CCCR register 0x00 (SDIO version) */
	struct mmc_cmd cmd;
	cmd.cmdidx = 52;  /* CMD52 = IO_RW_DIRECT */
	cmd.resp_type = MMC_RSP_R5;  /* R5 response for SDIO */
	/* CMD52 arg format: [31]=R/W, [30:28]=Function, [27]=RAW, [26]=0, [25:9]=Address, [8]=0, [7:0]=Data */
	/* Read function 0, address 0x00 (CCCR) */
	cmd.cmdarg = 0x00000000;  /* Read, Function 0, Address 0x00 */

	printf("ATBM: Sending CMD52 to read CCCR register 0x00...\n");
	ret = mmc_send_cmd(mmc, &cmd, NULL);
	printf("ATBM: CMD52 returned: %d\n", ret);
	if (ret == 0) {
		printf("ATBM: CMD52 response: 0x%08x\n", cmd.response[0]);
		unsigned char data = cmd.response[0] & 0xFF;
		printf("ATBM: CCCR register 0x00 = 0x%02x\n", data);
		printf("ATBM: SUCCESS! SDIO device is responding to CMD52!\n");
	} else {
		printf("ATBM: ERROR - CMD52 failed!\n");
		return -1;
	}

	printf("\n");
	printf("ATBM: Proceeding with firmware loading...\n");
	printf("\n");

	/* Call the firmware loader */
	extern int atbm_load_firmware(void);
	ret = atbm_load_firmware();

	if (ret == 0) {
		printf("\n");
		printf("========================================\n");
		printf("ATBM: FIRMWARE LOADED SUCCESSFULLY!\n");
		printf("ATBM: System should stay alive now!\n");
		printf("========================================\n");
		printf("\n");

		/* Wait and see if we stay alive */
		int alive_count = 0;
		while (1) {
			udelay(1000000);  /* 1 second */
			alive_count++;
			if (alive_count % 5 == 0) {
				printf("ATBM: System alive for %d seconds (firmware loaded!)\n", alive_count);
			}
		}
	}

	printf("\nATBM: Firmware loading failed, trying basic SDIO test...\n");
	printf("\n");

	printf("\nATBM: Waiting 100ms...\n");
	udelay(100000);  /* 100ms */

	/* Try SDIO init */
	printf("ATBM: Testing SDIO...\n");
	ret = atbm_sdio_init_sequence();

	if (ret == 0) {
		printf("\n");
		printf("========================================\n");
		printf("ATBM: SUCCESS! SDIO device responded after regulator enable!\n");
		printf("ATBM: WiFi chip is powered by PMIC regulator!\n");
		printf("========================================\n");
		printf("\n");
		gpio_found = 1;
		goto start_keepalive;
	}

	printf("ATBM: SDIO still not responding after regulator enable\n");
	printf("ATBM: Skipping GPIO shotgun (wastes time, confirmed no GPIO needed)\n");
	printf("\n");

	/*
	 * SKIP GPIO SHOTGUN - We confirmed from platform data that no GPIO
	 * is configured for WiFi power. The chip is always powered by PMIC.
	 * The GPIO shotgun wastes 600ms+ which is critical time.
	 */

start_keepalive:
	printf("\n");
	printf("========================================\n");
	printf("ATBM: STEP 2 - CMD52 Direct I/O Keepalive\n");
	printf("========================================\n");
	printf("\n");

	printf("ATBM: Attempting minimal firmware initialization...\n");
	printf("ATBM: The chip needs firmware loaded before watchdog can be fed!\n");
	printf("\n");

	/*
	 * CRITICAL DISCOVERY: The ATBM6441 has an internal MCU that needs
	 * firmware (mcu_fw.bin) loaded via SDIO before it will respond properly.
	 * The watchdog is in the MCU and won't be satisfied until firmware loads.
	 *
	 * The Linux driver does:
	 * 1. atbm_before_load_firmware() - Initialize chip registers
	 * 2. Upload mcu_fw.bin via AHB writes
	 * 3. atbm_after_load_firmware() - Finalize configuration
	 *
	 * We don't have time to implement full firmware loading in 6 seconds,
	 * but we can try the initialization sequence to see if it helps.
	 */

	printf("ATBM: Trying to initialize chip registers (atbm_before_load_firmware)...\n");

	/*
	 * From decompiled code, the initialization sequence is:
	 * 1. Read register 0 (control register)
	 * 2. Read register 1 (config register)
	 * 3. Set bit 12 (0x1000) in register 1
	 * 4. Write back to register 1
	 * 5. Poll register 1 until bit 13 (0x2000) is set
	 * 6. Configure various control bits
	 *
	 * These are SDIO function registers, not CCCR registers.
	 * We need CMD52 to access them.
	 */

	/* Try to read SDIO function 0 register 0 (CCCR revision) */
	unsigned char cccr_rev;
	ret = atbm_send_cmd52(0, 0x00, 0, 0, &cccr_rev);
	if (ret == 0) {
		printf("ATBM: CCCR Revision: 0x%02x\n", cccr_rev);
	} else {
		printf("ATBM: Failed to read CCCR - chip not responding to CMD52\n");
	}

	/* Try to enable Function 1 */
	unsigned char io_enable = 0x02;  /* Enable Function 1 */
	ret = atbm_send_cmd52(0, 0x02, 1, io_enable, NULL);
	if (ret == 0) {
		printf("ATBM: Enabled Function 1\n");
	} else {
		printf("ATBM: Failed to enable Function 1\n");
	}

	udelay(10000);  /* 10ms */

	/*
	 * Without full firmware loading, the watchdog will still reboot us.
	 * The only real solution is to:
	 * 1. Load firmware in U-Boot (complex, needs mcu_fw.bin embedded)
	 * 2. Speed up Linux boot to load driver faster
	 * 3. Find a hardware watchdog disable mechanism
	 *
	 * For now, just print diagnostic info and let it reboot.
	 */

	printf("\n");
	printf("========================================\n");
	printf("ATBM: FIRMWARE LOADING REQUIRED\n");
	printf("========================================\n");
	printf("\n");
	printf("The ATBM6441 WiFi chip requires firmware (mcu_fw.bin) to be\n");
	printf("loaded via SDIO before the watchdog can be satisfied.\n");
	printf("\n");
	printf("Solutions:\n");
	printf("1. Implement full firmware loading in U-Boot\n");
	printf("2. Speed up Linux boot to load driver within 6 seconds\n");
	printf("3. Find hardware watchdog disable mechanism\n");
	printf("\n");
	printf("========================================\n");
	printf("ATBM: DROPPING TO U-BOOT PROMPT\n");
	printf("========================================\n");
	printf("\n");
	printf("You can now use U-Boot commands to debug SDIO:\n");
	printf("  mmc list           - List MMC devices\n");
	printf("  mmc dev 1          - Select MMC device 1 (SDIO)\n");
	printf("  mmc info           - Show MMC device info\n");
	printf("  mmc rescan         - Rescan for MMC devices\n");
	printf("\n");
	printf("WARNING: System will reboot in ~6 seconds due to watchdog!\n");
	printf("Work fast or be prepared for multiple reboots.\n");
	printf("\n");

	/* Return to U-Boot prompt instead of infinite loop */
	return;
}

#endif /* CONFIG_JZ_MMC_MSC1 */

