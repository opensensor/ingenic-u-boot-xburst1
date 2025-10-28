/*
 * ATBM6441 Firmware Loader for U-Boot
 *
 * This implements the firmware loading sequence from the Linux driver
 * to satisfy the WiFi chip's watchdog before it reboots the system.
 *
 * Copyright (c) 2025
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <mmc.h>

/* Embedded firmware */
#include "firmware/mcu_fw.h"

/* SDIO Function 0 (CCCR) registers */
#define SDIO_CCCR_CCCR          0x00
#define SDIO_CCCR_SD            0x01
#define SDIO_CCCR_IOEx          0x02  /* I/O Enable */
#define SDIO_CCCR_IORx          0x03  /* I/O Ready */
#define SDIO_CCCR_IENx          0x04  /* Int Enable */
#define SDIO_CCCR_INTx          0x05  /* Int Pending */
#define SDIO_CCCR_ABORT         0x06  /* I/O Abort */
#define SDIO_CCCR_IF            0x07  /* Bus Interface Control */

/* ATBM6441 SDIO registers (Function 0) */
#define ATBM_HIFREG_CONFIG_REG_ID           0
#define ATBM_HIFREG_CONTROL_REG_ID          1
#define ATBM_HIFREG_IN_OUT_QUEUE_REG_ID     2
#define ATBM_HIFREG_AHB_DPORT_REG_ID        3
#define ATBM_HIFREG_SRAM_BASE_ADDR_REG_ID   4
#define ATBM_HIFREG_SRAM_DPORT_REG_ID       5
#define ATBM_HIFREG_TSET_GEN_R_W_REG_ID     6
#define ATBM_HIFREG_FRAME_LEN_REG_ID        7

/* Control register bits */
#define ATBM_HIFREG_CONT_NEXT_LEN_MASK      0x0FFF
#define ATBM_HIFREG_CONT_WUP_BIT            (1 << 12)
#define ATBM_HIFREG_CONT_RDY_BIT            (1 << 13)

/* MSC registers */
#define MSC_CTRL        0x00
#define MSC_STAT        0x04
#define MSC_CLKRT       0x08
#define MSC_CMDAT       0x0C
#define MSC_RESTO       0x10
#define MSC_RDTO        0x14
#define MSC_BLKLEN      0x18
#define MSC_NOB         0x1C
#define MSC_SNOB        0x20
#define MSC_IMASK       0x24
#define MSC_IREG        0x28
#define MSC_CMD         0x2C
#define MSC_ARG         0x30
#define MSC_RES         0x34
#define MSC_RXFIFO      0x38
#define MSC_TXFIFO      0x3C

/* SDIO commands */
#define SD_CMD_GO_IDLE_STATE        0
#define SD_CMD_SEND_OP_COND         1
#define SD_CMD_ALL_SEND_CID         2
#define SD_CMD_SEND_RELATIVE_ADDR   3
#define SD_CMD_SELECT_CARD          7
#define SD_CMD_SEND_IF_COND         8

#define SDIO_CMD_SEND_OP_COND       5
#define SDIO_CMD_RW_DIRECT          52
#define SDIO_CMD_RW_EXTENDED        53

/* Response types */
#define MSC_CMDAT_RESPONSE_NONE     0
#define MSC_CMDAT_RESPONSE_R1       1
#define MSC_CMDAT_RESPONSE_R2       2
#define MSC_CMDAT_RESPONSE_R3       3
#define MSC_CMDAT_RESPONSE_R4       4
#define MSC_CMDAT_RESPONSE_R5       5
#define MSC_CMDAT_RESPONSE_R6       6

/* Forward declaration - use the CMD52 implementation from atbm_keepalive_v3.c */
extern int atbm_send_cmd52(unsigned int func, unsigned int addr, unsigned int write, unsigned char data, unsigned char *resp_data);

/*
 * Wrapper for CMD52 to match our naming
 */
static int atbm_sdio_cmd52(int func, int addr, int write, unsigned char data, unsigned char *result)
{
	return atbm_send_cmd52(func, addr, write, data, result);
}

/*
 * Read ATBM register via CMD52
 */
static int atbm_reg_read_u32(int reg_id, unsigned int *value)
{
	unsigned char byte0, byte1, byte2, byte3;
	int ret;

	/* ATBM registers are accessed via Function 0, starting at address 0x10 */
	int base_addr = 0x10 + (reg_id * 4);

	printf("ATBM: Reading register %d (addr 0x%02x)...\n", reg_id, base_addr);

	ret = atbm_sdio_cmd52(0, base_addr + 0, 0, 0, &byte0);
	if (ret < 0) {
		printf("ATBM: CMD52 read failed at offset 0 (ret=%d)\n", ret);
		return ret;
	}
	printf("ATBM:   byte0 = 0x%02x\n", byte0);

	ret = atbm_sdio_cmd52(0, base_addr + 1, 0, 0, &byte1);
	if (ret < 0) {
		printf("ATBM: CMD52 read failed at offset 1 (ret=%d)\n", ret);
		return ret;
	}
	printf("ATBM:   byte1 = 0x%02x\n", byte1);

	ret = atbm_sdio_cmd52(0, base_addr + 2, 0, 0, &byte2);
	if (ret < 0) {
		printf("ATBM: CMD52 read failed at offset 2 (ret=%d)\n", ret);
		return ret;
	}
	printf("ATBM:   byte2 = 0x%02x\n", byte2);

	ret = atbm_sdio_cmd52(0, base_addr + 3, 0, 0, &byte3);
	if (ret < 0) {
		printf("ATBM: CMD52 read failed at offset 3 (ret=%d)\n", ret);
		return ret;
	}
	printf("ATBM:   byte3 = 0x%02x\n", byte3);

	*value = byte0 | (byte1 << 8) | (byte2 << 16) | (byte3 << 24);
	printf("ATBM: Register %d = 0x%08x\n", reg_id, *value);
	return 0;
}

/*
 * Write ATBM register via CMD52
 */
static int atbm_reg_write_u32(int reg_id, unsigned int value)
{
	int ret;
	int base_addr = 0x10 + (reg_id * 4);
	
	ret = atbm_sdio_cmd52(0, base_addr + 0, 1, (value >> 0) & 0xFF, NULL);
	if (ret < 0) return ret;
	
	ret = atbm_sdio_cmd52(0, base_addr + 1, 1, (value >> 8) & 0xFF, NULL);
	if (ret < 0) return ret;
	
	ret = atbm_sdio_cmd52(0, base_addr + 2, 1, (value >> 16) & 0xFF, NULL);
	if (ret < 0) return ret;
	
	ret = atbm_sdio_cmd52(0, base_addr + 3, 1, (value >> 24) & 0xFF, NULL);
	if (ret < 0) return ret;
	
	return 0;
}

/*
 * Initialize ATBM chip before firmware loading
 * Based on atbm_before_load_firmware() from Linux driver
 */
static int atbm_before_load_firmware(void)
{
	unsigned int reg_val;
	int ret;
	int timeout;
	
	printf("ATBM: before_load_firmware: Starting chip initialization...\n");
	
	/* Read register 0 (CONFIG) */
	ret = atbm_reg_read_u32(ATBM_HIFREG_CONFIG_REG_ID, &reg_val);
	if (ret < 0) {
		printf("ATBM: Failed to read CONFIG register\n");
		return ret;
	}
	printf("ATBM: CONFIG register: 0x%08x\n", reg_val);
	
	/* Read register 1 (CONTROL) */
	ret = atbm_reg_read_u32(ATBM_HIFREG_CONTROL_REG_ID, &reg_val);
	if (ret < 0) {
		printf("ATBM: Failed to read CONTROL register\n");
		return ret;
	}
	printf("ATBM: CONTROL register: 0x%08x\n", reg_val);
	
	/* Set WUP bit (bit 12) in CONTROL register */
	reg_val |= ATBM_HIFREG_CONT_WUP_BIT;
	ret = atbm_reg_write_u32(ATBM_HIFREG_CONTROL_REG_ID, reg_val);
	if (ret < 0) {
		printf("ATBM: Failed to set WUP bit\n");
		return ret;
	}
	printf("ATBM: Set WUP bit in CONTROL register\n");
	
	/* Poll for RDY bit (bit 13) in CONTROL register */
	printf("ATBM: Waiting for RDY bit...\n");
	timeout = 3000;  /* 3 seconds */
	while (timeout-- > 0) {
		ret = atbm_reg_read_u32(ATBM_HIFREG_CONTROL_REG_ID, &reg_val);
		if (ret < 0) {
			printf("ATBM: Failed to read CONTROL register\n");
			return ret;
		}
		
		if (reg_val & ATBM_HIFREG_CONT_RDY_BIT) {
			printf("ATBM: RDY bit set! CONTROL=0x%08x\n", reg_val);
			break;
		}
		
		udelay(1000);  /* 1ms */
	}
	
	if (timeout <= 0) {
		printf("ATBM: Timeout waiting for RDY bit\n");
		return -1;
	}
	
	printf("ATBM: Chip initialization complete\n");
	return 0;
}

/*
 * Main firmware loading entry point
 */
int atbm_load_firmware(void)
{
	int ret;
	
	printf("\n");
	printf("========================================\n");
	printf("ATBM6441 Firmware Loader\n");
	printf("========================================\n");
	printf("Firmware size: %u bytes\n", mcu_fw_bin_len);
	printf("\n");
	
	/* Step 1: Initialize chip */
	ret = atbm_before_load_firmware();
	if (ret < 0) {
		printf("ATBM: Chip initialization failed!\n");
		return ret;
	}
	
	printf("\nATBM: Firmware loading not yet implemented\n");
	printf("ATBM: But chip initialization succeeded!\n");
	printf("ATBM: This proves we can communicate with the WiFi chip!\n");
	
	return 0;
}

