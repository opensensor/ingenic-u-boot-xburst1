/*
 * Ingenic JZ MMC driver
 *
 * Copyright (c) 2013 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <mmc.h>
#include <asm/io.h>
#include <asm/arch/clk.h>
#include <asm/arch/mmc.h>
#include <asm/unaligned.h>
#include <asm/arch/cpm.h>
#include <asm/arch/base.h>

#include <watchdog.h>


struct jz_mmc_priv {
	uintptr_t base;
	uint32_t flags;
	int clk;
};

#define udelay(int) do{}while(0); //for decreasing time by ykliu
/* jz_mmc_priv flags */
#define JZ_MMC_BUS_WIDTH_MASK 0x3
#define JZ_MMC_BUS_WIDTH_1    0x0
#define JZ_MMC_BUS_WIDTH_4    0x2
#define JZ_MMC_BUS_WIDTH_8    0x3
#define JZ_MMC_SENT_INIT (1 << 2)

#ifdef CONFIG_SPL_BUILD
/* SPL will only use a single MMC device (CONFIG_JZ_MMC_SPLMSC) */
struct mmc mmc_dev[1];
struct jz_mmc_priv mmc_priv[1];
#else
struct mmc mmc_dev[3];
struct jz_mmc_priv mmc_priv[3];
#endif

static uint16_t jz_mmc_readw(struct jz_mmc_priv *priv, uintptr_t off)
{
	return readw(priv->base + off);
}

static uint32_t jz_mmc_readl(struct jz_mmc_priv *priv, uintptr_t off)
{
	return readl(priv->base + off);
}

static void jz_mmc_writel(uint32_t value, struct jz_mmc_priv *priv, uintptr_t off)
{
	writel(value, priv->base + off);
}

static void jz_mmc_soft_reset(struct jz_mmc_priv *priv)
{
	/* Save and restore clock divider */
	uint32_t clkrt = jz_mmc_readl(priv, MSC_CLKRT);

	/* Issue controller reset and wait until complete */
	jz_mmc_writel(MSC_CTRL_RESET, priv, MSC_CTRL);
#if defined(CONFIG_M200) || \
	defined(CONFIG_T10) || defined(CONFIG_T15) || \
	defined(CONFIG_T20) || defined(CONFIG_T21) || defined(CONFIG_T23) || \
	defined(CONFIG_T30) || defined(CONFIG_T31) || defined(CONFIG_C100)
	{
		uint32_t tmp = jz_mmc_readl(priv, MSC_CTRL);
		tmp &= ~MSC_CTRL_RESET;
		jz_mmc_writel(tmp, priv, MSC_CTRL);
	}
#else
	while (jz_mmc_readl(priv, MSC_STAT) & MSC_STAT_IS_RESETTING)
		;
#endif

	/* Re-enable continuous clock */
	{
		uint32_t ctrl = jz_mmc_readl(priv, MSC_CTRL);
		ctrl &= ~MSC_CTRL_CLOCK_CONTROL_MASK;
		ctrl |= MSC_CTRL_CLOCK_CONTROL_START;
		jz_mmc_writel(ctrl, priv, MSC_CTRL);
	}

	/* Clear any pending flags and restore clock divider */

	jz_mmc_writel(0xffffffff, priv, MSC_IFLG);
	jz_mmc_writel(clkrt, priv, MSC_CLKRT);
}

/* Extra helper: briefly stop and restart the MSC clock to nudge internal logic */
static void jz_mmc_stop_start_clock(struct jz_mmc_priv *priv)
{
	uint32_t ctrl = jz_mmc_readl(priv, MSC_CTRL);
	ctrl &= ~MSC_CTRL_CLOCK_CONTROL_MASK;
	ctrl |= MSC_CTRL_CLOCK_CONTROL_STOP;
	jz_mmc_writel(ctrl, priv, MSC_CTRL);
	udelay(50);
	ctrl &= ~MSC_CTRL_CLOCK_CONTROL_MASK;
	ctrl |= MSC_CTRL_CLOCK_CONTROL_START;
	jz_mmc_writel(ctrl, priv, MSC_CTRL);
	udelay(50);
}

/* Explicitly drain the response window a few times to discard any latched bytes */
static void jz_mmc_drain_res(struct jz_mmc_priv *priv)
{
	volatile uint32_t d;
	int i;
	for (i = 0; i < 8; ++i)
		d = jz_mmc_readl(priv, MSC_RES);
	(void)d;
}

/* CPM clock-gate hard reset helper: gate/ungate MSC clock to clear sticky state */
static void jz_mmc_cpm_gate_ungate(struct jz_mmc_priv *priv)
{
#if defined(CONFIG_T23) && defined(CONFIG_JZ_MMC_CPM_RESET_AFTER_SDIO_CMD5)
	uint32_t mask = 0;
	if (priv->base == MSC0_BASE)
		mask = CPM_CLKGR_MSC0;
	else if (priv->base == MSC1_BASE)
		mask = CPM_CLKGR_MSC1;
	if (!mask)
		return;

	/* Gate clock */
	uint32_t g = cpm_inl(CPM_CLKGR0);
	cpm_outl(g | mask, CPM_CLKGR0);
	{
		volatile int i;
		for (i = 0; i < 20000; ++i) ;
	}

	/* Ungate clock */
	cpm_outl((g | mask) & ~mask, CPM_CLKGR0);
	{
		volatile int i;
		for (i = 0; i < 20000; ++i) ;
	}

	/* Re-enable continuous clock and clear pending flags */
	uint32_t ctrl = jz_mmc_readl(priv, MSC_CTRL);
	ctrl &= ~MSC_CTRL_CLOCK_CONTROL_MASK;
	ctrl |= MSC_CTRL_CLOCK_CONTROL_START;
	jz_mmc_writel(ctrl, priv, MSC_CTRL);
	jz_mmc_writel(0xffffffff, priv, MSC_IFLG);

#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
	printf("CPM: Gate/ungate MSC clock done; CTRL=0x%08x, CLKGR0=0x%08x\n", ctrl, cpm_inl(CPM_CLKGR0));
#endif
#else
	(void)priv;
#endif
}


/* Force controller into a known idle state before sending a sensitive command */
static void jz_mmc_force_idle_before_cmd(struct jz_mmc_priv *priv, int cmdidx)
{
	/* 1) Stop clock */
	uint32_t ctrl = jz_mmc_readl(priv, MSC_CTRL);
	ctrl &= ~MSC_CTRL_CLOCK_CONTROL_MASK;
	ctrl |= MSC_CTRL_CLOCK_CONTROL_STOP;
	jz_mmc_writel(ctrl, priv, MSC_CTRL);
	udelay(10);

	/* 2) Clear ALL status bits (W1C) and ALL interrupt flags */
	jz_mmc_writel(0xffffffff, priv, MSC_STAT);
	udelay(10);
	jz_mmc_writel(0xffffffff, priv, MSC_IFLG);
	udelay(10);

	/* 3) Soft reset (toggle RESET bit) */
	jz_mmc_writel(MSC_CTRL_RESET, priv, MSC_CTRL);
	udelay(100);
	{
		uint32_t tmp = jz_mmc_readl(priv, MSC_CTRL);
		tmp &= ~MSC_CTRL_RESET;
		jz_mmc_writel(tmp, priv, MSC_CTRL);
	}
	udelay(10);

	/* 4) Restart clock in continuous mode */
	ctrl = jz_mmc_readl(priv, MSC_CTRL);
	ctrl &= ~MSC_CTRL_CLOCK_CONTROL_MASK;
	ctrl |= MSC_CTRL_CLOCK_CONTROL_START;
	jz_mmc_writel(ctrl, priv, MSC_CTRL);
	udelay(10);

	/* 5) Drain any residual response bytes */
	jz_mmc_drain_res(priv);

#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
	printf("JZ_MMC: Forced controller clear before CMD%d; STAT=0x%08x IFLG=0x%08x\n",
	       cmdidx, jz_mmc_readl(priv, MSC_STAT), jz_mmc_readl(priv, MSC_IFLG));
#endif
}


/* MSC register dump for debugging. Note: now also reads RES window (drains). */
static void jz_mmc_dump_regs(struct jz_mmc_priv *priv, const char *tag)
{
#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
	printf("\n=== JZ_MMC (%s) @0x%08lx ===\n", tag ? tag : "", priv->base);
	printf("CTRL=0x%08x STAT=0x%08x CLKRT=0x%08x CMDAT=0x%08x\n",
	       jz_mmc_readl(priv, MSC_CTRL), jz_mmc_readl(priv, MSC_STAT),
	       jz_mmc_readl(priv, MSC_CLKRT), jz_mmc_readl(priv, MSC_CMDAT));
	printf("RESTO=0x%08x RDTO=0x%08x IMASK=0x%08x IFLG=0x%08x\n",
	       jz_mmc_readl(priv, MSC_RESTO), jz_mmc_readl(priv, MSC_RDTO),
	       jz_mmc_readl(priv, MSC_IMASK), jz_mmc_readl(priv, MSC_IFLG));
	printf("CMD=0x%08x ARG=0x%08x LPM=0x%08x\n",
	       jz_mmc_readl(priv, MSC_CMD), jz_mmc_readl(priv, MSC_ARG),
	       jz_mmc_readl(priv, MSC_LPM));
	/* Dump 4 subsequent reads of RES window to inspect any latched bytes */
	{
		uint32_t r0 = jz_mmc_readl(priv, MSC_RES);
		uint32_t r1 = jz_mmc_readl(priv, MSC_RES);
		uint32_t r2 = jz_mmc_readl(priv, MSC_RES);
		uint32_t r3 = jz_mmc_readl(priv, MSC_RES);
		printf("RES0=0x%08x RES1=0x%08x RES2=0x%08x RES3=0x%08x\n", r0, r1, r2, r3);
	}
#endif
}


static int jz_mmc_send_cmd(struct mmc *mmc, struct mmc_cmd *cmd,
						struct mmc_data *data)
{
	struct jz_mmc_priv *priv = mmc->priv;
	uint32_t stat, cmdat = 0;

		/* Track last response to mitigate stale register reuse on timeouts */
		static uint32_t last_resp0 = 0;
		static uint32_t last_cmdidx = 0;

#if defined(CONFIG_T23) && defined(CONFIG_JZ_MMC_SDIO_SKIP_CMD3)
	/* Quirk: only skip CMD3 for SDIO devices to force default RCA=1 path */
	if ((cmd->cmdidx == 3) && mmc->is_sdio) {
	#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
		printf("JZ_MMC: Skipping CMD3 for SDIO due to RES quirk; forcing default RCA path\n");
	#endif
		return TIMEOUT; /* cause mmc core to use default RCA=0x0001 and proceed to CMD7 */
	}
#endif

		/* Workaround: after CMD5, controller may latch stale RES; reset before next cmd */
		if (last_cmdidx == 5 && cmd->cmdidx != 5) {
		#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
#if defined(CONFIG_JZ_MMC_CPM_RESET_AFTER_SDIO_CMD5)
#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
	printf("CPM: Gating MSC clock via CLKGR0 before post-CMD5 flush\n");
#endif
	jz_mmc_cpm_gate_ungate(priv);
		/* Nudge internal logic with a brief STOP/START and drain any leftover RES */
		jz_mmc_stop_start_clock(priv);
		jz_mmc_drain_res(priv);

	jz_mmc_dump_regs(priv, "post-CPM-reset after CMD5");
#endif
				jz_mmc_stop_start_clock(priv);
				jz_mmc_drain_res(priv);


			printf("JZ_MMC: Performing soft reset after CMD5 before CMD%d\n", cmd->cmdidx);
		#endif
			jz_mmc_soft_reset(priv);
				jz_mmc_stop_start_clock(priv);
				jz_mmc_drain_res(priv);


			/* Force next command to emit 80 init clocks to flush any latched response */
			priv->flags &= ~JZ_MMC_SENT_INIT;

				/* Extra flush: drain RES/IFLG and send a dummy CMD0 with INIT clocks (no response) */
				{
					jz_mmc_dump_regs(priv, "pre-flush after CMD5");
#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
					printf("JZ_MMC: Clearing CMD5 state from registers...\n");
#endif
					/* 1) Clear command and argument */
					jz_mmc_writel(0, priv, MSC_CMD);
					jz_mmc_writel(0, priv, MSC_ARG);
					udelay(10);
					/* 2) Clear ALL interrupt flags (write-1-to-clear) */
					jz_mmc_writel(0xffffffff, priv, MSC_IFLG);
					udelay(10);
					/* 3) Read out and discard the response register window */
					(void)jz_mmc_readl(priv, MSC_RES);
		/* If controller looks wedged (1f000000 pattern) or we're about to send CMD7,
		 * force an aggressive clear to make sure CMD7 actually transmits. */
		if ((cmd->cmdidx == 7 && mmc->is_sdio) ||
		    ((jz_mmc_readl(priv, MSC_STAT) & 0x1f000000) == 0x1f000000)) {
			jz_mmc_force_idle_before_cmd(priv, cmd->cmdidx);
		}

					(void)jz_mmc_readl(priv, MSC_RES);
					(void)jz_mmc_readl(priv, MSC_RES);
					(void)jz_mmc_readl(priv, MSC_RES);
					/* 4) Attempt to clear sticky status bits as well (W1C on some SoCs) */
					jz_mmc_writel(0xffffffff, priv, MSC_STAT);
					udelay(10);
#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
					printf("JZ_MMC: After clear - CMD=0x%08x ARG=0x%08x IFLG=0x%08x STAT=0x%08x\n",


					       jz_mmc_readl(priv, MSC_CMD),
					       jz_mmc_readl(priv, MSC_ARG),
					       jz_mmc_readl(priv, MSC_IFLG),
					       jz_mmc_readl(priv, MSC_STAT));
#endif
					uint32_t tmp, flush_cmdat;
					/* Now issue dummy CMD0 with INIT clocks (no response expected) */
					jz_mmc_writel(0xffffffff, priv, MSC_IFLG);
					/* Issue CMD0 with INIT clocks only (no response expected) */
					jz_mmc_writel(0, priv, MSC_CMD);
					jz_mmc_writel(0, priv, MSC_ARG);
					flush_cmdat = MSC_CMDAT_INIT | ((priv->flags & JZ_MMC_BUS_WIDTH_MASK) << 9);
					jz_mmc_writel(flush_cmdat, priv, MSC_CMDAT);
					jz_mmc_writel(MSC_CTRL_START_OP, priv, MSC_CTRL);
					/* Wait for TIME_OUT_RES/END_CMD_RES via STAT, then clear IFLG */
					int cnt = 10000;
					while (!((tmp = jz_mmc_readl(priv, MSC_STAT)) & (MSC_STAT_END_CMD_RES | MSC_STAT_TIME_OUT_RES)) && --cnt)
						;
					/* Clear any IFLG bits that may have latched */
					jz_mmc_writel(0xffffffff, priv, MSC_IFLG);
					/* Keep SENT_INIT clear so the upcoming real cmd also sends 80 clocks */
					priv->flags &= ~JZ_MMC_SENT_INIT;
					jz_mmc_dump_regs(priv, "post-flush after CMD5");
					/* Avoid repeatedly applying post-CMD5 flush to unrelated commands */
					last_cmdidx = 0;

#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
					printf("JZ_MMC: Issued dummy CMD0+INIT flush after CMD5\n");
#endif
				}

		}

	/* setup command: add CMD3-specific diagnostics and STAT clearing */
#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
	if (cmd->cmdidx == 3 || cmd->cmdidx == 7 || cmd->cmdidx == 52) {
		printf("JZ_MMC: About to send CMD%d\n", cmd->cmdidx);
		printf("  Pre-CMD%d: STAT=0x%08x IFLG=0x%08x CMD=0x%08x ARG=0x%08x\n",
		       cmd->cmdidx,
		       jz_mmc_readl(priv, MSC_STAT), jz_mmc_readl(priv, MSC_IFLG),
		       jz_mmc_readl(priv, MSC_CMD), jz_mmc_readl(priv, MSC_ARG));
		/* Force-clear STAT: write back current value (W1C), then mask-clear */
		uint32_t st = jz_mmc_readl(priv, MSC_STAT);
		jz_mmc_writel(st, priv, MSC_STAT);
		udelay(100);
		printf("  STAT after self-clear: 0x%08x\n", jz_mmc_readl(priv, MSC_STAT));
		jz_mmc_writel(0x1f000140, priv, MSC_STAT);
		udelay(100);
		printf("  STAT after mask clear: 0x%08x\n", jz_mmc_readl(priv, MSC_STAT));
	}
#endif

#if defined(CONFIG_T23)
	/* As a safety, always force-clear controller before CMD7 on SDIO */
	if (cmd->cmdidx == 7 && mmc->is_sdio) {
	#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
		printf("JZ_MMC: Forcing controller reset before CMD7\n");
	#endif
		jz_mmc_force_idle_before_cmd(priv, cmd->cmdidx);
	}
#endif

	jz_mmc_writel(cmd->cmdidx, priv, MSC_CMD);
	jz_mmc_writel(cmd->cmdarg, priv, MSC_ARG);
#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
	if (cmd->cmdidx == 3 || cmd->cmdidx == 7 || cmd->cmdidx == 52) {
		printf("  Post-write: CMD=0x%08x ARG=0x%08x\n",
		       jz_mmc_readl(priv, MSC_CMD), jz_mmc_readl(priv, MSC_ARG));
	}
#endif

	if (data) {
		/* setup data */
		cmdat |= MSC_CMDAT_DATA_EN;
#ifndef CONFIG_SPL_BUILD
		if (data->flags & MMC_DATA_WRITE)
			cmdat |= MSC_CMDAT_WRITE;
#endif

		jz_mmc_writel(data->blocks, priv, MSC_NOB);
		jz_mmc_writel(data->blocksize, priv, MSC_BLKLEN);
	}

	/* setup response: decode U-Boot MMC_RSP_* bitmask into MSC response format */
	{
		unsigned int rt = cmd->resp_type;
		/* Special-case CMD5: SDIO IO_SEND_OP_COND uses R4 on T23 */
		if (cmd->cmdidx == 5) {
			cmdat |= MSC_CMDAT_RESPONSE_R4;
		} else if (rt & MMC_RSP_PRESENT) {
			if (rt & MMC_RSP_136) {
				cmdat |= MSC_CMDAT_RESPONSE_R2; /* 136-bit */
			} else if ((rt & (MMC_RSP_CRC | MMC_RSP_OPCODE)) == (MMC_RSP_CRC | MMC_RSP_OPCODE)) {
				/* R1/R1b/R5/R6/R7 share the same 48-bit response with CRC+opcode */
				cmdat |= MSC_CMDAT_RESPONSE_R1;
			} else {
				/* R3: 48-bit response without CRC/opcode */
				cmdat |= MSC_CMDAT_RESPONSE_R3;
			}
		}
	}

	if (cmd->resp_type & MMC_RSP_BUSY)
		cmdat |= MSC_CMDAT_BUSY;

	/* set init for the first command only */
	if (!(priv->flags & JZ_MMC_SENT_INIT)) {
		cmdat |= MSC_CMDAT_INIT;
		priv->flags |= JZ_MMC_SENT_INIT;
	}

	cmdat |= (priv->flags & JZ_MMC_BUS_WIDTH_MASK) << 9;

	/* write the data setup */
	jz_mmc_writel(cmdat, priv, MSC_CMDAT);

#ifndef CONFIG_SPL_BUILD
	/* Keep IMASK/IFLG as-is; we'll clear flags after reading response */
#endif

	/* start the command (& the clock) */
	jz_mmc_writel(MSC_CTRL_START_OP, priv, MSC_CTRL);

#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
		if (cmd->cmdidx == 3 || cmd->cmdidx == 52) {
			printf("JZ_MMC: Flushing any stale RES before CMD%d completion\n", cmd->cmdidx);
		}
#endif
		if (cmd->cmdidx == 3 || cmd->cmdidx == 52) {
			volatile uint32_t d0 = jz_mmc_readl(priv, MSC_RES);
			volatile uint32_t d1 = jz_mmc_readl(priv, MSC_RES);
			volatile uint32_t d2 = jz_mmc_readl(priv, MSC_RES);
			volatile uint32_t d3 = jz_mmc_readl(priv, MSC_RES);
#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
			printf("  CMD%d pre-wait flush: RES=[0]=0x%08x [1]=0x%08x [2]=0x%08x [3]=0x%08x\n",
			       cmd->cmdidx, (uint32_t)d0, (uint32_t)d1, (uint32_t)d2, (uint32_t)d3);
#endif
		}


	/* wait for completion using STAT; add verbose trace for CMD3/CMD7/CMD52 */
	if (cmd->cmdidx == 3 || cmd->cmdidx == 7 || cmd->cmdidx == 52) {
#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
		printf("JZ_MMC: CMD%d waiting for completion...\n", cmd->cmdidx);
#endif
		int to = 10000;
		int tick = 0;
		do {
			stat = jz_mmc_readl(priv, MSC_STAT);
#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
			if ((tick++ % 100) == 0) {
				printf("JZ_MMC: CMD%d wait loop: STAT=0x%08x IFLG=0x%08x\n",
				       cmd->cmdidx, stat, jz_mmc_readl(priv, MSC_IFLG));
			}
#endif
			if (stat & (MSC_STAT_END_CMD_RES | MSC_STAT_TIME_OUT_RES))
				break;
			WATCHDOG_RESET();
			udelay(10);
		} while (--to);
#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
		printf("JZ_MMC: CMD%d completion detected (stat=0x%08x, remaining_to=%d)\n", cmd->cmdidx, stat, to);
		printf("  Post-CMD%d: STAT=0x%08x IFLG=0x%08x\n",
		       cmd->cmdidx, jz_mmc_readl(priv, MSC_STAT), jz_mmc_readl(priv, MSC_IFLG));
#endif
	} else {
		int to = 10000;
		do {
			stat = jz_mmc_readl(priv, MSC_STAT);
			if (stat & MSC_STAT_END_CMD_RES)
				break;
			/* TIME_OUT_RES can be sticky on T23; don't break solely on it */
			WATCHDOG_RESET();
			udelay(100);
		} while (--to);
	}
	/* Do not clear IFLG here; read response first like Linux driver */

	/* Determine if we may accept a response despite timeout (SDIO quirk) */
	int timed_out = (stat & MSC_STAT_TIME_OUT_RES) != 0;
	/* If END_CMD_RES is set, treat as non-timeout even if TIME_OUT_RES is also set (sticky bit) */
	if (stat & MSC_STAT_END_CMD_RES)
		timed_out = 0;
	int allow_timeout_resp = 0;
	if (timed_out && (cmd->resp_type & MMC_RSP_PRESENT)) {
		/* Only CMD5 (R4) can have valid response despite TIMEOUT on this controller */
		if (cmd->cmdidx == 5)
			allow_timeout_resp = 1;
	}

	if (cmd->resp_type & MMC_RSP_PRESENT) {
		/* read the response using 32-bit accesses like Linux driver */
		if (cmd->resp_type & MMC_RSP_136) {
			uint32_t res;
			int i;
			res = jz_mmc_readl(priv, MSC_RES);
			for (i = 0; i < 4; i++) {
				cmd->response[i] = res << 24;
				res = jz_mmc_readl(priv, MSC_RES);
				cmd->response[i] |= res << 8;
				res = jz_mmc_readl(priv, MSC_RES);
				cmd->response[i] |= res >> 8;
			}
		} else {
			/* Non-136-bit response (e.g., R1/R3/R4). Read raw words then assemble. */
			uint32_t r0 = jz_mmc_readl(priv, MSC_RES);
			uint32_t r1 = jz_mmc_readl(priv, MSC_RES);
			uint32_t r2 = jz_mmc_readl(priv, MSC_RES);
			cmd->response[0] = (r0 << 24) | (r1 << 8) | (r2 & 0xff);
		#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
			if (cmd->cmdidx == 5 || cmd->cmdidx == 3 || cmd->cmdidx == 7 || cmd->cmdidx == 52) {
				printf("JZ_MMC: Raw RES for CMD%d: r0=0x%08x r1=0x%08x r2=0x%08x -> resp[0]=0x%08x\n",
				       cmd->cmdidx, r0, r1, r2, cmd->response[0]);
			}
		#endif
		}

		/* If TIMEOUT flagged, decide whether to accept or fail */
		if (timed_out) {
			if (!allow_timeout_resp) {
				jz_mmc_writel(0xffffffff, priv, MSC_IFLG);
				return TIMEOUT;
			} else {
				/* Mitigate cached/stale response except for CMD5: reject if identical to last resp from a different cmd */
				if ((cmd->cmdidx != 5) && (cmd->response[0] == last_resp0) && (last_cmdidx != cmd->cmdidx)) {
				#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
					printf("JZ_MMC: TIMEOUT+stale resp reuse detected for CMD%d (resp=0x%08x); treating as TIMEOUT\n", cmd->cmdidx, cmd->response[0]);
				#endif
					jz_mmc_writel(0xffffffff, priv, MSC_IFLG);
					return TIMEOUT;
				}
			#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
				printf("JZ_MMC: TIMEOUT flagged but accepting response for CMD%d (resp=0x%08x) due to SDIO quirk\n", cmd->cmdidx, cmd->response[0]);
			#endif
			}
		}

		/* Update last response tracking when a response is actually read */
		last_resp0 = cmd->response[0];
		/* Now that response was read/handled, clear the IFLG bits for this cmd */
		jz_mmc_writel(0xffffffff, priv, MSC_IFLG);
#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
		if (cmd->cmdidx == 3 || cmd->cmdidx == 7 || cmd->cmdidx == 52) {
			uint32_t stp = jz_mmc_readl(priv, MSC_STAT);
			jz_mmc_writel(stp, priv, MSC_STAT);
			udelay(100);
			printf("  STAT after clear (post-CMD%d): 0x%08x\n", cmd->cmdidx, jz_mmc_readl(priv, MSC_STAT));
		}
#endif

		last_cmdidx = cmd->cmdidx;
	}

	/* For R1b, wait for PRG_DONE — but do not hang on SDIO; use a bounded wait */
	if (cmd->resp_type == MMC_RSP_R1b) {
		int to_prg;
		if (mmc->is_sdio) {
#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
			printf("JZ_MMC: R1b busy wait: SDIO detected; using bounded poll for PRG_DONE\n");
#endif
			to_prg = 2000; /* ~20ms at 10us step */
			while (!(jz_mmc_readl(priv, MSC_STAT) & MSC_STAT_PRG_DONE) && --to_prg) {
				WATCHDOG_RESET();
				udelay(10);
			}
			if (!to_prg) {
#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
				printf("JZ_MMC: R1b PRG_DONE not seen; proceeding to avoid hang on SDIO\n");
#endif
			}
		} else {
			/* Non-SDIO: wait with a generous but bounded loop to avoid hard lock */
			to_prg = 1000000; /* ~1s max at tight loop */
			while (!(jz_mmc_readl(priv, MSC_STAT) & MSC_STAT_PRG_DONE) && --to_prg) {
				WATCHDOG_RESET();
				udelay(10);
			}
			if (!to_prg) {
#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
				printf("JZ_MMC: WARNING: R1b PRG_DONE timeout\n");
#endif
			}
		}
		/* Clear PRG_DONE interrupt if latched */
		jz_mmc_writel(MSC_IREG_PRG_DONE, priv, MSC_IFLG);
	}

#ifndef CONFIG_SPL_BUILD
	if (data && (data->flags & MMC_DATA_WRITE)) {
		/* write the data */
		int sz = DIV_ROUND_UP(data->blocks * data->blocksize, 4);
		const void *buf = data->src;
		while (sz--) {
			uint32_t val = get_unaligned_le32(buf);
			while (!(jz_mmc_readl(priv, MSC_IFLG) & MSC_IREG_TXFIFO_WR_REQ)) {
				WATCHDOG_RESET();
			}
			jz_mmc_writel(val, priv, MSC_TXFIFO);
			buf += 4;
		}
		while (!(jz_mmc_readl(priv, MSC_STAT) & MSC_STAT_PRG_DONE)) {
			WATCHDOG_RESET();
		}
		jz_mmc_writel(MSC_IREG_PRG_DONE, priv, MSC_IFLG);
	} else if (data && (data->flags & MMC_DATA_READ)) {
		/* read the data */
		int sz = data->blocks * data->blocksize;
		void *buf = data->dest;
		do {
			stat = jz_mmc_readl(priv, MSC_STAT);
			if (stat & MSC_STAT_TIME_OUT_READ)
				return TIMEOUT;
			if (stat & MSC_STAT_CRC_READ_ERROR)
				return COMM_ERR;
			if (stat & MSC_STAT_DATA_FIFO_EMPTY) {
				udelay(100);
				continue;
			}
			do {
				uint32_t val = jz_mmc_readl(priv, MSC_RXFIFO);
				if (sz == 1)
					*(uint8_t *)buf = (uint8_t)val;
				else if (sz == 2)
					put_unaligned_le16(val, buf);
				else if (sz >= 4)
					put_unaligned_le32(val, buf);
				buf += 4;
				sz -= 4;
				stat = jz_mmc_readl(priv, MSC_STAT);
			} while (!(stat & MSC_STAT_DATA_FIFO_EMPTY));
		} while (!(stat & MSC_STAT_DATA_TRAN_DONE));
		while (!(jz_mmc_readl(priv, MSC_IFLG) & MSC_IREG_DATA_TRAN_DONE));
		jz_mmc_writel(MSC_IREG_DATA_TRAN_DONE, priv, MSC_IFLG);
	}
#else
	if (data && (data->flags & MMC_DATA_READ)) {
		/* read the data */
		int sz = data->blocks * data->blocksize;
		void *buf = data->dest;
		do {
			stat = jz_mmc_readl(priv, MSC_STAT);
			if (stat & MSC_STAT_TIME_OUT_READ)
				return TIMEOUT;
			if (stat & MSC_STAT_CRC_READ_ERROR)
				return COMM_ERR;
			if (stat & MSC_STAT_DATA_FIFO_EMPTY) {
				udelay(100);
				continue;
			}
			do {
				uint32_t val = jz_mmc_readl(priv, MSC_RXFIFO);
				if (sz == 1)
					*(uint8_t *)buf = (uint8_t)val;
				else if (sz == 2)
					put_unaligned_le16(val, buf);
				else if (sz >= 4)
					put_unaligned_le32(val, buf);
				buf += 4;
				sz -= 4;
				stat = jz_mmc_readl(priv, MSC_STAT);
			} while (!(stat & MSC_STAT_DATA_FIFO_EMPTY));
		} while (!(stat & MSC_STAT_DATA_TRAN_DONE));

		while (!(jz_mmc_readl(priv, MSC_IFLG) & MSC_IREG_DATA_TRAN_DONE));

		jz_mmc_writel(MSC_IREG_DATA_TRAN_DONE, priv, MSC_IFLG);
	}
#endif

	return 0;
}

static void jz_mmc_set_ios(struct mmc *mmc)
{
	struct jz_mmc_priv *priv = mmc->priv;
#ifndef CONFIG_FPGA
	uint32_t real_rate = 0;
	uint32_t lpm = 0;
	uint8_t clk_div = 0;

	if (mmc->clock > 1000000) {
		clk_set_rate(priv->clk, mmc->clock);
	} else {
		clk_set_rate(priv->clk, 24000000);
	}

	real_rate = clk_get_rate(priv->clk);

	/* calculate clock divide */
	while ((real_rate > mmc->clock) && (clk_div < 7)) {
		real_rate >>= 1;
		clk_div++;
	}

	jz_mmc_writel(clk_div, priv, MSC_CLKRT);

	if (real_rate > 25000000)
		lpm |= (0x2 << LPM_DRV_SEL_SHF) | LPM_SMP_SEL;

	jz_mmc_writel(lpm, priv, MSC_LPM);
#else
	if(mmc->clock < 400000) {
		/* 1/64 devclk, ~384KHz, for init */
		jz_mmc_writel(6, priv, MSC_CLKRT);
	} else {
#ifdef CONFIG_JZ_MMC_FORCE_CLKRT_ZERO
		/* Per vendor binary observation: force CLKRT=0 for normal r/w */
		jz_mmc_writel(0, priv, MSC_CLKRT);
#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
		printf("MSC: Forcing CLKRT=0 for normal r/w (per vendor)\n");
#endif
#else
#ifdef CONFIG_JZ_MMC_MSC1
		jz_mmc_writel(3, priv, MSC_CLKRT);
#else
		/* 1/2 devclk, ~12MHz, for data transfer */
		jz_mmc_writel(1, priv, MSC_CLKRT);
#endif
#endif
	}
#endif
	/* set the bus width for the next command */
	priv->flags &= ~JZ_MMC_BUS_WIDTH_MASK;

	if (mmc->bus_width == 8)
		priv->flags |= JZ_MMC_BUS_WIDTH_8;
	else if (mmc->bus_width == 4)
		priv->flags |= JZ_MMC_BUS_WIDTH_4;
	else
		priv->flags |= JZ_MMC_BUS_WIDTH_1;
#ifndef CONFIG_FPGA
	debug("jzmmc:clk_want:%d, clk_set:%d bus_width:%d\n",
	      mmc->clock, clk_get_rate(priv->clk) / (1 << clk_div), mmc->bus_width);
#endif
}

static int jz_mmc_core_init(struct mmc *mmc)
{
	int tmp;
	struct jz_mmc_priv *priv = mmc->priv;
	unsigned int clkrt = jz_mmc_readl(priv, MSC_CLKRT);

	/* reset */
	jz_mmc_writel(MSC_CTRL_RESET, priv, MSC_CTRL);
#if defined(CONFIG_M200) || \
	defined(CONFIG_T10) || defined(CONFIG_T15) || \
	defined(CONFIG_T20) || defined(CONFIG_T21) || defined(CONFIG_T23) || \
	defined(CONFIG_T30) || defined(CONFIG_T31) || defined(CONFIG_C100)
	tmp = jz_mmc_readl(priv, MSC_CTRL);
	tmp &= ~MSC_CTRL_RESET;
	jz_mmc_writel(tmp, priv, MSC_CTRL);
#else
	while (jz_mmc_readl(priv, MSC_STAT) & MSC_STAT_IS_RESETTING);
#endif

	/* Ensure continuous clock: SDIO requires it; enable for all MSC controllers */
	printf("MMC: Controller base=0x%08lx\n", priv->base);
	{
		uint32_t ctrl = jz_mmc_readl(priv, MSC_CTRL);
		ctrl &= ~MSC_CTRL_CLOCK_CONTROL_MASK;
		ctrl |= MSC_CTRL_CLOCK_CONTROL_START;
		jz_mmc_writel(ctrl, priv, MSC_CTRL);
		printf("MSC: Enabled continuous clock (CTRL=0x%08x)\n", ctrl);
	}

	/* maximum timeouts */
//	jz_mmc_writel(0xffffffff, priv, MSC_RESTO); //use the default value for decreasing time by ykliu
	jz_mmc_writel(0xffffffff, priv, MSC_RDTO);

#ifdef CONFIG_JZ_MMC_FORCE_CLKRT_ZERO
	/* Force CLKRT per vendor observation: 6 during init, 0 for normal */
	if (mmc->clock < 400000) {
		jz_mmc_writel(6, priv, MSC_CLKRT);
	#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
		printf("MSC: Forcing CLKRT=6 for init (per vendor)\n");
	#endif
	} else {
		jz_mmc_writel(0, priv, MSC_CLKRT);
	#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SPL_LIBCOMMON_SUPPORT)
		printf("MSC: Forcing CLKRT=0 for normal r/w (per vendor)\n");
	#endif
	}
#else
	jz_mmc_writel(clkrt, priv, MSC_CLKRT);
#endif

	return 0;
}

static void jz_mmc_init_one(int idx, int controller, uintptr_t base, int clock)
{
	struct mmc *mmc = &mmc_dev[idx];
	struct jz_mmc_priv *priv = &mmc_priv[idx];

	/* fill in the name */
	strcpy(mmc->name, "msc");
	mmc->name[10] = '0' + controller;
	mmc->name[11] = 0;

	/* setup priv */
	priv->base = base;
	priv->clk = clock;
	priv->flags = 0;

	/* setup mmc */
	mmc->priv = priv;
	mmc->send_cmd = jz_mmc_send_cmd;
	mmc->set_ios = jz_mmc_set_ios;
#if !defined(CONFIG_SPL_BUILD) || defined(CONFIG_SUPPORT_EMMC_BOOT)
	mmc->init = jz_mmc_core_init;
#else
	mmc->init = NULL;
#endif
	/* For MSC1 (SDIO WiFi), always report card present (non-removable device) */
	if (controller == 1) {
		/* MSC1: Force card detect to always return "present" for ATBM6441 WiFi */
		mmc->getcd = NULL;  /* NULL means "always present" in some MMC code paths */
	} else {
		mmc->getcd = NULL;
	}
	mmc->getwp = NULL;

	mmc->voltages = MMC_VDD_27_28 |
		MMC_VDD_28_29 | MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32 |
		MMC_VDD_32_33 | MMC_VDD_33_34 | MMC_VDD_34_35 | MMC_VDD_35_36;

	mmc->f_min = 200000;
#ifdef CONFIG_SPL_BUILD
	mmc->f_max = 24000000;
#ifdef CONFIG_JZ_MMC_MSC0_PA_8BIT
	mmc->host_caps = MMC_MODE_8BIT | MMC_MODE_HC;
#else
#ifdef CONFIG_JZ_MMC_1BIT
	mmc->host_caps = MMC_MODE_HS | MMC_MODE_HC;
#else
	mmc->host_caps = MMC_MODE_4BIT | MMC_MODE_HC;
#endif
#endif
#else
	mmc->f_max = 52000000;
#ifndef CONFIG_FPGA
#ifdef CONFIG_JZ_MMC_MSC0_PA_8BIT
	mmc->host_caps = MMC_MODE_8BIT | MMC_MODE_HS_52MHz | MMC_MODE_HS | MMC_MODE_HC;
#else
#ifdef CONFIG_JZ_MMC_1BIT
	mmc->host_caps = MMC_MODE_HS_52MHz | MMC_MODE_HS | MMC_MODE_HC;
#else
	mmc->host_caps = MMC_MODE_4BIT | MMC_MODE_HS_52MHz | MMC_MODE_HS | MMC_MODE_HC;
#endif
#endif
#else /* CONFIG_FPGA */
	mmc->host_caps = MMC_MODE_HS_52MHz | MMC_MODE_HS | MMC_MODE_HC;
#endif /* CONFIG_FPGA */
#endif

	mmc_register(mmc);
}

void jz_mmc_init(void)
{
	int i = 0;

#if defined(CONFIG_JZ_MMC_MSC0) && !defined(CONFIG_JZ_MMC_DISABLE_MSC0) && (!defined(CONFIG_SPL_BUILD) || (CONFIG_JZ_MMC_SPLMSC == 0))
	printf("MMC: Registering MSC0 at index %d, base=0x%08x\n", i, MSC0_BASE);
	jz_mmc_init_one(i++, 0, MSC0_BASE, MSC0);
#endif
#if defined(CONFIG_JZ_MMC_MSC1) && !defined(CONFIG_JZ_MMC_DISABLE_MSC1) && (!defined(CONFIG_SPL_BUILD) || (CONFIG_JZ_MMC_SPLMSC == 1))
#ifdef CONFIG_T23_REGISTER_MSC1_AT_MSC0_BASE
	{
		uintptr_t base1 = MSC0_BASE;
		int clk1 = MSC0;
		printf("MMC: Registering MSC1 at index %d, base=0x%08x\n", i, (unsigned int)base1);
		jz_mmc_init_one(i++, 1, base1, clk1);
	}
#else
	printf("MMC: Registering MSC1 at index %d, base=0x%08x\n", i, MSC1_BASE);
	jz_mmc_init_one(i++, 1, MSC1_BASE, MSC1);
#endif
#endif
#if defined(CONFIG_JZ_MMC_MSC2) && (!defined(CONFIG_SPL_BUILD) || (CONFIG_JZ_MMC_SPLMSC == 2))
	printf("MMC: Registering MSC2 at index %d, base=0x%08x\n", i, MSC2_BASE);
	jz_mmc_init_one(i++, 2, MSC2_BASE, MSC2);
#endif
}
