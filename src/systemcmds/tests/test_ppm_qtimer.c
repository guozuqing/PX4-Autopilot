/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file test_ppm_qtimer.c
 * QTIMER PPM status test command
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#ifdef HRT_PPM_QTIMER

/* QTIMER1 registers for PPM (from board_config.h) */
#define QTIMER1_BASE    0x4015c000
#define QTIMER_CH3_BASE (QTIMER1_BASE + (3 * 0x20))

#define REG16(addr) (*(volatile uint16_t *)(addr))

#define QTIMER_COMP1(base)  REG16((base) + 0x00)
#define QTIMER_COMP2(base)  REG16((base) + 0x02)
#define QTIMER_CAPT(base)   REG16((base) + 0x04)
#define QTIMER_LOAD(base)   REG16((base) + 0x06)
#define QTIMER_HOLD(base)   REG16((base) + 0x08)
#define QTIMER_CNTR(base)   REG16((base) + 0x0a)
#define QTIMER_CTRL(base)   REG16((base) + 0x0c)
#define QTIMER_SCTRL(base)  REG16((base) + 0x0e)
#define QTIMER_ENBL(base)   REG16((base) + 0x1e)

/* GPIO and IOMUX registers - addresses from iMXRT1176 reference manual
 * IOMUXC base = 0x400E8000
 * MUX offset for GPIO_EMC_B2_12 = 0x00E8 (index 54)
 * PAD offset for GPIO_EMC_B2_12 = 0x032C (index 54)
 */
#define GPIO_EMC_B2_12_MUX   0x400E80E8  /* IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_B2_12 */
#define GPIO_EMC_B2_12_PAD   0x400E832C  /* IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_B2_12 */
#define GPIO_EMC_B2_12_INPUT 0x400E84E8  /* IOMUXC_QTIMER1_TIMER3_SELECT_INPUT */
#define GPIO2_DR             0x42008000  /* GPIO2 data register */
#define GPIO2_GDIR           0x42008004  /* GPIO2 direction register */
#define GPIO2_PSR            0x42008008  /* GPIO2 pad status register */

#define REG32(addr) (*(volatile uint32_t *)(addr))

extern uint16_t ppm_buffer[];
extern unsigned ppm_decoded_channels;
extern uint64_t ppm_last_valid_decode;

__EXPORT int test_ppm_qtimer_main(int argc, char *argv[]);

int test_ppm_qtimer_main(int argc, char *argv[])
{
	/* Check if user wants to force IOMUX configuration */
	if (argc > 1 && strcmp(argv[1], "fix") == 0) {
		printf("Forcing IOMUX configuration for GPIO_EMC_B2_12...\n");
		/* Set MUX to ALT9 (QTIMER1_TIMER3) with SION=1 */
		REG32(GPIO_EMC_B2_12_MUX) = 0x19;  /* ALT9 + SION */
		/* Configure PAD with pull-up */
		REG32(GPIO_EMC_B2_12_PAD) = 0x10;  /* Pull-up enabled */
		/* Configure SELECT_INPUT */
		REG32(GPIO_EMC_B2_12_INPUT) = 0;   /* Select GPIO_EMC_B2_12 */
		printf("Done! MUX=0x%08lx\n", (unsigned long)REG32(GPIO_EMC_B2_12_MUX));
	}

	printf("\n=== QTIMER1 CH3 PPM Status ===\n");
	printf("Base Address: 0x%08lx\n", (unsigned long)QTIMER_CH3_BASE);
	printf("\nRegisters:\n");
	printf("  COMP1:  0x%04x\n", QTIMER_COMP1(QTIMER_CH3_BASE));
	printf("  COMP2:  0x%04x\n", QTIMER_COMP2(QTIMER_CH3_BASE));

	/* Reading CAPT triggers capture to HOLD */
	uint16_t capt = QTIMER_CAPT(QTIMER_CH3_BASE);
	uint16_t hold = QTIMER_HOLD(QTIMER_CH3_BASE);

	printf("  CAPT:   0x%04x\n", capt);
	printf("  LOAD:   0x%04x\n", QTIMER_LOAD(QTIMER_CH3_BASE));
	printf("  HOLD:   0x%04x (captured value)\n", hold);
	printf("  CNTR:   0x%04x (counter running: %s)\n",
	       QTIMER_CNTR(QTIMER_CH3_BASE),
	       QTIMER_CNTR(QTIMER_CH3_BASE) > 0 ? "YES" : "NO");

	uint16_t ctrl = QTIMER_CTRL(QTIMER_CH3_BASE);
	uint16_t sctrl = QTIMER_SCTRL(QTIMER_CH3_BASE);

	printf("  CTRL:   0x%04x (CM=%d, PCS=%d)\n", ctrl,
	       (ctrl >> 13) & 0x7, (ctrl >> 9) & 0xF);
	printf("  SCTRL:  0x%04x (IEF=%d, IEFIE=%d, CAP=%d)\n", sctrl,
	       (sctrl >> 11) & 1, (sctrl >> 10) & 1, (sctrl >> 6) & 3);
	printf("  ENBL:   0x%04x (CH3 enabled: %s)\n",
	       QTIMER_ENBL(QTIMER1_BASE),
	       (QTIMER_ENBL(QTIMER1_BASE) & (1 << 3)) ? "YES" : "NO");

	printf("\nGPIO & IOMUX Configuration:\n");
	uint32_t mux_reg = REG32(GPIO_EMC_B2_12_MUX);
	uint32_t pad_reg = REG32(GPIO_EMC_B2_12_PAD);
	uint32_t input_reg = REG32(GPIO_EMC_B2_12_INPUT);
	uint32_t gpio_dir = REG32(GPIO2_GDIR);
	uint32_t gpio_psr = REG32(GPIO2_PSR);

	printf("  MUX (0x%08lx): 0x%08lx (ALT=%ld, SION=%ld)\n",
	       (unsigned long)GPIO_EMC_B2_12_MUX, (unsigned long)mux_reg,
	       mux_reg & 0xF, (mux_reg >> 4) & 1);
	printf("  PAD (0x%08lx): 0x%08lx\n",
	       (unsigned long)GPIO_EMC_B2_12_PAD, (unsigned long)pad_reg);
	printf("  SELECT_INPUT (0x%08lx): 0x%08lx\n",
	       (unsigned long)GPIO_EMC_B2_12_INPUT, (unsigned long)input_reg);
	printf("  GPIO2_GDIR.22: %ld (0=input, 1=output)\n", (gpio_dir >> 22) & 1);
	printf("  GPIO2_PSR.22: %ld (pad level)\n", (gpio_psr >> 22) & 1);

	printf("\nInput Signal Status:\n");
	printf("  INPUT bit (SCTRL.8): %d (QTIMER sees this level)\n", (sctrl >> 8) & 1);
	printf("  IEF (edge detected): %d\n", (sctrl >> 11) & 1);

	printf("\nDiagnostic:\n");
	if ((mux_reg & 0xF) != 9) {
		printf("  ERROR: MUX not set to ALT9 (QTIMER)! Current: ALT%ld\n", mux_reg & 0xF);
	} else if (((gpio_dir >> 22) & 1) == 1) {
		printf("  WARNING: GPIO2.22 is OUTPUT mode (should be input)\n");
	} else if ((sctrl >> 11) & 1) {
		printf("  Edge detected - signal is toggling\n");
	} else {
		printf("  No edge detected:\n");
		printf("    - ALT function: %s\n", (mux_reg & 0xF) == 9 ? "OK (ALT9)" : "WRONG");
		printf("    - Pad level: %ld\n", (gpio_psr >> 22) & 1);
		printf("    - QTIMER sees: %d\n", (sctrl >> 8) & 1);
		printf("  Check PPM signal connection and level!\n");
	}

	printf("\nPPM Status:\n");
	printf("  Decoded channels: %u\n", ppm_decoded_channels);
	printf("  Last valid decode: %llu us ago\n",
	       ppm_last_valid_decode > 0 ? (hrt_absolute_time() - ppm_last_valid_decode) : 0);

	if (ppm_decoded_channels > 0) {
		printf("\nChannel values:\n");
		for (unsigned i = 0; i < ppm_decoded_channels && i < 8; i++) {
			printf("  CH%u: %u us\n", i + 1, ppm_buffer[i]);
		}
	}

	printf("\n");
	return 0;
}

#else

int test_ppm_qtimer_main(int argc, char *argv[])
{
	printf("HRT_PPM_QTIMER not defined - QTIMER PPM not enabled\n");
	return 1;
}

#endif
