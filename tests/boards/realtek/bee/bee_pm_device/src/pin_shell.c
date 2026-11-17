/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Shell commands that list the pins of every peripheral and move them to other
 * pads at run time.
 *
 * A peripheral is moved by taking it down with device_deinit(), pointing its
 * pinctrl configuration at an edited RAM copy of its states with
 * pinctrl_update_states() and bringing it up again with device_init(). The
 * copy keeps the number and the ids of the states, which is all that
 * pinctrl_update_states() accepts.
 *
 * A counter applies its pins each time a capture is configured rather than at
 * init, so it is not taken down and the new pad is used from the next capture.
 *
 * A GPIO port has no deinit, so a GPIO line is moved pin by pin: the pin is
 * disconnected, gpio_bee_pin_remap() hands it the new pad and it is configured
 * again.
 */

#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/shell/shell.h>

#ifdef CONFIG_GPIO_BEE
#include <zephyr/drivers/gpio/gpio_bee.h>
#endif

#include "pm_test.h"

/* Pads and function names of each IC */

struct pin_pad {
	const char *name;
	/* What the pad carries out of reset, NULL if nothing */
	const char *use;
	uint8_t pad;
	/* The pad is never handed out */
	bool locked;
};

#define PAD(p)              {#p, NULL, p, false}
#define PAD_USE(p, use)     {#p, use, p, false}
#define PAD_LOCKED(p, use)  {#p, use, p, true}
#define FUN(f)              [BEE_##f] = #f

#if defined(CONFIG_SOC_SERIES_RTL87X2J)

/* The SPIC pads carry the flash the code runs from */
static const struct pin_pad pin_pads[] = {
	PAD(P0_0), PAD(P0_1), PAD(P0_2), PAD_USE(P0_3, "log"), PAD(P0_4), PAD(P0_5), PAD(P0_6),
	PAD(P0_7), PAD_USE(P1_0, "swd"), PAD_USE(P1_1, "swd"), PAD(P1_6), PAD(P1_7), PAD(P2_0),
	PAD(P2_1), PAD(P2_2), PAD(P2_3), PAD(P2_4), PAD(P2_5), PAD(P2_6), PAD(P2_7), PAD(P3_0),
	PAD(P3_1), PAD(P3_2), PAD(P3_3), PAD(P3_4), PAD(P3_5), PAD(P3_6), PAD(P4_0), PAD(P4_1),
	PAD(P4_2), PAD(P4_3), PAD(MICBIAS), PAD(P5_1), PAD(P5_2), PAD(P6_0), PAD(P6_1), PAD(P6_2),
	PAD(P6_3), PAD(P6_4), PAD(P6_5), PAD(P6_6), PAD(P6_7), PAD(P7_0), PAD(P7_1),
	PAD_LOCKED(SPIC_CSN, "flash"), PAD_LOCKED(SPIC_SCK, "flash"), PAD_LOCKED(SPIC_SI, "flash"),
	PAD_LOCKED(SPIC_SO, "flash"), PAD_LOCKED(SPIC_WEN, "flash"),
	PAD_LOCKED(SPIC_HOLDEN, "flash"),
};

#define PIN_FUN_LAST BEE_LPQDEC_PHA

static const char *const fun_names[PIN_FUN_LAST + 1] = {
	FUN(IDLE_MODE),
	FUN(UART0_TX), FUN(UART0_RX), FUN(UART0_CTS), FUN(UART0_RTS),
	FUN(UART1_TX), FUN(UART1_RX), FUN(UART1_CTS), FUN(UART1_RTS),
	FUN(UART2_TX), FUN(UART2_RX), FUN(UART2_CTS), FUN(UART2_RTS),
	FUN(UART3_TX), FUN(UART3_RX), FUN(UART3_CTS), FUN(UART3_RTS),
	FUN(I2C0_CLK), FUN(I2C0_DAT), FUN(I2C1_CLK), FUN(I2C1_DAT),
	FUN(SPI0_SS_N_0_SLAVE), FUN(SPI0_CLK_SLAVE), FUN(SPI0_SO_SLAVE), FUN(SPI0_SI_SLAVE),
	FUN(SPI0_SS_N_0_MASTER), FUN(SPI0_SS_N_1_MASTER), FUN(SPI0_SS_N_2_MASTER),
	FUN(SPI0_CLK_MASTER), FUN(SPI0_MO_MASTER), FUN(SPI0_MI_MASTER),
	FUN(SPI1_SS_N_0_MASTER), FUN(SPI1_SS_N_1_MASTER), FUN(SPI1_SS_N_2_MASTER),
	FUN(SPI1_CLK_MASTER), FUN(SPI1_MO_MASTER), FUN(SPI1_MI_MASTER),
	FUN(SPI3W_DATA_MASTER), FUN(SPI3W_CLK_MASTER), FUN(SPI3W_CS_MASTER),
	FUN(SPI3W_QB_MASTER),
	FUN(PWM0), FUN(PWM1), FUN(PWM2), FUN(PWM3), FUN(PWM4), FUN(PWM5),
	FUN(PWM6_P), FUN(PWM6_N), FUN(PWM7_P), FUN(PWM7_N), FUN(PWM8_P), FUN(PWM8_N),
	FUN(PWM9_P), FUN(PWM9_N),
	FUN(IRDA_TX), FUN(IRDA_RX), FUN(CAN_TX), FUN(CAN_RX), FUN(SWD_CLK), FUN(SWD_DIO),
	FUN(DWGPIO),
	FUN(SPI2_SS_N_0_MASTER), FUN(SPI2_CLK_MASTER), FUN(SPI2_MO_MASTER), FUN(SPI2_MI_MASTER),
	FUN(DIG_DEBUG), FUN(DMIC1_CLK), FUN(DMIC1_DAT),
	FUN(LRC_I_CODEC_SLAVE), FUN(BCLK_I_CODEC_SLAVE), FUN(SDI_CODEC_SLAVE),
	FUN(SDO_CODEC_SLAVE),
	FUN(BT_COEX_I_0), FUN(BT_COEX_I_1), FUN(BT_COEX_I_2), FUN(BT_COEX_I_3),
	FUN(BT_COEX_O_0), FUN(BT_COEX_O_1), FUN(BT_COEX_O_2), FUN(BT_COEX_O_3),
	FUN(PTA_I2C_CLK_SLAVE), FUN(PTA_I2C_DAT_SLAVE), FUN(PTA_I2C_INT_OUT),
	FUN(EN_EXPA), FUN(EN_EXLNA), FUN(TRACECLKIN), FUN(TRACESWO),
	FUN(LRC_SPORT0), FUN(BCLK_SPORT0), FUN(ADCDAT_SPORT0), FUN(DACDAT_SPORT0), FUN(MCLK),
	FUN(KEY_COL_0), FUN(KEY_COL_1), FUN(KEY_COL_2), FUN(KEY_COL_3), FUN(KEY_COL_4),
	FUN(KEY_COL_5), FUN(KEY_COL_6), FUN(KEY_COL_7), FUN(KEY_COL_8), FUN(KEY_COL_9),
	FUN(KEY_COL_10), FUN(KEY_COL_11), FUN(KEY_COL_12), FUN(KEY_COL_13), FUN(KEY_COL_14),
	FUN(KEY_COL_15), FUN(KEY_COL_16), FUN(KEY_COL_17), FUN(KEY_COL_18), FUN(KEY_COL_19),
	FUN(KEY_ROW_0), FUN(KEY_ROW_1), FUN(KEY_ROW_2), FUN(KEY_ROW_3), FUN(KEY_ROW_4),
	FUN(KEY_ROW_5), FUN(KEY_ROW_6), FUN(KEY_ROW_7), FUN(KEY_ROW_8), FUN(KEY_ROW_9),
	FUN(KEY_ROW_10), FUN(KEY_ROW_11), FUN(KEY_ROW_12), FUN(KEY_ROW_13), FUN(KEY_ROW_14),
	FUN(KEY_ROW_15), FUN(KEY_ROW_16), FUN(KEY_ROW_17),
	FUN(SW_MODE), FUN(PWR_OFF), FUN(LPPWM_CH0), FUN(LPQDEC_LED), FUN(LPQDEC_PHB),
	FUN(LPQDEC_PHA),
};

#elif defined(CONFIG_SOC_SERIES_RTL87X2G)

/* Pads 61 to 63 and 69 to 71 do not exist */
static const struct pin_pad pin_pads[] = {
	PAD(P0_0), PAD(P0_1), PAD(P0_2), PAD_USE(P0_3, "log"), PAD(P0_4), PAD(P0_5), PAD(P0_6),
	PAD(P0_7), PAD_USE(P1_0, "swd"), PAD_USE(P1_1, "swd"), PAD(P1_2), PAD(P1_3), PAD(P1_4),
	PAD(P1_5), PAD(P1_6), PAD(P1_7), PAD(P2_0), PAD(P2_1), PAD(P2_2), PAD(P2_3), PAD(P2_4),
	PAD(P2_5), PAD(P2_6), PAD(P2_7), PAD(P3_0), PAD(P3_1), PAD(P3_2), PAD(P3_3), PAD(P3_4),
	PAD(P3_5), PAD(P3_6), PAD(P3_7), PAD(P4_0), PAD(P4_1), PAD(P4_2), PAD(P4_3), PAD(P4_4),
	PAD(P4_5), PAD(P4_6), PAD(P4_7), PAD(P5_0), PAD(P5_1), PAD(P5_2), PAD(P5_3), PAD(P5_4),
	PAD(P5_5), PAD(P5_6), PAD(P5_7), PAD(P6_0), PAD(P6_1), PAD(P6_2), PAD(P6_3), PAD(P6_4),
	PAD(P6_5), PAD(P6_6), PAD(P6_7), PAD(P7_0), PAD(P7_1), PAD(P7_2), PAD(P7_3), PAD(P7_4),
	PAD(MICBIAS), PAD_USE(XI32K, "32k xtal"), PAD_USE(XO32K, "32k xtal"), PAD(DACP), PAD(DACN),
	PAD(P9_0), PAD(P9_1), PAD(P9_2), PAD(P9_3), PAD(P9_4), PAD(P9_5), PAD(P9_6), PAD(P9_7),
	PAD(P10_0), PAD(P10_1), PAD(P10_2),
};

#define PIN_FUN_LAST BEE_SDHC1_D7_P4_7

static const char *const fun_names[PIN_FUN_LAST + 1] = {
	FUN(IDLE_MODE),
	FUN(UART0_TX), FUN(UART0_RX), FUN(UART0_CTS), FUN(UART0_RTS),
	FUN(UART1_TX), FUN(UART1_RX), FUN(UART1_CTS), FUN(UART1_RTS),
	FUN(UART2_TX), FUN(UART2_RX), FUN(UART2_CTS), FUN(UART2_RTS),
	FUN(UART3_TX), FUN(UART3_RX), FUN(UART3_CTS), FUN(UART3_RTS),
	FUN(UART4_TX), FUN(UART4_RX), FUN(UART4_CTS), FUN(UART4_RTS),
	FUN(UART5_TX), FUN(UART5_RX), FUN(UART5_CTS), FUN(UART5_RTS),
	FUN(I2C0_CLK), FUN(I2C0_DAT), FUN(I2C1_CLK), FUN(I2C1_DAT), FUN(I2C2_CLK), FUN(I2C2_DAT),
	FUN(I2C3_CLK), FUN(I2C3_DAT),
	FUN(SPI0_CLK_MASTER), FUN(SPI0_MO_MASTER), FUN(SPI0_MI_MASTER), FUN(SPI0_CSN_0_MASTER),
	FUN(SPI0_CSN_1_MASTER), FUN(SPI0_CSN_2_MASTER), FUN(SPI0_CSN_0_SLAVE), FUN(SPI0_CLK_SLAVE),
	FUN(SPI0_SO_SLAVE), FUN(SPI0_SI_SLAVE),
	FUN(SPI1_CLK_MASTER), FUN(SPI1_MO_MASTER), FUN(SPI1_MI_MASTER), FUN(SPI1_CSN_0_MASTER),
	FUN(SPI1_CSN_1_MASTER), FUN(SPI1_CSN_2_MASTER),
	FUN(SPI2W_DATA), FUN(SPI2W_CLK), FUN(SPI2W_CS),
	FUN(ENPWM0_P), FUN(ENPWM0_N), FUN(ENPWM1_P), FUN(ENPWM1_N),
	FUN(ENPWM2_P), FUN(ENPWM2_N), FUN(ENPWM3_P), FUN(ENPWM3_N),
	FUN(TIMER_PWM2), FUN(TIMER_PWM3),
	FUN(ISO7816_RST), FUN(ISO7816_CLK), FUN(ISO7816_IO), FUN(ISO7816_VCC_EN),
	FUN(DWGPIO),
	FUN(IRDA_TX), FUN(IRDA_RX),
	FUN(TIMER_PWM4), FUN(TIMER_PWM5), FUN(TIMER_PWM6), FUN(TIMER_PWM7), FUN(TIMER_PWM2_P),
	FUN(TIMER_PWM2_N), FUN(TIMER_PWM3_P), FUN(TIMER_PWM3_N),
	FUN(KEY_COL_0), FUN(KEY_COL_1), FUN(KEY_COL_2), FUN(KEY_COL_3), FUN(KEY_COL_4),
	FUN(KEY_COL_5), FUN(KEY_COL_6), FUN(KEY_COL_7), FUN(KEY_COL_8), FUN(KEY_COL_9),
	FUN(KEY_COL_10), FUN(KEY_COL_11), FUN(KEY_COL_12), FUN(KEY_COL_13), FUN(KEY_COL_14),
	FUN(KEY_COL_15), FUN(KEY_COL_16), FUN(KEY_COL_17), FUN(KEY_COL_18), FUN(KEY_COL_19),
	FUN(KEY_ROW_0), FUN(KEY_ROW_1), FUN(KEY_ROW_2), FUN(KEY_ROW_3), FUN(KEY_ROW_4),
	FUN(KEY_ROW_5), FUN(KEY_ROW_6), FUN(KEY_ROW_7), FUN(KEY_ROW_8), FUN(KEY_ROW_9),
	FUN(KEY_ROW_10), FUN(KEY_ROW_11),
	FUN(KM4_CLK_DIV_4),
	FUN(CARD_DETECT_N_0), FUN(BIU_VOLT_REG_0), FUN(BACK_END_POWER_0), FUN(CARD_INT_N_SDHC_0),
	FUN(A2C_TX), FUN(A2C_RX),
	FUN(LRC_SPORT1), FUN(BCLK_SPORT1), FUN(ADCDAT_SPORT1), FUN(DACDAT_SPORT1),
	FUN(DMIC1_CLK), FUN(DMIC1_DAT),
	FUN(LRC_I_CODEC_SLAVE), FUN(BCLK_I_CODEC_SLAVE), FUN(SDI_CODEC_SLAVE), FUN(SDO_CODEC_SLAVE),
	FUN(LRC_SPORT0), FUN(BCLK_SPORT0), FUN(ADCDAT_SPORT0), FUN(DACDAT_SPORT0),
	FUN(MCLK_OUT), FUN(MCLK_IN),
	FUN(LRC_RX_CODEC_SLAVE), FUN(LRC_RX_SPORT0), FUN(LRC_RX_SPORT1),
	FUN(PDM_DATA), FUN(PDM_CLK),
	FUN(I2S1_LRC_TX_SLAVE), FUN(I2S1_BCLK_SLAVE), FUN(I2S1_SDI_SLAVE), FUN(I2S1_SDO_SLAVE),
	FUN(I2S1_LRC_RX_SLAVE),
	FUN(BT_COEX_I_0), FUN(BT_COEX_I_1), FUN(BT_COEX_I_2), FUN(BT_COEX_I_3), FUN(BT_COEX_O_0),
	FUN(BT_COEX_O_1), FUN(BT_COEX_O_2), FUN(BT_COEX_O_3),
	FUN(PTA_I2C_CLK_SLAVE), FUN(PTA_I2C_DAT_SLAVE), FUN(PTA_I2C_INT_OUT),
	FUN(EN_EXPA), FUN(EN_EXLNA),
	FUN(SEL_TPM_SW), FUN(SEL_TPM_N_SW),
	FUN(ANT_SW0), FUN(ANT_SW1), FUN(ANT_SW2), FUN(ANT_SW3), FUN(ANT_SW4), FUN(ANT_SW5),
	FUN(PHY_GPIO_1), FUN(PHY_GPIO_2),
	FUN(SLOW_DEBUG_MUX_1), FUN(SLOW_DEBUG_MUX_2),
	FUN(TEST_MODE),
	FUN(SWD_CLK), FUN(SWD_DIO), FUN(DIG_DEBUG),
	FUN(SW_MODE), FUN(PWR_OFF),
	FUN(SDHC0_CLK_P9_4), FUN(SDHC0_CMD_P9_3), FUN(SDHC0_D0_P10_0), FUN(SDHC0_D1_P9_7),
	FUN(SDHC0_D2_P9_6), FUN(SDHC0_D3_P9_5), FUN(SDHC0_D4_P4_4), FUN(SDHC0_D5_P4_5),
	FUN(SDHC0_D6_P4_6), FUN(SDHC0_D7_P4_7),
	FUN(SDHC1_CLK_P9_4), FUN(SDHC1_CMD_P9_3), FUN(SDHC1_D0_P10_0), FUN(SDHC1_D1_P9_7),
	FUN(SDHC1_D2_P9_6), FUN(SDHC1_D3_P9_5), FUN(SDHC1_D4_P4_4), FUN(SDHC1_D5_P4_5),
	FUN(SDHC1_D6_P4_6), FUN(SDHC1_D7_P4_7),
};

/* The AON functions sit far above the others, so they get a table of their own */
#define FUN_AON(f) [BEE_##f - BEE_QDPH0_IN_NONE] = #f

static const char *const fun_aon_names[] = {
	FUN_AON(QDPH0_IN_NONE), FUN_AON(QDPH0_IN_P1_3_P1_4), FUN_AON(QDPH0_IN_P5_6_P5_7),
	FUN_AON(QDPH0_IN_P9_0_P9_1),
};

#elif defined(CONFIG_SOC_SERIES_RTL8752H)

/* Pads 10, 13 and 31 do not exist */
static const struct pin_pad pin_pads[] = {
	PAD(P0_0), PAD(P0_1), PAD(P0_2), PAD_USE(P0_3, "log"), PAD(P0_4), PAD(P0_5), PAD(P0_6),
	PAD(P0_7), PAD_USE(P1_0, "swd"), PAD_USE(P1_1, "swd"), PAD(P1_3), PAD(P1_4), PAD(P1_6),
	PAD(P1_7), PAD(P2_0), PAD(P2_1), PAD(P2_2), PAD(P2_3), PAD(P2_4), PAD(P2_5), PAD(P2_6),
	PAD(P2_7), PAD(P3_0), PAD(P3_1), PAD(P3_2), PAD(P3_3), PAD(P3_4), PAD(P3_5), PAD(P3_6),
	PAD(P4_0), PAD(P4_1), PAD(P4_2), PAD(P4_3), PAD(H_0), PAD(P5_1), PAD(P5_2),
};

#define PIN_FUN_LAST BEE_PWR_OFF

static const char *const fun_names[PIN_FUN_LAST + 1] = {
	FUN(IDLE_MODE),
	FUN(UART2_TX), FUN(UART2_RX), FUN(UART2_CTS), FUN(UART2_RTS),
	FUN(I2C0_CLK), FUN(I2C0_DAT), FUN(I2C1_CLK), FUN(I2C1_DAT),
	FUN(PWM2_P), FUN(PWM2_N),
	FUN(ENPWM0_P), FUN(ENPWM0_N),
	FUN(TIM_PWM0), FUN(TIM_PWM1), FUN(TIM_PWM2), FUN(TIM_PWM3), FUN(TIM_PWM4), FUN(TIM_PWM5),
	FUN(ENPWM0), FUN(ENPWM1),
	FUN(QDEC_PHASE_A_X), FUN(QDEC_PHASE_B_X), FUN(QDEC_PHASE_A_Y), FUN(QDEC_PHASE_B_Y),
	FUN(QDEC_PHASE_A_Z), FUN(QDEC_PHASE_B_Z),
	FUN(UART0_TX), FUN(UART0_RX), FUN(UART0_CTS), FUN(UART0_RTS),
	FUN(IRDA_TX), FUN(IRDA_RX),
	FUN(UART1_TX), FUN(UART1_RX), FUN(UART1_CTS), FUN(UART1_RTS),
	FUN(SPI1_SS_N_0_MASTER), FUN(SPI1_SS_N_1_MASTER), FUN(SPI1_SS_N_2_MASTER),
	FUN(SPI1_CLK_MASTER), FUN(SPI1_MO_MASTER), FUN(SPI1_MI_MASTER),
	FUN(SPI0_SS_N_0_SLAVE), FUN(SPI0_CLK_SLAVE), FUN(SPI0_SO_SLAVE), FUN(SPI0_SI_SLAVE),
	FUN(SPI0_SS_N_0_MASTER), FUN(SPI0_CLK_MASTER), FUN(SPI0_MO_MASTER), FUN(SPI0_MI_MASTER),
	FUN(SPI2W_DATA), FUN(SPI2W_CLK), FUN(SPI2W_CS),
	FUN(SWD_CLK), FUN(SWD_DIO),
	FUN(KEY_COL_0), FUN(KEY_COL_1), FUN(KEY_COL_2), FUN(KEY_COL_3), FUN(KEY_COL_4),
	FUN(KEY_COL_5), FUN(KEY_COL_6), FUN(KEY_COL_7), FUN(KEY_COL_8), FUN(KEY_COL_9),
	FUN(KEY_COL_10), FUN(KEY_COL_11), FUN(KEY_COL_12), FUN(KEY_COL_13), FUN(KEY_COL_14),
	FUN(KEY_COL_15), FUN(KEY_COL_16), FUN(KEY_COL_17), FUN(KEY_COL_18), FUN(KEY_COL_19),
	FUN(KEY_ROW_0), FUN(KEY_ROW_1), FUN(KEY_ROW_2), FUN(KEY_ROW_3), FUN(KEY_ROW_4),
	FUN(KEY_ROW_5), FUN(KEY_ROW_6), FUN(KEY_ROW_7), FUN(KEY_ROW_8), FUN(KEY_ROW_9),
	FUN(KEY_ROW_10), FUN(KEY_ROW_11),
	FUN(DWGPIO),
	FUN(DMIC1_CLK), FUN(DMIC1_DAT),
	FUN(LRC_I_CODEC_SLAVE), FUN(BCLK_I_CODEC_SLAVE), FUN(SDI_CODEC_SLAVE), FUN(SDO_CODEC_SLAVE),
	FUN(BT_COEX_I_0), FUN(BT_COEX_I_1), FUN(BT_COEX_I_2), FUN(BT_COEX_I_3), FUN(BT_COEX_O_0),
	FUN(BT_COEX_O_1), FUN(BT_COEX_O_2), FUN(BT_COEX_O_3),
	FUN(PTA_I2C_CLK_SLAVE), FUN(PTA_I2C_DAT_SLAVE), FUN(PTA_I2C_INT_OUT),
	FUN(EN_EXPA), FUN(EN_EXLNA),
	FUN(LRC_SPORT0), FUN(BCLK_SPORT0), FUN(ADCDAT_SPORT0), FUN(DACDAT_SPORT0),
	FUN(MCLK),
	FUN(SW_MODE), FUN(PWR_OFF),
};

#else
#error "Unsupported SoC series"
#endif

static const char *pin_fun_name(uint16_t fun, char *buf, size_t len)
{
	if (fun < ARRAY_SIZE(fun_names) && fun_names[fun] != NULL) {
		return fun_names[fun];
	}

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
	if (fun >= BEE_QDPH0_IN_NONE && fun - BEE_QDPH0_IN_NONE < ARRAY_SIZE(fun_aon_names)) {
		return fun_aon_names[fun - BEE_QDPH0_IN_NONE];
	}
#endif

	snprintf(buf, len, "fun%u", fun);

	return buf;
}

/* A pad that is to take a pin from now on is a target. A target that carries
 * something out of reset is only warned about, since a board may well use it.
 */
static int pin_parse_pad(const struct shell *sh, const char *arg, bool target, uint8_t *pad)
{
	const struct pin_pad *pp = NULL;
	unsigned long val;
	char *end;

	val = strtoul(arg, &end, 0);
	if (end == arg || *end != '\0') {
		val = ULONG_MAX;
	}

	for (size_t i = 0; i < ARRAY_SIZE(pin_pads); i++) {
		if (strcmp(arg, pin_pads[i].name) == 0 || val == pin_pads[i].pad) {
			pp = &pin_pads[i];
			break;
		}
	}

	if (pp == NULL) {
		shell_error(sh, "no pad %s", arg);
		return -EINVAL;
	}

	if (pp->locked) {
		shell_error(sh, "%s carries the %s", pp->name, pp->use);
		return -EACCES;
	}

	if (target && pp->use != NULL) {
		shell_warn(sh, "%s carries the %s out of reset", pp->name, pp->use);
	}

	*pad = pp->pad;

	return 0;
}

/* Peripherals with pin control */

struct pin_dev {
	const struct device *dev;
	const char *name;
	struct pinctrl_dev_config *pcfg;
	/* The driver applies the pins when it is used rather than at init */
	bool applied_on_use;
	/* RAM copy of the states, made on the first move */
	struct pinctrl_state *states;
	pinctrl_soc_pin_t *pins;
	uint8_t pin_cnt;
};

#define PIN_DEV_LABEL(label) #label,

#define PIN_DEV_NAME(node_id)                                                                      \
	COND_CODE_0(DT_NUM_NODELABELS(node_id), (DT_NODE_FULL_NAME(node_id)),                      \
		    (GET_ARG_N(1, DT_FOREACH_NODELABEL(node_id, PIN_DEV_LABEL))))

#define PIN_DEV_DECLARE(node_id) PINCTRL_DT_DEV_CONFIG_DECLARE(node_id);

#define PIN_DEV_ENTRY(node_id)                                                                     \
	{                                                                                          \
		.dev = DEVICE_DT_GET(node_id),                                                     \
		.name = PIN_DEV_NAME(node_id),                                                     \
		.pcfg = PINCTRL_DT_DEV_CONFIG_GET(node_id),                                        \
		IF_ENABLED(DT_NODE_HAS_COMPAT(node_id, realtek_bee_counter_timer),                 \
			   (.applied_on_use = true,))                                              \
	},

/* The counter only defines a pin control configuration for the instances that
 * have one
 */
#define PIN_DEV_IF_PINCTRL(node_id, fn) IF_ENABLED(DT_PINCTRL_HAS_IDX(node_id, 0), (fn(node_id)))

/* Every Bee driver that defines a pin control configuration for its instances */
#define PIN_DEV_FOREACH(fn)                                                                        \
	IF_ENABLED(CONFIG_COUNTER_CAPTURE,                                                         \
		   (DT_FOREACH_STATUS_OKAY_VARGS(realtek_bee_counter_timer, PIN_DEV_IF_PINCTRL, fn))) \
	IF_ENABLED(CONFIG_UART_BEE, (DT_FOREACH_STATUS_OKAY(realtek_bee_uart, fn)))                \
	IF_ENABLED(CONFIG_SPI_BEE, (DT_FOREACH_STATUS_OKAY(realtek_bee_spi, fn)))                  \
	IF_ENABLED(CONFIG_I2C_BEE, (DT_FOREACH_STATUS_OKAY(realtek_bee_i2c, fn)))                  \
	IF_ENABLED(CONFIG_PWM_BEE, (DT_FOREACH_STATUS_OKAY(realtek_bee_pwm, fn)))                  \
	IF_ENABLED(CONFIG_PWM_BEE_LPPWM, (DT_FOREACH_STATUS_OKAY(realtek_bee_lppwm, fn)))          \
	IF_ENABLED(CONFIG_QDEC_BEE, (DT_FOREACH_STATUS_OKAY(realtek_bee_basic_qdec, fn)            \
				     DT_FOREACH_STATUS_OKAY(realtek_bee_aon_qdec, fn)              \
				     DT_FOREACH_STATUS_OKAY(realtek_bee_lpqdec, fn)))              \
	IF_ENABLED(CONFIG_INPUT_BEE_KEYSCAN, (DT_FOREACH_STATUS_OKAY(realtek_bee_keyscan, fn)))    \
	IF_ENABLED(CONFIG_ADC_BEE, (DT_FOREACH_STATUS_OKAY(realtek_bee_adc, fn)))                  \
	IF_ENABLED(CONFIG_CAN_REALTEK_BEE, (DT_FOREACH_STATUS_OKAY(realtek_bee_can, fn)))          \
	IF_ENABLED(CONFIG_IR_BEE, (DT_FOREACH_STATUS_OKAY(realtek_bee_ir, fn)))                    \
	IF_ENABLED(CONFIG_I2S_BEE, (DT_FOREACH_STATUS_OKAY(realtek_bee_i2s, fn)))                  \
	IF_ENABLED(CONFIG_AUDIO_CODEC_BEE, (DT_FOREACH_STATUS_OKAY(realtek_bee_codec, fn)))

PIN_DEV_FOREACH(PIN_DEV_DECLARE)

static struct pin_dev pin_devs[] = {PIN_DEV_FOREACH(PIN_DEV_ENTRY)};

/* Room for the RAM copies of the states */
#define PIN_POOL_STATES 16
#define PIN_POOL_PINS   64

static struct pin_dev *pin_dev_find(const char *name)
{
	for (size_t i = 0; i < ARRAY_SIZE(pin_devs); i++) {
		if (strcmp(name, pin_devs[i].name) == 0 || strcmp(name, pin_devs[i].dev->name) == 0) {
			return &pin_devs[i];
		}
	}

	return NULL;
}

static bool pin_dev_is_console(const struct device *dev)
{
#if DT_HAS_CHOSEN(zephyr_console)
	if (dev == DEVICE_DT_GET(DT_CHOSEN(zephyr_console))) {
		return true;
	}
#endif
#if DT_HAS_CHOSEN(zephyr_shell_uart)
	if (dev == DEVICE_DT_GET(DT_CHOSEN(zephyr_shell_uart))) {
		return true;
	}
#endif

	return false;
}

static bool pin_dev_uses_pad(const struct pin_dev *pd, uint8_t pad)
{
	const struct pinctrl_dev_config *pcfg = pd->pcfg;

	for (uint8_t i = 0; i < pcfg->state_cnt; i++) {
		for (uint8_t j = 0; j < pcfg->states[i].pin_cnt; j++) {
			if (pcfg->states[i].pins[j].pin == pad) {
				return true;
			}
		}
	}

	return false;
}

static int pin_dev_copy_states(struct pin_dev *pd)
{
	static struct pinctrl_state state_pool[PIN_POOL_STATES];
	static pinctrl_soc_pin_t pin_pool[PIN_POOL_PINS];
	static size_t states_used;
	static size_t pins_used;
	const struct pinctrl_dev_config *pcfg = pd->pcfg;
	size_t cnt = 0;

	if (pd->states != NULL) {
		return 0;
	}

	for (uint8_t i = 0; i < pcfg->state_cnt; i++) {
		cnt += pcfg->states[i].pin_cnt;
	}

	if (states_used + pcfg->state_cnt > ARRAY_SIZE(state_pool) ||
	    pins_used + cnt > ARRAY_SIZE(pin_pool)) {
		return -ENOMEM;
	}

	pd->states = &state_pool[states_used];
	pd->pins = &pin_pool[pins_used];
	pd->pin_cnt = cnt;
	states_used += pcfg->state_cnt;
	pins_used += cnt;

	cnt = 0;
	for (uint8_t i = 0; i < pcfg->state_cnt; i++) {
		pd->states[i] = pcfg->states[i];
		memcpy(&pd->pins[cnt], pcfg->states[i].pins,
		       pcfg->states[i].pin_cnt * sizeof(pinctrl_soc_pin_t));
		pd->states[i].pins = &pd->pins[cnt];
		cnt += pcfg->states[i].pin_cnt;
	}

	return 0;
}

static const char *pin_dev_status(const struct device *dev)
{
	if (!dev->state->initialized) {
		return "deinit";
	}

	return device_is_ready(dev) ? "ready" : "init failed";
}

/* Test GPIO lines */

#ifdef CONFIG_GPIO_BEE

struct pin_gpio_line {
	const char *name;
	struct gpio_dt_spec *spec;
	/* Configuration the line idles in, 0 to leave it disconnected */
	gpio_flags_t idle_flags;
	/* Pad that carries the line, GPIO_BEE_PAD_NONE if not looked up yet */
	uint8_t pad;
	/* Pad the default state of the port gives the line at boot */
	uint8_t boot_pad;
};

static struct pin_gpio_line pin_gpio_lines[] = {
	{"out", &gpio_test_out, GPIO_OUTPUT_HIGH, GPIO_BEE_PAD_NONE, GPIO_BEE_PAD_NONE},
	{"in", &gpio_test_in, 0, GPIO_BEE_PAD_NONE, GPIO_BEE_PAD_NONE},
#if defined(CONFIG_SENSOR) && defined(CONFIG_QDEC_BEE)
	{"phase_a", &phase_a, GPIO_OUTPUT, GPIO_BEE_PAD_NONE, GPIO_BEE_PAD_NONE},
	{"phase_b", &phase_b, GPIO_OUTPUT, GPIO_BEE_PAD_NONE, GPIO_BEE_PAD_NONE},
#endif
};

struct pin_gpio_port {
	const struct device *dev;
	const char *name;
	const struct pinctrl_dev_config *pcfg;
};

DT_FOREACH_STATUS_OKAY(realtek_bee_gpio, PIN_DEV_DECLARE)

static const struct pin_gpio_port pin_gpio_ports[] = {
	DT_FOREACH_STATUS_OKAY(realtek_bee_gpio, PIN_DEV_ENTRY)};

static const char *pin_gpio_port_name(const struct device *port)
{
	for (size_t i = 0; i < ARRAY_SIZE(pin_gpio_ports); i++) {
		if (pin_gpio_ports[i].dev == port) {
			return pin_gpio_ports[i].name;
		}
	}

	return port->name;
}

/* The driver gives no pad for a pin, so start from the pad the default state of
 * the port picks for it, the same way the driver does at init, and follow the
 * moves from there.
 */
static void pin_gpio_lines_init(void)
{
	static bool done;

	if (done) {
		return;
	}

	done = true;

	for (size_t i = 0; i < ARRAY_SIZE(pin_gpio_lines); i++) {
		struct pin_gpio_line *line = &pin_gpio_lines[i];

		for (size_t j = 0; j < ARRAY_SIZE(pin_gpio_ports); j++) {
			const struct pinctrl_state *state;

			if (pin_gpio_ports[j].dev != line->spec->port ||
			    pinctrl_lookup_state(pin_gpio_ports[j].pcfg, PINCTRL_STATE_DEFAULT,
						 &state) < 0) {
				continue;
			}

			for (uint8_t k = 0; k < state->pin_cnt; k++) {
				const struct device *port;
				gpio_pin_t pin;

				if (state->pins[k].sleep_hardware_state ||
				    gpio_bee_pad_to_pin(state->pins[k].pin, &port, &pin) < 0) {
					continue;
				}

				if (port == line->spec->port && pin == line->spec->pin) {
					line->pad = state->pins[k].pin;
					line->boot_pad = line->pad;
					break;
				}
			}
		}
	}
}

static struct pin_gpio_line *pin_gpio_line_find(const char *name)
{
	for (size_t i = 0; i < ARRAY_SIZE(pin_gpio_lines); i++) {
		if (strcmp(name, pin_gpio_lines[i].name) == 0) {
			return &pin_gpio_lines[i];
		}
	}

	return NULL;
}

/* The other pads in the default state of a port are pins the sample does not
 * track, which still belong to the port. The boot pad of a test line is left to
 * the line, so it is free once the line moves away.
 */
static const struct pin_gpio_port *pin_gpio_port_holding(uint8_t pad)
{
	for (size_t i = 0; i < ARRAY_SIZE(pin_gpio_lines); i++) {
		if (pin_gpio_lines[i].boot_pad == pad) {
			return NULL;
		}
	}

	for (size_t i = 0; i < ARRAY_SIZE(pin_gpio_ports); i++) {
		const struct pinctrl_state *state;

		if (pinctrl_lookup_state(pin_gpio_ports[i].pcfg, PINCTRL_STATE_DEFAULT, &state) <
		    0) {
			continue;
		}

		for (uint8_t j = 0; j < state->pin_cnt; j++) {
			if (!state->pins[j].sleep_hardware_state && state->pins[j].pin == pad) {
				return &pin_gpio_ports[i];
			}
		}
	}

	return NULL;
}

#endif /* CONFIG_GPIO_BEE */

/* Find the peripheral that holds a pad, apart from @p self */
static const char *pin_dev_owner(uint8_t pad, const void *self)
{
	for (size_t i = 0; i < ARRAY_SIZE(pin_devs); i++) {
		if (&pin_devs[i] != self && pin_dev_uses_pad(&pin_devs[i], pad)) {
			return pin_devs[i].name;
		}
	}

	return NULL;
}

/* Find what holds a pad, apart from the pins of @p self */
static const char *pin_pad_owner(uint8_t pad, const void *self)
{
	const char *owner = pin_dev_owner(pad, self);

	if (owner != NULL) {
		return owner;
	}

#ifdef CONFIG_GPIO_BEE
	for (size_t i = 0; i < ARRAY_SIZE(pin_gpio_lines); i++) {
		if (&pin_gpio_lines[i] != self && pin_gpio_lines[i].pad == pad) {
			return pin_gpio_lines[i].name;
		}
	}

	const struct pin_gpio_port *gp = pin_gpio_port_holding(pad);

	if (gp != NULL) {
		return gp->name;
	}
#endif

	return NULL;
}

/* pin list */

#define PIN_LIST_FMT "%-9s %-26s %-30s %s"

static const char *pin_state_name(uint8_t id, char *buf, size_t len)
{
	switch (id) {
	case PINCTRL_STATE_DEFAULT:
		return "default";
	case PINCTRL_STATE_SLEEP:
		return "sleep";
	default:
		snprintf(buf, len, "state%u", id);
		return buf;
	}
}

static void pin_list_row(const struct shell *sh, const struct pin_pad *pp, bool *first,
			 const char *fun, const char *cfg, const char *dev)
{
	shell_print(sh, PIN_LIST_FMT, *first ? pp->name : "", fun, cfg, dev);
	*first = false;
}

static void pin_list_cfg(const pinctrl_soc_pin_t *p, char *buf, size_t len)
{
	const char *pull = p->pull_dis ? "no-pull" : p->pull_dir ? "pull-up" : "pull-down";

	if (p->sleep_hardware_state) {
		snprintf(buf, len, "%s", pull);
		return;
	}

	snprintf(buf, len, "%s %s%s drv%u", p->dir ? (p->drive ? "out-high" : "out-low") : "in",
		 pull, (!p->pull_dis && p->pull_strength) ? "-strong" : "", p->current_level);
}

static void pin_list_dev(const struct device *dev, const char *name, bool moved, char *buf,
			 size_t len)
{
	snprintf(buf, len, "%s %s%s%s", name, pin_dev_status(dev), moved ? " moved" : "",
		 pin_dev_is_console(dev) ? " console" : "");
}

/* Print the rows of a pad: every peripheral state that holds it, the GPIO that
 * holds it, or a single row with what the pad carries out of reset, idle if
 * nothing. @p only limits the rows to one peripheral.
 */
static void pin_list_pad(const struct shell *sh, const struct pin_pad *pp,
			 const struct pin_dev *only)
{
	uint8_t pad = pp->pad;
	char fun[32];
	char cfg[32];
	char dev[32];
	char buf[12];
	bool first = true;

	for (size_t i = 0; i < ARRAY_SIZE(pin_devs); i++) {
		const struct pin_dev *pd = &pin_devs[i];
		const struct pinctrl_dev_config *pcfg = pd->pcfg;

		if (only != NULL && pd != only) {
			continue;
		}

		for (uint8_t j = 0; j < pcfg->state_cnt; j++) {
			const struct pinctrl_state *state = &pcfg->states[j];

			for (uint8_t k = 0; k < state->pin_cnt; k++) {
				const pinctrl_soc_pin_t *p = &state->pins[k];
				int n;

				if (p->pin != pad) {
					continue;
				}

				n = snprintf(fun, sizeof(fun), "%s",
					     p->sleep_hardware_state
						     ? "dlps"
						     : pin_fun_name(p->fun, buf, sizeof(buf)));
				if (state->id != PINCTRL_STATE_DEFAULT) {
					snprintf(&fun[n], sizeof(fun) - n, " [%s]",
						 pin_state_name(state->id, buf, sizeof(buf)));
				}

				pin_list_cfg(p, cfg, sizeof(cfg));
				pin_list_dev(pd->dev, pd->name, pcfg->states == pd->states, dev,
					     sizeof(dev));
				pin_list_row(sh, pp, &first, fun, cfg, dev);
			}
		}
	}

	if (only != NULL) {
		return;
	}

#ifdef CONFIG_GPIO_BEE
	const struct pin_gpio_port *gp = pin_gpio_port_holding(pad);
	const struct device *port;
	gpio_pin_t pin;

	for (size_t i = 0; i < ARRAY_SIZE(pin_gpio_lines); i++) {
		const struct pin_gpio_line *line = &pin_gpio_lines[i];
		const char *port_name = pin_gpio_port_name(line->spec->port);

		if (line->pad != pad) {
			continue;
		}

		snprintf(fun, sizeof(fun), "GPIO %s %s.%u", line->name, port_name,
			 line->spec->pin);
		pin_list_dev(line->spec->port, port_name, line->pad != line->boot_pad, dev,
			     sizeof(dev));
		pin_list_row(sh, pp, &first, fun, "set by gpio api", dev);
	}

	if (gp != NULL && gpio_bee_pad_to_pin(pad, &port, &pin) == 0) {
		snprintf(fun, sizeof(fun), "GPIO %s.%u", gp->name, pin);
		pin_list_dev(gp->dev, gp->name, false, dev, sizeof(dev));
		pin_list_row(sh, pp, &first, fun, "set by gpio api", dev);
	}
#endif

	if (!first) {
		return;
	}

	if (pp->locked) {
		pin_list_row(sh, pp, &first, pp->use, "-", "-");
		return;
	}

	snprintf(fun, sizeof(fun), "%s", pp->use != NULL ? pp->use : "idle");
#ifdef CONFIG_GPIO_BEE
	if (gpio_bee_pad_to_pin(pad, &port, &pin) == 0) {
		snprintf(fun, sizeof(fun), "%s (%s.%u)", pp->use != NULL ? pp->use : "idle",
			 pin_gpio_port_name(port), pin);
	}
#endif
	pin_list_row(sh, pp, &first, fun, "-", "-");
}

static int cmd_pin_list(const struct shell *sh, size_t argc, char **argv)
{
	const struct pin_dev *only = NULL;

	if (argc > 1) {
		only = pin_dev_find(argv[1]);
		if (only == NULL) {
			shell_error(sh, "unknown device %s", argv[1]);
			return -ENODEV;
		}
	}

#ifdef CONFIG_GPIO_BEE
	pin_gpio_lines_init();
#endif

	shell_print(sh, PIN_LIST_FMT, "pad", "func", "config", "device");

	for (size_t i = 0; i < ARRAY_SIZE(pin_pads); i++) {
		pin_list_pad(sh, &pin_pads[i], only);
	}

	return 0;
}

/* pin set */

static int cmd_pin_set(const struct shell *sh, size_t argc, char **argv)
{
	struct pin_dev *pd = pin_dev_find(argv[1]);
	const char *owner;
	uint8_t old_pad, new_pad;
	int ret;

	ARG_UNUSED(argc);

	if (pd == NULL) {
		shell_error(sh, "unknown device %s", argv[1]);
		return -ENODEV;
	}

	if (pin_dev_is_console(pd->dev)) {
		shell_error(sh, "%s carries the console", pd->name);
		return -EACCES;
	}

	ret = pin_parse_pad(sh, argv[2], false, &old_pad);
	if (ret == 0) {
		ret = pin_parse_pad(sh, argv[3], true, &new_pad);
	}
	if (ret < 0) {
		return ret;
	}

	if (!pin_dev_uses_pad(pd, old_pad)) {
		shell_error(sh, "%s does not use %s", pd->name, argv[2]);
		return -EINVAL;
	}

#ifdef CONFIG_GPIO_BEE
	pin_gpio_lines_init();
#endif

	/* The latch trigger only selects the pad it watches, so it can share the
	 * pad with a GPIO.
	 */
	owner = pd->applied_on_use ? pin_dev_owner(new_pad, pd) : pin_pad_owner(new_pad, pd);
	if (owner == NULL && pin_dev_uses_pad(pd, new_pad)) {
		owner = pd->name;
	}
	if (owner != NULL) {
		shell_error(sh, "%s is used by %s", argv[3], owner);
		return -EBUSY;
	}

	/* The copy matches the live states, so it can be set up before the
	 * device goes down. It is only edited once the device is down.
	 */
	ret = pin_dev_copy_states(pd);
	if (ret < 0) {
		shell_error(sh, "no room to copy the states of %s", pd->name);
		return ret;
	}

	ret = pd->applied_on_use ? 0 : device_deinit(pd->dev);
	if (ret < 0) {
		shell_error(sh, "%s: deinit failed (%d)%s", pd->name, ret,
			    ret == -ENOTSUP ? ", the driver has no deinit"
			    : ret == -EBUSY ? ", the device is busy or not runtime suspended"
					    : "");
		return ret;
	}

	for (uint8_t i = 0; i < pd->pin_cnt; i++) {
		if (pd->pins[i].pin == old_pad) {
			pd->pins[i].pin = new_pad;
		}
	}

	ret = pinctrl_update_states(pd->pcfg, pd->states, pd->pcfg->state_cnt);
	if (ret < 0) {
		shell_error(sh, "%s: pinctrl update failed (%d)", pd->name, ret);
	}

	ret = pd->applied_on_use ? 0 : device_init(pd->dev);
	if (ret < 0) {
		shell_error(sh, "%s: init failed (%d)", pd->name, ret);
		return ret;
	}

	shell_print(sh, "%s: %s -> %s%s", pd->name, argv[2], argv[3],
		    pd->applied_on_use ? ", used from the next capture" : "");

	return 0;
}

/* pin gpio */

#ifdef CONFIG_GPIO_BEE

static void pin_gpio_release(struct pin_gpio_line *line)
{
	struct gpio_dt_spec *spec = line->spec;

	if (line->pad == GPIO_BEE_PAD_NONE) {
		return;
	}

	/* The PPU wakeup is armed on the pad, disable it with the interrupt */
	(void)gpio_pin_interrupt_configure(spec->port, spec->pin, GPIO_INT_DISABLE);
	(void)gpio_pin_configure(spec->port, spec->pin, GPIO_DISCONNECTED);
}

static void pin_gpio_idle(struct pin_gpio_line *line)
{
	if (line->idle_flags != 0) {
		(void)gpio_pin_configure_dt(line->spec, line->idle_flags);
	}
}

static int cmd_pin_gpio(const struct shell *sh, size_t argc, char **argv)
{
	struct pin_gpio_line *line = pin_gpio_line_find(argv[1]);
	struct gpio_dt_spec *spec;
	const struct device *port;
	const char *owner;
	gpio_pin_t pin;
	uint8_t pad;
	int ret;

	ARG_UNUSED(argc);

	if (line == NULL) {
		shell_error(sh, "unknown gpio line %s", argv[1]);
		return -ENODEV;
	}

	spec = line->spec;

	ret = pin_parse_pad(sh, argv[2], true, &pad);
	if (ret < 0) {
		return ret;
	}

	pin_gpio_lines_init();

	if (pad == line->pad) {
		shell_print(sh, "%s already on %s", line->name, argv[2]);
		return 0;
	}

	owner = pin_pad_owner(pad, line);
	if (owner != NULL) {
		shell_error(sh, "%s is used by %s", argv[2], owner);
		return -EBUSY;
	}

	/* The pad decides the pin: each pad carries one fixed GPIO pin */
	ret = gpio_bee_pad_to_pin(pad, &port, &pin);
	if (ret < 0) {
		shell_error(sh, "%s carries no enabled GPIO (%d)", argv[2], ret);
		return ret;
	}

	for (size_t i = 0; i < ARRAY_SIZE(pin_gpio_lines); i++) {
		if (&pin_gpio_lines[i] != line && pin_gpio_lines[i].spec->port == port &&
		    pin_gpio_lines[i].spec->pin == pin) {
			shell_error(sh, "%s %u is used by %s", pin_gpio_port_name(port), pin,
				    pin_gpio_lines[i].name);
			return -EBUSY;
		}
	}

	pin_gpio_release(line);

	/* A line that changes pin leaves its old pin without a pad */
	if (port != spec->port || pin != spec->pin) {
		(void)gpio_bee_pin_remap(spec->port, spec->pin, GPIO_BEE_PAD_NONE, NULL);
	}

	ret = gpio_bee_pin_remap(port, pin, pad, NULL);
	if (ret < 0) {
		shell_error(sh, "%s: remap to %s failed (%d)", line->name, argv[2], ret);
		(void)gpio_bee_pin_remap(spec->port, spec->pin, line->pad, NULL);
		pin_gpio_idle(line);
		return ret;
	}

	spec->port = port;
	spec->pin = pin;
	line->pad = pad;
	pin_gpio_idle(line);

	shell_print(sh, "%s: %s %u on %s", line->name, pin_gpio_port_name(port), pin, argv[2]);

	return 0;
}

static void pin_gpio_line_get(size_t idx, struct shell_static_entry *entry)
{
	entry->syntax = idx < ARRAY_SIZE(pin_gpio_lines) ? pin_gpio_lines[idx].name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(dsub_pin_gpio_line, pin_gpio_line_get);

#endif /* CONFIG_GPIO_BEE */

static void pin_dev_name_get(size_t idx, struct shell_static_entry *entry)
{
	entry->syntax = idx < ARRAY_SIZE(pin_devs) ? pin_devs[idx].name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(dsub_pin_dev, pin_dev_name_get);

SHELL_STATIC_SUBCMD_SET_CREATE(
	sub_pin,
	SHELL_CMD_ARG(list, &dsub_pin_dev, "list every pad and what holds it [dev]",
		      cmd_pin_list, 1, 1),
	SHELL_CMD_ARG(set, &dsub_pin_dev, "move a pin of a peripheral <dev> <pad> <new_pad>",
		      cmd_pin_set, 4, 0),
#ifdef CONFIG_GPIO_BEE
	SHELL_CMD_ARG(gpio, &dsub_pin_gpio_line, "move a test gpio line <line> <new_pad>",
		      cmd_pin_gpio, 3, 0),
#endif
	SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(pin, &sub_pin, "Pin control", NULL);
