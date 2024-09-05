/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RTL8752H_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RTL8752H_PINCTRL_H_

/*
 * The whole RTL8752H pin configuration information is encoded in a 32-bit bitfield
 * organized as follows:
 *
 * - 31..16: Pin function.
 * - 15:     Reserved.
 * - 14:     Pin direction configuration.
 * - 13:     Pin output drive configuration.
 * - 12..11: Pin pull configuration.
 * - 10..0:  Pin number (combination of port and pin).
 */

/**
 * @name RTL8752H pin configuration bit field positions and masks.
 * @{
 */

/** Position of the function field. */
#define RTL8752H_FUN_POS   16U
/** Mask for the function field. */
#define RTL8752H_FUN_MSK   0xFFFFU
/** Position of the direction field. */
#define RTL8752H_DIR_POS   14U
/** Mask for the low direction field. */
#define RTL8752H_DIR_MSK   0x1U
/** Position of the drive configuration field. */
#define RTL8752H_DRIVE_POS 13U
/** Mask for the drive configuration field. */
#define RTL8752H_DRIVE_MSK 0x1U
/** Position of the pull configuration field. */
#define RTL8752H_PULL_POS  11U
/** Mask for the pull configuration field. */
#define RTL8752H_PULL_MSK  0x3U
/** Position of the pin field. */
#define RTL8752H_PIN_POS   0U
/** Mask for the pin field. */
#define RTL8752H_PIN_MSK   0x7FFU

/** @} */

/**
 * @name RTL87X2G pinctrl pin functions.
 * @{
 */

#define RTL8752H_IDLE_MODE          0
#define RTL8752H_UART2_TX           1
#define RTL8752H_UART2_RX           2
#define RTL8752H_UART2_CTS          3
#define RTL8752H_UART2_RTS          4
#define RTL8752H_I2C0_CLK           5
#define RTL8752H_I2C0_DAT           6
#define RTL8752H_I2C1_CLK           7
#define RTL8752H_I2C1_DAT           8
#define RTL8752H_PWM2_P             9
#define RTL8752H_PWM2_N             10
#define RTL8752H_ENPWM0_P           11
#define RTL8752H_ENPWM0_N           12
#define RTL8752H_TIM_PWM0           13
#define RTL8752H_TIM_PWM1           14
#define RTL8752H_TIM_PWM2           15
#define RTL8752H_TIM_PWM3           16
#define RTL8752H_TIM_PWM4           17
#define RTL8752H_TIM_PWM5           18
#define RTL8752H_ENPWM0             19
#define RTL8752H_ENPWM1             20
#define RTL8752H_qdec_phase_a_x     21
#define RTL8752H_qdec_phase_b_x     22
#define RTL8752H_qdec_phase_a_y     23
#define RTL8752H_qdec_phase_b_y     24
#define RTL8752H_qdec_phase_a_z     25
#define RTL8752H_qdec_phase_b_z     26
#define RTL8752H_UART0_TX           29
#define RTL8752H_UART0_RX           30
#define RTL8752H_UART0_CTS          31
#define RTL8752H_UART0_RTS          32
#define RTL8752H_IRDA_TX            33
#define RTL8752H_IRDA_RX            34
#define RTL8752H_UART1_TX           35
#define RTL8752H_UART1_RX           36
#define RTL8752H_UART1_CTS          37
#define RTL8752H_UART1_RTS          38
#define RTL8752H_SPI1_SS_N_0_MASTER 39
#define RTL8752H_SPI1_SS_N_1_MASTER 40
#define RTL8752H_SPI1_SS_N_2_MASTER 41
#define RTL8752H_SPI1_CLK_MASTER    42
#define RTL8752H_SPI1_MO_MASTER     43
#define RTL8752H_SPI1_MI_MASTER     44
#define RTL8752H_SPI0_SS_N_0_SLAVE  45
#define RTL8752H_SPI0_CLK_SLAVE     46
#define RTL8752H_SPI0_SO_SLAVE      47
#define RTL8752H_SPI0_SI_SLAVE      48
#define RTL8752H_SPI0_SS_N_0_MASTER 49
#define RTL8752H_SPI0_CLK_MASTER    50
#define RTL8752H_SPI0_MO_MASTER     51
#define RTL8752H_SPI0_MI_MASTER     52
#define RTL8752H_SPI2W_DATA         53
#define RTL8752H_SPI2W_CLK          54
#define RTL8752H_SPI2W_CS           55
#define RTL8752H_SWD_CLK            56
#define RTL8752H_SWD_DIO            57
#define RTL8752H_KEY_COL_0          58
#define RTL8752H_KEY_COL_1          59
#define RTL8752H_KEY_COL_2          60
#define RTL8752H_KEY_COL_3          61
#define RTL8752H_KEY_COL_4          62
#define RTL8752H_KEY_COL_5          63
#define RTL8752H_KEY_COL_6          64
#define RTL8752H_KEY_COL_7          65
#define RTL8752H_KEY_COL_8          66
#define RTL8752H_KEY_COL_9          67
#define RTL8752H_KEY_COL_10         68
#define RTL8752H_KEY_COL_11         69
#define RTL8752H_KEY_COL_12         70
#define RTL8752H_KEY_COL_13         71
#define RTL8752H_KEY_COL_14         72
#define RTL8752H_KEY_COL_15         73
#define RTL8752H_KEY_COL_16         74
#define RTL8752H_KEY_COL_17         75
#define RTL8752H_KEY_COL_18         76
#define RTL8752H_KEY_COL_19         77
#define RTL8752H_KEY_ROW_0          78
#define RTL8752H_KEY_ROW_1          79
#define RTL8752H_KEY_ROW_2          80
#define RTL8752H_KEY_ROW_3          81
#define RTL8752H_KEY_ROW_4          82
#define RTL8752H_KEY_ROW_5          83
#define RTL8752H_KEY_ROW_6          84
#define RTL8752H_KEY_ROW_7          85
#define RTL8752H_KEY_ROW_8          86
#define RTL8752H_KEY_ROW_9          87
#define RTL8752H_KEY_ROW_10         88
#define RTL8752H_KEY_ROW_11         89
#define RTL8752H_DWGPIO             90
#define RTL8752H_DMIC1_CLK          96
#define RTL8752H_DMIC1_DAT          97
#define RTL8752H_LRC_I_CODEC_SLAVE  98
#define RTL8752H_BCLK_I_CODEC_SLAVE 99
#define RTL8752H_SDI_CODEC_SLAVE    100
#define RTL8752H_SDO_CODEC_SLAVE    101
#define RTL8752H_BT_COEX_I_0        106
#define RTL8752H_BT_COEX_I_1        107
#define RTL8752H_BT_COEX_I_2        108
#define RTL8752H_BT_COEX_I_3        109
#define RTL8752H_BT_COEX_O_0        110
#define RTL8752H_BT_COEX_O_1        111
#define RTL8752H_BT_COEX_O_2        112
#define RTL8752H_BT_COEX_O_3        113
#define RTL8752H_PTA_I2C_CLK_SLAVE  114
#define RTL8752H_PTA_I2C_DAT_SLAVE  115
#define RTL8752H_PTA_I2C_INT_OUT    116
#define RTL8752H_EN_EXPA            117
#define RTL8752H_EN_EXLNA           118
#define RTL8752H_LRC_SPORT0         123
#define RTL8752H_BCLK_SPORT0        124
#define RTL8752H_ADCDAT_SPORT0      125
#define RTL8752H_DACDAT_SPORT0      126
#define RTL8752H_MCLK               127

#define RTL8752H_SW_MODE 257

/** @} */

#define P0_0 0 /**<GPIO0   */
#define P0_1 1 /**<GPIO1   */
#define P0_2 2 /**<GPIO2   */
#define P0_3 3 /**<GPIO3   */
#define P0_4 4 /**<GPIO4   */
#define P0_5 5 /**<GPIO5   */
#define P0_6 6 /**<GPIO6   */
#define P0_7 7 /**<GPIO7   */
#define P1_0 8 /**<GPIO8   */
#define P1_1 9 /**<GPIO9   */
#define P1_3 11
#define P1_4 12
#define P1_6 14 /**<GPIO14   */
#define P1_7 15 /**<GPIO15   */
#define P2_0 16 /**<GPIO16   */
#define P2_1 17 /**<GPIO17   */
#define P2_2 18 /**<GPIO18   */
#define P2_3 19 /**<GPIO19   */
#define P2_4 20 /**<GPIO20   */
#define P2_5 21 /**<GPIO21   */
#define P2_6 22 /**<GPIO22   */
#define P2_7 23 /**<GPIO23   */
#define P3_0 24 /**<GPIO24   */
#define P3_1 25 /**<GPIO25   */
#define P3_2 26 /**<GPIO26   */
#define P3_3 27 /**<GPIO27   */
#define P3_4 28 /**<GPIO28   */
#define P3_5 29 /**<GPIO29   */
#define P3_6 30 /**<GPIO30   */
#define P4_0 32 /**<GPIO13   */
#define P4_1 33 /**<GPIO29   */
#define P4_2 34 /**<GPIO30   */
#define P4_3 35 /**<GPIO31   */
#define H_0  36 /**<GPIO10   MICBIAS   */
#define P5_1 37 /**<GPIO11   */
#define P5_2 38 /**<GPIO12   */

#define RTL8752H_DIR_IN    0
#define RTL8752H_DIR_OUT   1
#define RTL8752H_DRV_LOW   0
#define RTL8752H_DRV_HIGH  1
#define RTL8752H_PULL_DOWN 0
#define RTL8752H_PULL_UP   1
#define RTL8752H_PULL_NONE 2

/**
 * @name RTL8752H pinctrl helpers to indicate disconnected pins.
 * @{
 */

/** Indicates that a pin is disconnected */
#define RTL8752H_PIN_DISCONNECTED RTL8752H_PIN_MSK

/** @} */

/**
 * @brief Utility macro to build RTL8752H psels property entry.
 *
 * @param fun Pin function configuration (see RTL8752H_FUNC_{name} macros).
 * @param pin Pin (0..82).
 */
#define RTL8752H_PSEL(fun, pin, dir, drive, pull)                                                  \
	(((((pin) & RTL8752H_PIN_MSK) << RTL8752H_PIN_POS) |                                       \
	  (((RTL8752H_##fun) & RTL8752H_FUN_MSK) << RTL8752H_FUN_POS)) |                           \
	 ((((RTL8752H_##dir) & RTL8752H_DIR_MSK) << RTL8752H_DIR_POS) |                            \
	  (((RTL8752H_##drive) & RTL8752H_DRIVE_MSK) << RTL8752H_DRIVE_POS) |                      \
	  (((RTL8752H_##pull) & RTL8752H_PULL_MSK) << RTL8752H_PULL_POS)))

/**
 * @brief Utility macro to build rtl8752h psels property entry when a pin is disconnected.
 *
 * This can be useful in situations where code running before Zephyr, e.g. a bootloader
 * configures pins that later needs to be disconnected.
 *
 * @param fun Pin function configuration (see RTL8752H_FUNC_{name} macros).
 */
#define RTL8752H_PSEL_DISCONNECTED(fun)                                                            \
	(RTL8752H_PIN_DISCONNECTED << RTL8752H_PIN_POS |                                           \
	 ((RTL8752H_##fun & RTL8752H_FUN_MSK) << RTL8752H_FUN_POS))

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RTL8752H_PINCTRL_H_ */
