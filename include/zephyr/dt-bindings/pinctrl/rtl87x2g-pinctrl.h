/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RTL87X2G_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RTL87X2G_PINCTRL_H_

/*
 * The whole RTL87X2G pin configuration information is encoded in a 32-bit bitfield
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
 * @name RTL87X2G pin configuration bit field positions and masks.
 * @{
 */

/** Position of the function field. */
#define RTL87X2G_FUN_POS 16U
/** Mask for the function field. */
#define RTL87X2G_FUN_MSK 0xFFFFU
/** Position of the direction field. */
#define RTL87X2G_DIR_POS 14U
/** Mask for the low direction field. */
#define RTL87X2G_DIR_MSK 0x1U
/** Position of the drive configuration field. */
#define RTL87X2G_DRIVE_POS 13U
/** Mask for the drive configuration field. */
#define RTL87X2G_DRIVE_MSK 0x1U
/** Position of the pull configuration field. */
#define RTL87X2G_PULL_POS 11U
/** Mask for the pull configuration field. */
#define RTL87X2G_PULL_MSK 0x3U
/** Position of the pin field. */
#define RTL87X2G_PIN_POS 0U
/** Mask for the pin field. */
#define RTL87X2G_PIN_MSK 0x7FFU

/** @} */

/**
 * @name RTL87X2G pinctrl pin functions.
 * @{
 */

#define RTL87X2G_IDLE_MODE                   0
#define RTL87X2G_UART0_TX                    1
#define RTL87X2G_UART0_RX                    2
#define RTL87X2G_UART0_CTS                   3
#define RTL87X2G_UART0_RTS                   4
#define RTL87X2G_UART1_TX                    5
#define RTL87X2G_UART1_RX                    6
#define RTL87X2G_UART1_CTS                   7
#define RTL87X2G_UART1_RTS                   8
#define RTL87X2G_UART2_TX                    9
#define RTL87X2G_UART2_RX                    10
#define RTL87X2G_UART2_CTS                   11
#define RTL87X2G_UART2_RTS                   12
#define RTL87X2G_UART3_TX                    13
#define RTL87X2G_UART3_RX                    14
#define RTL87X2G_UART3_CTS                   15
#define RTL87X2G_UART3_RTS                   16
#define RTL87X2G_UART4_TX                    17
#define RTL87X2G_UART4_RX                    18
#define RTL87X2G_UART4_CTS                   19
#define RTL87X2G_UART4_RTS                   20
#define RTL87X2G_UART5_TX                    21
#define RTL87X2G_UART5_RX                    22
#define RTL87X2G_UART5_CTS                   23
#define RTL87X2G_UART5_RTS                   24
#define RTL87X2G_I2C0_CLK                    29
#define RTL87X2G_I2C0_DAT                    30
#define RTL87X2G_I2C1_CLK                    31
#define RTL87X2G_I2C1_DAT                    32
#define RTL87X2G_I2C2_CLK                    33
#define RTL87X2G_I2C2_DAT                    34
#define RTL87X2G_I2C3_CLK                    35
#define RTL87X2G_I2C3_DAT                    36
#define RTL87X2G_SPI0_CLK_MASTER             37
#define RTL87X2G_SPI0_MO_MASTER              38
#define RTL87X2G_SPI0_MI_MASTER              39
#define RTL87X2G_SPI0_CSN_0_MASTER           40
#define RTL87X2G_SPI0_CSN_1_MASTER           41
#define RTL87X2G_SPI0_CSN_2_MASTER           42
#define RTL87X2G_SPI0_CSN_0_SLAVE            43
#define RTL87X2G_SPI0_CLK_SLAVE              44
#define RTL87X2G_SPI0_SO_SLAVE               45
#define RTL87X2G_SPI0_SI_SLAVE               46
#define RTL87X2G_SPI1_CLK_MASTER             47
#define RTL87X2G_SPI1_MO_MASTER              48
#define RTL87X2G_SPI1_MI_MASTER              49
#define RTL87X2G_SPI1_CSN_0_MASTER           50
#define RTL87X2G_SPI1_CSN_1_MASTER           51
#define RTL87X2G_SPI1_CSN_2_MASTER           52
#define RTL87X2G_SPI2W_DATA                  53
#define RTL87X2G_SPI2W_CLK                   54
#define RTL87X2G_SPI2W_CS                    55
#define RTL87X2G_ENPWM0_P                    65
#define RTL87X2G_ENPWM0_N                    66
#define RTL87X2G_ENPWM1_P                    67
#define RTL87X2G_ENPWM1_N                    68
#define RTL87X2G_ENPWM2_P                    69
#define RTL87X2G_ENPWM2_N                    70
#define RTL87X2G_ENPWM3_P                    71
#define RTL87X2G_ENPWM3_N                    72
#define RTL87X2G_TIMER_PWM2                  83
#define RTL87X2G_TIMER_PWM3                  84
#define RTL87X2G_ISO7816_RST                 85
#define RTL87X2G_ISO7816_CLK                 86
#define RTL87X2G_ISO7816_IO                  87
#define RTL87X2G_ISO7816_VCC_EN              88
#define RTL87X2G_DWGPIO                      89
#define RTL87X2G_IRDA_TX                     90
#define RTL87X2G_IRDA_RX                     91
#define RTL87X2G_TIMER_PWM4                  92
#define RTL87X2G_TIMER_PWM5                  93
#define RTL87X2G_TIMER_PWM6                  94
#define RTL87X2G_TIMER_PWM7                  95
#define RTL87X2G_TIMER_PWM2_P                96
#define RTL87X2G_TIMER_PWM2_N                97
#define RTL87X2G_TIMER_PWM3_P                98
#define RTL87X2G_TIMER_PWM3_N                99
#define RTL87X2G_KEY_COL_0                   100
#define RTL87X2G_KEY_COL_1                   101
#define RTL87X2G_KEY_COL_2                   102
#define RTL87X2G_KEY_COL_3                   103
#define RTL87X2G_KEY_COL_4                   104
#define RTL87X2G_KEY_COL_5                   105
#define RTL87X2G_KEY_COL_6                   106
#define RTL87X2G_KEY_COL_7                   107
#define RTL87X2G_KEY_COL_8                   108
#define RTL87X2G_KEY_COL_9                   109
#define RTL87X2G_KEY_COL_10                  110
#define RTL87X2G_KEY_COL_11                  111
#define RTL87X2G_KEY_COL_12                  112
#define RTL87X2G_KEY_COL_13                  113
#define RTL87X2G_KEY_COL_14                  114
#define RTL87X2G_KEY_COL_15                  115
#define RTL87X2G_KEY_COL_16                  116
#define RTL87X2G_KEY_COL_17                  117
#define RTL87X2G_KEY_COL_18                  118
#define RTL87X2G_KEY_COL_19                  119
#define RTL87X2G_KEY_ROW_0                   120
#define RTL87X2G_KEY_ROW_1                   121
#define RTL87X2G_KEY_ROW_2                   122
#define RTL87X2G_KEY_ROW_3                   123
#define RTL87X2G_KEY_ROW_4                   124
#define RTL87X2G_KEY_ROW_5                   125
#define RTL87X2G_KEY_ROW_6                   126
#define RTL87X2G_KEY_ROW_7                   127
#define RTL87X2G_KEY_ROW_8                   128
#define RTL87X2G_KEY_ROW_9                   129
#define RTL87X2G_KEY_ROW_10                  130
#define RTL87X2G_KEY_ROW_11                  131
#define RTL87X2G_km4_clk_div_4               138
#define RTL87X2G_card_detect_n_0             147
#define RTL87X2G_biu_volt_reg_0              148
#define RTL87X2G_back_end_power_0            149
#define RTL87X2G_card_int_n_sdhc_0           150
#define RTL87X2G_CAN_TX                      155
#define RTL87X2G_CAN_RX                      156
#define RTL87X2G_LRC_SPORT1                  157
#define RTL87X2G_BCLK_SPORT1                 158
#define RTL87X2G_ADCDAT_SPORT1               159
#define RTL87X2G_DACDAT_SPORT1               160
#define RTL87X2G_DMIC1_CLK                   162
#define RTL87X2G_DMIC1_DAT                   163
#define RTL87X2G_LRC_I_CODEC_SLAVE           164
#define RTL87X2G_BCLK_I_CODEC_SLAVE          165
#define RTL87X2G_SDI_CODEC_SLAVE             166
#define RTL87X2G_SDO_CODEC_SLAVE             167
#define RTL87X2G_LRC_SPORT0                  172
#define RTL87X2G_BCLK_SPORT0                 173
#define RTL87X2G_ADCDAT_SPORT0               174
#define RTL87X2G_DACDAT_SPORT0               175
#define RTL87X2G_MCLK_OUT                    176
#define RTL87X2G_MCLK_IN                     189
#define RTL87X2G_LRC_RX_CODEC_SLAVE          190
#define RTL87X2G_LRC_RX_SPORT0               191
#define RTL87X2G_LRC_RX_SPORT1               192
#define RTL87X2G_PDM_DATA                    196
#define RTL87X2G_PDM_CLK                     197
#define RTL87X2G_I2S1_LRC_TX_SLAVE           198
#define RTL87X2G_I2S1_BCLK_SLAVE             199
#define RTL87X2G_I2S1_SDI_SLAVE              200
#define RTL87X2G_I2S1_SDO_SLAVE              201
#define RTL87X2G_I2S1_LRC_RX_SLAVE           202
#define RTL87X2G_BT_COEX_I_0                 216
#define RTL87X2G_BT_COEX_I_1                 217
#define RTL87X2G_BT_COEX_I_2                 218
#define RTL87X2G_BT_COEX_I_3                 219
#define RTL87X2G_BT_COEX_O_0                 220
#define RTL87X2G_BT_COEX_O_1                 221
#define RTL87X2G_BT_COEX_O_2                 222
#define RTL87X2G_BT_COEX_O_3                 223
#define RTL87X2G_PTA_I2C_CLK_SLAVE           224
#define RTL87X2G_PTA_I2C_DAT_SLAVE           225
#define RTL87X2G_PTA_I2C_INT_OUT             226
#define RTL87X2G_EN_EXPA                     227
#define RTL87X2G_EN_EXLNA                    228
#define RTL87X2G_SEL_TPM_SW                  229
#define RTL87X2G_SEL_TPM_N_SW                230
#define RTL87X2G_ANT_SW0                     231
#define RTL87X2G_ANT_SW1                     232
#define RTL87X2G_ANT_SW2                     233
#define RTL87X2G_ANT_SW3                     234
#define RTL87X2G_ANT_SW4                     235
#define RTL87X2G_ANT_SW5                     236
#define RTL87X2G_phy_gpio_1                  237
#define RTL87X2G_phy_gpio_2                  238
#define RTL87X2G_slow_debug_mux_1            239
#define RTL87X2G_slow_debug_mux_2            240
#define RTL87X2G_test_mode                   246
#define RTL87X2G_SWD_CLK                     253
#define RTL87X2G_SWD_DIO                     254
#define RTL87X2G_dig_debug                   255

#define RTL87X2G_SW_MODE                     257

#define RTL87X2G_QDPH0_IN_NONE               0x0F00
#define RTL87X2G_QDPH0_IN_P1_3_P1_4          0x0F01
#define RTL87X2G_QDPH0_IN_P5_6_P5_7          0x0F02
#define RTL87X2G_QDPH0_IN_P9_0_P9_1          0x0F03
/** @} */

#define P0_0        0       /*!< GPIOA_0  */
#define P0_1        1       /*!< GPIOA_1  */
#define P0_2        2       /*!< GPIOA_2  */
#define P0_3        3       /*!< GPIOA_3  */
#define P0_4        4       /*!< GPIOA_4  */
#define P0_5        5       /*!< GPIOA_5  */
#define P0_6        6       /*!< GPIOA_6  */
#define P0_7        7       /*!< GPIOA_7  */

#define P1_0        8       /*!< GPIOA_8  */
#define P1_1        9       /*!< GPIOA_9  */
#define P1_2        10      /*!< GPIOA_10 */
#define P1_3        11      /*!< GPIOA_11 */
#define P1_4        12      /*!< GPIOA_12 */
#define P1_5        13      /*!< GPIOA_13 */
#define P1_6        14      /*!< GPIOA_14 */
#define P1_7        15      /*!< GPIOA_15 */

#define P2_0        16      /*!< GPIOA_21 */
#define P2_1        17      /*!< GPIOA_22 */
#define P2_2        18      /*!< GPIOA_23 */
#define P2_3        19      /*!< GPIOA_24 */
#define P2_4        20      /*!< GPIOA_25 */
#define P2_5        21      /*!< GPIOA_26 */
#define P2_6        22      /*!< GPIOA_27 */
#define P2_7        23      /*!< GPIOA_28 */

#define P3_0        24      /*!< GPIOA_29 */
#define P3_1        25      /*!< GPIOA_30 */
#define P3_2        26      /*!< GPIOA_31 */
#define P3_3        27      /*!< GPIOB_0 */
#define P3_4        28      /*!< GPIOB_1 */
#define P3_5        29      /*!< GPIOB_2 */
#define P3_6        30      /*!< GPIOB_3 */
#define P3_7        31      /*!< GPIOB_4 */

#define P4_0        32      /*!< GPIOB_5  */
#define P4_1        33      /*!< GPIOB_6  */
#define P4_2        34      /*!< GPIOB_7  */
#define P4_3        35      /*!< GPIOB_8  */
#define P4_4        36      /*!< GPIOB_9  */
#define P4_5        37      /*!< GPIOB_10  */
#define P4_6        38      /*!< GPIOB_11  */
#define P4_7        39      /*!< GPIOB_12  */

#define P5_0        40      /*!< GPIOB_13  */
#define P5_1        41      /*!< GPIOB_14  */
#define P5_2        42      /*!< GPIOB_15 */
#define P5_3        43      /*!< GPIOB_16 */
#define P5_4        44      /*!< GPIOB_17 */
#define P5_5        45      /*!< GPIOB_18 */
#define P5_6        46      /*!< GPIOB_19 */
#define P5_7        47      /*!< GPIOB_20 */

#define P6_0        48      /*!< GPIOB_19 */
#define P6_1        49      /*!< GPIOB_20 */
#define P6_2        50      /*!< GPIOB_21 */
#define P6_3        51      /*!< GPIOB_22 */
#define P6_4        52      /*!< GPIOB_23 */
#define P6_5        53      /*!< GPIOB_24 */
#define P6_6        54      /*!< GPIOB_25 */
#define P6_7        55      /*!< GPIOB_26 */

#define P7_0        56      /*!< GPIOB_27 */
#define P7_1        57      /*!< GPIOB_28 */
#define P7_2        58      /*!< GPIOB_29 */
#define P7_3        59      /*!< GPIOB_30 */
#define P7_4        60      /*!< GPIOB_31 */

//#define P7_5        61      /*!< RSVD */
//#define P7_6        62      /*!< RSVD */
//#define P7_7        63      /*!< RSVD */

#define MICBIAS     64      /*!< P8_0 GPIOA_16*/
#define XI32K       65      /*!< P8_1 GPIOA_17*/
#define XO32K       66      /*!< P8_2 GPIOA_18*/
#define DACP        67      /*!< P8_3 GPIOA_19*/
#define DACN        68      /*!< P8_4 GPIOA_20*/

//#define P8_5        69      /*!< RSVD */
//#define P8_6        70      /*!< RSVD */
//#define P8_7        71      /*!< RSVD */

#define P9_0        72      /*!< GPIOB_21 */
#define P9_1        73      /*!< GPIOB_22 */
#define P9_2        74      /*!< GPIOB_23 */
#define P9_3        75      /*!< GPIOB_24 */
#define P9_4        76      /*!< GPIOB_25 */
#define P9_5        77      /*!< GPIOB_26 */
#define P9_6        78      /*!< GPIOB_27 */
#define P9_7        79      /*!< GPIOB_28 */

#define P10_0       80      /*!< GPIOB_29 */
#define P10_1       81      /*!< GPIOB_30 */
#define P10_2       82      /*!< GPIOB_31 */

#define ADC_0       P2_0    /*!< GPIO16   */
#define ADC_1       P2_1    /*!< GPIO17   */
#define ADC_2       P2_2    /*!< GPIO18   */
#define ADC_3       P2_3    /*!< GPIO19   */
#define ADC_4       P2_4    /*!< GPIO20   */
#define ADC_5       P2_5    /*!< GPIO21   */
#define ADC_6       P2_6    /*!< GPIO22   */
#define ADC_7       P2_7    /*!< GPIO23   */

#define RTL87X2G_DIR_IN           0
#define RTL87X2G_DIR_OUT          1
#define RTL87X2G_DRV_LOW          0
#define RTL87X2G_DRV_HIGH         1
#define RTL87X2G_PULL_DOWN        0
#define RTL87X2G_PULL_UP          1
#define RTL87X2G_PULL_NONE        2

/**
 * @name RTL87X2G pinctrl helpers to indicate disconnected pins.
 * @{
 */

/** Indicates that a pin is disconnected */
#define RTL87X2G_PIN_DISCONNECTED RTL87X2G_PIN_MSK

/** @} */

/**
 * @brief Utility macro to build RTL87X2G psels property entry.
 *
 * @param fun Pin function configuration (see RTL87X2G_FUNC_{name} macros).
 * @param pin Pin (0..82).
 */
#define RTL87X2G_PSEL(fun, pin, dir, drive, pull)                            \
    (((((pin) & RTL87X2G_PIN_MSK) << RTL87X2G_PIN_POS) |            \
      (((RTL87X2G_ ## fun) & RTL87X2G_FUN_MSK) << RTL87X2G_FUN_POS))  |\
     ((((RTL87X2G_ ## dir) & RTL87X2G_DIR_MSK) << RTL87X2G_DIR_POS) | \
      (((RTL87X2G_ ## drive) & RTL87X2G_DRIVE_MSK) << RTL87X2G_DRIVE_POS) | \
      (((RTL87X2G_ ## pull) & RTL87X2G_PULL_MSK) << RTL87X2G_PULL_POS)))

/**
 * @brief Utility macro to build nRF psels property entry when a pin is disconnected.
 *
 * This can be useful in situations where code running before Zephyr, e.g. a bootloader
 * configures pins that later needs to be disconnected.
 *
 * @param fun Pin function configuration (see NRF_FUN_{name} macros).
 */
#define RTL87X2G_PSEL_DISCONNECTED(fun)                            \
    (RTL87X2G_PIN_DISCONNECTED << RTL87X2G_PIN_POS |                                   \
     ((RTL87X2G_ ## fun & RTL87X2G_FUN_MSK) << RTL87X2G_FUN_POS))

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RTL87X2G_PINCTRL_H_ */
