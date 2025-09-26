/*
 * Copyright(c) 2025, Realtek Semiconductor Corporation.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RTL8752H_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RTL8752H_PINCTRL_H_

#include "bee-pinctrl.h"

/**
 * @name RTL8752H pinctrl pin functions.
 * @{
 */

#define BEE_IDLE_MODE          0
#define BEE_UART2_TX           1
#define BEE_UART2_RX           2
#define BEE_UART2_CTS          3
#define BEE_UART2_RTS          4
#define BEE_I2C0_CLK           5
#define BEE_I2C0_DAT           6
#define BEE_I2C1_CLK           7
#define BEE_I2C1_DAT           8
#define BEE_PWM2_P             9
#define BEE_PWM2_N             10
#define BEE_ENPWM0_P           11
#define BEE_ENPWM0_N           12
#define BEE_TIM_PWM0           13
#define BEE_TIM_PWM1           14
#define BEE_TIM_PWM2           15
#define BEE_TIM_PWM3           16
#define BEE_TIM_PWM4           17
#define BEE_TIM_PWM5           18
#define BEE_ENPWM0             19
#define BEE_ENPWM1             20
#define BEE_qdec_phase_a_x     21
#define BEE_qdec_phase_b_x     22
#define BEE_qdec_phase_a_y     23
#define BEE_qdec_phase_b_y     24
#define BEE_qdec_phase_a_z     25
#define BEE_qdec_phase_b_z     26
#define BEE_UART0_TX           29
#define BEE_UART0_RX           30
#define BEE_UART0_CTS          31
#define BEE_UART0_RTS          32
#define BEE_IRDA_TX            33
#define BEE_IRDA_RX            34
#define BEE_UART1_TX           35
#define BEE_UART1_RX           36
#define BEE_UART1_CTS          37
#define BEE_UART1_RTS          38
#define BEE_SPI1_SS_N_0_MASTER 39
#define BEE_SPI1_SS_N_1_MASTER 40
#define BEE_SPI1_SS_N_2_MASTER 41
#define BEE_SPI1_CLK_MASTER    42
#define BEE_SPI1_MO_MASTER     43
#define BEE_SPI1_MI_MASTER     44
#define BEE_SPI0_SS_N_0_SLAVE  45
#define BEE_SPI0_CLK_SLAVE     46
#define BEE_SPI0_SO_SLAVE      47
#define BEE_SPI0_SI_SLAVE      48
#define BEE_SPI0_SS_N_0_MASTER 49
#define BEE_SPI0_CLK_MASTER    50
#define BEE_SPI0_MO_MASTER     51
#define BEE_SPI0_MI_MASTER     52
#define BEE_SPI2W_DATA         53
#define BEE_SPI2W_CLK          54
#define BEE_SPI2W_CS           55
#define BEE_SWD_CLK            56
#define BEE_SWD_DIO            57
#define BEE_KEY_COL_0          58
#define BEE_KEY_COL_1          59
#define BEE_KEY_COL_2          60
#define BEE_KEY_COL_3          61
#define BEE_KEY_COL_4          62
#define BEE_KEY_COL_5          63
#define BEE_KEY_COL_6          64
#define BEE_KEY_COL_7          65
#define BEE_KEY_COL_8          66
#define BEE_KEY_COL_9          67
#define BEE_KEY_COL_10         68
#define BEE_KEY_COL_11         69
#define BEE_KEY_COL_12         70
#define BEE_KEY_COL_13         71
#define BEE_KEY_COL_14         72
#define BEE_KEY_COL_15         73
#define BEE_KEY_COL_16         74
#define BEE_KEY_COL_17         75
#define BEE_KEY_COL_18         76
#define BEE_KEY_COL_19         77
#define BEE_KEY_ROW_0          78
#define BEE_KEY_ROW_1          79
#define BEE_KEY_ROW_2          80
#define BEE_KEY_ROW_3          81
#define BEE_KEY_ROW_4          82
#define BEE_KEY_ROW_5          83
#define BEE_KEY_ROW_6          84
#define BEE_KEY_ROW_7          85
#define BEE_KEY_ROW_8          86
#define BEE_KEY_ROW_9          87
#define BEE_KEY_ROW_10         88
#define BEE_KEY_ROW_11         89
#define BEE_DWGPIO             90
#define BEE_DMIC1_CLK          96
#define BEE_DMIC1_DAT          97
#define BEE_LRC_I_CODEC_SLAVE  98
#define BEE_BCLK_I_CODEC_SLAVE 99
#define BEE_SDI_CODEC_SLAVE    100
#define BEE_SDO_CODEC_SLAVE    101
#define BEE_BT_COEX_I_0        106
#define BEE_BT_COEX_I_1        107
#define BEE_BT_COEX_I_2        108
#define BEE_BT_COEX_I_3        109
#define BEE_BT_COEX_O_0        110
#define BEE_BT_COEX_O_1        111
#define BEE_BT_COEX_O_2        112
#define BEE_BT_COEX_O_3        113
#define BEE_PTA_I2C_CLK_SLAVE  114
#define BEE_PTA_I2C_DAT_SLAVE  115
#define BEE_PTA_I2C_INT_OUT    116
#define BEE_EN_EXPA            117
#define BEE_EN_EXLNA           118
#define BEE_LRC_SPORT0         123
#define BEE_BCLK_SPORT0        124
#define BEE_ADCDAT_SPORT0      125
#define BEE_DACDAT_SPORT0      126
#define BEE_MCLK               127
#define BEE_PINMUX_MAX         (BEE_MCLK + 1)
#define BEE_SW_MODE            (BEE_PINMUX_MAX + 1)
#define BEE_PWR_OFF            (BEE_PINMUX_MAX + 2)

#define BEE_PIN_DISCONNECTED BEE_PIN_MSK

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

#define BEE_DIR_IN    0
#define BEE_DIR_OUT   1
#define BEE_DRV_LOW   0
#define BEE_DRV_HIGH  1
#define BEE_PULL_UP   0
#define BEE_PULL_DOWN 1
#define BEE_PULL_NONE 2

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RTL8752H_PINCTRL_H_ */
