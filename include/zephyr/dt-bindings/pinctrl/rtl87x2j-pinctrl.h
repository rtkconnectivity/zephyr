/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief RTL87X2J Pin Control (Pinmux) Header
 *
 * This file defines the pinmux functions and pin assignments for the
 * Realtek RTL87X2J series SoC.
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RTL87X2J_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RTL87X2J_PINCTRL_H_

#include "bee-pinctrl.h"

/**
 * @name RTL87X2J Pinmux Functions
 * @{
 */
#define BEE_IDLE_MODE          0                    /**< IDLE_MODE pinmux function */
#define BEE_UART0_TX           1                    /**< UART0_TX pinmux function */
#define BEE_UART0_RX           2                    /**< UART0_RX pinmux function */
#define BEE_UART0_CTS          3                    /**< UART0_CTS pinmux function */
#define BEE_UART0_RTS          4                    /**< UART0_RTS pinmux function */
#define BEE_UART1_TX           5                    /**< UART1_TX pinmux function */
#define BEE_UART1_RX           6                    /**< UART1_RX pinmux function */
#define BEE_UART1_CTS          7                    /**< UART1_CTS pinmux function */
#define BEE_UART1_RTS          8                    /**< UART1_RTS pinmux function */
#define BEE_UART2_TX           9                    /**< UART2_TX pinmux function */
#define BEE_UART2_RX           10                   /**< UART2_RX pinmux function */
#define BEE_UART2_CTS          11                   /**< UART2_CTS pinmux function */
#define BEE_UART2_RTS          12                   /**< UART2_RTS pinmux function */
#define BEE_UART3_TX           13                   /**< UART3_TX pinmux function */
#define BEE_UART3_RX           14                   /**< UART3_RX pinmux function */
#define BEE_UART3_CTS          15                   /**< UART3_CTS pinmux function */
#define BEE_UART3_RTS          16                   /**< UART3_RTS pinmux function */
#define BEE_I2C0_CLK           17                   /**< I2C0_CLK pinmux function */
#define BEE_I2C0_DAT           18                   /**< I2C0_DAT pinmux function */
#define BEE_I2C1_CLK           19                   /**< I2C1_CLK pinmux function */
#define BEE_I2C1_DAT           20                   /**< I2C1_DAT pinmux function */
#define BEE_SPI0_SS_N_0_SLAVE  21                   /**< SPI0_SS_N_0_SLAVE pinmux function */
#define BEE_SPI0_CLK_SLAVE     22                   /**< SPI0_CLK_SLAVE pinmux function */
#define BEE_SPI0_SO_SLAVE      23                   /**< SPI0_SO_SLAVE pinmux function */
#define BEE_SPI0_SI_SLAVE      24                   /**< SPI0_SI_SLAVE pinmux function */
#define BEE_SPI0_SS_N_0_MASTER 25                   /**< SPI0_SS_N_0_MASTER pinmux function */
#define BEE_SPI0_SS_N_1_MASTER 26                   /**< SPI0_SS_N_1_MASTER pinmux function */
#define BEE_SPI0_SS_N_2_MASTER 27                   /**< SPI0_SS_N_2_MASTER pinmux function */
#define BEE_SPI0_CLK_MASTER    28                   /**< SPI0_CLK_MASTER pinmux function */
#define BEE_SPI0_MO_MASTER     29                   /**< SPI0_MO_MASTER pinmux function */
#define BEE_SPI0_MI_MASTER     30                   /**< SPI0_MI_MASTER pinmux function */
#define BEE_SPI1_SS_N_0_MASTER 31                   /**< SPI1_SS_N_0_MASTER pinmux function */
#define BEE_SPI1_SS_N_1_MASTER 32                   /**< SPI1_SS_N_1_MASTER pinmux function */
#define BEE_SPI1_SS_N_2_MASTER 33                   /**< SPI1_SS_N_2_MASTER pinmux function */
#define BEE_SPI1_CLK_MASTER    34                   /**< SPI1_CLK_MASTER pinmux function */
#define BEE_SPI1_MO_MASTER     35                   /**< SPI1_MO_MASTER pinmux function */
#define BEE_SPI1_MI_MASTER     36                   /**< SPI1_MI_MASTER pinmux function */
#define BEE_SPI3W_DATA_MASTER  37                   /**< SPI3W_DATA_MASTER pinmux function */
#define BEE_SPI3W_CLK_MASTER   38                   /**< SPI3W_CLK_MASTER pinmux function */
#define BEE_SPI3W_CS_MASTER    39                   /**< SPI3W_CS_MASTER pinmux function */
#define BEE_SPI3W_QB_MASTER    40                   /**< SPI3W_QB_MASTER pinmux function */
#define BEE_PWM0               41                   /**< PWM0 pinmux function */
#define BEE_PWM1               42                   /**< PWM1 pinmux function */
#define BEE_PWM2               43                   /**< PWM2 pinmux function */
#define BEE_PWM3               44                   /**< PWM3 pinmux function */
#define BEE_PWM4               45                   /**< PWM4 pinmux function */
#define BEE_PWM5               46                   /**< PWM5 pinmux function */
#define BEE_PWM6_P             47                   /**< PWM6_P pinmux function */
#define BEE_PWM6_N             48                   /**< PWM6_N pinmux function */
#define BEE_PWM7_P             49                   /**< PWM7_P pinmux function */
#define BEE_PWM7_N             50                   /**< PWM7_N pinmux function */
#define BEE_PWM8_P             51                   /**< PWM8_P pinmux function */
#define BEE_PWM8_N             52                   /**< PWM8_N pinmux function */
#define BEE_PWM9_P             53                   /**< PWM9_P pinmux function */
#define BEE_PWM9_N             54                   /**< PWM9_N pinmux function */
#define BEE_IRDA_TX            55                   /**< IRDA_TX pinmux function */
#define BEE_IRDA_RX            56                   /**< IRDA_RX pinmux function */
#define BEE_CAN_TX             57                   /**< CAN_TX pinmux function */
#define BEE_CAN_RX             58                   /**< CAN_RX pinmux function */
#define BEE_SWD_CLK            59                   /**< SWD_CLK pinmux function */
#define BEE_SWD_DIO            60                   /**< SWD_DIO pinmux function */
#define BEE_DWGPIO             90                   /**< DWGPIO pinmux function */
#define BEE_SPI2_SS_N_0_MASTER 91                   /**< SPI2_SS_N_0_MASTER pinmux function */
#define BEE_SPI2_CLK_MASTER    92                   /**< SPI2_CLK_MASTER pinmux function */
#define BEE_SPI2_MO_MASTER     93                   /**< SPI2_MO_MASTER pinmux function */
#define BEE_SPI2_MI_MASTER     94                   /**< SPI2_MI_MASTER pinmux function */
#define BEE_DIG_DEBUG          95                   /**< DIG_DEBUG pinmux function */
#define BEE_DMIC1_CLK          96                   /**< DMIC1_CLK pinmux function */
#define BEE_DMIC1_DAT          97                   /**< DMIC1_DAT pinmux function */
#define BEE_LRC_I_CODEC_SLAVE  98                   /**< LRC_I_CODEC_SLAVE pinmux function */
#define BEE_BCLK_I_CODEC_SLAVE 99                   /**< BCLK_I_CODEC_SLAVE pinmux function */
#define BEE_SDI_CODEC_SLAVE    100                  /**< SDI_CODEC_SLAVE pinmux function */
#define BEE_SDO_CODEC_SLAVE    101                  /**< SDO_CODEC_SLAVE pinmux function */
#define BEE_BT_COEX_I_0        106                  /**< BT_COEX_I_0 pinmux function */
#define BEE_BT_COEX_I_1        107                  /**< BT_COEX_I_1 pinmux function */
#define BEE_BT_COEX_I_2        108                  /**< BT_COEX_I_2 pinmux function */
#define BEE_BT_COEX_I_3        109                  /**< BT_COEX_I_3 pinmux function */
#define BEE_BT_COEX_O_0        110                  /**< BT_COEX_O_0 pinmux function */
#define BEE_BT_COEX_O_1        111                  /**< BT_COEX_O_1 pinmux function */
#define BEE_BT_COEX_O_2        112                  /**< BT_COEX_O_2 pinmux function */
#define BEE_BT_COEX_O_3        113                  /**< BT_COEX_O_3 pinmux function */
#define BEE_PTA_I2C_CLK_SLAVE  114                  /**< PTA_I2C_CLK_SLAVE pinmux function */
#define BEE_PTA_I2C_DAT_SLAVE  115                  /**< PTA_I2C_DAT_SLAVE pinmux function */
#define BEE_PTA_I2C_INT_OUT    116                  /**< PTA_I2C_INT_OUT pinmux function */
#define BEE_EN_EXPA            117                  /**< EN_EXPA pinmux function */
#define BEE_EN_EXLNA           118                  /**< EN_EXLNA pinmux function */
#define BEE_TRACECLKIN         119                  /**< TRACECLKIN pinmux function */
#define BEE_TRACESWO           120                  /**< TRACESWO pinmux function */
#define BEE_LRC_SPORT0         123                  /**< LRC_SPORT0 pinmux function */
#define BEE_BCLK_SPORT0        124                  /**< BCLK_SPORT0 pinmux function */
#define BEE_ADCDAT_SPORT0      125                  /**< ADCDAT_SPORT0 pinmux function */
#define BEE_DACDAT_SPORT0      126                  /**< DACDAT_SPORT0 pinmux function */
#define BEE_MCLK               127                  /**< MCLK pinmux function */
#define BEE_KEY_COL_0          128                  /**< KEY_COL_0 pinmux function */
#define BEE_KEY_COL_1          129                  /**< KEY_COL_1 pinmux function */
#define BEE_KEY_COL_2          130                  /**< KEY_COL_2 pinmux function */
#define BEE_KEY_COL_3          131                  /**< KEY_COL_3 pinmux function */
#define BEE_KEY_COL_4          132                  /**< KEY_COL_4 pinmux function */
#define BEE_KEY_COL_5          133                  /**< KEY_COL_5 pinmux function */
#define BEE_KEY_COL_6          134                  /**< KEY_COL_6 pinmux function */
#define BEE_KEY_COL_7          135                  /**< KEY_COL_7 pinmux function */
#define BEE_KEY_COL_8          136                  /**< KEY_COL_8 pinmux function */
#define BEE_KEY_COL_9          137                  /**< KEY_COL_9 pinmux function */
#define BEE_KEY_COL_10         138                  /**< KEY_COL_10 pinmux function */
#define BEE_KEY_COL_11         139                  /**< KEY_COL_11 pinmux function */
#define BEE_KEY_COL_12         140                  /**< KEY_COL_12 pinmux function */
#define BEE_KEY_COL_13         141                  /**< KEY_COL_13 pinmux function */
#define BEE_KEY_COL_14         142                  /**< KEY_COL_14 pinmux function */
#define BEE_KEY_COL_15         143                  /**< KEY_COL_15 pinmux function */
#define BEE_KEY_COL_16         144                  /**< KEY_COL_16 pinmux function */
#define BEE_KEY_COL_17         145                  /**< KEY_COL_17 pinmux function */
#define BEE_KEY_COL_18         146                  /**< KEY_COL_18 pinmux function */
#define BEE_KEY_COL_19         147                  /**< KEY_COL_19 pinmux function */
#define BEE_KEY_ROW_0          148                  /**< KEY_ROW_0 pinmux function */
#define BEE_KEY_ROW_1          149                  /**< KEY_ROW_1 pinmux function */
#define BEE_KEY_ROW_2          150                  /**< KEY_ROW_2 pinmux function */
#define BEE_KEY_ROW_3          151                  /**< KEY_ROW_3 pinmux function */
#define BEE_KEY_ROW_4          152                  /**< KEY_ROW_4 pinmux function */
#define BEE_KEY_ROW_5          153                  /**< KEY_ROW_5 pinmux function */
#define BEE_KEY_ROW_6          154                  /**< KEY_ROW_6 pinmux function */
#define BEE_KEY_ROW_7          155                  /**< KEY_ROW_7 pinmux function */
#define BEE_KEY_ROW_8          156                  /**< KEY_ROW_8 pinmux function */
#define BEE_KEY_ROW_9          157                  /**< KEY_ROW_9 pinmux function */
#define BEE_KEY_ROW_10         158                  /**< KEY_ROW_10 pinmux function */
#define BEE_KEY_ROW_11         159                  /**< KEY_ROW_11 pinmux function */
#define BEE_KEY_ROW_12         160                  /**< KEY_ROW_12 pinmux function */
#define BEE_KEY_ROW_13         161                  /**< KEY_ROW_13 pinmux function */
#define BEE_KEY_ROW_14         162                  /**< KEY_ROW_14 pinmux function */
#define BEE_KEY_ROW_15         163                  /**< KEY_ROW_15 pinmux function */
#define BEE_KEY_ROW_16         164                  /**< KEY_ROW_16 pinmux function */
#define BEE_KEY_ROW_17         165                  /**< KEY_ROW_17 pinmux function */
#define BEE_PINMUX_MAX         (BEE_DIG_DEBUG + 1)  /**< PINMUX_MAX pinmux function */
#define BEE_SW_MODE            (BEE_PINMUX_MAX + 1) /**< SW_MODE pinmux function */
#define BEE_PWR_OFF            (BEE_PINMUX_MAX + 2) /**< PWR_OFF pinmux function */
#define BEE_LPPWM_CH0          (BEE_PWR_OFF + 1)
#define BEE_LPQDEC_LED         (BEE_PWR_OFF + 2)
#define BEE_LPQDEC_PHB         (BEE_PWR_OFF + 3)
#define BEE_LPQDEC_PHA         (BEE_PWR_OFF + 4)
#define BEE_PIN_DISCONNECTED   BEE_PIN_MSK /**< PIN_DISCONNECTED pinmux function */
/** @} */

/**
 * @name Pinctrl Available Pins
 * @{
 */
/* Note: P0_0/P0_1 default to SWD function for rtl87x2j. */
#define P0_0        0 /*!< GPIO_0 */
#define P0_1        1 /*!< GPIO_1 */
#define P0_2        2 /*!< GPIO_2 */
/* Note: P0_3 defaults to outputting the Realtek internal log for rtl87x2j. */
#define P0_3        3 /*!< GPIO_3 */
#define P0_4        4 /*!< GPIO_4 */
#define P0_5        5 /*!< GPIO_5 */
#define P0_6        6 /*!< GPIO_6 */
#define P0_7        7 /*!< GPIO_7 */
/* Note: P1_0/P1_1 default to SWD function for rtl87x2j. */
#define P1_0        8    /*!< GPIO_8 */
#define P1_1        9    /*!< GPIO_9 */
#define P1_6        10   /*!< GPIO_10 */
#define P1_7        11   /*!< GPIO_11 */
#define P2_0        12   /*!< GPIO_12 */
#define P2_1        13   /*!< GPIO_13 */
#define P2_2        14   /*!< GPIO_14 */
#define P2_3        15   /*!< GPIO_15 */
#define P2_4        16   /*!< GPIO_16 */
#define P2_5        17   /*!< GPIO_17 */
#define P2_6        18   /*!< GPIO_18 */
#define P2_7        19   /*!< GPIO_19 */
#define P3_0        20   /*!< GPIO_20 */
#define P3_1        21   /*!< GPIO_21 */
#define P3_2        22   /*!< GPIO_22 */
#define P3_3        23   /*!< GPIO_23 */
#define P3_4        24   /*!< GPIO_24 */
#define P3_5        25   /*!< GPIO_25 */
#define P3_6        26   /*!< GPIO_26 */
#define P4_0        27   /*!< GPIO_27 */
#define P4_1        28   /*!< GPIO_28 */
#define P4_2        29   /*!< GPIO_29 */
#define P4_3        30   /*!< GPIO_30 */
#define MICBIAS     31   /*!< GPIO_31 */
#define P5_1        32   /*!< GPIO_32 */
#define P5_2        33   /*!< GPIO_33 */
#define P6_0        34   /*!< GPIO_34 */
#define P6_1        35   /*!< GPIO_35 */
#define P6_2        36   /*!< GPIO_36 */
#define P6_3        37   /*!< GPIO_37 */
#define P6_4        38   /*!< GPIO_38 */
#define P6_5        39   /*!< GPIO_39 */
#define P6_6        40   /*!< GPIO_40 */
#define P6_7        41   /*!< GPIO_41 */
#define P7_0        42   /*!< GPIO_42 */
#define P7_1        43   /*!< GPIO_43 */
#define SPIC_CSN    44   /*!< GPIO_44 */
#define SPIC_SCK    45   /*!< GPIO_45 */
#define SPIC_SI     46   /*!< GPIO_46 */
#define SPIC_SO     47   /*!< GPIO_47 */
#define SPIC_WEN    48   /*!< GPIO_48 P1_4 */
#define SPIC_HOLDEN 49   /*!< GPIO_49 P1_3 */
#define ADC_0       P2_0 /*!< GPIO_12 */
#define ADC_1       P2_1 /*!< GPIO_13 */
#define ADC_2       P2_2 /*!< GPIO_14 */
#define ADC_3       P2_3 /*!< GPIO_15 */
#define ADC_4       P2_4 /*!< GPIO_16 */
#define ADC_5       P2_5 /*!< GPIO_17 */
#define ADC_6       P2_6 /*!< GPIO_18 */
#define ADC_7       P2_7 /*!< GPIO_19 */
/** @} */

/**
 * @name GPIO Pinctrl Configuration
 * @{
 */
/* Port 0 */
#define BEE_PSEL_GPIOA_0_P0_0 BEE_PSEL(DWGPIO, P0_0) /**< GPIOA_0 for P0_0 */
#define BEE_PSEL_GPIOA_1_P0_1 BEE_PSEL(DWGPIO, P0_1) /**< GPIOA_1 for P0_1 */
#define BEE_PSEL_GPIOA_2_P0_2 BEE_PSEL(DWGPIO, P0_2) /**< GPIOA_2 for P0_2 */
/* Note: P0_3 defaults to outputting the Realtek internal log for rtl87x2j. */
#define BEE_PSEL_GPIOA_3_P0_3 BEE_PSEL(DWGPIO, P0_3) /**< GPIOA_3 for P0_3 */
#define BEE_PSEL_GPIOA_4_P0_4 BEE_PSEL(DWGPIO, P0_4) /**< GPIOA_4 for P0_4 */
#define BEE_PSEL_GPIOA_5_P0_5 BEE_PSEL(DWGPIO, P0_5) /**< GPIOA_5 for P0_5 */
#define BEE_PSEL_GPIOA_6_P0_6 BEE_PSEL(DWGPIO, P0_6) /**< GPIOA_6 for P0_6 */
#define BEE_PSEL_GPIOA_7_P0_7 BEE_PSEL(DWGPIO, P0_7) /**< GPIOA_7 for P0_7 */

/* Port 1 */
/* Note: P1_0/P1_1 default to SWD function for rtl87x2j. */
#define BEE_PSEL_GPIOA_8_P1_0  BEE_PSEL(DWGPIO, P1_0) /**< GPIOA_8 for P1_0 */
#define BEE_PSEL_GPIOA_9_P1_1  BEE_PSEL(DWGPIO, P1_1) /**< GPIOA_9 for P1_1 */
#define BEE_PSEL_GPIOA_10_P1_6 BEE_PSEL(DWGPIO, P1_6) /**< GPIOA_10 for P1_6 */
#define BEE_PSEL_GPIOA_11_P1_7 BEE_PSEL(DWGPIO, P1_7) /**< GPIOA_11 for P1_7 */

/* Port 2 */
#define BEE_PSEL_GPIOA_12_P2_0 BEE_PSEL(DWGPIO, P2_0) /**< GPIOA_12 for P2_0 */
#define BEE_PSEL_GPIOA_13_P2_1 BEE_PSEL(DWGPIO, P2_1) /**< GPIOA_13 for P2_1 */
#define BEE_PSEL_GPIOA_14_P2_2 BEE_PSEL(DWGPIO, P2_2) /**< GPIOA_14 for P2_2 */
#define BEE_PSEL_GPIOA_15_P2_3 BEE_PSEL(DWGPIO, P2_3) /**< GPIOA_15 for P2_3 */
#define BEE_PSEL_GPIOA_16_P2_4 BEE_PSEL(DWGPIO, P2_4) /**< GPIOA_16 for P2_4 */
#define BEE_PSEL_GPIOA_17_P2_5 BEE_PSEL(DWGPIO, P2_5) /**< GPIOA_17 for P2_5 */
#define BEE_PSEL_GPIOA_18_P2_6 BEE_PSEL(DWGPIO, P2_6) /**< GPIOA_18 for P2_6 */
#define BEE_PSEL_GPIOA_19_P2_7 BEE_PSEL(DWGPIO, P2_7) /**< GPIOA_19 for P2_7 */

/* Port 3 */
#define BEE_PSEL_GPIOA_20_P3_0 BEE_PSEL(DWGPIO, P3_0) /**< GPIOA_20 for P3_0 */
#define BEE_PSEL_GPIOA_21_P3_1 BEE_PSEL(DWGPIO, P3_1) /**< GPIOA_21 for P3_1 */
#define BEE_PSEL_GPIOA_22_P3_2 BEE_PSEL(DWGPIO, P3_2) /**< GPIOA_22 for P3_2 */
#define BEE_PSEL_GPIOA_23_P3_3 BEE_PSEL(DWGPIO, P3_3) /**< GPIOA_23 for P3_3 */
#define BEE_PSEL_GPIOA_24_P3_4 BEE_PSEL(DWGPIO, P3_4) /**< GPIOA_24 for P3_4 */
#define BEE_PSEL_GPIOA_25_P3_5 BEE_PSEL(DWGPIO, P3_5) /**< GPIOA_25 for P3_5 */
#define BEE_PSEL_GPIOA_26_P3_6 BEE_PSEL(DWGPIO, P3_6) /**< GPIOA_26 for P3_6 */

/* Port 4 */
#define BEE_PSEL_GPIOA_27_P4_0 BEE_PSEL(DWGPIO, P4_0) /**< GPIOA_27 for P4_0 */
#define BEE_PSEL_GPIOA_28_P4_1 BEE_PSEL(DWGPIO, P4_1) /**< GPIOA_28 for P4_1 */
#define BEE_PSEL_GPIOA_29_P4_2 BEE_PSEL(DWGPIO, P4_2) /**< GPIOA_29 for P4_2 */
#define BEE_PSEL_GPIOA_30_P4_3 BEE_PSEL(DWGPIO, P4_3) /**< GPIOA_30 for P4_3 */

/* Port 5 */
#define BEE_PSEL_GPIOB_0_P5_1 BEE_PSEL(DWGPIO, P5_1) /**< GPIOB_0 for P5_1 */
#define BEE_PSEL_GPIOB_1_P5_2 BEE_PSEL(DWGPIO, P5_2) /**< GPIOB_1 for P5_2 */

/* Port 6 */
#define BEE_PSEL_GPIOB_2_P6_0 BEE_PSEL(DWGPIO, P6_0) /**< GPIOB_2 for P6_0 */
#define BEE_PSEL_GPIOB_3_P6_1 BEE_PSEL(DWGPIO, P6_1) /**< GPIOB_3 for P6_1 */
#define BEE_PSEL_GPIOB_4_P6_2 BEE_PSEL(DWGPIO, P6_2) /**< GPIOB_4 for P6_2 */
#define BEE_PSEL_GPIOB_5_P6_3 BEE_PSEL(DWGPIO, P6_3) /**< GPIOB_5 for P6_3 */
#define BEE_PSEL_GPIOB_6_P6_4 BEE_PSEL(DWGPIO, P6_4) /**< GPIOB_6 for P6_4 */
#define BEE_PSEL_GPIOB_7_P6_5 BEE_PSEL(DWGPIO, P6_5) /**< GPIOB_7 for P6_5 */
#define BEE_PSEL_GPIOB_8_P6_6 BEE_PSEL(DWGPIO, P6_6) /**< GPIOB_8 for P6_6 */
#define BEE_PSEL_GPIOB_9_P6_7 BEE_PSEL(DWGPIO, P6_7) /**< GPIOB_9 for P6_7 */

/* Port 7 */
#define BEE_PSEL_GPIOB_10_P7_0 BEE_PSEL(DWGPIO, P7_0) /**< GPIOB_10 for P7_0 */
#define BEE_PSEL_GPIOB_11_P7_1 BEE_PSEL(DWGPIO, P7_1) /**< GPIOB_11 for P7_1 */

/* SPI Flash */
#define BEE_PSEL_GPIOB_12_SPIC_CSN    BEE_PSEL(DWGPIO, SPIC_CSN)    /**< GPIOB_12 for SPIC_CSN */
#define BEE_PSEL_GPIOB_13_SPIC_SCK    BEE_PSEL(DWGPIO, SPIC_SCK)    /**< GPIOB_13 for SPIC_SCK */
#define BEE_PSEL_GPIOB_14_SPIC_SI     BEE_PSEL(DWGPIO, SPIC_SI)     /**< GPIOB_14 for SPIC_SI */
#define BEE_PSEL_GPIOB_15_SPIC_SO     BEE_PSEL(DWGPIO, SPIC_SO)     /**< GPIOB_15 for SPIC_SO */
#define BEE_PSEL_GPIOB_16_SPIC_WEN    BEE_PSEL(DWGPIO, SPIC_WEN)    /**< GPIOB_16 for SPIC_WEN */
/**< GPIOB_17 for SPIC_HOLDEN */
#define BEE_PSEL_GPIOB_17_SPIC_HOLDEN BEE_PSEL(DWGPIO, SPIC_HOLDEN)

/* Special Functions */
#define BEE_PSEL_GPIOA_31_MICBIAS BEE_PSEL(DWGPIO, MICBIAS) /**< GPIOA_31 for MICBIAS */

/** @} */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_RTL87X2J_PINCTRL_H_ */
