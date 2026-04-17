/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_RTL87X2J_CLOCKS_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_RTL87X2J_CLOCKS_H_

/**
 * @file
 * @brief Realtek RTL87x2J Clock Controller Devicetree Bindings
 */

/**
 * @brief Helper macro to reference an APB peripheral clock.
 *
 * @param peri The peripheral name.
 *             The macro automatically expands to `APB_PERIPH_<peri>_CLOCK`.
 */
#define APB_CLK(peri)       APB_PERIPH_##peri##_CLOCK

/**
 * @name APB Peripheral Clock IDs
 * @brief Clock identifiers for APB peripherals.
 * @{
 */

#define APB_PERIPH_WDT_CLOCK                0U    /**< WDT clock */
#define APB_PERIPH_MODEMRFCPI_CLOCK         1U    /**< MODEMRFCPI clock */
#define APB_PERIPH_DMA_CLOCK                2U    /**< DMA clock */
#define APB_PERIPH_SPI0_CLOCK               3U    /**< SPI0 clock */
#define APB_PERIPH_SPI1_CLOCK               4U    /**< SPI1 clock */
#define APB_PERIPH_SPI2_CLOCK               5U    /**< SPI2 clock */
#define APB_PERIPH_I2S_CLOCK                6U    /**< I2S clock */
#define APB_PERIPH_TIMER0_CLOCK             7U    /**< TIMER0 clock */
#define APB_PERIPH_TIMER1_CLOCK             8U    /**< TIMER1 clock */
#define APB_PERIPH_TIMER2_CLOCK             9U    /**< TIMER2 clock */
#define APB_PERIPH_USB_CLOCK                10U   /**< USB clock */
#define APB_PERIPH_ADC_CLOCK                11U   /**< ADC clock */
#define APB_PERIPH_I2C0_CLOCK               12U   /**< I2C0 clock */
#define APB_PERIPH_I2C1_CLOCK               13U   /**< I2C1 clock */
#define APB_PERIPH_KEYSCAN_CLOCK            14U   /**< KEYSCAN clock */
#define APB_PERIPH_SPI3W_CLOCK              15U   /**< SPI3W clock */
#define APB_PERIPH_CAN_CLOCK                16U   /**< CAN clock */
#define APB_PERIPH_UART0_CLOCK              17U   /**< UART0 clock */
#define APB_PERIPH_UART1_CLOCK              18U   /**< UART1 clock */
#define APB_PERIPH_UART2_CLOCK              19U   /**< UART2 clock */
#define APB_PERIPH_CODEC_CLOCK              20U   /**< CODEC clock */
#define APB_PERIPH_TMETER_CLOCK             21U   /**< TMETER clock */
#define APB_PERIPH_UART3_CLOCK              22U   /**< UART3 clock */
#define APB_PERIPH_GPIOA_CLOCK              23U   /**< GPIOA clock */
#define APB_PERIPH_GPIOB_CLOCK              24U   /**< GPIOB clock */
#define APB_PERIPH_IR_CLOCK                 25U   /**< IR clock */
#define APB_PERIPH_LPPWM_CLOCK              26U   /**< LPPWM clock */
#define APB_PERIPH_LPQDEC_CLOCK             27U   /**< LPQDEC clock */
#define APB_PERIPH_GRTC_CLOCK               28U   /**< GRTC clock */
#define APB_PERIPH_BLUEWIZ_CLOCK            29U   /**< BLUEWIZ clock */
#define APB_PERIPH_RTC_CLOCK                30U   /**< RTC clock */
#define APB_PERIPH_LPWDT_CLOCK              31U   /**< LPWDT clock */
#define APB_PERIPH_LPC_CLOCK                32U   /**< LPC clock */

/** @} */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_RTL87X2J_CLOCKS_H_ */
