/*
 * Copyright(c) 2024, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_RTL8752H_CLOCKS_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_RTL8752H_CLOCKS_H_

/**
 * @name Register offsets
 * @{
 */

/** @} */

/**
 * @name Clock enable/disable definitions for peripherals
 * @{
 */
#define APB_CLK(peri) APBPeriph_##peri##_CLOCK

#define APBPeriph_I2S0_CLOCK     0U
#define APBPeriph_I2S1_CLOCK     1U
#define APBPeriph_CODEC_CLOCK    2U
#define APBPeriph_GPIO_CLOCK     3U
#define APBPeriph_GDMA_CLOCK     4U
#define APBPeriph_TIMER_CLOCK    5U
#define APBPeriph_ENHTIMER_CLOCK 6U
#define APBPeriph_UART2_CLOCK    7U
#define APBPeriph_UART0_CLOCK    8U
#define APBPeriph_FLASH_CLOCK    9U
#define APBPeriph_PKE_CLOCK      10U
#define APBPeriph_SHA256_CLOCK   11U
#define APBPeriph_FLASH1_CLOCK   12U
#define APBPeriph_FLH_SEC_CLOCK  13U
#define APBPeriph_IR_CLOCK       14U
#define APBPeriph_SPI1_CLOCK     15U
#define APBPeriph_SPI0_CLOCK     16U
#define APBPeriph_UART1_CLOCK    17U
#define APBPeriph_IF8080_CLOCK   18U
#define APBPeriph_ADC_CLOCK      19U
#define APBPeriph_SPI2W_CLOCK    20U
#define APBPeriph_MODEM_CLOCK    21U
#define APBPeriph_BLUEWIZ_CLOCK  22U
#define APBPeriph_ZIGBEE_CLOCK   23U
#define APBPeriph_KEYSCAN_CLOCK  24U
#define APBPeriph_QDEC_CLOCK     25U
#define APBPeriph_I2C1_CLOCK     26U
#define APBPeriph_I2C0_CLOCK     27U

/** @} */

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_CLOCK_RTL8752H_CLOCKS_H_ */
