/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief RTL87X2J DMA Devicetree Bindings
 *
 * Handshake (slot) IDs for the RTL87X2J DMA controller. Values match the
 * DMA_HANDSHAKE_* defines in the HAL (rtl_dma_def.h). Use these in the slot
 * cell together with the "config" cell macros from <dt-bindings/dma/bee-dma.h>.
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_DMA_RTL87X2J_DMA_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_DMA_RTL87X2J_DMA_H_

#include "bee-dma.h"

/* Handshake (slot) IDs, second cell of a "dmas" entry. */
#define BEE_DMA_HANDSHAKE_UART0_TX     0
#define BEE_DMA_HANDSHAKE_UART0_RX     1
#define BEE_DMA_HANDSHAKE_UART1_TX     2
#define BEE_DMA_HANDSHAKE_UART1_RX     3
#define BEE_DMA_HANDSHAKE_SPI0_TX      4
#define BEE_DMA_HANDSHAKE_SPI0_RX      5
#define BEE_DMA_HANDSHAKE_SPI1_TX      6
#define BEE_DMA_HANDSHAKE_SPI1_RX      7
#define BEE_DMA_HANDSHAKE_I2C0_TX      8
#define BEE_DMA_HANDSHAKE_I2C0_RX      9
#define BEE_DMA_HANDSHAKE_I2C1_TX      10
#define BEE_DMA_HANDSHAKE_I2C1_RX      11
#define BEE_DMA_HANDSHAKE_ADC_RX       12
#define BEE_DMA_HANDSHAKE_AES_TX       13
#define BEE_DMA_HANDSHAKE_AES_RX       14
#define BEE_DMA_HANDSHAKE_SPIC0_TX     15
#define BEE_DMA_HANDSHAKE_I2S0_TX      16
#define BEE_DMA_HANDSHAKE_I2S0_RX      17
#define BEE_DMA_HANDSHAKE_UART2_TX     18
#define BEE_DMA_HANDSHAKE_UART2_RX     19
#define BEE_DMA_HANDSHAKE_UART3_TX     20
#define BEE_DMA_HANDSHAKE_UART3_RX     21
#define BEE_DMA_HANDSHAKE_SPIC0_RX     22
#define BEE_DMA_HANDSHAKE_TIMER2_TRX   23
#define BEE_DMA_HANDSHAKE_SPI2_TX      24
#define BEE_DMA_HANDSHAKE_SPI2_RX      25
#define BEE_DMA_HANDSHAKE_IR_TX        27
#define BEE_DMA_HANDSHAKE_IR_RX        28
#define BEE_DMA_HANDSHAKE_CANBUS_RX    29
#define BEE_DMA_HANDSHAKE_SHA256_TX    30
#define BEE_DMA_HANDSHAKE_TIMER1_6_TRX 31
#define BEE_DMA_HANDSHAKE_TIMER1_7_TRX 32
#define BEE_DMA_HANDSHAKE_TIMER1_8_TRX 33

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_DMA_RTL87X2J_DMA_H_ */
