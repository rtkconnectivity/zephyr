/*
 * Copyright(c) 2025, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for UART port on BEE family processor.
 *
 */

#ifndef ZEPHYR_DRIVERS_SERIAL_UART_BEE_H_
#define ZEPHYR_DRIVERS_SERIAL_UART_BEE_H_

#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#include <rtl_uart.h>
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#include <rtl876x_uart.h>
#endif

#ifdef CONFIG_UART_BEE_KEEP_ACTIVE_AFTER_RX_WAKEUP
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#include <rtl_pinmux.h>
#include <pm.h>
#include "power_manager_unit_platform.h"
#define BEE_PM_CHECK_PASS PM_CHECK_PASS
#define BEE_PM_CHECK_FAIL PM_CHECK_FAIL
#define BEE_PM_CHECK_RET  PMCheckResult
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#include <rtl876x_pinmux.h>
#include <dlps.h>
#define BEE_PM_CHECK_PASS PM_CHECK_PASS
#define BEE_PM_CHECK_FAIL PM_CHECK_FAIL
#define BEE_PM_CHECK_RET  PMCheckResult
extern void (*platform_pm_register_callback_func_with_priority)(void *cb_func,
								PlatformPMStage pf_pm_stage,
								int8_t priority);
#endif
#endif

#ifdef CONFIG_UART_ASYNC_API
#include <zephyr/drivers/dma.h>
#endif

struct uart_bee_config {
	UART_TypeDef *uart;
	uint16_t clkid;
	bool hw_flow_ctrl;
	const struct pinctrl_dev_config *pcfg;
#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	uart_irq_config_func_t irq_config_func;
#endif
};

/* driver data */
#ifdef CONFIG_UART_ASYNC_API
struct uart_dma_stream {
	const struct device *dma_dev;
	uint32_t dma_channel;
	struct dma_config dma_cfg;
	uint8_t priority;
	uint8_t src_addr_increment;
	uint8_t dst_addr_increment;
	int fifo_threshold;
	struct dma_block_config blk_cfg;
	uint8_t *buffer;
	size_t buffer_length;
	size_t offset;
	volatile size_t counter;
	int32_t timeout;
	struct k_work_delayable timeout_work;
	bool enabled;
};
#endif

struct uart_bee_data {
	const struct device *dev;
	struct uart_config uart_config;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
	bool tx_int_en;
	bool rx_int_en;
#endif
#ifdef CONFIG_UART_ASYNC_API
	uart_callback_t async_cb;
	void *async_user_data;
	struct uart_dma_stream dma_rx;
	struct uart_dma_stream dma_tx;
	uint8_t *rx_next_buffer;
	size_t rx_next_buffer_len;
#endif
#ifdef CONFIG_PM_DEVICE
#ifdef CONFIG_UART_BEE_KEEP_ACTIVE_AFTER_RX_WAKEUP
	BEE_PM_CHECK_RET uart_pm_check_state_idle;
#endif
	UARTStoreReg_Typedef store_buf;
#endif
};

#endif /* ZEPHYR_DRIVERS_SERIAL_UART_BEE_H_ */
