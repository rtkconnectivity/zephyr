/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief IR public API header file.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_IR_H_
#define ZEPHYR_INCLUDE_DRIVERS_IR_H_

#include <errno.h>
#include <stddef.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

enum ir_event_type {
	IR_TX_COMPLETED,
	IR_RX_RECEIVED,
	IR_RX_STOPPED,
};

struct ir_event_tx {
	/** @brief Number of bytes sent. */
	size_t len;
};

struct ir_event_rx {
	/** @brief Pointer to current buffer. */
	uint32_t *buf;
	/** @brief Number of new bytes received. */
	size_t len;
};

struct ir_event {
	/** @brief Type of event */
	enum ir_event_type type;
	/** @brief Event data */
	union ir_event_data {
		/** @brief #IR_TX_COMPLETED events data. */
		struct ir_event_tx tx;
		/** @brief #IR_RX_RECEIVED and #IR_RX_STOPPED event data. */
		struct ir_event_rx rx;
	} data;
};

/**
 * @typedef ir_callback_t
 * @brief Define the application callback function signature for
 * set_rx_cb() function.
 *
 * @param dev IR device instance.
 * @param evt Pointer to uart_event instance.
 * @param user_data Pointer to data specified by user.
 */
typedef void (*ir_callback_t)(const struct device *dev, struct ir_event *evt, void *user_data);

__subsystem struct ir_driver_api {
	int (*set_freq)(const struct device *dev, uint32_t freq, uint8_t duty);
	/** IR tx function */
	int (*tx_enable)(const struct device *dev, ir_callback_t callback, void *user_data);
	int (*tx)(const struct device *dev, const uint32_t *buf, size_t len);
	/** IR rx function */
	int (*rx_enable)(const struct device *dev, ir_callback_t callback, void *user_data,
			 uint32_t rx_len, uint32_t idle_cnt);
	/** Stop ir rx and receive remaining data */
	int (*rx_disable)(const struct device *dev, struct ir_event_rx *data);
};

/**
 * @brief Set frequency and duty for ir transmitter or receiver.
 *
 * This routine checks if the frequency and duty are valid data.
 * When the frequency and duty are valid data, they will be set
 * to hardware and it returns 0 to the calling thread. It returns
 * -ENOTSUP, otherwise.
 *
 * @param dev IR device instance.
 * @param freq Frequency of IR carrier.
 * @param duty Duty of IR carrier.
 *
 * @retval 0  If frequency and duty are set successfully.
 * @retval -ENOTSUP If the value of frequency or duty is not supported.
 */
static inline int ir_set_freq(const struct device *dev, uint32_t freq, uint8_t duty)
{
	const struct ir_driver_api *api = (const struct ir_driver_api *)dev->api;

	if (api->set_freq != NULL) {
		return api->set_freq(dev, freq, duty);
	}

	return -ENOSYS;
}

/**
 * @brief Enable ir tx function and set the interrupt callback function pointer for ir tx.
 *
 * This sets up the callback for ir tx interrupt. When the ir tx completes,
 * the specified function will be called with specified user data.
 *
 * @param dev IR device instance.
 * @param callback Pointer to the callback function.
 * @param user_data Data to pass to callback function.
 *
 * @retval 0 On success.
 * @retval -ENOSYS If this function is not implemented.
 */
__syscall int ir_tx_enable(const struct device *dev, ir_callback_t callback, void *user_data);

static inline int z_impl_ir_tx_enable(const struct device *dev, ir_callback_t callback,
				      void *user_data)
{
	const struct ir_driver_api *api = (const struct ir_driver_api *)dev->api;

	if (api->tx_enable != NULL) {
		return api->tx_enable(dev, callback, user_data);
	}

	return -ENOSYS;
}

/**
 * @brief Write a character to the device for output.
 *
 * This routine checks if the transmitter is full.  When the
 * transmitter is not full, it writes a character to the data
 * register. It waits and blocks the calling thread, otherwise. This
 * function is a blocking call.
 *
 * This function will stop unfinished transmission and start a new transmission.
 *
 * @param dev IR device instance.
 * @param buf Data to transmit.
 * @param len Number of bytes to send.
 *
 * @retval 0 On success.
 * @retval -ENOSYS If this function is not implemented.
 */
__syscall int ir_tx(const struct device *dev, const uint32_t *buf, size_t len);

static inline int z_impl_ir_tx(const struct device *dev, const uint32_t *buf, size_t len)
{
	const struct ir_driver_api *api = (const struct ir_driver_api *)dev->api;

	if (api->tx != NULL) {
		return api->tx(dev, buf, len);
	}

	return -ENOSYS;
}

/**
 * @brief Enable ir rx function and set the interrupt callback function pointer for ir rx.
 *
 * This sets up the callback for ir rx interrupt. When the ir rx receives
 * enough data or when the ir rx stops, the specified function will be
 * called with specified user data.
 *
 * @param dev IR device instance.
 * @param callback Pointer to the callback function.
 * @param user_data Data to pass to callback function.
 * @param rx_len Receive rx data length to trigger #IR_RX_RECEIVED callback.
 * @param idle_cnt Length of level to trigger #IR_RX_STOPPED callback .
 *
 * @retval 0 On success.
 * @retval -ENOSYS If this function is not implemented.
 */
__syscall int ir_rx_enable(const struct device *dev, ir_callback_t callback, void *user_data,
			   uint32_t rx_len, uint32_t idle_cnt);

static inline int z_impl_ir_rx_enable(const struct device *dev, ir_callback_t callback,
				      void *user_data, uint32_t rx_len, uint32_t idle_cnt)
{
	const struct ir_driver_api *api = (const struct ir_driver_api *)dev->api;

	if (api->rx_enable != NULL) {
		return api->rx_enable(dev, callback, user_data, rx_len, idle_cnt);
	}

	return -ENOSYS;
}

__syscall int ir_rx_disable(const struct device *dev, struct ir_event_rx *data);

static inline int z_impl_ir_rx_disable(const struct device *dev, struct ir_event_rx *data)
{
	const struct ir_driver_api *api = (const struct ir_driver_api *)dev->api;

	if (api->rx_disable != NULL) {
		return api->rx_disable(dev, data);
	}

	return -ENOSYS;
}

#ifdef __cplusplus
}
#endif

#include <zephyr/syscalls/ir.h>

#endif /* ZEPHYR_INCLUDE_DRIVERS_IR_H_ */
