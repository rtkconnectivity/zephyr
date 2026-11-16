/*
 * Copyright(c) 2026, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file bee_timer_common.h
 * @brief Realtek Bee Series Common Timer Abstraction Layer.
 */

#ifndef ZEPHYR_DRIVERS_COMMON_BEE_TIMER_H_
#define ZEPHYR_DRIVERS_COMMON_BEE_TIMER_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <stdbool.h>
#include <stdint.h>

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#include <rtl_tim.h>
#include <rtl_enh_tim.h>
#include <rtl_rcc.h>
#include <rtl_pinmux.h>
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#include <rtl876x_tim.h>
#include <rtl876x_enh_tim.h>
#include <rtl876x_rcc.h>
#include <rtl876x_nvic.h>
#include <vector_table.h>
#include <rtl876x_pinmux.h>
#elif defined(CONFIG_SOC_SERIES_RTL87X2J)
#include <rtl_timer.h>
#include <rtl_pinmux.h>
#else
#error "Unsupported Realtek Bee SoC series"
#endif

/**
 * @defgroup bee_timer_abstraction Timer Abstraction Interface
 * @brief Common structures and enums for timer operations.
 * @{
 */

/**
 * @brief Bee Timer Operation Modes.
 */
enum bee_timer_mode {
	/** Mode for Zephyr Counter driver (Timer counting down). */
	BEE_TIMER_MODE_COUNTER,
	/** Mode for Zephyr PWM driver (Pulse Width Modulation). */
	BEE_TIMER_MODE_PWM,
};

/**
 * @brief Bee PWM Output Modes.
 */
enum bee_pwm_output_mode {
	/** PWM output controlled by timer (normal PWM mode). */
	BEE_PWM_OUTPUT_MODE_TIMER,
	/** PWM output forced to low level. */
	BEE_PWM_OUTPUT_MODE_LOW,
	/** PWM output forced to high level. */
	BEE_PWM_OUTPUT_MODE_HIGH,
};

/**
 * @brief Bee PWM output configuration (RTL87X2J only).
 */
struct bee_pwm_output_cfg {
	/** Each PWM period starts with output low instead of high. */
	bool start_low;
	/** Drive the PWM output as open-drain instead of push-pull. */
	bool open_drain;
};

/**
 * @brief Bee latch count trigger edge.
 *
 * The values match the hardware trigger mode encoding on all supported SoCs.
 */
enum bee_latch_trigger_mode {
	/** Latch on rising edge. */
	BEE_LATCH_TRIGGER_RISING = 0,
	/** Latch on falling edge. */
	BEE_LATCH_TRIGGER_FALLING = 1,
	/** Latch on both edges. */
	BEE_LATCH_TRIGGER_BOTH = 2,
};

/**
 * @brief Bee GPIO latch count configuration.
 *
 * Only the latch count channel that is triggered by a GPIO pad is used: latch
 * count 0 on RTL87X2J (TIMER2_CH0 only) and RTL87X2G, latch count 2 on
 * RTL8752H. It pushes the running counter value into a FIFO when the selected
 * edge is seen on the trigger pad.
 */
struct bee_latch_cfg {
	/** Latch trigger edge. */
	enum bee_latch_trigger_mode trigger;
	/** Pad number the latch triggers on (SoC `Pin_Number` encoding). */
	uint16_t trigger_pad;
	/** Enable the hardware debounce filter on the trigger input (RTL87X2J only). */
	bool debounce_en;
	/** Debounce clock divider select (RTL87X2J only). */
	uint8_t debounce_div;
	/** Debounce count limit (RTL87X2J only). */
	uint16_t debounce_count;
};

/**
 * @brief Unified Timer Operations Structure.
 *
 * This structure holds function pointers to the specific HAL implementations.
 * Upper-layer drivers use these pointers to control the hardware.
 */
struct bee_timer_ops {
	/**
	 * @brief Initialize the timer hardware.
	 * @param reg Base address of the timer register.
	 * @param prescaler_idx Prescaler value or index (SoC dependent).
	 * @param top_val The reload/max count value (Period).
	 * @param mode Operation mode (Counter or PWM).
	 * @param pwm_cfg PWM output configuration, or NULL for the HAL defaults.
	 *                Only applied on RTL87X2J in PWM mode.
	 */
	void (*init)(uint32_t reg, uint8_t prescaler_idx, uint32_t top_val,
		     enum bee_timer_mode mode, const struct bee_pwm_output_cfg *pwm_cfg);

	/**
	 * @brief Start the timer.
	 * @param reg Base address of the timer register.
	 */
	void (*start)(uint32_t reg);

	/**
	 * @brief Stop the timer.
	 * @param reg Base address of the timer register.
	 */
	void (*stop)(uint32_t reg);

	/**
	 * @brief Get the current counter value.
	 * @param reg Base address of the timer register.
	 * @return Current counter tick value.
	 */
	uint32_t (*get_count)(uint32_t reg);

	/**
	 * @brief Get the current top (load/max) value.
	 * @param reg Base address of the timer register.
	 * @return Current top value.
	 */
	uint32_t (*get_top)(uint32_t reg);

	/**
	 * @brief Set a new top (period) value.
	 * @param reg Base address of the timer register.
	 * @param top_val New top value to set.
	 */
	void (*set_top)(uint32_t reg, uint32_t top_val);

	/**
	 * @brief Configure PWM duty cycle.
	 * @param reg Base address of the timer register.
	 * @param period_cyc Total cycle count for the period.
	 * @param pulse_cyc Active pulse width in cycles.
	 * @param inverted If true, the polarity is inverted.
	 * @return The actual PWM output mode used (may differ from timer mode if duty cycle
	 *         is 0% or 100%).
	 */
	enum bee_pwm_output_mode (*set_pwm_duty)(uint32_t reg, uint32_t period_cyc,
						 uint32_t pulse_cyc, bool inverted);

	/**
	 * @brief Enable the timer interrupt.
	 * @param reg Base address of the timer register.
	 */
	void (*int_enable)(uint32_t reg);

	/**
	 * @brief Disable the timer interrupt.
	 * @param reg Base address of the timer register.
	 */
	void (*int_disable)(uint32_t reg);

	/**
	 * @brief Clear the timer interrupt pending status.
	 * @param reg Base address of the timer register.
	 */
	void (*int_clear)(uint32_t reg);

	/**
	 * @brief Check if the timer interrupt is pending.
	 * @param reg Base address of the timer register.
	 * @return true if interrupt is pending, false otherwise.
	 */
	bool (*int_status)(uint32_t reg);

	/**
	 * @brief Configure the GPIO latch count (trigger edge, pad, debounce).
	 *
	 * The latch is left disabled; use @ref bee_timer_ops.latch_enable to start it.
	 *
	 * @param reg Base address of the timer register.
	 * @param cfg Latch configuration.
	 * @return 0 on success, -ENOTSUP if the timer or the requested option has
	 *         no latch support.
	 */
	int (*latch_configure)(uint32_t reg, const struct bee_latch_cfg *cfg);

	/**
	 * @brief Enable the GPIO latch count and its FIFO threshold interrupt.
	 * @param reg Base address of the timer register.
	 */
	void (*latch_enable)(uint32_t reg);

	/**
	 * @brief Disable the GPIO latch count and its FIFO threshold interrupt.
	 * @param reg Base address of the timer register.
	 */
	void (*latch_disable)(uint32_t reg);

	/**
	 * @brief Check if the latch FIFO threshold interrupt is pending.
	 * @param reg Base address of the timer register.
	 * @return true if interrupt is pending, false otherwise.
	 */
	bool (*latch_int_status)(uint32_t reg);

	/**
	 * @brief Clear the latch FIFO threshold interrupt pending status.
	 * @param reg Base address of the timer register.
	 */
	void (*latch_int_clear)(uint32_t reg);

	/**
	 * @brief Get the number of latched values held in the FIFO.
	 * @param reg Base address of the timer register.
	 * @return Number of values currently held in the latch FIFO.
	 */
	uint8_t (*latch_get_fifo_len)(uint32_t reg);

	/**
	 * @brief Pop one latched counter value from the FIFO.
	 * @param reg Base address of the timer register.
	 * @return The latched counter value.
	 */
	uint32_t (*latch_read_fifo)(uint32_t reg);

	/**
	 * @brief Drop all latched values and the pending latch interrupt.
	 * @param reg Base address of the timer register.
	 */
	void (*latch_clear)(uint32_t reg);
};

/**
 * @brief Retrieve the Timer Operations structure.
 *
 * This is the factory function used by drivers to get the correct function pointers.
 *
 * @param enhanced Set to true if requesting Enhanced Timer ops, false for Basic Timer.
 * @return const struct bee_timer_ops* Pointer to the operations structure, or NULL if not
 * supported.
 */
const struct bee_timer_ops *bee_timer_get_ops(bool enhanced);

/** @} */

#endif /* ZEPHYR_DRIVERS_COMMON_BEE_TIMER_H_ */
