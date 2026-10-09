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

#if defined(CONFIG_PM_DEVICE) && !defined(CONFIG_REALTEK_BEE_HAS_PCK600)
#define BEE_TIMER_PM_STORE 1
#endif

#if defined(BEE_TIMER_PM_STORE)
/**
 * @brief Shadow copy of the timer registers, kept by the drivers.
 *
 * Both flavours share one buffer because a timer instance is driven either as a
 * basic or as an enhanced timer, never as both. On RTL8752H the enable and the
 * interrupt bits live in a block shared by all instances, so that block is part
 * of the copy as well.
 */
union bee_timer_store_reg {
	struct {
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
		TIMStoreReg_Typedef regs;
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
		TIMStoreReg_TypeDef regs;
		TIMSHAREStoreReg_TypeDef share;
#endif
	} tim;
	struct {
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
		ENHTIMStoreReg_Typedef regs;
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
		ENHTIMStoreReg_TypeDef regs;
		ENHTIMShareStoreReg_TypeDef share;
#endif
	} enhtim;
};
#endif /* BEE_TIMER_PM_STORE */

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
	 */
	void (*init)(uint32_t reg, uint8_t prescaler_idx, uint32_t top_val,
		     enum bee_timer_mode mode);

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

#if defined(BEE_TIMER_PM_STORE)
	/**
	 * @brief Take the shadow copy the registers are restored from on resume.
	 * @param reg Base address of the timer register.
	 * @param buf Shadow copy to fill in.
	 */
	void (*pm_store)(uint32_t reg, union bee_timer_store_reg *buf);

	/**
	 * @brief Restore the registers from their shadow copy.
	 * @param reg Base address of the timer register.
	 * @param buf Shadow copy taken by pm_store().
	 */
	void (*pm_restore)(uint32_t reg, union bee_timer_store_reg *buf);
#endif
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
