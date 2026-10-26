/*
 * Copyright (c) 2025 Realtek Semiconductor Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file rtl87x2j_platform_init.h
 * @brief RTL87x2J platform initialization interface.
 *
 * Called from the Zephyr SoC hooks of the application image, in this order:
 *   soc_reset_hook()      -> rtl87x2j_platform_reset_init()
 *   soc_early_init_hook() -> rtl87x2j_platform_early_init()
 *   soc_late_init_hook()  -> rtl87x2j_platform_late_init()
 *
 * The steps Zephyr already owns are not repeated here: .data/.bss setup,
 * timestamp IRQ registration and log subsystem init.
 */

#ifndef RTL87X2J_PLATFORM_INIT_H
#define RTL87X2J_PLATFORM_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Repair defective SRAM rows (MBISR) from the eFlash NVR repair data.
 *
 * Must run before anything else relies on the data and buffer SRAMs.
 */
void rtl87x2j_platform_reset_init(void);

/**
 * @brief Pre-kernel platform initialization.
 *
 * Enables the ROM assert handler, sets up the eFlash and RXI300 drivers,
 * parses the ROM configuration from the OCCD partition, and applies the
 * root-of-trust and OTP protection settings.
 *
 * Does not use OS services; safe to call with interrupts disabled.
 */
void rtl87x2j_platform_early_init(void);

/**
 * @brief Post-kernel platform initialization.
 *
 * Applies the factory trim and clock/power settings, then brings up the
 * power manager, RF PHY, thermal tracking and AMU. Needs OS services and
 * registers its interrupt handlers through Zephyr.
 *
 * @note Call after rtl87x2j_platform_early_init().
 */
void rtl87x2j_platform_late_init(void);

#ifdef __cplusplus
}
#endif

#endif /* RTL87X2J_PLATFORM_INIT_H */
