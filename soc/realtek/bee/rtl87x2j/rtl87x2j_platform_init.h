/*
 * Copyright (c) 2025 Realtek Semiconductor Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file rtl87x2j_platform_init.h
 * @brief RTL87x2J platform initialization interface for Zephyr / MCUboot.
 *
 * Initialization model
 * ────────────────────────────────────────────────────────────────────────────
 * Phase 1 – rtl87x2j_platform_early_init()
 *   Called once from the MCUboot / Zephyr pre-kernel entry point (before the
 *   Zephyr scheduler starts).  Mirrors the original Realtek
 *   boot_patch_entry + SystemInit_zephyr sequence, with the following steps
 *   deliberately omitted because Zephyr / MCUboot takes ownership:
 *     - secure_boot_entry()  → MCUboot image authentication
 *     - image_entry()        → MCUboot image selection and jump
 *     - ram_init()           → Zephyr startup (.data copy, .bss zero-init)
 *     - timestamp_init()     → TIMESTAMP_IRQ registered on the Zephyr side
 *     - log subsystem init   → deferred to the Zephyr application layer
 *
 *   Step sequence in early_init:
 *     E1.  MBISR RAM repair (data + buffer SRAMs)
 *     E2.  Assert handler enable
 *     E3.  eFlash IRQ function-pointer assignment
 *     E4.  RXI300 bus-fabric init (skipped if AON IS_RXI300_DISABLE is set)
 *     E5.  ROM config parsing from OCCD flash partition
 *
 * Cold boot – rtl87x2j_platform_security_init()
 *   Called exactly once after early_init() by the first-stage boot image.
 *   Configures ROT, OTP protection, RAM power, CPU, RAP, oscillator calibration,
 *   the cold-boot PCK600/PCSM policy, schedule-plan hardware, and the boot-stage
 *   buffered-log callback.  A chain-loaded application must not call it again.
 *
 * Runtime – rtl87x2j_platform_runtime_init()
 *   Completes pre-kernel initialization after the per-image configuration has
 *   been reconstructed and, where applicable, security initialization has
 *   completed.  This phase owns image-local calibration state and callback
 *   registration.
 *
 *   Step sequence in runtime_init:
 *     E9.  FT-OTP factory trim data init
 *     E10. Active-mode clock source selection
 *     E15. Temperature conversion state reconstruction
 *     E18. Clear the first-stage buffered-log callback
 *
 * Phase 2 – rtl87x2j_platform_late_init()
 *   Called from a Zephyr SYS_INIT() late-init hook, after the kernel
 *   scheduler and memory allocator are fully operational.  Drivers here
 *   depend on Zephyr OS services (e.g. k_timer, k_heap) or need to register
 *   Zephyr interrupt handlers rather than raw ROM vectors.
 *
 *   Step sequence in late_init:
 *     L1. Wakeup-source init
 *     L2. Platform power-manager init
 *     L3. Thermal meter hardware init
 *     L4. RF PHY hardware-control block + full PHY stack init
 *     L5. Thermal tracking (TMETER_FW_IRQn handler)
 *     L6. AMU script load + measurement engine start
 *     L7. Log UART clock switched to auto-gate mode
 *     L8. Hardware timer ISR registration (TIMER0_CH0/CH1)
 */

#ifndef RTL87X2J_PLATFORM_INIT_H
#define RTL87X2J_PLATFORM_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Perform one-time cold-boot security initialization.
 *
 * Configures one-time Root-of-Trust, OTP protection, RAM power, CPU, RAP,
 * oscillator calibration, and PCK600/schedule-plan hardware state, plus the
 * first-stage buffered-log callback.  It does not establish runtime state for
 * a chain-loaded image.
 *
 * @note Call exactly once from the first-stage boot image, after
 *       rtl87x2j_platform_early_init().  Do not call it from a chain-loaded
 *       Zephyr application.
 */
void rtl87x2j_platform_security_init(void);

/**
 * @brief Pre-kernel SoC platform initialization (early stage).
 *
 * Performs all hardware and SoC initialization that must complete before the
 * Zephyr scheduler starts (or before MCUboot hands off to the Zephyr image).
 * Safe to call with interrupts disabled; does not use any OS services.
 *
 * @note Must be called exactly once, before the optional
 *       rtl87x2j_platform_security_init() and rtl87x2j_platform_runtime_init().
 */
void rtl87x2j_platform_early_init(void);

/**
 * @brief Complete pre-kernel, image-local platform initialization.
 *
 * Initializes calibration caches, current-image clock callbacks, temperature
 * conversion data, and other state owned by the current image.
 *
 * @note Call once after rtl87x2j_platform_early_init() and before
 *       rtl87x2j_platform_late_init().
 */
void rtl87x2j_platform_runtime_init(void);

/**
 * @brief Rebuild current-image PCK600 and schedule-plan runtime state.
 *
 * Call this only when a chain-loaded application uses RTK runtime power-policy
 * transitions.  Callback nodes are allocated from the current image's OS heap,
 * so MCUboot's callback graph cannot be inherited.
 */
void rtl87x2j_platform_power_runtime_init(void);

/**
 * @brief Post-kernel SoC platform initialization (late stage).
 *
 * Performs SoC driver initialization that depends on Zephyr OS services or
 * requires Zephyr-side IRQ registration.  Must be called from a
 * SYS_INIT(, APPLICATION, …) hook after the scheduler is running.
 *
 * @note Must be called after rtl87x2j_platform_early_init() has returned.
 */
void rtl87x2j_platform_late_init(void);

#ifdef __cplusplus
}
#endif

#endif /* RTL87X2J_PLATFORM_INIT_H */
