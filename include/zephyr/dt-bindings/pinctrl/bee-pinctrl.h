/*
 * Copyright(c) 2025, Realtek Semiconductor Corporation.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_BEE_PINCTRL_H_
#define ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_BEE_PINCTRL_H_

/*
 * The whole BEE pin configuration information is encoded in a 32-bit bitfield
 * organized as follows:
 *
 * - 31..16: Pin function.
 * - 15:     Reserved.
 * - 14:     Pin direction configuration.
 * - 13:     Pin output drive configuration.
 * - 12..11: Pin pull configuration.
 * - 10..0:  Pin number (combination of port and pin).
 */

/**
 * @name BEE pin configuration bit field positions and masks.
 * @{
 */

/** Position of the function field. */
#define BEE_FUN_POS   16U
/** Mask for the function field. */
#define BEE_FUN_MSK   0xFFFFU
/** Position of the direction field. */
#define BEE_DIR_POS   14U
/** Mask for the low direction field. */
#define BEE_DIR_MSK   0x1U
/** Position of the drive configuration field. */
#define BEE_DRIVE_POS 13U
/** Mask for the drive configuration field. */
#define BEE_DRIVE_MSK 0x1U
/** Position of the pull configuration field. */
#define BEE_PULL_POS  11U
/** Mask for the pull configuration field. */
#define BEE_PULL_MSK  0x3U
/** Position of the pin field. */
#define BEE_PIN_POS   0U
/** Mask for the pin field. */
#define BEE_PIN_MSK   0x7FFU

/** @} */

/**
 * @brief Utility macro to build BEE psels property entry.
 *
 * @param fun Pin function configuration (see BEE_FUNC_{name} macros).
 * @param pin Pin (0..82).
 */
#define BEE_PSEL(fun, pin, dir, drive, pull)                                                       \
	(((((pin) & BEE_PIN_MSK) << BEE_PIN_POS) | (((BEE_##fun) & BEE_FUN_MSK) << BEE_FUN_POS)) | \
	 ((((BEE_##dir) & BEE_DIR_MSK) << BEE_DIR_POS) |                                           \
	  (((BEE_##drive) & BEE_DRIVE_MSK) << BEE_DRIVE_POS) |                                     \
	  (((BEE_##pull) & BEE_PULL_MSK) << BEE_PULL_POS)))

/**
 * @brief Utility macro to build bee psels property entry when a pin is disconnected.
 *
 * This can be useful in situations where code running before Zephyr, e.g. a bootloader
 * configures pins that later needs to be disconnected.
 *
 * @param fun Pin function configuration (see BEE_FUNC_{name} macros).
 */
#define BEE_PSEL_DISCONNECTED(fun)                                                                 \
	(BEE_PIN_DISCONNECTED << BEE_PIN_POS | ((BEE_##fun & BEE_FUN_MSK) << BEE_FUN_POS))

#endif /* ZEPHYR_INCLUDE_DT_BINDINGS_PINCTRL_BEE_PINCTRL_H_ */
