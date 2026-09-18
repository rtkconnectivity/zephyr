/*
 * Copyright (c) 2025 Realtek Semiconductor Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file rtl87x2j_btcontroller_init.h
 * @brief RTL87x2J btcontroller initialization.
 */

#ifndef RTL87X2J_BTCONTROLLER_INIT_H
#define RTL87X2J_BTCONTROLLER_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Bring up the btcontroller (lowerstack).
 *
 * Calls low_stack_init() from the btcontroller archive, which creates the
 * lowerstack tasks and initializes the link layer.
 *
 * @note Must be called exactly once. The call site is not yet decided: where
 * this belongs relative to Zephyr kernel init, and how lowerstack's mint_os
 * coexists with the Zephyr scheduler, are still open questions.
 */
void rtl87x2j_btcontroller_init(void);

#ifdef __cplusplus
}
#endif

#endif /* RTL87X2J_BTCONTROLLER_INIT_H */