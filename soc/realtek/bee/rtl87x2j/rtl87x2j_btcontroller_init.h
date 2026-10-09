/*
 * Copyright (c) 2025 Realtek Semiconductor Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file rtl87x2j_btcontroller_init.h
 * @brief RTL87x2J btcontroller initialization.
 *
 * Self-contained: together with libbtcontroller.a this header is all a user
 * needs. It includes no btcontroller header, so no btcontroller include path
 * is required.
 */

#ifndef RTL87X2J_BTCONTROLLER_INIT_H
#define RTL87X2J_BTCONTROLLER_INIT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create the lowerstack tasks and initialize the link layer.
 *
 * Provided by libbtcontroller.a. Declared here rather than pulled in from
 * lowerstack's low_stack.h so that this header stays self-contained and no
 * btcontroller include directory has to be exposed.
 */
void low_stack_init(void);

/**
 * @brief Bring up the btcontroller (lowerstack).
 *
 * Calls low_stack_init() from libbtcontroller.a, which creates the lowerstack
 * tasks and initializes the link layer. In the standalone firmware image this
 * is reached via lowerstack_entry(), which additionally copies data, clears
 * BSS and parses the logical eFuse - all of which the Zephyr image already
 * does itself, so only low_stack_init() is called here.
 *
 * Defined as static inline, so libbtcontroller.a has no symbol of this name:
 * callers must include this header rather than declare the function
 * themselves. 'static' is required rather than a bare 'inline': under C99 and
 * later a bare 'inline' definition emits no out-of-line copy, so any
 * translation unit where GCC declines to inline the call (-O0, for instance)
 * is left with an undefined reference to this function.
 *
 * @note Must be called exactly once. The call site is not yet decided: where
 * this belongs relative to Zephyr kernel init, and how lowerstack's mint_os
 * coexists with the Zephyr scheduler, are still open questions.
 */
static inline void rtl87x2j_btcontroller_init(void)
{
	low_stack_init();
}

#ifdef __cplusplus
}
#endif

#endif /* RTL87X2J_BTCONTROLLER_INIT_H */
