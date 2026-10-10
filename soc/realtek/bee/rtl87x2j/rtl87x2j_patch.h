/*
 * Copyright (c) 2026 Realtek Semiconductor Corporation
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef RTL87X2J_PATCH_H
#define RTL87X2J_PATCH_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Install the RTL87X2J sys_patch function-pointer overrides.
 *
 * This function mirrors the pointer-initialization part of the Realtek
 * ``sys_patch`` target. It must run before any patched ROM/platform API is
 * called. Patch data initialization is intentionally provided separately.
 */
void rtl87x2j_patch_pointer_init(void);

/**
 * @brief Initialize the mutable data used by the RTL87X2J sys_patch sources.
 *
 * Call this after the per-image ROM/OCCD configuration has been reconstructed
 * and before PMU/PHY initialization consumes the patched data.
 */
void rtl87x2j_patch_data_init(void);

#ifdef __cplusplus
}
#endif

#endif /* RTL87X2J_PATCH_H */
