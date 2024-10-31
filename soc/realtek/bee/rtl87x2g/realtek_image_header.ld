/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#if defined(CONFIG_REALTEK_IMAGE_HEADER)
KEEP(*(.image_header))
KEEP(*(.".image_header.*"))
__end = .;
#endif
