/*
 * Copyright(c) 2024, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#define DT_DRV_COMPAT realtek_rtl87x2g_trng

#include <zephyr/drivers/entropy.h>
#include <string.h>

#include <utils.h>

static int entropy_rtl87x2g_get_entropy(const struct device *dev,
                                        uint8_t *buffer,
                                        uint16_t length)
{
    ARG_UNUSED(dev);

    while (length > 0)
    {
        size_t to_copy;
        uint32_t value;

        value = platform_random(0xFFFFFFFF);

        if (length % 4 == 0)
        {
            to_copy = 4;
        }
        else
        {
            to_copy = length % 4;
        }

        memcpy(buffer, &value, to_copy);
        buffer += to_copy;
        length -= to_copy;
    }

    return 0;
}

static int entropy_rtl87x2g_trng_init(const struct device *dev)
{
    ARG_UNUSED(dev);

    return 0;
}

static const struct entropy_driver_api entropy_rtl87x2g_trng_api_funcs =
{
    .get_entropy = entropy_rtl87x2g_get_entropy,
};

DEVICE_DT_INST_DEFINE(0,
                      entropy_rtl87x2g_trng_init, NULL,
                      NULL, NULL,
                      PRE_KERNEL_1, CONFIG_ENTROPY_INIT_PRIORITY,
                      &entropy_rtl87x2g_trng_api_funcs);