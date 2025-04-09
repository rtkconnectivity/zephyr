/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/ztest.h>

#if (CONFIG_SOC_SERIES_RTL8752H)
#include <dlps.h>
#define power_get_statistics platform_pm_get_statistics
#else /* CONFIG_SOC_SERIES_RTL8752H */
#include <pm.h>
#endif

#define LE_ADV_DURATION_SECONDS 20
#define DEVICE_NAME             CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN         (sizeof(DEVICE_NAME) - 1)

static uint32_t wakeup_count_before_test;
static uint32_t wakeup_count_after_test;
static uint32_t last_wakeup_clk, last_sleep_clk;

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
};

/* Set Scan Response data */
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

ZTEST_SUITE(btmac_wakeup, NULL, NULL, NULL, NULL, NULL);

#include "trace.h"
ZTEST(btmac_wakeup, test_adv_wakeup)
{
	int err;

	printk("Starting Beacon Demo\n");
	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	zassert_equal(err, 0, "Bluetooth init failed (err %d)\n", err);

	printk("Bluetooth initialized\n");

	/* Start advertising */

	err = bt_le_adv_start(
		BT_LE_ADV_PARAM(0, BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL), ad,
		ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	/*
	 *err = bt_le_adv_start(BT_LE_ADV_NCONN, ad, ARRAY_SIZE(ad),
	 *		      sd, ARRAY_SIZE(sd));
	 */
	zassert_equal(err, 0, "Advertising failed to start (err %d)\n", err);

	printk("Advertising started\n");
	power_get_statistics(&wakeup_count_before_test, &last_wakeup_clk, &last_sleep_clk);
	k_sleep(K_SECONDS(LE_ADV_DURATION_SECONDS));
	bt_le_adv_stop();
	power_get_statistics(&wakeup_count_after_test, &last_wakeup_clk, &last_sleep_clk);
	uint32_t wakeup_count_btmac = wakeup_count_after_test - wakeup_count_before_test;

	TC_PRINT("wakeupCount: %d, last_wakeup_clk:%d, last_sleep_clk:%d\n", wakeup_count_btmac,
		 last_wakeup_clk, last_sleep_clk);
	zassert_true(wakeup_count_btmac <=
			LE_ADV_DURATION_SECONDS * 1000 / 100 &&
			wakeup_count_btmac >=
			LE_ADV_DURATION_SECONDS * 1000 / (150 + 10), /* 0~10ms random delay */
		     "failed, wakeup Count: %d\n", wakeup_count_btmac);
}
