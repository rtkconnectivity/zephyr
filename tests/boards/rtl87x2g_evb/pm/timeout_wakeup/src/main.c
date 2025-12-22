/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <pm.h>
#include <os_pm.h>
#include <os_timer.h>
#include <trace.h>
#include <power_manager_unit_platform.h>
struct triggered_test_item {
	int key;
	struct k_work_poll work;
	struct k_poll_signal signal;
	struct k_poll_event event;
};

static struct triggered_test_item test_triggered_item;
static struct k_sem test_thread_sem;
static struct k_timer test_timer;
static struct k_work_delayable test_delayable_work;
static uint32_t test_timer_expire_num;
static uint32_t test_thread_sleep_num;
static uint32_t test_delayable_work_num;
static uint32_t test_triggered_work_num;
static uint32_t wakeup_count_before_test;
static uint32_t wakeup_count_after_test;
static uint32_t last_wakeup_clk, last_sleep_clk;
static uint16_t test_exclude_timer_cnt;
void *test_exclude_timer;

#define TIMER_EXPIRE_NUMBERS   50
#define THREAD_SLEEP_NUMBERS   50
#define DELAYABLE_WORK_NUMBERS 50
#define TRIGGERED_WORK_NUMBERS 50

void test_timer_handler(struct k_timer *dummy)
{
	test_timer_expire_num++;
	if (test_timer_expire_num == TIMER_EXPIRE_NUMBERS) {
		k_sem_give(&test_thread_sem);
		k_timer_stop(&test_timer);
	}
}

void test_timer_handler_oneshot(struct k_timer *dummy)
{
	k_timer_stop(&test_timer);
	k_sem_give(&test_thread_sem);
}

static void test_delayable_work_handler(struct k_work *work)
{
	test_delayable_work_num++;
	if (test_delayable_work_num == DELAYABLE_WORK_NUMBERS) {
		k_sem_give(&test_thread_sem);
	} else {
		k_work_schedule(&test_delayable_work, K_MSEC(100));
	}
}

static void triggered_work_handler(struct k_work *work)
{
	test_triggered_work_num++;
	if (test_triggered_work_num == TRIGGERED_WORK_NUMBERS) {
		k_sem_give(&test_thread_sem);
	} else {
		k_work_poll_submit(&test_triggered_item.work, &test_triggered_item.event, 1,
				   K_MSEC(100));
	}
}

ZTEST(timeout_wakeup, test_k_timer_wakeup)
{
	uint32_t wakeup_count_timer;

	power_get_statistics(&wakeup_count_before_test, &last_wakeup_clk, &last_sleep_clk);
	k_sem_init(&test_thread_sem, 0, UINT_MAX);
	k_timer_init(&test_timer, test_timer_handler, NULL);
	k_timer_start(&test_timer, K_MSEC(100), K_MSEC(100));
	k_sem_take(&test_thread_sem, K_FOREVER);
	power_get_statistics(&wakeup_count_after_test, &last_wakeup_clk, &last_sleep_clk);
	wakeup_count_timer = wakeup_count_after_test - wakeup_count_before_test;
	TC_PRINT("wakeupCount: %d, last_wakeup_clk:%d, last_sleep_clk:%d\n", wakeup_count_timer,
		 last_wakeup_clk, last_sleep_clk);

	zassert_true(wakeup_count_timer <= TIMER_EXPIRE_NUMBERS + 1 &&
			     wakeup_count_timer >= TIMER_EXPIRE_NUMBERS - 1,
		     "test_k_timer_wakeup failed, wakeup Count:%d\n", wakeup_count_timer);
}

ZTEST(timeout_wakeup, test_k_thread_wakeup)
{
	uint32_t wakeup_count_thread;

	power_get_statistics(&wakeup_count_before_test, &last_wakeup_clk, &last_sleep_clk);
	while (test_thread_sleep_num < THREAD_SLEEP_NUMBERS) {
		k_msleep(100);
		test_thread_sleep_num++;
	}
	power_get_statistics(&wakeup_count_after_test, &last_wakeup_clk, &last_sleep_clk);
	wakeup_count_thread = wakeup_count_after_test - wakeup_count_before_test;
	TC_PRINT("wakeupCount: %d, last_wakeup_clk:%d, last_sleep_clk:%d\n", wakeup_count_thread,
		 last_wakeup_clk, last_sleep_clk);
	zassert_true(wakeup_count_thread <= THREAD_SLEEP_NUMBERS + 1 &&
			     wakeup_count_thread >= THREAD_SLEEP_NUMBERS - 1,
		     "test_k_timer_wakeup failed, wakeup Count: %d\n", wakeup_count_thread);
}

ZTEST(timeout_wakeup, test_delayable_work_wakeup)
{
	uint32_t wakeup_count_delayable_work;

	power_get_statistics(&wakeup_count_before_test, &last_wakeup_clk, &last_sleep_clk);
	k_sem_init(&test_thread_sem, 0, UINT_MAX);
	k_work_init_delayable(&test_delayable_work, test_delayable_work_handler);
	k_work_schedule(&test_delayable_work, K_MSEC(100));
	k_sem_take(&test_thread_sem, K_FOREVER);
	power_get_statistics(&wakeup_count_after_test, &last_wakeup_clk, &last_sleep_clk);
	wakeup_count_delayable_work = wakeup_count_after_test - wakeup_count_before_test;

	TC_PRINT("wakeupCount: %d, last_wakeup_clk:%d, last_sleep_clk:%d\n",
		 wakeup_count_delayable_work, last_wakeup_clk, last_sleep_clk);

	zassert_true(wakeup_count_delayable_work <= DELAYABLE_WORK_NUMBERS + 1 &&
			     wakeup_count_delayable_work >= DELAYABLE_WORK_NUMBERS - 1,
		     "test_k_timer_wakeup failed, wakeup Count: %d\n", wakeup_count_delayable_work);
}

ZTEST(timeout_wakeup, test_triggered_work_wakeup)
{
	/* The k_work_poll_submit() interface schedules a triggered work item in response to a poll
	 * event (see Polling API),  that will call a user-defined function when a monitored
	 * resource becomes available or poll signal is raised, or a timeout occurs. This Test is
	 * testing the case that the work item is triggered by timeout occurs. We acctually not use
	 * signal&event to trigger, we use timeout to trigger.
	 */
	uint32_t wakeup_count_triggered_work;

	power_get_statistics(&wakeup_count_before_test, &last_wakeup_clk, &last_sleep_clk);
	k_sem_init(&test_thread_sem, 0, UINT_MAX);
	k_work_poll_init(&test_triggered_item.work, triggered_work_handler);
	k_poll_signal_init(&test_triggered_item.signal);
	k_poll_event_init(&test_triggered_item.event, K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY,
			  &test_triggered_item.signal);
	k_work_poll_submit(&test_triggered_item.work, &test_triggered_item.event, 1, K_MSEC(100));
	k_sem_take(&test_thread_sem, K_FOREVER);
	power_get_statistics(&wakeup_count_after_test, &last_wakeup_clk, &last_sleep_clk);
	wakeup_count_triggered_work = wakeup_count_after_test - wakeup_count_before_test;
	TC_PRINT("wakeupCount: %d, last_wakeup_clk:%d, last_sleep_clk:%d\n",
		 wakeup_count_triggered_work, last_wakeup_clk, last_sleep_clk);
	zassert_true(wakeup_count_triggered_work <= TRIGGERED_WORK_NUMBERS + 1 &&
			     wakeup_count_triggered_work >= TRIGGERED_WORK_NUMBERS - 1,
		     "test_k_timer_wakeup failed, wakeup Count: %d\n", wakeup_count_triggered_work);
}

ZTEST(timeout_wakeup, test_timing_apis)
{
	int64_t ms1, ms2, time_diff_ms;
	uint32_t cyc1, cyc2, time_diff_cyc, wakeup_count_timing_apis;

	ms1 = k_uptime_get();
	cyc1 = k_cycle_get_32();
	power_get_statistics(&wakeup_count_before_test, &last_wakeup_clk, &last_sleep_clk);
	k_msleep(3000);
	power_get_statistics(&wakeup_count_after_test, &last_wakeup_clk, &last_sleep_clk);
	ms2 = k_uptime_get();
	cyc2 = k_cycle_get_32();
	time_diff_ms = ms2 - ms1;
	time_diff_cyc = cyc2 - cyc1;
	wakeup_count_timing_apis = wakeup_count_after_test - wakeup_count_before_test;
	TC_PRINT("test_timing_apis: wakeupCount: %d, last_wakeup_clk:%d, last_sleep_clk:%d\n",
		 wakeup_count_timing_apis, last_wakeup_clk, last_sleep_clk);
	zassert_true(wakeup_count_timing_apis == 1, "test_timing_apis failed, wakeup Count: %d\n",
		     wakeup_count_timing_apis);
	TC_PRINT("sleep time, ms:%lld, cycle:%d\n", time_diff_ms, time_diff_cyc);
	zassert_true(time_diff_ms >= 3000 && time_diff_ms <= 3020,
		     "k_uptime_get is not accurated after exiting dlps!");
	zassert_true(k_cyc_to_ms_floor32(time_diff_cyc) >= 3000 &&
			     k_cyc_to_ms_floor32(time_diff_cyc) <= 3020,
		     "k_cycle_get_32 is not accurated after exiting dlps!");
}

#define MAX_RECORDS 3
int execution_order[MAX_RECORDS];
int exec_idx;

void test_exclude_timer_handler(void *dummy)
{
	test_exclude_timer_cnt = 44;
	if (exec_idx < MAX_RECORDS) {
		execution_order[exec_idx++] = 3;
	}
}

void pend_cb_before_driver_resume(void)
{
	if (exec_idx < MAX_RECORDS) {
		execution_order[exec_idx++] = 2;
	}
}

void restore_tag_cb(void)
{
	if (exec_idx < MAX_RECORDS) {
		execution_order[exec_idx++] = 1;
	}
}

void add_delay_to_trigger_exclude_timer(void)
{
	DBG_DIRECT("add delay to make systick isr triggered before driver resume!");
	DBG_DIRECT("add delay to make systick isr triggered before driver resume!");
}

ZTEST(timeout_wakeup, test_exclude_timer_execute_after_driver_resume)
{
	uint32_t wakeup_count_timer;
	bool status;

	platform_pm_register_callback_func_with_priority((void *)restore_tag_cb,
							 PLATFORM_PM_RESTORE, -2);
	platform_pm_register_callback_func_with_priority((void *)pend_cb_before_driver_resume,
							 PLATFORM_PM_PEND, -2);
	platform_pm_register_callback_func_with_priority((void *)add_delay_to_trigger_exclude_timer,
							 PLATFORM_PM_PEND, -3);
	power_get_statistics(&wakeup_count_before_test, &last_wakeup_clk, &last_sleep_clk);
	status = os_timer_create(&test_exclude_timer, "exclude_timer", 1, 80, false,
				 test_exclude_timer_handler);
	zassert_true(status != false, "error creating one-shot timer!");
	status = os_register_pm_excluded_handle(&test_exclude_timer, PLATFORM_PM_EXCLUDED_TIMER);
	zassert_true(status != false, "error register exclude timer!");

	k_sem_init(&test_thread_sem, 0, UINT_MAX);
	k_timer_init(&test_timer, test_timer_handler_oneshot, NULL);
	k_timer_start(&test_timer, K_SECONDS(1), K_SECONDS(1));
	status = os_timer_start(&test_exclude_timer);
	zassert_true(status != false, "error start exclude timer!");

	k_sem_take(&test_thread_sem, K_FOREVER);

	zassert_true(test_exclude_timer_cnt == 44,
		     "exclude timer is not executed, which is wrong!");
	zassert_true(execution_order[0] == 1 && execution_order[1] == 2 && execution_order[2] == 3,
		     "exclude timer is executed before the driver resume, which is risky! flow "
		     "is:%d,%d,%d",
		     execution_order[0], execution_order[1], execution_order[2]);
	power_get_statistics(&wakeup_count_after_test, &last_wakeup_clk, &last_sleep_clk);
	wakeup_count_timer = wakeup_count_after_test - wakeup_count_before_test;

	TC_PRINT("test_exclude_timer: wakeupCount: %d, last_wakeup_clk:%d, last_sleep_clk:%d\n",
		 wakeup_count_timer, last_wakeup_clk, last_sleep_clk);

	status = os_unregister_pm_excluded_handle(&test_exclude_timer, PLATFORM_PM_EXCLUDED_TIMER);

	zassert_true(status != false, "error unregister exclude timer!");
	zassert_true(wakeup_count_timer == 1, "test_exclude_timer failed, wakeup Count:%d\n",
		     wakeup_count_timer);
}

void teardown_fn(void *data)
{
	power_mode_pause();
}

ZTEST_SUITE(timeout_wakeup, NULL, NULL, NULL, NULL, teardown_fn);
