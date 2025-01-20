#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/devicetree.h>
#include <zephyr/sys/printk.h>
#include <dlps.h>

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

#define TIMER_EXPIRE_NUMBERS   200
#define THREAD_SLEEP_NUMBERS   100
#define DELAYABLE_WORK_NUMBERS 100
#define TRIGGERED_WORK_NUMBERS 100

#define SLEEP_PER_TIME 200 /*millisecond*/

#if (CONFIG_SOC_SERIES_RTL8752H)
#define power_get_statistics platform_pm_get_statistics
#endif

volatile uint64_t tick_anchor1, tick_anchor2;
volatile uint64_t delta_time;
#include "trace.h"
#include "cmsis_compiler.h"

static uint32_t kc_start_time;
static uint32_t kc_spend_time;

static inline void get_start_time_cyc2(void)
{
	kc_start_time = k_cycle_get_32();
}

static inline void get_test_duration_ms2(void)
{
	uint32_t spend_cycle = k_cycle_get_32() - kc_start_time;

	kc_spend_time = k_cyc_to_ms_ceil32(spend_cycle);
}

void test_timer_handler(struct k_timer *dummy)
{
	tick_anchor2 = k_uptime_get();
	delta_time = (tick_anchor2 - tick_anchor1);
	printk("%s, k_uptime delta %llu ms \r\n", __func__, delta_time);
	/* workaround: DBG_DIRECT print uint64_t */
	DBG_DIRECT("k_uptime %x %x", (uint32_t)(tick_anchor2>>32), (uint32_t)tick_anchor2);
	tick_anchor1 = tick_anchor2;
	get_test_duration_ms2();
	printk("%s, k_cycle delta %u ms \r\n", __func__, kc_spend_time);
	get_start_time_cyc2();

	test_timer_expire_num++;
	if (test_timer_expire_num == TIMER_EXPIRE_NUMBERS) {
		k_sem_give(&test_thread_sem);
		k_timer_stop(&test_timer);
	}
}

static void test_delayable_work_handler(struct k_work *work)
{
	test_delayable_work_num++;
	if (test_delayable_work_num == DELAYABLE_WORK_NUMBERS) {
		k_sem_give(&test_thread_sem);
	} else {
		k_work_schedule(&test_delayable_work, K_MSEC(SLEEP_PER_TIME));
	}
}

static void triggered_work_handler(struct k_work *work)
{
	test_triggered_work_num++;
	if (test_triggered_work_num == TRIGGERED_WORK_NUMBERS) {
		k_sem_give(&test_thread_sem);
	} else {
		k_work_poll_submit(&test_triggered_item.work, &test_triggered_item.event, 1,
				   K_MSEC(SLEEP_PER_TIME));
	}
}

ZTEST(timeout_wakeup, test_k_timer_wakeup)
{
	uint32_t wakeup_count_timer;

	power_get_statistics(&wakeup_count_before_test, &last_wakeup_clk, &last_sleep_clk);
	k_sem_init(&test_thread_sem, 0, UINT_MAX);

	k_timer_init(&test_timer, test_timer_handler, NULL);
	get_start_time_cyc2();
	k_timer_start(&test_timer, K_MSEC(SLEEP_PER_TIME), K_MSEC(SLEEP_PER_TIME));

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
	while (test_thread_sleep_num <= THREAD_SLEEP_NUMBERS) {
		k_msleep(SLEEP_PER_TIME);
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
	k_work_schedule(&test_delayable_work, K_MSEC(SLEEP_PER_TIME));

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
	k_work_poll_submit(&test_triggered_item.work, &test_triggered_item.event, 1,
			   K_MSEC(SLEEP_PER_TIME));
	k_sem_take(&test_thread_sem, K_FOREVER);
	power_get_statistics(&wakeup_count_after_test, &last_wakeup_clk, &last_sleep_clk);
	wakeup_count_triggered_work = wakeup_count_after_test - wakeup_count_before_test;
	TC_PRINT("wakeupCount: %d, last_wakeup_clk:%d, last_sleep_clk:%d\n",
		 wakeup_count_triggered_work, last_wakeup_clk, last_sleep_clk);
	zassert_true(wakeup_count_triggered_work <= TRIGGERED_WORK_NUMBERS + 1 &&
			     wakeup_count_triggered_work >= TRIGGERED_WORK_NUMBERS - 1,
		     "test_k_timer_wakeup failed, wakeup Count: %d\n", wakeup_count_triggered_work);
}

ZTEST_SUITE(timeout_wakeup, NULL, NULL, NULL, NULL, NULL);
