/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/pm/policy.h>
#include <zephyr/sys/time_units.h>
#include <zephyr/sys_clock.h>
#include <zephyr/ztest.h>
#include <zephyr/devicetree.h>

#include <aon_reg.h>
#include <pck600_snapshot.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bee_pm);

static struct k_timer periodic_timer;
static struct k_sem periodic_sem;

uint64_t last_cycle;
extern int exit_num;
static int last_exit_num;

static const uint32_t period_list_us[] = {20000, 10000, 9000, 8000, 7000, 6000, 5000, 3000, 2500};
#define PERIOD_CNT (ARRAY_SIZE(period_list_us))

#define EXPIRE_PER_PERIOD 10
#define TOTAL_EXPIRE_CNT  (PERIOD_CNT * EXPIRE_PER_PERIOD)

#define SNAPSHOT 0

struct timer_log_t {
	uint8_t expire_cnt;
	uint32_t period_us;
	uint64_t delta_cycle;
	int low_power_mode_count;
};
static struct timer_log_t log_buf[TOTAL_EXPIRE_CNT];
static int entered_dlps_times[PERIOD_CNT];

static uint8_t curr_period_idx;
static uint8_t expire_cnt_in_period;

static volatile uint16_t log_write_pos;
#if SNAPSHOT == 1
static const char *const names[] = {
	"I2S",        "ADC",        "UART3",      "UART2",      "UART1",      "UART0",
	"TMETER",     "TIMER1_CH8", "TIMER1_CH7", "TIMER1_CH6", "TIMER1_CH5", "TIMER1_CH4",
	"TIMER1_CH3", "TIMER1_CH2", "TIMER1_CH1", "TIMER1_CH0", "TIMER0_CH1", "TIMER0_CH0",
	"SPI3W",      "SPI2",       "SPI1",       "SPI0",       "KEYSCAN",    "IR",
	"I2C1",       "I2C0",       "GPIO",       "DMA",        "TIMER2_CH0", "CAN",
	"USB",        "RFC",        "PRO24G",     "PCC",        "MODEM",      "CAL80M",
	"CAL32K",     "CAL10M",     "BTMAC",      "BT_VEN",     "AUDIO",      "AACK",
	"AES",        "SHA256",     "CPU",        "SPIC0",      "RNG",        "VDREG"};

void dump_pck600_status(uint64_t mask)
{
	for (int i = 0; i < PCK600_SNAPSHOT_DEVICE_MAX; i++) {
		if (mask & (1ULL << i)) {
			printf("Module Active: %s (Bit %d)\n", names[i], i);
		}
	}
}
#endif

static void timer_period_fn(struct k_timer *t)
{
	/* DEBUG */
#if SNAPSHOT == 1
	uint64_t mask = pck600_snapshot_get_device_mask();

	printf("dump pck600 status start, mask: 0x%016llx\n", mask);
	dump_pck600_status(mask);
	printf("dump pck600 status end\n");
#endif

	uint64_t curr_cycle = k_cycle_get_64();

	uint16_t pos = log_write_pos;

	if (pos < TOTAL_EXPIRE_CNT) {
		log_buf[pos].expire_cnt = expire_cnt_in_period;
		log_buf[pos].period_us = period_list_us[curr_period_idx];
		log_buf[pos].delta_cycle = curr_cycle - last_cycle;
		log_buf[pos].low_power_mode_count = AON_REG_READ_BITFIELD(
			AON_REG_PCK600_AON_REG5X, VPON_LEAVE_FUNC_RET_MODE_CNT_VALUE);
		log_write_pos = pos + 1;
	}

	k_sem_give(&periodic_sem);
}

ZTEST(pm_rtl87x2j, test_timer)
{
	TC_PRINT("Start pm_rtl87x2j test, start cycle:%llu\n", k_cycle_get_64());

	curr_period_idx = 0;
	expire_cnt_in_period = 0;

	k_timer_init(&periodic_timer, timer_period_fn, NULL);

	for (curr_period_idx = 0; curr_period_idx < PERIOD_CNT; curr_period_idx++) {
		uint32_t next_us = period_list_us[curr_period_idx];

		last_exit_num = AON_REG_READ_BITFIELD(AON_REG_PCK600_AON_REG5X,
						      VPON_LEAVE_FUNC_RET_MODE_CNT_VALUE);

		for (expire_cnt_in_period = 1; expire_cnt_in_period <= EXPIRE_PER_PERIOD;
		     expire_cnt_in_period++) {
			last_cycle = k_cycle_get_64();
			k_timer_start(&periodic_timer, K_USEC(next_us), K_NO_WAIT);
			k_sem_take(&periodic_sem, K_FOREVER);
		}

		entered_dlps_times[curr_period_idx] =
			AON_REG_READ_BITFIELD(AON_REG_PCK600_AON_REG5X,
					      VPON_LEAVE_FUNC_RET_MODE_CNT_VALUE) -
			last_exit_num;
	}

	TC_PRINT("Test finished!\n");

	for (uint16_t i = 0; i < log_write_pos; i++) {
		const struct timer_log_t *l = &log_buf[i];

		TC_PRINT("Timer period:%u us, exp:%u, delta cycle:%llu, low_power_mode_count:%d\n",
			 l->period_us, l->expire_cnt, l->delta_cycle, l->low_power_mode_count);
	}

	for (uint16_t i = 0; i < PERIOD_CNT; i++) {
		TC_PRINT("Timer period:%u us, exp count: %d, DLPS entry count:%d\n",
			 period_list_us[i], EXPIRE_PER_PERIOD, entered_dlps_times[i]);
	}
}

void teardown_fn(void *data)
{
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, 0);
}

void before_fn(void *data)
{
	k_sem_init(&periodic_sem, 0, 1);
}

ZTEST_SUITE(bee_pm, NULL, NULL, before_fn, NULL, teardown_fn);
