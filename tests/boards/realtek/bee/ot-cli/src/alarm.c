/*
 * Copyright (c) 2024 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/device.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <string.h>
#include <inttypes.h>
#include <openthread/platform/time.h>
#include <openthread/platform/alarm-milli.h>
#include <openthread/platform/alarm-micro.h>
#include <openthread/platform/diag.h>
#include <openthread/platform/logging.h>
#include <openthread/logging.h>
#include <openthread-system.h>

#include <stdio.h>

#include "platform-zephyr.h"
#include "openthread-core-zephyr-config.h"

#include "mac_driver.h"
#include "mac_driver_mpan.h"
#include "mac_data_type.h"
#include "mac_stats.h"
#include "mac_802154_frame_parser.h"
#include "trace.h"
#include "power_manager_unit_zbmac.h"

extern uint64_t bt_clk_offset;
static bool timer_ms_fired, timer_us_fired;

//------------------------------------------------------------------------------
// Configuration / forward decls
//------------------------------------------------------------------------------

// Minimum ahead-of-time (in us) required to program HW timer; otherwise, fire immediately.
const uint32_t kMinScheduleAdvanceUs = 20;

//------------------------------------------------------------------------------
// State
//------------------------------------------------------------------------------

volatile uint64_t micro_alarm_us[MAX_PAN_NUM];
volatile uint64_t milli_alarm_us[MAX_PAN_NUM];
void micro_handler(uint8_t pan_idx);
void milli_handler(uint8_t pan_idx);

extern void (*zbmac_pm_init)(zbpm_adapter_t *padapter);
static zbpm_adapter_t zbpm_adap;
static bool zbpm_inited = false;

static void pm_init(void)
{
	if (true == zbpm_inited)
	{
		return;
	}

	zbpm_inited = true;
	mac_power_manager_init(&zbpm_adap, NULL, NULL);
}

static void zbpm_enter_pan0(void)
{
}

static void zbpm_enter_pan1(void)
{
}

extern void mac_notify_pm_wakeup(uint8_t pan_idx);
static void zbpm_exit_pan0(void)
{
    zbpm_adapter_t *padapter = &zbpm_adap;
    uint32_t curr_us;
    mac_bt_clk_t bt_clk_overflow_value =
    {
        .bt_clk_counter = 0,        /*!< [9..0] BT clock counter in 1 us unit */
        .bt_native_clk = 0x3FFFFF   /*!< [31..10] BT native clock[21:1] in unit of BT slot */
    };

    curr_us = mac_GetCurrentBTUS();
    if (curr_us < padapter->enter_time_us)
    {
        bt_clk_offset += MAX_BT_CLOCK_COUNTER;
    }
    mac_SetBTClkInt(MAC_BT_TIMER3, &bt_clk_overflow_value);
    mac_notify_pm_wakeup(0);
}

static void zbpm_exit_pan1(void)
{
    zbpm_adapter_t *padapter = &zbpm_adap;
    uint32_t curr_us;
    mac_bt_clk_t bt_clk_overflow_value =
    {
        .bt_clk_counter = 0,        /*!< [9..0] BT clock counter in 1 us unit */
        .bt_native_clk = 0x3FFFFF   /*!< [31..10] BT native clock[21:1] in unit of BT slot */
    };

    curr_us = mac_GetCurrentBTUS();
    if (curr_us < padapter->enter_time_us)
    {
        bt_clk_offset += MAX_BT_CLOCK_COUNTER;
    }
    mac_SetBTClkInt(MAC_BT_TIMER3, &bt_clk_overflow_value);
    mac_notify_pm_wakeup(1);
}

void mac_report_pm_wakeup(uint8_t pan_idx)
{
    micro_handler(pan_idx);
    milli_handler(pan_idx);
}

void BEE_SleepProcess(uint8_t pan_idx)
{
#if defined(CONFIG_OPENTHREAD_MTD)
    volatile zbpm_adapter_t *padapter = &zbpm_adap;
    uint64_t now;
    uint32_t now_btus;
    uint32_t s;
    uint32_t diff;

    if (false == zbpm_inited)
    {
        return;
    }

    s = irq_lock();
    now_btus = mac_GetCurrentBTUS();
    now = bt_clk_offset + now_btus;

    // Get the next valid (non-zero) alarm time among milli, micro, and tx_backoff
    uint64_t tmp_ms = (milli_alarm_us[pan_idx] > now) ? milli_alarm_us[pan_idx] : 0;
    uint64_t tmp_us = (micro_alarm_us[pan_idx] > now) ? micro_alarm_us[pan_idx] : 0;

    /*if (!tmp_ms && !tmp_us)
    {
        os_unlock(s);
        goto stay_awake;
    }*/

    // Default: use micro alarm if available
    diff = tmp_us ? tmp_us - now : UINT32_MAX;

    // If milli alarm is valid and triggers at least 1 ms earlier than micro alarm, use milli alarm instead
    if (tmp_ms && (!tmp_us || tmp_us > tmp_ms + 1000))
    {
        diff = tmp_ms - now;
    }

    if (diff < 10000)
    {
        irq_unlock(s);
        goto stay_awake;
    }

    if (diff > ((MAX_BT_CLOCK_COUNTER >> 1) - 1))
    {
        diff = (MAX_BT_CLOCK_COUNTER >> 1) - 1;
    }

    padapter->cfg.wake_interval_en = 0;
    padapter->cfg.stage_time_learned = 0;
    padapter->wakeup_time_us = now_btus + diff;
    padapter->wakeup_interval_us = 0;

    padapter->enter_callback = (pan_idx == 0) ? zbpm_enter_pan0 : zbpm_enter_pan1;
    padapter->exit_callback = (pan_idx == 0) ? zbpm_exit_pan0 : zbpm_exit_pan1;

    padapter->wakeup_reason = ZBMAC_PM_WAKEUP_UNKNOWN;
    padapter->error_code = ZBMAC_PM_ERROR_UNKNOWN;
    padapter->power_mode = ZBMAC_DEEP_SLEEP;
    //otLogNotePlat("sleep %u us tmp_us %llu", diff, tmp_us);

    irq_unlock(s);
    return;

stay_awake:
    s = irq_lock();
    if (padapter->power_mode == ZBMAC_DEEP_SLEEP)
    {
        padapter->power_mode = ZBMAC_ACTIVE;
    }
    irq_unlock(s);
#endif
}

//------------------------------------------------------------------------------
// Fast 64-bit arithmetic helpers (avoid __aeabi_uldiv / __aeabi_lmul in Flash)
//------------------------------------------------------------------------------

/**
 * 64-bit divide by 1000 using only HW 32-bit UDIV instructions.
 * Splits into three 32-bit divisions via long-division with 16-bit digits.
 */
static inline uint64_t div_1000(uint64_t x)
{
    uint32_t lo = (uint32_t)x;
    uint32_t hi = (uint32_t)(x >> 32);

    if (hi == 0)
    {
        return lo / 1000;
    }

    uint32_t q_hi = hi / 1000;
    uint32_t r_hi = hi % 1000;

    uint32_t mid = (r_hi << 16) | (lo >> 16);
    uint32_t q_mid = mid / 1000;
    uint32_t r_mid = mid % 1000;

    uint32_t bot = (r_mid << 16) | (lo & 0xFFFF);
    uint32_t q_lo = bot / 1000;

    return ((uint64_t)q_hi << 32) | ((uint64_t)q_mid << 16) | q_lo;
}

//------------------------------------------------------------------------------
// Common helpers
//------------------------------------------------------------------------------

typedef enum
{
    kAlarmMicro = 0,
    kAlarmMilli = 1,
} AlarmKind;

/**
 * Select HW timer ID based on PAN index and alarm kind.
 */
static inline uint32_t SelectTimerId(uint8_t pan_idx, AlarmKind kind)
{
    return (kind == kAlarmMicro) ? MAC_BT_TIMER1 : MAC_BT_TIMER0;
}

/**
 * Fire the corresponding alarm event immediately.
 */
static inline void FireAlarmNow(AlarmKind kind, uint8_t pan_idx)
{
    if (kind == kAlarmMicro)
    {
        timer_us_fired = true;
    }
    else
    {
        timer_ms_fired = true;
    }
    otSysEventSignalPending();
}

/**
 * Schedule an alarm or fire immediately if too close.
 * Returns true if scheduled; false if fired immediately.
 */

static bool ScheduleOrFire(AlarmKind kind,
                           otInstance *aInstance,
                           uint8_t     pan_idx,
                           uint64_t    target_us,
                           uint64_t    now_us,
                           uint32_t    curr_hw_us)
{
    uint32_t diff = 0;

    if (target_us > now_us)
    {
        uint64_t delta = target_us - now_us;
        diff           = (delta > UINT32_MAX) ? UINT32_MAX : (uint32_t)delta;
    }

    if (diff > kMinScheduleAdvanceUs)
    {
        const uint32_t tid = SelectTimerId(pan_idx, kind);
        mac_SetBTClkUSInt(tid, curr_hw_us + diff);
        // otLogNotePlat("%s set %u us", (kind == kAlarmMicro) ? "us" : "ms", diff);
        BEE_SleepProcess(pan_idx);
        return true;
    }

    // Too close in time: fire immediately.
    FireAlarmNow(kind, pan_idx);
    return false;
}

//------------------------------------------------------------------------------
// Platform API
//------------------------------------------------------------------------------

void micro_handler(uint8_t pan_idx)
{
    // Guard: only signal if an alarm is armed for this PAN.
    if (micro_alarm_us[pan_idx] > 0)
    {
        FireAlarmNow(kAlarmMicro, pan_idx);
    }
}

uint32_t __wrap_otPlatAlarmMicroGetNow(void)
{
    // Truncate to 32-bit microseconds as per OT platform API.
    return (uint32_t)otPlatTimeGet();
}

void __wrap_otPlatAlarmMicroStartAt(otInstance *aInstance, uint32_t t0, uint32_t dt)
{
    uint8_t pan_idx = 0;

    uint32_t s       = irq_lock();
    uint32_t curr_us = mac_GetCurrentBTUS();
    uint64_t now_us  = bt_clk_offset + curr_us;

    // Convert (t0, dt) to absolute 64-bit microsecond timestamp.
    uint64_t target_us = mac_ConvertT0AndDtTo64BitTime(t0, dt, &now_us);
    micro_alarm_us[pan_idx] = target_us;

    (void)ScheduleOrFire(kAlarmMicro, aInstance, pan_idx, target_us, now_us, curr_us);
    irq_unlock(s);
}

void __wrap_otPlatAlarmMicroStop(otInstance *aInstance)
{
    uint8_t pan_idx = 0;
    OT_UNUSED_VARIABLE(aInstance);

    // Disarm: handler will ignore spurious IRQs afterwards.
    micro_alarm_us[pan_idx] = 0;
    BEE_SleepProcess(pan_idx);
}

void milli_handler(uint8_t pan_idx)
{
    if (milli_alarm_us[pan_idx] > 0)
    {
        FireAlarmNow(kAlarmMilli, pan_idx);
    }
}

uint32_t __wrap_otPlatAlarmMilliGetNow(void)
{
    // Convert microseconds to milliseconds as per API contract.
    return (uint32_t)div_1000(otPlatTimeGet());
}

void __wrap_otPlatAlarmMilliStartAt(otInstance *aInstance, uint32_t t0, uint32_t dt)
{
    uint8_t pan_idx = 0;

    uint32_t s       = irq_lock();
    uint32_t curr_us = mac_GetCurrentBTUS();
    uint64_t now_us  = bt_clk_offset + curr_us;

    // For milli alarm, original logic converts using millisecond base.
    uint64_t now_ms    = div_1000(now_us);
    uint64_t target_ms = mac_ConvertT0AndDtTo64BitTime(t0, dt, &now_ms);
    uint64_t target_us = target_ms * 1000;
    milli_alarm_us[pan_idx] = target_us;

    (void)ScheduleOrFire(kAlarmMilli, aInstance, pan_idx, target_us, now_us, curr_us);
    irq_unlock(s);
}

void __wrap_otPlatAlarmMilliStop(otInstance *aInstance)
{
    uint8_t pan_idx = 0;
    OT_UNUSED_VARIABLE(aInstance);

    milli_alarm_us[pan_idx] = 0;
    BEE_SleepProcess(pan_idx);
}

uint64_t __wrap_otPlatTimeGet(void)
{
    uint32_t s = irq_lock();
    uint64_t now = bt_clk_offset + mac_GetCurrentBTUS();
    irq_unlock(s);
    return now;
}

void __wrap_platformAlarmMilliInit(void)
{
    //DBG_DIRECT("%s %d", __func__, __LINE__);
    mpan_RegisterTimer(0, MAC_BT_TIMER0, (mac_irq_callback_t)milli_handler, 0);
    mpan_RegisterSleepHandler(BEE_SleepProcess);
    pm_init();
}

void __wrap_platformAlarmMicroInit(void)
{
    //DBG_DIRECT("%s %d", __func__, __LINE__);
    mpan_RegisterTimer(0, MAC_BT_TIMER1, (mac_irq_callback_t)micro_handler, 0);
}

void __wrap_platformAlarmMilliProcess(otInstance *aInstance)
{
    uint8_t pan_idx = 0;

    if (timer_ms_fired) {
	timer_ms_fired = false;
    milli_alarm_us[pan_idx] = 0;
	otPlatAlarmMilliFired(aInstance);
    }
}

void __wrap_platformAlarmMicroProcess(otInstance *aInstance)
{
    uint8_t pan_idx = 0;

    if (timer_us_fired) {
	timer_us_fired = false;
    micro_alarm_us[pan_idx] = 0;
	otPlatAlarmMicroFired(aInstance);
    }
}

// replace misc.c
extern void WDG_SystemReset(int wdt_mode, int reset_reason);
void __wrap_otPlatReset(otInstance *aInstance)
{
    DBG_DIRECT("%s", __func__);
    WDG_SystemReset(0, 0xff);
}

// replace logging.c
void __wrap_otPlatLog(otLogLevel aLogLevel, otLogRegion aLogRegion, const char *aFormat, ...)
{
	ARG_UNUSED(aLogRegion);
	uint8_t l_msg[116];
    va_list ap;
    va_start(ap, aFormat);
    vsnprintf(l_msg, 116, aFormat, ap);
    va_end(ap);
    DBG_DIRECT("%s", l_msg);
}
