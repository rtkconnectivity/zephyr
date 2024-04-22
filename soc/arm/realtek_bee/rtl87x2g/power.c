#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/pm/pm.h>

#include <zephyr/kernel_structs.h>
#include <zephyr/init.h>
#include <string.h>
#include <zephyr/drivers/timer/system_timer.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/state.h>
#include <zephyr/pm/policy.h>
#include <zephyr/tracing/tracing.h>

#include <cmsis_core.h>

#include <pmu_manager.h>
#include <power_manager_slave.h>
#include <utils.h>
#include <platform_rtc.h>
#include <rom_api_for_zephyr.h>
#include <pm.h>
#include <rtl_pinmux.h>
#include <trace.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(soc, CONFIG_SOC_LOG_LEVEL);

volatile PINMUXStoreReg_Typedef Pinmux_StoreReg;
extern void Pinmux_DLPSEnter(void *PeriReg, void *StoreBuf);
extern void Pinmux_DLPSExit(void *PeriReg, void *StoreBuf);

volatile uint32_t CPU_StoreReg[6];
volatile uint8_t CPU_StoreReg_IPR[96];
volatile uint32_t Peripheral_StoreReg[2];
static void CPU_DLPS_Enter(void)
{
    //NVIC store
    uint32_t i;

    CPU_StoreReg[0] = NVIC->ISER[0];
    CPU_StoreReg[1] = NVIC->ISER[1];
    CPU_StoreReg[2] = NVIC->ISER[2];

    CPU_StoreReg[3] = NVIC->ISPR[0];
    CPU_StoreReg[4] = NVIC->ISPR[1];
    CPU_StoreReg[5] = NVIC->ISPR[2];

    //     skip System_IRQn, WDG_IRQn, RXI300_IRQn, RXI300_SEC_IRQn,
    //     Zigbee_IRQn which are handled in rom
    const uint8_t *IPR_pt = (const uint8_t *)NVIC->IPR;
    for (i = 5; i < 96; ++i)
    {
        CPU_StoreReg_IPR[i] = IPR_pt[i];
    }

    // peripheral reg store
    Peripheral_StoreReg[0] = SoC_VENDOR->u_008.REG_LOW_PRI_INT_MODE;
    Peripheral_StoreReg[1] = SoC_VENDOR->u_00C.REG_LOW_PRI_INT_EN;

    return;
}

void CPU_DLPS_Exit(void)
{
    // peripheral reg restore
    SoC_VENDOR->u_008.REG_LOW_PRI_INT_MODE = Peripheral_StoreReg[0];
    SoC_VENDOR->u_00C.REG_LOW_PRI_INT_EN = Peripheral_StoreReg[1];

    //NVIC restore
    uint32_t i;

    if ((CPU_StoreReg[0] & CPU_StoreReg[3]) || (CPU_StoreReg[1] & CPU_StoreReg[4]) ||
        (CPU_StoreReg[2] & CPU_StoreReg[5]))
    {
        /* During enter and exit dlps, system will disable all interrupts. If any interrupt occurs during this period, this log will be printed.
        Every bit of pending register corresponds to an interrupt. Please refer to IRQn_Type from System_IRQn to PF_RTC_IRQn.
        For example:  "miss interrupt: pending register: 0x100, 0x0 , 0x0"
        It means that RTC interrupt occur during dlps store and restore flow. But because all interrupts are masked, these interrupts are pending.
        */
        DBG_DIRECT("miss interrupt: pending register: 0x%x, 0x%x, 0x%x", CPU_StoreReg[3],
                       CPU_StoreReg[4], CPU_StoreReg[5]);
    }

    //     skip System_IRQn, WDG_IRQn, RXI300_IRQn, RXI300_SEC_IRQn,
    //     Zigbee_IRQn which are handled in rom
    uint8_t *IPR_pt = (uint8_t *)NVIC->IPR;
    for (i = 5; i < 96; ++i)
    {
        IPR_pt[i] = CPU_StoreReg_IPR[i];
    }

    NVIC->ISER[0] = CPU_StoreReg[0];
    NVIC->ISER[1] = CPU_StoreReg[1];
    NVIC->ISER[2] = CPU_StoreReg[2];

    return;
}

#ifdef CONFIG_PM_POLICY_CUSTOM
extern void power_manager_slave_send_idle_check_request(void);
extern void power_manager_slave_receive_m2s_inact_msg(PMM2SInactivateMessage *p_pm_m2s_inact_msg);
bool power_manager_slave_inact_action_handler_zephyr(void)
{
    bool ret = false;

    if (!power_manager_slave_system.suspended)
    {
        /* 1. disable IRQ */
        __disable_irq();

        bool is_interrupt = false;

        /* 2. send PM_ACTION_IDLE_CHECK_REQUEST to master */
        power_manager_slave_send_idle_check_request();

        /* 3. continuously receive message */
        while (1)
        {
            PMM2SInactivateMessage pm_m2s_inact_msg;
            power_manager_slave_receive_m2s_inact_msg(&pm_m2s_inact_msg);

            if (!power_manager_slave_inact_msg_handler(&pm_m2s_inact_msg, &is_interrupt))
            {
                break;
            }
        }

        if (power_manager_interface_get_unit_status(PM_SLAVE_0, PM_UNIT_PLATFORM) == PM_UNIT_INACTIVE)
        {
            ret = true;
        }
        else
        {
            /* 4. enable IRQ */
            __enable_irq();
        }
    }

    return ret;
}

const struct pm_state_info *pm_policy_next_state(uint8_t cpu, int32_t ticks)
{
    ARG_UNUSED(cpu);
    ARG_UNUSED(ticks);

    static const struct pm_state_info state[] =
    {
        {.state = PM_STATE_RUNTIME_IDLE},
        {.state = PM_STATE_SUSPEND_TO_RAM, .substate_id = 0},
        {.state = PM_STATE_SUSPEND_TO_RAM, .substate_id = 1},
        {.state = PM_STATE_SOFT_OFF},
    };

    extern void (*thermal_meter_read)(void);
    thermal_meter_read();

    extern void log_buffer_trigger_schedule_in_km4_idle_task(void);
    log_buffer_trigger_schedule_in_km4_idle_task();

    if (power_manager_slave_inact_action_handler_zephyr())
    {
        switch (platform_pm_get_power_mode())
        {
        case PLATFORM_POWERDOWN:
            return &state[3];
            break;

        case PLATFORM_DLPS_PFM:
            return &state[1];
            break;

        case PLATFORM_DLPS_RET:
            return &state[2];
            break;

        default:
            break;
        }
    }

    return &state[0];
}
#endif	/* CONFIG_PM_POLICY_CUSTOM */

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
    if (state == PM_STATE_RUNTIME_IDLE)
    {
        __set_BASEPRI(0); //clear the basepri, correspond to the arch_irq_lock() in idle
        __DSB();
        return;
    }

    k_sched_unlock(); // Corresponds to the "k_sched_lock" in pm_system_suspend() function. Becasue in rtk flow, we have locked it once.

    __set_BASEPRI(0); //clear the basepri, may not need
    __DSB();

    switch (state)
    {
    case PM_STATE_SUSPEND_TO_RAM:
        switch (substate_id)
        {
        case 0:
            lop_setting(PLATFORM_DLPS_PFM);
            break;

        case 1:
            lop_setting(PLATFORM_DLPS_RET);
            break;

        default:
            break;
        }
        break;

    case PM_STATE_SOFT_OFF:
        lop_setting(PLATFORM_POWERDOWN);
        AON_REG_WRITE_BITFIELD(AON_NS_REG0X_FW_GENERAL_NS, km4_pon_boot_done, 0);
        break;

    default:
        LOG_DBG("Unsupported power state %u", state);
        break;
    }

    AON_REG2X_SYS_TYPE reg2x_sys = {.d32 = AON_REG_READ(AON_REG2X_SYS)};
    reg2x_sys.FW_enter_lps = true; // trigger AON FSM power off sequence
    AON_REG_WRITE(AON_REG2X_SYS, reg2x_sys.d32);


    while (1) {};
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
    ARG_UNUSED(state);
    ARG_UNUSED(substate_id);

}

#ifdef CONFIG_PM_DEVICE
TYPE_SECTION_START_EXTERN(const struct device *, pm_device_slots);

#if !defined(CONFIG_PM_DEVICE_RUNTIME_EXCLUSIVE)
/* Number of devices successfully suspended. */
static size_t num_susp_rtk;

static int pm_suspend_devices_rtk(void)
{
	/* low stack do it instead */
    // Pad_ClearAllWakeupINT();
    // System_WakeupDebounceClear(0);

    // NVIC_DisableIRQ(System_IRQn);
    CPU_DLPS_Enter();

    Pinmux_DLPSEnter(PINMUX, (void *)&Pinmux_StoreReg);

    const struct device *devs;
	size_t devc;

	devc = z_device_get_all_static(&devs);

	num_susp_rtk = 0;

	for (const struct device *dev = devs + devc - 1; dev >= devs; dev--) {
		int ret;

		/*
		 * Ignore uninitialized devices, busy devices, wake up sources, and
		 * devices with runtime PM enabled.
		 */
		if (!device_is_ready(dev) || pm_device_is_busy(dev) ||
		    pm_device_state_is_locked(dev) ||
		    pm_device_wakeup_is_enabled(dev) ||
		    pm_device_runtime_is_enabled(dev)) {
			continue;
		}

		ret = pm_device_action_run(dev, PM_DEVICE_ACTION_SUSPEND);
		/* ignore devices not supporting or already at the given state */
		if ((ret == -ENOSYS) || (ret == -ENOTSUP) || (ret == -EALREADY)) {
			continue;
		} else if (ret < 0) {
			LOG_ERR("Device %s did not enter %s state (%d)",
				dev->name,
				pm_device_state_str(PM_DEVICE_STATE_SUSPENDED),
				ret);
			return ret;
		}

       // pm_device_state_lock(dev);//to bypass zephyr pm device system flow

		TYPE_SECTION_START(pm_device_slots)[num_susp_rtk] = dev;
		num_susp_rtk++;
	}

	return 0;
}

static void pm_resume_devices_rtk(void)
{
	Pinmux_DLPSExit(PINMUX, (void *)&Pinmux_StoreReg);
    
    for (int i = (num_susp_rtk - 1); i >= 0; i--) {
		// pm_device_state_unlock(TYPE_SECTION_START(pm_device_slots)[i]);//correspond the pm_device_state_lock() in pm_suspend_devices_rtk
        pm_device_action_run(TYPE_SECTION_START(pm_device_slots)[i],
				    PM_DEVICE_ACTION_RESUME);
	}

    // NVIC_InitTypeDef nvic_init_struct = {0};
    // nvic_init_struct.NVIC_IRQChannel         = System_IRQn;
    // nvic_init_struct.NVIC_IRQChannelCmd      = (FunctionalState)ENABLE;
    // nvic_init_struct.NVIC_IRQChannelPriority = 3;
    // NVIC_Init(&nvic_init_struct); //Enable SYSTEM_ON Interrupt

    CPU_DLPS_Exit();

	num_susp_rtk = 0;
}
#endif  /* !CONFIG_PM_DEVICE_RUNTIME_EXCLUSIVE */
#endif	/* CONFIG_PM_DEVICE */


/* Initialize power system */
static int rtl87x2g_power_init(void)
{
    int ret = 0;

    bt_power_mode_set(BTPOWER_DEEP_SLEEP);
    //bt_power_mode_set(BTPOWER_ACTIVE);

    power_mode_set(POWER_DLPS_MODE);
    //power_mode_set(POWER_ACTIVE_MODE);

    extern void NMI_Handler(void);
    z_arm_nmi_set_handler(NMI_Handler);

    platform_pm_register_callback_func_with_priority((void *)pm_suspend_devices_rtk, PLATFORM_PM_STORE, 1);
    platform_pm_register_callback_func_with_priority((void *)pm_resume_devices_rtk, PLATFORM_PM_RESTORE, 1);

    return ret;
}

SYS_INIT(rtl87x2g_power_init, APPLICATION, 1); //do it after lowerstack entry
