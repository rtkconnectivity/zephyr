#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/gpio/realtek-rtl87x2g-gpio.h>
#include <zephyr/ztest.h>
#include <pm.h>
#include <pmu_manager.h>

static struct k_sem test_thread_sem;

#define KEY_PRESS_NUMBERS 5
static uint32_t key_press_num;

#if DT_NODE_EXISTS(DT_NODELABEL(button2))
#define BUTTON DT_NODELABEL(button2)
#endif

#if DT_NODE_EXISTS(BUTTON)
#define BUTTON_DEV DT_PHANDLE(BUTTON, gpios)
#define BUTTON_PIN DT_PHA(BUTTON, gpios, pin)
#define BUTTON_FLAGS DT_PHA(BUTTON, gpios, flags)

static const struct device *const button_dev = DEVICE_DT_GET(BUTTON_DEV);

static struct k_work button_work;

static void button_cb(const struct device *port, struct gpio_callback *cb,
		      gpio_port_pins_t pins)
{
	k_work_submit(&button_work);
}
#endif /* BUTTON */

static void button_pressed(struct k_work *work)
{
	key_press_num++;
	printk("button press tested, key_press_num:%d!\n", key_press_num);
	if (key_press_num == KEY_PRESS_NUMBERS)
		k_sem_give(&test_thread_sem);
}

void pm_verify_powerdown_is_entered(void)
{
	printk("The system has entered power-down mode. Please KEY0(PIN4_0) to wake up!\n");
	AON_NS_REG0X_APP_TYPE aon_0x1ae0 = {.d32 = AON_REG_READ(AON_NS_REG0X_APP)};

	aon_0x1ae0.reset_reason = 0x9d;
	AON_REG_WRITE(AON_NS_REG0X_APP, aon_0x1ae0.d32);
}

ZTEST_SUITE(pad_wakeup_powerdown, NULL, NULL, NULL, NULL, NULL);

ZTEST(pad_wakeup_powerdown, test_gpio_wakeup_powerdown)
{
	printk("Starting gpio_wakeup_powerdown Test\n");
	platform_pm_register_callback_func_with_priority(pm_verify_powerdown_is_entered,
			PLATFORM_PM_STORE, 1);
	/* overwrite the power_mode_set(POWER_DLPS_MODE) in rtl87x2g_power_init() */
	power_mode_set(POWER_POWERDOWN_MODE);
	k_work_init(&button_work, button_pressed);
#if DT_NODE_EXISTS(BUTTON)
	int err;

	err = gpio_pin_configure(button_dev, BUTTON_PIN,
				BUTTON_FLAGS | GPIO_INPUT | GPIO_PULL_UP
				| RTL87X2G_GPIO_INPUT_PM_WAKEUP);
	if (err) {
		TC_PRINT("gpio_pin_configure err!");
	}

	static struct gpio_callback gpio_cb;

	err = gpio_pin_interrupt_configure(button_dev, BUTTON_PIN,
					   GPIO_INT_LEVEL_INACTIVE);
	if (err) {
		TC_PRINT("gpio_pin_interrupt_configure err!");
	}

	gpio_init_callback(&gpio_cb, button_cb, BIT(BUTTON_PIN));
	gpio_add_callback(button_dev, &gpio_cb);
#else
	printk("WARNING: Buttons not supported on this board.\n");
#endif

	AON_NS_REG0X_APP_TYPE aon_0x1ae0 = {.d32 = AON_REG_READ(AON_NS_REG0X_APP)};
	uint32_t reset_type = aon_0x1ae0.reset_reason;

	if (reset_type == 0x9d) {
		TC_PRINT("The system has waked up from power-down mode. Test Pass!\n");
	} else {
		k_sem_init(&test_thread_sem, 0, UINT_MAX);
		k_sem_take(&test_thread_sem, K_FOREVER);
	}
}
