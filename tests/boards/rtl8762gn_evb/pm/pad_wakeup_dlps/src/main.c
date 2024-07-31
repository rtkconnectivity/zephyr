#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/dt-bindings/gpio/realtek-rtl87x2g-gpio.h>
#include <zephyr/ztest.h>
#include <pm.h>

#define KEY_PRESS_NUMBERS 5
static uint32_t key_press_num;

static struct k_sem test_thread_sem;

static uint32_t wakeup_count_before_test;
static uint32_t wakeup_count_after_test;
static uint32_t last_wakeup_clk, last_sleep_clk;

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

ZTEST_SUITE(pad_wakeup_dlps, NULL, NULL, NULL, NULL, NULL);

ZTEST(pad_wakeup_dlps, test_gpio_wakeup_dlps)
{
	printk("Starting gpio_wakeup_dlps Test\n");
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
	power_get_statistics(&wakeup_count_before_test, &last_wakeup_clk, &last_sleep_clk);
	printk("After %d times key-pressing, calculate the wake-up times!\n", KEY_PRESS_NUMBERS);
	k_sem_init(&test_thread_sem, 0, UINT_MAX);
	k_sem_take(&test_thread_sem, K_FOREVER);
	power_get_statistics(&wakeup_count_after_test, &last_wakeup_clk, &last_sleep_clk);
	uint32_t wakeup_count_key_press = wakeup_count_after_test - wakeup_count_before_test;

	TC_PRINT("wakeupCount: %d, last_wakeup_clk:%d, last_sleep_clk:%d\n", wakeup_count_key_press,
			last_wakeup_clk, last_sleep_clk);
	zassert_true(wakeup_count_key_press == KEY_PRESS_NUMBERS, "test_gpio_wakeup_dlps failed,
			wakeup Count: %d\n", wakeup_count_key_press);
}
