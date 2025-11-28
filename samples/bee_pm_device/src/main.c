/*
 * Copyright (c) 2025 Realtek Semiconductor Corp.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/shell/shell.h>
#ifdef CONFIG_GPIO
#include <zephyr/drivers/gpio.h>
#if defined(CONFIG_GPIO_BEE)
#include <zephyr/dt-bindings/gpio/realtek-bee-gpio.h>
#endif
#endif
#ifdef CONFIG_SERIAL
#include <zephyr/drivers/uart.h>
#endif
#ifdef CONFIG_PWM
#include <zephyr/drivers/pwm.h>
#endif
#ifdef CONFIG_COUNTER
#include <zephyr/drivers/counter.h>
#endif
#ifdef CONFIG_SPI
#include <zephyr/drivers/spi.h>
#endif
#ifdef CONFIG_RTC
#include <zephyr/drivers/rtc.h>
#endif
#ifdef CONFIG_I2C
#include <zephyr/drivers/i2c.h>
#endif
#ifdef CONFIG_ADC
#include <zephyr/drivers/adc.h>
#endif
#ifdef CONFIG_SENSOR
#include <zephyr/drivers/sensor.h>
#if defined(CONFIG_QDEC_BEE)
#include <zephyr/drivers/sensor/qdec_bee.h>
#endif
#endif
#ifdef CONFIG_SDMMC_STACK
#include <zephyr/sd/sdmmc.h>
#endif
#ifdef CONFIG_CAN
#include <zephyr/drivers/can.h>
#endif
#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#include "trace.h"
#include <pm.h>
#include "power_manager_unit_platform.h"

#define PM_TEST_CHECK_PASS PM_CHECK_PASS
#define PM_TEST_CHECK_FAIL PM_CHECK_FAIL
#define PM_TEST_CHECK_RET  PMCheckResult

#define pm_test_register_check_cb(app_check)                                                       \
	platform_pm_register_callback_func_with_priority((void *)app_check, PLATFORM_PM_CHECK, 1)
#define pm_test_register_store_cb(app_store)                                                       \
	platform_pm_register_callback_func_with_priority((void *)app_store, PLATFORM_PM_STORE, 1)
#define pm_test_register_restore_cb(app_restore)                                                   \
	platform_pm_register_callback_func_with_priority((void *)app_restore, PLATFORM_PM_RESTORE, \
							 1)

#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#include "trace.h"
#include <dlps.h>

extern void (*platform_pm_register_callback_func_with_priority)(void *cb_func,
								PlatformPMStage pf_pm_stage,
								int8_t priority);

#define PM_TEST_CHECK_PASS PM_CHECK_PASS
#define PM_TEST_CHECK_FAIL PM_CHECK_FAIL
#define PM_TEST_CHECK_RET  PMCheckResult

#define pm_test_register_check_cb(app_check)                                                       \
	platform_pm_register_callback_func_with_priority((void *)app_check, PLATFORM_PM_CHECK, 1)
#define pm_test_register_store_cb(app_store)                                                       \
	platform_pm_register_callback_func_with_priority((void *)app_store, PLATFORM_PM_STORE, 1)
#define pm_test_register_restore_cb(app_restore)                                                   \
	platform_pm_register_callback_func_with_priority((void *)app_restore, PLATFORM_PM_RESTORE, \
							 1)

#else
#include "log_core.h"
#endif

#if defined(CONFIG_PM_DEVICE)
struct k_sem app_sem;

static PM_TEST_CHECK_RET dlps_check_flag = PM_TEST_CHECK_FAIL;

static PM_TEST_CHECK_RET app_check(void)
{
	return dlps_check_flag;
}

uint32_t pm_cnt;

static void app_store(void)
{
	DBG_DIRECT("[%s] %d line%d", __func__, ++pm_cnt, __LINE__);
}

static void app_restore(void)
{
	DBG_DIRECT("[%s] %d line%d", __func__, pm_cnt, __LINE__);
	k_sem_give(&app_sem);
}

#endif

int main(void)
{
	printf("[%lld] Bee pm device test for %s\n", k_uptime_get(), CONFIG_BOARD_TARGET);

#if defined(CONFIG_PM_DEVICE)
	k_sem_init(&app_sem, 0, 1);

	dlps_check_flag = PM_TEST_CHECK_FAIL;

	pm_test_register_check_cb(app_check);
	pm_test_register_store_cb(app_store);
	pm_test_register_restore_cb(app_restore);
#endif

	return 0;
}

static int shell_pm_test_uart(const struct shell *sh, size_t argc, char **argv)
{
#ifdef CONFIG_SERIAL
#if defined(CONFIG_PM_DEVICE)
	/* ==================================================================================== */
	/* enter dlps */
	printf("[%lld] before enter dlps\n", k_uptime_get());
	printf("[%lld] type on shell to wakeup\n", k_uptime_get());
	dlps_check_flag = PM_TEST_CHECK_PASS;
	k_sem_init(&app_sem, 0, 1);
	k_sem_take(&app_sem, K_FOREVER);

	dlps_check_flag = PM_TEST_CHECK_FAIL;

	/* rx 1 byte to wakeup */
	printf("[%lld] after exit dlps\n", k_uptime_get());
	/* ==================================================================================== */
#endif
#endif

	return 0;
}

#ifdef CONFIG_UART_ASYNC_API
const struct device *test_uart_dma_dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(test_uart_dma));
struct k_sem uart_dma_tx_sem;
struct k_sem uart_dma_rx_sem;
uint8_t uart_dma_rx_buf[1024];
uint32_t uart_dma_rx_len;
bool uart_dma_rx_enabled;

static void uart_async_console_callback(const struct device *dev, struct uart_event *evt,
					void *user_data)
{
	printf("[%s] evt->type=%d", __func__, evt->type);
	switch (evt->type) {
	case UART_TX_DONE:
		printf("uart dma tx done\n");
		k_sem_give(&uart_dma_tx_sem);
		break;
	case UART_RX_RDY:
		printf("uart dma rx ready\n");
		memcpy(uart_dma_rx_buf, &evt->data.rx.buf[evt->data.rx.offset], evt->data.rx.len);
		uart_dma_rx_len = evt->data.rx.len;
		k_sem_give(&uart_dma_rx_sem);
		break;
	default:
		break;
	}
}
static void uart_dma_enter_cb(void)
{
	DBG_DIRECT("[%s] line%d", __func__, __LINE__);
}

static void uart_dma_exit_cb(void)
{
	DBG_DIRECT("[%s] line%d", __func__, __LINE__);
	if (uart_dma_rx_enabled) {
	}
}

static int shell_pm_test_uart_dma(const struct shell *sh, size_t argc, char **argv)
{
	static bool pm_uart_dma_register_cb_flag;

	DBG_DIRECT("[%s] pm_uart_dma_register_cb_flag=%d line%d", __func__,
		   pm_uart_dma_register_cb_flag, __LINE__);
	if (pm_uart_dma_register_cb_flag == false) {
		pm_test_register_store_cb(uart_dma_enter_cb);
		pm_test_register_restore_cb(uart_dma_exit_cb);
		pm_uart_dma_register_cb_flag = true;
	}

	/* ==================================================================================== */
	k_sem_init(&uart_dma_tx_sem, 0, 1);
	k_sem_init(&uart_dma_rx_sem, 0, 1);
	memset(uart_dma_rx_buf, 0, sizeof(uart_dma_rx_buf));
	uart_dma_rx_len = 0;

	uart_callback_set(test_uart_dma_dev, uart_async_console_callback, NULL);

	uart_dma_rx_enabled = true;
	uart_rx_enable(test_uart_dma_dev, uart_dma_rx_buf, sizeof(uart_dma_rx_buf),
		       50 * USEC_PER_MSEC);
	printf("send some data from dma uart\n");
	k_sem_take(&uart_dma_rx_sem, K_FOREVER);

	printf("uart dma rx data(%d bytes):\n", uart_dma_rx_len);
	for (size_t i = 0; i < uart_dma_rx_len; i++) {
		if (i % 16 == 0) {
			printf("%d: ", i);
		}

		printf("%c ", uart_dma_rx_buf[i]);

		if (i % 16 == 15 || i == uart_dma_rx_len - 1) {
			printf("\n");
		}
	}

	uart_tx(test_uart_dma_dev, uart_dma_rx_buf, uart_dma_rx_len, 100 * USEC_PER_MSEC);
	k_sem_take(&uart_dma_tx_sem, K_FOREVER);

	/* ==================================================================================== */

	/* enter dlps */
	printf("[%lld] before enter dlps\n", k_uptime_get());
	printf("[%lld] type on shell to wakeup\n", k_uptime_get());
	dlps_check_flag = PM_TEST_CHECK_PASS;
	k_sem_init(&app_sem, 0, 1);
	k_sem_take(&app_sem, K_FOREVER);

	dlps_check_flag = PM_TEST_CHECK_FAIL;

	/* rx 1 byte to wakeup */
	printf("[%lld] after exit dlps\n", k_uptime_get());

	/* ==================================================================================== */

	k_sem_init(&uart_dma_tx_sem, 0, 1);
	k_sem_init(&uart_dma_rx_sem, 0, 1);
	memset(uart_dma_rx_buf, 0, sizeof(uart_dma_rx_buf));
	uart_dma_rx_len = 0;

	uart_rx_disable(test_uart_dma_dev);
	uart_rx_enable(test_uart_dma_dev, uart_dma_rx_buf, sizeof(uart_dma_rx_buf),
		       50 * USEC_PER_MSEC);
	printf("send some data from dma uart\n");
	k_sem_take(&uart_dma_rx_sem, K_FOREVER);

	printf("uart dma rx data(%d bytes):\n", uart_dma_rx_len);
	for (size_t i = 0; i < uart_dma_rx_len; i++) {
		if (i % 16 == 0) {
			printf("%d: ", i);
		}

		printf("%c ", uart_dma_rx_buf[i]);

		if (i % 16 == 15 || i == uart_dma_rx_len - 1) {
			printf("\n");
		}
	}

	uart_tx(test_uart_dma_dev, uart_dma_rx_buf, uart_dma_rx_len, 100 * USEC_PER_MSEC);
	k_sem_take(&uart_dma_tx_sem, K_FOREVER);

	uart_rx_disable(test_uart_dma_dev);
	uart_dma_rx_enabled = false;

	/* ==================================================================================== */
	return 0;
}
#endif

#ifdef CONFIG_COUNTER
const struct device *counter_timer_dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(test_counter_timer));

static void top_handler(const struct device *dev, void *user_data)
{
	printf("[%s]\n", __func__);

	k_sem_give(&app_sem);
}

static int shell_pm_test_counter(const struct shell *sh, size_t argc, char **argv)
{
	struct counter_top_cfg top_cfg;
	uint64_t current_sys_time_ms;
	uint64_t pre_sys_time_ms;
	uint64_t timeout_ms = atoi(argv[1]);

	/* ==================================================================================== */
	counter_start(counter_timer_dev);

	top_cfg.callback = top_handler;
	top_cfg.flags = 0;
	top_cfg.ticks = counter_us_to_ticks(counter_timer_dev, timeout_ms * 1000);
	pre_sys_time_ms = k_uptime_get();
	top_cfg.user_data = NULL;
	printf("[%lld] wait %lldms to trigger handler\n", pre_sys_time_ms, timeout_ms);
	counter_set_top_value(counter_timer_dev, &top_cfg);
	/* ==================================================================================== */

	/* enter dlps */
	dlps_check_flag = PM_TEST_CHECK_PASS;
	k_sem_init(&app_sem, 0, 1);
	k_sem_take(&app_sem, K_FOREVER);

	dlps_check_flag = PM_TEST_CHECK_FAIL;

	current_sys_time_ms = k_uptime_get();
	printf("[%lld] trigger handler after %lldms\n", current_sys_time_ms,
	       current_sys_time_ms - pre_sys_time_ms);

	counter_stop(counter_timer_dev);
	/* ==================================================================================== */

	return 0;
}
#endif

#ifdef CONFIG_GPIO
#define DEV_OUT       DT_GPIO_CTLR(DT_INST(0, bee_pm_device_gpio), out_gpios)
#define DEV_IN        DT_GPIO_CTLR(DT_INST(0, bee_pm_device_gpio), in_gpios)
#define PIN_OUT       DT_GPIO_PIN(DT_INST(0, bee_pm_device_gpio), out_gpios)
#define PIN_OUT_FLAGS DT_GPIO_FLAGS(DT_INST(0, bee_pm_device_gpio), out_gpios)
#define PIN_IN        DT_GPIO_PIN(DT_INST(0, bee_pm_device_gpio), in_gpios)
#define PIN_IN_FLAGS  DT_GPIO_FLAGS(DT_INST(0, bee_pm_device_gpio), in_gpios)

const struct device *const gpio_dev_in = DEVICE_DT_GET_OR_NULL(DEV_IN);
const struct device *const gpio_dev_out = DEVICE_DT_GET_OR_NULL(DEV_OUT);

struct gpio_callback gpio_cb;
struct k_sem gpio_sem;

static void gpio_callback(const struct device *dev_in, struct gpio_callback *gpio_cb, uint32_t pins)
{
	static uint8_t cnt;
#if defined(CONFIG_PM_DEVICE)
	dlps_check_flag = PM_TEST_CHECK_FAIL;
#endif
	k_sem_give(&gpio_sem);
	printf("[%lld] enter gpio callback cnt%d\n", k_uptime_get(), cnt);
	++cnt;
}

static int shell_pm_test_gpio(const struct shell *sh, size_t argc, char **argv)
{

	k_sem_init(&gpio_sem, 0, 1);

	/* ==================================================================================== */
	/* 1. set PIN_OUT to logical initial state inactive */
	gpio_pin_configure(gpio_dev_out, PIN_OUT, GPIO_OUTPUT_LOW | PIN_OUT_FLAGS);

	/* 2. configure PIN_IN callback and trigger condition */
	gpio_pin_configure(gpio_dev_in, PIN_IN,
			   (GPIO_INPUT | GPIO_PULL_UP | PIN_IN_FLAGS | BEE_GPIO_INPUT_PM_WAKEUP));

	gpio_init_callback(&gpio_cb, gpio_callback, BIT(PIN_IN));
	gpio_add_callback(gpio_dev_in, &gpio_cb);
	gpio_pin_interrupt_configure(gpio_dev_in, PIN_IN, GPIO_INT_EDGE_FALLING);

#if defined(CONFIG_PM_DEVICE)
	/* enter dlps */
	printf("[%lld] before enter dlps\n", k_uptime_get());
	printf("[%lld] connect input pin to output pin to wakeup\n", k_uptime_get());
	dlps_check_flag = PM_TEST_CHECK_PASS;
	k_sem_init(&app_sem, 0, 1);
	k_sem_take(&app_sem, K_FOREVER);

	dlps_check_flag = PM_TEST_CHECK_FAIL;

	/* rising edge to wakeup */
	printf("[%lld] after exit dlps\n", k_uptime_get());
#else
	printf("[%lld] connect input pin to output pin to wakeup\n", k_uptime_get());
	k_sem_init(&gpio_sem, 0, 1);
	k_sem_take(&gpio_sem, K_FOREVER);
#endif
	k_busy_wait(100000);
	gpio_pin_interrupt_configure(gpio_dev_in, PIN_IN, GPIO_INT_EDGE_RISING);

	/* ==================================================================================== */
#if defined(CONFIG_PM_DEVICE)
	/* enter dlps */
	printf("[%lld] before enter dlps\n", k_uptime_get());
	printf("[%lld] disconnect input pin to output pin to wakeup\n", k_uptime_get());
	dlps_check_flag = PM_TEST_CHECK_PASS;
	k_sem_init(&app_sem, 0, 1);
	k_sem_take(&app_sem, K_FOREVER);

	dlps_check_flag = PM_TEST_CHECK_FAIL;

	/* falling edge to wakeup */
	printf("[%lld] after exit dlps\n", k_uptime_get());
#else
	printf("[%lld] disconnect input pin to output pin to wakeup\n", k_uptime_get());
	k_sem_init(&gpio_sem, 0, 1);
	k_sem_take(&gpio_sem, K_FOREVER);
#endif
	k_busy_wait(100000);
	/* ==================================================================================== */
#if defined(CONFIG_BEE_GPIO_SUPPORT_BOTH_EDGE)
	/* both edge */
	gpio_pin_interrupt_configure(gpio_dev_in, PIN_IN, GPIO_INT_EDGE_BOTH);

#if defined(CONFIG_PM_DEVICE)
	/* enter dlps */
	printf("[%lld] before enter dlps\n", k_uptime_get());
	printf("[%lld] connect input pin to output pin to wakeup\n", k_uptime_get());
	dlps_check_flag = PM_TEST_CHECK_PASS;
	k_sem_init(&app_sem, 0, 1);
	k_sem_take(&app_sem, K_FOREVER);

	dlps_check_flag = PM_TEST_CHECK_FAIL;

	/* rising edge to wakeup */
	printf("[%lld] after exit dlps\n", k_uptime_get());
#else
	printf("[%lld] connect input pin to output pin to wakeup\n", k_uptime_get());
	k_sem_init(&gpio_sem, 0, 1);
	k_sem_take(&gpio_sem, K_FOREVER);
#endif
	k_busy_wait(100000);

	/* ==================================================================================== */
#if defined(CONFIG_PM_DEVICE)
	/* enter dlps */
	printf("[%lld] before enter dlps\n", k_uptime_get());
	printf("[%lld] disconnect input pin to output pin to wakeup\n", k_uptime_get());
	dlps_check_flag = PM_TEST_CHECK_PASS;
	k_sem_init(&app_sem, 0, 1);
	k_sem_take(&app_sem, K_FOREVER);

	dlps_check_flag = PM_TEST_CHECK_FAIL;

	/* falling edge to wakeup */
	printf("[%lld] after exit dlps\n", k_uptime_get());
#else
	printf("[%lld] disconnect input pin to output pin to wakeup\n", k_uptime_get());
	k_sem_init(&gpio_sem, 0, 1);
	k_sem_take(&gpio_sem, K_FOREVER);
#endif

	k_busy_wait(100000);

#endif

	gpio_remove_callback(gpio_dev_in, &gpio_cb);

	gpio_pin_interrupt_configure(gpio_dev_in, PIN_IN, GPIO_INT_DISABLE);
	gpio_pin_configure(gpio_dev_in, PIN_IN,
			   ((GPIO_INPUT) | GPIO_PULL_UP | PIN_IN_FLAGS) &
				   (~BEE_GPIO_INPUT_PM_WAKEUP));

	/* ==================================================================================== */

	return 0;
}
#endif

#ifdef CONFIG_PWM
const struct device *pwm_dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(test_pwm));

static int shell_pm_test_pwm(const struct shell *sh, size_t argc, char **argv)
{
	uint32_t period, pulse;

	/* ==================================================================================== */
	printf("[%lld] connect pwm pin to LA to watch the waveform\n", k_uptime_get());

	period = 50000;
	pulse = 10000;
	printf("[%lld] [PWM]: %s, [period]: %d, [pulse]: %d\n", k_uptime_get(), pwm_dev->name,
	       period, pulse);
	pwm_set_cycles(pwm_dev, 0, period, pulse, 0);
	k_busy_wait(500000);

	/* ==================================================================================== */
#if defined(CONFIG_PM_DEVICE)
	/* enter dlps */
	printf("[%lld] before enter dlps\n", k_uptime_get());
	dlps_check_flag = PM_TEST_CHECK_PASS;
	/* delay 500 ms to wakeup */
	k_sem_init(&app_sem, 0, 1);
	k_sem_take(&app_sem, K_MSEC(500));
	dlps_check_flag = PM_TEST_CHECK_FAIL;

	printf("[%lld] after exit dlps\n", k_uptime_get());
	k_busy_wait(500000);
#endif
	/* ==================================================================================== */

	period = 0;
	pulse = 0;
	printf("[%lld] [PWM]: %s, [period]: %d, [pulse]: %d\n", k_uptime_get(), pwm_dev->name,
	       period, pulse);
	pwm_set_cycles(pwm_dev, 0, period, pulse, 0);
	k_busy_wait(500000);

#if defined(CONFIG_PM_DEVICE)
	k_sleep(K_MSEC(10));
#endif

	period = 50000;
	pulse = 40000;
	printf("[%lld] [PWM]: %s, [period]: %d, [pulse]: %d\n", k_uptime_get(), pwm_dev->name,
	       period, pulse);
	pwm_set_cycles(pwm_dev, 0, period, pulse, 0);
	k_busy_wait(500000);

	/* ==================================================================================== */
#if defined(CONFIG_PM_DEVICE)
	/* enter dlps */
	printf("[%lld] before enter dlps\n", k_uptime_get());
	dlps_check_flag = PM_TEST_CHECK_PASS;
	/* delay 500 ms to wakeup */
	k_sem_init(&app_sem, 0, 1);
	k_sem_take(&app_sem, K_MSEC(500));
	dlps_check_flag = PM_TEST_CHECK_FAIL;

	printf("[%lld] after exit dlps\n", k_uptime_get());
	k_busy_wait(500000);
#endif
	/* ==================================================================================== */

	period = 0;
	pulse = 0;
	printf("[%lld] [PWM]: %s, [period]: %d, [pulse]: %d\n", k_uptime_get(), pwm_dev->name,
	       period, pulse);
	pwm_set_cycles(pwm_dev, 0, period, pulse, 0);
	/* ==================================================================================== */

	return 0;
}
#endif

#ifdef CONFIG_SPI
#define MODE_LOOP  0
#define FRAME_SIZE (8)
#define SPI_OP(frame_size)                                                                         \
	SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(frame_size) |            \
		SPI_LINES_SINGLE

#define SPI_DEV DT_COMPAT_GET_ANY_STATUS_OKAY(bee_pm_device_spi)
static struct spi_dt_spec spi_spec = SPI_DT_SPEC_GET(SPI_DEV, SPI_OP(FRAME_SIZE), 0);

#define BUF_SIZE 18

static const char tx_data[BUF_SIZE] = "0123456789abcdef-\0";
static __aligned(32) char buffer_tx[BUF_SIZE];
static __aligned(32) char buffer_rx[BUF_SIZE];

static int spi_complete_loop(struct spi_dt_spec *spec)
{
	memcpy(buffer_tx, tx_data, sizeof(tx_data));
	memset(buffer_rx, 0, sizeof(buffer_rx));

	const struct spi_buf tx_bufs[] = {
		{
			.buf = buffer_tx,
			.len = BUF_SIZE,
		},
	};
	const struct spi_buf rx_bufs[] = {
		{
			.buf = buffer_rx,
			.len = BUF_SIZE,
		},
	};
	const struct spi_buf_set tx = {.buffers = tx_bufs, .count = ARRAY_SIZE(tx_bufs)};
	const struct spi_buf_set rx = {.buffers = rx_bufs, .count = ARRAY_SIZE(rx_bufs)};

	int ret;

	printf("[%lld] Start complete loop\n", k_uptime_get());

	ret = spi_transceive_dt(spec, &tx, &rx);

	if (memcmp(buffer_tx, buffer_rx, BUF_SIZE)) {
		printf("[%lld] Buffer contents are different\n", k_uptime_get());
		return -1;
	}

	printf("[%lld] Buffer contents are same\n", k_uptime_get());
	return 0;
}

static int shell_pm_test_spi(const struct shell *sh, size_t argc, char **argv)
{

	/* ==================================================================================== */
	printf("[%lld] connect MOSI pin to the MISO of the SPI\n", k_uptime_get());

	if (spi_complete_loop(&spi_spec) < 0) {
		printf("[%lld] loopback test fali\n", k_uptime_get());
		return 0;
	}

	/* ==================================================================================== */
#if defined(CONFIG_PM_DEVICE)
	printf("[%lld] before enter dlps\n", k_uptime_get());
	printf("[%lld] type on shell to wakeup\n", k_uptime_get());
	dlps_check_flag = PM_TEST_CHECK_PASS;
	k_sem_init(&app_sem, 0, 1);
	k_sem_take(&app_sem, K_FOREVER);

	dlps_check_flag = PM_TEST_CHECK_FAIL;

	/* rx 1 byte to wakeup */
	printf("[%lld] after exit dlps\n", k_uptime_get());
#endif

	/* ==================================================================================== */
	if (spi_complete_loop(&spi_spec) < 0) {
		printf("[%lld] loopback test fali\n", k_uptime_get());
		return 0;
	}
	/* ==================================================================================== */

	return 0;
}
#endif

#ifdef CONFIG_RTC
static const struct device *rtc_dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(test_rtc));

static const struct rtc_time rtc_time_set = {
	.tm_sec = 50,
	.tm_min = 29,
	.tm_hour = 13,
	.tm_mday = 1,
	.tm_mon = 0,
	.tm_year = 121,
	.tm_wday = 5,
	.tm_yday = 1,
	.tm_isdst = -1,
	.tm_nsec = 0,
};

static const struct rtc_time alarm_time_set = {
	.tm_sec = 52,
	.tm_min = 29,
	.tm_hour = 13,
	.tm_mday = 1,
	.tm_mon = 0,
	.tm_year = 121,
	.tm_wday = 5,
	.tm_yday = 1,
	.tm_isdst = -1,
	.tm_nsec = 0,
};

static void rtc_alarm_callback_handler(const struct device *dev, uint16_t id, void *user_data)
{
#if defined(CONFIG_PM_DEVICE)
	k_sem_give(&app_sem);
#endif
}

static int shell_pm_test_rtc(const struct shell *sh, size_t argc, char **argv)
{
	uint64_t current_sys_time_ms;
	uint64_t pre_sys_time_ms;

	/* ==================================================================================== */
	rtc_alarm_set_callback(rtc_dev, 0, NULL, NULL);
	rtc_set_time(rtc_dev, &rtc_time_set);

	rtc_alarm_set_callback(rtc_dev, 0, rtc_alarm_callback_handler, NULL);

	pre_sys_time_ms = k_uptime_get();
	rtc_alarm_set_time(rtc_dev, 0, 0x1ff, &alarm_time_set);

	printf("[%lld] wait %dms to trigger handler\n", pre_sys_time_ms, 2000);

	/* ==================================================================================== */
#if defined(CONFIG_PM_DEVICE)
	/* enter dlps */
	dlps_check_flag = PM_TEST_CHECK_PASS;
	k_sem_init(&app_sem, 0, 1);
	k_sem_take(&app_sem, K_FOREVER);

	dlps_check_flag = PM_TEST_CHECK_FAIL;
	printf("[%s] wakeup\n", __func__);
#endif

	current_sys_time_ms = k_uptime_get();
	printf("[%lld] trigger handler after %lldms\n", current_sys_time_ms,
	       current_sys_time_ms - pre_sys_time_ms);
	/* ==================================================================================== */
	return 0;
}
#endif

#if defined(CONFIG_SENSOR) && defined(CONFIG_QDEC_BEE)
static const struct gpio_dt_spec qdec_phase_a = GPIO_DT_SPEC_GET(DT_ALIAS(test_qenca), gpios);
static const struct gpio_dt_spec qdec_phase_b = GPIO_DT_SPEC_GET(DT_ALIAS(test_qencb), gpios);
const struct device *const qdec_dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(test_qdec));

static int shell_pm_test_qdec(const struct shell *sh, size_t argc, char **argv)
{
	static bool toggle_a;
	struct sensor_value val;

	/* ==================================================================================== */
	gpio_pin_configure_dt(&qdec_phase_a, GPIO_OUTPUT);
	gpio_pin_configure_dt(&qdec_phase_b, GPIO_OUTPUT);

	k_busy_wait(100000);

	for (int i = 0; i < 12; i++) {
		toggle_a = !toggle_a;
		if (toggle_a) {
			gpio_pin_toggle_dt(&qdec_phase_a);
		} else {
			gpio_pin_toggle_dt(&qdec_phase_b);
		}
		k_busy_wait(100000);
		sensor_sample_fetch(qdec_dev);
		sensor_channel_get(qdec_dev, SENSOR_ATTR_QDEC_X_ROTATION, &val);
		printf("Position[%d] = %d degrees\n", i, val.val1);
	}

	/* ==================================================================================== */
#if defined(CONFIG_PM_DEVICE)
	printf("[%lld] before enter dlps\n", k_uptime_get());
	printf("[%lld] type on shell to wakeup\n", k_uptime_get());
	dlps_check_flag = PM_TEST_CHECK_PASS;
	k_sem_init(&app_sem, 0, 1);
	k_sem_take(&app_sem, K_FOREVER);

	dlps_check_flag = PM_TEST_CHECK_FAIL;

	/* rx 1 byte to wakeup */
	printf("[%lld] after exit dlps\n", k_uptime_get());
#endif

	for (int i = 0; i < 12; i++) {
		toggle_a = !toggle_a;
		if (!toggle_a) {
			gpio_pin_toggle_dt(&qdec_phase_a);
		} else {
			gpio_pin_toggle_dt(&qdec_phase_b);
		}
		k_busy_wait(100000);
		sensor_sample_fetch(qdec_dev);
		sensor_channel_get(qdec_dev, SENSOR_ATTR_QDEC_X_ROTATION, &val);
		printf("Position[%d] = %d degrees\n", i, val.val1);
	}

	/* ==================================================================================== */
#if defined(CONFIG_PM_DEVICE)
	printf("[%lld] before enter dlps\n", k_uptime_get());
	printf("[%lld] type on shell to wakeup\n", k_uptime_get());
	dlps_check_flag = PM_TEST_CHECK_PASS;
	k_sem_init(&app_sem, 0, 1);
	k_sem_take(&app_sem, K_FOREVER);

	dlps_check_flag = PM_TEST_CHECK_FAIL;

	/* rx 1 byte to wakeup */
	printf("[%lld] after exit dlps\n", k_uptime_get());
#endif

	for (int i = 0; i < 12; i++) {
		toggle_a = !toggle_a;
		if (!toggle_a) {
			gpio_pin_toggle_dt(&qdec_phase_a);
		} else {
			gpio_pin_toggle_dt(&qdec_phase_b);
		}
		k_busy_wait(100000);
		sensor_sample_fetch(qdec_dev);
		sensor_channel_get(qdec_dev, SENSOR_ATTR_QDEC_X_ROTATION, &val);
		printf("Position[%d] = %d degrees\n", i, val.val1);
	}

	/* ==================================================================================== */
#if defined(CONFIG_PM_DEVICE)
	printf("[%lld] before enter dlps\n", k_uptime_get());
	printf("[%lld] type on shell to wakeup\n", k_uptime_get());
	dlps_check_flag = PM_TEST_CHECK_PASS;
	k_sem_init(&app_sem, 0, 1);
	k_sem_take(&app_sem, K_FOREVER);

	dlps_check_flag = PM_TEST_CHECK_FAIL;

	/* rx 1 byte to wakeup */
	printf("[%lld] after exit dlps\n", k_uptime_get());
#endif

	for (int i = 0; i < 12; i++) {
		toggle_a = !toggle_a;
		if (toggle_a) {
			gpio_pin_toggle_dt(&qdec_phase_a);
		} else {
			gpio_pin_toggle_dt(&qdec_phase_b);
		}
		k_busy_wait(100000);
		sensor_sample_fetch(qdec_dev);
		sensor_channel_get(qdec_dev, SENSOR_ATTR_QDEC_X_ROTATION, &val);
		printf("Position[%d] = %d degrees\n", i, val.val1);
	}
	return 0;
}
#endif

#if defined(CONFIG_SENSOR) && (defined(CONFIG_AON_QDEC_BEE))
static const struct gpio_dt_spec aon_qdec_phase_a = GPIO_DT_SPEC_GET(DT_ALIAS(test_qenca), gpios);
static const struct gpio_dt_spec aon_qdec_phase_b = GPIO_DT_SPEC_GET(DT_ALIAS(test_qencb), gpios);
const struct device *const aon_qdec_dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(test_aon_qdec));

static int shell_pm_test_aon_qdec(const struct shell *sh, size_t argc, char **argv)
{
	static bool toggle_a;
	struct sensor_value val;

	gpio_pin_configure_dt(&aon_qdec_phase_a, GPIO_OUTPUT);
	gpio_pin_configure_dt(&aon_qdec_phase_b, GPIO_OUTPUT);

	k_busy_wait(100000);

	for (int i = 0; i < 10; i++) {
		toggle_a = !toggle_a;
		if (toggle_a) {
			gpio_pin_toggle_dt(&aon_qdec_phase_a);
		} else {
			gpio_pin_toggle_dt(&aon_qdec_phase_b);
		}
		k_busy_wait(100000);
		sensor_sample_fetch(aon_qdec_dev);
		sensor_channel_get(aon_qdec_dev, SENSOR_CHAN_ROTATION, &val);
		printf("Position[%d] = %d degrees\n", i, val.val1);
	}

#if defined(CONFIG_PM_DEVICE)
	printf("[%lld] before enter dlps\n", k_uptime_get());
	printf("[%lld] type on shell to wakeup\n", k_uptime_get());
	dlps_check_flag = PM_TEST_CHECK_PASS;
	k_sem_init(&app_sem, 0, 1);
	k_sem_take(&app_sem, K_FOREVER);

	dlps_check_flag = PM_TEST_CHECK_FAIL;

	/* rx 1 byte to wakeup */
	printf("[%lld] after exit dlps\n", k_uptime_get());
#endif

	for (int i = 0; i < 10; i++) {
		toggle_a = !toggle_a;
		if (!toggle_a) {
			gpio_pin_toggle_dt(&aon_qdec_phase_a);
		} else {
			gpio_pin_toggle_dt(&aon_qdec_phase_b);
		}
		k_busy_wait(100000);
		sensor_sample_fetch(aon_qdec_dev);
		sensor_channel_get(aon_qdec_dev, SENSOR_CHAN_ROTATION, &val);
		printf("Position[%d] = %d degrees\n", i, val.val1);
	}

#if defined(CONFIG_PM_DEVICE)
	printf("[%lld] before enter dlps\n", k_uptime_get());
	printf("[%lld] type on shell to wakeup\n", k_uptime_get());
	dlps_check_flag = PM_TEST_CHECK_PASS;
	k_sem_init(&app_sem, 0, 1);
	k_sem_take(&app_sem, K_FOREVER);

	dlps_check_flag = PM_TEST_CHECK_FAIL;

	/* rx 1 byte to wakeup */
	printf("[%lld] after exit dlps\n", k_uptime_get());
#endif

	for (int i = 0; i < 10; i++) {
		toggle_a = !toggle_a;
		if (!toggle_a) {
			gpio_pin_toggle_dt(&aon_qdec_phase_a);
		} else {
			gpio_pin_toggle_dt(&aon_qdec_phase_b);
		}
		k_busy_wait(100000);
		sensor_sample_fetch(aon_qdec_dev);
		sensor_channel_get(aon_qdec_dev, SENSOR_CHAN_ROTATION, &val);
		printf("Position[%d] = %d degrees\n", i, val.val1);
	}

#if defined(CONFIG_PM_DEVICE)
	printf("[%lld] before enter dlps\n", k_uptime_get());
	printf("[%lld] type on shell to wakeup\n", k_uptime_get());
	dlps_check_flag = PM_TEST_CHECK_PASS;
	k_sem_init(&app_sem, 0, 1);
	k_sem_take(&app_sem, K_FOREVER);

	dlps_check_flag = PM_TEST_CHECK_FAIL;

	/* rx 1 byte to wakeup */
	printf("[%lld] after exit dlps\n", k_uptime_get());
#endif

	for (int i = 0; i < 10; i++) {
		toggle_a = !toggle_a;
		if (toggle_a) {
			gpio_pin_toggle_dt(&aon_qdec_phase_a);
		} else {
			gpio_pin_toggle_dt(&aon_qdec_phase_b);
		}
		k_busy_wait(100000);
		sensor_sample_fetch(aon_qdec_dev);
		sensor_channel_get(aon_qdec_dev, SENSOR_CHAN_ROTATION, &val);
		printf("Position[%d] = %d degrees\n", i, val.val1);
	}
	return 0;
}
#endif

#ifdef CONFIG_I2C
const struct device *const i2c_dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(test_i2c));

static int shell_pm_test_i2c(const struct shell *sh, size_t argc, char **argv)
{
	unsigned char icm20618_addr = 0x68;
	unsigned char write_buf[6], read_buf[12];
	int write_len, read_len;
	/* ==================================================================================== */
	(void)memset(write_buf, 0, sizeof(write_buf));
	(void)memset(read_buf, 0, sizeof(read_buf));
	/* read id */
	write_buf[0] = 0x00;
	write_len = 1;
	read_len = 1;
	i2c_write_read(i2c_dev, icm20618_addr, write_buf, write_len, read_buf, read_len);
	printf("icm20618 addr:0x%x reg: 0x%x = 0x%x\n", icm20618_addr, write_buf[0], read_buf[0]);

	/* ==================================================================================== */
#if defined(CONFIG_PM_DEVICE)
	/* enter dlps */
	printf("[%lld] before enter dlps\n", k_uptime_get());
	printf("[%lld] type on shell to wakeup\n", k_uptime_get());
	dlps_check_flag = PM_TEST_CHECK_PASS;
	k_sem_init(&app_sem, 0, 1);
	k_sem_take(&app_sem, K_FOREVER);

	dlps_check_flag = PM_TEST_CHECK_FAIL;

	/* rx 1 byte to wakeup */
	printf("[%lld] after exit dlps\n", k_uptime_get());
#endif

	/* ==================================================================================== */
	(void)memset(write_buf, 0, sizeof(write_buf));
	(void)memset(read_buf, 0, sizeof(read_buf));
	/* read id */
	write_buf[0] = 0x00;
	write_len = 1;
	read_len = 1;
	i2c_write_read(i2c_dev, icm20618_addr, write_buf, write_len, read_buf, read_len);
	printf("icm20618 addr:0x%x reg: 0x%x = 0x%x\n", icm20618_addr, write_buf[0], read_buf[0]);
	/* ==================================================================================== */

	return 0;
}
#endif

#ifdef CONFIG_ADC
#define DT_SPEC_AND_COMMA(node_id, prop, idx) ADC_DT_SPEC_GET_BY_IDX(node_id, idx),
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels, DT_SPEC_AND_COMMA)};

#define ADC_BUFFER_SIZE   2
#define INVALID_ADC_VALUE SHRT_MIN

static const int adc_channels_count = ARRAY_SIZE(adc_channels);

static int32_t m_sample_buffer[ADC_BUFFER_SIZE];
static uint8_t m_samplings_done;

static void check_samples(int expected_count)
{
	printf("Samples read: ");
	for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
		int32_t sample_value = m_sample_buffer[i];

		printf("[%u]:%04hd ", i, sample_value);
		if (i < expected_count) {
			if (INVALID_ADC_VALUE == sample_value) {
				printf("[%u]:%4d should be filled ", i, sample_value);
			}
		} else {
			if (INVALID_ADC_VALUE != sample_value) {
				printf("[%u]:%4d should be %d ", i, sample_value,
				       INVALID_ADC_VALUE);
			}
		}
	}
	printf("\n");
}

static enum adc_action repeated_samplings_callback(const struct device *dev,
						   const struct adc_sequence *sequence,
						   uint16_t sampling_index)
{
	++m_samplings_done;
	printf("%s: done %d\n", __func__, m_samplings_done);
	if (m_samplings_done == 1U) {
		check_samples(MIN(adc_channels_count, 2));

		/* After first sampling continue normally. */
		return ADC_ACTION_CONTINUE;
	}

	check_samples(2 * MIN(adc_channels_count, 2));

	/*
	 * The second sampling is repeated 9 times (the samples are
	 * written in the same place), then the sequence is finished
	 * prematurely.
	 */
	if (m_samplings_done < 10) {
		return ADC_ACTION_REPEAT;
	}

	return ADC_ACTION_FINISH;
}

static int shell_pm_test_adc(const struct shell *sh, size_t argc, char **argv)
{
	/* ==================================================================================== */
	const struct adc_sequence_options options = {
		.callback = repeated_samplings_callback,
		.extra_samplings = 2,
		.interval_us = 0,
	};
	struct adc_sequence sequence = {
		.options = &options,
		.buffer = m_sample_buffer,
		.buffer_size = sizeof(m_sample_buffer),
		.resolution = 12,
	};

	for (uint8_t i = 0; i < adc_channels_count; i++) {
		adc_channel_setup_dt(&adc_channels[i]);
	}

	(void)adc_sequence_init_dt(&adc_channels[0], &sequence);
	printf("adc_channels_count=%d, adc_channels[0].channel_id=%d\n", adc_channels_count,
	       adc_channels[0].channel_id);
	if (adc_channels_count > 1) {
		sequence.channels |= BIT(adc_channels[1].channel_id);
	}

	for (uint8_t i = 0; i < ADC_BUFFER_SIZE; ++i) {
		m_sample_buffer[i] = INVALID_ADC_VALUE;
	}

	m_samplings_done = 0;

	adc_read_dt(&adc_channels[0], &sequence);

	/* ==================================================================================== */
#if defined(CONFIG_PM_DEVICE)
	/* enter dlps */
	printf("[%lld] before enter dlps\n", k_uptime_get());
	printf("[%lld] type on shell to wakeup\n", k_uptime_get());
	dlps_check_flag = PM_TEST_CHECK_PASS;
	k_sem_init(&app_sem, 0, 1);
	k_sem_take(&app_sem, K_FOREVER);

	dlps_check_flag = PM_TEST_CHECK_FAIL;

	/* rx 1 byte to wakeup */
	printf("[%lld] after exit dlps\n", k_uptime_get());
#endif

	/* ==================================================================================== */
	for (uint8_t i = 0; i < ADC_BUFFER_SIZE; ++i) {
		m_sample_buffer[i] = INVALID_ADC_VALUE;
	}

	m_samplings_done = 0;

	adc_read_dt(&adc_channels[0], &sequence);
	/* ==================================================================================== */

	return 0;
}
#endif

#if defined(CONFIG_SDMMC_STACK) || defined(CONFIG_SDIO_STACK)
static const struct device *const sdhc_dev_sdmmc = DEVICE_DT_GET_OR_NULL(DT_ALIAS(test_sdmmc));
static const struct device *const sdhc_dev_sdio = DEVICE_DT_GET_OR_NULL(DT_ALIAS(test_sdio));
uint8_t sdmmc_wbuf[512];
uint8_t sdmmc_rbuf[512];

static int shell_pm_test_sdhc(const struct shell *sh, size_t argc, char **argv)
{
	static struct sd_card sdmmc_card = {0}, sdio_card = {0};
	int ret;
	/* ==================================================================================== */

	if (sdhc_dev_sdmmc) {
		memset(sdmmc_rbuf, 0, sizeof(sdmmc_rbuf));

		for (uint32_t i = 0; i < sizeof(sdmmc_wbuf); i++) {
			sdmmc_wbuf[i] = i;
		}

		printf("before dlps sdmmc card %s initializing...\n", sdhc_dev_sdmmc->name);
		ret = sd_init(sdhc_dev_sdmmc, &sdmmc_card);

		if (ret != 0) {
			printf("before dlps sdmmc card initialization failed\n");
			return 0;
		}

		printf("before dlps sdmmc card %s  initialization success\n", sdhc_dev_sdmmc->name);

		sdmmc_write_blocks(&sdmmc_card, sdmmc_wbuf, 0, 1);
		sdmmc_read_blocks(&sdmmc_card, sdmmc_rbuf, 0, 1);
		if (memcmp(sdmmc_rbuf, sdmmc_wbuf, sizeof(sdmmc_rbuf))) {
			printf("before dlps sdmmc card read fail\n");
		} else {
			printf("before dlps sdmmc card read success\n");
		}
	}

	if (sdhc_dev_sdio) {
		printf("before dlps sdio card %s initializing...\n", sdhc_dev_sdio->name);
		ret = sd_init(sdhc_dev_sdio, &sdio_card);

		if (ret != 0) {
			printf("before dlps sdio card initialization failed\n");
			return 0;
		}

		printf("before dlps sdio card %s  initialization success\n", sdhc_dev_sdio->name);
	}

	/* ==================================================================================== */
#if defined(CONFIG_PM_DEVICE)
	/* enter dlps */
	printf("[%lld] before enter dlps\n", k_uptime_get());
	printf("[%lld] type on shell to wakeup\n", k_uptime_get());
	dlps_check_flag = PM_TEST_CHECK_PASS;
	k_sem_init(&app_sem, 0, 1);
	k_sem_take(&app_sem, K_FOREVER);

	dlps_check_flag = PM_TEST_CHECK_FAIL;

	/* rx 1 byte to wakeup */
	printf("[%lld] after exit dlps\n", k_uptime_get());
#endif
	/* ==================================================================================== */

	if (sdhc_dev_sdmmc) {
		memset(sdmmc_rbuf, 0, sizeof(sdmmc_rbuf));

		for (uint32_t i = 0; i < sizeof(sdmmc_wbuf); i++) {
			sdmmc_wbuf[i] = i * 3;
		}

		sdmmc_write_blocks(&sdmmc_card, sdmmc_wbuf, 0, 1);
		sdmmc_read_blocks(&sdmmc_card, sdmmc_rbuf, 0, 1);
		if (memcmp(sdmmc_rbuf, sdmmc_wbuf, sizeof(sdmmc_rbuf))) {
			printf("after dlps sdmmc card read fail\n");
		} else {
			printf("after dlps sdmmc card read success\n");
		}
	}

	if (sdhc_dev_sdio) {
		printf("after dlps sdio card %s initializing...\n", sdhc_dev_sdio->name);
		ret = sd_init(sdhc_dev_sdio, &sdio_card);

		if (ret != 0) {
			printf("after dlps sdio card initialization failed\n");
			return 0;
		}

		printf("after dlps sdio card %s  initialization success\n", sdhc_dev_sdio->name);
	}
	/* ==================================================================================== */
	return 0;
}
#endif

#ifdef CONFIG_CAN
static const struct device *const can_dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(test_can));
static volatile bool can_rx_received;

static void can_tx_callback(const struct device *dev, int error, void *user_data)
{
	printf("dev %s tx cb\n", dev->name);
}

static void can_rx_callback(const struct device *dev, struct can_frame *frame, void *user_data)
{
	printf("dev %s rx cb reecive id:0x%x, %ddata: ", dev->name, frame->id, frame->dlc);

	for (uint8_t i = 0; i < frame->dlc; i++) {
		printf("0x%02x ", frame->data[i]);
	}

	printf("\n");

	can_rx_received = true;
}

static int shell_pm_test_can(const struct shell *sh, size_t argc, char **argv)
{
	int ret;
	struct can_frame frame = {0};
	struct can_filter filter;

	/* ==================================================================================== */
	frame.flags = 0;
	frame.dlc = 0;
	frame.id = 0x123;
	frame.dlc = 8;

	filter.flags = 0U;
	filter.id = 0x456;
	filter.mask = 0x3ff;

	for (uint8_t i = 0; i < frame.dlc; i++) {
		frame.data[i] = i;
	}

	ret = can_start(can_dev);
	if (ret != 0) {
		printf("failed to start CAN controller (ret %d)\n", ret);
		return ret;
	}

	can_send(can_dev, &frame, K_NO_WAIT, can_tx_callback, NULL);

	k_busy_wait(10000);

	can_rx_received = false;
	can_add_rx_filter(can_dev, can_rx_callback, NULL, &filter);

	printf("waiting for frame with id 0x456 from tool\n");
	while (can_rx_received == false)
		;

	/* ==================================================================================== */
#if defined(CONFIG_PM_DEVICE)
	/* enter dlps */
	printf("[%lld] before enter dlps\n", k_uptime_get());
	printf("[%lld] type on shell to wakeup\n", k_uptime_get());
	dlps_check_flag = PM_TEST_CHECK_PASS;
	k_sem_init(&app_sem, 0, 1);
	k_sem_take(&app_sem, K_FOREVER);

	dlps_check_flag = PM_TEST_CHECK_FAIL;

	/* rx 1 byte to wakeup */
	printf("[%lld] after exit dlps\n", k_uptime_get());
#endif

	/* ==================================================================================== */
	can_send(can_dev, &frame, K_NO_WAIT, can_tx_callback, NULL);

	k_busy_wait(10000);

	can_rx_received = false;
	can_add_rx_filter(can_dev, can_rx_callback, NULL, &filter);

	printf("waiting for frame with id 0x456 from tool\n");
	while (can_rx_received == false)
		;

	ret = can_stop(can_dev);
	if (ret != 0) {
		printf("failed to stop CAN controller (ret %d)\n", ret);
		return ret;
	}
	/* ==================================================================================== */
	return 0;
}
#endif

#if defined(CONFIG_SERIAL)
#define SHELL_CMD_UART_PM_TEST SHELL_CMD_ARG(uart, NULL, "uart pm test", shell_pm_test_uart, 0, 0),
#else
#define SHELL_CMD_UART_PM_TEST
#endif

#if defined(CONFIG_UART_ASYNC_API)
#define SHELL_CMD_UART_DMA_PM_TEST                                                                 \
	SHELL_CMD_ARG(uartdma, NULL, "Uart dma pm test", shell_pm_test_uart_dma, 0, 0),
#else
#define SHELL_CMD_UART_DMA_PM_TEST
#endif

#if defined(CONFIG_GPIO)
#define SHELL_CMD_GPIO_PM_TEST SHELL_CMD_ARG(gpio, NULL, "gpio pm test", shell_pm_test_gpio, 0, 0),
#else
#define SHELL_CMD_GPIO_PM_TEST
#endif

#if defined(CONFIG_PWM)
#define SHELL_CMD_PWM_PM_TEST SHELL_CMD_ARG(pwm, NULL, "pwm pm test", shell_pm_test_pwm, 0, 0),
#else
#define SHELL_CMD_PWM_PM_TEST
#endif

#if defined(CONFIG_COUNTER)
#define SHELL_CMD_COUNTER_PM_TEST                                                                  \
	SHELL_CMD_ARG(counter, NULL, "counter pm test(input a time in ms)", shell_pm_test_counter, \
		      2, 0),
#else
#define SHELL_CMD_COUNTER_PM_TEST
#endif

#if defined(CONFIG_SPI)
#define SHELL_CMD_SPI_PM_TEST SHELL_CMD_ARG(spi, NULL, "spi pm test", shell_pm_test_spi, 0, 0),
#else
#define SHELL_CMD_SPI_PM_TEST
#endif

#if defined(CONFIG_RTC)
#define SHELL_CMD_RTC_PM_TEST SHELL_CMD_ARG(rtc, NULL, "rtc pm test", shell_pm_test_rtc, 0, 0),
#else
#define SHELL_CMD_RTC_PM_TEST
#endif

#if defined(CONFIG_I2C)
#define SHELL_CMD_I2C_PM_TEST SHELL_CMD_ARG(i2c, NULL, "i2c pm test", shell_pm_test_i2c, 0, 0),
#else
#define SHELL_CMD_I2C_PM_TEST
#endif

#if defined(CONFIG_ADC)
#define SHELL_CMD_ADC_PM_TEST SHELL_CMD_ARG(adc, NULL, "adc pm test", shell_pm_test_adc, 0, 0),
#else
#define SHELL_CMD_ADC_PM_TEST
#endif

#if defined(CONFIG_QDEC_BEE)
#define SHELL_CMD_QDEC_PM_TEST SHELL_CMD_ARG(qdec, NULL, "qdec pm test", shell_pm_test_qdec, 0, 0),
#else
#define SHELL_CMD_QDEC_PM_TEST
#endif

#if defined(CONFIG_AON_QDEC_BEE)
#define SHELL_CMD_AONQDEC_PM_TEST                                                                  \
	SHELL_CMD_ARG(aon_qdec, NULL, "aon_qdec pm test", shell_pm_test_aon_qdec, 0, 0),
#else
#define SHELL_CMD_AONQDEC_PM_TEST
#endif

#if defined(CONFIG_SDMMC_STACK) || defined(CONFIG_SDIO_STACK)
#define SHELL_CMD_SHDC_PM_TEST SHELL_CMD_ARG(sdhc, NULL, "sdhc pm test", shell_pm_test_sdhc, 0, 0),
#else
#define SHELL_CMD_SHDC_PM_TEST
#endif

#if defined(CONFIG_CAN)
#define SHELL_CMD_CAN_PM_TEST SHELL_CMD_ARG(can, NULL, "can pm test", shell_pm_test_can, 0, 0),
#else
#define SHELL_CMD_CAN_PM_TEST
#endif

#define SHELL_CMD_ARG_CREATE                                                                       \
	SHELL_CMD_UART_PM_TEST                                                                     \
	SHELL_CMD_UART_DMA_PM_TEST                                                                 \
	SHELL_CMD_GPIO_PM_TEST                                                                     \
	SHELL_CMD_PWM_PM_TEST                                                                      \
	SHELL_CMD_COUNTER_PM_TEST                                                                  \
	SHELL_CMD_SPI_PM_TEST                                                                      \
	SHELL_CMD_RTC_PM_TEST                                                                      \
	SHELL_CMD_QDEC_PM_TEST                                                                     \
	SHELL_CMD_AONQDEC_PM_TEST                                                                  \
	SHELL_CMD_I2C_PM_TEST                                                                      \
	SHELL_CMD_ADC_PM_TEST                                                                      \
	SHELL_CMD_SHDC_PM_TEST                                                                     \
	SHELL_CMD_CAN_PM_TEST                                                                      \
	SHELL_SUBCMD_SET_END /* Array terminated. */
SHELL_STATIC_SUBCMD_SET_CREATE(sub_pm_test, SHELL_CMD_ARG_CREATE);

SHELL_CMD_REGISTER(pm_test, &sub_pm_test, "Pm test", NULL);
