/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
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

#ifdef CONFIG_INPUT
#include <zephyr/input/input.h>
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
#if defined(CONFIG_PM_DEVICE)

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
#endif
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#if defined(CONFIG_PM_DEVICE)

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
#endif
#elif defined(CONFIG_SOC_SERIES_RTL87X2J)

#include "log_core.h"
#include <debug_port.h>
#include <pck600.h>

#endif /* SoC select */

/* Device declarations */

#ifdef CONFIG_UART_ASYNC_API
static const struct device *uart_dma_dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(test_uart_dma));
#endif

#ifdef CONFIG_COUNTER
static const struct device *counter_dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(test_counter_timer));
#endif

#ifdef CONFIG_GPIO
#define DEV_OUT DT_GPIO_CTLR(DT_INST(0, test_gpio_basic_api), out_gpios)
#define DEV_IN  DT_GPIO_CTLR(DT_INST(0, test_gpio_basic_api), in_gpios)

#define PIN_OUT       DT_GPIO_PIN(DT_INST(0, test_gpio_basic_api), out_gpios)
#define PIN_OUT_FLAGS DT_GPIO_FLAGS(DT_INST(0, test_gpio_basic_api), out_gpios)
#define PIN_IN        DT_GPIO_PIN(DT_INST(0, test_gpio_basic_api), in_gpios)
#define PIN_IN_FLAGS  DT_GPIO_FLAGS(DT_INST(0, test_gpio_basic_api), in_gpios)

static const struct device *const dev_in = DEVICE_DT_GET_OR_NULL(DEV_IN);
static const struct device *const dev_out = DEVICE_DT_GET_OR_NULL(DEV_OUT);
#endif

#ifdef CONFIG_PWM
static const struct device *pwm_dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(test_pwm));
static const struct device *lppwm_dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(test_lppwm));
#endif

#ifdef CONFIG_SPI
#define MODE_LOOP  0
#define FRAME_SIZE 8
#define SPI_OP_CFG(fs)                                                                             \
	(SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_WORD_SET(fs) | SPI_LINES_SINGLE)

#define SPI_DEV  DT_COMPAT_GET_ANY_STATUS_OKAY(test_spi_loopback)
#define BUF_SIZE 18

static struct spi_dt_spec spi_spec = SPI_DT_SPEC_GET(SPI_DEV, SPI_OP_CFG(FRAME_SIZE));
static const char tx_data[BUF_SIZE] = "0123456789abcdef-\0";
static __aligned(32) char spi_tx_buf[BUF_SIZE];
static __aligned(32) char spi_rx_buf[BUF_SIZE];
#endif

#ifdef CONFIG_RTC
static const struct device *rtc_dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(test_rtc));
static uint64_t current_sys_time_ms;
#endif

#if defined(CONFIG_SENSOR) && defined(CONFIG_QDEC_BEE)
static const struct device *qdec_dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(test_qdec));
static const struct gpio_dt_spec phase_a = GPIO_DT_SPEC_GET(DT_ALIAS(test_qenca), gpios);
static const struct gpio_dt_spec phase_b = GPIO_DT_SPEC_GET(DT_ALIAS(test_qencb), gpios);
#endif

#ifdef CONFIG_I2C
static const struct device *i2c_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(i2c0));
#endif

#if defined(CONFIG_SDMMC_STACK) || defined(CONFIG_SDIO_STACK)
static const struct device *sdhc_dev_sdmmc = DEVICE_DT_GET_OR_NULL(DT_ALIAS(test_sdmmc));
static const struct device *sdhc_dev_sdio = DEVICE_DT_GET_OR_NULL(DT_ALIAS(test_sdio));
static struct sd_card sdmmc_card;
static struct sd_card sdio_card;
#endif

#ifdef CONFIG_SDMMC_STACK
static uint8_t sdmmc_wbuf[512];
static uint8_t sdmmc_rbuf[512];
#endif

#ifdef CONFIG_CAN
static const struct device *can_dev = DEVICE_DT_GET_OR_NULL(DT_ALIAS(test_can));
#endif

/* PM state */

#if defined(CONFIG_PM_DEVICE)

struct k_sem pm_app_sem;

static PM_TEST_CHECK_RET pm_dlps_check_flag = PM_TEST_CHECK_FAIL;
static uint32_t pm_counter;

static PM_TEST_CHECK_RET app_check(void)
{
	return pm_dlps_check_flag;
}

static void app_store(void)
{
	DBG_DIRECT("[%s] %d line %d", __func__, ++pm_counter, __LINE__);
}

static void app_restore(void)
{
	DBG_DIRECT("[%s] %d line %d", __func__, pm_counter, __LINE__);
	k_sem_give(&pm_app_sem);
}

static void pm_test_enter_dlps_forever(void)
{
	printf("[%lld] before enter dlps\n", k_uptime_get());
	printf("[%lld] type on shell to wakeup\n", k_uptime_get());

	pm_dlps_check_flag = PM_TEST_CHECK_PASS;
	k_sem_init(&pm_app_sem, 0, 1);
	k_sem_take(&pm_app_sem, K_FOREVER);
	pm_dlps_check_flag = PM_TEST_CHECK_FAIL;

	printf("[%lld] after exit dlps\n", k_uptime_get());
}

static void pm_test_enter_dlps_timeout(k_timeout_t timeout)
{
	printf("[%lld] before enter dlps\n", k_uptime_get());

	pm_dlps_check_flag = PM_TEST_CHECK_PASS;
	k_sem_init(&pm_app_sem, 0, 1);
	k_sem_take(&pm_app_sem, timeout);
	pm_dlps_check_flag = PM_TEST_CHECK_FAIL;

	printf("[%lld] after exit dlps\n", k_uptime_get());
}

#endif /* CONFIG_PM_DEVICE */

int main(void)
{
	printf("[%lld] Hello World! %s\n", k_uptime_get(), CONFIG_BOARD_TARGET);

#if defined(CONFIG_SOC_SERIES_RTL87X2J)
	debug_port_aon_output(DEBUG_PCK600_OUTPUT_TO_VPON_PPU, ENABLE);
	printf("[%lld] wakeup_count=%d\n", k_uptime_get(), pck600_system_get_wakeup_count(NULL));
#endif

#if defined(CONFIG_PM_DEVICE)
	k_sem_init(&pm_app_sem, 0, 1);

	pm_dlps_check_flag = PM_TEST_CHECK_FAIL;

	pm_test_register_check_cb(app_check);
	pm_test_register_store_cb(app_store);
	pm_test_register_restore_cb(app_restore);
#endif

#ifdef CONFIG_GPIO
	gpio_pin_configure(dev_out, PIN_OUT, GPIO_OUTPUT_HIGH | PIN_OUT_FLAGS);
#endif

#ifdef CONFIG_SENSOR
	gpio_pin_configure_dt(&phase_a, GPIO_OUTPUT);
	gpio_pin_configure_dt(&phase_b, GPIO_OUTPUT);
#endif

	return 0;
}

/* UART PM test */

static int shell_pm_test_uart(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#ifdef CONFIG_SERIAL
#if defined(CONFIG_PM_DEVICE)
	pm_test_enter_dlps_forever();
#endif
#endif

	return 0;
}

/* UART DMA PM test */

#ifdef CONFIG_UART_ASYNC_API

static struct k_sem uart_dma_tx_sem;
static struct k_sem uart_dma_rx_sem;

static uint8_t uart_dma_rx_buf[1024];
static uint32_t uart_dma_rx_len;
static bool uart_dma_rx_enabled;

static void uart_async_console_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	switch (evt->type) {
	case UART_TX_DONE:
		k_sem_give(&uart_dma_tx_sem);
		break;

	case UART_RX_RDY:
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
}

static void uart_dma_exit_cb(void)
{
	if (uart_dma_rx_enabled) {
		/* Re-enable DMA RX after wakeup if needed (left intentionally empty) */
	}
}

static void pm_uart_dma_do_rx_tx_cycle(const struct device *dev)
{
	k_sem_init(&uart_dma_tx_sem, 0, 1);
	k_sem_init(&uart_dma_rx_sem, 0, 1);
	memset(uart_dma_rx_buf, 0, sizeof(uart_dma_rx_buf));
	uart_dma_rx_len = 0;

	uart_rx_enable(dev, uart_dma_rx_buf, sizeof(uart_dma_rx_buf), 50 * USEC_PER_MSEC);

	printf("send some data from dma uart\n");
	k_sem_take(&uart_dma_rx_sem, K_FOREVER);

	printf("uart dma rx %d bytes\n", uart_dma_rx_len);

	uart_tx(dev, uart_dma_rx_buf, uart_dma_rx_len, 100 * USEC_PER_MSEC);
	k_sem_take(&uart_dma_tx_sem, K_FOREVER);
}

#endif /* CONFIG_UART_ASYNC_API */

static int shell_pm_test_uart_dma(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#ifdef CONFIG_UART_ASYNC_API
#if defined(CONFIG_SOC_SERIES_RTL87X2G) || defined(CONFIG_SOC_SERIES_RTL8752H)
	static bool pm_uart_dma_cb_registered;

	if (!pm_uart_dma_cb_registered) {
		pm_test_register_store_cb(uart_dma_enter_cb);
		pm_test_register_restore_cb(uart_dma_exit_cb);
		pm_uart_dma_cb_registered = true;
	}
#endif
	uart_callback_set(uart_dma_dev, uart_async_console_cb, NULL);

	uart_dma_rx_enabled = true;

	/* First DMA RX/TX cycle before entering DLPS */
	pm_uart_dma_do_rx_tx_cycle(uart_dma_dev);

	/* Enter DLPS */
#if defined(CONFIG_PM_DEVICE)
	pm_test_enter_dlps_forever();
#endif

	/* Second DMA RX/TX cycle after exiting DLPS */
	k_sem_init(&uart_dma_tx_sem, 0, 1);
	k_sem_init(&uart_dma_rx_sem, 0, 1);
	memset(uart_dma_rx_buf, 0, sizeof(uart_dma_rx_buf));
	uart_dma_rx_len = 0;

	uart_rx_disable(uart_dma_dev);
	pm_uart_dma_do_rx_tx_cycle(uart_dma_dev);

	uart_rx_disable(uart_dma_dev);
	uart_dma_rx_enabled = false;

#endif /* CONFIG_UART_ASYNC_API */

	return 0;
}

/* COUNTER PM test */

#ifdef CONFIG_COUNTER
static struct k_sem pm_counter_sem;

static void counter_top_cb(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);

	uint64_t *pre_sys_time_ms = (uint64_t *)user_data;
	uint64_t now_ms = k_uptime_get();

	printf("top_handler\n");
#if defined(CONFIG_SOC_SERIES_RTL87X2J)
	printf("[%lld] wakeup_count=%d\n", k_uptime_get(), pck600_system_get_wakeup_count(NULL));
#endif
	printf("[%lld] trigger handler after %lldms\n", now_ms, now_ms - *pre_sys_time_ms);

#if defined(CONFIG_PM_DEVICE)
	k_sem_give(&pm_app_sem);
#else
	k_sem_give(&pm_counter_sem);
#endif
}

#endif /* CONFIG_COUNTER */

static int shell_pm_test_counter(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);

#if defined(CONFIG_SOC_SERIES_RTL87X2J)
	printf("[%lld] wakeup_count=%d\n", k_uptime_get(), pck600_system_get_wakeup_count(NULL));
#endif

#ifdef CONFIG_COUNTER
	struct counter_top_cfg top_cfg;
	uint64_t pre_sys_time_ms;
	uint64_t timeout_ms;

	if (argc < 2) {
		printf("Usage: pm_test counter <timeout_ms>\n");
		return 0;
	}

	k_sem_init(&pm_counter_sem, 0, 1);

	timeout_ms = strtoul(argv[1], NULL, 10);

	counter_start(counter_dev);

	top_cfg.callback = counter_top_cb;
	top_cfg.flags = 0;
	top_cfg.ticks = counter_us_to_ticks(counter_dev, timeout_ms * 1000);
	pre_sys_time_ms = k_uptime_get();
	top_cfg.user_data = &pre_sys_time_ms;

	printf("[%lld] wait %lldms to trigger handler\n", pre_sys_time_ms, timeout_ms);

	counter_set_top_value(counter_dev, &top_cfg);

#if defined(CONFIG_PM_DEVICE)
	pm_test_enter_dlps_forever();
#else
	k_sem_take(&pm_counter_sem, K_FOREVER);
#endif

	counter_stop(counter_dev);
#endif /* CONFIG_COUNTER */

	return 0;
}

/* GPIO PM test */

#ifdef CONFIG_GPIO

static struct gpio_callback pm_gpio_cb;
static struct k_sem pm_gpio_sem;

static void pm_gpio_irq_cb(const struct device *dev_in, struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(dev_in);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	static uint8_t irq_count;

#if defined(CONFIG_PM_DEVICE)
	pm_dlps_check_flag = PM_TEST_CHECK_FAIL;
#endif

	k_sem_give(&pm_gpio_sem);

#if defined(CONFIG_SOC_SERIES_RTL87X2J)
	printf("[%lld] wakeup_count=%d\n", k_uptime_get(), pck600_system_get_wakeup_count(NULL));
#endif
	printf("[%lld] enter gpio callback cnt %d\n", k_uptime_get(), irq_count);
	irq_count++;
}

static void pm_gpio_do_one_round(const char *hint)
{
#if defined(CONFIG_PM_DEVICE)
	printf("[%lld] before enter dlps\n", k_uptime_get());
	printf("[%lld] %s\n", k_uptime_get(), hint);

	pm_dlps_check_flag = PM_TEST_CHECK_PASS;
	k_sem_init(&pm_app_sem, 0, 1);
	k_sem_take(&pm_app_sem, K_FOREVER);
	pm_dlps_check_flag = PM_TEST_CHECK_FAIL;

	printf("[%lld] after exit dlps\n", k_uptime_get());
#else
	printf("[%lld] %s\n", k_uptime_get(), hint);
	k_sem_take(&pm_gpio_sem, K_FOREVER);
#endif

	k_busy_wait(100000);
}

#endif /* CONFIG_GPIO */

static int shell_pm_test_gpio(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);

#if defined(CONFIG_SOC_SERIES_RTL87X2J)
	printf("[%lld] wakeup_count=%d\n", k_uptime_get(), pck600_system_get_wakeup_count(NULL));
#endif

	int debounce_ms = argc > 1 ? (int)strtoul(argv[1], NULL, 10) : 8;

#ifdef CONFIG_GPIO
	k_sem_init(&pm_gpio_sem, 0, 1);

	gpio_pin_configure(dev_out, PIN_OUT, GPIO_OUTPUT_LOW | PIN_OUT_FLAGS);

#if defined(CONFIG_GPIO_BEE)
	gpio_pin_configure(dev_in, PIN_IN,
			   GPIO_INPUT | GPIO_PULL_UP | BEE_GPIO_INPUT_DEBOUNCE_MS(debounce_ms)
#if defined(CONFIG_PM_DEVICE)
				   | BEE_GPIO_INPUT_PM_WAKEUP
#endif
	);
#endif

	gpio_init_callback(&pm_gpio_cb, pm_gpio_irq_cb, BIT(PIN_IN));
	gpio_add_callback(dev_in, &pm_gpio_cb);

	/* Falling edge */
	gpio_pin_interrupt_configure(dev_in, PIN_IN, GPIO_INT_EDGE_FALLING);
	pm_gpio_do_one_round("connect input pin to output pin to wakeup");

	/* Rising edge */
	gpio_pin_interrupt_configure(dev_in, PIN_IN, GPIO_INT_EDGE_RISING);
	pm_gpio_do_one_round("disconnect input pin to output pin to wakeup");

	/* Both edges */
	gpio_pin_interrupt_configure(dev_in, PIN_IN, GPIO_INT_EDGE_BOTH);
	pm_gpio_do_one_round("connect input pin to output pin to wakeup");
	pm_gpio_do_one_round("disconnect input pin to output pin to wakeup");

	gpio_remove_callback(dev_in, &pm_gpio_cb);
	gpio_pin_interrupt_configure(dev_in, PIN_IN, GPIO_INT_DISABLE);

#if defined(CONFIG_GPIO_BEE)
	gpio_pin_configure(dev_in, PIN_IN,
			   (GPIO_INPUT | GPIO_PULL_UP | PIN_IN_FLAGS)
#if defined(CONFIG_PM_DEVICE)
				   & (~BEE_GPIO_INPUT_PM_WAKEUP)
#endif
	);
#endif

#endif /* CONFIG_GPIO */

	return 0;
}

/* PWM PM test */

static int shell_pm_test_pwm(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if defined(CONFIG_SOC_SERIES_RTL87X2J)
	printf("[%lld] wakeup_count=%d\n", k_uptime_get(), pck600_system_get_wakeup_count(NULL));
#endif

#ifdef CONFIG_PWM
	uint32_t period;
	uint32_t pulse;

	printf("[%lld] connect pwm pin to LA to watch the waveform\n", k_uptime_get());

	/* First waveform */
	period = 50000;
	pulse = 10000;
	printf("[%lld] [PWM]: %s, [period]: %u, [pulse]: %u\n", k_uptime_get(), pwm_dev->name,
	       period, pulse);
	pwm_set_cycles(pwm_dev, 0, period, pulse, 0);
	k_busy_wait(500000);

#if defined(CONFIG_PM_DEVICE)
	/* Enter DLPS in the middle of PWM test */
	pm_test_enter_dlps_timeout(K_MSEC(500));
	k_busy_wait(500000);
#endif

	/* Stop PWM */
	period = 0;
	pulse = 0;
	printf("[%lld] [PWM]: %s, [period]: %u, [pulse]: %u\n", k_uptime_get(), pwm_dev->name,
	       period, pulse);
	pwm_set_cycles(pwm_dev, 0, period, pulse, 0);
	k_sleep(K_MSEC(500));

#if defined(CONFIG_PM_DEVICE)
	k_sleep(K_MSEC(10));
#endif

	/* Second waveform */
	period = 50000;
	pulse = 40000;
	printf("[%lld] [PWM]: %s, [period]: %u, [pulse]: %u\n", k_uptime_get(), pwm_dev->name,
	       period, pulse);
	pwm_set_cycles(pwm_dev, 0, period, pulse, 0);
	k_busy_wait(500000);

#if defined(CONFIG_PM_DEVICE)
	pm_test_enter_dlps_timeout(K_MSEC(500));
	k_busy_wait(500000);
#endif

	/* Stop again */
	period = 0;
	pulse = 0;
	printf("[%lld] [PWM]: %s, [period]: %u, [pulse]: %u\n", k_uptime_get(), pwm_dev->name,
	       period, pulse);
	pwm_set_cycles(pwm_dev, 0, period, pulse, 0);

#endif /* CONFIG_PWM */

	return 0;
}

/* LPPWM PM test */

static int shell_pm_test_lppwm(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if defined(CONFIG_SOC_SERIES_RTL87X2J)
	printf("[%lld] wakeup_count=%d\n", k_uptime_get(), pck600_system_get_wakeup_count(NULL));
#endif

#ifdef CONFIG_PWM
	uint32_t period;
	uint32_t pulse;

	printf("[%lld] connect pwm pin to LA to watch the waveform\n", k_uptime_get());

	/* First waveform */
	period = 5 * 32;
	pulse = 1 * 32;
	printf("[%lld] [PWM]: %s, [period]: %u, [pulse]: %u\n", k_uptime_get(), lppwm_dev->name,
	       period, pulse);
	pwm_set_cycles(lppwm_dev, 0, period, pulse, 0);
	k_sleep(K_MSEC(500));

#if defined(CONFIG_PM_DEVICE)
	/* Enter DLPS in the middle of PWM test */
	pm_test_enter_dlps_timeout(K_MSEC(500));
	k_busy_wait(500000);
#endif

	/* Stop PWM */
	period = 0;
	pulse = 0;
	printf("[%lld] [PWM]: %s, [period]: %u, [pulse]: %u\n", k_uptime_get(), lppwm_dev->name,
	       period, pulse);
	pwm_set_cycles(lppwm_dev, 0, period, pulse, 0);
	k_sleep(K_MSEC(500));

#if defined(CONFIG_PM_DEVICE)
	k_sleep(K_MSEC(10));
#endif

	/* Second waveform */
	period = 5 * 32;
	pulse = 4 * 32;
	printf("[%lld] [PWM]: %s, [period]: %u, [pulse]: %u\n", k_uptime_get(), lppwm_dev->name,
	       period, pulse);
	pwm_set_cycles(lppwm_dev, 0, period, pulse, 0);
	k_sleep(K_MSEC(500));

#if defined(CONFIG_PM_DEVICE)
	pm_test_enter_dlps_timeout(K_MSEC(500));
	k_busy_wait(500000);
#endif

	/* Stop again */
	period = 0;
	pulse = 0;
	printf("[%lld] [PWM]: %s, [period]: %u, [pulse]: %u\n", k_uptime_get(), lppwm_dev->name,
	       period, pulse);
	pwm_set_cycles(lppwm_dev, 0, period, pulse, 0);

#endif /* CONFIG_PWM */

	return 0;
}

/* SPI PM test */

#ifdef CONFIG_SPI

static int spi_complete_loop(struct spi_dt_spec *spec)
{
	int ret;

	memcpy(spi_tx_buf, tx_data, sizeof(tx_data));
	memset(spi_rx_buf, 0, sizeof(spi_rx_buf));

	const struct spi_buf tx_bufs[] = {
		{
			.buf = spi_tx_buf,
			.len = BUF_SIZE,
		},
	};
	const struct spi_buf rx_bufs[] = {
		{
			.buf = spi_rx_buf,
			.len = BUF_SIZE,
		},
	};

	const struct spi_buf_set tx = {
		.buffers = tx_bufs,
		.count = ARRAY_SIZE(tx_bufs),
	};
	const struct spi_buf_set rx = {
		.buffers = rx_bufs,
		.count = ARRAY_SIZE(rx_bufs),
	};

	printf("[%lld] Start complete loop\n", k_uptime_get());

	ret = spi_transceive_dt(spec, &tx, &rx);
	if (ret) {
		printf("[%lld] spi_transceive_dt error: %d\n", k_uptime_get(), ret);
		return ret;
	}

	if (memcmp(spi_tx_buf, spi_rx_buf, BUF_SIZE)) {
		printf("[%lld] Buffer contents are different\n", k_uptime_get());
		return -1;
	}

	printf("[%lld] Buffer contents are same\n", k_uptime_get());
	return 0;
}

#endif /* CONFIG_SPI */

static int shell_pm_test_spi(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if defined(CONFIG_SOC_SERIES_RTL87X2J)
	printf("[%lld] wakeup_count=%d\n", k_uptime_get(), pck600_system_get_wakeup_count(NULL));
#endif

#ifdef CONFIG_SPI
	printf("[%lld] connect MOSI pin to the MISO of the SPI\n", k_uptime_get());

	if (spi_complete_loop(&spi_spec) < 0) {
		printf("[%lld] loopback test fail\n", k_uptime_get());
		return 0;
	}

#if defined(CONFIG_PM_DEVICE)
	pm_test_enter_dlps_forever();
#endif

	if (spi_complete_loop(&spi_spec) < 0) {
		printf("[%lld] loopback test fail\n", k_uptime_get());
		return 0;
	}

#endif /* CONFIG_SPI */

	return 0;
}

/* RTC PM test */

#ifdef CONFIG_RTC

static const struct rtc_time test_rtc_time_set = {
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

static void test_rtc_alarm_cb(const struct device *dev, uint16_t id, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(id);

	uint64_t *pre_sys_time_ms = (uint64_t *)user_data;
	uint64_t now_ms = k_uptime_get();

#if defined(CONFIG_SOC_SERIES_RTL87X2J)
	printf("[%lld] wakeup_count=%d\n", k_uptime_get(), pck600_system_get_wakeup_count(NULL));
#endif
	printf("[%lld] trigger handler after %lldms\n", now_ms,
	       now_ms - (*(uint64_t *)pre_sys_time_ms));

#if defined(CONFIG_PM_DEVICE)
	k_sem_give(&pm_app_sem);
#endif
}

#endif /* CONFIG_RTC */

static int shell_pm_test_rtc(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);

#if defined(CONFIG_SOC_SERIES_RTL87X2J)
	printf("[%lld] wakeup_count=%d\n", k_uptime_get(), pck600_system_get_wakeup_count(NULL));
#endif

#ifdef CONFIG_RTC
	uint32_t timeout_ms = 2000; /* default 2000 ms */

	/* Usage: pm_test rtc [timeout_ms] */
	if (argc > 2) {
		printf("Usage: pm_test rtc [timeout_ms]\n");
		return 0;
	}

	if (argc == 2) {
		timeout_ms = strtoul(argv[1], NULL, 10);
	}
	printf("Setting RTC alarm for %u ms\n", timeout_ms);
	rtc_alarm_set_callback(rtc_dev, 0, NULL, NULL);
	rtc_set_time(rtc_dev, &test_rtc_time_set);

	current_sys_time_ms = k_uptime_get();
	rtc_alarm_set_callback(rtc_dev, 0, test_rtc_alarm_cb, &current_sys_time_ms);

	struct rtc_time alarm_time = test_rtc_time_set;
	uint32_t timeout_sec = timeout_ms / 1000;

	alarm_time.tm_sec += timeout_sec;
	alarm_time.tm_min += alarm_time.tm_sec / 60;
	alarm_time.tm_sec %= 60;
	alarm_time.tm_hour += alarm_time.tm_min / 60;
	alarm_time.tm_min %= 60;

	rtc_alarm_set_time(rtc_dev, 0, 0x1ff, &alarm_time);

	printf("[%lld] wait %ums to trigger handler\n", current_sys_time_ms, timeout_ms);

#if defined(CONFIG_PM_DEVICE)
	pm_test_enter_dlps_forever();
#endif

#endif /* CONFIG_RTC */

	return 0;
}

#if defined(CONFIG_SENSOR) && defined(CONFIG_QDEC_BEE)
static int qdec_cb_count;

static void qdec_data_ready_cb(const struct device *dev, const struct sensor_trigger *trig)
{
	struct sensor_value val;

	ARG_UNUSED(trig);
	sensor_sample_fetch(dev);
	sensor_channel_get(dev, SENSOR_CHAN_QDEC_X_COUNT, &val);

#if defined(CONFIG_SOC_SERIES_RTL87X2J)
	printf("[%lld] wakeup_count=%d\n", k_uptime_get(), pck600_system_get_wakeup_count(NULL));
#endif
	printf("Position[%d] = %d degrees\n", qdec_cb_count++, val.val1);
}
#endif

/* QDEC PM test */

static int shell_pm_test_qdec(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if defined(CONFIG_SOC_SERIES_RTL87X2J)
	printf("[%lld] wakeup_count=%d\n", k_uptime_get(), pck600_system_get_wakeup_count(NULL));
#endif

#if defined(CONFIG_SENSOR) && defined(CONFIG_QDEC_BEE)

#if defined(CONFIG_PM_DEVICE)
	static bool toggle_a;

	struct sensor_value val;

	gpio_pin_configure_dt(&phase_a, GPIO_OUTPUT);
	gpio_pin_configure_dt(&phase_b, GPIO_OUTPUT);

	k_busy_wait(100000);

	/* First rotation */
	for (int i = 0; i < 12; i++) {
		toggle_a = !toggle_a;
		if (toggle_a) {
			gpio_pin_toggle_dt(&phase_a);
		} else {
			gpio_pin_toggle_dt(&phase_b);
		}

		k_busy_wait(100000);
		sensor_sample_fetch(qdec_dev);
		sensor_channel_get(qdec_dev, SENSOR_CHAN_QDEC_X_COUNT, &val);

		printf("Position[%d] = %d degrees\n", i, val.val1);
	}

#if defined(CONFIG_PM_DEVICE)
	pm_test_enter_dlps_forever();
#endif

	/* Additional rotations (logic kept identical to original) */
	for (int round = 0; round < 3; round++) {
		for (int i = 0; i < 12; i++) {
			toggle_a = !toggle_a;
			if ((round == 0 || round == 1) ? !toggle_a : toggle_a) {
				gpio_pin_toggle_dt(&phase_a);
			} else {
				gpio_pin_toggle_dt(&phase_b);
			}

			k_busy_wait(100000);
			sensor_sample_fetch(qdec_dev);
			sensor_channel_get(qdec_dev, SENSOR_CHAN_QDEC_X_COUNT, &val);
			printf("Position[%d] = %d degrees\n", i, val.val1);
		}

#if defined(CONFIG_PM_DEVICE)
		pm_test_enter_dlps_forever();
#endif
	}
#else
#if defined(CONFIG_QDEC_BEE)
	{
		struct sensor_trigger trig = {
			.type = SENSOR_TRIG_DATA_READY,
			.chan = (enum sensor_channel)SENSOR_CHAN_QDEC_X_COUNT,
		};

		qdec_cb_count = 0;
		sensor_trigger_set(qdec_dev, &trig, qdec_data_ready_cb);
		k_sleep(K_SECONDS(10));
		sensor_trigger_set(qdec_dev, &trig, NULL);
	}
#endif
#endif

#endif /* CONFIG_SENSOR && CONFIG_QDEC_BEE */

	return 0;
}

static int shell_pm_test_generate_waveform_gpio(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);

	int debounce_ms = argc > 1 ? (int)strtoul(argv[1], NULL, 10) : 8;
	uint32_t pulse_us = argc > 2 ? (uint32_t)strtoul(argv[2], NULL, 10) : 100;

#ifdef CONFIG_GPIO
	gpio_pin_configure(dev_out, PIN_OUT, GPIO_OUTPUT_HIGH | PIN_OUT_FLAGS);

	for (int i = 0; i < 6; i++) {
		int level = (i % 2 == 0) ? 1 : 0;

		gpio_pin_set(dev_out, PIN_OUT, level);
		k_busy_wait(debounce_ms * USEC_PER_MSEC);
		gpio_pin_set(dev_out, PIN_OUT, !level);
		k_busy_wait(pulse_us);

		gpio_pin_set(dev_out, PIN_OUT, level);
		k_busy_wait(500000);
		gpio_pin_set(dev_out, PIN_OUT, !level);
		k_busy_wait(debounce_ms * USEC_PER_MSEC);
		gpio_pin_set(dev_out, PIN_OUT, level);
		k_busy_wait(500000);
	}

	gpio_pin_configure(dev_out, PIN_OUT, GPIO_OUTPUT_HIGH | PIN_OUT_FLAGS);
#endif

	return 0;
}

static int shell_pm_test_generate_waveform_qdec(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if defined(CONFIG_SOC_SERIES_RTL87X2J)
	printf("[%lld] wakeup_count=%d\n", k_uptime_get(), pck600_system_get_wakeup_count(NULL));
#endif

#if defined(CONFIG_SENSOR) && defined(CONFIG_QDEC_BEE)
	/* false = forward (A first), true = reverse (B first) */
	const bool phase_init[] = {false, true, true, false};
	bool toggle_a = false;

	gpio_pin_configure_dt(&phase_a, GPIO_OUTPUT);
	gpio_pin_configure_dt(&phase_b, GPIO_OUTPUT);

	for (int i = 0; i < 40; i++) {
		if (i % 10 == 0) {
			/* Reset direction at the start of each phase. */
			toggle_a = phase_init[i / 10];
		}
		toggle_a = !toggle_a;
		if (toggle_a) {
			gpio_pin_toggle_dt(&phase_a);
		} else {
			gpio_pin_toggle_dt(&phase_b);
		}
		k_busy_wait(100000);
	}
#endif

	return 0;
}

/* I2C PM test */

static int shell_pm_test_i2c(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if defined(CONFIG_SOC_SERIES_RTL87X2J)
	printf("[%lld] wakeup_count=%d\n", k_uptime_get(), pck600_system_get_wakeup_count(NULL));
#endif

#ifdef CONFIG_I2C
	uint8_t icm20618_addr = 0x68;
	uint8_t write_buf[6];
	uint8_t read_buf[12];
	int write_len;
	int read_len;

	memset(write_buf, 0, sizeof(write_buf));
	memset(read_buf, 0, sizeof(read_buf));

	/* Read ID before DLPS */
	write_buf[0] = 0x00;
	write_len = 1;
	read_len = 1;

	i2c_write_read(i2c_dev, icm20618_addr, write_buf, write_len, read_buf, read_len);

	printf("icm20618 addr:0x%x reg:0x%x = 0x%x\n", icm20618_addr, write_buf[0], read_buf[0]);

#if defined(CONFIG_PM_DEVICE)
	pm_test_enter_dlps_forever();
#endif

	/* Read ID after DLPS */
	memset(write_buf, 0, sizeof(write_buf));
	memset(read_buf, 0, sizeof(read_buf));

	write_buf[0] = 0x00;
	write_len = 1;
	read_len = 1;

	i2c_write_read(i2c_dev, icm20618_addr, write_buf, write_len, read_buf, read_len);

	printf("icm20618 addr:0x%x reg:0x%x = 0x%x\n", icm20618_addr, write_buf[0], read_buf[0]);

#endif /* CONFIG_I2C */

	return 0;
}

/* ADC PM test */

#ifdef CONFIG_ADC

#define ADC_BUFFER_SIZE   1
#define INVALID_ADC_VALUE SHRT_MIN

#define DT_SPEC_AND_COMMA(node_id, prop, idx) ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels, DT_SPEC_AND_COMMA)};

static const int adc_channels_count = ARRAY_SIZE(adc_channels);

static int16_t adc_sample_buf[ADC_BUFFER_SIZE];

static int do_single_adc_read(const struct adc_dt_spec *adc, int16_t *out_val)
{
	int ret;

	if (!device_is_ready(adc->dev)) {
		printf("ADC device not ready\n");
		return -ENODEV;
	}

	struct adc_sequence sequence = {
		.buffer = adc_sample_buf,
		.buffer_size = sizeof(adc_sample_buf),
		.resolution = 12,
	};

	ret = adc_sequence_init_dt(adc, &sequence);
	if (ret < 0) {
		printf("adc_sequence_init_dt failed: %d\n", ret);
		return ret;
	}

	for (int i = 0; i < ADC_BUFFER_SIZE; i++) {
		adc_sample_buf[i] = INVALID_ADC_VALUE;
	}

	ret = adc_read_dt(adc, &sequence);
	if (ret < 0) {
		printf("adc_read_dt failed: %d\n", ret);
		return ret;
	}

	*out_val = adc_sample_buf[0];
	return 0;
}

#endif /* CONFIG_ADC */

static int shell_pm_test_adc(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if defined(CONFIG_SOC_SERIES_RTL87X2J)
	printf("[%lld] wakeup_count=%d\n", k_uptime_get(), pck600_system_get_wakeup_count(NULL));
#endif

#ifdef CONFIG_ADC
	if (adc_channels_count < 1) {
		printf("No ADC channel configured in zephyr_user node\n");
		return 0;
	}

	int ret;
	int16_t val_before = INVALID_ADC_VALUE;
	int16_t val_after = INVALID_ADC_VALUE;

	for (uint8_t i = 0; i < adc_channels_count; i++) {
		ret = adc_channel_setup_dt(&adc_channels[i]);
		if (ret < 0) {
			printf("adc_channel_setup_dt[%d] failed: %d\n", i, ret);
			return 0;
		}
	}

	ret = do_single_adc_read(&adc_channels[0], &val_before);
	if (ret == 0) {
		printf("ADC sample before dlps: %d\n", val_before);
	} else {
		printf("ADC sample before dlps failed: %d\n", ret);
	}

#if defined(CONFIG_PM_DEVICE)
	pm_test_enter_dlps_forever();
#endif

	ret = do_single_adc_read(&adc_channels[0], &val_after);
	if (ret == 0) {
		printf("ADC sample after dlps: %d\n", val_after);
	} else {
		printf("ADC sample after dlps failed: %d\n", ret);
	}

#endif /* CONFIG_ADC */

	return 0;
}

/* SDHC / SDIO PM test */

static int shell_pm_test_sdhc(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#if defined(CONFIG_SDMMC_STACK) || defined(CONFIG_SDIO_STACK)
	int ret;

	if (sdhc_dev_sdmmc) {
		memset(sdmmc_rbuf, 0, sizeof(sdmmc_rbuf));

		for (uint32_t i = 0; i < sizeof(sdmmc_wbuf); i++) {
			sdmmc_wbuf[i] = (uint8_t)i;
		}

		printf("before dlps sdmmc card %s initializing...\n", sdhc_dev_sdmmc->name);

		ret = sd_init(sdhc_dev_sdmmc, &sdmmc_card);
		if (ret != 0) {
			printf("before dlps sdmmc card initialization failed\n");
			return 0;
		}

		printf("before dlps sdmmc card %s initialization success\n", sdhc_dev_sdmmc->name);

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

		printf("before dlps sdio card %s initialization success\n", sdhc_dev_sdio->name);
	}

#if defined(CONFIG_PM_DEVICE)
	pm_test_enter_dlps_forever();
#endif

	if (sdhc_dev_sdmmc) {
		memset(sdmmc_rbuf, 0, sizeof(sdmmc_rbuf));

		for (uint32_t i = 0; i < sizeof(sdmmc_wbuf); i++) {
			sdmmc_wbuf[i] = (uint8_t)(i * 3U);
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

		printf("after dlps sdio card %s initialization success\n", sdhc_dev_sdio->name);
	}

#endif /* SDMMC/SDIO */

	return 0;
}

/* CAN PM test */

#ifdef CONFIG_CAN

static struct k_sem can_tx_sem;
static struct k_sem can_rx_sem;
static uint8_t can_tx_data[8];
static uint8_t can_rx_data[8];
static int rx_count;

static void can_tx_cb(const struct device *dev, int error, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);
	ARG_UNUSED(error);
	k_sem_give(&can_tx_sem);
}

static void can_rx_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{
	ARG_UNUSED(user_data);

	printf("[%lld] can rx: id=0x%x, dlc=%d, data: ", k_uptime_get(), frame->id, frame->dlc);
	for (uint8_t i = 0; i < frame->dlc; i++) {
		can_rx_data[i] = frame->data[i];
		printf("0x%02x ", can_rx_data[i]);
	}
	printf("\n");
	rx_count++;
	k_sem_give(&can_rx_sem);
}

#endif /* CONFIG_CAN */

static int shell_pm_test_can(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(sh);
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

#ifdef CONFIG_CAN
	struct can_frame frame = {0};
	struct can_filter filter = {0};
	int filter_id;
	int ret;

#if defined(CONFIG_SOC_SERIES_RTL87X2J)
	printf("[%lld] wakeup_count=%d\n", k_uptime_get(), pck600_system_get_wakeup_count(NULL));
#endif

	if (!can_dev) {
		printf("CAN device not found\n");
		return 0;
	}

	/* Set loopback mode */
	ret = can_set_mode(can_dev, CAN_MODE_LOOPBACK);
	if (ret != 0) {
		printf("CAN set mode failed: %d\n", ret);
		return ret;
	}

	ret = can_start(can_dev);
	if (ret != 0) {
		printf("CAN start failed: %d\n", ret);
		return ret;
	}

	/* Prepare test frame */
	frame.id = 0x123;
	frame.dlc = 8;
	frame.flags = 0;
	for (uint8_t i = 0; i < frame.dlc; i++) {
		frame.data[i] = i + 1;
		can_tx_data[i] = frame.data[i];
	}

	/* Test 1: Add filter, TX, check RX */
	k_sem_init(&can_tx_sem, 0, 1);
	k_sem_init(&can_rx_sem, 0, 1);
	rx_count = 0;

	filter.id = 0x123;
	filter.mask = 0x7FF;
	filter.flags = 0;
	filter_id = can_add_rx_filter(can_dev, can_rx_cb, NULL, &filter);

	ret = can_send(can_dev, &frame, K_MSEC(100), can_tx_cb, NULL);
	if (ret != 0) {
		printf("CAN send failed: %d\n", ret);
		return ret;
	}
	if (k_sem_take(&can_tx_sem, K_MSEC(200)) != 0) {
		printf("tx timeout\n");
	}
	if (k_sem_take(&can_rx_sem, K_MSEC(200)) != 0) {
		printf("rx timeout\n");
	}

	/* Verify RX data matches TX */
	if (rx_count == 1 && memcmp(can_tx_data, can_rx_data, 8) == 0) {
		printf("tx/rx match\n");
	} else {
		printf("tx/rx mismatch\n");
		printf("  rx_count: %d\n", rx_count);
		printf("  tx_data:");
		for (int i = 0; i < 8; i++) {
			printf(" %02x", can_tx_data[i]);
		}
		printf("\n");
		printf("  rx_data:");
		for (int i = 0; i < 8; i++) {
			printf(" %02x", can_rx_data[i]);
		}
		printf("\n");
	}

	/* Test 2: Remove filter, TX, check no RX */
	can_remove_rx_filter(can_dev, filter_id);
	rx_count = 0;

	frame.data[0] = 0xAA;
	can_tx_data[0] = frame.data[0];
	ret = can_send(can_dev, &frame, K_MSEC(100), can_tx_cb, NULL);
	k_sem_take(&can_tx_sem, K_MSEC(200));
	k_busy_wait(50000);

	if (rx_count != 0) {
		printf("unexpected rx\n");
	}

	/* Test 3: Add filter, enter DLPS, TX, check RX */
	filter_id = can_add_rx_filter(can_dev, can_rx_cb, NULL, &filter);
	rx_count = 0;

#if defined(CONFIG_PM_DEVICE)
	pm_test_enter_dlps_forever();
#endif

	frame.data[0] = 0x55;
	can_tx_data[0] = frame.data[0];
	ret = can_send(can_dev, &frame, K_MSEC(100), can_tx_cb, NULL);
	if (k_sem_take(&can_tx_sem, K_MSEC(200)) != 0) {
		printf("tx timeout after DLPS\n");
	}
	if (k_sem_take(&can_rx_sem, K_MSEC(200)) != 0) {
		printf("rx timeout after DLPS\n");
	}

	if (rx_count == 1 && memcmp(can_tx_data, can_rx_data, 8) == 0) {
		printf("tx/rx match\n");
	} else {
		printf("tx/rx mismatch\n");
		printf("  rx_count: %d\n", rx_count);
		printf("  tx_data:");
		for (int i = 0; i < 8; i++) {
			printf(" %02x", can_tx_data[i]);
		}
		printf("\n");
		printf("  rx_data:");
		for (int i = 0; i < 8; i++) {
			printf(" %02x", can_rx_data[i]);
		}
		printf("\n");
	}

	/* Test 4: Remove filter, TX, check no RX */
	can_remove_rx_filter(can_dev, filter_id);
	rx_count = 0;

	frame.data[0] = 0xCC;
	can_tx_data[0] = frame.data[0];
	ret = can_send(can_dev, &frame, K_MSEC(100), can_tx_cb, NULL);
	k_sem_take(&can_tx_sem, K_MSEC(200));
	k_busy_wait(50000);

	if (rx_count != 0) {
		printf("unexpected rx\n");
	}

	can_stop(can_dev);

#endif /* CONFIG_CAN */

	return 0;
}

/* Keyscan input callback */

#ifdef CONFIG_INPUT

static void keyscan_input_cb(struct input_event *evt, void *user_data)
{
	ARG_UNUSED(user_data);

	static uint8_t col, row;

	switch (evt->code) {
	case INPUT_ABS_X:
		col = (uint8_t)evt->value;
		return;
	case INPUT_ABS_Y:
		row = (uint8_t)evt->value;
		return;
	case INPUT_BTN_TOUCH:
		break;
	default:
		return;
	}

	if (!evt->sync) {
		return;
	}


#if defined(CONFIG_SOC_SERIES_RTL87X2J)
	printf("[%lld] wakeup_count=%d\n", k_uptime_get(), pck600_system_get_wakeup_count(NULL));
#endif
	printf("[%lld] key [row=%d, col=%d] %s\n", k_uptime_get(), row, col,
	       evt->value ? "pressed" : "released");

#if defined(CONFIG_PM_DEVICE)
	if (evt->value == 0) {
		pm_dlps_check_flag = PM_TEST_CHECK_FAIL;
		k_sem_give(&pm_app_sem);
	}
#endif
}

INPUT_CALLBACK_DEFINE(DEVICE_DT_GET(DT_NODELABEL(keyscan)), keyscan_input_cb, NULL);

#endif /* CONFIG_INPUT */

/* Shell commands */

#define SHELL_CMD_ARG_CREATE                                                                       \
	SHELL_CMD_ARG(uart, NULL, "uart pm test", shell_pm_test_uart, 0, 0),                       \
		SHELL_CMD_ARG(uartdma, NULL, "uart dma pm test", shell_pm_test_uart_dma, 0, 0),    \
		SHELL_CMD_ARG(gpio, NULL, "gpio pm test [debounce_ms]", shell_pm_test_gpio, 0, 1), \
		SHELL_CMD_ARG(pwm, NULL, "pwm pm test", shell_pm_test_pwm, 0, 0),                  \
		SHELL_CMD_ARG(lppwm, NULL, "lppwm pm test", shell_pm_test_lppwm, 0, 0),     \
		SHELL_CMD_ARG(counter, NULL, "counter pm test (input time in ms)",                 \
			      shell_pm_test_counter, 2, 0),                                        \
		SHELL_CMD_ARG(spi, NULL, "spi pm test", shell_pm_test_spi, 0, 0),                  \
		SHELL_CMD_ARG(rtc, NULL, "rtc pm test", shell_pm_test_rtc, 0, 0),                  \
		SHELL_CMD_ARG(qdec, NULL, "qdec pm test", shell_pm_test_qdec, 0, 0),               \
		SHELL_CMD_ARG(waveform_gpio, NULL,                                                 \
			      "generate gpio output waveform [debounce_ms [pulse_us]]",            \
			      shell_pm_test_generate_waveform_gpio, 0, 2),                         \
		SHELL_CMD_ARG(waveform_qdec, NULL, "generate qdec encoder emulation waveform",     \
			      shell_pm_test_generate_waveform_qdec, 0, 0),                         \
		SHELL_CMD_ARG(i2c, NULL, "i2c pm test", shell_pm_test_i2c, 0, 0),                  \
		SHELL_CMD_ARG(adc, NULL, "adc pm test", shell_pm_test_adc, 0, 0),                  \
		SHELL_CMD_ARG(sdhc, NULL, "sdhc/sdio pm test", shell_pm_test_sdhc, 0, 0),          \
		SHELL_CMD_ARG(can, NULL, "can pm test", shell_pm_test_can, 0, 0),                  \
		SHELL_SUBCMD_SET_END

SHELL_STATIC_SUBCMD_SET_CREATE(sub_pm_test, SHELL_CMD_ARG_CREATE);

SHELL_CMD_REGISTER(pm_test, &sub_pm_test, "PM tests", NULL);
