/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_rtl8752h_cctl

#include <stdint.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>

#include <rtl876x_rcc.h>

#include <trace.h>
#define DBG_DIRECT_SHOW 0

struct clock_control_rtl8752h_config {
	uint32_t reg;
};

typedef struct {
	uint32_t apbperiph;
	uint32_t apbperiph_clk;
} apb_cfg;

static const apb_cfg rtl8752h_apb_table[] = {
	{APBPeriph_I2S0, APBPeriph_I2S0_CLOCK},
	{APBPeriph_I2S1, APBPeriph_I2S1_CLOCK},
	{APBPeriph_CODEC, APBPeriph_CODEC_CLOCK},
	{APBPeriph_GPIO, APBPeriph_GPIO_CLOCK},
	{APBPeriph_GDMA, APBPeriph_GDMA_CLOCK},
	{APBPeriph_TIMER, APBPeriph_TIMER_CLOCK},
	{APBPeriph_ENHTIMER, APBPeriph_ENHTIMER_CLOCK},
	{APBPeriph_UART2, APBPeriph_UART2_CLOCK},
	{APBPeriph_UART0, APBPeriph_UART0_CLOCK},
	{APBPeriph_FLASH, APBPeriph_FLASH_CLOCK},
	{APBPeriph_PKE, APBPeriph_PKE_CLOCK},
	{APBPeriph_SHA256, APBPeriph_SHA256_CLOCK},
	{APBPeriph_FLASH1, APBPeriph_FLASH1_CLOCK},
	{APBPeriph_FLH_SEC, APBPeriph_FLH_SEC_CLOCK},
	{APBPeriph_IR, APBPeriph_IR_CLOCK},
	{APBPeriph_SPI1, APBPeriph_SPI1_CLOCK},
	{APBPeriph_SPI0, APBPeriph_SPI0_CLOCK},
	{APBPeriph_UART1, APBPeriph_UART1_CLOCK},
	{APBPeriph_IF8080, APBPeriph_IF8080_CLOCK},
	{APBPeriph_ADC, APBPeriph_ADC_CLOCK},
	{APBPeriph_SPI2W, APBPeriph_SPI2W_CLOCK},
	{APBPeriph_MODEMRFCPI_CLOCK, APBPeriph_MODEMRFCPI_CLOCK},
	{APBPeriph_BLUEWIZ, APBPeriph_BLUEWIZ_CLOCK},
	{APBPeriph_ZIGBEE, APBPeriph_ZIGBEE_CLOCK},
	{APBPeriph_KEYSCAN, APBPeriph_KEYSCAN_CLOCK},
	{APBPeriph_QDEC, APBPeriph_QDEC_CLOCK},
	{APBPeriph_I2C1, APBPeriph_I2C1_CLOCK},
	{APBPeriph_I2C0, APBPeriph_I2C0_CLOCK},
};

static int clock_control_rtl8752h_on(const struct device *dev, clock_control_subsys_t sys)
{
	uint16_t id = *(uint16_t *)sys;

	RCC_PeriphClockCmd(rtl8752h_apb_table[id].apbperiph, rtl8752h_apb_table[id].apbperiph_clk,
			   ENABLE);
#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s] sys=%d, apbperiph=0x%x, apbperiph_clk=0x%x", __func__, id,
		   rtl8752h_apb_table[id].apbperiph, rtl8752h_apb_table[id].apbperiph_clk);
#endif
	return 0;
}

static int clock_control_rtl8752h_off(const struct device *dev, clock_control_subsys_t sys)
{
	uint16_t id = *(uint16_t *)sys;

	RCC_PeriphClockCmd(rtl8752h_apb_table[id].apbperiph, rtl8752h_apb_table[id].apbperiph_clk,
			   DISABLE);

#if DBG_DIRECT_SHOW
	DBG_DIRECT("[%s] sys=%d, apbperiph=%d, apbperiph_clk=%d", __func__, sys,
		   rtl8752h_apb_table[id].apbperiph, rtl8752h_apb_table[id].apbperiph_clk);
#endif
	return 0;
}

static struct clock_control_driver_api clock_control_rtl8752h_api = {
	.on = clock_control_rtl8752h_on,
	.off = clock_control_rtl8752h_off,
};

static const struct clock_control_rtl8752h_config config = {
	.reg = DT_REG_ADDR(DT_INST_PARENT(0)),
};

DEVICE_DT_INST_DEFINE(0, NULL, NULL, NULL, &config, PRE_KERNEL_1,
		      CONFIG_CLOCK_CONTROL_INIT_PRIORITY, &clock_control_rtl8752h_api);
