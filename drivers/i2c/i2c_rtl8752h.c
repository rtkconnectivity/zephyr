/*
 * Copyright(c) 2020, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_rtl8752h_i2c

#include <errno.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/rtl8752h_clock_control.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(i2c_rtl8752h, CONFIG_I2C_LOG_LEVEL);

#include "i2c-priv.h"

#include <rtl876x_i2c.h>
#include <rtl876x_rcc.h>
#include "trace.h"
struct i2c_rtl8752h_config {
	uint32_t reg;
	uint32_t bitrate;
	uint16_t clkid;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_cfg_func)(void);
};

struct i2c_rtl8752h_data {
	struct k_sem bus_mutex;
	struct k_sem sync_sem;
	uint32_t dev_config;
	uint16_t slave_address;
	uint32_t xfer_len;
	struct i2c_msg *current;
	uint8_t errs;
	bool is_restart;
#ifdef CONFIG_PM_DEVICE
	I2CStoreReg_Typedef store_buf;
#endif
};

static void i2c_rtl8752h_log_err(struct i2c_rtl8752h_data *data)
{
	LOG_DBG("[%s] line%d\n", __func__, __LINE__);
	if (data->errs == I2C_ABRT_7B_ADDR_NOACK) {
		LOG_ERR("7 bit address no ack error");
	}

	if (data->errs == I2C_ABRT_10ADDR1_NOACK || data->errs == I2C_ABRT_10ADDR2_NOACK) {
		LOG_ERR("10 bit address no ack error");
	}

	if (data->errs == I2C_ABRT_TXDATA_NOACK) {
		LOG_ERR("data no ack error");
	}

	if (data->errs == I2C_ARB_LOST) {
		LOG_ERR("arbitration lost error");
	}
}

static void i2c_rtl8752h_isr(const struct device *dev)
{
	LOG_DBG("[%s] line%d\n", __func__, __LINE__);
	struct i2c_rtl8752h_data *data = dev->data;
	const struct i2c_rtl8752h_config *cfg = dev->config;
	I2C_TypeDef *i2c = (I2C_TypeDef *)cfg->reg;

	if (I2C_GetINTStatus(i2c, I2C_INT_TX_ABRT)) {
		data->errs = I2C_CheckAbortStatus(i2c);
		I2C_INTConfig(i2c, I2C_INT_TX_ABRT | I2C_INT_RX_FULL | I2C_INT_TX_EMPTY, DISABLE);
		I2C_ClearINTPendingBit(i2c, I2C_INT_TX_ABRT);
		I2C_ClearINTPendingBit(i2c, I2C_INT_RX_FULL);
		I2C_ClearINTPendingBit(i2c, I2C_INT_TX_EMPTY);
		k_sem_give(&data->sync_sem);
	} else if (I2C_GetINTStatus(i2c, I2C_INT_RX_FULL)) {
		I2C_INTConfig(i2c, I2C_INT_RX_FULL, DISABLE);
		I2C_ClearINTPendingBit(i2c, I2C_INT_RX_FULL);
		k_sem_give(&data->sync_sem);
	} else if (I2C_GetINTStatus(i2c, I2C_INT_TX_EMPTY)) {
		I2C_INTConfig(i2c, I2C_INT_TX_EMPTY, DISABLE);
		I2C_ClearINTPendingBit(i2c, I2C_INT_TX_EMPTY);
		k_sem_give(&data->sync_sem);
	}
}

static int i2c_rtl8752h_msg_handler(const struct device *dev)
{
	struct i2c_rtl8752h_data *data = dev->data;
	const struct i2c_rtl8752h_config *cfg = dev->config;
	I2C_TypeDef *i2c = (I2C_TypeDef *)cfg->reg;
	bool read_f = (data->current->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ;
	bool stop_f = (data->current->flags & I2C_MSG_STOP);

	data->errs = 0;

	k_sem_reset(&data->sync_sem);

	if (read_f) {
		LOG_DBG("[%s] read line%d\n", __func__, __LINE__);
		for (uint32_t cnt = 0; cnt < data->xfer_len; ++cnt) {
			if (cnt >= data->xfer_len - 1) {
				i2c->IC_DATA_CMD = BIT8 | (stop_f ? BIT9 : 0);
			} else {
				i2c->IC_DATA_CMD = BIT8;
			}

			I2C_INTConfig(i2c, I2C_INT_RX_FULL | I2C_INT_TX_ABRT, ENABLE);

			/* wait for interrupt */
			k_sem_take(&data->sync_sem, K_FOREVER);

			if (data->errs != I2C_Success) {
				return -EIO;
			}

			*data->current->buf++ = i2c->IC_DATA_CMD;
		}
	} else {
		LOG_DBG("[%s] write line%d\n", __func__, ___LINE__);
		for (uint32_t cnt = 0; cnt < data->xfer_len; ++cnt) {
			if (cnt >= data->xfer_len - 1) {
				i2c->IC_DATA_CMD = *data->current->buf++ | (stop_f ? BIT9 : 0);
			} else {
				i2c->IC_DATA_CMD = *data->current->buf++;
			}

			data->errs = I2C_CheckAbortStatus(i2c);
			if (data->errs != I2C_Success) {
				return -EIO;
			}

			if (i2c->IC_STATUS & I2C_FLAG_TFNF) {
				continue;
			}

			I2C_INTConfig(i2c, I2C_INT_TX_EMPTY | I2C_INT_TX_ABRT, ENABLE);
			/* wait for interrupt */
			k_sem_take(&data->sync_sem, K_FOREVER);

			if (data->errs != I2C_Success) {
				return -EIO;
			}
		}

		I2C_INTConfig(i2c, I2C_INT_TX_EMPTY | I2C_INT_TX_ABRT, ENABLE);
		/* wait for interrupt */
		k_sem_take(&data->sync_sem, K_FOREVER);
		if (data->errs != I2C_Success) {
			return -EIO;
		}
	}

	LOG_DBG("[%s] write exit line%d\n", __func__, ___LINE__);

	return 0;
}

static int i2c_rtl8752h_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				 uint16_t addr)
{
	LOG_DBG("[%s], num_msgs=%d line%d\n", __func__, num_msgs, __LINE__);
	struct i2c_rtl8752h_data *data = dev->data;
	const struct i2c_rtl8752h_config *cfg = dev->config;
	I2C_TypeDef *i2c = (I2C_TypeDef *)cfg->reg;
	struct i2c_msg *current, *next;
	int err = 0;

	current = msgs;

	/* First message flags implicitly contain I2C_MSG_RESTART flag. */
	current->flags |= I2C_MSG_RESTART;

	for (uint8_t i = 1; i <= num_msgs; i++) {

		if (i < num_msgs) {
			next = current + 1;

			/*
			 * If there have a R/W transfer state change between messages,
			 * An explicit I2C_MSG_RESTART flag is needed for the second message.
			 */
			if ((current->flags & I2C_MSG_RW_MASK) != (next->flags & I2C_MSG_RW_MASK)) {
				if ((next->flags & I2C_MSG_RESTART) == 0U) {
					return -EINVAL;
				}
			}

			/* Only the last message need I2C_MSG_STOP flag to free the Bus. */
			if (current->flags & I2C_MSG_STOP) {
				return -EINVAL;
			}
		} else {
			/* Last message flags implicitly contain I2C_MSG_STOP flag. */
			current->flags |= I2C_MSG_STOP;
		}

		if ((current->buf == NULL) || (current->len == 0U)) {
			return -EINVAL;
		}

		current++;
	}

	k_sem_take(&data->bus_mutex, K_FOREVER);

	/* Enable i2c device */
	I2C_Cmd(i2c, ENABLE);

	I2C_SetSlaveAddress(i2c, addr);
	data->slave_address = addr;

	for (uint8_t i = 0; i < num_msgs; ++i) {
		data->current = &msgs[i];
		data->xfer_len = msgs[i].len;

		err = i2c_rtl8752h_msg_handler(dev);

		if (err < 0) {
			i2c_rtl8752h_log_err(data);
			break;
		}
	}

	/* Disable I2C device */
	I2C_Cmd(i2c, DISABLE);

	k_sem_give(&data->bus_mutex);

	return err;
}

static int i2c_rtl8752h_configure(const struct device *dev, uint32_t dev_config)
{
	LOG_DBG("[%s] line%d\n", __func__, __LINE__);
	struct i2c_rtl8752h_data *data = dev->data;
	const struct i2c_rtl8752h_config *cfg = dev->config;
	uint32_t pclk;
	I2C_TypeDef *i2c = (I2C_TypeDef *)cfg->reg;
	int err = 0;

	k_sem_take(&data->bus_mutex, K_FOREVER);

	/* Disable I2C device */
	I2C_Cmd(i2c, DISABLE);

	pclk = 40000000;

	I2C_InitTypeDef i2c_init_struct;

	I2C_StructInit(&i2c_init_struct);
	i2c_init_struct.I2C_Clock = pclk;
	if (dev_config & I2C_MODE_CONTROLLER) {
		i2c_init_struct.I2C_DeviveMode = I2C_DeviveMode_Master;
	} else {
		i2c_init_struct.I2C_DeviveMode = I2C_DeviveMode_Slave;
	}

	if (dev_config & I2C_ADDR_10_BITS) {
		i2c_init_struct.I2C_AddressMode = I2C_AddressMode_10BIT;
	} else {
		i2c_init_struct.I2C_AddressMode = I2C_AddressMode_7BIT;
	}

	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		i2c_init_struct.I2C_ClockSpeed = I2C_BITRATE_STANDARD;
		break;
	case I2C_SPEED_FAST:
		i2c_init_struct.I2C_ClockSpeed = I2C_BITRATE_FAST;
		break;
	case I2C_SPEED_FAST_PLUS:
		i2c_init_struct.I2C_ClockSpeed = I2C_BITRATE_FAST_PLUS;
		break;
	default:
		err = -EINVAL;
		goto error;
	}

	data->dev_config = dev_config;

	I2C_Init(i2c, &i2c_init_struct);

	I2C_Cmd(i2c, ENABLE);
error:
	k_sem_give(&data->bus_mutex);

	return err;
}

#ifdef CONFIG_PM_DEVICE
static int i2c_rtl8752h_pm_action(const struct device *dev, enum pm_device_action action)
{
	struct i2c_rtl8752h_data *data = dev->data;
	const struct i2c_rtl8752h_config *cfg = dev->config;
	I2C_TypeDef *i2c = (I2C_TypeDef *)cfg->reg;
	int err;

	extern void I2C_DLPSEnter(void *PeriReg, void *StoreBuf);
	extern void I2C_DLPSExit(void *PeriReg, void *StoreBuf);

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:

		I2C_DLPSEnter(i2c, &data->store_buf);

		/* Move pins to sleep state */
		err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_SLEEP);
		if ((err < 0) && (err != -ENOENT)) {
			return err;
		}

		break;
	case PM_DEVICE_ACTION_RESUME:
		/* Set pins to active state */
		err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (err < 0) {
			return err;
		}

		I2C_DLPSExit(i2c, &data->store_buf);

		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static struct i2c_driver_api i2c_rtl8752h_driver_api = {
	.configure = i2c_rtl8752h_configure,
	.transfer = i2c_rtl8752h_transfer,
};

static int i2c_rtl8752h_init(const struct device *dev)
{
	LOG_DBG("[%s] line%d\n", __func__, __LINE__);
	struct i2c_rtl8752h_data *data = dev->data;
	const struct i2c_rtl8752h_config *cfg = dev->config;
	uint32_t bitrate_cfg;
	int err = 0;

	(void)clock_control_on(RTL8752H_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid);

	/* Configure pinmux  */

	err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	/* Mutex semaphore to protect the i2c api in multi-thread env. */
	k_sem_init(&data->bus_mutex, 1, 1);

	/* Sync semaphore to sync i2c state between isr and transfer api. */
	k_sem_init(&data->sync_sem, 0, K_SEM_MAX_LIMIT);

	cfg->irq_cfg_func();

	bitrate_cfg = i2c_map_dt_bitrate(cfg->bitrate);
	i2c_rtl8752h_configure(dev, I2C_MODE_CONTROLLER | bitrate_cfg);

	return 0;
}

#define I2C_RTL8752H_INIT(index)                                                                   \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
	static void i2c_rtl8752h_irq_cfg_func_##index(void)                                        \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(index), DT_INST_IRQ(index, priority), i2c_rtl8752h_isr,   \
			    DEVICE_DT_INST_GET(index), 0);                                         \
		irq_enable(DT_INST_IRQN(index));                                                   \
	}                                                                                          \
	static struct i2c_rtl8752h_data i2c_rtl8752h_data_##index;                                 \
	const static struct i2c_rtl8752h_config i2c_rtl8752h_cfg_##index = {                       \
		.reg = DT_INST_REG_ADDR(index),                                                    \
		.bitrate = DT_INST_PROP(index, clock_frequency),                                   \
		.clkid = DT_INST_CLOCKS_CELL(index, id),                                           \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                     \
		.irq_cfg_func = i2c_rtl8752h_irq_cfg_func_##index,                                 \
	};                                                                                         \
	PM_DEVICE_DT_INST_DEFINE(index, i2c_rtl8752h_pm_action);                                   \
	I2C_DEVICE_DT_INST_DEFINE(index, i2c_rtl8752h_init, PM_DEVICE_DT_INST_GET(index),          \
				  &i2c_rtl8752h_data_##index, &i2c_rtl8752h_cfg_##index,           \
				  POST_KERNEL, CONFIG_I2C_INIT_PRIORITY,                           \
				  &i2c_rtl8752h_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_RTL8752H_INIT)
