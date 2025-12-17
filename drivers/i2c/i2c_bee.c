/*
 * Copyright(c) 2025, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_bee_i2c

#include <errno.h>

#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/bee_clock_control.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/reset.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(i2c_bee, CONFIG_I2C_LOG_LEVEL);

#include "i2c-priv.h"

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
#include <rtl_i2c.h>
#include <rtl_rcc.h>
#elif defined(CONFIG_SOC_SERIES_RTL8752H)
#include <rtl876x_i2c.h>
#include <rtl876x_rcc.h>
#endif

#ifdef CONFIG_PM_DEVICE
	extern void I2C_DLPSEnter(void *PeriReg, void *StoreBuf);
	extern void I2C_DLPSExit(void *PeriReg, void *StoreBuf);
#endif

#define I2C_TIMEOUT 0xFFFFF

struct i2c_bee_config {
	uint32_t reg;
	uint32_t bitrate;
	uint16_t clkid;
	const struct pinctrl_dev_config *pcfg;
#if defined(CONFIG_I2C_BEE_INTERRUPT)
	void (*irq_cfg_func)(void);
#endif
};

struct i2c_bee_data {
	struct k_sem bus_mutex;
#if defined(CONFIG_I2C_BEE_INTERRUPT)
	struct k_sem sync_sem;
#endif
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

static void i2c_bee_log_err(struct i2c_bee_data *data)
{
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

	if (data->errs == I2C_ERR_TIMEOUT) {
		LOG_ERR("timeout");
	}
}

#if defined(CONFIG_I2C_BEE_INTERRUPT)
static void i2c_bee_isr(const struct device *dev)
{
	struct i2c_bee_data *data = dev->data;
	const struct i2c_bee_config *cfg = dev->config;
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
#endif

static int i2c_bee_msg_handler(const struct device *dev)
{
	struct i2c_bee_data *data = dev->data;
	const struct i2c_bee_config *cfg = dev->config;
	I2C_TypeDef *i2c = (I2C_TypeDef *)cfg->reg;
	bool read_f = (data->current->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ;
	bool stop_f = (data->current->flags & I2C_MSG_STOP);

	data->errs = 0;

#if !defined(CONFIG_I2C_BEE_INTERRUPT)
	uint32_t timeout;
#else
	k_sem_reset(&data->sync_sem);
#endif

	if (read_f) {
		for (uint32_t cnt = 0; cnt < data->xfer_len; ++cnt) {
			if (cnt >= data->xfer_len - 1) {
				i2c->IC_DATA_CMD = BIT8 | (stop_f ? BIT9 : 0);
			} else {
				i2c->IC_DATA_CMD = BIT8;
			}

			I2C_INTConfig(i2c, I2C_INT_RX_FULL | I2C_INT_TX_ABRT, ENABLE);

			/* wait for interrupt */
#if defined(CONFIG_I2C_BEE_INTERRUPT)
			k_sem_take(&data->sync_sem, K_FOREVER);
#else
			timeout = I2C_TIMEOUT;
			while (!(I2C_GetINTStatus(i2c, I2C_INT_TX_ABRT) ||
				 I2C_GetINTStatus(i2c, I2C_INT_RX_FULL))) {
				timeout--;
				if (timeout == 0) {
					return -EIO;
				}
			}
			if (I2C_GetINTStatus(i2c, I2C_INT_TX_ABRT)) {
				data->errs = I2C_CheckAbortStatus(i2c);
				I2C_INTConfig(i2c,
					      I2C_INT_TX_ABRT | I2C_INT_RX_FULL | I2C_INT_TX_EMPTY,
					      DISABLE);
				I2C_ClearINTPendingBit(i2c, I2C_INT_TX_ABRT);
				I2C_ClearINTPendingBit(i2c, I2C_INT_RX_FULL);
				I2C_ClearINTPendingBit(i2c, I2C_INT_TX_EMPTY);
			} else if (I2C_GetINTStatus(i2c, I2C_INT_RX_FULL)) {
				I2C_INTConfig(i2c, I2C_INT_RX_FULL, DISABLE);
				I2C_ClearINTPendingBit(i2c, I2C_INT_RX_FULL);
			}
#endif

			if (data->errs != I2C_Success) {
				return -EIO;
			}

			*data->current->buf++ = i2c->IC_DATA_CMD;
		}

	} else {
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
#if defined(CONFIG_I2C_BEE_INTERRUPT)
			k_sem_take(&data->sync_sem, K_FOREVER);
#else
			timeout = I2C_TIMEOUT;
			while (!(I2C_GetINTStatus(i2c, I2C_INT_TX_ABRT) ||
				 I2C_GetINTStatus(i2c, I2C_INT_TX_EMPTY))) {
				timeout--;
				if (timeout == 0) {
					return -EIO;
				}
			}

			if (I2C_GetINTStatus(i2c, I2C_INT_TX_ABRT)) {
				data->errs = I2C_CheckAbortStatus(i2c);
				I2C_INTConfig(i2c,
					      I2C_INT_TX_ABRT | I2C_INT_RX_FULL | I2C_INT_TX_EMPTY,
					      DISABLE);
				I2C_ClearINTPendingBit(i2c, I2C_INT_TX_ABRT);
				I2C_ClearINTPendingBit(i2c, I2C_INT_RX_FULL);
				I2C_ClearINTPendingBit(i2c, I2C_INT_TX_EMPTY);
			} else if (I2C_GetINTStatus(i2c, I2C_INT_TX_EMPTY)) {
				I2C_INTConfig(i2c, I2C_INT_TX_EMPTY, DISABLE);
				I2C_ClearINTPendingBit(i2c, I2C_INT_TX_EMPTY);
			}
#endif

			if (data->errs != I2C_Success) {
				return -EIO;
			}
		}

		I2C_INTConfig(i2c, I2C_INT_TX_EMPTY | I2C_INT_TX_ABRT, ENABLE);

		/* wait for interrupt */
#if defined(CONFIG_I2C_BEE_INTERRUPT)
		k_sem_take(&data->sync_sem, K_FOREVER);
#else
		timeout = I2C_TIMEOUT;
		while (!(I2C_GetINTStatus(i2c, I2C_INT_TX_ABRT) ||
			 I2C_GetINTStatus(i2c, I2C_INT_TX_EMPTY))) {
			timeout--;
			if (timeout == 0) {
				return -EIO;
			}
		}
		if (I2C_GetINTStatus(i2c, I2C_INT_TX_ABRT)) {
			data->errs = I2C_CheckAbortStatus(i2c);
			I2C_INTConfig(i2c, I2C_INT_TX_ABRT | I2C_INT_RX_FULL | I2C_INT_TX_EMPTY,
				      DISABLE);
			I2C_ClearINTPendingBit(i2c, I2C_INT_TX_ABRT);
			I2C_ClearINTPendingBit(i2c, I2C_INT_RX_FULL);
			I2C_ClearINTPendingBit(i2c, I2C_INT_TX_EMPTY);
		} else if (I2C_GetINTStatus(i2c, I2C_INT_TX_EMPTY)) {
			I2C_INTConfig(i2c, I2C_INT_TX_EMPTY, DISABLE);
			I2C_ClearINTPendingBit(i2c, I2C_INT_TX_EMPTY);
		}
#endif
		if (data->errs != I2C_Success) {
			return -EIO;
		}
	}

	LOG_DBG("i2c_bee_msg_handler exit line%d\n", __LINE__);

	return 0;
}

static int i2c_bee_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
			    uint16_t addr)
{
	struct i2c_bee_data *data = dev->data;
	const struct i2c_bee_config *cfg = dev->config;
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

		err = i2c_bee_msg_handler(dev);

		if (err < 0) {
			i2c_bee_log_err(data);
			break;
		}
	}

	/* Disable I2C device */
	I2C_Cmd(i2c, DISABLE);

	k_sem_give(&data->bus_mutex);
	return err;
}

static int i2c_bee_configure(const struct device *dev, uint32_t dev_config)
{
	struct i2c_bee_data *data = dev->data;
	const struct i2c_bee_config *cfg = dev->config;
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

	/* Enable i2c device */
	I2C_Cmd(i2c, ENABLE);
error:
	k_sem_give(&data->bus_mutex);

	return err;
}

#ifdef CONFIG_PM_DEVICE
static int i2c_bee_pm_action(const struct device *dev, enum pm_device_action action)
{
	struct i2c_bee_data *data = dev->data;
	const struct i2c_bee_config *cfg = dev->config;
	I2C_TypeDef *i2c = (I2C_TypeDef *)cfg->reg;
	int err;

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

		(void)clock_control_on(BEE_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid);

		I2C_DLPSExit(i2c, &data->store_buf);

		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static struct i2c_driver_api i2c_bee_driver_api = {
	.configure = i2c_bee_configure,
	.transfer = i2c_bee_transfer,
};

static int i2c_bee_init(const struct device *dev)
{
	struct i2c_bee_data *data = dev->data;
	const struct i2c_bee_config *cfg = dev->config;
	uint32_t bitrate_cfg;
	int err = 0;

	/* Configure pinmux  */
	err = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	(void)clock_control_on(BEE_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid);

	/* Mutex semaphore to protect the i2c api in multi-thread env. */
	k_sem_init(&data->bus_mutex, 1, 1);

#if defined(CONFIG_I2C_BEE_INTERRUPT)
	/* Sync semaphore to sync i2c state between isr and transfer api. */
	k_sem_init(&data->sync_sem, 0, K_SEM_MAX_LIMIT);
#endif

#if defined(CONFIG_I2C_BEE_INTERRUPT)
	cfg->irq_cfg_func();
#endif

	bitrate_cfg = i2c_map_dt_bitrate(cfg->bitrate);
	i2c_bee_configure(dev, I2C_MODE_CONTROLLER | bitrate_cfg);

	return 0;
}

#if defined(CONFIG_I2C_BEE_INTERRUPT)
#define I2C_IRQ_FUNC_DEFINE(index)                                                                 \
	static void i2c_bee_irq_cfg_func_##index(void)                                             \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(index), DT_INST_IRQ(index, priority), i2c_bee_isr,        \
			    DEVICE_DT_INST_GET(index), 0);                                         \
		irq_enable(DT_INST_IRQN(index));                                                   \
	}
#define I2C_IRQ_CONFIG(index) .irq_cfg_func = i2c_bee_irq_cfg_func_##index,
#else
#define I2C_IRQ_FUNC_DEFINE(index)
#define I2C_IRQ_CONFIG(index)
#endif

#define I2C_BEE_INIT(index)                                                                        \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
	I2C_IRQ_FUNC_DEFINE(index);                                                                \
	static struct i2c_bee_data i2c_bee_data_##index;                                           \
	const static struct i2c_bee_config i2c_bee_cfg_##index = {                                 \
		.reg = DT_INST_REG_ADDR(index),                                                    \
		.bitrate = DT_INST_PROP(index, clock_frequency),                                   \
		.clkid = DT_INST_CLOCKS_CELL(index, id),                                           \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                     \
		I2C_IRQ_CONFIG(index)};                                                            \
	PM_DEVICE_DT_INST_DEFINE(index, i2c_bee_pm_action);                                        \
	I2C_DEVICE_DT_INST_DEFINE(index, i2c_bee_init, PM_DEVICE_DT_INST_GET(index),               \
				  &i2c_bee_data_##index, &i2c_bee_cfg_##index, POST_KERNEL,        \
				  CONFIG_I2C_INIT_PRIORITY, &i2c_bee_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_BEE_INIT)
