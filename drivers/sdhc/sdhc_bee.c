/*
 * Copyright(c) 2025, Realtek Semiconductor Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT realtek_bee_sdhc

#include <zephyr/kernel.h>
#include <zephyr/drivers/sdhc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <soc.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/bee_clock_control.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <math.h>

#include "os_sync.h"
#include "rtl_sdhc.h"
#include "clock.h"
#include "trace.h"

LOG_MODULE_REGISTER(sdhc, CONFIG_SDHC_LOG_LEVEL);

#define PINCTRL_STATE_INTERRUPT (PINCTRL_STATE_PRIV_START + 1)

struct gpio_callback sdio_int_gpio_cb;

#define DEVICE_DT_GET_AND_COMMA(node_id) DEVICE_DT_GET(node_id),
static const struct device *const devices[] = {
	DT_FOREACH_STATUS_OKAY(DT_DRV_COMPAT, DEVICE_DT_GET_AND_COMMA)};

#ifdef CONFIG_PM_DEVICE
extern void SDHC_DLPSEnter(void *PeriReg, void *StoreBuf);
extern void SDHC_DLPSExit(void *PeriReg, void *StoreBuf);
#endif

struct sdhc_bee_config {
	const SDHC_TypeDef *sdhc_base;
	const uint16_t clkid;
	const struct pinctrl_dev_config *pcfg;
	const struct gpio_dt_spec pwr_gpio;
	const struct gpio_dt_spec int_gpio;
	void (*sd_irq_connect)(void);
	void (*sd_irq_enable)(void);
	void (*sd_irq_disable)(void);
	uint8_t pin_group;
	struct sdhc_host_props props;
};

struct sdhc_bee_data {
	uint8_t bus_width;
	uint32_t src_clock;
	uint32_t bus_clock;
	enum sdhc_power power_mode;
	enum sdhc_timing_mode timing;
	struct k_mutex s_request_mutex;
	sdhc_interrupt_cb_t cb;
	void *user_data;
	bool sdio_int_en;
#ifdef CONFIG_PM_DEVICE
	SDHCStoreReg_Typedef store_buf;
#endif
};

static void sdio_int_gpio_cb_func(const struct device *dev_in, struct gpio_callback *gpio_cb,
				  uint32_t pins)
{
	const struct sdhc_bee_config *config;
	struct sdhc_bee_data *data;

	for (int i = 0; i < ARRAY_SIZE(devices); i++) {
		data = (struct sdhc_bee_data *)(devices[i]->data);
		config = (struct sdhc_bee_config *)(devices[i]->config);
		if (dev_in == config->int_gpio.port && pins & BIT(config->int_gpio.pin)) {
			if (data->cb) {
				data->cb(devices[i], SDHC_INT_SDIO, data->user_data);
			}
		}
	}
}

static int sdhc_bee_enable_interrupt_pin(const struct device *dev)
{
	LOG_INF("[%s]", __func__);
	const struct sdhc_bee_config *config = dev->config;
	int ret;

	ret = gpio_pin_configure(config->int_gpio.port, config->int_gpio.pin,
				 (config->int_gpio.dt_flags | GPIO_INPUT | GPIO_PULL_UP));
	if (ret < 0) {
		return ret;
	}

	return gpio_pin_interrupt_configure(config->int_gpio.port, config->int_gpio.pin,
					    GPIO_INT_LEVEL_LOW);
}

static int sdhc_bee_disable_interrupt_pin(const struct device *dev)
{
	LOG_INF("[%s]", __func__);
	const struct sdhc_bee_config *config = dev->config;
	int ret;

	ret = gpio_pin_configure(config->int_gpio.port, config->int_gpio.pin, GPIO_DISCONNECTED);
	if (ret < 0) {
		return ret;
	}

	ret = gpio_pin_interrupt_configure(config->int_gpio.port, config->int_gpio.pin,
					   GPIO_INT_DISABLE);
	if (ret < 0) {
		return ret;
	}

	return pinctrl_apply_state(config->pcfg, PINCTRL_STATE_INTERRUPT);
}

static int sdhc_bee_do_transaction(const struct device *dev, struct sdhc_command *cmd,
				   struct sdhc_data *data)
{
	const struct sdhc_bee_config *cfg = dev->config;
	struct sdhc_bee_data *dev_data = dev->data;
	SDHC_TypeDef *sdhc_base = (SDHC_TypeDef *)cfg->sdhc_base;
	int ret = 0;
	SDHCRes_t sd_ret = SDHCRES_OK;
	DataInfo_t sdh_data;
	CmdInfo_t sdh_cmd;
	uint32_t rsp;
	bool high_capacity = true;
	uint8_t *pubuf;
	uint32_t blockaddr;
	uint32_t remainblock;

	if (dev_data->sdio_int_en) {
		sdhc_bee_disable_interrupt_pin(dev);
	}

	sdh_cmd.CmdIdx = cmd->opcode;
	sdh_cmd.CmdArg = cmd->arg;

	switch (cmd->opcode) {
	case SD_GO_IDLE_STATE:
		sdh_cmd.IsResetCmd = true;
		sdh_cmd.IsStopCmd = false;
		sdh_cmd.IsRspExpected = false;
		sdh_cmd.IsR2Rsp = false;
		sdh_cmd.CheckRspCrc = false;

		sd_ret = SDHC_SendNoDataCmd(sdhc_base, &sdh_cmd, NULL);
		if (sd_ret != SDHCRES_OK) {
			LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
			ret = -EIO;
		}

		break;
	case SD_SEND_IF_COND:
		if (cmd->arg != 0) {
			sdh_cmd.IsResetCmd = false;
			sdh_cmd.IsStopCmd = false;
			sdh_cmd.IsRspExpected = true;
			sdh_cmd.IsR2Rsp = false;
			sdh_cmd.CheckRspCrc = true;

			sd_ret = SDHC_SendNoDataCmd(sdhc_base, &sdh_cmd, &cmd->response[0]);
			if (sd_ret != SDHCRES_OK) {
				LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
				ret = -EIO;
			}
		} else {
			sdh_data.BlockSize = data->block_size;
			sdh_data.BlockCount = data->blocks;
			sdh_data.SendAutoStop = false;

			sdh_cmd.IsResetCmd = false;
			sdh_cmd.IsStopCmd = false;
			sdh_cmd.IsRspExpected = true;
			sdh_cmd.IsR2Rsp = false;
			sdh_cmd.CheckRspCrc = true;

			sd_ret = SDHC_SendCmdWithRxData(sdhc_base, &sdh_cmd, &rsp, &sdh_data,
							data->data);
			if (sd_ret != SDHCRES_OK) {
				LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
				ret = -EIO;
			}
		}

		break;
	case MMC_SEND_OP_COND:
		sdh_cmd.IsResetCmd = false;
		sdh_cmd.IsStopCmd = false;
		sdh_cmd.IsRspExpected = true;
		sdh_cmd.IsR2Rsp = false;
		sdh_cmd.CheckRspCrc = false;

		sd_ret = SDHC_SendNoDataCmd(sdhc_base, &sdh_cmd, &cmd->response[0]);
		if (sd_ret != SDHCRES_OK) {
			LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
			ret = -EIO;
		}

		break;
	case SD_VOL_SWITCH:
		sdh_cmd.IsResetCmd = false;
		sdh_cmd.IsStopCmd = false;
		sdh_cmd.IsRspExpected = true;
		sdh_cmd.IsR2Rsp = false;
		sdh_cmd.CheckRspCrc = false;

		sd_ret = SDHC_SendNoDataCmd(sdhc_base, &sdh_cmd, &cmd->response[0]);
		if (sd_ret != SDHCRES_OK) {
			LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
			ret = -EIO;
		}

		break;
	case SD_APP_CMD:
		sdh_cmd.IsResetCmd = false;
		sdh_cmd.IsStopCmd = false;
		sdh_cmd.IsRspExpected = true;
		sdh_cmd.IsR2Rsp = false;
		sdh_cmd.CheckRspCrc = false;

		sd_ret = SDHC_SendNoDataCmd(sdhc_base, &sdh_cmd, &cmd->response[0]);
		if (sd_ret != SDHCRES_OK) {
			LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
			ret = -EIO;
		}

		break;
	case SD_SEND_STATUS:
		sdh_cmd.IsResetCmd = false;
		sdh_cmd.IsStopCmd = false;
		sdh_cmd.IsRspExpected = true;
		sdh_cmd.IsR2Rsp = false;
		sdh_cmd.CheckRspCrc = true;

		sd_ret = SDHC_SendNoDataCmd(sdhc_base, &sdh_cmd, &cmd->response[0]);
		if (sd_ret != SDHCRES_OK) {
			LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
			ret = -EIO;
		}

		break;
	case SD_SET_BLOCK_SIZE:
		sdh_cmd.IsResetCmd = false;
		sdh_cmd.IsStopCmd = false;
		sdh_cmd.IsRspExpected = true;
		sdh_cmd.IsR2Rsp = false;
		sdh_cmd.CheckRspCrc = true;

		sd_ret = SDHC_SendNoDataCmd(sdhc_base, &sdh_cmd, &cmd->response[0]);
		if (sd_ret != SDHCRES_OK) {
			LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
			ret = -EIO;
		}

		break;
	case SD_SEND_RELATIVE_ADDR:
		sdh_cmd.IsResetCmd = false;
		sdh_cmd.IsStopCmd = false;
		sdh_cmd.IsRspExpected = true;
		sdh_cmd.IsR2Rsp = false;
		sdh_cmd.CheckRspCrc = true;

		sd_ret = SDHC_SendNoDataCmd(sdhc_base, &sdh_cmd, &cmd->response[0]);
		if (sd_ret != SDHCRES_OK) {
			LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
			ret = -EIO;
		}

		break;
	case SD_APP_SEND_OP_COND:
		sdh_cmd.IsResetCmd = false;
		sdh_cmd.IsStopCmd = false;
		sdh_cmd.IsRspExpected = true;
		sdh_cmd.IsR2Rsp = false;
		sdh_cmd.CheckRspCrc = false;

		sd_ret = SDHC_SendNoDataCmd(sdhc_base, &sdh_cmd, &cmd->response[0]);
		if (sd_ret != SDHCRES_OK) {
			LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
			ret = -EIO;
		}

		break;
	case SD_APP_CLEAR_CARD_DETECT:
		sdh_cmd.IsResetCmd = false;
		sdh_cmd.IsStopCmd = false;
		sdh_cmd.IsRspExpected = true;
		sdh_cmd.IsR2Rsp = false;
		sdh_cmd.CheckRspCrc = true;

		sd_ret = SDHC_SendNoDataCmd(sdhc_base, &sdh_cmd, &cmd->response[0]);
		if (sd_ret != SDHCRES_OK) {
			LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
			ret = -EIO;
		}
		break;
	case SDIO_SEND_OP_COND:
		sdh_cmd.IsResetCmd = false;
		sdh_cmd.IsStopCmd = false;
		sdh_cmd.IsRspExpected = true;
		sdh_cmd.IsR2Rsp = false;
		sdh_cmd.CheckRspCrc = false;

		sd_ret = SDHC_SendNoDataCmd(sdhc_base, &sdh_cmd, &cmd->response[0]);
		if (sd_ret != SDHCRES_OK) {
			LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
			ret = -EIO;
		}

		break;
	case SD_SELECT_CARD:
		sdh_cmd.IsResetCmd = false;
		sdh_cmd.IsStopCmd = false;
		sdh_cmd.IsRspExpected = true;
		sdh_cmd.IsR2Rsp = false;
		sdh_cmd.CheckRspCrc = true;

		sd_ret = SDHC_SendNoDataCmd(sdhc_base, &sdh_cmd, &cmd->response[0]);
		if (sd_ret != SDHCRES_OK) {
			LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
			ret = -EIO;
		}
		break;
	case SD_ALL_SEND_CID:
		sdh_cmd.IsResetCmd = false;
		sdh_cmd.IsStopCmd = false;
		sdh_cmd.IsRspExpected = true;
		sdh_cmd.IsR2Rsp = true;
		sdh_cmd.CheckRspCrc = false;

		sd_ret = SDHC_SendNoDataCmd(sdhc_base, &sdh_cmd, &cmd->response[0]);
		if (sd_ret != SDHCRES_OK) {
			LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
			ret = -EIO;
		}

		break;
	case SD_SEND_CSD:
		sdh_cmd.IsResetCmd = false;
		sdh_cmd.IsStopCmd = false;
		sdh_cmd.IsRspExpected = true;
		sdh_cmd.IsR2Rsp = true;
		sdh_cmd.CheckRspCrc = false;

		sd_ret = SDHC_SendNoDataCmd(sdhc_base, &sdh_cmd, &cmd->response[0]);
		if (sd_ret != SDHCRES_OK) {
			LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
			ret = -EIO;
		}

		break;
	case SD_SWITCH:
		if (data != NULL) {
			sdh_data.BlockSize = data->block_size;
			sdh_data.BlockCount = data->blocks;
			sdh_data.SendAutoStop = false,

			sdh_cmd.IsResetCmd = false;
			sdh_cmd.IsStopCmd = false;
			sdh_cmd.IsRspExpected = true;
			sdh_cmd.IsR2Rsp = false;
			sdh_cmd.CheckRspCrc = true;

			sd_ret = SDHC_SendCmdWithRxData(sdhc_base, &sdh_cmd, &rsp, &sdh_data,
							data->data);
			if (sd_ret != SDHCRES_OK) {
				LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
				ret = -EIO;
			}

		} else {
			sdh_cmd.IsResetCmd = false;
			sdh_cmd.IsStopCmd = false;
			sdh_cmd.IsRspExpected = true;
			sdh_cmd.IsR2Rsp = false;
			sdh_cmd.CheckRspCrc = true;

			sd_ret = SDHC_SendNoDataCmd(sdhc_base, &sdh_cmd, &rsp);
			if (sd_ret != SDHCRES_OK) {
				LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
				ret = -EIO;
			}

			sd_ret = SDHC_WaitData0Idle(sdhc_base, 2000);
			if (sd_ret != SDHCRES_OK) {
				LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
				ret = -ETIMEDOUT;
			}
		}
		break;
	case SDIO_RW_DIRECT:
		sdh_cmd.IsResetCmd = false;
		sdh_cmd.IsStopCmd = false;
		sdh_cmd.IsRspExpected = true;
		sdh_cmd.IsR2Rsp = false;
		sdh_cmd.CheckRspCrc = true;

		sd_ret = SDHC_SendNoDataCmd(sdhc_base, &sdh_cmd, &cmd->response[0]);
		if (sd_ret != SDHCRES_OK) {
			LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
			ret = -EIO;
		}
		break;
	case SDIO_RW_EXTENDED:
		sdh_data.BlockSize = data->block_size;
		sdh_data.BlockCount = data->blocks;
		sdh_data.SendAutoStop = false,

		sdh_cmd.IsResetCmd = false;
		sdh_cmd.IsStopCmd = false;
		sdh_cmd.IsRspExpected = true;
		sdh_cmd.IsR2Rsp = false;
		sdh_cmd.CheckRspCrc = true;

		if ((cmd->arg & BIT(SDIO_CMD_ARG_RW_SHIFT))) {
			sd_ret = SDHC_SendCmdWithTxData(sdhc_base, &sdh_cmd, &cmd->response[0],
							&sdh_data, data->data);
		} else {
			sd_ret = SDHC_SendCmdWithRxData(sdhc_base, &sdh_cmd, &cmd->response[0],
							&sdh_data, data->data);
		}
		if (sd_ret != SDHCRES_OK) {
			LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
			ret = -EIO;
		}
		break;
	case SD_APP_SEND_SCR:
		sdh_cmd.IsResetCmd = false;
		sdh_cmd.IsStopCmd = false;
		sdh_cmd.IsRspExpected = true;
		sdh_cmd.IsR2Rsp = false;
		sdh_cmd.CheckRspCrc = true;

		sdh_data.BlockSize = data->block_size;
		sdh_data.BlockCount = data->blocks;
		sdh_data.SendAutoStop = false;

		sd_ret = SDHC_SendCmdWithRxData(sdhc_base, &sdh_cmd, &rsp, &sdh_data, data->data);
		if (sd_ret != SDHCRES_OK) {
			LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
			ret = -EIO;
		}

		break;
	case SD_READ_SINGLE_BLOCK:
	case SD_READ_MULTIPLE_BLOCK:
		pubuf = data->data;
		blockaddr = data->block_addr;
		remainblock = data->blocks;

		if (sdh_cmd.CmdArg == data->block_addr) {
			high_capacity = false;
		}
		while (remainblock > 0) {
			uint32_t BlockCntSend = MIN(MAX_BLOCK_PER_XFER, remainblock);

			sdh_cmd.CmdArg = high_capacity ? blockaddr * data->block_size : blockaddr;
			sdh_cmd.IsResetCmd = false;
			sdh_cmd.IsStopCmd = false;
			sdh_cmd.IsRspExpected = true;
			sdh_cmd.IsR2Rsp = false;
			sdh_cmd.CheckRspCrc = true;

			sdh_data.BlockSize = data->block_size;
			sdh_data.BlockCount = BlockCntSend;
			sdh_data.SendAutoStop =
				sdh_cmd.CmdIdx == SD_READ_MULTIPLE_BLOCK ? true : false;

			sd_ret =
				SDHC_SendCmdWithRxData(sdhc_base, &sdh_cmd, &rsp, &sdh_data, pubuf);
			if (sd_ret != SDHCRES_OK) {
				LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
				return -EIO;
			}

			pubuf += (BlockCntSend * data->block_size);
			blockaddr += BlockCntSend;
			remainblock -= BlockCntSend;
		}
		break;
	case SD_WRITE_SINGLE_BLOCK:
	case SD_WRITE_MULTIPLE_BLOCK:
		pubuf = data->data;
		blockaddr = data->block_addr;
		remainblock = data->blocks;

		if (sdh_cmd.CmdArg == data->block_addr) {
			high_capacity = false;
		}
		while (remainblock > 0) {
			uint32_t BlockCntSend = MIN(MAX_BLOCK_PER_XFER, remainblock);

			sdh_cmd.CmdArg = high_capacity ? blockaddr * data->block_size : blockaddr;
			sdh_cmd.IsResetCmd = false;
			sdh_cmd.IsStopCmd = false;
			sdh_cmd.IsRspExpected = true;
			sdh_cmd.IsR2Rsp = false;
			sdh_cmd.CheckRspCrc = true;

			sdh_data.BlockSize = data->block_size;
			sdh_data.BlockCount = BlockCntSend;
			sdh_data.SendAutoStop =
				sdh_cmd.CmdIdx == SD_WRITE_MULTIPLE_BLOCK ? true : false;

			sd_ret =
				SDHC_SendCmdWithTxData(sdhc_base, &sdh_cmd, &rsp, &sdh_data, pubuf);
			if (sd_ret != SDHCRES_OK) {
				LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
				ret = -EIO;
			}

			sd_ret = SDHC_WaitData0Idle(sdhc_base, 2000);
			if (sd_ret != SDHCRES_OK) {
				LOG_ERR("[%s] error: sd_ret=%d line%d", __func__, sd_ret, __LINE__);
				ret = -ETIMEDOUT;
			}

			pubuf += (BlockCntSend * data->block_size);
			blockaddr += BlockCntSend;
			remainblock -= BlockCntSend;
		}

		break;
	default:
		ret = -ENOTSUP;
	}

	if (dev_data->sdio_int_en) {
		sdhc_bee_enable_interrupt_pin(dev);
	}

	return ret;
}

static int sdhc_bee_set_io(const struct device *dev, struct sdhc_io *ios)
{
	const struct sdhc_bee_config *cfg = dev->config;
	SDHC_TypeDef *sdhc_base = (SDHC_TypeDef *)cfg->sdhc_base;
	struct sdhc_bee_data *data = dev->data;
	uint8_t bus_width;

	LOG_INF("SDHC I/O: dev: %s, bus width %d, clock %dHz, card power %s, voltage %s", dev->name,
		ios->bus_width, ios->clock, ios->power_mode == SDHC_POWER_ON ? "ON" : "OFF",
		ios->signal_voltage == SD_VOL_1_8_V ? "1.8V" : "3.3V");

	if (ios->clock) {
		/* Check for frequency boundaries supported by host */
		if (ios->clock > cfg->props.f_max || ios->clock < cfg->props.f_min) {
			LOG_ERR("SDHC host supports clock between %dHz to %dHz", cfg->props.f_min,
				cfg->props.f_max);
		}

		if (data->bus_clock != (uint32_t)ios->clock) {
			SDHC_SetClkOutFreq(sdhc_base, ios->clock / 1000);
			data->bus_clock = SDHC_GetClkOutFreq_kHz(sdhc_base) * 1000;
			LOG_INF("Bus clock set to %d kHz", SDHC_GetClkOutFreq_kHz(sdhc_base));
		}
	}

	if (ios->bus_width) {
		/* Set bus width */
		switch (ios->bus_width) {
		case SDHC_BUS_WIDTH1BIT:
			bus_width = 1;
			break;
		case SDHC_BUS_WIDTH4BIT:
			bus_width = 4;
			break;
		default:
			return -ENOTSUP;
		}

		if (data->bus_width != bus_width) {
			SDHC_SetHostDataWidth(sdhc_base,
					      bus_width == 1 ? DATAWIDTH_1BIT : DATAWIDTH_4BIT);
			LOG_INF("Bus width set to %d bit", bus_width);

			data->bus_width = bus_width;
		}
	}

	/* Toggle card power supply */
	if ((data->power_mode != ios->power_mode) && (cfg->pwr_gpio.port)) {
		if (ios->power_mode == SDHC_POWER_OFF) {
			gpio_pin_set_dt(&cfg->pwr_gpio, 0);
		} else if (ios->power_mode == SDHC_POWER_ON) {
			gpio_pin_set_dt(&cfg->pwr_gpio, 1);
		}
		data->power_mode = ios->power_mode;
	}

	if (ios->timing) {
		/* Set I/O timing */
		if (data->timing != ios->timing) {
			switch (ios->timing) {
			case SDHC_TIMING_LEGACY:
			case SDHC_TIMING_HS:
				break;
			case SDHC_TIMING_SDR12:
			case SDHC_TIMING_SDR25:
			case SDHC_TIMING_DDR50:
			case SDHC_TIMING_DDR52:
			case SDHC_TIMING_SDR50:
			case SDHC_TIMING_HS400:
			case SDHC_TIMING_SDR104:
			case SDHC_TIMING_HS200:
			default:
				LOG_ERR("Timing mode not supported for this device");
				return -ENOTSUP;
			}

			LOG_INF("Bus timing successfully changed to %d", ios->timing);
			data->timing = ios->timing;
		}
	}

	return 0;
}

static int sdhc_bee_request(const struct device *dev, struct sdhc_command *cmd,
			    struct sdhc_data *data)
{
	LOG_INF("[%s] opcode=%d arg=0x%x data=0x%x", __func__, cmd->opcode, cmd->arg,
		(uint32_t)data);
	struct sdhc_bee_data *dev_data = (struct sdhc_bee_data *)dev->data;
	int retries = (int)(cmd->retries + 1);
	int ret = 0;

	if (k_mutex_lock(&dev_data->s_request_mutex, K_FOREVER) != 0) {
		return -ETIMEDOUT;
	}

	do {
		ret = sdhc_bee_do_transaction(dev, cmd, data);
		if (!ret) {
			break;
		}
	} while (--retries);

	if (ret) {
		LOG_ERR("SDHC send command %d error %d", cmd->opcode, ret);
	}

	k_mutex_unlock(&dev_data->s_request_mutex);
	return ret;
}

static int sdhc_bee_reset(const struct device *dev)
{
	const struct sdhc_bee_config *cfg = dev->config;
	SDHC_TypeDef *sdhc_base = (SDHC_TypeDef *)cfg->sdhc_base;

	ResetAll(sdhc_base);

	return 0;
}

static int sdhc_bee_get_card_present(const struct device *dev)
{
	return 1;
}

static int sdhc_bee_card_busy(const struct device *dev)
{
	return false;
}

static int sdhc_bee_get_host_props(const struct device *dev, struct sdhc_host_props *props)
{
	const struct sdhc_bee_config *cfg = dev->config;

	memcpy(props, &cfg->props, sizeof(struct sdhc_host_props));
	return 0;
}

static int sdhc_bee_enable_interrupt(const struct device *dev, sdhc_interrupt_cb_t callback,
				     int sources, void *user_data)
{
	LOG_INF("[%s] line%d", __func__, __LINE__);
	struct sdhc_bee_data *data = dev->data;
	int ret;

	data->cb = callback;
	data->user_data = user_data;

	if (data->sdio_int_en) {
		return 0;
	}

	if (sources & SDHC_INT_SDIO) {
		ret = sdhc_bee_enable_interrupt_pin(dev);
		if (ret) {
			LOG_ERR("Enable interrupt fail. int-gpio should be configured in 4 "
				"bit mode");
			return -EIO;
		}
		data->sdio_int_en = true;
	} else {
		LOG_ERR("Enable interrupt fail. Only support SDHC_INT_SDIO");
		return -ENOTSUP;
	}

	return 0;
}

static int sdhc_bee_disable_interrupt(const struct device *dev, int sources)
{
	LOG_INF("[%s] line%d", __func__, __LINE__);
	struct sdhc_bee_data *data = dev->data;
	int ret;

	if (sources & SDHC_INT_SDIO) {
		ret = sdhc_bee_disable_interrupt_pin(dev);
		data->sdio_int_en = false;
	} else {
		LOG_ERR("Disable interrupt fail. Only support SDHC_INT_SDIO");
		return -ENOTSUP;
	}

	data->cb = NULL;
	data->user_data = NULL;

	return 0;
}

static void sdio_bee_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;
	const struct sdhc_bee_config *cfg = dev->config;
	SDHC_TypeDef *sdhc_base = (SDHC_TypeDef *)cfg->sdhc_base;

	DisableIntrByNvic(sdhc_base);

	if (sdhc_base == SDHC0) {
		os_sem_give(gSDHC0Sem);
	} else {
		os_sem_give(gSDHC1Sem);
	}
}

static int sdhc_bee_init(const struct device *dev)
{
	const struct sdhc_bee_config *cfg = dev->config;
	struct sdhc_bee_data *data = dev->data;
	SDHC_TypeDef *sdhc_base = (SDHC_TypeDef *)cfg->sdhc_base;
	int ret;

	LOG_INF("[%s] %s initializing line%d", __func__, dev->name, __LINE__);

	/* Pin configuration */
	ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);

	if (ret < 0) {
		LOG_ERR("Failed to configure SDHC pins");
		return ret;
	}

	ret = clock_control_on(BEE_CLOCK_CONTROLLER, (clock_control_subsys_t)&cfg->clkid);

	if (ret != 0) {
		LOG_ERR("Error enabling SDHC clock");
		return ret;
	}

	pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_INTERRUPT);

	if (cfg->int_gpio.port) {
		gpio_init_callback(&sdio_int_gpio_cb, sdio_int_gpio_cb_func,
				   BIT(cfg->int_gpio.pin));
		gpio_add_callback(cfg->int_gpio.port, &sdio_int_gpio_cb);
	}

	/* Set power GPIO high, so card starts powered */
	if (cfg->pwr_gpio.port) {
		ret = gpio_pin_configure_dt(&cfg->pwr_gpio, GPIO_OUTPUT_ACTIVE);

		if (ret) {
			LOG_ERR("Failed to configure SDHC power pins");
			return ret;
		}
	}

	InitClk(sdhc_base);

	if (sdhc_bee_reset(dev)) {
		LOG_ERR("Fail to reset SDHC");
		return -EFAULT;
	}

	((SDHC_TypeDef *)sdhc_base)->RINTSTS = 0xffffffff;
	((SDHC_TypeDef *)sdhc_base)->INTMASK = 0;

	SDHC_CTRL_t ctrl = {.d32 = ((SDHC_TypeDef *)sdhc_base)->CTRL};

	ctrl.b.int_enable = 1;
	ctrl.b.use_internal_dmac = 1;
	((SDHC_TypeDef *)sdhc_base)->CTRL = ctrl.d32;

	SDHC_SetClkOutFreq(sdhc_base, 400);

	SDHC_SetHostDataWidth(sdhc_base, data->bus_width == 1 ? DATAWIDTH_1BIT : DATAWIDTH_4BIT);

	if (sdhc_base == SDHC0) {
		os_sem_create(&gSDHC0Sem, "gSDHC0Sem", 0, 1);
	} else {
		os_sem_create(&gSDHC1Sem, "gSDHC1Sem", 0, 1);
	}

	cfg->sd_irq_connect();

	k_mutex_init(&data->s_request_mutex);

	return 0;
}

#ifdef CONFIG_PM_DEVICE
static int sdhc_bee_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct sdhc_bee_config *config = dev->config;
	struct sdhc_bee_data *data = dev->data;
	SDHC_TypeDef *sdhc_base = (SDHC_TypeDef *)config->sdhc_base;
	int err;

	switch (action) {
	case PM_DEVICE_ACTION_SUSPEND:
		SDHC_DLPSEnter(sdhc_base, &data->store_buf);

		/* Move pins to sleep state */
		err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_SLEEP);
		if ((err < 0) && (err != -ENOENT)) {
			return err;
		}

		break;
	case PM_DEVICE_ACTION_RESUME:
		/* Set pins to active state */
		err = pinctrl_apply_state(config->pcfg, PINCTRL_STATE_DEFAULT);
		if (err < 0) {
			return err;
		}

		if (!data->sdio_int_en) {
			pinctrl_apply_state(config->pcfg, PINCTRL_STATE_INTERRUPT);
		}

		(void)clock_control_on(BEE_CLOCK_CONTROLLER,
				       (clock_control_subsys_t)&config->clkid);

		SDHC_DLPSExit(sdhc_base, &data->store_buf);
		SDHC_SetClkOutFreq(sdhc_base, data->bus_clock / 1000);
		SDHC_SetHostDataWidth(sdhc_base,
				      data->bus_width == 1 ? DATAWIDTH_1BIT : DATAWIDTH_4BIT);

		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}
#endif /* CONFIG_PM_DEVICE */

static const struct sdhc_driver_api sdhc_api = {
	.reset = sdhc_bee_reset,
	.request = sdhc_bee_request,
	.set_io = sdhc_bee_set_io,
	.get_card_present = sdhc_bee_get_card_present,
	.card_busy = sdhc_bee_card_busy,
	.get_host_props = sdhc_bee_get_host_props,
	.enable_interrupt = sdhc_bee_enable_interrupt,
	.disable_interrupt = sdhc_bee_disable_interrupt,
};

#define SDHC_BEE_INIT(n)                                                                           \
                                                                                                   \
	PINCTRL_DT_DEFINE(DT_DRV_INST(n));                                                         \
	static void sdio_bee_irq_enable_##n(void)                                                  \
	{                                                                                          \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
	static void sdio_bee_irq_disable_##n(void)                                                 \
	{                                                                                          \
		irq_disable(DT_INST_IRQN(n));                                                      \
	}                                                                                          \
	static void sdio_bee_irq_connect_##n(void)                                                 \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), sdio_bee_isr,               \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}                                                                                          \
	static const struct sdhc_bee_config sdhc_bee_##n##_config = {                              \
		.sdhc_base = (SDHC_TypeDef *)DT_INST_REG_ADDR(n),                                  \
		.clkid = DT_INST_CLOCKS_CELL(n, id),                                               \
		.sd_irq_connect = sdio_bee_irq_connect_##n,                                        \
		.sd_irq_enable = sdio_bee_irq_enable_##n,                                          \
		.sd_irq_disable = sdio_bee_irq_disable_##n,                                        \
		.pin_group = DT_INST_PROP(n, pin_group),                                           \
		.pcfg = PINCTRL_DT_DEV_CONFIG_GET(DT_DRV_INST(n)),                                 \
		.pwr_gpio = GPIO_DT_SPEC_INST_GET_OR(n, pwr_gpios, {0}),                           \
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(n, int_gpios, {0}),                           \
		.props = {.is_spi = false,                                                         \
			  .f_max = DT_INST_PROP(n, max_bus_freq),                                  \
			  .f_min = DT_INST_PROP(n, min_bus_freq),                                  \
			  .max_current_330 = DT_INST_PROP(n, max_current_330),                     \
			  .max_current_180 = DT_INST_PROP(n, max_current_180),                     \
			  .power_delay = DT_INST_PROP_OR(n, power_delay_ms, 0),                    \
			  .host_caps = {.vol_180_support = false,                                  \
					.vol_300_support = false,                                  \
					.vol_330_support = true,                                   \
					.suspend_res_support = false,                              \
					.sdma_support = false,                                     \
					.high_spd_support = true,                                  \
					.adma_2_support = false,                                   \
					.max_blk_len = 0,                                          \
					.ddr50_support = false,                                    \
					.sdr104_support = false,                                   \
					.sdr50_support = false,                                    \
					.uhs_2_support = false,                                    \
					.bus_8_bit_support = false,                                \
					.bus_4_bit_support =                                       \
						(DT_INST_PROP(n, bus_width) == 4) ? true : false,  \
					.hs200_support = false,                                    \
					.hs400_support = false}}};                                 \
                                                                                                   \
	static struct sdhc_bee_data sdhc_bee_##n##_data = {                                        \
		.bus_width = DT_INST_PROP(n, bus_width),                                           \
		.src_clock = 40000000,                                                             \
		.bus_clock = 5000000,                                                              \
		.power_mode = SDHC_POWER_ON,                                                       \
		.timing = SDHC_TIMING_LEGACY,                                                      \
	};                                                                                         \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(n, sdhc_bee_pm_action);                                           \
	DEVICE_DT_INST_DEFINE(n, &sdhc_bee_init, PM_DEVICE_DT_INST_GET(n), &sdhc_bee_##n##_data,   \
			      &sdhc_bee_##n##_config, POST_KERNEL, CONFIG_SDHC_INIT_PRIORITY,      \
			      &sdhc_api);

DT_INST_FOREACH_STATUS_OKAY(SDHC_BEE_INIT)
