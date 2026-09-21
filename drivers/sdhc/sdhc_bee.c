/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
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
#include <math.h>

#include "os_sync.h"
#include "rtl_sdhc.h"

LOG_MODULE_REGISTER(sdhc, CONFIG_SDHC_LOG_LEVEL);

#define PINCTRL_STATE_INTERRUPT (PINCTRL_STATE_PRIV_START + 1)

#define SDHC_RINTSTS_ERR_MASK                                                                      \
	(BIT(1) | BIT(6) | BIT(7) | BIT(8) | BIT(9) | BIT(10) | BIT(11) | BIT(12) | BIT(13) |      \
	 BIT(15))
#define SDHC_IDSTS_ERR_MASK (BIT(2) | BIT(4) | BIT(5))

/* IDMAC status bits used for Descriptor-Unavailable poll-demand recovery. */
#define SDHC_IDSTS_DU  BIT(4)
#define SDHC_IDSTS_AIS BIT(9)

/*
 * Transfer-completion bits (standard DWC_mobile_storage layout). A poll-demand
 * re-fetch must never be issued for an IRQ that already carries completion:
 * RINTSTS bit3 = Data-Transfer-Over, IDSTS bit0/1 = DMA Tx/Rx Interrupt. If a
 * stale prefetch DU is latched in the SAME interrupt as one of these, the
 * transfer is already done; swallowing it (return without giving the semaphore)
 * would lose the wakeup and deadlock the blob's os_sem_take.
 */
#define SDHC_RINTSTS_DTO   BIT(3)
#define SDHC_IDSTS_TI      BIT(0)
#define SDHC_IDSTS_RI      BIT(1)
#define SDHC_IDSTS_DMA_DONE (SDHC_IDSTS_TI | SDHC_IDSTS_RI)

/*
 * IDMAC state-machine field (IDSTS[16:13]). On the write path the IDMAC can
 * stop in the SUSPEND state holding a valid, owned descriptor without ever
 * asserting DU, so the DU re-poll never fires and the transfer stalls; a
 * poll-demand from the completion-command IRQ resumes it.
 */
#define SDHC_IDSTS_FSM_MASK    (0xFU << 13)
#define SDHC_IDSTS_FSM_SUSPEND (0x1U << 13)

/*
 * Max IDMAC poll-demand re-fetches per transfer. At 50 MHz the IDMAC's initial
 * descriptor fetch can win the race against the blob's OWN-bit store, so it
 * suspends with DU even though the descriptor is valid; a poll-demand makes it
 * re-fetch the settled descriptor. The cap keeps a genuinely broken descriptor
 * from spinning forever.
 */
#define SDHC_DU_REPOLL_MAX 8

#define SDHC_CTRL_INT_ENABLE        BIT(4)
#define SDHC_CTRL_USE_INTERNAL_DMAC BIT(25)

#define SDHC_STATUS_DATA_BUSY BIT(9)

#define SDHC_RINTSTS_CLEAR_ALL 0xffffffffU
#define SDHC_IDSTS_CLEAR_ALL   0xffffffffU

#define SDHC_ID_CLK_FREQ_KHZ 400

#define SDHC_DATA0_IDLE_TIMEOUT_MS 2000

#define SDHC_SRC_CLOCK_HZ      40000000
#define SDHC_INIT_BUS_CLOCK_HZ 5000000

struct gpio_callback sdio_int_gpio_cb;

#define DEVICE_DT_GET_AND_COMMA(node_id) DEVICE_DT_GET(node_id),
static const struct device *const devices[] = {
	DT_FOREACH_STATUS_OKAY(DT_DRV_COMPAT, DEVICE_DT_GET_AND_COMMA)};

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
	bool recovery_failed;
	uint32_t cur_opcode;
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

/* Only block reads/writes can wedge mid-transfer and need CMD12 recovery. */
static bool sdhc_bee_opcode_needs_recovery(uint32_t opcode)
{
	switch (opcode) {
	case SD_READ_SINGLE_BLOCK:
	case SD_READ_MULTIPLE_BLOCK:
	case SD_WRITE_SINGLE_BLOCK:
	case SD_WRITE_MULTIPLE_BLOCK:
		return true;
	default:
		return false;
	}
}

/* Recover after a failed data transfer: reset the datapath, drop stale status,
 * abort on the card with CMD12, and confirm DAT0 is released before retrying.
 */
static int sdhc_bee_err_recovery(const struct device *dev, uint32_t timeout_ms)
{
	const struct sdhc_bee_config *cfg = dev->config;
	SDHC_TypeDef *sdhc_base = (SDHC_TypeDef *)cfg->sdhc_base;
	CmdInfo_t stop_cmd = {
		.CmdIdx = SD_STOP_TRANSMISSION,
		.CmdArg = 0,
		.IsResetCmd = false,
		.IsStopCmd = true,
		.IsRspExpected = true,
		.IsR2Rsp = false,
		.CheckRspCrc = true,
	};
	SDHCRes_t sd_ret;
	uint32_t stop_resp = 0;
	uint32_t freq_khz;

	freq_khz = SDHC_GetClkOutFreq_kHz(sdhc_base);
	ResetAll(sdhc_base);
	sdhc_base->CTRL |= SDHC_CTRL_USE_INTERNAL_DMAC | SDHC_CTRL_INT_ENABLE;

	sdhc_base->RINTSTS = SDHC_RINTSTS_CLEAR_ALL;
	sdhc_base->IDSTS = SDHC_IDSTS_CLEAR_ALL;

	SDHC_SetClkOutFreq(sdhc_base, freq_khz);

	sd_ret = SDHC_SendNoDataCmd(sdhc_base, &stop_cmd, &stop_resp);
	if (sd_ret != SDHCRES_OK) {
		LOG_ERR("SDHC err recovery: CMD12 failed: %d", sd_ret);
		return -EIO;
	}

	sd_ret = SDHC_WaitData0Idle(sdhc_base, timeout_ms);
	if (sd_ret != SDHCRES_OK) {
		LOG_ERR("SDHC err recovery: DAT0 not idle after CMD12");
		return -EIO;
	}

	return 0;
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
	uint32_t busy_timeout = cmd->timeout_ms ? cmd->timeout_ms : SDHC_DATA0_IDLE_TIMEOUT_MS;

	if (dev_data->sdio_int_en) {
		sdhc_bee_disable_interrupt_pin(dev);
	}

	dev_data->cur_opcode = cmd->opcode;
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
				ret = -EIO;
			}

			sd_ret = SDHC_WaitData0Idle(sdhc_base, busy_timeout);
			if (sd_ret != SDHCRES_OK) {
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
			ret = -EIO;
		}
		break;
	case SD_APP_SEND_SCR:
	case SD_APP_SEND_NUM_WRITTEN_BLK:
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
				ret = -EIO;
				break;
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
				ret = -EIO;
			}

			sd_ret = SDHC_WaitData0Idle(sdhc_base, busy_timeout);
			if (sd_ret != SDHCRES_OK) {
				ret = -ETIMEDOUT;
			}

			pubuf += (BlockCntSend * data->block_size);
			blockaddr += BlockCntSend;
			remainblock -= BlockCntSend;
		}

		break;
	case SD_ERASE_BLOCK_START:       /* CMD32: SD erase range start */
	case SD_ERASE_BLOCK_END:         /* CMD33: SD erase range end */
	case SD_ERASE_BLOCK_OPERATION:   /* CMD38: erase, holds DAT0 busy */
		sdh_cmd.IsResetCmd = false;
		sdh_cmd.IsStopCmd = false;
		sdh_cmd.IsRspExpected = true;
		sdh_cmd.IsR2Rsp = false;
		sdh_cmd.CheckRspCrc = true;

		sd_ret = SDHC_SendNoDataCmd(sdhc_base, &sdh_cmd, &cmd->response[0]);
		if (sd_ret != SDHCRES_OK) {
			ret = -EIO;
		}

		/* CMD38 (ERASE) keeps DAT0 busy until the erase completes. */
		if (cmd->opcode == SD_ERASE_BLOCK_OPERATION) {
			sd_ret = SDHC_WaitData0Idle(sdhc_base, busy_timeout);
			if (sd_ret != SDHCRES_OK) {
				ret = -ETIMEDOUT;
			}
		}
		break;
	default:
		ret = -ENOTSUP;
	}

	if (ret != 0 && ret != -ENOTSUP) {
		if (sdhc_bee_opcode_needs_recovery(cmd->opcode)) {
			if (sdhc_bee_err_recovery(dev, busy_timeout) != 0) {
				dev_data->recovery_failed = true;
			}
		} else {
			sdhc_base->RINTSTS = SDHC_RINTSTS_CLEAR_ALL;
			sdhc_base->IDSTS = SDHC_IDSTS_CLEAR_ALL;
		}
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
	uint32_t freq_khz;

	LOG_INF("SDHC I/O: dev: %s, bus width %d, clock %dHz, card power %s, voltage %s", dev->name,
		ios->bus_width, ios->clock, ios->power_mode == SDHC_POWER_ON ? "ON" : "OFF",
		ios->signal_voltage == SD_VOL_1_8_V ? "1.8V" : "3.3V");

	if (ios->clock) {
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

	if (data->power_mode != ios->power_mode) {
		if (ios->power_mode == SDHC_POWER_OFF) {
			if (cfg->pwr_gpio.port) {
				gpio_pin_set_dt(&cfg->pwr_gpio, 0);
			}
		} else if (ios->power_mode == SDHC_POWER_ON) {
			if (cfg->pwr_gpio.port) {
				gpio_pin_set_dt(&cfg->pwr_gpio, 1);
			}

			/*
			 * sd_init_io() "resets the card" solely by toggling
			 * power_mode OFF->ON through this path; it never calls
			 * sdhc_reset(). So the host-side reset must run here on
			 * every power-up, independent of whether a physical
			 * power-control GPIO exists. Clearing only the W1C status
			 * bits is not enough: an IDMAC left in SUSPEND holding an
			 * unowned descriptor (e.g. after an aborted transfer)
			 * keeps re-asserting DU, which the per-command error check
			 * would then misread as a failure on every subsequent
			 * command. ResetAll() resets the IDMAC state machine;
			 * save and restore the clock it clears.
			 */
			freq_khz = SDHC_GetClkOutFreq_kHz(sdhc_base);
			ResetAll(sdhc_base);
			sdhc_base->CTRL |= SDHC_CTRL_USE_INTERNAL_DMAC | SDHC_CTRL_INT_ENABLE;
			sdhc_base->RINTSTS = SDHC_RINTSTS_CLEAR_ALL;
			sdhc_base->IDSTS = SDHC_IDSTS_CLEAR_ALL;
			SDHC_SetClkOutFreq(sdhc_base, freq_khz);
		}
		data->power_mode = ios->power_mode;
	}

	if (ios->timing) {
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
	LOG_DBG("opcode=%d arg=0x%x data=%p", cmd->opcode, cmd->arg, (void *)data);
	struct sdhc_bee_data *dev_data = (struct sdhc_bee_data *)dev->data;
	int retries = (int)(cmd->retries + 1);
	int ret = 0;
	k_timeout_t lock_timeout = cmd->timeout_ms ? K_MSEC(cmd->timeout_ms) : K_FOREVER;

	if (k_mutex_lock(&dev_data->s_request_mutex, lock_timeout) != 0) {
		return -ETIMEDOUT;
	}

	dev_data->recovery_failed = false;

	do {
		ret = sdhc_bee_do_transaction(dev, cmd, data);
		if (!ret) {
			break;
		}
		/* A wedged data transfer is unrecoverable; stop retrying. */
		if (dev_data->recovery_failed) {
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
	const struct sdhc_bee_config *cfg = dev->config;
	SDHC_TypeDef *sdhc_base = (SDHC_TypeDef *)cfg->sdhc_base;

	/* STATUS.data_busy is set while the card holds DAT0 low. */
	return (sdhc_base->STATUS & SDHC_STATUS_DATA_BUSY) ? 1 : 0;
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

/* Debug (CONFIG_SDHC_BEE_ISR_TRACE): ring-buffer every interrupt and dump it on
 * error, exposing the RINTSTS/IDSTS sequence leading up to the failure.
 */
#ifdef CONFIG_SDHC_BEE_ISR_TRACE
#define SDHC_ISR_TRACE_DEPTH 32

struct sdhc_isr_trace_entry {
	uint32_t seq;
	uint32_t rintsts;
	uint32_t idsts;
	uint8_t opcode;
};

static struct sdhc_isr_trace_entry sdhc_isr_trace[SDHC_ISR_TRACE_DEPTH];
static uint32_t sdhc_isr_trace_head;
static uint32_t sdhc_isr_seq;

static void sdhc_isr_trace_record(uint8_t opcode, uint32_t rintsts, uint32_t idsts)
{
	struct sdhc_isr_trace_entry *e = &sdhc_isr_trace[sdhc_isr_trace_head];

	e->seq = ++sdhc_isr_seq;
	e->opcode = opcode;
	e->rintsts = rintsts;
	e->idsts = idsts;
	sdhc_isr_trace_head = (sdhc_isr_trace_head + 1) % SDHC_ISR_TRACE_DEPTH;
}

static void sdhc_isr_trace_dump(void)
{
	LOG_ERR("== SDHC ISR trace (oldest -> newest) ==");
	for (uint32_t i = 0; i < SDHC_ISR_TRACE_DEPTH; i++) {
		uint32_t idx = (sdhc_isr_trace_head + i) % SDHC_ISR_TRACE_DEPTH;
		struct sdhc_isr_trace_entry *e = &sdhc_isr_trace[idx];

		if (e->seq == 0) {
			continue;
		}
		LOG_ERR("  #%u cmd%u RINTSTS=0x%08x IDSTS=0x%08x", e->seq, e->opcode, e->rintsts,
			e->idsts);
	}
}

/*
 * Snapshot the IDMAC descriptor state at the moment of an error, to tell a
 * cache-coherency DU (descriptor OWN=1 in memory but IDMAC read a stale 0)
 * apart from a genuinely unowned/clobbered descriptor.
 */
static void sdhc_bee_dump_dma_state(SDHC_TypeDef *sdhc_base)
{
	uint32_t dbaddr = sdhc_base->DBADDR;
	uint32_t dscaddr = sdhc_base->DSCADDR;

	LOG_ERR("  IDMAC: DBADDR=0x%08x DSCADDR=0x%08x", dbaddr, dscaddr);

	if (dbaddr != 0U) {
		volatile uint32_t *d = (volatile uint32_t *)dbaddr;

		/* DES0=ctrl/OWN(bit31), DES1=buffer sizes, DES2=buf1 addr, DES3=buf2 addr */
		LOG_ERR("  DESC@0x%08x: DES0=0x%08x(OWN=%u) DES1=0x%08x DES2=0x%08x DES3=0x%08x",
			dbaddr, d[0], (uint32_t)((d[0] >> 31) & 1U), d[1], d[2], d[3]);
	}
}
#endif /* CONFIG_SDHC_BEE_ISR_TRACE */

static void sdio_bee_isr(void *arg)
{
	const struct device *dev = (const struct device *)arg;
	const struct sdhc_bee_config *cfg = dev->config;
	struct sdhc_bee_data *dev_data = dev->data;
	SDHC_TypeDef *sdhc_base = (SDHC_TypeDef *)cfg->sdhc_base;
	uint32_t rintsts = sdhc_base->RINTSTS;
	uint32_t idsts = sdhc_base->IDSTS;

	cfg->sd_irq_disable();

#ifdef CONFIG_SDHC_BEE_ISR_TRACE
	sdhc_isr_trace_record(dev_data->cur_opcode, rintsts, idsts);
#endif

	static uint32_t du_repoll;
	/*
	 * Only re-poll for a Descriptor-Unavailable that stands alone. If this
	 * IRQ also carries transfer completion (DTO / DMA Tx-Rx) the transfer is
	 * already done and the DU is a stale prefetch artifact of the 50 MHz
	 * fetch-vs-OWN-store race; swallowing it here would lose the wakeup and
	 * deadlock the blob's os_sem_take (permanent hang on the write path).
	 */
	if ((idsts & SDHC_IDSTS_DU) && !(rintsts & SDHC_RINTSTS_ERR_MASK) &&
	    !(rintsts & SDHC_RINTSTS_DTO) && !(idsts & SDHC_IDSTS_DMA_DONE) &&
	    du_repoll < SDHC_DU_REPOLL_MAX) {
		du_repoll++;
		sdhc_base->IDSTS = idsts & (SDHC_IDSTS_DU | SDHC_IDSTS_AIS);
		sdhc_base->PLDMND = 1U;
		cfg->sd_irq_enable();
		return;
	}
	du_repoll = 0;

	/*
	 * A DU latched together with completion is the stale prefetch artifact,
	 * not a failure: ack it in hardware and drop it locally so it is not
	 * misclassified as a descriptor error (below) or misread by the blob.
	 * A DU that survives with no completion (e.g. re-poll cap hit) is left
	 * set and is reported as a genuine error.
	 */
	if ((idsts & SDHC_IDSTS_DU) &&
	    ((rintsts & SDHC_RINTSTS_DTO) || (idsts & SDHC_IDSTS_DMA_DONE))) {
		sdhc_base->IDSTS = SDHC_IDSTS_DU | SDHC_IDSTS_AIS;
		idsts &= ~(SDHC_IDSTS_DU | SDHC_IDSTS_AIS);
	}

	/*
	 * Write-path stall recovery: the IDMAC has stopped in the SUSPEND state
	 * with the data transfer not yet over and no error. It never raised DU
	 * (so the re-poll above did not run) but still needs a poll-demand to
	 * resume. Issue it here; the ensuing completion IRQ releases the blob's
	 * semaphore normally. A poll-demand on a non-suspended IDMAC is a no-op,
	 * so this is harmless on healthy transfers.
	 */
	if ((idsts & SDHC_IDSTS_FSM_MASK) == SDHC_IDSTS_FSM_SUSPEND &&
	    !(rintsts & SDHC_RINTSTS_DTO) && !(rintsts & SDHC_RINTSTS_ERR_MASK) &&
	    !(idsts & SDHC_IDSTS_ERR_MASK)) {
		sdhc_base->PLDMND = 1U;
	}

	if ((rintsts & SDHC_RINTSTS_ERR_MASK) || (idsts & SDHC_IDSTS_ERR_MASK)) {
		LOG_ERR("cmd %u error: RINTSTS=0x%08x IDSTS=0x%08x", dev_data->cur_opcode, rintsts,
			idsts);
#ifdef CONFIG_SDHC_BEE_ISR_TRACE
		sdhc_isr_trace_dump();

		/* Dump the IDMAC descriptor state once, at the first error. */
		static bool dma_state_dumped;

		if (!dma_state_dumped) {
			dma_state_dumped = true;
			sdhc_bee_dump_dma_state(sdhc_base);
		}
#endif
	}

	os_sem_give(gSDHC0Sem);
}

static int sdhc_bee_init(const struct device *dev)
{
	const struct sdhc_bee_config *cfg = dev->config;
	struct sdhc_bee_data *data = dev->data;
	SDHC_TypeDef *sdhc_base = (SDHC_TypeDef *)cfg->sdhc_base;
	int ret;

	LOG_DBG("initializing %s", dev->name);

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

	((SDHC_TypeDef *)sdhc_base)->RINTSTS = SDHC_RINTSTS_CLEAR_ALL;
	((SDHC_TypeDef *)sdhc_base)->INTMASK = 0;
	((SDHC_TypeDef *)sdhc_base)->CTRL |= SDHC_CTRL_USE_INTERNAL_DMAC | SDHC_CTRL_INT_ENABLE;

	SDHC_SetClkOutFreq(sdhc_base, SDHC_ID_CLK_FREQ_KHZ);

	SDHC_SetHostDataWidth(sdhc_base, data->bus_width == 1 ? DATAWIDTH_1BIT : DATAWIDTH_4BIT);

	os_sem_create(&gSDHC0Sem, "gSDHC0Sem", 0, 1);

	cfg->sd_irq_connect();

	k_mutex_init(&data->s_request_mutex);

	return 0;
}

static DEVICE_API(sdhc, sdhc_bee_api) = {
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
			  .bus_4_bit_support = (DT_INST_PROP(n, bus_width) == 4),                  \
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
					.bus_8_bit_support = false}}};                             \
                                                                                                   \
	static struct sdhc_bee_data sdhc_bee_##n##_data = {                                        \
		.bus_width = DT_INST_PROP(n, bus_width),                                           \
		.src_clock = SDHC_SRC_CLOCK_HZ,                                                    \
		.bus_clock = SDHC_INIT_BUS_CLOCK_HZ,                                               \
		.power_mode = SDHC_POWER_ON,                                                       \
		.timing = SDHC_TIMING_LEGACY,                                                      \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &sdhc_bee_init, NULL, &sdhc_bee_##n##_data,                       \
			      &sdhc_bee_##n##_config, POST_KERNEL, CONFIG_SDHC_INIT_PRIORITY,      \
			      &sdhc_bee_api);

DT_INST_FOREACH_STATUS_OKAY(SDHC_BEE_INIT)
