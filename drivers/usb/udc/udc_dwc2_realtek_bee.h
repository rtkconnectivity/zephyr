/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_USB_UDC_DWC2_REALTEK_BEE_H
#define ZEPHYR_DRIVERS_USB_UDC_DWC2_REALTEK_BEE_H

/*
 * Realtek Bee vendor quirk for the DWC2 controller: runs the Realtek USB PHY
 * bring-up and advertises the core's High-Speed capability.
 */
extern void hal_usb_phy_power_on(void);
extern void usb_rtk_disable_power_seq(void);
extern void hal_rtk_usb_init(void);
extern int hal_usb_suspend_enter(void);
extern int32_t usb_rtk_resume_sequence(void);

static int realtek_bee_udc_init(const struct device *dev)
{
	ARG_UNUSED(dev);

	hal_usb_phy_power_on();
	hal_rtk_usb_init();

	return 0;
}

static int realtek_bee_udc_hibernation_entry(const struct device *dev)
{
	ARG_UNUSED(dev);

	hal_usb_suspend_enter();

	return 0;
}

static int realtek_bee_udc_hibernation_exit(const struct device *dev)
{
	ARG_UNUSED(dev);

	usb_rtk_resume_sequence();

	return 0;
}

static int realtek_bee_udc_shutdown(const struct device *dev)
{
	ARG_UNUSED(dev);

	usb_rtk_disable_power_seq();

	return 0;
}

static int realtek_bee_udc_init_caps(const struct device *dev)
{
	struct udc_data *data = dev->data;

	data->caps.hs = true;
	data->caps.rwup = true;
	data->caps.mps0 = UDC_MPS0_64;

	return 0;
}

#define QUIRK_REALTEK_BEE_DEFINE(n)						\
	struct dwc2_vendor_quirks dwc2_vendor_quirks_##n = {			\
		.init = realtek_bee_udc_init,					\
		.shutdown = realtek_bee_udc_shutdown,				\
		.post_hibernation_entry = realtek_bee_udc_hibernation_entry,	\
		.pre_hibernation_exit = realtek_bee_udc_hibernation_exit,	\
		.caps = realtek_bee_udc_init_caps,				\
	};

DT_INST_FOREACH_STATUS_OKAY(QUIRK_REALTEK_BEE_DEFINE)

#endif /* ZEPHYR_DRIVERS_USB_UDC_DWC2_REALTEK_BEE_H */
