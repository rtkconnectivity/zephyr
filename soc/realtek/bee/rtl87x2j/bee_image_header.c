/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <image_check.h>
#include <rom_uuid.h>
#include <version.h>

extern void z_arm_reset(void);

const T_IMG_HEADER_FORMAT img_header __attribute__((section(".image_header"))) = {
	.auth = {
		.img_string = "IMGHDR",
		.auth_length = 0,
		.auth_type = 0,
		.cmac = {[0 ... 15] = 0xFF},
	},
	.ctrl_header = {
		.ic_type = 18,
		.secure_version = 0,
		.ctrl_flag.load_when_boot = 1,
		.ctrl_flag.integrity_check_en_in_boot = 0,
		.image_id = IMG_MCUAPP,
		.header_len = IMG_HEADER_SIZE,
		.payload_len = 0x100,
	},
	.uuid = DEFINE_symboltable_uuid,
	.magic_pattern = FLASH_TABLE_MAGIC_PATTERN,
	.git_ver = {
		.sub_version._version_major = KERNEL_VERSION_MAJOR,
		.sub_version._version_minor = KERNEL_VERSION_MINOR,
		.sub_version._version_revision = KERNEL_PATCHLEVEL,
	},
	.exe_entry = (unsigned int)z_arm_reset,
	.image_base = CONFIG_FLASH_BASE_ADDRESS + CONFIG_FLASH_LOAD_OFFSET,
	.ram_load_src = 0,
	.ram_load_len = 0,
	.ram_load_dst = 0,
};
