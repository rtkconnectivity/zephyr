#include <patch_header_check.h>
#include <stdlib.h>
#include <rom_uuid.h>
extern void z_arm_reset(void);

const T_IMG_HEADER_FORMAT img_header __attribute__((section(".image_header"))) =
{
    .auth =
    {
        .image_mac = {[0 ... 15] = 0xFF},
    },
    .ctrl_header =
    {
        .ic_type = 0xF,
        .secure_version = 0,
        .ctrl_flag.xip = 1,
        .ctrl_flag.enc = 0,
        .ctrl_flag.load_when_boot = 0,
        .ctrl_flag.enc_load = 0,
        .ctrl_flag.not_obsolete = 1,
        .image_id = IMG_MCUAPP,
        .payload_len = 0x100,
    },
    .uuid = DEFINE_symboltable_uuid,
    .exe_base = (unsigned int)z_arm_reset,
};