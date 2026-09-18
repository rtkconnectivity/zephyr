SECTION_PROLOGUE(_RTK_FLASH_CODE_SECTION,,)
{
    KEEP(*(.isr.text)) /* rtk internal lib would use this section. */
	. = ALIGN(4);
} GROUP_LINK_IN(ROMABLE_REGION)
