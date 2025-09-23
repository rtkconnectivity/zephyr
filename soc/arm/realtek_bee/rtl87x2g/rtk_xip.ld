SECTION_PROLOGUE(_RTK_FLASH_CODE_SECTION,,)
{
    *(.isr.text) /* rtk internal lib would use this section. */
    *(.app.flash.text)
	. = ALIGN(4);
} GROUP_LINK_IN(ROMABLE_REGION)
