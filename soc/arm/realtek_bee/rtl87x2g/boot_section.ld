#ifdef CONFIG_LINKER_USE_BOOT_SECTION

	SECTION_PROLOGUE(boot.text,,)
	{
		lnkr_boot_start = .;
		z_mapped_start = .;

		lnkr_boot_text_start = .;

		KEEP(*(.boot_text.__start))
		*(.boot_text)
		*(.boot_text.*)

		*(.text.k_mem_paging_backing_store_init)
		*(.text.k_mem_paging_eviction_init)

		lnkr_boot_text_end = .;
	} GROUP_ROM_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)

	SECTION_PROLOGUE(boot.rodata,,)
	{

		lnkr_boot_rodata_start = .;

		*(.boot_rodata)
		*(.boot_rodata.*)

		lnkr_boot_rodata_end = .;
	} GROUP_ROM_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)

	SECTION_PROLOGUE(boot.data,,)
	{
		. = ALIGN(4);

		lnkr_boot_data_start = .;

		*(.boot_data)
		*(.boot_data.*)

		lnkr_boot_data_end = .;
	} GROUP_DATA_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)

	SECTION_PROLOGUE(boot.bss, (NOLOAD),)
	{
		. = ALIGN(4);

		lnkr_boot_bss_start = .;

		*(.boot_bss)
		*(.boot_bss.*)

		lnkr_boot_bss_end = .;
	} GROUP_NOLOAD_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)

	SECTION_PROLOGUE(boot.noinit, (NOLOAD),)
	{
		. = ALIGN(4);

		lnkr_boot_noinit_start = .;

		*(.boot_noinit)
		*(.boot_noinit.*)

		lnkr_boot_noinit_end = .;

		lnkr_boot_end = .;
	} GROUP_NOLOAD_LINK_IN(RAMABLE_REGION, ROMABLE_REGION)

	lnkr_boot_text_size = lnkr_boot_text_end - lnkr_boot_text_start;
	lnkr_boot_rodata_size = lnkr_boot_rodata_end - lnkr_boot_rodata_start;
	lnkr_boot_data_size = lnkr_boot_data_end - lnkr_boot_data_start;
	lnkr_boot_bss_size = lnkr_boot_bss_end - lnkr_boot_bss_start;
	lnkr_boot_noinit_size = lnkr_boot_noinit_end - lnkr_boot_noinit_start;

#endif /* CONFIG_LINKER_USE_BOOT_SECTION */