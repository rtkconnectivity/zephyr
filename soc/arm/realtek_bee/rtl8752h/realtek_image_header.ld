#if defined(CONFIG_REALTEK_IMAGE_HEADER)
KEEP(*(.image_header))
KEEP(*(.".image_header.*"))
__end = .;
#endif
