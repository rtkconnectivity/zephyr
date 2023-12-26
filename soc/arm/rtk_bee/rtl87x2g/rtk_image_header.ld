#if defined(CONFIG_RTK_IMAGE_HEADER)
KEEP(*(.image_header))
KEEP(*(.".image_header.*"))
__end = .;
#endif
