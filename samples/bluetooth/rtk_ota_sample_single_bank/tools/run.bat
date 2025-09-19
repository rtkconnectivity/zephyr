del "zephyr*.bin"
cp ../build/zephyr/zephyr.bin .
@REM Prepend_header: 1. Calculate SHA256/crc by default.
@REM                 2. Calculate signature/flash-sec by option.
@REM                 3. Add mp header by default.
.\prepend_header\prepend_header.exe -t app_code  -b 15 -p "./zephyr.bin" -m 1 -i "./mp.ini"
1>NUL ren zephyr.bin zephyrImage.bin
1>NUL ren zephyr_MP.bin zephyrImage_MP.bin
@REM MD5: Calculate MD5 and add it in the image file name.
.\md5\md5.exe "zephyrImage_MP.bin"