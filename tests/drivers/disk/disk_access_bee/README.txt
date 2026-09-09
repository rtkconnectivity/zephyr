/*
 * Copyright (c) 2026, Realtek Semiconductor Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

Disk Access Test (Realtek Bee)
##############################

This test verifies the functionality of disk devices on the Realtek Bee series.
It is derived from tests/drivers/disk/disk_access and keeps that test's generic
setup/read/write phases, adding Realtek-specific items for physical erase,
power cycling, FatFs formatting and an endless soak.

WARNING: every phase below is destructive. Do not run this test with a card
that holds useful data.

Always-enabled phases
*********************

* Setup (suite setup): Verifies that disk initialization works, as well as
  testing disk_access_ioctl by querying the disk for its sector size and
  sector count. Note that this phase also verifies the memory buffers reserved
  for read/write tests are sufficiently large, and will fail if they are not
  (in which case the value of SECTOR_SIZE must be increased).

  When both CONFIG_DISK_DRIVER_MMC and CONFIG_DISK_DRIVER_SDMMC are enabled,
  setup probes the inserted card and binds it to the matching disk. The MMC
  volume ("SD2") is probed first on purpose: the SDMMC driver leaves
  card->type unset, so sd_command_init() falls through to the MMC path on
  failure and the "SD" volume would also succeed on an eMMC - then drive it
  through the SD-only erase opcodes. Whichever volume initializes is used by
  all later phases.

* Read test: Verifies that the driver can consistently read sectors. This test
  starts by reading sectors from a variety of start locations. Each location is
  read from several times, each time with a different number of desired sectors.
  The test deliberately will read sectors beyond the end of the disk, and if
  the driver does not reject this read request the tests will fail. Following
  these sector reads, the driver will read multiple times from the same memory
  location, to verify that the data being returned is the same.

* Write test: Verifies that the driver can consistently write sectors. This test
  follows the same flow as the read test, but at each step writes data to the
  disk and reads it back to verify correctness. The test first performs writes
  of various length to various sectors (once again, the driver must reject
  writes that would be outside the bounds of the disk), then performs multiple
  writes to the same location.

Optional phases
***************

The Realtek-specific items are hardware-specific and destructive, so they are
off by default: a plain "west build" runs only the phases above. Enable them
individually through the test's Kconfig:

* CONFIG_DISK_ACCESS_TEST_ERASE adds test_erase and test_erase_full (also
  requires CONFIG_DISK_DRIVER_MMC). test_erase writes a 0xAA pattern, confirms
  it reached the card, calls disk_access_erase() with
  DISK_ACCESS_ERASE_PHYSICAL and times it, then checks the range reads back
  uniform and no longer holds the pattern - a timing-plus-content check that a
  physical erase really happened rather than a no-op. test_erase_full erases
  the entire card and spot-checks the first, middle and last sector.

* CONFIG_DISK_ACCESS_TEST_POWER_CYCLE adds test_power_cycle, which toggles the
  disk through DISK_IOCTL_CTRL_DEINIT / DISK_IOCTL_CTRL_INIT 100 times. Each
  iteration drives the refcount 1 -> 0 (real power-down) and back 0 -> 1 (real
  power-up), asserting DISK_STATUS_UNINIT / DISK_STATUS_OK in between, and the
  loop is balanced so the card is left initialized and readable.

* CONFIG_DISK_ACCESS_TEST_FS_FORMAT adds test_y_fs_format, which formats the
  whole card with fs_mkfs(), mounts it, writes and reads back a file, then
  runs an SLC-cache experiment: it writes a 1 GB file logging the write rate
  every 64 MB (so the SLC-cache knee becomes visible as a drop), deletes it,
  and writes a second identical file. Zephyr's FatFs glue
  (modules/fatfs/zfs_diskio.c) does not implement CTRL_TRIM, so the delete
  only frees FAT clusters and never reaches the card; the second pass is
  therefore expected to start slow, and starting fast again would mean the card
  replenished the budget on its own. Writes ~2 GB and takes minutes. Before
  returning, the test forces a full deinit/init to undo the refcount leak left
  by f_mkfs() + mount vs. a single fs_unmount().

* CONFIG_DISK_ACCESS_STRESS adds test_z_stress_rw, a non-terminating
  read/write/verify soak that sweeps the whole card in 31-sector windows,
  wraps at the end and repeats forever. It logs errors instead of aborting, so
  the driver's error-recovery path keeps being exercised, and reports
  cumulative bytes plus the windowed write/read rate every 100 iterations.
  Every 10000 iterations it physically erases the region written by the
  previous 10000 so the card's fast-write budget can be replenished.

The "test_y_" / "test_z_" prefixes matter: ztest sorts by name, so the
destructive full-card format runs after the functional phases and the endless
soak runs last, where its infinite loop cannot starve anything.

Hardware notes
**************

* boards/rtl87x2g_evb_a_rtl8762gku.overlay enables sdhc0 at 50 MHz / 4-bit
  with both a zephyr,sdmmc-disk ("SD") and a zephyr,mmc-disk ("SD2") child, so
  the same binary handles an SD card or an eMMC.

* prj.conf raises CONFIG_SD_CMD_TIMEOUT to 30 s. A full-card erase (CMD38)
  holds the card busy on DAT0 for far longer than the 200 ms default, and the
  resulting response timeout would poison every subsequent command.

* On CONFIG_SOC_SERIES_RTL87X2G the scratch buffer rows are padded and
  32-byte aligned to satisfy CONFIG_SDHC_BUFFER_ALIGNMENT. Unaligned buffers
  make the SD stack bounce through the 512-byte per-card buffer one block at a
  time (CMD24/CMD17) instead of issuing real multi-block CMD25/CMD18.
