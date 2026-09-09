/*
 * Copyright (c) 2021 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * WARNING: This test will overwrite data on any disk utilized. Do not run
 * this test with an disk that has useful data
 */

#include <zephyr/kernel.h>
#include <zephyr/ztest.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/device.h>

#ifdef CONFIG_DISK_DRIVER_LOOPBACK
#include <ff.h>
#include <zephyr/fs/fs.h>
#include <zephyr/drivers/loopback_disk.h>
#endif

#ifdef CONFIG_DISK_ACCESS_TEST_FS_FORMAT
#include <ff.h>
#include <zephyr/fs/fs.h>
#endif

#if defined(CONFIG_DISK_DRIVER_SDMMC)
#define DISK_NAME_PHYS "SD"
#elif defined(CONFIG_DISK_DRIVER_MMC)
#define DISK_NAME_PHYS "SD2"
#elif defined(CONFIG_DISK_DRIVER_FLASH) || defined(CONFIG_DISK_DRIVER_FTL)
#define DISK_NAME_PHYS "NAND"
#elif defined(CONFIG_NVME)
#define DISK_NAME_PHYS "nvme0n0"
#elif defined(CONFIG_DISK_DRIVER_RAM)
/* Since ramdisk is enabled by default on e.g. qemu boards, it needs to be checked last to not
 * override other backends.
 */
#define DISK_NAME_PHYS "RAM"
#else
#error "No disk device defined, is your board supported?"
#endif

#ifdef CONFIG_DISK_DRIVER_LOOPBACK
#define DISK_NAME "loopback0"
#else
#define DISK_NAME DISK_NAME_PHYS
#endif

/* Assume the largest sector we will encounter is 512 bytes */
#define SECTOR_SIZE 512

/* Sector counts to read */
#define SECTOR_COUNT1 8
#define SECTOR_COUNT2 1
#define SECTOR_COUNT3 29
#define SECTOR_COUNT4 31

#define OVERFLOW_CANARY 0xDE

static const char *disk_pdrv = DISK_NAME;
static uint32_t disk_sector_count;
static uint32_t disk_sector_size;

#if defined(CONFIG_SOC_SERIES_RTL87X2G)
/* Row length padded to a 32-byte cache line and the whole buffer 32-byte
 * aligned so both rows satisfy CONFIG_SDHC_BUFFER_ALIGNMENT (and are DMA
 * cache-line aligned). Unaligned buffers make the SD stack bounce through the
 * 512-byte per-card buffer one block at a time (CMD24/CMD17); aligned buffers
 * let it issue real multi-block CMD25/CMD18. +4 keeps room past the last sector
 * for the overflow canary.
 */
#define SCRATCH_ROW_LEN ROUND_UP(SECTOR_COUNT4 * SECTOR_SIZE + 4, 32)
static uint8_t scratch_buf[2][SCRATCH_ROW_LEN] __aligned(32);
#else
/* + 4 to make sure the second buffer is dword-aligned for NVME */
static uint8_t scratch_buf[2][SECTOR_COUNT4 * SECTOR_SIZE + 4];
#endif

#ifdef CONFIG_DISK_DRIVER_LOOPBACK
#define BACKING_PATH "/"DISK_NAME_PHYS":"

static struct loopback_disk_access lo_access;
static FATFS fat_fs;
static struct fs_mount_t backing_mount = {
	.type = FS_FATFS,
	.mnt_point = BACKING_PATH,
	.fs_data = &fat_fs,
};
static const uint8_t zero_kb[1024] = {};
static void setup_loopback_backing(void)
{
	int rc;

	rc = fs_mkfs(FS_FATFS, (uintptr_t)&BACKING_PATH[1], NULL, 0);
	zassert_equal(rc, 0, "Failed to format backing file system");

	rc = fs_mount(&backing_mount);
	zassert_equal(rc, 0, "Failed to mount backing file system");

	struct fs_file_t f;

	fs_file_t_init(&f);
	rc = fs_open(&f, BACKING_PATH "/loopback.img", FS_O_WRITE | FS_O_CREATE);
	zassert_equal(rc, 0, "Failed to create backing file");
	for (int i = 0; i < 64; i++) {
		rc = fs_write(&f, zero_kb, sizeof(zero_kb));
		zassert_equal(rc, sizeof(zero_kb), "Failed to enlarge backing file");
	}
	rc = fs_close(&f);
	zassert_equal(rc, 0, "Failed to close backing file");

	rc = loopback_disk_access_register(&lo_access, BACKING_PATH "/loopback.img", DISK_NAME);
	zassert_equal(rc, 0, "Loopback disk access initialization failed");
}
#endif

/* Sets up test by initializing disk */
static void test_setup(void)
{
	int rc;
	uint32_t cmd_buf;

#if defined(CONFIG_DISK_DRIVER_MMC) && defined(CONFIG_DISK_DRIVER_SDMMC)
	/*
	 * Both disk drivers are enabled; detect the inserted card and bind it to
	 * the matching disk. Order matters: the MMC volume is probed FIRST.
	 *
	 * The MMC driver (mmc_subsys.c) pins card->type = CARD_MMC before
	 * sd_init(), so sd_command_init() jumps straight to the MMC path and an
	 * SD card - which never answers CMD1 - fails it cleanly, letting the
	 * loop fall through to the SDMMC volume. The SDMMC driver, by contrast,
	 * leaves the type unset: sd_command_init() tries the SDMMC path and, on
	 * failure, FALLS THROUGH to the MMC path - so the "SD" volume would also
	 * succeed on an eMMC and then drive it through the SD-only erase opcodes
	 * (CMD32/33). Probing MMC first avoids that mis-binding.
	 *
	 * Whichever volume initializes is kept in disk_pdrv for all later tests.
	 */
	static const char *const disk_candidates[] = {
		"SD2",
		"SD",
	};

	disk_pdrv = NULL;
	for (int i = 0; i < ARRAY_SIZE(disk_candidates); i++) {
		rc = disk_access_init(disk_candidates[i]);
		if (rc == 0) {
			disk_pdrv = disk_candidates[i];
			TC_PRINT("Detected disk via \"%s\"\n", disk_pdrv);
			break;
		}
		TC_PRINT("disk \"%s\" init failed (rc=%d), trying next\n",
			 disk_candidates[i], rc);
	}
	zassert_not_null(disk_pdrv, "No matching disk found for inserted card");
#else
	rc = disk_access_init(disk_pdrv);
	zassert_equal(rc, 0, "Disk access initialization failed");
#endif

	rc = disk_access_status(disk_pdrv);
	zassert_equal(rc, DISK_STATUS_OK, "Disk status is not OK");

	rc = disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_COUNT, &cmd_buf);
	zassert_equal(rc, 0, "Disk ioctl get sector count failed");

	disk_sector_count = cmd_buf;

	rc = disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_SIZE, &cmd_buf);
	zassert_equal(rc, 0, "Disk ioctl get sector size failed");
	disk_sector_size = cmd_buf;
	TC_PRINT("Disk: %u sectors x %u bytes\n", disk_sector_count, disk_sector_size);

	/* We could allocate memory once we know the sector size, but instead
	 * just verify our assumed maximum size
	 */
	zassert_true(cmd_buf <= SECTOR_SIZE,
		"Test will fail, SECTOR_SIZE definition must be increased");
}

/* Reads sectors, verifying overflow does not occur */
static int read_sector(uint8_t *buf, uint32_t start, uint32_t num_sectors)
{
	int rc;

	/* Set up overflow canary */
	buf[num_sectors * disk_sector_size] = OVERFLOW_CANARY;
	rc = disk_access_read(disk_pdrv, buf, start, num_sectors);
	/* Check canary */
	zassert_equal(buf[num_sectors * disk_sector_size], OVERFLOW_CANARY,
		"Read overflowed requested length");
	return rc; /* Let calling function check return code */
}

/* Tests reading from a variety of sectors */
static void test_sector_read(uint8_t *buf, uint32_t num_sectors)
{
	int rc, sector;

	TC_PRINT("Testing reads of %u sectors\n", num_sectors);
	/* Read from disk sector 0*/
	rc = read_sector(buf, 0, num_sectors);
	zassert_equal(rc, 0, "Failed to read from sector zero");
	/* Read from a sector in the "middle" of the disk */
	if (disk_sector_count / 2 > num_sectors) {
		sector = disk_sector_count / 2 - num_sectors;
	} else {
		sector = 0;
	}

	rc = read_sector(buf, sector, num_sectors);
	zassert_equal(rc, 0, "Failed to read from mid disk sector");
	/* Read from the last sector */
	rc = read_sector(buf, disk_sector_count - 1, num_sectors);
	if (num_sectors == 1) {
		zassert_equal(rc, 0, "Failed to read from last sector");
	} else {
		zassert_not_equal(rc, 0, "Disk should fail to read out of sector bounds");
	}
}

/* Write sector of disk, and check the data to ensure it is valid
 * WARNING: this test is destructive- it will overwrite data on the disk!
 */
static int write_sector_checked(uint8_t *wbuf, uint8_t *rbuf,
			uint32_t start, uint32_t num_sectors)
{
	int rc, i;

	/* First, fill the write buffer with data */
	for (i = 0; i < num_sectors * disk_sector_size; i++) {
		wbuf[i] = (i & (~num_sectors));
	}
	/* Now write data to the sector */
	rc = disk_access_write(disk_pdrv, wbuf, start, num_sectors);
	if (rc) {
		return rc; /* Let calling function handle disk error */
	}
	/* Read back the written data into another buffer */
	memset(rbuf, 0, num_sectors * disk_sector_size);
	rc = read_sector(rbuf, start, num_sectors);
	if (rc) {
		return rc;
	}
	/* Check the read data versus the written data */
	zassert_mem_equal(wbuf, rbuf, num_sectors * disk_sector_size,
		"Read data did not match data written to disk");
	return rc;
}

/* Tests writing to a variety of sectors
 * WARNING: this test is destructive- it will overwrite data on the disk!
 */
static void test_sector_write(uint8_t *wbuf, uint8_t *rbuf, uint32_t num_sectors)
{
	int rc, sector;

	TC_PRINT("Testing writes of %u sectors\n", num_sectors);
	/* Write to disk sector zero */
	rc = write_sector_checked(wbuf, rbuf, 0, num_sectors);
	zassert_equal(rc, 0, "Failed to write to sector zero");
	/* Write to a sector in the "middle" of the disk */
	if (disk_sector_count / 2 > num_sectors) {
		sector = disk_sector_count / 2 - num_sectors;
	} else {
		sector = 0;
	}

	rc = write_sector_checked(wbuf, rbuf, sector, num_sectors);
	zassert_equal(rc, 0, "Failed to write to mid disk sector");
	/* Write to the last sector */
	rc = write_sector_checked(wbuf, rbuf, disk_sector_count - 1, num_sectors);
	if (num_sectors == 1) {
		zassert_equal(rc, 0, "Failed to write to last sector");
	} else {
		zassert_not_equal(rc, 0, "Disk should fail to write out of sector bounds");
	}
}

/* Test multiple reads in series, and reading from a variety of blocks */
ZTEST(disk_driver, test_read)
{
	int rc, i;

	/* Verify all 4 read sizes work */
	test_sector_read(scratch_buf[0], SECTOR_COUNT1);
	test_sector_read(scratch_buf[0], SECTOR_COUNT2);
	test_sector_read(scratch_buf[0], SECTOR_COUNT3);
	test_sector_read(scratch_buf[0], SECTOR_COUNT4);

	/* Verify that reading from the same location returns to same data */
	memset(scratch_buf[0], 0, SECTOR_COUNT1 * disk_sector_size);
	rc = read_sector(scratch_buf[0], 0, SECTOR_COUNT1);
	zassert_equal(rc, 0, "Failed to read from disk");
	for (i = 0; i < 10; i++) {
		/* Read from sector, and compare it to the first read */
		memset(scratch_buf[1], 0xff, SECTOR_COUNT1 * disk_sector_size);
		rc = read_sector(scratch_buf[1], 0, SECTOR_COUNT1);
		zassert_equal(rc, 0, "Failed to read from disk at same sector location");
		zassert_mem_equal(scratch_buf[1], scratch_buf[0],
				SECTOR_COUNT1 * disk_sector_size,
				"Multiple reads mismatch");
	}
}

/* test writing data, and then verifying it was written correctly.
 * WARNING: this test is destructive- it will overwrite data on the disk!
 */
ZTEST(disk_driver, test_write)
{
	int rc, i;

	/* Verify all 4 sector write sizes work */
	test_sector_write(scratch_buf[0], scratch_buf[1], SECTOR_COUNT1);
	test_sector_write(scratch_buf[0], scratch_buf[1], SECTOR_COUNT2);
	test_sector_write(scratch_buf[0], scratch_buf[1], SECTOR_COUNT3);
	test_sector_write(scratch_buf[0], scratch_buf[1], SECTOR_COUNT4);

	/* Verify that multiple writes to the same location work */
	for (i = 0; i < 10; i++) {
		/* Write to sector- helper function verifies written data is correct */
		rc = write_sector_checked(scratch_buf[0], scratch_buf[1], 0, SECTOR_COUNT1);
		zassert_equal(rc, 0, "Failed to write to disk at same sector location");
	}
}

#if defined(CONFIG_DISK_DRIVER_MMC) && defined(CONFIG_DISK_ACCESS_TEST_ERASE)
/* Verify a physical erase happened: write a pattern, erase, read back. A real
 * erase leaves sectors uniform (0x00 or 0xFF) and no longer holding the pattern.
 * WARNING: destructive.
 */
ZTEST(disk_driver, test_erase)
{
	const uint32_t start = 0;
	const uint32_t count = SECTOR_COUNT4;
	const uint32_t bytes = count * disk_sector_size;
	uint32_t t0, us;
	uint8_t v;
	int rc;
	/* 1. Write 0xAA and confirm it is really on the card. */
	memset(scratch_buf[0], 0xAA, bytes);
	rc = disk_access_write(disk_pdrv, scratch_buf[0], start, count);
	zassert_equal(rc, 0, "write failed");
	memset(scratch_buf[1], 0, bytes);
	rc = disk_access_read(disk_pdrv, scratch_buf[1], start, count);
	zassert_equal(rc, 0, "read-back of pattern failed");
	zassert_mem_equal(scratch_buf[1], scratch_buf[0], bytes, "pattern not written");

	/* 2. Erase the range and time it (a real erase is not instant). */
	t0 = k_cycle_get_32();
	rc = disk_access_erase(disk_pdrv, start, count, DISK_ACCESS_ERASE_PHYSICAL);
	us = k_cyc_to_us_floor32(k_cycle_get_32() - t0);
	zassert_equal(rc, 0, "erase failed (rc=%d)", rc);
	TC_PRINT("erase [%u..%u] took %u us\n", start, start + count - 1, us);

	/* 3. Read back: must be uniform and no longer the 0xAA pattern. */
	memset(scratch_buf[1], 0xAA, bytes);
	rc = disk_access_read(disk_pdrv, scratch_buf[1], start, count);
	zassert_equal(rc, 0, "read-after-erase failed");
	v = scratch_buf[1][0];
	zassert_not_equal(v, 0xAA, "data unchanged - erase had no effect");
	zassert_true(v == 0x00 || v == 0xFF, "erased data not 0x00/0xFF (0x%02x)", v);
	for (uint32_t i = 0; i < bytes; i++) {
		zassert_equal(scratch_buf[1][i], v, "erase not uniform at byte %u", i);
	}
}

/* Erase the ENTIRE card (sector 0 .. last), then spot-check a few sectors read
 * back as the erased value. Destroys everything on the card; resets the FTL.
 */
ZTEST(disk_driver, test_erase_full)
{
	uint32_t probes[3];
	uint32_t t0, ms;
	uint8_t v;
	int rc;

	TC_PRINT("full erase of %u sectors (%u MB)...\n", disk_sector_count,
		 (uint32_t)(((uint64_t)disk_sector_count * disk_sector_size) >> 20));

	t0 = k_cycle_get_32();
	rc = disk_access_erase(disk_pdrv, 0, disk_sector_count, DISK_ACCESS_ERASE_PHYSICAL);
	ms = k_cyc_to_ms_floor32(k_cycle_get_32() - t0);
	zassert_equal(rc, 0, "full erase failed (rc=%d)", rc);
	TC_PRINT("full erase took %u ms\n", ms);

	/* Spot-check first, middle and last sector. */
	probes[0] = 0;
	probes[1] = disk_sector_count / 2;
	probes[2] = disk_sector_count - 1;
	for (uint32_t p = 0; p < ARRAY_SIZE(probes); p++) {
		rc = disk_access_read(disk_pdrv, scratch_buf[0], probes[p], 1);
		zassert_equal(rc, 0, "read after full erase failed at %u", probes[p]);
		v = scratch_buf[0][0];
		zassert_true(v == 0x00 || v == 0xFF,
			     "sector %u not erased (0x%02x)", probes[p], v);
		for (uint32_t i = 0; i < disk_sector_size; i++) {
			zassert_equal(scratch_buf[0][i], v,
				      "sector %u not uniform at byte %u", probes[p], i);
		}
	}
}
#endif /* CONFIG_DISK_DRIVER_MMC && CONFIG_DISK_ACCESS_TEST_ERASE */

#ifdef CONFIG_DISK_ACCESS_TEST_POWER_CYCLE
/* Power-cycle the eMMC via CTRL_DEINIT/CTRL_INIT. Each iteration DEINITs
 * (refcount 1->0, real power-down) then INITs (0->1, real power-up); the loop is
 * balanced so it ends with refcount 1 and the card initialized.
 */
#define POWER_CYCLE_COUNT 100
ZTEST(disk_driver, test_power_cycle)
{
	int rc, i;

	zassert_equal(disk_access_status(disk_pdrv), DISK_STATUS_OK,
		      "disk not initialized at start (refcount != 1?)");

	for (i = 0; i < POWER_CYCLE_COUNT; i++) {
		/* Power down: refcount 1 -> 0 triggers the real deinit. */
		rc = disk_access_ioctl(disk_pdrv, DISK_IOCTL_CTRL_DEINIT, NULL);
		zassert_equal(rc, 0, "cycle %d: deinit failed (rc=%d)", i, rc);
		zassert_equal(disk_access_status(disk_pdrv), DISK_STATUS_UNINIT,
			      "cycle %d: disk still initialized after deinit", i);

		/* Let the power rail settle before bringing the card back up. */
		k_msleep(10);

		/* Power up: refcount 0 -> 1 triggers the real init. */
		rc = disk_access_ioctl(disk_pdrv, DISK_IOCTL_CTRL_INIT, NULL);
		zassert_equal(rc, 0, "cycle %d: init failed (rc=%d)", i, rc);
		zassert_equal(disk_access_status(disk_pdrv), DISK_STATUS_OK,
			      "cycle %d: disk not OK after init", i);

		if ((i % 10) == 9) {
			TC_PRINT("power cycle %d/%d ok\n", i + 1, POWER_CYCLE_COUNT);
		}
	}

	/* Sanity: the card must still be readable after all the power cycling. */
	rc = disk_access_read(disk_pdrv, scratch_buf[0], 0, 1);
	zassert_equal(rc, 0, "read failed after %d power cycles", POWER_CYCLE_COUNT);
}
#endif /* CONFIG_DISK_ACCESS_TEST_POWER_CYCLE */

#ifdef CONFIG_DISK_ACCESS_TEST_FS_FORMAT
/* Format the eMMC with FatFs, mount it, write a file, read it back and verify.
 *
 * The RTL87X3G fatfs_mkfs() re-applies translate_path() to dev_id, so it needs
 * the leading '/' too - hence fs_fmt_mnt ("/<volume>:") is used for both
 * mnt_point and the mkfs dev_id.
 * The "test_y_" prefix sorts it (ztest SORT_BY_NAME) after the functional tests
 * but before test_z_stress_rw, so the destructive full-card mkfs runs late.
 * WARNING: destructive, wipes the whole card.
 */
/* This test selects the volume purely through the fs layer: it tries to
 * fs_mkfs() the MMC volume first, then the SDMMC volume. fs_mkfs() internally
 * drives disk init (CTRL_INIT -> disk_{mmc,sdmmc}_access_init, which rejects a
 * card that is not its own type with -ENOTSUP), so the format fails on the
 * wrong-type volume and succeeds on the one matching the inserted card - no
 * disk_access_init() call is needed here. A rejected CTRL_INIT does not bump
 * the disk refcount, so the failed attempt leaves no residual state.
 * fs_fmt_mnt ("/<volume>:") is built at runtime from whichever volume wins.
 */
#define FS_FMT_FILE_NAME "/fmt_tst.bin"

/* SLC-cache slowdown / TRIM-recovery experiment parameters (see test body). */
#define FSX_FILE_A_NAME "/big_a.bin"
#define FSX_FILE_B_NAME "/big_b.bin"
#define FSX_CHUNK   (SECTOR_COUNT4 * SECTOR_SIZE)   /* ~15.5 KB reused buffer */
#define FSX_TARGET  (1024ULL * 1024 * 1024)         /* 1024 MB written per pass */
#define FSX_REPORT  (64ULL * 1024 * 1024)           /* log window: every 64 MB */

static FATFS fs_fmt_fatfs;
static char fs_fmt_mnt[16];   /* "/<volume>:" of the fs-detected volume */
static struct fs_mount_t fs_fmt_mp = {
	.type = FS_FATFS,
	.fs_data = &fs_fmt_fatfs,
	.mnt_point = fs_fmt_mnt,
	.flags = FS_MOUNT_FLAG_USE_DISK_ACCESS,
};

ZTEST(disk_driver, test_y_fs_format)
{
	static const char payload[] = "zephyr fatfs format test payload";
	char readback[sizeof(payload)];
	char fmt_file[sizeof(fs_fmt_mnt) + sizeof(FS_FMT_FILE_NAME)];
	char file_a[sizeof(fs_fmt_mnt) + sizeof(FSX_FILE_A_NAME)];
	char file_b[sizeof(fs_fmt_mnt) + sizeof(FSX_FILE_B_NAME)];
	/* Format the volume already detected in setup (disk_pdrv). Re-probing here
	 * with a second candidate would send the wrong-type init to the shared
	 * controller (e.g. an MMC CMD1 to an SD card) and wedge it; reuse the
	 * known-good volume instead. The FatFs volume string equals the
	 * disk-access name, so disk_pdrv doubles as the FatFs volume id.
	 */
	const char *vol = disk_pdrv;
	struct fs_file_t file;
	ssize_t n;
	int rc;

	snprintk(fs_fmt_mnt, sizeof(fs_fmt_mnt), "/%s:", vol);
	/* Format the whole card as FAT (default params: auto type/cluster).
	 * fs_fmt_mnt is "/<vol>:" for the later fs_mount(); f_mkfs() wants the
	 * bare FatFs volume id "<vol>:", so skip the leading '/'.
	 */
	rc = fs_mkfs(FS_FATFS, (uintptr_t)(fs_fmt_mnt + 1), NULL, 0);
	zassert_equal(rc, 0, "fs_mkfs on \"%s\" failed (rc=%d)", fs_fmt_mnt, rc);
	TC_PRINT("fs: formatted via volume \"%s\"\n", vol);

	/* fs_fmt_mnt now holds "/<vol>:"; build the file paths and storage_dev. */
	fs_fmt_mp.storage_dev = (void *)vol;
	snprintk(fmt_file, sizeof(fmt_file), "%s" FS_FMT_FILE_NAME, fs_fmt_mnt);
	snprintk(file_a, sizeof(file_a), "%s" FSX_FILE_A_NAME, fs_fmt_mnt);
	snprintk(file_b, sizeof(file_b), "%s" FSX_FILE_B_NAME, fs_fmt_mnt);

	/* Mount the freshly formatted volume. */
	rc = fs_mount(&fs_fmt_mp);
	zassert_equal(rc, 0, "fs_mount failed (rc=%d)", rc);

	/* Write a file. */
	fs_file_t_init(&file);
	rc = fs_open(&file, fmt_file, FS_O_CREATE | FS_O_WRITE);
	zassert_equal(rc, 0, "fs_open (write) failed (rc=%d)", rc);
	n = fs_write(&file, payload, sizeof(payload));
	zassert_equal(n, (ssize_t)sizeof(payload), "fs_write short (%d)", (int)n);
	zassert_equal(fs_close(&file), 0, "fs_close (write) failed");

	/* Read it back and verify byte-for-byte. */
	fs_file_t_init(&file);
	rc = fs_open(&file, fmt_file, FS_O_READ);
	zassert_equal(rc, 0, "fs_open (read) failed (rc=%d)", rc);
	memset(readback, 0, sizeof(readback));
	n = fs_read(&file, readback, sizeof(readback));
	zassert_equal(n, (ssize_t)sizeof(payload), "fs_read short (%d)", (int)n);
	zassert_equal(fs_close(&file), 0, "fs_close (read) failed");
	zassert_mem_equal(readback, payload, sizeof(payload),
			  "readback mismatch after format + write");

	/* --- SLC-cache slowdown / TRIM-recovery experiment ----------------------
	 * Pass 0: write a ~900 MB file with a small reused buffer (no large RAM
	 *         needed) and log the per-64 MB write rate so the SLC-cache knee
	 *         becomes visible (fast until the ~0.9 GB budget, then a drop).
	 * Then delete that file: with CONFIG_FS_FATFS_USE_TRIM=y this issues
	 *         CTRL_TRIM -> DISK_IOCTL_ERASE_SECTORS -> mmc_erase, freeing the
	 *         blocks and refilling the fast SLC budget.
	 * Pass 1: write a second identical file. If it starts fast again, the TRIM
	 *         on delete recovered the speed; if it starts slow, it did not
	 *         (e.g. CONFIG_FS_FATFS_USE_TRIM disabled -> delete alone useless).
	 * WARNING: writes ~1.8 GB total, takes minutes.
	 */
	for (int pass = 0; pass < 2; pass++) {
		const char *path = (pass == 0) ? file_a : file_b;
		uint64_t written = 0;
		uint64_t win_bytes = 0, win_us = 0, next_report = FSX_REPORT;
		uint32_t seq = 0;

		TC_PRINT("fs-trim pass %d: writing %llu MB to %s\n",
			 pass, (unsigned long long)(FSX_TARGET >> 20), path);

		fs_file_t_init(&file);
		rc = fs_open(&file, path, FS_O_CREATE | FS_O_WRITE);
		zassert_equal(rc, 0, "fs_open big (write) failed (rc=%d)", rc);

		while (written < FSX_TARGET) {
			uint32_t t0;

			/* Vary content per chunk (avoid all-same, which some cards
			 * special-case); only a small buffer is ever in RAM.
			 */
			for (int b = 0; b < FSX_CHUNK; b++) {
				scratch_buf[0][b] = (uint8_t)(b + seq);
			}
			seq++;

			t0 = k_cycle_get_32();
			n = fs_write(&file, scratch_buf[0], FSX_CHUNK);
			win_us += k_cyc_to_us_floor64(k_cycle_get_32() - t0);
			if (n != (ssize_t)FSX_CHUNK) {
				TC_PRINT("fs-trim pass %d: fs_write short n=%d at %llu MB\n",
					 pass, (int)n, (unsigned long long)(written >> 20));
				break;
			}
			written += FSX_CHUNK;
			win_bytes += FSX_CHUNK;

			if (written >= next_report) {
				TC_PRINT("fs-trim pass %d: written %llu MB, "
					 "write %llu KB/s\n",
					 pass, (unsigned long long)(written >> 20),
					 (unsigned long long)(win_us ?
						win_bytes * 1000000ULL / win_us / 1024ULL : 0));
				win_bytes = 0;
				win_us = 0;
				next_report += FSX_REPORT;
			}
		}

		fs_sync(&file);
		zassert_equal(fs_close(&file), 0, "fs_close big failed");

		if (pass == 0) {
			/* Delete file A -> CTRL_TRIM path. Time it: on a card that
			 * really erases, unlink of a big file is not instantaneous.
			 */
			uint32_t t0 = k_cycle_get_32();
			uint64_t unlink_ms;

			rc = fs_unlink(file_a);
			unlink_ms = k_cyc_to_us_floor64(k_cycle_get_32() - t0) / 1000ULL;
			zassert_equal(rc, 0, "fs_unlink big_a failed (rc=%d)", rc);
			TC_PRINT("fs-trim: unlinked big_a in %llu ms (FS TRIM %s)\n",
				 (unsigned long long)unlink_ms,
				 IS_ENABLED(CONFIG_FS_FATFS_USE_TRIM) ?
					"enabled" : "DISABLED");
		}
	}

	/* Clean up file B before unmount. */
	(void)fs_unlink(file_b);

	/* f_mkfs() and the mount each issue a CTRL_INIT but fs_unmount() issues only
	 * one CTRL_DEINIT, so the refcount leaks to 2. Force a full deinit (refcount
	 * -> 0, real power-down) then a single INIT to restore the exact refcount 1 /
	 * DISK_STATUS_OK invariant for the tests that follow.
	 */
	rc = fs_unmount(&fs_fmt_mp);
	zassert_equal(rc, 0, "fs_unmount failed (rc=%d)", rc);

	bool force_deinit = true;

	(void)disk_access_ioctl(vol, DISK_IOCTL_CTRL_DEINIT, &force_deinit);
	rc = disk_access_ioctl(vol, DISK_IOCTL_CTRL_INIT, NULL);
	zassert_equal(rc, 0, "re-init after fs test failed (rc=%d)", rc);
	zassert_equal(disk_access_status(vol), DISK_STATUS_OK,
		      "disk not restored to OK after fs test");

}
#endif /* CONFIG_DISK_ACCESS_TEST_FS_FORMAT */

#ifdef CONFIG_DISK_ACCESS_STRESS
/* Endless read/write/verify soak: sweeps the whole card in SECTOR_COUNT4-sized
 * windows, wraps at the end and repeats forever, logging (never aborting) on
 * error so error recovery is exercised continuously. The "test_z_" prefix sorts
 * it last (ztest SORT_BY_NAME) so its infinite loop can't starve other tests.
 * WARNING: destructive - overwrites the entire disk!
 */
ZTEST(disk_driver, test_z_stress_rw)
{
	uint32_t iter = 0;
	uint32_t sweep = 0;
	uint32_t err_count = 0;
	uint32_t start = 0;
	/* Sector where the current 10000-iteration write window began; the region
	 * [erase_from, start) is trimmed every 10000 iterations (see below).
	 */
	uint32_t erase_from = start;
	int rc, i;
	/* Cumulative bytes successfully transferred (64-bit, won't wrap). */
	uint64_t total_bytes = 0;
	/* Per-100-iteration window accumulators (reset after each report) for the
	 * average write/read rate; only time inside disk_access_write()/read().
	 */
	uint64_t win_wr_bytes = 0, win_rd_bytes = 0;
	uint64_t win_wr_us = 0, win_rd_us = 0;
	uint32_t t0;

	TC_PRINT("Starting endless full-card read/write stress (non-fatal on error)\n");

	while (1) {
		/* Number of sectors in this window: a full SECTOR_COUNT4 unless
		 * we are at a non-aligned tail near the end of the card.
		 */
		uint32_t n = SECTOR_COUNT4;

		if (start + n > disk_sector_count) {
			n = disk_sector_count - start;
		}

		iter++;

		for (i = 0; i < n * disk_sector_size; i++) {
			scratch_buf[0][i] = (uint8_t)(i + iter);
		}

		t0 = k_cycle_get_32();
		rc = disk_access_write(disk_pdrv, scratch_buf[0], start, n);
		win_wr_us += k_cyc_to_us_floor64(k_cycle_get_32() - t0);
		if (rc) {
			err_count++;
			TC_PRINT("iter %u: write sector %u failed rc=%d (errs=%u)\n",
				 iter, start, rc, err_count);
			goto advance;
		}
		total_bytes += (uint64_t)n * disk_sector_size;
		win_wr_bytes += (uint64_t)n * disk_sector_size;

		memset(scratch_buf[1], 0, n * disk_sector_size);
		t0 = k_cycle_get_32();
		rc = disk_access_read(disk_pdrv, scratch_buf[1], start, n);
		win_rd_us += k_cyc_to_us_floor64(k_cycle_get_32() - t0);
		if (rc) {
			err_count++;
			TC_PRINT("iter %u: read sector %u failed rc=%d (errs=%u)\n",
				 iter, start, rc, err_count);
			goto advance;
		}
		total_bytes += (uint64_t)n * disk_sector_size;
		win_rd_bytes += (uint64_t)n * disk_sector_size;

		if (memcmp(scratch_buf[0], scratch_buf[1], n * disk_sector_size) != 0) {
			err_count++;
			TC_PRINT("iter %u: verify sector %u mismatch (errs=%u)\n",
				 iter, start, err_count);
		}

advance:
		/* Advance to the next window; wrap to the start of the card and
		 * count a completed full-card sweep.
		 */
		start += n;
		if (start >= disk_sector_count) {
			start = 0;
			sweep++;
		}

		if ((iter % 100) == 0) {
			/* Report cumulative bytes (auto-scaled kB/MB/GB, 1024-based)
			 * and the last-100-iteration average write/read rate in KB/s.
			 */
			uint64_t disp = total_bytes;
			const char *unit = "B";

			if (total_bytes >= (1ULL << 30)) {
				disp = total_bytes * 100ULL >> 30;
				unit = "GB";
			} else if (total_bytes >= (1ULL << 20)) {
				disp = total_bytes * 100ULL >> 20;
				unit = "MB";
			} else if (total_bytes >= (1ULL << 10)) {
				disp = total_bytes * 100ULL >> 10;
				unit = "kB";
			} else {
				disp = total_bytes * 100ULL;
			}

			TC_PRINT("iter %u sweep %u sector %u, total errors %u, "
				 "total rw %llu.%02llu %s, "
				 "write %llu KB/s, read %llu KB/s\n",
				 iter, sweep, start, err_count,
				 (unsigned long long)(disp / 100),
				 (unsigned long long)(disp % 100), unit,
				 (unsigned long long)(win_wr_us ?
					win_wr_bytes * 1000000ULL / win_wr_us / 1024ULL : 0),
				 (unsigned long long)(win_rd_us ?
					win_rd_bytes * 1000000ULL / win_rd_us / 1024ULL : 0));

			win_wr_bytes = 0;
			win_rd_bytes = 0;
			win_wr_us = 0;
			win_rd_us = 0;
		}

		/* Every 10000 iterations, trim the region written by the previous
		 * 10000 iterations so its blocks are freed and the SLC-cache fast
		 * budget can be replenished. [erase_from, start) is the window just
		 * written; if start has wrapped below erase_from, trim to the end of
		 * the card and let the next window cover the wrapped-around head.
		 */
		if ((iter % 10000) == 0) {
			uint32_t e_end = (start > erase_from) ? start : disk_sector_count;

			if (e_end > erase_from) {
				int erc = disk_access_erase(disk_pdrv, erase_from,
							    e_end - erase_from,
							    DISK_ACCESS_ERASE_PHYSICAL);

				TC_PRINT("iter %u: erase [%u..%u] (%u sectors) rc=%d\n",
					 iter, erase_from, e_end - 1,
					 e_end - erase_from, erc);
			}
			erase_from = start;
		}
	}
}
#endif /* CONFIG_DISK_ACCESS_STRESS */

static void *disk_driver_setup(void)
{
#ifdef CONFIG_DISK_DRIVER_LOOPBACK
	setup_loopback_backing();
#endif
	test_setup();

	return NULL;
}

ZTEST_SUITE(disk_driver, NULL, disk_driver_setup, NULL, NULL, NULL);
