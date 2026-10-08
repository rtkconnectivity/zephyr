import re
from typing import List

from pm_device_pytest.pm_device_common import run_pm_shell_random_wakeup_single

SDMMC_INIT_RE = re.compile(r"before dlps sdmmc card .* initialization success")
SDMMC_READ_OK_RE = re.compile(r"before dlps sdmmc card read success")
SDMMC_READ_FAIL_RE = re.compile(r"before dlps sdmmc card read fail")
SDIO_INIT_RE = re.compile(r"before dlps sdio card .* initialization success")
SDMMC_AFTER_INIT_RE = re.compile(r"after dlps sdmmc card .* initialization success")
SDMMC_AFTER_READ_OK_RE = re.compile(r"after dlps sdmmc card read success")
SDMMC_AFTER_READ_FAIL_RE = re.compile(r"after dlps sdmmc card read fail")
SDIO_AFTER_INIT_RE = re.compile(r"after dlps sdio card .* initialization success")


def _check_sdmmc_before_dlps(lines: List[str]) -> None:
    text = "\n".join(lines)
    if SDMMC_INIT_RE.search(text):
        assert SDMMC_INIT_RE.search(text), "SDMMC init before DLPS failed"
    if SDMMC_READ_OK_RE.search(text):
        assert SDMMC_READ_OK_RE.search(text), "SDMMC read before DLPS failed"
    elif SDMMC_READ_FAIL_RE.search(text):
        assert False, "SDMMC read before DLPS failed"
    if SDIO_INIT_RE.search(text):
        assert SDIO_INIT_RE.search(text), "SDIO init before DLPS failed"


def _check_sdmmc_after_dlps(lines: List[str]) -> None:
    text = "\n".join(lines)
    if SDMMC_AFTER_INIT_RE.search(text):
        assert SDMMC_AFTER_INIT_RE.search(text), "SDMMC init after DLPS failed"
    if SDMMC_AFTER_READ_OK_RE.search(text):
        assert SDMMC_AFTER_READ_OK_RE.search(text), "SDMMC read after DLPS failed"
    elif SDMMC_AFTER_READ_FAIL_RE.search(text):
        assert False, "SDMMC read after DLPS failed"
    if SDIO_AFTER_INIT_RE.search(text):
        assert SDIO_AFTER_INIT_RE.search(text), "SDIO init after DLPS failed"


def test_pm_sdhc(dut):
    run_pm_shell_random_wakeup_single(
        dut,
        subcmd="sdhc",
        timeout=30.0,
        rand_min_ms=1000,
        rand_max_ms=5000,
        delta_tolerance_ms=300,
        check_before_dlps=_check_sdmmc_before_dlps,
        check_after_dlps=_check_sdmmc_after_dlps,
        rand_seed=None,
    )