import re
from typing import List

from pm_device_pytest.pm_device_common import run_pm_shell_random_wakeup_single

START_LOOP_RE = re.compile(r"Start complete loop")
BUF_SAME_RE = re.compile(r"Buffer contents are same")
BUF_DIFF_RE = re.compile(r"Buffer contents are different")


def _count_spi_loops(lines: List[str]) -> tuple:
    start_cnt = sum(1 for line in lines if START_LOOP_RE.search(line))
    same_cnt = sum(1 for line in lines if BUF_SAME_RE.search(line))
    diff_cnt = sum(1 for line in lines if BUF_DIFF_RE.search(line))
    return start_cnt, same_cnt, diff_cnt


def test_pm_spi(dut):
    def check_before(lines: List[str]) -> None:
        start_cnt, same_cnt, diff_cnt = _count_spi_loops(lines)
        assert start_cnt >= 1, "Before dlps: expected at least 1 'Start complete loop'"
        assert same_cnt >= 1, "Before dlps: expected at least 1 'Buffer contents are same'"
        assert diff_cnt == 0, "Before dlps: found 'Buffer contents are different'"

    def check_after(lines: List[str]) -> None:
        start_cnt, same_cnt, diff_cnt = _count_spi_loops(lines)
        assert start_cnt >= 2, f"Expected at least 2 'Start complete loop', got {start_cnt}"
        assert same_cnt >= 2, f"Expected at least 2 'Buffer contents are same', got {same_cnt}"
        assert diff_cnt == 0, f"Found 'Buffer contents are different' in logs"

    run_pm_shell_random_wakeup_single(
        dut,
        subcmd="spi",
        timeout=10.0,
        rand_min_ms=1000,
        rand_max_ms=5000,
        delta_tolerance_ms=300,
        check_before_dlps=check_before,
        check_after_dlps=check_after,
        rand_seed=None,
    )