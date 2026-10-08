import random
from typing import List

from pm_device_pytest.pm_device_common import (
    run_pm_shell_single_auto_wakeup,
    assert_delta_close_to,
)

RTC_WAIT_RE = r"wait\s+(?P<timeout_ms>\d+)ms\s+to trigger handler"
RTC_TRIGGER_RE = r"trigger handler after\s+(?P<after_ms>\d+)ms"

TIMEOUT_MIN_MS = 1000
TIMEOUT_MAX_MS = 5000
RTC_DELTA_TOLERANCE_MS = 300


def _find_rtc_values(lines: List[str], regex: str) -> tuple:
    import re
    for line in lines:
        m = re.search(regex, line)
        if m:
            return int(m.group("timeout_ms")), int(m.group("after_ms"))
    raise AssertionError(f"No match for {regex} in logs")


def test_pm_rtc(dut):
    timeout_ms = random.randint(TIMEOUT_MIN_MS, TIMEOUT_MAX_MS)

    def check_before(lines: List[str]) -> None:
        import re
        for line in lines:
            m = re.search(RTC_WAIT_RE, line)
            if m and int(m.group("timeout_ms")) != timeout_ms:
                raise AssertionError(f"RTC timeout mismatch: expected {timeout_ms}")

    def check_after(lines: List[str]) -> None:
        import re
        _, after_ms = _find_rtc_values(lines, f"{RTC_WAIT_RE}|{RTC_TRIGGER_RE}")
        diff = abs(after_ms - timeout_ms)
        assert diff <= RTC_DELTA_TOLERANCE_MS, f"RTC trigger diff too large: {diff}ms"

    logs, session = run_pm_shell_single_auto_wakeup(
        dut,
        subcmd=f"rtc {timeout_ms}",
        timeout=15.0,
        done_regex=RTC_TRIGGER_RE,
        check_before_dlps=check_before,
        check_after_dlps=check_after,
    )

    assert_delta_close_to(session, target_ms=timeout_ms, tolerance_ms=RTC_DELTA_TOLERANCE_MS, reason="RTC auto wakeup")