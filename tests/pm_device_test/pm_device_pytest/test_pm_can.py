import re
from typing import List

from pm_device_pytest.pm_device_common import run_pm_shell_random_wakeup_single

# CAN RX frame log pattern
CAN_RX_RE = re.compile(r"can rx: id=(0x[0-9a-fA-F]+), dlc=(\d+), data:")

# TX/RX match pattern
CAN_MATCH_RE = re.compile(r"tx/rx match")

# DLPS enter/exit patterns
BEFORE_DLPS_RE = re.compile(r"before enter dlps")
AFTER_DLPS_RE = re.compile(r"after exit dlps")


def test_pm_can(dut):
    """Run CAN PM test and verify results."""

    def check_before(lines: List[str]) -> None:
        """Check CAN RX before DLPS."""
        rx_count = sum(1 for line in lines if CAN_RX_RE.search(line))
        match_count = sum(1 for line in lines if CAN_MATCH_RE.search(line))

        assert rx_count >= 1, "Before dlps: expected at least 1 CAN RX"
        assert match_count >= 1, "Before dlps: expected 'tx/rx match'"

    def check_after(lines: List[str]) -> None:
        """Check CAN RX after DLPS."""
        # Check for DLPS entry/exit
        has_before = any(BEFORE_DLPS_RE.search(line) for line in lines)
        has_after = any(AFTER_DLPS_RE.search(line) for line in lines)

        assert has_before, "After dlps: expected 'before enter dlps'"
        assert has_after, "After dlps: expected 'after exit dlps'"

        # Check CAN RX after wakeup
        # Find lines after "after exit dlps"
        dlps_idx = -1
        for i, line in enumerate(lines):
            if AFTER_DLPS_RE.search(line):
                dlps_idx = i
                break

        if dlps_idx >= 0:
            after_dlps_lines = lines[dlps_idx:]
            rx_count = sum(1 for line in after_dlps_lines if CAN_RX_RE.search(line))
            match_count = sum(1 for line in after_dlps_lines if CAN_MATCH_RE.search(line))

            assert rx_count >= 1, "After dlps: expected at least 1 CAN RX"
            assert match_count >= 1, "After dlps: expected 'tx/rx match'"

    run_pm_shell_random_wakeup_single(
        dut,
        subcmd="can",
        timeout=30.0,
        rand_min_ms=1000,
        rand_max_ms=5000,
        delta_tolerance_ms=300,
        check_before_dlps=check_before,
        check_after_dlps=check_after,
        rand_seed=None,
    )