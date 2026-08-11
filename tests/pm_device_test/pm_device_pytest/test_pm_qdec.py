import re

from pm_device_pytest.pm_device_common import run_pm_shell_random_wakeup_multi

_POS_RE = re.compile(r"Position\[(\d+)\]\s*=\s*(-?\d+)\s+degrees")


def _parse_positions(lines) -> list:
    return [(int(m.group(1)), int(m.group(2))) for line in lines if (m := _POS_RE.search(line))]


def test_pm_qdec_multi_session(dut):
    count = 3

    all_logs, sessions, rand_delays = run_pm_shell_random_wakeup_multi(
        dut,
        subcmd="qdec",
        count=count,
        timeout=10.0,
        rand_min_ms=1000,
        rand_max_ms=5000,
        delta_tolerance_ms=300,
        rand_seed=1,
    )

    assert len(sessions) == count, f"Expected {count} DLPS sessions"
    assert len(rand_delays) == count, f"Expected {count} random delays"
    assert all_logs, "No logs captured"

    positions = _parse_positions(all_logs)
    expected_total = 48
    assert len(positions) == expected_total, f"Expected {expected_total} Position lines, got {len(positions)}"

    degrees = [deg for (_, deg) in positions]

    assert degrees[0:12] == list(range(1, 13)), f"Segment 1 pattern mismatch"
    assert degrees[12:24] == list(range(11, -1, -1)), f"Segment 2 pattern mismatch"
    assert degrees[24:36] == list(range(-1, -13, -1)), f"Segment 3 pattern mismatch"
    assert degrees[36:48] == list(range(-11, 1)), f"Segment 4 pattern mismatch"