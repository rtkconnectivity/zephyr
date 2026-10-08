from typing import List

from pm_device_pytest.pm_device_common import (
    parse_all_dlps_sessions,
    assert_delta_close_to,
)


def _find_lines(lines: List[str], keyword: str) -> List[int]:
    return [idx for idx, line in enumerate(lines) if keyword in line]


def test_pm_pwm(dut):
    dut.write(b"pm_test pwm\r")

    lines = dut.readlines_until(regex=r"uart:~\$ *$", timeout=10.0)

    sessions = parse_all_dlps_sessions(lines)
    assert len(sessions) == 2, f"Expected 2 DLPS sessions, got {len(sessions)}"

    assert_delta_close_to(sessions[0], target_ms=500, tolerance_ms=200, reason="pwm first dlps")
    assert_delta_close_to(sessions[1], target_ms=500, tolerance_ms=200, reason="pwm second dlps")

    text = "\n".join(lines)
    assert "connect pwm pin to LA to watch the waveform" in text

    idx_p10000 = _find_lines(lines, "[PWM]: pwm6, [period]: 50000, [pulse]: 10000")
    idx_p0 = _find_lines(lines, "[PWM]: pwm6, [period]: 0, [pulse]: 0")
    idx_p40000 = _find_lines(lines, "[PWM]: pwm6, [period]: 50000, [pulse]: 40000")

    assert idx_p10000, "Missing PWM config: period 50000, pulse 10000"
    assert idx_p40000, "Missing PWM config: period 50000, pulse 40000"
    assert len(idx_p0) >= 2, "Expected at least two PWM stop logs"

    i_10000, i_0_first, i_40000, i_0_second = idx_p10000[0], idx_p0[0], idx_p40000[0], idx_p0[1]

    assert i_10000 < sessions[0].before_idx, "PWM 50000/10000 should appear before first DLPS"
    assert sessions[0].after_idx < i_0_first < i_40000 < sessions[1].before_idx, "PWM logs order incorrect"
    assert sessions[1].after_idx < i_0_second, "Final PWM 0/0 should appear after second DLPS"