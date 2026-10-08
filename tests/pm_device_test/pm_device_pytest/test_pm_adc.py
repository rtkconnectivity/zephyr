import re
from typing import List, Tuple

from pm_device_pytest.pm_device_common import run_pm_shell_random_wakeup_single

ADC_BEFORE_RE = re.compile(r"ADC sample before dlps:\s*(?P<val>-?\d+)")
ADC_AFTER_RE = re.compile(r"ADC sample after dlps:\s*(?P<val>-?\d+)")

EXPECT_MIN_VAL = 3000
EXPECT_MAX_VAL = 3600
MAX_DELTA_ABS = 300


def _find_adc_values(lines: List[str]) -> Tuple[List[int], List[int]]:
    before_vals = [int(m.group("val")) for line in lines if (m := ADC_BEFORE_RE.search(line))]
    after_vals = [int(m.group("val")) for line in lines if (m := ADC_AFTER_RE.search(line))]
    return before_vals, after_vals


def test_pm_adc(dut):
    def check_before(lines: List[str]) -> None:
        before_vals, _ = _find_adc_values(lines)
        assert before_vals, "No 'ADC sample before dlps' found"
        val = before_vals[-1]
        assert EXPECT_MIN_VAL <= val <= EXPECT_MAX_VAL, f"ADC before dlps out of range: {val}"

    def check_after(lines: List[str]) -> None:
        before_vals, after_vals = _find_adc_values(lines)
        assert before_vals, "Expected 'ADC sample before dlps' in session"
        assert after_vals, "Expected 'ADC sample after dlps' in session"
        before_val, after_val = before_vals[-1], after_vals[-1]
        assert EXPECT_MIN_VAL <= before_val <= EXPECT_MAX_VAL, f"ADC before dlps out of range: {before_val}"
        assert EXPECT_MIN_VAL <= after_val <= EXPECT_MAX_VAL, f"ADC after dlps out of range: {after_val}"
        delta = abs(after_val - before_val)
        assert delta <= MAX_DELTA_ABS, f"ADC delta too large: before={before_val}, after={after_val}, delta={delta}"

    run_pm_shell_random_wakeup_single(
        dut,
        subcmd="adc",
        timeout=10.0,
        rand_min_ms=1000,
        rand_max_ms=5000,
        delta_tolerance_ms=300,
        check_before_dlps=check_before,
        check_after_dlps=check_after,
        rand_seed=None,
    )