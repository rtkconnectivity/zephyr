import re
from typing import List

from pm_device_pytest.pm_device_common import run_pm_shell_random_wakeup_single

I2C_REG_RE = re.compile(r"icm20618 addr:0x68 reg:\s*0x0\s*=\s*0x(?P<val>[0-9a-fA-F]+)")
EXPECT_REG0_VAL = 0xA0


def _find_i2c_values(lines: List[str]) -> List[int]:
    return [int(m.group("val"), 16) for line in lines if (m := I2C_REG_RE.search(line))]


def test_pm_i2c(dut):
    def check_before(lines: List[str]) -> None:
        vals = _find_i2c_values(lines)
        assert vals, "No icm20618 read before dlps"
        assert vals[-1] == EXPECT_REG0_VAL, f"Before dlps: expected 0x{EXPECT_REG0_VAL:02x}, got 0x{vals[-1]:02x}"

    def check_after(lines: List[str]) -> None:
        vals = _find_i2c_values(lines)
        assert len(vals) >= 2, f"Expected at least 2 i2c reads, got {len(vals)}"
        assert vals[0] == EXPECT_REG0_VAL, f"Before dlps: expected 0x{EXPECT_REG0_VAL:02x}, got 0x{vals[0]:02x}"
        assert vals[-1] == EXPECT_REG0_VAL, f"After dlps: expected 0x{EXPECT_REG0_VAL:02x}, got 0x{vals[-1]:02x}"

    run_pm_shell_random_wakeup_single(
        dut,
        subcmd="i2c",
        timeout=10.0,
        rand_min_ms=1000,
        rand_max_ms=5000,
        delta_tolerance_ms=300,
        check_before_dlps=check_before,
        check_after_dlps=check_after,
        rand_seed=None,
    )