from pm_device_pytest.pm_device_common import run_pm_shell_random_wakeup_single


def test_pm_uart(dut):
    run_pm_shell_random_wakeup_single(
        dut,
        subcmd="uart",
        timeout=10.0,
        rand_min_ms=1000,
        rand_max_ms=5000,
        delta_tolerance_ms=300,
        check_before_dlps=None,
        check_after_dlps=None,
        rand_seed=None,
    )