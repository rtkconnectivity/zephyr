"""Run all PM tests in sequence with single build and flash.

Tests are selected based on Kconfig options in the firmware.
Each test uses its own pass/fail logic from the individual test_pm_xxx.py files.
"""

import importlib
from pathlib import Path
import sys


def _load_kconfig():
    """Load Kconfig options from build directory."""
    possible_paths = [
        Path(__file__).parent.parent / "build" / "zephyr" / "include" / "generated" / "zephyr" / "autoconf.h",
        Path(__file__).parent.parent / "twister-out" / "rtl87x2g_evb" / "tests" / "boards" / "pm_device_test" / "driver.pm.bee_evb_all_test" / "zephyr" / "include" / "generated" / "zephyr" / "autoconf.h",
    ]

    config = {}
    for autoconf_path in possible_paths:
        if autoconf_path.exists():
            with open(autoconf_path, "r") as f:
                for line in f:
                    if line.startswith("#define CONFIG_"):
                        parts = line.strip().split()
                        if len(parts) >= 3:
                            config[parts[1]] = parts[2]
            break

    return config


# Test configurations - maps Kconfig option to test file and function
# Format: (config_opts, module_name, func_name)
# - config_opts: single config or tuple of configs (any one enables the test)
# - module_name: the test file name (without .py)
# - func_name: the test function name in that file
TEST_CONFIGS = [
    (("CONFIG_ADC",), "test_pm_adc", "test_pm_adc"),
    (("CONFIG_AON_QDEC_BEE",), "test_pm_aon_qdec", "test_pm_aon_qdec_multi_session"),
    (("CONFIG_CAN",), "test_pm_can", "test_pm_can"),
    (("CONFIG_COUNTER",), "test_pm_counter", "test_pm_counter"),
    (("CONFIG_GPIO",), "test_pm_gpio", "test_pm_gpio"),
    (("CONFIG_I2C",), "test_pm_i2c", "test_pm_i2c"),
    (("CONFIG_PWM",), "test_pm_pwm", "test_pm_pwm"),
    (("CONFIG_QDEC_BEE", "CONFIG_QDEC_RTL87X3G"), "test_pm_qdec", "test_pm_qdec_multi_session"),
    (("CONFIG_RTC",), "test_pm_rtc", "test_pm_rtc"),
    (("CONFIG_SDMMC_STACK",), "test_pm_sdhc", "test_pm_sdhc"),
    (("CONFIG_SPI",), "test_pm_spi", "test_pm_spi"),
    (("CONFIG_SERIAL",), "test_pm_uart", "test_pm_uart"),
    (("CONFIG_UART_ASYNC_API",), "test_pm_uart_dma", "test_pm_uart_dma"),
]


def _get_enabled_tests(kconfig):
    """Get list of enabled tests based on Kconfig."""
    enabled = []

    for config_opts, module_name, func_name in TEST_CONFIGS:
        # Handle both single config and tuple of configs
        if isinstance(config_opts, tuple):
            config_list = config_opts
        else:
            config_list = (config_opts,)

        enabled_now = False
        enabled_config = None

        for config_opt in config_list:
            if config_opt in kconfig and kconfig[config_opt] == "1":
                enabled_now = True
                enabled_config = config_opt
                break

        if enabled_now:
            # Check if module exists
            try:
                importlib.import_module(f"pm_device_pytest.{module_name}")
                enabled.append((module_name, func_name))
                print(f"[CONFIG] {enabled_config} enabled - will run {module_name} test")
            except ModuleNotFoundError:
                print(f"[CONFIG] {enabled_config} enabled but {module_name}.py not found - skipping")
        else:
            config_str = " or ".join(config_list)
            print(f"[CONFIG] {config_str} disabled - skipping {module_name} test")

    return enabled


def test_pm_all(dut, dma_dut):
    """Run all PM tests based on Kconfig selection.

    Single build + flash, runs all enabled tests.
    Each test uses its own pass/fail logic from the individual test_pm_xxx.py files.
    """
    # Load Kconfig
    kconfig = _load_kconfig()
    if not kconfig:
        print("WARNING: Could not load Kconfig, running all tests")

    enabled_tests = _get_enabled_tests(kconfig)

    # Sort tests by module name (alphabetical order)
    enabled_tests = sorted(enabled_tests, key=lambda x: x[0])

    print("\n" + "=" * 50)
    print("Starting PM Test Suite")
    print(f"Found {len(enabled_tests)} tests to run")
    print("=" * 50)

    passed = 0
    failed = 0
    errors = 0
    results = {}

    for module_name, func_name in enabled_tests:
        print(f"\n--- Running: {module_name}.{func_name} ---")

        try:
            # Import the module and get the test function
            module = importlib.import_module(f"pm_device_pytest.{module_name}")
            test_func = getattr(module, func_name)

            # Run the test - some tests may need special fixtures
            if module_name == "test_pm_uart_dma":
                # uart_dma needs dma_dut fixture
                try:
                    test_func(dut, dma_dut)
                except TypeError:
                    # If function signature changed, try without dma_dut
                    test_func(dut)
            else:
                test_func(dut)

            # If we get here, test passed
            results[module_name] = "PASSED"
            passed += 1
            print(f"[{module_name}] PASSED")

        except AssertionError as e:
            results[module_name] = "FAILED"
            failed += 1
            print(f"[{module_name}] FAILED: {e}")

        except Exception as e:
            results[module_name] = "ERROR"
            errors += 1
            print(f"[{module_name}] ERROR: {e}")

    # Summary (sorted alphabetically)
    print("\n" + "=" * 50)
    print("Test Summary")
    print("=" * 50)
    for name in sorted(results.keys()):
        print(f"  {name}: {results[name]}")
    print(f"\nTotal: {passed} passed, {failed} failed, {errors} errors")
    print("=" * 50)

    # Assert if any test failed
    assert failed == 0, f"{failed} test(s) failed!"
    assert errors == 0, f"{errors} test(s) had errors!"