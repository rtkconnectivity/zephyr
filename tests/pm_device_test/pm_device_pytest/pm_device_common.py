# pm_device_pytest/pm_device_common.py

import random
import re
import time
from dataclasses import dataclass
from typing import Callable, List, Optional, Tuple

from .conftest import SerialShellDUT

# Timestamps in logs look like: [451730]
TIMESTAMP_REGEX = re.compile(r"\[(\d+)\]")


def extract_timestamp(line: str) -> Optional[int]:
    """Extract a timestamp in the form [123456] from a log line (unit: ms)."""
    m = TIMESTAMP_REGEX.search(line)
    if not m:
        return None
    return int(m.group(1))


@dataclass
class DLPSSession:
    """One DLPS round-trip (enter -> exit)."""
    before_ts: int
    after_ts: int
    before_idx: int
    after_idx: int

    @property
    def delta_ms(self) -> int:
        return self.after_ts - self.before_ts


def parse_all_dlps_sessions(lines: List[str]) -> List[DLPSSession]:
    """Parse all DLPS sessions from the given log lines.

    Matching rules:
    - A line containing "before enter dlps" is treated as the enter point.
    - A line containing "after exit dlps"   is treated as the exit point.
    - The most recent "before" is paired with the next "after".
    """
    sessions: List[DLPSSession] = []
    current_before_idx: Optional[int] = None
    current_before_ts: Optional[int] = None

    for idx, line in enumerate(lines):
        if "before enter dlps" in line:
            ts = extract_timestamp(line)
            assert ts is not None, f"Failed to parse timestamp from line: {line!r}"
            current_before_idx = idx
            current_before_ts = ts

        elif "after exit dlps" in line:
            ts = extract_timestamp(line)
            assert ts is not None, f"Failed to parse timestamp from line: {line!r}"
            if current_before_ts is not None:
                assert ts >= current_before_ts, (
                    f"after_exit_ts < before_enter_ts: {ts} < {current_before_ts}"
                )
                sessions.append(
                    DLPSSession(
                        before_ts=current_before_ts,
                        after_ts=ts,
                        before_idx=current_before_idx,
                        after_idx=idx,
                    )
                )
                current_before_idx = None
                current_before_ts = None

    return sessions


def assert_single_dlps_present(lines: List[str]) -> DLPSSession:
    """Assert that there is exactly one DLPS session in the log and return it."""
    sessions = parse_all_dlps_sessions(lines)
    assert len(sessions) == 1, f"Expected 1 DLPS session, got {len(sessions)}"
    return sessions[0]


def assert_delta_close_to(
    session: DLPSSession,
    *,
    target_ms: int,
    tolerance_ms: int,
    reason: str = "",
) -> None:
    """Assert that session.delta_ms is close to target_ms within the given tolerance."""
    delta = session.delta_ms
    diff = abs(delta - target_ms)
    msg = (
        f"DLPS delta_ms diff too large: |{delta} - {target_ms}| = {diff} > {tolerance_ms}"
        + (f" ({reason})" if reason else "")
    )
    assert diff <= tolerance_ms, msg


# =======================================================
# 1. Generic helper: random delay + shell-enter wakeup + timestamp check
# =======================================================

def run_pm_shell_random_wakeup_single(
    dut: SerialShellDUT,
    *,
    subcmd: str,
    timeout: float = 10.0,
    # Random delay range in milliseconds
    rand_min_ms: int = 1000,
    rand_max_ms: int = 5000,
    # Allowed difference between timestamp delta and random delay (ms)
    delta_tolerance_ms: int = 300,
    check_before_dlps: Optional[Callable[[List[str]], None]] = None,
    check_after_dlps: Optional[Callable[[List[str]], None]] = None,
    rand_seed: Optional[int] = None,
) -> Tuple[List[str], DLPSSession, int]:
    """Helper for single DLPS test cases where:

    - Firmware prints a prompt 'type on shell to wakeup'.
    - pytest waits for a random delay and then sends ENTER on the shell to wake up.
    - The timestamp delta between 'before enter dlps' and 'after exit dlps' is checked
      against the random delay.

    Typical usage:
      - For uart / i2c or other tests that use shell ENTER to wake up and need to
        validate random wake-up delays.
      - The random delay is controlled by pytest.
      - Verifies that 'after_ts - before_ts' ≈ random_delay_ms.

    Returns:
      (all_logs, dlps_session, rand_delay_ms)

    This helper assumes that the command finishes and returns to the shell prompt
    after the DLPS session.
    """
    if rand_seed is not None:
        random.seed(rand_seed)

    # 1. Generate a random delay
    rand_delay_ms = random.randint(rand_min_ms, rand_max_ms)

    # 2. Send command
    dut.write(f"pm_test {subcmd}\r".encode("ascii"))

    # 3. Read until "type on shell to wakeup"
    logs_pre = dut.readlines_until(
        regex=r"type on shell to wakeup",
        timeout=timeout,
    )
    text_pre = "\n".join(logs_pre)

    # Ensure that DLPS enter messages are present
    assert "before enter dlps" in text_pre, "Missing 'before enter dlps' log"
    assert "type on shell to wakeup" in text_pre, "Missing 'type on shell to wakeup' log"

    if check_before_dlps is not None:
        check_before_dlps(logs_pre)

    # 4. Wait for the random delay, then wake up via UART ENTER
    time.sleep(rand_delay_ms / 1000.0)
    dut.write(b"\r")

    # 5. Read until "after exit dlps"
    logs_mid = dut.readlines_until(
        regex=r"after exit dlps",
        timeout=timeout,
    )

    # 6. Then read until shell prompt to mark the end of this interaction
    logs_post = dut.readlines_until(
        regex=r"uart:~\$ *$",
        timeout=timeout,
    )

    all_logs: List[str] = logs_pre + logs_mid + logs_post

    # 7. Parse the DLPS session and check the random delay
    session = assert_single_dlps_present(all_logs)
    assert_delta_close_to(
        session,
        target_ms=rand_delay_ms,
        tolerance_ms=delta_tolerance_ms,
        reason=f"{subcmd} shell random wakeup",
    )

    if check_after_dlps is not None:
        check_after_dlps(all_logs)

    return all_logs, session, rand_delay_ms

def run_pm_shell_random_wakeup_multi(
    dut: SerialShellDUT,
    *,
    subcmd: str,
    count: int,
    timeout: float = 10.0,
    rand_min_ms: int = 1000,
    rand_max_ms: int = 5000,
    delta_tolerance_ms: int = 300,
    rand_seed: Optional[int] = None,
) -> Tuple[List[str], List[DLPSSession], List[int]]:
    """Run a command that performs multiple DLPS sessions in sequence.

    The firmware is expected to behave as:
      - For each session i in [0, count):
          * print "before enter dlps [ts]"
          * print "type on shell to wakeup"
          * enter DLPS and wait for shell ENTER
          * print "after exit dlps [ts]"
      - After all sessions are done, return to the shell prompt "uart:~$".

    This helper:
      - Sends "pm_test <subcmd>".
      - For each session:
          * waits for "before enter dlps" + "type on shell to wakeup".
          * waits for a random delay, sends ENTER.
          * waits for "after exit dlps".
      - After the last session, waits for "uart:~$".
      - Parses all DLPS sessions and checks each delta against its random delay.

    Returns:
      (all_logs, sessions, rand_delays_ms)
    """
    if rand_seed is not None:
        random.seed(rand_seed)

    # Generate random delays for all sessions in advance, for reproducibility
    rand_delays: List[int] = [
        random.randint(rand_min_ms, rand_max_ms) for _ in range(count)
    ]

    all_logs: List[str] = []

    # Send command once
    dut.write(f"pm_test {subcmd}\r".encode("ascii"))

    for i in range(count):
        # Wait for "before enter dlps" + "type on shell to wakeup" for this session
        logs_pre = dut.readlines_until(
            regex=r"type on shell to wakeup",
            timeout=timeout,
        )
        text_pre = "\n".join(logs_pre)
        assert "before enter dlps" in text_pre, (
            f"Session {i + 1} missing 'before enter dlps'"
        )
        assert "type on shell to wakeup" in text_pre, (
            f"Session {i + 1} missing 'type on shell to wakeup'"
        )

        all_logs.extend(logs_pre)

        # Wait random delay and wakeup
        time.sleep(rand_delays[i] / 1000.0)
        dut.write(b"\r")

        # Wait for "after exit dlps" of this session
        logs_mid = dut.readlines_until(
            regex=r"after exit dlps",
            timeout=timeout,
        )
        all_logs.extend(logs_mid)

    # After all sessions are done, read until shell prompt
    logs_post = dut.readlines_until(
        regex=r"uart:~\$ *$",
        timeout=timeout,
    )
    all_logs.extend(logs_post)

    # Parse sessions
    sessions = parse_all_dlps_sessions(all_logs)
    assert len(sessions) == count, (
        f"Expected {count} DLPS sessions, got {len(sessions)}"
    )

    # Check each session delta against its random delay
    for i, (sess, delay) in enumerate(zip(sessions, rand_delays), start=1):
        assert_delta_close_to(
            sess,
            target_ms=delay,
            tolerance_ms=delta_tolerance_ms,
            reason=f"{subcmd} session {i}",
        )

    return all_logs, sessions, rand_delays

def run_pm_shell_single_auto_wakeup(
    dut: SerialShellDUT,
    *,
    subcmd: str,
    timeout: float = 10.0,
    done_regex: Optional[str] = None,
    check_before_dlps: Optional[Callable[[List[str]], None]] = None,
    check_after_dlps: Optional[Callable[[List[str]], None]] = None,
) -> Tuple[List[str], DLPSSession]:
    """Helper for single DLPS test cases where:

    - Firmware enters DLPS and is woken up automatically (e.g. by a hardware
      interrupt such as RTC alarm, counter, GPIO, etc.).
    - No shell ENTER or other host-side action is used to wake up the device.
    - The function waits until:
        * a complete DLPS session is observed in logs
          ('before enter dlps' + 'after exit dlps'), and
        * an optional done_regex is matched (e.g. 'trigger handler after ...ms').

    Typical usage:
      - For tests where wakeup is driven solely by on-board peripherals and
        firmware, not by the test host (e.g. RTC auto wakeup).

    Returns:
      (all_logs, dlps_session)
    """
    # 1. Send command
    dut.write(f"pm_test {subcmd}\r".encode("ascii"))

    # 2. Collect logs until:
    #    - we see a complete DLPS session, and
    #    - (optionally) done_regex is matched.
    all_logs: List[str] = []
    start_time = time.time()
    done_re = re.compile(done_regex) if done_regex else None

    saw_before = False
    saw_after = False
    done_matched = False

    while time.time() - start_time < timeout:
        # readlines_until with a generic pattern that always returns on timeout
        chunk_lines = dut.readlines_until(regex=r".*", timeout=1.0)
        if not chunk_lines:
            continue

        all_logs.extend(chunk_lines)

        text = "\n".join(chunk_lines)

        if "before enter dlps" in text:
            saw_before = True
        if "after exit dlps" in text:
            saw_after = True
        if done_re is not None and done_re.search(text):
            done_matched = True

        # Break if we already have a complete DLPS session and (if required)
        # the done condition is satisfied.
        if saw_before and saw_after and (done_re is None or done_matched):
            break

    # 3. Basic assertions on DLPS session existence
    session = assert_single_dlps_present(all_logs)

    if check_before_dlps is not None:
        check_before_dlps(all_logs)

    if check_after_dlps is not None:
        check_after_dlps(all_logs)

    return all_logs, session
