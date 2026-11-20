#!/usr/bin/env python3
# Copyright (c) 2026 Realtek Semiconductor Corp.
# SPDX-License-Identifier: Apache-2.0
"""Run the driver tests that need no hands on a board and judge PASS/FAIL.

Phases (all by default, pick with --phases):
  twister  twister device testing of counter/dma/pwm/can/adc/spi/gpio
  uart     tests/drivers/uart/uart_basic_api, Enter is sent on each prompt
  pm       tests/boards/realtek/bee/bee_pm_device pm_test items, gpio/qdec are
           driven by a peer board already flashed with the same test (--peer-port)

Images are flashed with rtkprog over the console port. rtkprog drives RST with
RTS and BOOT with DTR, so the port is opened with both released, the same as
'rtkprog terminal', and the board is reset into the application before each run.
"""

import argparse
import json
import re
import subprocess
import sys
import time
from pathlib import Path

import serial

PM_TEST = "zephyr/tests/boards/realtek/bee/bee_pm_device"
# CONFIG_UART_BEE_KEEP_ACTIVE_TIMEOUT_MSEC plus a margin
KEEP_ACTIVE_S = 1.5
# Printable, so the shell echoes it once the UART is awake
PROBE = "~"
CTRL_U = "\x15"
ANSI = re.compile(r"\x1b\[[0-9;?]*[A-Za-z]")
RESULT = re.compile(r"(\w+) test (PASS|FAIL): ([^\n]*)")
WAKEUP = re.compile(r"wakeup_count=(\d+)")

# No wiring first, then the loopback wires
TWISTER_SCENARIOS = [
    "drivers.counter.basic_api",
    "drivers.dma.loop_transfer",
    "drivers.pwm",
    "drivers.can.api",
    "drivers.adc",
    "drivers.spi.loopback",
    "drivers.gpio.2pin",
]

# Flash and console share the port, the same map as the test plan with the port filled in
HW_MAP = """- connected: true
  id: "0"
  platform: {board}
  product: rtkprog
  runner: rtkprog
  runner_params:
    - --port
    - {port}
  flash_before: true
  fixtures:
    - gpio_loopback
    - spi_loopback
  serial_pty: rtkprog,--port,{port},-b,115200,terminal
"""


class Board:
    """Console of one board, every byte read goes to its log file."""

    def __init__(self, name, port, baud, log_path):
        self.name = name
        self.buf = ""
        # Closed in close()
        self.log = open(log_path, "a", buffering=1)  # noqa: SIM115
        self.ser = serial.Serial()
        self.ser.port = port
        self.ser.baudrate = baud
        self.ser.timeout = 0.05
        # Keep RST and BOOT released while the port opens
        self.ser.rts = False
        self.ser.dtr = False
        self.ser.open()

    def close(self):
        self.ser.close()
        self.log.close()

    def note(self, text):
        self.log.write(f"\n### {text}\n")

    def reset(self):
        """Reset into the application, as rtkprog reset_into_run does."""
        self.note("reset")
        self.ser.rts = False
        self.ser.dtr = False
        time.sleep(0.1)
        self.ser.rts = True
        time.sleep(0.6)
        self.ser.reset_input_buffer()
        self.ser.rts = False
        time.sleep(0.2)
        self.buf = ""

    def read(self, t):
        end = time.time() + t
        new = ""
        while time.time() < end:
            data = self.ser.read(4096)
            if data:
                text = ANSI.sub("", data.decode(errors="replace")).replace("\r", "")
                self.log.write(text)
                new += text
        self.buf += new
        return new

    def wait_for(self, pattern, timeout, start=0):
        """Read until pattern shows up in buf[start:], return the match or None."""
        end = time.time() + timeout
        while True:
            m = re.search(pattern, self.buf[start:])
            if m or time.time() >= end:
                return m
            self.read(0.05)

    def idle(self, quiet=0.3, timeout=5):
        """Read until the console has been quiet for 'quiet' seconds."""
        end = time.time() + timeout
        while time.time() < end and self.read(quiet):
            pass

    def send(self, text):
        for ch in text:
            self.ser.write(ch.encode())
            time.sleep(0.002)


def run(cmd, cwd, log_path):
    with open(log_path, "a") as log:
        log.write(f"\n$ {' '.join(map(str, cmd))}\n")
        log.flush()
        return subprocess.call(list(map(str, cmd)), cwd=cwd, stdout=log, stderr=subprocess.STDOUT)


def west_build(top, build_dir, board, src, log_path):
    return (
        run(["west", "build", "-p", "auto", "-d", build_dir, "-b", board, src], top, log_path) == 0
    )


def west_flash(top, build_dir, port, log_path):
    return (
        run(["west", "flash", "-d", build_dir, "-r", "rtkprog", "--port", port], top, log_path) == 0
    )


def console_baud(build_dir):
    """Baud of the shell UART, taken from the final devicetree."""
    dts = (Path(build_dir) / "zephyr" / "zephyr.dts").read_text()
    label = re.search(r"zephyr,(?:shell-uart|console) = &(\w+);", dts).group(1)
    node = dts[re.search(rf"\b{label}: ", dts).end() :]
    return int(re.search(r"current-speed = < (\w+) >;", node).group(1), 0)


class Report:
    def __init__(self):
        self.items = []

    def add(self, name, ok, detail=""):
        """Items are numbered 0, 1, 2... in the order they run."""
        self.items.append((len(self.items), name, ok, detail))
        print(f"{self.items[-1][0]:>2} {name:<26} {'PASS' if ok else 'FAIL'}  {detail}", flush=True)

    def failed(self):
        return [i for i in self.items if not i[2]]


def phase_twister(args, rep):
    """twister with the hardware map, check twister.json."""
    zephyr = args.topdir / "zephyr"
    outdir = args.outdir / "twister-out"
    hw_map = args.outdir / "hardware_map.yml"
    hw_map.write_text(HW_MAP.format(board=args.board, port=args.port))
    cmd = [
        "west",
        "twister",
        "--device-testing",
        "--hardware-map",
        hw_map,
        "-T",
        "tests/drivers/",
        "--no-detailed-test-id",
        "--short-build-path",
        "-O",
        outdir,
        "-c",
    ]
    for s in TWISTER_SCENARIOS:
        cmd += ["-s", s]
    run(cmd, zephyr, args.outdir / "twister.log")

    try:
        suites = json.loads((outdir / "twister.json").read_text())["testsuites"]
    except (OSError, ValueError, KeyError) as e:
        rep.add("twister", False, f"no twister.json ({e})")
        return

    for s in TWISTER_SCENARIOS:
        st = [
            t.get("status")
            for t in suites
            if Path(t.get("name", "")).name == s and t.get("platform") == args.board
        ]
        rep.add(s, st == ["passed"], ",".join(map(str, st)) or "not run")


def phase_uart(args, rep):
    """uart_basic_api, Enter answers each 'Please send characters'."""
    build = args.outdir / "uart_basic_api"
    src = args.topdir / "zephyr/tests/drivers/uart/uart_basic_api"
    log = args.outdir / "build.log"
    if not west_build(args.topdir, build, args.board, src, log) or not west_flash(
        args.topdir, build, args.port, log
    ):
        rep.add("uart_basic_api", False, f"build/flash failed, see {log}")
        return

    b = Board("dut", args.port, console_baud(build), args.outdir / "uart_basic_api.log")
    try:
        b.reset()
        pos = 0
        prompts = 0
        while prompts < 2:
            m = b.wait_for(r"Please send characters to serial console", 30, pos)
            if not m:
                break
            pos += m.end()
            prompts += 1
            time.sleep(0.2)
            b.send("\r")
        m = b.wait_for(r"PROJECT EXECUTION (SUCCESSFUL|FAILED)", 60)
        suites = re.findall(r"SUITE (\w+) - [\d.]+% \[(\w+)\]", b.buf)
        ok = (
            bool(m)
            and m.group(1) == "SUCCESSFUL"
            and {n for r, n in suites if r == "PASS"}
            >= {"uart_basic_api", "uart_basic_api_pending"}
        )
        rep.add(
            "uart_basic_api", ok, m.group(0) if m else f"no summary, {prompts} prompts answered"
        )
    finally:
        b.close()


class DutShell:
    """pm_test shell on the board under test.

    Once the shell UART reference is handed back, the UART sleeps after the
    keep active timeout and the frame that wakes it is lost. Each command goes
    out the same way: wait for the timeout, send a probe that must not be echoed,
    then type the command inside the keep active window. What goes wrong on the
    way is kept in errors and fails the item of that command.
    """

    def __init__(self, board):
        self.b = board
        self.uart_pm = False
        self.wakeups = []
        self.errors = []

    def start(self, name, cmd):
        """Type cmd, return where its output starts in the console buffer."""
        b = self.b
        b.note(name)
        self.errors = []
        b.idle(KEEP_ACTIVE_S if self.uart_pm else 0.3, timeout=120)
        start = len(b.buf)
        if self.uart_pm:
            b.send(PROBE)
            b.read(0.3)
            if PROBE in b.buf[start:]:
                self.errors.append("wakeup frame echoed")
        # CTRL+U drops whatever is left on the line
        b.send(CTRL_U + cmd + "\r")
        if not b.wait_for(re.escape(cmd), 2, start):
            self.errors.append("command not echoed")
        return start

    def finish(self, start, done, timeout):
        self.b.wait_for(done, timeout, start)
        self.b.idle()
        return self.b.buf[start:]

    def cmd(self, name, cmd, done, timeout):
        """Run cmd, return its output once regex 'done' shows up (or timeout)."""
        return self.finish(self.start(name, cmd), done, timeout)

    def check(self, out, rule):
        """Errors of the last command, the uart ones and the wakeup counts.

        Every test must start above the last count of the previous one. 'same'
        means the test must not sleep, 'rising' means each callback woke up.
        """
        err = list(self.errors)
        counts = [int(c) for c in WAKEUP.findall(out)]
        if not counts:
            return [*err, "no wakeup_count"]
        if self.wakeups and counts[0] <= self.wakeups[-1]:
            err.append(f"wakeup_count {counts[0]} not above {self.wakeups[-1]}")
        if rule == "same" and len(set(counts)) != 1:
            err.append(f"wakeup_count changed {counts}")
        if rule == "rising" and any(b <= a for a, b in zip(counts, counts[1:], strict=False)):
            err.append(f"wakeup_count not rising {counts}")
        self.wakeups += counts
        return err


def result_of(out, test):
    m = re.search(rf"{test} test (PASS|FAIL): ([^\n]*)", out)
    return (m.group(1) == "PASS", m.group(2)) if m else (False, "no result line")


def judge(rep, sh, item, test, out, rule="any"):
    ok, detail = result_of(out, test)
    err = sh.check(out, rule)
    if err:
        ok, detail = False, "; ".join([detail, *err])
    rep.add(item, ok, detail)


class Peer:
    """Peer board running the same test, its shell must stay awake (no PM)."""

    def __init__(self, board):
        self.b = board

    def send(self, cmd):
        start = len(self.b.buf)
        self.b.note(cmd)
        self.b.send(CTRL_U + cmd + "\r")
        self.b.wait_for(re.escape(cmd), 2, start)


def pwm_check(rep, sh, item, out, waves):
    """pwm/lppwm only check the API calls, the waveform needs an LA."""
    got = [(int(p), int(q)) for p, q in re.findall(r"\[period\]: (\d+), \[pulse\]: (\d+)", out)]
    err = sh.check(out, "any")
    rep.add(item, got == waves and not err, "; ".join([str(got), *err]))


def uart_wakeup(rep, sh):
    """Wake the sleeping UART up with the probe, then send an empty line."""
    start = sh.start("uart wakeup", "")
    err = sh.errors
    if not sh.b.wait_for(r"uart:~\$", 1, start):
        err.append("no prompt after the wakeup frame")
    sh.b.idle()
    rep.add("uart wakeup", not err, "; ".join(err) or "probe dropped, prompt answered")


def gaps_check(rep, item, out, pattern, lo, hi):
    ts = [int(t) for t in re.findall(rf"\[(\d+)\] {pattern}", out)]
    gaps = [b - a for a, b in zip(ts, ts[1:], strict=False)]
    if not all(lo <= g <= hi for g in gaps):
        rep.add(item + " interval", False, f"{gaps} ms, {lo}..{hi} expected")


def phase_pm(args, rep):
    log = args.outdir / "build.log"
    src = args.topdir / PM_TEST
    build = args.outdir / "bee_pm_device"
    peer_build = args.outdir / "peer_build"
    if not args.no_flash:
        ok = west_build(args.topdir, build, args.board, src, log) and west_flash(
            args.topdir, build, args.port, log
        )
        # The peer image is flashed beforehand with its own tool, only build
        # it here for its console baud
        if ok and args.peer_port:
            ok = west_build(args.topdir, peer_build, args.peer_board, src, log)
        if not ok:
            rep.add("bee_pm_device", False, f"build/flash failed, see {log}")
            return

    b = Board("dut", args.port, console_baud(build), args.outdir / "dut.log")
    peer = None
    if args.peer_port:
        peer = Peer(
            Board("peer", args.peer_port, console_baud(peer_build), args.outdir / "peer.log")
        )
    sh = DutShell(b)
    try:
        b.reset()
        if not b.wait_for(r"uart:~\$", 5):
            rep.add("bee_pm_device", False, f"no shell prompt on {args.port}")
            return
        sh.check(b.buf, "any")

        out = sh.cmd("shell_uart_put", "pm_test shell_uart_put", r"handed back", 3)
        m = re.search(r"[^\n]*handed back[^\n]*", out)
        rep.add(
            "shell_uart_put",
            "handed back, type on shell to wake up" in out,
            m.group(0).strip() if m else "no reply",
        )
        sh.uart_pm = True
        # Every command below goes through this wakeup
        uart_wakeup(rep, sh)

        # No wiring
        out = sh.cmd("counter", "pm_test counter 2000", r"counter test (PASS|FAIL)", 5)
        judge(rep, sh, "counter", "counter", out, "same")
        out = sh.cmd("rtc", "pm_test rtc", r"rtc test (PASS|FAIL)", 5)
        judge(rep, sh, "rtc", "rtc", out, "rising")
        out = sh.cmd("pwm", "pm_test pwm", r"(?s)(\[PWM\].*?){4}\n", 5)
        pwm_check(rep, sh, "pwm (log only)", out, [(50000, 10000), (0, 0), (50000, 40000), (0, 0)])
        out = sh.cmd("lppwm", "pm_test lppwm", r"(?s)(\[PWM\].*?){4}\n", 5)
        pwm_check(rep, sh, "lppwm (log only)", out, [(160, 32), (0, 0), (160, 128), (0, 0)])
        out = sh.cmd("can", "pm_test can", r"can test (PASS|FAIL)", 5)
        judge(rep, sh, "can", "can", out)

        # Loopback wires on the board under test, the second adc sample comes
        # after a DLPS round
        out = sh.cmd("adc", "pm_test adc", r"adc test (PASS|FAIL)", 5)
        judge(rep, sh, "adc", "adc", out, "rising")
        out = sh.cmd("spi", "pm_test spi", r"spi test (PASS|FAIL)", 5)
        judge(rep, sh, "spi", "spi", out)
        out = sh.cmd("latch", "pm_test counter latch", r"latch test (PASS|FAIL)", 10)
        judge(rep, sh, "latch", "latch", out)

        # Wired to the peer board
        out = sh.cmd("i2c", "pm_test i2c", r"i2c test (PASS|FAIL)", 5)
        judge(rep, sh, "i2c", "i2c", out)
        if peer:
            # 3s per round, so 4 rounds without edges end before the 20s wait
            start = sh.start("gpio", "pm_test gpio 10 3000")
            time.sleep(KEEP_ACTIVE_S)
            peer.send("pm_test waveform_gpio 10")
            out = sh.finish(start, r"gpio test (PASS|FAIL)", 20)
            judge(rep, sh, "gpio", "gpio", out, "rising")
            gaps_check(rep, "gpio", out, r"enter gpio callback", 700, 1300)
            peer.b.idle(timeout=10)

            start = sh.start("qdec", "pm_test qdec")
            time.sleep(KEEP_ACTIVE_S)
            peer.send("pm_test waveform_qdec")
            out = sh.finish(start, r"qdec test (PASS|FAIL)", 15)
            judge(rep, sh, "qdec", "qdec", out, "rising")
    finally:
        b.close()
        if peer:
            peer.b.close()


def main():
    top = Path(subprocess.check_output(["west", "topdir"], text=True).strip())
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        allow_abbrev=False,
    )
    ap.add_argument("--board", default="rtl87x2j_evb/rtl8762jth", help="board under test")
    ap.add_argument("--port", default="/dev/ttyUSB0", help="flash/console port of the board")
    ap.add_argument(
        "--peer-board",
        default="rtl8752h_evb/rtl8752hjl",
        help="board that drives the gpio/qdec waveforms",
    )
    ap.add_argument("--peer-port", help="console port of the peer, gpio/qdec need it")
    ap.add_argument("--phases", default="twister,uart,pm")
    ap.add_argument(
        "--no-flash", action="store_true", help="pm phase: use the image already on the board"
    )
    ap.add_argument(
        "--outdir",
        type=Path,
        default=top / "bee_auto_test",
        help="logs, twister-out and the uart_basic_api build",
    )
    args = ap.parse_args()
    args.topdir = top
    args.outdir.mkdir(parents=True, exist_ok=True)

    rep = Report()
    phases = {"twister": phase_twister, "uart": phase_uart, "pm": phase_pm}
    for name in args.phases.split(","):
        phases[name](args, rep)

    failed = rep.failed()
    print(f"\n{len(rep.items)} items, {len(failed)} failed, logs in {args.outdir}")
    for num, name, _, detail in failed:
        print(f"  FAIL {num} {name}: {detail}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
