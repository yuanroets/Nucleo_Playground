#!/usr/bin/env python3
"""SEGGER RTT data capture from ST-Link via OpenOCD and pipe to live IMU plotter.

This script:
1. Starts OpenOCD with ST-Link configuration
2. Connects to STM32 target over SWD
3. Reads SEGGER RTT data in real-time
4. Forwards formatted data to imu_live_plot_logger.py

Usage:
    python3 tools/rtt_to_plotter.py --port /tmp/openocd_fifo

Prerequisites:
    - openocd installed (brew install open-ocd on macOS)
    - STM32 code compiled and flashed with RTT enabled
    - ST-Link connected to Nucleo board via USB
"""

import subprocess
import sys
import time
import os
import argparse
from pathlib import Path

# OpenOCD configuration for ST-Link on Nucleo-L476RG
# This loads the interface and target scripts, then initializes SWD transport.
OPENOCD_CONFIG = """
source [find interface/stlink.cfg]
source [find target/stm32l4x.cfg]

init
"""

OPENOCD_COMMANDS = """
# Halt the target (non-invasive, doesn't affect normal execution)
halt

# Start RTT monitoring on channel 0
rtt setup 0x20000000 65536 "SEGGER RTT"
rtt start

# Resume target execution (code runs normally)
resume

# Keep connection alive
sleep 100000
"""


def find_openocd():
    """Find OpenOCD executable."""
    result = subprocess.run(["which", "openocd"], capture_output=True, text=True)
    if result.returncode == 0:
        return result.stdout.strip()
    return None


def start_openocd_rtt(openocd_path):
    """Start OpenOCD with RTT configuration."""
    print("[*] Starting OpenOCD with ST-Link...", file=sys.stderr)

    try:
        # Create temporary config file
        config_file = "/tmp/openocd_rtt.cfg"
        with open(config_file, "w") as f:
            f.write(OPENOCD_CONFIG)

        # Start OpenOCD and keep its output pipe open so we can inspect logs.
        proc = subprocess.Popen(
            [openocd_path, "-f", config_file, "-c", OPENOCD_COMMANDS],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )

        # Wait for OpenOCD to initialize
        time.sleep(2)

        print("[+] OpenOCD started, awaiting RTT data...", file=sys.stderr)
        return proc

    except Exception as e:
        print(f"[-] Failed to start OpenOCD: {e}", file=sys.stderr)
        return None


def capture_rtt_output(proc, timeout=30):
    """
    Capture RTT output from OpenOCD.

    Note: This is a placeholder approach. Real RTT data capture typically
    requires:
    - gdb with RTT support, OR
    - JLinkExe tool, OR
    - OpenOCD with RTT plugin (requires newer openocd build)

    For now, this script is intentionally documentation/scaffolding.
    It validates OpenOCD startup flow and explains why pyOCD is used for
    robust RTT byte-stream extraction in this repository.
    """
    print(
        "[!] RTT capture via OpenOCD requires additional setup.",
        file=sys.stderr,
    )
    print(
        "[!] Alternative: Use 'pyocd' with RTT support (pip install pyocd)",
        file=sys.stderr,
    )
    print(
        "[!] Or use 'JLinkExe' from SEGGER for professional RTT access.",
        file=sys.stderr,
    )


def main():
    parser = argparse.ArgumentParser(
        description="Capture SEGGER RTT data from ST-Link and forward to IMU plotter"
    )
    parser.add_argument(
        "--plotter-script",
        default="tools/imu_live_plot_logger.py",
        help="Path to IMU plotter script",
    )
    args = parser.parse_args()

    # 1) Resolve OpenOCD from PATH.
    openocd = find_openocd()
    if not openocd:
        print(
            "[-] openocd not found. Install with: brew install open-ocd",
            file=sys.stderr,
        )
        sys.exit(1)

    print(f"[+] Found OpenOCD at: {openocd}", file=sys.stderr)

    # 2) Start OpenOCD with a temporary config and RTT commands.
    proc = start_openocd_rtt(openocd)
    if not proc:
        sys.exit(1)

    try:
        # 3) Document current capture limitation and suggest supported options.
        capture_rtt_output(proc)

        # 4) Keep the OpenOCD process alive until user interrupts.
        proc.wait()

    except KeyboardInterrupt:
        print("\n[*] Stopping...", file=sys.stderr)
        proc.terminate()
        proc.wait(timeout=5)


if __name__ == "__main__":
    main()
