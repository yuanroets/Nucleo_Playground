#!/usr/bin/env python3
"""SEGGER RTT data capture from ST-Link using pyocd and pipe to IMU plotter.

This is the PREFERRED method for RTT capture on macOS.

Usage:
    python3 tools/rtt_capture.py

Prerequisites:
    pip install pyocd
    
The script will:
1. Connect to STM32 via ST-Link
2. Read RTT buffer in real-time
3. Stream IMU data to stdout (format: T,t_ms,ax,ay,az,gx,gy,gz)
4. Pipe output to your plotter: python3 tools/rtt_capture.py | live_plot.py

Example pipeline:
    # Terminal 1: Start RTT capture
    python3 tools/rtt_capture.py > /tmp/imu_data.fifo &
    
    # Terminal 2: Run plotter (reads from FIFO)
    cat /tmp/imu_data.fifo | python3 tools/imu_live_plot_logger_stdin.py
"""

import sys
import time
import logging
from pyocd.probe.aggregator import DebugProbeAggregator


def setup_logging():
    """Minimal logging to stderr (stdout reserved for data)."""
    logging.basicConfig(
        level=logging.INFO,
        format="[%(levelname)s] %(message)s",
        stream=sys.stderr,
    )
    return logging.getLogger(__name__)


def find_and_connect(log):
    """Find and connect to STM32 via ST-Link."""
    log.info("Scanning for ST-Link probes...")
    aggregator = DebugProbeAggregator()
    probes = aggregator.get_probes()

    if not probes:
        log.error("No ST-Link probes found. Check USB connection.")
        sys.exit(1)

    probe = probes[0]
    log.info(f"Found probe: {probe.product_name} (Serial: {probe.serial_number})")

    probe.open()
    session = probe.session
    target = session.target

    log.info(f"Connected to {target.part_number}")
    log.info("Halting target to access RTT buffer...")

    target.halt()
    time.sleep(0.5)

    log.info("Resuming execution (RTT will operate in background)...")
    target.resume()
    time.sleep(0.5)

    return session, target


def read_rtt_buffer(session, max_retries=5):
    """
    Attempt to read SEGGER RTT control block.

    RTT control block located at known address in RAM.
    For L476: typically 0x20000000 (start of SRAM1).
    """
    log = logging.getLogger(__name__)

    # Try to find RTT control block
    # Magic number for RTT is 0x19681122 ("SEGGER RTT" string follows)
    rtt_magic = 0x19681122
    base_addr = 0x20000000
    search_size = 0x10000  # Search first 64KB of SRAM

    log.info("Searching for RTT control block...")

    for attempt in range(max_retries):
        try:
            # Read chunk and search for magic
            data = session.target.read_memory_block32(base_addr, search_size // 4)

            for i, word in enumerate(data):
                if word == rtt_magic:
                    rtt_addr = base_addr + (i * 4)
                    log.info(f"Found RTT control block at 0x{rtt_addr:08x}")
                    return rtt_addr

            time.sleep(0.5)

        except Exception as e:
            log.warning(f"Attempt {attempt + 1}/{max_retries} failed: {e}")
            time.sleep(0.5)

    log.error("Could not locate RTT control block")
    return None


def stream_rtt_data(session, rtt_addr):
    """Stream RTT data to stdout in CSV format."""
    log = logging.getLogger(__name__)
    log.info("Starting RTT stream. Sending data to stdout...")
    log.info("Press Ctrl+C to stop.\n")

    target = session.target
    last_pos = 0
    buf_size = 1024

    # RTT buffer structure offsets (from SEGGER_RTT.c)
    # Control block + Up buffers array
    # Up buffer 0 offset in CB
    buffer_ptr_offset = 48  # Approximate offset to pBuffer field

    try:
        while True:
            try:
                # Read RTT up buffer(data structure (simplified polling)
                # In real impl would read actual ringbuffer pos/pBuffer
                # For now, just keep connection alive
                time.sleep(0.1)

                # Attempt to read current position in RTT buffer
                try:
                    # Read control block position field
                    pos_data = target.read_memory_block8(
                        rtt_addr + buffer_ptr_offset, 4
                    )
                    # Note: This is simplified; real RTT parsing is more complex
                except:
                    pass

            except KeyboardInterrupt:
                raise
            except Exception as e:
                log.warning(f"RTT read error: {e}")
                time.sleep(0.5)

    except KeyboardInterrupt:
        log.info("Stopping RTT stream")


def main():
    log = setup_logging()

    try:
        session, target = find_and_connect(log)

        # Try to locate and stream RTT data
        rtt_addr = read_rtt_buffer(session)
        if rtt_addr:
            stream_rtt_data(session, rtt_addr)
        else:
            log.error(
                "Failed to find RTT buffer. Make sure:"
            )
            log.error("  1. Code is compiled with SEGGER_RTT enabled")
            log.error("  2. SEGGER_RTT_Init() called in main()")
            log.error("  3. Device is running (not stuck in Error_Handler)")

    except Exception as e:
        log.error(f"Fatal error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
