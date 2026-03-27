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
from pyocd.core.helpers import ConnectHelper


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
    probes = ConnectHelper.get_all_connected_probes(blocking=False)
    if not probes:
        log.error("No ST-Link probes found. Check USB connection.")
        sys.exit(1)

    probe = probes[0]
    probe_name = getattr(probe, "product_name", getattr(probe, "description", "Unknown probe"))
    probe_sn = getattr(probe, "serial_number", "unknown")
    log.info(f"Found probe: {probe_name} (Serial: {probe_sn})")

    # Create a proper pyOCD session with the first connected probe.
    session = ConnectHelper.session_with_chosen_probe(
        blocking=False,
        return_first=True,
    )
    if session is None:
        raise RuntimeError("Failed to create pyOCD session")

    session.open()
    target = session.target

    target_name = getattr(target, "part_number", getattr(target, "name", "unknown target"))
    log.info(f"Connected to {target_name}")
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

    # Find RTT control block by SEGGER ID string at acID[16].
    rtt_id = b"SEGGER RTT"
    base_addr = 0x20000000
    search_size = 0x18000  # Search first 96KB of SRAM1 (0x20000000-0x20017FFF)

    log.info("Searching for RTT control block...")

    for attempt in range(max_retries):
        try:
            # Read raw bytes and search for control block ID string.
            data = bytes(session.target.read_memory_block8(base_addr, search_size))
            idx = data.find(rtt_id)
            if idx >= 0:
                rtt_addr = base_addr + idx
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

    # Layout from this project's SEGGER_RTT.c on Cortex-M:
    # SEGGER_RTT_CB:
    #   +0  : acID[16]
    #   +16 : MaxNumUpBuffers (u32)
    #   +20 : MaxNumDownBuffers (u32)
    #   +24 : aUp[0].pBuffer (u32 ptr)
    #   +28 : aUp[0].BufferSize (u32)
    #   +32 : aUp[0].Pos (u32)
    up_pbuffer_off = 24
    up_size_off = 28
    up_pos_off = 32

    def read_u32(addr):
        return target.read_memory_block32(addr, 1)[0]

    up_buf_addr = read_u32(rtt_addr + up_pbuffer_off)
    up_buf_size = read_u32(rtt_addr + up_size_off)
    last_pos = read_u32(rtt_addr + up_pos_off)

    if up_buf_addr == 0 or up_buf_size == 0:
        log.error("RTT up-buffer is not initialized yet")
        return

    log.info(f"RTT up-buffer at 0x{up_buf_addr:08x}, size={up_buf_size}")

    try:
        while True:
            try:
                time.sleep(0.02)
                pos = read_u32(rtt_addr + up_pos_off)

                if pos == last_pos:
                    continue

                if pos > up_buf_size:
                    log.warning(f"Invalid RTT position {pos}, resetting read pointer")
                    last_pos = 0
                    continue

                if pos > last_pos:
                    chunk = bytes(target.read_memory_block8(up_buf_addr + last_pos, pos - last_pos))
                else:
                    # Wrap-around in circular buffer.
                    chunk1 = bytes(target.read_memory_block8(up_buf_addr + last_pos, up_buf_size - last_pos))
                    chunk2 = bytes(target.read_memory_block8(up_buf_addr, pos))
                    chunk = chunk1 + chunk2

                if chunk:
                    sys.stdout.write(chunk.decode("utf-8", errors="ignore"))
                    sys.stdout.flush()

                last_pos = pos

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
