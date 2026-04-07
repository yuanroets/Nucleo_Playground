#!/usr/bin/env python3
"""SEGGER RTT data capture from ST-Link using pyocd.

Reads RTT output from the STM32 and splits the stream:
  - IMU lines  (prefix "T") -> stdout  -> piped into imu_live_plot_logger.py
  - All other lines         -> stderr  -> printed in your terminal

This means barometer readings (prefix "P"), boot messages, sensor health
summaries, and any error lines all appear in the terminal while the plotter
only receives the IMU CSV data it expects.

Usage:
    python3 tools/rtt_capture.py | python3 tools/imu_live_plot_logger.py

Prerequisites:
    pip install pyocd
"""

import sys
import time
import logging
from pyocd.core.helpers import ConnectHelper


# Firmware sends IMU CSV lines with this prefix.
# Everything else is treated as diagnostic text for the terminal.
IMU_PREFIX = "T,"


def setup_logging():
    """Minimal logging to stderr (stdout reserved for IMU CSV data)."""
    logging.basicConfig(
        level=logging.INFO,
        format="[%(levelname)s] %(message)s",
        stream=sys.stderr,
    )
    return logging.getLogger(__name__)


def find_and_connect(log):
    """Find and connect to the STM32 via ST-Link."""
    log.info("Scanning for ST-Link probes...")
    probes = ConnectHelper.get_all_connected_probes(blocking=False)
    if not probes:
        log.error("No ST-Link probes found. Check USB connection.")
        sys.exit(1)

    probe = probes[0]
    probe_name = getattr(probe, "product_name", getattr(probe, "description", "Unknown probe"))
    probe_sn = getattr(probe, "serial_number", "unknown")
    log.info(f"Found probe: {probe_name} (Serial: {probe_sn})")

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
    log.info("Halting target briefly to locate RTT control block...")

    target.halt()
    time.sleep(0.5)

    log.info("Resuming target execution...")
    target.resume()
    time.sleep(0.5)

    return session, target


def find_rtt_control_block(session, max_retries=5):
    """
    Locate the SEGGER RTT control block in SRAM by scanning for its
    magic ID string "SEGGER RTT".

    The control block sits at a fixed offset inside SRAM1 on the STM32L476.
    Its exact address depends on the linker, so we scan the first 96 KB.
    """
    log = logging.getLogger(__name__)

    rtt_id = b"SEGGER RTT"
    base_addr = 0x20000000
    search_size = 0x18000  # 96 KB covers all of SRAM1 on the L476

    log.info("Searching for RTT control block in SRAM...")

    for attempt in range(max_retries):
        try:
            data = bytes(session.target.read_memory_block8(base_addr, search_size))
            idx = data.find(rtt_id)
            if idx >= 0:
                rtt_addr = base_addr + idx
                log.info(f"RTT control block found at 0x{rtt_addr:08x}")
                return rtt_addr
            time.sleep(0.5)
        except Exception as e:
            log.warning(f"Attempt {attempt + 1}/{max_retries} failed: {e}")
            time.sleep(0.5)

    log.error("RTT control block not found. Is SEGGER_RTT_Init() called?")
    return None


def route_line(line):
    """
    Decide where each RTT line goes.

    Lines starting with "T," are IMU CSV rows intended for the plotter.
    They are written to stdout so the pipe to imu_live_plot_logger.py works.

    Every other line (barometer "P," rows, boot messages, health summaries,
    sensor errors) is written to stderr so it appears in the terminal.
    """
    stripped = line.rstrip("\r\n")
    if stripped.startswith(IMU_PREFIX):
        sys.stdout.write(line)
        sys.stdout.flush()
    else:
        # Print non-IMU lines to terminal with a clear prefix so they are easy
        # to spot and distinguish from pyocd log messages.
        print(f"[RTT] {stripped}", file=sys.stderr)


def stream_rtt_data(session, rtt_addr):
    """
    Read the RTT up-buffer in a polling loop and route each complete line.

    The RTT up-buffer is a circular FIFO in SRAM. The write position (Pos)
    is updated by the firmware after each SEGGER_RTT_Write / printf call.
    We read from our last known position to the current Pos on every poll,
    handle wrap-around, and then assemble complete newline-terminated lines
    before routing them.

    Control block layout (Cortex-M, 32-bit):
        +0   acID[16]           - "SEGGER RTT" magic string
        +16  MaxNumUpBuffers    - u32
        +20  MaxNumDownBuffers  - u32
        +24  aUp[0].pBuffer    - u32 pointer to ring buffer in SRAM
        +28  aUp[0].BufferSize - u32 total buffer capacity in bytes
        +32  aUp[0].WrOff      - u32 current write offset (firmware side)
        +36  aUp[0].RdOff      - u32 current read offset  (host side)
    """
    log = logging.getLogger(__name__)
    log.info("Starting RTT stream. IMU data -> plotter, everything else -> terminal.")
    log.info("Press Ctrl+C to stop.\n")

    target = session.target

    def read_u32(addr):
        return target.read_memory_block32(addr, 1)[0]

    up_buf_addr = read_u32(rtt_addr + 24)
    up_buf_size = read_u32(rtt_addr + 28)
    last_pos    = read_u32(rtt_addr + 32)

    if up_buf_addr == 0 or up_buf_size == 0:
        log.error("RTT up-buffer is not initialised. Has the firmware started?")
        return

    log.info(f"RTT up-buffer: address=0x{up_buf_addr:08x}  size={up_buf_size} bytes")

    # Partial line buffer: collects bytes until a newline is received.
    line_buf = ""

    try:
        while True:
            try:
                time.sleep(0.02)  # 20 ms poll -> up to 50 polls/s, low CPU load
                pos = read_u32(rtt_addr + 32)

                if pos == last_pos:
                    continue

                if pos > up_buf_size:
                    log.warning(f"RTT write offset {pos} exceeds buffer size, resetting.")
                    last_pos = 0
                    continue

                # Read new bytes from the circular buffer, handling wrap-around.
                if pos > last_pos:
                    chunk = bytes(
                        target.read_memory_block8(up_buf_addr + last_pos, pos - last_pos)
                    )
                else:
                    # Buffer has wrapped: read tail then head.
                    tail = bytes(
                        target.read_memory_block8(up_buf_addr + last_pos, up_buf_size - last_pos)
                    )
                    head = bytes(
                        target.read_memory_block8(up_buf_addr, pos)
                    )
                    chunk = tail + head

                last_pos = pos

                if not chunk:
                    continue

                # Decode and split into lines. Keep any incomplete tail in line_buf.
                text = chunk.decode("utf-8", errors="ignore")
                line_buf += text

                # Process every complete line (terminated by \n).
                while "\n" in line_buf:
                    line, line_buf = line_buf.split("\n", 1)
                    route_line(line + "\n")

            except KeyboardInterrupt:
                raise
            except Exception as e:
                log.warning(f"RTT read error: {e}")
                time.sleep(0.5)

    except KeyboardInterrupt:
        # Flush any partial line sitting in the buffer.
        if line_buf.strip():
            route_line(line_buf)
        log.info("RTT stream stopped.")


def main():
    log = setup_logging()

    try:
        # Step 1: connect over SWD through ST-Link.
        session, target = find_and_connect(log)

        # Step 2: locate SEGGER RTT control block created by firmware.
        rtt_addr = find_rtt_control_block(session)
        if rtt_addr:
            # Step 3: stream and route lines to plotter (stdout) or terminal (stderr).
            stream_rtt_data(session, rtt_addr)
        else:
            log.error("Cannot start stream without a valid RTT control block.")
            log.error("Check: SEGGER_RTT_Init() is called, firmware is running,")
            log.error("and the device is not stuck in Error_Handler().")
            sys.exit(1)

    except Exception as e:
        log.error(f"Fatal error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()