#!/usr/bin/env python3
"""Simple IMU live plot + CSV logger reading from stdin.

Expected line format from RTT capture:
T,t_ms,ax,ay,az,gx,gy,gz

Usage:
    python3 tools/rtt_capture.py | python3 tools/imu_live_plot_logger.py

Or from a named pipe:
    cat /tmp/imu_stream | python3 tools/imu_live_plot_logger.py
"""

import sys
import csv
import datetime as dt
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def parse_line(line):
    parts = line.strip().split(",")
    if len(parts) != 8 or parts[0] != "T":
        return None

    try:
        t_ms = int(parts[1])
        ax = int(parts[2])
        ay = int(parts[3])
        az = int(parts[4])
        gx = int(parts[5])
        gy = int(parts[6])
        gz = int(parts[7])
    except ValueError:
        return None

    return (t_ms, ax, ay, az, gx, gy, gz)


def main():
    # Visible time window in seconds for the scrolling plot.
    window_seconds = 5.0
    # Sampling rate from firmware configuration (BMI270 ODR 200 Hz).
    sample_rate_hz = 200.0
    # Number of samples to retain in the rolling window.
    window_size = int(window_seconds * sample_rate_hz)

    update_interval = 50  # milliseconds
    csv_file = None

    # Setup CSV logger if requested
    csv_writer = None
    if csv_file:
        try:
            f = open(csv_file, "w", newline="")
            csv_writer = csv.writer(f)
            csv_writer.writerow(["timestamp_ms", "ax", "ay", "az", "gx", "gy", "gz"])
        except IOError as e:
            print(f"Warning: Could not open CSV file: {e}", file=sys.stderr)

    # Data buffers for live plotting
    data = {
        "t": deque(maxlen=window_size),
        "ax": deque(maxlen=window_size),
        "ay": deque(maxlen=window_size),
        "az": deque(maxlen=window_size),
        "gx": deque(maxlen=window_size),
        "gy": deque(maxlen=window_size),
        "gz": deque(maxlen=window_size),
    }

    # Conversion factor from raw accelerometer LSB to g-units.
    # From firmware: ±16g maps to ~±32768 LSB -> 32768 / 16 = 2048 LSB per g.
    acc_lsb_per_g = 2048.0

    # Setup matplotlib
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))
    fig.suptitle("IMU Live Plot (6-Axis: Accel + Gyro)")

    # Accel subplot
    ax_accel = axes[0]
    (line_ax,) = ax_accel.plot([], [], label="ax", color="red")
    (line_ay,) = ax_accel.plot([], [], label="ay", color="green")
    (line_az,) = ax_accel.plot([], [], label="az", color="blue")
    ax_accel.set_ylabel("Accel (g, ±16g)")
    ax_accel.set_ylim(-16.0, 16.0)
    ax_accel.legend(loc="upper right")
    ax_accel.grid(True)

    # Gyro subplot
    ax_gyro = axes[1]
    (line_gx,) = ax_gyro.plot([], [], label="gx", color="red")
    (line_gy,) = ax_gyro.plot([], [], label="gy", color="green")
    (line_gz,) = ax_gyro.plot([], [], label="gz", color="blue")
    ax_gyro.set_xlabel("Time (s)")
    ax_gyro.set_ylabel("Gyro (LSB, ±2000dps)")
    ax_gyro.set_ylim(-40000, 40000)
    ax_gyro.legend(loc="upper right")
    ax_gyro.grid(True)

    def on_new_data(line_data):
        """Called when new data arrives from stdin."""
        parsed = parse_line(line_data)
        if parsed is None:
            return

        t_ms, ax, ay, az, gx, gy, gz = parsed

        # Convert timestamp from milliseconds to seconds for real-time x-axis.
        t_s = t_ms / 1000.0

        # Convert accelerometer readings from raw LSB to g-units.
        ax_g = ax / acc_lsb_per_g
        ay_g = ay / acc_lsb_per_g
        az_g = az / acc_lsb_per_g

        # Append to buffers using real time (seconds) on the x-axis.
        data["t"].append(t_s)
        data["ax"].append(ax_g)
        data["ay"].append(ay_g)
        data["az"].append(az_g)
        data["gx"].append(gx)
        data["gy"].append(gy)
        data["gz"].append(gz)

        # Log to CSV
        if csv_writer:
            csv_writer.writerow([t_ms, ax, ay, az, gx, gy, gz])

    def update_plot(frame):
        """Update plot with current data."""
        if len(data["t"]) == 0:
            return line_ax, line_ay, line_az, line_gx, line_gy, line_gz

        x_data = list(data["t"])

        # Update accel lines
        line_ax.set_data(x_data, list(data["ax"]))
        line_ay.set_data(x_data, list(data["ay"]))
        line_az.set_data(x_data, list(data["az"]))

        # Update gyro lines
        line_gx.set_data(x_data, list(data["gx"]))
        line_gy.set_data(x_data, list(data["gy"]))
        line_gz.set_data(x_data, list(data["gz"]))

        # Auto-scale x-axis in time (seconds) with a sliding window.
        if len(x_data) > 0:
            max_t = x_data[-1]
            min_t = max(0.0, max_t - window_seconds)
            ax_accel.set_xlim(min_t, max_t)
            ax_gyro.set_xlim(min_t, max_t)

        return line_ax, line_ay, line_az, line_gx, line_gy, line_gz

    # Start reading from stdin in background
    import threading

    def stdin_reader():
        """Read lines from stdin and update data."""
        try:
            for line in sys.stdin:
                on_new_data(line)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(f"Error reading stdin: {e}", file=sys.stderr)

    reader_thread = threading.Thread(target=stdin_reader, daemon=True)
    reader_thread.start()

    # Create animation
    ani = FuncAnimation(
        fig, update_plot, interval=update_interval, blit=False, cache_frame_data=False
    )

    print("Live IMU plot starting. Reading from stdin...", file=sys.stderr)
    print("Format: T,timestamp_ms,ax,ay,az,gx,gy,gz", file=sys.stderr)
    try:
        plt.show()
    except KeyboardInterrupt:
        print("\nPlot closed.", file=sys.stderr)
    finally:
        if csv_file and csv_writer:
            f.close()


if __name__ == "__main__":
    main()
