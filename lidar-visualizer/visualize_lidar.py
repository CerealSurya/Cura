
import re
import math
import subprocess
import select
import pty
import os
import time

import matplotlib.pyplot as plt

PORT = "/dev/cu.usbserial-140"
BAUD = "460800"
MAX_DISTANCE = 1000
DISPLAY_INTERVAL = 0.1  # Update plot 10 times per second

master, slave = pty.openpty()

process = subprocess.Popen(
    [
        "./output/Darwin/Release/ultra_simple",
        "--channel", "--serial", PORT, BAUD
    ],
    stdout=slave,
    stderr=slave,
    close_fds=True
)
os.close(slave)

pattern = re.compile(
    r"theta:\s*([-+]?\d*\.?\d+)"
    r"\s+Dist:\s*([-+]?\d*\.?\d+)"
)

# Collect the current revolution separately from the displayed scan
current_scan = []
displayed_scan = []
previous_angle = None

buffer = ""
last_display = time.monotonic()

plt.ion()
fig, ax = plt.subplots(subplot_kw={"projection": "polar"})
points = ax.scatter([], [], s=4)

ax.set_theta_zero_location("N")
ax.set_theta_direction(-1)
ax.set_ylim(0, MAX_DISTANCE)
ax.set_title("RPLIDAR C1 — Latest Complete Revolution")
ax.set_rlabel_position(135)

plt.show(block=False)

try:
    while process.poll() is None and plt.fignum_exists(fig.number):
        ready, _, _ = select.select([master], [], [], 0.02)

        if ready:
            try:
                buffer += os.read(master, 8192).decode(
                    "utf-8", errors="ignore"
                )
            except OSError:
                break

            lines = buffer.split("\n")
            buffer = lines.pop()

            for line in lines:
                match = pattern.search(line)
                if not match:
                    continue

                angle = float(match.group(1))
                distance = float(match.group(2))

                # Detect the beginning of a new revolution
                if (
                    previous_angle is not None
                    and previous_angle > 300
                    and angle < 60
                ):
                    # The previous revolution is complete.
                    # Keep it displayed while collecting the next one.
                    if current_scan:
                        displayed_scan = current_scan

                    current_scan = []

                previous_angle = angle

                if 0 < distance <= MAX_DISTANCE:
                    current_scan.append(
                        (math.radians(angle), distance)
                    )

        now = time.monotonic()

        # Redraw only the last completed revolution
        if now - last_display >= DISPLAY_INTERVAL:
            if displayed_scan:
                points.set_offsets(displayed_scan)
                fig.canvas.draw_idle()

            fig.canvas.flush_events()
            last_display = now

        plt.pause(0.001)

except KeyboardInterrupt:
    print("Stopping scan...")

finally:
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()

    os.close(master)
    plt.close("all")
