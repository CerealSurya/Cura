
import serial
import numpy as np
import matplotlib.pyplot as plt
import time

# =====================================
# 1. ESP32 CONFIGURATION
# =====================================

SERIAL_PORT = "/dev/cu.usbserial-0001"
BAUD_RATE = 115200

ROWS = 4
COLS = 4

MAX_DISTANCE = 2000

# =====================================
# 2. CONNECT TO ESP32
# =====================================

ser = serial.Serial(
    SERIAL_PORT,
    BAUD_RATE,
    timeout=2
)

time.sleep(2)

print("Connected to ESP32")
print("Waiting for sensor data...")

# =====================================
# 3. CREATE HEATMAP
# =====================================

plt.ion()

fig, ax = plt.subplots(figsize=(8, 7))

grid = np.zeros((ROWS, COLS))

heatmap = ax.imshow(
    grid,
    cmap="inferno",
    interpolation="nearest",
    vmin=0,
    vmax=MAX_DISTANCE
)

colorbar = plt.colorbar(heatmap, ax=ax)
colorbar.set_label("Distance (mm)")

ax.set_title("VL53L5CX Live Heatmap")
ax.set_xlabel("Sensor Column")
ax.set_ylabel("Sensor Row")

ax.set_xticks(range(COLS))
ax.set_yticks(range(ROWS))

# =====================================
# 4. ADD NUMERICAL LABELS
# =====================================

labels = []

for row in range(ROWS):
    for col in range(COLS):

        label = ax.text(
            col,
            row,
            "",
            ha="center",
            va="center",
            color="white",
            fontsize=11
        )

        labels.append(label)

plt.tight_layout()

# =====================================
# 5. RECEIVE SENSOR DATA
# =====================================

frame = []

try:

    while plt.fignum_exists(fig.number):

        line = ser.readline().decode(
            "utf-8",
            errors="ignore"
        ).strip()

        if not line:
            continue

        # Convert each row into integers
        try:

            values = [
                int(x)
                for x in line.replace(",", " ").split()
            ]

        except ValueError:
            continue

        # Only accept rows containing 4 values
        if len(values) != COLS:
            continue

        frame.append(values)

        # Wait for all 4 rows
        if len(frame) == ROWS:

            grid = np.array(frame, dtype=float)

            frame = []

            # Ignore invalid measurements
            masked_grid = np.ma.masked_where(
                (grid <= 0) | (grid > 4000),
                grid
            )

            # Update colors
            heatmap.set_data(masked_grid)

            # Update numbers
            for row in range(ROWS):

                for col in range(COLS):

                    value = grid[row, col]

                    index = row * COLS + col

                    if 0 < value <= 4000:

                        labels[index].set_text(
                            str(int(value))
                        )

                    else:

                        labels[index].set_text("N/A")

            # Refresh visualization
            fig.canvas.draw_idle()
            fig.canvas.flush_events()

            plt.pause(0.001)

except KeyboardInterrupt:

    print("Stopping visualizer")

finally:

    ser.close()
    plt.close("all")
