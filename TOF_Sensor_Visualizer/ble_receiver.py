
import asyncio
import numpy as np
import matplotlib.pyplot as plt
from bleak import BleakClient, BleakScanner

SERVICE_UUID = "12345678-1234-1234-1234-123456789000"
CHARACTERISTIC_UUID = "12345678-1234-1234-1234-123456789001"

# Store incoming packets
frames = {}

# Create visualization
plt.ion()

fig, ax = plt.subplots(figsize=(8, 7))

initial_data = np.zeros((8, 8))

heatmap = ax.imshow(
    initial_data,
    cmap="turbo",
    interpolation="nearest",
    vmin=0,
    vmax=2000
)

colorbar = plt.colorbar(heatmap, ax=ax)
colorbar.set_label("Distance (mm)")

ax.set_title("VL53L5CX Live Distance Heatmap")
ax.set_xlabel("Sensor Column")
ax.set_ylabel("Sensor Row")

ax.set_xticks(range(8))
ax.set_yticks(range(8))

# Display distance values inside each cell
labels = []

for row in range(8):
    row_labels = []
    for col in range(8):
        label = ax.text(
            col, row, "0",
            ha="center",
            va="center",
            color="white",
            fontsize=9
        )
        row_labels.append(label)
    labels.append(row_labels)

fig.tight_layout()
plt.show(block=False)


def notification_handler(sender, data):

    if len(data) != 19:
        return

    # Decode packet header
    frame_number = data[0] | (data[1] << 8)
    chunk_number = data[2]

    if chunk_number >= 8:
        return

    # Decode eight 16-bit distance measurements
    distances = []

    for j in range(8):
        index = 3 + j * 2

        distance = data[index] | (data[index + 1] << 8)

        distances.append(distance)

    # Start or update the current frame
    if frame_number not in frames:
        frames[frame_number] = {}

    frames[frame_number][chunk_number] = distances

    # Wait until all eight packets arrive
    if len(frames[frame_number]) == 8:

        complete_frame = []

        for chunk in range(8):
            complete_frame.extend(frames[frame_number][chunk])

        # Convert to 8x8 matrix
        matrix = np.array(complete_frame).reshape(8, 8)

        # Update heatmap
        heatmap.set_data(matrix)

        for row in range(8):
            for col in range(8):
                labels[row][col].set_text(
                    str(matrix[row, col])
                )

        ax.set_title(
            f"VL53L5CX Live Distance Heatmap | Frame {frame_number}"
        )

        fig.canvas.draw_idle()

        print(f"Received frame {frame_number}")

        # Remove completed frame
        del frames[frame_number]

        # Remove stale incomplete frames
        for old_frame in list(frames):
            if old_frame != frame_number and (
                (frame_number - old_frame) % 65536
            ) > 5:
                del frames[old_frame]


async def main():

    print("Searching for ESP32...")

    devices = await BleakScanner.discover(
        timeout=10,
        return_adv=True
    )

    target = None

    for device, advertisement in devices.values():

        if SERVICE_UUID.lower() in [
            uuid.lower()
            for uuid in (advertisement.service_uuids or [])
        ]:
            target = device
            break

    if target is None:
        print("ESP32 not found!")
        return

    print("Found:", target.name)
    print("Connecting...")

    async with BleakClient(target) as client:

        print("Connected:", client.is_connected)

        await client.start_notify(
            CHARACTERISTIC_UUID,
            notification_handler
        )

        print("Receiving sensor measurements...")
        print("Close the visualization window to stop.")

        while plt.fignum_exists(fig.number) and client.is_connected:
            plt.pause(0.001)
            await asyncio.sleep(0.01)

        if client.is_connected:
            await client.stop_notify(CHARACTERISTIC_UUID)


asyncio.run(main())
