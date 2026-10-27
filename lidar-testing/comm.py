import serial
import time

PORT = "/dev/tty.usbserial-140" #!Wil need to chnage if on diff device
BAUD = 460800

OUTPUT_FILE = "lidar_polar.txt"

CMD_START_SCAN = bytes([0xA5, 0x20])
CMD_STOP_SCAN  = bytes([0xA5, 0x25])

DESCRIPTOR_SIZE = 7
NODE_SIZE = 5


def read_exact(ser, count):
    """Read exactly count bytes, unless the serial timeout expires."""
    result = bytearray()

    while len(result) < count:
        chunk = ser.read(count - len(result))

        if not chunk:
            break

        result.extend(chunk)

    return bytes(result)


def decode_nodes(data):
    """
    Decode standard 5-byte RPLIDAR measurement nodes.
    Returns (angle_degrees, distance_mm, quality).
    """
    measurements = []
    i = 0

    while i + NODE_SIZE <= len(data):
        b0, b1, b2, b3, b4 = data[i:i + NODE_SIZE]

        # Validate S and inverse-S flags.
        start_flag = b0 & 0x01
        inverse_flag = (b0 >> 1) & 0x01

        # The C check bit in byte 1 must be set.
        if start_flag == inverse_flag or (b1 & 0x01) != 1:
            # Slide forward one byte to find the next valid node.
            i += 1
            continue

        # Fixed-point angle, Q6: degrees = angle_q6 / 64.
        angle_q6 = (b1 >> 1) | (b2 << 7)
        angle_deg = angle_q6 / 64.0

        # Fixed-point distance, Q2: mm = distance_q2 / 4.
        distance_q2 = b3 | (b4 << 8)
        distance_mm = distance_q2 / 4.0

        quality = b0 >> 2

        # Reject out-of-range angles and invalid zero-distance samples.
        if angle_deg < 360.0 and distance_mm > 0:
            measurements.append(
                (angle_deg, distance_mm, quality)
            )
            i += NODE_SIZE
        else:
            i += 1

    return measurements


def main():
    raw_data = bytearray()
    ser = serial.Serial(
        PORT,
        BAUD,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1,
    )

    try:
        ser.reset_input_buffer()

        ser.write(CMD_START_SCAN)
        ser.flush()

        # The standard scan response starts with a 7-byte descriptor.
        descriptor = read_exact(ser, DESCRIPTOR_SIZE)
        print("Response descriptor:", descriptor.hex(" ").upper())

        if len(descriptor) != DESCRIPTOR_SIZE:
            raise RuntimeError("Incomplete scan response descriptor")

        if descriptor[:2] != bytes([0xA5, 0x5A]):
            raise RuntimeError(
                "Unexpected response descriptor; check the C1 scan mode"
            )

        print("Scanning. Press Ctrl-C to stop.")

        while True:
            chunk = ser.read(4096)
            if chunk:
                raw_data.extend(chunk)
                print(
                    f"\rCaptured {len(raw_data)} bytes",
                    end="",
                    flush=True,
                )

    except KeyboardInterrupt:
        print("\nStopping scan...")

    finally:
        try:
            ser.write(CMD_STOP_SCAN)
            ser.flush()
            time.sleep(0.2)
        finally:
            ser.close()

    measurements = decode_nodes(raw_data)

    with open(OUTPUT_FILE, "w") as output:
        output.write("# angle_deg distance_mm quality\n")

        for angle, distance, quality in measurements:
            output.write(
                f"{angle:.2f} {distance:.2f} {quality}\n"
            )

    print(f"Decoded {len(measurements)} measurements")
    print(f"Saved polar coordinates to {OUTPUT_FILE}")


if __name__ == "__main__":
    main()