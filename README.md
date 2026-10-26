# Cura
HaptiDAR

FRI Project w/ Surya Jalapati, Inaba Amanat, Afnaan Nusayr & Paul Tran 

# RPLIDAR C1 Visualizer

A Python tool for visualizing live distance measurements from the SLAMTEC RPLIDAR C1 using Matplotlib.

## Requirements

- Python 3
- SLAMTEC RPLIDAR C1 connected to your computer
- Python packages in `lidar-visualizer/requirements.txt`
- A C++ compiler and `make` to build the included SDK on macOS/Linux

## 1. Build the SDK

From the project root, run:

```bash
make -C rplidar_sdk
```

This builds the included SDK's `ultra_simple` executable:

- macOS: `rplidar_sdk/output/Darwin/Release/ultra_simple`
- Linux: `rplidar_sdk/output/Linux/Release/ultra_simple`

You do not need to clone or install the SDK separately.

## 2. Install Python dependencies

From the project root, run:

```bash
python3 -m pip install -r lidar-visualizer/requirements.txt
```

## 3. Configure and run the visualizer

Open `lidar-visualizer/visualize_lidar.py` and configure:

- `PORT`: your lidar's serial port
- `BAUD`: serial baud rate; this project uses `460800` for the C1
- `MAX_DISTANCE`: maximum plotted distance in millimeters

The script finds the SDK build relative to its own location, so it can be run from any working directory:

```bash
python3 lidar-visualizer/visualize_lidar.py
```

Connect the sensor before starting the program. Press `Ctrl+C` in the terminal to stop it.

## Notes

The SDK executable is generated locally by `make -C rplidar_sdk`; it is not stored in Git. The serial port depends on your operating system and hardware setup.
