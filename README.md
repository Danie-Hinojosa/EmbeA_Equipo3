# Embebidos Equipo 3 — Seeed XIAO ESP32C3 (Arduino/PlatformIO)

Embedded project targeting the Seeed XIAO ESP32C3 using the Arduino framework and PlatformIO. The codebase is primarily C++ and includes IMU handling logic and optional CAN bus support via a bundled MCP CAN library.

- Board: Seeed XIAO ESP32C3
- Framework: Arduino (PlatformIO)
- Languages: C++, C

## Table of contents
- [Project structure](#project-structure)
- [Hardware](#hardware)
- [Getting started](#getting-started)
- [Build and upload](#build-and-upload)
- [Serial monitor](#serial-monitor)
- [Configuration](#configuration)
- [IMU module](#imu-module)
- [CAN bus (optional)](#can-bus-optional)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## Project structure

```
EmbeA_Equipo3/
├─ .vscode/                # VS Code project settings (optional)
├─ include/                # Header files (if any; PlatformIO include path)
│  └─ README               # PlatformIO include/headers guide
├─ lib/
│  ├─ README               # PlatformIO local libraries guide
│  └─ mcp_can/             # Local MCP CAN library (MCP2515)
├─ src/
│  ├─ main.cpp             # Application entry point
│  └─ bno_imu.cpp          # IMU (BNO*) handling and helpers
├─ test/
│  └─ README               # PlatformIO tests guide
├─ platformio.ini          # PlatformIO environment configuration
└─ .gitignore
```

Key files:
- [platformio.ini](https://github.com/Danie-Hinojosa/EmbeA_Equipo3/blob/main/platformio.ini)
- [src/main.cpp](https://github.com/Danie-Hinojosa/EmbeA_Equipo3/blob/main/src/main.cpp)
- [src/bno_imu.cpp](https://github.com/Danie-Hinojosa/EmbeA_Equipo3/blob/main/src/bno_imu.cpp)

## Hardware

- Seeed XIAO ESP32C3 (USB-C)
- IMU sensor (handled in `src/bno_imu.cpp`)
- Optional: MCP2515-based CAN controller + CAN transceiver (library present in `lib/mcp_can`)

Wiring depends on your sensor and CAN module. Refer to the XIAO ESP32C3 pinout and verify pin assignments in the source files. I2C (for most IMUs) and SPI (for MCP2515) pins can be configured in code.

## Getting started

1. Install:
   - Visual Studio Code
   - PlatformIO IDE extension (or PlatformIO Core CLI)

2. Clone this repository:
   ```bash
   git clone https://github.com/Danie-Hinojosa/EmbeA_Equipo3.git
   cd EmbeA_Equipo3
   ```

3. Open the folder in VS Code (PlatformIO will auto-detect the project).

## Build and upload

The project is preconfigured for the XIAO ESP32C3 in `platformio.ini`:

```ini
[env:seeed_xiao_esp32c3]
platform = espressif32
board = seeed_xiao_esp32c3
framework = arduino
```

- VS Code: Use the PlatformIO toolbar (checkmark) to Build and (right arrow) to Upload.
- CLI:
  ```bash
  pio run -e seeed_xiao_esp32c3
  pio run -e seeed_xiao_esp32c3 -t upload
  ```

Connect the board via USB-C and ensure the correct serial port is selected in PlatformIO if upload fails.

## Serial monitor

- Use PlatformIO’s Monitor:
  ```bash
  pio device monitor
  ```
- Default baud rate is typically 115200 unless overridden in `src/main.cpp`. Adjust with:
  ```bash
  pio device monitor -b 115200
  ```

## Configuration

- Pin mappings, sensor settings, and runtime parameters are defined in `src/main.cpp` and `src/bno_imu.cpp`.
- If you add headers, place them under `include/` and include in your sources as needed.
- Additional or custom libraries can be placed under `lib/` (PlatformIO will pick them up automatically).

## IMU module

- IMU-related logic lives in [`src/bno_imu.cpp`](https://github.com/Danie-Hinojosa/EmbeA_Equipo3/blob/main/src/bno_imu.cpp).
- Typical usage:
  - Initialize the IMU in `setup()`
  - Read and process sensor data in `loop()` or via helper functions
- Check that I2C pins and IMU address are correct for your hardware. If needed, scan I2C to confirm connectivity.

## CAN bus (optional)

- The project includes a local `mcp_can` library under `lib/mcp_can` for MCP2515-based CAN controllers (SPI).
- Ensure:
  - Proper SPI wiring (SCK/MISO/MOSI/CS) to the XIAO ESP32C3
  - A CAN transceiver (e.g., TJA1050/MCP2551) is connected to the CANH/CANL bus
  - CAN bus termination (typically 120 Ω at each end)
- Configure the CS pin and bitrate in your code where the CAN interface is initialized.

## Troubleshooting

- Build fails: Confirm the environment `[env:seeed_xiao_esp32c3]` is selected and PlatformIO has installed `platform = espressif32`.
- Upload issues: 
  - Check cable and port, try entering boot mode (double-tap reset) on the XIAO ESP32C3 if needed.
  - On some systems, selecting the correct upload port in PlatformIO is required.
- IMU not detected:
  - Verify I2C wiring and address
  - Confirm sensor power and common ground
- CAN not working:
  - Verify SPI CS pin and wiring
  - Ensure transceiver present and bus termination correct
  - Match bitrate with the rest of the network

## Contributing

Issues and pull requests are welcome. For larger changes, please open an issue first to discuss what you would like to change. Follow typical PlatformIO project conventions:
- Put headers in `include/`
- Put source files in `src/`
- Place reusable libraries in `lib/`

## License

Add a LICENSE file to specify the project license. If none is provided, the project remains “All rights reserved” by default.
