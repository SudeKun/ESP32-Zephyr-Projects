# ESP32 Zephyr Projects

This repository contains multiple sample projects for ESP32 boards using the Zephyr RTOS. Each project demonstrates different features such as WiFi, Bluetooth, sensors, and secure boot with MCUboot.

## Project Structure

- `mcuboot/` - Secure bootloader configuration and sources
- `sensor/` - Sensor demo application
- `smp/` - Simple Management Protocol (SMP) demo (Bluetooth, WiFi, sensor)
- `wifi/` - WiFi demo application

Each project contains its own source code, configuration files, and build directories.

## Getting Started

### Prerequisites

- ESP32 development board (e.g., ESP32 DevKitC, ESP32 Wrover)
- Zephyr SDK and toolchain installed
- CMake, west, and Python 3

### Build Instructions

1. Clone the repository:
   ```bash
   git clone https://github.com/SudeKun/ESP32-Zephyr-Projects.git
   cd ESP32-Zephyr-Projects
   ```

2. Initialize Zephyr (if not done):
   ```bash
   west init -l .
   west update
   ```

3. Build a sample project (e.g., wifi):
   ```bash
   cd wifi
   west build -b esp32_devkitc_wrover
   ```

4. Flash to your board:
   ```bash
   west flash
   ```

## Project Details

- **mcuboot**: Secure bootloader for ESP32
- **sensor**: Reads sensor data and outputs via UART
- **smp**: Demonstrates Bluetooth, WiFi, and sensor management
- **wifi**: Connects ESP32 to WiFi and runs HTTP/ping demos

## License

**Attribution:**
If you use this project, please credit the author: SudeKun.

## Contributing

Pull requests and issues are welcome!
