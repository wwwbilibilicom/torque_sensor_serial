# Serial Torque Sensor Driver

A lightweight C++ driver and application for reading data from a serial torque sensor. This project provides a simple API to interface with the hardware and includes a standalone executable node for data visualization.

## Features

- **Robust Serial Communication:** Built on top of the cross-platform [wjwwood/serial](https://github.com/wjwwood/serial) library.
- **Simple C++ API:** Clean `TorqueSensor` class for easy integration into other C++ projects.
- **Minimal Dependencies:** Self-contained build using CMake (serial library bundled).
- **Real-time Reading:** Efficient polling loop with configurable baud rate.

## Prerequisites

*   **Operating System:** Linux (Recommended)
*   **Compiler:** C++17 compatible compiler (GCC/Clang)
*   **Build System:** CMake 3.10 or higher

## Building the Project

1.  Clone the repository:
    ```bash
    git clone https://github.com/your-username/serial_torque_sensor.git
    cd serial_torque_sensor
    ```

2.  Create a build directory:
    ```bash
    mkdir build
    cd build
    ```

3.  Configure and build:
    ```bash
    cmake ..
    make
    ```

## Usage

### Running the Example Node

After building, you can run the example executable to start printing torque values to the terminal:

```bash
./build/torque_sensor_node
```

The output will look like:
```text
Initializing Torque Sensor on /dev/serial/by-id/... at 256000 baud...
Connected! Reading data (Press Ctrl+C to stop)...
Torque: 0.123
Torque: 0.125
...
```

### Configuration (Serial Port)

⚠️ **Important:** The serial port path is currently configured in `src/main.cpp`.

By default, it looks for:
```cpp
std::string port = "/dev/serial/by-id/usb-Hiker_sudio_YK_COM_Port_6D7816855250Port-if00";
```

**To change the port:**
1.  Find your device's serial port (e.g., `/dev/ttyUSB0` or check `/dev/serial/by-id/`).
2.  Open `src/main.cpp`.
3.  Update the `port` variable in the `main` function.
4.  Rebuild the project (`make -C build`).

## Analog Protocol Mapping (模拟量映射协议)

The driver supports sensors with analog output mapped to a digital value. The mapping logic follows a linear rule based on the configured range and voltage parameters:

- **Sensor Types:** Supported ranges are `RANGE_30NM` (±30Nm) and `RANGE_100NM` (±100Nm).
- **Mapping Rule:** 
  - `Zero Voltage`: The voltage value representing 0 Nm (e.g., 5.0V).
  - `Max Voltage`: The voltage value representing the maximum positive range (e.g., 10.0V).
  - The torque is calculated linearly: `Torque = (CurrentVoltage - ZeroVoltage) * (MaxRange / (MaxVoltage - ZeroVoltage))`.

Example for a 30Nm sensor with 0-10V output:
- 0V = -30 Nm
- 5V = 0 Nm
- 10V = 30 Nm

## Project Structure

... (omitted for brevity) ...

## Integrating into Your Code

You can use the `TorqueSensor` class in your own applications:

```cpp
#include "torque_sensor/TorqueSensor.h"

// Configuration
std::string port = "/dev/ttyUSB0";
uint32_t baudrate = 256000;
auto type = torque_sensor::TorqueSensor::TorqueSensorType::RANGE_30NM;
float zero_v = 5.0f;
float max_v = 10.0f;

// Initialize with port, baudrate, type, zero_voltage, and max_voltage
torque_sensor::TorqueSensor sensor(port, baudrate, type, zero_v, max_v);

if (sensor.connect()) {
    while (true) {
        if (sensor.update()) {
            float torque = sensor.getTorque(); // Thread-safe access
            // Do something with torque...
        }
    }
}
```

## License

This project includes the `serial` library which is licensed under the MIT License.
Please refer to `3rdparty/serial/LICENSE` for details regarding the third-party code.
