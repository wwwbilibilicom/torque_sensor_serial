# Serial Torque Sensor

## Project Overview
This project implements a C++ interface for reading data from a serial torque sensor. It provides a library `libtorque_sensor` for communicating with the hardware and an example executable `torque_sensor_node` that continuously reads and displays torque values.

The project includes the `serial` library (a cross-platform C++ library for RS-232 serial ports) as a 3rd-party dependency, which is built as part of the project.

## Project Structure

```text
/home/fyk/lwb_ws/serial_torque_sensor/
├── CMakeLists.txt           # Main build configuration
├── src/
│   ├── main.cpp             # Main executable entry point
│   └── torque_sensor/
│       └── TorqueSensor.cpp # Implementation of the TorqueSensor class
├── include/
│   └── torque_sensor/
│       └── TorqueSensor.h   # Header file for the TorqueSensor class
└── 3rdparty/
    └── serial/              # Bundled 'serial' library source code
```

## Prerequisites
*   **OS:** Linux (Tested on Linux based on file paths like `/dev/serial/...`)
*   **Compiler:** C++17 compatible compiler (e.g., GCC, Clang)
*   **Build System:** CMake (minimum version 3.10)

## Building the Project

1.  Create a build directory:
    ```bash
    mkdir build
    cd build
    ```

2.  Configure the project with CMake:
    ```bash
    cmake ..
    ```

3.  Build the executables:
    ```bash
    make
    ```

## Running the Application

After building, the executable `torque_sensor_node` will be located in the `build/` directory.

```bash
./build/torque_sensor_node
```

**Note on Serial Port:**
The serial port path is currently **hardcoded** in `src/main.cpp` as:
`/dev/serial/by-id/usb-Hiker_sudio_YK_COM_Port_6D7816855250Port-if00`

If your device is connected to a different port (e.g., `/dev/ttyUSB0`), you will need to modify `src/main.cpp` and recompile.

## Development Conventions

*   **C++ Standard:** C++17.
*   **Naming:** snake_case for files, CamelCase for classes.
*   **Architecture:**
    *   `TorqueSensor` class handles all low-level serial communication and data parsing.
    *   `main.cpp` handles the application loop and signal handling (Ctrl+C).
*   **Dependencies:** The project avoids external package managers for the `serial` library by bundling the source in `3rdparty/serial` and building it statically via `CMakeLists.txt`.
