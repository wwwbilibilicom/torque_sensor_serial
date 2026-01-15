cmake_minimum_required(VERSION 3.10)
project(serial_torque_sensor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories
include_directories(include)

# --------------------------------------------------------
# Build 'serial' library from 3rdparty sources
# --------------------------------------------------------
set(SERIAL_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/serial)
include_directories(${SERIAL_SOURCE_DIR}/include)

# Define serial sources explicitly to avoid catkin dependencies
set(SERIAL_SRCS
    ${SERIAL_SOURCE_DIR}/src/serial.cc
    ${SERIAL_SOURCE_DIR}/include/serial/serial.h
    ${SERIAL_SOURCE_DIR}/include/serial/v8stdint.h
)

if(UNIX)
    list(APPEND SERIAL_SRCS
        ${SERIAL_SOURCE_DIR}/src/impl/unix.cc
        ${SERIAL_SOURCE_DIR}/src/impl/list_ports/list_ports_linux.cc
    )
    set(SERIAL_LIBS rt pthread)
endif()

# Build serial as a static library
add_library(serial STATIC ${SERIAL_SRCS})
target_link_libraries(serial ${SERIAL_LIBS})
# --------------------------------------------------------

# Library for Torque Sensor
add_library(torque_sensor src/torque_sensor/TorqueSensor.cpp)
target_link_libraries(torque_sensor PUBLIC serial)

# Main Executable
add_executable(torque_sensor_node src/main.cpp)
target_link_libraries(torque_sensor_node torque_sensor)