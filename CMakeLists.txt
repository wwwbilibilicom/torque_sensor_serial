cmake_minimum_required(VERSION 3.10)
project(serial_torque_sensor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 包含目录
include_directories(include)

# --------------------------------------------------------
# Build 'serial' library from 3rdparty sources
# --------------------------------------------------------
set(SERIAL_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/serial)
include_directories(${SERIAL_SOURCE_DIR}/include)

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

add_library(serial STATIC ${SERIAL_SRCS})
target_link_libraries(serial ${SERIAL_LIBS})
# --------------------------------------------------------

# --------------------------------------------------------
# Library for Torque Sensor (包含解析逻辑和逻辑类)
# --------------------------------------------------------
add_library(torque_sensor STATIC)

# 将源文件添加到库中
target_sources(torque_sensor PRIVATE
    src/torque_sensor/TorqueSensor.cpp
    src/torque_sensor/SerialParser.cpp  # <-- 新增这一行
)

# 确保头文件路径正确
target_include_directories(torque_sensor PUBLIC include)

# 链接 serial 库
target_link_libraries(torque_sensor PUBLIC serial)
# --------------------------------------------------------

# Main Executable
add_executable(torque_sensor_node src/main.cpp)
target_link_libraries(torque_sensor_node torque_sensor)