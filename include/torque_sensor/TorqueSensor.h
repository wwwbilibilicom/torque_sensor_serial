#pragma once

#include <string>
#include <cstdint>
#include <memory>
#include <mutex>
#include <serial/serial.h>

namespace torque_sensor {

class TorqueSensor {
public:
    /**
     * @brief Enum for Torque Sensor Range
     */
    enum class TorqueSensorType {
        RANGE_30NM,
        RANGE_100NM
    };

    /**
     * @brief Construct a new Torque Sensor object
     * 
     * @param port_name The serial port path (e.g., /dev/ttyUSB0)
     * @param baudrate The baudrate for communication (default: 256000)
     * @param type The sensor type (RANGE_30NM or RANGE_100NM)
     * @param zero_voltage The voltage value corresponding to 0 Nm (e.g., 5.0)
     * @param max_voltage The voltage value corresponding to max range (e.g., 10.0)
     */
    TorqueSensor(const std::string& port_name, uint32_t baudrate, TorqueSensorType type, float zero_voltage, float max_voltage);

    /**
     * @brief Destroy the Torque Sensor object and close connection
     */
    ~TorqueSensor();

    /**
     * @brief Open the serial port connection
     * 
     * @return true If connection successful
     * @return false If connection failed
     */
    bool connect();

    /**
     * @brief Close the serial port connection
     */
    void disconnect();

    /**
     * @brief Check if the serial port is open
     * 
     * @return true 
     * @return false 
     */
    bool isConnected() const;

    /**
     * @brief Read available data from serial port and update torque value
     * 
     * @return true If a valid data packet was found and parsed
     * @return false If no new valid data was processed
     */
    bool update();

    /**
     * @brief Get the last received torque value
     * 
     * @return float Torque value
     */
    float getTorque() const;

private:
    std::string port_name_;
    uint32_t baudrate_;
    TorqueSensorType type_;
    float zero_voltage_;
    float max_voltage_;
    std::unique_ptr<serial::Serial> serial_;
    float torque_value_;
    mutable std::mutex mutex_;

    /**
     * @brief Helper to convert hex data to signed integer
     */
    int16_t hex2dec(uint16_t hexData);
};

} // namespace torque_sensor
