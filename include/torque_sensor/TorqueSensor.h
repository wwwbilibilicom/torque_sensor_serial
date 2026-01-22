#pragma once

#include <string>
#include <cstdint>
#include <memory>
#include <atomic>
#include <thread>
#include <serial/serial.h>
#include "iostream"
#include "SerialParser.h"

#define USB_TORQUE_SENSOR_FRAME_SIZE 4
namespace torque_sensor {

        
class FrequencyCaculator {
public:
    FrequencyCaculator() {
        start_ = std::chrono::high_resolution_clock::now();
    }

    void sampleFrequency() {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start_);
        start_ = end;
        if (duration.count() > 0) {
            frequency_hz_ = 1e6 / duration.count();
            //std::cout << duration.count() << std::endl;
            //std::cout << end.time_since_epoch().count() << std::endl;
        }
    }

	double getFrequency() const {
		return frequency_hz_;
	}


	private:
    std::chrono::high_resolution_clock::time_point start_;
	std::atomic<double> frequency_hz_ = 0.0;
};


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
     * @brief Open the serial port connection and start the background reading thread.
     * 
     * @return true If connection successful
     * @return false If connection failed
     */
    bool connect();

    /**
     * @brief Stop the reading thread and close the serial port connection.
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
     * @brief Get the latest torque value.
     * This method is thread-safe and lock-free.
     * 
     * @return float Torque value in Nm
     */
    float getTorque() const;
    float getFrequency() const;

private:
    /**
     * @brief Background loop for reading and parsing data.
     */
    void readingLoop();

    std::vector<uint8_t> read_buffer_;
    std::unique_ptr<SerialParser> parser_;
    void onFrameReceived(const SensorFrame& frame);
    static constexpr size_t MAX_READ_SIZE = 1024;

    std::string port_name_;
    uint32_t baudrate_;
    TorqueSensorType type_;
    float zero_voltage_;
    float max_voltage_;
    std::unique_ptr<serial::Serial> serial_;
    
    // High performance storage
    std::atomic<int16_t> raw_data_;

    // Threading and State
    std::atomic<bool> running_;
    std::thread reading_thread_;

    // Pre-computed constants
    float slope_;

    FrequencyCaculator frequency_calculator_;

    bool data_received_;
};

} // namespace torque_sensor
