#include "torque_sensor/TorqueSensor.h"
#include <iostream>
#include <vector>
#include <algorithm>

namespace torque_sensor {

TorqueSensor::TorqueSensor(const std::string& port_name, uint32_t baudrate, TorqueSensorType type, float zero_voltage, float max_voltage)
    : port_name_(port_name), baudrate_(baudrate), type_(type), zero_voltage_(zero_voltage), max_voltage_(max_voltage), torque_value_(0.0) {
    serial_ = std::make_unique<serial::Serial>();
}

TorqueSensor::~TorqueSensor() {
    disconnect();
}

bool TorqueSensor::connect() {
    try {
        serial_->setPort(port_name_);
        serial_->setBaudrate(baudrate_);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial_->setTimeout(to);
        serial_->open();

        if (serial_->isOpen()) {
            std::cout << "TORQUE Serial Port " << port_name_ << " initialized successfully" << std::endl;
            return true;
        } else {
            std::cerr << "TORQUE serial port not opened." << std::endl;
            return false;
        }
    } catch (const std::exception& e) {
        std::cerr << "Unable to open TORQUE port: " << e.what() << std::endl;
        return false;
    }
}

void TorqueSensor::disconnect() {
    if (serial_ && serial_->isOpen()) {
        serial_->close();
    }
}

bool TorqueSensor::isConnected() const {
    return serial_ && serial_->isOpen();
}

int16_t TorqueSensor::hex2dec(uint16_t hexData) {
    return (int16_t)hexData;
}

bool TorqueSensor::update() {
    if (!isConnected()) return false;

    try {
        size_t n = serial_->available();
        if (n >= 4) {
            // Use vector to safely handle dynamic size, avoiding stack overflow
            std::vector<uint8_t> buffer(n); 
            size_t bytes_read = serial_->read(buffer.data(), n);

            if (bytes_read < 4) return false;
            
            // Iterate through buffer to find the packet
            // Packet format: 0x03 (Header) | Data1 | Data2 | 0x0D (Footer)
            // We search backwards to find the most recent complete packet if multiple exist, 
            // or we can search forwards. The original code searched forwards and took the first one?
            // Actually the original code:
            // for (int i = 0; i <= n - 4; ++i) { if match break; }
            // This takes the FIRST valid packet in the buffer.
            
            bool found = false;
            for (size_t i = 0; i <= bytes_read - 4; ++i) {
                if (buffer[i] == 0x03 && buffer[i + 3] == 0x0d) {
                    uint8_t usb_torque_data1 = buffer[i + 1];
                    uint8_t usb_torque_data2 = buffer[i + 2];
                    
                    uint16_t combined = (uint16_t)usb_torque_data1 | ((uint16_t)usb_torque_data2 << 8);
                    
                    // Convert raw 16-bit integer to Voltage
                    // Assuming raw 32768 corresponds to max_voltage_ (e.g., 10V)
                    float current_voltage = ((float)hex2dec(combined) / 32768.0f) * max_voltage_;

                    float max_torque = (type_ == TorqueSensorType::RANGE_30NM) ? 30.0f : 100.0f;
                    
                    // Linear Mapping: 
                    // Slope = MaxTorque / (MaxVoltage - ZeroVoltage)
                    // Torque = (Voltage - ZeroVoltage) * Slope
                    // Note: This assumes symmetric mapping around ZeroVoltage
                    float calculated_torque = 0.0f;
                    if (max_voltage_ != zero_voltage_) {
                        float slope = max_torque / (max_voltage_ - zero_voltage_);
                        calculated_torque = (current_voltage - zero_voltage_) * slope;
                    }

                    {
                        std::lock_guard<std::mutex> lock(mutex_);
                        torque_value_ = calculated_torque;
                    }
                    
                    found = true;
                    // Original code used 'break', so it processes the first packet found.
                    // We stick to that behavior to be safe, though processing the last might be fresher.
                    // But with high baudrate and polling, maybe it's fine.
                    break; 
                }
            }
            return found;
        }
    } catch (const std::exception& e) {
        std::cerr << "Error reading torque sensor: " << e.what() << std::endl;
    }
    return false;
}

float TorqueSensor::getTorque() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return torque_value_;
}

} // namespace torque_sensor
