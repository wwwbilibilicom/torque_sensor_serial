#include "torque_sensor/TorqueSensor.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <algorithm>
#include <cmath>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

// Include Linux serial headers for low latency
#if defined(__linux__)
#include <linux/serial.h>
#endif

namespace torque_sensor {

TorqueSensor::TorqueSensor(const std::string& port_name, uint32_t baudrate, TorqueSensorType type, float zero_voltage, float max_voltage)
    : port_name_(port_name), 
      baudrate_(baudrate), 
      type_(type), 
      zero_voltage_(zero_voltage), 
      max_voltage_(max_voltage), 
      raw_data_(0),
      running_(false),
      slope_(0.0f) {
    
    buffer_.reserve(1024);

    float max_torque = (type_ == TorqueSensorType::RANGE_30NM) ? 30.0f : 100.0f;
    if (std::abs(max_voltage_ - zero_voltage_) > 1e-6) {
        slope_ = max_torque / (max_voltage_ - zero_voltage_);
    }

    serial_ = std::make_unique<serial::Serial>();
}

TorqueSensor::~TorqueSensor() {
    disconnect();
}

bool TorqueSensor::connect() {
    try {
        serial_->setPort(port_name_);
        serial_->setBaudrate(baudrate_);
        // Set a short timeout for non-blocking feel in the thread
        // 10ms read timeout ensures we check 'running_' flag frequently if no data
        serial::Timeout to = serial::Timeout::simpleTimeout(10);
        serial_->setTimeout(to);
        serial_->open();

        if (serial_->isOpen()) {
            std::cout << "TORQUE Serial Port " << port_name_ << " initialized successfully" << std::endl;

#if defined(__linux__)
            // Enable Low Latency Mode on Linux
            // Since serial library doesn't expose FD, we open the port briefly to set the flag.
            // The flag is a device property, so it persists.
            int fd = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (fd != -1) {
                struct serial_struct ser_info;
                if (ioctl(fd, TIOCGSERIAL, &ser_info) == 0) {
                    ser_info.flags |= ASYNC_LOW_LATENCY;
                    if (ioctl(fd, TIOCSSERIAL, &ser_info) == 0) {
                        std::cout << "  -> Low Latency Mode ENABLED." << std::endl;
                    } else {
                        std::cerr << "  -> Failed to set Low Latency Mode." << std::endl;
                    }
                }
                close(fd);
            } else {
                std::cerr << "  -> Could not access port for Low Latency setup (Locked?)." << std::endl;
            }
#endif

            // Start background thread
            running_ = true;
            reading_thread_ = std::thread(&TorqueSensor::readingLoop, this);
            
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
    running_ = false;
    if (reading_thread_.joinable()) {
        reading_thread_.join();
    }
    
    if (serial_ && serial_->isOpen()) {
        serial_->close();
    }
}

bool TorqueSensor::isConnected() const {
    return serial_ && serial_->isOpen();
}

void TorqueSensor::readingLoop() {
    std::vector<uint8_t> temp_read_buf(1024);
    static int read_count_ = 0;
    while (running_) {
        try {
            size_t bytes_read = serial_->read(temp_read_buf.data(), temp_read_buf.size());
            read_count_ ++;
            int find_count_ = 0;
            if (bytes_read > 0) {
                // std::cout << "Recv: ";
                // for (size_t i = 0; i < bytes_read; ++i) {
                //     std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(temp_read_buf[i]) << " ";
                // }
                // std::cout << std::dec << std::endl;

                buffer_.insert(buffer_.end(), temp_read_buf.begin(), temp_read_buf.begin() + bytes_read);

                size_t process_idx = 0;
                // Use a loop index to scan the buffer instead of erasing from the front repeatedly
                // This reduces complexity from O(N^2) to O(N) for the parsing step
                while (process_idx + 4 <= buffer_.size()) {
                    // 1. Search for Header 0x03
                    if (buffer_[process_idx] != 0x03) {
                         process_idx++;
                         continue;
                    }

                    // 2. Check Tail 0x0D (Header is at process_idx, so Tail is at process_idx + 3)
                    if (buffer_[process_idx + 3] == 0x0D) {
                        uint16_t combined = (uint16_t)buffer_[process_idx + 1] | ((uint16_t)buffer_[process_idx + 2] << 8);
                        
                        raw_data_.store(combined, std::memory_order_release);
                        frequency_calculator_.sampleFrequency();
                        find_count_ ++;
                        std::cout << "Read Count: " << read_count_ << ", Find Count: " << find_count_ << std::endl;
                        std::cout   << "Frame: " 
                                    << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buffer_[process_idx]) << " "
                                    << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buffer_[process_idx + 1]) << " "
                                    << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buffer_[process_idx + 2]) << " "
                                    << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(buffer_[process_idx + 3]) 
                                    << std::endl;

                        // Advance index past this valid frame
                        process_idx += 4;
                    } else {
                        // Header found but Tail mismatch -> Invalid start
                        // Advance by 1 to search for next potential header
                        process_idx++;
                    }
                }

                // Remove processed data from the beginning of the buffer in one go
                if (process_idx > 0) {
                    buffer_.erase(buffer_.begin(), buffer_.begin() + process_idx);
                }
            } else {
                 std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        } catch (const std::exception& e) {
            std::cerr << "Error in torque sensor reading loop: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

float TorqueSensor::getTorque() const {
    uint16_t raw = raw_data_.load(std::memory_order_acquire);
    float current_voltage = (static_cast<float>(raw) / 32768.0f) * max_voltage_;
    return (current_voltage - zero_voltage_) * slope_;
}

float TorqueSensor::getFrequency() const {
    return frequency_calculator_.getFrequency();
}
} // namespace torque_sensor
