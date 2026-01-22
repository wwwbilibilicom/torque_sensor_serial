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
      slope_(0.0f),
      data_received_{false} {
    
    read_buffer_.reserve(MAX_READ_SIZE);
    parser_ = std::make_unique<SerialParser>(
        std::bind(&TorqueSensor::onFrameReceived, this, std::placeholders::_1)
    );

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
    std::vector<uint8_t> temp_read_buf(4);
    static int read_count_ = 0;
    while (running_) {
        try {
            // 获取当前串口硬件缓冲区有多少字节在排队
            size_t available = serial_->available();
            
            if (available > 0) {
                // 限制单次读取量，不要超过 buffer 大小
                size_t bytes_to_read = std::min(available, MAX_READ_SIZE);

                // 直接读入预先分配好的连续物理内存区域
                // .data() 返回 uint8_t* 指针，性能等同于原生数组
                size_t bytes_read = serial_->read(read_buffer_.data(), bytes_to_read);

                if (bytes_read > 0) {
                    // 将原始数据流送入解包逻辑
                    parser_->parse(read_buffer_.data(), bytes_read);
                }
            } else {
                // 串口通常不需要极高频率的空转，休眠 500us 可显著降低 CPU 占用
                // 且不会对 1ms 级的数据产生延迟感知
                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }
        } catch (const std::exception& e) {
            std::cerr << "Error in torque sensor reading loop: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

void TorqueSensor::onFrameReceived(const SensorFrame& frame) {
    // 处理接收到的帧
    raw_data_.store(frame.value, std::memory_order_release);
    frequency_calculator_.sampleFrequency();
    data_received_ = true;
}

float TorqueSensor::getTorque() const {
    if (!data_received_) {
        std::cerr << "No valid data received yet." << std::endl;
        return 0.0f;
    }
    uint16_t raw = raw_data_.load(std::memory_order_acquire);
    float current_voltage = (static_cast<float>(raw) / 32768.0f) * max_voltage_;
    return (current_voltage - zero_voltage_) * slope_;
    // return static_cast<float>(raw_data_.load(std::memory_order_acquire));
}

float TorqueSensor::getFrequency() const {
    return frequency_calculator_.getFrequency();
}
} // namespace torque_sensor
