#include "torque_sensor/TorqueSensor.h"
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <thread>
#include <chrono>

volatile bool running = true;

void signalHandler(int signum) {
    running = false;
}

int main() {
    // Register signal handler for Ctrl+C
    signal(SIGINT, signalHandler);

    // Default port
    // You may want to make this configurable via command line arguments
    std::string port = "/dev/ttyUSB0";
    uint32_t baudrate = 256000;
    
    // Configuration for 100Nm Sensor with 0-10V output
    torque_sensor::TorqueSensor::TorqueSensorType type = torque_sensor::TorqueSensor::TorqueSensorType::RANGE_30NM;
    float zero_voltage = 5.0f;
    float max_voltage = 10.0f;

    std::cout << "Initializing Torque Sensor on " << port << " at " << baudrate << " baud..." << std::endl;
    std::cout << "Type: 100Nm, Zero: " << zero_voltage << "V, Max: " << max_voltage << "V" << std::endl;

    torque_sensor::TorqueSensor sensor(port, baudrate, type, zero_voltage, max_voltage);

    if (!sensor.connect()) {
        std::cerr << "Failed to connect to torque sensor." << std::endl;
        return 1;
    }

    std::cout << "Connected! Reading data (Press Ctrl+C to stop)..." << std::endl;

    // Loop at approx 100Hz for display purposes
    while (running) {
        float torque = sensor.getTorque();
        std::cout << "Torque: " << torque << " Nm" << std::endl;
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "\nStopping..." << std::endl;
    sensor.disconnect();
    
    return 0;
}