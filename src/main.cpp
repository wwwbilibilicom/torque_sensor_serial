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

    // Default port from the original example
    // You may want to make this configurable via command line arguments
    std::string port = "/dev/serial/by-id/usb-Hiker_sudio_YK_COM_Port_6D7816855250Port-if00";
    uint32_t baudrate = 256000;
    
    // Configuration for 30Nm Sensor with 0-10V output (0V=-30Nm, 5V=0Nm, 10V=30Nm)
    torque_sensor::TorqueSensor::TorqueSensorType type = torque_sensor::TorqueSensor::TorqueSensorType::RANGE_100NM;
    float zero_voltage = 5.0f;
    float max_voltage = 10.0f;

    std::cout << "Initializing Torque Sensor on " << port << " at " << baudrate << " baud..." << std::endl;
    std::cout << "Type: 30Nm, Zero: " << zero_voltage << "V, Max: " << max_voltage << "V" << std::endl;

    torque_sensor::TorqueSensor sensor(port, baudrate, type, zero_voltage, max_voltage);

    if (!sensor.connect()) {
        std::cerr << "Failed to connect to torque sensor." << std::endl;
        return 1;
    }

    std::cout << "Connected! Reading data (Press Ctrl+C to stop)..." << std::endl;

    while (running) {
        if (sensor.update()) {
            std::cout << "Torque: " << sensor.getTorque() << std::endl;
        }
        
        // Small sleep to prevent 100% CPU usage, adjust as needed for required sample rate
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    std::cout << "\nStopping..." << std::endl;
    sensor.disconnect();
    
    return 0;
}
