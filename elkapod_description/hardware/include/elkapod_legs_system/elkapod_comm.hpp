#include <cstdlib>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <array>

#include <libserial/SerialPort.h>

#include "spidev.hpp"

namespace elkapod_comm{
    typedef struct{
        std::array<float, 4> temperatures;
        std::array<float, 10> imu_data;
        float battery_voltage;
        float battery_percentage;
        bool battery_present;
    } SpiTransmissionResponse;

    typedef struct{
        std::array<float, 18> angles;

    } SpiTransmissionRequest;

    class UARTDevice{
        public:
            UARTDevice(const std::string device_path = "/dev/ttyAMA1", const uint32_t baudrate=115200, const int timeout_ms=100);
            ~UARTDevice();

            void connect();
            void disconnect();
            
            void sendSystemStartCommand();
            void sendSystemShutdownCommand();

            uint8_t sendAngles(float* angles);

        private:
            LibSerial::SerialPort serial_comm_;

            const std::string device_path_;
            const uint32_t baudrate_;
            const int timeout_ms_;
    };

    class ElkapodComm{
        public:
            ElkapodComm(std::unique_ptr<UARTDevice> uart, std::unique_ptr<SpiDevice> spi);

            void connect();
            void disconnect();

            void sendSystemStartCommand();
            void sendSystemShutdownCommand();
            SpiTransmissionResponse transfer(const SpiTransmissionRequest& request);

        private:
            SpiTransmissionResponse parseSpiTransferBytesResponse(const std::vector<uint8_t> bytes_response);
            std::unique_ptr<UARTDevice> uart_;
            std::unique_ptr<SpiDevice> spi_;

    };
};
