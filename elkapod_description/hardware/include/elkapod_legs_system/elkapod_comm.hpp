#include <cstdlib>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <array>

#include <libserial/SerialPort.h>

#include "spidev.hpp"

namespace elkapod_comm{
    typedef struct{

    } SpiTransmissionResponse;

    typedef struct{
        std::array<float, 18> angles;

    } SpiTransmissionRequest;

    class UARTDevice{
        public:
            UARTDevice() = default;
            ~UARTDevice();

            void connect(const std::string address = "/dev/ttyAMA1", const LibSerial::BaudRate baudrate = LibSerial::BaudRate::BAUD_2000000);
            void disconnect();
            
            uint8_t writeLED(uint8_t led_number, uint8_t state);
            uint8_t sendAngle(float angle);
            uint8_t sendAngles(float* angles);

            LibSerial::SerialPort serial_comm;

        private:
            uint8_t timeout_ms=100;
    };

    class ElkapodComm{
        public:
            ElkapodComm(std::unique_ptr<UARTDevice> uart, std::unique_ptr<SpiDevice> spi);

            SpiTransmissionResponse transfer(const SpiTransmissionRequest& request);
            void connect();
            void disconnect();

        private:
            std::unique_ptr<UARTDevice> uart_;
            std::unique_ptr<SpiDevice> spi_;

    };
};
