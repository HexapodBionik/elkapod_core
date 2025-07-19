#include <libserial/SerialPort.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <unistd.h>


class ElkapodComm{
    public:
        ElkapodComm() = default;
        void connect(const std::string address = "/dev/ttyAMA1", const LibSerial::BaudRate baudrate = LibSerial::BaudRate::BAUD_2000000);
        void disconnect();
        
        uint8_t writeLED(uint8_t led_number, uint8_t state);
        uint8_t sendAngle(float angle);
        uint8_t sendAngles(float* angles);

        LibSerial::SerialPort serial_comm;

    private:
        uint8_t timeout_ms=100;
};