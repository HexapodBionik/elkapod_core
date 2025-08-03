#ifndef SPIDEV_HPP
#define SPIDEV_HPP

#include <stdint.h>
#include <vector>
#include <string>
#include <linux/spi/spidev.h>

namespace elkapod_comm{
    class SpiDevice {
        public:
            SpiDevice(uint8_t bus, uint8_t cs);
            ~SpiDevice();

            void open();
            void close();

            void setMode(uint8_t mode);         
            void setSpeed(uint32_t hz);         
            void setBitsPerWord(uint8_t bits);  
            void applySettings();              

            void writeBytes(const std::vector<uint8_t>& bytes);
            std::vector<uint8_t> readBytes(uint32_t length);
            std::vector<uint8_t> transfer(const std::vector<uint8_t>& tx_bytes);

        private:
            std::string devicePath;
            int fd = -1;

            uint8_t mode = SPI_MODE_0;
            uint32_t speed = 500000;
            uint8_t bits = 8;

            uint8_t bus;
            uint8_t cs;
        };
};

#endif //SPIDEV_HPP