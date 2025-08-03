#include "elkapod_legs_system/spidev.hpp"

#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <string.h>
#include <stdexcept>

#include <linux/spi/spidev.h>

inline bool elkapod_comm::isCharacterDevice(const std::string& path) {
    struct stat st;
    if (stat(path.c_str(), &st) != 0)
        return false;
    return S_ISCHR(st.st_mode);  
}


elkapod_comm::SpiDevice::SpiDevice(uint8_t bus, uint8_t cs){
    device_path_ = "/dev/spidev" + std::to_string(bus) + "." + std::to_string(cs);

    if(!elkapod_comm::isCharacterDevice(device_path_)){
        throw std::runtime_error("Provided device path does not exits or device is not valid!");
    }
}

elkapod_comm::SpiDevice::~SpiDevice() {
    if (fd_ >= 0) {
        ::close(fd_);
    }
}

void elkapod_comm::SpiDevice::connect(){
    fd_ = ::open(device_path_.c_str(), O_RDWR);
    if (fd_ < 0) {
        throw std::runtime_error("Failed to open SPI device: " + device_path_);
    }
    applySettings();
}

void elkapod_comm::SpiDevice::disconnect(){
    if (fd_ >= 0) {
        ::close(fd_);
    }
}

void elkapod_comm::SpiDevice::setMode(uint8_t mode){
    mode_ = mode;
}   

void elkapod_comm::SpiDevice::setSpeed(uint32_t hz){
    speed_= hz;
}

void elkapod_comm::SpiDevice::setBitsPerWord(uint8_t bits){
    bits_ = bits;
}

void elkapod_comm::SpiDevice::applySettings(){
    if (ioctl(fd_, SPI_IOC_WR_MODE, &mode_) < 0)
        throw std::runtime_error("Failed to set SPI mode");

    if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_) < 0)
        throw std::runtime_error("Failed to set bits per word");

    if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_) < 0)
        throw std::runtime_error("Failed to set max speed");
}            




std::vector<uint8_t> elkapod_comm::SpiDevice::readBytes(uint32_t length){
    std::vector<uint8_t> rxBuff;
    rxBuff.reserve(length);
    uint8_t rxBuff_raw[length];

    struct spi_ioc_transfer spi_message[1];
    memset(spi_message, 0, sizeof(spi_message));
    
    spi_message[0].rx_buf = (unsigned long)rxBuff_raw;
    spi_message[0].len = length;
    spi_message[0].speed_hz = speed_;
    spi_message[0].bits_per_word = bits_;
    
    
    uint8_t status = ioctl(fd_, SPI_IOC_MESSAGE(1), spi_message);

    std::copy(rxBuff_raw, rxBuff_raw + length, std::back_inserter(rxBuff));
    return rxBuff;
}

void elkapod_comm::SpiDevice::writeBytes(const std::vector<uint8_t>& bytes){
    struct spi_ioc_transfer spi_message[1];
    memset(spi_message, 0, sizeof(spi_message));
    
    spi_message[0].tx_buf = (unsigned long)bytes.data();
    spi_message[0].len = bytes.size();
    spi_message[0].speed_hz  = speed_;
    spi_message[0].bits_per_word = bits_;
    spi_message[0].cs_change = 1;
    
    ioctl(fd_, SPI_IOC_MESSAGE(1), spi_message);
}

std::vector<uint8_t> elkapod_comm::SpiDevice::transfer(const std::vector<uint8_t>& bytes){
    std::vector<uint8_t> rxBuff(bytes.size());
    
    struct spi_ioc_transfer spi_message;
    memset(&spi_message, 0, sizeof(spi_message));
    
    spi_message.tx_buf = (unsigned long)bytes.data();
    spi_message.rx_buf = (unsigned long)rxBuff.data();
    spi_message.len = bytes.size();
    spi_message.speed_hz = speed_;
    spi_message.bits_per_word = bits_;

    int status = ioctl(fd_, SPI_IOC_MESSAGE(1), &spi_message);
    if (status < 0) {
        perror("SPI transfer failed");
    }

    return rxBuff;
}
