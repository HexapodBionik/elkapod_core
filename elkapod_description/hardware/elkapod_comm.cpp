#include "elkapod_legs_system/elkapod_comm.hpp"
#include <memory>
#include <cstring>
#include <algorithm>

using namespace elkapod_comm;

static LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    case 1152000: return LibSerial::BaudRate::BAUD_1152000;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 115200" << std::endl;
      return LibSerial::BaudRate::BAUD_115200;
  }
}

ElkapodComm::ElkapodComm(std::unique_ptr<UARTDevice> uart, std::unique_ptr<SpiDevice> spi): uart_(std::move(uart)), spi_(std::move(spi)){

}

void ElkapodComm::connect(){
    spi_->connect();
    uart_->connect();
}


void ElkapodComm::disconnect(){
    spi_->disconnect();
    uart_->disconnect();
}

SpiTransmissionResponse ElkapodComm::parseSpiTransferBytesResponse(const std::vector<uint8_t> bytes_response){
    std::array<float, 4> temperatures;
    std::vector<uint8_t> bytes_buff(16, 0);

    std::copy_n(bytes_response.begin(), 16, bytes_buff.begin());
    std::memcpy(temperatures.data(), bytes_buff.data(), 16);

    std::array<float, 10> imu_data;
    bytes_buff.resize(40);
    
    std::copy_n(bytes_response.begin() + 16, 40, bytes_buff.begin()); 
    std::memcpy(imu_data.data(), bytes_buff.data(), 40);

    float battery_voltage;
    float battery_percentage;
    bool battery_present;

    std::memcpy(&battery_voltage, bytes_response.data() + 56, sizeof(float));
    std::memcpy(&battery_percentage, bytes_response.data() + 60, sizeof(float));
    std::memcpy(&battery_present, bytes_response.data() + 64, sizeof(bool));

    SpiTransmissionResponse response{
        .temperatures = temperatures,
        .imu_data = imu_data,
        .battery_voltage = battery_voltage,
        .battery_percentage = battery_percentage,
        .battery_present = battery_present
    };
    return response; 
}

SpiTransmissionResponse ElkapodComm::transfer(const SpiTransmissionRequest& request){
    std::vector<uint8_t> bytes(80, 0);
    bytes[0] = 0x7E;
    bytes[79] = 0x0F;

    size_t offset = 1;  
    for (const float& f : request.angles) {
        std::memcpy(&bytes[offset], &f, sizeof(float));
        offset += sizeof(float);
    }

    std::vector<uint8_t> result = spi_->transfer(bytes);
    SpiTransmissionResponse response = parseSpiTransferBytesResponse(result);
    
    return response;
} 

void ElkapodComm::sendSystemStartCommand(){
    uart_->sendSystemStartCommand();
}

void ElkapodComm::sendSystemShutdownCommand(){
    uart_->sendSystemShutdownCommand();
}

bool ElkapodComm::checkConnection(){
    return uart_->checkConnection();
}

UARTDevice::UARTDevice(const std::string device_path, const uint32_t baudrate, const int timeout_ms): device_path_(device_path), baudrate_(baudrate), timeout_ms_(timeout_ms){
    if(!isCharacterDevice(device_path)){
        throw std::runtime_error("Provided device path does not exits or device is not valid!");
    }
}

UARTDevice::~UARTDevice(){
    if(serial_comm_.IsOpen()){
      serial_comm_.Close();
    }
}

void UARTDevice::connect(){
    serial_comm_.Open(device_path_);
    serial_comm_.SetBaudRate(convert_baud_rate(baudrate_));
    serial_comm_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
    serial_comm_.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial_comm_.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_comm_.SetStopBits(LibSerial::StopBits::STOP_BITS_2);
}

void UARTDevice::disconnect(){
    serial_comm_.Close();
}

void UARTDevice::sendSystemStartCommand() {
    char data[10] = {0};
    data[0] = 0x7E;
    data[1] = 0x21;
    data[2] = 0x87;
    data[8] = 0xff;
    data[9] = 0x0f;

    // 0x21 0x87 -> system start command

    serial_comm_.FlushIOBuffers();
    for(size_t i = 0; i < sizeof(data); ++i){
        serial_comm_.WriteByte(data[i]);
    }
    serial_comm_.DrainWriteBuffer();
}

void UARTDevice::sendSystemShutdownCommand() {
    char data[10] = {0};
    data[0] = 0x7E;
    data[1] = 0x11;
    data[2] = 0x38;
    data[8] = 0xff;
    data[9] = 0x0f;

    // 0x11 0x38 -> system shutdown command
    serial_comm_.FlushIOBuffers();
    for(size_t i = 0; i < sizeof(data); ++i){
        serial_comm_.WriteByte(data[i]);
    }
    serial_comm_.DrainWriteBuffer();
}

bool UARTDevice::checkConnection(){
    if(serial_comm_.IsOpen()){
        serial_comm_.FlushInputBuffer();
        LibSerial::DataBuffer buffer;
        buffer.resize(20);

        try{
            serial_comm_.Read(buffer, buffer.size(), timeout_ms_);
        }
        catch (const LibSerial::ReadTimeout&){
            return false;
        }

        if(buffer[0] == 0x7E && buffer[1] == 0xAA && buffer[18] == 0xFF && buffer[19] == 0x0F){
            return true;
        }
    }
    return false;
}

uint8_t UARTDevice::sendAngles(float* angles){
    char data[78];
    data[0] = 0x7E;
    data[1] = 78;
    data[2] = 0x03;

    std::memcpy(data + 3, angles, 18*sizeof(float));

    data[76] = 0xff;
    data[77] = 0x0f;

    serial_comm_.FlushIOBuffers();
    // for(int i = 0; i < sizeof(data); ++i){
    //     serial_comm.WriteByte(data[i]);
    // }

    LibSerial::DataBuffer buff;
    buff.insert(buff.end(), data, data + 78);
    serial_comm_.Write(buff);

    // LibSerial::DataBuffer writebuffer;
    // writebuffer.reserve(77);
    // std::copy(data, data+77, writebuffer.begin());
    // serial_comm.Write(writebuffer);

    serial_comm_.DrainWriteBuffer();
    LibSerial::DataBuffer buffer;
    buffer.reserve(2);

    try
    {
        serial_comm_.Read(buffer, 1, 100);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        return 1;
    }

    return buffer[0];
}
