#include "elkapod_legs_system/elkapod_comm.hpp"
#include <memory>
#include <cstring>

void ElkapodComm::connect(const std::string address, const LibSerial::BaudRate baudrate){
    serial_comm.Open(address);
    serial_comm.SetBaudRate(baudrate);
    serial_comm.SetFlowControl(LibSerial::FlowControl::FLOW_CONTROL_NONE);
    serial_comm.SetParity(LibSerial::Parity::PARITY_NONE);
    serial_comm.SetStopBits(LibSerial::StopBits::STOP_BITS_2);
}

void ElkapodComm::disconnect(){
    serial_comm.Close();
}

uint8_t ElkapodComm::writeLED(uint8_t led_number, uint8_t state){
    char data[7];

    data[0] = 0x7E;
    data[1] = 0x07;
    data[2] = 0x01;
    data[3] = led_number;
    data[4] = state;
    data[5] = 0xff;
    data[6] = 0x0f;
    

    serial_comm.FlushIOBuffers();
    for(int i = 0; i < sizeof(data); ++i){
        serial_comm.WriteByte(data[i]);
    }

    serial_comm.DrainWriteBuffer();
    LibSerial::DataBuffer buffer;
    buffer.reserve(2);

    try
    {
        serial_comm.Read(buffer, 1, 100);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        return 100;
    }

    return buffer[0];
}

uint8_t ElkapodComm::sendAngle(float angle){
    char data[9];

    data[0] = 0x7E;
    data[1] = 0x09;
    data[2] = 0x02;

    std::memcpy(data+3, &angle, sizeof(float));

    data[7] = 0xff;
    data[8] = 0x0f;

    serial_comm.FlushIOBuffers();
    for(int i = 0; i < sizeof(data); ++i){
        serial_comm.WriteByte(data[i]);
    }

    serial_comm.DrainWriteBuffer();
    LibSerial::DataBuffer buffer;
    buffer.reserve(2);

    try
    {
        serial_comm.Read(buffer, 1, 100);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        return 100;
    }

    return buffer[0];
}

uint8_t ElkapodComm::sendAngles(float* angles){
    char data[78];
    data[0] = 0x7E;
    data[1] = 78;
    data[2] = 0x03;

    std::memcpy(data + 3, angles, 18*sizeof(float));

    data[76] = 0xff;
    data[77] = 0x0f;

    serial_comm.FlushIOBuffers();
    for(int i = 0; i < sizeof(data); ++i){
        serial_comm.WriteByte(data[i]);
    }

    // LibSerial::DataBuffer writebuffer;
    // writebuffer.reserve(77);
    // std::copy(data, data+77, writebuffer.begin());
    // serial_comm.Write(writebuffer);

    serial_comm.DrainWriteBuffer();
    LibSerial::DataBuffer buffer;
    buffer.reserve(2);

    try
    {
        serial_comm.Read(buffer, 1, 100);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        return 1;
    }

    return buffer[0];
}
