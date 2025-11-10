#ifndef SPIDEV_HPP
#define SPIDEV_HPP

#include <linux/spi/spidev.h>
#include <stdint.h>
#include <sys/stat.h>

#include <string>
#include <vector>

namespace elkapod_comm {
inline bool isCharacterDevice(const std::string& path) {
  struct stat st;
  if (stat(path.c_str(), &st) != 0) return false;
  return S_ISCHR(st.st_mode);
}

class SpiDevice {
 public:
  SpiDevice(uint8_t bus, uint8_t cs);
  ~SpiDevice();

  void connect();
  void disconnect();

  void setMode(uint8_t mode);
  void setSpeed(uint32_t hz);
  void setBitsPerWord(uint8_t bits);
  void applySettings();

  void writeBytes(const std::vector<uint8_t>& bytes);
  std::vector<uint8_t> readBytes(uint32_t length);
  std::vector<uint8_t> transfer(const std::vector<uint8_t>& tx_bytes);

 private:
  std::string device_path_;
  int fd_ = -1;

  uint8_t mode_ = SPI_MODE_0;
  uint32_t speed_ = 500000;
  uint8_t bits_ = 8;

  uint8_t bus_;
  uint8_t cs_;
};
};  // namespace elkapod_comm

#endif  // SPIDEV_HPP