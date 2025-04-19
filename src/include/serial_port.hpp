#pragma once
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>
#include <string>

#include "util/singleton_base.hpp"

namespace at
{

class SerialPort : public Singleton<SerialPort>
{
public:
  friend class Singleton<SerialPort>;
  SerialPort();
  ~SerialPort();
  bool open(const std::string & device, int baudrate = 115200);
  void close();
  ssize_t write(const uint8_t * data, size_t size);
  ssize_t read(uint8_t * buffer, size_t size);
  bool isOpen() const;

private:
  int fd_;
  speed_t baudToSpeed(int baudrate)
  {
    // clang-format off
    switch (baudrate) {
      case 9600 : return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
      case 57600: return B57600;
      case 115200: return B115200;
      default: return B115200; // fallback
    }
    // clang-format on
  }
};
}  // namespace at