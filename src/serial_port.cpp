#include "serial_port.hpp"

namespace at
{
SerialPort::SerialPort() : fd_(-1) {}
SerialPort::~SerialPort()
{
  close();
}
bool SerialPort::open(const std::string & device, int baudrate)
{
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ < 0) return false;

  fcntl(fd_, F_SETFL, 0);  // 设置阻塞模式

  struct termios tty;
  memset(&tty, 0, sizeof tty);

  if (tcgetattr(fd_, &tty) != 0) return false;

  // 设置波特率
  speed_t speed = baudToSpeed(baudrate);
  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);

  // 8N1 模式
  tty.c_cflag &= ~PARENB;  // No parity
  tty.c_cflag &= ~CSTOPB;  // 1 stop bit
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;       // 8 bits
  tty.c_cflag &= ~CRTSCTS;  // No flow control
  tty.c_cflag |= CREAD | CLOCAL;

  // 原始输入输出模式
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_oflag &= ~OPOST;

  // 设置读取等待时间
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 10;  // 1秒超时（10 × 100ms）

  return tcsetattr(fd_, TCSANOW, &tty) == 0;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

ssize_t SerialPort::write(const uint8_t * data, size_t size)
{
  return ::write(fd_, data, size);
}

ssize_t SerialPort::read(uint8_t * buffer, size_t size)
{
  return ::read(fd_, buffer, size);
}

bool SerialPort::isOpen() const
{
  return fd_ >= 0;
}


}  // namespace at