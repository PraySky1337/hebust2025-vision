#pragma once

#include <cstdint>
#include <iostream>
#include <iomanip>

namespace at {

enum class LogLevel { INFO = 0, WARN, ERROR, DEBUG};

enum class IdentScheme {
  HSV,
  Weighing
};

#pragma pack(push, 1)
struct SendPacket
{
  uint8_t header{0x5A};
  float angle{0.f};
  float angular_v{0.f};
  float expected_angle{90.f};
  uint8_t mode{0};
};
#pragma pack(pop)

inline void print(const SendPacket & pkt)
{
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "[SendPacket] "
            << "angle: " << pkt.angle << "  "
            << "angular_v: " << pkt.angular_v << "  "
            << "expected_angle: " << pkt.expected_angle << "  "
            << "mode: " << static_cast<int>(pkt.mode)
            << std::endl;
}


} // namespace at