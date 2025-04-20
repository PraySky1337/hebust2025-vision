#pragma once

#include <cstdint>

namespace at {

enum class LogLevel { INFO = 0, WARN, ERROR, DEBUG};

enum class IdentScheme {
  HSV,
  Weighing
};

#pragma pack(push, 1)
struct SendPacket
{
  float angle;
  float angular_v;
  float expected_angle;
  uint8_t mode;
};
#pragma pack(pop)


} // namespace at