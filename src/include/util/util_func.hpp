#pragma once
#include <concepts>
#include <cstring>
#include <unistd.h>
#include <string>
#include <limits.h>


#include "interface.hpp"

namespace at
{

template <typename T>
concept Comparable = requires(T a, T b)
{
  // clang-format off
  { a < b  } -> std::convertible_to<bool>;
  { a <= b } -> std::convertible_to<bool>;
  { a >= b } -> std::convertible_to<bool>;
  // clang-format on
};

template <Comparable T>
constexpr bool inRange(const T & value, const T & lower, const T & upper)
{
  return value >= lower && value <= upper;
}

template <>
constexpr bool inRange<int>(const int & value, const int & lower, const int & upper)
{
  return static_cast<unsigned>(value - lower) <= static_cast<unsigned>(upper - lower);
}

template <>
constexpr bool inRange<float>(const float & value, const float & lower, const float & upper)
{
  return value >= lower && value <= upper;
}

template <>
constexpr bool inRange<double>(const double & value, const double & lower, const double & upper)
{
  return value >= lower && value <= upper;
}

// 打包结构体为字节流
inline void serialize(const SendPacket & data, uint8_t * buffer)
{
  std::memcpy(buffer, &data, sizeof(SendPacket));
}

// 反序列化（接收时用）
inline void deserialize(const uint8_t * buffer, SendPacket & data)
{
  std::memcpy(&data, buffer, sizeof(SendPacket));
}

inline std::string getExecutablePath() {
  char path[PATH_MAX] = {0};
  ssize_t len = readlink("/proc/self/exe", path, sizeof(path) - 1);
  if (len != -1) {
    path[len] = '\0';
    return std::string(path);
  }
  return "";
}

inline std::string getExecutableDir() {
  std::string fullPath = getExecutablePath();
  size_t pos = fullPath.find_last_of('/');
  if (pos != std::string::npos) {
    return fullPath.substr(0, pos);
  }
  return "";
}

}  // namespace at