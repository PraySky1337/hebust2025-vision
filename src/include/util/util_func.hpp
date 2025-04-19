#pragma once
#include <concepts>
#include <cstring>

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


}  // namespace at