#pragma once
#include <concepts>
#include <cstring>
#include <unistd.h>
#include <string>
#include <limits.h>
#include <opencv2/opencv.hpp>
#include <cmath>


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


inline std::string JudgeColor(const cv::Mat& image, const cv::Point& center, bool use_ring_sample = true) {
    cv::Vec3f color_sum(0, 0, 0);

    if (use_ring_sample) {
        int radius = 3;
        int count = 0;

        for (int dy = -radius; dy <= radius; ++dy) {
            for (int dx = -radius; dx <= radius; ++dx) {
                float dist = std::sqrt(dx * dx + dy * dy);
                if (dist > radius - 1 && dist <= radius + 1) {
                    int x = center.x + dx;
                    int y = center.y + dy;
                    if (x >= 0 && x < image.cols && y >= 0 && y < image.rows) {
                        color_sum += image.at<cv::Vec3b>(y, x);
                        count++;
                    }
                }
            }
        }

        if (count == 0) return "unknown";

        color_sum /= count;
    } else {
        if (center.x < 0 || center.x >= image.cols || center.y < 0 || center.y >= image.rows)
            return "unknown";

        cv::Vec3b color = image.at<cv::Vec3b>(center.y, center.x);
        color_sum = cv::Vec3f(color[0], color[1], color[2]);
    }

    float B = color_sum[0];
    float G = color_sum[1];
    float R = color_sum[2];
    (void)G;
    if (R > 150 && R > B + 50)
        return "red";
    else if (B > 150 && B > R + 50)
        return "blue";
    else
        return "unknown";
}


}  // namespace at