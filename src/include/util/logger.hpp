#pragma once

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

#include "singleton_base.hpp"

namespace at
{
enum class LogLevel { INFO = 0, WARN, ERROR };

class Logger : public Singleton<Logger>
{
public:
  friend class Singleton<Logger>;
  Logger(const std::filesystem::path & log_path = "log.txt", LogLevel level = LogLevel::INFO)
  : logLevel(level)
  {
    try {
      // 确保父目录存在
      if (!log_path.parent_path().empty()) {
        std::filesystem::create_directories(log_path.parent_path());
      }

      logFile.open(log_path, std::ios::app);
      if (!logFile.is_open()) {
        std::cerr << "无法打开日志文件: " << log_path << std::endl;
      }
    } catch (const std::filesystem::filesystem_error & e) {
      std::cerr << "文件系统错误: " << e.what() << std::endl;
    }
  }

  ~Logger()
  {
    if (logFile.is_open()) logFile.close();
  }

  void setLevel(LogLevel level) { logLevel = level; }

  void setAutoFlush(bool enabled) { autoFlush = enabled; }

  // 流式输出日志
  Logger & operator<<(LogLevel level)
  {
    currentLevel = level;
    return *this;
  }

  template <typename T>
  Logger & operator<<(const T & val)
  {
    buffer << val;  // 移除自动添加的空格
    if (autoFlush) {
      flush();
    }
    return *this;
  }

  // 特化字符串类型
  Logger & operator<<(const std::string & val)
  {
    buffer << val;
    return *this;
  }

  Logger & operator<<(const char * val)
  {
    buffer << val;
    return *this;
  }

  Logger & operator<<(std::ostream & (*endl)(std::ostream &))
  {
    (void)endl;
    flush();
    return *this;
  }

private:
  LogLevel logLevel;
  LogLevel currentLevel = LogLevel::INFO;
  std::ofstream logFile;
  std::mutex mtx;
  std::ostringstream buffer;
  bool autoFlush = false;  // 默认不自动flush

  void flush()
  {
    if (currentLevel < logLevel) {
      return;
    }
    std::lock_guard<std::mutex> lock(mtx);

    // 添加时间戳
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    char timeStr[32];
    std::strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", std::localtime(&time));

    std::string tag = levelToString(currentLevel);
    std::string output = std::string(timeStr) + " [" + tag + "] " + buffer.str();

    // 移除末尾多余空格
    while (!output.empty() && output.back() == ' ') {
      output.pop_back();
    }

    std::cout << output << std::endl;
    if (logFile.is_open()) {
      logFile << output << std::endl;
    }

    buffer.str("");  // 清空缓存
    buffer.clear();  // 清除流状态
  }

  std::string levelToString(LogLevel level)
  {
    switch (level) {
      case LogLevel::INFO:
        return "INFO";
      case LogLevel::WARN:
        return "WARN";
      case LogLevel::ERROR:
        return "ERROR";
      default:
        return "UNKNOWN";
    }
  }
};
}  // namespace at

#define LOG_INFO (at::Logger::getInstance() << at::LogLevel::INFO)
#define LOG_WARN (at::Logger::getInstance() << at::LogLevel::WARN)
#define LOG_ERROR (at::Logger::getInstance() << at::LogLevel::ERROR)
