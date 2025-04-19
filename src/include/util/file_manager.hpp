#pragma once
#include <atomic>
#include <filesystem>
#include <functional>
#include <memory>
#include <thread>

#include "logger.hpp"

namespace at
{
namespace fs = std::filesystem;

class FileMonitor
{
public:
  FileMonitor(const fs::path & file_path, std::function<void()> callback)
  : file_path_(file_path), callback_(callback), running_(false)
  {
  }

  ~FileMonitor() { stop(); }

  bool start()
  {
    if (!fs::exists(file_path_)) {
      LOG_ERROR << "File path: " << file_path_ << " not exist";
      return false;
    }

    running_.store(true);
    monitor_thread_ = std::make_unique<std::thread>([this]() {
      fs::file_time_type last_write_time = fs::last_write_time(file_path_);

      while (running_) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        try {
          auto new_write_time = fs::last_write_time(file_path_);
          if (new_write_time != last_write_time) {
            LOG_INFO << "File modified: " << file_path_;
            callback_();
            last_write_time = new_write_time;
          }
        } catch (const std::exception & e) {
          LOG_ERROR << "Monitor error: " << e.what();
        }
      }
      LOG_INFO << "File Monitor stopped: " << file_path_;
    });

    return true;
  }

  void stop()
  {
    running_.store(false);
    if (monitor_thread_ && monitor_thread_->joinable()) {
      monitor_thread_->join();
    }
  }

private:
  fs::path file_path_;
  std::function<void()> callback_;
  std::atomic<bool> running_;
  std::unique_ptr<std::thread> monitor_thread_;
};

}  // namespace at