#pragma once

#include <sys/epoll.h>
#include <sys/inotify.h>
#include <unistd.h>

#include <atomic>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "logger.hpp"

class FileEventMonitor
{
public:
  FileEventMonitor() : inotify_fd_(-1), epoll_fd_(-1), running_(false) {}

  ~FileEventMonitor() { stop(); }

  bool start()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) return true;

    inotify_fd_ = inotify_init1(IN_NONBLOCK);
    if (inotify_fd_ < 0) {
      LOG_ERROR << "Failed to init inotify" << std::endl;
      return false;
    }

    epoll_fd_ = epoll_create1(0);
    if (epoll_fd_ < 0) {
      LOG_ERROR << "Failed to init epoll" << std::endl;
      close(inotify_fd_);
      return false;
    }

    epoll_event ev;
    ev.events = EPOLLIN;
    ev.data.fd = inotify_fd_;
    if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, inotify_fd_, &ev) < 0) {
      LOG_ERROR << "Failed to add inotify to epoll" << std::endl;
      close(inotify_fd_);
      close(epoll_fd_);
      return false;
    }

    running_ = true;
    monitor_thread_ = std::thread([this]() { this->run(); });

    LOG_INFO << "FileEventMonitor started" << std::endl;
    return true;
  }

  void stop()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_) return;
      running_ = false;
    }

    if (monitor_thread_.joinable()) {
      monitor_thread_.join();
    }

    for (const auto & [wd, _] : callbacks_) {
      inotify_rm_watch(inotify_fd_, wd);
    }

    close(inotify_fd_);
    close(epoll_fd_);

    LOG_INFO << "FileEventMonitor stopped" << std::endl;
  }

  bool addWatch(const std::string & path, std::function<void()> callback)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (inotify_fd_ < 0) return false;

    int wd = inotify_add_watch(
      inotify_fd_, path.c_str(), IN_CLOSE_WRITE);
    if (wd < 0) {
      LOG_ERROR << "Failed to watch file: " << path << std::endl;
      return false;
    }

    callbacks_[wd] = callback;
    LOG_INFO << "Watching file: " << path << std::endl;
    return true;
  }

private:
  void run()
  {
    constexpr int BUF_LEN = 1024 * (sizeof(inotify_event) + 16);
    char buf[BUF_LEN];

    while (running_) {
      epoll_event events[1];
      int nfds = epoll_wait(epoll_fd_, events, 1, 1000);  // timeout 1s

      if (nfds <= 0) continue;

      int len = read(inotify_fd_, buf, BUF_LEN);
      if (len <= 0) continue;

      int i = 0;
      while (i < len) {
        inotify_event * event = reinterpret_cast<inotify_event *>(&buf[i]);

        if (event->mask & (IN_MODIFY | IN_CLOSE_WRITE | IN_ATTRIB)) {
          std::lock_guard<std::mutex> lock(mutex_);
          auto it = callbacks_.find(event->wd);
          if (it != callbacks_.end()) {
            LOG_INFO << "File modified, triggering callback" << std::endl;
            it->second();  // 调用回调
          }
        }

        i += sizeof(inotify_event) + event->len;
      }
    }
  }

  int inotify_fd_;
  int epoll_fd_;
  std::atomic<bool> running_;
  std::thread monitor_thread_;
  std::mutex mutex_;

  std::map<int, std::function<void()>> callbacks_;
};
