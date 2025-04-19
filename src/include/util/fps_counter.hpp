#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

struct FrameStats
{
  double fps;             // 帧率
  double frame_time;      // 当前帧时间
  double avg_frame_time;  // 平均帧时间
  double min_frame_time;  // 最小帧时间
  double max_frame_time;  // 最大帧时间
  double jitter;          // 帧间时间抖动（标准差）
  double percentile_95;   // 95%帧时间
  size_t frame_count;     // 统计的帧数量

  FrameStats()
  : fps(0.0),
    frame_time(0.0),
    avg_frame_time(0.0),
    min_frame_time(0.0),
    max_frame_time(0.0),
    jitter(0.0),
    percentile_95(0.0),
    frame_count(0)
  {
  }

  std::ostream & operator>>(std::ostream & os) const
  {
    // 保存当前格式状态
    auto flags = os.flags();
    auto precision = os.precision();

    os << std::fixed << std::setprecision(2) << "[性能统计] "
       << "FPS: " << fps << ", "
       << "帧时间: " << frame_time << "ms, "
       << "平均: " << avg_frame_time << "ms, "
       << "最小: " << min_frame_time << "ms, "
       << "最大: " << max_frame_time << "ms, "
       << "抖动: " << jitter << "ms, "
       << "95分位: " << percentile_95 << "ms, "
       << "帧数: " << frame_count;

    // 恢复格式状态
    os.flags(flags);
    os.precision(precision);

    return os;
  }

  // 友元声明，用于支持标准流操作符
  friend std::ostream & operator<<(std::ostream & os, const FrameStats & stats)
  {
    return stats >> os;
  }
};

class FPSCounter
{
public:
  explicit FPSCounter(const std::string & name, size_t window_size = 8)
  : name_(name), frame_count_(0), fps_(0.0), frame_time_(0.0), window_size_(window_size)
  {
    last_time_ = std::chrono::high_resolution_clock::now();
    registerCounter(name, this);
  }

  ~FPSCounter() { unregisterCounter(name_); }

  // 禁止拷贝
  FPSCounter(const FPSCounter &) = delete;
  FPSCounter & operator=(const FPSCounter &) = delete;

  // 允许移动
  FPSCounter(FPSCounter && other) noexcept
  : name_(std::move(other.name_)),
    last_time_(other.last_time_),
    frame_count_(other.frame_count_),
    fps_(other.fps_),
    frame_time_(other.frame_time_),
    window_size_(other.window_size_),
    frame_times_(std::move(other.frame_times_))
  {
    updateCounterPtr(name_, this);
  }

  // 记录一帧的时间，并计算单帧延迟
  double operator++()
  {
    if (!enabled_.load(std::memory_order_relaxed)) return 0.0;

    auto now = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = now - last_time_;
    last_time_ = now;

    frame_time_ = elapsed.count();
    frame_times_.push_back(frame_time_);

    if (frame_times_.size() > window_size_) {
      frame_times_.erase(frame_times_.begin());
    }

    if (++frame_count_ >= window_size_) {
      calculateFPS();
    }

    return frame_time_;
  }

  double operator++(int)
  {
    double old_frame_time = frame_time_;
    ++(*this);
    return old_frame_time;
  }

  // 获取当前帧率
  double getFPS() const { return fps_; }

  // 获取单次帧延迟（毫秒）
  double getFrameTime() const { return frame_time_; }

  // 获取帧间时间抖动 (标准差)
  double getJitter() const
  {
    if (frame_times_.empty()) return 0.0;
    double mean =
      std::accumulate(frame_times_.begin(), frame_times_.end(), 0.0) / frame_times_.size();
    double variance = std::accumulate(
                        frame_times_.begin(), frame_times_.end(), 0.0,
                        [mean](double sum, double t) { return sum + (t - mean) * (t - mean); }) /
                      frame_times_.size();
    return std::sqrt(variance);
  }

  static void enable(bool state) { enabled_.store(state, std::memory_order_relaxed); }

  static bool isEnabled() { return enabled_.load(std::memory_order_relaxed); }

  // 静态方法：获取所有计数器的统计信息
  static std::unordered_map<std::string, double> getAllFPS()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::unordered_map<std::string, double> result;
    for (const auto & [name, counter] : counters_) {
      if (counter) {
        result[name] = counter->getFPS();
      }
    }
    return result;
  }

  static std::unordered_map<std::string, double> getAllFrameTimes()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::unordered_map<std::string, double> result;
    for (const auto & [name, counter] : counters_) {
      if (counter) {
        result[name] = counter->getFrameTime();
      }
    }
    return result;
  }

  // 获取完整的统计信息
  FrameStats getStats() const
  {
    FrameStats stats;
    if (frame_times_.empty()) {
      return stats;
    }

    // 计算基础统计信息
    stats.fps = fps_;
    stats.frame_time = frame_time_;
    stats.frame_count = frame_times_.size();

    // 计算平均帧时间
    stats.avg_frame_time =
      std::accumulate(frame_times_.begin(), frame_times_.end(), 0.0) / frame_times_.size();

    // 计算最大最小帧时间
    auto [min_it, max_it] = std::minmax_element(frame_times_.begin(), frame_times_.end());
    stats.min_frame_time = *min_it;
    stats.max_frame_time = *max_it;

    // 计算标准差（抖动）
    stats.jitter = calculateJitter();

    // 计算95%分位数
    stats.percentile_95 = calculatePercentile(0.95);

    return stats;
  }

  // 获取所有计数器的统计信息
  static std::unordered_map<std::string, FrameStats> getAllStats()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::unordered_map<std::string, FrameStats> result;
    for (const auto & [name, counter] : counters_) {
      if (counter) {
        result[name] = counter->getStats();
      }
    }
    return result;
  }

private:
  void calculateFPS()
  {
    double sum = std::accumulate(frame_times_.begin(), frame_times_.end(), 0.0);
    fps_ = 1000.0 / (sum / frame_times_.size());
    frame_count_ = 0;
  }

  static void registerCounter(const std::string & name, FPSCounter * counter)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    counters_[name] = counter;
  }

  static void unregisterCounter(const std::string & name)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    counters_.erase(name);
  }

  static void updateCounterPtr(const std::string & name, FPSCounter * counter)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (counters_.find(name) != counters_.end()) {
      counters_[name] = counter;
    }
  }

  // 计算百分位数
  double calculatePercentile(double percentile) const
  {
    if (frame_times_.empty()) return 0.0;

    std::vector<double> sorted_times = frame_times_;
    std::sort(sorted_times.begin(), sorted_times.end());

    double index = percentile * (sorted_times.size() - 1);
    size_t lower_idx = static_cast<size_t>(std::floor(index));
    size_t upper_idx = static_cast<size_t>(std::ceil(index));

    if (lower_idx == upper_idx) {
      return sorted_times[lower_idx];
    }

    double weight = index - lower_idx;
    return sorted_times[lower_idx] * (1.0 - weight) + sorted_times[upper_idx] * weight;
  }

  // 计算标准差
  double calculateJitter() const
  {
    if (frame_times_.empty()) return 0.0;

    double mean =
      std::accumulate(frame_times_.begin(), frame_times_.end(), 0.0) / frame_times_.size();

    double variance = std::accumulate(
                        frame_times_.begin(), frame_times_.end(), 0.0,
                        [mean](double sum, double t) { return sum + (t - mean) * (t - mean); }) /
                      frame_times_.size();

    return std::sqrt(variance);
  }

  std::string name_;
  std::chrono::high_resolution_clock::time_point last_time_;
  size_t frame_count_;
  double fps_;
  double frame_time_;
  size_t window_size_;
  std::vector<double> frame_times_;  // 用于计算滑动窗口的 FPS 和 jitter

  static inline std::mutex mutex_;
  static inline std::unordered_map<std::string, FPSCounter *> counters_;
  static inline std::atomic<bool> enabled_ = true;  // 全局开关
};
