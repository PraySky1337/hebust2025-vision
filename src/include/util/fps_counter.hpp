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
  uint64_t total_frames;  // 总帧数
  double total_time;      // 总运行时间

  FrameStats()
  : fps(0.0),
    avg_frame_time(0.0),
    min_frame_time(0.0),
    max_frame_time(0.0),
    jitter(0.0),
    total_frames(0),
    total_time(0.0)
  {
  }

  std::ostream & operator>>(std::ostream & os) const
  {
    // 保存当前格式状态
    auto flags = os.flags();
    auto precision = os.precision();

    os << std::fixed << std::setprecision(2) << "[性能统计] \n"
       << "FPS: " << fps << "\n"
       << "总帧数: " << total_frames << "\n"
       << "平均帧时间: " << avg_frame_time << " ms\n"
       << "最小帧时间: " << min_frame_time << " ms\n"
       << "最大帧时间: " << max_frame_time << " ms\n"
       << "抖动: " << jitter << " ms\n"
       << "运行时间: " << total_time / 1000.0 << " s\n";  // 毫秒转换为秒

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
  : name_(name),
    frame_count_(0),  // 新增：总帧数计数器
    fps_(0.0),
    frame_time_(0.0),
    window_size_(window_size),
    total_frames_(0),
    total_time_(0.0),  // 新增：总运行时间
    start_time_(std::chrono::high_resolution_clock::now())  // 新增：开始时间
  {
    last_time_ = start_time_;
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
    frame_times_(std::move(other.frame_times_)),
    total_frames_(other.total_frames_),
    total_time_(other.total_time_),
    start_time_(other.start_time_)
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

    // 更新总运行时间（毫秒）
    total_time_ =
      std::chrono::duration<double, std::milli>(now - start_time_).count();

    frame_time_ = elapsed.count();
    frame_times_.push_back(frame_time_);

    if (frame_times_.size() > window_size_) {
      frame_times_.erase(frame_times_.begin());
    }

    ++total_frames_;  // 新增：增加总帧数计数

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
      std::accumulate(frame_times_.begin(), frame_times_.end(), 0.0) /
      frame_times_.size();
    double variance = std::accumulate(
                        frame_times_.begin(), frame_times_.end(), 0.0,
                        [mean](double sum, double t) {
                          return sum + (t - mean) * (t - mean);
                        }) /
                      frame_times_.size();
    return std::sqrt(variance);
  }

  static void enable(bool state)
  {
    enabled_.store(state, std::memory_order_relaxed);
  }

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
    stats.total_frames = total_frames_;
    // 使用成员变量存储的总时间
    stats.total_time = total_time_;

    // 计算平均帧时间
    stats.avg_frame_time =
      std::accumulate(frame_times_.begin(), frame_times_.end(), 0.0) /
      frame_times_.size();

    // 计算最大最小帧时间
    auto [min_it, max_it] =
      std::minmax_element(frame_times_.begin(), frame_times_.end());
    stats.min_frame_time = *min_it;
    stats.max_frame_time = *max_it;

    // 计算标准差（抖动）
    stats.jitter = calculateJitter();

    return stats;
  }

  // 获取所有计数器的统计信息
  [[nodiscard]] static std::unordered_map<std::string, FrameStats> getAllStats()
  {
    if (!isEnabled()) {
      return {};
    }
    std::lock_guard<std::mutex> lock(mutex_);
    std::unordered_map<std::string, FrameStats> result;
    for (const auto & [name, counter] : counters_) {
      if (counter) {
        result[name] = counter->getStats();
      }
    }
    return result;
  }

  // 新增：获取总帧数
  uint64_t getTotalFrames() const { return total_frames_; }

  // 新增：获取从开始到现在的平均帧率
  double getAverageFPS() const
  {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double>(now - start_time_).count();
    return duration > 0 ? total_frames_ / duration : 0.0;
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
    return sorted_times[lower_idx] * (1.0 - weight) +
           sorted_times[upper_idx] * weight;
  }

  // 计算标准差
  double calculateJitter() const
  {
    if (frame_times_.empty()) return 0.0;

    double mean =
      std::accumulate(frame_times_.begin(), frame_times_.end(), 0.0) /
      frame_times_.size();

    double variance = std::accumulate(
                        frame_times_.begin(), frame_times_.end(), 0.0,
                        [mean](double sum, double t) {
                          return sum + (t - mean) * (t - mean);
                        }) /
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
  uint64_t total_frames_;            // 总帧数
  double total_time_;                // 总运行时间
  std::chrono::high_resolution_clock::time_point start_time_;  // 新增：开始时间

  static inline std::mutex mutex_;
  static inline std::unordered_map<std::string, FPSCounter *> counters_;
  static inline std::atomic<bool> enabled_ = true;  // 全局开关
};
