#pragma once
#include <algorithm>
#include <chrono>

class TimeSlopeFunction
{
public:
  TimeSlopeFunction() = default;

  // 启动坡函数过程：设定持续时间、起始值、结束值
  void refresh(float duration_sec, float start_val, float end_val)
  {
    duration_ = std::max(0.0001f, duration_sec);
    start_val_ = start_val;
    end_val_ = end_val;
    start_time_ = std::chrono::steady_clock::now();
    active_ = true;
  }

  // 获取当前值（线性插值）
  float getValue()
  {
    if (!active_) return end_val_;

    using namespace std::chrono;
    float elapsed =
      duration_cast<duration<float>>(steady_clock::now() - start_time_).count();

    if (elapsed >= duration_) {
      active_ = false;
      return end_val_;
    }

    float alpha = elapsed / duration_;
    return start_val_ + alpha * (end_val_ - start_val_);
  }

  bool isFinished() const { return !active_; }

private:
  float start_val_ = 0.0f;
  float end_val_ = 1.0f;
  float duration_ = 1.0f;
  bool active_ = false;
  std::chrono::steady_clock::time_point start_time_;
};
