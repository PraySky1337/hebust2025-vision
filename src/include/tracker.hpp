#pragma once
#include <opencv2/core.hpp>

class DynamicRoiTracker
{
public:
  DynamicRoiTracker(int img_width, int img_height, int max_miss = 5)
  : frame_width_(img_width), frame_height_(img_height), max_miss_(max_miss)
  {
  }

  // 更新当前两个点
  void updatePoints(const cv::Point2f & p1, const cv::Point2f & p2)
  {
    pt1_ = p1;
    pt2_ = p2;
    has_target_ = true;
    miss_count_ = 0;
  }

  // 未检测到目标
  void miss()
  {
    if (++miss_count_ >= max_miss_) {
      has_target_ = false;
    }
  }

  // 获取当前 ROI，传入 scale 比值
  cv::Rect getRoi(float scale_ratio) const
  {
    if (!has_target_) {
      return cv::Rect(0, 0, frame_width_, frame_height_);
    }

    cv::Point2f center = (pt1_ + pt2_) * 0.5f;
    float len = static_cast<float>(cv::norm(pt1_ - pt2_));
    int half_len =
      std::max(1, static_cast<int>(len * scale_ratio * 0.5f));  // 防止为0

    int x =
      std::clamp(static_cast<int>(center.x - half_len), 0, frame_width_ - 1);
    int y =
      std::clamp(static_cast<int>(center.y - half_len), 0, frame_height_ - 1);
    int w = std::min(2 * half_len, frame_width_ - x);
    int h = std::min(2 * half_len, frame_height_ - y);

    if (w <= 0 || h <= 0) {
      return cv::Rect(
        0, 0, frame_width_, frame_height_);  // fallback to full image
    }

    return cv::Rect(x, y, w, h);
  }

  bool hasTarget() const { return has_target_; }

private:
  int frame_width_, frame_height_;
  int max_miss_, miss_count_ = 0;

  cv::Point2f pt1_, pt2_;
  bool has_target_ = false;
};
