#include <fcntl.h>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include "core.hpp"
#include "include/param.hpp"
#include "interface.hpp"
#include "kalman_filter.hpp"
#include "serial_port.hpp"
#include "util/logger.hpp"
#include "util/util_func.hpp"

namespace at
{
Core::Core(bool is_debug) : is_debug_(is_debug)
{
  auto & sm = state_machine_;
  sm.add_transition(STATE_IDLE, EVENT_START_IMU, STATE_IMU_RUNNING);
  sm.add_transition(STATE_IDLE, EVENT_START_VISION, STATE_VISION_RUNNING);
  sm.add_transition(STATE_IDLE, EVENT_QUIT, STATE_QUIT);
  sm.add_transition(STATE_IDLE, EVENT_ERROR, STATE_ERROR);

  sm.add_transition(STATE_VISION_RUNNING, EVENT_STOP, STATE_IDLE);
  sm.add_transition(STATE_VISION_RUNNING, EVENT_QUIT, STATE_QUIT);
  sm.add_transition(STATE_VISION_RUNNING, EVENT_START_IMU, STATE_IMU_RUNNING);
  sm.add_transition(STATE_VISION_RUNNING, EVENT_ERROR, STATE_ERROR);

  sm.add_transition(STATE_IMU_RUNNING, EVENT_STOP, STATE_IDLE);
  sm.add_transition(STATE_IMU_RUNNING, EVENT_QUIT, STATE_QUIT);
  sm.add_transition(
    STATE_IMU_RUNNING, EVENT_START_VISION, STATE_VISION_RUNNING);
  sm.add_transition(STATE_IMU_RUNNING, EVENT_ERROR, STATE_ERROR);

  this->setup_state_action();

  sm.set_initial_state(STATE_IDLE);
}
Core::~Core()
{
  if (this->stop()) {
    capture_thread_.reset();
    process_thread_.reset();
    LOG_INFO << "core end";
  } else {
    LOG_ERROR << "core destruct error";
  }
}
void Core::setup_state_action()
{
  auto & sm = state_machine_;
  sm.set_on_enter(STATE_IDLE, [this]() {
    LOG_INFO << "Current state: IDLE" << std::endl;
    vision_running_.store(false);
  });

  sm.set_on_enter(STATE_QUIT, [this]() {
    LOG_INFO << "Current state: QUIT" << std::endl;
    vision_running_.store(false);
    main_running_.store(false);
    stop();
  });

  sm.set_on_enter(STATE_ERROR, [this]() {
    LOG_ERROR << "Current state: ERROR" << std::endl;
    stop();
    throw std::runtime_error("State Error");
    vision_running_.store(false);
    main_running_.store(false);
  });

  sm.set_on_enter(STATE_IMU_RUNNING, [this]() {
    LOG_INFO << "Current state: IMU_RUNNING" << std::endl;
    vision_running_.store(true);
    send_packet.mode = 0;
  });

  sm.set_on_enter(STATE_VISION_RUNNING, [this]() {
    LOG_INFO << "Current state: VISION_RUNNING" << std::endl;
    vision_running_.store(true);
    send_packet.mode = 1;
  });

}

void Core::start(int device_id)
{
  auto & s = SerialPort::getInstance();
  auto & p = Param::getInstance();
  if (!cap_.open(device_id, cv::CAP_V4L2)) {
    std::string err = "invalid camera id: " + std::to_string(device_id);
    LOG_ERROR << err << std::endl;
    throw std::runtime_error(err.c_str());
  } else {
    LOG_INFO << "camera opened successfully" << std::endl;
  }
  if (!s.open(p.serial.device_path, p.serial.baudrate)) {
    std::string err = "Cannot open:" + p.serial.device_path.string() +
                      " with baudrate: " + std::to_string(p.serial.baudrate);
    LOG_ERROR << err << std::endl;
    throw std::runtime_error(err.c_str());
  } else {
    LOG_INFO << "serial port opened successfully" << std::endl;
  }

  vision_running_.store(false);
  capture_thread_ =
    std::make_unique<std::thread>(&Core::capture_thread_func, this);
  process_thread_ =
    std::make_unique<std::thread>(&Core::process_thread_func, this);

  main_running_.store(true);
  int key = -1;
  bool input_angle = false;
  float angle = 0.0f;
  uint8_t buffer[128];

  while (main_running_) {
    auto& s = SerialPort::getInstance();
    {
      std::shared_lock lock(packet_mutex);
      serialize(send_packet, buffer);
    }
    if (s.write(buffer, sizeof(buffer)) < 0) {
      LOG_ERROR << "数据发送失败" << std::endl;
    }
    if (input_angle) {
      if (key >= '0' && key <= '9') {
        angle = angle * 10 + (key - '0');
        LOG_INFO << "angle: " << angle << std::endl;
      } else if (key == 13) {  // enter
        if (angle <= 45.0f || angle >= 145.0f) {
          LOG_WARN << "overclaim angle" << angle << std::endl;
          angle = 90.0f;
        }
        LOG_INFO << "Final expected angle: " << angle << std::endl;
        input_angle = false;
      }
    }
    // clang-format off
    switch (key) {
      case 27:
        state_machine_.handle_event(EVENT_STOP);
        break;
      case 's': case 'S':
        // 树莓派 3b 是 四核心
        // 本来想用协程，但只能如此架构。
        input_angle = true;
        break;
      case 'i': case 'I':
        state_machine_.handle_event(EVENT_START_IMU);
        break;
      case 'v': case 'V':
        state_machine_.handle_event(EVENT_START_VISION);
        break;
      case 'q': case 'Q':
        state_machine_.handle_event(EVENT_QUIT);
        break;
    }
    // clang-format on
  }
}

bool Core::stop()
{
  if (!main_running_.load()) {
    LOG_WARN << "Core is already stopped" << std::endl;
    return false;
  }
  vision_running_.store(false);
  main_running_.store(false);
  bool ret = true;
  if (capture_thread_->joinable()) {
    capture_thread_->join();
    ret = false;
  } else {
    LOG_WARN << "video capture thread cannot join" << std::endl;
  }
  if (process_thread_->joinable()) {
    process_thread_->join();
    ret = false;
    LOG_WARN << "video process thread cannot join" << std::endl;
  }
  {
    cv::Mat temp;  // 清空队列
    while (frame_queue_.pop(temp)) {}
  }
  return ret;
}

void Core::capture_thread_func()
{
  cv::Mat frame;
BACK:
  while (vision_running_) {
    if (!cap_.isOpened()) [[unlikely]] {
      LOG_ERROR << "camera offline" << std::endl;
      break;
    }

    if (!cap_.read(frame)) [[unlikely]] {
      LOG_WARN << "failed to read frame" << std::endl;
      continue;
    }

    if (!frame.empty()) [[likely]] {
      frame_queue_.push(frame);
    }
  }
  if (
    state_machine_.get_state() != STATE_ERROR &&
    state_machine_.get_state() != STATE_QUIT) {
    goto BACK;
  } else {
    return;
  }
}

void Core::process_thread_func()
{
  cv::Mat frame;
BACK:
  while (vision_running_) {
    if (frame_queue_.waitAndPop(frame)) {
      if (!detect(frame, IdentScheme::HSV)) {
        continue;
      }
    } else {
      LOG_WARN << "frame queue is empty";
    }
  };

  if (
    state_machine_.get_state() != STATE_ERROR &&
    state_machine_.get_state() != STATE_QUIT) {
    goto BACK;
  } else {
    return;
  }
}

bool Core::detect(cv::Mat & input_img, IdentScheme is)
{
  float raw_angle;
  if (is == IdentScheme::HSV) {
    raw_angle = hsv_detect(input_img);
  } else if (is == IdentScheme::Weighing) {
    raw_angle = weighing_detect(input_img);
  } else {
    throw std::runtime_error("Unknown input IdentScheme");
  }
  if (raw_angle == 0xFFFF) {
    return false;
  } else if (!inRange(raw_angle, -180.0f, 180.0f)) {
    LOG_ERROR << "angle invalid";
    return false;
  } else {
    float ripe_angle = angle_kf_.update(raw_angle);
    float angular_v = angle_kf_.getVelocity();
    {
      std::unique_lock lock(packet_mutex);
      send_packet.angle = ripe_angle;
      send_packet.angular_v = angular_v;
    }
    return true;
  }
}

float Core::hsv_detect(cv::Mat & rgb_img)
{
  using std::vector;
  auto & p = Param::getInstance().img_proc;
  cv::Mat hsv;
  cv::cvtColor(rgb_img, hsv, cv::COLOR_BGR2HSV);
  cv::Scalar low{p.lower_h, p.lower_s, p.lower_v};
  cv::Scalar upper{p.upper_h, p.upper_s, p.upper_v};
  cv::Mat mask;
  cv::inRange(hsv, low, upper, mask);
  vector<vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  if (contours.empty()) {
    LOG_INFO << "contours is empty";
    return false;
  }
  std::vector<cv::Rect> results;
  for (const auto & contour : contours) {
    cv::Rect bbox = cv::boundingRect(contour);
    if (bbox.area() < 8 || !inRange(bbox.area(), p.min_area, p.max_area))
      continue;
    cv::Mat roi = hsv(bbox);
    vector<cv::Mat> hsv_channels;
    cv::split(roi, hsv_channels);
    double mean_v = cv::mean(hsv_channels[2])[0];
    if (mean_v > 180) {
      results.push_back(bbox);
    }
  }
  if (results.size() == 2) {
    return calculate_angle(results[0], results[1]);
  } else {
    LOG_INFO << "Failed detected, results num:" << results.size() << std::endl;
    return 0xFFFF;
  }
}

float Core::weighing_detect(cv::Mat & img)
{
  (void)img;
  return true;
}

float Core::calculate_angle(const cv::Rect & rect_1, const cv::Rect & rect_2)
{
  auto center_down = (rect_1.tl() + rect_1.br()) * 0.5;
  auto center_up = (rect_2.tl() + rect_2.br()) * 0.5;
  if (center_down.y < center_up.y) {
    std::swap(center_down, center_up);
  }
  auto vec = center_up - center_down;
  return std::atan2(vec.y, vec.x) * 180.0f / CV_PI;
}

}  // namespace at