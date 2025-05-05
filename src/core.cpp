#include "core.hpp"

#include <fcntl.h>
#include <termios.h>

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "interface.hpp"
#include "kalman_filter.hpp"
#include "param.hpp"
#include "serial_port.hpp"
#include "util/fps_counter.hpp"
#include "util/keyboard_listener.hpp"
#include "util/logger.hpp"
#include "util/util_func.hpp"

namespace at
{
Core::Core(bool is_debug)
: is_debug_(is_debug),
  tracker_(
    Param::getInstance().camera.width, Param::getInstance().camera.height),
  capture_fpsc("capture"),
  process_fpsc("process"),
  serial_fpsc("serial")

{
  FPSCounter::enable(is_debug_);
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
    serial_thread_.reset();
    LOG_INFO << "core end" << std::endl;
    if (is_debug_) {
      auto status = FPSCounter::getAllStats();
      LOG_DEBUG << "Process Status: \n" << status.at("process") << std::endl;
      LOG_DEBUG << "Capture Status: \n" << status.at("capture") << std::endl;
      LOG_DEBUG << "Serial Status: \n" << status.at("serial") << std::endl;
    }
    cap_.release();
  } else {
    LOG_ERROR << "core destruct error" << std::endl;
  }
}

void Core::serial_thread_func()
{
  auto & s = SerialPort::getInstance();
  auto & p = Param::getInstance();

  uint8_t buffer[128];

  while (main_running_.load()) {
    {
      std::shared_lock lock(packet_mutex);

      float correction_angle =
        angle_kf_.update(measurement_angle.load()) + p.other.led_offset;

      if (!slope_func_.isFinished()) {
        send_packet.angle = slope_func_.getValue();
      } else {
        send_packet.angle = correction_angle;
      }

      send_packet.angular_v = angle_kf_.getVelocity();

      if (is_debug_) {
        print(send_packet);
      }

      serialize(send_packet, buffer);
    }

    ssize_t sent = s.write(buffer, sizeof(buffer));
    if (sent < 0) {
      LOG_ERROR << "数据发送失败";
    }

    serial_fpsc++;
  }
}

void Core::setup_state_action()
{
  auto & sm = state_machine_;
  sm.set_on_enter(STATE_IDLE, [this]() {
    LOG_INFO << "Current state: IDLE" << std::endl;
    vision_running_.store(false);
    print_menu();
  });

  sm.set_on_enter(STATE_QUIT, [this]() {
    LOG_INFO << "Current state: QUIT" << std::endl;
    vision_running_.store(false);
    main_running_.store(false);
    stop();
  });

  sm.set_on_enter(STATE_ERROR, [this]() {
    LOG_ERROR << "Current state: ERROR" << std::endl;
    state_machine_.handle_event(EVENT_QUIT);
  });

  sm.set_on_enter(STATE_IMU_RUNNING, [this]() {
    LOG_INFO << "Current state: IMU_RUNNING" << std::endl;
    send_packet.mode = 0;
    vision_running_.store(true);
    cv.notify_all();
  });

  sm.set_on_enter(STATE_VISION_RUNNING, [this]() {
    LOG_INFO << "Current state: VISION_RUNNING" << std::endl;
    send_packet.mode = 1;
    vision_running_.store(true);
    cv.notify_all();
  });
}

void Core::setup_camera_opts()
{
  auto & c = Param::getInstance().camera;
  cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, c.width);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, c.height);
  cap_.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);  // 部分系统中：1 表示手动模式
  cap_.set(cv::CAP_PROP_EXPOSURE, c.exposure);  // 曝光值，单位和范围依驱动而异
  cap_.set(cv::CAP_PROP_GAIN, c.gain);
  cap_.set(cv::CAP_PROP_AUTO_WB, 0);  // 关闭自动白平衡
  cap_.set(cv::CAP_PROP_WB_TEMPERATURE, c.wb_temprature);  // 色温（若支持）
  cap_.set(cv::CAP_PROP_FPS, 30);  // 不一定生效，但可保留
}

void Core::init(int device_id)
{
  auto & s = SerialPort::getInstance();
  auto & p = Param::getInstance();
  if (!cap_.open(device_id, cv::CAP_V4L2)) {
    std::string err = "invalid camera id: " + std::to_string(device_id);
    LOG_ERROR << err << std::endl;
    throw std::runtime_error(err.c_str());
  } else {
    this->setup_camera_opts();
    LOG_INFO << "camera opened successfully" << std::endl;
  }
  if (!s.open(p.serial.device_path, p.serial.baudrate)) {
    std::string err = "Cannot open:" + p.serial.device_path.string() +
                      " with baudrate: " + std::to_string(p.serial.baudrate);
    LOG_ERROR << err << std::endl;
    throw std::runtime_error(err.c_str());
  } else {
    LOG_INFO << "serial port opened successfully" << std::endl;
    LOG_INFO << "Opened: " << cap_.isOpened()
             << ", W×H: " << cap_.get(cv::CAP_PROP_FRAME_WIDTH) << "×"
             << cap_.get(cv::CAP_PROP_FRAME_HEIGHT)
             << ", FPS: " << cap_.get(cv::CAP_PROP_FPS)
             << ", FOURCC: " << int(cap_.get(cv::CAP_PROP_FOURCC)) << std::endl;
  }

  vision_running_.store(false);
  capture_thread_ =
    std::make_unique<std::thread>(&Core::capture_thread_func, this);
  process_thread_ =
    std::make_unique<std::thread>(&Core::process_thread_func, this);
  serial_thread_ =
    std::make_unique<std::thread>(&Core::serial_thread_func, this);
}

void Core::start(int device_id)
{
  auto & p = Param::getInstance();
  init(device_id);

  main_running_.store(true);
  bool input_angle = false;
  float angle = 0.0f;
  at::Keyboard keyb;

  while (main_running_) {
    int key = keyb.get_char();
    if (input_angle) {
      if (key >= '0' && key <= '9') {
        angle = angle * 10 + (key - '0');
        LOG_INFO << "angle: " << angle << std::endl;
      } else if (key == 'q') {
        state_machine_.handle_event(EVENT_QUIT);
      } else if (key == 13) {  // enter
        if (angle < 60.0f || angle > 120.0f) {
          LOG_WARN << "overclaim angle: " << angle << std::endl;
          angle = 90.0f;
        }
        {
          std::lock_guard lk(packet_mutex);
          send_packet.expected_angle = angle;
          slope_func_.refresh(
            p.other.slope_duration_time, send_packet.angle,
            send_packet.expected_angle);
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
        LOG_INFO << "please input expected angle: " << std::endl;
        angle = 0.0f;
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
  main_running_.store(false);
  vision_running_.store(false);
  cv.notify_all();
  if (capture_thread_->joinable()) {
    capture_thread_->join();
  }
  if (process_thread_->joinable()) {
    process_thread_->join();
  }
  if (serial_thread_->joinable()) {
    serial_thread_->join();
  }
  return true;
}

void Core::capture_thread_func()
{
  cv::Mat frame;
  while (main_running_) {
    {
      std::unique_lock<std::mutex> lock(cv_mtx);
      cv.wait(
        lock, [this] { return !main_running_ || vision_running_.load(); });
    }
    while (vision_running_) {
      capture_fpsc++;
      if (!cap_.isOpened()) [[unlikely]] {
        LOG_ERROR << "camera offline" << std::endl;
        state_machine_.handle_event(EVENT_QUIT);
        break;
      }
      if (is_debug_) {
        if (!cap_.grab()) {
          LOG_WARN << "grab() failed\n";
          continue;
        }
        if (!cap_.retrieve(frame)) {
          LOG_WARN << "retrieve() failed\n";
          continue;
        }
      } else {
        if (!cap_.read(frame)) {
          LOG_WARN << "read() failed";
        }
      }
      if (!frame.empty()) [[likely]] {
        frame_queue_.push(frame);
      }
    }
  }
}

void Core::process_thread_func()
{
  cv::Mat img_raw, binary_img, result_img;
  while (main_running_) {
    {
      std::unique_lock<std::mutex> lock(cv_mtx);
      cv.wait(
        lock, [this] { return !main_running_ || vision_running_.load(); });
    }
    while (vision_running_.load()) {
      process_fpsc++;
      if (frame_queue_.waitAndPop(img_raw)) {
        binary_img = preprocess(img_raw);
        measurement_angle.store(
          weighing_detect(binary_img, img_raw, result_img));
        if (is_debug_) {
          cv::imshow("img_raw", img_raw);
          cv::imshow("binary", binary_img);
          cv::imshow("result", result_img);
          cv::waitKey(1);
        }
      } else {
        LOG_WARN << "frame queue is empty" << std::endl;
      }
    }
  };
}

cv::Mat Core::preprocess(const cv::Mat & input_img)
{
  auto & p = Param::getInstance();
  cv::Rect roi = tracker_.getRoi(p.img_proc.roi_ratio);

  cv::Mat cropped = input_img(roi);
  cv::Mat denoised, gray, binary;

  cv::GaussianBlur(cropped, denoised, cv::Size(3, 3), 0);
  cv::cvtColor(denoised, gray, cv::COLOR_BGR2GRAY);
  cv::threshold(gray, binary, p.img_proc.bin_thres, 255, cv::THRESH_BINARY);

  // 记录当前ROI左上角偏移，供后续坐标恢复
  current_roi_offset_ = roi.tl();

  return binary;
}

float Core::weighing_detect(
  const cv::Mat & binary_img, const cv::Mat & raw_img, cv::Mat & result_img)
{
  auto & p = Param::getInstance().img_proc;

  const cv::Point2f & roi_offset = current_roi_offset_;
  const cv::Rect roi_rect = tracker_.getRoi(p.roi_ratio);  // 实时获取

  if (is_debug_) {
    result_img = raw_img.clone();
    cv::rectangle(result_img, roi_rect, cv::Scalar(255, 0, 255), 2);
  } else {
    result_img = cv::Mat();
  }

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(
    binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  std::vector<cv::Point2f> target_pair;
  for (const auto & contour : contours) {
    double area = cv::contourArea(contour);
    if (area < p.min_area || area > p.max_area) continue;

    cv::Moments mu = cv::moments(contour);
    if (mu.m00 == 0) continue;

    cv::Point2f center(mu.m10 / mu.m00, mu.m01 / mu.m00);
    center += roi_offset;  // 坐标映射回全图
    target_pair.emplace_back(center);

    if (is_debug_) {
      std::vector<cv::Point> shifted;
      for (const auto & pt : contour)
        shifted.emplace_back((cv::Point2f)pt + roi_offset);
      draw_debug_result(result_img, shifted);
    }
  }

  if (target_pair.size() != 2) {
    tracker_.miss();
    LOG_INFO << "No matched led found" << std::endl;
    return -1;
  }

  tracker_.updatePoints(target_pair[0], target_pair[1]);

  double angle_rad = std::atan2(
    target_pair[0].y - target_pair[1].y, target_pair[0].x - target_pair[1].x);
  double angle_deg = angle_rad * 180.0 / CV_PI;

  if (is_debug_) {
    draw_line_result(
      result_img, {cv::Point(target_pair[0]), cv::Point(target_pair[1])},
      angle_deg);
  }

  return angle_deg;
}

void Core::draw_debug_result(
  cv::Mat & result_img, const std::vector<cv::Point> & contour) const
{
  // 绘制轮廓线
  cv::drawContours(
    result_img, std::vector<std::vector<cv::Point>>{contour}, -1,
    cv::Scalar(0, 255, 255), 3);

  // 绘制面积值
  cv::Moments m = cv::moments(contour);
  if (m.m00 > 0) {
    cv::Point center(m.m10 / m.m00, m.m01 / m.m00);
    std::string text =
      std::to_string(static_cast<int>(cv::contourArea(contour)));
    cv::putText(
      result_img, text, center, cv::FONT_HERSHEY_PLAIN, 1.0,
      cv::Scalar(0, 255, 0), 2);
  }
}

void Core::draw_line_result(
  cv::Mat & result_img, const std::vector<cv::Point> & pair_line,
  float angle_deg) const
{
  if (result_img.empty() || pair_line.size() != 2) {
    LOG_WARN << "input draw_line_result() parameter error!" << std::endl;
    return;
  }
  cv::line(result_img, pair_line.at(1), pair_line.at(0), cv::Scalar(0, 255, 0));

  // 计算中点
  cv::Point mid_pt = (pair_line[0] + pair_line[1]) * 0.5;

  // 绘制角度文字
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%.1f°", angle_deg);
  cv::putText(
    result_img, buf, mid_pt, cv::FONT_HERSHEY_SIMPLEX, 0.5,
    cv::Scalar(255, 0, 0), 1);
}

void Core::print_menu()
{
  std::cout << std::endl;
  std::cout << "┌───────────── 菜单 ─────────────┐\n";
  std::cout << "│ " << std::left << std::setw(5) << "v"
            << "│ 启动视觉模块            │\n";
  std::cout << "│ " << std::left << std::setw(5) << "i"
            << "│ 启动 IMU 模块           │\n";
  std::cout << "│ " << std::left << std::setw(5) << "s"
            << "│ 设置目标角度            │\n";
  std::cout << "│ " << std::left << std::setw(5) << "esc"
            << "│ 停止                    │\n";
  std::cout << "│ " << std::left << std::setw(5) << "q"
            << "│ 退出程序                │\n";
  std::cout << "└────────────────────────────────┘\n";
  std::cout << "请选择操作编号: " << std::endl;
}

}  // namespace at