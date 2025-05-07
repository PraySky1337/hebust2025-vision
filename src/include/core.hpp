#pragma once
#include <atomic>
#include <condition_variable>
#include <memory>
#include <opencv2/video/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <shared_mutex>
#include <thread>

#include "interface.hpp"
#include "tracker.hpp"
#include "kalman_filter.hpp"
#include "util/fps_counter.hpp"
#include "util/safe_queue.hpp"
#include "util/slope.hpp"
#include "util/state_machine.hpp"


namespace at
{
struct LEDInfo
{
  cv::Point center;
  double area;
  std::string color;
};

class Core
{
public:
  Core(bool is_debug);
  ~Core();

  // clang-format off
  enum State {
    STATE_IDLE,
    STATE_VISION_RUNNING,
    STATE_IMU_RUNNING,
    STATE_QUIT,
    STATE_ERROR
  };
  enum Event {
    EVENT_START_VISION,
    EVENT_START_IMU,
    EVENT_STOP,
    EVENT_QUIT,
    EVENT_ERROR
  };
  // clang-format on

  void start(int device_id = 0);
  bool stop();

private:
  void init(int device_id = 0);
  cv::Mat preprocess(const cv::Mat & img);
  void setup_state_action();
  void setup_camera_opts();

  float weighing_detect(const cv::Mat & binary_img, const cv::Mat & raw_img, cv::Mat & result_img);
  void capture_thread_func();
  void process_thread_func();
  void serial_thread_func();
  void print_menu();
  void draw_debug_result(cv::Mat & result_img, const std::vector<cv::Point> & contours) const;
  void draw_line_result(cv::Mat & result_img, const std::vector<cv::Point> & line, float angle_deg) const;

  bool is_debug_;
  StateMachine<State, Event> state_machine_;
  DynamicRoiTracker tracker_;
  TimeSlopeFunction slope_func_;
  cv::Point2f current_roi_offset_;
  std::atomic<bool> vision_running_{false};
  std::atomic<bool> main_running_{false};
  std::atomic<bool> serial_running_{false};
  std::atomic<float> target_angle;
  std::atomic<float> measurement_angle;
  SendPacket send_packet;
  std::shared_mutex packet_mutex;
  std::mutex cv_mtx;
  std::condition_variable cv;
  cv::VideoCapture cap_;
  AngleKalmanFilter angle_kf_;
  SafeQueue<cv::Mat> frame_queue_;
  std::unique_ptr<std::thread> capture_thread_;
  std::unique_ptr<std::thread> process_thread_;
  std::unique_ptr<std::thread> serial_thread_;
  FPSCounter capture_fpsc;
  FPSCounter process_fpsc;
  FPSCounter serial_fpsc;
};
}  // namespace at