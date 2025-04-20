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
#include "kalman_filter.hpp"
#include "util/safe_queue.hpp"
#include "util/state_machine.hpp"
#include "util/fps_counter.hpp"

namespace at
{

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
  bool detect(cv::Mat & img, IdentScheme is);
  void setup_state_action();
  void setup_camera_opts();
  float hsv_detect(cv::Mat & img);
  float weighing_detect(cv::Mat & img);
  float calculate_angle(const cv::Rect & rect_1, const cv::Rect & rect_2);
  void capture_thread_func();
  void process_thread_func();
  void print_menu();

  bool is_debug_;
  StateMachine<State, Event> state_machine_;
  std::atomic<bool> vision_running_{false};
  std::atomic<bool> main_running_{false};
  SendPacket send_packet;
  std::shared_mutex packet_mutex;
  std::mutex cv_mtx;
  std::condition_variable cv;
  cv::VideoCapture cap_;
  AngleKalmanFilter angle_kf_;
  SafeQueue<cv::Mat> frame_queue_;
  std::unique_ptr<std::thread> capture_thread_;
  std::unique_ptr<std::thread> process_thread_;
  FPSCounter capture_fpsc;
  FPSCounter process_fpsc;
};
}  // namespace at