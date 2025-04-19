#include <filesystem>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv4/opencv2/opencv.hpp>

#include "core.hpp"
#include "param.hpp"
#include "serial_port.hpp"
#include "util/logger.hpp"

namespace
{
namespace fs = std::filesystem;
const fs::path source_path = "/home/sxs/Desktop/raspberry_pi/vision";
constexpr bool is_debug = true;
}  // namespace

int main()
{
  at::Logger::init(source_path / "log" / "log.txt", at::LogLevel::INFO);
  at::Param::init(source_path, is_debug);
  at::SerialPort::init();

  at::Core core(is_debug);
  core.start();  // call it after at::Param::init() && at::SerialPort::init() && at::Logger::init()
  at::SerialPort::getInstance().close();
  return 0;
}