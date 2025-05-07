#include <filesystem>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv4/opencv2/opencv.hpp>

#include "core.hpp"
#include "options.hpp"
#include "param.hpp"
#include "serial_port.hpp"
#include "util/logger.hpp"

int main(int argc, char * argv[])
{
  at::CommandLineOptions opts;
  opts.parse(argc, argv);
  if (opts.show_help) {
    return 0;
  }
  at::Logger::init(opts.source_path / "log" / "log.txt", opts.log_level);
  at::Logger::getInstance().setDebug(opts.is_debug);
  at::Param::init(opts.source_path, opts.is_debug);
  at::SerialPort::init();

  auto & cam_id = at::Param::getInstance().camera.id;

  at::Core core(opts.is_debug);
  core.start(
    cam_id);  // call it after at::Param::init() && at::SerialPort::init() && at::Logger::init()
  at::SerialPort::getInstance().close();
  return 0;
}