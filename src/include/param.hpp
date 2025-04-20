#pragma once
#include <yaml-cpp/node/parse.h>
#include <yaml-cpp/yaml.h>

#include <exception>
#include <filesystem>
#include <opencv2/core/types.hpp>
#include <stdexcept>

#include "interface.hpp"
#include "util/file_manager.hpp"
#include "util/singleton_base.hpp"

namespace at
{
namespace fs = std::filesystem;
class Param : public Singleton<Param>
{
  friend class Singleton<Param>;

public:
  Param(const fs::path & source_path_, bool is_debug = true)
  : m_is_debug(is_debug), m_source_path(source_path_)

  {
    if (!fs::exists(m_source_path)) {
      std::string err = "unknown source path:" + m_source_path.string();
      LOG_ERROR << err << std::endl;
      throw std::runtime_error(err);
    }
    m_config_path = m_source_path / "config";
    m_logtxt_path = m_source_path / "log";
    if (!fs::exists(m_config_path)) {
      std::string err = "unknown config path" + m_config_path.string();
      LOG_ERROR << err << std::endl;
      throw std::runtime_error(err);
    }
    if (!fs::exists(m_logtxt_path)) {
      fs::create_directory(m_logtxt_path);
    }
    try {
      m_yaml_root = std::make_unique<YAML::Node>();
      *m_yaml_root = YAML::LoadFile(m_config_path / "config.yaml");
      load_config();
      update_config();
      if (m_is_debug) {
        m_config_monitor = std::make_unique<at::FileMonitor>(
          m_config_path / "config.yaml", [this]() { this->update_config(); });
      }
    } catch (const std::exception & e) {
      LOG_ERROR << "YAML error:" << e.what();
    }
  }

  ~Param() { m_running.store(false); }
  struct SerialP
  {
    int baudrate;
    fs::path device_path;
  } serial;
  struct CameraP
  {
    int width;
    int height;
    int fps;
    int exposure;
  } camera;
  struct ImgProcP
  {
    IdentScheme ident_scheme;
    double lower_h, lower_s, lower_v;
    double upper_h, upper_s, upper_v;
    int min_area;
    int max_area;
  } img_proc;

private:
  void update_config()
  {
    auto & root = *m_yaml_root;
    // img
    auto hsv_lower = root["img"]["hsv"]["lower"];
    auto hsv_upper = root["img"]["hsv"]["upper"];

    img_proc.lower_h = hsv_lower[0].as<int>();
    img_proc.lower_s = hsv_lower[1].as<int>();
    img_proc.lower_v = hsv_lower[2].as<int>();

    img_proc.upper_h = hsv_upper[0].as<int>();
    img_proc.upper_s = hsv_upper[1].as<int>();
    img_proc.upper_v = hsv_upper[2].as<int>();

    img_proc.min_area = root["img"]["area"]["min"].as<int>(10);
    img_proc.max_area = root["img"]["area"]["max"].as<int>(100);
  }
  void load_config()
  {
    auto & root = *m_yaml_root;
    // serial
    serial.device_path = root["serial"]["port"].as<std::string>("/dev/ttyACM0");
    serial.baudrate = root["serial"]["baudrate"].as<int>(9600);
    // camera
    camera.width = root["camera"]["width"].as<int>(640);
    camera.height = root["camera"]["height"].as<int>(480);
    camera.fps = root["camera"]["fps"].as<int>(60);
    camera.exposure = root["camera"]["exposure"].as<int>(-1);
  };
  bool m_is_debug;
  std::unique_ptr<YAML::Node> m_yaml_root;
  fs::path m_source_path;
  fs::path m_config_path;
  fs::path m_logtxt_path;
  std::atomic<bool> m_running;
  std::unique_ptr<at::FileMonitor> m_config_monitor;  // unblocked
};
}  // namespace at