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
      update_config();
      if (m_is_debug) {
        m_config_monitor.start();
        m_config_monitor.addWatch(
          m_config_path, [this] { this->update_config(); });
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
    int id;
    int width;
    int height;
    int fps;
    float exposure;
    float gain;
    int wb_temprature;
  } camera;
  struct ImgProcP
  {
    IdentScheme ident_scheme;
    int min_area;
    int max_area;
    int bin_thres;
    float roi_ratio;
  } img_proc;

  struct Other
  {
    float led_offset;
    float slope_duration_time;
  } other;

private:
  void update_config()
  {
    LOG_INFO << "update config !" << std::endl;
    *m_yaml_root = YAML::LoadFile(m_config_path / "config.yaml");
    auto & root = *m_yaml_root;
    // serial
    serial.device_path = root["serial"]["port"].as<std::string>("/dev/ttyUSB0");
    serial.baudrate = root["serial"]["baudrate"].as<int>(9600);
    // camera
    camera.id = root["camera"]["id"].as<int>(1);
    camera.width = root["camera"]["width"].as<int>(1280);
    camera.height = root["camera"]["height"].as<int>(720);
    camera.fps = root["camera"]["fps"].as<int>(30);
    camera.exposure = root["camera"]["exposure"].as<float>(10.0);
    camera.gain = root["camera"]["gain"].as<float>(1.0);
    camera.wb_temprature = root["camera"]["wb_temprature"].as<int>(3000);
    // img
    img_proc.min_area = root["img"]["area"]["min"].as<int>(15);
    img_proc.max_area = root["img"]["area"]["max"].as<int>(150);

    img_proc.bin_thres = root["img"]["threshold"].as<int>(150);
    img_proc.roi_ratio = root["img"]["roi"]["ratio"].as<float>(1.5);

    other.led_offset = root["other"]["led_offset"].as<float>(11.12);
    other.slope_duration_time =
      root["other"]["slope_duration_time"].as<float>(3.f);
  };
  bool m_is_debug;
  std::unique_ptr<YAML::Node> m_yaml_root;
  fs::path m_source_path;
  fs::path m_config_path;
  fs::path m_logtxt_path;
  std::atomic<bool> m_running;
  FileEventMonitor m_config_monitor;  // unblocked
};
}  // namespace at