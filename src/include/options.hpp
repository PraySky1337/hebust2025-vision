// CommandLineOptions.hpp
#pragma once

#include <iostream>
#include <string>
#include <filesystem>

#include "depend/cxxopts.hpp"
#include "util/util_func.hpp"
#include "interface.hpp"

namespace at {
namespace fs = std::filesystem;
class CommandLineOptions
{
public:
  fs::path source_path;
  bool is_debug = false;
  LogLevel log_level = LogLevel::INFO;
  bool show_help = false;

  void parse(int argc, char * argv[])
  {
    cxxopts::Options options("vision", "hebust AT 2025 电赛视觉上位机");

    options.add_options()(
      "s,source", "Source file path",
      cxxopts::value<fs::path>()->default_value(get_source_path()))(
      "debug", "Enable debug mode",
      cxxopts::value<bool>()->default_value("false"))(
      "l,log-level", "Log level (info/warn/error)",
      cxxopts::value<std::string>()->default_value("info"))(
      "h,help", "Print usage");

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
      std::cout << options.help() << std::endl;
      show_help = true;
      return;
    }

    source_path = result["source"].as<fs::path>();
    is_debug = result["debug"].as<bool>();

    std::string log_level_str = result["log-level"].as<std::string>();
    if (log_level_str == "info") {
      log_level = LogLevel::INFO;
    } else if (log_level_str == "warn") {
      log_level = LogLevel::WARN;
    } else if (log_level_str == "error") {
      log_level = LogLevel::ERROR;
    }
  }

private:
  fs::path get_source_path()
  {
    fs::path source_path;
    fs::path executable_path = getExecutableDir();
    source_path = executable_path.parent_path();
    return source_path;
  }
};
} // namespace at
