#include "my_car/debug_logger.hpp"

#include <cstdlib>      // for getenv
#include <filesystem>   // C++17
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>

DebugLogger::DebugLogger(const std::string & filename)
{
  std::string base_dir;

  const char * env_log_dir = std::getenv("ROS_LOG_DIR");
  if (env_log_dir) {
    base_dir = std::string(env_log_dir);
  } else {
    const char * home = std::getenv("HOME");
    base_dir = std::string(home ? home : ".") + "/.ros/log";
  }

  base_dir += "/latest_build";
  //std::cout << "base_dir= "<< base_dir << std::endl;
  std::filesystem::create_directories(base_dir);

  log_file_path_ = base_dir + "/" + filename;
  file_stream_.open(log_file_path_);
}

DebugLogger::~DebugLogger()
{
  if (file_stream_.is_open()) {
    file_stream_.close();
  }
}

