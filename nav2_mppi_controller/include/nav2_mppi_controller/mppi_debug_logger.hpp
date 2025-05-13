#ifndef MPPI_DEBUG_LOGGER_HPP_
#define MPPI_DEBUG_LOGGER_HPP_

#include <string>
#include <fstream>
#include <memory>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <filesystem>
#include <utility>

namespace mppi
{

/**
 * @brief Simple logger to write debug output to a ROS 2 log directory file
 */
class DebugLogger
{
public:
  /**
   * @brief Construct a logger and open the file
   * @param filename The filename to write to (e.g., "mppi_debug.log")
   */
  explicit DebugLogger(const std::string & filename = "mppi_debug.log");

  /**
   * @brief Clean up and close the log file
   */
  ~DebugLogger();

  /**
   * @brief Write a string to the debug log file
   * @param message The message to write
   */
  void write(const std::string & message);

  template<typename... Args>
  void write(Args&&... args)
  {
    if (!file_stream_.is_open()) {
      return;
    }

    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    file_stream_ << "[" << std::put_time(std::localtime(&now_time), "%F %T") << "] ";
    (file_stream_ << ... << std::forward<Args>(args)) << std::endl;
  }

private:
  std::string log_file_path_;
  std::ofstream file_stream_;
};

}  // namespace mppi

#endif  // MPPI_DEBUG_LOGGER_HPP_
