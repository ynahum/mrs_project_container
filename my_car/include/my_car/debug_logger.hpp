#ifndef DEBUG_LOGGER_HPP_
#define DEBUG_LOGGER_HPP_

#include <string>
#include <fstream>
#include <memory>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <filesystem>
#include <utility>

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
  explicit DebugLogger(const std::string & filename);

  /**
   * @brief Clean up and close the log file
   */
  ~DebugLogger();

  template<typename... Args>
  void write(Args&&... args)
  {
    if (!file_stream_.is_open()) {
      return;
    }

    using namespace std::chrono;
    auto now = system_clock::now().time_since_epoch();
    file_stream_ << "[" << std::fixed << std::setprecision(9) << duration<double>(now).count() << "] ";
    (file_stream_ << ... << std::forward<Args>(args)) << std::endl;
  }

private:
  std::string log_file_path_;
  std::ofstream file_stream_;
};


#endif  // DEBUG_LOGGER_HPP_
