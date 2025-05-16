#ifndef ROSBAG2_EXPORTER__UTILS_HPP_
#define ROSBAG2_EXPORTER__UTILS_HPP_

#include "builtin_interfaces/msg/time.hpp"
#include "rosbag2_exporter/handlers/base_handler.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <regex>
#include <string>
#include <utility>
#include <vector>

// ANSI escape codes for colourised terminal output
#define COLOR_RESET "\033[0m"
#define GREEN_LOG "\033[32m"
#define MAGENTA_LOG "\033[35m"
#define BOLD_LOG "\033[1m"
#define CYAN_LOG "\033[36m"
#define YELLOW_LOG "\033[33m"
#define UNDERLINE_LOG "\033[4m"

namespace rosbag2_exporter::utils
{

std::string get_cam_name(const std::string & path)
{
  const std::string prefix = "/sensor/camera/";
  size_t start = path.find(prefix);
  if (start == std::string::npos) {
    return "";  // Prefix not found
  }

  // Move past "/sensor/camera/"
  start += prefix.length();

  size_t end = path.find('/', start);
  return path.substr(start, end - start);
}

// Helper function to convert builtin_interfaces::msg::Time to nanoseconds
int64_t toNanoseconds(const builtin_interfaces::msg::Time & timestamp)
{
  return static_cast<int64_t>(timestamp.sec) * std::pow(10, 9) + timestamp.nanosec;
}

// Function to find the closest index in a sorted vector for a given timestamp
size_t find_closest_timestamp(
  const std::vector<DataMeta> & target_vector, const builtin_interfaces::msg::Time & timestamp,
  size_t & last_index)
{
  int64_t timestamp_ns = toNanoseconds(timestamp);

  // Start from the last known index
  size_t closest_index = last_index;
  int64_t min_diff = std::numeric_limits<int64_t>::max();

  // Iterate forward through the sorted vector
  for (size_t i = last_index; i < target_vector.size(); ++i) {
    int64_t target_ns = toNanoseconds(target_vector[i].timestamp);
    int64_t diff = std::abs(target_ns - timestamp_ns);

    // Update closest index and minimum difference
    if (diff < min_diff) {
      min_diff = diff;
      closest_index = i;
    } else {
      // Since the vector is sorted, stop when differences start increasing
      break;
    }
  }

  // Update the last index for the next call
  last_index = closest_index;
  return closest_index;
}

// Helper to  produce a stdout progress bar
void print_progress(int percentage)
{
  // Static is safe as this runs only until 100% is reached
  static int last_percentage = -1;

  // Never exceeds 100%
  if (percentage > 100) {
    percentage = 100;
  }
  // Prevent duplicate prints
  if (percentage == last_percentage) {
    return;
  }

  last_percentage = percentage;

  int width = 50;
  int pos = width * percentage / 100;

  std::cout << "\r\t\t\t\t\t\t [";
  for (int i = 0; i < width; ++i) {
    if (i < pos) {
      std::cout << "=";
    } else if (i == pos) {
      std::cout << ">";
    } else {
      std::cout << " ";
    }
  }

  std::cout << "] " << percentage << "%" << std::flush;

  // Print a newline once at 100%
  if (percentage >= 100) {
    std::cout << std::endl;
  }
}

/**
 * @brief Find and return all .mcap file paths in a directory, sorted by index.
 *
 * Looks for files with names ending in "_<index>.mcap" and returns them
 * sorted numerically by their index value.
 *
 * @param rosbag_directory Path to the folder containing .mcap files.
 * @return A vector of sorted .mcap file paths.
 */
std::vector<fs::path> find_rosbags(const fs::path & rosbag_directory)
{
  std::regex pattern(R"(.*_(\d+)\.mcap)");
  std::vector<std::pair<int, fs::path>> indexed;

  for (const auto & entry : fs::directory_iterator(rosbag_directory)) {
    if (!entry.is_regular_file()) {
      continue;
    }
    std::smatch match;
    std::string name = entry.path().filename().string();
    if (std::regex_match(name, match, pattern)) {
      indexed.emplace_back(std::stoi(match[1]), entry.path());
    }
  }

  std::sort(indexed.begin(), indexed.end());

  std::vector<fs::path> sorted;
  for (auto & [_, path] : indexed) {
    sorted.push_back(path);
  }
  return sorted;
}

/**
 * @brief Create a new name based on the original rosbag directory name.
 *
 * The original name is expected to be in the format:
 *      <timestamp>_<rosbag_name>/
 * where <timestamp> is in the format YYYY_MM_DD-HH_MM_SS,
 * and <rosbag_name> is the name of the recording.
 *
 * The new name is created by swapping the timestamp and recording name.
 * Example: 2025_03_25-12_47_53_arthurs_seat_loop
 * becomes: arthurs_seat_loop_2025_03_25-12_47_53
 *
 * @param rosbag_directory The path to the original rosbag directory.
 * @return The new name for the directory.
 */
std::string get_new_name(const fs::path & rosbag_directory)
{
  constexpr size_t timestamp_char_length = 19;
  std::string base_name = rosbag_directory.stem().string();
  std::string timestamp = base_name.substr(0, timestamp_char_length);
  std::string route_name =
    base_name.substr(timestamp_char_length + 1, base_name.size() - timestamp_char_length + 1);
  std::string new_name = route_name + "_" + timestamp;
  return new_name;
}

/**
 * @brief Get the rosbag index from the its filename.
 *
 * The filename is expected to be in the format:
 *      <name>_<index>.mcap
 * @param rosbag_path The path to the rosbag file.
 * @return The index extracted from the filename.
 */
size_t get_rosbag_index(const fs::path & rosbag_path)
{
  std::string base_name = rosbag_path.stem().string();
  size_t index = base_name.find_last_of('_');
  size_t dot_index = base_name.find_last_of('.');
  if (index == std::string::npos) {
    throw std::runtime_error(
      "Invalid rosbag filename format. Expected format: <name>_<index>.mcap");
  }
  return std::stoul(base_name.substr(index + 1, dot_index - index - 1));
}

}  // namespace rosbag2_exporter::utils

#endif  // ROSBAG2_EXPORTER__UTILS_HPP_
