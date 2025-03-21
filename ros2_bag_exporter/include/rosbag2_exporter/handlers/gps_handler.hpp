/*
 * Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer
 * Date: 13.10.2024
 */

#ifndef ROSBAG2_EXPORTER__HANDLERS__GPS_HANDLER_HPP_
#define ROSBAG2_EXPORTER__HANDLERS__GPS_HANDLER_HPP_

#include "rclcpp/logging.hpp"
#include "rosbag2_exporter/handlers/base_handler.hpp"

#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

namespace rosbag2_exporter
{

class GPSHandler : public BaseHandler
{
public:
  // Constructor to accept logger
  GPSHandler(const std::string & topic_dir, rclcpp::Logger logger)
  : BaseHandler(logger), topic_dir_(topic_dir)
  {
  }

  void process_message(const rclcpp::SerializedMessage & serialized_msg, size_t global_id) override
  {
    // Deserialize the incoming message
    sensor_msgs::msg::NavSatFix gps_data;
    rclcpp::Serialization<sensor_msgs::msg::NavSatFix> serializer;
    serializer.deserialize_message(&serialized_msg, &gps_data);

    // Create a timestamped filename
    std::stringstream ss_timestamp;
    ss_timestamp << gps_data.header.stamp.sec << "-" << std::setw(9) << std::setfill('0')
                 << gps_data.header.stamp.nanosec;
    std::string timestamp = ss_timestamp.str();

    // Create the full file path with '.csv' as the extension
    std::string filepath = topic_dir_ + "/" + timestamp + ".csv";

    // Ensure the directory exists, create if necessary
    if (!std::filesystem::exists(topic_dir_)) {
      std::filesystem::create_directories(topic_dir_);
    }

    // Open file and write GPS data as CSV
    std::ofstream outfile(filepath);
    if (!outfile.is_open()) {
      RCLCPP_ERROR(logger_, "Failed to open file to write GPS data: %s", filepath.c_str());
      return;
    }

    // Write GPS data (latitude, longitude, altitude, position covariance)
    outfile << "timestamp," << "latitude,longitude,altitude,"
            << "covariance[0],covariance[1],covariance[2],covariance[3],covariance[4],covariance[5]"
               ",covariance[6],covariance[7],covariance[8]"
            << std::endl;
    outfile << timestamp << "," << gps_data.latitude << "," << gps_data.longitude << ","
            << gps_data.altitude << "," << gps_data.position_covariance[0] << ","
            << gps_data.position_covariance[1] << "," << gps_data.position_covariance[2] << ","
            << gps_data.position_covariance[3] << "," << gps_data.position_covariance[4] << ","
            << gps_data.position_covariance[5] << "," << gps_data.position_covariance[6] << ","
            << gps_data.position_covariance[7] << "," << gps_data.position_covariance[8]
            << std::endl;

    outfile.close();

    RCLCPP_DEBUG(logger_, "Successfully wrote GPS data to %s", filepath.c_str());
  }

  bool save_msg_to_file(size_t index) override { return false; }

private:
  std::string topic_dir_;
};

}  // namespace rosbag2_exporter

#endif  // ROSBAG2_EXPORTER__HANDLERS__GPS_HANDLER_HPP_
