/*
 * Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer
 * Date: 13.10.2024
 */

#ifndef ROSBAG2_EXPORTER__HANDLERS__IMU_HANDLER_HPP_
#define ROSBAG2_EXPORTER__HANDLERS__IMU_HANDLER_HPP_

#include "rclcpp/logging.hpp"
#include "rosbag2_exporter/handlers/base_handler.hpp"

#include "sensor_msgs/msg/imu.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

namespace rosbag2_exporter
{

class IMUHandler : public BaseHandler
{
public:
  // Constructor to accept logger
  IMUHandler(const std::string & topic_dir, rclcpp::Logger logger)
  : BaseHandler(logger), topic_dir_(topic_dir)
  {
  }

  void process_message(const rclcpp::SerializedMessage & serialized_msg, size_t global_id) override
  {
    // Deserialize the incoming message
    sensor_msgs::msg::Imu imu_data;
    rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;
    serializer.deserialize_message(&serialized_msg, &imu_data);

    // Create a timestamped filename
    std::stringstream ss_timestamp;
    ss_timestamp << imu_data.header.stamp.sec << "-" << std::setw(9) << std::setfill('0')
                 << imu_data.header.stamp.nanosec;
    std::string timestamp = ss_timestamp.str();

    // Create the full file path with '.csv' as the extension
    std::string filepath = topic_dir_ + "/" + timestamp + ".csv";

    // Ensure the directory exists, create if necessary
    if (!std::filesystem::exists(topic_dir_)) {
      std::filesystem::create_directories(topic_dir_);
    }

    // Open file and write IMU data as CSV
    std::ofstream outfile(filepath);
    if (!outfile.is_open()) {
      RCLCPP_ERROR(logger_, "Failed to open file to write IMU data: %s", filepath.c_str());
      return;
    }

    // Write IMU data (orientation, angular velocity, linear acceleration)
    outfile << "timestamp," << "orientation_x,orientation_y,orientation_z,orientation_w,"
            << "angular_velocity_x,angular_velocity_y,angular_velocity_z,"
            << "linear_acceleration_x,linear_acceleration_y,linear_acceleration_z" << std::endl;
    outfile << timestamp << "," << imu_data.orientation.x << "," << imu_data.orientation.y << ","
            << imu_data.orientation.z << "," << imu_data.orientation.w << ","
            << imu_data.angular_velocity.x << "," << imu_data.angular_velocity.y << ","
            << imu_data.angular_velocity.z << "," << imu_data.linear_acceleration.x << ","
            << imu_data.linear_acceleration.y << "," << imu_data.linear_acceleration.z << std::endl;

    outfile.close();

    RCLCPP_DEBUG(logger_, "Successfully wrote IMU data to %s", filepath.c_str());
  }

  bool save_msg_to_file(size_t index) override { return false; }

private:
  std::string topic_dir_;
};

}  // namespace rosbag2_exporter

#endif  // ROSBAG2_EXPORTER__HANDLERS__IMU_HANDLER_HPP_
