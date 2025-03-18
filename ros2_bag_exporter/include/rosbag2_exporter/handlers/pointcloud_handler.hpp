/*
 * Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer
 * Date: 13.10.2024
 */

#ifndef ROSBAG2_EXPORTER__HANDLERS__POINTCLOUD_HANDLER_HPP_
#define ROSBAG2_EXPORTER__HANDLERS__POINTCLOUD_HANDLER_HPP_

#include "rclcpp/logging.hpp"
#include "rosbag2_exporter/handlers/base_handler.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

namespace rosbag2_exporter
{

class PointCloudHandler : public BaseHandler
{
public:
  // Constructor to accept logger
  PointCloudHandler(const std::string & topic_dir, rclcpp::Logger logger)
  : BaseHandler(logger), topic_dir_(topic_dir)
  {
  }

  void process_message(const rclcpp::SerializedMessage & serialized_msg, size_t global_id) override
  {
    sensor_msgs::msg::PointCloud2 pc2_msg;
    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
    serializer.deserialize_message(&serialized_msg, &pc2_msg);

    // Construct timestamp string
    std::stringstream ss_timestamp;
    ss_timestamp << pc2_msg.header.stamp.sec << "-" << std::setw(9) << std::setfill('0')
                 << pc2_msg.header.stamp.nanosec;
    std::string timestamp = ss_timestamp.str();

    // Ensure the directory exists
    std::filesystem::create_directories(topic_dir_);

    // Construct filename
    std::string filename = topic_dir_ + "/" + timestamp + ".pcd";

    data_meta_vec_.push_back(DataMeta{filename, pc2_msg.header.stamp, global_id});
    data_vec_.push_back(pc2_msg);
  }

  bool save_msg_to_file(size_t index) override
  {
    // Check index bounds
    if (index >= data_meta_vec_.size() || index >= data_vec_.size()) {
      RCLCPP_ERROR(logger_, "[PointCloudHandler] Provided index is out of range");
      return false;
    }

    DataMeta & data_meta = data_meta_vec_[index];
    sensor_msgs::msg::PointCloud2 & pc2_msg = data_vec_[index];

    // Check if the point cloud has an intensity field
    bool has_intensity = std::any_of(
      pc2_msg.fields.begin(), pc2_msg.fields.end(),
      [](const auto & field) { return field.name == "intensity"; });

    // Create the appropriate point cloud, convert the ROS message, and save it
    if (has_intensity) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
      pcl::fromROSMsg(pc2_msg, *cloud);
      return save_pointcloud_to_file<pcl::PointXYZI>(cloud, data_meta.data_path);
    } else {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(pc2_msg, *cloud);
      return save_pointcloud_to_file<pcl::PointXYZ>(cloud, data_meta.data_path);
    }
  }

private:
  std::string topic_dir_;

  std::vector<sensor_msgs::msg::PointCloud2> data_vec_;

  // Templated function to save a point cloud to file
  template <typename PointT>
  bool save_pointcloud_to_file(
    typename pcl::PointCloud<PointT>::Ptr cloud, const std::string & filename)
  {
    if (pcl::io::savePCDFileBinary(filename, *cloud) == -1) {
      RCLCPP_ERROR(logger_, "Failed to write PCD file to %s", filename.c_str());
      return false;
    }

    RCLCPP_DEBUG(logger_, "Successfully wrote PointCloud2 message to: %s", filename.c_str());
    return true;
  }
};

}  // namespace rosbag2_exporter

#endif  // ROSBAG2_EXPORTER__HANDLERS__POINTCLOUD_HANDLER_HPP_
