/*
 * Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer
 * Date: 13.10.2024
 */

#ifndef ROSBAG2_EXPORTER__BAG_EXPORTER_HPP_
#define ROSBAG2_EXPORTER__BAG_EXPORTER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/readers/sequential_reader.hpp"
#include "rosbag2_exporter/handlers/base_handler.hpp"
#include "rosbag2_exporter/handlers/camera_info_handler.hpp"
#include "rosbag2_exporter/handlers/compressed_image_handler.hpp"
#include "rosbag2_exporter/handlers/gps_handler.hpp"
#include "rosbag2_exporter/handlers/image_handler.hpp"
#include "rosbag2_exporter/handlers/imu_handler.hpp"
#include "rosbag2_exporter/handlers/pointcloud_handler.hpp"
#include "rosbag2_exporter/handlers/tf_handler.hpp"
#include "rosbag2_storage/storage_options.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace rosbag2_exporter
{

enum class MessageType { PointCloud2, Image, CompressedImage, CameraInfo, IMU, GPS, TF, Unknown };

struct TopicConfig
{
  std::string name;
  MessageType type;
  std::string encoding;
  int sample_interval;
  std::string topic_dir;
};

struct Handler
{
  std::shared_ptr<BaseHandler> handler;
  size_t current_index;
};

class BagExporter : public rclcpp::Node
{
public:
  explicit BagExporter(const rclcpp::NodeOptions & options);

private:
  void load_configuration(const std::string & config_file);
  void setup_handlers();
  void export_bag();
  void create_metadata_file();

  bool tf_extracted_;
  bool all_cameras_info_extracted_;
  std::string bag_path_;
  std::string output_dir_;
  std::string storage_id_;
  std::string rosbag_base_name_;
  std::vector<TopicConfig> topics_;
  std::vector<std::string> cam_info_topics_;
  std::map<std::string, Handler> handlers_;
};

}  // namespace rosbag2_exporter

#endif  // ROSBAG2_EXPORTER__BAG_EXPORTER_HPP_
