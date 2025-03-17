/*
 * Author: Hector Cruz <hcruzgo@ed.ac.uk>
 * Date: 17.03.2025
 */

#ifndef ROSBAG2_EXPORTER__HANDLERS__CAMERA_INFO_HANDLER_HPP_
#define ROSBAG2_EXPORTER__HANDLERS__CAMERA_INFO_HANDLER_HPP_

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/logging.hpp"
#include "rosbag2_exporter/handlers/base_handler.hpp"

#include "sensor_msgs/msg/camera_info.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <string>

namespace rosbag2_exporter
{

class CameraInfoHandler : public BaseHandler
{
public:
  // Constructor to accept logger
  CameraInfoHandler(const std::string & topic_dir, rclcpp::Logger logger)
  : BaseHandler(logger), topic_dir_(topic_dir)
  {
  }

  void process_message(
    const rclcpp::SerializedMessage & serialized_msg, [[maybe_unused]] const std::string & topic,
    size_t global_id) override
  {
    // Deserialize the incoming message
    sensor_msgs::msg::CameraInfo cam_info_msg;
    rclcpp::Serialization<sensor_msgs::msg::CameraInfo> serializer;
    serializer.deserialize_message(&serialized_msg, &cam_info_msg);

    // Parse this message into yaml dictionary
    create_yaml(cam_info_msg);

    // Create the full file path with '.csv' as the extension
    std::string filepath = topic_dir_ + "/camera_calibration" + ".yaml";

    data_meta_ = DataMeta{filepath, builtin_interfaces::msg::Time(), global_id};
  }

  bool save_msg_to_file([[maybe_unused]] size_t index) override
  {
    // Emit (serialize) to string
    YAML::Emitter out;
    out << yaml_root_;

    // Write to file if desired
    std::ofstream fout(data_meta_.data_path);
    fout << out.c_str();
    fout.close();

    return true;
  }

private:
  void create_yaml(const sensor_msgs::msg::CameraInfo & cam_info_msg)
  {
    yaml_root_["camera_frame_id"] = cam_info_msg.header.frame_id;
    yaml_root_["image_width"] = cam_info_msg.height;
    yaml_root_["image_height"] = cam_info_msg.width;

    // Extract camera intrinsic matrix
    yaml_root_["camera_matrix"] = YAML::Node(YAML::NodeType::Map);
    yaml_root_["camera_matrix"]["rows"] = 3;
    yaml_root_["camera_matrix"]["cols"] = 3;
    YAML::Node intrinsics_parameters(YAML::NodeType::Sequence);
    for (auto & param : cam_info_msg.k) {
      intrinsics_parameters.push_back(param);
    }
    yaml_root_["camera_matrix"]["data"] = intrinsics_parameters;

    // Extract Distortion model and D vector
    yaml_root_["distortion_model"] = cam_info_msg.distortion_model;
    yaml_root_["distortion_coefficients"] = YAML::Node(YAML::NodeType::Map);
    yaml_root_["distortion_coefficients"]["rows"] = 1;
    yaml_root_["distortion_coefficients"]["cols"] = 5;
    YAML::Node distortion_parameters(YAML::NodeType::Sequence);
    for (auto & param : cam_info_msg.d) {
      distortion_parameters.push_back(param);
    }
    yaml_root_["distortion_coefficients"]["data"] = distortion_parameters;

    // Extract vectorised projection matrix
    yaml_root_["projection_matrix"] = YAML::Node(YAML::NodeType::Map);
    yaml_root_["projection_matrix"]["rows"] = 3;
    yaml_root_["projection_matrix"]["cols"] = 4;
    YAML::Node projection_parameters(YAML::NodeType::Sequence);
    for (auto & param : cam_info_msg.p) {
      projection_parameters.push_back(param);
    }
    yaml_root_["projection_matrix"]["data"] = projection_parameters;

    // Extract vectorised rectification matrix
    yaml_root_["rectification_matrix"] = YAML::Node(YAML::NodeType::Map);
    yaml_root_["rectification_matrix"]["rows"] = 3;
    yaml_root_["rectification_matrix"]["cols"] = 3;
    YAML::Node rectification_parameters(YAML::NodeType::Sequence);
    for (auto & param : cam_info_msg.r) {
      rectification_parameters.push_back(param);
    }
    yaml_root_["rectification_matrix"]["data"] = rectification_parameters;
  }

private:
  std::string topic_dir_;

  DataMeta data_meta_;

  YAML::Node yaml_root_;
};

}  // namespace rosbag2_exporter

#endif  // ROSBAG2_EXPORTER__HANDLERS__CAMERA_INFO_HANDLER_HPP_
