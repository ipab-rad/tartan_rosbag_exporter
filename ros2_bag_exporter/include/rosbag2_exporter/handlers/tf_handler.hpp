/*
 * Author: Hector Cruz <hcruzgo@ed.ac.uk>
 * Date: 17.03.2025
 */

#ifndef ROSBAG2_EXPORTER__HANDLERS__TF_HANDLER_HPP_
#define ROSBAG2_EXPORTER__HANDLERS__TF_HANDLER_HPP_

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/logging.hpp"
#include "rosbag2_exporter/handlers/base_handler.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include <yaml-cpp/yaml.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>

namespace rosbag2_exporter
{

class TFHandler : public BaseHandler
{
public:
  // Constructor to accept logger
  TFHandler(const std::string & topic_dir, rclcpp::Logger logger)
  : BaseHandler(logger), topic_dir_(topic_dir)
  {
  }

  void process_message(const rclcpp::SerializedMessage & serialized_msg, size_t global_id) override
  {
    // Deserialize the incoming message
    tf2_msgs::msg::TFMessage tf_msg;
    rclcpp::Serialization<tf2_msgs::msg::TFMessage> serializer;
    serializer.deserialize_message(&serialized_msg, &tf_msg);

    // Save transforms count in the TF message
    size_t transformations_count = tf_msg.transforms.size();
    tranformations_count_map_.insert({transformations_count, tf_msg});
  }

  bool save_msg_to_file([[maybe_unused]] size_t index) override
  {
    if (tranformations_count_map_.empty()) {
      // Nothing to save
      return false;
    }

    // Get the map pair with the TF message containing the most transformations
    // Use a reverse iterator (rbegin) to access the entry with the largest key
    auto max_tfs_map_pair = tranformations_count_map_.rbegin();
    YAML::Node yaml_root = create_yaml(max_tfs_map_pair->second);

    // Serialise the YAML node to a string
    YAML::Emitter out;
    out << yaml_root;

    // Create the full file path with '.yaml' as the extension
    std::string filepath = topic_dir_ + "/transforms" + ".yaml";

    // Write to file
    std::ofstream fout(filepath);
    fout << out.c_str();
    fout.close();

    return true;
  }

private:
  YAML::Node create_yaml(const tf2_msgs::msg::TFMessage & tf2_msg)
  {
    YAML::Node yaml_root;
    YAML::Node transforms(YAML::NodeType::Sequence);

    // Parse all the TFs
    for (auto transform : tf2_msg.transforms) {
      YAML::Node tf_node;
      tf_node["parent_frame"] = transform.header.frame_id;
      tf_node["child_frame"] = transform.child_frame_id;
      tf_node["transform"]["x"] = transform.transform.translation.x;
      tf_node["transform"]["y"] = transform.transform.translation.y;
      tf_node["transform"]["z"] = transform.transform.translation.z;
      tf_node["transform"]["qx"] = transform.transform.rotation.x;
      tf_node["transform"]["qy"] = transform.transform.rotation.y;
      tf_node["transform"]["qz"] = transform.transform.rotation.z;
      tf_node["transform"]["qw"] = transform.transform.rotation.w;
      transforms.push_back(tf_node);
    }

    // Save the tf sequence in a top-level "transforms" key in YAML root
    yaml_root["transforms"] = transforms;

    return yaml_root;
  }

private:
  std::string topic_dir_;

  std::map<size_t, tf2_msgs::msg::TFMessage> tranformations_count_map_;
};

}  // namespace rosbag2_exporter

#endif  // ROSBAG2_EXPORTER__HANDLERS__TF_HANDLER_HPP_
