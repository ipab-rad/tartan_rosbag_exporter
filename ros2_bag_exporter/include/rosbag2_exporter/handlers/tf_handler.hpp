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

  void process_message(
    const rclcpp::SerializedMessage & serialized_msg, [[maybe_unused]] const std::string & topic,
    size_t global_id) override
  {
    // Deserialize the incoming message
    tf2_msgs::msg::TFMessage tf_msg;
    rclcpp::Serialization<tf2_msgs::msg::TFMessage> serializer;
    serializer.deserialize_message(&serialized_msg, &tf_msg);

    // Parse this message into yaml dictionary
    create_yaml(tf_msg);

    // Create the full file path with '.csv' as the extension
    std::string filepath = topic_dir_ + "/tf_tree" + ".yaml";

    data_meta_ = DataMeta{filepath, builtin_interfaces::msg::Time(), global_id};

    RCLCPP_DEBUG(logger_, "Successfully wrote GPS data to %s", filepath.c_str());
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
  void create_yaml(const tf2_msgs::msg::TFMessage & tf2_msg)
  {
    // Create a node for the sequence
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

    // Save the tf sequence ina top-level "transforms" key
    yaml_root_["transforms"] = transforms;
  }

private:
  std::string topic_dir_;

  DataMeta data_meta_;

  YAML::Node yaml_root_;
};

}  // namespace rosbag2_exporter

#endif  // ROSBAG2_EXPORTER__HANDLERS__TF_HANDLER_HPP_
