/*
 * Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer
 * Date: 13.10.2024
 */

#ifndef ROSBAG2_EXPORTER__HANDLERS__BASE_HANDLER_HPP_
#define ROSBAG2_EXPORTER__HANDLERS__BASE_HANDLER_HPP_

#include <string>
#include <vector>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rclcpp/logger.hpp>
#include "builtin_interfaces/msg/time.hpp"


namespace rosbag2_exporter
{

struct DataMeta
{
  std::string data_path;
  builtin_interfaces::msg::Time timestamp;
  size_t global_id;

  DataMeta() = default;

  DataMeta(std::string _data_path, builtin_interfaces::msg::Time _timestamp, size_t _global_id)
      : data_path(std::move(_data_path)), timestamp(std::move(_timestamp)), global_id(std::move(_global_id)) {}

};

class BaseHandler
{
public:
  virtual ~BaseHandler() = default;
  
  // Pure virtual function to process messages
  virtual void process_message(const rclcpp::SerializedMessage & serialized_msg,
                               const std::string & topic,
                               size_t global_id) = 0;

  virtual bool save_msg_to_file(size_t index) = 0;

public:
  std::vector<DataMeta> data_meta_vec_;

protected:
  rclcpp::Logger logger_;

  // Constructor to initialize the logger
  BaseHandler(rclcpp::Logger logger) : logger_(logger) {}
};

}  // namespace rosbag2_exporter

#endif  // ROSBAG2_EXPORTER__HANDLERS__BASE_HANDLER_HPP_
