/*
 * This file is based on the original work by:
 *
 * Original Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer
 * Original Date: 13.10.2024
 *
 * Adapted and Modified By: Hector Cruz, hcruzgo@ed.ac.uk
 * Adaptation Date: 24.01.2025
 *
 * Description of Adaptations:
 * - Used as a template to support CompressedImages handling
 */

#ifndef ROSBAG2_EXPORTER__HANDLERS__COMPRESSED_IMAGE_HANDLER_HPP_
#define ROSBAG2_EXPORTER__HANDLERS__COMPRESSED_IMAGE_HANDLER_HPP_

#include "rclcpp/logging.hpp"
#include "rosbag2_exporter/handlers/base_handler.hpp"

#include <sensor_msgs/msg/compressed_image.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace rosbag2_exporter
{

class CompressedImageHandler : public BaseHandler
{
public:
  CompressedImageHandler(
    const std::string & topic_dir, const std::string & encoding, rclcpp::Logger logger)
  : BaseHandler(logger), topic_dir_(topic_dir)
  {
  }

  // Handle compressed image messages
  void process_message(const rclcpp::SerializedMessage & serialized_msg, size_t global_id) override
  {
    // Deserialize the incoming compressed image message
    sensor_msgs::msg::CompressedImage compressed_img;
    rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serializer;
    serializer.deserialize_message(&serialized_msg, &compressed_img);

    // Determine file extension based on the compressed image format
    std::string extension;
    if (
      compressed_img.format.find("jpeg") != std::string::npos ||
      compressed_img.format.find("jpg") != std::string::npos) {
      extension = ".jpg";
    } else if (compressed_img.format.find("png") != std::string::npos) {
      extension = ".png";
    } else {
      // Only support .jpg and .png formats
      throw std::invalid_argument(
        "Unsupported compressed image format found: " + compressed_img.format);
    }

    // Create a timestamped filename and save compressed image directly
    std::stringstream ss_timestamp;
    ss_timestamp << compressed_img.header.stamp.sec << "-" << std::setw(9) << std::setfill('0')
                 << compressed_img.header.stamp.nanosec;
    std::string timestamp = ss_timestamp.str();

    // Create the full file path
    std::string filepath = topic_dir_ + "/" + timestamp + extension;

    // Save these for later
    data_meta_vec_.push_back(DataMeta{filepath, compressed_img.header.stamp, global_id});
    data_vec_.push_back(compressed_img);
  }

  bool save_msg_to_file(size_t index) override
  {
    // Check index bounds
    if (index >= data_meta_vec_.size() || index >= data_vec_.size()) {
      RCLCPP_ERROR(logger_, "[CompressedImageHandler] Provided index is out of range");
      return false;
    }

    DataMeta & data_meta = data_meta_vec_[index];
    sensor_msgs::msg::CompressedImage & compressed_image = data_vec_[index];

    // Save the compressed image data directly to file
    std::ofstream outfile(data_meta.data_path, std::ios::binary);
    if (!outfile.is_open()) {
      RCLCPP_ERROR(
        logger_, "Failed to open file to write compressed image: %s", data_meta.data_path.c_str());
      return false;
    }
    outfile.write(
      reinterpret_cast<const char *>(compressed_image.data.data()), compressed_image.data.size());
    outfile.close();

    RCLCPP_DEBUG(
      logger_, "Successfully wrote compressed image to '%s' ", data_meta.data_path.c_str());
    return true;
  }

private:
  std::string topic_dir_;
  // Defined to comply with class parent but not needed
  std::string encoding_;

  // Data vector
  std::vector<sensor_msgs::msg::CompressedImage> data_vec_;
};

}  // namespace rosbag2_exporter

#endif  // ROSBAG2_EXPORTER__HANDLERS__COMPRESSED_IMAGE_HANDLER_HPP_
