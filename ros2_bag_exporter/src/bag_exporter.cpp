/*
 * Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer
 * Date: 13.10.2024
 */

#include "rosbag2_exporter/bag_exporter.hpp"

#include "rosbag2_exporter/utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <fstream>

namespace rosbag2_exporter
{

BagExporter::BagExporter(const rclcpp::NodeOptions & options)
: Node("rosbag2_exporter", options), global_id_(0)
{
  // Find the package share directory
  std::string package_share_directory;
  try {
    package_share_directory = ament_index_cpp::get_package_share_directory("ros2_bag_exporter");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Package share directory not found: %s", e.what());
    rclcpp::shutdown();
    return;
  }

  rosbags_directory_ =
    this->declare_parameter<std::string>("rosbags_directory", "/my/path/to/rosbags");

  output_directory_ =
    this->declare_parameter<std::string>("output_directory", "/my/path/to/output");

  if (!std::filesystem::is_directory(rosbags_directory_)) {
    RCLCPP_ERROR(this->get_logger(), "%s is not a directory!", rosbags_directory_.c_str());
    rclcpp::shutdown();
    return;
  }

  std::string config_file = this->declare_parameter<std::string>(
    "config_file", package_share_directory + "/config/exporter_config.yaml");

  load_configuration(config_file);

  auto start_time_point = std::chrono::high_resolution_clock::now();

  process_rosbag_directory();

  auto end_time_point = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> duration = end_time_point - start_time_point;
  double elapsed_time = duration.count();

  RCLCPP_INFO(this->get_logger(), "Sensor data exportation took %f secs 🔥", elapsed_time);
}

void BagExporter::load_configuration(const std::string & config_file)
{
  try {
    RCLCPP_INFO(this->get_logger(), "Loading config from: %s", config_file.c_str());

    YAML::Node config = YAML::LoadFile(config_file);
    storage_id_ = config["storage_id"].as<std::string>();

    for (const auto & topic : config["topics"]) {
      TopicConfig tc;
      tc.name = topic["name"].as<std::string>();
      std::string type = topic["type"].as<std::string>();
      tc.topic_dir = topic["topic_dir"].as<std::string>();

      tc.sample_interval = topic["sample_interval"] ? topic["sample_interval"].as<int>()
                                                    : 1;  // Default to 1 (write every message)

      if (type == "PointCloud2") {
        tc.type = MessageType::PointCloud2;
      } else if (type == "Image") {
        tc.type = MessageType::Image;
        tc.encoding =
          topic["encoding"] ? topic["encoding"].as<std::string>() : "rgb8";  // default encoding
      } else if (type == "CompressedImage") {
        tc.type = MessageType::CompressedImage;
        tc.encoding =
          topic["encoding"] ? topic["encoding"].as<std::string>() : "rgb8";  // default encoding
      } else if (type == "CameraInfo") {
        tc.type = MessageType::CameraInfo;
      } else if (type == "IMU") {
        tc.type = MessageType::IMU;
      } else if (type == "GPS") {
        tc.type = MessageType::GPS;
      } else if (type == "TF") {
        tc.type = MessageType::TF;
      } else {
        tc.type = MessageType::Unknown;
      }

      topics_.push_back(tc);
    }
  } catch (const YAML::BadFile & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load configuration: %s", e.what());
    rclcpp::shutdown();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "An error occurred while loading configuration: %s", e.what());
    rclcpp::shutdown();
  }
}

void BagExporter::setup_handlers(const fs::path & output_directory_path)
{
  // Extract base name from rosbag file
  std::string output_directory = output_directory_path.string();

  for (const auto & topic : topics_) {
    // Create directory for each topic
    std::string abs_topic_dir = output_directory + "/" + topic.topic_dir;

    // Ensure the directory exists, create if necessary
    if (!std::filesystem::exists(abs_topic_dir)) {
      std::filesystem::create_directories(abs_topic_dir);
    }

    // Initialize handler based on message type
    if (topic.type == MessageType::PointCloud2) {
      auto handler = std::make_shared<PointCloudHandler>(abs_topic_dir, this->get_logger());
      handlers_[topic.name] = Handler{handler, 0};
    } else if (topic.type == MessageType::Image) {
      auto handler =
        std::make_shared<ImageHandler>(abs_topic_dir, topic.encoding, this->get_logger());
      handlers_[topic.name] = Handler{handler, 0};
    } else if (topic.type == MessageType::CompressedImage) {
      auto handler =
        std::make_shared<CompressedImageHandler>(abs_topic_dir, topic.encoding, this->get_logger());
      handlers_[topic.name] = Handler{handler, 0};
    } else if (topic.type == MessageType::CameraInfo) {
      auto handler = std::make_shared<CameraInfoHandler>(abs_topic_dir, this->get_logger());
      handlers_[topic.name] = Handler{handler, 0};
      cam_info_topics_.push_back(topic.name);
    } else if (topic.type == MessageType::IMU) {
      auto handler = std::make_shared<IMUHandler>(abs_topic_dir, this->get_logger());
      handlers_[topic.name] = Handler{handler, 0};
    } else if (topic.type == MessageType::GPS) {
      auto handler = std::make_shared<GPSHandler>(abs_topic_dir, this->get_logger());
      handlers_[topic.name] = Handler{handler, 0};
    } else if (topic.type == MessageType::TF) {
      auto handler = std::make_shared<TFHandler>(abs_topic_dir, this->get_logger());
      handlers_[topic.name] = Handler{handler, 0};
    } else {
      RCLCPP_WARN(
        this->get_logger(), "Unsupported message type for topic '%s'. Skipping.",
        topic.name.c_str());
    }
  }
}
void BagExporter::extract_data(const fs::path & rosbag)
{
  // Initialize reader
  rosbag2_cpp::readers::SequentialReader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = rosbag.string();
  storage_options.storage_id = storage_id_;
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.input_serialization_format = "cdr";
  converter_options.output_serialization_format = "cdr";

  try {
    reader.open(storage_options, converter_options);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open bag: %s", e.what());
    rclcpp::shutdown();
    return;
  }

  // Get topic metadata
  auto metadata = reader.get_metadata().topics_with_message_count;

  // Initialize handlers based on available topics
  size_t total_bag_messages = 0;
  for (auto & [topic_name, handler] : handlers_) {
    auto it = std::find_if(
      metadata.begin(), metadata.end(),
      [&topic_name](const rosbag2_storage::TopicInformation & ti) {
        return ti.topic_metadata.name == topic_name;
      });
    if (it == metadata.end()) {
      RCLCPP_WARN(this->get_logger(), "Topic '%s' not found in the bag ❌", topic_name.c_str());
      handler.handler.reset();  // Remove handler if topic not found
    } else {
      RCLCPP_INFO(this->get_logger(), "'%s' topic found ✅", topic_name.c_str());

      auto cam_info_topic_it = std::find_if(
        cam_info_topics_.begin(), cam_info_topics_.end(),
        [&topic_name](const std::string & topic) { return topic == topic_name; });

      // Only one camera info message will be saved per topic
      if (cam_info_topic_it != cam_info_topics_.end()) {
        total_bag_messages += 1;
      } else {
        total_bag_messages += it->message_count;
      }
    }
  }

  RCLCPP_INFO(this->get_logger(), "Extracting data ...");

  size_t camera_info_extracted_n = 0;
  bool all_cameras_info_extracted = false;
  bool tf_extracted = false;
  int progress = 0;

  // Read and process messages
  while (reader.has_next()) {
    auto serialized_msg = reader.read_next();
    std::string topic = serialized_msg->topic_name;

    auto handler_it = handlers_.find(topic);
    if (handler_it != handlers_.end() && handler_it->second.handler) {
      // Only process /tf_static once
      if (topic == "/tf_static") {
        if (!tf_extracted) {
          // Construct rclcpp::SerializedMessage from serialized_data
          rclcpp::SerializedMessage ser_msg(*serialized_msg->serialized_data);
          handlers_["/tf_static"].handler->process_message(ser_msg, global_id_);
          if (!handlers_["/tf_static"].handler->save_msg_to_file(0)) {
            throw std::runtime_error("Failed to save tf_static message to file");
          }
          tf_extracted = true;
        }

        global_id_ += 1;
        progress++;
        continue;
      }

      if (!all_cameras_info_extracted) {
        // Find if the current topic is one of the camera's info
        auto cam_info_topic_it = std::find_if(
          cam_info_topics_.begin(), cam_info_topics_.end(),
          [&topic](const std::string & topic_name) { return topic_name == topic; });

        if (cam_info_topic_it != cam_info_topics_.end()) {
          // If the current topic is a CameraInfo msg, process it.
          auto cam_info_handler = handlers_[topic];
          rclcpp::SerializedMessage ser_msg(*serialized_msg->serialized_data);

          cam_info_handler.handler->process_message(ser_msg, global_id_);
          if (!cam_info_handler.handler->save_msg_to_file(0)) {
            throw std::runtime_error("Failed to save camera info message to file");
          }

          // Increase counter
          camera_info_extracted_n += 1;

          // Remove this camera info handler so we do not
          // evaluate this topic again
          handler_it->second.handler.reset();

          global_id_ += 1;
          progress++;

          // Check if we are done with all the camera info messages
          if (camera_info_extracted_n == cam_info_topics_.size()) {
            all_cameras_info_extracted = true;
          }
          continue;
        }
      }

      size_t current_index = handler_it->second.current_index;

      // Find the sample interval for the topic
      auto topic_it = std::find_if(
        topics_.begin(), topics_.end(),
        [&topic](const TopicConfig & config) { return config.name == topic; });

      if (topic_it != topics_.end()) {
        size_t sample_interval = topic_it->sample_interval;  // Get the sample interval

        // Only write the message if it matches the sampling rate
        if (current_index % sample_interval == 0) {
          // Construct rclcpp::SerializedMessage from serialized_data
          rclcpp::SerializedMessage ser_msg(*serialized_msg->serialized_data);

          // Process the message
          handler_it->second.handler->process_message(ser_msg, global_id_);
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "No configuration found for topic: %s", topic.c_str());
      }

      handler_it->second.current_index++;
      global_id_++;
      progress++;

      // Log progress
      utils::print_progress(static_cast<int>(std::round((progress * 100.0) / total_bag_messages)));
    }
  }
}

void BagExporter::export_data(const fs::path & used_rosbag, const fs::path & output_directoy_path)
{
  // YAML C++ preserves list order, so topics_[0] corresponds to
  // the first sensor defined in the YAML file, used as the time sync reference.
  auto & main_sensor_handler = handlers_[topics_[0].name].handler;
  RCLCPP_INFO(
    this->get_logger(),
    "Using " BOLD_LOG "%s" COLOR_RESET " timestamp for cameras' time synchronisation",
    topics_[0].name.c_str());

  std::vector<size_t> msg_index(topics_.size(), 0);

  // Define YAML root
  YAML::Node root;

  // Save which rosbag was used to extract the data
  root["rosbags"] = YAML::Node(YAML::NodeType::Sequence);
  root["rosbags"].push_back(used_rosbag.filename().string());

  size_t sync_group_id = 0;
  size_t total_sync_groups = main_sensor_handler->data_meta_vec_.size();

  // Co-relate other sensors timestamps based on main sensor
  RCLCPP_INFO(this->get_logger(), "Creating metadata and exporting messages as files ...");
  for (size_t idx = 0; idx < total_sync_groups; ++idx) {
    DataMeta & main_data_meta = main_sensor_handler->data_meta_vec_[idx];
    YAML::Node sync_group;
    sync_group["id"] = sync_group_id;
    sync_group["stamp"]["sec"] = main_data_meta.timestamp.sec;
    sync_group["stamp"]["nanosec"] = main_data_meta.timestamp.nanosec;
    sync_group["lidar"]["global_id_"] = main_data_meta.global_id;

    // Save this pointcloud
    if (!main_sensor_handler->save_msg_to_file(idx)) {
      throw std::runtime_error(
        "Unable to save pointcloud msg as file, global_id: " +
        std::to_string(main_data_meta.global_id));
    }

    // Save file name relative to rosbag_base_name_
    fs::path lidar_relative_path = fs::relative(main_data_meta.data_path, output_directoy_path);
    sync_group["lidar"]["file"] = lidar_relative_path.string();

    auto & curr_lidar_time = main_data_meta.timestamp;

    // Where the cameras' YAML data will be saved
    YAML::Node cameras;

    // Find closest timestamp for each camera
    for (int k = 0; k < topics_.size(); k++) {
      // Only process camera messages
      if (
        topics_[k].type != MessageType::Image && topics_[k].type != MessageType::CompressedImage) {
        continue;
      }

      // Get current camera metadata
      auto & curr_cam_handler = handlers_[topics_[k].name].handler;
      auto & current_meta_data_vec = curr_cam_handler->data_meta_vec_;
      size_t closest_time_index =
        utils::find_closest_timestamp(current_meta_data_vec, curr_lidar_time, msg_index[k]);

      // Create YAML metadata based on the found time index
      auto current_cam_meta = current_meta_data_vec[closest_time_index];
      YAML::Node cam;
      cam["global_id_"] = current_cam_meta.global_id;
      cam["name"] = utils::get_cam_name(topics_[k].name);

      // Save this image
      if (!curr_cam_handler->save_msg_to_file(closest_time_index)) {
        throw std::runtime_error(
          "Unable to save image msg as file, global_id_: " +
          std::to_string(main_data_meta.global_id));
      }

      // Save file name relative to the output directory path
      fs::path cam_relative_path = fs::relative(current_cam_meta.data_path, output_directoy_path);
      cam["file"] = cam_relative_path.string();

      cameras.push_back(cam);
    }

    // Complete YAML sync group
    sync_group["cameras"] = cameras;
    root["time_sync_groups"].push_back(sync_group);

    ++sync_group_id;

    // Log progress
    utils::print_progress(
      static_cast<int>(std::round((sync_group_id * 100.0) / total_sync_groups)));
  }

  // Save the file in the rosbag output directory
  fs::path yaml_path = output_directoy_path / std::string("export_metadata.yaml");
  std::ofstream yaml_file(yaml_path.string());
  yaml_file << root;
  yaml_file.close();
}

void BagExporter::process_rosbag_directory()
{
  RCLCPP_INFO(
    this->get_logger(), "Processing: " MAGENTA_LOG "%s" COLOR_RESET, rosbags_directory_.c_str());

  // Get all the rosbags files
  auto rosbag_directory_path = std::filesystem::path(rosbags_directory_);
  auto rosbags_files = utils::find_rosbags(rosbag_directory_path);

  // Create a new directory for the output
  std::string new_rosbag_directory_name = utils::get_new_name(rosbag_directory_path);
  fs::path new_output_dir = fs::path(output_directory_) / new_rosbag_directory_name;
  fs::create_directories(new_output_dir);

  RCLCPP_INFO(
    this->get_logger(), "Found " UNDERLINE_LOG "%zu" COLOR_RESET " rosbags in the directory",
    rosbags_files.size());

  for (const auto & rosbag : rosbags_files) {
    RCLCPP_INFO(
      this->get_logger(), "Processing rosbag: " CYAN_LOG "%s" COLOR_RESET,
      rosbag.filename().string().c_str());

    // Extract base name from rosbag file
    size_t rosbag_index = utils::get_rosbag_index(rosbag);
    fs::path split_export_directory =
      new_output_dir / std::string("sequence_" + std::to_string(rosbag_index));

    setup_handlers(split_export_directory);

    extract_data(rosbag);

    export_data(rosbag, split_export_directory);
  }

  RCLCPP_INFO(
    this->get_logger(), "🚀 Data exported in: " GREEN_LOG "%s" COLOR_RESET,
    new_output_dir.string().c_str());
}

}  // namespace rosbag2_exporter

// Define the main function to initialize and run the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto exporter = std::make_shared<rosbag2_exporter::BagExporter>(rclcpp::NodeOptions());
  return 0;
}
