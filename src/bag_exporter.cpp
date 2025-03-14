/*
 * Author: Abdalrahman M. Amer, www.linkedin.com/in/abdalrahman-m-amer
 * Date: 13.10.2024
 */

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rosbag2_exporter/bag_exporter.hpp"
#include "rosbag2_exporter/utils.hpp"
#include <chrono>
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace rosbag2_exporter
{

BagExporter::BagExporter(const rclcpp::NodeOptions & options)
: Node("rosbag2_exporter", options)
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
    
  // Declare and get the config_file parameter (absolute path)
  std::string config_file = this->declare_parameter<std::string>(
      "config_file", package_share_directory + "/config/exporter_config.yaml");

  // Load configuration
  load_configuration(config_file);

  // Setup handlers based on topics
  setup_handlers();

  auto start_time_point = std::chrono::high_resolution_clock::now();

  // Start exporting
  export_bag();

  // Create Metadata
  create_metadata_file();

  auto end_time_point = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> duration = end_time_point - start_time_point;
  double elapsed_time = duration.count();

  RCLCPP_INFO(this->get_logger(), "Sensor data exportation took %f secs \U0001F525", elapsed_time);
}

void BagExporter::load_configuration(const std::string & config_file)
{
  try {
    RCLCPP_INFO(this->get_logger(), "Loading config from: %s", config_file.c_str());

    YAML::Node config = YAML::LoadFile(config_file);
    bag_path_ = config["bag_path"].as<std::string>();
    output_dir_ = config["output_dir"].as<std::string>();
    storage_id_ = config["storage_id"].as<std::string>();
    
    RCLCPP_INFO(this->get_logger(), "Reading from:\n \033[35m%s\033[0m", bag_path_.c_str());

    for (const auto & topic : config["topics"]) {
      TopicConfig tc;
      tc.name = topic["name"].as<std::string>();
      std::string type = topic["type"].as<std::string>();
      tc.topic_dir = topic["topic_dir"].as<std::string>();

      tc.sample_interval = topic["sample_interval"] ? topic["sample_interval"].as<int>() : 1;  // Default to 1 (write every message)

      if (type == "PointCloud2") {
        tc.type = MessageType::PointCloud2;
      } else if (type == "Image") {
        tc.type = MessageType::Image;
        tc.encoding = topic["encoding"] ? topic["encoding"].as<std::string>() : "rgb8"; // default encoding
      } else if (type == "CompressedImage") {
        tc.type = MessageType::CompressedImage;
        tc.encoding = topic["encoding"] ? topic["encoding"].as<std::string>() : "rgb8"; // default encoding
      } else if (type == "IMU") {
        tc.type = MessageType::IMU;
      } else if (type == "GPS") {
        tc.type = MessageType::GPS;
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

void BagExporter::setup_handlers()
{
  // Extract base name from rosbag file
  rosbag_base_name_ = std::filesystem::path(bag_path_).stem().string();
  
  for (const auto & topic : topics_) {
    // Create directory for each topic
    std::string abs_topic_dir = output_dir_ + "/" + rosbag_base_name_ + "/" + topic.topic_dir;

    // Ensure the directory exists, create if necessary
    if (!std::filesystem::exists(abs_topic_dir)) {
      std::filesystem::create_directories(abs_topic_dir);
    }

    // Initialize handler based on message type
    if (topic.type == MessageType::PointCloud2) {
      auto handler = std::make_shared<PointCloudHandler>(abs_topic_dir, this->get_logger());
      handlers_[topic.name] = Handler{handler, 0};
    } else if (topic.type == MessageType::Image) {
      auto handler = std::make_shared<ImageHandler>(abs_topic_dir, topic.encoding, this->get_logger());
      handlers_[topic.name] = Handler{handler, 0};
    } else if (topic.type == MessageType::CompressedImage) {
      auto handler = std::make_shared<CompressedImageHandler>(abs_topic_dir, topic.encoding, this->get_logger());
      handlers_[topic.name] = Handler{handler, 0};
    } else if (topic.type == MessageType::IMU) {
      auto handler = std::make_shared<IMUHandler>(abs_topic_dir, this->get_logger());
      handlers_[topic.name] = Handler{handler, 0};
    } else if (topic.type == MessageType::GPS) {
      auto handler = std::make_shared<GPSHandler>(abs_topic_dir, this->get_logger());
      handlers_[topic.name] = Handler{handler, 0};
    } else {
      RCLCPP_WARN(this->get_logger(), "Unsupported message type for topic '%s'. Skipping.", topic.name.c_str());
    }
  }
}
void BagExporter::export_bag()
{
  // Initialize reader
  rosbag2_cpp::readers::SequentialReader reader;
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path_;
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
    auto it = std::find_if(metadata.begin(), metadata.end(),
      [&topic_name](const rosbag2_storage::TopicInformation & ti) {
        return ti.topic_metadata.name == topic_name;
      });
    if (it == metadata.end()) {
      RCLCPP_WARN(this->get_logger(), "Topic '%s' not found in the bag \U0000274C", topic_name.c_str());
      handler.handler.reset(); // Remove handler if topic not found
    } else {
      RCLCPP_INFO(this->get_logger(), "'%s' topic found \U00002705", topic_name.c_str());
      total_bag_messages += it->message_count;
    }
  }

  RCLCPP_INFO(this->get_logger(), "Extracting data ...");

  // Global id counter
  size_t global_id = 0;
  // Read and process messages
  while (reader.has_next()) {
    auto serialized_msg = reader.read_next();
    std::string topic = serialized_msg->topic_name;

    auto handler_it = handlers_.find(topic);
    if (handler_it != handlers_.end() && handler_it->second.handler) {
      size_t current_index = handler_it->second.current_index;

      // Find the sample interval for the topic
      auto topic_it = std::find_if(topics_.begin(), topics_.end(),
        [&topic](const TopicConfig & config) {
          return config.name == topic;
        });

      if (topic_it != topics_.end()) {
        size_t sample_interval = topic_it->sample_interval;  // Get the sample interval

        // Only write the message if it matches the sampling rate
        if (current_index % sample_interval == 0) {
          // Construct rclcpp::SerializedMessage from serialized_data
          rclcpp::SerializedMessage ser_msg;
          size_t buffer_length = serialized_msg->serialized_data->buffer_length;
          ser_msg.reserve(buffer_length);
          std::memcpy(ser_msg.get_rcl_serialized_message().buffer, 
                      serialized_msg->serialized_data->buffer, 
                      buffer_length);
          ser_msg.get_rcl_serialized_message().buffer_length = buffer_length;

          // Process the message
          handler_it->second.handler->process_message(ser_msg, topic, global_id);
        }
      } else {
        RCLCPP_WARN(this->get_logger(), "No configuration found for topic: %s", topic.c_str());
      }

      handler_it->second.current_index++;
      global_id++;

      // Log progress
      print_progress(static_cast<int>(std::round((global_id * 100.0) / total_bag_messages)));
            
    }
  }

}

void BagExporter::create_metadata_file()
{

  // YAML C++ preserves list order, so topics_[0] corresponds to  
  // the first sensor defined in the YAML file, used as the time sync reference.
  auto & main_sensor_handler = handlers_[topics_[0].name].handler;
  RCLCPP_INFO(this->get_logger(), "Using '%s' timestamp for cameras' time synchronisation",
              topics_[0].name.c_str());

  std::vector<size_t> msg_index(topics_.size(), 0);

  // Define YAML root
  YAML::Node root;

  // Save which rosbag was used to extract the data
  root["rosbags"] = YAML::Node(YAML::NodeType::Sequence);
  root["rosbags"].push_back(std::filesystem::path(bag_path_).filename().string());

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
    sync_group["lidar"]["global_id"] = main_data_meta.global_id;

    // Save this pointcloud
    if (! main_sensor_handler->save_msg_to_file(idx)){
      RCLCPP_WARN(
        this->get_logger(), "Unable to save pointcloud msg as file, global_id: %li",
        main_data_meta.global_id);
    }

    // Save file name relative to rosbag_base_name_
    std::string abs_path_prefix = output_dir_ + "/" + rosbag_base_name_ + "/";
    if (main_data_meta.data_path.find(abs_path_prefix) == 0)
    {
        main_data_meta.data_path.erase(0, abs_path_prefix.length());
    }
    sync_group["lidar"]["file"] = main_data_meta.data_path;

    auto& curr_lidar_time = main_data_meta.timestamp;

    // Where the cameras' YAML data will be saved
    YAML::Node cameras;

    // Find closest timestamp for each camera
    for (int k = 0; k < topics_.size(); k++)
    {
        // Only process camera messages
        if (topics_[k].type != MessageType::Image &&
            topics_[k].type != MessageType::CompressedImage)
        {
            continue;
        }

        // Get current camera metadata
        auto &curr_cam_handler = handlers_[topics_[k].name].handler;
        auto &current_meta_data_vec = curr_cam_handler->data_meta_vec_;
        size_t closest_time_index = find_closest_timestamp(current_meta_data_vec,
                                                            curr_lidar_time, msg_index[k]);
        
        // Create YAML metadata based on the found time index
        auto current_cam_meta = current_meta_data_vec[closest_time_index];
        YAML::Node cam;
        cam["global_id"] = current_cam_meta.global_id;
        cam["name"] = get_cam_name(topics_[k].name);
        
        // Save this image
        if (!curr_cam_handler->save_msg_to_file(closest_time_index)) {
          RCLCPP_WARN(
            this->get_logger(), "Unable to save image msg as file, global_id: %li",
            main_data_meta.global_id);
        }

        // Save file name relative to rosbag_base_name_
        if (current_cam_meta.data_path.find(abs_path_prefix) == 0) { 
          current_cam_meta.data_path.erase(0, abs_path_prefix.length());  
        }
        cam["file"] = current_cam_meta.data_path;

        cameras.push_back(cam);
    }

    // Complete YAML sync group
    sync_group["cameras"] = cameras;
    root["time_sync_groups"].push_back(sync_group);

    ++sync_group_id;

    // Log progress
    print_progress(static_cast<int>(std::round((sync_group_id * 100.0) / total_sync_groups)));
  }

  // Save the file in the rosbag output directory
  std::string yaml_path = output_dir_ + "/" + rosbag_base_name_ + "/" + "export_metadata.yaml";
  std::ofstream yaml_file(yaml_path);
  yaml_file << root;
  yaml_file.close();

  RCLCPP_INFO(this->get_logger(), "\U0001F680 Metadata file created in: \033[36m%s\033[0m", yaml_path.c_str());
}

}  // namespace rosbag2_exporter

// Define the main function to initialize and run the node
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto exporter = std::make_shared<rosbag2_exporter::BagExporter>(rclcpp::NodeOptions());
  return 0;
}
