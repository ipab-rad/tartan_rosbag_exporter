
# ros2_bag_exporter

## Overview

ros2_bag_exporter is  C++ package designed to export ROS 2 (Humble) bag files into various formats, including images, point cloud data (PCD) files, IMU data, and GPS data. This tool facilitates the extraction and conversion of data from bag files for analysis, visualisation, and processing outside the ROS ecosystem.

### Features
#### Support for Multiple Message Types:
- **PointCloud2**: Export point cloud data to PCD files.
- **Image**: Convert image messages to PNG format.
- **CompressedImage**: Convert image messages to JPG or PNG format.
- **CameraInfo**: Export camera calibration data.
- **IMU**: Export IMU data for inertial measurement analysis.
- **GPS**: Export GPS coordinates and data.
- **TF**: Export vehicle's static transformations.

#### Automatic Metadata generation
- Automatically generate metadata by associating sensor message timestamps. The output YAML will specify which camera messages correspond to timestamps from the main LIDAR sensor. This is particularly useful for data labelling tasks where the LIDAR serves as the primary reference sensor

#### Configurable Export Settings:
- Define input bag files, output directories, and storage formats via a YAML configuration file.

#### Robust Error Handling:
- Comprehensive logging for easy troubleshooting.
- Graceful shutdown on critical errors.

#### Automatic Directory Management:
- Automatically creates necessary directories for each topic based on the configuration.

## Install
### Dependencies
Ensure that the following dependencies are installed on your system:
- ROS 2 Humble Hawksbill
- C++17 Compiler
- YAML-CPP
- OpenCV
- PCL (Point Cloud Library)
- CV Bridge
- rosbag2_cpp
- ament_index_cpp

### Installing Dependencies
You can install the necessary dependencies using apt:
```bash
sudo apt update
sudo apt install -y ros-humble-rclcpp ros-humble-rosbag2-cpp ros-humble-rosbag2-storage \
libyaml-cpp-dev libopencv-dev  ros-humble-cv-bridge ros-humble-sensor-msgs \
ros-humble-pcl-conversions ros-humble-pcl-ros libpcl-dev ros-humble-ament-index-cpp
```

## Usage

### Configuration
The behavior of `ros2_bag_exporter` is controlled via a YAML configuration file. Please refer to the configuration file exaample located at `config/exporter_config.yaml`.

#### Configuration File Structure
```yaml
bag_path: "/absolute/path/to/your/my_rosbag.mcap"
output_dir: "/absolute/path/to/output/directory"
storage_id: "mcap"  # Common storage ID; ensure it matches your bag's storage format
topics:
  - name: "/camera/color/image_raw"
    type: "Image"
    encoding: "rgb8"
    sample_interval: 5   # Write one sample every 5 messages
    topic_dir: "img/color_raw"  # Output data in <output_dir>/<bag_name>/img/color_raw
  - name: "/camera/color/image_raw/compressed"
    type: "CompressedImage"
    sample_interval: 5   # Write one sample every 5 messages
    topic_dir: "img/compressed"
  - name: "/camera/color/image_raw/camera_info"
    type: "CameraInfo"
    sample_interval: 0   # Not used for this type
    topic_dir: "img/compressed/calibration"
  - name: "/imu_topic"
    type: "IMU"
    sample_interval: 100  # Write one sample every 100 messages
    topic_dir: imu
  - name: "/gps_topic"
    type: "GPS"
    sample_interval: 100  # Write one sample every 100 messages
    topic_dir: gps
  - name: "/lidar/points"
    type: "PointCloud2"
    sample_interval: 10   # Write one sample every 10 messages
    topic_dir: "lidar"
  - name: "/tf_static"
    type: "TF"
    sample_interval: 0   # Not used for this type
    topic_dir: "transforms"
```

#### Parameter Descriptions
- `bag_path`: The absolute path to the ROS 2 bag file you wish to export.
- `output_dir`: The absolute path to the directory where exported files will be saved.
- `storage_id`: Specifies the storage format of the bag file. Common values include:
  - `sqlite3`: Default storage for ROS 2 bags.
  - `mcap`: For MCAP storage format.
- `topics`: A list of topics to export. Each topic requires:
  - `name`: The ROS 2 topic name.
  - `type`: The message type (`PointCloud2`, `Image`, `CompressedImage`,`CameraInfo`,`IMU`, `GPS` and `TF`).
  - `sample_interval`: The interval at which messages will be exported (e.g., 1 for every 2 messages).
  - `topic_dir`: The name of the directory where the sensor data from the topic will be saved. The directory is created in `<output_dir>/<bag_name>/`.

### Run
After building and sourcing the workspace, run the `bag_exporter` node using the following command:
```bash
ros2 run ros2_bag_exporter bag_exporter --ros-args -p config_file:=<path_to_config>
```
#### Command-Line Arguments
- `<path_to_config>`: Specify a custom path to the YAML configuration file.

### Output

The extractor will generate an `export_metadata.yaml` file inside the `<output_dir>/<bag_name>` directory.

This metadata file provides details about synchronisation between camera images and point clouds, organised into a list of `sync_groups`. Additionally, the names of the original rosbags used will be saved under the `rosbags` field.



```yaml
rosbags:
  - my_rosbag.mcap
time_sync_groups:
  - id: 0
    stamp:
      sec: 1733409913
      nanosec: 451862784
    lidar:
      global_id: 6
      file: lidar/top/1733409913-451862784.pcd
    cameras:
      - global_id: 7
        name: fsp_l
        file: camera/fsp_l/1733409913-471454988.jpg
      - global_id: 24
        name: rsp_l
        file: camera/rsp_l/1733409913-471454988.jpg
        ...
  - id: 1
    stamp:
      sec: 1733409913
      nanosec: 551808768
    lidar:
      global_id: 20
      file: lidar/top/1733409913-551808768.pcd
    cameras:
      - global_id: 17
        name: fsp_l
        file: camera/fsp_l/1733409913-570909847.jpg
      - global_id: 32
        name: rsp_l
      ...
```

## Troubleshooting
### 1. Configuration File Loading Error
**Error:**
```bash
[ERROR] [rosbag2_exporter]: Failed to load configuration: bad file: config/exporter_config.yaml
```
**Solution:**
Verify the existence and correctness of the YAML file.

### 2. Bag Storage Initialization Error
**Error:**
```bash
[ERROR] [rosbag2_storage]: No storage id specified, and no plugin found that could open URI
```
**Solution:**
Ensure that `storage_id` matches the bag file storage format (e.g., `sqlite3`).
