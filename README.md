# Tartan Rosbag Exporter

![License](https://img.shields.io/github/license/ros2/rosbag2)
![ROS2 Version](https://img.shields.io/badge/ROS2-Humble%20Hawksbill-brightgreen)
![Issues](https://img.shields.io/github/issues/ipab-rad/tartan_rosbag_exporter)

## Overview

This repository provides customised tools based on the original [Geekgineer/ros2_bag_exporter](https://github.com/Geekgineer/ros2_bag_exporter) project, tailored specifically for the University of Edinburgh's Autonomous Vehicle project.

The repository includes tools and a Docker environment for reading and exporting sensor data from ROS 2 bag files, facilitating data post-processing outside the ROS 2 ecosystem.

For general-purpose functionality, please refer to the original repository linked above.

## Usage

### Build and run the Docker container

To build and run the Docker container interactively, use:

```bash
./runtime.sh -p <rosbags_directory> -o <exported_data_directory>
```

where:

- `<rosbags_directory>`: Parent directory containing your ROS bags recordings
- `<exported_data_directory>`: Parent directory where the data is/will be exported.

The input directories will be mounted in `/opt/ros_ws/rosbags` and `/opt/ros_ws/exported_data` in the container respectively.

### Export your ROS bags

Once inside the Docker container, run the following command:

```bash
cd /opt/ros_ws

ros2 run ros2_bag_exporter bag_exporter --ros-args \
  -p rosbags_directory:=./rosbags/<my_recording_directory> \
  -p output_directory:=./exported_data \
  -p config_file:=./config/av_sensor_export_config.yaml
```

You will find the exported data inside a directory in `/opt/ros_ws/exported_data`. Please see [ros2_bag_exporter](./ros2_bag_exporter/README.md) for further reference.

### Run in development mode

If you plan to modify this package, use `dev.sh` instead to run docker container in development mode. The `ros2_bag_exporter` package will be mounted in `/opt/ros_ws/src` automatically and an alias `colcon_build` will be available for compilation.

```bash
./dev.sh -p <rosbags_directory> -o <exported_data_directory>

# rosbag_expoter@<your_machine>:/opt/ros_ws$ colcon_build
# Starting >>> ros2_bag_exporter
# Finished <<< ros2_bag_exporter [15.7s]
#
# Summary: 1 package finished [16.4s]
```


## License
This project is licensed under the Apache License 2.0.


## Contact
Maintainer: Hector Cruz ([hect95](https://github.com/hect95))

Email: hcruzgo@ed.ac.uk
