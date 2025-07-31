#ifndef ROSBAG2_EXPORTER__POINT_TYPES_HPP_
#define ROSBAG2_EXPORTER__POINT_TYPES_HPP_

#include <pcl/point_types.h>

namespace rosbag2_exporter
{

/**
 * Pointcloud data structure used by Nebula HESAI Lidar
 *  ROS driver.
 */

struct PointXYZIRCAEDT
{
  float x;
  float y;
  float z;
  std::uint8_t intensity;
  std::uint8_t return_type;
  std::uint16_t channel;
  float azimuth;
  float elevation;
  float distance;
  std::uint32_t time_stamp;
};

using NebulaPoint = PointXYZIRCAEDT;

}  // namespace rosbag2_exporter

POINT_CLOUD_REGISTER_POINT_STRUCT(
  rosbag2_exporter::PointXYZIRCAEDT,
  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(
    std::uint8_t, return_type,
    return_type)(std::uint16_t, channel, channel)(float, azimuth, azimuth)(
    float, elevation, elevation)(float, distance, distance)(std::uint32_t, time_stamp, time_stamp))

#endif  // ROSBAG2_EXPORTER__POINT_TYPES_HPP_
