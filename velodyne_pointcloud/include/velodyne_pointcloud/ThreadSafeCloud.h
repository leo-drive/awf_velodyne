#pragma once

#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "velodyne_pointcloud/datacontainerbase.h"
#include "velodyne_pointcloud/point_types.h"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <mutex>

namespace velodyne_pointcloud
{

class ThreadSafeCloud : public velodyne_rawdata::DataContainerBase
{
public:
  ThreadSafeCloud(
    const std::string & _frame_id, const bool & _is_remove_active,
    const std::vector<long> & _invalid_rings, const std::vector<long> & _invalid_angles_start,
    const std::vector<long> & _invalid_angles_end);
  void addPoint(
    const float & x, const float & y, const float & z, const uint8_t & return_type,
    const uint16_t & ring, const uint16_t & azimuth, const float & distance,
    const float & intensity, const double & time_stamp) override;

  void publish_cloud(
    const rclcpp::Time & _time,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher_xyzir,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher_xyziradt);

private:
  std::mutex mutex_;
  pcl::PointCloud<velodyne_pointcloud::PointXYZIR> cloud_xyzir_;
  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT> cloud_xyziradt_;
  const std::vector<long> invalid_rings_;
  const std::vector<long> invalid_angles_start_;
  const std::vector<long> invalid_angles_end_;
  const bool is_remove_active_;
};

}  // namespace velodyne_pointcloud
