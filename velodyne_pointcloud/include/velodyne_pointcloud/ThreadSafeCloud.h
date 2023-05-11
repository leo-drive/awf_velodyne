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
  explicit ThreadSafeCloud(const std::string & _frame_id);
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
};

}  // namespace velodyne_pointcloud
