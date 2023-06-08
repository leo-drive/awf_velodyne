#pragma once

#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "velodyne_pointcloud/InvalidPointChecker.h"
#include "velodyne_pointcloud/datacontainerbase.h"
#include "velodyne_pointcloud/point_types.h"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <atomic>
#include <mutex>

namespace velodyne_pointcloud
{

class ThreadSafeCloud : public velodyne_rawdata::DataContainerBase
{
public:
  ThreadSafeCloud(
    rclcpp::Node * _node, const std::string & _frame_id,
    const InvalidPointChecker & _invalid_point_checker, double _scan_phase,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher_xyzir,
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher_xyziradt,
    velodyne_rawdata::DataContainerBase & _overflow_buffer);
  void addPoint(
    const float & x, const float & y, const float & z, const uint8_t & return_type,
    const uint16_t & ring, const uint16_t & azimuth, const float & distance,
    const float & intensity, const double & time_stamp) override;

  void publish_cloud(const rclcpp::Time & _time, const bool _called_from_different_thread = false);
  void set_pub_time(rclcpp::Time & _time);
  void ready_to_publish();
  void enable_azimuth_check();
  uint16_t get_last_azimuth();

private:
  std::mutex mutex_;
  pcl::PointCloud<velodyne_pointcloud::PointXYZIR> cloud_xyzir_;
  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT> cloud_xyziradt_;
  const InvalidPointChecker invalid_point_checker_;
  rclcpp::Node * node_;
  const double scan_phase_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_xyzir_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_xyziradt_;
  //  std::optional<uint16_t> first_azimuth_;
  rclcpp::Time pub_time_;
  std::atomic_bool is_ready_to_publish_{false};
  std::optional<uint16_t> last_azimuth_;
  uint16_t last_azimuth_int_;
  bool check_azimuth_{false};

  enum class AzimuthSign {
    POSITIVE,
    NEGATIVE,
  } azimuth_sign_{AzimuthSign::POSITIVE};

  std::optional<uint16_t> azimuth_max_;
  std::optional<uint16_t> azimuth_min_;

  velodyne_rawdata::DataContainerBase & overflow_buffer_;
};

}  // namespace velodyne_pointcloud
