#include <velodyne_pointcloud/ThreadSafeCloud.h>

namespace velodyne_pointcloud
{
ThreadSafeCloud::ThreadSafeCloud(
  const std::string & _frame_id, const bool & _is_remove_active,
  const std::vector<long> & _invalid_rings, const std::vector<long> & _invalid_angles_start,
  const std::vector<long> & _invalid_angles_end)
: invalid_rings_{_invalid_rings},
  invalid_angles_start_{_invalid_angles_start},
  invalid_angles_end_{_invalid_angles_end},
  is_remove_active_{_is_remove_active}
{
  cloud_xyzir_.header.frame_id = _frame_id;
  cloud_xyzir_.height = 1;

  cloud_xyziradt_.header.frame_id = _frame_id;
  cloud_xyziradt_.height = 1;
}

void ThreadSafeCloud::addPoint(
  const float & x, const float & y, const float & z, const uint8_t & return_type,
  const uint16_t & ring, const uint16_t & azimuth, const float & distance, const float & intensity,
  const double & time_stamp)
{
  // Check for invalid rings
  if (is_remove_active_) {
    const auto iter = std::find(invalid_rings_.begin(), invalid_rings_.end(), ring);
    if (iter != invalid_rings_.end()) {
      const auto index = iter - invalid_rings_.begin();
      if (azimuth >= invalid_angles_start_[index] && azimuth <= invalid_angles_end_[index]) {
        return;
      }
    }
  }

  velodyne_pointcloud::PointXYZIR point_xyzir;
  point_xyzir.x = x;
  point_xyzir.y = y;
  point_xyzir.z = z;
  point_xyzir.intensity = intensity;
  point_xyzir.ring = ring;

  velodyne_pointcloud::PointXYZIRADT point_xyziradt;
  point_xyziradt.x = x;
  point_xyziradt.y = y;
  point_xyziradt.z = z;
  point_xyziradt.intensity = intensity;
  point_xyziradt.return_type = return_type;
  point_xyziradt.ring = ring;
  point_xyziradt.azimuth = azimuth;
  point_xyziradt.distance = distance;
  point_xyziradt.time_stamp = time_stamp;

  std::lock_guard<std::mutex> lock(mutex_);

  cloud_xyzir_.points.push_back(point_xyzir);
  ++cloud_xyzir_.width;

  cloud_xyziradt_.points.push_back(point_xyziradt);
  ++cloud_xyziradt_.width;
}
void ThreadSafeCloud::publish_cloud(
  const rclcpp::Time & _time,
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher_xyzir,
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher_xyziradt)
{
  if (cloud_xyzir_.empty() || cloud_xyziradt_.empty()) {
    return;
  }

  sensor_msgs::msg::PointCloud2 msg_xyzir;
  sensor_msgs::msg::PointCloud2 msg_xyziradt;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    pcl::toROSMsg(cloud_xyzir_, msg_xyzir);
    pcl::toROSMsg(cloud_xyziradt_, msg_xyziradt);

    cloud_xyzir_.clear();
    cloud_xyziradt_.clear();
    cloud_xyzir_.height = 1;
    cloud_xyziradt_.height = 1;
  }

  msg_xyzir.header.stamp = _time;
  msg_xyziradt.header.stamp = _time;

  if (_publisher_xyzir) {
    _publisher_xyzir->publish(msg_xyzir);
  }
  if (_publisher_xyziradt) {
    _publisher_xyziradt->publish(msg_xyziradt);
  }
}
}  // namespace velodyne_pointcloud
