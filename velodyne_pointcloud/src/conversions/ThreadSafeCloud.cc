#include <velodyne_pointcloud/ThreadSafeCloud.h>

namespace velodyne_pointcloud
{
ThreadSafeCloud::ThreadSafeCloud(const std::string & _frame_id)
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
  if (distance < 1) {
    return;
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
  sensor_msgs::msg::PointCloud2 msg_xyzir;
  sensor_msgs::msg::PointCloud2 msg_xyziradt;

  {
    std::lock_guard<std::mutex> lock(mutex_);
    pcl::toROSMsg(cloud_xyzir_, msg_xyzir);
    pcl::toROSMsg(cloud_xyziradt_, msg_xyziradt);
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
