#include <velodyne_pointcloud/ThreadSafeCloud.h>

namespace velodyne_pointcloud
{
ThreadSafeCloud::ThreadSafeCloud(
  rclcpp::Node * _node, const std::string & _frame_id,
  const InvalidPointChecker & _invalid_point_checker, double _scan_phase,
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher_xyzir,
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _publisher_xyziradt,
  velodyne_rawdata::DataContainerBase & _overflow_buffer)
: invalid_point_checker_{_invalid_point_checker},
  node_{_node},
  scan_phase_{_scan_phase},
  publisher_xyzir_{_publisher_xyzir},
  publisher_xyziradt_{_publisher_xyziradt},
  overflow_buffer_{_overflow_buffer}
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
  // Check for invalid points
  if (invalid_point_checker_.is_invalid(ring, azimuth)) {
    return;
  }

  // Add the point to clouds
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

//  last_azimuth_int_ = azimuth;

  if (!azimuth_max_.has_value()) {
    azimuth_max_.emplace(azimuth);
  } else if (azimuth > *azimuth_max_) {
    *azimuth_max_ = azimuth;
  }

  if (!azimuth_min_.has_value()) {
    azimuth_min_.emplace(azimuth);
  } else if (azimuth < *azimuth_min_) {
    *azimuth_min_ = azimuth;
  }

  //  std::lock_guard<std::mutex> lock(mutex_);

  if (!check_azimuth_) {
    cloud_xyzir_.points.push_back(point_xyzir);
    ++cloud_xyzir_.width;

    cloud_xyziradt_.points.push_back(point_xyziradt);
    ++cloud_xyziradt_.width;
    return;
  }

  uint16_t azimuth_start;
  bool full_positive_region = false;
  if (*azimuth_max_ - *azimuth_min_ < 18000) {
    azimuth_start = *azimuth_max_;
    full_positive_region = true;
  } else {
    azimuth_start = *azimuth_min_;
  }

  if (azimuth < azimuth_start || (!full_positive_region && azimuth > *azimuth_max_)) {
    // The point belongs to the same scan. Add it to the cloud.
    cloud_xyzir_.points.push_back(point_xyzir);
    ++cloud_xyzir_.width;

    cloud_xyziradt_.points.push_back(point_xyziradt);
    ++cloud_xyziradt_.width;
  } else {
    // The point belongs to the next scan.
    overflow_buffer_.addPoint(x, y, z, return_type, ring, azimuth, distance, intensity, time_stamp);
  }

  // Publish cloud
  //  const auto first_azimuth{static_cast<uint16_t>(cloud_xyziradt_.points.front().azimuth)};
  //  const auto diff = std::abs(first_azimuth - azimuth);
  //  const auto diff = first_azimuth - azimuth;
  //  if (diff < 0) {
  //    azimuth_sign_ = AzimuthSign::NEGATIVE;
  //  }
  //  else {
  //    azimuth_sign_ = AzimuthSign::POSITIVE;
  //  }

  //  if (
  //    is_ready_to_publish_ && std::abs(diff) < 4 && azimuth_sign_ == AzimuthSign::NEGATIVE &&
  //    last_azimuth_.has_value()) {
  //    //    RCLCPP_INFO_STREAM(node_->get_logger(), "Publishing.");
  //    publish_cloud(pub_time_);
  //    azimuth_sign_ = AzimuthSign::POSITIVE;
  //  }
}
uint16_t ThreadSafeCloud::get_last_azimuth() { return last_azimuth_int_; }

void ThreadSafeCloud::publish_cloud(
  const rclcpp::Time & _time, const bool _called_from_different_thread)
{
  if (cloud_xyzir_.empty() || cloud_xyziradt_.empty()) {
    return;
  }

  sensor_msgs::msg::PointCloud2 msg_xyzir;
  sensor_msgs::msg::PointCloud2 msg_xyziradt;

  if (_called_from_different_thread) {
    std::lock_guard<std::mutex> lock(mutex_);
    pcl::toROSMsg(cloud_xyzir_, msg_xyzir);
    pcl::toROSMsg(cloud_xyziradt_, msg_xyziradt);

//    if (!last_azimuth_.has_value()) {
//      last_azimuth_.emplace(static_cast<uint16_t>(cloud_xyziradt_.points.back().azimuth));
//    } else {
//      *last_azimuth_ = static_cast<uint16_t>(cloud_xyziradt_.points.back().azimuth);
//    }
    cloud_xyzir_.clear();
    cloud_xyziradt_.clear();
    cloud_xyzir_.height = 1;
    cloud_xyziradt_.height = 1;
  } else {
    pcl::toROSMsg(cloud_xyzir_, msg_xyzir);
    pcl::toROSMsg(cloud_xyziradt_, msg_xyziradt);

//    if (!last_azimuth_.has_value()) {
//      last_azimuth_.emplace(static_cast<uint16_t>(cloud_xyziradt_.points.back().azimuth));
//    } else {
//      *last_azimuth_ = static_cast<uint16_t>(cloud_xyziradt_.points.back().azimuth);
//    }
    cloud_xyzir_.clear();
    cloud_xyziradt_.clear();
    cloud_xyzir_.height = 1;
    cloud_xyziradt_.height = 1;
  }

  msg_xyzir.header.stamp = _time;
  msg_xyziradt.header.stamp = _time;

  if (publisher_xyzir_) {
    publisher_xyzir_->publish(msg_xyzir);
  }
  if (publisher_xyziradt_) {
    publisher_xyziradt_->publish(msg_xyziradt);
  }

  check_azimuth_ = false;  // Put this in the thread ifs?
}
void ThreadSafeCloud::set_pub_time(rclcpp::Time & _time)
{
  pub_time_ = _time;
  RCLCPP_INFO_STREAM(node_->get_logger(), "pub_time: " << pub_time_.nanoseconds());
}

void ThreadSafeCloud::ready_to_publish()
{
  const auto noww = node_->now();
  publish_cloud(noww, true);
  is_ready_to_publish_ = true;

  RCLCPP_INFO_STREAM(this->node_->get_logger(), "First time pub: " << noww.nanoseconds());
}

void ThreadSafeCloud::enable_azimuth_check() { check_azimuth_ = true; }

}  // namespace velodyne_pointcloud
