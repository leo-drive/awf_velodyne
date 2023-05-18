/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#include <velodyne_pointcloud/convert.h>

#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/pointcloudXYZIRADT.h>

#include <yaml-cpp/yaml.h>

#include <velodyne_pointcloud/output_builder.h>
#include <velodyne_pointcloud/func.h>

namespace velodyne_pointcloud
{

/** \brief For parameter service callback */
template <typename T>
bool get_param(const std::vector<rclcpp::Parameter> & p, const std::string & name, T & value)
{
  auto it = std::find_if(p.cbegin(), p.cend(), [&name](const rclcpp::Parameter & parameter) {
    return parameter.get_name() == name;
  });
  if (it != p.cend()) {
    value = it->template get_value<T>();
    return true;
  }
  return false;
}

inline std::chrono::nanoseconds toChronoNanoSeconds(const double seconds)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(seconds));
}

/** @brief Constructor. */
Convert::Convert(const rclcpp::NodeOptions & options)
: Node("velodyne_convert_node", options),
  // tf2_listener_(tf2_buffer_),
  num_points_threshold_(300),
  base_link_frame_("base_link")
{
  data_ = std::make_shared<velodyne_rawdata::RawData>(this);

  RCLCPP_INFO(this->get_logger(), "This node is only tested for VLP16, VLP32C, and VLS128. Use other models at your own risk.");

  // get path to angles.config file for this device
  std::string calibration_file = this->declare_parameter("calibration", "");

  rcl_interfaces::msg::ParameterDescriptor min_range_desc;
  min_range_desc.name = "min_range";
  min_range_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  min_range_desc.description = "minimum range to publish";
  rcl_interfaces::msg::FloatingPointRange min_range_range;
  min_range_range.from_value = 0.1;
  min_range_range.to_value = 10.0;
  min_range_desc.floating_point_range.push_back(min_range_range);
  config_.min_range = this->declare_parameter("min_range", 0.9, min_range_desc);

  rcl_interfaces::msg::ParameterDescriptor max_range_desc;
  max_range_desc.name = "max_range";
  max_range_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  max_range_desc.description = "maximum range to publish";
  rcl_interfaces::msg::FloatingPointRange max_range_range;
  max_range_range.from_value = 0.1;
  max_range_range.to_value = 250.0;
  max_range_desc.floating_point_range.push_back(max_range_range);
  config_.max_range = this->declare_parameter("max_range", 130.0, max_range_desc);

  rcl_interfaces::msg::ParameterDescriptor view_direction_desc;
  view_direction_desc.name = "view_direction";
  view_direction_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  view_direction_desc.description = "angle defining the center of view";
  rcl_interfaces::msg::FloatingPointRange view_direction_range;
  view_direction_range.from_value = -M_PI;
  view_direction_range.to_value = M_PI;
  view_direction_desc.floating_point_range.push_back(view_direction_range);
  config_.view_direction = this->declare_parameter("view_direction", 0.0, view_direction_desc);

  rcl_interfaces::msg::ParameterDescriptor view_width_desc;
  view_width_desc.name = "view_width";
  view_width_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  view_width_desc.description = "angle defining the view width";
  rcl_interfaces::msg::FloatingPointRange view_width_range;
  view_width_range.from_value = 0.0;
  view_width_range.to_value = 2.0 * M_PI;
  view_width_desc.floating_point_range.push_back(view_width_range);
  config_.view_width = this->declare_parameter("view_width", 2.0 * M_PI, view_width_desc);

  rcl_interfaces::msg::ParameterDescriptor num_points_threshold_desc;
  num_points_threshold_desc.name = "num_points_threshold";
  num_points_threshold_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
  num_points_threshold_desc.description = "num_points_threshold";
  rcl_interfaces::msg::IntegerRange num_points_threshold_range;
  num_points_threshold_range.from_value = 1;
  num_points_threshold_range.to_value = 10000;
  num_points_threshold_desc.integer_range.push_back(num_points_threshold_range);
  num_points_threshold_ = this->declare_parameter("num_points_threshold", 300, num_points_threshold_desc);

  rcl_interfaces::msg::ParameterDescriptor scan_phase_desc;
  scan_phase_desc.name = "scan_phase";
  scan_phase_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  scan_phase_desc.description = "start/end phase for the scan (in degrees)";
  rcl_interfaces::msg::FloatingPointRange scan_phase_range;
  scan_phase_range.from_value = 0.0;
  scan_phase_range.to_value = 359.0;
  scan_phase_desc.floating_point_range.push_back(scan_phase_range);
  config_.scan_phase = this->declare_parameter("scan_phase", 0.0, scan_phase_desc);

  const long scan_period = this->declare_parameter("scan_period", 100);
  config_.scan_period = std::make_unique<rclcpp::Duration>(std::chrono::milliseconds(scan_period));
  RCLCPP_INFO_STREAM(
    this->get_logger(), "Scan period: " << config_.scan_period->to_chrono<std::chrono::milliseconds>().count() << " ms.");

  RCLCPP_INFO(this->get_logger(), "correction angles: %s", calibration_file.c_str());

  data_->setup();
  data_->setParameters(
    config_.min_range, config_.max_range, config_.view_direction, config_.view_width);

  std::vector<double> invalid_intensity_double;
  invalid_intensity_double = this->declare_parameter<std::vector<double>>("invalid_intensity");
  // YAML::Node invalid_intensity_yaml = YAML::Load(invalid_intensity);
  invalid_intensity_array_ = std::vector<float>(data_->getNumLasers(), 0);
  for (size_t i = 0; i < invalid_intensity_double.size(); ++i) {
    invalid_intensity_array_.at(i) = static_cast<float>(invalid_intensity_double[i]);
  }

  // advertise
  velodyne_points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_points", rclcpp::SensorDataQoS());
  velodyne_points_ex_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_points_ex", rclcpp::SensorDataQoS());
  marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("velodyne_model_marker", 1);
  using std::placeholders::_1;
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&Convert::paramCallback, this, _1));


  // subscribe to VelodyneScan packets
  velodyne_scan_ =
    this->create_subscription<velodyne_msgs::msg::VelodyneScan>(
    "velodyne_packets", rclcpp::SensorDataQoS(),
    std::bind(&Convert::processScan, this, std::placeholders::_1));

  point_buffer_ = std::make_shared<velodyne_pointcloud::ThreadSafeCloud>("velodyne");
  pub_thread_ = std::thread(&Convert::periodicPublish, this);
}

rcl_interfaces::msg::SetParametersResult Convert::paramCallback(const std::vector<rclcpp::Parameter> & p)
{
  RCLCPP_INFO(this->get_logger(), "Reconfigure Request");

  if(get_param(p, "min_range", config_.min_range) ||
     get_param(p, "max_range", config_.max_range) ||
     get_param(p, "view_direction", config_.view_direction) ||
     get_param(p, "view_width", config_.view_width))
  {
    data_->setParameters(
      config_.min_range, config_.max_range, config_.view_direction, config_.view_width);
  }

  get_param(p, "num_points_threshold", num_points_threshold_);
  get_param(p, "scan_phase", config_.scan_phase);

  std::vector<double> invalid_intensity_double;
  auto it = std::find_if(p.cbegin(), p.cend(), [](const rclcpp::Parameter & parameter) {
    return parameter.get_name() == "invalid_intensity";
  });
  if (it != p.cend()) {
    invalid_intensity_double = it->as_double_array();
  }

  invalid_intensity_array_ = std::vector<float>(data_->getNumLasers(), 0);
  for (size_t i = 0; i < invalid_intensity_double.size(); ++i) {
    invalid_intensity_array_.at(i) = static_cast<float>(invalid_intensity_double[i]);
  }

  // if(get_param(p, "invalid_intensity", invalid_intensity))
  // {
    // YAML::Node invalid_intensity_yaml = YAML::Load(invalid_intensity);
    // invalid_intensity_array_ = std::vector<float>(data_->getNumLasers(), 0);
    // for (size_t i = 0; i < invalid_intensity_yaml.size(); ++i) {
    //   invalid_intensity_array_.at(i) = invalid_intensity_yaml[i].as<float>();
    // }
  // }
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  return result;
}

/** @brief Callback for raw scan messages. */
void Convert::processScan(const velodyne_msgs::msg::VelodyneScan::SharedPtr scanMsg)
{
  bool activate_xyziradt = velodyne_points_ex_pub_->get_subscription_count() > 0;
  bool activate_xyzir = velodyne_points_pub_->get_subscription_count() > 0;

  if (activate_xyziradt || activate_xyzir) {
    // Hold the time when the first packet has arrived.
    if (!next_pub_time_.has_value()) {
      // Self-sync nodes.
      // Get the current timestamp.
      const auto time_in_nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                     std::chrono::system_clock::now().time_since_epoch())
                                     .count();
      const auto next_pub_time = ((time_in_nanosec / 100000000) % 10 + 1) * 100000000;
      // Make it even number so that all nodes will have next even time. Bitwise operation is used
      // to prevent branching with if condition.
      const auto next_pub_time_even = (next_pub_time + 1) & ~1;
      const auto pub_time = time_in_nanosec - (time_in_nanosec % 1000000000) + next_pub_time_even;
      next_pub_time_.emplace(pub_time, this->get_clock()->get_clock_type());
      is_ready_to_pub_ = true;

      //      RCLCPP_INFO_STREAM(this->get_logger(), "Time: " << time_in_nanosec);
      //      RCLCPP_INFO_STREAM(this->get_logger(), "Next: " << pub_time << '\n');
    }

    // Unpack all the packets.
    for (const auto & packet : scanMsg->packets) {
      data_->unpack(packet, *point_buffer_);
    }
  }
}

void Convert::periodicPublish()
{
  while (rclcpp::ok() && !is_ready_to_pub_) {
    // Don't exit the thread.
  }

  while (rclcpp::ok() && is_ready_to_pub_) {
    // Publish points from buffer
    point_buffer_->publish_cloud(*next_pub_time_, velodyne_points_pub_, velodyne_points_ex_pub_);

    // Wait until next publish time.
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> sleep_until{
      std::chrono::nanoseconds{next_pub_time_->nanoseconds()}};
    std::this_thread::sleep_until(sleep_until);
    *next_pub_time_ += *config_.scan_period;
  }
}

visualization_msgs::msg::MarkerArray Convert::createVelodyneModelMakerMsg(
  const std_msgs::msg::Header & header)
{
  auto generatePoint = [](double x, double y, double z) {
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    return point;
  };

  auto generateQuaternion = [](double roll, double pitch, double yaw) {
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(roll, pitch, yaw);
    return tf2::toMsg(tf_quat);;
  };

  auto generateVector3 = [](double x, double y, double z) {
    geometry_msgs::msg::Vector3 vec;
    vec.x = x;
    vec.y = y;
    vec.z = z;
    return vec;
  };

  auto generateColor = [](float r, float g, float b, float a) {
    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
  };

  //array[0]:bottom body, array[1]:middle body(laser window), array[2]: top body, array[3]:cable
  const double radius = 0.1033;
  const std::array<geometry_msgs::msg::Point, 4> pos = {
    generatePoint(0.0, 0.0, -0.0285), generatePoint(0.0, 0.0, 0.0), generatePoint(0.0, 0.0, 0.0255),
    generatePoint(-radius / 2.0 - 0.005, 0.0, -0.03)};
  const std::array<geometry_msgs::msg::Quaternion, 4> quta = {
    generateQuaternion(0.0, 0.0, 0.0), generateQuaternion(0.0, 0.0, 0.0),
    generateQuaternion(0.0, 0.0, 0.0), generateQuaternion(0.0, M_PI_2, 0.0)};
  const std::array<geometry_msgs::msg::Vector3, 4> scale = {
    generateVector3(radius, radius, 0.020), generateVector3(radius, radius, 0.037),
    generateVector3(radius, radius, 0.015), generateVector3(0.0127, 0.0127, 0.02)};
  const std::array<std_msgs::msg::ColorRGBA, 4> color = {
    generateColor(0.85, 0.85, 0.85, 0.85), generateColor(0.1, 0.1, 0.1, 0.98),
    generateColor(0.85, 0.85, 0.85, 0.85), generateColor(0.2, 0.2, 0.2, 0.98)};

  visualization_msgs::msg::MarkerArray marker_array_msg;
  for (size_t i = 0; i < 4; ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = std::string(header.frame_id) + "_velodyne_model";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = pos[i];
    marker.pose.orientation = quta[i];
    marker.scale = scale[i];
    marker.color = color[i];
    marker_array_msg.markers.push_back(marker);
  }

  return marker_array_msg;
}

// looks like this function is never used
// bool Convert::getTransform(
//   const std::string & target_frame, const std::string & source_frame,
//   tf2::Transform * tf2_transform_ptr)
// {
//   if (target_frame == source_frame) {
//     tf2_transform_ptr->setOrigin(tf2::Vector3(0, 0, 0));
//     tf2_transform_ptr->setRotation(tf2::Quaternion(0, 0, 0, 1));
//     return true;
//   }

//   try {
//     const auto transform_msg =
//       tf2_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
//     tf2::convert(transform_msg.transform, *tf2_transform_ptr);
//   } catch (tf2::TransformException & ex) {
//     RCLCPP_WARN(this->get_logger(), "%s", ex.what());
//     RCLCPP_ERROR(this->get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

//     tf2_transform_ptr->setOrigin(tf2::Vector3(0, 0, 0));
//     tf2_transform_ptr->setRotation(tf2::Quaternion(0, 0, 0, 1));
//     return false;
//   }
//   return true;
// }

}  // namespace velodyne_pointcloud

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(velodyne_pointcloud::Convert)
