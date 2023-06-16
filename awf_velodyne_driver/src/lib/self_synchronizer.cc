#include <velodyne_driver/self_synchronizer.h>

namespace velodyne_driver
{
SelfSynchronizer::SelfSynchronizer(rclcpp::Node * node) : node_{node} {}

void SelfSynchronizer::sync()
{
  // Get the current timestamp.
  const auto time_in_nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                 std::chrono::system_clock::now().time_since_epoch())
                                 .count();
  const auto next_pub_time = ((time_in_nanosec / 100000000) % 10 + 1) * 100000000;

  // Make it even number so that all nodes will have next even time. Bitwise operation is used
  // to prevent branching with if condition.
  const auto next_pub_time_even = (next_pub_time + 1) & ~1;
  const auto pub_time = time_in_nanosec - (time_in_nanosec % 1000000000) + next_pub_time_even;
  rclcpp::Time first_pub_time(pub_time, node_->get_clock()->get_clock_type());

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Time: " << time_in_nanosec);
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Next: " << pub_time << '\n');

  sync_status_ = true;

  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> sleep_until{
    std::chrono::nanoseconds{first_pub_time.nanoseconds()}};
  std::this_thread::sleep_until(sleep_until);
  RCLCPP_DEBUG_STREAM(node_->get_logger(), "Woke up.");
}
bool SelfSynchronizer::is_synchronized() { return sync_status_; }
}  // namespace velodyne_driver