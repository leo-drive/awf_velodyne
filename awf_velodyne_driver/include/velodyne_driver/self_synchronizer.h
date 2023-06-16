#pragma once

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <thread>

namespace velodyne_driver
{

class SelfSynchronizer
{
public:
  /**
   * @brief Constructor.
   * @param node The node pointer is for logging.
   */
  SelfSynchronizer(rclcpp::Node * node);

  /**
   * @brief Wait until a fixed calculated time.
   * @details The time is calculated based on the current time. The time is calculated as the
   *         next even millisecond. For example, if the current time is 1.2345, the next time will
   *         be 2.0000. If the current time is 2.2345, the next time will be 4.0000.
   */
  void sync();

  /**
   * @brief Checks the sync status.
   * @return True if the node is synced, false otherwise.
   */
  bool is_synchronized();

private:
  bool sync_status_{false};
  rclcpp::Node * node_;
};

}  // namespace velodyne_driver
